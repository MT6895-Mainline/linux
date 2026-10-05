// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MIPI CSI-2 receiver C/D-PHY v3.0 driver
 *
 * Copyright (c) 2019 MediaTek Inc.
 * Copyright (c) 2023, MediaTek Inc.
 * Copyright (c) 2023, BayLibre Inc.
 * Copyright (c) 2025 MediaTek Inc.
 *
 * The register programming is MediaTek's camsys PHY 3.0 sequence
 * (mtk_cam-seninf-hw_phy_3_0.c in MiCode/Xiaomi_Kernel_OpenSource
 * 270b84910b941bf75490fccdb5fb2b3021dd7cbf) for D-PHY with __SMT == 0 and
 * legacy_phy == 0, in the order verified on the Xiaomi Redmi K50 (CSI3, one
 * data lane at 672 Mbit/s). The generic PHY part follows
 * phy-mtk-mipi-csi-0-5.c and MediaTek's Genio phy-mtk-mipi-csi-2-1.c.
 *
 * Each CSI port has a 16 KiB register block: two analog halves (A and B) of
 * three pairs each, a D-PHY and a C-PHY digital front end. Only D-PHY over
 * both halves is supported (4D1C: the clock lane on pair L2 of half A and up
 * to four data lanes in the vendor's fixed order).
 *
 * The block is only accessible while the SENINF power domains and clocks are
 * on. The consumer, the SENINF driver, calls phy_configure(), phy_power_on()
 * and phy_power_off() only in that state, so the registers are accessed there
 * and, while powered, by the debugfs dump; never at probe or validate time.
 * As in the vendor sequence, phy_configure() applies the analog defaults and
 * the D-PHY timing before the consumer enables its CSI-2 receiver, and
 * phy_power_on() then powers the analog halves up and enables the lanes.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/units.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-csi-3-0-rx-reg.h"

#define MTK_CSI_RX_DATA_LANES		4

/* Vendor constants; the settle and trail counters run on SENINF_CK. */
#define MTK_CSI_RX_COUNTER_HZ		(273 * HZ_PER_MHZ)
#define MTK_CSI_RX_SETTLE_CYCLES	0x1c	/* DPHY_SETTLE, 102.6 ns */
#define MTK_CSI_RX_TRAIL_UI		224	/* DPHY_TRAIL_SPEC */
#define MTK_CSI_RX_DPHY_SPARE0		0xf1	/* legacy_phy == 0 */

/*
 * Per-lane bit rates up to the end of the lowest EQ band of the vendor table,
 * the only band verified. From the minimum up, the 8-bit HS trail counter
 * covers the 224 UI trail window whatever the transmitter trail; the D-PHY
 * minimum of 80 Mbit/s would need up to 765 cycles.
 */
#define MTK_CSI_RX_MIN_RATE		240000000ULL
#define MTK_CSI_RX_MAX_RATE		2500000000ULL	/* exclusive */

/* Offset calibration enables, in the vendor order */
static const u32 mtk_csi_rx_os_cal_en[] = {
	RG_CSI0_L0_T0AB_EQ_OS_CAL_EN,
	RG_CSI0_L1_T1AB_EQ_OS_CAL_EN,
	RG_CSI0_L2_T1BC_EQ_OS_CAL_EN,
	RG_CSI0_XX_T0BC_EQ_OS_CAL_EN,
	RG_CSI0_XX_T0CA_EQ_OS_CAL_EN,
	RG_CSI0_XX_T1CA_EQ_OS_CAL_EN,
};

/*
 * Registers recorded at the first configuration of this driver binding, for
 * the debugfs dump. They are the reset values only if nothing powered the PHY
 * since boot, as a reloaded module finds the state of the last power-off.
 */
static const u16 mtk_csi_rx_initial_regs[] = {
	MTK_CSI_RX_ANA_A + CDPHY_RX_ANA_0,
	MTK_CSI_RX_ANA_B + CDPHY_RX_ANA_0,
	MTK_CSI_RX_DPHY + DPHY_RX_LANE_EN,
	MTK_CSI_RX_DPHY + DPHY_RX_LANE_SELECT,
	MTK_CSI_RX_DPHY + DPHY_RX_SPARE0,
};

struct mtk_csi_rx_cfg {
	u64 hs_clk_rate;
	u8 lanes;
	u8 data_settle;
	u8 clk_settle;
	u8 trail;		/* 0 disables the trail counter */
};

struct mtk_csi_rx {
	struct device *dev;
	void __iomem *base;
	struct phy *phy;
	struct mutex lock;	/* cfg, flags, initial_val and register access */
	struct mtk_csi_rx_cfg cfg;
	bool configured;
	bool powered;
	bool initial_read;
	u32 initial_val[ARRAY_SIZE(mtk_csi_rx_initial_regs)];
};

/* csirx_phyA_init() for one half */
static void mtk_csi_rx_ana_init(void __iomem *ana)
{
	mtk_phy_update_field(ana + CDPHY_RX_ANA_1, RG_CSI0_BG_LPRX_VTL_SEL, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_1, RG_CSI0_BG_LPRX_VTH_SEL, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_BG_ALP_RX_VTL_SEL, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_BG_ALP_RX_VTH_SEL, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_1, RG_CSI0_BG_VREF_SEL, 8);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_1, RG_CSI0_CDPHY_EQ_DES_VREF_SEL, 2);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 3);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 1);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 1);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 1);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 1);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0);
	/* Termination ("r50"); MT6895 does not trim it from efuses */
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_L0P_T0A_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_L0N_T0B_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_3, RG_CSI0_L1P_T0C_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_3, RG_CSI0_L1N_T1A_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_4, RG_CSI0_L2P_T1B_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_4, RG_CSI0_L2N_T1C_HSRT_CODE, 0x10);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_0, RG_CSI0_CPHY_T0_CDR_FIRST_EDGE_EN, 0);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_0, RG_CSI0_CPHY_T1_CDR_FIRST_EDGE_EN, 0);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_CPHY_T0_CDR_SELF_CAL_EN, 0);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_2, RG_CSI0_CPHY_T1_CDR_SELF_CAL_EN, 0);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_CK_DELAY, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_CK_DELAY, 4);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_AB_WIDTH, 9);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_BC_WIDTH, 9);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_CA_WIDTH, 9);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_AB_WIDTH, 9);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_BC_WIDTH, 9);
	mtk_phy_update_field(ana + CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_CA_WIDTH, 9);
}

/* csirx_dphy_init(), lane by lane; all data lanes and both clock lanes */
static void mtk_csi_rx_dphy_timing(void __iomem *dphy, const struct mtk_csi_rx_cfg *cfg)
{
	unsigned int i;

	for (i = 0; i < MTK_CSI_RX_DATA_LANES; i++) {
		void __iomem *reg = dphy + DPHY_RX_DATA_LANE_HS_PARAMETER(i);

		mtk_phy_update_field(reg, RG_CDPHY_RX_LD_HS_SETTLE_PARAMETER, cfg->data_settle);
		mtk_phy_update_field(reg, RG_CDPHY_RX_LD_HS_PREPARE_PARAMETER, 0);
		mtk_phy_update_field(reg, RG_DPHY_RX_LD_HS_TRAIL_PARAMETER, cfg->trail);
		mtk_phy_update_field(reg, RG_DPHY_RX_LD_HS_TRAIL_EN, cfg->trail != 0);
	}
	for (i = 0; i < 2; i++)
		mtk_phy_update_field(dphy + DPHY_RX_CLOCK_LANE_HS_PARAMETER(i),
				     RG_DPHY_RX_LC_HS_SETTLE_PARAMETER, cfg->clk_settle);
}

/* csirx_phyA_setting() for D-PHY 4D1C, one half */
static void mtk_csi_rx_ana_dphy(void __iomem *ana, bool clock_half)
{
	void __iomem *ana0 = ana + CDPHY_RX_ANA_0;
	void __iomem *ana5 = ana + CDPHY_RX_ANA_5;

	mtk_phy_update_field(ana0, RG_CSI0_CPHY_EN, 0);
	/* Clear the clock modes, select the pair clocks, set the clock pair */
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L2_CKSEL, 1);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(ana0, RG_CSI0_DPHY_L2_CKMODE_EN, clock_half);
	mtk_phy_update_field(ana0, RG_CSI0_CPHY_T0_HSMODE_EN, 1);
	mtk_phy_update_field(ana0, RG_CSI0_CPHY_T1_HSMODE_EN, 1);
	/* EQ for per-lane rates below 2.5 Gbit/s */
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_SR1, 0);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_SR0, 0);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_LATCH_EN, 1);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_DG1_EN, 0);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_DG0_EN, 0);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_IS, 1);
	mtk_phy_update_field(ana5, RG_CSI0_CDPHY_EQ_BW, 0);
}

/* csirx_phyA_power_on(): power one half down, then optionally up */
static void mtk_csi_rx_ana_power(void __iomem *ana, bool on)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_csi_rx_os_cal_en); i++)
		mtk_phy_clear_bits(ana + CDPHY_RX_ANA_8, mtk_csi_rx_os_cal_en[i]);
	mtk_phy_clear_bits(ana + CDPHY_RX_ANA_0, RG_CSI0_BG_LPF_EN);
	mtk_phy_clear_bits(ana + CDPHY_RX_ANA_0, RG_CSI0_BG_CORE_EN);
	fsleep(200);
	if (!on)
		return;

	mtk_phy_set_bits(ana + CDPHY_RX_ANA_0, RG_CSI0_BG_CORE_EN);
	fsleep(30);
	mtk_phy_set_bits(ana + CDPHY_RX_ANA_0, RG_CSI0_BG_LPF_EN);
	udelay(1);
	for (i = 0; i < ARRAY_SIZE(mtk_csi_rx_os_cal_en); i++)
		mtk_phy_set_bits(ana + CDPHY_RX_ANA_8, mtk_csi_rx_os_cal_en[i]);
	udelay(1);
}

/* csirx_dphy_setting() for 4D1C */
static void mtk_csi_rx_dphy_lanes(void __iomem *dphy)
{
	void __iomem *sel = dphy + DPHY_RX_LANE_SELECT;
	void __iomem *en = dphy + DPHY_RX_LANE_EN;

	/*
	 * Pads 0-5 are the pairs L0-L2 of half A, then of half B: data lanes
	 * 0-3 on A.L1, B.L0, A.L0 and B.L1, the clock lane on A.L2.
	 */
	mtk_phy_update_field(sel, RG_DPHY_RX_LD3_SEL, 4);
	mtk_phy_update_field(sel, RG_DPHY_RX_LD2_SEL, 0);
	mtk_phy_update_field(sel, RG_DPHY_RX_LD1_SEL, 3);
	mtk_phy_update_field(sel, RG_DPHY_RX_LD0_SEL, 1);
	mtk_phy_update_field(sel, RG_DPHY_RX_LC0_SEL, 2);
	/* All data lanes; the SENINF CSI-2 receiver selects the used ones */
	mtk_phy_set_bits(en, DPHY_RX_LD0_EN);
	mtk_phy_set_bits(en, DPHY_RX_LD1_EN);
	mtk_phy_set_bits(en, DPHY_RX_LD2_EN);
	mtk_phy_set_bits(en, DPHY_RX_LD3_EN);
	mtk_phy_set_bits(en, DPHY_RX_LC0_EN);
	mtk_phy_clear_bits(en, DPHY_RX_LC1_EN);
	mtk_phy_set_bits(sel, DPHY_RX_CK_DATA_MUX_EN);
	writel(MTK_CSI_RX_DPHY_SPARE0, dphy + DPHY_RX_SPARE0);
}

static int mtk_csi_rx_power_on(struct phy *phy)
{
	struct mtk_csi_rx *rx = phy_get_drvdata(phy);
	void __iomem *ana_a = rx->base + MTK_CSI_RX_ANA_A;
	void __iomem *ana_b = rx->base + MTK_CSI_RX_ANA_B;
	void __iomem *afifo = ana_a + CDPHY_RX_ANA_SETTING_1;

	guard(mutex)(&rx->lock);
	/* Every power-on follows its own phy_configure(). */
	if (!rx->configured || rx->powered)
		return -EINVAL;

	/* csirx_phy_setting(): async FIFO of the port (in half A), mode, power, lanes */
	mtk_phy_update_field(afifo, RG_AFIFO_DUMMY_VALID_EN, 1);
	mtk_phy_update_field(afifo, RG_CSI0_ASYNC_OPTION, 5);
	mtk_phy_update_field(afifo, RG_AFIFO_DUMMY_VALID_PREPARE_NUM, 4);
	mtk_phy_update_field(afifo, RG_AFIFO_DUMMY_VALID_NUM, 1);
	mtk_csi_rx_ana_dphy(ana_a, true);
	mtk_csi_rx_ana_dphy(ana_b, false);
	mtk_csi_rx_ana_power(ana_a, true);
	mtk_csi_rx_ana_power(ana_b, true);
	mtk_csi_rx_dphy_lanes(rx->base + MTK_CSI_RX_DPHY);
	rx->powered = true;

	dev_dbg(rx->dev, "on: %llu bit/s, %u lanes, settle %u/%u, trail %u\n",
		rx->cfg.hs_clk_rate, rx->cfg.lanes, rx->cfg.data_settle,
		rx->cfg.clk_settle, rx->cfg.trail);
	return 0;
}

static int mtk_csi_rx_power_off(struct phy *phy)
{
	struct mtk_csi_rx *rx = phy_get_drvdata(phy);

	guard(mutex)(&rx->lock);
	/* mtk_cam_seninf_poweroff() */
	mtk_csi_rx_ana_power(rx->base + MTK_CSI_RX_ANA_A, false);
	mtk_csi_rx_ana_power(rx->base + MTK_CSI_RX_ANA_B, false);
	/* Not in the vendor code: every power-on starts with the lanes disabled */
	mtk_phy_clear_bits(rx->base + MTK_CSI_RX_DPHY + DPHY_RX_LANE_EN,
			   DPHY_RX_LD0_EN | DPHY_RX_LD1_EN | DPHY_RX_LD2_EN |
			   DPHY_RX_LD3_EN | DPHY_RX_LC0_EN | DPHY_RX_LC1_EN);
	rx->powered = false;
	rx->configured = false;
	return 0;
}

/* Cycles of the settle and trail counter clock in @ps, rounded up */
static u64 mtk_csi_rx_cycles(u64 ps)
{
	return mul_u64_u64_div_u64_roundup(ps, MTK_CSI_RX_COUNTER_HZ, PSEC_PER_SEC);
}

/* Validate @opts and derive the register values, without register access */
static int mtk_csi_rx_compute(struct phy_configure_opts_mipi_dphy *opts,
			      struct mtk_csi_rx_cfg *cfg)
{
	unsigned int tx_trail_ps = opts->hs_trail;
	u64 rate = opts->hs_clk_rate;
	u64 data_settle, clk_settle, window_ps, trail = 0;
	u32 rate_mbps;
	int ret;

	/* The generic validator and the trail window divide by the rate. */
	if (rate < MTK_CSI_RX_MIN_RATE || rate >= MTK_CSI_RX_MAX_RATE)
		return -ERANGE;
	if (opts->lanes < 1 || opts->lanes > MTK_CSI_RX_DATA_LANES)
		return -EINVAL;
	ret = phy_mipi_dphy_config_validate(opts);
	if (ret)
		return ret;

	/* The vendor's fixed settle, raised where the D-PHY minimum needs more */
	data_settle = max_t(u64, MTK_CSI_RX_SETTLE_CYCLES, mtk_csi_rx_cycles(opts->hs_settle));
	clk_settle = max_t(u64, MTK_CSI_RX_SETTLE_CYCLES, mtk_csi_rx_cycles(opts->clk_settle));

	/*
	 * Vendor rule for sensors with a fixed trail: the counter covers what
	 * the transmitter's T_HS-TRAIL leaves of a 224 UI window. It is off when
	 * that trail is unknown or not shorter than the window. As in the vendor
	 * code, whose counters this reproduces, the window is in whole ns of the
	 * rate in whole Mbit/s.
	 */
	rate_mbps = div_u64(rate, HZ_PER_MHZ);
	window_ps = (u64)(MTK_CSI_RX_TRAIL_UI * NSEC_PER_USEC / rate_mbps) * PSEC_PER_NSEC;
	if (tx_trail_ps && tx_trail_ps < window_ps)
		trail = mtk_csi_rx_cycles(window_ps - tx_trail_ps);

	if (data_settle > U8_MAX || clk_settle > U8_MAX || trail > U8_MAX)
		return -ERANGE;

	cfg->hs_clk_rate = rate;
	cfg->lanes = opts->lanes;
	cfg->data_settle = data_settle;
	cfg->clk_settle = clk_settle;
	cfg->trail = trail;
	return 0;
}

static int mtk_csi_rx_validate(struct phy *phy, enum phy_mode mode, int submode,
			       union phy_configure_opts *opts)
{
	struct mtk_csi_rx_cfg cfg;

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return mtk_csi_rx_compute(&opts->mipi_dphy, &cfg);
}

/*
 * csirx_phy_init(): analog defaults, D-PHY timing and the C-PHY post filter,
 * written before the consumer enables its CSI-2 receiver. The consumer keeps
 * the block powered and calls this before each phy_power_on().
 */
static int mtk_csi_rx_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct mtk_csi_rx *rx = phy_get_drvdata(phy);
	struct mtk_csi_rx_cfg cfg;
	unsigned int i;
	int ret;

	ret = mtk_csi_rx_compute(&opts->mipi_dphy, &cfg);
	if (ret) {
		dev_err(rx->dev, "unsupported configuration: %lu bit/s on %u lanes: %d\n",
			opts->mipi_dphy.hs_clk_rate, opts->mipi_dphy.lanes, ret);
		return ret;
	}

	guard(mutex)(&rx->lock);
	/* Not while the lanes run */
	if (rx->powered)
		return -EBUSY;

	if (!rx->initial_read) {
		for (i = 0; i < ARRAY_SIZE(mtk_csi_rx_initial_regs); i++)
			rx->initial_val[i] = readl(rx->base + mtk_csi_rx_initial_regs[i]);
		rx->initial_read = true;
	}

	mtk_csi_rx_ana_init(rx->base + MTK_CSI_RX_ANA_A);
	mtk_csi_rx_ana_init(rx->base + MTK_CSI_RX_ANA_B);
	mtk_csi_rx_dphy_timing(rx->base + MTK_CSI_RX_DPHY, &cfg);
	mtk_phy_update_field(rx->base + MTK_CSI_RX_CPHY + CPHY_RX_DETECT_CTRL_POST,
			     RG_CPHY_RX_DATA_VALID_POST_EN, 1);

	rx->cfg = cfg;
	rx->configured = true;
	return 0;
}

static int mtk_csi_rx_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	return mode == PHY_MODE_MIPI_DPHY ? 0 : -EINVAL;
}

static const struct phy_ops mtk_csi_rx_ops = {
	.power_on	= mtk_csi_rx_power_on,
	.power_off	= mtk_csi_rx_power_off,
	.set_mode	= mtk_csi_rx_set_mode,
	.configure	= mtk_csi_rx_configure,
	.validate	= mtk_csi_rx_validate,
	.owner		= THIS_MODULE,
};

static struct phy *mtk_csi_rx_xlate(struct device *dev, const struct of_phandle_args *args)
{
	struct mtk_csi_rx *rx = dev_get_drvdata(dev);

	if (args->args_count != 1) {
		dev_err(dev, "expected one PHY cell, got %d\n", args->args_count);
		return ERR_PTR(-EINVAL);
	}

	switch (args->args[0]) {
	case PHY_TYPE_DPHY:
		return rx->phy;
	case PHY_TYPE_CPHY:
		dev_err(dev, "C-PHY mode is not supported\n");
		return ERR_PTR(-EOPNOTSUPP);
	default:
		dev_err(dev, "invalid PHY type %u\n", args->args[0]);
		return ERR_PTR(-EINVAL);
	}
}

/* The register set of the vendor's debug dump, plus the lane state machines */
static const u16 mtk_csi_rx_ana_dump[] = {
	CDPHY_RX_ANA_0, CDPHY_RX_ANA_1, CDPHY_RX_ANA_2, CDPHY_RX_ANA_3,
	CDPHY_RX_ANA_4, CDPHY_RX_ANA_5, CDPHY_RX_ANA_6, CDPHY_RX_ANA_7,
	CDPHY_RX_ANA_8, CDPHY_RX_ANA_AD_0, CDPHY_RX_ANA_AD_HS_0,
	CDPHY_RX_ANA_AD_HS_1, CDPHY_RX_ANA_SETTING_0, CDPHY_RX_ANA_SETTING_1,
};

static const u16 mtk_csi_rx_dphy_dump[] = {
	DPHY_RX_LANE_EN, DPHY_RX_LANE_SELECT,
	DPHY_RX_CLOCK_LANE_HS_PARAMETER(0), DPHY_RX_CLOCK_LANE_HS_PARAMETER(1),
	DPHY_RX_DATA_LANE_HS_PARAMETER(0), DPHY_RX_DATA_LANE_HS_PARAMETER(1),
	DPHY_RX_DATA_LANE_HS_PARAMETER(2), DPHY_RX_DATA_LANE_HS_PARAMETER(3),
	DPHY_RX_CLOCK_LANE_FSM, DPHY_RX_DATA_LANE_FSM, DPHY_RX_SPARE0,
};

static const u16 mtk_csi_rx_cphy_dump[] = {
	CPHY_RX_CTRL, CPHY_RX_DETECT_CTRL_POST,
};

static void mtk_csi_rx_dump(struct seq_file *s, void __iomem *base, unsigned int block,
			    const u16 *regs, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		seq_printf(s, "%04x: %08x\n", block + regs[i], readl(base + block + regs[i]));
}

static int mtk_csi_rx_regs_show(struct seq_file *s, void *unused)
{
	struct mtk_csi_rx *rx = s->private;
	unsigned int i;

	guard(mutex)(&rx->lock);
	/* The registers are only accessible while the consumer keeps them powered. */
	if (!rx->powered)
		return -EBUSY;

	seq_printf(s, "rate %llu lanes %u settle %u/%u trail %u\n", rx->cfg.hs_clk_rate,
		   rx->cfg.lanes, rx->cfg.data_settle, rx->cfg.clk_settle, rx->cfg.trail);
	for (i = 0; i < ARRAY_SIZE(mtk_csi_rx_initial_regs); i++)
		seq_printf(s, "initial %04x: %08x\n", mtk_csi_rx_initial_regs[i],
			   rx->initial_val[i]);
	mtk_csi_rx_dump(s, rx->base, MTK_CSI_RX_ANA_A, mtk_csi_rx_ana_dump,
			ARRAY_SIZE(mtk_csi_rx_ana_dump));
	mtk_csi_rx_dump(s, rx->base, MTK_CSI_RX_ANA_B, mtk_csi_rx_ana_dump,
			ARRAY_SIZE(mtk_csi_rx_ana_dump));
	mtk_csi_rx_dump(s, rx->base, MTK_CSI_RX_DPHY, mtk_csi_rx_dphy_dump,
			ARRAY_SIZE(mtk_csi_rx_dphy_dump));
	mtk_csi_rx_dump(s, rx->base, MTK_CSI_RX_CPHY, mtk_csi_rx_cphy_dump,
			ARRAY_SIZE(mtk_csi_rx_cphy_dump));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_csi_rx_regs);

static void mtk_csi_rx_debugfs_remove(void *dentry)
{
	debugfs_remove(dentry);
}

static int mtk_csi_rx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct mtk_csi_rx *rx;
	struct dentry *regs;
	int ret;

	rx = devm_kzalloc(dev, sizeof(*rx), GFP_KERNEL);
	if (!rx)
		return -ENOMEM;
	rx->dev = dev;
	platform_set_drvdata(pdev, rx);

	ret = devm_mutex_init(dev, &rx->lock);
	if (ret)
		return ret;

	/* Mapping only: the block may be unpowered here. */
	rx->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rx->base))
		return PTR_ERR(rx->base);

	rx->phy = devm_phy_create(dev, NULL, &mtk_csi_rx_ops);
	if (IS_ERR(rx->phy))
		return dev_err_probe(dev, PTR_ERR(rx->phy), "failed to create PHY\n");
	phy_set_drvdata(rx->phy, rx);

	/* Removed before the PHY and the mapping go away */
	regs = debugfs_create_file("regs", 0400, rx->phy->debugfs, rx, &mtk_csi_rx_regs_fops);
	ret = devm_add_action_or_reset(dev, mtk_csi_rx_debugfs_remove, regs);
	if (ret)
		return ret;

	provider = devm_of_phy_provider_register(dev, mtk_csi_rx_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider), "failed to register PHY provider\n");

	return 0;
}

static const struct of_device_id mtk_csi_rx_of_match[] = {
	{ .compatible = "mediatek,mt6895-csi-rx" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_csi_rx_of_match);

static struct platform_driver mtk_csi_rx_driver = {
	.probe = mtk_csi_rx_probe,
	.driver = {
		.name = "phy-mtk-mipi-csi-3-0",
		.of_match_table = mtk_csi_rx_of_match,
		/* Unbinding would remove the consumer under a running stream. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_csi_rx_driver);

MODULE_DESCRIPTION("MediaTek MIPI CSI-2 receiver C/D-PHY v3.0 driver");
MODULE_LICENSE("GPL");
