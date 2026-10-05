// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6895 SENINF register programming
 *
 * Copyright (c) 2020 MediaTek Inc.
 *
 * The sequences follow MediaTek's GPL-2.0 ISP 7.1 driver in
 * MiCode/Xiaomi_Kernel_OpenSource 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/media/platform/mtk-isp/camsys/isp7_1/cam/mtk_cam-seninf-hw_phy_3_0.c,
 * in the order that captured frames on MT6895 (CSI3, one D-PHY lane, RAW10).
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#include "mtk_seninf71.h"
#include "mtk_seninf71_regs.h"

#define MTK_SENINF_CAM_MUX_ROUTE	(RG_SENINF_CAM_MUX_PCSR_SRC_SEL | \
					 RG_SENINF_CAM_MUX_PCSR_EN | \
					 RG_SENINF_CAM_MUX_PCSR_CHK_PIX_MODE | \
					 RG_SENINF_CAM_MUX_PCSR_CHK_EN | \
					 CAM_MUX_PCSR_NEXT_SRC_SEL | \
					 RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN0 | \
					 RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN1)
#define MTK_SENINF_CAM_MUX_FILTER	(RG_SENINF_CAM_MUX_PCSR_VC_SEL | \
					 RG_SENINF_CAM_MUX_PCSR_VC_SEL_EN | \
					 RG_SENINF_CAM_MUX_PCSR_DT_SEL | \
					 RG_SENINF_CAM_MUX_PCSR_DT_SEL_EN)
#define MTK_SENINF_CAM_MUX_IRQS		(RO_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ | \
					 RO_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ | \
					 RO_SENINF_CAM_MUX_PCSR_VSYNC_IRQ)
#define MTK_SENINF_GCSR_DYN_BUSY	(RG_SENINF_CAM_MUX_GCSR_DYN_SWITCH_BSY0 | \
					 RG_SENINF_CAM_MUX_GCSR_DYN_SWITCH_BSY1 | \
					 RG_SENINF_CAM_MUX_GCSR_DYN_PAGE_SEL | \
					 RG_SENINF_CAM_MUX_GCSR_OCC_PG1_EN)

static u32 mtk_seninf_read(struct mtk_seninf *priv, u32 reg)
{
	return readl(priv->base + reg);
}

static void mtk_seninf_write(struct mtk_seninf *priv, u32 reg, u32 val)
{
	writel(val, priv->base + reg);
}

static void mtk_seninf_update(struct mtk_seninf *priv, u32 reg, u32 mask, u32 val)
{
	u32 tmp = readl(priv->base + reg);

	writel((tmp & ~mask) | (val & mask), priv->base + reg);
}

static int mtk_seninf_check_idle(struct mtk_seninf *priv, u32 reg, u32 mask)
{
	u32 val = mtk_seninf_read(priv, reg);

	if (!(val & mask))
		return 0;

	dev_err(priv->dev, "register %#06x is in use (%#x)\n", reg, val);
	return -EBUSY;
}

/*
 * A mux that any other CAM_MUX selects is taken, even while that CAM_MUX is
 * disabled: after power-up every CAM_MUX is enabled and selects mux 0.
 */
static int mtk_seninf_mark_mux(struct mtk_seninf *priv, unsigned long *used,
			       unsigned int cammux, unsigned int mux)
{
	if (mux == SENINF_CAM_MUX_SRC_NONE)
		return 0;

	if (mux >= priv->soc->num_mux) {
		dev_err(priv->dev, "CAM_MUX %u selects mux %u\n", cammux, mux);
		return -EBUSY;
	}

	*used |= BIT(mux);
	return 0;
}

/* Check that the path is idle and pick the mux for it. Reads only. */
static int mtk_seninf_route_check(struct mtk_seninf *priv, unsigned int intf,
				  unsigned int cammux)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	u32 base = SENINF_INTF_BASE(intf), csi2 = SENINF_CSI2_BASE(intf);
	u32 gcsr = SENINF_CAM_MUX_GCSR_BASE;
	u32 cammux_all = GENMASK(soc->num_cammux - 1, 0);
	unsigned long used = priv->mux_busy;
	unsigned int i;
	u32 ctrl;
	int ret;

	ret = mtk_seninf_check_idle(priv, base + SENINF_CTRL, SENINF_EN) ?:
	      mtk_seninf_check_idle(priv, base + SENINF_CSI2_CTRL, RG_SENINF_CSI2_EN) ?:
	      mtk_seninf_check_idle(priv, base + SENINF_TESTMDL_CTRL, RG_SENINF_TESTMDL_EN) ?:
	      mtk_seninf_check_idle(priv, csi2 + SENINF_CSI2_EN, U32_MAX) ?:
	      mtk_seninf_check_idle(priv, csi2 + SENINF_CSI2_IRQ_EN, RG_CSI2_IRQ_EN_ALL) ?:
	      mtk_seninf_check_idle(priv, SENINF_TOP_PHY_CTRL_CSI(intf / 2),
				    PHY_SENINF_DPHY_EN | PHY_SENINF_CPHY_EN) ?:
	      mtk_seninf_check_idle(priv, SENINF_CAM_MUX_PCSR_BASE(cammux) +
				    SENINF_CAM_MUX_PCSR_IRQ_EN, U32_MAX) ?:
	      mtk_seninf_check_idle(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_CTRL,
				    MTK_SENINF_GCSR_DYN_BUSY) ?:
	      mtk_seninf_check_idle(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_EN0, cammux_all) ?:
	      mtk_seninf_check_idle(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_EN1, cammux_all);
	if (ret)
		return ret;

	for (i = 0; i < soc->num_cammux; i++) {
		ctrl = mtk_seninf_read(priv, SENINF_CAM_MUX_PCSR_BASE(i) +
				       SENINF_CAM_MUX_PCSR_CTRL);
		if (ctrl & (RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN0 |
			    RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN1)) {
			dev_err(priv->dev, "CAM_MUX %u switches dynamically (%#x)\n", i, ctrl);
			return -EBUSY;
		}

		if (i == cammux)
			continue;

		ret = mtk_seninf_mark_mux(priv, &used, i,
					  FIELD_GET(RG_SENINF_CAM_MUX_PCSR_SRC_SEL, ctrl)) ?:
		      mtk_seninf_mark_mux(priv, &used, i,
					  FIELD_GET(CAM_MUX_PCSR_NEXT_SRC_SEL, ctrl));
		if (ret)
			return ret;
	}

	for (i = 0; i < soc->num_mux; i++) {
		if (priv->mux_busy & BIT(i))
			continue;

		ret = mtk_seninf_check_idle(priv, SENINF_MUX_BASE(i) + SENINF_MUX_CTRL_0,
					    SENINF_MUX_EN);
		if (ret)
			return ret;
	}

	for (i = 0; i < soc->num_mux_alloc; i++)
		if (!(used & BIT(soc->mux_alloc[i])))
			return soc->mux_alloc[i];

	dev_err(priv->dev, "no free mux for CAM_MUX %u\n", cammux);
	return -EBUSY;
}

/* Isolate the CAM_MUX, reset the CSI-2 receiver and set its stream filter. */
static int mtk_seninf_csi2_reset(struct mtk_seninf *priv, unsigned int intf,
				 const struct mtk_seninf_route *route)
{
	u32 base = SENINF_INTF_BASE(intf), csi2 = SENINF_CSI2_BASE(intf);
	u32 pcsr = SENINF_CAM_MUX_PCSR_BASE(route->cammux);
	unsigned int i;

	/* Every CAM_MUX leaves reset enabled; isolate this one first. */
	mtk_seninf_update(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL, RG_SENINF_CAM_MUX_PCSR_EN, 0);
	if (mtk_seninf_read(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL) & RG_SENINF_CAM_MUX_PCSR_EN) {
		dev_err(priv->dev, "CAM_MUX %u stays enabled\n", route->cammux);
		return -EIO;
	}

	mtk_seninf_update(priv, base + SENINF_CSI2_CTRL, SENINF_CSI2_SW_RST, SENINF_CSI2_SW_RST);
	udelay(1);
	mtk_seninf_update(priv, base + SENINF_CSI2_CTRL, SENINF_CSI2_SW_RST, 0);

	/* Filter S0 passes the route's VC and DT to channel group 0. */
	for (i = 0; i < SENINF_CSI2_NUM_DI; i++)
		mtk_seninf_write(priv, csi2 + SENINF_CSI2_S_DI_CTRL(i), 0);
	for (i = 0; i < SENINF_CSI2_NUM_CH; i++)
		mtk_seninf_write(priv, csi2 + SENINF_CSI2_CH_CTRL(i), 0);
	mtk_seninf_write(priv, csi2 + SENINF_CSI2_S_DI_CTRL(0),
			 FIELD_PREP(RG_CSI2_S_DT_SEL, route->dt) |
			 FIELD_PREP(RG_CSI2_S_VC_SEL, route->vc) |
			 FIELD_PREP(RG_CSI2_S_DT_INTERLEAVE_MODE, 1) |
			 RG_CSI2_S_VC_INTERLEAVE_EN);
	mtk_seninf_write(priv, csi2 + SENINF_CSI2_CH_CTRL(0), RG_CSI2_CH_S_GRP_EN(0));

	return 0;
}

/*
 * Enable the CSI-2 receiver for the route's lanes and select D-PHY for the
 * port, after phy_configure() and before phy_power_on().
 */
static void mtk_seninf_csi2_enable(struct mtk_seninf *priv, unsigned int intf,
				   const struct mtk_seninf_route *route)
{
	u32 base = SENINF_INTF_BASE(intf), csi2 = SENINF_CSI2_BASE(intf);
	u32 top_phy = SENINF_TOP_PHY_CTRL_CSI(intf / 2);
	u64 cycles;

	mtk_seninf_update(priv, base + SENINF_CSI2_CTRL, RG_SENINF_CSI2_EN, RG_SENINF_CSI2_EN);
	mtk_seninf_update(priv, base + SENINF_CTRL, SENINF_EN, SENINF_EN);
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_DBG_CTRL, RG_CSI2_DBG_PACKET_CNT_EN,
			  RG_CSI2_DBG_PACKET_CNT_EN);
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_OPT, RG_CSI2_CPHY_SEL, 0);
	mtk_seninf_write(priv, csi2 + SENINF_CSI2_EN, GENMASK(route->lanes - 1, 0));
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_HDR_MODE_0, RG_CSI2_HEADER_MODE, 0);
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_HDR_MODE_0, RG_CSI2_HEADER_LEN, 0);

	/*
	 * Resync dummy cycles, as the vendor computes them: the SENINF clock
	 * cycles that 64 bits take on one lane, plus one.
	 */
	cycles = div64_u64(64ULL * priv->soc->csi_clk_hz, route->lane_rate) + 1;
	cycles = min_t(u64, cycles, FIELD_MAX(RG_CSI2_RESYNC_DMY_CYCLE));
	mtk_seninf_write(priv, csi2 + SENINF_CSI2_RESYNC_MERGE_CTRL,
			 SENINF_CSI2_RESYNC_MERGE_DPHY);
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_RESYNC_MERGE_CTRL, RG_CSI2_RESYNC_DMY_CYCLE,
			  FIELD_PREP(RG_CSI2_RESYNC_DMY_CYCLE, cycles));
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_RESYNC_MERGE_CTRL, RG_CSI2_RESYNC_DMY_CNT,
			  FIELD_PREP(RG_CSI2_RESYNC_DMY_CNT, 3));
	mtk_seninf_update(priv, csi2 + SENINF_CSI2_RESYNC_MERGE_CTRL, RG_CSI2_RESYNC_DMY_EN,
			  FIELD_PREP(RG_CSI2_RESYNC_DMY_EN, route->lanes == 2 ? 0x3 : 0xf));

	/* D-PHY on the whole port (4D1C) */
	mtk_seninf_update(priv, top_phy, RG_PHY_SENINF_CPHY_MODE, 0);
	mtk_seninf_update(priv, top_phy, PHY_SENINF_CPHY_EN, 0);
	mtk_seninf_update(priv, top_phy, PHY_SENINF_DPHY_EN, PHY_SENINF_DPHY_EN);
}

/* Stop the lanes of the CSI-2 receiver; the PHY powers off after this. */
static void mtk_seninf_csi2_off(struct mtk_seninf *priv, unsigned int intf)
{
	mtk_seninf_write(priv, SENINF_CSI2_BASE(intf) + SENINF_CSI2_EN, 0);
}

/* Return the interface controls to their reset values. */
static void mtk_seninf_intf_off(struct mtk_seninf *priv, unsigned int intf)
{
	u32 base = SENINF_INTF_BASE(intf);

	mtk_seninf_update(priv, SENINF_TOP_PHY_CTRL_CSI(intf / 2), PHY_SENINF_DPHY_EN, 0);
	mtk_seninf_update(priv, base + SENINF_CTRL, SENINF_EN, 0);
	mtk_seninf_update(priv, base + SENINF_CSI2_CTRL, RG_SENINF_CSI2_EN, 0);
}

/* Route the interface through the mux to the CAM_MUX, both still disabled. */
static void mtk_seninf_route_setup(struct mtk_seninf *priv, struct mtk_seninf_input *in,
				   const struct mtk_seninf_route *route)
{
	u32 top = SENINF_TOP_MUX_CTRL(in->mux), top_src = RG_SENINF_TOP_MUX_SRC_SEL(in->mux);
	u32 mux = SENINF_MUX_BASE(in->mux), pcsr = SENINF_CAM_MUX_PCSR_BASE(route->cammux);
	unsigned int pix = priv->soc->pixel_mode;

	in->saved_top_src = field_get(top_src, mtk_seninf_read(priv, top));
	mtk_seninf_update(priv, top, top_src, field_prep(top_src, in->intf));

	mtk_seninf_write(priv, mux + SENINF_MUX_CTRL_0, 0);
	mtk_seninf_write(priv, mux + SENINF_MUX_CTRL_0, SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST);
	mtk_seninf_write(priv, mux + SENINF_MUX_CTRL_0, 0);
	mtk_seninf_write(priv, mux + SENINF_MUX_CTRL_1,
			 FIELD_PREP(RG_SENINF_MUX_FIFO_PUSH_EN, SENINF_MUX_FIFO_PUSH_ALL) |
			 FIELD_PREP(RG_SENINF_MUX_PIX_MODE_SEL, pix) |
			 FIELD_PREP(RG_SENINF_MUX_SRC_SEL, SENINF_MUX_SRC_MIPI_SENSOR));
	mtk_seninf_update(priv, mux + SENINF_MUX_OPT,
			  RG_SENINF_MUX_HSYNC_POL | RG_SENINF_MUX_VSYNC_POL, 0);

	/* The CAM_MUX size checker stays disabled; only its pixel mode is set. */
	mtk_seninf_update(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL, MTK_SENINF_CAM_MUX_ROUTE,
			  FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_SRC_SEL, in->mux) |
			  FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_CHK_PIX_MODE, pix) |
			  FIELD_PREP(CAM_MUX_PCSR_NEXT_SRC_SEL, SENINF_CAM_MUX_SRC_NONE));
	mtk_seninf_update(priv, pcsr + SENINF_CAM_MUX_PCSR_OPT, MTK_SENINF_CAM_MUX_FILTER,
			  FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_VC_SEL, route->vc) |
			  RG_SENINF_CAM_MUX_PCSR_VC_SEL_EN |
			  FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_DT_SEL, route->dt) |
			  RG_SENINF_CAM_MUX_PCSR_DT_SEL_EN);
	mtk_seninf_write(priv, pcsr + SENINF_CAM_MUX_PCSR_CHK_CTL,
			 FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_EXP_VSIZE, route->height) |
			 FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_EXP_HSIZE, route->width));
	mtk_seninf_write(priv, pcsr + SENINF_CAM_MUX_PCSR_IRQ_STATUS, MTK_SENINF_CAM_MUX_IRQS);
	mtk_seninf_update(priv, mux + SENINF_MUX_FRAME_SIZE_MON_CTRL,
			  RG_SENINF_MUX_FRAME_SIZE_MON_EN, RG_SENINF_MUX_FRAME_SIZE_MON_EN);
}

/**
 * mtk_seninf_route_start() - start a route, up to the point where data flows
 * @priv: SENINF device, runtime-active, with &mtk_seninf.lock held
 * @in: interface of the route
 * @route: route parameters
 *
 * Resets the CSI-2 receiver, configures the PHY, enables the receiver and
 * powers the PHY on, in the vendor order, then routes the interface through a
 * free mux to the CAM_MUX and enables both. The camera engine behind the
 * CAM_MUX must already be armed; the sensor starts after this returns.
 *
 * Return: 0 on success or a negative error code.
 */
int mtk_seninf_route_start(struct mtk_seninf *priv, struct mtk_seninf_input *in,
			   const struct mtk_seninf_route *route)
{
	union phy_configure_opts opts = {};
	int mux, ret;

	mux = mtk_seninf_route_check(priv, in->intf, route->cammux);
	if (mux < 0)
		return mux;

	ret = phy_mipi_dphy_get_default_config_for_hsclk(route->lane_rate, route->lanes,
							 &opts.mipi_dphy);
	if (ret)
		return ret;

	/* The transmitter's own T_HS-TRAIL, when the device tree gives it. */
	if (in->hs_trail_ps)
		opts.mipi_dphy.hs_trail = in->hs_trail_ps;

	ret = phy_init(in->phy);
	if (ret)
		return ret;

	ret = mtk_seninf_csi2_reset(priv, in->intf, route);
	if (ret)
		goto err_phy_exit;

	/* Analog defaults and D-PHY timing precede the receiver enables. */
	ret = phy_set_mode(in->phy, PHY_MODE_MIPI_DPHY);
	if (!ret)
		ret = phy_configure(in->phy, &opts);
	if (ret) {
		dev_err(priv->dev,
			"failed to configure the PHY: %d (%llu bit/s, T_HS-TRAIL %u ps%s)\n",
			ret, route->lane_rate, opts.mipi_dphy.hs_trail,
			in->hs_trail_ps ? " from mediatek,hs-trail-ps" : "");
		goto err_phy_exit;
	}

	mtk_seninf_csi2_enable(priv, in->intf, route);

	ret = phy_power_on(in->phy);
	if (ret)
		goto err_intf_off;

	/* Write 1 to clear. IRQ_EN is never written: bit 31 is the clear mode. */
	mtk_seninf_write(priv, SENINF_CSI2_BASE(in->intf) + SENINF_CSI2_IRQ_STATUS, U32_MAX);

	in->mux = mux;
	in->cammux = route->cammux;
	mtk_seninf_route_setup(priv, in, route);

	/* Open the route; data from the sensor reaches the engine from here on. */
	mtk_seninf_write(priv, SENINF_MUX_BASE(mux) + SENINF_MUX_CTRL_0, SENINF_MUX_EN);
	mtk_seninf_update(priv, SENINF_CAM_MUX_PCSR_BASE(route->cammux) + SENINF_CAM_MUX_PCSR_CTRL,
			  RG_SENINF_CAM_MUX_PCSR_EN, RG_SENINF_CAM_MUX_PCSR_EN);

	priv->mux_busy |= BIT(mux);
	in->streaming = true;

	return 0;

err_intf_off:
	mtk_seninf_csi2_off(priv, in->intf);
	mtk_seninf_intf_off(priv, in->intf);
err_phy_exit:
	phy_exit(in->phy);
	return ret;
}

/**
 * mtk_seninf_route_stop() - stop a running route
 * @priv: SENINF device, runtime-active, with &mtk_seninf.lock held
 * @in: interface of the route
 *
 * Closes the CAM_MUX and the mux, stops the CSI-2 receiver, powers the PHY
 * off and logs the CSI-2 and mux status of the stream. The engine behind the
 * CAM_MUX must be idle and the sensor stopped.
 */
void mtk_seninf_route_stop(struct mtk_seninf *priv, struct mtk_seninf_input *in)
{
	u32 pcsr = SENINF_CAM_MUX_PCSR_BASE(in->cammux), mux = SENINF_MUX_BASE(in->mux);
	u32 status, mux_status, mux_err_size;
	int ret;

	/* The mux status covers the whole stream; read it before the teardown. */
	mux_status = mtk_seninf_read(priv, mux + SENINF_MUX_IRQ_STATUS);
	mux_err_size = mtk_seninf_read(priv, mux + SENINF_MUX_ERR_SIZE);

	mtk_seninf_update(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL, RG_SENINF_CAM_MUX_PCSR_EN, 0);
	mtk_seninf_update(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL,
			  RG_SENINF_CAM_MUX_PCSR_SRC_SEL | CAM_MUX_PCSR_NEXT_SRC_SEL,
			  FIELD_PREP(RG_SENINF_CAM_MUX_PCSR_SRC_SEL, SENINF_CAM_MUX_SRC_NONE) |
			  FIELD_PREP(CAM_MUX_PCSR_NEXT_SRC_SEL, SENINF_CAM_MUX_SRC_NONE));
	mtk_seninf_write(priv, pcsr + SENINF_CAM_MUX_PCSR_IRQ_STATUS, MTK_SENINF_CAM_MUX_IRQS);

	mtk_seninf_update(priv, mux + SENINF_MUX_CTRL_0, SENINF_MUX_EN, 0);
	mtk_seninf_update(priv, mux + SENINF_MUX_FRAME_SIZE_MON_CTRL,
			  RG_SENINF_MUX_FRAME_SIZE_MON_EN, 0);

	mtk_seninf_csi2_off(priv, in->intf);
	ret = phy_power_off(in->phy);
	if (ret)
		dev_err(priv->dev, "failed to power off the PHY: %d\n", ret);
	phy_exit(in->phy);

	status = mtk_seninf_read(priv, SENINF_CSI2_BASE(in->intf) + SENINF_CSI2_IRQ_STATUS);
	if (status & SENINF_CSI2_IRQ_ERRORS)
		dev_warn_ratelimited(priv->dev, "interface %u CSI-2 errors, status %#x\n",
				     in->intf, status);
	else
		dev_dbg(priv->dev, "interface %u CSI-2 status %#x\n", in->intf, status);

	if (mux_status & SENINF_MUX_IRQ_ERRORS)
		dev_warn_ratelimited(priv->dev, "mux %u errors, status %#x, error size %#x\n",
				     in->mux, mux_status, mux_err_size);
	else
		dev_dbg(priv->dev, "mux %u status %#x, error size %#x\n",
			in->mux, mux_status, mux_err_size);

	mtk_seninf_intf_off(priv, in->intf);
	mtk_seninf_update(priv, SENINF_TOP_MUX_CTRL(in->mux), RG_SENINF_TOP_MUX_SRC_SEL(in->mux),
			  field_prep(RG_SENINF_TOP_MUX_SRC_SEL(in->mux), in->saved_top_src));

	priv->mux_busy &= ~BIT(in->mux);
	in->streaming = false;
}

/**
 * mtk_seninf_route_log() - log the registers of a running route
 * @priv: SENINF device, with &mtk_seninf.lock held
 * @in: interface of the running route
 */
void mtk_seninf_route_log(struct mtk_seninf *priv, const struct mtk_seninf_input *in)
{
	u32 base = SENINF_INTF_BASE(in->intf), csi2 = SENINF_CSI2_BASE(in->intf);
	u32 mux = SENINF_MUX_BASE(in->mux), pcsr = SENINF_CAM_MUX_PCSR_BASE(in->cammux);
	u32 gcsr = SENINF_CAM_MUX_GCSR_BASE;
	struct device *dev = priv->dev;

	dev_info(dev, "interface %u -> mux %u -> CAM_MUX %u, top mux %#010x\n",
		 in->intf, in->mux, in->cammux,
		 mtk_seninf_read(priv, SENINF_TOP_MUX_CTRL(in->mux)));
	dev_info(dev, "CAM_MUX dyn %#x dyn_en %#x/%#x enabled %#x\n",
		 mtk_seninf_read(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_CTRL),
		 mtk_seninf_read(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_EN0),
		 mtk_seninf_read(priv, gcsr + SENINF_CAM_MUX_GCSR_DYN_EN1),
		 mtk_seninf_read(priv, gcsr + SENINF_CAM_MUX_GCSR_MUX_EN));
	dev_info(dev, "interface ctrl %#x csi2_ctrl %#x\n",
		 mtk_seninf_read(priv, base + SENINF_CTRL),
		 mtk_seninf_read(priv, base + SENINF_CSI2_CTRL));
	dev_info(dev, "CSI-2 en %#x resync %#x di0 %#x ch0 %#x irq_en %#x status %#x\n",
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_EN),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_RESYNC_MERGE_CTRL),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_S_DI_CTRL(0)),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_CH_CTRL(0)),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_IRQ_EN),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_IRQ_STATUS));
	dev_info(dev, "CSI-2 line/frame %#x last packet %#x packets %#x\n",
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_LINE_FRAME_NUM),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_PACKET_STATUS),
		 mtk_seninf_read(priv, csi2 + SENINF_CSI2_PACKET_CNT_STATUS));
	dev_info(dev, "mux ctrl %#x/%#x opt %#x size %#x monitor %#x h %u/%u v %u/%u\n",
		 mtk_seninf_read(priv, mux + SENINF_MUX_CTRL_0),
		 mtk_seninf_read(priv, mux + SENINF_MUX_CTRL_1),
		 mtk_seninf_read(priv, mux + SENINF_MUX_OPT),
		 mtk_seninf_read(priv, mux + SENINF_MUX_SIZE),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FRAME_SIZE_MON_CTRL),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FRAME_SIZE_MON_H_VALID),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FRAME_SIZE_MON_H_BLANK),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FRAME_SIZE_MON_V_VALID),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FRAME_SIZE_MON_V_BLANK));
	dev_info(dev, "mux status %#x error size %#x FIFO %#x\n",
		 mtk_seninf_read(priv, mux + SENINF_MUX_IRQ_STATUS),
		 mtk_seninf_read(priv, mux + SENINF_MUX_ERR_SIZE),
		 mtk_seninf_read(priv, mux + SENINF_MUX_FIFO_STATUS));
	dev_info(dev, "CAM_MUX ctrl %#x opt %#x chk_ctl %#x chk_res %#x\n",
		 mtk_seninf_read(priv, pcsr + SENINF_CAM_MUX_PCSR_CTRL),
		 mtk_seninf_read(priv, pcsr + SENINF_CAM_MUX_PCSR_OPT),
		 mtk_seninf_read(priv, pcsr + SENINF_CAM_MUX_PCSR_CHK_CTL),
		 mtk_seninf_read(priv, pcsr + SENINF_CAM_MUX_PCSR_CHK_RES));
}
