// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/soc/mediatek/mtk-mmsys.h>

#include "mtk-mmsys.h"
#include "mt8167-mmsys.h"
#include "mt8173-mmsys.h"
#include "mt8183-mmsys.h"
#include "mt8186-mmsys.h"
#include "mt8188-mmsys.h"
#include "mt8192-mmsys.h"
#include "mt6895-mmsys.h"
#include "mt8195-mmsys.h"
#include "mt8365-mmsys.h"

#define MMSYS_SW_RESET_PER_REG 32

static const struct mtk_mmsys_driver_data mt2701_mmsys_driver_data = {
	.clk_driver = "clk-mt2701-mm",
	.routes = mmsys_default_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_default_routing_table),
};

static const struct mtk_mmsys_driver_data mt2712_mmsys_driver_data = {
	.clk_driver = "clk-mt2712-mm",
	.routes = mmsys_default_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_default_routing_table),
};

static const struct mtk_mmsys_driver_data mt6779_mmsys_driver_data = {
	.clk_driver = "clk-mt6779-mm",
};

static const struct mtk_mmsys_driver_data mt6795_mmsys_driver_data = {
	.clk_driver = "clk-mt6795-mm",
	.routes = mt8173_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8173_mmsys_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 64,
};

static const struct mtk_mmsys_driver_data mt6797_mmsys_driver_data = {
	.clk_driver = "clk-mt6797-mm",
};

static const struct mtk_mmsys_driver_data mt8167_mmsys_driver_data = {
	.clk_driver = "clk-mt8167-mm",
	.routes = mt8167_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8167_mmsys_routing_table),
};

static const struct mtk_mmsys_driver_data mt8173_mmsys_driver_data = {
	.clk_driver = "clk-mt8173-mm",
	.routes = mt8173_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8173_mmsys_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 64,
};

static const struct mtk_mmsys_driver_data mt8183_mmsys_driver_data = {
	.clk_driver = "clk-mt8183-mm",
	.routes = mmsys_mt8183_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8183_routing_table),
	.sw0_rst_offset = MT8183_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8186_mmsys_driver_data = {
	.clk_driver = "clk-mt8186-mm",
	.routes = mmsys_mt8186_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8186_routing_table),
	.sw0_rst_offset = MT8186_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8188_vdosys0_driver_data = {
	.clk_driver = "clk-mt8188-vdo0",
	.routes = mmsys_mt8188_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8188_routing_table),
	.sw0_rst_offset = MT8188_VDO0_SW0_RST_B,
	.rst_tb = mmsys_mt8188_vdo0_rst_tb,
	.num_resets = ARRAY_SIZE(mmsys_mt8188_vdo0_rst_tb),
};

static const struct mtk_mmsys_driver_data mt8188_vdosys1_driver_data = {
	.clk_driver = "clk-mt8188-vdo1",
	.routes = mmsys_mt8188_vdo1_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8188_vdo1_routing_table),
	.sw0_rst_offset = MT8188_VDO1_SW0_RST_B,
	.rst_tb = mmsys_mt8188_vdo1_rst_tb,
	.num_resets = ARRAY_SIZE(mmsys_mt8188_vdo1_rst_tb),
	.vsync_len = 1,
};

static const struct mtk_mmsys_driver_data mt8188_vppsys0_driver_data = {
	.clk_driver = "clk-mt8188-vpp0",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8188_vppsys1_driver_data = {
	.clk_driver = "clk-mt8188-vpp1",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8192_mmsys_driver_data = {
	.clk_driver = "clk-mt8192-mm",
	.routes = mmsys_mt8192_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8192_routing_table),
	.sw0_rst_offset = MT8186_MMSYS_SW0_RST_B,
	.num_resets = 32,
};

static const struct mtk_mmsys_driver_data mt8195_vdosys0_driver_data = {
	.clk_driver = "clk-mt8195-vdo0",
	.routes = mmsys_mt8195_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8195_routing_table),
};

static const struct mtk_mmsys_driver_data mt8195_vdosys1_driver_data = {
	.clk_driver = "clk-mt8195-vdo1",
	.routes = mmsys_mt8195_vdo1_routing_table,
	.num_routes = ARRAY_SIZE(mmsys_mt8195_vdo1_routing_table),
	.sw0_rst_offset = MT8195_VDO1_SW0_RST_B,
	.num_resets = 64,
};

static const struct mtk_mmsys_driver_data mt8195_vppsys0_driver_data = {
	.clk_driver = "clk-mt8195-vpp0",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8195_vppsys1_driver_data = {
	.clk_driver = "clk-mt8195-vpp1",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt6895_mmsys0_driver_data = {
	.clk_driver = "clk-mt6895-mmsys0",
};

static const struct mtk_mmsys_driver_data mt6895_mmsys1_driver_data = {
	.clk_driver = "clk-mt6895-mmsys1",
	.is_vppsys = true,
};

static const struct mtk_mmsys_driver_data mt8365_mmsys_driver_data = {
	.clk_driver = "clk-mt8365-mm",
	.routes = mt8365_mmsys_routing_table,
	.num_routes = ARRAY_SIZE(mt8365_mmsys_routing_table),
};

struct mtk_mmsys {
	void __iomem *regs;
	const struct mtk_mmsys_driver_data *data;
	struct platform_device *clks_pdev;
	struct platform_device *drm_pdev;
	spinlock_t lock; /* protects mmsys_sw_rst_b reg */
	struct reset_controller_dev rcdev;
	struct cmdq_client_reg cmdq_base;
};

static void mtk_mmsys_update_bits(struct mtk_mmsys *mmsys, u32 offset, u32 mask, u32 val,
				  struct cmdq_pkt *cmdq_pkt)
{
	int ret;
	u32 tmp;

	if (mmsys->cmdq_base.size && cmdq_pkt) {
		ret = cmdq_pkt_write_mask(cmdq_pkt, mmsys->cmdq_base.subsys,
					  mmsys->cmdq_base.offset + offset, val,
					  mask);
		if (ret)
			pr_debug("CMDQ unavailable: using CPU write\n");
		else
			return;
	}
	tmp = readl_relaxed(mmsys->regs + offset);
	tmp = (tmp & ~mask) | (val & mask);
	writel_relaxed(tmp, mmsys->regs + offset);
}

void mtk_mmsys_ddp_connect(struct device *dev,
			   enum mtk_ddp_comp_id cur,
			   enum mtk_ddp_comp_id next)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);
	const struct mtk_mmsys_routes *routes = mmsys->data->routes;
	int i;

	for (i = 0; i < mmsys->data->num_routes; i++)
		if (cur == routes[i].from_comp && next == routes[i].to_comp)
			mtk_mmsys_update_bits(mmsys, routes[i].addr, routes[i].mask,
					      routes[i].val, NULL);

	if (mmsys->data->vsync_len)
		mtk_mmsys_update_bits(mmsys, MT8188_VDO1_MIXER_VSYNC_LEN, GENMASK(31, 0),
				      mmsys->data->vsync_len, NULL);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_connect);

void mtk_mmsys_ddp_disconnect(struct device *dev,
			      enum mtk_ddp_comp_id cur,
			      enum mtk_ddp_comp_id next)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);
	const struct mtk_mmsys_routes *routes = mmsys->data->routes;
	int i;

	for (i = 0; i < mmsys->data->num_routes; i++)
		if (cur == routes[i].from_comp && next == routes[i].to_comp)
			mtk_mmsys_update_bits(mmsys, routes[i].addr, routes[i].mask, 0, NULL);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_disconnect);

void mtk_mmsys_merge_async_config(struct device *dev, int idx, int width, int height,
				  struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_MERGE0_ASYNC_CFG_WD + 0x10 * idx,
			      ~0, height << 16 | width, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_merge_async_config);

void mtk_mmsys_hdr_config(struct device *dev, int be_width, int be_height,
			  struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_HDRBE_ASYNC_CFG_WD, ~0,
			      be_height << 16 | be_width, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_hdr_config);

void mtk_mmsys_mixer_in_config(struct device *dev, int idx, bool alpha_sel, u16 alpha,
			       u8 mode, u32 biwidth, struct cmdq_pkt *cmdq_pkt)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);

	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_MIXER_IN1_ALPHA + (idx - 1) * 4, ~0,
			      alpha << 16 | alpha, cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_HDR_TOP_CFG, BIT(15 + idx), 0, cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_HDR_TOP_CFG, BIT(19 + idx),
			      alpha_sel << (19 + idx), cmdq_pkt);
	mtk_mmsys_update_bits(mmsys, MT8195_VDO1_MIXER_IN1_PAD + (idx - 1) * 4,
			      GENMASK(31, 16) | GENMASK(1, 0), biwidth << 16 | mode, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_mixer_in_config);

void mtk_mmsys_mixer_in_channel_swap(struct device *dev, int idx, bool channel_swap,
				     struct cmdq_pkt *cmdq_pkt)
{
	mtk_mmsys_update_bits(dev_get_drvdata(dev), MT8195_VDO1_MIXER_IN1_PAD + (idx - 1) * 4,
			      BIT(4), channel_swap << 4, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_mixer_in_channel_swap);

void mtk_mmsys_ddp_dpi_fmt_config(struct device *dev, u32 val)
{
	struct mtk_mmsys *mmsys = dev_get_drvdata(dev);

	switch (val) {
	case MTK_DPI_RGB888_SDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB888_SDR_CON, NULL);
		break;
	case MTK_DPI_RGB565_SDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB565_SDR_CON, NULL);
		break;
	case MTK_DPI_RGB565_DDR_CON:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB565_DDR_CON, NULL);
		break;
	case MTK_DPI_RGB888_DDR_CON:
	default:
		mtk_mmsys_update_bits(mmsys, MT8186_MMSYS_DPI_OUTPUT_FORMAT,
				      MT8186_DPI_FORMAT_MASK, MT8186_DPI_RGB888_DDR_CON, NULL);
		break;
	}
}
EXPORT_SYMBOL_GPL(mtk_mmsys_ddp_dpi_fmt_config);

void mtk_mmsys_vpp_rsz_merge_config(struct device *dev, u32 id, bool enable,
				    struct cmdq_pkt *cmdq_pkt)
{
	u32 reg;

	switch (id) {
	case 2:
		reg = MT8195_SVPP2_BUF_BF_RSZ_SWITCH;
		break;
	case 3:
		reg = MT8195_SVPP3_BUF_BF_RSZ_SWITCH;
		break;
	default:
		dev_err(dev, "Invalid id %d\n", id);
		return;
	}

	mtk_mmsys_update_bits(dev_get_drvdata(dev), reg, ~0, enable, cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_vpp_rsz_merge_config);

void mtk_mmsys_vpp_rsz_dcm_config(struct device *dev, bool enable,
				  struct cmdq_pkt *cmdq_pkt)
{
	u32 client;

	client = MT8195_SVPP1_MDP_RSZ;
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_1ST_DIS0, client,
			      ((enable) ? client : 0), cmdq_pkt);
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_2ND_DIS0, client,
			      ((enable) ? client : 0), cmdq_pkt);

	client = MT8195_SVPP2_MDP_RSZ | MT8195_SVPP3_MDP_RSZ;
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_1ST_DIS1, client,
			      ((enable) ? client : 0), cmdq_pkt);
	mtk_mmsys_update_bits(dev_get_drvdata(dev),
			      MT8195_VPP1_HW_DCM_2ND_DIS1, client,
			      ((enable) ? client : 0), cmdq_pkt);
}
EXPORT_SYMBOL_GPL(mtk_mmsys_vpp_rsz_dcm_config);

static int mtk_mmsys_reset_update(struct reset_controller_dev *rcdev, unsigned long id,
				  bool assert)
{
	struct mtk_mmsys *mmsys = container_of(rcdev, struct mtk_mmsys, rcdev);
	unsigned long flags;
	u32 offset;
	u32 reg;

	if (mmsys->data->rst_tb) {
		if (id >= mmsys->data->num_resets) {
			dev_err(rcdev->dev, "Invalid reset ID: %lu (>=%u)\n",
				id, mmsys->data->num_resets);
			return -EINVAL;
		}
		id = mmsys->data->rst_tb[id];
	}

	offset = (id / MMSYS_SW_RESET_PER_REG) * sizeof(u32);
	id = id % MMSYS_SW_RESET_PER_REG;
	reg = mmsys->data->sw0_rst_offset + offset;

	spin_lock_irqsave(&mmsys->lock, flags);

	if (assert)
		mtk_mmsys_update_bits(mmsys, reg, BIT(id), 0, NULL);
	else
		mtk_mmsys_update_bits(mmsys, reg, BIT(id), BIT(id), NULL);

	spin_unlock_irqrestore(&mmsys->lock, flags);

	return 0;
}

static int mtk_mmsys_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return mtk_mmsys_reset_update(rcdev, id, true);
}

static int mtk_mmsys_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return mtk_mmsys_reset_update(rcdev, id, false);
}

static int mtk_mmsys_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = mtk_mmsys_reset_assert(rcdev, id);
	if (ret)
		return ret;

	usleep_range(1000, 1100);

	return mtk_mmsys_reset_deassert(rcdev, id);
}

static const struct reset_control_ops mtk_mmsys_reset_ops = {
	.assert = mtk_mmsys_reset_assert,
	.deassert = mtk_mmsys_reset_deassert,
	.reset = mtk_mmsys_reset,
};

static int mtk_mmsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct platform_device *clks;
	struct platform_device *drm;
	struct mtk_mmsys *mmsys;
	int ret;

	mmsys = devm_kzalloc(dev, sizeof(*mmsys), GFP_KERNEL);
	if (!mmsys)
		return -ENOMEM;

	mmsys->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mmsys->regs)) {
		ret = PTR_ERR(mmsys->regs);
		dev_err(dev, "Failed to ioremap mmsys registers: %d\n", ret);
		return ret;
	}

	mmsys->data = of_device_get_match_data(&pdev->dev);

	if (mmsys->data->num_resets > 0) {
		spin_lock_init(&mmsys->lock);

		mmsys->rcdev.owner = THIS_MODULE;
		mmsys->rcdev.nr_resets = mmsys->data->num_resets;
		mmsys->rcdev.ops = &mtk_mmsys_reset_ops;
		mmsys->rcdev.of_node = pdev->dev.of_node;
		ret = devm_reset_controller_register(&pdev->dev, &mmsys->rcdev);
		if (ret) {
			dev_err(&pdev->dev, "Couldn't register mmsys reset controller: %d\n", ret);
			return ret;
		}
	}

	/* CMDQ is optional */
	ret = cmdq_dev_get_client_reg(dev, &mmsys->cmdq_base, 0);
	if (ret)
		dev_dbg(dev, "No mediatek,gce-client-reg!\n");

	platform_set_drvdata(pdev, mmsys);

	clks = platform_device_register_data(&pdev->dev, mmsys->data->clk_driver,
					     PLATFORM_DEVID_AUTO, NULL, 0);
	if (IS_ERR(clks))
		return PTR_ERR(clks);
	mmsys->clks_pdev = clks;

	/*
	 * The MT6895 display path OVL0/RDMA0 -> DSC_WRAP0 -> DSI0 physically
	 * routes through the "PQ" post-processing chain (TDSHP0, COLOR0,
	 * CCORR0/1, C3D0, AAL0, GAMMA0, POSTMASK0, DITHER0, CM0, SPR0, PQ0).
	 * Those blocks have no mainline ddp driver (pure passthrough in video
	 * mode) so no DT consumer enables their gate clocks; without them the
	 * pipeline backpressures and stalls (OVL stops producing vblank after
	 * a few frames). Enable the gates directly (the clk provider for this
	 * mmsys0 device is registered asynchronously, so of_clk_get here would
	 * race). Bits per the mmsys0 gate layout: MM0_0 set@0x104 (COLOR0=18,
	 * CCORR0=19, CCORR1=20, AAL0=21, GAMMA0=22, POSTMASK0=23, DITHER0=24,
	 * CM0=25, SPR0=26), MM0_1 set@0x114 (TDSHP0=2, C3D0=3).
	 */
	if (mmsys->data == &mt6895_mmsys0_driver_data) {
		/*
		 * XAGA: do NOT touch the display hardware at probe time.
		 * These gate RMWs run ~0.1s after boot while LK's video
		 * stream is still settling; any register write to mmsys then
		 * can tip a marginal stream into the stalled grey state.
		 * The downstream kernel probes at ~0.78s (stream settled,
		 * 10/10 clean handoffs). The PQ-chain EN bits are re-armed
		 * in ddp_hw_init (XAGA-stop block) anyway, after the DSI is
		 * stopped, which is the safe moment.
		 * (The crossbar F24/F8C must stay 0 - the working device
		 * routes OVL1_2L via DLI0/bgout natively.)
		 */
	}

	/*
	 * The mt6895 display data path OVL0/RDMA0 -> DSC_WRAP0 -> MAIN0 ->
	 * DSI0 physically routes through the PQ post-processing chain. Those
	 * blocks have no mainline ddp driver, but their gate clocks AND their
	 * own enable/start bits must be set or data stops after a few frames
	 * (OVL produces ~3 FME_CPL then the DSI buffer underruns). LK left the
	 * crossbar + mutex configured; we just need the blocks turned on.
	 * Enable bits/offsets from the stock mt6895 mediatek_v2 drivers.
	 */
	if (mmsys->data == &mt6895_mmsys0_driver_data) {
		struct {
			phys_addr_t base;
			u32 reg;
			u32 val;
			u32 mask;
		} const pq[] = {
			/* TDSHP0: bypass shadow @0x67c, then CTRL@0x100 bit0 */
			{0x14007000, 0x67c, 1, 0x1},
			{0x14007000, 0x100, 1, 0x1},
			/* C3D0: EN@0x0 =1 */
			{0x14008000, 0x000, 1, 0x1},
			/* COLOR0: bypass shadow @0xcb0, then START@0xc00 =1 */
			{0x14009000, 0xcb0, 1, 0x1},
			{0x14009000, 0xc00, 1, 0x3},
			/* CCORR0/1: EN@0x0 =1 */
			{0x1400a000, 0x000, 1, 0x1},
			{0x1400b000, 0x000, 1, 0x1},
			/* AAL0: EN@0x0 =1 */
			{0x1400d000, 0x000, 1, 0x1},
			/* GAMMA0: EN@0x0 =1 */
			{0x1400e000, 0x000, 1, 0x1},
			/* POSTMASK0: EN@0x0 =1 */
			{0x1400f000, 0x000, 1, 0x1},
			/* DITHER0: EN@0x0 =1 */
			{0x14010000, 0x000, 1, 0x1},
			/* CM0: EN@0x0 =1 */
			{0x14013000, 0x000, 1, 0x1},
			/* SPR0: EN@0xc =1 */
			{0x14014000, 0x00c, 1, 0x1},
		};
		int i;

		/*
		 * XAGA: DISABLED - do NOT poke the PQ chain EN/start bits.
		 * LK already configured the whole display pipeline (OVL/RDMA ->
		 * PQ chain -> DSC -> DSI) in its working relay state. Writing
		 * these EN bits from here (even with matching values) disturbs
		 * LK's config and corrupts the frame boundaries -> RDMA
		 * EOF_ABNORMAL + DSC ABN_EOF -> the panel shows its grey+purple
		 * decode-fail pattern. The stock mediatek_v2 driver drives the PQ
		 * modules as real relay components; it never blind-pokes them.
		 */
		if (false) {
		for (i = 0; i < ARRAY_SIZE(pq); i++) {
			void __iomem *pqbase = ioremap(pq[i].base, 0x1000);
			u32 v;

			if (!pqbase)
				continue;
			v = readl(pqbase + pq[i].reg);
			v &= ~pq[i].mask;
			v |= pq[i].val & pq[i].mask;
			writel(v, pqbase + pq[i].reg);
			iounmap(pqbase);
		}
		}

		/*
		 * mt6895_mtk_sodi_config (RDMA0, en=1): select the DSI-buffer
		 * SMI request source and route the RDMA urgent/ultra signals so
		 * the DSI TX-buffer flow control feeds the pipeline. Without
		 * these the RDMA/DSC starves after a few frames and the DSI
		 * underruns (INP_UNFINISH). Values from the stock mtk_drm_ddp_comp.c.
		 *   SODI_REQ_MASK(0xF4) = 0xF500
		 *   EMI_REQ_CTL(0xF8)   = 0xdf
		 *   DUMMY0(0x400)       = 0x7
		 *   MISC(0xF0)          ultra sel = 0
		 *
		 * XAGA: DISABLED - LK already set these flow-control regs for the
		 * live pipeline; re-poking them here can disturb the RDMA request
		 * pacing and corrupt frames (EOF_ABNORMAL). Preserve LK's values.
		 */
		if (false) {
		writel_relaxed(0xF500, mmsys->regs + 0xF4);
		writel_relaxed(0xdf, mmsys->regs + 0xF8);
		writel_relaxed(0x7, mmsys->regs + 0x400);
		{
			u32 v = readl(mmsys->regs + 0xF0);

			v &= ~((0x3 << 10) | (0x3 << 2));
			writel_relaxed(v, mmsys->regs + 0xF0);
		}
		}
	}

	if (mmsys->data->is_vppsys)
		goto out_probe_done;

	drm = platform_device_register_data(&pdev->dev, "mediatek-drm",
					    PLATFORM_DEVID_AUTO, NULL, 0);
	if (IS_ERR(drm)) {
		platform_device_unregister(clks);
		return PTR_ERR(drm);
	}
	mmsys->drm_pdev = drm;

out_probe_done:
	return 0;
}

static void mtk_mmsys_remove(struct platform_device *pdev)
{
	struct mtk_mmsys *mmsys = platform_get_drvdata(pdev);

	platform_device_unregister(mmsys->drm_pdev);
	platform_device_unregister(mmsys->clks_pdev);
}

static const struct of_device_id of_match_mtk_mmsys[] = {
	{ .compatible = "mediatek,mt2701-mmsys", .data = &mt2701_mmsys_driver_data },
	{ .compatible = "mediatek,mt2712-mmsys", .data = &mt2712_mmsys_driver_data },
	{ .compatible = "mediatek,mt6779-mmsys", .data = &mt6779_mmsys_driver_data },
	{ .compatible = "mediatek,mt6795-mmsys", .data = &mt6795_mmsys_driver_data },
	{ .compatible = "mediatek,mt6797-mmsys", .data = &mt6797_mmsys_driver_data },
	{ .compatible = "mediatek,mt8167-mmsys", .data = &mt8167_mmsys_driver_data },
	{ .compatible = "mediatek,mt8173-mmsys", .data = &mt8173_mmsys_driver_data },
	{ .compatible = "mediatek,mt8183-mmsys", .data = &mt8183_mmsys_driver_data },
	{ .compatible = "mediatek,mt8186-mmsys", .data = &mt8186_mmsys_driver_data },
	{ .compatible = "mediatek,mt8188-vdosys0", .data = &mt8188_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8188-vdosys1", .data = &mt8188_vdosys1_driver_data },
	{ .compatible = "mediatek,mt8188-vppsys0", .data = &mt8188_vppsys0_driver_data },
	{ .compatible = "mediatek,mt8188-vppsys1", .data = &mt8188_vppsys1_driver_data },
	{ .compatible = "mediatek,mt8192-mmsys", .data = &mt8192_mmsys_driver_data },
	/* "mediatek,mt8195-mmsys" compatible is deprecated */
	{ .compatible = "mediatek,mt8195-mmsys", .data = &mt8195_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8195-vdosys0", .data = &mt8195_vdosys0_driver_data },
	{ .compatible = "mediatek,mt8195-vdosys1", .data = &mt8195_vdosys1_driver_data },
	{ .compatible = "mediatek,mt8195-vppsys0", .data = &mt8195_vppsys0_driver_data },
	{ .compatible = "mediatek,mt8195-vppsys1", .data = &mt8195_vppsys1_driver_data },
	{ .compatible = "mediatek,mt6895-mmsys0", .data = &mt6895_mmsys0_driver_data },
	{ .compatible = "mediatek,mt6895-mmsys1", .data = &mt6895_mmsys1_driver_data },
	{ .compatible = "mediatek,mt8365-mmsys", .data = &mt8365_mmsys_driver_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_mtk_mmsys);

static struct platform_driver mtk_mmsys_drv = {
	.driver = {
		.name = "mtk-mmsys",
		.of_match_table = of_match_mtk_mmsys,
	},
	.probe = mtk_mmsys_probe,
	.remove = mtk_mmsys_remove,
};
module_platform_driver(mtk_mmsys_drv);

MODULE_AUTHOR("Yongqiang Niu <yongqiang.niu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek SoC MMSYS driver");
MODULE_LICENSE("GPL");
