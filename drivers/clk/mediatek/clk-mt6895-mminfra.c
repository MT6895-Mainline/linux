// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Ren-Ting Wang <ren-ting.wang@mediatek.com>
 *
 * MediaTek MT6895 MMINFRA_CONFIG clock driver.
 *
 * Provides the GCE/SMI clock gates required by the display subsystem
 * (cmdq/gce, smi-common). The gates are ungated on demand by their
 * consumers; LK leaves them off at handoff.
 */
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/mt6895-clk.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mminfra_cg0_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mminfra_cg1_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

#define GATE_MMINFRA_CG0(_id, _name, _parent, _shift)			\
	GATE_MTK_FLAGS(_id, _name, _parent, &mminfra_cg0_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr, CLK_IS_CRITICAL)

#define GATE_MMINFRA_CG1(_id, _name, _parent, _shift)			\
	GATE_MTK_FLAGS(_id, _name, _parent, &mminfra_cg1_regs, _shift,	\
		 &mtk_clk_gate_ops_setclr, CLK_IS_CRITICAL)

static const struct mtk_gate mminfra_clks[] = {
	/* MMINFRA_CONFIG0 */
	GATE_MMINFRA_CG0(CLK_MMINFRA_GCE_D, "mminfra_gce_d", "mminfra_ck", 0),
	GATE_MMINFRA_CG0(CLK_MMINFRA_GCE_M, "mminfra_gce_m", "mminfra_ck", 1),
	GATE_MMINFRA_CG0(CLK_MMINFRA_SMI, "mminfra_smi", "clk26m", 2),
	/* MMINFRA_CONFIG1 */
	GATE_MMINFRA_CG1(CLK_MMINFRA_GCE_26M, "mminfra_gce_26m", "mminfra_ck", 17),
};

static const struct mtk_clk_desc mminfra_desc = {
	.clks = mminfra_clks,
	.num_clks = ARRAY_SIZE(mminfra_clks),
};

static const struct of_device_id of_match_clk_mt6895_mminfra[] = {
	{ .compatible = "mediatek,mt6895-mminfra_config", .data = &mminfra_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6895_mminfra);

static int mtk_mminfra_probe(struct platform_device *pdev)
{
	int ret = mtk_clk_simple_probe(pdev);
	void __iomem *base = ioremap(0x1e800000, 0x1000);

	dev_info(&pdev->dev, "XAGA-MMINFRA: probe ret=%d\n", ret);
	if (base) {
		dev_info(&pdev->dev,
			"XAGA-MMINFRA: CG_CON0(0x100)=0x%x CG_CON1(0x110)=0x%x (0=ungated)\n",
			readl(base + 0x100), readl(base + 0x110));
		iounmap(base);
	}
	return ret;
}

static struct platform_driver clk_mt6895_mminfra_drv = {
	.probe = mtk_mminfra_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6895-mminfra",
		.of_match_table = of_match_clk_mt6895_mminfra,
	},
};
module_platform_driver(clk_mt6895_mminfra_drv);

MODULE_DESCRIPTION("MediaTek MT6895 MMINFRA_CONFIG clocks driver");
MODULE_LICENSE("GPL");
