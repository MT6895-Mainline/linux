// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2021 MediaTek Inc.
// Author: Ren-Ting Wang <ren-ting.wang@mediatek.com>

#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "clk-mtk.h"
#include "clk-gate.h"

#include <dt-bindings/clock/mt6895-clk.h>

void mt6895_mfg_bringup_enable(void);
void mt6895_mfg_pdc_enable_now(void);

#define MT_CCF_BRINGUP		1

/* Regular Number Definition */
#define INV_OFS			-1
#define INV_BIT			-1

static const struct mtk_gate_regs mfgcfg_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_MFGCFG(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &mfgcfg_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_setclr,	\
	}

static const struct mtk_gate mfgcfg_clks[] = {
	GATE_MFGCFG(CLK_MFGCFG_BG3D, "mfgcfg_bg3d",
			"mfg_ref_ck"/* parent */, 0),
};

static const struct mtk_clk_desc mfgcfg_mcd = {
	.clks = mfgcfg_clks,
	.num_clks = CLK_MFGCFG_NR_CLK,
};

static void mt6895_mfg_pdc_enable(void __iomem *base)
{
	static const u32 en_regs[] = {
		0x400, 0x418, 0x430, 0x448, 0x460, 0x478,
		0x490, 0x4a8, 0x4c0, 0x4d8,
		0x100, 0x120, 0x140, 0x118, 0x0c0, 0x098, 0x1c0,
	};
	static const u32 rsv_regs[] = {
		0x404, 0x41c, 0x434, 0x44c, 0x464, 0x47c,
		0x494, 0x4ac, 0x4c4, 0x4dc,
	};
	u32 val;
	int i;

	/* Port of downstream gpufreq __gpufreq_pdc_control(POWER_ON). */
	for (i = 0; i < ARRAY_SIZE(en_regs); i++) {
		val = readl(base + en_regs[i]);
		val |= BIT(0);
		writel(val, base + en_regs[i]);
	}
	for (i = 0; i < ARRAY_SIZE(rsv_regs); i++) {
		val = readl(base + rsv_regs[i]);
		val |= BIT(31);
		writel(val, base + rsv_regs[i]);
	}
}

static void mt6895_mfg_hwdcm_enable(void __iomem *top)
{
	void __iomem *rpc;
	u32 val;

	/* Direct ioremap: the DT resource translation for the second MFG_RPC
	 * range is unreliable during early boot on this port.
	 */
	rpc = ioremap(0x13f90000, 0x10000);
	if (!rpc)
		return;

	/* Port of downstream gpufreq __gpufreq_hw_dcm_control(). */
	val = readl(top + 0x10);
	val |= BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5);
	val &= ~BIT(6);
	val |= BIT(15);
	writel(val, top + 0x10);

	val = readl(top + 0x20);
	val |= BIT(23) | BIT(25);
	writel(val, top + 0x20);

	val = readl(top + 0xb0);
	val &= ~BIT(8);
	val &= ~BIT(10);
	val |= BIT(13) | BIT(14) | BIT(17) | BIT(18);
	val &= ~BIT(21);
	writel(val, top + 0xb0);

	val = readl(rpc + 0x1034);
	val &= ~BIT(0);
	writel(val, rpc + 0x1034);

	iounmap(rpc);
}

static void mt6895_mfg_acp_enable(void __iomem *top)
{
	u32 val;

	/* Port of downstream gpufreq __gpufreq_acp_control() (MFG_TOP only). */
	val = readl(top + 0x168);
	val |= BIT(0) | BIT(1) | BIT(2) | BIT(3);
	writel(val, top + 0x168);

	val = readl(top + 0x8e0);
	val |= 0x855;
	writel(val, top + 0x8e0);

	val = readl(top + 0x8e8);
	val |= 0x855;
	writel(val, top + 0x8e8);

	val = readl(top + 0x910);
	val |= 0x855;
	writel(val, top + 0x910);

	val = readl(top + 0x918);
	val |= 0x855;
	writel(val, top + 0x918);

	val = readl(top + 0x900);
	val |= 0x055;
	writel(val, top + 0x900);

	val = readl(top + 0x908);
	val |= 0x055;
	writel(val, top + 0x908);

	val = readl(top + 0x920);
	val |= 0x055;
	writel(val, top + 0x920);

	val = readl(top + 0x928);
	val |= 0x055;
	writel(val, top + 0x928);
}

/* GPM1.0: di/dt reduction / MFG I2M protector. Downstream enables this by
 * default (g_gpm_enable=true) and programs it on every GPU power-on.
 */
static void mt6895_mfg_gpm_enable(void __iomem *base)
{
	writel(0x20300316, base + 0xF60);
	writel(0x1800000C, base + 0xF64);
	writel(0x01010802, base + 0xF68);
	writel(0x000227F3, base + 0xFA8);

	udelay(1);

	writel(0x20300317, base + 0xF60);
}

void mt6895_mfg_bringup_enable(void)
{
	void __iomem *base, *sleep;
	u32 val;

	/* Re-apply after the MFG1 power domain has been brought up.
	 * MFG_TOP (0x13fbf000) lives inside the MFG power domain; writes done
	 * by clk_mt6895_mfgcfg_probe() before scpsys powers MFG1 may be lost.
	 * Downstream gpufreq does AOC/PDC/ACP/HWDCM after MFG MTCMOS is on.
	 */
	base = ioremap(0x13fbf000, 0x1000);
	if (!base)
		return;

	/* AOC2.0: release VGPU SRAM isolation with the SPM HW semaphore,
	 * mirroring downstream __gpufreq_aoc_control(POWER_ON).
	 */
	sleep = ioremap(0x1c001000, 0x1000);
	if (sleep) {
		do {
			val = readl(sleep + 0x6ac);
			val |= BIT(0);
			writel(val, sleep + 0x6ac);
		} while ((readl(sleep + 0x6ac) & BIT(0)) != BIT(0));

		val = readl(sleep + 0xf30);
		val &= ~(BIT(9) | BIT(10));
		writel(val, sleep + 0xf30);

		val = readl(sleep + 0x6ac);
		val |= BIT(0);
		writel(val, sleep + 0x6ac);

		iounmap(sleep);
	}

	/* XAGA EXPERIMENT: do NOT enable PDCv2 here. With PDC enabled, the GPU
	 * takes over MFG4-12 and powers them off because no shader power request
	 * is active yet, leaving the domains stuck in 0x100d (requested but no
	 * PWR_ACK). Keep them software-powered via scpsys instead.
	 */
	/* mt6895_mfg_pdc_enable(base); */
	mt6895_mfg_hwdcm_enable(base);
	mt6895_mfg_acp_enable(base);
	mt6895_mfg_gpm_enable(base);

	/* Downstream gpufreq also restores MFG_DUMMY_REG and
	 * MFG_SRAM_FUL_SEL_ULV after power-on.  On a cold boot the saved values
	 * are zero, so write zeros to match the kbase cold-boot path.
	 */
	writel(0, base + 0x500);
	writel(0, base + 0x80);

	pr_info_once("XAGA-MFG: MFG_TOP ACP/HWDCM/GPM re-applied after MFG1 power-on (PDC SKIPPED, PDC0=%#x PDC0_RSV=%#x ACP0=%#x)\n",
		     readl(base + 0x400), readl(base + 0x404), readl(base + 0x168));
	iounmap(base);
}
EXPORT_SYMBOL_GPL(mt6895_mfg_bringup_enable);

/* Enable PDCv2 after the shader power request has been asserted, so the GPU
 * power state machine can complete the shader MTCMOS handshake without the
 * PDC immediately powering the domains off (no request was active yet).
 */
void mt6895_mfg_pdc_enable_now(void)
{
	void __iomem *base;

	base = ioremap(0x13fbf000, 0x1000);
	if (!base)
		return;

	mt6895_mfg_pdc_enable(base);
	pr_info_once("XAGA-MFG: PDCv2 enabled after shader power request (PDC0=%#x PDC0_RSV=%#x)\n",
		     readl(base + 0x400), readl(base + 0x404));
	iounmap(base);
}
EXPORT_SYMBOL_GPL(mt6895_mfg_pdc_enable_now);

static int clk_mt6895_mfgcfg_probe(struct platform_device *pdev)
{
	void __iomem *base;
	int r;

#if MT_CCF_BRINGUP
	pr_notice("%s init begin\n", __func__);
#endif

	base = of_iomap(pdev->dev.of_node, 0);
	if (!base)
		return -ENOMEM;

	mt6895_mfg_pdc_enable(base);
	mt6895_mfg_hwdcm_enable(base);
	mt6895_mfg_acp_enable(base);

	r = mtk_clk_simple_probe(pdev);
	if (r)
		dev_err(&pdev->dev,
			"could not register clock provider: %s: %d\n",
			pdev->name, r);

#if MT_CCF_BRINGUP
	pr_notice("%s init end\n", __func__);
#endif

	return r;
}

static const struct of_device_id of_match_clk_mt6895_mfgcfg[] = {
	{
		.compatible = "mediatek,mt6895-mfg",
		.data = &mfgcfg_mcd,
	},
	{}
};

static struct platform_driver clk_mt6895_mfgcfg_drv = {
	.probe = clk_mt6895_mfgcfg_probe,
	.driver = {
		.name = "clk-mt6895-mfgcfg",
		.of_match_table = of_match_clk_mt6895_mfgcfg,
	},
};

static int __init clk_mt6895_mfgcfg_init(void)
{
	return platform_driver_register(&clk_mt6895_mfgcfg_drv);
}

static void __exit clk_mt6895_mfgcfg_exit(void)
{
	platform_driver_unregister(&clk_mt6895_mfgcfg_drv);
}

arch_initcall(clk_mt6895_mfgcfg_init);
module_exit(clk_mt6895_mfgcfg_exit);
MODULE_LICENSE("GPL");
