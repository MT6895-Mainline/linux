// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/mt6895-clk.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mm0_0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mm0_1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

static const struct mtk_gate_regs mm0_2_cg_regs = {
	.set_ofs = 0x1a4,
	.clr_ofs = 0x1a8,
	.sta_ofs = 0x1a0,
};

#define GATE_MM0_0(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm0_0_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_MM0_1(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm0_1_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_MM0_2(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm0_2_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate mm0_clks[] = {
	/* MM0_0 */
	GATE_MM0_0(CLK_MM0_DISP_MUTEX0, "mm0_disp_mutex0", "disp0_ck", 0),
	GATE_MM0_0(CLK_MM0_DISP_OVL0, "mm0_disp_ovl0", "disp0_ck", 1),
	GATE_MM0_0(CLK_MM0_DISP_MERGE0, "mm0_disp_merge0", "disp0_ck", 2),
	GATE_MM0_0(CLK_MM0_DISP_FAKE_ENG0, "mm0_disp_fake_eng0", "disp0_ck", 3),
	GATE_MM0_0(CLK_MM0_INLINEROT0, "mm0_inlinerot0", "disp0_ck", 4),
	GATE_MM0_0(CLK_MM0_DISP_WDMA0, "mm0_disp_wdma0", "disp0_ck", 5),
	GATE_MM0_0(CLK_MM0_DISP_FAKE_ENG1, "mm0_disp_fake_eng1", "disp0_ck", 6),
	GATE_MM0_0(CLK_MM0_DISP_DPI0, "mm0_disp_dpi0", "disp0_ck", 7),
	GATE_MM0_0(CLK_MM0_DISP_OVL0_2L_NW, "mm0_disp_ovl0_2l_nw", "disp0_ck", 8),
	GATE_MM0_0(CLK_MM0_DISP_RDMA0, "mm0_disp_rdma0", "disp0_ck", 9),
	GATE_MM0_0(CLK_MM0_DISP_RDMA1, "mm0_disp_rdma1", "disp0_ck", 10),
	GATE_MM0_0(CLK_MM0_DISP_DLI_ASYNC0, "mm0_disp_dli_async0", "disp0_ck", 11),
	GATE_MM0_0(CLK_MM0_DISP_DLI_ASYNC1, "mm0_disp_dli_async1", "disp0_ck", 12),
	GATE_MM0_0(CLK_MM0_DISP_DLI_ASYNC2, "mm0_disp_dli_async2", "disp0_ck", 13),
	GATE_MM0_0(CLK_MM0_DISP_DLO_ASYNC0, "mm0_disp_dlo_async0", "disp0_ck", 14),
	GATE_MM0_0(CLK_MM0_DISP_DLO_ASYNC1, "mm0_disp_dlo_async1", "disp0_ck", 15),
	GATE_MM0_0(CLK_MM0_DISP_DLO_ASYNC2, "mm0_disp_dlo_async2", "disp0_ck", 16),
	GATE_MM0_0(CLK_MM0_DISP_RSZ0, "mm0_disp_rsz0", "disp0_ck", 17),
	GATE_MM0_0(CLK_MM0_DISP_COLOR0, "mm0_disp_color0", "disp0_ck", 18),
	GATE_MM0_0(CLK_MM0_DISP_CCORR0, "mm0_disp_ccorr0", "disp0_ck", 19),
	GATE_MM0_0(CLK_MM0_DISP_CCORR1, "mm0_disp_ccorr1", "disp0_ck", 20),
	GATE_MM0_0(CLK_MM0_DISP_AAL0, "mm0_disp_aal0", "disp0_ck", 21),
	GATE_MM0_0(CLK_MM0_DISP_GAMMA0, "mm0_disp_gamma0", "disp0_ck", 22),
	GATE_MM0_0(CLK_MM0_DISP_POSTMASK0, "mm0_disp_postmask0", "disp0_ck", 23),
	GATE_MM0_0(CLK_MM0_DISP_DITHER0, "mm0_disp_dither0", "disp0_ck", 24),
	GATE_MM0_0(CLK_MM0_DISP_CM0, "mm0_disp_cm0", "disp0_ck", 25),
	GATE_MM0_0(CLK_MM0_DISP_SPR0, "mm0_disp_spr0", "disp0_ck", 26),
	GATE_MM0_0(CLK_MM0_DISP_DSC_WRAP0, "mm0_disp_dsc_wrap0", "disp0_ck", 27),
	GATE_MM0_0(CLK_MM0_FMM_DISP_DSI0, "mm0_fmm_clk0", "disp0_ck", 29),
	GATE_MM0_0(CLK_MM0_DISP_UFBC_WDMA0, "mm0_disp_ufbc_wdma0", "disp0_ck", 30),
	GATE_MM0_0(CLK_MM0_DISP_WDMA1, "mm0_disp_wdma1", "disp0_ck", 31),
	/* MM0_1 */
	GATE_MM0_1(CLK_MM0_FMM_DISP_DP_INTF0, "mm0_fmm_dp_clk", "disp0_ck", 0),
	GATE_MM0_1(CLK_MM0_APB_BUS, "mm0_apb_bus", "disp0_ck", 1),
	GATE_MM0_1(CLK_MM0_DISP_TDSHP0, "mm0_disp_tdshp0", "disp0_ck", 2),
	GATE_MM0_1(CLK_MM0_DISP_C3D0, "mm0_disp_c3d0", "disp0_ck", 3),
	GATE_MM0_1(CLK_MM0_DISP_Y2R0, "mm0_disp_y2r0", "disp0_ck", 4),
	GATE_MM0_1(CLK_MM0_MDP_AAL0, "mm0_mdp_aal0", "disp0_ck", 5),
	GATE_MM0_1(CLK_MM0_DISP_CHIST0, "mm0_disp_chist0", "disp0_ck", 6),
	GATE_MM0_1(CLK_MM0_DISP_CHIST1, "mm0_disp_chist1", "disp0_ck", 7),
	GATE_MM0_1(CLK_MM0_DISP_OVL0_2L, "mm0_disp_ovl0_2l", "disp0_ck", 8),
	GATE_MM0_1(CLK_MM0_DISP_DLI_ASYNC3, "mm0_disp_dli_async3", "disp0_ck", 9),
	GATE_MM0_1(CLK_MM0_DISP_DLO_ASYNC3, "mm0_disp_dlo_async3", "disp0_ck", 10),
	GATE_MM0_1(CLK_MM0_DISP_OVL1_2L, "mm0_disp_ovl1_2l", "disp0_ck", 12),
	GATE_MM0_1(CLK_MM0_DISP_OVL1_2L_NW, "mm0_disp_ovl1_2l_nw", "disp0_ck", 16),
	GATE_MM0_1(CLK_MM0_SMI_COMMON, "mm0_smi_common", "disp0_ck", 20),
	/* MM0_2 */
	GATE_MM0_2(CLK_MM0_DISP_DSI, "mm0_clk", "disp0_ck", 0),
	GATE_MM0_2(CLK_MM0_DISP_DP_INTF0, "mm0_dp_clk", "disp0_ck", 1),
	GATE_MM0_2(CLK_MM0_SIG_EMI, "mm0_sig_emi", "disp0_ck", 11),
};

static const struct mtk_clk_desc mm0_desc = {
	.clks = mm0_clks,
	.num_clks = ARRAY_SIZE(mm0_clks),
};

static const struct platform_device_id clk_mt6895_mmsys0_id_table[] = {
	{ .name = "clk-mt6895-mmsys0", .driver_data = (kernel_ulong_t)&mm0_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, clk_mt6895_mmsys0_id_table);

static const struct of_device_id of_match_clk_mt6895_mmsys0[] = {
	{ .compatible = "mediatek,mt6895-mmsys0", .data = &mm0_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6895_mmsys0);

static struct platform_driver clk_mt6895_mmsys0_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6895-mmsys0",
		.of_match_table = of_match_clk_mt6895_mmsys0,
	},
	.id_table = clk_mt6895_mmsys0_id_table,
};
module_platform_driver(clk_mt6895_mmsys0_drv);

MODULE_DESCRIPTION("MediaTek MT6895 MMSYS0 clocks driver");
MODULE_LICENSE("GPL");
