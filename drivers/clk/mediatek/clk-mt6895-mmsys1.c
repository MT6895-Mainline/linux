// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/mt6895-clk.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mm1_0_cg_regs = {
	.set_ofs = 0x104,
	.clr_ofs = 0x108,
	.sta_ofs = 0x100,
};

static const struct mtk_gate_regs mm1_1_cg_regs = {
	.set_ofs = 0x114,
	.clr_ofs = 0x118,
	.sta_ofs = 0x110,
};

static const struct mtk_gate_regs mm1_2_cg_regs = {
	.set_ofs = 0x1a4,
	.clr_ofs = 0x1a8,
	.sta_ofs = 0x1a0,
};

#define GATE_MM1_0(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm1_0_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_MM1_1(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm1_1_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

#define GATE_MM1_2(_id, _name, _parent, _shift)				\
	GATE_MTK(_id, _name, _parent, &mm1_2_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate mm1_clks[] = {
	/* MM1_0 */
	GATE_MM1_0(CLK_MM1_DISP_MUTEX0, "mm1_disp_mutex0", "disp1_ck", 0),
	GATE_MM1_0(CLK_MM1_DISP_OVL0, "mm1_disp_ovl0", "disp1_ck", 1),
	GATE_MM1_0(CLK_MM1_DISP_MERGE0, "mm1_disp_merge0", "disp1_ck", 2),
	GATE_MM1_0(CLK_MM1_DISP_FAKE_ENG0, "mm1_disp_fake_eng0", "disp1_ck", 3),
	GATE_MM1_0(CLK_MM1_INLINEROT0, "mm1_inlinerot0", "disp1_ck", 4),
	GATE_MM1_0(CLK_MM1_DISP_WDMA0, "mm1_disp_wdma0", "disp1_ck", 5),
	GATE_MM1_0(CLK_MM1_DISP_FAKE_ENG1, "mm1_disp_fake_eng1", "disp1_ck", 6),
	GATE_MM1_0(CLK_MM1_DISP_DPI0, "mm1_disp_dpi0", "disp1_ck", 7),
	GATE_MM1_0(CLK_MM1_DISP_OVL0_2L_NW, "mm1_disp_ovl0_2l_nw", "disp1_ck", 8),
	GATE_MM1_0(CLK_MM1_DISP_RDMA0, "mm1_disp_rdma0", "disp1_ck", 9),
	GATE_MM1_0(CLK_MM1_DISP_RDMA1, "mm1_disp_rdma1", "disp1_ck", 10),
	GATE_MM1_0(CLK_MM1_DISP_DLI_ASYNC0, "mm1_disp_dli_async0", "disp1_ck", 11),
	GATE_MM1_0(CLK_MM1_DISP_DLI_ASYNC1, "mm1_disp_dli_async1", "disp1_ck", 12),
	GATE_MM1_0(CLK_MM1_DISP_DLI_ASYNC2, "mm1_disp_dli_async2", "disp1_ck", 13),
	GATE_MM1_0(CLK_MM1_DISP_DLO_ASYNC0, "mm1_disp_dlo_async0", "disp1_ck", 14),
	GATE_MM1_0(CLK_MM1_DISP_DLO_ASYNC1, "mm1_disp_dlo_async1", "disp1_ck", 15),
	GATE_MM1_0(CLK_MM1_DISP_DLO_ASYNC2, "mm1_disp_dlo_async2", "disp1_ck", 16),
	GATE_MM1_0(CLK_MM1_DISP_RSZ0, "mm1_disp_rsz0", "disp1_ck", 17),
	GATE_MM1_0(CLK_MM1_DISP_COLOR0, "mm1_disp_color0", "disp1_ck", 18),
	GATE_MM1_0(CLK_MM1_DISP_CCORR0, "mm1_disp_ccorr0", "disp1_ck", 19),
	GATE_MM1_0(CLK_MM1_DISP_CCORR1, "mm1_disp_ccorr1", "disp1_ck", 20),
	GATE_MM1_0(CLK_MM1_DISP_AAL0, "mm1_disp_aal0", "disp1_ck", 21),
	GATE_MM1_0(CLK_MM1_DISP_GAMMA0, "mm1_disp_gamma0", "disp1_ck", 22),
	GATE_MM1_0(CLK_MM1_DISP_POSTMASK0, "mm1_disp_postmask0", "disp1_ck", 23),
	GATE_MM1_0(CLK_MM1_DISP_DITHER0, "mm1_disp_dither0", "disp1_ck", 24),
	GATE_MM1_0(CLK_MM1_DISP_CM0, "mm1_disp_cm0", "disp1_ck", 25),
	GATE_MM1_0(CLK_MM1_DISP_SPR0, "mm1_disp_spr0", "disp1_ck", 26),
	GATE_MM1_0(CLK_MM1_DISP_DSC_WRAP0, "mm1_disp_dsc_wrap0", "disp1_ck", 27),
	GATE_MM1_0(CLK_MM1_FMM_DISP_DSI0, "mm1_fmm_clk0", "disp1_ck", 29),
	GATE_MM1_0(CLK_MM1_DISP_UFBC_WDMA0, "mm1_disp_ufbc_wdma0", "disp1_ck", 30),
	GATE_MM1_0(CLK_MM1_DISP_WDMA1, "mm1_disp_wdma1", "disp1_ck", 31),
	/* MM1_1 */
	GATE_MM1_1(CLK_MM1_FMM_DISP_DP_INTF0, "mm1_fmm_dp_clk", "disp1_ck", 0),
	GATE_MM1_1(CLK_MM1_APB_BUS, "mm1_apb_bus", "disp1_ck", 1),
	GATE_MM1_1(CLK_MM1_DISP_TDSHP0, "mm1_disp_tdshp0", "disp1_ck", 2),
	GATE_MM1_1(CLK_MM1_DISP_C3D0, "mm1_disp_c3d0", "disp1_ck", 3),
	GATE_MM1_1(CLK_MM1_DISP_Y2R0, "mm1_disp_y2r0", "disp1_ck", 4),
	GATE_MM1_1(CLK_MM1_MDP_AAL0, "mm1_mdp_aal0", "disp1_ck", 5),
	GATE_MM1_1(CLK_MM1_DISP_CHIST0, "mm1_disp_chist0", "disp1_ck", 6),
	GATE_MM1_1(CLK_MM1_DISP_CHIST1, "mm1_disp_chist1", "disp1_ck", 7),
	GATE_MM1_1(CLK_MM1_DISP_OVL0_2L, "mm1_disp_ovl0_2l", "disp1_ck", 8),
	GATE_MM1_1(CLK_MM1_DISP_DLI_ASYNC3, "mm1_disp_dli_async3", "disp1_ck", 9),
	GATE_MM1_1(CLK_MM1_DISP_DLO_ASYNC3, "mm1_disp_dlo_async3", "disp1_ck", 10),
	GATE_MM1_1(CLK_MM1_DISP_OVL1_2L, "mm1_disp_ovl1_2l", "disp1_ck", 12),
	GATE_MM1_1(CLK_MM1_DISP_OVL1_2L_NW, "mm1_disp_ovl1_2l_nw", "disp1_ck", 16),
	GATE_MM1_1(CLK_MM1_SMI_COMMON, "mm1_smi_common", "disp1_ck", 20),
	/* MM1_2 */
	GATE_MM1_2(CLK_MM1_DISP_DSI, "mm1_clk", "disp1_ck", 0),
	GATE_MM1_2(CLK_MM1_DISP_DP_INTF0, "mm1_dp_clk", "disp1_ck", 1),
	GATE_MM1_2(CLK_MM1_SIG_EMI, "mm1_sig_emi", "hf_fdisp2_ck", 11),
};

static const struct mtk_clk_desc mm1_desc = {
	.clks = mm1_clks,
	.num_clks = ARRAY_SIZE(mm1_clks),
};

static const struct platform_device_id clk_mt6895_mmsys1_id_table[] = {
	{ .name = "clk-mt6895-mmsys1", .driver_data = (kernel_ulong_t)&mm1_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, clk_mt6895_mmsys1_id_table);

static const struct of_device_id of_match_clk_mt6895_mmsys1[] = {
	{ .compatible = "mediatek,mt6895-mmsys1", .data = &mm1_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6895_mmsys1);

static struct platform_driver clk_mt6895_mmsys1_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6895-mmsys1",
		.of_match_table = of_match_clk_mt6895_mmsys1,
	},
	.id_table = clk_mt6895_mmsys1_id_table,
};
module_platform_driver(clk_mt6895_mmsys1_drv);

MODULE_DESCRIPTION("MediaTek MT6895 MMSYS1 clocks driver");
MODULE_LICENSE("GPL");