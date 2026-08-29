// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 * Author: Ren-Ting Wang <ren-ting.wang@mediatek.com>
 *
 * Ported from downstream clk-mt6895-adsp.c (compatible "mediatek,mt6895-afe").
 * Provides the AUDIO_TOP_CON0/1/2 clock gates consumed by the MT6895 AFE
 * (aud_afe_clk, aud_dac_clk, ... aud_3rd_dac_hires_clk).
 *
 * Gate bits (reg offset / bit) verified identical to the downstream table;
 * gate names follow the consumer-facing names used by mt6895-afe-clk.c and
 * the AFE device tree node.
 */
#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/mt6895-clk.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs afe0_cg_regs = {
	.set_ofs = 0x0,
	.clr_ofs = 0x0,
	.sta_ofs = 0x0,
};

static const struct mtk_gate_regs afe1_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x4,
	.sta_ofs = 0x4,
};

static const struct mtk_gate_regs afe2_cg_regs = {
	.set_ofs = 0x8,
	.clr_ofs = 0x8,
	.sta_ofs = 0x8,
};

#define GATE_AFE0(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe0_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

#define GATE_AFE1(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe1_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

#define GATE_AFE2(_id, _name, _parent, _shift) {	\
		.id = _id,				\
		.name = _name,				\
		.parent_name = _parent,			\
		.regs = &afe2_cg_regs,			\
		.shift = _shift,			\
		.ops = &mtk_clk_gate_ops_no_setclr,	\
	}

static const struct mtk_gate afe_clks[] = {
	/* AFE0: AUDIO_TOP_CON0 */
	GATE_AFE0(CLK_AFE_AFE, "aud_afe_clk", "audio_ck", 2),
	GATE_AFE0(CLK_AFE_22M, "aud_apll22m_clk", "aud_engen1_ck", 8),
	GATE_AFE0(CLK_AFE_24M, "aud_apll24m_clk", "aud_engen2_ck", 9),
	GATE_AFE0(CLK_AFE_APLL2_TUNER, "aud_apll2_tuner_clk", "aud_engen2_ck", 18),
	GATE_AFE0(CLK_AFE_APLL_TUNER, "aud_apll1_tuner_clk", "aud_engen1_ck", 19),
	GATE_AFE0(CLK_AFE_TDM, "aud_tdm_clk", "aud_1_ck", 20),
	GATE_AFE0(CLK_AFE_ADC, "aud_adc_clk", "audio_ck", 24),
	GATE_AFE0(CLK_AFE_DAC, "aud_dac_clk", "audio_ck", 25),
	GATE_AFE0(CLK_AFE_DAC_PREDIS, "aud_dac_predis_clk", "audio_ck", 26),
	GATE_AFE0(CLK_AFE_TML, "aud_tml_clk", "audio_ck", 27),
	GATE_AFE0(CLK_AFE_NLE, "aud_nle", "audio_ck", 28),
	/* AFE1: AUDIO_TOP_CON1 */
	GATE_AFE1(CLK_AFE_GENERAL3_ASRC, "aud_general3_asrc", "audio_ck", 11),
	GATE_AFE1(CLK_AFE_CONNSYS_I2S_ASRC, "aud_connsys_i2s_asrc", "audio_ck", 12),
	GATE_AFE1(CLK_AFE_GENERAL1_ASRC, "aud_general1_asrc", "audio_ck", 13),
	GATE_AFE1(CLK_AFE_GENERAL2_ASRC, "aud_general2_asrc", "audio_ck", 14),
	GATE_AFE1(CLK_AFE_DAC_HIRES, "aud_dac_hires_clk", "audio_h_ck", 15),
	GATE_AFE1(CLK_AFE_ADC_HIRES, "aud_adc_hires_clk", "audio_h_ck", 16),
	GATE_AFE1(CLK_AFE_ADC_HIRES_TML, "aud_adc_hires_tml", "audio_h_ck", 17),
	GATE_AFE1(CLK_AFE_ADDA6_ADC, "aud_adda6_adc_clk", "audio_ck", 20),
	GATE_AFE1(CLK_AFE_ADDA6_ADC_HIRES, "aud_adda6_adc_hires_clk", "audio_h_ck", 21),
	GATE_AFE1(CLK_AFE_3RD_DAC, "aud_3rd_dac_clk", "audio_ck", 28),
	GATE_AFE1(CLK_AFE_3RD_DAC_PREDIS, "aud_3rd_dac_predis_clk", "audio_ck", 29),
	GATE_AFE1(CLK_AFE_3RD_DAC_TML, "aud_3rd_dac_tml", "audio_ck", 30),
	GATE_AFE1(CLK_AFE_3RD_DAC_HIRES, "aud_3rd_dac_hires_clk", "audio_h_ck", 31),
	/* AFE2: AUDIO_TOP_CON2 */
	GATE_AFE2(CLK_AFE_I2S5_BCLK, "aud_i2s5_bclk", "audio_ck", 0),
	GATE_AFE2(CLK_AFE_I2S6_BCLK, "aud_i2s6_bclk", "audio_ck", 1),
	GATE_AFE2(CLK_AFE_I2S7_BCLK, "aud_i2s7_bclk", "audio_ck", 2),
	GATE_AFE2(CLK_AFE_I2S8_BCLK, "aud_i2s8_bclk", "audio_ck", 3),
	GATE_AFE2(CLK_AFE_I2S9_BCLK, "aud_i2s9_bclk", "audio_ck", 4),
	GATE_AFE2(CLK_AFE_ETDM_IN0_BCLK, "aud_etdm_in0_bclk", "audio_ck", 5),
	GATE_AFE2(CLK_AFE_ETDM_OUT0_BCLK, "aud_etdm_out0_bclk", "audio_ck", 6),
	GATE_AFE2(CLK_AFE_I2S1_BCLK, "aud_i2s1_bclk", "audio_ck", 7),
	GATE_AFE2(CLK_AFE_I2S2_BCLK, "aud_i2s2_bclk", "audio_ck", 8),
	GATE_AFE2(CLK_AFE_I2S3_BCLK, "aud_i2s3_bclk", "audio_ck", 9),
	GATE_AFE2(CLK_AFE_I2S4_BCLK, "aud_i2s4_bclk", "audio_ck", 10),
	GATE_AFE2(CLK_AFE_ETDM_IN1_BCLK, "aud_etdm_in1_bclk", "audio_ck", 23),
	GATE_AFE2(CLK_AFE_ETDM_OUT1_BCLK, "aud_etdm_out1_bclk", "audio_ck", 24),
};

static const struct mtk_clk_desc afe_desc = {
	.clks = afe_clks,
	.num_clks = CLK_AFE_NR_CLK,
};

static const struct of_device_id of_match_clk_mt6895_afe[] = {
	{ .compatible = "mediatek,mt6895-afe", .data = &afe_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6895_afe);

static struct platform_driver clk_mt6895_afe_drv = {
	.probe = mtk_clk_simple_probe,
	.remove = mtk_clk_simple_remove,
	.driver = {
		.name = "clk-mt6895-afe",
		.of_match_table = of_match_clk_mt6895_afe,
	},
};
module_platform_driver(clk_mt6895_afe_drv);

MODULE_DESCRIPTION("MediaTek MT6895 audio (AFE) clocks driver");
MODULE_LICENSE("GPL");
