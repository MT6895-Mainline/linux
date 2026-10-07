// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022 MediaTek Inc.

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>

#include "../mtk_cam-seninf.h"
#include "../mtk_cam-seninf-hw.h"
#include "../mtk_cam-seninf-regs.h"
#include "mtk_cam-seninf-top-ctrl.h"
#include "mtk_cam-seninf-seninf1-mux.h"
#include "mtk_cam-seninf-seninf1.h"
#include "mtk_cam-seninf-seninf1-csi2.h"
#include "mtk_cam-seninf-tg1.h"
#include "mtk_cam-seninf-cammux.h"
#include "mtk_cam-seninf-mipi-rx-ana-cdphy-csi0a.h"
#include "mtk_cam-seninf-csi0-cphy.h"
#include "mtk_cam-seninf-csi0-dphy.h"
#include "../kd_imgsensor_define_v4l2.h"

static struct mtk_cam_seninf_cfg _seninf_cfg = {
	.mux_num = 8,
	.seninf_num = 4,
	.cam_mux_num = 11,
	.pref_mux_num = 11,
};

struct mtk_cam_seninf_cfg *g_seninf_cfg = &_seninf_cfg;

static inline u32 get_port_num(int port)
{
    if (port >= CSI_PORT_0A)
        return FIELD_GET(CSI_PORT_A_MASK, port - CSI_PORT_0A);
    else
        return port;
}

static inline void mtk_cam_seninf_set_di_ch_ctrl(void __iomem *pseninf,
						 unsigned int stream_id,
						 struct seninf_vc *vc)
{
	if (stream_id > SENINF_MAX_STREAM)
		return;

	SENINF_BITS(pseninf, SENINF_CSI2_S0_DI_CTRL + (stream_id << 0x2),
		    RG_CSI2_DT_SEL, vc->dt);
	SENINF_BITS(pseninf, SENINF_CSI2_S0_DI_CTRL + (stream_id << 0x2),
		    RG_CSI2_VC_SEL, 0);
	SENINF_BITS(pseninf, SENINF_CSI2_S0_DI_CTRL + (stream_id << 0x2),
		    RG_CSI2_DT_INTERLEAVE_MODE, 1);
	SENINF_BITS(pseninf, SENINF_CSI2_S0_DI_CTRL + (stream_id << 0x2),
		    RG_CSI2_VC_INTERLEAVE_EN, 1);

	switch (stream_id) {
	case 0:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S0_GRP_EN, 1);
		break;
	case 1:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S1_GRP_EN, 1);
		break;
	case 2:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S2_GRP_EN, 1);
		break;
	case 3:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S3_GRP_EN, 1);
		break;
	case 4:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S4_GRP_EN, 1);
		break;
	case 5:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S5_GRP_EN, 1);
		break;
	case 6:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S6_GRP_EN, 1);
		break;
	case 7:
		SENINF_BITS(pseninf, SENINF_CSI2_CH0_CTRL,
			    RG_CSI2_S7_GRP_EN, 1);
		break;
	default:
		return;
	}
}

int mtk_cam_seninf_init_iomem(struct seninf_ctx *ctx, void __iomem *if_base,
			      void __iomem *ana_base)
{
	u32 i;

	// reg_ana_csi_rx
	ctx->reg_ana_csi_rx[CSI_PORT_0]  = ana_base + ANA_CSI_RX_PORT0_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_0A] = ana_base + ANA_CSI_RX_PORT0_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_0B] = ana_base + ANA_CSI_RX_PORT0B_OFFSET;

	ctx->reg_ana_csi_rx[CSI_PORT_1]  = ana_base + ANA_CSI_RX_PORT1_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_1A] = ana_base + ANA_CSI_RX_PORT1_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_1B] = ana_base + ANA_CSI_RX_PORT1B_OFFSET;

	ctx->reg_ana_csi_rx[CSI_PORT_2]  = ana_base + ANA_CSI_RX_PORT2_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_2A] = ana_base + ANA_CSI_RX_PORT2_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_2B] = ana_base + ANA_CSI_RX_PORT2B_OFFSET;

	ctx->reg_ana_csi_rx[CSI_PORT_3]  = ana_base + ANA_CSI_RX_PORT3_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_3A] = ana_base + ANA_CSI_RX_PORT3_OFFSET;
	ctx->reg_ana_csi_rx[CSI_PORT_3B] = ana_base + ANA_CSI_RX_PORT3B_OFFSET;

	// reg_ana_dphy_top
	ctx->reg_ana_dphy_top[CSI_PORT_0]  = ana_base + ANA_DPHY_TOP_PORT0_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_0A] = ana_base + ANA_DPHY_TOP_PORT0_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_0B] = ana_base + ANA_DPHY_TOP_PORT0_OFFSET;

	ctx->reg_ana_dphy_top[CSI_PORT_1]  = ana_base + ANA_DPHY_TOP_PORT1_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_1A] = ana_base + ANA_DPHY_TOP_PORT1_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_1B] = ana_base + ANA_DPHY_TOP_PORT1_OFFSET;

	ctx->reg_ana_dphy_top[CSI_PORT_2]  = ana_base + ANA_DPHY_TOP_PORT2_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_2A] = ana_base + ANA_DPHY_TOP_PORT2_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_2B] = ana_base + ANA_DPHY_TOP_PORT2_OFFSET;

	ctx->reg_ana_dphy_top[CSI_PORT_3]  = ana_base + ANA_DPHY_TOP_PORT3_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_3A] = ana_base + ANA_DPHY_TOP_PORT3_OFFSET;
	ctx->reg_ana_dphy_top[CSI_PORT_3B] = ana_base + ANA_DPHY_TOP_PORT3_OFFSET;

	// reg_ana_cphy_top
	ctx->reg_ana_cphy_top[CSI_PORT_0]  = ana_base + ANA_CPHY_TOP_PORT0_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_0A] = ana_base + ANA_CPHY_TOP_PORT0_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_0B] = ana_base + ANA_CPHY_TOP_PORT0_OFFSET;

	ctx->reg_ana_cphy_top[CSI_PORT_1]  = ana_base + ANA_CPHY_TOP_PORT1_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_1A] = ana_base + ANA_CPHY_TOP_PORT1_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_1B] = ana_base + ANA_CPHY_TOP_PORT1_OFFSET;

	ctx->reg_ana_cphy_top[CSI_PORT_2]  = ana_base + ANA_CPHY_TOP_PORT2_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_2A] = ana_base + ANA_CPHY_TOP_PORT2_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_2B] = ana_base + ANA_CPHY_TOP_PORT2_OFFSET;

	ctx->reg_ana_cphy_top[CSI_PORT_3]  = ana_base + ANA_CPHY_TOP_PORT3_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_3A] = ana_base + ANA_CPHY_TOP_PORT3_OFFSET;
	ctx->reg_ana_cphy_top[CSI_PORT_3B] = ana_base + ANA_CPHY_TOP_PORT3_OFFSET;

	ctx->reg_if_top = if_base;

	for (i = SENINF_1; i < _seninf_cfg.seninf_num; i++) {
		ctx->reg_if_ctrl[i]  = if_base + IF_CTRL_BASE_OFFSET  + (IF_PORT_STRIDE * i);
		ctx->reg_if_tg[i]    = if_base + IF_TG_BASE_OFFSET    + (IF_PORT_STRIDE * i);
		ctx->reg_if_csi2[i]  = if_base + IF_CSI2_BASE_OFFSET  + (IF_PORT_STRIDE * i);
	}

	for (i = SENINF_MUX1; i < _seninf_cfg.mux_num; i++)
		ctx->reg_if_mux[i] = if_base + IF_MUX_BASE_OFFSET + (IF_PORT_STRIDE * i);

	ctx->reg_if_cam_mux = if_base + IF_CAM_MUX_OFFSET;

	return 0;
}
int mtk_cam_seninf_init_port(struct seninf_ctx *ctx, int port)
{
	u32 port_num;

	port_num = get_port_num(port);

	ctx->port = port;
	ctx->port_num = port_num;
	ctx->port_a = CSI_PORT_0A + (port_num << 1);
	ctx->port_b = ctx->port_a + 1;
	ctx->is_4d1c = (port == port_num);

	ctx->seninf_idx = seninf_csi[port];

	return 0;
}

int mtk_cam_seninf_is_cammux_used(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;
	u32 temp = readl(seninf_cam_mux + SENINF_CAM_MUX_EN);

	return !!(temp & (1 << cam_mux));
}

int mtk_cam_seninf_cammux(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;
	u32 temp;

	temp = readl(seninf_cam_mux + SENINF_CAM_MUX_EN);
	SENINF_WRITE_REG(seninf_cam_mux, SENINF_CAM_MUX_EN,
			 temp | BIT(cam_mux));

	SENINF_WRITE_REG(seninf_cam_mux, SENINF_CAM_MUX_IRQ_STATUS,
			 3 << (cam_mux * 2)); /* clr irq */

	dev_dbg(ctx->dev, "cam_mux %d EN 0x%x IRQ_EN 0x%x IRQ_STATUS 0x%x\n",
		cam_mux, readl(seninf_cam_mux + SENINF_CAM_MUX_EN),
		readl(seninf_cam_mux + SENINF_CAM_MUX_IRQ_EN),
		readl(seninf_cam_mux + SENINF_CAM_MUX_IRQ_STATUS));

	return 0;
}

int mtk_cam_seninf_disable_cammux(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *base = ctx->reg_if_cam_mux;
	u32 temp;

	temp = readl(base + SENINF_CAM_MUX_EN);

	if ((1 << cam_mux) & temp) {
		SENINF_WRITE_REG(base, SENINF_CAM_MUX_EN,
				 temp & (~(1 << cam_mux)));

		dev_dbg(ctx->dev, "cammux %d EN %x IRQ_EN %x IRQ_STATUS %x",
			cam_mux, readl(base + SENINF_CAM_MUX_EN),
			readl(base + SENINF_CAM_MUX_IRQ_EN),
			readl(base + SENINF_CAM_MUX_IRQ_STATUS));
	}

	return 0;
}

int mtk_cam_seninf_disable_all_cammux(struct seninf_ctx *ctx)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;

	SENINF_WRITE_REG(seninf_cam_mux, SENINF_CAM_MUX_EN, 0);

	dev_dbg(ctx->dev, "%s all cam_mux EN 0x%x\n", __func__,
		readl(seninf_cam_mux + SENINF_CAM_MUX_EN));

	return 0;
}

int mtk_cam_seninf_set_top_mux_ctrl(struct seninf_ctx *ctx, int mux_idx,
				    int seninf_src)
{
	void __iomem *seninf = ctx->reg_if_top;

	switch (mux_idx) {
	case SENINF_MUX1:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
			    RG_SENINF_MUX1_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX2:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
			    RG_SENINF_MUX2_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX3:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
			    RG_SENINF_MUX3_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX4:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
			    RG_SENINF_MUX4_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX5:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
			    RG_SENINF_MUX5_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX6:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
			    RG_SENINF_MUX6_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX7:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
			    RG_SENINF_MUX7_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX8:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
			    RG_SENINF_MUX8_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX9:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
			    RG_SENINF_MUX9_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX10:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
			    RG_SENINF_MUX10_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX11:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
			    RG_SENINF_MUX11_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX12:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
			    RG_SENINF_MUX12_SRC_SEL, seninf_src);
		break;
	case SENINF_MUX13:
		SENINF_BITS(seninf, SENINF_TOP_MUX_CTRL_3,
			    RG_SENINF_MUX13_SRC_SEL, seninf_src);
		break;
	default:
		dev_dbg(ctx->dev, "invalid mux_idx %d\n", mux_idx);
		return -EINVAL;
	}

	return 0;
}

int mtk_cam_seninf_get_top_mux_ctrl(struct seninf_ctx *ctx, int mux_idx)
{
	void __iomem *seninf = ctx->reg_if_top;
	u32 seninf_src = 0;

	switch (mux_idx) {
	case SENINF_MUX1:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
					      RG_SENINF_MUX1_SRC_SEL);
		break;
	case SENINF_MUX2:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
					      RG_SENINF_MUX2_SRC_SEL);
		break;
	case SENINF_MUX3:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
					      RG_SENINF_MUX3_SRC_SEL);
		break;
	case SENINF_MUX4:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_0,
					      RG_SENINF_MUX4_SRC_SEL);
		break;
	case SENINF_MUX5:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
					      RG_SENINF_MUX5_SRC_SEL);
		break;
	case SENINF_MUX6:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
					      RG_SENINF_MUX6_SRC_SEL);
		break;
	case SENINF_MUX7:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
					      RG_SENINF_MUX7_SRC_SEL);
		break;
	case SENINF_MUX8:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_1,
					      RG_SENINF_MUX8_SRC_SEL);
		break;
	case SENINF_MUX9:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
					      RG_SENINF_MUX9_SRC_SEL);
		break;
	case SENINF_MUX10:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
					      RG_SENINF_MUX10_SRC_SEL);
		break;
	case SENINF_MUX11:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
					      RG_SENINF_MUX11_SRC_SEL);
		break;
	case SENINF_MUX12:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_2,
					      RG_SENINF_MUX12_SRC_SEL);
		break;
	case SENINF_MUX13:
		seninf_src = SENINF_READ_BITS(seninf, SENINF_TOP_MUX_CTRL_3,
					      RG_SENINF_MUX13_SRC_SEL);
		break;
	default:
		dev_dbg(ctx->dev, "invalid mux_idx %d", mux_idx);
		return -EINVAL;
	}

	return seninf_src;
}

int mtk_cam_seninf_get_cammux_ctrl(struct seninf_ctx *ctx, int cam_mux)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;
	u32 seninf_mux_src = 0;

	switch (cam_mux) {
	case SENINF_CAM_MUX0:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_0,
						  RG_SENINF_CAM_MUX0_SRC_SEL);
		break;
	case SENINF_CAM_MUX1:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_0,
						  RG_SENINF_CAM_MUX1_SRC_SEL);
		break;
	case SENINF_CAM_MUX2:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_0,
						  RG_SENINF_CAM_MUX2_SRC_SEL);
		break;
	case SENINF_CAM_MUX3:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_0,
						  RG_SENINF_CAM_MUX3_SRC_SEL);
		break;
	case SENINF_CAM_MUX4:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_1,
						  RG_SENINF_CAM_MUX4_SRC_SEL);
		break;
	case SENINF_CAM_MUX5:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_1,
						  RG_SENINF_CAM_MUX5_SRC_SEL);
		break;
	case SENINF_CAM_MUX6:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_1,
						  RG_SENINF_CAM_MUX6_SRC_SEL);
		break;
	case SENINF_CAM_MUX7:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_1,
						  RG_SENINF_CAM_MUX7_SRC_SEL);
		break;
	case SENINF_CAM_MUX8:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_2,
						  RG_SENINF_CAM_MUX8_SRC_SEL);
		break;
	case SENINF_CAM_MUX9:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_2,
						  RG_SENINF_CAM_MUX9_SRC_SEL);
		break;
	case SENINF_CAM_MUX10:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_2,
						  RG_SENINF_CAM_MUX10_SRC_SEL);
		break;
	case SENINF_CAM_MUX11:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_2,
						  RG_SENINF_CAM_MUX11_SRC_SEL);
		break;
	case SENINF_CAM_MUX12:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_3,
						  RG_SENINF_CAM_MUX12_SRC_SEL);
		break;
	case SENINF_CAM_MUX13:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_3,
						  RG_SENINF_CAM_MUX13_SRC_SEL);
		break;
	case SENINF_CAM_MUX14:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_3,
						  RG_SENINF_CAM_MUX14_SRC_SEL);
		break;
	case SENINF_CAM_MUX15:
		seninf_mux_src = SENINF_READ_BITS(seninf_cam_mux,
						  SENINF_CAM_MUX_CTRL_3,
						  RG_SENINF_CAM_MUX15_SRC_SEL);
		break;
	default:
		dev_dbg(ctx->dev, "invalid cam_mux %d", cam_mux);
		return -EINVAL;
	}

	return seninf_mux_src;
}

u32 mtk_cam_seninf_get_cammux_res(struct seninf_ctx *ctx, int cam_mux)
{
	return readl(ctx->reg_if_cam_mux +
			       SENINF_CAM_MUX0_CHK_RES + (0x10 * cam_mux));
}

int mtk_cam_seninf_set_cammux_vc(struct seninf_ctx *ctx, int cam_mux,
				 int vc_sel, int dt_sel, int vc_en,
				 int dt_en)
{
	void __iomem *seninf_cam_mux_vc_addr = ctx->reg_if_cam_mux + (cam_mux << 0x2);

	SENINF_BITS(seninf_cam_mux_vc_addr, SENINF_CAM_MUX0_OPT,
		    RG_SENINF_CAM_MUX0_VC_SEL, vc_sel);
	SENINF_BITS(seninf_cam_mux_vc_addr, SENINF_CAM_MUX0_OPT,
		    RG_SENINF_CAM_MUX0_DT_SEL, dt_sel);
	SENINF_BITS(seninf_cam_mux_vc_addr, SENINF_CAM_MUX0_OPT,
		    RG_SENINF_CAM_MUX0_VC_EN, vc_en);
	SENINF_BITS(seninf_cam_mux_vc_addr, SENINF_CAM_MUX0_OPT,
		    RG_SENINF_CAM_MUX0_DT_EN, dt_en);

	return 0;
}

int mtk_cam_seninf_switch_to_cammux_inner_page(struct seninf_ctx *ctx,
					       bool inner)
{
	void __iomem *seninf_cam_mux_addr = ctx->reg_if_cam_mux;

	SENINF_BITS(seninf_cam_mux_addr, SENINF_CAM_MUX_DYN_CTRL,
		    CAM_MUX_DYN_PAGE_SEL, inner ? 0 : 1);

	return 0;
}

int mtk_cam_seninf_set_cammux_next_ctrl(struct seninf_ctx *ctx, int src,
					int target)
{
	void __iomem *seninf_cam_mux_base = ctx->reg_if_cam_mux;

	if (target >= _seninf_cfg.cam_mux_num) {
		dev_dbg(ctx->dev,
			"%s err cam_mux %d >= SENINF_CAM_MUX_NUM %d\n",
			__func__, target, _seninf_cfg.cam_mux_num);

		return 0;
	}

	switch (target) {
	case SENINF_CAM_MUX0:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_0,
			    CAM_MUX0_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX1:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_0,
			    CAM_MUX1_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX2:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_0,
			    CAM_MUX2_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX3:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_0,
			    CAM_MUX3_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX4:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_1,
			    CAM_MUX4_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX5:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_1,
			    CAM_MUX5_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX6:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_1,
			    CAM_MUX6_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX7:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_1,
			    CAM_MUX7_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX8:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_2,
			    CAM_MUX8_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX9:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_2,
			    CAM_MUX9_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX10:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_2,
			    CAM_MUX10_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX11:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_2,
			    CAM_MUX11_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX12:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_3,
			    CAM_MUX12_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX13:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_3,
			    CAM_MUX13_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX14:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_3,
			    CAM_MUX14_NEXT_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX15:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_NEXT_CTRL_3,
			    CAM_MUX15_NEXT_SRC_SEL, src);
		break;
	default:
		dev_dbg(ctx->dev, "invalid src %d target %d", src, target);
		return -EINVAL;
	}

	return 0;
}

int mtk_cam_seninf_set_cammux_src(struct seninf_ctx *ctx, int src,
                                  int target)
{
    void __iomem *seninf_cam_mux_base = ctx->reg_if_cam_mux;

    if (target < 0 || target >= _seninf_cfg.cam_mux_num) {
        dev_dbg(ctx->dev,
            "%s err cam_mux %d >= SENINF_CAM_MUX_NUM %d\n",
            __func__, target, _seninf_cfg.cam_mux_num);
        return 0;
    }

	switch (target) {
	case SENINF_CAM_MUX0:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(0), RG_SENINF_CAM_MUX0_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX1:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(0), RG_SENINF_CAM_MUX1_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX2:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(0), RG_SENINF_CAM_MUX2_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX3:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(0), RG_SENINF_CAM_MUX3_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX4:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(1), RG_SENINF_CAM_MUX4_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX5:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(1), RG_SENINF_CAM_MUX5_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX6:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(1), RG_SENINF_CAM_MUX6_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX7:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(1), RG_SENINF_CAM_MUX7_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX8:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(2), RG_SENINF_CAM_MUX8_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX9:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(2), RG_SENINF_CAM_MUX9_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX10:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(2), RG_SENINF_CAM_MUX10_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX11:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(2), RG_SENINF_CAM_MUX11_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX12:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(3), RG_SENINF_CAM_MUX12_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX13:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(3), RG_SENINF_CAM_MUX13_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX14:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(3), RG_SENINF_CAM_MUX14_SRC_SEL, src);
		break;
	case SENINF_CAM_MUX15:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CTRL(3), RG_SENINF_CAM_MUX15_SRC_SEL, src);
		break;
	default:
		dev_dbg(ctx->dev, "%s: invalid target %d\n", __func__, target);
		return -EINVAL;
	}

    SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CHK_CTL_1(target),
                RG_SENINF_CAM_MUX_EXP_HSIZE, 0);
    SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX_CHK_CTL_1(target),
                RG_SENINF_CAM_MUX_EXP_VSIZE, 0);

    return 0;
}

int mtk_cam_seninf_set_vc(struct seninf_ctx *ctx, u32 seninf_idx,
			  struct seninf_vcinfo *vcinfo)
{
	void __iomem *seninf_csi2 = ctx->reg_if_csi2[seninf_idx];
	int i;
	struct seninf_vc *vc;

	if (WARN_ON(!vcinfo || !vcinfo->cnt))
		return -EINVAL;

	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S0_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S1_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S2_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S3_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S4_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S5_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S6_DI_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_S7_DI_CTRL, 0);

	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_CH0_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_CH1_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_CH2_CTRL, 0);
	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_CH3_CTRL, 0);

	for (i = 0; i < vcinfo->cnt; i++) {
		vc = &vcinfo->vc[i];

		/* General Long Packet Data Types: 0x10-0x17 */
		if (vc->dt >= 0x10 && vc->dt <= 0x17) {
			SENINF_BITS(seninf_csi2, SENINF_CSI2_OPT,
				    RG_CSI2_GENERIC_LONG_PACKET_EN, 1);
		}

		mtk_cam_seninf_set_di_ch_ctrl(seninf_csi2, i, vc);
	}

	dev_dbg(ctx->dev, "DI_CTRL 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		readl(seninf_csi2 + SENINF_CSI2_S0_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S1_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S2_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S3_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S4_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S5_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S6_DI_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_S7_DI_CTRL));

	dev_dbg(ctx->dev, "CH_CTRL 0x%x 0x%x 0x%x 0x%x\n",
		readl(seninf_csi2 + SENINF_CSI2_CH0_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_CH1_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_CH2_CTRL),
		readl(seninf_csi2 + SENINF_CSI2_CH3_CTRL));

	return 0;
}

int mtk_cam_seninf_set_mux_ctrl(struct seninf_ctx *ctx, u32 mux, int hs_pol,
				int vs_pol, int src_sel, int pixel_mode)
{
	u32 temp = 0;
	void __iomem *seninf_mux;

	seninf_mux = ctx->reg_if_mux[mux];

	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_1,
		    RG_SENINF_MUX_SRC_SEL, src_sel);

	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_1,
		    RG_SENINF_MUX_PIX_MODE_SEL, pixel_mode);

	SENINF_BITS(seninf_mux, SENINF_MUX_OPT,
		    RG_SENINF_MUX_HSYNC_POL, hs_pol);

	SENINF_BITS(seninf_mux, SENINF_MUX_OPT,
		    RG_SENINF_MUX_VSYNC_POL, vs_pol);

	temp = readl(seninf_mux + SENINF_MUX_CTRL_0);
	SENINF_WRITE_REG(seninf_mux, SENINF_MUX_CTRL_0, temp |
			 SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST);
	SENINF_WRITE_REG(seninf_mux, SENINF_MUX_CTRL_0, temp & 0xFFFFFFF9);

	return 0;
}

int mtk_cam_seninf_update_mux_pixel_mode(struct seninf_ctx *ctx, u32 mux,
					 int pixel_mode)
{
	u32 temp = 0;
	void __iomem *seninf_mux;

	seninf_mux = ctx->reg_if_mux[mux];

	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_1,
		    RG_SENINF_MUX_PIX_MODE_SEL, pixel_mode);

	temp = readl(seninf_mux + SENINF_MUX_CTRL_0);
	SENINF_WRITE_REG(seninf_mux, SENINF_MUX_CTRL_0, temp |
			 SENINF_MUX_IRQ_SW_RST | SENINF_MUX_SW_RST);
	SENINF_WRITE_REG(seninf_mux, SENINF_MUX_CTRL_0, temp & 0xFFFFFFF9);

	dev_dbg(ctx->dev,
		"%s mux %d SENINF_MUX_CTRL_1(0x%x), SENINF_MUX_OPT(0x%x)",
		__func__, mux, readl(seninf_mux + SENINF_MUX_CTRL_1),
		readl(seninf_mux + SENINF_MUX_OPT));

	return 0;
}

int mtk_cam_seninf_set_mux_crop(struct seninf_ctx *ctx, u32 mux, int start_x,
				int end_x, int enable)
{
	void __iomem *seninf_mux = ctx->reg_if_mux[mux];

	SENINF_BITS(seninf_mux, SENINF_MUX_CROP_PIX_CTRL,
		    RG_SENINF_MUX_CROP_START_8PIX_CNT, start_x / 8);
	SENINF_BITS(seninf_mux, SENINF_MUX_CROP_PIX_CTRL,
		    RG_SENINF_MUX_CROP_END_8PIX_CNT,
		    start_x / 8 + (end_x - start_x + 1) / 8 - 1 +
			    (((end_x - start_x + 1) % 8) > 0 ? 1 : 0));
	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_1,
		    RG_SENINF_MUX_CROP_EN, enable);

	dev_dbg(ctx->dev, "MUX_CROP_PIX_CTRL 0x%x MUX_CTRL_1 0x%x\n",
		readl(seninf_mux + SENINF_MUX_CROP_PIX_CTRL),
		readl(seninf_mux + SENINF_MUX_CTRL_1));

	dev_dbg(ctx->dev, "mux %d, start %d, end %d, enable %d\n",
		mux, start_x, end_x, enable);

	return 0;
}

int mtk_cam_seninf_is_mux_used(struct seninf_ctx *ctx, u32 mux)
{
	void __iomem *seninf_mux = ctx->reg_if_mux[mux];

	return SENINF_READ_BITS(seninf_mux, SENINF_MUX_CTRL_0, SENINF_MUX_EN);
}

int mtk_cam_seninf_mux(struct seninf_ctx *ctx, u32 mux)
{
	void __iomem *seninf_mux = ctx->reg_if_mux[mux];

	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_0, SENINF_MUX_EN, 1);
	return 0;
}

int mtk_cam_seninf_disable_mux(struct seninf_ctx *ctx, u32 mux)
{
	int i;
	void __iomem *seninf_mux = ctx->reg_if_mux[mux];

	SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_0, SENINF_MUX_EN, 0);

	/* also disable CAM_MUX with input from mux */
	for (i = SENINF_CAM_MUX0; i < _seninf_cfg.cam_mux_num; i++) {
		if (mux == mtk_cam_seninf_get_cammux_ctrl(ctx, i))
			mtk_cam_seninf_disable_cammux(ctx, i);
	}

	return 0;
}

int mtk_cam_seninf_disable_all_mux(struct seninf_ctx *ctx)
{
	int i;
	void __iomem *seninf_mux;

	for (i = 0; i < _seninf_cfg.mux_num; i++) {
		seninf_mux = ctx->reg_if_mux[i];
		SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_0, SENINF_MUX_EN, 0);
	}

	return 0;
}

int mtk_cam_seninf_set_cammux_chk_pixel_mode(struct seninf_ctx *ctx,
					     int cam_mux, int pixel_mode)
{
	void __iomem *seninf_cam_mux_base = ctx->reg_if_cam_mux;

	switch (cam_mux) {
	case SENINF_CAM_MUX0:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX0_CHK_CTL_0,
			    RG_SENINF_CAM_MUX0_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX1:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX1_CHK_CTL_0,
			    RG_SENINF_CAM_MUX1_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX2:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX2_CHK_CTL_0,
			    RG_SENINF_CAM_MUX2_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX3:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX3_CHK_CTL_0,
			    RG_SENINF_CAM_MUX3_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX4:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX4_CHK_CTL_0,
			    RG_SENINF_CAM_MUX4_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX5:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX5_CHK_CTL_0,
			    RG_SENINF_CAM_MUX5_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX6:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX6_CHK_CTL_0,
			    RG_SENINF_CAM_MUX6_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX7:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX7_CHK_CTL_0,
			    RG_SENINF_CAM_MUX7_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX8:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX8_CHK_CTL_0,
			    RG_SENINF_CAM_MUX8_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX9:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX9_CHK_CTL_0,
			    RG_SENINF_CAM_MUX9_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX10:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX10_CHK_CTL_0,
			    RG_SENINF_CAM_MUX10_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX11:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX11_CHK_CTL_0,
			    RG_SENINF_CAM_MUX11_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX12:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX12_CHK_CTL_0,
			    RG_SENINF_CAM_MUX12_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX13:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX13_CHK_CTL_0,
			    RG_SENINF_CAM_MUX13_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX14:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX14_CHK_CTL_0,
			    RG_SENINF_CAM_MUX14_PIX_MODE_SEL, pixel_mode);
		break;
	case SENINF_CAM_MUX15:
		SENINF_BITS(seninf_cam_mux_base, SENINF_CAM_MUX15_CHK_CTL_0,
			    RG_SENINF_CAM_MUX15_PIX_MODE_SEL, pixel_mode);
		break;
	default:
		dev_dbg(ctx->dev, "invalid cam_mux %d pixel_mode %d\n",
			cam_mux, pixel_mode);
		return -EINVAL;
	}

	return 0;
}

static void csirx_phy_a_power_off(struct seninf_ctx *ctx, u32 port_idx)
{
	void __iomem *base = ctx->reg_ana_csi_rx[port_idx];

	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L0_T0AB_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L1_T1AB_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L2_T1BC_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T0BC_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T0CA_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T1CA_EQ_OS_CAL_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_BG_LPF_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_BG_CORE_EN, 0);

	usleep_range(200, 300);
}

static int csirx_phy_a_power_on(struct seninf_ctx *ctx, u32 port_idx)
{
	void __iomem *base = ctx->reg_ana_csi_rx[port_idx];

	/* Power off first to ensure a clean state */
	csirx_phy_a_power_off(ctx, port_idx);

	/* Power on sequence */
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_BG_CORE_EN, 1);
	usleep_range(10, 20);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_BG_LPF_EN, 1);
	usleep_range(10, 20);

	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L0_T0AB_EQ_OS_CAL_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L1_T1AB_EQ_OS_CAL_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_L2_T1BC_EQ_OS_CAL_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T0BC_EQ_OS_CAL_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T0CA_EQ_OS_CAL_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_8, RG_CSI0_XX_T1CA_EQ_OS_CAL_EN, 1);

	usleep_range(200, 300);

	return 0;
}

static int apply_efuse_data(struct seninf_ctx *ctx)
{
	void __iomem *base;
	u32 m_csi_efuse = ctx->m_csi_efuse;
	u32 port;
	int ret = 0;

	if (m_csi_efuse == 0) {
		dev_dbg(ctx->dev, "No efuse data. Returned.\n");
		return -1;
	}

	port = ctx->port;
	base = ctx->reg_ana_csi_rx[port];

	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0P_T0A_HSRT_CODE,
		    (m_csi_efuse >> 27) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0N_T0B_HSRT_CODE,
		    (m_csi_efuse >> 27) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1P_T0C_HSRT_CODE,
		    (m_csi_efuse >> 22) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1N_T1A_HSRT_CODE,
		    (m_csi_efuse >> 22) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2P_T1B_HSRT_CODE,
		    (m_csi_efuse >> 17) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2N_T1C_HSRT_CODE,
		    (m_csi_efuse >> 17) & 0x1f);

	dev_dbg(ctx->dev,
		"CSI%dA CDPHY_RX_ANA_2(0x%x) CDPHY_RX_ANA_3(0x%x) CDPHY_RX_ANA_4(0x%x)",
		ctx->port,
		readl(base + CDPHY_RX_ANA_2),
		readl(base + CDPHY_RX_ANA_3),
		readl(base + CDPHY_RX_ANA_4));

	if (ctx->is_4d1c == 0)
		return ret;

	port = ctx->port_b;
	base = ctx->reg_ana_csi_rx[port];

	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0P_T0A_HSRT_CODE,
		    (m_csi_efuse >> 12) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0N_T0B_HSRT_CODE,
		    (m_csi_efuse >> 12) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1P_T0C_HSRT_CODE,
		    (m_csi_efuse >> 7) & 0x1f);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1N_T1A_HSRT_CODE,
		    (m_csi_efuse >> 7) & 0x1f);

	if (port < CSI_PORT_2A) {
		SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2P_T1B_HSRT_CODE,
			    (m_csi_efuse >> 2) & 0x1f);
		SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2N_T1C_HSRT_CODE,
			    (m_csi_efuse >> 2) & 0x1f);
		
		dev_dbg(ctx->dev,
			"CSI%dB CDPHY_RX_ANA_2(0x%x) CDPHY_RX_ANA_3(0x%x) CDPHY_RX_ANA_4(0x%x)",
			ctx->port,
			readl(base + CDPHY_RX_ANA_2),
			readl(base + CDPHY_RX_ANA_3),
			readl(base + CDPHY_RX_ANA_4));
	} else {
		dev_dbg(ctx->dev,
			"CSI%dB CDPHY_RX_ANA_2(0x%x) CDPHY_RX_ANA_3(0x%x)",
			ctx->port,
			readl(base + CDPHY_RX_ANA_2),
			readl(base + CDPHY_RX_ANA_3));
	}

	return 0;
}

static void csirx_phy_a_setup_port(struct seninf_ctx *ctx, u32 port)
{
	void __iomem *base = ctx->reg_ana_csi_rx[port];

	SENINF_BITS(base, CDPHY_RX_ANA_1, RG_CSI0_BG_LPRX_VTL_SEL, 0x4);
	SENINF_BITS(base, CDPHY_RX_ANA_1, RG_CSI0_BG_LPRX_VTH_SEL, 0x4);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_BG_ALP_RX_VTL_SEL, 0x4);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_BG_ALP_RX_VTH_SEL, 0x4);
	SENINF_BITS(base, CDPHY_RX_ANA_1, RG_CSI0_BG_VREF_SEL, 0x8);
	SENINF_BITS(base, CDPHY_RX_ANA_1, RG_CSI0_CDPHY_EQ_DES_VREF_SEL, 0x2);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_9, RG_CSI0_RESERVE, 0x3003);
	SENINF_BITS(base, CDPHY_RX_ANA_SETTING_0, CSR_CSI_RST_MODE, 0x2);

	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0P_T0A_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_L0N_T0B_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1P_T0C_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_3, RG_CSI0_L1N_T1A_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2P_T1B_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_4, RG_CSI0_L2N_T1C_HSRT_CODE, 0x10);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_CPHY_T0_CDR_FIRST_EDGE_EN, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_CPHY_T1_CDR_FIRST_EDGE_EN, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_CPHY_T0_CDR_SELF_CAL_EN, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_2, RG_CSI0_CPHY_T1_CDR_SELF_CAL_EN, 0x0);

	SENINF_BITS(base, CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_CK_DELAY, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_CK_DELAY, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_AB_WIDTH, 0x9);
	SENINF_BITS(base, CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_BC_WIDTH, 0x9);
	SENINF_BITS(base, CDPHY_RX_ANA_6, RG_CSI0_CPHY_T0_CDR_CA_WIDTH, 0x9);
	SENINF_BITS(base, CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_AB_WIDTH, 0x9);
	SENINF_BITS(base, CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_BC_WIDTH, 0x9);
	SENINF_BITS(base, CDPHY_RX_ANA_7, RG_CSI0_CPHY_T1_CDR_CA_WIDTH, 0x9);

	dev_dbg(ctx->dev, "port:%d CDPHY_RX_ANA_0(0x%x)\n",
		port, SENINF_READ_REG(base, CDPHY_RX_ANA_0));
}

static int csirx_phyA_init(struct seninf_ctx *ctx)
{
	csirx_phy_a_setup_port(ctx, ctx->port);

	if (ctx->is_4d1c)
		csirx_phy_a_setup_port(ctx, ctx->port_b);

	apply_efuse_data(ctx);

	return 0;
}

static int csirx_dphy_init(struct seninf_ctx *ctx)
{
	void __iomem *base = ctx->reg_ana_dphy_top[ctx->port];
	u8 bits_per_pixel;
	int settle_delay_dt, settle_delay_ck, hs_trail, hs_trail_en;
	u64 data_rate;

	/* Settle delay parameters */
	settle_delay_dt = ctx->is_cphy ? ctx->core->cphy_settle_delay_dt :
					 ctx->core->dphy_settle_delay_dt;
	settle_delay_ck = ctx->core->settle_delay_ck;

	/* HS trail parameter */
	hs_trail = ctx->hs_trail_parameter;

	/* Settle delay for data lanes */
	SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
		    RG_CDPHY_RX_LD0_TRIO0_HS_SETTLE_PARAMETER,
		    settle_delay_dt);
	SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
		    RG_CDPHY_RX_LD1_TRIO1_HS_SETTLE_PARAMETER,
		    settle_delay_dt);
	SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
		    RG_CDPHY_RX_LD2_TRIO2_HS_SETTLE_PARAMETER,
		    settle_delay_dt);
	SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
		    RG_CDPHY_RX_LD3_TRIO3_HS_SETTLE_PARAMETER,
		    settle_delay_dt);

	/* Settle delay for clock lanes */
	SENINF_BITS(base, DPHY_RX_CLOCK_LANE0_HS_PARAMETER,
		    RG_DPHY_RX_LC0_HS_SETTLE_PARAMETER,
		    settle_delay_ck);
	SENINF_BITS(base, DPHY_RX_CLOCK_LANE1_HS_PARAMETER,
		    RG_DPHY_RX_LC1_HS_SETTLE_PARAMETER,
		    settle_delay_ck);

	/* HS prepare parameter for data lanes */
	SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
		    RG_CDPHY_RX_LD0_TRIO0_HS_PREPARE_PARAMETER, 2);
	SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
		    RG_CDPHY_RX_LD1_TRIO1_HS_PREPARE_PARAMETER, 2);
	SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
		    RG_CDPHY_RX_LD2_TRIO2_HS_PREPARE_PARAMETER, 2);
	SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
		    RG_CDPHY_RX_LD3_TRIO3_HS_PREPARE_PARAMETER, 2);

	/* HS trail parameter for data lanes */
	SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
		    RG_DPHY_RX_LD0_HS_TRAIL_PARAMETER, hs_trail);
	SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
		    RG_DPHY_RX_LD1_HS_TRAIL_PARAMETER, hs_trail);
	SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
		    RG_DPHY_RX_LD2_HS_TRAIL_PARAMETER, hs_trail);
	SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
		    RG_DPHY_RX_LD3_HS_TRAIL_PARAMETER, hs_trail);

	if (!ctx->is_cphy) {
		bits_per_pixel = 10;

		data_rate = ctx->customized_pixel_rate ?
			    ctx->customized_pixel_rate : ctx->mipi_pixel_rate;
		data_rate *= bits_per_pixel;
		do_div(data_rate, ctx->num_data_lanes);

		hs_trail_en = data_rate < 1400000000;

		SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
			    RG_DPHY_RX_LD0_HS_TRAIL_EN, hs_trail_en);
		SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
			    RG_DPHY_RX_LD1_HS_TRAIL_EN, hs_trail_en);
		SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
			    RG_DPHY_RX_LD2_HS_TRAIL_EN, hs_trail_en);
		SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
			    RG_DPHY_RX_LD3_HS_TRAIL_EN, hs_trail_en);
	}

	return 0;
}

static int csirx_cphy_init(struct seninf_ctx *ctx)
{
	void __iomem *base = ctx->reg_ana_cphy_top[ctx->port];

	SENINF_BITS(base, CPHY_RX_DETECT_CTRL_POST,
		    RG_CPHY_RX_DATA_VALID_POST_EN, 1);

	return 0;
}

static int csirx_phy_init(struct seninf_ctx *ctx)
{
	/* phyA init */
	csirx_phyA_init(ctx);

	/* phyD init */
	csirx_dphy_init(ctx);
	csirx_cphy_init(ctx);

	return 0;
}

static int csirx_seninf_csi2_setting(struct seninf_ctx *ctx)
{
	void __iomem *seninf_csi2 = ctx->reg_if_csi2[ctx->seninf_idx];
	int csi_en;

	SENINF_BITS(seninf_csi2, SENINF_CSI2_DBG_CTRL,
		    RG_CSI2_DBG_PACKET_CNT_EN, 1);

	/* lane/trio count */
	SENINF_BITS(seninf_csi2, SENINF_CSI2_RESYNC_MERGE_CTRL,
		    RG_CSI2_RESYNC_CYCLE_CNT_OPT, 1);

	csi_en = BIT(ctx->num_data_lanes) - 1;

	if (ctx->is_cphy) { /* cphy */
		u8 map_hdr_len[] = { 0, 1, 2, 4, 5 };

		SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_EN, csi_en);
		SENINF_BITS(seninf_csi2, SENINF_CSI2_OPT,
				RG_CSI2_CPHY_SEL, 1);
		SENINF_BITS(seninf_csi2, SENINF_CSI2_HDR_MODE_0,
				RG_CSI2_HEADER_MODE, 2);
		SENINF_BITS(seninf_csi2, SENINF_CSI2_HDR_MODE_0,
				RG_CSI2_HEADER_LEN,
				map_hdr_len[ctx->num_data_lanes]);
	} else { /* dphy */
		SENINF_BITS(seninf_csi2, SENINF_CSI2_OPT, RG_CSI2_CPHY_SEL, 0);
		SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_EN, csi_en);
		SENINF_BITS(seninf_csi2, SENINF_CSI2_HDR_MODE_0,
				RG_CSI2_HEADER_MODE, 0);
		SENINF_BITS(seninf_csi2, SENINF_CSI2_HDR_MODE_0,
				RG_CSI2_HEADER_LEN, 0);
	}

	return 0;
}

static int csirx_seninf_setting(struct seninf_ctx *ctx)
{
	void __iomem *seninf = ctx->reg_if_ctrl[ctx->seninf_idx];

	/* enable/disable seninf csi */
	SENINF_BITS(seninf, SENINF_CSI_CTRL, RG_SENINF_CSI2_EN, 1);

	/* enable/disable seninf, enable after csi2, testmdl is done */
	SENINF_BITS(seninf, SENINF_CTRL, SENINF_EN, 1);

	return 0;
}

static int csirx_seninf_top_setting(struct seninf_ctx *ctx)
{
	void __iomem *seninf_top = ctx->reg_if_top;

	switch (ctx->port) {
	case CSI_PORT_0:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
			    RG_PHY_SENINF_MUX0_CPHY_MODE, 0); //4T
		break;
	case CSI_PORT_0A:
	case CSI_PORT_0B:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
			    RG_PHY_SENINF_MUX0_CPHY_MODE, 2); //2+2T
		break;
	case CSI_PORT_1:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
			    RG_PHY_SENINF_MUX1_CPHY_MODE, 0); //4T
		break;
	case CSI_PORT_1A:
	case CSI_PORT_1B:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
			    RG_PHY_SENINF_MUX1_CPHY_MODE, 2); //2+2T
		break;
	case CSI_PORT_2:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
			    RG_PHY_SENINF_MUX2_CPHY_MODE, 0); //4T
		break;
	case CSI_PORT_2A:
	case CSI_PORT_2B:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
			    RG_PHY_SENINF_MUX2_CPHY_MODE, 2); //2+2T
		break;
	case CSI_PORT_3:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
			    RG_PHY_SENINF_MUX3_CPHY_MODE, 0); //4T
		break;
	case CSI_PORT_3A:
	case CSI_PORT_3B:
		SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
			    RG_PHY_SENINF_MUX3_CPHY_MODE, 2); //2+2T
		break;
	default:
		break;
	}

	/* port operation mode */
	switch (ctx->port) {
	case CSI_PORT_0:
	case CSI_PORT_0A:
	case CSI_PORT_0B:
		if (ctx->is_cphy) {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
					PHY_SENINF_MUX0_DPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
					PHY_SENINF_MUX0_CPHY_EN, 1);
		} else {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
					PHY_SENINF_MUX0_CPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI0,
					PHY_SENINF_MUX0_DPHY_EN, 1);
		}
		break;
	case CSI_PORT_1:
	case CSI_PORT_1A:
	case CSI_PORT_1B:
		if (ctx->is_cphy) {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
					PHY_SENINF_MUX1_DPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
					PHY_SENINF_MUX1_CPHY_EN, 1);
		} else {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
					PHY_SENINF_MUX1_CPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI1,
					PHY_SENINF_MUX1_DPHY_EN, 1);
		}
		break;
	case CSI_PORT_2:
	case CSI_PORT_2A:
	case CSI_PORT_2B:
		if (ctx->is_cphy) {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
					PHY_SENINF_MUX2_DPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
					PHY_SENINF_MUX2_CPHY_EN, 1);
		} else {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
					PHY_SENINF_MUX2_CPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI2,
					PHY_SENINF_MUX2_DPHY_EN, 1);
		}
		break;
	case CSI_PORT_3:
	case CSI_PORT_3A:
	case CSI_PORT_3B:
		if (ctx->is_cphy) {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
					PHY_SENINF_MUX3_DPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
					PHY_SENINF_MUX3_CPHY_EN, 1);
		} else {
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
					PHY_SENINF_MUX3_CPHY_EN, 0);
			SENINF_BITS(seninf_top, SENINF_TOP_PHY_CTRL_CSI3,
					PHY_SENINF_MUX3_DPHY_EN, 1);
		}
		break;
	default:
		dev_err(ctx->dev, "Invalid CSI port: %d\n", ctx->port);
    	return -EINVAL;
	}

	return 0;
}

static void csirx_phyA_dphy_4d1c_setting(struct seninf_ctx *ctx, void __iomem *baseA, void __iomem *baseB)
{
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 0);
	/* clear clk sel first */
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);

	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKSEL, 1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKSEL, 1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKSEL, 1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKSEL, 1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKSEL, 1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKSEL, 1);

	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);

	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);

	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);
}

static void csirx_phyA_dphy_setting(struct seninf_ctx *ctx, void __iomem *base)
{
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 0);
	/* clear clk sel first */
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);

	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKSEL, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKSEL, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKSEL, 0);

	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L0_CKMODE_EN, 0);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L1_CKMODE_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_DPHY_L2_CKMODE_EN, 0);

	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);
}

static void csirx_phyA_cphy_4d1c_setting(struct seninf_ctx *ctx, void __iomem *baseA, void __iomem *baseB)
{
	SENINF_BITS(baseA, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 1);

	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x0);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x3);
	SENINF_BITS(baseA, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);

	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x0);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x3);
	SENINF_BITS(baseB, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);
}

static void csirx_phyA_cphy_setting(struct seninf_ctx *ctx, void __iomem *base)
{
	SENINF_BITS(base, CDPHY_RX_ANA_0, RG_CSI0_CPHY_EN, 1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_BW, 0x3);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_IS, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_LATCH_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG0_EN, 0x1);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_DG1_EN, 0x0);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR0, 0x3);
	SENINF_BITS(base, CDPHY_RX_ANA_5, RG_CSI0_CDPHY_EQ_SR1, 0x0);
}

static void csirx_phyA_power_on_wrapper(struct seninf_ctx *ctx)
{
	if (ctx->is_4d1c) {
		csirx_phy_a_power_on(ctx, ctx->port_a);
		csirx_phy_a_power_on(ctx, ctx->port_b);
	} else {
		csirx_phy_a_power_on(ctx, ctx->port);
	}
}

static int csirx_phyA_setting(struct seninf_ctx *ctx)
{
	void __iomem *base, *baseA, *baseB;

	base = ctx->reg_ana_csi_rx[ctx->port];
	baseA = ctx->reg_ana_csi_rx[ctx->port_a];
	baseB = ctx->reg_ana_csi_rx[ctx->port_b];

	if (!ctx->is_cphy) { /* dphy */
		if (ctx->is_4d1c)
			csirx_phyA_dphy_4d1c_setting(ctx, baseA, baseB);
		else
			csirx_phyA_dphy_setting(ctx, base);
	} else { /* cphy */
		if (ctx->is_4d1c)
			csirx_phyA_cphy_4d1c_setting(ctx, baseA, baseB);
		else
			csirx_phyA_cphy_setting(ctx, base);
	}

	csirx_phyA_power_on_wrapper(ctx);

	return 0;
}

static int csirx_dphy_setting(struct seninf_ctx *ctx)
{
	void __iomem *base = ctx->reg_ana_dphy_top[ctx->port];

	if (ctx->is_4d1c) {
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD3_SEL, 4);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD2_SEL, 0);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD1_SEL, 3);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD0_SEL, 1);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LC0_SEL, 2);

		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD0_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD1_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD2_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD3_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LC0_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LC1_EN, 0);
	} else {
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD3_SEL, 5);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD2_SEL, 3);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD1_SEL, 2);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LD0_SEL, 0);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LC1_SEL, 4);
		SENINF_BITS(base, DPHY_RX_LANE_SELECT, RG_DPHY_RX_LC0_SEL, 1);

		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD0_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD1_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD2_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LD3_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LC0_EN, 1);
		SENINF_BITS(base, DPHY_RX_LANE_EN, DPHY_RX_LC1_EN, 1);
	}

	SENINF_BITS(base, DPHY_RX_LANE_SELECT, DPHY_RX_CK_DATA_MUX_EN, 1);

	return 0;
}

static int csirx_cphy_setting(struct seninf_ctx *ctx)
{
	void __iomem *base = ctx->reg_ana_cphy_top[ctx->port];

	switch (ctx->port) {
	case CSI_PORT_0:
	case CSI_PORT_1:
	case CSI_PORT_2:
	case CSI_PORT_3:
	case CSI_PORT_0A:
	case CSI_PORT_1A:
	case CSI_PORT_2A:
	case CSI_PORT_3A:
		if (ctx->num_data_lanes == 3) {
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR0_LPRX_EN, 1);
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR1_LPRX_EN, 1);
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR2_LPRX_EN, 1);
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR3_LPRX_EN, 0);
		} else if (ctx->num_data_lanes == 2) {
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR0_LPRX_EN, 1);
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR1_LPRX_EN, 1);
		} else {
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR0_LPRX_EN, 1);
		}
		break;
	case CSI_PORT_0B:
	case CSI_PORT_1B:
	case CSI_PORT_2B:
	case CSI_PORT_3B:
		if (ctx->num_data_lanes == 2) {
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR2_LPRX_EN, 1);
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR3_LPRX_EN, 1);
		} else {
			SENINF_BITS(base, CPHY_RX_CTRL, CPHY_RX_TR2_LPRX_EN, 1);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int csirx_phy_setting(struct seninf_ctx *ctx)
{
	/* phyA */
	csirx_phyA_setting(ctx);

	if (ctx->is_cphy)
		csirx_cphy_setting(ctx);
	else
		csirx_dphy_setting(ctx);

	return 0;
}

int mtk_cam_seninf_set_csi_mipi(struct seninf_ctx *ctx)
{
	csirx_phy_init(ctx);

	/* seninf csi2 */
	csirx_seninf_csi2_setting(ctx);

	/* seninf */
	csirx_seninf_setting(ctx);

	/* seninf top */
	csirx_seninf_top_setting(ctx);

	/* phy */
	csirx_phy_setting(ctx);

	return 0;
}

int mtk_cam_seninf_poweroff(struct seninf_ctx *ctx)
{
	void __iomem *seninf_csi2;

	seninf_csi2 = ctx->reg_if_csi2[ctx->seninf_idx];

	SENINF_WRITE_REG(seninf_csi2, SENINF_CSI2_EN, 0x0);

	if (ctx->is_4d1c) {
		csirx_phy_a_power_off(ctx, ctx->port_a);
		csirx_phy_a_power_off(ctx, ctx->port_b);
	} else {
		csirx_phy_a_power_off(ctx, ctx->port);
	}

	return 0;
}

int mtk_cam_seninf_reset(struct seninf_ctx *ctx, u32 seninf_idx)
{
	int i;
	void __iomem *seninf_mux;
	void __iomem *seninf = ctx->reg_if_ctrl[seninf_idx];

	SENINF_BITS(seninf, SENINF_CSI_CTRL, SENINF_CSI2_SW_RST, 1);
	udelay(1);
	SENINF_BITS(seninf, SENINF_CSI_CTRL, SENINF_CSI2_SW_RST, 0);

	dev_dbg(ctx->dev, "reset seninf %d\n", seninf_idx);

	for (i = SENINF_MUX1; i < _seninf_cfg.mux_num; i++)
		if (mtk_cam_seninf_get_top_mux_ctrl(ctx, i) == seninf_idx &&
		    mtk_cam_seninf_is_mux_used(ctx, i)) {
			seninf_mux = ctx->reg_if_mux[i];
			SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_0,
				    SENINF_MUX_SW_RST, 1);
			udelay(1);
			SENINF_BITS(seninf_mux, SENINF_MUX_CTRL_0,
				    SENINF_MUX_SW_RST, 0);
			dev_dbg(ctx->dev, "reset mux %d\n", i);
		}

	return 0;
}

int mtk_cam_seninf_set_idle(struct seninf_ctx *ctx)
{
	int i;
	struct seninf_vcinfo *vcinfo = &ctx->vcinfo;
	struct seninf_vc *vc;

	for (i = 0; i < vcinfo->cnt; i++) {
		vc = &vcinfo->vc[i];
		if (vc->enable) {
			mtk_cam_seninf_disable_mux(ctx, vc->mux);
			mtk_cam_seninf_disable_cammux(ctx, vc->cam);
			ctx->pad2cam[vc->out_pad] = 0xff;
		}
	}

	return 0;
}

int mtk_cam_seninf_irq_handler(int irq, void *data)
{
	struct seninf_core *core = (struct seninf_core *)data;
	unsigned long flags; /* for mipi err detection */
	struct seninf_ctx *ctx;
	struct seninf_vc *vc;
	void __iomem *csi2, *pmux, *seninf_cam_mux;
	int i;
	unsigned int csi_irq_ro;
	unsigned int mux_irq_ro;
	unsigned int cam_irq_exp_ro;
	unsigned int cam_irq_res_ro;

	spin_lock_irqsave(&core->spinlock_irq, flags);

	/* debug for set_reg case: REG_KEY_CSI_IRQ_EN */
	if (core->csi_irq_en_flag) {
		list_for_each_entry(ctx, &core->list, list) {
			csi2 = ctx->reg_if_csi2[ctx->seninf_idx];
			csi_irq_ro =
				readl(csi2 + SENINF_CSI2_IRQ_STATUS);

			if (csi_irq_ro) {
				SENINF_WRITE_REG(csi2, SENINF_CSI2_IRQ_STATUS,
						 0xFFFFFFFF);
			}

			if (csi_irq_ro & RO_CSI2_ECC_ERR_CORRECTED_IRQ)
				ctx->ecc_err_corrected_cnt++;
			if (csi_irq_ro & RO_CSI2_ECC_ERR_DOUBLE_IRQ)
				ctx->ecc_err_double_cnt++;
			if (csi_irq_ro & RO_CSI2_CRC_ERR_IRQ)
				ctx->crc_err_cnt++;
			if (csi_irq_ro & RO_CSI2_ERR_LANE_RESYNC_IRQ)
				ctx->err_lane_resync_cnt++;
			if (csi_irq_ro & RO_CSI2_RECEIVE_DATA_NOT_ENOUGH_IRQ)
				ctx->data_not_enough_cnt++;

			for (i = 0; i < ctx->vcinfo.cnt; i++) {
				vc = &ctx->vcinfo.vc[i];
				pmux = ctx->reg_if_mux[vc->mux];
				seninf_cam_mux = ctx->reg_if_cam_mux;

				mux_irq_ro = readl(pmux +
							     SENINF_MUX_IRQ_STATUS);

				cam_irq_exp_ro = readl(seninf_cam_mux +
								 SENINF_CAM_MUX0_CHK_CTL_1 +
								 (0x10 * (vc->cam)));

				cam_irq_res_ro = readl(seninf_cam_mux +
								 SENINF_CAM_MUX0_CHK_RES +
								 (0x10 * (vc->cam)));

				if (mux_irq_ro)
					SENINF_WRITE_REG(pmux,
							 SENINF_MUX_IRQ_STATUS,
							 0xFFFFFFFF);

				if (cam_irq_res_ro != cam_irq_exp_ro)
					SENINF_WRITE_REG(seninf_cam_mux,
							 SENINF_CAM_MUX0_CHK_RES +
							 (0x10 * (vc->cam)),
							 0xFFFFFFFF);

				if (mux_irq_ro & (0x1 << 0))
					ctx->fifo_overrun_cnt++;

				if (cam_irq_res_ro != cam_irq_exp_ro)
					ctx->size_err_cnt++;
			}

			/* dump status counter: debug for electrical signal */
			if (ctx->data_not_enough_cnt >= core->detection_cnt ||
			    ctx->err_lane_resync_cnt >= core->detection_cnt ||
			    ctx->crc_err_cnt >= core->detection_cnt ||
			    ctx->ecc_err_double_cnt >= core->detection_cnt ||
			    ctx->ecc_err_corrected_cnt >= core->detection_cnt ||
			    ctx->fifo_overrun_cnt >= core->detection_cnt ||
			    ctx->size_err_cnt >= core->detection_cnt) {
				/* disable all interrupts */
				SENINF_WRITE_REG(csi2, SENINF_CSI2_IRQ_EN, 0x80000000);

				if (ctx->data_not_enough_cnt >= core->detection_cnt)
					ctx->data_not_enough_flag = 1;
				if (ctx->err_lane_resync_cnt >= core->detection_cnt)
					ctx->err_lane_resync_flag = 1;
				if (ctx->crc_err_cnt >= core->detection_cnt)
					ctx->crc_err_flag = 1;
				if (ctx->ecc_err_double_cnt >= core->detection_cnt)
					ctx->ecc_err_double_flag = 1;
				if (ctx->ecc_err_corrected_cnt >= core->detection_cnt)
					ctx->ecc_err_corrected_flag = 1;
				if (ctx->fifo_overrun_cnt >= core->detection_cnt)
					ctx->fifo_overrun_flag = 1;
				if (ctx->size_err_cnt >= core->detection_cnt)
					ctx->size_err_flag = 1;
			}
		}
	}

	spin_unlock_irqrestore(&core->spinlock_irq, flags);

	return 0;
}

int mtk_cam_seninf_set_sw_cfg_busy(struct seninf_ctx *ctx, bool enable, int index)
{
    void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;
    unsigned int mask;

    mask = index ? RG_SENINF_CAM_MUX_DYN_SWITCH_BSY1 : RG_SENINF_CAM_MUX_DYN_SWITCH_BSY0;

    SENINF_BITS_VAR(seninf_cam_mux, SENINF_CAM_MUX_DYN_CTRL, mask, enable);

    return 0;
}

int mtk_cam_seninf_reset_cam_mux_dyn_en(struct seninf_ctx *ctx, int index)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;

	if (index == 0)
		SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_DYN_EN,
			    RG_SENINF_CAM_MUX_DYN_SWITCH_EN0, 0);
	else
		SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_DYN_EN,
			    RG_SENINF_CAM_MUX_DYN_SWITCH_EN1, 0);
	return 0;
}

int mtk_cam_seninf_enable_global_drop_irq(struct seninf_ctx *ctx, bool enable,
					  int index)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;

	if (index == 0)
		SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_IRQ_EN,
			    RG_SENINF_CAM_MUX15_HSIZE_ERR_IRQ_EN, enable);
	else
		SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_IRQ_EN,
			    RG_SENINF_CAM_MUX15_VSIZE_ERR_IRQ_EN, enable);
	return 0;
}

int mtk_cam_seninf_enable_cam_mux_vsync_irq(struct seninf_ctx *ctx, bool enable,
					    int cam_mux)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;
	int tmp = 0;

	tmp = SENINF_READ_BITS(seninf_cam_mux, SENINF_CAM_MUX_VSYNC_IRQ_EN,
			       RG_SENINF_CAM_MUX_VSYNC_IRQ_EN);
	if (enable)
		tmp |= (enable << cam_mux);
	else
		tmp &= ~(enable << cam_mux);
	SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_VSYNC_IRQ_EN,
		    RG_SENINF_CAM_MUX_VSYNC_IRQ_EN, tmp);
	return 0;
}

int mtk_cam_seninf_disable_all_cam_mux_vsync_irq(struct seninf_ctx *ctx)
{
	void __iomem *seninf_cam_mux = ctx->reg_if_cam_mux;

	SENINF_BITS(seninf_cam_mux, SENINF_CAM_MUX_VSYNC_IRQ_EN,
		    RG_SENINF_CAM_MUX_VSYNC_IRQ_EN, 0);
	return 0;
}

int mtk_cam_seninf_set_reg(struct seninf_ctx *ctx, u32 key, u32 val)
{
	int i;
	void __iomem *base = ctx->reg_ana_dphy_top[ctx->port];
	void __iomem *csi2 = ctx->reg_if_csi2[ctx->seninf_idx];
	void __iomem *pmux, *pcammux;
	struct seninf_vc *vc;
	struct seninf_core *core;
	struct seninf_ctx *ctx_;
	void __iomem *csi2_;

	core = dev_get_drvdata(ctx->dev->parent);

	if (!ctx->streaming)
		return 0;

	dev_dbg(ctx->dev, "%s key = 0x%x, val = 0x%x\n", __func__, key, val);

	switch (key) {
	case REG_KEY_SETTLE_CK:
		SENINF_BITS(base, DPHY_RX_CLOCK_LANE0_HS_PARAMETER,
			    RG_DPHY_RX_LC0_HS_SETTLE_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_CLOCK_LANE1_HS_PARAMETER,
			    RG_DPHY_RX_LC1_HS_SETTLE_PARAMETER, val);
		break;
	case REG_KEY_SETTLE_DT:
		SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
			    RG_CDPHY_RX_LD0_TRIO0_HS_SETTLE_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
			    RG_CDPHY_RX_LD1_TRIO1_HS_SETTLE_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
			    RG_CDPHY_RX_LD2_TRIO2_HS_SETTLE_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
			    RG_CDPHY_RX_LD3_TRIO3_HS_SETTLE_PARAMETER, val);
		break;
	case REG_KEY_HS_TRAIL_EN:
		SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
			    RG_DPHY_RX_LD0_HS_TRAIL_EN, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
			    RG_DPHY_RX_LD1_HS_TRAIL_EN, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
			    RG_DPHY_RX_LD2_HS_TRAIL_EN, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
			    RG_DPHY_RX_LD3_HS_TRAIL_EN, val);
		break;
	case REG_KEY_HS_TRAIL_PARAM:
		SENINF_BITS(base, DPHY_RX_DATA_LANE0_HS_PARAMETER,
			    RG_DPHY_RX_LD0_HS_TRAIL_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE1_HS_PARAMETER,
			    RG_DPHY_RX_LD1_HS_TRAIL_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE2_HS_PARAMETER,
			    RG_DPHY_RX_LD2_HS_TRAIL_PARAMETER, val);
		SENINF_BITS(base, DPHY_RX_DATA_LANE3_HS_PARAMETER,
			    RG_DPHY_RX_LD3_HS_TRAIL_PARAMETER, val);
		break;
	case REG_KEY_CSI_IRQ_STAT:
		SENINF_WRITE_REG(csi2, SENINF_CSI2_IRQ_STATUS,
				 val & 0xFFFFFFFF);
		break;
	case REG_KEY_MUX_IRQ_STAT:
		for (i = 0; i < ctx->vcinfo.cnt; i++) {
			vc = &ctx->vcinfo.vc[i];
			pmux = ctx->reg_if_mux[vc->mux];
			SENINF_WRITE_REG(pmux, SENINF_MUX_IRQ_STATUS,
					 val & 0xFFFFFFFF);
		}
		break;
	case REG_KEY_CAMMUX_IRQ_STAT:
		for (i = 0; i < ctx->vcinfo.cnt; i++) {
			vc = &ctx->vcinfo.vc[i];
			pcammux = ctx->reg_if_cam_mux;
			SENINF_WRITE_REG(pcammux, SENINF_CAM_MUX_IRQ_STATUS,
					 val & 0xFFFFFFFF);
		}
		break;
	case REG_KEY_CSI_IRQ_EN:
		{
			if (!val) {
				core->csi_irq_en_flag = 0;
				return 0;
			}

			core->csi_irq_en_flag = 1;
			core->detection_cnt = val;
			list_for_each_entry(ctx_, &core->list, list) {
				csi2_ = ctx_->reg_if_csi2[ctx_->seninf_idx];
				SENINF_WRITE_REG(csi2_, SENINF_CSI2_IRQ_EN,
						 0xA0002058);
			}
		}
		break;
	}

	return 0;
}
