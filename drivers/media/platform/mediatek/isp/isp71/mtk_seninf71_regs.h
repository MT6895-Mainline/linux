/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6895 SENINF register map
 *
 * Copyright (c) 2020 MediaTek Inc.
 *
 * Offsets and field names are MediaTek's, from the GPL-2.0 ISP 7.1 driver in
 * MiCode/Xiaomi_Kernel_OpenSource 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/media/platform/mtk-isp/camsys/isp7_1/cam/mtk_csi_phy_3_0/
 * mtk_cam-seninf-{top-ctrl,cammux-gcsr,cammux-pcsr,seninf1,seninf1-csi2,
 * seninf1-mux}.h, rewritten as BIT()/GENMASK() masks.
 */

#ifndef __MTK_SENINF71_REGS_H__
#define __MTK_SENINF71_REGS_H__

#include <linux/bits.h>

/* Interface n and mux n share the register page n. */
#define SENINF_PAGE(n)				((n) * 0x1000)

/* Top control */
#define SENINF_TOP_MUX_CTRL(m)			(0x0010 + 4 * ((m) / 4))
#define  RG_SENINF_TOP_MUX_SRC_SEL(m)		(GENMASK(4, 0) << (8 * ((m) % 4)))
#define SENINF_TOP_PHY_CTRL_CSI(p)		(0x0040 + 4 * (p))
#define  PHY_SENINF_DPHY_EN			BIT(0)
#define  PHY_SENINF_CPHY_EN			BIT(1)
#define  RG_PHY_SENINF_CPHY_MODE		GENMASK(9, 8)

/* CAM_MUX global control */
#define SENINF_CAM_MUX_GCSR_BASE		0x0300
#define SENINF_CAM_MUX_GCSR_DYN_CTRL		0x0004
#define  RG_SENINF_CAM_MUX_GCSR_DYN_SWITCH_BSY0	BIT(0)
#define  RG_SENINF_CAM_MUX_GCSR_DYN_SWITCH_BSY1	BIT(1)
#define  RG_SENINF_CAM_MUX_GCSR_DYN_PAGE_SEL	BIT(7)
#define  RG_SENINF_CAM_MUX_GCSR_OCC_PG1_EN	BIT(8)
#define SENINF_CAM_MUX_GCSR_MUX_EN		0x00c0
#define SENINF_CAM_MUX_GCSR_DYN_EN0		0x00c8
#define SENINF_CAM_MUX_GCSR_DYN_EN1		0x00cc

/* CAM_MUX k control */
#define SENINF_CAM_MUX_PCSR_BASE(k)		(0x0400 + 0x20 * (k))
#define SENINF_CAM_MUX_PCSR_CTRL		0x0000
#define  RG_SENINF_CAM_MUX_PCSR_SRC_SEL		GENMASK(4, 0)
#define  RG_SENINF_CAM_MUX_PCSR_EN		BIT(7)
#define  RG_SENINF_CAM_MUX_PCSR_CHK_PIX_MODE	GENMASK(9, 8)
#define  RG_SENINF_CAM_MUX_PCSR_CHK_EN		BIT(15)
#define  CAM_MUX_PCSR_NEXT_SRC_SEL		GENMASK(20, 16)
#define  RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN0	BIT(22)
#define  RG_SENINF_CAM_MUX_PCSR_DYN_SWITCH_EN1	BIT(23)
#define SENINF_CAM_MUX_PCSR_OPT			0x0004
#define  RG_SENINF_CAM_MUX_PCSR_VC_SEL		GENMASK(4, 0)
#define  RG_SENINF_CAM_MUX_PCSR_VC_SEL_EN	BIT(7)
#define  RG_SENINF_CAM_MUX_PCSR_DT_SEL		GENMASK(13, 8)
#define  RG_SENINF_CAM_MUX_PCSR_DT_SEL_EN	BIT(15)
#define SENINF_CAM_MUX_PCSR_IRQ_EN		0x0008
#define SENINF_CAM_MUX_PCSR_IRQ_STATUS		0x000c
#define  RO_SENINF_CAM_MUX_PCSR_HSIZE_ERR_IRQ	BIT(0)
#define  RO_SENINF_CAM_MUX_PCSR_VSIZE_ERR_IRQ	BIT(1)
#define  RO_SENINF_CAM_MUX_PCSR_VSYNC_IRQ	BIT(8)
#define SENINF_CAM_MUX_PCSR_CHK_CTL		0x0014
#define  RG_SENINF_CAM_MUX_PCSR_EXP_HSIZE	GENMASK(15, 0)
#define  RG_SENINF_CAM_MUX_PCSR_EXP_VSIZE	GENMASK(31, 16)
#define SENINF_CAM_MUX_PCSR_CHK_RES		0x0018
/* SRC_SEL and NEXT_SRC_SEL value that selects no mux */
#define SENINF_CAM_MUX_SRC_NONE			0x1f

/* Interface i control */
#define SENINF_INTF_BASE(i)			(SENINF_PAGE(i) + 0x0200)
#define SENINF_CTRL				0x0000
#define  SENINF_EN				BIT(0)
#define SENINF_CSI2_CTRL			0x0010
#define  RG_SENINF_CSI2_EN			BIT(0)
#define  SENINF_CSI2_SW_RST			BIT(4)
#define SENINF_TESTMDL_CTRL			0x0020
#define  RG_SENINF_TESTMDL_EN			BIT(0)

/* Interface i CSI-2 receiver */
#define SENINF_CSI2_BASE(i)			(SENINF_PAGE(i) + 0x0a00)
#define SENINF_CSI2_EN				0x0000
#define SENINF_CSI2_OPT				0x0004
#define  RG_CSI2_CPHY_SEL			BIT(0)
#define SENINF_CSI2_HDR_MODE_0			0x0008
#define  RG_CSI2_HEADER_MODE			GENMASK(7, 0)
#define  RG_CSI2_HEADER_LEN			GENMASK(10, 8)
#define SENINF_CSI2_RESYNC_MERGE_CTRL		0x0010
#define  RG_CSI2_RESYNC_DMY_EN			GENMASK(15, 12)
#define  RG_CSI2_RESYNC_DMY_CYCLE		GENMASK(27, 16)
#define  RG_CSI2_RESYNC_DMY_CNT			GENMASK(31, 28)
/* D-PHY base value of RESYNC_MERGE_CTRL, before the dummy-cycle fields */
#define SENINF_CSI2_RESYNC_MERGE_DPHY		0x2020f106
#define SENINF_CSI2_S_DI_CTRL(s)		(0x0020 + 4 * (s))
#define  RG_CSI2_S_VC_INTERLEAVE_EN		BIT(0)
#define  RG_CSI2_S_DT_INTERLEAVE_MODE		GENMASK(5, 4)
#define  RG_CSI2_S_VC_SEL			GENMASK(12, 8)
#define  RG_CSI2_S_DT_SEL			GENMASK(21, 16)
#define SENINF_CSI2_NUM_DI			8
#define SENINF_CSI2_CH_CTRL(c)			(0x0060 + 4 * (c))
#define  RG_CSI2_CH_S_GRP_EN(s)			BIT(8 + (s))
#define SENINF_CSI2_NUM_CH			4
/* Bit 31 of IRQ_EN is RG_CSI2_IRQ_CLR_MODE (reset value 1); never written. */
#define SENINF_CSI2_IRQ_EN			0x00c0
#define  RG_CSI2_IRQ_EN_ALL			GENMASK(30, 0)
#define SENINF_CSI2_IRQ_STATUS			0x00c8
/*
 * Error bits: frame sync, packet ID, corrected and double ECC, CRC, multi-lane
 * sync, lane resync, merge FIFO almost full, per-stream frame sync, resync
 * FIFO overflow, async FIFO overrun and short data. Bits 2, 5 and 8-12 report
 * normal packets.
 */
#define  SENINF_CSI2_IRQ_ERRORS			(GENMASK(29, 16) | BIT(14) | BIT(13) | \
						 BIT(7) | BIT(6) | BIT(4) | BIT(3) | \
						 BIT(1) | BIT(0))
/* Frame and line counters and the header of the last packet received */
#define SENINF_CSI2_LINE_FRAME_NUM		0x00d0
#define  RO_CSI2_LINE_NUM			GENMASK(15, 0)
#define  RO_CSI2_FRAME_NUM			GENMASK(31, 16)
#define SENINF_CSI2_PACKET_STATUS		0x00d4
#define  RO_CSI2_PACKET_DT			GENMASK(5, 0)
#define  RO_CSI2_PACKET_VC			GENMASK(12, 8)
#define  RO_CSI2_PACKET_WC			GENMASK(31, 16)
#define SENINF_CSI2_PACKET_CNT_STATUS		0x00dc
#define  RO_CSI2_PACKET_CNT			GENMASK(15, 0)
#define  RO_CSI2_PACKET_CNT_BUF			GENMASK(31, 16)
#define SENINF_CSI2_DBG_CTRL			0x00e0
#define  RG_CSI2_DBG_PACKET_CNT_EN		BIT(17)

/* Mux m */
#define SENINF_MUX_BASE(m)			(SENINF_PAGE(m) + 0x0d00)
#define SENINF_MUX_CTRL_0			0x0000
#define  SENINF_MUX_EN				BIT(0)
#define  SENINF_MUX_IRQ_SW_RST			BIT(1)
#define  SENINF_MUX_SW_RST			BIT(2)
#define SENINF_MUX_CTRL_1			0x0004
#define  RG_SENINF_MUX_SRC_SEL			GENMASK(3, 0)
#define  RG_SENINF_MUX_PIX_MODE_SEL		GENMASK(9, 8)
#define  RG_SENINF_MUX_FIFO_PUSH_EN		GENMASK(21, 16)
/* MIPI_SENSOR + CSI-2 channel group 0 (mtk_cam-seninf-def.h) */
#define SENINF_MUX_SRC_MIPI_SENSOR		8
#define SENINF_MUX_FIFO_PUSH_ALL		0x1f
#define SENINF_MUX_OPT				0x0008
#define  RG_SENINF_MUX_HSYNC_POL		BIT(16)
#define  RG_SENINF_MUX_VSYNC_POL		BIT(17)
/* Write 1 to clear; the vendor driver polls it with the interrupts disabled. */
#define SENINF_MUX_IRQ_STATUS			0x0018
#define  RO_SENINF_MUX_FIFO_OVERRUN_IRQ		BIT(0)
#define  RO_SENINF_MUX_FSM_ERR_IRQ		BIT(1)
#define  RO_SENINF_MUX_HSIZE_ERR_IRQ		BIT(2)
#define  RO_SENINF_MUX_VSIZE_ERR_IRQ		BIT(3)
/*
 * Only a FIFO overrun is an error. The size checks compare the frame with
 * SENINF_MUX_IMG_SIZE, which neither this driver nor the vendor driver
 * programs, so HSIZE_ERR and VSIZE_ERR are set after every frame.
 */
#define  SENINF_MUX_IRQ_ERRORS			RO_SENINF_MUX_FIFO_OVERRUN_IRQ
#define SENINF_MUX_SIZE				0x0030
/* Received size of the frame that set a size error */
#define SENINF_MUX_ERR_SIZE			0x0034
#define  RO_SENINF_MUX_RCV_ERR_HSIZE		GENMASK(15, 0)
#define  RO_SENINF_MUX_RCV_ERR_VSIZE		GENMASK(31, 16)
#define SENINF_MUX_FIFO_STATUS			0x0040
#define SENINF_MUX_FRAME_SIZE_MON_CTRL		0x00a8
#define  RG_SENINF_MUX_FRAME_SIZE_MON_EN	BIT(0)
#define SENINF_MUX_FRAME_SIZE_MON_H_VALID	0x00b0
#define SENINF_MUX_FRAME_SIZE_MON_H_BLANK	0x00b4
#define SENINF_MUX_FRAME_SIZE_MON_V_VALID	0x00b8
#define SENINF_MUX_FRAME_SIZE_MON_V_BLANK	0x00bc

#endif /* __MTK_SENINF71_REGS_H__ */
