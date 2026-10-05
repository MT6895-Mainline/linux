/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek ISP 7.1 CAMSV registers
 *
 * Offsets and field names follow MediaTek's ISP 7.1 CAMSV register header
 * (mtk_cam-sv-regs.h).
 *
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_CAMSV71_REGS_H__
#define __MTK_CAMSV71_REGS_H__

#include <linux/bits.h>

#define REG_CAMSV_TOP_FBC_CNT_SET		0x0014
#define CAMSV_RCNT_INC1				BIT(0)

#define REG_CAMSV_MODULE_EN			0x0040
#define CAMSV_MODULE_EN_TG_EN			BIT(0)
#define CAMSV_MODULE_EN_PAK_EN			BIT(2)
#define CAMSV_MODULE_EN_IMGO_EN			BIT(4)
#define CAMSV_MODULE_EN_UFE_EN			BIT(6)
#define CAMSV_MODULE_EN_DB_LOAD_SRC		GENMASK(29, 28)
#define CAMSV_DB_LOAD_SRC_SUB_SOF		2
#define CAMSV_MODULE_EN_DB_EN			BIT(30)

#define REG_CAMSV_FMT_SEL			0x0044
#define CAMSV_FMT_SEL_TG1_FMT			GENMASK(2, 0)
#define CAMSV_TG_FMT_RAW10			1

/* INT_EN uses the INT_STATUS layout; INT_STATUS is read-clear. */
#define REG_CAMSV_INT_EN			0x0048
#define REG_CAMSV_INT_STATUS			0x004c
#define CAMSV_INT_TG_ERR			BIT(4)
#define CAMSV_INT_TG_GBERR			BIT(5)
#define CAMSV_INT_TG_SOF			BIT(6)
#define CAMSV_INT_TG_SOF_DROP			BIT(8)
#define CAMSV_INT_DB_LOAD_ERR			BIT(10)
#define CAMSV_INT_PASS1_DON			BIT(11)
#define CAMSV_INT_SW_PASS1_DON			BIT(12)
#define CAMSV_INT_DMA_ERR			BIT(16)
#define CAMSV_INT_IMGO_OVERR			BIT(17)
#define CAMSV_INT_IMGO_DROP			BIT(19)
#define CAMSV_INT_IMGO_DONE			BIT(20)
#define CAMSV_INT_SW_ENQUE_ERR			BIT(24)

/* Bit 2 resets the DMA of both CAMSVs of a GCAMSV pair: never used. */
#define REG_CAMSV_SW_CTL			0x0050
#define CAMSV_SW_CTL_IMGO_RST_TRIG		BIT(0)
#define CAMSV_SW_CTL_IMGO_RST_ST		BIT(1)

#define REG_CAMSV_CLK_EN			0x0060
#define CAMSV_CLK_EN_TG_DP			BIT(0)
#define CAMSV_CLK_EN_QBN_DP			BIT(1)
#define CAMSV_CLK_EN_PAK_DP			BIT(2)
#define CAMSV_CLK_EN_IMGO_DP			BIT(15)

/* Written 0: clears MASK_DB_LOAD, without which the inner registers never latch. */
#define REG_CAMSV_DCIF_SET			0x0074
#define REG_CAMSV_SUB_CTRL			0x0078

#define REG_CAMSV_PAK				0x007c
#define CAMSV_PAK_MODE				GENMASK(7, 0)
#define CAMSV_PAK_DBL_MODE			GENMASK(9, 8)
/* MIPI RAW10 packing, four pixels in five bytes (V4L2_PIX_FMT_S*10P). */
#define CAMSV_PAK_MODE_RAW10_MIPI		143

#define REG_CAMSV_TG_SEN_MODE			0x0100
#define CAMSV_TG_SEN_MODE_CMOS_EN		BIT(0)
#define CAMSV_TG_SEN_MODE_DBL_DATA_BUS		GENMASK(5, 4)
#define CAMSV_TG_SEN_MODE_TIME_STP_EN		BIT(16)

#define REG_CAMSV_TG_VF_CON			0x0104
#define CAMSV_TG_VF_CON_VFDATA_EN		BIT(0)

/* Grab windows: end in [31:16], start (always 0 here) in [15:0]. */
#define REG_CAMSV_TG_SEN_GRAB_PXL		0x0108
#define REG_CAMSV_TG_SEN_GRAB_LIN		0x010c
#define CAMSV_TG_GRAB_END			GENMASK(31, 16)

#define REG_CAMSV_TG_PATH_CFG			0x0110
#define CAMSV_TG_PATH_CFG_DB_LOAD_DIS		BIT(8)

#define REG_CAMSV_TG_INTER_ST			0x013c
#define CAMSV_TG_INTER_ST_CAM_CS		GENMASK(13, 8)
#define CAMSV_TG_CAM_CS_IDLE			1

#define REG_CAMSV_TG_SUB_PERIOD			0x0164

/* Size of the last completed frame. */
#define REG_CAMSV_TG_FRMSIZE_ST_R		0x016c
#define CAMSV_TG_FRMSIZE_W			GENMASK(31, 16)
#define CAMSV_TG_FRMSIZE_H			GENMASK(15, 0)

#define REG_CAMSV_PAK_CON			0x01c0
#define CAMSV_PAK_CON_PAK_IN_BIT		GENMASK(20, 16)

#define REG_CAMSV_FBC_IMGO_CTL1			0x0240
#define CAMSV_FBC_RESET				BIT(8)
#define CAMSV_FBC_DB_EN				BIT(9)
#define CAMSV_FBC_EN				BIT(15)

/*
 * RCNT counts the credits applied at frame starts, WCNT the frames written
 * (field names from another ISP 7.1 copy of the header, behaviour measured on
 * MT6895).
 */
#define REG_CAMSV_FBC_IMGO_CTL2			0x0244
#define CAMSV_FBC_RCNT				GENMASK(7, 0)
#define CAMSV_FBC_WCNT				GENMASK(15, 8)

#define REG_CAMSV_SPECIAL_FUN_EN		0x0600
#define CAMSV_SPECIAL_FUN_DCM_MODE		BIT(26)

#define REG_CAMSV_IMGO_BASE_ADDR		0x0700
#define REG_CAMSV_IMGO_BASE_ADDR_MSB		0x0704
#define CAMSV_IMGO_ADDR_MSB			GENMASK(3, 0)
#define REG_CAMSV_IMGO_OFST_ADDR		0x0708
#define REG_CAMSV_IMGO_OFST_ADDR_MSB		0x070c
#define REG_CAMSV_IMGO_XSIZE			0x0710
#define REG_CAMSV_IMGO_YSIZE			0x0714
#define REG_CAMSV_IMGO_STRIDE			0x0718
#define REG_CAMSV_IMGO_CON(n)			(0x0720 + 4 * (n))
#define CAMSV_IMGO_NUM_CON			5
#define REG_CAMSV_IMGO_ERR_STAT			0x0734
#define REG_CAMSV_IMGO_CROP			0x074c

/* Software tag, latched into the inner window with the address. */
#define REG_CAMSV_FRAME_SEQ_NO			0x075c

/*
 * SMI LARB port control (mtk-smi): the SMI driver owns and programs it, the
 * CAMSV driver only reads it.
 */
#define SMI_LARB_NONSEC_CON(port)		(0x380 + 4 * (port))
#define SMI_LARB_MMU_EN				BIT(0)
#define SMI_LARB_BANK_SEL			GENMASK(15, 8)

#endif /* __MTK_CAMSV71_REGS_H__ */
