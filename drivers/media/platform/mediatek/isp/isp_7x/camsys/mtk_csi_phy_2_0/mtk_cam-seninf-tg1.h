/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#ifndef __SENINF_TG1_H__
#define __SENINF_TG1_H__

#include <linux/bitfield.h>

// TM_CTL
#define TM_CTL 0x0008
#define TM_EN                  GENMASK(0, 0)
#define TM_RST                 GENMASK(1, 1)
#define TM_FMT                 GENMASK(2, 2)
#define TM_BIN_IMG_SWITCH_EN   GENMASK(3, 3)
#define TM_PAT                 GENMASK(8, 4)

// TM_SIZE
#define TM_SIZE 0x000c
#define TM_PXL  GENMASK(15, 0)
#define TM_LINE GENMASK(31, 16)

// TM_CLK
#define TM_CLK 0x0010
#define TM_CLK_CNT      GENMASK(7, 0)
#define TM_CLRBAR_OFT   GENMASK(20, 8)
#define TM_CLRBAR_IDX   GENMASK(30, 28)

// TM_DUM
#define TM_DUM 0x0018
#define TM_DUMMYPXL GENMASK(15, 0)
#define TM_VSYNC    GENMASK(31, 16)

// TM_RAND_SEED
#define TM_RAND_SEED 0x001c
#define TM_SEED GENMASK(31, 0)

// TM_RAND_CTL
#define TM_RAND_CTL 0x0020
#define TM_DIFF_FRM GENMASK(0, 0)

#endif
