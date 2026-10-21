/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Raw page-3 vendor MAIN blocks the replay table does not cover
 * (0x2840 is separately replayed via xaga_aa_tbl). Extracted from the
 * Android CQ dump (dmesg-android-cam5.txt); see isp/cam-yuv-plan.md.
 * 0x45d0 is a 36-bit DMA base+msb pair pointing at the vendor coefficient
 * buffer -> word0/word1 are substituted at runtime. Generated.
 */
#ifndef _MTK_CAM_RAW_EXTRA_TBL_H
#define _MTK_CAM_RAW_EXTRA_TBL_H

static const u32 xaga_raw_extra_0[6] = {
	0x00000001, 0x0000080a, 0x00d900e8, 0x0000c4a8, 0x01000006, 0x00018950,
};

static const u32 xaga_raw_extra_1[2] = {
	0x000003ff, 0x00080040,
};

static const u32 xaga_raw_extra_2[1] = {
	0x00000000,
};

static const u32 xaga_raw_extra_3[5] = {
	0x0d900910, 0x06c80910, 0x090f0000, 0x06c70000, 0x00000c92,
};

static const u32 xaga_raw_extra_4[7] = {
	0xdd7f07ec, 0x00000003, 0x00000000, 0x00000000, 0x000007ff, 0x0000000f, 0x00000800,
};

static const struct xaga_yuv_main_block xaga_raw_extra_tbl[] = {
	{ 0x2500,   6, xaga_raw_extra_0 },  /*  */
	{ 0x2520,   2, xaga_raw_extra_1 },  /*  */
	{ 0x2550,   1, xaga_raw_extra_2 },  /*  */
	{ 0x2570,   5, xaga_raw_extra_3 },  /*  */
	{ 0x45d0,   7, xaga_raw_extra_4 },  /* DMA base+size (substitute) */
};

#define XAGA_RAW_EXTRA_TBL_NUM	ARRAY_SIZE(xaga_raw_extra_tbl)
#define XAGA_RAW_EXTRA_DMA_BASE_REG	0x45d0

#endif /* _MTK_CAM_RAW_EXTRA_TBL_H */
