// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX582 image sensor driver
 *
 * Copyright (c) 2019 MediaTek Inc.
 *
 * The register tables, the power sequence and the control registers are
 * taken from the MediaTek imgsensor driver of the Xiaomi Redmi K50 main
 * camera, MiCode/Xiaomi_Kernel_OpenSource commit
 * 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/misc/mediatek/imgsensor/src-v4l2/common/rubensimx582_mipi_raw/.
 * The table sections follow the Sony setting sheet as reproduced in
 * Rockchip's imx582.c (Copyright (C) 2025 Rockchip Electronics Co., Ltd.).
 * The V4L2 structure follows gc02m1.c and s5k3m5.c (Copyright (c) 2025
 * Linaro Ltd).
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/units.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define IMX582_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX582_CHIP_ID			0x0582

#define IMX582_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX582_MODE_STANDBY		0x00
#define IMX582_MODE_STREAMING		0x01
#define IMX582_REG_IMG_ORIENTATION	CCI_REG8(0x0101)
#define IMX582_HFLIP			BIT(0)
#define IMX582_VFLIP			BIT(1)
#define IMX582_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define IMX582_REG_TEMP_SEN_CTL		CCI_REG8(0x0138)
#define IMX582_TEMP_SEN_EN		0x01
#define IMX582_REG_COARSE_INTEG_TIME	CCI_REG16(0x0202)
#define IMX582_REG_ANA_GAIN_GLOBAL	CCI_REG16(0x0204)
/* 0x020e is also the global digital gain of the CCS register layout. */
#define IMX582_REG_DIG_GAIN_GR		CCI_REG16(0x020e)
#define IMX582_REG_DIG_GAIN_R		CCI_REG16(0x0210)
#define IMX582_REG_DIG_GAIN_B		CCI_REG16(0x0212)
#define IMX582_REG_DIG_GAIN_GB		CCI_REG16(0x0214)
#define IMX582_REG_FRM_LENGTH_LINES	CCI_REG16(0x0340)
#define IMX582_REG_FRM_LENGTH_CTL	CCI_REG8(0x0350)
#define IMX582_FRM_LENGTH_AUTO		0x01
#define IMX582_REG_TEST_PATTERN_MODE	CCI_REG16(0x0600)
/* 0x00 selects the normal mode transition, 0x01 the fast one. */
#define IMX582_REG_MODE_TRANSITION	CCI_REG8(0x3020)

#define IMX582_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define IMX582_LINK_FREQ		(685 * HZ_PER_MHZ)
#define IMX582_DATA_LANES		4
#define IMX582_PIXEL_RATE_MIN		734400000
#define IMX582_PIXEL_RATE_MAX		864000000

#define IMX582_NATIVE_WIDTH		8000
#define IMX582_NATIVE_HEIGHT		6000

#define IMX582_FRAME_LENGTH_MAX		0xffff

/* Vendor limits: at least 6 lines, steps of 2, 48 lines before the frame end */
#define IMX582_EXPOSURE_MIN		6
#define IMX582_EXPOSURE_STEP		2
#define IMX582_EXPOSURE_MARGIN		48

/*
 * V4L2_CID_ANALOGUE_GAIN is the gain in 1/1024 units. The ANA_GAIN_GLOBAL
 * code c gives a gain of 1024 / (1024 - c); the vendor limits are the codes
 * 112 to 1008, 1.12x to 64x.
 */
#define IMX582_GAIN_UNIT		1024
#define IMX582_ANA_GAIN_CODE_MIN	112
#define IMX582_ANA_GAIN_CODE_MAX	1008
#define IMX582_ANA_GAIN_MIN		(IMX582_GAIN_UNIT * IMX582_GAIN_UNIT / \
					 (IMX582_GAIN_UNIT - IMX582_ANA_GAIN_CODE_MIN))
#define IMX582_ANA_GAIN_MAX		(IMX582_GAIN_UNIT * IMX582_GAIN_UNIT / \
					 (IMX582_GAIN_UNIT - IMX582_ANA_GAIN_CODE_MAX))
#define IMX582_ANA_GAIN_DEFAULT		IMX582_ANA_GAIN_MIN

/* DIG_GAIN_* are 4.8 fixed point; 0x100 is 1.0, the value of the tables. */
#define IMX582_DIG_GAIN_MIN		0x0100
#define IMX582_DIG_GAIN_MAX		0x0fff
#define IMX582_DIG_GAIN_DEFAULT		0x0100

#define IMX582_AUTOSUSPEND_DELAY_MS	1000

static const s64 imx582_link_freq_menu[] = {
	IMX582_LINK_FREQ,
};

static const char * const imx582_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

/*
 * Indexed by (vflip << 1) | hflip. Rockchip's driver, which never flips, uses
 * RGGB; the vendor driver flips both ways and reports a blue-first order. The
 * mixed orders follow from these two.
 */
static const u32 imx582_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const struct v4l2_rect imx582_native_rect = {
	.left = 0,
	.top = 0,
	.width = IMX582_NATIVE_WIDTH,
	.height = IMX582_NATIVE_HEIGHT,
};

/* Power-up order and the delay after each step; power-down reverses it. */
struct imx582_supply_desc {
	const char *name;
	bool optional;
	unsigned int delay_us;
};

static const struct imx582_supply_desc imx582_supply_descs[] = {
	/* The actuator of the module shares the bus and is powered first. */
	{ "afvdd", true, 0 },
	{ "vcam", true, 1000 },
	{ "avdd", false, 1000 },
	{ "avdd2", false, 1000 },
	{ "dvdd", false, 1000 },
	{ "dovdd", false, 1000 },
};

#define IMX582_NUM_SUPPLIES		ARRAY_SIZE(imx582_supply_descs)

/* Global setting (vendor rubensimx582_init_setting), after every power-up */
static const struct cci_reg_sequence imx582_init_regs[] = {
	/* external clock 24 MHz */
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	/* register set version */
	{ CCI_REG8(0x3c7e), 0x03 },
	{ CCI_REG8(0x3c7f), 0x06 },
	/* global setting */
	{ CCI_REG8(0x3c00), 0x10 },
	{ CCI_REG8(0x3c01), 0x10 },
	{ CCI_REG8(0x3c02), 0x10 },
	{ CCI_REG8(0x3c03), 0x10 },
	{ CCI_REG8(0x3c04), 0x10 },
	{ CCI_REG8(0x3c05), 0x01 },
	{ CCI_REG8(0x3c06), 0x00 },
	{ CCI_REG8(0x3c07), 0x00 },
	{ CCI_REG8(0x3c08), 0x03 },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c0a), 0x01 },
	{ CCI_REG8(0x3c0b), 0x00 },
	{ CCI_REG8(0x3c0c), 0x00 },
	{ CCI_REG8(0x3c0d), 0x03 },
	{ CCI_REG8(0x3c0e), 0xff },
	{ CCI_REG8(0x3c0f), 0x20 },
	{ CCI_REG8(0x6e1d), 0x00 },
	{ CCI_REG8(0x6e25), 0x00 },
	{ CCI_REG8(0x6e38), 0x03 },
	{ CCI_REG8(0x6e3b), 0x01 },
	{ CCI_REG8(0x9004), 0x2c },
	{ CCI_REG8(0x9200), 0xf4 },
	{ CCI_REG8(0x9201), 0xa7 },
	{ CCI_REG8(0x9202), 0xf4 },
	{ CCI_REG8(0x9203), 0xaa },
	{ CCI_REG8(0x9204), 0xf4 },
	{ CCI_REG8(0x9205), 0xad },
	{ CCI_REG8(0x9206), 0xf4 },
	{ CCI_REG8(0x9207), 0xb0 },
	{ CCI_REG8(0x9208), 0xf4 },
	{ CCI_REG8(0x9209), 0xb3 },
	{ CCI_REG8(0x920a), 0xb7 },
	{ CCI_REG8(0x920b), 0x34 },
	{ CCI_REG8(0x920c), 0xb7 },
	{ CCI_REG8(0x920d), 0x36 },
	{ CCI_REG8(0x920e), 0xb7 },
	{ CCI_REG8(0x920f), 0x37 },
	{ CCI_REG8(0x9210), 0xb7 },
	{ CCI_REG8(0x9211), 0x38 },
	{ CCI_REG8(0x9212), 0xb7 },
	{ CCI_REG8(0x9213), 0x39 },
	{ CCI_REG8(0x9214), 0xb7 },
	{ CCI_REG8(0x9215), 0x3a },
	{ CCI_REG8(0x9216), 0xb7 },
	{ CCI_REG8(0x9217), 0x3c },
	{ CCI_REG8(0x9218), 0xb7 },
	{ CCI_REG8(0x9219), 0x3d },
	{ CCI_REG8(0x921a), 0xb7 },
	{ CCI_REG8(0x921b), 0x3e },
	{ CCI_REG8(0x921c), 0xb7 },
	{ CCI_REG8(0x921d), 0x3f },
	{ CCI_REG8(0x921e), 0x85 },
	{ CCI_REG8(0x921f), 0x77 },
	{ CCI_REG8(0x9226), 0x42 },
	{ CCI_REG8(0x9227), 0x52 },
	{ CCI_REG8(0x9228), 0x60 },
	{ CCI_REG8(0x9229), 0xb9 },
	{ CCI_REG8(0x922a), 0x60 },
	{ CCI_REG8(0x922b), 0xbf },
	{ CCI_REG8(0x922c), 0x60 },
	{ CCI_REG8(0x922d), 0xc5 },
	{ CCI_REG8(0x922e), 0x60 },
	{ CCI_REG8(0x922f), 0xcb },
	{ CCI_REG8(0x9230), 0x60 },
	{ CCI_REG8(0x9231), 0xd1 },
	{ CCI_REG8(0x9232), 0x60 },
	{ CCI_REG8(0x9233), 0xd7 },
	{ CCI_REG8(0x9234), 0x60 },
	{ CCI_REG8(0x9235), 0xdd },
	{ CCI_REG8(0x9236), 0x60 },
	{ CCI_REG8(0x9237), 0xe3 },
	{ CCI_REG8(0x9238), 0x60 },
	{ CCI_REG8(0x9239), 0xe9 },
	{ CCI_REG8(0x923a), 0x60 },
	{ CCI_REG8(0x923b), 0xef },
	{ CCI_REG8(0x923c), 0x60 },
	{ CCI_REG8(0x923d), 0xf5 },
	{ CCI_REG8(0x923e), 0x60 },
	{ CCI_REG8(0x923f), 0xf9 },
	{ CCI_REG8(0x9240), 0x60 },
	{ CCI_REG8(0x9241), 0xfd },
	{ CCI_REG8(0x9242), 0x61 },
	{ CCI_REG8(0x9243), 0x01 },
	{ CCI_REG8(0x9244), 0x61 },
	{ CCI_REG8(0x9245), 0x05 },
	{ CCI_REG8(0x924a), 0x61 },
	{ CCI_REG8(0x924b), 0x6b },
	{ CCI_REG8(0x924c), 0x61 },
	{ CCI_REG8(0x924d), 0x7f },
	{ CCI_REG8(0x924e), 0x61 },
	{ CCI_REG8(0x924f), 0x92 },
	{ CCI_REG8(0x9250), 0x61 },
	{ CCI_REG8(0x9251), 0x9c },
	{ CCI_REG8(0x9252), 0x61 },
	{ CCI_REG8(0x9253), 0xab },
	{ CCI_REG8(0x9254), 0x61 },
	{ CCI_REG8(0x9255), 0xc4 },
	{ CCI_REG8(0x9256), 0x61 },
	{ CCI_REG8(0x9257), 0xce },
	{ CCI_REG8(0x9810), 0x14 },
	{ CCI_REG8(0x9814), 0x14 },
	{ CCI_REG8(0xc449), 0x04 },
	{ CCI_REG8(0xc44a), 0x01 },
	{ CCI_REG8(0xe286), 0x31 },
	{ CCI_REG8(0xe2a6), 0x32 },
	{ CCI_REG8(0xe2c6), 0x33 },
	/* vendor additions, not in the Rockchip copy of the sheet */
	{ CCI_REG8(0xe186), 0x2b },
	{ CCI_REG8(0x3e14), 0x01 },
};

/* Image quality setting, written after the global setting */
static const struct cci_reg_sequence imx582_iq_regs[] = {
	{ CCI_REG8(0x88d6), 0x60 },
	{ CCI_REG8(0x9852), 0x00 },
	{ CCI_REG8(0xae09), 0xff },
	{ CCI_REG8(0xae0a), 0xff },
	{ CCI_REG8(0xae12), 0x58 },
	{ CCI_REG8(0xae13), 0x58 },
	{ CCI_REG8(0xae15), 0x10 },
	{ CCI_REG8(0xae16), 0x10 },
	{ CCI_REG8(0xb071), 0x00 },
};

/*
 * 1920x1080: H4V4 binning of 8000x4320 from (0, 832), digital crop
 * from x = 40. VT 734.4 MHz, OP 1370 Mbit/s per lane, line length
 * 2912, frame length 2100 (120 fps). No PDAF output.
 * Vendor custom2, unchanged.
 */
static const struct cci_reg_sequence imx582_1920x1080_regs[] = {
	/* RAW10 output, 4 data lanes */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	/* line length (pixel clocks) and frame length (lines) */
	{ CCI_REG8(0x0342), 0x0b },
	{ CCI_REG8(0x0343), 0x60 },
	{ CCI_REG8(0x0340), 0x08 },
	{ CCI_REG8(0x0341), 0x34 },
	/* analogue crop */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x03 },
	{ CCI_REG8(0x0347), 0x40 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0x3f },
	{ CCI_REG8(0x034a), 0x14 },
	{ CCI_REG8(0x034b), 0x1f },
	/* binning */
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x44 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3246), 0x89 },
	{ CCI_REG8(0x3247), 0x89 },
	/* digital crop and scaling (no scaling) */
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x28 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x07 },
	{ CCI_REG8(0x040d), 0x80 },
	{ CCI_REG8(0x040e), 0x04 },
	{ CCI_REG8(0x040f), 0x38 },
	/* output size */
	{ CCI_REG8(0x034c), 0x07 },
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x04 },
	{ CCI_REG8(0x034f), 0x38 },
	/* clocks: EXCK 24 MHz; VT and OP PLL */
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x02 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x99 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x18 },
	{ CCI_REG8(0x030e), 0x05 },
	{ CCI_REG8(0x030f), 0x5a },
	{ CCI_REG8(0x0310), 0x01 },
	/* other settings */
	{ CCI_REG8(0x3620), 0x00 },
	{ CCI_REG8(0x3621), 0x00 },
	{ CCI_REG8(0x380c), 0x80 },
	{ CCI_REG8(0x3c13), 0x00 },
	{ CCI_REG8(0x3c14), 0x28 },
	{ CCI_REG8(0x3c15), 0x28 },
	{ CCI_REG8(0x3c16), 0x32 },
	{ CCI_REG8(0x3c17), 0x46 },
	{ CCI_REG8(0x3c18), 0x67 },
	{ CCI_REG8(0x3c19), 0x8f },
	{ CCI_REG8(0x3c1a), 0x8f },
	{ CCI_REG8(0x3c1b), 0x99 },
	{ CCI_REG8(0x3c1c), 0xad },
	{ CCI_REG8(0x3c1d), 0xce },
	{ CCI_REG8(0x3c1e), 0x8f },
	{ CCI_REG8(0x3c1f), 0x8f },
	{ CCI_REG8(0x3c20), 0x99 },
	{ CCI_REG8(0x3c21), 0xad },
	{ CCI_REG8(0x3c22), 0xce },
	{ CCI_REG8(0x3c25), 0x22 },
	{ CCI_REG8(0x3c26), 0x23 },
	{ CCI_REG8(0x3c27), 0xe6 },
	{ CCI_REG8(0x3c28), 0xe6 },
	{ CCI_REG8(0x3c29), 0x08 },
	{ CCI_REG8(0x3c2a), 0x0f },
	{ CCI_REG8(0x3c2b), 0x14 },
	{ CCI_REG8(0x3f0c), 0x00 },
	{ CCI_REG8(0x3f14), 0x00 },
	{ CCI_REG8(0x3f80), 0x00 },
	{ CCI_REG8(0x3f81), 0x00 },
	{ CCI_REG8(0x3f82), 0x00 },
	{ CCI_REG8(0x3f83), 0x00 },
	{ CCI_REG8(0x3f8c), 0x00 },
	{ CCI_REG8(0x3f8d), 0x00 },
	{ CCI_REG8(0x3ff4), 0x00 },
	{ CCI_REG8(0x3ff5), 0x4c },
	{ CCI_REG8(0x3ffc), 0x00 },
	{ CCI_REG8(0x3ffd), 0x00 },
	/* integration times; V4L2_CID_EXPOSURE sets 0x0202 */
	{ CCI_REG8(0x0202), 0x08 },
	{ CCI_REG8(0x0203), 0x04 },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3fe0), 0x01 },
	{ CCI_REG8(0x3fe1), 0xf4 },
	/* gains; the controls set 0x0204 and 0x020e-0x0215 */
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x70 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x70 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0210), 0x01 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x01 },
	{ CCI_REG8(0x0213), 0x00 },
	{ CCI_REG8(0x0214), 0x01 },
	{ CCI_REG8(0x0215), 0x00 },
	{ CCI_REG8(0x3fe2), 0x00 },
	{ CCI_REG8(0x3fe3), 0x70 },
	{ CCI_REG8(0x3fe4), 0x01 },
	{ CCI_REG8(0x3fe5), 0x00 },
	/* PDAF: type and phase data output (0x3e3b) */
	{ CCI_REG8(0x3e20), 0x01 },
	{ CCI_REG8(0x3e3b), 0x00 },
	{ CCI_REG8(0x4034), 0x01 },
	{ CCI_REG8(0x4035), 0xf0 },
};

/*
 * 4000x3000: H2V2 binning of the full array. VT 864 MHz, OP
 * 1370 Mbit/s per lane, line length 7872, frame length 3658
 * (30 fps). Vendor preview with 0x3e3b = 0x00: the phase data
 * on virtual channel 1 is off, as in the vendor custom3/custom4.
 */
static const struct cci_reg_sequence imx582_4000x3000_regs[] = {
	/* RAW10 output, 4 data lanes */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	/* line length (pixel clocks) and frame length (lines) */
	{ CCI_REG8(0x0342), 0x1e },
	{ CCI_REG8(0x0343), 0xc0 },
	{ CCI_REG8(0x0340), 0x0e },
	{ CCI_REG8(0x0341), 0x4a },
	/* analogue crop */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x1f },
	{ CCI_REG8(0x0349), 0x3f },
	{ CCI_REG8(0x034a), 0x17 },
	{ CCI_REG8(0x034b), 0x6f },
	/* binning */
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0902), 0x08 },
	{ CCI_REG8(0x3246), 0x81 },
	{ CCI_REG8(0x3247), 0x81 },
	/* digital crop and scaling (no scaling) */
	{ CCI_REG8(0x0401), 0x00 },
	{ CCI_REG8(0x0404), 0x00 },
	{ CCI_REG8(0x0405), 0x10 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f },
	{ CCI_REG8(0x040d), 0xa0 },
	{ CCI_REG8(0x040e), 0x0b },
	{ CCI_REG8(0x040f), 0xb8 },
	/* output size */
	{ CCI_REG8(0x034c), 0x0f },
	{ CCI_REG8(0x034d), 0xa0 },
	{ CCI_REG8(0x034e), 0x0b },
	{ CCI_REG8(0x034f), 0xb8 },
	/* clocks: EXCK 24 MHz; VT and OP PLL */
	{ CCI_REG8(0x0301), 0x05 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x02 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xb4 },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x18 },
	{ CCI_REG8(0x030e), 0x05 },
	{ CCI_REG8(0x030f), 0x5a },
	{ CCI_REG8(0x0310), 0x01 },
	/* other settings */
	{ CCI_REG8(0x3620), 0x00 },
	{ CCI_REG8(0x3621), 0x00 },
	{ CCI_REG8(0x380c), 0x80 },
	{ CCI_REG8(0x3c13), 0x00 },
	{ CCI_REG8(0x3c14), 0x28 },
	{ CCI_REG8(0x3c15), 0x28 },
	{ CCI_REG8(0x3c16), 0x32 },
	{ CCI_REG8(0x3c17), 0x46 },
	{ CCI_REG8(0x3c18), 0x67 },
	{ CCI_REG8(0x3c19), 0x8f },
	{ CCI_REG8(0x3c1a), 0x8f },
	{ CCI_REG8(0x3c1b), 0x99 },
	{ CCI_REG8(0x3c1c), 0xad },
	{ CCI_REG8(0x3c1d), 0xce },
	{ CCI_REG8(0x3c1e), 0x8f },
	{ CCI_REG8(0x3c1f), 0x8f },
	{ CCI_REG8(0x3c20), 0x99 },
	{ CCI_REG8(0x3c21), 0xad },
	{ CCI_REG8(0x3c22), 0xce },
	{ CCI_REG8(0x3c25), 0x22 },
	{ CCI_REG8(0x3c26), 0x23 },
	{ CCI_REG8(0x3c27), 0xe6 },
	{ CCI_REG8(0x3c28), 0xe6 },
	{ CCI_REG8(0x3c29), 0x08 },
	{ CCI_REG8(0x3c2a), 0x0f },
	{ CCI_REG8(0x3c2b), 0x14 },
	{ CCI_REG8(0x3f0c), 0x01 },
	{ CCI_REG8(0x3f14), 0x00 },
	{ CCI_REG8(0x3f80), 0x07 },
	{ CCI_REG8(0x3f81), 0xb9 },
	{ CCI_REG8(0x3f82), 0x00 },
	{ CCI_REG8(0x3f83), 0x00 },
	{ CCI_REG8(0x3f8c), 0x02 },
	{ CCI_REG8(0x3f8d), 0x8b },
	{ CCI_REG8(0x3ff4), 0x00 },
	{ CCI_REG8(0x3ff5), 0x00 },
	{ CCI_REG8(0x3ffc), 0x01 },
	{ CCI_REG8(0x3ffd), 0x0e },
	/* integration times; V4L2_CID_EXPOSURE sets 0x0202 */
	{ CCI_REG8(0x0202), 0x0e },
	{ CCI_REG8(0x0203), 0x1a },
	{ CCI_REG8(0x0224), 0x01 },
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3fe0), 0x01 },
	{ CCI_REG8(0x3fe1), 0xf4 },
	/* gains; the controls set 0x0204 and 0x020e-0x0215 */
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x70 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x70 },
	{ CCI_REG8(0x0218), 0x01 },
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0210), 0x01 },
	{ CCI_REG8(0x0211), 0x00 },
	{ CCI_REG8(0x0212), 0x01 },
	{ CCI_REG8(0x0213), 0x00 },
	{ CCI_REG8(0x0214), 0x01 },
	{ CCI_REG8(0x0215), 0x00 },
	{ CCI_REG8(0x3fe2), 0x00 },
	{ CCI_REG8(0x3fe3), 0x70 },
	{ CCI_REG8(0x3fe4), 0x01 },
	{ CCI_REG8(0x3fe5), 0x00 },
	/* PDAF: type and phase data output (0x3e3b) */
	{ CCI_REG8(0x3e20), 0x02 },
	{ CCI_REG8(0x3e3b), 0x00 },
	{ CCI_REG8(0x4034), 0x01 },
	{ CCI_REG8(0x4035), 0xf0 },
};

struct imx582_mode {
	u32 width;
	u32 height;
	/* Area of the pixel array the output covers, before any flip */
	struct v4l2_rect crop;
	u64 pixel_rate;
	u32 line_length;		/* LINE_LENGTH_PCK */
	u32 frame_length_min;		/* FRM_LENGTH_LINES of the table */
	u32 frame_length_def;
	u32 exposure_def;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct imx582_mode imx582_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = 160,
			.top = 832,
			.width = 7680,
			.height = 4320,
		},
		.pixel_rate = 734400000,
		.line_length = 2912,
		.frame_length_min = 2100,
		/* 30 fps, as the vendor's set_max_framerate() computes it */
		.frame_length_def = 8406,
		.exposure_def = 2052,
		.regs = imx582_1920x1080_regs,
		.num_regs = ARRAY_SIZE(imx582_1920x1080_regs),
	},
	{
		.width = 4000,
		.height = 3000,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 8000,
			.height = 6000,
		},
		.pixel_rate = 864000000,
		.line_length = 7872,
		.frame_length_min = 3658,
		/*
		 * 15 fps by default: libcamera's software ISP does not keep up
		 * with 12 Mpixel frames at 30 fps on the Redmi K50.
		 */
		.frame_length_def = 7316,
		.exposure_def = 3610,
		.regs = imx582_4000x3000_regs,
		.num_regs = ARRAY_SIZE(imx582_4000x3000_regs),
	},
};

struct imx582 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	/* In imx582_supply_descs order; NULL for an absent optional supply */
	struct regulator *supplies[IMX582_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	/* Cluster: hflip and vflip must stay adjacent. */
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/* Protected by the state lock */
	const struct imx582_mode *mode;	/* mode of the active format */
	bool switching_mode;		/* the controls only store values */
};

static inline struct imx582 *to_imx582(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx582, sd);
}

static u32 imx582_mbus_code(bool hflip, bool vflip)
{
	return imx582_mbus_codes[(vflip ? 2 : 0) | (hflip ? 1 : 0)];
}

/* Bayer order of the applied flips; a tried or failed value is not applied. */
static u32 imx582_cur_mbus_code(struct imx582 *imx582)
{
	return imx582_mbus_code(imx582->hflip->cur.val, imx582->vflip->cur.val);
}

static void imx582_set_active_code(struct imx582 *imx582, u32 code)
{
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_get_locked_active_state(&imx582->sd);
	v4l2_subdev_state_get_format(state, 0)->code = code;
}

static void imx582_fill_format(struct imx582 *imx582,
			       const struct imx582_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx582_cur_mbus_code(imx582);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/*
 * The exposure ends at least 48 lines before the frame does. The default is
 * the mode's, kept within the limit.
 */
static int imx582_update_exposure_limit(struct imx582 *imx582, s32 vblank)
{
	const struct imx582_mode *mode = imx582->mode;
	s64 max = (s64)mode->height + vblank - IMX582_EXPOSURE_MARGIN;

	return __v4l2_ctrl_modify_range(imx582->exposure, IMX582_EXPOSURE_MIN,
					max, IMX582_EXPOSURE_STEP,
					min_t(s64, mode->exposure_def, max));
}

/* Vendor gain2reg(): code = 1024 - 1024 * 1024 / gain, within the vendor limits */
static u32 imx582_gain_to_code(u32 gain)
{
	u32 code = IMX582_GAIN_UNIT - IMX582_GAIN_UNIT * IMX582_GAIN_UNIT / gain;

	return clamp(code, IMX582_ANA_GAIN_CODE_MIN, IMX582_ANA_GAIN_CODE_MAX);
}

/*
 * As in the vendor driver, frame length, exposure and gain change under the
 * group hold, so that the sensor applies each update at one frame boundary.
 * The frame length auto extension set at stream start covers an exposure
 * update that lands a frame before its VBLANK update.
 */
static int imx582_write_held(struct imx582 *imx582, struct v4l2_ctrl *ctrl)
{
	struct regmap *regmap = imx582->regmap;
	int ret = 0, hold_ret;

	cci_write(regmap, IMX582_REG_GROUP_HOLD, 1, &ret);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		cci_write(regmap, IMX582_REG_FRM_LENGTH_LINES,
			  imx582->mode->height + ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(regmap, IMX582_REG_COARSE_INTEG_TIME, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(regmap, IMX582_REG_ANA_GAIN_GLOBAL,
			  imx582_gain_to_code(ctrl->val), &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		/* Per colour or, in the CCS layout, global in 0x020e: set all. */
		cci_write(regmap, IMX582_REG_DIG_GAIN_GR, ctrl->val, &ret);
		cci_write(regmap, IMX582_REG_DIG_GAIN_R, ctrl->val, &ret);
		cci_write(regmap, IMX582_REG_DIG_GAIN_B, ctrl->val, &ret);
		cci_write(regmap, IMX582_REG_DIG_GAIN_GB, ctrl->val, &ret);
		break;
	}

	/* Release the hold even after a failed write. */
	hold_ret = cci_write(regmap, IMX582_REG_GROUP_HOLD, 0, NULL);

	return ret ?: hold_ret;
}

static int imx582_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx582 *imx582 = container_of(ctrl->handler, struct imx582,
					     ctrls);
	u32 code = 0;
	u64 val;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = imx582_update_exposure_limit(imx582, ctrl->val);
		if (ret)
			goto err_vblank;
		break;
	case V4L2_CID_HFLIP:
		/* The flips change the Bayer order of the active format. */
		code = imx582_mbus_code(imx582->hflip->val, imx582->vflip->val);
		break;
	}

	/*
	 * Write only while powered, and not during a mode switch, which never
	 * happens while streaming: enable_streams writes the mode table and
	 * then all controls.
	 */
	if (imx582->switching_mode ||
	    pm_runtime_get_if_active(imx582->dev) <= 0) {
		if (code)
			imx582_set_active_code(imx582, code);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx582_write_held(imx582, ctrl);
		break;
	case V4L2_CID_HFLIP:
		/* Cluster master: also applies V4L2_CID_VFLIP. */
		val = (imx582->hflip->val ? IMX582_HFLIP : 0) |
		      (imx582->vflip->val ? IMX582_VFLIP : 0);
		ret = cci_update_bits(imx582->regmap, IMX582_REG_IMG_ORIENTATION,
				      IMX582_HFLIP | IMX582_VFLIP, val, NULL);
		if (!ret)
			imx582_set_active_code(imx582, code);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx582->regmap, IMX582_REG_TEST_PATTERN_MODE,
				ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(imx582->dev);

	if (ret && ctrl->id == V4L2_CID_VBLANK)
		goto err_vblank;

	return ret;

err_vblank:
	/*
	 * VBLANK keeps its value: back to its exposure limit. The exposure is
	 * within that limit, so this neither fails nor writes.
	 */
	imx582_update_exposure_limit(imx582, ctrl->cur.val);
	return ret;
}

static const struct v4l2_ctrl_ops imx582_ctrl_ops = {
	.s_ctrl = imx582_set_ctrl,
};

static int imx582_init_controls(struct imx582 *imx582)
{
	const struct imx582_mode *mode = imx582->mode;
	struct v4l2_ctrl_handler *hdl = &imx582->ctrls;
	const struct v4l2_ctrl_ops *ops = &imx582_ctrl_ops;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u32 hblank;
	int ret;

	ret = v4l2_fwnode_device_parse(imx582->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 12);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx582_link_freq_menu) - 1, 0,
				      imx582_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx582->pixel_rate = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE,
					       IMX582_PIXEL_RATE_MIN,
					       IMX582_PIXEL_RATE_MAX, 1,
					       mode->pixel_rate);

	hblank = mode->line_length - mode->width;
	imx582->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (imx582->hblank)
		imx582->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The handler setup in enable_streams writes the controls in creation
	 * order: frame length, then exposure, as the vendor driver does.
	 */
	imx582->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					   mode->frame_length_min - mode->height,
					   IMX582_FRAME_LENGTH_MAX - mode->height,
					   1, mode->frame_length_def - mode->height);

	imx582->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     IMX582_EXPOSURE_MIN,
					     mode->frame_length_def -
					     IMX582_EXPOSURE_MARGIN,
					     IMX582_EXPOSURE_STEP,
					     mode->exposure_def);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, IMX582_ANA_GAIN_MIN,
			  IMX582_ANA_GAIN_MAX, 1, IMX582_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, IMX582_DIG_GAIN_MIN,
			  IMX582_DIG_GAIN_MAX, 1, IMX582_DIG_GAIN_DEFAULT);

	imx582->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx582->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx582->hflip && imx582->vflip) {
		imx582->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		imx582->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		v4l2_ctrl_cluster(2, &imx582->hflip);
	}

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx582_test_pattern_menu) - 1,
				     0, 0, imx582_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx582->sd.ctrl_handler = hdl;

	return 0;
}

/* Move the controls to the limits and defaults of @mode. */
static int imx582_apply_mode(struct imx582 *imx582,
			     const struct imx582_mode *mode)
{
	s64 hblank = mode->line_length - mode->width;
	s64 vblank_def = mode->frame_length_def - mode->height;
	int ret;

	/* The VBLANK handler reads the height of the new mode. */
	imx582->mode = mode;

	ret = __v4l2_ctrl_s_ctrl_int64(imx582->pixel_rate, mode->pixel_rate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx582->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	/*
	 * VBLANK first: a value outside its new range is clamped, and the
	 * handler then lowers the exposure limit, and the exposure, with it.
	 */
	ret = __v4l2_ctrl_modify_range(imx582->vblank,
				       mode->frame_length_min - mode->height,
				       IMX582_FRAME_LENGTH_MAX - mode->height,
				       1, vblank_def);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx582->vblank, vblank_def);
	if (ret)
		return ret;

	/* The handler does not run when VBLANK keeps its value. */
	ret = imx582_update_exposure_limit(imx582, vblank_def);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(imx582->exposure, mode->exposure_def);
}

/*
 * Switch the controls to @mode, with the state lock held and while not
 * streaming. The controls only store their values: enable_streams writes the
 * mode table and then every control. On failure the controls return to the
 * previous mode, which the active format keeps.
 */
static int imx582_set_mode(struct imx582 *imx582,
			   const struct imx582_mode *mode)
{
	const struct imx582_mode *prev = imx582->mode;
	int ret;

	imx582->switching_mode = true;

	ret = imx582_apply_mode(imx582, mode);
	if (ret)
		imx582_apply_mode(imx582, prev);

	imx582->switching_mode = false;

	return ret;
}

static int imx582_write_init(struct imx582 *imx582)
{
	int ret = 0;

	cci_multi_reg_write(imx582->regmap, imx582_init_regs,
			    ARRAY_SIZE(imx582_init_regs), &ret);
	cci_multi_reg_write(imx582->regmap, imx582_iq_regs,
			    ARRAY_SIZE(imx582_iq_regs), &ret);
	/* The vendor enables the temperature sensor here. */
	cci_write(imx582->regmap, IMX582_REG_TEMP_SEN_CTL,
		  IMX582_TEMP_SEN_EN, &ret);

	return ret;
}

static int imx582_write_stream(struct imx582 *imx582, bool on)
{
	int ret = 0;

	if (on) {
		/* As the vendor: frame length auto extension, normal transition */
		cci_write(imx582->regmap, IMX582_REG_FRM_LENGTH_CTL,
			  IMX582_FRM_LENGTH_AUTO, &ret);
		cci_write(imx582->regmap, IMX582_REG_MODE_TRANSITION, 0, &ret);
	}

	cci_write(imx582->regmap, IMX582_REG_MODE_SELECT,
		  on ? IMX582_MODE_STREAMING : IMX582_MODE_STANDBY, &ret);

	return ret;
}

/*
 * Stop the transmitter. If the standby write fails, hold the sensor in reset
 * instead; it stays unusable until runtime suspend powers it down and the
 * next runtime resume initialises it again.
 */
static int imx582_stop(struct imx582 *imx582)
{
	int ret;

	ret = imx582_write_stream(imx582, false);
	if (!ret || !imx582->reset_gpio)
		return ret;

	dev_err(imx582->dev, "standby failed (%d), holding the sensor in reset\n",
		ret);
	ret = gpiod_set_value_cansleep(imx582->reset_gpio, 1);
	usleep_range(3000, 4000);

	return ret;
}

static int imx582_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx582 *imx582 = to_imx582(sd);
	const struct imx582_mode *mode = imx582->mode;
	int ret;

	/* Normally the receiver's runtime PM link has resumed the sensor. */
	ret = pm_runtime_resume_and_get(imx582->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(imx582->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_rpm_put;

	/* Frame length, exposure, gains and flips of the first frame */
	ret = __v4l2_ctrl_handler_setup(&imx582->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = imx582_write_stream(imx582, true);
	if (ret) {
		/* A failed write may still have started the transmitter. */
		imx582_stop(imx582);
		goto err_rpm_put;
	}

	__v4l2_ctrl_grab(imx582->hflip, true);
	__v4l2_ctrl_grab(imx582->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx582->dev);
	return ret;
}

static int imx582_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx582 *imx582 = to_imx582(sd);
	int ret;

	ret = imx582_stop(imx582);
	if (ret)
		dev_err(imx582->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(imx582->hflip, false);
	__v4l2_ctrl_grab(imx582->vflip, false);

	pm_runtime_put_autosuspend(imx582->dev);

	/*
	 * The reference is dropped: report success so that the stream is
	 * marked disabled, as the core does for s_stream failures.
	 */
	return 0;
}

static int imx582_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = imx582_cur_mbus_code(to_imx582(sd));

	return 0;
}

static int imx582_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx582_mode *mode;

	if (fse->index >= ARRAY_SIZE(imx582_modes) ||
	    fse->code != imx582_cur_mbus_code(to_imx582(sd)))
		return -EINVAL;

	mode = &imx582_modes[fse->index];
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static int imx582_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx582 *imx582 = to_imx582(sd);
	const struct imx582_mode *mode;
	int ret;

	mode = v4l2_find_nearest_size(imx582_modes, ARRAY_SIZE(imx582_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	imx582_fill_format(imx582, mode, &fmt->format);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (v4l2_subdev_is_streaming(sd))
			return -EBUSY;

		/* Also for the same mode: VBLANK and exposure to defaults */
		ret = imx582_set_mode(imx582, mode);
		if (ret)
			return ret;
	}

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	return 0;
}

static int imx582_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r = imx582_native_rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx582_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, 0)->code;
	v4l2_subdev_unlock_state(state);

	/* Image data only: the phase data output is off in every mode. */
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].stream = 0;
	fd->entry[0].pixelcode = code;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW10;

	return 0;
}

static int imx582_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = IMX582_LINK_FREQ;
	config->bus.mipi_csi2.num_data_lanes = IMX582_DATA_LANES;

	return 0;
}

static int imx582_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct imx582_mode *mode = &imx582_modes[0];

	imx582_fill_format(to_imx582(sd), mode,
			   v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	return 0;
}

static const struct v4l2_subdev_video_ops imx582_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx582_pad_ops = {
	.enum_mbus_code = imx582_enum_mbus_code,
	.enum_frame_size = imx582_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx582_set_fmt,
	.get_selection = imx582_get_selection,
	.get_frame_desc = imx582_get_frame_desc,
	.get_mbus_config = imx582_get_mbus_config,
	.enable_streams = imx582_enable_streams,
	.disable_streams = imx582_disable_streams,
};

static const struct v4l2_subdev_ops imx582_subdev_ops = {
	.video = &imx582_video_ops,
	.pad = &imx582_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx582_internal_ops = {
	.init_state = imx582_init_state,
};

static void imx582_sleep_us(unsigned int us)
{
	if (us)
		usleep_range(us, us + 1000);
}

/* Switch off the first @count supplies in reverse order. */
static void imx582_supplies_off(struct imx582 *imx582, unsigned int count)
{
	while (count--) {
		if (imx582->supplies[count])
			regulator_disable(imx582->supplies[count]);
		imx582_sleep_us(imx582_supply_descs[count].delay_us);
	}
}

/* Order and delays of the vendor power sequence */
static int imx582_power_on(struct imx582 *imx582)
{
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(imx582->reset_gpio, 1);

	/* One at a time, with the vendor delay after each supply. */
	for (i = 0; i < IMX582_NUM_SUPPLIES; i++) {
		if (imx582->supplies[i]) {
			ret = regulator_enable(imx582->supplies[i]);
			if (ret)
				goto err_supplies;
		}
		imx582_sleep_us(imx582_supply_descs[i].delay_us);
	}

	ret = clk_prepare_enable(imx582->xclk);
	if (ret)
		goto err_supplies;

	/* Connect MCLK to the pad; a no-op without pinctrl states. */
	ret = pinctrl_pm_select_default_state(imx582->dev);
	if (ret)
		goto err_pins;
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(imx582->reset_gpio, 0);
	usleep_range(3000, 4000);

	return 0;

err_pins:
	/* Undo a partially applied default state. */
	pinctrl_pm_select_sleep_state(imx582->dev);
	clk_disable_unprepare(imx582->xclk);
err_supplies:
	imx582_supplies_off(imx582, i);
	return ret;
}

static void imx582_power_off(struct imx582 *imx582)
{
	gpiod_set_value_cansleep(imx582->reset_gpio, 1);
	usleep_range(3000, 4000);

	/* Park the MCLK pad as a low GPIO before gating the clock. */
	pinctrl_pm_select_sleep_state(imx582->dev);
	usleep_range(1000, 2000);
	clk_disable_unprepare(imx582->xclk);

	imx582_supplies_off(imx582, IMX582_NUM_SUPPLIES);
}

static int imx582_runtime_resume(struct device *dev)
{
	struct imx582 *imx582 = to_imx582(dev_get_drvdata(dev));
	int ret;

	ret = imx582_power_on(imx582);
	if (ret)
		goto err;

	ret = imx582_write_init(imx582);
	if (ret) {
		imx582_power_off(imx582);
		goto err;
	}

	return 0;

err:
	dev_err(dev, "power-up failed: %d\n", ret);
	/*
	 * Any other code sets the runtime PM error state, which the consumers
	 * linked to the sensor for runtime PM inherit: no stream could start
	 * again until all of their drivers are rebound.
	 */
	return -EAGAIN;
}

static int imx582_runtime_suspend(struct device *dev)
{
	imx582_power_off(to_imx582(dev_get_drvdata(dev)));

	return 0;
}

static int imx582_identify(struct imx582 *imx582)
{
	u64 id;
	int ret;

	ret = cci_read(imx582->regmap, IMX582_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx582->dev, ret, "failed to read chip id\n");

	if (id != IMX582_CHIP_ID)
		return dev_err_probe(imx582->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx\n", id);

	return 0;
}

static int imx582_parse_fwnode(struct imx582 *imx582)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	/* Connected or not: the sensor can be identified on its own. */
	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(imx582->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT |
					     FWNODE_GRAPH_DEVICE_DISABLED);
	if (!ep)
		return dev_err_probe(imx582->dev, -ENXIO, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(imx582->dev, ret, "invalid endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX582_DATA_LANES) {
		ret = dev_err_probe(imx582->dev, -EINVAL,
				    "only four data lanes are supported\n");
		goto out;
	}

	/* The link frequency the modes need must be allowed by the board. */
	ret = v4l2_link_freq_to_bitmap(imx582->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx582_link_freq_menu,
				       ARRAY_SIZE(imx582_link_freq_menu),
				       &link_freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int imx582_get_supplies(struct imx582 *imx582)
{
	struct regulator *reg;
	unsigned int i;

	for (i = 0; i < IMX582_NUM_SUPPLIES; i++) {
		const struct imx582_supply_desc *desc = &imx582_supply_descs[i];

		if (desc->optional) {
			reg = devm_regulator_get_optional(imx582->dev,
							  desc->name);
			if (reg == ERR_PTR(-ENODEV))
				reg = NULL;
		} else {
			reg = devm_regulator_get(imx582->dev, desc->name);
		}
		if (IS_ERR(reg))
			return dev_err_probe(imx582->dev, PTR_ERR(reg),
					     "failed to get %s\n", desc->name);

		imx582->supplies[i] = reg;
	}

	return 0;
}

static void imx582_disable_runtime_pm(struct imx582 *imx582)
{
	pm_runtime_disable(imx582->dev);
	if (!pm_runtime_status_suspended(imx582->dev)) {
		imx582_power_off(imx582);
		pm_runtime_set_suspended(imx582->dev);
	}
	pm_runtime_dont_use_autosuspend(imx582->dev);
}

static int imx582_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx582 *imx582;
	unsigned long rate;
	unsigned int i;
	int ret;

	imx582 = devm_kzalloc(dev, sizeof(*imx582), GFP_KERNEL);
	if (!imx582)
		return -ENOMEM;

	imx582->dev = dev;
	v4l2_i2c_subdev_init(&imx582->sd, client, &imx582_subdev_ops);

	ret = imx582_parse_fwnode(imx582);
	if (ret)
		return ret;

	imx582->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx582->regmap))
		return dev_err_probe(dev, PTR_ERR(imx582->regmap),
				     "failed to init CCI\n");

	/* The board selects a 24 MHz parent; the rate is checked, never set. */
	imx582->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx582->xclk))
		return dev_err_probe(dev, PTR_ERR(imx582->xclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(imx582->xclk);
	if (rate != IMX582_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK must be 24 MHz, got %lu Hz\n", rate);

	ret = imx582_get_supplies(imx582);
	if (ret)
		return ret;

	/* Asserted (XCLR low) whenever the sensor is not powered. */
	imx582->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx582->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx582->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Logged here, after the last possible deferral, so only once */
	for (i = 0; i < IMX582_NUM_SUPPLIES; i++) {
		if (!imx582->supplies[i])
			dev_info(dev, "no %s supply, powering up without it\n",
				 imx582_supply_descs[i].name);
	}

	/* Identify with the sensor powered only for the chip ID read. */
	ret = imx582_power_on(imx582);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = imx582_identify(imx582);
	imx582_power_off(imx582);
	if (ret)
		return ret;

	imx582->mode = &imx582_modes[0];
	ret = imx582_init_controls(imx582);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	imx582->sd.state_lock = imx582->ctrls.lock;
	imx582->sd.internal_ops = &imx582_internal_ops;
	imx582->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx582->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx582->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx582->sd.entity, 1, &imx582->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&imx582->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev\n");
		goto err_entity;
	}

	/* Suspended until a user resumes it; resume writes the init tables. */
	pm_runtime_set_autosuspend_delay(dev, IMX582_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx582->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	return 0;

err_pm:
	imx582_disable_runtime_pm(imx582);
	v4l2_subdev_cleanup(&imx582->sd);
err_entity:
	media_entity_cleanup(&imx582->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx582->ctrls);
	return ret;
}

static void imx582_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx582 *imx582 = to_imx582(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx582->ctrls);

	imx582_disable_runtime_pm(imx582);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx582_pm_ops, imx582_runtime_suspend,
				 imx582_runtime_resume, NULL);

static const struct of_device_id imx582_of_match[] = {
	{ .compatible = "sony,imx582" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx582_of_match);

static struct i2c_driver imx582_i2c_driver = {
	.driver = {
		.name = "imx582",
		.of_match_table = imx582_of_match,
		.pm = pm_ptr(&imx582_pm_ops),
		/* Unbinding would remove the sensor under a running stream. */
		.suppress_bind_attrs = true,
	},
	.probe = imx582_probe,
	.remove = imx582_remove,
};
module_i2c_driver(imx582_i2c_driver);

MODULE_DESCRIPTION("Sony IMX582 image sensor driver");
MODULE_LICENSE("GPL");
