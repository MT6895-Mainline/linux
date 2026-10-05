// SPDX-License-Identifier: GPL-2.0-only
/*
 * GalaxyCore GC02M1 image sensor driver
 *
 * Copyright (c) 2019 MediaTek Inc.
 *
 * The init sequence, the analogue gain table and the gain split are taken from
 * the MediaTek imgsensor driver of the Xiaomi Redmi K50 macro camera,
 * MiCode/Xiaomi_Kernel_OpenSource commit 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/misc/mediatek/imgsensor/src-v4l2/common/rubensgc02m1_mipi_raw/.
 * The V4L2 structure follows gc08a3.c (Copyright 2024 MediaTek) and s5kjn1.c
 * (Copyright (c) 2025 Linaro Ltd).
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

/* 0xfe selects the register page; the chip ID is read on page 0 after reset. */
#define GC02M1_REG_PAGE_SELECT		CCI_REG8(0xfe)
#define GC02M1_PAGE_0			0x00
#define GC02M1_PAGE_1			0x01
#define GC02M1_REG_CHIP_ID_H		CCI_REG8(0xf0)
#define GC02M1_REG_CHIP_ID_L		CCI_REG8(0xf1)
#define GC02M1_CHIP_ID			0x02e0

/* Page 0 */
#define GC02M1_REG_EXPOSURE_H		CCI_REG8(0x03)	/* [5:0], double-buffered */
#define GC02M1_REG_EXPOSURE_L		CCI_REG8(0x04)
#define GC02M1_REG_MIRROR_FLIP		CCI_REG8(0x17)
#define GC02M1_MIRROR_FLIP_BASE		0x80
#define GC02M1_HFLIP			BIT(0)
#define GC02M1_VFLIP			BIT(1)
#define GC02M1_REG_STREAM		CCI_REG8(0x3e)
#define GC02M1_STREAM_ON		0x90
#define GC02M1_STREAM_OFF		0x00
#define GC02M1_REG_FRAME_LENGTH_H	CCI_REG8(0x41)	/* [5:0], reads back at once */
#define GC02M1_REG_FRAME_LENGTH_L	CCI_REG8(0x42)
#define GC02M1_REG_DGAIN_H		CCI_REG8(0xb1)	/* [4:0], double-buffered */
#define GC02M1_REG_DGAIN_L		CCI_REG8(0xb2)
#define GC02M1_REG_AGAIN_INDEX		CCI_REG8(0xb6)	/* [3:0], reads back at once */

/* Page 1 */
#define GC02M1_REG_TEST_PATTERN		CCI_REG8(0x8c)
#define GC02M1_TEST_PATTERN_OFF		0x10
#define GC02M1_TEST_PATTERN_BARS	0x11

#define GC02M1_XCLK_FREQ		(24 * HZ_PER_MHZ)
#define GC02M1_LINK_FREQ		(336 * HZ_PER_MHZ)
#define GC02M1_PIXEL_RATE		(84 * HZ_PER_MHZ)
#define GC02M1_DATA_LANES		1

#define GC02M1_WIDTH			1600
#define GC02M1_HEIGHT			1200
#define GC02M1_LINE_LENGTH		2192
#define GC02M1_HBLANK			(GC02M1_LINE_LENGTH - GC02M1_WIDTH)
#define GC02M1_FRAME_LENGTH_MIN		1268
#define GC02M1_FRAME_LENGTH_MAX		0x3fff
#define GC02M1_VBLANK_MIN		(GC02M1_FRAME_LENGTH_MIN - GC02M1_HEIGHT)
#define GC02M1_VBLANK_MAX		(GC02M1_FRAME_LENGTH_MAX - GC02M1_HEIGHT)

#define GC02M1_EXPOSURE_MIN		4
#define GC02M1_EXPOSURE_MARGIN		16
#define GC02M1_EXPOSURE_DEFAULT		1149

/*
 * V4L2_CID_ANALOGUE_GAIN is the total gain, 1024 = 1.0, as in the vendor
 * driver: the largest analogue step not above it goes to 0xb6 and the
 * remainder to the sensor digital gain, which stays within 0x400..0x5ff.
 */
#define GC02M1_GAIN_UNIT		1024
#define GC02M1_GAIN_MIN			GC02M1_GAIN_UNIT
#define GC02M1_GAIN_MAX			(12 * GC02M1_GAIN_UNIT)

#define GC02M1_AUTOSUSPEND_DELAY_MS	1000

static const s64 gc02m1_link_freq_menu[] = {
	GC02M1_LINK_FREQ,
};

static const char * const gc02m1_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

/* Indexed by (vflip << 1) | hflip; the flipped orders are not verified. */
static const u32 gc02m1_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* Gain of each 0xb6 analogue step, 1024 = 1.0 (vendor GC02M1SUNNY_AGC_Param) */
static const u16 gc02m1_again_table[] = {
	1024, 1536, 2035, 2519, 3165, 3626, 4147, 4593,
	5095, 5697, 6270, 6714, 7210, 7686, 8214, 10337,
};

static const struct v4l2_rect gc02m1_crop = {
	.left = 0,
	.top = 0,
	.width = GC02M1_WIDTH,
	.height = GC02M1_HEIGHT,
};

/*
 * 1600x1200 RAW10 on one lane from a 24 MHz MCLK: line length 2192 at 84 MHz,
 * frame length 1268, exposure 1149, gain 1.0. Written verbatim after every
 * power-up; the order and the repeated writes are significant. The table ends
 * on page 0 with streaming off.
 */
static const struct cci_reg_sequence gc02m1_init_regs[] = {
	/* system */
	{ CCI_REG8(0xfc), 0x01 },
	{ CCI_REG8(0xf4), 0x41 },
	{ CCI_REG8(0xf5), 0xc0 },
	{ CCI_REG8(0xf6), 0x44 },
	{ CCI_REG8(0xf8), 0x38 },
	{ CCI_REG8(0xf9), 0x82 },
	{ CCI_REG8(0xfa), 0x00 },
	{ CCI_REG8(0xfd), 0x80 },
	{ CCI_REG8(0xfc), 0x81 },
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x0b },
	{ CCI_REG8(0xf7), 0x01 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x80 },
	{ CCI_REG8(0xfc), 0x8e },
	/* CISCTL */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x87), 0x09 },
	{ CCI_REG8(0xee), 0x72 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x90 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x90), 0x00 },
	{ CCI_REG8(0x03), 0x04 },
	{ CCI_REG8(0x04), 0x7d },
	{ CCI_REG8(0x41), 0x04 },
	{ CCI_REG8(0x42), 0xf4 },
	{ CCI_REG8(0x05), 0x04 },
	{ CCI_REG8(0x06), 0x48 },
	{ CCI_REG8(0x07), 0x00 },
	{ CCI_REG8(0x08), 0x18 },
	{ CCI_REG8(0x9d), 0x18 },
	{ CCI_REG8(0x09), 0x00 },
	{ CCI_REG8(0x0a), 0x02 },
	{ CCI_REG8(0x0d), 0x04 },
	{ CCI_REG8(0x0e), 0xbc },
	{ CCI_REG8(0x17), 0x80 },
	{ CCI_REG8(0x19), 0x04 },
	{ CCI_REG8(0x24), 0x00 },
	{ CCI_REG8(0x56), 0x20 },
	{ CCI_REG8(0x5b), 0x00 },
	{ CCI_REG8(0x5e), 0x01 },
	/* analogue register width */
	{ CCI_REG8(0x21), 0x3c },
	{ CCI_REG8(0x44), 0x20 },
	{ CCI_REG8(0xcc), 0x01 },
	/* analogue mode */
	{ CCI_REG8(0x1a), 0x04 },
	{ CCI_REG8(0x1f), 0x11 },
	{ CCI_REG8(0x27), 0x30 },
	{ CCI_REG8(0x2b), 0x00 },
	{ CCI_REG8(0x33), 0x00 },
	{ CCI_REG8(0x53), 0x90 },
	{ CCI_REG8(0xe6), 0x50 },
	/* analogue voltage */
	{ CCI_REG8(0x39), 0x07 },
	{ CCI_REG8(0x43), 0x04 },
	{ CCI_REG8(0x46), 0x2a },
	{ CCI_REG8(0x7c), 0xa0 },
	{ CCI_REG8(0xd0), 0xbe },
	{ CCI_REG8(0xd1), 0x60 },
	{ CCI_REG8(0xd2), 0x40 },
	{ CCI_REG8(0xd3), 0xf3 },
	{ CCI_REG8(0xde), 0x1d },
	/* analogue current */
	{ CCI_REG8(0xcd), 0x05 },
	{ CCI_REG8(0xce), 0x6f },
	/* CISCTL reset */
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x88 },
	{ CCI_REG8(0xfe), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xfc), 0x8e },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xe0), 0x01 },
	{ CCI_REG8(0xfe), 0x00 },
	/* ISP */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x53), 0x44 },
	{ CCI_REG8(0x87), 0x53 },
	{ CCI_REG8(0x89), 0x03 },
	/* gain */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0xb0), 0x74 },
	{ CCI_REG8(0xb1), 0x04 },
	{ CCI_REG8(0xb2), 0x00 },
	{ CCI_REG8(0xb6), 0x00 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0xd8), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x2a },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x19 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0xd0 },
	{ CCI_REG8(0xc0), 0x2f },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x90 },
	{ CCI_REG8(0xc0), 0x39 },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x04 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x0f },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x1a },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x25 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x2c },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x32 },
	{ CCI_REG8(0xc0), 0xc0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x20 },
	{ CCI_REG8(0xc0), 0x38 },
	{ CCI_REG8(0xc0), 0xe0 },
	{ CCI_REG8(0xc0), 0x01 },
	{ CCI_REG8(0xc0), 0x60 },
	{ CCI_REG8(0xc0), 0x3c },
	{ CCI_REG8(0xc0), 0x00 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0xa0 },
	{ CCI_REG8(0xc0), 0x40 },
	{ CCI_REG8(0xc0), 0x80 },
	{ CCI_REG8(0xc0), 0x02 },
	{ CCI_REG8(0xc0), 0x18 },
	{ CCI_REG8(0xc0), 0x5c },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x9f), 0x10 },
	/* BLK */
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x26), 0x20 },
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x40), 0x22 },
	{ CCI_REG8(0x46), 0x7f },
	{ CCI_REG8(0x49), 0x0f },
	{ CCI_REG8(0x4a), 0xf0 },
	{ CCI_REG8(0xfe), 0x04 },
	{ CCI_REG8(0x14), 0x80 },
	{ CCI_REG8(0x15), 0x80 },
	{ CCI_REG8(0x16), 0x80 },
	{ CCI_REG8(0x17), 0x80 },
	/* anti-blooming */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x41), 0x20 },
	{ CCI_REG8(0x4c), 0x00 },
	{ CCI_REG8(0x4d), 0x0c },
	{ CCI_REG8(0x44), 0x08 },
	{ CCI_REG8(0x48), 0x03 },
	/* window 1600x1200 */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x90), 0x01 },
	{ CCI_REG8(0x91), 0x00 },
	{ CCI_REG8(0x92), 0x06 },
	{ CCI_REG8(0x93), 0x00 },
	{ CCI_REG8(0x94), 0x06 },
	{ CCI_REG8(0x95), 0x04 },
	{ CCI_REG8(0x96), 0xb0 },
	{ CCI_REG8(0x97), 0x06 },
	{ CCI_REG8(0x98), 0x40 },
	/* mipi */
	{ CCI_REG8(0xfe), 0x03 },
	{ CCI_REG8(0x01), 0x23 },
	{ CCI_REG8(0x03), 0xce },
	{ CCI_REG8(0x04), 0x48 },
	{ CCI_REG8(0x15), 0x00 },
	{ CCI_REG8(0x21), 0x10 },
	{ CCI_REG8(0x22), 0x05 },
	{ CCI_REG8(0x23), 0x20 },
	{ CCI_REG8(0x25), 0x20 },
	{ CCI_REG8(0x26), 0x08 },
	{ CCI_REG8(0x29), 0x06 },
	{ CCI_REG8(0x2a), 0x0a },
	{ CCI_REG8(0x2b), 0x08 },
	/* out */
	{ CCI_REG8(0xfe), 0x01 },
	{ CCI_REG8(0x8c), 0x10 },
	{ CCI_REG8(0xfe), 0x00 },
	{ CCI_REG8(0x3e), 0x00 },
};

struct gc02m1 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	struct regulator *dovdd;
	struct regulator *avdd;
	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	/* Cluster: hflip and vflip must stay adjacent. */
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct gc02m1 *to_gc02m1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gc02m1, sd);
}

/*
 * The CCI regmap has no locking and the page register is global state: all
 * accesses run under the control handler lock (also the subdev state lock),
 * in the runtime PM callbacks or in probe.
 */
static int gc02m1_select_page(struct gc02m1 *gc02m1, u8 page, int *err)
{
	return cci_write(gc02m1->regmap, GC02M1_REG_PAGE_SELECT, page, err);
}

static u32 gc02m1_mbus_code(bool hflip, bool vflip)
{
	return gc02m1_mbus_codes[(vflip ? 2 : 0) | (hflip ? 1 : 0)];
}

/* Bayer order of the applied flips; a tried or failed value is not applied. */
static u32 gc02m1_cur_mbus_code(struct gc02m1 *gc02m1)
{
	return gc02m1_mbus_code(gc02m1->hflip->cur.val, gc02m1->vflip->cur.val);
}

static void gc02m1_set_active_code(struct gc02m1 *gc02m1, u32 code)
{
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_get_locked_active_state(&gc02m1->sd);
	v4l2_subdev_state_get_format(state, 0)->code = code;
}

static void gc02m1_fill_format(struct gc02m1 *gc02m1,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = GC02M1_WIDTH;
	fmt->height = GC02M1_HEIGHT;
	fmt->code = gc02m1_cur_mbus_code(gc02m1);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static void gc02m1_write_gain(struct gc02m1 *gc02m1, u32 gain, int *err)
{
	unsigned int idx = ARRAY_SIZE(gc02m1_again_table) - 1;
	u32 dgain;

	while (idx && gain < gc02m1_again_table[idx])
		idx--;

	/* Below the next analogue step, so within 0x400..0x5ff */
	dgain = gain * GC02M1_GAIN_UNIT / gc02m1_again_table[idx];

	gc02m1_select_page(gc02m1, GC02M1_PAGE_0, err);
	cci_write(gc02m1->regmap, GC02M1_REG_AGAIN_INDEX, idx, err);
	cci_write(gc02m1->regmap, GC02M1_REG_DGAIN_H, (dgain >> 8) & 0x1f, err);
	cci_write(gc02m1->regmap, GC02M1_REG_DGAIN_L, dgain & 0xff, err);
}

static int gc02m1_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc02m1 *gc02m1 = container_of(ctrl->handler, struct gc02m1,
					     ctrls);
	u32 code = 0;
	int ret = 0;
	u32 val;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* The exposure must end 16 lines before the frame does. */
		ret = __v4l2_ctrl_modify_range(gc02m1->exposure,
					       gc02m1->exposure->minimum,
					       GC02M1_HEIGHT + ctrl->val -
					       GC02M1_EXPOSURE_MARGIN,
					       gc02m1->exposure->step,
					       gc02m1->exposure->default_value);
		if (ret)
			return ret;
		break;
	case V4L2_CID_HFLIP:
		/* The flips change the Bayer order of the active format. */
		code = gc02m1_mbus_code(gc02m1->hflip->val, gc02m1->vflip->val);
		break;
	}

	/* Write only while powered; enable_streams applies all controls. */
	if (pm_runtime_get_if_active(gc02m1->dev) <= 0) {
		if (code)
			gc02m1_set_active_code(gc02m1, code);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_EXPOSURE_H,
			  (ctrl->val >> 8) & 0x3f, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_EXPOSURE_L,
			  ctrl->val & 0xff, &ret);
		break;
	case V4L2_CID_VBLANK:
		/*
		 * Write the byte that moves the frame length towards the new
		 * value first: in between, the length stays at least the
		 * smaller of the old and new values, above the active lines.
		 */
		val = GC02M1_HEIGHT + ctrl->val;
		gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);
		if (ctrl->val < ctrl->cur.val)
			cci_write(gc02m1->regmap, GC02M1_REG_FRAME_LENGTH_L,
				  val & 0xff, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_FRAME_LENGTH_H,
			  (val >> 8) & 0x3f, &ret);
		if (ctrl->val >= ctrl->cur.val)
			cci_write(gc02m1->regmap, GC02M1_REG_FRAME_LENGTH_L,
				  val & 0xff, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc02m1_write_gain(gc02m1, ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
		/* Cluster master: also applies V4L2_CID_VFLIP. */
		val = GC02M1_MIRROR_FLIP_BASE |
		      (gc02m1->hflip->val ? GC02M1_HFLIP : 0) |
		      (gc02m1->vflip->val ? GC02M1_VFLIP : 0);
		gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_MIRROR_FLIP, val, &ret);
		if (!ret)
			gc02m1_set_active_code(gc02m1, code);
		break;
	case V4L2_CID_TEST_PATTERN:
		gc02m1_select_page(gc02m1, GC02M1_PAGE_1, &ret);
		cci_write(gc02m1->regmap, GC02M1_REG_TEST_PATTERN,
			  ctrl->val ? GC02M1_TEST_PATTERN_BARS :
				      GC02M1_TEST_PATTERN_OFF, &ret);
		gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(gc02m1->dev);

	return ret;
}

static const struct v4l2_ctrl_ops gc02m1_ctrl_ops = {
	.s_ctrl = gc02m1_set_ctrl,
};

static int gc02m1_init_controls(struct gc02m1 *gc02m1)
{
	struct v4l2_ctrl_handler *hdl = &gc02m1->ctrls;
	const struct v4l2_ctrl_ops *ops = &gc02m1_ctrl_ops;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(gc02m1->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 11);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(gc02m1_link_freq_menu) - 1, 0,
				      gc02m1_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, GC02M1_PIXEL_RATE,
			  GC02M1_PIXEL_RATE, 1, GC02M1_PIXEL_RATE);

	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, GC02M1_HBLANK,
				 GC02M1_HBLANK, 1, GC02M1_HBLANK);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The handler setup in enable_streams writes the controls in creation
	 * order: exposure, frame length, then gain, as the vendor driver does.
	 */
	gc02m1->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     GC02M1_EXPOSURE_MIN,
					     GC02M1_FRAME_LENGTH_MIN -
					     GC02M1_EXPOSURE_MARGIN, 1,
					     GC02M1_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, GC02M1_VBLANK_MIN,
			  GC02M1_VBLANK_MAX, 1, GC02M1_VBLANK_MIN);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, GC02M1_GAIN_MIN,
			  GC02M1_GAIN_MAX, 1, GC02M1_GAIN_MIN);

	gc02m1->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	gc02m1->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (gc02m1->hflip && gc02m1->vflip) {
		gc02m1->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		gc02m1->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		v4l2_ctrl_cluster(2, &gc02m1->hflip);
	}

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(gc02m1_test_pattern_menu) - 1,
				     0, 0, gc02m1_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	gc02m1->sd.ctrl_handler = hdl;

	return 0;
}

static int gc02m1_write_stream(struct gc02m1 *gc02m1, bool on)
{
	int ret = 0;

	gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);
	cci_write(gc02m1->regmap, GC02M1_REG_STREAM,
		  on ? GC02M1_STREAM_ON : GC02M1_STREAM_OFF, &ret);
	gc02m1_select_page(gc02m1, GC02M1_PAGE_0, &ret);

	return ret;
}

/*
 * Stop the transmitter. If the stream-off write fails, hold the sensor in
 * reset instead; it stays unusable until runtime suspend powers it down and
 * the next runtime resume initialises it again.
 */
static int gc02m1_stop(struct gc02m1 *gc02m1)
{
	int ret;

	ret = gc02m1_write_stream(gc02m1, false);
	if (!ret || !gc02m1->reset_gpio)
		return ret;

	dev_err(gc02m1->dev, "stream off failed (%d), holding the sensor in reset\n",
		ret);
	ret = gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	usleep_range(5000, 6000);

	return ret;
}

static int gc02m1_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	/* Normally the receiver's runtime PM link has resumed the sensor. */
	ret = pm_runtime_resume_and_get(gc02m1->dev);
	if (ret)
		return ret;

	/* Exposure and digital gain written before stream-on apply to frame 0. */
	ret = __v4l2_ctrl_handler_setup(&gc02m1->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = gc02m1_write_stream(gc02m1, true);
	if (ret) {
		/* A failed write may still have started the transmitter. */
		gc02m1_stop(gc02m1);
		goto err_rpm_put;
	}

	__v4l2_ctrl_grab(gc02m1->hflip, true);
	__v4l2_ctrl_grab(gc02m1->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(gc02m1->dev);
	return ret;
}

static int gc02m1_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct gc02m1 *gc02m1 = to_gc02m1(sd);
	int ret;

	ret = gc02m1_stop(gc02m1);
	if (ret)
		dev_err(gc02m1->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(gc02m1->hflip, false);
	__v4l2_ctrl_grab(gc02m1->vflip, false);

	pm_runtime_put_autosuspend(gc02m1->dev);

	/*
	 * The reference is dropped: report success so that the stream is
	 * marked disabled, as the core does for s_stream failures.
	 */
	return 0;
}

static int gc02m1_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = gc02m1_cur_mbus_code(to_gc02m1(sd));

	return 0;
}

static int gc02m1_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != gc02m1_cur_mbus_code(to_gc02m1(sd)))
		return -EINVAL;

	fse->min_width = GC02M1_WIDTH;
	fse->max_width = GC02M1_WIDTH;
	fse->min_height = GC02M1_HEIGHT;
	fse->max_height = GC02M1_HEIGHT;

	return 0;
}

static int gc02m1_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	/* Single mode: any request results in 1600x1200 RAW10. */
	gc02m1_fill_format(to_gc02m1(sd), &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int gc02m1_get_selection(struct v4l2_subdev *sd,
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
		sel->r = gc02m1_crop;
		return 0;
	default:
		return -EINVAL;
	}
}

static int gc02m1_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, 0)->code;
	v4l2_subdev_unlock_state(state);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].stream = 0;
	fd->entry[0].pixelcode = code;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW10;

	return 0;
}

static int gc02m1_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = GC02M1_LINK_FREQ;
	config->bus.mipi_csi2.num_data_lanes = GC02M1_DATA_LANES;

	return 0;
}

static int gc02m1_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	gc02m1_fill_format(to_gc02m1(sd), v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = gc02m1_crop;

	return 0;
}

static const struct v4l2_subdev_video_ops gc02m1_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gc02m1_pad_ops = {
	.enum_mbus_code = gc02m1_enum_mbus_code,
	.enum_frame_size = gc02m1_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = gc02m1_set_fmt,
	.get_selection = gc02m1_get_selection,
	.get_frame_desc = gc02m1_get_frame_desc,
	.get_mbus_config = gc02m1_get_mbus_config,
	.enable_streams = gc02m1_enable_streams,
	.disable_streams = gc02m1_disable_streams,
};

static const struct v4l2_subdev_ops gc02m1_subdev_ops = {
	.video = &gc02m1_video_ops,
	.pad = &gc02m1_pad_ops,
};

static const struct v4l2_subdev_internal_ops gc02m1_internal_ops = {
	.init_state = gc02m1_init_state,
};

/* Order and delays of the vendor power sequence */
static int gc02m1_power_on(struct gc02m1 *gc02m1)
{
	int ret;

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	usleep_range(1000, 2000);

	/* One at a time: DOVDD must be up before AVDD. */
	ret = regulator_enable(gc02m1->dovdd);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	ret = regulator_enable(gc02m1->avdd);
	if (ret)
		goto err_dovdd;
	usleep_range(2000, 3000);

	ret = clk_prepare_enable(gc02m1->xclk);
	if (ret)
		goto err_avdd;

	/* Connect MCLK to the pad; a no-op without pinctrl states. */
	ret = pinctrl_pm_select_default_state(gc02m1->dev);
	if (ret)
		goto err_pins;
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(gc02m1->reset_gpio, 0);
	usleep_range(5000, 6000);

	return 0;

err_pins:
	/* Undo a partially applied default state. */
	pinctrl_pm_select_sleep_state(gc02m1->dev);
	clk_disable_unprepare(gc02m1->xclk);
err_avdd:
	regulator_disable(gc02m1->avdd);
	usleep_range(2000, 3000);
err_dovdd:
	regulator_disable(gc02m1->dovdd);
	usleep_range(1000, 2000);
	return ret;
}

static void gc02m1_power_off(struct gc02m1 *gc02m1)
{
	gpiod_set_value_cansleep(gc02m1->reset_gpio, 1);
	usleep_range(5000, 6000);

	/* Park the MCLK pad as a low GPIO before gating the clock. */
	pinctrl_pm_select_sleep_state(gc02m1->dev);
	usleep_range(1000, 2000);
	clk_disable_unprepare(gc02m1->xclk);

	regulator_disable(gc02m1->avdd);
	usleep_range(2000, 3000);
	regulator_disable(gc02m1->dovdd);
	/* Minimum off time before the next power-up */
	usleep_range(1000, 2000);
}

static int gc02m1_runtime_resume(struct device *dev)
{
	struct gc02m1 *gc02m1 = to_gc02m1(dev_get_drvdata(dev));
	int ret;

	ret = gc02m1_power_on(gc02m1);
	if (ret)
		goto err;

	ret = cci_multi_reg_write(gc02m1->regmap, gc02m1_init_regs,
				  ARRAY_SIZE(gc02m1_init_regs), NULL);
	if (ret) {
		gc02m1_power_off(gc02m1);
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

static int gc02m1_runtime_suspend(struct device *dev)
{
	gc02m1_power_off(to_gc02m1(dev_get_drvdata(dev)));

	return 0;
}

static int gc02m1_identify(struct gc02m1 *gc02m1)
{
	u64 hi, lo;
	int ret = 0;

	/* Read right after reset; the ID registers need no page select. */
	cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID_H, &hi, &ret);
	cci_read(gc02m1->regmap, GC02M1_REG_CHIP_ID_L, &lo, &ret);
	if (ret)
		return dev_err_probe(gc02m1->dev, ret, "failed to read chip id\n");

	if (((hi << 8) | lo) != GC02M1_CHIP_ID)
		return dev_err_probe(gc02m1->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx\n",
				     (hi << 8) | lo);

	return 0;
}

static int gc02m1_parse_fwnode(struct gc02m1 *gc02m1)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(gc02m1->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return dev_err_probe(gc02m1->dev, -ENXIO, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(gc02m1->dev, ret, "invalid endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != GC02M1_DATA_LANES) {
		ret = dev_err_probe(gc02m1->dev, -EINVAL,
				    "only one data lane is supported\n");
		goto out;
	}

	/* The link frequency the mode needs must be allowed by the board. */
	ret = v4l2_link_freq_to_bitmap(gc02m1->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       gc02m1_link_freq_menu,
				       ARRAY_SIZE(gc02m1_link_freq_menu),
				       &link_freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static void gc02m1_disable_runtime_pm(struct gc02m1 *gc02m1)
{
	pm_runtime_disable(gc02m1->dev);
	if (!pm_runtime_status_suspended(gc02m1->dev)) {
		gc02m1_power_off(gc02m1);
		pm_runtime_set_suspended(gc02m1->dev);
	}
	pm_runtime_dont_use_autosuspend(gc02m1->dev);
}

static int gc02m1_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct gc02m1 *gc02m1;
	unsigned long rate;
	int ret;

	gc02m1 = devm_kzalloc(dev, sizeof(*gc02m1), GFP_KERNEL);
	if (!gc02m1)
		return -ENOMEM;

	gc02m1->dev = dev;
	v4l2_i2c_subdev_init(&gc02m1->sd, client, &gc02m1_subdev_ops);

	ret = gc02m1_parse_fwnode(gc02m1);
	if (ret)
		return ret;

	gc02m1->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(gc02m1->regmap))
		return dev_err_probe(dev, PTR_ERR(gc02m1->regmap),
				     "failed to init CCI\n");

	/* The board selects a 24 MHz parent; the rate is checked, never set. */
	gc02m1->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(gc02m1->xclk))
		return dev_err_probe(dev, PTR_ERR(gc02m1->xclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(gc02m1->xclk);
	if (rate != GC02M1_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK must be 24 MHz, got %lu Hz\n", rate);

	gc02m1->dovdd = devm_regulator_get(dev, "dovdd");
	if (IS_ERR(gc02m1->dovdd))
		return dev_err_probe(dev, PTR_ERR(gc02m1->dovdd),
				     "failed to get DOVDD\n");

	gc02m1->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(gc02m1->avdd))
		return dev_err_probe(dev, PTR_ERR(gc02m1->avdd),
				     "failed to get AVDD\n");

	/* Asserted (RESETB low) whenever the sensor is not powered. */
	gc02m1->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(gc02m1->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(gc02m1->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Identify with the sensor powered only for the chip ID read. */
	ret = gc02m1_power_on(gc02m1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = gc02m1_identify(gc02m1);
	gc02m1_power_off(gc02m1);
	if (ret)
		return ret;

	ret = gc02m1_init_controls(gc02m1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	gc02m1->sd.state_lock = gc02m1->ctrls.lock;
	gc02m1->sd.internal_ops = &gc02m1_internal_ops;
	gc02m1->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	gc02m1->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	gc02m1->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&gc02m1->sd.entity, 1, &gc02m1->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&gc02m1->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev\n");
		goto err_entity;
	}

	/* Suspended until a user resumes it; resume writes the init table. */
	pm_runtime_set_autosuspend_delay(dev, GC02M1_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&gc02m1->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	return 0;

err_pm:
	gc02m1_disable_runtime_pm(gc02m1);
	v4l2_subdev_cleanup(&gc02m1->sd);
err_entity:
	media_entity_cleanup(&gc02m1->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&gc02m1->ctrls);
	return ret;
}

static void gc02m1_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc02m1 *gc02m1 = to_gc02m1(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&gc02m1->ctrls);

	gc02m1_disable_runtime_pm(gc02m1);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gc02m1_pm_ops, gc02m1_runtime_suspend,
				 gc02m1_runtime_resume, NULL);

static const struct of_device_id gc02m1_of_match[] = {
	{ .compatible = "galaxycore,gc02m1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gc02m1_of_match);

static struct i2c_driver gc02m1_i2c_driver = {
	.driver = {
		.name = "gc02m1",
		.of_match_table = gc02m1_of_match,
		.pm = pm_ptr(&gc02m1_pm_ops),
		/* Unbinding would remove the sensor under a running stream. */
		.suppress_bind_attrs = true,
	},
	.probe = gc02m1_probe,
	.remove = gc02m1_remove,
};
module_i2c_driver(gc02m1_i2c_driver);

MODULE_DESCRIPTION("GalaxyCore GC02M1 image sensor driver");
MODULE_LICENSE("GPL");
