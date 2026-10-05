// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S5K4H7 image sensor driver
 *
 * Copyright (c) 2019 MediaTek Inc.
 *
 * The register tables, the power sequence, the exposure limits, the analogue
 * gain encoding and the use of the grouped parameter hold are taken from the
 * MediaTek imgsensor driver of the Xiaomi Redmi K50 ultra-wide camera,
 * MiCode/Xiaomi_Kernel_OpenSource commit
 * 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/misc/mediatek/imgsensor/src-v4l2/common/rubenss5k4h7_mipi_raw/.
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

/* Vendor register names where the vendor driver has one, MIPI CCS names otherwise */
#define S5K4H7_REG_SENSOR_ID_H		CCI_REG8(0x0000)
#define S5K4H7_REG_SENSOR_ID_L		CCI_REG8(0x0001)
#define S5K4H7_SENSOR_ID		0x487b
#define S5K4H7_REG_REVISION		CCI_REG8(0x0002)
#define S5K4H7_REG_PIXEL_ORDER		CCI_REG8(0x0006)
#define S5K4H7_REG_DATA_PEDESTAL	CCI_REG16(0x0008)
#define S5K4H7_REG_AGAIN_CAPABILITY	CCI_REG16(0x0080)
#define S5K4H7_REG_AGAIN_CODE_MIN	CCI_REG16(0x0084)
#define S5K4H7_REG_AGAIN_CODE_MAX	CCI_REG16(0x0086)
#define S5K4H7_REG_AGAIN_CODE_STEP	CCI_REG16(0x0088)
#define S5K4H7_REG_AGAIN_TYPE		CCI_REG16(0x008a)
#define S5K4H7_REG_AGAIN_M0		CCI_REG16(0x008c)
#define S5K4H7_REG_AGAIN_C0		CCI_REG16(0x008e)
#define S5K4H7_REG_AGAIN_M1		CCI_REG16(0x0090)
#define S5K4H7_REG_AGAIN_C1		CCI_REG16(0x0092)
#define S5K4H7_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K4H7_MODE_STANDBY		0x00
#define S5K4H7_MODE_STREAMING		0x01
#define S5K4H7_REG_IMAGE_ORIENTATION	CCI_REG8(0x0101)
#define S5K4H7_HFLIP			BIT(0)
#define S5K4H7_VFLIP			BIT(1)
/* Written as the vendor does: the low byte clears 0x0105 mask_corrupted_frames. */
#define S5K4H7_REG_GROUP_HOLD		CCI_REG16(0x0104)
#define S5K4H7_GROUP_HOLD_ON		0x0100
#define S5K4H7_GROUP_HOLD_OFF		0x0000
#define S5K4H7_REG_COARSE_INTEGRATION_TIME	CCI_REG16(0x0202)
#define S5K4H7_REG_AGAIN		CCI_REG16(0x0204)
#define S5K4H7_REG_FRAME_LENGTH_LINES	CCI_REG16(0x0340)
#define S5K4H7_REG_TEST_PATTERN_MODE	CCI_REG16(0x0600)

#define S5K4H7_XCLK_FREQ		(24 * HZ_PER_MHZ)
/* OP PLL: 24 MHz / 6 * 165 = 660 Mbit/s per lane */
#define S5K4H7_LINK_FREQ		(330 * HZ_PER_MHZ)
/* VT PLL: 24 MHz / 6 * 140 / 4 = 140 MHz, two pixels per clock */
#define S5K4H7_PIXEL_RATE		(280 * HZ_PER_MHZ)
#define S5K4H7_DATA_LANES		4

#define S5K4H7_LINE_LENGTH		3688
#define S5K4H7_FRAME_LENGTH_MAX		0xffff

#define S5K4H7_EXPOSURE_MIN		2
#define S5K4H7_EXPOSURE_MARGIN		5
#define S5K4H7_EXPOSURE_DEFAULT		976

/* Analogue gain code = gain * 32: 1.0 to 16.0 */
#define S5K4H7_AGAIN_MIN		32
#define S5K4H7_AGAIN_MAX		512

/* The vendor modes read x 8..3271 and y 8..2455 at most. */
#define S5K4H7_NATIVE_WIDTH		3280
#define S5K4H7_NATIVE_HEIGHT		2464

#define S5K4H7_AUTOSUSPEND_DELAY_MS	1000

struct s5k4h7_mode {
	u32 width;
	u32 height;
	/* Analogue crop in the pixel array */
	struct v4l2_rect crop;
	/* Frame length in lines: the vendor table's, and the default */
	u32 frame_length_min;
	u32 frame_length_def;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

struct s5k4h7 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	/* Optional, NULL if the board does not describe it */
	struct regulator *vcam;
	struct regulator *vddio;
	struct regulator *vddd;
	struct regulator *vdda;
	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	/* Cluster: hflip and vflip must stay adjacent. */
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct s5k4h7 *to_s5k4h7(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4h7, sd);
}

static const s64 s5k4h7_link_freq_menu[] = {
	S5K4H7_LINK_FREQ,
};

/* Menu index is the CCS test_pattern_mode value. */
static const char * const s5k4h7_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

/*
 * Indexed by (vflip << 1) | hflip. GRBG without flips is the vendor's output
 * format; the flipped orders assume that a flip reverses the readout of the
 * even-sized window (CCS pixel order rule) and are not verified.
 */
static const u32 s5k4h7_mbus_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

/*
 * Largest window the vendor modes read, used as the active area. No mode
 * reads the border around it, so it also bounds every crop.
 */
static const struct v4l2_rect s5k4h7_active_area = {
	.left = 8,
	.top = 8,
	.width = 3264,
	.height = 2448,
};

/*
 * Vendor tables, one byte per write as the vendor writes them. The init table
 * follows every power-up, a mode table every stream start; the sensor stays
 * in software standby until mode_select is set.
 */
/* rubenss5k4h7_init_setting, vendor header lines 26-59 */
static const struct cci_reg_sequence s5k4h7_init_regs[] = {
	/* mode_select: software standby */
	{ CCI_REG8(0x0100), 0x00 },
	/* mapped_defect_correct_en (sensor OTP defect map) */
	{ CCI_REG8(0x0b05), 0x01 },
	/* Undocumented analogue and timing settings */
	{ CCI_REG8(0x3074), 0x06 },
	{ CCI_REG8(0x3075), 0x2f },
	{ CCI_REG8(0x308a), 0x20 },
	{ CCI_REG8(0x308b), 0x08 },
	{ CCI_REG8(0x308c), 0x0b },
	{ CCI_REG8(0x3081), 0x07 },
	{ CCI_REG8(0x307b), 0x85 },
	{ CCI_REG8(0x307a), 0x0a },
	{ CCI_REG8(0x3079), 0x0a },
	{ CCI_REG8(0x306e), 0x71 },
	{ CCI_REG8(0x306f), 0x28 },
	{ CCI_REG8(0x301f), 0x20 },
	{ CCI_REG8(0x306b), 0x9a },
	{ CCI_REG8(0x3091), 0x1f },
	{ CCI_REG8(0x30c4), 0x06 },
	{ CCI_REG8(0x3200), 0x09 },
	{ CCI_REG8(0x306a), 0x79 },
	{ CCI_REG8(0x30b0), 0xff },
	{ CCI_REG8(0x306d), 0x08 },
	{ CCI_REG8(0x3080), 0x00 },
	{ CCI_REG8(0x3929), 0x3f },
	{ CCI_REG8(0x3084), 0x16 },
	{ CCI_REG8(0x3070), 0x0f },
	{ CCI_REG8(0x3b45), 0x01 },
	{ CCI_REG8(0x30c2), 0x05 },
	{ CCI_REG8(0x3069), 0x87 },
	{ CCI_REG8(0x3924), 0x7f },
	{ CCI_REG8(0x3925), 0xfd },
	{ CCI_REG8(0x3c08), 0xff },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c31), 0xff },
	{ CCI_REG8(0x3c32), 0xff },
};

/* rubenss5k4h7_preview_setting, vendor header lines 63-112 */
static const struct cci_reg_sequence s5k4h7_3264x2448_regs[] = {
	/* EXTCLK 24.00 MHz */
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	/* PLLs: VT 24 / 6 * 140 = 560 MHz, OP 24 / 6 * 165 = 660 MHz */
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xa5 },
	/* Undocumented */
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	/* vt_pix_clk_div 4: 140 MHz, two pixels per clock = 280 Mpixel/s */
	{ CCI_REG8(0x0301), 0x04 },
	/* requested_link_rate 660.0 Mbit/s (U16.16) */
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0x94 },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	/* csi_data_format RAW10, csi_lane_mode four lanes */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	/* Undocumented */
	{ CCI_REG8(0x3906), 0x04 },
	/* Readout x 8..3271, y 8..2455; output 3264x2448 */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x09 },
	{ CCI_REG8(0x034b), 0x97 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0x90 },
	/* No binning, no skipping */
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	/* image_orientation: V4L2_CID_HFLIP/VFLIP overwrite it */
	{ CCI_REG8(0x0101), 0x00 },
	/* frame_length_lines 2530 (V4L2_CID_VBLANK overwrites it), line_length_pck 3688 */
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	/* fine_integration_time 3544; coarse 2, V4L2_CID_EXPOSURE overwrites it */
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x00 },
	{ CCI_REG8(0x0203), 0x02 },
	/* Undocumented */
	{ CCI_REG8(0x3931), 0x02 },
	{ CCI_REG8(0x3400), 0x01 },
};

/* rubenss5k4h7_normal_video_setting, vendor header lines 121-170 */
static const struct cci_reg_sequence s5k4h7_3264x1836_regs[] = {
	/* EXTCLK 24.00 MHz */
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	/* PLLs: VT 24 / 6 * 140 = 560 MHz, OP 24 / 6 * 165 = 660 MHz */
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xa5 },
	/* Undocumented */
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	/* vt_pix_clk_div 4: 140 MHz, two pixels per clock = 280 Mpixel/s */
	{ CCI_REG8(0x0301), 0x04 },
	/* requested_link_rate 660.0 Mbit/s (U16.16) */
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0x94 },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	/* csi_data_format RAW10, csi_lane_mode four lanes */
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	/* Undocumented */
	{ CCI_REG8(0x3906), 0x04 },
	/* Readout x 8..3271, y 314..2149; output 3264x1836 */
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x01 },
	{ CCI_REG8(0x0347), 0x3a },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x08 },
	{ CCI_REG8(0x034b), 0x65 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x07 },
	{ CCI_REG8(0x034f), 0x2c },
	/* No binning, no skipping */
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	/* image_orientation: V4L2_CID_HFLIP/VFLIP overwrite it */
	{ CCI_REG8(0x0101), 0x00 },
	/* frame_length_lines 2530 (V4L2_CID_VBLANK overwrites it), line_length_pck 3688 */
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	/* fine_integration_time 3544; coarse 2, V4L2_CID_EXPOSURE overwrites it */
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x00 },
	{ CCI_REG8(0x0203), 0x02 },
	/* Undocumented */
	{ CCI_REG8(0x3931), 0x02 },
	{ CCI_REG8(0x3400), 0x01 },
};

static const struct s5k4h7_mode s5k4h7_modes[] = {
	{
		/* Vendor preview (also capture, hs_video and slim_video) */
		.width = 3264,
		.height = 2448,
		.crop = {
			.left = 8,
			.top = 8,
			.width = 3264,
			.height = 2448,
		},
		.frame_length_min = 2530,
		/*
		 * 20 fps by default: libcamera's software ISP does not keep up
		 * with 8 Mpixel frames at 30 fps on the Redmi K50.
		 */
		.frame_length_def = 3796,
		.regs = s5k4h7_3264x2448_regs,
		.num_regs = ARRAY_SIZE(s5k4h7_3264x2448_regs),
	},
	{
		/* Vendor normal_video: 16:9 crop of the full width */
		.width = 3264,
		.height = 1836,
		.crop = {
			.left = 8,
			.top = 314,
			.width = 3264,
			.height = 1836,
		},
		.frame_length_min = 2530,
		.frame_length_def = 2530,
		.regs = s5k4h7_3264x1836_regs,
		.num_regs = ARRAY_SIZE(s5k4h7_3264x1836_regs),
	},
};

static u32 s5k4h7_mbus_code(bool hflip, bool vflip)
{
	return s5k4h7_mbus_codes[(vflip ? 2 : 0) | (hflip ? 1 : 0)];
}

/* Bayer order of the applied flips; a tried or failed value is not applied. */
static u32 s5k4h7_cur_mbus_code(struct s5k4h7 *s5k4h7)
{
	return s5k4h7_mbus_code(s5k4h7->hflip->cur.val, s5k4h7->vflip->cur.val);
}

static const struct s5k4h7_mode *s5k4h7_find_mode(u32 width, u32 height)
{
	return v4l2_find_nearest_size(s5k4h7_modes, ARRAY_SIZE(s5k4h7_modes),
				      width, height, width, height);
}

static void s5k4h7_fill_format(struct s5k4h7 *s5k4h7,
			       const struct s5k4h7_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = s5k4h7_cur_mbus_code(s5k4h7);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/*
 * While streaming, write a 16-bit register under the grouped parameter hold,
 * as the vendor driver does for the frame length and the exposure, so that
 * both bytes take effect in the same frame. The hold is always released.
 */
static int s5k4h7_write_held(struct s5k4h7 *s5k4h7, u32 reg, u64 val)
{
	int ret = 0;
	int err;

	if (!v4l2_subdev_is_streaming(&s5k4h7->sd))
		return cci_write(s5k4h7->regmap, reg, val, NULL);

	cci_write(s5k4h7->regmap, S5K4H7_REG_GROUP_HOLD, S5K4H7_GROUP_HOLD_ON,
		  &ret);
	cci_write(s5k4h7->regmap, reg, val, &ret);
	err = cci_write(s5k4h7->regmap, S5K4H7_REG_GROUP_HOLD,
			S5K4H7_GROUP_HOLD_OFF, NULL);

	return ret ?: err;
}

/*
 * The CCI regmap has no locking: all accesses run under the control handler
 * lock (also the subdev state lock), in the runtime PM callbacks or in probe.
 */
static int s5k4h7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7 *s5k4h7 = container_of(ctrl->handler, struct s5k4h7,
					     ctrls);
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 code = 0;
	int ret = 0;
	u32 val;

	state = v4l2_subdev_get_locked_active_state(&s5k4h7->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* The exposure must end 5 lines before the frame does. */
		ret = __v4l2_ctrl_modify_range(s5k4h7->exposure,
					       s5k4h7->exposure->minimum,
					       fmt->height + ctrl->val -
					       S5K4H7_EXPOSURE_MARGIN,
					       s5k4h7->exposure->step,
					       s5k4h7->exposure->default_value);
		if (ret)
			return ret;
		break;
	case V4L2_CID_HFLIP:
		/* The flips change the Bayer order of the active format. */
		code = s5k4h7_mbus_code(s5k4h7->hflip->val, s5k4h7->vflip->val);
		break;
	}

	/* Write only while powered; enable_streams applies all controls. */
	if (pm_runtime_get_if_active(s5k4h7->dev) <= 0) {
		if (code)
			fmt->code = code;
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = s5k4h7_write_held(s5k4h7, S5K4H7_REG_FRAME_LENGTH_LINES,
					fmt->height + ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = s5k4h7_write_held(s5k4h7,
					S5K4H7_REG_COARSE_INTEGRATION_TIME,
					ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		/* Without a hold, as the vendor driver writes the gain */
		cci_write(s5k4h7->regmap, S5K4H7_REG_AGAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_HFLIP:
		/* Cluster master: also applies V4L2_CID_VFLIP. */
		val = (s5k4h7->hflip->val ? S5K4H7_HFLIP : 0) |
		      (s5k4h7->vflip->val ? S5K4H7_VFLIP : 0);
		cci_write(s5k4h7->regmap, S5K4H7_REG_IMAGE_ORIENTATION, val,
			  &ret);
		if (!ret)
			fmt->code = code;
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(s5k4h7->regmap, S5K4H7_REG_TEST_PATTERN_MODE,
			  ctrl->val, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(s5k4h7->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7_ctrl_ops = {
	.s_ctrl = s5k4h7_set_ctrl,
};

static int s5k4h7_init_controls(struct s5k4h7 *s5k4h7)
{
	const struct s5k4h7_mode *mode = &s5k4h7_modes[0];
	u32 hblank = S5K4H7_LINE_LENGTH - mode->width;
	u32 vblank_min = mode->frame_length_min - mode->height;
	u32 vblank = mode->frame_length_def - mode->height;
	struct v4l2_ctrl_handler *hdl = &s5k4h7->ctrls;
	const struct v4l2_ctrl_ops *ops = &s5k4h7_ctrl_ops;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	ret = v4l2_fwnode_device_parse(s5k4h7->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 11);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(s5k4h7_link_freq_menu) - 1, 0,
				      s5k4h7_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, S5K4H7_PIXEL_RATE,
			  S5K4H7_PIXEL_RATE, 1, S5K4H7_PIXEL_RATE);

	s5k4h7->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (s5k4h7->hblank)
		s5k4h7->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The handler setup in enable_streams writes the controls in creation
	 * order: frame length, exposure, then gain, as the vendor driver does.
	 */
	s5k4h7->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK, vblank_min,
					   S5K4H7_FRAME_LENGTH_MAX - mode->height,
					   1, vblank);

	/*
	 * Step 1: the vendor declares a step of two lines but rounds to it in
	 * only one of its exposure paths. Odd values are not verified.
	 */
	s5k4h7->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     S5K4H7_EXPOSURE_MIN,
					     mode->frame_length_def -
					     S5K4H7_EXPOSURE_MARGIN, 1,
					     S5K4H7_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, S5K4H7_AGAIN_MIN,
			  S5K4H7_AGAIN_MAX, 1, S5K4H7_AGAIN_MIN);

	s5k4h7->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	s5k4h7->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (s5k4h7->hflip && s5k4h7->vflip) {
		s5k4h7->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		s5k4h7->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		v4l2_ctrl_cluster(2, &s5k4h7->hflip);
	}

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h7_test_pattern_menu) - 1,
				     0, 0, s5k4h7_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	s5k4h7->sd.ctrl_handler = hdl;

	return 0;
}

/* Called with the state lock held, after the active format has changed */
static int s5k4h7_update_mode_ctrls(struct s5k4h7 *s5k4h7,
				    const struct s5k4h7_mode *mode)
{
	u32 hblank = S5K4H7_LINE_LENGTH - mode->width;
	u32 vblank_min = mode->frame_length_min - mode->height;
	u32 vblank = mode->frame_length_def - mode->height;
	int ret;

	ret = __v4l2_ctrl_modify_range(s5k4h7->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(s5k4h7->vblank, vblank_min,
				       S5K4H7_FRAME_LENGTH_MAX - mode->height,
				       1, vblank);
	if (ret)
		return ret;

	/* Every mode starts at its default frame length. */
	ret = __v4l2_ctrl_s_ctrl(s5k4h7->vblank, vblank);
	if (ret)
		return ret;

	/* The VBLANK handler does not run if the value did not change. */
	return __v4l2_ctrl_modify_range(s5k4h7->exposure, S5K4H7_EXPOSURE_MIN,
					mode->frame_length_def -
					S5K4H7_EXPOSURE_MARGIN, 1,
					S5K4H7_EXPOSURE_DEFAULT);
}

/*
 * Stop the transmitter. If the standby write fails, hold the sensor in reset
 * instead; it stays unusable until runtime suspend powers it down and the
 * next runtime resume initialises it again.
 */
static int s5k4h7_stop(struct s5k4h7 *s5k4h7)
{
	int ret;

	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_MODE_SELECT,
			S5K4H7_MODE_STANDBY, NULL);
	if (!ret || !s5k4h7->reset_gpio)
		return ret;

	dev_err(s5k4h7->dev, "stream off failed (%d), holding the sensor in reset\n",
		ret);
	ret = gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	usleep_range(1000, 2000);

	return ret;
}

static int s5k4h7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	const struct v4l2_mbus_framefmt *fmt;
	const struct s5k4h7_mode *mode;
	int ret;

	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = s5k4h7_find_mode(fmt->width, fmt->height);

	/* Normally the receiver's runtime PM link has resumed the sensor. */
	ret = pm_runtime_resume_and_get(s5k4h7->dev);
	if (ret)
		return ret;

	ret = cci_multi_reg_write(s5k4h7->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_rpm_put;

	/* Frame length, exposure, gain, flips and test pattern of frame 0 */
	ret = __v4l2_ctrl_handler_setup(&s5k4h7->ctrls);
	if (ret)
		goto err_rpm_put;

	/* Vendor streaming_control(): 5 ms before and after mode_select */
	usleep_range(5000, 6000);
	ret = cci_write(s5k4h7->regmap, S5K4H7_REG_MODE_SELECT,
			S5K4H7_MODE_STREAMING, NULL);
	if (ret) {
		/* A failed write may still have started the transmitter. */
		s5k4h7_stop(s5k4h7);
		goto err_rpm_put;
	}
	usleep_range(5000, 6000);

	__v4l2_ctrl_grab(s5k4h7->hflip, true);
	__v4l2_ctrl_grab(s5k4h7->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(s5k4h7->dev);
	return ret;
}

static int s5k4h7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	int ret;

	ret = s5k4h7_stop(s5k4h7);
	if (ret)
		dev_err(s5k4h7->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(s5k4h7->hflip, false);
	__v4l2_ctrl_grab(s5k4h7->vflip, false);

	pm_runtime_put_autosuspend(s5k4h7->dev);

	/*
	 * The reference is dropped: report success so that the stream is
	 * marked disabled, as the core does for s_stream failures.
	 */
	return 0;
}

static int s5k4h7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = s5k4h7_cur_mbus_code(to_s5k4h7(sd));

	return 0;
}

static int s5k4h7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k4h7_modes) ||
	    fse->code != s5k4h7_cur_mbus_code(to_s5k4h7(sd)))
		return -EINVAL;

	fse->min_width = s5k4h7_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int s5k4h7_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);
	const struct s5k4h7_mode *mode;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = s5k4h7_find_mode(fmt->format.width, fmt->format.height);

	s5k4h7_fill_format(s5k4h7, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	/* The control handlers read the new height from the active state. */
	return s5k4h7_update_mode_ctrls(s5k4h7, mode);
}

static int s5k4h7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K4H7_NATIVE_WIDTH;
		sel->r.height = S5K4H7_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = s5k4h7_active_area;
		return 0;
	default:
		return -EINVAL;
	}
}

static int s5k4h7_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
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

static int s5k4h7_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = S5K4H7_LINK_FREQ;
	config->bus.mipi_csi2.num_data_lanes = S5K4H7_DATA_LANES;

	return 0;
}

static int s5k4h7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct s5k4h7_mode *mode = &s5k4h7_modes[0];

	s5k4h7_fill_format(to_s5k4h7(sd), mode,
			   v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	return 0;
}

static const struct v4l2_subdev_video_ops s5k4h7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h7_pad_ops = {
	.enum_mbus_code = s5k4h7_enum_mbus_code,
	.enum_frame_size = s5k4h7_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k4h7_set_fmt,
	.get_selection = s5k4h7_get_selection,
	.get_frame_desc = s5k4h7_get_frame_desc,
	.get_mbus_config = s5k4h7_get_mbus_config,
	.enable_streams = s5k4h7_enable_streams,
	.disable_streams = s5k4h7_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h7_subdev_ops = {
	.video = &s5k4h7_video_ops,
	.pad = &s5k4h7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7_internal_ops = {
	.init_state = s5k4h7_init_state,
};

/* Vendor pw_seq order and delays: RST 0, AVDD1, DOVDD, DVDD, AVDD, RST 1, MCLK */
static int s5k4h7_power_on(struct s5k4h7 *s5k4h7)
{
	int ret;

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	usleep_range(1000, 2000);

	/* One supply at a time, never regulator_bulk_enable() */
	if (s5k4h7->vcam) {
		ret = regulator_enable(s5k4h7->vcam);
		if (ret)
			return ret;
	}
	usleep_range(1000, 2000);

	ret = regulator_enable(s5k4h7->vddio);
	if (ret)
		goto err_vcam;
	usleep_range(1000, 2000);

	ret = regulator_enable(s5k4h7->vddd);
	if (ret)
		goto err_vddio;
	usleep_range(1000, 2000);

	ret = regulator_enable(s5k4h7->vdda);
	if (ret)
		goto err_vddd;
	usleep_range(2000, 3000);

	/* The vendor releases the reset before MCLK starts. */
	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 0);
	usleep_range(1000, 2000);

	ret = clk_prepare_enable(s5k4h7->xclk);
	if (ret)
		goto err_reset;

	/* Connect MCLK to the pad; a no-op without pinctrl states. */
	ret = pinctrl_pm_select_default_state(s5k4h7->dev);
	if (ret)
		goto err_pins;
	usleep_range(2000, 3000);

	return 0;

err_pins:
	/* Undo a partially applied default state. */
	pinctrl_pm_select_sleep_state(s5k4h7->dev);
	clk_disable_unprepare(s5k4h7->xclk);
err_reset:
	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	usleep_range(1000, 2000);
	regulator_disable(s5k4h7->vdda);
	usleep_range(2000, 3000);
err_vddd:
	regulator_disable(s5k4h7->vddd);
	usleep_range(1000, 2000);
err_vddio:
	regulator_disable(s5k4h7->vddio);
	usleep_range(1000, 2000);
err_vcam:
	if (s5k4h7->vcam)
		regulator_disable(s5k4h7->vcam);
	usleep_range(1000, 2000);
	return ret;
}

/* The vendor pw_seq in reverse, with the same delays */
static void s5k4h7_power_off(struct s5k4h7 *s5k4h7)
{
	/* Park the MCLK pad as a low GPIO before gating the clock. */
	pinctrl_pm_select_sleep_state(s5k4h7->dev);
	usleep_range(2000, 3000);
	clk_disable_unprepare(s5k4h7->xclk);

	gpiod_set_value_cansleep(s5k4h7->reset_gpio, 1);
	usleep_range(1000, 2000);

	regulator_disable(s5k4h7->vdda);
	usleep_range(2000, 3000);
	regulator_disable(s5k4h7->vddd);
	usleep_range(1000, 2000);
	regulator_disable(s5k4h7->vddio);
	usleep_range(1000, 2000);
	if (s5k4h7->vcam)
		regulator_disable(s5k4h7->vcam);
	/* Minimum off time before the next power-up */
	usleep_range(1000, 2000);
}

static int s5k4h7_runtime_resume(struct device *dev)
{
	struct s5k4h7 *s5k4h7 = to_s5k4h7(dev_get_drvdata(dev));
	int ret;

	ret = s5k4h7_power_on(s5k4h7);
	if (ret)
		goto err;

	ret = cci_multi_reg_write(s5k4h7->regmap, s5k4h7_init_regs,
				  ARRAY_SIZE(s5k4h7_init_regs), NULL);
	if (ret) {
		s5k4h7_power_off(s5k4h7);
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

static int s5k4h7_runtime_suspend(struct device *dev)
{
	s5k4h7_power_off(to_s5k4h7(dev_get_drvdata(dev)));

	return 0;
}

/*
 * Report of the read-only CCS information registers that the vendor driver
 * never reads: Bayer order, black level and analogue gain model. Only
 * logged; a failed read does not fail the probe.
 */
static void s5k4h7_log_info(struct s5k4h7 *s5k4h7)
{
	u64 rev = 0, order = 0, pedestal = 0, cap = 0, min = 0, max = 0;
	u64 step = 0, type = 0, m0 = 0, c0 = 0, m1 = 0, c1 = 0;
	int ret = 0;

	cci_read(s5k4h7->regmap, S5K4H7_REG_REVISION, &rev, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_PIXEL_ORDER, &order, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_DATA_PEDESTAL, &pedestal, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_CAPABILITY, &cap, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_CODE_MIN, &min, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_CODE_MAX, &max, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_CODE_STEP, &step, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_TYPE, &type, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_M0, &m0, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_C0, &c0, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_M1, &m1, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_AGAIN_C1, &c1, &ret);

	/* M0, C0, M1 and C1 are signed. */
	dev_info(s5k4h7->dev,
		 "chip id %#x, revision %#llx, pixel order %llu, pedestal %llu, gain capability %llu, codes %llu..%llu/%llu, type %llu, m0 %d c0 %d m1 %d c1 %d, read status %d\n",
		 S5K4H7_SENSOR_ID, rev, order, pedestal, cap, min, max, step,
		 type, (s16)m0, (s16)c0, (s16)m1, (s16)c1, ret);
}

static int s5k4h7_identify(struct s5k4h7 *s5k4h7)
{
	u64 hi, lo;
	int ret = 0;

	/* Two 8-bit reads, as the vendor driver reads the ID */
	cci_read(s5k4h7->regmap, S5K4H7_REG_SENSOR_ID_H, &hi, &ret);
	cci_read(s5k4h7->regmap, S5K4H7_REG_SENSOR_ID_L, &lo, &ret);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret, "failed to read chip id\n");

	if (((hi << 8) | lo) != S5K4H7_SENSOR_ID)
		return dev_err_probe(s5k4h7->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx, expected 0x%04x\n",
				     (hi << 8) | lo, S5K4H7_SENSOR_ID);

	s5k4h7_log_info(s5k4h7);

	return 0;
}

static int s5k4h7_parse_fwnode(struct s5k4h7 *s5k4h7)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	/*
	 * Also accept an endpoint without an available remote: the sensor can
	 * be identified before the board connects or enables the receiver.
	 */
	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(s5k4h7->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT |
					     FWNODE_GRAPH_DEVICE_DISABLED);
	if (!ep)
		return dev_err_probe(s5k4h7->dev, -ENXIO, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(s5k4h7->dev, ret, "invalid endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K4H7_DATA_LANES) {
		ret = dev_err_probe(s5k4h7->dev, -EINVAL,
				    "four data lanes are required\n");
		goto out;
	}

	/* The link frequency the modes need must be allowed by the board. */
	ret = v4l2_link_freq_to_bitmap(s5k4h7->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       s5k4h7_link_freq_menu,
				       ARRAY_SIZE(s5k4h7_link_freq_menu),
				       &link_freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static void s5k4h7_disable_runtime_pm(struct s5k4h7 *s5k4h7)
{
	pm_runtime_disable(s5k4h7->dev);
	if (!pm_runtime_status_suspended(s5k4h7->dev)) {
		s5k4h7_power_off(s5k4h7);
		pm_runtime_set_suspended(s5k4h7->dev);
	}
	pm_runtime_dont_use_autosuspend(s5k4h7->dev);
}

static int s5k4h7_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k4h7 *s5k4h7;
	unsigned long rate;
	int ret;

	s5k4h7 = devm_kzalloc(dev, sizeof(*s5k4h7), GFP_KERNEL);
	if (!s5k4h7)
		return -ENOMEM;

	s5k4h7->dev = dev;
	v4l2_i2c_subdev_init(&s5k4h7->sd, client, &s5k4h7_subdev_ops);

	ret = s5k4h7_parse_fwnode(s5k4h7);
	if (ret)
		return ret;

	s5k4h7->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k4h7->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->regmap),
				     "failed to init CCI\n");

	/* The board selects a 24 MHz parent; the rate is checked, never set. */
	s5k4h7->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k4h7->xclk))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->xclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(s5k4h7->xclk);
	if (rate != S5K4H7_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK must be 24 MHz, got %lu Hz\n", rate);

	s5k4h7->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(s5k4h7->vddio))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->vddio),
				     "failed to get VDDIO\n");

	s5k4h7->vddd = devm_regulator_get(dev, "vddd");
	if (IS_ERR(s5k4h7->vddd))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->vddd),
				     "failed to get VDDD\n");

	s5k4h7->vdda = devm_regulator_get(dev, "vdda");
	if (IS_ERR(s5k4h7->vdda))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->vdda),
				     "failed to get VDDA\n");

	/* Asserted (reset pin low) whenever the sensor is not powered */
	s5k4h7->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(s5k4h7->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k4h7->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Last, so that a deferred probe does not repeat the message below */
	s5k4h7->vcam = devm_regulator_get_optional(dev, "vcam");
	if (IS_ERR(s5k4h7->vcam)) {
		if (PTR_ERR(s5k4h7->vcam) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(s5k4h7->vcam),
					     "failed to get VCAM\n");

		dev_info(dev, "no vcam supply\n");
		s5k4h7->vcam = NULL;
	}

	/* Identify with the sensor powered only for the chip ID read. */
	ret = s5k4h7_power_on(s5k4h7);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = s5k4h7_identify(s5k4h7);
	s5k4h7_power_off(s5k4h7);
	if (ret)
		return ret;

	ret = s5k4h7_init_controls(s5k4h7);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	s5k4h7->sd.state_lock = s5k4h7->ctrls.lock;
	s5k4h7->sd.internal_ops = &s5k4h7_internal_ops;
	s5k4h7->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k4h7->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k4h7->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&s5k4h7->sd.entity, 1, &s5k4h7->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&s5k4h7->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev\n");
		goto err_entity;
	}

	/* Suspended until a user resumes it; resume writes the init table. */
	pm_runtime_set_autosuspend_delay(dev, S5K4H7_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&s5k4h7->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	return 0;

err_pm:
	s5k4h7_disable_runtime_pm(s5k4h7);
	v4l2_subdev_cleanup(&s5k4h7->sd);
err_entity:
	media_entity_cleanup(&s5k4h7->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s5k4h7->ctrls);
	return ret;
}

static void s5k4h7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7 *s5k4h7 = to_s5k4h7(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k4h7->ctrls);

	s5k4h7_disable_runtime_pm(s5k4h7);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k4h7_pm_ops, s5k4h7_runtime_suspend,
				 s5k4h7_runtime_resume, NULL);

static const struct of_device_id s5k4h7_of_match[] = {
	{ .compatible = "samsung,s5k4h7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7_of_match);

static struct i2c_driver s5k4h7_i2c_driver = {
	.driver = {
		.name = "s5k4h7",
		.of_match_table = s5k4h7_of_match,
		.pm = pm_ptr(&s5k4h7_pm_ops),
		/* Unbinding would remove the sensor under a running stream. */
		.suppress_bind_attrs = true,
	},
	.probe = s5k4h7_probe,
	.remove = s5k4h7_remove,
};
module_i2c_driver(s5k4h7_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K4H7 image sensor driver");
MODULE_LICENSE("GPL");
