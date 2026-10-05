// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX596 image sensor driver
 *
 * Copyright (c) 2019 MediaTek Inc.
 *
 * The register tables, the power sequence and the gain conversion are taken
 * from the MediaTek imgsensor driver of the Xiaomi Redmi K50 front camera,
 * MiCode/Xiaomi_Kernel_OpenSource commit 270b84910b941bf75490fccdb5fb2b3021dd7cbf,
 * drivers/misc/mediatek/imgsensor/src-v4l2/common/rubensimx596_mipi_raw/.
 * The V4L2 structure follows gc02m1.c and s5k3m5.c (Copyright (c) 2025 Linaro
 * Ltd).
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

#define IMX596_REG_CHIP_ID		CCI_REG16(0x0016)
#define IMX596_CHIP_ID			0x0596
#define IMX596_REG_REVISION		CCI_REG8(0x0018)
#define IMX596_REG_MODE_SELECT		CCI_REG8(0x0100)
#define IMX596_MODE_STANDBY		0x00
#define IMX596_MODE_STREAMING		0x01
#define IMX596_REG_ORIENTATION		CCI_REG8(0x0101)
#define IMX596_HFLIP			BIT(0)
#define IMX596_VFLIP			BIT(1)
#define IMX596_REG_GROUP_HOLD		CCI_REG8(0x0104)
#define IMX596_REG_CSI_DATA_FORMAT	CCI_REG16(0x0112)
#define IMX596_REG_CSI_LANE_MODE	CCI_REG8(0x0114)
#define IMX596_REG_EXPOSURE		CCI_REG16(0x0202)
#define IMX596_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
/* CCS digital_gain_global as in the vendor tables, 0x0100 = 1.0 */
#define IMX596_REG_DIGITAL_GAIN		CCI_REG16(0x020e)
#define IMX596_REG_PLL_MODE		CCI_REG8(0x0310)
#define IMX596_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define IMX596_REG_TEST_PATTERN		CCI_REG16(0x0600)

/* CCS limit registers, read once at probe for the log */
#define IMX596_REG_LIMIT_AGAIN_MIN	CCI_REG16(0x0084)
#define IMX596_REG_LIMIT_AGAIN_MAX	CCI_REG16(0x0086)
#define IMX596_REG_LIMIT_AGAIN_M0	CCI_REG16(0x008c)
#define IMX596_REG_LIMIT_AGAIN_C0	CCI_REG16(0x008e)
#define IMX596_REG_LIMIT_AGAIN_M1	CCI_REG16(0x0090)
#define IMX596_REG_LIMIT_AGAIN_C1	CCI_REG16(0x0092)
#define IMX596_REG_LIMIT_EXPOSURE_MIN	CCI_REG16(0x1004)
#define IMX596_REG_LIMIT_EXPOSURE_MARGIN CCI_REG16(0x1006)
#define IMX596_REG_LIMIT_DGAIN_MIN	CCI_REG16(0x1084)
#define IMX596_REG_LIMIT_DGAIN_MAX	CCI_REG16(0x1086)

/*
 * Not a CCS register: on the related IMX586, bit 0 selects per-channel
 * digital gains instead of a global one. Read once at probe for the log.
 */
#define IMX596_REG_DGAIN_MODE		CCI_REG8(0x3130)

#define IMX596_XCLK_FREQ		(24 * HZ_PER_MHZ)
/* 1356 Mbit/s per lane: 24 MHz / 4 x 226 (op_pre_pll_clk_div, op_pll_multiplier) */
#define IMX596_LINK_FREQ		(678 * HZ_PER_MHZ)
/* 24 MHz / 3 x 274 / 2 / 5 x 4 (pre_pll_clk_div, pll_multiplier, vt dividers) */
#define IMX596_PIXEL_RATE		(876800 * HZ_PER_KHZ)
#define IMX596_DATA_LANES		4

#define IMX596_NATIVE_WIDTH		5184
#define IMX596_NATIVE_HEIGHT		3904
#define IMX596_LINE_LENGTH		4644
#define IMX596_FRAME_LENGTH_30FPS	6293
#define IMX596_FRAME_LENGTH_MAX		0xffff

#define IMX596_EXPOSURE_MIN		16
#define IMX596_EXPOSURE_MARGIN		24
#define IMX596_EXPOSURE_DEFAULT		976

/*
 * V4L2_CID_ANALOGUE_GAIN is the gain in 1/1024 units, as in the vendor
 * driver, which converts it to the analogue gain code 0 (1x) to 960 (16x).
 */
#define IMX596_GAIN_UNIT		1024
#define IMX596_GAIN_MIN			IMX596_GAIN_UNIT
#define IMX596_GAIN_MAX			(16 * IMX596_GAIN_UNIT)

#define IMX596_DGAIN_MIN		0x0100
#define IMX596_DGAIN_MAX		0x0fff
#define IMX596_DGAIN_DEFAULT		0x0100

#define IMX596_AUTOSUSPEND_DELAY_MS	1000

static const s64 imx596_link_freq_menu[] = {
	IMX596_LINK_FREQ,
};

/* Values of test_pattern_mode (0x0600) */
static const char * const imx596_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade to Grey Colour Bars",
	"PN9",
};

/* Indexed by (vflip << 1) | hflip; the flipped orders are not verified. */
static const u32 imx596_mbus_codes[] = {
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const struct v4l2_rect imx596_native_area = {
	.left = 0,
	.top = 0,
	.width = IMX596_NATIVE_WIDTH,
	.height = IMX596_NATIVE_HEIGHT,
};

/*
 * The vendor register tables, copied in order and unchanged. None of them
 * starts streaming.
 */

/* Common settings, written after every power-up (vendor sensor_init) */
/* rubensimx596mipiraw_Sensor.h:26-389 (rubensimx596_init_setting, 364 writes) */
static const struct cci_reg_sequence imx596_init_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x321c), 0x00 },
	{ CCI_REG8(0x33f0), 0x05 },
	{ CCI_REG8(0x33f1), 0x03 },
	{ CCI_REG8(0x32c8), 0x01 },
	{ CCI_REG8(0x40a2), 0x01 },
	{ CCI_REG8(0x441f), 0x01 },
	{ CCI_REG8(0x4b20), 0x07 },
	{ CCI_REG8(0x5abe), 0x3a },
	{ CCI_REG8(0x5ac8), 0x3a },
	{ CCI_REG8(0x5ad0), 0x3a },
	{ CCI_REG8(0x5ada), 0x3a },
	{ CCI_REG8(0x5c04), 0x00 },
	{ CCI_REG8(0x5c05), 0x00 },
	{ CCI_REG8(0x5c06), 0x00 },
	{ CCI_REG8(0x5c0b), 0x00 },
	{ CCI_REG8(0x5c0c), 0x00 },
	{ CCI_REG8(0x5c0d), 0x00 },
	{ CCI_REG8(0x5c0e), 0x00 },
	{ CCI_REG8(0x5d55), 0x00 },
	{ CCI_REG8(0x5d56), 0x00 },
	{ CCI_REG8(0x6104), 0x0a },
	{ CCI_REG8(0x6105), 0x0a },
	{ CCI_REG8(0x6107), 0x0a },
	{ CCI_REG8(0x610e), 0x07 },
	{ CCI_REG8(0x610f), 0x07 },
	{ CCI_REG8(0x6110), 0x07 },
	{ CCI_REG8(0x6111), 0x07 },
	{ CCI_REG8(0x6112), 0x07 },
	{ CCI_REG8(0x6113), 0x07 },
	{ CCI_REG8(0x6114), 0x07 },
	{ CCI_REG8(0x6115), 0x07 },
	{ CCI_REG8(0x611c), 0x0b },
	{ CCI_REG8(0x611d), 0x07 },
	{ CCI_REG8(0x611e), 0x09 },
	{ CCI_REG8(0x611f), 0x07 },
	{ CCI_REG8(0x6122), 0x09 },
	{ CCI_REG8(0x612b), 0x07 },
	{ CCI_REG8(0x612d), 0x07 },
	{ CCI_REG8(0x612e), 0x07 },
	{ CCI_REG8(0x612f), 0x07 },
	{ CCI_REG8(0x6131), 0x07 },
	{ CCI_REG8(0x6138), 0x0b },
	{ CCI_REG8(0x6139), 0x07 },
	{ CCI_REG8(0x613a), 0x0b },
	{ CCI_REG8(0x613f), 0x07 },
	{ CCI_REG8(0x6182), 0x01 },
	{ CCI_REG8(0x6183), 0x01 },
	{ CCI_REG8(0x6184), 0x01 },
	{ CCI_REG8(0x6185), 0x01 },
	{ CCI_REG8(0x6186), 0x01 },
	{ CCI_REG8(0x6187), 0x01 },
	{ CCI_REG8(0x6188), 0x01 },
	{ CCI_REG8(0x6189), 0x01 },
	{ CCI_REG8(0x618a), 0x01 },
	{ CCI_REG8(0x618b), 0x01 },
	{ CCI_REG8(0x618c), 0x01 },
	{ CCI_REG8(0x618d), 0x01 },
	{ CCI_REG8(0x618e), 0x01 },
	{ CCI_REG8(0x618f), 0x01 },
	{ CCI_REG8(0x6194), 0x01 },
	{ CCI_REG8(0x6195), 0x01 },
	{ CCI_REG8(0x6197), 0x01 },
	{ CCI_REG8(0x6199), 0x01 },
	{ CCI_REG8(0x619b), 0x01 },
	{ CCI_REG8(0x619d), 0x01 },
	{ CCI_REG8(0x61a2), 0x01 },
	{ CCI_REG8(0x61a3), 0x01 },
	{ CCI_REG8(0x61a5), 0x01 },
	{ CCI_REG8(0x6205), 0x29 },
	{ CCI_REG8(0x6207), 0x29 },
	{ CCI_REG8(0x6209), 0x29 },
	{ CCI_REG8(0x620b), 0x29 },
	{ CCI_REG8(0x620d), 0x29 },
	{ CCI_REG8(0x620f), 0x29 },
	{ CCI_REG8(0x6211), 0x29 },
	{ CCI_REG8(0x6213), 0x29 },
	{ CCI_REG8(0x6215), 0x29 },
	{ CCI_REG8(0x6217), 0x29 },
	{ CCI_REG8(0x6219), 0x29 },
	{ CCI_REG8(0x621b), 0x29 },
	{ CCI_REG8(0x621d), 0x29 },
	{ CCI_REG8(0x621f), 0x29 },
	{ CCI_REG8(0x6221), 0x0a },
	{ CCI_REG8(0x622b), 0x29 },
	{ CCI_REG8(0x622d), 0x29 },
	{ CCI_REG8(0x6231), 0x29 },
	{ CCI_REG8(0x6235), 0x29 },
	{ CCI_REG8(0x6239), 0x29 },
	{ CCI_REG8(0x623d), 0x29 },
	{ CCI_REG8(0x6249), 0x29 },
	{ CCI_REG8(0x624b), 0x29 },
	{ CCI_REG8(0x624f), 0x29 },
	{ CCI_REG8(0x628f), 0x90 },
	{ CCI_REG8(0x6291), 0x90 },
	{ CCI_REG8(0x6293), 0x90 },
	{ CCI_REG8(0x6295), 0x90 },
	{ CCI_REG8(0x6297), 0x90 },
	{ CCI_REG8(0x6299), 0x90 },
	{ CCI_REG8(0x629b), 0x90 },
	{ CCI_REG8(0x629d), 0x90 },
	{ CCI_REG8(0x629f), 0x90 },
	{ CCI_REG8(0x62a1), 0x90 },
	{ CCI_REG8(0x62a3), 0x90 },
	{ CCI_REG8(0x62a5), 0x90 },
	{ CCI_REG8(0x62a7), 0x90 },
	{ CCI_REG8(0x62a9), 0x90 },
	{ CCI_REG8(0x62ab), 0x24 },
	{ CCI_REG8(0x62b5), 0x90 },
	{ CCI_REG8(0x62b7), 0x90 },
	{ CCI_REG8(0x62bb), 0x90 },
	{ CCI_REG8(0x62bf), 0x90 },
	{ CCI_REG8(0x62c3), 0x90 },
	{ CCI_REG8(0x62c7), 0x90 },
	{ CCI_REG8(0x62d3), 0x90 },
	{ CCI_REG8(0x62d5), 0x90 },
	{ CCI_REG8(0x62d9), 0x90 },
	{ CCI_REG8(0x6319), 0x90 },
	{ CCI_REG8(0x631b), 0x90 },
	{ CCI_REG8(0x631f), 0x90 },
	{ CCI_REG8(0x6323), 0x90 },
	{ CCI_REG8(0x636d), 0x28 },
	{ CCI_REG8(0x636e), 0x2a },
	{ CCI_REG8(0x636f), 0x29 },
	{ CCI_REG8(0x6370), 0x29 },
	{ CCI_REG8(0x6371), 0x0e },
	{ CCI_REG8(0x6372), 0x0e },
	{ CCI_REG8(0x6373), 0x28 },
	{ CCI_REG8(0x6374), 0x0f },
	{ CCI_REG8(0x6375), 0x1e },
	{ CCI_REG8(0x6376), 0x0d },
	{ CCI_REG8(0x637d), 0x2a },
	{ CCI_REG8(0x637e), 0x2b },
	{ CCI_REG8(0x637f), 0x2a },
	{ CCI_REG8(0x6380), 0x2a },
	{ CCI_REG8(0x6381), 0x28 },
	{ CCI_REG8(0x6382), 0x29 },
	{ CCI_REG8(0x6383), 0x2a },
	{ CCI_REG8(0x6384), 0x27 },
	{ CCI_REG8(0x6385), 0x1e },
	{ CCI_REG8(0x6386), 0x1e },
	{ CCI_REG8(0x638d), 0x28 },
	{ CCI_REG8(0x638e), 0x32 },
	{ CCI_REG8(0x638f), 0x28 },
	{ CCI_REG8(0x6390), 0x32 },
	{ CCI_REG8(0x6391), 0x29 },
	{ CCI_REG8(0x6392), 0x2a },
	{ CCI_REG8(0x6393), 0x28 },
	{ CCI_REG8(0x6394), 0x29 },
	{ CCI_REG8(0x6395), 0x1d },
	{ CCI_REG8(0x6396), 0x1f },
	{ CCI_REG8(0x639d), 0x28 },
	{ CCI_REG8(0x639e), 0x32 },
	{ CCI_REG8(0x639f), 0x28 },
	{ CCI_REG8(0x63a0), 0x32 },
	{ CCI_REG8(0x63a1), 0x2a },
	{ CCI_REG8(0x63a2), 0x2b },
	{ CCI_REG8(0x63a3), 0x28 },
	{ CCI_REG8(0x63a4), 0x2a },
	{ CCI_REG8(0x63a5), 0x1e },
	{ CCI_REG8(0x63a6), 0x20 },
	{ CCI_REG8(0x63ad), 0x28 },
	{ CCI_REG8(0x63ae), 0x32 },
	{ CCI_REG8(0x63af), 0x28 },
	{ CCI_REG8(0x63b0), 0x1e },
	{ CCI_REG8(0x63b4), 0x28 },
	{ CCI_REG8(0x63b5), 0x32 },
	{ CCI_REG8(0x63b6), 0x28 },
	{ CCI_REG8(0x63b7), 0x1e },
	{ CCI_REG8(0x63c9), 0x01 },
	{ CCI_REG8(0x63da), 0x03 },
	{ CCI_REG8(0x63de), 0x03 },
	{ CCI_REG8(0x63ea), 0x05 },
	{ CCI_REG8(0x63ed), 0x01 },
	{ CCI_REG8(0x63ee), 0x04 },
	{ CCI_REG8(0x63f8), 0x03 },
	{ CCI_REG8(0x63fa), 0x03 },
	{ CCI_REG8(0x63ff), 0x05 },
	{ CCI_REG8(0x6401), 0x04 },
	{ CCI_REG8(0x6403), 0x04 },
	{ CCI_REG8(0x6404), 0x03 },
	{ CCI_REG8(0x6405), 0x04 },
	{ CCI_REG8(0x6406), 0x03 },
	{ CCI_REG8(0x6407), 0x1f },
	{ CCI_REG8(0x6408), 0x0e },
	{ CCI_REG8(0x6409), 0x02 },
	{ CCI_REG8(0x640a), 0x1f },
	{ CCI_REG8(0x640b), 0x0d },
	{ CCI_REG8(0x640c), 0x1f },
	{ CCI_REG8(0x6417), 0x04 },
	{ CCI_REG8(0x6418), 0x03 },
	{ CCI_REG8(0x641a), 0x02 },
	{ CCI_REG8(0x641b), 0x07 },
	{ CCI_REG8(0x641c), 0x0d },
	{ CCI_REG8(0x6427), 0x04 },
	{ CCI_REG8(0x6428), 0x03 },
	{ CCI_REG8(0x642a), 0x03 },
	{ CCI_REG8(0x642b), 0x08 },
	{ CCI_REG8(0x642c), 0x0c },
	{ CCI_REG8(0x6437), 0x01 },
	{ CCI_REG8(0x643a), 0x02 },
	{ CCI_REG8(0x643b), 0x05 },
	{ CCI_REG8(0x643c), 0x07 },
	{ CCI_REG8(0x6446), 0x04 },
	{ CCI_REG8(0x644d), 0x01 },
	{ CCI_REG8(0x6499), 0x01 },
	{ CCI_REG8(0x649a), 0x01 },
	{ CCI_REG8(0x649b), 0x01 },
	{ CCI_REG8(0x649c), 0x01 },
	{ CCI_REG8(0x649d), 0x01 },
	{ CCI_REG8(0x649e), 0x01 },
	{ CCI_REG8(0x649f), 0x01 },
	{ CCI_REG8(0x64a0), 0x01 },
	{ CCI_REG8(0x64a7), 0x01 },
	{ CCI_REG8(0x64a8), 0x01 },
	{ CCI_REG8(0x64a9), 0x01 },
	{ CCI_REG8(0x64aa), 0x01 },
	{ CCI_REG8(0x64ab), 0x01 },
	{ CCI_REG8(0x64ac), 0x01 },
	{ CCI_REG8(0x64ad), 0x01 },
	{ CCI_REG8(0x64ae), 0x01 },
	{ CCI_REG8(0x64b5), 0x01 },
	{ CCI_REG8(0x64b6), 0x01 },
	{ CCI_REG8(0x64b7), 0x01 },
	{ CCI_REG8(0x64b8), 0x01 },
	{ CCI_REG8(0x64b9), 0x01 },
	{ CCI_REG8(0x64ba), 0x01 },
	{ CCI_REG8(0x64bb), 0x01 },
	{ CCI_REG8(0x64bc), 0x01 },
	{ CCI_REG8(0x64c3), 0x01 },
	{ CCI_REG8(0x64c4), 0x01 },
	{ CCI_REG8(0x64c5), 0x01 },
	{ CCI_REG8(0x64c6), 0x01 },
	{ CCI_REG8(0x64c7), 0x01 },
	{ CCI_REG8(0x64c8), 0x01 },
	{ CCI_REG8(0x64c9), 0x01 },
	{ CCI_REG8(0x64ca), 0x01 },
	{ CCI_REG8(0x64d1), 0x01 },
	{ CCI_REG8(0x64d2), 0x01 },
	{ CCI_REG8(0x64d3), 0x01 },
	{ CCI_REG8(0x64d4), 0x01 },
	{ CCI_REG8(0x64d8), 0x01 },
	{ CCI_REG8(0x64d9), 0x01 },
	{ CCI_REG8(0x64da), 0x01 },
	{ CCI_REG8(0x64db), 0x01 },
	{ CCI_REG8(0x651f), 0x1e },
	{ CCI_REG8(0x6520), 0x1e },
	{ CCI_REG8(0x6523), 0x1e },
	{ CCI_REG8(0x6524), 0x1e },
	{ CCI_REG8(0x6526), 0x1e },
	{ CCI_REG8(0x6528), 0x1e },
	{ CCI_REG8(0x6533), 0x1e },
	{ CCI_REG8(0x6534), 0x1e },
	{ CCI_REG8(0x6536), 0x1e },
	{ CCI_REG8(0x6538), 0x1e },
	{ CCI_REG8(0x666f), 0x15 },
	{ CCI_REG8(0x6670), 0x15 },
	{ CCI_REG8(0x6671), 0x15 },
	{ CCI_REG8(0x6672), 0x15 },
	{ CCI_REG8(0x6673), 0x15 },
	{ CCI_REG8(0x6674), 0x15 },
	{ CCI_REG8(0x6675), 0x15 },
	{ CCI_REG8(0x6676), 0x15 },
	{ CCI_REG8(0x6677), 0x15 },
	{ CCI_REG8(0x6678), 0x15 },
	{ CCI_REG8(0x6679), 0x15 },
	{ CCI_REG8(0x667a), 0x15 },
	{ CCI_REG8(0x667b), 0x15 },
	{ CCI_REG8(0x667c), 0x15 },
	{ CCI_REG8(0x6681), 0x15 },
	{ CCI_REG8(0x6682), 0x15 },
	{ CCI_REG8(0x6684), 0x15 },
	{ CCI_REG8(0x6686), 0x15 },
	{ CCI_REG8(0x6688), 0x15 },
	{ CCI_REG8(0x668a), 0x15 },
	{ CCI_REG8(0x668f), 0x15 },
	{ CCI_REG8(0x6690), 0x15 },
	{ CCI_REG8(0x6692), 0x15 },
	{ CCI_REG8(0x66bd), 0x0a },
	{ CCI_REG8(0x66be), 0x0a },
	{ CCI_REG8(0x66ca), 0x0a },
	{ CCI_REG8(0x66cb), 0x0a },
	{ CCI_REG8(0x66d4), 0x0a },
	{ CCI_REG8(0x66d7), 0x0a },
	{ CCI_REG8(0x6a35), 0x36 },
	{ CCI_REG8(0x6a36), 0x0e },
	{ CCI_REG8(0x6a37), 0x36 },
	{ CCI_REG8(0x6a38), 0x36 },
	{ CCI_REG8(0x6a39), 0x36 },
	{ CCI_REG8(0x6a3a), 0x36 },
	{ CCI_REG8(0x6a3b), 0x36 },
	{ CCI_REG8(0x6a3c), 0x0e },
	{ CCI_REG8(0x6a3d), 0x36 },
	{ CCI_REG8(0x6a3e), 0x36 },
	{ CCI_REG8(0x6a3f), 0x36 },
	{ CCI_REG8(0x6a40), 0x36 },
	{ CCI_REG8(0x6a41), 0x36 },
	{ CCI_REG8(0x7910), 0x00 },
	{ CCI_REG8(0x8502), 0x01 },
	{ CCI_REG8(0x8505), 0x00 },
	{ CCI_REG8(0x8605), 0x01 },
	{ CCI_REG8(0x9003), 0x02 },
	{ CCI_REG8(0x9200), 0x86 },
	{ CCI_REG8(0x9201), 0x08 },
	{ CCI_REG8(0x9202), 0x86 },
	{ CCI_REG8(0x9203), 0x09 },
	{ CCI_REG8(0xbc77), 0x4c },
	{ CCI_REG8(0xbc79), 0x7c },
	{ CCI_REG8(0xbc7a), 0x06 },
	{ CCI_REG8(0xbc7b), 0xe8 },
	{ CCI_REG8(0xbc7c), 0x06 },
	{ CCI_REG8(0xbc7d), 0x08 },
	{ CCI_REG8(0xbc7e), 0x0f },
	{ CCI_REG8(0xbc7f), 0x20 },
	{ CCI_REG8(0xbc80), 0x06 },
	{ CCI_REG8(0xbc81), 0xe8 },
	{ CCI_REG8(0xbc82), 0x06 },
	{ CCI_REG8(0xbc83), 0xe8 },
	{ CCI_REG8(0xbc84), 0x06 },
	{ CCI_REG8(0xbc85), 0xe8 },
	{ CCI_REG8(0xbc86), 0x06 },
	{ CCI_REG8(0xbc87), 0xe8 },
	{ CCI_REG8(0xa015), 0x90 },
	{ CCI_REG8(0xa016), 0x90 },
	{ CCI_REG8(0xa017), 0x34 },
	{ CCI_REG8(0xa018), 0xe8 },
	{ CCI_REG8(0xa019), 0x50 },
	{ CCI_REG8(0xa01a), 0x06 },
	{ CCI_REG8(0xa165), 0x10 },
	{ CCI_REG8(0xa16b), 0x10 },
	{ CCI_REG8(0xa171), 0x10 },
	{ CCI_REG8(0xa189), 0xc4 },
	{ CCI_REG8(0xa18f), 0xc4 },
	{ CCI_REG8(0xa195), 0xc4 },
	{ CCI_REG8(0xa19b), 0x0b },
	{ CCI_REG8(0xa1a1), 0x0b },
	{ CCI_REG8(0xa1a7), 0x0b },
	{ CCI_REG8(0xa337), 0x80 },
	{ CCI_REG8(0xa339), 0x80 },
	{ CCI_REG8(0xa33b), 0x80 },
	{ CCI_REG8(0xa51d), 0x10 },
	{ CCI_REG8(0xa520), 0x00 },
	{ CCI_REG8(0xa905), 0x40 },
	{ CCI_REG8(0xa90b), 0x00 },
	{ CCI_REG8(0xaa08), 0xff },
	{ CCI_REG8(0xaa0e), 0xff },
	{ CCI_REG8(0xab11), 0x40 },
	{ CCI_REG8(0xab1d), 0x40 },
	{ CCI_REG8(0xad01), 0x70 },
	{ CCI_REG8(0xad0d), 0x0b },
	{ CCI_REG8(0xad0e), 0x00 },
	{ CCI_REG8(0xad0f), 0x59 },
	{ CCI_REG8(0xad10), 0x00 },
	{ CCI_REG8(0xad11), 0x76 },
	{ CCI_REG8(0xad13), 0x11 },
	{ CCI_REG8(0xad15), 0xaf },
	{ CCI_REG8(0xad17), 0xe7 },
	{ CCI_REG8(0xad19), 0x0f },
	{ CCI_REG8(0xad1a), 0x00 },
	{ CCI_REG8(0xad1b), 0x69 },
	{ CCI_REG8(0xad1c), 0x00 },
	{ CCI_REG8(0xad1d), 0x89 },
	{ CCI_REG8(0x41d0), 0x00 },
};

/* 2592x1952: 2x2 binned full array, 4 lanes, 1356 Mbit/s (vendor preview) */
/* rubensimx596mipiraw_Sensor.h:395-481 (rubensimx596_preview_setting, 87 writes) */
static const struct cci_reg_sequence imx596_2592x1952_regs[] = {
	{ CCI_REG8(0x0114), 0x03 },	/* CSI_lane_mode: 4 lanes */
	{ CCI_REG8(0x0342), 0x12 },	/* line_length_pck */
	{ CCI_REG8(0x0343), 0x24 },
	{ CCI_REG8(0x0340), 0x18 },	/* frame_length_lines (control VBLANK) */
	{ CCI_REG8(0x0341), 0x95 },
	{ CCI_REG8(0x0344), 0x00 },	/* x_addr_start */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },	/* y_addr_start */
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x14 },	/* x_addr_end */
	{ CCI_REG8(0x0349), 0x3f },
	{ CCI_REG8(0x034a), 0x0f },	/* y_addr_end */
	{ CCI_REG8(0x034b), 0x3f },
	{ CCI_REG8(0x0220), 0x62 },	/* HDR_mode: 0x62, enable bit 0 clear */
	{ CCI_REG8(0x0221), 0x11 },	/* HDR_resolution_reduction */
	{ CCI_REG8(0x0222), 0x01 },	/* Exposure_ratio */
	{ CCI_REG8(0x0900), 0x01 },	/* binning_mode: on */
	{ CCI_REG8(0x0901), 0x22 },	/* binning_type: 2x2 */
	{ CCI_REG8(0x0902), 0x09 },	/* binning_weighting (Sony value) */
	{ CCI_REG8(0x30d8), 0x00 },
	{ CCI_REG8(0x3200), 0x41 },
	{ CCI_REG8(0x3201), 0x41 },
	{ CCI_REG8(0x0408), 0x00 },	/* digital_crop_x_offset */
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },	/* digital_crop_y_offset */
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0a },	/* digital_crop_image_width */
	{ CCI_REG8(0x040d), 0x20 },
	{ CCI_REG8(0x040e), 0x07 },	/* digital_crop_image_height */
	{ CCI_REG8(0x040f), 0xa0 },
	{ CCI_REG8(0x034c), 0x0a },	/* x_output_size */
	{ CCI_REG8(0x034d), 0x20 },
	{ CCI_REG8(0x034e), 0x07 },	/* y_output_size */
	{ CCI_REG8(0x034f), 0xa0 },
	{ CCI_REG8(0x0301), 0x05 },	/* vt_pix_clk_div */
	{ CCI_REG8(0x0303), 0x02 },	/* vt_sys_clk_div */
	{ CCI_REG8(0x0305), 0x03 },	/* pre_pll_clk_div */
	{ CCI_REG8(0x0306), 0x01 },	/* pll_multiplier */
	{ CCI_REG8(0x0307), 0x12 },
	{ CCI_REG8(0x030b), 0x01 },	/* op_sys_clk_div */
	{ CCI_REG8(0x030d), 0x04 },	/* op_pre_pll_clk_div */
	{ CCI_REG8(0x030e), 0x00 },	/* op_pll_multiplier */
	{ CCI_REG8(0x030f), 0xe2 },
	{ CCI_REG8(0x32d3), 0x01 },
	{ CCI_REG8(0x32d5), 0x00 },
	{ CCI_REG8(0x32d6), 0x00 },
	{ CCI_REG8(0x4000), 0x06 },
	{ CCI_REG8(0x4001), 0x04 },
	{ CCI_REG8(0x40a0), 0x04 },
	{ CCI_REG8(0x40a1), 0x88 },
	{ CCI_REG8(0x40a4), 0x04 },
	{ CCI_REG8(0x40a5), 0x88 },
	{ CCI_REG8(0x40b8), 0x01 },
	{ CCI_REG8(0x40b9), 0x05 },
	{ CCI_REG8(0x41a4), 0x00 },
	{ CCI_REG8(0x0202), 0x18 },	/* coarse_integration_time (control EXPOSURE) */
	{ CCI_REG8(0x0203), 0x7d },
	{ CCI_REG8(0x0224), 0x01 },	/* Direct_short_integration_time */
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3116), 0x01 },
	{ CCI_REG8(0x3117), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },	/* analog_gain_code_global (control ANALOGUE_GAIN) */
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },	/* digital gain (control DIGITAL_GAIN) */
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },	/* Short_analog_gain_global */
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },	/* Short_digital_gain_global */
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x3118), 0x00 },
	{ CCI_REG8(0x3119), 0x00 },
	{ CCI_REG8(0x311a), 0x01 },
	{ CCI_REG8(0x311b), 0x00 },
	{ CCI_REG8(0x0808), 0x02 },	/* phy_ctrl: manual D-PHY timing */
	{ CCI_REG8(0x080b), 0x9f },	/* tclk_post_ex[7:0] */
	{ CCI_REG8(0x080d), 0x57 },	/* ths_prepare_ex[7:0] */
	{ CCI_REG8(0x080e), 0x00 },	/* ths_zero_min_ex */
	{ CCI_REG8(0x080f), 0x9f },
	{ CCI_REG8(0x0810), 0x00 },	/* ths_trail_ex */
	{ CCI_REG8(0x0811), 0x57 },
	{ CCI_REG8(0x0813), 0x57 },	/* tclk_trail_min_ex[7:0] */
	{ CCI_REG8(0x0815), 0x57 },	/* tclk_prepare_ex[7:0] */
	{ CCI_REG8(0x0816), 0x01 },	/* tclk_zero_ex */
	{ CCI_REG8(0x0817), 0x6f },
	{ CCI_REG8(0x0819), 0x47 },	/* tlpx_ex[7:0] */
	{ CCI_REG8(0x0825), 0x8f },
	{ CCI_REG8(0x0827), 0x0f },
};

/* 1920x1080: 2x2 binned 3840x2160 centre (vendor hs_video, 120 fps table) */
/* rubensimx596mipiraw_Sensor.h:582-668 (rubensimx596_hs_video_setting, 87 writes) */
static const struct cci_reg_sequence imx596_1920x1080_regs[] = {
	{ CCI_REG8(0x0114), 0x03 },	/* CSI_lane_mode: 4 lanes */
	{ CCI_REG8(0x0342), 0x12 },	/* line_length_pck */
	{ CCI_REG8(0x0343), 0x24 },
	{ CCI_REG8(0x0340), 0x06 },	/* frame_length_lines (control VBLANK) */
	{ CCI_REG8(0x0341), 0x25 },
	{ CCI_REG8(0x0344), 0x00 },	/* x_addr_start */
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x03 },	/* y_addr_start */
	{ CCI_REG8(0x0347), 0x60 },
	{ CCI_REG8(0x0348), 0x14 },	/* x_addr_end */
	{ CCI_REG8(0x0349), 0x3f },
	{ CCI_REG8(0x034a), 0x0b },	/* y_addr_end */
	{ CCI_REG8(0x034b), 0xdf },
	{ CCI_REG8(0x0220), 0x62 },	/* HDR_mode: 0x62, enable bit 0 clear */
	{ CCI_REG8(0x0221), 0x11 },	/* HDR_resolution_reduction */
	{ CCI_REG8(0x0222), 0x01 },	/* Exposure_ratio */
	{ CCI_REG8(0x0900), 0x01 },	/* binning_mode: on */
	{ CCI_REG8(0x0901), 0x22 },	/* binning_type: 2x2 */
	{ CCI_REG8(0x0902), 0x09 },	/* binning_weighting (Sony value) */
	{ CCI_REG8(0x30d8), 0x00 },
	{ CCI_REG8(0x3200), 0x41 },
	{ CCI_REG8(0x3201), 0x41 },
	{ CCI_REG8(0x0408), 0x01 },	/* digital_crop_x_offset */
	{ CCI_REG8(0x0409), 0x50 },
	{ CCI_REG8(0x040a), 0x00 },	/* digital_crop_y_offset */
	{ CCI_REG8(0x040b), 0x04 },
	{ CCI_REG8(0x040c), 0x07 },	/* digital_crop_image_width */
	{ CCI_REG8(0x040d), 0x80 },
	{ CCI_REG8(0x040e), 0x04 },	/* digital_crop_image_height */
	{ CCI_REG8(0x040f), 0x38 },
	{ CCI_REG8(0x034c), 0x07 },	/* x_output_size */
	{ CCI_REG8(0x034d), 0x80 },
	{ CCI_REG8(0x034e), 0x04 },	/* y_output_size */
	{ CCI_REG8(0x034f), 0x38 },
	{ CCI_REG8(0x0301), 0x05 },	/* vt_pix_clk_div */
	{ CCI_REG8(0x0303), 0x02 },	/* vt_sys_clk_div */
	{ CCI_REG8(0x0305), 0x03 },	/* pre_pll_clk_div */
	{ CCI_REG8(0x0306), 0x01 },	/* pll_multiplier */
	{ CCI_REG8(0x0307), 0x12 },
	{ CCI_REG8(0x030b), 0x01 },	/* op_sys_clk_div */
	{ CCI_REG8(0x030d), 0x04 },	/* op_pre_pll_clk_div */
	{ CCI_REG8(0x030e), 0x00 },	/* op_pll_multiplier */
	{ CCI_REG8(0x030f), 0xe2 },
	{ CCI_REG8(0x32d3), 0x01 },
	{ CCI_REG8(0x32d5), 0x00 },
	{ CCI_REG8(0x32d6), 0x00 },
	{ CCI_REG8(0x4000), 0x06 },
	{ CCI_REG8(0x4001), 0x04 },
	{ CCI_REG8(0x40a0), 0x03 },
	{ CCI_REG8(0x40a1), 0x70 },
	{ CCI_REG8(0x40a4), 0x00 },
	{ CCI_REG8(0x40a5), 0x14 },
	{ CCI_REG8(0x40b8), 0x04 },
	{ CCI_REG8(0x40b9), 0x7e },
	{ CCI_REG8(0x41a4), 0x00 },
	{ CCI_REG8(0x0202), 0x06 },	/* coarse_integration_time (control EXPOSURE) */
	{ CCI_REG8(0x0203), 0x0d },
	{ CCI_REG8(0x0224), 0x01 },	/* Direct_short_integration_time */
	{ CCI_REG8(0x0225), 0xf4 },
	{ CCI_REG8(0x3116), 0x01 },
	{ CCI_REG8(0x3117), 0xf4 },
	{ CCI_REG8(0x0204), 0x00 },	/* analog_gain_code_global (control ANALOGUE_GAIN) */
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },	/* digital gain (control DIGITAL_GAIN) */
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },	/* Short_analog_gain_global */
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x0218), 0x01 },	/* Short_digital_gain_global */
	{ CCI_REG8(0x0219), 0x00 },
	{ CCI_REG8(0x3118), 0x00 },
	{ CCI_REG8(0x3119), 0x00 },
	{ CCI_REG8(0x311a), 0x01 },
	{ CCI_REG8(0x311b), 0x00 },
	{ CCI_REG8(0x0808), 0x02 },	/* phy_ctrl: manual D-PHY timing */
	{ CCI_REG8(0x080b), 0x9f },	/* tclk_post_ex[7:0] */
	{ CCI_REG8(0x080d), 0x57 },	/* ths_prepare_ex[7:0] */
	{ CCI_REG8(0x080e), 0x00 },	/* ths_zero_min_ex */
	{ CCI_REG8(0x080f), 0x9f },
	{ CCI_REG8(0x0810), 0x00 },	/* ths_trail_ex */
	{ CCI_REG8(0x0811), 0x57 },
	{ CCI_REG8(0x0813), 0x57 },	/* tclk_trail_min_ex[7:0] */
	{ CCI_REG8(0x0815), 0x57 },	/* tclk_prepare_ex[7:0] */
	{ CCI_REG8(0x0816), 0x01 },	/* tclk_zero_ex */
	{ CCI_REG8(0x0817), 0x6f },
	{ CCI_REG8(0x0819), 0x47 },	/* tlpx_ex[7:0] */
	{ CCI_REG8(0x0825), 0x8f },
	{ CCI_REG8(0x0827), 0x0f },
};

struct imx596_mode {
	u32 width;
	u32 height;
	/* Part of the pixel array that the output image covers */
	struct v4l2_rect crop;
	/* Frame length limits in lines; the default gives 30 fps */
	u32 frame_length_min;
	u32 frame_length_def;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const struct imx596_mode imx596_modes[] = {
	{
		.width = 2592,
		.height = 1952,
		.crop = {
			.left = 0,
			.top = 0,
			.width = 5184,
			.height = 3904,
		},
		.frame_length_min = IMX596_FRAME_LENGTH_30FPS,
		.frame_length_def = IMX596_FRAME_LENGTH_30FPS,
		.regs = imx596_2592x1952_regs,
		.num_regs = ARRAY_SIZE(imx596_2592x1952_regs),
	},
	{
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = 672,
			.top = 872,
			.width = 3840,
			.height = 2160,
		},
		/* The vendor table runs at 120 fps (1573 lines). */
		.frame_length_min = 1573,
		.frame_length_def = IMX596_FRAME_LENGTH_30FPS,
		.regs = imx596_1920x1080_regs,
		.num_regs = ARRAY_SIZE(imx596_1920x1080_regs),
	},
};

struct imx596 {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	struct regulator *vcam;		/* optional */
	struct regulator *dovdd;
	struct regulator *avdd;
	struct regulator *dvdd;
	struct gpio_desc *reset_gpio;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	/* Cluster: hflip and vflip must stay adjacent. */
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
};

static inline struct imx596 *to_imx596(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx596, sd);
}

static const struct imx596_mode *
imx596_find_mode(const struct v4l2_mbus_framefmt *fmt)
{
	return v4l2_find_nearest_size(imx596_modes, ARRAY_SIZE(imx596_modes),
				      width, height, fmt->width, fmt->height);
}

static u32 imx596_mbus_code(bool hflip, bool vflip)
{
	return imx596_mbus_codes[(vflip ? 2 : 0) | (hflip ? 1 : 0)];
}

/* Bayer order of the applied flips; a tried or failed value is not applied. */
static u32 imx596_cur_mbus_code(struct imx596 *imx596)
{
	return imx596_mbus_code(imx596->hflip->cur.val, imx596->vflip->cur.val);
}

static void imx596_fill_format(struct imx596 *imx596,
			       const struct imx596_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = imx596_cur_mbus_code(imx596);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

/* Vendor gain2reg(): code = 1024 - 1024 * 1024 / gain, 0 (1x) to 960 (16x) */
static u32 imx596_gain_to_code(u32 gain)
{
	return IMX596_GAIN_UNIT - IMX596_GAIN_UNIT * IMX596_GAIN_UNIT / gain;
}

static int imx596_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx596 *imx596 = container_of(ctrl->handler, struct imx596,
					     ctrls);
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 code = 0;
	int ret = 0;
	int hold;

	/* The control handler lock is the state lock. */
	state = v4l2_subdev_get_locked_active_state(&imx596->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* The exposure must end 24 lines before the frame does. */
		ret = __v4l2_ctrl_modify_range(imx596->exposure,
					       imx596->exposure->minimum,
					       fmt->height + ctrl->val -
					       IMX596_EXPOSURE_MARGIN,
					       imx596->exposure->step,
					       imx596->exposure->default_value);
		if (ret)
			return ret;
		break;
	case V4L2_CID_HFLIP:
		/* The flips change the Bayer order of the active format. */
		code = imx596_mbus_code(imx596->hflip->val, imx596->vflip->val);
		break;
	}

	/* Write only while powered; enable_streams applies all controls. */
	if (pm_runtime_get_if_active(imx596->dev) <= 0) {
		if (code)
			fmt->code = code;
		return 0;
	}

	/*
	 * As the vendor driver does for the frame length, the exposure and the
	 * gain: each update inside a grouped parameter hold, so that a
	 * multi-byte value takes effect in one frame.
	 */
	cci_write(imx596->regmap, IMX596_REG_GROUP_HOLD, 1, &ret);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(imx596->regmap, IMX596_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(imx596->regmap, IMX596_REG_FRAME_LENGTH,
			  fmt->height + ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(imx596->regmap, IMX596_REG_ANALOGUE_GAIN,
			  imx596_gain_to_code(ctrl->val), &ret);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		cci_write(imx596->regmap, IMX596_REG_DIGITAL_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_HFLIP:
		/* Cluster master: also applies V4L2_CID_VFLIP. */
		cci_write(imx596->regmap, IMX596_REG_ORIENTATION,
			  (imx596->hflip->val ? IMX596_HFLIP : 0) |
			  (imx596->vflip->val ? IMX596_VFLIP : 0), &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(imx596->regmap, IMX596_REG_TEST_PATTERN, ctrl->val,
			  &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	/* Release the hold even after a failed write. */
	hold = cci_write(imx596->regmap, IMX596_REG_GROUP_HOLD, 0, NULL);
	if (!ret)
		ret = hold;

	/* The new Bayer order applies once the flips are written. */
	if (!ret && code)
		fmt->code = code;

	pm_runtime_put_autosuspend(imx596->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx596_ctrl_ops = {
	.s_ctrl = imx596_set_ctrl,
};

static int imx596_init_controls(struct imx596 *imx596)
{
	const struct imx596_mode *mode = &imx596_modes[0];
	struct v4l2_ctrl_handler *hdl = &imx596->ctrls;
	const struct v4l2_ctrl_ops *ops = &imx596_ctrl_ops;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	u32 hblank;
	int ret;

	ret = v4l2_fwnode_device_parse(imx596->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 12);

	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ,
				      ARRAY_SIZE(imx596_link_freq_menu) - 1, 0,
				      imx596_link_freq_menu);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, IMX596_PIXEL_RATE,
			  IMX596_PIXEL_RATE, 1, IMX596_PIXEL_RATE);

	hblank = IMX596_LINE_LENGTH - mode->width;
	imx596->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank,
					   hblank, 1, hblank);
	if (imx596->hblank)
		imx596->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * The handler setup in enable_streams writes the controls in creation
	 * order: frame length before exposure and gain, as the vendor does.
	 */
	imx596->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					   mode->frame_length_min - mode->height,
					   IMX596_FRAME_LENGTH_MAX - mode->height,
					   1, mode->frame_length_def - mode->height);

	imx596->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					     IMX596_EXPOSURE_MIN,
					     mode->frame_length_def -
					     IMX596_EXPOSURE_MARGIN, 1,
					     IMX596_EXPOSURE_DEFAULT);

	/* 1x as in the mode tables; the vendor control starts at 4x. */
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN, IMX596_GAIN_MIN,
			  IMX596_GAIN_MAX, 1, IMX596_GAIN_MIN);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN, IMX596_DGAIN_MIN,
			  IMX596_DGAIN_MAX, 1, IMX596_DGAIN_DEFAULT);

	imx596->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	imx596->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx596->hflip && imx596->vflip) {
		imx596->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		imx596->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
		v4l2_ctrl_cluster(2, &imx596->hflip);
	}

	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx596_test_pattern_menu) - 1,
				     0, 0, imx596_test_pattern_menu);

	v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	imx596->sd.ctrl_handler = hdl;

	return 0;
}

/* Limits and defaults of the timing controls; the state already has @mode. */
static int imx596_update_controls(struct imx596 *imx596,
				  const struct imx596_mode *mode)
{
	u32 hblank = IMX596_LINE_LENGTH - mode->width;
	int ret;

	ret = __v4l2_ctrl_modify_range(imx596->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx596->vblank,
				       mode->frame_length_min - mode->height,
				       IMX596_FRAME_LENGTH_MAX - mode->height,
				       1, mode->frame_length_def - mode->height);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl(imx596->vblank,
				 mode->frame_length_def - mode->height);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(imx596->exposure, IMX596_EXPOSURE_MIN,
				       mode->frame_length_def -
				       IMX596_EXPOSURE_MARGIN, 1,
				       IMX596_EXPOSURE_DEFAULT);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(imx596->exposure, IMX596_EXPOSURE_DEFAULT);
}

/*
 * Stop the transmitter. If the standby write fails, hold the sensor in reset
 * instead; it stays unusable until runtime suspend powers it down and the
 * next runtime resume initialises it again.
 */
static int imx596_stop(struct imx596 *imx596)
{
	int ret;

	ret = cci_write(imx596->regmap, IMX596_REG_MODE_SELECT,
			IMX596_MODE_STANDBY, NULL);
	if (!ret || !imx596->reset_gpio)
		return ret;

	dev_err(imx596->dev, "standby failed (%d), holding the sensor in reset\n",
		ret);
	ret = gpiod_set_value_cansleep(imx596->reset_gpio, 1);
	usleep_range(2000, 3000);

	return ret;
}

static int imx596_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct imx596 *imx596 = to_imx596(sd);
	const struct imx596_mode *mode;
	int ret;

	/* Normally the receiver's runtime PM link has resumed the sensor. */
	ret = pm_runtime_resume_and_get(imx596->dev);
	if (ret)
		return ret;

	mode = imx596_find_mode(v4l2_subdev_state_get_format(state, 0));
	ret = cci_multi_reg_write(imx596->regmap, mode->regs, mode->num_regs,
				  NULL);
	if (ret)
		goto err_rpm_put;

	/* Frame length, exposure and gains before stream-on apply to frame 0. */
	ret = __v4l2_ctrl_handler_setup(&imx596->ctrls);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx596->regmap, IMX596_REG_MODE_SELECT,
			IMX596_MODE_STREAMING, NULL);
	if (ret) {
		/* A failed write may still have started the transmitter. */
		imx596_stop(imx596);
		goto err_rpm_put;
	}

	__v4l2_ctrl_grab(imx596->hflip, true);
	__v4l2_ctrl_grab(imx596->vflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put_autosuspend(imx596->dev);
	return ret;
}

static int imx596_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct imx596 *imx596 = to_imx596(sd);
	int ret;

	ret = imx596_stop(imx596);
	if (ret)
		dev_err(imx596->dev, "failed to stop streaming: %d\n", ret);

	__v4l2_ctrl_grab(imx596->hflip, false);
	__v4l2_ctrl_grab(imx596->vflip, false);

	pm_runtime_put_autosuspend(imx596->dev);

	/*
	 * The reference is dropped: report success so that the stream is
	 * marked disabled, as the core does for s_stream failures.
	 */
	return 0;
}

static int imx596_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = imx596_cur_mbus_code(to_imx596(sd));

	return 0;
}

static int imx596_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx596_mode *mode;

	if (fse->index >= ARRAY_SIZE(imx596_modes) ||
	    fse->code != imx596_cur_mbus_code(to_imx596(sd)))
		return -EINVAL;

	mode = &imx596_modes[fse->index];
	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static int imx596_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx596 *imx596 = to_imx596(sd);
	const struct imx596_mode *mode;

	/* The mode is programmed at stream start; it cannot change while on. */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	mode = imx596_find_mode(&fmt->format);
	imx596_fill_format(imx596, mode, &fmt->format);

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		return imx596_update_controls(imx596, mode);

	return 0;
}

static int imx596_get_selection(struct v4l2_subdev *sd,
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
		sel->r = imx596_native_area;
		return 0;
	default:
		return -EINVAL;
	}
}

static int imx596_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	u32 code;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	code = v4l2_subdev_state_get_format(state, 0)->code;
	v4l2_subdev_unlock_state(state);

	/* Image data only: the modes have no PDAF, HDR or embedded data stream. */
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].stream = 0;
	fd->entry[0].pixelcode = code;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW10;

	return 0;
}

static int imx596_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = IMX596_LINK_FREQ;
	config->bus.mipi_csi2.num_data_lanes = IMX596_DATA_LANES;

	return 0;
}

static int imx596_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	const struct imx596_mode *mode = &imx596_modes[0];

	imx596_fill_format(to_imx596(sd), mode,
			   v4l2_subdev_state_get_format(state, 0));
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	return 0;
}

static const struct v4l2_subdev_video_ops imx596_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx596_pad_ops = {
	.enum_mbus_code = imx596_enum_mbus_code,
	.enum_frame_size = imx596_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx596_set_fmt,
	.get_selection = imx596_get_selection,
	.get_frame_desc = imx596_get_frame_desc,
	.get_mbus_config = imx596_get_mbus_config,
	.enable_streams = imx596_enable_streams,
	.disable_streams = imx596_disable_streams,
};

static const struct v4l2_subdev_ops imx596_subdev_ops = {
	.video = &imx596_video_ops,
	.pad = &imx596_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx596_internal_ops = {
	.init_state = imx596_init_state,
};

/* Order and delays of the vendor power sequence */
static int imx596_power_on(struct imx596 *imx596)
{
	int ret;

	gpiod_set_value_cansleep(imx596->reset_gpio, 1);
	usleep_range(1000, 2000);

	/* One at a time, in the vendor order */
	if (imx596->vcam) {
		ret = regulator_enable(imx596->vcam);
		if (ret)
			return ret;
		usleep_range(1000, 2000);
	}

	ret = regulator_enable(imx596->dovdd);
	if (ret)
		goto err_vcam;
	usleep_range(1000, 2000);

	ret = regulator_enable(imx596->avdd);
	if (ret)
		goto err_dovdd;
	usleep_range(1000, 2000);

	ret = regulator_enable(imx596->dvdd);
	if (ret)
		goto err_avdd;
	usleep_range(1000, 2000);

	ret = clk_prepare_enable(imx596->xclk);
	if (ret)
		goto err_dvdd;

	/* Connect MCLK to the pad; a no-op without pinctrl states. */
	ret = pinctrl_pm_select_default_state(imx596->dev);
	if (ret)
		goto err_pins;
	usleep_range(1000, 2000);

	gpiod_set_value_cansleep(imx596->reset_gpio, 0);
	usleep_range(2000, 3000);

	return 0;

err_pins:
	/* Undo a partially applied default state. */
	pinctrl_pm_select_sleep_state(imx596->dev);
	clk_disable_unprepare(imx596->xclk);
err_dvdd:
	regulator_disable(imx596->dvdd);
	usleep_range(1000, 2000);
err_avdd:
	regulator_disable(imx596->avdd);
	usleep_range(1000, 2000);
err_dovdd:
	regulator_disable(imx596->dovdd);
	usleep_range(1000, 2000);
err_vcam:
	if (imx596->vcam) {
		regulator_disable(imx596->vcam);
		usleep_range(1000, 2000);
	}
	return ret;
}

/* Reverse order; the last wait is the minimum off time. */
static void imx596_power_off(struct imx596 *imx596)
{
	gpiod_set_value_cansleep(imx596->reset_gpio, 1);
	usleep_range(2000, 3000);

	/*
	 * Park the MCLK pad as a low GPIO before gating the clock: other
	 * consumers may keep the clock running.
	 */
	pinctrl_pm_select_sleep_state(imx596->dev);
	usleep_range(1000, 2000);
	clk_disable_unprepare(imx596->xclk);

	regulator_disable(imx596->dvdd);
	usleep_range(1000, 2000);
	regulator_disable(imx596->avdd);
	usleep_range(1000, 2000);
	regulator_disable(imx596->dovdd);
	usleep_range(1000, 2000);

	if (imx596->vcam) {
		regulator_disable(imx596->vcam);
		usleep_range(1000, 2000);
	}
}

static int imx596_runtime_resume(struct device *dev)
{
	struct imx596 *imx596 = to_imx596(dev_get_drvdata(dev));
	int ret;

	ret = imx596_power_on(imx596);
	if (ret)
		goto err;

	ret = cci_multi_reg_write(imx596->regmap, imx596_init_regs,
				  ARRAY_SIZE(imx596_init_regs), NULL);
	if (ret) {
		imx596_power_off(imx596);
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

static int imx596_runtime_suspend(struct device *dev)
{
	imx596_power_off(to_imx596(dev_get_drvdata(dev)));

	return 0;
}

/*
 * Log the power-on values that the register tables rely on (RAW10 data
 * format, PLL mode, digital gain mode) and the limits the sensor reports.
 * Reads only.
 */
static void imx596_log_info(struct imx596 *imx596)
{
	u64 rev, format, lanes, pll, ag_min, ag_max, m0, c0, m1, c1;
	u64 exp_min, exp_margin, dg_min, dg_max, dg_mode;
	struct regmap *regmap = imx596->regmap;
	int ret = 0;

	cci_read(regmap, IMX596_REG_REVISION, &rev, &ret);
	cci_read(regmap, IMX596_REG_CSI_DATA_FORMAT, &format, &ret);
	cci_read(regmap, IMX596_REG_CSI_LANE_MODE, &lanes, &ret);
	cci_read(regmap, IMX596_REG_PLL_MODE, &pll, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_MIN, &ag_min, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_MAX, &ag_max, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_M0, &m0, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_C0, &c0, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_M1, &m1, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_AGAIN_C1, &c1, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_EXPOSURE_MIN, &exp_min, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_EXPOSURE_MARGIN, &exp_margin, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_DGAIN_MIN, &dg_min, &ret);
	cci_read(regmap, IMX596_REG_LIMIT_DGAIN_MAX, &dg_max, &ret);
	cci_read(regmap, IMX596_REG_DGAIN_MODE, &dg_mode, &ret);
	if (ret) {
		dev_info(imx596->dev, "chip id 0x%04x\n", IMX596_CHIP_ID);
		return;
	}

	dev_info(imx596->dev,
		 "chip id 0x%04x revision 0x%02llx, data format 0x%04llx, lane mode %llu, PLL mode %llu\n",
		 IMX596_CHIP_ID, rev, format, lanes, pll);
	dev_info(imx596->dev,
		 "analogue gain code %llu-%llu (m0 %d c0 %d m1 %d c1 %d), exposure min %llu margin %llu, digital gain 0x%04llx-0x%04llx mode 0x%02llx\n",
		 ag_min, ag_max, (s16)m0, (s16)c0, (s16)m1, (s16)c1,
		 exp_min, exp_margin, dg_min, dg_max, dg_mode);
}

static int imx596_identify(struct imx596 *imx596)
{
	u64 id;
	int ret;

	/* Read right after reset, before any write */
	ret = cci_read(imx596->regmap, IMX596_REG_CHIP_ID, &id, NULL);
	if (ret)
		return dev_err_probe(imx596->dev, ret, "failed to read chip id\n");

	if (id != IMX596_CHIP_ID)
		return dev_err_probe(imx596->dev, -ENODEV,
				     "chip id mismatch: 0x%04llx\n", id);

	imx596_log_info(imx596);

	return 0;
}

static int imx596_parse_fwnode(struct imx596 *imx596)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned long link_freq_bitmap;
	int ret;

	/* Also accept an endpoint that has no available remote. */
	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(imx596->dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT |
					     FWNODE_GRAPH_DEVICE_DISABLED);
	if (!ep)
		return dev_err_probe(imx596->dev, -ENXIO, "missing endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(imx596->dev, ret, "invalid endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != IMX596_DATA_LANES) {
		ret = dev_err_probe(imx596->dev, -EINVAL,
				    "only four data lanes are supported\n");
		goto out;
	}

	/* The link frequency the modes need must be allowed by the board. */
	ret = v4l2_link_freq_to_bitmap(imx596->dev, bus_cfg.link_frequencies,
				       bus_cfg.nr_of_link_frequencies,
				       imx596_link_freq_menu,
				       ARRAY_SIZE(imx596_link_freq_menu),
				       &link_freq_bitmap);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static void imx596_disable_runtime_pm(struct imx596 *imx596)
{
	pm_runtime_disable(imx596->dev);
	if (!pm_runtime_status_suspended(imx596->dev)) {
		imx596_power_off(imx596);
		pm_runtime_set_suspended(imx596->dev);
	}
	pm_runtime_dont_use_autosuspend(imx596->dev);
}

static int imx596_get_supplies(struct imx596 *imx596)
{
	struct device *dev = imx596->dev;

	/* A board-specific 1.2 V module rail, switched first when present */
	imx596->vcam = devm_regulator_get_optional(dev, "vcam");
	if (IS_ERR(imx596->vcam)) {
		if (PTR_ERR(imx596->vcam) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(imx596->vcam),
					     "failed to get vcam\n");

		dev_info(dev, "no vcam supply\n");
		imx596->vcam = NULL;
	}

	imx596->dovdd = devm_regulator_get(dev, "dovdd");
	if (IS_ERR(imx596->dovdd))
		return dev_err_probe(dev, PTR_ERR(imx596->dovdd),
				     "failed to get DOVDD\n");

	imx596->avdd = devm_regulator_get(dev, "avdd");
	if (IS_ERR(imx596->avdd))
		return dev_err_probe(dev, PTR_ERR(imx596->avdd),
				     "failed to get AVDD\n");

	imx596->dvdd = devm_regulator_get(dev, "dvdd");
	if (IS_ERR(imx596->dvdd))
		return dev_err_probe(dev, PTR_ERR(imx596->dvdd),
				     "failed to get DVDD\n");

	return 0;
}

static int imx596_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx596 *imx596;
	unsigned long rate;
	int ret;

	imx596 = devm_kzalloc(dev, sizeof(*imx596), GFP_KERNEL);
	if (!imx596)
		return -ENOMEM;

	imx596->dev = dev;
	v4l2_i2c_subdev_init(&imx596->sd, client, &imx596_subdev_ops);

	ret = imx596_parse_fwnode(imx596);
	if (ret)
		return ret;

	imx596->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx596->regmap))
		return dev_err_probe(dev, PTR_ERR(imx596->regmap),
				     "failed to init CCI\n");

	/* The board selects a 24 MHz parent; the rate is checked, never set. */
	imx596->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx596->xclk))
		return dev_err_probe(dev, PTR_ERR(imx596->xclk),
				     "failed to get MCLK\n");

	rate = clk_get_rate(imx596->xclk);
	if (rate != IMX596_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "MCLK must be 24 MHz, got %lu Hz\n", rate);

	ret = imx596_get_supplies(imx596);
	if (ret)
		return ret;

	/* Asserted (XCLR low) whenever the sensor is not powered */
	imx596->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx596->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx596->reset_gpio),
				     "failed to get reset GPIO\n");

	/* Identify with the sensor powered only for the register reads. */
	ret = imx596_power_on(imx596);
	if (ret)
		return dev_err_probe(dev, ret, "failed to power on\n");

	ret = imx596_identify(imx596);
	imx596_power_off(imx596);
	if (ret)
		return ret;

	ret = imx596_init_controls(imx596);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init controls\n");

	imx596->sd.state_lock = imx596->ctrls.lock;
	imx596->sd.internal_ops = &imx596_internal_ops;
	imx596->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx596->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx596->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx596->sd.entity, 1, &imx596->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&imx596->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init subdev\n");
		goto err_entity;
	}

	/* Suspended until a user resumes it; resume writes the init table. */
	pm_runtime_set_autosuspend_delay(dev, IMX596_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx596->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	return 0;

err_pm:
	imx596_disable_runtime_pm(imx596);
	v4l2_subdev_cleanup(&imx596->sd);
err_entity:
	media_entity_cleanup(&imx596->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&imx596->ctrls);
	return ret;
}

static void imx596_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx596 *imx596 = to_imx596(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx596->ctrls);

	imx596_disable_runtime_pm(imx596);
}

static DEFINE_RUNTIME_DEV_PM_OPS(imx596_pm_ops, imx596_runtime_suspend,
				 imx596_runtime_resume, NULL);

static const struct of_device_id imx596_of_match[] = {
	{ .compatible = "sony,imx596" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx596_of_match);

static struct i2c_driver imx596_i2c_driver = {
	.driver = {
		.name = "imx596",
		.of_match_table = imx596_of_match,
		.pm = pm_ptr(&imx596_pm_ops),
		/* Unbinding would remove the sensor under a running stream. */
		.suppress_bind_attrs = true,
	},
	.probe = imx596_probe,
	.remove = imx596_remove,
};
module_i2c_driver(imx596_i2c_driver);

MODULE_DESCRIPTION("Sony IMX596 image sensor driver");
MODULE_LICENSE("GPL");
