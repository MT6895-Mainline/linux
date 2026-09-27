// SPDX-License-Identifier: GPL-2.0
/*
 * OPLUS 21143 (OPPO K10 / OnePlus Ace Racing "qqcandy", MT6895) Tianma
 * NT36672C VDO-mode TDDI LCD panel driver.
 *
 * Ported from the OPPO 5.10 vendor kernel
 *   drivers/gpu/drm/panel/oplus21143_tianma_nt36672c_dsi_vdo.c
 * to the mainline 6.18 DRM panel + mediatek_v2 mtk_panel_ext model.
 *
 * Porting notes vs the 5.10 original:
 *  - Init sequences, lane_swap tables, DSC params and all seven display
 *    modes are byte-exact copies of upstream.
 *  - MIPI_DSI_MODE_EOT_PACKET no longer exists (renamed/flipped to
 *    MIPI_DSI_MODE_NO_EOT_PACKET); the flag was dropped entirely, same
 *    as the 6.12 port of this driver.
 *  - The vendor _lcm_i2c_write_bytes(0x03,0x43) panel-power write targets
 *    the SM5109 bias IC (stock driver lcm_sm5109_i2c, DT
 *    "mediatek,I2C_LCD_BIAS"); this tree has no SM5109 driver, so the
 *    bias IC runs on its stock power-on defaults. Do NOT redirect this
 *    write at any other I2C backlight IC: the register maps differ and a
 *    misdirected write misconfigures the bias rails. AVDD/AVEE gating
 *    uses plain GPIOs (pio120/pio119) through the "bias" gpio-index
 *    property.
 *  - vufsldo regulator handling, get_boot_mode()/mtk_boot_common,
 *    oplus device_info/ofp registration, touch gesture notifier and
 *    msm disp notify chains have no equivalent in this tree and were
 *    stubbed out; power sequencing uses the bias GPIOs only.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/gpio/consumer.h>

#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/of_graph.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#define REGFLAG_CMD		0xFFFA
#define REGFLAG_DELAY		0xFFFC
#define REGFLAG_UDELAY		0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_MAX		4095
#define BRIGHTNESS_HALF		2047
#define MAX_NORMAL_BRIGHTNESS	3276

static unsigned int esd_brightness = 1023;
static u32 flag_hbm;
static int bl_gamma;
static unsigned int cabc_lastlevel = 1;
static unsigned int last_brightness;
static int flag_dimming;

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[128];
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD, 3, {0x51, 0x00, 0x00}},
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	{REGFLAG_CMD, 3, {0x51, 0x0f, 0xff}},
};

struct jdi {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *leden_gpio;
	struct regulator *vufsldo;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_led_en;
	struct gpio_desc *bias_pos, *bias_neg;
	bool prepared;
	bool enabled;
	bool hbm_en;
	int error;
};

#define jdi_dcs_write_seq_static(ctx, seq...)				\
	({								\
		static const u8 d[] = { seq };				\
		jdi_dcs_write(ctx, d, ARRAY_SIZE(d));			\
	})

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
	return container_of(panel, struct jdi, panel);
}

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	const char *addr;

	if (ctx->error < 0)
		return;

	addr = data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

/* Upstream init sequence, byte-exact (5.10 :497-877). */
static void jdi_panel_init(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x36, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x3B, 0x03, 0x14, 0x36, 0x04, 0x04);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x89, 0x28, 0x00, 0x0C, 0x00, 0xAA, 0x02, 0x0E, 0x00, 0x43, 0x00, 0x07, 0x08, 0xBB, 0x08, 0x7A);
	jdi_dcs_write_seq_static(ctx, 0xC2, 0x1B, 0xA0);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x20);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x01, 0x66);
	jdi_dcs_write_seq_static(ctx, 0x06, 0x64);
	jdi_dcs_write_seq_static(ctx, 0x07, 0x28);
	jdi_dcs_write_seq_static(ctx, 0x17, 0x66);
	jdi_dcs_write_seq_static(ctx, 0x1B, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x83);
	jdi_dcs_write_seq_static(ctx, 0x5C, 0x90);
	jdi_dcs_write_seq_static(ctx, 0x5E, 0xB6);
	jdi_dcs_write_seq_static(ctx, 0x69, 0xD0);
	jdi_dcs_write_seq_static(ctx, 0x95, 0xD1);
	jdi_dcs_write_seq_static(ctx, 0x96, 0xD1);
	jdi_dcs_write_seq_static(ctx, 0xF2, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF3, 0x54);
	jdi_dcs_write_seq_static(ctx, 0xF4, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF5, 0x54);
	jdi_dcs_write_seq_static(ctx, 0xF6, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF7, 0x54);
	jdi_dcs_write_seq_static(ctx, 0xF8, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xF9, 0x54);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x23);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x00, 0x80);
	jdi_dcs_write_seq_static(ctx, 0x07, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x08, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x09, 0x04);
	jdi_dcs_write_seq_static(ctx, 0x11, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x12, 0x96);
	jdi_dcs_write_seq_static(ctx, 0x15, 0x68);
	jdi_dcs_write_seq_static(ctx, 0x16, 0x0B);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x24);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x03, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x04, 0x15);
	jdi_dcs_write_seq_static(ctx, 0x05, 0x17);
	jdi_dcs_write_seq_static(ctx, 0x09, 0x26);
	jdi_dcs_write_seq_static(ctx, 0x0A, 0x27);
	jdi_dcs_write_seq_static(ctx, 0x0B, 0x28);
	jdi_dcs_write_seq_static(ctx, 0x0C, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x0D, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x0E, 0x2F);
	jdi_dcs_write_seq_static(ctx, 0x0F, 0x2D);
	jdi_dcs_write_seq_static(ctx, 0x10, 0x2E);
	jdi_dcs_write_seq_static(ctx, 0x11, 0x2C);
	jdi_dcs_write_seq_static(ctx, 0x12, 0x8B);
	jdi_dcs_write_seq_static(ctx, 0x13, 0x8C);
	jdi_dcs_write_seq_static(ctx, 0x16, 0x0F);
	jdi_dcs_write_seq_static(ctx, 0x17, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x1B, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x1C, 0x15);
	jdi_dcs_write_seq_static(ctx, 0x1D, 0x17);
	jdi_dcs_write_seq_static(ctx, 0x21, 0x26);
	jdi_dcs_write_seq_static(ctx, 0x22, 0x27);
	jdi_dcs_write_seq_static(ctx, 0x23, 0x28);
	jdi_dcs_write_seq_static(ctx, 0x24, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x25, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x26, 0x2F);
	jdi_dcs_write_seq_static(ctx, 0x27, 0x2D);
	jdi_dcs_write_seq_static(ctx, 0x28, 0x2E);
	jdi_dcs_write_seq_static(ctx, 0x29, 0x2C);
	jdi_dcs_write_seq_static(ctx, 0x2A, 0x8B);
	jdi_dcs_write_seq_static(ctx, 0x2B, 0x8C);
	jdi_dcs_write_seq_static(ctx, 0x30, 0x0F);
	jdi_dcs_write_seq_static(ctx, 0x31, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x32, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x33, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x36, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x4E, 0x2F);
	jdi_dcs_write_seq_static(ctx, 0x4F, 0x30);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x2F);
	jdi_dcs_write_seq_static(ctx, 0x71, 0x28);
	jdi_dcs_write_seq_static(ctx, 0x77, 0x80);
	jdi_dcs_write_seq_static(ctx, 0x79, 0x04);
	jdi_dcs_write_seq_static(ctx, 0x7A, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x7B, 0x8D);
	jdi_dcs_write_seq_static(ctx, 0x7D, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x80, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x81, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x82, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x84, 0x31);
	jdi_dcs_write_seq_static(ctx, 0x85, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x86, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x87, 0x31);
	jdi_dcs_write_seq_static(ctx, 0x90, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x92, 0x31);
	jdi_dcs_write_seq_static(ctx, 0x93, 0x13);
	jdi_dcs_write_seq_static(ctx, 0x94, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x95, 0x31);
	jdi_dcs_write_seq_static(ctx, 0x9C, 0xF4);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xA0, 0x0D);
	jdi_dcs_write_seq_static(ctx, 0xA2, 0x0D);
	jdi_dcs_write_seq_static(ctx, 0xA3, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xA4, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xA5, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xC6, 0x40);
	jdi_dcs_write_seq_static(ctx, 0xC9, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xD9, 0x80);
	jdi_dcs_write_seq_static(ctx, 0xE9, 0x02);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x25);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x0F, 0x1B);
	jdi_dcs_write_seq_static(ctx, 0x18, 0x20);
	jdi_dcs_write_seq_static(ctx, 0x19, 0xE4);
	jdi_dcs_write_seq_static(ctx, 0x20, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x21, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x66, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x67, 0x29);
	jdi_dcs_write_seq_static(ctx, 0x68, 0x50);
	jdi_dcs_write_seq_static(ctx, 0x69, 0x60);
	jdi_dcs_write_seq_static(ctx, 0x6B, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x71, 0x6D);
	jdi_dcs_write_seq_static(ctx, 0x77, 0x60);
	jdi_dcs_write_seq_static(ctx, 0x78, 0xA5);
	jdi_dcs_write_seq_static(ctx, 0x7D, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x7E, 0x2D);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x4D);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0xA9);
	jdi_dcs_write_seq_static(ctx, 0xC2, 0xD2);
	jdi_dcs_write_seq_static(ctx, 0xC4, 0x11);
	jdi_dcs_write_seq_static(ctx, 0xD6, 0x80);
	jdi_dcs_write_seq_static(ctx, 0xD7, 0x82);
	jdi_dcs_write_seq_static(ctx, 0xDA, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xDD, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xE0, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xF0, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xF1, 0x04);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x26);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x00, 0x10);
	jdi_dcs_write_seq_static(ctx, 0x01, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x03, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x04, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x05, 0x08);
	jdi_dcs_write_seq_static(ctx, 0x06, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x08, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x14, 0x06);
	jdi_dcs_write_seq_static(ctx, 0x15, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x74, 0xAF);
	jdi_dcs_write_seq_static(ctx, 0x81, 0x0D);
	jdi_dcs_write_seq_static(ctx, 0x83, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x84, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x85, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x86, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x87, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x88, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x8A, 0x1A);
	jdi_dcs_write_seq_static(ctx, 0x8B, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x8C, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x8E, 0x42);
	jdi_dcs_write_seq_static(ctx, 0x8F, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x90, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x91, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x9A, 0x80);
	jdi_dcs_write_seq_static(ctx, 0x9B, 0x04);
	jdi_dcs_write_seq_static(ctx, 0x9C, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x9E, 0x00);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x27);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x01, 0x6C);
	jdi_dcs_write_seq_static(ctx, 0x20, 0x81);
	jdi_dcs_write_seq_static(ctx, 0x21, 0x6B);
	jdi_dcs_write_seq_static(ctx, 0x25, 0x81);
	jdi_dcs_write_seq_static(ctx, 0x26, 0x96);
	jdi_dcs_write_seq_static(ctx, 0x6E, 0x23);
	jdi_dcs_write_seq_static(ctx, 0x6F, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x70, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x71, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x72, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x73, 0x21);
	jdi_dcs_write_seq_static(ctx, 0x74, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x75, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x76, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x77, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x7D, 0x09);
	jdi_dcs_write_seq_static(ctx, 0x7E, 0x6F);
	jdi_dcs_write_seq_static(ctx, 0x80, 0x23);
	jdi_dcs_write_seq_static(ctx, 0x82, 0x09);
	jdi_dcs_write_seq_static(ctx, 0x83, 0x6F);
	jdi_dcs_write_seq_static(ctx, 0x88, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x89, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xE3, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xE4, 0xE4);
	jdi_dcs_write_seq_static(ctx, 0xE5, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xE6, 0xD7);
	jdi_dcs_write_seq_static(ctx, 0xE9, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xEA, 0x1D);
	jdi_dcs_write_seq_static(ctx, 0xEB, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xEC, 0x2B);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x2A);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x00, 0x91);
	jdi_dcs_write_seq_static(ctx, 0x03, 0x20);
	jdi_dcs_write_seq_static(ctx, 0x07, 0x64);
	jdi_dcs_write_seq_static(ctx, 0x0A, 0x70);
	jdi_dcs_write_seq_static(ctx, 0x0C, 0x09);
	jdi_dcs_write_seq_static(ctx, 0x0D, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x0E, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x0F, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x11, 0xF1);
	jdi_dcs_write_seq_static(ctx, 0x15, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x16, 0xB9);
	jdi_dcs_write_seq_static(ctx, 0x19, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x1A, 0x8D);
	jdi_dcs_write_seq_static(ctx, 0x1B, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x1D, 0x36);
	jdi_dcs_write_seq_static(ctx, 0x1E, 0x36);
	jdi_dcs_write_seq_static(ctx, 0x1F, 0x36);
	jdi_dcs_write_seq_static(ctx, 0x20, 0x37);
	jdi_dcs_write_seq_static(ctx, 0x28, 0xE1);
	jdi_dcs_write_seq_static(ctx, 0x29, 0x1F);
	jdi_dcs_write_seq_static(ctx, 0x2A, 0xFF);
	jdi_dcs_write_seq_static(ctx, 0x2B, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x2D, 0x05);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x06);
	jdi_dcs_write_seq_static(ctx, 0x30, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x31, 0x8D);
	jdi_dcs_write_seq_static(ctx, 0x33, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x34, 0xE3);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x45);
	jdi_dcs_write_seq_static(ctx, 0x36, 0xCB);
	jdi_dcs_write_seq_static(ctx, 0x37, 0xDC);
	jdi_dcs_write_seq_static(ctx, 0x38, 0x4B);
	jdi_dcs_write_seq_static(ctx, 0x39, 0xC5);
	jdi_dcs_write_seq_static(ctx, 0x3A, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x46, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x47, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x4A, 0xF1);
	jdi_dcs_write_seq_static(ctx, 0x4E, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x4F, 0xB9);
	jdi_dcs_write_seq_static(ctx, 0x52, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x8D);
	jdi_dcs_write_seq_static(ctx, 0x54, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x56, 0x36);
	jdi_dcs_write_seq_static(ctx, 0x57, 0x4E);
	jdi_dcs_write_seq_static(ctx, 0x58, 0x4E);
	jdi_dcs_write_seq_static(ctx, 0x59, 0x4E);
	jdi_dcs_write_seq_static(ctx, 0x60, 0x80);
	jdi_dcs_write_seq_static(ctx, 0x61, 0xDF);
	jdi_dcs_write_seq_static(ctx, 0x62, 0x17);
	jdi_dcs_write_seq_static(ctx, 0x63, 0xDB);
	jdi_dcs_write_seq_static(ctx, 0x65, 0x05);
	jdi_dcs_write_seq_static(ctx, 0x66, 0x04);
	jdi_dcs_write_seq_static(ctx, 0x67, 0x54);
	jdi_dcs_write_seq_static(ctx, 0x68, 0xA5);
	jdi_dcs_write_seq_static(ctx, 0x6A, 0xE0);
	jdi_dcs_write_seq_static(ctx, 0x6B, 0xE1);
	jdi_dcs_write_seq_static(ctx, 0x6C, 0x30);
	jdi_dcs_write_seq_static(ctx, 0x6D, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x6E, 0xDC);
	jdi_dcs_write_seq_static(ctx, 0x6F, 0x34);
	jdi_dcs_write_seq_static(ctx, 0x70, 0xFC);
	jdi_dcs_write_seq_static(ctx, 0x71, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x7A, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x7B, 0x40);
	jdi_dcs_write_seq_static(ctx, 0x7C, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x7F, 0xA0);
	jdi_dcs_write_seq_static(ctx, 0x83, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x84, 0xB9);
	jdi_dcs_write_seq_static(ctx, 0x87, 0x0E);
	jdi_dcs_write_seq_static(ctx, 0x88, 0x8D);
	jdi_dcs_write_seq_static(ctx, 0x89, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x8B, 0x36);
	jdi_dcs_write_seq_static(ctx, 0x8C, 0x75);
	jdi_dcs_write_seq_static(ctx, 0x8D, 0x75);
	jdi_dcs_write_seq_static(ctx, 0x8E, 0x75);
	jdi_dcs_write_seq_static(ctx, 0x95, 0x80);
	jdi_dcs_write_seq_static(ctx, 0x96, 0x99);
	jdi_dcs_write_seq_static(ctx, 0x97, 0x0B);
	jdi_dcs_write_seq_static(ctx, 0x98, 0xF3);
	jdi_dcs_write_seq_static(ctx, 0x9A, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x9B, 0x03);
	jdi_dcs_write_seq_static(ctx, 0x9C, 0x0F);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x73);
	jdi_dcs_write_seq_static(ctx, 0x9F, 0xE7);
	jdi_dcs_write_seq_static(ctx, 0xA0, 0x9B);
	jdi_dcs_write_seq_static(ctx, 0xA2, 0x21);
	jdi_dcs_write_seq_static(ctx, 0xA3, 0xAA);
	jdi_dcs_write_seq_static(ctx, 0xA4, 0x98);
	jdi_dcs_write_seq_static(ctx, 0xA5, 0x23);
	jdi_dcs_write_seq_static(ctx, 0xA6, 0xA8);
	jdi_dcs_write_seq_static(ctx, 0xA7, 0x0F);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x2C);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x00, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x01, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x02, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x03, 0x12);
	jdi_dcs_write_seq_static(ctx, 0x04, 0x12);
	jdi_dcs_write_seq_static(ctx, 0x05, 0x12);
	jdi_dcs_write_seq_static(ctx, 0x0D, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x0E, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x17, 0x42);
	jdi_dcs_write_seq_static(ctx, 0x18, 0x42);
	jdi_dcs_write_seq_static(ctx, 0x19, 0x42);
	jdi_dcs_write_seq_static(ctx, 0x2D, 0xAF);
	jdi_dcs_write_seq_static(ctx, 0x2F, 0x10);
	jdi_dcs_write_seq_static(ctx, 0x30, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x32, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x33, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x37, 0x14);
	jdi_dcs_write_seq_static(ctx, 0x4D, 0x12);
	jdi_dcs_write_seq_static(ctx, 0x4E, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x4F, 0x0B);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x54, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x55, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x56, 0x1F);
	jdi_dcs_write_seq_static(ctx, 0x58, 0x1F);
	jdi_dcs_write_seq_static(ctx, 0x59, 0x1F);
	jdi_dcs_write_seq_static(ctx, 0x61, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x62, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x6B, 0x69);
	jdi_dcs_write_seq_static(ctx, 0x6C, 0x69);
	jdi_dcs_write_seq_static(ctx, 0x6D, 0x69);
	jdi_dcs_write_seq_static(ctx, 0x80, 0xAF);
	jdi_dcs_write_seq_static(ctx, 0x81, 0x10);
	jdi_dcs_write_seq_static(ctx, 0x82, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x84, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x85, 0xEB);
	jdi_dcs_write_seq_static(ctx, 0x87, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x89, 0x1E);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x1F);
	jdi_dcs_write_seq_static(ctx, 0x9E, 0x02);
	jdi_dcs_write_seq_static(ctx, 0x9F, 0x0B);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0xE0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x82);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0xF0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x1C, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x33, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x5A, 0x00);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0xD0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x22);
	jdi_dcs_write_seq_static(ctx, 0x54, 0x02);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0xC0);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x9C, 0x11);
	jdi_dcs_write_seq_static(ctx, 0x9D, 0x11);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x2B);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xB7, 0x21);
	jdi_dcs_write_seq_static(ctx, 0xB8, 0x12);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x01);

	jdi_dcs_write_seq_static(ctx, 0xFF, 0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x35, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(105 * 1000, 105 * 1000 + 100);
	jdi_dcs_write_seq_static(ctx, 0x29, 0x00);
	usleep_range(45 * 1000, 45 * 1000 + 100);
}

/*
 * Bias rail sequencing (AVDD = bias-gpios index 0 / pio120, AVEE = index 1 /
 * pio119): GPIO gates driven here, in the stock order (AVDD then AVEE).
 * The positive/negative rail voltages themselves come from the SM5109 bias
 * IC (i2c9 @ 0x3e, stock driver lcm_sm5109_i2c), which runs on its power-on
 * defaults -- this tree has no SM5109 driver yet. Order mirrors upstream
 * lcm_panel_poweron/poweroff.
 */
static int jdi_bias_enable(struct jdi *ctx)
{
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);
	usleep_range(5000, 5100);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "cannot get bias-gpios 1 %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	usleep_range(5000, 5100);

	return 0;
}

static void jdi_bias_disable(struct jdi *ctx);

static int jdi_panel_poweron(struct drm_panel *panel)
{
struct jdi *ctx = panel_to_jdi(panel);

if (ctx->prepared)
return 0;
usleep_range(5000, 5100);
return jdi_bias_enable(ctx);
}

static int jdi_panel_poweroff(struct drm_panel *panel)
{
struct jdi *ctx = panel_to_jdi(panel);

if (ctx->prepared)
return 0;
jdi_bias_disable(ctx);
usleep_range(5000, 5100);
return 0;
}

static void jdi_bias_disable(struct jdi *ctx)
{
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	if (!IS_ERR(ctx->bias_neg)) {
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
	}
	usleep_range(5000, 5100);

	ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	if (!IS_ERR(ctx->bias_pos)) {
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
	usleep_range(5000, 5100);
}

static int jdi_disable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		/*
		 * Do not call backlight_update_status() from the DRM modeset
		 * disable path. mtk_atomic_commit() already holds mtk_crtc->lock,
		 * while mtkfb_set_backlight_level() takes the same lock again.
		 * jdi_unprepare() shuts the LED enable GPIO down afterwards.
		 */
	}

	ctx->enabled = false;
	pr_info("debug for %s\n", __func__);
	return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (!ctx->prepared)
		return 0;

	if (ctx->pinctrl)
		pinctrl_select_default_state(ctx->dev);

	if (ctx->leden_gpio)
		gpiod_set_value(ctx->leden_gpio, 0);

	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(25 * 1000, 25 * 1000 + 100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(70 * 1000, 70 * 1000 + 100);

	/* Pull reset low, then cut the bias rails (AVEE first). */
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (!IS_ERR(ctx->reset_gpio)) {
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	}
	usleep_range(5000, 5100);

	if (ctx->vufsldo)
		regulator_disable(ctx->vufsldo);

	jdi_bias_disable(ctx);

	ctx->error = 0;
	ctx->prepared = false;
	pr_info("debug for %s-\n", __func__);

	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	usleep_range(12000, 12100);

	/* Bias rails up before reset (AVDD then AVEE), as upstream does.
	 * The vendor's _lcm_i2c_write_bytes(0x03, 0x43) after this targets
	 * the SM5109 bias IC, which has no driver in this tree; the rails
	 * run on the IC's defaults. It is deliberately not sent here.
	 */
	ret = jdi_bias_enable(ctx);
	if (ret < 0)
		return ret;

	if (ctx->vufsldo) {
		regulator_set_voltage(ctx->vufsldo, 1800000, 1800000);
		regulator_enable(ctx->vufsldo);
	}


	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	ctx->leden_gpio = devm_gpiod_get(ctx->dev, "leden", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->leden_gpio)) {
		dev_info(ctx->dev, "cannot get leden-gpios %ld\n",
			 PTR_ERR(ctx->leden_gpio));
		ctx->leden_gpio = NULL;
	}

	ctx->pinctrl = devm_pinctrl_get(ctx->dev);
	if (IS_ERR(ctx->pinctrl)) {
		dev_info(ctx->dev, "cannot get pinctrl %ld\n", PTR_ERR(ctx->pinctrl));
		ctx->pinctrl = NULL;
	} else {
		ctx->pinctrl_led_en = pinctrl_lookup_state(ctx->pinctrl, "led_en");
		if (IS_ERR(ctx->pinctrl_led_en)) {
			dev_info(ctx->dev, "cannot get led_en pinctrl state %ld\n",
				 PTR_ERR(ctx->pinctrl_led_en));
			ctx->pinctrl_led_en = NULL;
		} else {
			int pinctrl_ret = pinctrl_select_state(ctx->pinctrl, ctx->pinctrl_led_en);
			pr_info("debug for jdi_probe pinctrl led_en ret=%d\n", pinctrl_ret);
		}
	}

	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(15000, 15100);

	jdi_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		jdi_unprepare(panel);

	ctx->prepared = true;

	pr_info("debug for %s-\n", __func__);
	return ret;
}

static int jdi_enable(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		ctx->backlight->props.brightness = 2047;
		/*
		 * Do NOT call backlight_update_status() from here: it runs
		 * inside the DRM modeset path while the crtc lock is held,
		 * and mtkfb_set_backlight_level() would take the same lock
		 * again (D-state deadlock, v91). Userspace writes brightness
		 * via /sys/class/backlight/lcd-backlight/brightness instead.
		 */
	}

	/* qqcandy: no backlight class yet; mimic the original
	 * jdi_setbacklight_cmdq() order after display on:
	 * 0xFF 0x10, 0xFB 0x01, 0x53 0x24, then DCS 0x51.
	 */
	if (ctx->leden_gpio)
		gpiod_set_value(ctx->leden_gpio, 1);


	/* Backlight is pure panel DCS 0x51, as on stock: the SM5109 on
	 * i2c9 supplies only the bias rails. No I2C backlight-IC write
	 * belongs here.
	 */
	jdi_dcs_write_seq_static(ctx, 0xFF, 0x10);
	jdi_dcs_write_seq_static(ctx, 0xFB, 0x01);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x24);
	jdi_dcs_write_seq_static(ctx, 0x51, 0x07, 0xFF);
	pr_info("debug for jdi_enable backlight, error=%d\n", ctx->error);

	ctx->enabled = true;
	pr_info("debug for %s\n", __func__);
	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 373644,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 2538,		/* VFP */
	.vsync_end = 2412 + 2538 + 10,		/* VSA */
	.vtotal = 2412 + 2538 + 10 + 10,	/* VBP */
};

static const struct drm_display_mode performance_mode_90hz = {
	.clock = 373494,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 880,		/* VFP */
	.vsync_end = 2412 + 880 + 10,		/* VSA */
	.vtotal = 2412 + 880 + 10 + 10,		/* VBP */
};

static const struct drm_display_mode performance_mode_120hz = {
	.clock = 373794,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 54,		/* VFP */
	.vsync_end = 2412 + 54 + 10,		/* VSA */
	.vtotal = 2412 + 54 + 10 + 10,		/* VBP */
};

static const struct drm_display_mode performance_mode_30hz = {
	.clock = 373719,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 7510,		/* VFP */
	.vsync_end = 2412 + 7510 + 10,		/* VSA */
	.vtotal = 2412 + 7510 + 10 + 10,	/* VBP */
};

static const struct drm_display_mode performance_mode_45hz = {
	.clock = 373607,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 4194,		/* VFP */
	.vsync_end = 2412 + 4194 + 10,		/* VSA */
	.vtotal = 2412 + 4194 + 10 + 10,	/* VBP */
};

static const struct drm_display_mode performance_mode_48hz = {
	.clock = 373614,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 3780,		/* VFP */
	.vsync_end = 2412 + 3780 + 10,		/* VSA */
	.vtotal = 2412 + 3780 + 10 + 10,	/* VBP */
};

static const struct drm_display_mode performance_mode_50hz = {
	.clock = 373644,
	.hdisplay = 1080,
	.hsync_start = 1080 + 81,		/* HFP */
	.hsync_end = 1080 + 81 + 22,		/* HSA */
	.htotal = 1080 + 81 + 22 + 70,		/* HBP */
	.vdisplay = 2412,
	.vsync_start = 2412 + 3532,		/* VFP */
	.vsync_end = 2412 + 3532 + 10,		/* VSA */
	.vtotal = 2412 + 3532 + 10 + 10,	/* VBP */
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_90hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_120hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_30hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_45hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_48hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

static struct mtk_panel_params ext_params_50hz = {
	.pll_clk = 550,
	.phy_timcon = {
		.hs_trail = 15,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	},
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2412,
		.pic_width = 1080,
		.slice_height = 12,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 170,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 67,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 2235,
		.slice_bpg_offset = 2170,
		.initial_offset = 6144,
		.final_offset = 7072,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
	.data_rate = 1100,
};

/* Gamma tables for the low-brightness band switch, upstream byte-exact. */
static void jdi_gamma_enter(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb1[] = {0xFF, 0x20};
	char bl_tb2[] = {0xFB, 0x01};
	char bl_tb3[] = {0xAE, 0x01};
	char bl_tb4[] = {0x95, 0x09};
	char bl_tb5[] = {0x96, 0x09};
	char bl_tb6[] = {0xB0, 0x00, 0x00, 0x00, 0x05, 0x00, 0x0D, 0x00, 0x15, 0x00, 0x1D, 0x00, 0x24, 0x00, 0x2B, 0x00, 0x31};
	char bl_tb7[] = {0xB1, 0x00, 0x38, 0x00, 0x4F, 0x00, 0x65, 0x00, 0x8D, 0x00, 0xB2, 0x00, 0xF9, 0x01, 0x40, 0x01, 0x42};
	char bl_tb8[] = {0xB2, 0x01, 0x8D, 0x01, 0xE8, 0x02, 0x2E, 0x02, 0x7B, 0x02, 0xB2, 0x02, 0xF4, 0x03, 0x07, 0x03, 0x1B};
	char bl_tb9[] = {0xB3, 0x03, 0x31, 0x03, 0x4A, 0x03, 0x69, 0x03, 0x90, 0x03, 0xB6, 0x03, 0xBC, 0x00, 0x00};

	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
}

static void jdi_gamma_exit(void *dsi, dcs_write_gce cb, void *handle)
{
	char bl_tb1[] = {0xFF, 0x20};
	char bl_tb2[] = {0xFB, 0x01};
	char bl_tb3[] = {0xAE, 0x01};
	char bl_tb4[] = {0x95, 0xD1};
	char bl_tb5[] = {0x96, 0xD1};
	char bl_tb6[] = {0xB0, 0x00, 0x00, 0x00, 0x20, 0x00, 0x47, 0x00, 0x65, 0x00, 0x7E, 0x00, 0x95, 0x00, 0xA9, 0x00, 0xBB};
	char bl_tb7[] = {0xB1, 0x00, 0xCC, 0x01, 0x06, 0x01, 0x2E, 0x01, 0x70, 0x01, 0x9E, 0x01, 0xE9, 0x02, 0x22, 0x02, 0x24};
	char bl_tb8[] = {0xB2, 0x02, 0x5B, 0x02, 0x99, 0x02, 0xC3, 0x02, 0xF7, 0x03, 0x1B, 0x03, 0x43, 0x03, 0x52, 0x03, 0x5F};
	char bl_tb9[] = {0xB3, 0x03, 0x70, 0x03, 0x82, 0x03, 0x98, 0x03, 0xAC, 0x03, 0xD0, 0x03, 0xD8, 0x00, 0x00};

	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
	cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
	cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
	cb(dsi, handle, bl_tb8, ARRAY_SIZE(bl_tb8));
	cb(dsi, handle, bl_tb9, ARRAY_SIZE(bl_tb9));
}

static int jdi_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x07, 0xFF};
	char bl_tb1[] = {0x55, 0x00};
	char bl_tb2[] = {0xFF, 0x10};
	char bl_tb3[] = {0xFB, 0x01};
	char bl_tb4[] = {0x53, 0x0C};
	char bl_tb5[] = {0x53, 0x24};
	char bl_tb6[] = {0x53, 0x2C};
	char bl_tb7[] = {0x68, 0x01, 0x01};

	/*
	 * Stock sends no I2C write in this path: backlight is pure DCS
	 * 0x51 and the SM5109 bias IC runs on its defaults.
	 */
	if (level > 4095)
		level = 4095;

	bl_tb0[1] = level >> 8;
	bl_tb0[2] = level & 0xFF;

	if (!cb)
		return -1;

	if (level < 14 && level > 0 && bl_gamma == 0) {
		bl_gamma = 1;
		jdi_gamma_enter(dsi, cb, handle);
	} else if (level > 13 && bl_gamma == 1) {
		bl_gamma = 0;
		jdi_gamma_exit(dsi, cb, handle);
	} else if (level == 0) {
		bl_gamma = 0;
	}
	cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
	if (level == 0) {
		flag_dimming = 0;
		cb(dsi, handle, bl_tb7, ARRAY_SIZE(bl_tb7));
		cb(dsi, handle, bl_tb4, ARRAY_SIZE(bl_tb4));
	} else if (flag_dimming == 1) {
		cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
		flag_dimming = 0;
	}
	if (last_brightness == 0) {
		usleep_range(15 * 1000, 15 * 1000 + 100);
		if (cabc_lastlevel != 0) {
			bl_tb1[1] = cabc_lastlevel;
			cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
			if (cabc_lastlevel == 3)
				cb(dsi, handle, bl_tb6, ARRAY_SIZE(bl_tb6));
			else
				cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
		} else {
			cb(dsi, handle, bl_tb5, ARRAY_SIZE(bl_tb5));
		}
		flag_dimming = 1;
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	esd_brightness = level;
	last_brightness = level;
	pr_debug("debug for %s backlight = %d bl_gamma = %d cabc_lastlevel = %d\n",
		 __func__, level, bl_gamma, cabc_lastlevel);
	return 0;
}

static void lcm_setbrightness(void *dsi, dcs_write_gce cb, void *handle,
			      unsigned int level)
{
	unsigned int BL_MSB, BL_LSB, hbm_brightness;
	unsigned int i;

	pr_debug("%s level is %d\n", __func__, level);

	if (level > BRIGHTNESS_HALF) {
		hbm_brightness = level;
		BL_LSB = hbm_brightness >> 8;
		BL_MSB = hbm_brightness & 0xFF;

		lcm_setbrightness_hbm[0].para_list[1] = BL_LSB;
		lcm_setbrightness_hbm[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_hbm) / sizeof(struct LCM_setting_table); i++)
			cb(dsi, handle, lcm_setbrightness_hbm[i].para_list,
			   lcm_setbrightness_hbm[i].count);
	} else {
		BL_LSB = level >> 8;
		BL_MSB = level & 0xFF;

		lcm_setbrightness_normal[0].para_list[1] = BL_LSB;
		lcm_setbrightness_normal[0].para_list[2] = BL_MSB;

		for (i = 0; i < sizeof(lcm_setbrightness_normal) / sizeof(struct LCM_setting_table); i++)
			cb(dsi, handle, lcm_setbrightness_normal[i].para_list,
			   lcm_setbrightness_normal[i].count);
	}
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	struct jdi *ctx = panel_to_jdi(panel);
	unsigned int i;
	unsigned int level;

	if (!cb)
		return -1;
	if (ctx->hbm_en == en)
		goto done;

	if (en == 1) {
		for (i = 0; i < sizeof(lcm_finger_HBM_on_setting) / sizeof(struct LCM_setting_table); i++)
			cb(dsi, handle, lcm_finger_HBM_on_setting[i].para_list,
			   lcm_finger_HBM_on_setting[i].count);
	} else if (en == 0) {
		level = last_brightness;
		lcm_setbrightness(dsi, cb, handle, last_brightness);
		if (level <= BRIGHTNESS_HALF)
			flag_hbm = 0;
		else
			flag_hbm = 1;
	}
	ctx->hbm_en = en;
done:
	return 0;
}

#ifdef CONFIG_MI_DISP
static void oplus_esd_backlight_recovery(struct drm_panel *panel, void *dsi,
					 dcs_write_gce cb, void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	if (!cb)
		return;

	bl_tb0[1] = esd_brightness >> 8;
	bl_tb0[2] = esd_brightness & 0xFF;
	pr_info("%s esd_brightness=%x bl_tb0[1]=%x, bl_tb0[2]=%x\n",
		__func__, esd_brightness, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
}
#endif

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct jdi *ctx = panel_to_jdi(panel);

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return PTR_ERR(ctx->reset_gpio);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
					    unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
				   struct drm_connector *connector,
				   unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);

	if (m == NULL) {
		pr_err("%s:%d invalid display_mode\n", __func__, __LINE__);
		return -1;
	}
	if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params_90hz;
	else if (drm_mode_vrefresh(m) == 120)
		ext->params = &ext_params_120hz;
	else if (drm_mode_vrefresh(m) == 30)
		ext->params = &ext_params_30hz;
	else if (drm_mode_vrefresh(m) == 45)
		ext->params = &ext_params_45hz;
	else if (drm_mode_vrefresh(m) == 48)
		ext->params = &ext_params_48hz;
	else if (drm_mode_vrefresh(m) == 50)
		ext->params = &ext_params_50hz;
	else
		ret = 1;

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.panel_poweron = jdi_panel_poweron,
	.panel_poweroff = jdi_panel_poweroff,
	.set_backlight_cmdq = jdi_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.ext_param_set = mtk_panel_ext_param_set,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
#ifdef CONFIG_MI_DISP
	.esd_restore_backlight = oplus_esd_backlight_recovery,
#endif
};
#endif /* CONFIG_MTK_PANEL_EXT */

static int jdi_get_modes(struct drm_panel *panel,
			 struct drm_connector *connector)
{
	static const struct drm_display_mode * const modes[] = {
		&default_mode,
		&performance_mode_90hz,
		&performance_mode_120hz,
		&performance_mode_30hz,
		&performance_mode_45hz,
		&performance_mode_48hz,
		&performance_mode_50hz,
	};
	struct drm_display_mode *mode;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		mode = drm_mode_duplicate(connector->dev, modes[i]);
		if (!mode) {
			dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
				modes[i]->hdisplay, modes[i]->vdisplay,
				drm_mode_vrefresh(modes[i]));
			return -ENOMEM;
		}
		drm_mode_set_name(mode);
		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = 68;
	connector->display_info.height_mm = 153;

	return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
	.disable = jdi_disable,
	.unprepare = jdi_unprepare,
	.prepare = jdi_prepare,
	.enable = jdi_enable,
	.get_modes = jdi_get_modes,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct jdi *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("debug for oplus21143_tianma_nt36672c_dsi_vdo %s+\n", __func__);

	/*
	 * Upstream dual-panel coexistence check (:2528-2543): only bind when
	 * the DSI host endpoint points BACK at this node.
	 */
	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected, skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%pOF\n", remote_node);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			  | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	/* Just validate the reset GPIO exists before registering. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->vufsldo = devm_regulator_get(dev, "vufsldo");
	if (IS_ERR(ctx->vufsldo)) {
		dev_info(dev, "cannot get vufsldo regulator %ld\n",
			 PTR_ERR(ctx->vufsldo));
		ctx->vufsldo = NULL;
	}


	drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	pr_info("debug for %s end lcm,oplus21143_tianma_nt36672c_dsi_vdo\n",
		__func__);

	return ret;
}

static void jdi_remove(struct mipi_dsi_device *dsi)
{
	struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	/* XAGA: host_unregister already detaches an attached child before
	 * driver remove runs; detach again only if still attached, else
	 * WARN_ON(!attached) fires in mipi_dsi_detach. */
	if (dsi->attached)
		mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
}

static const struct of_device_id jdi_of_match[] = {
	{
		.compatible = "oplus21143,tianma,nt36672c,vdo",
	},
	{}
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
	.probe = jdi_probe,
	.remove = jdi_remove,
	.driver = {
		.name = "oplus21143_tianma_nt36672c_dsi_vdo",
		.of_match_table = jdi_of_match,
	},
};
module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("OPLUS 21143 Tianma NT36672C VDO LCD Panel Driver");
MODULE_LICENSE("GPL");
