// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi yuechu (Civi 3, MT6895) M9 42 02 0A 1080x2400 AMOLED panel driver
 *
 * The panel is a 4-lane MIPI DSI command-mode (CMD mode) OLED; the command
 * sequence and DSC parameters come from yuechu's Android kernel
 * panel-m9-42-02-0a-dsc-cmd.c; power on/off uses the standard DRM panel lifecycle.
 *
 * Key differences from pearl (LCD, VDO mode, external KTZ8863A I2C backlight):
 *   - dsi->mode_flags lacks MIPI_DSI_MODE_VIDEO; the panel is refreshed by TE;
 *   - brightness goes through the DDIC's DCS 0x51 register (12 bit) and
 *     must be issued via CMDQ (direct DSI writes would hang the display);
 *   - panel power-up order: vddi(1.8V) -> dvdd(1.6V) -> vci/vibr(3.0V),
 *     with reset as a low 12ms / high 1ms / low 10ms / high 12ms pulse.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"

/* ---------- basic panel parameters ---------- */
#define YUECHU_WIDTH			1080
#define YUECHU_HEIGHT			2400
#define YUECHU_WIDTH_MM			71
#define YUECHU_HEIGHT_MM		153

/* 120/90/60 Hz timings (HFP/HSA/HBP and VFP/VSA/VBP are identical) */
#define YUECHU_HFP			40
#define YUECHU_HSA			20
#define YUECHU_HBP			40
#define YUECHU_VFP			10
#define YUECHU_VSA			2
#define YUECHU_VBP			16

#define YUECHU_HTOTAL			(YUECHU_WIDTH + YUECHU_HFP + YUECHU_HSA + YUECHU_HBP)
#define YUECHU_VTOTAL			(YUECHU_HEIGHT + YUECHU_VFP + YUECHU_VSA + YUECHU_VBP)

#define YUECHU_FPS_120			120
#define YUECHU_FPS_90			90
#define YUECHU_FPS_60			60

/*
 * pixel clock = htotal * vtotal * fps / 1000 (kHz):
 *   1180 * 2428 * 120 / 1000 = 343804
 *   1180 * 2428 *  90 / 1000 = 257853
 *   1180 * 2428 *  60 / 1000 = 171902
 */
#define YUECHU_CLK_120			343804
#define YUECHU_CLK_90			257853
#define YUECHU_CLK_60			171902

/* per-lane DSI rate (Mbps/lane), same for all three modes */
#define YUECHU_DATA_RATE		1100

/* the DCS 0x51 brightness register is 12 bit */
#define YUECHU_MAX_BRIGHTNESS		4095
#define YUECHU_DEFAULT_BRIGHTNESS	2047	/* factory max brightness, the Android default */

/* DSC panel ID: 0x6c226101 -> m9 / 2 / 61 / 01, CSOT first screen */
#define YUECHU_DSC_PANEL_NAME		0x6c226101

/* pseudo-commands in the command table */
#define YUECHU_REGFLAG_DELAY		0xFFFC
#define YUECHU_REGFLAG_END		0xFFFD

struct yuechu_panel {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *dvdd_gpio;
	struct gpio_desc *vddi_gpio;
	struct regulator *vci;		/* VIBR 3.0V */
	struct regulator *vddi_reg;	/* VRF18_AIF 1.8V */
	bool prepared;
	bool enabled;
	bool gir_status;
	unsigned int dynamic_fps;
	int error;
};

static inline struct yuechu_panel *to_yuechu_panel(struct drm_panel *panel)
{
	return container_of(panel, struct yuechu_panel, panel);
}

/*
 * DDIC page-select commands (0xF0/0xFF/0xFC/0xB0 etc., >= 0xB0) use generic writes;
 * the rest use DCS writes, matching the Android kernel's lcm_dcs_write().
 */
static void yuechu_dsi_write(struct yuechu_panel *ctx, const void *data,
			     size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const u8 *buf = data;
	ssize_t ret;

	if (ctx->error < 0)
		return;

	if (buf[0] >= 0xb0 || buf[0] == 0xff || buf[0] == 0xf0 ||
	    buf[0] == 0xfc)
		ret = mipi_dsi_generic_write(dsi, data, len);
	else
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to write command %zd: %*ph\n", ret, (int)len, data);
		ctx->error = ret;
	}
}

#define yuechu_write(ctx, ...) \
	do { \
		static const u8 command[] = { __VA_ARGS__ }; \
		yuechu_dsi_write((ctx), command, ARRAY_SIZE(command)); \
	} while (0)

/* command table entry: cmd=0 and count=0 means a delay (para_list[0] is ms) */
struct yuechu_setting_table {
	u8 cmd;
	u8 count;
	u8 para_list[32];
};

#define YUECHU_DELAY(ms)	{ YUECHU_REGFLAG_DELAY, 1, { (ms) } }
#define YUECHU_TABLE_END	{ YUECHU_REGFLAG_END, 0, { 0 } }

static void yuechu_push_table(struct yuechu_panel *ctx,
			      const struct yuechu_setting_table *table,
			      unsigned int count)
{
	u8 buf[sizeof(table->para_list) + 1];
	unsigned int i, j;

	for (i = 0; i < count; i++) {
		if (ctx->error < 0)
			return;

		if (table[i].cmd == YUECHU_REGFLAG_DELAY) {
			msleep(table[i].para_list[0]);
			continue;
		}
		if (table[i].cmd == YUECHU_REGFLAG_END)
			continue;

		buf[0] = table[i].cmd;
		for (j = 0; j < table[i].count; j++)
			buf[j + 1] = table[i].para_list[j];
		yuechu_dsi_write(ctx, buf, table[i].count + 1);
	}
}

/*
 * frame-rate switch table: 0x2F selects the rate, 0x5F toggles GIR, page0's
 * 0x6F/0xC0 tune VDC, page2's 0xCC/0xCE trigger the gamma update. Each rate has GIR on/off sets.
 */
static const struct yuechu_setting_table yuechu_giron_120hz[] = {
	{ 0x2F, 1, { 0x02 } },
	{ 0x5F, 1, { 0x00 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x09 } },
	{ 0xC0, 1, { 0x60 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

static const struct yuechu_setting_table yuechu_giroff_120hz[] = {
	{ 0x2F, 1, { 0x02 } },
	{ 0x5F, 1, { 0x01 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x09 } },
	{ 0xC0, 1, { 0x20 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

static const struct yuechu_setting_table yuechu_giron_90hz[] = {
	{ 0x2F, 1, { 0x04 } },
	{ 0x5F, 1, { 0x00 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x0B } },
	{ 0xC0, 1, { 0xA0 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

static const struct yuechu_setting_table yuechu_giroff_90hz[] = {
	{ 0x2F, 1, { 0x04 } },
	{ 0x5F, 1, { 0x01 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x0B } },
	{ 0xC0, 1, { 0x80 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

static const struct yuechu_setting_table yuechu_giron_60hz[] = {
	{ 0x2F, 1, { 0x03 } },
	{ 0x5F, 1, { 0x00 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x0A } },
	{ 0xC0, 1, { 0x40 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

static const struct yuechu_setting_table yuechu_giroff_60hz[] = {
	{ 0x2F, 1, { 0x03 } },
	{ 0x5F, 1, { 0x01 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x0A } },
	{ 0xC0, 1, { 0x00 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	{ 0xCC, 1, { 0x30 } },
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
};

/*
 * Panel initialization sequence (from yuechu's Android kernel init_setting[]).
 * The trailing 0x11 (sleep out) + 120 ms + 0x29 (display on) must not be omitted.
 *
 * Note that entries 1 and 4 are VDC placeholders: the Android side overrides
 * them from bootargs with panel-m9-42-02-0a-dsc-cmd.oled_vdc=0000002d02840061
 * (gir_off_vdc_index=1 / gir_on_vdc_index=3); mainline has no cmdline parsing,
 * so the factory values from the Android DT are kept here.
 */
static const struct yuechu_setting_table yuechu_init_setting[] = {
	/* VDC setting (overridden by oled_vdc) */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x08 } },
	{ 0xE9, 4, { 0x00, 0x00, 0x00, 0x16 } },
	{ 0x6F, 1, { 0x07 } },
	{ 0xB9, 4, { 0x05, 0xDB, 0x00, 0x7A } },
	/* RCN setting */
	{ 0xFF, 4, { 0xAA, 0x55, 0xA5, 0x80 } },
	{ 0x6F, 1, { 0x1D } },
	{ 0xF2, 1, { 0x05 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x1C } },
	{ 0xC0, 5, { 0x03, 0x14, 0x25, 0xA9, 0x78 } },
	/* VBP / VFP */
	{ 0x3B, 4, { 0x00, 0x0A, 0x00, 0x16 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	/* Vesa Decode enable */
	{ 0x03, 1, { 0x01 } },
	{ 0x90, 1, { 0x11 } },
	/* DSC Setting */
	{ 0x91, 18, { 0xAB, 0x28, 0x00, 0x0C, 0xC2, 0x00, 0x02, 0x0E,
		      0x01, 0x1F, 0x00, 0x07, 0x08, 0xBB, 0x08, 0x7A,
		      0x10, 0xF0 } },
	{ 0x2C, 1, { 0x00 } },
	/* Round Off */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x07 } },
	{ 0xC0, 1, { 0x00 } },
	/* Brightness Control */
	{ 0x53, 1, { 0x20 } },
	/* TE ON */
	{ 0x35, 1, { 0x00 } },
	/* Column 0..1079 */
	{ 0x2A, 4, { 0x00, 0x00, 0x04, 0x37 } },
	/* Row 0..2399 */
	{ 0x2B, 4, { 0x00, 0x00, 0x09, 0x5F } },
	/* 60 Hz (the frame-rate table switches to the target rate later) */
	{ 0x2F, 1, { 0x03 } },
	/* GIR OFF */
	{ 0x5F, 1, { 0x01 } },
	/* Page 0 */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x0A } },
	/* Normal gamma */
	{ 0xC0, 1, { 0x00 } },
	/* AOD no black */
	{ 0xFF, 4, { 0xAA, 0x55, 0xA5, 0x80 } },
	{ 0x6F, 1, { 0x61 } },
	{ 0xF3, 1, { 0x80 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	/* dimming 4 frame */
	{ 0x6F, 1, { 0x02 } },
	{ 0xB2, 2, { 0x04, 0x04 } },
	/* Disable aod reload */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0xB2, 1, { 0x18 } },
	{ 0x6F, 1, { 0x01 } },
	{ 0xCA, 1, { 0xAA } },
	{ 0x6F, 1, { 0x01 } },
	{ 0xC0, 1, { 0xB3 } },
	{ 0x6F, 1, { 0x11 } },
	{ 0xC0, 1, { 0x30 } },
	/* Page 2 */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x02 } },
	/* By Gamma Update */
	{ 0xCC, 1, { 0x30 } },
	/* By Gamma Update Echo and Delay 20ms 14H */
	{ 0xCE, 1, { 0x01 } },
	{ 0xCC, 1, { 0x00 } },
	/* Sleep out delay 120ms 78H */
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x01 } },
	{ 0xC3, 7, { 0x9B, 0x01, 0x94, 0xD0, 0x22, 0x02, 0x00 } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x00 } },
	{ 0x6F, 1, { 0x07 } },
	{ 0xB5, 2, { 0x29 } },
	{ 0x6F, 1, { 0x16 } },
	{ 0xB5, 2, { 0x0D, 0x19 } },
	{ 0x6F, 1, { 0x26 } },
	{ 0xB5, 5, { 0x0D, 0x0D, 0x0D, 0x0D, 0x0D } },
	{ 0x6F, 1, { 0x0C } },
	{ 0xB5, 2, { 0x2F } },
	{ 0x6F, 1, { 0x1B } },
	{ 0xB5, 2, { 0x0D, 0x1A } },
	{ 0xF0, 5, { 0x55, 0xAA, 0x52, 0x08, 0x07 } },
	{ 0xC0, 1, { 0x01 } },
	{ 0xC1, 1, { 0x00 } },
	{ 0xC2, 1, { 0x00 } },
	{ 0xC3, 1, { 0x00 } },
	{ 0xC4, 1, { 0x00 } },
	{ 0xC5, 1, { 0x00 } },
	{ 0xC6, 1, { 0x00 } },
	{ 0xC7, 1, { 0x00 } },
	{ 0xC8, 1, { 0x00 } },
	{ 0xCD, 1, { 0x00 } },
	{ 0xCE, 1, { 0x00 } },
	{ 0xCF, 1, { 0x00 } },
	{ 0xD0, 1, { 0x00 } },
	{ 0xD1, 1, { 0x00 } },
	{ 0xD2, 1, { 0x00 } },
	{ 0xD3, 1, { 0x00 } },
	{ 0xD4, 1, { 0x00 } },
	{ 0xD5, 1, { 0x00 } },
	{ 0xD6, 1, { 0x00 } },
	{ 0xD7, 1, { 0x00 } },
	{ 0xD8, 1, { 0x00 } },
	/* GIR / per-frame-rate gamma curves */
	{ 0x6F, 1, { 0x00 } },
	{ 0xC9, 21, { 0x31, 0x00, 0x21, 0xC9, 0x21, 0x10, 0x00, 0x00,
		      0x3F, 0xE7, 0xB7, 0x88, 0x3F, 0xE7, 0xFA, 0x60,
		      0x80, 0x04, 0x5D, 0x05, 0x90 } },
	{ 0x6F, 1, { 0x15 } },
	{ 0xC9, 15, { 0x81, 0xC0, 0x16, 0x82, 0x1C, 0x73, 0x33, 0xA0,
		      0x21, 0x6C, 0x0F, 0xA3, 0xA4, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x24 } },
	{ 0xC9, 7, { 0x03, 0xFA, 0x00, 0x04, 0x37, 0x04, 0x40 } },
	{ 0x6F, 1, { 0x00 } },
	{ 0xCA, 21, { 0x37, 0x00, 0x21, 0xC9, 0x21, 0x10, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x3F, 0xE7, 0xFA, 0x60,
		      0x80, 0x00, 0x00, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x15 } },
	{ 0xCA, 15, { 0x82, 0x1D, 0x16, 0x82, 0x79, 0x73, 0x33, 0xA0,
		      0x00, 0x00, 0x03, 0x5D, 0xA4, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x24 } },
	{ 0xCA, 7, { 0x04, 0x1A, 0x05, 0x04, 0x37, 0x16, 0x40 } },
	{ 0x6F, 1, { 0x00 } },
	{ 0xCB, 21, { 0x3D, 0x00, 0x21, 0x10, 0x21, 0x10, 0x00, 0x00,
		      0x3F, 0xE8, 0x3C, 0x80, 0x80, 0x00, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x15 } },
	{ 0xCB, 15, { 0x81, 0xC0, 0x74, 0x82, 0x1C, 0xD0, 0x33, 0x10,
		      0x00, 0x00, 0x0C, 0xA4, 0x5C, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x24 } },
	{ 0xCB, 7, { 0x04, 0x25, 0x17, 0x04, 0x37, 0x22, 0x40 } },
	{ 0x6F, 1, { 0x00 } },
	{ 0xCC, 21, { 0x3B, 0x00, 0x21, 0x10, 0x21, 0x10, 0x00, 0x00,
		      0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
		      0x7F, 0xFB, 0xBA, 0xDF, 0x00 } },
	{ 0x6F, 1, { 0x15 } },
	{ 0xCC, 15, { 0x82, 0x1D, 0x74, 0x82, 0x79, 0xD0, 0x33, 0x1F,
		      0xDE, 0xF0, 0x00, 0x5C, 0x5C, 0x00, 0x00 } },
	{ 0x6F, 1, { 0x24 } },
	{ 0xCC, 7, { 0x04, 0x32, 0x23, 0x04, 0x37, 0x39, 0x40 } },
	/* sleep out + 120ms + display on */
	{ 0x11, 0, { 0 } },
	YUECHU_DELAY(120),
	{ 0x29, 0, { 0 } },
};

static void yuechu_panel_init(struct yuechu_panel *ctx)
{
	/* panel-side reset pulse: low 12ms -> high 1ms -> low 10ms -> high 12ms */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	udelay(12 * 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	udelay(1 * 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	udelay(12 * 1000);

	yuechu_push_table(ctx, yuechu_init_setting,
			   ARRAY_SIZE(yuechu_init_setting));

	/* default 120Hz, GIR off */
	ctx->gir_status = false;
	ctx->dynamic_fps = YUECHU_FPS_120;
	yuechu_push_table(ctx, yuechu_giroff_120hz,
			   ARRAY_SIZE(yuechu_giroff_120hz));
}

/* ---------- DRM panel lifecycle ---------- */

static int yuechu_panel_prepare(struct drm_panel *panel)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	/* vrf18(1.8V) -> vddi(1.8V) -> dvdd(1.6V) -> vci/vibr(3.0V) */
	ret = regulator_enable(ctx->vddi_reg);
	if (ret)
		return ret;
	gpiod_set_value_cansleep(ctx->vddi_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 1);
	usleep_range(5000, 6000);

	ret = regulator_enable(ctx->vci);
	if (ret) {
		dev_err(ctx->dev, "failed to enable vci/vibr: %d\n", ret);
		goto err_dvdd;
	}

	ctx->error = 0;
	yuechu_panel_init(ctx);
	if (ctx->error < 0) {
		ret = ctx->error;
		goto err_vci;
	}

	ctx->prepared = true;
	dev_info(ctx->dev, "yuechu panel initialized\n");
	return 0;

err_vci:
	regulator_disable(ctx->vci);
err_dvdd:
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 0);
	gpiod_set_value_cansleep(ctx->vddi_gpio, 0);
	regulator_disable(ctx->vddi_reg);
	return ret;
}

static int yuechu_panel_unprepare(struct drm_panel *panel)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);

	if (!ctx->prepared)
		return 0;

	/* display off first, then sleep in */
	yuechu_write(ctx, 0x28);
	msleep(5);
	yuechu_write(ctx, 0x10);
	msleep(125);

	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	udelay(1000);
	regulator_disable(ctx->vci);
	udelay(2000);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 0);
	udelay(2000);
	gpiod_set_value_cansleep(ctx->vddi_gpio, 0);
	udelay(2000);
	regulator_disable(ctx->vddi_reg);

	ctx->error = 0;
	ctx->prepared = false;
	ctx->gir_status = false;
	return 0;
}

static int yuechu_panel_enable(struct drm_panel *panel)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);

	if (ctx->enabled)
		return 0;

	/* re-issue 0x51 with the current brightness after the panel is on */
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = true;
	return 0;
}

static int yuechu_panel_disable(struct drm_panel *panel)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = false;
	return 0;
}

/* ---------- display modes ---------- */

static const struct drm_display_mode yuechu_mode_120 = {
	.clock = YUECHU_CLK_120,
	.hdisplay = YUECHU_WIDTH,
	.hsync_start = YUECHU_WIDTH + YUECHU_HFP,
	.hsync_end = YUECHU_WIDTH + YUECHU_HFP + YUECHU_HSA,
	.htotal = YUECHU_HTOTAL,
	.vdisplay = YUECHU_HEIGHT,
	.vsync_start = YUECHU_HEIGHT + YUECHU_VFP,
	.vsync_end = YUECHU_HEIGHT + YUECHU_VFP + YUECHU_VSA,
	.vtotal = YUECHU_VTOTAL,
};

static const struct drm_display_mode yuechu_mode_90 = {
	.clock = YUECHU_CLK_90,
	.hdisplay = YUECHU_WIDTH,
	.hsync_start = YUECHU_WIDTH + YUECHU_HFP,
	.hsync_end = YUECHU_WIDTH + YUECHU_HFP + YUECHU_HSA,
	.htotal = YUECHU_HTOTAL,
	.vdisplay = YUECHU_HEIGHT,
	.vsync_start = YUECHU_HEIGHT + YUECHU_VFP,
	.vsync_end = YUECHU_HEIGHT + YUECHU_VFP + YUECHU_VSA,
	.vtotal = YUECHU_VTOTAL,
};

static const struct drm_display_mode yuechu_mode_60 = {
	.clock = YUECHU_CLK_60,
	.hdisplay = YUECHU_WIDTH,
	.hsync_start = YUECHU_WIDTH + YUECHU_HFP,
	.hsync_end = YUECHU_WIDTH + YUECHU_HFP + YUECHU_HSA,
	.htotal = YUECHU_HTOTAL,
	.vdisplay = YUECHU_HEIGHT,
	.vsync_start = YUECHU_HEIGHT + YUECHU_VFP,
	.vsync_end = YUECHU_HEIGHT + YUECHU_VFP + YUECHU_VSA,
	.vtotal = YUECHU_VTOTAL,
};

/*
 * bl_sync_enable / aod_delay_enable are wrapped in
 * #ifdef CONFIG_MI_DISP_FOD_SYNC in mtk_panel_ext.h; when the option is off the
 * struct has no such fields and using them fails to compile ("field designator
 * does not refer to any field"), so they are factored into a macro that
 * expands by configuration: with FOD_SYNC on, the vendor values are kept (both 1);
 * with it off, nothing is expanded.
 */
#ifdef CONFIG_MI_DISP_FOD_SYNC
#define YUECHU_FOD_SYNC_PARAMS						\
	, .bl_sync_enable = 1						\
	, .aod_delay_enable = 1
#else
#define YUECHU_FOD_SYNC_PARAMS
#endif

/*
 * One parameter set per frame rate: the DSC parameters are identical, only
 * cmd_null_pkt_len (CMD-mode null-packet length) and the dyn_fps target rate differ.
 */
#define YUECHU_DSC_PARAMS(rate, null_len, fps)				\
	.pll_clk = (rate) / 2,						\
	.cust_esd_check = 0,						\
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,		\
	.lp_perline_en = 0,						\
	.physical_width_um = 68256,					\
	.physical_height_um = 151680,					\
	.cmd_null_pkt_en = 1,						\
	.cmd_null_pkt_len = (null_len),					\
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,			\
	.dsc_params = {							\
		.enable = 1,						\
		.ver = 17,						\
		.slice_mode = 1,					\
		.rgb_swap = 0,						\
		.dsc_cfg = 40,						\
		.rct_on = 1,						\
		.bit_per_channel = 10,					\
		.dsc_line_buf_depth = 11,				\
		.bp_enable = 1,						\
		.bit_per_pixel = 128,					\
		.pic_height = YUECHU_HEIGHT,				\
		.pic_width = YUECHU_WIDTH,				\
		.slice_height = 12,					\
		.slice_width = YUECHU_WIDTH / 2,			\
		.chunk_size = YUECHU_WIDTH / 2,				\
		.xmit_delay = 512,					\
		.dec_delay = 526,					\
		.scale_value = 32,					\
		.increment_interval = 287,				\
		.decrement_interval = 7,				\
		.line_bpg_offset = 12,					\
		.nfl_bpg_offset = 2235,					\
		.slice_bpg_offset = 2170,				\
		.initial_offset = 6144,					\
		.final_offset = 4336,					\
		.flatness_minqp = 7,					\
		.flatness_maxqp = 16,					\
		.rc_model_size = 8192,					\
		.rc_edge_factor = 6,					\
		.rc_quant_incr_limit0 = 15,				\
		.rc_quant_incr_limit1 = 15,				\
		.rc_tgt_offset_hi = 3,					\
		.rc_tgt_offset_lo = 3,					\
		.dsc_config_panel_name = YUECHU_DSC_PANEL_NAME,		\
	},								\
	.data_rate = (rate),						\
	.dyn_fps = {							\
		.switch_en = 1,						\
		.vact_timing_fps = (fps),				\
	}								\
	YUECHU_FOD_SYNC_PARAMS

static struct mtk_panel_params yuechu_ext_params_120 = {
	YUECHU_DSC_PARAMS(YUECHU_DATA_RATE, 350, YUECHU_FPS_120),
};

static struct mtk_panel_params yuechu_ext_params_90 = {
	YUECHU_DSC_PARAMS(YUECHU_DATA_RATE, 900, YUECHU_FPS_90),
};

static struct mtk_panel_params yuechu_ext_params_60 = {
	YUECHU_DSC_PARAMS(YUECHU_DATA_RATE, 1300, YUECHU_FPS_60),
};

/* ---------- mtk_panel_ext callbacks ---------- */

static struct drm_display_mode *yuechu_mode_by_id(struct drm_connector *connector,
						  unsigned int id)
{
	struct drm_display_mode *mode;
	unsigned int i = 0;

	if (!connector)
		return NULL;

	list_for_each_entry(mode, &connector->modes, head) {
		if (i++ == id)
			return mode;
	}
	return NULL;
}

static struct mtk_panel_params *yuechu_params_by_vrefresh(struct drm_connector *connector,
							  unsigned int id)
{
	struct drm_display_mode *mode = yuechu_mode_by_id(connector, id);
	int vrefresh;

	if (!mode)
		return &yuechu_ext_params_120;

	vrefresh = drm_mode_vrefresh(mode);
	if (vrefresh == YUECHU_FPS_60)
		return &yuechu_ext_params_60;
	if (vrefresh == YUECHU_FPS_90)
		return &yuechu_ext_params_90;
	return &yuechu_ext_params_120;
}

static int yuechu_ext_param_set(struct drm_panel *panel,
				struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);

	if (!ext || !connector)
		return -EINVAL;

	ext->params = yuechu_params_by_vrefresh(connector, mode);
	return 0;
}

static int yuechu_ext_param_get(struct drm_panel *panel,
				struct drm_connector *connector,
				struct mtk_panel_params **ext_param,
				unsigned int mode)
{
	if (!ext_param || !connector)
		return -EINVAL;

	*ext_param = yuechu_params_by_vrefresh(connector, mode);
	return 0;
}

static int yuechu_ext_reset(struct drm_panel *panel, int on)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, on);
	return 0;
}

static bool yuechu_get_panel_initialized(struct drm_panel *panel)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);

	return ctx->prepared;
}

/* the v2 DSI handshake path calls this after DRM prepare; init is already done */
static void yuechu_panel_ext_init(struct drm_panel *panel)
{
	(void)panel;
}

/*
 * Brightness: the DDIC register is DCS 0x51, 12 bit (0..4095).
 * It must go through the CRTC's CMDQ backlight path (set_backlight_cmdq);
 * writing DSI messages directly would contend with the CMD-mode trigger loop and hang the display.
 */
static int yuechu_set_backlight_cmdq(void *dsi_drv, dcs_write_gce cb,
				     void *handle, unsigned int level)
{
	u8 command[] = { 0x51, level >> 8, level & 0xff };

	if (!dsi_drv || !cb)
		return -EINVAL;

	cb(dsi_drv, handle, command, ARRAY_SIZE(command));
	return 0;
}

extern int mtkfb_set_backlight_level(unsigned int level);

/* mi_disp's SET_BRIGHTNESS ioctl lands here and is also handed to the v2 CRTC CMDQ backlight */
static int yuechu_setbacklight_control(struct drm_panel *panel,
				       unsigned int level)
{
	if (level > YUECHU_MAX_BRIGHTNESS)
		level = YUECHU_MAX_BRIGHTNESS;
	return mtkfb_set_backlight_level(level);
}

static void yuechu_switch_fps(struct yuechu_panel *ctx, unsigned int fps)
{
	const struct yuechu_setting_table *table;
	unsigned int count;

	switch (fps) {
	case YUECHU_FPS_60:
		table = ctx->gir_status ? yuechu_giron_60hz : yuechu_giroff_60hz;
		count = ARRAY_SIZE(yuechu_giron_60hz);
		break;
	case YUECHU_FPS_90:
		table = ctx->gir_status ? yuechu_giron_90hz : yuechu_giroff_90hz;
		count = ARRAY_SIZE(yuechu_giron_90hz);
		break;
	case YUECHU_FPS_120:
	default:
		table = ctx->gir_status ? yuechu_giron_120hz : yuechu_giroff_120hz;
		count = ARRAY_SIZE(yuechu_giron_120hz);
		break;
	}

	yuechu_push_table(ctx, table, count);
	ctx->dynamic_fps = fps;
}

static int yuechu_mode_switch(struct drm_panel *panel,
			      struct drm_connector *connector,
			      unsigned int cur_mode, unsigned int dst_mode,
			      enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct yuechu_panel *ctx = to_yuechu_panel(panel);
	struct drm_display_mode *dst;
	int vrefresh;

	if (stage != BEFORE_DSI_POWERDOWN || cur_mode == dst_mode)
		return 0;

	dst = yuechu_mode_by_id(connector, dst_mode);
	if (!dst)
		return -EINVAL;

	vrefresh = drm_mode_vrefresh(dst);
	if (vrefresh == drm_mode_vrefresh(yuechu_mode_by_id(connector, cur_mode)))
		return 0;

	dev_info(ctx->dev, "switching frame rate %u -> %d Hz\n", ctx->dynamic_fps, vrefresh);
	yuechu_switch_fps(ctx, vrefresh);
	return ctx->error < 0 ? ctx->error : 0;
}

static struct mtk_panel_funcs yuechu_ext_funcs = {
	.reset = yuechu_ext_reset,
	.get_panel_initialized = yuechu_get_panel_initialized,
	.init = yuechu_panel_ext_init,
	.ext_param_set = yuechu_ext_param_set,
	.ext_param_get = yuechu_ext_param_get,
	.set_backlight_cmdq = yuechu_set_backlight_cmdq,
	.setbacklight_control = yuechu_setbacklight_control,
	.mode_switch = yuechu_mode_switch,
};

/* ---------- standard backlight device ---------- */

/*
 * Userspace (Plasma PowerDevil / auto brightness) controls the level through
 * /sys/class/backlight. The Android side uses the private mtk_leds
 * (mediatek,disp-leds) + led_mode=4 chain; mainline uses a standard backlight device whose writes go through mtkfb_set_backlight_level().
 */
static int yuechu_bl_update_status(struct backlight_device *bl)
{
	/* when the panel is off the pipeline is stopped; the enable callback restores the level */
	if (bl->props.power != FB_BLANK_UNBLANK ||
	    (bl->props.state & BL_CORE_FBBLANK))
		return 0;

	return mtkfb_set_backlight_level(backlight_get_brightness(bl));
}

static const struct backlight_ops yuechu_bl_ops = {
	.update_status = yuechu_bl_update_status,
};

/* ---------- mode list ---------- */

static int yuechu_panel_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	const struct drm_display_mode *modes[] = {
		&yuechu_mode_120,
		&yuechu_mode_90,
		&yuechu_mode_60,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, modes[i]);
		if (!mode)
			return -ENOMEM;
		drm_mode_set_name(mode);
		mode->type = DRM_MODE_TYPE_DRIVER;
		/* matches Android: 60Hz is the preferred mode */
		if (modes[i] == &yuechu_mode_60)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = YUECHU_WIDTH_MM;
	connector->display_info.height_mm = YUECHU_HEIGHT_MM;
	return ARRAY_SIZE(modes);
}

static const struct drm_panel_funcs yuechu_panel_funcs = {
	.prepare = yuechu_panel_prepare,
	.unprepare = yuechu_panel_unprepare,
	.enable = yuechu_panel_enable,
	.disable = yuechu_panel_disable,
	.get_modes = yuechu_panel_get_modes,
};

/* ---------- probe / remove ---------- */

static int yuechu_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct yuechu_panel *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* CMD mode: no MIPI_DSI_MODE_VIDEO; EOT is sent unless NO_EOT_PACKET is set */
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset-gpios\n");

	ctx->dvdd_gpio = devm_gpiod_get(dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->dvdd_gpio),
				     "failed to get dvdd-gpios\n");

	ctx->vddi_gpio = devm_gpiod_get(dev, "vddi", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddi_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi_gpio),
				     "failed to get vddi-gpios\n");

	/* VIBR 3.0V */
	ctx->vci = devm_regulator_get(dev, "vibr");
	if (IS_ERR(ctx->vci))
		return dev_err_probe(dev, PTR_ERR(ctx->vci),
				     "failed to get vibr regulator\n");

	/* VRF18_AIF 1.8V */
	ctx->vddi_reg = devm_regulator_get(dev, "vrf18");
	if (IS_ERR(ctx->vddi_reg))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi_reg),
				     "failed to get vrf18 regulator\n");

	/*
	 * LK powers the panel before handing control to the kernel, so the use counts
	 * of both regulators must be bumped here; otherwise the regulator framework
	 * turns "unused" supplies off at late_initcall and the screen blacks out mid-boot.
	 */
	ret = regulator_enable(ctx->vddi_reg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vrf18\n");
	ret = regulator_enable(ctx->vci);
	if (ret) {
		regulator_disable(ctx->vddi_reg);
		return dev_err_probe(dev, ret, "failed to enable vibr\n");
	}

	/*
	 * ESD error-flag pin (mi,esd-err-irq-gpio in the Android DT).
	 *
	 * Mainline dropped of_get_named_gpio_flags() along with
	 * include/linux/of_gpio.h, so the gpiod API is used here. of_find_gpio()
	 * tries "<con_id>-gpios" then "<con_id>-gpio", so the con_id
	 * "mi,esd-err-irq-gpio".
	 *
	 * desc_to_gpio() converts it back to the raw GPIO number that mediatek_v2's
	 * ESD check needs: mi_disp_esd_check.c calls gpio_to_irq()/gpio_request() on it.
	 * devm_gpiod_put() must follow immediately, or that gpio_request() gets -EBUSY.
	 */
	{
		struct gpio_desc *esd_gpio;

		esd_gpio = devm_gpiod_get_optional(dev, "mi,esd-err-irq",
						   GPIOD_IN);
		if (esd_gpio) {
			yuechu_ext_params_120.err_flag_irq_gpio =
				desc_to_gpio(esd_gpio);
			devm_gpiod_put(dev, esd_gpio);
		} else {
			dev_info(dev, "no mi,esd-err-irq-gpio, ESD pin check disabled\n");
		}
	}
	yuechu_ext_params_90.err_flag_irq_gpio =
		yuechu_ext_params_120.err_flag_irq_gpio;
	yuechu_ext_params_90.err_flag_irq_flags =
		yuechu_ext_params_120.err_flag_irq_flags;
	yuechu_ext_params_60.err_flag_irq_gpio =
		yuechu_ext_params_120.err_flag_irq_gpio;
	yuechu_ext_params_60.err_flag_irq_flags =
		yuechu_ext_params_120.err_flag_irq_flags;

	INIT_LIST_HEAD(&ctx->panel.list);
	INIT_LIST_HEAD(&ctx->panel.followers);
	mutex_init(&ctx->panel.follower_lock);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &yuechu_panel_funcs;
	ctx->panel.connector_type = DRM_MODE_CONNECTOR_DSI;
	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	/* if the DT gives no backlight, register a standard backlight device */
	if (!ctx->panel.backlight) {
		struct backlight_properties props = {
			.type = BACKLIGHT_RAW,
			.max_brightness = YUECHU_MAX_BRIGHTNESS,
			.brightness = YUECHU_DEFAULT_BRIGHTNESS,
			.power = FB_BLANK_UNBLANK,
		};

		ctx->panel.backlight = devm_backlight_device_register(
			dev, "m9_42_02_0a", dev, ctx, &yuechu_bl_ops, &props);
		if (IS_ERR(ctx->panel.backlight))
			return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
					     "failed to register backlight device\n");
	}
	ctx->backlight = ctx->panel.backlight;

	/*
	 * LK has already lit and initialized the panel: mark it prepared/enabled
	 * to avoid a second power-up flashing the screen. After the first unprepare, full init runs.
	 */
	ctx->prepared = true;
	ctx->enabled = true;
	ctx->gir_status = false;
	ctx->dynamic_fps = YUECHU_FPS_120;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	ret = mtk_panel_ext_create(dev, &yuechu_ext_params_120,
				   &yuechu_ext_funcs, &ctx->panel);
	if (ret < 0) {
		mipi_dsi_detach(dsi);
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dev_info(dev, "yuechu m9_42_02_0a_dsc_cmd panel registered (1080x2400 CMD mode)\n");
	return 0;
}

static void yuechu_panel_remove(struct mipi_dsi_device *dsi)
{
	struct yuechu_panel *ctx = mipi_dsi_get_drvdata(dsi);
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
}

static const struct of_device_id yuechu_panel_of_match[] = {
	{ .compatible = "xiaomi,yuechu-m9-42-02-0a" },
	{ .compatible = "m9_42_02_0a_dsc_cmd,lcm" },
	{ }
};
MODULE_DEVICE_TABLE(of, yuechu_panel_of_match);

static struct mipi_dsi_driver yuechu_panel_driver = {
	.probe = yuechu_panel_probe,
	.remove = yuechu_panel_remove,
	.driver = {
		.name = "panel-m9-42-02-0a-dsc-cmd",
		.of_match_table = yuechu_panel_of_match,
	},
};
module_mipi_dsi_driver(yuechu_panel_driver);

MODULE_DESCRIPTION("Xiaomi yuechu M9 42 02 0A 1080x2400 DSC command-mode AMOLED panel");
MODULE_LICENSE("GPL");
