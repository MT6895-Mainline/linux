// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi rubens L11A 38 0A 0A 1440x3200 AMOLED panel.
 *
 * The panel is a four-lane MIPI DSI command-mode panel.  Its command
 * sequences and DSC parameters are taken from the rubens Android kernel;
 * power handling is implemented with the normal DRM panel lifecycle.
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

#define RUBENS_WIDTH		1440
#define RUBENS_HEIGHT		3200
#define RUBENS_WIDTH_MM		69
#define RUBENS_HEIGHT_MM	154
#define RUBENS_DATA_RATE	1360

struct rubens_panel {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *dvdd_gpio;
	struct regulator *vci;
	struct regulator *vddi;
	bool prepared;
	bool enabled;
	int error;
};

static inline struct rubens_panel *to_rubens_panel(struct drm_panel *panel)
{
	return container_of(panel, struct rubens_panel, panel);
}

static void rubens_dsi_write(struct rubens_panel *ctx, const void *data,
			     size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	const u8 *buf = data;

	if (ctx->error < 0)
		return;

	/* The vendor table uses generic writes for command-2 page selectors. */
	if (buf[0] >= 0xb0 || buf[0] == 0xff || buf[0] == 0xf0 ||
	    buf[0] == 0xfc)
		ret = mipi_dsi_generic_write(dsi, data, len);
	else
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0)
		ctx->error = ret;
}

#define rubens_write(ctx, ...) \
	do { \
		static const u8 command[] = { __VA_ARGS__ }; \
		rubens_dsi_write((ctx), command, ARRAY_SIZE(command)); \
	} while (0)

/* DSC PPS supplied by the rubens L11A panel vendor. */
static const u8 rubens_pps[] = {
	0x9e, 0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x0c, 0x80,
	0x05, 0xa0, 0x00, 0x19, 0x02, 0xd0, 0x02, 0xd0, 0x02,
	0x00, 0x02, 0x68, 0x00, 0x20, 0x02, 0xbe, 0x00, 0x0a,
	0x00, 0x0c, 0x04, 0x00, 0x03, 0x0d, 0x18, 0x00, 0x10,
	0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b, 0x0b, 0x33,
	0x0e, 0x1c, 0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70,
	0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00,
	0x09, 0x40, 0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19,
	0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void rubens_panel_init(struct rubens_panel *ctx)
{
	rubens_write(ctx, 0x9d, 0x01);
	rubens_dsi_write(ctx, rubens_pps, ARRAY_SIZE(rubens_pps));
	rubens_write(ctx, 0x11);
	msleep(120);
	rubens_write(ctx, 0x35, 0x00);
	rubens_write(ctx, 0x2a, 0x00, 0x00, 0x05, 0x9f);
	rubens_write(ctx, 0x2b, 0x00, 0x00, 0x0c, 0x7f);
	rubens_write(ctx, 0xf0, 0x5a, 0x5a);
	rubens_write(ctx, 0xc3, 0x4c);
	rubens_write(ctx, 0xf0, 0xa5, 0xa5);
	rubens_write(ctx, 0xf0, 0x5a, 0x5a);
	rubens_write(ctx, 0x60, 0x08);
	rubens_write(ctx, 0xf7, 0x0f);
	rubens_write(ctx, 0xf0, 0xa5, 0xa5);
	/* Panel voltage, dimming, and error-flag setup. */
	rubens_write(ctx, 0xf0, 0x5a, 0x5a);
	rubens_write(ctx, 0xb0, 0x00, 0x0a, 0xb1);
	rubens_write(ctx, 0xb1, 0x38);
	rubens_write(ctx, 0xb0, 0x00, 0x0b, 0xb1);
	rubens_write(ctx, 0xb1, 0x40);
	rubens_write(ctx, 0xf1, 0x0f);
	rubens_write(ctx, 0xb0, 0x00, 0x0e, 0x94);
	rubens_write(ctx, 0x94, 0x08);
	rubens_write(ctx, 0xb0, 0x00, 0x0d, 0x94);
	rubens_write(ctx, 0x94, 0x60);
	rubens_write(ctx, 0x53, 0x28);
	rubens_write(ctx, 0x51, 0x00, 0x00);
	rubens_write(ctx, 0xf7, 0x0f);
	rubens_write(ctx, 0xf0, 0xa5, 0xa5);
	/* FFC and TE settings are required before the first frame. */
	rubens_write(ctx, 0xfc, 0x5a, 0x5a);
	rubens_write(ctx, 0xb0, 0x00, 0x2a, 0xc5);
	rubens_write(ctx, 0xc5, 0x11, 0x10, 0x50, 0x05, 0x47, 0x4b,
		     0x46, 0x5a, 0x4d, 0x9a, 0x4c, 0x0c, 0x47, 0x4b,
		     0x46, 0x5a, 0x4d, 0x9a, 0x4c, 0x0c, 0x47, 0x4b,
		     0x46, 0x5a, 0x4d, 0x9a, 0x4c, 0x0c);
	rubens_write(ctx, 0xfc, 0xa5, 0xa5);
	rubens_write(ctx, 0xf0, 0x5a, 0x5a);
	rubens_write(ctx, 0xb0, 0x00, 0x01, 0xbd);
	rubens_write(ctx, 0xbd, 0x83);
	rubens_write(ctx, 0xb0, 0x00, 0x2d, 0xbd);
	rubens_write(ctx, 0xbd, 0x04);
	rubens_write(ctx, 0xf7, 0x0f);
	rubens_write(ctx, 0xf0, 0xa5, 0xa5);
	rubens_write(ctx, 0x29);
}

/* The v2 DSI handoff path calls ext->init after the DRM prepare step. */
static void rubens_panel_ext_init(struct drm_panel *panel)
{
	(void)panel;
}

static int rubens_panel_prepare(struct drm_panel *panel)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	ret = regulator_enable(ctx->vddi);
	if (ret)
		return ret;
	ret = regulator_enable(ctx->vci);
	if (ret)
		goto disable_vddi;

	gpiod_set_value_cansleep(ctx->dvdd_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);

	ctx->error = 0;
	rubens_panel_init(ctx);
	if (ctx->error < 0) {
		ret = ctx->error;
		goto power_down;
	}
	ctx->prepared = true;
	return 0;

power_down:
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 0);
	regulator_disable(ctx->vci);
disable_vddi:
	regulator_disable(ctx->vddi);
	return ret;
}

static int rubens_panel_unprepare(struct drm_panel *panel)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);

	if (!ctx->prepared)
		return 0;

	rubens_write(ctx, 0x28);
	msleep(50);
	rubens_write(ctx, 0x10);
	msleep(150);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	gpiod_set_value_cansleep(ctx->dvdd_gpio, 0);
	regulator_disable(ctx->vci);
	regulator_disable(ctx->vddi);
	ctx->error = 0;
	ctx->prepared = false;
	return 0;
}

static int rubens_panel_enable(struct drm_panel *panel)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);

	if (ctx->enabled)
		return 0;
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = true;
	return 0;
}

static int rubens_panel_disable(struct drm_panel *panel)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);

	if (!ctx->enabled)
		return 0;
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = false;
	return 0;
}

static const struct drm_display_mode rubens_mode_60 = {
	.clock = 288750,
	.hdisplay = RUBENS_WIDTH,
	.hsync_start = RUBENS_WIDTH + 20,
	.hsync_end = RUBENS_WIDTH + 22,
	.htotal = RUBENS_WIDTH + 38,
	.vdisplay = RUBENS_HEIGHT,
	.vsync_start = RUBENS_HEIGHT + 54,
	.vsync_end = RUBENS_HEIGHT + 64,
	.vtotal = RUBENS_HEIGHT + 74,
};

static const struct drm_display_mode rubens_mode_120 = {
	.clock = 582000,
	.hdisplay = RUBENS_WIDTH,
	.hsync_start = RUBENS_WIDTH + 20,
	.hsync_end = RUBENS_WIDTH + 22,
	.htotal = RUBENS_WIDTH + 38,
	.vdisplay = RUBENS_HEIGHT,
	.vsync_start = RUBENS_HEIGHT + 54,
	.vsync_end = RUBENS_HEIGHT + 64,
	.vtotal = RUBENS_HEIGHT + 74,
};

static struct mtk_panel_params rubens_ext_params = {
	.pll_clk = 680,
	.data_rate = RUBENS_DATA_RATE,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = 69552,
	.physical_height_um = 154560,
	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = RUBENS_HEIGHT,
		.pic_width = RUBENS_WIDTH,
		.slice_height = 25,
		.slice_width = 720,
		.chunk_size = 720,
		.xmit_delay = 512,
		.dec_delay = 616,
		.scale_value = 32,
		.increment_interval = 702,
		.decrement_interval = 10,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 1024,
		.slice_bpg_offset = 781,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
	},
};

static int rubens_ext_param_set(struct drm_panel *panel,
				struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);

	if (!ext || mode > 1)
		return -EINVAL;
	if (!connector)
		return -EINVAL;
	ext->params = &rubens_ext_params;
	return 0;
}

static int rubens_ext_reset(struct drm_panel *panel, int on)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, on);
	return 0;
}

static int rubens_set_backlight_cmdq(void *dsi_drv, dcs_write_gce cb,
				     void *handle, unsigned int level)
{
	u8 command[] = { 0x51, level >> 8, level & 0xff };

	if (!dsi_drv || !cb)
		return -EINVAL;
	cb(dsi_drv, handle, command, ARRAY_SIZE(command));
	return 0;
}

/*
 * mi_disp's SET_BRIGHTNESS ioctl needs this callback; route it through the
 * v2 CRTC backlight helper so the DCS 0x51 write goes out via CMDQ.
 */
extern int mtkfb_set_backlight_level(unsigned int level);

static int rubens_setbacklight_control(struct drm_panel *panel,
				       unsigned int level)
{
	if (level > 4095)
		level = 4095;
	return mtkfb_set_backlight_level(level);
}

static int rubens_mode_switch(struct drm_panel *panel,
			      struct drm_connector *connector, unsigned int cur_mode,
			      unsigned int dst_mode,
			      enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct rubens_panel *ctx = to_rubens_panel(panel);

	if (stage != BEFORE_DSI_POWERDOWN || cur_mode == dst_mode)
		return 0;
	/* Page 0x5a selects the 120/60 Hz command-mode timing. */
	rubens_write(ctx, 0xf0, 0x5a, 0x5a);
	if (dst_mode)
		rubens_write(ctx, 0x60, 0x00);
	else
		rubens_write(ctx, 0x60, 0x08);
	rubens_write(ctx, 0xf7, 0x0f);
	rubens_write(ctx, 0xf0, 0xa5, 0xa5);
	return ctx->error < 0 ? ctx->error : 0;
}

static struct mtk_panel_funcs rubens_ext_funcs = {
	.reset = rubens_ext_reset,
	.init = rubens_panel_ext_init,
	.ext_param_set = rubens_ext_param_set,
	.set_backlight_cmdq = rubens_set_backlight_cmdq,
	.setbacklight_control = rubens_setbacklight_control,
	.mode_switch = rubens_mode_switch,
};

static int rubens_panel_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	const struct drm_display_mode *modes[] = {
		&rubens_mode_60,
		&rubens_mode_120,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, modes[i]);
		if (!mode)
			return -ENOMEM;
		drm_mode_set_name(mode);
		mode->type = DRM_MODE_TYPE_DRIVER;
		if (!i)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
	}
	connector->display_info.width_mm = RUBENS_WIDTH_MM;
	connector->display_info.height_mm = RUBENS_HEIGHT_MM;
	return ARRAY_SIZE(modes);
}

static const struct drm_panel_funcs rubens_panel_funcs = {
	.prepare = rubens_panel_prepare,
	.unprepare = rubens_panel_unprepare,
	.enable = rubens_panel_enable,
	.disable = rubens_panel_disable,
	.get_modes = rubens_panel_get_modes,
};

static int rubens_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rubens_panel *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	mipi_dsi_set_drvdata(dsi, ctx);
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Keep the flags supported by this kernel's MIPI DSI API. */
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* LK has already powered and enabled the panel before Linux starts. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return PTR_ERR(ctx->reset_gpio);
	ctx->dvdd_gpio = devm_gpiod_get(dev, "dvdd", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->dvdd_gpio))
		return PTR_ERR(ctx->dvdd_gpio);
	ctx->vci = devm_regulator_get(dev, "vibr");
	if (IS_ERR(ctx->vci))
		return PTR_ERR(ctx->vci);
	ctx->vddi = devm_regulator_get(dev, "vrf18");
	if (IS_ERR(ctx->vddi))
		return PTR_ERR(ctx->vddi);
	ret = regulator_enable(ctx->vddi);
	if (ret)
		return ret;
	ret = regulator_enable(ctx->vci);
	if (ret) {
		regulator_disable(ctx->vddi);
		return ret;
	}

	INIT_LIST_HEAD(&ctx->panel.list);
	INIT_LIST_HEAD(&ctx->panel.followers);
	mutex_init(&ctx->panel.follower_lock);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &rubens_panel_funcs;
	ctx->panel.connector_type = DRM_MODE_CONNECTOR_DSI;
	ctx->panel.prepare_prev_first = true;
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;
	ctx->backlight = ctx->panel.backlight;
	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_add(&ctx->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return ret;
	}
	ret = mtk_panel_ext_create(dev, &rubens_ext_params, &rubens_ext_funcs,
				   &ctx->panel);
	if (ret < 0) {
		mipi_dsi_detach(dsi);
		drm_panel_remove(&ctx->panel);
	}
	return ret;
}

static void rubens_panel_remove(struct mipi_dsi_device *dsi)
{
	struct rubens_panel *ctx = mipi_dsi_get_drvdata(dsi);
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
}

static const struct of_device_id rubens_panel_of_match[] = {
	{ .compatible = "xiaomi,rubens-l11a-38-0a-0a" },
	{ .compatible = "l11a_38_0a_0a_dsc_cmd,lcm" },
	{ }
};
MODULE_DEVICE_TABLE(of, rubens_panel_of_match);

static struct mipi_dsi_driver rubens_panel_driver = {
	.probe = rubens_panel_probe,
	.remove = rubens_panel_remove,
	.driver = {
		.name = "panel-l11a-38-0a-0a-dsc-cmd",
		.of_match_table = rubens_panel_of_match,
	},
};
module_mipi_dsi_driver(rubens_panel_driver);

MODULE_DESCRIPTION("Xiaomi rubens L11A 38 0A 0A DSC command-mode panel");
MODULE_LICENSE("GPL");
