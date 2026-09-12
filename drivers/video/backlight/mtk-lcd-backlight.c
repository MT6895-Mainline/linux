// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek LCD Backlight Driver
 */

#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

extern int mtkfb_set_backlight_level(unsigned int level);

/*
 * Lowest level handed to the panel while the display is meant to be on.
 * The NT36672C takes DCS 0x51 literally and also gets 0x53 0x0C
 * (dimming/BL off) from jdi_setbacklight_cmdq(), so a small userspace value
 * walks the panel down to a fully black screen with no on-screen way back.
 * Desktop brightness sliders happily send 3% (128/4095) or even 0, so clamp
 * a requested value below the floor up to the floor instead. Deliberate
 * blanking (DPMS, suspend, fbcon blank) still reaches level 0 through the
 * props.power / props.state path in mtk_lcd_bl_update_status().
 *
 * qqcandy: runtime-tunable floor, as a percentage of max_brightness.
 *   0      = clamp disabled (legacy full-range behaviour)
 *   1..100 = floor = max(1, max_brightness * percent / 100)
 *   >100   = treated as 100
 *
 * 5% (204/4095) is the dimmest value verified to stay readable on this panel:
 * a request of 128 (3.1%) is what blacked the screen out.
 */
static unsigned int min_brightness_percent = 5;
module_param(min_brightness_percent, uint, 0644);
MODULE_PARM_DESC(min_brightness_percent,
		 "minimum backlight percent of max_brightness while display on (0 disables, >100 -> 100)");

/*
 * qqcandy: brightness response curve (FALLBACK, not the stock curve).
 *
 * The stock MT6895/qqcandy vendor kernel has no level->duty remap for this
 * panel: the OPPO NT36672C driver (oplus21143_tianma_nt36672c_dsi_vdo.c,
 * jdi_setbacklight_cmdq()) writes the requested level straight into DCS 0x51,
 * so stock behaviour is linear. No brightness table/gamma exists in the stock
 * DTB (mtk_leds carries only max/min-brightness), in the OnePlus 5.10 source
 * for this model, or in the OTA vendor/odm/vendor_dlkm images. The only remap
 * table on this host (drivers/gpu/drm/panel/leds-ktz8863a.c bl_level_remap[])
 * belongs to the xaga external backlight IC and must NOT be copied.
 *
 * This is therefore the documented fallback: a gamma curve laid on top of the
 * duty floor, so the floor is the curve's base instead of a clamp that eats
 * the bottom of the slider:
 *   floor = min_brightness_percent ? max(1, max * pct / 100) : 0
 *   out   = floor + (max - floor) * (in / max) ^ (gamma/100)
 * default gamma 2.2, floor 204 (5%). out(0)=floor, out(max)=max and the map
 * is monotonic non-decreasing over the whole range. bl_gamma_x100=100 or
 * bl_curve_enable=0 restores legacy linear; the old clamp below still applies
 * the floor unchanged on that non-curve path.
 *
 * A full (max_brightness + 1)-entry LUT is built at probe with a fixed-point
 * Q16.16 log2/exp2 pair: integer only (no floats/pow()), deterministic and
 * exact for every input (no interpolation error). It is rebuilt lazily if
 * bl_gamma_x100 or min_brightness_percent changes at runtime.
 */
#define MTK_LCD_BL_MAX_BRIGHTNESS	4095

static bool bl_curve_enable = true;
module_param(bl_curve_enable, bool, 0644);
MODULE_PARM_DESC(bl_curve_enable,
		 "qqcandy: apply brightness response curve (default on; 0 = legacy linear)");

static unsigned int bl_gamma_x100 = 220;
module_param(bl_gamma_x100, uint, 0644);
MODULE_PARM_DESC(bl_gamma_x100,
		 "qqcandy: curve gamma * 100 (100 = linear, clamp 10..1000, default 220)");

/* Q16.16 factors for 2^frac: 2^(2^-(i+1)). */
static const u32 bl_exp2_tab[16] = {
	92682, 77936, 71468, 68438, 66971, 66250, 65892, 65714,
	65625, 65580, 65558, 65547, 65542, 65539, 65537, 65537,
};

static u32 bl_curve_lut[MTK_LCD_BL_MAX_BRIGHTNESS + 1];
static unsigned int bl_lut_gamma_x100;
static u32 bl_lut_floor;
static bool bl_lut_valid;

static unsigned int bl_gamma_effective(void)
{
	unsigned int g = bl_gamma_x100;

	if (g == 0 || g == 100)
		return 100;
	if (g < 10)
		return 10;
	if (g > 1000)
		return 1000;
	return g;
}

/* Same floor the clamp below computes, 0 when min_brightness_percent is off. */
static u32 bl_floor_duty(u32 max)
{
	unsigned int percent;

	if (!min_brightness_percent)
		return 0;
	percent = min(min_brightness_percent, 100U);
	return max(1U, max * percent / 100);
}

/* log2(v) in Q16.16, v >= 1. */
static u32 bl_fx_log2(u32 v)
{
	u32 n = fls(v) - 1;
	u32 m = (u32)(((u64)v << 16) >> n);
	u32 frac = 0;
	int i;

	for (i = 15; i >= 0; i--) {
		m = (u32)(((u64)m * m) >> 16);
		if (m >= (1U << 17)) {
			frac |= 1U << i;
			m >>= 1;
		}
	}

	return (n << 16) + frac;
}

/* 2^f in Q16.16 for f in [0, 1) expressed as Q16.16. */
static u32 bl_fx_exp2_frac(u32 f)
{
	u32 r = 65536;
	int i;

	for (i = 0; i < 16; i++) {
		if (f & (1U << (15 - i)))
			r = (u32)(((u64)r * bl_exp2_tab[i]) >> 16);
	}

	return r;
}

static u32 bl_curve_map(u32 x, u32 max, u32 floor, unsigned int gamma_x100)
{
	u32 range = max - floor;
	s32 lx, lmax, l;
	s64 gamma_q16;
	u32 a, n, f, inv, t_q16;
	u64 prod;

	if (!x)
		return floor;
	if (x >= max)
		return max;
	if (gamma_x100 == 100)
		return floor + (u32)(((u64)range * x) / max);

	lx = (s32)bl_fx_log2(x);
	lmax = (s32)bl_fx_log2(max);
	gamma_q16 = ((s64)gamma_x100 << 16) / 100;
	l = (s32)((gamma_q16 * (s64)(lx - lmax)) >> 16);	/* <= 0 */
	a = (u32)(-l);
	n = a >> 16;
	f = a & 0xffff;
	if (n >= 32)
		return floor;

	inv = (u32)((65536ULL * 65536ULL) / bl_fx_exp2_frac(f));	/* 2^-f */
	t_q16 = inv >> n;			/* (x/max)^gamma, Q16.16, <= 1.0 */
	prod = (u64)range * t_q16;
	return floor + (u32)(prod >> 16);
}

static void bl_curve_build_lut(u32 floor, unsigned int gamma_x100)
{
	u32 i;

	bl_curve_lut[0] = floor;
	for (i = 1; i <= MTK_LCD_BL_MAX_BRIGHTNESS; i++)
		bl_curve_lut[i] = bl_curve_map(i, MTK_LCD_BL_MAX_BRIGHTNESS,
					       floor, gamma_x100);
	bl_lut_floor = floor;
	bl_lut_gamma_x100 = gamma_x100;
	bl_lut_valid = true;
}

static int mtk_lcd_bl_update_status(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK ||
	    bd->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) {
		/*
		 * qqcandy: blank/suspend path. Pass 0 through unclamped so the
		 * panel can actually go dark; neither curve nor floor apply.
		 */
		return mtkfb_set_backlight_level(0);
	}

	/* qqcandy: fallback gamma curve, panel duty in -> panel duty out. */
	if (bl_curve_enable) {
		unsigned int gamma = bl_gamma_effective();
		u32 floor = bl_floor_duty((u32)bd->props.max_brightness);

		if (!bl_lut_valid || bl_lut_gamma_x100 != gamma ||
		    bl_lut_floor != floor)
			bl_curve_build_lut(floor, gamma);

		if ((unsigned int)brightness > MTK_LCD_BL_MAX_BRIGHTNESS)
			brightness = MTK_LCD_BL_MAX_BRIGHTNESS;
		brightness = bl_curve_lut[brightness];
	}

	/* qqcandy: floor only the backlight-class/userspace request. */
	if (min_brightness_percent) {
		unsigned int percent = min(min_brightness_percent, 100U);
		unsigned int floor = max(1U, (unsigned int)bd->props.max_brightness *
					     percent / 100);

		if (brightness < (int)floor)
			brightness = (int)floor;
	}

	return mtkfb_set_backlight_level(brightness);
}

static const struct backlight_ops mtk_lcd_bl_ops = {
	.update_status = mtk_lcd_bl_update_status,
};

static int mtk_lcd_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bd;
	int ret;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MTK_LCD_BL_MAX_BRIGHTNESS;
	props.brightness = 2047;
	props.power = FB_BLANK_UNBLANK;

	bd = devm_backlight_device_register(&pdev->dev, "lcd-backlight",
					    &pdev->dev, NULL,
					    &mtk_lcd_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

	/* qqcandy: build the fallback curve LUT for boot-time gamma/floor. */
	bl_curve_build_lut(bl_floor_duty(MTK_LCD_BL_MAX_BRIGHTNESS),
			   bl_gamma_effective());

	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->update_status) {
		ret = bd->ops->update_status(bd);
		mutex_unlock(&bd->ops_lock);
		if (ret)
			dev_info(&pdev->dev, "initial backlight update deferred: %d\n", ret);
	} else {
		mutex_unlock(&bd->ops_lock);
		dev_info(&pdev->dev, "initial backlight update deferred: %d\n", -2);
	}

	dev_info(&pdev->dev, "qqcandy lcd-backlight registered (max=%u, brightness=%u)\n",
		 bd->props.max_brightness, bd->props.brightness);

	return 0;
}

static const struct of_device_id mtk_lcd_bl_of_match[] = {
	{ .compatible = "mediatek,lcd-backlight" },
	{ .compatible = "mediatek,disp-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_lcd_bl_of_match);

static struct platform_driver mtk_lcd_bl_driver = {
	.probe = mtk_lcd_bl_probe,
	.driver = {
		.name = "mtk-lcd-backlight",
		.of_match_table = mtk_lcd_bl_of_match,
	},
};
module_platform_driver(mtk_lcd_bl_driver);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MediaTek LCD Backlight Driver");
MODULE_LICENSE("GPL");
