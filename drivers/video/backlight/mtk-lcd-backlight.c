// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek LCD Backlight Driver
 */

#include <linux/backlight.h>
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

static int mtk_lcd_bl_update_status(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK ||
	    bd->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) {
		/*
		 * qqcandy: blank/suspend path. Pass 0 through unclamped so the
		 * panel can actually go dark; the floor must never apply here.
		 */
		brightness = 0;
	} else if (min_brightness_percent) {
		/* qqcandy: floor only the backlight-class/userspace request. */
		unsigned int percent = min(min_brightness_percent, 100U);
		unsigned int floor = max(1U, (unsigned int)bd->props.max_brightness *
					     percent / 100);

		if (brightness < floor)
			brightness = floor;
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
	props.max_brightness = 4095;
	props.brightness = 2047;
	props.power = FB_BLANK_UNBLANK;

	bd = devm_backlight_device_register(&pdev->dev, "lcd-backlight",
					    &pdev->dev, NULL,
					    &mtk_lcd_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	platform_set_drvdata(pdev, bd);

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
