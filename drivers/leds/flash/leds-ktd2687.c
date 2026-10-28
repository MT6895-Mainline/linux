// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kinetic KTD2687 dual-channel I2C flash LED driver
 *
 * The KTD2687 has two current sinks with independent 7-bit torch and flash
 * current registers.  On the Xiaomi MT6895 boards (xaga et al.) both sinks
 * are wired to the two strings of one rear-camera flashlight package, and
 * the vendor driver always programs the same current into both of them
 * while enabling/disabling them together.  The chip is therefore modelled
 * as a single LED class device owning current sinks 0 and 1, which keeps
 * the semantics of the vendor stack intact while exposing the standard LED
 * flash interface.
 *
 * Torch current is specified here as the *combined* current of both
 * strings, matching what the vendor stack and the Android camera HAL use:
 * a requested total is split evenly across the two sinks.  Torch levels can
 * be changed at any time while the torch is on - only the current register
 * is rewritten and the chip stays enabled - so brightness ramps are
 * seamless and never blink the LED off.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>

#define KTD2687_REG_ENABLE		0x01
#define KTD2687_REG_LED0_FLASH_BR	0x03
#define KTD2687_REG_LED1_FLASH_BR	0x04
#define KTD2687_REG_LED0_TORCH_BR	0x05
#define KTD2687_REG_LED1_TORCH_BR	0x06
#define KTD2687_REG_FLASH_TOUT		0x08
#define KTD2687_REG_FLAG1		0x0a
#define KTD2687_REG_FLAG2		0x0b

/* REG_ENABLE */
#define KTD2687_EN_LED_MASK		GENMASK(1, 0)
#define KTD2687_EN_LED_ON		GENMASK(1, 0)
#define KTD2687_EN_MODE_MASK		GENMASK(3, 2)
#define KTD2687_MODE_SHDN		0x0
#define KTD2687_MODE_FLASH		0x1
#define KTD2687_MODE_TORCH		0x2

/* Per-string current registers: 7-bit code, bit 7 is left cleared. */
#define KTD2687_BR_MASK			GENMASK(6, 0)

/* Torch current: (code + 1) x 1464.84 uA, code 0..0x7f. */
#define KTD2687_TORCH_STEP_UA		1465
#define KTD2687_TORCH_CODE_MAX		0x7f

/*
 * Torch scale exposed to userspace: LED level N means N x 1000 uA of
 * combined current, the same numbering the vendor stack and the Android
 * camera HAL use for this chip.
 *
 * The vendor conversion truncates (per_string / 1465) - 1 into the 7-bit
 * register, so level 377 still lands on code 0x7f and 378 is the first
 * level that would overflow the field.  That is exactly where the HAL's
 * 1..377 range comes from, and this macro reproduces the same boundary.
 */
#define KTD2687_TORCH_LEVEL_UA		1000
#define KTD2687_TORCH_LEVEL_MAX		\
	((KTD2687_TORCH_CODE_MAX + 2) * KTD2687_TORCH_STEP_UA * 2 / \
	 KTD2687_TORCH_LEVEL_UA)

/* Flash current: (code + 1) x 11718.75 uA, code 0..0x7f. */
#define KTD2687_FLASH_STEP_UA		11719
#define KTD2687_FLASH_CODE_MAX		0x7f

/*
 * Flash safety timer: 10 ms..100 ms in 10 ms steps (codes 0x00..0x08),
 * then 150 ms..400 ms in 50 ms steps (codes 0x09..0x0d).
 */
#define KTD2687_TOUT_MIN_US		10000
#define KTD2687_TOUT_MAX_US		400000
#define KTD2687_TOUT_FINE_STEP_US	10000
#define KTD2687_TOUT_FINE_MAX_US	100000
#define KTD2687_TOUT_COARSE_STEP_US	50000

/* Fault bits in REG_FLAG1. */
#define KTD2687_FLAG1_TIMEOUT		BIT(0)
#define KTD2687_FLAG1_THERMAL		BIT(2)
#define KTD2687_FLAG1_LED1_SHORT	BIT(4)
#define KTD2687_FLAG1_LED0_SHORT	BIT(5)

struct ktd2687_priv {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;		/* serialises register access */
	struct led_classdev_flash flash;
};

static struct ktd2687_priv *to_ktd2687(struct led_classdev *lcdev)
{
	return container_of(lcdev, struct ktd2687_priv, flash.led_cdev);
}

static const struct regmap_config ktd2687_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = KTD2687_REG_FLAG2,
};

/* Convert a combined (both strings) torch current to a per-string code. */
static u8 ktd2687_torch_code_from_ua(u32 combined_ua)
{
	u32 per_string_ua = combined_ua / 2;
	u32 code;

	if (per_string_ua < KTD2687_TORCH_STEP_UA)
		return 0;

	code = per_string_ua / KTD2687_TORCH_STEP_UA - 1;
	return min_t(u32, code, KTD2687_TORCH_CODE_MAX);
}

/* Convert a combined (both strings) flash current to a per-string code. */
static u8 ktd2687_flash_code_from_ua(u32 combined_ua)
{
	u32 per_string_ua = combined_ua / 2;
	u32 code;

	if (per_string_ua < KTD2687_FLASH_STEP_UA)
		return 0;

	code = per_string_ua / KTD2687_FLASH_STEP_UA - 1;
	return min_t(u32, code, KTD2687_FLASH_CODE_MAX);
}

static int ktd2687_write_current(struct ktd2687_priv *priv, u32 reg0, u32 reg1,
				 u8 code)
{
	int ret;

	ret = regmap_update_bits(priv->regmap, reg0, KTD2687_BR_MASK, code);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, reg1, KTD2687_BR_MASK, code);
}

static int ktd2687_set_mode(struct ktd2687_priv *priv, u8 mode)
{
	return regmap_update_bits(priv->regmap, KTD2687_REG_ENABLE,
				  KTD2687_EN_MODE_MASK,
				  mode << 2);
}

static int ktd2687_set_enabled(struct ktd2687_priv *priv, bool on)
{
	return regmap_update_bits(priv->regmap, KTD2687_REG_ENABLE,
				  KTD2687_EN_LED_MASK,
				  on ? KTD2687_EN_LED_ON : 0);
}

/*
 * Return whether the chip is currently driving the strings in torch mode.
 * This is read back from the chip rather than cached so that a write error,
 * or the flash safety timer expiring, can never leave the cached state
 * disagreeing with the hardware.
 */
static int ktd2687_is_torch_on(struct ktd2687_priv *priv, bool *on)
{
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, KTD2687_REG_ENABLE, &val);
	if (ret)
		return ret;

	*on = ((val & KTD2687_EN_MODE_MASK) >> 2) == KTD2687_MODE_TORCH &&
	      (val & KTD2687_EN_LED_MASK);
	return 0;
}

/*
 * Torch brightness.  Levels are mapped 1:1 onto the combined-current scale
 * used by the vendor stack, i.e. LED level N is N x 1000 uA in total.  A
 * level change while the torch is already on only rewrites the current
 * registers, so the LED never drops out - that is what makes ramps smooth.
 */
static int ktd2687_torch_brightness_set(struct led_classdev *lcdev,
					enum led_brightness level)
{
	struct ktd2687_priv *priv = to_ktd2687(lcdev);
	bool on;
	u8 code;
	int ret;

	mutex_lock(&priv->lock);

	if (level == LED_OFF) {
		ret = ktd2687_set_enabled(priv, false);
		if (!ret)
			ret = ktd2687_set_mode(priv, KTD2687_MODE_SHDN);
		goto unlock;
	}

	ret = ktd2687_is_torch_on(priv, &on);
	if (ret)
		goto unlock;

	code = ktd2687_torch_code_from_ua((u32)level * KTD2687_TORCH_LEVEL_UA);

	/* Current first: the LED must not light up at the previous level. */
	ret = ktd2687_write_current(priv, KTD2687_REG_LED0_TORCH_BR,
				    KTD2687_REG_LED1_TORCH_BR, code);
	if (ret || on)
		goto unlock;

	ret = ktd2687_set_mode(priv, KTD2687_MODE_TORCH);
	if (!ret)
		ret = ktd2687_set_enabled(priv, true);

unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static enum led_brightness ktd2687_torch_brightness_get(
						struct led_classdev *lcdev)
{
	struct ktd2687_priv *priv = to_ktd2687(lcdev);
	unsigned int val;
	bool on;
	int ret;

	mutex_lock(&priv->lock);

	ret = ktd2687_is_torch_on(priv, &on);
	if (ret || !on) {
		ret = LED_OFF;
		goto unlock;
	}

	ret = regmap_read(priv->regmap, KTD2687_REG_LED0_TORCH_BR, &val);
	if (ret < 0)
		goto unlock;

	/* Report the same combined-current scale the setter accepts. */
	ret = ((val & KTD2687_BR_MASK) + 1) * KTD2687_TORCH_STEP_UA * 2 /
	      KTD2687_TORCH_LEVEL_UA;

unlock:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ktd2687_flash_brightness_set(struct led_classdev_flash *fled_cdev,
					u32 brightness)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	u8 code = ktd2687_flash_code_from_ua(brightness);
	int ret;

	mutex_lock(&priv->lock);
	ret = ktd2687_write_current(priv, KTD2687_REG_LED0_FLASH_BR,
				    KTD2687_REG_LED1_FLASH_BR, code);
	mutex_unlock(&priv->lock);

	return ret;
}

static int ktd2687_flash_brightness_get(struct led_classdev_flash *fled_cdev,
					u32 *brightness)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	unsigned int val;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, KTD2687_REG_LED0_FLASH_BR, &val);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	*brightness = (u32)((val & KTD2687_BR_MASK) + 1) *
		      KTD2687_FLASH_STEP_UA * 2;
	return 0;
}

static int ktd2687_flash_strobe_set(struct led_classdev_flash *fled_cdev,
				    bool state)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	int ret;

	mutex_lock(&priv->lock);

	if (state) {
		/* Flash mode drives both strings together. */
		ret = ktd2687_set_mode(priv, KTD2687_MODE_FLASH);
		if (!ret)
			ret = ktd2687_set_enabled(priv, true);
	} else {
		ret = ktd2687_set_enabled(priv, false);
		if (!ret)
			ret = ktd2687_set_mode(priv, KTD2687_MODE_SHDN);
	}

	mutex_unlock(&priv->lock);
	return ret;
}

static int ktd2687_flash_strobe_get(struct led_classdev_flash *fled_cdev,
				    bool *state)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	unsigned int val;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, KTD2687_REG_ENABLE, &val);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	*state = ((val & KTD2687_EN_MODE_MASK) >> 2) == KTD2687_MODE_FLASH &&
		 (val & KTD2687_EN_LED_MASK);
	return 0;
}

static int ktd2687_flash_timeout_set(struct led_classdev_flash *fled_cdev,
				     u32 timeout)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	u8 code;
	int ret;

	if (timeout <= KTD2687_TOUT_FINE_MAX_US)
		code = timeout / KTD2687_TOUT_FINE_STEP_US - 1;
	else
		code = 0x09 + (timeout - KTD2687_TOUT_FINE_MAX_US) /
			      KTD2687_TOUT_COARSE_STEP_US;

	mutex_lock(&priv->lock);
	ret = regmap_update_bits(priv->regmap, KTD2687_REG_FLASH_TOUT,
				 GENMASK(4, 0), code);
	mutex_unlock(&priv->lock);

	return ret;
}

static int ktd2687_flash_fault_get(struct led_classdev_flash *fled_cdev,
				   u32 *fault)
{
	struct ktd2687_priv *priv = container_of(fled_cdev,
						 struct ktd2687_priv, flash);
	unsigned int val;
	u32 faults = 0;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, KTD2687_REG_FLAG1, &val);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	if (val & (KTD2687_FLAG1_LED0_SHORT | KTD2687_FLAG1_LED1_SHORT))
		faults |= LED_FAULT_SHORT_CIRCUIT;
	if (val & KTD2687_FLAG1_THERMAL)
		faults |= LED_FAULT_OVER_TEMPERATURE;
	if (val & KTD2687_FLAG1_TIMEOUT)
		faults |= LED_FAULT_TIMEOUT;

	*fault = faults;
	return 0;
}

static const struct led_flash_ops ktd2687_flash_ops = {
	.flash_brightness_set = ktd2687_flash_brightness_set,
	.flash_brightness_get = ktd2687_flash_brightness_get,
	.strobe_set = ktd2687_flash_strobe_set,
	.strobe_get = ktd2687_flash_strobe_get,
	.timeout_set = ktd2687_flash_timeout_set,
	.fault_get = ktd2687_flash_fault_get,
};

static void ktd2687_init_properties(struct ktd2687_priv *priv,
				    struct fwnode_handle *fwnode)
{
	struct led_classdev_flash *flash = &priv->flash;
	struct led_classdev *lcdev = &flash->led_cdev;
	struct led_flash_setting *s;
	u32 ua;

	/*
	 * Torch scale: level N is N x 1000 uA of combined current, matching
	 * the numbering the vendor stack and Android camera HAL have always
	 * used for this chip.  led-max-microamp therefore sets max_brightness
	 * directly (377000 uA -> 377 levels).
	 */
	if (!fwnode_property_read_u32(fwnode, "led-max-microamp", &ua) &&
	    ua >= KTD2687_TORCH_LEVEL_UA)
		lcdev->max_brightness = ua / KTD2687_TORCH_LEVEL_UA;
	else
		lcdev->max_brightness = KTD2687_TORCH_LEVEL_MAX;

	if (lcdev->max_brightness > KTD2687_TORCH_LEVEL_MAX)
		lcdev->max_brightness = KTD2687_TORCH_LEVEL_MAX;

	lcdev->brightness_set_blocking = ktd2687_torch_brightness_set;
	lcdev->brightness_get = ktd2687_torch_brightness_get;
	lcdev->flags |= LED_DEV_CAP_FLASH;

	s = &flash->brightness;
	s->min = KTD2687_FLASH_STEP_UA * 2;
	/*
	 * One code of the 7-bit flash register is FLASH_STEP_UA per string,
	 * i.e. twice that combined.  Using that as the step lets the class
	 * round a requested current onto an exact register code, so what the
	 * sysfs attribute reports back is what the chip really got.
	 */
	s->step = KTD2687_FLASH_STEP_UA * 2;
	if (!fwnode_property_read_u32(fwnode, "flash-max-microamp", &ua))
		s->max = s->val = clamp_val(ua, s->min,
					    KTD2687_FLASH_CODE_MAX *
					    KTD2687_FLASH_STEP_UA * 2);
	else
		s->max = s->val = KTD2687_FLASH_CODE_MAX *
				  KTD2687_FLASH_STEP_UA * 2;

	s = &flash->timeout;
	s->min = KTD2687_TOUT_MIN_US;
	s->step = KTD2687_TOUT_FINE_STEP_US;
	s->max = s->val = KTD2687_TOUT_MAX_US;
	if (!fwnode_property_read_u32(fwnode, "flash-max-timeout-us", &ua))
		s->max = s->val = clamp_val(ua, s->min, KTD2687_TOUT_MAX_US);

	flash->ops = &ktd2687_flash_ops;
}

static int ktd2687_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct led_init_data init_data = {};
	struct fwnode_handle *fwnode;
	struct ktd2687_priv *priv;
	unsigned int val;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	mutex_init(&priv->lock);

	priv->regmap = devm_regmap_init_i2c(client, &ktd2687_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to allocate register map\n");

	/* Bring the chip out of shutdown before touching any register. */
	ret = regmap_read(priv->regmap, KTD2687_REG_FLAG1, &val);
	if (ret)
		return dev_err_probe(dev, ret, "chip not responding\n");

	/* Both strings off, flash safety timer at its maximum. */
	ret = ktd2687_set_enabled(priv, false);
	if (!ret)
		ret = ktd2687_set_mode(priv, KTD2687_MODE_SHDN);
	if (!ret)
		ret = regmap_update_bits(priv->regmap, KTD2687_REG_FLASH_TOUT,
					 GENMASK(4, 0), 0x0d);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise chip\n");

	/* Clear any stale fault latches. */
	regmap_read(priv->regmap, KTD2687_REG_FLAG1, &val);
	regmap_read(priv->regmap, KTD2687_REG_FLAG2, &val);

	fwnode = device_get_next_child_node(dev, NULL);
	if (!fwnode)
		fwnode = dev_fwnode(dev);
	init_data.fwnode = fwnode;
	init_data.devicename = "ktd2687";
	init_data.default_label = ":flash";

	ktd2687_init_properties(priv, fwnode);

	ret = devm_led_classdev_flash_register_ext(dev, &priv->flash,
						   &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register flash LED\n");

	i2c_set_clientdata(client, priv);

	dev_info(dev, "registered, torch max %d (x1000 uA), flash max %u uA\n",
		 priv->flash.led_cdev.max_brightness, priv->flash.brightness.max);
	return 0;
}

static void ktd2687_remove(struct i2c_client *client)
{
	struct ktd2687_priv *priv = i2c_get_clientdata(client);

	mutex_lock(&priv->lock);
	ktd2687_set_enabled(priv, false);
	ktd2687_set_mode(priv, KTD2687_MODE_SHDN);
	mutex_unlock(&priv->lock);
}

static void ktd2687_shutdown(struct i2c_client *client)
{
	ktd2687_remove(client);
}

static const struct of_device_id ktd2687_of_match[] = {
	{ .compatible = "kinetic,ktd2687" },
	{ }
};
MODULE_DEVICE_TABLE(of, ktd2687_of_match);

static const struct i2c_device_id ktd2687_id_table[] = {
	{ "ktd2687" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktd2687_id_table);

static struct i2c_driver ktd2687_driver = {
	.driver = {
		.name = "ktd2687",
		.of_match_table = ktd2687_of_match,
	},
	.probe = ktd2687_probe,
	.remove = ktd2687_remove,
	.shutdown = ktd2687_shutdown,
	.id_table = ktd2687_id_table,
};
module_i2c_driver(ktd2687_driver);

MODULE_AUTHOR("xaga mainline port");
MODULE_DESCRIPTION("Kinetic KTD2687 dual-channel flash LED driver");
MODULE_LICENSE("GPL");
