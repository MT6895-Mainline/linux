// SPDX-License-Identifier: GPL-2.0-only
/*
 * Onsemi FAN53870 camera PMIC.
 * Register definitions: FAN53870/D, revision 11, tables 6-20.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

#define FAN53870_PRODUCT_ID	0x00
#define FAN53870_ENABLE		0x03
#define FAN53870_LDO1_VOUT	0x04
#define FAN53870_LDO7_VOUT	0x0a
#define FAN53870_LDO12_SEQ	0x0b
#define FAN53870_LDO7_SEQ		0x0e
#define FAN53870_ID		0x01

struct fan53870_enable {
	struct device *dev;
	struct i2c_adapter *adapter;
	struct gpio_desc *gpio;
};

static void fan53870_disable(void *data)
{
	struct fan53870_enable *enable = data;
	int ret;

	i2c_lock_bus(enable->adapter, I2C_LOCK_SEGMENT);
	ret = gpiod_direction_output(enable->gpio, 0);
	i2c_unlock_bus(enable->adapter, I2C_LOCK_SEGMENT);
	if (ret)
		dev_err(enable->dev, "Failed to deassert board enable: %d\n", ret);
}

static int fan53870_enable_access(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fan53870_enable *enable;
	struct gpio_desc *gpio;
	int ret;

	gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(gpio))
		return dev_err_probe(dev, PTR_ERR(gpio), "Failed to request board enable\n");
	if (!gpio)
		return 0;

	/* An I2C GPIO expander could deadlock while the bus is locked below. */
	if (gpiod_cansleep(gpio))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "Board enable requires a non-sleeping GPIO\n");

	enable = devm_kzalloc(dev, sizeof(*enable), GFP_KERNEL);
	if (!enable)
		return -ENOMEM;
	enable->dev = dev;
	enable->adapter = client->adapter;
	enable->gpio = gpio;

	/* Later regulator registrations unwind before this, then GPIO release. */
	ret = devm_add_action_or_reset(dev, fan53870_disable, enable);
	if (ret)
		return ret;

	/*
	 * This board signal may release RESET_B. The datasheet requires its
	 * rising edge to occur between I2C transactions to avoid false STARTs.
	 * Hold it asserted for the provider lifetime; do not retry with pulses.
	 */
	i2c_lock_bus(client->adapter, I2C_LOCK_SEGMENT);
	ret = gpiod_direction_output(gpio, 1);
	if (!ret)
		msleep(20);
	i2c_unlock_bus(client->adapter, I2C_LOCK_SEGMENT);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to assert board enable\n");

	return 0;
}

/* Selector zero selects a factory default, not a programmable voltage. */
static const struct linear_range fan53870_low_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x63, 0xbb, 8000),
};

static const struct linear_range fan53870_high_ranges[] = {
	REGULATOR_LINEAR_RANGE(1500000, 0x10, 0xff, 8000),
};

static int fan53870_get_voltage_sel(struct regulator_dev *rdev)
{
	int sel;

	sel = regulator_get_voltage_sel_regmap(rdev);
	if (sel < 0)
		return sel;

	/*
	 * Do not infer a voltage from the factory-default selector. Let the
	 * regulator core apply this output's explicit DT voltage constraints.
	 */
	if (!sel)
		return -ENOTRECOVERABLE;
	if (regulator_list_voltage_linear_range(rdev, sel) <= 0)
		return -EINVAL;

	return sel;
}

static const struct regulator_ops fan53870_ops = {
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.get_voltage_sel = fan53870_get_voltage_sel,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

#define FAN53870_LDO(_num, _supply, _ranges, _count) {		\
	.name = "ldo" #_num,					\
	.of_match = "ldo" #_num,					\
	.regulators_node = "regulators",				\
	.id = (_num) - 1,					\
	.type = REGULATOR_VOLTAGE,				\
	.owner = THIS_MODULE,					\
	.ops = &fan53870_ops,					\
	.linear_ranges = _ranges,				\
	.n_linear_ranges = ARRAY_SIZE(_ranges),			\
	.n_voltages = _count,					\
	.vsel_reg = FAN53870_LDO1_VOUT + (_num) - 1,		\
	.vsel_mask = 0xff,					\
	.enable_reg = FAN53870_ENABLE,				\
	.enable_mask = BIT((_num) - 1),				\
	.enable_time = 1500,					\
	.supply_name = _supply,					\
}

static const struct regulator_desc fan53870_regulators[] = {
	FAN53870_LDO(1, "vin12", fan53870_low_ranges, 0xbc),
	FAN53870_LDO(2, "vin12", fan53870_low_ranges, 0xbc),
	FAN53870_LDO(3, "vin34", fan53870_high_ranges, 0x100),
	FAN53870_LDO(4, "vin34", fan53870_high_ranges, 0x100),
	FAN53870_LDO(5, "vin5", fan53870_high_ranges, 0x100),
	FAN53870_LDO(6, "vin6", fan53870_high_ranges, 0x100),
	FAN53870_LDO(7, "vin7", fan53870_high_ranges, 0x100),
};

static bool fan53870_writeable_reg(struct device *dev, unsigned int reg)
{
	return reg >= FAN53870_ENABLE && reg <= FAN53870_LDO7_VOUT;
}

static const struct regmap_config fan53870_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FAN53870_LDO7_SEQ,
	.writeable_reg = fan53870_writeable_reg,
};

static int fan53870_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { .dev = dev };
	struct device_node *regulators, *node;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	unsigned int value, present = 0;
	u32 min_uv, max_uv;
	int i, ret;

	ret = fan53870_enable_access(client);
	if (ret)
		return ret;

	regmap = devm_regmap_init_i2c(client, &fan53870_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to create regmap\n");
	config.regmap = regmap;

	ret = regmap_read(regmap, FAN53870_PRODUCT_ID, &value);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read product ID\n");
	if (value != FAN53870_ID)
		return dev_err_probe(dev, -ENODEV, "Unexpected product ID %#x\n", value);

	regulators = of_get_child_by_name(dev->of_node, "regulators");
	if (!regulators)
		return -EINVAL;

	/*
	 * Only take ownership of outputs explicitly described by the board.
	 * Never issue a software reset or rewrite sequencing for another consumer.
	 */
	for (i = 0; i < ARRAY_SIZE(fan53870_regulators); i++) {
		node = of_get_child_by_name(regulators, fan53870_regulators[i].of_match);
		if (!of_device_is_available(node)) {
			of_node_put(node);
			continue;
		}
		ret = of_property_read_u32(node, "regulator-min-microvolt", &min_uv);
		if (!ret)
			ret = of_property_read_u32(node, "regulator-max-microvolt", &max_uv);
		of_node_put(node);
		if (ret || !min_uv || min_uv > max_uv) {
			of_node_put(regulators);
			return dev_err_probe(dev, -EINVAL,
					     "LDO%d needs explicit voltage constraints\n", i + 1);
		}
		present |= BIT(i);
	}
	of_node_put(regulators);
	if (!present)
		return -EINVAL;

	/* The ENABLE bits do not control outputs assigned to a sequencer. */
	for (i = 0; i < ARRAY_SIZE(fan53870_regulators); i++) {
		if (!(present & BIT(i)))
			continue;
		ret = regmap_read(regmap, FAN53870_LDO12_SEQ + i / 2, &value);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to read LDO%d sequence\n", i + 1);
		if (value & (0x7 << (3 * (i % 2))))
			return dev_err_probe(dev, -EOPNOTSUPP,
					     "LDO%d uses unsupported hardware sequencing\n", i + 1);
	}

	for (i = 0; i < ARRAY_SIZE(fan53870_regulators); i++) {
		if (!(present & BIT(i)))
			continue;
		rdev = devm_regulator_register(dev, &fan53870_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Failed to register LDO%d\n", i + 1);
	}

	return 0;
}

static const struct of_device_id fan53870_of_match[] = {
	{ .compatible = "onnn,fan53870" },
	{ }
};
MODULE_DEVICE_TABLE(of, fan53870_of_match);

static struct i2c_driver fan53870_driver = {
	.driver = {
		.name = "fan53870",
		.of_match_table = fan53870_of_match,
	},
	.probe = fan53870_probe,
};
module_i2c_driver(fan53870_driver);

MODULE_DESCRIPTION("Onsemi FAN53870 camera PMIC regulator driver");
MODULE_LICENSE("GPL");
