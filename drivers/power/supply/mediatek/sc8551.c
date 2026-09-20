// SPDX-License-Identifier: GPL-2.0-only
/*
 * SouthChip SC8551 / SC8551A 2:1 charge-pump driver (mainline port)
 *
 * The register map and general behaviour are derived from the downstream
 * MediaTek SC8551 driver.  This first stage only probes the part, applies
 * safe protection defaults, keeps the charge pump disabled and exposes
 * diagnostics through power_supply plus the existing MediaTek charger class,
 * so that a future CP manager can drive it.
 *
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#include "charger_class.h"
#include "sc8551.h"

#define SC8551_DEVICE_ID	0x51

/* cp_set_mode() values (shared with the MTK charger stack) */
#define SC8551_MODE_4_1		0
#define SC8551_MODE_2_1		1
#define SC8551_MODE_1_1		2

enum sc8551_role {
	SC8551_ROLE_SLAVE = 0,
	SC8551_ROLE_MASTER,
};

struct sc8551_chip {
	struct device *dev;
	struct regmap *regmap;
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum sc8551_role role;
	const char *name;
	struct dentry *dbgfs;
	unsigned int fault_count;
};

static const struct regmap_config sc8551_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x31,
};

/* ------------------------------------------------------------------ */
/* register helpers                                                   */
/* ------------------------------------------------------------------ */

static int sc8551_read_adc(struct sc8551_chip *chip, u8 reg, u16 mask,
			   int step, int rate, int *value)
{
	u8 data[2];
	int ret;

	ret = regmap_raw_read(chip->regmap, reg, data, sizeof(data));
	if (ret)
		return ret;

	*value = (((data[0] << 8) | data[1]) & mask) * step / rate;
	return 0;
}

static int sc8551_set_bat_ovp(struct sc8551_chip *chip, int mv)
{
	u8 data;

	if (mv < SC8551_BAT_OVP_BASE)
		mv = SC8551_BAT_OVP_BASE;
	data = ((mv - SC8551_BAT_OVP_BASE) / SC8551_BAT_OVP_LSB)
	       << SC8551_BAT_OVP_SHIFT;

	return regmap_update_bits(chip->regmap, SC8551_BAT_OVP_REG,
				  SC8551_BAT_OVP_MASK, data);
}

static int sc8551_set_bus_ovp(struct sc8551_chip *chip, int mv)
{
	u8 data;

	if (mv < SC8551_BUS_OVP_BASE)
		mv = SC8551_BUS_OVP_BASE;
	data = ((mv - SC8551_BUS_OVP_BASE) / SC8551_BUS_OVP_LSB)
	       << SC8551_BUS_OVP_SHIFT;

	return regmap_update_bits(chip->regmap, SC8551_BUS_OVP_REG,
				  SC8551_BUS_OVP_MASK, data);
}

static int sc8551_set_ac_ovp(struct sc8551_chip *chip, int mv)
{
	u8 data;

	if (mv < SC8551_VAC_OVP_BASE)
		mv = SC8551_VAC_OVP_BASE;
	data = ((mv - SC8551_VAC_OVP_BASE) / SC8551_VAC_OVP_LSB)
	       << SC8551_VAC_OVP_SHIFT;

	return regmap_update_bits(chip->regmap, SC8551_AC_PROTECT_REG,
				  SC8551_VAC_OVP_MASK, data);
}

static int sc8551_enable_charge(struct sc8551_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8551_CHG_CTRL_REG,
				  SC8551_ENABLE_CHG_BIT,
				  en ? SC8551_ENABLE_CHG_BIT : 0);
}

static int sc8551_is_charge_enabled(struct sc8551_chip *chip, bool *en)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, SC8551_CHG_CTRL_REG, &val);
	if (ret)
		return ret;

	*en = !!(val & SC8551_ENABLE_CHG_BIT);
	return 0;
}

static int sc8551_enable_adc(struct sc8551_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8551_ADC_CTRL_REG,
				  SC8551_ENABLE_ADC_BIT,
				  en ? SC8551_ENABLE_ADC_BIT : 0);
}

static int sc8551_enable_bypass(struct sc8551_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8551_BYPASS_REG,
				  SC8551_ENABLE_BYPASS_BIT,
				  en ? SC8551_ENABLE_BYPASS_BIT : 0);
}

static int sc8551_is_bypass_enabled(struct sc8551_chip *chip, bool *en)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, SC8551_BYPASS_REG, &val);
	if (ret)
		return ret;

	*en = !!(val & SC8551_ENABLE_BYPASS_BIT);
	return 0;
}

/* ------------------------------------------------------------------ */
/* fault IRQ                                                          */
/* ------------------------------------------------------------------ */

static irqreturn_t sc8551_irq_thread(int irq, void *data)
{
	struct sc8551_chip *chip = data;
	unsigned int stat0 = 0, stat1 = 0, int_flag = 0;

	regmap_read(chip->regmap, SC8551_FLT_STAT0_REG, &stat0);
	regmap_read(chip->regmap, SC8551_FLT_STAT1_REG, &stat1);
	regmap_read(chip->regmap, SC8551_INT_FLAG_REG, &int_flag);

	if (stat0 & (SC8551_BAT_OVP_STAT_BIT | SC8551_BAT_OCP_STAT_BIT |
		     SC8551_BUS_OVP_STAT_BIT | SC8551_BUS_OCP_STAT_BIT)) {
		chip->fault_count++;
		dev_warn(chip->dev,
			 "fault: stat0=%#x stat1=%#x int=%#x count=%u\n",
			 stat0, stat1, int_flag, chip->fault_count);
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* debugfs controls (bring-up / diagnostics)                          */
/* ------------------------------------------------------------------ */

static int sc8551_dbgfs_enable_get(void *data, u64 *val)
{
	struct sc8551_chip *chip = data;
	bool en;
	int ret;

	ret = sc8551_is_charge_enabled(chip, &en);
	if (!ret)
		*val = en;
	return ret;
}

static int sc8551_dbgfs_enable_set(void *data, u64 val)
{
	return sc8551_enable_charge(data, val != 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8551_enable_fops, sc8551_dbgfs_enable_get,
			 sc8551_dbgfs_enable_set, "%llu\n");

static int sc8551_dbgfs_bypass_get(void *data, u64 *val)
{
	struct sc8551_chip *chip = data;
	bool en;
	int ret;

	ret = sc8551_is_bypass_enabled(chip, &en);
	if (!ret)
		*val = en;
	return ret;
}

static int sc8551_dbgfs_bypass_set(void *data, u64 val)
{
	return sc8551_enable_bypass(data, val != 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8551_bypass_fops, sc8551_dbgfs_bypass_get,
			 sc8551_dbgfs_bypass_set, "%llu\n");

#define SC8551_DBGFS_ADC(_name, _reg, _mask, _lsb, _rate)		\
	static int sc8551_dbgfs_##_name##_get(void *data, u64 *val)	\
	{								\
		struct sc8551_chip *chip = data;			\
		int v, ret;						\
									\
		ret = sc8551_read_adc(chip, _reg, _mask, _lsb, _rate, &v); \
		if (!ret)						\
			*val = v;					\
		return ret;						\
	}								\
	DEFINE_DEBUGFS_ATTRIBUTE(sc8551_##_name##_fops,			\
				 sc8551_dbgfs_##_name##_get, NULL, "%llu\n")

SC8551_DBGFS_ADC(vbus, SC8551_ADC_VBUS_REG, SC8551_ADC_VBUS_MASK,
		 SC8551_ADC_VBUS_LSB, SC8551_ADC_VBUS_RATE);
SC8551_DBGFS_ADC(ibus, SC8551_ADC_IBUS_REG, SC8551_ADC_IBUS_MASK,
		 SC8551_ADC_IBUS_LSB, SC8551_ADC_IBUS_RATE);
SC8551_DBGFS_ADC(vbatt, SC8551_ADC_VBAT_REG, SC8551_ADC_VBAT_MASK,
		 SC8551_ADC_VBAT_LSB, SC8551_ADC_VBAT_RATE);
SC8551_DBGFS_ADC(ibatt, SC8551_ADC_IBAT_REG, SC8551_ADC_IBAT_MASK,
		 SC8551_ADC_IBAT_LSB, SC8551_ADC_IBAT_RATE);

static int sc8551_dbgfs_faults_get(void *data, u64 *val)
{
	struct sc8551_chip *chip = data;

	*val = READ_ONCE(chip->fault_count);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8551_faults_fops, sc8551_dbgfs_faults_get,
			 NULL, "%llu\n");

/* ------------------------------------------------------------------ */
/* charger_class ops                                                  */
/* ------------------------------------------------------------------ */

static int sc8551_ops_enable(struct charger_device *chg_dev, bool en)
{
	return sc8551_enable_charge(charger_get_data(chg_dev), en);
}

static int sc8551_ops_is_enabled(struct charger_device *chg_dev, bool *en)
{
	return sc8551_is_charge_enabled(charger_get_data(chg_dev), en);
}

static int sc8551_ops_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	struct sc8551_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8551_read_adc(chip, SC8551_ADC_VBUS_REG, SC8551_ADC_VBUS_MASK,
			      SC8551_ADC_VBUS_LSB, SC8551_ADC_VBUS_RATE, &val);
	if (!ret)
		*vbus = val * 1000;
	return ret;
}

static int sc8551_ops_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	struct sc8551_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8551_read_adc(chip, SC8551_ADC_IBUS_REG, SC8551_ADC_IBUS_MASK,
			      SC8551_ADC_IBUS_LSB, SC8551_ADC_IBUS_RATE, &val);
	if (!ret)
		*ibus = val * 1000;
	return ret;
}

static int sc8551_ops_get_vbatt(struct charger_device *chg_dev, u32 *vbatt)
{
	struct sc8551_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8551_read_adc(chip, SC8551_ADC_VBAT_REG, SC8551_ADC_VBAT_MASK,
			      SC8551_ADC_VBAT_LSB, SC8551_ADC_VBAT_RATE, &val);
	if (!ret)
		*vbatt = val * 1000;
	return ret;
}

static int sc8551_ops_get_ibatt(struct charger_device *chg_dev, u32 *ibatt)
{
	struct sc8551_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8551_read_adc(chip, SC8551_ADC_IBAT_REG, SC8551_ADC_IBAT_MASK,
			      SC8551_ADC_IBAT_LSB, SC8551_ADC_IBAT_RATE, &val);
	if (!ret)
		*ibatt = val * 1000;
	return ret;
}

static int sc8551_ops_set_mode(struct charger_device *chg_dev, int mode)
{
	struct sc8551_chip *chip = charger_get_data(chg_dev);

	/*
	 * The SC8551 is a fixed 2:1 divider; "1:1" is its bypass path.
	 * 4:1 belongs to the SC8561 (pearlpro) and is not supported here.
	 */
	switch (mode) {
	case SC8551_MODE_2_1:
		return sc8551_enable_bypass(chip, false);
	case SC8551_MODE_1_1:
		return sc8551_enable_bypass(chip, true);
	default:
		return -EOPNOTSUPP;
	}
}

static int sc8551_ops_is_bypass_enabled(struct charger_device *chg_dev,
					bool *en)
{
	return sc8551_is_bypass_enabled(charger_get_data(chg_dev), en);
}

static int sc8551_ops_mode_init(struct charger_device *chg_dev, int value)
{
	return 0;
}

static int sc8551_ops_bypass_support(struct charger_device *chg_dev, bool *en)
{
	*en = true;
	return 0;
}

static int sc8551_ops_enable_adc(struct charger_device *chg_dev, bool en)
{
	return sc8551_enable_adc(charger_get_data(chg_dev), en);
}

static const struct charger_ops sc8551_chg_ops = {
	.enable = sc8551_ops_enable,
	.is_enabled = sc8551_ops_is_enabled,
	.get_vbus_adc = sc8551_ops_get_vbus,
	.get_ibus_adc = sc8551_ops_get_ibus,
	.cp_get_vbatt = sc8551_ops_get_vbatt,
	.cp_get_ibatt = sc8551_ops_get_ibatt,
	.cp_set_mode = sc8551_ops_set_mode,
	.is_bypass_enabled = sc8551_ops_is_bypass_enabled,
	.cp_device_init = sc8551_ops_mode_init,
	.cp_get_bypass_support = sc8551_ops_bypass_support,
	.cp_enable_adc = sc8551_ops_enable_adc,
};

/* ------------------------------------------------------------------ */
/* power_supply diagnostics                                           */
/* ------------------------------------------------------------------ */

static enum power_supply_property sc8551_psy_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int sc8551_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sc8551_chip *chip = power_supply_get_drvdata(psy);
	bool en;
	int v, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sc8551_is_charge_enabled(chip, &en);
		if (ret)
			return ret;
		val->intval = en;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc8551_is_charge_enabled(chip, &en);
		if (ret)
			return ret;
		val->intval = en ? POWER_SUPPLY_STATUS_CHARGING :
				   POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8551_read_adc(chip, SC8551_ADC_VBUS_REG,
				      SC8551_ADC_VBUS_MASK,
				      SC8551_ADC_VBUS_LSB,
				      SC8551_ADC_VBUS_RATE, &v);
		if (ret)
			return ret;
		val->intval = v * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8551_read_adc(chip, SC8551_ADC_IBUS_REG,
				      SC8551_ADC_IBUS_MASK,
				      SC8551_ADC_IBUS_LSB,
				      SC8551_ADC_IBUS_RATE, &v);
		if (ret)
			return ret;
		val->intval = v * 1000;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "SouthChip";
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SC8551A";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

static int sc8551_probe(struct i2c_client *client)
{
	struct sc8551_chip *chip;
	struct power_supply_config psy_cfg = {};
	struct charger_properties chg_props = {};
	unsigned int id;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->role = (enum sc8551_role)(uintptr_t)i2c_get_match_data(client);
	chip->name = chip->role == SC8551_ROLE_MASTER ? "cp_master" : "cp_slave";
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &sc8551_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	ret = regmap_read(chip->regmap, SC8551_PART_INFO_REG, &id);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "failed to read part info\n");
	if (id != SC8551_DEVICE_ID)
		return dev_err_probe(chip->dev, -ENODEV,
				     "unexpected part id %#x\n", id);

	/*
	 * Disable the watchdog.  Nothing in this driver kicks it, and an
	 * expired watchdog silently turns the pump off mid-charge (the
	 * downstream driver disables it for the same reason).
	 */
	regmap_update_bits(chip->regmap, SC8551_CTRL_REG,
			   SC8551_WD_TIMEOUT_DIS_BIT,
			   SC8551_WD_TIMEOUT_DIS_BIT);

	/* Keep the pump off until a CP manager drives it. */
	sc8551_enable_charge(chip, false);
	sc8551_enable_adc(chip, true);
	sc8551_set_bat_ovp(chip, 5000);
	sc8551_set_bus_ovp(chip, 12300);
	sc8551_set_ac_ovp(chip, 13000);


	if (client->irq > 0) {
		ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
						sc8551_irq_thread,
						IRQF_ONESHOT |
						IRQF_TRIGGER_FALLING,
						dev_name(chip->dev), chip);
		if (ret)
			return dev_err_probe(chip->dev, ret,
					     "failed to request irq %d\n",
					     client->irq);
	}

	chg_props.alias_name = chip->name;
	chip->chg_dev = charger_device_register(chip->name, chip->dev, chip,
						&sc8551_chg_ops, &chg_props);
	if (IS_ERR(chip->chg_dev))
		return dev_err_probe(chip->dev, PTR_ERR(chip->chg_dev),
				     "failed to register charger\n");

	chip->psy_desc.name = chip->role == SC8551_ROLE_MASTER ?
			      "sc8551-master" : "sc8551-slave";
	chip->psy_desc.type = POWER_SUPPLY_TYPE_USB;
	chip->psy_desc.properties = sc8551_psy_props;
	chip->psy_desc.num_properties = ARRAY_SIZE(sc8551_psy_props);
	chip->psy_desc.get_property = sc8551_get_property;
	psy_cfg.drv_data = chip;
	chip->psy = devm_power_supply_register(chip->dev, &chip->psy_desc,
					       &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->psy),
				     "failed to register power supply\n");

	chip->dbgfs = debugfs_create_dir(chip->name, NULL);
	debugfs_create_file("enable", 0600, chip->dbgfs, chip,
			    &sc8551_enable_fops);
	debugfs_create_file("bypass", 0600, chip->dbgfs, chip,
			    &sc8551_bypass_fops);
	debugfs_create_file("vbus", 0400, chip->dbgfs, chip,
			    &sc8551_vbus_fops);
	debugfs_create_file("ibus", 0400, chip->dbgfs, chip,
			    &sc8551_ibus_fops);
	debugfs_create_file("vbatt", 0400, chip->dbgfs, chip,
			    &sc8551_vbatt_fops);
	debugfs_create_file("ibatt", 0400, chip->dbgfs, chip,
			    &sc8551_ibatt_fops);
	debugfs_create_file("faults", 0400, chip->dbgfs, chip,
			    &sc8551_faults_fops);

	dev_info(chip->dev, "SC8551 %s probed (part id %#x, irq %d)\n",
		 chip->role == SC8551_ROLE_MASTER ? "master" : "slave", id,
		 client->irq);

	return 0;
}

static void sc8551_remove(struct i2c_client *client)
{
	struct sc8551_chip *chip = i2c_get_clientdata(client);

	sc8551_enable_charge(chip, false);
	debugfs_remove_recursive(chip->dbgfs);
	charger_device_unregister(chip->chg_dev);
}

static const struct i2c_device_id sc8551_i2c_ids[] = {
	{ "sc8551-master", SC8551_ROLE_MASTER },
	{ "sc8551-slave", SC8551_ROLE_SLAVE },
	{}
};
MODULE_DEVICE_TABLE(i2c, sc8551_i2c_ids);

static const struct of_device_id sc8551_of_match[] = {
	{ .compatible = "southchip,sc8551-master",
	  .data = (void *)SC8551_ROLE_MASTER },
	{ .compatible = "southchip,sc8551-slave",
	  .data = (void *)SC8551_ROLE_SLAVE },
	{}
};
MODULE_DEVICE_TABLE(of, sc8551_of_match);

static struct i2c_driver sc8551_driver = {
	.driver = {
		.name = "sc8551",
		.of_match_table = sc8551_of_match,
	},
	.probe = sc8551_probe,
	.remove = sc8551_remove,
	.id_table = sc8551_i2c_ids,
};
module_i2c_driver(sc8551_driver);

MODULE_AUTHOR("pearl mainline port");
MODULE_DESCRIPTION("SouthChip SC8551 charge-pump driver");
MODULE_LICENSE("GPL");
