// SPDX-License-Identifier: GPL-2.0-only
/*
 * SouthChip SC8561 / LN8410 2:1 / 4:1 charge-pump driver (mainline port)
 *
 * xagapro carries two SC8561 pumps (master on i2c9, slave on i2c7) instead
 * of the SC8551A pair used on xaga.  Unlike the SC8551 this part can divide
 * by 4 as well as by 2, so it can run from the high-voltage (15..21 V) PPS
 * APDOs the 120 W-class adapters expose.
 *
 * The register map and the mode/protection sequences are derived from the
 * downstream MediaTek sc8561.c.  The driver keeps the pump disabled and
 * exposes it through the MediaTek charger class for the CP manager.
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
#include "sc8561.h"

/* cp_set_mode() values (shared with the MTK charger stack) */
#define SC8561_MODE_4_1		0
#define SC8561_MODE_2_1		1
#define SC8561_MODE_1_1		2

enum sc8561_role {
	SC8561_ROLE_SLAVE = 0,
	SC8561_ROLE_MASTER,
};

struct sc8561_chip {
	struct device *dev;
	struct regmap *regmap;
	struct charger_device *chg_dev;
	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum sc8561_role role;
	const char *name;
	struct dentry *dbgfs;
	unsigned int fault_count;
	int work_mode;
};

static const struct regmap_config sc8561_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7c,
};

/* ------------------------------------------------------------------ */
/* register helpers                                                   */
/* ------------------------------------------------------------------ */

static int sc8561_read_adc(struct sc8561_chip *chip, int ch, int num,
			   int den, int *value)
{
	unsigned int hi, lo;
	int ret;

	ret = regmap_read(chip->regmap,
			  SC8561_ADC_REG_BASE + (ch << 1), &hi);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap,
			  SC8561_ADC_REG_BASE + (ch << 1) + 1, &lo);
	if (ret)
		return ret;

	*value = (((hi & SC8561_ADC_HI_MASK) << 8) | (lo & 0xff)) *
		 num / den;
	return 0;
}

static int sc8561_enable_charge(struct sc8561_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8561_CHG_CTRL_REG,
				  SC8561_CHG_EN_MASK,
				  en ? SC8561_CHG_EN_MASK : 0);
}

static int sc8561_is_charge_enabled(struct sc8561_chip *chip, bool *en)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, SC8561_CHG_CTRL_REG, &val);
	if (ret)
		return ret;

	*en = !!(val & SC8561_CHG_EN_MASK);
	return 0;
}

static int sc8561_enable_adc(struct sc8561_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8561_ADC_CTRL_REG,
				  SC8561_ADC_EN_MASK,
				  en ? SC8561_ADC_EN_MASK : 0);
}

static int sc8561_set_bat_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (mv < SC8561_BAT_OVP_BASE)
		mv = SC8561_BAT_OVP_BASE;
	val = (mv - SC8561_BAT_OVP_BASE) / SC8561_BAT_OVP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_BAT_OVP_REG,
				  SC8561_BAT_OVP_MASK,
				  val << SC8561_BAT_OVP_SHIFT);
}

static int sc8561_set_bat_ocp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (mv < SC8561_BAT_OCP_BASE)
		mv = SC8561_BAT_OCP_BASE;
	val = (mv - SC8561_BAT_OCP_BASE) / SC8561_BAT_OCP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_BAT_OCP_REG,
				  SC8561_BAT_OCP_MASK,
				  val << SC8561_BAT_OCP_SHIFT);
}

static int sc8561_set_usb_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (mv == 6500)
		val = SC8561_USB_OVP_6PV5;
	else
		val = (mv - SC8561_USB_OVP_BASE) / SC8561_USB_OVP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_USB_OVP_REG,
				  SC8561_USB_OVP_MASK,
				  val << SC8561_USB_OVP_SHIFT);
}

static int sc8561_set_wpc_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	val = (mv - SC8561_WPC_OVP_BASE) / SC8561_WPC_OVP_LSB;
	return regmap_update_bits(chip->regmap, SC8561_WPC_OVP_REG,
				  SC8561_WPC_OVP_MASK,
				  val << SC8561_WPC_OVP_SHIFT);
}

/*
 * Bus OVP has three different scales depending on the divider ratio, so it
 * can only be programmed once the work mode is known.
 */
static int sc8561_set_bus_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (chip->work_mode == SC8561_FORWARD_4_1_CHARGER_MODE) {
		if (mv < 14000)
			mv = 14000;
		else if (mv > 22000)
			mv = 22000;
		val = (mv - SC8561_BUS_OVP_41MODE_BASE) /
		      SC8561_BUS_OVP_41MODE_LSB;
	} else if (chip->work_mode == SC8561_FORWARD_2_1_CHARGER_MODE) {
		if (mv < 7000)
			mv = 7000;
		else if (mv > 13300)
			mv = 13300;
		val = (mv - SC8561_BUS_OVP_21MODE_BASE) /
		      SC8561_BUS_OVP_21MODE_LSB;
	} else {
		if (mv < 3500)
			mv = 3500;
		else if (mv > 5500)
			mv = 5500;
		val = (mv - SC8561_BUS_OVP_11MODE_BASE) /
		      SC8561_BUS_OVP_11MODE_LSB;
	}

	return regmap_update_bits(chip->regmap, SC8561_BUS_OVP_REG,
				  SC8561_BUS_OVP_MASK,
				  val << SC8561_BUS_OVP_SHIFT);
}

static int sc8561_set_out_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (mv < SC8561_OUT_OVP_BASE)
		mv = SC8561_OUT_OVP_BASE;
	val = (mv - SC8561_OUT_OVP_BASE) / SC8561_OUT_OVP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_BUS_OVP_REG,
				  SC8561_OUT_OVP_MASK,
				  val << SC8561_OUT_OVP_SHIFT);
}

static int sc8561_set_bus_ocp(struct sc8561_chip *chip, int mv)
{
	u8 val;

	if (mv < SC8561_BUS_OCP_BASE)
		mv = SC8561_BUS_OCP_BASE;
	val = (mv - SC8561_BUS_OCP_BASE) / SC8561_BUS_OCP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_BUS_OCP_REG,
				  SC8561_BUS_OCP_MASK,
				  val << SC8561_BUS_OCP_SHIFT);
}

static int sc8561_set_pmid2out_ovp(struct sc8561_chip *chip, int mv)
{
	u8 val = (mv - SC8561_PMID2OUT_OVP_BASE) /
		 SC8561_PMID2OUT_OVP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_PMID2OUT_OVP_REG,
				  SC8561_PMID2OUT_OVP_MASK,
				  val << SC8561_PMID2OUT_OVP_SHIFT);
}

static int sc8561_set_pmid2out_uvp(struct sc8561_chip *chip, int mv)
{
	u8 val = (mv - SC8561_PMID2OUT_UVP_BASE) /
		 SC8561_PMID2OUT_UVP_LSB;

	return regmap_update_bits(chip->regmap, SC8561_PMID2OUT_UVP_REG,
				  SC8561_PMID2OUT_UVP_MASK,
				  val << SC8561_PMID2OUT_UVP_SHIFT);
}

/* "enable" helpers clear the DIS bit, which is active high on this part */
#define SC8561_DIS_HELPER(_name, _reg, _bit)				\
	static int sc8561_enable_##_name(struct sc8561_chip *chip, bool en)\
	{								\
		return regmap_update_bits(chip->regmap, _reg, _bit,	\
					  en ? 0 : _bit);		\
	}

SC8561_DIS_HELPER(bat_ovp, SC8561_BAT_OVP_REG, SC8561_BAT_OVP_DIS_MASK)
SC8561_DIS_HELPER(bat_ocp, SC8561_BAT_OCP_REG, SC8561_BAT_OCP_DIS_MASK)
SC8561_DIS_HELPER(bus_ocp, SC8561_BUS_OCP_REG, SC8561_BUS_OCP_DIS_MASK)
SC8561_DIS_HELPER(bus_ucp, SC8561_BUS_UCP_REG, SC8561_BUS_UCP_DIS_MASK)
SC8561_DIS_HELPER(pmid2out_ovp, SC8561_PMID2OUT_OVP_REG,
		  SC8561_PMID2OUT_OVP_DIS_MASK)
SC8561_DIS_HELPER(pmid2out_uvp, SC8561_PMID2OUT_UVP_REG,
		  SC8561_PMID2OUT_UVP_DIS_MASK)

static int sc8561_set_operation_mode(struct sc8561_chip *chip, int mode)
{
	u8 val;

	switch (mode) {
	case SC8561_MODE_4_1:
		val = SC8561_FORWARD_4_1_CHARGER_MODE;
		break;
	case SC8561_MODE_2_1:
		val = SC8561_FORWARD_2_1_CHARGER_MODE;
		break;
	case SC8561_MODE_1_1:
		val = SC8561_FORWARD_1_1_CHARGER_MODE;
		break;
	default:
		return -EINVAL;
	}

	chip->work_mode = val;
	return regmap_update_bits(chip->regmap, SC8561_MODE_REG,
				  SC8561_MODE_MASK,
				  val << SC8561_MODE_SHIFT);
}

static bool sc8561_is_bypass_enabled_hw(struct sc8561_chip *chip)
{
	unsigned int val;

	if (regmap_read(chip->regmap, SC8561_MODE_REG, &val))
		return false;

	return !!(val & SC8561_ENABLE_BYPASS_BIT);
}

static int sc8561_enable_bypass(struct sc8561_chip *chip, bool en)
{
	return regmap_update_bits(chip->regmap, SC8561_MODE_REG,
				  SC8561_ENABLE_BYPASS_BIT,
				  en ? SC8561_ENABLE_BYPASS_BIT : 0);
}

static int sc8561_init_protection(struct sc8561_chip *chip, int mode)
{
	int busovp, busocp, usbovp;

	switch (mode) {
	case SC8561_MODE_4_1:
		busovp = 22000;
		busocp = 3750;
		usbovp = 22000;
		break;
	case SC8561_MODE_2_1:
		busovp = 11000;
		busocp = 6100;
		usbovp = 14000;
		break;
	default:
		busovp = 6500;
		busocp = 6100;
		usbovp = 14000;
		break;
	}

	sc8561_enable_bat_ovp(chip, true);
	sc8561_enable_bat_ocp(chip, false);
	sc8561_enable_bus_ocp(chip, true);
	sc8561_enable_bus_ucp(chip, true);
	sc8561_enable_pmid2out_ovp(chip, true);
	sc8561_enable_pmid2out_uvp(chip, true);

	sc8561_set_bat_ovp(chip, 4650);
	sc8561_set_bat_ocp(chip, 12400);
	sc8561_set_wpc_ovp(chip, 22000);
	sc8561_set_out_ovp(chip, 5000);
	sc8561_set_pmid2out_uvp(chip, 100);
	sc8561_set_pmid2out_ovp(chip, 600);

	sc8561_set_usb_ovp(chip, usbovp);
	sc8561_set_bus_ovp(chip, busovp);
	sc8561_set_bus_ocp(chip, busocp);
	return 0;
}

static int sc8561_init_gates(struct sc8561_chip *chip)
{
	/* ACDRV manual mode + WPC/OVP gate enable (downstream init) */
	regmap_update_bits(chip->regmap, SC8561_CHG_CTRL_REG,
			   SC8561_ACDRV_MANUAL_EN_MASK |
			   SC8561_WPCGATE_EN_MASK |
			   SC8561_OVPGATE_EN_MASK,
			   (SC8561_ACDRV_MANUAL_MODE <<
			    SC8561_ACDRV_MANUAL_EN_SHIFT) |
			   (SC8561_WPCGATE_ENABLE <<
			    SC8561_WPCGATE_EN_SHIFT) |
			   (SC8561_OVPGATE_ENABLE <<
			    SC8561_OVPGATE_EN_SHIFT));

	/* UCP fall deglitch 5 ms */
	regmap_update_bits(chip->regmap, SC8561_BUS_UCP_REG,
			   SC8561_BUS_UCP_FALL_DG_MASK,
			   SC8561_BUS_UCP_FALL_DG_5MS <<
			   SC8561_BUS_UCP_FALL_DG_SHIFT);

	/* TSBAT sense off */
	regmap_update_bits(chip->regmap, SC8561_TSBAT_REG,
			   SC8561_TSBAT_EN_MASK,
			   SC8561_TSBAT_DISABLE << SC8561_TSBAT_EN_SHIFT);

	/* IBAT sense 1 mOhm, sync with no phase shift */
	regmap_update_bits(chip->regmap, SC8561_MODE_REG,
			   SC8561_IBAT_SNS_RES_MASK,
			   SC8561_IBAT_SNS_RES_1MHM <<
			   SC8561_IBAT_SNS_RES_SHIFT);
	regmap_update_bits(chip->regmap, SC8561_FSW_SYNC_REG,
			   SC8561_SYNC_MASK,
			   SC8561_SYNC_NO_SHIFT << SC8561_SYNC_SHIFT);

	/* OVPGATE 20 ms on-deglitch */
	regmap_update_bits(chip->regmap, SC8561_USB_OVP_REG,
			   SC8561_OVPGATE_ON_DG_MASK,
			   SC8561_OVPGATE_ON_DG_20MS <<
			   SC8561_OVPGATE_ON_DG_SHIFT);

	/* drive the external AC FETs up */
	return regmap_update_bits(chip->regmap, SC8561_ACDRV_REG,
				  SC8561_ACDRV_UP_MASK,
				  SC8561_ACDRV_UP_ENABLE <<
				  SC8561_ACDRV_UP_SHIFT);
}

static int sc8561_init_device(struct sc8561_chip *chip)
{
	/*
	 * Nothing kicks the watchdog, and an expired watchdog silently
	 * turns the pump off mid-charge.  Disable it (downstream does the
	 * same).
	 */
	regmap_update_bits(chip->regmap, SC8561_SS_WD_REG,
			   SC8561_WD_TIMEOUT_SET_MASK,
			   SC8561_WD_TIMEOUT_DISABLE <<
			   SC8561_WD_TIMEOUT_SET_SHIFT);
	regmap_update_bits(chip->regmap, SC8561_SS_WD_REG,
			   SC8561_SS_TIMEOUT_SET_MASK,
			   SC8561_SS_TIMEOUT_5120MS <<
			   SC8561_SS_TIMEOUT_SET_SHIFT);

	/* all ADC channels on, continuous conversion */
	regmap_update_bits(chip->regmap, SC8561_ADC_CTRL_REG,
			   SC8561_ADC_RATE_MASK | SC8561_IBUS_ADC_DIS_MASK, 0);
	regmap_write(chip->regmap, SC8561_ADC_FN_DISABLE_REG, 0);

	/* safe default: 2:1 with the matching protection envelope */
	sc8561_set_operation_mode(chip, SC8561_MODE_2_1);
	sc8561_init_protection(chip, SC8561_MODE_2_1);
	sc8561_enable_adc(chip, true);
	sc8561_init_gates(chip);

	return sc8561_enable_charge(chip, false);
}

/* ------------------------------------------------------------------ */
/* fault IRQ                                                          */
/* ------------------------------------------------------------------ */

static irqreturn_t sc8561_irq_thread(int irq, void *data)
{
	struct sc8561_chip *chip = data;
	unsigned int flag = 0, mask = 0;

	regmap_read(chip->regmap, SC8561_FLAG_REG, &flag);
	regmap_read(chip->regmap, SC8561_FLAG_MASK_REG, &mask);

	if (flag & (SC8561_FLAG_TSHUT | SC8561_FLAG_SS_TIMEOUT |
		    SC8561_FLAG_WD_TIMEOUT | SC8561_FLAG_CONV_OCP |
		    SC8561_FLAG_VBUS_OVP | SC8561_FLAG_VOUT_OVP)) {
		chip->fault_count++;
		dev_warn(chip->dev, "fault: flag=%#x mask=%#x count=%u\n",
			 flag, mask, chip->fault_count);
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* debugfs controls (bring-up / diagnostics)                          */
/* ------------------------------------------------------------------ */

static int sc8561_dbgfs_enable_get(void *data, u64 *val)
{
	struct sc8561_chip *chip = data;
	bool en;
	int ret;

	ret = sc8561_is_charge_enabled(chip, &en);
	if (!ret)
		*val = en;
	return ret;
}

static int sc8561_dbgfs_enable_set(void *data, u64 val)
{
	return sc8561_enable_charge(data, val != 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8561_enable_fops, sc8561_dbgfs_enable_get,
			 sc8561_dbgfs_enable_set, "%llu\n");

static int sc8561_dbgfs_bypass_get(void *data, u64 *val)
{
	*val = sc8561_is_bypass_enabled_hw(data);
	return 0;
}

static int sc8561_dbgfs_bypass_set(void *data, u64 val)
{
	return sc8561_enable_bypass(data, val != 0);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8561_bypass_fops, sc8561_dbgfs_bypass_get,
			 sc8561_dbgfs_bypass_set, "%llu\n");

static int sc8561_dbgfs_faults_get(void *data, u64 *val)
{
	struct sc8561_chip *chip = data;

	*val = READ_ONCE(chip->fault_count);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8561_faults_fops, sc8561_dbgfs_faults_get,
			 NULL, "%llu\n");

#define SC8561_DBGFS_ADC(_name, _ch, _num, _den)			\
	static int sc8561_dbgfs_##_name##_get(void *data, u64 *val)	\
	{								\
		struct sc8561_chip *chip = data;			\
		int v, ret;						\
									\
		ret = sc8561_read_adc(chip, _ch, _num, _den, &v);	\
		if (!ret)						\
			*val = v;					\
		return ret;						\
	}								\
	DEFINE_DEBUGFS_ATTRIBUTE(sc8561_##_name##_fops,			\
				 sc8561_dbgfs_##_name##_get, NULL, "%llu\n")

SC8561_DBGFS_ADC(vbus, SC8561_ADC_VBUS, SC8561_VBUS_ADC_NUM,
		 SC8561_VBUS_ADC_DEN);
SC8561_DBGFS_ADC(ibus, SC8561_ADC_IBUS, SC8561_IBUS_ADC_NUM,
		 SC8561_IBUS_ADC_DEN);
SC8561_DBGFS_ADC(vbatt, SC8561_ADC_VBAT, SC8561_VBAT_ADC_NUM,
		 SC8561_VBAT_ADC_DEN);
SC8561_DBGFS_ADC(ibatt, SC8561_ADC_IBAT, SC8561_IBAT_ADC_NUM,
		 SC8561_IBAT_ADC_DEN);

/* ------------------------------------------------------------------ */
/* charger_class ops                                                  */
/* ------------------------------------------------------------------ */

static int sc8561_ops_enable(struct charger_device *chg_dev, bool en)
{
	return sc8561_enable_charge(charger_get_data(chg_dev), en);
}

static int sc8561_ops_is_enabled(struct charger_device *chg_dev, bool *en)
{
	return sc8561_is_charge_enabled(charger_get_data(chg_dev), en);
}

static int sc8561_ops_get_vbus(struct charger_device *chg_dev, u32 *vbus)
{
	struct sc8561_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8561_read_adc(chip, SC8561_ADC_VBUS, SC8561_VBUS_ADC_NUM,
			      SC8561_VBUS_ADC_DEN, &val);
	if (!ret)
		*vbus = val * 1000;
	return ret;
}

static int sc8561_ops_get_ibus(struct charger_device *chg_dev, u32 *ibus)
{
	struct sc8561_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8561_read_adc(chip, SC8561_ADC_IBUS, SC8561_IBUS_ADC_NUM,
			      SC8561_IBUS_ADC_DEN, &val);
	if (!ret)
		*ibus = val * 1000;
	return ret;
}

static int sc8561_ops_get_vbatt(struct charger_device *chg_dev, u32 *vbatt)
{
	struct sc8561_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8561_read_adc(chip, SC8561_ADC_VBAT, SC8561_VBAT_ADC_NUM,
			      SC8561_VBAT_ADC_DEN, &val);
	if (!ret)
		*vbatt = val * 1000;
	return ret;
}

static int sc8561_ops_get_ibatt(struct charger_device *chg_dev, u32 *ibatt)
{
	struct sc8561_chip *chip = charger_get_data(chg_dev);
	int val, ret;

	ret = sc8561_read_adc(chip, SC8561_ADC_IBAT, SC8561_IBAT_ADC_NUM,
			      SC8561_IBAT_ADC_DEN, &val);
	if (!ret)
		*ibatt = val * 1000;
	return ret;
}

static int sc8561_set_mode_and_prot(struct sc8561_chip *chip, int mode)
{
	int ret;

	ret = sc8561_set_operation_mode(chip, mode);
	if (ret)
		return ret;

	return sc8561_init_protection(chip, mode);
}

static int sc8561_ops_set_mode(struct charger_device *chg_dev, int mode)
{
	return sc8561_set_mode_and_prot(charger_get_data(chg_dev), mode);
}

static int sc8561_ops_device_init(struct charger_device *chg_dev, int value)
{
	return sc8561_set_mode_and_prot(charger_get_data(chg_dev), value);
}

static int sc8561_ops_is_bypass_enabled(struct charger_device *chg_dev,
					bool *en)
{
	*en = sc8561_is_bypass_enabled_hw(charger_get_data(chg_dev));
	return 0;
}

static int sc8561_ops_bypass_support(struct charger_device *chg_dev, bool *en)
{
	/* the SC8561 1:1 path is not wired as a bypass here */
	*en = false;
	return 0;
}

static int sc8561_ops_enable_adc(struct charger_device *chg_dev, bool en)
{
	return sc8561_enable_adc(charger_get_data(chg_dev), en);
}

static const struct charger_ops sc8561_chg_ops = {
	.enable = sc8561_ops_enable,
	.is_enabled = sc8561_ops_is_enabled,
	.get_vbus_adc = sc8561_ops_get_vbus,
	.get_ibus_adc = sc8561_ops_get_ibus,
	.cp_get_vbatt = sc8561_ops_get_vbatt,
	.cp_get_ibatt = sc8561_ops_get_ibatt,
	.cp_set_mode = sc8561_ops_set_mode,
	.is_bypass_enabled = sc8561_ops_is_bypass_enabled,
	.cp_device_init = sc8561_ops_device_init,
	.cp_get_bypass_support = sc8561_ops_bypass_support,
	.cp_enable_adc = sc8561_ops_enable_adc,
};

/* ------------------------------------------------------------------ */
/* power_supply diagnostics                                           */
/* ------------------------------------------------------------------ */

static enum power_supply_property sc8561_psy_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int sc8561_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sc8561_chip *chip = power_supply_get_drvdata(psy);
	bool en;
	int v, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = sc8561_is_charge_enabled(chip, &en);
		if (ret)
			return ret;
		val->intval = en;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = sc8561_is_charge_enabled(chip, &en);
		if (ret)
			return ret;
		val->intval = en ? POWER_SUPPLY_STATUS_CHARGING :
				   POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8561_read_adc(chip, SC8561_ADC_VBUS,
				      SC8561_VBUS_ADC_NUM,
				      SC8561_VBUS_ADC_DEN, &v);
		if (ret)
			return ret;
		val->intval = v * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8561_read_adc(chip, SC8561_ADC_IBUS,
				      SC8561_IBUS_ADC_NUM,
				      SC8561_IBUS_ADC_DEN, &v);
		if (ret)
			return ret;
		val->intval = v * 1000;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "SouthChip";
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->role == SC8561_ROLE_MASTER ?
			      "SC8561" : "SC8561";
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                     */
/* ------------------------------------------------------------------ */

static int sc8561_probe(struct i2c_client *client)
{
	struct sc8561_chip *chip;
	struct power_supply_config psy_cfg = {};
	struct charger_properties chg_props = {};
	unsigned int id;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &client->dev;
	chip->role = (enum sc8561_role)(uintptr_t)i2c_get_match_data(client);
	chip->name = chip->role == SC8561_ROLE_MASTER ? "cp_master" : "cp_slave";
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &sc8561_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(chip->dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	ret = regmap_read(chip->regmap, SC8561_PART_INFO_REG, &id);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "failed to read part info\n");
	if (id != SC8561_DEVICE_ID)
		return dev_err_probe(chip->dev, -ENODEV,
				     "unexpected part id %#x\n", id);

	ret = sc8561_init_device(chip);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "failed to init device\n");

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(chip->dev, client->irq, NULL,
						sc8561_irq_thread,
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
						&sc8561_chg_ops, &chg_props);
	if (IS_ERR(chip->chg_dev))
		return dev_err_probe(chip->dev, PTR_ERR(chip->chg_dev),
				     "failed to register charger\n");

	chip->psy_desc.name = chip->role == SC8561_ROLE_MASTER ?
			      "sc8561-master" : "sc8561-slave";
	chip->psy_desc.type = POWER_SUPPLY_TYPE_USB;
	chip->psy_desc.properties = sc8561_psy_props;
	chip->psy_desc.num_properties = ARRAY_SIZE(sc8561_psy_props);
	chip->psy_desc.get_property = sc8561_get_property;
	psy_cfg.drv_data = chip;
	chip->psy = devm_power_supply_register(chip->dev, &chip->psy_desc,
					       &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->psy),
				     "failed to register power supply\n");

	chip->dbgfs = debugfs_create_dir(chip->name, NULL);
	debugfs_create_file("enable", 0600, chip->dbgfs, chip,
			    &sc8561_enable_fops);
	debugfs_create_file("bypass", 0600, chip->dbgfs, chip,
			    &sc8561_bypass_fops);
	debugfs_create_file("vbus", 0400, chip->dbgfs, chip,
			    &sc8561_vbus_fops);
	debugfs_create_file("ibus", 0400, chip->dbgfs, chip,
			    &sc8561_ibus_fops);
	debugfs_create_file("vbatt", 0400, chip->dbgfs, chip,
			    &sc8561_vbatt_fops);
	debugfs_create_file("ibatt", 0400, chip->dbgfs, chip,
			    &sc8561_ibatt_fops);
	debugfs_create_file("faults", 0400, chip->dbgfs, chip,
			    &sc8561_faults_fops);

	dev_info(chip->dev, "SC8561 %s probed (part id %#x, irq %d)\n",
		 chip->role == SC8561_ROLE_MASTER ? "master" : "slave", id,
		 client->irq);

	return 0;
}

static void sc8561_remove(struct i2c_client *client)
{
	struct sc8561_chip *chip = i2c_get_clientdata(client);

	sc8561_enable_charge(chip, false);
	debugfs_remove_recursive(chip->dbgfs);
	charger_device_unregister(chip->chg_dev);
}

static const struct i2c_device_id sc8561_i2c_ids[] = {
	{ "sc8561-master", SC8561_ROLE_MASTER },
	{ "sc8561-slave", SC8561_ROLE_SLAVE },
	{}
};
MODULE_DEVICE_TABLE(i2c, sc8561_i2c_ids);

static const struct of_device_id sc8561_of_match[] = {
	{ .compatible = "southchip,sc8561-master",
	  .data = (void *)SC8561_ROLE_MASTER },
	{ .compatible = "southchip,sc8561-slave",
	  .data = (void *)SC8561_ROLE_SLAVE },
	{}
};
MODULE_DEVICE_TABLE(of, sc8561_of_match);

static struct i2c_driver sc8561_driver = {
	.driver = {
		.name = "sc8561",
		.of_match_table = sc8561_of_match,
	},
	.probe = sc8561_probe,
	.remove = sc8561_remove,
	.id_table = sc8561_i2c_ids,
};
module_i2c_driver(sc8561_driver);

MODULE_AUTHOR("xaga mainline port");
MODULE_DESCRIPTION("SouthChip SC8561 2:1/4:1 charge-pump driver");
MODULE_LICENSE("GPL");
