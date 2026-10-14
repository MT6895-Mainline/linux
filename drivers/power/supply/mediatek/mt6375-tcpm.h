/* SPDX-License-Identifier: GPL-2.0-only */
/* Included by mt6375-charger.c after the field and USB switch helpers. */

/*
 * Input current limit for the tcpm-managed direct-charge path (uA).
 *
 * pearl_cp_manager writes 3 A when handing the load back to the MT6375
 * (PEARL_MT6375_AICR in pearl_cp_manager.c). The cap here used to be
 * 1500000: that write was rejected with -EINVAL, the caller did not check
 * the return value, and the limit silently stayed at its 100000 initial
 * value -- net discharge while plugged in.
 *
 * MT6375_TCPM_SINK_LIMIT_MAX_UA takes the range limit of the CHG_AICR field
 * itself (see mt6375_chg_fields[F_IAICR] = MT6375_CHG_RANGE(100, 3225, 25,
 * 2, false), i.e. 100..3225 mA in 25 mA steps); 3 A is within range and is
 * also the value the SoC dtsi has always used as aicr = <3000>.
 */
#define MT6375_TCPM_SINK_ENABLE_UA	100000	/* minimum current limit needed to enable the sink */
#define MT6375_TCPM_SINK_LIMIT_UA	1500000	/* default limit for direct charge (fallback) */
#define MT6375_TCPM_SINK_LIMIT_MAX_UA	3000000	/* max limit that can be requested before the CP manager takes over */

/* power_lock must be held across each sink/source transition. */
static int mt6375_tcpm_quiesce(struct mt6375_chg_data *ddata)
{
	static const enum mt6375_chg_reg_field off[] = {
		F_CHG_EN, F_BUCK_EN, F_BC12_EN, F_DP_LDO_EN, F_DM_LDO_EN,
		F_DP_PULL_REN, F_DM_PULL_REN, F_DP_PULL_IEN, F_DM_PULL_IEN,
		F_DP_DET_EN, F_DM_DET_EN, F_DPDM_SW_VCP_EN, F_MANUAL_MODE,
		F_BLEED_DIS_EN,
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(off); i++) {
		ret = mt6375_chg_field_set(ddata, off[i], 0);
		if (ret)
			return ret;
	}
	ret = mt6375_chg_set_usbsw(ddata, USBSW_USB);
	if (!ret)
		atomic_set(&ddata->attach, 0);
	return ret;
}

static int mt6375_tcpm_source_enable(struct regulator_dev *rdev)
{
	struct mt6375_chg_data *ddata = rdev->reg_data;
	int ret, cleanup, vbus;

	mutex_lock(&ddata->power_lock);
	if (ddata->power_fault) {
		ret = -EIO;
		goto out;
	}
	if (ddata->sink_requested) {
		ret = -EBUSY;
		goto out;
	}
	if (ddata->source_enabled) {
		ret = 0;
		goto out;
	}
	ret = mt6375_tcpm_quiesce(ddata);
	if (ret)
		goto fault;

	/* Fresh ADC conversion: do not boost into an externally powered bus. */
	ret = mt6375_chg_iio_read(ddata, ADC_CHAN_CHGVINDIV5, &vbus);
	if (ret)
		goto out;
	if (vbus < 0 || vbus >= 800000) {
		ret = -EBUSY;
		goto out;
	}
	ret = regulator_is_enabled_regmap(rdev);
	if (ret) {
		if (ret > 0)
			ret = -EBUSY;
		goto fault;
	}
	ret = mt6375_set_boost_param(ddata, true);
	if (ret)
		goto fault;
	ret = regulator_enable_regmap(rdev);
	if (!ret) {
		ret = regulator_is_enabled_regmap(rdev);
		if (ret == 1) {
			ddata->source_enabled = true;
			ret = 0;
			goto out;
		}
		if (!ret)
			ret = -EIO;
	}
	/* An I2C error may follow a completed write: explicitly try to turn off. */
	cleanup = regulator_disable_regmap(rdev);
	if (!cleanup)
		cleanup = regulator_is_enabled_regmap(rdev);
	ddata->source_enabled = cleanup != 0;
	if (!cleanup)
		mt6375_set_boost_param(ddata, false);
fault:
	/* No automatic re-enable after a partially completed power transition. */
	ddata->power_fault = true;
out:
	mutex_unlock(&ddata->power_lock);
	return ret;
}

static int mt6375_tcpm_source_disable(struct regulator_dev *rdev)
{
	struct mt6375_chg_data *ddata = rdev->reg_data;
	int ret;

	mutex_lock(&ddata->power_lock);
	/* Stop boost before restoring buck parameters; never re-enable on error. */
	ret = regulator_disable_regmap(rdev);
	if (ret)
		goto fault;
	ret = regulator_is_enabled_regmap(rdev);
	if (ret) {
		if (ret > 0)
			ret = -EIO;
		goto fault;
	}
	ddata->source_enabled = false;
	ret = mt6375_set_boost_param(ddata, false);
	if (ret)
		goto fault;
	goto out;
fault:
	ddata->power_fault = true;
out:
	mutex_unlock(&ddata->power_lock);
	return ret;
}

static int mt6375_tcpm_stop_sink(struct mt6375_chg_data *ddata)
{
	int ret, buck_ret;

	/* Attempt both switches even if the first write fails. */
	ret = mt6375_chg_field_set(ddata, F_CHG_EN, 0);
	buck_ret = mt6375_chg_field_set(ddata, F_BUCK_EN, 0);
	if (!ret)
		ret = buck_ret;
	if (ret)
		ddata->power_fault = true;
	else
		atomic_set(&ddata->attach, 0);
	return ret;
}

static int mt6375_tcpm_apply_sink(struct mt6375_chg_data *ddata)
{
	int ret;
	bool enable = ddata->sink_requested &&
		      ddata->sink_limit_ua >= MT6375_TCPM_SINK_ENABLE_UA;

	if (!enable)
		return mt6375_tcpm_stop_sink(ddata);
	if (ddata->power_fault || ddata->source_enabled)
		return -EBUSY;
	ret = regulator_is_enabled_regmap(ddata->rdev);
	if (ret) {
		if (ret > 0)
			ret = -EBUSY;
		goto fault;
	}

	/* This managed interface uses power_supply's microamp units. */
	ret = mt6375_chg_field_set(ddata, F_IAICR, ddata->sink_limit_ua / 1000);
	if (ret)
		goto fault;
	ret = mt6375_chg_field_set(ddata, F_BUCK_EN, 1);
	if (!ret)
		ret = mt6375_chg_field_set(ddata, F_CHG_EN, 1);
	if (ret)
		goto fault;
	atomic_set(&ddata->attach, 1);
	return 0;
fault:
	/* A failed lower limit must not leave the previous high-current sink on. */
	mt6375_tcpm_stop_sink(ddata);
	ddata->power_fault = true;
	return ret;
}

static int mt6375_tcpm_set_property(struct mt6375_chg_data *ddata,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	int ret;

	mutex_lock(&ddata->power_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (val->intval != 0 && val->intval != 1) {
			ret = -EINVAL;
			break;
		}
		ddata->sink_requested = val->intval;
		if (!ddata->sink_requested)
			ddata->sink_limit_ua = 0;
		ret = mt6375_tcpm_apply_sink(ddata);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0 ||
		    val->intval > MT6375_TCPM_SINK_LIMIT_MAX_UA) {
			ret = -EINVAL;
			break;
		}
		ddata->sink_limit_ua = val->intval;
		ret = mt6375_tcpm_apply_sink(ddata);
		break;
	default:
		/* Legacy direct charging, DP/DM, and fast-charge knobs are excluded. */
		ret = -EOPNOTSUPP;
		break;
	}
	mutex_unlock(&ddata->power_lock);
	if (!ret)
		power_supply_changed(ddata->psy);
	return ret;
}
