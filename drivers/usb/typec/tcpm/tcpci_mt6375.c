// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on drivers/usb/typec/tcpm/tcpci_mt6370.c
 *   Copyright (C) 2022 Richtek Technology Corp.
 *   Author: ChiYuan Huang <cy_huang@richtek.com>
 *
 * and drivers/misc/mediatek/typec/tcpc/tcpc_mt6375.c
 *   Copyright (c) 2020 MediaTek Inc.
 *   Author: Gene Chen <gene_chen@richtek.com>
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/tcpci.h>
#include <linux/usb/tcpm.h>

/* MT6375 TCPC vendor registers (all in TCPC bank, 0x00xx) */
#define MT6375_REG_PHYCTRL1 0x80
#define MT6375_REG_PHYCTRL2 0x81
#define MT6375_REG_PHYCTRL3 0x82
#define MT6375_REG_PHYCTRL7 0x86
#define MT6375_REG_VCONCTRL3 0x8C
#define MT6375_REG_SYSCTRL1 0x8F
#define MT6375_REG_PHYCTRL9 0xAC
#define MT6375_REG_SYSCTRL3 0xB0
#define MT6375_REG_TCPCCTRL1 0xB1
#define MT6375_REG_TCPCCTRL2 0xB2
#define MT6375_REG_TCPCCTRL3 0xB3
#define MT6375_REG_LPWRCTRL3 0xBB
#define MT6375_REG_HILOCTRL9 0xC8
#define MT6375_REG_SHIELDCTRL1 0xCA
#define MT6375_REG_TYPECOTPCTRL 0xCD

#define MT6375_MSK_SHIPPING_OFF BIT(5)
#define MT6375_MSK_AUTOIDLE_EN BIT(3)
#define MT6375_MSK_OPEN40MS_EN BIT(4)
#define MT6375_MSK_VREFTS_EN BIT(7)

#define MT6375_NORMAL_RP_DUTY 330

struct mt6375_priv {
	struct device *dev;
	struct regulator *vbus;
	struct power_supply *charger;
	bool vbus_on;
	bool power_fault;
	struct tcpci *tcpci;
	struct tcpci_data tcpci_data;
};

static int mt6375_write16(struct regmap *regmap, unsigned int reg, u16 val)
{
    val = cpu_to_le16(val);

    return regmap_raw_write(regmap, reg, &val, sizeof(val));
}

static int mt6375_tcpc_init(struct tcpci *tcpci, struct tcpci_data *data)
{
    struct regmap *regmap = data->regmap;
    int ret;

    /* Software reset */
    ret = regmap_write(regmap, MT6375_REG_SYSCTRL3, 0x01);
    if (ret)
        return ret;
    usleep_range(1000, 2000);

    /* PD_IRQB path + shipping off + autoidle on */
    ret = regmap_write(regmap, MT6375_REG_SYSCTRL1,
                       0x80 | MT6375_MSK_SHIPPING_OFF |
                       MT6375_MSK_AUTOIDLE_EN);
    if (ret)
        return ret;

    /* TCPC filter / DRP timing */
    ret = regmap_write(regmap, MT6375_REG_TCPCCTRL1, 0x0A);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_TCPCCTRL2, 0x04);
    if (ret)
        return ret;
    ret = mt6375_write16(regmap, MT6375_REG_TCPCCTRL3,
                         MT6375_NORMAL_RP_DUTY);
    if (ret)
        return ret;

    /* PHY setup from downstream init */
    ret = regmap_write(regmap, MT6375_REG_PHYCTRL1, 0x74);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_PHYCTRL2, 0x3A);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_PHYCTRL3, 0x82);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_PHYCTRL7, 0x36);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_PHYCTRL9, 0x3C);
    if (ret)
        return ret;

    /* VCONN current limit / CC filter */
    ret = regmap_write(regmap, MT6375_REG_VCONCTRL3, 0x41);
    if (ret)
        return ret;
    ret = regmap_write(regmap, MT6375_REG_HILOCTRL9, 0x0A);
    if (ret)
        return ret;

    /* Enable CC open 40ms and VREFTS */
    ret = regmap_update_bits(regmap, MT6375_REG_SHIELDCTRL1,
                             MT6375_MSK_OPEN40MS_EN,
                             MT6375_MSK_OPEN40MS_EN);
    if (ret)
        return ret;
    ret = regmap_update_bits(regmap, MT6375_REG_TYPECOTPCTRL,
                             MT6375_MSK_VREFTS_EN,
                             MT6375_MSK_VREFTS_EN);
    if (ret)
        return ret;

    /* Enable CC status change alert */
    ret = regmap_update_bits(regmap, TCPC_TCPC_CTRL,
                             TCPC_TCPC_CTRL_EN_LK4CONN_ALRT,
                             TCPC_TCPC_CTRL_EN_LK4CONN_ALRT);
    if (ret)
        return ret;

    /* Clear bleed discharge */
    ret = regmap_update_bits(regmap, TCPC_POWER_CTRL,
                             TCPC_POWER_CTRL_BLEED_DISCHARGE, 0);
    if (ret)
        return ret;

    /* Enable TCPC interrupts for CC / VBUS changes. */
    {
        u16 alert_mask = TCPC_ALERT_TX_SUCCESS | TCPC_ALERT_TX_FAILED |
                TCPC_ALERT_TX_DISCARDED | TCPC_ALERT_RX_STATUS |
                TCPC_ALERT_RX_HARD_RST | TCPC_ALERT_CC_STATUS |
                TCPC_ALERT_POWER_STATUS;

        return regmap_raw_write(regmap, TCPC_ALERT_MASK, &alert_mask,
                                sizeof(alert_mask));
    }
}

static int mt6375_start_drp_toggling(struct tcpci *tcpci,
                                     struct tcpci_data *data,
                                     enum typec_cc_status cc)
{
    /* Vendor-specific low-power setting before LOOK4CONNECTION */
    return regmap_write(data->regmap, MT6375_REG_LPWRCTRL3, 0xD8);
}

static int mt6375_get_current_limit(struct tcpci *tcpci, struct tcpci_data *data)
{
	/* Default Rp gives no PD/Type-C higher-current entitlement. */
	return 100;
}

static int mt6375_set_current_limit(struct tcpci *tcpci, struct tcpci_data *data,
				    u32 max_ma, u32 mv)
{
	struct mt6375_priv *priv = container_of(data, struct mt6375_priv, tcpci_data);
	union power_supply_propval value;

	/* Fixed 5 V only. Default USB current stays at 100 mA before enumeration. */
	if (mv && mv != 5000)
		return -EINVAL;
	if (!mv || max_ma < 100)
		value.intval = 0;
	else if (max_ma <= 500)
		value.intval = 100000;
	else
		value.intval = min(max_ma, 1500U) * 1000;
	return power_supply_set_property(priv->charger,
					POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT, &value);
}

/* TCPM serializes callbacks. Only this consumer owns the boost enable vote. */
static int mt6375_set_managed_vbus(struct mt6375_priv *priv, bool source, bool sink)
{
	struct regmap *map = priv->tcpci_data.regmap;
	union power_supply_propval value = { .intval = 0 };
	unsigned int status;
	int ret, cleanup;

	if (source && sink)
		return -EINVAL;
	if (priv->power_fault && (source || sink))
		return -EIO;

	if (!source && priv->vbus_on) {
		ret = regulator_disable(priv->vbus);
		if (ret)
			goto fault;
		priv->vbus_on = false;
	}
	if (!source) {
		ret = regmap_write(map, TCPC_COMMAND, TCPC_CMD_DISABLE_SRC_VBUS);
		if (ret)
			goto fault;
	}
	if (!sink) {
		ret = regmap_write(map, TCPC_COMMAND, TCPC_CMD_DISABLE_SINK_VBUS);
		if (ret)
			goto fault;
		ret = power_supply_set_property(priv->charger, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret)
			goto fault;
	}
	if (source && !priv->vbus_on) {
		/* Independent of the PMIC ADC check; propagate failed status reads. */
		ret = regmap_read(map, TCPC_POWER_STATUS, &status);
		if (ret)
			return ret;
		if (status & TCPC_POWER_STATUS_VBUS_PRES)
			return -EBUSY;
		ret = regulator_enable(priv->vbus);
		if (ret)
			return ret;
		priv->vbus_on = true;
		ret = regmap_write(map, TCPC_COMMAND, TCPC_CMD_SRC_VBUS_DEFAULT);
		if (ret) {
			cleanup = regulator_disable(priv->vbus);
			if (!cleanup)
				priv->vbus_on = false;
			goto fault;
		}
	}
	if (sink) {
		ret = regulator_is_enabled(priv->vbus);
		if (ret) {
			if (ret > 0)
				ret = -EBUSY;
			goto fault;
		}
		ret = regmap_write(map, TCPC_COMMAND, TCPC_CMD_SINK_VBUS);
		if (ret)
			goto fault;
		value.intval = 1;
		ret = power_supply_set_property(priv->charger, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret) {
			regmap_write(map, TCPC_COMMAND, TCPC_CMD_DISABLE_SINK_VBUS);
			value.intval = 0;
			power_supply_set_property(priv->charger, POWER_SUPPLY_PROP_ONLINE, &value);
			goto fault;
		}
	}
	/* Entire command sequence handled here; bypass the generic TCPCI writes. */
	return 1;
fault:
	priv->power_fault = true;
	dev_err(priv->dev, "VBUS transition failed: %d; further power-on blocked\n", ret);
	return ret;
}

static int mt6375_set_vbus(struct tcpci *tcpci, struct tcpci_data *data,
			   bool source, bool sink)
{
	struct mt6375_priv *priv = container_of(data, struct mt6375_priv,
					    tcpci_data);
	int ret;

	if (priv->charger)
		return mt6375_set_managed_vbus(priv, source, sink);

	if (!priv->vbus)
		return 0;

	ret = regulator_is_enabled(priv->vbus);
	if (ret < 0)
		return ret;

	if (ret && !source)
		return regulator_disable(priv->vbus);

	if (!ret && source)
		return regulator_enable(priv->vbus);

	return 0;
}

static irqreturn_t mt6375_irq_handler(int irq, void *dev_id)
{
    struct mt6375_priv *priv = dev_id;

    return tcpci_irq(priv->tcpci);
}

static void mt6375_unregister_tcpci_port(void *tcpci)
{
    tcpci_unregister_port(tcpci);
}

static int mt6375_tcpc_probe(struct platform_device *pdev)
{
	struct mt6375_priv *priv;
	struct device *dev = &pdev->dev;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	priv->tcpci_data.regmap = dev_get_regmap(dev->parent, NULL);
	if (!priv->tcpci_data.regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get parent regmap\n");

	irq = platform_get_irq_byname(pdev, "PD_IRQB");
	if (irq < 0)
		return irq;

	priv->tcpci_data.auto_discharge_disconnect = 1;
	priv->tcpci_data.init = mt6375_tcpc_init;
	priv->tcpci_data.start_drp_toggling = mt6375_start_drp_toggling;

	priv->vbus = devm_regulator_get_optional(dev, "vbus");
	if (IS_ERR(priv->vbus)) {
		ret = PTR_ERR(priv->vbus);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "Failed to get VBUS regulator\n");
		priv->vbus = NULL;
	}
	if (device_property_present(dev, "power-supplies")) {
		if (!priv->vbus)
			return dev_err_probe(dev, -EINVAL, "Managed port needs VBUS regulator\n");
		priv->charger = devm_power_supply_get_by_reference(dev, "power-supplies");
		if (IS_ERR(priv->charger))
			return dev_err_probe(dev, PTR_ERR(priv->charger), "Charger unavailable\n");
		if (!priv->charger)
			return -EPROBE_DEFER;
		priv->tcpci_data.set_current_limit = mt6375_set_current_limit;
		priv->tcpci_data.get_current_limit = mt6375_get_current_limit;
	}
	if (priv->vbus)
		priv->tcpci_data.set_vbus = mt6375_set_vbus;

	priv->tcpci = tcpci_register_port(dev, &priv->tcpci_data);
	if (IS_ERR(priv->tcpci))
		return dev_err_probe(dev, PTR_ERR(priv->tcpci),
				     "Failed to register tcpci port\n");

	ret = devm_add_action_or_reset(dev, mt6375_unregister_tcpci_port,
				       priv->tcpci);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, mt6375_irq_handler,
					IRQF_ONESHOT, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request irq\n");

	device_init_wakeup(dev, true);
	dev_pm_set_wake_irq(dev, irq);

	return 0;
}

static void mt6375_tcpc_remove(struct platform_device *pdev)
{
    dev_pm_clear_wake_irq(&pdev->dev);
    device_init_wakeup(&pdev->dev, false);
}

static const struct of_device_id mt6375_tcpc_devid_table[] = {
    { .compatible = "mediatek,mt6375-tcpc" },
    {}
};
MODULE_DEVICE_TABLE(of, mt6375_tcpc_devid_table);

static struct platform_driver mt6375_tcpc_driver = {
    .driver = {
        .name = "mt6375-tcpc",
        .of_match_table = mt6375_tcpc_devid_table,
    },
    .probe = mt6375_tcpc_probe,
    .remove = mt6375_tcpc_remove,
};
module_platform_driver(mt6375_tcpc_driver);

MODULE_AUTHOR("ChiYuan Huang <cy_huang@richtek.com>");
MODULE_AUTHOR("Gene Chen <gene_chen@richtek.com>");
MODULE_DESCRIPTION("MT6375 USB Type-C Port Controller Interface Driver");
MODULE_LICENSE("GPL v2");
