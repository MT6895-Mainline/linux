// SPDX-License-Identifier: GPL-2.0
/*
 * mtu3_dr.c - dual role switch and host glue layer
 *
 * Copyright (C) 2016 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <linux/string_choices.h>
#include "mtu3.h"
#include "mtu3_dr.h"
#include "mtu3_debug.h"

#define USB2_PORT 2
#define USB3_PORT 3

static inline struct ssusb_mtk *otg_sx_to_ssusb(struct otg_switch_mtk *otg_sx)
{
	return container_of(otg_sx, struct ssusb_mtk, otg_switch);
}

static void toggle_opstate(struct ssusb_mtk *ssusb)
{
	mtu3_setbits(ssusb->mac_base, U3D_DEVICE_CONTROL, DC_SESSION);
	mtu3_setbits(ssusb->mac_base, U3D_POWER_MANAGEMENT, SOFT_CONN);
}

/* only port0 supports dual-role mode */
static int ssusb_port0_switch(struct ssusb_mtk *ssusb,
	int version, bool tohost)
{
	void __iomem *ibase = ssusb->ippc_base;
	u32 value;

	dev_dbg(ssusb->dev, "%s (switch u%d port0 to %s)\n", __func__,
		version, tohost ? "host" : "device");

	if (version == USB2_PORT) {
		/* 1. power off and disable u2 port0 */
		value = mtu3_readl(ibase, SSUSB_U2_CTRL(0));
		value |= SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS;
		mtu3_writel(ibase, SSUSB_U2_CTRL(0), value);

		/* 2. power on, enable u2 port0 and select its mode */
		value = mtu3_readl(ibase, SSUSB_U2_CTRL(0));
		value &= ~(SSUSB_U2_PORT_PDN | SSUSB_U2_PORT_DIS);
		value = tohost ? (value | SSUSB_U2_PORT_HOST_SEL) :
			(value & (~SSUSB_U2_PORT_HOST_SEL));
		mtu3_writel(ibase, SSUSB_U2_CTRL(0), value);
	} else {
		/* 1. power off and disable u3 port0 */
		value = mtu3_readl(ibase, SSUSB_U3_CTRL(0));
		value |= SSUSB_U3_PORT_PDN | SSUSB_U3_PORT_DIS;
		mtu3_writel(ibase, SSUSB_U3_CTRL(0), value);

		/* 2. power on, enable u3 port0 and select its mode */
		value = mtu3_readl(ibase, SSUSB_U3_CTRL(0));
		value &= ~(SSUSB_U3_PORT_PDN | SSUSB_U3_PORT_DIS);
		value = tohost ? (value | SSUSB_U3_PORT_HOST_SEL) :
			(value & (~SSUSB_U3_PORT_HOST_SEL));
		mtu3_writel(ibase, SSUSB_U3_CTRL(0), value);
	}

	return 0;
}

static void switch_port_to_host(struct ssusb_mtk *ssusb)
{
	u32 check_clk = 0;

	dev_dbg(ssusb->dev, "%s\n", __func__);

	ssusb_port0_switch(ssusb, USB2_PORT, true);

	if (ssusb->otg_switch.is_u3_drd) {
		ssusb_port0_switch(ssusb, USB3_PORT, true);
		check_clk = SSUSB_U3_MAC_RST_B_STS;
	}

	ssusb_check_clocks(ssusb, check_clk);

	/* after all clocks are stable */
	toggle_opstate(ssusb);
}

static int switch_port_to_device(struct ssusb_mtk *ssusb)
{
	u32 check_clk = 0;

	dev_dbg(ssusb->dev, "%s\n", __func__);

	ssusb_port0_switch(ssusb, USB2_PORT, false);

	if (ssusb->otg_switch.is_u3_drd) {
		ssusb_port0_switch(ssusb, USB3_PORT, false);
		check_clk = SSUSB_U3_MAC_RST_B_STS;
	}

	return ssusb_check_clocks(ssusb, check_clk);
}

int ssusb_set_vbus(struct otg_switch_mtk *otg_sx, int is_on)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct regulator *vbus = otg_sx->vbus;
	int ret;

	/* vbus is optional */
	if (!vbus)
		return 0;

	dev_dbg(ssusb->dev, "%s: turn %s\n", __func__, str_on_off(is_on));

	if (is_on) {
		ret = regulator_enable(vbus);
		if (ret) {
			dev_err(ssusb->dev, "vbus regulator enable failed\n");
			return ret;
		}
	} else {
		regulator_disable(vbus);
	}

	return 0;
}

/*
 * Xaga routes VBUS detection through its external Type-C controller. Match
 * the vendor MT6895 device-mode setup: this overrides the MAC's VBUS input,
 * not the physical VBUS supply. TCPM's DEVICE/NONE events assert/clear it.
 */
static void ssusb_set_device_vbus(struct ssusb_mtk *ssusb, bool valid)
{
	u32 u2_ctrl, misc;

	if (!ssusb->otg_switch.sw_vbus_detect)
		return;

	u2_ctrl = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
	misc = mtu3_readl(ssusb->mac_base, U3D_MISC_CTRL);
	if (valid) {
		u2_ctrl &= ~SSUSB_U2_PORT_OTG_SEL;
		misc |= VBUS_FRC_EN | VBUS_ON;
	} else {
		u2_ctrl |= SSUSB_U2_PORT_OTG_SEL;
		misc &= ~(VBUS_FRC_EN | VBUS_ON);
	}
	mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), u2_ctrl);
	mtu3_writel(ssusb->mac_base, U3D_MISC_CTRL, misc);
}

static void ssusb_mode_sw_work_locked(struct work_struct *work)
{
	struct otg_switch_mtk *otg_sx =
		container_of(work, struct otg_switch_mtk, dr_work);
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct mtu3 *mtu = ssusb->u3d;
	enum usb_role desired_role = READ_ONCE(otg_sx->desired_role);
	bool device_attached = desired_role == USB_ROLE_DEVICE;
	enum usb_role current_role;
	enum phy_mode phy_mode;
	int i, ret;

	current_role = ssusb->is_host ? USB_ROLE_HOST : USB_ROLE_DEVICE;

	if (desired_role == USB_ROLE_NONE) {
		/* the default mode is host as probe does */
		desired_role = USB_ROLE_HOST;
		if (otg_sx->default_role == USB_ROLE_DEVICE)
			desired_role = USB_ROLE_DEVICE;
	}

	if (current_role == desired_role && !otg_sx->sw_vbus_detect)
		return;

	dev_dbg(ssusb->dev, "set role : %s\n", usb_role_string(desired_role));
	mtu3_dbg_trace(ssusb->dev, "set role : %s", usb_role_string(desired_role));
	ret = pm_runtime_resume_and_get(ssusb->dev);
	if (ret < 0)
		return;

	/* NONE can map to DEVICE, but must not leave VBUS-valid asserted. */
	if (!device_attached)
		ssusb_set_device_vbus(ssusb, false);
	if (current_role == desired_role)
		goto update_vbus;

	phy_mode = desired_role == USB_ROLE_HOST ? PHY_MODE_USB_HOST : PHY_MODE_USB_DEVICE;
	for (i = 0; i < ssusb->num_phys; i++) {
		ret = phy_set_mode(ssusb->phys[i], phy_mode);
		if (ret) {
			dev_err(ssusb->dev, "failed to set PHY role: %d\n", ret);
			goto out;
		}
	}

	switch (desired_role) {
	case USB_ROLE_HOST:
		ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_HOST);
		mtu3_stop(mtu);
		switch_port_to_host(ssusb);
		ssusb_set_vbus(otg_sx, 1);
		ssusb->is_host = true;
		break;
	case USB_ROLE_DEVICE:
		ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_DEVICE);
		ssusb->is_host = false;
		ssusb_set_vbus(otg_sx, 0);
		ret = switch_port_to_device(ssusb);
		if (ret)
			goto out;
		mtu3_start(mtu);
		break;
	case USB_ROLE_NONE:
	default:
		dev_err(ssusb->dev, "invalid role\n");
	}
update_vbus:
	ssusb_set_device_vbus(ssusb, device_attached &&
			      !ssusb->is_host && mtu->is_active);
out:
	pm_runtime_put(ssusb->dev);
}

static void ssusb_mode_sw_work(struct work_struct *work)
{
	struct otg_switch_mtk *otg_sx =
		container_of(work, struct otg_switch_mtk, dr_work);

	mutex_lock(&otg_sx->role_lock);
	ssusb_mode_sw_work_locked(work);
	mutex_unlock(&otg_sx->role_lock);
}

/* Caller holds role_lock and the gadget device lock (excludes bind/unbind). */
static int ssusb_recover_unbound_device(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;
	struct mtu3 *mtu = ssusb->u3d;
	unsigned long flags;
	int i, ret;

	if (ssusb->is_host || READ_ONCE(otg_sx->desired_role) != USB_ROLE_DEVICE)
		return -EBUSY;
	if (mtu->gadget_driver || mtu->softconnect || mtu->connected)
		return -EBUSY;

	/* Never change the power role or call the VBUS regulator here. */
	for (i = 0; i < ssusb->num_phys; i++) {
		ret = phy_set_mode(ssusb->phys[i], PHY_MODE_USB_DEVICE);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&mtu->lock, flags);
	mtu3_stop(mtu);
	spin_unlock_irqrestore(&mtu->lock, flags);
	synchronize_irq(mtu->irq);

	ssusb_set_force_mode(ssusb, MTU3_DR_FORCE_DEVICE);
	ret = switch_port_to_device(ssusb);
	if (ret)
		return ret;

	spin_lock_irqsave(&mtu->lock, flags);
	mtu3_start(mtu);
	spin_unlock_irqrestore(&mtu->lock, flags);
	return 0;
}

/* Local xaga recovery interface; ordinary repeated role requests stay no-ops. */
static ssize_t device_recover_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ssusb_mtk *ssusb = dev_get_drvdata(dev);
	struct mtu3 *mtu = ssusb->u3d;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	device_lock(&mtu->g.dev);
	mutex_lock(&ssusb->otg_switch.role_lock);
	ret = ssusb_recover_unbound_device(ssusb);
	mutex_unlock(&ssusb->otg_switch.role_lock);
	device_unlock(&mtu->g.dev);
	pm_runtime_put(dev);
	dev_info(dev, "unbound device recovery result: %d\n", ret);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(device_recover);

static struct attribute *ssusb_recovery_attrs[] = {
	&dev_attr_device_recover.attr,
	NULL,
};
static const struct attribute_group ssusb_recovery_group = {
	.attrs = ssusb_recovery_attrs,
};

static void ssusb_set_mode(struct otg_switch_mtk *otg_sx, enum usb_role role)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);

	if (ssusb->dr_mode != USB_DR_MODE_OTG)
		return;

	WRITE_ONCE(otg_sx->desired_role, role);
	queue_work(system_freezable_wq, &otg_sx->dr_work);
}

static int ssusb_id_notifier(struct notifier_block *nb,
	unsigned long event, void *ptr)
{
	struct otg_switch_mtk *otg_sx =
		container_of(nb, struct otg_switch_mtk, id_nb);

	ssusb_set_mode(otg_sx, event ? USB_ROLE_HOST : USB_ROLE_DEVICE);

	return NOTIFY_DONE;
}

static int ssusb_extcon_register(struct otg_switch_mtk *otg_sx)
{
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct extcon_dev *edev = otg_sx->edev;
	int ret;

	/* extcon is optional */
	if (!edev)
		return 0;

	otg_sx->id_nb.notifier_call = ssusb_id_notifier;
	ret = devm_extcon_register_notifier(ssusb->dev, edev, EXTCON_USB_HOST,
					&otg_sx->id_nb);
	if (ret < 0) {
		dev_err(ssusb->dev, "failed to register notifier for USB-HOST\n");
		return ret;
	}

	ret = extcon_get_state(edev, EXTCON_USB_HOST);
	dev_dbg(ssusb->dev, "EXTCON_USB_HOST: %d\n", ret);

	/* default as host, switch to device mode if needed */
	if (!ret)
		ssusb_set_mode(otg_sx, USB_ROLE_DEVICE);

	return 0;
}

/*
 * We provide an interface via debugfs to switch between host and device modes
 * depending on user input.
 * This is useful in special cases, such as uses TYPE-A receptacle but also
 * wants to support dual-role mode.
 */
void ssusb_mode_switch(struct ssusb_mtk *ssusb, int to_host)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	ssusb_set_mode(otg_sx, to_host ? USB_ROLE_HOST : USB_ROLE_DEVICE);
}

void ssusb_set_force_mode(struct ssusb_mtk *ssusb,
			  enum mtu3_dr_force_mode mode)
{
	u32 value;

	value = mtu3_readl(ssusb->ippc_base, SSUSB_U2_CTRL(0));
	switch (mode) {
	case MTU3_DR_FORCE_DEVICE:
		value |= SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_HOST:
		value |= SSUSB_U2_PORT_FORCE_IDDIG;
		value &= ~SSUSB_U2_PORT_RG_IDDIG;
		break;
	case MTU3_DR_FORCE_NONE:
		value &= ~(SSUSB_U2_PORT_FORCE_IDDIG | SSUSB_U2_PORT_RG_IDDIG);
		break;
	default:
		return;
	}
	mtu3_writel(ssusb->ippc_base, SSUSB_U2_CTRL(0), value);
}

static int ssusb_role_sw_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct ssusb_mtk *ssusb = usb_role_switch_get_drvdata(sw);
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	ssusb_set_mode(otg_sx, role);

	return 0;
}

static enum usb_role ssusb_role_sw_get(struct usb_role_switch *sw)
{
	struct ssusb_mtk *ssusb = usb_role_switch_get_drvdata(sw);

	return ssusb->is_host ? USB_ROLE_HOST : USB_ROLE_DEVICE;
}

static int ssusb_role_sw_register(struct otg_switch_mtk *otg_sx)
{
	struct usb_role_switch_desc role_sx_desc = { 0 };
	struct ssusb_mtk *ssusb = otg_sx_to_ssusb(otg_sx);
	struct device *dev = ssusb->dev;
	enum usb_dr_mode mode;

	if (!otg_sx->role_sw_used)
		return 0;

	mode = usb_get_role_switch_default_mode(dev);
	if (mode == USB_DR_MODE_PERIPHERAL)
		otg_sx->default_role = USB_ROLE_DEVICE;
	else
		otg_sx->default_role = USB_ROLE_HOST;

	role_sx_desc.set = ssusb_role_sw_set;
	role_sx_desc.get = ssusb_role_sw_get;
	role_sx_desc.fwnode = dev_fwnode(dev);
	role_sx_desc.driver_data = ssusb;
	/* Only TCPM may assert attachment on boards using software VBUS detect. */
	role_sx_desc.allow_userspace_control = !otg_sx->sw_vbus_detect;
	otg_sx->role_sw = usb_role_switch_register(dev, &role_sx_desc);
	if (IS_ERR(otg_sx->role_sw))
		return PTR_ERR(otg_sx->role_sw);

	ssusb_set_mode(otg_sx, otg_sx->sw_vbus_detect ?
		       USB_ROLE_NONE : otg_sx->default_role);

	return 0;
}

int ssusb_otg_switch_init(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;
	int ret = 0;

	INIT_WORK(&otg_sx->dr_work, ssusb_mode_sw_work);
	mutex_init(&otg_sx->role_lock);
	otg_sx->sw_vbus_detect = otg_sx->role_sw_used &&
		of_machine_is_compatible("xiaomi,xaga");

	if (otg_sx->manual_drd_enabled)
		ssusb_dr_debugfs_init(ssusb);
	else if (otg_sx->role_sw_used)
		ret = ssusb_role_sw_register(otg_sx);
	else
		ret = ssusb_extcon_register(otg_sx);

	if (!ret && otg_sx->role_sw_used && of_machine_is_compatible("xiaomi,xaga")) {
		ret = device_add_group(ssusb->dev, &ssusb_recovery_group);
		if (ret) {
			ssusb_otg_switch_exit(ssusb);
			return ret;
		}
		otg_sx->recovery_registered = true;
	}

	return ret;
}

void ssusb_otg_switch_exit(struct ssusb_mtk *ssusb)
{
	struct otg_switch_mtk *otg_sx = &ssusb->otg_switch;

	if (otg_sx->recovery_registered) {
		device_remove_group(ssusb->dev, &ssusb_recovery_group);
		otg_sx->recovery_registered = false;
	}
	cancel_work_sync(&otg_sx->dr_work);
	usb_role_switch_unregister(otg_sx->role_sw);
}
