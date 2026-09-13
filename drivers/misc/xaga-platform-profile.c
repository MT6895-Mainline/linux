// SPDX-License-Identifier: GPL-2.0
/*
 * xaga-platform-profile.c - fake ACPI platform_profile bridge for the
 * Redmi Note 11T Pro (xaga, MT6895).
 *
 * power-profiles-daemon (PPD) only understands the legacy ACPI platform
 * profile interface:
 *
 *   /sys/firmware/acpi/platform_profile_choices
 *   /sys/firmware/acpi/platform_profile
 *
 * The upstream platform_profile subsystem refuses to initialise when ACPI
 * is disabled (drivers/acpi/platform_profile.c returns -EOPNOTSUPP if
 * acpi_disabled), and PPD does not read the newer
 * /sys/class/platform-profile/ interface either.  ACPI is disabled on this
 * devicetree-booted phone, so PPD falls back to its do-nothing placeholder.
 *
 * This driver fakes just enough of the legacy ACPI interface for PPD to
 * probe successfully, and maps the three profiles onto the MT6895 cpufreq
 * governors via mtk_cpufreq_apply_profile():
 *
 *   low-power   -> powersave
 *   balanced    -> schedutil
 *   performance -> performance
 *
 * It is not a generic platform_profile provider and is only meant for xaga.
 */

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/sysfs.h>

/* Provided by drivers/cpufreq/mediatek-cpufreq-hw.c */
extern int mtk_cpufreq_apply_profile(const char *profile);

#define XAGA_PROFILE_NAME_MAX 16

static struct kobject *xaga_pp_kobj;
static char xaga_current_profile[XAGA_PROFILE_NAME_MAX] = "balanced";

static ssize_t platform_profile_choices_show(struct kobject *kobj,
					     struct kobj_attribute *attr,
					     char *buf)
{
	return sysfs_emit(buf, "low-power balanced performance\n");
}

static ssize_t profile_show(struct kobject *kobj,
			    struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%s\n", xaga_current_profile);
}

static ssize_t profile_store(struct kobject *kobj,
			     struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	char name[XAGA_PROFILE_NAME_MAX];
	const char *profile;
	int ret;

	strscpy(name, buf, sizeof(name));
	profile = strim(name);

	if (strcmp(profile, "low-power") && strcmp(profile, "quiet") &&
	    strcmp(profile, "balanced") && strcmp(profile, "performance"))
		return -EINVAL;

	ret = mtk_cpufreq_apply_profile(profile);
	if (ret)
		return ret;

	strscpy(xaga_current_profile, profile, sizeof(xaga_current_profile));

	/*
	 * Wake up pollers (PPD uses POLLPRI on this file for changes it did
	 * not make itself).
	 */
	sysfs_notify(kobj, NULL, "platform_profile");

	return count;
}

static struct kobj_attribute platform_profile_attr =
	__ATTR(platform_profile, 0644, profile_show, profile_store);
static struct kobj_attribute platform_profile_choices_attr =
	__ATTR_RO(platform_profile_choices);

static struct attribute *xaga_pp_attrs[] = {
	&platform_profile_attr.attr,
	&platform_profile_choices_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(xaga_pp);

static int __init xaga_platform_profile_init(void)
{
	int ret;

#if IS_ENABLED(CONFIG_ACPI)
	/*
	 * If real ACPI is present it already owns /sys/firmware/acpi and the
	 * platform_profile subsystem; there is nothing to fake.
	 */
	if (acpi_kobj) {
		pr_info("xaga-platform-profile: real ACPI present, not faking\n");
		return -EEXIST;
	}
#endif

	/*
	 * firmware_kobj is /sys/firmware.  Creating the "acpi" kobject here is
	 * a deliberate lie: ACPI is disabled on this DT platform, so nothing
	 * else owns that name.
	 */
	xaga_pp_kobj = kobject_create_and_add("acpi", firmware_kobj);
	if (!xaga_pp_kobj)
		return -ENOMEM;

	ret = sysfs_create_groups(xaga_pp_kobj, xaga_pp_groups);
	if (ret) {
		kobject_put(xaga_pp_kobj);
		xaga_pp_kobj = NULL;
		return ret;
	}

	/* Best effort: start in balanced.  Governors are registered by now. */
	ret = mtk_cpufreq_apply_profile("balanced");
	if (ret)
		pr_warn("xaga-platform-profile: could not set initial balanced governor: %d\n",
			ret);

	pr_info("xaga-platform-profile: fake ACPI platform_profile registered\n");
	return 0;
}

static void __exit xaga_platform_profile_exit(void)
{
	if (!xaga_pp_kobj)
		return;

	sysfs_remove_groups(xaga_pp_kobj, xaga_pp_groups);
	kobject_put(xaga_pp_kobj);
	xaga_pp_kobj = NULL;
}

late_initcall(xaga_platform_profile_init);
module_exit(xaga_platform_profile_exit);

MODULE_AUTHOR("xaga mainline port");
MODULE_DESCRIPTION("Fake ACPI platform_profile bridge for PPD (xaga / MT6895)");
MODULE_LICENSE("GPL");
