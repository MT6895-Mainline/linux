// SPDX-License-Identifier: GPL-2.0
/*
 * Manual DDR-OPP pinning for the MT6895 DVFSRC.
 *
 * The DVFSRC aggregates bandwidth requests from many clients and picks
 * a DRAM operating point; on this port nothing votes yet, so DRAM stays
 * at whatever level LK left behind (3200 Mbps on xaga). Until the full
 * interconnect/bw-vote stack is ported, expose the raw SW_REQ dram
 * field so userspace can pin an operating point by hand.
 *
 * Register layout matches downstream mtk-dvfsrc.c mt6983 data (MT6895
 * shares that generation):
 *
 *   DVFSRC_SW_REQ (0x18): bits [15:12] dram level, [6:4] vcore level
 *   DVFSRC_LEVEL  (0x5f0): currently applied level (low 6 bits)
 *
 * The block must be initialized through the EL3 VCOREFS service before
 * it services requests; probe issues MTK_SIP_DVFSRC_INIT and reports
 * whether the running firmware supports it.
 */
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <soc/mediatek/dramc.h>

#define SW_REQ		0x18
#define LEVEL		0x5f0

#define MTK_SIP_VCOREFS_CONTROL 	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, 			   ARM_SMCCC_OWNER_SIP, 0x506)
#define MTK_SIP_DVFSRC_INIT	0x00

static unsigned int boot_level = 0;
module_param(boot_level, uint, 0644);
MODULE_PARM_DESC(boot_level,
		 "DRAM level pinned at probe once the EB-side init succeeds");

static bool allow_low_band;
module_param(allow_low_band, bool, 0644);
MODULE_PARM_DESC(allow_low_band,
		 "Permit writing low-band levels (9..15) -- drops DRAM to "
		 "800 Mbps and will wedge an active GPU");

struct xaga_dvfsrc_pin {
	void __iomem *base;
	bool fw_ready;
	unsigned int dram_type;
};

static u32 xpin_read(struct xaga_dvfsrc_pin *d, u32 off)
{
	return readl(d->base + off);
}

static void xpin_write(struct xaga_dvfsrc_pin *d, u32 off, u32 val)
{
	writel(val, d->base + off);
}

/* Write only the dram-level nibble of SW_REQ; vcore and other
 * requester fields stay untouched.
 */
static void xpin_set_level(struct xaga_dvfsrc_pin *d, u32 lvl)
{
	xpin_write(d, SW_REQ,
		   (xpin_read(d, SW_REQ) & ~(0xf << 12)) |
		   ((lvl & 0xf) << 12));
}

static ssize_t dram_level_raw_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct xaga_dvfsrc_pin *d = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", (xpin_read(d, SW_REQ) >> 12) & 0xf);
}

static ssize_t dram_level_raw_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct xaga_dvfsrc_pin *d = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;
	if (val > 0xf)
		return -EINVAL;
	if (!d->fw_ready)
		return -EOPNOTSUPP;
	/* The low band (9..15) parks DRAM at 800 Mbps; with an active GPU
	 * that wedges jobs hard. Require an explicit opt-out.
	 */
	if (val >= 9 && !allow_low_band)
		return -EINVAL;

	xpin_set_level(d, val);

	return count;
}
static DEVICE_ATTR_RW(dram_level_raw);

static ssize_t level_applied_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct xaga_dvfsrc_pin *d = dev_get_drvdata(dev);

	return sprintf(buf, "0x%08x\n", xpin_read(d, LEVEL));
}
static DEVICE_ATTR_RO(level_applied);

static ssize_t sw_req_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct xaga_dvfsrc_pin *d = dev_get_drvdata(dev);

	return sprintf(buf, "0x%08x\n", xpin_read(d, SW_REQ));
}
static DEVICE_ATTR_RO(sw_req);

static ssize_t fw_ready_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct xaga_dvfsrc_pin *d = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", d->fw_ready);
}
static DEVICE_ATTR_RO(fw_ready);

static struct attribute *xaga_dvfsrc_attrs[] = {
	&dev_attr_dram_level_raw.attr,
	&dev_attr_level_applied.attr,
	&dev_attr_sw_req.attr,
	&dev_attr_fw_ready.attr,
	NULL,
};
ATTRIBUTE_GROUPS(xaga_dvfsrc);

static int xaga_dvfsrc_pin_probe(struct platform_device *pdev)
{
	struct arm_smccc_res res;
	struct xaga_dvfsrc_pin *d;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(d->base))
		return PTR_ERR(d->base);

	/* Ask EL3 to bring up the DVFSRC firmware. a0 == 0 means success;
	 * a1 carries the detected DRAM type.
	 */
	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, MTK_SIP_DVFSRC_INIT,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == 0) {
		static const u32 seq[] = { 0, 8, 9 };
		unsigned int i;
		int rate_before = mtk_dramc_get_data_rate();

		d->fw_ready = true;
		d->dram_type = res.a1;

		/* Bring-up quirk: the firmware ignores a direct high-band
		 * request from the boot state. Visiting the low band once
		 * makes high-band requests latch -- empirically
		 * 0 -> 8 -> 9 -> N ends pinned at N. Settle between steps;
		 * DDR shuffles are slow.
		 */
		for (i = 0; i < ARRAY_SIZE(seq); i++) {
			xpin_set_level(d, seq[i]);
			msleep(100);
		}
		if (boot_level <= 0xf) {
			xpin_set_level(d, boot_level);
			msleep(100);
		}

		dev_info(&pdev->dev,
			 "boot pin: %d -> %d Mbps (dram level %u)\n",
			 rate_before, mtk_dramc_get_data_rate(), boot_level);
	} else {
		dev_warn(&pdev->dev,
			 "VCOREFS init not supported by EL3 (a0=%ld); DRAM stays at boot OPP\n",
			 res.a0);
	}

	dev_set_drvdata(&pdev->dev, d);
	dev_info(&pdev->dev, "probe: SW_REQ=0x%08x LEVEL=0x%08x fw_ready=%d dram_type=%u\n",
		 xpin_read(d, SW_REQ), xpin_read(d, LEVEL),
		 d->fw_ready, d->dram_type);

	return 0;
}

static const struct of_device_id xaga_dvfsrc_of_match[] = {
	{ .compatible = "mediatek,mt6895-dvfsrc-pin" },
	{ }
};
MODULE_DEVICE_TABLE(of, xaga_dvfsrc_of_match);

static struct platform_driver xaga_dvfsrc_pin_drv = {
	.probe = xaga_dvfsrc_pin_probe,
	.driver = {
		.name = "mt6895-dvfsrc-pin",
		.of_match_table = xaga_dvfsrc_of_match,
		.dev_groups = xaga_dvfsrc_groups,
	},
};
module_platform_driver(xaga_dvfsrc_pin_drv);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6895 DVFSRC manual DDR-OPP pinning");
