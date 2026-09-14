// SPDX-License-Identifier: GPL-2.0
/*
 * MT6895 DVFSRC bring-up: SW_REQ pinning + free-run release.
 *
 * The DVFSRC aggregates bandwidth requests from many clients and picks a
 * DRAM operating point; on this port nothing votes yet, so DRAM stays at
 * whatever level LK left behind (3200 Mbps on xaga).
 *
 * Register layout matches downstream mtk-dvfsrc.c mt6983 data (MT6895
 * shares that generation):
 *
 *   DVFSRC_SW_REQ (0x18): bits [15:12] dram level, [6:4] vcore level
 *   DVFSRC_LEVEL  (0x5f0): currently applied level (low 6 bits)
 *
 * Two EL3 VCOREFS services matter:
 *
 *   MTK_SIP_DVFSRC_INIT  (0x00) - bring the firmware up.  Downstream
 *                                 deliberately leaves a "high OPP" lock
 *                                 held at this point while it registers
 *                                 the interconnect/regulator stack.
 *   MTK_SIP_DVFSRC_START (0x01) - release that lock and let the DVFSRC
 *                                 free-run (downstream mtk-dvfsrc-start.c
 *                                 issues this at late_initcall_sync).
 *
 * This driver previously sent only INIT, which is why plain SW_REQ writes
 * landed in the register but never moved LEVEL: the firmware stayed
 * locked.  Probe now parks SW_REQ once and START is issued at
 * late_initcall_sync, so the hardware voters (MCUSYS/GPUSYS/EMI) can
 * drive DRAM on demand again.
 *
 * NOTE: there is a firmware quirk where a lone high-band request from the
 * boot state is ignored and the low band (9..15) has to be visited once
 * (0 -> 8 -> 9 -> N) to make a level latch.  That sequence drops DRAM to
 * 800 Mbps mid-way and wedges an active GPU, so it is NOT done here.  It
 * is only reachable via the latch_on_write opt-in, for experiments on a
 * headless unit.
 */
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define SW_REQ		0x18
#define LEVEL		0x5f0

#define MTK_SIP_VCOREFS_CONTROL \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
			   ARM_SMCCC_OWNER_SIP, 0x506)
#define MTK_SIP_DVFSRC_INIT	0x00
#define MTK_SIP_DVFSRC_START	0x01

struct xaga_dvfsrc_pin {
	void __iomem *base;
	bool fw_ready;
	unsigned int dram_type;
};

static struct xaga_dvfsrc_pin *xaga_dvfsrc;

/*
 * SW_REQ floor programmed once the DVFSRC lock is released.  SW_REQ is
 * only a floor: the hardware bandwidth voters raise the level above it on
 * demand (measured: floor 2 -> idle 2133, lifted to 4266..5500 under a
 * DRAM load, back down after).
 *
 * Higher value = faster (fmeter):
 *
 *   0 ->  800      4 -> 3200      8 -> 6400
 *   2 -> 2133      6 -> 5500
 *
 * Real idle power (no SSH on the link): 800 -> 0.9W, 2133 -> 1.0W.
 *
 * Default 0 (800) for the lowest idle; because the floor is not a pin,
 * the hardware still boosts under load.  9..15 is a separate low-power
 * band that wedges an active GPU and is kept out of the default path.
 */
static unsigned int boot_level;
module_param(boot_level, uint, 0644);
MODULE_PARM_DESC(boot_level,
		 "SW_REQ DRAM floor applied after free-run (0..8 high band)");

/* Release the DVFSRC high-OPP lock at late_initcall_sync. */
static bool enable_free_run = true;
module_param(enable_free_run, bool, 0644);
MODULE_PARM_DESC(enable_free_run,
		 "Send MTK_SIP_DVFSRC_START so the DVFSRC can free-run");

/*
 * Opt-in: replay 0 -> 8 -> 9 -> N on every dram_level_raw write so the
 * value always latches.  Costs a ~300ms excursion through 800 Mbps and
 * can wedge an active GPU, so it is off by default.
 */
static bool latch_on_write;
module_param(latch_on_write, bool, 0644);
MODULE_PARM_DESC(latch_on_write,
		 "Replay the 0->8->9->N latch on each dram_level_raw write");

/* Opt-in: allow runtime writes into the 9..15 low band (800 Mbps). */
static bool allow_low_band;
module_param(allow_low_band, bool, 0644);
MODULE_PARM_DESC(allow_low_band,
		 "Permit writing low-band levels (9..15, 800 Mbps)");

static u32 xpin_read(struct xaga_dvfsrc_pin *d, u32 off)
{
	return readl(d->base + off);
}

static void xpin_write(struct xaga_dvfsrc_pin *d, u32 off, u32 val)
{
	writel(val, d->base + off);
}

/* Write only the dram-level nibble of SW_REQ; vcore and the other
 * requester fields stay untouched.
 */
static void xpin_set_level(struct xaga_dvfsrc_pin *d, u32 lvl)
{
	xpin_write(d, SW_REQ,
		   (xpin_read(d, SW_REQ) & ~(0xf << 12)) |
		   ((lvl & 0xf) << 12));
}

/*
 * Firmware quirk workaround: visit the low band once so a high-band
 * request latches -- empirically 0 -> 8 -> 9 -> N ends pinned at N.
 * DDR shuffles are slow, so settle between steps.
 *
 * DANGER: the 8/9 steps drop DRAM to 800 Mbps and will wedge an active
 * GPU.  Only call this on a quiescent/headless system.
 */
static void xpin_latch_level(struct xaga_dvfsrc_pin *d, u32 lvl)
{
	static const u32 seq[] = { 0, 8, 9 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		xpin_set_level(d, seq[i]);
		msleep(100);
	}
	xpin_set_level(d, lvl);
	msleep(100);
}

/*
 * GPU -> DRAM floor coupling.
 *
 * The DVFSRC hardware voter is traffic-based, and under a GPU load it
 * thrashes between levels (measured under furmark: 800..6400 swinging
 * every ~0.25s, averaging ~2500) which starves the GPU.  Android never
 * relies on it alone -- ged/gpufreq/mmqos post sustained software votes.
 * Do the same crudely here: hold a DRAM floor derived from the GPU
 * frequency, which the devfreq simple_ondemand governor pins at max
 * while the GPU is loaded.
 */
struct xaga_gpu_dram_map {
	unsigned long min_freq;	/* inclusive */
	u32 level;
};

/*
 * The GPU's lowest OPP is ~219 MHz and there is no useful OPP between it
 * and ~400 MHz, so anything below 400 MHz is treated as idle and drops
 * straight to the boot floor (800 Mbps) rather than holding a pointless
 * mid level.
 */
static const struct xaga_gpu_dram_map xaga_gpu_dram_map[] = {
	{ 600000000, 8 },	/* 6400 Mbps */
	{ 400000000, 6 },	/* 5500 Mbps */
	{ 0,         0 },	/* idle -> boot_level (800 Mbps) */
};

static u32 xaga_gpu_freq_to_level(unsigned long freq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(xaga_gpu_dram_map); i++)
		if (freq >= xaga_gpu_dram_map[i].min_freq)
			return xaga_gpu_dram_map[i].level;

	return 0;
}

static int xaga_gpu_dvfs_notify(struct notifier_block *nb,
				unsigned long event, void *ptr)
{
	struct devfreq_freqs *freqs = ptr;
	u32 lvl;

	if (event != DEVFREQ_POSTCHANGE)
		return NOTIFY_DONE;
	if (!xaga_dvfsrc || !xaga_dvfsrc->fw_ready)
		return NOTIFY_DONE;

	lvl = xaga_gpu_freq_to_level(freqs->new);
	if (lvl < boot_level)
		lvl = boot_level;
	xpin_set_level(xaga_dvfsrc, lvl);

	return NOTIFY_DONE;
}

static struct notifier_block xaga_gpu_dvfs_nb = {
	.notifier_call = xaga_gpu_dvfs_notify,
};

static void xaga_dvfsrc_couple_gpu(void)
{
	struct device_node *np;
	struct devfreq *gpu;
	u32 lvl;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "arm,mali-valhall-csf");
	if (!np) {
		pr_warn("xaga-dvfsrc: no GPU node, DRAM floor stays static\n");
		return;
	}

	gpu = devfreq_get_devfreq_by_node(np);
	of_node_put(np);
	if (IS_ERR_OR_NULL(gpu)) {
		pr_warn("xaga-dvfsrc: no GPU devfreq, DRAM floor stays static\n");
		return;
	}

	ret = devfreq_register_notifier(gpu, &xaga_gpu_dvfs_nb,
					DEVFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		pr_warn("xaga-dvfsrc: GPU notifier registration failed %d\n",
			ret);
		return;
	}

	/* Apply the current GPU frequency immediately. */
	lvl = xaga_gpu_freq_to_level(gpu->previous_freq);
	if (lvl < boot_level)
		lvl = boot_level;
	xpin_set_level(xaga_dvfsrc, lvl);

	pr_info("xaga-dvfsrc: GPU-coupled DRAM floor active (gpu=%lu Hz -> level %u)\n",
		gpu->previous_freq, lvl);
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

	/* The low band parks DRAM at 800 Mbps, which wedges an active GPU;
	 * require an explicit opt-in.
	 */
	if (val >= 9 && !allow_low_band)
		return -EINVAL;

	if (latch_on_write)
		xpin_latch_level(d, val);
	else
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
		d->fw_ready = true;
		d->dram_type = res.a1;
		/*
		 * No SW_REQ park here: the firmware still holds its
		 * high-OPP lock until START, so writes at this point are
		 * dropped.  The floor is programmed from
		 * xaga_dvfsrc_free_run_init() instead.
		 */
	} else {
		dev_warn(&pdev->dev,
			 "VCOREFS init not supported by EL3 (a0=%ld); DRAM stays at boot OPP\n",
			 res.a0);
	}

	xaga_dvfsrc = d;
	dev_set_drvdata(&pdev->dev, d);
	dev_info(&pdev->dev,
		 "probe: parked level %u, SW_REQ=0x%08x LEVEL=0x%08x fw_ready=%d dram_type=%u\n",
		 boot_level, xpin_read(d, SW_REQ), xpin_read(d, LEVEL),
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

/*
 * Release the DVFSRC high-OPP lock once the rest of the kernel is up, so
 * the block can free-run on hardware bandwidth voters instead of sitting
 * pinned.  Mirrors downstream mtk-dvfsrc-start.c.
 */
static int __init xaga_dvfsrc_free_run_init(void)
{
	struct arm_smccc_res res;

	if (!enable_free_run)
		return 0;
	if (!xaga_dvfsrc || !xaga_dvfsrc->fw_ready)
		return 0;

	arm_smccc_smc(MTK_SIP_VCOREFS_CONTROL, MTK_SIP_DVFSRC_START,
		      0, 0, 0, 0, 0, 0, &res);
	pr_info("xaga-dvfsrc: START a0=%ld free-run released, SW_REQ=0x%08x LEVEL=0x%08x\n",
		res.a0, xpin_read(xaga_dvfsrc, SW_REQ),
		xpin_read(xaga_dvfsrc, LEVEL));

	/*
	 * SW_REQ writes only take effect once the lock is released, so the
	 * DRAM floor is programmed here rather than at probe.  With nothing
	 * else voting during boot this is the idle operating point; the
	 * hardware bandwidth voters lift it on demand from here on.
	 */
	if (boot_level <= 0xf) {
		xpin_set_level(xaga_dvfsrc, boot_level);
		pr_info("xaga-dvfsrc: floor=%u SW_REQ=0x%08x LEVEL=0x%08x\n",
			boot_level, xpin_read(xaga_dvfsrc, SW_REQ),
			xpin_read(xaga_dvfsrc, LEVEL));
	}

	/* Hold a sustained floor while the GPU is loaded. */
	xaga_dvfsrc_couple_gpu();

	return 0;
}
late_initcall_sync(xaga_dvfsrc_free_run_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MT6895 DVFSRC manual DDR-OPP pinning + free-run release");
