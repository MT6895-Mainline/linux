// SPDX-License-Identifier: GPL-2.0 or MIT
/* Copyright 2019 Collabora ltd. */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <linux/soc/mediatek/mt6895_gpueb.h>
#include <linux/devfreq_cooling.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>

#include <drm/drm_managed.h>

#include "panthor_devfreq.h"
#include "panthor_device.h"

/*
 * XAGA (MT6895): upper bound for the shader-complex ("stacks") clock.
 * Downstream pairs each rate with a VSTACK voltage (gpufreq_mt6895.h:
 * 368MHz@569mV ... 860MHz@750mV); without regulator control we must not
 * exceed the voltage envelope left by the bootloader (368MHz).
 */
/* Match the stock OS: this unit's STACK working table tops out at
 * 852MHz (SegmentID 2) and the EB applies AVS-trimmed voltages plus
 * VSRAM sequencing itself, so the full table is safe to expose.
 */
static unsigned int stack_max_rate = 852000000;
module_param(stack_max_rate, uint, 0644);
MODULE_PARM_DESC(stack_max_rate,
		 "Max shader-complex (stacks) clock rate in Hz");

/*
 * XAGA (MT6895): rate/voltage pairs for the shader-complex ("stacks")
 * domain, taken from the downstream gpufreq working table
 * (gpufreq_mt6895.h g_default_stack[]). The rail is VSTACK = MT6368 BUCK2.
 * Downstream raises VSTACK ahead of the clock and lowers it behind it.
 */
static const struct xaga_stack_opp {
	unsigned long rate;
	unsigned int volt_uv;
} xaga_stack_table[] = {
	{ 219000000UL, 500000 },
	{ 231000000UL, 506250 },
	{ 243000000UL, 512500 },
	{ 255000000UL, 518750 },
	{ 268000000UL, 525000 },
	{ 280000000UL, 531250 },
	{ 292000000UL, 537500 },
	{ 304000000UL, 543750 },
	{ 317000000UL, 550000 },
	{ 334000000UL, 556250 },
	{ 351000000UL, 562500 },
	{ 368000000UL, 568750 },
	{ 385000000UL, 575000 },
	{ 402000000UL, 581250 },
	{ 419000000UL, 587500 },
	{ 436000000UL, 593750 },
	{ 453000000UL, 600000 },
	{ 470000000UL, 606250 },
	{ 487000000UL, 612500 },
	{ 504000000UL, 618750 },
	{ 521000000UL, 625000 },
	{ 538000000UL, 631250 },
	{ 555000000UL, 637500 },
	{ 572000000UL, 643750 },
	{ 590000000UL, 650000 },
	{ 616000000UL, 656250 },
	{ 642000000UL, 662500 },
	{ 668000000UL, 668750 },
	{ 695000000UL, 675000 },
	{ 721000000UL, 681250 },
	{ 747000000UL, 687500 },
	{ 773000000UL, 693750 },
	{ 800000000UL, 700000 },
	{ 807000000UL, 706250 },
	{ 815000000UL, 712500 },
	{ 822000000UL, 718750 },
	{ 830000000UL, 725000 },
	{ 837000000UL, 731250 },
	{ 845000000UL, 737500 },
	{ 852000000UL, 743750 },
	{ 860000000UL, 750000 },
};

static unsigned int xaga_stack_volt(unsigned long rate)
{
	unsigned int i, uv = xaga_stack_table[0].volt_uv;

	for (i = 0; i < ARRAY_SIZE(xaga_stack_table); i++) {
		if (xaga_stack_table[i].rate <= rate)
			uv = xaga_stack_table[i].volt_uv;
		else
			break;
	}

	return uv;
}

/*
 * XAGA: GPUEB-owned DVFS. The firmware's working tables for this silicon
 * (GPU SegmentID 0 / STACK SegmentID 2, AVS-trimmed) were captured live
 * from /proc/gpufreqv2/ on a stock device. Indexes sent with CMD_COMMIT
 * are indexes into THESE tables, not into the DT OPP table -- the EB then
 * applies both PLLs, both bucks and its private VSRAM rails itself.
 */
static const unsigned int xaga_eb_gpu_freq[] = {
	950000000, 880000000, 800000000, 610000000, 430000000, 350000000,
};
static const unsigned int xaga_eb_stack_freq[] = {
	852000000, 845000000, 837000000, 830000000, 822000000, 815000000,
	807000000, 800000000, 773000000, 747000000, 721000000, 695000000,
	668000000, 642000000, 616000000, 590000000, 572000000, 555000000,
	538000000, 521000000, 504000000, 487000000, 470000000, 453000000,
	436000000, 419000000, 402000000, 385000000, 368000000, 351000000,
	334000000, 317000000, 304000000, 292000000, 280000000, 268000000,
	255000000, 243000000, 231000000, 219000000,
};

#define XAGA_EB_TARGET_GPU	1	/* gpufreq_ipi.h TARGET_GPU   */
#define XAGA_EB_TARGET_STACK	2	/* gpufreq_ipi.h TARGET_STACK */

static bool xaga_eb_failed;
static int xaga_eb_last_gpu_idx = -1;

static bool xaga_eb_mode(void)
{
	return !xaga_eb_failed && mt6895_gpueb_available();
}

/* Nearest working-table entry at or below freq. Tables are sorted
 * highest-first, so scan from the top down.
 */
static int xaga_eb_gpu_idx(unsigned long freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(xaga_eb_gpu_freq); i++) {
		if (freq >= xaga_eb_gpu_freq[i])
			return i;
	}
	return ARRAY_SIZE(xaga_eb_gpu_freq) - 1;
}

static int xaga_eb_stack_idx(unsigned long freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(xaga_eb_stack_freq); i++) {
		if (freq >= xaga_eb_stack_freq[i])
			return i;
	}
	return ARRAY_SIZE(xaga_eb_stack_freq) - 1;
}

static int xaga_eb_commit(struct panthor_device *ptdev, unsigned long freq)
{
	unsigned long sreq = min_t(unsigned long, freq, stack_max_rate);
	int gidx = xaga_eb_gpu_idx(freq);
	int sidx = xaga_eb_stack_idx(sreq);
	bool up = xaga_eb_last_gpu_idx >= 0 && gidx > xaga_eb_last_gpu_idx;
	int ret;

	/* Mirror downstream ordering: the core domain leads on the way up
	 * and trails on the way down.
	 */
	if (up)
		ret = mt6895_gpueb_commit(XAGA_EB_TARGET_GPU, gidx);
	else
		ret = mt6895_gpueb_commit(XAGA_EB_TARGET_STACK, sidx);
	if (!ret)
		ret = up ? mt6895_gpueb_commit(XAGA_EB_TARGET_STACK, sidx) :
			   mt6895_gpueb_commit(XAGA_EB_TARGET_GPU, gidx);

	if (ret) {
		dev_warn(ptdev->base.dev,
			 "GPUEB commit failed (gpu=%d stack=%d): %d, falling back to CCF\n",
			 gidx, sidx, ret);
		xaga_eb_failed = true;
		return ret;
	}

	xaga_eb_last_gpu_idx = gidx;
	ptdev->current_frequency = freq;
	return 0;
}

/**
 * struct panthor_devfreq - Device frequency management
 */
struct panthor_devfreq {
	/** @devfreq: devfreq device. */
	struct devfreq *devfreq;

	/** @vstack: VSTACK rail (MT6368 BUCK2) for the stacks domain, or NULL. */
	struct regulator *vstack;

	/** @gov_data: Governor data. */
	struct devfreq_simple_ondemand_data gov_data;

	/** @busy_time: Busy time. */
	ktime_t busy_time;

	/** @idle_time: Idle time. */
	ktime_t idle_time;

	/** @time_last_update: Last update time. */
	ktime_t time_last_update;

	/** @last_busy_state: True if the GPU was busy last time we updated the state. */
	bool last_busy_state;

	/**
	 * @lock: Lock used to protect busy_time, idle_time, time_last_update and
	 * last_busy_state.
	 *
	 * These fields can be accessed concurrently by panthor_devfreq_get_dev_status()
	 * and panthor_devfreq_record_{busy,idle}().
	 */
	spinlock_t lock;
};

static void panthor_devfreq_update_utilization(struct panthor_devfreq *pdevfreq)
{
	ktime_t now, last;

	now = ktime_get();
	last = pdevfreq->time_last_update;

	if (pdevfreq->last_busy_state)
		pdevfreq->busy_time += ktime_sub(now, last);
	else
		pdevfreq->idle_time += ktime_sub(now, last);

	pdevfreq->time_last_update = now;
}

static int panthor_devfreq_target(struct device *dev, unsigned long *freq,
				  u32 flags)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	dev_pm_opp_put(opp);

	/* XAGA: let the GPUEB apply the whole DVFS -- both PLLs, both bucks
	 * and its private VSRAM rails -- instead of driving CCF and the
	 * local regulator stack. Falls back to CCF on failure.
	 */
	if (xaga_eb_mode())
		return xaga_eb_commit(ptdev, *freq);

	err = dev_pm_opp_set_rate(dev, *freq);
	if (!err) {
		ptdev->current_frequency = *freq;

		/* XAGA (MT6895): scale the shader-complex "stacks" domain along
		 * with the core clock, pairing each rate with its VSTACK
		 * voltage like downstream gpufreq does. Voltage leads on the
		 * way up and trails on the way down. If the rail refuses to
		 * move up, refuse to raise the clock with it.
		 */
		if (ptdev->clks.stacks && !IS_ERR(ptdev->clks.stacks)) {
			unsigned long stack_rate =
				min_t(unsigned long, *freq, stack_max_rate);
			unsigned long cur_stack = clk_get_rate(ptdev->clks.stacks);
			unsigned int uv = xaga_stack_volt(stack_rate);
			bool up = stack_rate > cur_stack;
			bool have_vstack =
				!IS_ERR_OR_NULL(ptdev->devfreq->vstack);

			if (up && have_vstack &&
			    regulator_set_voltage(ptdev->devfreq->vstack,
						  uv, uv)) {
				dev_warn(dev,
					 "vstack raise to %u uV failed, stacks stay %lu Hz\n",
					 uv, cur_stack);
			} else {
				if (clk_set_rate(ptdev->clks.stacks, stack_rate))
					dev_warn(dev,
						 "failed to set stacks rate %lu\n",
						 stack_rate);
				else if (!up && have_vstack)
					regulator_set_voltage(ptdev->devfreq->vstack,
							      uv, uv);
			}
		}
	}

	return err;
}

static void panthor_devfreq_reset(struct panthor_devfreq *pdevfreq)
{
	pdevfreq->busy_time = 0;
	pdevfreq->idle_time = 0;
	pdevfreq->time_last_update = ktime_get();
}

static int panthor_devfreq_get_dev_status(struct device *dev,
					  struct devfreq_dev_status *status)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	/* XAGA: with the GPUEB owning the clocks, clk_get_rate() returns a
	 * stale CCF cache; the tracked value is authoritative for both paths.
	 */
	status->current_frequency = ptdev->current_frequency;

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);

	status->total_time = ktime_to_ns(ktime_add(pdevfreq->busy_time,
						   pdevfreq->idle_time));

	status->busy_time = ktime_to_ns(pdevfreq->busy_time);

	panthor_devfreq_reset(pdevfreq);

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);

	drm_dbg(&ptdev->base, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->busy_time / (status->total_time / 100),
		status->current_frequency / 1000 / 1000);

	return 0;
}

static struct devfreq_dev_profile panthor_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	.polling_ms = 50, /* ~3 frames */
	.target = panthor_devfreq_target,
	.get_dev_status = panthor_devfreq_get_dev_status,
};

int panthor_devfreq_init(struct panthor_device *ptdev)
{
	/* There's actually 2 regulators (mali and sram), but the OPP core only
	 * supports one.
	 *
	 * We assume the sram regulator is coupled with the mali one and let
	 * the coupling logic deal with voltage updates.
	 */
	static const char * const reg_names[] = { "mali", NULL };
	struct thermal_cooling_device *cooling;
	struct device *dev = ptdev->base.dev;
	struct panthor_devfreq *pdevfreq;
	struct dev_pm_opp *opp;
	unsigned long cur_freq;
	unsigned long freq = ULONG_MAX;
	int ret;

	pdevfreq = drmm_kzalloc(&ptdev->base, sizeof(*ptdev->devfreq), GFP_KERNEL);
	if (!pdevfreq)
		return -ENOMEM;

	ptdev->devfreq = pdevfreq;

	/* XAGA: optional VSTACK rail for the shader-complex domain. */
	pdevfreq->vstack = devm_regulator_get_optional(dev, "vstack");
	if (IS_ERR(pdevfreq->vstack)) {
		if (PTR_ERR(pdevfreq->vstack) == -ENODEV)
			pdevfreq->vstack = NULL;
		else
			return PTR_ERR(pdevfreq->vstack);
	}

	ret = devm_pm_opp_set_regulators(dev, reg_names);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Couldn't set OPP regulators\n");

		return ret;
	}

	ret = devm_pm_opp_of_add_table(dev);
	if (ret)
		return ret;

	spin_lock_init(&pdevfreq->lock);

	panthor_devfreq_reset(pdevfreq);

	if (xaga_eb_mode())
		/* The EB restored its own boot OPP after POWER_CONTROL. */
		cur_freq = 219000000;
	else
		cur_freq = clk_get_rate(ptdev->clks.core);

	/* Regulator coupling only takes care of synchronizing/balancing voltage
	 * updates, but the coupled regulator needs to be enabled manually.
	 *
	 * We use devm_regulator_get_enable_optional() and keep the sram supply
	 * enabled until the device is removed, just like we do for the mali
	 * supply, which is enabled when dev_pm_opp_set_opp(dev, opp) is called,
	 * and disabled when the opp_table is torn down, using the devm action.
	 *
	 * If we really care about disabling regulators on suspend, we should:
	 * - use devm_regulator_get_optional() here
	 * - call dev_pm_opp_set_opp(dev, NULL) before leaving this function
	 *   (this disables the regulator passed to the OPP layer)
	 * - call dev_pm_opp_set_opp(dev, NULL) and
	 *   regulator_disable(ptdev->regulators.sram) in
	 *   panthor_devfreq_suspend()
	 * - call dev_pm_opp_set_opp(dev, default_opp) and
	 *   regulator_enable(ptdev->regulators.sram) in
	 *   panthor_devfreq_resume()
	 *
	 * But without knowing if it's beneficial or not (in term of power
	 * consumption), or how much it slows down the suspend/resume steps,
	 * let's just keep regulators enabled for the device lifetime.
	 */
	ret = devm_regulator_get_enable_optional(dev, "sram");
	if (ret && ret != -ENODEV) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Couldn't retrieve/enable sram supply\n");
		return ret;
	}

	opp = devfreq_recommended_opp(dev, &cur_freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	panthor_devfreq_profile.initial_freq = cur_freq;
	ptdev->current_frequency = cur_freq;

	/*
	 * Set the recommend OPP this will enable and configure the regulator
	 * if any and will avoid a switch off by regulator_late_cleanup()
	 */
	if (!xaga_eb_mode()) {
		ret = dev_pm_opp_set_opp(dev, opp);
		dev_pm_opp_put(opp);
		if (ret) {
			DRM_DEV_ERROR(dev, "Couldn't set recommended OPP\n");
			return ret;
		}
	} else {
		dev_pm_opp_put(opp);
	}

	/* Find the fastest defined rate  */
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	ptdev->fast_rate = freq;

	dev_pm_opp_put(opp);

	/*
	 * Setup default thresholds for the simple_ondemand governor.
	 * The values are chosen based on experiments.
	 */
	pdevfreq->gov_data.upthreshold = 45;
	pdevfreq->gov_data.downdifferential = 5;

	pdevfreq->devfreq = devm_devfreq_add_device(dev, &panthor_devfreq_profile,
						    DEVFREQ_GOV_SIMPLE_ONDEMAND,
						    &pdevfreq->gov_data);
	if (IS_ERR(pdevfreq->devfreq)) {
		DRM_DEV_ERROR(dev, "Couldn't initialize GPU devfreq\n");
		ret = PTR_ERR(pdevfreq->devfreq);
		pdevfreq->devfreq = NULL;
		return ret;
	}

	cooling = devfreq_cooling_em_register(pdevfreq->devfreq, NULL);
	if (IS_ERR(cooling))
		DRM_DEV_INFO(dev, "Failed to register cooling device\n");

	return 0;
}

void panthor_devfreq_resume(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;

	if (!pdevfreq || !pdevfreq->devfreq)
		return;

	panthor_devfreq_reset(pdevfreq);

	drm_WARN_ON(&ptdev->base, devfreq_resume_device(pdevfreq->devfreq));
}

void panthor_devfreq_suspend(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;

	if (!pdevfreq || !pdevfreq->devfreq)
		return;

	drm_WARN_ON(&ptdev->base, devfreq_suspend_device(pdevfreq->devfreq));
}

void panthor_devfreq_record_busy(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	if (!pdevfreq || !pdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);
	pdevfreq->last_busy_state = true;

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
}

void panthor_devfreq_record_idle(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	if (!pdevfreq || !pdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);
	pdevfreq->last_busy_state = false;

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
}
