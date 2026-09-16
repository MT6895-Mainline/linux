// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * SCP <-> host time synchronisation.
 *
 * The SCP timestamps samples with its own view of the system counter. For each
 * data notification we pair the SCP timestamp with a freshly read host
 * timestamp and architectural counter, correct for the IPI transfer delay,
 * and push (host - scp) into a small averaging filter. Sample timestamps are
 * then offset into host boot time.
 *
 * The counter-to-nanoseconds conversion is derived from CNTFRQ rather than
 * hard-coded to the 13 MHz assumed downstream, so the offset stays correct if
 * the counter runs at a different rate.
 */

#define pr_fmt(fmt) "timesync " fmt

#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/timer.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/sched/clock.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/math64.h>
#include <asm/arch_timer.h>

#include "timesync.h"
#include "sensor_comm.h"

#define ARCH_TIMER_SHIFT 21

static u32 arch_timer_mult = 161319385; /* 1e9/13MHz << 21 */
static bool timesync_suspend_flag;
static struct timer_list timesync_timer;
static struct work_struct timesync_work;
static struct wakeup_source *wakeup_src;

static inline int64_t arch_counter_to_ns(int64_t cyc)
{
	return (cyc * arch_timer_mult) >> ARCH_TIMER_SHIFT;
}

static void timesync_filter_calculate(struct timesync_filter *filter,
		int64_t host_time, int64_t scp_time)
{
	int i = 0;
	int64_t avg = 0, delta = 0;
	unsigned long flags;

	spin_lock_irqsave(&filter->lock, flags);
	if (host_time > filter->last_time + filter->max_diff ||
			filter->last_time == 0) {
		filter->tail = 0;
		filter->cnt = 0;
	} else if (host_time < filter->last_time + filter->min_diff) {
		spin_unlock_irqrestore(&filter->lock, flags);
		return;
	}
	filter->last_time = host_time;

	filter->buffer[filter->tail++] = host_time - scp_time;
	filter->tail &= (filter->bufsize - 1);
	if (filter->cnt < filter->bufsize)
		filter->cnt++;

	for (i = 1, avg = 0; i < filter->cnt; i++)
		avg += (filter->buffer[i] - filter->buffer[0]);
	filter->offset = div_s64(avg, filter->cnt) + filter->buffer[0];
	if (!filter->offset_debug) {
		filter->offset_debug = filter->offset;
	} else {
		delta = filter->offset_debug - filter->offset;
		if (unlikely(delta >= 2500000) || unlikely(delta <= -2500000))
			pr_err("%s host with scp jump too large %lld\n",
				filter->name, delta);
		filter->offset_debug = filter->offset;
	}
	spin_unlock_irqrestore(&filter->lock, flags);
}

static int timesync_comm_with_nolock(void)
{
	int ret = 0;
	unsigned long flags;
	struct sensor_comm_ctrl *ctrl = NULL;
	struct sensor_comm_timesync *time = NULL;
	int64_t now_time = 0, arch_counter = 0, schedclock = 0;
	struct timespec64 real_time = { 0 };
	struct rtc_time android_time;

	if (READ_ONCE(timesync_suspend_flag))
		return 0;

	ctrl = kzalloc(sizeof(*ctrl) + sizeof(*time), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->sensor_type = 0;
	ctrl->command = SENS_COMM_CTRL_TIMESYNC_CMD;
	ctrl->length = sizeof(*time);
	time = (struct sensor_comm_timesync *)ctrl->data;

	local_irq_save(flags);
	now_time = ktime_get_boottime_ns();
	arch_counter = __arch_counter_get_cntvct();
	schedclock = sched_clock();
	ktime_get_real_ts64(&real_time);
	local_irq_restore(flags);

	time->host_timestamp = now_time;
	time->host_archcounter = arch_counter;
	time->sched_clock = schedclock;
	real_time.tv_sec -= (uint64_t)sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(real_time.tv_sec, &android_time);
	time->usecond = real_time.tv_nsec / 1000;
	time->second = android_time.tm_sec;
	time->minute = android_time.tm_min;
	time->hour = android_time.tm_hour;
	time->day = android_time.tm_mday;
	time->month = android_time.tm_mon + 1;
	pr_debug("boot %lld sched %lld android %02d-%02d %02d:%02d:%02d.%06d\n",
		now_time, schedclock, time->month, time->day, time->hour,
		time->minute, time->second, time->usecond);
	ret = sensor_comm_ctrl_send(ctrl, sizeof(*ctrl) + ctrl->length);
	kfree(ctrl);

	return ret;
}

static int timesync_comm_with(void)
{
	int ret = 0;

	if (READ_ONCE(timesync_suspend_flag))
		return 0;

	__pm_stay_awake(wakeup_src);
	ret = timesync_comm_with_nolock();
	__pm_relax(wakeup_src);
	return ret;
}

static void timesync_work_func(struct work_struct *work)
{
	timesync_comm_with();
}

static void timesync_timer_func(struct timer_list *list)
{
	schedule_work(&timesync_work);
	timesync_start();
}

void timesync_filter_set(struct timesync_filter *filter,
		int64_t scp_timestamp, int64_t scp_archcounter)
{
	unsigned long flags;
	int64_t host_timestamp = 0, host_archcounter = 0;
	int64_t ipi_transfer_time = 0;

	local_irq_save(flags);
	host_timestamp = ktime_get_boottime_ns();
	host_archcounter = __arch_counter_get_cntvct();
	local_irq_restore(flags);
	ipi_transfer_time = arch_counter_to_ns(host_archcounter -
		scp_archcounter);
	scp_timestamp = scp_timestamp + ipi_transfer_time;

	timesync_filter_calculate(filter, host_timestamp, scp_timestamp);
}

int64_t timesync_filter_get(struct timesync_filter *filter)
{
	unsigned long flags;
	int64_t offset = 0;

	spin_lock_irqsave(&filter->lock, flags);
	offset = filter->offset;
	spin_unlock_irqrestore(&filter->lock, flags);
	return offset;
}

int timesync_filter_init(struct timesync_filter *filter)
{
	if (!filter->max_diff)
		filter->max_diff = 10000000000LL;
	if (!filter->min_diff)
		filter->min_diff = 10000000LL;
	if (!filter->bufsize)
		filter->bufsize = 16;
	if (!filter->name)
		return -EINVAL;
	spin_lock_init(&filter->lock);
	filter->bufsize = roundup_pow_of_two(filter->bufsize);
	filter->buffer = kcalloc(filter->bufsize, sizeof(*filter->buffer),
			GFP_KERNEL);
	if (!filter->buffer)
		return -ENOMEM;
	return 0;
}

void timesync_filter_exit(struct timesync_filter *filter)
{
	kfree(filter->buffer);
	filter->buffer = NULL;
}

void timesync_start(void)
{
	mod_timer(&timesync_timer, jiffies + msecs_to_jiffies(10000));
}

void timesync_stop(void)
{
	timer_delete_sync(&timesync_timer);
}

void timesync_resume(void)
{
	pr_info("host resume boottime %lld\n", ktime_get_boottime_ns());
	WRITE_ONCE(timesync_suspend_flag, false);
	timesync_comm_with();
}

void timesync_suspend(void)
{
	pr_info("host suspend boottime %lld\n", ktime_get_boottime_ns());
	WRITE_ONCE(timesync_suspend_flag, true);
}

int timesync_init(void)
{
	u32 freq = arch_timer_get_cntfrq();

	if (freq)
		arch_timer_mult = div_u64((u64)NSEC_PER_SEC << ARCH_TIMER_SHIFT,
			freq);
	pr_info("arch counter %u Hz, mult %u shift %d\n",
		freq, arch_timer_mult, ARCH_TIMER_SHIFT);

	INIT_WORK(&timesync_work, timesync_work_func);
	timer_setup(&timesync_timer, timesync_timer_func, 0);
	wakeup_src = wakeup_source_register(NULL, "timesync");
	if (!wakeup_src) {
		pr_err("timesync wakeup source init fail\n");
		return -ENOMEM;
	}

	return 0;
}

void timesync_exit(void)
{
	timesync_stop();
	wakeup_source_unregister(wakeup_src);
	flush_work(&timesync_work);
}
