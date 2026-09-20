// SPDX-License-Identifier: GPL-2.0
/*
 * pearl_logdump: periodically write kernel dmesg to a raw block device
 * so logs survive hard hangs and watchdog resets.
 *
 * Uses a kernel timer to flush the kmsg ring buffer to the
 * "cust" partition every 5 seconds.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/blkdev.h>
#include <linux/kmsg_dump.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define LOGDUMP_INTERVAL_HZ 5
#define LOGDUMP_MAX_BYTES 0x40000 /* 256KB per dump */

static struct block_device *logdump_bdev;
static struct workqueue_struct *logdump_wq;
static struct delayed_work logdump_work;
static bool logdump_active;
static kmsg_dump_t logdump_reason = KMSG_DUMP_EMERG;

static void logdump_write_fn(struct work_struct *work)
{
	struct kmsg_dump_iter iter;
	char *buf;
	size_t len;
	loff_t pos = 0;
	struct file *filp;
	int ret;

	if (!logdump_bdev)
		return;

	buf = kmalloc(LOGDUMP_MAX_BYTES, GFP_KERNEL);
	if (!buf)
		return;

	kmsg_dump_rewind(&iter);
	len = kmsg_dump_get_buffer(&iter, true, buf,
		LOGDUMP_MAX_BYTES, NULL);
	if (len == 0) {
		kfree(buf);
		goto requeue;
	}

	filp = filp_open("/dev/sdc83", O_WRONLY | O_LARGEFILE, 0);
	if (IS_ERR(filp)) {
		kfree(buf);
		goto requeue;
	}

	kernel_write(filp, buf, len, &pos);
	vfs_fsync(filp, 0);
	filp_close(filp, NULL);
	kfree(buf);

requeue:
	queue_delayed_work(logdump_wq, &logdump_work,
		LOGDUMP_INTERVAL_HZ * HZ);
}

static int __init pearl_logdump_init(void)
{
	dev_t devt;
	struct device *dev;

	/* cust partition: find by partition name or major:minor */
	/* sdc83 -> major from device tree or lookup */
	/* For now use name-based lookup */
	dev = class_find_device_by_name(&block_class_type, "sdc83");
	if (!dev) {
		pr_info("pearl_logdump: sdc83 not found yet, will retry\n");
		return -EPROBE_DEFER;
	}

	logdump_bdev = bdgrab(dev_to_bdev(dev));
	put_device(dev);
	if (!logdump_bdev) {
		pr_info("pearl_logdump: bdgrab fail\n");
		return -1;
	}

	logdump_wq = alloc_workqueue("pearl_logdump", WQ_MEM_RECLAIM, 0);
	if (!logdump_wq) {
		bdput(logdump_bdev);
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&logdump_work, logdump_write_fn);
	queue_delayed_work(logdump_wq, &logdump_work, 10 * HZ);

	logdump_active = true;
	pr_info("pearl_logdump: active, writing dmesg to cust every %ds\n",
		LOGDUMP_INTERVAL_HZ);
	return 0;
}

static void __exit pearl_logdump_exit(void)
{
	if (logdump_active) {
		cancel_delayed_work_sync(&logdump_work);
		destroy_workqueue(logdump_wq);
		bdput(logdump_bdev);
	}
}

late_initcall(pearl_logdump_init);
module_exit(pearl_logdump_exit);
MODULE_LICENSE("GPL");
