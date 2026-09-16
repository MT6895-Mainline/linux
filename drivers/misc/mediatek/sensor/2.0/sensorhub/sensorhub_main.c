// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 * Copyright (C) 2024 The Linux Foundation
 *
 * MT6895 SCP sensor-hub transport.
 *
 * This replaces the downstream "transceiver" layer. Downstream forwards
 * samples to the Android hf_manager character device; here the same protocol
 * feeds Linux consumers through a small callback interface, and the IIO
 * frontend in sensorhub_iio.c exposes ambient light and acceleration.
 *
 * Lifecycle, mirroring the downstream order:
 *   1. sensor_comm_init()  - register IPI handlers, hook the ready chain
 *   2. sensor_list_init()  - register the LIST notify and ring handlers
 *   3. timesync_init()     - prepare host/SCP time correlation
 *   4. host_ready_init()   - arm the platform READY notifier last
 * Once firmware reports ready we publish the shared-memory rings, start the
 * periodic time sync and enumerate the firmware sensor list.
 *
 * Runs on a partially ported stack, so it deliberately avoids the downstream
 * BUG_ON() paths and the automatic SCP reset (see ready.c).
 */

#define pr_fmt(fmt) "sensorhub " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/pm_wakeup.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>

#include "scp.h"
#include "ready.h"
#include "ipi_comm.h"
#include "sensor_comm.h"
#include "sensor_list.h"
#include "share_memory.h"
#include "timesync.h"
#include "sensorhub.h"

/* Batch deepest allowed sampling period/latency, in nanoseconds. */
#define SENSORHUB_MAX_DELAY	(1000000000LL)
#define SENSORHUB_MIN_DELAY	(5000000LL)	/* 200 Hz ceiling */
#define SENSORHUB_MAX_LATENCY	(60000000000LL)

struct sensorhub_dev {
	struct timesync_filter filter;

	struct share_mem shm_reader;
	struct share_mem_data shm_buffer[8];

	struct sensor_info support_list[SENSOR_TYPE_SENSOR_MAX];
	unsigned int support_size;

	/* serialises control transfers and sensor state */
	struct mutex lock;
	struct mutex cb_lock;
	sensorhub_sample_cb_t sample_cb;

	struct wakeup_source *wakeup_src;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct work_struct boot_work;

	struct sensor_comm_batch batch[SENSOR_TYPE_SENSOR_MAX];
	bool enabled[SENSOR_TYPE_SENSOR_MAX];

	atomic_t ready;
	atomic_t wp_dropped;
};

static struct sensorhub_dev shub;

static bool sensorhub_armed;
static DEFINE_SPINLOCK(wp_fifo_lock);
static DEFINE_KFIFO(wp_fifo, uint32_t, 32);

/*
 * The sensor hub is armed automatically at boot now that the mailbox
 * pin-index bug is fixed. Two knobs remain:
 *
 *   sensorhub.start_armed=0   (kernel command line) never arm automatically
 *   enable                    writing 1 arms it later, one-way
 *
 * The driver still comes up inert if start_armed=0, and every failure path is
 * bounded (see sensor_comm.c), so a protocol fault cannot loop the boot.
 */
static bool sensorhub_armed;
static bool sensorhub_start_armed = true;
module_param_named(start_armed, sensorhub_start_armed, bool, 0444);
MODULE_PARM_DESC(start_armed,
	"Arm the SCP sensor-hub handshake during boot (default 1)");

static int sensorhub_arm(void)
{
	int ret;

	if (READ_ONCE(sensorhub_armed))
		return 0;

	ret = host_ready_init();
	if (ret < 0) {
		pr_err("cannot arm sensor hub: %d\n", ret);
		return ret;
	}

	WRITE_ONCE(sensorhub_armed, true);
	pr_info("sensor hub armed\n");
	return 0;
}

static int sensorhub_enable_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;

	if (!on)
		return -EINVAL; /* armed is a one-way transition */

	return sensorhub_arm();
}

static int sensorhub_enable_get(char *buffer, const struct kernel_param *kp)
{
	return sysfs_emit(buffer, "%d\n", READ_ONCE(sensorhub_armed));
}

static const struct kernel_param_ops sensorhub_enable_ops = {
	.set = sensorhub_enable_set,
	.get = sensorhub_enable_get,
};

module_param_cb(enable, &sensorhub_enable_ops, NULL, 0644);
MODULE_PARM_DESC(enable,
	"Write 1 to arm the SCP sensor-hub handshake (one-way)");

/* ------------------------------------------------------------------ */
/* Sample delivery                                                     */
/* ------------------------------------------------------------------ */

static void sensorhub_deliver(struct sensorhub_dev *dev,
		const struct share_mem_data *src)
{
	struct sensorhub_sample s;
	sensorhub_sample_cb_t cb;

	if (src->sensor_type >= SENSOR_TYPE_SENSOR_MAX ||
			src->action >= MAX_ACTION) {
		pr_err_ratelimited("invalid sensor event %u %u\n",
			src->sensor_type, src->action);
		return;
	}

	/* Only real samples go to consumers; FLUSH/BIAS/CALI/TEMP/RAW are not
	 * IIO samples. They are still visible in the debug log above if the
	 * sensor type is out of range.
	 */
	if (src->action != DATA_ACTION)
		return;

	s.sensor_type = src->sensor_type;
	s.timestamp = src->timestamp + timesync_filter_get(&dev->filter);
	memcpy(s.value, src->value, sizeof(s.value));

	mutex_lock(&dev->cb_lock);
	cb = dev->sample_cb;
	if (cb)
		cb(&s);
	mutex_unlock(&dev->cb_lock);
}

static void sensorhub_read(struct sensorhub_dev *dev, uint32_t write_position)
{
	int ret = 0;
	unsigned int i = 0;
	struct share_mem *shm = &dev->shm_reader;
	struct share_mem_data *buffer = dev->shm_buffer;
	uint32_t size = sizeof(dev->shm_buffer);
	uint32_t item_size = sizeof(dev->shm_buffer[0]);

	ret = share_mem_seek(shm, write_position);
	if (ret < 0) {
		pr_err("%s seek fail %d\n", shm->name, ret);
		return;
	}

	while (1) {
		ret = share_mem_read(shm, buffer, size);
		if (ret < 0 || ret > size) {
			pr_err("%s read fail %d\n", shm->name, ret);
			break;
		}
		if (ret == 0)
			break;
		if (ret % item_size) {
			pr_err("%s size not item aligned %d\n", shm->name, ret);
			break;
		}
		for (i = 0; i < (ret / item_size); i++)
			sensorhub_deliver(dev, &buffer[i]);
	}
}

static void sensorhub_work_fn(struct work_struct *work)
{
	struct sensorhub_dev *dev = container_of(work, struct sensorhub_dev,
		work);
	uint32_t wp = 0;

	while (kfifo_out(&wp_fifo, &wp, 1))
		sensorhub_read(dev, wp);
}

static void sensorhub_data_notify(struct sensor_comm_notify *n,
		void *private_data)
{
	struct sensorhub_dev *dev = private_data;
	struct data_notify *dnotify = (struct data_notify *)n->value;
	unsigned long flags = 0;
	uint32_t wp = 0;

	if (n->command != SENS_COMM_NOTIFY_DATA_CMD &&
	    n->command != SENS_COMM_NOTIFY_FULL_CMD)
		return;

	/* Runs in the mailbox IRQ context: only touch the kfifo and the
	 * time-sync filter here, everything else is deferred.
	 */
	spin_lock_irqsave(&wp_fifo_lock, flags);
	timesync_filter_set(&dev->filter, dnotify->scp_timestamp,
		dnotify->scp_archcounter);
	if (kfifo_is_full(&wp_fifo)) {
		if (kfifo_out(&wp_fifo, &wp, 1))
			atomic_inc(&dev->wp_dropped);
	}
	wp = dnotify->write_position;
	kfifo_in(&wp_fifo, &wp, 1);
	spin_unlock_irqrestore(&wp_fifo_lock, flags);

	queue_work(dev->wq, &dev->work);
}

/* ------------------------------------------------------------------ */
/* Control path                                                        */
/* ------------------------------------------------------------------ */

static int sensorhub_comm_with(int sensor_type, int cmd,
		void *data, uint8_t length)
{
	int ret = 0;
	struct sensor_comm_ctrl *ctrl = NULL;

	ctrl = kzalloc(sizeof(*ctrl) + length, GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;
	ctrl->sensor_type = sensor_type;
	ctrl->command = cmd;
	ctrl->length = length;
	if (length)
		memcpy(ctrl->data, data, length);
	ret = sensor_comm_ctrl_send(ctrl, sizeof(*ctrl) + ctrl->length);
	kfree(ctrl);
	return ret;
}

static int sensorhub_clamp_batch(int64_t *delay, int64_t *latency)
{
	if (*delay < SENSORHUB_MIN_DELAY)
		*delay = SENSORHUB_MIN_DELAY;
	if (*delay > SENSORHUB_MAX_DELAY)
		*delay = SENSORHUB_MAX_DELAY;
	if (*latency < 0)
		*latency = 0;
	if (*latency > SENSORHUB_MAX_LATENCY)
		*latency = SENSORHUB_MAX_LATENCY;
	return 0;
}

int sensorhub_sensor_enable(int sensor_type, int64_t delay, int64_t latency)
{
	int ret = 0;
	struct sensor_comm_batch batch;

	if (sensor_type <= SENSOR_TYPE_INVALID ||
			sensor_type >= SENSOR_TYPE_SENSOR_MAX)
		return -EINVAL;
	if (!atomic_read(&shub.ready))
		return -ENODEV;

	sensorhub_clamp_batch(&delay, &latency);
	batch.delay = delay;
	batch.latency = latency;

	mutex_lock(&shub.lock);
	if (shub.enabled[sensor_type]) {
		/* Already streaming: just retune the batch parameters. */
		ret = sensorhub_comm_with(sensor_type,
			SENS_COMM_CTRL_ENABLE_CMD, &batch, sizeof(batch));
		if (ret >= 0)
			shub.batch[sensor_type] = batch;
		mutex_unlock(&shub.lock);
		return ret;
	}

	scp_register_sensor(SENS_FEATURE_ID, sensor_type);
	ret = sensorhub_comm_with(sensor_type, SENS_COMM_CTRL_ENABLE_CMD,
		&batch, sizeof(batch));
	if (ret >= 0) {
		shub.enabled[sensor_type] = true;
		shub.batch[sensor_type] = batch;
	} else {
		scp_deregister_sensor(SENS_FEATURE_ID, sensor_type);
	}
	mutex_unlock(&shub.lock);
	return ret;
}

int sensorhub_sensor_disable(int sensor_type)
{
	int ret = 0;

	if (sensor_type <= SENSOR_TYPE_INVALID ||
			sensor_type >= SENSOR_TYPE_SENSOR_MAX)
		return -EINVAL;
	if (!atomic_read(&shub.ready))
		return -ENODEV;

	mutex_lock(&shub.lock);
	if (!shub.enabled[sensor_type]) {
		mutex_unlock(&shub.lock);
		return 0;
	}
	ret = sensorhub_comm_with(sensor_type, SENS_COMM_CTRL_DISABLE_CMD,
		NULL, 0);
	shub.batch[sensor_type].delay = S64_MAX;
	shub.batch[sensor_type].latency = S64_MAX;
	shub.enabled[sensor_type] = false;
	scp_deregister_sensor(SENS_FEATURE_ID, sensor_type);
	mutex_unlock(&shub.lock);
	return ret;
}

int sensorhub_register_sample_cb(sensorhub_sample_cb_t cb)
{
	mutex_lock(&shub.cb_lock);
	if (shub.sample_cb) {
		mutex_unlock(&shub.cb_lock);
		return -EBUSY;
	}
	shub.sample_cb = cb;
	mutex_unlock(&shub.cb_lock);
	return 0;
}

void sensorhub_unregister_sample_cb(sensorhub_sample_cb_t cb)
{
	mutex_lock(&shub.cb_lock);
	if (shub.sample_cb == cb)
		shub.sample_cb = NULL;
	mutex_unlock(&shub.cb_lock);
}

unsigned int sensorhub_sensor_count(void)
{
	return shub.support_size;
}

const struct sensor_info *sensorhub_sensor_at(unsigned int index)
{
	if (index >= shub.support_size)
		return NULL;
	return &shub.support_list[index];
}

const struct sensor_info *sensorhub_find_sensor(int sensor_type)
{
	unsigned int i;

	for (i = 0; i < shub.support_size; i++) {
		if (shub.support_list[i].sensor_type == sensor_type)
			return &shub.support_list[i];
	}
	return NULL;
}

/* ------------------------------------------------------------------ */
/* Shared memory and bootup                                            */
/* ------------------------------------------------------------------ */

static int sensorhub_shm_cfg(struct share_mem_config *cfg, void *private_data)
{
	struct sensorhub_dev *dev = private_data;
	unsigned long flags = 0;

	spin_lock_irqsave(&wp_fifo_lock, flags);
	kfifo_reset(&wp_fifo);
	spin_unlock_irqrestore(&wp_fifo_lock, flags);

	dev->shm_reader.name = "sensorhub_r";
	dev->shm_reader.item_size = sizeof(struct share_mem_data);
	dev->shm_reader.buffer_full_detect = false;
	return share_mem_init(&dev->shm_reader, cfg);
}

/*
 * The firmware is told about every ring it defines, not just the ones Linux
 * consumes. Downstream registers handlers for all six payloads, so the
 * advertised set must match or the firmware may refuse to stream. These
 * filler rings are initialised but never read.
 */
struct sensorhub_filler_shm {
	struct share_mem shm;
	uint32_t item_size;
	const char *name;
};

static struct sensorhub_filler_shm filler_shm[] = {
	{ .item_size = sizeof(struct share_mem_super_data), .name = "super_r" },
	{ .item_size = sizeof(struct share_mem_debug), .name = "debug_r" },
	{ .item_size = sizeof(struct share_mem_cmd), .name = "cust_cmd_w" },
	{ .item_size = sizeof(struct share_mem_cmd), .name = "cust_cmd_r" },
};

static int sensorhub_filler_cfg(struct share_mem_config *cfg, void *private_data)
{
	struct sensorhub_filler_shm *filler = private_data;

	filler->shm.name = (char *)filler->name;
	filler->shm.item_size = filler->item_size;
	filler->shm.buffer_full_detect = false;
	return share_mem_init(&filler->shm, cfg);
}

static void sensorhub_dump_list(struct sensorhub_dev *dev)
{
	unsigned int i;
	const struct sensor_info *info;

	pr_info("firmware reports %u sensors\n", dev->support_size);
	for (i = 0; i < dev->support_size; i++) {
		info = &dev->support_list[i];
		pr_info("  type %3u gain %-8u name %-16s vendor %s\n",
			info->sensor_type, info->gain,
			info->name, info->vendor);
	}
}

static void sensorhub_restore_sensors(struct sensorhub_dev *dev)
{
	unsigned int i;
	int ret = 0;
	struct sensor_info *info;

	/*
	 * The SCP was restarted: re-arm every sensor the host had enabled.
	 * Batching parameters were lost with the firmware, so replay the
	 * cached ones.
	 */
	mutex_lock(&dev->lock);
	for (i = 0; i < dev->support_size; i++) {
		info = &dev->support_list[i];
		if (!dev->enabled[info->sensor_type])
			continue;
		ret = sensorhub_comm_with(info->sensor_type,
			SENS_COMM_CTRL_ENABLE_CMD,
			&dev->batch[info->sensor_type],
			sizeof(dev->batch[info->sensor_type]));
		if (ret < 0)
			pr_err("restore enable %u failed %d\n",
				info->sensor_type, ret);
	}
	mutex_unlock(&dev->lock);
}

static void sensorhub_boot_work_fn(struct work_struct *work)
{
	struct sensorhub_dev *dev = container_of(work, struct sensorhub_dev,
		boot_work);
	int ret = 0;

	/*
	 * Nothing here can work without the receive callbacks, and trying
	 * would only generate control transfers that can never be answered.
	 */
	if (!ipi_comm_handlers_ready()) {
		pr_err("sensor IPI handlers missing, aborting boot\n");
		return;
	}

	ret = share_mem_config();
	if (ret < 0) {
		pr_err("shared memory config failed %d\n", ret);
		return;
	}

	timesync_start();

	memset(dev->support_list, 0, sizeof(dev->support_list));
	dev->support_size = 0;
	ret = sensor_list_get_list(dev->support_list,
		ARRAY_SIZE(dev->support_list));
	if (ret < 0) {
		pr_err("sensor list request failed %d\n", ret);
		return;
	}
	dev->support_size = ret;
	sensorhub_dump_list(dev);

	sensorhub_restore_sensors(dev);
}

static int sensorhub_ready_notifier_call(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct sensorhub_dev *dev = &shub;

	if (event) {
		atomic_set(&dev->ready, 1);
		if (dev->wq)
			queue_work(dev->wq, &dev->boot_work);
	} else {
		atomic_set(&dev->ready, 0);
		timesync_stop();
	}

	return NOTIFY_DONE;
}

static struct notifier_block sensorhub_ready_notifier = {
	.notifier_call = sensorhub_ready_notifier_call,
	.priority = READY_HIGHPRI,
};

/* ------------------------------------------------------------------ */
/* Module init/exit                                                    */
/* ------------------------------------------------------------------ */

static int sensorhub_probe(struct platform_device *pdev)
{
	struct sensorhub_dev *dev = &shub;
	int ret = 0;

	mutex_init(&dev->lock);
	mutex_init(&dev->cb_lock);
	atomic_set(&dev->ready, 0);
	atomic_set(&dev->wp_dropped, 0);

	dev->filter.name = "sensorhub";
	ret = timesync_filter_init(&dev->filter);
	if (ret < 0) {
		pr_err("timesync filter init failed %d\n", ret);
		return ret;
	}

	dev->wakeup_src = wakeup_source_register(NULL, "sensorhub");
	if (!dev->wakeup_src) {
		ret = -ENOMEM;
		goto err_filter;
	}

	dev->wq = alloc_workqueue("sensorhub",
		WQ_MEM_RECLAIM | WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!dev->wq) {
		ret = -ENOMEM;
		goto err_wakeup;
	}
	INIT_WORK(&dev->work, sensorhub_work_fn);
	INIT_WORK(&dev->boot_work, sensorhub_boot_work_fn);

	/* Register the ring handler before any SCP traffic can arrive. */
	share_mem_config_handler_register(SHARE_MEM_DATA_PAYLOAD_TYPE,
		sensorhub_shm_cfg, dev);
	share_mem_config_handler_register(SHARE_MEM_SUPER_DATA_PAYLOAD_TYPE,
		sensorhub_filler_cfg, &filler_shm[0]);
	share_mem_config_handler_register(SHARE_MEM_DEBUG_PAYLOAD_TYPE,
		sensorhub_filler_cfg, &filler_shm[1]);
	share_mem_config_handler_register(SHARE_MEM_CUSTOM_W_PAYLOAD_TYPE,
		sensorhub_filler_cfg, &filler_shm[2]);
	share_mem_config_handler_register(SHARE_MEM_CUSTOM_R_PAYLOAD_TYPE,
		sensorhub_filler_cfg, &filler_shm[3]);
	sensor_comm_notify_handler_register(SENS_COMM_NOTIFY_DATA_CMD,
		sensorhub_data_notify, dev);
	sensor_comm_notify_handler_register(SENS_COMM_NOTIFY_FULL_CMD,
		sensorhub_data_notify, dev);

	ret = timesync_init();
	if (ret < 0)
		goto err_wq;

	ret = sensor_comm_init();
	if (ret < 0)
		goto err_timesync;

	ret = sensor_list_init();
	if (ret < 0)
		goto err_comm;

	sensor_ready_notifier_chain_register(&sensorhub_ready_notifier);

	ret = sensorhub_iio_init(&pdev->dev);
	if (ret < 0)
		goto err_ready;

	/*
	 * Arm the SCP handshake now that everything is registered. If
	 * start_armed=0 the driver stays inert and userspace can arm it later
	 * through the "enable" parameter.
	 */
	if (sensorhub_start_armed) {
		ret = sensorhub_arm();
		if (ret < 0) {
			/* Not fatal: the driver stays inert and usable. */
			pr_err("sensor hub left disarmed (%d)\n", ret);
		}
	} else {
		pr_info("sensor-hub transport ready, inert (start_armed=0); write 1 to %s to arm\n",
			"/sys/module/sensorhub/parameters/enable");
	}
	return 0;

err_ready:
	sensor_ready_notifier_chain_unregister(&sensorhub_ready_notifier);
	sensor_list_exit();
err_comm:
	sensor_comm_exit();
err_timesync:
	timesync_exit();
err_wq:
	sensor_comm_notify_handler_unregister(SENS_COMM_NOTIFY_DATA_CMD);
	sensor_comm_notify_handler_unregister(SENS_COMM_NOTIFY_FULL_CMD);
	share_mem_config_handler_unregister(SHARE_MEM_DATA_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_SUPER_DATA_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_DEBUG_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_CUSTOM_W_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_CUSTOM_R_PAYLOAD_TYPE);
	destroy_workqueue(dev->wq);
err_wakeup:
	wakeup_source_unregister(dev->wakeup_src);
err_filter:
	timesync_filter_exit(&dev->filter);
	return ret;
}

static void sensorhub_remove(struct platform_device *pdev)
{
	struct sensorhub_dev *dev = &shub;

	if (READ_ONCE(sensorhub_armed))
		host_ready_exit();
	sensorhub_iio_exit();
	sensor_ready_notifier_chain_unregister(&sensorhub_ready_notifier);
	sensor_comm_notify_handler_unregister(SENS_COMM_NOTIFY_DATA_CMD);
	sensor_comm_notify_handler_unregister(SENS_COMM_NOTIFY_FULL_CMD);
	share_mem_config_handler_unregister(SHARE_MEM_DATA_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_SUPER_DATA_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_DEBUG_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_CUSTOM_W_PAYLOAD_TYPE);
	share_mem_config_handler_unregister(SHARE_MEM_CUSTOM_R_PAYLOAD_TYPE);
	sensor_list_exit();
	sensor_comm_exit();
	timesync_exit();
	cancel_work_sync(&dev->work);
	cancel_work_sync(&dev->boot_work);
	destroy_workqueue(dev->wq);
	wakeup_source_unregister(dev->wakeup_src);
	timesync_filter_exit(&dev->filter);
}

static const struct of_device_id sensorhub_of_match[] = {
	{ .compatible = "mediatek,scp-sensorhub" },
	{ }
};
MODULE_DEVICE_TABLE(of, sensorhub_of_match);

static struct platform_driver sensorhub_driver = {
	.probe = sensorhub_probe,
	.remove = sensorhub_remove,
	.driver = {
		.name = "mtk-scp-sensorhub",
		.of_match_table = sensorhub_of_match,
	},
};
module_platform_driver(sensorhub_driver);

MODULE_DESCRIPTION("MT6895 SCP sensor-hub transport and IIO frontend");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MediaTek Inc.");

