// SPDX-License-Identifier: GPL-2.0
/*
 * XAGA: publish the SCP sensor hub through /dev/hf_manager.
 *
 * The MTK sensor HAL (vendor/lib64/hw/sensors.mediatek.V2.0.so, shipped for the
 * Waydroid Android stack) does not talk to the hub directly: it links
 * libhfmanager.so and reads sensors through /dev/hf_manager, whose list comes
 * from the hf_device registered here.  Without this, hf_manager has no device
 * at all, the HAL publishes no accelerometer/gyroscope/light, and the MTK
 * camera HAL's SensorProviderService fails with
 *
 *   doEnableSensor: Unable to find sensor type(GYRO), removed active user P1node-SenData
 *
 * before it ever hands a buffer to the kernel.
 *
 * The sensor list is the SCP firmware's own list (all types it reports, gyro
 * included); enable/disable/batch are forwarded to sensorhub_sensor_*(), and
 * samples decoded from the shared-memory rings are reported through
 * manager->report().
 */
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "../core/hf_manager.h"
#include "sensorhub.h"

#define SHUB_HF_NAME "xaga-sensorhub"

static struct hf_device shub_hf_dev;
static struct sensor_info shub_hf_list[SENSOR_TYPE_SENSOR_MAX];
static bool shub_hf_registered;

static int shub_hf_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	if (!sensorhub_find_sensor(sensor_type))
		return -EINVAL;

	if (en)
		return sensorhub_sensor_enable(sensor_type, 0, 0);

	return sensorhub_sensor_disable(sensor_type);
}

static int shub_hf_batch(struct hf_device *hfdev, int sensor_type,
			 int64_t delay, int64_t latency)
{
	if (!sensorhub_find_sensor(sensor_type))
		return -EINVAL;

	/* hf_manager keeps the state; (re)program the SCP for the new delay. */
	return sensorhub_sensor_enable(sensor_type, delay, latency);
}

static int shub_hf_flush(struct hf_device *hfdev, int sensor_type)
{
	return 0;
}

void sensorhub_hf_report(const struct sensorhub_sample *s)
{
	struct hf_manager *manager = shub_hf_dev.manager;
	struct hf_manager_event event;
	unsigned int i;

	if (!shub_hf_registered || !manager || !manager->report)
		return;

	memset(&event, 0, sizeof(event));
	event.timestamp = s->timestamp;
	event.sensor_type = s->sensor_type;
	event.action = DATA_ACTION;

	for (i = 0; i < ARRAY_SIZE(s->value) && i < ARRAY_SIZE(event.word); i++)
		event.word[i] = s->value[i];

	manager->report(manager, &event);
}

static unsigned int shub_hf_build_list(void)
{
	unsigned int i, n = sensorhub_sensor_count();

	if (n > ARRAY_SIZE(shub_hf_list))
		n = ARRAY_SIZE(shub_hf_list);

	for (i = 0; i < n; i++) {
		const struct sensor_info *info = sensorhub_sensor_at(i);

		if (!info)
			break;
		shub_hf_list[i] = *info;
	}

	shub_hf_dev.support_list = shub_hf_list;
	shub_hf_dev.support_size = i;

	return i;
}

int sensorhub_hf_init(void)
{
	unsigned int n;
	int ret;

	n = shub_hf_build_list();
	if (!n) {
		pr_info("sensorhub: no firmware sensors yet, hf_manager not registered\n");
		return -ENODEV;
	}

	shub_hf_dev.dev_name = SHUB_HF_NAME;
	shub_hf_dev.device_poll = HF_DEVICE_IO_POLLING;
	shub_hf_dev.device_bus = HF_DEVICE_IO_ASYNC;
	shub_hf_dev.enable = shub_hf_enable;
	shub_hf_dev.batch = shub_hf_batch;
	shub_hf_dev.flush = shub_hf_flush;

	/*
	 * Clients enumerate hfcore.device_list, which hf_manager_create() does
	 * not populate; the device must be registered first (downstream does
	 * hf_device_register() + hf_manager_create(), or the combined
	 * hf_device_register_manager_create()).
	 */
	ret = hf_device_register(&shub_hf_dev);
	if (ret) {
		pr_err("sensorhub: hf_device_register failed: %d\n", ret);
		return ret;
	}

	ret = hf_manager_create(&shub_hf_dev);
	if (ret) {
		pr_err("sensorhub: hf_manager_create failed: %d\n", ret);
		hf_device_unregister(&shub_hf_dev);
		return ret;
	}

	/*
	 * The sample callback slot is single-owner and the IIO frontend already
	 * took it, so samples are forwarded here from that callback
	 * (sensorhub_iio_sample_cb -> sensorhub_hf_report); nothing to register.
	 */
	shub_hf_registered = true;
	pr_info("sensorhub: hf_manager device \"%s\" registered, %u sensors\n",
		SHUB_HF_NAME, n);

	return 0;
}

void sensorhub_hf_exit(void)
{
	if (!shub_hf_registered)
		return;

	hf_manager_destroy(shub_hf_dev.manager);
	hf_device_unregister(&shub_hf_dev);
	shub_hf_registered = false;
}
