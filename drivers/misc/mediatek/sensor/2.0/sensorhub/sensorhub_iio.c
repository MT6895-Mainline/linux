// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 The Linux Foundation
 *
 * IIO frontend for the MT6895 SCP sensor hub.
 *
 * The SCP owns the physical ALS and IMU, so this driver does not touch any
 * I2C/SPI bus: it subscribes to decoded samples from sensorhub_main.c and
 * republishes them as ordinary IIO devices, which is what mainline userspace
 * (iio-sensor-proxy, desktop brightness/rotation daemons) expects.
 *
 * Two devices are registered:
 *   - "pearl-als"   : in_illuminance_raw / _scale      (ambient light)
 *   - "pearl-accel" : in_accel_{x,y,z}_raw / _scale    (screen rotation)
 *
 * Values are reported exactly as the firmware sends them, with the firmware's
 * per-sensor gain published as the channel scale. The raw/gain convention is
 * intentionally not "prettified" here; see the module parameter notes and the
 * porting report for the empirical validation of units.
 */

#define pr_fmt(fmt) "sensorhub_iio " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/machine.h>
#include <linux/iio/sysfs.h>

#include "hf_sensor_type.h"
#include "sensorhub.h"

#define SENSORHUB_IIO_MAX_CHANS 3

struct sensorhub_iio_state {
	struct iio_dev *indio_dev;
	int sensor_type;
	int num_chans;

	struct mutex lock;
	int32_t value[SENSORHUB_IIO_MAX_CHANS];
	int64_t timestamp;
	bool has_data;

	int64_t delay_ns;
	bool active;

	/* Orientation of the sensor package, from DT "mount-matrix". */
	struct iio_mount_matrix mount_matrix;
};

static struct sensorhub_iio_state *als_state;
static struct sensorhub_iio_state *accel_state;

static struct sensorhub_iio_state *sensorhub_iio_for_type(int sensor_type)
{
	if (als_state && als_state->sensor_type == sensor_type)
		return als_state;
	if (accel_state && accel_state->sensor_type == sensor_type)
		return accel_state;
	return NULL;
}

/*
 * The firmware list is the only authority on sensor presence, so the gain is
 * looked up on every scale read rather than cached: the list can change after
 * an SCP restart.
 */
static int sensorhub_iio_gain(int sensor_type)
{
	const struct sensor_info *info = sensorhub_find_sensor(sensor_type);

	if (!info || !info->gain)
		return 1;
	return info->gain;
}

/* ------------------------------------------------------------------ */
/* Sample path                                                         */
/* ------------------------------------------------------------------ */

struct sensorhub_iio_sample {
	int32_t chan[SENSORHUB_IIO_MAX_CHANS];
	s64 timestamp __aligned(8);
};

static void sensorhub_iio_push(struct sensorhub_iio_state *st,
		const struct sensorhub_sample *s)
{
	struct sensorhub_iio_sample sample = { 0 };
	int i;

	for (i = 0; i < st->num_chans; i++)
		sample.chan[i] = s->value[i];

	/*
	 * Push straight through when userspace has a buffer running. The
	 * triggered-buffer poll function covers the case where a trigger is
	 * assigned but the SCP is not streaming.
	 */
	if (iio_buffer_enabled(st->indio_dev))
		iio_push_to_buffers_with_timestamp(st->indio_dev, &sample,
			s->timestamp);
}

static void sensorhub_iio_sample_cb(const struct sensorhub_sample *s)
{
	struct sensorhub_iio_state *st = sensorhub_iio_for_type(s->sensor_type);
	int i;

	if (!st)
		return;

	mutex_lock(&st->lock);
	for (i = 0; i < st->num_chans; i++)
		st->value[i] = s->value[i];
	st->timestamp = s->timestamp;
	st->has_data = true;
	mutex_unlock(&st->lock);

	sensorhub_iio_push(st, s);
}

/* ------------------------------------------------------------------ */
/* Channel definitions                                                 */
/* ------------------------------------------------------------------ */

#define SENSORHUB_SCAN_TYPE {			\
	.sign = 's',				\
	.realbits = 32,				\
	.storagebits = 32,			\
	.endianness = IIO_LE,			\
}

static const struct iio_chan_spec sensorhub_als_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = SENSORHUB_SCAN_TYPE,
	},
};

/*
 * The SCP reports raw axes in the sensor package's own frame; the physical
 * mounting differs per device. Exposing a mount matrix lets the orientation be
 * corrected from DT ("mount-matrix") instead of by patching axis order, which
 * is what iio-sensor-proxy and desktop rotation daemons expect.
 */
static const struct iio_mount_matrix *
sensorhub_iio_get_mount_matrix(const struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan)
{
	struct sensorhub_iio_state *st = iio_priv(indio_dev);

	return &st->mount_matrix;
}

static const struct iio_chan_spec_ext_info sensorhub_accel_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_TYPE, sensorhub_iio_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec sensorhub_accel_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = SENSORHUB_SCAN_TYPE,
		.ext_info = sensorhub_accel_ext_info,
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 1,
		.scan_type = SENSORHUB_SCAN_TYPE,
		.ext_info = sensorhub_accel_ext_info,
	},
	{
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 2,
		.scan_type = SENSORHUB_SCAN_TYPE,
		.ext_info = sensorhub_accel_ext_info,
	},
};

/* ------------------------------------------------------------------ */
/* Sensor activation                                                   */
/* ------------------------------------------------------------------ */

static int sensorhub_iio_ensure_active(struct sensorhub_iio_state *st)
{
	int ret;

	if (st->active)
		return 0;

	/*
	 * The SCP may not have published its sensor list yet (or may have been
	 * restarted). Report that honestly instead of pretending to sample.
	 */
	ret = sensorhub_sensor_enable(st->sensor_type, st->delay_ns, 0);
	if (ret < 0)
		return ret;

	st->active = true;
	return 0;
}

static int sensorhub_iio_deactivate(struct sensorhub_iio_state *st)
{
	int ret;

	if (!st->active)
		return 0;

	ret = sensorhub_sensor_disable(st->sensor_type);
	st->active = false;
	return ret;
}

/* ------------------------------------------------------------------ */
/* read_raw / write_raw                                                */
/* ------------------------------------------------------------------ */

static int sensorhub_iio_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct sensorhub_iio_state *st = iio_priv(indio_dev);
	int gain, ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = sensorhub_iio_ensure_active(st);
		if (ret < 0)
			return ret;
		mutex_lock(&st->lock);
		if (!st->has_data) {
			mutex_unlock(&st->lock);
			return -EAGAIN;
		}
		*val = st->value[chan->scan_index];
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		gain = sensorhub_iio_gain(st->sensor_type);
		*val = 1;
		*val2 = gain;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = div_u64(NSEC_PER_SEC, st->delay_ns);
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int sensorhub_iio_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct sensorhub_iio_state *st = iio_priv(indio_dev);
	int ret = 0;
	int64_t delay;

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;
	if (val <= 0)
		return -EINVAL;

	delay = div_u64(NSEC_PER_SEC, val);
	st->delay_ns = delay;

	/* Apply immediately if the sensor is already streaming. */
	if (st->active)
		ret = sensorhub_sensor_enable(st->sensor_type, delay, 0);
	return ret;
}

static const struct iio_info sensorhub_iio_info = {
	.read_raw = sensorhub_iio_read_raw,
	.write_raw = sensorhub_iio_write_raw,
};

/* ------------------------------------------------------------------ */
/* Triggered buffer                                                    */
/* ------------------------------------------------------------------ */

static irqreturn_t sensorhub_iio_poll(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct sensorhub_iio_state *st = iio_priv(indio_dev);
	struct sensorhub_iio_sample sample = { 0 };
	int i;

	if (!iio_buffer_enabled(indio_dev))
		return IRQ_HANDLED;

	mutex_lock(&st->lock);
	if (!st->has_data) {
		mutex_unlock(&st->lock);
		return IRQ_HANDLED;
	}
	for (i = 0; i < st->num_chans; i++)
		sample.chan[i] = st->value[i];
	mutex_unlock(&st->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, &sample,
		iio_get_time_ns(indio_dev));
	return IRQ_HANDLED;
}

/*
 * Start/stop SCP sampling together with the IIO buffer. These callbacks run
 * under the IIO mlock mutex, so issuing control transfers here is allowed.
 * A failure to start is reported so userspace is not left reading a buffer
 * that will never fill.
 */
static int sensorhub_iio_buffer_postenable(struct iio_dev *indio_dev)
{
	struct sensorhub_iio_state *st = iio_priv(indio_dev);
	int ret;

	ret = sensorhub_iio_ensure_active(st);
	if (ret < 0)
		pr_err("%s: cannot start sampling: %d\n",
			indio_dev->name, ret);
	return ret;
}

static int sensorhub_iio_buffer_predisable(struct iio_dev *indio_dev)
{
	struct sensorhub_iio_state *st = iio_priv(indio_dev);

	sensorhub_iio_deactivate(st);
	return 0;
}

static const struct iio_buffer_setup_ops sensorhub_iio_buffer_ops = {
	.postenable = sensorhub_iio_buffer_postenable,
	.predisable = sensorhub_iio_buffer_predisable,
};

/* ------------------------------------------------------------------ */
/* Registration                                                        */
/* ------------------------------------------------------------------ */

static void sensorhub_iio_state_release(void *data)
{
	struct sensorhub_iio_state *st = data;

	if (st == als_state)
		als_state = NULL;
	else if (st == accel_state)
		accel_state = NULL;
}

static int sensorhub_iio_register_one(struct device *parent,
		const char *name, int sensor_type,
		const struct iio_chan_spec *channels, int num_chans,
		struct sensorhub_iio_state **out)
{
	struct iio_dev *indio_dev;
	struct sensorhub_iio_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(parent, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->indio_dev = indio_dev;
	st->sensor_type = sensor_type;
	st->num_chans = num_chans;
	st->delay_ns = 100000000; /* 10 Hz default */
	mutex_init(&st->lock);

	/*
	 * Falls back to the identity matrix when the device has no
	 * "mount-matrix" property, so the axes are passed through unchanged.
	 */
	ret = iio_read_mount_matrix(parent, &st->mount_matrix);
	if (ret < 0) {
		pr_err("%s: invalid mount matrix %d\n", name, ret);
		return ret;
	}

	indio_dev->name = name;
	indio_dev->info = &sensorhub_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = channels;
	indio_dev->num_channels = num_chans;

	ret = devm_iio_triggered_buffer_setup(parent, indio_dev,
		iio_pollfunc_store_time, sensorhub_iio_poll,
		&sensorhub_iio_buffer_ops);
	if (ret < 0) {
		pr_err("%s: triggered buffer setup failed %d\n", name, ret);
		return ret;
	}

	ret = devm_add_action_or_reset(parent, sensorhub_iio_state_release, st);
	if (ret < 0)
		return ret;

	ret = devm_iio_device_register(parent, indio_dev);
	if (ret < 0) {
		pr_err("%s: iio device register failed %d\n", name, ret);
		return ret;
	}

	*out = st;
	pr_info("registered %s for sensor type %d\n", name, sensor_type);
	return 0;
}

int sensorhub_iio_init(struct device *parent)
{
	int ret;

	ret = sensorhub_register_sample_cb(sensorhub_iio_sample_cb);
	if (ret < 0) {
		pr_err("sample callback registration failed %d\n", ret);
		return ret;
	}

	ret = sensorhub_iio_register_one(parent, "pearl-als",
		SENSOR_TYPE_LIGHT, sensorhub_als_channels,
		ARRAY_SIZE(sensorhub_als_channels), &als_state);
	if (ret < 0)
		goto err_cb;

	ret = sensorhub_iio_register_one(parent, "pearl-accel",
		SENSOR_TYPE_ACCELEROMETER, sensorhub_accel_channels,
		ARRAY_SIZE(sensorhub_accel_channels), &accel_state);
	if (ret < 0)
		goto err_cb;

	return 0;

err_cb:
	sensorhub_unregister_sample_cb(sensorhub_iio_sample_cb);
	return ret;
}

void sensorhub_iio_exit(void)
{
	if (als_state && als_state->active)
		sensorhub_sensor_disable(als_state->sensor_type);
	if (accel_state && accel_state->active)
		sensorhub_sensor_disable(accel_state->sensor_type);
	sensorhub_unregister_sample_cb(sensorhub_iio_sample_cb);
	als_state = NULL;
	accel_state = NULL;
}
