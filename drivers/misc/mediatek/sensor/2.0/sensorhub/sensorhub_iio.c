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
 * Five devices are registered (all fed by SCP samples, enabled lazily on
 * the first read):
 *   - "pearl-als"   : in_illuminance_raw / _scale      (ambient light)
 *   - "pearl-accel" : in_accel_{x,y,z}_raw / _scale    (screen rotation)
 *   - "pearl-gyro"  : in_anglvel_{x,y,z}_raw / _scale  (LSM6DSO gyro)
 *   - "pearl-mag"   : in_magn_{x,y,z}_raw / _scale     (AK09919 compass)
 *   - "pearl-steps" : in_count_raw                     (SCP step counter)
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
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/machine.h>
#include <linux/iio/sysfs.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/math.h>

#include "hf_sensor_type.h"
#include "sensorhub.h"

#define SENSORHUB_IIO_MAX_CHANS 4

/* magnetometer axes (the fused heading rides in the 4th slot) */
#define SENSORHUB_IIO_MAG_CHANS 3

/*
 * How long a raw read waits for the SCP to answer an enable request before
 * giving up. The SCP round trip is a few IPI cycles, so anything above the
 * firmware's own startup latency is fine; too large and udev/iio-sensor-proxy
 * block noticeably on a sensor that is simply absent.
 */
#define SENSORHUB_IIO_READ_TIMEOUT_MS 500

struct sensorhub_iio_state {
	struct iio_dev *indio_dev;
	int sensor_type;
	int num_chans;

	struct mutex lock;
	struct iio_trigger *trig;
	int32_t value[SENSORHUB_IIO_MAX_CHANS];
	int64_t timestamp;
	bool has_data;

	/*
	 * Waited on by read_raw(), woken by the sample callback once a sample
	 * lands. Without this, a read that races the enable request always
	 * returns -EAGAIN and userspace gives up before the SCP has had a
	 * chance to answer.
	 */
	wait_queue_head_t read_wait;

	int64_t delay_ns;
	bool active;

	/* Orientation of the sensor package, from DT "mount-matrix". */
	struct iio_mount_matrix mount_matrix;
};

extern atomic_t dbg_iio_cb_calls;
extern atomic_t dbg_iio_cb_nomatch;
extern atomic_t dbg_iio_read_raw;
extern int dbg_iio_last_type;

static struct sensorhub_iio_state *als_state;
static struct sensorhub_iio_state *accel_state;
static struct sensorhub_iio_state *gyro_state;
static struct sensorhub_iio_state *mag_state;
static struct sensorhub_iio_state *steps_state;

static struct sensorhub_iio_state ** const sensorhub_iio_states[] = {
	&als_state, &accel_state, &gyro_state, &mag_state, &steps_state,
};

static struct sensorhub_iio_state *sensorhub_iio_for_type(int sensor_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sensorhub_iio_states); i++) {
		struct sensorhub_iio_state *st = *sensorhub_iio_states[i];

		if (st && st->sensor_type == sensor_type)
			return st;
	}
	return NULL;
}

/* fused heading, implemented in the fixed-point section below */
static s32 sensorhub_mag_heading(s32 mx, s32 my, s32 mz, bool compensated);

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

	mutex_lock(&st->lock);
	for (i = 0; i < st->num_chans; i++)
		sample.chan[i] = st->value[i];
	mutex_unlock(&st->lock);

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

	atomic_inc(&dbg_iio_cb_calls);
	dbg_iio_last_type = s->sensor_type;
	if (!st) {
		atomic_inc(&dbg_iio_cb_nomatch);
		return;
	}

	mutex_lock(&st->lock);
	for (i = 0; i < st->num_chans; i++)
		st->value[i] = s->value[i];
	st->timestamp = s->timestamp;
	st->has_data = true;
	/*
	 * The magnetometer device also carries the fused heading so it can
	 * ride in the buffer (iio-sensor-proxy's compass reads it from the
	 * ring buffer, not from a direct sysfs read).
	 */
	if (st->sensor_type == SENSOR_TYPE_MAGNETIC_FIELD &&
	    st->num_chans > SENSORHUB_IIO_MAG_CHANS)
		st->value[SENSORHUB_IIO_MAG_CHANS] =
			sensorhub_mag_heading(s->value[0], s->value[1],
					      s->value[2], true);
	mutex_unlock(&st->lock);

	wake_up_interruptible(&st->read_wait);

	sensorhub_iio_push(st, s);
}

/* ------------------------------------------------------------------ */
/* Magnetic heading (fixed point; kernel code cannot use floats)       */
/* ------------------------------------------------------------------ */

/*
 * atan2 in centi-degrees via CORDIC (vectoring mode, 16 iterations).
 * Returns the angle of the vector (x, y) in (-18000..18000].  Inputs are
 * normalised first so the 16-bit pre-scale cannot overflow s64.
 */
static s32 sensorhub_atan2_cdeg(s64 y, s64 x)
{
	static const u16 atan_cdeg[16] = {
		4500, 2657, 1404, 712, 357, 179, 89, 45,
		22, 11, 6, 3, 1, 1, 0, 0
	};
	s64 angle = 0;
	bool flip = false;
	int i;

	if (x == 0 && y == 0)
		return 0;

	if (x < 0) {
		x = -x;
		y = -y;
		flip = true;
	}

	while (x > (1LL << 42) || x < -(1LL << 42) ||
	       y > (1LL << 42) || y < -(1LL << 42)) {
		x >>= 1;
		y >>= 1;
	}

	x <<= 16;
	y <<= 16;

	for (i = 0; i < ARRAY_SIZE(atan_cdeg); i++) {
		s64 nx, ny;

		if (y < 0) {
			nx = x - (y >> i);
			ny = y + (x >> i);
			angle += atan_cdeg[i];
		} else {
			nx = x + (y >> i);
			ny = y - (x >> i);
			angle -= atan_cdeg[i];
		}
		x = nx;
		y = ny;
	}

	angle = -angle;
	if (flip)
		angle = angle <= 0 ? angle + 18000 : angle - 18000;
	if (angle > 18000)
		angle -= 36000;
	if (angle <= -18000)
		angle += 36000;

	return (s32)angle;
}

/*
 * Heading of the device's top edge (+y) relative to magnetic north, in
 * centi-degrees.  Tilt compensated by projecting the magnetic vector and
 * the device top axis onto the horizontal plane using the latest
 * accelerometer sample; falls back to the raw x/y magnetic angle when no
 * accelerometer data is available or when "compensated" is false.
 *
 * The maths is entirely integer: the gravity vector is normalised to Q12
 * (4096 = 1 g), the horizontal projections are scaled by 4096 and the
 * cross product is scaled back before the atan2, so every intermediate
 * stays well inside s64.
 */
static s32 sensorhub_mag_heading(s32 mx, s32 my, s32 mz, bool compensated)
{
	s32 ax = 0, ay = 0, az = 0;
	s64 g2, g, axq, ayq, azq, dot;
	s64 nx, ny, nz, tx, ty, tz, cross_u, dot_nt;

	if (!compensated || !accel_state)
		return sensorhub_atan2_cdeg(my, mx);

	mutex_lock(&accel_state->lock);
	if (accel_state->has_data) {
		ax = accel_state->value[0];
		ay = accel_state->value[1];
		az = accel_state->value[2];
	}
	mutex_unlock(&accel_state->lock);

	g2 = (s64)ax * ax + (s64)ay * ay + (s64)az * az;
	if (!g2)
		return sensorhub_atan2_cdeg(my, mx);

	g = int_sqrt(g2);
	axq = ((s64)ax << 12) / g;
	ayq = ((s64)ay << 12) / g;
	azq = ((s64)az << 12) / g;

	/* horizontal magnetic north: n = m*4096 - (m.â)â */
	dot = (s64)mx * axq + (s64)my * ayq + (s64)mz * azq;
	nx = ((s64)mx << 12) - dot * axq;
	ny = ((s64)my << 12) - dot * ayq;
	nz = ((s64)mz << 12) - dot * azq;

	/* device top projected: t = (0,4096,0) - ây*â */
	tx = -ayq * axq;
	ty = ((s64)4096 << 12) - ayq * ayq;
	tz = -ayq * azq;

	/* scale n down so the cross product fits s64 */
	nx >>= 20;
	ny >>= 20;
	nz >>= 20;

	/* heading = atan2((n x t).â, n.t); â is Q12 so the cross is >> 12 */
	cross_u = ((ny * tz - nz * ty) * axq +
		   (nz * tx - nx * tz) * ayq +
		   (nx * ty - ny * tx) * azq) >> 12;
	dot_nt = nx * tx + ny * ty + nz * tz;

	return sensorhub_atan2_cdeg(cross_u, dot_nt);
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

/*
 * Gyroscope: same LSM6DSO package as the accelerometer, so the same mount
 * matrix (from DT) applies and is shared by type.
 */
static const struct iio_chan_spec sensorhub_gyro_channels[] = {
	{
		.type = IIO_ANGL_VEL,
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
		.type = IIO_ANGL_VEL,
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
		.type = IIO_ANGL_VEL,
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

/*
 * Magnetometer (AK09919): a separate package on the SCP bus.  Its mounting
 * is not documented anywhere yet, so the axes are passed through raw (no
 * mount matrix) until the compass heading can be calibrated on hardware.
 */
static const struct iio_chan_spec sensorhub_mag_channels[] = {
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = SENSORHUB_SCAN_TYPE,
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 1,
		.scan_type = SENSORHUB_SCAN_TYPE,
	},
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 2,
		.scan_type = SENSORHUB_SCAN_TYPE,
	},
	/*
	 * Fused heading (not buffered): iio-sensor-proxy's compass driver looks
	 * for in_rot_from_north_magnetic_tilt_comp; this kernel's IIO has no
	 * uncompensated variant of the modifier, so only expose the tilt
	 * compensated channel.
	 */
	{
		.type = IIO_ROT,
		.modified = 1,
		.channel2 = IIO_MOD_NORTH_MAGN_TILT_COMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = SENSORHUB_IIO_MAG_CHANS,
		.scan_type = SENSORHUB_SCAN_TYPE,
	},
};

/*
 * Step counter: the SCP keeps the count, single cumulative value.  There is
 * no dedicated IIO type for pedometers; IIO_COUNT with a raw channel is the
 * usual stand-in.
 */
static const struct iio_chan_spec sensorhub_steps_channels[] = {
	{
		.type = IIO_COUNT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = SENSORHUB_SCAN_TYPE,
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

	atomic_inc(&dbg_iio_read_raw);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * Enabling the sensor only tells the SCP to start streaming; the
		 * first sample always arrives later. Wait for it here instead of
		 * failing immediately, otherwise the very first read from
		 * userspace (which is how udev, iio-sensor-proxy and every
		 * orientation daemon probe the device) can never succeed.
		 */
		ret = sensorhub_iio_ensure_active(st);
		if (ret < 0)
			return ret;

		ret = wait_event_interruptible_timeout(st->read_wait,
				READ_ONCE(st->has_data),
				msecs_to_jiffies(SENSORHUB_IIO_READ_TIMEOUT_MS));
		if (ret < 0)
			return ret;	/* interrupted by a signal */
		if (ret == 0)
			return -ETIMEDOUT;

		mutex_lock(&st->lock);
		*val = st->value[chan->scan_index];
		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ROT) {
			/* centi-degrees -> radians: pi/180/100 */
			*val = 174533;
			*val2 = 1000000000;
			return IIO_VAL_FRACTIONAL;
		}

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

/*
 * Runs from the trigger's poll function (tasklet context), so it must not
 * sleep: use trylock and bail out if a sample is being updated concurrently.
 * The SCP pushes samples through sensorhub_iio_push() as well; this handler
 * only covers the case where userspace has a buffer enabled and the trigger
 * is firing without fresh data.
 */
static irqreturn_t sensorhub_iio_poll(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct sensorhub_iio_state *st = iio_priv(indio_dev);
	struct sensorhub_iio_sample sample = { 0 };
	int i;

	if (!iio_buffer_enabled(indio_dev))
		return IRQ_HANDLED;

	if (!mutex_trylock(&st->lock))
		return IRQ_HANDLED;
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


/*
 * The SCP streams as soon as the sensor is enabled, so the trigger state is
 * just the sensor enable state. Implementing this is mandatory for the
 * triggered buffer path: without it the IIO core cannot enable the buffer and
 * userspace (iio-sensor-proxy) is left with a device it can never read.
 */
static int sensorhub_iio_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct sensorhub_iio_state *st = iio_priv(indio_dev);

	if (state)
		return sensorhub_iio_ensure_active(st);

	return sensorhub_iio_deactivate(st);
}

static const struct iio_trigger_ops sensorhub_iio_trigger_ops = {
	.set_trigger_state = sensorhub_iio_set_trigger_state,
};

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
	int i;

	for (i = 0; i < ARRAY_SIZE(sensorhub_iio_states); i++) {
		if (*sensorhub_iio_states[i] == st)
			*sensorhub_iio_states[i] = NULL;
	}
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
	init_waitqueue_head(&st->read_wait);

	/*
	 * Falls back to the identity matrix when the device has no
	 * "mount-matrix" property, so the axes are passed through unchanged.
	 */
	ret = iio_read_mount_matrix(parent, &st->mount_matrix);
	if (ret < 0) {
		pr_err("%s: invalid mount matrix %d\n", name, ret);
		return ret;
	}
	dev_dbg(parent, "%s: mount matrix %s\n", name,
		st->mount_matrix.rotation[0]);

	indio_dev->name = name;
	indio_dev->info = &sensorhub_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
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
	st->trig = devm_iio_trigger_alloc(parent, "%s-dev%d", name,
				       iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &sensorhub_iio_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);

	ret = devm_iio_trigger_register(parent, st->trig);
	if (ret) {
		pr_err("%s: iio trigger register failed %d\n", name, ret);
		return ret;
	}

	indio_dev->trig = iio_trigger_get(st->trig);
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

	ret = sensorhub_iio_register_one(parent, "pearl-gyro",
		SENSOR_TYPE_GYROSCOPE, sensorhub_gyro_channels,
		ARRAY_SIZE(sensorhub_gyro_channels), &gyro_state);
	if (ret < 0)
		goto err_cb;

	ret = sensorhub_iio_register_one(parent, "pearl-mag",
		SENSOR_TYPE_MAGNETIC_FIELD, sensorhub_mag_channels,
		ARRAY_SIZE(sensorhub_mag_channels), &mag_state);
	if (ret < 0)
		goto err_cb;

	ret = sensorhub_iio_register_one(parent, "pearl-steps",
		SENSOR_TYPE_STEP_COUNTER, sensorhub_steps_channels,
		ARRAY_SIZE(sensorhub_steps_channels), &steps_state);
	if (ret < 0)
		goto err_cb;

	return 0;

err_cb:
	sensorhub_unregister_sample_cb(sensorhub_iio_sample_cb);
	return ret;
}

void sensorhub_iio_exit(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sensorhub_iio_states); i++) {
		struct sensorhub_iio_state *st = *sensorhub_iio_states[i];

		if (st && st->active)
			sensorhub_sensor_disable(st->sensor_type);
	}
	sensorhub_unregister_sample_cb(sensorhub_iio_sample_cb);
	for (i = 0; i < ARRAY_SIZE(sensorhub_iio_states); i++)
		*sensorhub_iio_states[i] = NULL;
}
