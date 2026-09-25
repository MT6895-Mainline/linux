// SPDX-License-Identifier: GPL-2.0
/*
 * mtk_hf_iio -- expose MediaTek SCP sensors as standard Linux IIO devices.
 *
 * Why this exists
 * ---------------
 * On this board (MT6895 / qqcandy) every motion and light sensor lives behind
 * the SCP coprocessor.  The MTK kernel stack (mtk_sensorhub + hf_manager) already
 * carries the whole set and hf_manager exports a real kernel client API
 * (hf_client_create / hf_client_find_sensor / hf_client_control_sensor /
 * hf_client_poll_sensor_timeout).  What was missing is a *consumer*: the stock
 * consumer is the Android HIDL sensor HAL, which cannot run on a plain systemd
 * rootfs (no binder/hwbinder/hwservicemanager).
 *
 * This driver is the native replacement.  It registers each sensor as a standard
 * IIO device so that iio-sensor-proxy, libiio, KDE/GNOME and any other
 * /sys/bus/iio consumer work unmodified:
 *
 *     accel -> in_accel_{x,y,z}_raw      (iio-sensor-proxy: accelerometer)
 *     gyro  -> in_anglvel_{x,y,z}_raw    (gyroscope)
 *     magn  -> in_magn_{x,y,z}_raw       (compass)
 *     als   -> in_illuminance_raw        (ambient light)
 *     prox  -> in_proximity_raw          (proximity)
 *
 * Each device supports the direct (polled read_raw) path and a triggered buffer,
 * so both `cat in_accel_x_raw` and buffered clients work.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/hrtimer.h>

#include "hf_manager.h"
#include "hf_sensor_type.h"

#define DRV_NAME "mtk_hf_iio"

struct hfiio_type {
	u8		type;
	const char	*name;
	const char	*label;
	enum iio_chan_type chan_type;
	u8		nchans;
	u8		direction;	/* MTK axis-mapping index, see coordinate_map() */
};

/*
 * `direction` selects an axis transform from hf_manager's coordinate_map()
 * table.  The stock DTBO for this board (PGZ110_16.0.2.400 dtbo entry 5) says
 * gsensor_2/msensor_1 direction = 7, but that entry is the *Android HAL's*
 * input: the HAL applies its own sign convention on top.
 *
 * Measured on the panel:
 * Bench results (user-reported screen orientation):
 *   c[0] identity        -> up/down AND left/right inverted = pure 180 deg
 *   c[7] {-1,-1,-1} swap -> looked like a 90 deg error (Z was negated too)
 *   c[1] { 1, 1, 1} swap -> also wrong
 * A pure 180 deg error is fixed by negating X and Y while keeping Z: that is
 *   c[2] = { sign {-1,-1, 1}, map {0,1,2} }  ->  (x,y,z) => (-x,-y, z)
 * so direction 2 is what this panel wants.
 *
 *   c[1] = { sign { 1, 1, 1}, map {1,0,2} }  ->  (x,y,z) => ( y, x, z)
 *   c[4] = { sign {-1, 1,-1}, map {0,1,2} }  ->  (x,y,z) => (-x, y,-z)
 */
static const struct hfiio_type hfiio_types[] = {
	{  1, "accel", "bmi220-acc",  IIO_ACCEL,     3, 2 },
	{  4, "gyro",  "bmi220-gyro", IIO_ANGL_VEL,  3, 2 },
	{  2, "magn",  "akm09918",    IIO_MAGN,      3, 2 },
	{  5, "als",   "mn78911-als", IIO_LIGHT,     1, 4 },
	{  8, "prox",  "mn78911-ps",  IIO_PROXIMITY, 1, 4 },
};

#define HFIIO_MAX_DEV ARRAY_SIZE(hfiio_types)

/*
 * Runtime-selectable transform index into hf_manager's coordinate_map() table.
 * -1 keeps the per-sensor `direction` from the table; 0..7 overrides it for the
 * 3-axis sensors.  Exposed as a module parameter so the correct panel mapping can
 * be found on the bench without rebuilding.
 */
static int hfiio_direction_override = -1;
module_param_named(direction_override, hfiio_direction_override, int, 0644);
MODULE_PARM_DESC(direction_override,
	"override axis-mapping index 0..7 for 3-axis sensors (-1 = per-sensor default)");

struct hfiio_dev {
	const struct hfiio_type	*type;
	struct iio_dev		*indio;
	struct hfiio_state	*st;
	bool			present;
	bool			enabled;
	bool			got_sample;
	struct iio_trigger	*trig;
	struct hrtimer		timer;
	ktime_t			period;
	spinlock_t		lock;
	s64			ts;
	s32			vals[3];
	int			nvals;
};

struct hfiio_state {
	struct device		*dev;
	struct hf_client	*client;
	struct task_struct	*thread;
	struct hfiio_dev	*devs[HFIIO_MAX_DEV];
	int			ndevs;
	bool			stop;
	u32			sample_delay_us;
};

/* ---------------------------------------------------------------- */
/* channel tables                                                    */
/* ---------------------------------------------------------------- */

#define HFIIO_AXIS_CHAN(_type, _mod, _scan)				\
	{								\
		.type = (_type), .modified = 1, .channel2 = (_mod),	\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_index = (_scan),					\
		.scan_type = {						\
			.sign = 's', .realbits = 32, .storagebits = 32,	\
			.endianness = IIO_LE,				\
		},							\
	}

#define HFIIO_3AXIS(_type)						\
	HFIIO_AXIS_CHAN(_type, IIO_MOD_X, 0),				\
	HFIIO_AXIS_CHAN(_type, IIO_MOD_Y, 1),				\
	HFIIO_AXIS_CHAN(_type, IIO_MOD_Z, 2)

static const struct iio_chan_spec hfiio_accel_chans[] = { HFIIO_3AXIS(IIO_ACCEL) };
static const struct iio_chan_spec hfiio_gyro_chans[]  = { HFIIO_3AXIS(IIO_ANGL_VEL) };
static const struct iio_chan_spec hfiio_magn_chans[]  = { HFIIO_3AXIS(IIO_MAGN) };

#define HFIIO_1AXIS(_type)						\
	{								\
		.type = (_type),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = 0,					\
		.scan_type = {						\
			.sign = 's', .realbits = 32, .storagebits = 32,	\
			.endianness = IIO_LE,				\
		},							\
	}

static const struct iio_chan_spec hfiio_als_chans[]  = { HFIIO_1AXIS(IIO_LIGHT) };
static const struct iio_chan_spec hfiio_prox_chans[] = { HFIIO_1AXIS(IIO_PROXIMITY) };

static const struct iio_chan_spec *hfiio_chans_for(enum iio_chan_type t, int *n)
{
	switch (t) {
	case IIO_ACCEL:
		*n = ARRAY_SIZE(hfiio_accel_chans);
		return hfiio_accel_chans;
	case IIO_ANGL_VEL:
		*n = ARRAY_SIZE(hfiio_gyro_chans);
		return hfiio_gyro_chans;
	case IIO_MAGN:
		*n = ARRAY_SIZE(hfiio_magn_chans);
		return hfiio_magn_chans;
	case IIO_LIGHT:
		*n = ARRAY_SIZE(hfiio_als_chans);
		return hfiio_als_chans;
	case IIO_PROXIMITY:
		*n = ARRAY_SIZE(hfiio_prox_chans);
		return hfiio_prox_chans;
	default:
		*n = 0;
		return NULL;
	}
}

/* ---------------------------------------------------------------- */
/* read_raw                                                          */
/* ---------------------------------------------------------------- */

static int hfiio_set_enable(struct hfiio_dev *d, bool on);

static int hfiio_read_raw(struct iio_dev *indio,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct hfiio_dev *d = iio_priv(indio);
	unsigned long flags;
	int idx, i;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * On-demand power: enable the sensor on first read, then wait
		 * briefly for the SCP to deliver a sample.  Waiting here (instead
		 * of returning -EAGAIN straight away) is what makes consumers
		 * that only read once -- iio-sensor-proxy's light poller -- work
		 * while still leaving unused sensors powered down.
		 */
		if (!READ_ONCE(d->enabled)) {
			hfiio_set_enable(d, true);
			/* up to ~300 ms in 10 ms steps for the first sample */
			for (i = 0; i < 30; i++) {
				if (READ_ONCE(d->got_sample))
					break;
				usleep_range(9000, 11000);
			}
		}
		idx = chan->scan_index;
		spin_lock_irqsave(&d->lock, flags);
		if (idx < 0 || idx >= d->nvals) {
			spin_unlock_irqrestore(&d->lock, flags);
			/* no sample yet: let the caller retry shortly */
			return d->got_sample ? -ENODATA : -EAGAIN;
		}
		d->got_sample = true;
		*val = d->vals[idx];
		spin_unlock_irqrestore(&d->lock, flags);
		return IIO_VAL_INT;

	/*
	 * The SCP reports engineering units per the MTK HAL contract:
	 *   accel  : mg          (1e-3 g)
	 *   gyro   : mdeg/s
	 *   magn   : 0.01 uT
	 * Expose the matching scale so consumers get SI-ish values.
	 */
	case IIO_CHAN_INFO_SCALE:
		switch (d->type->chan_type) {
		case IIO_ACCEL:
			*val = 0; *val2 = 9806;		/* mg -> m/s^2 */
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ANGL_VEL:
			*val = 0; *val2 = 17453;	/* mdeg/s -> rad/s */
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_MAGN:
			*val = 0; *val2 = 1;		/* 0.01 uT -> uT * 100 */
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_LIGHT:
			/*
			 * Official OTA dtbo entry 5 gives als_ratio = 0xc8 (200)
			 * for the mn78911, i.e. lux = raw * 200.
			 */
			*val = 200; *val2 = 0;
			return IIO_VAL_INT;
		default:
			*val = 1; *val2 = 0;
			return IIO_VAL_INT;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 1000000 / max_t(u32, d->st->sample_delay_us, 1);
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info hfiio_info = {
	.read_raw = hfiio_read_raw,
};

/* ---------------------------------------------------------------- */
/* triggered buffer                                                  */
/* ---------------------------------------------------------------- */

static irqreturn_t hfiio_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio = pf->indio_dev;
	struct hfiio_dev *d = iio_priv(indio);
	unsigned long flags;
	s32 vals[3];
	s64 ts;
	int n;

	spin_lock_irqsave(&d->lock, flags);
	ts = d->ts;
	n = d->nvals;
	memcpy(vals, d->vals, sizeof(vals));
	spin_unlock_irqrestore(&d->lock, flags);

	if (n > 0)
		iio_push_to_buffers_with_timestamp(indio, vals, ts);
	iio_trigger_notify_done(indio->trig);
	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------- */
/* enable / disable                                                  */
/* ---------------------------------------------------------------- */

static int hfiio_set_enable(struct hfiio_dev *d, bool on)
{
	struct hf_manager_cmd cmd;
	struct hf_manager_batch *b;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.sensor_type = d->type->type;
	cmd.action = on ? HF_MANAGER_SENSOR_ENABLE : HF_MANAGER_SENSOR_DISABLE;
	cmd.length = sizeof(*b);
	b = (struct hf_manager_batch *)cmd.data;
	b->delay = (int64_t)d->st->sample_delay_us * 1000;
	b->latency = 0;

	ret = hf_client_control_sensor(d->st->client, &cmd);
	if (ret < 0)
		dev_warn(d->st->dev, "control(type=%u on=%d) ret=%d\n",
			 d->type->type, on, ret);
	else
		d->enabled = on;
	return ret;
}

static int hfiio_buffer_postenable(struct iio_dev *indio)
{
	return hfiio_set_enable(iio_priv(indio), true);
}

static int hfiio_buffer_preenable(struct iio_dev *indio)
{
	return hfiio_set_enable(iio_priv(indio), false);
}

static const struct iio_buffer_setup_ops hfiio_buffer_ops = {
	.postenable = hfiio_buffer_postenable,
	.preenable = hfiio_buffer_preenable,
};

/* ---------------------------------------------------------------- */
/* event pump                                                        */
/* ---------------------------------------------------------------- */

static struct hfiio_dev *hfiio_lookup(struct hfiio_state *st, u8 type)
{
	int i;

	for (i = 0; i < st->ndevs; i++)
		if (st->devs[i]->type->type == type)
			return st->devs[i];
	return NULL;
}

static int hfiio_thread(void *arg)
{
	struct hfiio_state *st = arg;
	struct hf_manager_event *ev;
	int i, n;

	ev = kcalloc(16, sizeof(*ev), GFP_KERNEL);
	if (!ev)
		return -ENOMEM;

	while (!kthread_should_stop() && !READ_ONCE(st->stop)) {
		n = hf_client_poll_sensor_timeout(st->client, ev, 16,
						  msecs_to_jiffies(1000));
		if (n == -ETIMEDOUT)
			continue;
		if (n < 0) {
			if (n != -ERESTARTSYS)
				dev_dbg(st->dev, "poll ret=%d\n", n);
			continue;
		}
		for (i = 0; i < n; i++) {
			struct hfiio_dev *d;
			unsigned long flags;
			int k, cnt;

			d = hfiio_lookup(st, ev[i].sensor_type);
			if (!d)
				continue;
			cnt = min_t(int, d->type->nchans, 3);
			/*
			 * Map the SCP's sensor-frame axes into the device frame
			 * using the official MTK transform (exported by
			 * hf_manager) with the board's DTBO direction value.
			 */
			if (cnt == 3 && hfiio_direction_override != -2) {
				u8 dir = d->type->direction;

				if (hfiio_direction_override >= 0 &&
				    hfiio_direction_override < 8)
					dir = (u8)hfiio_direction_override;
				coordinate_map(dir, ev[i].word);
			}
			spin_lock_irqsave(&d->lock, flags);
			for (k = 0; k < cnt; k++)
				d->vals[k] = ev[i].word[k];
			d->nvals = cnt;
			d->ts = ev[i].timestamp;
			spin_unlock_irqrestore(&d->lock, flags);
		}
	}
	kfree(ev);
	return 0;
}

/* ---------------------------------------------------------------- */
/* timer trigger (this kernel has no CONFIG_IIO_SW_TRIGGER)           */
/* ---------------------------------------------------------------- */

static enum hrtimer_restart hfiio_timer_fn(struct hrtimer *t)
{
	struct hfiio_dev *d = container_of(t, struct hfiio_dev, timer);

	if (READ_ONCE(d->enabled) && d->indio->active_scan_mask &&
	    !bitmap_empty(d->indio->active_scan_mask, d->indio->masklength))
		iio_trigger_poll(d->trig);
	hrtimer_forward_now(t, d->period);
	return HRTIMER_RESTART;
}

static int hfiio_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct hfiio_dev *d = iio_trigger_get_drvdata(trig);

	if (state)
		hrtimer_start(&d->timer, d->period, HRTIMER_MODE_REL_HARD);
	else
		hrtimer_cancel(&d->timer);
	return 0;
}

static const struct iio_trigger_ops hfiio_trigger_ops = {
	.set_trigger_state = hfiio_trig_set_state,
};

static int hfiio_setup_trigger(struct iio_dev *indio, struct hfiio_dev *d,
			       struct device *parent)
{
	struct iio_trigger *trig;
	int ret;

	trig = devm_iio_trigger_alloc(parent, "%s-dev", indio->name);
	if (!trig)
		return -ENOMEM;

	trig->ops = &hfiio_trigger_ops;
	iio_trigger_set_drvdata(trig, d);
	d->trig = trig;

	hrtimer_setup(&d->timer, hfiio_timer_fn, CLOCK_MONOTONIC,
		      HRTIMER_MODE_REL_HARD);
	d->period = ns_to_ktime((u64)d->st->sample_delay_us * 1000ULL);

	ret = devm_iio_trigger_register(parent, trig);
	if (ret)
		return ret;

	/*
	 * Bind the trigger as immutable: this is the kernel-blessed way to hand a
	 * consumer a fixed trigger.  It sets indio->trig, marks the attribute
	 * read-only (so iio-sensor-proxy does not try to write it and hit EPERM)
	 * and makes trigger/current_trigger already name our trigger at the
	 * moment the device appears.
	 */
	ret = iio_trigger_set_immutable(indio, trig);
	if (ret)
		return ret;
	return 0;
}

/* ---------------------------------------------------------------- */
/* platform driver                                                   */
/* ---------------------------------------------------------------- */

static int hfiio_probe(struct platform_device *pdev)
{
	struct hfiio_state *st;
	int i, ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;
	st->dev = &pdev->dev;
	st->sample_delay_us = 100000;	/* 10 Hz */

	st->client = hf_client_create();
	if (IS_ERR_OR_NULL(st->client)) {
		ret = PTR_ERR(st->client) ?: -ENODEV;
		dev_err(&pdev->dev, "hf_client_create failed: %d\n", ret);
		return ret;
	}

	for (i = 0; i < HFIIO_MAX_DEV; i++) {
		const struct hfiio_type *t = &hfiio_types[i];
		struct iio_dev *indio;
		struct hfiio_dev *d;
		const struct iio_chan_spec *chans;
		int nchans;

		if (hf_client_find_sensor(st->client, t->type) < 0) {
			dev_info(&pdev->dev, "type %u (%s) not present\n",
				 t->type, t->name);
			continue;
		}
		chans = hfiio_chans_for(t->chan_type, &nchans);
		if (!chans)
			continue;

		indio = devm_iio_device_alloc(&pdev->dev, sizeof(*d));
		if (!indio)
			continue;

		d = iio_priv(indio);
		d->type = t;
		d->indio = indio;
		d->st = st;
		d->present = true;
		spin_lock_init(&d->lock);

		indio->name = t->name;
		indio->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
		indio->info = &hfiio_info;
		indio->channels = chans;
		indio->num_channels = nchans;

		ret = devm_iio_triggered_buffer_setup(&pdev->dev, indio, NULL,
						      hfiio_trigger_handler,
						      &hfiio_buffer_ops);
		if (ret) {
			dev_err(&pdev->dev, "buffer setup %s ret=%d\n",
				t->name, ret);
			continue;
		}
		ret = hfiio_setup_trigger(indio, d, &pdev->dev);
		if (ret) {
			dev_err(&pdev->dev, "trigger setup %s ret=%d\n",
				t->name, ret);
			continue;
		}
		ret = devm_iio_device_register(&pdev->dev, indio);
		if (ret) {
			dev_err(&pdev->dev, "register %s ret=%d\n", t->name, ret);
			continue;
		}
		dev_info(&pdev->dev, "registered IIO '%s' (type %u, %s)\n",
			 t->name, t->type, t->label);
		st->devs[st->ndevs++] = d;
	}

	if (st->ndevs == 0) {
		dev_err(&pdev->dev, "no sensors registered\n");
		hf_client_destroy(st->client);
		return -ENODEV;
	}

	st->thread = kthread_run(hfiio_thread, st, DRV_NAME "/pump");
	if (IS_ERR(st->thread)) {
		ret = PTR_ERR(st->thread);
		st->thread = NULL;
		dev_warn(&pdev->dev, "pump thread failed: %d\n", ret);
	}

	/*
	 * On-demand power: sensors are enabled the first time a consumer reads
	 * them, so the SCP is not kept awake for sensors nobody uses.  The
	 * first read kicks the sensor and may return -EAGAIN once; see
	 * hfiio_read_raw().
	 */

	platform_set_drvdata(pdev, st);
	dev_info(&pdev->dev, "ready: %d sensors, pump=%s\n", st->ndevs,
		 st->thread ? "yes" : "no");
	return 0;
}

/* .remove returns void; devm releases the IIO devices automatically. */
static void hfiio_remove(struct platform_device *pdev)
{
	struct hfiio_state *st = platform_get_drvdata(pdev);
	int i;

	if (!st)
		return;

	WRITE_ONCE(st->stop, true);
	if (st->thread)
		kthread_stop(st->thread);

	for (i = st->ndevs - 1; i >= 0; i--) {
		hrtimer_cancel(&st->devs[i]->timer);
		hfiio_set_enable(st->devs[i], false);
		/*
		 * Drop the reference iio_trigger_set_immutable() took; the devm
		 * action releases the alloc/free pair, so without this the module
		 * refcount would stay elevated and rmmod would fail.
		 */
		if (st->devs[i]->indio->trig) {
			iio_trigger_put(st->devs[i]->indio->trig);
			st->devs[i]->indio->trig = NULL;
		}
	}

	hf_client_destroy(st->client);
	dev_info(&pdev->dev, "removed\n");
}

static const struct platform_device_id hfiio_ids[] = {
	{ .name = DRV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(platform, hfiio_ids);

static struct platform_driver hfiio_driver = {
	.probe		= hfiio_probe,
	.remove		= hfiio_remove,
	.id_table	= hfiio_ids,
	.driver = {
		.name	= DRV_NAME,
	},
};

static struct platform_device *hfiio_pdev;

static int __init hfiio_init(void)
{
	int ret;

	hfiio_pdev = platform_device_register_simple(DRV_NAME, -1, NULL, 0);
	if (IS_ERR(hfiio_pdev))
		return PTR_ERR(hfiio_pdev);

	ret = platform_driver_register(&hfiio_driver);
	if (ret) {
		platform_device_unregister(hfiio_pdev);
		hfiio_pdev = NULL;
		return ret;
	}
	return 0;
}

static void __exit hfiio_exit(void)
{
	platform_driver_unregister(&hfiio_driver);
	if (hfiio_pdev)
		platform_device_unregister(hfiio_pdev);
}

module_init(hfiio_init);
module_exit(hfiio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek SCP sensors as standard Linux IIO devices");
