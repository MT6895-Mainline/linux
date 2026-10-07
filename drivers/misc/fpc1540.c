// SPDX-License-Identifier: GPL-2.0
/*
 * FPC1540/FPC15xx fingerprint sensor SPI driver (xaga, MT6895).
 *
 * The sensor sits on the hardware SPI3 controller behind a 1.8 V rail, a reset
 * GPIO and an IRQ GPIO.  This driver owns all of it:
 *
 *   - power (vdd / optional vddio), hardware reset, HWID identification
 *   - the FPC register/command protocol over SPI
 *   - image capture (19712-byte raw stream) and the native 2-bit noise-shaped
 *     decode into a 112x88 8-bit image
 *   - finger detect from the IRQ GPIO
 *
 * Userspace interface: /dev/fpc1540 (misc device, see <linux/fpc1540.h>):
 *   read()        decoded 112x88 image
 *   poll()        EPOLLIN on finger detect or a new image
 *   ioctl()       GET_HWID / POWER / RESET / READ_REG / WRITE_REG / SEND_CMD /
 *                 CAPTURE / WAIT_FINGER / FINGER_STATE
 * plus sysfs attributes "hwid", "finger" and "power" on the SPI device.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/fpc1540.h>

#define FPC1540_DRV_NAME	"fpc1540"

/* FPC102x register map */
#define FPC_REG_STATUS		0x14
#define FPC_REG_IRQ		0x18
#define FPC_REG_IRQ_CLEAR	0x1c
#define FPC_REG_ERR_CLEAR	0x38
#define FPC_REG_IMG_CAPT_SIZE	0x54
#define FPC_REG_IMAGE_SETUP	0x5c
#define FPC_REG_IMG_RD		0x64
#define FPC_REG_SAMPLE_PX_DLY	0x68
#define FPC_REG_TST_COL_PATTERN	0x78
#define FPC_REG_FINGER_DRIVE_CONF 0x8c
#define FPC_REG_FINGER_DRIVE_DLY 0x90
#define FPC_REG_HWID		0xfc

#define FPC_CMD_FINGER_PRESENT_QUERY	0x20
#define FPC_CMD_WAIT_FOR_FINGER		0x24
#define FPC_CMD_ACTIVATE_IDLE_MODE	0x34
#define FPC_CMD_CAPTURE_IMAGE		0xc0
#define FPC_CMD_READ_IMAGE		0xc4
#define FPC_CMD_SOFT_RESET		0xf8

#define FPC_IRQ_FINGER_DOWN	BIT(0)
#define FPC_IRQ_ERROR		BIT(2)
#define FPC_IRQ_FIFO_NEW_DATA	BIT(5)
#define FPC_IRQ_COMMAND_DONE	BIT(7)

#define FPC_PXL_BIAS_CTRL	0x0f00

#define FPC1540_HWID_MASK	0xff00
#define FPC1540_HWID_1540	0x1800
#define FPC1540_HWID_1542	0x1c00
#define FPC1540_HWID_1542_L11	0x1e00
#define FPC1540_HWID_1542SA	0x1f00

#define FPC1540_SPI_DELAY_US	5

struct fpc1540 {
	struct device		*dev;
	struct spi_device	*spi;

	struct regulator	*vdd;
	struct regulator	*vddio;
	struct gpio_desc	*reset;
	struct gpio_desc	*irq;

	struct mutex		lock;		/* serializes SPI and ioctls */
	wait_queue_head_t	finger_wq;
	int			irq_num;
	volatile int		finger;

	u16			hwid;
	bool			powered;
	bool			configured;
	bool			image_valid;

	u8			*raw;		/* FPC1540_RAW_MAX */
	unsigned int		raw_len;
	u8			*image;		/* FPC1540_IMG_BYTES */
	s32			*scratch;
	s32			*tmp;

	struct miscdevice	misc;
};

/* ------------------------------------------------------------------------- */
/* SPI primitives                                                            */
/* ------------------------------------------------------------------------- */

static int fpc1540_xfer(struct fpc1540 *f, struct spi_transfer *t, int n)
{
	struct spi_message m;

	spi_message_init(&m);
	while (n--)
		spi_message_add_tail(t++, &m);

	return spi_sync(f->spi, &m);
}

/* Raw SPI transfer used by the TEE trustlet emulator: send @tx (tx_len bytes),
 * then clock @rx (rx_len bytes) with chip select held, exactly like the
 * Microtrust platform ut_pf_spi_send_and_receive() primitive. */
static int fpc1540_raw_transfer(struct fpc1540 *f, const u8 *tx, u32 tx_len,
				u8 *rx, u32 rx_len)
{
	struct spi_transfer t[2];
	int n = 0, ret;

	memset(t, 0, sizeof(t));
	if (tx_len) {
		t[n].tx_buf = tx;
		t[n].len = tx_len;
		t[n].delay.value = FPC1540_SPI_DELAY_US;
		t[n].delay.unit = SPI_DELAY_UNIT_USECS;
		n++;
	}
	if (rx_len) {
		t[n].rx_buf = rx;
		t[n].len = rx_len;
		n++;
	}
	if (!n)
		return 0;
	mutex_lock(&f->lock);
	ret = fpc1540_xfer(f, t, n);
	mutex_unlock(&f->lock);
	return ret;
}

static int fpc1540_read_reg(struct fpc1540 *f, u8 reg, bool dummy, u8 *out,
			    u8 len)
{
	u8 addr[2] = { reg, 0 };
	struct spi_transfer t[2] = {
		{
			.tx_buf = addr,
			.len = dummy ? 2 : 1,
			.delay = { FPC1540_SPI_DELAY_US, SPI_DELAY_UNIT_USECS },
		},
		{
			.rx_buf = out,
			.len = len,
		},
	};

	return fpc1540_xfer(f, t, 2);
}

static int fpc1540_write_reg(struct fpc1540 *f, u8 reg, const u8 *data,
			     u8 len)
{
	u8 addr = reg;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &addr,
			.len = 1,
			.delay = { FPC1540_SPI_DELAY_US, SPI_DELAY_UNIT_USECS },
		},
		{
			.tx_buf = data,
			.len = len,
			.delay = { FPC1540_SPI_DELAY_US, SPI_DELAY_UNIT_USECS },
		},
	};

	return fpc1540_xfer(f, t, 2);
}

static int fpc1540_write_be(struct fpc1540 *f, u8 reg, u32 val, int len)
{
	u8 b[4];
	int i;

	for (i = 0; i < len; i++)
		b[i] = (val >> (8 * (len - 1 - i))) & 0xff;

	return fpc1540_write_reg(f, reg, b, len);
}

static int fpc1540_cmd(struct fpc1540 *f, u8 cmd)
{
	struct spi_transfer t = {
		.tx_buf = &cmd,
		.len = 1,
		.delay = { FPC1540_SPI_DELAY_US, SPI_DELAY_UNIT_USECS },
	};

	return fpc1540_xfer(f, &t, 1);
}

static int fpc1540_read_irq(struct fpc1540 *f, u8 *irq)
{
	return fpc1540_read_reg(f, FPC_REG_IRQ, false, irq, 1);
}

/* Reading IRQ_CLEAR returns the pending bits and deasserts the sensor's
 * interrupt line.  FPC parts assert the IRQ line while any bit is pending, so
 * a stale REBOOT (0xff) left over from a soft reset will hold the line high
 * forever unless it is read through this register. */
static int fpc1540_read_irq_clear(struct fpc1540 *f, u8 *irq)
{
	return fpc1540_read_reg(f, FPC_REG_IRQ_CLEAR, false, irq, 1);
}

static void fpc1540_note_finger(struct fpc1540 *f, int state)
{
	if (f->finger == state)
		return;
	f->finger = state;
	sysfs_notify(&f->dev->kobj, NULL, "finger");
	wake_up_interruptible_all(&f->finger_wq);
}

static int fpc1540_poll_irq(struct fpc1540 *f, u8 mask, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	u8 irq = 0;
	int ret;

	for (;;) {
		ret = fpc1540_read_irq(f, &irq);
		if (ret)
			return ret;
		if (irq & mask)
			return irq;
		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(1000, 2000);
	}
}

/* ------------------------------------------------------------------------- */
/* power / reset                                                             */
/* ------------------------------------------------------------------------- */

static void fpc1540_reset_pulse(struct fpc1540 *f)
{
	if (!f->reset)
		return;

	/* Reference sequence: high 100us, low 1000us, high 1250us. */
	gpiod_set_value_cansleep(f->reset, 1);
	udelay(100);
	gpiod_set_value_cansleep(f->reset, 0);
	udelay(1000);
	gpiod_set_value_cansleep(f->reset, 1);
	udelay(1250);
}

static int fpc1540_power_on(struct fpc1540 *f)
{
	int ret;

	if (f->powered)
		return 0;

	if (f->reset)
		gpiod_set_value_cansleep(f->reset, 0);

	if (f->vdd) {
		ret = regulator_enable(f->vdd);
		if (ret)
			return ret;
	}
	if (f->vddio) {
		ret = regulator_enable(f->vddio);
		if (ret) {
			if (f->vdd)
				regulator_disable(f->vdd);
			return ret;
		}
	}

	msleep(20);
	f->powered = true;
	fpc1540_reset_pulse(f);
	msleep(10);

	return 0;
}

static int fpc1540_power_off(struct fpc1540 *f)
{
	if (f->reset)
		gpiod_set_value_cansleep(f->reset, 0);

	if (f->vddio)
		regulator_disable(f->vddio);
	if (f->vdd)
		regulator_disable(f->vdd);

	f->powered = false;
	f->configured = false;
	msleep(20);

	return 0;
}

/* ------------------------------------------------------------------------- */
/* identification                                                            */
/* ------------------------------------------------------------------------- */

static bool fpc1540_hwid_is_fpc(u16 hwid)
{
	switch (hwid & FPC1540_HWID_MASK) {
	case FPC1540_HWID_1540:
	case FPC1540_HWID_1542:
	case FPC1540_HWID_1542_L11:
	case FPC1540_HWID_1542SA:
		return true;
	default:
		return false;
	}
}

/* ------------------------------------------------------------------------- */
/* analog front-end setup                                                    */
/* ------------------------------------------------------------------------- */

static void fpc1540_apply_setup(struct fpc1540 *f)
{
	/* FPC1150-derived base table with the 0x1811 corrections. */
	fpc1540_write_be(f, 0x90, 0x09, 1);
	fpc1540_write_be(f, FPC_REG_SAMPLE_PX_DLY, 0x14141414, 4);
	fpc1540_write_be(f, FPC_REG_SAMPLE_PX_DLY, 0x08080808, 4);
	fpc1540_write_be(f, FPC_REG_FINGER_DRIVE_CONF, 0x12, 1);
	fpc1540_write_be(f, 0xa0, 0x0a02, 2);
	fpc1540_write_be(f, 0xa8, 0x0b | FPC_PXL_BIAS_CTRL, 2);
	fpc1540_write_be(f, FPC_REG_IMAGE_SETUP, 0x0b, 1);
	fpc1540_write_be(f, FPC_REG_IMG_RD, 0x0e, 1);
	fpc1540_write_be(f, FPC_REG_TST_COL_PATTERN, 0x0000, 2);
	fpc1540_write_be(f, 0xd8, 0x015050, 4);	/* FNGR_DET_THRES */
	fpc1540_write_be(f, 0xdc, 0x190100ff, 4);	/* FNGR_DET_CNTR */
	fpc1540_write_be(f, FPC_REG_IMG_CAPT_SIZE, 0x00580070, 4);

	/* Analog front-end read-modify-write sequence from the vendor TEE. */
	{
		u8 b[16];

		fpc1540_read_reg(f, 0x4c, false, b, 10);
		b[9] &= ~0x01;
		fpc1540_write_reg(f, 0x4c, b, 10);

		fpc1540_read_reg(f, 0x5c, false, b, 1);
		b[0] = (b[0] & 0xf0) | 0x02;
		fpc1540_write_reg(f, 0x5c, b, 1);

		fpc1540_read_reg(f, 0x88, false, b, 5);
		b[1] |= 0x08;
		fpc1540_write_reg(f, 0x88, b, 5);

		fpc1540_read_reg(f, 0x8c, false, b, 3);
		b[2] &= 0x3f;
		fpc1540_write_reg(f, 0x8c, b, 3);

		fpc1540_read_reg(f, 0x6e, false, b, 1);
		b[0] = (b[0] & ~0x03) | 0x0c;
		fpc1540_write_reg(f, 0x6e, b, 1);

		fpc1540_read_reg(f, 0x9c, false, b, 7);
		b[3] |= 0x04;
		fpc1540_write_reg(f, 0x9c, b, 7);

		fpc1540_read_reg(f, 0xa0, false, b, 2);
		b[0] = 0x0f;
		b[1] &= 0xf0;
		fpc1540_write_reg(f, 0xa0, b, 2);

		fpc1540_read_reg(f, 0xa2, false, b, 3);
		b[0] &= ~0x20;
		fpc1540_write_reg(f, 0xa2, b, 3);

		fpc1540_read_reg(f, 0xa8, false, b, 14);
		b[10] |= 0x10;
		b[11] = 0xa0;
		b[13] = (b[13] & 0x8f) | 0x30;
		fpc1540_write_reg(f, 0xa8, b, 14);

		fpc1540_read_reg(f, 0x8c, false, b, 3);
		fpc1540_write_reg(f, 0x8c, b, 3);
	}
}

/* ------------------------------------------------------------------------- */
/* image decode: 2-bit noise-shaped lane -> 8-bit image                      */
/* ------------------------------------------------------------------------- */

/* Separable 7-tap binomial low-pass (sigma ~1.5).  This is the filter that
 * turns the sensor's 2-bit spatial dither into a smooth 8-bit image; a 5-tap
 * kernel is too weak and the dither still shows through. */
static void fpc1540_blur(s32 *a, s32 *tmp, int w, int h)
{
	static const int k[7] = { 1, 6, 15, 20, 15, 6, 1 };
	int r, c, i;
	s32 s;

	/* horizontal */
	for (r = 0; r < h; r++) {
		for (c = 0; c < w; c++) {
			s = 0;
			for (i = -3; i <= 3; i++) {
				int cc = c + i;

				if (cc < 0)
					cc = 0;
				if (cc >= w)
					cc = w - 1;
				s += k[i + 3] * a[r * w + cc];
			}
			tmp[r * w + c] = s >> 6;
		}
	}

	/* vertical */
	for (c = 0; c < w; c++) {
		for (r = 0; r < h; r++) {
			s = 0;
			for (i = -3; i <= 3; i++) {
				int rr = r + i;

				if (rr < 0)
					rr = 0;
				if (rr >= h)
					rr = h - 1;
				s += k[i + 3] * tmp[rr * w + c];
			}
			a[r * w + c] = s >> 6;
		}
	}
}

static void fpc1540_decode(struct fpc1540 *f)
{
	const int w = FPC1540_IMG_W;
	const int h = FPC1540_IMG_H;
	const int n = FPC1540_IMG_BYTES;
	s32 *a = f->scratch;
	s64 sum = 0, sumsq = 0, var;
	s32 rowmean, mean, sd;
	int i, r, c;

	for (i = 0; i < n; i++)
		a[i] = f->raw[1 + 2 * i];

	/* Only the per-row baseline is removed: subtracting the per-column mean
	 * as well strips the vertical ridge structure and leaves pure dither. */
	for (r = 0; r < h; r++) {
		rowmean = 0;
		for (c = 0; c < w; c++)
			rowmean += a[r * w + c];
		rowmean /= w;
		for (c = 0; c < w; c++)
			a[r * w + c] -= rowmean;
	}

	fpc1540_blur(a, f->tmp, w, h);

	for (i = 0; i < n; i++) {
		sum += a[i];
		sumsq += (s64)a[i] * a[i];
	}
	mean = div_s64(sum, n);
	var = sumsq / n - (s64)mean * mean;
	if (var < 1)
		var = 1;
	sd = int_sqrt64(var);
	if (sd < 1)
		sd = 1;

	for (i = 0; i < n; i++) {
		/* +-2 sigma maps to the 8-bit range. */
		s32 v = ((a[i] - mean) * 6380) / (sd * 100) + 128;

		if (v < 0)
			v = 0;
		if (v > 255)
			v = 255;
		f->image[i] = (u8)v;
	}
}

/* ------------------------------------------------------------------------- */
/* capture                                                                   */
/* ------------------------------------------------------------------------- */

/* One-time sensor bring-up: soft reset and AFE programming.  Doing this once
 * (not per capture) matches the userspace capture path and keeps single frames
 * usable; re-running the reset before every frame leaves the first frame
 * noticeably noisier. */
static void fpc1540_configure(struct fpc1540 *f)
{
	u8 irq = 0;

	fpc1540_read_irq_clear(f, &irq);
	fpc1540_cmd(f, FPC_CMD_SOFT_RESET);
	msleep(5);

	/* The soft reset raises REBOOT (0xff) on the IRQ line.  Clear it, or the
	 * line stays high and the GPIO-based finger detect reports a stuck
	 * "finger present". */
	fpc1540_read_irq_clear(f, &irq);

	fpc1540_apply_setup(f);

	fpc1540_write_be(f, FPC_REG_IMAGE_SETUP, 0x0a, 1);
	fpc1540_write_be(f, FPC_REG_IMG_RD, 0x0c, 1);
	fpc1540_write_be(f, FPC_REG_IMAGE_SETUP, 0x0b, 1);
	fpc1540_write_be(f, FPC_REG_IMG_RD, 0x0e, 1);

	fpc1540_read_irq_clear(f, &irq);
	fpc1540_note_finger(f, 0);
}

static int fpc1540_capture(struct fpc1540 *f)
{
	u8 cmd[2] = { FPC_CMD_READ_IMAGE, 0 };
	u8 irq = 0;
	int ret;

	if (!f->powered) {
		ret = fpc1540_power_on(f);
		if (ret)
			return ret;
	}

	mutex_lock(&f->lock);

	if (!f->configured) {
		fpc1540_configure(f);
		f->configured = true;
	}

	fpc1540_read_irq(f, &irq);
	fpc1540_cmd(f, FPC_CMD_ACTIVATE_IDLE_MODE);
	usleep_range(1000, 2000);
	fpc1540_cmd(f, FPC_CMD_CAPTURE_IMAGE);

	ret = fpc1540_poll_irq(f, FPC_IRQ_FIFO_NEW_DATA | FPC_IRQ_ERROR, 1000);
	if (ret < 0) {
		dev_err(f->dev, "capture: no IRQ\n");
		goto out;
	}

	fpc1540_read_irq(f, &irq);
	if (irq & FPC_IRQ_ERROR) {
		u8 err = 0;

		fpc1540_read_reg(f, FPC_REG_ERR_CLEAR, false, &err, 1);
		dev_err(f->dev, "capture: sensor error irq=0x%02x err=0x%02x\n",
			irq, err);
		ret = -EIO;
		goto out;
	}

	{
		struct spi_transfer t[2] = {
			{
				.tx_buf = cmd,
				.len = 2,
				.delay = { FPC1540_SPI_DELAY_US,
					   SPI_DELAY_UNIT_USECS },
			},
			{
				.rx_buf = f->raw,
				.len = f->raw_len,
			},
		};

		ret = fpc1540_xfer(f, t, 2);
	}
	if (ret)
		goto out;

	fpc1540_read_irq(f, &irq);
	if (irq & FPC_IRQ_ERROR) {
		dev_err(f->dev, "capture: fetch error irq=0x%02x\n", irq);
		ret = -EIO;
		goto out;
	}

	if (f->raw_len == FPC1540_RAW_FRAME) {
		fpc1540_decode(f);
		f->image_valid = true;
	}
	ret = 0;

out:
	/* Drop any latched IRQ (REBOOT/FIFO) so the interrupt line deasserts
	 * instead of pinning the GPIO-based finger detect high. */
	fpc1540_read_irq_clear(f, &irq);
	if (f->irq)
		fpc1540_note_finger(f, gpiod_get_value_cansleep(f->irq) ? 1 : 0);
	mutex_unlock(&f->lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* finger detect                                                             */
/* ------------------------------------------------------------------------- */

static irqreturn_t fpc1540_irq_thread(int irq, void *data)
{
	struct fpc1540 *f = data;
	int level;

	if (!f->irq)
		return IRQ_HANDLED;

	level = gpiod_get_value_cansleep(f->irq);
	fpc1540_note_finger(f, level ? 1 : 0);

	return IRQ_HANDLED;
}

static int fpc1540_wait_finger(struct fpc1540 *f, unsigned int timeout_ms,
			       int *state)
{
	long ret;

	if (timeout_ms == 0)
		ret = wait_event_interruptible(f->finger_wq, f->finger);
	else
		ret = wait_event_interruptible_timeout(f->finger_wq, f->finger,
						       msecs_to_jiffies(timeout_ms));
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	*state = f->finger;
	return 0;
}

/* ------------------------------------------------------------------------- */
/* misc device                                                               */
/* ------------------------------------------------------------------------- */

static int fpc1540_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;

	file->private_data = container_of(misc, struct fpc1540, misc);
	return 0;
}

static ssize_t fpc1540_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct fpc1540 *f = file->private_data;
	ssize_t ret = 0;

	if (mutex_lock_interruptible(&f->lock))
		return -ERESTARTSYS;

	if (!f->image_valid) {
		ret = -EAGAIN;
		goto out;
	}
	if (*ppos >= FPC1540_IMG_BYTES)
		goto out;
	if (count > FPC1540_IMG_BYTES - *ppos)
		count = FPC1540_IMG_BYTES - *ppos;

	if (copy_to_user(buf, f->image + *ppos, count)) {
		ret = -EFAULT;
		goto out;
	}
	*ppos += count;
	if (*ppos >= FPC1540_IMG_BYTES)
		f->image_valid = false;
	ret = count;

out:
	mutex_unlock(&f->lock);
	return ret;
}

static __poll_t fpc1540_poll(struct file *file, poll_table *wait)
{
	struct fpc1540 *f = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &f->finger_wq, wait);
	if (f->finger || f->image_valid)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static long fpc1540_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct fpc1540 *f = file->private_data;
	void __user *uarg = (void __user *)arg;
	int ret = 0;

	if (_IOC_TYPE(cmd) != FPC1540_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > FPC1540_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case FPC1540_IOC_GET_HWID:
		return put_user(f->hwid, (__u16 __user *)uarg);

	case FPC1540_IOC_POWER: {
		__u32 mode;

		if (get_user(mode, (__u32 __user *)uarg))
			return -EFAULT;
		mutex_lock(&f->lock);
		switch (mode) {
		case FPC1540_POWER_OFF:
			ret = fpc1540_power_off(f);
			break;
		case FPC1540_POWER_ON:
			ret = fpc1540_power_on(f);
			break;
		case FPC1540_POWER_CYCLE:
			fpc1540_power_off(f);
			msleep(20);
			ret = fpc1540_power_on(f);
			break;
		default:
			ret = -EINVAL;
		}
		mutex_unlock(&f->lock);
		return ret;
	}

	case FPC1540_IOC_RESET:
		if (!f->powered)
			return -ENODEV;
		mutex_lock(&f->lock);
		fpc1540_reset_pulse(f);
		mutex_unlock(&f->lock);
		return 0;

	case FPC1540_IOC_READ_REG:
	case FPC1540_IOC_WRITE_REG: {
		struct fpc1540_reg_io r;

		if (copy_from_user(&r, uarg, sizeof(r)))
			return -EFAULT;
		if (r.len == 0 || r.len > sizeof(r.data))
			return -EINVAL;
		mutex_lock(&f->lock);
		if (cmd == FPC1540_IOC_READ_REG) {
			ret = fpc1540_read_reg(f, r.reg, r.dummy, r.data, r.len);
			if (!ret && copy_to_user(uarg, &r, sizeof(r)))
				ret = -EFAULT;
		} else {
			ret = fpc1540_write_reg(f, r.reg, r.data, r.len);
		}
		mutex_unlock(&f->lock);
		return ret;
	}

	case FPC1540_IOC_SEND_CMD: {
		struct fpc1540_cmd_io c;

		if (copy_from_user(&c, uarg, sizeof(c)))
			return -EFAULT;
		mutex_lock(&f->lock);
		ret = fpc1540_cmd(f, c.cmd);
		if (!ret && c.irq_mask) {
			ret = fpc1540_poll_irq(f, c.irq_mask, c.timeout_ms);
			if (ret >= 0) {
				c.status = ret;
				ret = 0;
			} else {
				u8 irq;

				fpc1540_read_irq(f, &irq);
				c.status = irq;
			}
		}
		mutex_unlock(&f->lock);
		if (!ret && copy_to_user(uarg, &c, sizeof(c)))
			ret = -EFAULT;
		return ret;
	}

	case FPC1540_IOC_CAPTURE: {
		struct fpc1540_image_io img = {
			.width = FPC1540_IMG_W,
			.height = FPC1540_IMG_H,
			.bytes = FPC1540_IMG_BYTES,
		};

		ret = fpc1540_capture(f);
		if (ret)
			return ret;
		/* A new capture restarts the image stream: reset the read offset so
		 * the next read() returns this frame (noop_llseek ignores seeks). */
		file->f_pos = 0;
		if (copy_to_user(uarg, &img, sizeof(img)))
			return -EFAULT;
		return 0;
	}

	case FPC1540_IOC_WAIT_FINGER: {
		__u32 timeout;
		int state = 0;

		if (get_user(timeout, (__u32 __user *)uarg))
			return -EFAULT;
		ret = fpc1540_wait_finger(f, timeout, &state);
		if (ret)
			return ret;
		return put_user((__u32)state, (__u32 __user *)uarg);
	}

	case FPC1540_IOC_FINGER_STATE:
		return put_user((__u32)f->finger, (__u32 __user *)uarg);

	case FPC1540_IOC_GET_RAW:
		if (copy_to_user(uarg, f->raw, f->raw_len))
			return -EFAULT;
		return 0;

	case FPC1540_IOC_RAW_XFER: {
		struct fpc1540_raw_xfer x;
		u8 *buf;
		u32 total;

		if (copy_from_user(&x, uarg, sizeof(x)))
			return -EFAULT;
		if (x.tx_len > FPC1540_RAW_MAX || x.rx_len > FPC1540_RAW_MAX)
			return -EINVAL;
		total = x.tx_len + x.rx_len;
		if (!total)
			return 0;
		buf = kzalloc(total, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
		if (x.tx_len &&
		    copy_from_user(buf, (void __user *)(uintptr_t)x.tx, x.tx_len)) {
			kfree(buf);
			return -EFAULT;
		}
		ret = fpc1540_raw_transfer(f, buf, x.tx_len,
					   buf + x.tx_len, x.rx_len);
		if (!ret && x.rx_len &&
		    copy_to_user((void __user *)(uintptr_t)x.rx,
				 buf + x.tx_len, x.rx_len))
			ret = -EFAULT;
		kfree(buf);
		return ret;
	}
	}

	return -ENOTTY;
}

static const struct file_operations fpc1540_fops = {
	.owner		= THIS_MODULE,
	.open		= fpc1540_open,
	.read		= fpc1540_read,
	.poll		= fpc1540_poll,
	.unlocked_ioctl	= fpc1540_ioctl,
	.llseek		= noop_llseek,
};

/* ------------------------------------------------------------------------- */
/* sysfs                                                                     */
/* ------------------------------------------------------------------------- */

static ssize_t hwid_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct fpc1540 *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "0x%04x\n", f->hwid);
}
static DEVICE_ATTR_RO(hwid);

static ssize_t finger_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct fpc1540 *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", f->finger);
}
static DEVICE_ATTR_RO(finger);

static ssize_t powered_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct fpc1540 *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", f->powered ? 1 : 0);
}

static ssize_t powered_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct fpc1540 *f = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&f->lock);
	ret = on ? fpc1540_power_on(f) : fpc1540_power_off(f);
	mutex_unlock(&f->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(powered);

/* Raw read length in bytes; lets userspace probe the readout framing. */
static ssize_t raw_len_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct fpc1540 *f = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", f->raw_len);
}

static ssize_t raw_len_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct fpc1540 *f = dev_get_drvdata(dev);
	unsigned int v;

	if (kstrtouint(buf, 0, &v))
		return -EINVAL;
	if (v == 0 || v > FPC1540_RAW_MAX)
		return -EINVAL;
	f->raw_len = v;

	return count;
}
static DEVICE_ATTR_RW(raw_len);

static struct attribute *fpc1540_attrs[] = {
	&dev_attr_hwid.attr,
	&dev_attr_finger.attr,
	&dev_attr_powered.attr,
	&dev_attr_raw_len.attr,
	NULL,
};

static const struct attribute_group fpc1540_group = {
	.attrs = fpc1540_attrs,
};

/* ------------------------------------------------------------------------- */
/* probe / remove                                                            */
/* ------------------------------------------------------------------------- */

static int fpc1540_request_irq(struct fpc1540 *f)
{
	int irq;

	if (!f->irq)
		return 0;

	irq = gpiod_to_irq(f->irq);
	if (irq < 0)
		return irq;
	f->irq_num = irq;

	return devm_request_threaded_irq(f->dev, irq, NULL, fpc1540_irq_thread,
					 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
					 IRQF_ONESHOT,
					 dev_name(f->dev), f);
}

static int fpc1540_probe(struct spi_device *spi)
{
	struct fpc1540 *f;
	u8 id[2];
	int ret;

	f = devm_kzalloc(&spi->dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;

	f->dev = &spi->dev;
	f->spi = spi;
	spi_set_drvdata(spi, f);
	dev_set_drvdata(f->dev, f);

	mutex_init(&f->lock);
	init_waitqueue_head(&f->finger_wq);

	f->raw = devm_kzalloc(f->dev, FPC1540_RAW_MAX, GFP_KERNEL);
	f->raw_len = FPC1540_RAW_FRAME;
	f->image = devm_kzalloc(f->dev, FPC1540_IMG_BYTES, GFP_KERNEL);
	f->scratch = devm_kcalloc(f->dev, FPC1540_IMG_BYTES, sizeof(s32),
				  GFP_KERNEL);
	f->tmp = devm_kcalloc(f->dev, FPC1540_IMG_BYTES, sizeof(s32),
			      GFP_KERNEL);
	if (!f->raw || !f->image || !f->scratch || !f->tmp)
		return -ENOMEM;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	if (!spi->max_speed_hz)
		spi->max_speed_hz = 1000000;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(f->dev, ret, "spi_setup failed\n");

	f->vdd = devm_regulator_get_optional(f->dev, "vdd");
	if (IS_ERR(f->vdd)) {
		if (PTR_ERR(f->vdd) != -ENODEV)
			return dev_err_probe(f->dev, PTR_ERR(f->vdd), "no vdd\n");
		f->vdd = devm_regulator_get_optional(f->dev, "vibr");
		if (IS_ERR(f->vdd))
			f->vdd = NULL;
	}

	f->vddio = devm_regulator_get_optional(f->dev, "vddio");
	if (IS_ERR(f->vddio))
		f->vddio = NULL;

	f->reset = devm_gpiod_get_optional(f->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(f->reset))
		return dev_err_probe(f->dev, PTR_ERR(f->reset), "reset gpio\n");

	f->irq = devm_gpiod_get_optional(f->dev, "irq", GPIOD_IN);
	if (IS_ERR(f->irq))
		return dev_err_probe(f->dev, PTR_ERR(f->irq), "irq gpio\n");

	ret = fpc1540_power_on(f);
	if (ret)
		return dev_err_probe(f->dev, ret, "power on failed\n");

	ret = fpc1540_read_reg(f, FPC_REG_HWID, false, id, 2);
	if (ret)
		return dev_err_probe(f->dev, ret, "hwid read failed\n");
	f->hwid = ((u16)id[0] << 8) | id[1];

	if (!fpc1540_hwid_is_fpc(f->hwid)) {
		dev_info(f->dev, "no FPC part (hwid=0x%04x)\n", f->hwid);
		fpc1540_power_off(f);
		return -ENODEV;
	}

	/* Program the AFE once; captures only re-arm and read. */
	fpc1540_configure(f);
	f->configured = true;

	f->misc.name = FPC1540_DRV_NAME;
	f->misc.minor = MISC_DYNAMIC_MINOR;
	f->misc.fops = &fpc1540_fops;
	f->misc.parent = f->dev;
	ret = devm_device_add_group(f->dev, &fpc1540_group);
	if (ret)
		return dev_err_probe(f->dev, ret, "sysfs group failed\n");
	ret = misc_register(&f->misc);
	if (ret)
		return dev_err_probe(f->dev, ret, "misc_register failed\n");

	ret = fpc1540_request_irq(f);
	if (ret) {
		dev_warn(f->dev, "no finger-detect IRQ: %d\n", ret);
		f->irq_num = 0;
	}

	dev_info(f->dev, "FPC1540 hwid=0x%04x, /dev/%s ready\n",
		 f->hwid, FPC1540_DRV_NAME);

	return 0;
}

static void fpc1540_remove(struct spi_device *spi)
{
	struct fpc1540 *f = spi_get_drvdata(spi);

	misc_deregister(&f->misc);
	fpc1540_power_off(f);
}

static const struct of_device_id fpc1540_of_match[] = {
	{ .compatible = "fpc,fpc1540" },
	{ .compatible = "fpc,fpc1022" },
	{ }
};
MODULE_DEVICE_TABLE(of, fpc1540_of_match);

static const struct spi_device_id fpc1540_id[] = {
	{ "fpc1540", 0 },
	{ "fpc1022", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, fpc1540_id);

static struct spi_driver fpc1540_driver = {
	.driver = {
		.name = FPC1540_DRV_NAME,
		.of_match_table = fpc1540_of_match,
	},
	.probe = fpc1540_probe,
	.remove = fpc1540_remove,
	.id_table = fpc1540_id,
};
module_spi_driver(fpc1540_driver);

MODULE_AUTHOR("xaga mainline");
MODULE_DESCRIPTION("FPC1540/FPC15xx fingerprint sensor SPI driver");
MODULE_LICENSE("GPL");
