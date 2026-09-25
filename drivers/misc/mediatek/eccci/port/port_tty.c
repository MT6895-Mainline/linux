// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 MediaTek Inc.
 */

/*
 * Standard Linux tty front-end for the CCCI UART2 (AT) port.
 *
 * /dev/ttyCCCI0 is a real tty (registered on a platform device, so udev
 * reports SUBSYSTEM=="tty") backed by the exact same struct port_t that the
 * legacy char node /dev/ttyC0 uses (CCCI_UART2_RX/TX, the AT command queue).
 *
 * The legacy char device is left untouched.  The tty only takes over the Rx
 * path while it is open: port_char_recv_skb() consults ccci_tty_is_open()
 * first.  While the tty is closed the Rx/Tx behaviour is byte for byte the
 * old char device behaviour.  Open/close/dequeue semantics mirror
 * port_dev_open()/port_dev_close() from port_proxy.c.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/tty_port.h>

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_bm.h"
#include "port_proxy.h"
#include "port_char.h"
#include "port_t.h"

#if IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER)

#define CCCI_TTY_DRIVER_NAME	"ccci_tty"
#define CCCI_TTY_NAME		"ttyCCCI"
#define CCCI_TTY_INDEX		0

struct ccci_tty_ctx {
	struct port_t		*port;
	struct tty_driver	*driver;
	struct tty_port		port_data;
	struct platform_device	*pdev;
};

static struct ccci_tty_ctx ccci_tty;

/* Serialises registration and open/close. */
static DEFINE_MUTEX(ccci_tty_mutex);
/* Guards ccci_tty_open_tty, which the Rx hook reads from atomic context. */
static DEFINE_SPINLOCK(ccci_tty_rx_lock);
static struct tty_struct *ccci_tty_open_tty;

bool ccci_tty_is_open(void)
{
	unsigned long flags;
	bool open;

	if (!ccci_tty.port)
		return false;

	spin_lock_irqsave(&ccci_tty_rx_lock, flags);
	open = ccci_tty_open_tty != NULL;
	spin_unlock_irqrestore(&ccci_tty_rx_lock, flags);

	return open;
}

/*
 * Rx entry point used by port_char_recv_skb().  Only called while the tty is
 * open; it consumes (and frees) the skb in every case.
 */
void ccci_tty_rx_skb(struct port_t *port, struct sk_buff *skb)
{
	struct tty_struct *tty;
	unsigned long flags;

	if (unlikely(!port || !skb))
		return;

	spin_lock_irqsave(&ccci_tty_rx_lock, flags);
	tty = ccci_tty_open_tty;
	spin_unlock_irqrestore(&ccci_tty_rx_lock, flags);

	if (unlikely(!tty)) {
		/* Lost the race with close: nobody is listening any more. */
		ccci_free_skb(skb);
		return;
	}

	/*
	 * The AT port has no PORT_F_USER_HEADER, so this is exactly what the
	 * static port_adjust_skb() in port_proxy.c does for the char node.
	 */
	skb_pull(skb, sizeof(struct ccci_header));
	tty_insert_flip_string(&ccci_tty.port_data, skb->data, skb->len);
	tty_flip_buffer_push(&ccci_tty.port_data);
	ccci_free_skb(skb);
}

/*
 * The tty core's ->write() has no struct file, so derive O_NONBLOCK from the
 * single open file the tty keeps in tty->tty_files.  Default to blocking for
 * kernel internal writes that have no file attached.
 */
static bool ccci_tty_write_nonblock(struct tty_struct *tty)
{
	struct tty_file_private *priv;
	bool nonblock = false;
	unsigned long flags;

	spin_lock_irqsave(&tty->files_lock, flags);
	list_for_each_entry(priv, &tty->tty_files, list) {
		nonblock = !!(priv->file->f_flags & O_NONBLOCK);
		break;
	}
	spin_unlock_irqrestore(&tty->files_lock, flags);

	return nonblock;
}

static ssize_t ccci_tty_write(struct tty_struct *tty, const u8 *buf,
	size_t count)
{
	struct port_t *port = ccci_tty.port;
	struct sk_buff *skb = NULL;
	struct ccci_header *ccci_h = NULL;
	size_t actual_count = 0;
	bool blocking;
	int ret, i;

	if (unlikely(!port))
		return -ENODEV;
	if (count == 0)
		return 0;

	blocking = !ccci_tty_write_nonblock(tty);

	/* Replicates the non-PORT_F_USER_HEADER branch of port_dev_write(). */
	actual_count = count > CCCI_MTU ? CCCI_MTU : count;
	skb = ccci_alloc_skb(actual_count + sizeof(struct ccci_header), 1,
		0);
	if (unlikely(!skb))
		return -ENOMEM;

	ccci_h = (struct ccci_header *)skb_put(skb,
		sizeof(struct ccci_header));
	ccci_h->data[0] = 0;
	ccci_h->data[1] = actual_count + sizeof(struct ccci_header);
	ccci_h->channel = port->tx_ch;
	ccci_h->reserved = 0;

	/* buf comes from the tty layer and is always kernel memory here. */
	memcpy(skb_put(skb, actual_count), buf, actual_count);

	if (port->flags & PORT_F_CH_TRAFFIC)
		port_ch_dump(port, 1, skb->data + sizeof(struct ccci_header),
			actual_count);

	/*
	 * The CCCI HIF must NEVER be entered in blocking mode from here.
	 * md_ccif_send_skb() answers a full ring with `goto retry` and only
	 * leaves on md_state EXCEPTION/GATED; at READY that is an unbounded
	 * spin in kernel context (the v701 incident, HANDOFF).  The AT queue
	 * is not drained by the modem today, so a blocking write would wedge a
	 * kernel thread as soon as q5 filled.
	 *
	 * The packet is therefore always offered non-blocking, and blocking
	 * callers get a bounded, interruptible-by-signal wait instead.
	 * On failure the skb is still ours (md_ccif_send_skb() only frees it
	 * on the success path), so the same skb can be retried.
	 */
	for (i = 0; i < 200; i++) {
		ret = port_send_skb_to_md(port, skb, 0);
		if (ret != -EBUSY || !blocking)
			break;
		msleep(5);
	}
	if (ret) {
		if (ret == -EBUSY)
			ret = -EAGAIN;
		ccci_free_skb(skb);
		return ret;
	}

	return actual_count;
}

static unsigned int ccci_tty_write_room(struct tty_struct *tty)
{
	struct port_t *port = ccci_tty.port;
	int room;

	if (unlikely(!port))
		return 0;

	room = port_write_room_to_md(port);
	/*
	 * port_write_room_to_md() only returns a value once the port saw a
	 * full Rx queue.  For the flow controlled AT channel we can always
	 * accept up to one MTU; ->write() reports the real error if any.
	 */
	if (room <= 0)
		room = CCCI_MTU;

	return (unsigned int)room;
}

static unsigned int ccci_tty_chars_in_buffer(struct tty_struct *tty)
{
	/* A packet is either handed to the modem or rejected, never queued. */
	return 0;
}

static int ccci_tty_activate(struct tty_port *tport, struct tty_struct *tty)
{
	struct port_t *port = ccci_tty.port;
	unsigned long flags;

	if (unlikely(!port))
		return -ENODEV;

	/* Mirrors port_dev_open(). */
	atomic_inc(&port->usage_cnt);
	port_user_register(port);

	spin_lock_irqsave(&ccci_tty_rx_lock, flags);
	WRITE_ONCE(ccci_tty_open_tty, tty);
	spin_unlock_irqrestore(&ccci_tty_rx_lock, flags);

	CCCI_NORMAL_LOG(port->md_id, CHAR,
		"tty %s open by %s, usage_cnt=%d\n", port->name,
		current->comm, atomic_read(&port->usage_cnt));

	return 0;
}

static void ccci_tty_shutdown(struct tty_port *tport)
{
	struct port_t *port = ccci_tty.port;
	struct sk_buff *skb = NULL;
	unsigned long flags;
	int clear_cnt = 0;
	int md_id;

	if (unlikely(!port))
		return;
	md_id = port->md_id;

	spin_lock_irqsave(&ccci_tty_rx_lock, flags);
	WRITE_ONCE(ccci_tty_open_tty, NULL);
	spin_unlock_irqrestore(&ccci_tty_rx_lock, flags);

	/* Mirrors port_dev_close(). */
	atomic_dec(&port->usage_cnt);
	spin_lock_irqsave(&port->rx_skb_list.lock, flags);
	while ((skb = __skb_dequeue(&port->rx_skb_list)) != NULL) {
		ccci_free_skb(skb);
		clear_cnt++;
	}
	port->rx_drop_cnt += clear_cnt;
	/* flush Rx */
	port_ask_more_req_to_md(port);
	spin_unlock_irqrestore(&port->rx_skb_list.lock, flags);
	port_user_unregister(port);

	CCCI_NORMAL_LOG(md_id, CHAR,
		"tty %s close by %s, clear_cnt=%d, drop=%d usage_cnt=%d\n",
		port->name, current->comm, clear_cnt, port->rx_drop_cnt,
		atomic_read(&port->usage_cnt));
}

static int ccci_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	return tty_port_install(&ccci_tty.port_data, driver, tty);
}

static int ccci_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct port_t *port = ccci_tty.port;
	int ret;

	if (unlikely(!port))
		return -ENODEV;

	mutex_lock(&ccci_tty_mutex);
	/* Like port_dev_open(): the AT port has a single user at a time. */
	if (atomic_read(&port->usage_cnt)) {
		mutex_unlock(&ccci_tty_mutex);
		return -EBUSY;
	}
	ret = tty_port_open(&ccci_tty.port_data, tty, filp);
	mutex_unlock(&ccci_tty_mutex);

	return ret;
}

static void ccci_tty_close(struct tty_struct *tty, struct file *filp)
{
	mutex_lock(&ccci_tty_mutex);
	tty_port_close(&ccci_tty.port_data, tty, filp);
	mutex_unlock(&ccci_tty_mutex);
}

static const struct tty_operations ccci_tty_ops = {
	.install = ccci_tty_install,
	.open = ccci_tty_open,
	.close = ccci_tty_close,
	.write = ccci_tty_write,
	.write_room = ccci_tty_write_room,
	.chars_in_buffer = ccci_tty_chars_in_buffer,
};

static const struct tty_port_operations ccci_tty_port_ops = {
	.activate = ccci_tty_activate,
	.shutdown = ccci_tty_shutdown,
};

/*
 * A platform tty must have a *bound driver*: udev derives the DRIVERS property
 * from the device chain, and every ModemManager plugin, generic included, drops
 * a port with "filtered as couldn't retrieve drivers" when there is none.
 * Binding this no-op driver to the "ccci_tty" platform device makes sysfs show
 * /sys/devices/platform/ccci_tty/driver -> .../ccci_tty and gives udev
 * DRIVERS=ccci_tty.  It has no other function.
 */
static int ccci_tty_plat_probe(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ccci_tty_platform_driver = {
	.probe = ccci_tty_plat_probe,
	.driver = {
		.name = CCCI_TTY_DRIVER_NAME,
	},
};

int ccci_tty_port_register(struct port_t *port)
{
	struct tty_driver *driver = NULL;
	struct platform_device *pdev = NULL;
	struct device *dev;
	int ret;

	if (unlikely(!port))
		return -EINVAL;

	mutex_lock(&ccci_tty_mutex);
	if (ccci_tty.port) {
		/* Already registered by a previous init. */
		mutex_unlock(&ccci_tty_mutex);
		return 0;
	}

	ret = platform_driver_register(&ccci_tty_platform_driver);
	if (ret) {
		CCCI_ERROR_LOG(port->md_id, CHAR,
			"register platform driver %s fail, ret=%d\n",
			CCCI_TTY_DRIVER_NAME, ret);
		goto out_unlock;
	}

	pdev = platform_device_register_simple(CCCI_TTY_DRIVER_NAME, -1,
		NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		CCCI_ERROR_LOG(port->md_id, CHAR,
			"register platform device %s fail, ret=%d\n",
			CCCI_TTY_DRIVER_NAME, ret);
		pdev = NULL;
		goto out_unlock;
	}

	driver = tty_alloc_driver(1,
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(driver)) {
		ret = PTR_ERR(driver);
		driver = NULL;
		CCCI_ERROR_LOG(port->md_id, CHAR,
			"alloc tty driver %s fail, ret=%d\n",
			CCCI_TTY_DRIVER_NAME, ret);
		goto out_unlock;
	}

	driver->driver_name = CCCI_TTY_DRIVER_NAME;
	driver->name = CCCI_TTY_NAME;
	driver->type = TTY_DRIVER_TYPE_SERIAL;
	driver->subtype = SERIAL_TYPE_NORMAL;
	driver->init_termios = tty_std_termios;
	driver->init_termios.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	tty_set_operations(driver, &ccci_tty_ops);

	tty_port_init(&ccci_tty.port_data);
	ccci_tty.port_data.ops = &ccci_tty_port_ops;

	ret = tty_register_driver(driver);
	if (ret) {
		CCCI_ERROR_LOG(port->md_id, CHAR,
			"register tty driver %s fail, ret=%d\n",
			CCCI_TTY_DRIVER_NAME, ret);
		goto out_free_driver;
	}

	dev = tty_port_register_device_attr(&ccci_tty.port_data, driver,
		CCCI_TTY_INDEX, &pdev->dev, NULL, NULL);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		CCCI_ERROR_LOG(port->md_id, CHAR,
			"register tty device %s%d fail, ret=%d\n",
			CCCI_TTY_NAME, CCCI_TTY_INDEX, ret);
		goto out_unreg_driver;
	}

	ccci_tty.port = port;
	ccci_tty.driver = driver;
	ccci_tty.pdev = pdev;
	mutex_unlock(&ccci_tty_mutex);

	CCCI_NORMAL_LOG(port->md_id, CHAR,
		"tty front-end /dev/%s%d registered for %s (ch %d/%d)\n",
		CCCI_TTY_NAME, CCCI_TTY_INDEX, port->name, port->rx_ch,
		port->tx_ch);

	return 0;

out_unreg_driver:
	tty_unregister_driver(driver);
out_free_driver:
	tty_driver_kref_put(driver);
	tty_port_destroy(&ccci_tty.port_data);
out_unlock:
	if (pdev)
		platform_device_unregister(pdev);
	mutex_unlock(&ccci_tty_mutex);

	return ret;
}

#else /* !IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER) */

int ccci_tty_port_register(struct port_t *port)
{
	return 0;
}

bool ccci_tty_is_open(void)
{
	return false;
}

void ccci_tty_rx_skb(struct port_t *port, struct sk_buff *skb)
{
	ccci_free_skb(skb);
}

#endif /* IS_ENABLED(CONFIG_MTK_ECCCI_DRIVER) */
