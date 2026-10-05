// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 The Pearl mainline port
 *
 * CCCI AT-port TTY driver.
 *
 * The modem AT channel (CCCI_UART2, the queue the vendor port table names
 * "ttyC0") used to be a raw char node.  ModemManager - and every other
 * standard modem tool - needs a real tty, so this port op registers
 * /dev/ttyCCCI0 and bridges it onto the very same CCCI data path:
 *
 *   write():   ccci_alloc_skb() + CCCI header + port_send_skb_to_md()
 *   receive:   port_ops.recv_skb() -> drop the CCCI header -> tty flip buffer
 *
 * The 16-byte CCCI header is stripped here exactly like the char port did
 * (PORT_F_ADJUST_HEADER), so the tty only ever sees AT payload bytes.
 *
 * The device node is a child of a tiny platform device, and that is
 * deliberate: a parentless tty lives in /sys/devices/virtual/..., where
 * ModemManager (a) filters it out as a "virtual device" and (b) cannot
 * retrieve any driver name for it, which makes every plugin refuse the port.
 *
 * All device-model work happens in a late_initcall, never from the CCI port
 * init: registering platform devices/drivers while the modem driver is still
 * probing deadlocks the boot before userspace ever starts.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/tty_port.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_bm.h"
#include "ccci_core.h"
#include "ccci_fsm.h"
#include "port_proxy.h"
#include "port_t.h"

#define AT_TTY_DRV_NAME		"ttyCCCI"
#define AT_TTY_DEV_NAME		"ttyCCCI"
#define AT_TTY_MINORS		1
#define AT_TTY_PDEV_NAME	"ccci_at_tty"
/* same packet budget port_dev_write() enforces (CCCI_MTU incl. header) */
#define AT_TTY_WRITE_ROOM	(CCCI_MTU - sizeof(struct ccci_header))

struct ccci_at_tty {
	struct tty_driver *driver;
	struct tty_port port;
	struct port_t *ccci_port;
	struct device *dev;
	struct platform_device *pdev;
	bool port_inited;
};

static struct ccci_at_tty at_tty;

/* ------------------------------------------------------------- tty side */

static int ccci_at_tty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(&at_tty.port, tty, filp);
}

static void ccci_at_tty_close(struct tty_struct *tty, struct file *filp)
{
	tty_port_close(&at_tty.port, tty, filp);
}

static void ccci_at_tty_hangup(struct tty_struct *tty)
{
	tty_port_hangup(&at_tty.port);
}

static ssize_t ccci_at_tty_write(struct tty_struct *tty, const u8 *buf,
				 size_t count)
{
	struct port_t *port = at_tty.ccci_port;
	struct sk_buff *skb;
	struct ccci_header *ccci_h;
	size_t n;
	int ret, md_state;

	if (port == NULL)
		return -ENODEV;
	if (count == 0)
		return 0;

	md_state = ccci_fsm_get_md_state(port->md_id);
	if (md_state != READY && md_state != EXCEPTION) {
		/* never block a writer while the modem is not up */
		CCCI_DEBUG_LOG(port->md_id, CHAR,
			"%s: MD not ready (state %d)\n", __func__, md_state);
		return -ENODEV;
	}

	/* never hand the modem more than one CCCI packet worth of data */
	n = count > AT_TTY_WRITE_ROOM ? AT_TTY_WRITE_ROOM : count;
	/* non-blocking: a full TX queue must surface as -EAGAIN to the tty
	 * layer (which retries) instead of parking the writer in D state
	 */
	skb = ccci_alloc_skb(n + sizeof(struct ccci_header), 1, 0);
	if (skb == NULL) {
		CCCI_DEBUG_LOG(port->md_id, CHAR,
			"%s: alloc skb fail (%zu)\n", __func__, n);
		return -EAGAIN;
	}

	ccci_h = skb_put(skb, sizeof(struct ccci_header));
	ccci_h->data[0] = 0;
	ccci_h->data[1] = n + sizeof(struct ccci_header);
	ccci_h->channel = port->tx_ch;
	ccci_h->reserved = 0;
	memcpy(skb_put(skb, n), buf, n);

	if (port->flags & PORT_F_CH_TRAFFIC)
		port_ch_dump(port, 1, (void *)buf, n);

	ret = port_send_skb_to_md(port, skb, 0);
	if (ret) {
		/* port_send_skb_to_md() consumes the skb on failure */
		CCCI_DEBUG_LOG(port->md_id, CHAR,
			"%s: send to MD fail %d\n", __func__, ret);
		return ret == -EBUSY ? -EAGAIN : ret;
	}
	return n;
}

static unsigned int ccci_at_tty_write_room(struct tty_struct *tty)
{
	return AT_TTY_WRITE_ROOM;
}

static const struct tty_operations ccci_at_tty_ops = {
	.open = ccci_at_tty_open,
	.close = ccci_at_tty_close,
	.hangup = ccci_at_tty_hangup,
	.write = ccci_at_tty_write,
	.write_room = ccci_at_tty_write_room,
};

/* tty_port_init() leaves port->ops NULL in this tree, and tty_port_open()
 * dereferences port->ops->activate unconditionally, so a port ops struct is
 * mandatory.  A virtual port has no carrier/DTR lines: all hooks are optional
 * and stay NULL, which the tty_port helpers tolerate.
 */
static const struct tty_port_operations ccci_at_tty_port_ops = {
};

/* ------------------------------------------------------------ port side */

static int port_tty_init(struct port_t *port)
{
	struct tty_driver *drv;
	int ret;
	int md_id = port->md_id;

	CCCI_NORMAL_LOG(md_id, CHAR,
		"tty port %s is initializing\n", port->name);

	port->skb_from_pool = 1;
	port->interception = 0;
	/* nothing is queued here, but keep a sane flow-control watermark so the
	 * HIF never thinks this port is permanently full
	 */
	port->rx_length_th = 32;

	if (at_tty.driver != NULL) {
		/* driver already registered (md1 only in practice) */
		at_tty.ccci_port = port;
		return 0;
	}
	at_tty.ccci_port = port;
	tty_port_init(&at_tty.port);
	at_tty.port.ops = &ccci_at_tty_port_ops;

	drv = tty_alloc_driver(AT_TTY_MINORS,
			TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
			TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(drv)) {
		ret = PTR_ERR(drv);
		CCCI_ERROR_LOG(md_id, CHAR,
			"tty_alloc_driver fail %d\n", ret);
		return ret;
	}
	drv->driver_name = AT_TTY_DRV_NAME;
	drv->name = AT_TTY_DEV_NAME;
	drv->major = 0;			/* dynamic major */
	drv->minor_start = 0;
	drv->type = TTY_DRIVER_TYPE_SERIAL;
	drv->subtype = SERIAL_TYPE_NORMAL;
	drv->init_termios = tty_std_termios;
	drv->init_termios.c_cflag =
		B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	drv->init_termios.c_lflag = 0;
	tty_set_operations(drv, &ccci_at_tty_ops);

	/* Only the tty driver itself is registered here; it is pure tty-core
	 * bookkeeping.  The device node (which needs the driver model) is
	 * registered later from ccci_at_tty_late_init().
	 */
	ret = tty_register_driver(drv);
	if (ret) {
		CCCI_ERROR_LOG(md_id, CHAR,
			"tty_register_driver fail %d\n", ret);
		tty_driver_kref_put(drv);
		return ret;
	}
	at_tty.driver = drv;
	at_tty.port_inited = true;

	CCCI_NORMAL_LOG(md_id, CHAR,
		"AT tty driver %s ready (tx_ch=%d rx_ch=%d), node comes up later\n",
		AT_TTY_DRV_NAME, port->tx_ch, port->rx_ch);
	return 0;
}

static int port_tty_recv_skb(struct port_t *port, struct sk_buff *skb)
{
	unsigned int len;
	const u8 *data;

	if (skb->len <= sizeof(struct ccci_header)) {
		/* a bare CCCI header carries no AT payload: ignore it */
		ccci_free_skb(skb);
		return 0;
	}

	len = skb->len - sizeof(struct ccci_header);
	data = skb->data + sizeof(struct ccci_header);

	if (port->flags & PORT_F_CH_TRAFFIC)
		port_ch_dump(port, 0, (void *)data, len);

	/* No tty reader?  The core drops the data for us, which is exactly what
	 * the char port did when it had no open handle: AT responses are never
	 * replayed to a later reader.
	 */
	tty_insert_flip_string(&at_tty.port, data, len);
	tty_flip_buffer_push(&at_tty.port);

	ccci_free_skb(skb);
	return 0;
}

static void port_tty_dump_info(struct port_t *port, unsigned int flag)
{
	if (port == NULL)
		return;
	CCCI_REPEAT_LOG(port->md_id, CHAR,
		"ATTY(%s):%dT tty=%s\n",
		port->name, port->tx_pkg_cnt,
		at_tty.driver ? "up" : "down");
}

struct port_ops tty_port_ops = {
	.init = &port_tty_init,
	.recv_skb = &port_tty_recv_skb,
	.dump_info = &port_tty_dump_info,
};

/* --------------------------------------------------------- device model */

/* A zero-work platform driver: it exists only so that the tty's parent device
 * carries a real driver name in sysfs, which is what ModemManager needs in
 * order to accept the port at all.
 */
static int ccci_at_tty_pdev_probe(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ccci_at_tty_pdrv = {
	.probe = ccci_at_tty_pdev_probe,
	.driver = {
		.name = AT_TTY_PDEV_NAME,
	},
};

static int __init ccci_at_tty_late_init(void)
{
	struct platform_device *pdev;
	struct device *dev;
	int ret;

	if (!at_tty.port_inited)
		return 0;	/* AT port never showed up, nothing to do */

	ret = platform_driver_register(&ccci_at_tty_pdrv);
	if (ret) {
		pr_err("ccci_at_tty: platform_driver_register fail %d\n", ret);
		return ret;
	}

	pdev = platform_device_register_simple(AT_TTY_PDEV_NAME,
			PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		pr_err("ccci_at_tty: platform_device_register fail %d\n", ret);
		return ret;
	}
	at_tty.pdev = pdev;

	/* tty_port_register_device() also links the port into driver->ports[];
	 * without that link tty_init_dev() refuses the open with -EINVAL.
	 */
	dev = tty_port_register_device(&at_tty.port, at_tty.driver, 0,
			&pdev->dev);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("ccci_at_tty: tty_port_register_device fail %d\n", ret);
		return ret;
	}
	at_tty.dev = dev;

	pr_info("ccci_at_tty: AT tty registered as /dev/%s0\n", AT_TTY_DEV_NAME);
	return 0;
}
late_initcall(ccci_at_tty_late_init);
