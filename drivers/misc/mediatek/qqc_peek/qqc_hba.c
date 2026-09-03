// SPDX-License-Identifier: GPL-2.0
/* qqc_hba v2: kernel-side MIPC client on ttyCMIPC4, attached before MD start.
 * Split TX/RX threads + per-send timing, to answer "which call blocks?". */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/string.h>

extern int mtk_ccci_request_port(char *name);
extern int mtk_ccci_open_port(int index);
extern int mtk_ccci_send_data(int index, const char *buf, int size);
extern int mtk_ccci_read_data(int index, char *buf, size_t count);

#define QQC "QQCHBA"
static int port = -1;
static volatile int stop;
static u16 txid;

static void mk_frame(u8 *f, u16 id)
{
	memset(f, 0, 16);
	*(u32 *)(f + 0) = 0x24541984;
	f[8] = 0xff;			/* ps = ALL (stock TEST/OPEN) */
	*(u16 *)(f + 10) = id;
	*(u16 *)(f + 12) = ++txid;
	*(u16 *)(f + 14) = 0;
}

static int tx_thread(void *x)
{
	u8 f[16];
	int i, r;
	unsigned long t0, dt;

	for (i = 0; i < 400 && !stop; i++) {
		mk_frame(f, 0x0305);
		t0 = jiffies;
		r = mtk_ccci_send_data(port, (char *)f, 16);
		dt = jiffies - t0;
		if (i < 5 || i % 20 == 0 || dt > 10)
			pr_info(QQC " TX #%d ret=%d dt=%luj(%lums)\n", i, r, dt,
				dt * 1000 / HZ);
		msleep(100);
	}
	pr_info(QQC " tx thread end at #%d\n", i);
	return 0;
}

static int rx_thread(void *x)
{
	u8 rx[256];
	int n, total = 0, loops = 0;

	while (!stop) {
		n = mtk_ccci_read_data(port, (char *)rx, sizeof(rx));
		loops++;
		if (n > 0) {
			total += n;
			pr_info(QQC " RX %d bytes (total %d): %*phN\n", n, total,
				n > 32 ? 32 : n, rx);
		}
		if (loops % 600 == 0)
			pr_info(QQC " rx alive: loops=%d total_rx=%d\n", loops, total);
		msleep(50);
	}
	pr_info(QQC " rx thread end loops=%d total_rx=%d\n", loops, total);
	return 0;
}

static int __init qqc_hba_init(void)
{
	int r;

	port = mtk_ccci_request_port("ttyCMIPC4");
	pr_info(QQC " request_port(ttyCMIPC4) = %d\n", port);
	if (port < 0)
		return -ENODEV;
	r = mtk_ccci_open_port(port);
	pr_info(QQC " open_port(%d) = %d\n", port, r);
	if (r < 0)
		return r;
	kthread_run(tx_thread, NULL, "qqc_hba_tx");
	kthread_run(rx_thread, NULL, "qqc_hba_rx");
	return 0;
}
static void __exit qqc_hba_exit(void) { stop = 1; pr_info(QQC " exit\n"); }
module_init(qqc_hba_init);
module_exit(qqc_hba_exit);
MODULE_LICENSE("GPL");
