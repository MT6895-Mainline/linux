// SPDX-License-Identifier: GPL-2.0
/* qqc_fast: read-only fast histogram of APCCIF BUSY/START (5 ms x 3 s). */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define AP_CCIF_PA 0x10209000UL
#define N 600

struct pair { u32 busy, start; int cnt; };

static int __init qqc_fast_init(void)
{
	void __iomem *b = ioremap(AP_CCIF_PA, 0x1000);
	struct pair p[16];
	int np = 0, i, j;
	u32 tchg = 0, lchg = 0, first_b = 0, first_s = 0, same = 0;

	if (!b) { pr_err("QQCFAST ioremap fail\n"); return -ENOMEM; }
	for (i = 0; i < N; i++) {
		u32 busy = readl(b + 0x04);
		u32 start = readl(b + 0x08);
		if (i == 0) { first_b = busy; first_s = start; }
		else if (busy == first_b && start == first_s) same++;
		for (j = 0; j < np; j++)
			if (p[j].busy == busy && p[j].start == start) { p[j].cnt++; break; }
		if (j == np && np < 16) { p[np].busy = busy; p[np].start = start; p[np].cnt = 1; np++; }
		udelay(5000);
	}
	pr_info("QQCFAST samples=%d nonequal_to_first=%u\n", N, same);
	for (j = 0; j < np; j++)
		pr_info("QQCFAST value[%d] busy=%08x start=%08x cnt=%d\n", j, p[j].busy, p[j].start, p[j].cnt);
	pr_info("QQCFAST first busy=%08x start=%08x\n", first_b, first_s);
	iounmap(b);
	return 0;
}
static void __exit qqc_fast_exit(void) {}
module_init(qqc_fast_init); module_exit(qqc_fast_exit);
MODULE_LICENSE("GPL");
