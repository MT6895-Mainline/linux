// SPDX-License-Identifier: GPL-2.0
/* qqc_unlatch: clear APCCIF_BUSY bit0 once, then watch BUSY/START for 60 s. */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>

#define AP_CCIF_PA 0x10209000UL
static void __iomem *b;

static int watch(void *x)
{
	u32 pb = ~0u, ps = ~0u;
	int i;

	for (i = 0; i < 60; i++) {
		u32 busy = readl(b + 0x04), start = readl(b + 0x08);
		if (busy != pb || start != ps)
			pr_info("QQCUNLATCH t=%2ds busy=%08x start=%08x\n", i, busy, start);
		pb = busy; ps = start;
		msleep(1000);
	}
	pr_info("QQCUNLATCH watch end busy=%08x start=%08x\n", readl(b + 0x04), readl(b + 0x08));
	return 0;
}

static int __init unlatch_init(void)
{
	u32 busy0, start0;

	b = ioremap(AP_CCIF_PA, 0x1000);
	if (!b) return -ENOMEM;
	busy0 = readl(b + 0x04); start0 = readl(b + 0x08);
	pr_info("QQCUNLATCH before busy=%08x start=%08x\n", busy0, start0);
	writel(busy0 & ~1u, b + 0x04);
	mb();
	pr_info("QQCUNLATCH wrote busy&~1 -> busy=%08x start=%08x\n",
		readl(b + 0x04), readl(b + 0x08));
	kthread_run(watch, NULL, "qqc_unlatch");
	return 0;
}
static void __exit unlatch_exit(void) {}
module_init(unlatch_init); module_exit(unlatch_exit);
MODULE_LICENSE("GPL");
