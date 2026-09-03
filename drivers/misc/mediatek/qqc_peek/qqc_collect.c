// SPDX-License-Identifier: GPL-2.0
/* qqc_collect: one-shot read-only CCIF collector (mode=1 window dump, mode=2 fast hist). */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>

#define AP_CCIF_PA 0x10209000UL
#define MD_CCIF_PA 0x1020a000UL
#define WIN 0x1000
#define MODE 1

static int mode = 0;
module_param(mode, int, 0400);
MODULE_PARM_DESC(mode, "0=both, 1=window dump, 2=fast hist");

static void dump_win(void __iomem *b, const char *tag)
{
	u32 v[WIN / 4];
	int i;

	for (i = 0; i < WIN / 4; i++)
		v[i] = readl(b + i * 4);
	pr_info("QQCCOL WINDOW %s begin (%d bytes)\n", tag, WIN);
	for (i = 0; i < WIN / 4; i += 8)
		pr_info("QQCCOL %s %03x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			tag, i * 4, v[i], v[i+1], v[i+2], v[i+3],
			v[i+4], v[i+5], v[i+6], v[i+7]);
	pr_info("QQCCOL WINDOW %s end\n", tag);
}

static void fast_hist(void __iomem *b)
{
	u32 busy, start, pb = 0, ps = 0, chg = 0, n1 = 0;
	int i;

	for (i = 0; i < 600; i++) {
		busy = readl(b + 0x04);
		start = readl(b + 0x08);
		if (busy == 1 && start == 1) n1++;
		if (i && (busy != pb || start != ps)) {
			chg++;
			if (chg <= 20)
				pr_info("QQCCOL chg t=%dms busy=%08x start=%08x\n", i * 5, busy, start);
		}
		pb = busy; ps = start;
		udelay(5000);
	}
	pr_info("QQCCOL HIST n=600 changes=%u busy1start1=%u final busy=%08x start=%08x\n",
		chg, n1, pb, ps);
}

static int __init qqc_collect_init(void)
{
	void __iomem *ap = ioremap(AP_CCIF_PA, WIN);
	void __iomem *md = ioremap(MD_CCIF_PA, WIN);

	if (!ap) { pr_err("QQCCOL ap ioremap fail\n"); return -ENOMEM; }
	if (!md) pr_err("QQCCOL md ioremap fail (MD window skipped)\n");

	pr_info("QQCCOL start mode=%d ap=%px md=%px BUSY=%08x START=%08x\n",
		mode, ap, md, readl(ap + 0x04), readl(ap + 0x08));
	if (mode == 0 || mode == 1) {
		dump_win(ap, "APCCIF");
		if (md)
			dump_win(md, "MDCCIF");
	}
	if (mode == 0 || mode == 2)
		fast_hist(ap);
	pr_info("QQCCOL done\n");
	return 0;
}
static void __exit qqc_collect_exit(void) {}
module_init(qqc_collect_init); module_exit(qqc_collect_exit);
MODULE_LICENSE("GPL");
