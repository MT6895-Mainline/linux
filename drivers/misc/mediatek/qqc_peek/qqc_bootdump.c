// SPDX-License-Identifier: GPL-2.0
/*
 * qqcandy boot-time CCIF snapshot (read-only + RAM text capture) — v842b
 *
 * Captures the WORKING (stock 5.10) AP-side CCIF state during the md_state 2->3
 * window (see docs-local/v840-mipc-probe/RESULT.md).
 *
 * Two capture channels:
 *  1. printk -> console -> (if the platform registers it) ramoops/pstore.
 *  2. a rolling text buffer mirrored into the tail of the ramoops RAM region
 *     (0x48090000 + 0xC0000) behind a magic, readable later with a read-only
 *     physical peek from Arch.  This is a plain RAM write inside the region the
 *     stock platform reserves for ramoops; no partition is touched.
 *
 * Safety: only ioremap()+readl() of the two CCIF register windows the vendor
 * CCCI driver already owns (AP_CCIF 0x10209000 / MD_CCIF 0x1020a000, per
 * arch/arm64/boot/dts/mediatek/mt6895.dts ccifdriver), plus the RAM text
 * mirror above.  No other MMIO, no MD-private DRAM, no partition writes.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/stdarg.h>

#define QQC_MARK "QQCDUMP"

#define AP_CCIF_PA 0x10209000UL
#define MD_CCIF_PA 0x1020a000UL
#define CCIF_WIN   0x1000UL

#define APCCIF_CON       0x00
#define APCCIF_BUSY      0x04
#define APCCIF_START     0x08
#define APCCIF_TCHNUM    0x0C
#define APCCIF_RCHNUM    0x10
#define APCCIF_ACK       0x14
#define APCCIF_IRQ0_MASK 0x20
#define APCCIF_IRQ1_MASK 0x24
#define APCCIF_CHDATA    0x100
#define PCCIF_SRAM_SIZE  512
#define TAIL_OFF         (APCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32))

#define QQC_PA_BASE    0x48090000UL
#define QQC_TEXT_OFF   0xC0000UL
#define QQC_TEXT_MAX   65536
#define QQC_TEXT_MAGIC 0x5151434438353331ULL	/* "QQCD8531" */

struct qqc_hdr {
	u64 magic;
	u32 len;
	u32 seq;
};

static void __iomem *qqc_apb;
static void __iomem *qqc_mdb;
static char qqc_text[QQC_TEXT_MAX];
static u32 qqc_text_len;
static u32 qqc_seq;
static DEFINE_SPINLOCK(qqc_lock);

/* default OFF: do not touch the ramoops RAM region on someone else's device */
static bool mirror;
module_param(mirror, bool, 0400);
MODULE_PARM_DESC(mirror, "mirror the capture into the ramoops RAM tail (default off)");

/* append to the rolling buffer and also print to the console */
static void qqc_emit(const char *fmt, ...)
{
	va_list ap;
	char line[256];
	unsigned long flags;
	int n;

	va_start(ap, fmt);
	n = vscnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);

	pr_info("%s %s", QQC_MARK, line);

	spin_lock_irqsave(&qqc_lock, flags);
	if (qqc_text_len + n < QQC_TEXT_MAX) {
		memcpy(qqc_text + qqc_text_len, line, n);
		qqc_text_len += n;
	}
	spin_unlock_irqrestore(&qqc_lock, flags);
}

/* mirror the rolling buffer into the ramoops RAM tail */
static void qqc_flush(void)
{
	void __iomem *map;

	if (!mirror)
		return;
	unsigned long flags;
	struct qqc_hdr hdr;
	u32 len;

	spin_lock_irqsave(&qqc_lock, flags);
	len = qqc_text_len;
	hdr.magic = QQC_TEXT_MAGIC;
	hdr.len = len;
	hdr.seq = ++qqc_seq;
	spin_unlock_irqrestore(&qqc_lock, flags);

	map = memremap(QQC_PA_BASE + QQC_TEXT_OFF,
		       sizeof(hdr) + QQC_TEXT_MAX, MEMREMAP_WB);
	if (!map) {
		pr_err("%s flush memremap failed\n", QQC_MARK);
		return;
	}
	memcpy_toio(map, &hdr, sizeof(hdr));
	memcpy_toio(map + sizeof(hdr), qqc_text, len);
	memunmap(map);
}

static void qqc_line(const char *tag)
{
	u32 con, busy, start, tch, rch, ack, iq0, iq1;
	u32 ta0, ta1, ta2, tb0, tb1, tb2;

	if (!qqc_apb)
		return;

	con   = readl(qqc_apb + APCCIF_CON);
	busy  = readl(qqc_apb + APCCIF_BUSY);
	start = readl(qqc_apb + APCCIF_START);
	tch   = readl(qqc_apb + APCCIF_TCHNUM);
	rch   = readl(qqc_apb + APCCIF_RCHNUM);
	ack   = readl(qqc_apb + APCCIF_ACK);
	iq0   = readl(qqc_apb + APCCIF_IRQ0_MASK);
	iq1   = readl(qqc_apb + APCCIF_IRQ1_MASK);

	ta0 = readl(qqc_apb + TAIL_OFF);
	ta1 = readl(qqc_apb + TAIL_OFF + 4);
	ta2 = readl(qqc_apb + TAIL_OFF + 8);
	if (qqc_mdb) {
		tb0 = readl(qqc_mdb + TAIL_OFF);
		tb1 = readl(qqc_mdb + TAIL_OFF + 4);
		tb2 = readl(qqc_mdb + TAIL_OFF + 8);
	} else {
		tb0 = tb1 = tb2 = 0;
	}

	qqc_emit("%-8s con=%08x busy=%08x start=%08x tch=%08x rch=%08x ack=%08x iq0=%08x iq1=%08x tailA=%08x,%08x,%08x tailB=%08x,%08x,%08x\n",
		 tag, con, busy, start, tch, rch, ack, iq0, iq1,
		 ta0, ta1, ta2, tb0, tb1, tb2);
}

static void qqc_sram_dump(const char *tag)
{
	u32 i;
	u32 v[PCCIF_SRAM_SIZE / sizeof(u32)];

	if (!qqc_apb)
		return;

	for (i = 0; i < ARRAY_SIZE(v); i++)
		v[i] = readl(qqc_apb + APCCIF_CHDATA + i * 4);

	qqc_emit("SRAM %s begin\n", tag);
	for (i = 0; i < ARRAY_SIZE(v); i += 8)
		qqc_emit("SRAM %s %04x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			 tag, i * 4, v[i], v[i + 1], v[i + 2], v[i + 3],
			 v[i + 4], v[i + 5], v[i + 6], v[i + 7]);
	qqc_emit("SRAM %s end\n", tag);
}

static int qqc_bootdump_thread(void *unused)
{
	int i;

	for (i = 0; i < 60; i++) {
		char tag[16];

		snprintf(tag, sizeof(tag), "t=%d.%01ds", i / 2, (i % 2) * 5);
		qqc_line(tag);
		if (i == 26 || i == 50) {
			char stag[16];

			snprintf(stag, sizeof(stag), "t=%d.%01ds", i / 2, (i % 2) * 5);
			qqc_sram_dump(stag);
		}
		if ((i % 4) == 3 || i == 59)
			qqc_flush();
		msleep(500);
	}
	qqc_flush();
	pr_info("%s thread done len=%u\n", QQC_MARK, qqc_text_len);
	return 0;
}

static int __init qqc_bootdump_init(void)
{
	qqc_apb = ioremap(AP_CCIF_PA, CCIF_WIN);
	if (!qqc_apb) {
		pr_err("%s ioremap AP_CCIF failed\n", QQC_MARK);
		return -ENOMEM;
	}
	qqc_mdb = ioremap(MD_CCIF_PA, CCIF_WIN);
	if (!qqc_mdb)
		pr_err("%s ioremap MD_CCIF failed (tailB reads 0)\n", QQC_MARK);

	qqc_emit("init ok ap=%px md=%px\n", qqc_apb, qqc_mdb);
	qqc_flush();
	kthread_run(qqc_bootdump_thread, NULL, "qqc_bootdump");
	return 0;
}

#ifdef MODULE
static void __exit qqc_bootdump_exit(void) { pr_info("%s module exit\n", QQC_MARK); }
module_init(qqc_bootdump_init);
module_exit(qqc_bootdump_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("qqcandy CCIF boot snapshot (v842b)");
#else
late_initcall(qqc_bootdump_init);
#endif
