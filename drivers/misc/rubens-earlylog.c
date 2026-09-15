// SPDX-License-Identifier: GPL-2.0
/* Independent early printk ring retained in DRAM across warm reboot. */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/rubens_earlylog.h>

#include <asm/cacheflush.h>

#define RUBENS_EARLYLOG_PA	0x48180000UL
/*
 * Two 128KB slots (256KB total). The ring must fit in a single arm64
 * early_ioremap() call, which is limited to NR_FIX_BTMAPS (65) pages: the
 * earlier 512KB layout silently failed to map and left the ring disarmed.
 */
#define RUBENS_EARLYLOG_SIZE	0x40000UL
#define RUBENS_EARLYLOG_SLOT	(RUBENS_EARLYLOG_SIZE / 2)
#define RUBENS_EARLYLOG_HDR	0x1000U
#define RUBENS_EARLYLOG_RING	(RUBENS_EARLYLOG_SLOT - RUBENS_EARLYLOG_HDR)
#define RUBENS_EARLYLOG_MAGIC	0x474f4c45U /* "ELOG" */
#define RUBENS_EARLYLOG_COMMIT	0x54494d43U /* "CMIT" */
#define RUBENS_EARLYLOG_MAX_MSG	256

static void __iomem *rubens_earlylog_base;
static unsigned int rubens_earlylog_current;
static unsigned int rubens_earlylog_previous = UINT_MAX;
static atomic_t rubens_earlylog_busy = ATOMIC_INIT(0);
static raw_spinlock_t rubens_earlylog_lock;

static void __iomem *rubens_earlylog_header(unsigned int slot)
{
	return rubens_earlylog_base + slot * RUBENS_EARLYLOG_SLOT;
}

static void __iomem *rubens_earlylog_ring(unsigned int slot)
{
	return rubens_earlylog_header(slot) + RUBENS_EARLYLOG_HDR;
}

static bool rubens_earlylog_valid(unsigned int slot, u32 *generation)
{
	void __iomem *header = rubens_earlylog_header(slot);

	if (readl(header) != RUBENS_EARLYLOG_MAGIC ||
	    readl(header + 4) != 1 ||
	    readl(header + 24) != RUBENS_EARLYLOG_COMMIT)
		return false;
	*generation = readl(header + 8);
	return true;
}

static void rubens_earlylog_start(void)
{
	void __iomem *header = rubens_earlylog_header(rubens_earlylog_current);
	u32 generation = 0;

	if (rubens_earlylog_previous != UINT_MAX)
		generation = readl(rubens_earlylog_header(rubens_earlylog_previous) + 8);

	/* Invalidate first, then publish the new slot only after its metadata is
	 * complete. A reset during this sequence must leave the other slot valid. */
	writel(0, header + 24);
	writel(RUBENS_EARLYLOG_MAGIC, header);
	writel(1, header + 4);
	writel(generation + 1, header + 8);
	writel(0, header + 12);
	writel(0, header + 16);
	writel(0, header + 20);
	writel(RUBENS_EARLYLOG_COMMIT, header + 24);
}

static void rubens_earlylog_write(const char *buf, size_t len)
{
	void __iomem *header;
	void __iomem *ring;
	u32 cursor;
	size_t i;
	unsigned long flags;

	if (!rubens_earlylog_base || atomic_xchg(&rubens_earlylog_busy, 1))
		return;

	raw_spin_lock_irqsave(&rubens_earlylog_lock, flags);
	header = rubens_earlylog_header(rubens_earlylog_current);
	ring = rubens_earlylog_ring(rubens_earlylog_current);
	cursor = readl(header + 12);
	/* Keep the previous slot authoritative if reset interrupts this write. */
	writel(0, header + 24);
	for (i = 0; i < len; i++)
		writeb(buf[i], ring + ((cursor + i) % RUBENS_EARLYLOG_RING));
	writel(cursor + len, header + 12);
	writel(readl(header + 16) + len, header + 16);
	writel(RUBENS_EARLYLOG_COMMIT, header + 24);
	raw_spin_unlock_irqrestore(&rubens_earlylog_lock, flags);

	atomic_set(&rubens_earlylog_busy, 0);
}

void rubens_earlylog_printk(const char *fmt, va_list args)
{
	char buf[RUBENS_EARLYLOG_MAX_MSG];
	va_list copy;
	int len;

	if (!rubens_earlylog_base)
		return;
	va_copy(copy, args);
	len = vscnprintf(buf, sizeof(buf), fmt, copy);
	va_end(copy);
	if (len > 0)
		rubens_earlylog_write(buf, len);
}

void rubens_earlylog_stage(u32 stage)
{
	if (rubens_earlylog_base) {
		writel(stage, rubens_earlylog_header(rubens_earlylog_current) + 20);
	}
}

static size_t rubens_earlylog_copy_slot(unsigned int slot, char *buf, size_t size)
{
	void __iomem *header = rubens_earlylog_header(slot);
	void __iomem *ring = rubens_earlylog_ring(slot);
	u32 cursor = readl(header + 12);
	u32 total = readl(header + 16);
	size_t len = min_t(size_t, min_t(u32, total, RUBENS_EARLYLOG_RING),
				  size);
	u32 start;
	size_t i;

	start = cursor % RUBENS_EARLYLOG_RING;
	start = (start + RUBENS_EARLYLOG_RING - len) % RUBENS_EARLYLOG_RING;
	for (i = 0; i < len; i++)
		buf[i] = readb(ring + ((start + i) % RUBENS_EARLYLOG_RING));
	return len;
}

size_t rubens_earlylog_copy_previous(char *buf, size_t size)
{
	if (!rubens_earlylog_base || rubens_earlylog_previous == UINT_MAX)
		return 0;
	return rubens_earlylog_copy_slot(rubens_earlylog_previous, buf, size);
}

void __init rubens_earlylog_early_init(void)
{
	u32 generation[2] = { 0, 0 };
	bool valid[2];

	raw_spin_lock_init(&rubens_earlylog_lock);
	if (memblock_reserve(RUBENS_EARLYLOG_PA, RUBENS_EARLYLOG_SIZE)) {
		pr_err("rubens-earlylog: cannot reserve 0x%08lx+0x%lx\n",
		       RUBENS_EARLYLOG_PA, RUBENS_EARLYLOG_SIZE);
		return;
	}
	rubens_earlylog_base = early_ioremap(RUBENS_EARLYLOG_PA, RUBENS_EARLYLOG_SIZE);
	if (!rubens_earlylog_base) {
		pr_err("rubens-earlylog: early_ioremap 0x%08lx+0x%lx failed\n",
		       RUBENS_EARLYLOG_PA, RUBENS_EARLYLOG_SIZE);
		return;
	}

	valid[0] = rubens_earlylog_valid(0, &generation[0]);
	valid[1] = rubens_earlylog_valid(1, &generation[1]);
	if (valid[0] && valid[1]) {
		rubens_earlylog_previous = generation[0] >= generation[1] ? 0 : 1;
		rubens_earlylog_current = 1 - rubens_earlylog_previous;
	} else if (valid[0]) {
		rubens_earlylog_previous = 0;
		rubens_earlylog_current = 1;
	} else if (valid[1]) {
		rubens_earlylog_previous = 1;
		rubens_earlylog_current = 0;
	}

	rubens_earlylog_start();
	pr_info("rubens-earlylog: slot %u armed at 0x%08lx\n",
		rubens_earlylog_current, RUBENS_EARLYLOG_PA);
}

void __init rubens_earlylog_late_init(void)
{
	void __iomem *early = rubens_earlylog_base;
	void __iomem *perm;

	if (!rubens_earlylog_base)
		return;
	/* The region is reserved/no-map on the Rubens DT. Use a permanent
	 * uncached mapping instead of creating a cached alias to DRAM. */
	perm = ioremap(RUBENS_EARLYLOG_PA, RUBENS_EARLYLOG_SIZE);
	if (!perm)
		return;
	rubens_earlylog_base = perm;
	/* Release the setup_arch fixmap slot before early_ioremap_reset();
	 * otherwise it is reported as an early ioremap leak. */
	early_iounmap(early, RUBENS_EARLYLOG_SIZE);
}
