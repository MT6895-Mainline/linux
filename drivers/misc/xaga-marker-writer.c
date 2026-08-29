// SPDX-License-Identifier: GPL-2.0
/*
 * xaga boot-stage marker writer (XAGR ring), built into the kernel.
 *
 * xaga (Redmi Note 11T Pro / POCO X4 GT / Redmi K50i, MT6895) boot trace.
 * Arms a 64KB "XAGR" header + circular text ring in the log_store reserved
 * DRAM region (0x7ffbf000) at the head of arm64 setup_arch - the earliest
 * point the arm64 MMU fixmap makes the region writable - and mirrors every
 * printk() (via vprintk_emit) into the ring. Markers survive an AP watchdog
 * reboot in DRAM; LK's PL_LOG_STORE restores this region into the expdb
 * partition on the next boot, so a boot hang can be located even when the
 * kernel dies before any console is up.
 *
 * Layout matches the reader (lineage_xaga drivers/misc/xaga-marker.c):
 *   u32 magic @0x0000, u32 cursor @0x0004, u32 total @0x0008,
 *   u32 stage @0x1000, text ring @0x2000 (0xE000 bytes).
 *
 * The ring lives in log_store (0x7ffbf000), NOT minirdump (0x48170000):
 * writing minirdump triggers MTK's mrdump machinery and reboots the device
 * immediately (device findings 2026-08-09). log_store is a non-secure
 * reserved area not managed by mrdump/aee, and LK's PL_LOG_STORE dumps it
 * into expdb on every boot.
 *
 * The early_ioremap() mapping used for the setup_arch window is a fixmap
 * slot that paging_init()/early_ioremap_reset() invalidate; a permanent
 * memremap() mapping is established once paging_init is done so the mirror
 * keeps writing for the whole boot (and the panic tail is captured).
 *
 * Built-in (it was a module until the vendor-ramdisk module never wrote -
 * never confirmed loaded): CONFIG_XAGA_MARKER_WRITER is set only by the xaga
 * defconfig fragment; other devices leave it off. The module-load notifier
 * still logs every later module load, so a hang in a vendor module probe
 * leaves that module's name as the last ring entry.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memremap.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/printk.h>
#include <linux/xaga_marker.h>

#include <asm/cacheflush.h>

/* log_store reserved region: non-secure, survives the WDT reboot in DRAM */
#define XAGA_MRDUMP_PA	0x7ffbf000UL
#define XAGA_MRDUMP_SZ	0x10000UL
#define XAGA_RING_OFF	0x2000U
#define XAGA_RING_SZ	0xE000U
#define XAGA_MAGIC	0x52474158UL	/* "XAGR" */
#define XAGA_MAX_MSG	256

static void __iomem *xaga_mr_base;
static bool xaga_early_map;	/* still using the early_ioremap fixmap slot */
static bool xaga_direct_map;	/* using the direct map (needs cache clean) */

static void xaga_marker_ring_write(const char *buf, int n)
{
	void __iomem *ring;
	u32 cursor;
	int i;

	if (!xaga_mr_base)
		return;
	/* Re-assert our magic on every write: MTK aee/mrdump_mini may rewrite
	 * the region header; the next write restores it. */
	writel(XAGA_MAGIC, xaga_mr_base + 0x0000);
	cursor = readl(xaga_mr_base + 0x0004);
	ring = xaga_mr_base + XAGA_RING_OFF;
	for (i = 0; i < n; i++)
		writeb(buf[i], ring + ((cursor + i) % XAGA_RING_SZ));
	writel(cursor + n, xaga_mr_base + 0x0004);
	writel(readl(xaga_mr_base + 0x0008) + n, xaga_mr_base + 0x0008);
	/*
	 * The direct-map alias is write-back cached. A WDT hard reset does
	 * NOT flush the CPU cache, so push the writes to DRAM now or the
	 * ring is lost before LK can restore it into expdb.
	 */
	if (xaga_direct_map)
		dcache_clean_poc((unsigned long)xaga_mr_base,
				 (unsigned long)xaga_mr_base + XAGA_MRDUMP_SZ);
}

void xaga_marker_put(const char *fmt, ...)
{
	va_list args;
	char buf[XAGA_MAX_MSG];
	int n;

	va_start(args, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (n <= 0)
		return;
	xaga_marker_ring_write(buf, n);
}
EXPORT_SYMBOL_GPL(xaga_marker_put);

void xaga_marker_stage(u32 stage)
{
	if (!xaga_mr_base)
		return;
	writel(stage, xaga_mr_base + 0x1000);
	xaga_marker_put("stage=%u\n", stage);
}
EXPORT_SYMBOL_GPL(xaga_marker_stage);

/* Mirrors every printk() into the ring while armed; called from
 * vprintk_emit. Must be safe in any printk context: no printk, no locks, no
 * allocation. The ring is lock-free: concurrent writers may occasionally
 * interleave, acceptable for a diagnostic ring. Every 64th message also
 * writes a MIRROR:n heartbeat so the LK log_store recovery (which dumps
 * this region into expdb on the next boot) proves the mirror is live. */
static unsigned int xaga_mirror_cnt;

void xaga_marker_early_printk(const char *fmt, va_list args)
{
	va_list ap;
	char buf[XAGA_MAX_MSG];
	int n;

	if (!xaga_mr_base)
		return;
	va_copy(ap, args);
	n = vscnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n <= 0)
		return;
	xaga_marker_ring_write(buf, n);
	if (++xaga_mirror_cnt % 64 == 0) {
		char hb[32];
		int hn = snprintf(hb, sizeof(hb), "MIRROR:%u\n", xaga_mirror_cnt);

		xaga_marker_ring_write(hb, hn);
	}
}

/* Called from the head of arm64 setup_arch, right after
 * early_fixmap_init()/early_ioremap_init() - the earliest point the arm64
 * MMU maps the reserved region (it is not in the linear map before
 * paging_init). Everything printed from here on lands in the ring. */
void __init xaga_marker_early_init(void)
{
	xaga_mr_base = early_ioremap(XAGA_MRDUMP_PA, XAGA_MRDUMP_SZ);
	if (!xaga_mr_base) {
		pr_info("xaga-marker-writer: early_ioremap 0x%08lx failed\n",
			XAGA_MRDUMP_PA);
		return;
	}
	xaga_early_map = true;
	/* fresh ring per boot: only the last boot's markers survive */
	writel(XAGA_MAGIC, xaga_mr_base + 0x0000);
	writel(0, xaga_mr_base + 0x0004);
	writel(0, xaga_mr_base + 0x0008);
	writel(0, xaga_mr_base + 0x1000);
	pr_info("xaga-marker-writer: XAGR ring armed at 0x%08lx\n",
		XAGA_MRDUMP_PA);
	xaga_marker_stage(1);
}

static int xaga_marker_module_nb(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	struct module *mod = data;

	switch (action) {
	case MODULE_STATE_COMING:
	case MODULE_STATE_LIVE:
		xaga_marker_put("module: %s\n", mod->name);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block xaga_marker_nb = {
	.notifier_call = xaga_marker_module_nb,
};

static int __init xaga_marker_w_late_init(void)
{
	void *perm;

	/*
	 * Replace the setup_arch-era early_ioremap fixmap mapping (which
	 * paging_init()/early_ioremap_reset() invalidates) with a stable
	 * direct-map alias. log_store is reserved System RAM (not no-map),
	 * so memremap() returns the direct map pointer; the ring writes go
	 * through it and dcache_clean_poc() pushes them to DRAM so a WDT
	 * hard reset doesn't lose them. Only swap if still on the early
	 * slot; memremap() needs the real MM/paging_init.
	 */
	if (xaga_early_map) {
		perm = memremap(XAGA_MRDUMP_PA, XAGA_MRDUMP_SZ, MEMREMAP_WB);
		if (perm) {
			xaga_mr_base = perm;
			xaga_early_map = false;
			xaga_direct_map = true;
			dcache_clean_poc((unsigned long)xaga_mr_base,
					 (unsigned long)xaga_mr_base +
					 XAGA_MRDUMP_SZ);
			xaga_marker_put("marker writer: permanent mapping armed\n");
		}
	}
	xaga_marker_put("marker writer built-in init\n");
	register_module_notifier(&xaga_marker_nb);
	return 0;
}
core_initcall(xaga_marker_w_late_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("xaga boot-stage marker writer (XAGR ring), built-in");
