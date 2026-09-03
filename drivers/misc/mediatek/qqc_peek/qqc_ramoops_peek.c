// SPDX-License-Identifier: GPL-2.0
/*
 * qqc_ramoops_peek — v842 read-back helper for the 5.10 reference capture.
 *
 * Reads (only reads) the ramoops region that the stock Android cmdline
 * declares (ramoops.mem_address=0x48090000 mem_size=0xe0000
 * pmsg_size=0x10000) and writes a snapshot to a file so it can be analysed
 * from Arch.  Used because our 6.18 kernel's pstore did not publish the
 * previous boot's console-ramoops entry.
 *
 * Safety: memremap() + memcpy_fromio() only.  No MMIO writes, no partition
 * writes, no MD-private DRAM.
 *
 * Usage:
 *   insmod qqc_ramoops_peek.ko            # default 0x48090000 / 0xe0000
 *   insmod qqc_ramoops_peek.ko addr=0x48090000 size=0xe0000 out=/root/ramoops.bin
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

static unsigned long long addr = 0x48090000ULL;
static unsigned int size = 0xe0000;
static char *out = "/root/ramoops_snapshot.bin";

module_param(addr, ullong, 0400);
MODULE_PARM_DESC(addr, "physical address of the ramoops region");
module_param(size, uint, 0400);
MODULE_PARM_DESC(size, "size of the ramoops region");
module_param(out, charp, 0400);
MODULE_PARM_DESC(out, "output file path");

static int __init qqc_ramoops_peek_init(void)
{
	void __iomem *map;
	void *buf;
	struct file *f;
	unsigned int nonzero = 0, i;
	loff_t pos = 0;
	ssize_t wr;

	buf = vzalloc(size);
	if (!buf)
		return -ENOMEM;

	map = memremap(addr, size, MEMREMAP_WB);
	if (!map) {
		pr_err("QQC-RAMOOPS: memremap %#llx/%#x failed\n", addr, size);
		vfree(buf);
		return -ENOMEM;
	}
	memcpy_fromio(buf, map, size);
	memunmap(map);

	for (i = 0; i < size; i++)
		if (((u8 *)buf)[i])
			nonzero++;

	pr_info("QQC-RAMOOPS: %#llx/%#x nonzero=%u first16=%*phN\n",
		addr, size, nonzero, 16, buf);

	f = filp_open(out, O_WRONLY | O_CREAT | O_TRUNC | O_LARGEFILE, 0600);
	if (IS_ERR(f)) {
		pr_err("QQC-RAMOOPS: open %s failed %ld\n", out, PTR_ERR(f));
		vfree(buf);
		return PTR_ERR(f);
	}
	wr = kernel_write(f, buf, size, &pos);
	pr_info("QQC-RAMOOPS: wrote %zd bytes to %s\n", wr, out);
	filp_close(f, NULL);

	vfree(buf);
	return 0;
}

static void __exit qqc_ramoops_peek_exit(void)
{
	pr_info("QQC-RAMOOPS: exit\n");
}

module_init(qqc_ramoops_peek_init);
module_exit(qqc_ramoops_peek_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("qqcandy read-only ramoops region peek (v842)");
