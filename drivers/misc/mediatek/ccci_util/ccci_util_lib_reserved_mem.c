// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>       /* min() */
#include <linux/memblock.h>
#include <linux/suspend.h>
#include <asm/cacheflush.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <asm/setup.h>
#include <linux/mm.h>
#include <asm/page.h>
#include <linux/atomic.h>
#include <linux/page_owner.h>
#include <linux/irq.h>
#include <linux/syscore_ops.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <linux/vmalloc.h>
#include <linux/page-flags.h>
#include <linux/mmzone.h>
#include <linux/cpu.h>
#include "ccci_util_lib_reserved_mem.h"


/**
 *	vmap_reserved_mem - map reserved memory into virtually contiguous space
 *	@start:		start of reserved memory
 *	@size:		size of reserved memory
 *	@prot:		page protection for the mapping
 */
void *vmap_reserved_mem(phys_addr_t start, phys_addr_t size, pgprot_t prot)
{
	long i;
	long page_count;
	unsigned long pfn;
	void *vaddr = NULL;
	phys_addr_t addr = start;
	struct page *page;
	struct page **pages;

	page_count = DIV_ROUND_UP(size, PAGE_SIZE);
	pages = vmalloc(page_count * sizeof(struct page *));

	if (!pages)
		return NULL;

	for (i = 0; i < page_count; i++) {
		pfn = __phys_to_pfn(addr);
		page = pfn_to_page(pfn);
		pages[i] = page;
		addr += PAGE_SIZE;
	}

	vaddr = vmap(pages, page_count, VM_MAP, prot);
	vfree(pages);
	return vaddr;
}
EXPORT_SYMBOL(vmap_reserved_mem);

/*
 * qqcandy: a "no-map" reserved-memory node describes an address range that is
 * NOT backed by RAM (a hardware hole).  Handing such pages to the buddy
 * allocator is fatal: the UFS driver can then pick them up for a scatterlist
 * and arch_sync_dma_for_device()/dcache_clean_poc() will touch an address that
 * does not exist.  Observed as:
 *
 *   Internal error: Oops: 0000000096000147 [#1] SMP
 *   pc : dcache_clean_poc+0x20/0x38
 *     dma_direct_map_sg -> scsi_dma_map -> ufshcd_queuecommand
 *     read_pages -> page_cache_ra_order -> filemap_fault
 *   x26/x21 = 0xbdbc2000   (the hole around mblock-29-ccci_tag_mem)
 *
 * mblock-29-ccci_tag_mem (0xbdbf0000, 64 KiB) is declared
 * "nomap non-reusable" in qqcandy-mblock.dtsi, yet ccci_util_lib_fo.c's
 * dump_retrieve_info() may still route it here through the LK "retrieve"
 * list, so refuse to release any range that overlaps a no-map node.
 */
static bool range_hits_nomap_reserved(phys_addr_t start, phys_addr_t end)
{
	struct device_node *rm, *child;

	rm = of_find_node_by_path("/reserved-memory");
	if (!rm)
		return false;

	for_each_child_of_node(rm, child) {
		struct resource res;

		if (!of_property_read_bool(child, "no-map"))
			continue;
		if (of_address_to_resource(child, 0, &res))
			continue;
		if (start < (phys_addr_t)res.end + 1 &&
		    end > (phys_addr_t)res.start) {
			pr_warn("%s: refusing to free %pa..%pa (inside no-map %pOF)\n",
				__func__, &start, &end, child);
			of_node_put(child);
			of_node_put(rm);
			return true;
		}
	}

	of_node_put(rm);
	return false;
}

int free_reserved_memory(phys_addr_t start_phys,
				phys_addr_t end_phys)
{

	phys_addr_t pos;
	unsigned long pages = 0;

	if (end_phys <= start_phys) {

		pr_notice("%s end_phys is smaller than start_phys start_phys:0x%pa end_phys:0x%pa\n"
			, __func__, &start_phys, &end_phys);
		return -1;
	}

	if (range_hits_nomap_reserved(start_phys, end_phys))
		return -1;

	for (pos = start_phys; pos < end_phys; pos += PAGE_SIZE, pages++)
		free_reserved_page(phys_to_page(pos));

	if (pages)
		pr_info("Freeing reserved memory: %ldK from phys %llx\n",
			pages << (PAGE_SHIFT - 10),
			(unsigned long long)start_phys);

	return 0;
}
EXPORT_SYMBOL(free_reserved_memory);
