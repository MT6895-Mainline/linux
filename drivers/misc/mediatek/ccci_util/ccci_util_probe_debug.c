// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: first observable behaviour of the ported ccci_util.
 *
 * This runs at late_initcall and is strictly read-only bookkeeping: it asks
 * the kernel resource tree what it already believes about the DRAM regions
 * the device firmware reserves for the modem, and it asks memremap() whether
 * a linear/WB mapping can be established. It never touches MMIO (every range
 * here is DRAM, and no register is read), and it never dereferences a mapping
 * it creates - the returned pointer is only printed.
 *
 * The list mirrors the reserved-memory nodes in qqcandy-mblock.dtsi (and the
 * stock firmware DT in .qqcandy-ref/stock-fdt.dts):
 *
 *   ccci_tag_mem   0x00000000bdbf0000 + 0x00010000
 *   ap_md_c_smem   0x0000000088000000 + 0x05060000
 *   ap_md_nc_smem  0x000000008e000000 + 0x00120000
 *   md_mem_usage   0x00000000d0000000 + 0x024b0000
 *   md_mem_usage   0x00000000d3000000 + 0x00230000
 *   md_mem_usage   0x00000000d4000000 + 0x08da0000
 *   md_mem_usage   0x00000000ed600000 + 0x00a00000
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>	/* REGION_* and region_intersects() */
#include <linux/types.h>

struct ccci_probe_region {
	const char *name;
	phys_addr_t base;
	resource_size_t size;
};

static const struct ccci_probe_region ccci_probe_regions[] = {
	{ "ccci_tag_mem",  0x00000000bdbf0000ULL, 0x00010000ULL },
	{ "ap_md_c_smem",  0x0000000088000000ULL, 0x05060000ULL },
	{ "ap_md_nc_smem", 0x000000008e000000ULL, 0x00120000ULL },
	{ "md_mem_usage0", 0x00000000d0000000ULL, 0x024b0000ULL },
	{ "md_mem_usage1", 0x00000000d3000000ULL, 0x00230000ULL },
	{ "md_mem_usage2", 0x00000000d4000000ULL, 0x08da0000ULL },
	{ "md_mem_usage3", 0x00000000ed600000ULL, 0x00a00000ULL },
};

/* region_intersects() result -> short string */
static const char *ccci_probe_region_str(int ret)
{
	switch (ret) {
	case REGION_DISJOINT:
		return "DISJOINT";
	case REGION_INTERSECTS:
		return "INTERSECTS";
	case REGION_MIXED:
		return "MIXED";
	default:
		return "?";
	}
}

struct ccci_probe_res {
	const char *name;
	unsigned long flags;
	bool found;
};

/* First matching resource in the walk is the one that covers the region. */
static int ccci_probe_res_cb(struct resource *res, void *arg)
{
	struct ccci_probe_res *info = arg;

	if (!info->found) {
		info->name = res->name ? res->name : "unnamed";
		info->flags = res->flags;
		info->found = true;
	}
	return 0;
}

static int __init ccci_util_probe_resource_tree(void)
{
	int i;

	pr_info("CCCI-PROBE: resource-tree diagnostic start (read-only, no MMIO, no deref)\n");

	for (i = 0; i < ARRAY_SIZE(ccci_probe_regions); i++) {
		const struct ccci_probe_region *r = &ccci_probe_regions[i];
		struct ccci_probe_res info = {
			.name = "none", .flags = 0, .found = false,
		};
		phys_addr_t base = r->base;
		void *map = NULL;
		void *map_log = NULL;
		const char *map_str;
		int sysram, memres;

		/*
		 * Two independent queries: is the range seen as System RAM,
		 * and is it seen as any IORESOURCE_MEM resource at all.
		 */
		sysram = region_intersects(r->base, r->size,
					   IORESOURCE_SYSTEM_RAM,
					   IORES_DESC_NONE);
		memres = region_intersects(r->base, r->size,
					   IORESOURCE_MEM, IORES_DESC_NONE);

		/* Name/flags of the covering resource, for the "reserved" bit. */
		walk_iomem_res_desc(IORES_DESC_NONE, IORESOURCE_MEM,
				    r->base, r->base + r->size - 1,
				    &info, ccci_probe_res_cb);

		/*
		 * memremap() only validates and maps. It is skipped for a
		 * MIXED range because the implementation WARNs and returns
		 * NULL there; skipping keeps a normal boot warning-free.
		 * The returned pointer is only printed, never read through.
		 */
		if (sysram != REGION_MIXED) {
			map = memremap(r->base, r->size, MEMREMAP_WB);
			map_log = map;
			map_str = map ? "OK" : "NULL";
			if (map)
				memunmap(map);
		} else {
			map_str = "skipped(MIXED)";
		}

		pr_info("CCCI-PROBE: %-14s pa=%pa size=0x%llx sysram=%s mem=%s res=%s flags=0x%lx busy=%s memremap=%s ptr=%px\n",
			r->name, &base, (unsigned long long)r->size,
			ccci_probe_region_str(sysram),
			ccci_probe_region_str(memres),
			info.name, info.flags,
			(info.flags & IORESOURCE_BUSY) ? "yes" : "no",
			map_str, map_log);
	}

	pr_info("CCCI-PROBE: resource-tree diagnostic done\n");

	return 0;
}
late_initcall(ccci_util_probe_resource_tree);
