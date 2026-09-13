// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: read-only SMEM/CCB layout forensics for the CCCI bring-up.
 *
 * Loading only checks the DT/stash tag header (same discipline as
 * ccci_probe). A separate runtime write may queue one dump per module load.
 * The dump maps the tag region, re-parses the chain, rebuilds the official
 * SMEM region table merged with the LK overrides, and then walks the SMEM
 * and CCB reserved regions strictly read-only: no writes, no MMIO, no
 * clock/SMC involvement - both regions are DRAM reserved-memory. Before any
 * dereference the physical base/size must match the ground truth recorded
 * from the device resource tree, otherwise the run fails closed.
 *
 * Even a read-only module can hang the SoC; neither a workqueue nor
 * cancel_work_sync() isolates a bus fault.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "../ccci_util/ccci_tag_parse.h"
#include "ccci_smem_layout.h"

#ifndef MODULE
#error "The CCCI SMEM dump must only be built as a module"
#endif

#define CCCI_TAG_MEM_BASE	0x00000000bdbf0000ULL
#define CCCI_TAG_MEM_SIZE	0x00010000U

/*
 * Ground truth from the device resource tree (HANDOFF §80.7/§80.12):
 * ap_md_nc_smem for the SMEM, and the CCB slice inside ap_md_c_smem
 * (0x88000000 + 0x5060000). Anything else fails closed.
 */
#define CCCI_SMEM_BASE		0x000000008e000000ULL
#define CCCI_SMEM_MAX_SIZE	0x00120000U
#define CCCI_CCB_BASE		0x0000000089000000ULL
#define CCCI_CCB_MAX_SIZE	0x04000000U

#define CCCI_NC_NODE_MAX	64u
#define CCCI_OV_MAX		16u
#define CCCI_CSMEM_MAX		16u

/* Verbatim property bytes captured before the embedded DTB takes over. */
extern u8 xaga_ccci_lk_prop[64];
extern int xaga_ccci_lk_prop_len;
extern char xaga_ccci_lk_prop_name[32];

/* Payload views collected during the tag walk, interpreted afterwards. */
struct ccci_smem_tag_view {
	const void *data;
	unsigned int size;
};

struct ccci_smem_tag_ctx {
	const void *base;	/* tag region mapping */
	unsigned int count;	/* tags walked */
	struct ccci_smem_tag_view nc_ext;	/* nc_smem_info_ext */
	struct ccci_smem_tag_view nc_ext_num;	/* nc_smem_info_ext_num */
	struct ccci_smem_tag_view lk_nc;	/* nc_smem_layout (full LK table) */
	struct ccci_smem_tag_view lk_nc_num;	/* nc_smem_layout_num */
	struct ccci_smem_tag_view csmem_info;	/* md1_bank4_cache_info */
	struct ccci_smem_tag_view csmem_layout;	/* md1_bank4_cache_layout */
	struct ccci_smem_tag_view ccb_gear_id;	/* ccb_gear_id */
	struct ccci_smem_tag_view cache_offset;	/* md1_smem_cahce_offset */
};

enum ccci_smem_dump_state {
	CCCI_SMEM_DUMP_UNAVAILABLE,
	CCCI_SMEM_DUMP_READY,
	CCCI_SMEM_DUMP_QUEUED,
	CCCI_SMEM_DUMP_RUNNING,
	CCCI_SMEM_DUMP_DONE,
	CCCI_SMEM_DUMP_FAILED,
};

static struct ccci_tag_hdr ccci_smem_dump_header;
static enum ccci_smem_dump_state ccci_smem_dump_state = CCCI_SMEM_DUMP_UNAVAILABLE;
static DEFINE_MUTEX(ccci_smem_dump_lock);
static int ccci_smem_dump_last_result = -ENODATA;
static bool ccci_smem_dump_trigger;

static int ccci_smem_dump_read_header(struct ccci_tag_hdr *hdr,
				      const char **source)
{
	struct device_node *node;
	const void *raw = NULL;
	int len = 0, ret = -ENODEV;

	*source = "none";
	node = of_find_compatible_node(NULL, NULL, "mediatek,mddriver");
	if (node) {
		raw = of_get_property(node, "ccci,modem_info_v2", &len);
		if (raw) {
			*source = "runtime DT";
			if (len < (int)sizeof(*hdr)) {
				ret = -EMSGSIZE;
			} else {
				memcpy(hdr, raw, sizeof(*hdr));
				ret = ccci_validate_tag_hdr(hdr, CCCI_TAG_MEM_BASE,
							    CCCI_TAG_MEM_SIZE);
			}
		}
		of_node_put(node);
		/* A malformed present property is an error, not a fallback. */
		if (raw)
			return ret;
	}

	if (!xaga_ccci_lk_prop_len)
		return -ENODEV;
	if (strcmp(xaga_ccci_lk_prop_name, "ccci,modem_info_v2"))
		return -EOPNOTSUPP;
	if (xaga_ccci_lk_prop_len < 0 ||
	    xaga_ccci_lk_prop_len > (int)sizeof(xaga_ccci_lk_prop))
		return -EMSGSIZE;
	if (xaga_ccci_lk_prop_len < (int)sizeof(*hdr))
		return -EMSGSIZE;
	*source = "LKINFO stash";
	memcpy(hdr, xaga_ccci_lk_prop, sizeof(*hdr));
	return ccci_validate_tag_hdr(hdr, CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE);
}

/*
 * Walk the raw tag chain for payloads the shared parser does not interpret.
 * Mirrors ccci_parse_tag_chain()'s semantics exactly: tags start at offset 0,
 * offsets strictly increase, exactly hdr->tag_num tags are walked, and the
 * last tag's next_tag_offset may be zero or a bounded forward terminator.
 */
static int ccci_smem_walk_tags(const void *buf, const struct ccci_tag_hdr *hdr,
			       int (*fn)(const struct ccci_tag *tag,
					 unsigned int offset, void *ctx),
			       void *ctx)
{
	unsigned int offset = 0, i;

	for (i = 0; i < (unsigned int)hdr->tag_num; i++) {
		const struct ccci_tag *tag;
		unsigned int next;
		int ret;

		if (offset + sizeof(*tag) > hdr->size)
			return -EMSGSIZE;
		tag = buf + offset;
		if (!memchr(tag->tag_name, 0, sizeof(tag->tag_name)))
			return -EBADMSG;
		if (tag->data_offset > hdr->size ||
		    tag->data_size > hdr->size - tag->data_offset)
			return -ERANGE;
		ret = fn(tag, offset, ctx);
		if (ret)
			return ret;

		next = tag->next_tag_offset;
		/* Last tag: next_tag_offset may be zero or forward pointer. */
		if (i + 1 == (unsigned int)hdr->tag_num && !next)
			break;
		if (next <= offset || next - offset < sizeof(*tag))
			return -EBADMSG;
		if (next > hdr->size)
			return -ERANGE;
		offset = next;
	}
	return 0;
}

static int ccci_smem_collect_tag(const struct ccci_tag *tag, unsigned int offset,
				 void *ctx)
{
	struct ccci_smem_tag_ctx *tc = ctx;
	struct ccci_smem_tag_view *view = NULL;

	pr_info("CCCI-SMEM: tag[%u] off=0x%x name=\"%s\" data=0x%x/0x%x\n",
		tc->count++, offset, tag->tag_name, tag->data_offset,
		tag->data_size);

	if (!strcmp(tag->tag_name, "nc_smem_info_ext"))
		view = &tc->nc_ext;
	else if (!strcmp(tag->tag_name, "nc_smem_info_ext_num"))
		view = &tc->nc_ext_num;
	else if (!strcmp(tag->tag_name, "nc_smem_layout"))
		view = &tc->lk_nc;
	else if (!strcmp(tag->tag_name, "nc_smem_layout_num"))
		view = &tc->lk_nc_num;
	else if (!strcmp(tag->tag_name, "md1_bank4_cache_info"))
		view = &tc->csmem_info;
	else if (!strcmp(tag->tag_name, "md1_bank4_cache_layout"))
		view = &tc->csmem_layout;
	else if (!strcmp(tag->tag_name, "ccb_gear_id"))
		view = &tc->ccb_gear_id;
	else if (!strcmp(tag->tag_name, "md1_smem_cahce_offset"))
		view = &tc->cache_offset;	/* sic, official tag name */

	if (view) {
		view->data = (const char *)tc->base + tag->data_offset;
		view->size = tag->data_size;
	}
	return 0;
}

static bool ccci_smem_view_u32(const struct ccci_smem_tag_view *v,
			       unsigned int *out)
{
	if (!v->data || v->size < sizeof(*out))
		return false;
	memcpy(out, v->data, sizeof(*out));
	return true;
}

static unsigned int ccci_smem_view_count(const struct ccci_smem_tag_view *num_view,
					 const struct ccci_smem_tag_view *arr_view,
					 size_t elem_size, unsigned int cap,
					 const char *what)
{
	unsigned int num = 0, avail;

	if (!ccci_smem_view_u32(num_view, &num)) {
		pr_info("CCCI-SMEM: %s: count tag missing; skipping\n", what);
		return 0;
	}
	avail = arr_view->size ? (unsigned int)(arr_view->size / elem_size) : 0;
	if (num > cap || num > avail) {
		unsigned int capped = min3(num, cap, avail);

		pr_info("CCCI-SMEM: %s: num=%u capped to %u (cap=%u avail=%u)\n",
			what, num, capped, cap, avail);
		num = capped;
	}
	return num;
}

static void ccci_smem_print_overrides(const char *what,
				      const struct ccci_smem_override *ov,
				      unsigned int num)
{
	unsigned int i;

	pr_info("CCCI-SMEM: %s: %u entries\n", what, num);
	for (i = 0; i < num; i++)
		pr_info("CCCI-SMEM:   [%2u] id=%u ap=0x%08x md=0x%08x size=0x%08x\n",
			i, ov[i].id, ov[i].ap_offset, ov[i].md_offset,
			ov[i].size);
}

static void ccci_smem_print_table(const struct ccci_smem_region_tbl *tbl,
				  unsigned int n, const char *what)
{
	unsigned int i;

	pr_info("CCCI-SMEM: %s: %u regions\n", what, n);
	for (i = 0; i < n; i++) {
		if (tbl[i].size == 0 && tbl[i].offset == 0)
			continue;
		pr_info("CCCI-SMEM:   [%2u] %-16s id=%2u off=0x%06x size=0x%06x\n",
			i, tbl[i].name, tbl[i].id, tbl[i].offset, tbl[i].size);
	}
}

/* Guarded u32 read from a WB mapping; *ok=0 means out of range. */
static unsigned int ccci_smem_read_u32(const void *base, size_t map_size,
				       size_t off, int *ok)
{
	unsigned int v = 0;

	*ok = 0;
	if (map_size >= sizeof(v) && off <= map_size - sizeof(v)) {
		memcpy(&v, (const char *)base + off, sizeof(v));
		*ok = 1;
	}
	return v;
}

static void ccci_smem_dump_hex32(const void *base, size_t map_size, size_t off,
				 unsigned int words, const char *label)
{
	unsigned int row;

	for (row = 0; row < words; row += 8) {
		char line[80];
		size_t used = 0;
		unsigned int k;

		line[0] = '\0';
		for (k = row; k < words && k < row + 8; k++) {
			int ok;
			unsigned int v = ccci_smem_read_u32(base, map_size,
							    off + k * 4, &ok);

			used += scnprintf(line + used, sizeof(line) - used,
					  ok ? "%08x " : "xxxxxxxx ", v);
		}
		pr_info("CCCI-SMEM: %s+0x%02zx: %s\n", label, off + row * 4,
			line);
	}
}

static void ccci_smem_dump_work_fn(struct work_struct *work)
{
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_tag_ctx tc = {};
	struct ccci_tag_result res = {};
	struct ccci_smem_override ov[CCCI_OV_MAX];
	struct ccci_smem_csmem_item csmem[CCCI_CSMEM_MAX];
	struct ccci_smem_csmem_item csmem_info = {};
	unsigned int ov_num, lk_num, csmem_num = 0;
	size_t smem_size = 0, ccb_size = 0;
	void *tag_map, *smem_map = NULL, *ccb_map = NULL;
	unsigned int i;
	int ret;

	mutex_lock(&ccci_smem_dump_lock);
	ccci_smem_dump_state = CCCI_SMEM_DUMP_RUNNING;
	mutex_unlock(&ccci_smem_dump_lock);

	pr_info("CCCI-SMEM: step 1: map tag region pa=0x%llx size=0x%x WB\n",
		CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE);
	tag_map = memremap(CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE, MEMREMAP_WB);
	if (!tag_map) {
		ret = -ENOMEM;
		goto out_unmap;
	}
	tc.base = tag_map;

	pr_info("CCCI-SMEM: step 2: parse tag chain, size=0x%x count=%d\n",
		ccci_smem_dump_header.size, ccci_smem_dump_header.tag_num);
	ret = ccci_parse_tag_chain(&ccci_smem_dump_header, tag_map,
				   ccci_smem_dump_header.size, &res);
	if (ret) {
		pr_info("CCCI-SMEM: parse failed: ret=%d walked=%u; layout not usable\n",
			ret, res.tags_walked);
		goto out_unmap;
	}

	ret = ccci_smem_walk_tags(tag_map, &ccci_smem_dump_header,
				  ccci_smem_collect_tag, &tc);
	if (ret) {
		pr_info("CCCI-SMEM: tag walk failed: ret=%d\n", ret);
		goto out_unmap;
	}

	if (res.smem_found)
		pr_info("CCCI-SMEM: smem_layout: base=0x%llx total=0x%x ap_md1=+0x%x/0x%x\n",
			res.smem.base_addr, res.smem.total_smem_size,
			res.smem.ap_md1_smem_offset, res.smem.ap_md1_smem_size);
	if (res.ccb_found)
		pr_info("CCCI-SMEM: ccb_info: addr=0x%llx size=0x%x\n",
			res.ccb.addr, res.ccb.size);

	/* LK's own full region table (raw, uninterpreted), streamed from the
	 * mapped tag payload to keep the work function's stack small.
	 */
	lk_num = ccci_smem_view_count(&tc.lk_nc_num, &tc.lk_nc,
				      sizeof(struct ccci_smem_override),
				      CCCI_NC_NODE_MAX, "nc_smem_layout");
	pr_info("CCCI-SMEM: nc_smem_layout (LK full table): %u entries\n",
		lk_num);
	for (i = 0; i < lk_num; i++) {
		struct ccci_smem_override node;

		memcpy(&node, (const char *)tc.lk_nc.data +
				   i * sizeof(node), sizeof(node));
		pr_info("CCCI-SMEM:   [%2u] id=%u ap=0x%08x md=0x%08x size=0x%08x\n",
			i, node.id, node.ap_offset, node.md_offset, node.size);
	}

	/* Overrides the official driver merges into the fat table. */
	ov_num = ccci_smem_view_count(&tc.nc_ext_num, &tc.nc_ext,
				      sizeof(ov[0]), CCCI_OV_MAX,
				      "nc_smem_info_ext");
	for (i = 0; i < ov_num; i++)
		memcpy(&ov[i], (const char *)tc.nc_ext.data +
				   i * sizeof(ov[0]), sizeof(ov[0]));
	ccci_smem_print_overrides("nc_smem_info_ext (overrides)", ov, ov_num);

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, ov_num);
	if (ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM,
				   CCCI_SMEM_MAX_SIZE)) {
		pr_info("CCCI-SMEM: built table exceeds SMEM 0x%x; not reading SMEM\n",
			CCCI_SMEM_MAX_SIZE);
		ret = -ERANGE;
		goto out_unmap;
	}
	ccci_smem_print_table(tbl, CCCI_SMEM_FAT_NUM, "built SMEM table");
	pr_info("CCCI-SMEM: built table span=0x%x of SMEM 0x%x\n",
		ccci_smem_layout_end(tbl, CCCI_SMEM_FAT_NUM),
		CCCI_SMEM_MAX_SIZE);

	/* Cacheable CCB sub-regions, straight from the LK tags. */
	if (tc.csmem_info.data && tc.csmem_info.size >= sizeof(csmem_info))
		memcpy(&csmem_info, tc.csmem_info.data, sizeof(csmem_info));
	if (tc.csmem_layout.data &&
	    csmem_info.item_cnt <= CCCI_CSMEM_MAX &&
	    (size_t)csmem_info.item_cnt * sizeof(csmem[0]) <= tc.csmem_layout.size)
		csmem_num = csmem_info.item_cnt;
	else if (tc.csmem_layout.data)
		pr_info("CCCI-SMEM: md1_bank4_cache_layout: info tag missing or cnt=%u out of range; skipping items\n",
			csmem_info.item_cnt);
	for (i = 0; i < csmem_num; i++)
		memcpy(&csmem[i], (const char *)tc.csmem_layout.data +
				      i * sizeof(csmem[0]), sizeof(csmem[0]));
	pr_info("CCCI-SMEM: md1_bank4_cache_info: addr=0x%llx size=0x%x cnt=%u\n",
		csmem_info.addr, csmem_info.size, csmem_info.item_cnt);
	for (i = 0; i < csmem_num; i++)
		pr_info("CCCI-SMEM:   csmem[%u] addr=0x%llx md_off=0x%x size=0x%x\n",
			i, csmem[i].addr, csmem[i].md_offset, csmem[i].size);
	{
		unsigned int gear = 0, coff = 0;

		if (ccci_smem_view_u32(&tc.ccb_gear_id, &gear))
			pr_info("CCCI-SMEM: ccb_gear_id=%u\n", gear);
		if (ccci_smem_view_u32(&tc.cache_offset, &coff))
			pr_info("CCCI-SMEM: md1_smem_cahce_offset=0x%x\n", coff);
	}

	/* ---- SMEM walk: DRAM only, read-only, guarded by ground truth ---- */
	if (res.smem.base_addr != CCCI_SMEM_BASE ||
	    !res.smem.total_smem_size ||
	    res.smem.total_smem_size > CCCI_SMEM_MAX_SIZE) {
		pr_info("CCCI-SMEM: SMEM guard failed: base=0x%llx total=0x%x (want base=0x%llx total<=0x%x); refusing to map\n",
			res.smem.base_addr, res.smem.total_smem_size,
			CCCI_SMEM_BASE, CCCI_SMEM_MAX_SIZE);
		ret = -ERANGE;
		goto out_unmap;
	}
	smem_size = res.smem.total_smem_size;
	pr_info("CCCI-SMEM: step 3: map SMEM pa=0x%llx size=0x%zx WB (read-only walk)\n",
		CCCI_SMEM_BASE, smem_size);
	smem_map = memremap(CCCI_SMEM_BASE, smem_size, MEMREMAP_WB);
	if (!smem_map) {
		ret = -ENOMEM;
		goto out_unmap;
	}

	{
		int idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
						SMEM_USER_RAW_MDCCCI_DBG);
		int ok;
		unsigned int v;

		if (idx >= 0 && tbl[idx].size >= 0x38) {
			v = ccci_smem_read_u32(smem_map, smem_size,
					       tbl[idx].offset + 0x34, &ok);
			pr_info("CCCI-SMEM: RAW_MDCCCI_DBG+0x34 (seqerr): %s0x%08x\n",
				ok ? "" : "unreadable ", ok ? v : 0);
		}
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_RAW_MDSS_DBG);
		if (idx >= 0 && tbl[idx].size >= 0x1c38) {
			size_t scan;
			unsigned int hits = 0;
			int scan_ok;

			v = ccci_smem_read_u32(smem_map, smem_size,
					       tbl[idx].offset + 0x1c34, &ok);
			pr_info("CCCI-SMEM: RAW_MDSS_DBG+0x1c34 (epof): %s0x%08x\n",
				ok ? "" : "unreadable ", ok ? v : 0);
			for (scan = 0; scan + 4 <= tbl[idx].size; scan += 4) {
				if (ccci_smem_read_u32(smem_map, smem_size,
						       tbl[idx].offset + scan,
						       &scan_ok) ==
					    0xBAEBAE10U && scan_ok) {
					pr_info("CCCI-SMEM: EPON magic 0xBAEBAE10 at SMEM+0x%zx\n",
						tbl[idx].offset + scan);
					if (++hits >= 4)
						break;
				}
			}
			if (!hits)
				pr_info("CCCI-SMEM: EPON magic 0xBAEBAE10 not found in RAW_MDSS_DBG\n");
		}
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_CCISM_MCU);
		if (idx >= 0 && tbl[idx].size >= 64)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset, 16, "CCISM_MCU");
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_CCISM_MCU_EXP);
		if (idx >= 0 && tbl[idx].size >= 64)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset, 16,
					     "CCISM_MCU_EXP");
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_RAW_CCB_CTRL);
		if (idx >= 0 && tbl[idx].size >= 64)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset, 16,
					     "RAW_CCB_CTRL");
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_RAW_RUNTIME_DATA);
		if (idx >= 0 && tbl[idx].size >= 32)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset, 8,
					     "RAW_RUNTIME_DATA");
	}
	memunmap(smem_map);
	smem_map = NULL;

	/* ---- CCB anchor: secondary; mismatch skips, not fails ---- */
	if (res.ccb_found && res.ccb.addr == CCCI_CCB_BASE &&
	    res.ccb.size && res.ccb.size <= CCCI_CCB_MAX_SIZE) {
		ccb_size = res.ccb.size;
		pr_info("CCCI-SMEM: step 4: map CCB pa=0x%llx size=0x%zx WB (read-only anchor)\n",
			CCCI_CCB_BASE, ccb_size);
		ccb_map = memremap(CCCI_CCB_BASE, ccb_size, MEMREMAP_WB);
		if (!ccb_map) {
			pr_info("CCCI-SMEM: CCB memremap failed; skipping\n");
		} else {
			unsigned int k;

			for (k = 0; k < csmem_num; k++) {
				unsigned long long a = csmem[k].addr;

				if (a >= CCCI_CCB_BASE &&
				    a - CCCI_CCB_BASE + 16 <= ccb_size)
					ccci_smem_dump_hex32(ccb_map, ccb_size,
							     a - CCCI_CCB_BASE,
							     4, "csmem");
			}
			ccci_smem_dump_hex32(ccb_map, ccb_size, 0, 8, "CCB");
			memunmap(ccb_map);
			ccb_map = NULL;
		}
	} else if (res.ccb_found) {
		pr_info("CCCI-SMEM: CCB guard mismatch (addr=0x%llx size=0x%x); skipping CCB\n",
			res.ccb.addr, res.ccb.size);
	}

	ret = 0;
	pr_info("CCCI-SMEM: complete: read-only dump done, nothing written\n");

out_unmap:
	if (smem_map)
		memunmap(smem_map);
	if (ccb_map)
		memunmap(ccb_map);
	if (tag_map)
		memunmap(tag_map);

	mutex_lock(&ccci_smem_dump_lock);
	ccci_smem_dump_last_result = ret;
	ccci_smem_dump_state = ret ? CCCI_SMEM_DUMP_FAILED :
				    CCCI_SMEM_DUMP_DONE;
	mutex_unlock(&ccci_smem_dump_lock);
}

static DECLARE_WORK(ccci_smem_dump_work, ccci_smem_dump_work_fn);

/*
 * Caller serializes state changes. Return 1 to queue, 0 for a no-op, or an
 * errno. COMING/GOING modules cannot arm a dump; writing 0 never resets it.
 */
static int ccci_smem_dump_request(enum ccci_smem_dump_state *state, bool live,
				  bool on)
{
	if (!on)
		return 0;
	if (!live)
		return -EAGAIN;
	if (*state == CCCI_SMEM_DUMP_UNAVAILABLE)
		return -ENODATA;
	if (*state != CCCI_SMEM_DUMP_READY)
		return -EALREADY;
	*state = CCCI_SMEM_DUMP_QUEUED;
	return 1;
}

static int ccci_smem_dump_trigger_set(const char *val,
				      const struct kernel_param *kp)
{
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;

	mutex_lock(&ccci_smem_dump_lock);
	ret = ccci_smem_dump_request(&ccci_smem_dump_state,
				     READ_ONCE(THIS_MODULE->state) ==
				     MODULE_STATE_LIVE, on);
	if (ret > 0) {
		ret = 0;
		*(bool *)kp->arg = true;
		ccci_smem_dump_last_result = -EINPROGRESS;
		if (!schedule_work(&ccci_smem_dump_work)) {
			ret = -EBUSY;
			ccci_smem_dump_last_result = ret;
			ccci_smem_dump_state = CCCI_SMEM_DUMP_FAILED;
		}
	}
	mutex_unlock(&ccci_smem_dump_lock);
	return ret;
}

static const struct kernel_param_ops ccci_smem_dump_trigger_ops = {
	.set = ccci_smem_dump_trigger_set,
	.get = param_get_bool,
};

module_param_cb(trigger, &ccci_smem_dump_trigger_ops, &ccci_smem_dump_trigger,
		0600);
MODULE_PARM_DESC(trigger,
		 "write 1 after loading to attempt one read-only SMEM/CCB dump; 0 does not arm or cancel");

static int ccci_smem_dump_status_get(char *buffer,
				     const struct kernel_param *kp)
{
	static const char * const names[] = {
		[CCCI_SMEM_DUMP_UNAVAILABLE] = "unavailable",
		[CCCI_SMEM_DUMP_READY] = "ready",
		[CCCI_SMEM_DUMP_QUEUED] = "queued",
		[CCCI_SMEM_DUMP_RUNNING] = "running",
		[CCCI_SMEM_DUMP_DONE] = "done",
		[CCCI_SMEM_DUMP_FAILED] = "failed",
	};
	int len;

	mutex_lock(&ccci_smem_dump_lock);
	len = scnprintf(buffer, PAGE_SIZE, "%s result=%d\n",
			names[ccci_smem_dump_state],
			ccci_smem_dump_last_result);
	mutex_unlock(&ccci_smem_dump_lock);
	return len;
}

static const struct kernel_param_ops ccci_smem_dump_status_ops = {
	.get = ccci_smem_dump_status_get,
};
module_param_cb(status, &ccci_smem_dump_status_ops, NULL, 0400);
MODULE_PARM_DESC(status, "read state and final errno; ready is not a dump");

static int __init ccci_smem_dump_init(void)
{
	const char *source;
	int ret;

	ret = ccci_smem_dump_read_header(&ccci_smem_dump_header, &source);
	pr_info("CCCI-SMEM: header source=%s result=%d base=0x%llx size=0x%x version=%d count=%d err=%d ld_flag=0x%x md1_err=%d\n",
		source, ret, ccci_smem_dump_header.base_addr,
		ccci_smem_dump_header.size, ccci_smem_dump_header.version,
		ccci_smem_dump_header.tag_num, ccci_smem_dump_header.err_no,
		ccci_smem_dump_header.ld_flag,
		ccci_smem_dump_header.ld_md_errno[0]);
	if (ret)
		return ret;

	mutex_lock(&ccci_smem_dump_lock);
	ccci_smem_dump_state = CCCI_SMEM_DUMP_READY;
	ccci_smem_dump_last_result = -ENODATA;
	mutex_unlock(&ccci_smem_dump_lock);
	pr_info("CCCI-SMEM: ready; nothing mapped or read; runtime trigger required\n");
	return 0;
}
module_init(ccci_smem_dump_init);

static void __exit ccci_smem_dump_exit(void)
{
	cancel_work_sync(&ccci_smem_dump_work);
	pr_info("CCCI-SMEM: module unloaded\n");
}
module_exit(ccci_smem_dump_exit);

MODULE_DESCRIPTION("MediaTek CCCI SMEM/CCB read-only layout dump (qqcandy)");
MODULE_LICENSE("GPL");
