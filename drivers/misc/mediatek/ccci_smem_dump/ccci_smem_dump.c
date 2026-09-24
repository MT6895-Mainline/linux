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

/*
 * The modem image bank. The LK tag md_bank0_base names the AP-side base of
 * the MD's bank0 and region 0 of md_mem_layout is the 38.4 MB md1rom entry,
 * so the first megabyte of DRAM there is either the firmware LK loaded or
 * nothing at all.
 *
 * WARNING: reading it from the AP is NOT possible on this SoC. With the MD
 * power domain off, the first access raises "Unable to handle kernel ttbr
 * address size fault" (ESR 0x96000000) even though dump_pagetable shows a
 * valid AF=1 PTE mapping exactly PA 0xD0000000 - the md_mem_usage regions are
 * hardware-protected against the AP. The Oops kills the workqueue worker, the
 * module is then stuck in RUNNING (rmmod hangs) and the box needs a reset.
 * Hence the stage is opt-in and off by default; it is kept only as a probe
 * for a future (MD-domain-up) attempt with the user present.
 */
#define CCCI_MDIMG_BASE		0x00000000d0000000ULL
#define CCCI_MDIMG_MAP_SIZE	0x00100000U

static bool ccci_smem_dump_mdimg_read;
module_param_named(mdimg_read, ccci_smem_dump_mdimg_read, bool, 0600);
MODULE_PARM_DESC(mdimg_read,
		 "DANGEROUS: also read MD bank0 at 0xd0000000; faults the AP today (see HANDOFF §80.32)");

/*
 * Optional and off by default: deassert the MD power domain's AXI bus
 * protection entries (the scpsys bp_table, IFR_TYPE, in infracfg).
 *
 * The eccci driver programs the CCIF entirely from its probe - the clock
 * gates, the SRAM clear and the magic/MDSS-address publish in
 * ccci_reset_ccif_hw() - which on this port happens with the MD domain still
 * off, because nothing holds it on the way LK does on stock. With the
 * protection asserted every AP-side CCIF access is dropped silently (reads
 * come back 0, writes never land), which is exactly what the device shows.
 * Writing the clr registers is the same set/clr register class already used
 * and verified for the CCIF clock gates, and the domain is held on by the
 * mddriver probe at that point, so there is no traffic to a dead domain.
 */
static bool ccci_smem_dump_bp_clear;
module_param_named(bp_clear, ccci_smem_dump_bp_clear, bool, 0600);
MODULE_PARM_DESC(bp_clear,
		 "deassert the MD-domain AXI bus protection at load (see HANDOFF 80.36)");

/*
 * Optional and off by default: apply the vendor's MD source-clock-enable
 * setting (INFRA_AO_MD_SRCCLKENA, infra_ao+0xF0C) exactly as
 * md_cd_srcclkena_setting() does - keep the upper bits, put 0x21 in the low
 * byte. That vendor step is bypassed on our power_flow_config (bit0 clear, and
 * the same is true of the official mt6895.dts), yet the device reads the low
 * byte back as 0x00, i.e. the modem's source clock may never have been
 * enabled. Same always-on infracfg block and same write class as the CCIF
 * gate writes we already do, and enabling a clock is the benign direction.
 */
static bool ccci_smem_dump_srcclkena;
module_param_named(srcclkena, ccci_smem_dump_srcclkena, bool, 0600);
MODULE_PARM_DESC(srcclkena,
		 "apply the vendor MD_SRCCLKENA=0x21 setting at load (see HANDOFF 80.39)");

/*
 * DANGEROUS and off by default: reading the DEVAPC instance blocks wedges the
 * bus - the first read of 0x1000e000 left the worker in D state and only a
 * reboot cleared it (HANDOFF 80.36, second addendum). The stage used to run
 * unconditionally, so every trigger of this module was a booby trap; it is
 * now opt-in like the MD bank0 stage.
 */
static bool ccci_smem_dump_dapc_read;
module_param_named(dapc_read, ccci_smem_dump_dapc_read, bool, 0600);
MODULE_PARM_DESC(dapc_read,
		 "DANGEROUS: also read the five DEVAPC instance blocks; wedges the bus (see HANDOFF 80.36)");

/*
 * DANGEROUS and off by default. Reading the CCIF register bank before the eccci
 * driver has pulsed its reset HANGS THE BUS: on 2026-09-13 22:15 this stage was
 * enabled with the MD domain up and the six clock gates opened, and the worker
 * stuck on the very first readl (0x10209000+0x00, status=running result=-115);
 * the device was unusable for ~4 minutes until it reset itself. So "read the
 * ground truth before the driver programs the block" is NOT a viable
 * experiment - the block only answers after the driver's reset pulse, and even
 * then it answers 0 (see HANDOFF 80.44). Kept only as a record.
 */
static bool ccci_smem_dump_ccif_probe;
module_param_named(ccif_probe, ccci_smem_dump_ccif_probe, bool, 0600);
MODULE_PARM_DESC(ccif_probe,
		 "DANGEROUS: read the CCIF banks before the driver programs them; hangs the bus (HANDOFF 80.44)");

/*
 * Off by default like every risky stage. This is the corrected version of
 * the 80.45 comparison (which read a DPMAIF register via devmem2 with its
 * IFRAO gates unproven and only re-proved that a gated read wedges the
 * bus): open the three stock DPMAIF gates, prove them latched in STA,
 * require the MD power domain up, and only then read the four windows
 * from the stock dpmaif node in this worker (worst case is the known
 * ~4-minute bus-wedge self-recovery).
 */
static bool ccci_smem_dump_dpmaif_probe;
module_param_named(dpmaif_probe, ccci_smem_dump_dpmaif_probe, bool, 0600);
MODULE_PARM_DESC(dpmaif_probe,
		 "open the 3 stock DPMAIF clock gates and read its 4 windows (same-family comparison, HANDOFF 80.44.5/80.45)");

static void ccci_smem_dump_srcclkena_run(void)
{
	void *ao = ioremap(0x0000000010001000ULL, 0x1000);
	unsigned int before, after;

	if (!ao) {
		pr_info("CCCI-SMEM: srcclkena: infra_ao ioremap failed\n");
		return;
	}
	before = readl(ao + 0x0f0c);
	writel((before & ~0xffU) | 0x21U, ao + 0x0f0c);
	mb();
	after = readl(ao + 0x0f0c);
	pr_info("CCCI-SMEM: srcclkena: MD_SRCCLKENA before=0x%08x after=0x%08x%s\n",
		before, after, (after & 0xff) == 0x21 ? " (write took)" : " (write did NOT stick)");
	iounmap(ao);
}

static void ccci_smem_dump_bp_clear_run(void)
{
	void *ao = ioremap(0x0000000010001000ULL, 0x1000);

	if (!ao) {
		pr_info("CCCI-SMEM: bp_clear: infra_ao ioremap failed\n");
		return;
	}
	pr_info("CCCI-SMEM: bp_clear: before 0xc4c=0x%08x 0xc5c=0x%08x 0xc6c=0x%08x\n",
		readl(ao + 0x0c4c), readl(ao + 0x0c5c), readl(ao + 0x0c6c));
	writel(BIT(28), ao + 0x0c48);		/* INFRASYS0_MD */
	mb();
	writel(BIT(9), ao + 0x0c58);		/* INFRASYS1_MD */
	mb();
	writel(BIT(16) | BIT(17), ao + 0x0c68);	/* EMISYS0_MD */
	mb();
	pr_info("CCCI-SMEM: bp_clear: after  0xc4c=0x%08x 0xc5c=0x%08x 0xc6c=0x%08x\n",
		readl(ao + 0x0c4c), readl(ao + 0x0c5c), readl(ao + 0x0c6c));
	iounmap(ao);
}

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
	struct ccci_smem_tag_view md_mem_layout;	/* md_mem_layout */
	struct ccci_smem_tag_view md_bank0_base;	/* md_bank0_base */
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
	else if (!strcmp(tag->tag_name, "md_mem_layout"))
		view = &tc->md_mem_layout;
	else if (!strcmp(tag->tag_name, "md_bank0_base"))
		view = &tc->md_bank0_base;

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
	unsigned long long md_bank0 = 0;
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

	/* LK's MD memory map: the key for translating modem PCs into image
	 * offsets. Dump the raw payload. */
	if (tc.md_mem_layout.data) {
		unsigned int k;

		pr_info("CCCI-SMEM: md_mem_layout payload (%u bytes):\n",
			tc.md_mem_layout.size);
		for (k = 0; k * 4 + 4 <= tc.md_mem_layout.size && k < 108; k++) {
			unsigned int v;

			memcpy(&v, (const char *)tc.md_mem_layout.data + k * 4, 4);
			pr_info("CCCI-SMEM:   mm[%02u]=0x%08x\n", k, v);
		}
	}
	if (tc.md_bank0_base.data && tc.md_bank0_base.size >= 8) {
		memcpy(&md_bank0, tc.md_bank0_base.data, 8);
		pr_info("CCCI-SMEM: md_bank0_base=0x%llx\n", md_bank0);
	}

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
		pr_info("CCCI-SMEM:   csmem[%u] addr=0x%llx md_off=0x%x size=0x%x id=%u\n",
			i, csmem[i].addr, csmem[i].md_offset, csmem[i].size,
			csmem[i].item_cnt);
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
		/* The MD copies its CCIF SRAM snapshot into MDSS_DBG at
		 * CCCI_EE_OFFSET_CCIF_SRAM (952) when it throws an
		 * exception; dump the surrounding window too. */
		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_RAW_MDSS_DBG);
		if (idx >= 0 && tbl[idx].size >= 0x420)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset + 0x3a0, 28,
					     "MDSS_CCIFSNAP");
		/* MDSS head: the MD's own boot log / state view. */
		if (idx >= 0 && tbl[idx].size >= 0x100)
			ccci_smem_dump_hex32(smem_map, smem_size,
					     tbl[idx].offset, 32,
					     "MDSS_HEAD");
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

	/* ---- MD bank0: did LK actually populate the modem image? ----
	 * Read-only DRAM: no MMIO, no clocks, no SMC. The window is hashed so
	 * the host can compare it against the OTA md1img without shipping the
	 * megabyte itself.
	 */
	if (md_bank0 != CCCI_MDIMG_BASE || !ccci_smem_dump_mdimg_read) {
		pr_info("CCCI-SMEM: step 5 skipped: md_bank0_base=0x%llx mdimg_read=%d\n",
			md_bank0, ccci_smem_dump_mdimg_read);
	} else {
		void *img;

		pr_info("CCCI-SMEM: step 5: map MD bank0 pa=0x%llx size=0x%x WB (read-only)\n",
			md_bank0, CCCI_MDIMG_MAP_SIZE);
		img = memremap(md_bank0, CCCI_MDIMG_MAP_SIZE, MEMREMAP_WB);
		if (!img) {
			pr_info("CCCI-SMEM: bank0 memremap failed; skipping\n");
		} else {
			const u32 *wr = img;
			size_t words = CCCI_MDIMG_MAP_SIZE / sizeof(u32);
			u64 fnv = 0xcbf29ce484222325ULL;
			unsigned long long zero = 0;
			size_t k;

			for (k = 0; k < words; k++) {
				u32 v = wr[k];
				int shift;

				if (!v)
					zero++;
				for (shift = 0; shift < 32; shift += 8)
					fnv = (fnv ^ ((v >> shift) & 0xff)) *
					      0x100000001b3ULL;
			}
			pr_info("CCCI-SMEM: MDIMG pa=0x%llx window=0x%x zero_words=%llu/%zu fnv1a=0x%llx\n",
				md_bank0, CCCI_MDIMG_MAP_SIZE, zero, words, fnv);
			ccci_smem_dump_hex32(img, CCCI_MDIMG_MAP_SIZE, 0, 16,
					     "MDIMG");
			ccci_smem_dump_hex32(img, CCCI_MDIMG_MAP_SIZE, 0x1e0, 16,
					     "MDIMG");
			ccci_smem_dump_hex32(img, CCCI_MDIMG_MAP_SIZE, 0x380, 16,
					     "MDIMG");
			memunmap(img);
		}
	}

	/* ---- infra_ao: CCIF gate / reset / power state (read-only) ----
	 * The HF 0x10001000 block is always-on and reading it is independent of
	 * the MD domain (ccci_diag proved that before). Needed because on the
	 * device the AP's writes into the CCIF SRAM do not stick while the CCIF
	 * registers read back as 0 - consistent with the block still being held
	 * in reset / not enabled, see HANDOFF §80.34.
	 */
	{
		static const struct {
			unsigned int off;
			const char *name;
		} ao_regs[] = {
			{ 0x088, "IFRAO1_SET" },
			{ 0x08c, "IFRAO1_CLR" },
			{ 0x094, "IFRAO1_STA" },
			{ 0x0c0, "IFRAO3_SET" },
			{ 0x0c4, "IFRAO3_CLR" },
			{ 0x0c8, "IFRAO3_STA" },
			{ 0x150, "CCIF_RST_SET" },
			{ 0x154, "CCIF_RST_CLR" },
			{ 0xbf0, "CCIF_PWR_BF0" },
			{ 0xf0c, "MD_SRCCLKENA" },
			{ 0xf50, "RST_VER1_SET" },
			{ 0xf54, "RST_VER1_CLR" },
			/* MD-domain AXI bus protection (scpsys bp_table,
			 * IFR_TYPE): sta bits SET = the AP cannot reach the
			 * MD domain, which is exactly how an inert CCIF looks
			 * (reads 0, writes dropped). */
			{ 0xc4c, "BP_INFRASYS0_STA" },
			{ 0xc5c, "BP_INFRASYS1_STA" },
			{ 0xc6c, "BP_EMISYS0_STA" },
			{ 0xc48, "BP_INFRASYS0_CLR" },
			{ 0xc58, "BP_INFRASYS1_CLR" },
			{ 0xc68, "BP_EMISYS0_CLR" },
		};
		void *ao = ioremap(0x0000000010001000ULL, 0x1000);
		unsigned int r;

		if (!ao) {
			pr_info("CCCI-SMEM: infra_ao ioremap failed; skipped\n");
		} else {
			for (r = 0; r < ARRAY_SIZE(ao_regs); r++)
				pr_info("CCCI-SMEM: INFRA_AO+0x%03x %-13s = 0x%08x\n",
					ao_regs[r].off, ao_regs[r].name,
					readl(ao + ao_regs[r].off));
			iounmap(ao);
		}
	}

	/* ---- CCIF register banks + SRAM windows (read-only, gated) ----
	 * Ordered before the DEVAPC stage on purpose: this is the only stage
	 * that answers "does the block respond to the AP at all", and it must
	 * run before the eccci driver clears/publishes the SRAM.
	 */
	if (ccci_smem_dump_ccif_probe) {
		static const struct {
			unsigned int base;
			const char *name;
		} win[] = {
			{ 0x10209000, "APCCIF" },
			{ 0x1020a000, "MDCCIF" },
		};
		void *ao = ioremap(0x0000000010001000ULL, 0x1000);
		unsigned int bp = ao ? readl(ao + 0xc4c) : 0;
		unsigned int w;

		if (ao)
			iounmap(ao);
		/* §80.36 corrected the polarity: bit28 SET = protection
		 * asserted = the MD domain is off, so do not touch CCIF. */
		if (bp & (1u << 28)) {
			pr_info("CCCI-SMEM: CCIF probe skipped: BP_INFRASYS0_STA=0x%08x (MD domain off)\n",
				bp);
		} else {
			/* Open the same six gates the driver opens, with the
			 * same bits, BEFORE reading CCIF: an MTK peripheral
			 * register read with the clock gated off wedges the
			 * bus (this project learned that the hard way). The
			 * status word tells us what LK had left. */
			void *ao2 = ioremap(0x0000000010001000ULL, 0x1000);
			unsigned int sta1 = ao2 ? readl(ao2 + 0x94) : 0;
			unsigned int sta3 = ao2 ? readl(ao2 + 0xc8) : 0;

			if (ao2) {
				writel((1u << 12) | (1u << 13) | (1u << 23) |
				       (1u << 26), ao2 + 0x88);
				writel((1u << 10) | (1u << 29), ao2 + 0xc0);
				pr_info("CCCI-SMEM: CCIF probe: gates before STA1=0x%08x STA3=0x%08x, after STA1=0x%08x STA3=0x%08x\n",
					sta1, sta3, readl(ao2 + 0x94),
					readl(ao2 + 0xc8));
				iounmap(ao2);
			} else {
				pr_info("CCCI-SMEM: CCIF probe: infra_ao re-ioremap failed\n");
			}
			pr_info("CCCI-SMEM: CCIF probe: BP_INFRASYS0_STA=0x%08x (MD domain on)\n",
				bp);
			for (w = 0; w < ARRAY_SIZE(win); w++) {
				void *ccif = ioremap(win[w].base, 0x1000);

				if (!ccif) {
					pr_info("CCCI-SMEM: %s ioremap failed\n",
						win[w].name);
					continue;
				}
				pr_info("CCCI-SMEM: %s CON=0x%08x BUSY=0x%08x START=0x%08x TCH=0x%08x RCH=0x%08x ACK=0x%08x\n",
					win[w].name,
					readl(ccif + 0x00), readl(ccif + 0x04),
					readl(ccif + 0x08), readl(ccif + 0x0c),
					readl(ccif + 0x10), readl(ccif + 0x14));
				pr_info("CCCI-SMEM: %s CHDATA[0x100..] %08x %08x %08x %08x %08x %08x %08x %08x\n",
					win[w].name,
					readl(ccif + 0x100), readl(ccif + 0x104),
					readl(ccif + 0x108), readl(ccif + 0x10c),
					readl(ccif + 0x110), readl(ccif + 0x114),
					readl(ccif + 0x118), readl(ccif + 0x11c));
				pr_info("CCCI-SMEM: %s CHDATA[0x2f0..] tail %08x %08x %08x %08x\n",
					win[w].name,
					readl(ccif + 0x2f0), readl(ccif + 0x2f4),
					readl(ccif + 0x2f8), readl(ccif + 0x2fc));
				iounmap(ccif);
			}
		}
	}

	/* ---- DPMAIF windows (read-only, gated) ----
	 * The same-family comparison HANDOFF 80.44.5 asked for, done the way
	 * 80.45 failed to do it. Stock clocks are IFRAO2[3] cldmabclk,
	 * IFRAO3[26] dpmaif_main and IFRAO4[17] dpmaif_26m (stock
	 * clk-mt6895-bus.c); nothing in our tree ever opens them, so they are
	 * opened here and proven latched in STA before any window is touched.
	 * The windows are the four reg entries of the stock dpmaif node.
	 * Reading a live block returns data; the CCIF failure signature is
	 * all-zero reads with dropped writes; a hang means the block is dark
	 * like the DEVAPC instances. Either of the last two means the problem
	 * is not CCIF-specific.
	 */
	if (ccci_smem_dump_dpmaif_probe) {
		static const struct {
			unsigned int base;
			const char *name;
		} win[] = {
			{ 0x10014000, "DPMAIF_AO_UL" },
			{ 0x1022c000, "DPMAIF_PD_MD_MISC" },
			{ 0x1022d000, "DPMAIF_PD_UL" },
			{ 0x1022e000, "DPMAIF_SRAM" },
		};
		void *ao = ioremap(0x0000000010001000ULL, 0x1000);
		unsigned int bp = ao ? readl(ao + 0xc4c) : 0;

		if (ao)
			iounmap(ao);
		/* Same polarity as the CCIF probe: bit28 SET = MD domain off. */
		if (bp & (1u << 28)) {
			pr_info("CCCI-SMEM: DPMAIF probe skipped: BP_INFRASYS0_STA=0x%08x (MD domain off)\n",
				bp);
		} else {
			void *ao2 = ioremap(0x0000000010001000ULL, 0x1000);

			if (!ao2) {
				pr_info("CCCI-SMEM: DPMAIF probe: infra_ao re-ioremap failed\n");
			} else {
				unsigned int b2 = readl(ao2 + 0xac);
				unsigned int b3 = readl(ao2 + 0xc8);
				unsigned int b4 = readl(ao2 + 0xe8);
				bool latched;

				writel(1u << 3, ao2 + 0xa4);   /* IFRAO2 cldmabclk */
				writel(1u << 26, ao2 + 0xc0);  /* IFRAO3 dpmaif_main */
				writel(1u << 17, ao2 + 0xe0);  /* IFRAO4 dpmaif_26m */
				latched = ((readl(ao2 + 0xac) & (1u << 3)) != 0) &&
					  ((readl(ao2 + 0xc8) & (1u << 26)) != 0) &&
					  ((readl(ao2 + 0xe8) & (1u << 17)) != 0);
				pr_info("CCCI-SMEM: DPMAIF probe: gates before STA2=0x%08x STA3=0x%08x STA4=0x%08x, after STA2=0x%08x STA3=0x%08x STA4=0x%08x latched=%d\n",
					b2, b3, b4, readl(ao2 + 0xac),
					readl(ao2 + 0xc8), readl(ao2 + 0xe8),
					latched);
				iounmap(ao2);
				if (!latched) {
					pr_info("CCCI-SMEM: DPMAIF probe: a gate did not latch; not touching the windows\n");
				} else {
					unsigned int w;

					pr_info("CCCI-SMEM: DPMAIF probe: BP_INFRASYS0_STA=0x%08x (MD domain on), reading windows\n",
						bp);
					for (w = 0; w < ARRAY_SIZE(win); w++) {
						void *dpmaif = ioremap(win[w].base, 0x1000);

						if (!dpmaif) {
							pr_info("CCCI-SMEM: %s ioremap failed\n",
								win[w].name);
							continue;
						}
						pr_info("CCCI-SMEM: %s reading\n",
							win[w].name);
						pr_info("CCCI-SMEM: %s +0x00 %08x %08x %08x %08x +0x30 %08x %08x %08x %08x +0x46c %08x %08x\n",
							win[w].name,
							readl(dpmaif + 0x00),
							readl(dpmaif + 0x04),
							readl(dpmaif + 0x08),
							readl(dpmaif + 0x0c),
							readl(dpmaif + 0x30),
							readl(dpmaif + 0x34),
							readl(dpmaif + 0x38),
							readl(dpmaif + 0x3c),
							readl(dpmaif + 0x46c),
							readl(dpmaif + 0x470));
						iounmap(dpmaif);
					}
				}
			}
		}
	}

	/* ---- DEVAPC instances: violation record (read-only, gated) ----
	 * Bases come from the stock FDT (always-on infra blocks). If a DEVAPC
	 * rule denies the AP access to the CCIF, the hardware behaves exactly
	 * like the CCIF does today - reads return the default 0, writes are
	 * dropped - and it records the offending address in VIO_DBG*. Offsets
	 * per devapc-mt6895.h: mask 0x000, sta 0x400, dbg 0x900/904/908/90C,
	 * apc_con 0xF00.
	 * NOT run unless dapc_read=1: the first read wedges the bus (§80.36).
	 */
	if (ccci_smem_dump_dapc_read) {
		static const struct {
			unsigned int base;
			const char *name;
		} dapc[] = {
			{ 0x1000e000, "ao_infra_peri0" },
			{ 0x10015000, "mpu_ao" },
			{ 0x10019000, "ao_md" },
			{ 0x1001c000, "ao_mm" },
			{ 0x10022000, "ao_infra_peri1" },
		};
		unsigned int d;

		for (d = 0; d < ARRAY_SIZE(dapc); d++) {
			void *base = ioremap(dapc[d].base, 0x1000);

			if (!base) {
				pr_info("CCCI-SMEM: DEVAPC %s ioremap failed\n",
					dapc[d].name);
				continue;
			}
			pr_info("CCCI-SMEM: DEVAPC %-14s sta=0x%08x apc_con=0x%08x dbg=%08x %08x %08x %08x\n",
				dapc[d].name, readl(base + 0x400),
				readl(base + 0xf00), readl(base + 0x900),
				readl(base + 0x904), readl(base + 0x908),
				readl(base + 0x90c));
			iounmap(base);
		}
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
	if (ccci_smem_dump_bp_clear)
		ccci_smem_dump_bp_clear_run();
	if (ccci_smem_dump_srcclkena)
		ccci_smem_dump_srcclkena_run();
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
