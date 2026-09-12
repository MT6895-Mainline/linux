// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: standalone, opt-in runtime probe for the LK/modem tag blob.
 *
 * This was previously built into the ccci_util composite object (with the
 * runtime trigger in ccci_util_probe_runtime.c and the parse inline in
 * ccci_util_lib_fo.c). The parse dereferences modem reserved memory, which on
 * this device can raise an external abort and reset the phone, so it must
 * never be part of the boot image. It now lives only in this loadable module:
 * nothing runs unless a human explicitly loads the module on a running system.
 *
 * Safety properties (kept identical to the built-in version):
 *   - reads the LK info header (struct _ccci_lk_info_v2) from the
 *     "ccci,modem_info_v2" DT property, length-checked;
 *   - refuses any header base other than the fixed ccci_tag_mem region;
 *   - memremaps exactly ccci_tag_mem (0xbdbf0000, 0x10000) with
 *     MEMREMAP_WB, and reads only inside that mapping;
 *   - walks at most CCCI_PROBE_MAX_TAGS tags, every read checked against the
 *     header-declared blob size (<= 0x10000) before it happens, with strictly
 *     increasing offsets;
 *   - never writes to the mapping, never touches MMIO (no register range is
 *     mapped or read), and never reads any other reserved region;
 *   - no /dev/mem.
 *
 * Triggering: loading the module queues the parse on a workqueue (module_init);
 * writing a non-zero value to
 * /sys/module/ccci_probe/parameters/ccci_util_probe queues it again. "0" is
 * ignored. The parse never runs in the sysfs write context.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/errno.h>

/* qqcandy: tag layout constants, copied from ccci_util_lib_fo.c. */
#define CCCI_TAG_NAME_LEN_V2	(64)
#define CCCI_LK_INFO_VER_V2	(2)
#define MAX_MD_NUM_AT_LK	(4)

#define CCCI_TAG_MEM_BASE	(0x00000000bdbf0000ULL)
#define CCCI_TAG_MEM_SIZE	(0x00010000ULL)
#define CCCI_PROBE_MAX_TAGS	(64)

/*
 * qqcandy: minimal LK tag layouts, copied verbatim from ccci_util_lib_fo.c so
 * that this module is self-contained and does not depend on any built-in
 * symbol.
 */
struct _ccci_lk_info_v2 {
	unsigned long long lk_info_base_addr;
	unsigned int       lk_info_size;
	int                lk_info_err_no;
	int                lk_info_version;
	int                lk_info_tag_num;
	unsigned int       lk_info_ld_flag;
	int                lk_info_ld_md_errno[MAX_MD_NUM_AT_LK];
};

struct _ccci_tag_v2 {
	char tag_name[CCCI_TAG_NAME_LEN_V2];
	unsigned int data_offset;
	unsigned int data_size;
	unsigned int next_tag_offset;
};

struct _smem_layout {
	unsigned long long base_addr;
	unsigned int ap_md1_smem_offset;
	unsigned int ap_md1_smem_size;
	unsigned int ap_md3_smem_offset;
	unsigned int ap_md3_smem_size;
	unsigned int md1_md3_smem_offset;
	unsigned int md1_md3_smem_size;
	unsigned int total_smem_size;
};

struct _ccb_layout {
	unsigned long long ccb_data_buffer_addr;
	unsigned int ccb_data_buffer_size;
};

struct _sib_item {
	unsigned long long md1_sib_addr;
	unsigned int md1_sib_size;
};

/*
 * qqcandy: result of the bounded, read-only runtime tag parse. All addresses
 * are the raw values decoded from the LK tag blob; the parse never follows
 * them. Zero-initialised by ccci_probe_tag_parse().
 */
struct ccci_probe_result {
	bool header_valid;

	u64 lk_info_base_addr;
	u32 lk_info_size;
	int lk_info_err_no;
	int lk_info_version;
	int lk_info_tag_num;
	u32 lk_info_ld_flag;
	int lk_info_ld_md_errno[MAX_MD_NUM_AT_LK];

	unsigned int tags_walked;
	bool known_tag_found;

	bool smem_found;
	u64 smem_base_addr;
	u32 smem_ap_md1_offset;
	u32 smem_ap_md1_size;
	u32 smem_ap_md3_offset;
	u32 smem_ap_md3_size;
	u32 smem_md1_md3_offset;
	u32 smem_md1_md3_size;

	bool ccb_found;
	u64 ccb_addr;
	u32 ccb_size;

	bool sib_found;
	u64 sib_addr;
	u32 sib_size;
};

/* Gate armed by module_init and by the module parameter's set hook. */
static bool ccci_util_probe;

static void ccci_probe_work_fn(struct work_struct *work);
static DECLARE_WORK(ccci_probe_work, ccci_probe_work_fn);

static bool ccci_probe_tag_payload_in_range(unsigned int data_off,
					    unsigned int data_size,
					    unsigned int need,
					    unsigned int blob_size)
{
	if (data_size < need)
		return false;
	if (data_off > blob_size)
		return false;
	return (blob_size - data_off) >= need;
}

static int ccci_probe_tag_parse(struct ccci_probe_result *out)
{
	struct device_node *node;
	const void *raw;
	int len;
	struct _ccci_lk_info_v2 hdr;
	void *map;
	unsigned int blob_size;
	unsigned int tag_off;
	unsigned int i;

	if (!out)
		return -EINVAL;
	memset(out, 0, sizeof(*out));

	pr_info("CCCI-PROBE: runtime tag parse step 1: look up mddriver DT node\n");
	node = of_find_compatible_node(NULL, NULL, "mediatek,mddriver");
	if (!node) {
		pr_info("CCCI-PROBE: no mediatek,mddriver node; stop, nothing read\n");
		return -ENODEV;
	}
	raw = of_get_property(node, "ccci,modem_info_v2", &len);
	of_node_put(node);
	if (!raw) {
		pr_info("CCCI-PROBE: ccci,modem_info_v2 absent; stop, will not guess other regions\n");
		return -ENOENT;
	}
	if (len < (int)sizeof(hdr)) {
		pr_info("CCCI-PROBE: ccci,modem_info_v2 len=%d < %zu; stop\n",
			len, sizeof(hdr));
		return -EINVAL;
	}

	/* raw points into kernel DT memory; bounded by the len check above */
	memcpy(&hdr, raw, sizeof(hdr));
	out->header_valid = true;
	out->lk_info_base_addr = hdr.lk_info_base_addr;
	out->lk_info_size = hdr.lk_info_size;
	out->lk_info_err_no = hdr.lk_info_err_no;
	out->lk_info_version = hdr.lk_info_version;
	out->lk_info_tag_num = hdr.lk_info_tag_num;
	out->lk_info_ld_flag = hdr.lk_info_ld_flag;
	for (i = 0; i < MAX_MD_NUM_AT_LK; i++)
		out->lk_info_ld_md_errno[i] = hdr.lk_info_ld_md_errno[i];

	pr_info("CCCI-PROBE: header base=0x%llx size=0x%x err_no=%d version=%d tag_num=%d ld_flag=0x%x ld_md_errno=[%d,%d,%d,%d]\n",
		(unsigned long long)hdr.lk_info_base_addr, hdr.lk_info_size,
		hdr.lk_info_err_no, hdr.lk_info_version, hdr.lk_info_tag_num,
		hdr.lk_info_ld_flag,
		hdr.lk_info_ld_md_errno[0], hdr.lk_info_ld_md_errno[1],
		hdr.lk_info_ld_md_errno[2], hdr.lk_info_ld_md_errno[3]);

	/*
	 * Safety: only ever read the fixed ccci_tag_mem region. A header that
	 * points elsewhere is not followed - we stop instead of reading a
	 * DT-provided address.
	 */
	if (hdr.lk_info_base_addr != CCCI_TAG_MEM_BASE) {
		pr_info("CCCI-PROBE: header base 0x%llx != ccci_tag_mem 0x%llx; stop, no fallback region\n",
			(unsigned long long)hdr.lk_info_base_addr,
			(unsigned long long)CCCI_TAG_MEM_BASE);
		return -ERANGE;
	}
	if (hdr.lk_info_size == 0 || hdr.lk_info_size > CCCI_TAG_MEM_SIZE) {
		pr_info("CCCI-PROBE: header size 0x%x outside (0, 0x%llx]; stop\n",
			hdr.lk_info_size, (unsigned long long)CCCI_TAG_MEM_SIZE);
		return -ERANGE;
	}
	blob_size = hdr.lk_info_size;

	if ((unsigned int)hdr.lk_info_version < CCCI_LK_INFO_VER_V2) {
		pr_info("CCCI-PROBE: tag version %d < %d (v1 not walked); stop\n",
			hdr.lk_info_version, CCCI_LK_INFO_VER_V2);
		return -EOPNOTSUPP;
	}

	pr_info("CCCI-PROBE: runtime tag parse step 2: memremap ccci_tag_mem pa=0x%llx size=0x%llx WB\n",
		(unsigned long long)CCCI_TAG_MEM_BASE,
		(unsigned long long)CCCI_TAG_MEM_SIZE);
	map = memremap(CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE, MEMREMAP_WB);
	if (!map) {
		pr_info("CCCI-PROBE: memremap ccci_tag_mem failed; stop\n");
		return -ENOMEM;
	}

	pr_info("CCCI-PROBE: runtime tag parse step 3: walk blob_size=0x%x max_tags=%d\n",
		blob_size, CCCI_PROBE_MAX_TAGS);

	/*
	 * Tag count from the header is treated as untrusted: clamp to
	 * CCCI_PROBE_MAX_TAGS, and require every offset to stay inside the
	 * header-declared blob_size before any read.
	 */
	tag_off = 0;
	for (i = 0; i < (unsigned int)hdr.lk_info_tag_num &&
			i < CCCI_PROBE_MAX_TAGS; i++) {
		struct _ccci_tag_v2 tag;
		char name[CCCI_TAG_NAME_LEN_V2];
		unsigned int next;

		if (tag_off > blob_size ||
		    (blob_size - tag_off) < sizeof(tag))
			break;

		memcpy(&tag, (const char *)map + tag_off, sizeof(tag));
		memcpy(name, tag.tag_name, sizeof(name));
		name[sizeof(name) - 1] = '\0';
		out->tags_walked++;

		pr_info("CCCI-PROBE: tag[%u] off=0x%x name=\"%s\" data_off=0x%x data_size=0x%x next=0x%x\n",
			i, tag_off, name, tag.data_offset, tag.data_size,
			tag.next_tag_offset);

		if (!strcmp(name, "smem_layout") &&
		    ccci_probe_tag_payload_in_range(tag.data_offset,
				tag.data_size, sizeof(struct _smem_layout),
				blob_size)) {
			struct _smem_layout sl;

			memcpy(&sl, (const char *)map + tag.data_offset,
				sizeof(sl));
			out->smem_found = true;
			out->known_tag_found = true;
			out->smem_base_addr = sl.base_addr;
			out->smem_ap_md1_offset = sl.ap_md1_smem_offset;
			out->smem_ap_md1_size = sl.ap_md1_smem_size;
			out->smem_ap_md3_offset = sl.ap_md3_smem_offset;
			out->smem_ap_md3_size = sl.ap_md3_smem_size;
			out->smem_md1_md3_offset = sl.md1_md3_smem_offset;
			out->smem_md1_md3_size = sl.md1_md3_smem_size;
			pr_info("CCCI-PROBE: smem_layout base=0x%llx ap_md1=+0x%x/0x%x ap_md3=+0x%x/0x%x md1_md3=+0x%x/0x%x\n",
				(unsigned long long)sl.base_addr,
				sl.ap_md1_smem_offset, sl.ap_md1_smem_size,
				sl.ap_md3_smem_offset, sl.ap_md3_smem_size,
				sl.md1_md3_smem_offset, sl.md1_md3_smem_size);
		} else if (!strcmp(name, "ccb_info") &&
			   ccci_probe_tag_payload_in_range(tag.data_offset,
				tag.data_size, sizeof(struct _ccb_layout),
				blob_size)) {
			struct _ccb_layout ccb;

			memcpy(&ccb, (const char *)map + tag.data_offset,
				sizeof(ccb));
			out->ccb_found = true;
			out->known_tag_found = true;
			out->ccb_addr = ccb.ccb_data_buffer_addr;
			out->ccb_size = ccb.ccb_data_buffer_size;
			pr_info("CCCI-PROBE: ccb_info addr=0x%llx size=0x%x\n",
				(unsigned long long)ccb.ccb_data_buffer_addr,
				ccb.ccb_data_buffer_size);
		} else if (!strcmp(name, "md1_sib_info") &&
			   ccci_probe_tag_payload_in_range(tag.data_offset,
				tag.data_size, sizeof(struct _sib_item),
				blob_size)) {
			struct _sib_item sib;

			memcpy(&sib, (const char *)map + tag.data_offset,
				sizeof(sib));
			out->sib_found = true;
			out->known_tag_found = true;
			out->sib_addr = sib.md1_sib_addr;
			out->sib_size = sib.md1_sib_size;
			pr_info("CCCI-PROBE: md1_sib_info addr=0x%llx size=0x%x\n",
				(unsigned long long)sib.md1_sib_addr,
				sib.md1_sib_size);
		}

		next = tag.next_tag_offset;
		if (next <= tag_off || next > blob_size)
			break;
		tag_off = next;
	}

	memunmap(map);
	pr_info("CCCI-PROBE: runtime tag parse step 4: done, walked=%u known_tag=%d\n",
		out->tags_walked, out->known_tag_found);

	return 0;
}

static void ccci_probe_print_smem(const struct ccci_probe_result *res)
{
	pr_info("CCCI-PROBE: summary SMEM base=0x%llx AP<->MD1=0x%llx size=0x%x AP<->MD3=0x%llx size=0x%x MD1<->MD3=0x%llx size=0x%x\n",
		(unsigned long long)res->smem_base_addr,
		(unsigned long long)(res->smem_base_addr +
				     res->smem_ap_md1_offset),
		res->smem_ap_md1_size,
		(unsigned long long)(res->smem_base_addr +
				     res->smem_ap_md3_offset),
		res->smem_ap_md3_size,
		(unsigned long long)(res->smem_base_addr +
				     res->smem_md1_md3_offset),
		res->smem_md1_md3_size);
}

static void ccci_probe_work_fn(struct work_struct *work)
{
	struct ccci_probe_result res;
	int ret;

	pr_info("CCCI-PROBE: runtime trigger: work item start (not sysfs context)\n");

	ret = ccci_probe_tag_parse(&res);
	pr_info("CCCI-PROBE: runtime trigger: ccci_probe_tag_parse() = %d\n",
		ret);

	if (!res.header_valid) {
		pr_info("CCCI-PROBE: no header decoded; no reserved region was read\n");
		return;
	}

	pr_info("CCCI-PROBE: summary header base=0x%llx size=0x%x err_no=%d version=%d tag_num=%d ld_flag=0x%x walked=%u known_tag=%d\n",
		(unsigned long long)res.lk_info_base_addr, res.lk_info_size,
		res.lk_info_err_no, res.lk_info_version, res.lk_info_tag_num,
		res.lk_info_ld_flag, res.tags_walked, res.known_tag_found);

	if (res.smem_found)
		ccci_probe_print_smem(&res);
	else
		pr_info("CCCI-PROBE: summary no usable smem_layout tag found\n");

	if (res.ccb_found)
		pr_info("CCCI-PROBE: summary ccb addr=0x%llx size=0x%x\n",
			(unsigned long long)res.ccb_addr, res.ccb_size);
	if (res.sib_found)
		pr_info("CCCI-PROBE: summary md1_sib addr=0x%llx size=0x%x\n",
			(unsigned long long)res.sib_addr, res.sib_size);

	pr_info("CCCI-PROBE: runtime trigger: work item done\n");
}

/*
 * set hook: "0" does nothing; non-zero arms the gate and queues the parse.
 * Reached both from an explicit insmod parameter and from a later sysfs write.
 */
static int ccci_util_probe_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;

	if (!on) {
		pr_info("CCCI-PROBE: parameter written 0, ignored (no parse queued)\n");
		return 0;
	}

	*(bool *)kp->arg = true;

	pr_info("CCCI-PROBE: parameter armed, queueing bounded tag parse\n");
	if (!schedule_work(&ccci_probe_work))
		pr_info("CCCI-PROBE: tag parse already queued/running\n");

	return 0;
}

static const struct kernel_param_ops ccci_util_probe_ops = {
	.set = ccci_util_probe_set,
	.get = param_get_bool,
};

module_param_cb(ccci_util_probe, &ccci_util_probe_ops, &ccci_util_probe, 0644);
MODULE_PARM_DESC(ccci_util_probe,
	"arm the LK/modem tag parse; write non-zero to schedule it, default off");

static int __init ccci_probe_init(void)
{
	ccci_util_probe = true;
	pr_info("CCCI-PROBE: module loaded, queueing bounded LK tag parse\n");
	schedule_work(&ccci_probe_work);
	return 0;
}
module_init(ccci_probe_init);

static void __exit ccci_probe_exit(void)
{
	cancel_work_sync(&ccci_probe_work);
	pr_info("CCCI-PROBE: module unloaded\n");
}
module_exit(ccci_probe_exit);

MODULE_DESCRIPTION("MediaTek CCCI reserved-memory probe (qqcandy)");
MODULE_LICENSE("GPL");
