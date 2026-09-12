// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: opt-in LK/modem tag diagnostic, not a modem memory consumer.
 * Loading only checks the DT/stash header. A separate runtime write may
 * queue one read per module load. Even a read-only module can hang the SoC;
 * neither a workqueue nor cancel_work_sync() isolates a bus fault.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/workqueue.h>

#include "../ccci_util/ccci_tag_parse.h"

#ifndef MODULE
#error "The CCCI reserved-memory probe must only be built as a module"
#endif

#define CCCI_TAG_MEM_BASE	0x00000000bdbf0000ULL
#define CCCI_TAG_MEM_SIZE	0x00010000U

/* Verbatim property bytes captured before the embedded DTB takes over. */
extern u8 xaga_ccci_lk_prop[64];
extern int xaga_ccci_lk_prop_len;
extern char xaga_ccci_lk_prop_name[32];

enum ccci_probe_state {
	CCCI_PROBE_UNAVAILABLE,
	CCCI_PROBE_READY,
	CCCI_PROBE_QUEUED,
	CCCI_PROBE_RUNNING,
	CCCI_PROBE_DONE,
	CCCI_PROBE_FAILED,
};

static struct ccci_tag_hdr ccci_probe_header;
static enum ccci_probe_state ccci_probe_state = CCCI_PROBE_UNAVAILABLE;
static DEFINE_MUTEX(ccci_probe_lock);
static int ccci_probe_last_result = -ENODATA;
static bool ccci_util_probe;

static int ccci_probe_read_header(struct ccci_tag_hdr *hdr,
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
			if (len < sizeof(*hdr)) {
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
	    xaga_ccci_lk_prop_len > sizeof(xaga_ccci_lk_prop))
		return -EMSGSIZE;
	if (xaga_ccci_lk_prop_len < sizeof(*hdr))
		return -EMSGSIZE;
	*source = "LKINFO stash";
	memcpy(hdr, xaga_ccci_lk_prop, sizeof(*hdr));
	return ccci_validate_tag_hdr(hdr, CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE);
}

static void ccci_probe_log_tag(unsigned int index, unsigned int offset,
			       const struct ccci_tag *tag)
{
	pr_info("CCCI-PROBE: tag[%u] off=0x%x name=\"%s\" data=0x%x/0x%x next=0x%x\n",
		index, offset, tag->tag_name, tag->data_offset, tag->data_size,
		tag->next_tag_offset);
}

static void ccci_probe_work_fn(struct work_struct *work)
{
	struct ccci_tag_result res = {};
	void *map;
	int ret;

	mutex_lock(&ccci_probe_lock);
	ccci_probe_state = CCCI_PROBE_RUNNING;
	mutex_unlock(&ccci_probe_lock);

	pr_info("CCCI-PROBE: step 1: map fixed tag region pa=0x%llx size=0x%x WB\n",
		CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE);
	map = memremap(CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE, MEMREMAP_WB);
	if (!map) {
		ret = -ENOMEM;
		goto done;
	}

	pr_info("CCCI-PROBE: step 2: begin actual reads, size=0x%x count=%d\n",
		ccci_probe_header.size, ccci_probe_header.tag_num);
	ret = ccci_parse_tag_chain(&ccci_probe_header, map,
				   ccci_probe_header.size, &res);
	memunmap(map);

done:
	if (ret) {
		pr_info("CCCI-PROBE: parse failed: ret=%d walked=%u last_off=0x%x; layout not usable\n",
			ret, res.tags_walked, res.tag_offset);
	} else {
		pr_info("CCCI-PROBE: complete: walked=%u SMEM base=0x%llx total=0x%x AP-MD1=+0x%x/0x%x\n",
			res.tags_walked, res.smem.base_addr, res.smem.total_smem_size,
			res.smem.ap_md1_smem_offset, res.smem.ap_md1_smem_size);
		if (res.ccb_found)
			pr_info("CCCI-PROBE: CCB addr=0x%llx size=0x%x\n",
				res.ccb.addr, res.ccb.size);
		if (res.sib_found)
			pr_info("CCCI-PROBE: SIB addr=0x%llx size=0x%x\n",
				res.sib.addr, res.sib.size);
	}

	mutex_lock(&ccci_probe_lock);
	ccci_probe_last_result = ret;
	ccci_probe_state = ret ? CCCI_PROBE_FAILED : CCCI_PROBE_DONE;
	mutex_unlock(&ccci_probe_lock);
}

static DECLARE_WORK(ccci_probe_work, ccci_probe_work_fn);

/*
 * Caller serializes state changes. Return 1 to queue, 0 for a no-op, or an
 * errno. COMING/GOING modules cannot arm a read; writing 0 never resets it.
 */
static int ccci_probe_request_read(enum ccci_probe_state *state,
				  bool live, bool on)
{
	if (!on)
		return 0;
	if (!live)
		return -EAGAIN;
	if (*state == CCCI_PROBE_UNAVAILABLE)
		return -ENODATA;
	if (*state != CCCI_PROBE_READY)
		return -EALREADY;
	*state = CCCI_PROBE_QUEUED;
	return 1;
}

static int ccci_util_probe_set(const char *val, const struct kernel_param *kp)
{
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;

	mutex_lock(&ccci_probe_lock);
	ret = ccci_probe_request_read(&ccci_probe_state,
				     READ_ONCE(THIS_MODULE->state) == MODULE_STATE_LIVE,
				     on);
	if (ret > 0) {
		ret = 0;
		*(bool *)kp->arg = true;
		ccci_probe_last_result = -EINPROGRESS;
		if (!schedule_work(&ccci_probe_work)) {
			ret = -EBUSY;
			ccci_probe_last_result = ret;
			ccci_probe_state = CCCI_PROBE_FAILED;
		}
	}
	mutex_unlock(&ccci_probe_lock);
	return ret;
}

static const struct kernel_param_ops ccci_util_probe_ops = {
	.set = ccci_util_probe_set,
	.get = param_get_bool,
};

module_param_cb(ccci_util_probe, &ccci_util_probe_ops, &ccci_util_probe, 0600);
MODULE_PARM_DESC(ccci_util_probe,
	"write 1 after loading to attempt one tag read; 0 does not arm or cancel");

static int ccci_probe_status_get(char *buffer, const struct kernel_param *kp)
{
	static const char * const names[] = {
		[CCCI_PROBE_UNAVAILABLE] = "unavailable",
		[CCCI_PROBE_READY] = "ready",
		[CCCI_PROBE_QUEUED] = "queued",
		[CCCI_PROBE_RUNNING] = "running",
		[CCCI_PROBE_DONE] = "done",
		[CCCI_PROBE_FAILED] = "failed",
	};
	int len;

	mutex_lock(&ccci_probe_lock);
	len = scnprintf(buffer, PAGE_SIZE, "%s result=%d\n",
			names[ccci_probe_state], ccci_probe_last_result);
	mutex_unlock(&ccci_probe_lock);
	return len;
}

static const struct kernel_param_ops ccci_probe_status_ops = {
	.get = ccci_probe_status_get,
};
module_param_cb(status, &ccci_probe_status_ops, NULL, 0400);
MODULE_PARM_DESC(status, "read state and final errno; ready is not a tag read");

static int __init ccci_probe_init(void)
{
	const char *source;
	int ret;

	ret = ccci_probe_read_header(&ccci_probe_header, &source);
	pr_info("CCCI-PROBE: header source=%s result=%d base=0x%llx size=0x%x version=%d count=%d err=%d ld_flag=0x%x md1_err=%d\n",
		source, ret, ccci_probe_header.base_addr,
		ccci_probe_header.size, ccci_probe_header.version,
		ccci_probe_header.tag_num, ccci_probe_header.err_no,
		ccci_probe_header.ld_flag, ccci_probe_header.ld_md_errno[0]);
	if (ret)
		return ret;

	mutex_lock(&ccci_probe_lock);
	ccci_probe_state = CCCI_PROBE_READY;
	ccci_probe_last_result = -ENODATA;
	mutex_unlock(&ccci_probe_lock);
	pr_info("CCCI-PROBE: ready; no tag region mapped or read; runtime trigger required\n");
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
