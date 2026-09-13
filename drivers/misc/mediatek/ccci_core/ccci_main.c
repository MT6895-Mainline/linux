// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek CCCI core.
 *
 * There is no CCCI device node in the board DTS - LK only carves the tag
 * chain out of DRAM as the "mediatek,ccci_tag_mem" reserved-memory region and
 * hands the describing header over as a DT property. So this is a plain
 * module_init that looks the region up itself rather than a platform driver
 * waiting for a match that never comes.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/slab.h>
#include "ccci_internal.h"

/* reserved-memory node LK fills with the tag chain */
#define CCCI_TAG_MEM_COMPAT	"mediatek,ccci_tag_mem"

struct ccci_device *g_ccci_dev;

static const char *ccci_state_name(enum ccci_state state)
{
	switch (state) {
	case CCCI_STATE_INVALID: return "INVALID";
	case CCCI_STATE_IDLE: return "IDLE";
	case CCCI_STATE_READY: return "READY";
	case CCCI_STATE_BOOT: return "BOOT";
	case CCCI_STATE_RUNNING: return "RUNNING";
	case CCCI_STATE_EXCEPTION: return "EXCEPTION";
	default: return "UNKNOWN";
	}
}

static int ccci_find_tag_region(struct ccci_device *cdev)
{
	struct device_node *node;
	struct resource res;
	int ret;

	node = of_find_compatible_node(NULL, NULL, CCCI_TAG_MEM_COMPAT);
	if (!node) {
		pr_err("CCCI: no %s node in DT\n", CCCI_TAG_MEM_COMPAT);
		return -ENODEV;
	}

	ret = of_address_to_resource(node, 0, &res);
	of_node_put(node);
	if (ret) {
		pr_err("CCCI: failed to read tag region address: %d\n", ret);
		return ret;
	}

	cdev->tag_base = res.start;
	cdev->tag_size = resource_size(&res);

	pr_info("CCCI: tag region base=0x%llx size=0x%zx\n",
		(unsigned long long)cdev->tag_base, cdev->tag_size);

	return 0;
}

static void ccci_record_memory_info(struct ccci_device *cdev)
{
	struct ccci_tag_result *result = &cdev->parse_result;

	/* ccci_parse_tag_chain() only returns 0 when smem_layout was found. */
	cdev->smem_base = result->smem.base_addr;
	cdev->smem_size = result->smem.total_smem_size;
	pr_info("CCCI: SMEM base=0x%llx size=0x%x AP-MD1=+0x%x/0x%x\n",
		result->smem.base_addr, result->smem.total_smem_size,
		result->smem.ap_md1_smem_offset, result->smem.ap_md1_smem_size);

	if (result->ccb_found) {
		cdev->ccb_base = result->ccb.addr;
		cdev->ccb_size = result->ccb.size;
		pr_info("CCCI: CCB base=0x%llx size=0x%x\n",
			result->ccb.addr, result->ccb.size);
	}

	/*
	 * ponytail: SMEM/CCB are described but deliberately left unmapped -
	 * they are live modem-owned memory and nothing here reads them yet.
	 * Map them in the phase that actually talks to the modem.
	 */
}

static int __init ccci_core_init(void)
{
	struct ccci_device *cdev;
	struct ccci_tag_hdr hdr;
	const char *source;
	int ret;

	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	mutex_init(&cdev->state_lock);
	mutex_init(&cdev->port_lock);
	INIT_LIST_HEAD(&cdev->ports);
	cdev->state = CCCI_STATE_IDLE;

	ret = ccci_find_tag_region(cdev);
	if (ret)
		goto err_free;

	ret = ccci_get_lk_tag_hdr(&hdr, &source);
	if (ret) {
		pr_err("CCCI: no LK tag header (%s): %d\n", source, ret);
		goto err_free;
	}

	ret = ccci_validate_tag_hdr(&hdr, cdev->tag_base, cdev->tag_size);
	if (ret) {
		pr_err("CCCI: tag header from %s failed validation: %d\n",
		       source, ret);
		goto err_free;
	}

	pr_info("CCCI: tag header from %s: v%d tags=%d size=0x%x\n",
		source, hdr.version, hdr.tag_num, hdr.size);

	/* Carved-out DRAM, not MMIO - WB, matching the verified probe path. */
	cdev->tag_virt = memremap(cdev->tag_base, cdev->tag_size, MEMREMAP_WB);
	if (!cdev->tag_virt) {
		pr_err("CCCI: failed to map tag region\n");
		ret = -ENOMEM;
		goto err_free;
	}

	ret = ccci_parse_tag_chain(&hdr, cdev->tag_virt, hdr.size,
				   &cdev->parse_result);
	if (ret) {
		pr_err("CCCI: tag parse failed: ret=%d walked=%u last_off=0x%x\n",
		       ret, cdev->parse_result.tags_walked,
		       cdev->parse_result.tag_offset);
		goto err_unmap;
	}

	pr_info("CCCI: parsed %u tags\n", cdev->parse_result.tags_walked);
	ccci_record_memory_info(cdev);

	cdev->state = CCCI_STATE_READY;
	g_ccci_dev = cdev;

	ret = ccci_debugfs_init(cdev);
	if (ret)
		pr_warn("CCCI: debugfs init failed: %d\n", ret); /* non-fatal */

	pr_info("CCCI: core ready, state=%s\n", ccci_state_name(cdev->state));
	return 0;

err_unmap:
	memunmap(cdev->tag_virt);
err_free:
	kfree(cdev);
	return ret;
}

static void __exit ccci_core_exit(void)
{
	struct ccci_device *cdev = g_ccci_dev;

	if (!cdev)
		return;

	/*
	 * ponytail: no port clients exist yet, so unload only warns. Take a
	 * module ref in ccci_port_register() once a real client lands.
	 */
	WARN_ON(!list_empty(&cdev->ports));

	g_ccci_dev = NULL;
	ccci_debugfs_exit(cdev);

	if (cdev->tag_virt)
		memunmap(cdev->tag_virt);

	kfree(cdev);
	pr_info("CCCI: core removed\n");
}

module_init(ccci_core_init);
module_exit(ccci_core_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek CCCI Core Driver");
MODULE_AUTHOR("Furruka");
