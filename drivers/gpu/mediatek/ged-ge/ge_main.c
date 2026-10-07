// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 *
 * Ported (GE subset) from drivers/gpu/mediatek/ged/src/ged_main.c
 * (crdroid-16-xaga).  Only the gralloc-extra (GE) bridge commands are kept;
 * the GPU DVFS / 3D-fence / event-bridge commands are intentionally omitted
 * because they belong to the GPU stack and this driver must not touch it.
 *
 * Exposes /proc/ged speaking the vendor GED_BRIDGE_PACKAGE ioctl protocol.
 */

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "ged_ge_abi.h"
#include "ged_ge.h"


#define GED_PROC_NAME "ged"

static long ged_dispatch(struct GED_BRIDGE_PACKAGE *pkg)
{
	void *pvIn = NULL, *pvOut = NULL;
	size_t inSize = 0, outSize = 0;
	int ret = 0;

	/* GE_ALLOC has a variable-length region_sizes[] tail. */
	if (GED_GET_BRIDGE_ID(pkg->ui32FunctionID) ==
	    GED_BRIDGE_COMMAND_GE_ALLOC)
		inSize = sizeof(struct GED_BRIDGE_IN_GE_ALLOC) +
			 (size_t)sizeof(uint32_t) * GED_GE_ALLOC_STRUCT_MAX;
	else
		inSize = pkg->i32InBufferSize;

	if (inSize > 0) {
		pvIn = kzalloc(inSize, GFP_KERNEL);
		if (!pvIn)
			return -ENOMEM;
		if (copy_from_user(pvIn, pkg->pvParamIn, inSize))
			goto out_free_in;
	}

	if (pkg->i32OutBufferSize > 0) {
		outSize = pkg->i32OutBufferSize;
		pvOut = kzalloc(outSize, GFP_KERNEL);
		if (!pvOut)
			goto out_free_in;
	}

	switch (GED_GET_BRIDGE_ID(pkg->ui32FunctionID)) {
	case GED_BRIDGE_COMMAND_GE_ALLOC: {
		struct GED_BRIDGE_IN_GE_ALLOC *in = pvIn;
		struct GED_BRIDGE_OUT_GE_ALLOC *out = pvOut;

		if (!in || !out ||
		    inSize < sizeof(*in) ||
		    outSize < sizeof(*out))
			goto out_bad;
		out->ge_fd = ged_ge_alloc(in->region_num, in->region_sizes);
		out->eError = (out->ge_fd >= 0) ? 0 : -1;
		break;
	}
	case GED_BRIDGE_COMMAND_GE_GET: {
		struct GED_BRIDGE_IN_GE_GET *in = pvIn;
		struct GED_BRIDGE_OUT_GE_GET *out = pvOut;

		if (!in || !out ||
		    inSize < sizeof(*in) ||
		    outSize < sizeof(*out))
			goto out_bad;
		out->eError = ged_ge_get(in->ge_fd, in->region_id,
					 in->uint32_offset,
					 in->uint32_size, out->data);
		break;
	}
	case GED_BRIDGE_COMMAND_GE_SET: {
		struct GED_BRIDGE_IN_GE_SET *in = pvIn;
		struct GED_BRIDGE_OUT_GE_SET *out = pvOut;

		if (!in || !out ||
		    inSize < sizeof(*in) ||
		    outSize < sizeof(*out))
			goto out_bad;
		out->eError = ged_ge_set(in->ge_fd, in->region_id,
					 in->uint32_offset,
					 in->uint32_size, in->data);
		break;
	}
	case GED_BRIDGE_COMMAND_GE_INFO: {
		struct GED_BRIDGE_IN_GE_INFO *in = pvIn;
		struct GED_BRIDGE_OUT_GE_INFO *out = pvOut;

		if (!in || !out ||
		    inSize < sizeof(*in) ||
		    outSize < sizeof(*out))
			goto out_bad;
		ret = ged_ge_info(in->ge_fd, &out->unique_id);
		out->eError = ret;
		break;
	}
	default:
		ret = -ENOTTY;
		goto out_copy;
	}

out_copy:
	if (pvOut && pkg->i32OutBufferSize > 0)
		if (copy_to_user(pkg->pvParamOut, pvOut, pkg->i32OutBufferSize))
			ret = -EFAULT;
	goto out_free;

out_bad:
	ret = -EINVAL;
	goto out_free;

out_free_in:
	ret = -EFAULT;
out_free:
	kfree(pvIn);
	kfree(pvOut);
	return ret;
}

static long ged_ioctl(struct file *pFile,
		      unsigned int ioctlCmd, unsigned long arg)
{
	struct GED_BRIDGE_PACKAGE *pkgUM = (void __user *)arg;
	struct GED_BRIDGE_PACKAGE pkgKM;

	if (!pkgUM)
		return -EFAULT;
	if (copy_from_user(&pkgKM, pkgUM, sizeof(pkgKM)))
		return -EFAULT;

	return ged_dispatch(&pkgKM);
}

static const struct proc_ops ged_proc_ops = {
	.proc_ioctl	= ged_ioctl,
#ifdef CONFIG_COMPAT
	.proc_compat_ioctl = ged_ioctl,
#endif
};

static int __init ged_ge_bridge_init(void)
{
	struct proc_dir_entry *entry;
	int ret;

	ret = ged_ge_store_init();
	if (ret)
		return ret;

	entry = proc_create(GED_PROC_NAME, 0664, NULL, &ged_proc_ops);
	if (!entry) {
		ged_ge_store_exit();
		return -ENOMEM;
	}
	return 0;
}

static void __exit ged_ge_bridge_exit(void)
{
	remove_proc_entry(GED_PROC_NAME, NULL);
	ged_ge_store_exit();
}

module_init(ged_ge_bridge_init);
module_exit(ged_ge_bridge_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek gralloc-extra (GE) region store (/proc/ged)");
