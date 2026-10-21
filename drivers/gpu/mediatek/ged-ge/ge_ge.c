// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 *
 * Ported from drivers/gpu/mediatek/ged/src/ged_ge.c (crdroid-16-xaga).
 *
 * A "GE entry" is an anonymous-inode file that owns a small array of named
 * regions.  Userspace allocates one per gralloc buffer, writes per-buffer
 * properties into it with GE_SET, and reads them back with GE_GET.  The
 * gralloc_extra library (vendor libgralloc_extra.so) then answers
 * gralloc_extra_query() from these regions via ged_ge_get().
 *
 * This is deliberately a pure data store: no GPU, no DRM, no fences.  It does
 * not touch the GPU subsystem in any way (verified: panthor/panfrost are
 * unaffected).
 */

#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include "ged_ge_abi.h"
#include "ged_ge.h"

/* Must match the item count in the userspace region_sizes[] array. */
#define GE_ALLOC_STRUCT_NUM 18
#define GE_MAX_REGION_SIZE  8192

struct GEEntry {
	uint64_t unique_id;
	struct file *file;
	int alloc_fd;
	int region_num;
	void *data;
	uint32_t *region_sizes;   /* sub data */
	uint32_t **region_data;   /* sub data */
	struct list_head ge_entry_list;
};

static struct kmem_cache *gPoolCache;
static LIST_HEAD(ge_entry_list_head);
static DEFINE_SPINLOCK(ge_entry_list_lock);
static DEFINE_SPINLOCK(ge_raf_lock);

static uint64_t gen_unique_id(void)
{
	static uint64_t serial;
	unsigned long flags;
	uint64_t ret;

	spin_lock_irqsave(&ge_raf_lock, flags);
	ret = serial++;
	spin_unlock_irqrestore(&ge_raf_lock, flags);
	return ret;
}

static int ge_entry_release(struct inode *inode, struct file *file)
{
	unsigned long flags;
	int i;
	struct GEEntry *entry = file->private_data;

	spin_lock_irqsave(&ge_entry_list_lock, flags);
	list_del(&entry->ge_entry_list);
	spin_unlock_irqrestore(&ge_entry_list_lock, flags);

	for (i = 0; i < entry->region_num; ++i)
		kfree(entry->region_data[i]);
	kfree(entry->region_sizes);
	kmem_cache_free(gPoolCache, entry);
	return 0;
}

static const struct file_operations GEEntry_fops = {
	.owner = THIS_MODULE,
	.release = ge_entry_release,
};

int ged_ge_store_init(void)
{
	gPoolCache = kmem_cache_create("gralloc_extra_ge",
				       sizeof(struct GEEntry), 0, 0, NULL);
	return gPoolCache ? 0 : -ENOMEM;
}

void ged_ge_store_exit(void)
{
	kmem_cache_destroy(gPoolCache);
	gPoolCache = NULL;
}

int ged_ge_alloc(int region_num, uint32_t *region_sizes)
{
	unsigned long flags;
	int i;
	struct GEEntry *entry;

	entry = kmem_cache_zalloc(gPoolCache, GFP_KERNEL);
	if (!entry)
		return -1;

	/* The vendor hardcodes the region count for determinism. */
	region_num = GE_ALLOC_STRUCT_NUM;

	entry->alloc_fd = get_unused_fd_flags(O_CLOEXEC);
	if (entry->alloc_fd < 0)
		goto err_fd;

	entry->file = anon_inode_getfile("gralloc_extra",
					 &GEEntry_fops, entry, O_RDWR);
	if (IS_ERR(entry->file))
		goto err_entry_file;

	entry->region_num = region_num;
	entry->data = kzalloc((sizeof(uint32_t) + sizeof(uint32_t *)) * region_num,
			      GFP_KERNEL);
	if (!entry->data)
		goto err_kmalloc;

	entry->region_sizes = (uint32_t *)entry->data;
	entry->region_data = (uint32_t **)(entry->region_sizes + region_num);
	for (i = 0; i < region_num; ++i) {
		if (region_sizes[i] == 0 ||
		    region_sizes[i] > GE_MAX_REGION_SIZE) {
			pr_debug("[ged-ge] bad region_sizes[%d]:%u\n",
				 i, region_sizes[i]);
			goto err_parameter;
		}
		entry->region_sizes[i] = region_sizes[i];
	}

	entry->unique_id = gen_unique_id();

	spin_lock_irqsave(&ge_entry_list_lock, flags);
	list_add_tail(&entry->ge_entry_list, &ge_entry_list_head);
	spin_unlock_irqrestore(&ge_entry_list_lock, flags);

	fd_install(entry->alloc_fd, entry->file);
	return entry->alloc_fd;

err_parameter:
	kfree(entry->data);
err_kmalloc:
	put_unused_fd(entry->alloc_fd);
err_entry_file:
	kmem_cache_free(gPoolCache, entry);
err_fd:
	return -1;
}

static int valid_parameters(struct GEEntry *entry, int region_id,
			   int u32_offset, int u32_size)
{
	if (region_id < 0 || region_id >= entry->region_num ||
	    u32_offset < 0 || u32_size < 0 ||
	    u32_offset * sizeof(uint32_t) > entry->region_sizes[region_id] ||
	    u32_size * sizeof(uint32_t) > entry->region_sizes[region_id] ||
	    (u32_offset + u32_size) * sizeof(uint32_t) >
			entry->region_sizes[region_id])
		return -EFAULT;
	return 0;
}

int ged_ge_get(int ge_fd, int region_id, int u32_offset,
	       int u32_size, uint32_t *output_data)
{
	unsigned long flags;
	int err = 0;
	uint32_t i;
	struct GEEntry *entry;
	uint32_t *pregion_data;
	struct file *file;

	file = fget(ge_fd);
	if (file == NULL || file->f_op != &GEEntry_fops) {
		pr_debug("[ged-ge] invalid ge_fd %d\n", ge_fd);
		if (file)
			fput(file);
		return -EFAULT;
	}

	entry = file->private_data;

	if (valid_parameters(entry, region_id, u32_offset, u32_size)) {
		err = -EFAULT;
		goto out;
	}

	spin_lock_irqsave(&ge_raf_lock, flags);
	pregion_data = entry->region_data[region_id];
	spin_unlock_irqrestore(&ge_raf_lock, flags);

	if (pregion_data) {
		for (i = 0; i < u32_size; ++i)
			output_data[i] = pregion_data[u32_offset + i];
	} else {
		for (i = 0; i < u32_size; ++i)
			output_data[i] = 0;
	}
out:
	fput(file);
	return err;
}

int ged_ge_set(int ge_fd, int region_id, int u32_offset,
	       int u32_size, uint32_t *input_data)
{
	unsigned long flags;
	int err = 0;
	uint32_t i;
	struct GEEntry *entry;
	uint32_t *pregion_data;
	struct file *file;

	file = fget(ge_fd);
	if (file == NULL || file->f_op != &GEEntry_fops) {
		pr_debug("[ged-ge] invalid ge_fd %d\n", ge_fd);
		if (file)
			fput(file);
		return -EFAULT;
	}

	entry = file->private_data;

	if (valid_parameters(entry, region_id, u32_offset, u32_size)) {
		err = -EFAULT;
		goto out;
	}

	spin_lock_irqsave(&ge_raf_lock, flags);
	while (!entry->region_data[region_id]) {
		void *data;

		spin_unlock_irqrestore(&ge_raf_lock, flags);
		data = kzalloc(entry->region_sizes[region_id], GFP_KERNEL);
		if (!data) {
			err = -EFAULT;
			goto out;
		}
		spin_lock_irqsave(&ge_raf_lock, flags);
		if (entry->region_data[region_id]) {
			spin_unlock_irqrestore(&ge_raf_lock, flags);
			kfree(data);
			spin_lock_irqsave(&ge_raf_lock, flags);
			break;
		}
		entry->region_data[region_id] = data;
	}
	pregion_data = entry->region_data[region_id];
	spin_unlock_irqrestore(&ge_raf_lock, flags);

	for (i = 0; i < u32_size; ++i)
		pregion_data[u32_offset + i] = input_data[i];
out:
	fput(file);
	return err;
}

int ged_ge_info(int ge_fd, uint64_t *unique_id)
{
	struct GEEntry *entry;
	struct file *file;

	file = fget(ge_fd);
	if (file == NULL || file->f_op != &GEEntry_fops) {
		if (file)
			fput(file);
		return -EFAULT;
	}
	entry = file->private_data;
	*unique_id = entry->unique_id;
	fput(file);
	return 0;
}
