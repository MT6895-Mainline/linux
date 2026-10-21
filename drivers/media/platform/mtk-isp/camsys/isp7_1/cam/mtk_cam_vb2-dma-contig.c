// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 *
 * DMA contig memory allocator for the mtk-cam videobuf2 queues.
 * Adapted from videobuf2-dma-contig.c for the kernel 7.2 API.
 */

#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>

#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-memops.h>

#include "mtk_cam_vb2-dma-contig.h"

struct mtk_cam_vb2_buf {
	struct device			*dev;
	void				*vaddr;
	unsigned long			size;
	void				*cookie;
	dma_addr_t			dma_addr;
	unsigned long			attrs;
	enum dma_data_direction		dma_dir;
	struct sg_table			*dma_sgt;
	struct frame_vector		*vec;

	/* MMAP related */
	struct vb2_vmarea_handler	handler;
	refcount_t			refcount;
	struct sg_table			*sgt_base;

	/* DMABUF related */
	struct dma_buf_attachment	*db_attach;

	struct vb2_buffer		*vb;
};

/*********************************************/
/*        scatterlist table functions        */
/*********************************************/

static unsigned long mtk_cam_vb2_get_contiguous_size(struct sg_table *sgt)
{
	struct scatterlist *s;
	dma_addr_t expected = sg_dma_address(sgt->sgl);
	unsigned int i;
	unsigned long size = 0;

	for_each_sgtable_dma_sg(sgt, s, i) {
		if (sg_dma_address(s) != expected)
			break;
		expected += sg_dma_len(s);
		size += sg_dma_len(s);
	}
	return size;
}

/*********************************************/
/*         callbacks for all buffers         */
/*********************************************/

static void *mtk_cam_vb2_cookie(struct vb2_buffer *vb, void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;

	return &buf->dma_addr;
}

static void *mtk_cam_vb2_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;

	if (buf->vaddr)
		return buf->vaddr;

	if (buf->db_attach) {
		struct iosys_map map;

		if (!dma_buf_vmap_unlocked(buf->db_attach->dmabuf, &map))
			buf->vaddr = map.vaddr;
	}
	return buf->vaddr;
}

static unsigned int mtk_cam_vb2_num_users(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;

	return refcount_read(&buf->refcount);
}

static void mtk_cam_vb2_prepare(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	struct sg_table *sgt = buf->dma_sgt;

	if (buf->vb && buf->vb->skip_cache_sync_on_prepare)
		return;

	if (!sgt)
		return;

	dma_sync_sgtable_for_device(buf->dev, sgt, buf->dma_dir);
}

static void mtk_cam_vb2_finish(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	struct sg_table *sgt = buf->dma_sgt;

	if (buf->vb && buf->vb->skip_cache_sync_on_finish)
		return;

	if (!sgt)
		return;

	dma_sync_sgtable_for_cpu(buf->dev, sgt, buf->dma_dir);
}

/*********************************************/
/*        callbacks for MMAP buffers         */
/*********************************************/

static void mtk_cam_vb2_put(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;

	if (!refcount_dec_and_test(&buf->refcount))
		return;

	if (buf->sgt_base) {
		sg_free_table(buf->sgt_base);
		kfree(buf->sgt_base);
	}
	dma_free_attrs(buf->dev, buf->size, buf->cookie,
		       buf->dma_addr, buf->attrs);
	put_device(buf->dev);
	kfree(buf);
}

static void *mtk_cam_vb2_alloc(struct vb2_buffer *vb,
				struct device *dev,
				unsigned long size)
{
	struct mtk_cam_vb2_buf *buf;

	if (WARN_ON(!dev))
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->attrs = vb->vb2_queue->dma_attrs;
	buf->dma_dir = vb->vb2_queue->dma_dir;
	buf->vb = vb;
	buf->size = size;
	/* Prevent the device from being released while the buffer is used */
	buf->dev = get_device(dev);

	buf->cookie = dma_alloc_attrs(buf->dev,
				      buf->size,
				      &buf->dma_addr,
				      GFP_KERNEL | vb->vb2_queue->gfp_flags,
				      buf->attrs);
	if (!buf->cookie) {
		dev_err(dev, "dma alloc of size %lu failed\n", size);
		put_device(buf->dev);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	if ((buf->attrs & DMA_ATTR_NO_KERNEL_MAPPING) == 0)
		buf->vaddr = buf->cookie;

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = mtk_cam_vb2_put;
	buf->handler.arg = buf;

	refcount_set(&buf->refcount, 1);

	return buf;
}

static int mtk_cam_vb2_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	int ret;

	if (!buf) {
		pr_err("No buffer to map\n");
		return -EINVAL;
	}

	ret = dma_mmap_attrs(buf->dev, vma, buf->cookie, buf->dma_addr,
			     buf->size, buf->attrs);
	if (ret) {
		pr_err("Remapping memory failed, error: %d\n", ret);
		return ret;
	}

	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_private_data	= &buf->handler;
	vma->vm_ops		= &vb2_common_vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

/*********************************************/
/*         DMABUF ops for exporters          */
/*********************************************/

struct mtk_cam_vb2_attachment {
	struct sg_table sgt;
	enum dma_data_direction dma_dir;
};

static int mtk_cam_vb2_dmabuf_ops_attach(struct dma_buf *dbuf,
	struct dma_buf_attachment *dbuf_attach)
{
	struct mtk_cam_vb2_attachment *attach;
	unsigned int i;
	struct scatterlist *rd, *wr;
	struct sg_table *sgt;
	struct mtk_cam_vb2_buf *buf = dbuf->priv;
	int ret;

	attach = kzalloc(sizeof(*attach), GFP_KERNEL);
	if (!attach)
		return -ENOMEM;

	sgt = &attach->sgt;
	/*
	 * Copy the buf->sgt_base scatter list to the attachment, as we can't
	 * map the same scatter list to multiple attachments at the same time.
	 */
	ret = sg_alloc_table(sgt, buf->sgt_base->orig_nents, GFP_KERNEL);
	if (ret) {
		kfree(attach);
		return -ENOMEM;
	}

	rd = buf->sgt_base->sgl;
	wr = sgt->sgl;
	for (i = 0; i < sgt->orig_nents; ++i) {
		sg_set_page(wr, sg_page(rd), rd->length, rd->offset);
		rd = sg_next(rd);
		wr = sg_next(wr);
	}

	attach->dma_dir = DMA_NONE;
	dbuf_attach->priv = attach;

	return 0;
}

static void mtk_cam_vb2_dmabuf_ops_detach(struct dma_buf *dbuf,
	struct dma_buf_attachment *db_attach)
{
	struct mtk_cam_vb2_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	if (!attach)
		return;

	sgt = &attach->sgt;

	/* release the scatterlist cache */
	if (attach->dma_dir != DMA_NONE)
		/*
		 * Cache sync can be skipped here, as the memory is
		 * allocated from device coherent memory, which means the
		 * memory locations do not require any explicit cache
		 * maintenance prior or after being used by the device.
		 */
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
	sg_free_table(sgt);
	kfree(attach);
	db_attach->priv = NULL;
}

static struct sg_table *mtk_cam_vb2_dmabuf_ops_map(
	struct dma_buf_attachment *db_attach, enum dma_data_direction dma_dir)
{
	struct mtk_cam_vb2_attachment *attach = db_attach->priv;
	struct sg_table *sgt;

	sgt = &attach->sgt;
	/* return previously mapped sg table */
	if (attach->dma_dir == dma_dir)
		return sgt;

	/* release any previous cache */
	if (attach->dma_dir != DMA_NONE) {
		dma_unmap_sgtable(db_attach->dev, sgt, attach->dma_dir,
				  DMA_ATTR_SKIP_CPU_SYNC);
		attach->dma_dir = DMA_NONE;
	}

	/*
	 * mapping to the client with new direction, no cache sync
	 * required see comment in mtk_cam_vb2_dmabuf_ops_detach()
	 */
	if (dma_map_sgtable(db_attach->dev, sgt, dma_dir,
			    DMA_ATTR_SKIP_CPU_SYNC)) {
		pr_err("failed to map scatterlist\n");
		return ERR_PTR(-EIO);
	}

	attach->dma_dir = dma_dir;

	return sgt;
}

static void mtk_cam_vb2_dmabuf_ops_unmap(struct dma_buf_attachment *db_attach,
	struct sg_table *sgt, enum dma_data_direction dma_dir)
{
	/* nothing to be done here */
}

static void mtk_cam_vb2_dmabuf_ops_release(struct dma_buf *dbuf)
{
	/* drop reference obtained in mtk_cam_vb2_get_dmabuf */
	mtk_cam_vb2_put(dbuf->priv);
}

static int mtk_cam_vb2_dmabuf_ops_vmap(struct dma_buf *dbuf,
				       struct iosys_map *map)
{
	struct mtk_cam_vb2_buf *buf = dbuf->priv;

	if (!buf->vaddr)
		return -EINVAL;

	iosys_map_set_vaddr(map, buf->vaddr);

	return 0;
}

static int mtk_cam_vb2_dmabuf_ops_mmap(struct dma_buf *dbuf,
	struct vm_area_struct *vma)
{
	return mtk_cam_vb2_mmap(dbuf->priv, vma);
}

static const struct dma_buf_ops mtk_cam_vb2_dmabuf_ops = {
	.attach = mtk_cam_vb2_dmabuf_ops_attach,
	.detach = mtk_cam_vb2_dmabuf_ops_detach,
	.map_dma_buf = mtk_cam_vb2_dmabuf_ops_map,
	.unmap_dma_buf = mtk_cam_vb2_dmabuf_ops_unmap,
	.vmap = mtk_cam_vb2_dmabuf_ops_vmap,
	.mmap = mtk_cam_vb2_dmabuf_ops_mmap,
	.release = mtk_cam_vb2_dmabuf_ops_release,
};

static struct sg_table *mtk_cam_vb2_get_base_sgt(struct mtk_cam_vb2_buf *buf)
{
	int ret;
	struct sg_table *sgt;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		dev_err(buf->dev, "failed to alloc sg table\n");
		return NULL;
	}

	ret = dma_get_sgtable_attrs(buf->dev, sgt, buf->cookie, buf->dma_addr,
				    buf->size, buf->attrs);
	if (ret < 0) {
		dev_err(buf->dev, "failed to get scatterlist from DMA API\n");
		kfree(sgt);
		return NULL;
	}

	return sgt;
}

static struct dma_buf *mtk_cam_vb2_get_dmabuf(struct vb2_buffer *vb,
					      void *buf_priv,
					      unsigned long flags)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.ops = &mtk_cam_vb2_dmabuf_ops;
	exp_info.size = buf->size;
	exp_info.flags = flags;
	exp_info.priv = buf;

	if (!buf->sgt_base)
		buf->sgt_base = mtk_cam_vb2_get_base_sgt(buf);

	if (WARN_ON(!buf->sgt_base))
		return NULL;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf))
		return NULL;

	/* dmabuf keeps reference to vb2 buffer */
	refcount_inc(&buf->refcount);

	return dbuf;
}

/*********************************************/
/*       callbacks for DMABUF buffers        */
/*********************************************/

static int mtk_cam_vb2_map_dmabuf(void *mem_priv)
{
	struct mtk_cam_vb2_buf *buf = mem_priv;
	struct sg_table *sgt;
	unsigned long contig_size;

	if (WARN_ON(!buf->db_attach)) {
		pr_err("trying to pin a non attached buffer\n");
		return -EINVAL;
	}

	if (WARN_ON(buf->dma_sgt)) {
		pr_err("dmabuf buffer is already pinned\n");
		return 0;
	}

	/* get the associated scatterlist for this buffer */
	sgt = dma_buf_map_attachment_unlocked(buf->db_attach, buf->dma_dir);
	if (IS_ERR(sgt)) {
		pr_err("Error getting dmabuf scatterlist\n");
		return -EINVAL;
	}

	/* checking if dmabuf is big enough to store contiguous chunk */
	contig_size = mtk_cam_vb2_get_contiguous_size(sgt);
	{
		struct scatterlist *sg;
		unsigned int k;

		dev_info(buf->dev,
			 "XAGA-MAPDMABUF dev=%s want=%lu dmabuf=%zu nents=%u contig=%lu\n",
			 dev_name(buf->dev), buf->size,
			 buf->db_attach->dmabuf->size, sgt->nents, contig_size);
		for_each_sgtable_dma_sg(sgt, sg, k) {
			if (k >= 8)
				break;
			dev_info(buf->dev,
				 "  XAGA-SG[%u] dma=0x%llx len=%u\n",
				 k, (u64)sg_dma_address(sg), sg_dma_len(sg));
		}
	}
	if (contig_size < buf->size) {
		pr_err("contiguous chunk is too small %lu/%lu\n",
		       contig_size, buf->size);
		dma_buf_unmap_attachment_unlocked(buf->db_attach, sgt,
						  buf->dma_dir);
		return -EFAULT;
	}

	buf->dma_addr = sg_dma_address(sgt->sgl);
	buf->dma_sgt = sgt;
	buf->vaddr = NULL;

	return 0;
}

static void mtk_cam_vb2_unmap_dmabuf(void *mem_priv)
{
	struct mtk_cam_vb2_buf *buf = mem_priv;
	struct sg_table *sgt = buf->dma_sgt;
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(buf->vaddr);

	if (WARN_ON(!buf->db_attach)) {
		pr_err("trying to unpin a not attached buffer\n");
		return;
	}

	if (WARN_ON(!sgt)) {
		pr_err("dmabuf buffer is already unpinned\n");
		return;
	}

	if (buf->vaddr) {
		dma_buf_vunmap_unlocked(buf->db_attach->dmabuf, &map);
		buf->vaddr = NULL;
	}
	dma_buf_unmap_attachment_unlocked(buf->db_attach, sgt, buf->dma_dir);

	buf->dma_addr = 0;
	buf->dma_sgt = NULL;
}

static void mtk_cam_vb2_detach_dmabuf(void *mem_priv)
{
	struct mtk_cam_vb2_buf *buf = mem_priv;

	/* if vb2 works correctly you should never detach mapped buffer */
	if (WARN_ON(buf->dma_addr))
		mtk_cam_vb2_unmap_dmabuf(buf);

	/* detach this attachment */
	dma_buf_detach(buf->db_attach->dmabuf, buf->db_attach);
	kfree(buf);
}

static void *mtk_cam_vb2_attach_dmabuf(struct vb2_buffer *vb,
				       struct device *dev,
				       struct dma_buf *dbuf,
				       unsigned long size)
{
	struct mtk_cam_vb2_buf *buf;
	struct dma_buf_attachment *dba;

	if (dbuf->size < size)
		return ERR_PTR(-EFAULT);

	if (WARN_ON(!dev))
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->dev = dev;
	buf->vb = vb;
	buf->dma_dir = vb->vb2_queue->dma_dir;

	/* create attachment for the dmabuf with the user device */
	dba = dma_buf_attach(dbuf, buf->dev);
	if (IS_ERR(dba)) {
		pr_err("failed to attach dmabuf\n");
		kfree(buf);
		return dba;
	}

	buf->size = size;
	buf->db_attach = dba;

	return buf;
}

/*********************************************/
/*       DMA CONTIG exported functions       */
/*********************************************/

const struct vb2_mem_ops mtk_cam_dma_contig_memops = {
	.alloc		= mtk_cam_vb2_alloc,
	.put		= mtk_cam_vb2_put,
	.get_dmabuf	= mtk_cam_vb2_get_dmabuf,
	.cookie		= mtk_cam_vb2_cookie,
	.vaddr		= mtk_cam_vb2_vaddr,
	.mmap		= mtk_cam_vb2_mmap,
	.get_userptr	= NULL,
	.put_userptr	= NULL,
	.prepare	= mtk_cam_vb2_prepare,
	.finish		= mtk_cam_vb2_finish,
	.map_dmabuf	= mtk_cam_vb2_map_dmabuf,
	.unmap_dmabuf	= mtk_cam_vb2_unmap_dmabuf,
	.attach_dmabuf	= mtk_cam_vb2_attach_dmabuf,
	.detach_dmabuf	= mtk_cam_vb2_detach_dmabuf,
	.num_users	= mtk_cam_vb2_num_users,
};
EXPORT_SYMBOL_GPL(mtk_cam_dma_contig_memops);

void mtk_cam_vb2_sync_for_device(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	struct sg_table *sgt = buf->dma_sgt;

	if (!sgt)
		return;

	dma_sync_sgtable_for_device(buf->dev, sgt, buf->dma_dir);
}

void mtk_cam_vb2_sync_for_cpu(void *buf_priv)
{
	struct mtk_cam_vb2_buf *buf = buf_priv;
	struct sg_table *sgt = buf->dma_sgt;

	if (!sgt)
		return;

	dma_sync_sgtable_for_cpu(buf->dev, sgt, buf->dma_dir);
}

MODULE_DESCRIPTION("DMA-contig memory handling routines for mtk-cam videobuf2");
MODULE_LICENSE("GPL");
