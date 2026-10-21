// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2018 MediaTek Inc.
//
// Mainline port: CCD shared buffers are coherent DMA allocations (contiguous
// by construction, which the composer needs because libccd writes CQ
// descriptors as byte offsets inside one buffer).  The internal dma-heap
// allocation API (dma_heap_find/dma_heap_buffer_alloc) was removed, and the
// mainline "system" heap is neither contiguous nor uncached, so instead of
// allocating from a heap we export the coherent allocation itself as a
// dma-buf.  That is what libccd needs: a valid fd it can mmap to write the CQ
// table (see isp/no-hal-loader/).

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_data/mtk_ccd.h>
#include <linux/remoteproc/mtk_ccd_mem.h>
#include <linux/rpmsg/mtk_ccd_rpmsg.h>
#include <uapi/linux/mtk_ccd_controls.h>

#define CCD_ALLOCATE_MAX_BUFFER_SIZE 0x20000000UL /*512MB*/

struct mtk_ccd_buf;

struct mtk_ccd_buf {
	struct device *dev;
	void *vaddr;
	unsigned long size;
	dma_addr_t dma_addr;

	/* DMABUF related (mainline port) */
	struct dma_buf *dbuf;
};

static int ccd_dmabuf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	struct mtk_ccd_buf *buf = dbuf->priv;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (size > buf->size)
		return -EINVAL;

	return dma_mmap_attrs(buf->dev, vma, buf->vaddr, buf->dma_addr,
			      buf->size, 0);
}

static struct sg_table *ccd_dmabuf_map_dma_buf(
					struct dma_buf_attachment *attach,
					enum dma_data_direction dir)
{
	struct mtk_ccd_buf *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	/* one contiguous chunk: the coherent allocation itself */
	sg_set_page(sgt->sgl, virt_to_page(buf->vaddr), buf->size,
		    offset_in_page(buf->vaddr));
	sg_dma_address(sgt->sgl) = buf->dma_addr;
	sg_dma_len(sgt->sgl) = buf->size;

	return sgt;
}

static void ccd_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				     struct sg_table *sgt,
				     enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static void ccd_dmabuf_release(struct dma_buf *dbuf)
{
	/* the coherent memory is owned by mtk_ccd_buf_put() */
}

static const struct dma_buf_ops ccd_dmabuf_ops = {
	.map_dma_buf = ccd_dmabuf_map_dma_buf,
	.unmap_dma_buf = ccd_dmabuf_unmap_dma_buf,
	.mmap = ccd_dmabuf_mmap,
	.release = ccd_dmabuf_release,
};

static struct mtk_ccd_buf *mtk_ccd_buf_alloc(
					   struct device *dev, unsigned long size)
{
	struct mtk_ccd_buf *buf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->vaddr = dma_alloc_attrs(dev, size, &buf->dma_addr, GFP_KERNEL, 0);
	if (!buf->vaddr) {
		pr_info("ccd dma alloc fail\n");
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	buf->dev = get_device(dev);
	buf->size = size;

	/* Export the coherent allocation as a dma-buf so the userspace
	 * composer can be handed an fd (working_buf_fd / msg_buf_fd) to mmap
	 * and write the CQ table into. */
	exp_info.ops = &ccd_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR | O_CLOEXEC;
	exp_info.priv = buf;
	buf->dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(buf->dbuf)) {
		pr_info("ccd dma_buf export fail: %ld\n", PTR_ERR(buf->dbuf));
		buf->dbuf = NULL;
		dma_free_attrs(buf->dev, buf->size, buf->vaddr, buf->dma_addr, 0);
		put_device(buf->dev);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	return buf;
}

static void mtk_ccd_buf_put(struct mtk_ccd_buf *buf)
{
	if (buf->dbuf) {
		dma_buf_put(buf->dbuf);
		buf->dbuf = NULL;
	}

	dma_free_attrs(buf->dev, buf->size, buf->vaddr, buf->dma_addr, 0);

	put_device(buf->dev);
	kfree(buf);
}

static dma_addr_t mtk_ccd_buf_get_daddr(struct mtk_ccd_buf *buf)
{
	return buf->dma_addr;
}

static void *mtk_ccd_buf_get_vaddr(struct mtk_ccd_buf *buf)
{
	return buf->vaddr;
}

struct dma_buf *mtk_ccd_get_buffer_dmabuf(struct mtk_ccd *ccd, void *mem_priv)
{
	struct mtk_ccd_buf *buf = mem_priv;

	if (ccd == NULL || buf == NULL) {
		pr_info("mtk_ccd or mem_priv is NULL\n");
		return NULL;
	}

	return buf->dbuf;
}
EXPORT_SYMBOL_GPL(mtk_ccd_get_buffer_dmabuf);

struct mtk_ccd_memory *mtk_ccd_mem_init(struct device *dev)
{
	struct mtk_ccd_memory *ccd_memory;

	ccd_memory = kzalloc(sizeof(*ccd_memory), GFP_KERNEL);
	if (!ccd_memory)
		return NULL;

	ccd_memory->dev = dev;
	ccd_memory->num_buffers = 0;
	mutex_init(&ccd_memory->mmap_lock);

	return ccd_memory;
}
EXPORT_SYMBOL_GPL(mtk_ccd_mem_init);

void mtk_ccd_mem_release(struct mtk_ccd *ccd)
{
	struct mtk_ccd_memory *ccd_memory = ccd->ccd_memory;

	kfree(ccd_memory);
}
EXPORT_SYMBOL_GPL(mtk_ccd_mem_release);

void *mtk_ccd_get_buffer(struct mtk_ccd *ccd,
			 struct mem_obj *mem_buff_data)
{
	void *va;
	dma_addr_t da;
	unsigned int buffers;
	struct mtk_ccd_buf *buf;
	struct mtk_ccd_mem *ccd_buffer;
	struct mtk_ccd_memory *ccd_memory = ccd->ccd_memory;

	mem_buff_data->iova = 0;
	mem_buff_data->va = NULL;

	mutex_lock(&ccd_memory->mmap_lock);
	buffers = ccd_memory->num_buffers;
	if (mem_buff_data->len > CCD_ALLOCATE_MAX_BUFFER_SIZE ||
	    mem_buff_data->len == 0U ||
	    buffers >= MAX_NUMBER_OF_BUFFER) {
		dev_info(ccd_memory->dev,
			"%s: Failed: buffer len = %u num_buffers = %d !!\n",
			 __func__, mem_buff_data->len, buffers);
		mutex_unlock(&ccd_memory->mmap_lock);
		return ERR_PTR(-EINVAL);
	}

	ccd_buffer = &ccd_memory->bufs[buffers];
	buf = mtk_ccd_buf_alloc(ccd_memory->dev, mem_buff_data->len);
	ccd_buffer->mem_priv = buf;
	ccd_buffer->size = mem_buff_data->len;
	if (IS_ERR(ccd_buffer->mem_priv)) {
		dev_info(ccd_memory->dev, "%s: CCD buf allocation failed\n",
			__func__);
		mutex_unlock(&ccd_memory->mmap_lock);
		return ERR_PTR(-ENOMEM);
	}

	va = mtk_ccd_buf_get_vaddr(buf);
	da = mtk_ccd_buf_get_daddr(buf);
	mem_buff_data->iova = da;
	mem_buff_data->va = va;
	ccd_memory->num_buffers++;
	mutex_unlock(&ccd_memory->mmap_lock);

	dev_dbg(ccd_memory->dev,
		"Num_bufs = %d iova = %pad va = %p size = %d priv = %p\n",
		 ccd_memory->num_buffers, &mem_buff_data->iova,
		 mem_buff_data->va,
		 (unsigned int)ccd_buffer->size,
		 ccd_buffer->mem_priv);

	return ccd_buffer->mem_priv;
}
EXPORT_SYMBOL_GPL(mtk_ccd_get_buffer);

int mtk_ccd_put_buffer(struct mtk_ccd *ccd,
			struct mem_obj *mem_buff_data)
{
	struct mtk_ccd_buf *buf;
	void *va;
	dma_addr_t da;
	int ret = -EINVAL;
	struct mtk_ccd_mem *ccd_buffer;
	unsigned int buffer, num_buffers, last_buffer;
	struct mtk_ccd_memory *ccd_memory = ccd->ccd_memory;

	mutex_lock(&ccd_memory->mmap_lock);
	num_buffers = ccd_memory->num_buffers;
	if (num_buffers != 0U) {
		for (buffer = 0; buffer < num_buffers; buffer++) {
			ccd_buffer = &ccd_memory->bufs[buffer];
			buf = (struct mtk_ccd_buf *)ccd_buffer->mem_priv;
			va = mtk_ccd_buf_get_vaddr(buf);
			da = mtk_ccd_buf_get_daddr(buf);

			if (mem_buff_data->va == va &&
				mem_buff_data->len == ccd_buffer->size) {
				dev_dbg(ccd_memory->dev,
					"Free buff = %d iova = %pad va = %p, queue_num = %d\n",
					 buffer, &mem_buff_data->iova,
					 mem_buff_data->va,
					 num_buffers);
				mtk_ccd_buf_put(buf);
				last_buffer = num_buffers - 1U;
				if (last_buffer != buffer)
					ccd_memory->bufs[buffer] =
						ccd_memory->bufs[last_buffer];

				ccd_memory->bufs[last_buffer].mem_priv = NULL;
				ccd_memory->bufs[last_buffer].size = 0;
				ccd_memory->num_buffers--;
				ret = 0;
				break;
			}
		}
	}
	mutex_unlock(&ccd_memory->mmap_lock);

	if (ret != 0)
		dev_info(ccd_memory->dev,
			"Can not free memory va %p iova %pad len %u!\n",
			 mem_buff_data->va, &mem_buff_data->iova,
			 mem_buff_data->len);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_ccd_put_buffer);


int mtk_ccd_get_buffer_fd(struct mtk_ccd *ccd, void *mem_priv)
{
	struct mtk_ccd_buf *buf = mem_priv;
	int fd;

	if (ccd == NULL || buf == NULL || buf->dbuf == NULL) {
		pr_info("mtk_ccd or buf is NULL\n");
		return -EINVAL;
	}

	fd = dma_buf_fd(buf->dbuf, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		pr_info("mtk_ccd couldn't get fd from dma_buf\n");
		return -EINVAL;
	}

	/* increase file count */
	get_dma_buf(buf->dbuf);

	return fd;
}
EXPORT_SYMBOL_GPL(mtk_ccd_get_buffer_fd);


static int mtk_ccd_put_fd(struct mtk_ccd *ccd,
			struct mem_obj *mem_buff_data,
			unsigned int target_fd)
{
	/* The fd owns its own file reference and is closed by whoever holds
	 * it; the extra reference taken in mtk_ccd_get_buffer_fd() is dropped
	 * by mtk_ccd_buf_put() via dma_buf_put(). */
	return 0;
}

int mtk_ccd_put_buffer_fd(struct mtk_ccd *ccd,
				struct mem_obj *mem_buff_data,
				unsigned int target_fd)
{
	if (ccd == NULL || mem_buff_data == NULL) {
		pr_info("mtk_ccd or mem_obj is NULL\n");
		return -EINVAL;
	}
	return mtk_ccd_put_fd(ccd, mem_buff_data, target_fd);
}
EXPORT_SYMBOL_GPL(mtk_ccd_put_buffer_fd);

MODULE_IMPORT_NS("DMA_BUF");

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek ccd memory interface");
