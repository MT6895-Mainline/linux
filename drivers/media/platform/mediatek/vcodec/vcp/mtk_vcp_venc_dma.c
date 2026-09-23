// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/iosys-map.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <media/videobuf2-core.h>

#include "mtk_vcp_venc_dma.h"

void vcp_venc_dma_release(struct vcp_venc_dma_buffer *buffer)
{
	int i;

	if (!buffer)
		return;
	for (i = buffer->planes - 1; i >= 0; i--) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];

		if (p->sgt)
			dma_buf_unmap_attachment(p->attach, p->sgt,
						buffer->direction);
		if (p->attach && p->dbuf)
			dma_buf_detach(p->dbuf, p->attach);
		if (p->staging)
			dma_free_noncoherent(buffer->dev, p->staging_alloc,
					     p->staging, p->staging_dma,
					     buffer->direction);
		if (p->dbuf)
			dma_buf_put(p->dbuf);
	}
	put_device(buffer->dev);
	kfree(buffer);
	module_put(THIS_MODULE);
}

void vcp_venc_dma_pool_clear(struct vcp_venc_dma_pool *pool)
{
	struct vcp_venc_dma_buffer *b, *next;

	list_for_each_entry_safe(b, next, &pool->buffers, list) {
		list_del(&b->list);
		vcp_venc_dma_release(b);
	}
	pool->bytes = 0;
}

void vcp_venc_dma_recycle(struct vcp_venc_dma_pool *pool,
	struct vcp_venc_dma_buffer *buffer)
{
	size_t bytes = 0;
	unsigned int i;

	/* Direct mappings borrow the client's allocation; only private
	 * staging is worth pooling. */
	for (i = 0; i < buffer->planes; i++)
		if (buffer->plane[i].direct)
			goto release;
	for (i = 0; i < buffer->planes; i++)
		bytes += buffer->plane[i].size;
	if (pool->bytes > SZ_64M || bytes > SZ_64M - pool->bytes) {
		goto release;
	}
	/* A validated firmware return has ended DMA. Retain private storage,
	 * but release every client reference before caching the record.
	 */
	for (i = 0; i < buffer->planes; i++) {
		dma_buf_put(buffer->plane[i].dbuf);
		buffer->plane[i].dbuf = NULL;
	}
	buffer->cookie = 0;
	pool->bytes += bytes;
	list_add_tail(&buffer->list, &pool->buffers);
	return;
release:
	vcp_venc_dma_release(buffer);
}

static void venc_dma_reuse(struct vcp_venc_dma_buffer *buffer,
	struct vcp_venc_dma_pool *pool)
{
	struct vcp_venc_dma_buffer *b;
	unsigned int i;

	if (!pool)
		return;
	list_for_each_entry(b, &pool->buffers, list) {
		if (b->dev != buffer->dev || b->direction != buffer->direction ||
		    b->planes != buffer->planes)
			continue;
		for (i = 0; i < b->planes; i++)
			if (b->plane[i].size != buffer->plane[i].size)
				break;
		if (i != b->planes)
			continue;
		list_del(&b->list);
		for (i = 0; i < b->planes; i++) {
			buffer->plane[i].staging = b->plane[i].staging;
			buffer->plane[i].staging_dma = b->plane[i].staging_dma;
			buffer->plane[i].staging_alloc = b->plane[i].staging_alloc;
			b->plane[i].staging = NULL;
			b->plane[i].staging_alloc = 0;
			pool->bytes -= b->plane[i].size;
		}
		vcp_venc_dma_release(b);
		return;
	}
}

static struct vcp_venc_dma_buffer *venc_dma_import(struct device *dev,
	struct vb2_buffer *vb, enum dma_data_direction direction)
{
	struct vcp_venc_dma_buffer *buffer;
	unsigned int i;
	int ret;

	if (!dev || !vb || !vb->vb2_queue || !vb->num_planes ||
	    vb->num_planes > 3 ||
	    (direction != DMA_TO_DEVICE && direction != DMA_FROM_DEVICE) ||
	    (direction == DMA_FROM_DEVICE && vb->num_planes != 1) ||
	    (vb->memory != VB2_MEMORY_MMAP && vb->memory != VB2_MEMORY_DMABUF))
		return ERR_PTR(-EINVAL);
	buffer = kzalloc_obj(*buffer);
	if (!buffer)
		return ERR_PTR(-ENOMEM);
	INIT_LIST_HEAD(&buffer->list);
	buffer->direction = direction;
	buffer->planes = vb->num_planes;
	buffer->dev = get_device(dev);
	__module_get(THIS_MODULE);
	for (i = 0; i < buffer->planes; i++) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];
		struct vb2_plane *vp = &vb->planes[i];
		struct dma_buf *dbuf;

		p->dev = dev;
		p->direction = direction;
		p->size = vp->length;
		p->offset = vp->data_offset;
		if (!p->size || p->offset >= p->size ||
		    (direction == DMA_FROM_DEVICE && p->offset)) {
			ret = -EINVAL;
			goto fail;
		}
		if (vb->memory == VB2_MEMORY_DMABUF) {
			dbuf = vp->dbuf;
			if (!dbuf) {
				ret = -EINVAL;
				goto fail;
			}
			get_dma_buf(dbuf);
		} else {
			const struct vb2_mem_ops *ops = vb->vb2_queue->mem_ops;

			if (!ops || !ops->get_dmabuf || !vp->mem_priv) {
				ret = -EOPNOTSUPP;
				goto fail;
			}
			dbuf = ops->get_dmabuf(vb, vp->mem_priv, O_RDWR);
			if (IS_ERR_OR_NULL(dbuf)) {
				ret = dbuf ? PTR_ERR(dbuf) : -ENOMEM;
				goto fail;
			}
		}
		p->dbuf = dbuf;
		/* v4l2_plane.length is the whole plane allocation and already
		 * includes data_offset. Requiring length + data_offset to fit would
		 * reject every full-size plane with a nonzero data offset.
		 */
		if (p->size > dbuf->size) {
			ret = -EINVAL;
			goto fail;
		}

	}
	return buffer;
fail:
	vcp_venc_dma_release(buffer);
	return ERR_PTR(ret);
}

static int venc_dma_copy(struct vcp_venc_dma_plane *p, u32 bytes, bool output)
{
	struct iosys_map map = {};
	int ret, end;

	if (!p->staging || bytes > p->size)
		return -EINVAL;
	/* Cached staging: ownership passes through explicit syncs instead of
	 * an uncached mapping, which is what made the per-frame copy measure
	 * like ~213 MB/s of uncached stores.
	 */
	if (output)
		dma_sync_single_for_cpu(p->dev, p->staging_dma, p->staging_alloc,
				       DMA_FROM_DEVICE);
	ret = dma_buf_begin_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
	if (ret)
		return ret;
	ret = dma_buf_vmap_unlocked(p->dbuf, &map);
	if (!ret) {
		/* The staging buffer has the same plane base as the dma-buf. Keep
		 * data_offset in that coordinate system for the firmware.
		 */
		if (output)
			iosys_map_memcpy_to(&map, 0, p->staging, bytes);
		else
			iosys_map_memcpy_from(p->staging, &map, 0, bytes);
		dma_buf_vunmap_unlocked(p->dbuf, &map);
	}
	end = dma_buf_end_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
	ret = ret ?: end;
	if (!ret && !output)
		dma_sync_single_for_device(p->dev, p->staging_dma,
					  p->staging_alloc, DMA_TO_DEVICE);
	return ret;
}

/* Private image storage is cached CPU memory with a streaming DMA mapping.
 * Reuse it whenever the instance pool hands back a buffer of the same shape;
 * grow by replacing, never by realloc-in-place.
 */
static int venc_dma_alloc_staging(struct vcp_venc_dma_plane *p, u32 size)
{
	if (p->staging && p->staging_alloc >= size)
		return 0;
	if (p->staging) {
		dma_free_noncoherent(p->dev, p->staging_alloc, p->staging,
				     p->staging_dma, p->direction);
		p->staging = NULL;
		p->staging_alloc = 0;
	}
	p->staging = dma_alloc_noncoherent(p->dev, size, &p->staging_dma,
					   p->direction, GFP_KERNEL);
	if (!p->staging)
		return -ENOMEM;
	p->staging_alloc = size;
	return 0;
}

/* A direct pass-through is only correct when the client allocation already
 * is the firmware image: no repack, no stride-vs-width gap to zero, image
 * bytes at the firmware's own offsets, and the hardware's overread guard
 * inside the allocation. Everything else keeps the staging repack.
 */
static bool venc_layout_direct(const struct vcp_venc_input_layout *layout,
			       unsigned int plane)
{
	unsigned int j;

	for (j = 0; j < layout->components; j++) {
		const struct vcp_venc_component *c = &layout->component[j];

		if (c->plane != plane)
			continue;
		if (c->src_offset != c->dst_offset || c->stride < c->row_bytes)
			return false;
	}
	return true;
}

static void venc_dma_direct_unmap(struct vcp_venc_dma_plane *p)
{
	if (p->sgt) {
		dma_buf_unmap_attachment(p->attach, p->sgt, p->direction);
		p->sgt = NULL;
	}
	if (p->attach && p->dbuf) {
		dma_buf_detach(p->dbuf, p->attach);
		p->attach = NULL;
	}
	p->direct = false;
}

static int venc_dma_direct_map(struct vcp_venc_dma_plane *p, u32 need)
{
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t addr;

	attach = dma_buf_attach(p->dbuf, p->dev);
	if (IS_ERR(attach))
		return PTR_ERR(attach);
	sgt = dma_buf_map_attachment(attach, p->direction);
	if (IS_ERR(sgt)) {
		dma_buf_detach(p->dbuf, attach);
		return PTR_ERR(sgt);
	}
	/* One DMA-mapped entry means one contiguous IOVA span. */
	if (sgt->nents != 1 ||
	    sg_dma_len(sgt->sgl) < (size_t)p->offset + need) {
		dma_buf_unmap_attachment(attach, sgt, p->direction);
		dma_buf_detach(p->dbuf, attach);
		return -EOPNOTSUPP;
	}
	addr = sg_dma_address(sgt->sgl) + p->offset;
	if (addr >= BIT_ULL(34) || need > BIT_ULL(34) - addr) {
		dma_buf_unmap_attachment(attach, sgt, p->direction);
		dma_buf_detach(p->dbuf, attach);
		return -ERANGE;
	}
	p->attach = attach;
	p->sgt = sgt;
	p->direct = true;
	p->address = addr;
	return 0;
}

struct vcp_venc_dma_buffer *vcp_venc_dma_stage(struct device *dev,
	struct vb2_buffer *vb, enum dma_data_direction direction,
	struct vcp_venc_dma_pool *pool)
{
	struct vcp_venc_dma_buffer *buffer;
	unsigned int i;
	int ret;

	buffer = venc_dma_import(dev, vb, direction);
	if (IS_ERR(buffer))
		return buffer;
	venc_dma_reuse(buffer, pool);
	for (i = 0; i < buffer->planes; i++) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];

		ret = venc_dma_alloc_staging(p, p->size);
		if (ret) {
			goto fail;
		}
		if (p->staging_dma >= BIT_ULL(34) ||
		    p->size > BIT_ULL(34) - p->staging_dma) {
			ret = -ERANGE;
			goto fail;
		}
		p->address = p->staging_dma;
		if (direction == DMA_TO_DEVICE) {
			ret = venc_dma_copy(p, p->size, false);
			if (ret)
				goto fail;
		}
		/* Cached private storage: hand ownership to the device before the
		 * firmware writes into it. Without this the CPU's dirty lines can be
		 * written back over the coded bytes, which is what left the first
		 * coded buffer (the one carrying SPS/PPS) reading back as zeros.
		 */
		if (direction == DMA_FROM_DEVICE)
			dma_sync_single_for_device(p->dev, p->staging_dma,
						  p->staging_alloc,
						  DMA_FROM_DEVICE);
	}
	return buffer;
fail:
	vcp_venc_dma_release(buffer);
	return ERR_PTR(ret);
}

/* Copy only image samples. Prefixes, row padding, extra coded rows and guard
 * bytes are never read from the user's allocation or left uninitialized.
 */
struct vcp_venc_dma_buffer *vcp_venc_dma_stage_input(struct device *dev,
	struct vb2_buffer *vb, const struct vcp_venc_input_layout *layout,
	struct vcp_venc_dma_pool *pool)
{
	struct vcp_venc_dma_buffer *buffer;
	unsigned int i, j, row;
	int ret, end;

	if (!layout || !vb || vb->num_planes != layout->planes)
		return ERR_PTR(-EINVAL);
	for (i = 0; i < vb->num_planes; i++) {
		struct vb2_plane *p = &vb->planes[i];

		if (p->bytesused > p->length || p->data_offset >= p->bytesused ||
		    layout->src_size[i] > p->bytesused - p->data_offset)
			return ERR_PTR(-EINVAL);
	}
	buffer = venc_dma_import(dev, vb, DMA_TO_DEVICE);
	if (IS_ERR(buffer))
		return buffer;
	for (i = 0; i < buffer->planes; i++)
		buffer->plane[i].size = layout->src_size[i];
	/* Fast path: hand the firmware the client's own IOVA when the
	 * allocation already is the private image (layout, guard and aperture
	 * all match). Fall back to the staging repack below otherwise.
	 */
	ret = -EOPNOTSUPP;
	for (i = 0; i < buffer->planes; i++) {
		struct vb2_plane *vp = &vb->planes[i];

			if (!venc_layout_direct(layout, i) ||
			    layout->src_size[i] > vp->bytesused - vp->data_offset) {
			ret = -EOPNOTSUPP;
			break;
		}
			ret = venc_dma_direct_map(&buffer->plane[i], layout->src_size[i]);
		if (ret)
			break;
	}
	if (!ret)
		return buffer;
	for (i = 0; i < buffer->planes; i++)
		venc_dma_direct_unmap(&buffer->plane[i]);
	venc_dma_reuse(buffer, pool);
	for (i = 0; i < buffer->planes; i++) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];
		struct iosys_map map = {};
		u32 offset = p->offset, filled = 0;

		p->size = layout->dst_size[i];
		p->offset = 0;
		ret = venc_dma_alloc_staging(p, p->size);
		if (ret) {
			goto fail;
		}
		if (p->staging_dma >= BIT_ULL(34) ||
		    p->size > BIT_ULL(34) - p->staging_dma) {
			ret = -ERANGE;
			goto fail;
		}
		p->address = p->staging_dma;
		ret = dma_buf_begin_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
		if (ret)
			goto fail;
		ret = dma_buf_vmap_unlocked(p->dbuf, &map);
		if (!ret) {
			for (j = 0; j < layout->components; j++) {
				const struct vcp_venc_component *c = &layout->component[j];

				if (c->plane != i)
					continue;
				/* calc_layout orders components within each plane. Clear
				 * only gaps/guards; image bytes are overwritten below.
				 */
				memset(p->staging + filled, 0, c->dst_offset - filled);
				if (c->stride == c->row_bytes) {
					iosys_map_memcpy_from(p->staging + c->dst_offset,
							      &map, offset + c->src_offset,
							      c->rows * c->stride);
				} else {
					for (row = 0; row < c->rows; row++) {
						iosys_map_memcpy_from(p->staging + c->dst_offset +
								      row * c->stride, &map,
								      offset + c->src_offset +
								      row * c->stride,
								      c->row_bytes);
						memset(p->staging + c->dst_offset +
						       row * c->stride + c->row_bytes, 0,
						       c->stride - c->row_bytes);
					}
				}
				filled = c->dst_offset + c->rows * c->stride;
			}
			memset(p->staging + filled, 0, p->size - filled);
			dma_buf_vunmap_unlocked(p->dbuf, &map);
		}
		end = dma_buf_end_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
		ret = ret ?: end;
		if (ret)
			goto fail;
		dma_sync_single_for_device(dev, p->staging_dma, p->staging_alloc,
					  DMA_TO_DEVICE);
	}
	return buffer;
fail:
	vcp_venc_dma_release(buffer);
	return ERR_PTR(ret);
}

int vcp_venc_dma_copy_output(struct vcp_venc_dma_buffer *buffer, u32 bytes)
{
	if (buffer->direction != DMA_FROM_DEVICE || buffer->planes != 1)
		return -EINVAL;
	return venc_dma_copy(&buffer->plane[0], bytes, true);
}

MODULE_IMPORT_NS("DMA_BUF");
