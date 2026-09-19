// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/iosys-map.h>
#include <linux/slab.h>
#include <media/videobuf2-core.h>

#include "mtk_vcp_venc_dma.h"

void vcp_venc_dma_release(struct vcp_venc_dma_buffer *buffer)
{
	int i;

	if (!buffer)
		return;
	for (i = buffer->planes - 1; i >= 0; i--) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];

		if (p->staging)
			dma_free_coherent(buffer->dev, p->size, p->staging,
					  p->staging_dma);
		if (p->dbuf)
			dma_buf_put(p->dbuf);
	}
	put_device(buffer->dev);
	kfree(buffer);
	module_put(THIS_MODULE);
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
	return ret ?: end;
}

struct vcp_venc_dma_buffer *vcp_venc_dma_stage(struct device *dev,
	struct vb2_buffer *vb, enum dma_data_direction direction)
{
	struct vcp_venc_dma_buffer *buffer;
	unsigned int i;
	int ret;

	buffer = venc_dma_import(dev, vb, direction);
	if (IS_ERR(buffer))
		return buffer;
	for (i = 0; i < buffer->planes; i++) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];

		p->staging = dma_alloc_coherent(dev, p->size, &p->staging_dma,
						GFP_KERNEL);
		if (!p->staging) {
			ret = -ENOMEM;
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
	struct vb2_buffer *vb, const struct vcp_venc_input_layout *layout)
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
	for (i = 0; i < buffer->planes; i++) {
		struct vcp_venc_dma_plane *p = &buffer->plane[i];
		struct iosys_map map = {};
		u32 offset = p->offset;

		p->size = layout->dst_size[i];
		p->offset = 0;
		p->staging = dma_alloc_coherent(dev, p->size, &p->staging_dma,
						GFP_KERNEL);
		if (!p->staging) {
			ret = -ENOMEM;
			goto fail;
		}
		if (p->staging_dma >= BIT_ULL(34) ||
		    p->size > BIT_ULL(34) - p->staging_dma) {
			ret = -ERANGE;
			goto fail;
		}
		p->address = p->staging_dma;
		memset(p->staging, 0, p->size);
		ret = dma_buf_begin_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
		if (ret)
			goto fail;
		ret = dma_buf_vmap_unlocked(p->dbuf, &map);
		if (!ret) {
			for (j = 0; j < layout->components; j++) {
				const struct vcp_venc_component *c = &layout->component[j];

				if (c->plane != i)
					continue;
				for (row = 0; row < c->rows; row++)
					iosys_map_memcpy_from(p->staging + c->dst_offset +
							      row * c->stride, &map,
							      offset + c->src_offset +
							      row * c->stride, c->row_bytes);
			}
			dma_buf_vunmap_unlocked(p->dbuf, &map);
		}
		end = dma_buf_end_cpu_access(p->dbuf, DMA_BIDIRECTIONAL);
		ret = ret ?: end;
		if (ret)
			goto fail;
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
