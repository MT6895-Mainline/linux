/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VENC_DMA_H
#define __MTK_VCP_VENC_DMA_H

#include <linux/dma-direction.h>
#include <linux/list.h>
#include <linux/types.h>

#include "mtk_vcp_venc_layout.h"

struct device;
struct vb2_buffer;
struct dma_buf;

/* Per-instance, caller-serialized storage of returned private DMA buffers. */
struct vcp_venc_dma_pool {
	struct list_head buffers;
	size_t bytes;
};

struct vcp_venc_dma_plane {
	struct dma_buf *dbuf;
	struct device *dev;
	enum dma_data_direction direction;
	dma_addr_t address;
	u32 size, offset;
	void *staging;
	dma_addr_t staging_dma;
	size_t staging_alloc;
	/* Strict passthrough: the client allocation already matches the
	 * firmware layout and maps as one contiguous IOVA span. */
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	bool direct;
};

/* No vb2/context pointer survives submission. The record holds dma-buf
 * references for CPU copies; only private staging is mapped for device DMA.
 */
struct vcp_venc_dma_buffer {
	struct list_head list;
	struct device *dev;
	u64 cookie;
	enum dma_data_direction direction;
	unsigned int planes;
	/* Bytes this record was charged to the instance DMA budget. The budget
	 * is charged with the firmware's padded layout (dst_size) while
	 * plane[].size carries the visible image (src_size) on the direct
	 * path, so the charge cannot be recomputed from the planes at release
	 * time; storing it is what keeps the accounting symmetric.
	 */
	size_t charge;
	struct vcp_venc_dma_plane plane[3];
};

/* Firmware receives private cached storage (explicitly synced) or, when the
 * source already matches the firmware layout exactly, the mapped client
 * buffer itself. An unconfirmed stop retains that storage and its DMA
 * mapping, never a bare address into a reusable userspace buffer.
 */
struct vcp_venc_dma_buffer *vcp_venc_dma_stage(struct device *dev,
	struct vb2_buffer *vb, enum dma_data_direction direction,
	struct vcp_venc_dma_pool *pool);
struct vcp_venc_dma_buffer *vcp_venc_dma_stage_input(struct device *dev,
	struct vb2_buffer *vb, const struct vcp_venc_input_layout *layout,
	struct vcp_venc_dma_pool *pool);
void vcp_venc_dma_pool_clear(struct vcp_venc_dma_pool *pool);
void vcp_venc_dma_recycle(struct vcp_venc_dma_pool *pool,
	struct vcp_venc_dma_buffer *buffer);
int vcp_venc_dma_copy_output(struct vcp_venc_dma_buffer *buffer, u32 bytes);
/* Only before submission, after firmware return/DEINIT, or after confirmed
 * VCP and VENC quiescence. The caller serializes record access.
 */
void vcp_venc_dma_release(struct vcp_venc_dma_buffer *buffer);

#endif
