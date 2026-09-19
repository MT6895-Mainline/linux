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

struct vcp_venc_dma_plane {
	struct dma_buf *dbuf;
	dma_addr_t address;
	u32 size, offset;
	void *staging;
	dma_addr_t staging_dma;
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
	struct vcp_venc_dma_plane plane[3];
};

/* Firmware receives only private coherent storage. An unconfirmed stop
 * retains that storage, never DMA access to reusable userspace buffers.
 */
struct vcp_venc_dma_buffer *vcp_venc_dma_stage(struct device *dev,
	struct vb2_buffer *vb, enum dma_data_direction direction);
struct vcp_venc_dma_buffer *vcp_venc_dma_stage_input(struct device *dev,
	struct vb2_buffer *vb, const struct vcp_venc_input_layout *layout);
int vcp_venc_dma_copy_output(struct vcp_venc_dma_buffer *buffer, u32 bytes);
/* Only before submission, after firmware return/DEINIT, or after confirmed
 * VCP and VENC quiescence. The caller serializes record access.
 */
void vcp_venc_dma_release(struct vcp_venc_dma_buffer *buffer);

#endif
