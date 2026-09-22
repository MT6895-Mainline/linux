/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VDEC_H
#define __MTK_VCP_VDEC_H

#include <linux/remoteproc/mtk_vcp.h>
#include "mtk_vcp_vdec_abi.h"

struct mtk_vcp_vdec;

/* Hooks execute in the threaded mailbox handler. notify must only schedule
 * work: calling this API from a hook would deadlock the RPC transaction.
 */
struct mtk_vcp_vdec_ops {
	int (*power)(void *priv, unsigned int core, bool on);
	int (*wait_irq)(void *priv, unsigned int core);
	int (*alloc)(void *priv, u32 type, size_t size, struct mtk_vcp_mem *mem);
	void (*free)(void *priv, u32 type, struct mtk_vcp_mem *mem);
	void (*notify)(void *priv);
};

enum vcp_vdec_event_type {
	VCP_VDEC_DISPLAY,
	VCP_VDEC_FREE_FRAME,
	VCP_VDEC_FREE_BITSTREAM,
};

struct vcp_vdec_event {
	enum vcp_vdec_event_type type;
	u64 cookie, timestamp;
};

struct vcp_vdec_picture {
	u32 width, height, stride, buffer_height, size[2], dpb, fourcc;
	u32 crop_left, crop_top, crop_width, crop_height;
	u8 input_driven;
};

/* Caller holds a VCP boot reference and owns all submitted DMA allocations.
 * Frame storage stays pinned until FREE_FRAME, independently of DISPLAY.
 * Bitstream storage stays pinned until FREE_BITSTREAM. Failed RPCs retain
 * ownership until successful DEINIT or confirmed VCP and hardware shutdown.
 * Only one session can register the decoder handler at a time.
 */
struct mtk_vcp_vdec *mtk_vcp_vdec_create(struct device *dev, struct mtk_vcp *vcp,
				      const struct mtk_vcp_vdec_ops *ops, void *priv);
int mtk_vcp_vdec_init(struct mtk_vcp_vdec *dec);
int mtk_vcp_vdec_set_codec(struct mtk_vcp_vdec *dec, u32 codec_id);
int mtk_vcp_vdec_query_cap(struct mtk_vcp_vdec *dec, u32 id, void *out, size_t size);
int mtk_vcp_vdec_picture(struct mtk_vcp_vdec *dec, struct vcp_vdec_picture *pic);
int mtk_vcp_vdec_frame(struct mtk_vcp_vdec *dec, u64 cookie, unsigned int index,
		       dma_addr_t y, dma_addr_t c);
int mtk_vcp_vdec_start(struct mtk_vcp_vdec *dec, u64 cookie, dma_addr_t dma,
		       u32 bytes, u32 capacity, u64 timestamp, u32 *changed);
int mtk_vcp_vdec_reset(struct mtk_vcp_vdec *dec, bool drain);
int mtk_vcp_vdec_event(struct mtk_vcp_vdec *dec, struct vcp_vdec_event *event);
int mtk_vcp_vdec_deinit(struct mtk_vcp_vdec *dec);
/* after_reset additionally requires the caller to have quiesced VDEC DMA. */
int mtk_vcp_vdec_destroy(struct mtk_vcp_vdec *dec, bool after_reset);
#endif
