/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VENC_H
#define __MTK_VCP_VENC_H

#include <linux/remoteproc/mtk_vcp.h>

struct mtk_vcp_venc;
struct mtk_vcp_venc_inst;
struct vb2_buffer;
struct vcp_venc_video_format;
struct vcp_venc_frame_sizes;

struct vcp_venc_buffer_ids {
	u64 frame, bitstream;
};

/* Sizes are fixed by the xaga firmware ABI, independent of V4L2 limits. */
#define VCP_VENC_PLANES 8
#define VCP_VENC_BUFFERS 64
#define VCP_VENC_BITSTREAM_BASE 0x130000000ULL
#define VCP_VENC_BITSTREAM_END  0x150000000ULL

struct vcp_venc_config {
	__le32 input_fourcc, bitrate, pic_w, pic_h, buf_w, buf_h;
	__le32 gop_size, intra_period, framerate, profile, level, wfd;
	__le32 operationrate, scenario, prependheader, bitratemode;
	__le32 roi_rc_qp, roion, heif_grid_size;
	__le32 color_desc[17];
	__le32 resolution_change, max_w, max_h, num_b_frame, slbc_ready;
	__le32 i_qp, p_qp, b_qp, svp_mode, tsvc, max_qp, min_qp;
	__le32 i_p_qp_delta, qp_control_mode, frame_level_qp;
	__le32 highquality, dummynal, slbc_addr;
};

struct vcp_venc_result {
	u64 bitstream_cookie, frame_cookie;
	u32 bytes;
	bool keyframe;
};

struct vcp_venc_frame {
	u64 bitstream_cookie, frame_cookie, timestamp;
	/* input points at the payload (plane base + data_offset); input_size
	 * excludes the offset. data_offset retains the original plane offset.
	 */
	dma_addr_t input[3], bitstream;
	u32 input_size[3], data_offset[3], bitstream_size;
	u8 planes;
};

/* These run in the threaded mailbox handler. They must not call this API;
 * buffers_ready must wake/schedule a consumer rather than dequeue inline.
 * All hooks are mandatory; there is no fake power/IRQ/memory fallback.
 * type 0 uses VCP's software DMA domain; type 1 uses VENC's hardware domain.
 */
struct mtk_vcp_venc_ops {
	int (*power)(void *priv, u64 instance, unsigned int core, bool on);
	int (*wait_irq)(void *priv, u64 instance, unsigned int core, u32 *status);
	int (*alloc)(void *priv, u32 type, size_t size, struct mtk_vcp_mem *mem);
	void (*free)(void *priv, u32 type, struct mtk_vcp_mem *mem);
	void (*buffers_ready)(void *priv, u64 instance);
};

/* The caller owns a boot reference, keeps both devices alive, and must keep
 * DMA buffers alive until returned by firmware or both VCP and VENC quiesce.
 */
struct mtk_vcp_venc *mtk_vcp_venc_create(struct device *dev,
					 struct device *bitstream_dev,
					 struct mtk_vcp *vcp,
					 const struct mtk_vcp_venc_ops *ops,
					 void *priv);
int mtk_vcp_venc_destroy(struct mtk_vcp_venc *enc);
struct mtk_vcp_venc_inst *mtk_vcp_venc_new(struct mtk_vcp_venc *enc);
u64 mtk_vcp_venc_cookie(struct mtk_vcp_venc_inst *inst);
int mtk_vcp_venc_init(struct mtk_vcp_venc_inst *inst);
int mtk_vcp_venc_query(struct mtk_vcp_venc_inst *inst, u32 id,
		       void *output, size_t size);
int mtk_vcp_venc_query_caps(struct mtk_vcp_venc_inst *inst,
	struct vcp_venc_video_format *formats,
	struct vcp_venc_frame_sizes *sizes);
int mtk_vcp_venc_configure(struct mtk_vcp_venc_inst *inst,
			   const struct vcp_venc_config *config,
			   u32 sizeimage[VCP_VENC_PLANES], bool *synchronous);
int mtk_vcp_venc_set_param(struct mtk_vcp_venc_inst *inst, u32 id,
			 const u32 *data, size_t count);
/* mode: 2 sequence header, 3 frame, 4 final frame/flush. ENCODE_DONE is a
 * submission ACK. In asynchronous mode the returned buffers arrive through
 * PUT_BUFFER; dequeue() supplies the actual buffer/byte count.
 */
int mtk_vcp_venc_submit(struct mtk_vcp_venc_inst *inst, unsigned int mode,
			const struct vcp_venc_frame *frame);
/* V4L2 submission owns independent DMA-BUF references. Source/destination
 * remain pinned until dequeue returns their respective cookies, successful
 * DEINIT, or free(after_reset). On an uncertain RPC failure they stay pinned.
 * ids are nonzero only once ownership was transferred to the protocol.
 * Raw and vb2 submissions cannot be mixed before a successful DEINIT.
 * Caller supplies normal V4L2 buffer synchronization and serializes its queues.
 */
int mtk_vcp_venc_submit_vb2(struct mtk_vcp_venc_inst *inst, unsigned int mode,
	struct vb2_buffer *source, struct vb2_buffer *destination,
	struct vcp_venc_buffer_ids *ids);
int mtk_vcp_venc_dequeue(struct mtk_vcp_venc_inst *inst,
			 struct vcp_venc_result *result);
int mtk_vcp_venc_deinit(struct mtk_vcp_venc_inst *inst);
/* Failed deinit leaves the instance intact. after_reset may only be used
 * after hardware quiescence; VCP offline is checked before dropping records.
 */
int mtk_vcp_venc_free(struct mtk_vcp_venc_inst *inst, bool after_reset);

#endif
