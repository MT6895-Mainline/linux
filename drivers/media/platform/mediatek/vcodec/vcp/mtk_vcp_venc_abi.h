/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VENC_ABI_H
#define __MTK_VCP_VENC_ABI_H

#include <linux/build_bug.h>
#include <linux/videodev2.h>
#include "mtk_vcp_venc.h"

#define VCP_CODEC_H264_ENCODER 13
#define VCP_VENC_MAX_CAPS 64

/* Vendor venc_ipi_msg.h: do not depend on frontend enum ordering. */
enum vcp_venc_param_id {
	VCP_VENC_PARAM_FORCE_INTRA = 1,
	VCP_VENC_PARAM_BITRATE = 2,
	VCP_VENC_PARAM_FRAMERATE = 3,
	VCP_VENC_PARAM_GOP_SIZE = 4,
	VCP_VENC_PARAM_INTRA_PERIOD = 5,
	VCP_VENC_PARAM_PREPEND_HEADER = 7,
};
static_assert(VCP_VENC_PARAM_FORCE_INTRA == 1);
static_assert(VCP_VENC_PARAM_BITRATE == 2);
static_assert(VCP_VENC_PARAM_FRAMERATE == 3);
static_assert(VCP_VENC_PARAM_GOP_SIZE == 4);
static_assert(VCP_VENC_PARAM_INTRA_PERIOD == 5);
static_assert(VCP_VENC_PARAM_PREPEND_HEADER == 7);

enum vcp_venc_query_id {
	VCP_VENC_QUERY_SUPPORTED_FORMATS = 0,
	VCP_VENC_QUERY_FRAME_SIZES = 1,
};

/* These are copied from the vendor wire ABI. Do not use the mainline
 * VIDEO_MAX_FRAME value here: the firmware table has 64 entries.
 */
struct vcp_venc_video_format {
	__le32 fourcc;
	__le32 type;
	__le32 num_planes;
};

struct vcp_venc_frame_sizes {
	__le32 fourcc;
	__le32 profile;
	__le32 level;
	struct v4l2_frmsize_stepwise stepwise;
};

enum vcp_venc_command {
	VCP_ENC_INIT = 0x1000,
	VCP_ENC_SET_PARAM,
	VCP_ENC_ENCODE,
	VCP_ENC_DEINIT,
	VCP_ENC_QUERY,
	VCP_ENC_INIT_DONE = 0x2000,
	VCP_ENC_SET_PARAM_DONE,
	VCP_ENC_ENCODE_DONE,
	VCP_ENC_DEINIT_DONE,
	VCP_ENC_QUERY_DONE,
	VCP_ENC_TRACE,
	VCP_ENC_POWER_ON = 0x3000,
	VCP_ENC_POWER_OFF,
	VCP_ENC_PUT_BUFFER,
	VCP_ENC_ALLOC,
	VCP_ENC_FREE,
	VCP_ENC_WAIT_ISR,
	VCP_ENC_CHECK_ID,
};

struct vcp_venc_ring {
	__le64 bitstream[VCP_VENC_BUFFERS], frame[VCP_VENC_BUFFERS];
	__le32 bytes[VCP_VENC_BUFFERS], keyframe[VCP_VENC_BUFFERS];
	__le32 read, write, count, reserved;
};

struct vcp_venc_info {
	__le64 bs_dma, bs_fd, fb_dma[VCP_VENC_PLANES], fb_fd[VCP_VENC_PLANES];
	/* Vendor ABI names these AP-side virtual addresses.  They are opaque
	 * completion identities to this driver; the internal cookie tables keep
	 * the DMA ownership records separate from the wire format.
	 */
	__le64 venc_bs_va, venc_fb_va;
	__le32 fb_num_planes, index;
	__le64 timestamp;
	__le32 qpmap;
};

struct vcp_venc_vsi {
	struct vcp_venc_config config;
	__le32 sizeimage[VCP_VENC_PLANES];
	struct vcp_venc_ring free;
	struct vcp_venc_info info;
	__le32 sync_mode;
	__le64 meta_addr;
	__le32 meta_size, meta_offset;
	__le64 qpmap_addr;
	__le32 qpmap_size;
};

struct vcp_venc_init_msg { __le32 id, reserved; __le64 instance; };
struct vcp_venc_cmd_msg { __le32 id, firmware_instance; };
struct vcp_venc_param_msg {
	__le32 id, firmware_instance, parameter, count, data[8];
};
struct vcp_venc_query_msg { __le32 id, query; __le64 instance, cookie; };
struct vcp_venc_encode_msg {
	__le32 id, firmware_instance, input[3], size[3], output, output_size;
	__le32 offset[3];
	u8 planes, mode;
	__le16 padding;
	__le32 meta_size;
};
struct vcp_venc_ack { __le32 id, status; __le64 instance; };
struct vcp_venc_init_ack { struct vcp_venc_ack hdr; __le32 vsi, reserved; };
struct vcp_venc_encode_ack {
	struct vcp_venc_ack hdr;
	__le32 state, keyframe, bytes, reserved;
};
struct vcp_venc_query_ack {
	struct vcp_venc_ack hdr;
	__le32 query, padding;
	__le64 cookie;
	__le32 address, reserved;
};
struct vcp_venc_service {
	struct vcp_venc_ack hdr;
	__le32 codec_or_irq, timeout;
};
struct vcp_venc_mem_msg {
	struct vcp_venc_ack hdr;
	__le32 type, len;
	__le64 iova, pa, cookie;
	__le32 property, log;
};

static_assert(sizeof(struct vcp_venc_config) == 216);
static_assert(sizeof(struct vcp_venc_ring) == 1552);
static_assert(offsetof(struct vcp_venc_info, venc_bs_va) == 144);
static_assert(offsetof(struct vcp_venc_info, venc_fb_va) == 152);
static_assert(offsetof(struct vcp_venc_info, fb_num_planes) == 160);
static_assert(offsetof(struct vcp_venc_vsi, free) == 248);
static_assert(offsetof(struct vcp_venc_vsi, info) == 1800);
static_assert(offsetof(struct vcp_venc_vsi, sync_mode) == 1984);
static_assert(sizeof(struct vcp_venc_vsi) == 2024);
static_assert(sizeof(struct vcp_venc_init_msg) == 16);
static_assert(sizeof(struct vcp_venc_encode_msg) == 60);
static_assert(sizeof(struct vcp_venc_encode_ack) == 32);
static_assert(sizeof(struct vcp_venc_param_msg) == 48);
static_assert(sizeof(struct vcp_venc_query_ack) == 40);
static_assert(sizeof(struct vcp_venc_service) == 24);
static_assert(sizeof(struct vcp_venc_mem_msg) == 56);
static_assert(sizeof(struct vcp_venc_video_format) == 12);
static_assert(sizeof(struct vcp_venc_frame_sizes) == 36);

#endif
