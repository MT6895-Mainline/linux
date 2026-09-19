/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VDEC_ABI_H
#define __MTK_VCP_VDEC_ABI_H

#include <linux/build_bug.h>
#include <linux/types.h>
#include <linux/videodev2.h>

/* MT6895 RISC-V VCP vendor ABI, verified against xaga vendor headers. */
#define VCP_VDEC_AP_SEND_BASE       0xa000
#define VCP_VDEC_ACK_BASE           0xb000
#define VCP_VDEC_VCP_SEND_BASE      0xc000
#define VCP_VDEC_AP_ACK_BASE        0xd000

enum vcp_vdec_msg_id {
	VCP_VDEC_AP_INIT = VCP_VDEC_AP_SEND_BASE,
	VCP_VDEC_AP_START,
	VCP_VDEC_AP_END,
	VCP_VDEC_AP_DEINIT,
	VCP_VDEC_AP_RESET,
	VCP_VDEC_AP_SET_PARAM,
	VCP_VDEC_AP_QUERY_CAP,
	VCP_VDEC_AP_FRAME_BUFFER,
	VCP_VDEC_AP_BACKUP,
	VCP_VDEC_INIT_DONE = VCP_VDEC_ACK_BASE,
	VCP_VDEC_START_DONE,
	VCP_VDEC_DONE,
	VCP_VDEC_DEINIT_DONE,
	VCP_VDEC_RESET_DONE,
	VCP_VDEC_SET_PARAM_DONE,
	VCP_VDEC_QUERY_CAP_DONE,
	VCP_VDEC_PUT_FRAME_BUFFER = VCP_VDEC_VCP_SEND_BASE,
	VCP_VDEC_LOCK_CORE,
	VCP_VDEC_UNLOCK_CORE,
	VCP_VDEC_LOCK_LAT,
	VCP_VDEC_UNLOCK_LAT,
	VCP_VDEC_MEM_ALLOC,
	VCP_VDEC_MEM_FREE,
	VCP_VDEC_WAITISR,
	VCP_VDEC_GET_FRAME_BUFFER,
	VCP_VDEC_CHECK_CODEC_ID,
};

struct vcp_vdec_cmd {
	__le32 msg_id, ctx_id, vcp_inst_addr, reserved;
};

struct vcp_vdec_init {
	__le32 msg_id, ctx_id, reserved;
	__le64 ap_inst_addr;
};

struct vcp_vdec_start {
	__le32 msg_id, ctx_id, vcp_inst_addr;
	__le32 data[3], reserved;
};

struct vcp_vdec_set_param {
	__le32 msg_id, ctx_id, vcp_inst_addr, id;
	__le32 data[12];
};

struct vcp_vdec_query_cap {
	__le32 msg_id, ctx_id, id;
	__le64 ap_inst_addr, ap_data_addr;
};

/* The query id field of VCP_VDEC_AP_QUERY_CAP. The reply is an array of
 * VCP_VDEC_CAPS entries of the matching structure, written by the firmware
 * and copied out by the AP.
 */
enum vcp_vdec_cap_id {
	VCP_VDEC_CAP_SUPPORTED_FORMATS = 8,
	VCP_VDEC_CAP_FRAME_SIZES = 9,
};

struct vcp_vdec_cap_format {
	__le32 fourcc;
	__le32 type;
	__le32 num_planes;
};

struct vcp_vdec_cap_framesize {
	__le32 fourcc;
	__le32 profile;
	__le32 level;
	struct v4l2_frmsize_stepwise stepwise;
};

struct vcp_vdec_ack {
	__le32 msg_id, ctx_id, status;
	__le64 ap_inst_addr;
	__le32 codec_id, reserved;
};

struct vcp_vdec_init_ack {
	__le32 msg_id, ctx_id, status;
	__le64 ap_inst_addr;
	__le32 vcp_inst_addr;
};

struct vcp_vdec_mem_op {
	__le32 msg_id, ctx_id, status;
	__le64 ap_inst_addr;
	__le32 mem_type, mem_len;
	__le64 mem_iova, mem_pa, mem_va;
	__le32 reserved[2];
};

/* Vendor frame/plane limits are independent of the AP vb2 limits. */
#define VCP_VDEC_BUFFERS 64
#define VCP_VDEC_PLANES 8
#define VCP_VDEC_CAPS 128
#define VCP_VDEC_CHECK_ID_DONE 0xd008

/* Codec ids carried in the VCP_VDEC_CHECK_CODEC_ID handshake. They are the
 * vendor enum mtk_codec_type, shared with the encoder ABI, and have nothing
 * to do with the V4L2 fourcc. During INIT the firmware probes the codecs it
 * knows, one at a time, until the AP confirms the one the session uses.
 */
enum vcp_vdec_codec {
	VCP_VDEC_UNKNOWN = 0,
	VCP_VDEC_H264,
	VCP_VDEC_H265,
	VCP_VDEC_HEIF,
	VCP_VDEC_VP8,
	VCP_VDEC_VP9,
	VCP_VDEC_MPEG4,
	VCP_VDEC_H263,
	VCP_VDEC_MPEG12,
	VCP_VDEC_WMV,
	VCP_VDEC_RV30,
	VCP_VDEC_RV40,
	VCP_VDEC_AV1,
};

struct vcp_vdec_query_ack {
	__le32 id, ctx, status;
	__le64 cookie;
	__le32 query;
	__le64 data_cookie;
	__le32 address;
};

struct vcp_vdec_fb {
	__le64 cookie, y, c, general, timestamp;
	__le32 general_size, reserved;
};

struct vcp_vdec_bs_ring {
	__le64 cookie[VCP_VDEC_BUFFERS];
	__le32 read, write, count, reserved;
};

struct vcp_vdec_fb_ring {
	struct vcp_vdec_fb frame[VCP_VDEC_BUFFERS];
	__le32 read, write, count, reserved;
};

struct vcp_vdec_info {
	__le32 dpb_size, changed;
	__le64 bs_dma, bs_fd;
	__le64 fb_dma[VCP_VDEC_PLANES], fb_fd[VCP_VDEC_PLANES];
	__le64 bs_cookie, fb_cookie;
	__le32 planes, index, wait_keyframe, error_map;
	__le64 timestamp;
	__le32 queued_frames;
};

struct vcp_vdec_pic {
	__le32 width, height, buffer_width, buffer_height;
	__le32 plane_size[VCP_VDEC_PLANES];
	__le32 bitdepth, layout, fourcc;
};

struct vcp_vdec_format {
	__le32 fourcc, type, planes;
};

struct vcp_vdec_frame_size {
	__le32 fourcc, profile, level;
	__le32 min_width, max_width, step_width;
	__le32 min_height, max_height, step_height;
};

struct vcp_vdec_vsi {
	struct vcp_vdec_bs_ring free_bs;
	struct vcp_vdec_fb_ring free_fb, display;
	struct vcp_vdec_info dec;
	struct vcp_vdec_pic pic;
	__le32 color_desc[17];
	__le32 crop_left, crop_top, crop_width, crop_height;
	struct vcp_vdec_format formats[VCP_VDEC_CAPS];
	struct vcp_vdec_frame_size frame_sizes[VCP_VDEC_CAPS];
	__le32 aspect_ratio, fixed_buffers, fixed_buffers_svp, interlacing, codec_type;
	u8 crc_path[256], golden_path[256];
	u8 input_driven, ipi_blocked;
	__le32 general_fd;
	__le64 general_dma;
	__le32 general_size;
};

static_assert(sizeof(struct vcp_vdec_vsi) == 13752);
static_assert(sizeof(struct vcp_vdec_info) == 200);
static_assert(sizeof(struct vcp_vdec_pic) == 60);
static_assert(sizeof(struct vcp_vdec_bs_ring) == 528);
static_assert(sizeof(struct vcp_vdec_fb_ring) == 3088);
static_assert(sizeof(struct vcp_vdec_fb) == 48);
static_assert(sizeof(struct vcp_vdec_query_ack) == 48);
static_assert(offsetof(struct vcp_vdec_vsi, dec) == 6704);
static_assert(offsetof(struct vcp_vdec_vsi, pic) == 6904);
static_assert(offsetof(struct vcp_vdec_vsi, input_driven) == 13724);
static_assert(offsetof(struct vcp_vdec_vsi, general_dma) == 13736);

static_assert(sizeof(struct vcp_vdec_cmd) == 16);
static_assert(sizeof(struct vcp_vdec_init) == 24);
static_assert(sizeof(struct vcp_vdec_start) == 28);
static_assert(sizeof(struct vcp_vdec_set_param) == 64);
static_assert(sizeof(struct vcp_vdec_query_cap) == 32);
static_assert(sizeof(struct vcp_vdec_ack) == 32);
static_assert(sizeof(struct vcp_vdec_init_ack) == 32);
static_assert(sizeof(struct vcp_vdec_mem_op) == 64);

#endif
