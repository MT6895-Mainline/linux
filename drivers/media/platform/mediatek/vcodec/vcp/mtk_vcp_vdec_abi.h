/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VDEC_ABI_H
#define __MTK_VCP_VDEC_ABI_H

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

static_assert(sizeof(struct vcp_vdec_cmd) == 16);
static_assert(sizeof(struct vcp_vdec_init) == 24);
static_assert(sizeof(struct vcp_vdec_start) == 28);
static_assert(sizeof(struct vcp_vdec_set_param) == 64);
static_assert(sizeof(struct vcp_vdec_query_cap) == 32);
static_assert(sizeof(struct vcp_vdec_ack) == 32);
static_assert(sizeof(struct vcp_vdec_init_ack) == 32);
static_assert(sizeof(struct vcp_vdec_mem_op) == 64);

#endif
