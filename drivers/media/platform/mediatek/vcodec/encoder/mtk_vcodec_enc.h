/* SPDX-License-Identifier: GPL-2.0 */
/*
* Copyright (c) 2016 MediaTek Inc.
* Author: PC Chen <pc.chen@mediatek.com>
*         Tiffany Lin <tiffany.lin@mediatek.com>
*/

#ifndef _MTK_VCODEC_ENC_H_
#define _MTK_VCODEC_ENC_H_

#include <media/videobuf2-core.h>
#include <media/v4l2-mem2mem.h>

#include "mtk_vcodec_enc_drv.h"

/* Verified H.264/HEVC encode boxes reach 4K on the MT6895 VCP firmware
 * (see ENCODER-EXPANSION.md), so the VCP backend advertises it. This also
 * raises the H.264 level ceiling to 5.1 for spec-conformant 4K headers;
 * no dims-vs-level gate is applied because the firmware decodes
 * level/dims mismatches exactly (see venc_vcp_if.c).
 */
#define MTK_VENC_4K_CAPABILITY_ENABLE BIT(0)

#define MTK_VENC_IRQ_STATUS_SPS	0x1
#define MTK_VENC_IRQ_STATUS_PPS	0x2
#define MTK_VENC_IRQ_STATUS_FRM	0x4
#define MTK_VENC_IRQ_STATUS_DRAM	0x8
#define MTK_VENC_IRQ_STATUS_PAUSE	0x10
#define MTK_VENC_IRQ_STATUS_SWITCH	0x20

#define MTK_VENC_IRQ_STATUS_OFFSET	0x05C
#define MTK_VENC_IRQ_ACK_OFFSET	0x060

/**
 * struct mtk_video_enc_buf - Private data related to each VB2 buffer.
 * @m2m_buf:	M2M buffer
 * @param_change: Types of encode parameter change before encoding this
 *				buffer
 * @enc_params: Encode parameters changed before encode this buffer
 */
struct mtk_video_enc_buf {
	struct v4l2_m2m_buffer m2m_buf;

	u32 param_change;
	struct mtk_enc_params enc_params;
};

extern const struct v4l2_ioctl_ops mtk_venc_ioctl_ops;
extern const struct v4l2_m2m_ops mtk_venc_m2m_ops;

int mtk_venc_unlock(struct mtk_vcodec_enc_ctx *ctx);
int mtk_venc_lock(struct mtk_vcodec_enc_ctx *ctx);
int mtk_vcodec_enc_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq);
void mtk_vcodec_enc_release(struct mtk_vcodec_enc_ctx *ctx);
int mtk_vcodec_enc_ctrls_setup(struct mtk_vcodec_enc_ctx *ctx);
void mtk_vcodec_enc_set_default_params(struct mtk_vcodec_enc_ctx *ctx);

#endif /* _MTK_VCODEC_ENC_H_ */
