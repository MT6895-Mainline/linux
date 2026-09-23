/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: Daniel Hsiao <daniel.hsiao@mediatek.com>
 *		Jungchang Tsao <jungchang.tsao@mediatek.com>
 *		Tiffany Lin <tiffany.lin@mediatek.com>
 */

#ifndef _VENC_DRV_IF_H_
#define _VENC_DRV_IF_H_

#include "mtk_vcodec_enc_drv.h"

/* Upstream V4L2 has no HEIF fourcc; matches vendor V4L2_PIX_FMT_HEIF and the
 * firmware caps entry 0x46494548. Single home for this macro: the protocol
 * layer compares the same value via v4l2_fourcc('H', 'E', 'I', 'F').
 */
#define V4L2_PIX_FMT_HEIF v4l2_fourcc('H', 'E', 'I', 'F')

/*
 * enum venc_yuv_fmt - The type of input yuv format
 * (VPU related: If you change the order, you must also update the VPU codes.)
 * @VENC_YUV_FORMAT_I420: I420 YUV format
 * @VENC_YUV_FORMAT_YV12: YV12 YUV format
 * @VENC_YUV_FORMAT_NV12: NV12 YUV format
 * @VENC_YUV_FORMAT_NV21: NV21 YUV format
 * @VENC_YUV_FORMAT_MT10: Mediatek 10-bit tile block mode
 * @VENC_YUV_FORMAT_P010: P010 10-bit YUV format
 * @VENC_YUV_FORMAT_RGB: packed 32-bit RGB, sentinel only
 *
 * The firmware advertises packed RGB as a raw encoder input (its capability
 * table lists BGR3/RBG3/AR24/BA24/BGR4/RBG4/BA30/RA30/AR30/AB30 with
 * type=2), and converts RGB to YUV inside the encoder. The wire value in
 * the venc_yuv_fmt space is not published by the vendor header this tree
 * was derived from; it was found by scanning on hardware.
 *
 * Two packed orders are distinct codes, so the sentinel carries which one
 * the client asked for and the VCP backend picks the matching firmware code.
 */
enum venc_yuv_fmt {
	VENC_YUV_FORMAT_I420 = 3,
	VENC_YUV_FORMAT_YV12 = 5,
	VENC_YUV_FORMAT_NV12 = 6,
	VENC_YUV_FORMAT_NV21 = 7,
	VENC_YUV_FORMAT_MT10 = 25,
	VENC_YUV_FORMAT_P010 = 26,
	/* Packed 32-bit RGB, memory order B,G,R,X (V4L2 "AR24"). */
	VENC_YUV_FORMAT_RGB_BGRX = 0x100,
	/* Packed 32-bit RGB, memory order A,R,G,B (V4L2 "BA24"). */
	VENC_YUV_FORMAT_RGB_ARGB = 0x101,
};

/*
 * enum venc_start_opt - encode frame option used in venc_if_encode()
 * @VENC_START_OPT_ENCODE_SEQUENCE_HEADER: encode SPS/PPS for H264
 * @VENC_START_OPT_ENCODE_FRAME: encode normal frame
 * @VENC_START_OPT_ENCODE_FRAME_FINAL: drain delayed firmware output
 */
enum venc_start_opt {
	VENC_START_OPT_ENCODE_SEQUENCE_HEADER,
	VENC_START_OPT_ENCODE_FRAME,
	VENC_START_OPT_ENCODE_FRAME_FINAL,
};

/*
 * enum venc_set_param_type - The type of set parameter used in
 *						      venc_if_set_param()
 * (VPU related: If you change the order, you must also update the VPU codes.)
 * @VENC_SET_PARAM_ENC: set encoder parameters
 * @VENC_SET_PARAM_FORCE_INTRA: force an intra frame
 * @VENC_SET_PARAM_ADJUST_BITRATE: adjust bitrate (in bps)
 * @VENC_SET_PARAM_ADJUST_FRAMERATE: set frame rate
 * @VENC_SET_PARAM_GOP_SIZE: set IDR interval
 * @VENC_SET_PARAM_INTRA_PERIOD: set I frame interval
 * @VENC_SET_PARAM_SKIP_FRAME: set H264 skip one frame
 * @VENC_SET_PARAM_PREPEND_HEADER: set H264 prepend SPS/PPS before IDR
 * @VENC_SET_PARAM_TS_MODE: set VP8 temporal scalability mode
 */
enum venc_set_param_type {
	VENC_SET_PARAM_ENC,
	VENC_SET_PARAM_FORCE_INTRA,
	VENC_SET_PARAM_ADJUST_BITRATE,
	VENC_SET_PARAM_ADJUST_FRAMERATE,
	VENC_SET_PARAM_GOP_SIZE,
	VENC_SET_PARAM_INTRA_PERIOD,
	VENC_SET_PARAM_SKIP_FRAME,
	VENC_SET_PARAM_PREPEND_HEADER,
	VENC_SET_PARAM_TS_MODE,
};

/*
 * struct venc_enc_prm - encoder settings for VENC_SET_PARAM_ENC used in
 *					  venc_if_set_param()
 * @input_fourcc: input yuv format
 * @h264_profile: V4L2 defined H.264 profile
 * @h264_level: V4L2 defined H.264 level
 * @width: image width
 * @height: image height
 * @buf_width: buffer width
 * @buf_height: buffer height
 * @frm_rate: frame rate in fps
 * @intra_period: intra frame period
 * @bitrate: target bitrate in bps
 * @gop_size: group of picture size
 */
struct venc_enc_param {
	enum venc_yuv_fmt input_yuv_fmt;
	unsigned int h264_profile;
	unsigned int h264_level;
	unsigned int width;
	unsigned int height;
	unsigned int buf_width;
	unsigned int buf_height;
	unsigned int frm_rate;
	unsigned int intra_period;
	unsigned int bitrate;
	unsigned int bitrate_mode;
	unsigned int num_b_frame;
	unsigned int gop_size;
	unsigned int heif_grid_size;
};

/**
 * struct venc_frame_info - per-frame information to pass to the firmware.
 *
 * @frm_count:		sequential number for this frame
 * @skip_frm_count:	number of frames skipped so far while decoding
 * @frm_type:		type of the frame, from enum venc_h264_frame_type
 */
struct venc_frame_info {
	unsigned int frm_count;		/* per frame update */
	unsigned int skip_frm_count;	/* per frame update */
	unsigned int frm_type;		/* per frame update */
};

/*
 * struct venc_frm_buf - frame buffer information used in venc_if_encode()
 * @fb_addr: plane frame buffer addresses
 */
struct venc_frm_buf {
	struct mtk_vcodec_fb fb_addr[MTK_VCODEC_MAX_PLANES];
};

/*
 * struct venc_done_result - This is return information used in venc_if_encode()
 * @bs_size: output bitstream size
 * @is_key_frm: output is key frame or not
 */
struct venc_done_result {
	unsigned int bs_size;
	bool is_key_frm;
	bool async;
};

extern const struct venc_common_if venc_h264_if;
extern const struct venc_common_if venc_vp8_if;
extern const struct venc_common_if venc_vcp_encoder_if;

/*
 * venc_if_init - Create the driver handle
 * @ctx: device context
 * @fourcc: encoder input format
 * Return: 0 if creating handle successfully, otherwise it is failed.
 */
int venc_if_init(struct mtk_vcodec_enc_ctx *ctx, unsigned int fourcc);

/*
 * venc_if_deinit - Release the driver handle
 * @ctx: device context
 * Return: 0 if releasing handle successfully, otherwise it is failed.
 */
int venc_if_deinit(struct mtk_vcodec_enc_ctx *ctx);

/*
 * venc_if_set_param - Set parameter to driver
 * @ctx: device context
 * @type: parameter type
 * @in: input parameter
 * Return: 0 if setting param successfully, otherwise it is failed.
 */
int venc_if_set_param(struct mtk_vcodec_enc_ctx *ctx,
		      enum venc_set_param_type type,
		      struct venc_enc_param *in);

/*
 * venc_if_encode - Encode one frame
 * @ctx: device context
 * @opt: encode frame option
 * @frm_buf: input frame buffer information
 * @bs_buf: output bitstream buffer information
 * @result: encode result
 * Return: 0 if encoding frame successfully, otherwise it is failed.
 */
int venc_if_encode(struct mtk_vcodec_enc_ctx *ctx,
		   enum venc_start_opt opt,
		   struct venc_frm_buf *frm_buf,
		   struct mtk_vcodec_mem *bs_buf,
		   struct venc_done_result *result);

void venc_vcp_encoder_buffers_ready(struct mtk_vcodec_enc_dev *dev);

#endif /* _VENC_DRV_IF_H_ */
