// SPDX-License-Identifier: GPL-2.0-only
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/videobuf2-v4l2.h>

#include "mtk_vcodec_enc.h"
#include "../vcp/mtk_vcp_venc_abi.h"
#include "../vcp/mtk_vcp_venc_layout.h"
#include "venc_drv_base.h"

static bool vcp_force_async;
module_param_named(force_async, vcp_force_async, bool, 0644);
MODULE_PARM_DESC(force_async,
	"allow pipelined VCP frames without B-frame reorder (experimental)");

struct vcp_encoder_pending {
	u64 frame_cookie, bitstream_cookie;
	struct vb2_v4l2_buffer *src, *dst;
	u64 timestamp;
	struct v4l2_timecode timecode;
};

struct vcp_encoder_handle {
	struct mtk_vcodec_enc_ctx *ctx;
	struct mtk_vcodec_enc_dev *dev;
	struct mtk_vcp_venc_inst *inst;
	struct vcp_encoder_pending pending[VCP_VENC_BUFFERS];
	struct vcp_venc_input_layout input_layout;
	bool claimed, booted, initialized, configured, synchronous, serialized, failed;
};

static void vcp_encoder_abort_pending(struct vcp_encoder_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++) {
		struct vcp_encoder_pending *p = &h->pending[i];

		if (p->src)
			v4l2_m2m_buf_done(p->src, VB2_BUF_STATE_ERROR);
		if (p->dst) {
			vb2_set_plane_payload(&p->dst->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(p->dst, VB2_BUF_STATE_ERROR);
		}
		memset(p, 0, sizeof(*p));
	}
}

static struct vcp_encoder_pending *vcp_encoder_pending_slot(struct vcp_encoder_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++)
		if (!h->pending[i].frame_cookie && !h->pending[i].bitstream_cookie)
			return &h->pending[i];
	return NULL;
}

static bool vcp_encoder_pending_empty(struct vcp_encoder_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++)
		if (h->pending[i].frame_cookie || h->pending[i].bitstream_cookie)
			return false;
	return true;
}

static int vcp_encoder_complete(struct vcp_encoder_handle *h,
			     const struct vcp_venc_result *done)
{
	struct vcp_encoder_pending *frame = NULL, *bitstream = NULL, *p;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++) {
		p = &h->pending[i];
		if (done->frame_cookie && p->frame_cookie == done->frame_cookie)
			frame = p;
		if (done->bitstream_cookie &&
		    p->bitstream_cookie == done->bitstream_cookie)
			bitstream = p;
	}
	if ((done->frame_cookie && !frame) ||
	    (done->bitstream_cookie && !bitstream) ||
	    (frame && bitstream && frame != bitstream))
		return -EPROTO;
	p = frame ?: bitstream;
	if (!p)
		return -EPROTO;
	if (bitstream) {
		if (!p->dst)
			return -EPROTO;
		p->dst->vb2_buf.timestamp = p->timestamp;
		p->dst->timecode = p->timecode;
		if (done->keyframe)
			p->dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
		vb2_set_plane_payload(&p->dst->vb2_buf, 0, done->bytes);
		v4l2_m2m_buf_done(p->dst, VB2_BUF_STATE_DONE);
		p->dst = NULL;
		p->bitstream_cookie = 0;
	}
	if (frame) {
		if (!p->src)
			return -EPROTO;
		v4l2_m2m_buf_done(p->src, VB2_BUF_STATE_DONE);
		p->src = NULL;
		p->frame_cookie = 0;
	}
	return 0;
}

/* enc_mutex serializes every caller. Failed cleanup remains device-owned;
 * neither a file close nor an INIT error may discard its ownership record.
 */
static int vcp_encoder_stop(struct vcp_encoder_handle *h, bool graceful)
{
	struct mtk_vcodec_enc_dev *dev = h->dev;
	int ret;

	h->configured = false;
	vcp_encoder_abort_pending(h);
	if (graceful && h->inst && h->initialized) {
		ret = mtk_vcp_venc_deinit(h->inst);
		if (!ret && mtk_vcp_venc_hw_idle(dev->vcp_hw)) {
			ret = mtk_vcp_venc_free(h->inst, false);
			if (!ret)
				h->inst = NULL;
		}
		if (ret)
			dev_err(&dev->plat_dev->dev, "VCP DEINIT cleanup failed: %d\n", ret);
	}
	if (h->booted) {
		ret = mtk_vcp_shutdown(dev->vcp);
		if (ret)
			return ret;
		h->booted = false;
	}
	/* The reservation remains held until firmware and hardware DMA stop. */
	if (h->inst) {
		if (!mtk_vcp_is_offline(dev->vcp))
			return -EBUSY;
		ret = mtk_vcp_venc_hw_quiesce(dev->vcp_hw);
		if (ret)
			return ret;
		ret = mtk_vcp_venc_free(h->inst, true);
		if (ret)
			return ret;
		h->inst = NULL;
	}
	h->initialized = false;
	h->synchronous = false;
	h->serialized = false;
	if (h->claimed) {
		mtk_vcp_release(dev->vcp, h);
		h->claimed = false;
	}
	return 0;
}

static void vcp_encoder_dispose(struct vcp_encoder_handle *h, int cleanup)
{
	if (cleanup) {
		h->dev->vcp_faulted = true;
		h->ctx = NULL;
		dev_err(&h->dev->plat_dev->dev,
			"VCP cleanup unconfirmed (%d); retaining session/DMA until reboot\n",
			cleanup);
		return;
	}
	h->dev->vcp_session = NULL;
	h->dev->vcp_faulted = false;
	kfree(h);
	module_put(THIS_MODULE);
}

static int vcp_encoder_init(struct mtk_vcodec_enc_ctx *ctx)
{
	struct mtk_vcodec_enc_dev *dev = ctx->dev;
	struct vcp_encoder_handle *h;
	int ret, cleanup;

	if (!dev->vcp_venc)
		return -EIO;
	if (dev->vcp_faulted) {
		h = dev->vcp_session;
		/* A live file must finish returning its buffers before recovery. */
		if (!h || h->ctx)
			return -EIO;
		ret = vcp_encoder_stop(h, false);
		if (ret)
			return ret;
		vcp_encoder_dispose(h, 0);
	}
	if (dev->vcp_session)
		return -EBUSY;
	h = kzalloc_obj(*h);
	if (!h)
		return -ENOMEM;
	ret = mtk_vcp_claim(dev->vcp, h);
	if (ret) {
		kfree(h);
		return ret;
	}
	h->claimed = true;
	__module_get(THIS_MODULE);
	h->ctx = ctx;
	h->dev = dev;
	dev->vcp_session = h;
	ret = mtk_vcp_boot(dev->vcp);
	if (ret) {
		dev_err(&dev->plat_dev->dev, "VCP boot failed: %d\n", ret);
		goto err;
	}
	h->booted = true;
	h->inst = mtk_vcp_venc_new(dev->vcp_venc);
	if (IS_ERR(h->inst)) {
		ret = PTR_ERR(h->inst);
		h->inst = NULL;
		goto err;
	}
	ret = mtk_vcp_venc_set_codec(h->inst, ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc);
	if (ret)
		goto err;
	ret = mtk_vcp_venc_init(h->inst);
	if (ret) {
		dev_err(&dev->plat_dev->dev, "VCP encoder INIT failed: %d\n", ret);
		goto err;
	}
	h->initialized = true;
	ctx->drv_handle = h;
	return 0;
err:
	/* An INIT timeout may already have caused firmware DMA allocations. */
	cleanup = vcp_encoder_stop(h, false);
	vcp_encoder_dispose(h, cleanup);
	return ret;
}

/* The shipped xaga image consumes the common firmware level enum, not the
 * interleaved HEVC-only enum in the released vendor driver. Keep the wire
 * values explicit: H.264-only entries leave holes between HEVC levels.
 */
static int vcp_hevc_level(unsigned int level, unsigned int tier)
{
	static const u8 main_tier[] = { 2, 8, 10, 13, 15, 18, 20, 23, 25, 27, 29, 31, 33 };

	if (level >= ARRAY_SIZE(main_tier) || tier > V4L2_MPEG_VIDEO_HEVC_TIER_HIGH ||
	    (tier == V4L2_MPEG_VIDEO_HEVC_TIER_HIGH && level < V4L2_MPEG_VIDEO_HEVC_LEVEL_4))
		return -EINVAL;
	return main_tier[level] + tier;
}

/* Wire indices into the 17-word vendor color description. Only the three
 * VUI bytes and full_range are meaningful for SDR; the mastering, light
 * level and is_hdr words require 10-bit samples.
 */
enum vcp_color_desc_index {
	VCP_COLOR_PRIMARIES = 0,
	VCP_COLOR_TRANSFER = 1,
	VCP_COLOR_MATRIX = 2,
	VCP_COLOR_MAX_LUMINANCE = 11,
	VCP_COLOR_MIN_LUMINANCE = 12,
	VCP_COLOR_MAX_CLL = 13,
	VCP_COLOR_MAX_FALL = 14,
	VCP_COLOR_IS_HDR = 15,
	VCP_COLOR_FULL_RANGE = 16,
};

static int vcp_encoder_fill_color(const struct mtk_vcodec_enc_ctx *ctx,
				  bool ten_bit, __le32 out[17])
{
	const u32 *desc = ctx->enc_params.color_desc;
	unsigned int i;

	if (!ctx->enc_params.color_desc_set)
		return 0;
	/* The frontend validates the same bounds; re-check here so a future
	 * writer cannot push out-of-range VUI bytes onto the wire.
	 */
	if (desc[VCP_COLOR_PRIMARIES] > 255 ||
	    desc[VCP_COLOR_TRANSFER] > 255 ||
	    desc[VCP_COLOR_MATRIX] > 255 ||
	    desc[VCP_COLOR_IS_HDR] > 1 || desc[VCP_COLOR_FULL_RANGE] > 1)
		return -EINVAL;
	if (!ten_bit && (desc[VCP_COLOR_IS_HDR] ||
			 desc[VCP_COLOR_MAX_LUMINANCE] ||
			 desc[VCP_COLOR_MIN_LUMINANCE] ||
			 desc[VCP_COLOR_MAX_CLL] || desc[VCP_COLOR_MAX_FALL]))
		return -EINVAL;
	for (i = 0; i < 17; i++)
		out[i] = cpu_to_le32(desc[i]);
	return 0;
}

/* MPEG4-Part2 and H.263 have confirmed codec IDs, but this firmware build
 * NACKs their INIT, so the exposure gate keeps those fourccs out and no
 * per-codec branch exists below. See ENCODER-EXPANSION.md for the evidence
 * and what re-enabling takes.
 */
static int vcp_encoder_set_param(void *handle, enum venc_set_param_type type,
				      struct venc_enc_param *p)
{
	struct vcp_encoder_handle *h = handle;
	struct vcp_venc_config config = {};
	u32 sizes[VCP_VENC_PLANES] = {};
	u32 id, value = 0, dst_fourcc;
	size_t count = 1;
	bool synchronous;
	struct mtk_q_data *q;
	unsigned int i;
	bool ten_bit;
	int ret, cleanup;

	if (!h || h->failed || !h->inst)
		return -EIO;
	if (type != VENC_SET_PARAM_ENC) {
		if (!h->configured)
			return -EINVAL;
		/* FORCE_INTRA/PREPEND_HEADER are zero-payload vendor commands. */
		if (!p && type != VENC_SET_PARAM_FORCE_INTRA &&
		    type != VENC_SET_PARAM_PREPEND_HEADER)
			return -EINVAL;
		switch (type) {
		case VENC_SET_PARAM_FORCE_INTRA:
			id = VCP_VENC_PARAM_FORCE_INTRA;
			count = 0;
			break;
		case VENC_SET_PARAM_PREPEND_HEADER:
			id = VCP_VENC_PARAM_PREPEND_HEADER;
			count = 0;
			break;
		case VENC_SET_PARAM_ADJUST_BITRATE:
			id = VCP_VENC_PARAM_BITRATE;
			value = p->bitrate;
			break;
		case VENC_SET_PARAM_ADJUST_FRAMERATE:
			id = VCP_VENC_PARAM_FRAMERATE;
			value = p->frm_rate;
			if (!value)
				return -EINVAL;
			break;
		case VENC_SET_PARAM_GOP_SIZE:
			id = VCP_VENC_PARAM_GOP_SIZE;
			value = p->gop_size;
			break;
		case VENC_SET_PARAM_INTRA_PERIOD:
			id = VCP_VENC_PARAM_INTRA_PERIOD;
			value = p->intra_period;
			break;
		default:
			return -EOPNOTSUPP;
		}
		return mtk_vcp_venc_set_param(h->inst, id, count ? &value : NULL, count);
	}
	if (!p || !p->frm_rate)
		return -EINVAL;
	switch (p->input_yuv_fmt) {
	case VENC_YUV_FORMAT_I420:
	case VENC_YUV_FORMAT_YV12:
	case VENC_YUV_FORMAT_NV12:
	case VENC_YUV_FORMAT_NV21:
	case VENC_YUV_FORMAT_P010:
		break;
	default:
		return -EINVAL;
	}
	q = &h->ctx->q_data[MTK_Q_DATA_SRC];
	ret = vcp_venc_calc_layout(q->fmt->fourcc, q->coded_width,
				   q->coded_height, &h->input_layout);
	if (ret)
		return ret;
	dst_fourcc = h->ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc;
	/* Verified encode boxes (visible geometry). H.264 corrupts past
	 * 4096 macroblock columns on this firmware; HEVC cannot allocate
	 * at 4096 and up; HEIF stills share the HEVC box. SRC negotiation
	 * cannot know the DST codec, so this is the deterministic gate.
	 */
	{
		unsigned int max_w, max_h;

		switch (dst_fourcc) {
		case V4L2_PIX_FMT_H264:
			max_w = 4096; max_h = 4320;
			break;
		case V4L2_PIX_FMT_HEVC:
		case v4l2_fourcc('H', 'E', 'I', 'F'):
			max_w = 3840; max_h = 2160;
			break;
		default:
			return -EINVAL;
		}
		if (q->visible_width > max_w || q->visible_height > max_h)
			return -EINVAL;
	}
	/* No dims-vs-level gate: the firmware decodes a 3840x2160
	 * stream carrying a level 4.0 SPS exactly (30/30), and software
	 * decoders treat level as advisory, so a mismatch is tolerated end
	 * to end. Clients that need spec-conformant headers can set a
	 * fitting level; the 4K capability raise makes 5.1 selectable.
	 */
	/* Despite its name this vendor field contains venc_yuv_fmt, not FourCC. */
	config.input_fourcc = cpu_to_le32(p->input_yuv_fmt);
	config.bitrate = cpu_to_le32(p->bitrate);
	/* Vendor passes the V4L2 bitrate-mode enum straight through; only the
	 * modes with verified behavior are accepted, CQ is not.
	 */
	if (p->bitrate_mode != V4L2_MPEG_VIDEO_BITRATE_MODE_CBR &&
	    p->bitrate_mode != V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
		return -EINVAL;
	config.bitratemode = cpu_to_le32(p->bitrate_mode);
	config.pic_w = cpu_to_le32(q->visible_width);
	config.pic_h = cpu_to_le32(q->visible_height);
	config.buf_w = cpu_to_le32(h->input_layout.buf_width);
	config.buf_h = cpu_to_le32(h->input_layout.buf_height);
	config.gop_size = cpu_to_le32(p->gop_size);
	/* Firmware emits an IDR every frame when intra_period is 0, which
	 * also makes B-frames and rate-control measurements meaningless
	 * (every historical default-config encode was all-IDR). V4L2 leaves
	 * 0 as "unspecified" here, so follow the GOP interval instead; the
	 * control value itself is untouched for readback. Verified: GOP=15
	 * default then yields periodic IDRs, IPERIOD=60 gives 1 IDR + 59 P.
	 */
	config.intra_period = cpu_to_le32(p->intra_period ?
					  p->intra_period : p->gop_size);
	config.framerate = cpu_to_le32(p->frm_rate);
	config.profile = cpu_to_le32(p->h264_profile);
	config.level = cpu_to_le32(p->h264_level);
	/* B-frames reorder in firmware; single stills cannot use them.
	 * Timestamp restore is by completion cookie, not by queue order,
	 * so reordered completions keep their own PTS.
	 */
	if (dst_fourcc == v4l2_fourcc('H', 'E', 'I', 'F')) {
		if (p->num_b_frame)
			return -EINVAL;
	} else if (p->num_b_frame > 2) {
		return -EINVAL;
	}
	config.num_b_frame = cpu_to_le32(p->num_b_frame);
	config.max_qp = cpu_to_le32(h->ctx->enc_params.h264_max_qp);
	/* P010 is the only 10-bit input with a V4L2 mapping; MT10 tile mode
	 * has no userspace layout, so it is never advertised or accepted.
	 */
	ten_bit = (p->input_yuv_fmt == VENC_YUV_FORMAT_P010);
	if (dst_fourcc == V4L2_PIX_FMT_HEVC ||
	    dst_fourcc == v4l2_fourcc('H', 'E', 'I', 'F')) {
		const struct mtk_enc_params *params = &h->ctx->enc_params;
		bool main10 = params->hevc_profile == V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10;

		/* Profile and level are firmware values, not V4L2 enum ordinals.
		 * HEIF stills share the HEVC profile/level mapping; the still
		 * bitstream is one coded picture per submitted frame.
		 */
		if ((!main10 && params->hevc_profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN) ||
		    params->hevc_level > V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2 ||
		    params->hevc_tier > V4L2_MPEG_VIDEO_HEVC_TIER_HIGH ||
		    main10 != ten_bit)
			return -EINVAL;
		ret = vcp_hevc_level(params->hevc_level, params->hevc_tier);
		if (ret < 0)
			return ret;
		config.profile = cpu_to_le32(main10 ? 4 : 2);
		config.level = cpu_to_le32(ret);
		config.max_qp = cpu_to_le32(params->hevc_max_qp);
		if (dst_fourcc == v4l2_fourcc('H', 'E', 'I', 'F'))
			config.heif_grid_size = cpu_to_le32(p->heif_grid_size);
	} else if (ten_bit) {
		/* Only HEVC/HEIF map 10-bit input (Main10) to a firmware
		 * profile. Anything else with P010 pixels would be emitted
		 * with an 8-bit profile label (observed: H.264 High
		 * profile_idc=100 carrying 10-bit depths), so fail closed.
		 */
		return -EINVAL;
	}
	/* MPEG4-Part2 and H.263 have confirmed codec IDs, but this firmware
	 * build NACKs their INIT, so the exposure gate keeps those fourccs
	 * out and no per-codec branch exists here. See ENCODER-EXPANSION.md
	 * for the evidence and the vendor mapping tables to restore.
	 */
	ret = vcp_encoder_fill_color(h->ctx, ten_bit, config.color_desc);
	if (ret)
		return ret;
	h->configured = false;
	ret = mtk_vcp_venc_configure(h->inst, &config, sizes, &synchronous);
	if (ret)
		return ret;
	/* In asynchronous mode normal frames are completed by PUT_BUFFER work. */
	for (i = 0; i < VCP_VENC_PLANES; i++) {
		if (i < q->fmt->num_planes) {
			/* Firmware minima apply to private padded DMA storage, not
			 * to the public source allocation.
			 */
			if (sizes[i] > h->input_layout.dst_size[i]) {
				ret = -EINVAL;
				goto rollback;
			}
		} else if (sizes[i]) {
			ret = -EPROTO;
			goto rollback;
		}
	}
	h->synchronous = synchronous;
	/*
	 * The MT6895 VCP ABI exposes one shared venc info slot. The firmware
	 * acknowledges ENCODE before it publishes the corresponding completion,
	 * but the hardware path does not accept a second frame while the first
	 * slot is still in flight. Keep the protocol's sync_mode value above,
	 * while serializing frontend jobs until each completion ring item has
	 * been consumed.
	 */
	/* B-frame reorder delay requires multiple frames in flight: the
	 * firmware holds reference inputs across submits and completes out
	 * of order, so a strictly serialized frontend deadlocks after the
	 * first completion (verified: 1/7 returned, then -ETIMEDOUT).
	 * Firmware advertises async operation and accepts pipelined
	 * submits; completions are matched by cookie, restoring each
	 * picture's own timestamp. Sessions without B-frames stay
	 * serialized, preserving all previously validated behavior.
	 */
	h->serialized = !vcp_force_async && !p->num_b_frame;
	h->configured = true;
	return 0;

rollback:
	/* SET_PARAM already committed the configuration in firmware. Tear the
	 * session down so the frontend and protocol instance cannot disagree
	 * about whether frame submission is allowed.
	 */
	h->failed = true;
	cleanup = vcp_encoder_stop(h, true);
	if (cleanup) {
		h->dev->vcp_faulted = true;
		dev_err(&h->dev->plat_dev->dev,
			"VCP configure rollback failed: %d\n", cleanup);
	}
	return ret;
}

static int vcp_encoder_encode(void *handle, enum venc_start_opt opt,
				   struct venc_frm_buf *frm, struct mtk_vcodec_mem *bs,
				   struct venc_done_result *result)
{
	struct vcp_encoder_handle *h = handle;
	struct vcp_venc_result done;
	struct vcp_venc_buffer_ids ids;
	struct vcp_encoder_pending *pending = NULL;
	unsigned long deadline;
	bool frame_done, bitstream_done = false;
	int ret, cleanup;

	if (!h || !result ||
	    (opt != VENC_START_OPT_ENCODE_FRAME_FINAL && !bs))
		return -EINVAL;
	memset(result, 0, sizeof(*result));
	if (h->failed || !h->configured || !h->inst)
		return -EIO;
	if (opt != VENC_START_OPT_ENCODE_SEQUENCE_HEADER &&
	    opt != VENC_START_OPT_ENCODE_FRAME &&
	    opt != VENC_START_OPT_ENCODE_FRAME_FINAL)
		return -EINVAL;
	if (opt != VENC_START_OPT_ENCODE_FRAME_FINAL &&
	    !h->ctx->active_dst)
		return -EINVAL;
	if (opt == VENC_START_OPT_ENCODE_FRAME) {
		if (!h->ctx->active_src)
			return -EINVAL;
		if (!h->synchronous && !h->serialized) {
			pending = vcp_encoder_pending_slot(h);
			if (!pending)
				return -ENOSPC;
		}
	}
	ret = mtk_vcp_venc_submit_vb2(h->inst,
			opt == VENC_START_OPT_ENCODE_SEQUENCE_HEADER ? 2 :
			opt == VENC_START_OPT_ENCODE_FRAME ? 3 : 4,
			opt == VENC_START_OPT_ENCODE_FRAME_FINAL ? NULL :
				h->ctx->active_src,
			opt == VENC_START_OPT_ENCODE_FRAME_FINAL ? NULL :
				h->ctx->active_dst, &h->input_layout, &ids);
	if (ret)
		goto fail;
	if (pending) {
		if (!ids.frame || !ids.bitstream) {
			ret = -EPROTO;
			goto fail;
		}
		pending->frame_cookie = ids.frame;
		pending->bitstream_cookie = ids.bitstream;
		pending->src = to_vb2_v4l2_buffer(h->ctx->active_src);
		pending->dst = to_vb2_v4l2_buffer(h->ctx->active_dst);
		pending->timestamp = pending->src->vb2_buf.timestamp;
		pending->timecode = pending->src->timecode;
		result->async = true;
		return 0;
	}
	deadline = jiffies + msecs_to_jiffies(2000);
	if (opt == VENC_START_OPT_ENCODE_FRAME_FINAL) {
		/* Trailing B-frame references release seconds after EOS on
		 * this firmware (3 s linger verified for 640x480x60); a
		 * 2 s drain budget turns healthy drains into ETIMEDOUT.
		 * Normal submits keep the 2 s budget below.
		 */
		deadline = jiffies + msecs_to_jiffies(10000);
		for (;;) {
			unsigned long seq = READ_ONCE(h->dev->vcp_notify_seq);
			long left;

			while (!(ret = mtk_vcp_venc_dequeue(h->inst, &done))) {
				ret = vcp_encoder_complete(h, &done);
				if (ret)
					goto fail;
			}
			if (ret != -EAGAIN)
				goto fail;
			if (vcp_encoder_pending_empty(h))
				return 0;
			left = deadline - jiffies;
			if (left <= 0) {
				ret = -ETIMEDOUT;
				goto fail;
			}
			ret = wait_event_interruptible_timeout(h->dev->vcp_wait,
				READ_ONCE(h->dev->vcp_notify_seq) != seq, left);
			if (ret > 0)
				continue;
			if (!ret)
				ret = -ETIMEDOUT;
			goto fail;
		}
	}
	frame_done = !ids.frame;
	bitstream_done = !ids.bitstream;
	while (!frame_done || !bitstream_done) {
		unsigned long seq = READ_ONCE(h->dev->vcp_notify_seq);
		long left;

		ret = mtk_vcp_venc_dequeue(h->inst, &done);
		if (ret == -EAGAIN) {
			left = deadline - jiffies;
			if (left <= 0) {
				ret = -ETIMEDOUT;
				goto fail;
			}
			ret = wait_event_interruptible_timeout(h->dev->vcp_wait,
				READ_ONCE(h->dev->vcp_notify_seq) != seq,
				left);
			if (ret > 0)
				continue;
			if (!ret)
				ret = -ETIMEDOUT;
			goto fail;
		}
		if (ret)
			goto fail;
		if ((!done.frame_cookie && !done.bitstream_cookie) ||
		    (done.frame_cookie && (frame_done || done.frame_cookie != ids.frame)) ||
		    (done.bitstream_cookie && (bitstream_done ||
					      done.bitstream_cookie != ids.bitstream)) ||
		    (!done.bitstream_cookie && done.bytes) || done.bytes > bs->size) {
			ret = -EPROTO;
			goto fail;
		}
		if (done.frame_cookie)
			frame_done = true;
		if (done.bitstream_cookie) {
			bitstream_done = true;
			result->bs_size = done.bytes;
			result->is_key_frm = done.keyframe;
		}
	}
	return 0;
fail:
	h->failed = true;
	h->ctx->state = MTK_STATE_ABORT;
	memset(result, 0, sizeof(*result));
	cleanup = vcp_encoder_stop(h, false);
	if (cleanup) {
		h->dev->vcp_faulted = true;
		dev_err(&h->dev->plat_dev->dev,
			"VCP encode failed %d, stop failed %d; private DMA retained\n",
			ret, cleanup);
	}
	return ret;
}

void venc_vcp_encoder_buffers_ready(struct mtk_vcodec_enc_dev *dev)
{
	struct vcp_encoder_handle *h = dev->vcp_session;
	struct vcp_venc_result done;
	unsigned int completed = 0;
	int ret, cleanup;

	if (!h || !h->ctx || !h->inst || h->failed || h->synchronous ||
	    h->serialized)
		return;
	while (!(ret = mtk_vcp_venc_dequeue(h->inst, &done))) {
		dev_dbg(&dev->plat_dev->dev,
			 "VENC dequeue: frame=%#llx bitstream=%#llx bytes=%u keyframe=%u\n",
			 done.frame_cookie, done.bitstream_cookie, done.bytes,
			 done.keyframe);
		ret = vcp_encoder_complete(h, &done);
		if (ret)
			break;
		completed++;
	}
	if (ret == -EAGAIN) {
		if (completed)
			dev_dbg(&dev->plat_dev->dev,
				 "VENC completion worker returned %u buffer pairs\n",
				 completed);
		return;
	}
	h->failed = true;
	h->ctx->state = MTK_STATE_ABORT;
	vcp_encoder_abort_pending(h);
	vb2_queue_error(&h->ctx->m2m_ctx->out_q_ctx.q);
	vb2_queue_error(&h->ctx->m2m_ctx->cap_q_ctx.q);
	cleanup = vcp_encoder_stop(h, false);
	if (cleanup) {
		dev->vcp_faulted = true;
		dev_err(&dev->plat_dev->dev,
			"VCP async completion failed %d, stop failed %d\n",
			ret, cleanup);
	} else {
		dev_err(&dev->plat_dev->dev,
			"VCP async completion failed: %d\n", ret);
	}
}

static int vcp_encoder_deinit(void *handle)
{
	struct vcp_encoder_handle *h = handle;
	int ret;

	if (!h)
		return 0;
	ret = vcp_encoder_stop(h, !h->failed);
	vcp_encoder_dispose(h, ret);
	return ret;
}

const struct venc_common_if venc_vcp_encoder_if = {
	.init = vcp_encoder_init,
	.encode = vcp_encoder_encode,
	.set_param = vcp_encoder_set_param,
	.deinit = vcp_encoder_deinit,
};
