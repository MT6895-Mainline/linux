// SPDX-License-Identifier: GPL-2.0-only
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/videobuf2-v4l2.h>

#include "mtk_vcodec_enc.h"
#include "../vcp/mtk_vcp_venc_abi.h"
#include "venc_drv_base.h"

struct vcp_h264_pending {
	u64 frame_cookie, bitstream_cookie;
	struct vb2_v4l2_buffer *src, *dst;
	u64 timestamp;
	struct v4l2_timecode timecode;
};

struct vcp_h264_handle {
	struct mtk_vcodec_enc_ctx *ctx;
	struct mtk_vcodec_enc_dev *dev;
	struct mtk_vcp_venc_inst *inst;
	struct vcp_h264_pending pending[VCP_VENC_BUFFERS];
	bool booted, initialized, configured, synchronous, serialized, failed;
};

static void vcp_h264_abort_pending(struct vcp_h264_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++) {
		struct vcp_h264_pending *p = &h->pending[i];

		if (p->src)
			v4l2_m2m_buf_done(p->src, VB2_BUF_STATE_ERROR);
		if (p->dst) {
			vb2_set_plane_payload(&p->dst->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(p->dst, VB2_BUF_STATE_ERROR);
		}
		memset(p, 0, sizeof(*p));
	}
}

static struct vcp_h264_pending *vcp_h264_pending_slot(struct vcp_h264_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++)
		if (!h->pending[i].frame_cookie && !h->pending[i].bitstream_cookie)
			return &h->pending[i];
	return NULL;
}

static bool vcp_h264_pending_empty(struct vcp_h264_handle *h)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h->pending); i++)
		if (h->pending[i].frame_cookie || h->pending[i].bitstream_cookie)
			return false;
	return true;
}

static int vcp_h264_complete(struct vcp_h264_handle *h,
			     const struct vcp_venc_result *done)
{
	struct vcp_h264_pending *frame = NULL, *bitstream = NULL, *p;
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
static int vcp_h264_stop(struct vcp_h264_handle *h, bool graceful)
{
	struct mtk_vcodec_enc_dev *dev = h->dev;
	int ret;

	h->configured = false;
	vcp_h264_abort_pending(h);
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
	/*
	 * Dropping the boot reference only unloads the VCP when no other codec
	 * session holds one, so a decoder that keeps the firmware running is not
	 * an encoder teardown failure. The session is only retained when this
	 * encoder still owns a firmware instance whose hardware could not be
	 * quiesced.
	 */
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
	return 0;
}

static void vcp_h264_dispose(struct vcp_h264_handle *h, int cleanup)
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
	kfree(h);
	module_put(THIS_MODULE);
}

static int vcp_h264_init(struct mtk_vcodec_enc_ctx *ctx)
{
	struct mtk_vcodec_enc_dev *dev = ctx->dev;
	struct vcp_h264_handle *h;
	int ret, cleanup;

	if (!dev->vcp_venc || dev->vcp_faulted)
		return -EIO;
	if (dev->vcp_session)
		return -EBUSY;
	h = kzalloc_obj(*h);
	if (!h)
		return -ENOMEM;
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
	cleanup = vcp_h264_stop(h, false);
	vcp_h264_dispose(h, cleanup);
	return ret;
}

static int vcp_h264_set_param(void *handle, enum venc_set_param_type type,
				      struct venc_enc_param *p)
{
	struct vcp_h264_handle *h = handle;
	struct vcp_venc_config config = {};
	u32 sizes[VCP_VENC_PLANES] = {};
	u32 id, value = 0;
	size_t count = 1;
	bool synchronous;
	struct mtk_q_data *q;
	unsigned int i;
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
		break;
	default:
		return -EINVAL;
	}
	q = &h->ctx->q_data[MTK_Q_DATA_SRC];
	/* Despite its name this vendor field contains venc_yuv_fmt, not FourCC. */
	config.input_fourcc = cpu_to_le32(p->input_yuv_fmt);
	config.bitrate = cpu_to_le32(p->bitrate);
	config.bitratemode = cpu_to_le32(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	config.pic_w = cpu_to_le32(q->visible_width);
	config.pic_h = cpu_to_le32(q->visible_height);
	config.buf_w = cpu_to_le32(q->coded_width);
	config.buf_h = cpu_to_le32(q->coded_height);
	config.gop_size = cpu_to_le32(p->gop_size);
	config.intra_period = cpu_to_le32(p->intra_period);
	config.framerate = cpu_to_le32(p->frm_rate);
	config.profile = cpu_to_le32(p->h264_profile);
	config.level = cpu_to_le32(p->h264_level);
	config.num_b_frame = cpu_to_le32(0);
	config.max_qp = cpu_to_le32(h->ctx->enc_params.h264_max_qp);
	h->configured = false;
	ret = mtk_vcp_venc_configure(h->inst, &config, sizes, &synchronous);
	if (ret)
		return ret;
	/* In asynchronous mode normal frames are completed by PUT_BUFFER work. */
	for (i = 0; i < VCP_VENC_PLANES; i++) {
		if (i < q->fmt->num_planes) {
			/* Some VCP firmware revisions leave optional sizeimage entries
			 * zero. The V4L2 capture format already owns the usable buffer
			 * size; reject only a firmware requirement that exceeds it.
			 */
			if (sizes[i] > q->sizeimage[i]) {
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
	h->serialized = true;
	h->configured = true;
	return 0;

rollback:
	/* SET_PARAM already committed the configuration in firmware. Tear the
	 * session down so the frontend and protocol instance cannot disagree
	 * about whether frame submission is allowed.
	 */
	h->failed = true;
	cleanup = vcp_h264_stop(h, true);
	if (cleanup) {
		h->dev->vcp_faulted = true;
		dev_err(&h->dev->plat_dev->dev,
			"VCP configure rollback failed: %d\n", cleanup);
	}
	return ret;
}

static int vcp_h264_encode(void *handle, enum venc_start_opt opt,
				   struct venc_frm_buf *frm, struct mtk_vcodec_mem *bs,
				   struct venc_done_result *result)
{
	struct vcp_h264_handle *h = handle;
	struct vcp_venc_result done;
	struct vcp_venc_buffer_ids ids;
	struct vcp_h264_pending *pending = NULL;
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
			pending = vcp_h264_pending_slot(h);
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
				h->ctx->active_dst, &ids);
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
		for (;;) {
			unsigned long seq = READ_ONCE(h->dev->vcp_notify_seq);
			long left;

			while (!(ret = mtk_vcp_venc_dequeue(h->inst, &done))) {
				ret = vcp_h264_complete(h, &done);
				if (ret)
					goto fail;
			}
			if (ret != -EAGAIN)
				goto fail;
			if (vcp_h264_pending_empty(h))
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
	cleanup = vcp_h264_stop(h, false);
	if (cleanup) {
		h->dev->vcp_faulted = true;
		dev_err(&h->dev->plat_dev->dev,
			"VCP encode failed %d, stop failed %d; private DMA retained\n",
			ret, cleanup);
	}
	return ret;
}

void venc_vcp_h264_buffers_ready(struct mtk_vcodec_enc_dev *dev)
{
	struct vcp_h264_handle *h = dev->vcp_session;
	struct vcp_venc_result done;
	unsigned int completed = 0;
	int ret, cleanup;

	if (!h || !h->ctx || !h->inst || h->failed || h->synchronous ||
	    h->serialized)
		return;
	while (!(ret = mtk_vcp_venc_dequeue(h->inst, &done))) {
		dev_info(&dev->plat_dev->dev,
			 "VENC dequeue: frame=%#llx bitstream=%#llx bytes=%u keyframe=%u\n",
			 done.frame_cookie, done.bitstream_cookie, done.bytes,
			 done.keyframe);
		ret = vcp_h264_complete(h, &done);
		if (ret)
			break;
		completed++;
	}
	if (ret == -EAGAIN) {
		if (completed)
			dev_info(&dev->plat_dev->dev,
				 "VENC completion worker returned %u buffer pairs\n",
				 completed);
		return;
	}
	h->failed = true;
	h->ctx->state = MTK_STATE_ABORT;
	vcp_h264_abort_pending(h);
	vb2_queue_error(&h->ctx->m2m_ctx->out_q_ctx.q);
	vb2_queue_error(&h->ctx->m2m_ctx->cap_q_ctx.q);
	cleanup = vcp_h264_stop(h, false);
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

static int vcp_h264_deinit(void *handle)
{
	struct vcp_h264_handle *h = handle;
	int ret;

	if (!h)
		return 0;
	ret = vcp_h264_stop(h, !h->failed);
	vcp_h264_dispose(h, ret);
	return ret;
}

const struct venc_common_if venc_vcp_h264_if = {
	.init = vcp_h264_init,
	.encode = vcp_h264_encode,
	.set_param = vcp_h264_set_param,
	.deinit = vcp_h264_deinit,
};
