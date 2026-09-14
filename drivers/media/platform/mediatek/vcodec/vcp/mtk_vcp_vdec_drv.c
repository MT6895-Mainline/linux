// SPDX-License-Identifier: GPL-2.0-only
/* Stateful V4L2 H.264 decoding through the MT6895 VCP firmware. */
#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-vmalloc.h>
#include "mtk_vcp_vdec_hw.h"

#define DEC_SURFACES 36
struct vdec_ctx;
struct vdec_dev {
	struct device *dev, *bs_dev, *ube_dev;
	struct mtk_vcp *vcp;
	struct mtk_vcp_vdec_hw *hw;
	struct video_device video;
	struct v4l2_device v4l2;
	struct v4l2_m2m_dev *m2m;
	struct workqueue_struct *queue;
	struct mutex lock;
	struct vdec_ctx *ctx;
};
struct vdec_surface {
	struct mtk_vcp_mem plane[2];
	u64 cookie;
	bool free, pending;
};
struct vdec_pending {
	u32 surface;
	u64 timestamp;
};
struct vdec_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler controls;
	struct vdec_dev *dev;
	struct mtk_vcp_vdec *decoder;
	struct v4l2_pix_format_mplane src_fmt, dst_fmt;
	struct vcp_vdec_picture pic;
	struct mtk_vcp_mem bs;
	struct vdec_surface surfaces[DEC_SURFACES];
	struct vdec_pending pending[DEC_SURFACES];
	u32 pool_count, pending_read, pending_count, sequence, source_sequence;
	u64 next_cookie, source_cookie;
	struct work_struct work;
	wait_queue_head_t wait;
	atomic_t notification;
	bool booted, initialized, header, stopping, failed, orphan;
	bool source_done, draining, drained;
	bool submitted;   /* head OUTPUT buffer handed to firmware, release pending */
	bool last_pending; /* previous capture sequence still needs its LAST marker */
	bool wait_capture; /* new sequence waits for the client to restart CAPTURE */
};

static struct vdec_ctx *file_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct vdec_ctx, fh);
}
static bool is_output(enum v4l2_buf_type type)
{
	return type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
}
static bool valid_type(enum v4l2_buf_type type)
{
	return is_output(type) || type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
}
static struct v4l2_pix_format_mplane *queue_format(struct vdec_ctx *c, enum v4l2_buf_type type)
{
	return is_output(type) ? &c->src_fmt : &c->dst_fmt;
}

static int codec_power(void *priv, unsigned int core, bool on)
{
	struct vdec_ctx *c = priv;

	return mtk_vcp_vdec_hw_power(c->dev->hw, core, on);
}
static int codec_wait(void *priv, unsigned int core)
{
	struct vdec_ctx *c = priv;

	return mtk_vcp_vdec_hw_wait(c->dev->hw, core);
}
static int codec_alloc(void *priv, u32 type, size_t size, struct mtk_vcp_mem *mem)
{
	struct vdec_ctx *c = priv;

	return mtk_vcp_vdec_hw_alloc(c->dev->hw, type, size, mem);
}
static void codec_free(void *priv, u32 type, struct mtk_vcp_mem *mem)
{
	struct vdec_ctx *c = priv;

	mtk_vcp_vdec_hw_free(c->dev->hw, type, mem);
}
static void codec_notify(void *priv)
{
	struct vdec_ctx *c = priv;

	atomic_inc(&c->notification);
	wake_up(&c->wait);
}
static const struct mtk_vcp_vdec_ops codec_ops = {
	.power = codec_power, .wait_irq = codec_wait,
	.alloc = codec_alloc, .free = codec_free, .notify = codec_notify,
};

static int session_teardown(struct vdec_ctx *c);

static int session_boot(struct vdec_ctx *c)
{
	int ret;

	/* Device ownership is already held by the caller. */
	if (c->decoder)
		return 0;
	c->decoder = mtk_vcp_vdec_create(c->dev->dev, c->dev->vcp, &codec_ops, c);
	if (IS_ERR(c->decoder)) {
		ret = PTR_ERR(c->decoder);
		c->decoder = NULL;
		cmpxchg(&c->dev->ctx, c, NULL);
		return ret;
	}
	ret = mtk_vcp_boot(c->dev->vcp);
	if (ret) {
		dev_info(c->dev->dev, "session boot failed: %d\n", ret);
		return ret;
	}
	c->booted = true;
	ret = mtk_vcp_vdec_init(c->decoder);
	if (ret) {
		dev_info(c->dev->dev, "session init failed: %d\n", ret);
		return ret;
	}
	c->initialized = true;
	c->bs.size = c->src_fmt.plane_fmt[0].sizeimage;
	c->bs.cpu = dma_alloc_coherent(c->dev->bs_dev, c->bs.size, &c->bs.dma, GFP_KERNEL);
	if (c->bs.cpu)
		return 0;
	/* A session without its bitstream buffer must not be left half
	 * initialized: a later CAPTURE restart would otherwise reuse it and
	 * write through the missing mapping.
	 */
	ret = session_teardown(c);
	if (ret)
		return ret;
	cmpxchg(&c->dev->ctx, c, NULL);
	return -ENOMEM;
}

static int session_start(struct vdec_ctx *c)
{
	/* A retained session is only usable once firmware and bitstream DMA
	 * both exist.
	 */
	if (c->decoder)
		return c->initialized && c->bs.cpu ? 0 : -EIO;
	/* Other file handles may inspect formats, but only one owns VDEC. */
	if (cmpxchg(&c->dev->ctx, NULL, c))
		return -EBUSY;
	if (!mtk_vcp_is_offline(c->dev->vcp)) {
		cmpxchg(&c->dev->ctx, c, NULL);
		return -EBUSY;
	}
	return session_boot(c);
}

/* Tears down firmware, VCP, hardware and DMA, but keeps device ownership
 * so the caller can immediately boot a replacement session.
 */
static int session_teardown(struct vdec_ctx *c)
{
	int ret = 0, stopped, i, j;

	if (!c->decoder)
		return 0;
	if (c->initialized)
		ret = mtk_vcp_vdec_deinit(c->decoder);
	else
		ret = -EIO;
	stopped = c->booted ? mtk_vcp_shutdown(c->dev->vcp) : 0;
	if (stopped || (ret && !mtk_vcp_is_offline(c->dev->vcp)) ||
	    mtk_vcp_vdec_hw_stop(c->dev->hw))
		goto retain;
	stopped = mtk_vcp_vdec_destroy(c->decoder, !!ret);
	if (stopped)
		goto retain;
	c->decoder = NULL;
	c->booted = false;
	c->initialized = false;
	if (c->bs.cpu)
		dma_free_coherent(c->dev->bs_dev, c->bs.size, c->bs.cpu, c->bs.dma);
	memset(&c->bs, 0, sizeof(c->bs));
	for (i = 0; i < DEC_SURFACES; i++)
		for (j = 0; j < 2; j++)
			if (c->surfaces[i].plane[j].cpu)
				codec_free(c, 1, &c->surfaces[i].plane[j]);
	memset(c->surfaces, 0, sizeof(c->surfaces));
	c->pool_count = 0;
	c->pending_count = 0;
	c->pending_read = 0;
	return 0;
retain:
	if (!c->orphan) {
		c->orphan = true;
		__module_get(THIS_MODULE);
		dev_err(c->dev->dev, "decoder shutdown uncertain; retaining session and DMA\n");
	}
	return -EIO;
}

static int session_stop(struct vdec_ctx *c)
{
	int ret = session_teardown(c);

	if (ret)
		return ret;
	cmpxchg(&c->dev->ctx, c, NULL);
	return 0;
}

static int collect_events(struct vdec_ctx *c)
{
	struct vcp_vdec_event event;
	int ret;
	unsigned int i;

	while (!(ret = mtk_vcp_vdec_event(c->decoder, &event))) {
		if (event.type == VCP_VDEC_FREE_BITSTREAM) {
			if (event.cookie != c->source_cookie)
				return -EPROTO;
			c->source_done = true;
			continue;
		}
		for (i = 0; i < c->pool_count; i++)
			if (c->surfaces[i].cookie == event.cookie)
				break;
		if (i == c->pool_count)
			return -EPROTO;
		if (event.type == VCP_VDEC_FREE_FRAME) {
			c->surfaces[i].free = true;
			continue;
		}
		if (c->pending_count == DEC_SURFACES || c->surfaces[i].pending)
			return -EOVERFLOW;
		c->surfaces[i].pending = true;
		c->pending[(c->pending_read + c->pending_count++) % DEC_SURFACES] =
			(struct vdec_pending){ .surface = i, .timestamp = event.timestamp };
	}
	return ret == -EAGAIN ? 0 : ret;
}

/* MM21: 16x32 luma tiles and 16x16 interleaved chroma tiles in raster order. */
static void detile(void *destination, const void *source, u32 stride, u32 height, u32 tile_h)
{
	u32 x, y;

	for (y = 0; y < height; y++)
		for (x = 0; x < stride; x += 16) {
			u32 offset = (y / tile_h * (stride / 16) + x / 16) * tile_h * 16;

			memcpy(destination + y * stride + x, source + offset + y % tile_h * 16, 16);
		}
}

/* A capture buffer can only receive the picture it was sized for. After a
 * midstream resolution change the queue still holds buffers of the previous
 * picture; writing the new one there runs past the end of the mapping.
 */
static bool capture_fits(const struct vdec_ctx *c, struct vb2_v4l2_buffer *vb)
{
	u32 luma = c->pic.size[0], chroma = c->pic.size[1];

	if (c->dst_fmt.num_planes == 1)
		return vb2_plane_size(&vb->vb2_buf, 0) >= (size_t)luma + chroma;
	return vb2_plane_size(&vb->vb2_buf, 0) >= luma &&
	       vb2_plane_size(&vb->vb2_buf, 1) >= chroma;
}

static int deliver_frames(struct vdec_ctx *c)
{
	struct vb2_v4l2_buffer *vb;

	while (c->pending_count && (vb = v4l2_m2m_next_dst_buf(c->fh.m2m_ctx))) {
		struct vdec_pending *p = &c->pending[c->pending_read];
		struct vdec_surface *s = &c->surfaces[p->surface];

		v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx);
		/* detile() writes a whole picture: a buffer queued for an
		 * earlier resolution only fits part of it and must be handed
		 * back instead of written past its end.
		 */
		if (!capture_fits(c, vb)) {
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
			continue;
		}
		if (c->dst_fmt.num_planes == 1) {
			u8 *base = vb2_plane_vaddr(&vb->vb2_buf, 0);

			if (!base) {
				v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
				return -EFAULT;
			}
			dma_rmb();
			detile(base, s->plane[0].cpu,
			       c->pic.stride, c->pic.buffer_height, 32);
			detile(base + c->pic.size[0], s->plane[1].cpu,
			       c->pic.stride, c->pic.buffer_height / 2, 16);
			vb2_set_plane_payload(&vb->vb2_buf, 0,
					      c->pic.size[0] + c->pic.size[1]);
		} else {
			void *y = vb2_plane_vaddr(&vb->vb2_buf, 0);
			void *uv = vb2_plane_vaddr(&vb->vb2_buf, 1);

			if (!y || !uv) {
				v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
				return -EFAULT;
			}
			dma_rmb();
			detile(y, s->plane[0].cpu,
			       c->pic.stride, c->pic.buffer_height, 32);
			detile(uv, s->plane[1].cpu,
			       c->pic.stride, c->pic.buffer_height / 2, 16);
			vb2_set_plane_payload(&vb->vb2_buf, 0, c->pic.size[0]);
			vb2_set_plane_payload(&vb->vb2_buf, 1, c->pic.size[1]);
		}
		vb->vb2_buf.timestamp = p->timestamp;
		vb->field = V4L2_FIELD_NONE;
		vb->sequence = c->sequence++;
		s->pending = false;
		c->pending_read = (c->pending_read + 1) % DEC_SURFACES;
		c->pending_count--;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	}
	return 0;
}

static int queue_surfaces(struct vdec_ctx *c)
{
	unsigned int i;
	int ret;

	for (i = 0; i < c->pool_count; i++) {
		struct vdec_surface *s = &c->surfaces[i];

		if (!s->free || s->pending)
			continue;
		s->free = false;
		s->cookie = ++c->next_cookie;
		ret = mtk_vcp_vdec_frame(c->decoder, s->cookie, i,
					 s->plane[0].dma, s->plane[1].dma);
		if (ret)
			return ret;
	}
	return 0;
}

static bool surfaces_idle(struct vdec_ctx *c)
{
	unsigned int i;

	if (c->pending_count)
		return false;
	for (i = 0; i < c->pool_count; i++)
		if (!c->surfaces[i].free)
			return false;
	return true;
}

/* Ends the capture sequence that belongs to the previous resolution. The last
 * buffer handed to the client must carry V4L2_BUF_FLAG_LAST; it may be empty
 * and is therefore not tied to a firmware frame. When the client has no
 * capture buffer queued yet the marker is deferred until one arrives.
 */
static int finish_old_sequence(struct vdec_ctx *c)
{
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst = v4l2_m2m_dst_buf_remove(m);

	if (!dst) {
		c->last_pending = true;
		return 0;
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
	if (c->dst_fmt.num_planes > 1)
		vb2_set_plane_payload(&dst->vb2_buf, 1, 0);
	dst->sequence = c->sequence++;
	v4l2_m2m_last_buffer_done(m, dst);
	c->last_pending = false;
	return 0;
}

static int parse_headers(struct vdec_ctx *c, struct vb2_v4l2_buffer *src);

/* Midstream resolution change. The old pictures are drained and delivered, the
 * capture sequence that belongs to them is terminated with a LAST buffer, and
 * the session is rebuilt for the geometry the new sequence describes. The new
 * resolution is only decoded once the client restarts CAPTURE, as the stateful
 * decoder specification requires. Already-decoded but undelivered tail frames
 * may be lost if the application stopped queueing capture buffers.
 */
static int res_change_restart(struct vdec_ctx *c, struct vb2_v4l2_buffer *src)
{
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	unsigned long deadline;
	int seq, ret;

	dev_info(c->dev->dev, "res_change: draining old resolution\n");
	ret = mtk_vcp_vdec_reset(c->decoder, true);
	if (ret)
		return ret;
	/* The flush dropped the queued access unit inside firmware; its release
	 * is not delivered any more.
	 */
	c->submitted = false;
	c->source_done = false;
	/* Pull already-decoded references without waiting for resources that
	 * a drain never returns; the flush below reclaims those.
	 */
	deadline = jiffies + msecs_to_jiffies(500);
	seq = atomic_read(&c->notification);
	for (;;) {
		ret = collect_events(c);
		if (!ret)
			ret = deliver_frames(c);
		if (ret)
			return ret;
		if (READ_ONCE(c->stopping))
			return -ECANCELED;
		if (time_after_eq(jiffies, deadline))
			break;
		wait_event_timeout(c->wait,
				   atomic_read(&c->notification) != seq ||
				   READ_ONCE(c->stopping),
				   msecs_to_jiffies(20));
		seq = atomic_read(&c->notification);
	}
	/* Drain does not return unused queued resources; a flush reclaims
	 * them and drops the queued new-resolution access unit, which the
	 * next job resubmits through the header path.
	 */
	ret = mtk_vcp_vdec_reset(c->decoder, false);
	if (ret)
		return ret;
	deadline = jiffies + msecs_to_jiffies(2000);
	seq = atomic_read(&c->notification);
	for (;;) {
		ret = collect_events(c);
		if (!ret)
			ret = deliver_frames(c);
		if (ret)
			return ret;
		if (surfaces_idle(c))
			break;
		if (READ_ONCE(c->stopping))
			return -ECANCELED;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		wait_event_timeout(c->wait,
				   atomic_read(&c->notification) != seq ||
				   READ_ONCE(c->stopping),
				   msecs_to_jiffies(20));
		seq = atomic_read(&c->notification);
	}
	ret = session_teardown(c);
	if (ret)
		return ret;
	ret = session_boot(c);
	if (ret)
		return ret;
	dev_info(c->dev->dev, "res_change: session rebuilt\n");
	c->header = false;
	m->ignore_cap_streaming = true;
	/* Queries issued after the SOURCE_CHANGE event must describe the stream
	 * that follows it, so the new sequence is parsed before it is published.
	 */
	if (src) {
		ret = parse_headers(c, src);
		if (ret && ret != -EAGAIN)
			return ret;
	}
	/* The new resolution is decoded once the client restarts CAPTURE. */
	c->wait_capture = true;
	return finish_old_sequence(c);
}

static int allocate_surfaces(struct vdec_ctx *c)
{
	unsigned int i, j;
	int ret;

	c->pool_count = c->pic.dpb + 3;
	if (c->pool_count > DEC_SURFACES ||
	    (u64)c->pool_count * (c->pic.size[0] + c->pic.size[1]) > SZ_256M)
		return -E2BIG;
	for (i = 0; i < c->pool_count; i++) {
		for (j = 0; j < 2; j++) {
			ret = codec_alloc(c, 1, c->pic.size[j], &c->surfaces[i].plane[j]);
			if (ret)
				return ret;
		}
		c->surfaces[i].free = true;
	}
	return 0;
}

/* Fill in the buffer layout for the picture the firmware parsed. The firmware
 * geometry is fixed, but the client chooses between the supported single- and
 * multi-planar layouts when it negotiates CAPTURE.
 */
static void picture_format(struct vdec_ctx *c, struct v4l2_pix_format_mplane *f)
{
	if (f->pixelformat != V4L2_PIX_FMT_NV12M &&
	    f->pixelformat != V4L2_PIX_FMT_NV12)
		f->pixelformat = V4L2_PIX_FMT_NV12M;

	f->width = c->pic.stride;
	f->height = c->pic.buffer_height;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = c->src_fmt.colorspace;
	f->xfer_func = c->src_fmt.xfer_func;
	f->ycbcr_enc = c->src_fmt.ycbcr_enc;
	f->quantization = c->src_fmt.quantization;
	if (f->pixelformat == V4L2_PIX_FMT_NV12) {
		f->num_planes = 1;
		f->plane_fmt[0].bytesperline = c->pic.stride;
		f->plane_fmt[0].sizeimage = c->pic.size[0] + c->pic.size[1];
	} else {
		f->num_planes = 2;
		f->plane_fmt[0].bytesperline = c->pic.stride;
		f->plane_fmt[1].bytesperline = c->pic.stride;
		f->plane_fmt[0].sizeimage = c->pic.size[0];
		f->plane_fmt[1].sizeimage = c->pic.size[1];
	}
}

static void capture_format(struct vdec_ctx *c)
{
	picture_format(c, &c->dst_fmt);
}

static int submit_source(struct vdec_ctx *c, struct vb2_v4l2_buffer *src, u32 *changed)
{
	struct vb2_plane *p = &src->vb2_buf.planes[0];
	void *data = vb2_plane_vaddr(&src->vb2_buf, 0);
	u32 bytes = p->bytesused - p->data_offset;

	/* A start code plus a NAL header is the smallest valid submission;
	 * shorter units wedge xaga firmware instead of failing cleanly. The
	 * bitstream mapping is checked as well, so a session that could not
	 * allocate it can never be submitted to.
	 */
	if (!data || !c->bs.cpu || bytes < 4 || bytes > c->bs.size)
		return -EINVAL;
	memcpy(c->bs.cpu, data + p->data_offset, bytes);
	c->source_cookie = ++c->next_cookie;
	c->source_done = false;
	dma_wmb();
	{ u8 *b8 = c->bs.cpu;
	dev_info(c->dev->dev, "submit bytes=%u head=%*ph\n", bytes,
		 bytes < 16 ? bytes : 16, b8); }
	return mtk_vcp_vdec_start(c->decoder, c->source_cookie, c->bs.dma,
				 bytes, c->bs.size, src->vb2_buf.timestamp, changed);
}

/* Hands the pending OUTPUT buffer to firmware for sequence parsing and
 * publishes the geometry it describes. Returns 0 once the picture is known,
 * -EAGAIN when the buffer did not carry a complete sequence, or a negative
 * error. The access unit itself is not consumed: the decode pass submits it
 * again.
 */
static int parse_headers(struct vdec_ctx *c, struct vb2_v4l2_buffer *src)
{
	const struct v4l2_event event = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};
	u32 changed;
	int ret;

	ret = submit_source(c, src, &changed);
	if (ret) {
		dev_info(c->dev->dev, "header submit failed: %d\n", ret);
		return ret;
	}
	/* The parse pass is not the decode pass that follows it. */
	c->submitted = false;
	if (!(changed & BIT(0)))
		return -EAGAIN;
	ret = collect_events(c);
	if (!ret)
		ret = mtk_vcp_vdec_picture(c->decoder, &c->pic);
	if (!ret)
		ret = allocate_surfaces(c);
	if (ret)
		return ret;
	capture_format(c);
	c->header = true;
	c->fh.m2m_ctx->ignore_cap_streaming = false;
	v4l2_event_queue_fh(&c->fh, &event);
	dev_info(c->dev->dev, "header parsed: %ux%u dpb=%u surfaces=%u\n",
		 c->pic.width, c->pic.height, c->pic.dpb, c->pool_count);
	/* The picture geometry is known from here on. H.264 carries its frame
	 * rate in the VUI, which this frontend does not parse, so assume the
	 * panel rate: a decoder that cannot keep up with the display would drop
	 * frames anyway.
	 */
	mtk_vcp_vdec_hw_set_perf(c->dev->hw, c->pic.width, c->pic.height, 60);
	return 0;
}

static void decode_work(struct work_struct *work)
{
	struct vdec_ctx *c = container_of(work, struct vdec_ctx, work);
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src;
	unsigned long deadline;
	u32 changed;
	int ret = 0;

	if (READ_ONCE(c->stopping))
		goto finish;
	ret = session_start(c);
	if (ret)
		goto error;
	/* A resolution change ends the previous capture sequence before any frame
	 * of the new one is decoded; the marker may have been deferred until a
	 * capture buffer became available.
	 */
	if (READ_ONCE(c->last_pending)) {
		ret = finish_old_sequence(c);
		if (ret)
			goto error;
		goto finish;
	}
	if (READ_ONCE(c->wait_capture))
		goto finish;
	src = v4l2_m2m_next_src_buf(m);
	if (!c->header) {
		if (!src)
			goto finish;
		ret = parse_headers(c, src);
		if (ret == -EAGAIN) {
			/* Incomplete sequence headers are consumed before
			 * capture starts.
			 */
			src = v4l2_m2m_src_buf_remove(m);
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
			ret = 0;
		}
		if (ret)
			goto error;
		goto finish;
	}
	ret = collect_events(c);
	if (!ret)
		ret = deliver_frames(c);
	if (ret)
		goto error;
	if (src && !c->drained) {
		if (!vb2_get_plane_payload(&src->vb2_buf, 0)) {
			c->draining = true;
			src = v4l2_m2m_src_buf_remove(m);
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
			c->submitted = false;
			goto drain;
		}
		ret = queue_surfaces(c);
		if (ret)
			goto error;
		if (!c->submitted) {
			/* An earlier job may have handed this buffer to firmware
			 * before STREAMOFF interrupted the wait for its release;
			 * resubmitting it would decode the access unit twice.
			 */
			ret = submit_source(c, src, &changed);
			if (ret)
				goto error;
			c->submitted = true;
			if (changed & (BIT(2) | BIT(3))) {
				ret = -EPIPE;
				goto error;
			}
			if (changed & BIT(0)) {
				ret = res_change_restart(c, src);
				if (ret)
					goto error;
				goto finish;
			}
		}
		deadline = jiffies + msecs_to_jiffies(5000);
		while (!c->source_done) {
			int seq = atomic_read(&c->notification);

			ret = collect_events(c);
			if (!ret)
				ret = deliver_frames(c);
			if (ret)
				goto error;
			if (c->source_done)
				break;
			if (READ_ONCE(c->stopping))
				goto finish;
			if (time_after_eq(jiffies, deadline)) {
				ret = -ETIMEDOUT;
				goto error;
			}
			wait_event_timeout(c->wait, atomic_read(&c->notification) != seq ||
						   READ_ONCE(c->stopping), msecs_to_jiffies(20));
		}
		c->submitted = false;
		src = v4l2_m2m_src_buf_remove(m);
		src->sequence = c->source_sequence++;
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
	}
drain:
	if (c->draining && !c->drained && !v4l2_m2m_num_src_bufs_ready(m)) {
		ret = mtk_vcp_vdec_reset(c->decoder, true);
		if (!ret)
			ret = collect_events(c);
		if (!ret)
			ret = deliver_frames(c);
		if (ret)
			goto error;
		c->drained = true;
	}
	if (c->drained && !c->pending_count) {
		struct vb2_v4l2_buffer *dst = v4l2_m2m_dst_buf_remove(m);
		const struct v4l2_event event = { .type = V4L2_EVENT_EOS };

		if (dst) {
			vb2_set_plane_payload(&dst->vb2_buf, 0, 0);
			if (c->dst_fmt.num_planes > 1)
				vb2_set_plane_payload(&dst->vb2_buf, 1, 0);
			dst->sequence = c->sequence++;
			v4l2_m2m_last_buffer_done(m, dst);
			v4l2_event_queue_fh(&c->fh, &event);
		}
	}
	goto finish;
error:
	dev_err(c->dev->dev, "decode failed: %d\n", ret);
	WRITE_ONCE(c->failed, true);
	vb2_queue_error(v4l2_m2m_get_src_vq(m));
	vb2_queue_error(v4l2_m2m_get_dst_vq(m));
finish:
	v4l2_m2m_job_finish(c->dev->m2m, m);
}

static void device_run(void *priv)
{
	struct vdec_ctx *c = priv;

	dev_info_ratelimited(c->dev->dev,
			     "job run: header=%d src=%u dst=%u pending=%u draining=%d drained=%d\n",
			     READ_ONCE(c->header),
			     v4l2_m2m_num_src_bufs_ready(c->fh.m2m_ctx),
			     v4l2_m2m_num_dst_bufs_ready(c->fh.m2m_ctx),
			     READ_ONCE(c->pending_count),
			     READ_ONCE(c->draining), READ_ONCE(c->drained));
	queue_work(c->dev->queue, &c->work);
}
static int job_ready(void *priv)
{
	struct vdec_ctx *c = priv;
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	bool src = v4l2_m2m_num_src_bufs_ready(m), dst = v4l2_m2m_num_dst_bufs_ready(m);

	if (READ_ONCE(c->stopping) || READ_ONCE(c->failed))
		return 0;
	/* The capture sequence of the previous resolution still needs its LAST
	 * marker, which only needs a capture buffer to hand out.
	 */
	if (READ_ONCE(c->last_pending))
		return dst;
	/* A resolution change suspends decoding until the client restarts
	 * CAPTURE, as the stateful decoder protocol requires.
	 */
	if (READ_ONCE(c->wait_capture))
		return 0;
	if (!READ_ONCE(c->header))
		return src;
	/* Decoding ahead of a capture renegotiation would hand the current
	 * picture to buffers sized for the previous resolution; the header
	 * pass already told the application about the change, so wait for it
	 * to stop and restart the capture queue with matching buffers.
	 */
	if (dst && !capture_fits(c, v4l2_m2m_next_dst_buf(m)))
		return 0;
	if (READ_ONCE(c->pending_count) && dst)
		return 1;
	if (READ_ONCE(c->draining) && !READ_ONCE(c->drained) && !src)
		return 1;
	if (READ_ONCE(c->drained))
		return !READ_ONCE(c->pending_count) && dst;
	return src && dst;
}
static void job_abort(void *priv)
{
	struct vdec_ctx *c = priv;

	WRITE_ONCE(c->stopping, true);
	wake_up(&c->wait);
}
static const struct v4l2_m2m_ops m2m_ops = {
	.device_run = device_run, .job_ready = job_ready, .job_abort = job_abort,
};

static int queue_setup(struct vb2_queue *q, unsigned int *buffers,
		       unsigned int *planes, unsigned int sizes[], struct device *alloc_devs[])
{
	struct vdec_ctx *c = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *f = queue_format(c, q->type);
	unsigned int i;

	if (*planes) {
		if (*planes != f->num_planes)
			return -EINVAL;
		for (i = 0; i < *planes; i++)
			if (sizes[i] < f->plane_fmt[i].sizeimage)
				return -EINVAL;
		return 0;
	}
	*planes = f->num_planes;
	for (i = 0; i < *planes; i++)
		sizes[i] = f->plane_fmt[i].sizeimage;
	return 0;
}
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vdec_ctx *c = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *f = queue_format(c, vb->type);
	unsigned int i;

	for (i = 0; i < f->num_planes; i++)
		if (vb2_plane_size(vb, i) < f->plane_fmt[i].sizeimage)
			return -EINVAL;
	if (is_output(vb->type) &&
	    (vb->planes[0].data_offset > vb2_get_plane_payload(vb, 0) ||
	     vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)))
		return -EINVAL;
	return 0;
}
static void buffer_queue(struct vb2_buffer *vb)
{
	struct vdec_ctx *c = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(c->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

/* LAST sets the mem2mem has_stopped flag, which suppresses all further
 * scheduling. A completed drain additionally leaves the firmware waiting for a
 * restart, while a resolution change already rebuilt the session, so only the
 * mem2mem state is cleared in that case.
 */
static int resume_streaming(struct vdec_ctx *c)
{
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	int ret;

	if (!v4l2_m2m_has_stopped(m))
		return 0;
	if (READ_ONCE(c->drained) && c->decoder) {
		/* LAST disables scheduling; join the worker before resetting. */
		flush_work(&c->work);
		ret = mtk_vcp_vdec_reset(c->decoder, false);
		if (ret)
			return ret;
		ret = collect_events(c);
		if (ret)
			return ret;
	}
	/* A stopped queue owns no more buffers; the last one was already
	 * dequeued or is being discarded by the queue restart.
	 */
	vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(m));
	return 0;
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vdec_ctx *c = vb2_get_drv_priv(q);
	int ret;

	if (c->orphan)
		return -EIO;
	WRITE_ONCE(c->stopping, false);
	WRITE_ONCE(c->failed, false);
	if (is_output(q->type)) {
		c->draining = false;
		c->drained = false;
		c->submitted = false;
		c->wait_capture = false;
		v4l2_m2m_clear_state(c->fh.m2m_ctx);
		return 0;
	}
	/* Restarting CAPTURE is how a client resumes after a drain or after a
	 * resolution change. Clear the software and firmware drain state so the
	 * queued OUTPUT buffers are scheduled again.
	 */
	c->wait_capture = false;
	ret = resume_streaming(c);
	if (ret)
		return ret;
	c->draining = false;
	c->drained = false;
	v4l2_m2m_clear_state(c->fh.m2m_ctx);
	return 0;
}
static void stop_streaming(struct vb2_queue *q)
{
	struct vdec_ctx *c = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;

	WRITE_ONCE(c->stopping, true);
	wake_up(&c->wait);
	flush_work(&c->work);
	if (!is_output(q->type)) {
		/* Capture re-setup (e.g. after SOURCE_CHANGE) keeps the
		 * firmware session alive; output STREAMOFF or release ends it.
		 */
		while ((vb = v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx)))
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	session_stop(c);
	while ((vb = v4l2_m2m_src_buf_remove(c->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	while ((vb = v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	c->header = false;
	c->fh.m2m_ctx->ignore_cap_streaming = true;
}
static const struct vb2_ops queue_ops = {
	.queue_setup = queue_setup, .buf_prepare = buffer_prepare, .buf_queue = buffer_queue,
	.start_streaming = start_streaming, .stop_streaming = stop_streaming,
};
static int queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct vdec_ctx *c = priv;
	struct vb2_queue *q;
	int i, ret;

	for (i = 0; i < 2; i++) {
		q = i ? dst : src;
		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->io_modes = VB2_MMAP;
		q->drv_priv = c;
		q->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		q->ops = &queue_ops;
		q->mem_ops = &vb2_vmalloc_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &c->dev->lock;
		q->dev = c->dev->dev;
		q->allow_zero_bytesused = 1;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

static int querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "mtk-vcp-dec", sizeof(cap->driver));
	strscpy(cap->card, "MT6895 VCP H.264 decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:mt6895-vcp-dec", sizeof(cap->bus_info));
	return 0;
}
static int enum_format(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (!valid_type(f->type))
		return -EINVAL;
	if (is_output(f->type)) {
		if (f->index)
			return -EINVAL;
		f->pixelformat = V4L2_PIX_FMT_H264;
		return 0;
	}
	if (f->index == 0)
		f->pixelformat = V4L2_PIX_FMT_NV12M;
	else if (f->index == 1)
		f->pixelformat = V4L2_PIX_FMT_NV12;
	else
		return -EINVAL;
	return 0;
}
static int get_format(struct file *file, void *priv, struct v4l2_format *f)
{
	if (!valid_type(f->type))
		return -EINVAL;
	f->fmt.pix_mp = *queue_format(file_ctx(file), f->type);
	return 0;
}
static int try_format(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vdec_ctx *c = file_ctx(file);
	struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;
	u32 size = p->plane_fmt[0].sizeimage;

	if (!valid_type(f->type))
		return -EINVAL;
	if (!is_output(f->type) && c->header) {
		/* The geometry parsed by firmware is fixed, but a client that
		 * negotiates CAPTURE after the first SOURCE_CHANGE may still pick
		 * either of the supported layouts for it.
		 */
		picture_format(c, p);
		return 0;
	}
	p->width = ALIGN(clamp_t(u32, p->width, 16, 4096), 16);
	p->height = ALIGN(clamp_t(u32, p->height, 32, 2176), 32);
	p->field = V4L2_FIELD_NONE;
	memset(p->plane_fmt, 0, sizeof(p->plane_fmt));
	memset(p->reserved, 0, sizeof(p->reserved));
	if (is_output(f->type)) {
		p->pixelformat = V4L2_PIX_FMT_H264;
		p->num_planes = 1;
		p->plane_fmt[0].sizeimage = clamp_t(u32, size ?: SZ_4M, SZ_64K, SZ_16M);
	} else {
		if (p->pixelformat != V4L2_PIX_FMT_NV12M &&
		    p->pixelformat != V4L2_PIX_FMT_NV12)
			p->pixelformat = V4L2_PIX_FMT_NV12M;
		p->colorspace = c->src_fmt.colorspace;
		p->xfer_func = c->src_fmt.xfer_func;
		p->ycbcr_enc = c->src_fmt.ycbcr_enc;
		p->quantization = c->src_fmt.quantization;
		if (p->pixelformat == V4L2_PIX_FMT_NV12) {
			p->num_planes = 1;
			p->plane_fmt[0].bytesperline = p->width;
			p->plane_fmt[0].sizeimage = p->width * p->height * 3 / 2;
		} else {
			p->num_planes = 2;
			p->plane_fmt[0].bytesperline = p->width;
			p->plane_fmt[1].bytesperline = p->width;
			p->plane_fmt[0].sizeimage = p->width * p->height;
			p->plane_fmt[1].sizeimage = p->width * p->height / 2;
		}
	}
	return 0;
}
static int set_format(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vdec_ctx *c = file_ctx(file);
	int ret;

	if (!valid_type(f->type))
		return -EINVAL;
	if (vb2_is_busy(v4l2_m2m_get_vq(c->fh.m2m_ctx, f->type)))
		return -EBUSY;
	ret = try_format(file, priv, f);
	if (!ret) {
		*queue_format(c, f->type) = f->fmt.pix_mp;
		if (is_output(f->type)) {
			c->dst_fmt.colorspace = c->src_fmt.colorspace;
			c->dst_fmt.xfer_func = c->src_fmt.xfer_func;
			c->dst_fmt.ycbcr_enc = c->src_fmt.ycbcr_enc;
			c->dst_fmt.quantization = c->src_fmt.quantization;
		}
	}
	return ret;
}
static int get_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct vdec_ctx *c = file_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE && s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;
	s->r = (struct v4l2_rect){ .width = c->dst_fmt.width, .height = c->dst_fmt.height };
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (c->header && c->pic.crop_width && c->pic.crop_height)
			s->r = (struct v4l2_rect){ c->pic.crop_left, c->pic.crop_top,
						 c->pic.crop_width, c->pic.crop_height };
		return 0;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		return 0;
	default:
		return -EINVAL;
	}
}
static int enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *s)
{
	if (s->index || (s->pixel_format != V4L2_PIX_FMT_H264 &&
			 s->pixel_format != V4L2_PIX_FMT_NV12M &&
			 s->pixel_format != V4L2_PIX_FMT_NV12))
		return -EINVAL;
	s->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	s->stepwise = (struct v4l2_frmsize_stepwise){ 16, 4096, 16, 32, 2176, 32 };
	return 0;
}
static int subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *s)
{
	if (s->type == V4L2_EVENT_SOURCE_CHANGE)
		return v4l2_src_change_event_subscribe(fh, s);
	if (s->type == V4L2_EVENT_EOS)
		return v4l2_event_subscribe(fh, s, 2, NULL);
	return v4l2_ctrl_subscribe_event(fh, s);
}
static int decoder_cmd(struct file *file, void *priv, struct v4l2_decoder_cmd *cmd)
{
	struct vdec_ctx *c = file_ctx(file);
	int ret = v4l2_m2m_ioctl_try_decoder_cmd(file, priv, cmd);

	if (ret)
		return ret;
	if (cmd->cmd == V4L2_DEC_CMD_STOP) {
		c->draining = true;
	} else if (cmd->cmd == V4L2_DEC_CMD_START) {
		if (READ_ONCE(c->draining) && !v4l2_m2m_has_stopped(c->fh.m2m_ctx))
			return -EBUSY;
		ret = resume_streaming(c);
		if (ret)
			return ret;
		c->draining = false;
		c->drained = false;
		c->wait_capture = false;
		v4l2_m2m_clear_state(c->fh.m2m_ctx);
	} else {
		return -EINVAL;
	}
	v4l2_m2m_try_schedule(c->fh.m2m_ctx);
	return 0;
}
static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = querycap,
	.vidioc_enum_fmt_vid_cap = enum_format, .vidioc_enum_fmt_vid_out = enum_format,
	.vidioc_g_fmt_vid_cap_mplane = get_format, .vidioc_g_fmt_vid_out_mplane = get_format,
	.vidioc_try_fmt_vid_cap_mplane = try_format, .vidioc_try_fmt_vid_out_mplane = try_format,
	.vidioc_s_fmt_vid_cap_mplane = set_format, .vidioc_s_fmt_vid_out_mplane = set_format,
	.vidioc_enum_framesizes = enum_framesizes, .vidioc_g_selection = get_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs, .vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf, .vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf, .vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon, .vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_decoder_cmd = decoder_cmd, .vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_subscribe_event = subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int vdec_open(struct file *file)
{
	struct vdec_dev *d = video_drvdata(file);
	struct vdec_ctx *c;
	struct v4l2_format f = {};
	struct v4l2_ctrl *ctrl;
	int ret;

	if (mutex_lock_interruptible(&d->lock))
		return -ERESTARTSYS;
	c = kzalloc_obj(*c);
	if (!c) {
		ret = -ENOMEM;
		goto unlock;
	}
	c->dev = d;
	c->next_cookie = 0x1000;
	INIT_WORK(&c->work, decode_work);
	init_waitqueue_head(&c->wait);
	atomic_set(&c->notification, 0);
	v4l2_fh_init(&c->fh, &d->video);
	file->private_data = &c->fh;
	v4l2_ctrl_handler_init(&c->controls, 1);
	ctrl = v4l2_ctrl_new_std(&c->controls, NULL, V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 64, 1, 2);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (c->controls.error) {
		ret = c->controls.error;
		goto free;
	}
	c->fh.ctrl_handler = &c->controls;
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f.fmt.pix_mp.width = 1920;
	f.fmt.pix_mp.height = 1088;
	try_format(file, NULL, &f);
	c->src_fmt = f.fmt.pix_mp;
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	try_format(file, NULL, &f);
	c->dst_fmt = f.fmt.pix_mp;
	c->fh.m2m_ctx = v4l2_m2m_ctx_init(d->m2m, c, queue_init);
	if (IS_ERR(c->fh.m2m_ctx)) {
		ret = PTR_ERR(c->fh.m2m_ctx);
		goto free;
	}
	c->fh.m2m_ctx->ignore_cap_streaming = true;
	v4l2_m2m_set_src_buffered(c->fh.m2m_ctx, true);
	v4l2_m2m_set_dst_buffered(c->fh.m2m_ctx, true);
	v4l2_fh_add(&c->fh, file);
	mutex_unlock(&d->lock);
	return 0;
free:
	v4l2_ctrl_handler_free(&c->controls);
	v4l2_fh_exit(&c->fh);
	kfree(c);
unlock:
	mutex_unlock(&d->lock);
	return ret;
}
static int vdec_release(struct file *file)
{
	struct vdec_ctx *c = file_ctx(file);
	struct vdec_dev *d = c->dev;

	mutex_lock(&d->lock);
	WRITE_ONCE(c->stopping, true);
	wake_up(&c->wait);
	v4l2_m2m_ctx_release(c->fh.m2m_ctx);
	flush_work(&c->work);
	session_stop(c);
	v4l2_ctrl_handler_free(&c->controls);
	v4l2_fh_del(&c->fh, file);
	v4l2_fh_exit(&c->fh);
	if (!c->orphan)
		kfree(c);
	mutex_unlock(&d->lock);
	return 0;
}
static const struct v4l2_file_operations file_ops = {
	.owner = THIS_MODULE, .open = vdec_open, .release = vdec_release,
	.poll = v4l2_m2m_fop_poll, .mmap = v4l2_m2m_fop_mmap, .unlocked_ioctl = video_ioctl2,
};

static void put_device_action(void *data)
{
	put_device(data);
}
static struct device *get_dma_device(struct device *dev, const char *property)
{
	struct device_node *node = of_parse_phandle(dev->of_node, property, 0);
	struct platform_device *p;
	int ret;

	if (!node)
		return ERR_PTR(-EINVAL);
	p = of_find_device_by_node(node);
	of_node_put(node);
	if (!p)
		return ERR_PTR(-EPROBE_DEFER);
	ret = devm_add_action_or_reset(dev, put_device_action, &p->dev);
	if (ret)
		return ERR_PTR(ret);
	if (!device_is_bound(&p->dev))
		return ERR_PTR(-EPROBE_DEFER);
	return &p->dev;
}
static void put_vcp(void *data)
{
	mtk_vcp_put(data);
}
static int vdec_probe(struct platform_device *pdev)
{
	struct vdec_dev *d;
	int ret;

	d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->dev = &pdev->dev;
	mutex_init(&d->lock);
	d->bs_dev = get_dma_device(d->dev, "mediatek,vcp-vdec-dma");
	if (IS_ERR(d->bs_dev))
		return PTR_ERR(d->bs_dev);
	d->ube_dev = get_dma_device(d->dev, "mediatek,vdec-ube-dma");
	if (IS_ERR(d->ube_dev))
		return PTR_ERR(d->ube_dev);
	d->vcp = mtk_vcp_get(d->dev);
	if (IS_ERR(d->vcp))
		return PTR_ERR(d->vcp);
	ret = devm_add_action_or_reset(d->dev, put_vcp, d->vcp);
	if (ret)
		return ret;
	d->hw = mtk_vcp_vdec_hw_create(pdev, d->vcp, d->ube_dev);
	if (IS_ERR(d->hw))
		return PTR_ERR(d->hw);
	d->queue = alloc_ordered_workqueue("mtk-vcp-vdec", WQ_MEM_RECLAIM | WQ_FREEZABLE);
	if (!d->queue)
		return -ENOMEM;
	ret = v4l2_device_register(d->dev, &d->v4l2);
	if (ret)
		goto work;
	d->m2m = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(d->m2m)) {
		ret = PTR_ERR(d->m2m);
		goto v4l2;
	}
	strscpy(d->video.name, "mtk-vcp-dec", sizeof(d->video.name));
	d->video.fops = &file_ops;
	d->video.ioctl_ops = &ioctl_ops;
	d->video.release = video_device_release_empty;
	d->video.v4l2_dev = &d->v4l2;
	d->video.lock = &d->lock;
	d->video.vfl_dir = VFL_DIR_M2M;
	d->video.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&d->video, d);
	platform_set_drvdata(pdev, d);
	ret = video_register_device(&d->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto m2m;
	dev_info(d->dev, "H.264 decoder registered as /dev/video%d\n", d->video.num);
	return 0;
m2m:
	v4l2_m2m_release(d->m2m);
v4l2:
	v4l2_device_unregister(&d->v4l2);
work:
	destroy_workqueue(d->queue);
	return ret;
}
static void vdec_remove(struct platform_device *pdev)
{
	struct vdec_dev *d = platform_get_drvdata(pdev);

	video_unregister_device(&d->video);
	v4l2_m2m_release(d->m2m);
	v4l2_device_unregister(&d->v4l2);
	destroy_workqueue(d->queue);
}
static const struct of_device_id vdec_match[] = {
	{ .compatible = "mediatek,mt6895-vcodec-dec" }, {}
};
MODULE_DEVICE_TABLE(of, vdec_match);
static int vdec_suspend(struct device *dev)
{
	struct vdec_dev *d = dev_get_drvdata(dev);

	/* Firmware sessions and retained DMA cannot survive system suspend. */
	return READ_ONCE(d->ctx) ? -EBUSY : 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(vdec_pm_ops, vdec_suspend, NULL);
static struct platform_driver vdec_driver = {
	.probe = vdec_probe, .remove = vdec_remove,
	.driver = { .name = "mtk-vcp-dec", .of_match_table = vdec_match,
		    .suppress_bind_attrs = true, .pm = pm_sleep_ptr(&vdec_pm_ops) },
};
static int dma_probe(struct platform_device *pdev)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(&pdev->dev);
	bool ube = of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6895-vdec-ube-dma");

	if (!domain || !domain->geometry.force_aperture ||
	    domain->geometry.aperture_start != (ube ? 0x20000000ULL : 0x110000000ULL) ||
	    domain->geometry.aperture_end != (ube ? 0x32bfffffULL : 0x12fffffffULL))
		return -EINVAL;
	return dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
}
static const struct of_device_id dma_match[] = {
	{ .compatible = "mediatek,mt6895-vcp-vdec-dma" },
	{ .compatible = "mediatek,mt6895-vdec-ube-dma" }, {}
};
MODULE_DEVICE_TABLE(of, dma_match);
static struct platform_driver dma_driver = {
	.probe = dma_probe,
	.driver = {
		.name = "mtk-vcp-vdec-dma",
		.of_match_table = dma_match,
		.suppress_bind_attrs = true,
	},
};
static int __init vdec_init(void)
{
	int ret = platform_driver_register(&dma_driver);

	if (ret)
		return ret;
	ret = platform_driver_register(&vdec_driver);
	if (ret)
		platform_driver_unregister(&dma_driver);
	return ret;
}
static void __exit vdec_exit(void)
{
	platform_driver_unregister(&vdec_driver);
	platform_driver_unregister(&dma_driver);
}
module_init(vdec_init);
module_exit(vdec_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6895 VCP stateful H.264 V4L2 decoder");
