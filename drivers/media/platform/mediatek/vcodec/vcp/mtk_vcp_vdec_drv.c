// SPDX-License-Identifier: GPL-2.0-only
/* Stateful V4L2 decoding through the MT6895 VCP firmware. */
#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/ktime.h>
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
#include <media/videobuf2-dma-sg.h>
#include "mtk_vcp_vdec_hw.h"
#include "mtk_vcp_vdec_bitstream.h"

#define DEC_SURFACES 36

static bool vdec_caps_dump;
module_param_named(caps_dump, vdec_caps_dump, bool, 0644);
MODULE_PARM_DESC(caps_dump, "dump the firmware decoder capability tables on session boot");

/* VP8 uses synchronous bitstream/frame pairing; VP9 chroma follows luma
 * in the shared surface allocation. Both use the verified MM21 layout.
 * HEIF remains unadvertised until a working firmware output is verified.
 */
struct vdec_codec {
	u32 fourcc;
	u32 vcp_fourcc;
	u32 codec_id;
	struct v4l2_frmsize_stepwise size;
};

static const struct vdec_codec vdec_codecs[] = {
	{ V4L2_PIX_FMT_H264, v4l2_fourcc('H', '2', '6', '4'), VCP_VDEC_H264,
	  { 16, 4096, 16, 16, 2176, 16 } },
	{ V4L2_PIX_FMT_HEVC, v4l2_fourcc('H', '2', '6', '5'), VCP_VDEC_H265,
	  { 16, 4096, 16, 16, 2176, 16 } },
	{ V4L2_PIX_FMT_VP9, v4l2_fourcc('V', 'P', '9', '0'), VCP_VDEC_VP9,
	  { 16, 4096, 16, 16, 2176, 16 } },
	{ V4L2_PIX_FMT_VP8, v4l2_fourcc('V', 'P', '8', '0'), VCP_VDEC_VP8,
	  { 16, 2048, 16, 16, 1088, 32 } },
	{ V4L2_PIX_FMT_MPEG2, v4l2_fourcc('M', 'P', 'G', '2'), VCP_VDEC_MPEG12,
	  { 16, 2048, 16, 16, 1088, 32 } },
	{ V4L2_PIX_FMT_MPEG4, v4l2_fourcc('M', 'P', 'G', '4'), VCP_VDEC_MPEG4,
	  { 16, 2048, 16, 16, 1088, 32 } },
	{ V4L2_PIX_FMT_H263, v4l2_fourcc('H', '2', '6', '3'), VCP_VDEC_H263,
	  { 16, 1408, 16, 16, 1152, 32 } },
	/* Upstream V4L2 spells AV1 'AV01' while the firmware table spells it
	 * 'AV10'; the vcp_fourcc column carries the firmware spelling.
	 */
	{ V4L2_PIX_FMT_AV1, v4l2_fourcc('A', 'V', '1', '0'), VCP_VDEC_AV1,
	  { 16, 4096, 16, 16, 2176, 32 } },
};

static const struct vdec_codec *vdec_codec_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vdec_codecs); i++)
		if (vdec_codecs[i].fourcc == fourcc)
			return &vdec_codecs[i];
	return NULL;
}

/* CAPTURE geometry follows whatever the decoded stream turns out to be, so it
 * accepts the union of the ranges the supported codecs publish.
 */
static const struct v4l2_frmsize_stepwise vdec_capture_size = {
	16, 4096, 16, 16, 2176, 16,
};

/* Round a requested dimension down to a step the firmware accepts, inside the
 * range the codec publishes.
 */
static u32 vdec_dimension(u32 v, u32 min, u32 max, u32 step)
{
	v = clamp_t(u32, v, min, max);
	return v / step * step;
}

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
	/* plane[0] owns the entire allocation; plane[1] is a chroma view. */
	struct mtk_vcp_mem plane[2];
	u64 cookie;
	bool free, pending;
};
struct vdec_pending {
	u32 surface;
	u64 timestamp;
	u64 fw_done_ns;
};
/* perf_frames arms ktime staging splits for the next N delivered frames
 * (0 = off). Re-arm by writing the count again through sysfs; each write
 * resets the consumed counter. Zero overhead beyond one atomic when off.
 */
static int perf_frames;
static atomic_t perf_used = ATOMIC_INIT(0);
static int perf_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (!ret)
		atomic_set(&perf_used, 0);
	return ret;
}
static const struct kernel_param_ops perf_ops = {
	.set = perf_set, .get = param_get_int,
};
module_param_cb(perf_frames, &perf_ops, &perf_frames, 0644);
MODULE_PARM_DESC(perf_frames, "trace ktime splits for the next N delivered frames");
struct vdec_ctx {
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler controls;
	struct vdec_dev *dev;
	struct mtk_vcp_vdec *decoder;
	struct v4l2_pix_format_mplane src_fmt, dst_fmt;
	u32 codec_id;
	struct vcp_vdec_picture pic;
	struct mtk_vcp_mem bs;
	struct vdec_surface surfaces[DEC_SURFACES];
	struct vdec_pending pending[DEC_SURFACES];
	u32 pool_count, pending_read, pending_count, sequence, source_sequence;
	u64 next_cookie, source_cookie;
	struct work_struct work;
	wait_queue_head_t wait;
	atomic_t notification;
	bool booted, initialized, header, stopping, failed, orphan, released;
	bool source_done, draining, drained;
	bool submitted;   /* head OUTPUT buffer handed to firmware, release pending */
	bool last_pending; /* previous capture sequence still needs its LAST marker */
	bool wait_capture; /* new sequence waits for the client to restart CAPTURE */
	u32 prev_dst_size[2], prev_dst_planes; /* CAPTURE geometry before the change */
};

static struct vdec_ctx *file_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct vdec_ctx, fh);
}

/*
 * DEBUG: temporary instrumentation for the decoder investigations (the two
 * resolution-change hang paths and the session that reports no frames with
 * the VCP left offline).  Everything from here down to the end of
 * vdec_state() is debug-only, as are every VCPDBG() line and vdec_state()
 * call in this file.  Remove all of it before the series is submitted.
 */
#define VCPDBG(fmt, ...) pr_info("VCPDBG:%s: " fmt, __func__, ##__VA_ARGS__)

static void vdec_state(struct vdec_ctx *c, const char *tag)
{
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;

	if (!m) {
		VCPDBG("state %s: no m2m context\n", tag);
		return;
	}
	VCPDBG("state %s: hdr=%d boot=%d init=%d stop=%d fail=%d orph=%d done=%d drn=%d drnd=%d sub=%d lastp=%d waitcap=%d pool=%u pend=%u rd=%u seq=%u sseq=%u srcq=%u dstq=%u stopd=%d dst=%ux%u pic=%ux%u stride=%u/%u\n",
	       tag, c->header, c->booted, c->initialized, c->stopping,
	       c->failed, c->orphan, c->source_done, c->draining, c->drained,
	       c->submitted, c->last_pending, c->wait_capture, c->pool_count,
	       c->pending_count, c->pending_read, c->sequence, c->source_sequence,
	       v4l2_m2m_num_src_bufs_ready(m), v4l2_m2m_num_dst_bufs_ready(m),
	       v4l2_m2m_has_stopped(m), c->dst_fmt.width, c->dst_fmt.height,
	       c->pic.width, c->pic.height, c->pic.stride, c->pic.buffer_height);
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

/* Ask the firmware which formats it can decode and refuse a session whose
 * negotiated format is not among them. A stream the firmware cannot handle
 * makes it stop answering and resets the VCP core, so the mismatch has to
 * fail the session here instead of reaching the decoder.
 */
static int vdec_check_caps(struct vdec_ctx *c)
{
	const struct vdec_codec *k = vdec_codec_by_fourcc(c->src_fmt.pixelformat);
	struct vcp_vdec_cap_framesize *sizes;
	struct vcp_vdec_cap_format *fmts;
	bool found = false;
	int ret, i;

	if (!k)
		return -EINVAL;
	fmts = kzalloc(sizeof(*fmts) * VCP_VDEC_CAPS, GFP_KERNEL);
	sizes = kzalloc(sizeof(*sizes) * VCP_VDEC_CAPS, GFP_KERNEL);
	if (!fmts || !sizes) {
		ret = -ENOMEM;
		goto out;
	}
	ret = mtk_vcp_vdec_query_cap(c->decoder, VCP_VDEC_CAP_SUPPORTED_FORMATS,
				     fmts, sizeof(*fmts) * VCP_VDEC_CAPS);
	if (ret) {
		/* The query is advisory: keep a session the firmware cannot
		 * describe, rather than refusing to decode at all.
		 */
		dev_warn(c->dev->dev, "VDEC format query failed: %d\n", ret);
		ret = 0;
		goto out;
	}
	for (i = 0; i < VCP_VDEC_CAPS && le32_to_cpu(fmts[i].fourcc); i++) {
		u32 fourcc = le32_to_cpu(fmts[i].fourcc);

		if (vdec_caps_dump)
			dev_info(c->dev->dev,
				 "VDEC cap fmt[%d]: fourcc=%p4cc type=%u planes=%u\n", i,
				 &fourcc, le32_to_cpu(fmts[i].type),
				 le32_to_cpu(fmts[i].num_planes));
		if (fourcc == k->vcp_fourcc)
			found = true;
	}
	if (!found) {
		dev_err(c->dev->dev,
			"VDEC format %p4cc is not in the firmware format table\n",
			&k->fourcc);
		ret = -EINVAL;
		goto out;
	}
	ret = mtk_vcp_vdec_query_cap(c->decoder, VCP_VDEC_CAP_FRAME_SIZES, sizes,
				     sizeof(*sizes) * VCP_VDEC_CAPS);
	if (ret) {
		dev_warn(c->dev->dev, "VDEC frame size query failed: %d\n", ret);
		ret = 0;
		goto out;
	}
	for (i = 0; i < VCP_VDEC_CAPS && vdec_caps_dump &&
	     le32_to_cpu(sizes[i].fourcc); i++)
		dev_info(c->dev->dev,
			 "VDEC cap size[%d]: fourcc=%p4cc profile=%u level=%u %ux%u..%ux%u step %ux%u\n",
			 i, &sizes[i].fourcc, le32_to_cpu(sizes[i].profile),
			 le32_to_cpu(sizes[i].level),
			 le32_to_cpu(sizes[i].stepwise.min_width),
			 le32_to_cpu(sizes[i].stepwise.min_height),
			 le32_to_cpu(sizes[i].stepwise.max_width),
			 le32_to_cpu(sizes[i].stepwise.max_height),
			 le32_to_cpu(sizes[i].stepwise.step_width),
			 le32_to_cpu(sizes[i].stepwise.step_height));
out:
	kfree(fmts);
	kfree(sizes);
	return ret;
}

static int session_claim(struct vdec_ctx *c)
{
	struct vdec_ctx *owner;
	int ret;

	ret = mtk_vcp_claim(c->dev->vcp, c);
	if (ret)
		return ret;
	owner = cmpxchg(&c->dev->ctx, NULL, c);
	if (owner && owner != c) {
		mtk_vcp_release(c->dev->vcp, c);
		return -EBUSY;
	}
	return 0;
}

static void session_release(struct vdec_ctx *c)
{
	if (cmpxchg(&c->dev->ctx, c, NULL) == c)
		mtk_vcp_release(c->dev->vcp, c);
}

static int session_boot(struct vdec_ctx *c)
{
	int ret;

	/* Device ownership is already held by the caller. */
	if (c->decoder)
		return 0;
	VCPDBG("boot: creating session, vcp offline=%d\n",
	       mtk_vcp_is_offline(c->dev->vcp));
	c->decoder = mtk_vcp_vdec_create(c->dev->dev, c->dev->vcp, &codec_ops, c);
	if (IS_ERR(c->decoder)) {
		ret = PTR_ERR(c->decoder);
		c->decoder = NULL;
		session_release(c);
		VCPDBG("boot: decoder create failed: %d\n", ret);
		return ret;
	}
	VCPDBG("boot: decoder created\n");
	ret = mtk_vcp_boot(c->dev->vcp);
	if (ret) {
		dev_info(c->dev->dev, "session boot failed: %d\n", ret);
		VCPDBG("boot: vcp boot failed: %d, offline=%d\n", ret,
		       mtk_vcp_is_offline(c->dev->vcp));
		goto rollback;
	}
	c->booted = true;
	VCPDBG("boot: vcp running, offline=%d\n",
	       mtk_vcp_is_offline(c->dev->vcp));
	/* The firmware probes the codec of the session during INIT, so the id
	 * the CHECK_CODEC_ID handshake expects has to be set first.
	 */
	ret = mtk_vcp_vdec_set_codec(c->decoder, c->codec_id);
	if (ret) {
		dev_info(c->dev->dev, "session codec %#x rejected: %d\n",
			 c->codec_id, ret);
		goto rollback;
	}
	ret = mtk_vcp_vdec_init(c->decoder);
	if (ret) {
		dev_info(c->dev->dev, "session init failed: %d\n", ret);
		VCPDBG("boot: vdec init failed: %d\n", ret);
		goto rollback;
	}
	c->initialized = true;
	ret = vdec_check_caps(c);
	if (ret)
		goto rollback;
	c->bs.size = c->src_fmt.plane_fmt[0].sizeimage;
	c->bs.cpu = dma_alloc_coherent(c->dev->bs_dev, c->bs.size, &c->bs.dma, GFP_KERNEL);
	VCPDBG("boot: bitstream mapping size=%zu cpu=%px dma=%pad\n",
	       c->bs.size, c->bs.cpu, &c->bs.dma);
	if (c->bs.cpu) {
		vdec_state(c, "booted");
		return 0;
	}
	ret = -ENOMEM;
rollback:
	/* A session without a usable bitstream mapping, or with a format the
	 * firmware does not decode, must not be left half initialized: a later
	 * CAPTURE restart would otherwise reuse it.
	 */
	{
		int err = session_teardown(c);

		if (err)
			return err;
		session_release(c);
	}
	VCPDBG("boot: rolled back, ret=%d\n", ret);
	return ret;
}

static int session_start(struct vdec_ctx *c)
{
	if (c->orphan || READ_ONCE(c->failed))
		return -EIO;
	/* A retained session is only usable once firmware and bitstream DMA
	 * both exist.
	 */
	if (c->decoder) {
		VCPDBG("start: retained session init=%d bs=%px\n",
		       c->initialized, c->bs.cpu);
		return c->initialized && c->bs.cpu ? 0 : -EIO;
	}
	/* OUTPUT normally reserved the engine; keep the worker check idempotent. */
	{
		int ret = session_claim(c);

		if (ret)
			return ret;
	}
	return session_boot(c);
}

/* Tears down firmware, VCP, hardware and DMA, but keeps device ownership
 * so the caller can immediately boot a replacement session.
 */
static int session_teardown(struct vdec_ctx *c)
{
	int ret = 0, stopped, i;

	if (!c->decoder)
		return 0;
	VCPDBG("teardown: init=%d boot=%d orphan=%d\n", c->initialized,
	       c->booted, c->orphan);
	/* Before sequence parsing, service2 has no frame queue. Sending its
	 * null FRAME_BUFFER flush at that point crashes xaga firmware. Stop
	 * the VCP and use the reset cleanup path for such partial sessions.
	 */
	if (c->initialized && c->booted && c->header)
		ret = mtk_vcp_vdec_deinit(c->decoder);
	else
		ret = -EIO;
	if (!ret)
		c->initialized = false;
	stopped = c->booted ? mtk_vcp_shutdown(c->dev->vcp) : 0;
	/* A successful shutdown consumes our reference even if later hardware
	 * cleanup fails, or another codec keeps the VCP online.
	 */
	if (!stopped)
		c->booted = false;
	if (stopped || (ret && !mtk_vcp_is_offline(c->dev->vcp)) ||
	    mtk_vcp_vdec_hw_stop(c->dev->hw)) {
		VCPDBG("teardown: retain (deinit=%d shutdown=%d offline=%d)\n",
		       ret, stopped, mtk_vcp_is_offline(c->dev->vcp));
		goto retain;
	}
	stopped = mtk_vcp_vdec_destroy(c->decoder, !!ret);
	if (stopped) {
		VCPDBG("teardown: destroy failed: %d\n", stopped);
		goto retain;
	}
	c->decoder = NULL;
	c->booted = false;
	c->initialized = false;
	if (c->bs.cpu)
		dma_free_coherent(c->dev->bs_dev, c->bs.size, c->bs.cpu, c->bs.dma);
	memset(&c->bs, 0, sizeof(c->bs));
	for (i = 0; i < DEC_SURFACES; i++)
		if (c->surfaces[i].plane[0].cpu)
			codec_free(c, 1, &c->surfaces[i].plane[0]);
	memset(c->surfaces, 0, sizeof(c->surfaces));
	c->pool_count = 0;
	c->pending_count = 0;
	c->pending_read = 0;
	if (c->orphan) {
		c->orphan = false;
		module_put(THIS_MODULE);
	}
	VCPDBG("teardown: complete\n");
	return 0;
retain:
	WRITE_ONCE(c->failed, true);
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

	VCPDBG("stop: teardown=%d\n", ret);
	if (ret)
		return ret;
	session_release(c);
	return 0;
}

/* The file is gone and its worker has finished. Retry only the recorded
 * cleanup; teardown still requires confirmed firmware and hardware stop.
 * Called under the video device mutex before creating another file context.
 */
static int recover_released_session(struct vdec_dev *d)
{
	struct vdec_ctx *c = READ_ONCE(d->ctx);
	int ret;

	if (!c || !c->released)
		return 0;
	ret = session_stop(c);
	if (ret)
		return ret;
	kfree(c);
	return 0;
}

static int collect_events(struct vdec_ctx *c)
{
	struct vcp_vdec_event event;
	int ret;
	unsigned int i;

	while (!(ret = mtk_vcp_vdec_event(c->decoder, &event))) {
		VCPDBG("event: type=%d cookie=%#llx source_cookie=%#llx\n",
		       event.type, event.cookie, c->source_cookie);
		if (event.type == VCP_VDEC_FREE_BITSTREAM) {
			if (event.cookie != c->source_cookie) {
				VCPDBG("event: bitstream cookie mismatch\n");
				return -EPROTO;
			}
			c->source_done = true;
			continue;
		}
		for (i = 0; i < c->pool_count; i++)
			if (c->surfaces[i].cookie == event.cookie)
				break;
		if (i == c->pool_count) {
			VCPDBG("event: unknown cookie, pool=%u\n", c->pool_count);
			return -EPROTO;
		}
		if (event.type == VCP_VDEC_FREE_FRAME) {
			c->surfaces[i].free = true;
			continue;
		}
		if (c->pending_count == DEC_SURFACES || c->surfaces[i].pending) {
			VCPDBG("event: overflow, pending=%u surface=%u\n",
			       c->pending_count, i);
			return -EOVERFLOW;
		}
		/* DISPLAY carries the timestamp of this picture, after firmware
		 * reordering. A submission FIFO attaches a future reference's PTS
		 * to a B picture. Preserve zero, duplicates and discontinuities:
		 * timestamps are caller metadata, not a sort key or a clock.
		 */
		u64 timestamp = event.timestamp;

		c->surfaces[i].pending = true;
		c->pending[(c->pending_read + c->pending_count++) % DEC_SURFACES] =
			(struct vdec_pending){ .surface = i, .timestamp = timestamp,
						.fw_done_ns = ktime_get_ns() };
		VCPDBG("event: display surface=%u pending=%u ts=%llu\n", i,
		       c->pending_count, timestamp);
	}
	if (ret != -EAGAIN)
		VCPDBG("events: stopped with %d\n", ret);
	return ret == -EAGAIN ? 0 : ret;
}

/* MM21: 16x32 luma tiles and 16x16 interleaved chroma tiles in raster order.
 * MT2T luma: the same 16x32 grid at 10 bits per sample. Each 16x4 slab
 * stores 16 LSB bytes then 64 MSB bytes; LSB byte k packs samples
 * four vertical samples at bits 1:0 through 7:6; MSBs are row-major.
 * Thus MSB index is row * 16 + col, but LSB pair is col * 4 + row.
 * MT2T chroma: U and V share one grid of 8x4 row-major groups. Group
 * (gx, gy) covers columns 8 * (gx / 4)..+7 and rows 16 * gy + 4 * (gx % 4)
 * ..+3 with the same geometry for U and V. Each 80-byte group stores 8
 * U LSB bytes interleaved with 8 V LSB bytes ([U0 V0 U1 V1 ...]), then
 * 32 U MSB bytes interleaved with 32 V MSB bytes; even bytes belong to U,
 * odd bytes to V. Cell (i, j) is MSB index j * 8 + i and LSB pair
 * k = i * 4 + j of its own plane (MSBs remain row-major).
 */
static void detile(void *destination, const void *source, u32 stride, u32 height, u32 tile_h)
{
	u32 x, y;

	for (y = 0; y < height; y++)
		for (x = 0; x < stride; x += 16) {
			u32 offset = (y / tile_h * (stride / 16) + x / 16) * tile_h * 16;

			memcpy(destination + y * stride + x, source + offset + y % tile_h * 16, 16);
		}
}

static void detile_10_chroma(__le16 *dst_uv, const u8 *source, u32 stride,
			       u32 buffer_height)
{
	u32 wc = stride / 2, hc = buffer_height / 2;
	u32 groups_per_row = wc / 2, slabs = hc / 16;
	u32 s, gx;

	for (s = 0; s < slabs; s++) {
		for (gx = 0; gx < groups_per_row; gx += 4) {
			u32 c0 = (gx / 4) * 8, r0 = s * 16;
			unsigned int p, j, i;

			if (c0 + 8 > wc || r0 + 16 > hc)
				continue;
			for (p = 0; p < 4; p++) {
				const u8 *group =
					source + (s * groups_per_row + gx + p) * 80;
				u32 rr = r0 + p * 4;

				for (j = 0; j < 4; j++) {
					for (i = 0; i < 8; i++) {
						unsigned int k = j * 8 + i;
						unsigned int low = i * 4 + j;
						u32 u = (u32)group[16 + 2 * k] << 2 |
							((group[2 * (low / 4)] >>
							  (2 * (low % 4))) & 3);
						u32 v = (u32)group[17 + 2 * k] << 2 |
							((group[2 * (low / 4) + 1] >>
							  (2 * (low % 4))) & 3);
						__le16 *cell = dst_uv +
							(rr + j) * stride +
							(c0 + i) * 2;

						cell[0] = cpu_to_le16(u << 6);
						cell[1] = cpu_to_le16(v << 6);
					}
				}
			}
		}
	}
}

static void detile_10_plane(__le16 *destination, const u8 *source, u32 words,
			    u32 xstep, u32 grid_w, u32 grid_h)
{
	u32 tx, ty, y;

	for (ty = 0; ty < grid_h; ty += 32) {
		for (tx = 0; tx < grid_w; tx += 16) {
			const u8 *tile = source +
				((ty / 32 * (grid_w / 16) + tx / 16) * 32 * 16 * 10 / 8);

			for (y = 0; y < 32; y += 4) {
				const u8 *lsb = tile + (y / 4 * 80);
				const u8 *msb = lsb + 16;
				unsigned int r, x;

				for (r = 0; r < 4; r++) {
					for (x = 0; x < 16; x++) {
						unsigned int k = r * 16 + x;
						u32 low = x * 4 + r;
						u32 value = (u32)msb[k] << 2 |
							((lsb[low / 4] >> (2 * (low % 4))) & 3);

						destination[(ty + y + r) * words +
							  (tx + x) * xstep] =
							cpu_to_le16(value << 6);
					}
				}
			}
		}
	}
}

/* A capture buffer can only receive the picture it was sized for. After a
 * midstream resolution change the queue still holds buffers of the previous
 * picture; writing the new one there runs past the end of the mapping.
 */
static bool capture_fits(const struct vdec_ctx *c, struct vb2_v4l2_buffer *vb)
{
	u32 luma = c->pic.size[0], chroma = c->pic.size[1];

	/* P010 output is larger than the MT2T tiles it is converted from. */
	if (c->pic.fourcc == V4L2_PIX_FMT_MT2T)
		return vb2_plane_size(&vb->vb2_buf, 0) >=
			(size_t)c->pic.stride * c->pic.buffer_height * 3;
	if (c->dst_fmt.num_planes == 1)
		return vb2_plane_size(&vb->vb2_buf, 0) >= (size_t)luma + chroma;
	return vb2_plane_size(&vb->vb2_buf, 0) >= luma &&
	       vb2_plane_size(&vb->vb2_buf, 1) >= chroma;
}

/* CAPTURE is written by the CPU detiler and read by DMA-BUF importers.
 * A persistent GPU/display mapping does not remap (or clean) each frame.
 * Transfer cache ownership around every CPU write, before publishing DONE.
 */
static void capture_sync(struct vb2_buffer *vb, bool for_cpu)
{
	unsigned int i;

	for (i = 0; i < vb->num_planes; i++) {
		struct sg_table *sgt = vb2_dma_sg_plane_desc(vb, i);

		if (for_cpu)
			dma_sync_sgtable_for_cpu(vb->vb2_queue->dev, sgt,
						DMA_BIDIRECTIONAL);
		else
			dma_sync_sgtable_for_device(vb->vb2_queue->dev, sgt,
						   DMA_BIDIRECTIONAL);
	}
}

static int deliver_frames(struct vdec_ctx *c)
{
	struct vb2_v4l2_buffer *vb;

	VCPDBG("deliver: in pending=%u dstq=%u\n", c->pending_count,
	       v4l2_m2m_num_dst_bufs_ready(c->fh.m2m_ctx));
	while (c->pending_count && (vb = v4l2_m2m_next_dst_buf(c->fh.m2m_ctx))) {
		struct vdec_pending *p = &c->pending[c->pending_read];
		struct vdec_surface *s = &c->surfaces[p->surface];

		v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx);
		capture_sync(&vb->vb2_buf, true);
		if (c->pic.fourcc == V4L2_PIX_FMT_MT2T) {
			/* 10-bit output as standard single-plane P010. Luma
			 * detiles from plane 0; U and V share plane 1 with one
			 * 8x4 row-major group per 80 bytes and land interleaved
			 * in the P010 chroma plane.
			 */
			__le16 *base = vb2_plane_vaddr(&vb->vb2_buf, 0);
			u32 stride = c->pic.stride, bh = c->pic.buffer_height;
			size_t y_words = (size_t)stride * bh;
			size_t total = (y_words + y_words / 2) * 2;

			if (!base || vb2_plane_size(&vb->vb2_buf, 0) < total) {
				v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
				return -EFAULT;
			}
			dma_rmb();
			if (perf_frames > 0 &&
			    atomic_inc_return(&perf_used) <= perf_frames) {
				u64 t0 = ktime_get_ns(), t1, t2;

				detile_10_plane(base, s->plane[0].cpu, stride,
						1, stride, bh);
				t1 = ktime_get_ns();
				detile_10_chroma(base + y_words, s->plane[1].cpu,
						 stride, bh);
				t2 = ktime_get_ns();
				pr_info("VCPERF seq=%u fw_us=%llu y_us=%llu c_us=%llu\n",
					c->sequence, (t0 - p->fw_done_ns) / 1000,
					(t1 - t0) / 1000, (t2 - t1) / 1000);
			} else {
				detile_10_plane(base, s->plane[0].cpu, stride,
						1, stride, bh);
				detile_10_chroma(base + y_words, s->plane[1].cpu,
						 stride, bh);
			}
			vb2_set_plane_payload(&vb->vb2_buf, 0, total);
			goto delivered;
		}
		/* detile() writes a whole picture: a buffer queued for an
		 * earlier resolution only fits part of it and must be handed
		 * back instead of written past its end.
		 */
		if (!capture_fits(c, vb)) {
			VCPDBG("deliver: capture buffer too small for %ux%u\n",
			       c->pic.width, c->pic.height);
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
			if (perf_frames > 0 &&
			    atomic_inc_return(&perf_used) <= perf_frames) {
				u64 t0 = ktime_get_ns(), t1, t2;

				detile(base, s->plane[0].cpu,
				       c->pic.stride, c->pic.buffer_height, 32);
				t1 = ktime_get_ns();
				detile(base + c->pic.size[0], s->plane[1].cpu,
				       c->pic.stride, c->pic.buffer_height / 2,
				       16);
				t2 = ktime_get_ns();
				pr_info("VCPERF seq=%u fw_us=%llu y_us=%llu c_us=%llu\n",
					c->sequence, (t0 - p->fw_done_ns) / 1000,
					(t1 - t0) / 1000, (t2 - t1) / 1000);
			} else {
				detile(base, s->plane[0].cpu,
				       c->pic.stride, c->pic.buffer_height, 32);
				detile(base + c->pic.size[0], s->plane[1].cpu,
				       c->pic.stride, c->pic.buffer_height / 2,
				       16);
			}
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
			if (perf_frames > 0 &&
			    atomic_inc_return(&perf_used) <= perf_frames) {
				u64 t0 = ktime_get_ns(), t1, t2;

				detile(y, s->plane[0].cpu,
				       c->pic.stride, c->pic.buffer_height, 32);
				t1 = ktime_get_ns();
				detile(uv, s->plane[1].cpu,
				       c->pic.stride, c->pic.buffer_height / 2,
				       16);
				t2 = ktime_get_ns();
				pr_info("VCPERF seq=%u fw_us=%llu y_us=%llu c_us=%llu\n",
					c->sequence, (t0 - p->fw_done_ns) / 1000,
					(t1 - t0) / 1000, (t2 - t1) / 1000);
			} else {
				detile(y, s->plane[0].cpu,
				       c->pic.stride, c->pic.buffer_height, 32);
				detile(uv, s->plane[1].cpu,
				       c->pic.stride, c->pic.buffer_height / 2,
				       16);
			}
			vb2_set_plane_payload(&vb->vb2_buf, 0, c->pic.size[0]);
			vb2_set_plane_payload(&vb->vb2_buf, 1, c->pic.size[1]);
		}
delivered:
		capture_sync(&vb->vb2_buf, false);
		vb->vb2_buf.timestamp = p->timestamp;
		vb->field = V4L2_FIELD_NONE;
		vb->sequence = c->sequence++;
		s->pending = false;
		c->pending_read = (c->pending_read + 1) % DEC_SURFACES;
		c->pending_count--;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
		VCPDBG("deliver: frame seq=%u ts=%llu left=%u\n", vb->sequence,
		       vb->vb2_buf.timestamp, c->pending_count);
	}
	VCPDBG("deliver: out pending=%u dstq=%u\n", c->pending_count,
	       v4l2_m2m_num_dst_bufs_ready(c->fh.m2m_ctx));
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
		if (ret) {
			VCPDBG("surfaces: queue frame %u failed: %d\n", i, ret);
			return ret;
		}
		VCPDBG("surfaces: queued %u cookie=%#llx\n", i, s->cookie);
	}
	return 0;
}

static bool surfaces_idle(struct vdec_ctx *c)
{
	unsigned int i;

	if (c->pending_count)
		return false;
	for (i = 0; i < c->pool_count; i++)
		if (!c->surfaces[i].free) {
			VCPDBG("surfaces_idle: %u of %u still owned by firmware\n", i,
			       c->pool_count);
			return false;
		}
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
		VCPDBG("last: no capture buffer available, deferring the LAST marker\n");
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
	vdec_state(c, "res_change/in");
	ret = mtk_vcp_vdec_reset(c->decoder, true);
	if (ret) {
		VCPDBG("res_change: drain reset failed: %d\n", ret);
		return ret;
	}
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
	VCPDBG("res_change: drain done, pending=%u\n", c->pending_count);
	ret = mtk_vcp_vdec_reset(c->decoder, false);
	if (ret) {
		VCPDBG("res_change: flush reset failed: %d\n", ret);
		return ret;
	}
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
		if (time_after_eq(jiffies, deadline)) {
			VCPDBG("res_change: surfaces never went idle\n");
			return -ETIMEDOUT;
		}
		wait_event_timeout(c->wait,
				   atomic_read(&c->notification) != seq ||
				   READ_ONCE(c->stopping),
				   msecs_to_jiffies(20));
		seq = atomic_read(&c->notification);
	}
	VCPDBG("res_change: surfaces idle, rebuilding the session\n");
	ret = session_teardown(c);
	if (ret) {
		VCPDBG("res_change: teardown failed: %d\n", ret);
		return ret;
	}
	ret = session_boot(c);
	if (ret) {
		VCPDBG("res_change: session reboot failed: %d\n", ret);
		return ret;
	}
	dev_info(c->dev->dev, "res_change: session rebuilt\n");
	c->header = false;
	m->ignore_cap_streaming = true;
	/* Queries issued after the SOURCE_CHANGE event must describe the stream
	 * that follows it, so the new sequence is parsed before it is published.
	 */
	/* Remember what the capture queue is still sized for: the client may
	 * have to re-queue a buffer it allocated for that geometry before the
	 * deferred LAST marker of the old sequence can be handed out.
	 */
	c->prev_dst_planes = c->dst_fmt.num_planes;
	c->prev_dst_size[0] = c->dst_fmt.plane_fmt[0].sizeimage;
	c->prev_dst_size[1] = c->dst_fmt.plane_fmt[1].sizeimage;
	if (src) {
		ret = parse_headers(c, src);
		if (ret && ret != -EAGAIN) {
			VCPDBG("res_change: header reparse failed: %d\n", ret);
			return ret;
		}
	}
	/* The new resolution is decoded once the client restarts CAPTURE. */
	c->wait_capture = true;
	vdec_state(c, "res_change/out");
	return finish_old_sequence(c);
}

static int allocate_surfaces(struct vdec_ctx *c)
{
	size_t bytes = (size_t)c->pic.size[0] + c->pic.size[1];
	unsigned int i;
	int ret;

	c->pool_count = c->pic.dpb + 3;
	if (c->pool_count > DEC_SURFACES ||
	    (u64)c->pool_count * bytes > SZ_256M)
		return -E2BIG;
	for (i = 0; i < c->pool_count; i++) {
		struct vdec_surface *s = &c->surfaces[i];

		ret = codec_alloc(c, 1, bytes, &s->plane[0]);
		if (ret)
			return ret;
		s->plane[1].cpu = s->plane[0].cpu + c->pic.size[0];
		s->plane[1].dma = s->plane[0].dma + c->pic.size[0];
		s->plane[1].size = c->pic.size[1];
		s->free = true;
	}
	return 0;
}

/* Fill in the buffer layout for the picture the firmware parsed. The firmware
 * geometry is fixed, but the client chooses between the supported single- and
 * multi-planar layouts when it negotiates CAPTURE. 10-bit pictures convert
 * to standard single-plane P010.
 */
static void picture_format(struct vdec_ctx *c, struct v4l2_pix_format_mplane *f)
{
	if (c->pic.fourcc == V4L2_PIX_FMT_MT2T)
		f->pixelformat = V4L2_PIX_FMT_P010;
	else if (f->pixelformat != V4L2_PIX_FMT_NV12M &&
	    f->pixelformat != V4L2_PIX_FMT_NV12)
		f->pixelformat = V4L2_PIX_FMT_NV12M;

	f->width = c->pic.stride;
	f->height = c->pic.buffer_height;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = c->src_fmt.colorspace;
	f->xfer_func = c->src_fmt.xfer_func;
	f->ycbcr_enc = c->src_fmt.ycbcr_enc;
	f->quantization = c->src_fmt.quantization;
	if (f->pixelformat == V4L2_PIX_FMT_P010) {
		f->num_planes = 1;
		f->plane_fmt[0].bytesperline = c->pic.stride * 2;
		f->plane_fmt[0].sizeimage =
			(size_t)c->pic.stride * c->pic.buffer_height * 3;
	} else if (f->pixelformat == V4L2_PIX_FMT_NV12) {
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
	int ret;

	/* Annex B needs a start code and NAL header; VP9 may carry a one-byte
	 * show_existing_frame. Both queue-time and DMA snapshot guards apply.
	 */
	if (!data || !c->bs.cpu || !bytes || bytes > c->bs.size ||
	    (bytes < 4 && c->src_fmt.pixelformat != V4L2_PIX_FMT_VP9))
		return -EINVAL;
	memcpy(c->bs.cpu, data + p->data_offset, bytes);
	/* Check the exact DMA copy as well as QBUF: a userspace mapping must
	 * not be able to change the SPS after the queue-time validation.
	 */
	ret = vcp_vdec_bitstream_guard(c->src_fmt.pixelformat, c->bs.cpu, bytes);
	if (ret)
		return ret;
	c->source_cookie = ++c->next_cookie;
	c->source_done = false;
	VCPDBG("submit: bytes=%u offset=%u cookie=%#llx\n", bytes,
	       p->data_offset, c->source_cookie);
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
		VCPDBG("parse: header submit failed: %d\n", ret);
		return ret;
	}
	/* The parse pass is not the decode pass that follows it. */
	c->submitted = false;
	VCPDBG("parse: changed=%#x\n", changed);
	if (!(changed & BIT(0))) {
		VCPDBG("parse: no picture yet, need another access unit\n");
		return -EAGAIN;
	}
	ret = collect_events(c);
	if (!ret)
		ret = mtk_vcp_vdec_picture(c->decoder, &c->pic);
	VCPDBG("parse: picture %ux%u dpb=%u stride=%u bh=%u ret=%d\n",
	       c->pic.width, c->pic.height, c->pic.dpb, c->pic.stride,
	       c->pic.buffer_height, ret);
	/* The picture geometry is known from here on. H.264 carries its frame rate
	 * in the VUI, which this frontend does not parse, so the request assumes the
	 * panel rate; whether that workload has an operating point at all is decided
	 * by the DVFSRC table. The step is asked for before any capture buffer is
	 * published, so a stream the rail cannot serve fails the session instead of
	 * decoding at a step nobody was granted.
	 */
	if (!ret)
		ret = mtk_vcp_vdec_hw_set_perf(c->dev->hw, c->pic.width, c->pic.height, 60);
	if (!ret)
		ret = allocate_surfaces(c);
	if (ret) {
		VCPDBG("parse: picture/perf/surfaces failed: %d\n", ret);
		return ret;
	}
	VCPDBG("parse: surfaces allocated, pool=%u\n", c->pool_count);
	capture_format(c);
	c->header = true;
	c->fh.m2m_ctx->ignore_cap_streaming = false;
	v4l2_event_queue_fh(&c->fh, &event);
	dev_info(c->dev->dev, "header parsed: %ux%u dpb=%u surfaces=%u\n",
		 c->pic.width, c->pic.height, c->pic.dpb, c->pool_count);
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

	vdec_state(c, "work/in");
	if (READ_ONCE(c->stopping))
		goto finish;
	ret = session_start(c);
	if (ret) {
		VCPDBG("work: session_start failed: %d\n", ret);
		goto error;
	}
	VCPDBG("work: session ready\n");
	/* A resolution change ends the previous capture sequence before any frame
	 * of the new one is decoded; the marker may have been deferred until a
	 * capture buffer became available.
	 */
	if (READ_ONCE(c->last_pending)) {
		VCPDBG("work: deferred LAST marker, finishing the old sequence\n");
		ret = finish_old_sequence(c);
		if (ret)
			goto error;
		goto finish;
	}
	if (READ_ONCE(c->wait_capture)) {
		VCPDBG("work: waiting for the client to restart CAPTURE\n");
		goto finish;
	}
	src = v4l2_m2m_next_src_buf(m);
	if (!c->header) {
		if (!src) {
			VCPDBG("work: header pass, nothing queued on OUTPUT\n");
			goto finish;
		}
		VCPDBG("work: header pass over output buffer %u\n", src->vb2_buf.index);
		ret = parse_headers(c, src);
		if (ret == -EAGAIN) {
			/* Incomplete sequence headers are consumed before
			 * capture starts.
			 */
			VCPDBG("work: incomplete sequence, consuming the buffer\n");
			src = v4l2_m2m_src_buf_remove(m);
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
			ret = 0;
		}
		if (ret) {
			VCPDBG("work: header pass failed: %d\n", ret);
			goto error;
		}
		goto finish;
	}
	ret = collect_events(c);
	if (!ret)
		ret = deliver_frames(c);
	if (ret) {
		VCPDBG("work: event delivery failed: %d\n", ret);
		goto error;
	}
	if (src && !c->drained) {
		if (!vb2_get_plane_payload(&src->vb2_buf, 0)) {
			VCPDBG("work: empty output buffer, entering drain\n");
			c->draining = true;
			src = v4l2_m2m_src_buf_remove(m);
			v4l2_m2m_buf_done(src, VB2_BUF_STATE_DONE);
			c->submitted = false;
			goto drain;
		}
		ret = queue_surfaces(c);
		if (ret) {
			VCPDBG("work: queueing surfaces failed: %d\n", ret);
			goto error;
		}
		if (!c->submitted) {
			/* An earlier job may have handed this buffer to firmware
			 * before STREAMOFF interrupted the wait for its release;
			 * resubmitting it would decode the access unit twice.
			 */
			ret = submit_source(c, src, &changed);
			if (ret) {
				VCPDBG("work: submit failed: %d\n", ret);
				goto error;
			}
			c->submitted = true;
			VCPDBG("work: submitted, changed=%#x\n", changed);
			if (changed & (BIT(2) | BIT(3))) {
				VCPDBG("work: firmware reports a stream error\n");
				ret = -EPIPE;
				goto error;
			}
			if (changed & BIT(0)) {
				ret = res_change_restart(c, src);
				if (ret) {
					VCPDBG("work: resolution change failed: %d\n", ret);
					goto error;
				}
				goto finish;
			}
		}
		deadline = jiffies + msecs_to_jiffies(5000);
		while (!c->source_done) {
			int seq = atomic_read(&c->notification);

			ret = collect_events(c);
			if (!ret)
				ret = deliver_frames(c);
			if (ret) {
				VCPDBG("work: wait loop delivery failed: %d\n", ret);
				goto error;
			}
			if (c->source_done)
				break;
			if (READ_ONCE(c->stopping)) {
				VCPDBG("work: stopped while waiting for the bitstream\n");
				goto finish;
			}
			if (time_after_eq(jiffies, deadline)) {
				VCPDBG("work: timed out waiting for the bitstream release, submitted=%d notification=%d\n",
				       c->submitted, atomic_read(&c->notification));
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
		VCPDBG("work: source done, seq=%u pending=%u\n", src->sequence,
		       c->pending_count);
	}
drain:
	if (c->draining && !c->drained && !v4l2_m2m_num_src_bufs_ready(m)) {
		VCPDBG("work: draining, flushing firmware\n");
		ret = mtk_vcp_vdec_reset(c->decoder, true);
		if (!ret)
			ret = collect_events(c);
		if (!ret)
			ret = deliver_frames(c);
		if (ret)
			goto error;
		c->drained = true;
		VCPDBG("work: drained, pending=%u\n", c->pending_count);
	}
	if (c->drained && !c->pending_count) {
		struct vb2_v4l2_buffer *dst = v4l2_m2m_dst_buf_remove(m);
		const struct v4l2_event event = { .type = V4L2_EVENT_EOS };

		VCPDBG("work: drain complete, dst=%s\n", dst ? "available" : "none");
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
	vdec_state(c, "work/error");
	WRITE_ONCE(c->failed, true);
	vb2_queue_error(v4l2_m2m_get_src_vq(m));
	vb2_queue_error(v4l2_m2m_get_dst_vq(m));
finish:
	v4l2_m2m_job_finish(c->dev->m2m, m);
	VCPDBG("work: job finished\n");
}

static void device_run(void *priv)
{
	struct vdec_ctx *c = priv;

	vdec_state(c, "device_run");
	queue_work(c->dev->queue, &c->work);
}
static int job_ready(void *priv)
{
	struct vdec_ctx *c = priv;
	struct v4l2_m2m_ctx *m = c->fh.m2m_ctx;
	bool src = v4l2_m2m_num_src_bufs_ready(m), dst = v4l2_m2m_num_dst_bufs_ready(m);
	bool fits = dst ? capture_fits(c, v4l2_m2m_next_dst_buf(m)) : false;
	int ready;

	if (READ_ONCE(c->stopping) || READ_ONCE(c->failed))
		ready = 0;
	/* The capture sequence of the previous resolution still needs its LAST
	 * marker, which only needs a capture buffer to hand out.
	 */
	else if (READ_ONCE(c->last_pending))
		ready = dst;
	/* A resolution change suspends decoding until the client restarts
	 * CAPTURE, as the stateful decoder protocol requires.
	 */
	else if (READ_ONCE(c->wait_capture))
		ready = 0;
	else if (!READ_ONCE(c->header))
		ready = src;
	/* Decoding ahead of a capture renegotiation would hand the current
	 * picture to buffers sized for the previous resolution; the header
	 * pass already told the application about the change, so wait for it
	 * to stop and restart the capture queue with matching buffers.
	 */
	else if (dst && !fits)
		ready = 0;
	else if (READ_ONCE(c->pending_count) && dst)
		ready = 1;
	else if (READ_ONCE(c->draining) && !READ_ONCE(c->drained) && !src)
		ready = 1;
	else if (READ_ONCE(c->drained))
		ready = !READ_ONCE(c->pending_count) && dst;
	else
		ready = src && dst;

	VCPDBG("job_ready: src=%u dst=%u fits=%d hdr=%d lastp=%d waitcap=%d pend=%u drn=%d drnd=%d stop=%d fail=%d sub=%d -> %d\n",
	       src, dst, fits, READ_ONCE(c->header), READ_ONCE(c->last_pending),
	       READ_ONCE(c->wait_capture), READ_ONCE(c->pending_count),
	       READ_ONCE(c->draining), READ_ONCE(c->drained),
	       READ_ONCE(c->stopping), READ_ONCE(c->failed), c->submitted, ready);
	return ready;
}
static void job_abort(void *priv)
{
	struct vdec_ctx *c = priv;

	vdec_state(c, "job_abort");
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
		VCPDBG("queue_setup: %u bufs, %u planes accepted\n", *buffers,
		       *planes);
		return 0;
	}
	*planes = f->num_planes;
	for (i = 0; i < *planes; i++)
		sizes[i] = f->plane_fmt[i].sizeimage;
	VCPDBG("queue_setup: %u bufs, configuring %u planes (%ux%u size=%u/%u)\n",
	       *buffers, *planes, f->width, f->height, sizes[0],
	       *planes > 1 ? sizes[1] : 0);
	return 0;
}
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct vdec_ctx *c = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *f = queue_format(c, vb->type);
	unsigned int i;
	int ret;

	for (i = 0; i < f->num_planes; i++) {
		u32 need = f->plane_fmt[i].sizeimage;

		/* The capture queue still holds buffers allocated for the previous
		 * geometry: the client has to be able to hand them back to receive
		 * the LAST marker of that sequence, and the new geometry only starts
		 * once it restarts the queue. Nothing is written into a buffer that
		 * does not fit, deliver_frames() rejects those instead.
		 */
		if (!is_output(vb->type) &&
		    (READ_ONCE(c->last_pending) || READ_ONCE(c->wait_capture)) &&
		    i < c->prev_dst_planes && c->prev_dst_size[i] < need)
			need = c->prev_dst_size[i];
		if (vb2_plane_size(vb, i) < need) {
			VCPDBG("prepare: %s buf %u plane %u too small: have=%lu need=%u, geometry %ux%u\n",
			       is_output(vb->type) ? "output" : "capture",
			       vb->index, i, vb2_plane_size(vb, i), need,
			       f->width, f->height);
			return -EINVAL;
		}
	}
	if (is_output(vb->type) &&
	    (vb->planes[0].data_offset > vb2_get_plane_payload(vb, 0) ||
	     vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0) ||
	     vb2_get_plane_payload(vb, 0) - vb->planes[0].data_offset <
	     (c->src_fmt.pixelformat == V4L2_PIX_FMT_VP9 ? 1 : 4))) {
		VCPDBG("prepare: output buf %u bad offset/payload\n", vb->index);
		return -EINVAL;
	}
	if (is_output(vb->type)) {
		const u8 *data = vb2_plane_vaddr(vb, 0);

		if (!data)
			return -EINVAL;
		ret = vcp_vdec_bitstream_guard(f->pixelformat,
				data + vb->planes[0].data_offset,
				vb2_get_plane_payload(vb, 0) - vb->planes[0].data_offset);
		if (ret)
			return ret;
	}
	VCPDBG("prepare: %s buf %u ok, %ux%u planes=%u\n",
	       is_output(vb->type) ? "output" : "capture", vb->index,
	       f->width, f->height, f->num_planes);
	return 0;
}
static void buffer_queue(struct vb2_buffer *vb)
{
	struct vdec_ctx *c = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(c->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
	VCPDBG("queue: %s buf %u queued ts=%llu (srcq=%u dstq=%u)\n",
	       is_output(vb->type) ? "output" : "capture", vb->index,
	       vb->timestamp,
	       v4l2_m2m_num_src_bufs_ready(c->fh.m2m_ctx),
	       v4l2_m2m_num_dst_bufs_ready(c->fh.m2m_ctx));
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

	VCPDBG("resume: has_stopped=%d drained=%d\n",
	       v4l2_m2m_has_stopped(m), c->drained);
	if (!v4l2_m2m_has_stopped(m))
		return 0;
	if (READ_ONCE(c->drained) && c->decoder) {
		/* LAST disables scheduling; join the worker before resetting. */
		flush_work(&c->work);
		ret = mtk_vcp_vdec_reset(c->decoder, false);
		if (ret) {
			VCPDBG("resume: firmware reset failed: %d\n", ret);
			return ret;
		}
		ret = collect_events(c);
		if (ret) {
			VCPDBG("resume: event collection failed: %d\n", ret);
			return ret;
		}
	}
	/* A stopped queue owns no more buffers; the last one was already
	 * dequeued or is being discarded by the queue restart.
	 */
	vb2_clear_last_buffer_dequeued(v4l2_m2m_get_dst_vq(m));
	VCPDBG("resume: mem2mem stopped state cleared\n");
	return 0;
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vdec_ctx *c = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;
	int ret;

	VCPDBG("start_streaming: %s count=%u\n",
	       is_output(q->type) ? "output" : "capture", count);
	/* CAPTURE restart must not revive a failed firmware session. OUTPUT
	 * STREAMOFF tears it down; only then may OUTPUT start clear the fault.
	 */
	if (c->orphan || (READ_ONCE(c->failed) &&
			  (!is_output(q->type) || c->decoder))) {
		ret = -EIO;
		goto return_buffers;
	}
	WRITE_ONCE(c->stopping, false);
	if (is_output(q->type)) {
		/* Reserve before returning success, even before a worker boots VCP.
		 * Encoders use the same reservation, in either startup order.
		 */
		ret = session_claim(c);
		if (ret)
			goto return_buffers;
		WRITE_ONCE(c->failed, false);
		c->draining = false;
		c->drained = false;
		c->submitted = false;
		c->wait_capture = false;
		v4l2_m2m_clear_state(c->fh.m2m_ctx);
		vdec_state(c, "start_streaming/output");
		return 0;
	}
	/* Restarting CAPTURE is how a client resumes after a drain or after a
	 * resolution change. Clear the software and firmware drain state so the
	 * queued OUTPUT buffers are scheduled again.
	 */
	c->wait_capture = false;
	ret = resume_streaming(c);
	if (ret) {
		VCPDBG("start_streaming: resume failed: %d\n", ret);
		WRITE_ONCE(c->failed, true);
		goto return_buffers;
	}
	c->draining = false;
	c->drained = false;
	v4l2_m2m_clear_state(c->fh.m2m_ctx);
	vdec_state(c, "start_streaming/capture");
	return 0;
return_buffers:
	/* vb2 requires every buffer handed to a failed STREAMON back. */
	while ((vb = is_output(q->type) ?
		v4l2_m2m_src_buf_remove(c->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx)))
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_QUEUED);
	return ret;
}
static void stop_streaming(struct vb2_queue *q)
{
	struct vdec_ctx *c = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vb;
	unsigned int dropped = 0;

	VCPDBG("stop_streaming: %s entry\n",
	       is_output(q->type) ? "output" : "capture");
	vdec_state(c, "stop_streaming/in");
	WRITE_ONCE(c->stopping, true);
	wake_up(&c->wait);
	flush_work(&c->work);
	if (!is_output(q->type)) {
		/* Capture re-setup (e.g. after SOURCE_CHANGE) keeps the
		 * firmware session alive; output STREAMOFF or release ends it.
		 */
		while ((vb = v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx))) {
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
			dropped++;
		}
		VCPDBG("stop_streaming: capture, dropped=%u lastp=%d waitcap=%d\n",
		       dropped, c->last_pending, c->wait_capture);
		/* Restarting CAPTURE abandons the old capture sequence, so its
		 * deferred LAST marker must not be handed out on the first
		 * buffer of the new one.
		 */
		if (READ_ONCE(c->last_pending)) {
			WRITE_ONCE(c->last_pending, false);
			VCPDBG("stop_streaming: capture, dropping the deferred LAST marker\n");
		}
		return;
	}
	session_stop(c);
	while ((vb = v4l2_m2m_src_buf_remove(c->fh.m2m_ctx))) {
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		dropped++;
	}
	while ((vb = v4l2_m2m_dst_buf_remove(c->fh.m2m_ctx))) {
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		dropped++;
	}
	c->header = false;
	c->fh.m2m_ctx->ignore_cap_streaming = true;
	VCPDBG("stop_streaming: output, dropped=%u lastp=%d waitcap=%d\n",
	       dropped, c->last_pending, c->wait_capture);
	vdec_state(c, "stop_streaming/out");
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
		q->mem_ops = i ? &vb2_dma_sg_memops : &vb2_vmalloc_memops;
		/* CAPTURE has CPU writes as well as external device reads. */
		q->bidirectional = !!i;
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
	strscpy(cap->card, "MT6895 VCP decoder", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:mt6895-vcp-dec", sizeof(cap->bus_info));
	return 0;
}
static int enum_format(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (!valid_type(f->type))
		return -EINVAL;
	if (is_output(f->type)) {
		if (f->index >= ARRAY_SIZE(vdec_codecs))
			return -EINVAL;
		f->pixelformat = vdec_codecs[f->index].fourcc;
		/* Coded OUTPUT formats drive codec detection (including
		 * v4l2-compliance stateful-codec identification).
		 */
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
		return 0;
	}
	if (f->index == 0)
		f->pixelformat = V4L2_PIX_FMT_NV12M;
	else if (f->index == 1)
		f->pixelformat = V4L2_PIX_FMT_NV12;
	else if (f->index == 2)
		f->pixelformat = V4L2_PIX_FMT_P010;
	else
		return -EINVAL;
	f->flags = 0;
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
	p->field = V4L2_FIELD_NONE;
	memset(p->plane_fmt, 0, sizeof(p->plane_fmt));
	memset(p->reserved, 0, sizeof(p->reserved));
	if (is_output(f->type)) {
		const struct vdec_codec *k;

		k = vdec_codec_by_fourcc(p->pixelformat);
		if (!k) {
			k = &vdec_codecs[0];
			p->pixelformat = k->fourcc;
		}
		p->width = vdec_dimension(p->width, k->size.min_width,
					  k->size.max_width, k->size.step_width);
		p->height = vdec_dimension(p->height, k->size.min_height,
					   k->size.max_height, k->size.step_height);
		p->num_planes = 1;
		p->plane_fmt[0].sizeimage = clamp_t(u32, size ?: SZ_4M, SZ_64K, SZ_16M);
	} else {
		p->width = vdec_dimension(p->width, vdec_capture_size.min_width,
					  vdec_capture_size.max_width,
					  vdec_capture_size.step_width);
		p->height = vdec_dimension(p->height, vdec_capture_size.min_height,
					   vdec_capture_size.max_height,
					   vdec_capture_size.step_height);
		if (p->pixelformat != V4L2_PIX_FMT_NV12M &&
		    p->pixelformat != V4L2_PIX_FMT_NV12 &&
		    p->pixelformat != V4L2_PIX_FMT_P010)
			p->pixelformat = V4L2_PIX_FMT_NV12M;
		p->colorspace = c->src_fmt.colorspace;
		p->xfer_func = c->src_fmt.xfer_func;
		p->ycbcr_enc = c->src_fmt.ycbcr_enc;
		p->quantization = c->src_fmt.quantization;
		if (p->pixelformat == V4L2_PIX_FMT_P010) {
			p->num_planes = 1;
			p->plane_fmt[0].bytesperline = p->width * 2;
			p->plane_fmt[0].sizeimage = (size_t)p->width * p->height * 3;
		} else if (p->pixelformat == V4L2_PIX_FMT_NV12) {
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
			u32 fourcc = f->fmt.pix_mp.pixelformat;

			c->codec_id = vdec_codec_by_fourcc(fourcc)->codec_id;
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
	const struct vdec_codec *k;

	if (s->index)
		return -EINVAL;
	if (s->pixel_format == V4L2_PIX_FMT_NV12M ||
	    s->pixel_format == V4L2_PIX_FMT_NV12 ||
	    s->pixel_format == V4L2_PIX_FMT_P010) {
		s->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		s->stepwise = vdec_capture_size;
		return 0;
	}
	k = vdec_codec_by_fourcc(s->pixel_format);
	if (!k)
		return -EINVAL;
	s->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	s->stepwise = k->size;
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

	VCPDBG("decoder_cmd: cmd=%u\n", cmd->cmd);
	if (ret)
		return ret;
	if (READ_ONCE(c->failed) || c->orphan)
		return -EIO;
	if (cmd->cmd == V4L2_DEC_CMD_STOP) {
		c->draining = true;
		vdec_state(c, "decoder_cmd/stop");
	} else if (cmd->cmd == V4L2_DEC_CMD_START) {
		if (READ_ONCE(c->draining) && !v4l2_m2m_has_stopped(c->fh.m2m_ctx)) {
			VCPDBG("decoder_cmd: START while draining and not stopped\n");
			return -EBUSY;
		}
		ret = resume_streaming(c);
		if (ret) {
			VCPDBG("decoder_cmd: resume failed: %d\n", ret);
			return ret;
		}
		c->draining = false;
		c->drained = false;
		c->wait_capture = false;
		v4l2_m2m_clear_state(c->fh.m2m_ctx);
		vdec_state(c, "decoder_cmd/start");
	} else {
		return -EINVAL;
	}
	v4l2_m2m_try_schedule(c->fh.m2m_ctx);
	VCPDBG("decoder_cmd: done, ret=%d\n", ret);
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
	ret = recover_released_session(d);
	if (ret)
		goto unlock;
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
	c->codec_id = vdec_codec_by_fourcc(c->src_fmt.pixelformat)->codec_id;
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
	VCPDBG("open: new context %px, source %ux%u\n", c, c->src_fmt.width,
	       c->src_fmt.height);
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
	vdec_state(c, "release/in");
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
	else
		c->released = true;
	mutex_unlock(&d->lock);
	VCPDBG("release: done\n");
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
MODULE_DESCRIPTION("MediaTek MT6895 VCP stateful V4L2 decoder");
