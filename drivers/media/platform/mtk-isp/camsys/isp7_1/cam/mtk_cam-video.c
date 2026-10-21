// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2019 MediaTek Inc.

#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/iosys-map.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_cam.h"
#include "mtk_cam-feature.h"
#include "mtk_cam-video.h"
#include "mtk_camera-v4l2-controls.h"
#include "mtk_camera-videodev2.h"
#include "mtk_cam-ufbc-def.h"

#include "mtk_cam_vb2-dma-contig.h"
#include "mtk_cam-trace.h"

/* XAGA: 1 = fill frame_params.img_outs[] for the raw YUV outputs
 * (YUVO_1..5, RZH1N2TO_1..3, DRZS4NO_1..3) when their buffers are queued.
 *
 * This is a MAINLINE-ONLY addition - the downstream mtk_cam-video.c has no
 * MTKCAM_IPI_RAW_YUVO_1 case at all. libccd gates each DMA branch on
 * img_outs[i].buf[0][0].iova != 0 (mask bits 10..14 for yuvo1..5) and takes the
 * ipi format from img_outs[i].fmt.format, so filling these makes the composer
 * emit CAM_YUV DRAM branches the vendor never programs - only RZH1N2TO_R2
 * (0x4900) and TNCSYO_R1 (0x4bc0) appear in the vendor's captured page-5 table.
 * With those extra branches live, streaming collapses the display/SMI:
 *
 *   [DISP][E][IRQ] DDP_COMPONENT_DSI0: buffer underrun / OVL frame underflow
 *   [cmdq][err] smi hang:0 -> conninfra ... reason=bus hang -> watchdog reset
 *
 * Default 0: leave img_outs[] zero for those ports so libccd composes only what
 * the vendor composes. IMGO is unaffected (it is handled separately below).
 */
static int p_yuv_outs;
module_param(p_yuv_outs, int, 0644);

/**
 * The MTK camera HAL passes a media request fd in v4l2_format.reserved[]
 * on S_FMT and expects it echoed back. This is not standard V4L2 and makes
 * v4l2-compliance fail the format tests. Keep it on by default for HAL
 * compatibility; set to 0 (e.g. via
 * /sys/module/mtk_cam_isp/parameters/request_fd_echo) for standard
 * userspace / v4l2-compliance runs.
 */
static bool request_fd_echo = true;
module_param(request_fd_echo, bool, 0644);
MODULE_PARM_DESC(request_fd_echo,
	"Echo request fd in v4l2_format.reserved[] on S_FMT (MTK HAL compat)");

/* For end point race condition */
#include <linux/platform_data/mtk_ccd.h>
static struct mtk_ccd *g_ccd;

/*
 * Note
 *	differt dma(fmt) would have different bus_size
 *	align xsize(bytes per line) with [bus_size * pixel_mode]
 */
int mtk_cam_is_fullg(unsigned int ipi_fmt)
{
	return (ipi_fmt == MTKCAM_IPI_IMG_FMT_FG_BAYER8)
		|| (ipi_fmt == MTKCAM_IPI_IMG_FMT_FG_BAYER10)
		|| (ipi_fmt == MTKCAM_IPI_IMG_FMT_FG_BAYER12);
}

static inline
int mtk_cam_dma_bus_size(int bpp, int pixel_mode_shift, int is_fg)
{
	unsigned int bus_size = ALIGN(bpp, 16) << pixel_mode_shift;

	if (is_fg)
		bus_size <<= 1;
	return bus_size / 8; /* in bytes */
}

static inline
int mtk_cam_yuv_dma_bus_size(int bpp, int pixel_mode_shift)
{
	unsigned int bus_size = ALIGN(bpp, 32);

	return bus_size / 8; /* in bytes */
}

int mtk_cam_dmao_xsize(int w, unsigned int ipi_fmt, int pixel_mode_shift)
{
	const int is_fg		= mtk_cam_is_fullg(ipi_fmt);
	const int bpp		= mtk_cam_get_pixel_bits(ipi_fmt);
	const int bytes		= is_fg ?
		DIV_ROUND_UP(w * bpp * 3 / 2, 8) : DIV_ROUND_UP(w * bpp, 8);
	const int bus_size	= mtk_cam_dma_bus_size(bpp, pixel_mode_shift, is_fg);

	return ALIGN(bytes, bus_size);
}

static int mtk_cam_vb2_queue_setup(struct vb2_queue *vq,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct mtk_cam_device *cam = vb2_get_drv_priv(vq);
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vq);
	struct mtk_raw_pipeline *raw_pipeline;
	unsigned int max_buffer_count = node->desc.max_buf_count;
	const struct v4l2_format *fmt = &node->active_fmt;
	unsigned int size;
	int i;
	int min_buf_sz;

	min_buf_sz = ALIGN(IMG_MIN_WIDTH, IMG_PIX_ALIGN) * IMG_MIN_HEIGHT;

	/* Check the limitation of buffer size */
	if (max_buffer_count)
		*num_buffers = clamp_val(*num_buffers, 1, max_buffer_count);

	if (node->desc.smem_alloc)
		vq->dma_attrs |= DMA_ATTR_NO_KERNEL_MAPPING;

	if (vq->type == V4L2_BUF_TYPE_META_OUTPUT ||
	    vq->type == V4L2_BUF_TYPE_META_CAPTURE)
		size = fmt->fmt.meta.buffersize;
	else if (fmt->fmt.pix_mp.plane_fmt[0].sizeimage)
		/**
		 * Buffers must be large enough for the currently set
		 * format, so CREATE_BUFS can reject undersized requests.
		 */
		size = max_t(u32, fmt->fmt.pix_mp.plane_fmt[0].sizeimage,
			     min_buf_sz);
	else
		size = min_buf_sz;

	/* Add for q.create_bufs with fmt.g_sizeimage(p) / 2 test */
	if (*num_planes) {
		if (sizes[0] < size || *num_planes != 1)
			return -EINVAL;
	} else {
		/* TBC: subsampling configuration */
		if (is_raw_subdev(node->uid.pipe_id)) {
			raw_pipeline = mtk_cam_dev_get_raw_pipeline(cam, node->uid.pipe_id);
			if (raw_pipeline && node->desc.id == MTK_RAW_MAIN_STREAM_OUT) {
				if (raw_pipeline->dynamic_exposure_num_max >
				    fmt->fmt.pix_mp.num_planes) {
					*num_planes = raw_pipeline->dynamic_exposure_num_max;
					for (i = 0; i < *num_planes; i++)
						sizes[i] = size;
				} else if (raw_pipeline && raw_pipeline->user_res.raw_res.feature &&
					fmt->fmt.pix_mp.num_planes > 1) {
					*num_planes = fmt->fmt.pix_mp.num_planes;

					for (i = 0; i < *num_planes; i++)
						sizes[i] = size;
				} else {
					*num_planes = 1;
					sizes[0] = size;
				}
			} else if (raw_pipeline && raw_pipeline->user_res.raw_res.feature
				&& fmt->fmt.pix_mp.num_planes > 1) {
				*num_planes = fmt->fmt.pix_mp.num_planes;
				for (i = 0; i < *num_planes; i++)
					sizes[i] = size;
			} else {
				*num_planes = 1;
				sizes[0] = size;
			}
		} else if (is_camsv_subdev(node->uid.pipe_id)) {
			*num_planes = 1;
			sizes[0] = size;
		} else if (is_mraw_subdev(node->uid.pipe_id)) {
			*num_planes = 1;
			sizes[0] = size;
		} else {
			*num_planes = 1;
			sizes[0] = size;
		}
	}

	return 0;
}

static int mtk_cam_vb2_buf_init(struct vb2_buffer *vb)
{
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vb->vb2_queue);
	struct device *dev = vb->vb2_queue->dev;
	struct mtk_cam_buffer *buf;
	dma_addr_t addr;

	buf = mtk_cam_vb2_buf_to_dev_buf(vb);
	buf->daddr = vb2_dma_contig_plane_dma_addr(vb, 0);
	buf->scp_addr = 0;

	/* SCP address is only valid for meta input buffer */
	if (!node->desc.smem_alloc)
		return 0;

	buf = mtk_cam_vb2_buf_to_dev_buf(vb);
	/* Use coherent address to get iova address */
	addr = dma_map_resource(dev, buf->daddr, vb->planes[0].length,
				DMA_BIDIRECTIONAL, DMA_ATTR_SKIP_CPU_SYNC);
	if (dma_mapping_error(dev, addr)) {
		dev_info(dev, "failed to map meta addr:%pad\n", &buf->daddr);
		return -EFAULT;
	}
	buf->scp_addr = buf->daddr;
	buf->daddr = addr;

	return 0;
}

static int mtk_cam_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vb->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);
	const struct v4l2_format *fmt = &node->active_fmt;
	unsigned int size;
	unsigned int plane;

	if (node->desc.need_cache_sync_on_prepare) {
		dev_dbg(vb->vb2_queue->dev, "%s: %s\n",
			__func__, node->desc.name);
		for (plane = 0; plane < vb->num_planes; ++plane)
			mtk_cam_vb2_sync_for_device(vb->planes[plane].mem_priv);
	}

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_META_OUTPUT ||
	    vb->vb2_queue->type == V4L2_BUF_TYPE_META_CAPTURE)
		size = fmt->fmt.meta.buffersize;
	else
		size = fmt->fmt.pix_mp.plane_fmt[0].sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_info(vb->vb2_queue->dev, "plane size is too small:%lu<%u\n",
			vb2_plane_size(vb, 0), size);
		/* return -EINVAL; */
	}

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if ((vb2_get_plane_payload(vb, 0) != size) && (vb->vb2_queue->streaming)) {
			dev_dbg(vb->vb2_queue->dev, "plane payload is mismatch:%lu:%u\n",
				vb2_get_plane_payload(vb, 0), size);
			/* todo: user must set correct byteused */
			/* return -EINVAL;*/
		}
		return 0;
	}

	v4l2_buf->field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void mtk_cam_vb2_buf_finish(struct vb2_buffer *vb)
{
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vb->vb2_queue);
	unsigned int plane;

	/* XAGA trace: userspace is dequeuing this CAMSYS buffer */
	pr_info("XAGA-DQ %s len0=%u len1=%u np=%u\n", node->desc.name,
		vb->num_planes > 0 ? vb->planes[0].length : 0,
		vb->num_planes > 1 ? vb->planes[1].length : 0,
		vb->num_planes);
	u32 *hdr;
	unsigned int width, height, payload_off, plane_sz;

	if (node->desc.need_cache_sync_on_finish) {
		dev_dbg(vb->vb2_queue->dev, "%s: %s\n",
			__func__, node->desc.name);
		for (plane = 0; plane < vb->num_planes; ++plane)
			mtk_cam_vb2_sync_for_cpu(vb->planes[plane].mem_priv);
	}

	/*
	 * The camsv FBC write path puts a 24-byte frame header
	 * ([1][width][height][stride][payload offset][payload size])
	 * in front of the image data. Standard v4l2 userspace expects
	 * the frame at offset 0 with bytesused == width * height * bpp,
	 * so move the payload over the header and report the payload
	 * size (ffmpeg rejects any other bytesused with a blank frame).
	 * The header fields are validated against the active format
	 * first; a buffer never written by hardware keeps its old
	 * payload.
	 */
	if (node->desc.id != MTK_CAMSV_MAIN_STREAM_OUT ||
	    vb->num_planes != 1)
		return;

	plane_sz = vb2_plane_size(vb, 0);
	hdr = vb2_plane_vaddr(vb, 0);
	if (!hdr || plane_sz < 4096)
		return;

	width = node->active_fmt.fmt.pix_mp.width;
	height = node->active_fmt.fmt.pix_mp.height;
	payload_off = hdr[4];
	if (hdr[1] != width || hdr[2] != height ||
	    payload_off < 8 || payload_off > 64 || (payload_off & 7) ||
	    payload_off >= plane_sz / 2)
		return;

	memmove(hdr, (u8 *)hdr + payload_off, plane_sz - payload_off);
	vb2_set_plane_payload(vb, 0, plane_sz - payload_off);
}

/*
 * XAGA mainline: number of nodes that still need to start streaming
 * before ctx_stream_on may fire. A node counts as "pending" only if
 * its vb2 queue actually holds buffers (i.e. userspace queued to it),
 * which keeps single-node capture working while still deferring
 * ctx_stream_on when a real multi-node setup (e.g. meta-in + imgo) is
 * in use and one of those nodes has not started yet.
 *
 * The ctx does not keep a list of its nodes, so iterate every video
 * node of the pipelines reachable from the pipelines registered under
 * this ctx (raw/sv/mraw) through their vdev_nodes arrays.
 */
static unsigned int count_pending_stream_nodes(struct mtk_cam_ctx *ctx,
					       struct mtk_cam_video_device *cur)
{
	struct mtk_cam_video_device *v;
	unsigned int pending = 1; /* the node currently starting */
	unsigned int i, n;

	/* raw pipeline nodes */
	if (ctx->pipe) {
		n = ARRAY_SIZE(ctx->pipe->vdev_nodes);
		for (i = 0; i < n; i++) {
			v = &ctx->pipe->vdev_nodes[i];
			if (v == cur || vb2_start_streaming_called(&v->vb2_q))
				continue;
			if (v->vb2_q.queued_count > 0)
				pending++;
		}
	}

	/* camsv pipeline nodes */
	for (i = 0; i < MAX_SV_PIPES_PER_STREAM; i++) {
		struct mtk_camsv_pipeline *svp;
		unsigned int k;

		if (!ctx->sv_pipe[i])
			continue;
		svp = ctx->sv_pipe[i];
		n = ARRAY_SIZE(svp->vdev_nodes);
		for (k = 0; k < n; k++) {
			v = &svp->vdev_nodes[k];
			if (v == cur || vb2_start_streaming_called(&v->vb2_q))
				continue;
			if (v->vb2_q.queued_count > 0)
				pending++;
		}
	}

	return pending;
}

static int mtk_cam_vb2_start_streaming(struct vb2_queue *vq,
				       unsigned int count)
{
	struct mtk_cam_device *cam = vb2_get_drv_priv(vq);
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vq);
	struct media_entity *entity = &node->vdev.entity;
	struct mtk_cam_ctx *ctx = NULL;
	struct device *dev = cam->dev;
	int ret, is_first_ctx = 0;

	/* check entity is linked */
	if (!node->enabled) {
		dev_info(cam->dev,
			"%s: stream on failed, node is not enabled\n",
			node->desc.name);
		ret = -ENOLINK;
		goto fail_return_buffer;
	}

	is_first_ctx = !cam->composer_cnt;
	if (is_first_ctx) {
		/* power on the remote proc device */
		if (!cam->rproc_handle)  {
			/* Get the remote proc device of composers */
			cam->rproc_handle =
				rproc_get_by_phandle(cam->rproc_phandle);
			if (!cam->rproc_handle) {
				dev_info(cam->dev,
					"fail to get rproc_handle\n");
				ret = -ENOLINK;
				goto fail_return_buffer;
			}
			g_ccd = (struct mtk_ccd *)cam->rproc_handle->priv;
		}
	}

	mutex_lock(g_ccd->ccd_open_mutex);
	/*
	 * Seninf fan-out (raw + camsv on one sensor VC): the second node's pad
	 * is already part of the running ctx's media pipeline, so reuse that
	 * ctx. Starting a new session would fail media_pipeline_start() with
	 * -EBUSY on the shared sensor/seninf pads.
	 */
	ctx = mtk_cam_find_ctx(cam, entity);
	if (ctx && !ctx->streaming)
		ctx = NULL;

	if (!ctx) {
		if (!media_entity_is_streaming(entity)) {
			ctx = mtk_cam_start_ctx(cam, node);
			if (!ctx) {
				mutex_unlock(g_ccd->ccd_open_mutex);
				return -ENOLINK;
			}
		} else {
			ctx = mtk_cam_find_ctx(cam, entity);
			if (WARN_ON(!ctx)) {
				mutex_unlock(g_ccd->ccd_open_mutex);
				return -ENOLINK;
			}
		}
	}

	cam->streaming_pipe |= (1 << node->uid.pipe_id);
	ctx->streaming_pipe |= (1 << node->uid.pipe_id);
	ctx->streaming_node_cnt++;

#if CCD_READY
	if (ctx->streaming_node_cnt == 1)
		if (is_raw_subdev(node->uid.pipe_id)) {
			if (!isp_composer_create_session(ctx)) {
				ctx->session_created = 1;
			} else {
				complete(&ctx->session_complete);
				ctx->streaming_node_cnt--;
				ctx->streaming_pipe &= ~(1 << node->uid.pipe_id);
				cam->streaming_pipe &= ~(1 << node->uid.pipe_id);
				mtk_cam_dev_req_cleanup(ctx, node->uid.pipe_id,
								VB2_BUF_STATE_QUEUED);
				mtk_cam_stop_ctx(ctx, entity);
				mutex_unlock(g_ccd->ccd_open_mutex);
				return -EBUSY;
			}
		}
#endif

	/* XAGA diag: promoted from dev_dbg (raw streamon gating trace) */
	dev_info(dev, "%s:%s:ctx(%d): node:%d count info:%d/%d\n", __func__,
		node->desc.name, ctx->stream_id, node->desc.id,
		ctx->streaming_node_cnt, ctx->enabled_node_cnt);
	mutex_unlock(g_ccd->ccd_open_mutex);

	/*
	 * XAGA mainline: the vendor gate (streaming_node_cnt ==
	 * enabled_node_cnt) assumes every reachable video node gets
	 * buffers queued each frame (the Android HAL streams all 22
	 * raw-0 nodes together). Plain single-node streaming would
	 * never fire ctx_stream_on. New rule: fire ctx_stream_on as
	 * soon as every video node that belongs to this ctx's
	 * pipelines AND has queued buffers is streaming. For the
	 * single-node capture used on mainline this means
	 * ctx_stream_on fires immediately.
	 */
	/* Context already up (fan-out second node): its buffers were enqueued
	 * through the request-less path; nothing more to configure.
	 */
	if (ctx->streaming)
		return 0;

	if (ctx->streaming_node_cnt < count_pending_stream_nodes(ctx, node))
		return 0;

	/* all enabled nodes are streaming, enable all subdevs */
	MTK_CAM_TRACE_BEGIN(BASIC, "ctx_stream_on");
	ret = mtk_cam_ctx_stream_on(ctx);
	MTK_CAM_TRACE_END(BASIC);

	if (ret)
		goto fail_destroy_session;

	return 0;

fail_destroy_session:
	if (ctx->session_created)
		isp_composer_destroy_session(ctx);

	ctx->streaming_node_cnt--;
	ctx->streaming_pipe &= ~(1 << node->uid.pipe_id);
	cam->streaming_pipe &= ~(1 << node->uid.pipe_id);
	mtk_cam_dev_req_cleanup(ctx, node->uid.pipe_id, VB2_BUF_STATE_QUEUED);
	mtk_cam_stop_ctx(ctx, entity);
fail_return_buffer:
	/* relese bufs by request */
	return ret;
}

static void mtk_cam_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_cam_device *cam = vb2_get_drv_priv(vq);
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vq);
	struct device *dev = cam->dev;
	struct mtk_cam_ctx *ctx;

	ctx = mtk_cam_find_ctx(cam, &node->vdev.entity);
	if (WARN_ON(!ctx)) {
		/* the ctx is stop, media_pipeline_stop is called */
		mtk_cam_dev_req_clean_pending(cam, node->uid.pipe_id,
					      VB2_BUF_STATE_ERROR);
		return;
	}

	dev_dbg(dev, "%s:%s:ctx(%d): node:%d count info:%d\n", __func__,
		node->desc.name, ctx->stream_id, node->desc.id, ctx->streaming_node_cnt);

	/*
	 * XAGA mainline: stream-on is fired by the relaxed "every node with
	 * queued buffers is streaming" rule above, but stream-off still used
	 * the vendor gate (streaming_node_cnt == enabled_node_cnt), which only
	 * holds when ALL enabled graph nodes stream - i.e. never for a
	 * single-node capture (1 of 22).  ctx->streaming then stayed true
	 * forever, so the next STREAMON short-circuited in
	 * mtk_cam_ctx_stream_on() ("already streaming on"), the raw engine was
	 * never selected again (pipe->enabled_raw == 0), get_master_raw_id()
	 * returned no master, no CQ was ever applied, and the ctx watchdog
	 * spun on forever.  Any partly-streamed ctx must be torn down here.
	 */
	if (ctx->streaming_node_cnt == ctx->enabled_node_cnt ||
	    ctx->streaming_node_cnt <= 1) {
		MTK_CAM_TRACE_BEGIN(BASIC, "ctx_stream_off");
		mtk_cam_ctx_stream_off(ctx);
		MTK_CAM_TRACE_END(BASIC);
	}

	if (cam->streaming_pipe & (1 << node->uid.pipe_id)) {
		/* NOTE: take multi-pipelines case into consideration     */
		/* Moreover, must clean bit mask before req cleanup       */
		/* Otherwise, would cause req not removed in pending list */
		cam->streaming_pipe &= ~(1 << node->uid.pipe_id);
		mtk_cam_dev_req_cleanup(ctx, node->uid.pipe_id, VB2_BUF_STATE_ERROR);
	}

	/* all bufs of node should be return by per requests */

	/* NOTE: take multi-pipelines case into consideration */
	cam->streaming_pipe &= ~(1 << node->uid.pipe_id);
	ctx->streaming_node_cnt--;
	if (ctx->streaming_node_cnt)
		return;

	mtk_cam_stop_ctx(ctx, &node->vdev.entity);
}

int is_mtk_format(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_YUYV10:
	case V4L2_PIX_FMT_YVYU10:
	case V4L2_PIX_FMT_UYVY10:
	case V4L2_PIX_FMT_VYUY10:
	case V4L2_PIX_FMT_YUYV12:
	case V4L2_PIX_FMT_YVYU12:
	case V4L2_PIX_FMT_UYVY12:
	case V4L2_PIX_FMT_VYUY12:
	case V4L2_PIX_FMT_MTISP_YUYV10P:
	case V4L2_PIX_FMT_MTISP_YVYU10P:
	case V4L2_PIX_FMT_MTISP_UYVY10P:
	case V4L2_PIX_FMT_MTISP_VYUY10P:
	case V4L2_PIX_FMT_MTISP_YUYV12P:
	case V4L2_PIX_FMT_MTISP_YVYU12P:
	case V4L2_PIX_FMT_MTISP_UYVY12P:
	case V4L2_PIX_FMT_MTISP_VYUY12P:
	case V4L2_PIX_FMT_NV12_10:
	case V4L2_PIX_FMT_NV21_10:
	case V4L2_PIX_FMT_NV16_10:
	case V4L2_PIX_FMT_NV61_10:
	case V4L2_PIX_FMT_NV12_12:
	case V4L2_PIX_FMT_NV21_12:
	case V4L2_PIX_FMT_NV16_12:
	case V4L2_PIX_FMT_NV61_12:
	case V4L2_PIX_FMT_MTISP_NV12_10P:
	case V4L2_PIX_FMT_MTISP_NV21_10P:
	case V4L2_PIX_FMT_MTISP_NV16_10P:
	case V4L2_PIX_FMT_MTISP_NV61_10P:
	case V4L2_PIX_FMT_MTISP_NV12_12P:
	case V4L2_PIX_FMT_MTISP_NV21_12P:
	case V4L2_PIX_FMT_MTISP_NV16_12P:
	case V4L2_PIX_FMT_MTISP_NV61_12P:
	case V4L2_PIX_FMT_MTISP_NV12_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_UFBC:
	case V4L2_PIX_FMT_MTISP_NV12_10_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_10_UFBC:
	case V4L2_PIX_FMT_MTISP_NV12_12_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_12_UFBC:
	case V4L2_PIX_FMT_MTISP_SGRB8F:
	case V4L2_PIX_FMT_MTISP_SGRB10F:
	case V4L2_PIX_FMT_MTISP_SGRB12F:
		return 1;
	break;
	default:
		return 0;
	break;
	}
}

int is_yuv_ufo(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_MTISP_NV12_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_UFBC:
	case V4L2_PIX_FMT_MTISP_NV12_10_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_10_UFBC:
	case V4L2_PIX_FMT_MTISP_NV12_12_UFBC:
	case V4L2_PIX_FMT_MTISP_NV21_12_UFBC:
		return 1;
	default:
		return 0;
	}
}

int is_raw_ufo(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_MTISP_BAYER8_UFBC:
	case V4L2_PIX_FMT_MTISP_BAYER10_UFBC:
	case V4L2_PIX_FMT_MTISP_BAYER12_UFBC:
	case V4L2_PIX_FMT_MTISP_BAYER14_UFBC:
		return 1;
	default:
		return 0;
	}
}

int is_fullg_rb(u32 pixelformat)
{
	switch (pixelformat) {
	case V4L2_PIX_FMT_MTISP_SGRB8F:
	case V4L2_PIX_FMT_MTISP_SGRB10F:
	case V4L2_PIX_FMT_MTISP_SGRB12F:
		return 1;
	default:
		return 0;
	}
}

const struct mtk_format_info *mtk_format_info(u32 format)
{
	static const struct mtk_format_info formats[] = {
		/* YUV planar formats */
		{ .format = V4L2_PIX_FMT_NV12_10,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV21_10,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV16_10,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV61_10,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_YUYV10,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_YVYU10,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_UYVY10,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_VYUY10,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV12_12,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV21_12,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV16_12,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_NV61_12,  .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_YUYV12,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_YVYU12,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_UYVY12,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_VYUY12,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 2, .bit_r_den = 1 },
		/* YUV packed formats */
		{ .format = V4L2_PIX_FMT_MTISP_YUYV10P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_YVYU10P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_UYVY10P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_VYUY10P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV12_10P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV21_10P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV16_10P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV61_10P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_YUYV12P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_YVYU12P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_UYVY12P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_VYUY12P,	  .mem_planes = 1, .comp_planes = 1,
			.bpp = { 2, 0, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_NV12_12P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_NV21_12P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_NV16_12P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_NV61_12P, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		/* YUV UFBC formats */
		{ .format = V4L2_PIX_FMT_MTISP_NV12_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_MTISP_NV21_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 2, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_MTISP_NV12_10_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV21_10_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_NV12_12_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_NV21_12_UFBC, .mem_planes = 1, .comp_planes = 2,
			.bpp = { 1, 2, 0, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_BAYER8_UFBC, .mem_planes = 1, .comp_planes = 1,
			.bpp = { 1, 0, 0, 0 }, .hdiv = 1, .vdiv = 1,
			.bit_r_num = 1, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_MTISP_BAYER10_UFBC, .mem_planes = 1, .comp_planes = 1,
			.bpp = { 1, 0, 0, 0 }, .hdiv = 1, .vdiv = 1,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_BAYER12_UFBC, .mem_planes = 1, .comp_planes = 1,
			.bpp = { 1, 0, 0, 0 }, .hdiv = 1, .vdiv = 1,
			.bit_r_num = 3, .bit_r_den = 2 },
		{ .format = V4L2_PIX_FMT_MTISP_BAYER14_UFBC, .mem_planes = 1, .comp_planes = 1,
			.bpp = { 1, 0, 0, 0 }, .hdiv = 1, .vdiv = 1,
			.bit_r_num = 7, .bit_r_den = 4 },
		/* Full-G RGB formats */
		{ .format = V4L2_PIX_FMT_MTISP_SGRB8F, .mem_planes = 1, .comp_planes = 3,
			.bpp = { 1, 1, 1, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 1, .bit_r_den = 1 },
		{ .format = V4L2_PIX_FMT_MTISP_SGRB10F, .mem_planes = 1, .comp_planes = 3,
			.bpp = { 1, 1, 1, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 5, .bit_r_den = 4 },
		{ .format = V4L2_PIX_FMT_MTISP_SGRB12F, .mem_planes = 1, .comp_planes = 3,
			.bpp = { 1, 1, 1, 0 }, .hdiv = 2, .vdiv = 2,
			.bit_r_num = 3, .bit_r_den = 2 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i)
		if (formats[i].format == format)
			return &formats[i];
	return NULL;
}

/*
 * XAGA-PROBE: sample a RAW P1 output buffer a few seconds after the HAL queued
 * it, to see whether the camsys DMA engines actually wrote pixels into it.
 * (Non-zero Y plane => P1 works and the black preview is downstream.)
 */
struct xaga_p1_probe {
	struct delayed_work dwork;
	struct dma_buf *db;
	int fd;
};

static void xaga_p1_probe_fn(struct work_struct *work)
{
	struct xaga_p1_probe *p = container_of(to_delayed_work(work),
					       struct xaga_p1_probe, dwork);
	struct iosys_map map;
	unsigned int i, n, nz = 0, zero = 0;
	u8 mn = 0xff, mx = 0;
	u64 sum = 0;
	u8 *v;

	n = (unsigned int)min_t(u64, (u64)p->db->size, 8ull << 20);
	if (dma_buf_begin_cpu_access(p->db, DMA_FROM_DEVICE) == 0) {
		if (dma_buf_vmap(p->db, &map) == 0) {
			v = map.vaddr;
			for (i = 0; i < n; i++) {
				u8 b = v[i];

				if (!b) {
					zero++;
					continue;
				}
				nz++;
				sum += b;
				if (b < mn)
					mn = b;
				if (b > mx)
					mx = b;
			}
			pr_info("XAGA-PROBE[P1] fd=%d size=%llu scanned=%u nz=%u zero=%u min=%u max=%u avg=%llu head=%02x%02x%02x%02x %02x%02x%02x%02x\n",
				p->fd, (u64)p->db->size, n, nz, zero,
				mn, mx, nz ? div_u64(sum, nz) : 0,
				v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
			dma_buf_vunmap(p->db, &map);
		} else {
			pr_info("XAGA-PROBE[P1] fd=%d vmap BUSY\n", p->fd);
		}
		dma_buf_end_cpu_access(p->db, DMA_FROM_DEVICE);
	} else {
		pr_info("XAGA-PROBE[P1] fd=%d begin_cpu_access failed\n", p->fd);
	}
	dma_buf_put(p->db);
	kfree(p);
}

static DEFINE_SPINLOCK(xaga_p1_probe_lock);
static int xaga_p1_probe_fds[32];
static int xaga_p1_probe_nfd;
static int xaga_p1_probe_total;

static void xaga_p1_probe_take(int fd)
{
	struct xaga_p1_probe *p;
	struct dma_buf *db;
	unsigned long flags;
	int k, dup = 0;

	if (fd <= 0 || xaga_p1_probe_total >= 8)
		return;

	spin_lock_irqsave(&xaga_p1_probe_lock, flags);
	for (k = 0; k < xaga_p1_probe_nfd; k++) {
		if (xaga_p1_probe_fds[k] == fd) {
			dup = 1;
			break;
		}
	}
	if (!dup) {
		if (xaga_p1_probe_nfd >= 32) {
			spin_unlock_irqrestore(&xaga_p1_probe_lock, flags);
			return;
		}
		xaga_p1_probe_fds[xaga_p1_probe_nfd++] = fd;
		xaga_p1_probe_total++;
	}
	spin_unlock_irqrestore(&xaga_p1_probe_lock, flags);
	if (dup)
		return;

	db = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(db)) {
		pr_info("XAGA-PROBE[P1] fd=%d get failed\n", fd);
		return;
	}
	if (db->size < (256u << 10)) {
		dma_buf_put(db);
		return;
	}
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		dma_buf_put(db);
		return;
	}
	p->db = db;
	p->fd = fd;
	INIT_DELAYED_WORK(&p->dwork, xaga_p1_probe_fn);
	schedule_delayed_work(&p->dwork, msecs_to_jiffies(7000));
	pr_info("XAGA-PROBE[P1] armed fd=%d size=%llu\n", fd, (u64)db->size);
}

static void mtk_cam_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_cam_device *cam = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int pipe_id;
	struct mtk_cam_buffer *buf = mtk_cam_vb2_buf_to_dev_buf(vb);
	struct mtk_cam_request *req = to_mtk_cam_req(vb->request);
	struct mtk_cam_request_stream_data *req_stream_data;
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vb->vb2_queue);
	struct mtk_raw_pde_config *pde_cfg;
	struct device *dev = cam->dev;
	unsigned int desc_id;
	unsigned int dma_port;
	unsigned int width, height, stride;
	void *vaddr;
	struct mtkcam_ipi_frame_param *frame_param;
	struct mtkcam_ipi_meta_input *meta_in;
	struct mtkcam_ipi_meta_output *meta_out;
	struct mtk_camsv_frame_params *sv_frame_params;
	struct mtk_raw_pipeline *raw_pipline;
	bool is_no_req = false;


	dma_port = node->desc.dma_port;
	pipe_id = node->uid.pipe_id;
	/* XAGA trace: which camsys node actually receives a queued buffer */
	pr_info("XAGA-QBUF %s pipe=%u req=%d idx=%u\n", node->desc.name, pipe_id,
		vb->request ? 1 : 0, vb->index);
	if (!vb->request) {
		/**
		 * Standard (request-less) userspace: no media request is
		 * attached to this buffer. Use the per-pipe internal
		 * request so the frame pipeline keeps working.
		 */
		req = mtk_cam_no_req_get(cam, node, vb);
		if (!req) {
			dev_info(dev, "%s: no internal req for pipe %d\n",
				 __func__, pipe_id);
			return;
		}
		is_no_req = true;
	}

	req_stream_data = mtk_cam_req_get_s_data(req, pipe_id, 0);
	if (!req_stream_data) {
		dev_info(dev, "%s: pipe(%d) has no stream data\n",
			 __func__, pipe_id);
		return;
	}
	frame_param = &req_stream_data->frame_params;
	sv_frame_params = &req_stream_data->sv_frame_params;
	raw_pipline = mtk_cam_dev_get_raw_pipeline(cam, pipe_id);
	mtk_cam_s_data_set_vbuf(req_stream_data, buf, node->desc.id);

	/* update buffer internal address */
	switch (dma_port) {
	case MTKCAM_IPI_RAW_META_STATS_CFG:
		desc_id = node->desc.id-MTK_RAW_SINK_NUM;
		meta_in = &frame_param->meta_inputs[desc_id];
		meta_in->buf.ccd_fd = vb->planes[0].m.fd;
		/* vb->planes[0].bytesused; todo: vb2_q byteused is zero before stream on */
		meta_in->buf.size = node->active_fmt.fmt.meta.buffersize;
		meta_in->buf.iova = buf->daddr;
		meta_in->uid.id = dma_port;
		vaddr = vb2_plane_vaddr(vb, 0);
		break;
	case MTKCAM_IPI_RAW_META_STATS_0:
	case MTKCAM_IPI_RAW_META_STATS_1:
	case MTKCAM_IPI_RAW_META_STATS_2:
		pde_cfg = &cam->raw.pipelines[node->uid.pipe_id].pde_config;
		desc_id = node->desc.id-MTK_RAW_META_OUT_BEGIN;
		meta_out = &frame_param->meta_outputs[desc_id];
		meta_out->buf.ccd_fd = vb->planes[0].m.fd;
		meta_out->buf.size = node->active_fmt.fmt.meta.buffersize;
		meta_out->buf.iova = buf->daddr;
		meta_out->uid.id = dma_port;
		vaddr = vb2_plane_vaddr(vb, 0);
		mtk_cam_set_meta_stats_info(dma_port, vaddr, pde_cfg);
		break;
	case MTKCAM_IPI_CAMSV_MAIN_OUT:
		dev_info(dev, "%s: camsv qbuf pipe=%d daddr=0x%llx req=%s\n",
			 __func__, pipe_id, (u64)buf->daddr,
			 req->req.debug_str);
		if (node->desc.id == MTK_RAW_MAIN_STREAM_SV_1_OUT) {
			sv_frame_params->img_out.buf[0][0].iova = buf->daddr;
		} else {
#if PDAF_READY
			sv_frame_params->img_out.buf[0][0].iova = buf->daddr +
				mtk_cam_get_meta_size(dma_port);
#else
			sv_frame_params->img_out.buf[0][0].iova = buf->daddr;
#endif
			width = node->active_fmt.fmt.pix_mp.width;
			height = node->active_fmt.fmt.pix_mp.height;
			stride = node->active_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
#if PDAF_READY
			vaddr = vb2_plane_vaddr(vb, 0);
			mtk_cam_set_sv_meta_stats_info(
				node->desc.dma_port, vaddr, width, height, stride);
#endif
		}
		break;
	/*
	 * XAGA: raw image outputs (IMGO + YUVO_1..5).  The request-carrying
	 * path fills these in mtk_cam_req_update() -> mtk_cam_config_raw_img_out()
	 * / mtk_cam_config_raw_img_fmt(); the request-less path had no case, so
	 * frame_params.img_outs[] stayed all-zero.  The userspace CCD composer
	 * (libccd) gates every DMA branch on
	 * img_outs[i].buf[0][0].iova != 0 (mask bits 10..14 for yuvo1..5) and
	 * takes the ipi-fmt from img_outs[i].fmt.format, so without this it
	 * composes only the TG defaults and the YUV DMA never writes.
	 */
	case MTKCAM_IPI_RAW_IMGO:
	case MTKCAM_IPI_RAW_YUVO_1:
	case MTKCAM_IPI_RAW_YUVO_2:
	case MTKCAM_IPI_RAW_YUVO_3:
	case MTKCAM_IPI_RAW_YUVO_4:
	case MTKCAM_IPI_RAW_YUVO_5:
	case MTKCAM_IPI_RAW_RZH1N2TO_1:
	case MTKCAM_IPI_RAW_RZH1N2TO_2:
	case MTKCAM_IPI_RAW_RZH1N2TO_3:
	case MTKCAM_IPI_RAW_DRZS4NO_1:
	case MTKCAM_IPI_RAW_DRZS4NO_2:
	case MTKCAM_IPI_RAW_DRZS4NO_3:
	{
		struct mtkcam_ipi_img_output *img_out;

		/* See p_yuv_outs: leaving img_outs[] zero for these ports stops
		 * libccd composing the extra CAM_YUV DRAM branches. */
		if (!p_yuv_outs) {
			dev_info(dev,
				 "XAGA-NRFILL: port=%d left to libccd default (p_yuv_outs=0)\n",
				 dma_port);
			break;
		}

		desc_id = node->desc.id - MTK_RAW_SOURCE_BEGIN;
		img_out = &frame_param->img_outs[desc_id];
		img_out->uid.pipe_id = node->uid.pipe_id;
		img_out->uid.id = dma_port;
		img_out->buf[0][0].ccd_fd = vb->planes[0].m.fd;
		img_out->buf[0][0].size =
			node->active_fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
		img_out->buf[0][0].iova = buf->daddr;
		img_out->fmt.format = mtk_cam_get_img_fmt(
			node->active_fmt.fmt.pix_mp.pixelformat);
		img_out->fmt.s.w = node->active_fmt.fmt.pix_mp.width;
		img_out->fmt.s.h = node->active_fmt.fmt.pix_mp.height;
		img_out->fmt.stride[0] =
			node->active_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
		if (img_out->crop.s.w == 0) {
			img_out->crop.p.x = 0;
			img_out->crop.p.y = 0;
			img_out->crop.s.w = node->active_fmt.fmt.pix_mp.width;
			img_out->crop.s.h = node->active_fmt.fmt.pix_mp.height;
		}
		xaga_p1_probe_take(vb->planes[0].m.fd);
		dev_info(dev,
			 "XAGA-NRFILL pipe=%d port=%d id=%d iova=0x%llx size=%u fmt=%d %ux%u str=%u\n",
			 pipe_id, dma_port, desc_id, (u64)buf->daddr,
			 img_out->buf[0][0].size, img_out->fmt.format,
			 img_out->fmt.s.w, img_out->fmt.s.h,
			 img_out->fmt.stride[0]);
		break;
	}
#if MRAW_READY
	case MTKCAM_IPI_MRAW_META_STATS_CFG:
	case MTKCAM_IPI_MRAW_META_STATS_0:
		mtk_cam_mraw_handle_enque(vb);
		break;
#endif
	default:
		dev_dbg(dev, "%s:pipe(%d):buffer with invalid port(%d)\n",
			__func__, pipe_id, dma_port);
		break;
	}

	/*
	 * Request-less userspace: re-queued buffers (frames after the
	 * initial burst) must rejoin the camsv using list, which normally
	 * happens at request enqueue time in the HAL flow.
	 *
	 * Raw pipelines have no analogous per-frame enqueue: the raw state
	 * machine drives the CQ double-buffer load off camsys_state_list,
	 * so the state element must be re-registered here as well
	 * (mtk_cam_no_req_get already reset it to E_STATE_READY).
	 */
	if (is_no_req && (is_camsv_subdev(pipe_id) || is_raw_subdev(pipe_id))) {
		struct mtk_cam_ctx *ctx = mtk_cam_find_ctx(cam, &node->vdev.entity);

		if (ctx && ctx->streaming) {
			if (is_camsv_subdev(pipe_id))
				mtk_cam_sv_req_enqueue(ctx, req, 0);
			/* register the state element for the next frame directly */
			mtk_cam_no_req_register_state(ctx, req, pipe_id);
		}
	}

}

static void mtk_cam_vb2_buf_cleanup(struct vb2_buffer *vb)
{
	struct mtk_cam_video_device *node = mtk_cam_vbq_to_vdev(vb->vb2_queue);
	struct mtk_cam_buffer *buf;
	struct device *dev = vb->vb2_queue->dev;

	if (!node->desc.smem_alloc)
		return;

	buf = mtk_cam_vb2_buf_to_dev_buf(vb);
	dma_unmap_page_attrs(dev, buf->daddr,
			     vb->planes[0].length,
			     DMA_BIDIRECTIONAL,
			     DMA_ATTR_SKIP_CPU_SYNC);
}

static void mtk_cam_vb2_request_complete(struct vb2_buffer *vb)
{
	struct mtk_cam_device *cam = vb2_get_drv_priv(vb->vb2_queue);

	dev_dbg(vb->vb2_queue->dev, "%s\n", __func__);

	v4l2_ctrl_request_complete(vb->req_obj.req,
				   cam->v4l2_dev.ctrl_handler);
}

static int mtk_cam_vb2_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (v4l2_buf->field == V4L2_FIELD_ANY)
			v4l2_buf->field = V4L2_FIELD_NONE;

		if (v4l2_buf->field != V4L2_FIELD_NONE)
			return -EINVAL;
	}
	return 0;
}

static const struct vb2_ops mtk_cam_vb2_ops = {
	.queue_setup = mtk_cam_vb2_queue_setup,


	.buf_out_validate = mtk_cam_vb2_buf_out_validate,
	.buf_init = mtk_cam_vb2_buf_init,
	.buf_prepare = mtk_cam_vb2_buf_prepare,
	.buf_finish = mtk_cam_vb2_buf_finish,
	.buf_cleanup = mtk_cam_vb2_buf_cleanup,

	.start_streaming = mtk_cam_vb2_start_streaming,
	.stop_streaming = mtk_cam_vb2_stop_streaming,

	.buf_queue = mtk_cam_vb2_buf_queue,
	.buf_request_complete = mtk_cam_vb2_request_complete,
};

static const struct v4l2_file_operations mtk_cam_v4l2_fops = {
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = v4l2_compat_ioctl32,
#endif
};

unsigned int mtk_cam_get_sensor_pixel_id(unsigned int fmt)
{
	switch (fmt & SENSOR_FMT_MASK) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR14_1X14:
		return MTKCAM_IPI_BAYER_PXL_ID_B;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
		return MTKCAM_IPI_BAYER_PXL_ID_GB;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
		return MTKCAM_IPI_BAYER_PXL_ID_GR;
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return MTKCAM_IPI_BAYER_PXL_ID_R;
	default:
		return MTKCAM_IPI_BAYER_PXL_ID_UNKNOWN;
	}
}

unsigned int mtk_cam_get_sensor_fmt(unsigned int fmt)
{
	switch (fmt & SENSOR_FMT_MASK) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		return MTKCAM_IPI_IMG_FMT_BAYER8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		return MTKCAM_IPI_IMG_FMT_BAYER10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		return MTKCAM_IPI_IMG_FMT_BAYER12;
	case MEDIA_BUS_FMT_SBGGR14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return MTKCAM_IPI_IMG_FMT_BAYER14;
	default:
		return MTKCAM_IPI_IMG_FMT_UNKNOWN;
	}
}

unsigned int mtk_cam_get_pixel_bits(unsigned int ipi_fmt)
{
	switch (ipi_fmt) {
	case MTKCAM_IPI_IMG_FMT_BAYER8:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8:
		return 8;
	case MTKCAM_IPI_IMG_FMT_BAYER10:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10:
	case MTKCAM_IPI_IMG_FMT_BAYER10_MIPI:
		return 10;
	case MTKCAM_IPI_IMG_FMT_BAYER12:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12:
		return 12;
	case MTKCAM_IPI_IMG_FMT_BAYER14:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER14:
		return 14;
	case MTKCAM_IPI_IMG_FMT_BAYER10_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER12_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER14_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER16:
	case MTKCAM_IPI_IMG_FMT_YUYV:
	case MTKCAM_IPI_IMG_FMT_YVYU:
	case MTKCAM_IPI_IMG_FMT_UYVY:
	case MTKCAM_IPI_IMG_FMT_VYUY:
		return 16;
	case MTKCAM_IPI_IMG_FMT_Y8:
	case MTKCAM_IPI_IMG_FMT_YUV_422_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_422_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_3P:
	case MTKCAM_IPI_IMG_FMT_YUV_420_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_420_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_3P:
		return 8;
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210:
		return 32;
	case MTKCAM_IPI_IMG_FMT_YUV_P210:
	case MTKCAM_IPI_IMG_FMT_YVU_P210:
	case MTKCAM_IPI_IMG_FMT_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_YUV_P212:
	case MTKCAM_IPI_IMG_FMT_YVU_P212:
	case MTKCAM_IPI_IMG_FMT_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_YVU_P012:
		return 16;
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210_PACKED:
		return 20;
	case MTKCAM_IPI_IMG_FMT_YUV_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P010_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P010_PACKED:
		return 10;
	case MTKCAM_IPI_IMG_FMT_YUV_P212_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P212_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P012_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P012_PACKED:
		return 12;
	case MTKCAM_IPI_IMG_FMT_RGB_8B_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8_3P:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV12:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV21:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER8:
		return 8;
	case MTKCAM_IPI_IMG_FMT_RGB_10B_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER10:
		return 10;
	case MTKCAM_IPI_IMG_FMT_RGB_12B_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER12:
		return 12;
	case MTKCAM_IPI_IMG_FMT_RGB_10B_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P:
	case MTKCAM_IPI_IMG_FMT_RGB_12B_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P:
		return 16;

	default:
		break;
	}
	pr_debug("not supported ipi-fmt 0x%08x", ipi_fmt);

	return -1;
}

unsigned int mtk_cam_get_img_fmt(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_GREY:
		return MTKCAM_IPI_IMG_FMT_Y8;
	case V4L2_PIX_FMT_YUYV:
		return MTKCAM_IPI_IMG_FMT_YUYV;
	case V4L2_PIX_FMT_YVYU:
		return MTKCAM_IPI_IMG_FMT_YVYU;
	case V4L2_PIX_FMT_NV16:
		return MTKCAM_IPI_IMG_FMT_YUV_422_2P;
	case V4L2_PIX_FMT_NV61:
		return MTKCAM_IPI_IMG_FMT_YVU_422_2P;
	case V4L2_PIX_FMT_NV12:
		return MTKCAM_IPI_IMG_FMT_YUV_420_2P;
	case V4L2_PIX_FMT_NV21:
		return MTKCAM_IPI_IMG_FMT_YVU_420_2P;
	case V4L2_PIX_FMT_YUV422P:
		return MTKCAM_IPI_IMG_FMT_YUV_422_3P;
	case V4L2_PIX_FMT_YUV420:
		return MTKCAM_IPI_IMG_FMT_YUV_420_3P;
	case V4L2_PIX_FMT_YVU420:
		return MTKCAM_IPI_IMG_FMT_YVU_420_3P;
	case V4L2_PIX_FMT_NV12_10:
		return MTKCAM_IPI_IMG_FMT_YUV_P010;
	case V4L2_PIX_FMT_NV21_10:
		return MTKCAM_IPI_IMG_FMT_YVU_P010;
	case V4L2_PIX_FMT_NV16_10:
		return MTKCAM_IPI_IMG_FMT_YUV_P210;
	case V4L2_PIX_FMT_NV61_10:
		return MTKCAM_IPI_IMG_FMT_YVU_P210;
	case V4L2_PIX_FMT_MTISP_NV12_10P:
		return MTKCAM_IPI_IMG_FMT_YUV_P010_PACKED;
	case V4L2_PIX_FMT_MTISP_NV21_10P:
		return MTKCAM_IPI_IMG_FMT_YVU_P010_PACKED;
	case V4L2_PIX_FMT_MTISP_NV16_10P:
		return MTKCAM_IPI_IMG_FMT_YUV_P210_PACKED;
	case V4L2_PIX_FMT_MTISP_NV61_10P:
		return MTKCAM_IPI_IMG_FMT_YVU_P210_PACKED;
	case V4L2_PIX_FMT_YUYV10:
		return MTKCAM_IPI_IMG_FMT_YUYV_Y210;
	case V4L2_PIX_FMT_YVYU10:
		return MTKCAM_IPI_IMG_FMT_YVYU_Y210;
	case V4L2_PIX_FMT_UYVY10:
		return MTKCAM_IPI_IMG_FMT_UYVY_Y210;
	case V4L2_PIX_FMT_VYUY10:
		return MTKCAM_IPI_IMG_FMT_VYUY_Y210;
	case V4L2_PIX_FMT_MTISP_YUYV10P:
		return MTKCAM_IPI_IMG_FMT_YUYV_Y210_PACKED;
	case V4L2_PIX_FMT_MTISP_YVYU10P:
		return MTKCAM_IPI_IMG_FMT_YVYU_Y210_PACKED;
	case V4L2_PIX_FMT_MTISP_UYVY10P:
		return MTKCAM_IPI_IMG_FMT_UYVY_Y210_PACKED;
	case V4L2_PIX_FMT_MTISP_VYUY10P:
		return MTKCAM_IPI_IMG_FMT_VYUY_Y210_PACKED;
	case V4L2_PIX_FMT_NV12_12:
		return MTKCAM_IPI_IMG_FMT_YUV_P012;
	case V4L2_PIX_FMT_NV21_12:
		return MTKCAM_IPI_IMG_FMT_YVU_P012;
	case V4L2_PIX_FMT_NV16_12:
		return MTKCAM_IPI_IMG_FMT_YUV_P212;
	case V4L2_PIX_FMT_NV61_12:
		return MTKCAM_IPI_IMG_FMT_YVU_P212;
	case V4L2_PIX_FMT_MTISP_NV12_12P:
		return MTKCAM_IPI_IMG_FMT_YUV_P012_PACKED;
	case V4L2_PIX_FMT_MTISP_NV21_12P:
		return MTKCAM_IPI_IMG_FMT_YVU_P012_PACKED;
	case V4L2_PIX_FMT_MTISP_NV16_12P:
		return MTKCAM_IPI_IMG_FMT_YUV_P212_PACKED;
	case V4L2_PIX_FMT_MTISP_NV61_12P:
		return MTKCAM_IPI_IMG_FMT_YVU_P212_PACKED;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return MTKCAM_IPI_IMG_FMT_BAYER8;
	case V4L2_PIX_FMT_MTISP_SBGGR8F:
	case V4L2_PIX_FMT_MTISP_SGBRG8F:
	case V4L2_PIX_FMT_MTISP_SGRBG8F:
	case V4L2_PIX_FMT_MTISP_SRGGB8F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER8;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SRGGB10:
		return MTKCAM_IPI_IMG_FMT_BAYER10_UNPACKED;
	case V4L2_PIX_FMT_SBGGR10P:
	case V4L2_PIX_FMT_SGBRG10P:
	case V4L2_PIX_FMT_SGRBG10P:
	case V4L2_PIX_FMT_SRGGB10P:
		return MTKCAM_IPI_IMG_FMT_BAYER10_MIPI;
	case V4L2_PIX_FMT_MTISP_SBGGR10:
	case V4L2_PIX_FMT_MTISP_SGBRG10:
	case V4L2_PIX_FMT_MTISP_SGRBG10:
	case V4L2_PIX_FMT_MTISP_SRGGB10:
		return MTKCAM_IPI_IMG_FMT_BAYER10;
	case V4L2_PIX_FMT_MTISP_SBGGR10F:
	case V4L2_PIX_FMT_MTISP_SGBRG10F:
	case V4L2_PIX_FMT_MTISP_SGRBG10F:
	case V4L2_PIX_FMT_MTISP_SRGGB10F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER10;
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SRGGB12:
		return MTKCAM_IPI_IMG_FMT_BAYER12_UNPACKED;
	case V4L2_PIX_FMT_MTISP_SBGGR12:
	case V4L2_PIX_FMT_MTISP_SGBRG12:
	case V4L2_PIX_FMT_MTISP_SGRBG12:
	case V4L2_PIX_FMT_MTISP_SRGGB12:
		return MTKCAM_IPI_IMG_FMT_BAYER12;
	case V4L2_PIX_FMT_MTISP_SBGGR12F:
	case V4L2_PIX_FMT_MTISP_SGBRG12F:
	case V4L2_PIX_FMT_MTISP_SGRBG12F:
	case V4L2_PIX_FMT_MTISP_SRGGB12F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER12;
	case V4L2_PIX_FMT_SBGGR14:
	case V4L2_PIX_FMT_SGBRG14:
	case V4L2_PIX_FMT_SGRBG14:
	case V4L2_PIX_FMT_SRGGB14:
		return MTKCAM_IPI_IMG_FMT_BAYER14_UNPACKED;
	case V4L2_PIX_FMT_MTISP_SBGGR14:
	case V4L2_PIX_FMT_MTISP_SGBRG14:
	case V4L2_PIX_FMT_MTISP_SGRBG14:
	case V4L2_PIX_FMT_MTISP_SRGGB14:
		return MTKCAM_IPI_IMG_FMT_BAYER14;
	case V4L2_PIX_FMT_MTISP_SBGGR14F:
	case V4L2_PIX_FMT_MTISP_SGBRG14F:
	case V4L2_PIX_FMT_MTISP_SGRBG14F:
	case V4L2_PIX_FMT_MTISP_SRGGB14F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER14;
	case V4L2_PIX_FMT_SBGGR16:
	case V4L2_PIX_FMT_SGBRG16:
	case V4L2_PIX_FMT_SGRBG16:
	case V4L2_PIX_FMT_SRGGB16:
		return MTKCAM_IPI_IMG_FMT_BAYER16;
	case V4L2_PIX_FMT_MTISP_NV12_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_NV12;
	case V4L2_PIX_FMT_MTISP_NV21_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_NV21;
	case V4L2_PIX_FMT_MTISP_NV12_10_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_YUV_P010;
	case V4L2_PIX_FMT_MTISP_NV21_10_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_YVU_P010;
	case V4L2_PIX_FMT_MTISP_NV12_12_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_YUV_P012;
	case V4L2_PIX_FMT_MTISP_NV21_12_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_YVU_P012;
	case V4L2_PIX_FMT_MTISP_BAYER8_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_BAYER8;
	case V4L2_PIX_FMT_MTISP_BAYER10_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_BAYER10;
	case V4L2_PIX_FMT_MTISP_BAYER12_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_BAYER12;
	case V4L2_PIX_FMT_MTISP_BAYER14_UFBC:
		return MTKCAM_IPI_IMG_FMT_UFBC_BAYER14;
	case V4L2_PIX_FMT_MTISP_SGRB8F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER8_3P;
	case V4L2_PIX_FMT_MTISP_SGRB10F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P_PACKED;
	case V4L2_PIX_FMT_MTISP_SGRB12F:
		return MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P_PACKED;
	default:
		return MTKCAM_IPI_IMG_FMT_UNKNOWN;
	}
}
/* for mmqos and base is 100 */
int mtk_cam_get_fmt_size_factor(unsigned int ipi_fmt)
{
	switch (ipi_fmt) {
	case MTKCAM_IPI_IMG_FMT_BAYER8:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8:
	case MTKCAM_IPI_IMG_FMT_BAYER10:
	case MTKCAM_IPI_IMG_FMT_BAYER10_MIPI:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10:
	case MTKCAM_IPI_IMG_FMT_BAYER12:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12:
	case MTKCAM_IPI_IMG_FMT_BAYER14:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER14:
	case MTKCAM_IPI_IMG_FMT_BAYER16:
	case MTKCAM_IPI_IMG_FMT_BAYER10_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER12_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER14_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER8:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER10:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER12:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER14:
	case MTKCAM_IPI_IMG_FMT_YUYV:
	case MTKCAM_IPI_IMG_FMT_YVYU:
	case MTKCAM_IPI_IMG_FMT_UYVY:
	case MTKCAM_IPI_IMG_FMT_VYUY:
	case MTKCAM_IPI_IMG_FMT_Y8:
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210:
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210_PACKED:
		return 100;
	case MTKCAM_IPI_IMG_FMT_YUV_420_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_420_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_3P:
	case MTKCAM_IPI_IMG_FMT_YUV_P010_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P010_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P012_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P012_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_YVU_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV12:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV21:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P012:
		return 150;
	case MTKCAM_IPI_IMG_FMT_YUV_422_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_422_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_3P:
	case MTKCAM_IPI_IMG_FMT_YUV_P210:
	case MTKCAM_IPI_IMG_FMT_YVU_P210:
	case MTKCAM_IPI_IMG_FMT_YUV_P212:
	case MTKCAM_IPI_IMG_FMT_YVU_P212:
	case MTKCAM_IPI_IMG_FMT_YUV_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P212_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P212_PACKED:
		return 200;
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_RGB_8B_3P:
	case MTKCAM_IPI_IMG_FMT_RGB_10B_3P:
	case MTKCAM_IPI_IMG_FMT_RGB_12B_3P:
	case MTKCAM_IPI_IMG_FMT_RGB_10B_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_RGB_12B_3P_PACKED:
		return 300;
	default:
		pr_info("%s unsupport format(%d)", __func__, ipi_fmt);
		break;
	}
	return 100;
}

int mtk_cam_fill_pixfmt_mp(struct v4l2_pix_format_mplane *pixfmt,
			u32 pixelformat, u32 width, u32 height)
{
	struct v4l2_plane_pix_format *plane;
	unsigned int ipi_fmt = mtk_cam_get_img_fmt(pixelformat);
	u8 pixel_bits = mtk_cam_get_pixel_bits(ipi_fmt);
	u32 stride;
	u32 aligned_width;
	u8 pixel_mode_shift = 0; /* todo: should set by resMgr */
	u8 bus_size;
	u8 i;

	pixfmt->width = width;
	pixfmt->height = height;
	pixfmt->pixelformat = pixelformat;
	plane = &pixfmt->plane_fmt[0];
	bus_size = mtk_cam_yuv_dma_bus_size(pixel_bits, pixel_mode_shift);
	plane->sizeimage = 0;

	if (is_mtk_format(pixelformat)) {
		const struct mtk_format_info *info;

		info = mtk_format_info(pixelformat);
		pixfmt->num_planes = info->mem_planes;

		if (!info)
			return -EINVAL;

		if (info->mem_planes == 1) {
			if (is_yuv_ufo(pixelformat)) {
				/* UFO format width should align 64 pixel */
				aligned_width = ALIGN(width, 64);
				stride = aligned_width * info->bit_r_num / info->bit_r_den;

				if (stride > plane->bytesperline)
					plane->bytesperline = stride;
				plane->sizeimage = stride * height;
				plane->sizeimage += stride * height / 2;
				plane->sizeimage += ALIGN((aligned_width / 64), 8) * height;
				plane->sizeimage += ALIGN((aligned_width / 64), 8) * height / 2;
				plane->sizeimage += sizeof(struct UfbcBufferHeader);
			} else if (is_raw_ufo(pixelformat)) {
				/* UFO format width should align 64 pixel */
				aligned_width = ALIGN(width, 64);
				stride = aligned_width * info->bit_r_num / info->bit_r_den;

				if (stride > plane->bytesperline)
					plane->bytesperline = stride;
				plane->sizeimage = stride * height;
				plane->sizeimage += ALIGN((aligned_width / 64), 8) * height;
				plane->sizeimage += sizeof(struct UfbcBufferHeader);
			} else {
				/* width should be bus_size align */
				aligned_width = ALIGN(DIV_ROUND_UP(width
					* info->bit_r_num, info->bit_r_den), bus_size);
				stride = aligned_width * info->bpp[0];

				if (stride > plane->bytesperline)
					plane->bytesperline = stride;

				for (i = 0; i < info->comp_planes; i++) {
					unsigned int hdiv = (i == 0) ? 1 : info->hdiv;
					unsigned int vdiv = (i == 0) ? 1 : info->vdiv;
					if (plane->bytesperline > stride) {
						if (is_fullg_rb(pixelformat)) {
							plane->sizeimage +=
							DIV_ROUND_UP(plane->bytesperline, hdiv)
							* DIV_ROUND_UP(height, vdiv);
						} else {
							plane->sizeimage += plane->bytesperline
							* DIV_ROUND_UP(height, vdiv);
						}
					} else {
						plane->sizeimage += info->bpp[i]
						* DIV_ROUND_UP(aligned_width, hdiv)
						* DIV_ROUND_UP(height, vdiv);
					}
				}
			}
			pr_debug("%s stride %d sizeimage %d\n", __func__,
				plane->bytesperline, plane->sizeimage);
		} else {
			pr_debug("do not support non contiguous mplane\n");
		}
	} else {
		const struct v4l2_format_info *info;

		pr_debug("pixelformat:0x%x sizeimage:%d\n", pixelformat, plane->sizeimage);
		info = v4l2_format_info(pixelformat);
		pixfmt->num_planes = info->mem_planes;

		if (!info)
			return -EINVAL;

		if (info->mem_planes == 1) {

			aligned_width = ALIGN(width, bus_size);
			stride = aligned_width * info->bpp[0];
			if (stride > plane->bytesperline)
				plane->bytesperline = stride;

			for (i = 0; i < info->comp_planes; i++) {
				unsigned int hdiv = (i == 0) ? 1 : info->hdiv;
				unsigned int vdiv = (i == 0) ? 1 : info->vdiv;

				plane->sizeimage += info->bpp[i]
					* DIV_ROUND_UP(aligned_width, hdiv)
					* DIV_ROUND_UP(height, vdiv);
			}
			pr_debug("%s stride %d sizeimage %d\n", __func__,
				plane->bytesperline, plane->sizeimage);
		} else {
			pr_debug("do not support non contiguous mplane\n");
		}
	}

	return 0;
}

static void cal_image_pix_mp(unsigned int node_id,
			     struct v4l2_pix_format_mplane *mp,
			     unsigned int pixel_mode)
{
	unsigned int ipi_fmt = mtk_cam_get_img_fmt(mp->pixelformat);
	unsigned int width = mp->width;
	unsigned int height = mp->height;
	unsigned int stride, i;

	pr_debug("fmt:0x%x ipi_fmt:%d\n", mp->pixelformat, ipi_fmt);
	switch (ipi_fmt) {
	case MTKCAM_IPI_IMG_FMT_BAYER8:
	case MTKCAM_IPI_IMG_FMT_BAYER10:
	case MTKCAM_IPI_IMG_FMT_BAYER12:
	case MTKCAM_IPI_IMG_FMT_BAYER14:
	case MTKCAM_IPI_IMG_FMT_BAYER16:
	case MTKCAM_IPI_IMG_FMT_BAYER10_MIPI:
	case MTKCAM_IPI_IMG_FMT_BAYER10_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER12_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_BAYER14_UNPACKED:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER14:
		stride = mtk_cam_dmao_xsize(width, ipi_fmt, pixel_mode);
		for (i = 0; i < mp->num_planes; i++) {
			if (stride > mp->plane_fmt[i].bytesperline)
				mp->plane_fmt[i].bytesperline = stride;
			mp->plane_fmt[i].sizeimage = mp->plane_fmt[i].bytesperline * height;
		}
	break;
	case MTKCAM_IPI_IMG_FMT_YUYV:
	case MTKCAM_IPI_IMG_FMT_YVYU:
	case MTKCAM_IPI_IMG_FMT_UYVY:
	case MTKCAM_IPI_IMG_FMT_VYUY:
	case MTKCAM_IPI_IMG_FMT_YUV_422_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_422_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_422_3P:
	case MTKCAM_IPI_IMG_FMT_YUV_420_2P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_2P:
	case MTKCAM_IPI_IMG_FMT_YUV_420_3P:
	case MTKCAM_IPI_IMG_FMT_YVU_420_3P:
	case MTKCAM_IPI_IMG_FMT_Y8:
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210:
	case MTKCAM_IPI_IMG_FMT_YUYV_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVYU_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_UYVY_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_VYUY_Y210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P210:
	case MTKCAM_IPI_IMG_FMT_YVU_P210:
	case MTKCAM_IPI_IMG_FMT_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_YUV_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P210_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P010_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P010_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P212:
	case MTKCAM_IPI_IMG_FMT_YVU_P212:
	case MTKCAM_IPI_IMG_FMT_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_YVU_P012:
	case MTKCAM_IPI_IMG_FMT_YUV_P212_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P212_PACKED:
	case MTKCAM_IPI_IMG_FMT_YUV_P012_PACKED:
	case MTKCAM_IPI_IMG_FMT_YVU_P012_PACKED:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV12:
	case MTKCAM_IPI_IMG_FMT_UFBC_NV21:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P010:
	case MTKCAM_IPI_IMG_FMT_UFBC_YUV_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_YVU_P012:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER8:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER10:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER12:
	case MTKCAM_IPI_IMG_FMT_UFBC_BAYER14:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER8_3P:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER10_3P_PACKED:
	case MTKCAM_IPI_IMG_FMT_FG_BAYER12_3P_PACKED:
		mtk_cam_fill_pixfmt_mp(mp, mp->pixelformat, width, height);
	default:
		break;
	}
}

static int mtk_video_init_format(struct mtk_cam_video_device *video)
{
	struct mtk_cam_dev_node_desc *desc = &video->desc;
	struct v4l2_format *active = &video->active_fmt;
	const struct v4l2_format *default_fmt =
		&desc->fmts[desc->default_fmt_idx].vfmt;

	memset(active, 0, sizeof(*active));
	active->type = desc->buf_type;

	if (!desc->image) {
		active->fmt.meta.dataformat = default_fmt->fmt.meta.dataformat;
		active->fmt.meta.buffersize = default_fmt->fmt.meta.buffersize;
		return 0;
	}

	active->fmt.pix_mp.pixelformat = default_fmt->fmt.pix_mp.pixelformat;
	active->fmt.pix_mp.width = default_fmt->fmt.pix_mp.width;
	active->fmt.pix_mp.height = default_fmt->fmt.pix_mp.height;
	active->fmt.pix_mp.num_planes = default_fmt->fmt.pix_mp.num_planes;

	/**
	 * Use the same pixel mode as s_fmt/try_fmt so that g_fmt always
	 * agrees with them (camsv main out uses pixel mode 3).
	 */
	cal_image_pix_mp(desc->id, &active->fmt.pix_mp,
			 desc->dma_port == MTKCAM_IPI_CAMSV_MAIN_OUT ? 3 : 0);

	/**
	 * TODO: to support multi-plane: for example, yuv or do it as
	 * following?
	 */
	active->fmt.pix_mp.num_planes = 1;
	active->fmt.pix_mp.colorspace = V4L2_COLORSPACE_SRGB;
	active->fmt.pix_mp.field = V4L2_FIELD_NONE;
	active->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	active->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	active->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_SRGB;

	return 0;
}

int mtk_cam_video_register(struct mtk_cam_video_device *video,
			   struct v4l2_device *v4l2_dev)
{
	struct mtk_cam_device *cam =
		container_of(v4l2_dev, struct mtk_cam_device, v4l2_dev);
	struct media_pad *pad = &video->pad;
	struct video_device *vdev = &video->vdev;
	struct vb2_queue *q = &video->vb2_q;
	unsigned int output = V4L2_TYPE_IS_OUTPUT(video->desc.buf_type);
	int ret;

	if (video->desc.link_flags & MEDIA_LNK_FL_ENABLED)
		video->enabled = true;
	else
		video->enabled = false;

	mutex_init(&video->q_lock);

	/* initialize vb2_queue */
	q->type = video->desc.buf_type;
	q->io_modes = VB2_MMAP | VB2_DMABUF;

	if (q->type == V4L2_BUF_TYPE_META_OUTPUT || video->desc.id == MTK_RAW_META_OUT_1)
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	else
		/**
		 * The ISP stamps buffers with a monotonic clock
		 * (s_data->timestamp_mono). BOOT is not a valid V4L2
		 * timestamp class, so declare MONOTONIC as upstream wants.
		 */
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	if (video->desc.smem_alloc) {
		q->bidirectional = 1;
		/* q->dev = cam->smem_dev; FIXME impl for real SCP */
		q->dev = cam->dev;
	} else {
		if (video->uid.pipe_id >= MTKCAM_SUBDEV_CAMSV_START &&
			video->uid.pipe_id < MTKCAM_SUBDEV_CAMSV_END) {
			switch (video->uid.pipe_id) {
			case MTKCAM_SUBDEV_CAMSV_0:
			case MTKCAM_SUBDEV_CAMSV_1:
			case MTKCAM_SUBDEV_CAMSV_4:
			case MTKCAM_SUBDEV_CAMSV_5:
			case MTKCAM_SUBDEV_CAMSV_8:
			case MTKCAM_SUBDEV_CAMSV_9:
			case MTKCAM_SUBDEV_CAMSV_12:
			case MTKCAM_SUBDEV_CAMSV_13:
				q->dev = find_larb(&cam->larb, 13);
				break;
			default:
				q->dev = find_larb(&cam->larb, 14);
				break;
			}
		} else if (video->uid.pipe_id >= MTKCAM_SUBDEV_MRAW_START &&
			video->uid.pipe_id < MTKCAM_SUBDEV_MRAW_END) {
			switch (video->uid.pipe_id) {
			case MTKCAM_SUBDEV_MRAW_0:
			case MTKCAM_SUBDEV_MRAW_2:
				q->dev = find_larb(&cam->larb, 25);
				break;
			default:
				q->dev = find_larb(&cam->larb, 26);
				break;
			}
		} else {
			switch (video->desc.id) {
			case MTK_RAW_YUVO_1_OUT:
			case MTK_RAW_YUVO_2_OUT:
			case MTK_RAW_YUVO_3_OUT:
			case MTK_RAW_YUVO_4_OUT:
			case MTK_RAW_YUVO_5_OUT:
			case MTK_RAW_DRZS4NO_1_OUT:
			case MTK_RAW_DRZS4NO_2_OUT:
			case MTK_RAW_DRZS4NO_3_OUT:
			case MTK_RAW_RZH1N2TO_1_OUT:
			case MTK_RAW_RZH1N2TO_2_OUT:
			case MTK_RAW_RZH1N2TO_3_OUT:
				/* should have a better implementation here */
				q->dev = cam->raw.yuvs[0];
				break;
			default:
				q->dev = cam->raw.devs[0];
				break;
			}
		}
	}

	q->supports_requests = true;
	q->lock = &video->q_lock;
	q->ops = &mtk_cam_vb2_ops;
	q->mem_ops = &mtk_cam_dma_contig_memops;
	q->drv_priv = cam;
	q->buf_struct_size = sizeof(struct mtk_cam_buffer);

	if (output)
		q->timestamp_flags |= V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	else
		q->timestamp_flags |= V4L2_BUF_FLAG_TSTAMP_SRC_SOE;

	/* No minimum buffers limitation */
	q->min_queued_buffers = 0;

	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_info(v4l2_dev->dev, "Failed to init vb2 queue: %d\n", ret);
		goto error_vb2_init;
	}

	pad->flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, pad);
	if (ret < 0) {
		dev_info(v4l2_dev->dev, "Failed to init video entity: %d\n",
			ret);
		goto error_media_init;
	}

	ret = mtk_video_init_format(video);
	if (ret < 0) {
		dev_info(v4l2_dev->dev, "Failed to init format: %d\n", ret);
		goto error_video_register;
	}

	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	vdev->entity.ops = NULL;
	vdev->fops = &mtk_cam_v4l2_fops;
	vdev->device_caps = video->desc.cap | V4L2_CAP_STREAMING;
	/*
	 * V4L2_CAP_IO_MC makes userspace expect the formats to be filtered by
	 * media bus code.  Only the CAMSV-only graph does that; the vendor HAL
	 * relies on the unfiltered list.
	 */
	if (cam->camsv_only)
		vdev->device_caps |= V4L2_CAP_IO_MC;
	vdev->v4l2_dev = v4l2_dev;

	vdev->vfl_dir = output ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->queue = &video->vb2_q;
	vdev->ioctl_ops = video->desc.ioctl_ops;
	vdev->release = video_device_release_empty;
	/* TODO: share q_lock or use another lock? */
	vdev->lock = &video->q_lock;
	strscpy(vdev->name, video->desc.name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_info(v4l2_dev->dev, "Failed to register video device: %d\n",
			ret);
		goto error_video_register;
	}
	video_set_drvdata(vdev, cam);

	dev_dbg(v4l2_dev->dev, "registered vdev:%d:%s\n",
		video->desc.id, vdev->name);

	return 0;

error_video_register:
	media_entity_cleanup(&vdev->entity);
error_media_init:
	vb2_queue_release(&video->vb2_q);
error_vb2_init:
	mutex_destroy(&video->q_lock);

	return ret;
}

void mtk_cam_video_unregister(struct mtk_cam_video_device *video)
{
	video_unregister_device(&video->vdev);
	vb2_queue_release(&video->vb2_q);
	media_entity_cleanup(&video->vdev.entity);
	mutex_destroy(&video->q_lock);
}

const struct v4l2_format *
mtk_cam_dev_find_fmt(struct mtk_cam_dev_node_desc *desc, u32 format)
{
	unsigned int i;
	const struct v4l2_format *fmt;

	for (i = 0; i < desc->num_fmts; i++) {
		fmt = &desc->fmts[i].vfmt;
		if (fmt->fmt.pix_mp.pixelformat == format)
			return fmt;
	}

	return NULL;
}

int mtk_cam_vidioc_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	struct mtk_cam_device *cam = video_drvdata(file);

	strscpy(cap->driver, dev_driver_string(cam->dev), sizeof(cap->driver));
	strscpy(cap->card, dev_driver_string(cam->dev), sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(cam->dev));

	return 0;
}

/* Single virtual camera input; needed by ffmpeg's v4l2 demuxer */
int mtk_cam_vidioc_enum_input(struct file *file, void *fh,
			      struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strscpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	input->audioset = 0;
	input->tuner = 0;
	input->std = 0;
	input->status = 0;
	input->capabilities = 0;

	return 0;
}

int mtk_cam_vidioc_g_input(struct file *file, void *fh,
			   unsigned int *index)
{
	*index = 0;

	return 0;
}

int mtk_cam_vidioc_s_input(struct file *file, void *fh,
			   unsigned int index)
{
	return index ? -EINVAL : 0;
}

/* Streaming parameters: report the sensor's 30 fps frame rate */
int mtk_cam_vidioc_g_parm(struct file *file, void *fh,
			  struct v4l2_streamparm *a)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);

	if (a->type != node->desc.buf_type)
		return -EINVAL;

	memset(&a->parm, 0, sizeof(a->parm));
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = 30;

	return 0;
}

int mtk_cam_vidioc_s_parm(struct file *file, void *fh,
			  struct v4l2_streamparm *a)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);
	struct v4l2_fract *tpf;
	unsigned int fps;

	if (a->type != node->desc.buf_type)
		return -EINVAL;

	tpf = &a->parm.capture.timeperframe;
	if (!tpf->denominator)
		return -EINVAL;
	fps = tpf->denominator / max(tpf->numerator, 1u);
	if (fps < 1)
		fps = 1;
	if (fps > 120)
		fps = 120;

	/* accepted but not applied: the sensor frame rate is fixed */
	tpf->numerator = 1;
	tpf->denominator = fps;
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

	return 0;
}

int mtk_cam_vidioc_enum_framesizes(struct file *filp, void *priv,
				   struct v4l2_frmsizeenum *sizes)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(filp);
	const struct v4l2_format *dev_fmt;

	dev_fmt = mtk_cam_dev_find_fmt(&node->desc, sizes->pixel_format);
	if (!dev_fmt || sizes->index)
		return -EINVAL;

	sizes->type = node->desc.frmsizes->type;
	memcpy(&sizes->stepwise, &node->desc.frmsizes->stepwise,
	       sizeof(sizes->stepwise));
	return 0;
}

static bool mtk_cam_fmt_matches_mbus(unsigned int sensor_fmt,
				     unsigned int img_fmt)
{
	if (img_fmt == sensor_fmt)
		return true;

	/*
	 * The packed ("CSI2P") Bayer fourccs the standard V4L2 userspace
	 * knows map to the *_MIPI / *_UNPACKED IPI variants of the same bit
	 * depth, not to the MTK-private MTISP fourccs.
	 */
	switch (sensor_fmt) {
	case MTKCAM_IPI_IMG_FMT_BAYER10:
		return img_fmt == MTKCAM_IPI_IMG_FMT_BAYER10_MIPI ||
		       img_fmt == MTKCAM_IPI_IMG_FMT_BAYER10_UNPACKED;
	case MTKCAM_IPI_IMG_FMT_BAYER12:
		return img_fmt == MTKCAM_IPI_IMG_FMT_BAYER12_UNPACKED;
	default:
		return false;
	}
}

int mtk_cam_vidioc_enum_fmt(struct file *file, void *fh,
			    struct v4l2_fmtdesc *f)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);
	struct mtk_cam_device *cam =
		container_of(node->vdev.v4l2_dev, struct mtk_cam_device,
			     v4l2_dev);
	unsigned int sensor_fmt = 0;
	unsigned int i, index = 0;

	/*
	 * In the CAMSV-only graph the node advertises V4L2_CAP_IO_MC, so
	 * userspace (libcamera) asks for the formats matching a media bus code.
	 * Filter by the matching IPI format; without this every Bayer order and
	 * bit depth is reported for any code.  The vendor HAL depends on the
	 * unfiltered list, so leave sensor_fmt at 0 for it.
	 */
	if (cam->camsv_only && f->mbus_code)
		sensor_fmt = mtk_cam_get_sensor_fmt(f->mbus_code);

	for (i = 0; i < node->desc.num_fmts; i++) {
		unsigned int pf = node->desc.fmts[i].vfmt.fmt.pix_mp.pixelformat;

		if (sensor_fmt &&
		    !mtk_cam_fmt_matches_mbus(sensor_fmt,
					      mtk_cam_get_img_fmt(pf)))
			continue;

		if (index++ != f->index)
			continue;

		/*
		 * Mainline v4l_fill_fmtdesc() only knows the upstream 4CCs;
		 * every MTK-private format (33 of them) falls into its default
		 * branch and emits a WARN plus a full stack trace.
		 * camerahalserver enumerates them on every camera open, which
		 * floods the 2 MB ring buffer.  v4l_fill_fmtdesc() returns
		 * early once description is set, so fill it here as the V4L2
		 * API expects a driver to do for private formats.
		 */
		f->pixelformat = pf;
		f->flags = 0;
		snprintf(f->description, sizeof(f->description), "%p4cc",
			 &f->pixelformat);
		return 0;
	}

	return -EINVAL;
}

int mtk_cam_vidioc_g_fmt(struct file *file, void *fh,
			 struct v4l2_format *f)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);

	f->fmt = node->active_fmt.fmt;

	return 0;
}

int mtk_cam_vidioc_s_fmt(struct file *file, void *fh,
			 struct v4l2_format *f)
{
	struct mtk_cam_device *cam = video_drvdata(file);
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);
	struct mtk_cam_request *cam_req;
	struct media_request *req;
	struct v4l2_format *vfmt;
	struct mtk_cam_request_stream_data *stream_data;
	s32 fd;
	struct mtk_raw_pipeline *raw_pipeline;
	int raw_feature = 0;

	if (!vb2_is_busy(node->vdev.queue)) {
		/* Get the valid format */
		raw_pipeline = mtk_cam_dev_get_raw_pipeline(cam, node->uid.pipe_id);
		if (raw_pipeline)
			raw_feature = raw_pipeline->user_res.raw_res.feature;

		mtk_cam_video_set_fmt(node, f, raw_feature);

		/* Configure to video device */
		node->active_fmt = *f;
		/* XAGA trace: map HAL stream -> camsys node (size/format) */
		pr_info("XAGA-VFMT %s fmt=0x%x %ux%u nplanes=%u\n",
			node->desc.name, f->fmt.pix_mp.pixelformat,
			f->fmt.pix_mp.width, f->fmt.pix_mp.height,
			f->fmt.pix_mp.num_planes);
		return 0;
	}

	fd = mtk_cam_fmt_get_request(&f->fmt.pix_mp);
	if (fd < 0)
		return -EINVAL;

	req = media_request_get_by_fd(&cam->media_dev, fd);
	if (IS_ERR(req)) {
		dev_info(cam->dev,
			"%s:pipe(%d):%s:invalid request_fd:%d\n",
			__func__, node->uid.pipe_id, node->desc.name, fd);

		return -EINVAL;
	}

	cam_req = to_mtk_cam_req(req);
	dev_dbg(cam->dev,
		"%s:%s:pipe(%d):%s:pending s_fmt: pixelfmt(0x%x), w(%d), h(%d)\n",
		__func__, cam_req->req.debug_str, node->uid.pipe_id, node->desc.name,
		f->fmt.pix_mp.pixelformat,  f->fmt.pix_mp.width, f->fmt.pix_mp.height);

	stream_data = mtk_cam_req_get_s_data_no_chk(cam_req, node->uid.pipe_id, 0);
	stream_data->vdev_fmt_update |= (1 << node->desc.id);
	vfmt = mtk_cam_s_data_get_vfmt(stream_data, node->desc.id);
	*vfmt = *f;
	media_request_put(req);

	return 0;
}

int mtk_cam_video_set_fmt(struct mtk_cam_video_device *node, struct v4l2_format *f, int raw_feature)
{
	struct mtk_cam_device *cam = video_get_drvdata(&node->vdev);
	const struct v4l2_format *dev_fmt;
	struct v4l2_format try_fmt;
	s32 request_fd, i;
	u32 bytesperline, sizeimage;
	u32 is_hdr = 0, is_hdr_m2m = 0;

	dev_dbg(cam->dev,
			"%s:pipe(%d):%s:feature(0x%x)\n",
			__func__, node->uid.pipe_id, node->desc.name, raw_feature);

	request_fd = mtk_cam_fmt_get_request(&f->fmt.pix_mp);
	memset(&try_fmt, 0, sizeof(try_fmt));
	try_fmt.type = f->type;

	/* Validate pixelformat */
	dev_fmt = mtk_cam_dev_find_fmt(&node->desc, f->fmt.pix_mp.pixelformat);
	if (!dev_fmt) {
		dev_dbg(cam->dev, "unknown fmt:%d\n",
			f->fmt.pix_mp.pixelformat);
		dev_fmt = &node->desc.fmts[node->desc.default_fmt_idx].vfmt;
	}
	try_fmt.fmt.pix_mp.pixelformat = dev_fmt->fmt.pix_mp.pixelformat;

	/* Validate image width & height range */
	try_fmt.fmt.pix_mp.width = clamp_val(f->fmt.pix_mp.width,
					     IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	try_fmt.fmt.pix_mp.height = clamp_val(f->fmt.pix_mp.height,
					      IMG_MIN_HEIGHT, IMG_MAX_HEIGHT);
	/* 4 bytes alignment for width */
	/* Todo: width and stride should align bus_size */
	try_fmt.fmt.pix_mp.width = ALIGN(try_fmt.fmt.pix_mp.width, IMG_PIX_ALIGN);
	try_fmt.fmt.pix_mp.num_planes = 1;

	if (raw_feature & MTK_CAM_FEATURE_HDR_MASK) {
		if (raw_feature & MTK_CAM_FEATURE_OFFLINE_M2M_MASK)
			is_hdr_m2m = 1;
		else
			is_hdr = 1;
	}

	/* support 1/2/3 plane */
	/* Note: vhdr m2m main stream is implicitly multiple plane */
	/* but not nego through try format */
	if ((node->desc.id == MTK_RAW_MAIN_STREAM_OUT && is_hdr) ||
		(node->desc.id == MTK_RAW_RAWI_2_IN && is_hdr_m2m)) {

		switch (raw_feature & MTK_CAM_FEATURE_HDR_MASK) {
		case STAGGER_2_EXPOSURE_LE_SE:
		case STAGGER_2_EXPOSURE_SE_LE:
		case MSTREAM_NE_SE:
		case MSTREAM_SE_NE:
			try_fmt.fmt.pix_mp.num_planes = 2;
			break;
		case STAGGER_3_EXPOSURE_LE_NE_SE:
		case STAGGER_3_EXPOSURE_SE_NE_LE:
			try_fmt.fmt.pix_mp.num_planes = 3;
			break;
		default:
			try_fmt.fmt.pix_mp.num_planes = 1;
		}
	}

	for (i = 0 ; i < try_fmt.fmt.pix_mp.num_planes ; i++)
		try_fmt.fmt.pix_mp.plane_fmt[i].bytesperline =
				f->fmt.pix_mp.plane_fmt[i].bytesperline;

	/* bytesperline & sizeimage calculation */
	if (node->desc.dma_port == MTKCAM_IPI_CAMSV_MAIN_OUT)
		cal_image_pix_mp(node->desc.id, &try_fmt.fmt.pix_mp, 3);
	else
		cal_image_pix_mp(node->desc.id, &try_fmt.fmt.pix_mp, 0);

	/* subsample */
	if (node->desc.id >= MTK_RAW_MAIN_STREAM_OUT &&
		node->desc.id < MTK_RAW_RZH1N2TO_1_OUT &&
		(raw_feature & MTK_CAM_FEATURE_SUBSAMPLE_MASK)) {
		switch (raw_feature) {
		case HIGHFPS_2_SUBSAMPLE:
			try_fmt.fmt.pix_mp.num_planes = 2;
			break;
		case HIGHFPS_4_SUBSAMPLE:
			try_fmt.fmt.pix_mp.num_planes = 4;
			break;
		case HIGHFPS_8_SUBSAMPLE:
			try_fmt.fmt.pix_mp.num_planes = 8;
			break;
		case HIGHFPS_16_SUBSAMPLE:
			try_fmt.fmt.pix_mp.num_planes =
				MAX_SUBSAMPLE_PLANE_NUM;
			break;
		default:
			try_fmt.fmt.pix_mp.num_planes = 1;
		}
		bytesperline = try_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
		sizeimage = try_fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
		for (i = 0 ; i < try_fmt.fmt.pix_mp.num_planes ; i++) {
			try_fmt.fmt.pix_mp.plane_fmt[i].bytesperline =
				bytesperline;
			try_fmt.fmt.pix_mp.plane_fmt[i].sizeimage =
				sizeimage;
		}
		dev_info(cam->dev, "%s id:%d raw_feature:0x%x stride:%d size:%d\n",
			 __func__, node->desc.id, raw_feature,
			 bytesperline, sizeimage);
	}
	/* TODO: support camsv meta header */
#if PDAF_READY
	/* add header size for vc channel */
	if (node->desc.dma_port == MTKCAM_IPI_CAMSV_MAIN_OUT &&
		node->desc.id == MTK_CAMSV_MAIN_STREAM_OUT)
		try_fmt.fmt.pix_mp.plane_fmt[0].sizeimage +=
		mtk_cam_get_meta_size(MTKCAM_IPI_CAMSV_MAIN_OUT);
#endif

	/* Constant format fields */
	try_fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_SRGB;
	try_fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
	try_fmt.fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	try_fmt.fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	try_fmt.fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_SRGB;

	*f = try_fmt;
	if (request_fd_echo)
		mtk_cam_fmt_set_request(&f->fmt.pix_mp, request_fd);

	return 0;
}

int mtk_cam_vidioc_try_fmt(struct file *file, void *fh,
			   struct v4l2_format *f)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);
	int raw_feature = 0;

	if (is_raw_subdev(node->uid.pipe_id))
		raw_feature = mtk_cam_fmt_get_raw_feature(&f->fmt.pix_mp);

	mtk_cam_video_set_fmt(node, f, raw_feature);

	/**
	 * try_fmt is a negotiation probe: never push the request fd back
	 * into reserved[], callers may pass garbage there.
	 */
	memset(f->fmt.pix_mp.reserved, 0, sizeof(f->fmt.pix_mp.reserved));

	return 0;
}

int mtk_cam_vidioc_meta_enum_fmt(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);

	if (f->index)
		return -EINVAL;

	/* See mtk_cam_vidioc_enum_fmt(): mainline v4l_fill_fmtdesc() also
	 * WARNs on the MTK-private META formats.
	 */
	f->pixelformat = node->active_fmt.fmt.meta.dataformat;
	f->flags = 0;
	snprintf(f->description, sizeof(f->description), "%p4cc", &f->pixelformat);

	return 0;
}

int mtk_cam_vidioc_g_meta_fmt(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);

	f->fmt.meta.dataformat = node->active_fmt.fmt.meta.dataformat;
	f->fmt.meta.buffersize = node->active_fmt.fmt.meta.buffersize;

	return 0;
}

int mtk_cam_vidioc_s_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct mtk_cam_device *cam = video_drvdata(file);
	struct mtk_cam_video_device *node = file_to_mtk_cam_node(file);
	struct mtk_cam_request_stream_data *stream_data;
	struct mtk_cam_request *cam_req;
	struct media_request *req;
	struct v4l2_selection *vsel;
	s32 fd;

	fd = mtk_cam_selection_get_request(s);
	if (fd < 0)
		return -EINVAL;

	req = media_request_get_by_fd(&cam->media_dev, fd);
	if (IS_ERR(req)) {
		if (!vb2_is_busy(node->vdev.queue)) {
			dev_info(cam->dev,
				"%s:pipe(%d):%s: apply setting without fd\n",
				__func__, node->uid.pipe_id, node->desc.name);
			node->pending_crop = *s;
			return 0;
		}

		/*
		 * XAGA mainline: the Android HAL issues S_SELECTION without a
		 * request fd even once the queue is busy, and expects the crop
		 * to be staged. Returning -EINVAL here made the HAL log
		 * "SetSelection ... Invalid argument" on every session.
		 */
		dev_info(cam->dev,
			"%s:pipe(%d):%s:apply selection without fd\n",
			__func__, node->uid.pipe_id, node->desc.name);
		node->pending_crop = *s;

		return 0;
	}

	cam_req = to_mtk_cam_req(req);
	stream_data = mtk_cam_req_get_s_data_no_chk(cam_req, node->uid.pipe_id, 0);
	stream_data->vdev_selection_update |= (1 << node->desc.id);
	vsel = mtk_cam_s_data_get_vsel(stream_data, node->desc.id);
	*vsel = *s;
	dev_dbg(cam->dev,
		"%s:%s:pipe(%d):%s:pending vidioc_s_selection (%d,%d,%d,%d)\n",
		__func__, cam_req->req.debug_str, node->uid.pipe_id, node->desc.name,
		s->r.left, s->r.top, s->r.width, s->r.height);

	media_request_put(req);

	return 0;
}

void mtk_cam_fmt_set_raw_feature(struct v4l2_pix_format_mplane *fmt_mp, int raw_feature)
{
	u8 *reserved = fmt_mp->reserved;

	fmt_mp->flags = raw_feature & 0x000000FF;
	reserved[4] = (raw_feature & 0x0000FF00) >> 8;
	reserved[5] = (raw_feature & 0x00FF0000) >> 16;
	reserved[6] = (raw_feature & 0xFF000000) >> 24;
}

int mtk_cam_fmt_get_raw_feature(struct v4l2_pix_format_mplane *fmt_mp)
{
	int raw_feature = fmt_mp->flags;

	/**
	 * Current 8 bits flag is not enough so we also use the reserved[4-6] to
	 * save the feature flags.
	 */
	raw_feature |= ((unsigned int)fmt_mp->reserved[4]) << 8 & 0x0000FF00;
	raw_feature |= ((unsigned int)fmt_mp->reserved[5]) << 16 & 0x00FF0000;
	raw_feature |= ((unsigned int)fmt_mp->reserved[6]) << 24 & 0xFF000000;

	return raw_feature;
}

int mtk_cam_fmt_get_request(struct v4l2_pix_format_mplane *fmt_mp)
{
	int field;
	int reserved_fields = 4;
	s32 request_fd = 0;

	for (field = 0; field < reserved_fields; field++) {
		request_fd +=
			fmt_mp->reserved[field] << BITS_PER_BYTE * field;
		fmt_mp->reserved[field] = 0;
	}

	return request_fd;
}

void mtk_cam_fmt_set_request(struct v4l2_pix_format_mplane *fmt_mp, int request_fd)
{
	u8 *reserved = fmt_mp->reserved;

	reserved[0] = request_fd & 0x000000FF;
	reserved[1] = (request_fd & 0x0000FF00) >> 8;
	reserved[2] = (request_fd & 0x00FF0000) >> 16;
	reserved[3] = (request_fd & 0xFF000000) >> 24;
}

int mtk_cam_selection_get_request(struct v4l2_selection *crop)
{
	s32 request_fd = 0;

	request_fd = crop->reserved[0];
	crop->reserved[0] = 0;

	return request_fd;
}

void mtk_cam_selection_set_request(struct v4l2_selection *crop, int request_fd)
{
	u32 *reserved = crop->reserved;

	reserved[0] = request_fd;
}
