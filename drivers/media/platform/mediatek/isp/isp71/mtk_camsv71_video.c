// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek ISP 7.1 CAMSV raw capture: video capture node and buffer queue
 *
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 * Copyright (c) 2025 MediaTek Inc.
 */

#include <linux/align.h>
#include <linux/dma-mapping.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_camsv71.h"

static struct mtk_camsv_buffer *to_mtk_camsv_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct mtk_camsv_buffer, vb);
}

static const struct mtk_camsv_format *
mtk_camsv_find_fourcc(const struct mtk_camsv *cam, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < cam->soc->num_formats; i++)
		if (cam->soc->formats[i].fourcc == fourcc)
			return &cam->soc->formats[i];
	return NULL;
}

/*
 * Lines start at bytesperline intervals: at least the bytes the IMGO writes
 * per line, larger strides (as libcamera's GPU debayering asks for) up to the
 * 16-bit IMGO_STRIDE limit.
 */
static void mtk_camsv_try_pix(struct mtk_camsv *cam, struct v4l2_pix_format *pix)
{
	const struct mtk_camsv_format *fmt;
	u32 min_bpl;

	fmt = mtk_camsv_find_fourcc(cam, pix->pixelformat) ?: &cam->soc->formats[0];
	pix->pixelformat = fmt->fourcc;
	pix->width = clamp_t(u32, ALIGN_DOWN(pix->width, MTK_CAMSV_WIDTH_ALIGN),
			     MTK_CAMSV_MIN_WIDTH, MTK_CAMSV_MAX_WIDTH);
	pix->height = clamp_t(u32, ALIGN_DOWN(pix->height, MTK_CAMSV_HEIGHT_ALIGN),
			      MTK_CAMSV_MIN_HEIGHT, MTK_CAMSV_MAX_HEIGHT);
	min_bpl = mtk_camsv_xsize(cam, fmt, pix->width);
	if (pix->bytesperline < min_bpl || pix->bytesperline > MTK_CAMSV_MAX_STRIDE)
		pix->bytesperline = min_bpl;
	else
		pix->bytesperline = ALIGN(pix->bytesperline,
					  2U << cam->soc->pixel_mode);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;
}

/* ----------------------------------------------------------------------------
 * Buffer queue
 */

static int mtk_camsv_queue_setup(struct vb2_queue *vq, unsigned int *num_buffers,
				 unsigned int *num_planes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);

	if (*num_planes)
		return *num_planes != 1 || sizes[0] < cam->pix.sizeimage ?
		       -EINVAL : 0;
	*num_planes = 1;
	sizes[0] = cam->pix.sizeimage;
	return 0;
}

static int mtk_camsv_buf_init(struct vb2_buffer *vb)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vb->vb2_queue);
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/*
	 * Never an empty address (camisp fix beb11846d7a1), only page aligned
	 * buffers as in every verified run, and only what the IMGO reaches.
	 */
	if (!addr || !IS_ALIGNED(addr, PAGE_SIZE) ||
	    addr + vb2_plane_size(vb, 0) - 1 > DMA_BIT_MASK(MTK_CAMSV_DMA_BITS)) {
		dev_err(cam->dev, "unusable buffer address %pad\n", &addr);
		return -EINVAL;
	}
	to_mtk_camsv_buffer(vb)->addr = addr;
	return 0;
}

static int mtk_camsv_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < cam->pix.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, cam->pix.sizeimage);
	return 0;
}

static void mtk_camsv_buf_queue(struct vb2_buffer *vb)
{
	mtk_camsv71_queue_buffer(vb2_get_drv_priv(vb->vb2_queue),
				 to_mtk_camsv_buffer(vb));
}

/* The IRQ is disabled and the CAMSV reset, or it never ran. */
static void mtk_camsv_return_all(struct mtk_camsv *cam,
				 enum vb2_buffer_state state)
{
	struct mtk_camsv_buffer *buf, *tmp;

	spin_lock_irq(&cam->lock);
	if (cam->pending)
		list_add(&cam->pending->list, &cam->queued);
	if (cam->active)
		list_add(&cam->active->list, &cam->queued);
	cam->pending = NULL;
	cam->active = NULL;
	list_for_each_entry_safe(buf, tmp, &cam->queued, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irq(&cam->lock);
}

/*
 * Teardown could not prove that the CAMSV stopped: keep the module, the
 * power and the LARB reference, and refuse further streams.
 */
static void mtk_camsv_quarantine(struct mtk_camsv *cam, int err)
{
	cam->quarantined = true;
	__module_get(THIS_MODULE);
	dev_err(cam->dev,
		"stop not confirmed (%d): power and %s kept, reboot required\n",
		err, dev_name(cam->larb));
}

static void mtk_camsv_log_stop(struct mtk_camsv *cam)
{
	const struct mtk_camsv_stats *s = &cam->stats;

	dev_info(cam->dev,
		 "stream stop%s: frames %u written %u dropped %u bad %u irq %#x late %#x imgo_err %#x fbc %#x reset %#x tg %#x larb %#x/%#x credits %u early %u hit %u deferred %u nobuf %u after %u straddle %u imgo_drop %u lead_us %d\n",
		 s->failed ? " after an error" : s->forced ? " forced" : "",
		 cam->frame, s->done, s->drops, s->bad, s->irq_status,
		 s->late_status, s->imgo_err, s->fbc_stop, s->fbc_reset,
		 s->tg_stop, s->larb_start, mtk_camsv71_larb_port(cam),
		 s->credits, s->early, s->early_hit, s->deferred, s->drop_nobuf,
		 s->drop_after, s->drop_straddle, s->imgo_drop,
		 s->lead_us == U32_MAX ? -1 : (int)s->lead_us);
}

static int mtk_camsv_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);
	int ret, err;

	if (cam->quarantined) {
		dev_err(cam->dev, "quarantined after a failed stop\n");
		ret = -EIO;
		goto err_return;
	}
	memset(&cam->stats, 0, sizeof(cam->stats));

	ret = video_device_pipeline_alloc_start(&cam->vdev);
	if (ret)
		goto err_return;
	/* Resumes the SENINF and then the sensor first (device links). */
	ret = pm_runtime_resume_and_get(cam->dev);
	if (ret)
		goto err_pipeline;
	ret = mtk_camsv71_larb_get(cam);
	if (ret)
		goto err_pm;
	if (!mtk_camsv71_hw_idle(cam)) {
		ret = -EBUSY;
		goto err_larb;
	}
	ret = mtk_camsv71_hw_start(cam);
	if (ret)
		goto err_hw;
	/* Armed: now the SENINF route, its PHY and, last, the sensor. */
	ret = v4l2_subdev_enable_streams(&cam->subdev, MTK_CAMSV_PAD_SRC,
					 BIT_ULL(0));
	if (ret)
		goto err_hw;
	return 0;

err_hw:
	err = mtk_camsv71_hw_stop_capture(cam, false);
	err = mtk_camsv71_hw_disable(cam, err);
	if (err)
		mtk_camsv_quarantine(cam, err);
err_larb:
	if (!cam->quarantined)
		mtk_camsv71_larb_put(cam);
err_pm:
	if (!cam->quarantined)
		pm_runtime_put(cam->dev);
err_pipeline:
	video_device_pipeline_stop(&cam->vdev);
err_return:
	mtk_camsv_return_all(cam, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void mtk_camsv_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_camsv *cam = vb2_get_drv_priv(vq);
	int tg_ret, ret;

	tg_ret = mtk_camsv71_hw_stop_capture(cam, true);
	/* The SENINF stops the sensor first, then its route and PHY. */
	ret = v4l2_subdev_disable_streams(&cam->subdev, MTK_CAMSV_PAD_SRC,
					  BIT_ULL(0));
	if (ret)
		dev_err(cam->dev, "SENINF stream off failed: %d\n", ret);
	ret = mtk_camsv71_hw_disable(cam, tg_ret);
	if (ret)
		mtk_camsv_quarantine(cam, ret);
	mtk_camsv_return_all(cam, VB2_BUF_STATE_ERROR);
	mtk_camsv_log_stop(cam);
	if (!cam->quarantined) {
		mtk_camsv71_larb_put(cam);
		pm_runtime_put(cam->dev);
	}
	video_device_pipeline_stop(&cam->vdev);
}

static const struct vb2_ops mtk_camsv_vb2_ops = {
	.queue_setup = mtk_camsv_queue_setup,
	.buf_init = mtk_camsv_buf_init,
	.buf_prepare = mtk_camsv_buf_prepare,
	.buf_queue = mtk_camsv_buf_queue,
	.start_streaming = mtk_camsv_start_streaming,
	.stop_streaming = mtk_camsv_stop_streaming,
};

/* ----------------------------------------------------------------------------
 * Video node
 */

static int mtk_camsv_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, MTK_CAMSV71_NAME, sizeof(cap->driver));
	strscpy(cap->card, "MediaTek ISP 7.1 CAMSV", sizeof(cap->card));
	return 0;
}

static int mtk_camsv_enum_fmt(struct file *file, void *priv,
			      struct v4l2_fmtdesc *f)
{
	struct mtk_camsv *cam = video_drvdata(file);
	const struct mtk_camsv_format *fmt;

	if (f->mbus_code) {
		fmt = mtk_camsv71_find_code(cam, f->mbus_code);
		if (!fmt || f->index)
			return -EINVAL;
	} else {
		if (f->index >= cam->soc->num_formats)
			return -EINVAL;
		fmt = &cam->soc->formats[f->index];
	}
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int mtk_camsv_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_camsv *cam = video_drvdata(file);

	f->fmt.pix = cam->pix;
	return 0;
}

static int mtk_camsv_try_fmt(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	mtk_camsv_try_pix(video_drvdata(file), &f->fmt.pix);
	return 0;
}

static int mtk_camsv_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mtk_camsv *cam = video_drvdata(file);

	if (vb2_is_busy(&cam->queue))
		return -EBUSY;
	mtk_camsv_try_pix(cam, &f->fmt.pix);
	cam->pix = f->fmt.pix;
	cam->fmt = mtk_camsv_find_fourcc(cam, cam->pix.pixelformat);
	return 0;
}

static int mtk_camsv_enum_framesizes(struct file *file, void *priv,
				     struct v4l2_frmsizeenum *fsize)
{
	struct mtk_camsv *cam = video_drvdata(file);

	if (fsize->index || !mtk_camsv_find_fourcc(cam, fsize->pixel_format))
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = MTK_CAMSV_MIN_WIDTH;
	fsize->stepwise.max_width = MTK_CAMSV_MAX_WIDTH;
	fsize->stepwise.step_width = MTK_CAMSV_WIDTH_ALIGN;
	fsize->stepwise.min_height = MTK_CAMSV_MIN_HEIGHT;
	fsize->stepwise.max_height = MTK_CAMSV_MAX_HEIGHT;
	fsize->stepwise.step_height = MTK_CAMSV_HEIGHT_ALIGN;
	return 0;
}

static const struct v4l2_ioctl_ops mtk_camsv_ioctl_ops = {
	.vidioc_querycap = mtk_camsv_querycap,
	.vidioc_enum_fmt_vid_cap = mtk_camsv_enum_fmt,
	.vidioc_g_fmt_vid_cap = mtk_camsv_g_fmt,
	.vidioc_s_fmt_vid_cap = mtk_camsv_s_fmt,
	.vidioc_try_fmt_vid_cap = mtk_camsv_try_fmt,
	.vidioc_enum_framesizes = mtk_camsv_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

/*
 * The media device and the V4L2 device belong to the SENINF driver. Pin its
 * module while the node is open, as subdev nodes do: unloading it would free
 * them under the open file.
 */
static int mtk_camsv_video_open(struct file *file)
{
	struct mtk_camsv *cam = video_drvdata(file);
	int ret;

	if (!try_module_get(cam->v4l2_owner))
		return -ENODEV;

	ret = v4l2_fh_open(file);
	if (ret)
		module_put(cam->v4l2_owner);
	return ret;
}

static int mtk_camsv_video_release(struct file *file)
{
	struct mtk_camsv *cam = video_drvdata(file);
	int ret;

	ret = vb2_fop_release(file);
	module_put(cam->v4l2_owner);
	return ret;
}

static const struct v4l2_file_operations mtk_camsv_fops = {
	.owner = THIS_MODULE,
	.open = mtk_camsv_video_open,
	.release = mtk_camsv_video_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* The video format must match the CAMSV source pad at stream on. */
static int mtk_camsv_video_link_validate(struct media_link *link)
{
	struct video_device *vdev = media_entity_to_video_device(link->sink->entity);
	struct mtk_camsv *cam = video_get_drvdata(vdev);
	const struct v4l2_mbus_framefmt *mf;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(&cam->subdev);
	mf = v4l2_subdev_state_get_format(state, MTK_CAMSV_PAD_SRC);
	if (mf->code != cam->fmt->code || mf->width != cam->pix.width ||
	    mf->height != cam->pix.height) {
		dev_dbg(cam->dev, "format %#x %ux%u does not match %p4cc %ux%u\n",
			mf->code, mf->width, mf->height, &cam->pix.pixelformat,
			cam->pix.width, cam->pix.height);
		ret = -EPIPE;
	}
	v4l2_subdev_unlock_state(state);
	return ret;
}

static const struct media_entity_operations mtk_camsv_video_entity_ops = {
	.link_validate = mtk_camsv_video_link_validate,
};

/* Called when the SENINF binds the sub-device, possibly more than once. */
int mtk_camsv71_video_register(struct mtk_camsv *cam)
{
	struct video_device *vdev = &cam->vdev;
	struct vb2_queue *q = &cam->queue;
	int ret;

	memset(vdev, 0, sizeof(*vdev));
	memset(q, 0, sizeof(*q));

	cam->fmt = &cam->soc->formats[0];
	cam->pix = (struct v4l2_pix_format) {
		.width = MTK_CAMSV_DEF_WIDTH,
		.height = MTK_CAMSV_DEF_HEIGHT,
		.pixelformat = cam->fmt->fourcc,
	};
	mtk_camsv_try_pix(cam, &cam->pix);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->dev = cam->dev;
	q->ops = &mtk_camsv_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mtk_camsv_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	q->lock = &cam->vlock;
	q->drv_priv = cam;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	cam->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &cam->vdev_pad);
	if (ret)
		return ret;
	vdev->entity.ops = &mtk_camsv_video_entity_ops;
	snprintf(vdev->name, sizeof(vdev->name), "%s capture", cam->subdev.name);
	vdev->fops = &mtk_camsv_fops;
	vdev->ioctl_ops = &mtk_camsv_ioctl_ops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->release = video_device_release_empty;
	vdev->lock = &cam->vlock;
	vdev->queue = q;
	vdev->v4l2_dev = cam->subdev.v4l2_dev;
	vdev->dev_parent = cam->dev;
	video_set_drvdata(vdev, cam);
	cam->v4l2_owner = vdev->v4l2_dev->dev->driver->owner;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;
	ret = media_create_pad_link(&cam->subdev.entity, MTK_CAMSV_PAD_SRC,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret)
		vb2_video_unregister_device(vdev);
	return ret;
}

void mtk_camsv71_video_unregister(struct mtk_camsv *cam)
{
	vb2_video_unregister_device(&cam->vdev);
}
