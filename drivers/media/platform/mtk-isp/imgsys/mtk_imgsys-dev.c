// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018 MediaTek Inc.
 *
 * Author: Frederic Chen <frederic.chen@mediatek.com>
 *
 */

#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/hashtable.h>
#include <linux/iommu.h>
#include <linux/iosys-map.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include "mtk_imgsys-dev.h"
#include "mtk-img-ipi.h"
#include "mtk_header_desc.h"
#include "mtk_imgsys-trace.h"

struct fd_kva_list_t fd_kva_info_list = {
	.mymutex = __MUTEX_INITIALIZER(fd_kva_info_list.mymutex),
	.mylist = LIST_HEAD_INIT(fd_kva_info_list.mylist)
};

unsigned int nodes_num;

int mtk_imgsys_pipe_init(struct mtk_imgsys_dev *imgsys_dev,
				struct mtk_imgsys_pipe *pipe,
				const struct mtk_imgsys_pipe_desc *setting)
{
	int ret;
	u32 i;
	size_t nodes_size;

	pipe->imgsys_dev = imgsys_dev;
	pipe->desc = setting;
	pipe->nodes_enabled = 0ULL;
	pipe->nodes_streaming = 0ULL;

	atomic_set(&pipe->pipe_job_sequence, 0);
	INIT_LIST_HEAD(&pipe->pipe_job_running_list);
	INIT_LIST_HEAD(&pipe->pipe_job_pending_list);
	INIT_LIST_HEAD(&pipe->iova_cache.list);
	spin_lock_init(&pipe->iova_cache.lock);
	hash_init(pipe->iova_cache.hlists);
	//spin_lock_init(&pipe->job_lock);
	//mutex_init(&pipe->job_lock);
	spin_lock_init(&pipe->pending_job_lock);
	spin_lock_init(&pipe->running_job_lock);
	mutex_init(&pipe->lock);

	nodes_num = pipe->desc->total_queues;
	nodes_size = sizeof(*pipe->nodes) * nodes_num;
	pipe->nodes = devm_kzalloc(imgsys_dev->dev, nodes_size, GFP_KERNEL);
	if (!pipe->nodes)
		return -ENOMEM;

	for (i = 0; i < nodes_num; i++) {
		pipe->nodes[i].desc = &pipe->desc->queue_descs[i];
		pipe->nodes[i].flags = pipe->nodes[i].desc->flags;
		spin_lock_init(&pipe->nodes[i].buf_list_lock);
		INIT_LIST_HEAD(&pipe->nodes[i].buf_list);

		pipe->nodes[i].crop.left = 0;
		pipe->nodes[i].crop.top = 0;
		pipe->nodes[i].crop.width =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_width;
		pipe->nodes[i].crop.height =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_height;
		pipe->nodes[i].compose.left = 0;
		pipe->nodes[i].compose.top = 0;
		pipe->nodes[i].compose.width =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_width;
		pipe->nodes[i].compose.height =
			pipe->nodes[i].desc->frmsizeenum->stepwise.max_height;
	}

	ret = mtk_imgsys_pipe_v4l2_register(pipe, &imgsys_dev->mdev,
					 &imgsys_dev->v4l2_dev);
	if (ret) {
		dev_info(pipe->imgsys_dev->dev,
			"%s: failed(%d) to create V4L2 devices\n",
			pipe->desc->name, ret);

		goto err_destroy_pipe_lock;
	}

	return 0;

err_destroy_pipe_lock:
	mutex_destroy(&pipe->lock);

	return ret;
}

int mtk_imgsys_pipe_release(struct mtk_imgsys_pipe *pipe)
{
	mtk_imgsys_pipe_v4l2_unregister(pipe);
	mutex_destroy(&pipe->lock);

	return 0;
}

int mtk_imgsys_pipe_next_job_id(struct mtk_imgsys_pipe *pipe)
{
	int global_job_id = atomic_inc_return(&pipe->pipe_job_sequence);

	return (global_job_id & 0x0000FFFF) | (pipe->desc->id << 16);
}

#ifdef BATCH_MODE_V3
int mtk_imgsys_pipe_next_job_id_batch_mode(struct mtk_imgsys_pipe *pipe,
		unsigned short user_id)
{
	int global_job_id = atomic_inc_return(&pipe->pipe_job_sequence);

	return (global_job_id & 0x0000FFFF) | (user_id << 16);
}
#endif

struct mtk_imgsys_request *mtk_imgsys_pipe_get_running_job(
						struct mtk_imgsys_pipe *pipe,
						int id)
{
	struct mtk_imgsys_request *req;
	unsigned long flag;

	spin_lock_irqsave(&pipe->running_job_lock, flag);
	list_for_each_entry(req,
			    &pipe->pipe_job_running_list, list) {
		if (req->id == id) {
			spin_unlock_irqrestore(&pipe->running_job_lock, flag);
			return req;
		}
	}
	spin_unlock_irqrestore(&pipe->running_job_lock, flag);

	return NULL;
}

void mtk_imgsys_pipe_remove_job(struct mtk_imgsys_request *req)
{
	unsigned long flag;
	struct list_head *prev, *next, *entry;

	entry = &req->list;
	prev = entry->prev;
	next = entry->next;

	spin_lock_irqsave(&req->imgsys_pipe->running_job_lock, flag);
#ifdef JOB_RUN
	if ((req->list.next == LIST_POISON1) ||
					(req->list.prev == LIST_POISON2)) {
		dev_info(req->imgsys_pipe->imgsys_dev->dev, "%s: req-fd %d already deleted, prev(0x%lx), next(0x%lx, entry(0x%lx))",
			__func__, req->tstate.req_fd, req->list.prev, req->list.next, entry);
		spin_unlock_irqrestore(&req->imgsys_pipe->running_job_lock, flag);
		return;
	}

	if ((prev->next != entry) || (next->prev != entry)) {
		dev_info(req->imgsys_pipe->imgsys_dev->dev, "%s: req-fd %d not in list, prev->next(0x%lx), next->prev(0x%lx), entry(0x%lx)",
			__func__, req->tstate.req_fd, prev->next, next->prev, entry);
		spin_unlock_irqrestore(&req->imgsys_pipe->running_job_lock, flag);
		return;
	}

	list_del(&req->list);
#endif
	req->imgsys_pipe->num_jobs--;
	spin_unlock_irqrestore(&req->imgsys_pipe->running_job_lock, flag);

	dev_dbg(req->imgsys_pipe->imgsys_dev->dev,
		"%s:%s:req->id(%d),num of running jobs(%d) entry(0x%lx)\n", __func__,
		req->imgsys_pipe->desc->name, req->id,
		req->imgsys_pipe->num_jobs, entry);
}

void mtk_imgsys_pipe_debug_job(struct mtk_imgsys_pipe *pipe,
			    struct mtk_imgsys_request *req)
{
	int i;

	dev_dbg(pipe->imgsys_dev->dev, "%s:%s: pipe-job(%p),id(%d)\n",
		__func__, pipe->desc->name, req, req->id);

	for (i = 0; i < pipe->desc->total_queues ; i++) {
		if (req->buf_map[i])
			dev_dbg(pipe->imgsys_dev->dev, "%s:%s:buf(%p)\n",
				pipe->desc->name, pipe->nodes[i].desc->name,
				req->buf_map[i]);
	}
}

#ifdef BATCH_MODE_V3
bool is_batch_mode(struct mtk_imgsys_request *req)
{
	return req->is_batch_mode;
}

void mtk_imgsys_frame_done(struct mtk_imgsys_request *req)
{
	bool found = false;
	const int user_id = mtk_imgsys_pipe_get_pipe_from_job_id(req->id);
	struct mtk_imgsys_user *user;
	struct mtk_imgsys_dev *imgsys_dev = req->imgsys_pipe->imgsys_dev;
	unsigned long flags;

	mutex_lock(&imgsys_dev->imgsys_users.user_lock);
	list_for_each_entry(user, &imgsys_dev->imgsys_users.list, entry) {
		if (user == NULL)
			continue;

		if (user->id == user_id) {
			found = true;
			break;
		}
	}
	mutex_unlock(&imgsys_dev->imgsys_users.user_lock);

	// add into user's done_list and wake it up
	if (found) {
		spin_lock_irqsave(&user->lock, flags);

		list_add_tail(&req->done_pack->done_entry, &user->done_list);

		user->dqdonestate = true;


		pr_info("%s: user id(%x) req(%p) add done_pack\n", __func__,
			user->id, req);

		wake_up(&user->done_wq);
		wake_up(&user->enque_wq);
		spin_unlock_irqrestore(&user->lock, flags);

		if (!IS_ERR(req)) {
			pr_info("%s free req(%p)\n", __func__, req);
			vfree(req);
		}
	}
}
#endif

void mtk_imgsys_pipe_job_finish(struct mtk_imgsys_request *req,
			     enum vb2_buffer_state vbf_state)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *in_buf;
	int i;
	int req_id = req->id;
	unsigned int vb2_buffer_index;

#ifdef BATCH_MODE_V3
	// batch mode
	if (is_batch_mode(req)) {
		mtk_imgsys_frame_done(req);
		return;
	}
#endif
	dev_dbg(pipe->imgsys_dev->dev, "%s: req %d 0x%lx state(%d)",
			__func__, req->tstate.req_fd, &req->req, req->req.state);
	if (req->req.state != MEDIA_REQUEST_STATE_QUEUED) {
		dev_info(pipe->imgsys_dev->dev, "%s: req %d 0x%lx flushed in state(%d)", __func__,
					req->tstate.req_fd, &req->req, req->req.state);
		goto done;
	}

	i = is_singledev_mode(req);
	if (!i)
		i = MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT;

	in_buf = req->buf_map[i];

	for (i = 0; i < pipe->desc->total_queues; i++) {
		struct mtk_imgsys_dev_buffer *dev_buf = req->buf_map[i];
		struct mtk_imgsys_video_device *node;

		if (!dev_buf)
			continue;

		if (dev_buf != in_buf)
			dev_buf->vbb.vb2_buf.timestamp =
				in_buf->vbb.vb2_buf.timestamp;

		vb2_buffer_index = dev_buf->vbb.vb2_buf.index;
		node = mtk_imgsys_vbq_to_node(dev_buf->vbb.vb2_buf.vb2_queue);

		spin_lock(&node->buf_list_lock);
		list_del(&dev_buf->list);
		spin_unlock(&node->buf_list_lock);


		/*  vb2_buf is changed after below function  */
		vb2_buffer_done(&dev_buf->vbb.vb2_buf, vbf_state);

		dev_dbg(pipe->imgsys_dev->dev,
			"%s:%s:%s: return buf, idx(%d), state(%d)\n",
			__func__, pipe->desc->name, node->desc->name,
			vb2_buffer_index, vbf_state);
	}
done:
	req->tstate.time_notify2vb2done = ktime_get_boottime_ns()/1000;
	complete(&req->done);
		dev_dbg(pipe->imgsys_dev->dev,
			"[KT]%s:%d:%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld,%6lld\n",
			__func__, req->tstate.req_fd,
			//req->tstate.time_hwenq,
			req->tstate.time_composingStart,
			req->tstate.time_composingEnd,
			req->tstate.time_qw2composer,
			req->tstate.time_compfuncStart,
			req->tstate.time_ipisendStart,
			req->tstate.time_reddonescpStart,
			req->tstate.time_doframeinfo,
			req->tstate.time_qw2runner,
			req->tstate.time_runnerStart,
			req->tstate.time_send2cmq,
			req->tstate.time_mdpcbStart,
			req->tstate.time_notifyStart,
			req->tstate.time_unmapiovaEnd,
			req->tstate.time_notify2vb2done);

	dev_dbg(pipe->imgsys_dev->dev,
		"%s:%s: finished job id(%d), vbf_state(%d)\n",
		__func__, pipe->desc->name, req_id, vbf_state);
}

static u32 dip_pass1_fmt_get_stride(struct v4l2_pix_format_mplane *mfmt,
				    const struct mtk_imgsys_dev_format *fmt,
				    unsigned int plane)
{
	unsigned int width = ALIGN(mfmt->width, 4);
	unsigned int pixel_bits = fmt->row_depth[0];
	unsigned int bpl, ppl;

	ppl = DIV_ROUND_UP(width * fmt->depth[0], fmt->row_depth[0]);
	bpl = DIV_ROUND_UP(ppl * pixel_bits, 8);

	return ALIGN(bpl, fmt->pass_1_align);
}

/* Stride that is accepted by MDP HW */
static u32 dip_mdp_fmt_get_stride(struct v4l2_pix_format_mplane *mfmt,
				  const struct mtk_imgsys_dev_format *fmt,
				  unsigned int plane)
#ifdef MDP_COLOR
{
	enum mdp_color c = fmt->mdp_color;
	u32 bytesperline = (mfmt->width * fmt->row_depth[plane]) / 8;
	u32 stride = (bytesperline * DIP_MCOLOR_BITS_PER_PIXEL(c))
		/ fmt->row_depth[0];

	if (plane == 0)
		return stride;

	if (plane < DIP_MCOLOR_GET_PLANE_COUNT(c)) {
		if (DIP_MCOLOR_IS_BLOCK_MODE(c))
			stride = stride / 2;
		return stride;
	}

	return 0;
}
#else
{
	/* mainline port: the MDP_COLOR tables are not ported, so derive a plain
	 * bytes-per-line stride from the format's row depth.  Without this the
	 * video-node formats that use the MDP path get sizeimage 0 and REQBUFS
	 * fails. */
	u32 bpp = fmt->row_depth[plane];

	if (!bpp)
		return 0;

	return ALIGN(DIV_ROUND_UP(mfmt->width * bpp, 8), 16);
}
#endif

static int mtk_imgsys_pipe_get_stride(struct mtk_imgsys_pipe *pipe,
				   struct v4l2_pix_format_mplane *mfmt,
				   const struct mtk_imgsys_dev_format *dfmt,
				   int plane, char *buf_name)
{
	int bpl;

	if (dfmt->pass_1_align)
		bpl = dip_pass1_fmt_get_stride(mfmt, dfmt, plane);
	else
		bpl = dip_mdp_fmt_get_stride(mfmt, dfmt, plane);

	dev_dbg(pipe->imgsys_dev->dev,
		"%s:%s:%s: plane(%d), pixelformat(%x) width(%d), stride(%d)",
		__func__, pipe->desc->name, buf_name, plane, mfmt->pixelformat,
		mfmt->width, bpl);

	return bpl;
}

void mtk_imgsys_pipe_try_fmt(struct mtk_imgsys_pipe *pipe,
			  struct mtk_imgsys_video_device *node,
			  struct v4l2_format *fmt,
			  const struct v4l2_format *ufmt,
			  const struct mtk_imgsys_dev_format *dfmt)
{
	int i;

	fmt->type = ufmt->type;
	fmt->fmt.pix_mp.width =
		clamp_val(ufmt->fmt.pix_mp.width,
			  node->desc->frmsizeenum->stepwise.min_width,
			  node->desc->frmsizeenum->stepwise.max_width);
	fmt->fmt.pix_mp.height =
		clamp_val(ufmt->fmt.pix_mp.height,
			  node->desc->frmsizeenum->stepwise.min_height,
			  node->desc->frmsizeenum->stepwise.max_height);
	fmt->fmt.pix_mp.num_planes = ufmt->fmt.pix_mp.num_planes;
	fmt->fmt.pix_mp.quantization = ufmt->fmt.pix_mp.quantization;
	fmt->fmt.pix_mp.colorspace = ufmt->fmt.pix_mp.colorspace;
	fmt->fmt.pix_mp.field = V4L2_FIELD_NONE;

	/* Only MDP 0 and MDP 1 allow the color space change */
#ifdef MDP_PORT
	if (node->desc->id != MTK_IMGSYS_VIDEO_NODE_ID_WROT_A_CAPTURE &&
	    node->desc->id != MTK_IMGSYS_VIDEO_NODE_ID_WROT_B_CAPTURE) {
		fmt->fmt.pix_mp.quantization = V4L2_QUANTIZATION_FULL_RANGE;
		fmt->fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	}
#endif
	fmt->fmt.pix_mp.pixelformat = dfmt->format;
	fmt->fmt.pix_mp.num_planes = dfmt->num_planes;
	fmt->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->fmt.pix_mp.field = V4L2_FIELD_NONE;

	for (i = 0; i < fmt->fmt.pix_mp.num_planes; ++i) {
		unsigned int stride;
		unsigned int sizeimage;

		if (is_desc_fmt(dfmt)) {
			fmt->fmt.pix_mp.plane_fmt[i].sizeimage =
							dfmt->buffer_size;
		} else {
			stride = mtk_imgsys_pipe_get_stride(pipe,
							&fmt->fmt.pix_mp, dfmt,
							i, node->desc->name);
			if (dfmt->pass_1_align)
				sizeimage = stride * fmt->fmt.pix_mp.height;
			else
				sizeimage = DIV_ROUND_UP(stride *
							 fmt->fmt.pix_mp.height
							 * dfmt->depth[i],
							 dfmt->row_depth[i]);

			fmt->fmt.pix_mp.plane_fmt[i].sizeimage = sizeimage;
			fmt->fmt.pix_mp.plane_fmt[i].bytesperline = stride;
		}
	}

}

static void set_meta_fmt(struct mtk_imgsys_pipe *pipe,
			 struct v4l2_meta_format *fmt,
			 const struct mtk_imgsys_dev_format *dev_fmt)
{
	fmt->dataformat = dev_fmt->format;

	if (dev_fmt->buffer_size <= 0) {
		dev_dbg(pipe->imgsys_dev->dev,
			"%s: Invalid meta buf size(%u), use default(%u)\n",
			pipe->desc->name, dev_fmt->buffer_size,
			MTK_DIP_DEV_META_BUF_DEFAULT_SIZE);

		fmt->buffersize =
			MTK_DIP_DEV_META_BUF_DEFAULT_SIZE;
	} else {
		fmt->buffersize = dev_fmt->buffer_size;
	}
}

void mtk_imgsys_pipe_load_default_fmt(struct mtk_imgsys_pipe *pipe,
				   struct mtk_imgsys_video_device *node,
				   struct v4l2_format *fmt)
{
	int idx = node->desc->default_fmt_idx;

	if (idx >= node->desc->num_fmts) {
		dev_info(pipe->imgsys_dev->dev,
			"%s:%s: invalid idx(%d), must < num_fmts(%d)\n",
			__func__, node->desc->name, idx, node->desc->num_fmts);

		idx = 0;
	}

	fmt->type = node->desc->buf_type;
	if (mtk_imgsys_buf_is_meta(node->desc->buf_type)) {
		set_meta_fmt(pipe, &fmt->fmt.meta,
			     &node->desc->fmts[idx]);
	} else {
		fmt->fmt.pix_mp.width = node->desc->default_width;
		fmt->fmt.pix_mp.height = node->desc->default_height;
		mtk_imgsys_pipe_try_fmt(pipe, node, fmt, fmt,
				     &node->desc->fmts[idx]);
	}
}

const struct mtk_imgsys_dev_format*
mtk_imgsys_pipe_find_fmt(struct mtk_imgsys_pipe *pipe,
		      struct mtk_imgsys_video_device *node,
		      u32 format)
{
	int i;

	for (i = 0; i < node->desc->num_fmts; i++) {
		if (node->desc->fmts[i].format == format)
			return &node->desc->fmts[i];
	}

	return NULL;
}

bool is_desc_mode(struct mtk_imgsys_request *req)
{
	/* Mainline port: a request that carries only video buffers (no control
	 * meta) is legal; treat it as std mode instead of dereferencing NULL. */
	struct mtk_imgsys_dev_buffer *meta =
		req->buf_map[MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT];

	if (!meta || !meta->dev_fmt)
		return 0;

	return (meta->dev_fmt->format == V4L2_META_FMT_MTISP_DESC) ? 1 : 0;
}

int is_singledev_mode(struct mtk_imgsys_request *req)
{
	int ret = 0;

	if (req->buf_map[MTK_IMGSYS_VIDEO_NODE_SIGDEV_OUT])
		ret = MTK_IMGSYS_VIDEO_NODE_SIGDEV_OUT;
	else if (req->buf_map[MTK_IMGSYS_VIDEO_NODE_SIGDEV_NORM_OUT])
		ret = MTK_IMGSYS_VIDEO_NODE_SIGDEV_NORM_OUT;

	return ret;
}

bool is_desc_fmt(const struct mtk_imgsys_dev_format *dev_fmt)
{

	bool std_fmt;

	switch (dev_fmt->format) {
	case V4L2_META_FMT_MTISP_DESC:
	case V4L2_META_FMT_MTISP_SD:
	case V4L2_META_FMT_MTISP_DESC_NORM:
	case V4L2_META_FMT_MTISP_SDNORM:
		std_fmt = false;
		break;
	default:
		std_fmt = true;
		break;
	}

	return !std_fmt;
}
/*
 * XAGA dma_buf lifetime accounting.
 *
 * The DIP path hands buffer references between a process-context stage
 * (mtk_imgsys_desc_fill_dmabuf() -> dma_buf_get()), a workqueue stage
 * (mtk_imgsys_desc_iova() -> mtk_imgsys_get_iova()) and a completion stage
 * (mtk_imgsys_iova_map_tbl_unmap() -> mtk_imgsys_put_dma_buf()).  A reference
 * dropped twice - or parked in dev_buf->iova_map_table without ever having
 * been taken - frees the buffer while the DIP is still DMAing into its IOVA.
 * That is invisible to the SMMU (the mapping stays valid) and shows up only as
 * random RAM corruption, a garbled display and a spinning CPU.  Count
 * gets/puts per buffer and shout when the books do not balance.
 *
 * Invariant: gets >= puts + (maps - unmaps).
 */
#define XAGA_ACCT_SLOTS		256
#define XAGA_ACCT_MAX_REPORTS	64

struct xaga_acct_entry {
	struct dma_buf *db;
	int gets;	/* dma_buf_get() taken by this driver */
	int puts;	/* dma_buf_put() done by this driver */
	int maps;	/* references parked in dev_buf->iova_map_table */
	int unmaps;	/* references drained from that table */
};

static struct xaga_acct_entry xaga_acct[XAGA_ACCT_SLOTS];
static DEFINE_SPINLOCK(xaga_acct_lock);
static unsigned int xaga_acct_reports;

static struct xaga_acct_entry *xaga_acct_find(struct dma_buf *db)
{
	int i, slot = -1;

	for (i = 0; i < XAGA_ACCT_SLOTS; i++) {
		if (xaga_acct[i].db == db)
			return &xaga_acct[i];
		if (slot < 0 && !xaga_acct[i].db)
			slot = i;
	}
	if (slot < 0)
		return NULL;
	xaga_acct[slot].db = db;
	return &xaga_acct[slot];
}

static void xaga_acct_event(struct dma_buf *db, const char *tag, int kind)
{
	unsigned long flags;
	struct xaga_acct_entry *e;
	int gets = 0, puts = 0, live = 0;

	if (IS_ERR_OR_NULL(db))
		return;

	spin_lock_irqsave(&xaga_acct_lock, flags);
	e = xaga_acct_find(db);
	if (e) {
		switch (kind) {
		case 0: e->gets++; break;
		case 1: e->puts++; break;
		case 2: e->maps++; break;
		case 3: e->unmaps++; break;
		}
		gets = e->gets;
		puts = e->puts;
		live = e->maps - e->unmaps;
	}
	spin_unlock_irqrestore(&xaga_acct_lock, flags);

	if (e && (puts + live) > gets && xaga_acct_reports < XAGA_ACCT_MAX_REPORTS) {
		xaga_acct_reports++;
		pr_err("XAGA-ACCT[%s] UNBALANCED db=%p gets=%d puts=%d parked=%d -> %d reference(s) released that were never taken (buffer may be freed while the DIP still writes to its IOVA)\n",
		       tag, db, gets, puts, live, (puts + live) - gets);
	}
}

static void xaga_acct_get(struct dma_buf *db, const char *tag)
{
	xaga_acct_event(db, tag, 0);
}

static void xaga_acct_put(struct dma_buf *db, const char *tag)
{
	xaga_acct_event(db, tag, 1);
}

static void xaga_acct_map(struct dma_buf *db, const char *tag)
{
	xaga_acct_event(db, tag, 2);
}

static void xaga_acct_unmap(struct dma_buf *db, const char *tag)
{
	xaga_acct_event(db, tag, 3);
}


static u64 mtk_imgsys_get_iova(struct dma_buf *dma_buf, s32 ionFd,
				struct mtk_imgsys_dev *imgsys_dev,
				struct mtk_imgsys_dev_buffer *dev_buf)
{
	dma_addr_t dma_addr;
	struct device *dev;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	struct mtk_imgsys_pipe *pipe = &imgsys_dev->imgsys_pipe[0];
	struct mtk_imgsys_dma_buf_iova_get_info *iova_info;
	bool cache = false;

	spin_lock(&pipe->iova_cache.lock);
#ifdef LINEAR_CACHE
	list_for_each_entry(iova_info, &pipe->iova_cache.list, list_entry) {
#else
	hash_for_each_possible(pipe->iova_cache.hlists, iova_info, hnode, ionFd) {
#endif
		if ((ionFd == iova_info->ionfd) &&
				(dma_buf == iova_info->dma_buf)) {
			cache = true;
			dma_addr = iova_info->dma_addr;
			dma_buf_put(dma_buf);
			xaga_acct_put(dma_buf, "iova-cache-hit");
			break;
		}
	}
	spin_unlock(&pipe->iova_cache.lock);

	if (cache) {
		dev_dbg(imgsys_dev->dev, "%s fd:%d cache hit\n", __func__, ionFd);
		return dma_addr;
	}

	if (IS_ERR(dma_buf)) {
		dev_dbg(imgsys_dev->dev, "%s: dma_buf 0x%xlx",
							__func__, dma_buf);
		return 0;
	}

	dev = imgsys_dev->dev;

	attach = dma_buf_attach(dma_buf, dev);

	if (IS_ERR(attach))
		goto err_attach;

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);

	if (IS_ERR(sgt))
		goto err_map;

	dma_addr = sg_dma_address(sgt->sgl);

	dev_dbg(imgsys_dev->dev,
		"%s - sg_dma_address : ionFd(%d)-dma_addr:%lx\n",
		__func__, ionFd, dma_addr);

	//add dma_buf_info_list to req for GCECB put it back
	//add dmainfo to ionmaplist
	{
		struct mtk_imgsys_dma_buf_iova_get_info *ion;

		ion = vzalloc(sizeof(*ion));
		ion->ionfd = ionFd;
		ion->dma_addr = dma_addr;
		ion->dma_buf = dma_buf;
		ion->attach = attach;
		ion->sgt = sgt;
		pr_debug("mtk_imgsys_dma_buf_iova_get_info:dma_buf:%lx,attach:%lx,sgt:%lx\n",
				ion->dma_buf, ion->attach, ion->sgt);

		// add data to list head
		spin_lock(&dev_buf->iova_map_table.lock);
		list_add_tail(&ion->list_entry, &dev_buf->iova_map_table.list);
		spin_unlock(&dev_buf->iova_map_table.lock);

		/* XAGA: also make the buffer reachable at its physical address */
		/* the reference taken by the caller is now parked in this entry */
		xaga_acct_map(dma_buf, "iova-map");
	}

	return dma_addr;

err_map:
	pr_info("err_map");
	dma_buf_detach(dma_buf, attach);

err_attach:
	pr_info("err_attach");
	dma_buf_put(dma_buf);
	xaga_acct_put(dma_buf, "iova-error");

	return 0;
}

void mtk_imgsys_put_dma_buf(struct dma_buf *dma_buf,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	if (!IS_ERR(dma_buf)) {

		dma_buf_unmap_attachment(attach, sgt,
			DMA_BIDIRECTIONAL);
		dma_buf_detach(dma_buf, attach);
		dma_buf_put(dma_buf);
		xaga_acct_unmap(dma_buf, "iova-unmap");
		xaga_acct_put(dma_buf, "iova-unmap");
	}
}

void *get_kva(struct mtk_imgsys_dev_buffer *buf, struct iosys_map *map)
{
	struct dma_buf *dmabuf;
	void *vaddr;
	struct header_desc *desc;

	int fd = buf->vbb.vb2_buf.planes[0].m.fd;

	if (!fd) {
		pr_info("%s: vb2buffer fd is 0!!!", __func__);
		return NULL;
	}
	dmabuf = dma_buf_get(buf->vbb.vb2_buf.planes[0].m.fd);
	if (IS_ERR(dmabuf)) {
		pr_info("dmabuf %lx", dmabuf);
		goto ERROR_PUT;
	}
	xaga_acct_get(dmabuf, "kva");

	dma_buf_begin_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	if (dma_buf_vmap(dmabuf, map)) { /* mainline port: iosys_map */
		dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
		dma_buf_put(dmabuf);
		pr_info("%s: dma_buf_vmap fail", __func__);
		return NULL;
	}
	vaddr = map->vaddr;
	desc = (struct header_desc *)vaddr;

	if (IS_ERR(vaddr))
		goto ERROR;

	//Keep dmabuf used in latter for put kva
	buf->dma_buf_putkva = dmabuf;


	return vaddr;

ERROR:
	dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
	pr_info("%s:8", __func__);
ERROR_PUT:
	dma_buf_put(dmabuf);
	xaga_acct_put(dmabuf, "kva-error");
	pr_info("%s:9", __func__);

	return NULL;
}

static void put_kva(struct buf_va_info_t *buf_va_info)
{
	struct dma_buf *dmabuf;

	dmabuf = buf_va_info->dma_buf_putkva;
	if (!IS_ERR(dmabuf)) {
		dma_buf_vunmap(dmabuf, &buf_va_info->map);
		dma_buf_end_cpu_access(dmabuf, DMA_BIDIRECTIONAL);
		dma_buf_unmap_attachment(buf_va_info->attach, buf_va_info->sgt,
			DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, buf_va_info->attach);
		dma_buf_put(dmabuf);
		xaga_acct_put(dmabuf, "put_kva");
	}
}

void flush_fd_kva_list(struct mtk_imgsys_dev *imgsys_dev)
{
	struct buf_va_info_t *buf_va_info = NULL;

	dev_dbg(imgsys_dev->dev, "%s+\n", __func__);
	mutex_lock(&(fd_kva_info_list.mymutex));
	while (!list_empty(&fd_kva_info_list.mylist)) {
		buf_va_info = vlist_node_of(fd_kva_info_list.mylist.next,
			struct buf_va_info_t);
		list_del_init(vlist_link(buf_va_info, struct buf_va_info_t));
		dev_info(imgsys_dev->dev, "%s delete fd(%d) kva(0x%lx), dma_buf_putkva(%p)\n",
				__func__, buf_va_info->buf_fd, buf_va_info->kva,
				buf_va_info->dma_buf_putkva);
		put_kva(buf_va_info);
		vfree(buf_va_info);
		buf_va_info = NULL;
	}
	mutex_unlock(&(fd_kva_info_list.mymutex));

	dev_dbg(imgsys_dev->dev, "%s -\n", __func__);
}

struct fd_kva_list_t *get_fd_kva_list(void)
{
	return &fd_kva_info_list;
}

struct buf_va_info_t *get_first_sd_buf(void)
{
	struct buf_va_info_t_list *buf_entry;

	if (list_empty(&fd_kva_info_list.mylist)) {
		pr_info("%s no buffer found\n", __func__);

		return NULL;
	}

	buf_entry = list_first_entry(&fd_kva_info_list.mylist,
				struct buf_va_info_t_list, link);

	return &buf_entry->node;
}

static void mtk_imgsys_desc_fill_dmabuf(struct mtk_imgsys_pipe *pipe,
				struct frameparams *fparams,
				bool isMENode)
{
	struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0;
	struct dma_buf *dbuf;
	struct v4l2_ext_plane *plane;

	for (i = 0; i < FRAME_BUF_MAX; i++) {
		/* XAGA: num_planes comes from the userspace-shared descriptor; planes[] is only
		 * IMGBUF_MAX_PLANES (3) long, so clamp it or the loop walks past the array and
		 * treats neighbouring descriptor words as fds/pointers. */
		if (fparams->bufs[i].buf.num_planes > IMGBUF_MAX_PLANES) {
			pr_warn("%s: bufs[%d] num_planes=%u > %d, clamping\n", __func__, i,
				fparams->bufs[i].buf.num_planes, IMGBUF_MAX_PLANES);
			fparams->bufs[i].buf.num_planes = IMGBUF_MAX_PLANES;
		}
		for (j = 0; j < fparams->bufs[i].buf.num_planes; j++) {
			plane = &fparams->bufs[i].buf.planes[j];
			if (plane->m.dma_buf.fd == 0)
				continue;
			/* get dma_buf first */
			dbuf = dma_buf_get(plane->m.dma_buf.fd);
			if (IS_ERR(dbuf)) {
				plane->reserved[0] = 0;
				plane->reserved[1] = 0;
				pr_info("%s: dma_buf:%lx fd:%d", __func__, dbuf,
									plane->m.dma_buf.fd);
				continue;
			}

			plane->reserved[0] = (u64)dbuf;
			plane->reserved[1] = (u64)dbuf->size;
			xaga_acct_get(dbuf, "fill");
			if (isMENode) {
				dma_buf_put(dbuf);
				xaga_acct_put(dbuf, "fill-ME");
				/*
				 * XAGA: for the ME node the reference is dropped
				 * here, so reserved[0] is a BORROWED pointer from
				 * now on.  Anything that later feeds it to
				 * mtk_imgsys_get_iova() (which consumes exactly one
				 * reference on every path) over-puts it and frees
				 * the buffer while the engines still DMA into it.
				 */
				dev_dbg(dev,
					"%s: ME node fd=%d dbuf=%p reference released (borrowed)\n",
					__func__, plane->m.dma_buf.fd, dbuf);
			}
			dev_dbg(dev,
				"%s - bufs[%d].buf.planes[%d]: fd(%d), dmabuf(%llx)\n",
				__func__, i, j,
			plane->m.dma_buf.fd,
			plane->reserved[0]);
		}
	}
}

static void mtk_imgsys_kva_cache(struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct list_head *ptr = NULL;
	bool find = false;
	struct buf_va_info_t *buf_va_info;
	struct iosys_map kva_map; /* mainline port: dma_buf_vmap/vunmap handle */

	mutex_lock(&(fd_kva_info_list.mymutex));
	list_for_each(ptr, &(fd_kva_info_list.mylist)) {
		buf_va_info = vlist_node_of(ptr, struct buf_va_info_t);
		if (buf_va_info->buf_fd == dev_buf->vbb.vb2_buf.planes[0].m.fd) {
			find = true;
			break;
		}
	}
	if (find) {
		dev_buf->va_daddr[0] = buf_va_info->kva + dev_buf->dataofst;
		mutex_unlock(&(fd_kva_info_list.mymutex));
		pr_debug(
			"%s : fd(%d), find kva(0x%lx), offset(0x%x) -> (0x%lx)\n",
				__func__, dev_buf->vbb.vb2_buf.planes[0].m.fd,
				buf_va_info->kva, dev_buf->dataofst,
				dev_buf->va_daddr[0]);
	} else {
		mutex_unlock(&(fd_kva_info_list.mymutex));
		dev_buf->va_daddr[0] = (u64)get_kva(dev_buf, &kva_map);

		buf_va_info = (struct buf_va_info_t *)
			vzalloc(sizeof(vlist_type(struct buf_va_info_t)));
		INIT_LIST_HEAD(vlist_link(buf_va_info, struct buf_va_info_t));
		if (buf_va_info == NULL) {
			pr_info("%s: null buf_va_info\n", __func__);
			return;
		}
		buf_va_info->buf_fd = dev_buf->vbb.vb2_buf.planes[0].m.fd;
		buf_va_info->kva = dev_buf->va_daddr[0];
		buf_va_info->map = kva_map;
		buf_va_info->dma_buf_putkva = (void *)dev_buf->dma_buf_putkva;

		mutex_lock(&(fd_kva_info_list.mymutex));
		list_add_tail(vlist_link(buf_va_info, struct buf_va_info_t),
		   &fd_kva_info_list.mylist);
		mutex_unlock(&(fd_kva_info_list.mymutex));
		pr_info(
			"%s : fd(%d), base kva(0x%lx), offset(0x%x), dma_buf_putkva(%p)\n",
				__func__, dev_buf->vbb.vb2_buf.planes[0].m.fd,
				dev_buf->va_daddr[0],
				dev_buf->dataofst, dev_buf->dma_buf_putkva);

		dev_buf->va_daddr[0] += dev_buf->dataofst;
	}
}

/*
 * XAGA-PROBE: sample the content of an ISP output buffer a few seconds after
 * it was handed to the driver, so we can tell whether the DMA engines actually
 * wrote pixels into the buffer userspace receives.  The dma_buf reference is
 * taken here (process context, valid fd table) and dropped in the work item.
 */
struct xaga_probe_work {
	struct delayed_work dwork;
	struct dma_buf *db;
	const char *tag;
	int fd;
};

static void xaga_probe_fn(struct work_struct *work)
{
	struct xaga_probe_work *p = container_of(to_delayed_work(work),
					struct xaga_probe_work, dwork);
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
			pr_info("XAGA-PROBE[%s] fd=%d size=%llu scanned=%u nz=%u zero=%u min=%u max=%u avg=%llu head=%02x%02x%02x%02x %02x%02x%02x%02x\n",
				p->tag, p->fd, (u64)p->db->size, n, nz, zero,
				mn, mx, nz ? div_u64(sum, nz) : 0,
				v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
			dma_buf_vunmap(p->db, &map);
		} else {
			pr_info("XAGA-PROBE[%s] fd=%d vmap BUSY\n", p->tag, p->fd);
		}
		dma_buf_end_cpu_access(p->db, DMA_FROM_DEVICE);
	} else {
		pr_info("XAGA-PROBE[%s] fd=%d begin_cpu_access failed\n",
			p->tag, p->fd);
	}
	dma_buf_put(p->db);
	kfree(p);
}

static DEFINE_SPINLOCK(xaga_probe_lock);
static int xaga_probe_fds[64];
static int xaga_probe_nfd;
static int xaga_probe_total;

static void __maybe_unused xaga_probe_take(int fd, const char *tag)
{
	struct xaga_probe_work *p;
	struct dma_buf *db;
	unsigned long flags;
	int k, dup = 0;

	if (fd <= 0 || xaga_probe_total >= 16)
		return;

	spin_lock_irqsave(&xaga_probe_lock, flags);
	for (k = 0; k < xaga_probe_nfd; k++) {
		if (xaga_probe_fds[k] == fd) {
			dup = 1;
			break;
		}
	}
	if (!dup) {
		if (xaga_probe_nfd >= 64) {
			spin_unlock_irqrestore(&xaga_probe_lock, flags);
			return;
		}
		xaga_probe_fds[xaga_probe_nfd++] = fd;
		xaga_probe_total++;
	}
	spin_unlock_irqrestore(&xaga_probe_lock, flags);
	if (dup)
		return;

	db = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(db)) {
		pr_info("XAGA-PROBE[%s] fd=%d get failed\n", tag, fd);
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
	p->tag = tag;
	p->fd = fd;
	INIT_DELAYED_WORK(&p->dwork, xaga_probe_fn);
	schedule_delayed_work(&p->dwork, msecs_to_jiffies(5000));
	pr_info("XAGA-PROBE[%s] armed fd=%d size=%llu\n", tag, fd, (u64)db->size);
}

static void mtk_imgsys_desc_iova(struct mtk_imgsys_pipe *pipe,
				struct frameparams *fparams,
				struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0;
	struct dma_buf *dmabuf;

	IMGSYS_SYSTRACE_BEGIN("%s\n", __func__);

	dev_dbg(dev, "%s: enter fparams=%p FRAME_BUF_MAX=%d\n",
		__func__, fparams, FRAME_BUF_MAX);
	for (i = 0; i < FRAME_BUF_MAX; i++) {
		if (fparams->bufs[i].buf.num_planes > IMGBUF_MAX_PLANES) {
			pr_warn("%s: bufs[%d] num_planes=%u > %d, clamping\n", __func__, i,
				fparams->bufs[i].buf.num_planes, IMGBUF_MAX_PLANES);
			fparams->bufs[i].buf.num_planes = IMGBUF_MAX_PLANES;
		}
		for (j = 0; j < fparams->bufs[i].buf.num_planes; j++) {
			if (fparams->bufs[i].buf.planes[j].m.dma_buf.fd == 0)
				continue;

			/*
			 * XAGA: this function runs from iova_worker(), i.e. a
			 * workqueue kernel thread.  A workqueue has no user fd
			 * table (current->files == NULL), so dma_buf_get(fd) here
			 * returns -EBADF for *every* fd, always - every DIP plane
			 * then kept a bogus descriptor address, the ME/DIP job
			 * never completed and GCE timed out on SW event 514
			 * (imgsys_cmdq_task_cb "SW event timeout"), which the
			 * vendor HAL turns into an unrecoverable ImgStream error
			 * and a HAL restart.
			 *
			 * The fd must be resolved where the fd is still valid, in
			 * the enqueue/ioctl path: mtk_imgsys_desc_fill_dmabuf()
			 * already did dma_buf_get() there and cached the owned
			 * struct dma_buf * in reserved[0].  Consume that pointer
			 * (do NOT look the fd up again) and hand it to
			 * get_iova(), which takes over the reference - it consumes
			 * exactly one reference on every path (dma_buf_put() on a
			 * cache hit, otherwise it is kept in the iova cache).
			 * This mirrors the downstream driver.
			 */
			dmabuf = (struct dma_buf *)(uintptr_t)
				fparams->bufs[i].buf.planes[j].reserved[0];
			if (IS_ERR_OR_NULL(dmabuf)) {
				pr_info("%s: no cached dmabuf, fd=%d reserved[0]=0x%llx\n",
					__func__,
					fparams->bufs[i].buf.planes[j].m.dma_buf.fd,
					fparams->bufs[i].buf.planes[j].reserved[0]);
				continue;
			}
			fparams->bufs[i].buf.planes[j].reserved[0] =
			mtk_imgsys_get_iova(dmabuf,
			fparams->bufs[i].buf.planes[j].m.dma_buf.fd,
					pipe->imgsys_dev, dev_buf);
			dev_dbg(dev,
				"%s - bufs[%d].buf.planes[%d]: fd(%d), iova(%llx)\n",
				__func__, i, j,
			fparams->bufs[i].buf.planes[j].m.dma_buf.fd,
				fparams->bufs[i].buf.planes[j].reserved[0]);
		}
	}

	IMGSYS_SYSTRACE_END();

}

static void mtk_imgsys_desc_fill_ipi_param(struct mtk_imgsys_pipe *pipe,
					struct mtk_imgsys_dev_buffer *dev_buf,
					struct mtk_imgsys_request *req,
					bool isMENode)
{
	struct device *dev = pipe->imgsys_dev->dev;
	struct header_desc *desc_dma = NULL;
	struct header_desc_norm *desc_norm = NULL;
	unsigned int i = 0, j = 0, tnum, tmax;

	if (dev_buf->vbb.vb2_buf.memory == VB2_MEMORY_DMABUF) {
		if (!req->buf_same) {
			dev_dbg(dev,
			"%s : fd(%d)\n",
				__func__, dev_buf->vbb.vb2_buf.planes[0].m.fd);

			mtk_imgsys_kva_cache(dev_buf);
		} else
			dev_buf->va_daddr[0] = req->buf_va_daddr +
			dev_buf->dataofst;
	} else if (!dev_buf->va_daddr[0]) {
		/* mainline port: MMAP buffers never go through the dmabuf kva
		 * cache and their buf_prepare is not run for request buffers, so
		 * take the kernel VA here. */
		dev_buf->va_daddr[0] =
			(u64)vb2_plane_vaddr(&dev_buf->vbb.vb2_buf, 0);
	}

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	/*getKVA*/
	if (!dev_buf->va_daddr[0]) {
		pr_info("%s:%s fail!! desc_dma is NULL (mem=%d vaddr=0x%llx) !",
			__func__, pipe->desc->name,
			dev_buf->vbb.vb2_buf.memory,
			(u64)vb2_plane_vaddr(&dev_buf->vbb.vb2_buf, 0));
		return;
	}
	//init map table list for each devbuf
	INIT_LIST_HEAD(&dev_buf->iova_map_table.list);
	spin_lock_init(&dev_buf->iova_map_table.lock);

	switch (dev_buf->dev_fmt->format) {
	/* For SMVR */
	default:
	case V4L2_META_FMT_MTISP_DESC:
		desc_dma = (struct header_desc *)dev_buf->va_daddr[0];
		tnum = desc_dma->fparams_tnum;
		tmax = TIME_MAX;
		break;
	/* For Normal */
	case V4L2_META_FMT_MTISP_DESC_NORM:
		desc_norm = (struct header_desc_norm *)dev_buf->va_daddr[0];
		tnum = desc_norm->fparams_tnum;
		tmax = TMAX;
		break;
	}

	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (desc_norm) {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&desc_norm->fparams[i][j], isMENode);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&desc_dma->fparams[i][j], isMENode);
		}
	}

	/* do put_kva once while last user at streamoff */
	/*if (!req->buf_same)*/
	/*	put_kva(dev_buf->dma_buf_putkva, (void *)dev_buf->va_daddr[0]);*/

}

void mtk_imgsys_desc_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct mtk_imgsys_dev_buffer *buf_dma;
	int i = 0;
	bool isMENode = false;
	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	for (i = 0; i < pipe->desc->total_queues; i++) {
		buf_dma = req->buf_map[i];

		if (i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) {
			if (!buf_dma) {
				dev_dbg(dev, "No enqueued tuning buffer\n");
				continue;
			}

		} else if (i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT)
			isMENode = true;

		if (buf_dma) {

			req->buf_same =
				(req->buf_fd == buf_dma->vbb.vb2_buf.planes[0].m.fd)?true:false;

			mtk_imgsys_desc_fill_ipi_param(pipe,
				buf_dma,
				req, isMENode);
		}
	}

	/* do put_kva once while last user at streamoff */
	/*if (buf_ctrlmeta) {*/
	/*	if (buf_ctrlmeta->vbb.vb2_buf.memory == VB2_MEMORY_DMABUF)*/
	/*		put_kva(buf_ctrlmeta->dma_buf_putkva,*/
	/*					(void *)buf_ctrlmeta->va_daddr[0]);*/
	/*}*/
}

static void mtk_imgsys_desc_set(struct mtk_imgsys_pipe *pipe,
				void *src,
				struct header_desc *dst,
				bool iova_need,
				struct mtk_imgsys_dev_buffer *dev_buf)
{

	unsigned int tmax, tnum, t, j;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
	case V4L2_META_FMT_MTISP_DESC:
		input_smvr = (struct header_desc *)src;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
	case V4L2_META_FMT_MTISP_DESC_NORM:
	default:
		input_norm = (struct header_desc_norm *)src;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_smvr) {
		for (t = 0; t < tnum; t++) {
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_smvr->fparams[t][j], dev_buf);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_smvr->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = input_smvr->fparams_tnum;
	} else {
		for (t = 0; t < tnum; t++) {
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_norm->fparams[t][j], dev_buf);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_norm->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = input_norm->fparams_tnum;
	}
}

static void mtk_imgsys_desc_set_skip(struct mtk_imgsys_pipe *pipe,
				int dma,
				void *src,
				struct header_desc *dst,
				bool iova_need,
				struct mtk_imgsys_dev_buffer *dev_buf)
{

	unsigned int tmax, tnum, t, j;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;
	struct singlenode_desc *desc_sd = NULL;
	struct singlenode_desc_norm *desc_sd_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		desc_sd = (struct singlenode_desc *)dev_buf->va_daddr[0];
		input_smvr = (struct header_desc *)src;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
	default:
		desc_sd_norm =
			(struct singlenode_desc_norm *)dev_buf->va_daddr[0];
		input_norm = (struct header_desc_norm *)src;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_smvr) {
		for (t = 0; t < tnum; t++) {
			if (!desc_sd->dmas_enable[dma][t])
				continue;
			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_smvr->fparams[t][j], dev_buf);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_smvr->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = input_smvr->fparams_tnum;
	} else {
		for (t = 0; t < tnum; t++) {
			if (!desc_sd_norm->dmas_enable[dma][t])
				continue;

			for (j = 0; j < SCALE_MAX; j++) {
				if (iova_need)
					mtk_imgsys_desc_iova(pipe,
					&input_norm->fparams[t][j], dev_buf);
				/* TODO: disable memcpy for imgstream fd */
				if (!dst)
					continue;
				memcpy(&dst->fparams[t][j],
						&input_norm->fparams[t][j],
						sizeof(struct frameparams));
			}
		}
		if (dst)
			dst->fparams_tnum = input_norm->fparams_tnum;
	}
}


void mtk_imgsys_desc_map_iova(struct mtk_imgsys_request *req)
{
	unsigned int i;
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *buf_dma;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct header_desc *desc;
	void *desc_dma = NULL;
	bool need_iova = true;
	unsigned int s, e;

	s = ktime_get_boottime_ns()/1000;
	for (i = 0; i < pipe->desc->total_queues; i++) {

		buf_dma = req->buf_map[i];
		if (!buf_dma)
			continue;

		/* XAGA: identify which request DMA slot carries a zero IOVA -
		 * a zero here makes the DIP read from IOVA 0 (IOMMU fault on
		 * LARB10 port16) and stall the whole pipeline. */
		pr_info("%s: buf[%d] %s fmt=0x%x isp=0x%llx va=0x%llx scp=0x%llx ofst=0x%x\n",
			__func__, i, pipe->desc->name,
			buf_dma->dev_fmt ? buf_dma->dev_fmt->format : 0,
			(u64)buf_dma->isp_daddr[0], (u64)buf_dma->va_daddr[0],
			(u64)buf_dma->scp_daddr[0], buf_dma->dataofst);

		if (!buf_dma->va_daddr[0]) {
			pr_info("%s fail!! desc_dma is NULL !", __func__);
			return;
		}

		desc = NULL;
		/* TODO: desc mode */
		/* if (dip_param) */
		/*	dip_param->dmas_enable[i] = 1; */

		if (i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) {
			if (dip_param)
				desc = &dip_param->tuning_meta;
			need_iova = true;
		} else if (i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT) {
			if (dip_param)
				desc = &dip_param->ctrl_meta;
			need_iova = false;
		} else {
			if (dip_param)
				desc = &dip_param->dmas[i];
			need_iova = true;
		}

		desc_dma = (void *)buf_dma->va_daddr[0];
		mtk_imgsys_desc_set(pipe, desc_dma, desc, need_iova, buf_dma);
	}
	e = ktime_get_boottime_ns()/1000;
	pr_debug("%s takes %d ms\n", __func__, (e - s));
}

void mtk_imgsys_sd_desc_map_iova(struct mtk_imgsys_request *req)
{
	unsigned int i;
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct mtk_imgsys_dev_buffer *buf_sd;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct singlenode_desc *desc_sd = NULL;
	struct singlenode_desc_norm *desc_sd_norm = NULL;
	void *src;
	struct header_desc *dst;
	int b;

	b = is_singledev_mode(req);
	buf_sd = req->buf_map[b];
	if (!buf_sd)
		return;

	if (!buf_sd->va_daddr[0]) {
		pr_info("%s fail!! desc_sd is NULL !", __func__);
		return;
	}

	switch (buf_sd->dev_fmt->format) {
	default:
	case V4L2_META_FMT_MTISP_SD:
		desc_sd = (struct singlenode_desc *)buf_sd->va_daddr[0];
		break;
	case V4L2_META_FMT_MTISP_SDNORM:
		desc_sd_norm =
			(struct singlenode_desc_norm *)buf_sd->va_daddr[0];
		break;
	}

	/*
	 * XAGA diagnostic: dump the descriptor the HAL queued before walking it.
	 * The first dwords hold offsets/sizes, and a bogus value used as an
	 * address is what produced the iova_worker oops (fault at 0x2afe54).
	 * Read-only: no behaviour change.
	 */
	{
		u32 *dw = (u32 *)buf_sd->va_daddr[0];

		pr_info("XAGA-SDDESC fmt=0x%x va=0x%llx dip_param=%p sd=%d dw=%08x %08x %08x %08x %08x %08x %08x %08x\n",
			buf_sd->dev_fmt->format,
			(unsigned long long)buf_sd->va_daddr[0],
			dip_param, desc_sd ? 1 : 0,
			dw[0], dw[1], dw[2], dw[3],
			dw[4], dw[5], dw[6], dw[7]);
	}

	/*
	 * XAGA: the reference reports tuning_tnum/ctrl_tnum of 1/1, 4/4, 8/8 and
	 * 12/12 across a session (from its XAGA-DIPMETA print).  Read the same
	 * two fields out of the norm descriptor here so the two kernels can be
	 * compared like for like.
	 */
	if (desc_sd_norm)
		pr_info("XAGA-DIPMETA fmt=0x%x va=0x%llx tuning_tnum=%u ctrl_tnum=%u\n",
			buf_sd->dev_fmt->format,
			(unsigned long long)buf_sd->va_daddr[0],
			desc_sd_norm->tuning_meta.fparams_tnum,
			desc_sd_norm->ctrl_meta.fparams_tnum);

	/*
	 * XAGA: the singledev map writes the built descriptors into the request's
	 * frame-param (dip_param = req->working_buf->frameparam.vaddr).  When that
	 * buffer is not backed (vaddr == 0 - the configuration this port builds
	 * does not allocate kernel-owned working-buffer memory), every
	 * &dip_param->... below is a NULL + deep-offset dereference and the
	 * workqueue oopses (fault seen at 0x2afe54, inside the 2.86 MB
	 * img_ipi_frameparam).  The vendor never reaches this path without an
	 * allocated working buffer, so skip it loudly rather than crash.
	 */
	if (!dip_param) {
		pr_info("XAGA-SDDESC %s: no working frameparam (vaddr=0), skipping desc map\n",
			__func__);
		return;
	}

	for (i = 0; i < IMG_MAX_HW_DMAS; i++) {

		if (desc_sd)
			src = (void *)&desc_sd->dmas[i];
		else
			src = (void *)&desc_sd_norm->dmas[i];

		if (!dip_param)
			dst = NULL;
		else {
			dst = &dip_param->dmas[i];
			/* TODO: desc mode */
			/* dip_param->dmas_enable[i] = 1; */
		}

		mtk_imgsys_desc_set_skip(pipe, i, src, dst, 1, buf_sd);
	}

	/* tuning */
	if (desc_sd)
		src = (void *)&desc_sd->tuning_meta;
	else
		src = (void *)&desc_sd_norm->tuning_meta;

	if (!dip_param)
		dst = NULL;
	else
		dst = &dip_param->tuning_meta;
	mtk_imgsys_desc_set_skip(pipe,
		MTK_IMGSYS_VIDEO_NODE_TUNING_OUT, src, dst, 1, buf_sd);

	/* ctrl_meta */
	if (desc_sd)
		src = (void *)&desc_sd->ctrl_meta;
	else
		src = (void *)&desc_sd_norm->ctrl_meta;

	if (!dip_param)
		dst = NULL;
	else
		dst = &dip_param->ctrl_meta;
	mtk_imgsys_desc_set(pipe, src, dst, 0, buf_sd);

}


static void mtk_imgsys_std2desc_fill_bufinfo(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	int i = 0;
#ifndef USE_V4L2_FMT
	struct v4l2_plane_pix_format *vfmt;
	struct plane_pix_format *bfmt;

	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.width =
						dev_buf->fmt.fmt.pix_mp.width;
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.height =
						dev_buf->fmt.fmt.pix_mp.height;
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.pixelformat =
					dev_buf->fmt.fmt.pix_mp.pixelformat;

	for (i = 0; i < ipidma->fparams[0][0].bufs[0].buf.num_planes; i++) {
		vfmt = &dev_buf->fmt.fmt.pix_mp.plane_fmt[i];
		bfmt =
		&ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i];
		bfmt->sizeimage = vfmt->sizeimage;
		bfmt->bytesperline = vfmt->sizeimage;
	}
#else
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp = dev_buf->fmt.fmt.pix_mp;
#endif
	ipidma->fparams[0][0].bufs[0].crop = dev_buf->crop;
	/* ipidma->fparams[0][0].bufs[0].compose = dev_buf->compose;*/
	ipidma->fparams[0][0].bufs[0].rotation = dev_buf->rotation;
	ipidma->fparams[0][0].bufs[0].hflip = dev_buf->hflip;
	ipidma->fparams[0][0].bufs[0].vflip = dev_buf->vflip;

	dev_dbg(pipe->imgsys_dev->dev,
		"[%s] rotat(%d), hflip(%d), vflip(%d)\n",
		__func__,
		ipidma->fparams[0][0].bufs[0].rotation,
		ipidma->fparams[0][0].bufs[0].hflip,
		ipidma->fparams[0][0].bufs[0].vflip);

	for (i = 0; i < dev_buf->fmt.fmt.pix_mp.num_planes; i++) {
		dev_dbg(pipe->imgsys_dev->dev,
			"[%s] multi-planes : width(%d), width(%d), pixelformat(%d), sizeimage(%d), bytesperline(%d)\n",
			__func__,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.width,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.height,
		ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.pixelformat,
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i].sizeimage,
	ipidma->fparams[0][0].bufs[0].fmt.fmt.pix_mp.plane_fmt[i].bytesperline);
	}
}

static void mtk_imgsys_meta_std2desc_fill_extbuf(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.fd =
		dev_buf->vbb.vb2_buf.planes[0].m.fd;
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.offset =
		dev_buf->vbb.vb2_buf.planes[0].m.offset;
	ipidma->fparams[0][0].bufs[0].buf.planes[0].reserved[0] =
		(uint32_t)dev_buf->isp_daddr[0];
	dev_dbg(dev,
		"[%s]multi-plane[%d]:fd(%d),offset(%d),iova(0x%llx)\n",
		__func__, 0,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.fd,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.offset,
	ipidma->fparams[0][0].bufs[0].buf.planes[0].reserved[0]);
}


static void mtk_imgsys_std2desc_fill_extbuf(struct mtk_imgsys_pipe *pipe,
					struct header_desc *ipidma,
					struct mtk_imgsys_dev_buffer *dev_buf)
{
	struct device *dev = pipe->imgsys_dev->dev;
	int i = 0;

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		dev_buf->scp_daddr[0],
		dev_buf->isp_daddr[0],
		dev_buf->va_daddr[0]);

	for (i = 0; i < dev_buf->fmt.fmt.pix_mp.num_planes; i++) {
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.fd =
			dev_buf->vbb.vb2_buf.planes[i].m.fd;
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.offset =
			dev_buf->vbb.vb2_buf.planes[i].m.offset;
		ipidma->fparams[0][0].bufs[0].buf.planes[i].reserved[0] =
			(uint32_t)dev_buf->isp_daddr[i];
		dev_dbg(dev,
			"[%s]multi-plane[%d]:fd(%d),offset(%d),iova(0x%llx)\n",
			__func__, i,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.fd,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].m.dma_buf.offset,
		ipidma->fparams[0][0].bufs[0].buf.planes[i].reserved[0]);
	}

}

void mtk_imgsys_std_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct img_ipi_frameparam *dip_param = req->working_buf->frameparam.vaddr;
	struct mtk_imgsys_dev_buffer *buf_dma;
	struct mtk_imgsys_dev_buffer *buf_tuning;
	struct mtk_imgsys_dev_buffer *buf_ctrlmeta;
	int i = 0;

	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	memset(dip_param, 0, sizeof(*dip_param));

	/* Tuning buffer */
	buf_tuning =
		req->buf_map[MTK_IMGSYS_VIDEO_NODE_TUNING_OUT];
	if (buf_tuning) {
		dev_dbg(dev,
			"Tuning buf queued: scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
			buf_tuning->scp_daddr[0],
			buf_tuning->isp_daddr[0],
			buf_tuning->va_daddr[0]);

		mtk_imgsys_meta_std2desc_fill_extbuf(pipe,
			&dip_param->tuning_meta,
			buf_tuning);

	} else {
		dev_dbg(dev,
			"No enqueued tuning buffer\n");
	}

	buf_ctrlmeta = req->buf_map[MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT];
	if (buf_ctrlmeta) {
		/* TODO: kva */
		dev_dbg(dev,
			"Ctrlmeta buf queued: scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
			buf_ctrlmeta->scp_daddr[0],
			buf_ctrlmeta->isp_daddr[0],
			buf_ctrlmeta->va_daddr[0]);

		mtk_imgsys_meta_std2desc_fill_extbuf(pipe,
			&dip_param->ctrl_meta,
			buf_ctrlmeta);

		pr_info("%s: ctrl_meta fd=%d va=0x%llx off_ctrlmeta=%ld cm12=0x%x cm16=0x%x cm36=0x%x\n",
			__func__,
			dip_param->ctrl_meta.fparams[0][0].bufs[0].buf.planes[0].m.dma_buf.fd,
			buf_ctrlmeta->va_daddr[0],
			(long)((u8 *)&dip_param->ctrl_meta - (u8 *)dip_param),
			*(u32 *)((u8 *)&dip_param->ctrl_meta + 12),
			*(u32 *)((u8 *)&dip_param->ctrl_meta + 16),
			*(u32 *)((u8 *)&dip_param->ctrl_meta + 36));
	}
	for (i = 0; i < pipe->desc->total_queues; i++) {
		if ((i == MTK_IMGSYS_VIDEO_NODE_TUNING_OUT) ||
			(i == MTK_IMGSYS_VIDEO_NODE_CTRLMETA_OUT))
			continue;

		buf_dma = req->buf_map[i];
		if (buf_dma) {
			mtk_imgsys_std2desc_fill_extbuf(pipe,
				&dip_param->dmas[i],
				buf_dma);
			mtk_imgsys_std2desc_fill_bufinfo(pipe,
				&dip_param->dmas[i],
				buf_dma);
		}
	}
}

static void mtk_imgsys_singledevice_fill_ipi_param(struct mtk_imgsys_pipe *pipe,
					void *sd_dma,
					struct mtk_imgsys_dev_buffer *dev_buf,
					bool isMENode)
{
	//struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0, tnum, tmax;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		input_smvr = (struct header_desc *)sd_dma;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		input_norm = (struct header_desc_norm *)sd_dma;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_norm) {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_norm->fparams[i][j], isMENode);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_smvr->fparams[i][j], isMENode);
		}
	}
}

static void mtk_imgsys_sd_fill_dmas(struct mtk_imgsys_pipe *pipe, int dma,
					void *sd_dma,
					struct mtk_imgsys_dev_buffer *dev_buf,
					bool isMENode)
{
	//struct device *dev = pipe->imgsys_dev->dev;
	unsigned int i = 0, j = 0, tnum, tmax;
	struct header_desc *input_smvr = NULL;
	struct header_desc_norm *input_norm = NULL;
	struct singlenode_desc *sd_desc = NULL;
	struct singlenode_desc_norm *sd_desc_norm = NULL;

	switch (dev_buf->dev_fmt->format) {
	case V4L2_META_FMT_MTISP_SD:
		sd_desc =
			(struct singlenode_desc *)dev_buf->va_daddr[0];
		input_smvr = (struct header_desc *)sd_dma;
		tmax = TIME_MAX;
		tnum = input_smvr->fparams_tnum;
		break;
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		sd_desc_norm =
			(struct singlenode_desc_norm *)dev_buf->va_daddr[0];
		input_norm = (struct header_desc_norm *)sd_dma;
		tmax = TMAX;
		tnum = input_norm->fparams_tnum;
		break;
	}
	/*fd2iova : need 2D array parsing*/
	if (tnum >= tmax) {
		tnum = tmax;
		pr_debug("%s: %d bufinfo enqueued exceeds %d\n", __func__,
			tnum, tmax);
	}

	if (input_norm) {
		for (i = 0; i < tnum ; i++) {
			if (!sd_desc_norm->dmas_enable[dma][i])
				continue;
			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_norm->fparams[i][j], isMENode);
		}
	} else {
		for (i = 0; i < tnum ; i++) {
			if (!sd_desc->dmas_enable[dma][i])
				continue;

			for (j = 0; j < SCALE_MAX; j++)
				mtk_imgsys_desc_fill_dmabuf(pipe,
					&input_smvr->fparams[i][j], isMENode);
		}
	}
}


void mtk_imgsys_singledevice_ipi_params_config(struct mtk_imgsys_request *req)
{
	struct mtk_imgsys_pipe *pipe = req->imgsys_pipe;
	struct device *dev = pipe->imgsys_dev->dev;
	struct mtk_imgsys_dev_buffer *buf_in;
	struct singlenode_desc *singledevice_desc_dma = NULL;
	struct singlenode_desc_norm *singledevice_desc_norm = NULL;
	void *tuning_meta, *ctrl_meta;
	int i = 0;
	bool isMENode = false;

	dev_dbg(dev, "%s:%s: pipe-job id(%d)\n", __func__,
		pipe->desc->name, req->id);

	i = is_singledev_mode(req);
	buf_in = req->buf_map[i];
	if (!buf_in)
		return;

	if (buf_in->vbb.vb2_buf.memory == VB2_MEMORY_DMABUF)
		mtk_imgsys_kva_cache(buf_in);
	/* TODO:  MMAP imgbuffer path vaddr = 0 */

	dev_dbg(dev,
		"%s : scp_daddr(0x%llx),isp_daddr(0x%llx),va_daddr(0x%llx)\n",
		__func__,
		buf_in->scp_daddr[0],
		buf_in->isp_daddr[0],
		buf_in->va_daddr[0]);

	/*getKVA*/
	if (!buf_in->va_daddr[0]) {
		pr_info("%s fail!! desc_dma is NULL !", __func__);
		return;
	}

	//init map table list for each devbuf
	INIT_LIST_HEAD(&buf_in->iova_map_table.list);
	spin_lock_init(&buf_in->iova_map_table.lock);

	//tuning_meta
	switch (buf_in->dev_fmt->format) {
	/* SMVR */
	case V4L2_META_FMT_MTISP_SD:
		singledevice_desc_dma =
			(struct singlenode_desc *)buf_in->va_daddr[0];
		tuning_meta = (void *) &singledevice_desc_dma->tuning_meta;
		ctrl_meta = (void *) &singledevice_desc_dma->ctrl_meta;
		break;
	/* NORM */
	default:
	case V4L2_META_FMT_MTISP_SDNORM:
		singledevice_desc_norm =
			(struct singlenode_desc_norm *)buf_in->va_daddr[0];
		tuning_meta = (void *) &singledevice_desc_norm->tuning_meta;
		ctrl_meta = (void *) &singledevice_desc_norm->ctrl_meta;
		break;
	}
	mtk_imgsys_sd_fill_dmas(pipe,
			MTK_IMGSYS_VIDEO_NODE_TUNING_OUT,
			tuning_meta,
			buf_in, isMENode); //ME's tuning need fd2iova????

	mtk_imgsys_singledevice_fill_ipi_param(pipe,
			ctrl_meta,
			buf_in, 1);

	for (i = 0; i < IMG_MAX_HW_DMAS; i++) {

		if (singledevice_desc_dma)
			mtk_imgsys_sd_fill_dmas(pipe, i,
				(void *)&singledevice_desc_dma->dmas[i],
				buf_in, isMENode);
		else
			mtk_imgsys_sd_fill_dmas(pipe, i,
				(void *)&singledevice_desc_norm->dmas[i],
				buf_in, isMENode);
	}
}

void mtk_imgsys_pipe_try_enqueue(struct mtk_imgsys_pipe *pipe)
{
	struct mtk_imgsys_request *req = NULL;
	unsigned long flag;

	if (!pipe->streaming)
		return;

	spin_lock_irqsave(&pipe->pending_job_lock, flag);
	if (list_empty(&pipe->pipe_job_pending_list)) {
		spin_unlock_irqrestore(&pipe->pending_job_lock, flag);
		pr_info("%s: no requests", __func__);
		return;
	}
	req = list_first_entry(&pipe->pipe_job_pending_list, struct mtk_imgsys_request, list);
	list_del(&req->list);
	pipe->num_pending_jobs--;
	spin_unlock_irqrestore(&pipe->pending_job_lock, flag);

	spin_lock_irqsave(&pipe->running_job_lock, flag);
#ifdef JOB_RUN
	list_add_tail(&req->list,
		      &pipe->pipe_job_running_list);
#endif
	pipe->num_jobs++;
	spin_unlock_irqrestore(&pipe->running_job_lock, flag);

	mtk_imgsys_hw_enqueue(pipe->imgsys_dev, req);
	dev_dbg(pipe->imgsys_dev->dev,
		"%s:%s: pending jobs(%d), running jobs(%d)\n",
		__func__, pipe->desc->name, pipe->num_pending_jobs,
		pipe->num_jobs);
}
