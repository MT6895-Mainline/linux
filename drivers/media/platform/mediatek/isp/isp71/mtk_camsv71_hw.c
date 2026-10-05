// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek ISP 7.1 CAMSV raw capture: register sequences and interrupt
 *
 * The register programming follows MediaTek's ISP 7.1 CAMSV driver
 * (mtk_cam-sv.c) in the order verified on MT6895: reset without the reset of
 * the paired CAMSV, configuration, the first buffer, arming, and a stop at a
 * frame boundary. Buffer addresses, tags and frame buffer credits written
 * during frame n take effect at the start of frame (SOF) n + 1. At most one
 * buffer is programmed ahead: by the SOF handler, or when it is queued if the
 * SOF found none. A buffer completes at SW_PASS1_DON. Without a credit the
 * hardware drops the frame.
 *
 * Copyright (c) 2019 MediaTek Inc.
 * Copyright (c) 2025 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/moduleparam.h>

#include <media/v4l2-event.h>

#include "mtk_camsv71.h"
#include "mtk_camsv71_regs.h"

/* Errors that spoil one frame: its buffer is returned with ERROR. */
#define MTK_CAMSV_INT_FRAME_ERR		(CAMSV_INT_TG_ERR | CAMSV_INT_TG_GBERR | \
					 CAMSV_INT_TG_SOF_DROP | CAMSV_INT_DMA_ERR | \
					 CAMSV_INT_IMGO_OVERR)
/* Errors after which the double-buffer or credit state is unknown. */
#define MTK_CAMSV_INT_FATAL		(CAMSV_INT_DB_LOAD_ERR | CAMSV_INT_SW_ENQUE_ERR)
/* A written frame ends with both, a dropped frame with PASS1_DON only. */
#define MTK_CAMSV_INT_FRAME_END		(CAMSV_INT_PASS1_DON | CAMSV_INT_SW_PASS1_DON)

#define MTK_CAMSV_MAX_BAD_RUN		16
#define MTK_CAMSV_PAK_IN_BIT		14
#define MTK_CAMSV_RESET_TIMEOUT_US	100000
#define MTK_CAMSV_TG_POLL_US		50
#define MTK_CAMSV_TG_IDLE_TIMEOUT_US	500000
#define MTK_CAMSV_STOP_MIN_MS		200
#define MTK_CAMSV_STOP_MAX_MS		2000
#define MTK_CAMSV_MAX_GUARD_US		5000

/* Early-credit switches, read each time a buffer is queued. */
static bool early_credit = true;
module_param(early_credit, bool, 0644);
MODULE_PARM_DESC(early_credit,
		 "Program a buffer queued with nothing pending at once (default: on)");

static unsigned int early_guard_us = 200;
module_param(early_guard_us, uint, 0644);
MODULE_PARM_DESC(early_guard_us,
		 "Leave buffers queued this close to the expected frame start to it (default: 200, 0: off, max 5000)");

static void mtk_camsv_rmw(struct mtk_camsv *cam, u32 reg, u32 mask, u32 val)
{
	writel((readl(cam->base + reg) & ~mask) | (val & mask), cam->base + reg);
}

/* Double-buffer load of the TG registers, VF_DATA_EN among them. */
static void mtk_camsv_tg_db_load(struct mtk_camsv *cam)
{
	mtk_camsv_rmw(cam, REG_CAMSV_TG_PATH_CFG, CAMSV_TG_PATH_CFG_DB_LOAD_DIS,
		      CAMSV_TG_PATH_CFG_DB_LOAD_DIS);
	mtk_camsv_rmw(cam, REG_CAMSV_TG_PATH_CFG, CAMSV_TG_PATH_CFG_DB_LOAD_DIS, 0);
}

/* DB_EN must go off and on again before VF_DATA_EN is set. */
static void mtk_camsv_db_load(struct mtk_camsv *cam)
{
	mtk_camsv_rmw(cam, REG_CAMSV_MODULE_EN, CAMSV_MODULE_EN_DB_EN, 0);
	mtk_camsv_rmw(cam, REG_CAMSV_MODULE_EN, CAMSV_MODULE_EN_DB_EN,
		      CAMSV_MODULE_EN_DB_EN);
}

static void mtk_camsv_vf_off(struct mtk_camsv *cam)
{
	mtk_camsv_rmw(cam, REG_CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_VFDATA_EN, 0);
	mtk_camsv_tg_db_load(cam);
}

static bool mtk_camsv_capturing(struct mtk_camsv *cam)
{
	return cam->state == MTK_CAMSV_RUNNING || cam->state == MTK_CAMSV_STOPPING;
}

/* Both windows: no TG, IMGO or UFE, no CMOS, no VF and no interrupt enabled. */
bool mtk_camsv71_hw_idle(struct mtk_camsv *cam)
{
	void __iomem * const win[] = { cam->base, cam->inner };
	u32 module, sen, vf, int_en;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(win); i++) {
		module = readl(win[i] + REG_CAMSV_MODULE_EN);
		sen = readl(win[i] + REG_CAMSV_TG_SEN_MODE);
		vf = readl(win[i] + REG_CAMSV_TG_VF_CON);
		int_en = readl(win[i] + REG_CAMSV_INT_EN);
		if ((module & (CAMSV_MODULE_EN_TG_EN | CAMSV_MODULE_EN_IMGO_EN |
			       CAMSV_MODULE_EN_UFE_EN)) ||
		    (sen & CAMSV_TG_SEN_MODE_CMOS_EN) ||
		    (vf & CAMSV_TG_VF_CON_VFDATA_EN) || int_en) {
			dev_err(cam->dev,
				"%s window busy: module %#x sen %#x vf %#x int_en %#x\n",
				i ? "inner" : "outer", module, sen, vf, int_en);
			return false;
		}
	}
	return true;
}

/* IMGO DMA and FBC reset; never SW_CTL = 4, which also resets the paired CAMSV. */
static int mtk_camsv_reset(struct mtk_camsv *cam)
{
	u32 val;
	int ret;

	writel(0, cam->base + REG_CAMSV_SW_CTL);
	writel(CAMSV_SW_CTL_IMGO_RST_TRIG, cam->base + REG_CAMSV_SW_CTL);
	ret = readl_poll_timeout(cam->base + REG_CAMSV_SW_CTL, val,
				 val & CAMSV_SW_CTL_IMGO_RST_ST, 1,
				 MTK_CAMSV_RESET_TIMEOUT_US);
	if (ret) {
		dev_err(cam->dev, "IMGO reset timed out (SW_CTL %#x)\n", val);
		return ret;
	}
	writel(CAMSV_FBC_RESET, cam->base + REG_CAMSV_FBC_IMGO_CTL1);
	writel(0, cam->base + REG_CAMSV_SW_CTL);
	return 0;
}

/*
 * TG, packer and IMGO DMA configuration, in the verified order and with full
 * register writes. The DMA geometry comes from the video format, which sized
 * the buffers.
 */
static void mtk_camsv_hw_config(struct mtk_camsv *cam)
{
	const struct mtk_camsv_soc *soc = cam->soc;
	const struct mtk_camsv_format *fmt = cam->fmt;
	const struct v4l2_pix_format *pix = &cam->pix;
	unsigned int i;

	writel(0, cam->base + REG_CAMSV_TG_VF_CON);
	writel(CAMSV_TG_SEN_MODE_TIME_STP_EN |
	       FIELD_PREP(CAMSV_TG_SEN_MODE_DBL_DATA_BUS, soc->pixel_mode),
	       cam->base + REG_CAMSV_TG_SEN_MODE);
	writel(0, cam->base + REG_CAMSV_TG_SUB_PERIOD);
	writel(0, cam->base + REG_CAMSV_TG_PATH_CFG);
	writel(FIELD_PREP(CAMSV_TG_GRAB_END, pix->width),
	       cam->base + REG_CAMSV_TG_SEN_GRAB_PXL);
	writel(FIELD_PREP(CAMSV_TG_GRAB_END, pix->height),
	       cam->base + REG_CAMSV_TG_SEN_GRAB_LIN);
	writel(FIELD_PREP(CAMSV_MODULE_EN_DB_LOAD_SRC, CAMSV_DB_LOAD_SRC_SUB_SOF) |
	       CAMSV_MODULE_EN_PAK_EN | CAMSV_MODULE_EN_TG_EN,
	       cam->base + REG_CAMSV_MODULE_EN);
	writel(0, cam->base + REG_CAMSV_SUB_CTRL);
	writel(0, cam->base + REG_CAMSV_DCIF_SET);
	writel(FIELD_PREP(CAMSV_FMT_SEL_TG1_FMT, fmt->tg_fmt),
	       cam->base + REG_CAMSV_FMT_SEL);
	writel(0, cam->base + REG_CAMSV_INT_EN);
	writel(FIELD_PREP(CAMSV_PAK_CON_PAK_IN_BIT, MTK_CAMSV_PAK_IN_BIT),
	       cam->base + REG_CAMSV_PAK_CON);
	writel(FIELD_PREP(CAMSV_PAK_MODE, fmt->pak_mode) |
	       FIELD_PREP(CAMSV_PAK_DBL_MODE, soc->pixel_mode),
	       cam->base + REG_CAMSV_PAK);
	writel(soc->special_fun_en, cam->base + REG_CAMSV_SPECIAL_FUN_EN);
	writel(mtk_camsv_xsize(cam, fmt, pix->width) - 1,
	       cam->base + REG_CAMSV_IMGO_XSIZE);
	writel(pix->height - 1, cam->base + REG_CAMSV_IMGO_YSIZE);
	writel(pix->bytesperline, cam->base + REG_CAMSV_IMGO_STRIDE);
	writel(0, cam->base + REG_CAMSV_IMGO_CROP);
	writel(0, cam->base + REG_CAMSV_IMGO_OFST_ADDR);
	writel(0, cam->base + REG_CAMSV_IMGO_OFST_ADDR_MSB);
	for (i = 0; i < CAMSV_IMGO_NUM_CON; i++)
		writel(soc->imgo_con[i], cam->base + REG_CAMSV_IMGO_CON(i));
	writel(0, cam->base + REG_CAMSV_FBC_IMGO_CTL1);
	mtk_camsv_rmw(cam, REG_CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_CMOS_EN,
		      CAMSV_TG_SEN_MODE_CMOS_EN);
	mtk_camsv_rmw(cam, REG_CAMSV_MODULE_EN, CAMSV_MODULE_EN_IMGO_EN,
		      CAMSV_MODULE_EN_IMGO_EN);
	writel(CAMSV_FBC_EN, cam->base + REG_CAMSV_FBC_IMGO_CTL1);
}

/*
 * One buffer, one credit: tag, address, then RCNT_INC1 (MediaTek
 * mtk_cam_sv_enquehwbuf()). The caller holds cam->lock, the CAMSV captures
 * and no other programming is pending. A SOF copies the outer tag and address
 * that reached the CAMSV before it into the inner window and applies the
 * credit written before it. The read-back returns only once the tag and the
 * address have reached the CAMSV, so the SOF that applies the credit also
 * latches them; a SOF between the two halves latches the tag without the
 * credit (a straddle), which mtk_camsv_sof() recognises.
 */
static void mtk_camsv_program(struct mtk_camsv *cam,
			      struct mtk_camsv_buffer *buf, u32 tag, bool early)
{
	buf->tag = tag;
	buf->early = early;
	buf->missed = false;
	writel(tag, cam->base + REG_CAMSV_FRAME_SEQ_NO);
	writel(lower_32_bits(buf->addr), cam->base + REG_CAMSV_IMGO_BASE_ADDR);
	writel(FIELD_PREP(CAMSV_IMGO_ADDR_MSB, upper_32_bits(buf->addr)),
	       cam->base + REG_CAMSV_IMGO_BASE_ADDR_MSB);
	readl(cam->base + REG_CAMSV_IMGO_BASE_ADDR_MSB);
	writel(CAMSV_RCNT_INC1, cam->base + REG_CAMSV_TOP_FBC_CNT_SET);
	buf->credit_ts = ktime_get_ns();
	cam->pending = buf;
	cam->stats.credits++;
	if (early)
		cam->stats.early++;
}

static void mtk_camsv_program_next(struct mtk_camsv *cam, bool early)
{
	struct mtk_camsv_buffer *buf;

	buf = list_first_entry_or_null(&cam->queued, struct mtk_camsv_buffer,
				       list);
	if (!buf)
		return;		/* no credit: the hardware drops the next frame */
	list_del(&buf->list);
	mtk_camsv_program(cam, buf, cam->frame + 1, early);
}

static dma_addr_t mtk_camsv_inner_addr(struct mtk_camsv *cam)
{
	u32 msb = readl(cam->inner + REG_CAMSV_IMGO_BASE_ADDR_MSB);

	return (dma_addr_t)FIELD_GET(CAMSV_IMGO_ADDR_MSB, msb) << 32 |
	       readl(cam->inner + REG_CAMSV_IMGO_BASE_ADDR);
}

/*
 * The hardware no longer matches the buffer model, or reported a fatal error:
 * stop capturing and keep every buffer it may still own until the reset at
 * stream off.
 */
static void mtk_camsv_fail(struct mtk_camsv *cam, u32 status, const char *why)
{
	mtk_camsv_vf_off(cam);
	writel(0, cam->base + REG_CAMSV_INT_EN);
	cam->state = MTK_CAMSV_FAILED;
	dev_err_ratelimited(cam->dev, "%s (status %#x, frame %u): capture stopped\n",
			    why, status, cam->frame);
	vb2_queue_error(&cam->queue);
	wake_up(&cam->stop_wq);
}

/*
 * Frame start. It applied the pending credit or none: RCNT moves only at a
 * SOF, by the credits written before it, and at most one is outstanding. The
 * credit implies the tag and the address, which reached the CAMSV before it,
 * and the inner window must show both. Without the credit this frame is
 * dropped: if the pending tag latched alone, the SOF fell between the tag and
 * the credit (a straddle), otherwise the buffer was programmed after the SOF
 * and before this interrupt; either way the next SOF applies the credit.
 */
static void mtk_camsv_sof(struct mtk_camsv *cam, u32 status, u64 ts)
{
	struct v4l2_event ev = { .type = V4L2_EVENT_FRAME_SYNC };
	struct mtk_camsv_buffer *buf = cam->pending;
	u8 rcnt, credits;
	u32 tag;

	if (cam->active) {
		mtk_camsv_fail(cam, status, "frame start before the frame end");
		return;
	}
	if (cam->last_sof_ts)
		cam->period_ns = ts - cam->last_sof_ts;
	cam->last_sof_ts = ts;
	cam->frame++;
	cam->in_frame = true;

	rcnt = FIELD_GET(CAMSV_FBC_RCNT,
			 readl(cam->base + REG_CAMSV_FBC_IMGO_CTL2));
	credits = rcnt - cam->rcnt;
	cam->rcnt = rcnt;
	tag = readl(cam->inner + REG_CAMSV_FRAME_SEQ_NO);

	if (credits > 1 || (credits && !buf)) {
		mtk_camsv_fail(cam, status, "credit count out of step");
		return;
	}
	if (credits) {
		if (tag != buf->tag || mtk_camsv_inner_addr(cam) != buf->addr) {
			mtk_camsv_fail(cam, status, "credit applied without its buffer");
			return;
		}
		if (status & CAMSV_INT_IMGO_DROP) {
			mtk_camsv_fail(cam, status, "frame with a credit dropped");
			return;
		}
		if (buf->early && !buf->missed) {
			cam->stats.early_hit++;
			cam->stats.lead_us = min_t(u64, cam->stats.lead_us,
						   div_u64(ts - min(ts, buf->credit_ts),
							   NSEC_PER_USEC));
		}
		cam->pending = NULL;
		cam->active = buf;
		buf->frame = cam->frame;
		buf->ts = ts;
	} else if (!buf) {
		cam->stats.drops++;
		cam->stats.drop_nobuf++;
	} else {
		if (buf->missed) {
			/* Its writes reached the CAMSV before the last SOF. */
			mtk_camsv_fail(cam, status, "credit not applied");
			return;
		}
		buf->missed = true;
		cam->stats.drops++;
		if (tag == buf->tag)
			cam->stats.drop_straddle++;
		else
			cam->stats.drop_after++;
	}

	ev.u.frame_sync.frame_sequence = cam->frame - 1;
	v4l2_event_queue(cam->subdev.devnode, &ev);

	if (cam->state == MTK_CAMSV_RUNNING && !cam->pending)
		mtk_camsv_program_next(cam, false);
}

/* A frame that started before VF off took effect: no credit was outstanding. */
static void mtk_camsv_sof_after_stop(struct mtk_camsv *cam)
{
	cam->frame++;
	cam->stats.drops++;
	cam->stats.drop_nobuf++;
}

/*
 * End of the current frame. With @next_sof the following SOF happened before
 * the status was read: the inner tag and the frame size then already describe
 * the next frame and only the write count can be checked.
 */
static void mtk_camsv_eof(struct mtk_camsv *cam, u32 status, bool next_sof)
{
	struct mtk_camsv_buffer *buf = cam->active;
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	bool written = status & CAMSV_INT_SW_PASS1_DON;
	u8 wcnt, consumed;

	/*
	 * The done bits of a written frame arrive together, but should a read
	 * catch PASS1_DON alone, wait for SW_PASS1_DON: a frame start before
	 * it fails the stream.
	 */
	if (buf && !written)
		return;

	wcnt = FIELD_GET(CAMSV_FBC_WCNT,
			 readl(cam->base + REG_CAMSV_FBC_IMGO_CTL2));
	consumed = wcnt - cam->wcnt;
	cam->wcnt = wcnt;
	cam->in_frame = false;

	/* A written frame used the latched credit, a dropped frame none. */
	if (written != !!buf || consumed != written) {
		mtk_camsv_fail(cam, status, "frame end does not match the credits");
		return;
	}

	if (buf) {
		cam->active = NULL;
		if (cam->frame_err ||
		    (!next_sof &&
		     (readl(cam->inner + REG_CAMSV_FRAME_SEQ_NO) != buf->tag ||
		      readl(cam->base + REG_CAMSV_TG_FRMSIZE_ST_R) != cam->frame_size)))
			state = VB2_BUF_STATE_ERROR;
		buf->vb.vb2_buf.timestamp = buf->ts;
		buf->vb.sequence = buf->frame - 1;
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		cam->stats.done++;
	}

	if (cam->frame_err || state == VB2_BUF_STATE_ERROR) {
		cam->frame_err = 0;
		cam->stats.bad++;
		if (++cam->bad_run >= MTK_CAMSV_MAX_BAD_RUN) {
			mtk_camsv_fail(cam, status, "too many bad frames in a row");
			return;
		}
	} else {
		cam->bad_run = 0;
	}

	/* Frame boundary with no credit outstanding: the verified stop point. */
	if (cam->state == MTK_CAMSV_STOPPING && !cam->pending) {
		mtk_camsv_vf_off(cam);
		cam->state = MTK_CAMSV_STOPPED;
		wake_up(&cam->stop_wq);
	}
}

irqreturn_t mtk_camsv71_isr(int irq, void *data)
{
	struct mtk_camsv *cam = data;
	u64 ts = ktime_get_ns();
	bool sof, eof;
	u32 status;

	status = readl(cam->base + REG_CAMSV_INT_STATUS);	/* read-clear */
	if (!status)
		return IRQ_NONE;

	spin_lock(&cam->lock);
	cam->stats.irq_status |= status;
	if (status & CAMSV_INT_IMGO_DROP)
		cam->stats.imgo_drop++;
	if (!mtk_camsv_capturing(cam)) {
		/* Nothing is expected after VF off: mask anything further. */
		writel(0, cam->base + REG_CAMSV_INT_EN);
		if (cam->state == MTK_CAMSV_STOPPED && (status & CAMSV_INT_TG_SOF))
			mtk_camsv_sof_after_stop(cam);
		goto out;
	}
	if (status & MTK_CAMSV_INT_FATAL) {
		mtk_camsv_fail(cam, status, "double-buffer load or enqueue error");
		goto out;
	}

	cam->frame_err |= status & MTK_CAMSV_INT_FRAME_ERR;
	sof = status & CAMSV_INT_TG_SOF;
	eof = status & MTK_CAMSV_INT_FRAME_END;

	/*
	 * A late handler sees the end of frame n with the start of frame
	 * n + 1, a very late one the start and the end of one frame.
	 */
	if (sof && eof && !cam->in_frame) {
		mtk_camsv_sof(cam, status, ts);
		sof = false;
	}
	if (eof && mtk_camsv_capturing(cam))
		mtk_camsv_eof(cam, status, sof);
	if (sof && mtk_camsv_capturing(cam))
		mtk_camsv_sof(cam, status, ts);
	else if (sof && cam->state == MTK_CAMSV_STOPPED)
		mtk_camsv_sof_after_stop(cam);
out:
	spin_unlock(&cam->lock);
	return IRQ_HANDLED;
}

/*
 * Within early_guard_us of the expected next SOF, or past it before its
 * interrupt. The prediction is the last SOF interrupt time plus the last
 * period. A straddle is detected and safe only if the inner address copy and
 * the FBC credit take effect at the same frame start, which has not been
 * measured at the microsecond scale; the guard keeps early credits out of
 * that window.
 */
static bool mtk_camsv_near_sof(struct mtk_camsv *cam)
{
	u64 guard = (u64)min(READ_ONCE(early_guard_us), MTK_CAMSV_MAX_GUARD_US) *
		    NSEC_PER_USEC;

	if (!guard)
		return false;
	if (!cam->period_ns)
		return true;
	return ktime_get_ns() + guard >= cam->last_sof_ts + cam->period_ns;
}

/*
 * buf_queue: a buffer queued while capturing with nothing pending is
 * programmed at once and normally used from the next SOF; otherwise the SOF
 * handler programs the queued buffers in order. RUNNING implies that the
 * CAMSV is powered and armed: every transition out of it holds cam->lock, and
 * vb2 serialises buf_queue with stream on and off under cam->vlock.
 */
void mtk_camsv71_queue_buffer(struct mtk_camsv *cam, struct mtk_camsv_buffer *buf)
{
	unsigned long flags;

	spin_lock_irqsave(&cam->lock, flags);
	list_add_tail(&buf->list, &cam->queued);
	if (cam->state == MTK_CAMSV_RUNNING && !cam->pending && !cam->quarantined) {
		if (READ_ONCE(early_credit) && !mtk_camsv_near_sof(cam))
			mtk_camsv_program_next(cam, true);
		else
			cam->stats.deferred++;
	}
	spin_unlock_irqrestore(&cam->lock, flags);
}

/*
 * Reset, configuration, first buffer and arming. The caller checked that the
 * CAMSV is idle; on failure it must tear down as after a stream.
 */
int mtk_camsv71_hw_start(struct mtk_camsv *cam)
{
	struct mtk_camsv_buffer *first;
	u32 ctl2;
	int ret;

	ret = mtk_camsv_reset(cam);
	if (ret)
		return ret;
	mtk_camsv_hw_config(cam);

	spin_lock_irq(&cam->lock);
	/* min_queued_buffers guarantees a first buffer. */
	first = list_first_entry_or_null(&cam->queued, struct mtk_camsv_buffer,
					 list);
	if (!first) {
		spin_unlock_irq(&cam->lock);
		return -EINVAL;
	}
	list_del(&first->list);
	cam->active = NULL;
	cam->frame = 0;
	cam->in_frame = false;
	cam->frame_err = 0;
	cam->bad_run = 0;
	cam->last_sof_ts = 0;
	cam->period_ns = 0;
	cam->frame_size = FIELD_PREP(CAMSV_TG_FRMSIZE_W, cam->pix.width) |
			  FIELD_PREP(CAMSV_TG_FRMSIZE_H, cam->pix.height);
	ctl2 = readl(cam->base + REG_CAMSV_FBC_IMGO_CTL2);
	cam->wcnt = FIELD_GET(CAMSV_FBC_WCNT, ctl2);
	/* Before the first credit: the first SOF finds RCNT one higher. */
	cam->rcnt = FIELD_GET(CAMSV_FBC_RCNT, ctl2);
	cam->stats.lead_us = U32_MAX;
	/* Programmed before arming, the first credit applies at once. */
	mtk_camsv_program(cam, first, 1, false);
	cam->state = MTK_CAMSV_RUNNING;
	spin_unlock_irq(&cam->lock);

	/* Arm (MediaTek mtk_cam_sv_top_enable()). */
	writel(CAMSV_CLK_EN_TG_DP | CAMSV_CLK_EN_QBN_DP | CAMSV_CLK_EN_PAK_DP |
	       CAMSV_CLK_EN_IMGO_DP, cam->base + REG_CAMSV_CLK_EN);
	mtk_camsv_db_load(cam);
	mtk_camsv_tg_db_load(cam);
	mtk_camsv_rmw(cam, REG_CAMSV_FBC_IMGO_CTL1, CAMSV_FBC_DB_EN,
		      CAMSV_FBC_DB_EN);
	readl(cam->base + REG_CAMSV_INT_STATUS);	/* drop stale status */
	enable_irq(cam->irq);
	cam->irq_enabled = true;
	writel(cam->soc->int_en, cam->base + REG_CAMSV_INT_EN);
	mtk_camsv_rmw(cam, REG_CAMSV_TG_VF_CON, CAMSV_TG_VF_CON_VFDATA_EN,
		      CAMSV_TG_VF_CON_VFDATA_EN);
	mtk_camsv_tg_db_load(cam);	/* latch VF before the first frame */
	return 0;
}

static unsigned long mtk_camsv_stop_timeout(struct mtk_camsv *cam)
{
	u64 period_ns = READ_ONCE(cam->period_ns);
	u64 ms = MTK_CAMSV_STOP_MAX_MS;

	/* Up to two frame ends: the pending credit drains first. */
	if (period_ns)
		ms = clamp_t(u64, 4 * div_u64(period_ns, NSEC_PER_MSEC) + 50,
			     MTK_CAMSV_STOP_MIN_MS, MTK_CAMSV_STOP_MAX_MS);
	return msecs_to_jiffies(ms);
}

/*
 * Stop capturing. With @at_frame_end no further credit is given and the
 * handler clears VF_DATA_EN at the first frame end without a credit
 * outstanding, or the timeout forces it. Then INT_EN 0, IRQ off, VF off and
 * TG double-buffer load from here, and wait for the TG to go idle. Returns 0
 * once it is idle.
 */
int mtk_camsv71_hw_stop_capture(struct mtk_camsv *cam, bool at_frame_end)
{
	u32 tg;
	int ret;

	spin_lock_irq(&cam->lock);
	if (cam->state == MTK_CAMSV_RUNNING)
		cam->state = at_frame_end ? MTK_CAMSV_STOPPING : MTK_CAMSV_STOPPED;
	spin_unlock_irq(&cam->lock);

	if (at_frame_end)
		wait_event_timeout(cam->stop_wq,
				   READ_ONCE(cam->state) != MTK_CAMSV_STOPPING,
				   mtk_camsv_stop_timeout(cam));

	writel(0, cam->base + REG_CAMSV_INT_EN);
	if (cam->irq_enabled) {
		disable_irq(cam->irq);
		cam->irq_enabled = false;
	}
	/* The handler has finished and cannot run again. */
	if (cam->state == MTK_CAMSV_STOPPING) {
		dev_warn(cam->dev, "no frame end within the stop timeout\n");
		cam->state = MTK_CAMSV_STOPPED;
		cam->stats.forced = true;
	}
	cam->stats.failed = cam->state == MTK_CAMSV_FAILED;

	mtk_camsv_vf_off(cam);
	ret = readl_poll_timeout(cam->base + REG_CAMSV_TG_INTER_ST, tg,
				 FIELD_GET(CAMSV_TG_INTER_ST_CAM_CS, tg) ==
				 CAMSV_TG_CAM_CS_IDLE,
				 MTK_CAMSV_TG_POLL_US, MTK_CAMSV_TG_IDLE_TIMEOUT_US);
	cam->stats.tg_stop = tg;
	if (ret)
		dev_err(cam->dev, "TG not idle after VF off (TG_INTER_ST %#x)\n", tg);
	return ret;
}

/*
 * After the SENINF and the sensor stopped: reset, module off and the idle
 * check of both windows. @tg_ret is the result of the TG idle wait. Returns 0
 * when the CAMSV is provably stopped; otherwise the module stays configured
 * and the caller quarantines the device.
 */
int mtk_camsv71_hw_disable(struct mtk_camsv *cam, int tg_ret)
{
	int ret;

	/* The IRQ is off: keep the late status before the reset clears it. */
	cam->stats.late_status = readl(cam->base + REG_CAMSV_INT_STATUS);
	cam->stats.imgo_err = readl(cam->base + REG_CAMSV_IMGO_ERR_STAT);
	cam->stats.fbc_stop = readl(cam->base + REG_CAMSV_FBC_IMGO_CTL2);
	ret = mtk_camsv_reset(cam);
	cam->stats.fbc_reset = readl(cam->base + REG_CAMSV_FBC_IMGO_CTL2);
	if (tg_ret || ret)
		return tg_ret ?: ret;

	mtk_camsv_rmw(cam, REG_CAMSV_TG_SEN_MODE, CAMSV_TG_SEN_MODE_CMOS_EN, 0);
	mtk_camsv_tg_db_load(cam);
	writel(0, cam->base + REG_CAMSV_MODULE_EN);
	writel(0, cam->base + REG_CAMSV_FBC_IMGO_CTL1);
	writel(CAMSV_MODULE_EN_DB_EN, cam->base + REG_CAMSV_MODULE_EN);
	writel(0, cam->base + REG_CAMSV_CLK_EN);
	if (!mtk_camsv71_hw_idle(cam))
		return -EBUSY;
	cam->state = MTK_CAMSV_OFF;
	return 0;
}
