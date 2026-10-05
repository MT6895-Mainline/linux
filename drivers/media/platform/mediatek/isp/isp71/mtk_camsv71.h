/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek ISP 7.1 CAMSV raw capture driver
 *
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 * Copyright (c) 2025 MediaTek Inc.
 */

#ifndef __MTK_CAMSV71_H__
#define __MTK_CAMSV71_H__

#include <linux/align.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/math.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#define MTK_CAMSV71_NAME		"mtk-camsv71"

#define MTK_CAMSV_PAD_SINK		0
#define MTK_CAMSV_PAD_SRC		1
#define MTK_CAMSV_NUM_PADS		2

#define MTK_CAMSV_NUM_CLKS		12
/* IOVA width of the IOMMU domain; IMGO_BASE_ADDR_MSB holds bits 35:32. */
#define MTK_CAMSV_DMA_BITS		34

#define MTK_CAMSV_MIN_WIDTH		80
#define MTK_CAMSV_MIN_HEIGHT		60
#define MTK_CAMSV_MAX_WIDTH		8192
#define MTK_CAMSV_MAX_HEIGHT		6144
/* Four RAW10 pixels per five bytes; two pixels per clock in pixel mode 1. */
#define MTK_CAMSV_WIDTH_ALIGN		4
#define MTK_CAMSV_HEIGHT_ALIGN		2
#define MTK_CAMSV_MAX_STRIDE		0xfffc

/* Default format: the verified GC02M1 mode, 2000 bytes per line. */
#define MTK_CAMSV_DEF_WIDTH		1600
#define MTK_CAMSV_DEF_HEIGHT		1200

struct mtk_camsv_format {
	u32 code;
	u32 fourcc;
	u8 bpp;
	u8 tg_fmt;		/* FMT_SEL.TG1_FMT */
	u8 pak_mode;		/* PAK.PAK_MODE */
};

struct mtk_camsv_soc {
	const phys_addr_t *bases;	/* instance id = index */
	unsigned int num_bases;
	const struct mtk_camsv_format *formats;
	unsigned int num_formats;
	const u32 *imgo_con;		/* IMGO_CON0..4 */
	u32 int_en;
	u32 special_fun_en;
	u8 pixel_mode;			/* SoC constant shared with the SENINF */
};

enum mtk_camsv_state {
	MTK_CAMSV_OFF,		/* not armed */
	MTK_CAMSV_RUNNING,	/* VF on, at most one credit ahead (SOF or queue) */
	MTK_CAMSV_STOPPING,	/* no new credit, VF off at the next free frame end */
	MTK_CAMSV_STOPPED,	/* VF off */
	MTK_CAMSV_FAILED,	/* VF off after a fatal error, buffers kept until reset */
};

struct mtk_camsv_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_addr_t addr;
	u32 tag;		/* FRAME_SEQ_NO written with it, unique per stream */
	u32 frame;		/* frame that latched it, counted from 1 */
	u64 ts;			/* SOF time of that frame */
	u64 credit_ts;		/* time its credit was written */
	bool early;		/* credit written when the buffer was queued */
	bool missed;		/* a SOF passed without applying its credit */
};

/* Per-stream counters, reported when the stream stops. */
struct mtk_camsv_stats {
	u32 done;
	u32 drops;
	u32 drop_nobuf;		/* SOF without a programmed buffer */
	u32 drop_after;		/* buffer programmed after the SOF, before its IRQ */
	u32 drop_straddle;	/* SOF between the tag and the credit of a buffer */
	u32 imgo_drop;		/* IMGO_DROP interrupts */
	u32 credits;		/* buffers programmed, the first one included */
	u32 early;		/* of which when they were queued */
	u32 early_hit;		/* early credits applied by the next SOF */
	u32 deferred;		/* queued with nothing pending, left to the SOF */
	u32 lead_us;		/* least SOF IRQ - credit time of an early hit */
	u32 bad;
	u32 irq_status;
	u32 late_status;
	u32 imgo_err;
	u32 fbc_stop;
	u32 fbc_reset;
	u32 tg_stop;
	u32 larb_start;
	bool forced;
	bool failed;
};

struct mtk_camsv {
	struct device *dev;
	const struct mtk_camsv_soc *soc;
	unsigned int id;
	void __iomem *base;
	void __iomem *inner;
	int irq;
	struct clk_bulk_data clks[MTK_CAMSV_NUM_CLKS];

	/* LARB of the IMGO port and its port control register (read only). */
	struct device *larb;
	void __iomem *larb_port_con;

	struct v4l2_subdev subdev;
	struct media_pad pads[MTK_CAMSV_NUM_PADS];
	struct media_pad *seninf_pad;	/* remote pad of the enabled stream */

	struct video_device vdev;
	struct media_pad vdev_pad;
	struct vb2_queue queue;
	struct mutex vlock;		/* serialises the video node and vb2 */
	struct module *v4l2_owner;	/* module of the media and V4L2 devices */
	struct v4l2_pix_format pix;
	const struct mtk_camsv_format *fmt;
	bool irq_enabled;
	bool quarantined;

	spinlock_t lock;		/* protects the members below, taken in hard IRQ */
	struct list_head queued;	/* buffers not yet given to the hardware */
	struct mtk_camsv_buffer *pending;	/* programmed, not yet latched */
	struct mtk_camsv_buffer *active;	/* latched, being written */
	enum mtk_camsv_state state;
	bool in_frame;
	u32 frame;			/* SOFs since arming */
	u8 rcnt;			/* FBC read count at the last SOF */
	u8 wcnt;			/* last FBC write count */
	u32 frame_err;
	u32 bad_run;
	u32 frame_size;			/* expected TG_FRMSIZE_ST_R */
	u64 last_sof_ts;
	u64 period_ns;
	struct mtk_camsv_stats stats;
	wait_queue_head_t stop_wq;
};

static inline struct mtk_camsv *sd_to_mtk_camsv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_camsv, subdev);
}

/* Bytes the IMGO writes per line (MediaTek mtk_cam_sv_xsize_cal()). */
static inline u32 mtk_camsv_xsize(const struct mtk_camsv *cam,
				  const struct mtk_camsv_format *fmt, u32 width)
{
	return ALIGN(DIV_ROUND_UP(width * fmt->bpp, 8), 2U << cam->soc->pixel_mode);
}

const struct mtk_camsv_format *mtk_camsv71_find_code(const struct mtk_camsv *cam,
						     u32 code);
int mtk_camsv71_larb_get(struct mtk_camsv *cam);
void mtk_camsv71_larb_put(struct mtk_camsv *cam);
u32 mtk_camsv71_larb_port(struct mtk_camsv *cam);

bool mtk_camsv71_hw_idle(struct mtk_camsv *cam);
int mtk_camsv71_hw_start(struct mtk_camsv *cam);
int mtk_camsv71_hw_stop_capture(struct mtk_camsv *cam, bool at_frame_end);
int mtk_camsv71_hw_disable(struct mtk_camsv *cam, int tg_ret);
void mtk_camsv71_queue_buffer(struct mtk_camsv *cam, struct mtk_camsv_buffer *buf);
irqreturn_t mtk_camsv71_isr(int irq, void *data);

int mtk_camsv71_video_register(struct mtk_camsv *cam);
void mtk_camsv71_video_unregister(struct mtk_camsv *cam);

#endif /* __MTK_CAMSV71_H__ */
