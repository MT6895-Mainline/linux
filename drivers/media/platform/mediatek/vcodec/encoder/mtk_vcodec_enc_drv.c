// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *	Tiffany Lin <tiffany.lin@mediatek.com>
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_vcodec_enc.h"
#include "mtk_vcodec_enc_pm.h"
#include "venc_drv_if.h"
#include "../common/mtk_vcodec_intr.h"

/* The MT6895 frontend talks to the VCP transport, protocol and hardware
 * adapter directly. Everything below is therefore compiled out when the VCP
 * backend is not part of the kernel image.
 */
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
#define MT6895_VCP_VENC_DMA_ID 0x60421

static int mtk_vcodec_vcp_dma_probe(struct platform_device *pdev)
{
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(&pdev->dev);
	struct iommu_domain *domain = iommu_get_domain_for_dev(&pdev->dev);
	int ret;

	if (!fwspec || fwspec->num_ids != 1 ||
	    fwspec->ids[0] != MT6895_VCP_VENC_DMA_ID)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid VCP VENC IOMMU stream ID\n");
	if (!domain || !domain->geometry.force_aperture ||
	    domain->geometry.aperture_start != VCP_VENC_BITSTREAM_BASE ||
	    domain->geometry.aperture_end != VCP_VENC_BITSTREAM_END - 1)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "invalid VCP VENC IOMMU aperture\n");
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(34));
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set VCP VENC DMA mask\n");
	dma_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));
	dev_info(&pdev->dev, "VCP VENC DMA domain ready: %#llx-%#llx\n",
		 (u64)domain->geometry.aperture_start,
		 (u64)domain->geometry.aperture_end);
	return 0;
}

static const struct of_device_id mtk_vcodec_vcp_dma_match[] = {
	{ .compatible = "mediatek,mt6895-vcp-venc-dma" },
	{},
};
MODULE_DEVICE_TABLE(of, mtk_vcodec_vcp_dma_match);

static struct platform_driver mtk_vcodec_vcp_dma_driver = {
	.probe = mtk_vcodec_vcp_dma_probe,
	.driver = {
		.name = "mt6895-vcp-venc-dma",
		.of_match_table = mtk_vcodec_vcp_dma_match,
		.suppress_bind_attrs = true,
	},
};

static void mtk_vcodec_put_device(void *data)
{
	put_device(data);
}

static struct device *mtk_vcodec_get_vcp_dma_dev(struct device *dev)
{
	struct platform_device *pdev;
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, "mediatek,vcp-venc-dma", 0);
	if (!node)
		return ERR_PTR(-EINVAL);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	if (!device_is_bound(&pdev->dev) ||
	    pdev->dev.driver != &mtk_vcodec_vcp_dma_driver.driver) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return &pdev->dev;
}
#endif /* CONFIG_VIDEO_MEDIATEK_VCODEC_VCP */

static const struct mtk_video_fmt mtk_video_formats_output[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.type = MTK_FMT_FRAME,
		.num_planes = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.type = MTK_FMT_FRAME,
		.num_planes = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU420M,
		.type = MTK_FMT_FRAME,
		.num_planes = 3,
	},
};

static const struct mtk_video_fmt mtk_video_formats_capture_h264[] =  {
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.type = MTK_FMT_ENC,
		.num_planes = 1,
	},
};

static const struct mtk_video_fmt mtk_video_formats_capture_vp8[] =  {
	{
		.fourcc = V4L2_PIX_FMT_VP8,
		.type = MTK_FMT_ENC,
		.num_planes = 1,
	},
};

#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
static void mtk_vcodec_vcp_ready(void *priv, u64 instance)
{
	struct mtk_vcodec_enc_dev *dev = priv;

	(void)instance;
	WRITE_ONCE(dev->vcp_notify_seq, dev->vcp_notify_seq + 1);
	wake_up(&dev->vcp_wait);
	if (dev->encode_workqueue)
		queue_work(dev->encode_workqueue, &dev->vcp_done_work);
}

static void mtk_vcodec_vcp_done_worker(struct work_struct *work)
{
	struct mtk_vcodec_enc_dev *dev =
		container_of(work, struct mtk_vcodec_enc_dev, vcp_done_work);

	mutex_lock(&dev->enc_mutex);
	/* Results stay queued on their protocol instance. Always drain the
	 * current session so a coalesced stale notification cannot hide a newer
	 * session's completion behind a single saved instance cookie.
	 */
	venc_vcp_h264_buffers_ready(dev);
	mutex_unlock(&dev->enc_mutex);
}
#endif /* CONFIG_VIDEO_MEDIATEK_VCODEC_VCP */

static void clean_irq_status(unsigned int irq_status, void __iomem *addr)
{
	if (irq_status & MTK_VENC_IRQ_STATUS_PAUSE)
		writel(MTK_VENC_IRQ_STATUS_PAUSE, addr);

	if (irq_status & MTK_VENC_IRQ_STATUS_SWITCH)
		writel(MTK_VENC_IRQ_STATUS_SWITCH, addr);

	if (irq_status & MTK_VENC_IRQ_STATUS_DRAM)
		writel(MTK_VENC_IRQ_STATUS_DRAM, addr);

	if (irq_status & MTK_VENC_IRQ_STATUS_SPS)
		writel(MTK_VENC_IRQ_STATUS_SPS, addr);

	if (irq_status & MTK_VENC_IRQ_STATUS_PPS)
		writel(MTK_VENC_IRQ_STATUS_PPS, addr);

	if (irq_status & MTK_VENC_IRQ_STATUS_FRM)
		writel(MTK_VENC_IRQ_STATUS_FRM, addr);

}
static irqreturn_t mtk_vcodec_enc_irq_handler(int irq, void *priv)
{
	struct mtk_vcodec_enc_dev *dev = priv;
	struct mtk_vcodec_enc_ctx *ctx;
	unsigned long flags;
	void __iomem *addr;
	int core_id;

	spin_lock_irqsave(&dev->irqlock, flags);
	ctx = dev->curr_ctx;
	spin_unlock_irqrestore(&dev->irqlock, flags);

	core_id = dev->venc_pdata->core_id;
	if (core_id < 0 || core_id >= NUM_MAX_VCODEC_REG_BASE) {
		mtk_v4l2_venc_err(ctx, "Invalid core id: %d, ctx id: %d", core_id, ctx->id);
		return IRQ_HANDLED;
	}

	mtk_v4l2_venc_dbg(1, ctx, "id: %d, core id: %d", ctx->id, core_id);

	addr = dev->reg_base[core_id] + MTK_VENC_IRQ_ACK_OFFSET;

	ctx->irq_status = readl(dev->reg_base[core_id] +
				(MTK_VENC_IRQ_STATUS_OFFSET));

	clean_irq_status(ctx->irq_status, addr);

	wake_up_enc_ctx(ctx, MTK_INST_IRQ_RECEIVED, 0);
	return IRQ_HANDLED;
}

static int fops_vcodec_open(struct file *file)
{
	struct mtk_vcodec_enc_dev *dev = video_drvdata(file);
	struct mtk_vcodec_enc_ctx *ctx = NULL;
	int ret = 0;
	struct vb2_queue *src_vq;
	unsigned long flags;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;

	mutex_lock(&dev->dev_mutex);
	/*
	 * Use simple counter to uniquely identify this context. Only
	 * used for logging.
	 */
	ctx->id = dev->id_counter++;
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	v4l2_fh_add(&ctx->fh, file);
	INIT_LIST_HEAD(&ctx->list);
	ctx->dev = dev;
	init_waitqueue_head(&ctx->queue[0]);
	mutex_init(&ctx->q_mutex);

	ctx->type = MTK_INST_ENCODER;
	ret = mtk_vcodec_enc_ctrls_setup(ctx);
	if (ret) {
		mtk_v4l2_venc_err(ctx, "Failed to setup controls() (%d)", ret);
		goto err_ctrls_setup;
	}
	ctx->m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev_enc, ctx,
					 &mtk_vcodec_enc_queue_init);
	if (IS_ERR((__force void *)ctx->m2m_ctx)) {
		ret = PTR_ERR((__force void *)ctx->m2m_ctx);
		mtk_v4l2_venc_err(ctx, "Failed to v4l2_m2m_ctx_init() (%d)", ret);
		goto err_m2m_ctx_init;
	}
	src_vq = v4l2_m2m_get_vq(ctx->m2m_ctx,
				 V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	ctx->empty_flush_buf.vb.vb2_buf.vb2_queue = src_vq;
	mtk_vcodec_enc_set_default_params(ctx);

	/* VCP boots in venc_vcp_h264_if.init, never during udev probing. */
	if (!dev->venc_pdata->uses_vcp && v4l2_fh_is_singular(&ctx->fh)) {
		/*
		 * load fireware to checks if it was loaded already and
		 * does nothing in that case
		 */
		ret = mtk_vcodec_fw_load_firmware(dev->fw_handler);
		if (ret < 0) {
			/*
			 * Return 0 if downloading firmware successfully,
			 * otherwise it is failed
			 */
			mtk_v4l2_venc_err(ctx, "vpu_load_firmware failed!");
			goto err_load_fw;
		}

		dev->enc_capability =
			mtk_vcodec_fw_get_venc_capa(dev->fw_handler);
		mtk_v4l2_venc_dbg(0, ctx, "encoder capability %x", dev->enc_capability);
	}

	mtk_v4l2_venc_dbg(2, ctx, "Create instance [%d]@%p m2m_ctx=%p ",
			  ctx->id, ctx, ctx->m2m_ctx);

	spin_lock_irqsave(&dev->dev_ctx_lock, flags);
	list_add(&ctx->list, &dev->ctx_list);
	spin_unlock_irqrestore(&dev->dev_ctx_lock, flags);

	mutex_unlock(&dev->dev_mutex);
	mtk_v4l2_venc_dbg(0, ctx, "%s encoder [%d]", dev_name(&dev->plat_dev->dev),
			  ctx->id);
	return ret;

	/* Deinit when failure occurred */
err_load_fw:
	v4l2_m2m_ctx_release(ctx->m2m_ctx);
err_m2m_ctx_init:
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);
err_ctrls_setup:
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);

	return ret;
}

static int fops_vcodec_release(struct file *file)
{
	struct mtk_vcodec_enc_dev *dev = video_drvdata(file);
	struct mtk_vcodec_enc_ctx *ctx = file_to_enc_ctx(file);
	unsigned long flags;

	mtk_v4l2_venc_dbg(1, ctx, "[%d] encoder", ctx->id);
	mutex_lock(&dev->dev_mutex);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	mtk_vcodec_enc_release(ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->ctrl_hdl);

	/*
	 * Cancel any pending encode work before freeing the context.
	 * Although v4l2_m2m_ctx_release() waits for m2m job completion,
	 * the workqueue handler (mtk_venc_worker) may still be accessing
	 * the context after v4l2_m2m_job_finish() returns. Without this,
	 * a use-after-free occurs when the worker accesses ctx after kfree.
	 */
	cancel_work_sync(&ctx->encode_work);

	spin_lock_irqsave(&dev->dev_ctx_lock, flags);
	list_del_init(&ctx->list);
	spin_unlock_irqrestore(&dev->dev_ctx_lock, flags);
	kfree(ctx);
	mutex_unlock(&dev->dev_mutex);
	return 0;
}

static const struct v4l2_file_operations mtk_vcodec_fops = {
	.owner		= THIS_MODULE,
	.open		= fops_vcodec_open,
	.release	= fops_vcodec_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static int mtk_vcodec_probe(struct platform_device *pdev)
{
	struct mtk_vcodec_enc_dev *dev;
	struct video_device *vfd_enc;
	phandle rproc_phandle;
	enum mtk_vcodec_fw_type fw_type;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->ctx_list);
	dev->plat_dev = pdev;
	dev->venc_pdata = of_device_get_match_data(&pdev->dev);
	if (!dev->venc_pdata)
		return -ENODEV;
	init_waitqueue_head(&dev->vcp_wait);

	if (dev->venc_pdata->uses_vcp) {
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
		dev->vcp_bitstream_dev = mtk_vcodec_get_vcp_dma_dev(&pdev->dev);
		if (IS_ERR(dev->vcp_bitstream_dev))
			return dev_err_probe(&pdev->dev,
					     PTR_ERR(dev->vcp_bitstream_dev),
					     "failed to get VCP VENC DMA device\n");
		ret = devm_add_action_or_reset(&pdev->dev, mtk_vcodec_put_device,
					       dev->vcp_bitstream_dev);
		if (ret)
			return ret;
		dev->vcp = mtk_vcp_get(&pdev->dev);
		if (IS_ERR(dev->vcp))
			return PTR_ERR(dev->vcp);
		dev->vcp_hw = mtk_vcp_venc_hw_create(pdev, dev->vcp,
						     mtk_vcodec_vcp_ready, dev);
		if (IS_ERR(dev->vcp_hw)) {
			ret = PTR_ERR(dev->vcp_hw);
			mtk_vcp_put(dev->vcp);
			return ret;
		}
		dev->vcp_venc = mtk_vcp_venc_create(&pdev->dev,
						    dev->vcp_bitstream_dev,
						    dev->vcp,
						    mtk_vcp_venc_hw_ops(),
						    dev->vcp_hw);
		if (IS_ERR(dev->vcp_venc)) {
			ret = PTR_ERR(dev->vcp_venc);
			mtk_vcp_put(dev->vcp);
			return ret;
		}
		dev->fw_handler = NULL;
		/* Do not advertise unverified firmware capabilities (including 4K). */
		dev->enc_capability = 0;
		goto codec_resources;
#else
		/* The MT6895 encoder only exists in its firmware-backed form. */
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "VCP encoder support is not configured\n");
#endif
	}

	if (!of_property_read_u32(pdev->dev.of_node, "mediatek,vpu",
				  &rproc_phandle)) {
		fw_type = VPU;
	} else if (!of_property_read_u32(pdev->dev.of_node, "mediatek,scp",
					 &rproc_phandle)) {
		fw_type = SCP;
	} else {
		dev_err(&pdev->dev, "[MTK VCODEC] Could not get venc IPI device");
		return -ENODEV;
	}
	dma_set_max_seg_size(&pdev->dev, UINT_MAX);

	dev->fw_handler = mtk_vcodec_fw_select(dev, fw_type, ENCODER);
	if (IS_ERR(dev->fw_handler))
		return PTR_ERR(dev->fw_handler);

#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
codec_resources:
#endif
	if (!dev->venc_pdata->uses_vcp) {
		ret = mtk_vcodec_init_enc_clk(dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "[MTK VCODEC] Failed to get mtk vcodec clock source!");
			goto err_enc_pm;
		}

		pm_runtime_enable(&pdev->dev);

		dev->reg_base[dev->venc_pdata->core_id] =
			devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(dev->reg_base[dev->venc_pdata->core_id])) {
			ret = PTR_ERR(dev->reg_base[dev->venc_pdata->core_id]);
			goto err_res;
		}

		dev->enc_irq = platform_get_irq(pdev, 0);
		if (dev->enc_irq < 0) {
			ret = dev->enc_irq;
			goto err_res;
		}

		irq_set_status_flags(dev->enc_irq, IRQ_NOAUTOEN);
		ret = devm_request_irq(&pdev->dev, dev->enc_irq,
				       mtk_vcodec_enc_irq_handler,
				       0, pdev->name, dev);
		if (ret) {
			dev_err(&pdev->dev,
				"[MTK VCODEC] Failed to install dev->enc_irq %d (%d) core_id (%d)",
				dev->enc_irq, ret, dev->venc_pdata->core_id);
			ret = -EINVAL;
			goto err_res;
		}
	}

	mutex_init(&dev->enc_mutex);
	mutex_init(&dev->dev_mutex);
	spin_lock_init(&dev->dev_ctx_lock);
	spin_lock_init(&dev->irqlock);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name), "%s",
		 "[MTK_V4L2_VENC]");

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "[MTK VCODEC] v4l2_device_register err=%d", ret);
		goto err_res;
	}

	/* allocate video device for encoder and register it */
	vfd_enc = video_device_alloc();
	if (!vfd_enc) {
		dev_err(&pdev->dev, "[MTK VCODEC] Failed to allocate video device");
		ret = -ENOMEM;
		goto err_enc_alloc;
	}
	vfd_enc->fops           = &mtk_vcodec_fops;
	vfd_enc->ioctl_ops      = &mtk_venc_ioctl_ops;
	vfd_enc->release        = video_device_release;
	vfd_enc->lock           = &dev->dev_mutex;
	vfd_enc->v4l2_dev       = &dev->v4l2_dev;
	vfd_enc->vfl_dir        = VFL_DIR_M2M;
	vfd_enc->device_caps	= V4L2_CAP_VIDEO_M2M_MPLANE |
					V4L2_CAP_STREAMING;

	snprintf(vfd_enc->name, sizeof(vfd_enc->name), "%s",
		 MTK_VCODEC_ENC_NAME);
	video_set_drvdata(vfd_enc, dev);
	dev->vfd_enc = vfd_enc;
	platform_set_drvdata(pdev, dev);

	dev->m2m_dev_enc = v4l2_m2m_init(&mtk_venc_m2m_ops);
	if (IS_ERR((__force void *)dev->m2m_dev_enc)) {
		dev_err(&pdev->dev, "[MTK VCODEC] Failed to init mem2mem enc device");
		ret = PTR_ERR((__force void *)dev->m2m_dev_enc);
		goto err_enc_mem_init;
	}

	dev->encode_workqueue =
			alloc_ordered_workqueue(MTK_VCODEC_ENC_NAME,
						WQ_MEM_RECLAIM |
						WQ_FREEZABLE);
	if (!dev->encode_workqueue) {
		dev_err(&pdev->dev, "[MTK VCODEC] Failed to create encode workqueue");
		ret = -EINVAL;
		goto err_event_workq;
	}
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	INIT_WORK(&dev->vcp_done_work, mtk_vcodec_vcp_done_worker);
#endif

	ret = video_register_device(vfd_enc, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&pdev->dev, "[MTK VCODEC] Failed to register video device");
		goto err_enc_reg;
	}

	mtk_vcodec_dbgfs_init(dev, true);
	dev_dbg(&pdev->dev,  "[MTK VCODEC] encoder %d registered as /dev/video%d",
		dev->venc_pdata->core_id, vfd_enc->num);

	return 0;

err_enc_reg:
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	if (dev->vcp_venc)
		mtk_vcp_ipi_unregister(dev->vcp, MTK_VCP_ENCODER);
#endif
	destroy_workqueue(dev->encode_workqueue);
err_event_workq:
	v4l2_m2m_release(dev->m2m_dev_enc);
err_enc_mem_init:
	video_unregister_device(vfd_enc);
err_enc_alloc:
	v4l2_device_unregister(&dev->v4l2_dev);
err_res:
	if (!dev->venc_pdata->uses_vcp)
		pm_runtime_disable(dev->pm.dev);
err_enc_pm:
	if (dev->fw_handler)
		mtk_vcodec_fw_release(dev->fw_handler);
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	if (dev->vcp_venc)
		mtk_vcp_venc_destroy(dev->vcp_venc);
	if (dev->vcp)
		mtk_vcp_put(dev->vcp);
#endif
	return ret;
}

static const struct mtk_vcodec_enc_pdata mt8173_avc_pdata = {
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 60000000,
	.core_id = VENC_SYS,
};

static const struct mtk_vcodec_enc_pdata mt8173_vp8_pdata = {
	.capture_formats = mtk_video_formats_capture_vp8,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_vp8),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 9000000,
	.core_id = VENC_LT_SYS,
};

static const struct mtk_vcodec_enc_pdata mt8183_pdata = {
	.uses_ext = true,
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 40000000,
	.core_id = VENC_SYS,
};

static const struct mtk_vcodec_enc_pdata mt8188_pdata = {
	.uses_ext = true,
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 50000000,
	.core_id = VENC_SYS,
	.uses_34bit = true,
};

static const struct mtk_vcodec_enc_pdata mt8192_pdata = {
	.uses_ext = true,
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 100000000,
	.core_id = VENC_SYS,
};

static const struct mtk_vcodec_enc_pdata mt8195_pdata = {
	.uses_ext = true,
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 100000000,
	.core_id = VENC_SYS,
};

#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
static const struct mtk_vcodec_enc_pdata mt6895_pdata = {
	.uses_vcp = true,
	.uses_34bit = true,
	.capture_formats = mtk_video_formats_capture_h264,
	.num_capture_formats = ARRAY_SIZE(mtk_video_formats_capture_h264),
	.output_formats = mtk_video_formats_output,
	.num_output_formats = ARRAY_SIZE(mtk_video_formats_output),
	.min_bitrate = 64,
	.max_bitrate = 100000000,
	.core_id = VENC_SYS,
};
#endif

static const struct of_device_id mtk_vcodec_enc_match[] = {
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	{.compatible = "mediatek,mt6895-vcodec-enc", .data = &mt6895_pdata},
#endif
	{.compatible = "mediatek,mt8173-vcodec-enc",
			.data = &mt8173_avc_pdata},
	{.compatible = "mediatek,mt8173-vcodec-enc-vp8",
			.data = &mt8173_vp8_pdata},
	{.compatible = "mediatek,mt8183-vcodec-enc", .data = &mt8183_pdata},
	{.compatible = "mediatek,mt8188-vcodec-enc", .data = &mt8188_pdata},
	{.compatible = "mediatek,mt8192-vcodec-enc", .data = &mt8192_pdata},
	{.compatible = "mediatek,mt8195-vcodec-enc", .data = &mt8195_pdata},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_vcodec_enc_match);

static void mtk_vcodec_enc_remove(struct platform_device *pdev)
{
	struct mtk_vcodec_enc_dev *dev = platform_get_drvdata(pdev);

	/* The IPI callback can queue vcp_done_work. Unregister it first so no
	 * callback can race the cancellation and destruction below.
	 */
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	if (dev->vcp_venc)
		mtk_vcp_ipi_unregister(dev->vcp, MTK_VCP_ENCODER);
	cancel_work_sync(&dev->vcp_done_work);
#endif

	if (dev->vfd_enc) {
		video_unregister_device(dev->vfd_enc);
		dev->vfd_enc = NULL;
	}
	destroy_workqueue(dev->encode_workqueue);
	if (dev->m2m_dev_enc)
		v4l2_m2m_release(dev->m2m_dev_enc);

	mtk_vcodec_dbgfs_deinit(&dev->dbgfs);
	v4l2_device_unregister(&dev->v4l2_dev);
	if (!dev->venc_pdata->uses_vcp)
		pm_runtime_disable(dev->pm.dev);
	if (dev->fw_handler)
		mtk_vcodec_fw_release(dev->fw_handler);
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	if (dev->vcp_venc)
		mtk_vcp_venc_destroy(dev->vcp_venc);
	if (dev->vcp)
		mtk_vcp_put(dev->vcp);
#endif
}

static struct platform_driver mtk_vcodec_enc_driver = {
	.probe	= mtk_vcodec_probe,
	.remove = mtk_vcodec_enc_remove,
	.driver	= {
		.name	= MTK_VCODEC_ENC_NAME,
		.of_match_table = mtk_vcodec_enc_match,
		/* Faulted VCP sessions can retain DMA until a cold restart. */
		.suppress_bind_attrs = true,
	},
};

static int __init mtk_vcodec_enc_init(void)
{
	int ret;

#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	ret = platform_driver_register(&mtk_vcodec_vcp_dma_driver);
	if (ret)
		return ret;
#endif
	ret = platform_driver_register(&mtk_vcodec_enc_driver);
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	if (ret)
		platform_driver_unregister(&mtk_vcodec_vcp_dma_driver);
#endif
	return ret;
}
module_init(mtk_vcodec_enc_init);

static void __exit mtk_vcodec_enc_exit(void)
{
	platform_driver_unregister(&mtk_vcodec_enc_driver);
#if IS_ENABLED(CONFIG_VIDEO_MEDIATEK_VCODEC_VCP)
	platform_driver_unregister(&mtk_vcodec_vcp_dma_driver);
#endif
}
module_exit(mtk_vcodec_enc_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Mediatek video codec V4L2 encoder driver");
