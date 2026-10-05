// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek ISP 7.1 CAMSV raw capture driver
 *
 * A CAMSV instance writes the pixel stream of one SENINF camera multiplexer
 * (CAM_MUX) output to memory through its IMGO DMA port, without ISP
 * processing. Each instance registers an async sub-device, bound by the
 * SENINF driver that owns the media device, and adds its video capture node
 * when that sub-device is registered.
 *
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/align.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>

#include <dt-bindings/memory/mtk-memory-port.h>

#include <media/v4l2-async.h>
#include <media/v4l2-event.h>

#include "mtk_camsv71.h"
#include "mtk_camsv71_regs.h"

#define MTK_CAMSV_EVENT_DEPTH		8

static const char * const mtk_camsv_clk_names[MTK_CAMSV_NUM_CLKS] = {
	"cam2mm0", "cam2mm1", "cam2sys", "cam", "camtg", "larb", "gcamsv",
	"camsv-top", "top-cam", "top-camtg", "top-camtm", "camsv-cq",
};

static const char * const mtk_camsv_pd_names[] = { "cam", "isp" };

/* CAMSV 0-9, the instances with a command queue clock. */
static const phys_addr_t mt6895_camsv_bases[] = {
	0x1a110000, 0x1a111000, 0x1a112000, 0x1a113000, 0x1a114000,
	0x1a115000, 0x1a116000, 0x1a117000, 0x1a180000, 0x1a181000,
};

static const struct mtk_camsv_format mt6895_camsv_formats[] = {
	{
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.fourcc = V4L2_PIX_FMT_SRGGB10P,
		.bpp = 10,
		.tg_fmt = CAMSV_TG_FMT_RAW10,
		.pak_mode = CAMSV_PAK_MODE_RAW10_MIPI,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.fourcc = V4L2_PIX_FMT_SBGGR10P,
		.bpp = 10,
		.tg_fmt = CAMSV_TG_FMT_RAW10,
		.pak_mode = CAMSV_PAK_MODE_RAW10_MIPI,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.fourcc = V4L2_PIX_FMT_SGBRG10P,
		.bpp = 10,
		.tg_fmt = CAMSV_TG_FMT_RAW10,
		.pak_mode = CAMSV_PAK_MODE_RAW10_MIPI,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.fourcc = V4L2_PIX_FMT_SGRBG10P,
		.bpp = 10,
		.tg_fmt = CAMSV_TG_FMT_RAW10,
		.pak_mode = CAMSV_PAK_MODE_RAW10_MIPI,
	},
};

/* IMGO FIFO and priority thresholds of CAMSV 0-9 (mtk_cam_sv_dmao_config()). */
static const u32 mt6895_camsv_imgo_con[CAMSV_IMGO_NUM_CON] = {
	0x10000300, 0x00c00060, 0x01800120, 0x020001a0, 0x012000c0,
};

#define MT6895_CAMSV_INT_EN	(CAMSV_INT_TG_ERR | CAMSV_INT_TG_GBERR | \
				 CAMSV_INT_TG_SOF | CAMSV_INT_TG_SOF_DROP | \
				 CAMSV_INT_DB_LOAD_ERR | CAMSV_INT_PASS1_DON | \
				 CAMSV_INT_SW_PASS1_DON | CAMSV_INT_DMA_ERR | \
				 CAMSV_INT_IMGO_OVERR | CAMSV_INT_IMGO_DROP | \
				 CAMSV_INT_IMGO_DONE | CAMSV_INT_SW_ENQUE_ERR)
static_assert(MT6895_CAMSV_INT_EN == 0x011b1d70);

static const struct mtk_camsv_soc mt6895_camsv_soc = {
	.bases = mt6895_camsv_bases,
	.num_bases = ARRAY_SIZE(mt6895_camsv_bases),
	.formats = mt6895_camsv_formats,
	.num_formats = ARRAY_SIZE(mt6895_camsv_formats),
	.imgo_con = mt6895_camsv_imgo_con,
	.int_en = MT6895_CAMSV_INT_EN,
	.special_fun_en = CAMSV_SPECIAL_FUN_DCM_MODE,
	.pixel_mode = 1,
};

const struct mtk_camsv_format *mtk_camsv71_find_code(const struct mtk_camsv *cam,
						     u32 code)
{
	unsigned int i;

	for (i = 0; i < cam->soc->num_formats; i++)
		if (cam->soc->formats[i].code == code)
			return &cam->soc->formats[i];
	return NULL;
}

/* ----------------------------------------------------------------------------
 * LARB of the IMGO port
 */

/* The LARB port control; the caller holds a runtime PM reference on the LARB. */
u32 mtk_camsv71_larb_port(struct mtk_camsv *cam)
{
	return readl(cam->larb_port_con);
}

/*
 * The LARB's runtime resume, through the IOMMU's device link, put the IMGO
 * port in translation mode. Require the LARB to be active, hold it until the
 * stream stops and check the port control before any DMA.
 */
int mtk_camsv71_larb_get(struct mtk_camsv *cam)
{
	u32 val;
	int ret;

	ret = pm_runtime_get_if_active(cam->larb);
	if (ret <= 0) {
		dev_err(cam->dev, "%s is not runtime-active (%d)\n",
			dev_name(cam->larb), ret);
		return -EIO;
	}

	val = mtk_camsv71_larb_port(cam);
	cam->stats.larb_start = val;
	dev_dbg(cam->dev, "%s port control %#x\n", dev_name(cam->larb), val);
	if (!(val & SMI_LARB_MMU_EN) || (val & SMI_LARB_BANK_SEL)) {
		dev_err(cam->dev, "%s port control %#x: not IOMMU bank 0\n",
			dev_name(cam->larb), val);
		pm_runtime_put(cam->larb);
		return -EIO;
	}
	return 0;
}

void mtk_camsv71_larb_put(struct mtk_camsv *cam)
{
	pm_runtime_put(cam->larb);
}

static void mtk_camsv_put_device(void *data)
{
	put_device(data);
}

/*
 * Find the LARB of the only IOMMU port in the IOMMU's mediatek,larbs list.
 * Its driver must be bound: the IOMMU's runtime PM link to it is stateless,
 * and runtime PM skips a supplier without a driver, which would leave the
 * port untranslated.
 */
static int mtk_camsv_find_larb(struct mtk_camsv *cam)
{
	struct device *dev = cam->dev;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct platform_device *larb_pdev;
	unsigned int larb_id, port, i;
	struct device_node *np;
	struct resource res;
	bool bound;
	u32 id;
	int ret;

	if (!fwspec || fwspec->num_ids != 1 || !is_of_node(fwspec->iommu_fwnode))
		return dev_err_probe(dev, -EINVAL, "exactly one IOMMU port is required\n");
	larb_id = MTK_M4U_TO_LARB(fwspec->ids[0]);
	port = MTK_M4U_TO_PORT(fwspec->ids[0]);

	for (i = 0; ; i++) {
		np = of_parse_phandle(to_of_node(fwspec->iommu_fwnode),
				      "mediatek,larbs", i);
		if (!np)
			return dev_err_probe(dev, -ENODEV,
					     "LARB%u is not managed by the IOMMU\n",
					     larb_id);
		if (!of_property_read_u32(np, "mediatek,larb-id", &id) &&
		    id == larb_id)
			break;
		of_node_put(np);
	}

	ret = of_device_is_available(np) ? of_address_to_resource(np, 0, &res) :
					   -ENODEV;
	if (!ret && resource_size(&res) < SMI_LARB_NONSEC_CON(port) + sizeof(u32))
		ret = -EINVAL;
	larb_pdev = ret ? NULL : of_find_device_by_node(np);
	of_node_put(np);
	if (ret)
		return dev_err_probe(dev, ret, "LARB%u port %u is unusable\n",
				     larb_id, port);
	if (!larb_pdev)
		return dev_err_probe(dev, -EPROBE_DEFER, "LARB%u not created\n",
				     larb_id);
	cam->larb = &larb_pdev->dev;
	ret = devm_add_action_or_reset(dev, mtk_camsv_put_device, cam->larb);
	if (ret)
		return ret;

	scoped_guard(device, cam->larb)
		bound = device_is_bound(cam->larb);
	if (!bound)
		return dev_err_probe(dev, -EPROBE_DEFER, "%s not bound\n",
				     dev_name(cam->larb));

	/* Read only, without requesting the region: the SMI driver owns it. */
	cam->larb_port_con = devm_ioremap(dev, res.start + SMI_LARB_NONSEC_CON(port),
					  sizeof(u32));
	return cam->larb_port_con ? 0 : -ENOMEM;
}

/* ----------------------------------------------------------------------------
 * Sub-device
 */

static int mtk_camsv_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct mtk_camsv *cam = sd_to_mtk_camsv(sd);
	const struct v4l2_mbus_framefmt def = {
		.code = cam->soc->formats[0].code,
		.width = MTK_CAMSV_DEF_WIDTH,
		.height = MTK_CAMSV_DEF_HEIGHT,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};

	*v4l2_subdev_state_get_format(state, MTK_CAMSV_PAD_SINK) = def;
	*v4l2_subdev_state_get_format(state, MTK_CAMSV_PAD_SRC) = def;
	return 0;
}

static int mtk_camsv_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct mtk_camsv *cam = sd_to_mtk_camsv(sd);

	if (code->pad == MTK_CAMSV_PAD_SRC) {
		/* The source pad passes the sink format through. */
		if (code->index)
			return -EINVAL;
		code->code = v4l2_subdev_state_get_format(state,
							  MTK_CAMSV_PAD_SINK)->code;
		return 0;
	}
	if (code->index >= cam->soc->num_formats)
		return -EINVAL;
	code->code = cam->soc->formats[code->index].code;
	return 0;
}

static int mtk_camsv_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct mtk_camsv *cam = sd_to_mtk_camsv(sd);
	const struct mtk_camsv_format *info;
	struct v4l2_mbus_framefmt *sink;

	if (fmt->pad == MTK_CAMSV_PAD_SRC)
		return v4l2_subdev_get_fmt(sd, state, fmt);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	info = mtk_camsv71_find_code(cam, fmt->format.code) ?: &cam->soc->formats[0];
	sink = v4l2_subdev_state_get_format(state, MTK_CAMSV_PAD_SINK);
	sink->code = info->code;
	sink->width = clamp_t(u32, ALIGN_DOWN(fmt->format.width, MTK_CAMSV_WIDTH_ALIGN),
			      MTK_CAMSV_MIN_WIDTH, MTK_CAMSV_MAX_WIDTH);
	sink->height = clamp_t(u32, ALIGN_DOWN(fmt->format.height, MTK_CAMSV_HEIGHT_ALIGN),
			       MTK_CAMSV_MIN_HEIGHT, MTK_CAMSV_MAX_HEIGHT);
	sink->field = V4L2_FIELD_NONE;
	sink->colorspace = fmt->format.colorspace ?: V4L2_COLORSPACE_RAW;
	sink->ycbcr_enc = fmt->format.ycbcr_enc;
	sink->quantization = fmt->format.quantization;
	sink->xfer_func = fmt->format.xfer_func;
	fmt->format = *sink;

	*v4l2_subdev_state_get_format(state, MTK_CAMSV_PAD_SRC) = *sink;
	return 0;
}

/* Called with the CAMSV armed: the SENINF route, its PHY and the sensor. */
static int mtk_camsv_enable_streams(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state, u32 pad,
				    u64 streams_mask)
{
	struct mtk_camsv *cam = sd_to_mtk_camsv(sd);
	struct media_pad *remote;
	int ret;

	remote = media_pad_remote_pad_unique(&cam->pads[MTK_CAMSV_PAD_SINK]);
	if (IS_ERR(remote))
		return PTR_ERR(remote);
	if (!is_media_entity_v4l2_subdev(remote->entity))
		return -EINVAL;

	ret = v4l2_subdev_enable_streams(media_entity_to_v4l2_subdev(remote->entity),
					 remote->index, BIT_ULL(0));
	if (ret)
		return ret;
	cam->seninf_pad = remote;
	return 0;
}

static int mtk_camsv_disable_streams(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state, u32 pad,
				     u64 streams_mask)
{
	struct mtk_camsv *cam = sd_to_mtk_camsv(sd);
	struct media_pad *remote = cam->seninf_pad;
	int ret;

	ret = v4l2_subdev_disable_streams(media_entity_to_v4l2_subdev(remote->entity),
					  remote->index, BIT_ULL(0));
	if (!ret)
		cam->seninf_pad = NULL;
	return ret;
}

static int mtk_camsv_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				     struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC || sub->id)
		return -EINVAL;
	return v4l2_event_subscribe(fh, sub, MTK_CAMSV_EVENT_DEPTH, NULL);
}

static const struct v4l2_subdev_core_ops mtk_camsv_core_ops = {
	.subscribe_event = mtk_camsv_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops mtk_camsv_pad_ops = {
	.enum_mbus_code = mtk_camsv_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mtk_camsv_set_fmt,
	.enable_streams = mtk_camsv_enable_streams,
	.disable_streams = mtk_camsv_disable_streams,
};

static const struct v4l2_subdev_ops mtk_camsv_subdev_ops = {
	.core = &mtk_camsv_core_ops,
	.pad = &mtk_camsv_pad_ops,
};

static int mtk_camsv_registered(struct v4l2_subdev *sd)
{
	return mtk_camsv71_video_register(sd_to_mtk_camsv(sd));
}

static void mtk_camsv_unregistered(struct v4l2_subdev *sd)
{
	mtk_camsv71_video_unregister(sd_to_mtk_camsv(sd));
}

static const struct v4l2_subdev_internal_ops mtk_camsv_internal_ops = {
	.init_state = mtk_camsv_init_state,
	.registered = mtk_camsv_registered,
	.unregistered = mtk_camsv_unregistered,
};

static const struct media_entity_operations mtk_camsv_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

/* ----------------------------------------------------------------------------
 * Power management
 */

static int mtk_camsv_runtime_suspend(struct device *dev)
{
	struct mtk_camsv *cam = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(MTK_CAMSV_NUM_CLKS, cam->clks);
	return 0;
}

static int mtk_camsv_runtime_resume(struct device *dev)
{
	struct mtk_camsv *cam = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(MTK_CAMSV_NUM_CLKS, cam->clks);
}

/*
 * There is no verified way to stop and restart a stream across system sleep,
 * and a quarantined CAMSV keeps its power.
 */
static int mtk_camsv_suspend(struct device *dev)
{
	struct mtk_camsv *cam = dev_get_drvdata(dev);

	if (cam->quarantined || vb2_is_streaming(&cam->queue)) {
		dev_warn(dev, "%s: refusing system suspend\n",
			 cam->quarantined ? "quarantined" : "streaming");
		return -EBUSY;
	}
	return pm_runtime_force_suspend(dev);
}

static int mtk_camsv_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static const struct dev_pm_ops mtk_camsv_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(mtk_camsv_suspend, mtk_camsv_resume)
	RUNTIME_PM_OPS(mtk_camsv_runtime_suspend, mtk_camsv_runtime_resume, NULL)
};

/* ----------------------------------------------------------------------------
 * Probe: resources only, no register access
 */

static int mtk_camsv_probe(struct platform_device *pdev)
{
	const struct dev_pm_domain_attach_data pd_data = {
		.pd_names = mtk_camsv_pd_names,
		.num_pd_names = ARRAY_SIZE(mtk_camsv_pd_names),
	};
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *pd_list;
	struct iommu_domain *domain;
	struct v4l2_subdev *sd;
	struct mtk_camsv *cam;
	struct resource *res;
	unsigned int i;
	int ret;

	cam = devm_kzalloc(dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;
	cam->dev = dev;
	cam->soc = device_get_match_data(dev);
	if (!cam->soc)
		return -ENODEV;
	platform_set_drvdata(pdev, cam);

	domain = iommu_get_domain_for_dev(dev);
	if (!domain || !iommu_is_dma_domain(domain))
		return dev_err_probe(dev, -ENODEV, "no IOMMU DMA domain\n");
	ret = mtk_camsv_find_larb(cam);
	if (ret)
		return ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	if (!res)
		return dev_err_probe(dev, -EINVAL, "no base registers\n");
	for (i = 0; i < cam->soc->num_bases; i++)
		if (cam->soc->bases[i] == res->start)
			break;
	if (i == cam->soc->num_bases)
		return dev_err_probe(dev, -ENODEV, "unknown CAMSV at %pa\n",
				     &res->start);
	cam->id = i;
	cam->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cam->base))
		return PTR_ERR(cam->base);
	cam->inner = devm_platform_ioremap_resource_byname(pdev, "inner");
	if (IS_ERR(cam->inner))
		return PTR_ERR(cam->inner);

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0)
		return cam->irq;
	ret = devm_request_irq(dev, cam->irq, mtk_camsv71_isr, IRQF_NO_AUTOEN,
			       dev_name(dev), cam);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the IRQ\n");

	for (i = 0; i < MTK_CAMSV_NUM_CLKS; i++)
		cam->clks[i].id = mtk_camsv_clk_names[i];
	ret = devm_clk_bulk_get(dev, MTK_CAMSV_NUM_CLKS, cam->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the clocks\n");

	/* The platform core attaches a single domain, not two. */
	if (!dev->pm_domain) {
		ret = devm_pm_domain_attach_list(dev, &pd_data, &pd_list);
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "failed to attach the power domains\n");
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(MTK_CAMSV_DMA_BITS));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set the DMA mask\n");
	/* One contiguous IOVA range per imported buffer. */
	dma_set_max_seg_size(dev, UINT_MAX);

	spin_lock_init(&cam->lock);
	INIT_LIST_HEAD(&cam->queued);
	init_waitqueue_head(&cam->stop_wq);
	ret = devm_mutex_init(dev, &cam->vlock);
	if (ret)
		return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	sd = &cam->subdev;
	v4l2_subdev_init(sd, &mtk_camsv_subdev_ops);
	sd->internal_ops = &mtk_camsv_internal_ops;
	sd->dev = dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	sd->entity.ops = &mtk_camsv_entity_ops;
	snprintf(sd->name, sizeof(sd->name), "mtk-camsv%u", cam->id);
	cam->pads[MTK_CAMSV_PAD_SINK].flags = MEDIA_PAD_FL_SINK |
					      MEDIA_PAD_FL_MUST_CONNECT;
	cam->pads[MTK_CAMSV_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, MTK_CAMSV_NUM_PADS, cam->pads);
	if (ret)
		return ret;
	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity;

	/* The SENINF notifier binds it, which adds the video node. */
	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_state;
	return 0;

err_state:
	v4l2_subdev_cleanup(sd);
err_entity:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void mtk_camsv_remove(struct platform_device *pdev)
{
	struct mtk_camsv *cam = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&cam->subdev);
	v4l2_subdev_cleanup(&cam->subdev);
	media_entity_cleanup(&cam->subdev.entity);
}

static const struct of_device_id mtk_camsv_of_match[] = {
	{ .compatible = "mediatek,mt6895-camsv", .data = &mt6895_camsv_soc },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_camsv_of_match);

static struct platform_driver mtk_camsv_driver = {
	.probe = mtk_camsv_probe,
	.remove = mtk_camsv_remove,
	.driver = {
		.name = MTK_CAMSV71_NAME,
		.of_match_table = mtk_camsv_of_match,
		.pm = pm_ptr(&mtk_camsv_pm_ops),
		/* A stream or a quarantine pins the device until reboot. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_camsv_driver);

MODULE_DESCRIPTION("MediaTek ISP 7.1 CAMSV raw capture driver");
MODULE_LICENSE("GPL");
