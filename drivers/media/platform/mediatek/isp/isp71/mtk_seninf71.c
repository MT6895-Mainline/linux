// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6895 camera sensor interface (SENINF)
 *
 * Copyright (c) 2022 MediaTek Inc.
 *
 * The SENINF receives MIPI CSI-2 on up to ten interfaces and routes each
 * stream through a mux to one of its CAM_MUX outputs, which feed the camera
 * engines. This driver owns the media device of the camera pipeline. Its
 * subdev has one sink pad per interface and one source pad per CAM_MUX; the
 * pad numbers are the device tree port numbers, so the device tree graph
 * describes the SoC-fixed wiring of the CAM_MUX outputs to the engines.
 *
 * One sensor streams at a time, selected by its media link: at most one
 * sensor link is enabled, and the CAM_MUX outputs carry the format of the
 * interface that link ends on, as in a video mux. There is no user-visible
 * routing.
 *
 * The V4L2 structure follows the MediaTek ISP 3.0 SENINF driver.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/string.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "mtk_seninf71.h"

#define MTK_SENINF_DRIVER_NAME		"mtk-seninf71"
/* The name userspace (libcamera) matches the camera media device by */
#define MTK_SENINF_MEDIA_DRIVER_NAME	"mtk-seninf"

#define MTK_SENINF_MAX_LANES		4
#define MTK_SENINF_MIN_WIDTH		2
#define MTK_SENINF_MAX_WIDTH		65534
#define MTK_SENINF_MIN_HEIGHT		1
#define MTK_SENINF_MAX_HEIGHT		65535
/* Range of the mediatek,hs-trail-ps endpoint property */
#define MTK_SENINF_MIN_HS_TRAIL_PS	60000
#define MTK_SENINF_MAX_HS_TRAIL_PS	1000000

/*
 * Sensor ports (bit n = DT port n) left out of the graph, so that the media
 * device registers without a sensor that fails to probe.
 */
static unsigned int ignore_ports;
module_param(ignore_ports, uint, 0444);
MODULE_PARM_DESC(ignore_ports, "Bitmask of sensor ports (bit n = DT port n) to leave out");

struct mtk_seninf_format_info {
	u32 code;
	u8 dt;
	u8 bpp;
};

static const struct mtk_seninf_format_info mtk_seninf_formats[] = {
	{ MEDIA_BUS_FMT_SBGGR8_1X8, MIPI_CSI2_DT_RAW8, 8 },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, MIPI_CSI2_DT_RAW8, 8 },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, MIPI_CSI2_DT_RAW8, 8 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, MIPI_CSI2_DT_RAW8, 8 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, MIPI_CSI2_DT_RAW10, 10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, MIPI_CSI2_DT_RAW10, 10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_CSI2_DT_RAW10, 10 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, MIPI_CSI2_DT_RAW10, 10 },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, MIPI_CSI2_DT_RAW12, 12 },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, MIPI_CSI2_DT_RAW12, 12 },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, MIPI_CSI2_DT_RAW12, 12 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, MIPI_CSI2_DT_RAW12, 12 },
};

static const struct v4l2_mbus_framefmt mtk_seninf_default_fmt = {
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.width = 1600,
	.height = 1200,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.xfer_func = V4L2_XFER_FUNC_NONE,
};

struct mtk_seninf_asd {
	struct v4l2_async_connection asc;
	unsigned int port;
};

static inline struct mtk_seninf *sd_to_mtk_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_seninf, sd);
}

static inline struct mtk_seninf *notifier_to_mtk_seninf(struct v4l2_async_notifier *n)
{
	return container_of(n, struct mtk_seninf, notifier);
}

static inline struct mtk_seninf_asd *to_mtk_seninf_asd(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct mtk_seninf_asd, asc);
}

static bool mtk_seninf_pad_is_sink(struct mtk_seninf *priv, unsigned int pad)
{
	return pad < priv->soc->num_intf;
}

static const struct mtk_seninf_format_info *mtk_seninf_format_info(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_seninf_formats); i++)
		if (mtk_seninf_formats[i].code == code)
			return &mtk_seninf_formats[i];

	return NULL;
}

/* -----------------------------------------------------------------------------
 * Subdev operations
 */

/* Every pad starts with the default format. */
static int mtk_seninf_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	unsigned int pad;

	for (pad = 0; pad < sd->entity.num_pads; pad++)
		*v4l2_subdev_state_get_format(state, pad) = mtk_seninf_default_fmt;

	return 0;
}

/* The connected outputs carry the format of the interface @sink. */
static void mtk_seninf_propagate(struct mtk_seninf *priv, struct v4l2_subdev_state *state,
				 unsigned int sink)
{
	const struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, sink);
	unsigned int i;

	for (i = 0; i < priv->soc->num_cammux; i++)
		if (priv->outputs[i].connected)
			*v4l2_subdev_state_get_format(state, priv->soc->num_intf + i) = *fmt;
}

static int mtk_seninf_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_mbus_code_enum *code)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	const struct v4l2_mbus_framefmt *fmt;

	if (mtk_seninf_pad_is_sink(priv, code->pad)) {
		if (code->index >= ARRAY_SIZE(mtk_seninf_formats))
			return -EINVAL;

		code->code = mtk_seninf_formats[code->index].code;
		return 0;
	}

	/* A source pad carries the format of the selected interface unchanged. */
	if (code->index)
		return -EINVAL;

	fmt = v4l2_subdev_state_get_format(state, code->pad);
	code->code = fmt->code;
	return 0;
}

static int mtk_seninf_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_mbus_framefmt *format;

	/* The SENINF does not process pixels: a source pad mirrors its sink. */
	if (!mtk_seninf_pad_is_sink(priv, fmt->pad))
		return v4l2_subdev_get_fmt(sd, state, fmt);

	/* A started pipeline holds every pad of the SENINF. */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    (media_pad_is_streaming(&priv->pads[fmt->pad]) || priv->inputs[fmt->pad].streaming))
		return -EBUSY;

	if (!mtk_seninf_format_info(fmt->format.code))
		fmt->format.code = mtk_seninf_default_fmt.code;

	/* Two pixels per clock: even widths. The sizes are 16-bit fields. */
	fmt->format.width = clamp(ALIGN_DOWN(fmt->format.width, 2),
				  MTK_SENINF_MIN_WIDTH, MTK_SENINF_MAX_WIDTH);
	fmt->format.height = clamp(fmt->format.height,
				   MTK_SENINF_MIN_HEIGHT, MTK_SENINF_MAX_HEIGHT);
	fmt->format.field = V4L2_FIELD_NONE;

	format = v4l2_subdev_state_get_format(state, fmt->pad);
	*format = fmt->format;

	/*
	 * The ACTIVE outputs mirror the interface of the enabled sensor link. A
	 * TRY format propagates from any interface, independently of the links.
	 */
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY || (int)fmt->pad == priv->active_sink)
		mtk_seninf_propagate(priv, state, fmt->pad);

	return 0;
}

/*
 * Link rate, lane count, virtual channel and data type of the sensor stream.
 * The lane rate comes from the sensor's link frequency.
 */
static int mtk_seninf_get_bus(struct mtk_seninf *priv, struct mtk_seninf_input *in,
			      const struct mtk_seninf_format_info *info,
			      struct mtk_seninf_route *route)
{
	const struct v4l2_mbus_frame_desc_entry *entry;
	struct v4l2_mbus_frame_desc fd;
	unsigned int i;
	s64 link_freq;
	int lanes, ret;

	lanes = v4l2_get_active_data_lanes(in->source_pad, in->bus.num_data_lanes);
	if (lanes < 0)
		return lanes;

	link_freq = v4l2_get_link_freq(in->source_pad, info->bpp, 2 * lanes);
	if (link_freq < 0) {
		dev_err(priv->dev, "no link frequency from %s: %lld\n",
			in->source_sd->name, link_freq);
		return link_freq;
	}

	route->lanes = lanes;
	route->lane_rate = 2 * link_freq;
	route->vc = 0;
	route->dt = info->dt;

	ret = v4l2_subdev_call(in->source_sd, pad, get_frame_desc, in->source_pad->index, &fd);
	if (ret == -ENOIOCTLCMD)
		return 0;
	if (ret)
		return ret;

	if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2)
		return -EINVAL;

	for (i = 0; i < fd.num_entries; i++) {
		entry = &fd.entry[i];
		if (entry->stream)
			continue;

		if (entry->bus.csi2.dt != info->dt || entry->bus.csi2.vc > MTK_SENINF_MAX_VC) {
			dev_err(priv->dev, "%s sends VC %u DT %#x, expected DT %#x\n",
				in->source_sd->name, entry->bus.csi2.vc,
				entry->bus.csi2.dt, info->dt);
			return -EPIPE;
		}

		route->vc = entry->bus.csi2.vc;
		return 0;
	}

	dev_err(priv->dev, "%s describes no stream 0\n", in->source_sd->name);
	return -EPIPE;
}

static void mtk_seninf_check_clk_rates(struct mtk_seninf *priv)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	unsigned long rate;
	unsigned int i;

	for (i = 0; i < soc->num_clks; i++) {
		if (!soc->clks[i].csi_rate)
			continue;

		rate = clk_get_rate(priv->clks[i].clk);
		if (rate != soc->csi_clk_hz)
			dev_warn(priv->dev, "%s runs at %lu Hz, the CSI-2 timing assumes %lu Hz\n",
				 soc->clks[i].name, rate, soc->csi_clk_hz);
	}
}

/*
 * As the vendor driver assumes, the muxes, the CAM_MUXes and the camera
 * engines move 2^pixel_mode pixels per ISP clock cycle. Refuse a stream whose
 * CSI-2 pixel rate, the lane rate times the lanes over the bits per pixel,
 * exceeds that.
 */
static int mtk_seninf_check_pixel_rate(struct mtk_seninf *priv,
				       const struct mtk_seninf_route *route, unsigned int bpp)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	u64 pixel_rate = div_u64(route->lane_rate * route->lanes, bpp);
	unsigned long isp_hz = 0;
	unsigned int i;

	for (i = 0; i < soc->num_clks; i++)
		if (soc->clks[i].isp_rate)
			isp_hz = clk_get_rate(priv->clks[i].clk);

	dev_dbg(priv->dev, "pixel rate %llu/s, ISP clock %lu Hz, %u pixels per cycle\n",
		pixel_rate, isp_hz, 1U << soc->pixel_mode);

	if (pixel_rate > ((u64)isp_hz << soc->pixel_mode)) {
		dev_err(priv->dev, "pixel rate %llu/s exceeds %u pixels per cycle at %lu Hz\n",
			pixel_rate, 1U << soc->pixel_mode, isp_hz);
		return -ERANGE;
	}

	return 0;
}

/*
 * Called by the camera engine on a source pad once it is armed. The sensor
 * starts last, after the route is open.
 */
static int mtk_seninf_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				     u32 pad, u64 streams_mask)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	const struct v4l2_mbus_framefmt *fmt, *out_fmt;
	const struct mtk_seninf_format_info *info;
	struct mtk_seninf_route route = {};
	struct mtk_seninf_output *out;
	struct mtk_seninf_input *in;
	int ret;

	/* The enabled sensor link selects the interface. */
	if (priv->active_sink < 0)
		return -ENOLINK;

	route.cammux = pad - priv->soc->num_intf;
	in = &priv->inputs[priv->active_sink];
	out = &priv->outputs[route.cammux];
	if (!in->source_sd || !out->pm_link)
		return -ENOLINK;
	if (in->streaming)
		return -EBUSY;

	/* SENINF registers are only accessed while the engine is powered. */
	if (!pm_runtime_active(out->pm_link->consumer)) {
		dev_err(priv->dev, "consumer of pad %u is not powered\n", pad);
		return -EIO;
	}

	fmt = v4l2_subdev_state_get_format(state, priv->active_sink);
	out_fmt = v4l2_subdev_state_get_format(state, pad);
	if (fmt->code != out_fmt->code || fmt->width != out_fmt->width ||
	    fmt->height != out_fmt->height)
		return -EPIPE;

	info = mtk_seninf_format_info(fmt->code);
	if (!info)
		return -EINVAL;

	route.width = fmt->width;
	route.height = fmt->height;
	ret = mtk_seninf_get_bus(priv, in, info, &route);
	if (ret)
		return ret;

	mtk_seninf_check_clk_rates(priv);
	ret = mtk_seninf_check_pixel_rate(priv, &route, info->bpp);
	if (ret)
		return ret;

	/* Keep the module while the hardware streams. */
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	ret = pm_runtime_resume_and_get(priv->dev);
	if (ret)
		goto err_module_put;

	ret = mtk_seninf_route_start(priv, in, &route);
	if (ret)
		goto err_pm_put;

	ret = v4l2_subdev_enable_streams(in->source_sd, in->source_pad->index, BIT_ULL(0));
	if (ret) {
		dev_err(priv->dev, "failed to start %s: %d\n", in->source_sd->name, ret);
		goto err_route_stop;
	}

	return 0;

err_route_stop:
	mtk_seninf_route_stop(priv, in);
err_pm_put:
	pm_runtime_put(priv->dev);
err_module_put:
	module_put(THIS_MODULE);
	return ret;
}

/*
 * Called by the camera engine once it is idle. The sensor stops first. The
 * route is torn down even if the sensor fails to stop, so that the stream
 * state stays consistent with the hardware.
 */
static int mtk_seninf_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				      u32 pad, u64 streams_mask)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	unsigned int cammux = pad - priv->soc->num_intf;
	struct mtk_seninf_input *in = NULL;
	unsigned int i;
	int ret;

	for (i = 0; i < priv->soc->num_intf; i++) {
		if (priv->inputs[i].streaming && priv->inputs[i].cammux == cammux) {
			in = &priv->inputs[i];
			break;
		}
	}
	if (!in)
		return -EINVAL;

	if (in->source_sd) {
		ret = v4l2_subdev_disable_streams(in->source_sd, in->source_pad->index,
						  BIT_ULL(0));
		if (ret)
			dev_err(priv->dev, "failed to stop %s: %d\n", in->source_sd->name, ret);
	}

	mtk_seninf_route_stop(priv, in);
	pm_runtime_put(priv->dev);
	module_put(THIS_MODULE);

	return 0;
}

static int mtk_seninf_log_status(struct v4l2_subdev *sd)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_subdev_state *state;
	unsigned int i;

	/* Registers are read only for running routes, which keep them powered. */
	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (priv->active_sink >= 0 && priv->inputs[priv->active_sink].source_sd)
		dev_info(priv->dev, "sensor link enabled: %s -> interface %d\n",
			 priv->inputs[priv->active_sink].source_sd->name, priv->active_sink);
	else
		dev_info(priv->dev, "no sensor link enabled\n");

	for (i = 0; i < priv->soc->num_intf; i++)
		if (priv->inputs[i].streaming)
			mtk_seninf_route_log(priv, &priv->inputs[i]);

	if (!priv->mux_busy)
		dev_info(priv->dev, "no route is running\n");

	v4l2_subdev_unlock_state(state);

	return 0;
}

static const struct v4l2_subdev_core_ops mtk_seninf_core_ops = {
	.log_status = mtk_seninf_log_status,
};

static const struct v4l2_subdev_pad_ops mtk_seninf_pad_ops = {
	.enum_mbus_code = mtk_seninf_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mtk_seninf_set_fmt,
	.enable_streams = mtk_seninf_enable_streams,
	.disable_streams = mtk_seninf_disable_streams,
};

static const struct v4l2_subdev_ops mtk_seninf_subdev_ops = {
	.core = &mtk_seninf_core_ops,
	.pad = &mtk_seninf_pad_ops,
};

static const struct v4l2_subdev_internal_ops mtk_seninf_internal_ops = {
	.init_state = mtk_seninf_init_state,
};

/*
 * At most one sensor link is enabled, and links change only while no route
 * runs. The runtime PM link that powers a sensor with the SENINF follows the
 * media link, so that only the selected sensor powers up.
 */
static int mtk_seninf_link_setup(struct media_entity *entity, const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct device_link *old_link = NULL;
	struct v4l2_subdev_state *state;
	struct mtk_seninf_input *in;
	int sink = local->index;
	int ret = 0;

	/* The CAM_MUX outputs are wired to their engines. */
	if (!mtk_seninf_pad_is_sink(priv, local->index))
		return 0;

	in = &priv->inputs[sink];
	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (priv->mux_busy) {
		ret = -EBUSY;
		goto out;
	}

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (priv->active_sink == sink)
			goto out;
		if (priv->active_sink >= 0) {
			dev_dbg(priv->dev, "interface %d already has the sensor link\n",
				priv->active_sink);
			ret = -EBUSY;
			goto out;
		}
		if (!in->source_sd || remote != in->source_pad) {
			ret = -EINVAL;
			goto out;
		}

		in->pm_link = device_link_add(priv->dev, in->source_sd->dev,
					      DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!in->pm_link) {
			dev_err(priv->dev, "failed to link to %s\n", in->source_sd->name);
			ret = -EINVAL;
			goto out;
		}

		priv->active_sink = sink;
		mtk_seninf_propagate(priv, state, sink);
	} else if (priv->active_sink == sink) {
		old_link = in->pm_link;
		in->pm_link = NULL;
		priv->active_sink = -1;
	}

out:
	v4l2_subdev_unlock_state(state);
	if (old_link)
		device_link_del(old_link);

	return ret;
}

/* All pads are interdependent: a stream claims every sensor link. */
static const struct media_entity_operations mtk_seninf_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_setup = mtk_seninf_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Async notifier
 */

/* The sensor pad of the link from @sensor to the interface @port, enabled or not. */
static struct media_pad *mtk_seninf_sensor_pad(struct mtk_seninf *priv, unsigned int port,
					       struct media_entity *sensor)
{
	struct media_link *link;

	for_each_media_entity_data_link(&priv->sd.entity, link)
		if (link->sink == &priv->pads[port] && link->source->entity == sensor)
			return link->source;

	return NULL;
}

static int mtk_seninf_bound_input(struct mtk_seninf *priv, struct v4l2_subdev *sd,
				  unsigned int port)
{
	struct mtk_seninf_input *in = &priv->inputs[port];
	struct device_link *link = NULL;
	struct media_pad *pad;
	bool enable;
	int ret;

	guard(mutex)(&priv->lock);

	/* The lowest connected interface starts with its sensor link enabled. */
	enable = (int)port == priv->default_sink && priv->active_sink < 0;

	ret = v4l2_create_fwnode_links_to_pad(sd, &priv->pads[port],
					      enable ? MEDIA_LNK_FL_ENABLED : 0);
	if (ret)
		return ret;

	pad = mtk_seninf_sensor_pad(priv, port, &sd->entity);
	if (!pad)
		return -ENOLINK;

	/* Powering the SENINF powers the sensor of the enabled link first. */
	if (enable) {
		link = device_link_add(priv->dev, sd->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!link) {
			dev_err(priv->dev, "failed to link to %s\n", sd->name);
			return -EINVAL;
		}
		priv->active_sink = port;
		mtk_seninf_propagate(priv, v4l2_subdev_get_locked_active_state(&priv->sd), port);
	}

	in->source_sd = sd;
	in->source_pad = pad;
	in->pm_link = link;

	return 0;
}

static int mtk_seninf_bound_output(struct mtk_seninf *priv, struct v4l2_subdev *sd,
				   struct v4l2_async_connection *asc, unsigned int port)
{
	struct mtk_seninf_output *out = &priv->outputs[port - priv->soc->num_intf];
	struct device_link *link;
	int pad, ret;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode, MEDIA_PAD_FL_SINK);
	if (pad < 0)
		return pad;

	ret = v4l2_create_fwnode_links_to_pad(&priv->sd, &sd->entity.pads[pad],
					      MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;

	/* Powering the engine powers the SENINF, and through it the sensor. */
	link = device_link_add(sd->dev, priv->dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	if (!link) {
		dev_err(priv->dev, "failed to link %s\n", sd->name);
		return -EINVAL;
	}

	mutex_lock(&priv->lock);
	out->pm_link = link;
	mutex_unlock(&priv->lock);

	return 0;
}

static int mtk_seninf_notifier_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *sd,
				     struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = notifier_to_mtk_seninf(notifier);
	unsigned int port = to_mtk_seninf_asd(asc)->port;

	dev_dbg(priv->dev, "%s bound to port %u\n", sd->name, port);

	if (mtk_seninf_pad_is_sink(priv, port))
		return mtk_seninf_bound_input(priv, sd, port);

	return mtk_seninf_bound_output(priv, sd, asc, port);
}

static void mtk_seninf_notifier_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *sd,
				       struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = notifier_to_mtk_seninf(notifier);
	unsigned int port = to_mtk_seninf_asd(asc)->port;
	struct device_link *link;

	mutex_lock(&priv->lock);
	if (mtk_seninf_pad_is_sink(priv, port)) {
		struct mtk_seninf_input *in = &priv->inputs[port];

		link = in->pm_link;
		in->pm_link = NULL;
		in->source_sd = NULL;
		in->source_pad = NULL;
		if (priv->active_sink == (int)port)
			priv->active_sink = -1;
	} else {
		struct mtk_seninf_output *out = &priv->outputs[port - priv->soc->num_intf];

		link = out->pm_link;
		out->pm_link = NULL;
	}
	mutex_unlock(&priv->lock);

	/* A sensor whose media link is disabled has no runtime PM link. */
	if (link)
		device_link_del(link);
}

/* All sensors and engines are bound: expose the complete graph. */
static int mtk_seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_seninf *priv = notifier_to_mtk_seninf(notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
	if (ret)
		return ret;

	if (media_devnode_is_registered(priv->mdev.devnode))
		return 0;

	return media_device_register(&priv->mdev);
}

static const struct v4l2_async_notifier_operations mtk_seninf_notifier_ops = {
	.bound = mtk_seninf_notifier_bound,
	.unbind = mtk_seninf_notifier_unbind,
	.complete = mtk_seninf_notifier_complete,
};

/* -----------------------------------------------------------------------------
 * Probe and remove
 */

static int mtk_seninf_parse_input(struct mtk_seninf *priv, struct fwnode_handle *ep,
				  unsigned int port)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct mtk_seninf_input *in = &priv->inputs[port];
	struct v4l2_mbus_config_mipi_csi2 *bus = &vep.bus.mipi_csi2;
	struct device *dev = priv->dev;
	char phy_name[8];
	unsigned int i;
	int ret;

	/* Interface 2N receives CSI port N as a whole (4D1C). */
	if (port % 2)
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "port %u: split CSI ports are not supported\n", port);

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		return dev_err_probe(dev, ret, "port %u: invalid endpoint\n", port);

	/* The lane routing is fixed: only logical lanes 1..N, no inversion. */
	if (!bus->num_data_lanes || bus->num_data_lanes > MTK_SENINF_MAX_LANES)
		return dev_err_probe(dev, -EINVAL, "port %u: %u data lanes\n",
				     port, bus->num_data_lanes);
	for (i = 0; i < bus->num_data_lanes; i++)
		if (bus->data_lanes[i] != i + 1)
			return dev_err_probe(dev, -EINVAL, "port %u: data lanes must be 1..%u\n",
					     port, bus->num_data_lanes);
	for (i = 0; i <= bus->num_data_lanes; i++)
		if (bus->lane_polarities[i])
			return dev_err_probe(dev, -EINVAL,
					     "port %u: lane polarity inversion is not supported\n",
					     port);

	/* The transmitter's T_HS-TRAIL; the D-PHY default applies without it. */
	in->hs_trail_ps = 0;
	if (fwnode_property_present(ep, "mediatek,hs-trail-ps")) {
		ret = fwnode_property_read_u32(ep, "mediatek,hs-trail-ps", &in->hs_trail_ps);
		if (ret || in->hs_trail_ps < MTK_SENINF_MIN_HS_TRAIL_PS ||
		    in->hs_trail_ps > MTK_SENINF_MAX_HS_TRAIL_PS)
			return dev_err_probe(dev, -EINVAL,
					     "port %u: invalid mediatek,hs-trail-ps\n", port);
	}

	dev_dbg(dev, "port %u: %u data lanes, T_HS-TRAIL %u ps (0: D-PHY default)\n",
		port, bus->num_data_lanes, in->hs_trail_ps);

	snprintf(phy_name, sizeof(phy_name), "csi%u", port / 2);
	in->phy = devm_phy_get(dev, phy_name);
	if (IS_ERR(in->phy))
		return dev_err_probe(dev, PTR_ERR(in->phy), "port %u: no PHY %s\n",
				     port, phy_name);

	in->bus = *bus;
	return 0;
}

static int mtk_seninf_parse_endpoint(struct mtk_seninf *priv, struct fwnode_handle *ep)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	struct mtk_seninf_asd *asd;
	struct fwnode_handle *remote;
	struct fwnode_endpoint fep;
	unsigned int port;
	bool available;
	int ret;

	ret = fwnode_graph_parse_endpoint(ep, &fep);
	if (ret)
		return ret;

	port = fep.port;
	if (port >= soc->num_intf + soc->num_cammux)
		return dev_err_probe(priv->dev, -EINVAL, "invalid port %u\n", port);

	/* Skip connections to disabled devices. */
	remote = fwnode_graph_get_remote_port_parent(ep);
	available = fwnode_device_is_available(remote);
	fwnode_handle_put(remote);
	if (!available)
		return 0;

	if (mtk_seninf_pad_is_sink(priv, port) && (ignore_ports & BIT(port))) {
		dev_info(priv->dev, "port %u: sensor ignored (ignore_ports)\n", port);
		return 0;
	}

	if (mtk_seninf_pad_is_sink(priv, port)) {
		if (priv->inputs[port].connected)
			return dev_err_probe(priv->dev, -EINVAL, "port %u: two endpoints\n", port);

		ret = mtk_seninf_parse_input(priv, ep, port);
		if (ret)
			return ret;

		priv->inputs[port].connected = true;
	} else {
		if (priv->outputs[port - soc->num_intf].connected)
			return dev_err_probe(priv->dev, -EINVAL, "port %u: two endpoints\n", port);

		priv->outputs[port - soc->num_intf].connected = true;
	}

	asd = v4l2_async_nf_add_fwnode_remote(&priv->notifier, ep, struct mtk_seninf_asd);
	if (IS_ERR(asd))
		return PTR_ERR(asd);

	asd->port = port;
	return 0;
}

static int mtk_seninf_parse_endpoints(struct mtk_seninf *priv)
{
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	for (i = 0; i < priv->soc->num_intf; i++)
		priv->inputs[i].intf = i;

	fwnode_graph_for_each_endpoint(dev_fwnode(priv->dev), ep) {
		ret = mtk_seninf_parse_endpoint(priv, ep);
		if (ret) {
			fwnode_handle_put(ep);
			return ret;
		}
	}

	/* No sensor link is enabled before the sensors bind. */
	priv->active_sink = -1;
	priv->default_sink = -1;
	for (i = 0; i < priv->soc->num_intf; i++) {
		if (priv->inputs[i].connected) {
			priv->default_sink = i;
			break;
		}
	}

	return 0;
}

static int mtk_seninf_get_clocks(struct mtk_seninf *priv)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	struct device *dev = priv->dev;
	struct clk *clk;
	unsigned int i;

	priv->clks = devm_kcalloc(dev, soc->num_clks, sizeof(*priv->clks), GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < soc->num_clks; i++) {
		if (soc->clks[i].optional)
			clk = devm_clk_get_optional(dev, soc->clks[i].name);
		else
			clk = devm_clk_get(dev, soc->clks[i].name);
		if (IS_ERR(clk))
			return dev_err_probe(dev, PTR_ERR(clk), "failed to get clock %s\n",
					     soc->clks[i].name);

		priv->clks[i].id = soc->clks[i].name;
		priv->clks[i].clk = clk;
	}

	return 0;
}

static int mtk_seninf_attach_power_domains(struct mtk_seninf *priv)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	const struct dev_pm_domain_attach_data data = {
		.pd_names = soc->pd_names,
		.num_pd_names = soc->num_pds,
	};
	struct dev_pm_domain_list *list;
	int ret;

	/* Runtime PM device links to every domain; nothing powers up here. */
	ret = devm_pm_domain_attach_list(priv->dev, &data, &list);
	if (ret < 0)
		return dev_err_probe(priv->dev, ret, "failed to attach power domains\n");
	if (ret != soc->num_pds)
		return dev_err_probe(priv->dev, -ENODEV, "%d of %u power domains\n",
				     ret, soc->num_pds);

	return 0;
}

static int mtk_seninf_subdev_init(struct mtk_seninf *priv)
{
	const struct mtk_seninf_soc *soc = priv->soc;
	unsigned int i, num_pads = soc->num_intf + soc->num_cammux;
	struct v4l2_subdev *sd = &priv->sd;
	int ret;

	v4l2_subdev_init(sd, &mtk_seninf_subdev_ops);
	sd->internal_ops = &mtk_seninf_internal_ops;
	sd->dev = priv->dev;
	sd->fwnode = dev_fwnode(priv->dev);
	sd->state_lock = &priv->lock;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &mtk_seninf_entity_ops;
	strscpy(sd->name, MTK_SENINF_DRIVER_NAME);

	for (i = 0; i < num_pads; i++)
		priv->pads[i].flags = mtk_seninf_pad_is_sink(priv, i) ?
				      MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, num_pads, priv->pads);
	if (ret)
		return ret;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_device_register_subdev(&priv->v4l2_dev, sd);
	if (ret)
		goto err_subdev_cleanup;

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(sd);
err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void mtk_seninf_subdev_cleanup(struct mtk_seninf *priv)
{
	v4l2_device_unregister_subdev(&priv->sd);
	v4l2_subdev_cleanup(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
}

static int mtk_seninf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_seninf *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->soc = device_get_match_data(dev);
	platform_set_drvdata(pdev, priv);

	/* Probe never accesses the registers: they need the engine's clocks. */
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = mtk_seninf_get_clocks(priv);
	if (ret)
		return ret;

	ret = mtk_seninf_attach_power_domains(priv);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	v4l2_async_nf_init(&priv->notifier, &priv->v4l2_dev);
	priv->notifier.ops = &mtk_seninf_notifier_ops;

	ret = mtk_seninf_parse_endpoints(priv);
	if (ret)
		goto err_nf_cleanup;

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_nf_cleanup;

	priv->mdev.dev = dev;
	strscpy(priv->mdev.model, priv->soc->model);
	strscpy(priv->mdev.driver_name, MTK_SENINF_MEDIA_DRIVER_NAME);
	media_device_init(&priv->mdev);

	priv->v4l2_dev.mdev = &priv->mdev;
	ret = v4l2_device_register(dev, &priv->v4l2_dev);
	if (ret)
		goto err_media_cleanup;

	ret = mtk_seninf_subdev_init(priv);
	if (ret)
		goto err_v4l2_unregister;

	/* The media device is registered once every remote subdev is bound. */
	ret = v4l2_async_nf_register(&priv->notifier);
	if (ret)
		goto err_subdev_cleanup;

	return 0;

err_subdev_cleanup:
	mtk_seninf_subdev_cleanup(priv);
err_v4l2_unregister:
	v4l2_device_unregister(&priv->v4l2_dev);
err_media_cleanup:
	media_device_cleanup(&priv->mdev);
err_nf_cleanup:
	v4l2_async_nf_cleanup(&priv->notifier);
	return ret;
}

static void mtk_seninf_remove(struct platform_device *pdev)
{
	struct mtk_seninf *priv = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
	mtk_seninf_subdev_cleanup(priv);
	media_device_unregister(&priv->mdev);
	v4l2_device_unregister(&priv->v4l2_dev);
	media_device_cleanup(&priv->mdev);
}

static int mtk_seninf_runtime_suspend(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(priv->soc->num_clks, priv->clks);

	return 0;
}

/* Clocks only: the registers are programmed when a route starts. */
static int mtk_seninf_runtime_resume(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(priv->soc->num_clks, priv->clks);
}

/* A running route has no defined state across system suspend: refuse it. */
static int mtk_seninf_suspend(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	scoped_guard(mutex, &priv->lock) {
		if (priv->mux_busy)
			return -EBUSY;
	}

	return pm_runtime_force_suspend(dev);
}

static const struct dev_pm_ops mtk_seninf_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(mtk_seninf_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(mtk_seninf_runtime_suspend, mtk_seninf_runtime_resume, NULL)
};

static const struct mtk_seninf_clk_info mt6895_seninf_clks[] = {
	{ .name = "seninf", .isp_rate = true },
	{ .name = "top-seninf", .csi_rate = true },
	{ .name = "top-seninf1", .csi_rate = true },
	{ .name = "top-seninf2", .csi_rate = true },
	{ .name = "top-seninf3", .csi_rate = true },
	{ .name = "top-seninf4", .csi_rate = true },
	{ .name = "top-camtm", .optional = true },
};

static const char * const mt6895_seninf_pds[] = { "isp", "cam" };

/* The vendor's preferred muxes, without mux 0, which every CAM_MUX selects after reset */
static const u8 mt6895_seninf_mux_alloc[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

static const struct mtk_seninf_soc mt6895_seninf_soc = {
	.model = "MediaTek MT6895 camera",
	.clks = mt6895_seninf_clks,
	.num_clks = ARRAY_SIZE(mt6895_seninf_clks),
	.pd_names = mt6895_seninf_pds,
	.num_pds = ARRAY_SIZE(mt6895_seninf_pds),
	.mux_alloc = mt6895_seninf_mux_alloc,
	.num_mux_alloc = ARRAY_SIZE(mt6895_seninf_mux_alloc),
	.num_intf = 10,
	.num_mux = 17,
	.num_cammux = 23,
	.pixel_mode = 1,
	.csi_clk_hz = 273000000,
};

static const struct of_device_id mtk_seninf_of_match[] = {
	{ .compatible = "mediatek,mt6895-seninf", .data = &mt6895_seninf_soc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);

static struct platform_driver mtk_seninf_driver = {
	.probe = mtk_seninf_probe,
	.remove = mtk_seninf_remove,
	.driver = {
		.name = MTK_SENINF_DRIVER_NAME,
		.of_match_table = mtk_seninf_of_match,
		.pm = pm_ptr(&mtk_seninf_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_seninf_driver);

MODULE_DESCRIPTION("MediaTek MT6895 camera sensor interface driver");
MODULE_LICENSE("GPL");
