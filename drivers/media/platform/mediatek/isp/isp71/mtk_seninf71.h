/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6895 camera sensor interface (SENINF)
 *
 * Copyright (c) 2022 MediaTek Inc.
 */

#ifndef __MTK_SENINF71_H__
#define __MTK_SENINF71_H__

#include <linux/mutex.h>
#include <linux/types.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

struct clk_bulk_data;
struct device;
struct device_link;
struct phy;

#define MTK_SENINF_MAX_INTF		10
#define MTK_SENINF_MAX_CAMMUX		23
#define MTK_SENINF_MAX_PADS		(MTK_SENINF_MAX_INTF + MTK_SENINF_MAX_CAMMUX)
/* Width of the CSI-2 and CAM_MUX virtual channel filters */
#define MTK_SENINF_MAX_VC		31

/**
 * struct mtk_seninf_clk_info - SENINF clock description
 * @name: clock name in the device tree
 * @optional: the clock may be absent
 * @csi_rate: the clock must run at &mtk_seninf_soc.csi_clk_hz
 * @isp_rate: the clock runs at the ISP clock, which bounds the pixel rate
 *	      together with &mtk_seninf_soc.pixel_mode
 */
struct mtk_seninf_clk_info {
	const char *name;
	bool optional;
	bool csi_rate;
	bool isp_rate;
};

/**
 * struct mtk_seninf_soc - SoC-specific SENINF parameters
 * @model: media device model
 * @clks: clocks, enabled while the SENINF is runtime-active
 * @num_clks: number of entries in @clks
 * @pd_names: power domain names
 * @num_pds: number of entries in @pd_names
 * @mux_alloc: muxes that routes may use, in order of preference
 * @num_mux_alloc: number of entries in @mux_alloc
 * @num_intf: number of interfaces, the sink pads 0 to @num_intf - 1
 * @num_mux: number of muxes
 * @num_cammux: number of CAM_MUX outputs, the source pads after the sinks
 * @pixel_mode: pixels per clock (log2) of the muxes and CAM_MUX checkers
 * @csi_clk_hz: SENINF counter clock rate the CSI-2 timing is based on
 */
struct mtk_seninf_soc {
	const char *model;
	const struct mtk_seninf_clk_info *clks;
	unsigned int num_clks;
	const char * const *pd_names;
	unsigned int num_pds;
	const u8 *mux_alloc;
	unsigned int num_mux_alloc;
	unsigned int num_intf;
	unsigned int num_mux;
	unsigned int num_cammux;
	unsigned int pixel_mode;
	unsigned long csi_clk_hz;
};

/**
 * struct mtk_seninf_input - SENINF interface (sink pad)
 * @intf: interface index, equal to the sink pad and DT port number
 * @connected: the DT connects a sensor to this interface
 * @bus: CSI-2 bus configuration of the DT endpoint
 * @hs_trail_ps: T_HS-TRAIL of the transmitter from the DT endpoint, in ps;
 *		 0 for the D-PHY default
 * @phy: CSI-2 receiver PHY of the interface
 * @source_sd: bound sensor subdev
 * @source_pad: sensor source pad linked to this interface
 * @pm_link: runtime PM link from the SENINF to the sensor, present while the
 *	     media link from the sensor is enabled
 * @streaming: a route from this interface is running
 * @cammux: CAM_MUX of the running route
 * @mux: mux of the running route
 * @saved_top_src: interface the mux selected before the route started
 *
 * @source_sd, @source_pad and @pm_link are protected by &mtk_seninf.lock, as
 * is the route state.
 */
struct mtk_seninf_input {
	unsigned int intf;
	bool connected;
	struct v4l2_mbus_config_mipi_csi2 bus;
	u32 hs_trail_ps;
	struct phy *phy;

	struct v4l2_subdev *source_sd;
	struct media_pad *source_pad;
	struct device_link *pm_link;

	bool streaming;
	unsigned int cammux;
	unsigned int mux;
	u32 saved_top_src;
};

/**
 * struct mtk_seninf_output - SENINF CAM_MUX output (source pad)
 * @connected: the DT connects a camera engine to this output
 * @pm_link: runtime PM link from the engine to the SENINF, protected by
 *	     &mtk_seninf.lock
 */
struct mtk_seninf_output {
	bool connected;
	struct device_link *pm_link;
};

/**
 * struct mtk_seninf_route - parameters of a route being started
 * @cammux: CAM_MUX output
 * @lanes: number of CSI-2 data lanes
 * @lane_rate: data rate per lane in bit/s
 * @width: frame width in pixels
 * @height: frame height in lines
 * @vc: CSI-2 virtual channel
 * @dt: CSI-2 data type
 */
struct mtk_seninf_route {
	unsigned int cammux;
	unsigned int lanes;
	u64 lane_rate;
	u32 width;
	u32 height;
	u8 vc;
	u8 dt;
};

/**
 * struct mtk_seninf - SENINF device
 * @dev: platform device
 * @soc: SoC-specific parameters
 * @base: register base
 * @clks: clocks, as described by &mtk_seninf_soc.clks
 * @mdev: media device of the camera pipeline
 * @v4l2_dev: V4L2 device of the camera pipeline
 * @sd: SENINF subdev
 * @pads: media pads, sinks first
 * @notifier: async notifier for the sensors and camera engines
 * @lock: subdev state lock; also serialises the routes and register access
 * @inputs: interfaces
 * @outputs: CAM_MUX outputs
 * @mux_busy: muxes used by running routes
 * @default_sink: lowest connected interface, whose sensor link starts
 *		  enabled, or -1
 * @active_sink: interface whose sensor link is enabled, or -1; protected by
 *		 @lock
 */
struct mtk_seninf {
	struct device *dev;
	const struct mtk_seninf_soc *soc;
	void __iomem *base;
	struct clk_bulk_data *clks;

	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev sd;
	struct media_pad pads[MTK_SENINF_MAX_PADS];
	struct v4l2_async_notifier notifier;

	struct mutex lock; /* see the kernel-doc above */
	struct mtk_seninf_input inputs[MTK_SENINF_MAX_INTF];
	struct mtk_seninf_output outputs[MTK_SENINF_MAX_CAMMUX];
	unsigned long mux_busy;
	int default_sink;
	int active_sink;
};

int mtk_seninf_route_start(struct mtk_seninf *priv, struct mtk_seninf_input *in,
			   const struct mtk_seninf_route *route);
void mtk_seninf_route_stop(struct mtk_seninf *priv, struct mtk_seninf_input *in);
void mtk_seninf_route_log(struct mtk_seninf *priv, const struct mtk_seninf_input *in);

#endif /* __MTK_SENINF71_H__ */
