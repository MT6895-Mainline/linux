// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "mtk_layering_rule.h"
#include "mtk_drm_crtc.h"
#include "mtk_disp_pmqos.h"
#include "mtk_drm_mmp.h"
#include "mtk_drm_drv.h"
#include "mtk_dump.h"
#include <linux/pm_opp.h>
#include <linux/vmalloc.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>

#include <dt-bindings/interconnect/mtk,mmqos.h>
#include <soc/mediatek/mmqos.h>


#define CRTC_NUM		3
static struct drm_crtc *dev_crtc;

/* No request from a CRTC: it holds the lowest step the table offers. */
#define MMCLK_LEVEL_NONE	(-1)
/* Nothing has been programmed on the rail yet. */
#define MMCLK_LEVEL_INVALID	(-2)

/*
 * Display side of the multimedia DVFS interface.
 *
 * VCORE is a shared rail: the DVFSRC provider owns the multimedia clock
 * muxes and serves the highest request of every consumer, so the display
 * only asks for the voltage of the step it needs and never programs a clock
 * or a mux.  The requests of all CRTCs are aggregated in @freq_level and the
 * aggregate is the only thing written to the rail.
 */
struct mtk_disp_mmdvfs {
	struct device *dev;
	/* VCORE request of the display, owned by this context. */
	struct regulator *vcore;
	/*
	 * The display's step table, lowest first.  The frequency names the
	 * step and the voltage is what is requested; neither the clock rate
	 * nor the mux parent is programmed from here.
	 */
	u32 *freq_steps;
	int *step_volt;
	unsigned int step_size;
	/* Level requested by each CRTC, MMCLK_LEVEL_NONE when idle. */
	int freq_level[CRTC_NUM];
	/* Level last programmed, MMCLK_LEVEL_INVALID before the first vote. */
	int voted_level;
	bool opp_table_added;
};

/*
 * The display driver is a single instance, so the state is published here
 * for the runtime callers that only have a CRTC.
 */
static struct mtk_disp_mmdvfs *disp_mmdvfs;

/*
 * Guards @disp_mmdvfs and everything it points to: the lookup and the whole
 * request it leads to, and the allocation and the teardown.  A runtime caller
 * therefore never reaches a context that teardown has already freed, and a
 * rebind cannot overlap the teardown of the previous one.  It is the inner
 * lock of this file - nothing here takes another lock while holding it.
 */
static DEFINE_MUTEX(disp_mmdvfs_lock);

void mtk_disp_pmqos_get_icc_path_name(char *buf, int buf_len,
				struct mtk_ddp_comp *comp, char *qos_event)
{
	int len;

	len = snprintf(buf, buf_len, "%s_%s", mtk_dump_comp_str(comp), qos_event);
}

int __mtk_disp_pmqos_slot_look_up(int comp_id, int mode)
{
	switch (comp_id) {
	case DDP_COMPONENT_OVL0:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL0_FBDC_BW;
		else
			return DISP_PMQOS_OVL0_BW;
	case DDP_COMPONENT_OVL1:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL1_FBDC_BW;
		else
			return DISP_PMQOS_OVL1_BW;
	case DDP_COMPONENT_OVL0_2L:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL0_2L_FBDC_BW;
		else
			return DISP_PMQOS_OVL0_2L_BW;
	case DDP_COMPONENT_OVL1_2L:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL1_2L_FBDC_BW;
		else
			return DISP_PMQOS_OVL1_2L_BW;
	case DDP_COMPONENT_OVL2_2L:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL2_2L_FBDC_BW;
		else
			return DISP_PMQOS_OVL2_2L_BW;
	case DDP_COMPONENT_OVL3_2L:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL3_2L_FBDC_BW;
		else
			return DISP_PMQOS_OVL3_2L_BW;
	case DDP_COMPONENT_OVL0_2L_NWCG:
	case DDP_COMPONENT_OVL2_2L_NWCG:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL0_2L_NWCG_FBDC_BW;
		else
			return DISP_PMQOS_OVL0_2L_NWCG_BW;
	case DDP_COMPONENT_OVL1_2L_NWCG:
	case DDP_COMPONENT_OVL3_2L_NWCG:
		if (mode == DISP_BW_FBDC_MODE)
			return DISP_PMQOS_OVL1_2L_NWCG_FBDC_BW;
		else
			return DISP_PMQOS_OVL1_2L_NWCG_BW;
	case DDP_COMPONENT_RDMA0:
		return DISP_PMQOS_RDMA0_BW;
	case DDP_COMPONENT_RDMA1:
		return DISP_PMQOS_RDMA1_BW;
	case DDP_COMPONENT_RDMA2:
		return DISP_PMQOS_RDMA2_BW;
	case DDP_COMPONENT_WDMA0:
		return DISP_PMQOS_WDMA0_BW;
	case DDP_COMPONENT_WDMA1:
		return DISP_PMQOS_WDMA1_BW;
	default:
		DDPPR_ERR("%s, unknown comp %d\n", __func__, comp_id);
		break;
	}

	return -EINVAL;
}

int __mtk_disp_set_module_bw(struct icc_path *request, int comp_id,
			     unsigned int bandwidth, unsigned int bw_mode)
{
	DDPINFO("%s set %d bw = %u\n", __func__, comp_id, bandwidth);
	bandwidth = bandwidth * 133 / 100;

	mtk_icc_set_bw(request, MBps_to_icc(bandwidth), 0);

	DRM_MMP_MARK(pmqos, comp_id, bandwidth);

	return 0;
}

void __mtk_disp_set_module_hrt(struct icc_path *request,
			       unsigned int bandwidth)
{
	if (bandwidth > 0)
		mtk_icc_set_bw(request, 0, MTK_MMQOS_MAX_BW);
	else
		mtk_icc_set_bw(request, 0, MBps_to_icc(bandwidth));
}

static bool mtk_disp_check_segment(struct mtk_drm_crtc *mtk_crtc,
				struct mtk_drm_private *priv)
{
	bool ret = true;
	int hact = 0;
	int vact = 0;
	int vrefresh = 0;

	if (IS_ERR_OR_NULL(mtk_crtc)) {
		DDPPR_ERR("%s, mtk_crtc is NULL\n", __func__);
		return ret;
	}

	if (IS_ERR_OR_NULL(priv)) {
		DDPPR_ERR("%s, private is NULL\n", __func__);
		return ret;
	}

	hact = mtk_crtc->base.state->adjusted_mode.hdisplay;
	vact = mtk_crtc->base.state->adjusted_mode.vdisplay;
	vrefresh = drm_mode_vrefresh(&mtk_crtc->base.state->adjusted_mode);

	switch (priv->seg_id) {
	case 1:
		if (hact >= 1440)
			ret = false;
		else if (hact >= 1080 && vrefresh > 168)
			ret = false;
		break;
	case 2:
		if (hact >= 1440 && vrefresh > 120)
			ret = false;
		else if (hact >= 1080 && vrefresh > 168)
			ret = false;
		break;
	case 3:
		if (hact >= 1440 && vrefresh > 120)
			ret = false;
		else if (hact >= 1080 && vrefresh > 180)
			ret = false;
		break;
	default:
		ret = true;
		break;
	}

/*
 *	DDPMSG("%s, segment:%d, mode(%d, %d, %d)\n",
 *			__func__, priv->seg_id, hact, vact, vrefresh);
 */

	if (ret == false)
		DDPPR_ERR("%s, check sement fail: segment:%d, mode(%d, %d, %d)\n",
			__func__, priv->seg_id, hact, vact, vrefresh);

	return ret;
}

int mtk_disp_set_hrt_bw(struct mtk_drm_crtc *mtk_crtc, unsigned int bw)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_ddp_comp *comp;
	unsigned int tmp;
	int i, j, ret = 0;

	tmp = bw;

	for (i = 0; i < DDP_PATH_NR; i++) {
		if (!(mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode].req_hrt[i]))
			continue;
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, j, i) {
			ret |= mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_HRT_BW,
						   &tmp);
		}
		if (!mtk_crtc->is_dual_pipe)
			continue;
		for_each_comp_in_dual_pipe(comp, mtk_crtc, j, i)
			ret |= mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_HRT_BW,
					&tmp);
	}

	if (ret == RDMA_REQ_HRT)
		tmp = mtk_drm_primary_frame_bw(crtc);

	if (priv->data->mmsys_id == MMSYS_MT6895) {
		if (mtk_disp_check_segment(mtk_crtc, priv) == false)
			tmp = 1;
	}

	mtk_icc_set_bw(priv->hrt_bw_request, 0, MBps_to_icc(tmp));
	DRM_MMP_MARK(hrt_bw, 0, tmp);
	DDPINFO("set HRT bw %u\n", tmp);

	return ret;
}

void mtk_drm_pan_disp_set_hrt_bw(struct drm_crtc *crtc, const char *caller)
{
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_display_mode *mode;
	unsigned int bw = 0;

	dev_crtc = crtc;
	mtk_crtc = to_mtk_crtc(dev_crtc);
	mode = &crtc->state->adjusted_mode;

	bw = _layering_get_frame_bw(crtc, mode);
	mtk_disp_set_hrt_bw(mtk_crtc, bw);
	DDPINFO("%s:pan_disp_set_hrt_bw: %u\n", caller, bw);
}

int mtk_disp_hrt_cond_change_cb(struct notifier_block *nb, unsigned long value,
				void *v)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(dev_crtc);
	int i, ret;
	unsigned int hrt_idx;

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	/* No need to repaint when display suspend */
	if (!mtk_crtc->enabled) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

		return 0;
	}

	switch (value) {
	case BW_THROTTLE_START: /* CAM on */
		DDPMSG("DISP BW Throttle start\n");
		/* TODO: concider memory session */
		DDPINFO("CAM trigger repaint\n");
		hrt_idx = _layering_rule_get_hrt_idx();
		hrt_idx++;
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		drm_trigger_repaint(DRM_REPAINT_FOR_IDLE, dev_crtc->dev);
		for (i = 0; i < 5; ++i) {
			ret = wait_event_timeout(
				mtk_crtc->qos_ctx->hrt_cond_wq,
				atomic_read(&mtk_crtc->qos_ctx->hrt_cond_sig),
				HZ / 5);
			if (ret == 0)
				DDPINFO("wait repaint timeout %d\n", i);
			atomic_set(&mtk_crtc->qos_ctx->hrt_cond_sig, 0);
			if (atomic_read(&mtk_crtc->qos_ctx->last_hrt_idx) >=
			    hrt_idx)
				break;
		}
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
		break;
	case BW_THROTTLE_END: /* CAM off */
		DDPMSG("DISP BW Throttle end\n");
		/* TODO: switch DC */
		break;
	default:
		break;
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

struct notifier_block pmqos_hrt_notifier = {
	.notifier_call = mtk_disp_hrt_cond_change_cb,
};

int mtk_disp_hrt_bw_dbg(void)
{
	mtk_disp_hrt_cond_change_cb(NULL, BW_THROTTLE_START, NULL);

	return 0;
}

int mtk_disp_hrt_cond_init(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_drm_private *priv;

	dev_crtc = crtc;
	mtk_crtc = to_mtk_crtc(dev_crtc);

	if (IS_ERR_OR_NULL(mtk_crtc)) {
		DDPPR_ERR("%s:mtk_crtc is NULL\n", __func__);
		return -EINVAL;
	}

	priv = mtk_crtc->base.dev->dev_private;

	mtk_crtc->qos_ctx = vmalloc(sizeof(struct mtk_drm_qos_ctx));
	if (mtk_crtc->qos_ctx == NULL) {
		DDPPR_ERR("%s:allocate qos_ctx failed\n", __func__);
		return -ENOMEM;
	}
	memset(mtk_crtc->qos_ctx, 0, sizeof(struct mtk_drm_qos_ctx));
	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
		mtk_mmqos_register_bw_throttle_notifier(&pmqos_hrt_notifier);

	return 0;
}

/*
 * Read the operating points of the display once and keep, for every step,
 * the frequency that names it and the voltage that is requested.  The
 * topckgen mux parents behind those frequencies belong to the DVFSRC
 * provider, so no clock rate and no mux parent is programmed here.
 */
static int mtk_disp_mmdvfs_parse_opp(struct mtk_disp_mmdvfs *ctx)
{
	struct device *dev = ctx->dev;
	struct dev_pm_opp *opp;
	unsigned long freq = 0, volt;
	int count, i, ret;

	count = dev_pm_opp_get_opp_count(dev);
	if (count <= 0) {
		dev_err(dev, "no usable display operating points: %d\n",
			count);
		return count ? count : -ENODEV;
	}

	ctx->freq_steps = kcalloc(count, sizeof(*ctx->freq_steps),
				  GFP_KERNEL);
	ctx->step_volt = kcalloc(count, sizeof(*ctx->step_volt), GFP_KERNEL);
	if (!ctx->freq_steps || !ctx->step_volt)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		opp = dev_pm_opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp)) {
			ret = PTR_ERR(opp);
			dev_err(dev, "operating point %d is missing: %d\n",
				i, ret);
			return ret;
		}
		volt = dev_pm_opp_get_voltage(opp);
		dev_pm_opp_put(opp);

		if (!volt || volt > INT_MAX) {
			dev_err(dev, "operating point %lu Hz has no usable voltage (%lu uV)\n",
				freq, volt);
			return -EINVAL;
		}
		if (freq > U32_MAX) {
			dev_err(dev, "operating point %lu Hz does not fit the display clock steps\n",
				freq);
			return -EINVAL;
		}

		ctx->freq_steps[i] = freq;
		ctx->step_volt[i] = volt;
		freq++;
	}
	ctx->step_size = count;

	dev_info(dev, "mmdvfs: %d display steps, %u Hz to %u Hz\n", count,
		 ctx->freq_steps[0], ctx->freq_steps[count - 1]);

	return 0;
}

static int mtk_disp_mmdvfs_probe(struct mtk_disp_mmdvfs *ctx)
{
	struct device *dev = ctx->dev;
	bool opp_declared, supply_declared;
	int ret;

	opp_declared =
		of_property_present(dev->of_node, "operating-points-v2");
	supply_declared =
		of_property_present(dev->of_node, "dvfsrc-vcore-supply");

	/*
	 * A device that names neither has no way to describe the step it
	 * needs, which is normal on the older SoCs, so voting stays off.
	 */
	if (!opp_declared && !supply_declared) {
		dev_info(dev, "no display OPP table, mmdvfs voting disabled\n");
		return 0;
	}

	/*
	 * A step is named by its OPP entry and requested from the shared
	 * rail, so one without the other is a broken description and must
	 * not be silently ignored.
	 */
	if (!opp_declared || !supply_declared) {
		dev_err(dev, "incomplete mmdvfs description: an OPP table needs a dvfsrc-vcore supply\n");
		return -EINVAL;
	}

	ret = dev_pm_opp_of_add_table(dev);
	if (ret) {
		dev_err(dev, "failed to add the display OPP table: %d\n", ret);
		return ret;
	}
	ctx->opp_table_added = true;

	/*
	 * Every step of a declared table carries a voltage that has to be
	 * requested, so this has to be a real supply.  A provider that has
	 * not probed yet defers the whole display bind.
	 */
	ctx->vcore = regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(ctx->vcore)) {
		ret = PTR_ERR(ctx->vcore);
		ctx->vcore = NULL;
		return ret;
	}
	if (!ctx->vcore) {
		dev_err(dev, "no dvfsrc-vcore supply for the display\n");
		return -ENODEV;
	}

	return mtk_disp_mmdvfs_parse_opp(ctx);
}

/* disp_mmdvfs_lock is held, and no request can be in flight. */
static void mtk_disp_mmdvfs_release(struct mtk_disp_mmdvfs *ctx)
{
	if (ctx->opp_table_added) {
		dev_pm_opp_of_remove_table(ctx->dev);
		ctx->opp_table_added = false;
	}

	kfree(ctx->freq_steps);
	ctx->freq_steps = NULL;
	kfree(ctx->step_volt);
	ctx->step_volt = NULL;
	ctx->step_size = 0;

	/*
	 * Drop the consumer without writing a voltage.  The rail keeps
	 * whatever the last aggregate asked for, because the provider owns
	 * VCORE and the other consumers are still voting on it; the display
	 * only gives up its own request here.  Lowering it is for a shutdown
	 * that is known to be final, which is not this path.
	 */
	if (ctx->vcore) {
		regulator_put(ctx->vcore);
		ctx->vcore = NULL;
	}

	/* The next bind must not inherit the previous one's votes. */
	kfree(ctx);
}

/*
 * Request the voltage of @level from the shared rail.  @level is a step of
 * the display table, MMCLK_LEVEL_NONE for the lowest one.
 */
static int mtk_disp_mmdvfs_vote(struct mtk_disp_mmdvfs *ctx, int level)
{
	int volt, ret;

	volt = ctx->step_volt[level >= 0 ? level : 0];

	/*
	 * "At least this voltage": the rail serves the highest request of
	 * all multimedia consumers, so only the minimum is named here.
	 */
	ret = regulator_set_voltage(ctx->vcore, volt, INT_MAX);
	if (ret)
		dev_err(ctx->dev, "VCORE request for %d uV failed: %d\n",
			volt, ret);

	return ret;
}

/* disp_mmdvfs_lock is held. */
static void mtk_disp_mmdvfs_request(struct mtk_disp_mmdvfs *ctx,
				    struct drm_crtc *crtc, int level,
				    const char *caller)
{
	unsigned int idx = drm_crtc_index(crtc);
	int old_level, vote_level, i, ret;

	if (idx >= CRTC_NUM) {
		DDPPR_ERR("%s: crtc index %u is out of range\n", caller, idx);
		return;
	}

	/*
	 * A caller that asks for a step above the table gets the top one: the
	 * rail must cover what it asked for, so clamping down to idle would
	 * lower the voltage instead.  Only a negative level means idle.
	 */
	if (level < 0)
		level = MMCLK_LEVEL_NONE;
	else if (level >= (int)ctx->step_size)
		level = ctx->step_size - 1;

	old_level = ctx->freq_level[idx];
	ctx->freq_level[idx] = level;

	/* The rail has to cover the highest request of all CRTCs. */
	vote_level = level;
	for (i = 0; i < CRTC_NUM; i++)
		if (ctx->freq_level[i] > vote_level)
			vote_level = ctx->freq_level[i];

	if (vote_level == ctx->voted_level)
		return;

	DDPINFO("%s: crtc%u level %d (was %d), rail level %d\n",
		caller, idx, level, old_level, vote_level);

	ret = mtk_disp_mmdvfs_vote(ctx, vote_level);
	if (ret) {
		/*
		 * Keep the cached requests equal to what the rail was left
		 * at, so a later request of the same level is programmed
		 * again instead of being taken for granted.
		 */
		ctx->freq_level[idx] = old_level;
		ctx->voted_level = MMCLK_LEVEL_INVALID;
		DDPPR_ERR("%s: crtc%u level %d was not applied: %d\n",
			  caller, idx, level, ret);
		return;
	}

	ctx->voted_level = vote_level;
}

int mtk_drm_mmdvfs_init(struct device *dev)
{
	struct mtk_disp_mmdvfs *ctx;
	int i, ret;

	if (!dev)
		return -EINVAL;

	mutex_lock(&disp_mmdvfs_lock);
	if (disp_mmdvfs) {
		/*
		 * The rail request has a single owner.  Initializing the same
		 * device again is a no-op; a second device would take the
		 * votes of the first one with it, so it is refused instead.
		 */
		if (disp_mmdvfs->dev == dev) {
			mutex_unlock(&disp_mmdvfs_lock);
			return 0;
		}
		dev_err(dev, "mmdvfs is already owned by %s\n",
			dev_name(disp_mmdvfs->dev));
		mutex_unlock(&disp_mmdvfs_lock);
		return -EBUSY;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mutex_unlock(&disp_mmdvfs_lock);
		return -ENOMEM;
	}

	ctx->dev = dev;
	ctx->voted_level = MMCLK_LEVEL_INVALID;
	for (i = 0; i < CRTC_NUM; i++)
		ctx->freq_level[i] = MMCLK_LEVEL_NONE;

	ret = mtk_disp_mmdvfs_probe(ctx);
	if (ret) {
		mtk_disp_mmdvfs_release(ctx);
		mutex_unlock(&disp_mmdvfs_lock);
		return ret;
	}

	disp_mmdvfs = ctx;
	mutex_unlock(&disp_mmdvfs_lock);

	return 0;
}

void mtk_drm_mmdvfs_exit(struct device *dev)
{
	struct mtk_disp_mmdvfs *ctx;

	if (!dev)
		return;

	mutex_lock(&disp_mmdvfs_lock);
	ctx = disp_mmdvfs;
	/*
	 * Only the owner gives up the rail request.  A device that failed to
	 * initialize, because another one already owns it, must not tear the
	 * live instance down on its way out of the bind.
	 */
	if (ctx && ctx->dev == dev) {
		disp_mmdvfs = NULL;
		mtk_disp_mmdvfs_release(ctx);
	}
	mutex_unlock(&disp_mmdvfs_lock);
}

unsigned int mtk_drm_get_mmclk_step_size(void)
{
	struct mtk_disp_mmdvfs *ctx;
	unsigned int step_size;

	mutex_lock(&disp_mmdvfs_lock);
	ctx = disp_mmdvfs;
	step_size = ctx ? ctx->step_size : 0;
	mutex_unlock(&disp_mmdvfs_lock);

	return step_size;
}

void mtk_drm_set_mmclk(struct drm_crtc *crtc, int level,
				const char *caller)
{
	struct mtk_disp_mmdvfs *ctx;

	if (!crtc)
		return;

	mutex_lock(&disp_mmdvfs_lock);
	ctx = disp_mmdvfs;
	if (ctx && ctx->freq_steps && ctx->step_volt && ctx->step_size)
		mtk_disp_mmdvfs_request(ctx, crtc, level, caller);
	mutex_unlock(&disp_mmdvfs_lock);
}

void mtk_drm_set_mmclk_by_pixclk(struct drm_crtc *crtc,
	unsigned int pixclk, const char *caller)
{
	struct mtk_disp_mmdvfs *ctx;
	/* The pixel clock is in MHz and the steps are in Hz. */
	u64 freq = (u64)pixclk * 1000000;
	unsigned int i;
	int level;

	if (!crtc)
		return;

	mutex_lock(&disp_mmdvfs_lock);
	ctx = disp_mmdvfs;
	if (!ctx || !ctx->freq_steps || !ctx->step_volt || !ctx->step_size) {
		mutex_unlock(&disp_mmdvfs_lock);
		return;
	}

	if (!freq) {
		/* No pixel clock: hold the lowest step of the table. */
		level = MMCLK_LEVEL_NONE;
	} else if (freq > ctx->freq_steps[ctx->step_size - 1]) {
		DDPMSG("%s: pixel clock %u MHz is above the top step (%u Hz)\n",
		       caller, pixclk, ctx->freq_steps[ctx->step_size - 1]);
		level = ctx->step_size - 1;
	} else {
		/* The lowest step that still covers the pixel clock. */
		level = ctx->step_size - 1;
		for (i = 0; i < ctx->step_size; i++) {
			if (freq <= ctx->freq_steps[i]) {
				level = i;
				break;
			}
		}
	}

	mtk_disp_mmdvfs_request(ctx, crtc, level, caller);
	mutex_unlock(&disp_mmdvfs_lock);
}
