// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 * Copyright (c) 2024 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/dvfsrc.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/spinlock.h>

/* DVFSRC_LEVEL */
#define DVFSRC_V1_LEVEL_TARGET_LEVEL	GENMASK(15, 0)
#define DVFSRC_TGT_LEVEL_IDLE		0x00
#define DVFSRC_V1_LEVEL_CURRENT_LEVEL	GENMASK(31, 16)

#define DVFSRC_V4_LEVEL_TARGET_LEVEL	GENMASK(15, 8)

/* Highest of the five MT6895 VCORE steps, used for the multimedia handover. */
#define DVFSRC_MT6895_VCORE_HANDOVER	4
#define DVFSRC_V4_LEVEL_TARGET_PRESENT	BIT(16)

/* DVFSRC_LEVEL on the MT6895 generation */
#define DVFSRC_MT6895_LEVEL_CURRENT	GENMASK(5, 0)

/* DVFSRC_SW_REQ, DVFSRC_SW_REQ2 */
#define DVFSRC_V1_SW_REQ2_DRAM_LEVEL	GENMASK(1, 0)
#define DVFSRC_V1_SW_REQ2_VCORE_LEVEL	GENMASK(3, 2)

#define DVFSRC_V4_SW_REQ_DRAM_LEVEL	GENMASK(15, 12)
#define DVFSRC_V2_SW_REQ_DRAM_LEVEL	GENMASK(3, 0)
#define DVFSRC_V2_SW_REQ_VCORE_LEVEL	GENMASK(6, 4)

/* DVFSRC_VCORE */
#define DVFSRC_V2_VCORE_REQ_VSCP_LEVEL	GENMASK(14, 12)

#define DVFSRC_POLL_TIMEOUT_US		1000
#define STARTUP_TIME_US			1

#define MTK_SIP_DVFSRC_INIT		0x0
#define MTK_SIP_DVFSRC_START		0x1

struct dvfsrc_bw_constraints {
	u16 max_dram_nom_bw;
	u16 max_dram_peak_bw;
	u16 max_dram_hrt_bw;
};

struct dvfsrc_opp {
	u32 vcore_opp;
	u32 dram_opp;
};

struct dvfsrc_opp_desc {
	const struct dvfsrc_opp *opps;
	u32 num_opp;
};

struct dvfsrc_soc_data;
struct mtk_dvfsrc {
	struct device *dev;
	struct platform_device *icc;
	struct platform_device *regulator;
	const struct dvfsrc_soc_data *dvd;
	const struct dvfsrc_opp_desc *curr_opps;
	void __iomem *regs;
	/* Protect read/modify/write of shared software request fields. */
	spinlock_t req_lock;
	int dram_type;
	struct clk *clk;
};

struct dvfsrc_soc_data {
	const int *regs;
	const struct dvfsrc_opp_desc *opps_desc;
	u32 num_opp_desc;
	u32 (*get_target_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_current_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_vcore_level)(struct mtk_dvfsrc *dvfsrc);
	u32 (*get_vscp_level)(struct mtk_dvfsrc *dvfsrc);
	void (*set_dram_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_dram_peak_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_dram_hrt_bw)(struct mtk_dvfsrc *dvfsrc, u64 bw);
	void (*set_opp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	void (*set_vcore_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	void (*set_vscp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	int (*wait_for_opp_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	int (*wait_for_vcore_level)(struct mtk_dvfsrc *dvfsrc, u32 level);
	const struct dvfsrc_bw_constraints *bw_constraints;
};

static u32 dvfsrc_readl(struct mtk_dvfsrc *dvfs, u32 offset)
{
	return readl(dvfs->regs + dvfs->dvd->regs[offset]);
}

static void dvfsrc_writel(struct mtk_dvfsrc *dvfs, u32 offset, u32 val)
{
	writel(val, dvfs->regs + dvfs->dvd->regs[offset]);
}

enum dvfsrc_regs {
	DVFSRC_SW_REQ,
	DVFSRC_SW_REQ2,
	DVFSRC_LEVEL,
	DVFSRC_TARGET_LEVEL,
	DVFSRC_SW_BW,
	DVFSRC_SW_PEAK_BW,
	DVFSRC_SW_HRT_BW,
	DVFSRC_VCORE,
	DVFSRC_REGS_MAX,
};

static const int dvfsrc_mt8183_regs[] = {
	[DVFSRC_SW_REQ] = 0x4,
	[DVFSRC_SW_REQ2] = 0x8,
	[DVFSRC_LEVEL] = 0xDC,
	[DVFSRC_SW_BW] = 0x160,
};

static const int dvfsrc_mt8195_regs[] = {
	[DVFSRC_SW_REQ] = 0xc,
	[DVFSRC_VCORE] = 0x6c,
	[DVFSRC_SW_PEAK_BW] = 0x278,
	[DVFSRC_SW_BW] = 0x26c,
	[DVFSRC_SW_HRT_BW] = 0x290,
	[DVFSRC_LEVEL] = 0xd44,
	[DVFSRC_TARGET_LEVEL] = 0xd48,
};

/*
 * MT6895 (and the MT6983 family it belongs to) keeps the DRAM level in
 * SW_REQ[15:12], the VCORE_SW request in SW_REQ[6:4] and the VSCP request
 * in VCORE_REQUEST[14:12].  Unlike the MT8196, this generation has no gear
 * tables in the MCU: the OPP combinations are described in software.
 */
static const int dvfsrc_mt6895_regs[] = {
	[DVFSRC_SW_REQ] = 0x18,
	[DVFSRC_VCORE] = 0x80,
	[DVFSRC_SW_BW] = 0x1e8,
	[DVFSRC_SW_PEAK_BW] = 0x1f4,
	[DVFSRC_SW_HRT_BW] = 0x20c,
	[DVFSRC_LEVEL] = 0x5f0,
	[DVFSRC_TARGET_LEVEL] = 0x5f0,
};

static const struct dvfsrc_opp *dvfsrc_get_current_opp(struct mtk_dvfsrc *dvfsrc)
{
	u32 level = dvfsrc->dvd->get_current_level(dvfsrc);

	return &dvfsrc->curr_opps->opps[level];
}

static bool dvfsrc_is_idle(struct mtk_dvfsrc *dvfsrc)
{
	if (!dvfsrc->dvd->get_target_level)
		return true;

	return dvfsrc->dvd->get_target_level(dvfsrc) == DVFSRC_TGT_LEVEL_IDLE;
}

static int dvfsrc_wait_for_vcore_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *curr;

	return readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					 curr->vcore_opp >= level, STARTUP_TIME_US,
					 DVFSRC_POLL_TIMEOUT_US);
}

static int dvfsrc_wait_for_opp_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *target, *curr;
	int ret;

	target = &dvfsrc->curr_opps->opps[level];
	ret = readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					curr->dram_opp >= target->dram_opp &&
					curr->vcore_opp >= target->vcore_opp,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "timeout! target OPP: %u, dram: %d, vcore: %d\n", level,
			 curr->dram_opp, curr->vcore_opp);
		return ret;
	}

	return 0;
}

static int dvfsrc_wait_for_opp_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *target, *curr;
	int ret;

	target = &dvfsrc->curr_opps->opps[level];
	ret = readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					curr->dram_opp >= target->dram_opp &&
					curr->vcore_opp >= target->vcore_opp,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "timeout! target OPP: %u, dram: %d\n", level, curr->dram_opp);
		return ret;
	}

	return 0;
}

static u32 dvfsrc_get_target_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);

	return FIELD_GET(DVFSRC_V1_LEVEL_TARGET_LEVEL, val);
}

static u32 dvfsrc_get_current_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);
	u32 current_level = FIELD_GET(DVFSRC_V1_LEVEL_CURRENT_LEVEL, val);

	return ffs(current_level) - 1;
}

static u32 dvfsrc_get_target_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	return dvfsrc_readl(dvfsrc, DVFSRC_TARGET_LEVEL);
}

static u32 dvfsrc_get_current_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);
	u32 level = ffs(val);

	/* Valid levels */
	if (level < dvfsrc->curr_opps->num_opp)
		return dvfsrc->curr_opps->num_opp - level;

	/* Zero for level 0 or invalid level */
	return 0;
}

static u32 dvfsrc_get_vcore_level_v1(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ2);

	return FIELD_GET(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, val);
}

static void dvfsrc_set_vcore_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ2);

	val &= ~DVFSRC_V1_SW_REQ2_VCORE_LEVEL;
	val |= FIELD_PREP(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ2, val);
}

static u32 dvfsrc_get_vcore_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	return FIELD_GET(DVFSRC_V2_SW_REQ_VCORE_LEVEL, val);
}

static void dvfsrc_set_vcore_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&dvfsrc->req_lock, flags);
	val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	val &= ~DVFSRC_V2_SW_REQ_VCORE_LEVEL;
	val |= FIELD_PREP(DVFSRC_V2_SW_REQ_VCORE_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
	spin_unlock_irqrestore(&dvfsrc->req_lock, flags);
}

static u32 dvfsrc_get_vscp_level_v2(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	return FIELD_GET(DVFSRC_V2_VCORE_REQ_VSCP_LEVEL, val);
}

static void dvfsrc_set_vscp_level_v2(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_VCORE);

	val &= ~DVFSRC_V2_VCORE_REQ_VSCP_LEVEL;
	val |= FIELD_PREP(DVFSRC_V2_VCORE_REQ_VSCP_LEVEL, level);

	dvfsrc_writel(dvfsrc, DVFSRC_VCORE, val);
}

static void __dvfsrc_set_dram_bw_v1(struct mtk_dvfsrc *dvfsrc, u32 reg,
				    u16 max_bw, u16 min_bw, u64 bw)
{
	u32 new_bw = (u32)div_u64(bw, 100 * 1000);

	/* If bw constraints (in mbps) are defined make sure to respect them */
	if (max_bw)
		new_bw = min(new_bw, max_bw);
	if (min_bw && new_bw > 0)
		new_bw = max(new_bw, min_bw);

	dvfsrc_writel(dvfsrc, reg, new_bw);
}

static void dvfsrc_set_dram_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	u64 max_bw = dvfsrc->dvd->bw_constraints->max_dram_nom_bw;

	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_BW, max_bw, 0, bw);
};

static void dvfsrc_set_dram_peak_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	u64 max_bw = dvfsrc->dvd->bw_constraints->max_dram_peak_bw;

	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_PEAK_BW, max_bw, 0, bw);
}

static void dvfsrc_set_dram_hrt_bw_v1(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	u64 max_bw = dvfsrc->dvd->bw_constraints->max_dram_hrt_bw;

	__dvfsrc_set_dram_bw_v1(dvfsrc, DVFSRC_SW_HRT_BW, max_bw, 0, bw);
}

static void dvfsrc_set_opp_level_v1(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *opp = &dvfsrc->curr_opps->opps[level];
	u32 val;

	/* Translate Pstate to DVFSRC level and set it to DVFSRC HW */
	val = FIELD_PREP(DVFSRC_V1_SW_REQ2_DRAM_LEVEL, opp->dram_opp);
	val |= FIELD_PREP(DVFSRC_V1_SW_REQ2_VCORE_LEVEL, opp->vcore_opp);

	dev_dbg(dvfsrc->dev, "vcore_opp: %d, dram_opp: %d\n", opp->vcore_opp, opp->dram_opp);
	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
}

static u32 dvfsrc_get_target_level_v4(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_TARGET_LEVEL);

	if (val & DVFSRC_V4_LEVEL_TARGET_PRESENT)
		return FIELD_GET(DVFSRC_V4_LEVEL_TARGET_LEVEL, val) + 1;
	return 0;
}

static void dvfsrc_set_dram_level_v4(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	val &= ~DVFSRC_V4_SW_REQ_DRAM_LEVEL;
	val |= FIELD_PREP(DVFSRC_V4_SW_REQ_DRAM_LEVEL, level);

	dev_dbg(dvfsrc->dev, "%s level=%u\n", __func__, level);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
}

static u32 dvfsrc_get_current_level_mt6895(struct mtk_dvfsrc *dvfsrc)
{
	u32 val = dvfsrc_readl(dvfsrc, DVFSRC_LEVEL);
	u32 level = FIELD_GET(DVFSRC_MT6895_LEVEL_CURRENT, val) + 1;

	/* The hardware counts down from the highest level. */
	if (level >= dvfsrc->curr_opps->num_opp)
		return 0;

	return dvfsrc->curr_opps->num_opp - level;
}

static void dvfsrc_set_dram_bw_mt6895(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	bw = min_t(u64, div_u64(bw, 1000 * 100), 0x3ff);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_BW, bw);
}

static void dvfsrc_set_dram_peak_bw_mt6895(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	bw = min_t(u64, div_u64(bw, 1000 * 100), 0x3ff);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_PEAK_BW, bw);
}

static void dvfsrc_set_dram_hrt_bw_mt6895(struct mtk_dvfsrc *dvfsrc, u64 bw)
{
	bw = min_t(u64, div_u64(div_u64(bw, 1000) + 29, 30), 0x3ff);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_HRT_BW, bw);
}

static void dvfsrc_set_opp_level_mt6895(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *opp = &dvfsrc->curr_opps->opps[level];
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&dvfsrc->req_lock, flags);
	val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);

	val &= ~DVFSRC_V4_SW_REQ_DRAM_LEVEL;
	val |= FIELD_PREP(DVFSRC_V4_SW_REQ_DRAM_LEVEL, opp->dram_opp);

	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
	spin_unlock_irqrestore(&dvfsrc->req_lock, flags);
}

static int dvfsrc_wait_for_opp_level_mt6895(struct mtk_dvfsrc *dvfsrc, u32 level)
{
	const struct dvfsrc_opp *target, *curr;

	target = &dvfsrc->curr_opps->opps[level];

	return readx_poll_timeout_atomic(dvfsrc_get_current_opp, dvfsrc, curr,
					 curr->dram_opp >= target->dram_opp,
					 STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
}

int mtk_dvfsrc_send_request(const struct device *dev, u32 cmd, u64 data)
{
	struct mtk_dvfsrc *dvfsrc = dev_get_drvdata(dev);
	bool state;
	int ret;

	dev_dbg(dvfsrc->dev, "cmd: %d, data: %llu\n", cmd, data);

	switch (cmd) {
	case MTK_DVFSRC_CMD_BW:
		dvfsrc->dvd->set_dram_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_HRT_BW:
		if (dvfsrc->dvd->set_dram_hrt_bw)
			dvfsrc->dvd->set_dram_hrt_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_PEAK_BW:
		if (dvfsrc->dvd->set_dram_peak_bw)
			dvfsrc->dvd->set_dram_peak_bw(dvfsrc, data);
		return 0;
	case MTK_DVFSRC_CMD_OPP:
		if (!dvfsrc->dvd->set_opp_level)
			return 0;

		dvfsrc->dvd->set_opp_level(dvfsrc, data);
		break;
	case MTK_DVFSRC_CMD_VCORE_LEVEL:
		dvfsrc->dvd->set_vcore_level(dvfsrc, data);
		break;
	case MTK_DVFSRC_CMD_VSCP_LEVEL:
		if (!dvfsrc->dvd->set_vscp_level)
			return 0;

		dvfsrc->dvd->set_vscp_level(dvfsrc, data);
		break;
	default:
		dev_err(dvfsrc->dev, "unknown command: %d\n", cmd);
		return -EOPNOTSUPP;
	}

	/* DVFSRC needs at least 2T(~196ns) to handle a request */
	udelay(STARTUP_TIME_US);

	ret = readx_poll_timeout_atomic(dvfsrc_is_idle, dvfsrc, state, state,
					STARTUP_TIME_US, DVFSRC_POLL_TIMEOUT_US);
	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "%d: idle timeout, data: %llu, last: %d -> %d\n", cmd, data,
			 dvfsrc->dvd->get_current_level(dvfsrc),
			 dvfsrc->dvd->get_target_level(dvfsrc));
		return ret;
	}

	if (cmd == MTK_DVFSRC_CMD_OPP)
		ret = dvfsrc->dvd->wait_for_opp_level(dvfsrc, data);
	else
		ret = dvfsrc->dvd->wait_for_vcore_level(dvfsrc, data);

	if (ret < 0) {
		dev_warn(dvfsrc->dev,
			 "%d: wait timeout, data: %llu, last: %d -> %d\n",
			 cmd, data,
			 dvfsrc->dvd->get_current_level(dvfsrc),
			 dvfsrc->dvd->get_target_level(dvfsrc));
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_dvfsrc_send_request);

int mtk_dvfsrc_query_info(const struct device *dev, u32 cmd, int *data)
{
	struct mtk_dvfsrc *dvfsrc = dev_get_drvdata(dev);

	switch (cmd) {
	case MTK_DVFSRC_CMD_VCORE_LEVEL:
		*data = dvfsrc->dvd->get_vcore_level(dvfsrc);
		break;
	case MTK_DVFSRC_CMD_VSCP_LEVEL:
		*data = dvfsrc->dvd->get_vscp_level(dvfsrc);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}
EXPORT_SYMBOL(mtk_dvfsrc_query_info);

/* XAGA: instance whose DRAM SW_REQ floor is coupled to the GPU (see below). */
static struct mtk_dvfsrc *mtk_dvfsrc_mt6895;

static int mtk_dvfsrc_probe(struct platform_device *pdev)
{
	struct arm_smccc_res ares;
	struct mtk_dvfsrc *dvfsrc;
	int ret;

	dvfsrc = devm_kzalloc(&pdev->dev, sizeof(*dvfsrc), GFP_KERNEL);
	if (!dvfsrc)
		return -ENOMEM;

	dvfsrc->dvd = of_device_get_match_data(&pdev->dev);
	if (!dvfsrc->dvd)
		return dev_err_probe(&pdev->dev, -ENODEV, "missing platform data\n");

	/*
	 * A platform either has firmware-published gear tables or describes
	 * its operating points in software.  Check that here, so that a new
	 * SoC cannot come up with no table at all or - the failure mode this
	 * replaces - with a pointer but a zero count, which would send every
	 * DRAM type, valid ones included, to the first table.
	 */
	if (!dvfsrc->dvd->opps_desc || !dvfsrc->dvd->num_opp_desc)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "no DVFSRC operating points described\n");

	dvfsrc->dev = &pdev->dev;
	spin_lock_init(&dvfsrc->req_lock);

	dvfsrc->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(dvfsrc->regs))
		return PTR_ERR(dvfsrc->regs);

	/* Some SoCs gate the DVFSRC MCU clock outside of the AP clock tree. */
	dvfsrc->clk = devm_clk_get_optional_enabled(&pdev->dev, NULL);
	if (IS_ERR(dvfsrc->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(dvfsrc->clk),
				     "Couldn't get and enable DVFSRC clock\n");

	arm_smccc_smc(MTK_SIP_DVFSRC_VCOREFS_CONTROL, MTK_SIP_DVFSRC_INIT,
		      0, 0, 0, 0, 0, 0, &ares);
	if (ares.a0)
		return dev_err_probe(&pdev->dev, -EINVAL, "DVFSRC init failed: %lu\n", ares.a0);

	dvfsrc->dram_type = ares.a1;
	dev_dbg(&pdev->dev, "DRAM Type: %d\n", dvfsrc->dram_type);

	u32 dram_type = dvfsrc->dram_type;

	if (dram_type >= dvfsrc->dvd->num_opp_desc) {
		dev_warn(&pdev->dev, "unknown DRAM type %u, using first OPP table\n", dram_type);
		dram_type = 0;
	}
	dvfsrc->curr_opps = &dvfsrc->dvd->opps_desc[dram_type];
	platform_set_drvdata(pdev, dvfsrc);

	/*
	 * Seed the highest multimedia request before starting the collector, so
	 * the rail cannot be lowered under the bootloader clock rates while the
	 * regulator provider hands those clocks over to its own policy.
	 */
	if (of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6895-dvfsrc"))
		dvfsrc->dvd->set_vcore_level(dvfsrc,
					     DVFSRC_MT6895_VCORE_HANDOVER);

	/* Everything is set up - make it run! */
	arm_smccc_smc(MTK_SIP_DVFSRC_VCOREFS_CONTROL, MTK_SIP_DVFSRC_START,
		      0, 0, 0, 0, 0, 0, &ares);
	if (ares.a0)
		return dev_err_probe(&pdev->dev, -EINVAL, "Cannot start DVFSRC: %lu\n", ares.a0);
	/*
	 * Re-submit the seeded request through the normal path so the collector
	 * records it as an AP request. The poll is only a confirmation: another
	 * requester holding the rail, or a gear that does not report this step
	 * yet, would otherwise abort the whole provider and leave the display
	 * and both codecs without their supply.
	 */
	if (of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6895-dvfsrc")) {
		ret = mtk_dvfsrc_send_request(&pdev->dev,
					      MTK_DVFSRC_CMD_VCORE_LEVEL,
					      DVFSRC_MT6895_VCORE_HANDOVER);
		if (ret)
			dev_warn(&pdev->dev,
				 "multimedia handover voltage not confirmed: %d\n", ret);
	}

	/* Child providers may immediately submit synchronous voltage requests. */
	ret = devm_of_platform_populate(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to populate child devices\n");

	if (of_device_is_compatible(pdev->dev.of_node, "mediatek,mt6895-dvfsrc"))
		mtk_dvfsrc_mt6895 = dvfsrc;

	return 0;
}

static const struct dvfsrc_bw_constraints dvfsrc_bw_constr_v1 = { 0, 0, 0 };
static const struct dvfsrc_bw_constraints dvfsrc_bw_constr_v2 = {
	.max_dram_nom_bw = 255,
	.max_dram_peak_bw = 255,
	.max_dram_hrt_bw = 1023,
};

/*
 * Publish a software operating point table together with its size.  The
 * two always go together: the bound check in mtk_dvfsrc_probe() indexes
 * @opps_desc with the DRAM type reported by the firmware, so a platform
 * that describes its tables without saying how many there are would have
 * every request, including the valid ones, fall back to the first table.
 */
#define DVFSRC_SW_OPPS(_name)		\
	.opps_desc = _name,		\
	.num_opp_desc = ARRAY_SIZE(_name)

static const struct dvfsrc_opp dvfsrc_opp_mt6893_lp4[] = {
	{ 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 },
	{ 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 },
	{ 0, 2 }, { 1, 2 }, { 2, 2 }, { 3, 2 },
	{ 0, 3 }, { 1, 3 }, { 2, 3 }, { 3, 3 },
	{ 1, 4 }, { 2, 4 }, { 3, 4 }, { 2, 5 },
	{ 3, 5 }, { 3, 6 }, { 4, 6 }, { 4, 7 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt6893_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt6893_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt6893_lp4),
	}
};

static const struct dvfsrc_soc_data mt6893_data = {
	DVFSRC_SW_OPPS(dvfsrc_opp_mt6893_desc),
	.regs = dvfsrc_mt8195_regs,
	.get_target_level = dvfsrc_get_target_level_v2,
	.get_current_level = dvfsrc_get_current_level_v2,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v2,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_constraints = &dvfsrc_bw_constr_v2,
};

static const struct dvfsrc_opp dvfsrc_opp_mt8183_lp4[] = {
	{ 0, 0 }, { 0, 1 }, { 0, 2 }, { 1, 2 },
};

static const struct dvfsrc_opp dvfsrc_opp_mt8183_lp3[] = {
	{ 0, 0 }, { 0, 1 }, { 1, 1 }, { 1, 2 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt8183_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt8183_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp4),
	},
	[1] = {
		.opps = dvfsrc_opp_mt8183_lp3,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp3),
	},
	[2] = {
		.opps = dvfsrc_opp_mt8183_lp3,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8183_lp3),
	}
};

static const struct dvfsrc_soc_data mt8183_data = {
	DVFSRC_SW_OPPS(dvfsrc_opp_mt8183_desc),
	.regs = dvfsrc_mt8183_regs,
	.get_target_level = dvfsrc_get_target_level_v1,
	.get_current_level = dvfsrc_get_current_level_v1,
	.get_vcore_level = dvfsrc_get_vcore_level_v1,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_opp_level = dvfsrc_set_opp_level_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v1,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v1,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_constraints = &dvfsrc_bw_constr_v1,
};

static const struct dvfsrc_opp dvfsrc_opp_mt8195_lp4[] = {
	{ 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 },
	{ 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 },
	{ 0, 2 }, { 1, 2 }, { 2, 2 }, { 3, 2 },
	{ 1, 3 }, { 2, 3 }, { 3, 3 }, { 1, 4 },
	{ 2, 4 }, { 3, 4 }, { 2, 5 }, { 3, 5 },
	{ 3, 6 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt8195_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt8195_lp4,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt8195_lp4),
	}
};

static const struct dvfsrc_soc_data mt8195_data = {
	DVFSRC_SW_OPPS(dvfsrc_opp_mt8195_desc),
	.regs = dvfsrc_mt8195_regs,
	.get_target_level = dvfsrc_get_target_level_v2,
	.get_current_level = dvfsrc_get_current_level_v2,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.set_dram_bw = dvfsrc_set_dram_bw_v1,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_v1,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_v1,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_v2,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
	.bw_constraints = &dvfsrc_bw_constr_v2,
};

/*
 * Software OPP combinations for the MT6983/MT6895 generation, ordered from
 * the lowest to the highest request.  Each entry pairs a vcore level with a
 * DRAM level; the hardware serves the highest level requested by any client.
 */
static const struct dvfsrc_opp dvfsrc_opp_mt6895[] = {
	{ 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 }, { 4, 0 },
	{ 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 }, { 4, 1 },
	{ 1, 2 }, { 2, 2 }, { 3, 2 }, { 4, 2 },
	{ 1, 3 }, { 2, 3 }, { 3, 3 }, { 4, 3 },
	{ 2, 4 }, { 3, 4 }, { 4, 4 },
	{ 3, 5 }, { 4, 5 },
	{ 3, 6 }, { 4, 6 },
	{ 4, 7 },
	{ 4, 8 },
};

static const struct dvfsrc_opp_desc dvfsrc_opp_mt6895_desc[] = {
	[0] = {
		.opps = dvfsrc_opp_mt6895,
		.num_opp = ARRAY_SIZE(dvfsrc_opp_mt6895),
	}
};

static const struct dvfsrc_soc_data mt6895_data = {
	DVFSRC_SW_OPPS(dvfsrc_opp_mt6895_desc),
	.regs = dvfsrc_mt6895_regs,
	.get_target_level = dvfsrc_get_target_level_v4,
	.get_current_level = dvfsrc_get_current_level_mt6895,
	.get_vcore_level = dvfsrc_get_vcore_level_v2,
	.get_vscp_level = dvfsrc_get_vscp_level_v2,
	.set_dram_bw = dvfsrc_set_dram_bw_mt6895,
	.set_dram_peak_bw = dvfsrc_set_dram_peak_bw_mt6895,
	.set_dram_hrt_bw = dvfsrc_set_dram_hrt_bw_mt6895,
	.set_opp_level = dvfsrc_set_opp_level_mt6895,
	.set_vcore_level = dvfsrc_set_vcore_level_v2,
	.set_vscp_level = dvfsrc_set_vscp_level_v2,
	.wait_for_opp_level = dvfsrc_wait_for_opp_level_mt6895,
	.wait_for_vcore_level = dvfsrc_wait_for_vcore_level_v1,
};

static const struct of_device_id mtk_dvfsrc_of_match[] = {
	{ .compatible = "mediatek,mt6893-dvfsrc", .data = &mt6893_data },
	{ .compatible = "mediatek,mt8183-dvfsrc", .data = &mt8183_data },
	{ .compatible = "mediatek,mt6895-dvfsrc", .data = &mt6895_data },
	{ .compatible = "mediatek,mt8195-dvfsrc", .data = &mt8195_data },
	{ /* sentinel */ }
};

static struct platform_driver mtk_dvfsrc_driver = {
	.probe	= mtk_dvfsrc_probe,
	.driver = {
		.name = "mtk-dvfsrc",
		.of_match_table = mtk_dvfsrc_of_match,
	},
};
module_platform_driver(mtk_dvfsrc_driver);

/*
 * XAGA: GPU -> DRAM floor coupling.
 *
 * The DVFSRC hardware bandwidth voter is traffic-based; under a GPU load it
 * thrashes between levels and starves the GPU.  Android supplements it with
 * sustained software votes (ged/gpufreq/mmqos); do the same here by holding a
 * DRAM floor derived from the GPU frequency.  SW_REQ[15:12] is a floor, not a
 * pin: the hardware voters still lift DRAM above it on demand.
 *
 * This logic used to live in drivers/memory/mediatek/dvfsrc-pin.c, removed
 * when the generic MT6895 DVFSRC provider landed.  The interconnect provider
 * only writes SW_BW/SW_PEAK_BW/SW_HRT_BW, so SW_REQ stays free for this floor
 * and the two do not fight.
 */
struct mtk_dvfsrc_gpu_map {
	unsigned long min_freq;	/* inclusive */
	u32 dram_opp;		/* raw SW_REQ[15:12] value */
};

static const struct mtk_dvfsrc_gpu_map mtk_dvfsrc_gpu_map[] = {
	{ 600000000, 8 },	/* 6400 Mbps */
	{ 400000000, 6 },	/* 5500 Mbps */
	{ 0,         0 },	/* idle -> 800 Mbps floor */
};

static u32 mtk_dvfsrc_gpu_to_dram(unsigned long freq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_dvfsrc_gpu_map); i++)
		if (freq >= mtk_dvfsrc_gpu_map[i].min_freq)
			return mtk_dvfsrc_gpu_map[i].dram_opp;

	return 0;
}

static void mtk_dvfsrc_set_dram_floor(struct mtk_dvfsrc *dvfsrc, u32 dram_opp)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&dvfsrc->req_lock, flags);
	val = dvfsrc_readl(dvfsrc, DVFSRC_SW_REQ);
	val &= ~DVFSRC_V4_SW_REQ_DRAM_LEVEL;
	val |= FIELD_PREP(DVFSRC_V4_SW_REQ_DRAM_LEVEL, dram_opp);
	dvfsrc_writel(dvfsrc, DVFSRC_SW_REQ, val);
	spin_unlock_irqrestore(&dvfsrc->req_lock, flags);
}

static int mtk_dvfsrc_gpu_notify(struct notifier_block *nb,
				 unsigned long event, void *ptr)
{
	struct devfreq_freqs *freqs = ptr;

	if (event != DEVFREQ_POSTCHANGE || !mtk_dvfsrc_mt6895)
		return NOTIFY_DONE;

	mtk_dvfsrc_set_dram_floor(mtk_dvfsrc_mt6895,
				  mtk_dvfsrc_gpu_to_dram(freqs->new));
	return NOTIFY_DONE;
}

static struct notifier_block mtk_dvfsrc_gpu_nb = {
	.notifier_call = mtk_dvfsrc_gpu_notify,
};

static int __init mtk_dvfsrc_couple_gpu(void)
{
	struct device_node *np;
	struct devfreq *gpu;
	int ret;

	if (!mtk_dvfsrc_mt6895)
		return 0;

	np = of_find_compatible_node(NULL, NULL, "arm,mali-valhall-csf");
	if (!np) {
		pr_warn("mtk-dvfsrc: no GPU node, DRAM floor stays static\n");
		return 0;
	}

	gpu = devfreq_get_devfreq_by_node(np);
	of_node_put(np);
	if (IS_ERR_OR_NULL(gpu)) {
		pr_warn("mtk-dvfsrc: no GPU devfreq, DRAM floor stays static\n");
		return 0;
	}

	ret = devfreq_register_notifier(gpu, &mtk_dvfsrc_gpu_nb,
					DEVFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		pr_warn("mtk-dvfsrc: GPU notifier registration failed %d\n", ret);
		return 0;
	}

	mtk_dvfsrc_set_dram_floor(mtk_dvfsrc_mt6895,
				  mtk_dvfsrc_gpu_to_dram(gpu->previous_freq));

	pr_info("mtk-dvfsrc: GPU-coupled DRAM floor active (gpu=%lu Hz)\n",
		gpu->previous_freq);
	return 0;
}
late_initcall_sync(mtk_dvfsrc_couple_gpu);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_AUTHOR("Dawei Chien <dawei.chien@mediatek.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek DVFSRC driver");
