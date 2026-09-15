// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 * Copyright (c) 2024 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/soc/mediatek/dvfsrc.h>

#include "mtk-dvfsrc-multimedia.h"

enum dvfsrc_regulator_id {
	DVFSRC_ID_VCORE,
	DVFSRC_ID_VSCP,
	DVFSRC_ID_MAX
};

struct dvfsrc_regulator_pdata {
	const struct regulator_desc *descs;
	u32 size;
};

#define MTK_DVFSRC_VREG(match, _name, _volt_table)	\
{							\
	.name = match,					\
	.of_match = match,				\
	.ops = &dvfsrc_vcore_ops,			\
	.type = REGULATOR_VOLTAGE,			\
	.id = DVFSRC_ID_##_name,			\
	.owner = THIS_MODULE,				\
	.n_voltages = ARRAY_SIZE(_volt_table),		\
	.volt_table = _volt_table,			\
}

static inline struct device *to_dvfs_regulator_dev(struct regulator_dev *rdev)
{
	return rdev_get_dev(rdev)->parent;
}

static inline struct device *to_dvfsrc_dev(struct regulator_dev *rdev)
{
	return to_dvfs_regulator_dev(rdev)->parent;
}

static int dvfsrc_get_cmd(int rdev_id, enum mtk_dvfsrc_cmd *cmd)
{
	switch (rdev_id) {
	case DVFSRC_ID_VCORE:
		*cmd = MTK_DVFSRC_CMD_VCORE_LEVEL;
		break;
	case DVFSRC_ID_VSCP:
		*cmd = MTK_DVFSRC_CMD_VSCP_LEVEL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int dvfsrc_set_voltage_sel(struct regulator_dev *rdev,
				  unsigned int selector)
{
	struct device *dvfsrc_dev = to_dvfsrc_dev(rdev);
	enum mtk_dvfsrc_cmd req_cmd;
	int id = rdev_get_id(rdev);
	int ret;

	ret = dvfsrc_get_cmd(id, &req_cmd);
	if (ret)
		return ret;

	return mtk_dvfsrc_send_request(dvfsrc_dev, req_cmd, selector);
}

static int dvfsrc_get_voltage_sel(struct regulator_dev *rdev)
{
	struct device *dvfsrc_dev = to_dvfsrc_dev(rdev);
	enum mtk_dvfsrc_cmd query_cmd;
	int id = rdev_get_id(rdev);
	int val, ret;

	ret = dvfsrc_get_cmd(id, &query_cmd);
	if (ret)
		return ret;

	ret = mtk_dvfsrc_query_info(dvfsrc_dev, query_cmd, &val);
	if (ret)
		return ret;

	return val;
}

static const struct regulator_ops dvfsrc_vcore_ops = {
	.list_voltage = regulator_list_voltage_table,
	.get_voltage_sel = dvfsrc_get_voltage_sel,
	.set_voltage_sel = dvfsrc_set_voltage_sel,
};

static const unsigned int mt6873_voltages[] = {
	575000,
	600000,
	650000,
	725000,
};

static const struct regulator_desc mt6873_regulators[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt6873_voltages),
	MTK_DVFSRC_VREG("dvfsrc-vscp", VSCP, mt6873_voltages),
};

static const struct dvfsrc_regulator_pdata mt6873_data = {
	.descs = mt6873_regulators,
	.size = ARRAY_SIZE(mt6873_regulators),
};

static const unsigned int mt6893_voltages[] = {
	575000,
	600000,
	650000,
	725000,
	750000,
};

static const struct regulator_desc mt6893_regulators[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt6893_voltages),
	MTK_DVFSRC_VREG("dvfsrc-vscp", VSCP, mt6893_voltages),
};

static const struct dvfsrc_regulator_pdata mt6893_data = {
	.descs = mt6893_regulators,
	.size = ARRAY_SIZE(mt6893_regulators),
};

static const unsigned int mt8183_voltages[] = {
	725000,
	800000,
};

static const struct regulator_desc mt8183_regulators[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt8183_voltages),
};

static const struct dvfsrc_regulator_pdata mt8183_data = {
	.descs = mt8183_regulators,
	.size = ARRAY_SIZE(mt8183_regulators),
};

static const unsigned int mt8195_voltages[] = {
	550000,
	600000,
	650000,
	750000,
};

static const struct regulator_desc mt8195_regulators[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt8195_voltages),
	MTK_DVFSRC_VREG("dvfsrc-vscp", VSCP, mt8195_voltages),
};

static const struct dvfsrc_regulator_pdata mt8195_data = {
	.descs = mt8195_regulators,
	.size = ARRAY_SIZE(mt8195_regulators),
};

static const unsigned int mt8196_voltages[] = {
	575000,
	600000,
	650000,
	725000,
	825000,
	875000,
};

static const struct regulator_desc mt8196_regulators[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt8196_voltages),
};

static const struct dvfsrc_regulator_pdata mt8196_data = {
	.descs = mt8196_regulators,
	.size = ARRAY_SIZE(mt8196_regulators),
};

/* MT6895 multimedia policy, ordered by increasing VCORE request. */
enum mt6895_mm_source {
	MM_U6D2, MM_U5D2, MM_M5, MM_U4, MM_M5D2, MM_M4D2,
	MM_MM4, MM_U4D2, MM_MM6, MM_U6, MM_TVD, MM_IMG2, MM_MM6D2,
	MM_SOURCE_COUNT,
};

static const char * const mt6895_mux_names[] = {
	"disp0", "disp1", "mminfra", "venc", "vdec", "mdp0", "mdp1",
};

static const char * const mt6895_source_names[] = {
	"univpll-d6-d2", "univpll-d5-d2", "mainpll-d5", "univpll-d4",
	"mainpll-d5-d2", "mainpll-d4-d2", "mmpll-d4", "univpll-d4-d2",
	"mmpll-d6", "univpll-d6", "tvdpll", "imgpll-d2", "mmpll-d6-d2",
};

static const unsigned long mt6895_source_rates[] = {
	208000000, 249600000, 436800000, 624000000, 218400000,
	273000000, 687500000, 312000000, 458333333, 416000000,
	594000000, 660000000, 229166667,
};

static const u8 mt6895_parents[][MTK_MM_STEPS] = {
	{ MM_U6D2, MM_U5D2, MM_M5, MM_U4, MM_U4 },
	{ MM_U6D2, MM_U5D2, MM_M5, MM_U4, MM_U4 },
	{ MM_M5D2, MM_M4D2, MM_M5, MM_U4, MM_MM4 },
	{ MM_U5D2, MM_U4D2, MM_MM6, MM_U4, MM_U4 },
	{ MM_M5D2, MM_U5D2, MM_U6, MM_TVD, MM_IMG2 },
	{ MM_MM6D2, MM_M4D2, MM_M5, MM_U4, MM_U4 },
	{ MM_MM6D2, MM_M4D2, MM_M5, MM_U4, MM_U4 },
};

struct mt6895_mm {
	struct device *dev;
	/* Serializes policy state and the voltage/clock transaction. */
	struct mutex lock;
	struct clk *mux[MTK_MM_MUXES];
	struct clk *source[MM_SOURCE_COUNT];
	struct mtk_mm_state state;
};

static int mt6895_mm_voltage(void *ctx, unsigned int step)
{
	struct mt6895_mm *mm = ctx;

	return mtk_dvfsrc_send_request(mm->dev->parent,
				      MTK_DVFSRC_CMD_VCORE_LEVEL, step);
}

static int mt6895_mm_parent(void *ctx, unsigned int mux, unsigned int step)
{
	struct mt6895_mm *mm = ctx;

	return clk_set_parent(mm->mux[mux],
			      mm->source[mt6895_parents[mux][step]]);
}

static const struct mtk_mm_ops mt6895_mm_ops = {
	.voltage = mt6895_mm_voltage,
	.parent = mt6895_mm_parent,
};

static int mt6895_mm_set_voltage_sel(struct regulator_dev *rdev,
				     unsigned int selector)
{
	struct mt6895_mm *mm = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&mm->lock);
	ret = mtk_mm_transition(&mm->state, &mt6895_mm_ops, mm, selector);
	if (ret)
		dev_err(mm->dev, "multimedia transition failed: %d; recovery: %d\n",
			ret, mm->state.recovery_error);
	mutex_unlock(&mm->lock);
	return ret;
}

static int mt6895_mm_get_voltage_sel(struct regulator_dev *rdev)
{
	struct mt6895_mm *mm = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&mm->lock);
	ret = mm->state.faulted ? -EIO : mm->state.step;
	mutex_unlock(&mm->lock);
	return ret;
}

static const struct regulator_ops mt6895_mm_regulator_ops = {
	.list_voltage = regulator_list_voltage_table,
	.get_voltage_sel = mt6895_mm_get_voltage_sel,
	.set_voltage_sel = mt6895_mm_set_voltage_sel,
};

static void mt6895_mm_release(void *data)
{
	struct mt6895_mm *mm = data;
	int ret;

	/* Retain a conservative request after consumers have been detached. */
	ret = mt6895_mm_voltage(mm, MTK_MM_STEPS - 1);
	if (ret)
		dev_err(mm->dev, "failed to retain handover voltage: %d\n", ret);
}

static void mt6895_mm_disable_clock(void *data)
{
	clk_disable_unprepare(data);
}

static int mt6895_mm_init(struct device *dev, struct mt6895_mm **result)
{
	struct mt6895_mm *mm;
	unsigned long rate, expected;
	int i, j, ret;

	mm = devm_kzalloc(dev, sizeof(*mm), GFP_KERNEL);
	if (!mm)
		return -ENOMEM;
	mm->dev = dev;
	mutex_init(&mm->lock);

	/* Freeze the policy's sources as well as its muxes against other writers. */
	for (i = 0; i < MM_SOURCE_COUNT; i++) {
		mm->source[i] = devm_clk_get(dev, mt6895_source_names[i]);
		if (IS_ERR(mm->source[i]))
			return PTR_ERR(mm->source[i]);
		ret = devm_clk_rate_exclusive_get(dev, mm->source[i]);
		if (ret)
			return ret;
		rate = clk_get_rate(mm->source[i]);
		expected = mt6895_source_rates[i];
		/* Allow integer PLL rounding, not a different firmware PLL policy. */
		if (rate < expected - 1000 || rate > expected + 1000) {
			dev_warn(dev,
				 "unexpected %s rate %lu (expected %lu), voltage only\n",
				 mt6895_source_names[i], rate, expected);
			return -EOPNOTSUPP;
		}
	}
	for (i = 0; i < ARRAY_SIZE(mm->mux); i++) {
		mm->mux[i] = devm_clk_get(dev, mt6895_mux_names[i]);
		if (IS_ERR(mm->mux[i]))
			return PTR_ERR(mm->mux[i]);
		ret = devm_clk_rate_exclusive_get(dev, mm->mux[i]);
		if (ret)
			return ret;
		for (j = 0; j < MTK_MM_STEPS; j++) {
			if (clk_has_parent(mm->mux[i],
					   mm->source[mt6895_parents[i][j]]))
				continue;
			dev_warn(dev, "%s cannot take %s, voltage only\n",
				 mt6895_mux_names[i],
				 mt6895_source_names[mt6895_parents[i][j]]);
			return -EOPNOTSUPP;
		}
		rate = clk_get_rate(mm->mux[i]);
		expected = mt6895_source_rates[mt6895_parents[i][MTK_MM_STEPS - 1]];
		if (!rate || rate > expected + 1000) {
			dev_warn(dev, "unsupported boot %s rate %lu, voltage only\n",
				 mt6895_mux_names[i], rate);
			return -EOPNOTSUPP;
		}
	}

	/*
	 * The parent seeds the highest request before starting DVFSRC. The
	 * confirmation poll can time out while another requester holds the rail,
	 * which is not a handover failure: the board floor already covers the
	 * boot rate of every mux below, so the handover still continues.
	 */
	ret = mt6895_mm_voltage(mm, MTK_MM_STEPS - 1);
	if (ret)
		dev_warn(dev, "VCORE handover request not confirmed: %d\n", ret);
	/*
	 * Keep bootloader display roots running across handover. Engine gates
	 * remain runtime managed; root gating needs separate coordination.
	 */
	for (i = 0; i < ARRAY_SIZE(mm->mux); i++) {
		ret = clk_prepare_enable(mm->mux[i]);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, mt6895_mm_disable_clock,
					       mm->mux[i]);
		if (ret)
			return ret;
	}
	/* Align every mux with the step the rail was just asked for. */
	for (i = 0; i < ARRAY_SIZE(mm->mux); i++) {
		ret = mt6895_mm_parent(mm, i, MTK_MM_STEPS - 1);
		if (ret)
			return dev_err_probe(dev, ret,
					     "cannot program %s for the handover step\n",
					     mt6895_mux_names[i]);
	}
	mm->state.step = MTK_MM_STEPS - 1;
	ret = devm_add_action_or_reset(dev, mt6895_mm_release, mm);
	if (ret)
		return ret;
	*result = mm;
	return 0;
}

static const unsigned int mt6895_voltages[] = {
	575000,
	600000,
	650000,
	725000,
	750000,
};

static const struct regulator_desc mt6895_regulators[] = {
	{
		.name = "dvfsrc-vcore",
		.of_match = "dvfsrc-vcore",
		.ops = &mt6895_mm_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.id = DVFSRC_ID_VCORE,
		.owner = THIS_MODULE,
		.n_voltages = ARRAY_SIZE(mt6895_voltages),
		.volt_table = mt6895_voltages,
	},
	MTK_DVFSRC_VREG("dvfsrc-vscp", VSCP, mt6895_voltages),
};

static const struct dvfsrc_regulator_pdata mt6895_data = {
	.descs = mt6895_regulators,
	.size = ARRAY_SIZE(mt6895_regulators),
};

/*
 * Used when the multimedia clocks cannot be handed over: the rail still
 * serves voltage requests, but this driver leaves the mux parents alone.
 */
static const struct regulator_desc mt6895_regulators_plain[] = {
	MTK_DVFSRC_VREG("dvfsrc-vcore", VCORE, mt6895_voltages),
	MTK_DVFSRC_VREG("dvfsrc-vscp", VSCP, mt6895_voltages),
};

static int dvfsrc_vcore_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = { .dev = &pdev->dev };
	const struct dvfsrc_regulator_pdata *pdata;
	const struct regulator_desc *descs;
	struct mt6895_mm *mm = NULL;
	int i, ret;

	pdata = device_get_match_data(&pdev->dev);
	if (!pdata)
		return -EINVAL;

	descs = pdata->descs;
	if (pdata == &mt6895_data) {
		ret = mt6895_mm_init(&pdev->dev, &mm);
		if (ret == -EOPNOTSUPP) {
			dev_warn(&pdev->dev,
				 "multimedia clock handover unavailable, voltage only\n");
			mm = NULL;
			descs = mt6895_regulators_plain;
		} else if (ret) {
			return dev_err_probe(&pdev->dev, ret,
					     "failed multimedia clock handover\n");
		} else {
			config.driver_data = mm;
		}
	}

	for (i = 0; i < pdata->size; i++) {
		const struct regulator_desc *vrdesc = &descs[i];
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(&pdev->dev, vrdesc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
					     "failed to register %s\n", vrdesc->name);
	}

	return 0;
}

static const struct of_device_id mtk_dvfsrc_regulator_match[] = {
	{ .compatible = "mediatek,mt6873-dvfsrc-regulator", .data = &mt6873_data },
	{ .compatible = "mediatek,mt6895-dvfsrc-regulator", .data = &mt6895_data },
	{ .compatible = "mediatek,mt6893-dvfsrc-regulator", .data = &mt6893_data },
	{ .compatible = "mediatek,mt8183-dvfsrc-regulator", .data = &mt8183_data },
	{ .compatible = "mediatek,mt8192-dvfsrc-regulator", .data = &mt6873_data },
	{ .compatible = "mediatek,mt8195-dvfsrc-regulator", .data = &mt8195_data },
	{ .compatible = "mediatek,mt8196-dvfsrc-regulator", .data = &mt8196_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_dvfsrc_regulator_match);

static struct platform_driver mtk_dvfsrc_regulator_driver = {
	.driver = {
		.name  = "mtk-dvfsrc-regulator",
		.of_match_table = mtk_dvfsrc_regulator_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = dvfsrc_vcore_regulator_probe,
};
module_platform_driver(mtk_dvfsrc_regulator_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_AUTHOR("Arvin wang <arvin.wang@mediatek.com>");
MODULE_DESCRIPTION("MediaTek DVFS Resource Collector Regulator driver");
MODULE_LICENSE("GPL");
