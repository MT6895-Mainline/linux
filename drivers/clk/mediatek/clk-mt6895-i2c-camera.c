// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6895 camera I2C gates in the south I2C wrapper.
 *
 * The existing I2C1/5/6/7 drivers use fixed clock placeholders while setup.c
 * keeps their common clocks enabled. Only the gates of the camera buses I2C2,
 * I2C4, I2C8 and I2C9 are managed here: disabling an untracked shared i2c_sel
 * or DMA clock would break the other buses.
 * The parent is therefore the existing fixed 26 MHz clock until all I2C
 * consumers can be migrated together. The wrapper bits are independent.
 *
 * Register layout is shared with the stock MT6895 south I2C wrapper:
 * status 0xe00, enable/clear 0xe04, disable/set 0xe08.
 */

#include <dt-bindings/clock/mt6895-clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs camera_i2c_regs = {
	.sta_ofs = 0xe00,
	.clr_ofs = 0xe04,
	.set_ofs = 0xe08,
};

static const struct mtk_gate camera_i2c_gates[] = {
	GATE_MTK(CLK_IMPS_AP_CLOCK_I2C2, "imps_i2c2", "i2c5-main",
		 &camera_i2c_regs, 1, &mtk_clk_gate_ops_setclr),
	GATE_MTK(CLK_IMPS_AP_CLOCK_I2C4, "imps_i2c4", "i2c5-main",
		 &camera_i2c_regs, 3, &mtk_clk_gate_ops_setclr),
	GATE_MTK(CLK_IMPS_AP_CLOCK_I2C8, "imps_i2c8", "i2c5-main",
		 &camera_i2c_regs, 5, &mtk_clk_gate_ops_setclr),
	GATE_MTK(CLK_IMPS_AP_CLOCK_I2C9, "imps_i2c9", "i2c5-main",
		 &camera_i2c_regs, 6, &mtk_clk_gate_ops_setclr),
};

static int mt6895_camera_i2c_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_hw_onecell_data *data;
	int ret;

	/* Keep the stock IDs (1, 3, 5 and 6) and the unregistered holes. */
	data = mtk_alloc_clk_data(CLK_IMPS_NR_CLK);
	if (!data)
		return -ENOMEM;

	ret = mtk_clk_register_gates(&pdev->dev, node, camera_i2c_gates,
				     ARRAY_SIZE(camera_i2c_gates), data);
	if (ret)
		goto free_data;

	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, data);
	if (ret)
		goto unregister_gates;

	platform_set_drvdata(pdev, data);
	return 0;

unregister_gates:
	mtk_clk_unregister_gates(camera_i2c_gates,
				 ARRAY_SIZE(camera_i2c_gates), data);
free_data:
	mtk_free_clk_data(data);
	return ret;
}

static void mt6895_camera_i2c_remove(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *data = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	mtk_clk_unregister_gates(camera_i2c_gates,
				 ARRAY_SIZE(camera_i2c_gates), data);
	mtk_free_clk_data(data);
}

static const struct of_device_id mt6895_camera_i2c_match[] = {
	{ .compatible = "mediatek,mt6895-imp_iic_wrap_s" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6895_camera_i2c_match);

static struct platform_driver mt6895_camera_i2c_driver = {
	.probe = mt6895_camera_i2c_probe,
	.remove = mt6895_camera_i2c_remove,
	.driver = {
		.name = "mt6895-camera-i2c-clocks",
		.of_match_table = mt6895_camera_i2c_match,
	},
};
module_platform_driver(mt6895_camera_i2c_driver);

MODULE_DESCRIPTION("MT6895 camera I2C wrapper gates");
MODULE_LICENSE("GPL");
