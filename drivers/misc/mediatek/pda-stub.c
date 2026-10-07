// SPDX-License-Identifier: GPL-2.0
/*
 * Stub driver for the MT6895 camera PDA (PDAF) block.
 *
 * The downstream driver (drivers/misc/mediatek/cameraisp/pda) is not ported
 * yet.  Without a driver the device never probes, its fw_devlink consumer
 * links stay unresolved and the scpsys power-controller sync_state stays
 * pending forever, which can trip the deferred-probe-timeout devlink walk
 * at ~13s.  Bind the device with this stub until the real driver is ported.
 */
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

static int camera_pda_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "camera-pda stub: probed\n");
	return 0;
}

static const struct of_device_id camera_pda_of_match[] = {
	{ .compatible = "mediatek,camera-pda" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, camera_pda_of_match);

static struct platform_driver camera_pda_driver = {
	.probe = camera_pda_probe,
	.driver = {
		.name = "camera-pda",
		.of_match_table = camera_pda_of_match,
	},
};
module_platform_driver(camera_pda_driver);
MODULE_LICENSE("GPL");
