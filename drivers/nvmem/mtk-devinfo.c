// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek bootloader "devinfo" NVMEM provider.
 *
 * The bootloader hands the kernel an efuse shadow table in the /chosen
 * "atag,devinfo" property: a word count followed by that many 32-bit words,
 * indexed by the byte offset declared in the provider's child nodes.
 *
 * This is the authoritative source for the LVTS thermal sensor calibration on
 * MT6895: the raw efuse window at the same offsets returns different data, and
 * zeroes where the per-sensor counts should be.
 *
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Mac Lu <mac.lu@mediatek.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Enough for every efuse word the bootloader has ever been seen to pass. */
#define MTK_DEVINFO_MAX_WORDS	400

/* atag,devinfo payload: a word count followed by that many 32-bit words. */
struct mtk_devinfo_tag {
	u32 size;
	u32 data[];
};

struct mtk_devinfo {
	u32 size;
	u32 data[];
};

static int mtk_devinfo_read(void *context, unsigned int offset, void *val,
			    size_t bytes)
{
	struct mtk_devinfo *priv = context;
	unsigned int index = offset / sizeof(u32);
	unsigned int words = bytes / sizeof(u32);
	u32 *out = val;
	unsigned int i;

	if (offset % sizeof(u32) || bytes % sizeof(u32))
		return -EINVAL;
	if (index > priv->size || words > priv->size - index)
		return -EINVAL;

	for (i = 0; i < words; i++)
		out[i] = priv->data[index + i];

	return 0;
}

static int mtk_devinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *chosen;
	struct mtk_devinfo_tag *tag;
	struct nvmem_config config = {};
	struct mtk_devinfo *priv;
	int len;
	u32 size;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		chosen = of_find_node_by_path("/chosen@0");
	if (!chosen)
		return dev_err_probe(dev, -ENXIO, "no /chosen node\n");

	tag = (struct mtk_devinfo_tag *)of_get_property(chosen, "atag,devinfo",
							&len);
	of_node_put(chosen);
	if (!tag)
		return dev_err_probe(dev, -ENXIO, "no atag,devinfo property\n");

	size = tag->size;
	if (!size || size > MTK_DEVINFO_MAX_WORDS ||
	    len < (int)(sizeof(*tag) + size * sizeof(u32)))
		return dev_err_probe(dev, -EINVAL,
				     "bad atag,devinfo size %u (len %d)\n",
				     size, len);

	priv = devm_kzalloc(dev, sizeof(*priv) + size * sizeof(u32),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->size = size;
	memcpy(priv->data, tag->data, size * sizeof(u32));

	config.dev = dev;
	config.name = "mtk-devinfo";
	config.read_only = true;
	config.add_legacy_fixed_of_cells = true;
	config.reg_read = mtk_devinfo_read;
	config.size = size * sizeof(u32);
	config.word_size = sizeof(u32);
	config.stride = sizeof(u32);
	config.priv = priv;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id mtk_devinfo_of_match[] = {
	{ .compatible = "mediatek,devinfo" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_devinfo_of_match);

static struct platform_driver mtk_devinfo_driver = {
	.probe = mtk_devinfo_probe,
	.driver = {
		.name = "mtk-devinfo",
		.of_match_table = mtk_devinfo_of_match,
	},
};
module_platform_driver(mtk_devinfo_driver);

MODULE_AUTHOR("Mac Lu <mac.lu@mediatek.com>");
MODULE_DESCRIPTION("MediaTek bootloader devinfo NVMEM provider");
MODULE_LICENSE("GPL v2");
