// SPDX-License-Identifier: GPL-2.0-only
/*
 * MTK SPMI PMIC polling keys driver
 *
 * Polls the key debounce status register of a MediaTek SPMI PMIC (e.g.
 * the MT6363 on the Dimensity 8100 / mt6895 xaga board) and reports
 * press/release via the input subsystem.
 *
 * The mainline MediaTek SPMI stack (spmi-mtk-pmif) does not yet provide
 * an SPMI-PMIC MFD with an interrupt domain (mtk-spmi-pmic is downstream
 * only), so the keys are polled here instead of using the PMIC IRQs.
 * The poll interval is short enough that key debounce behaviour is not
 * required from the driver (the PMIC debounces internally); we just
 * read the debounced status bits.
 *
 * DT binding:
 *   pmic@4 {
 *       compatible = "mediatek,mt6363";
 *       reg = <4 0>;
 *       mt6363keys {
 *           compatible = "mediatek,mt6363-keys";
 *           power { linux,keycodes = <KEY_POWER>; wakeup-source; };
 *           home  { linux,keycodes = <KEY_VOLUMEUP>; };
 *       };
 *   };
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MT6363_REG_TOPSTATUS	0x1e

#define MTK_KEYS_MAX		2
#define MTK_KEYS_POLL_MS	30

static const struct regmap_config mtk_keys_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

struct mtk_keys_info {
	unsigned int keycode;
	u32 deb_mask;
	unsigned int deb_reg;
	bool wakeup;
};

struct mtk_keys {
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input_dev;
	struct delayed_work poll_work;
	struct mtk_keys_info keys[MTK_KEYS_MAX];
	int num_keys;
	bool key_pressed[MTK_KEYS_MAX];
	u32 last_status;
};

static void mtk_keys_work(struct work_struct *work)
{
	struct mtk_keys *keys = container_of(work, struct mtk_keys,
					     poll_work.work);
	u32 status;
	int i, ret;

	ret = regmap_read(keys->regmap, keys->keys[0].deb_reg, &status);
	if (ret) {
		dev_err(keys->dev, "failed to read key status: %d\n", ret);
		goto reschedule;
	}

	if (status != keys->last_status) {
		dev_info(keys->dev, "TOPSTATUS = 0x%02x\n", status);
		keys->last_status = status;
	}

	for (i = 0; i < keys->num_keys; i++) {
		bool pressed;

		pressed = !(status & keys->keys[i].deb_mask);
		if (pressed != keys->key_pressed[i]) {
			keys->key_pressed[i] = pressed;
			input_report_key(keys->input_dev,
					 keys->keys[i].keycode, pressed);
			input_sync(keys->input_dev);
		}
	}

reschedule:
	schedule_delayed_work(&keys->poll_work,
			      msecs_to_jiffies(MTK_KEYS_POLL_MS));
}

static int mtk_keys_parse(struct mtk_keys *keys)
{
	struct device *dev = keys->dev;
	struct device_node *keys_np, *child;
	struct mtk_keys_info *info;
	int i = 0;

	/* As an MFD child, dev->of_node IS the "mediatek,mt6363-keys" node. */
	keys_np = of_node_get(dev->of_node);
	if (!keys_np) {
		keys->num_keys = 0;
		return 0;
	}

	for_each_available_child_of_node(keys_np, child) {
		if (i >= MTK_KEYS_MAX)
			break;

		info = &keys->keys[i];
		if (of_property_read_u32(child, "linux,keycodes",
					 &info->keycode)) {
			dev_err(dev, "key %d: missing linux,keycodes\n", i);
			of_node_put(child);
			of_node_put(keys_np);
			return -EINVAL;
		}

		/* MT6363 TOPSTATUS layout (verified on hardware):
		 * PWRKEY debounce at bit1, HOMEKEY debounce at bit3.
		 * (Same layout as MT6359: deb masks 0x2 / 0x8.)
		 * The stock xaga DT's child order is power then home. */
		info->deb_reg = MT6363_REG_TOPSTATUS;
		info->deb_mask = (i == 0) ? BIT(1) : BIT(3);
		info->wakeup = of_property_read_bool(child, "wakeup-source");
		i++;
	}

	of_node_put(keys_np);
	keys->num_keys = i;

	if (keys->num_keys == 0) {
		dev_err(dev, "no keys defined\n");
		return -EINVAL;
	}

	return 0;
}

static int mtk_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_keys *keys;
	int i, error;

	keys = devm_kzalloc(dev, sizeof(*keys), GFP_KERNEL);
	if (!keys)
		return -ENOMEM;

	keys->dev = dev;
	INIT_DELAYED_WORK(&keys->poll_work, mtk_keys_work);

	/* Regmap is provided by the parent SPMI PMIC MFD. */
	keys->regmap = dev_get_regmap(dev->parent, NULL);
	if (!keys->regmap) {
		dev_err(dev, "failed to get parent regmap\n");
		return -ENODEV;
	}

	error = mtk_keys_parse(keys);
	if (error)
		return error;

	if (keys->num_keys > 0) {
		keys->input_dev = devm_input_allocate_device(dev);
		if (!keys->input_dev)
			return -ENOMEM;

		keys->input_dev->name = "mtk-spmi-keys";
		keys->input_dev->id.bustype = BUS_HOST;
		keys->input_dev->dev.parent = dev;

		__set_bit(EV_KEY, keys->input_dev->evbit);
		for (i = 0; i < keys->num_keys; i++)
			__set_bit(keys->keys[i].keycode, keys->input_dev->keybit);

		error = input_register_device(keys->input_dev);
		if (error)
			return error;

		schedule_delayed_work(&keys->poll_work,
				      msecs_to_jiffies(MTK_KEYS_POLL_MS));
	}

	platform_set_drvdata(pdev, keys);

	dev_info(dev, "probed %d keys (polling %dms)\n",
		 keys->num_keys, MTK_KEYS_POLL_MS);

	return 0;
}

static void mtk_keys_remove(struct platform_device *pdev)
{
	struct mtk_keys *keys = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&keys->poll_work);
}

static const struct of_device_id mtk_keys_of_match[] = {
	{ .compatible = "mediatek,mt6363-keys" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_keys_of_match);

static struct platform_driver mtk_keys_driver = {
	.driver = {
		.name = "mtk-spmi-keys",
		.of_match_table = mtk_keys_of_match,
	},
	.probe = mtk_keys_probe,
	.remove = mtk_keys_remove,
};
module_platform_driver(mtk_keys_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MTK SPMI PMIC polling keys driver");
