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

#include <linux/bits.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MT6363_REG_TOPSTATUS	0x1e

/*
 * PON/STRUP long-press handling.  The PMIC itself performs the reset once
 * PWRKEY_RST_EN is set; the AP is not involved at press time.  This mirrors
 * what the downstream mtk-pmic-keys.c programmes from the same DT properties
 * ("mediatek,long-press-mode" / "power-off-time-sec"), and is what turns a
 * ~10 s power-key hold into a warm reset instead of the PMIC's default ~16 s
 * power-off.  LK only observes/clears the resulting JUST_PWRKEY_RST status.
 */
#define MT6363_REG_STRUP_CON11		0x0a0e
#define MT6363_REG_STRUP_CON12		0x0a0f
#define MT6363_PWRKEY_RST_EN		BIT(2)
#define MT6363_RST_MODE_SHIFT		4
#define MT6363_RST_MODE_MASK		GENMASK(5, 4)
#define MT6363_RST_MODE_PWRKEY		0
#define MT6363_RST_MODE_PWRKEY_HOME	1
#define MT6363_RST_MODE_PWRKEY_HOME2	2
#define MT6363_RST_MODE_ALL		3
#define MT6363_RST_DU_SHIFT		6
#define MT6363_RST_DU_MASK		GENMASK(7, 6)

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

static void mtk_keys_lp_reset_setup(struct mtk_keys *keys)
{
	unsigned int con11 = 0, con12 = 0, n11 = 0, n12 = 0;
	u32 mode, dur;
	int ret;

	if (of_property_read_u32(keys->dev->of_node,
				 "mediatek,long-press-mode", &mode))
		mode = 0;
	if (of_property_read_u32(keys->dev->of_node,
				 "power-off-time-sec", &dur))
		dur = 0;

	ret = regmap_read(keys->regmap, MT6363_REG_STRUP_CON11, &con11);
	if (!ret)
		ret = regmap_read(keys->regmap, MT6363_REG_STRUP_CON12, &con12);
	if (ret) {
		dev_err(keys->dev, "failed to read PON config: %d\n", ret);
		return;
	}

	/* Long-press duration: 2-bit RST_DU field. */
	regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON12,
			   MT6363_RST_DU_MASK,
			   (dur & 3) << MT6363_RST_DU_SHIFT);

	switch (mode) {
	case 1: /* single power key */
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON12,
				   MT6363_RST_MODE_MASK,
				   MT6363_RST_MODE_PWRKEY <<
				   MT6363_RST_MODE_SHIFT);
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON11,
				   MT6363_PWRKEY_RST_EN,
				   MT6363_PWRKEY_RST_EN);
		break;
	case 2: /* power key + home key */
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON12,
				   MT6363_RST_MODE_MASK,
				   MT6363_RST_MODE_PWRKEY_HOME <<
				   MT6363_RST_MODE_SHIFT);
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON11,
				   MT6363_PWRKEY_RST_EN,
				   MT6363_PWRKEY_RST_EN);
		break;
	default: /* disabled */
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON11,
				   MT6363_PWRKEY_RST_EN, 0);
		regmap_update_bits(keys->regmap, MT6363_REG_STRUP_CON12,
				   MT6363_RST_MODE_MASK,
				   MT6363_RST_MODE_ALL <<
				   MT6363_RST_MODE_SHIFT);
		break;
	}

	regmap_read(keys->regmap, MT6363_REG_STRUP_CON11, &n11);
	regmap_read(keys->regmap, MT6363_REG_STRUP_CON12, &n12);
	dev_info(keys->dev,
		 "PON long-press mode=%u time=%u: CON11 %#04x->%#04x, CON12 %#04x->%#04x\n",
		 mode, dur, con11, n11, con12, n12);
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

	mtk_keys_lp_reset_setup(keys);

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
