// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/iio/consumer.h>
#include <linux/iio/iio.h>

#include <dt-bindings/iio/mt635x-auxadc.h>

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_core.h"
#include "ccci_debug.h"

#define TAG "ccci_adc"

static struct device *md_adc_pdev;
static int adc_num;
static int adc_val;
static int adc_mV;
static bool qqc_adc_ready;
static bool qqc_adc_fallback_disable;

module_param_named(fallback_disable, qqc_adc_fallback_disable, bool, 0444);
MODULE_PARM_DESC(fallback_disable,
	"v485: disable the module-side md_auxadc fallback (1 = disable)");

#ifdef CCCI_KMODULE_ENABLE
/*
 * for debug log:
 * 0 to disable; 1 for print to ram; 2 for print to uart
 * other value to desiable all log
 */
#ifndef CCCI_LOG_LEVEL /* for platform override */
#define CCCI_LOG_LEVEL CCCI_LOG_CRITICAL_UART
#endif
unsigned int ccci_debug_enable = CCCI_LOG_LEVEL;
#endif

static int ccci_get_adc_info(struct device *dev)
{
	int ret, val, mV;
	struct iio_channel *md_channel;

	adc_val = -1;
	adc_mV = -1;
	md_channel = iio_channel_get(dev, "md-channel");

	ret = IS_ERR(md_channel);
	if (ret) {
		if (PTR_ERR(md_channel) == -EPROBE_DEFER) {
			CCCI_ERROR_LOG(-1, TAG, "%s EPROBE_DEFER\r\n",
					__func__);
			return -EPROBE_DEFER;
		}
		CCCI_ERROR_LOG(-1, TAG, "fail to get iio channel (%d)", ret);
		goto Fail;
	}
	adc_num = md_channel->channel->channel;

	ret = iio_read_channel_raw(md_channel, &val);
	if (ret < 0) {
		iio_channel_release(md_channel);
		CCCI_ERROR_LOG(-1, TAG, "iio_read_channel_raw fail");
		goto Fail;
	}
	adc_val = val;

	ret = iio_read_channel_processed(md_channel, &mV);
	iio_channel_release(md_channel);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG, "iio_read_channel_processed fail");
		goto Fail;
	}
	adc_mV = mV;

	CCCI_NORMAL_LOG(0, TAG, "md_ch = %d, raw_val = %d[%dmV]\n",
		adc_num, val, mV);
	return ret;

Fail:
	return -1;
}

int ccci_get_adc_num(void)
{
	return adc_num;
}
EXPORT_SYMBOL(ccci_get_adc_num);

int ccci_get_adc_val(void)
{
	return adc_val;
}
EXPORT_SYMBOL(ccci_get_adc_val);

int ccci_get_adc_mV(void)
{
	return adc_mV;
}
EXPORT_SYMBOL(ccci_get_adc_mV);

int get_auxadc_probe(struct platform_device *pdev)
{
	int ret;

	ret = ccci_get_adc_info(&pdev->dev);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG, "ccci get adc info fail");
		return ret;
	}
	md_adc_pdev = &pdev->dev;
	qqc_adc_ready = true;
	return 0;
}


static const struct of_device_id ccci_auxadc_of_ids[] = {
	{.compatible = "mediatek,md_auxadc"},
	{}
};


static struct platform_driver ccci_auxadc_driver = {

	.driver = {
			.name = "ccci_auxadc",
			.of_match_table = ccci_auxadc_of_ids,
	},

	.probe = get_auxadc_probe,
};

/*
 * v485 module-side fallback for the missing /soc/md_auxadc node.
 *
 * The authenticated stock qqcandy runtime FDT declares /soc/md_auxadc with
 * io-channels = <&pmic_adc 0x313>, and the vendor source that produces that
 * cell is mt6895.dts:14134-14136
 *   &md_auxadc { io-channels = <&pmic_adc
 *                (ADC_PURES_OPEN_MASK | AUXADC_VIN1)>; }
 * i.e. PMIC AUXADC channel 19 (VIN1) with the pull-up OPEN (pures 3); the cell
 * 0x300 | 0x13 = 0x313.  ccci_get_adc_info() therefore only ever runs through
 * the of_match_table probe + iio_channel_get(dev, "md-channel").
 *
 * Our mainline DTS now declares that node as well
 * (arch/arm64/boot/dts/mediatek/mt6895-oplus-qqcandy.dts), but the Image
 * carries its own DTB override and re-flashing boot_a/boot_b is out of scope,
 * so on the current module-only deployment no platform device appears and
 * adc_num/adc_val/adc_mV keep their static zeros.  The AP then reports
 * "ADC val:0, EVB" to the modem (ccci_modem.c:2022-2030 sets BOOT_INFO bit 1
 * to 0, i.e. "EVB") and answers RPC 0x4007 with 0.
 *
 * This resolves the very same provider channel/pures pair directly through the
 * public IIO consumer API instead of DT.  It runs only when no
 * "mediatek,md_auxadc" DT node exists and the normal probe has not succeeded,
 * so it becomes dead code by itself once the image carries the node.  No
 * guessed values are used: channel 19 / pures 3 come from the vendor source
 * and the stock FDT cell 0x313.
 *
 * The IIO core registers its bus device as "iio:deviceN" (industrialio-core.c
 * dev_set_name) while the DT node name lives in <iio>/name, so the provider is
 * located by its parent's device-tree compatible, not by bus name.
 */
static int qqc_iio_match_mt6363_auxadc(struct device *dev, const void *data)
{
	struct device_node *np;

	if (!dev->parent)
		return 0;
	np = dev->parent->of_node;
	if (!np)
		return 0;
	return of_device_is_compatible(np, "mediatek,mt6363-auxadc");
}

static int qqc_md_auxadc_fallback(void)
{
	struct device *iiodev;
	struct iio_dev *indio;
	const struct iio_chan_spec *spec = NULL;
	struct iio_channel *chan;
	int i, ret, val = 0, mV = 0;

	if (qqc_adc_fallback_disable) {
		CCCI_ERROR_LOG(-1, TAG,
			"v485 fallback disabled by module parameter");
		return -ENODEV;
	}

	iiodev = bus_find_device(&iio_bus_type, NULL, NULL,
				 qqc_iio_match_mt6363_auxadc);
	if (!iiodev) {
		CCCI_ERROR_LOG(-1, TAG,
			"v485 fallback: mt6363-auxadc IIO device not found");
		return -ENODEV;
	}
	indio = dev_to_iio_dev(iiodev);

	for (i = 0; i < indio->num_channels; i++) {
		if (indio->channels[i].channel == AUXADC_VIN1 &&
		    indio->channels[i].channel2 == ADC_PURES_OPEN) {
			spec = &indio->channels[i];
			break;
		}
	}
	if (!spec) {
		CCCI_ERROR_LOG(-1, TAG,
			"v485 fallback: VIN1/pures-open channel not found");
		ret = -ENODEV;
		goto put;
	}

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan) {
		ret = -ENOMEM;
		goto put;
	}
	chan->indio_dev = indio;	/* takes over the bus_find_device ref */
	chan->channel = spec;

	adc_val = -1;
	adc_mV = -1;
	ret = iio_read_channel_raw(chan, &val);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG,
			"v485 fallback: read raw fail %d", ret);
		goto rel;
	}
	adc_val = val;

	ret = iio_read_channel_processed(chan, &mV);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG,
			"v485 fallback: read mV fail %d", ret);
		goto rel;
	}
	adc_mV = mV;
	adc_num = spec->channel;
	qqc_adc_ready = true;
	ret = 0;

	CCCI_NORMAL_LOG(0, TAG,
		"v485 fallback md_ch = %d, raw_val = %d[%dmV]\n",
		adc_num, adc_val, adc_mV);
rel:
	iio_channel_release(chan);	/* iio_device_put() + kfree() */
	return ret;
put:
	iio_device_put(indio);		/* balances bus_find_device ref */
	return ret;
}

static int __init ccci_auxadc_init(void)
{
	struct device_node *np;
	int ret;

	ret = platform_driver_register(&ccci_auxadc_driver);
	if (ret) {
		CCCI_ERROR_LOG(-1, TAG, "ccci auxadc driver init fail %d", ret);
		return ret;
	}
	if (!qqc_adc_ready) {
		np = of_find_compatible_node(NULL, NULL,
					     "mediatek,md_auxadc");
		if (np)
			of_node_put(np);
		else
			qqc_md_auxadc_fallback();
	}
	return 0;
}

module_init(ccci_auxadc_init);

MODULE_AUTHOR("ccci");
MODULE_DESCRIPTION("ccci auxadc driver");
MODULE_LICENSE("GPL");
