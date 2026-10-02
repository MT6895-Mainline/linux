# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MTK_SENSOR_SUPPORT) += core/
obj-$(CONFIG_MTK_SENSORHUB) += sensorhub/
obj-$(CONFIG_MTK_HF_IIO) += mtk_hf_iio.o
CFLAGS_mtk_hf_iio.o += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/core
CFLAGS_mtk_hf_iio.o += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/sensorhub
