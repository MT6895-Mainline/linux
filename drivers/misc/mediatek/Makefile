# SPDX-License-Identifier: GPL-2.0
#
# Xiaomi camera: build the imgsensor subdrivers (and the rest of the MTK camera
# glue) with __XIAOMI_CAMERA__ set, exactly like the downstream MTK tree
# (crdroid .../drivers/misc/mediatek/Makefile).  Some sensor drivers, e.g. the
# GC02M1 macro (xagagc02m1_mipi_raw), use the __XIAOMI_CAMERA__-only
# subdrv_i2c_*_u8_u8 helpers in adaptor-subdrv.h and fail to build without it.
subdir-ccflags-y += -D__XIAOMI_CAMERA__

obj-y += pda-stub.o
obj-$(CONFIG_MTK_CMDQ_MBOX_EXT) += cmdq/
obj-$(CONFIG_MTK_V4L2_IMGSENSOR) += imgsensor/src-v4l2/
obj-$(CONFIG_MTK_MMINFRA) += mminfra/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += scp/
obj-$(CONFIG_MTK_SENSORHUB) += sensor/2.0/
obj-$(CONFIG_INTERCONNECT_MTK_EXTENSION) += mtk-interconnect/
obj-$(CONFIG_MTK_CAM_CAL) += cam_cal/
