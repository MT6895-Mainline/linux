# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MTK_CMDQ_MBOX_EXT) += cmdq/
obj-$(CONFIG_MTK_MMINFRA) += mminfra/
obj-$(CONFIG_MTK_BTIF) += btif/
obj-$(CONFIG_MTK_COMBO) += connectivity/
obj-$(CONFIG_MTK_TINYSYS_SCP_SUPPORT) += scp/
obj-$(CONFIG_MTK_SENSOR_SUPPORT) += sensor/2.0/
obj-$(CONFIG_MTK_CCCI_MAINLINE) += ccci_util/ ccmni/ eccci/

#
# Xiaomi camera: build the imgsensor subdrivers (and the rest of the MTK camera
# glue) with __XIAOMI_CAMERA__ set, exactly like the downstream MTK tree.
# Some sensor drivers, e.g. the GC02M1 macro, use the __XIAOMI_CAMERA__-only
# subdrv_i2c_*_u8_u8 helpers in adaptor-subdrv.h and fail to build without it.
#
subdir-ccflags-y += -D__XIAOMI_CAMERA__

obj-y += pda-stub.o
obj-$(CONFIG_MTK_V4L2_IMGSENSOR) += imgsensor/src-v4l2/
obj-$(CONFIG_INTERCONNECT_MTK_EXTENSION) += mtk-interconnect/
obj-$(CONFIG_MTK_V4L2_LENS) += lens/vcm/v4l2/
obj-$(CONFIG_MTK_FLASHLIGHT) += flashlight/
obj-$(CONFIG_MTK_V4L2_FLASHLIGHT) += flashlight/v4l2/
