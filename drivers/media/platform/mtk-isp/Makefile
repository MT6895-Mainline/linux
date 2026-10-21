# SPDX-License-Identifier: GPL-2.0
obj-y += camsys/
obj-$(CONFIG_VIDEO_MTK_ISP_HCP) += mtk-hcp/
obj-$(CONFIG_VIDEO_MTK_ISP_IMGSYS) += imgsys/
obj-$(CONFIG_VIDEO_MTK_ISP_IPESYS) += mtk-ipesys-me/

subdir-ccflags-y += -D__XIAOMI_CAMERA__
# MT6895 (xaga) port: xaga-specific bits are always compiled in
subdir-ccflags-y += -DXAGA_CAM
