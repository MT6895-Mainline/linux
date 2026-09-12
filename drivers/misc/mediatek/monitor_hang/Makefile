# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2015 MediaTek Inc.
#
# qqcandy: the vendor Makefile added the AEE, mt-plat and (conditionally)
# MTK ION include paths. None of them are needed by the report-only port,
# which now only uses generic kernel headers.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/

obj-$(CONFIG_MTK_HANG_DETECT) += monitor_hang.o
monitor_hang-objs += hang_detect.o hang_unwind.o
