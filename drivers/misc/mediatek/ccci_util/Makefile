# SPDX-License-Identifier: GPL-2.0
# CCCI utility library (mainline port, built-in)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/ccci-include \
	-I$(srctree)/drivers/misc/mediatek/ccci-include/conn_md/include \
	-I$(srctree)/drivers/misc/mediatek/ccci-include/rps \
	-I$(srctree)/drivers/misc/mediatek/ccci_util \
	-I$(srctree)/drivers/misc/mediatek/ccmni \
	-I$(srctree)/drivers/misc/mediatek/eccci \
	-I$(srctree)/drivers/misc/mediatek/eccci/inc \
	-I$(srctree)/drivers/misc/mediatek/eccci/platform \
	-I$(srctree)/drivers/misc/mediatek/eccci/hif \
	-I$(srctree)/drivers/misc/mediatek/eccci/port \
	-I$(srctree)/drivers/misc/mediatek/eccci/fsm \
	-I$(srctree)/drivers/misc/mediatek/eccci/udc \
	-I$(srctree)/drivers/misc/mediatek/eccci/udc/udc_lib_inc

# vendor config emulation (paths kept identical to the proven out-of-tree build)
ccflags-y += -DCCCI_KMODULE_ENABLE
ccflags-y += -DCCCI_CCMNI_MODULE
ccflags-y += -DMET_USER_EVENT_SUPPORT
ccflags-y += -DCONFIG_MTK_ECCCI_DRIVER
ccflags-y += -DCONFIG_MTK_ECCCI_CLDMA
ccflags-y += -DCONFIG_MTK_NET_CCMNI
ccflags-y += -DCONFIG_MTK_CCCI_DEVICES

obj-$(CONFIG_MTK_CCCI_MAINLINE) += ccci_util_lib.o
ccci_util_lib-y := ccci_util_lib_fo.o 		ccci_util_lib_load_img.o 		ccci_util_lib_sys.o 		ccci_private_log.o 		ccci_util_lib_time.o 		ccci_util_lib_main.o 		ccci_util_ld_md_errno.o 		ccci_util_broadcast.o 		ccci_util_sib.o 		ccci_util_lib_reserved_mem.o 		ccci_util_md_rat.o
