# SPDX-License-Identifier: GPL-2.0
#
# qqcandy: vendor ccci_util Makefile, adapted to 6.18.
#  - built-in only (this device has no modules directory);
#  - the vendor guard on CONFIG_MTK_ECCCI_DRIVER / CONFIG_MTK_FPGA is gone,
#    the parent Makefile keys on CONFIG_MTK_CCCI_UTIL instead;
#  - ccci_util_dummy.o is added: it supplies the __weak fallbacks the vendor
#    tree got from the (absent) ECCCI/CCCI driver;
#  - ccci_util_probe_debug.o is new and holds the read-only diagnostic;
#  - the risky runtime probe (module_param_cb + workqueue + bounded tag parse)
#    was moved out into the separate ccci_probe.ko module
#    (drivers/misc/mediatek/ccci_probe/). Nothing here dereferences a reserved
#    region at boot: the LK/modem parse gate is a permanently-false bool.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include

obj-$(CONFIG_MTK_CCCI_UTIL) += ccci_util_lib.o

ccci_util_lib-y := \
	ccci_util_lib_fo.o \
	ccci_util_lib_load_img.o \
	ccci_util_lib_sys.o \
	ccci_private_log.o \
	ccci_util_lib_time.o \
	ccci_util_lib_main.o \
	ccci_util_ld_md_errno.o \
	ccci_util_broadcast.o \
	ccci_util_pin_broadcast.o \
	ccci_util_sib.o \
	ccci_util_lib_reserved_mem.o \
	ccci_util_md_rat.o \
	ccci_util_dummy.o \
	ccci_util_probe_debug.o
