# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(srctree)/drivers/gpu/mediatek/gpueb/include/

obj-$(CONFIG_MTK_GPUEB) += gpueb.o
gpueb-y := \
	gpueb_init.o \
	gpueb_ipi.o \
	gpueb_logger.o \
	gpueb_plat_service.o \
	gpueb_reserved_mem.o \
	gpueb_gpumpu.o \
	gpueb_hwvoter_dbg.o \
	gpueb_debug.o \
	gpueb_timesync.o \
	gpueb_power.o
