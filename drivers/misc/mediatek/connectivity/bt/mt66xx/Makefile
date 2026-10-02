# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_MTK_BT_MT66XX) += btmtk_drv.o

btmtk_drv-objs := btmtk_main.o \
                  btmtk_fw_log.o \
                  btmtk_woble.o \
                  btmtk_chip_reset.o \
                  btmtk_buffer_mode.o \
                  btif/btmtk_btif_main.o \
                  btif/btmtk_mt66xx.o \
                  btif/btmtk_char_dev.o \
                  btif/btmtk_dbg.o \
                  btif/btmtk_dbg_tp_evt_if.o \
                  btif/btmtk_irq.o \
                  btif/btmtk_queue.o
btmtk_drv-y := $(btmtk_drv-objs)

ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/bt/mt66xx/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/bt/mt66xx/include/btif
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/bt/shim
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/conninfra/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/conninfra/debug_utility
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/conninfra/debug_utility/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/conninfra/debug_utility/metlog
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/conninfra/platform/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/connadp/common
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/connadp/power_throttling
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/connfem/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/btif/common/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/btif/common/plat_inc
ccflags-y += -DCONNAC20_CHIPID=6895
ccflags-y += -DCHIP_IF_BTIF=1
ccflags-y += -DUSE_DEVICE_NODE=0
ccflags-y += -DCFG_SUPPORT_BLUEZ=1
ccflags-y += -DCONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH
ccflags-y += -Wno-unused-variable -Wno-unused-function -Wno-unused-but-set-variable -Wno-unused-label -Wno-format -Wno-parentheses
ccflags-y += -include $(srctree)/drivers/misc/mediatek/connectivity/bt/shim/mtk_compat_types.h
