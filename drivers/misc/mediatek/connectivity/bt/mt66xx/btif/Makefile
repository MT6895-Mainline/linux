# SPDX-License-Identifier: GPL-2.0
#
# MediaTek MT66xx Bluetooth-over-BTIF driver for the mainline xaga port.

BT_PLATFORM := 6895

ccflags-y += -D FW_LOG_DEFAULT_ON=0
ccflags-y += -D BUILD_QA_DBG=0
ccflags-y += -D CONNAC20_CHIPID=$(BT_PLATFORM)
ccflags-y += -D CHIP_IF_BTIF
ccflags-y += -D USE_DEVICE_NODE=0
ccflags-y += -D CUSTOMER_FW_UPDATE=0
ccflags-y += -D PM_QOS_CONTROL=0
ccflags-y += -D CONFIG_MP_WAKEUP_SOURCE_SYSFS_STAT=1

MODULE_SRC := $(srctree)/drivers/misc/mediatek/connectivity/bt/mt66xx
CONN_INFRA_SRC := $(srctree)/drivers/misc/mediatek/connectivity/conninfra
CONNFEM_SRC := $(srctree)/drivers/misc/mediatek/connectivity/connfem
CONNADP_SRC := $(srctree)/drivers/misc/mediatek/connectivity/connadp
BTIF_SRC := $(srctree)/drivers/misc/mediatek/btif

ccflags-y += -I$(CONNADP_SRC)/common
ccflags-y += -I$(srctree)/drivers/misc/mediatek/connectivity/compat/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(CONNADP_SRC)/power_throttling
ccflags-y += -I$(CONN_INFRA_SRC)/include
ccflags-y += -I$(CONN_INFRA_SRC)/debug_utility/include
ccflags-y += -I$(CONN_INFRA_SRC)/debug_utility/metlog
ccflags-y += -I$(CONN_INFRA_SRC)/debug_utility/
ccflags-y += -I$(CONNFEM_SRC)/include
ccflags-y += -I$(BTIF_SRC)/common/inc
ccflags-y += -I$(MODULE_SRC)/include
ccflags-y += -I$(MODULE_SRC)/include/btif

obj-$(CONFIG_MTK_BTIF) += bt_drv_$(BT_PLATFORM).o

bt_drv_$(BT_PLATFORM)-y := \
btmtk_dbg.o \
btmtk_dbg_tp_evt_if.o \
btmtk_irq.o \
btmtk_char_dev.o \
../btmtk_fw_log.o \
../btmtk_main.o \
btmtk_mt66xx.o \
btmtk_btif_main.o \
btmtk_queue.o
