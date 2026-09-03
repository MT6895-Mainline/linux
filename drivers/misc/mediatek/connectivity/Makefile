# SPDX-License-Identifier: GPL-2.0
#
# MT6895 connectivity stack (downstream CONNAC2X2 SOC7_0 fullmac port).
# Built-in only for the xaga mainline bring-up.

obj-y += connadp/
obj-y += conninfra/
obj-y += connfem/
obj-$(CONFIG_MTK_BTIF) += bt/mt66xx/btif/
obj-y += wlan_adaptor/
obj-y += wlan_drv_gen4m/

export CONFIG_MTK_COMBO_WIFI_HIF := axi
export CONNAC_VER := 2_0
export MTK_COMBO_CHIP := CONNAC2X2_SOC7_0
export WLAN_CHIP_ID := 6895
export MTK_ANDROID_WMT := y
export MTK_ANDROID_EMI := y
export WIFI_IP_SET := 1
export MTK_WLAN_SERVICE := yes
# Match the stock Android user build; mode 0 is invalid for the downstream
# conninfra firmware interface and was being written into CONN SYSRAM.
export TARGET_BUILD_VARIANT := user
export MODULE_NAME := wlan_drv_gen4m_6895
export CONFIG_MTK_CONNSYS_DEDICATED_LOG_PATH := y
# Tell the downstream wlan Makefile this is a built-in driver so it does not
# define symbols that mainline cfg80211 already provides (e.g. is_world_regdom).
export KBUILD_SUBDIR_CCFLAGS += MTK_WCN_REMOVE_KERNEL_MODULE
