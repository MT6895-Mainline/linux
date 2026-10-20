# SPDX-License-Identifier: GPL-2.0
#
# MediaTek hf_manager: the char-device sensor broker (/dev/hf_manager) that the
# camera HAL's SensorProviderService talks to.  Pure framework code with no
# platform dependency -- real sensors register via the exported hf_device_* API.
# Built under CONFIG_MTK_SENSORHUB so it lands in the same kernel image.

obj-$(CONFIG_MTK_SENSORHUB) += hf_manager.o
