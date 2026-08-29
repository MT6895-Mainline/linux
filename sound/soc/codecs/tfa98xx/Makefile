# SPDX-License-Identifier: GPL-2.0
#
# Goodix TFA98xx smart amplifier family (xaga: tfa9874 x2 on I2C5)
#
# -DTFA_NON_DSP_SOLUTION: no ADSP/SCP dependency, I2S direct drive.

ccflags-y += -I$(srctree)/sound/soc/codecs/tfa98xx/inc
ccflags-y += -I$(srctree)/sound/soc/mediatek/common
ccflags-y += -DTFA_NON_DSP_SOLUTION
ccflags-y += $(call cc-option,-Wno-date-time,)
# downstream builds with -Werror; relax for mainline port
ccflags-y += $(call cc-option,-Wno-error,)

snd-soc-tfa98xx-objs += src/tfa98xx.o
snd-soc-tfa98xx-objs += src/tfa_container.o
snd-soc-tfa98xx-objs += src/tfa_dsp.o
snd-soc-tfa98xx-objs += src/tfa_init.o

obj-$(CONFIG_SND_SOC_TFA9874) += snd-soc-tfa98xx.o
