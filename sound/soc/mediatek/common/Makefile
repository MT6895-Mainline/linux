# SPDX-License-Identifier: GPL-2.0
# platform driver
snd-soc-mtk-common-y := mtk-afe-platform-driver.o mtk-afe-fe-dai.o mtk-dsp-sof-common.o mtk-soundcard-driver.o
snd-soc-mtk-common-y += mtk-dai-adda-common.o
# notifier-chain + dma-buf exporter helpers used by the AFE core (self-contained)
snd-soc-mtk-common-y += mtk-afe-external.o mtk-mmap-ion.o
snd-soc-mtk-common-$(CONFIG_SND_SOC_MTK_SMART_PHONE) += mtk-sp-pcm-ops.o mtk-afe-debug.o
snd-soc-mtk-common-$(CONFIG_SND_SOC_MTK_SRAM) += mtk-sram-manager.o

obj-$(CONFIG_SND_SOC_MEDIATEK) += snd-soc-mtk-common.o

# speaker amp glue; tfa98xx calls mtk_spk_set_type() so keep it built-in
obj-$(CONFIG_SND_SOC_MTK_SPK_AMP) += mtk-sp-spk-amp.o

obj-$(CONFIG_SND_SOC_MTK_BTCVSD) += mtk-btcvsd.o
