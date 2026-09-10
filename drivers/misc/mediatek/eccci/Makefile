# SPDX-License-Identifier: GPL-2.0
# Evolved CCCI modem driver (mainline port, built-in) - MT6895: CLDMA+DPMAIF+CCIF
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

obj-$(CONFIG_MTK_CCCI_MAINLINE) += ccci_md_all.o ccci_ccif.o ccci_dpmaif.o 		ccci_cldma.o ccci_md_clk.o ccci_rtc.o ccci_auxadc.o

ccci_md_all-y := ccci_core.o ccci_bm.o ccci_modem.o modem_sys1.o 		udc/udc.o 		hif/ccci_hif.o 		port/port_cfg.o port/port_char.o port/port_ctlmsg.o 		port/port_udc.o port/port_ipc.o port/port_net.o 		port/port_poller.o port/port_proxy.o port/port_rpc.o 		port/port_smem.o port/port_sysmsg.o port/ccci_swtp.o 		fsm/ccci_fsm.o fsm/ccci_fsm_ioctl.o fsm/ccci_fsm_monitor.o 		fsm/ccci_fsm_poller.o fsm/ccci_fsm_ee.o fsm/ccci_fsm_sys.o 		fsm/mdee_dumper_v1.o fsm/mdee_dumper_v2.o fsm/mdee_dumper_v3.o 		fsm/mdee_dumper_v5.o 		platform/ccci_platform.o platform/ap_md_reg_dump.o 		platform/md_sys1_platform.o

ccci_ccif-y := hif/ccci_ringbuf.o hif/ccci_hif_ccif.o

ccci_dpmaif-y := hif/ccci_hif_dpmaif.o hif/dpmaif_drv.o 	hif/ccci_hif_dpmaif_comm.o hif/ccci_hif_dpmaif_v3.o hif/dpmaif_drv_v3.o 	hif/dpmaif_bat.o hif/dpmaif_bat_v3.o hif/dpmaif_debug.o hif/net_speed.o 	hif/net_pool.o hif/md_spd_dvfs_fn.o hif/md_spd_dvfs_method.o

ccci_cldma-y := hif/ccci_cldma_plat.o hif/ccci_hif_cldma.o
