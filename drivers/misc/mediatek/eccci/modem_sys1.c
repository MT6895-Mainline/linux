// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/string.h>
#include <linux/soc/mediatek/ccci_mtk_sip_svc.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched/clock.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h> /* local_clock() */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>

#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
#include <mt-plat/aee.h>
#endif
#if IS_ENABLED(CONFIG_MTK_PBM)
#include "mtk_pbm.h"
#include "mtk_mdpm.h"
#endif
#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_core.h"
#include "ccci_modem.h"
#include "ccci_bm.h"
#include "ccci_platform.h"
#include "md_sys1_platform.h"
#include "modem_reg_base.h"
#include "modem_secure_base.h"

#include "ccci_debug.h"
#include "ccci_port.h"
#include "hif/ccci_hif_cldma.h"
#include "hif/ccci_hif_ccif.h"
#include "modem_sys.h"

#define TAG "mcd"

static void debug_in_flight_mode(struct ccci_modem *md);

#ifdef CCCI_KMODULE_ENABLE
bool spm_is_md1_sleep(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, MD_CLOCK_REQUEST,
		MD_GET_SLEEP_MODE, 0, 0, 0, 0, 0, &res);

	CCCI_NORMAL_LOG(-1, TAG,
		"[%s] flag_1=%llx, flag_2=%llx, flag_3=%llx, flag_4=%llx\n",
		__func__, res.a0, res.a1, res.a2, res.a3);
	if (res.a0 == 0)
		return res.a1; /* 1-md is sleep, 0 - is not */
	else
		return 2; /* not support, no wait */
}

#endif

void ccif_enable_irq(struct ccci_modem *md)
{
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;

	if (atomic_cmpxchg(&md_info->ccif_irq_enabled, 0, 1) == 0) {
		enable_irq(md_info->ap_ccif_irq_id);
		CCCI_NORMAL_LOG(md->index, TAG, "enable ccif irq\n");
	}
}

void ccif_disable_irq(struct ccci_modem *md)
{
	struct md_sys1_info *md_info = (struct md_sys1_info *)md->private_data;

	if (atomic_cmpxchg(&md_info->ccif_irq_enabled, 1, 0) == 1) {
		disable_irq_nosync(md_info->ap_ccif_irq_id);
		/*CCCI_NORMAL_LOG(md->index, TAG, "disable ccif irq\n");*/
	}
}

void wdt_enable_irq(struct ccci_modem *md)
{
	if (atomic_cmpxchg(&md->wdt_enabled, 0, 1) == 0) {
		enable_irq(md->md_wdt_irq_id);
		CCCI_NORMAL_LOG(md->index, TAG, "enable wdt irq\n");
	}
}

void wdt_disable_irq(struct ccci_modem *md)
{
	if (atomic_cmpxchg(&md->wdt_enabled, 1, 0) == 1) {
		/*
		 * may be called in isr, so use disable_irq_nosync.
		 * if use disable_irq in isr, system will hang
		 */
		disable_irq_nosync(md->md_wdt_irq_id);
		/*CCCI_NORMAL_LOG(md->index, TAG, "disable wdt irq\n");*/
	}
}

static irqreturn_t md_cd_wdt_isr(int irq, void *data)
{
	struct ccci_modem *md = (struct ccci_modem *)data;

	//CCCI_ERROR_LOG(md->index, TAG, "MD WDT IRQ\n");
	//ccci_event_log("md%d: MD WDT IRQ\n", md->index);
	ccif_disable_irq(md);
	wdt_disable_irq(md);
	ccci_fsm_recv_md_interrupt(md->index, MD_IRQ_WDT);

	return IRQ_HANDLED;
}

static void md_cd_ccif_delayed_work(struct ccci_modem *md)
{
	if (md->hif_flag & (1<<CLDMA_HIF_ID))  {
		/* stop CLDMA, we don't want to get CLDMA IRQ when MD is
		 * resetting CLDMA after it got cleaq_ack
		 */
		ccci_hif_stop(CLDMA_HIF_ID);
		CCCI_NORMAL_LOG(md->index, TAG,
			"%s: stop cldma done\n", __func__);
		/*dump rxq after cldma stop to avoid race condition*/
		ccci_hif_dump_status(1 << CLDMA_HIF_ID, DUMP_FLAG_QUEUE_0_1,
			NULL, 1 << IN);
		CCCI_NORMAL_LOG(md->index, TAG,
			"%s: dump queue0-1 done\n", __func__);

		ccci_hif_hw_reset(1 << CLDMA_HIF_ID, md->index);
		CCCI_NORMAL_LOG(md->index, TAG,
			"%s: hw reset done\n", __func__);
		ccci_hif_clear_all_queue(1 << CLDMA_HIF_ID, IN);
	}
}

static void md_cd_exception(struct ccci_modem *md, enum HIF_EX_STAGE stage)
{
	struct ccci_smem_region *mdccci_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDCCCI_DBG);

	CCCI_ERROR_LOG(md->index, TAG, "MD exception HIF %d\n", stage);
	ccci_event_log("md%d:MD exception HIF %d\n", md->index, stage);
	/* in exception mode, MD won't sleep, so we do not
	 * need to request MD resource first
	 */
	switch (stage) {
	case HIF_EX_INIT:
		ccci_hif_dump_status(1 << CCIF_HIF_ID, DUMP_FLAG_CCIF |
			DUMP_FLAG_IRQ_STATUS, NULL, 0);
		if (*((int *)(mdccci_dbg->base_ap_view_vir +
			CCCI_SMEM_OFFSET_SEQERR)) != 0) {
			CCCI_ERROR_LOG(md->index, TAG,
				"MD found wrong sequence number\n");
		}
		if (md->hif_flag & (1<<CLDMA_HIF_ID))  {
			CCCI_ERROR_LOG(md->index, TAG,
				"dump cldma on ccif hs0\n");
			ccci_hif_dump_status(1 << CLDMA_HIF_ID,
				DUMP_FLAG_CLDMA, NULL, -1);
			/* disable CLDMA except un-stop queues */
			ccci_hif_stop_for_ee(1 << CLDMA_HIF_ID);
			/* purge Tx queue */
			ccci_hif_clear_all_queue(1 << CLDMA_HIF_ID, OUT);
		}
		ccci_hif_md_exception(md->hif_flag, stage);
		/* Rx dispatch does NOT depend on queue index
		 * in port structure, so it still can find right port.
		 */
		ccci_hif_send_data(CCIF_HIF_ID, H2D_EXCEPTION_ACK);
		break;
	case HIF_EX_INIT_DONE:
		break;
	case HIF_EX_CLEARQ_DONE:
		/* give DHL some time to flush data */
		msleep(2000);
		md_cd_ccif_delayed_work(md);
		ccci_hif_md_exception(md->hif_flag, stage);
		/* tell MD to reset CLDMA */
		ccci_hif_send_data(CCIF_HIF_ID, H2D_EXCEPTION_CLEARQ_ACK);
		CCCI_NORMAL_LOG(md->index, TAG, "send clearq_ack to MD\n");
		break;
	case HIF_EX_ALLQ_RESET:
		md->per_md_data.is_in_ee_dump = 1;
		if (md->hif_flag & (1<<CLDMA_HIF_ID))
			ccci_hif_all_q_reset(1 << CLDMA_HIF_ID);
		ccci_hif_md_exception(md->hif_flag, stage);
		break;
	default:
		break;
	};
}

static void polling_ready(struct ccci_modem *md, int step)
{
	int cnt = 500; /*MD timeout is 10s*/
	int time_once = 10;
	struct md_sys1_info *md_info =
		(struct md_sys1_info *)md->private_data;

#ifdef CCCI_EE_HS_POLLING_TIME
	cnt = CCCI_EE_HS_POLLING_TIME / time_once;
#endif
	while (cnt > 0) {
		if (md_info->channel_id & (1 << step)) {
			CCCI_DEBUG_LOG(md->index, TAG,
				"poll RCHNUM %d\n", md_info->channel_id);
			return;
		}
		msleep(time_once);
		cnt--;
	}
	CCCI_ERROR_LOG(md->index, TAG,
		"poll EE HS timeout, RCHNUM %d\n", md_info->channel_id);
}

static int md_cd_ee_handshake(struct ccci_modem *md, int timeout)
{
	/* seems sometime MD send D2H_EXCEPTION_INIT_DONE and
	 * D2H_EXCEPTION_CLEARQ_DONE together
	 */
	/*polling_ready(md_ctrl, D2H_EXCEPTION_INIT);*/
	md_cd_exception(md, HIF_EX_INIT);
	polling_ready(md, D2H_EXCEPTION_INIT_DONE);
	md_cd_exception(md, HIF_EX_INIT_DONE);

	polling_ready(md, D2H_EXCEPTION_CLEARQ_DONE);
	md_cd_exception(md, HIF_EX_CLEARQ_DONE);

	polling_ready(md, D2H_EXCEPTION_ALLQ_RESET);
	md_cd_exception(md, HIF_EX_ALLQ_RESET);

	return 0;
}

int md_fsm_exp_info(int md_id, unsigned int channel_id)
{
	struct ccci_modem *md;
	struct md_sys1_info *md_info;

	/*CCCI_DEBUG_LOG(0, TAG, "%s\n", __func__);*/
	md = ccci_md_get_modem_by_id(md_id);
	if (!md)
		return 0;
	if (channel_id & (1 << D2H_EXCEPTION_INIT)) {
		ccci_fsm_recv_md_interrupt(md->index, MD_IRQ_CCIF_EX);
		return 0;
	}
	md_info = (struct md_sys1_info *)md->private_data;
	md_info->channel_id = channel_id;

	if (md_info->channel_id & (1<<AP_MD_PEER_WAKEUP))
		__pm_wakeup_event(md_info->peer_wake_lock,
			jiffies_to_msecs(HZ));
	if (md_info->channel_id & (1<<AP_MD_SEQ_ERROR)) {
		CCCI_ERROR_LOG(md->index, TAG, "MD check seq fail\n");
		md->ops->dump_info(md, DUMP_FLAG_CCIF, NULL, 0);
	}

	if (md_info->channel_id & (1 << D2H_EXCEPTION_INIT)) {
		/* do not disable IRQ, as CCB still needs it */
		ccci_fsm_recv_md_interrupt(md->index, MD_IRQ_CCIF_EX);
	}

	return 0;
}
EXPORT_SYMBOL(md_fsm_exp_info);

static inline int md_sys1_sw_init(struct ccci_modem *md)
{
	int ret;

	CCCI_BOOTUP_LOG(md->index, TAG,
		"%s, MD_WDT IRQ(%d)\n", __func__, md->md_wdt_irq_id);

	ret = request_irq(md->md_wdt_irq_id, md_cd_wdt_isr,
			md->md_wdt_irq_flags, "MD_WDT", md);
	if (ret) {
		CCCI_ERROR_LOG(md->index, TAG,
			"request MD_WDT IRQ(%d) error %d\n",
			md->md_wdt_irq_id, ret);
		return ret;
	}
	ret = irq_set_irq_wake(md->md_wdt_irq_id, 1);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"irq_set_irq_wake MD_WDT IRQ(%d) error %d\n",
			md->md_wdt_irq_id, ret);
	/* IRQ is enabled after requested, so call enable_irq after
	 * request_irq will get a unbalance warning
	 */
	return 0;
}

static int md_cd_init(struct ccci_modem *md)
{
	CCCI_INIT_LOG(md->index, TAG, "CCCI: modem is initializing\n");

	return 0;
}

/* Please delete this function once it can be deleted. */
static int ccci_md_hif_start(struct ccci_modem *md, int stage)
{
	switch (stage) {
	case 1:
		/*enable clk: cldma*/
		if (md->hw_info->plat_ptr->set_clk_cg)
			md->hw_info->plat_ptr->set_clk_cg(md, 1);

		//ccci_hif_set_clk_cg(1 << CLDMA_HIF_ID, md->index, 1);

		//if (md->hif_flag & (1 << CLDMA_HIF_ID)) {
			/* 2. clearring buffer, just in case */
			//ccci_hif_clear_all_queue(1 << CLDMA_HIF_ID, OUT);
			//ccci_hif_clear_all_queue(1 << CLDMA_HIF_ID, IN);
			//if (md->hw_info->plat_ptr->cldma_hw_rst)
			//	md->hw_info->plat_ptr->cldma_hw_rst(md->index);
			//ccci_hif_hw_reset(1 << CLDMA_HIF_ID, md->index);
		//}
		break;

	case 2:
		if (md->hif_flag & (1 << CCIF_HIF_ID))
			ccif_enable_irq(md);
		//if (md->hif_flag & (1 << CLDMA_HIF_ID)) {
			/* 8. start CLDMA */
			//ccci_hif_start(CLDMA_HIF_ID);
		//}
		break;
	default:
		break;
	}
	return 0;
}

static int md_cd_start(struct ccci_modem *md)
{
	int ret = 0;

	if (md->per_md_data.config.setting & MD_SETTING_FIRST_BOOT) {
		if (md->hw_info->plat_ptr->remap_md_reg)
			md->hw_info->plat_ptr->remap_md_reg(md);
		md_sys1_sw_init(md);

		//ccci_hif_late_init(md->index, md->hif_flag);
		/* init security, as security depends on dummy_char,
		 * which is ready very late.
		 */
		ccci_init_security();
		ccci_md_clear_smem(md->index, 1);
		if (md->hw_info->plat_ptr->start_platform) {
			ret = md->hw_info->plat_ptr->start_platform(md);
			if (ret) {
				CCCI_BOOTUP_LOG(md->index, TAG,
					"power on MD BROM fail %d\n", ret);
				goto out;
			}
		}
		md->per_md_data.config.setting &= ~MD_SETTING_FIRST_BOOT;
	} else
		ccci_md_clear_smem(md->index, 0);

	CCCI_BOOTUP_LOG(md->index, TAG, "modem is starting\n");
	/* 1. load modem image */

	CCCI_BOOTUP_LOG(md->index, TAG,
		"modem image ready, bypass load\n");
	ret = ccci_get_md_check_hdr_inf(md->index,
		&md->per_md_data.img_info[IMG_MD],
		md->per_md_data.img_post_fix);
	if (ret < 0) {
		CCCI_BOOTUP_LOG(md->index, TAG,
			"partition read fail(%d)\n", ret);
		/* goto out; */
	} else
		CCCI_BOOTUP_LOG(md->index, TAG,
			"partition read success\n");

#ifdef FEATURE_BSI_BPI_SRAM_CFG
	ccci_set_bsi_bpi_SRAM_cfg(md, 1, MD_FLIGHT_MODE_NONE);
#endif
	ccci_md_hif_start(md, 1);
	/* 4. power on modem, do NOT touch MD register before this */
	if (md->hw_info->plat_ptr->power_on) {
		ret = md->hw_info->plat_ptr->power_on(md);
		if (ret) {
			CCCI_BOOTUP_LOG(md->index, TAG,
				"power on MD fail %d\n", ret);
			goto out;
		}
	}
#ifdef SET_EMI_STEP_BY_STAGE
	ccci_set_mem_access_protection_1st_stage(md);
#endif
	/* 5. update mutex */
	atomic_set(&md->reset_on_going, 0);

	md->per_md_data.md_dbg_dump_flag = MD_DBG_DUMP_AP_REG;

	/* 7. let modem go */
	if (md->hw_info->plat_ptr->let_md_go)
		md->hw_info->plat_ptr->let_md_go(md);
	wdt_enable_irq(md);
	ccci_md_hif_start(md, 2);

	md->per_md_data.is_in_ee_dump = 0;
	md->is_force_asserted = 0;
 out:
	CCCI_BOOTUP_LOG(md->index, TAG,
		"modem started %d\n", ret);
	/* used for throttling feature - start */
	/*ccci_modem_boot_count[md->index]++;*/
	/* used for throttling feature - end */
	return ret;
}

static int check_power_off_en(struct ccci_modem *md)
{
	int smem_val;
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDSS_DBG);

	if (md->index != MD_SYS1)
		return 1;

	smem_val = *((int *)((long)mdss_dbg->base_ap_view_vir +
		md->hw_info->plat_val->offset_epof_md1));
	CCCI_NORMAL_LOG(md->index, TAG,
		"share for power off:%x\n", smem_val);
	if (smem_val != 0) {
		CCCI_NORMAL_LOG(md->index, TAG,
			"[ccci]enable power off check\n");
		return 1;
	}
	CCCI_NORMAL_LOG(md->index, TAG,
		"disable power off check\n");
	return 0;
}

static int md_cd_soft_start(struct ccci_modem *md, unsigned int mode)
{
	if (md->hw_info->plat_ptr->soft_power_on == NULL)
		return -1;
	return md->hw_info->plat_ptr->soft_power_on(md, mode);
}

static int md_cd_soft_stop(struct ccci_modem *md, unsigned int mode)
{
	if (md->hw_info->plat_ptr->soft_power_off == NULL)
		return -1;
	return md->hw_info->plat_ptr->soft_power_off(md, mode);
}

void __weak md1_sleep_timeout_proc(void)
{
	CCCI_DEBUG_LOG(-1, TAG, "No %s\n", __func__);
}

static int md_cd_pre_stop(struct ccci_modem *md, unsigned int stop_type)
{
	u32 pending;
	struct ccci_smem_region *mdccci_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDCCCI_DBG);
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDSS_DBG);
	struct ccci_per_md *per_md_data =
		ccci_get_per_md_data(md->index);
	int md_dbg_dump_flag = per_md_data->md_dbg_dump_flag;

	/* 1. mutex check */
	if (atomic_add_return(1, &md->reset_on_going) > 1) {
		CCCI_NORMAL_LOG(md->index, TAG,
			"One reset flow is on-going\n");
		return -CCCI_ERR_MD_IN_RESET;
	}

	CCCI_NORMAL_LOG(md->index, TAG,
		"%s: CCCI modem is resetting\n", __func__);
	/* 2. disable WDT IRQ */
	wdt_disable_irq(md);

	/* only debug in Flight mode */
	if (stop_type == MD_FLIGHT_MODE_ENTER) {
		debug_in_flight_mode(md);
#ifdef CCCI_KMODULE_ENABLE
		pending = 0;
#else
		pending = mt_irq_get_pending(md->md_wdt_irq_id);
#endif
		if (pending) {
			CCCI_NORMAL_LOG(md->index, TAG, "WDT IRQ occur.");
			CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD EX log\n");
			if (md_dbg_dump_flag & (1 << MD_DBG_DUMP_SMEM)) {
				ccci_util_mem_dump(md->index,
					CCCI_DUMP_MEM_DUMP,
					mdccci_dbg->base_ap_view_vir,
					mdccci_dbg->size);
				ccci_util_mem_dump(md->index,
					CCCI_DUMP_MEM_DUMP,
					mdss_dbg->base_ap_view_vir,
					mdss_dbg->size);
			}
			if (md->hw_info->plat_ptr->debug_reg)
				md->hw_info->plat_ptr->debug_reg(md);
			/* cldma_dump_register(CLDMA_HIF_ID);*/
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
			aed_md_exception_api(NULL, 0, NULL, 0,
				"WDT IRQ occur.", DB_OPT_DEFAULT);
#endif
		}
	}

	CCCI_NORMAL_LOG(md->index, TAG, "Reset when MD state: %d\n",
			ccci_fsm_get_md_state(md->index));
	return 0;
}
static void debug_in_flight_mode(struct ccci_modem *md)
{
	int count = 0;
	int en_power_check = check_power_off_en(md);
	struct ccci_smem_region *mdccci_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDCCCI_DBG);
	struct ccci_smem_region *mdss_dbg =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_MDSS_DBG);
	struct ccci_per_md *per_md_data =
		ccci_get_per_md_data(md->index);
	int md_dbg_dump_flag = per_md_data->md_dbg_dump_flag;

	count = 5;
	while (spm_is_md1_sleep() == 0) {
		count--;
		if (count == 0) {
			if (en_power_check) {
				CCCI_NORMAL_LOG(md->index, TAG,
				"MD is not in sleep mode, dump md status!\n");
				CCCI_MEM_LOG_TAG(md->index, TAG,
				"Dump MD EX log\n");
				if (md_dbg_dump_flag &
					(1 << MD_DBG_DUMP_SMEM)) {
					ccci_util_mem_dump(md->index,
					CCCI_DUMP_MEM_DUMP,
					mdccci_dbg->base_ap_view_vir,
					mdccci_dbg->size);
					ccci_util_mem_dump(md->index,
					CCCI_DUMP_MEM_DUMP,
					mdss_dbg->base_ap_view_vir,
					mdss_dbg->size);
				}
				if (md->hw_info->plat_ptr->debug_reg)
					md->hw_info->plat_ptr->debug_reg(md);
	/* cldma_dump_register(CLDMA_HIF_ID);*/
#if IS_ENABLED(CONFIG_MTK_AEE_FEATURE)
				aed_md_exception_api(
				mdss_dbg->base_ap_view_vir,
				mdss_dbg->size, NULL, 0,
				"After AP send EPOF, MD didn't go to sleep in 4 seconds.",
				DB_OPT_DEFAULT);
#endif
			} else
				md1_sleep_timeout_proc();
			break;
		}
		msleep(1000);
	}
}
static int md_cd_stop(struct ccci_modem *md, unsigned int stop_type)
{
	int ret = 0;

	CCCI_NORMAL_LOG(md->index, TAG,
		"modem is power off, stop_type=%d\n", stop_type);
	ccif_disable_irq(md);
	/* Check EMI before */
	if (md->hw_info->plat_ptr->check_emi_state)
		md->hw_info->plat_ptr->check_emi_state(md, 1);

	/* power off MD */
	if (md->hw_info->plat_ptr->power_off)
		ret = md->hw_info->plat_ptr->power_off(md,
			stop_type == MD_FLIGHT_MODE_ENTER ? 100 : 0);
	CCCI_NORMAL_LOG(md->index, TAG,
		"modem is power off done, %d\n", ret);

	if (md->hif_flag & (1<<CLDMA_HIF_ID)) {
		ccci_hif_clear(1 << CLDMA_HIF_ID);
		ccci_hif_stop(CLDMA_HIF_ID);
		ccci_hif_hw_reset(1 << CLDMA_HIF_ID, md->index);
		ccci_hif_set_clk_cg(1 << CLDMA_HIF_ID, md->index, 0);
	}
	/* Check EMI after */
	if (md->hw_info->plat_ptr->check_emi_state)
		md->hw_info->plat_ptr->check_emi_state(md, 0);

	/*disable cldma & ccif clk*/
	if (md->hw_info->plat_ptr->set_clk_cg)
		md->hw_info->plat_ptr->set_clk_cg(md, 0);

#ifdef FEATURE_BSI_BPI_SRAM_CFG
	ccci_set_bsi_bpi_SRAM_cfg(md, 0, stop_type);
#endif

	return 0;
}

static void dump_runtime_data_v2(struct ccci_modem *md,
	struct ap_query_md_feature *ap_feature)
{
	u8 i = 0;

	CCCI_BOOTUP_LOG(md->index, TAG,
		"head_pattern 0x%x\n", ap_feature->head_pattern);

	for (i = BOOT_INFO; i < AP_RUNTIME_FEATURE_ID_MAX; i++) {
		CCCI_BOOTUP_LOG(md->index, TAG,
			"feature %u: mask %u, version %u\n",
			i, ap_feature->feature_set[i].support_mask,
			ap_feature->feature_set[i].version);
	}
	CCCI_BOOTUP_LOG(md->index, TAG,
		"share_memory_support 0x%x\n",
		ap_feature->share_memory_support);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"ap_runtime_data_addr 0x%x\n",
		ap_feature->ap_runtime_data_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"ap_runtime_data_size 0x%x\n",
		ap_feature->ap_runtime_data_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"md_runtime_data_addr 0x%x\n",
		ap_feature->md_runtime_data_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"md_runtime_data_size 0x%x\n",
		ap_feature->md_runtime_data_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_start_addr 0x%x\n",
		ap_feature->set_md_mpu_start_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_total_size 0x%x\n",
		ap_feature->set_md_mpu_total_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"tail_pattern 0x%x\n",
		ap_feature->tail_pattern);
}

static void dump_runtime_data_v2_1(struct ccci_modem *md,
	struct ap_query_md_feature_v2_1 *ap_feature)
{
	u8 i = 0;

	CCCI_BOOTUP_LOG(md->index, TAG,
		"head_pattern 0x%x\n", ap_feature->head_pattern);

	for (i = BOOT_INFO; i < AP_RUNTIME_FEATURE_ID_MAX; i++) {
		CCCI_BOOTUP_LOG(md->index, TAG,
			"feature %u: mask %u, version %u\n",
			i, ap_feature->feature_set[i].support_mask,
			ap_feature->feature_set[i].version);
	}
	CCCI_BOOTUP_LOG(md->index, TAG,
		"share_memory_support 0x%x\n",
		ap_feature->share_memory_support);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"ap_runtime_data_addr 0x%x\n",
		ap_feature->ap_runtime_data_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"ap_runtime_data_size 0x%x\n",
		ap_feature->ap_runtime_data_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"md_runtime_data_addr 0x%x\n",
		ap_feature->md_runtime_data_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"md_runtime_data_size 0x%x\n",
		ap_feature->md_runtime_data_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_noncached_start_addr 0x%x\n",
		ap_feature->noncached_mpu_start_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_noncached_total_size 0x%x\n",
		ap_feature->noncached_mpu_total_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_cached_start_addr 0x%x\n",
		ap_feature->cached_mpu_start_addr);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"set_md_mpu_cached_total_size 0x%x\n",
		ap_feature->cached_mpu_total_size);
	CCCI_BOOTUP_LOG(md->index, TAG,
		"tail_pattern 0x%x\n",
		ap_feature->tail_pattern);
}

#if (MD_GENERATION < 6297)
static void md_cd_smem_sub_region_init(struct ccci_modem *md)
{
	int i;
	int __iomem *addr;
	struct ccci_smem_region *dbm =
		ccci_md_get_smem_by_user_id(md->index, SMEM_USER_RAW_DBM);

	/* Region 0, dbm */
	addr = (int __iomem *)dbm->base_ap_view_vir;
	addr[0] = 0x44444444; /* Guard pattern 1 header */
	addr[1] = 0x44444444; /* Guard pattern 2 header */

#ifdef DISABLE_PBM_FEATURE
	for (i = 2; i < (CCCI_SMEM_SIZE_DBM / 4 + 2); i++)
		addr[i] = 0xFFFFFFFF;
#else
	for (i = 2; i < (CCCI_SMEM_SIZE_DBM / 4 + 2); i++)
		addr[i] = 0x00000000;
#endif

	addr[i++] = 0x44444444; /* Guard pattern 1 tail */
	addr[i++] = 0x44444444; /* Guard pattern 2 tail */

#if IS_ENABLED(CONFIG_MTK_PBM)
	addr += CCCI_SMEM_SIZE_DBM_GUARD;
#endif
	init_md_section_level(KR_MD1, addr);
}
#else
static void md_cd_smem_sub_region_init(struct ccci_modem *md)
{
#if IS_ENABLED(CONFIG_MTK_PBM)
	int __iomem *addr;
	struct ccci_smem_region *dbm =
		ccci_md_get_smem_by_user_id(md->index, SMEM_USER_RAW_DBM);

	/* Region 0, dbm */
	addr = (int __iomem *)dbm->base_ap_view_vir;
	init_md_section_level(KR_MD1, addr);
#endif
}
#endif

static void config_ap_runtime_data_v2(struct ccci_modem *md,
	struct ap_query_md_feature *ap_feature)
{
	struct ccci_smem_region *runtime_data =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_RUNTIME_DATA);

	ap_feature->head_pattern = AP_FEATURE_QUERY_PATTERN;
	/*AP query MD feature set */

	ap_feature->share_memory_support = INTERNAL_MODEM;
	ap_feature->ap_runtime_data_addr = runtime_data->base_md_view_phy;
	ap_feature->ap_runtime_data_size = CCCI_SMEM_SIZE_RUNTIME_AP;
	ap_feature->md_runtime_data_addr =
		ap_feature->ap_runtime_data_addr + CCCI_SMEM_SIZE_RUNTIME_AP;
	ap_feature->md_runtime_data_size = CCCI_SMEM_SIZE_RUNTIME_MD;

	ap_feature->set_md_mpu_start_addr =
		md->mem_layout.md_bank4_noncacheable_total.base_md_view_phy;
	ap_feature->set_md_mpu_total_size =
		md->mem_layout.md_bank4_noncacheable_total.size;

	/* Set Flag for modem on feature_set[1].version,
	 * specially: [1].support_mask = 0
	 */
	ap_feature->feature_set[1].support_mask = 0;
	/* ver.1: set_md_mpu_total_size =
	 * ap md1 share + md1&md3 share
	 */
	/* ver.0: set_md_mpu_total_size = ap md1 share */
	ap_feature->feature_set[1].version = 1;
	ap_feature->tail_pattern = AP_FEATURE_QUERY_PATTERN;
}

static void config_ap_runtime_data_v2_1(struct ccci_modem *md,
	struct ap_query_md_feature_v2_1 *ap_feature)
{
	struct ccci_smem_region *runtime_data =
		ccci_md_get_smem_by_user_id(md->index,
			SMEM_USER_RAW_RUNTIME_DATA);

	ap_feature->head_pattern = AP_FEATURE_QUERY_PATTERN;
	/*AP query MD feature set */

	/* to let md know that this is new AP. */
	ap_feature->share_memory_support = MULTI_MD_MPU_SUPPORT;
	ap_feature->ap_runtime_data_addr = runtime_data->base_md_view_phy;
	ap_feature->ap_runtime_data_size = CCCI_SMEM_SIZE_RUNTIME_AP;
	ap_feature->md_runtime_data_addr =
		ap_feature->ap_runtime_data_addr + CCCI_SMEM_SIZE_RUNTIME_AP;
	ap_feature->md_runtime_data_size = CCCI_SMEM_SIZE_RUNTIME_MD;

	ap_feature->noncached_mpu_start_addr =
		md->mem_layout.md_bank4_noncacheable_total.base_md_view_phy;
	ap_feature->noncached_mpu_total_size =
		md->mem_layout.md_bank4_noncacheable_total.size;
	ap_feature->cached_mpu_start_addr =
		md->mem_layout.md_bank4_cacheable_total.base_md_view_phy;
	ap_feature->cached_mpu_total_size =
		md->mem_layout.md_bank4_cacheable_total.size;

	/* Set Flag for modem on feature_set[1].version,
	 * specially: [1].support_mask = 0
	 */
	ap_feature->feature_set[1].support_mask = 0;
	/* ver.1: set_md_mpu_total_size =
	 * ap md1 share + md1&md3 share
	 */
	/* ver.0: set_md_mpu_total_size = ap md1 share */
	ap_feature->feature_set[1].version = 1;
	ap_feature->tail_pattern = AP_FEATURE_QUERY_PATTERN;
}

static int md_cd_send_runtime_data_v2(struct ccci_modem *md,
	unsigned int tx_ch, unsigned int txqno, int skb_from_pool)
{
	int packet_size;
	struct ap_query_md_feature *ap_rt_data = NULL;
	struct ap_query_md_feature_v2_1 *ap_rt_data_v2_1 = NULL;
	int ret;

	if (md->runtime_version < AP_MD_HS_V2) {
		CCCI_ERROR_LOG(md->index, TAG,
			"unsupported runtime version %d\n",
			md->runtime_version);
		return -CCCI_ERR_CCIF_INVALID_RUNTIME_LEN;
	}

	if (md->multi_md_mpu_support) {
		packet_size = sizeof(struct ap_query_md_feature_v2_1) +
			sizeof(struct ccci_header);

		ap_rt_data_v2_1 = (struct ap_query_md_feature_v2_1 *)
			ccci_hif_fill_rt_header(CCIF_HIF_ID, packet_size,
			tx_ch, txqno);
		if (!ap_rt_data_v2_1) {
			CCCI_ERROR_LOG(md->index, TAG, "rt header v2 NULL!\n");
			return -1;
		}

		memset_io(ap_rt_data_v2_1, 0,
			sizeof(struct ap_query_md_feature_v2_1));
		config_ap_runtime_data_v2_1(md, ap_rt_data_v2_1);
		dump_runtime_data_v2_1(md, ap_rt_data_v2_1);
	} else {
		/* infactly, 6292 should not be this else condition */
		packet_size = sizeof(struct ap_query_md_feature) +
			sizeof(struct ccci_header);
		ap_rt_data = (struct ap_query_md_feature *)
			ccci_hif_fill_rt_header(CCIF_HIF_ID, packet_size,
			tx_ch, txqno);
		if (!ap_rt_data) {
			CCCI_ERROR_LOG(md->index, TAG, "rt header NULL!\n");
			return -1;
		}
		memset_io(ap_rt_data, 0, sizeof(struct ap_query_md_feature));
		config_ap_runtime_data_v2(md, ap_rt_data);
		dump_runtime_data_v2(md, ap_rt_data);
	}

	md_cd_smem_sub_region_init(md);

	ret = ccci_hif_send_data(CCIF_HIF_ID, H2D_SRAM);
	return ret;
}

static int md_cd_force_assert(struct ccci_modem *md, enum MD_COMM_TYPE type)
{
	CCCI_NORMAL_LOG(md->index, TAG,
		"force assert MD using %d\n", type);
	if (type == CCIF_INTERRUPT)
		ccci_hif_send_data(CCIF_HIF_ID, AP_MD_SEQ_ERROR);
	else if (type == CCIF_MPU_INTR) {
		ccci_hif_send_data(CCIF_HIF_ID, H2D_MPU_FORCE_ASSERT);
		md->ops->dump_info(md, DUMP_FLAG_CCIF_REG, NULL, 0);
	}
	return 0;
}

static int md_cd_dump_info(struct ccci_modem *md,
	enum MODEM_DUMP_FLAG flag, void *buff, int length)
{

	if (flag & DUMP_FLAG_CCIF_REG) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump CCIF REG\n");
		ccci_hif_dump_status(CCIF_HIF_ID, DUMP_FLAG_CCIF_REG,
			NULL, -1);
	}
	if (flag & DUMP_FLAG_PCCIF_REG) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "ignore Dump PCCIF REG\n");
		/*md_cd_dump_pccif_reg(md);*/
	}
	if (flag & DUMP_FLAG_CCIF) {
		unsigned int *dest_buff = NULL;
		unsigned char ccif_sram[CCCI_EE_SIZE_CCIF_SRAM] = { 0 };

		if (buff)
			dest_buff = (unsigned int *)buff;
		else
			dest_buff = (unsigned int *)ccif_sram;
		if (length < sizeof(ccif_sram) && length > 0) {
			CCCI_ERROR_LOG(md->index, TAG,
				"dump CCIF SRAM length illegal %d/%zu\n",
				length, sizeof(ccif_sram));
			dest_buff = (unsigned int *)ccif_sram;
		} else {
			length = sizeof(ccif_sram);
		}

		ccci_hif_dump_status(1 << CCIF_HIF_ID, DUMP_FLAG_CCIF,
			buff, length);
	}

	/*HIF related dump flag*/
	if (flag & (DUMP_FLAG_QUEUE_0_1 | DUMP_FLAG_QUEUE_0 |
		DUMP_FLAG_IRQ_STATUS | DUMP_FLAG_CLDMA))
		ccci_hif_dump_status(md->hif_flag, flag, NULL, length);

	if ((flag & DUMP_FLAG_REG) && md->hw_info->plat_ptr->debug_reg)
		md->hw_info->plat_ptr->debug_reg(md);
	if (flag & DUMP_FLAG_SMEM_EXP) {
		struct ccci_smem_region *mdccci_dbg =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_RAW_MDCCCI_DBG);
		struct ccci_smem_region *mdss_dbg =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_RAW_MDSS_DBG);

		CCCI_MEM_LOG_TAG(md->index, TAG,
			"Dump exception share memory\n");
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP,
			mdccci_dbg->base_ap_view_vir, mdccci_dbg->size);
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP,
			mdss_dbg->base_ap_view_vir, mdss_dbg->size);
	}
	if (flag & DUMP_FLAG_SMEM_CCISM) {
		struct ccci_smem_region *scp =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_CCISM_SCP);

		CCCI_MEM_LOG_TAG(md->index, TAG,
			"Dump CCISM share memory\n");
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP,
			scp->base_ap_view_vir, scp->size);
	}
	if (flag & DUMP_FLAG_SMEM_CCB_CTRL) {
		struct ccci_smem_region *ccb_ctl =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_RAW_CCB_CTRL);

		if (ccb_ctl) {
			CCCI_MEM_LOG_TAG(md->index, TAG,
				"Dump CCB CTRL share memory\n");
			ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP,
				ccb_ctl->base_ap_view_vir,
				32 * ccb_configs_len * 2);
		}
	}
	if (flag & DUMP_FLAG_SMEM_CCB_DATA) {
		int i, j;
		unsigned char *curr_ch_p = NULL;
		unsigned int *curr_p = NULL;
		struct ccci_smem_region *ccb_data =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_CCB_START);
		struct ccci_smem_region *dhl_raw =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_RAW_DHL);

		if (ccb_data && ccb_data->size != 0) {
			CCCI_MEM_LOG_TAG(md->index, TAG,
				"Dump CCB DATA share memory\n");
			curr_ch_p = ccb_data->base_ap_view_vir;
			curr_p = (unsigned int *)curr_ch_p;
			for (i = 0; i < ccb_configs_len; i++) {
				/* dump dl buffer */
				for (j = 0; j < ccb_configs[i].dl_buff_size /
					ccb_configs[i].dl_page_size;  j++) {
					ccci_dump_write(md->index,
						CCCI_DUMP_MEM_DUMP, 0,
						"ul_buf%2d-page%2d %p: %08X %08X %08X %08X\n",
						i, j, curr_p,
						*curr_p, *(curr_p + 1),
						*(curr_p + 2), *(curr_p + 3));

					curr_ch_p +=
						ccb_configs[i].dl_page_size;
					curr_p = (unsigned int *)curr_ch_p;
				}

				/* dump ul buffer */
				for (j = 0; j < ccb_configs[i].ul_buff_size /
					ccb_configs[i].ul_page_size; j++) {
					ccci_dump_write(md->index,
						CCCI_DUMP_MEM_DUMP, 0,
						"dl_buf%2d-page%2d %p: %08X %08X %08X %08X\n",
						i, j, curr_p,
						*curr_p, *(curr_p + 1),
						*(curr_p + 2), *(curr_p + 3));

					curr_ch_p +=
						ccb_configs[i].ul_page_size;
					curr_p = (unsigned int *)curr_ch_p;
				}
			}
		}
		if (dhl_raw && dhl_raw->size) {
			CCCI_MEM_LOG_TAG(md->index, TAG,
				"Dump DHL RAW share memory\n");
			curr_ch_p = dhl_raw->base_ap_view_vir;
			curr_p = (unsigned int *)curr_ch_p;
			ccci_dump_write(md->index,
				CCCI_DUMP_MEM_DUMP, 0,
					"%p: %08X %08X %08X %08X\n",
					curr_p, *curr_p, *(curr_p + 1),
					*(curr_p + 2), *(curr_p + 3));
		}
	}

	if (flag & DUMP_FLAG_LAYOUT) {
		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD layout struct\n");
		ccci_util_mem_dump(md->index, CCCI_DUMP_MEM_DUMP,
			&md->mem_layout, sizeof(struct ccci_mem_layout));
	}

	if (flag & DUMP_FLAG_SMEM_MDSLP) {
		struct ccci_smem_region *low_pwr =
			ccci_md_get_smem_by_user_id(md->index,
				SMEM_USER_RAW_DBM);

		CCCI_MEM_LOG_TAG(md->index, TAG, "Dump MD SLP registers\n");
		ccci_cmpt_mem_dump(md->index, low_pwr->base_ap_view_vir,
			low_pwr->size);
	}
	if (flag & DUMP_FLAG_MD_WDT) {
		CCCI_MEM_LOG_TAG(md->index, TAG,
			"wdt_enabled=%d\n", atomic_read(&md->wdt_enabled));
#if IS_ENABLED(CONFIG_MTK_IRQ_DBG)
		CCCI_NORMAL_LOG(md->index, TAG, "Dump md WDT IRQ status\n");
		mt_irq_dump_status(md->md_wdt_irq_id);
#endif
	}

	if ((flag & DUMP_MD_BOOTUP_STATUS) &&
		md->hw_info->plat_ptr->get_md_bootup_status)
		md->hw_info->plat_ptr->get_md_bootup_status((unsigned int *)buff,
			length);

	return length;
}

static int md_cd_ee_callback(struct ccci_modem *md, enum MODEM_EE_FLAG flag)
{
	if (flag & EE_FLAG_ENABLE_WDT)
		wdt_enable_irq(md);
	if (flag & EE_FLAG_DISABLE_WDT)
		wdt_disable_irq(md);
	return 0;
}

static int md_cd_send_ccb_tx_notify(struct ccci_modem *md, int core_id)
{
	/* CCCI_NORMAL_LOG(md->index, TAG,
	 * "ccb tx notify to core %d\n", core_id);
	 */
	switch (core_id) {
	case P_CORE:
		ccci_hif_send_data(CCIF_HIF_ID, AP_MD_CCB_WAKEUP);
		break;
	case VOLTE_CORE:
	default:
		break;
	}
	return 0;
}

static struct ccci_modem_ops md_cd_ops = {
	.init = &md_cd_init,
	.start = &md_cd_start,
	.stop = &md_cd_stop,
	.soft_start = &md_cd_soft_start,
	.soft_stop = &md_cd_soft_stop,
	.pre_stop = &md_cd_pre_stop,
	.send_runtime_data = &md_cd_send_runtime_data_v2,
	.ee_handshake = &md_cd_ee_handshake,
	.force_assert = &md_cd_force_assert,
	.dump_info = &md_cd_dump_info,
	.ee_callback = &md_cd_ee_callback,
	.send_ccb_tx_notify = &md_cd_send_ccb_tx_notify,
};

/* ===== PEARL-MDLOG-BEGIN (generated by patch-mdlog.py) ===== */
/* PEARL: MODEM 日志使能（mdlogger / TST 握手）用的最小 CCCI 发送点。
 *
 * 握手消息要发在 CCCI_MD_LOG_TX(43) 上。用户态 /dev/ttyC1 的 write 在
 * md_state == BOOT_WAITING_FOR_HS1/HS2 时被 port_dev_write() 拒绝
 * (-ENODEV，只有 FS/RPC 端口例外)，而那正是基带活着、需要握手的时间窗；
 * 这里改用 ccci_port_send_msg_to_md()，它允许 HS1/HS2/READY/EXCEPTION。
 *
 * 用法（root，mdsys1 设备的 sysfs 目录下）：
 *   echo "12 0" > .../mdlog              # 发 msg 0x0C  reserved 0
 *   echo "0x2 0xffffffff" > .../mdlog    # 发 msg 0x02  reserved 0xffffffff
 *   echo "10 0 1" > .../mdlog            # 第三个数 = blocking
 *   cat .../mdlog                        # 看上一次发送的返回值/md_state
 */
/* PEARL-MDLOG: 自动首发开关。默认 0（关闭），要把 60 条 0x0C 自动塞进
 * 基带启动窗口时，由用户态先 arm：
 *   echo 1 > /sys/module/ccci_md_all/parameters/pearl_mdlog_auto
 * （写 module parameter 不会 open /dev/ttyC1，因此不会推迟基带启动） */
/* 自动握手开关的编译期默认值（moved above first use）。
 *
 * 第 31 轮实验 M（2026-09-20）结论：默认必须为 0。
 *   auto=1（在 HS1/HS2 窗口里每 80ms 连发 0x0C，最多 30+20 发）会让基带
 *   在 HS1+5.43~5.48s **确定性地**死掉（4/4 次一致），断言在
 *   mcu/driver/ccismcore/src/ccismcore_ccci.c:1326（MD 侧 CCCI 驱动）。
 *   auto=0 时失败回到开轮之前的 A/B 随机形态（HS1+0.42s / digrf_iomux.c:496，
 *   或 HS1+43.5s 的 HS2 超时），AMMS/DRDI 请求也照常工作（257 行日志）。
 * 真正的 mdlogger 是"发一次 0x0C 然后等回复"，不是洪泛；要在启动窗口抓 MD 日志
 * 必须改成单发+等待，或挪到 HS2 稳定之后再发。
 * 手动触发仍可用：echo 1 > /sys/module/ccci_md_all/parameters/pearl_mdlog_auto
 */
#ifndef PEARL_MDLOG_AUTO_DEF
#define PEARL_MDLOG_AUTO_DEF 0
#endif

static unsigned int pearl_mdlog_auto = PEARL_MDLOG_AUTO_DEF;
module_param(pearl_mdlog_auto, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_auto,
	"PEARL: auto-send the mdlog step1 (0x0C) message when the MD reaches HS1");

static unsigned int pearl_mdlog_last_msg;
static unsigned int pearl_mdlog_last_resv;
static int pearl_mdlog_last_ret = 0x7fffffff;
static int pearl_mdlog_last_state = -1;
static unsigned int pearl_mdlog_send_cnt;

/* PEARL-MDLOG-EXC: mdlog 用的 TX 通道。43 = CCCI_MD_LOG_TX(/dev/ttyC1,
 * mdlog 数据口)；8 = CCCI_UART1_TX(/dev/ccci_md_log_ctrl, META/控制口)。
 * 厂商在 EXCEPTION 期间同时放行这两条；我们在 43 上从没收到过回复，
 * 所以留一个开关去试 8。
 * 注意：本文件里 pearl_mdlog_send 出现在参数块之前，声明必须放在这里。
 */
static unsigned int pearl_mdlog_tx_ch = 43;
module_param(pearl_mdlog_tx_ch, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_tx_ch,
	"PEARL: TX channel for the mdlog messages (43=MD_LOG_TX, 8=UART1_TX)");

int pearl_mdlog_send(unsigned int msg, unsigned int resv, int blocking,
	const char *why)
{
	int ret;

	ret = ccci_port_send_msg_to_md(0 /* MD_SYS1 */, pearl_mdlog_tx_ch,
		msg, resv, blocking);
	pearl_mdlog_last_msg = msg;
	pearl_mdlog_last_resv = resv;
	pearl_mdlog_last_ret = ret;
	pearl_mdlog_last_state = ccci_fsm_get_md_state(0);
	pearl_mdlog_send_cnt++;
	CCCI_ERROR_LOG(0, TAG,
		"PEARL-MDLOG[%s]: ch=%u msg=0x%02x resv=0x%x blocking=%d ret=%d state=%d n=%u\n",
		why, pearl_mdlog_tx_ch, msg, resv, blocking, ret,
		pearl_mdlog_last_state, pearl_mdlog_send_cnt);
	return ret;
}

static ssize_t md_cd_mdlog_show(struct ccci_modem *md, char *buf)
{
	return snprintf(buf, 320,
		"usage: echo '<msg> [reserved] [blocking]' > mdlog\n"
		"ch=%d (CCCI_MD_LOG_TX) md_state=%d\n"
		"msg: 0x0C=step1 0x02=step2 0x08=start(SD) 0x09=start(USB) 0x0A=step3\n"
		"last: msg=0x%02x resv=0x%x ret=%d state=%d count=%u\n",
		CCCI_MD_LOG_TX, ccci_fsm_get_md_state(md->index),
		pearl_mdlog_last_msg, pearl_mdlog_last_resv,
		pearl_mdlog_last_ret, pearl_mdlog_last_state,
		pearl_mdlog_send_cnt);
}

static ssize_t md_cd_mdlog_store(struct ccci_modem *md,
	const char *buf, size_t count)
{
	unsigned int msg = 0, resv = 0;
	int blocking = 0, n;

	n = sscanf(buf, "%i %i %i", &msg, &resv, &blocking);
	if (n < 1) {
		CCCI_ERROR_LOG(md->index, TAG,
			"PEARL-MDLOG: bad input (need 'msg [resv] [blocking]')\n");
		return -EINVAL;
	}
	pearl_mdlog_send(msg, resv, blocking, "sysfs");
	return count;
}

/* 只有在 pearl_mdlog_auto 打开时才发（给 AMMS 之类的自动触发点用） */
int pearl_mdlog_send_armed(unsigned int msg, unsigned int resv, int blocking,
	const char *why)
{
	if (!pearl_mdlog_auto)
		return -1;
	return pearl_mdlog_send(msg, resv, blocking, why);
}

/* PEARL-MDLOG: 自动首发（受 pearl_mdlog_auto 开关约束，默认关闭）。
 *
 * 基带只在 md_state == BOOT_WAITING_FOR_HS1/HS2 的短窗口里活着等这次握手
 * （实测干净冷启动：0->2 在 7.659s，2->3 在 7.907s，3->5 在 13.123s），
 * 用户态按 100 ms 轮询都可能错过，所以状态一变到 HS1 就由内核直接连发。
 */
/* PEARL-MDLOG: 旧 kick work 已被第 31 轮状态机取代 */
/* PEARL-MDLOG: 旧 pearl_mdlog_kick 已被第 31 轮版本取代 */
/* ===== PEARL-MDLOG-END ===== */

/* ===== PEARL-MDLOGRX-BEGIN (patch-mdlog-rx.py) ===== */
/* 自动握手开关的编译期默认值见文件上方（已前移）*/

/*
 * PEARL 第 31 轮：mdlog 接收缓冲区。
 * 存的是"原样"的 16 字节 CCCI 头 + 负载，可由 sysfs 只读拖出来。
 */
#define PEARL_MDLOGRX_SZ		(256 * 1024)
#define PEARL_MDLOGRX_MAXREC	8192

static unsigned char *pearl_mdlogrx_buf;
static DEFINE_SPINLOCK(pearl_mdlogrx_lock);
static unsigned int pearl_mdlogrx_head;	/* 累计写入字节 */
static unsigned int pearl_mdlogrx_pkts;
static unsigned int pearl_mdlogrx_lost;
static unsigned int pearl_mdlogrx_last_ch = 0xffff;
static unsigned int pearl_mdlogrx_last_data0;
static unsigned int pearl_mdlogrx_last_data1;
static unsigned int pearl_mdlogrx_last_resv;
static unsigned long long pearl_mdlogrx_last_ts;

struct pearl_mdlog_msg {
	unsigned int ch;
	unsigned int data0;
	unsigned int data1;
	unsigned int resv;
	unsigned int payload_len;
	unsigned long long ts;
	unsigned char payload[64];
};
#define PEARL_MDLOG_MSG_MAX	96
static struct pearl_mdlog_msg pearl_mdlog_msgs[PEARL_MDLOG_MSG_MAX];
static unsigned int pearl_mdlog_msg_n;
static DEFINE_SPINLOCK(pearl_mdlog_msg_lock);

static void pearl_mdlog_note(unsigned int ch, unsigned int d0, unsigned int d1,
	unsigned int resv, const unsigned char *pl, unsigned int plen)
{
	unsigned long flags;
	struct pearl_mdlog_msg *m;

	if (plen > sizeof(m->payload))
		plen = sizeof(m->payload);
	spin_lock_irqsave(&pearl_mdlog_msg_lock, flags);
	m = &pearl_mdlog_msgs[pearl_mdlog_msg_n % PEARL_MDLOG_MSG_MAX];
	m->ch = ch;
	m->data0 = d0;
	m->data1 = d1;
	m->resv = resv;
	m->payload_len = plen;
	m->ts = local_clock();
	if (pl)
		memcpy(m->payload, pl, plen);
	pearl_mdlog_msg_n++;
	spin_unlock_irqrestore(&pearl_mdlog_msg_lock, flags);
}

/* 在内核里被 proxy_dispatch_recv_skb() 调用（可能是 softirq 上下文，不能睡） */
void pearl_mdlogrx_capture(struct sk_buff *skb, unsigned int hif_id)
{
	struct ccci_header *h;
	unsigned long flags;
	unsigned int len, off, head, first, room, i;
	unsigned char *pl;
	static unsigned int hook_n;

	if (!skb || skb->len < sizeof(struct ccci_header))
		return;
	h = (struct ccci_header *)skb->data;
	if (h->channel != 42 && h->channel != 43) {
		/* 诊断：确认钩子真的被执行到（前 4 次任意通道） */
		if (hook_n < 4) {
			hook_n++;
			CCCI_ERROR_LOG(0, TAG,
				"PEARL-MDLOGRX-HOOK: seen ch=%u hif=%u len=%u d0=0x%08x d1=0x%08x (hook_n=%u)\n",
				h->channel, hif_id, skb->len, h->data[0],
				h->data[1], hook_n);
		}
		return;
	}
	if (!pearl_mdlogrx_buf)
		return;

	len = skb->len;
	if (len > PEARL_MDLOGRX_MAXREC)
		len = PEARL_MDLOGRX_MAXREC;
	pl = skb->data + sizeof(struct ccci_header);
	pearl_mdlog_note(h->channel, h->data[0], h->data[1], h->reserved,
		pl, (len > sizeof(struct ccci_header)) ?
		(len - sizeof(struct ccci_header)) : 0);

	spin_lock_irqsave(&pearl_mdlogrx_lock, flags);
	off = pearl_mdlogrx_head & (PEARL_MDLOGRX_SZ - 1);
	first = PEARL_MDLOGRX_SZ - off;
	if (first > len)
		first = len;
	memcpy(pearl_mdlogrx_buf + off, skb->data, first);
	if (first < len)
		memcpy(pearl_mdlogrx_buf, skb->data + first, len - first);
	pearl_mdlogrx_head += len;
	pearl_mdlogrx_pkts++;
	pearl_mdlogrx_last_ch = h->channel;
	pearl_mdlogrx_last_data0 = h->data[0];
	pearl_mdlogrx_last_data1 = h->data[1];
	pearl_mdlogrx_last_resv = h->reserved;
	pearl_mdlogrx_last_ts = local_clock();
	room = pearl_mdlogrx_head - PEARL_MDLOGRX_SZ;
	if (room > PEARL_MDLOGRX_SZ)
		pearl_mdlogrx_lost++;
	head = pearl_mdlogrx_head;
	spin_unlock_irqrestore(&pearl_mdlogrx_lock, flags);

	/* 只对前 24 包打 dmesg，避免刷屏 */
	if (pearl_mdlogrx_pkts <= 24) {
		CCCI_ERROR_LOG(0, TAG,
			"PEARL-MDLOGRX[%u]: hif=%u ch=%u data0=0x%08x data1=0x%08x resv=0x%08x len=%u head=%u\n",
			pearl_mdlogrx_pkts, hif_id, h->channel, h->data[0],
			h->data[1], h->reserved, skb->len, head);
		/* 负载前 16 字节，按 32bit 打，方便肉眼认 */
		for (i = 0; i + 4 <= 16 && (i + 4 + sizeof(struct ccci_header)) <= skb->len;
			i += 4)
			CCCI_ERROR_LOG(0, TAG,
				"PEARL-MDLOGRX[%u].pl[%u]=0x%08x\n",
				pearl_mdlogrx_pkts, i, *(unsigned int *)(pl + i));
	}
}
EXPORT_SYMBOL(pearl_mdlogrx_capture);

/* ---------------- mdlog 三步握手状态机 ---------------- */
/* PEARL-MDLOG-EXC: 单发要等久一点（原来 80ms 就判 no-reply）*/
static unsigned int pearl_mdlog_step1_ms = 500;
module_param(pearl_mdlog_step1_ms, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_step1_ms, "PEARL: step1(0x0C) resend interval in ms");

/* PEARL-MDLOG-EXC: 剂量=1（>=3 发会灌爆 MD 侧队列，见实验数据）*/
static unsigned int pearl_mdlog_step1_max = 1;
module_param(pearl_mdlog_step1_max, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_step1_max,
	"PEARL: fast step1(0x0C) sends (step1_ms interval each)");

/* 快发 phase 之后再慢发这么多条（1 s 间隔），覆盖 B 形态那 43 s 的窗口。
 * 第 20 轮只快发 60 条、6 s 就停，B 形态的后半段根本没人握手。 */
/* PEARL-MDLOG-EXC: 慢发次数=0，保持总剂量 1 */
static unsigned int pearl_mdlog_step1_slow = 0;
module_param(pearl_mdlog_step1_slow, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_step1_slow,
	"PEARL: extra step1(0x0C) sends at 1s interval after the fast phase");

static unsigned int pearl_mdlog_step2_resv;
module_param(pearl_mdlog_step2_resv, uint, 0644);
MODULE_PARM_DESC(pearl_mdlog_step2_resv,
	"PEARL: reserved sent in step2(0x02); 0 = use the value MODEM reported");

#define PEARL_MDLOG_MEM_SZ	(4 * 1024 * 1024)
static void *pearl_mdlog_mem;
static phys_addr_t pearl_mdlog_mem_pa;

static void pearl_mdlog_fn(struct work_struct *work);
static DECLARE_WORK(pearl_mdlog_work, pearl_mdlog_fn);
static unsigned int pearl_mdlog_stage;	/* 0=idle 1=step1 2=step2 3=step3 9=done */

static void pearl_mdlog_fn(struct work_struct *work)
{
	unsigned int i, max = pearl_mdlog_step1_max;
	unsigned int seen_before = pearl_mdlog_msg_n;
	u32 resv, use;
	bool got = false;

	if (!max)
		max = 1;
	pearl_mdlog_stage = 1;
	CCCI_ERROR_LOG(0, TAG,
		"PEARL-MDLOG-FSM: start, md_state=%d fast=%u slow=%u interval=%ums\n",
		ccci_fsm_get_md_state(0), max, pearl_mdlog_step1_slow,
		pearl_mdlog_step1_ms);

	for (i = 0; i < max + pearl_mdlog_step1_slow; i++) {
		int st = ccci_fsm_get_md_state(0);
		unsigned int delay;

		if (pearl_mdlog_msg_n != seen_before)
			break;
		/* PEARL-MDLOG-EXC: 只在 EXCEPTION / READY 发；HS1/HS2 是厂商
		 * 明确禁止一切端口流量的窗口（见 port_proxy.c），在那里发会把
		 * 基带打死（实验 M）。 */
		if (st != EXCEPTION && st != READY)
			break;
		delay = (i < max) ? pearl_mdlog_step1_ms : 1000;
		pearl_mdlog_send(0x0C, 0, 0, (i < max) ? "fsm-step1" : "fsm-step1-slow");
		msleep(delay ? delay : 1);
		if (pearl_mdlog_msg_n != seen_before) {
			got = true;
			break;
		}
	}

	if (!got)
		CCCI_ERROR_LOG(0, TAG,
			"PEARL-MDLOG-FSM: no reply after %u step1 (md_state=%d rx_pkts=%u)\n",
			i, ccci_fsm_get_md_state(0), pearl_mdlogrx_pkts);

	/* 只要收到过 42/43 号通道的报文，就继续走 step2/step3 */
	if (pearl_mdlog_msg_n != seen_before || pearl_mdlog_stage == 9) {
		resv = pearl_mdlog_last_resv;
		use = pearl_mdlog_step2_resv ? pearl_mdlog_step2_resv : resv;
		CCCI_ERROR_LOG(0, TAG,
			"PEARL-MDLOG-FSM: reply seen (ch=%u data1=0x%x resv=0x%x) -> step2 resv=0x%x\n",
			pearl_mdlogrx_last_ch, pearl_mdlogrx_last_data1, resv, use);

		if (resv != 0 && !pearl_mdlog_mem && modem_sys[0]
				&& modem_sys[0]->plat_dev) {
			pearl_mdlog_mem = dma_alloc_coherent(
				&modem_sys[0]->plat_dev->dev,
				PEARL_MDLOG_MEM_SZ, &pearl_mdlog_mem_pa,
				GFP_KERNEL);
			CCCI_ERROR_LOG(0, TAG,
				"PEARL-MDLOG-FSM: alloc %uKB -> %p pa=0x%llx\n",
				PEARL_MDLOG_MEM_SZ / 1024, pearl_mdlog_mem,
				(unsigned long long)pearl_mdlog_mem_pa);
			if (pearl_mdlog_mem && !pearl_mdlog_step2_resv)
				use = (u32)pearl_mdlog_mem_pa;
		}

		pearl_mdlog_stage = 2;
		pearl_mdlog_send(0x02, use, 0, "fsm-step2");
		msleep(200);

		pearl_mdlog_send(0x08, 0, 0, "fsm-step3-sd");
		msleep(50);
		pearl_mdlog_send(0x0A, 0, 0, "fsm-step3");
		pearl_mdlog_stage = 9;
		CCCI_ERROR_LOG(0, TAG,
			"PEARL-MDLOG-FSM: done (mem=%p pa=0x%llx rx_pkts=%u head=%u)\n",
			pearl_mdlog_mem,
			(unsigned long long)pearl_mdlog_mem_pa,
			pearl_mdlogrx_pkts, pearl_mdlogrx_head);
	} else {
		pearl_mdlog_stage = 9;
	}
}

void pearl_mdlog_kick(void)
{
	static int armed;

	if (armed || !pearl_mdlog_auto)
		return;
	/* PEARL-MDLOG-EXC: 双保险——不在 EXCEPTION 就别拉日志 */
	if (ccci_fsm_get_md_state(0) != EXCEPTION) {
		CCCI_ERROR_LOG(0, TAG,
			"PEARL-MDLOG-EXC: refuse kick at md_state=%d (only EXCEPTION)\n",
			ccci_fsm_get_md_state(0));
		return;
	}
	armed = 1;
	if (!pearl_mdlogrx_buf)
		pearl_mdlogrx_buf = kzalloc(PEARL_MDLOGRX_SZ, GFP_ATOMIC);
	CCCI_ERROR_LOG(0, TAG,
		"PEARL-MDLOG-FSM: kick at md_state=%d auto=%u buf=%p\n",
		ccci_fsm_get_md_state(0), pearl_mdlog_auto, pearl_mdlogrx_buf);
	schedule_work(&pearl_mdlog_work);
}
EXPORT_SYMBOL(pearl_mdlog_kick);

/* ---------------- sysfs: /sys/kernel/ccci/mdsys1/mdlogrx ---------------- */
static ssize_t md_cd_mdlogrx_show(struct ccci_modem *md, char *buf)
{
	unsigned long flags;
	unsigned int head, pkt, lost, ch, d0, d1, resv, n, i, base;
	int pos = 0;

	if (!pearl_mdlogrx_buf)
		return snprintf(buf, 320,
			"mdlogrx: buffer not allocated yet (auto=%u stage=%u)\n",
			pearl_mdlog_auto, pearl_mdlog_stage);

	spin_lock_irqsave(&pearl_mdlogrx_lock, flags);
	head = pearl_mdlogrx_head;
	pkt = pearl_mdlogrx_pkts;
	lost = pearl_mdlogrx_lost;
	ch = pearl_mdlogrx_last_ch;
	d0 = pearl_mdlogrx_last_data0;
	d1 = pearl_mdlogrx_last_data1;
	resv = pearl_mdlogrx_last_resv;
	spin_unlock_irqrestore(&pearl_mdlogrx_lock, flags);

	pos += snprintf(buf + pos, 400 - pos,
		"stage=%u md_state=%d auto=%u pkts=%u lost=%u head=%u size=%u\n"
		"last: ch=%u data0=0x%08x data1=0x%08x resv=0x%08x\n"
		"mem=%p pa=0x%llx step2_resv_arg=0x%x\n",
		pearl_mdlog_stage, ccci_fsm_get_md_state(md->index),
		pearl_mdlog_auto, pkt, lost, head, PEARL_MDLOGRX_SZ,
		ch, d0, d1, resv, pearl_mdlog_mem,
		(unsigned long long)pearl_mdlog_mem_pa, pearl_mdlog_step2_resv);

	spin_lock_irqsave(&pearl_mdlog_msg_lock, flags);
	n = pearl_mdlog_msg_n;
	if (n > PEARL_MDLOG_MSG_MAX)
		base = n - PEARL_MDLOG_MSG_MAX;
	else
		base = 0;
	pos += snprintf(buf + pos, 1000 - pos, "-- msg ring (%u total) --\n", n);
	for (i = base; i < n && pos < 3800; i++) {
		struct pearl_mdlog_msg *m =
			&pearl_mdlog_msgs[i % PEARL_MDLOG_MSG_MAX];

		pos += snprintf(buf + pos, 3900 - pos,
			"%4u ch=%u d0=0x%08x d1=0x%08x resv=0x%08x plen=%u pl=%*phN\n",
			i, m->ch, m->data0, m->data1, m->resv,
			m->payload_len, m->payload_len > 24 ? 24 : m->payload_len,
			m->payload);
	}
	spin_unlock_irqrestore(&pearl_mdlog_msg_lock, flags);
	/* sysfs show 只给了一个 PAGE_SIZE 的 buffer */
	if (pos > 4000)
		pos = 4000;
	return pos;
}
/* ===== PEARL-MDLOGRX-END ===== */









static ssize_t md_cd_debug_show(struct ccci_modem *md, char *buf)
{
	int curr = 0;

	curr = snprintf(buf, 16, "%d\n", ccci_debug_enable);
	if (curr < 0 || curr >= 16) {
		CCCI_ERROR_LOG(md->index, TAG,
			"%s-%d:snprintf fail,curr = %d\n", __func__, __LINE__, curr);
		return -1;
	}
	return curr;
}

static ssize_t md_cd_debug_store(struct ccci_modem *md,
	const char *buf, size_t count)
{
	ccci_debug_enable = buf[0] - '0';

	return count;
}

static ssize_t md_cd_dump_show(struct ccci_modem *md, char *buf)
{
	int count = 0;

	count = snprintf(buf, 256,
		"support: ccif cldma register smem image layout\n");
	return count;
}

static ssize_t md_cd_dump_store(struct ccci_modem *md,
	const char *buf, size_t count)
{
	enum MD_STATE md_state = ccci_fsm_get_md_state(md->index);
	/* echo will bring "xxx\n" here,
	 * so we eliminate the "\n" during comparing
	 */
	if (md_state != GATED && md_state != INVALID) {
		if (strncmp(buf, "ccif", count - 1) == 0)
			ccci_hif_dump_status(1 << CCIF_HIF_ID,
				DUMP_FLAG_CCIF_REG | DUMP_FLAG_CCIF, NULL, 0);
		if ((strncmp(buf, "cldma", count - 1) == 0) &&
			(md->hif_flag&(1<<CLDMA_HIF_ID)))
			ccci_hif_dump_status(1 << CLDMA_HIF_ID,
				DUMP_FLAG_CLDMA, NULL, -1);
		if (strncmp(buf, "register", count - 1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_REG, NULL, 0);
		if (strncmp(buf, "smem_exp", count-1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_SMEM_EXP, NULL, 0);
		if (strncmp(buf, "smem_ccism", count-1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_SMEM_CCISM, NULL, 0);
		if (strncmp(buf, "smem_ccb_ctrl", count-1) == 0)
			md->ops->dump_info(md,
				DUMP_FLAG_SMEM_CCB_CTRL, NULL, 0);
		if (strncmp(buf, "smem_ccb_data", count-1) == 0)
			md->ops->dump_info(md,
				DUMP_FLAG_SMEM_CCB_DATA, NULL, 0);
		if (strncmp(buf, "pccif", count - 1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_PCCIF_REG, NULL, 0);
		if (strncmp(buf, "layout", count - 1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_LAYOUT, NULL, 0);
		if (strncmp(buf, "mdslp", count - 1) == 0)
			md->ops->dump_info(md, DUMP_FLAG_SMEM_MDSLP, NULL, 0);
		if (strncmp(buf, "dpmaif", count - 1) == 0)
			ccci_hif_dump_status(1<<DPMAIF_HIF_ID,
				DUMP_FLAG_REG, NULL, -1);
		if (strncmp(buf, "port", count - 1) == 0)
			ccci_port_dump_status(md->index);
	}
	return count;
}

static ssize_t md_net_speed_show(struct ccci_modem *md, char *buf)
{
	return snprintf(buf, 4096, "curr netspeed log: %d\n",
		ccci_hif_dump_status(DPMAIF_HIF_ID, DUMP_FLAG_TOGGLE_NET_SPD,
			NULL, -1));
}

static ssize_t md_cd_parameter_show(struct ccci_modem *md, char *buf)
{
	int count = 0;

	count += snprintf(buf + count, 128,
		"PACKET_HISTORY_DEPTH=%d\n", PACKET_HISTORY_DEPTH);
	count += snprintf(buf + count, 128, "BD_NUM=%ld\n", MAX_BD_NUM);

	return count;
}

static ssize_t md_cd_parameter_store(struct ccci_modem *md,
	const char *buf, size_t count)
{
	return count;
}
/* PEARL-28 阶段 2：这些实现都在 fsm/ccci_fsm_scp.c 里（那一侧没有独立头文件
 * 可供 include，所以在这里显式声明一次，保持本轮改动不扩散）。
 */
#if defined(CCCI_KMODULE_ENABLE)
int pearl_scp_ipi_register_now(const char *why);
void pearl_scp_ipi_register_info(int *registered, int *auto_registered,
				int *early_flag);
extern unsigned int pearl_scp_ipi_autoreg;
#endif

/* ===== PEARL-28 阶段 2：手动推迟注册口 =====
 * echo 1 > /sys/kernel/ccci/mdsys1/scp_ipi_register
 * 读回来可以看到 registered/auto/early 三个状态位与当前 md_state。
 */
static ssize_t md_cd_scp_ipi_register_show(struct ccci_modem *md, char *buf)
{
	int registered = 0, auto_reg = 0, early = 0;
	int curr;

	pearl_scp_ipi_register_info(&registered, &auto_reg, &early);
	curr = snprintf(buf, 400,
		"usage: echo 1 > scp_ipi_register   (register IPI_IN_APCCCI_0 now)\n"
		"registered=%d auto_registered=%d early_in_fsm_scp_init0=%d\n"
		"md_state=%d autoreg_param=%u\n",
		registered, auto_reg, early,
		ccci_fsm_get_md_state(md->index),
		pearl_scp_ipi_autoreg);
	return curr;
}

static ssize_t md_cd_scp_ipi_register_store(struct ccci_modem *md,
	const char *buf, size_t count)
{
	int ret;

	if (buf[0] != '1') {
		CCCI_ERROR_LOG(md->index, TAG,
			"PEARL-28: write 1 to register, got '%c'\n", buf[0]);
		return -EINVAL;
	}
	ret = pearl_scp_ipi_register_now("sysfs");
	CCCI_NORMAL_LOG(md->index, TAG,
		"PEARL-28 sysfs: pearl_scp_ipi_register_now -> %d (md_state=%d)\n",
		ret, ccci_fsm_get_md_state(md->index));
	return count;
}

CCCI_MD_ATTR(NULL, debug, 0660, md_cd_debug_show, md_cd_debug_store);
CCCI_MD_ATTR(NULL, dump, 0660, md_cd_dump_show, md_cd_dump_store);
CCCI_MD_ATTR(NULL, net_speed, 0660, md_net_speed_show, NULL);
CCCI_MD_ATTR(NULL, parameter, 0660, md_cd_parameter_show,
	md_cd_parameter_store);
CCCI_MD_ATTR(NULL, mdlog, 0660, md_cd_mdlog_show, md_cd_mdlog_store);
CCCI_MD_ATTR(NULL, mdlogrx, 0440, md_cd_mdlogrx_show, NULL);
CCCI_MD_ATTR(NULL, scp_ipi_register, 0660, md_cd_scp_ipi_register_show,
	md_cd_scp_ipi_register_store);

static void md_cd_sysfs_init(struct ccci_modem *md)
{
	int ret;

	ccci_md_attr_debug.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_debug.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_debug.attr.name, ret);

	ccci_md_attr_dump.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_dump.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_dump.attr.name, ret);

	ccci_md_attr_net_speed.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_net_speed.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_net_speed.attr.name, ret);

	ccci_md_attr_parameter.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_parameter.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_parameter.attr.name, ret);
	ccci_md_attr_mdlog.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_mdlog.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_mdlog.attr.name, ret);

	ccci_md_attr_mdlogrx.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_mdlogrx.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_mdlogrx.attr.name, ret);

	ccci_md_attr_scp_ipi_register.modem = md;
	ret = sysfs_create_file(&md->kobj, &ccci_md_attr_scp_ipi_register.attr);
	if (ret)
		CCCI_ERROR_LOG(md->index, TAG,
			"fail to add sysfs node %s %d\n",
			ccci_md_attr_scp_ipi_register.attr.name, ret);
}

void ccci_hif_cldma_restore_reg(struct ccci_modem *md)
{
}

static void ccci_modem_restore_reg(struct ccci_modem *md)
{
	enum MD_STATE md_state = ccci_fsm_get_md_state(md->index);

	if (md_state == GATED || md_state == WAITING_TO_STOP ||
		md_state == INVALID) {
		CCCI_NORMAL_LOG(md->index, TAG,
			"Resume no need restore for md_state=%d\n", md_state);
		return;
	}

	if (md->hif_flag & (1 << CLDMA_HIF_ID))
		ccci_hif_cldma_restore_reg(md);

	ccci_hif_resume(md->index, md->hif_flag);
}

int ccci_modem_syssuspend(void *unused)
{
	struct ccci_modem *md;

	CCCI_DEBUG_LOG(0, TAG, "%s\n", __func__);
	md = ccci_md_get_modem_by_id(0);
	if (md != NULL)
		ccci_hif_suspend(md->index, md->hif_flag);
	return 0;
}

void ccci_modem_sysresume(void *unused)
{
	struct ccci_modem *md;

	CCCI_DEBUG_LOG(0, TAG, "%s\n", __func__);
	md = ccci_md_get_modem_by_id(0);
	if (md != NULL)
		ccci_modem_restore_reg(md);
}


static const struct syscore_ops ccci_modem_sysops = {
	.suspend = ccci_modem_syssuspend,
	.resume = ccci_modem_sysresume,
};

static struct syscore ccci_modem_syscore = {
	.ops = &ccci_modem_sysops,
};

static u64 cldma_dmamask = DMA_BIT_MASK(36);

int ccci_modem_init_common(struct platform_device *plat_dev,
	struct ccci_dev_cfg *dev_cfg, struct md_hw_info *md_hw)
{
	struct ccci_modem *md = NULL;
	struct md_sys1_info *md_info = NULL;
	int md_id;
	int ret;

	/* Allocate md ctrl memory and do initialize */
	md = ccci_md_alloc(sizeof(struct md_sys1_info));
	if (md == NULL) {
		CCCI_ERROR_LOG(-1, TAG,
			"%s:alloc modem ctrl mem fail\n", __func__);
		return -1;
	}
	md->index = md_id = dev_cfg->index;
	md->per_md_data.md_capability = dev_cfg->capability;
	md->hw_info = md_hw;

	md->plat_dev = plat_dev;
	md->plat_dev->dev.dma_mask = &cldma_dmamask;
	md->plat_dev->dev.coherent_dma_mask = cldma_dmamask;
	md->ops = &md_cd_ops;
	CCCI_INIT_LOG(md_id, TAG,
		"%s:md=%p,md->private_data=%p\n", __func__,
		md, md->private_data);

	/* register modem */
	ccci_md_register(md);

	/* init modem private data */
	md_info = (struct md_sys1_info *)md->private_data;

	snprintf(md->trm_wakelock_name, sizeof(md->trm_wakelock_name),
		"md%d_cldma_trm", md_id + 1);
	md->trm_wake_lock = wakeup_source_register(NULL, md->trm_wakelock_name);
	if (!md->trm_wake_lock) {
		CCCI_ERROR_LOG(md->index, TAG,
			"%s %d: init wakeup source fail",
			__func__, __LINE__);
		return -1;
	}
	snprintf(md_info->peer_wakelock_name,
		sizeof(md_info->peer_wakelock_name),
		"md%d_cldma_peer", md_id + 1);

	md_info->peer_wake_lock =
		wakeup_source_register(NULL, md_info->peer_wakelock_name);
	if (!md_info->peer_wake_lock) {
		CCCI_ERROR_LOG(md->index, TAG,
			"%s %d: init wakeup source fail",
			__func__, __LINE__);
		return -1;
	}
	/* Copy HW info */
	md_info->ap_ccif_irq_id = md_hw->ap_ccif_irq1_id;
	md_info->channel_id = 0;
	atomic_set(&md_info->ccif_irq_enabled, 1);

	md->md_wdt_irq_id = md_hw->md_wdt_irq_id;
	atomic_set(&md->reset_on_going, 1);
	/* IRQ is default enabled after request_irq */
	atomic_set(&md->wdt_enabled, 1);

	ret = of_property_read_u32(plat_dev->dev.of_node,
		"mediatek,mdhif_type", &md->hif_flag);
	if (ret != 0)
		md->hif_flag = (1 << MD1_NET_HIF | 1 << MD1_NORMAL_HIF);

	//ret = ccci_hif_init(md->index, md->hif_flag);
	//if (ret < 0) {
	//	CCCI_ERROR_LOG(md->index, TAG,
	//		"[%s] error: ccci_hif_init() failed(%d)\n",
	//		__func__, ret);
	//	return ret;
	//}
	/* register SYS CORE suspend resume call back */
	register_syscore(&ccci_modem_syscore);

	/* add sysfs entries */
	md_cd_sysfs_init(md);
	/* hook up to device */
	plat_dev->dev.platform_data = md;

	return 0;
}
EXPORT_SYMBOL(ccci_modem_init_common);

void ccci_platform_common_init(struct ccci_modem *md)
{
	if (md->hw_info->plat_ptr->init)
		md->hw_info->plat_ptr->init(md);
}

int Is_MD_EMI_voilation(void)
{
	return 1;
}

