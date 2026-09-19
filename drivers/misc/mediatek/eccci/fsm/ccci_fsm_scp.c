// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifdef CCCI_KMODULE_ENABLE
#include <linux/string.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#endif
#include <linux/clk.h> /* for clk_prepare/un* */

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_fsm_internal.h"
#include "md_sys1_platform.h"

/* XAGA-25: 本树 SCP 驱动 (CONFIG_MTK_TINYSYS_SCP_SUPPORT) 是独立模块
 * scp.ko，而 CCCI 是内建 (CONFIG_MTK_CCCI_MAINLINE=y)。内建对象不能引用
 * 只由模块导出的符号 (scp_A_register_notify / scp_ipidev)，否则 vmlinux
 * 链接阶段报 undefined reference。只有 SCP 也内建时才允许碰这两个符号。
 */
#if defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT) && \
	!defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT_MODULE)
#define CCCI_SCP_DRIVER_BUILTIN
#endif

#ifdef FEATURE_SCP_CCCI_SUPPORT
#include "scp_ipi.h"

#ifdef CCCI_KMODULE_ENABLE
void ccci_scp_md_state_sync(int md_state);

struct ccci_fsm_scp ccci_scp_ctl = {
	.md_id = 0,
	.md_state_sync = &ccci_scp_md_state_sync,
};

static struct ccci_clk_node scp_clk_table[] = {
	{ NULL, "infra-ccif2-ap"},
	{ NULL, "infra-ccif2-md"},
};

void ccci_scp_md_state_sync(int md_state)
{
	schedule_work(&ccci_scp_ctl.scp_md_state_sync_work);
}


/* XAGA-25: 这里原来又定义了一份 ccci_debug_enable。原厂 ccci_fsm_scp.o
 * 是独立模块所以不冲突；本树把它并进内建的 ccci_md_all 后，与
 * ccci_core.c:37 的同名定义在 vmlinux.o 链接时撞成
 * "duplicate symbol: ccci_debug_enable"。ccci_debug.h 已有 extern 声明，
 * 直接用 ccci_core.c 那一份。 */
#endif

static atomic_t scp_state = ATOMIC_INIT(SCP_CCCI_STATE_INVALID);
static struct ccci_ipi_msg scp_ipi_tx_msg;
static struct mutex scp_ipi_tx_mutex;
static struct work_struct scp_ipi_rx_work;
static wait_queue_head_t scp_ipi_rx_wq __maybe_unused;
static struct ccci_skb_queue scp_ipi_rx_skb_list;
static unsigned int init_work_done __maybe_unused;
static unsigned int scp_clk_last_state;
#if (MD_GENERATION >= 6297)
static struct ccci_ipi_msg scp_ipi_rx_msg __maybe_unused;
#endif

static int ccci_scp_ipi_send(int md_id, int op_id, void *data)
{
	int ret = 0;
#if (MD_GENERATION >= 6297) && defined(CCCI_SCP_DRIVER_BUILTIN)
	int ipi_status = 0;
	unsigned int cnt = 0;
#endif
	if (atomic_read(&scp_state) == SCP_CCCI_STATE_INVALID) {
		CCCI_ERROR_LOG(md_id, FSM,
			"ignore IPI %d, SCP state %d!\n",
			op_id, atomic_read(&scp_state));
		return -CCCI_ERR_MD_NOT_READY;
	}

	mutex_lock(&scp_ipi_tx_mutex);
	memset(&scp_ipi_tx_msg, 0, sizeof(scp_ipi_tx_msg));
	scp_ipi_tx_msg.md_id = md_id;
	scp_ipi_tx_msg.op_id = op_id;
	scp_ipi_tx_msg.data[0] = *((u32 *)data);
	CCCI_NORMAL_LOG(scp_ipi_tx_msg.md_id, FSM,
		"IPI send op_id=%d/data=0x%x, size=%d\n",
		scp_ipi_tx_msg.op_id, scp_ipi_tx_msg.data[0],
		(int)sizeof(struct ccci_ipi_msg));
#if (MD_GENERATION >= 6297)
#ifdef CCCI_SCP_DRIVER_BUILTIN
	while (1) {
		ipi_status = mtk_ipi_send(&scp_ipidev, IPI_OUT_APCCCI_0,
		0, &scp_ipi_tx_msg, (sizeof(scp_ipi_tx_msg) / 4), 1);
		if (ipi_status != IPI_PIN_BUSY)
			break;
		cnt++;
		if (cnt > 10) {
			CCCI_ERROR_LOG(md_id, FSM, "IPI send 10 times!\n");
			/* aee_kernel_warning("ccci", "ipi:tx busy");*/
			break;
		}
	}
	if (ipi_status != IPI_ACTION_DONE) {
		CCCI_ERROR_LOG(md_id, FSM, "IPI send fail!\n");
		ret = -CCCI_ERR_MD_NOT_READY;
	}
#else
	/* XAGA-25: scp_ipidev 由 scp.ko 提供，内建 CCCI 不能引用它；
	 * 而且 scp.ko 没加载时这条 IPI 也没人应答（scp_state 恒为 INVALID，
	 * 上面那句判断早已 return）。语义与原来一致：MD 未就绪。 */
	CCCI_NORMAL_LOG(md_id, FSM,
		"XAGA-25 skip SCP IPI %d, SCP driver is a module\n", op_id);
	ret = -CCCI_ERR_MD_NOT_READY;
#endif
#else
	if (scp_ipi_send(IPI_APCCCI, &scp_ipi_tx_msg,
			sizeof(scp_ipi_tx_msg), 1, SCP_A_ID) != SCP_IPI_DONE) {
		CCCI_ERROR_LOG(md_id, FSM, "IPI send fail!\n");
		ret = -CCCI_ERR_MD_NOT_READY;
	}
#endif
	mutex_unlock(&scp_ipi_tx_mutex);
	return ret;
}

static int scp_set_clk_cg(unsigned int on)
{
	int idx, ret;

	if (!(on == 0 || on == 1)) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"%s:on=%u is invalid\n", __func__, on);
		return -1;
	}

	if (on == scp_clk_last_state) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM, "%s:on=%u skip set scp clk!\n",
			__func__, on);
		return 0;
	}

	for (idx = 0; idx < ARRAY_SIZE(scp_clk_table); idx++) {
		if (scp_clk_table[idx].clk_ref == NULL) {
			/* XAGA-25: 没走平台设备 probe 时 clk 没被 devm_clk_get
			 * 填过，clk_prepare_enable(NULL) 会直接空指针崩。 */
			CCCI_ERROR_LOG(MD_SYS1, FSM,
				"%s: clk %s not available\n", __func__,
				scp_clk_table[idx].clk_name);
			return -1;
		}
		if (on) {
			ret = clk_prepare_enable(scp_clk_table[idx].clk_ref);
			if (ret) {
				CCCI_ERROR_LOG(MD_SYS1, FSM,
					"open scp clk fail:%s,ret=%d\n",
					scp_clk_table[idx].clk_name, ret);
				return -1;
			}
		} else
			clk_disable_unprepare(scp_clk_table[idx].clk_ref);
	}

	CCCI_NORMAL_LOG(MD_SYS1, FSM, "%s:on=%u set done!\n",
		__func__, on);
	scp_clk_last_state = on;

	return 0;
}

static void ccci_scp_md_state_sync_work(struct work_struct *work)
{
	struct ccci_fsm_scp *scp_ctl = container_of(work,
		struct ccci_fsm_scp, scp_md_state_sync_work);
	struct ccci_fsm_ctl *ctl = fsm_get_entity_by_md_id(scp_ctl->md_id);
	enum MD_STATE_FOR_USER state;
	int ret = 0;
	int count = 0;

	if (!ctl) {
		CCCI_ERROR_LOG(ctl->md_id, FSM, "%s ctl is NULL !\n", __func__);
		return;
	}

	switch (ctl->md_state) {
	case READY:
		if (scp_ctl->md_id == MD_SYS1) {
			while (count < SCP_BOOT_TIMEOUT/EVENT_POLL_INTEVAL) {
				if (atomic_read(&scp_state) ==
					SCP_CCCI_STATE_BOOTING
					|| atomic_read(&scp_state)
					== SCP_CCCI_STATE_RBREADY
					|| atomic_read(&scp_state)
					== SCP_CCCI_STATE_STOP)
					break;
				count++;
				msleep(EVENT_POLL_INTEVAL);
			}
			if (count == SCP_BOOT_TIMEOUT/EVENT_POLL_INTEVAL)
				CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
					"SCP init not ready!\n");
			else {
				ret = scp_set_clk_cg(1);
				if (ret) {
					CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
						"fail to set scp clk, ret = %d\n", ret);
					break;
				}

				ret = ccci_port_send_msg_to_md(scp_ctl->md_id,
					CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);
				if (ret < 0)
					CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
						"fail to send CCISM_SHM_INIT %d\n",
						ret);
			}
		} else
			break;
		break;
	case INVALID:
	case GATED:
		state = MD_STATE_INVALID;
		ccci_scp_ipi_send(scp_ctl->md_id,
			CCCI_OP_MD_STATE, &state);
		break;
	case EXCEPTION:
		state = MD_STATE_EXCEPTION;
		ccci_scp_ipi_send(scp_ctl->md_id,
			CCCI_OP_MD_STATE, &state);
		break;
	default:
		break;
	};
}

static void __maybe_unused ccci_scp_ipi_rx_work(struct work_struct *work)
{
	struct ccci_ipi_msg *ipi_msg_ptr = NULL;
	struct sk_buff *skb = NULL;
	int data, ret;

	while (!skb_queue_empty(&scp_ipi_rx_skb_list.skb_list)) {
		skb = ccci_skb_dequeue(&scp_ipi_rx_skb_list);
		if (skb == NULL) {
			CCCI_ERROR_LOG(-1, CORE,
				"ccci_skb_dequeue fail\n");
			return;
		}
		ipi_msg_ptr = (struct ccci_ipi_msg *)skb->data;
		if (!get_modem_is_enabled(ipi_msg_ptr->md_id)) {
			CCCI_ERROR_LOG(ipi_msg_ptr->md_id,
				CORE, "MD not exist\n");
			return;
		}
		switch (ipi_msg_ptr->op_id) {
		case CCCI_OP_SCP_STATE:
			switch (ipi_msg_ptr->data[0]) {
			case SCP_CCCI_STATE_BOOTING:
				if (atomic_read(&scp_state) ==
					SCP_CCCI_STATE_RBREADY) {
					CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"SCP reset detected\n");
					ccci_port_send_msg_to_md(MD_SYS1,
					CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);
					ccci_port_send_msg_to_md(MD_SYS3,
					CCCI_CONTROL_TX,
					C2K_CCISM_SHM_INIT, 0, 1);
				} else {
					CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"SCP boot up\n");
				}
				/* too early to init share memory here,
				 * EMI MPU may not be ready yet
				 */
				break;
			case SCP_CCCI_STATE_RBREADY:
				switch (ipi_msg_ptr->md_id) {
				case MD_SYS1:
					ccci_port_send_msg_to_md(MD_SYS1,
					CCCI_SYSTEM_TX,
					CCISM_SHM_INIT_DONE, 0, 1);
					break;
				case MD_SYS3:
					ccci_port_send_msg_to_md(MD_SYS3,
					CCCI_CONTROL_TX,
					C2K_CCISM_SHM_INIT_DONE, 0, 1);
					break;
				};
				data =
				ccci_fsm_get_md_state_for_user(
					ipi_msg_ptr->md_id);
				ccci_scp_ipi_send(ipi_msg_ptr->md_id,
					CCCI_OP_MD_STATE, &data);
				break;
			case SCP_CCCI_STATE_STOP:
				CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"MD INVALID,scp send ack to ap\n");
				ret = scp_set_clk_cg(0);
				if (ret)
					CCCI_ERROR_LOG(ipi_msg_ptr->md_id, FSM,
						"fail to set scp clk, ret = %d\n", ret);
				break;
			default:
				break;
			};
			atomic_set(&scp_state, ipi_msg_ptr->data[0]);
			break;
		default:
			break;
		};
		ccci_free_skb(skb);
	}
}

#if (MD_GENERATION >= 6297)
/*
 * IPI for logger init
 * @param id:   IPI id
 * @param prdata: callback function parameter
 * @param data:  IPI data
 * @param len: IPI data length
 */
static int __maybe_unused ccci_scp_ipi_handler(unsigned int id, void *prdata,
			void *data,
			unsigned int len)
{
	struct sk_buff *skb = NULL;

	if (len != sizeof(struct ccci_ipi_msg)) {
		CCCI_ERROR_LOG(-1, CORE,
		"IPI handler, data length wrong %d vs. %d\n",
		len, (int)sizeof(struct ccci_ipi_msg));
		return -1;
	}

	skb = ccci_alloc_skb(len, 0, 0);
	if (!skb)
		return -1;

	memcpy(skb_put(skb, len), data, len);
	ccci_skb_enqueue(&scp_ipi_rx_skb_list, skb);
	/* ipi_send use mutex, can not be called from ISR context */
	schedule_work(&scp_ipi_rx_work);

	return 0;
}
#else
static void ccci_scp_ipi_handler(int id, void *data, unsigned int len)
{
	struct ccci_ipi_msg *ipi_msg_ptr = (struct ccci_ipi_msg *)data;
	struct sk_buff *skb = NULL;

	if (len != sizeof(struct ccci_ipi_msg)) {
		CCCI_ERROR_LOG(-1, CORE,
		"IPI handler, data length wrong %d vs. %d\n",
		len, (int)sizeof(struct ccci_ipi_msg));
		return;
	}
	CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, CORE,
		"IPI handler %d/0x%x, %d\n",
		ipi_msg_ptr->op_id,
		ipi_msg_ptr->data[0], len);

	skb = ccci_alloc_skb(len, 0, 0);
	if (!skb)
		return;
	memcpy(skb_put(skb, len), data, len);
	ccci_skb_enqueue(&scp_ipi_rx_skb_list, skb);
	/* ipi_send use mutex, can not be called from ISR context */
	schedule_work(&scp_ipi_rx_work);
}
#endif
#endif

int fsm_ccism_init_ack_handler(int md_id, int data)
{
#ifdef FEATURE_SCP_CCCI_SUPPORT
	struct ccci_smem_region *ccism_scp =
		ccci_md_get_smem_by_user_id(md_id, SMEM_USER_CCISM_SCP);

	/* XAGA-25: 原来这里没有任何判空，region 没配好就是空指针崩。
	 * 实测 "md1 get scp-sys-md1-main failed" 说明相关资源确实可能缺。 */
	if (ccism_scp == NULL || ccism_scp->base_ap_view_vir == NULL) {
		CCCI_ERROR_LOG(md_id, FSM,
			"CCISM_SHM_INIT_ACK: ccism_scp(%px) not ready\n",
			ccism_scp);
		return 0;
	}
	memset_io(ccism_scp->base_ap_view_vir, 0, ccism_scp->size);
	ccci_scp_ipi_send(md_id, CCCI_OP_SHM_INIT,
		&ccism_scp->base_ap_view_phy);
#endif
	return 0;
}

static int fsm_sim_type_handler(int md_id, int data)
{
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(md_id);

	per_md_data->sim_type = data;
	return 0;
}

#ifdef CCCI_KMODULE_ENABLE
#ifdef FEATURE_SCP_CCCI_SUPPORT
#ifdef CCCI_SCP_DRIVER_BUILTIN
void fsm_scp_init0(void)
{
	enum MD_STATE_FOR_USER state =
		ccci_fsm_get_md_state_for_user(ccci_scp_ctl.md_id);
	mutex_init(&scp_ipi_tx_mutex);

	if (!init_work_done) {
		INIT_WORK(&scp_ipi_rx_work, ccci_scp_ipi_rx_work);
		init_work_done = 1;
	}
	init_waitqueue_head(&scp_ipi_rx_wq);
	ccci_skb_queue_init(&scp_ipi_rx_skb_list, 16, 16, 0);

	CCCI_NORMAL_LOG(-1, FSM, "register IPI\n");

#if (MD_GENERATION >= 6297)
	if (mtk_ipi_register(&scp_ipidev, IPI_IN_APCCCI_0,
		(void *)ccci_scp_ipi_handler, NULL,
		&scp_ipi_rx_msg) != IPI_ACTION_DONE)
		CCCI_ERROR_LOG(-1, FSM, "register IPI fail!\n");
#else
	if (scp_ipi_registration(IPI_APCCCI, ccci_scp_ipi_handler,
		"AP CCCI") != SCP_IPI_DONE)
		CCCI_ERROR_LOG(-1, FSM, "register IPI fail!\n");
#endif
	atomic_set(&scp_state, SCP_CCCI_STATE_BOOTING);

	if (state != MD_STATE_INVALID)
		ccci_scp_md_state_sync(state);
}

static int apsync_event(struct notifier_block *this,
	unsigned long event, void *ptr)
{
	switch (event) {
	case SCP_EVENT_READY:
		fsm_scp_init0();
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block apsync_notifier = {
	.notifier_call = apsync_event,
};
#endif	/* CCCI_SCP_DRIVER_BUILTIN */
#endif
#endif
int fsm_scp_init(struct ccci_fsm_scp *scp_ctl)
{
#ifndef CCCI_KMODULE_ENABLE
	struct ccci_fsm_ctl *ctl =
		container_of(scp_ctl, struct ccci_fsm_ctl, scp_ctl);
#endif
	int ret = 0;

#ifdef FEATURE_SCP_CCCI_SUPPORT
#ifdef CCCI_SCP_DRIVER_BUILTIN
	scp_A_register_notify(&apsync_notifier);
#else
	/* XAGA-25: scp_A_register_notify 由 scp.ko 导出，内建 CCCI 不能引用。
	 * 它唯一的作用是把 SCP_EVENT_READY 接到 fsm_scp_init0()（IPI 注册），
	 * 而 scp.ko 没加载时这个事件永远不会来。真正决定 HS2 的是下面那两个
	 * register_ccci_sys_call_back()，照常执行。 */
	CCCI_NORMAL_LOG(-1, FSM,
		"XAGA-25 skip scp_A_register_notify, SCP driver is a module\n");
#endif
#endif
#ifndef CCCI_KMODULE_ENABLE
	scp_ctl->md_id = ctl->md_id;
#endif
#ifdef FEATURE_SCP_CCCI_SUPPORT
	INIT_WORK(&scp_ctl->scp_md_state_sync_work,
		ccci_scp_md_state_sync_work);
	register_ccci_sys_call_back(scp_ctl->md_id, CCISM_SHM_INIT_ACK,
		fsm_ccism_init_ack_handler);
#endif

	register_ccci_sys_call_back(scp_ctl->md_id, MD_SIM_TYPE,
		fsm_sim_type_handler);

	return ret;
}

#ifdef CCCI_KMODULE_ENABLE
/* XAGA-25: 本树把 SCP 胶水折进内建的 ccci_md_all，而没有走原厂的
 * "mediatek,ccci_md_scp" 平台设备（本机 DTS 没有这个节点，而且 scp.ko
 * 一 insmod 就挂死）。所以由 ccci_fsm_init() 直接调用这一入口，把
 * fsm_scp_init() 的两个 register_ccci_sys_call_back() 装上 ——
 * 这正是基带 HS2 阶段缺的东西（CCISM_SHM_INIT_ACK / MD_SIM_TYPE）。
 */
void ccci_fsm_scp_builtin_start(void)
{
	int ret;

	if (ccci_scp_ctl.md_id != MD_SYS1)
		return;

	ret = fsm_scp_init(&ccci_scp_ctl);
	CCCI_NORMAL_LOG(-1, FSM,
		"XAGA-25 %s: fsm_scp_init ret=%d md_id=%d sync=%ps\n",
		__func__, ret, ccci_scp_ctl.md_id,
		(void *)ccci_scp_ctl.md_state_sync);
	ccci_fsm_scp_register(ccci_scp_ctl.md_id, &ccci_scp_ctl);
}
#endif

static int ccif_scp_clk_init(struct device *dev)
{
	int idx;

	for (idx = 0; idx < ARRAY_SIZE(scp_clk_table); idx++) {
		scp_clk_table[idx].clk_ref = devm_clk_get(dev,
			scp_clk_table[idx].clk_name);
		if (IS_ERR(scp_clk_table[idx].clk_ref)) {
			CCCI_ERROR_LOG(-1, FSM,
				"%s:scp get %s failed\n",
				scp_clk_table[idx].clk_name);
			scp_clk_table[idx].clk_ref = NULL;
			return -1;
		}
	}

	return 0;
}

#ifdef CCCI_KMODULE_ENABLE
#ifdef FEATURE_SCP_CCCI_SUPPORT
static int ccci_scp_probe(struct platform_device *pdev)
{
	int ret;

	ret = ccif_scp_clk_init(&pdev->dev);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, FSM, "ccif scp clk init fail");
		return ret;
	}

	ret = fsm_scp_init(&ccci_scp_ctl);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, FSM, "ccci get scp info fail");
		return ret;
	}
	ccci_fsm_scp_register(0, &ccci_scp_ctl);
	return 0;
}


static const struct of_device_id ccci_scp_of_ids[] = {
	{.compatible = "mediatek,ccci_md_scp"},
	{}
};

static struct platform_driver ccci_scp_driver = {

	.driver = {
		.name = "ccci_md_scp",
		.of_match_table = ccci_scp_of_ids,
	},

	.probe = ccci_scp_probe,
};

static int __init ccci_scp_init(void)
{
	int ret;

	CCCI_NORMAL_LOG(-1, FSM, "ccci scp driver init start\n");

	ret = platform_driver_register(&ccci_scp_driver);
	if (ret) {
		CCCI_ERROR_LOG(-1, FSM, "ccci scp driver init fail %d", ret);
		return ret;
	}
	CCCI_NORMAL_LOG(-1, FSM, "ccci scp driver init end\n");
	return 0;
}

module_init(ccci_scp_init);
#endif
MODULE_AUTHOR("ccci");
MODULE_DESCRIPTION("ccci scp driver");
MODULE_LICENSE("GPL");

#endif
