// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#include <linux/list.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/wait.h>
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/netdevice.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h> /* for clk_prepare/un* */
#include <linux/syscore_ops.h>

#include "ccci_core.h"
#include "ccci_modem.h"
#include "ccci_bm.h"
#include "ccci_platform.h"
#include "ccci_hif_ccif.h"
#include "md_sys1_platform.h"
#include "modem_secure_base.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#define TAG "cif"
#define CCIF_EMPTY_IRQ_POLL_MS 20
/* struct md_ccif_ctrl *ccif_ctrl; */

unsigned int devapc_check_flag;
spinlock_t devapc_flag_lock;
static atomic_t ccif_data0_observe_count = ATOMIC_INIT(0);
static atomic_t ccif_data1_observe_count = ATOMIC_INIT(0);

int ccif_read32(void *b, unsigned long a)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&devapc_flag_lock, flags);
	ret = ((devapc_check_flag == 1) ?
			ioread32((void __iomem *)((b)+(a))) : 0);
	spin_unlock_irqrestore(&devapc_flag_lock, flags);

	return ret;
}

void ccif_write32(void *b, unsigned long a, unsigned int v)
{
	unsigned long flags;

	spin_lock_irqsave(&devapc_flag_lock, flags);
	if (devapc_check_flag == 1) {
		writel(v, (b) + (a));
		mb();
	}
	spin_unlock_irqrestore(&devapc_flag_lock, flags);
}

/* this table maybe can be set array when multi, or else. */
static struct ccci_clk_node ccif_clk_table[] = {
	{ NULL, "infra-ccif-ap"},
	{ NULL, "infra-ccif-md"},
	{ NULL, "infra-ccif1-ap"},
	{ NULL, "infra-ccif1-md"},
	{ NULL, "infra-ccif4-md"},
	{ NULL, "infra-ccif5-md"},
};
#define IS_PASS_SKB(per_md_data, qno)	\
	(!per_md_data->data_usb_bypass && (per_md_data->is_in_ee_dump == 0) \
	 && ((1<<qno) & NET_RX_QUEUE_MASK))

/* #define RUN_WQ_BY_CHECKING_RINGBUF */

struct c2k_port {
	enum c2k_channel ch;
	enum c2k_channel excp_ch;
	enum CCCI_CH tx_ch_mapping;
	enum CCCI_CH rx_ch_mapping;
};

static struct c2k_port c2k_ports[] = {
	/* c2k control channel mapping to 2 pairs of CCCI channels,
	 * please mind the order in this array,
	 * make sure CCCI_CONTROL_TX/RX be first.
	 */
	/*control channel */
	{CTRL_CH_C2K, CTRL_CH_C2K_EXCP, CCCI_CONTROL_TX, CCCI_CONTROL_RX,},
	/*control channel */
	{CTRL_CH_C2K, CTRL_CH_C2K, CCCI_STATUS_TX, CCCI_STATUS_RX,},
	/*audio channel */
	{AUDIO_CH_C2K, AUDIO_CH_C2K, CCCI_PCM_TX, CCCI_PCM_RX,},
	/*network channel for CCMNI1 */
	{NET1_CH_C2K, NET1_CH_C2K, CCCI_CCMNI1_TX, CCCI_CCMNI1_RX,},
	/*network channel for CCMNI1 */
	{NET1_CH_C2K, NET1_CH_C2K, CCCI_CCMNI1_DL_ACK, CCCI_CCMNI1_DL_ACK,},
	/*network channel for CCMNI2 */
	{NET2_CH_C2K, NET2_CH_C2K, CCCI_CCMNI2_TX, CCCI_CCMNI2_RX,},
	/*network channel for CCMNI2 */
	{NET2_CH_C2K, NET2_CH_C2K, CCCI_CCMNI2_DL_ACK, CCCI_CCMNI2_DL_ACK,},
	/*network channel for CCMNI3 */
	{NET3_CH_C2K, NET3_CH_C2K, CCCI_CCMNI3_TX, CCCI_CCMNI3_RX,},
	/*network channel for CCMNI3 */
	{NET3_CH_C2K, NET3_CH_C2K, CCCI_CCMNI3_DL_ACK, CCCI_CCMNI3_DL_ACK,},
	/*network channel for CCMNI4 */
	{NET4_CH_C2K, NET4_CH_C2K, CCCI_CCMNI4_TX, CCCI_CCMNI4_RX,},
	/*network channel for CCMNI5 */
	{NET5_CH_C2K, NET5_CH_C2K, CCCI_CCMNI5_TX, CCCI_CCMNI5_RX,},
	/*network channel for CCMNI6 */
	{NET6_CH_C2K, NET6_CH_C2K, CCCI_CCMNI6_TX, CCCI_CCMNI6_RX,},
	/*network channel for CCMNI7 */
	{NET7_CH_C2K, NET7_CH_C2K, CCCI_CCMNI7_TX, CCCI_CCMNI7_RX,},
	/*network channel for CCMNI8 */
	{NET8_CH_C2K, NET8_CH_C2K, CCCI_CCMNI8_TX, CCCI_CCMNI8_RX,},
	{NET10_CH_C2K, NET10_CH_C2K, CCCI_CCMNI10_TX, CCCI_CCMNI10_RX,},
	{NET11_CH_C2K, NET11_CH_C2K, CCCI_CCMNI11_TX, CCCI_CCMNI11_RX,},
	{NET12_CH_C2K, NET12_CH_C2K, CCCI_CCMNI12_TX, CCCI_CCMNI12_RX,},
	{NET13_CH_C2K, NET13_CH_C2K, CCCI_CCMNI13_TX, CCCI_CCMNI13_RX,},
	{NET14_CH_C2K, NET14_CH_C2K, CCCI_CCMNI14_TX, CCCI_CCMNI14_RX,},
	{NET15_CH_C2K, NET15_CH_C2K, CCCI_CCMNI15_TX, CCCI_CCMNI15_RX,},
	{NET16_CH_C2K, NET16_CH_C2K, CCCI_CCMNI16_TX, CCCI_CCMNI16_RX,},
	{NET17_CH_C2K, NET17_CH_C2K, CCCI_CCMNI17_TX, CCCI_CCMNI17_RX,},
	{NET18_CH_C2K, NET18_CH_C2K, CCCI_CCMNI18_TX, CCCI_CCMNI18_RX,},
	{NET19_CH_C2K, NET19_CH_C2K, CCCI_CCMNI19_TX, CCCI_CCMNI19_RX,},
	{NET20_CH_C2K, NET20_CH_C2K, CCCI_CCMNI20_TX, CCCI_CCMNI20_RX,},
	{NET21_CH_C2K, NET21_CH_C2K, CCCI_CCMNI21_TX, CCCI_CCMNI21_RX,},
	/*mdlogger ctrl channel */
	{MDLOG_CTRL_CH_C2K, MDLOG_CTRL_CH_C2K, CCCI_UART1_TX, CCCI_UART1_RX,},
	/*mdlogger data channel */
	{MDLOG_CH_C2K, MDLOG_CH_C2K, CCCI_MD_LOG_TX, CCCI_MD_LOG_RX,},
	/*flashless channel, new */
	{FS_CH_C2K, FS_CH_C2K, CCCI_FS_TX, CCCI_FS_RX,},
	/*ppp channel, for usb bypass */
	{DATA_PPP_CH_C2K, DATA_PPP_CH_C2K,
	CCCI_C2K_PPP_DATA, CCCI_C2K_PPP_DATA,},
	/*AT for rild, new */
	{AT_CH_C2K, AT_CH_C2K, CCCI_C2K_AT, CCCI_C2K_AT,},
	/*AT2 for rild, new */
	{AT2_CH_C2K, AT2_CH_C2K, CCCI_C2K_AT2, CCCI_C2K_AT2,},
	/*AT3 for rild, new */
	{AT3_CH_C2K, AT3_CH_C2K, CCCI_C2K_AT3, CCCI_C2K_AT3,},
	/*AT4 for rild, new */
	{AT4_CH_C2K, AT4_CH_C2K, CCCI_C2K_AT4, CCCI_C2K_AT4,},
	/*AT5 for rild, new */
	{AT5_CH_C2K, AT5_CH_C2K, CCCI_C2K_AT5, CCCI_C2K_AT5,},
	/*AT6 for rild, new */
	{AT6_CH_C2K, AT6_CH_C2K, CCCI_C2K_AT6, CCCI_C2K_AT6,},
	/*AT7 for rild, new */
	{AT7_CH_C2K, AT7_CH_C2K, CCCI_C2K_AT7, CCCI_C2K_AT7,},
	/*AT8 for rild, new */
	{AT8_CH_C2K, AT8_CH_C2K, CCCI_C2K_AT8, CCCI_C2K_AT8,},
	/*agps channel */
	{AGPS_CH_C2K, AGPS_CH_C2K, CCCI_IPC_UART_TX, CCCI_IPC_UART_RX,},
	{MD2AP_LOOPBACK_C2K, MD2AP_LOOPBACK_C2K, CCCI_C2K_LB_DL,
	CCCI_C2K_LB_DL,},
	{LOOPBACK_C2K, LOOPBACK_C2K, CCCI_LB_IT_TX, CCCI_LB_IT_RX,},
	{STATUS_CH_C2K, STATUS_CH_C2K, CCCI_CONTROL_TX, CCCI_CONTROL_RX,},
};

/*always keep this in mind:
 * what if there are more than 1 modems using CLDMA...
 */

/*ccif share memory setting*/
/*need confirm with md. haow*/

/* for md gen95/97 chip */
static int rx_queue_buffer_size_up_95[QUEUE_NUM] = { 80 * 1024, 80 * 1024,
	40 * 1024, 80 * 1024, 20 * 1024, 20 * 1024, 64 * 1024, 0 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};

static int tx_queue_buffer_size_up_95[QUEUE_NUM] = { 128 * 1024, 40 * 1024,
	8 * 1024, 40 * 1024, 20 * 1024, 20 * 1024, 64 * 1024, 0 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};
static int rx_exp_buffer_size_up_95[QUEUE_NUM] = { 12 * 1024, 32 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 8 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};

static int tx_exp_buffer_size_up_95[QUEUE_NUM] = { 12 * 1024, 32 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 8 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};

/* for md gen98 chip */
static int rx_queue_buffer_size_up_98[QUEUE_NUM] = { 80 * 1024, 80 * 1024,
	40 * 1024, 80 * 1024, 20 * 1024, 20 * 1024, 48 * 1024, 0 * 1024,
	8 * 1024, 16 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};

static int tx_queue_buffer_size_up_98[QUEUE_NUM] = { 128 * 1024, 40 * 1024,
	8 * 1024, 40 * 1024, 20 * 1024, 20 * 1024, 48 * 1024, 0 * 1024,
	8 * 1024, 16 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 0 * 1024,
	0 * 1024, 0 * 1024,
};

static int rx_queue_buffer_size[QUEUE_NUM] = { 80 * 1024, 80 * 1024,
	40 * 1024, 80 * 1024, 20 * 1024, 20 * 1024, 64 * 1024, 0 * 1024,
};

static int tx_queue_buffer_size[QUEUE_NUM] = { 128 * 1024, 40 * 1024,
	8 * 1024, 40 * 1024, 20 * 1024, 20 * 1024, 64 * 1024, 0 * 1024,
};
static int rx_exp_buffer_size[QUEUE_NUM] = { 12 * 1024, 32 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 8 * 1024, 0 * 1024,
};

static int tx_exp_buffer_size[QUEUE_NUM] = { 12 * 1024, 32 * 1024,
	8 * 1024, 0 * 1024, 0 * 1024, 0 * 1024, 8 * 1024, 0 * 1024,
};

#ifdef CCCI_KMODULE_ENABLE
/*
 * for debug log:
 * 0 to disable; 1 for print to ram; 2 for print to uart
 * other value to desiable all log
 */
#ifndef CCCI_LOG_LEVEL /* for platform override */
#define CCCI_LOG_LEVEL CCCI_LOG_CRITICAL_UART
#endif
unsigned int ccci_debug_enable = CCCI_LOG_LEVEL;
//void __iomem *infra_ao_base;
#endif

void ccci_hif_set_devapc_flag(unsigned int value)
{
	devapc_check_flag = value;
}
EXPORT_SYMBOL(ccci_hif_set_devapc_flag);

static void md_ccif_dump(unsigned char *title, unsigned char hif_id)
{
	int idx;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "%s: %s\n", __func__, title);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_CON(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_CON,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_CON));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_BUSY(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_BUSY,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_START(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_START,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_TCHNUM(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_TCHNUM,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_TCHNUM));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_RCHNUM(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_RCHNUM,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "AP_ACK(%p)=0x%x\n",
			md_ctrl->ccif_ap_base + APCCIF_ACK,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_ACK));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_CON(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_CON,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_CON));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_BUSY(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_BUSY,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_BUSY));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_START(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_START,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_START));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_TCHNUM(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_TCHNUM,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_TCHNUM));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_RCHNUM(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_RCHNUM,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_RCHNUM));
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG, "MD_ACK(%p)=0x%x\n",
			md_ctrl->ccif_md_base + APCCIF_ACK,
			ccif_read32(md_ctrl->ccif_md_base, APCCIF_ACK));

	for (idx = 0;
		 idx < md_ctrl->sram_size / sizeof(u32);
		 idx += 4) {
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
				 "CHDATA(%p): %08X %08X %08X %08X\n",
				 md_ctrl->ccif_ap_base + APCCIF_CHDATA +
				 idx * sizeof(u32),
				 ccif_read32(md_ctrl->ccif_ap_base +
						APCCIF_CHDATA,
						(idx + 0) * sizeof(u32)),
				 ccif_read32(md_ctrl->ccif_ap_base +
						APCCIF_CHDATA,
						(idx + 1) * sizeof(u32)),
				 ccif_read32(md_ctrl->ccif_ap_base +
						APCCIF_CHDATA,
						(idx + 2) * sizeof(u32)),
				 ccif_read32(md_ctrl->ccif_ap_base +
						APCCIF_CHDATA,
						(idx + 3) * sizeof(u32)));
	}

}

static void md_ccif_queue_dump(unsigned char hif_id)
{
	int idx;
	unsigned long long ts = 0;
	unsigned long nsec_rem = 0;

	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (!md_ctrl || !md_ctrl->rxq[0].ringbuf)
		return;

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump md_ctrl->channel_id 0x%lx\n",
		md_ctrl->channel_id);
	ts = md_ctrl->traffic_info.latest_isr_time;
	nsec_rem = do_div(ts, NSEC_PER_SEC);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump CCIF latest isr %5llu.%06lu\n", ts,
		nsec_rem / 1000);
#ifdef DEBUG_FOR_CCB
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump CCIF latest r_ch: 0x%x\n",
		md_ctrl->traffic_info.last_ccif_r_ch);
	ts = md_ctrl->traffic_info.latest_ccb_isr_time;
	nsec_rem = do_div(ts, NSEC_PER_SEC);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump CCIF latest ccb_isr %5llu.%06lu\n", ts,
		nsec_rem / 1000);
#endif

	for (idx = 0; idx < QUEUE_NUM; idx++) {
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d TX: w=%d, r=%d, len=%d, %p\n",
		idx, md_ctrl->txq[idx].ringbuf->tx_control.write,
		md_ctrl->txq[idx].ringbuf->tx_control.read,
		md_ctrl->txq[idx].ringbuf->tx_control.length,
		md_ctrl->txq[idx].ringbuf);
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d RX: w=%d, r=%d, len=%d, isr_cnt=%lld\n",
		idx, md_ctrl->rxq[idx].ringbuf->rx_control.write,
		md_ctrl->rxq[idx].ringbuf->rx_control.read,
		md_ctrl->rxq[idx].ringbuf->rx_control.length,
		md_ctrl->isr_cnt[idx]);
		ts = md_ctrl->traffic_info.latest_q_rx_isr_time[idx];
		nsec_rem = do_div(ts, NSEC_PER_SEC);
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d RX: last isr %5llu.%06lu\n", idx, ts,
		nsec_rem / 1000);
		ts = md_ctrl->traffic_info.latest_q_rx_time[idx];
		nsec_rem = do_div(ts, NSEC_PER_SEC);
		CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d RX: last wq  %5llu.%06lu\n", idx, ts,
		nsec_rem / 1000);
	}
	ccci_md_dump_log_history(md_ctrl->md_id,
		&md_ctrl->traffic_info, 1, QUEUE_NUM, QUEUE_NUM);
}

static void md_ccif_dump_queue_history(unsigned char hif_id, unsigned int qno)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (!md_ctrl || !md_ctrl->rxq[qno].ringbuf)
		return;

	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump md_ctrl->channel_id 0x%lx\n", md_ctrl->channel_id);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Dump CCIF Queue%d Control\n", qno);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d TX: w=%d, r=%d, len=%d\n",
		qno, md_ctrl->txq[qno].ringbuf->tx_control.write,
		md_ctrl->txq[qno].ringbuf->tx_control.read,
		md_ctrl->txq[qno].ringbuf->tx_control.length);
	CCCI_MEM_LOG_TAG(md_ctrl->md_id, TAG,
		"Q%d RX: w=%d, r=%d, len=%d\n",
		qno, md_ctrl->rxq[qno].ringbuf->rx_control.write,
		md_ctrl->rxq[qno].ringbuf->rx_control.read,
		md_ctrl->rxq[qno].ringbuf->rx_control.length);
	ccci_md_dump_log_history(md_ctrl->md_id,
		&md_ctrl->traffic_info, 0, qno, qno);
}

static void md_cd_dump_ccif_reg(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int idx;

	/* B15R (log-only): post-EE TXQ1 snapshot. This dump hook PROVABLY
	 * executes on the EE path ("Dump CCIF REG" rows at 91.8/105.9/118.0
	 * in B15 logs). Read-only; no behavior change. */
	if (ccif_ctrl && ccif_ctrl->txq[1].ringbuf) {
		CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
			"B15R postee txq1: read=%u write=%u len=%u\n",
			(unsigned int)ccif_ctrl->txq[1].ringbuf->tx_control.read,
			(unsigned int)ccif_ctrl->txq[1].ringbuf->tx_control.write,
			(unsigned int)ccif_ctrl->txq[1].ringbuf->tx_control.length);
	}

	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_CON(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_CON,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_CON));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_BUSY(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_BUSY,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_BUSY));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_START(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_START,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_START));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_TCHNUM(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_TCHNUM,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_TCHNUM));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_RCHNUM(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_RCHNUM,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_RCHNUM));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "AP_ACK(%p)=%x\n",
		ccif_ctrl->ccif_ap_base + APCCIF_ACK,
		ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_ACK));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_CON(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_CON,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_CON));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_BUSY(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_BUSY,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_BUSY));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_START(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_START,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_START));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_TCHNUM(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_TCHNUM,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_TCHNUM));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_RCHNUM(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_RCHNUM,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_RCHNUM));
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG, "MD_ACK(%p)=%x\n",
		ccif_ctrl->ccif_md_base + APCCIF_ACK,
		ccif_read32(ccif_ctrl->ccif_md_base, APCCIF_ACK));

	for (idx = 0; idx < ccif_ctrl->sram_size / sizeof(u32);
		idx += 4) {
		CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG,
			"CHDATA(%p): %08X %08X %08X %08X\n",
			ccif_ctrl->ccif_ap_base + APCCIF_CHDATA +
			idx * sizeof(u32),
			ccif_read32(ccif_ctrl->ccif_ap_base + APCCIF_CHDATA,
				(idx + 0) * sizeof(u32)),
			ccif_read32(ccif_ctrl->ccif_ap_base + APCCIF_CHDATA,
				(idx + 1) * sizeof(u32)),
			ccif_read32(ccif_ctrl->ccif_ap_base + APCCIF_CHDATA,
				(idx + 2) * sizeof(u32)),
			ccif_read32(ccif_ctrl->ccif_ap_base + APCCIF_CHDATA,
				(idx + 3) * sizeof(u32)));
	}
}

static int ccif_debug_dump_data(unsigned int hif_id, int *buff, int length)
{
	int i;
	unsigned int *dest_buff = NULL;
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int sram_size = ccif_ctrl->sram_size;

	if (!buff || length < 0 || length > sram_size)
		return 0;

	dest_buff = (unsigned int *)buff;

	for (i = 0; i < length / sizeof(unsigned int); i++) {
		*(dest_buff + i) = ccif_read32(ccif_ctrl->ccif_ap_base,
			APCCIF_CHDATA + (sram_size - length) +
			i * sizeof(unsigned int));
	}
	CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG,
		"Dump CCIF SRAM (last %d bytes)\n", length);
	ccci_util_mem_dump(ccif_ctrl->md_id,
		CCCI_DUMP_MEM_DUMP, dest_buff, length);

	return 0;
}

static int md_ccif_op_dump_status(unsigned char hif_id,
	enum MODEM_DUMP_FLAG flag, void *buff, int length)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (!ccif_ctrl)
		return -1;

	if (ccif_ctrl->ccif_state == HIFCCIF_STATE_PWROFF
		|| ccif_ctrl->ccif_state == HIFCCIF_STATE_MIN) {
		CCCI_MEM_LOG_TAG(ccif_ctrl->md_id, TAG,
			"CCIF not power on, skip dump\n");
		return -2;
	}

	/*runtime data, boot, long time no response EE */
	if (flag & DUMP_FLAG_CCIF) {
		ccif_debug_dump_data(hif_id, buff, length);
		md_ccif_dump("Dump CCIF SRAM\n", hif_id);
		md_ccif_queue_dump(hif_id);
	}
	if (flag & DUMP_FLAG_IRQ_STATUS) {
#if IS_ENABLED(CONFIG_MTK_IRQ_DBG)
		CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
		"Dump AP CCIF IRQ status\n");
		mt_irq_dump_status(ccif_ctrl->ap_ccif_irq0_id);
		mt_irq_dump_status(ccif_ctrl->ap_ccif_irq1_id);
#else
		CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
			"Dump AP CCIF IRQ status not support\n");
#endif
	}
	if (flag & DUMP_FLAG_QUEUE_0)
		md_ccif_dump_queue_history(hif_id, 0);
	if (flag & DUMP_FLAG_QUEUE_0_1) {
		md_ccif_dump_queue_history(hif_id, 0);
		md_ccif_dump_queue_history(hif_id, 1);
	}
	if (flag & (DUMP_FLAG_CCIF_REG | DUMP_FLAG_REG))
		md_cd_dump_ccif_reg(hif_id);
	if (flag & DUMP_FLAG_GET_TRAFFIC) {
		if (buff && length == 24) { /* u64 * 3 */
			unsigned long long *dest_buff = (unsigned long long *)buff;

			dest_buff[0] = ccif_ctrl->traffic_info.latest_isr_time;
			dest_buff[1] = ccif_ctrl->traffic_info.latest_q_rx_isr_time[0];
			dest_buff[2] = ccif_ctrl->traffic_info.latest_q_rx_time[0];
		}
	}
	return 0;
}

/*direction: 1: tx; 0: rx*/
static int c2k_ch_to_ccci_ch(int c2k_ch, int direction)
{
	u16 c2k_channel_id;
	int i = 0;

	c2k_channel_id = (u16) c2k_ch;
	for (i = 0; i < (sizeof(c2k_ports) / sizeof(struct c2k_port)); i++) {
		if (c2k_channel_id == c2k_ports[i].ch) {
			CCCI_DEBUG_LOG(MD_SYS3, TAG,
				"%s:channel(%d)-->(T%d R%d)\n",
				(direction == OUT) ? "TX" : "RX", c2k_ch,
				c2k_ports[i].tx_ch_mapping,
				c2k_ports[i].rx_ch_mapping);
			return (direction == OUT) ? c2k_ports[i].tx_ch_mapping :
				c2k_ports[i].rx_ch_mapping;
		}
	}

	CCCI_ERROR_LOG(MD_SYS3, TAG,
		"%s:ERR cannot find mapped c2k ch ID(%d)\n",
		direction ? "TX" : "RX", c2k_ch);
	return CCCI_OVER_MAX_CH;
}

static int ccci_ch_to_c2k_ch(int md_state, int ccci_ch, int direction)
{
	u16 ccci_channel_id;
	u16 channel_map;
	int i = 0;

	ccci_channel_id = (u16) ccci_ch;
	for (i = 0; i < (sizeof(c2k_ports) / sizeof(struct c2k_port)); i++) {
		channel_map = (direction == OUT) ? c2k_ports[i].tx_ch_mapping :
			c2k_ports[i].rx_ch_mapping;

		if (ccci_channel_id == channel_map) {
			CCCI_DEBUG_LOG(MD_SYS3, TAG, "%s:channel(%d)-->(%d)\n",
					(direction == OUT) ? "TX" : "RX",
					ccci_channel_id, c2k_ports[i].ch);
			return (md_state != EXCEPTION) ? c2k_ports[i].ch :
				c2k_ports[i].excp_ch;
		}
	}

	CCCI_ERROR_LOG(MD_SYS3, TAG,
		"%s:ERR cannot find mapped ccci ch ID(%d)\n",
		direction ? "TX" : "RX", ccci_ch);
	return C2K_OVER_MAX_CH;
}

static inline void ccci_md_check_rx_seq_num(unsigned char md_id,
	struct ccci_hif_traffic *traffic_info,
	struct ccci_header *ccci_h, int qno)
{
	u16 channel, seq_num, assert_bit;
	unsigned int param[3] = {0};

	channel = ccci_h->channel;
	seq_num = ccci_h->seq_num;
	assert_bit = ccci_h->assert_bit;

	if (assert_bit && traffic_info->seq_nums[IN][channel] != 0
		&& ((seq_num - traffic_info->seq_nums[IN][channel])
		& 0x7FFF) != 1) {
		CCCI_ERROR_LOG(md_id, CORE,
			"channel %d seq number out-of-order %d->%d (data: %X, %X)\n",
			channel, seq_num, traffic_info->seq_nums[IN][channel],
			ccci_h->data[0], ccci_h->data[1]);
		md_ccif_op_dump_status(CCIF_HIF_ID, DUMP_FLAG_CCIF, NULL, qno);
		param[0] = channel;
		param[1] = traffic_info->seq_nums[IN][channel];
		param[2] = seq_num;
		ccci_md_force_assert(md_id, MD_FORCE_ASSERT_BY_MD_SEQ_ERROR,
			(char *)param, sizeof(param));

	} else {
		traffic_info->seq_nums[IN][channel] = seq_num;
	}
}

static void md_ccif_sram_rx_work(struct work_struct *work)
{
	struct md_ccif_ctrl *md_ctrl =
		container_of(work, struct md_ccif_ctrl, ccif_sram_work);
	struct ccci_header *dl_pkg =
		&md_ctrl->ccif_sram_layout->dl_header;
	struct ccci_header *ccci_h;
	struct ccci_header ccci_hdr;
	struct sk_buff *skb = NULL;
	int pkg_size, ret = 0, retry_cnt = 0;
	int c2k_to_ccci_ch = 0;

	u32 i = 0;
	u8 *md_feature = (u8 *)(&md_ctrl->ccif_sram_layout->md_rt_data);

	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"%s:dk_pkg=%p, md_featrue=%p\n", __func__,
		dl_pkg, md_feature);
	pkg_size =
		sizeof(struct ccci_header) + sizeof(struct md_query_ap_feature);

	skb = ccci_alloc_skb(pkg_size, 1, 1);
	if (skb == NULL) {
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"%s: alloc skb size=%d, failed\n", __func__,
			pkg_size);
		return;
	}
	skb_put(skb, pkg_size);
	ccci_h = (struct ccci_header *)skb->data;
	ccci_h->data[0] = ccif_read32(&dl_pkg->data[0], 0);
	ccci_h->data[1] = ccif_read32(&dl_pkg->data[1], 0);
	/*ccci_h->channel = ccif_read32(&dl_pkg->channel,0); */
	*(((u32 *) ccci_h) + 2) = ccif_read32((((u32 *) dl_pkg) + 2), 0);
	if (md_ctrl->md_id == MD_SYS3) {
		c2k_to_ccci_ch = c2k_ch_to_ccci_ch(ccci_h->channel, IN);
		ccci_h->channel = (u16) c2k_to_ccci_ch;
	}
	ccci_h->reserved = ccif_read32(&dl_pkg->reserved, 0);

	/*warning: make sure struct md_query_ap_feature is 4 bypes align */
	while (i < sizeof(struct md_query_ap_feature)) {
		*((u32 *) (skb->data + sizeof(struct ccci_header) + i))
		= ccif_read32(md_feature, i);
		i += 4;
	}

	if (test_and_clear_bit((D2H_SRAM), &md_ctrl->wakeup_ch)) {
		CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
			"CCIF_MD wakeup source:(SRX_IDX/%d)(%u), HS1\n",
			ccci_h->channel, md_ctrl->wakeup_count);
	}
	ccci_hdr = *ccci_h;
	ccci_md_check_rx_seq_num(md_ctrl->md_id,
		&md_ctrl->traffic_info, &ccci_hdr, 0);

 RETRY:
	ret = ccci_port_recv_skb(md_ctrl->md_id, md_ctrl->hif_id, skb,
			NORMAL_DATA);
	CCCI_DEBUG_LOG(md_ctrl->md_id, TAG, "Rx msg %x %x %x %x ret=%d\n",
		ccci_hdr.data[0], ccci_hdr.data[1],
		*(((u32 *)&ccci_hdr) + 2), ccci_hdr.reserved, ret);
	if (ret >= 0 || ret == -CCCI_ERR_DROP_PACKET) {
		CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
			"%s:ccci_port_recv_skb ret=%d\n", __func__,
			ret);
	} else {
		if (retry_cnt < 20) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"%s:ccci_md_recv_skb ret=%d,retry=%d\n", __func__,
			ret, retry_cnt);
			udelay(5);
			retry_cnt++;
			goto RETRY;
		}
		ccci_free_skb(skb);
		CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"%s:ccci_port_recv_skb ret=%d\n", __func__, ret);
	}
}

static void c2k_mem_dump(void *start_addr, int len)
{
	unsigned int *curr_p = (unsigned int *)start_addr;
	unsigned char *curr_ch_p;
	int _16_fix_num = len / 16;
	int tail_num = len % 16;
	char buf[16];
	int i, j;

	if (curr_p == NULL) {
		CCCI_ERROR_LOG(MD_SYS3, TAG, "[C2K-DUMP]NULL point to dump!\n");
		return;
	}
	if (len == 0) {
		CCCI_ERROR_LOG(MD_SYS3, TAG, "[C2K-DUMP]Not need to dump\n");
		return;
	}

	CCCI_DEBUG_LOG(MD_SYS3, TAG, "[C2K-DUMP]Base: 0x%lx, len: %d\n",
		(unsigned long)start_addr, len);
	/*Fix section */
	for (i = 0; i < _16_fix_num; i++) {
		CCCI_MEM_LOG(MD_SYS3, TAG,
			"[C2K-DUMP]%03X: %08X %08X %08X %08X\n",
			i * 16, *curr_p, *(curr_p + 1),
			*(curr_p + 2), *(curr_p + 3));
		curr_p += 4;
	}

	/*Tail section */
	if (tail_num > 0) {
		curr_ch_p = (unsigned char *)curr_p;
		for (j = 0; j < tail_num; j++) {
			buf[j] = *curr_ch_p;
			curr_ch_p++;
		}
		for (; j < 16; j++)
			buf[j] = 0;
		curr_p = (unsigned int *)buf;
		CCCI_MEM_LOG(MD_SYS3, TAG,
			"[C2K-DUMP]%03X: %08X %08X %08X %08X\n",
			i * 16, *curr_p, *(curr_p + 1),
			*(curr_p + 2), *(curr_p + 3));
	}
}

static int ccif_check_flow_ctrl(struct md_ccif_ctrl *md_ctrl,
	struct md_ccif_queue *queue, struct ccci_ringbuf *rx_buf)
{
	int is_busy, buf_size = 0;
	int ret = 0;

	is_busy = ccif_is_md_queue_busy(md_ctrl, queue->index);
	if (is_busy < 0) {
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"ccif flow ctrl: check modem return %d\n",
			is_busy);
		return 0;
	}
	if (is_busy > 0) {
		buf_size = rx_buf->rx_control.write - rx_buf->rx_control.read;
		if (buf_size < 0)
			buf_size += rx_buf->rx_control.length;
		if (queue->resume_cnt < FLOW_CTRL_THRESHOLD &&
			buf_size <= rx_buf->rx_control.length /
			(2 << (queue->resume_cnt * 2))) {
			if (ccci_fsm_get_md_state(md_ctrl->md_id) == READY) {
				ret = ccci_port_send_msg_to_md(md_ctrl->md_id,
						CCCI_CONTROL_TX,
						C2K_FLOW_CTRL_MSG,
						queue->index, 0);
				if (ret < 0)
					CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
						"fail to resume md Q%d, ret0x%x\n",
						queue->index, ret);
				else {
					queue->resume_cnt++;
					CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
						"flow ctrl: resume Q%d, buf %d, cnt %d\n",
						queue->index, buf_size,
						queue->resume_cnt);
				}
			}
		}
	} else
		queue->resume_cnt = 0;

	return ret;
}

static void md_ccif_traffic_work_func(struct work_struct *work)
{
	struct ccci_hif_traffic *traffic_inf =
		container_of(work, struct ccci_hif_traffic,
			traffic_work_struct);
	struct md_ccif_ctrl *md_ctrl =
		container_of(traffic_inf, struct md_ccif_ctrl, traffic_info);
	char *string = NULL;
	char *string_temp = NULL;
	int idx, ret;

	ccci_port_dump_status(md_ctrl->md_id);
	ccci_channel_dump_packet_counter(md_ctrl->md_id,
		&md_ctrl->traffic_info);
	/*pre_cnt for tx, pkt_cont for rx*/
	if (md_ctrl->md_id == MD_SYS3) {
		CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
		"traffic(AT): tx:[%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld\n",
		CCCI_C2K_AT,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT],
		CCCI_C2K_AT2,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT2],
		CCCI_C2K_AT3,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT3],
		CCCI_C2K_AT4,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT4],
		CCCI_C2K_AT5,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT5],
		CCCI_C2K_AT6,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT6],
		CCCI_C2K_AT7,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT7],
		CCCI_C2K_AT8,
		md_ctrl->traffic_info.logic_ch_pkt_pre_cnt[CCCI_C2K_AT8]);
		CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
		"traffic(AT): rx:[%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld, [%d]%ld\n",
		CCCI_C2K_AT,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT],
		CCCI_C2K_AT2,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT2],
		CCCI_C2K_AT3,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT3],
		CCCI_C2K_AT4,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT4],
		CCCI_C2K_AT5,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT5],
		CCCI_C2K_AT6,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT6],
		CCCI_C2K_AT7,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT7],
		CCCI_C2K_AT8,
		md_ctrl->traffic_info.logic_ch_pkt_cnt[CCCI_C2K_AT8]);
	} else {
		string = kmalloc(1024, GFP_ATOMIC);
		string_temp = kmalloc(1024, GFP_ATOMIC);
		if (string == NULL || string_temp == NULL) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"Fail alloc traffic Mem for isr cnt!\n");
			goto err_exit1;
		}

		ret = snprintf(string, 1024, "total cnt=%lld;",
			md_ctrl->traffic_info.isr_cnt);
		if (ret < 0 || ret >= 1024) {
			CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
				"string buffer fail %d", ret);
		}
		for (idx = 0; idx < CCIF_CH_NUM; idx++) {
			ret = snprintf(string_temp, 1024,
				"%srxq%d isr_cnt=%llu;",	string, idx,
				md_ctrl->isr_cnt[idx]);
			if (ret < 0 || ret >= 1024) {
				CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
					"string_temp buffer full %d", ret);
			}
			ret = snprintf(string, 1024, "%s", string_temp);
			if (ret < 0 || ret >= 1024) {
				CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
					"string buffer full %d", ret);
				break;
			}
		}
		CCCI_NORMAL_LOG(md_ctrl->md_id, TAG, "%s\n", string);

	}
err_exit1:
	kfree(string);
	kfree(string_temp);

	mod_timer(&md_ctrl->traffic_monitor,
		jiffies + CCIF_TRAFFIC_MONITOR_INTERVAL * HZ);
}

static void md_ccif_traffic_monitor_func(struct timer_list *t)
{
	struct md_ccif_ctrl *md_ctrl = container_of(t, struct md_ccif_ctrl, traffic_monitor);

	schedule_work(&md_ctrl->traffic_info.traffic_work_struct);
}

atomic_t lb_dl_q;
/*this function may be called from both workqueue and softirq (NAPI)*/
static unsigned long rx_data_cnt;
static unsigned int pkg_num;

static int ccif_rx_collect(struct md_ccif_queue *queue, int budget,
	int blocking, int *result)
{

	struct ccci_ringbuf *rx_buf = queue->ringbuf;
	unsigned char *data_ptr;
	int ret = 0, count = 0, pkg_size;
	unsigned long flags;
	int qno = queue->index;
	struct ccci_header *ccci_h = NULL;
	struct ccci_header ccci_hdr;
	struct sk_buff *skb;
	int c2k_to_ccci_ch = 0;
	unsigned char from_pool;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(queue->hif_id);
	struct ccci_per_md *per_md_data =
		ccci_get_per_md_data(md_ctrl->md_id);

	if (atomic_read(&queue->rx_on_going)) {
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"Q%d rx is on-going(%d)1\n",
			queue->index, atomic_read(&queue->rx_on_going));
		*result = 0;
		return -1;
	}
	atomic_set(&queue->rx_on_going, 1);

	if (IS_PASS_SKB(per_md_data, qno))
		from_pool = 0;
	else
		from_pool = 1;

	while (1) {
		md_ctrl->traffic_info.latest_q_rx_time[qno] = local_clock();
		spin_lock_irqsave(&queue->rx_lock, flags);
		pkg_size = ccci_ringbuf_readable(md_ctrl->md_id, rx_buf);
		spin_unlock_irqrestore(&queue->rx_lock, flags);
		if (pkg_size < 0) {
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"Q%d Rx:rbf readable ret=%d\n",
				queue->index, pkg_size);
			ret = 0;
			goto OUT;
		}

		skb = ccci_alloc_skb(pkg_size, from_pool, blocking);

		if (skb == NULL) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				       "Q%d Rx:ccci_alloc_skb pkg_size=%d failed,count=%d\n",
				       queue->index, pkg_size, count);
			ret = -ENOMEM;
			goto OUT;
		}

		data_ptr = (unsigned char *)skb_put(skb, pkg_size);
		/*copy data into skb */
		spin_lock_irqsave(&queue->rx_lock, flags);
		ret = ccci_ringbuf_read(md_ctrl->md_id, rx_buf,
				data_ptr, pkg_size);
		spin_unlock_irqrestore(&queue->rx_lock, flags);
		if (unlikely(ret < 0)) {
			ccci_free_skb(skb);
			goto OUT;
		}
		ccci_h = (struct ccci_header *)skb->data;
		if (md_ctrl->md_id == MD_SYS3) {
			/* md3(c2k) logical channel number is not
			 * the same as other modems,
			 * so we need use mapping table to
			 * convert channel id here.
			 */
			c2k_to_ccci_ch = c2k_ch_to_ccci_ch(ccci_h->channel, IN);
			ccci_h->channel = (u16) c2k_to_ccci_ch;

			/* heart beat msg from c2k control channel,
			 * but handled by ECCCI status channel handler,
			 * we hack the channel ID here.
			 */
			if (ccci_h->channel == CCCI_C2K_LB_DL) {
				CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
					"Q%d Rx lb_dl\n", queue->index);
				c2k_mem_dump(data_ptr, pkg_size);
			}
		}
		if (test_and_clear_bit(queue->index, &md_ctrl->wakeup_ch)) {
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
				"CCIF_MD wakeup source:(%d/%d/%x)(%u) %s\n",
				queue->index, ccci_h->channel,
				ccci_h->reserved, md_ctrl->wakeup_count,
				ccci_port_get_dev_name(ccci_h->channel));
			if (ccci_h->channel == CCCI_FS_RX)
				ccci_h->data[0] |= CCCI_FS_AP_CCCI_WAKEUP;
			else if (ccci_h->channel >= CCCI_MIPC0_CHANNEL_RX &&
				ccci_h->channel <= CCCI_MIPC9_CHANNEL_RX) {
				/*
				 * MIPC message data struct:
				 * typedef struct {
				 * u32 magic; u16 padding[2]; u8 msg_sim_ps_id;
				 * u8 msg_flag;
				 * u16 msg_id;  //log to show,offset is 10bytes
				 * u16 msg_txid; u16 msg_len;} mipc_msg_hdr_t;
				 */
				CCCI_NOTICE_LOG(0, TAG,
					"%s:CCCI_MIPC ch%d wakeup,msg_id=0x%x\n",
					__func__, ccci_h->channel,
					*(unsigned short *)((unsigned char *)skb->data +
						sizeof(struct ccci_header) + 10));
			}
		}
		if (ccci_h->channel == CCCI_C2K_LB_DL)
			atomic_set(&lb_dl_q, queue->index);

		ccci_hdr = *ccci_h;

		ret = ccci_port_recv_skb(md_ctrl->md_id,
			queue->hif_id, skb, NORMAL_DATA);

		if (ret >= 0 || ret == -CCCI_ERR_DROP_PACKET) {
			count++;
			ccci_md_check_rx_seq_num(md_ctrl->md_id,
				&md_ctrl->traffic_info,
				&ccci_hdr, queue->index);
			ccci_md_add_log_history(&md_ctrl->traffic_info, IN,
				(int)queue->index, &ccci_hdr,
				(ret >= 0 ? 0 : 1));
			ccci_channel_update_packet_counter(
				md_ctrl->traffic_info.logic_ch_pkt_cnt,
				&ccci_hdr);

			if (queue->debug_id) {
				CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
					"Q%d Rx recv req ret=%d\n",
					queue->index, ret);
				queue->debug_id = 0;
			}
			spin_lock_irqsave(&queue->rx_lock, flags);
			ccci_ringbuf_move_rpointer(md_ctrl->md_id,
				rx_buf, pkg_size);
			spin_unlock_irqrestore(&queue->rx_lock, flags);
			if (likely(ccci_md_get_cap_by_id(md_ctrl->md_id)
					& MODEM_CAP_TXBUSY_STOP))
				ccif_check_flow_ctrl(md_ctrl, queue, rx_buf);

			if (ccci_hdr.channel == CCCI_MD_LOG_RX) {
				rx_data_cnt += pkg_size - 16;
				pkg_num++;
				CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
					    "Q%d Rx buf read=%d, write=%d, pkg_size=%d, log_cnt=%ld, pkg_num=%d\n",
					    queue->index,
					    rx_buf->rx_control.read,
					    rx_buf->rx_control.write, pkg_size,
						rx_data_cnt, pkg_num);
			}
			ret = 0;
		} else {
			/*leave package into share memory,
			 * and waiting ccci to receive
			 */
			ccci_free_skb(skb);

			if (queue->debug_id == 0) {
				queue->debug_id = 1;
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"Q%d Rx err, ret = 0x%x\n",
					queue->index, ret);
			}

			goto OUT;
		}
		if (count > budget) {
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"Q%d count > budget, exit now\n",
				queue->index);
			goto OUT;
		}
	}

 OUT:
	atomic_set(&queue->rx_on_going, 0);
	*result = count;
	CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
		"Q%d rx %d pkg,ret=%d\n",
		queue->index, count, ret);
	spin_lock_irqsave(&queue->rx_lock, flags);
	if (ret != -CCCI_ERR_PORT_RX_FULL
		&& ret != -EAGAIN) {
		pkg_size = ccci_ringbuf_readable(md_ctrl->md_id, rx_buf);
		if (pkg_size > 0)
			ret = -EAGAIN;
	}
	spin_unlock_irqrestore(&queue->rx_lock, flags);
	return ret;
}

static void ccif_rx_work(struct work_struct *work)
{
	int result = 0, ret = 0;
	struct md_ccif_queue *queue =
		container_of(work, struct md_ccif_queue, qwork);
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(queue->hif_id);

	ret = ccif_rx_collect(queue, queue->budget, 1, &result);
	if (ret == -EAGAIN) {
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"Q%u queue again\n", queue->index);
		queue_work(queue->worker, &queue->qwork);
	} else {
		ccci_port_queue_status_notify(md_ctrl->md_id, queue->hif_id,
			queue->index, IN, RX_FLUSH);
	}
}

void ccif_polling_ready(unsigned char hif_id, int step)
{
	int cnt = 500; /*MD timeout is 10s*/
	int time_once = 10;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

#ifdef CCCI_EE_HS_POLLING_TIME
	cnt = CCCI_EE_HS_POLLING_TIME / time_once;
#endif
	while (cnt > 0) {
		if (md_ctrl->channel_id & (1 << step)) {
			clear_bit(step, &md_ctrl->channel_id);
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"poll RCHNUM %ld\n", md_ctrl->channel_id);
			return;
		}
		msleep(time_once);
		cnt--;
	}
	CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
		"poll EE HS timeout, RCHNUM %ld\n",
		md_ctrl->channel_id);
}

/* DIAGNOSTIC 20260922 (v428): one-shot dump of the whole CCIF window register
 * set on both views.  Read-only.  Used to establish, empirically, which register
 * bit the modem's ccismc_polling_submit_one_gpd is really waiting on
 * (modem 0xC020A010 == AP ccif_md_base + APCCIF_RCHNUM, see DT reg[1]).
 */
/* v552: the project's CCIF-DIAG instrumentation is OFF by default because it
 * is behaviour-perturbing, not inert.  The ch15 trace in md_ccif_send() runs
 * 500x udelay(1) plus ~5000 MMIO reads while holding the queue tx spinlock
 * with IRQs off, and the per-send SENT/DOORBELL lines flood the kernel ring
 * buffer - in the v544 run 31552 of 34275 dmesg lines were ccci1 noise and
 * every boot-time message before t=44 s had been evicted.  Enable explicitly
 * with ccci_ccif_diag=1 for a dedicated diagnostic window.
 */
static bool ccif_diag_enable;
module_param_named(ccci_ccif_diag, ccif_diag_enable, bool, 0644);
MODULE_PARM_DESC(ccci_ccif_diag,
	"v552: enable CCIF-DIAG instrumentation (default off)");

/*
 * v698 DIAGNOSTIC (read-only, behaviour-neutral): select which channel gets the
 * 500x1us WIN-SRAM1US window sampler.  The sampler was hard-coded to channel 15,
 * so the q5 (CCCI_UART2 / AT-MIPC) doorbell-delivery question could not be
 * answered: a single immediate WIN-SENT sample is taken before the modem can
 * possibly latch the channel.  Default -1 keeps the historical ch15 behaviour,
 * so this parameter is inert unless explicitly set.
 */
static int ccif_win_ch = -1;
module_param_named(ccci_ccif_win_ch, ccif_win_ch, int, 0644);
MODULE_PARM_DESC(ccci_ccif_win_ch,
	"v698: channel for the WIN-SRAM1US sampler (-1 = 15, i.e. legacy)");

static void ccif_diag_win(struct md_ccif_ctrl *md_ctrl, const char *tag, int ch)
{
	if (!ccif_diag_enable)
		return;
	CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
		"CCIF-DIAG %s ch=%d | AP: B=0x%x S=0x%x T=0x%x R=0x%x A=0x%x | MD: B=0x%x S=0x%x T=0x%x R=0x%x A=0x%x\n",
		tag, ch,
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_TCHNUM),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_ACK),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_TCHNUM),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_ACK));
}

/* v443 DIAGNOSTIC: defined later; used by ccci_reset_ccif_hw below. */
static void ccif_dump_bank(struct md_ccif_ctrl *md_ctrl, const char *tag);

static int md_ccif_send(unsigned char hif_id, int channel_id)
{
	int busy = 0;
	unsigned int busy_after;
	/* DIAGNOSTIC 20260922 (v426/v427): attribute the AP->MD CCIF window that
	 * never retires to a specific send.
	 *
	 * BEHAVIOUR NOTE: the return contract is deliberately left EXACTLY as
	 * vendor (always 0) so that this run is a pure observation.  The refusal
	 * is derived from the BUSY bitmap by the caller instead of by changing
	 * this function's return type/semantics.
	 */
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (md_ctrl == NULL)
		return -1;

	busy = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY);
	if (busy & (1 << channel_id)) {
		if (ccif_diag_enable)
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"CCIF-DIAG REFUSED ch=%d BUSY=0x%x START=0x%x\n",
				channel_id, (unsigned int)busy,
				ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START));
		CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
			"CCIF channel %d busy\n", channel_id);
		return 0;
	}
	/*
	 * v437 EXPERIMENT RESULT -- FALSIFIED, sequence restored to stock.
	 * Skipping the manual BUSY write for H2D_SRAM (15) and issuing only
	 * TCHNUM = 15 produced NO effect at all: BUSY_after = 0x0, START_after =
	 * 0x0, and the 500-sample WIN-SRAM1US window showed MD_RCHNUM bit15 never
	 * set (MD view only 0x0/0x2), with the same +43.513 s line-2004 assert.
	 * So the manual BUSY write is NOT the defect -- it is the only thing that
	 * makes the AP attempt anything on ch15 at all.  Conclusion (RESULT.md 18):
	 * no AP-side TCHNUM write can drive ch15, consistently with
	 * CCIF_HW_CH_RX_RESERVED = (1<<15)|(1<<20) (ch15 is an RX-reserved class
	 * channel).  The stock BUSY+TCHNUM sequence is therefore kept.
	 */
	ccif_write32(md_ctrl->ccif_ap_base,
		APCCIF_BUSY, 1 << channel_id);
	ccif_write32(md_ctrl->ccif_ap_base,
		APCCIF_TCHNUM, channel_id);
	busy_after = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY);
	if (ccif_diag_enable) {
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"CCIF-DIAG SENT ch=%d BUSY_before=0x%x BUSY_after=0x%x START_after=0x%x TCHNUM=0x%x\n",
			channel_id, (unsigned int)busy, busy_after,
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START),
			ccif_read32(md_ctrl->ccif_ap_base, APCCIF_TCHNUM));
		ccif_diag_win(md_ctrl, "WIN-SENT", channel_id);
	}
	/* DIAGNOSTIC 20260923 (v432): 1 us resolution around the ch15 doorbell.
	 * Earlier 50 us sampling showed BUSY bit15 appears and is gone within
	 * ~70 us, so the pulse width was never resolved.  This samples both views
	 * every ~1 us for ~500 us to answer the decisive question: does the
	 * AP->MD SRAM doorbell ever become visible on the MD-side RCHNUM at all?
	 * Read-only; 500 us in process context.
	 */
	if (channel_id == (ccif_win_ch < 0 ? 15 : ccif_win_ch) &&
	    ccif_diag_enable) {
		int k;

		for (k = 0; k < 500; k++) {
			ccif_diag_win(md_ctrl, "WIN-SRAM1US", channel_id);
			udelay(1);
		}
	}
	CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
		"CCIF start=0x%x\n",
		ccif_read32(md_ctrl->ccif_ap_base,
			APCCIF_START));
	return 0;
}

/*
 * v438 DIAGNOSTIC (read-only, behaviour-neutral): observe the CCISM ring
 * control state and the CCIF register pair across the HS1 -> 43.5 s window, so
 * the live modem polling phase can be classified without perturbing anything.
 *
 * Rationale (RESULT.md 19.2): md_ccif_ring_buf_init()/md_ccif_exp_ring_buf_init()
 * pack 16 rings into SMEM_USER_CCISM_MCU / _EXP and set txq[i].ccif_ch = i, so
 * queue index i is served by CCIF channel i and its ring control block is
 * already mapped by this driver.  Reading those indices is therefore
 * behaviour-neutral: no ring state is written, no register is written, nothing
 * is ACKed, and no timing loop is added on any modem-visible path.
 *
 * Field semantics per hif/ccci_ringbuf.h: each ring starts with
 * rx_control{read,write,length} then tx_control{read,write,length}.
 * For the AP->MD direction the relevant pair is tx_control: write = AP
 * producer, read = MD consumer.
 *
 * The comparison set is q1 and q4 (known-working low channels) against q15
 * (H2D_SRAM / reserved channel 15); q0 is included for completeness.  The
 * modem-side argument (assert para0 = 1) is NOT assumed to be a queue index --
 * sampling all four decides that from evidence.
 */
static int ccif_obs_left;
static struct delayed_work ccif_obs_dw;

static void ccif_obs_dump(struct md_ccif_ctrl *md_ctrl, const char *tag)
{
	static const int idxs[] = { 0, 1, 4, 15 };
	struct ccci_smem_region *mcu, *exp;
	int k;

	mcu = ccci_md_get_smem_by_user_id(md_ctrl->md_id, SMEM_USER_CCISM_MCU);
	exp = ccci_md_get_smem_by_user_id(md_ctrl->md_id, SMEM_USER_CCISM_MCU_EXP);

	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"OBS %s smem mcu=%08x/%u exp=%08x/%u | AP B=%08x S=%08x R=%08x A=%08x | MD B=%08x S=%08x R=%08x A=%08x\n",
		tag,
		mcu ? (unsigned int)mcu->base_ap_view_phy : 0,
		mcu ? (unsigned int)mcu->size : 0,
		exp ? (unsigned int)exp->base_ap_view_phy : 0,
		exp ? (unsigned int)exp->size : 0,
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_ACK),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_ACK));

	for (k = 0; k < ARRAY_SIZE(idxs); k++) {
		int i = idxs[k];
		struct ccci_ringbuf *rn = md_ctrl->txq[i].ringbuf_bak[RB_NORMAL];
		struct ccci_ringbuf *re = md_ctrl->txq[i].ringbuf_bak[RB_EXP];

		CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
			"OBS %s q%-2d N[txr=%u txw=%u txl=%u rxr=%u rxw=%u] E[txr=%u txw=%u txl=%u rxr=%u rxw=%u]\n",
			tag, i,
			rn ? rn->tx_control.read : 0xdeadbeef,
			rn ? rn->tx_control.write : 0xdeadbeef,
			rn ? rn->tx_control.length : 0xdeadbeef,
			rn ? rn->rx_control.read : 0xdeadbeef,
			rn ? rn->rx_control.write : 0xdeadbeef,
			re ? re->tx_control.read : 0xdeadbeef,
			re ? re->tx_control.write : 0xdeadbeef,
			re ? re->tx_control.length : 0xdeadbeef,
			re ? re->rx_control.read : 0xdeadbeef,
			re ? re->rx_control.write : 0xdeadbeef);
	}
}

static void ccif_obs_work(struct work_struct *work)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(CCIF_HIF_ID);

	if (!md_ctrl)
		return;
	ccif_obs_dump(md_ctrl, "WAIT");
	if (--ccif_obs_left > 0)
		schedule_delayed_work(&ccif_obs_dw, msecs_to_jiffies(2000));
}

static int md_ccif_send_data(unsigned char hif_id, int channel_id)
{
	switch (channel_id) {
	case H2D_EXCEPTION_CLEARQ_ACK:
		md_ccif_switch_ringbuf(CCIF_HIF_ID, RB_EXP);
		md_ccif_reset_queue(CCIF_HIF_ID, 0);
		break;
	case H2D_SRAM:
		/*
		 * v438 DIAGNOSTIC: the runtime-data notification is the exact
		 * moment the modem enters ccismc_polling_submit_one_gpd, so
		 * start the read-only observer here (immediate sample + one
		 * every 2 s, covering the whole ~43.5 s wait).
		 */
		{
			struct md_ccif_ctrl *c =
				(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

			if (c) {
				ccif_obs_dump(c, "SRAM-SEND");
				ccif_obs_left = 30;
				schedule_delayed_work(&ccif_obs_dw,
					msecs_to_jiffies(2000));
			}
		}
		break;
	default:
		break;
	}
	return md_ccif_send(hif_id, channel_id);
}

void ccci_ccif_send_notify(unsigned char user_id)
{
	if (user_id < (CCIF_CH_NUM - AP_MD_DATA_NOTIFY))
		md_ccif_send(CCIF_HIF_ID, (user_id + AP_MD_DATA_NOTIFY));
}
EXPORT_SYMBOL(ccci_ccif_send_notify);


void md_ccif_sram_reset(unsigned char hif_id)
{
	int idx = 0;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"%s\n", __func__);
	for (idx = 0; idx < md_ctrl->sram_size / sizeof(u32);
		 idx += 1)
		ccif_write32(md_ctrl->ccif_ap_base + APCCIF_CHDATA,
			idx * sizeof(u32), 0);
	ccci_reset_seq_num(&md_ctrl->traffic_info);

}

void md_ccif_reset_queue(unsigned char hif_id, unsigned char for_start)
{
	int i;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	unsigned long flags;

	if (for_start) {
		mod_timer(&md_ctrl->traffic_monitor,
			jiffies + CCIF_TRAFFIC_MONITOR_INTERVAL * HZ);
	} else {
		timer_delete(&md_ctrl->traffic_monitor);
		/*
		 *ccci_reset_ccif_hw(md_ctrl->md_id,
		 *	ccif_id, md_ctrl->ccif_ap_base,
		 *	md_ctrl->ccif_md_base, md_ctrl);
		 */
	}

	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG, "%s\n", __func__);
	for (i = 0; i < QUEUE_NUM; ++i) {
		flush_work(&md_ctrl->rxq[i].qwork);
		spin_lock_irqsave(&md_ctrl->rxq[i].rx_lock, flags);
		ccci_ringbuf_reset(md_ctrl->md_id,
			md_ctrl->rxq[i].ringbuf, 0);
		spin_unlock_irqrestore(&md_ctrl->rxq[i].rx_lock, flags);
		md_ctrl->rxq[i].resume_cnt = 0;

		spin_lock_irqsave(&md_ctrl->txq[i].tx_lock, flags);
		ccci_ringbuf_reset(md_ctrl->md_id,
			md_ctrl->txq[i].ringbuf, 1);
		spin_unlock_irqrestore(&md_ctrl->txq[i].tx_lock, flags);

		ccif_wake_up_tx_queue(md_ctrl, i);
		md_ctrl->txq[i].wakeup = 0;
	}
	ccif_reset_busy_queue(md_ctrl);
	ccci_reset_seq_num(&md_ctrl->traffic_info);
	memset(md_ctrl->isr_cnt, 0, sizeof(md_ctrl->isr_cnt));
}

void md_ccif_switch_ringbuf(unsigned char hif_id, enum ringbuf_id rb_id)
{
	int i;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	unsigned long flags;

	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG, "%s\n", __func__);
	for (i = 0; i < QUEUE_NUM; ++i) {
		spin_lock_irqsave(&md_ctrl->rxq[i].rx_lock, flags);
		md_ctrl->rxq[i].ringbuf = md_ctrl->rxq[i].ringbuf_bak[rb_id];
		spin_unlock_irqrestore(&md_ctrl->rxq[i].rx_lock, flags);

		spin_lock_irqsave(&md_ctrl->txq[i].tx_lock, flags);
		md_ctrl->txq[i].ringbuf = md_ctrl->txq[i].ringbuf_bak[rb_id];
		spin_unlock_irqrestore(&md_ctrl->txq[i].tx_lock, flags);
	}
}

static void md_ccif_check_ringbuf(struct md_ccif_ctrl *md_ctrl, int qno)
{
#ifdef RUN_WQ_BY_CHECKING_RINGBUF
	unsigned long flags;
	int data_to_read;

	if (atomic_read(&md_ctrl->rxq[qno].rx_on_going)) {
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG, "Q%d rx is on-going(%d)3\n",
			     md_ctrl->rxq[qno].index,
			     atomic_read(&md_ctrl->rxq[qno].rx_on_going));
		return;
	}
	spin_lock_irqsave(&md_ctrl->rxq[qno].rx_lock, flags);
	data_to_read =
		ccci_ringbuf_readable(md_ctrl->md_id,
			md_ctrl->rxq[qno].ringbuf);
	spin_unlock_irqrestore(&md_ctrl->rxq[qno].rx_lock, flags);
	if (unlikely(data_to_read > 0)
		&& ccci_md_napi_check_and_notice(md, qno) == 0
		&& ccci_fsm_get_md_state(md_ctrl->md_id) != EXCEPTION) {
		CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
			"%d data remain in q%d\n", data_to_read, qno);
		queue_work(md_ctrl->rxq[qno].worker,
			&md_ctrl->rxq[qno].qwork);
	}
#endif
}

/*exception and SRAM channel handler*/
static void md_ccif_handle_exception(struct md_ccif_ctrl *md_ctrl)
{
	CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
		"ccif_irq_tasklet1: ch %lx\n", md_ctrl->channel_id);
	if (md_ctrl->channel_id & CCIF_HW_CH_RX_RESERVED) {
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"Interrupt from reserved ccif ch(%ld)\n",
			md_ctrl->channel_id);
		md_ctrl->channel_id &= ~CCIF_HW_CH_RX_RESERVED;
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"After cleared reserved ccif ch(%ld)\n",
			md_ctrl->channel_id);
	}
	if (md_ctrl->channel_id & (1 << D2H_EXCEPTION_INIT)) {
		clear_bit(D2H_EXCEPTION_INIT, &md_ctrl->channel_id);
		md_fsm_exp_info(md_ctrl->md_id, (1 << D2H_EXCEPTION_INIT));
	}

	if (md_ctrl->channel_id & (1 << AP_MD_SEQ_ERROR)) {
		clear_bit(AP_MD_SEQ_ERROR, &md_ctrl->channel_id);
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"MD check seq fail\n");
		ccci_md_dump_info(md_ctrl->md_id,
			DUMP_FLAG_CCIF, NULL, 0);
	}
	if (md_ctrl->channel_id & (1 << (D2H_SRAM))) {
		clear_bit(D2H_SRAM, &md_ctrl->channel_id);
		schedule_work(&md_ctrl->ccif_sram_work);
	}
	CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
		"ccif_irq_tasklet2: ch %ld\n", md_ctrl->channel_id);
}

static void md_ccif_launch_work(struct md_ccif_ctrl *md_ctrl)
{
	int i;

	if (md_ctrl->channel_id & (1 << (D2H_SRAM))) {
		clear_bit(D2H_SRAM, &md_ctrl->channel_id);
		schedule_work(&md_ctrl->ccif_sram_work);
	}

	if (md_ctrl->channel_id & (1 << AP_MD_CCB_WAKEUP)) {
		clear_bit(AP_MD_CCB_WAKEUP, &md_ctrl->channel_id);

#ifdef DEBUG_FOR_CCB
		/* CCB count here for channel_id of ccb is clear in this if */
		md_ctrl->traffic_info.latest_q_rx_isr_time[AP_MD_CCB_WAKEUP]
				= local_clock();
		md_ctrl->traffic_info.latest_ccb_isr_time
			= local_clock();
#endif
		ccci_port_queue_status_notify(md_ctrl->md_id, CCIF_HIF_ID,
			AP_MD_CCB_WAKEUP, -1, RX_IRQ);
	}
	for (i = 0; i < QUEUE_NUM; i++) {
		if (md_ctrl->channel_id & (1 << (i + D2H_RINGQ0))) {
			md_ctrl->traffic_info.latest_q_rx_isr_time[i]
				= local_clock();
			clear_bit(i + D2H_RINGQ0, &md_ctrl->channel_id);
			if (atomic_read(&md_ctrl->rxq[i].rx_on_going)) {
				CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"Q%d rx is on-going(%d)2\n",
				md_ctrl->rxq[i].index,
				atomic_read(&md_ctrl->rxq[i].rx_on_going));
				continue;
			}
			queue_work(md_ctrl->rxq[i].worker,
				&md_ctrl->rxq[i].qwork);
		} else
			md_ccif_check_ringbuf(md_ctrl, i);
	}
}

static void md_ccif_process_data0(struct md_ccif_ctrl *md_ctrl,
	unsigned int ch_id)
{
	unsigned int i;

	for (i = 0; i < CCIF_CH_NUM; i++)
		if (ch_id & 0x1 << i) {
			set_bit(i, &md_ctrl->channel_id);
			md_ctrl->isr_cnt[i]++;
		}
	/* for 91/92, HIF CCIF is for C2K, only 16 CH;
	 * for 93, only lower 16 CH is for data
	 */
	ccif_write32(md_ctrl->ccif_ap_base,
		APCCIF_ACK, ch_id & 0xFFFF);
	ccif_diag_win(md_ctrl, "WIN-ISR", (int)ch_id);

	/* igore exception queue */
	if (ch_id >> RINGQ_BASE) {
		md_ctrl->traffic_info.isr_cnt++;
		md_ctrl->traffic_info.latest_isr_time
			= local_clock();
#ifdef DEBUG_FOR_CCB
	/* infactly, maybe md_ctrl->channel_id is, which maybe cleared */
		md_ctrl->traffic_info.last_ccif_r_ch = ch_id;
#endif
		md_ccif_launch_work(md_ctrl);
	} else
		md_ccif_handle_exception(md_ctrl);
}

static void md_ccif_data0_poll(struct work_struct *work)
{
	struct md_ccif_ctrl *md_ctrl = container_of(to_delayed_work(work),
		struct md_ccif_ctrl, data0_poll_work);
	unsigned int ch_id;

	if (READ_ONCE(md_ctrl->ccif_state) != HIFCCIF_STATE_PWRON)
		return;

	ch_id = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM);
	if (!ch_id) {
		md_ctrl->data0_empty_polls++;
		if (md_ctrl->data0_empty_polls == 1 ||
		    !(md_ctrl->data0_empty_polls % 50))
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
				"WORKAROUND: DATA0 empty IRQ masked, poll=%u\n",
				md_ctrl->data0_empty_polls);
		mod_delayed_work(system_wq, &md_ctrl->data0_poll_work,
			msecs_to_jiffies(CCIF_EMPTY_IRQ_POLL_MS));
		return;
	}

	md_ccif_process_data0(md_ctrl, ch_id);
	CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
		"WORKAROUND: DATA0 polling recovered ch=0x%x after %u polls\n",
		ch_id, md_ctrl->data0_empty_polls);
	if (READ_ONCE(md_ctrl->ccif_state) == HIFCCIF_STATE_PWRON &&
	    atomic_cmpxchg(&md_ctrl->data0_irq_masked, 1, 0) == 1)
		enable_irq(md_ctrl->ap_ccif_irq0_id);
}

static irqreturn_t md_ccif_isr(int irq, void *data)
{
	struct md_ccif_ctrl *md_ctrl = (struct md_ccif_ctrl *)data;
	unsigned int ch_id;
	int observe_count = atomic_inc_return(&ccif_data0_observe_count);

	if (observe_count <= 4)
		pr_info("CCCI-OBS: CCIF_DATA0 entry n=%d irq=%d cpu=%u\n",
			observe_count, irq, raw_smp_processor_id());
	/* Must ack first, otherwise IRQ will rush in. */
	ch_id = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM);
	if (!ch_id) {
		if (atomic_cmpxchg(&md_ctrl->data0_irq_masked, 0, 1) == 0) {
			md_ctrl->data0_empty_polls = 0;
			disable_irq_nosync(irq);
			mod_delayed_work(system_wq, &md_ctrl->data0_poll_work,
				msecs_to_jiffies(CCIF_EMPTY_IRQ_POLL_MS));
			CCCI_NOTICE_LOG(md_ctrl->md_id, TAG,
				"WORKAROUND: mask DATA0 IRQ with empty RCHNUM\n");
		}
		goto out;
	}

	md_ccif_process_data0(md_ctrl, ch_id);
out:
	if (observe_count <= 4)
		pr_info("CCCI-OBS: CCIF_DATA0 exit n=%d irq=%d cpu=%u ch=0x%x\n",
			observe_count, irq, raw_smp_processor_id(), ch_id);

	return IRQ_HANDLED;
}

static inline void md_ccif_queue_struct_init(struct md_ccif_queue *queue,
	unsigned char hif_id, enum DIRECTION dir, unsigned char index)
{
	queue->dir = dir;
	queue->index = index;
	queue->hif_id = hif_id;
	init_waitqueue_head(&queue->req_wq);
	spin_lock_init(&queue->rx_lock);
	spin_lock_init(&queue->tx_lock);
	atomic_set(&queue->rx_on_going, 0);
	queue->debug_id = 0;
	queue->wakeup = 0;
	queue->resume_cnt = 0;
	queue->budget = RX_BUGDET;
}

static int md_ccif_op_write_room(unsigned char hif_id, unsigned char qno)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (qno >= QUEUE_NUM)
		return -CCCI_ERR_INVALID_QUEUE_INDEX;
	return ccci_ringbuf_writeable(md_ctrl->md_id,
				md_ctrl->txq[qno].ringbuf, 0);
}

static int md_ccif_op_send_skb(unsigned char hif_id, int qno,
	struct sk_buff *skb, int skb_from_pool, int blocking)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	struct md_ccif_queue *queue = NULL;
	/* struct ccci_header *ccci_h =
	 * (struct ccci_header *)req->skb->data;
	 */
	int ret;
	/* struct ccci_header *ccci_h; */
	unsigned long flags;
	int ccci_to_c2k_ch = 0;
	int md_flow_ctrl = 0;
	struct ccci_header *ccci_h = NULL;
	int md_cap = ccci_md_get_cap_by_id(md_ctrl->md_id);
	struct ccci_per_md *per_md_data =
		ccci_get_per_md_data(md_ctrl->md_id);
	int md_state;
	/* DIAGNOSTIC 20260922 (v426/v427): packet identity captured before the
	 * skb is freed, so the doorbell outcome can be correlated with the FS
	 * request/reply it belongs to.  Read-only. */
	unsigned int diag_len = 0, diag_d0 = 0;
	u16 diag_lch = 0, diag_seq = 0;
	unsigned int diag_ab = 0;
	int diag_ret = 0;
	unsigned int diag_busy_b = 0, diag_busy_a = 0;

	if (qno == 0xFF)
		return -CCCI_ERR_INVALID_QUEUE_INDEX;

	queue = &md_ctrl->txq[qno];

	ccci_h = (struct ccci_header *)skb->data;

	if (ccci_h->channel == CCCI_C2K_LB_DL)
		qno = atomic_read(&lb_dl_q);
	if (md_ctrl->plat_val.md_gen < 6295) {
		if (qno > 7) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"qno error (%d)\n", qno);
			return -CCCI_ERR_INVALID_QUEUE_INDEX;
		}
	}
	queue = &md_ctrl->txq[qno];
 retry:
	/* we use irqsave as network require a lock in softirq,
	 * cause a potential deadlock
	 */
	spin_lock_irqsave(&queue->tx_lock, flags);

	if (ccci_ringbuf_writeable(md_ctrl->md_id,
			queue->ringbuf, skb->len) > 0) {
		if (ccci_h->channel == CCCI_C2K_LB_DL) {
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"Q%d Tx lb_dl\n", queue->index);
			c2k_mem_dump(skb->data, skb->len);
		}
		ccci_md_inc_tx_seq_num(md_ctrl->md_id,
			&md_ctrl->traffic_info, ccci_h);

		ccci_channel_update_packet_counter(
			md_ctrl->traffic_info.logic_ch_pkt_cnt,
			ccci_h);

		if (md_ctrl->md_id == MD_SYS3) {
			/* heart beat msg is sent from status channel in ECCCI,
			 * but from control channel in C2K,
			 * no status channel in C2K
			 */
			if (ccci_h->channel == CCCI_STATUS_TX) {
				ccci_h->channel = CCCI_CONTROL_TX;
				ccci_h->data[1] = C2K_HB_MSG;
				ccci_h->reserved = md_ctrl->heart_beat_counter;
				md_ctrl->heart_beat_counter++;
				ccci_md_inc_tx_seq_num(md_ctrl->md_id,
					&md_ctrl->traffic_info, ccci_h);
			}

			/* md3(c2k) logical channel number is not
			 * the same as other modems,
			 * so we need to use mapping table to
			 * convert channel id here.
			 */
			ccci_to_c2k_ch =
			ccci_ch_to_c2k_ch(ccci_fsm_get_md_state(md_ctrl->md_id),
				ccci_h->channel, OUT);
			if (ccci_to_c2k_ch >= 0
				&& ccci_to_c2k_ch < C2K_OVER_MAX_CH)
				ccci_h->channel = (u16) ccci_to_c2k_ch;
			else {
				ret = -CCCI_ERR_INVALID_LOGIC_CHANNEL_ID;
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"channel num error (%d)\n",
					ccci_to_c2k_ch);
				spin_unlock_irqrestore(&queue->tx_lock, flags);
				return ret;
			}
			if (ccci_h->data[1] == C2K_HB_MSG)
				CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
					"hb: 0x%x\n", ccci_h->channel);
		}
		/* copy skb to ringbuf */
		diag_len = skb->len;
		diag_lch = ccci_h->channel;
		diag_seq = ccci_h->seq_num;
		diag_ab = ccci_h->assert_bit;
		diag_d0 = ccci_h->data[0];
		ret = ccci_ringbuf_write(md_ctrl->md_id,
				queue->ringbuf, skb->data, skb->len);
		if (ret != skb->len)
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"TX:ERR rbf write: ret(%d)!=req(%d)\n",
				ret, skb->len);
		ccci_md_add_log_history(&md_ctrl->traffic_info, OUT,
			(int)queue->index, ccci_h, 0);
		/* free request */
		ccci_free_skb(skb);

		/* B15R (log-only): TXQ1 read/write snapshots for the 36 B reply
		 * frame. Deterministic (fires only for our frame size, no
		 * polling). CCCI_ERROR_LOG is always visible (pr_notice).
		 * Read-only; no behavior change. */
		if (ret == 36 && queue->index == 1) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"B15R txq1 pub: read=%u write=%u len=%u slot_off=%u\n",
				(unsigned int)queue->ringbuf->tx_control.read,
				(unsigned int)queue->ringbuf->tx_control.write,
				(unsigned int)queue->ringbuf->tx_control.length,
				(unsigned int)queue->ringbuf->tx_control.write >= 56 ?
				(unsigned int)queue->ringbuf->tx_control.write - 56 :
				(unsigned int)(queue->ringbuf->tx_control.length +
					queue->ringbuf->tx_control.write - 56));
		}

		/* send ccif request */
		diag_busy_b = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY);
		diag_ret = md_ccif_send(hif_id, queue->ccif_ch);
		diag_busy_a = ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY);
		if (ccif_diag_enable)
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"CCIF-DIAG DOORBELL q=%d ch=%d len=%u lch=%u seq=%u ab=%u d0=0x%x rbf=%d ret=%d busy_b=0x%x busy_a=0x%x refused=%d\n",
				queue->index, queue->ccif_ch, diag_len,
				(unsigned int)diag_lch, (unsigned int)diag_seq,
				diag_ab, diag_d0, ret, diag_ret,
				diag_busy_b, diag_busy_a,
				!!(diag_busy_b & (1 << queue->ccif_ch)));
		/* B15R (log-only): post-doorbell snapshot, same frame class. */
		if (ret == 36 && queue->index == 1) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"B15R txq1 doorbell: ch=%d read=%u write=%u\n",
				queue->index, queue->ccif_ch,
				(unsigned int)queue->ringbuf->tx_control.read,
				(unsigned int)queue->ringbuf->tx_control.write);
		}
		spin_unlock_irqrestore(&queue->tx_lock, flags);
	} else {
		md_flow_ctrl = ccif_is_md_flow_ctrl_supported(md_ctrl);
		if (likely(md_cap & MODEM_CAP_TXBUSY_STOP)
			&& md_flow_ctrl > 0) {
			ccif_set_busy_queue(md_ctrl, qno);
			/* double check tx buffer after set busy bit.
			 * it is to avoid tx buffer is empty now
			 */
			if (unlikely(ccci_ringbuf_writeable(md_ctrl->md_id,
					queue->ringbuf, skb->len) > 0)) {
				ccif_clear_busy_queue(md_ctrl, qno);
				spin_unlock_irqrestore(&queue->tx_lock, flags);
				goto retry;
			} else {
				CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
					"flow ctrl: TX busy on Q%d\n",
					queue->index);
				ccci_port_queue_status_notify(md_ctrl->md_id,
					hif_id, queue->index, OUT, TX_FULL);
			}
		} else
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"flow ctrl is invalid, cap = %d, md_flow_ctrl = %d\n",
				md_cap, md_flow_ctrl);

		spin_unlock_irqrestore(&queue->tx_lock, flags);

		if (blocking) {
			if (md_flow_ctrl > 0) {
				CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
					"flow ctrl: Q%d is blocking, skb->len = %d\n",
					queue->index, skb->len);
				ret = wait_event_interruptible_exclusive(
					queue->req_wq,
					(queue->wakeup != 0));
				queue->wakeup = 0;
				if (ret == -ERESTARTSYS)
					return -EINTR;
			}
			md_state = ccci_fsm_get_md_state(md_ctrl->md_id);
			if (md_state == EXCEPTION
					&& ccci_h->channel != CCCI_MD_LOG_TX
					&& ccci_h->channel != CCCI_UART1_TX
					&& ccci_h->channel != CCCI_FS_TX) {
				CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
					"tx retry break for EE for Q%d, ch %d\n",
					queue->index, ccci_h->channel);
				return -ETXTBSY;
			} else if (md_state == GATED) {
				CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
					"tx retry break for Gated for Q%d, ch %d\n",
					queue->index, ccci_h->channel);
				return -ETXTBSY;
			}
			CCCI_REPEAT_LOG(md_ctrl->md_id, TAG,
				"tx retry for Q%d, ch %d\n",
				queue->index, ccci_h->channel);
				goto retry;
		} else {
			if (per_md_data->data_usb_bypass)
				return -ENOMEM;
			else
				return -EBUSY;
			CCCI_DEBUG_LOG(md_ctrl->md_id, TAG,
				"tx fail on q%d\n", qno);
		}
	}
	return 0;
}

static int md_ccif_op_give_more(unsigned char hif_id, unsigned char qno)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (!md_ctrl)
		return -CCCI_ERR_HIF_NOT_POWER_ON;

	if (qno == 0xFF)
		return -CCCI_ERR_INVALID_QUEUE_INDEX;
	queue_work(md_ctrl->rxq[qno].worker,
		&md_ctrl->rxq[qno].qwork);
	return 0;
}

static int md_ccif_stop_queue(unsigned char hif_id,
	unsigned char qno, enum DIRECTION dir)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	if (dir == OUT)
		ccif_set_busy_queue(md_ctrl, qno);

	return 0;
}

static int md_ccif_start_queue(unsigned char hif_id,
	unsigned char qno, enum DIRECTION dir)
{
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	struct md_ccif_queue *queue = NULL;
	unsigned long flags;

	if (dir == OUT
		&& likely(ccci_md_get_cap_by_id(md_ctrl->md_id)
		& MODEM_CAP_TXBUSY_STOP
		&& (qno < QUEUE_NUM))) {
		queue = &md_ctrl->txq[qno];
		spin_lock_irqsave(&queue->tx_lock, flags);
		ccif_wake_up_tx_queue(md_ctrl, qno);
		/*special for net queue*/
		ccci_hif_queue_status_notify(md_ctrl->md_id,
			hif_id, qno, OUT, TX_IRQ);
		spin_unlock_irqrestore(&queue->tx_lock, flags);
	}
	return 0;
}

int md_ccif_exp_ring_buf_init(struct md_ccif_ctrl *md_ctrl)
{
	int i = 0;
	unsigned char *buf;
	int bufsize = 0;
	struct ccci_ringbuf *ringbuf;
	struct ccci_smem_region *ccism;

	ccism = ccci_md_get_smem_by_user_id(md_ctrl->md_id,
		SMEM_USER_CCISM_MCU_EXP);
	if (ccism->size)
		memset_io(ccism->base_ap_view_vir, 0, ccism->size);

	buf = (unsigned char *)ccism->base_ap_view_vir;

	for (i = 0; i < QUEUE_NUM; i++) {

		if (md_ctrl->plat_val.md_gen >= 6295) {
			bufsize = CCCI_RINGBUF_CTL_LEN +
			rx_exp_buffer_size_up_95[i]
			+ tx_exp_buffer_size_up_95[i];
			ringbuf =
		    ccci_create_ringbuf(md_ctrl->md_id, buf, bufsize,
				rx_exp_buffer_size_up_95[i],
				tx_exp_buffer_size_up_95[i]);
			if (ringbuf == NULL) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"ccci_create_ringbuf %d failed\n", i);
				return -1;
			}

		} else {
			bufsize = CCCI_RINGBUF_CTL_LEN + rx_exp_buffer_size[i]
				+ tx_exp_buffer_size[i];
			ringbuf =
			    ccci_create_ringbuf(md_ctrl->md_id, buf, bufsize,
					rx_exp_buffer_size[i],
					tx_exp_buffer_size[i]);
			if (ringbuf == NULL) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"ccci_create_ringbuf %d failed\n", i);
				return -1;
			}
		}
		/*rx */
		md_ctrl->rxq[i].ringbuf_bak[RB_EXP] = ringbuf;
		md_ctrl->rxq[i].ccif_ch = D2H_RINGQ0 + i;
		/*tx */
		md_ctrl->txq[i].ringbuf_bak[RB_EXP] = ringbuf;
		md_ctrl->txq[i].ccif_ch = H2D_RINGQ0 + i;
		buf += bufsize;
	}

	return 0;
}

int md_ccif_ring_buf_init(unsigned char hif_id)
{
	int i = 0;
	unsigned char *buf;
	int bufsize = 0;
	struct md_ccif_ctrl *md_ctrl;
	struct ccci_ringbuf *ringbuf;
	struct ccci_smem_region *ccism;

	md_ctrl = (struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	ccism = ccci_md_get_smem_by_user_id(md_ctrl->md_id,
		SMEM_USER_CCISM_MCU);
	if (ccism->size)
		memset_io(ccism->base_ap_view_vir, 0, ccism->size);
	md_ctrl->total_smem_size = 0;
	/*CCIF_MD_SMEM_RESERVE; */
	buf = (unsigned char *)ccism->base_ap_view_vir;

	for (i = 0; i < QUEUE_NUM; i++) {
		if (md_ctrl->plat_val.md_gen >= 6298) {
			bufsize = CCCI_RINGBUF_CTL_LEN
			+ rx_queue_buffer_size_up_98[i]
			+ tx_queue_buffer_size_up_98[i];

			if (md_ctrl->total_smem_size + bufsize > ccism->size) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"share memory too small,please check configure,smem_size=%d\n",
					ccism->size);
				return -1;
			}
			ringbuf =
			    ccci_create_ringbuf(md_ctrl->md_id, buf, bufsize,
					rx_queue_buffer_size_up_98[i],
					tx_queue_buffer_size_up_98[i]);
		} else if (md_ctrl->plat_val.md_gen >= 6295) {
			bufsize = CCCI_RINGBUF_CTL_LEN
			+ rx_queue_buffer_size_up_95[i]
			+ tx_queue_buffer_size_up_95[i];

			if (md_ctrl->total_smem_size + bufsize > ccism->size) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"share memory too small,please check configure,smem_size=%d\n",
					ccism->size);
				return -1;
			}
			ringbuf =
			    ccci_create_ringbuf(md_ctrl->md_id, buf, bufsize,
					rx_queue_buffer_size_up_95[i],
					tx_queue_buffer_size_up_95[i]);
		} else {
			bufsize = CCCI_RINGBUF_CTL_LEN + rx_queue_buffer_size[i]
				+ tx_queue_buffer_size[i];
			if (md_ctrl->total_smem_size + bufsize > ccism->size) {
				CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
					"share memory too small,please check configure,smem_size=%d\n",
					ccism->size);
				return -1;
			}
			ringbuf =
			    ccci_create_ringbuf(md_ctrl->md_id, buf, bufsize,
					rx_queue_buffer_size[i],
					tx_queue_buffer_size[i]);
		}

		if (ringbuf == NULL) {
			CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
				"ccci_create_ringbuf %d failed\n", i);
			return -1;
		}
		/*rx */
		md_ctrl->rxq[i].ringbuf_bak[RB_NORMAL] = ringbuf;
		md_ctrl->rxq[i].ringbuf = ringbuf;
		md_ctrl->rxq[i].ccif_ch = D2H_RINGQ0 + i;
		if (i != C2K_MD_LOG_RX_Q)
			md_ctrl->rxq[i].worker =
				alloc_workqueue("rx%d_worker",
					WQ_UNBOUND | WQ_MEM_RECLAIM
					| WQ_HIGHPRI,
					1, i);
		else
			md_ctrl->rxq[i].worker =
				alloc_workqueue("rx%d_worker",
					WQ_UNBOUND | WQ_MEM_RECLAIM, 1, i);
		INIT_WORK(&md_ctrl->rxq[i].qwork, ccif_rx_work);
		/*tx */
		md_ctrl->txq[i].ringbuf_bak[RB_NORMAL] = ringbuf;
		md_ctrl->txq[i].ringbuf = ringbuf;
		md_ctrl->txq[i].ccif_ch = H2D_RINGQ0 + i;
		buf += bufsize;
		md_ctrl->total_smem_size += bufsize;
	}

	md_ccif_exp_ring_buf_init(md_ctrl);

	/* v438 DIAGNOSTIC: timeline point 1 -- state right after ring init. */
	ccif_obs_dump(md_ctrl, "RING-INIT");

	/*flow control zone is behind ring buffer zone*/
#ifdef FLOW_CTRL_ENABLE
	if (ccci_md_get_cap_by_id(md_ctrl->md_id) & MODEM_CAP_TXBUSY_STOP) {
		md_ctrl->flow_ctrl =
		(struct ccif_flow_control *)(ccism->base_ap_view_vir
		+ md_ctrl->total_smem_size);
		md_ctrl->total_smem_size += sizeof(struct ccif_flow_control);
	} else {
		md_ctrl->flow_ctrl = NULL;
		CCCI_INIT_LOG(md_ctrl->md_id, TAG, "No flow control for AP\n");
	}
#else
	md_ctrl->flow_ctrl = NULL;
	CCCI_INIT_LOG(md_ctrl->md_id, TAG, "flow control is disabled\n");
#endif
	ccism->size = md_ctrl->total_smem_size;
	return 0;
}

#define PCCIF_BUSY (0x4)
#define PCCIF_TCHNUM (0xC)
#define PCCIF_ACK (0x14)
#define PCCIF_CHDATA (0x100)
#define PCCIF_SRAM_SIZE (512)
void ccci_reset_ccif_hw(unsigned char md_id,
	int ccif_id, void __iomem *baseA,
	void __iomem *baseB, struct md_ccif_ctrl *md_ctrl)
{
	int i;
	struct ccci_smem_region *region;
	struct ccci_smem_region *region_resv;
	int reset_bit = -1;
	unsigned int tail_size;

	CCCI_NORMAL_LOG(md_id, TAG, "%s, ccif_hw_reset_ver = %d\n",
			__func__, md_ctrl->ccif_hw_reset_ver);

	if (md_ctrl->ccif_hw_reset_ver == 1) {
		reset_bit = 26;

		/* set ccif0 reset bit */
		ccci_write32(md_ctrl->infracfg_base, 0xF50, 1 << reset_bit);

		/* set ccif0 reset bit */
		ccci_write32(md_ctrl->infracfg_base, 0xF54, 1 << reset_bit);
	} else {
		switch (ccif_id) {
		case AP_MD1_CCIF:
			reset_bit = 8;
			break;
		}

		if (reset_bit == -1)
			return;

	/*
	 *this reset bit will clear
	 *CCIF's busy/wch/irq, but not SRAM
	 */
	/*set reset bit*/
		regmap_write(md_ctrl->plat_val.infra_ao_base,
			0x150, 1 << reset_bit);
		/*clear reset bit*/
		regmap_write(md_ctrl->plat_val.infra_ao_base,
			0x154, 1 << reset_bit);
	}

	/*
	 * The reset pulse leaves the SRAM intact, so this is the only point
	 * where the MD view can still carry LK's pre-written smem-info tail.
	 * Log it before the clear loop wipes both views (§80.41).
	 */
	CCCI_NORMAL_LOG(md_id, TAG,
		"WORKAROUND: tail pre-clear A=%08x/%08x/%08x B=%08x/%08x/%08x flag=%d\n",
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32)),
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 2 * sizeof(u32)),
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 2 * sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - sizeof(u32)),
		devapc_check_flag);

	/* clear SRAM */
	for (i = 0; i < PCCIF_SRAM_SIZE/sizeof(unsigned int); i++) {
		ccif_write32(baseA, PCCIF_CHDATA+i*sizeof(unsigned int), 0);
		ccif_write32(baseB, PCCIF_CHDATA+i*sizeof(unsigned int), 0);
	}

	/* extend from 36bytes to 72bytes in CCIF SRAM */
	/* 0~60bytes for bootup trace,
	 *last 12bytes for magic pattern,smem address and size
	 */
	/* v435 cross-read probe result (RESULT.md 16): the two CHDATA windows are
	 * ONE SHARED CELL (A->B=11223344, B->A=55667788 => views_shared=1), so the
	 * tail is written to baseA alone, exactly as stock does.  The probe has
	 * been removed, and no baseB write is needed or wanted.
	 */
	region = ccci_md_get_smem_by_user_id(md_id,
		SMEM_USER_RAW_MDSS_DBG);
	ccif_write32(baseA,
		PCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32),
		0x7274626E);
	ccif_write32(baseA,
		PCCIF_CHDATA + PCCIF_SRAM_SIZE - 2 * sizeof(u32),
		region->base_md_view_phy);
	/*
	 * v434: the tail's third word is NOT a plain "region size" -- it is the
	 * `size` of the writable MPU region the modem creates for this buffer.
	 * Decoded from authenticated md1rom (see
	 * docs-local/v427-ccif-sram-window-20260923/RESULT.md 12.2):
	 *   Set_HS1_Boot_Trace (0x902e6836) requires word0 == 0x7274626E, takes
	 *     word1 as the SMEM base and writes its trace-end marker 0x56552552
	 *     at *** word1 + 0x3800 *** (the modem hardcodes 14KB itself).
	 *   MPU_Init (0x918a7c9a) -> mpu_auto_make_region_noprot(word1, word2),
	 *     so the modem's writable window is [word1, word1 + word2).
	 *   INT_hasEMMAddress (0x902e67f8) asserts (line 2857) unless
	 *     0x2800 <= word2 <= 0x10000000.
	 * Publishing word2 == region->size (0x3800 == exactly the marker offset)
	 * therefore puts the modem's own marker write one byte outside the region
	 * it just made writable: MPU violation, and the modem never reaches HS1.
	 * Reproduced twice (v433).  It is NOT a too-small region definition: our
	 * ccci_modem.c region table and platform/md_sys1_platform.c are
	 * byte-identical to the authenticated vendor tree, and the modem hardcodes
	 * the 14KB offset itself.
	 * So the required value is > 0x3800.  (b) settles it from stock, not by
	 * guess: the device's own LK "nc_smem_info_ext" override sets
	 * RAW_MDSS_DBG = 0x6000 (see the table in ccci_modem.c), so publishing
	 * region->size now yields word2 == 0x6000 and the modem's hardcoded
	 * word1+0x3800 marker lands inside the MPU window.  The previous
	 * `region->size + sizeof(u32)` hack is therefore removed and the stock
	 * value is published verbatim.
	 */
	region_resv = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_RAW_RESERVED);
	tail_size = region->size;
	CCCI_NORMAL_LOG(md_id, TAG,
		"v436 tail: region->size=%u(0x%x) reserved=%u(0x%x) publish=0x%x\n",
		(unsigned int)region->size, (unsigned int)region->size,
		region_resv ? (unsigned int)region_resv->size : 0,
		region_resv ? (unsigned int)region_resv->size : 0,
		tail_size);
	ccif_write32(baseA,
		PCCIF_CHDATA + PCCIF_SRAM_SIZE - sizeof(u32),
		tail_size);

	/*
	 * Read the tail back from BOTH windows.  Only baseA was written: the v435
	 * cross-read probe proved the two CHDATA windows are one shared cell, so
	 * baseB must read back identically.  This log is the standing proof of
	 * that fact and of the published extent.  Read-only.
	 */
	CCCI_NORMAL_LOG(md_id, TAG,
		"v436 tail wrote, readback A=%08x/%08x/%08x B=%08x/%08x/%08x (region phy=%08x publish=0x%x)\n",
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32)),
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 2 * sizeof(u32)),
		ccif_read32(baseA, PCCIF_CHDATA + PCCIF_SRAM_SIZE - sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 3 * sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - 2 * sizeof(u32)),
		ccif_read32(baseB, PCCIF_CHDATA + PCCIF_SRAM_SIZE - sizeof(u32)),
		(unsigned int)region->base_md_view_phy,
		tail_size);

	/* v443 DIAGNOSTIC: CCIF0 state after Linux's own CCIF setup
	 * (reset pulse + SRAM clear + tail).  Compare against the BOOT dump:
	 * a bit that existed at BOOT and is gone here was destroyed by Linux. */
	ccif_dump_bank(md_ctrl, "POST-RST");

}
EXPORT_SYMBOL(ccci_reset_ccif_hw);

static int ccif_debug(unsigned char hif_id,
		enum ccci_hif_debug_flg flag, int *para)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int ret = -1;

	switch (flag) {
	case CCCI_HIF_DEBUG_SET_WAKEUP:
		CCCI_NORMAL_LOG(-1, TAG,
			"CCIF0 Wake up old path: channel_id == 0x%x\n",
			ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_RCHNUM));
		ccif_ctrl->wakeup_ch =
			ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_RCHNUM);
		ret = 0;
		break;
	case CCCI_HIF_DEBUG_RESET:
		ccci_reset_ccif_hw(ccif_ctrl->md_id, AP_MD1_CCIF,
			ccif_ctrl->ccif_ap_base,
			ccif_ctrl->ccif_md_base, ccif_ctrl);
		ret = 0;
		break;
	default:
		break;
	}
	return ret;
}

static struct ccif_irq_cb_func_info ccif_irq_cb[ID_CCIF_CB_MAX];
int register_ccif_irq_cb(unsigned char user_id, void (*func)(unsigned char user_id))
{
	if (user_id < ID_CCIF_CB_MAX && (user_id < (CCIF_CH_NUM - AP_MD_DATA_NOTIFY))) {
		ccif_irq_cb[user_id].id = user_id;
		ccif_irq_cb[user_id].qno = user_id + AP_MD_DATA_NOTIFY;
		ccif_irq_cb[user_id].cb_func = func;
		return 0;
	}
	return -1;
}
EXPORT_SYMBOL(register_ccif_irq_cb);

/* mask_set == 1: mask, clear bit, ==0: unmask set bit */
int ccif_mask_setting(unsigned char user_id, unsigned char mask_set)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(CCIF_HIF_ID);
	unsigned int reg_val;
	unsigned int reg_offset;
	unsigned long flags;

	if (ccif_ctrl == NULL)
		return -2;

	switch (user_id) {
	case ID_CCIF_USER_DATA:
		spin_lock_irqsave(&ccif_ctrl->mask_lock, flags);
		reg_offset = APCCIF_IRQ1_MASK;
		reg_val = ccif_read32(ccif_ctrl->ccif_ap_base, reg_offset);
		reg_val =
			mask_set?(reg_val&~(1<<AP_MD_DATA_NOTIFY)):(reg_val|(1<<AP_MD_DATA_NOTIFY));
		ccif_write32(ccif_ctrl->ccif_ap_base, reg_offset, reg_val);
		spin_unlock_irqrestore(&ccif_ctrl->mask_lock, flags);
		break;
	default:
		return -1;
	};
	return 0;
}
EXPORT_SYMBOL(ccif_mask_setting);

static irqreturn_t md_cd_ccif_isr(int irq, void *data)
{
	struct md_ccif_ctrl *ccif_ctrl = (struct md_ccif_ctrl *)data;
	int channel_id;
	int observe_count = atomic_inc_return(&ccif_data1_observe_count);

	if (observe_count <= 4)
		pr_info("CCCI-OBS: CCIF_DATA1 entry n=%d irq=%d cpu=%u\n",
			observe_count, irq, raw_smp_processor_id());

	/* must ack first, otherwise IRQ will rush in */
	channel_id = ccif_read32(ccif_ctrl->ccif_ap_base,
		APCCIF_RCHNUM);
	//CCCI_DEBUG_LOG(ccif_ctrl->md_id, TAG,
	//	"MD CCIF IRQ 0x%X\n", channel_id);
	/*don't ack data queue to avoid missing rx intr*/
	ccif_write32(ccif_ctrl->ccif_ap_base, APCCIF_ACK,
		channel_id & (0xFFFF << RINGQ_EXP_BASE));

	if (channel_id & (1 << AP_MD_DATA_NOTIFY) &&
		ccif_irq_cb[ID_CCIF_USER_DATA].cb_func)
		ccif_irq_cb[ID_CCIF_USER_DATA].cb_func(ccif_irq_cb[ID_CCIF_USER_DATA].id);

	md_fsm_exp_info(ccif_ctrl->md_id, channel_id);
	if (observe_count <= 4)
		pr_info("CCCI-OBS: CCIF_DATA1 exit n=%d irq=%d cpu=%u ch=0x%x\n",
			observe_count, irq, raw_smp_processor_id(), channel_id);

	return IRQ_HANDLED;
}

static int ccif_late_init(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int ret = 0;

	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s\n", __func__);

	/* IRQ is enabled after requested, so call enable_irq after
	 * request_irq will get a unbalance warning
	 */
	ret = request_irq(ccif_ctrl->ap_ccif_irq1_id, md_cd_ccif_isr,
			ccif_ctrl->ap_ccif_irq1_flags, "CCIF_AP_DATA1",
			ccif_ctrl);
	if (ret) {
		CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
			"request CCIF_AP_DATA IRQ1(%d) error %d\n",
			ccif_ctrl->ap_ccif_irq1_id, ret);
		return -1;
	}
	ret = irq_set_irq_wake(ccif_ctrl->ap_ccif_irq1_id, 1);
	if (ret)
		CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
			"irq_set_irq_wake ccif ap_ccif_irq1_id(%d) error %d\n",
			ccif_ctrl->ap_ccif_irq1_id, ret);

	md_ccif_ring_buf_init(CCIF_HIF_ID);

	return 0;
}

/*
 * qqcandy: keep the raw infra-ao fallback only for DTs which do not expose
 * the official six CCIF clocks.  The normal qqcandy DT uses the MT6895 CCF
 * provider, matching the official 5.10 path.
 */
#define CCI_RAW_IFRAO1_BITS ((1u << 12) | (1u << 13) | \
				 (1u << 23) | (1u << 26))
#define CCI_RAW_IFRAO3_BITS ((1u << 10) | (1u << 29))

static bool ccif_has_clk_refs(void)
{
	int idx;

	for (idx = 0; idx < ARRAY_SIZE(ccif_clk_table); idx++)
		if (ccif_clk_table[idx].clk_ref)
			return true;

	return false;
}

static int ccif_raw_gates(struct md_ccif_ctrl *ccif_ctrl, bool on)
{
	unsigned int sta1, sta3;
	int ret;

	if (ccif_has_clk_refs())
		return 0;

	/*
	 * MT6895 uses mtk_clk_gate_ops_setclr: enable writes CLR and a zero
	 * status bit means enabled.  The old fallback did the inverse, leaving
	 * all six CCIF gates disabled while reporting them as on.
	 */
	if (on) {
		ret = regmap_write(ccif_ctrl->plat_val.infra_ao_base, 0x8C,
				   CCI_RAW_IFRAO1_BITS);
		if (ret)
			return ret;
		ret = regmap_write(ccif_ctrl->plat_val.infra_ao_base, 0xC4,
				   CCI_RAW_IFRAO3_BITS);
	} else {
		ret = regmap_write(ccif_ctrl->plat_val.infra_ao_base, 0x88,
				   CCI_RAW_IFRAO1_BITS);
		if (ret)
			return ret;
		ret = regmap_write(ccif_ctrl->plat_val.infra_ao_base, 0xC0,
				   CCI_RAW_IFRAO3_BITS);
	}
	if (ret)
		return ret;

	ret = regmap_read(ccif_ctrl->plat_val.infra_ao_base, 0x94, &sta1);
	if (ret)
		return ret;
	ret = regmap_read(ccif_ctrl->plat_val.infra_ao_base, 0xC8, &sta3);
	if (ret)
		return ret;

	if (on) {
		if ((sta1 & CCI_RAW_IFRAO1_BITS) ||
		    (sta3 & CCI_RAW_IFRAO3_BITS)) {
			pr_err("CCI-CCIF: raw gate ON failed: STA1=0x%08x STA3=0x%08x\n",
			       sta1, sta3);
			return -EIO;
		}
	} else if ((sta1 & CCI_RAW_IFRAO1_BITS) != CCI_RAW_IFRAO1_BITS ||
		   (sta3 & CCI_RAW_IFRAO3_BITS) != CCI_RAW_IFRAO3_BITS) {
		pr_err("CCI-CCIF: raw gate OFF failed: STA1=0x%08x STA3=0x%08x\n",
		       sta1, sta3);
		return -EIO;
	}

	pr_info("CCI-CCIF: raw gates %s: STA1=0x%08x STA3=0x%08x\n",
		on ? "ON" : "OFF", sta1, sta3);
	return 0;
}

static int ccif_set_clk_on(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int idx, ret = 0;
	unsigned long flags;

	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s start\n", __func__);

	ret = ccif_raw_gates(ccif_ctrl, true);
	if (ret) {
		CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
			"%s, raw gate enable failed %d\n", __func__, ret);
		return ret;
	}

	for (idx = 0; idx < ARRAY_SIZE(ccif_clk_table); idx++) {
		if (ccif_clk_table[idx].clk_ref == NULL)
			continue;
		ret = clk_prepare_enable(ccif_clk_table[idx].clk_ref);
		if (ret) {
			CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
				"%s,ret=%d\n",
				__func__, ret);
			return ret;
		}
		spin_lock_irqsave(&devapc_flag_lock, flags);
		devapc_check_flag = 1;
		spin_unlock_irqrestore(&devapc_flag_lock, flags);
	}

	/* A legacy DT has no clk_refs, so the vendor loop cannot set this flag. */
	if (!ccif_has_clk_refs()) {
		spin_lock_irqsave(&devapc_flag_lock, flags);
		devapc_check_flag = 1;
		spin_unlock_irqrestore(&devapc_flag_lock, flags);
		pr_info("CCI-CCIF: %s: devapc_check_flag=1 (raw-gate path)\n",
			__func__);
	}

	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s end\n", __func__);
	return 0;
}

/*
 * for ccif4,5 power off action different:
 * gen97: 0x1000330C [31:0] write 0x0
 * gen98: 0x10001BF0 [15:0] write 0xF7FF
 * gen95: 0x10001C10 [31:0] write 0x0
 */
static void ccif_set_clk_off(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int idx, ret;
	unsigned long flags;

	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s start\n", __func__);

	/* The official CCF path clears this in its loop, after the tail writes. */
	if (!ccif_has_clk_refs()) {
		spin_lock_irqsave(&devapc_flag_lock, flags);
		devapc_check_flag = 0;
		spin_unlock_irqrestore(&devapc_flag_lock, flags);
	}

	if ((ccif_ctrl->plat_val.md_gen >= 6298) ||
	    (ccif_ctrl->ccif_hw_reset_ver == 1)) {
		/* write 1 clear register */
		regmap_write(ccif_ctrl->plat_val.infra_ao_base,
			0xBF0, 0xF7FF);
	} else if (ccif_ctrl->plat_val.md_gen <= 6297) {
		/* Clean MD_PCCIF4_SW_READY and MD_PCCIF4_PWR_ON */
		if (ccif_ctrl->plat_val.md_gen == 6297) {
			if (!IS_ERR(ccif_ctrl->pericfg_base)) {
				CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s:pericfg_base:0x%p\n",
					__func__, ccif_ctrl->pericfg_base);
				regmap_write(ccif_ctrl->pericfg_base, 0x30c, 0x0);
			}
		} else {
		/* set gen95 clock */
			CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s:infra_ao_base:0x%p\n",
				__func__, ccif_ctrl->plat_val.infra_ao_base);
			regmap_write(ccif_ctrl->plat_val.infra_ao_base, 0xC10, 0x0);
		}

	}
	for (idx = 0; idx < ARRAY_SIZE(ccif_clk_table); idx++) {
		if (ccif_clk_table[idx].clk_ref == NULL)
			continue;
		if (strcmp(ccif_clk_table[idx].clk_name, "infra-ccif4-md") == 0
			&& ccif_ctrl->md_ccif4_base) {
			udelay(1000);
			CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
				"%s: after 1ms, set md_ccif4_base + 0x14 = 0xFF\n", __func__);
			/* special use ccci_write32 */
			ccci_write32(ccif_ctrl->md_ccif4_base, 0x14, 0xFF);
 		}
		if (strcmp(ccif_clk_table[idx].clk_name, "infra-ccif5-md") == 0
			&& ccif_ctrl->md_ccif5_base) {
			udelay(1000);
			CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
				"%s: after 1ms, set md_ccif5_base + 0x14 = 0xFF\n", __func__);
			/* special use ccci_write32 */
			ccci_write32(ccif_ctrl->md_ccif5_base, 0x14, 0xFF);
		}
		spin_lock_irqsave(&devapc_flag_lock, flags);
		devapc_check_flag = 0;
		spin_unlock_irqrestore(&devapc_flag_lock, flags);
		clk_disable_unprepare(ccif_clk_table[idx].clk_ref);
	}
	ret = ccif_raw_gates(ccif_ctrl, false);
	if (ret)
		CCCI_ERROR_LOG(ccif_ctrl->md_id, TAG,
			"%s, raw gate disable failed %d\n", __func__, ret);

	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s end\n", __func__);
}

static int ccif_start(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	int ret;

	if (ccif_ctrl->ccif_state == HIFCCIF_STATE_PWRON)
		return 0;
	if (ccif_ctrl->ccif_state == HIFCCIF_STATE_MIN)
		ccif_late_init(hif_id);
	if (hif_id != CCIF_HIF_ID)
		CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s but %d\n",
			__func__, hif_id);
	ret = ccif_set_clk_on(hif_id);
	if (ret)
		return ret;
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
		"start stage: SRAM reset begin\n");
	md_ccif_sram_reset(CCIF_HIF_ID);
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
		"start stage: SRAM reset done\n");
	md_ccif_switch_ringbuf(CCIF_HIF_ID, RB_EXP);
	md_ccif_reset_queue(CCIF_HIF_ID, 1);
	md_ccif_switch_ringbuf(CCIF_HIF_ID, RB_NORMAL);
	md_ccif_reset_queue(CCIF_HIF_ID, 1);
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
		"start stage: queue reset done, HW reset begin\n");

	/* clear all ccif irq before enable it.*/
	ccci_reset_ccif_hw(ccif_ctrl->md_id, AP_MD1_CCIF,
		ccif_ctrl->ccif_ap_base,
		ccif_ctrl->ccif_md_base, ccif_ctrl);
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG,
		"start stage: HW reset done\n");
	WRITE_ONCE(ccif_ctrl->ccif_state, HIFCCIF_STATE_PWRON);
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s\n", __func__);
	return 0;
}

static int ccif_stop(unsigned char hif_id)
{
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);
	bool data0_irq_masked;

	if (ccif_ctrl->ccif_state == HIFCCIF_STATE_PWROFF
		|| ccif_ctrl->ccif_state == HIFCCIF_STATE_MIN)
		return 0;
	/* ACK CCIF for MD. while entering flight mode,
	 * we may send something after MD slept
	 */
	WRITE_ONCE(ccif_ctrl->ccif_state, HIFCCIF_STATE_PWROFF);
	cancel_delayed_work_sync(&ccif_ctrl->data0_poll_work);
	ccci_reset_ccif_hw(ccif_ctrl->md_id, AP_MD1_CCIF,
		ccif_ctrl->ccif_ap_base, ccif_ctrl->ccif_md_base, ccif_ctrl);
	synchronize_irq(ccif_ctrl->ap_ccif_irq0_id);
	cancel_delayed_work_sync(&ccif_ctrl->data0_poll_work);
	data0_irq_masked = atomic_xchg(&ccif_ctrl->data0_irq_masked, 0);
	if (data0_irq_masked)
		enable_irq(ccif_ctrl->ap_ccif_irq0_id);
	/*disable ccif clk*/
	ccif_set_clk_off(hif_id);
	CCCI_NORMAL_LOG(ccif_ctrl->md_id, TAG, "%s\n", __func__);
	return 0;
}

/*return ap_rt_data pointer after filling header*/
static void *ccif_hif_fill_rt_header(unsigned char hif_id, int packet_size,
	unsigned int tx_ch, unsigned int txqno)
{
	struct ccci_header *ccci_h;
	struct ccci_header ccci_h_bk;
	struct md_ccif_ctrl *md_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(hif_id);

	ccci_h =
		(struct ccci_header *)&md_ctrl->ccif_sram_layout->up_header;
	/*header */
	ccif_write32(&ccci_h->data[0], 0, 0x00);
	ccif_write32(&ccci_h->data[1], 0, packet_size);
	ccif_write32(&ccci_h->reserved, 0, MD_INIT_CHK_ID);
	/*ccif_write32(&ccci_h->channel,0,CCCI_CONTROL_TX); */
	/*as Runtime data always be the first packet
	 * we send on control channel
	 */
	ccif_write32((u32 *) ccci_h + 2, 0, tx_ch);
	/*ccci_header need backup for log history*/
	ccci_h_bk.data[0] = ccif_read32(&ccci_h->data[0], 0);
	ccci_h_bk.data[1] = ccif_read32(&ccci_h->data[1], 0);
	*((u32 *)&ccci_h_bk + 2) = ccif_read32((u32 *) ccci_h + 2, 0);
	ccci_h_bk.reserved = ccif_read32(&ccci_h->reserved, 0);
	ccci_md_add_log_history(&md_ctrl->traffic_info, OUT,
		(int)txqno, &ccci_h_bk, 0);

	return (void *)&md_ctrl->ccif_sram_layout->ap_rt_data;
}

static struct ccci_hif_ops ccci_hif_ccif_ops = {
	.send_skb = &md_ccif_op_send_skb,
	.give_more = &md_ccif_op_give_more,
	.write_room = &md_ccif_op_write_room,
	.stop_queue = &md_ccif_stop_queue,
	.start_queue = &md_ccif_start_queue,
	.dump_status = &md_ccif_op_dump_status,

	.start = &ccif_start,
	.stop = &ccif_stop,
	.debug = &ccif_debug,
	.send_data = &md_ccif_send_data,
	.fill_rt_header = &ccif_hif_fill_rt_header,
};

static u64 ccif_dmamask = DMA_BIT_MASK(36);
/*
 * v443 DIAGNOSTIC (read-only, provenance-driven).
 *
 * Phase B of the C1 investigation asked specifically for the channel ENABLE /
 * IRQ mask class.  `hif/ccif_hif_reg.h` defines APCCIF_IRQ0_MASK (0x20) and
 * APCCIF_IRQ1_MASK (0x24) on both banks, and those two registers have never
 * been captured in this project -- only CON/BUSY/START/TCHNUM/RCHNUM/ACK (and
 * the CHDATA window) were ever read.
 *
 * This compares the bootloader-provided CCIF0 state (probe, before any Linux
 * write) against the state after Linux's own CCIF setup (end of
 * ccci_reset_ccif_hw), so any bit that mainline destroys is visible.  Read-only:
 * nothing is written, no mask is modified, nothing is ACKed.
 */
static void ccif_dump_bank(struct md_ccif_ctrl *md_ctrl, const char *tag)
{
	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"BANK %s A: CON=%08x BUSY=%08x START=%08x TCHNUM=%08x RCHNUM=%08x ACK=%08x IRQ0M=%08x IRQ1M=%08x\n",
		tag,
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_CON),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_TCHNUM),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_ACK),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_IRQ0_MASK),
		ccif_read32(md_ctrl->ccif_ap_base, APCCIF_IRQ1_MASK));
	CCCI_NORMAL_LOG(md_ctrl->md_id, TAG,
		"BANK %s B: CON=%08x BUSY=%08x START=%08x TCHNUM=%08x RCHNUM=%08x ACK=%08x IRQ0M=%08x IRQ1M=%08x\n",
		tag,
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_CON),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_BUSY),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_START),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_TCHNUM),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_RCHNUM),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_ACK),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_IRQ0_MASK),
		ccif_read32(md_ctrl->ccif_md_base, APCCIF_IRQ1_MASK));
}

static int ccif_hif_hw_init(struct device *dev, struct md_ccif_ctrl *md_ctrl)
{
	struct device_node *node = NULL;
	int idx = 0;
	int ret;

	if (!dev) {
		CCCI_ERROR_LOG(-1, TAG, "No ccif driver in dtsi\n");
		ret = -3;
		return ret;
	}

	if (IS_ERR(md_ctrl->plat_val.infra_ao_base)) {
		CCCI_ERROR_LOG(-1, TAG, "No infra_ao register in dtsi\n");
		ret = -4;
		return ret;
	}

	node = dev->of_node;
	if (!node) {
		CCCI_ERROR_LOG(-1, TAG, "No ccif node in dtsi\n");
		ret = -5;
		return ret;
	}
	md_ctrl->ccif_ap_base = of_iomap(node, 0);
	md_ctrl->ccif_md_base = of_iomap(node, 1);

	md_ctrl->ccif2_ap_base = of_iomap(node, 2);
	md_ctrl->ccif2_md_base = of_iomap(node, 3);

	/* v443 DIAGNOSTIC: bootloader-provided CCIF0 state, before any Linux
	 * write to this block (timeline point "bootloader entry"). */
	ccif_dump_bank(md_ctrl, "BOOT");

	md_ctrl->ap_ccif_irq0_id = irq_of_parse_and_map(node, 0);
	md_ctrl->ap_ccif_irq1_id = irq_of_parse_and_map(node, 1);

	/* Device tree using none flag to register irq,
	 * sensitivity has set at "irq_of_parse_and_map"
	 */
	md_ctrl->ap_ccif_irq0_flags = IRQF_TRIGGER_NONE;
	md_ctrl->ap_ccif_irq1_flags = IRQF_TRIGGER_NONE;
	ret = of_property_read_u32(dev->of_node,
		"mediatek,sram_size", &md_ctrl->sram_size);
	if (ret < 0)
		md_ctrl->sram_size = CCIF_SRAM_SIZE;
	md_ctrl->ccif_sram_layout =
		(struct ccif_sram_layout *)(md_ctrl->ccif_ap_base
		+ APCCIF_CHDATA);
	for (idx = 0; idx < ARRAY_SIZE(ccif_clk_table); idx++) {
		ccif_clk_table[idx].clk_ref = devm_clk_get(dev,
			ccif_clk_table[idx].clk_name);
		if (IS_ERR(ccif_clk_table[idx].clk_ref)) {
			CCCI_ERROR_LOG(-1, TAG,
				 "ccif get %s failed\n",
					ccif_clk_table[idx].clk_name);
			ccif_clk_table[idx].clk_ref = NULL;
		}
	}
	dev->dma_mask = &ccif_dmamask;
	dev->coherent_dma_mask = ccif_dmamask;
	dev->platform_data = md_ctrl;
	node = of_find_compatible_node(NULL, NULL,
		"mediatek,md_ccif4");
	if (node) {
		md_ctrl->md_ccif4_base = of_iomap(node, 0);
		if (!md_ctrl->md_ccif4_base) {
			CCCI_ERROR_LOG(-1, TAG,
				"ccif4_base fail\n");
			return -6;
		}
	}
	node = of_find_compatible_node(NULL, NULL,
		"mediatek,md_ccif5");
	if (node) {
		md_ctrl->md_ccif5_base = of_iomap(node, 0);
		if (!md_ctrl->md_ccif5_base) {
			CCCI_ERROR_LOG(-1, TAG,
				"ccif5_base fail\n");
			return -7;
		}
	}
	/* Get pericfg base(0x10003000) for ccif4,5 */
	md_ctrl->pericfg_base = syscon_regmap_lookup_by_phandle(dev->of_node,
		"ccif-pericfg");
	if (IS_ERR(md_ctrl->pericfg_base))
		CCCI_ERROR_LOG(-1, TAG,
			"%s: get ccif-pericfg failed\n", __func__);

	if (!md_ctrl->ccif_ap_base || !md_ctrl->ccif_md_base) {
		CCCI_ERROR_LOG(-1, TAG,
			"ap_ccif_base=NULL or ccif_md_base NULL\n");
		return -2;
	}

	pr_info("CCIF bases: ap=%px md=%px ccif2_ap=%px ccif2_md=%px\n",
		md_ctrl->ccif_ap_base, md_ctrl->ccif_md_base,
		md_ctrl->ccif2_ap_base, md_ctrl->ccif2_md_base);

	if (!md_ctrl->ccif2_ap_base || !md_ctrl->ccif2_md_base)
		CCCI_ERROR_LOG(-1, TAG,
			"ccif2_ap_base=NULL or ccif2_md_base NULL\n");

	if (md_ctrl->ap_ccif_irq0_id == 0 ||
		md_ctrl->ap_ccif_irq1_id == 0) {
		CCCI_ERROR_LOG(-1, TAG,
			"ccif_irq0:%d,ccif_irq1:%d\n",
			md_ctrl->ap_ccif_irq0_id, md_ctrl->ap_ccif_irq1_id);
		return -2;
	}

	CCCI_DEBUG_LOG(-1, TAG,
		"ap_ccif_base:0x%p, ccif_md_base:0x%p\n",
		md_ctrl->ccif_ap_base,
		md_ctrl->ccif_md_base);
	CCCI_DEBUG_LOG(-1, TAG, "ccif_irq0:%d,ccif_irq1:%d\n",
		md_ctrl->ap_ccif_irq0_id, md_ctrl->ap_ccif_irq1_id);
	spin_lock_init(&md_ctrl->mask_lock);
	ret = request_irq(md_ctrl->ap_ccif_irq0_id, md_ccif_isr,
			md_ctrl->ap_ccif_irq0_flags, "CCIF_AP_DATA0", md_ctrl);
	if (ret) {
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"request CCIF_AP_DATA IRQ0(%d) error %d\n",
			md_ctrl->ap_ccif_irq0_id, ret);
		return -1;
	}
	ret = irq_set_irq_wake(md_ctrl->ap_ccif_irq0_id, 1);
	if (ret)
		CCCI_ERROR_LOG(md_ctrl->md_id, TAG,
			"irq_set_irq_wake ccif ap_ccif_irq0_id(%d) error %d\n",
			md_ctrl->ap_ccif_irq0_id, ret);

	ret = of_property_read_u32(dev->of_node, "mediatek,ccif_hw_reset_ver",
			&md_ctrl->ccif_hw_reset_ver);
	if (ret < 0)
		md_ctrl->ccif_hw_reset_ver = 0;

	if (md_ctrl->ccif_hw_reset_ver == 1) {
		node = of_find_compatible_node(NULL, NULL, "mediatek,infracfg");

		if (!node) {
			CCCI_ERROR_LOG(-1, TAG,
				       "[%s] error: infracfg node is not exist\n",
				       __func__);
			return -8;
		}

		md_ctrl->infracfg_base = of_iomap(node, 0);
		if (!md_ctrl->infracfg_base) {
			CCCI_ERROR_LOG(-1, TAG,
				       "[%s] error: infracfg_base fail\n",
				       __func__);
			return -8;
		}
	}

	return 0;

}

int ccci_ccif_hif_init(struct platform_device *pdev,
	unsigned char hif_id, unsigned char md_id)
{
	int i, ret;
	struct device_node *node_md;
	struct md_ccif_ctrl *md_ctrl;

	spin_lock_init(&devapc_flag_lock);

	md_ctrl = kzalloc(sizeof(struct md_ccif_ctrl), GFP_KERNEL);
	if (!md_ctrl) {
		CCCI_ERROR_LOG(-1, TAG,
			"%s:alloc hif_ctrl fail\n", __func__);
		return -1;
	}
	/* ccif_ctrl = md_ctrl; */
	INIT_WORK(&md_ctrl->ccif_sram_work, md_ccif_sram_rx_work);
	INIT_DELAYED_WORK(&ccif_obs_dw, ccif_obs_work);
	INIT_DELAYED_WORK(&md_ctrl->data0_poll_work, md_ccif_data0_poll);
	atomic_set(&md_ctrl->data0_irq_masked, 0);

	timer_setup(&md_ctrl->traffic_monitor, md_ccif_traffic_monitor_func, 0);
	md_ctrl->heart_beat_counter = 0;
	INIT_WORK(&md_ctrl->traffic_info.traffic_work_struct,
		md_ccif_traffic_work_func);

	md_ctrl->channel_id = 0;
	md_ctrl->md_id = md_id;
	md_ctrl->hif_id = hif_id;
	node_md = of_find_compatible_node(NULL, NULL,
		"mediatek,mddriver");
	of_property_read_u32(node_md,
		"mediatek,md_generation", &md_ctrl->plat_val.md_gen);
	md_ctrl->plat_val.infra_ao_base =
		syscon_regmap_lookup_by_phandle(node_md,
		"ccci-infracfg");
	if (IS_ERR(md_ctrl->plat_val.infra_ao_base)) {
		CCCI_ERROR_LOG(-1, TAG, "infra_ao_base get fail\n");
		return -2;
	}

	atomic_set(&md_ctrl->reset_on_going, 1);
	md_ctrl->wakeup_ch = 0;
	atomic_set(&md_ctrl->ccif_irq_enabled, 1);
	atomic_set(&md_ctrl->ccif_irq1_enabled, 1);
	ccci_reset_seq_num(&md_ctrl->traffic_info);

	/*init queue */
	for (i = 0; i < QUEUE_NUM; i++) {
		md_ccif_queue_struct_init(&md_ctrl->txq[i],
			md_ctrl->hif_id, OUT, i);
		md_ccif_queue_struct_init(&md_ctrl->rxq[i],
			md_ctrl->hif_id, IN, i);
	}

	md_ctrl->ops = &ccci_hif_ccif_ops;
	md_ctrl->plat_dev = pdev;
	ret = ccif_hif_hw_init(&pdev->dev, md_ctrl);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG, "ccci ccif hw init fail");
		return ret;
	}
	ccci_hif_register(md_ctrl->hif_id, (void *)md_ctrl, &ccci_hif_ccif_ops);

	return 0;
}

int ccci_hif_ccif_probe(struct platform_device *pdev)
{
	int ret;

	ret = ccci_ccif_hif_init(pdev, CCIF_HIF_ID, MD_SYS1);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, TAG, "ccci ccif init fail");
		return ret;
	}

	return 0;
}

static int ccif_suspend_noirq(struct device *dev)
{
	return 0;
}

static int ccif_resume_noirq(struct device *dev)
{
	struct arm_smccc_res res;
	struct md_ccif_ctrl *ccif_ctrl =
		(struct md_ccif_ctrl *)ccci_hif_get_by_id(CCIF_HIF_ID);
	unsigned int ccif_ch;

	if (ccif_ctrl && ccif_ctrl->plat_val.md_gen == 6293)
		ccif_write32(ccif_ctrl->ccif_ap_base, APCCIF_CON, 0x01);
	else if (!ccif_ctrl) {
		CCCI_ERROR_LOG(-1, TAG,
			"[%s] error: ccci_hif_get_by_id failed.", __func__);
		return 0;
	}

	arm_smccc_smc(MTK_SIP_KERNEL_CCCI_CONTROL, MD_CLOCK_REQUEST,
		MD_WAKEUP_AP_SRC, WAKE_SRC_HIF_CCIF0, 0, 0, 0, 0, &res);
	CCCI_NORMAL_LOG(-1, TAG,
		"[%s] flag_1=0x%llx, flag_2=0x%llx, flag_3=0x%llx, flag_4=0x%llx\n",
		__func__, res.a0, res.a1, res.a2, res.a3);
	if (!res.a0 && res.a1 == WAKE_SRC_HIF_CCIF0) {
		ccif_ch = ccif_read32(ccif_ctrl->ccif_ap_base, APCCIF_RCHNUM);
		CCCI_NORMAL_LOG(-1, TAG,
			"CCIF 0 Wake up: channel_id == 0x%x\n", ccif_ch);
		ccif_ctrl->wakeup_ch = ccif_ch;
		ccif_ctrl->wakeup_count++;
		if (test_and_clear_bit(AP_MD_CCB_WAKEUP,
			&ccif_ctrl->wakeup_ch))
			CCCI_NOTICE_LOG(ccif_ctrl->md_id, TAG,
				"CCIF_MD wakeup source:(CCB)(%u)\n",
				ccif_ctrl->wakeup_count);
	}
	return 0;
}


static const struct dev_pm_ops ccif_pm_ops = {
	.suspend_noirq = ccif_suspend_noirq,
	.resume_noirq = ccif_resume_noirq,
};

static const struct of_device_id ccci_ccif_of_ids[] = {
	{.compatible = "mediatek,ccci_ccif"},
	{}
};

static struct platform_driver ccci_hif_ccif_driver = {

	.driver = {
		.name = "ccci_hif_ccif",
		.of_match_table = ccci_ccif_of_ids,
		.pm = &ccif_pm_ops,
	},

	.probe = ccci_hif_ccif_probe,
};

static int __init ccci_hif_ccif_init(void)
{
	int ret;

	ret = platform_driver_register(&ccci_hif_ccif_driver);
	if (ret) {
		CCCI_ERROR_LOG(-1, TAG, "ccci hif_ccif driver init fail %d",
			ret);
		return ret;
	}
	return 0;
}

static void __exit ccci_hif_ccif_exit(void)
{
}

module_init(ccci_hif_ccif_init);
module_exit(ccci_hif_ccif_exit);

MODULE_AUTHOR("ccci");
MODULE_DESCRIPTION("ccci hif ccif driver");
MODULE_LICENSE("GPL");
