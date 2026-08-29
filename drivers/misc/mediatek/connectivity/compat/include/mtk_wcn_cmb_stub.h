/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal MTK WCN CMB stub definitions for the mainline out-of-tree port.
 * Only the pieces used by connadp/wmt_build_in_adapter.c are provided.
 */
#ifndef _MTK_WCN_CMB_STUB_H_
#define _MTK_WCN_CMB_STUB_H_

#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of.h>

enum COMBO_IF {
COMBO_IF_UART = 0,
COMBO_IF_MSDC = 1,
COMBO_IF_BTIF = 2,
COMBO_IF_MAX,
};

typedef void (*msdc_sdio_irq_handler_t)(void *);
typedef void (*pm_callback_t)(pm_message_t state, void *data);

struct sdio_ops {
void (*sdio_request_eirq)(msdc_sdio_irq_handler_t irq_handler,
void *data);
void (*sdio_enable_eirq)(void);
void (*sdio_disable_eirq)(void);
void (*sdio_register_pm)(pm_callback_t pm_cb, void *data);
};

extern struct sdio_ops mt_sdio_ops[4];

extern int mtk_wcn_cmb_stub_query_ctrl(void);
extern int mtk_wcn_cmb_stub_trigger_assert(void);
extern void mtk_wcn_cmb_stub_clock_fail_dump(void);
extern int mtk_wcn_sdio_irq_flag_set(int flag);

#endif /* _MTK_WCN_CMB_STUB_H_ */
