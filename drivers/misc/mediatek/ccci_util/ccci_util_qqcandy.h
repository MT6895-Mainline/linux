/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: prototypes for symbols that the vendor ccci_util got from headers
 * this port does not carry (mtk_spm.h, clkbuf.h, rawbulk, mbim, ...). They are
 * only defined as __weak fallbacks in ccci_util_dummy.c. Declaring them here
 * keeps -Wmissing-prototypes (always on in this tree) quiet without changing
 * the vendor linkage.
 */

#ifndef __CCCI_UTIL_QQCANDY_H__
#define __CCCI_UTIL_QQCANDY_H__

#include <linux/skbuff.h>
#include <linux/types.h>

/* ccci_util_dummy.c fallbacks */
bool spm_is_md1_sleep(void);
void spm_ap_mdsrc_req(u8 lock);
unsigned int mt_irq_get_pending(unsigned int irq);
char *ccci_get_ap_platform(void);
bool is_clk_buf_from_pmic(void);
void clk_buf_get_swctrl_status(void *swctrl_status);
void clk_buf_get_rf_drv_curr(void *rf_drv_curr);
void clk_buf_save_afc_val(unsigned int afcdac);
int rawbulk_push_upstream_buffer(int transfer_id, const void *buffer,
				 unsigned int length);
int mbim_start_xmit(struct sk_buff *skb, int ifid);

/* ccci_util_lib_sys.c helpers consumed by the ECCCI core in the vendor tree */
int ccci_get_plat_ft_inf(char buf[], int size);
int ccci_common_sysfs_init(void);

#endif /* __CCCI_UTIL_QQCANDY_H__ */
