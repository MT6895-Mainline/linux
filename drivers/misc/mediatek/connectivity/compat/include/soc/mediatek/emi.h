/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal MediaTek EMI header for the mainline out-of-tree port.
 * The downstream EMI MPU is not used; this only provides the data types
 * referenced by the connectivity drivers.
 */
#ifndef __EMI_H__
#define __EMI_H__

#include <linux/types.h>
#include <linux/irqreturn.h>

#define MTK_EMIMPU_NO_PROTECTION0
#define MTK_EMIMPU_SEC_RW1
#define MTK_EMIMPU_SEC_RW_NSEC_R2
#define MTK_EMIMPU_SEC_RW_NSEC_W3
#define MTK_EMIMPU_SEC_R_NSEC_R4
#define MTK_EMIMPU_FORBIDDEN5
#define MTK_EMIMPU_SEC_R_NSEC_RW6

#define MTK_EMIMPU_UNLOCKfalse
#define MTK_EMIMPU_LOCKtrue

struct emimpu_region_t {
unsigned long long start;
unsigned long long end;
unsigned int rg_num;
bool lock;
unsigned int *apc;
};

int mtk_emimpu_init_region(struct emimpu_region_t *rg_info, unsigned int rg_num);
int mtk_emimpu_set_addr(struct emimpu_region_t *rg_info,
unsigned long long start, unsigned long long end);
int mtk_emimpu_set_apc(struct emimpu_region_t *rg_info,
       unsigned int d_num, unsigned int apc);
int mtk_emimpu_lock_region(struct emimpu_region_t *rg_info, bool lock);
int mtk_emimpu_set_protection(struct emimpu_region_t *rg_info);
int mtk_emimpu_free_region(struct emimpu_region_t *rg_info);

#endif /* __EMI_H__ */
