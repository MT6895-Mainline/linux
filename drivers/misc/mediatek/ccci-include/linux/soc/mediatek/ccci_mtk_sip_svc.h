/* SPDX-License-Identifier: GPL-2.0 */
/* Subset of vendor include/linux/soc/mediatek/mtk_sip_svc.h for the eccci port */
#ifndef __MTK_SIP_SVC_H__
#define __MTK_SIP_SVC_H__

#include <linux/arm-smccc.h>

#define MTK_SIP_SMC_CONVENTION          ARM_SMCCC_SMC_64

#define MTK_SIP_SMC_CMD(fn_id) \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, MTK_SIP_SMC_CONVENTION, \
			   ARM_SMCCC_OWNER_SIP, fn_id)

/* CCCI (MD register access via ATF) */
#define MTK_SIP_KERNEL_CCCI_CONTROL \
	MTK_SIP_SMC_CMD(0x505)
/* TRNG */
#define MTK_SIP_KERNEL_GET_RND \
	MTK_SIP_SMC_CMD(0x26A)

#endif /* __MTK_SIP_SVC_H__ */
