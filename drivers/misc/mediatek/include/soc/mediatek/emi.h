/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal EMI MPU interface for the SCP port (qqcandy).
 *
 * WORKAROUND: the stock tree implements these in
 * drivers/memory/mediatek/{emimpu,emi-cen}.c, which we have not ported.
 * The only SCP-side user is set_scp_mpu() in scp_helper.c, and its body is
 * compiled out unless CONFIG_MTK_EMI is set — this header only needs to
 * resolve. The stock implementation itself is SMC-backed (arm_smccc via
 * MTK_SIP EMI calls into the stock ATF), so a real port later is feasible:
 * port emimpu.c or inline the SMC sequence if SCP SMEM MPU protection is
 * ever needed. The SCP DRAM regions are already reserved in
 * qqcandy-mblock.dtsi, which keeps AP away from them in the meantime.
 */
#ifndef __SOC_MEDIATEK_EMI_H
#define __SOC_MEDIATEK_EMI_H

#include <linux/types.h>

/* Region ids used by the scp port (values from stock emi.h). */
#define MPU_REGION_ID_SCP_SMEM		26
/* Domain ids. */
#define MPU_DOMAIN_D0			0
#define MPU_DOMAIN_D3			3
/* Access permission. */
#define MTK_EMIMPU_NO_PROTECTION		0

struct emimpu_region_t {
	unsigned int start;
	unsigned int end;
	unsigned int region;
};

static inline int mtk_emimpu_init_region(struct emimpu_region_t *rg,
					 unsigned int region)
{
	return -EOPNOTSUPP;
}

static inline int mtk_emimpu_set_addr(struct emimpu_region_t *rg,
				      phys_addr_t start, phys_addr_t end)
{
	return -EOPNOTSUPP;
}

static inline int mtk_emimpu_set_apc(struct emimpu_region_t *rg,
				     unsigned int domain,
				     unsigned int access_permission)
{
	return -EOPNOTSUPP;
}

static inline int mtk_emimpu_set_protection(struct emimpu_region_t *rg)
{
	return -EOPNOTSUPP;
}

static inline int mtk_emimpu_free_region(struct emimpu_region_t *rg)
{
	return -EOPNOTSUPP;
}

#endif /* __SOC_MEDIATEK_EMI_H */
