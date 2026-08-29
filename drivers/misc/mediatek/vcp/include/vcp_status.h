/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub of the downstream vcp_status.h for the cmdq-ext port.
 * The VCP feature/ready/mem functions are never called on the display path
 * (the call sites are guarded by CONFIG_MTK_TINYSYS_VCP_SUPPORT, which is not
 * enabled for this port). Kept as declarations so mtk-cmdq-helper-ext.c parses.
 */
#ifndef VCP_STATUS_H
#define VCP_STATUS_H

#include <linux/types.h>

#include "vcp.h"

typedef phys_addr_t (*vcp_get_reserve_mem_phys_fp)(enum vcp_reserve_mem_id_t id);
typedef phys_addr_t (*vcp_get_reserve_mem_virt_fp)(enum vcp_reserve_mem_id_t id);
typedef void (*vcp_register_feature_fp)(enum feature_id id);
typedef void (*vcp_deregister_feature_fp)(enum feature_id id);
typedef unsigned int (*is_vcp_ready_fp)(enum vcp_core_id id);

struct vcp_status_fp {
	vcp_get_reserve_mem_phys_fp	vcp_get_reserve_mem_phys;
	vcp_get_reserve_mem_virt_fp	vcp_get_reserve_mem_virt;
	vcp_register_feature_fp		vcp_register_feature;
	vcp_deregister_feature_fp	vcp_deregister_feature;
	is_vcp_ready_fp			is_vcp_ready;
};

phys_addr_t vcp_get_reserve_mem_phys_ex(enum vcp_reserve_mem_id_t id);
phys_addr_t vcp_get_reserve_mem_virt_ex(enum vcp_reserve_mem_id_t id);
void vcp_register_feature_ex(enum feature_id id);
void vcp_deregister_feature_ex(enum feature_id id);
unsigned int is_vcp_ready_ex(enum vcp_core_id id);

#endif
