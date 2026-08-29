/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub of the downstream vcp.h for the cmdq-ext port.
 * VCP (video codec processor) is only used by the MDP/MML HDR/AAL readback
 * path, which the display DSC pipeline does not exercise (verified by
 * disabling CONFIG_MTK_TINYSYS_VCP_SUPPORT in the downstream Android build).
 * Only the enum IDs referenced by mtk-cmdq-helper-ext.c are kept.
 */
#ifndef __VCP_H__
#define __VCP_H__

enum vcp_core_id {
	VCP_A_ID = 0,
	VCP_CORE_TOTAL = 1,
};

enum vcp_reserve_mem_id_t {
	VDEC_MEM_ID,
	VENC_MEM_ID,
	VCP_A_LOGGER_MEM_ID,
	VDEC_SET_PROP_MEM_ID,
	VENC_SET_PROP_MEM_ID,
	VDEC_VCP_LOG_INFO_ID,
	VENC_VCP_LOG_INFO_ID,
	GCE_MEM_ID,
	NUMS_MEM_ID,
};

enum feature_id {
	RTOS_FEATURE_ID,
	VDEC_FEATURE_ID,
	VENC_FEATURE_ID,
	GCE_FEATURE_ID,
	NUM_FEATURE_ID,
};

#endif
