/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VDEC_HW_H
#define __MTK_VCP_VDEC_HW_H
#include "mtk_vcp_vdec.h"
struct platform_device;
struct mtk_vcp_vdec_hw;
struct mtk_vcp_vdec_hw *mtk_vcp_vdec_hw_create(struct platform_device *pdev,
					    struct mtk_vcp *vcp, struct device *ube);
int mtk_vcp_vdec_hw_power(struct mtk_vcp_vdec_hw *hw, unsigned int core, bool on);
int mtk_vcp_vdec_hw_wait(struct mtk_vcp_vdec_hw *hw, unsigned int core);
int mtk_vcp_vdec_hw_alloc(struct mtk_vcp_vdec_hw *hw, u32 type, size_t size,
			struct mtk_vcp_mem *mem);
void mtk_vcp_vdec_hw_free(struct mtk_vcp_vdec_hw *hw, u32 type, struct mtk_vcp_mem *mem);
/* Call after successful DEINIT, or confirmed reset with no owned engines. */
int mtk_vcp_vdec_hw_stop(struct mtk_vcp_vdec_hw *hw);
#endif
