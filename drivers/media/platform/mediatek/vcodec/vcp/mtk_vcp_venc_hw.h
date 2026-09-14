/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VENC_HW_H
#define __MTK_VCP_VENC_HW_H

#include "mtk_vcp_venc.h"

struct platform_device;
struct mtk_vcp_venc_hw;

/* Probe-time setup only. No firmware start or core power-on is performed. */
struct mtk_vcp_venc_hw *mtk_vcp_venc_hw_create(struct platform_device *pdev,
	struct mtk_vcp *vcp, void (*notify)(void *, u64), void *priv);
const struct mtk_vcp_venc_ops *mtk_vcp_venc_hw_ops(void);
bool mtk_vcp_venc_hw_idle(struct mtk_vcp_venc_hw *hw);
/* Fault cleanup: caller has stopped submissions and VCP must be offline.
 * On failure all power/DMA ownership must be retained by the frontend.
 */
int mtk_vcp_venc_hw_quiesce(struct mtk_vcp_venc_hw *hw);

#endif
