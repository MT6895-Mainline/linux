/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VDEC_HW_H
#define __MTK_VCP_VDEC_HW_H
#include "mtk_vcp_vdec.h"
struct platform_device;
struct mtk_vcp_vdec_hw;
struct mtk_vcp_vdec_hw *mtk_vcp_vdec_hw_create(struct platform_device *pdev,
					    struct mtk_vcp *vcp, struct device *ube);
int mtk_vcp_vdec_hw_power(struct mtk_vcp_vdec_hw *hw, unsigned int core, bool on);
/* Ask the shared DVFSRC rail for the VCORE step this stream's throughput needs.
 * Only the voltage is requested; clock rates and mux parents stay with the
 * DVFSRC provider. The step is held for the session, not for one power cycle,
 * and is restored before every power-up.
 *
 * Returns -EOPNOTSUPP without an OPP table, -ERANGE for a stream above the top
 * step and -EIO once hardware state is uncertain, so callers must propagate the
 * failure instead of decoding at a step nobody was granted.
 */
int mtk_vcp_vdec_hw_set_perf(struct mtk_vcp_vdec_hw *hw, u32 width, u32 height,
			      u32 fps);
int mtk_vcp_vdec_hw_wait(struct mtk_vcp_vdec_hw *hw, unsigned int core);
int mtk_vcp_vdec_hw_alloc(struct mtk_vcp_vdec_hw *hw, u32 type, size_t size,
			struct mtk_vcp_mem *mem);
void mtk_vcp_vdec_hw_free(struct mtk_vcp_vdec_hw *hw, u32 type, struct mtk_vcp_mem *mem);
/* Call after successful DEINIT, or confirmed firmware shutdown.
 * Uncertain/owned engines require VCP offline and successful hardware break
 * and reset before PM references or DMA can be released.
 * A nonzero return means shutdown was not confirmed: the caller must keep
 * the session and the voltage request it holds. PM retries do not drop usage
 * references twice; IRQ recovery failure retains clocks, references and DMA.
 * A successful stop ends the
 * session and gives up the stream's step, leaving the request at the top of the
 * OPP table for a session that powers up before its geometry is known.
 */
int mtk_vcp_vdec_hw_stop(struct mtk_vcp_vdec_hw *hw);
#endif
