/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_MTK_VCP_H
#define __LINUX_MTK_VCP_H
#include <linux/types.h>

struct device;
struct mtk_vcp;
#define MTK_VCP_IPI_MAX_PAYLOAD 64

enum mtk_vcp_codec {
	MTK_VCP_DECODER,
	MTK_VCP_ENCODER,
	MTK_VCP_CODEC_COUNT,
};

enum mtk_vcp_mem_id {
	MTK_VCP_MEM_VDEC,
	MTK_VCP_MEM_VENC,
	MTK_VCP_MEM_LOGGER,
	MTK_VCP_MEM_VDEC_PROP,
	MTK_VCP_MEM_VENC_PROP,
	MTK_VCP_MEM_VDEC_LOG,
	MTK_VCP_MEM_VENC_LOG,
	MTK_VCP_MEM_GCE,
	MTK_VCP_MEM_COUNT,
};

struct mtk_vcp_mem {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

/* Callbacks run in a threaded IRQ and must not register/unregister handlers. */
typedef void (*mtk_vcp_ipi_handler_t)(void *priv, const void *data, size_t len);
struct mtk_vcp *mtk_vcp_get(struct device *dev);
void mtk_vcp_put(struct mtk_vcp *vcp);
/* Reserve the codec engine before boot. Same-owner claims are idempotent.
 * Keep ownership across reset/DRC and failed cleanup; release only after
 * firmware and codec DMA are confirmed stopped.
 */
int mtk_vcp_claim(struct mtk_vcp *vcp, const void *owner);
void mtk_vcp_release(struct mtk_vcp *vcp, const void *owner);
int mtk_vcp_boot(struct mtk_vcp *vcp);
int mtk_vcp_shutdown(struct mtk_vcp *vcp);
bool mtk_vcp_is_offline(struct mtk_vcp *vcp);
/* The caller holds a boot reference during allocation and retains the
 * returned record until firmware release or confirmed processor shutdown.
 */
int mtk_vcp_alloc_workmem(struct mtk_vcp *vcp, size_t size, struct mtk_vcp_mem *mem);
void mtk_vcp_free_workmem(struct mtk_vcp *vcp, struct mtk_vcp_mem *mem);
/* Caller must keep a boot reference; callbacks during boot may also query SHM. */
int mtk_vcp_get_mem(struct mtk_vcp *vcp, unsigned int id, struct mtk_vcp_mem *mem);
int mtk_vcp_ipi_register(struct mtk_vcp *vcp, unsigned int codec,
			 mtk_vcp_ipi_handler_t handler, void *priv);
void mtk_vcp_ipi_unregister(struct mtk_vcp *vcp, unsigned int codec);
int mtk_vcp_ipi_send(struct mtk_vcp *vcp, unsigned int codec,
		     const void *data, size_t len);
/* Decoder FRAME_BUFFER resource service; uses the decoder receive handler. */
int mtk_vcp_vdec_resource_send(struct mtk_vcp *vcp,
			       const void *data, size_t len);
#endif
