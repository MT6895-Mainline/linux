/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub of the downstream iommu_debug.h for the cmdq-ext port.
 * Only the fault-callback hook used by mtk-cmdq-mailbox-ext.c is needed;
 * the full MTK iommu debug framework is not part of this port.
 */
#ifndef IOMMU_DEBUG_H
#define IOMMU_DEBUG_H

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>

typedef int (*mtk_iommu_fault_callback_t)(int port,
				dma_addr_t mva, void *cb_data);

static inline int mtk_iommu_register_fault_callback(int port,
				mtk_iommu_fault_callback_t fn,
				void *cb_data, bool is_vpu)
{
	return 0;
}

static inline int mtk_iommu_unregister_fault_callback(int port, bool is_vpu)
{
	return 0;
}

#endif
