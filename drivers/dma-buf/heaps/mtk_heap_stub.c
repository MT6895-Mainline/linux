// SPDX-License-Identifier: GPL-2.0
/*
 * MT6895 mainline port: stubs for the downstream MTK dma-heap helpers.
 * There is no secure heap on mainline; dmabuf_to_secure_handle() reports
 * failure so secure (HSF) paths stay disabled.
 */
#include <linux/dma-buf.h>
#include "mtk_heap.h"

u32 dmabuf_to_secure_handle(const struct dma_buf *dmabuf)
{
	return 0;
}
EXPORT_SYMBOL_GPL(dmabuf_to_secure_handle);
