// SPDX-License-Identifier: GPL-2.0
#include <linux/dma-buf.h>
#include "mtk_heap.h"

long mtk_dma_buf_set_name(struct dma_buf *dmabuf, const char *buf)
{
	dma_buf_set_name(dmabuf, buf);
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_dma_buf_set_name);
