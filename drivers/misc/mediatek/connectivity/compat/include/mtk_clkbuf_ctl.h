/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _COMPAT_MTK_CLKBUF_CTL_H
#define _COMPAT_MTK_CLKBUF_CTL_H

#include <linux/types.h>

/* COMMON_KERNEL_CLK_SUPPORT is defined by conninfra/platform/include/clock_mng.h */
#define CLK_BUF_CONN 0

static inline void KERNEL_clk_buf_ctrl(int id, bool on)
{
}

#endif /* _COMPAT_MTK_CLKBUF_CTL_H */
