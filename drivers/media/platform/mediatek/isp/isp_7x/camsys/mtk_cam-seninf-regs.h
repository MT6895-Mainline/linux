/* SPDX-License-Identifier: GPL-2.0 */
// Copyright (c) 2022 MediaTek Inc.

#include <linux/bitfield.h>

#ifndef __MTK_CAM_SENINF_REGS_H__
#define __MTK_CAM_SENINF_REGS_H__

#define SENINF_BITS(base, reg, field, val) do { \
    void __iomem *__p = (void __iomem *)((u8 __iomem *)(base) + (reg)); \
    u32 __v = readl(__p); \
    __v &= ~(field); \
    __v |= FIELD_PREP(field, val); \
    writel(__v, __p); \
} while (0)

#define SENINF_READ_BITS(base, reg, field) \
({ \
    void __iomem *__p = (void __iomem *)((u8 __iomem *)(base) + (reg)); \
    u32 __v = readl(__p); \
    FIELD_GET(field, __v); \
})

#define SENINF_READ_REG(base, reg) \
({ \
	u32 __iomem *__p = (base) + (reg); \
	u32 __v = readl(__p); \
	__v; \
})

#define SENINF_WRITE_REG(base, reg, val) { \
	u32 __iomem *__p = (base) + (reg); \
	writel(val, __p); \
}

#define SENINF_BITS_VAR(base, reg, mask, val) do { \
    void __iomem *__p = (void __iomem *)((u8 __iomem *)(base) + (reg)); \
    u32 __v = readl(__p); \
    __v &= ~(mask); \
    __v |= ((val) << __builtin_ctz(mask)) & (mask); \
    writel(__v, __p); \
} while (0)

#endif
