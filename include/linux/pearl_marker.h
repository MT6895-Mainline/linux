/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_PEARL_MARKER_H
#define _LINUX_PEARL_MARKER_H

#include <linux/stdarg.h>
#include <linux/types.h>

#ifdef CONFIG_PEARL_MARKER_WRITER
void pearl_marker_early_init(void);
void pearl_marker_stage(u32 stage);
void pearl_marker_early_printk(const char *fmt, va_list args);
void pearl_marker_put(const char *fmt, ...) __printf(1, 2);
#else
static inline void pearl_marker_early_init(void) { }
static inline void pearl_marker_stage(u32 stage) { }
static inline void pearl_marker_early_printk(const char *fmt, va_list args) { }
#endif

#endif /* _LINUX_PEARL_MARKER_H */
