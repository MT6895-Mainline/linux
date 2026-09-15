/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_RUBENS_EARLYLOG_H
#define _LINUX_RUBENS_EARLYLOG_H

#include <linux/stdarg.h>
#include <linux/types.h>

#ifdef CONFIG_RUBENS_EARLYLOG
void rubens_earlylog_early_init(void);
void rubens_earlylog_late_init(void);
void rubens_earlylog_printk(const char *fmt, va_list args);
void rubens_earlylog_stage(u32 stage);
size_t rubens_earlylog_copy_previous(char *buf, size_t size);
#else
static inline void rubens_earlylog_early_init(void) { }
static inline void rubens_earlylog_late_init(void) { }
static inline void rubens_earlylog_printk(const char *fmt, va_list args) { }
static inline void rubens_earlylog_stage(u32 stage) { }
static inline size_t rubens_earlylog_copy_previous(char *buf, size_t size)
{
	return 0;
}
#endif

#endif
