// SPDX-License-Identifier: GPL-2.0
/* Shared symbols required by independently loadable CCCI modules. */
#include <linux/export.h>
#include <linux/module.h>
#include "inc/ccci_debug.h"

unsigned int ccci_debug_enable = CCCI_LOG_ALL_UART;
EXPORT_SYMBOL_GPL(ccci_debug_enable);
MODULE_LICENSE("GPL");
