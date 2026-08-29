/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal mtk_disp_notify.h for the mainline out-of-tree connectivity port.
 * The display notifier is stubbed in connadp/common/mtk_compat.c.
 */
#ifndef _MTK_DISP_NOTIFY_H_
#define _MTK_DISP_NOTIFY_H_

#include <linux/notifier.h>

#define MTK_DISP_EARLY_EVENT_BLANK 0x00
#define MTK_DISP_BLANK_UNBLANK 0x00
#define MTK_DISP_BLANK_POWERDOWN 0x01

int mtk_disp_notifier_register(const char *source, struct notifier_block *nb);
int mtk_disp_notifier_unregister(struct notifier_block *nb);

#endif /* _MTK_DISP_NOTIFY_H_ */
