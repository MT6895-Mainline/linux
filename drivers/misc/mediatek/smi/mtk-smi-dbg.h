/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub for the downstream MediaTek SMI debug driver.
 * The full mtk-smi-dbg.c is not ported yet; these stubs keep MMINFRA
 * buildable without changing its downstream source structure.
 */
#ifndef __MTK_SMI_DBG_H__
#define __MTK_SMI_DBG_H__

#include <linux/notifier.h>

static inline int mtk_smi_dbg_register_notifier(struct notifier_block *nb)
{
return 0;
}

static inline void mtk_smi_dbg_cg_status(void)
{
}

#endif
