/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal pmic_lbat_service.h stub for the mainline out-of-tree port.
 * Battery-voltage TX backoff is not wired up on mainline.
 */
#ifndef __MTK_LBAT_SERVICE_H__
#define __MTK_LBAT_SERVICE_H__

#include <linux/err.h>
#include <linux/types.h>

#define RESTORE_VOLT4200
#define BACKOFF_VOLT3400

struct lbat_user;

static inline struct lbat_user *lbat_user_register(const char *name,
   unsigned int hv_thd_volt,
   unsigned int lv1_thd_volt,
   unsigned int lv2_thd_volt,
   void (*callback)(unsigned int))
{
return ERR_PTR(-EOPNOTSUPP);
}

#endif /* __MTK_LBAT_SERVICE_H__ */
