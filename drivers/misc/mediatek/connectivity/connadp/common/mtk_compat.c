// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MTK platform compatibility stubs for the mainline port.
 * These replace Android/MediaTek display and EMI MPU services that are
 * not needed for basic Wi-Fi operation on xaga.
 */
#include <linux/notifier.h>
#include <linux/module.h>
#include <linux/types.h>

/*
 * Display notifier stubs.  The downstream Wi-Fi/conninfra code registers
 * fb notifiers for display blank events.  When mediatek_v2 is enabled the
 * real driver provides these symbols, so only provide fallback stubs for
 * the non-v2 mainline DRM configuration.
 */
#include "mtk_disp_notify.h"

#if !defined(CONFIG_DRM_MEDIATEK_V2)
static BLOCKING_NOTIFIER_HEAD(disp_notifier_list);

int mtk_disp_notifier_register(const char *source, struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&disp_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mtk_disp_notifier_register);

int mtk_disp_notifier_unregister(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&disp_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mtk_disp_notifier_unregister);
#endif /* !CONFIG_DRM_MEDIATEK_V2 */

/* Minimal EMI MPU stubs used by the downstream gen4m plat_priv.c. */
struct emimpu_region_t {
unsigned long long start;
unsigned long long end;
unsigned int rg_num;
bool lock;
unsigned int *apc;
};

int mtk_emimpu_init_region(struct emimpu_region_t *rg_info, unsigned int rg_num)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_init_region);

int mtk_emimpu_set_addr(struct emimpu_region_t *rg_info,
unsigned long long start, unsigned long long end)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_set_addr);

int mtk_emimpu_set_apc(struct emimpu_region_t *rg_info,
       unsigned int d_num, unsigned int apc)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_set_apc);

int mtk_emimpu_lock_region(struct emimpu_region_t *rg_info, bool lock)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_lock_region);

int mtk_emimpu_set_protection(struct emimpu_region_t *rg_info)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_set_protection);

int mtk_emimpu_free_region(struct emimpu_region_t *rg_info)
{
return 0;
}
EXPORT_SYMBOL(mtk_emimpu_free_region);

/*
 * WEXT event stub.  Mainline builds with CONFIG_CFG80211_WEXT off do not
 * export wireless_send_event; the downstream gen4m still references it from
 * its legacy WEXT/P2P paths.  A no-op is enough for normal cfg80211 userspace.
 */
#include <net/iw_handler.h>
void wireless_send_event(struct net_device *dev, unsigned int cmd,
 union iwreq_data *wrqu, const char *extra)
{
}
EXPORT_SYMBOL(wireless_send_event);
