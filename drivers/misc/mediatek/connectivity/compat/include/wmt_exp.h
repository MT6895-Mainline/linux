/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal wmt_exp.h for the mainline out-of-tree connectivity port.
 * The full common_main/wmt_drv stack is not built; only the types used by
 * wmt_chrdev_wifi are provided.
 */
#ifndef _WMT_EXP_H_
#define _WMT_EXP_H_

#include <linux/types.h>

typedef int MTK_WCN_BOOL;
#define MTK_WCN_BOOL_FALSE 0
#define MTK_WCN_BOOL_TRUE  1

typedef enum {
WMTDRV_TYPE_BT = 0,
WMTDRV_TYPE_FM = 1,
WMTDRV_TYPE_GPS = 2,
WMTDRV_TYPE_WIFI = 3,
WMTDRV_TYPE_WMT = 4,
WMTDRV_TYPE_MAX
} ENUM_WMTDRV_TYPE_T;

extern MTK_WCN_BOOL mtk_wcn_wmt_func_on(ENUM_WMTDRV_TYPE_T type);
extern MTK_WCN_BOOL mtk_wcn_wmt_func_off(ENUM_WMTDRV_TYPE_T type);

#endif /* _WMT_EXP_H_ */
