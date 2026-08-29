/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub for downstream mtk_low_battery_throttling.h.
 * The connadp power throttling code only needs the level enum when the
 * full MTK low-battery framework is not built on mainline.
 */
#ifndef __MTK_LOW_BATTERY_THROTTLING_H__
#define __MTK_LOW_BATTERY_THROTTLING_H__

enum LOW_BATTERY_LEVEL_TAG {
LOW_BATTERY_LEVEL_0 = 0,
LOW_BATTERY_LEVEL_1 = 1,
LOW_BATTERY_LEVEL_2 = 2,
LOW_BATTERY_LEVEL_NUM
};

#endif /* __MTK_LOW_BATTERY_THROTTLING_H__ */
