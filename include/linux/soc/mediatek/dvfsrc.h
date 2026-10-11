/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2021 MediaTek Inc.
 * Copyright (c) 2024 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#ifndef __MEDIATEK_DVFSRC_H
#define __MEDIATEK_DVFSRC_H

enum mtk_dvfsrc_cmd {
	MTK_DVFSRC_CMD_BW,
	MTK_DVFSRC_CMD_HRT_BW,
	MTK_DVFSRC_CMD_PEAK_BW,
	MTK_DVFSRC_CMD_OPP,
	MTK_DVFSRC_CMD_VCORE_LEVEL,
	MTK_DVFSRC_CMD_VSCP_LEVEL,
	MTK_DVFSRC_CMD_MAX,
};

/*
 * Users of the SW_REQ[15:12] DRAM floor. Several devices may hold a floor at
 * once and the hardware serves the highest; this field is separate from the
 * interconnect's SW_BW/SW_PEAK_BW/SW_HRT_BW bandwidth votes, so software
 * floors and hardware votes compose.
 */
enum mtk_dvfsrc_floor_user {
	MTK_DVFSRC_FLOOR_GPU = 0,
	MTK_DVFSRC_FLOOR_VENC,
	MTK_DVFSRC_FLOOR_USERS,
};

#if IS_ENABLED(CONFIG_MTK_DVFSRC)

int mtk_dvfsrc_send_request(const struct device *dev, u32 cmd, u64 data);
int mtk_dvfsrc_query_info(const struct device *dev, u32 cmd, int *data);
void mtk_dvfsrc_set_dram_floor(enum mtk_dvfsrc_floor_user user, u32 dram_opp);

#else

static inline int mtk_dvfsrc_send_request(const struct device *dev, u32 cmd, u64 data)
{ return -ENODEV; }

static inline int mtk_dvfsrc_query_info(const struct device *dev, u32 cmd, int *data)
{ return -ENODEV; }

static inline void mtk_dvfsrc_set_dram_floor(enum mtk_dvfsrc_floor_user user,
					     u32 dram_opp)
{ }

#endif /* CONFIG_MTK_DVFSRC */

#endif
