/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#ifndef __MTK_CAM_SENINF_ROUTE_H__
#define __MTK_CAM_SENINF_ROUTE_H__

#define MAX_MUX_CHANNEL 4

void mtk_cam_seninf_init_res(struct seninf_core *core);

struct seninf_mux *mtk_cam_seninf_mux_get(struct seninf_ctx *ctx);
struct seninf_mux *mtk_cam_seninf_mux_get_pref(struct seninf_ctx *ctx, int pref_cnt);
void mtk_cam_seninf_mux_put(struct seninf_ctx *ctx, struct seninf_mux *mux);
void mtk_cam_seninf_release_mux(struct seninf_ctx *ctx);
struct seninf_vc *mtk_cam_seninf_get_vc_by_pad(struct seninf_ctx *ctx, int idx);
int mtk_cam_seninf_get_vcinfo(struct seninf_ctx *ctx);

#endif /* __MTK_CAM_SENINF_ROUTE_H__ */
