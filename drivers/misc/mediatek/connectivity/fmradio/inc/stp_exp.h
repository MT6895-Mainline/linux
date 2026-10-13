/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub for the connac2x build: the FM driver includes the legacy WMT/STP
 * headers unconditionally, but the connac2x code path uses nothing from
 * them (the platform layer in plat/conn_infra.c talks to conninfra).
 */
#ifndef _STP_EXP_H_
#define _STP_EXP_H_
#endif
