// SPDX-License-Identifier: GPL-2.0
/*
 * Mainline port: minimal stubs for the vendor cmdq-sec-mailbox.c.
 *
 * The secure GCE path (gce_m_sec / MTEE) is not ported on mainline. These
 * no-op implementations let the non-secure CMDQ path link and run; packets
 * that would have been marked secure are submitted through the normal path.
 */
#include <linux/kernel.h>
#include <linux/module.h>

#include "cmdq-sec.h"

s32 cmdq_sec_pkt_set_data(struct cmdq_pkt *pkt, const u64 dapc_engine,
			  const u64 port_sec_engine,
			  const enum CMDQ_SEC_SCENARIO scenario,
			  const enum cmdq_sec_meta_type meta_type)
{
	pr_info_once("[cmdq-sec-stub] secure packet requested; sec path stubbed\n");
	return 0;
}
EXPORT_SYMBOL(cmdq_sec_pkt_set_data);

void cmdq_sec_pkt_set_mtee(struct cmdq_pkt *pkt, const bool enable)
{
}
EXPORT_SYMBOL(cmdq_sec_pkt_set_mtee);

void cmdq_sec_mbox_stop(struct cmdq_client *cl)
{
}
EXPORT_SYMBOL(cmdq_sec_mbox_stop);
