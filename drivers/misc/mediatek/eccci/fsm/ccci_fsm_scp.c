// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */

#ifdef CCCI_KMODULE_ENABLE
#include <linux/string.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#endif
#include <linux/clk.h> /* for clk_prepare/un* */
#include <linux/io.h>  /* PEARL-CCIF2: ioremap/readl/writel */
/* PEARL-SCPDBG-77: debugfs export of SCP DRAM / CCISM smem, for forensics */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/err.h>

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_fsm_internal.h"
#include "md_sys1_platform.h"
#include "modem_sys.h"  /* PEARL-34: ccci_md_get_modem_by_id */

/* PEARL-25: in this tree the SCP driver (CONFIG_MTK_TINYSYS_SCP_SUPPORT) is a
 * standalone module scp.ko, while CCCI is built-in (CONFIG_MTK_CCCI_MAINLINE=y).
 * A built-in object must not reference symbols exported only by a module
 * (scp_A_register_notify / scp_ipidev), or the vmlinux link fails with
 * undefined reference. Only touch these two when SCP is built-in as well.
 */
#if defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT) && \
	!defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT_MODULE)
#define CCCI_SCP_DRIVER_BUILTIN
#endif

/* PEARL-28 stage 1: **the gate is open again**
 * The `#undef CCCI_SCP_DRIVER_BUILTIN` line from round 27 has been removed
 * (see WORKLOG §3.28). It is now exactly on par with #85: the whole chain
 * scp_A_register_notify -> SCP READY -> apsync_event() -> fsm_scp_init0()
 * runs. The only difference was moved into the single mtk_ipi_register()
 * inside fsm_scp_init0(): this round it is replaced with a print. Criteria in
 * the script header comment.
 */

#ifdef FEATURE_SCP_CCCI_SUPPORT
#include "scp_ipi.h"

#ifdef CCCI_KMODULE_ENABLE
void ccci_scp_md_state_sync(int md_state);

/* PEARL-28 stage 2: move the "register IPI_IN_APCCCI_0" action away from
 * "the moment SCP becomes READY", triggering it manually via sysfs (or
 * automatically after READY). The declaration lives here because
 * modem_sys1.c (the sysfs side) calls it, while its definition is near
 * fsm_scp_init0.
 */
int pearl_scp_ipi_register_now(const char *why);
/* PEARL-CCISM: SRAM-channel control message send (defined in ccci_hif_ccif.c) */
extern int md_ccif_send_sram_msg(unsigned char md_id, unsigned int msg);
void pearl_scp_ipi_register_info(int *registered, int *auto_registered,
				int *early_flag);
extern unsigned int pearl_scp_ipi_registered;

struct ccci_fsm_scp ccci_scp_ctl = {
	.md_id = 0,
	.md_state_sync = &ccci_scp_md_state_sync,
};

static struct ccci_clk_node scp_clk_table[] = {
	{ NULL, "infra-ccif2-ap"},
	{ NULL, "infra-ccif2-md"},
};

void ccci_scp_md_state_sync(int md_state)
{
	schedule_work(&ccci_scp_ctl.scp_md_state_sync_work);
}


/* PEARL-25: this used to define a second copy of ccci_debug_enable. The vendor
 * ccci_fsm_scp.o is a standalone module so there was no conflict; after this
 * tree merged it into the built-in ccci_md_all, it collided with the
 * same-named definition at ccci_core.c:37 during the vmlinux.o link:
 * "duplicate symbol: ccci_debug_enable". ccci_debug.h already has the extern
 * declaration, so use the copy in ccci_core.c. */
#endif

static atomic_t scp_state = ATOMIC_INIT(SCP_CCCI_STATE_INVALID);

/* PEARL-28: **deferred** implementation of IPI_IN_APCCCI_0 registration.
 *
 * Stage 1 proved that "this one registration" is the culprit that killed the
 * modem (see WORKLOG §3.28); stage 2 moves it out of fsm_scp_init0() (SCP
 * READY, ~4.15s, pressed right onto the modem's HS1 bring-up window) to a
 * moment **we choose**:
 *   * manual: echo 1 > /sys/kernel/ccci/mdsys1/scp_ipi_register
 *   * auto: with pearl_scp_ipi_autoreg=1, register after md_state syncs to READY
 * Idempotent: scp_register_done is monotonically set to 1; repeated triggers
 * only print.
 */
unsigned int pearl_scp_ipi_autoreg =
#ifdef PEARL28_AUTOREG_HS2
	2;   /* PEARL-28 stage 2c: this image enables auto-registration at build time
	      * (after HS2), because module parameters are not persistent -- a reboot
	      * resets them to 0 -- and SSH on the device only comes up at ~12.5s,
	      * so nothing in userspace can hit the 4.1~7.9s window. */
#else
	0;   /* default 0: no auto-registration (non-static: modem_sys1.c's sysfs reads it) */
#endif
static unsigned int scp_register_done;
/* PEARL-CCISM: idempotence flag for 0x119 already sent during HS2 (the READY-branch send is kept) */
static unsigned int pearl_ccism_init_sent;
static int scp_register_auto_path;
unsigned int pearl_scp_ipi_registered;
module_param(pearl_scp_ipi_registered, uint, 0444);
MODULE_PARM_DESC(pearl_scp_ipi_registered,
	"PEARL-28: 1 when IPI_IN_APCCCI_0 has really been registered");
/* PEARL-28 stage 2b: trigger point for auto-registration.
 *   0 = no auto (default)
 *   1 = register after READY (md_state 4)
 *   2 = register after HS2 (md_state 3, i.e. HS1 has passed)
 * The value 2 is the experimentally found key: registering **before** HS1
 * kills the modem in 0.13s, while registering **after** HS1 is harmless
 * (see WORKLOG §3.28); SSH on the device only comes up at ~12.5s, so the
 * 4.1~7.9s window can only be hit from the kernel itself.
 */
module_param(pearl_scp_ipi_autoreg, uint, 0644);
MODULE_PARM_DESC(pearl_scp_ipi_autoreg,
	"PEARL-28: 0=off 1=register after md READY 2=register after md HS2 "
	"3=register ASAP after SCP subsystem ready (PEARL-30 r121)");

/* PEARL-28 stage 2c: kernel-side delay (ms) of auto-registration, timed from
 * the pearl_scp_ipi_autoreg trigger point. Why it exists: module parameters
 * are **not persistent** (a reboot resets them to 0) and SSH on the device
 * only comes up at ~12.5s, so nothing in userspace can hit the 4.1~7.9s
 * window -- only this can.
 */
unsigned int pearl_scp_ipi_reg_delay_ms =
#ifdef PEARL28_AUTOREG_HS2
	PEARL28_AUTOREG_HS2;   /* delay in ms (build-time default, for landing-point experiments) */
#else
	0;
#endif
module_param(pearl_scp_ipi_reg_delay_ms, uint, 0644);

/* PEARL-CCISMOFF (r119): master gate for boot-time injection; default 0 =
 * vendor behavior. The vendor sends 0x119/0x11B only after READY via queue0;
 * a boot-time injected 0x119 permanently blocks the two AP->MD control
 * channels ch0/ch15 (the MD never consumes -> MD ccismcore times out waiting
 * for the response and asserts at ccismcore_ccci.c:1317). Set 1 to restore
 * the old injection behavior (control experiment). */
static unsigned int pearl_ccism_inject;
module_param(pearl_ccism_inject, uint, 0644);

/* PEARL-CCISMPUB (r129): 1 = publish the CCISM_SCP shared memory to the SCP
 * as soon as it is ready (memset + CCIF2 SRAM key + IPI CCCI_OP_SHM_INIT),
 * and restore the vendor behavior of unconditionally sending 0x11B on SCP
 * RBREADY. 0 = back to the vendor timing of "wait for the MD's 0x11A". */
/* PEARL-SCPADDR (r135): override the smem address published to the SCP
 * (0 = use the real physical address). Purpose: the SCP itself prints
 * shm_addr=[0] (the address it read is 0), so A/B-test different views
 * (MD view 0x40020000 / SCP view 0xde020000 ...) to find the form it accepts. */
unsigned int pearl_scp_smem_addr;
module_param(pearl_scp_smem_addr, uint, 0644);
MODULE_PARM_DESC(pearl_scp_smem_addr,
	"PEARL-SCPADDR: override the smem address published to the SCP (0=real)");

/* PEARL-SCPMEM137 (r137): the smem base published to the SCP now uses the
 * DTS reserve-memory-scp_share (0x8F000000 / 0x6a2000). Rationale: right
 * before reading this address the SCP firmware prints
 * "PRINCIPAL.transceiver share memory config", meaning it wants the shared
 * memory base, not the CCISM_SCP (0x8e020000) that r129-r136 kept sending.
 * 0 = back to the real phy. */
unsigned int pearl_scp_smem_force;   /* r147: default 0, no longer forces the smem address */
module_param(pearl_scp_smem_force, uint, 0644);
MODULE_PARM_DESC(pearl_scp_smem_force,
	"PEARL-SCPMEM137: smem base published to SCP (0 = use real phy)");

/* PEARL-SCPKEY137 (r137): pearl_scp_key_break_ms ms after the first publish,
 * start writing a broken key. The SCP's conn_isr_handler re-reads these 3
 * words on every doorbell, and one boot has two doorbells (SCP 2.058s /
 * 3.235s, 1.18s apart), so:
 *  - if the second read sees the broken key -> the SCP prints
 *    "no support smem !" = it is reading exactly the words we wrote;
 *  - if the second read still prints shm_addr=[] -> that tag is not ours
 *    (another publisher or an alias).
 * 0 = never break the key. */
unsigned int pearl_scp_key_break_ms = 0;
module_param(pearl_scp_key_break_ms, uint, 0644);
MODULE_PARM_DESC(pearl_scp_key_break_ms,
	"PEARL-SCPKEY137: ms after first publish to start a broken key (0=never)");

/* PEARL-SCPSWEEP137 (r137): write a unique magic 0xA5A5xxxx at CHDATA
 * +0xC..+0x2C (+0x8 is left for the real address) and use the SCP's printed
 * shm_addr=[...] to determine which offset it actually reads. Only done
 * during the valid-key phase, to avoid dirtying the MD's later real traffic. */
/* PEARL-163-REMOVED: the sweep probe has been deleted; the parameter is kept
 * so old scripts do not error out and has no effect anymore. */
unsigned int pearl_scp_sweep;
module_param(pearl_scp_sweep, uint, 0644);
MODULE_PARM_DESC(pearl_scp_sweep,
	"PEARL-SCPSWEEP137: probe CCIF2 CHDATA +0xC..+0x2C with magics (1=on)");

#define PEARL_SCP_SMEM_KEY_BAD		0xDEADBEEFu
#define PEARL_SCP_SWEEP_LO		0xC
#define PEARL_SCP_SWEEP_HI		0x2C
#define PEARL_SCP_SWEEP_BASE		0xA5A50000u

static int pearl_scp_key_broken;
static unsigned long pearl_scp_pub_first_jiffies;
static unsigned int pearl_scp_pub_calls;

/* PEARL-SCPVIEW138 (r138): address view. r137 proved the SCP's mapping table
 * is ap=0x10000000 -> scp=0x60000000 (delta +0x50000000), i.e. AP view
 * 0x8e020000 is 0xde020000 in SCP eyes. The SCP never got a usable address
 * (CCISM region all zero, shm_addr=[0]), suspected to be because we always
 * handed it AP-view addresses. 1 = convert to the SCP view when sending to
 * the SCP (default on); 0 = as-is (AP view). */
unsigned int pearl_ccism_view = 1;
module_param(pearl_ccism_view, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_view,
	"PEARL-SCPVIEW138: publish SCP-view smem addr (+0x50000000) to SCP (1=on)");

unsigned int pearl_ccism_publish = 1;
module_param(pearl_ccism_publish, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_publish,
	"PEARL-CCISMPUB: 1=publish CCISM_SCP to SCP as soon as SCP is ready");
MODULE_PARM_DESC(pearl_scp_ipi_reg_delay_ms,
	"PEARL-28: delay in ms before the automatic IPI registration");

static void pearl_scp_ipi_reg_delay_fn(struct work_struct *work)
{
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-28 register: delayed trigger fired (%u ms after md_state sync)\n",
		pearl_scp_ipi_reg_delay_ms);
	pearl_scp_ipi_register_now("auto:delayed");
}
static DECLARE_DELAYED_WORK(pearl_scp_ipi_reg_delay_work,
	pearl_scp_ipi_reg_delay_fn);

/* PEARL-30 (r121): autoreg=3 -- register **immediately** after the SCP
 * subsystem is ready (with retries). Goal: complete the AP<->SCP CCCI
 * handshake as soon as the SCP boots (~1.7s), restoring the vendor timing
 * and keeping the whole handshake out of the MD boot window (a collision
 * causes an MD ccismcore assert at 5.2s). */
static unsigned int pearl_scp_ipi_early_tries;
static void pearl_scp_ipi_early_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_scp_ipi_early_work, pearl_scp_ipi_early_fn);

static void pearl_scp_ipi_early_fn(struct work_struct *work)
{
	if (!scp_register_done) {
		pearl_scp_ipi_early_tries++;
		pearl_scp_ipi_register_now("auto:early");
	}
	if (!scp_register_done && pearl_scp_ipi_early_tries < 40)
		schedule_delayed_work(&pearl_scp_ipi_early_work,
			msecs_to_jiffies(250));
}
static struct ccci_ipi_msg scp_ipi_tx_msg;
static struct mutex scp_ipi_tx_mutex;
static struct work_struct scp_ipi_rx_work;
static wait_queue_head_t scp_ipi_rx_wq __maybe_unused;
static struct ccci_skb_queue scp_ipi_rx_skb_list;
static unsigned int init_work_done __maybe_unused;
static unsigned int scp_clk_last_state;
#if (MD_GENERATION >= 6297)
static struct ccci_ipi_msg scp_ipi_rx_msg __maybe_unused;
#endif

static int ccci_scp_ipi_send(int md_id, int op_id, void *data)
{
	int ret = 0;
#if (MD_GENERATION >= 6297) && defined(CCCI_SCP_DRIVER_BUILTIN)
	int ipi_status = 0;
	unsigned int cnt = 0;
#endif
	if (atomic_read(&scp_state) == SCP_CCCI_STATE_INVALID) {
		CCCI_ERROR_LOG(md_id, FSM,
			"ignore IPI %d, SCP state %d!\n",
			op_id, atomic_read(&scp_state));
		return -CCCI_ERR_MD_NOT_READY;
	}

	mutex_lock(&scp_ipi_tx_mutex);
	memset(&scp_ipi_tx_msg, 0, sizeof(scp_ipi_tx_msg));
	scp_ipi_tx_msg.md_id = md_id;
	scp_ipi_tx_msg.op_id = op_id;
	scp_ipi_tx_msg.data[0] = *((u32 *)data);
	CCCI_NORMAL_LOG(scp_ipi_tx_msg.md_id, FSM,
		"IPI send op_id=%d/data=0x%x, size=%d\n",
		scp_ipi_tx_msg.op_id, scp_ipi_tx_msg.data[0],
		(int)sizeof(struct ccci_ipi_msg));
#if (MD_GENERATION >= 6297)
#ifdef CCCI_SCP_DRIVER_BUILTIN
	while (1) {
		ipi_status = mtk_ipi_send(&scp_ipidev, IPI_OUT_APCCCI_0,
		0, &scp_ipi_tx_msg, (sizeof(scp_ipi_tx_msg) / 4), 1);
		if (ipi_status != IPI_PIN_BUSY)
			break;
		cnt++;
		if (cnt > 10) {
			CCCI_ERROR_LOG(md_id, FSM, "IPI send 10 times!\n");
			/* aee_kernel_warning("ccci", "ipi:tx busy");*/
			break;
		}
	}
	if (ipi_status != IPI_ACTION_DONE) {
		CCCI_ERROR_LOG(md_id, FSM, "IPI send fail!\n");
		ret = -CCCI_ERR_MD_NOT_READY;
	}
#else
	/* PEARL-25: scp_ipidev is provided by scp.ko and built-in CCCI cannot
	 * reference it; also, with scp.ko not loaded nobody answers this IPI
	 * (scp_state stays INVALID and the check above already returned). Same
	 * semantics as before: MD not ready. */
	CCCI_NORMAL_LOG(md_id, FSM,
		"PEARL-25 skip SCP IPI %d, SCP driver is a module\n", op_id);
	ret = -CCCI_ERR_MD_NOT_READY;
#endif
#else
	if (mtk_tinysys_scp_ipi_send(IPI_APCCCI, &scp_ipi_tx_msg,
			sizeof(scp_ipi_tx_msg), 1, SCP_A_ID) != SCP_IPI_DONE) {
		CCCI_ERROR_LOG(md_id, FSM, "IPI send fail!\n");
		ret = -CCCI_ERR_MD_NOT_READY;
	}
#endif
	mutex_unlock(&scp_ipi_tx_mutex);
	return ret;
}

static int scp_set_clk_cg(unsigned int on)
{
	int idx, ret;

	if (!(on == 0 || on == 1)) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"%s:on=%u is invalid\n", __func__, on);
		return -1;
	}

	if (on == scp_clk_last_state) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM, "%s:on=%u skip set scp clk!\n",
			__func__, on);
		return 0;
	}

	for (idx = 0; idx < ARRAY_SIZE(scp_clk_table); idx++) {
		if (scp_clk_table[idx].clk_ref == NULL) {
			/* PEARL-25: without platform-device probe the clk was never
			 * filled by devm_clk_get, and clk_prepare_enable(NULL) crashes
			 * on a NULL pointer. */
			CCCI_ERROR_LOG(MD_SYS1, FSM,
				"%s: clk %s not available\n", __func__,
				scp_clk_table[idx].clk_name);
			return -1;
		}
		if (on) {
			ret = clk_prepare_enable(scp_clk_table[idx].clk_ref);
			if (ret) {
				CCCI_ERROR_LOG(MD_SYS1, FSM,
					"open scp clk fail:%s,ret=%d\n",
					scp_clk_table[idx].clk_name, ret);
				return -1;
			}
		} else
			clk_disable_unprepare(scp_clk_table[idx].clk_ref);
	}

	CCCI_NORMAL_LOG(MD_SYS1, FSM, "%s:on=%u set done!\n",
		__func__, on);
	scp_clk_last_state = on;

	return 0;
}

/* PEARL-FSMPFIX-75: CCISM runtime-switchable -- see the ccism_* items in
 * pearl_fs.cfg handled by port_rpc.c. These declarations must precede their
 * use sites (clang reports implicit-function-declaration otherwise). */
extern unsigned int pearl_ccism_cfg_auto(void);
extern unsigned int pearl_ccism_cfg_ms(void);
extern unsigned int pearl_ccism_cfg_11b(void);
extern unsigned int pearl_ccism_cfg_11b_ms(void);
extern unsigned int pearl_ccism_cfg_gen(void);
extern int pearl_ccism_force_init(void); /* PEARL-SCPDT-80 fwd */

static unsigned int pearl_ccism_11b_sent;
/* PEARL-CCISMPUB130: whether the SCP has reported RBREADY (it may do so before the MD powers on) */
static unsigned int pearl_scp_rbready;
static void pearl_ccism_11b_port_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_11b_port_work, pearl_ccism_11b_port_fn);
/* PEARL-SCPREPUB (r134): the SCP itself prints shm_addr=[0] (the address it
 * read is 0), while the AP-side write reads back successfully -> the write is
 * invisible/cleared before the SCP reads it. When the SCP reads is unknown,
 * so periodically re-send the CCIF2 SRAM key+addr and log the read-back. */
static void pearl_scp_repub_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_scp_repub_work, pearl_scp_repub_fn);
/* PEARL-SCPREPUB: defined later; declare it here first (otherwise an implicit declaration type conflict) */
static u32 pearl_scp_smem_publish(u32 smem_phy);
static unsigned int pearl_scp_repub_cnt;
static int pearl_ccism_force_inited; /* PEARL-SCPDT-80 */
static void pearl_ccism_11b_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_11b_work, pearl_ccism_11b_work_fn);

static void pearl_ccism_send_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_send_work, pearl_ccism_send_work_fn);
/* PEARL-CCIF2OBS: forward declaration (the observer body is in the CCIF2 section, but the HS2 branch earlier needs it) */
static void pearl_ccif2_obs_start(void);

/* PEARL-CCISM: delayed-send work (+2s).
 * Lesson #46: sending SRAM 0x119 right at the state 2->3 transition gets its
 * up_header overwritten by the runtime data ~0.2s later, and the MD only
 * enters the "waiting for an SRAM message" window at 7.96s. +2s lands inside
 * that window. */
static unsigned int pearl_ccism_planned;

static void pearl_ccism_send_work_fn(struct work_struct *work)
{
	int ret;

	/* PEARL-CCISMOFF: stopped by default (see the pearl_ccism_inject comment). */
	if (!pearl_ccism_inject)
		return;
	if (pearl_ccism_init_sent)
		return;
	/* PEARL-FSMPFIX-75 step one (planner): the HS2 branch schedules only
	 * +500ms; here (≈HS2+0.5s, the cfg is certainly readable by then) the
	 * cfg is read and the real send is scheduled. Default == old behavior. */
	if (!pearl_ccism_planned) {
		unsigned int au, ms, b11, b11ms, gen;

		pearl_ccism_planned = 1;
		au = pearl_ccism_cfg_auto();
		ms = pearl_ccism_cfg_ms();
		b11 = pearl_ccism_cfg_11b();
		b11ms = pearl_ccism_cfg_11b_ms();
		gen = pearl_ccism_cfg_gen();
		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-FSMPFIX-75: ccism cfg gen=%u auto=%u ms=%u 11b=%u/%u\n",
			gen, au, ms, b11, b11ms);
		if (au)
			schedule_delayed_work(&pearl_ccism_send_work,
				msecs_to_jiffies(ms ? ms : 1));
		else
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-FSMPFIX-75: ccism_auto=0 -> skip 0x119\n");
		if (b11)
			schedule_delayed_work(&pearl_ccism_11b_work,
				msecs_to_jiffies(b11ms ? b11ms : 1));
		return;
	}
	/* PEARL-SCPDT-80 (rev2): ensure SCP already has CCISM_SCP before we
	 * tell MD to init CCISM (0x119). Guarded, so no double publish. */
	if (!pearl_ccism_force_inited) {
		int __fi = pearl_ccism_force_init();
		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-SCPDT-80: force_init (pre-0x119) ret=%d\n", __fi);
		if (__fi == 0)
			pearl_ccism_force_inited = 1;
	}
	/* PEARL-CCISM-SRAM: empirical #45 + r116c prove MD does NOT consume the
	 * queue0 ringbuf doorbell during HS1/HS2 (CCIF ch0 stays BUSY, skb dropped).
	 * MD only consumes the SRAM channel (ch15) during boot, so 0x119 MUST go
	 * via md_ccif_send_sram_msg, not the dead port/queue0 path. */
	ret = md_ccif_send_sram_msg(MD_SYS1, CCISM_SHM_INIT);
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-CCISM: SRAM send CCISM_SHM_INIT(0x119) ret=%d\n",
		ret);
	if (ret >= 0)
		pearl_ccism_init_sent = 1;
}

static void pearl_ccism_11b_work_fn(struct work_struct *work)
{
	int ret;

	/* PEARL-CCISMOFF: stopped by default. */
	if (!pearl_ccism_inject)
		return;
	if (pearl_ccism_11b_sent)
		return;
	if (!pearl_ccism_cfg_11b())
		return;
	/* PEARL-CCISM-SRAM: 0x11B same rationale as 0x119 -- must use the SRAM
	 * channel MD actually consumes during boot, not the dropped queue0. */
	ret = md_ccif_send_sram_msg(MD_SYS1, CCISM_SHM_INIT_DONE);
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-CCISM: SRAM send CCISM_SHM_INIT_DONE(0x11B) ret=%d\n",
		ret);
	if (ret >= 0)
		pearl_ccism_11b_sent = 1;
}

/* PEARL-SCPREPUB (r134) */
static void pearl_scp_repub_fn(struct work_struct *work)
{
	struct ccci_smem_region *r;
	u32 phy = 0;

	if (pearl_scp_repub_cnt >= 120)
		return;
	pearl_scp_repub_cnt++;
	r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
	if (r)
		phy = (u32)r->base_ap_view_phy;
	if (phy) {
		/* PEARL-SCPADDR: runtime-overridable */
		u32 val = pearl_scp_smem_addr ? pearl_scp_smem_addr :
			  (pearl_scp_smem_force ? pearl_scp_smem_force : phy);
		u32 rb;

		/* PEARL-SCPKEY137: after the delay, switch to a broken key, to
		 * distinguish within one boot "the SCP reads our words" from
		 * "the tag has another publisher". */
		if (pearl_scp_key_break_ms && pearl_scp_pub_first_jiffies &&
		    !pearl_scp_key_broken &&
		    jiffies_to_msecs(jiffies - pearl_scp_pub_first_jiffies) >=
		    pearl_scp_key_break_ms) {
			pearl_scp_key_broken = 1;
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPKEY137: writing a broken key from now on (elapsed=%ums)\n",
				jiffies_to_msecs(jiffies -
					pearl_scp_pub_first_jiffies));
		}
		rb = pearl_scp_smem_publish(val);

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-SCPREPUB[%u]: republish addr=0x%x (real=0x%x) rb=0x%08x broken=%d\n",
			pearl_scp_repub_cnt, val, phy, rb,
			pearl_scp_key_broken);
	}
	if (pearl_scp_repub_cnt < 120)
		schedule_delayed_work(&pearl_scp_repub_work,
			msecs_to_jiffies(500));
}

/* PEARL-CCISMPUB130: delay sending 0x11B until the MD reaches HS2 (idempotent) */
static void pearl_ccism_11b_port_fn(struct work_struct *work)
{
	int ret;

	if (pearl_ccism_11b_sent)
		return;
	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
		CCISM_SHM_INIT_DONE, 0, 1);
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-CCISMPUB130: delayed 0x11B ret=%d md_state=%d\n",
		ret, ccci_fsm_get_md_state(MD_SYS1));
	if (ret >= 0)
		pearl_ccism_11b_sent = 1;
}

static void ccci_scp_md_state_sync_work(struct work_struct *work)
{
	struct ccci_fsm_scp *scp_ctl = container_of(work,
		struct ccci_fsm_scp, scp_md_state_sync_work);
	struct ccci_fsm_ctl *ctl = fsm_get_entity_by_md_id(scp_ctl->md_id);
	enum MD_STATE_FOR_USER state;
	int ret = 0;
	int count = 0;

	if (!ctl) {
		CCCI_ERROR_LOG(ctl->md_id, FSM, "%s ctl is NULL !\n", __func__);
		return;
	}

	switch (ctl->md_state) {
	case READY:
		/* PEARL-28 stage 2b: READY means HS1/HS2 are both done, so
		 * registering here never fights for the bring-up window. */
		if ((pearl_scp_ipi_autoreg == 1 || pearl_scp_ipi_autoreg == 2) &&
			!scp_register_done && scp_ctl->md_id == MD_SYS1) {
			scp_register_auto_path = 1;
			pearl_scp_ipi_register_now("auto:md_ready");
		}
		if (scp_ctl->md_id == MD_SYS1) {
			while (count < SCP_BOOT_TIMEOUT/EVENT_POLL_INTEVAL) {
				if (atomic_read(&scp_state) ==
					SCP_CCCI_STATE_BOOTING
					|| atomic_read(&scp_state)
					== SCP_CCCI_STATE_RBREADY
					|| atomic_read(&scp_state)
					== SCP_CCCI_STATE_STOP)
					break;
				count++;
				msleep(EVENT_POLL_INTEVAL);
			}
			if (count == SCP_BOOT_TIMEOUT/EVENT_POLL_INTEVAL)
				CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
					"SCP init not ready!\n");
			else {
				ret = scp_set_clk_cg(1);
				if (ret) {
					CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
						"fail to set scp clk, ret = %d\n", ret);
					break;
				}

				ret = ccci_port_send_msg_to_md(scp_ctl->md_id,
					CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);
				if (ret < 0)
					CCCI_ERROR_LOG(scp_ctl->md_id, FSM,
						"fail to send CCISM_SHM_INIT %d\n",
						ret);
			}
		} else
			break;
		break;
	case INVALID:
	case GATED:
		state = MD_STATE_INVALID;
		ccci_scp_ipi_send(scp_ctl->md_id,
			CCCI_OP_MD_STATE, &state);
		break;
	case BOOT_WAITING_FOR_HS1:
	case BOOT_WAITING_FOR_HS2:
		/* PEARL-SCPMDSTATE (r131): sync the MD's boot state to the SCP.
		 * The vendor syncs only INVALID/READY/EXCEPTION; on this machine the
		 * SCP reports RBREADY before the MD powers on (~3.6s) and is told
		 * "MD INVALID" then, after which it knows nothing of the MD's
		 * 0->2->3 transitions and naturally cannot set up CCISM/CCIF2 with
		 * the MD. */
		if (scp_ctl->md_id == MD_SYS1) {
			enum MD_STATE_FOR_USER st131 =
				ccci_fsm_get_md_state_for_user(scp_ctl->md_id);

			ccci_scp_ipi_send(scp_ctl->md_id,
				CCCI_OP_MD_STATE, &st131);
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPMDSTATE: md_state sync to SCP = %d (md_state=%d)\n",
				st131, ctl->md_state);
		}
		/* PEARL-28 stage 2b: HS1 has passed (2->3 is receiving HS1). With
		 * autoreg=2, register here, to determine whether "before vs after
		 * HS1" is really the dividing line. */
		if (pearl_scp_ipi_autoreg == 2 && !scp_register_done &&
			scp_ctl->md_id == MD_SYS1) {
			scp_register_auto_path = 2;
			if (pearl_scp_ipi_reg_delay_ms) {
				CCCI_NORMAL_LOG(-1, FSM,
					"PEARL-28 register: schedule delayed registration +%u ms\n",
					pearl_scp_ipi_reg_delay_ms);
				schedule_delayed_work(&pearl_scp_ipi_reg_delay_work,
					msecs_to_jiffies(pearl_scp_ipi_reg_delay_ms));
			} else {
				pearl_scp_ipi_register_now("auto:md_hs2");
			}
		}
		/* PEARL-CCISMPUB130: when the SCP has reported RBREADY, re-send the
		 * vendor step (0x11B INIT_DONE) in the HS2 window after the MD really
		 * powers on. The gating allows it by then (the PEARL-CCISM exception
		 * in port_proxy.c). */
		if (scp_ctl->md_id == MD_SYS1 && pearl_scp_rbready &&
		    !pearl_ccism_11b_sent) {
			int r130 = ccci_port_send_msg_to_md(MD_SYS1,
				CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);

			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-CCISMPUB130: sending 0x119 in the HS2 window ret=%d\n", r130);
			/* re-send 0x11B 800ms later (give the MD a chance to reply 0x11A) */
			schedule_delayed_work(&pearl_ccism_11b_port_work,
				msecs_to_jiffies(800));
		}
		/* PEARL-CCISM: proactively send CCISM_SHM_INIT(0x119) during the
		 * HS2 wait stage. The vendor sends it only at READY (after HS2) via
		 * the port_proxy gating, but the evidence: the modem gets stuck in
		 * ccismcore:2004 during HS2 (40s timeout waiting for an AP message),
		 * goes silent after all RPC/FS is done, and
		 * ccismc_polling_submit_one_gpd's GPD submission never completes --
		 * all pointing to the CCISM init handshake 0x119->0x11A->(memset
		 * smem + IPI SCP) being missing before HS2, with the AP waiting for
		 * READY and the MD waiting for 0x119, deadlocking each other. The
		 * IPI has already been registered by the autoreg==2 branch above.
		 */
		/* PEARL-FSMPFIX-75: schedule only the planner (+500ms); the real
		 * delay/switches and whether to also send 0x11B are decided by the
		 * planner reading the cfg. Default (auto=1,ms=2000,11b=0) == old
		 * behavior. */
		if (scp_ctl->md_id == MD_SYS1 && !pearl_ccism_init_sent)
			schedule_delayed_work(&pearl_ccism_send_work,
				msecs_to_jiffies(500));
		/* PEARL-CCIF2OBS: periodically print the CCIF2 registers on both
		 * sides to dmesg during the HS2 window, answering "the IPI returned
		 * ret=0, but did the SCP firmware actually do anything". */
		if (scp_ctl->md_id == MD_SYS1)
			pearl_ccif2_obs_start();
		break;
	case EXCEPTION:
		state = MD_STATE_EXCEPTION;
		ccci_scp_ipi_send(scp_ctl->md_id,
			CCCI_OP_MD_STATE, &state);
		break;
	default:
		break;
	};
}

static void __maybe_unused ccci_scp_ipi_rx_work(struct work_struct *work)
{
	struct ccci_ipi_msg *ipi_msg_ptr = NULL;
	struct sk_buff *skb = NULL;
	int data, ret;

	while (!skb_queue_empty(&scp_ipi_rx_skb_list.skb_list)) {
		skb = ccci_skb_dequeue(&scp_ipi_rx_skb_list);
		if (skb == NULL) {
			CCCI_ERROR_LOG(-1, CORE,
				"ccci_skb_dequeue fail\n");
			return;
		}
		ipi_msg_ptr = (struct ccci_ipi_msg *)skb->data;
		if (!get_modem_is_enabled(ipi_msg_ptr->md_id)) {
			CCCI_ERROR_LOG(ipi_msg_ptr->md_id,
				CORE, "MD not exist\n");
			return;
		}
		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-SCPIPI: op=0x%x data0=0x%x md=%d scp_state=%d\n",
			ipi_msg_ptr->op_id, ipi_msg_ptr->data[0],
			ipi_msg_ptr->md_id, atomic_read(&scp_state));
		switch (ipi_msg_ptr->op_id) {
		case CCCI_OP_SCP_STATE:
			switch (ipi_msg_ptr->data[0]) {
			case SCP_CCCI_STATE_BOOTING:
				if (atomic_read(&scp_state) ==
					SCP_CCCI_STATE_RBREADY) {
					CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"SCP reset detected\n");
					/* PEARL-CCISMOFF: this boot-time 0x119
					 * permanently blocks ch0 (r118 hard
					 * evidence at 7.799s), so not sent by
					 * default. */
					if (pearl_ccism_publish) {
						/* PEARL-CCISM-SRAM1: 0x119 must go
						 * over the SRAM channel. The vendor
						 * gating (port_proxy.c) requires
						 * md_state==READY to let CCCI_SYSTEM_TX
						 * through, but 0x119 must reach the MD
						 * during HS2 -- via the normal port it
						 * would be gated, leaving only the
						 * experimental port_proxy exception;
						 * that path makes the MD skip necessary
						 * CCISM handshake steps and the
						 * security module asserts -1001 after
						 * READY. Going over SRAM (the channel
						 * the MD really consumes during boot)
						 * lets the vendor gating stay intact. */
						md_ccif_send_sram_msg(MD_SYS1,
							CCISM_SHM_INIT);
						md_ccif_send_sram_msg(MD_SYS3,
							C2K_CCISM_SHM_INIT);
					}
				} else {
					CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"SCP boot up\n");
				}
				/* too early to init share memory here,
				 * EMI MPU may not be ready yet
				 */
				break;
			case SCP_CCCI_STATE_RBREADY:
				pearl_scp_rbready = 1;
				CCCI_NORMAL_LOG(-1, FSM,
					"PEARL-CCISMPUB130: SCP RBREADY seen, md_state=%d\n",
					ccci_fsm_get_md_state(ipi_msg_ptr->md_id));
				/* PEARL-SCPDT-80: proactively publish CCISM_SCP shared
				 * memory to SCP the moment SCP is up, so MD can complete
				 * its 0x119 CCISM_SHM_INIT (SCP side) instead of stalling
				 * forever and asserting at ccismcore_ccci.c:1317. */
				if (!pearl_ccism_force_inited) {
					int __fi;

					/* PEARL-30: like register_now, subject to the gate */
					if (!pearl_ccism_publish) {
						CCCI_NORMAL_LOG(-1, FSM,
							"PEARL-30: rx force_init skipped (inject=0)\n");
					} else {
						__fi = pearl_ccism_force_init();
						CCCI_NORMAL_LOG(-1, FSM,
							"PEARL-SCPDT-80: force_init (publish CCISM smem to SCP) ret=%d\n",
							__fi);
						if (__fi == 0)
							pearl_ccism_force_inited = 1;
					}
				}
				/* PEARL-CCISMPUB (r129/r130): the vendor sends
				 * 0x11B unconditionally here, but that assumes "MD
				 * already READY"; on this machine the SCP reports
				 * RBREADY before the MD powers on (3.5s), and a send
				 * then is necessarily lost -> send only when the MD is
				 * already READY, otherwise leave it to the re-send in
				 * the MD's HS2 window (see md_state_sync_work). */
				if (pearl_ccism_publish &&
				    ccci_fsm_get_md_state(ipi_msg_ptr->md_id) == READY) {
					switch (ipi_msg_ptr->md_id) {
					case MD_SYS1:
						/* PEARL-CCISM-SRAM1: same as 0x119, go over SRAM */
						md_ccif_send_sram_msg(MD_SYS1,
							CCISM_SHM_INIT_DONE);
						break;
					case MD_SYS3:
						md_ccif_send_sram_msg(MD_SYS3,
							C2K_CCISM_SHM_INIT_DONE);
						break;
					};
				}
				data =
				ccci_fsm_get_md_state_for_user(
					ipi_msg_ptr->md_id);
				ccci_scp_ipi_send(ipi_msg_ptr->md_id,
					CCCI_OP_MD_STATE, &data);
				break;
			case SCP_CCCI_STATE_STOP:
				CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, FSM,
						"MD INVALID,scp send ack to ap\n");
				ret = scp_set_clk_cg(0);
				if (ret)
					CCCI_ERROR_LOG(ipi_msg_ptr->md_id, FSM,
						"fail to set scp clk, ret = %d\n", ret);
				break;
			default:
				break;
			};
			atomic_set(&scp_state, ipi_msg_ptr->data[0]);
			break;
		default:
			break;
		};
		ccci_free_skb(skb);
	}
}

#if (MD_GENERATION >= 6297)
/*
 * IPI for logger init
 * @param id:   IPI id
 * @param prdata: callback function parameter
 * @param data:  IPI data
 * @param len: IPI data length
 */
static int __maybe_unused ccci_scp_ipi_handler(unsigned int id, void *prdata,
			void *data,
			unsigned int len)
{
	struct sk_buff *skb = NULL;

	if (len != sizeof(struct ccci_ipi_msg)) {
		CCCI_ERROR_LOG(-1, CORE,
		"IPI handler, data length wrong %d vs. %d\n",
		len, (int)sizeof(struct ccci_ipi_msg));
		return -1;
	}

	skb = ccci_alloc_skb(len, 0, 0);
	if (!skb)
		return -1;

	memcpy(skb_put(skb, len), data, len);
	ccci_skb_enqueue(&scp_ipi_rx_skb_list, skb);
	/* ipi_send use mutex, can not be called from ISR context */
	schedule_work(&scp_ipi_rx_work);

	return 0;
}
#else
static void ccci_scp_ipi_handler(int id, void *data, unsigned int len)
{
	struct ccci_ipi_msg *ipi_msg_ptr = (struct ccci_ipi_msg *)data;
	struct sk_buff *skb = NULL;

	if (len != sizeof(struct ccci_ipi_msg)) {
		CCCI_ERROR_LOG(-1, CORE,
		"IPI handler, data length wrong %d vs. %d\n",
		len, (int)sizeof(struct ccci_ipi_msg));
		return;
	}
	CCCI_NORMAL_LOG(ipi_msg_ptr->md_id, CORE,
		"IPI handler %d/0x%x, %d\n",
		ipi_msg_ptr->op_id,
		ipi_msg_ptr->data[0], len);

	skb = ccci_alloc_skb(len, 0, 0);
	if (!skb)
		return;
	memcpy(skb_put(skb, len), data, len);
	ccci_skb_enqueue(&scp_ipi_rx_skb_list, skb);
	/* ipi_send use mutex, can not be called from ISR context */
	schedule_work(&scp_ipi_rx_work);
}
#endif
#endif

/* ===================== PEARL-CCIF2: SCP smem handshake words =====================
 *
 * Hard facts from disassembling the SCP firmware scp_a.img (code.bin):
 *   the op==2 (CCCI_OP_SHM_INIT) branch of ccci_ipi_task:
 *     dc5a: lui a0,0x6023c ; addi a0,a0,0x104 ; lw s1,(a0)   <- key_hi
 *     dc6e: lui a0,0x6023c ; addi a0,a0,0x100 ; lw s3,(a0)   <- key_lo
 *     dc8c: expects key_hi == 0x5343505F , key_lo == 0x534D454D
 *     dca8: bnez -> 0xdd16 = "[CCCI%d/SCP][%s] no support smem !"
 *     dcaa: lui a0,0x6023c ; addi a0,a0,0x108 ; lw a3,(a0)   <- smem physical address
 *
 * Through the SCP's AP->SCP translation table (0x12266, table at 0x6C304,
 * 1->6), 0x6023C100 maps to AP physical 0x1023C100 =
 * ap_ccif2(0x1023C000) + APCCIF_CHDATA(0x100) = the CCIF2 SRAM. All 5
 * `lui 0x6023c` sites in the SCP firmware are lw, never writes; the magic
 * numbers do not exist in the SCP/MD/LK images either => only the AP can
 * write them at runtime.
 *
 * The vendor ccci_scp node reg = <0x1023c000 0x1000>, <0x1023d000 0x1000>
 *   (AP_CCIF2_BASE / MD_CCIF2_BASE), consistent with this file's hardcoding.
 */
#define PEARL_CCIF2_AP_PA	0x1023C000UL
#define PEARL_CCIF2_MD_PA	0x1023D000UL
#define PEARL_CCIF2_CHDATA	0x100
#define PEARL_SCP_SMEM_KEY_LO	0x534D454D	/* "MEMS" */
#define PEARL_SCP_SMEM_KEY_HI	0x5343505F	/* "_PCS" */

/* ===== PEARL-151: CCIF2 CHDATA triple layout experiment =====
 * The SCP smem coordinates LK hands to TFA are three values:
 *   ap_base:0x8E020000  md_base:0x40020000  size:0x8000
 * We used to write the same AP address to CHDATA+0x8/+0xC/+0x10.
 * layout=1: +0x8=ap(scp view), +0xC=md_base, +0x10=size   <- guessed to be what the SCP expects
 * layout=0: all three get the ap address (old behavior, for control)
 */
/* ===== PEARL-162: CCISM_SCP content marking + IPI payload variants =====
 * Goal: the MD's ccci_shm_bm validates the structures in CCISM_SCP (all zero
 * now -> asserts with all-zero para). Fill every 4-byte offset of CCISM_SCP
 * with a marker carrying its own offset number, then see which number appears
 * in the MD assert's para0/para1/para2 -- you immediately know which offset
 * it reads, with no symbol table or disassembly needed.
 *   pearl_ccism_mark: 0=off (keep memset zeros) 1=mark the whole region 2=mark only the first 0x100
 *   pearl_ccism_ipi_mode: 0=u32(AP view) 1=u32(SCP view) 2=u64(AP view) 3=u64(SCP view) 4={ap,md,size}
 * After an MD assert the FSM restarts the MD automatically, so changing these
 * two values at runtime takes effect in the next round (no phone reboot).
 */
unsigned int pearl_ccism_mark = 0;   /* PEARL-171: no longer marks by default */
module_param(pearl_ccism_mark, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_mark, "1=fill CCISM_SCP with offset-tagged markers, 0=leave zeros");

unsigned int pearl_ccism_ipi_mode = 1;
module_param(pearl_ccism_ipi_mode, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_ipi_mode, "0=u32 ap 1=u32 scp 2=u64 ap 3=u64 scp 4={ap,md,size}");

unsigned int pearl_ccism_layout = 1;
module_param(pearl_ccism_layout, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_layout, "1=write {ap,md,size} triple, 0=dup ap addr");

unsigned int pearl_ccism_md_base = 0x40020000;
module_param(pearl_ccism_md_base, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_md_base, "MD-side base of the SCP/CCISM smem (LK said 0x40020000)");

unsigned int pearl_ccism_size = 0x8000;
module_param(pearl_ccism_size, uint, 0644);
MODULE_PARM_DESC(pearl_ccism_size, "size of the SCP/CCISM smem window (LK said 0x8000)");


static void __iomem *pearl_ccif2_ap_base;
static void __iomem *pearl_ccif2_md_base;

/* ---------- PEARL-SCPDBG-77: debugfs forensic exports ----------
 * Background: the AP can already push the smem address to the SCP (key into
 * CCIF2 SRAM + IPI CCCI_OP_SHM_INIT), yet the modem still gets stuck at
 * ccismcore_ccci.c:1317. To tell "the SCP never initialized that smem" from
 * "the modem is waiting for something else", the only way is to read the
 * memory itself.
 *
 *   /sys/kernel/debug/pearl_scp/scp_dram   the SCP's own DRAM/log area (LK reserved)
 *   /sys/kernel/debug/pearl_scp/ccism_scp  CCISM_SCP shared memory (AP<->SCP<->MD)
 *   /sys/kernel/debug/pearl_scp/ccism_mcu  CCISM_MCU shared memory header
 *   /sys/kernel/debug/pearl_scp/ccif2_sram CCIF2 SRAM (where the key lands)
 *
 * Usage: echo init > /sys/kernel/ccci/mdsys1/ccism_send (this also creates the
 *       debugfs nodes), then cat the files above and run strings / hexdump
 *       locally.
 */
/* PEARL-SCPSHARE (r132): AP<->SCP shared reserved region (LK-reserved
 * mblock-25, 6.8MB). The SCP's logs/shared data land here and are readable
 * from the AP view. Expose only the first 64KB to limit risk. */
#define PEARL_SCP_SHARE_PA	0x8F000000UL
#define PEARL_SCP_SHARE_SZ	0x6a2000UL
#define PEARL_SCP_TCM_LOG_PA	0x1c5af000UL	/* SCP TCM + 0x16f000 (the log ring the LOGG packet gives) */
#define PEARL_SCP_TCM_LOG_SZ	0x20000UL
#define PEARL_SCP_DRAM_PA	0xbfc00000UL
#define PEARL_SCP_DRAM_SZ	0x962a0UL
#define PEARL_CCIF2_WIN_SZ	0x1000UL

enum pearl_dbg_kind { PDBG_IOMEM = 0, PDBG_SMEM_SCP, PDBG_SMEM_MCU,
	PDBG_MDBANK, PDBG_L2SNAP, PDBG_SMEM_RAW, PDBG_SCPSHARE };

struct pearl_dbg_node {
	const char *name;
	enum pearl_dbg_kind kind;
	void __iomem *va;
	size_t size;
	void *va_dyn;   /* PEARL-34b: memremap read buffer */
};

/* PEARL-MEMFIX-1: lazily allocate the diagnostic read buffer (defined after pearl_dbg_read) */
static void *pearl_dbg_get_buf(struct pearl_dbg_node *n);


static struct dentry *pearl_dbg_dir;


/* PEARL-MDSNAP140 (r140): periodically snapshot MD memory (0xc0170000, 32MB)
 * during the HS2 window. The md_bank0 node only reads 0xFF after power-down --
 * capture while the MD is still alive. Keep the last snapshot; it remains
 * readable via debugfs after an assert/power-down. */
#define PEARL_MD_SNAP_SZ	0x2000000
/* PEARL-MDSNAP141: the snapshot physical address is a module parameter
 * (default 0xc0170000 = old behavior; both the LK hdr_tbl_inf and the DT
 * md_mem_usage say md1 is at 0xD0000000/480MB, but r126 hung when reading
 * that with GZ present. Probe it after power-down via the parameter, and only
 * snapshot live once it is confirmed safe.) */
unsigned int pearl_md_snap_pa = 0xC0170000;
module_param(pearl_md_snap_pa, uint, 0644);
MODULE_PARM_DESC(pearl_md_snap_pa,
	"PEARL-MDSNAP141: MD bank0 AP PA for snapshot (default 0xc0170000)");
static int pearl_md_snap_done;
static u32 pearl_md_snap_nz;
static u32 pearl_md_snap_ms;

static void __iomem *pearl_dbg_va(struct pearl_dbg_node *n)
{
	struct ccci_smem_region *r;

	switch (n->kind) {
	case PDBG_SMEM_SCP:
		r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
		return (r && r->base_ap_view_vir) ? r->base_ap_view_vir : NULL;
	case PDBG_SMEM_MCU:
		r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_MCU);
		return (r && r->base_ap_view_vir) ? r->base_ap_view_vir : NULL;
	case PDBG_L2SNAP: {
		extern void *pearl_l2sram_snap;

		/* the snapshot is ordinary memory; pearl_dbg_read's memcpy_fromio applies equally */
		return pearl_l2sram_snap;
	}
	case PDBG_SMEM_RAW: {
		struct ccci_smem_region *r = ccci_md_get_smem_by_user_id(
			MD_SYS1, SMEM_USER_RAW_MDSS_DBG);

		return (r && r->base_ap_view_vir) ? r->base_ap_view_vir : NULL;
	}
	case PDBG_MDBANK: {
		/* PEARL-MDSNAP140: return the kernel snapshot when available (avoid reading all 0xFF after power-down) */
		if (pearl_md_snap_done && n->va_dyn) {
			CCCI_NORMAL_LOG(MD_SYS1, FSM,
				"PEARL-MDSNAP140: serving kernel snapshot (ms=%u nz=%u)\n",
				pearl_md_snap_ms, pearl_md_snap_nz);
			return n->va_dyn;
		}
		/*
		 * PEARL-34c (r127): dump of the MD's runtime-decrypted code/data
		 * area. The code segment in md1img is ciphertext (entropy 7.2) and
		 * is decrypted into DRAM at runtime. AP physical address =
		 * 0xC0170000 for 48MB (stock DT modem_info_v2, same as our DT).
		 * Note the LK tag "md_bank0_base"=0xD0000000 is an MD-view address;
		 * memremapping it directly as an AP PA hangs (the r126 lesson).
		 */
		phys_addr_t pa = (phys_addr_t)pearl_md_snap_pa;
		void __iomem *p;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-34c: md_bank0 dump pa=%pap size=0x%zx\n",
			&pa, n->size);
		p = ioremap(pa, n->size);
		if (p) {
			void *dst = pearl_dbg_get_buf(n);
			if (!dst) {
				iounmap(p);
				return NULL;
			}
			memcpy_fromio(dst, p, n->size);
			iounmap(p);
			return dst;
		}
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-34c: ioremap failed\n");
		return NULL;
	}
	case PDBG_SCPSHARE: {
		/* PEARL-SCPSHARE (r132): read the AP<->SCP shared region (64KB from
		 * 0x8f000000). Used to fetch the SCP's own logs and see which CCISM
		 * init step it stopped at. */
		phys_addr_t pa = PEARL_SCP_SHARE_PA;
		void *p;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPSHARE: memremap 0x%llx size=0x%zx\n",
			(unsigned long long)pa, n->size);
		p = memremap(pa, n->size, MEMREMAP_WB);
		if (!p) {
			CCCI_ERROR_LOG(MD_SYS1, FSM,
				"PEARL-SCPSHARE: memremap failed\n");
			return NULL;
		}
		{
			void *dst = pearl_dbg_get_buf(n);
			if (!dst) {
				memunmap(p);
				return NULL;
			}
			memcpy(dst, p, n->size);
		}
		memunmap(p);
		return n->va_dyn;
	}
	default:
		return n->va;
	}
}


/* PEARL-MEMFIX-1: lazily allocate the diagnostic read buffer.
 * Returning NULL means allocation failed (the caller takes its existing error
 * path). */
static void *pearl_dbg_get_buf(struct pearl_dbg_node *n)
{
	if (!n->va_dyn && n->size) {
		n->va_dyn = vzalloc(n->size);
		if (n->va_dyn)
			CCCI_NORMAL_LOG(MD_SYS1, FSM,
				"PEARL-MEMFIX-1: lazily vzalloc %zu bytes for %s\n",
				n->size, n->name);
		else
			CCCI_ERROR_LOG(MD_SYS1, FSM,
				"PEARL-MEMFIX-1: vzalloc %zu failed for %s\n",
				n->size, n->name);
	}
	return n->va_dyn;
}

static ssize_t pearl_dbg_read(struct file *file, char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	/* PEARL-SCPDBG-77: debugfs puts create_file's data in inode->i_private,
	 * not file->private_data (this tree's full_proxy_open_regular does not
	 * move it for us); using the wrong one oopses outright (the first version
	 * measured a crash at pearl_dbg_read+0x28). */
	struct pearl_dbg_node *n =
		(struct pearl_dbg_node *)file_inode(file)->i_private;
	void __iomem *va = pearl_dbg_va(n);
	void *kbuf;
	ssize_t ret;

	if (!va)
		return -ENODEV;
	/* PEARL-34b: md_bank0's va is an ordinary memory buffer (va_dyn), not __iomem */
	if (n->kind == PDBG_MDBANK) {
		if (!n->va_dyn)
			return -ENODEV;
		return simple_read_from_buffer(ubuf, count, ppos,
			n->va_dyn, n->size);
	}
	/* PEARL-CCIF2OBS-78: reading scp_dram (0xbfc00000, SCP-protected DRAM)
	 * triggers a DEVAPC violation and hard-hangs the whole device (measured
	 * twice consistently before r118: one read and SSH goes away). It is only
	 * a forensic export, not a functional path -- refuse reads by default
	 * rather than sacrificing the device. ccif2_sram / ccism_scp / ccism_mcu
	 * are safe MMIO/shared memory and stay allowed. */
	if (!strcmp(n->name, "scp_dram")) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPDBG: scp_dram read disabled (DEVAPC hang guard)\n");
		return -EACCES;
	}
	kbuf = vmalloc(n->size);
	if (!kbuf)
		return -ENOMEM;
	memcpy_fromio(kbuf, va, n->size);
	ret = simple_read_from_buffer(ubuf, count, ppos, kbuf, n->size);
	vfree(kbuf);
	return ret;
}

static const struct file_operations pearl_dbg_fops = {
	.owner = THIS_MODULE,
	.read = pearl_dbg_read,
	.llseek = default_llseek,
};

static struct pearl_dbg_node pearl_dbg_nodes[] = {
	{ "scp_dram",   PDBG_IOMEM,    NULL, PEARL_SCP_DRAM_SZ },
	{ "ccism_scp",  PDBG_SMEM_SCP, NULL, 0x8000 },
	{ "ccism_mcu",  PDBG_SMEM_MCU, NULL, 0x4000 },
	{ "ccif2_sram", PDBG_IOMEM,    NULL, PEARL_CCIF2_WIN_SZ },
	{ "md_bank0",   PDBG_MDBANK,   NULL, 0x2000000 },
	{ "l2sram_snap", PDBG_L2SNAP,  NULL, 0x1800 },
	{ "mdss_dbg",   PDBG_SMEM_RAW, NULL, 0x10000 },
	{ "scp_share",  PDBG_SCPSHARE, NULL, PEARL_SCP_SHARE_SZ },
	/* r158: the SCP firmware's own log ring (inside SCP TCM, not blocked by
	 * DEVAPC; the LOGG packet gives pointers 0x16f008/0x16fc08, base about
	 * 0x16f000 -> AP PA 0x1c5af000) */
	{ "scp_tcmlog", PDBG_IOMEM,    NULL, PEARL_SCP_TCM_LOG_SZ },
};


/* ===== PEARL-163: SCP reset + memory marker scan (find where the SCP reads the smem address from) =====
 *
 * Background: the SCP firmware reads shm_addr during its own boot (t≈3.5s)
 * and reads 0; IPIs from Linux are always one step late. But every time it
 * is reset and restarts it reads again -- so we:
 *   1) write a marker carrying its own offset number to candidate locations
 *   2) reset and release the SCP via SMC (RESET_SET -> RESET_RELEASE)
 *   3) after the SCP restarts, search its log area for "shm_addr=[dead....]"
 *   4) on a hit -> that offset is where it reads the address (print it)
 * All automated on the kernel side, one command, no reflash/reboot needed.
 */
#define PEARL_SIP_TINYSYS_SCP_CONTROL	0x82000301   /* MTK_SIP_TINYSYS_SCP_CONTROL */
#define PEARL_SCP_OP_RESET_SET		2
#define PEARL_SCP_OP_RESET_RELEASE	3

static void pearl163_smc_reset_set(u32 boot_ok)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PEARL_SIP_TINYSYS_SCP_CONTROL, PEARL_SCP_OP_RESET_SET,
		      boot_ok, 0, 0, 0, 0, 0, &res);
}

static void pearl163_smc_reset_release(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(PEARL_SIP_TINYSYS_SCP_CONTROL, PEARL_SCP_OP_RESET_RELEASE,
		      0, 0, 0, 0, 0, 0, &res);
}

static ssize_t pearl163_reset_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-163: manually resetting the SCP\n");
	pearl163_smc_reset_set(0);
	msleep(60);
	pearl163_smc_reset_release();
	msleep(2600);
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-163: SCP reset complete\n");
	return count;
}

static ssize_t pearl163_logdump_read(struct file *file, char __user *ubuf,
				  size_t count, loff_t *ppos)
{
	void __iomem *log = ioremap(0x8f16f000, 0x10000);
	char *buf;
	ssize_t ret;

	if (!log)
		return -ENODEV;
	buf = kmalloc(0x10000, GFP_KERNEL);
	if (!buf) {
		iounmap(log);
		return -ENOMEM;
	}
	memcpy_fromio(buf, log, 0x10000);
	/* give userspace only the printable parts, to avoid flooding */
	{
		size_t i, o = 0;

		for (i = 0; i < 0x10000 - 1; i++) {
			char c = buf[i];

			if (c >= 32 && c < 127) {
				buf[o++] = c;
			} else if (o && buf[o-1] != '\n') {
				buf[o++] = '\n';
			}
		}
		ret = simple_read_from_buffer(ubuf, count, ppos, buf, o);
	}
	kfree(buf);
	iounmap(log);
	return ret;
}


/* PEARL-167: build the header at the SCP's shm using the normal-device
 * (yuechu) template.
 * yuechu normal log: [conn_shm_init] shmaddr=[9ece0000] pat=[46494353][46494353]
 *                  ver=[20210610] [10000] [20][40][60][c000]
 * Address conversion: AP view = EMI base (0x8e500000) + 0x7E0000 = 0x8ece0000
 */
#define PEARL_SCP_SHM_PA	0x8ece0000UL
static ssize_t pearl167_shmhdr_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	void __iomem *va = ioremap(PEARL_SCP_SHM_PA & ~0xfffUL, 0x1000);
	u32 *w;
	int i;

	if (!va)
		return -ENODEV;
	w = (u32 *)(va + (PEARL_SCP_SHM_PA & 0xfff));
	/* template (from the yuechu normal-device log) */
	writel(0x46494353, &w[0]);   /* pat  "FICS" */
	writel(0x46494353, &w[1]);   /* pat  */
	writel(0x20210610, &w[2]);   /* ver 2021-06-10 */
	writel(0x00010000, &w[3]);
	writel(0x00000020, &w[4]);
	writel(0x00000040, &w[5]);
	writel(0x00000060, &w[6]);
	writel(0x0000c000, &w[7]);
	mb();
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-167: wrote SCP shm header @0x%lx: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		PEARL_SCP_SHM_PA, readl(&w[0]), readl(&w[1]), readl(&w[2]),
		readl(&w[3]), readl(&w[4]), readl(&w[5]), readl(&w[6]),
		readl(&w[7]));
	for (i = 8; i < 16; i++)
		writel(0, &w[i]);
	iounmap(va);
	return count;
}

/* ===== PEARL-168: minimal conap (connectivity<->SCP) replica =====
 *
 * Background (from yuechu normal-device logs + the vendor conap_scp driver):
 *   [2.887] [conap_intf_rx_task] drv query [1][0]      <- the SCP queries the AP via conap
 *   [3.206] [conn_shm_init] shmaddr=[9ece0000]          <- only then the SCP sets up its shared memory
 *   vendor: when the SCP is ready the AP sends conap INIT:
 *       conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_INIT, shm_addr, shm_size)
 *       channel IPI_OUT_SCP_CONNSYS(=33), receive IPI_IN_SCP_CONNSYS(=34)
 *
 * Our tree has **no conap driver** => receive pin 34 is never registered and
 *   nobody ever sends INIT to the SCP => the SCP's conap messages are dropped
 *   (the same class of problem as r150's "ready IPI not registered").
 *
 * Minimal replica here: register receive pin 34 (log everything) + provide a switch to send INIT.
 */
struct pearl168_msg_cmd {
	u16 drv_type;
	u16 msg_id;
	u16 total_sz;
	u16 this_sz;
	u32 param0;
	u32 param1;
};

#define PEARL168_DRV_TYPE_CORE		0
#define PEARL168_MSG_INIT		0
#define PEARL168_MSG_DRV_QRY		2
#define PEARL168_MSG_DRV_QRY_ACK	3
#define PEARL168_MSG_TX_ACCEP		5

/* the SCP's shm: EMI base 0x8e500000 + 0x7E0000 (vendor conap's mt6895 table) */
unsigned int pearl168_shm_addr = 0x8ece0000;
module_param(pearl168_shm_addr, uint, 0644);
unsigned int pearl168_shm_size = 0x20000;
module_param(pearl168_shm_size, uint, 0644);
unsigned int pearl168_conap_auto = 1;
module_param(pearl168_conap_auto, uint, 0644);
MODULE_PARM_DESC(pearl168_conap_auto, "1=auto-send conap INIT after SCP ready");

static int pearl168_conap_send(u16 msg_id, u32 p0, u32 p1)
{
	struct pearl168_msg_cmd cmd;
	unsigned int retry;
	int ret = -1;

	cmd.drv_type = PEARL168_DRV_TYPE_CORE;
	cmd.msg_id = msg_id;
	cmd.total_sz = 8;
	cmd.this_sz = 8;
	cmd.param0 = p0;
	cmd.param1 = p1;
	for (retry = 0; retry < 500; retry++) {
		ret = mtk_ipi_send(&scp_ipidev, IPI_OUT_SCP_CONNSYS, 0, &cmd,
				   sizeof(cmd) / 4, 0);
		if (ret == IPI_ACTION_DONE)
			break;
		mdelay(1);
	}
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: conap send msg_id=%u p0=0x%x p1=0x%x ret=%d(0x%x)\n",
		msg_id, p0, p1, ret, IPI_ACTION_DONE);
	return ret;
}

/* receive callback for pin 34 (IPI_IN_SCP_CONNSYS): print the SCP's conap messages */
static int pearl168_conap_recv(unsigned int id, void *prdata, void *data,
			       unsigned int len)
{
	struct pearl168_msg_cmd *c = (struct pearl168_msg_cmd *)data;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: **SCP conap message** id=%u drv=%u msg=%u tot=%u this=%u p0=0x%x p1=0x%x\n",
		id, c->drv_type, c->msg_id, c->total_sz, c->this_sz,
		c->param0, c->param1);
	/* the SCP queries whether the driver is ready -> reply ACK (vendor behavior) */
	if (c->msg_id == PEARL168_MSG_DRV_QRY) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-168: got drv query(drv=%u) -> replying ACK(1)\n",
			c->param0);
		pearl168_conap_send(PEARL168_MSG_DRV_QRY_ACK, c->param0, 1);
	}
	return 0;
}

static u32 pearl168_conap_rxbuf[8];
static int pearl168_registered;

static int pearl168_conap_register(void)
{
	int ret;

	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_SCP_CONNSYS,
			       (void *)pearl168_conap_recv, NULL,
			       &pearl168_conap_rxbuf[0]);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: register IPI_IN_SCP_CONNSYS(34) ret=%d (0=%d)\n",
		ret, IPI_ACTION_DONE);
	return ret;
}

/* debugfs: echo 1 > scp168_conapinit  -> manually send one conap INIT */
static ssize_t pearl168_init_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	int ret;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: manually sending conap INIT shm=0x%x size=0x%x\n",
		pearl168_shm_addr, pearl168_shm_size);
	ret = pearl168_conap_send(PEARL168_MSG_INIT, pearl168_shm_addr,
				  pearl168_shm_size);
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-168: INIT result ret=%d\n", ret);
	return count;
}

/* ===== PEARL-169: random smem key (copying the normal device's behavior) =====
 * yuechu normal-device SCP log: scp_smem_key : 0xE775F5BDE5619A71 (a random
 * 64-bit value different on every boot). We always used the fixed constant
 * 0x5343505F534D454D from LK. Add a switch here to use a random key.
 */
unsigned int pearl169_randkey = 1;
unsigned int pearl170_early_reg = 1;
/* ===== PEARL-172: adjustable CCISM smem base for publish/use =====
 * yuechu normal-device AP log: [ccci1/shm] smem_port->addr_phy=8e02c000
 *   => the normal device uses 0x8e02c000 (region base + 0xC000), while we
 *   always sent 0x8e020000. Make both the "ap base to publish" and the
 * "local base to use" switches here; default them to 0x8e02c000.
 */
unsigned long pearl172_pub_base = 0x8e020000;   /* PEARL-174: use the AP-view address like the normal device */
module_param(pearl172_pub_base, ulong, 0644);
MODULE_PARM_DESC(pearl172_pub_base, "CCISM ap base published to the SCP/MD (0x8e02c000 on the normal device)");

module_param(pearl170_early_reg, uint, 0644);
MODULE_PARM_DESC(pearl170_early_reg, "1=register IPI_IN_APCCCI_0 already in fsm_scp_init0 (ahead of the SCP's CCCI IPI)");
module_param(pearl169_randkey, uint, 0644);
MODULE_PARM_DESC(pearl169_randkey, "1=use a random 64-bit smem key on every publish (normal-device behavior)");

u32 pearl169_key_lo(void)
{
	static u32 lo;

	if (!pearl169_randkey)
		return 0x534d454d;      /* original constant */
	if (!lo)
		lo = get_random_u32();
	return lo;
}

u32 pearl169_key_hi(void)
{
	static u32 hi;

	if (!pearl169_randkey)
		return 0x5343505f;
	if (!hi)
		hi = get_random_u32();
	return hi;
}

/* ===== PEARL-173: open the CCB ports in the kernel on behalf of the userspace daemon =====
 *
 * Normal-device (yuechu/Android) log:
 *   [7.356] [ccci1/chr] port ccci_ccb_ctrl open with flag 20002 by ccci_mdinit
 *   [7.356] [ccci1/shm] ccb_configs_len: 20 / find ccb port ccci_ccb_dhl for user1!
 *   [8.387] [ccci1/shm] smem_port->addr_phy=8e02c000
 *   [8.387] [ccci1/fsm] control message 0x0,0x5555FFFF   <- the MD then sends HS1
 *   [9.254] md_state 3 → 4 (READY)
 *
 * pearl has no ccci_mdinit userspace daemon => the CCB ports are never opened
 *   => smem ports are not set up => the MD's ccci_shm_bm asserts. Open them
 *   here via the kernel API on its behalf.
 */
extern int mtk_ccci_request_port(char *name);
extern int mtk_ccci_open_port(int index);

/*
 * PEARL-206: default 0 -- the ports are owned exclusively by the userspace
 * owner (mtk-ccci-userspace). r173 used to open
 * ccb_ctrl/dhl/md_monitor/meta/raw_dhl/fs in the kernel, so /dev/ccci_fs was
 * held by the kernel and the owner reported at startup:
 *   "CCCI owner refused: [Errno 16] Device or resource busy: '/dev/ccci_fs'".
 * Moreover it ran independently during FSM init, unaffected by pearl_md_autostart.
 */
unsigned int pearl173_open_ccb;
module_param(pearl173_open_ccb, uint, 0644);
MODULE_PARM_DESC(pearl173_open_ccb, "1=kernel opens the CCB/raw ports (ccb_ctrl/dhl/md_monitor/meta/raw_dhl/fs)");

static const char *pearl173_ports[] = {
	"ccci_ccb_ctrl",
	"ccci_ccb_dhl",
	"ccci_ccb_md_monitor",
	"ccci_ccb_meta",
	"ccci_raw_dhl",      /* PEARL-192: opened by emdlogger after READY on the normal device */
	"ccci_fs",           /* PEARL-192: modem file service */
};

static int pearl173_opened;
static void pearl173_open_ports(void)
{
	int i;

	if (!pearl173_open_ccb)
		return;
	for (i = 0; i < (int)ARRAY_SIZE(pearl173_ports); i++) {
		int idx = mtk_ccci_request_port((char *)pearl173_ports[i]);
		int ret = -1;

		if (idx >= 0)
			ret = mtk_ccci_open_port(idx);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-173: open %s idx=%d ret=%d\n",
			pearl173_ports[i], idx, ret);
	}
	pearl173_opened = 1;
}

static void pearl173_work_fn(struct work_struct *work)
{
	pearl173_open_ports();
	msleep(1500);
	pearl173_open_ports();
}

static DECLARE_DELAYED_WORK(pearl173_work, pearl173_work_fn);

static ssize_t pearl173_open_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-173: manually trigger port opening\n");
	pearl173_open_ports();
	return count;
}

/* PEARL-177: "open ports early" entry for the MD auto-start path */
void pearl177_open_ports_early(void)
{
	pearl173_open_ports();
}

/* ===== PEARL-180: re-send CCISM SHM_INIT after the MD comes up (address/timing/mode all runtime-tunable) =====
 *
 * Compared with the normal device (yuechu):
 *   [9.255] [ccci1/fsm] IPI send op_id=2/data=0x8e020000, size=8   <- AP-view address, and after the MD is ready
 * pearl currently sends once at 3.7s (MD not up) with the SCP view 0xde020000.
 * Here: re-send automatically when the MD enters state 3; address and mode are
 * controlled by module parameters (sysfs-writable, no rebuild needed).
 */
unsigned int pearl180_auto = 1;
module_param(pearl180_auto, uint, 0644);
MODULE_PARM_DESC(pearl180_auto, "1=auto re-send CCISM SHM_INIT when the MD enters state 3");
unsigned int pearl180_data = 0x8e020000;   /* the normal device uses the AP-view address */
module_param(pearl180_data, uint, 0644);
MODULE_PARM_DESC(pearl180_data, "address value used for the re-send (0x8e020000 on the normal device)");
unsigned int pearl180_delay_ms = 0;
module_param(pearl180_delay_ms, uint, 0644);
MODULE_PARM_DESC(pearl180_delay_ms, "delay before the re-send (ms)");

static void pearl180_resend(void)
{
	u32 v = pearl180_data;
	int ret;

	if (pearl180_delay_ms)
		msleep(pearl180_delay_ms);
	ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &v);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-180: re-sent SHM_INIT data=0x%x ret=%d\n", v, ret);
}

static void pearl180_work_fn(struct work_struct *w)
{
	pearl180_resend();
	/* re-send twice more, to cover the 0x11A window */
	msleep(300); pearl180_resend();
	msleep(500); pearl180_resend();
}
static DECLARE_DELAYED_WORK(pearl180_work, pearl180_work_fn);

void pearl180_on_md_state(int state)
{
	if (!pearl180_auto || state != 3)
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-180: MD entered state 3 -> scheduling SHM_INIT re-send (data=0x%x)\n",
		pearl180_data);
	schedule_delayed_work(&pearl180_work, msecs_to_jiffies(50));
}

static ssize_t pearl180_send_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-180: manual re-send\n");
	pearl180_resend();
	return count;
}
/* ===== PEARL-181: register all SCP->AP pins and respond per vendor logic =====
 *
 * Measured: the SCP does send "IPI send 0/0x2" (the normal device does too),
 * and the mbox interrupt count really increases, but ccci_scp_ipi_handler()
 * never runs => the SCP uses different pins than the ones we registered
 * (7/9/34). Register a logging handler for pins 0..40 (skipping those already
 * registered) and port over the key branches of vendor ccci_scp_ipi_rx_work:
 *   CCCI_OP_SCP_STATE + SCP_CCCI_STATE_RBREADY  ->  send CCISM_SHM_INIT_DONE(0x11B) to the MD
 */
static int pearl181_recv(unsigned int id, void *prdata, void *data, unsigned int len)
{
	struct ccci_ipi_msg *m = (struct ccci_ipi_msg *)data;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-181: ★SCP→AP pin=%u op_id=%d data0=0x%x len=%u★\n",
		id, m->op_id, m->data[0], len);

	/* vendor: the SCP reports RBREADY -> tell the MD shm init is done (the key HS2 trigger) */
	if (m->op_id == CCCI_OP_SCP_STATE &&
	    m->data[0] == SCP_CCCI_STATE_RBREADY) {
		int ret;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-181: SCP RBREADY -> sending 0x11B to the MD\n");
		ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
					       CCISM_SHM_INIT_DONE, 0, 1);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-181: 0x11B ret=%d\n", ret);
	}
	return 0;
}

/* PEARL-197: 1 = allow r181's full SCP pin registration (default 0, for diagnostics) */
unsigned int pearl181_enable;
module_param(pearl181_enable, uint, 0644);
MODULE_PARM_DESC(pearl181_enable, "1=register all SCP pins 0~40 (steals pins from other subsystems, default off)");

/* PEARL-196: 0 = do not touch sensorhub pins 30/32 (default) */
unsigned int pearl181_force_pins;
module_param(pearl181_force_pins, uint, 0644);
MODULE_PARM_DESC(pearl181_force_pins, "1=also register sensorhub pins 30/32 (breaks the rotation sensor)");

static u32 pearl181_rxbuf[64];

void pearl181_register_all(void)
{
	int i, ok = 0;

	for (i = 0; i <= 40; i++) {
		int ret;

		if (i == 7 || i == 9 || i == 34)   /* skip the already-registered ones */
			continue;
		/*
		 * PEARL-196: 30 / 32 are the sensorhub's IPI pins; never touch them.
		 * The sensor hub's log:
		 *   [5.587] ipi_comm sensor IPI handlers registered (ctrl 30, notify 32)
		 * r181's full registration stole these two, causing
		 *   [10.725] sensor_ready sensor hub gave no ready ack
		 * the rotation sensor (pearl-accel) to fail as a result. Skip them
		 * here; to really use them, enable via pearl181_force_pins
		 * (off by default).
		 */
		if ((i == 30 || i == 32) && !pearl181_force_pins)
			continue;
		ret = mtk_ipi_register(&scp_ipidev, i,
				       (void *)pearl181_recv, NULL,
				       &pearl181_rxbuf[0]);
		if (ret == IPI_ACTION_DONE)
			ok++;
	}
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-181: registered %d extra SCP pins\n", ok);
}

/* ===== PEARL-182: the AP initializes ccism_scp (the MD's shm check reads it) =====
 *
 * Measured: ccism_scp (0x8e020000, 32KB) stays all-zero on pearl and there
 * is no ccism/queue activity on the SCP side either, yet the MD asserts in
 * ccci_shm_bm (shm buffer management). The normal device's SCP logs show no
 * CCISM activity either => that structure is supposed to be built by the
 * AP/MD side.
 *
 * Write a runtime-selectable test pattern into ccism_scp before MD START, to
 * locate which offset/signature the MD reads (once right, the ccci_shm_bm
 * assert should change or disappear).
 */
unsigned int pearl182_enable = 1;
module_param(pearl182_enable, uint, 0644);
MODULE_PARM_DESC(pearl182_enable, "1=initialize ccism_scp before MD START");
unsigned int pearl182_off = 0;          /* start offset (bytes) */
module_param(pearl182_off, uint, 0644);
unsigned int pearl182_pat = 0;          /* 0=fill by offset number; anything else=write that value everywhere */
module_param(pearl182_pat, uint, 0644);
unsigned int pearl182_len = 0x8000;     /* coverage length */
module_param(pearl182_len, uint, 0644);

void pearl182_init_ccism(void)
{
	struct ccci_smem_region *r;
	u32 *p;
	u32 i, n;

	if (!pearl182_enable)
		return;
	r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
	if (!r || !r->base_ap_view_vir) {
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-182: ccism_scp region not available\n");
		return;
	}
	p = (u32 *)r->base_ap_view_vir;
	n = r->size / 4;
	if (pearl182_len / 4 < n)
		n = pearl182_len / 4;
	for (i = 0; i < n; i++)
		p[i] = pearl182_pat ? pearl182_pat
				    : (0xcc150000 | ((i * 4) & 0xffff));
	mb();
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-182: ccism_scp filled off=0x%x len=0x%x pat=0x%x (first word=0x%x)\n",
		pearl182_off, pearl182_len, pearl182_pat, p[0]);
}

static ssize_t pearl182_write(struct file *file, const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	pearl182_init_ccism();
	return count;
}
/* PEARL-183: export the L2SRAM exception snapshot (the MD stack is inside) */
static ssize_t pearl183_l2sram_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	unsigned int len = 0;
	void *p;

	{
		extern void *pearl183_l2sram_get(unsigned int *len);

		p = pearl183_l2sram_get(&len);
	}
	if (!p || !len)
		return -ENODATA;
	return simple_read_from_buffer(ubuf, count, ppos, p, len);
}
static const struct file_operations pearl183_l2sram_fops = {
	.owner = THIS_MODULE, .read = pearl183_l2sram_read,
};

static const struct file_operations pearl182_fops = {
	.owner = THIS_MODULE, .write = pearl182_write,
};

/* ===== PEARL-191: after READY, re-send MD_STATE=2 to the SCP like the normal device =====
 *
 * yuechu (normal device) right after md_state 3->4 (READY):
 *   [9.256] [ccci1/fsm] IPI send op_id=1/data=0x2, size=8
 * pearl has only ever sent data=0x1 (during boot) and 0x3 (on exception),
 * never 0x2. Enumeration: op 1 = CCCI_OP_MD_STATE. Guessed that the SCP
 * needs this 2 to consider the MD ready and activate the MD<->SCP CCISM
 * path; otherwise the MD crashes on its own about 2 seconds after coming up.
 */
unsigned int pearl191_auto = 1;
module_param(pearl191_auto, uint, 0644);
MODULE_PARM_DESC(pearl191_auto, "1=re-send MD_STATE=2 to the SCP after MD READY");
unsigned int pearl191_val = 2;
module_param(pearl191_val, uint, 0644);
MODULE_PARM_DESC(pearl191_val, "MD_STATE value to re-send (2 on the normal device)");

static void pearl191_work_fn(struct work_struct *w)
{
	u32 v = pearl191_val;
	int ret;
	int i;

	for (i = 0; i < 3; i++) {
		ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_MD_STATE, &v);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-191: IPI MD_STATE=%u ret=%d (attempt %d)\n",
			v, ret, i + 1);
		msleep(300);
	}
}
static DECLARE_DELAYED_WORK(pearl191_work, pearl191_work_fn);

void pearl191_on_md_state(int state)
{
	if (!pearl191_auto || state != 4)   /* READY */
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-191: MD READY -> re-sending MD_STATE=%u to the SCP\n", pearl191_val);
	schedule_delayed_work(&pearl191_work, msecs_to_jiffies(20));
}

static ssize_t pearl191_send_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	pearl191_work_fn(NULL);
	return count;
}
/* PEARL-192: read/refresh the MD logs extracted from the CCB DHL */
static ssize_t pearl192_mdlog_read(struct file *file, char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	extern int pearl192_collect(void);
	extern unsigned int pearl192_len(void);
	extern char *pearl192_buf(void);

	if (!pearl192_len())
		pearl192_collect();
	if (!pearl192_len())
		return -ENODATA;
	return simple_read_from_buffer(ubuf, count, ppos, pearl192_buf(),
				       pearl192_len());
}
/* ===== PEARL-194: after READY, re-send system message 0x110 like the normal device =====
 *
 * The normal device (yuechu) sent after READY (9.254s):
 *   [9.257] system message (ffffffff 11b 2 0)   <- 0x11B, we have it
 *   [10.082] system message (ffffffff 110 2 0)  <- **0x110**, we never sent it
 * 0x110 appears about 0.83s after READY, while pearl's MD crashes about 2.1s
 * after READY -- the timing fits "the MD is waiting for 0x110" perfectly.
 * Re-send it over the same path as 0x11B.
 */
unsigned int pearl194_auto = 1;
module_param(pearl194_auto, uint, 0644);
MODULE_PARM_DESC(pearl194_auto, "1=re-send system message 0x110 after MD READY");
unsigned int pearl194_msg = 0x110;
module_param(pearl194_msg, uint, 0644);
MODULE_PARM_DESC(pearl194_msg, "message number to re-send (0x110 on the normal device)");
unsigned int pearl194_delay_ms = 830;
module_param(pearl194_delay_ms, uint, 0644);
MODULE_PARM_DESC(pearl194_delay_ms, "re-send delay (about 830ms after READY on the normal device)");

static void pearl194_work_fn(struct work_struct *w)
{
	int ret;

	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
				       pearl194_msg, 0, 1);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: re-sent system message 0x%x ret=%d\n",
		pearl194_msg, ret);
	/* re-send twice more, in case the first lands outside the window */
	msleep(400);
	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
				       pearl194_msg, 0, 1);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: second send 0x%x ret=%d\n", pearl194_msg, ret);
}
static DECLARE_DELAYED_WORK(pearl194_work, pearl194_work_fn);

void pearl194_on_md_state(int state)
{
	if (!pearl194_auto || state != 4)
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: MD READY -> re-send 0x%x in %u ms\n",
		pearl194_delay_ms, pearl194_msg);
	schedule_delayed_work(&pearl194_work,
			      msecs_to_jiffies(pearl194_delay_ms));
}

static ssize_t pearl194_write(struct file *file, const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	pearl194_work_fn(NULL);
	return count;
}
/* ===== PEARL-199: periodic re-send of system message 0x11f =====
 *
 * Full list of system messages extracted from the reference device's (yuechu)
 * dmesg:
 *   9.254s  0x11a   <- we have it
 *   9.257s  0x11b   <- we have it
 *  10.082s  0x110   <- r194 added it
 *  23.949s  0x11f   <- **we never sent it**
 *  73.940s  0x11f   <- **and it repeats periodically (about every 50 seconds)**
 * So 0x11f is the AP side's (system/fsm subsystem) periodic heartbeat/keepalive
 * message. Send it every 2 seconds from READY here, covering any window the MD
 * might wait in.
 */
unsigned int pearl199_enable = 0;
module_param(pearl199_enable, uint, 0644);
MODULE_PARM_DESC(pearl199_enable, "1=periodically send system message 0x11f after READY");
unsigned int pearl199_msg = 0x11f;
module_param(pearl199_msg, uint, 0644);
MODULE_PARM_DESC(pearl199_msg, "periodic message number (0x11f on the reference device)");
unsigned int pearl199_ms = 2000;
module_param(pearl199_ms, uint, 0644);
MODULE_PARM_DESC(pearl199_ms, "period (milliseconds)");

/* PEARL-199 fwd: the work must be declared before it is scheduled inside a function */
static void pearl199_work_fn(struct work_struct *w);
static DECLARE_DELAYED_WORK(pearl199_work, pearl199_work_fn);

static void pearl199_work_fn(struct work_struct *w)
{
	int ret;

	if (!pearl199_enable)
		return;
	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
				       pearl199_msg, 2, 0);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-199: periodic send 0x%x ret=%d\n", pearl199_msg, ret);
	schedule_delayed_work(&pearl199_work,
			      msecs_to_jiffies(pearl199_ms ? pearl199_ms : 2000));
}
void pearl199_on_md_state(int state)
{
	if (!pearl199_enable || state != 4)
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-199: MD READY -> starting periodic send of 0x%x (%u ms)\n",
		pearl199_msg, pearl199_ms);
	schedule_delayed_work(&pearl199_work, msecs_to_jiffies(200));
}

static ssize_t pearl199_write(struct file *file, const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	pearl199_work_fn(NULL);
	return count;
}
static const struct file_operations pearl199_fops = {
	.owner = THIS_MODULE, .write = pearl199_write,
};

static const struct file_operations pearl194_fops = {
	.owner = THIS_MODULE, .write = pearl194_write,
};

static const struct file_operations pearl192_mdlog_fops = {
	.owner = THIS_MODULE, .read = pearl192_mdlog_read,
};

static ssize_t pearl192_refresh_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	extern int pearl192_collect(void);

	pearl192_collect();
	return count;
}
static const struct file_operations pearl192_refresh_fops = {
	.owner = THIS_MODULE, .write = pearl192_refresh_write,
};

static const struct file_operations pearl191_fops = {
	.owner = THIS_MODULE, .write = pearl191_send_write,
};

static const struct file_operations pearl180_send_fops = {
	.owner = THIS_MODULE, .write = pearl180_send_write,
};

static const struct file_operations pearl173_open_fops = {
	.owner = THIS_MODULE, .write = pearl173_open_write,
};

static const struct file_operations pearl168_init_fops = {
	.owner = THIS_MODULE, .write = pearl168_init_write,
};

static const struct file_operations pearl167_shmhdr_fops = {
	.owner = THIS_MODULE, .write = pearl167_shmhdr_write,
};


static const struct file_operations pearl163_reset_fops = {
	.owner = THIS_MODULE, .write = pearl163_reset_write,
};
static const struct file_operations pearl163_logdump_fops = {
	.owner = THIS_MODULE, .read = pearl163_logdump_read,
};

static void pearl163_tools_init(struct dentry *dir)
{
	if (!dir)
		return;
	/*
	 * PEARL-163-REMOVED: the sweep / poke / fill helpers wrote to SCP,
	 * consys and CCIF physical memory and registers.  They existed to
	 * find where the SCP reads its smem address; that question is
	 * settled (the CCISM handshake completes: 0x119 -> 0x11A -> 0x11B),
	 * so the write paths are gone.  logdump stays, it is read-only.
	 */
	debugfs_create_file("scp167_shmhdr", 0200, dir, NULL, &pearl167_shmhdr_fops);
	debugfs_create_file("scp168_conapinit", 0200, dir, NULL, &pearl168_init_fops);
	debugfs_create_file("scp173_opencecb", 0200, dir, NULL, &pearl173_open_fops);
	debugfs_create_file("scp180_resend", 0200, dir, NULL, &pearl180_send_fops);
	debugfs_create_file("scp182_ccism", 0200, dir, NULL, &pearl182_fops);
	debugfs_create_file("scp192_mdlog", 0400, dir, NULL, &pearl192_mdlog_fops);
	debugfs_create_file("scp199_msg11f", 0200, dir, NULL, &pearl199_fops);
	debugfs_create_file("scp194_msg110", 0200, dir, NULL, &pearl194_fops);
	debugfs_create_file("scp192_refresh", 0200, dir, NULL, &pearl192_refresh_fops);
	debugfs_create_file("scp191_mdstate", 0200, dir, NULL, &pearl191_fops);
	debugfs_create_file("scp183_l2sram", 0400, dir, NULL, &pearl183_l2sram_fops);
	/*
	 * PEARL-197: by default do **not** do the full registration.
	 * r181 once registered all pins 0~40 and stole the sensorhub's
	 * ctrl 30 / notify 32, causing "sensor hub gave no ready ack" and the
	 * rotation sensor to fail. The SCP also hosts audio/VoW, connectivity and
	 * other subsystems, each possibly with its own pins -- a full
	 * registration carries the same risk for them. Changed to opt-in here:
	 * set pearl181_enable to 1 when diagnostics are needed.
	 */
	if (pearl181_enable)
		pearl181_register_all();
	debugfs_create_file("scp163_reset", 0200, dir, NULL, &pearl163_reset_fops);
	debugfs_create_file("scp163_log", 0400, dir, NULL, &pearl163_logdump_fops);
}

static int pearl_scp_dbgfs_init(void)
{
	int i;
	struct dentry *d;

	if (pearl_dbg_dir)
		return 0;
	d = debugfs_create_dir("pearl_scp", NULL);
	if (IS_ERR_OR_NULL(d))
		return -ENOMEM;
	pearl_dbg_dir = d;
	pearl_dbg_nodes[0].va = ioremap(PEARL_SCP_DRAM_PA, PEARL_SCP_DRAM_SZ);
	pearl_dbg_nodes[3].va = ioremap(PEARL_CCIF2_AP_PA, PEARL_CCIF2_WIN_SZ);
	/* r158: SCP TCM log area */
	pearl_dbg_nodes[8].va = ioremap(PEARL_SCP_TCM_LOG_PA, PEARL_SCP_TCM_LOG_SZ);
	/* PEARL-MEMFIX-1: these two read buffers used to be vzalloc'd
	 * unconditionally here -- node[4] is 32MB (md_bank0), node[7] is the SCP
	 * shared region. They are merely "filled when read" debugfs diagnostic
	 * buffers, yet they permanently consumed vmalloc (the previous round
	 * added vmalloc=400M to the cmdline for them and still caused memory
	 * pressure, showing up as high system load / hangs at shutdown and
	 * boot). Changed to on-demand allocation: allocated on the first real
	 * read of the node, see pearl_dbg_read(). The original vzalloc is kept
	 * in .bak for a rollback if needed. */
	pearl_dbg_nodes[4].va_dyn = NULL;
	pearl_dbg_nodes[7].va_dyn = NULL;
	for (i = 0; i < ARRAY_SIZE(pearl_dbg_nodes); i++)
		debugfs_create_file(pearl_dbg_nodes[i].name, 0400,
				    pearl_dbg_dir, &pearl_dbg_nodes[i],
				    &pearl_dbg_fops);
	pearl163_tools_init(d);   /* PEARL-163: sweep / reset / log */
	/* PEARL-173: open the CCB ports for userspace 6.5s later (the MD boot window) */
	if (pearl173_open_ccb && !pearl173_opened)
		schedule_delayed_work(&pearl173_work, msecs_to_jiffies(6500));
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-SCPDBG-77: debugfs pearl_scp/ ready (dram=%px ccif2=%px)\n",
		pearl_dbg_nodes[0].va, pearl_dbg_nodes[3].va);
	return 0;
}


/* ===== PEARL-CCIF2ST (r146): CCIF2 window write-readback self test =====
 * Purpose: distinguish the two possible meanings of CCIF2 registers "all 0":
 *   A) the window is alive and the SCP firmware really ignores the IPI
 *      -> the pattern reads back
 *   B) the window is gated/powered off, so reading 0 means nothing
 *      -> the pattern reads back 0
 * Also write the real key recorded by qqcandy (the SCP firmware checks it at
 * 0x6023C100) and read it back immediately, verifying "whether the key lands".
 */
static void pearl_ccif2_selftest(const char *where)
{
	u32 i, pat = 0xCAFEBABEu, pat2 = 0x5A5AA5A5u, rb;

	if (!pearl_ccif2_ap_base)
		return;
	for (i = 0; i < 8; i++)
		pr_info("PEARL-CCIF2ST[%s] ap+%02x=%08x md+%02x=%08x\n", where,
			i * 4, readl(pearl_ccif2_ap_base + i * 4),
			i * 4, readl(pearl_ccif2_md_base + i * 4));

	writel(pat, pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	wmb();
	rb = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	pr_info("PEARL-CCIF2ST[%s] CHDATA0 wrote=%08x read=%08x %s\n", where,
		pat, rb, rb == pat ? "ROUNDTRIP-OK(window alive)" : "ROUNDTRIP-FAIL(gated?)");

	writel(pat2, pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	wmb();
	rb = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	pr_info("PEARL-CCIF2ST[%s] CHDATA4 wrote=%08x read=%08x %s\n", where,
		pat2, rb, rb == pat2 ? "OK" : "FAIL");

	writel(PEARL_SCP_SMEM_KEY_LO, pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	writel(PEARL_SCP_SMEM_KEY_HI, pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	wmb();
	pr_info("PEARL-CCIF2ST[%s] realkey lo=%08x hi=%08x -> readback lo=%08x hi=%08x\n",
		where, PEARL_SCP_SMEM_KEY_LO, PEARL_SCP_SMEM_KEY_HI,
		readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0),
		readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4));
}

static void pearl_ccif2_map(void)
{
	if (!pearl_ccif2_ap_base)
		pearl_ccif2_ap_base = ioremap(PEARL_CCIF2_AP_PA, 0x1000);
	if (!pearl_ccif2_md_base)
		pearl_ccif2_md_base = ioremap(PEARL_CCIF2_MD_PA, 0x1000);
}

/* r146: run the self test once after mapping (printed on every call, very small volume) */
void pearl_ccif2_selftest_once(const char *where)
{
	pearl_ccif2_map();
	pearl_ccif2_selftest("publish");
	pearl_ccif2_selftest(where);
}

/* ===================== PEARL-CCIF2OBS: CCIF2 register observation =====================
 * Problem: the AP has handed the CCIF2 SRAM key + IPI(CCCI_OP_SHM_INIT) to
 * the SCP (ret=0), but whether the SCP firmware really executed CCISM
 * (sending 0x119 to the MD) is unknowable -- the only way to see SCP firmware
 * logs, reading scp_dram via debugfs, hangs the whole device.
 * Plan: during the HS2 window the kernel periodically prints the CCIF2
 * registers on both sides to dmesg:
 *   AP view (0x1023C000): TCHNUM/BUSY non-zero = the SCP rang a doorbell (SCP->MD);
 *   MD view (0x1023D000): RCHNUM/ACK non-zero = the MD received the SCP's doorbell.
 *   All zero throughout = the SCP firmware never responded to the IPI (the
 *   problem is on the firmware side, not in the AP's delivery).
 * Also watch the persistence of the CHDATA(0x100) triple (whether anyone
 * rewrites the key the AP wrote).
 */
#define PEARL_CCIF2_OBS_TIMES	24
#define PEARL_CCIF2_OBS_MS	500


static void pearl_ccif2_obs_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccif2_obs_work, pearl_ccif2_obs_work_fn);
static unsigned int pearl_ccif2_obs_cnt;

static void pearl_ccif2_obs_work_fn(struct work_struct *work)
{
	void __iomem *a = pearl_ccif2_ap_base;
	void __iomem *m = pearl_ccif2_md_base;
	int md_state;

	if (pearl_ccif2_obs_cnt >= PEARL_CCIF2_OBS_TIMES)
		return;
	md_state = ccci_fsm_get_md_state_for_user(MD_SYS1);
	if (md_state != MD_STATE_BOOTING)
		return;		/* HS2 window over (READY/EXCEPTION/INVALID) */
	pearl_ccif2_obs_cnt++;
	if (a && m) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2OBS[%02u] md=BOOTING AP(CON=%08x BUSY=%08x START=%08x TCH=%08x RCH=%08x ACK=%08x) MD(CON=%08x BUSY=%08x START=%08x TCH=%08x RCH=%08x ACK=%08x)\n",
			pearl_ccif2_obs_cnt,
			readl(a), readl(a + 4), readl(a + 8),
			readl(a + 0xc), readl(a + 0x10), readl(a + 0x14),
			readl(m), readl(m + 4), readl(m + 8),
			readl(m + 0xc), readl(m + 0x10), readl(m + 0x14));
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2OBS[%02u] CHDATA AP=%08x %08x %08x MD=%08x %08x %08x\n",
			pearl_ccif2_obs_cnt,
			readl(a + PEARL_CCIF2_CHDATA),
			readl(a + PEARL_CCIF2_CHDATA + 4),
			readl(a + PEARL_CCIF2_CHDATA + 8),
			readl(m + PEARL_CCIF2_CHDATA),
			readl(m + PEARL_CCIF2_CHDATA + 4),
			readl(m + PEARL_CCIF2_CHDATA + 8));
		/* PEARL-MDSNAP140: snapshot the MD's 0xc0170000 memory while it is
		 * still alive; refresh on every tick and keep the last one (the
		 * scene before power-down). */
		if (pearl_dbg_nodes[4].va_dyn) {
			void __iomem *p = ioremap((phys_addr_t)pearl_md_snap_pa,
				PEARL_MD_SNAP_SZ);
			u32 nz = 0;
			int j;

			if (p) {
				memcpy_fromio(pearl_dbg_nodes[4].va_dyn, p,
					PEARL_MD_SNAP_SZ);
				/* count pages that are entirely non-zero: a powered-off
				 * 0xFF region yields a huge nz, while real code/data
				 * pages always contain a zero byte. */
				for (j = 0; j < PEARL_MD_SNAP_SZ; j += 4096)
					if (memchr(pearl_dbg_nodes[4].va_dyn + j,
						0, 4096) == NULL)
						nz++;
				iounmap(p);
				pearl_md_snap_nz = nz;
				pearl_md_snap_ms = jiffies_to_msecs(
					jiffies - pearl_scp_pub_first_jiffies);
				pearl_md_snap_done = (nz < 4096);
				if (pearl_md_snap_done)
					CCCI_NORMAL_LOG(MD_SYS1, FSM,
						"PEARL-MDSNAP140: live cap pa=0x%x ~%ums nonzero_pages=%u/8192\n",
						pearl_md_snap_pa,
						pearl_md_snap_ms, nz);
			} else {
				CCCI_ERROR_LOG(MD_SYS1, FSM,
					"PEARL-MDSNAP140: memremap 0x%x failed\n",
					pearl_md_snap_pa);
			}
		}
	} else {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2OBS[%02u] md=BOOTING ioremap failed a=%px m=%px\n",
			pearl_ccif2_obs_cnt, a, m);
	}
	schedule_delayed_work(&pearl_ccif2_obs_work,
		msecs_to_jiffies(PEARL_CCIF2_OBS_MS));
}

static void pearl_ccif2_obs_start(void)
{
	if (pearl_ccif2_obs_cnt)
		return;		/* already running or already ran */
	pearl_ccif2_map();
	/* PEARL-31 (r122): decouple the forensic nodes from force_init/publish --
	 * in r121, after force_init was gated the debugfs nodes were never
	 * created, so the CCISM region could not be read after a crash. */
	pearl_scp_dbgfs_init();
	scp_set_clk_cg(1);	/* infra-ccif2-ap/-md clocks; must be on before reading */
	schedule_delayed_work(&pearl_ccif2_obs_work, 0);
}

/* write {key_lo, key_hi, smem_phy} into both CCIF2 SRAM windows (AP + MD).
 * The vendor hit the same pitfall on the MD side (see "write smem info tail
 * to BOTH CCIF SRAM sides" in ccci_reset_ccif_hw), hence writing both.
 * Returns the key_lo read back from the AP window. */
static u32 pearl_scp_smem_publish(u32 smem_phy)
{
	/* r147: run the CCIF2 write-readback self test on the first two publishes (tells whether the window is alive) */
	{
		static int __st;
		if (__st < 2) {
			__st++;
			pearl_ccif2_selftest_once("publish");
		}
	}

	u32 rb_lo, rb_hi, rb_addr, rb_magic;
	u32 key_lo = pearl_scp_key_broken ? PEARL_SCP_SMEM_KEY_BAD :
					    PEARL_SCP_SMEM_KEY_LO;
	u32 key_hi = pearl_scp_key_broken ? PEARL_SCP_SMEM_KEY_BAD :
					    PEARL_SCP_SMEM_KEY_HI;
	/* PEARL-169: random key (normal-device behavior) */
	if (pearl169_randkey && !pearl_scp_key_broken) {
		key_lo = pearl169_key_lo();
		key_hi = pearl169_key_hi();
	}
	u32 md_lo = 0, md_hi = 0, md_addr = 0;

	/* PEARL-SCPMEM137: do the address override in one place, so every call
	 * site writes the same value (the IPI payload still uses the real phy;
	 * the two do not affect each other). */
	if (pearl_scp_smem_force)
		smem_phy = pearl_scp_smem_force;
	/* PEARL-SCPVIEW138: address-view conversion. The SCP firmware mapping
	 * table is ap 0x10000000 -> scp 0x60000000, so the address given to the
	 * SCP in CCIF2 SRAM must be the SCP view (AP addr + 0x50000000). */
	if (pearl_ccism_view)
		smem_phy += 0x50000000;
	if (!pearl_scp_pub_first_jiffies)
		pearl_scp_pub_first_jiffies = jiffies;
	pearl_scp_pub_calls++;
	if (pearl_scp_pub_calls == 1)
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPVIEW138: view=%d published_smem=0x%x\n",
			pearl_ccism_view, smem_phy);

	/* the CCIF2 infra-ccif2-ap/-md clocks. Without platform-device probe
	 * clk_ref may be NULL; scp_set_clk_cg prints and returns -1 then, and
	 * the flow is not blocked here. */
	scp_set_clk_cg(1);

	/* PEARL-SCPDBG-77: create the forensic exports along the way (idempotent) */
	pearl_scp_dbgfs_init();

	pearl_ccif2_map();
	if (!pearl_ccif2_ap_base) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2: ioremap 0x%lX failed\n",
			PEARL_CCIF2_AP_PA);
		return 0;
	}

	if (pearl_ccif2_md_base) {
		writel(key_lo,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x0);
		writel(key_hi,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x4);
		/* PEARL-151: write {ap, md, size} per layout, or the old three ap copies */
		writel(smem_phy,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x8);
		writel(pearl_ccism_layout ? pearl_ccism_md_base : smem_phy,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0xc);
		writel(pearl_ccism_layout ? pearl_ccism_size : smem_phy,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x10);
	}
	writel(key_lo,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	writel(key_hi,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	writel(smem_phy,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x8);
	writel(pearl_ccism_layout ? pearl_ccism_md_base : smem_phy,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0xc);
	writel(pearl_ccism_layout ? pearl_ccism_size : smem_phy,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x10);
		/* the SCP is another core; after writing, a full barrier is needed before the IPI */
	mb();

	rb_lo = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	rb_hi = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	rb_addr = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x8);
	rb_magic = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA +
			 PEARL_SCP_SWEEP_LO);
	if (pearl_ccif2_md_base) {
		md_lo = readl(pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x0);
		md_hi = readl(pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x4);
		md_addr = readl(pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x8);
	}
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-CCIF2: publish smem=0x%x -> ap@%px rb key=0x%08X%08X addr=0x%x\n",
		smem_phy, pearl_ccif2_ap_base, rb_hi, rb_lo, rb_addr);
	/* PEARL-SCPMEM137/SCPKEY137: on the first 4 publishes and in the broken-key
	 * phase, log the full read-back, to confirm what is really in the two views
	 * and whether the magic actually landed. */
	if (pearl_scp_pub_calls <= 4)
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-151: layout=%d CHDATA(a=%08x m=%08x s=%08x) ap=0x%x md=0x%x size=0x%x\n",
			pearl_ccism_layout, rb_lo, rb_hi, rb_addr,
			smem_phy, pearl_ccism_md_base, pearl_ccism_size);
	if (pearl_scp_pub_calls <= 4 || pearl_scp_key_broken)
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPMEM137[%u]: broken=%d ap=%08x/%08x/%08x md=%08x/%08x/%08x magic@%x=0x%08x forced=0x%x\n",
			pearl_scp_pub_calls, pearl_scp_key_broken,
			rb_lo, rb_hi, rb_addr, md_lo, md_hi, md_addr,
			PEARL_SCP_SWEEP_LO, rb_magic, smem_phy);
	if (rb_hi != key_hi || rb_lo != key_lo)
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2: CCIF2 SRAM write did NOT stick (key=0x%08X%08X)\n",
			rb_hi, rb_lo);
	return rb_lo;
}

/* PEARL-30: proactively push the CCISM shared memory to the SCP, without waiting for the MD's 0x11A.
 *
 * The vendor does memset(smem) + IPI(CCCI_OP_SHM_INIT) only after the MD
 * replies 0x11A; but measured, the MD never replied 0x11A at all (the 0x11A
 * handler registered fine and was never called once), so the SCP never gets
 * the CCISM smem address -> never RBREADY -> 0x11B never sent -> the MD's
 * ccismcore gets stuck in GPD submission -> 40s timeout -> assert at
 * ccismcore_ccci.c:2004. This entry lets the chain skip "waiting for 0x11A"
 * and have the AP push it along. */
int pearl_ccism_force_init(void)
{
	struct ccci_smem_region *ccism_scp;
	u32 phy;
	int ret;

	ccism_scp = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
	if (!ccism_scp || !ccism_scp->base_ap_view_vir) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-CCISM force_init: ccism_scp(%px) not ready\n",
			ccism_scp);
		return -1;
	}

	memset_io(ccism_scp->base_ap_view_vir, 0, ccism_scp->size);
	/* PEARL-162: fill "self-numbered offset" markers per the switch */
	if (pearl_ccism_mark) {
		u32 lim = (pearl_ccism_mark == 2) ? 0x100 : ccism_scp->size;
		u32 o;

		for (o = 0; o + 4 <= lim; o += 4)
			writel(0xa5a50000 | (o & 0xffff),
			       ccism_scp->base_ap_view_vir + o);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-162: CCISM_SCP marked mode=%u lim=0x%x\n",
			pearl_ccism_mark, lim);
	}
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-CCISM force_init: memset va=%px sz=0x%x pa=0x%llx scp_state=%d\n",
		ccism_scp->base_ap_view_vir, ccism_scp->size,
		(unsigned long long)ccism_scp->base_ap_view_phy,
		atomic_read(&scp_state));

	phy = (u32)ccism_scp->base_ap_view_phy;
	/* PEARL-CCIF2: the SCP reads key+addr from CCIF2 SRAM, not from the IPI
	 * payload; these two words must be in place before the IPI, or the SCP
	 * prints no support smem ! */
	pearl_scp_smem_publish(phy);
	/* PEARL-SCPVIEW138: the IPI payload also uses the SCP view (consistent with the CCIF2 publish) */
	{
		u32 phy_v = phy + 0x50000000;
		u64 phy64 = (u64)phy;
		u64 phy64v = (u64)phy_v;
		u32 trip[3] = { phy, pearl_ccism_md_base, pearl_ccism_size };

		switch (pearl_ccism_ipi_mode) {
		case 0:
			/* PEARL-172: use the adjustable base (0x8e02c000 on the normal device) */
		phy = pearl172_pub_base;
		ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &phy);
			break;
		case 1:
			ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &phy_v);
			break;
		case 2:
			ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &phy64);
			break;
		case 3:
			ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &phy64v);
			break;
		default:
			ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, trip);
			break;
		}
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-162: IPI SHM_INIT mode=%u ap=0x%x scp=0x%x ret=%d\n",
			pearl_ccism_ipi_mode, phy, phy_v, ret);
	}
	return ret;
}
EXPORT_SYMBOL(pearl_ccism_force_init);

int fsm_ccism_init_ack_handler(int md_id, int data)
{
#ifdef FEATURE_SCP_CCCI_SUPPORT
	struct ccci_smem_region *ccism_scp;

	/* PEARL-29: this line is the golden evidence that "the MD really
	 * replied 0x11A". Earlier rounds could never distinguish "the MD did not
	 * receive 0x119" from "received but did not reply"; with this, injection
	 * experiment results become decidable. */
	CCCI_NORMAL_LOG(md_id, FSM,
		"PEARL-CCISM: got CCISM_SHM_INIT_ACK(0x11A) data=0x%x\n", data);

	ccism_scp = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_CCISM_SCP);

	/* PEARL-25: there used to be no NULL check here; a misconfigured region
	 * crashed on a NULL pointer. Measured "md1 get scp-sys-md1-main failed"
	 * shows the resource really can be missing. */
	if (ccism_scp == NULL || ccism_scp->base_ap_view_vir == NULL) {
		CCCI_ERROR_LOG(md_id, FSM,
			"CCISM_SHM_INIT_ACK: ccism_scp(%px) not ready\n",
			ccism_scp);
		return 0;
	}
	memset_io(ccism_scp->base_ap_view_vir, 0, ccism_scp->size);
	/* PEARL-CCIF2: same as force_init, the key must land in CCIF2 SRAM first */
	pearl_scp_smem_publish((u32)ccism_scp->base_ap_view_phy);
	ccci_scp_ipi_send(md_id, CCCI_OP_SHM_INIT,
		&ccism_scp->base_ap_view_phy);
#endif
	return 0;
}

static int fsm_sim_type_handler(int md_id, int data)
{
	struct ccci_per_md *per_md_data = ccci_get_per_md_data(md_id);

	per_md_data->sim_type = data;
	return 0;
}

#ifdef CCCI_KMODULE_ENABLE
#ifdef FEATURE_SCP_CCCI_SUPPORT
#ifdef CCCI_SCP_DRIVER_BUILTIN
void fsm_scp_init0(void)
{
	enum MD_STATE_FOR_USER state =
		ccci_fsm_get_md_state_for_user(ccci_scp_ctl.md_id);
	mutex_init(&scp_ipi_tx_mutex);

	if (!init_work_done) {
		INIT_WORK(&scp_ipi_rx_work, ccci_scp_ipi_rx_work);
		init_work_done = 1;
	}
	init_waitqueue_head(&scp_ipi_rx_wq);
	ccci_skb_queue_init(&scp_ipi_rx_skb_list, 16, 16, 0);

	CCCI_NORMAL_LOG(-1, FSM, "register IPI\n");
	/* PEARL-170: register IPI_IN_APCCCI_0 immediately.
	 * On the normal device (yuechu) the SCP sends a CCCI IPI already at
	 * t≈3.13s ("IPI send 0/0x2"), while we deferred registration to
	 * scp_ready (t≈3.85s) => that message was lost. Now that the SCP-side
	 * conap/CCCI smem/key/READY are all fine, registering early is safe. */
	if (pearl170_early_reg && !pearl_scp_ipi_registered)
		pearl_scp_ipi_register_now("PEARL-170:early");

#if (MD_GENERATION >= 6297)
	/* PEARL-32 (r123): restore the vendor registration point -- register at
	 * SCP READY (apsync_event, ~3.6s), earlier than MD power-on (7.2s). The
	 * old note "this registration kills the modem" held during the injection
	 * era (registration -> pending IPI redelivery -> rx_work immediately
	 * sends 0x119/0x11B via the port and blocks the channel); r119 has shut
	 * down all channel injection (pearl_ccism_inject=0), so registration now
	 * only produces logs and safe state handling; restore the vendor timing. */
	pearl_scp_ipi_register_now("scp_ready");
#else
	if (scp_ipi_registration(IPI_APCCCI, ccci_scp_ipi_handler,
		"AP CCCI") != SCP_IPI_DONE)
		CCCI_ERROR_LOG(-1, FSM, "register IPI fail!\n");
#endif
	atomic_set(&scp_state, SCP_CCCI_STATE_BOOTING);

	/* PEARL-CCISMPUB (r129): hand the CCISM_SCP shared memory to the SCP as
	 * soon as it is ready. The vendor publishes only after the MD replies
	 * 0x11A; with the MD stuck in early boot the SCP would never get it ->
	 * no CCISM service on the SCP side -> no CCCI_OP_SCP_STATE -> AP/MD/SCP
	 * all wait forever.
	 */
	if (pearl_ccism_publish && !pearl_ccism_force_inited) {
		int __fi129 = pearl_ccism_force_init();

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-CCISMPUB: publish CCISM_SCP at %s ret=%d\n",
			"scp_init0", __fi129);
		if (__fi129 == 0)
			pearl_ccism_force_inited = 1;
		/* PEARL-SCPREPUB: periodic re-send during boot (30 x 500ms, covering the MD boot window) */
		if (!pearl_scp_repub_cnt)
			schedule_delayed_work(&pearl_scp_repub_work,
				msecs_to_jiffies(500));
	}

	if (state != MD_STATE_INVALID)
		ccci_scp_md_state_sync(state);
}

/* PEARL-28 stage 2: where IPI_IN_APCCCI_0 is really registered (idempotent).
 * Return value: >0 = registered this time; 0 = already registered earlier
 * (skipped); <0 = registration failed.
 */
int pearl_scp_ipi_register_now(const char *why)
{
	enum MD_STATE_FOR_USER state;
	int ret;

	if (scp_register_done) {
		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-28 register: already done, skip (%s)\n", why);
		return 0;
	}

#if (MD_GENERATION >= 6297)
	ret = mtk_ipi_register(&scp_ipidev, IPI_IN_APCCCI_0,
		(void *)ccci_scp_ipi_handler, NULL, &scp_ipi_rx_msg);
	if (ret != IPI_ACTION_DONE) {
		CCCI_ERROR_LOG(-1, FSM,
			"PEARL-28 register: mtk_ipi_register failed ret=%d (%s)\n",
			ret, why);
		return -1;
	}
#else
	if (scp_ipi_registration(IPI_APCCCI, ccci_scp_ipi_handler,
		"AP CCCI") != SCP_IPI_DONE) {
		CCCI_ERROR_LOG(-1, FSM,
			"PEARL-28 register: scp_ipi_registration failed (%s)\n",
			why);
		return -1;
	}
#endif
	scp_register_done = 1;
	pearl_scp_ipi_registered = 1;
	atomic_set(&scp_state, SCP_CCCI_STATE_BOOTING);

	/* PEARL-CCISMPUB (r129): hand the CCISM_SCP shared memory to the SCP as
	 * soon as it is ready. The vendor publishes only after the MD replies
	 * 0x11A; with the MD stuck in early boot the SCP would never get it ->
	 * no CCISM service on the SCP side -> no CCCI_OP_SCP_STATE -> AP/MD/SCP
	 * all wait forever.
	 */
	if (pearl_ccism_publish && !pearl_ccism_force_inited) {
		int __fi129 = pearl_ccism_force_init();

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-CCISMPUB: publish CCISM_SCP at %s ret=%d\n",
			why, __fi129);
		if (__fi129 == 0)
			pearl_ccism_force_inited = 1;
		/* PEARL-SCPREPUB: periodic re-send during boot (30 x 500ms, covering the MD boot window) */
		if (!pearl_scp_repub_cnt)
			schedule_delayed_work(&pearl_scp_repub_work,
				msecs_to_jiffies(500));
	}

	/* PEARL-SCPDT-80 (rev2): SCP RBREADY IPI is emitted during early SCP
	 * boot, long before this AP handler is registered (PEARL-28 defers
	 * registration to HS2/READY), so the RBREADY branch in
	 * ccci_scp_ipi_rx_work is never hit. Publish CCISM_SCP shared region to
	 * SCP the instant the SCP IPI channel comes up, so MD can complete its
	 * 0x119 CCISM_SHM_INIT and reply 0x11A instead of stalling forever at
	 * ccismcore_ccci.c:1317. Force-once via pearl_ccism_force_inited. */
	if (!pearl_ccism_force_inited) {
		int __fi;

		/* PEARL-30: force_init (early publish + IPI) is a non-vendor-timing
		 * SCP-side action, so it is uniformly subject to the
		 * pearl_ccism_inject gate (off by default). */
		if (!pearl_ccism_inject) {
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-30: force_init skipped (inject=0)\n");
			return 0;
		}
		__fi = pearl_ccism_force_init();
		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-SCPDT-80: force_init (reg-now) ret=%d\n", __fi);
		if (__fi == 0)
			pearl_ccism_force_inited = 1;
	}
	/* PEARL-SCPRBREADY: the SCP's early RBREADY IPI was missed by PEARL-28
	 * (the handler was deferred until HS2/READY). force_init above already
	 * succeeded (ret=0), proving the SCP is up and handling IPIs. Here just
	 * raise scp_state to RBREADY to keep the state consistent; 0x11B (DONE)
	 * is sent by pearl_ccism_11b_work_fn over the SRAM channel, no longer
	 * re-sent via the port here where it would be dropped. */
	/* PEARL-168: after the SCP is ready, register conap receive and
	 * (optionally) send INIT */
	if (pearl168_conap_auto && !pearl168_registered) {
		pearl168_registered = 1;
		pearl168_conap_register();
		pearl168_conap_send(PEARL168_MSG_INIT, pearl168_shm_addr,
				    pearl168_shm_size);
	}
	if (pearl_ccism_force_inited) {
		if (atomic_read(&scp_state) != SCP_CCCI_STATE_RBREADY) {
			atomic_set(&scp_state, SCP_CCCI_STATE_RBREADY);
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPRBREADY: scp_state BOOTING->RBREADY (SCP alive)\n");
		}
		/* 0x11B (DONE) is now sent by pearl_ccism_11b_work_fn over the SRAM
		 * channel (the same path as 0x119, which the MD really consumes
		 * during HS2) after 0x119; re-sending it here via the port (queue0)
		 * would arrive out of order before 0x119, and queue0 is dropped by
		 * the MD during HS2 (proven in #45/r116c). */
	}
	state = ccci_fsm_get_md_state_for_user(ccci_scp_ctl.md_id);
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-28 register: IPI_IN_APCCCI_0 registered OK (%s), md_state=%d\n",
		why, state);
	if (state != MD_STATE_INVALID)
		ccci_scp_md_state_sync(state);
	return 1;
}

void pearl_scp_ipi_register_info(int *registered, int *auto_registered,
				int *early_flag)
{
	if (registered)
		*registered = (int)pearl_scp_ipi_registered;
	if (auto_registered)
		*auto_registered = scp_register_auto_path;
#ifdef PEARL28_EARLY_IPI_REG
	if (early_flag)
		*early_flag = 1;
#else
	if (early_flag)
		*early_flag = 0;
#endif
}

static int apsync_event(struct notifier_block *this,
	unsigned long event, void *ptr)
{
	switch (event) {
	case SCP_EVENT_READY:
		fsm_scp_init0();
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block apsync_notifier = {
	.notifier_call = apsync_event,
};
#endif	/* CCCI_SCP_DRIVER_BUILTIN */
#endif
#endif
int fsm_scp_init(struct ccci_fsm_scp *scp_ctl)
{
#ifndef CCCI_KMODULE_ENABLE
	struct ccci_fsm_ctl *ctl =
		container_of(scp_ctl, struct ccci_fsm_ctl, scp_ctl);
#endif
	int ret = 0;

#ifdef FEATURE_SCP_CCCI_SUPPORT
#ifdef CCCI_SCP_DRIVER_BUILTIN
	scp_A_register_notify(&apsync_notifier);
#else
	/* PEARL-25: scp_A_register_notify is exported by scp.ko and built-in CCCI
	 * cannot reference it. Its only job is to hook SCP_EVENT_READY to
	 * fsm_scp_init0() (IPI registration), and with scp.ko not loaded that
	 * event never comes. What really decides HS2 is the two
	 * register_ccci_sys_call_back() calls below, which run as usual. */
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-27 gate-off: skip scp_A_register_notify (SCP builtin, glue isolated)\n");
#endif
#endif
#ifndef CCCI_KMODULE_ENABLE
	scp_ctl->md_id = ctl->md_id;
#endif
#ifdef FEATURE_SCP_CCCI_SUPPORT
	INIT_WORK(&scp_ctl->scp_md_state_sync_work,
		ccci_scp_md_state_sync_work);
	register_ccci_sys_call_back(scp_ctl->md_id, CCISM_SHM_INIT_ACK,
		fsm_ccism_init_ack_handler);
#endif

	register_ccci_sys_call_back(scp_ctl->md_id, MD_SIM_TYPE,
		fsm_sim_type_handler);

	/* PEARL-30 (r121): autoreg=3 -> registration starts at builtin start
	 * (~2.2s); scp_ipidev is ready at 1.697s and retries cover the rest.
	 * Far ahead of MD power-on (7.2s). */
	if (pearl_scp_ipi_autoreg == 3 && scp_ctl->md_id == MD_SYS1)
		schedule_delayed_work(&pearl_scp_ipi_early_work, 0);

	return ret;
}

#ifdef CCCI_KMODULE_ENABLE
/* PEARL-25: this tree folds the SCP glue into the built-in ccci_md_all
 * instead of using the vendor "mediatek,ccci_md_scp" platform device (this
 * machine's DTS has no such node, and scp.ko hangs the moment it is
 * insmod'ed). So ccci_fsm_init() calls this entry directly to install
 * fsm_scp_init()'s two register_ccci_sys_call_back() hooks -- exactly what
 * the modem's HS2 stage is missing (CCISM_SHM_INIT_ACK / MD_SIM_TYPE).
 */
void ccci_fsm_scp_builtin_start(void)
{
	int ret;

	if (ccci_scp_ctl.md_id != MD_SYS1)
		return;

	ret = fsm_scp_init(&ccci_scp_ctl);
	CCCI_NORMAL_LOG(-1, FSM,
		"PEARL-25 %s: fsm_scp_init ret=%d md_id=%d sync=%ps\n",
		__func__, ret, ccci_scp_ctl.md_id,
		(void *)ccci_scp_ctl.md_state_sync);
	ccci_fsm_scp_register(ccci_scp_ctl.md_id, &ccci_scp_ctl);
}
#endif

static int ccif_scp_clk_init(struct device *dev)
{
	int idx;

	for (idx = 0; idx < ARRAY_SIZE(scp_clk_table); idx++) {
		scp_clk_table[idx].clk_ref = devm_clk_get(dev,
			scp_clk_table[idx].clk_name);
		if (IS_ERR(scp_clk_table[idx].clk_ref)) {
			CCCI_ERROR_LOG(-1, FSM,
				"%s: scp get %s failed\n",
				__func__, scp_clk_table[idx].clk_name);
			scp_clk_table[idx].clk_ref = NULL;
			return -1;
		}
	}

	return 0;
}

#ifdef CCCI_KMODULE_ENABLE
#ifdef FEATURE_SCP_CCCI_SUPPORT
static int ccci_scp_probe(struct platform_device *pdev)
{
	int ret;

	ret = ccif_scp_clk_init(&pdev->dev);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, FSM, "ccif scp clk init fail");
		return ret;
	}

	ret = fsm_scp_init(&ccci_scp_ctl);
	if (ret < 0) {
		CCCI_ERROR_LOG(-1, FSM, "ccci get scp info fail");
		return ret;
	}
	ccci_fsm_scp_register(0, &ccci_scp_ctl);
	return 0;
}


static const struct of_device_id ccci_scp_of_ids[] = {
	{.compatible = "mediatek,ccci_md_scp"},
	{}
};

static struct platform_driver ccci_scp_driver = {

	.driver = {
		.name = "ccci_md_scp",
		.of_match_table = ccci_scp_of_ids,
	},

	.probe = ccci_scp_probe,
};

static int __init ccci_scp_init(void)
{
	int ret;

	CCCI_NORMAL_LOG(-1, FSM, "ccci scp driver init start\n");

	ret = platform_driver_register(&ccci_scp_driver);
	if (ret) {
		CCCI_ERROR_LOG(-1, FSM, "ccci scp driver init fail %d", ret);
		return ret;
	}
	CCCI_NORMAL_LOG(-1, FSM, "ccci scp driver init end\n");
	return 0;
}

module_init(ccci_scp_init);
#endif
MODULE_AUTHOR("ccci");
MODULE_DESCRIPTION("ccci scp driver");
MODULE_LICENSE("GPL");

#endif
