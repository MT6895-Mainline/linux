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
/* PEARL-SCPDBG-77: debugfs 导出 SCP DRAM / CCISM smem，用于取证 */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/err.h>

#include "ccci_config.h"
#include "ccci_common_config.h"
#include "ccci_fsm_internal.h"
#include "md_sys1_platform.h"
#include "modem_sys.h"  /* PEARL-34: ccci_md_get_modem_by_id */

/* PEARL-25: 本树 SCP 驱动 (CONFIG_MTK_TINYSYS_SCP_SUPPORT) 是独立模块
 * scp.ko，而 CCCI 是内建 (CONFIG_MTK_CCCI_MAINLINE=y)。内建对象不能引用
 * 只由模块导出的符号 (scp_A_register_notify / scp_ipidev)，否则 vmlinux
 * 链接阶段报 undefined reference。只有 SCP 也内建时才允许碰这两个符号。
 */
#if defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT) && \
	!defined(CONFIG_MTK_TINYSYS_SCP_SUPPORT_MODULE)
#define CCCI_SCP_DRIVER_BUILTIN
#endif

/* PEARL-28 阶段 1：★门恢复打开★
 * 第 27 轮那行 `#undef CCCI_SCP_DRIVER_BUILTIN` 已删除（见 WORKLOG §3.28）。
 * 现在与 #85 完全同级：scp_A_register_notify → SCP READY → apsync_event()
 * → fsm_scp_init0() 全链路都会跑。唯一的差别被挪到 fsm_scp_init0() 里那一次
 * mtk_ipi_register()：本轮把它换成打印。判据见脚本头注释。
 */

#ifdef FEATURE_SCP_CCCI_SUPPORT
#include "scp_ipi.h"

#ifdef CCCI_KMODULE_ENABLE
void ccci_scp_md_state_sync(int md_state);

/* PEARL-28 阶段 2：把"注册 IPI_IN_APCCCI_0"这件事从"SCP READY 时刻"挪出来，
 * 由 sysfs 手动触发（或在 READY 之后自动触发）。声明放这里是因为
 * modem_sys1.c（sysfs 那一侧）要调它，而它的定义在 fsm_scp_init0 附近。
 */
int pearl_scp_ipi_register_now(const char *why);
/* PEARL-CCISM: SRAM 通道控制消息发送（定义在 ccci_hif_ccif.c） */
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


/* PEARL-25: 这里原来又定义了一份 ccci_debug_enable。原厂 ccci_fsm_scp.o
 * 是独立模块所以不冲突；本树把它并进内建的 ccci_md_all 后，与
 * ccci_core.c:37 的同名定义在 vmlinux.o 链接时撞成
 * "duplicate symbol: ccci_debug_enable"。ccci_debug.h 已有 extern 声明，
 * 直接用 ccci_core.c 那一份。 */
#endif

static atomic_t scp_state = ATOMIC_INIT(SCP_CCCI_STATE_INVALID);

/* PEARL-28：IPI_IN_APCCCI_0 注册的**推迟**实现。
 *
 * 阶段 1 已证实"这一次注册"就是打死基带的肇事者（见 WORKLOG §3.28），
 * 阶段 2 的做法是把它从 fsm_scp_init0()（SCP READY，约 4.15s，压在基带
 * HS1 bring-up 窗口上）挪到一个**由我们选择**的时刻：
 *   * 手动：echo 1 > /sys/kernel/ccci/mdsys1/scp_ipi_register
 *   * 自动：pearl_scp_ipi_autoreg=1 时，md_state 同步到 READY 之后再注册
 * 幂等：scp_register_done 单调置 1，重复触发只打印。
 */
unsigned int pearl_scp_ipi_autoreg =
#ifdef PEARL28_AUTOREG_HS2
	2;   /* PEARL-28 阶段 2c：本镜像编译期就把自动注册打开（HS2 之后），
	      * 理由：模块参数不持久，重启回到 0，而设备上 SSH 要 ~12.5s 才通，
	      * 4.1~7.9s 那个窗口没有用户态办法打进去。 */
#else
	0;   /* 默认 0：不自动注册（非 static：modem_sys1.c 的 sysfs 要读） */
#endif
static unsigned int scp_register_done;
/* PEARL-CCISM: 0x119 已在 HS2 阶段发出的幂等标志（READY 分支的发送仍保留） */
static unsigned int pearl_ccism_init_sent;
static int scp_register_auto_path;
unsigned int pearl_scp_ipi_registered;
module_param(pearl_scp_ipi_registered, uint, 0444);
MODULE_PARM_DESC(pearl_scp_ipi_registered,
	"PEARL-28: 1 when IPI_IN_APCCCI_0 has really been registered");
/* PEARL-28 阶段 2b：自动注册的触发点。
 *   0 = 不自动（默认）
 *   1 = 到 READY（md_state 4）后再注册
 *   2 = 到 HS2（md_state 3，即 HS1 已过）后再注册
 * 取值 2 是实验出来的关键：注册落在 HS1 **之前**会 0.13s 打死基带，
 * 落在 HS1 **之后**则无害（见 WORKLOG §3.28），而设备上 SSH 要 ~12.5s 才通，
 * 4.1~7.9s 这个窗口只能由内核自己打。
 */
module_param(pearl_scp_ipi_autoreg, uint, 0644);
MODULE_PARM_DESC(pearl_scp_ipi_autoreg,
	"PEARL-28: 0=off 1=register after md READY 2=register after md HS2 "
	"3=register ASAP after SCP subsystem ready (PEARL-30 r121)");

/* PEARL-28 阶段 2c：自动注册的内核侧延迟（毫秒），由 pearl_scp_ipi_autoreg 触发点
 * 之后开始计时。存在的理由：模块参数**不持久**（重启回到 0），而设备上 SSH 要到
 * ~12.5s 才通，4.1~7.9s 这个窗口没有任何用户态办法打进去 —— 只能靠它。
 */
unsigned int pearl_scp_ipi_reg_delay_ms =
#ifdef PEARL28_AUTOREG_HS2
	PEARL28_AUTOREG_HS2;   /* 延迟毫秒数（编译期默认，便于落点实验） */
#else
	0;
#endif
module_param(pearl_scp_ipi_reg_delay_ms, uint, 0644);

/* PEARL-CCISMOFF (r119): 启动期注入总闸，默认 0 = 原厂行为。
 * 原厂 0x119/0x11B 只在 READY 之后经 queue0 发送；启动期注入的 0x119 会把
 * ch0/ch15 两条 AP->MD 控制通道永久堵死（MD 永不消费 -> MD ccismcore 等
 * 应答超时，ccismcore_ccci.c:1317 断言）。置 1 恢复旧注入行为（对照实验）。 */
static unsigned int pearl_ccism_inject;
module_param(pearl_ccism_inject, uint, 0644);

/* PEARL-CCISMPUB (r129): 1 = SCP 一就绪就把 CCISM_SCP 共享内存发布给 SCP
 * （memset + CCIF2 SRAM key + IPI CCCI_OP_SHM_INIT），并恢复原厂在
 * SCP RBREADY 时无条件发 0x11B 的行为。0 = 回到原厂"等 MD 0x11A"时序。 */
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

/* PEARL-30 (r121): autoreg=3 —— SCP 子系统就绪后**立即**注册（带重试）。
 * 目的：让 AP<->SCP CCCI 握手在 SCP 启动(~1.7s)就完成，恢复原厂时序，
 * 避免整个握手撞进 MD 启动窗口（对撞 -> MD ccismcore 5.2s 断言）。 */
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
	/* PEARL-25: scp_ipidev 由 scp.ko 提供，内建 CCCI 不能引用它；
	 * 而且 scp.ko 没加载时这条 IPI 也没人应答（scp_state 恒为 INVALID，
	 * 上面那句判断早已 return）。语义与原来一致：MD 未就绪。 */
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
			/* PEARL-25: 没走平台设备 probe 时 clk 没被 devm_clk_get
			 * 填过，clk_prepare_enable(NULL) 会直接空指针崩。 */
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

/* PEARL-FSMPFIX-75: CCISM 运行时可切 —— 见 port_rpc.c 的 pearl_fs.cfg ccism_* 项。
 * 这些声明必须放在使用点之前（clang 报 implicit-function-declaration）。 */
extern unsigned int pearl_ccism_cfg_auto(void);
extern unsigned int pearl_ccism_cfg_ms(void);
extern unsigned int pearl_ccism_cfg_11b(void);
extern unsigned int pearl_ccism_cfg_11b_ms(void);
extern unsigned int pearl_ccism_cfg_gen(void);
extern int pearl_ccism_force_init(void); /* PEARL-SCPDT-80 fwd */

static unsigned int pearl_ccism_11b_sent;
/* PEARL-CCISMPUB130: SCP 是否已报 RBREADY（它可能在 MD 上电前就报） */
static unsigned int pearl_scp_rbready;
static void pearl_ccism_11b_port_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_11b_port_work, pearl_ccism_11b_port_fn);
/* PEARL-SCPREPUB (r134): SCP 自打印 shm_addr=[0]（它读到的地址是 0），
 * 而 AP 侧写入并回读成功 -> 写入在 SCP 读之前不可见/被清。SCP 何时读不确定，
 * 所以周期性重发 CCIF2 SRAM 的 key+addr，并把回读值打进日志。 */
static void pearl_scp_repub_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_scp_repub_work, pearl_scp_repub_fn);
/* PEARL-SCPREPUB: 定义在后面，这里先声明（否则隐式声明类型冲突） */
static u32 pearl_scp_smem_publish(u32 smem_phy);
static unsigned int pearl_scp_repub_cnt;
static int pearl_ccism_force_inited; /* PEARL-SCPDT-80 */
static void pearl_ccism_11b_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_11b_work, pearl_ccism_11b_work_fn);

static void pearl_ccism_send_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_ccism_send_work, pearl_ccism_send_work_fn);
/* PEARL-CCIF2OBS: 前向声明（观测器实体在 CCIF2 段，HS2 分支在前面就要用） */
static void pearl_ccif2_obs_start(void);

/* PEARL-CCISM: 延迟发送 work（+2s）。
 * #46 教训：SRAM 0x119 在 state 2→3 瞬间发出会被 ~0.2s 后的 runtime data
 * 覆盖 up_header，且 MD 7.96s 才进入"等 SRAM 消息"窗口。+2s 落在窗口内。 */
static unsigned int pearl_ccism_planned;

static void pearl_ccism_send_work_fn(struct work_struct *work)
{
	int ret;

	/* PEARL-CCISMOFF: 默认停发（见 pearl_ccism_inject 注释）。 */
	if (!pearl_ccism_inject)
		return;
	if (pearl_ccism_init_sent)
		return;
	/* PEARL-FSMPFIX-75 第一步（planner）：HS2 分支只排 +500ms，由这里
	 * （≈HS2+0.5s，cfg 必已可读）读 cfg，再排真正的发送。默认 == 旧行为。 */
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

	/* PEARL-CCISMOFF: 默认停发。 */
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

	if (pearl_scp_repub_cnt >= 30)
		return;
	pearl_scp_repub_cnt++;
	r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
	if (r)
		phy = (u32)r->base_ap_view_phy;
	if (phy) {
		u32 rb = pearl_scp_smem_publish(phy);

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-SCPREPUB[%u]: republish phy=0x%x rb=0x%08x\n",
			pearl_scp_repub_cnt, phy, rb);
	}
	if (pearl_scp_repub_cnt < 30)
		schedule_delayed_work(&pearl_scp_repub_work,
			msecs_to_jiffies(500));
}

/* PEARL-CCISMPUB130: 延迟到 MD 进入 HS2 后再发 0x11B（幂等） */
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
		/* PEARL-28 阶段 2b：READY 意味着 HS1/HS2 都已经走完，
		 * 注册放到这里绝不抢 bring-up 窗口。 */
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
		/* PEARL-SCPMDSTATE (r131): 把 MD 的启动状态同步给 SCP。
		 * 原厂只在 INVALID/READY/EXCEPTION 同步，本机 SCP 在 MD 上电前
		 * (~3.6s) 就 RBREADY，此时被告知"MD INVALID"，之后 MD 0->2->3
		 * 的变化 SCP 一无所知，自然无法与 MD 建立 CCISM/CCIF2。
		 */
		if (scp_ctl->md_id == MD_SYS1) {
			enum MD_STATE_FOR_USER st131 =
				ccci_fsm_get_md_state_for_user(scp_ctl->md_id);

			ccci_scp_ipi_send(scp_ctl->md_id,
				CCCI_OP_MD_STATE, &st131);
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPMDSTATE: md_state sync to SCP = %d (md_state=%d)\n",
				st131, ctl->md_state);
		}
		/* PEARL-28 阶段 2b：HS1 已过（2→3 就是收到 HS1）。取 2 时在这里注册，
		 * 用来判定"HS1 之前 vs 之后"是不是真正的分界。 */
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
		/* PEARL-CCISMPUB130: SCP 已 RBREADY 时，在 MD 真正上电后的 HS2
		 * 窗口补发原厂那一步（0x11B INIT_DONE）。此时门控已放行
		 * （port_proxy.c 的 PEARL-CCISM 例外）。 */
		if (scp_ctl->md_id == MD_SYS1 && pearl_scp_rbready &&
		    !pearl_ccism_11b_sent) {
			int r130 = ccci_port_send_msg_to_md(MD_SYS1,
				CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);

			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-CCISMPUB130: HS2 窗口发 0x119 ret=%d\n", r130);
			/* 800ms 后再补 0x11B（给 MD 回 0x11A 的机会） */
			schedule_delayed_work(&pearl_ccism_11b_port_work,
				msecs_to_jiffies(800));
		}
		/* PEARL-CCISM: 在 HS2 等待阶段主动发 CCISM_SHM_INIT(0x119)。
		 * 原厂只在 READY（HS2 之后）经 port_proxy 门控发出，但实证：
		 * 基带 HS2 阶段卡在 ccismcore:2004（等 AP 消息超时 40s）、
		 * RPC/FS 全部跑完后静默、ccismc_polling_submit_one_gpd 的 GPD
		 * 提交完不成 —— 指向 0x119→0x11A→(memset smem + IPI SCP) 这条
		 * CCISM 初始化握手在 HS2 之前缺失，AP 等 READY / MD 等 0x119
		 * 互相死等。此时 IPI 已由上面 autoreg==2 分支注册完毕。 */
		/* PEARL-FSMPFIX-75: 只排 planner(+500ms)，真正的延迟/开关/是否补
		 * 0x11B 由 planner 读 cfg 决定。默认 (auto=1,ms=2000,11b=0)==旧行为。 */
		if (scp_ctl->md_id == MD_SYS1 && !pearl_ccism_init_sent)
			schedule_delayed_work(&pearl_ccism_send_work,
				msecs_to_jiffies(500));
		/* PEARL-CCIF2OBS: HS2 窗口内定时打印 CCIF2 两侧寄存器到 dmesg，
		 * 回答 "IPI 已 ret=0，SCP 固件到底动没动"。 */
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
					/* PEARL-CCISMOFF: 启动期这条 0x119 会把
					 * ch0 永久堵死（r118 铁证 7.799s），
					 * 默认不发。 */
					if (pearl_ccism_publish) {
						ccci_port_send_msg_to_md(MD_SYS1,
						CCCI_SYSTEM_TX, CCISM_SHM_INIT, 0, 1);
						ccci_port_send_msg_to_md(MD_SYS3,
						CCCI_CONTROL_TX,
						C2K_CCISM_SHM_INIT, 0, 1);
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

					/* PEARL-30: 同 register_now，纳入闸门 */
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
				/* PEARL-CCISMPUB (r129/r130): 原厂此处无条件发 0x11B，
				 * 但那是以"MD 已 READY"为前提；本机 SCP 在 MD 上电前就
				 * RBREADY（3.5s），此时发必然丢 -> 仅在 MD 已 READY 时发，
				 * 否则交给 MD 的 HS2 窗口补发（见 md_state_sync_work）。 */
				if (pearl_ccism_publish &&
				    ccci_fsm_get_md_state(ipi_msg_ptr->md_id) == READY) {
					switch (ipi_msg_ptr->md_id) {
					case MD_SYS1:
						ccci_port_send_msg_to_md(MD_SYS1,
						CCCI_SYSTEM_TX,
						CCISM_SHM_INIT_DONE, 0, 1);
						break;
					case MD_SYS3:
						ccci_port_send_msg_to_md(MD_SYS3,
						CCCI_CONTROL_TX,
						C2K_CCISM_SHM_INIT_DONE, 0, 1);
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

/* ===================== PEARL-CCIF2: SCP smem 握手字 =====================
 *
 * 从 SCP 固件 scp_a.img(code.bin) 反汇编得到的硬事实：
 *   ccci_ipi_task 的 op==2(CCCI_OP_SHM_INIT) 分支：
 *     dc5a: lui a0,0x6023c ; addi a0,a0,0x104 ; lw s1,(a0)   <- key_hi
 *     dc6e: lui a0,0x6023c ; addi a0,a0,0x100 ; lw s3,(a0)   <- key_lo
 *     dc8c: 期望 key_hi == 0x5343505F , key_lo == 0x534D454D
 *     dca8: bnez -> 0xdd16 = "[CCCI%d/SCP][%s] no support smem !"
 *     dcaa: lui a0,0x6023c ; addi a0,a0,0x108 ; lw a3,(a0)   <- smem 物理地址
 *
 * 0x6023C100 经 SCP 的 AP->SCP 转换表(0x12266, 表在 0x6C304 起, 1->6)换算
 * = AP 物理 0x1023C100 = ap_ccif2(0x1023C000) + APCCIF_CHDATA(0x100)
 * = CCIF2 的 SRAM。SCP 固件里 5 处 `lui 0x6023c` 全是 lw，从不写；
 * 魔数在 SCP/MD/LK 镜像里也不存在 => 只能 AP 运行期写。
 *
 * 原厂 ccci_scp 节点 reg = <0x1023c000 0x1000>, <0x1023d000 0x1000>
 *   （AP_CCIF2_BASE / MD_CCIF2_BASE），与本文件硬编码一致。
 */
#define PEARL_CCIF2_AP_PA	0x1023C000UL
#define PEARL_CCIF2_MD_PA	0x1023D000UL
#define PEARL_CCIF2_CHDATA	0x100
#define PEARL_SCP_SMEM_KEY_LO	0x534D454D	/* "MEMS" */
#define PEARL_SCP_SMEM_KEY_HI	0x5343505F	/* "_PCS" */

static void __iomem *pearl_ccif2_ap_base;
static void __iomem *pearl_ccif2_md_base;

/* ---------- PEARL-SCPDBG-77: debugfs 取证出口 ----------
 * 背景：AP 已经能把 smem 地址推给 SCP（key 写 CCIF2 SRAM + IPI CCCI_OP_SHM_INIT），
 * 但 modem 仍卡在 ccismcore_ccci.c:1317。要判断"到底是 SCP 没初始化那块 smem，
 * 还是 modem 在等别的东西"，唯一的办法是把内存本体读出来看。
 *
 *   /sys/kernel/debug/pearl_scp/scp_dram   SCP 自己的 DRAM/日志区（LK 预留）
 *   /sys/kernel/debug/pearl_scp/ccism_scp  CCISM_SCP 共享内存（AP<->SCP<->MD）
 *   /sys/kernel/debug/pearl_scp/ccism_mcu  CCISM_MCU 共享内存头部
 *   /sys/kernel/debug/pearl_scp/ccif2_sram CCIF2 SRAM（key 落点）
 *
 * 用法：echo init > /sys/kernel/ccci/mdsys1/ccism_send（这会顺带建好 debugfs）
 *       然后 cat 上述文件，本地再 strings / hexdump。
 */
/* PEARL-SCPSHARE (r132): AP<->SCP 共享预留区（LK 预留 mblock-25，6.8MB）。
 * SCP 的日志/共享数据落在这里；AP 视角可读。只暴露前 64KB 以控风险。 */
#define PEARL_SCP_SHARE_PA	0x8F000000UL
#define PEARL_SCP_SHARE_SZ	0x6a2000UL
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
	void *va_dyn;   /* PEARL-34b: memremap 读取缓冲 */
};

static struct dentry *pearl_dbg_dir;

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

		/* 快照是普通内存，pearl_dbg_read 的 memcpy_fromio 同样适用 */
		return pearl_l2sram_snap;
	}
	case PDBG_SMEM_RAW: {
		struct ccci_smem_region *r = ccci_md_get_smem_by_user_id(
			MD_SYS1, SMEM_USER_RAW_MDSS_DBG);

		return (r && r->base_ap_view_vir) ? r->base_ap_view_vir : NULL;
	}
	case PDBG_MDBANK: {
		/*
		 * PEARL-34c (r127): MD 运行期已解密的代码/数据区 dump。
		 * md1img 里代码段是密文（熵 7.2），运行期解密载入 DRAM。
		 * AP 物理地址 = 0xC0170000 起 48MB（stock DT modem_info_v2，
		 * 与我们 DT 一致）。注意 LK tag "md_bank0_base"=0xD0000000 是
		 * MD 视图地址，直接当 AP PA memremap 会挂死（r126 教训）。
		 */
		phys_addr_t pa = 0xC0170000ULL;
		void *p;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-34c: md_bank0 dump pa=%pap size=0x%zx\n",
			&pa, &n->size);
		p = memremap(pa, n->size, MEMREMAP_WB);
		if (p) {
			memcpy(n->va_dyn, p, n->size);
			memunmap(p);
			return n->va_dyn;
		}
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-34c: memremap failed\n");
		return NULL;
	}
	case PDBG_SCPSHARE: {
		/* PEARL-SCPSHARE (r132): 读 AP<->SCP 共享区（0x8f000000 起 64KB）。
		 * 用于取 SCP 自己的日志，判断它 CCISM 初始化停在哪一步。 */
		phys_addr_t pa = PEARL_SCP_SHARE_PA;
		void *p;

		if (!n->va_dyn)
			return NULL;
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPSHARE: memremap 0x%llx size=0x%zx\n",
			(unsigned long long)pa, n->size);
		p = memremap(pa, n->size, MEMREMAP_WB);
		if (!p) {
			CCCI_ERROR_LOG(MD_SYS1, FSM,
				"PEARL-SCPSHARE: memremap failed\n");
			return NULL;
		}
		memcpy(n->va_dyn, p, n->size);
		memunmap(p);
		return n->va_dyn;
	}
	default:
		return n->va;
	}
}

static ssize_t pearl_dbg_read(struct file *file, char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	/* PEARL-SCPDBG-77: debugfs 把 create_file 的 data 放在 inode->i_private，
	 * 不是 file->private_data（本树 full_proxy_open_regular 不会替我们搬），
	 * 用错会直接 oops（首版实测崩在 pearl_dbg_read+0x28）。 */
	struct pearl_dbg_node *n =
		(struct pearl_dbg_node *)file_inode(file)->i_private;
	void __iomem *va = pearl_dbg_va(n);
	void *kbuf;
	ssize_t ret;

	if (!va)
		return -ENODEV;
	/* PEARL-34b: md_bank0 的 va 是普通内存缓冲（va_dyn），不是 __iomem */
	if (n->kind == PDBG_MDBANK) {
		if (!n->va_dyn)
			return -ENODEV;
		return simple_read_from_buffer(ubuf, count, ppos,
			n->va_dyn, n->size);
	}
	/* PEARL-CCIF2OBS-78: 读 scp_dram(0xbfc00000, SCP 保护的 DRAM) 会触发
	 * DEVAPC 违规把整机硬挂死（r118 前实测两次一致：读一次 SSH 即失联）。
	 * 它只是取证出口，不是功能路径——默认拒绝读，别拿整台设备陪葬。
	 * ccif2_sram / ccism_scp / ccism_mcu 是安全 MMIO/共享内存，照常放行。 */
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
};

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
	/* PEARL-34b: md_bank0 的 32MB 读取缓冲（惰性内容，读时 memremap 填） */
	pearl_dbg_nodes[4].va_dyn = vzalloc(pearl_dbg_nodes[4].size);
	/* PEARL-SCPSHARE: 第 7 个节点的读取缓冲（惰性 memremap 填） */
	pearl_dbg_nodes[7].va_dyn = vzalloc(pearl_dbg_nodes[7].size);
	for (i = 0; i < ARRAY_SIZE(pearl_dbg_nodes); i++)
		debugfs_create_file(pearl_dbg_nodes[i].name, 0400,
				    pearl_dbg_dir, &pearl_dbg_nodes[i],
				    &pearl_dbg_fops);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-SCPDBG-77: debugfs pearl_scp/ ready (dram=%px ccif2=%px)\n",
		pearl_dbg_nodes[0].va, pearl_dbg_nodes[3].va);
	return 0;
}

static void pearl_ccif2_map(void)
{
	if (!pearl_ccif2_ap_base)
		pearl_ccif2_ap_base = ioremap(PEARL_CCIF2_AP_PA, 0x1000);
	if (!pearl_ccif2_md_base)
		pearl_ccif2_md_base = ioremap(PEARL_CCIF2_MD_PA, 0x1000);
}

/* ===================== PEARL-CCIF2OBS: CCIF2 寄存器观测 =====================
 * 问题：AP 已把 CCIF2 SRAM 钥匙 + IPI(CCCI_OP_SHM_INIT) 交给 SCP（ret=0），
 * 但 SCP 固件是否真的执行了 CCISM（向 MD 发 0x119）无从得知——唯一能看
 * SCP 固件日志的 scp_dram debugfs 读会挂死整机。
 * 方案：HS2 窗口内由内核定时把 CCIF2 两侧寄存器打进 dmesg：
 *   AP 视图(0x1023C000)：TCHNUM/BUSY 非 0 = SCP 发过门铃(SCP->MD)；
 *   MD 视图(0x1023D000)：RCHNUM/ACK 非 0 = MD 收到过 SCP 的门铃。
 *   全程全 0 = SCP 固件根本没响应 IPI（问题在固件侧，不在 AP 投递）。
 * 顺带看 CHDATA(0x100) 三元组持久性（AP 写的 key 有没有被谁改写）。
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
		return;		/* HS2 窗口结束（READY/EXCEPTION/INVALID） */
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
		return;		/* 已经在跑/跑过 */
	pearl_ccif2_map();
	/* PEARL-31 (r122): 取证节点与 force_init/publish 解耦——r121 里
	 * force_init 被闸后 debugfs 没建起来，崩后无法读 CCISM 区域。 */
	pearl_scp_dbgfs_init();
	scp_set_clk_cg(1);	/* infra-ccif2-ap/-md 时钟，读前必须开 */
	schedule_delayed_work(&pearl_ccif2_obs_work, 0);
}

/* 把 {key_lo, key_hi, smem_phy} 写进 CCIF2 SRAM 的两个窗口（AP + MD）。
 * 原厂在 MD 侧踩过同一个坑（见 ccci_reset_ccif_hw 里"write smem info tail
 * to BOTH CCIF SRAM sides"），所以两侧都写。返回从 AP 窗口读回的 key_lo。 */
static u32 pearl_scp_smem_publish(u32 smem_phy)
{
	u32 rb_lo, rb_hi, rb_addr;

	/* CCIF2 的 infra-ccif2-ap/-md 时钟。没走平台设备 probe 时 clk_ref
	 * 可能是 NULL，scp_set_clk_cg 会打印并返回 -1，这里不阻塞流程。 */
	scp_set_clk_cg(1);

	/* PEARL-SCPDBG-77: 顺手把取证出口建起来（幂等） */
	pearl_scp_dbgfs_init();

	pearl_ccif2_map();
	if (!pearl_ccif2_ap_base) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2: ioremap 0x%lX failed\n",
			PEARL_CCIF2_AP_PA);
		return 0;
	}

	if (pearl_ccif2_md_base) {
		writel(PEARL_SCP_SMEM_KEY_LO,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x0);
		writel(PEARL_SCP_SMEM_KEY_HI,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x4);
		writel(smem_phy,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x8);
	}
	writel(PEARL_SCP_SMEM_KEY_LO,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	writel(PEARL_SCP_SMEM_KEY_HI,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	writel(smem_phy,
		pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x8);
	/* SCP 是另一个核，写完要一次全屏障再发 IPI */
	mb();

	rb_lo = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x0);
	rb_hi = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x4);
	rb_addr = readl(pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + 0x8);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-CCIF2: publish smem=0x%x -> ap@%px rb key=0x%08X%08X addr=0x%x\n",
		smem_phy, pearl_ccif2_ap_base, rb_hi, rb_lo, rb_addr);
	if (rb_hi != PEARL_SCP_SMEM_KEY_HI || rb_lo != PEARL_SCP_SMEM_KEY_LO)
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-CCIF2: CCIF2 SRAM write did NOT stick (key=0x%08X%08X)\n",
			rb_hi, rb_lo);
	return rb_lo;
}

/* PEARL-30: 主动把 CCISM 共享内存推给 SCP，不等 MD 的 0x11A。
 *
 * 原厂是 MD 回 0x11A 才做 memset(smem) + IPI(CCCI_OP_SHM_INIT)；但实测
 * MD 从头到尾没回过 0x11A（0x11A 的 handler 注册成功、一次都没被调用），
 * 于是 SCP 永远拿不到 CCISM smem 地址 -> 永远不 RBREADY -> 0x11B 永远不发
 * -> MD 的 ccismcore 卡在 GPD 提交 -> 40s 超时 -> ccismcore_ccci.c:2004 断言。
 * 这个入口让链路跳过"等 0x11A"，由 AP 主动推一把。 */
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
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-CCISM force_init: memset va=%px sz=0x%x pa=0x%llx scp_state=%d\n",
		ccism_scp->base_ap_view_vir, ccism_scp->size,
		(unsigned long long)ccism_scp->base_ap_view_phy,
		atomic_read(&scp_state));

	phy = (u32)ccism_scp->base_ap_view_phy;
	/* PEARL-CCIF2: SCP 从 CCIF2 SRAM 读 key+addr，不是从 IPI 载荷；
	 * 必须在 IPI 之前把这两个字放好，否则 SCP 打 no support smem ! */
	pearl_scp_smem_publish(phy);
	ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &phy);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-CCISM force_init: IPI CCCI_OP_SHM_INIT pa=0x%x ret=%d\n",
		phy, ret);
	return ret;
}
EXPORT_SYMBOL(pearl_ccism_force_init);

int fsm_ccism_init_ack_handler(int md_id, int data)
{
#ifdef FEATURE_SCP_CCCI_SUPPORT
	struct ccci_smem_region *ccism_scp;

	/* PEARL-29: 这一行是"MD 真的回了 0x11A"的黄金证据。
	 * 前几轮一直无法区分"MD 没收到 0x119"和"收到了但没回"，
	 * 有了它，注入实验的结果就是可判定的。 */
	CCCI_NORMAL_LOG(md_id, FSM,
		"PEARL-CCISM: got CCISM_SHM_INIT_ACK(0x11A) data=0x%x\n", data);

	ccism_scp = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_CCISM_SCP);

	/* PEARL-25: 原来这里没有任何判空，region 没配好就是空指针崩。
	 * 实测 "md1 get scp-sys-md1-main failed" 说明相关资源确实可能缺。 */
	if (ccism_scp == NULL || ccism_scp->base_ap_view_vir == NULL) {
		CCCI_ERROR_LOG(md_id, FSM,
			"CCISM_SHM_INIT_ACK: ccism_scp(%px) not ready\n",
			ccism_scp);
		return 0;
	}
	memset_io(ccism_scp->base_ap_view_vir, 0, ccism_scp->size);
	/* PEARL-CCIF2: 同 force_init，key 必须先落到 CCIF2 SRAM */
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

#if (MD_GENERATION >= 6297)
	/* PEARL-32 (r123): 恢复原厂注册点 —— SCP READY(apsync_event, ~3.6s) 时
	 * 注册，早于 MD 上电(7.2s)。旧注记"这次注册会打死基带"成立于注入时代
	 * （注册 -> pending IPI 补投递 -> rx_work 立即经 port 发 0x119/0x11B
	 * 堵死通道）；r119 已闸死全部通道注入（pearl_ccism_inject=0），现在
	 * 注册只产生日志与安全的状态处理，恢复原厂时序。 */
	pearl_scp_ipi_register_now("scp_ready");
#else
	if (scp_ipi_registration(IPI_APCCCI, ccci_scp_ipi_handler,
		"AP CCCI") != SCP_IPI_DONE)
		CCCI_ERROR_LOG(-1, FSM, "register IPI fail!\n");
#endif
	atomic_set(&scp_state, SCP_CCCI_STATE_BOOTING);

	/* PEARL-CCISMPUB (r129): SCP 就绪即把 CCISM_SCP 共享内存交给它。
	 * 原厂只在 MD 回 0x11A 后发布；而 MD 卡在早期启动 -> SCP 永远拿不到
	 * -> SCP 侧 CCISM 服务不起 -> 不发 CCCI_OP_SCP_STATE -> AP/MD/SCP 三方死等。
	 */
	if (pearl_ccism_publish && !pearl_ccism_force_inited) {
		int __fi129 = pearl_ccism_force_init();

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-CCISMPUB: publish CCISM_SCP at %s ret=%d\n",
			"scp_init0", __fi129);
		if (__fi129 == 0)
			pearl_ccism_force_inited = 1;
		/* PEARL-SCPREPUB: 启动周期重发（30 x 500ms，覆盖 MD 启动窗口） */
		if (!pearl_scp_repub_cnt)
			schedule_delayed_work(&pearl_scp_repub_work,
				msecs_to_jiffies(500));
	}

	if (state != MD_STATE_INVALID)
		ccci_scp_md_state_sync(state);
}

/* PEARL-28 阶段 2：真正做 IPI_IN_APCCCI_0 注册的地方（幂等）。
 * 返回值：>0 = 本次注册成功；0 = 早就注册过（跳过）；<0 = 注册失败。
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

	/* PEARL-CCISMPUB (r129): SCP 就绪即把 CCISM_SCP 共享内存交给它。
	 * 原厂只在 MD 回 0x11A 后发布；而 MD 卡在早期启动 -> SCP 永远拿不到
	 * -> SCP 侧 CCISM 服务不起 -> 不发 CCCI_OP_SCP_STATE -> AP/MD/SCP 三方死等。
	 */
	if (pearl_ccism_publish && !pearl_ccism_force_inited) {
		int __fi129 = pearl_ccism_force_init();

		CCCI_NORMAL_LOG(-1, FSM,
			"PEARL-CCISMPUB: publish CCISM_SCP at %s ret=%d\n",
			why, __fi129);
		if (__fi129 == 0)
			pearl_ccism_force_inited = 1;
		/* PEARL-SCPREPUB: 启动周期重发（30 x 500ms，覆盖 MD 启动窗口） */
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

		/* PEARL-30: force_init(early publish + IPI) 属于非原厂时序的
		 * SCP 侧动作，统一纳入 pearl_ccism_inject 闸门（默认关）。 */
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
	/* PEARL-SCPRBREADY: SCP 早起的 RBREADY IPI 被 PEARL-28 错过(handler 推迟到 HS2/READY 才注册)。上面 force_init 已成功(ret=0)，
	 * 证明 SCP 已起来并在处理 IPI。这里仅把 scp_state 提到 RBREADY 维持状态一致；
	 * 0x11B(DONE) 由 pearl_ccism_11b_work_fn 经 SRAM 通道发出，不再于此经会被丢弃的 port 补发。 */
	if (pearl_ccism_force_inited) {
		if (atomic_read(&scp_state) != SCP_CCCI_STATE_RBREADY) {
			atomic_set(&scp_state, SCP_CCCI_STATE_RBREADY);
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPRBREADY: scp_state BOOTING->RBREADY (SCP alive)\n");
		}
		/* 0x11B(DONE) 改由 pearl_ccism_11b_work_fn 经 SRAM 通道(0x119 同路，
		 * MD 在 HS2 实际消费)在 0x119 之后发出；此处若经 port(queue0)补发会
		 * 先于 0x119 乱序，且 queue0 在 HS2 被 MD 丢弃(实证 #45/r116c)。 */
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
	/* PEARL-25: scp_A_register_notify 由 scp.ko 导出，内建 CCCI 不能引用。
	 * 它唯一的作用是把 SCP_EVENT_READY 接到 fsm_scp_init0()（IPI 注册），
	 * 而 scp.ko 没加载时这个事件永远不会来。真正决定 HS2 的是下面那两个
	 * register_ccci_sys_call_back()，照常执行。 */
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

	/* PEARL-30 (r121): autoreg=3 -> builtin start(~2.2s) 就开始注册，
	 * scp_ipidev 在 1.697s 已就绪，重试兜底。远早于 MD 上电(7.2s)。 */
	if (pearl_scp_ipi_autoreg == 3 && scp_ctl->md_id == MD_SYS1)
		schedule_delayed_work(&pearl_scp_ipi_early_work, 0);

	return ret;
}

#ifdef CCCI_KMODULE_ENABLE
/* PEARL-25: 本树把 SCP 胶水折进内建的 ccci_md_all，而没有走原厂的
 * "mediatek,ccci_md_scp" 平台设备（本机 DTS 没有这个节点，而且 scp.ko
 * 一 insmod 就挂死）。所以由 ccci_fsm_init() 直接调用这一入口，把
 * fsm_scp_init() 的两个 register_ccci_sys_call_back() 装上 ——
 * 这正是基带 HS2 阶段缺的东西（CCISM_SHM_INIT_ACK / MD_SIM_TYPE）。
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
