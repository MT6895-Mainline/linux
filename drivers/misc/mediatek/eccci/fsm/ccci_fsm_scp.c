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
/* PEARL-SCPADDR (r135): 覆盖发布给 SCP 的 smem 地址（0 = 用真实物理地址）。
 * 目的：SCP 自打印 shm_addr=[0]（它读到的地址是 0），需 A/B 试不同视角
 * （MD 视角 0x40020000 / SCP 视角 0xde020000 ...）找出它接受的形式。 */
unsigned int pearl_scp_smem_addr;
module_param(pearl_scp_smem_addr, uint, 0644);
MODULE_PARM_DESC(pearl_scp_smem_addr,
	"PEARL-SCPADDR: override the smem address published to the SCP (0=real)");

/* PEARL-SCPMEM137 (r137): 发布给 SCP 的 smem 基址改用 DTS 里的
 * reserve-memory-scp_share（0x8F000000 / 0x6a2000）。理由：SCP 固件在
 * 读这个地址之前打的是 "PRINCIPAL.transceiver share memory config"，
 * 说明它要的是共享内存基址，而不是 r129-r136 一直发的 CCISM_SCP
 * (0x8e020000)。0 = 回到真实 phy。 */
unsigned int pearl_scp_smem_force;   /* r147: 默认 0，不再强制改 smem 地址 */
module_param(pearl_scp_smem_force, uint, 0644);
MODULE_PARM_DESC(pearl_scp_smem_force,
	"PEARL-SCPMEM137: smem base published to SCP (0 = use real phy)");

/* PEARL-SCPKEY137 (r137): 第一次发布后 pearl_scp_key_break_ms 毫秒起写坏 key。
 * SCP 的 conn_isr_handler 每次门铃都重读这 3 个字，一次启动里有两次门铃
 * （SCP 2.058s / 3.235s，间隔 1.18s），于是：
 *  - 第二次数到坏 key -> SCP 打 "no support smem !" = 它读的正是我们写的字；
 *  - 第二次仍打 shm_addr=[] -> 那个 tag 不是我们写的（另有发布者或别名）。
 * 0 = 从不写坏 key。 */
unsigned int pearl_scp_key_break_ms = 0;
module_param(pearl_scp_key_break_ms, uint, 0644);
MODULE_PARM_DESC(pearl_scp_key_break_ms,
	"PEARL-SCPKEY137: ms after first publish to start a broken key (0=never)");

/* PEARL-SCPSWEEP137 (r137): 在 CHDATA 的 +0xC..+0x2C 写唯一 magic
 * 0xA5A5xxxx（+0x8 留给真实地址），用 SCP 打的 shm_addr=[...] 判定它实际
 * 读的偏移。只在写合法 key 阶段做，避免脏了 MD 后续的真实报文。 */
unsigned int pearl_scp_sweep = 0;
module_param(pearl_scp_sweep, uint, 0644);
MODULE_PARM_DESC(pearl163_sweep_run,
	"PEARL-SCPSWEEP137: probe CCIF2 CHDATA +0xC..+0x2C with magics (1=on)");

#define PEARL_SCP_SMEM_KEY_BAD		0xDEADBEEFu
#define PEARL_SCP_SWEEP_LO		0xC
#define PEARL_SCP_SWEEP_HI		0x2C
#define PEARL_SCP_SWEEP_BASE		0xA5A50000u

static int pearl_scp_key_broken;
static unsigned long pearl_scp_pub_first_jiffies;
static unsigned int pearl_scp_pub_calls;

/* PEARL-SCPVIEW138 (r138): 地址视角。r137 实证 SCP 的映射表是
 * ap=0x10000000 -> scp=0x60000000（差 +0x50000000），即 AP 视角 0x8e020000
 * 在 SCP 眼里是 0xde020000。SCP 一直拿不到能用的地址（CCISM 区全 0、
 * shm_addr=[0]），怀疑就是我们一直给 AP 视角地址。1 = 发给 SCP 时换算成
 * SCP 视角（默认开）；0 = 原样（AP 视角）。 */
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

	if (pearl_scp_repub_cnt >= 120)
		return;
	pearl_scp_repub_cnt++;
	r = ccci_md_get_smem_by_user_id(MD_SYS1, SMEM_USER_CCISM_SCP);
	if (r)
		phy = (u32)r->base_ap_view_phy;
	if (phy) {
		/* PEARL-SCPADDR: 运行时可覆盖 */
		u32 val = pearl_scp_smem_addr ? pearl_scp_smem_addr :
			  (pearl_scp_smem_force ? pearl_scp_smem_force : phy);
		u32 rb;

		/* PEARL-SCPKEY137: 到点后改写坏 key，用于一次启动内区分
		 * "SCP 读的是我们的字" 与 "tag 另有发布者"。 */
		if (pearl_scp_key_break_ms && pearl_scp_pub_first_jiffies &&
		    !pearl_scp_key_broken &&
		    jiffies_to_msecs(jiffies - pearl_scp_pub_first_jiffies) >=
		    pearl_scp_key_break_ms) {
			pearl_scp_key_broken = 1;
			CCCI_NORMAL_LOG(-1, FSM,
				"PEARL-SCPKEY137: 从现在起写坏 key (elapsed=%ums)\n",
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

/* ===== PEARL-151: CCIF2 CHDATA 三元组布局实验 =====
 * LK 告诉 TFA 的 SCP smem 坐标是三个值：
 *   ap_base:0x8E020000  md_base:0x40020000  size:0x8000
 * 我们此前往 CHDATA+0x8/+0xC/+0x10 写的都是同一个 AP 地址。
 * layout=1: +0x8=ap(scp view), +0xC=md_base, +0x10=size   ← 推测是 SCP 期望的
 * layout=0: 三个都写 ap 地址（旧行为，做对照）
 */
/* ===== PEARL-162: CCISM_SCP 内容标记法 + IPI 载荷变体 =====
 * 目的：MD 的 ccci_shm_bm 会校验 CCISM_SCP 里的结构（现在全 0 → 断言 para 全 0）。
 * 我们把 CCISM_SCP 每个 4 字节偏移填成"自带偏移编号"的 marker，
 * 然后看 MD 断言的 para0/para1/para2 里出现哪个编号 —— 立刻知道它读的是哪个偏移，
 * 完全不需要符号表或反汇编。
 *   pearl_ccism_mark: 0=不填(保持 memset 全 0) 1=整区填 marker 2=只填前 0x100
 *   pearl_ccism_ipi_mode: 0=u32(AP视角) 1=u32(SCP视角) 2=u64(AP视角) 3=u64(SCP视角) 4={ap,md,size}
 * MD 断言后 FSM 会自动重启 MD，所以运行期改这两个值即可在下一轮生效（不必重启手机）。
 */
unsigned int pearl_ccism_mark = 0;   /* PEARL-171: 默认不再涂标记 */
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
#define PEARL_SCP_TCM_LOG_PA	0x1c5af000UL	/* SCP TCM + 0x16f000（LOGG 包给的日志环） */
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
	void *va_dyn;   /* PEARL-34b: memremap 读取缓冲 */
};

static struct dentry *pearl_dbg_dir;


/* PEARL-MDSNAP140 (r140): HS2 窗口内周期性快照 MD 内存（0xc0170000, 32MB）。
 * md_bank0 节点只读断电后的 0xFF —— 必须在 MD 还活着时抓。保留最后一次
 * 快照，断言/断电后仍可从 debugfs 读。 */
#define PEARL_MD_SNAP_SZ	0x2000000
/* PEARL-MDSNAP141: 快照物理地址做成模块参数（默认 0xc0170000=旧行为；
 * 实测 LK hdr_tbl_inf 与 DT md_mem_usage 都说 md1 在 0xD0000000/480MB，
 * 但 r126 在 GZ 在场时读它挂死。先断电后改写参数探测，确认安全再活体快照。） */
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

		/* 快照是普通内存，pearl_dbg_read 的 memcpy_fromio 同样适用 */
		return pearl_l2sram_snap;
	}
	case PDBG_SMEM_RAW: {
		struct ccci_smem_region *r = ccci_md_get_smem_by_user_id(
			MD_SYS1, SMEM_USER_RAW_MDSS_DBG);

		return (r && r->base_ap_view_vir) ? r->base_ap_view_vir : NULL;
	}
	case PDBG_MDBANK: {
		/* PEARL-MDSNAP140: 有内核快照就返回快照（避免断电后读回全 0xFF） */
		if (pearl_md_snap_done && n->va_dyn) {
			CCCI_NORMAL_LOG(MD_SYS1, FSM,
				"PEARL-MDSNAP140: serving kernel snapshot (ms=%u nz=%u)\n",
				pearl_md_snap_ms, pearl_md_snap_nz);
			return n->va_dyn;
		}
		/*
		 * PEARL-34c (r127): MD 运行期已解密的代码/数据区 dump。
		 * md1img 里代码段是密文（熵 7.2），运行期解密载入 DRAM。
		 * AP 物理地址 = 0xC0170000 起 48MB（stock DT modem_info_v2，
		 * 与我们 DT 一致）。注意 LK tag "md_bank0_base"=0xD0000000 是
		 * MD 视图地址，直接当 AP PA memremap 会挂死（r126 教训）。
		 */
		phys_addr_t pa = (phys_addr_t)pearl_md_snap_pa;
		void __iomem *p;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-34c: md_bank0 dump pa=%pap size=0x%zx\n",
			&pa, n->size);
		p = ioremap(pa, n->size);
		if (p) {
			memcpy_fromio(n->va_dyn, p, n->size);
			iounmap(p);
			return n->va_dyn;
		}
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-34c: ioremap failed\n");
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
	/* r158: SCP 固件自己的日志环（在 SCP TCM 里，DEVAPC 不拦；LOGG 包给出
	 * 指针 0x16f008/0x16fc08，基址约 0x16f000 → AP PA 0x1c5af000） */
	{ "scp_tcmlog", PDBG_IOMEM,    NULL, PEARL_SCP_TCM_LOG_SZ },
};


/* ===== PEARL-163: SCP 复位 + 内存标记扫描（找 SCP 从哪里读 smem 地址） =====
 *
 * 背景：SCP 固件在自己启动时（t≈3.5s）读 shm_addr，读到 0；我们从 Linux 发 IPI
 * 永远晚一步。但它每次被复位重启都会重新读一遍 —— 所以我们：
 *   ① 往候选位置写一个"自带偏移编号"的 marker
 *   ② 用 SMC 复位并释放 SCP（RESET_SET → RESET_RELEASE）
 *   ③ 等 SCP 重启后，在它的日志区里搜 "shm_addr=[dead....]" 
 *   ④ 命中 → 那个偏移就是它读地址的位置（打印出来）
 * 全部在内核侧自动跑，一条命令完成，不用刷机/重启手机。
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

/* 在 SCP 日志区里找 marker 字符串 "shm_addr=[deadXXXX" */
static int pearl163_find_marker(void __iomem *log_va, u32 log_sz, char *out, size_t out_sz)
{
	char *buf = kmalloc(log_sz, GFP_KERNEL);
	int found = 0;
	size_t i;

	if (!buf)
		return 0;
	memcpy_fromio(buf, log_va, log_sz);
	for (i = 0; i + 22 < log_sz; i++) {
		if (!memcmp(buf + i, "shm_addr=[dead", 14)) {
			memcpy(out, buf + i, min_t(size_t, 24, out_sz - 1));
			out[min_t(size_t, 24, out_sz - 1)] = 0;
			found = 1;
			break;
		}
	}
	kfree(buf);
	return found;
}

static void pearl163_sweep_run(u32 base_pa, u32 size)
{
	void __iomem *va, *log;
	u32 off;
	char hit[32];

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-163: fill start pa=0x%x size=0x%x\n", base_pa, size);
	va = ioremap(base_pa, size);
	log = ioremap(0x8f16f000, 0x10000);   /* SCP 日志环（share 区 +0x16f000） */
	if (!va || !log) {
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-163: ioremap fail\n");
		goto out;
	}
	/* ① 整区填"自带偏移编号"的 marker：0xdead0000 | (offset>>2) 的低 16 位
	 *    → SCP 若从任一位置读地址，日志里就会打出对应的编号，一次定位 */
	for (off = 0; off + 4 <= size; off += 4)
		writel(0xdead0000 | ((off >> 2) & 0xffff), va + off);
	mb();
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-163: fill done, 复位 SCP\n");

	/* ② 复位并释放 SCP（它会重新启动、重新读 smem 地址） */
	pearl163_smc_reset_set(0);
	msleep(80);
	pearl163_smc_reset_release();
	msleep(4000);            /* 等 SCP 起来并打印 */

	/* ③ 在 SCP 日志里找 marker */
	if (pearl163_find_marker(log, 0x10000, hit, sizeof(hit))) {
		u32 v = 0;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-163: ★HIT★ base=0x%x -> %s\n", base_pa, hit);
		/* 从 "shm_addr=[deadXXXX" 里取数 */
		{
			char *b = strchr(hit, '[');

			if (b)
				kstrtouint(b + 1, 16, &v);
		}
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-163: 解码 value=0x%x -> 偏移 = 0x%x (base=0x%x)\n",
			v, (v & 0xffff) << 2, base_pa);
	} else {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-163: 无命中（base=0x%x size=0x%x）\n", base_pa, size);
	}
out:
	if (va)
		iounmap(va);
	if (log)
		iounmap(log);
}

/* debugfs: echo "<base_pa> <size>" > sweep */
static ssize_t pearl163_sweep_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	char kbuf[64];
	unsigned int base = 0, size = 0;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = 0;
	{
		char *sp = strchr(kbuf, ' ');

		if (!sp)
			return -EINVAL;
		*sp = 0;
		if (kstrtouint(kbuf, 16, &base) || kstrtouint(sp + 1, 16, &size))
			return -EINVAL;
		if (!size || size > 0x200000)
			return -EINVAL;
	}
	/* 只允许安全区域，避免 DEVAPC 挂死整机 */
	if (!((base >= 0x8e000000 && base + size <= 0x8e200000) ||
	      (base >= 0x8f000000 && base + size <= 0x8f700000) ||
	      (base >= 0x1c400000 && base + size <= 0x1c600000) ||
	      (base >= 0x1023c000 && base + size <= 0x1023e000))) {
		CCCI_ERROR_LOG(MD_SYS1, FSM,
			"PEARL-163: 地址不在白名单（0x8e00_0000/0x8f00_0000/0x1c40_0000/0x1023C000）\n");
		return -EINVAL;
	}
	pearl163_sweep_run(base, size);
	return count;
}

static ssize_t pearl163_reset_write(struct file *file, const char __user *ubuf,
				 size_t count, loff_t *ppos)
{
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-163: 手动复位 SCP\n");
	pearl163_smc_reset_set(0);
	msleep(60);
	pearl163_smc_reset_release();
	msleep(2600);
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-163: SCP 复位完成\n");
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
	/* 只把可打印段丢给用户，省得刷屏 */
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

/* PEARL-163b: 往指定物理地址写一个 32 位值（用于把真实 smem 地址写进扫描命中的位置） */
static ssize_t pearl163_poke_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	char kbuf[64];
	unsigned int pa = 0, val = 0;
	char *sp;
	void __iomem *va;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = 0;
	sp = strchr(kbuf, ' ');
	if (!sp)
		return -EINVAL;
	*sp = 0;
	if (kstrtouint(kbuf, 16, &pa) || kstrtouint(sp + 1, 16, &val))
		return -EINVAL;
	if (!((pa >= 0x8e000000 && pa < 0x8e200000) ||
	      (pa >= 0x8f000000 && pa < 0x8f700000) ||
	      (pa >= 0x1c400000 && pa < 0x1c600000) ||
	      (pa >= 0x8e500000 && pa < 0x8ed00000) ||   /* r167 consys EMI */
	      (pa >= 0x1023c000 && pa < 0x1023e000)))
		return -EINVAL;
	va = ioremap(pa & ~0xfffUL, 0x1000);
	if (!va)
		return -ENODEV;
	writel(val, va + (pa & 0xfff));
	mb();
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-163b: poke 0x%x <- 0x%x (读回 0x%x)\n",
		pa, val, readl(va + (pa & 0xfff)));
	iounmap(va);
	return count;
}

/* PEARL-165: 只填充 marker，不复位 SCP —— 用于"填充 → 冷启动(重启手机) → 看 SCP 日志"的实验。
 * 理由：SCP 只在【冷启动】时读一次 shm 地址，热复位不会重读（r164 实测）。 */
static ssize_t pearl165_fill_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	char kbuf[64];
	unsigned int pa = 0, size = 0;
	char *sp;
	void __iomem *va;
	u32 off;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;
	kbuf[count] = 0;
	sp = strchr(kbuf, ' ');
	if (!sp)
		return -EINVAL;
	*sp = 0;
	if (kstrtouint(kbuf, 16, &pa) || kstrtouint(sp + 1, 16, &size))
		return -EINVAL;
	if (!size || size > 0x200000)
		return -EINVAL;
	if (!((pa >= 0x8e000000 && pa + size <= 0x8e200000) ||
	      (pa >= 0x8f000000 && pa + size <= 0x8f700000) ||
	      (pa >= 0x1c400000 && pa + size <= 0x1c600000) ||
	      /* r166: SCP 自己的 DRAM（LK 的 SCP-reserved / 放启动参数的地方）。

	       *       0xbfc00000 之后有 DEVAPC 保护（读会挂），所以只放开前 3MB */

	      (pa >= 0xbf900000 && pa + size <= 0xbfc00000) ||
	      /* r167: connsys EMI 保留区（SCP 的 shm 就在其中 +0x7E0000） */
	      (pa >= 0x8e500000 && pa + size <= 0x8ed00000) ||
	      (pa >= 0x1023c000 && pa + size <= 0x1023e000)))
		return -EINVAL;
	va = ioremap(pa, size);
	if (!va)
		return -ENODEV;
	for (off = 0; off + 4 <= size; off += 4)
		writel(0xdead0000 | ((off >> 2) & 0xffff), va + off);
	mb();
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-165: fill-only pa=0x%x size=0x%x 完成（请重启手机让 SCP 冷启动）\n",
		pa, size);
	iounmap(va);
	return count;
}

static const struct file_operations pearl165_fill_fops = {
	.owner = THIS_MODULE, .write = pearl165_fill_write,
};

/* PEARL-167: 按正常机（yuechu）的模板，在 SCP 的 shm 处建好头部。
 * yuechu 正常日志: [conn_shm_init] shmaddr=[9ece0000] pat=[46494353][46494353]
 *                  ver=[20210610] [10000] [20][40][60][c000]
 * 地址换算：AP 视角 = EMI 基址(0x8e500000) + 0x7E0000 = 0x8ece0000
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
	/* 模板（来自 yuechu 正常机日志） */
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
		"PEARL-167: 已写 SCP shm 头 @0x%lx: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		PEARL_SCP_SHM_PA, readl(&w[0]), readl(&w[1]), readl(&w[2]),
		readl(&w[3]), readl(&w[4]), readl(&w[5]), readl(&w[6]),
		readl(&w[7]));
	for (i = 8; i < 16; i++)
		writel(0, &w[i]);
	iounmap(va);
	return count;
}

/* ===== PEARL-168: conap（connectivity↔SCP）最小复刻 =====
 *
 * 背景（来自 yuechu 正常机日志 + vendor conap_scp 驱动）：
 *   [2.887] [conap_intf_rx_task] drv query [1][0]      ← SCP 通过 conap 向 AP 查询
 *   [3.206] [conn_shm_init] shmaddr=[9ece0000]          ← 之后 SCP 才把共享内存建起来
 *   vendor: SCP ready 时 AP 发 conap INIT：
 *       conap_scp_ipi_send_cmd(DRV_TYPE_CORE, CONAP_SCP_CORE_INIT, shm_addr, shm_size)
 *       通道 IPI_OUT_SCP_CONNSYS(=33)，接收 IPI_IN_SCP_CONNSYS(=34)
 *
 * 我们的树里【没有 conap 驱动】⇒ 34 号接收 pin 从未注册、也从没人给 SCP 发 INIT
 *   ⇒ SCP 的 conap 消息被丢弃（与 r150「ready IPI 没注册」同一类问题）。
 *
 * 这里做最小复刻：注册 34 号接收 pin（打印日志）+ 提供一个发送 INIT 的开关。
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

/* SCP 的 shm：EMI 基址 0x8e500000 + 0x7E0000（vendor conap 的 mt6895 表） */
unsigned int pearl168_shm_addr = 0x8ece0000;
module_param(pearl168_shm_addr, uint, 0644);
unsigned int pearl168_shm_size = 0x20000;
module_param(pearl168_shm_size, uint, 0644);
unsigned int pearl168_conap_auto = 1;
module_param(pearl168_conap_auto, uint, 0644);
MODULE_PARM_DESC(pearl168_conap_auto, "1=SCP ready 后自动发 conap INIT");

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

/* 34 号（IPI_IN_SCP_CONNSYS）接收回调：把 SCP 的 conap 消息打出来 */
static int pearl168_conap_recv(unsigned int id, void *prdata, void *data,
			       unsigned int len)
{
	struct pearl168_msg_cmd *c = (struct pearl168_msg_cmd *)data;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: ★SCP conap 消息★ id=%u drv=%u msg=%u tot=%u this=%u p0=0x%x p1=0x%x\n",
		id, c->drv_type, c->msg_id, c->total_sz, c->this_sz,
		c->param0, c->param1);
	/* SCP 查询驱动是否就绪 → 回 ACK（照 vendor 行为） */
	if (c->msg_id == PEARL168_MSG_DRV_QRY) {
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-168: 收到 drv query(drv=%u) → 回 ACK(1)\n",
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

/* debugfs: echo 1 > scp168_conapinit  → 手动发一次 conap INIT */
static ssize_t pearl168_init_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	int ret;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-168: 手动发 conap INIT shm=0x%x size=0x%x\n",
		pearl168_shm_addr, pearl168_shm_size);
	ret = pearl168_conap_send(PEARL168_MSG_INIT, pearl168_shm_addr,
				  pearl168_shm_size);
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-168: INIT 结果 ret=%d\n", ret);
	return count;
}

/* ===== PEARL-169: 随机 smem key（照抄正常工作机的行为） =====
 * yuechu 正常机 SCP 日志：scp_smem_key : 0xE775F5BDE5619A71（每次启动都不同的随机 64 位）
 * 我们一直用 LK 给的固定常量 0x5343505F534D454D。这里加开关用随机 key。
 */
unsigned int pearl169_randkey = 1;
unsigned int pearl170_early_reg = 1;
/* ===== PEARL-172: 发布/使用的 CCISM smem 基址可调 =====
 * yuechu 正常机 AP 日志：[ccci1/shm] smem_port->addr_phy=8e02c000
 *   ⇒ 正常机用的是 0x8e02c000（区域基址 + 0xC000），而我们一直发 0x8e020000。
 * 这里把"要发布的 ap 基址"和"要用的本地基址"都做成开关，默认改成 0x8e02c000。
 */
unsigned long pearl172_pub_base = 0x8e020000;   /* PEARL-174: 照正常机用 AP 视角地址 */
module_param(pearl172_pub_base, ulong, 0644);
MODULE_PARM_DESC(pearl172_pub_base, "要发布给 SCP/MD 的 CCISM ap 基址（正常机为 0x8e02c000）");

module_param(pearl170_early_reg, uint, 0644);
MODULE_PARM_DESC(pearl170_early_reg, "1=在 fsm_scp_init0 就注册 IPI_IN_APCCCI_0（抢在 SCP 的 CCCI IPI 之前）");
module_param(pearl169_randkey, uint, 0644);
MODULE_PARM_DESC(pearl169_randkey, "1=每次发布用随机 64 位 smem key（照正常机行为）");

u32 pearl169_key_lo(void)
{
	static u32 lo;

	if (!pearl169_randkey)
		return 0x534d454d;      /* 原常量 */
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

/* ===== PEARL-173: 在内核侧替用户态守护进程打开 CCB 端口 =====
 *
 * 正常机（yuechu/Android）日志：
 *   [7.356] [ccci1/chr] port ccci_ccb_ctrl open with flag 20002 by ccci_mdinit
 *   [7.356] [ccci1/shm] ccb_configs_len: 20 / find ccb port ccci_ccb_dhl for user1!
 *   [8.387] [ccci1/shm] smem_port->addr_phy=8e02c000
 *   [8.387] [ccci1/fsm] control message 0x0,0x5555FFFF   ← MD 随即发 HS1
 *   [9.254] md_state 3 → 4 (READY)
 *
 * pearl 没有 ccci_mdinit 这个用户态守护进程 ⇒ CCB 端口从未被打开 ⇒ smem 端口不建立
 *   ⇒ MD 的 ccci_shm_bm 断言。这里用内核 API 代它打开。
 */
extern int mtk_ccci_request_port(char *name);
extern int mtk_ccci_open_port(int index);

unsigned int pearl173_open_ccb = 1;
module_param(pearl173_open_ccb, uint, 0644);
MODULE_PARM_DESC(pearl173_open_ccb, "1=内核代开 CCB/raw 端口（ccb_ctrl/dhl/md_monitor/meta/raw_dhl/fs）");

static const char *pearl173_ports[] = {
	"ccci_ccb_ctrl",
	"ccci_ccb_dhl",
	"ccci_ccb_md_monitor",
	"ccci_ccb_meta",
	"ccci_raw_dhl",      /* PEARL-192: 正常机 READY 后由 emdlogger 打开 */
	"ccci_fs",           /* PEARL-192: 基带文件服务 */
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
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-173: 手动触发开端口\n");
	pearl173_open_ports();
	return count;
}

/* PEARL-177: 给 MD 自动启动路径用的"提前开端口"入口 */
void pearl177_open_ports_early(void)
{
	pearl173_open_ports();
}

/* ===== PEARL-180: MD 起来后补发 CCISM SHM_INIT（地址/时机/模式全部可运行时调） =====
 *
 * 对照正常机（yuechu）：
 *   [9.255] [ccci1/fsm] IPI send op_id=2/data=0x8e020000, size=8   ← AP 视角地址，且在 MD ready 之后
 * pearl 现在是 3.7s（MD 未起）用 SCP 视角 0xde020000 发一次。
 * 这里：MD 进 state 3 时自动补发，地址与模式用模块参数控制（sysfs 可写，无需重编）。
 */
unsigned int pearl180_auto = 1;
module_param(pearl180_auto, uint, 0644);
MODULE_PARM_DESC(pearl180_auto, "1=MD 进 state 3 时自动补发 CCISM SHM_INIT");
unsigned int pearl180_data = 0x8e020000;   /* 正常机用的是 AP 视角地址 */
module_param(pearl180_data, uint, 0644);
MODULE_PARM_DESC(pearl180_data, "补发时用的地址值（正常机 0x8e020000）");
unsigned int pearl180_delay_ms = 0;
module_param(pearl180_delay_ms, uint, 0644);
MODULE_PARM_DESC(pearl180_delay_ms, "补发前的延迟（ms）");

static void pearl180_resend(void)
{
	u32 v = pearl180_data;
	int ret;

	if (pearl180_delay_ms)
		msleep(pearl180_delay_ms);
	ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_SHM_INIT, &v);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-180: 补发 SHM_INIT data=0x%x ret=%d\n", v, ret);
}

static void pearl180_work_fn(struct work_struct *w)
{
	pearl180_resend();
	/* 再补两次，覆盖 0x11A 窗口 */
	msleep(300); pearl180_resend();
	msleep(500); pearl180_resend();
}
static DECLARE_DELAYED_WORK(pearl180_work, pearl180_work_fn);

void pearl180_on_md_state(int state)
{
	if (!pearl180_auto || state != 3)
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-180: MD 进入 state 3 → 安排补发 SHM_INIT(data=0x%x)\n",
		pearl180_data);
	schedule_delayed_work(&pearl180_work, msecs_to_jiffies(50));
}

static ssize_t pearl180_send_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	CCCI_NORMAL_LOG(MD_SYS1, FSM, "PEARL-180: 手动补发\n");
	pearl180_resend();
	return count;
}
/* ===== PEARL-181: 把 SCP→AP 的 pin 全部注册上，并照 vendor 逻辑回应 =====
 *
 * 实测：SCP 会发 "IPI send 0/0x2"（正常机也有 ✓），mbox 中断计数确实增加 ✓，
 * 但 ccci_scp_ipi_handler() 从未执行 ✗ ⇒ SCP 用的 pin 与我们注册的（7/9/34）不同。
 * 这里把 0..40 号 pin 全部注册一个日志 handler（已注册的跳过），并把
 * vendor ccci_scp_ipi_rx_work 的关键分支搬过来：
 *   CCCI_OP_SCP_STATE + SCP_CCCI_STATE_RBREADY  →  给 MD 发 CCISM_SHM_INIT_DONE(0x11B)
 */
static int pearl181_recv(unsigned int id, void *prdata, void *data, unsigned int len)
{
	struct ccci_ipi_msg *m = (struct ccci_ipi_msg *)data;

	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-181: ★SCP→AP pin=%u op_id=%d data0=0x%x len=%u★\n",
		id, m->op_id, m->data[0], len);

	/* vendor: SCP 报 RBREADY → 告诉 MD shm init 完成（HS2 的关键触发） */
	if (m->op_id == CCCI_OP_SCP_STATE &&
	    m->data[0] == SCP_CCCI_STATE_RBREADY) {
		int ret;

		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-181: SCP RBREADY → 发 0x11B 给 MD\n");
		ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
					       CCISM_SHM_INIT_DONE, 0, 1);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-181: 0x11B ret=%d\n", ret);
	}
	return 0;
}

/* PEARL-197: 1 = 允许 r181 的全量 SCP pin 注册（默认 0，诊断用） */
unsigned int pearl181_enable;
module_param(pearl181_enable, uint, 0644);
MODULE_PARM_DESC(pearl181_enable, "1=注册 0~40 全部 SCP pin（会抢各子系统的引脚，默认关）");

/* PEARL-196: 0 = 不碰 sensorhub 的 30/32 引脚（默认） */
unsigned int pearl181_force_pins;
module_param(pearl181_force_pins, uint, 0644);
MODULE_PARM_DESC(pearl181_force_pins, "1=连 sensorhub 的 30/32 引脚也注册（会弄坏旋转传感器）");

static u32 pearl181_rxbuf[64];

void pearl181_register_all(void)
{
	int i, ok = 0;

	for (i = 0; i <= 40; i++) {
		int ret;

		if (i == 7 || i == 9 || i == 34)   /* 已注册的跳过 */
			continue;
		/*
		 * PEARL-196: 30 / 32 是 sensorhub 的 IPI 引脚，绝对不能碰。
		 * 传感器 hub 的日志：
		 *   [5.587] ipi_comm sensor IPI handlers registered (ctrl 30, notify 32)
		 * r181 的全量注册把这两个抢了，导致
		 *   [10.725] sensor_ready sensor hub gave no ready ack
		 * 旋转传感器（pearl-accel）随之失效。这里跳过它们；真要用可以
		 * 通过 pearl181_force_pins 打开（默认关闭）。
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
		"PEARL-181: 额外注册 %d 个 SCP pin\n", ok);
}

/* ===== PEARL-182: 由 AP 初始化 ccism_scp（MD 的 shm 检查要读它） =====
 *
 * 实测：ccism_scp（0x8e020000, 32KB）在 pearl 上始终全 0，SCP 侧也没有任何
 * ccism/queue 活动，而 MD 偏偏断言在 ccci_shm_bm（shm 缓冲管理）。
 * 正常机的 SCP 日志同样没有 CCISM 活动 ⇒ 那块结构应由 AP/MD 侧建立。
 *
 * 这里在 MD START 之前往 ccism_scp 写入可运行时指定的测试图案，用来定位
 * MD 到底读哪一个偏移/哪一种签名（写对了 → ccci_shm_bm 断言应改变或消失）。
 */
unsigned int pearl182_enable = 1;
module_param(pearl182_enable, uint, 0644);
MODULE_PARM_DESC(pearl182_enable, "1=MD START 前初始化 ccism_scp");
unsigned int pearl182_off = 0;          /* 起始偏移（字节） */
module_param(pearl182_off, uint, 0644);
unsigned int pearl182_pat = 0;          /* 0=按偏移编号填充；其它=全部写成该值 */
module_param(pearl182_pat, uint, 0644);
unsigned int pearl182_len = 0x8000;     /* 覆盖长度 */
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
		CCCI_ERROR_LOG(MD_SYS1, FSM, "PEARL-182: ccism_scp 区域不可用\n");
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
		"PEARL-182: ccism_scp 已填 off=0x%x len=0x%x pat=0x%x (首字=0x%x)\n",
		pearl182_off, pearl182_len, pearl182_pat, p[0]);
}

static ssize_t pearl182_write(struct file *file, const char __user *ubuf,
			      size_t count, loff_t *ppos)
{
	pearl182_init_ccism();
	return count;
}
/* PEARL-183: 导出 L2SRAM 异常快照（MD 栈在里面） */
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

/* ===== PEARL-191: READY 之后照正常机给 SCP 补发 MD_STATE=2 =====
 *
 * yuechu（正常机）在 md_state 3->4 (READY) 之后立刻：
 *   [9.256] [ccci1/fsm] IPI send op_id=1/data=0x2, size=8
 * pearl 只发过 data=0x1（启动期）和 0x3（异常时），从来没有 0x2。
 * 枚举：op 1 = CCCI_OP_MD_STATE。推测 SCP 需要这个 2 才会把 MD 当成就绪，
 * 进而激活 MD<->SCP 的 CCISM 通路；否则 MD 起来约 2 秒后自行崩溃。
 */
unsigned int pearl191_auto = 1;
module_param(pearl191_auto, uint, 0644);
MODULE_PARM_DESC(pearl191_auto, "1=MD READY 后给 SCP 补发 MD_STATE=2");
unsigned int pearl191_val = 2;
module_param(pearl191_val, uint, 0644);
MODULE_PARM_DESC(pearl191_val, "补发的 MD_STATE 值（正常机为 2）");

static void pearl191_work_fn(struct work_struct *w)
{
	u32 v = pearl191_val;
	int ret;
	int i;

	for (i = 0; i < 3; i++) {
		ret = ccci_scp_ipi_send(MD_SYS1, CCCI_OP_MD_STATE, &v);
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-191: IPI MD_STATE=%u ret=%d (第 %d 次)\n",
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
		"PEARL-191: MD READY → 给 SCP 补发 MD_STATE=%u\n", pearl191_val);
	schedule_delayed_work(&pearl191_work, msecs_to_jiffies(20));
}

static ssize_t pearl191_send_write(struct file *file, const char __user *ubuf,
				   size_t count, loff_t *ppos)
{
	pearl191_work_fn(NULL);
	return count;
}
/* PEARL-192: 读/刷新 CCB DHL 里提取出的 MD 日志 */
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
/* ===== PEARL-194: READY 之后照正常机补发 system message 0x110 =====
 *
 * 正常机（yuechu）READY(9.254s) 之后发过：
 *   [9.257] system message (ffffffff 11b 2 0)   <- 0x11B，我们有 ✓
 *   [10.082] system message (ffffffff 110 2 0)  <- ★0x110★，我们从来没发过 ✗
 * 0x110 出现在 READY 之后约 0.83 秒，而 pearl 的 MD 在 READY 之后约 2.1 秒
 * 崩溃 —— 时序完全吻合"MD 在等 0x110"。用与 0x11B 相同的通路补发。
 */
unsigned int pearl194_auto = 1;
module_param(pearl194_auto, uint, 0644);
MODULE_PARM_DESC(pearl194_auto, "1=MD READY 后补发 system message 0x110");
unsigned int pearl194_msg = 0x110;
module_param(pearl194_msg, uint, 0644);
MODULE_PARM_DESC(pearl194_msg, "补发的消息号（正常机为 0x110）");
unsigned int pearl194_delay_ms = 830;
module_param(pearl194_delay_ms, uint, 0644);
MODULE_PARM_DESC(pearl194_delay_ms, "补发延迟（正常机相对 READY 约 830ms）");

static void pearl194_work_fn(struct work_struct *w)
{
	int ret;

	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
				       pearl194_msg, 0, 1);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: 补发 system message 0x%x ret=%d\n",
		pearl194_msg, ret);
	/* 再补两次，防止第一次落在窗口外 */
	msleep(400);
	ret = ccci_port_send_msg_to_md(MD_SYS1, CCCI_SYSTEM_TX,
				       pearl194_msg, 0, 1);
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: 第二次 0x%x ret=%d\n", pearl194_msg, ret);
}
static DECLARE_DELAYED_WORK(pearl194_work, pearl194_work_fn);

void pearl194_on_md_state(int state)
{
	if (!pearl194_auto || state != 4)
		return;
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-194: MD READY → %u ms 后补发 0x%x\n",
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

static const struct file_operations pearl163_poke_fops = {
	.owner = THIS_MODULE, .write = pearl163_poke_write,
};

static const struct file_operations pearl163_sweep_fops = {
	.owner = THIS_MODULE, .write = pearl163_sweep_write,
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
	debugfs_create_file("scp163_sweep", 0200, dir, NULL, &pearl163_sweep_fops);
	debugfs_create_file("scp163_poke", 0200, dir, NULL, &pearl163_poke_fops);
	debugfs_create_file("scp165_fill", 0200, dir, NULL, &pearl165_fill_fops);
	debugfs_create_file("scp167_shmhdr", 0200, dir, NULL, &pearl167_shmhdr_fops);
	debugfs_create_file("scp168_conapinit", 0200, dir, NULL, &pearl168_init_fops);
	debugfs_create_file("scp173_opencecb", 0200, dir, NULL, &pearl173_open_fops);
	debugfs_create_file("scp180_resend", 0200, dir, NULL, &pearl180_send_fops);
	debugfs_create_file("scp182_ccism", 0200, dir, NULL, &pearl182_fops);
	debugfs_create_file("scp192_mdlog", 0400, dir, NULL, &pearl192_mdlog_fops);
	debugfs_create_file("scp194_msg110", 0200, dir, NULL, &pearl194_fops);
	debugfs_create_file("scp192_refresh", 0200, dir, NULL, &pearl192_refresh_fops);
	debugfs_create_file("scp191_mdstate", 0200, dir, NULL, &pearl191_fops);
	debugfs_create_file("scp183_l2sram", 0400, dir, NULL, &pearl183_l2sram_fops);
	/*
	 * PEARL-197: 默认【不】做全量注册。
	 * r181 曾经把 0~40 号 pin 全部注册，结果抢走了 sensorhub 的
	 * ctrl 30 / notify 32，导致 "sensor hub gave no ready ack"、
	 * 旋转传感器失效。SCP 上还有音频/VoW、连接性等子系统，各自也可能
	 * 有自己的 pin —— 全量注册对它们是同样的风险。这里改成 opt-in：
	 * 需要诊断时把 pearl181_enable 设为 1。
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
	/* r158: SCP TCM 日志区 */
	pearl_dbg_nodes[8].va = ioremap(PEARL_SCP_TCM_LOG_PA, PEARL_SCP_TCM_LOG_SZ);
	/* PEARL-34b: md_bank0 的 32MB 读取缓冲（惰性内容，读时 memremap 填） */
	pearl_dbg_nodes[4].va_dyn = vzalloc(pearl_dbg_nodes[4].size);
	/* PEARL-SCPSHARE: 第 7 个节点的读取缓冲（惰性 memremap 填） */
	pearl_dbg_nodes[7].va_dyn = vzalloc(pearl_dbg_nodes[7].size);
	for (i = 0; i < ARRAY_SIZE(pearl_dbg_nodes); i++)
		debugfs_create_file(pearl_dbg_nodes[i].name, 0400,
				    pearl_dbg_dir, &pearl_dbg_nodes[i],
				    &pearl_dbg_fops);
	pearl163_tools_init(d);   /* PEARL-163: sweep / reset / log */
	/* PEARL-173: 6.5s 后（MD 启动窗口）代用户态打开 CCB 端口 */
	if (pearl173_open_ccb && !pearl173_opened)
		schedule_delayed_work(&pearl173_work, msecs_to_jiffies(6500));
	CCCI_NORMAL_LOG(MD_SYS1, FSM,
		"PEARL-SCPDBG-77: debugfs pearl_scp/ ready (dram=%px ccif2=%px)\n",
		pearl_dbg_nodes[0].va, pearl_dbg_nodes[3].va);
	return 0;
}


/* ===== PEARL-CCIF2ST (r146): CCIF2 窗口写-回读自检 =====
 * 目的：分辨 CCIF2 寄存器"全 0"的两种可能
 *   A) 窗口活着，SCP 固件确实没响应 IPI  -> 模式能回读
 *   B) 窗口被门控/断电，读 0 无意义        -> 模式读回 0
 * 顺带把 qqcandy 记录的真 key（SCP 固件在 0x6023C100 校验）
 * 写进去并立刻回读，验证"钥匙是否落地"。
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
		pat, rb, rb == pat ? "ROUNDTRIP-OK(窗口活着)" : "ROUNDTRIP-FAIL(被门控?)");

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

/* r146: 映射完成后做一次自检（每次调用都打，量很小） */
void pearl_ccif2_selftest_once(const char *where)
{
	pearl_ccif2_map();
	pearl_ccif2_selftest("publish");
	pearl_ccif2_selftest(where);
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
		/* PEARL-MDSNAP140: MD 还活着时快照它的 0xc0170000 内存，
		 * 每个 tick 刷新一次、保留最后一次（断电前的现场）。 */
		if (pearl_dbg_nodes[4].va_dyn) {
			void __iomem *p = ioremap((phys_addr_t)pearl_md_snap_pa,
				PEARL_MD_SNAP_SZ);
			u32 nz = 0;
			int j;

			if (p) {
				memcpy_fromio(pearl_dbg_nodes[4].va_dyn, p,
					PEARL_MD_SNAP_SZ);
				/* 统计"整页全非零"的页数：全 0xFF 的断电虚区会
				 * 得到很大的 nz，真实代码/数据区页内必有零字节。 */
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
	/* r147: 头两次发布做 CCIF2 写-回读自检（判定窗口是否活着） */
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
	/* PEARL-169: 随机 key（照正常机行为） */
	if (pearl169_randkey && !pearl_scp_key_broken) {
		key_lo = pearl169_key_lo();
		key_hi = pearl169_key_hi();
	}
	u32 off;
	u32 md_lo = 0, md_hi = 0, md_addr = 0;

	/* PEARL-SCPMEM137: 统一在这里做地址覆盖，保证任何调用点写进去的
	 * 都是同一个值（IPI 载荷仍用真实 phy，两者互不影响）。 */
	if (pearl_scp_smem_force)
		smem_phy = pearl_scp_smem_force;
	/* PEARL-SCPVIEW138: 地址视角换算。SCP 固件映射表 ap0x10000000->scp0x60000000，
	 * 所以 CCIF2 SRAM 里给 SCP 的地址要是 SCP 视角（AP addr + 0x50000000）。 */
	if (pearl_ccism_view)
		smem_phy += 0x50000000;
	if (!pearl_scp_pub_first_jiffies)
		pearl_scp_pub_first_jiffies = jiffies;
	pearl_scp_pub_calls++;
	if (pearl_scp_pub_calls == 1)
		CCCI_NORMAL_LOG(MD_SYS1, FSM,
			"PEARL-SCPVIEW138: view=%d published_smem=0x%x\n",
			pearl_ccism_view, smem_phy);

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
		writel(key_lo,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x0);
		writel(key_hi,
			pearl_ccif2_md_base + PEARL_CCIF2_CHDATA + 0x4);
		/* PEARL-151: 按 layout 写 {ap, md, size} 或旧的三份 ap */
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
	/* PEARL-SCPSWEEP137: 只在写合法 key 的阶段铺 magic，避免脏到 MD
	 * 后续的真实报文。magic 值自带偏移，SCP 打出来的就是它读的偏移。 */
	if (pearl163_sweep_run && !pearl_scp_key_broken) {
		for (off = PEARL_SCP_SWEEP_LO; off <= PEARL_SCP_SWEEP_HI;
		     off += 4) {
			writel(PEARL_SCP_SWEEP_BASE | off,
				pearl_ccif2_ap_base + PEARL_CCIF2_CHDATA + off);
			if (pearl_ccif2_md_base)
				writel(PEARL_SCP_SWEEP_BASE | off,
					pearl_ccif2_md_base +
					PEARL_CCIF2_CHDATA + off);
		}
	}
	/* SCP 是另一个核，写完要一次全屏障再发 IPI */
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
	/* PEARL-SCPMEM137/SCPKEY137: 前 4 次与坏 key 阶段打全量读回，用来
	 * 确认两个视图里到底有什么、以及 magic 是否真的落盘。 */
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
	/* PEARL-162: 按开关填"自带偏移编号"的 marker */
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
	/* PEARL-CCIF2: SCP 从 CCIF2 SRAM 读 key+addr，不是从 IPI 载荷；
	 * 必须在 IPI 之前把这两个字放好，否则 SCP 打 no support smem ! */
	pearl_scp_smem_publish(phy);
	/* PEARL-SCPVIEW138: IPI 载荷也用 SCP 视角（与 CCIF2 发布一致） */
	{
		u32 phy_v = phy + 0x50000000;
		u64 phy64 = (u64)phy;
		u64 phy64v = (u64)phy_v;
		u32 trip[3] = { phy, pearl_ccism_md_base, pearl_ccism_size };

		switch (pearl_ccism_ipi_mode) {
		case 0:
			/* PEARL-172: 用可调基址（正常机 0x8e02c000） */
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
	/* PEARL-170: 立即注册 IPI_IN_APCCCI_0。
	 * 正常机(yuechu) SCP 在 t≈3.13s 就发 CCCI IPI（"IPI send 0/0x2"），
	 * 而我们把注册推迟到 scp_ready(t≈3.85s) ⇒ 那条消息被丢。
	 * 现在 SCP 侧 conap/CCCI smem/key/READY 都已正常，提前注册是安全的。 */
	if (pearl170_early_reg && !pearl_scp_ipi_registered)
		pearl_scp_ipi_register_now("PEARL-170:early");

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
	/* PEARL-168: SCP 就绪后注册 conap 接收并（可选）发 INIT */
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
