// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * qqcandy: staged, read-first CCIF bring-up diagnostics for the CCI stack.
 *
 * Three one-shot runtime triggers per module load, strictly ordered:
 *
 *   ccif_clk  (A) enable the six official CCIF clock gates by writing the
 *               infra-ao set registers (bit list verbatim from the official
 *               clk-mt6895-bus.c), verify they latched by reading STA, then
 *               READ ONLY the AP-side CCIF registers and SRAM.
 *   ccif_md   (B) same for the MD-side CCIF register bank (its gate is
 *               among the six enabled in A).
 *   ccif_ring (C) parse the LK tag chain fresh, rebuild the SMEM region
 *               table, and initialize the 16 normal + 16 exception CCIF
 *               ring buffers inside CCISM_MCU/CCISM_MCU_EXP exactly like
 *               the official md_ccif_ring_buf_init(): zero the region and
 *               write the official queue blocks. This is the first write
 *               to modem-visible memory; with the modem held in reset it
 *               has no consumer. Then register the two AP CCIF IRQs with
 *               IRQF_NO_AUTOEN: handlers installed but never enabled.
 *
 * The only CCIF register write anywhere (including the handlers, which
 * cannot run while the IRQs stay disabled) is the APCCIF ACK in the
 * handler. Unloading frees the IRQs, unmaps everything and restores the
 * clock gates to their pre-load state. Even so, none of this isolates a
 * bus fault: a wrong gate or region could still hang the SoC.
 *
 * DEVICE FINDING (2026-09-13): enabling the six clock gates is NOT
 * sufficient. The CCIF banks sit behind the MD power domain (genpd "md",
 * off with zero users on our boots); the first AP_CCIF read wedged the
 * bus with the domain off and the worker stayed D-state forever (device
 * recovered by a software reboot).
 *
 * A0 is therefore implemented as this module's platform probe on the
 * "mediatek,mddriver" node: pm_runtime_get_sync takes the MD MTCMOS
 * reference before any trigger is accepted. Without the mddriver DT
 * node (or with a failed domain power-on) every trigger fails closed
 * with -EPERM. The node lives in the board DTS and requires a kernel
 * image with the embedded DTB to be flashed.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/kstrtox.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "../ccci_util/ccci_tag_parse.h"
#include "../ccci_smem_dump/ccci_smem_layout.h"
#include "ccci_ccif_ringbuf.h"

#ifndef MODULE
#error "The CCI CCIF diagnostic must only be built as a module"
#endif

/* Official clk-mt6895-bus.c gate files, base = infracfg_ao 0x10001000
 * (size 0x1000 in our DT), verified on device by the LVTS STA observation
 * (0x10001090 = base + ifrao0_cg_regs.sta_ofs 0x90).
 */
#define INFRACFG_AO_BASE	0x0000000010001000ULL
#define INFRACFG_AO_SIZE	0x1000

#define IFRAO1_SET		0x88
#define IFRAO1_CLR		0x8C
#define IFRAO1_STA		0x94
#define IFRAO3_SET		0xC0
#define IFRAO3_CLR		0xC4
#define IFRAO3_STA		0xC8

/* The six clocks of the official ccif_clk_table[]:
 * ccif1_ap/ccif1_md/ccif_ap/ccif_md live in IFRAO1 bits 12/13/23/26,
 * ccif5_md/ccif4_md in IFRAO3 bits 10/29.
 */
#define IFRAO1_CCIF_BITS	((1u << 12) | (1u << 13) | \
				 (1u << 23) | (1u << 26))
#define IFRAO3_CCIF_BITS	((1u << 10) | (1u << 29))

/* MD bus protections (scpsys BUS_PROT_IGN entries do NOT verify these). */
#define IFRAO_PROT_INFRASYS1_STA	0x0C5C
#define IFRAO_PROT_INFRASYS0_STA	0x0C4C
#define IFRAO_PROT_EMISYS0_STA		0x0C6C
#define PROT_MASK_INFRASYS1_MD		BIT(9)
#define PROT_MASK_INFRASYS0_MD		BIT(28)
#define PROT_MASK_EMISYS0_MD		(BIT(17) | BIT(16))

/* topckgen md1_clk_mod (official md_cd_topclkgen_on clears bits 8|9). */
#define TOPCKGEN_BASE		0x0000000010000000ULL
#define TOPCKGEN_MD1_CLK_MOD	0x00
#define MD1_CLK_MOD_BITS	(BIT(8) | BIT(9))

/* AOC sequencer (official md1_disable_sequencer_setting, md_gen >= 6298). */
#define SEQ_BASE		0x000000001c803000ULL
#define SEQ_SIZE		0x1000
#define SEQ_CFG			0x204
#define SEQ_STA			0x310
#define SEQ_STA_DONE		0x1010001U

/* MD clock request (official md_cd_srcclkena_setting; bypassed by
 * power_flow_config in official boots because LK already set it - a
 * cold boot like ours has to set it). */
#define INFRA_AO_MD_SRCCLKENA	0x0F0C
#define SRCCLKENA_MD1		0x21

/* SPM MTCMOS state (scp base = 0x1c001000; MD ctl 0xE00, pwr_sta 0xF34). */
#define SPM_BASE		0x000000001c001000ULL
#define SPM_MD_PWR_CTL		0xE00
#define SPM_PWR_STA		0xF34

/* Official ccifdriver@10209000 reg[0]/reg[1]. */
#define AP_CCIF_BASE		0x0000000010209000ULL
#define MD_CCIF_BASE		0x000000001020A000ULL
#define CCIF_BANK_SIZE		0x1000

/* APCCIF registers (hif/ccif_hif_reg.h). */
#define APCCIF_CON		0x00
#define APCCIF_BUSY		0x04
#define APCCIF_START		0x08
#define APCCIF_TCHNUM		0x0C
#define APCCIF_RCHNUM		0x10
#define APCCIF_ACK		0x14
#define APCCIF_IRQ0_MASK	0x20
#define APCCIF_IRQ1_MASK	0x24
#define APCCIF_CHDATA		0x100
#define CCIF_SRAM_DUMP_WORDS	16

/* GIC SPI 241/242 + 32; the +32 SPI offset was verified on the running
 * device (LVTS DT SPI 213/214 appear as Linux IRQ 245/246).
 */
#define CCIF_IRQ_DATA		(241 + 32)
#define CCIF_IRQ_EXCP		(242 + 32)

/* Ground truth from the device (HANDOFF §80.15). */
#define CCCI_TAG_MEM_BASE	0x00000000bdbf0000ULL
#define CCCI_TAG_MEM_SIZE	0x00010000U
#define CCCI_SMEM_BASE		0x000000008e000000ULL
#define CCCI_SMEM_MAX_SIZE	0x00120000U

/* Verbatim property bytes captured before the embedded DTB takes over. */
extern u8 xaga_ccci_lk_prop[64];
extern int xaga_ccci_lk_prop_len;
extern char xaga_ccci_lk_prop_name[32];

enum ccif_phase {
	CCIF_PHASE_IDLE,
	CCIF_PHASE_A_DONE,
	CCIF_PHASE_B_DONE,
	CCIF_PHASE_C_DONE,
};

static DEFINE_MUTEX(ccif_lock);
static int ccif_last_errno;

static void __iomem *infracfg_ao_map;
static void __iomem *ap_ccif_map;
static void __iomem *md_ccif_map;
static unsigned int ifrao1_prev, ifrao3_prev;
static bool clocks_on;
static int data_irq = -1, excp_irq = -1;
static bool trigger_a, trigger_b, trigger_c;

/* ---- helpers ------------------------------------------------------- */

static void ccif_read_bank(void __iomem *base, const char *what)
{
	unsigned int i;

	pr_info("CCI-CCIF: %s CON=0x%08x BUSY=0x%08x START=0x%08x TCHNUM=0x%08x RCHNUM=0x%08x\n",
		what, readl(base + APCCIF_CON), readl(base + APCCIF_BUSY),
		readl(base + APCCIF_START), readl(base + APCCIF_TCHNUM),
		readl(base + APCCIF_RCHNUM));
	pr_info("CCI-CCIF: %s IRQ0_MASK=0x%08x IRQ1_MASK=0x%08x\n", what,
		readl(base + APCCIF_IRQ0_MASK), readl(base + APCCIF_IRQ1_MASK));
	for (i = 0; i < CCIF_SRAM_DUMP_WORDS; i += 8) {
		char line[8 * 11];
		size_t used = 0;
		unsigned int k;

		for (k = i; k < i + 8; k++)
			used += scnprintf(line + used, sizeof(line) - used,
					  "%08x ",
					  readl(base + APCCIF_CHDATA +
						k * 4));
		pr_info("CCI-CCIF: %s SRAM+0x%02x: %s\n", what, i * 4, line);
	}
}

/* ---- phase C: tag walk for the nc_smem_info_ext overrides ---------- */

struct ccif_tag_view {
	const void *data;
	unsigned int size;
};

struct ccif_tag_ctx {
	const void *base;
	struct ccif_tag_view nc_ext;
	struct ccif_tag_view nc_ext_num;
};

static int ccif_collect_tag(const struct ccci_tag *tag, unsigned int offset,
			    void *ctx)
{
	struct ccif_tag_ctx *tc = ctx;
	struct ccif_tag_view *view = NULL;

	if (!strcmp(tag->tag_name, "nc_smem_info_ext"))
		view = &tc->nc_ext;
	else if (!strcmp(tag->tag_name, "nc_smem_info_ext_num"))
		view = &tc->nc_ext_num;

	if (view) {
		view->data = (const char *)tc->base + tag->data_offset;
		view->size = tag->data_size;
	}
	return 0;
}

/* Mirrors ccci_parse_tag_chain()'s walker semantics exactly. */
static int ccif_walk_tags(const void *buf, const struct ccci_tag_hdr *hdr,
			  int (*fn)(const struct ccci_tag *tag,
				    unsigned int offset, void *ctx), void *ctx)
{
	unsigned int offset = 0, i;

	for (i = 0; i < (unsigned int)hdr->tag_num; i++) {
		const struct ccci_tag *tag;
		unsigned int next;
		int ret;

		if (offset + sizeof(*tag) > hdr->size)
			return -EMSGSIZE;
		tag = buf + offset;
		if (!memchr(tag->tag_name, 0, sizeof(tag->tag_name)))
			return -EBADMSG;
		if (tag->data_offset > hdr->size ||
		    tag->data_size > hdr->size - tag->data_offset)
			return -ERANGE;
		ret = fn(tag, offset, ctx);
		if (ret)
			return ret;
		next = tag->next_tag_offset;
		if (i + 1 == (unsigned int)hdr->tag_num && !next)
			break;
		if (next <= offset || next - offset < sizeof(*tag))
			return -EBADMSG;
		if (next > hdr->size)
			return -ERANGE;
		offset = next;
	}
	return 0;
}

static int ccif_read_header(struct ccci_tag_hdr *hdr)
{
	if (!xaga_ccci_lk_prop_len)
		return -ENODEV;
	if (strcmp(xaga_ccci_lk_prop_name, "ccci,modem_info_v2"))
		return -EOPNOTSUPP;
	if (xaga_ccci_lk_prop_len < (int)sizeof(*hdr) ||
	    xaga_ccci_lk_prop_len > (int)sizeof(xaga_ccci_lk_prop))
		return -EMSGSIZE;
	memcpy(hdr, xaga_ccci_lk_prop, sizeof(*hdr));
	return ccci_validate_tag_hdr(hdr, CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE);
}

/* ---- one-shot trigger plumbing (ccci_probe pattern) ---------------- */

static enum ccif_phase ccif_done = CCIF_PHASE_IDLE;
static bool ccif_armed;
static bool md_powered;

/*
 * A phase advances only after its work reported success, so a failed
 * phase A (e.g. a gate that did not latch) can never be followed by a
 * register-hungry phase B. Arming is one-shot per module load: a failed
 * attempt also consumes it, exactly like ccci_probe.
 */
static int ccif_request(bool live, bool on, enum ccif_phase need)
{
	if (!on)
		return 0;
	if (!live)
		return -EAGAIN;
	if (ccif_armed)
		return -EALREADY;
	if (!md_powered)
		return -EPERM;
	if (ccif_done != need)
		return -EKEYREJECTED;
	ccif_armed = true;
	return 1;
}

static void ccif_work_fn(struct work_struct *work);
static DECLARE_WORK(ccif_work, ccif_work_fn);
static enum ccif_phase ccif_pending;

static int ccif_trigger_set(enum ccif_phase need, bool *arg, const char *val,
			    const struct kernel_param *kp)
{
	bool on;
	int ret;

	ret = kstrtobool(val, &on);
	if (ret)
		return ret;

	mutex_lock(&ccif_lock);
	ret = ccif_request(READ_ONCE(THIS_MODULE->state) == MODULE_STATE_LIVE,
			   on, need);
	if (ret > 0) {
		ret = 0;
		*arg = true;
		ccif_last_errno = -EINPROGRESS;
		ccif_pending = need + 1;
		if (!schedule_work(&ccif_work)) {
			ret = -EBUSY;
			ccif_last_errno = ret;
			ccif_armed = false;
		}
	}
	mutex_unlock(&ccif_lock);
	return ret;
}

static int ccif_trigger_a_set(const char *val, const struct kernel_param *kp)
{
	return ccif_trigger_set(CCIF_PHASE_IDLE, &trigger_a, val, kp);
}

static int ccif_trigger_b_set(const char *val, const struct kernel_param *kp)
{
	return ccif_trigger_set(CCIF_PHASE_A_DONE, &trigger_b, val, kp);
}

static int ccif_trigger_c_set(const char *val, const struct kernel_param *kp)
{
	return ccif_trigger_set(CCIF_PHASE_B_DONE, &trigger_c, val, kp);
}

static const struct kernel_param_ops ccif_ops_a = {
	.set = ccif_trigger_a_set, .get = param_get_bool,
};
static const struct kernel_param_ops ccif_ops_b = {
	.set = ccif_trigger_b_set, .get = param_get_bool,
};
static const struct kernel_param_ops ccif_ops_c = {
	.set = ccif_trigger_c_set, .get = param_get_bool,
};

module_param_cb(ccif_clk, &ccif_ops_a, &trigger_a, 0600);
MODULE_PARM_DESC(ccif_clk, "A: enable the 6 CCIF clock gates and read AP CCIF (one shot)");

module_param_cb(ccif_md, &ccif_ops_b, &trigger_b, 0600);
MODULE_PARM_DESC(ccif_md, "B: read the MD-side CCIF bank (requires A, one shot)");

module_param_cb(ccif_ring, &ccif_ops_c, &trigger_c, 0600);
MODULE_PARM_DESC(ccif_ring, "C: init CCISM ring buffers + register (masked) CCIF IRQs (requires B, one shot)");

/* ---- the IRQ handler: defensive, and unreachable while NO_AUTOEN --- */

static irqreturn_t ccif_isr(int irq, void *data)
{
	void __iomem *base = data;
	unsigned int ch;

	if (!base)
		return IRQ_NONE;
	ch = readl(base + APCCIF_RCHNUM);
	if (!ch)
		return IRQ_NONE;
	pr_info("CCI-CCIF: isr irq=%d RCHNUM=0x%x (masked; unexpected)\n", irq, ch);
	writel(ch, base + APCCIF_ACK);
	return IRQ_HANDLED;
}

/* ---- phase workers -------------------------------------------------- */

static int ccif_phase_a(void)
{
	unsigned int sta1, sta3, prot1, prot0, emi0, clkmod;
	void __iomem *topckgen_map = NULL, *seq_map = NULL;

	infracfg_ao_map = ioremap(INFRACFG_AO_BASE, INFRACFG_AO_SIZE);
	if (!infracfg_ao_map)
		return -ENOMEM;

	ifrao1_prev = readl(infracfg_ao_map + IFRAO1_STA);
	ifrao3_prev = readl(infracfg_ao_map + IFRAO3_STA);
	pr_info("CCI-CCIF: A: gates before: IFRAO1_STA=0x%08x IFRAO3_STA=0x%08x\n",
		ifrao1_prev, ifrao3_prev);

	writel(IFRAO1_CCIF_BITS, infracfg_ao_map + IFRAO1_SET);
	writel(IFRAO3_CCIF_BITS, infracfg_ao_map + IFRAO3_SET);

	sta1 = readl(infracfg_ao_map + IFRAO1_STA);
	sta3 = readl(infracfg_ao_map + IFRAO3_STA);
	pr_info("CCI-CCIF: A: gates after:  IFRAO1_STA=0x%08x IFRAO3_STA=0x%08x\n",
		sta1, sta3);
	if ((sta1 & IFRAO1_CCIF_BITS) != IFRAO1_CCIF_BITS ||
	    (sta3 & IFRAO3_CCIF_BITS) != IFRAO3_CCIF_BITS) {
		pr_err("CCI-CCIF: A: gate did not latch; restoring and aborting before any CCIF access\n");
		writel(IFRAO1_CCIF_BITS & ~ifrao1_prev,
		       infracfg_ao_map + IFRAO1_CLR);
		writel(IFRAO3_CCIF_BITS & ~ifrao3_prev,
		       infracfg_ao_map + IFRAO3_CLR);
		iounmap(infracfg_ao_map);
		infracfg_ao_map = NULL;
		return -EIO;
	}
	clocks_on = true;

	/*
	 * Forensics before anything MD-side: the 2026-09-13 wedge showed
	 * PWR_ACK on + gates latched is still not enough. Read the MD bus
	 * protections (the scpsys BUS_PROT_IGN entries never verify them),
	 * the SPM-visible topckgen md1_clk_mod, then apply the two
	 * remaining official pre-CCIF steps.
	 */
	prot1 = readl(infracfg_ao_map + IFRAO_PROT_INFRASYS1_STA);
	prot0 = readl(infracfg_ao_map + IFRAO_PROT_INFRASYS0_STA);
	emi0 = readl(infracfg_ao_map + IFRAO_PROT_EMISYS0_STA);
	pr_info("CCI-CCIF: A: MD bus prot sta: INFRASYS1=0x%08x INFRASYS0=0x%08x EMISYS0=0x%08x\n",
		prot1, prot0, emi0);

	topckgen_map = ioremap(TOPCKGEN_BASE, 0x100);
	if (!topckgen_map) {
		iounmap(infracfg_ao_map);
		infracfg_ao_map = NULL;
		return -ENOMEM;
	}
	clkmod = readl(topckgen_map + TOPCKGEN_MD1_CLK_MOD);
	pr_info("CCI-CCIF: A: md1_clk_mod before=0x%08x\n", clkmod);
	if (clkmod & MD1_CLK_MOD_BITS) {
		clkmod &= ~MD1_CLK_MOD_BITS;
		writel(clkmod, topckgen_map + TOPCKGEN_MD1_CLK_MOD);
		pr_info("CCI-CCIF: A: md1_clk_mod after=0x%08x\n",
			readl(topckgen_map + TOPCKGEN_MD1_CLK_MOD));
	}
	iounmap(topckgen_map);

	seq_map = ioremap(SEQ_BASE, SEQ_SIZE);
	if (!seq_map) {
		iounmap(infracfg_ao_map);
		infracfg_ao_map = NULL;
		return -ENOMEM;
	}
	pr_info("CCI-CCIF: A: sequencer cfg=0x%08x sta=0x%08x (before)\n",
		readl(seq_map + SEQ_CFG), readl(seq_map + SEQ_STA));
	writel(0, seq_map + SEQ_CFG);
	{
		unsigned int val = 0, waited = 0;

		while (readl(seq_map + SEQ_STA) != SEQ_STA_DONE &&
		       waited < 1000) {
			mdelay(1);
			waited++;
		}
		val = readl(seq_map + SEQ_STA);
		pr_info("CCI-CCIF: A: sequencer sta after %ums: 0x%08x (want 0x%08x)\n",
			waited, val, SEQ_STA_DONE);
		if (val != SEQ_STA_DONE)
			pr_warn("CCI-CCIF: A: sequencer did not reach DONE; continuing with evidence\n");
	}
	iounmap(seq_map);

	/*
	 * Cold-boot prerequisite: request the MD clock sources. Official
	 * bypasses this (power_flow_config bit0) because LK left it set;
	 * our genpd off/on cycle starts from nothing.
	 */
	{
		unsigned int srcclk = readl(infracfg_ao_map +
					    INFRA_AO_MD_SRCCLKENA);

		pr_info("CCI-CCIF: A: MD_SRCCLKENA before=0x%08x\n", srcclk);
		if ((srcclk & 0xFF) != SRCCLKENA_MD1) {
			srcclk = (srcclk & ~0xFFu) | SRCCLKENA_MD1;
			writel(srcclk, infracfg_ao_map +
				       INFRA_AO_MD_SRCCLKENA);
			pr_info("CCI-CCIF: A: MD_SRCCLKENA after=0x%08x\n",
				readl(infracfg_ao_map +
				      INFRA_AO_MD_SRCCLKENA));
		}
	}

	/* SPM-side evidence: MTCMOS switch and PWR_STA for MD. */
	{
		void __iomem *spm_map = ioremap(SPM_BASE, 0x1000);

		if (spm_map) {
			pr_info("CCI-CCIF: A: SPM MD_PWR_CTL=0x%08x PWR_STA&md=0x%08x\n",
				readl(spm_map + SPM_MD_PWR_CTL),
				readl(spm_map + SPM_PWR_STA) &
				(unsigned int)BIT(0));
			iounmap(spm_map);
		}
	}

	prot1 = readl(infracfg_ao_map + IFRAO_PROT_INFRASYS1_STA);
	prot0 = readl(infracfg_ao_map + IFRAO_PROT_INFRASYS0_STA);
	emi0 = readl(infracfg_ao_map + IFRAO_PROT_EMISYS0_STA);
	if ((prot1 & PROT_MASK_INFRASYS1_MD) ||
	    (prot0 & PROT_MASK_INFRASYS0_MD) ||
	    (emi0 & PROT_MASK_EMISYS0_MD)) {
		pr_err("CCI-CCIF: A: MD bus protections still engaged (INFRASYS1=0x%08x INFRASYS0=0x%08x EMISYS0=0x%08x); aborting before CCIF access\n",
			prot1, prot0, emi0);
		iounmap(infracfg_ao_map);
		infracfg_ao_map = NULL;
		return -EACCES;
	}
	pr_info("CCI-CCIF: A: MD bus protections clear; attempting CCIF read\n");

	ap_ccif_map = ioremap(AP_CCIF_BASE, CCIF_BANK_SIZE);
	if (!ap_ccif_map)
		return -ENOMEM;
	ccif_read_bank(ap_ccif_map, "AP_CCIF");
	pr_info("CCI-CCIF: A: done (clocks held on by module)\n");
	return 0;
}

static int ccif_phase_b(void)
{
	md_ccif_map = ioremap(MD_CCIF_BASE, CCIF_BANK_SIZE);
	if (!md_ccif_map)
		return -ENOMEM;
	ccif_read_bank(md_ccif_map, "MD_CCIF");
	pr_info("CCI-CCIF: B: done\n");
	return 0;
}

static int ccif_phase_c(void)
{
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_override ov[16];
	struct ccci_tag_result res = {};
	struct ccci_tag_hdr hdr = {};
	struct ccif_tag_ctx tc = {};
	size_t offsets[CCCI_CCIF_QUEUE_NUM];
	void *tag_map, *smem_map = NULL;
	unsigned int ov_num = 0, i;
	long n_used, x_used;
	int idx, ret;

	ret = ccif_read_header(&hdr);
	if (ret) {
		pr_err("CCI-CCIF: C: no valid tag header: %d\n", ret);
		return ret;
	}

	tag_map = memremap(CCCI_TAG_MEM_BASE, CCCI_TAG_MEM_SIZE, MEMREMAP_WB);
	if (!tag_map)
		return -ENOMEM;
	tc.base = tag_map;

	ret = ccci_parse_tag_chain(&hdr, tag_map, hdr.size, &res);
	if (ret) {
		pr_err("CCI-CCIF: C: tag parse failed: %d\n", ret);
		goto out;
	}
	ret = ccif_walk_tags(tag_map, &hdr, ccif_collect_tag, &tc);
	if (ret) {
		pr_err("CCI-CCIF: C: tag walk failed: %d\n", ret);
		goto out;
	}

	if (tc.nc_ext_num.data && tc.nc_ext_num.size >= sizeof(unsigned int))
		memcpy(&ov_num, tc.nc_ext_num.data, sizeof(ov_num));
	ov_num = min_t(unsigned int, ov_num, ARRAY_SIZE(ov));
	if (tc.nc_ext.size < ov_num * sizeof(ov[0]))
		ov_num = tc.nc_ext.size / sizeof(ov[0]);
	for (i = 0; i < ov_num; i++)
		memcpy(&ov[i], (const char *)tc.nc_ext.data + i * sizeof(ov[0]),
		       sizeof(ov[0]));

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, ov_num);
	if (ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, CCCI_SMEM_MAX_SIZE)) {
		pr_err("CCI-CCIF: C: built table exceeds SMEM; refusing\n");
		ret = -ERANGE;
		goto out;
	}

	if (res.smem.base_addr != CCCI_SMEM_BASE ||
	    !res.smem.total_smem_size ||
	    res.smem.total_smem_size > CCCI_SMEM_MAX_SIZE) {
		pr_err("CCI-CCIF: C: SMEM guard failed (base=0x%llx total=0x%x); refusing\n",
		       res.smem.base_addr, res.smem.total_smem_size);
		ret = -ERANGE;
		goto out;
	}

	smem_map = memremap(CCCI_SMEM_BASE, res.smem.total_smem_size,
			    MEMREMAP_WB);
	if (!smem_map) {
		ret = -ENOMEM;
		goto out;
	}

	idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM, SMEM_USER_CCISM_MCU);
	if (idx < 0 || !tbl[idx].size) {
		pr_err("CCI-CCIF: C: no CCISM_MCU region\n");
		ret = -ENODEV;
		goto out;
	}
	if (ccci_ccif_rb_fits(ccci_ccif_rx_up_98, ccci_ccif_tx_up_98,
			      tbl[idx].size)) {
		pr_err("CCI-CCIF: C: up_98 queues do not fit CCISM_MCU size=0x%x; refusing\n",
		       tbl[idx].size);
		ret = -ERANGE;
		goto out;
	}
	n_used = ccci_ccif_rb_region_fill((char *)smem_map + tbl[idx].offset,
					  tbl[idx].size, ccci_ccif_rx_up_98,
					  ccci_ccif_tx_up_98, offsets);
	if (n_used < 0) {
		ret = -EFAULT;
		goto out;
	}
	pr_info("CCI-CCIF: C: CCISM_MCU off=0x%x size=0x%x: %ld/0x%x bytes of ring buffers (q0 at +0x%zx, q15 at +0x%zx)\n",
		tbl[idx].offset, tbl[idx].size, n_used, tbl[idx].size,
		offsets[0], offsets[15]);

	idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
				    SMEM_USER_CCISM_MCU_EXP);
	if (idx < 0 || !tbl[idx].size) {
		pr_err("CCI-CCIF: C: no CCISM_MCU_EXP region\n");
		ret = -ENODEV;
		goto out;
	}
	if (ccci_ccif_rb_fits(ccci_ccif_rx_exp_up_95, ccci_ccif_tx_exp_up_95,
			      tbl[idx].size)) {
		pr_err("CCI-CCIF: C: up_95 exp queues do not fit CCISM_MCU_EXP size=0x%x; refusing\n",
		       tbl[idx].size);
		ret = -ERANGE;
		goto out;
	}
	x_used = ccci_ccif_rb_region_fill((char *)smem_map + tbl[idx].offset,
					  tbl[idx].size,
					  ccci_ccif_rx_exp_up_95,
					  ccci_ccif_tx_exp_up_95, offsets);
	if (x_used < 0) {
		ret = -EFAULT;
		goto out;
	}
	pr_info("CCI-CCIF: C: CCISM_MCU_EXP off=0x%x size=0x%x: %ld/0x%x bytes of ring buffers\n",
		tbl[idx].offset, tbl[idx].size, x_used, tbl[idx].size);

	/* Read back the first queue header of each region as a sanity check. */
	{
		unsigned int magic;

		idx = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					    SMEM_USER_CCISM_MCU);
		memcpy(&magic, (char *)smem_map + tbl[idx].offset, 4);
		if (magic != CCCI_RBF_HEADER) {
			pr_err("CCI-CCIF: C: readback magic mismatch 0x%08x\n",
			       magic);
			ret = -EIO;
			goto out;
		}
		pr_info("CCI-CCIF: C: readback CCISM_MCU q0 header magic 0x%08x OK\n",
			magic);
	}

	/* IRQs registered but left disabled (IRQF_NO_AUTOEN): the handler
	 * cannot fire until a later, explicit enable. */
	ret = request_irq(CCIF_IRQ_DATA, ccif_isr, IRQF_NO_AUTOEN,
			  "ccif_data", ap_ccif_map);
	if (ret)
		pr_err("CCI-CCIF: C: request_irq(%d) failed: %d\n",
		       CCIF_IRQ_DATA, ret);
	else
		data_irq = CCIF_IRQ_DATA;
	ret = request_irq(CCIF_IRQ_EXCP, ccif_isr, IRQF_NO_AUTOEN,
			  "ccif_excp", ap_ccif_map);
	if (ret)
		pr_err("CCI-CCIF: C: request_irq(%d) failed: %d\n",
		       CCIF_IRQ_EXCP, ret);
	else
		excp_irq = CCIF_IRQ_EXCP;

	pr_info("CCI-CCIF: C: done (IRQs registered %s%s, both masked)\n",
		data_irq > 0 ? "data " : "", excp_irq > 0 ? "excp" : "");
	ret = 0;

out:
	if (smem_map)
		memunmap(smem_map);
	memunmap(tag_map);
	return ret;
}

static void ccif_work_fn(struct work_struct *work)
{
	int ret = 0;

	mutex_lock(&ccif_lock);
	ccif_last_errno = -EINPROGRESS;
	mutex_unlock(&ccif_lock);

	switch (ccif_pending) {
	case CCIF_PHASE_A_DONE:
		ret = ccif_phase_a();
		break;
	case CCIF_PHASE_B_DONE:
		ret = ccif_phase_b();
		break;
	case CCIF_PHASE_C_DONE:
		ret = ccif_phase_c();
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_lock(&ccif_lock);
	ccif_last_errno = ret;
	if (!ret)
		ccif_done = ccif_pending;
	mutex_unlock(&ccif_lock);
}

/* ---- status / module glue ------------------------------------------ */

static int ccif_status_get(char *buffer, const struct kernel_param *kp)
{
	static const char * const names[] = {
		"idle", "A:clk+AP-read", "B:+MD-read", "C:+ringbuf+IRQ",
	};
	int len;

	mutex_lock(&ccif_lock);
	len = scnprintf(buffer, PAGE_SIZE, "%s errno=%d clocks=%s armed=%d md=%s\n",
			names[ccif_done], ccif_last_errno,
			clocks_on ? "on" : "off", ccif_armed,
			md_powered ? "on" : "off");
	mutex_unlock(&ccif_lock);
	return len;
}

static const struct kernel_param_ops ccif_status_ops = {
	.get = ccif_status_get,
};
module_param_cb(status, &ccif_status_ops, NULL, 0400);
MODULE_PARM_DESC(status, "phase, last errno, clock state");

/*
 * Probe takes the MD MTCMOS reference (the missing prerequisite found on
 * 2026-09-13: the CCIF banks hang the bus with the genpd "md" domain off).
 * With no mddriver node the module still loads idle and every trigger
 * fails closed with -EPERM.
 */
static int ccci_ccif_probe(struct platform_device *pdev)
{
	int ret;

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		dev_err(&pdev->dev, "MD power domain get failed: %d\n", ret);
		return ret;
	}

	mutex_lock(&ccif_lock);
	md_powered = true;
	mutex_unlock(&ccif_lock);
	dev_info(&pdev->dev, "MD power domain on; CCIF prerequisites ready\n");
	return 0;
}

static void ccci_ccif_remove(struct platform_device *pdev)
{
	mutex_lock(&ccif_lock);
	md_powered = false;
	mutex_unlock(&ccif_lock);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	dev_info(&pdev->dev, "MD power domain released\n");
}

static const struct of_device_id ccci_ccif_of_match[] = {
	{ .compatible = "mediatek,mddriver" },
	{ }
};
MODULE_DEVICE_TABLE(of, ccci_ccif_of_match);

static struct platform_driver ccci_ccif_driver = {
	.probe = ccci_ccif_probe,
	.remove = ccci_ccif_remove,
	.driver = {
		.name = "ccci_ccif",
		.of_match_table = ccci_ccif_of_match,
	},
};

static int __init ccif_mod_init(void)
{
	int ret;

	ret = platform_driver_register(&ccci_ccif_driver);
	if (ret) {
		pr_err("CCI-CCIF: driver register failed: %d\n", ret);
		return ret;
	}
	pr_info("CCI-CCIF: loaded; waiting for mddriver probe (MD power domain)\n");
	return 0;
}
module_init(ccif_mod_init);

static void __exit ccif_mod_exit(void)
{
	platform_driver_unregister(&ccci_ccif_driver);
	cancel_work_sync(&ccif_work);
	if (data_irq > 0)
		free_irq(data_irq, ap_ccif_map);
	if (excp_irq > 0)
		free_irq(excp_irq, ap_ccif_map);
	if (md_ccif_map)
		iounmap(md_ccif_map);
	if (ap_ccif_map)
		iounmap(ap_ccif_map);
	if (infracfg_ao_map) {
		if (clocks_on) {
			/* Restore only the bits we turned on ourselves. */
			writel(IFRAO1_CCIF_BITS & ~ifrao1_prev,
			       infracfg_ao_map + IFRAO1_CLR);
			writel(IFRAO3_CCIF_BITS & ~ifrao3_prev,
			       infracfg_ao_map + IFRAO3_CLR);
			pr_info("CCI-CCIF: exit: gates restored (IFRAO1 back to 0x%08x, IFRAO3 back to 0x%08x)\n",
				ifrao1_prev, ifrao3_prev);
		}
		iounmap(infracfg_ao_map);
	}
	pr_info("CCI-CCIF: module unloaded\n");
}
module_exit(ccif_mod_exit);

MODULE_DESCRIPTION("MediaTek CCI CCIF staged bring-up diagnostic (qqcandy)");
MODULE_LICENSE("GPL");
