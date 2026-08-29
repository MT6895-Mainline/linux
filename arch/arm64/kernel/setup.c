// SPDX-License-Identifier: GPL-2.0-only
/*
 * Based on arch/arm/kernel/setup.c
 *
 * Copyright (C) 1995-2001 Russell King
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/acpi.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/cache.h>
#include <linux/screen_info.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/panic_notifier.h>
#include <linux/proc_fs.h>
#include <linux/memblock.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>
#include <linux/efi.h>
#include <linux/psci.h>
#include <linux/sched/task.h>
#include <linux/scs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/xaga_marker.h>

#include <asm/acpi.h>
#include <asm/fixmap.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/daifflags.h>
#include <asm/elf.h>
#include <asm/cpufeature.h>
#include <asm/cpu_ops.h>
#include <asm/kasan.h>
#include <asm/numa.h>
#include <asm/rsi.h>
#include <asm/scs.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/traps.h>
#include <asm/efi.h>
#include <asm/xen/hypervisor.h>
#include <asm/mmu_context.h>

/*
 * XAGA boot-stage markers (legacy wrapper): the printk mirror now lives in
 * drivers/misc/xaga-marker-writer.c, armed at the head of setup_arch into the
 * log_store region (0x7ffbf000), which LK's PL_LOG_STORE restores into the
 * expdb partition on the next boot. These wrappers keep the init/main.c stage
 * calls compiling; they just forward into the new ring writer.
 */
void xaga_word_stage(u32 stage);
void xaga_stage(int stage);

void xaga_word_stage(u32 stage)
{
	xaga_marker_stage(stage);
}

void xaga_stage(int stage)
{
	xaga_marker_stage(stage);
}

/*
 * XAGA GPU MTCMOS bring-up hack.
 *
 * LK leaves the MT6895 GPU TOP domain (MFG1) powered on, so panthor can read
 * GPU_ID/features, but the shader-core sub-domains MFG2..MFG12 are OFF ->
 * shader_present=0x0 and panthor fails. The proper fix is a full scpsys/pm-
 * domains port; this pokes the PWR_CON registers directly (sequence taken
 * from the downstream mtk-scpsys driver) to power on MFG1..MFG12 before
 * panthor probes (postcore_initcall < device_initcall).
 *
 * scpsys base 0x1c001000 (live DT "mediatek,mt6895-scpsys"). PWR_CON offsets:
 * MFG1=0xEBC, MFG2..MFG12=0xEC0..0xEE8. Bits: PWR_ON=BIT2, PWR_ON_2ND=BIT3,
 * PWR_CLK_DIS=BIT4, PWR_ISO=BIT1, PWR_RST_B=BIT0, SRAM_PDN=BIT8/ACK=BIT12,
 * status = bits 31:30 (domain ON when both set).
 */
#define XAGA_SCPSYS_PA	0x1c001000UL
#define XAGA_SCPSYS_SZ	0x1000

static int __init __maybe_unused xaga_gpu_power_on(void)
{
	static const u32 mfg_offs[] = {
		0xEBC, 0xEC0, 0xEC4, 0xEC8, 0xECC,
		0xED0, 0xED4, 0xED8, 0xEDC, 0xEE0, 0xEE4, 0xEE8,
	};
	void __iomem *scpsys;
	u32 sta = GENMASK(31, 30);
	int i;

	scpsys = ioremap(XAGA_SCPSYS_PA, XAGA_SCPSYS_SZ);
	if (!scpsys) {
		pr_err("XAGA-GPU: scpsys ioremap failed\n");
		return -ENOMEM;
	}

	pr_info("XAGA-GPU: PWR_STA@F34=%#x PWR_STA2@F38=%#x\n",
		readl(scpsys + 0xF34), readl(scpsys + 0xF38));

	/* MFG clock bring-up (correct regs, mt6895 clk-mt6895.c):
	 *  mfgpll_pll_ctrl @0x13fa0000, mfgscpll_pll_ctrl @0x13fa0c00
	 *    CON0=+0x008 bit0 EN bit31 LOCK; CON1=+0x00C bit24 PD [21:0] PCW;
	 *    CON3=+0x014 bit0 PWR/RST_B
	 *  topckgen CLK_CFG_30 @0x10000000+0x1f0 (SET +0x1f4, CLR +0x1f8):
	 *    bit16 mfg_sel0 (0=ref 1=mfgpll), bit17 mfg_sel1 (0=ref 1=mfgscpll)
	 */
	{
		void __iomem *top = ioremap(0x10000000, 0x1000);
		void __iomem *mfgpll = ioremap(0x13fa0000, 0x400);
		void __iomem *mfgsc = ioremap(0x13fa0c00, 0x400);
		void __iomem *plls[] = { mfgpll, mfgsc };
		const char *names[] = { "MFGPLL", "MFGSCPLL" };
		u32 v;
		int p;

		if (top && mfgpll && mfgsc) {
			pr_info("XAGA-GPU: CLK_CFG30=%#x\n", readl(top + 0x1f0));
			for (p = 0; p < 2; p++) {
				pr_info("XAGA-GPU: %s before CON0=%#x CON1=%#x CON3=%#x\n",
					names[p], readl(plls[p] + 0x008),
					readl(plls[p] + 0x00C), readl(plls[p] + 0x014));
				/* PWR on (CON3 bit0), ISO off (CON3 bit1) - mtk_pll_prepare */
				v = readl(plls[p] + 0x014);
				writel((v | BIT(0)) & ~BIT(1), plls[p] + 0x014);
				udelay(2);
				/* EN on (CON0 bit0) */
				v = readl(plls[p] + 0x008);
				writel(v | BIT(0), plls[p] + 0x008);
				/* PD off (CON1 bit24), leave PCW as LK configured */
				v = readl(plls[p] + 0x00C);
				writel(v & ~BIT(24), plls[p] + 0x00C);
				udelay(200);
				pr_info("XAGA-GPU: %s after  CON0=%#x CON1=%#x CON3=%#x\n",
					names[p], readl(plls[p] + 0x008),
					readl(plls[p] + 0x00C), readl(plls[p] + 0x014));
			}
			/* select mfgpll/mfgscpll on the mux (bit16/bit17) */
			writel(BIT(16) | BIT(17), top + 0x1f8);	/* CLR */
			writel(BIT(16) | BIT(17), top + 0x1f4);	/* SET */
			pr_info("XAGA-GPU: CLK_CFG30 after=%#x\n", readl(top + 0x1f0));

			/* open mfgcfg BG3D gate (0x13fbf000: SET +0x4, CLR +0x8, STA +0x0) */
			{
				void __iomem *mfgcg = ioremap(0x13fbf000, 0x1000);
				if (mfgcg) {
					writel(BIT(0), mfgcg + 0x4);	/* gate on */
					pr_info("XAGA-GPU: MFGCFG STA=%#x\n",
						readl(mfgcg + 0x0));
					iounmap(mfgcg);
				}
			}
		} else {
			pr_err("XAGA-GPU: clk ioremap failed\n");
		}
		if (top)
			iounmap(top);
		if (mfgpll)
			iounmap(mfgpll);
		if (mfgsc)
			iounmap(mfgsc);
	}

	for (i = 0; i < ARRAY_SIZE(mfg_offs); i++) {
		void __iomem *ctl = scpsys + mfg_offs[i];
		u32 val;
		int tmo;

		val = readl(ctl);
		if ((val & sta) == sta) {
			pr_info("XAGA-GPU: mfg%d already on (%#x)\n", i + 1, val);
			continue;
		}

		/* Release SRAM power-down first (ack bit 12 -> 0). */
		val &= ~BIT(8);
		writel(val, ctl);
		tmo = 100000;
		while (tmo-- && (readl(ctl) & BIT(12)))
			udelay(1);
		pr_info("XAGA-GPU: mfg%d after sram-rel reg=%#x sram_ack=%d\n",
			i + 1, readl(ctl), !!(readl(ctl) & BIT(12)));

		/* MTCMOS power-on */
		val = readl(ctl);
		val |= BIT(2);
		writel(val, ctl);
		val |= BIT(3);
		writel(val, ctl);

		tmo = 100000;
		while (tmo-- && (readl(ctl) & sta) != sta)
			udelay(1);
		if (tmo < 0) {
			pr_err("XAGA-GPU: mfg%d PWR_CON timeout (reg=%#x) PWR_STA=%#x\n",
			       i + 1, readl(ctl), readl(scpsys + 0xF34));
			continue;
		}

		udelay(100);
		val = readl(ctl);
		val &= ~BIT(4);			/* PWR_CLK_DIS */
		writel(val, ctl);
		val &= ~BIT(1);			/* PWR_ISO */
		writel(val, ctl);
		val |= BIT(0);			/* PWR_RST_B */
		writel(val, ctl);

		pr_info("XAGA-GPU: mfg%d powered on (reg=%#x PWR_STA=%#x)\n",
			i + 1, readl(ctl), readl(scpsys + 0xF34));
	}

	pr_info("XAGA-GPU: done, PWR_STA=%#x\n", readl(scpsys + 0xF34));
	iounmap(scpsys);
	return 0;
}

/*
 * DISABLED (2026-08-09): the direct MTCMOS/PLL poke is a dead end — GPU_ID
 * reads 0x0 regardless (GPU fully unpowered), and once console=ttyGS0 became
 * active (cmdline re-read) the initcall's MMIO poke/pr_info stall at boot.
 * GPU bring-up needs the proper mtk-scpsys + mt6895 clock driver ports (§13).
 * Keep the function above as reference; do not re-enable via blind pokes.
 */
/* postcore_initcall(xaga_gpu_power_on); */

/*
 * XAGA i2c5 clock + pinmux enable.
 *
 * The MT6375 PMIC (charger/gauge/tcpc) lives on i2c5 (0x11280000). Its clocks
 * are fixed-clock stubs in the DTS (no mt6895 clock driver), so the clock
 * framework cannot touch the real hardware gates. LK does NOT drive i2c5 (the
 * gate was observed OFF at boot), so we must bring it up entirely ourselves:
 *   - imp_iic_wrap_c CG @0x11282000: STA +0xE00, CLR +0xE04, SET +0xE08,
 *     bit0 = i2c5 (CLK_IMPC_AP_CLOCK_I2C5, parent i2c_ck)
 *   - pericfg_ao perao1 @0x11036000+0x40: bit5 = CLK_PERAOP_1_DMA_BCLK (apdma)
 *   - i2c_sel mux (topckgen CLK_CFG_11 @0x100000c0 bits 8-9) boots at parent 2
 *     (univpll_d5_d4); force parent 0 (tck_26m = 26MHz, FACTOR 1:1) to match
 *     the fixed-clock stub, then latch with CLK_CFG_UPDATE1 @0x10000008 bit14.
 *   - i2c5 pins: GPIO33=SCL5, GPIO34=SDA5 (mode 1). Mode field is a 4-bit
 *     RMW field; pins 32-39 live at gpio base 0x10005000 + 0x0340 + pin*0x10
 *     (GPIO33 -> 0x10005550, GPIO34 -> 0x10005560), bits 3:0.
 */
#define XAGA_IMPC_PA	0x11282000UL
#define XAGA_PERI_PA	0x11036000UL
#define XAGA_TOP_PA	0x10000000UL
#define XAGA_GPIO_PA	0x10005000UL

static int __init xaga_i2c_power_on(void)
{
	void __iomem *impc = ioremap(XAGA_IMPC_PA, 0x1000);
	void __iomem *peri = ioremap(XAGA_PERI_PA, 0x1000);
	void __iomem *top = ioremap(XAGA_TOP_PA, 0x1000);
	void __iomem *gpio = ioremap(XAGA_GPIO_PA, 0x1000);
	u32 v;

	if (impc) {
		pr_info("XAGA-I2C: impc CG STA=%#x\n", readl(impc + 0xE00));
		/*
		 * mtk_clk_gate_ops_setclr is INVERTED: enable = clear the bit
		 * (mtk_cg_clr_bit -> CLR reg), disable = set the bit (SET reg),
		 * is_enabled = bit cleared. STA=0 at boot already meant ON.
		 * Writing SET (0xE08) as we did before actually CLOSED the gate
		 * and left the controller unclocked (all regs read 0). Use CLR.
		 * bit0 = i2c5, bit1 = i2c6 (both in imp_iic_wrap_c).
		 */
		writel(BIT(0) | BIT(1), impc + 0xE04);	/* CLR -> ENABLE i2c5/i2c6 */
		pr_info("XAGA-I2C: impc CG STA after=%#x (want bit0/bit1=0 = enabled)\n",
			readl(impc + 0xE00));
		iounmap(impc);
	} else {
		pr_err("XAGA-I2C: impc ioremap failed\n");
	}

	/* i2c7 gate: imp_iic_wrap_s @0x11d07000, CLK_IMPS_AP_CLOCK_I2C7 = bit 4 */
	{
		void __iomem *imps = ioremap(0x11d07000, 0x1000);
		if (imps) {
			pr_info("XAGA-I2C: imps CG STA=%#x\n", readl(imps + 0xE00));
			writel(BIT(4), imps + 0xE04);	/* CLR -> ENABLE i2c7 */
			pr_info("XAGA-I2C: imps CG STA after=%#x (want bit4=0)\n",
				readl(imps + 0xE00));
			iounmap(imps);
		} else {
			pr_err("XAGA-I2C: imps ioremap failed\n");
		}
	}

	/* i2c1 gate: imp_iic_wrap_s @0x11d07000, CLK_IMPS_AP_CLOCK_I2C1 = bit 0 */
	{
		void __iomem *imps = ioremap(0x11d07000, 0x1000);
		if (imps) {
			pr_info("XAGA-I2C: i2c1 imps CG STA=%#x\n",
				readl(imps + 0xE00));
			writel(BIT(0), imps + 0xE04);	/* CLR -> ENABLE i2c1 */
			pr_info("XAGA-I2C: i2c1 imps CG STA after=%#x (want bit0=0)\n",
				readl(imps + 0xE00));
			iounmap(imps);
		} else {
			pr_err("XAGA-I2C: i2c1 imps ioremap failed\n");
		}
	}

	if (peri) {
		v = readl(peri + 0x40);
		pr_info("XAGA-I2C: peri DMA gate=%#x\n", v);
		iounmap(peri);
	} else {
		pr_err("XAGA-I2C: peri ioremap failed\n");
	}

	if (top) {
		v = readl(top + 0xC0);
		pr_info("XAGA-I2C: CLK_CFG_11=%#x i2c_sel=%u\n",
			v, (v >> 8) & 0x3);
		if (((v >> 8) & 0x3) != 0) {
			/* select parent 0 (tck_26m = 26MHz) + latch */
			writel(0x300, top + 0xC8);	/* CLK_CFG_11_CLR */
			writel(BIT(14), top + 0x08);	/* CLK_CFG_UPDATE1 */
			pr_info("XAGA-I2C: CLK_CFG_11 after=%#x i2c_sel=%u\n",
				readl(top + 0xC0), (readl(top + 0xC0) >> 8) & 0x3);
		}
		iounmap(top);
	} else {
		pr_err("XAGA-I2C: topckgen ioremap failed\n");
	}

	if (gpio) {
		/*
		 * GPIO33 = SCL5, GPIO34 = SDA5, both mode 1. Mode field for
		 * pins 32-39: s_addr 0x0340, x_addrs 0x10 (register stride,
		 * 8 pins/reg at 4 bits each), s_bit 0, x_bits 4.
		 *   bits = (pin-32)*4; offset = 0x340 + 0x10*(bits/32);
		 *   bitpos = bits % 32
		 * GPIO33: offset 0x340, bits 7:4. GPIO34: offset 0x340,
		 * bits 11:8. RMW (no set/clr aliases).
		 */
		v = readl(gpio + 0x340);
		writel((v & ~0xf0) | (1 << 4), gpio + 0x340);	/* GPIO33=SCL5 */
		v = readl(gpio + 0x340);
		writel((v & ~0xf00) | (1 << 8), gpio + 0x340);	/* GPIO34=SDA5 */
		pr_info("XAGA-I2C: gpio33/34 mode reg=%#x (want bits7:4=1,bits11:8=1)\n",
			readl(gpio + 0x340));
		/*
		 * i2c7: GPIO29=SCL7, GPIO30=SDA7, mode 1.
		 *   pins 24-31 -> s_addr 0x330; bits=(pin-24)*4
		 *   pin 29: bits 20..23, pin 30: bits 24..27
		 */
		v = readl(gpio + 0x330);
		writel((v & ~0x00f00000) | (1 << 20), gpio + 0x330);	/* GPIO29=SCL7 */
		v = readl(gpio + 0x330);
		writel((v & ~0x0f000000) | (1 << 24), gpio + 0x330);	/* GPIO30=SDA7 */
		pr_info("XAGA-I2C: gpio29/30 mode reg=%#x\n", readl(gpio + 0x330));
		/*
		 * i2c1: GPIO8=SCL1, GPIO9=SDA1, mode 1.
		 *   pins 8-15 -> s_addr 0x310; bits=(pin-8)*4
		 *   pin 8: bits 0..3, pin 9: bits 4..7
		 */
		v = readl(gpio + 0x310);
		writel((v & ~0x00f) | (1 << 0), gpio + 0x310);		/* GPIO8=SCL1 */
		v = readl(gpio + 0x310);
		writel((v & ~0x0f0) | (1 << 4), gpio + 0x310);		/* GPIO9=SDA1 */
		pr_info("XAGA-I2C: gpio8/9 mode reg=%#x\n", readl(gpio + 0x310));
		/*
		 * i2c6: GPIO31=SCL6, GPIO32=SDA6, mode 1.
		 *   pin 31: pins 24-31 -> s_addr 0x330, bits=(31-24)*4=28
		 *   pin 32: pins 32-39 -> s_addr 0x340, bits=(32-32)*4=0
		 */
		v = readl(gpio + 0x330);
		writel((v & ~(0xfUL << 28)) | (1UL << 28), gpio + 0x330); /* GPIO31=SCL6 */
		v = readl(gpio + 0x340);
		writel((v & ~0xfUL) | 1UL, gpio + 0x340);		/* GPIO32=SDA6 */
		pr_info("XAGA-I2C: gpio31/32 mode reg=%#x/%#x\n",
			readl(gpio + 0x330), readl(gpio + 0x340));
		iounmap(gpio);
	} else {
		pr_err("XAGA-I2C: gpio ioremap failed\n");
	}

	/*
	 * i2c7 pins (GPIO29/30) live in iocfg_br (i_base 6 = 0x11d40000):
	 *   IES: +0x70 bits 2,7   PU: +0x90 bits 2,7   PD: +0x80 bits 2,7
	 */
	{
		void __iomem *br = ioremap(0x11d40000, 0x1000);
		if (br) {
			v = readl(br + 0x70);
			writel(v | BIT(2) | BIT(7), br + 0x70);		/* IES */
			v = readl(br + 0x90);
			writel(v | BIT(2) | BIT(7), br + 0x90);		/* PU */
			v = readl(br + 0x80);
			writel(v & ~(BIT(2) | BIT(7)), br + 0x80);	/* PD off */
			pr_info("XAGA-I2C: i2c7 pins IES=%#x PU=%#x PD=%#x\n",
				readl(br + 0x70), readl(br + 0x90), readl(br + 0x80));
			iounmap(br);
		}
	}

	/*
	 * i2c6 pins (GPIO31/32) live in iocfg_lt (i_base 13 = 0x11f30000):
	 *   pin31: IES +0x60 bit8, SMT +0xe0 bit8, PU +0x90 bit5, PD +0x70 bit5
	 *   pin32: IES +0x60 bit12, SMT +0xe0 bit10, PU +0x90 bit9, PD +0x70 bit9
	 */
	{
		void __iomem *lt = ioremap(0x11f30000, 0x1000);
		if (lt) {
			v = readl(lt + 0x60);
			writel(v | BIT(8) | BIT(12), lt + 0x60);	/* IES */
			v = readl(lt + 0xe0);
			writel(v | BIT(8) | BIT(10), lt + 0xe0);	/* SMT */
			v = readl(lt + 0x90);
			writel(v | BIT(5) | BIT(9), lt + 0x90);		/* PU */
			v = readl(lt + 0x70);
			writel(v & ~(BIT(5) | BIT(9)), lt + 0x70);	/* PD off */
			pr_info("XAGA-I2C: i2c6 pins IES=%#x SMT=%#x PU=%#x PD=%#x\n",
				readl(lt + 0x60), readl(lt + 0xe0),
				readl(lt + 0x90), readl(lt + 0x70));
			iounmap(lt);
		}
	}

	/*
	 * Pull-up SCL5/SDA5 in case the board has no external pull resistors
	 * (an unpulled/held-low SDA makes every transfer time out). PU/PD are
	 * 1-bit fields in the pin's iocfg block (i_base from pinctrl DT regs):
	 *   pin 33: PU @iocfg_rmm(0x11c40000)+0x60 bit15, PD @+0x40 bit15
	 *   pin 34: PU @iocfg_rt (0x11c30000)+0xb0 bit1,  PD @+0x90 bit1
	 * Pull-up = PU=1, PD=0.
	 * ALSO enable the input buffers (IES, required for i2c — otherwise the
	 * controller's SDA_IN/SCL_IN read 0 and it thinks the bus is forever
	 * busy) and the schmitt trigger (SMT):
	 *   pin 33: IES @0x11c40000+0x30 bit19, SMT @0x11c40000+0xb0 bit14
	 *   pin 34: IES @0x11c30000+0x70 bit1,  SMT @0x11c30000+0x110 bit0
	 */
	{
		void __iomem *iocfg;
		iocfg = ioremap(0x11c40000, 0x1000);	/* iocfg_rmm, pin 33 */
		if (iocfg) {
			v = readl(iocfg + 0x60);
			writel(v | BIT(15), iocfg + 0x60);
			v = readl(iocfg + 0x40);
			writel(v & ~BIT(15), iocfg + 0x40);
			v = readl(iocfg + 0x30);
			writel(v | BIT(19), iocfg + 0x30);	/* IES */
			v = readl(iocfg + 0xb0);
			writel(v | BIT(14), iocfg + 0xb0);	/* SMT */
			pr_info("XAGA-I2C: pin33 PU=%#x PD=%#x IES=%#x SMT=%#x\n",
				readl(iocfg + 0x60), readl(iocfg + 0x40),
				readl(iocfg + 0x30), readl(iocfg + 0xb0));
			iounmap(iocfg);
		}
		iocfg = ioremap(0x11c30000, 0x1000);	/* iocfg_rt, pin 34 */
		if (iocfg) {
			v = readl(iocfg + 0xb0);
			writel(v | BIT(1), iocfg + 0xb0);
			v = readl(iocfg + 0x90);
			writel(v & ~BIT(1), iocfg + 0x90);
			v = readl(iocfg + 0x70);
			writel(v | BIT(1), iocfg + 0x70);	/* IES */
			v = readl(iocfg + 0x110);
			writel(v | BIT(0), iocfg + 0x110);	/* SMT */
			pr_info("XAGA-I2C: pin34 PU=%#x PD=%#x IES=%#x SMT=%#x\n",
				readl(iocfg + 0xb0), readl(iocfg + 0x90),
				readl(iocfg + 0x70), readl(iocfg + 0x110));
			iounmap(iocfg);
		}
	}

	/*
	 * i2c1 pins (GPIO8=SCL1, GPIO9=SDA1) live in iocfg_br (i_base 6 =
	 * 0x11d40000), per pinctrl-mt6895.c ies/smt/pu/pd ranges:
	 *   IES: +0x70 bits 18/20   SMT: +0xd0 bits 17/19
	 *   PU:  +0x90 bits 18/20   PD:  +0x80 bits 18/20
	 */
	{
		void __iomem *br = ioremap(0x11d40000, 0x1000);
		if (br) {
			v = readl(br + 0x70);
			writel(v | BIT(18) | BIT(20), br + 0x70);	/* IES */
			v = readl(br + 0xd0);
			writel(v | BIT(17) | BIT(19), br + 0xd0);	/* SMT */
			v = readl(br + 0x90);
			writel(v | BIT(18) | BIT(20), br + 0x90);	/* PU */
			v = readl(br + 0x80);
			writel(v & ~(BIT(18) | BIT(20)), br + 0x80);	/* PD off */
			pr_info("XAGA-I2C: i2c1 pins IES=%#x SMT=%#x PU=%#x PD=%#x\n",
				readl(br + 0x70), readl(br + 0xd0),
				readl(br + 0x90), readl(br + 0x80));
			iounmap(br);
		}
	}

	/*
	 * SPI2 (touchscreen NT36672E) clock + pin setup.
	 *   - SPI2_BCLK gate: perao0 @0x11036000+0x3c bit 19, active-low
	 *     (enable = clear bit, mtk_clk_gate_ops_no_setclr).
	 *   - SPI_SEL mux: topckgen CLK_CFG_7 @0x10000080 bits 16-18, set
	 *     parent 0 (tck_26m = 26MHz) to match the fixed-clock stub; latch
	 *     via CLK_CFG_UPDATE @0x10000004 bit 30 (TOP_MUX_SPI_SHIFT).
	 *   - SPI2 pins GPIO109(MI)/110(CSB)/111(MO)/112(CLK), mode 1:
	 *     pin 109-111 -> reg 0x430, pin 112 -> reg 0x440; field bits
	 *     (pin%8)*4 (109=20,110=24,111=28,112=0).
	 *   - INT pin GPIO135 mode 0 (GPIO), reg 0x400 bits 28..31.
	 */
	{
		void __iomem *peri = ioremap(0x11036000, 0x1000);
		if (peri) {
			v = readl(peri + 0x3c);
			pr_info("XAGA-SPI: perao0=%#x spi2 gate=%u (want 0=enabled)\n",
				v, (v >> 19) & 1);
			writel(v & ~BIT(19), peri + 0x3c);
			pr_info("XAGA-SPI: perao0 after=%#x\n", readl(peri + 0x3c));
			iounmap(peri);
		}
	}
	{
		void __iomem *top = ioremap(0x10000000, 0x1000);
		if (top) {
			v = readl(top + 0x80);
			pr_info("XAGA-SPI: CLK_CFG_7=%#x spi_sel=%u (want 0=26M)\n",
				v, (v >> 16) & 0x7);
			writel(0x70000, top + 0x88);	/* CLR bits 16-18 */
			writel(0, top + 0x84);		/* SET parent 0 */
			writel(BIT(30), top + 0x04);	/* CLK_CFG_UPDATE latch */
			pr_info("XAGA-SPI: CLK_CFG_7 after=%#x\n",
				readl(top + 0x80));
			iounmap(top);
		}
	}
	{
		void __iomem *gpio = ioremap(0x10005000, 0x1000);
		if (gpio) {
			/*
			 * mode s_addr = 0x300 + 0x10*(N/8), bit = (N%8)*4
			 * GPIO109/110/111 -> reg 0x3d0; GPIO112 -> 0x3e0.
			 */
			v = readl(gpio + 0x3d0);
			writel((v & ~0xfff00000) | (1 << 20) | (1 << 24) | (1 << 28),
			       gpio + 0x3d0);
			pr_info("XAGA-SPI: gpio109-111 mode reg=%#x (want bits 23:20,27:24,31:28 = 1)\n",
				readl(gpio + 0x3d0));
			/* GPIO112=CLK -> reg 0x3e0, bits 3:0 */
			v = readl(gpio + 0x3e0);
			writel((v & ~0xf) | 1, gpio + 0x3e0);
			pr_info("XAGA-SPI: gpio112 mode reg=%#x (want bit0=1)\n",
				readl(gpio + 0x3e0));
			/* GPIO135=INT -> reg 0x400 bits 31:28 = 0 (GPIO) */
			v = readl(gpio + 0x400);
			writel(v & ~0xf0000000, gpio + 0x400);
			pr_info("XAGA-SPI: gpio135 mode reg=%#x (want 0)\n",
				readl(gpio + 0x400));
			iounmap(gpio);
		}
	}
	/*
	 * SPI2 pins (GPIO109-112) live in iocfg_rm (i_base 1 = 0x11c20000):
	 *   IES: +0x50 bits 21/22/23/24   SMT: +0xa0 bit 13
	 *   PU:  +0x70 bits 21/22/23/24   PD:  +0x60 bits 21/22/23/24
	 * INT pin GPIO135 lives in iocfg_brr (i_base 7 = 0x11d50000):
	 *   IES: +0x30 bit 2, SMT: +0x80 bit 0, PU: +0x50 bit 2, PD: +0x40 bit 2
	 */
	{
		void __iomem *rm = ioremap(0x11c20000, 0x1000);
		if (rm) {
			u32 pins = BIT(21) | BIT(22) | BIT(23) | BIT(24);
			v = readl(rm + 0x50);
			writel(v | pins, rm + 0x50);		/* IES */
			v = readl(rm + 0xa0);
			writel(v | BIT(13), rm + 0xa0);		/* SMT */
			v = readl(rm + 0x70);
			writel(v | pins, rm + 0x70);		/* PU */
			v = readl(rm + 0x60);
			writel(v & ~pins, rm + 0x60);		/* PD off */
			pr_info("XAGA-SPI: spi2 pins IES=%#x SMT=%#x PU=%#x PD=%#x\n",
				readl(rm + 0x50), readl(rm + 0xa0),
				readl(rm + 0x70), readl(rm + 0x60));
			iounmap(rm);
		}
		rm = ioremap(0x11d50000, 0x1000);	/* iocfg_brr, GPIO135 */
		if (rm) {
			v = readl(rm + 0x30);
			writel(v | BIT(2), rm + 0x30);		/* IES */
			v = readl(rm + 0x80);
			writel(v | BIT(0), rm + 0x80);		/* SMT */
			v = readl(rm + 0x50);
			writel(v | BIT(2), rm + 0x50);		/* PU */
			v = readl(rm + 0x40);
			writel(v & ~BIT(2), rm + 0x40);		/* PD off */
			pr_info("XAGA-SPI: gpio135 IES=%#x SMT=%#x PU=%#x PD=%#x\n",
				readl(rm + 0x30), readl(rm + 0x80),
				readl(rm + 0x50), readl(rm + 0x40));
			iounmap(rm);
		}
	}

	return 0;
}
postcore_initcall(xaga_i2c_power_on);

static int num_standard_resources;
static struct resource *standard_resources;

phys_addr_t __fdt_pointer __initdata;
u64 mmu_enabled_at_boot __initdata;

/*
 * Standard memory resources
 */
static struct resource mem_res[] = {
	{
		.name = "Kernel code",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	},
	{
		.name = "Kernel data",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_SYSTEM_RAM
	}
};

#define kernel_code mem_res[0]
#define kernel_data mem_res[1]

/*
 * The recorded values of x0 .. x3 upon kernel entry.
 */
u64 __cacheline_aligned boot_args[4];

void __init smp_setup_processor_id(void)
{
	u64 mpidr = read_cpuid_mpidr() & MPIDR_HWID_BITMASK;
	set_cpu_logical_map(0, mpidr);

	pr_info("Booting Linux on physical CPU 0x%010lx [0x%08x]\n",
		(unsigned long)mpidr, read_cpuid_id());
}

bool arch_match_cpu_phys_id(int cpu, u64 phys_id)
{
	return phys_id == cpu_logical_map(cpu);
}

struct mpidr_hash mpidr_hash;
/**
 * smp_build_mpidr_hash - Pre-compute shifts required at each affinity
 *			  level in order to build a linear index from an
 *			  MPIDR value. Resulting algorithm is a collision
 *			  free hash carried out through shifting and ORing
 */
static void __init smp_build_mpidr_hash(void)
{
	u32 i, affinity, fs[4], bits[4], ls;
	u64 mask = 0;
	/*
	 * Pre-scan the list of MPIDRS and filter out bits that do
	 * not contribute to affinity levels, ie they never toggle.
	 */
	for_each_possible_cpu(i)
		mask |= (cpu_logical_map(i) ^ cpu_logical_map(0));
	pr_debug("mask of set bits %#llx\n", mask);
	/*
	 * Find and stash the last and first bit set at all affinity levels to
	 * check how many bits are required to represent them.
	 */
	for (i = 0; i < 4; i++) {
		affinity = MPIDR_AFFINITY_LEVEL(mask, i);
		/*
		 * Find the MSB bit and LSB bits position
		 * to determine how many bits are required
		 * to express the affinity level.
		 */
		ls = fls(affinity);
		fs[i] = affinity ? ffs(affinity) - 1 : 0;
		bits[i] = ls - fs[i];
	}
	/*
	 * An index can be created from the MPIDR_EL1 by isolating the
	 * significant bits at each affinity level and by shifting
	 * them in order to compress the 32 bits values space to a
	 * compressed set of values. This is equivalent to hashing
	 * the MPIDR_EL1 through shifting and ORing. It is a collision free
	 * hash though not minimal since some levels might contain a number
	 * of CPUs that is not an exact power of 2 and their bit
	 * representation might contain holes, eg MPIDR_EL1[7:0] = {0x2, 0x80}.
	 */
	mpidr_hash.shift_aff[0] = MPIDR_LEVEL_SHIFT(0) + fs[0];
	mpidr_hash.shift_aff[1] = MPIDR_LEVEL_SHIFT(1) + fs[1] - bits[0];
	mpidr_hash.shift_aff[2] = MPIDR_LEVEL_SHIFT(2) + fs[2] -
						(bits[1] + bits[0]);
	mpidr_hash.shift_aff[3] = MPIDR_LEVEL_SHIFT(3) +
				  fs[3] - (bits[2] + bits[1] + bits[0]);
	mpidr_hash.mask = mask;
	mpidr_hash.bits = bits[3] + bits[2] + bits[1] + bits[0];
	pr_debug("MPIDR hash: aff0[%u] aff1[%u] aff2[%u] aff3[%u] mask[%#llx] bits[%u]\n",
		mpidr_hash.shift_aff[0],
		mpidr_hash.shift_aff[1],
		mpidr_hash.shift_aff[2],
		mpidr_hash.shift_aff[3],
		mpidr_hash.mask,
		mpidr_hash.bits);
	/*
	 * 4x is an arbitrary value used to warn on a hash table much bigger
	 * than expected on most systems.
	 */
	if (mpidr_hash_size() > 4 * num_possible_cpus())
		pr_warn("Large number of MPIDR hash buckets detected\n");
}

static void __init setup_machine_fdt(phys_addr_t dt_phys)
{
	int size = 0;
	void *dt_virt = fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL);
	const char *name;

	if (dt_virt)
		memblock_reserve(dt_phys, size);

	/*
	 * dt_virt is a fixmap address, hence __pa(dt_virt) can't be used.
	 * Pass dt_phys directly.
	 */
	if (!early_init_dt_scan(dt_virt, dt_phys)) {
		pr_crit("\n"
			"Error: invalid device tree blob: PA=%pa, VA=%px, size=%d bytes\n"
			"The dtb must be 8-byte aligned and must not exceed 2 MB in size.\n"
			"\nPlease check your bootloader.\n",
			&dt_phys, dt_virt, size);

		/*
		 * Note that in this _really_ early stage we cannot even BUG()
		 * or oops, so the least terrible thing to do is cpu_relax(),
		 * or else we could end-up printing non-initialized data, etc.
		 */
		while (true)
			cpu_relax();
	}

	/* Early fixups are done, map the FDT as read-only now */
	fixmap_remap_fdt(dt_phys, &size, PAGE_KERNEL_RO);

	name = of_flat_dt_get_machine_name();
	if (!name)
		return;

	pr_info("Machine model: %s\n", name);
	dump_stack_set_arch_desc("%s (DT)", name);
}

static void __init request_standard_resources(void)
{
	struct memblock_region *region;
	struct resource *res;
	unsigned long i = 0;
	size_t res_size;

	kernel_code.start   = __pa_symbol(_text);
	kernel_code.end     = __pa_symbol(__init_begin - 1);
	kernel_data.start   = __pa_symbol(_sdata);
	kernel_data.end     = __pa_symbol(_end - 1);
	insert_resource(&iomem_resource, &kernel_code);
	insert_resource(&iomem_resource, &kernel_data);

	num_standard_resources = memblock.memory.cnt;
	res_size = num_standard_resources * sizeof(*standard_resources);
	standard_resources = memblock_alloc_or_panic(res_size, SMP_CACHE_BYTES);

	for_each_mem_region(region) {
		res = &standard_resources[i++];
		if (memblock_is_nomap(region)) {
			res->name  = "reserved";
			res->flags = IORESOURCE_MEM;
			res->start = __pfn_to_phys(memblock_region_reserved_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_reserved_end_pfn(region)) - 1;
		} else {
			res->name  = "System RAM";
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			res->start = __pfn_to_phys(memblock_region_memory_base_pfn(region));
			res->end = __pfn_to_phys(memblock_region_memory_end_pfn(region)) - 1;
		}

		insert_resource(&iomem_resource, res);
	}
}

static int __init reserve_memblock_reserved_regions(void)
{
	u64 i, j;

	for (i = 0; i < num_standard_resources; ++i) {
		struct resource *mem = &standard_resources[i];
		phys_addr_t r_start, r_end, mem_size = resource_size(mem);

		if (!memblock_is_region_reserved(mem->start, mem_size))
			continue;

		for_each_reserved_mem_range(j, &r_start, &r_end) {
			resource_size_t start, end;

			start = max(PFN_PHYS(PFN_DOWN(r_start)), mem->start);
			end = min(PFN_PHYS(PFN_UP(r_end)) - 1, mem->end);

			if (start > mem->end || end < mem->start)
				continue;

			reserve_region_with_split(mem, start, end, "reserved");
		}
	}

	return 0;
}
arch_initcall(reserve_memblock_reserved_regions);

u64 __cpu_logical_map[NR_CPUS] = { [0 ... NR_CPUS-1] = INVALID_HWID };

u64 cpu_logical_map(unsigned int cpu)
{
	return __cpu_logical_map[cpu];
}

void __init __no_sanitize_address setup_arch(char **cmdline_p)
{
	setup_initial_init_mm(_text, _etext, _edata, _end);

	*cmdline_p = boot_command_line;

	kaslr_init();

	early_fixmap_init();
	early_ioremap_init();

	/* Earliest point the fixmap maps the xaga log_store ring (0x7ffbf000);
	 * from here on every printk() is mirrored into it, and LK restores the
	 * region into expdb on the next boot. */
	xaga_marker_early_init();

	setup_machine_fdt(__fdt_pointer);

	/*
	 * XAGA: override the FDT LK handed us (its Android DT) with our own
	 * embedded mt6895-xiaomi-xaga.dtb. Doing this right after
	 * setup_machine_fdt() (which already consumed /chosen bootargs and
	 * /memory from LK's FDT into memblock) means EVERYTHING that follows
	 * uses OUR tree: early_init_fdt_scan_reserved_mem() in
	 * arm64_memblock_init() will apply our no-map on the framebuffer
	 * region, paging_init() will exclude it from the direct map, and
	 * unflatten_device_tree() builds the driver tree from ours.
	 */
	extern char _binary_arch_arm64_boot_dts_mediatek_mt6895_xiaomi_xaga_dtb_start[];
	extern char _binary_arch_arm64_boot_dts_mediatek_mt6895_xiaomi_xaga_dtb_end[];
	if (acpi_disabled) {
		pr_info("XAGA-DTB: overriding LK FDT with embedded "
			"mt6895-xiaomi-xaga.dtb (%d bytes)\n",
			(int)(_binary_arch_arm64_boot_dts_mediatek_mt6895_xiaomi_xaga_dtb_end -
			      _binary_arch_arm64_boot_dts_mediatek_mt6895_xiaomi_xaga_dtb_start));
		initial_boot_params = _binary_arch_arm64_boot_dts_mediatek_mt6895_xiaomi_xaga_dtb_start;

		/*
		 * LK's cmdline was already captured by setup_machine_fdt()
		 * (from LK's own FDT /chosen/bootargs); vendor_boot/boot.img
		 * cmdlines are ignored by LK. Re-read /chosen/bootargs from
		 * OUR embedded FDT so cmdline changes (console=ttyGS0,
		 * printk.devkmsg=on, ...) actually take effect. saved_command_line
		 * is built later in start_kernel, so this propagates everywhere.
		 */
		early_init_dt_scan_chosen(boot_command_line);
		pr_info("XAGA-CMDLINE: %s\n", boot_command_line);

		/*
		 * Keep every clock/power-domain running. LK left the display
		 * (and UFS) clocked and scanning; without a DRM driver, the
		 * clock core would gate the DISP clocks at late boot and
		 * freeze the panel. clk_ignore_unused is a __setup param,
		 * parsed after setup_arch(), so appending it here is enough.
		 */
		if (!strstr(boot_command_line, "clk_ignore_unused"))
			strncat(boot_command_line, " clk_ignore_unused",
				COMMAND_LINE_SIZE - strlen(boot_command_line) - 1);
	}

	/*
	 * Initialise the static keys early as they may be enabled by the
	 * cpufeature code and early parameters.
	 */
	jump_label_init();
	parse_early_param();

	dynamic_scs_init();

	/*
	 * The primary CPU enters the kernel with all DAIF exceptions masked.
	 *
	 * We must unmask Debug and SError before preemption or scheduling is
	 * possible to ensure that these are consistently unmasked across
	 * threads, and we want to unmask SError as soon as possible after
	 * initializing earlycon so that we can report any SErrors immediately.
	 *
	 * IRQ and FIQ will be unmasked after the root irqchip has been
	 * detected and initialized.
	 */
	local_daif_restore(DAIF_PROCCTX_NOIRQ);

	/*
	 * TTBR0 is only used for the identity mapping at this stage. Make it
	 * point to zero page to avoid speculatively fetching new entries.
	 */
	cpu_uninstall_idmap();

	xen_early_init();
	efi_init();

	if (!efi_enabled(EFI_BOOT)) {
		if ((u64)_text % MIN_KIMG_ALIGN)
			pr_warn(FW_BUG "Kernel image misaligned at boot, please fix your bootloader!");
		WARN_TAINT(mmu_enabled_at_boot, TAINT_FIRMWARE_WORKAROUND,
			   FW_BUG "Booted with MMU enabled!");
	}

	arm64_memblock_init();

	paging_init();

	acpi_table_upgrade();

	/* Parse the ACPI tables for possible boot-time configuration */
	acpi_boot_table_init();

	if (acpi_disabled)
		unflatten_device_tree();

	bootmem_init();

	kasan_init();
	request_standard_resources();

	early_ioremap_reset();

	if (acpi_disabled)
		psci_dt_init();
	else
		psci_acpi_init();

	arm64_rsi_init();

	init_bootcpu_ops();
	smp_init_cpus();
	smp_build_mpidr_hash();

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
	/*
	 * Make sure init_thread_info.ttbr0 always generates translation
	 * faults in case uaccess_enable() is inadvertently called by the init
	 * thread.
	 */
	init_task.thread_info.ttbr0 = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
#endif

	if (boot_args[1] || boot_args[2] || boot_args[3]) {
		pr_err("WARNING: x1-x3 nonzero in violation of boot protocol:\n"
			"\tx1: %016llx\n\tx2: %016llx\n\tx3: %016llx\n"
			"This indicates a broken bootloader or old kernel\n",
			boot_args[1], boot_args[2], boot_args[3]);
	}
}

static inline bool cpu_can_disable(unsigned int cpu)
{
#ifdef CONFIG_HOTPLUG_CPU
	const struct cpu_operations *ops = get_cpu_ops(cpu);

	if (ops && ops->cpu_can_disable)
		return ops->cpu_can_disable(cpu);
#endif
	return false;
}

bool arch_cpu_is_hotpluggable(int num)
{
	return cpu_can_disable(num);
}

static void dump_kernel_offset(void)
{
	const unsigned long offset = kaslr_offset();

	if (IS_ENABLED(CONFIG_RANDOMIZE_BASE) && offset > 0) {
		pr_emerg("Kernel Offset: 0x%lx from 0x%lx\n",
			 offset, KIMAGE_VADDR);
		pr_emerg("PHYS_OFFSET: 0x%llx\n", PHYS_OFFSET);
	} else {
		pr_emerg("Kernel Offset: disabled\n");
	}
}

static int arm64_panic_block_dump(struct notifier_block *self,
				  unsigned long v, void *p)
{
	dump_kernel_offset();
	dump_cpu_features();
	dump_mem_limit();
	return 0;
}

static struct notifier_block arm64_panic_block = {
	.notifier_call = arm64_panic_block_dump
};

static int __init register_arm64_panic_block(void)
{
	atomic_notifier_chain_register(&panic_notifier_list,
				       &arm64_panic_block);
	return 0;
}
device_initcall(register_arm64_panic_block);

static int __init check_mmu_enabled_at_boot(void)
{
	if (!efi_enabled(EFI_BOOT) && mmu_enabled_at_boot)
		panic("Non-EFI boot detected with MMU and caches enabled");
	return 0;
}
device_initcall_sync(check_mmu_enabled_at_boot);
