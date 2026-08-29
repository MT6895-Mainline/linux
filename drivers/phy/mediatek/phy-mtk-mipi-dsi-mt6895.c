// SPDX-License-Identifier: GPL-2.0
#include "phy-mtk-io.h"
#include "phy-mtk-mipi-dsi.h"

/*
 * MT6895 MIPI TX (D-PHY) PLL and lane programming.
 *
 * The MT6895 mipi-tx block uses the MT6983-generation register layout, which
 * is NOT the mt8183/mt2701 layout that most mainline phy drivers use. In
 * particular the PLL control registers are at 0x2c/0x30 (CON0/CON1) with the
 * power control at 0x28, the PLL enable is bit0 of CON1 (not bit4), and the
 * fractional PCW needs the div3 + fbksel computation. Register offsets taken
 * from the stock xaga kernel drivers/gpu/drm/mediatek/mediatek_v2/mtk_mipi_tx.c
 * (mtk_mipi_tx_pll_prepare_mt6983 / _dsi_get_pcw_mt6983).
 */

/* MT6983-generation registers */
#define MIPITX_LANE_CON_MT6983		0x0004
#define MIPITX_VOLTAGE_SEL_MT6983	0x0008
#define FLD_RG_DSI_PRD_REF_SEL		GENMASK(5, 0)
#define MIPITX_PRESERVED_MT6983		0x000C
#define MIPITX_PLL_PWR			0x0028
#define AD_DSI_PLL_SDM_PWR_ON		BIT(0)
#define AD_DSI_PLL_SDM_ISO_EN		BIT(1)
#define MIPITX_PLL_CON0			0x002C
#define MIPITX_PLL_CON1			0x0030
#define MIPITX_PLL_CON4			0x003C
#define RG_DSI_PLL_EN_MT6983		BIT(0)
#define FLD_RG_DSI_PLL_FBSEL_MT6983	BIT(13)
#define FLD_RG_DSI_PLL_POSDIV		GENMASK(10, 8)
#define FLD_RG_DSI_PLL_DIV3_EN		BIT(28)
#define MIPITX_SW_CTRL_CON4_MT6983	0x0050

/* lane LDO output enables (common DSI lane registers, used by the
 * downstream generic mtk_mipi_tx_power_on_signal for this generation) */
#define MIPITX_DSI_CLOCK_LANE		0x04
#define MIPITX_DSI_DATA_LANE0		0x08
#define MIPITX_DSI_DATA_LANE1		0x0c
#define MIPITX_DSI_DATA_LANE2		0x10
#define MIPITX_DSI_DATA_LANE3		0x14
#define RG_DSI_LNTx_LDOOUT_EN		BIT(0)
#define MIPITX_DSI_TOP_CON		0x40
#define RG_DSI_PAD_TIE_LOW_EN		BIT(11)

static int mtk_mipi_tx_pll_enable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;
	unsigned int txdiv, txdiv0, div3, div3_en, fbksel;
	unsigned int rate_mhz;
	u64 pcw, pf;
	u32 tmp;

	dev_dbg(mipi_tx->dev, "enable: %u bps\n", mipi_tx->data_rate);

	rate_mhz = mipi_tx->data_rate / 1000000;

	if (rate_mhz >= 6000) {
		txdiv = 1; txdiv0 = 0; div3 = 1; div3_en = 0;
	} else if (rate_mhz >= 3000) {
		txdiv = 2; txdiv0 = 1; div3 = 1; div3_en = 0;
	} else if (rate_mhz >= 2000) {
		txdiv = 1; txdiv0 = 0; div3 = 3; div3_en = 1;
	} else if (rate_mhz >= 1500) {
		txdiv = 4; txdiv0 = 2; div3 = 1; div3_en = 0;
	} else if (rate_mhz >= 1000) {
		txdiv = 2; txdiv0 = 1; div3 = 3; div3_en = 1;
	} else if (rate_mhz >= 750) {
		txdiv = 8; txdiv0 = 3; div3 = 1; div3_en = 0;
	} else if (rate_mhz >= 510) {
		txdiv = 4; txdiv0 = 2; div3 = 3; div3_en = 1;
	} else {
		dev_err(mipi_tx->dev, "data rate too low: %u MHz\n", rate_mhz);
		return -EINVAL;
	}

	if (rate_mhz < 2500)
		mtk_phy_update_bits(base + MIPITX_VOLTAGE_SEL_MT6983,
				    FLD_RG_DSI_PRD_REF_SEL, 0x0);
	else
		mtk_phy_update_bits(base + MIPITX_VOLTAGE_SEL_MT6983,
				    FLD_RG_DSI_PRD_REF_SEL, 0x4);

	writel(0x0, base + MIPITX_PRESERVED_MT6983);
	writel(0x00FF12E0, base + MIPITX_PLL_CON4);
	/* BG_LPF_EN / BG_CORE_EN */
	writel(0x3FFF0180, base + MIPITX_LANE_CON_MT6983);
	udelay(500);
	writel(0x3FFF0080, base + MIPITX_LANE_CON_MT6983);

	/* step 1: SDM_RWR_ON / SDM_ISO_EN */
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);
	udelay(30);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);

	fbksel = ((rate_mhz >> 1) * txdiv) >= 3800 ? 2 : 1;
	mtk_phy_update_bits(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_FBSEL_MT6983,
			    (fbksel - 1) << 13);

	/*
	 * Fractional PCW, mirroring downstream _dsi_get_pcw_mt6983:
	 *   r  = (rate/2) * txdiv * div3 / fbksel     (rate in MHz)
	 *   pcw = r / 26,  pcw_floor = r % 26  (26 MHz ref)
	 *   bits 24-31: pcw; 16-23/8-15/0-7: 256-based fraction of pcw_floor/26
	 */
	{
		u64 r = (u64)(rate_mhz >> 1) * txdiv * div3 / fbksel;

		pcw = r / 26;
		pf = r % 26;

		tmp = ((pcw & 0xFF) << 24) |
		      (((256 * pf / 26) & 0xFF) << 16) |
		      (((256 * (256 * pf % 26) / 26) & 0xFF) << 8) |
		      ((256 * (256 * (256 * pf % 26) % 26) / 26) & 0xFF);
		writel(tmp, base + MIPITX_PLL_CON0);
	}

	mtk_phy_update_bits(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_POSDIV,
			    txdiv0 << 8);
	mtk_phy_update_bits(base + MIPITX_PLL_CON1, FLD_RG_DSI_PLL_DIV3_EN,
			    div3_en << 28);
	mtk_phy_set_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN_MT6983);

	udelay(50);

	return 0;
}

static void mtk_mipi_tx_pll_disable(struct clk_hw *hw)
{
	struct mtk_mipi_tx *mipi_tx = mtk_mipi_tx_from_clk_hw(hw);
	void __iomem *base = mipi_tx->regs;

	mtk_phy_clear_bits(base + MIPITX_PLL_CON1, RG_DSI_PLL_EN_MT6983);
	mtk_phy_clear_bits(base + MIPITX_SW_CTRL_CON4_MT6983, 1);
	mtk_phy_set_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_ISO_EN);
	mtk_phy_clear_bits(base + MIPITX_PLL_PWR, AD_DSI_PLL_SDM_PWR_ON);

	writel(0x3FFF0180, base + MIPITX_LANE_CON_MT6983);
	writel(0x3FFF0100, base + MIPITX_LANE_CON_MT6983);
}

static long mtk_mipi_tx_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	return clamp_val(rate, 125000000, 6000000000UL);
}

static const struct clk_ops mtk_mipi_tx_pll_ops = {
	.enable = mtk_mipi_tx_pll_enable,
	.disable = mtk_mipi_tx_pll_disable,
	.round_rate = mtk_mipi_tx_pll_round_rate,
	.set_rate = mtk_mipi_tx_pll_set_rate,
	.recalc_rate = mtk_mipi_tx_pll_recalc_rate,
};

static void mtk_mipi_tx_power_on_signal(struct phy *phy)
{
	/* mt6983/mt6895: lane LDO + pad config done in pll_enable via
	 * MIPITX_LANE_CON_MT6983 writes; no separate signal step */
}

static void mtk_mipi_tx_power_off_signal(struct phy *phy)
{
}

const struct mtk_mipitx_data mt6895_mipitx_data = {
	.mipi_tx_clk_ops = &mtk_mipi_tx_pll_ops,
	.mipi_tx_enable_signal = mtk_mipi_tx_power_on_signal,
	.mipi_tx_disable_signal = mtk_mipi_tx_power_off_signal,
};
