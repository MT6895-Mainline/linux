// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6895 pericfg_ao bus clock gates (minimal).
 *
 * The stock clk-mt6895-peri.c registers the full pericfg gate set against
 * topckgen parents that this port's clock tree does not carry. Only the
 * BTIF path needs gates from this block today.
 *
 * Implemented with plain clk_hw_register_gate() over a direct ioremap of
 * the pericfg_ao block instead of the regmap-gate helpers: the regmap path
 * depends on syscon provider ordering that made probe fail with -ENOMEM on
 * this kernel, and a dead BTIF clock hangs the whole AP bus when the BT
 * stack touches its FIFOs.
 *
 * Register layout from the stock driver: PERAO0 bits at 0x3c, PERAO1 bits
 * at 0x40, both direct RMW, on the pericfg_ao block (0x11036000).
 */

#include <dt-bindings/clock/mt6895-clk.h>
#include <linux/clk-provider.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define PERAO0_BTIF_OFS 0x3c
#define PERAO0_BTIF_BIT 14
#define PERAO1_DMA_BCLK_OFS 0x40
#define PERAO1_DMA_BCLK_BIT 5

static DEFINE_SPINLOCK(peri_ao_lock);

struct peri_ao_clk_data {
struct clk_hw_onecell_data *onedata; /* indexed by CLK_PERAOP_* ids */
void __iomem *base;
};

static struct clk_hw *peri_ao_of_clk_get_hw(struct of_phandle_args *clkspec,
    void *_data)
{
struct peri_ao_clk_data *d = _data;
unsigned int idx = clkspec->args[0];

if (idx >= d->onedata->num || !d->onedata->hws[idx])
return ERR_PTR(-ENOENT);
return d->onedata->hws[idx];
}

static int peri_ao_clk_probe(struct platform_device *pdev)
{
struct device *dev = &pdev->dev;
struct peri_ao_clk_data *d;
struct clk_hw_onecell_data *onedata;
int ret;

d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
if (!d)
return -ENOMEM;

d->base = devm_platform_ioremap_resource(pdev, 0);
if (IS_ERR(d->base))
return PTR_ERR(d->base);

onedata = devm_kzalloc(dev, struct_size(onedata, hws, CLK_PERAO_NR_CLK),
       GFP_KERNEL);
if (!onedata)
return -ENOMEM;
onedata->num = CLK_PERAO_NR_CLK;
d->onedata = onedata;

onedata->hws[CLK_PERAOP_0_BTIF] =
devm_clk_hw_register_gate(dev, "peraop_0_btif", "clk26m", 0,
  d->base + PERAO0_BTIF_OFS,
  PERAO0_BTIF_BIT, 0, &peri_ao_lock);
if (IS_ERR(onedata->hws[CLK_PERAOP_0_BTIF]))
return PTR_ERR(onedata->hws[CLK_PERAOP_0_BTIF]);

onedata->hws[CLK_PERAOP_1_DMA_BCLK] =
devm_clk_hw_register_gate(dev, "peraop_1_dma_bclk", "clk26m",
  0, d->base + PERAO1_DMA_BCLK_OFS,
  PERAO1_DMA_BCLK_BIT, 0,
  &peri_ao_lock);
if (IS_ERR(onedata->hws[CLK_PERAOP_1_DMA_BCLK]))
return PTR_ERR(onedata->hws[CLK_PERAOP_1_DMA_BCLK]);

ret = devm_of_clk_add_hw_provider(dev, peri_ao_of_clk_get_hw, d);
if (ret)
return ret;

platform_set_drvdata(pdev, d);
dev_info(dev, "pericfg_ao BTIF bus gates registered\n");
return 0;
}

static const struct of_device_id of_match_clk_mt6895_peri_ao[] = {
{ .compatible = "mediatek,mt6895-pericfg-ao" },
{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6895_peri_ao);

static struct platform_driver clk_mt6895_peri_ao_drv = {
.probe = peri_ao_clk_probe,
.driver = {
.name = "clk-mt6895-peri_ao",
.of_match_table = of_match_clk_mt6895_peri_ao,
},
};
module_platform_driver(clk_mt6895_peri_ao_drv);

MODULE_LICENSE("GPL");
