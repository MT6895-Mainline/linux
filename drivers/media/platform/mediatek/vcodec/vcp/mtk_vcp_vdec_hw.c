// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include "mtk_vcp_vdec_hw.h"

/* Highest operating point of the vendor OPP table: 660 MHz at 750 mV. */
#define VDEC_MAX_RATE 660000000UL

struct vdec_core {
	void __iomem *misc;
	struct device *domain, *larb;
	struct completion irq_done;
	int irq;
	bool owned;
};
struct mtk_vcp_vdec_hw {
	struct device *dev, *ube;
	struct mtk_vcp *vcp;
	struct clk_bulk_data clocks[3];
	struct vdec_core core[2];
	struct regulator *vcore;
	unsigned long perf_rate;
	struct mutex perf_lock;
	bool powered, uncertain;
};

static irqreturn_t vdec_irq(int irq, void *priv)
{
	struct vdec_core *c = priv;
	u32 status = readl(c->misc + 0xa4);

	if (!(status & BIT(16)))
		return IRQ_NONE;
	writel(status | 0x11, c->misc + 0xa4);
	writel(readl(c->misc + 0xa4) & ~0x10, c->misc + 0xa4);
	complete(&c->irq_done);
	return IRQ_HANDLED;
}

static int vdec_power_on(struct mtk_vcp_vdec_hw *hw)
{
	int domains = 0, larbs = 0, i, ret;

	if (hw->powered)
		return 0;
	for (i = 0; i < 3; i++) {
		unsigned long rate = clk_get_rate(hw->clocks[i].clk);

		if (!rate || rate > VDEC_MAX_RATE)
			return -ERANGE;
	}
	for (; domains < 2; domains++) {
		ret = pm_runtime_resume_and_get(hw->core[domains].domain);
		if (ret < 0)
			goto rollback;
	}
	for (; larbs < 2; larbs++) {
		ret = pm_runtime_resume_and_get(hw->core[larbs].larb);
		if (ret < 0)
			goto rollback;
	}
	ret = clk_bulk_prepare_enable(3, hw->clocks);
	if (ret)
		goto rollback;
	hw->powered = true;
	__module_get(THIS_MODULE);
	return 0;
rollback:
	while (larbs--)
		pm_runtime_put_sync(hw->core[larbs].larb);
	while (domains--)
		pm_runtime_put_sync(hw->core[domains].domain);
	return ret;
}

int mtk_vcp_vdec_hw_power(struct mtk_vcp_vdec_hw *hw, unsigned int core, bool on)
{
	int ret;

	if (core >= 2 || hw->uncertain)
		return -EINVAL;
	if (on) {
		if (hw->core[core].owned)
			return -EBUSY;
		ret = vdec_power_on(hw);
		if (ret)
			return ret;
	} else if (!hw->core[core].owned) {
		return -EINVAL;
	}
	/* Firmware owns the interrupt except for explicit WAITISR requests.
	 * Keep the rails until the complete session has been deinitialized.
	 */
	hw->core[core].owned = on;
	return 0;
}

int mtk_vcp_vdec_hw_wait(struct mtk_vcp_vdec_hw *hw, unsigned int core)
{
	struct vdec_core *c;
	int ret;

	if (core >= 2 || !hw->core[core].owned || hw->uncertain)
		return -EINVAL;
	c = &hw->core[core];
	reinit_completion(&c->irq_done);
	enable_irq(c->irq);
	ret = wait_for_completion_timeout(&c->irq_done, msecs_to_jiffies(1000)) ?
		0 : -ETIMEDOUT;
	disable_irq(c->irq);
	if (ret)
		hw->uncertain = true;
	return ret;
}

/*
 * Move the decoder to the operating point its stream needs.
 *
 * Same shape as the encoder side: the frame geometry and the stream's frame
 * rate give the pixel throughput, the OPP table maps that to a voltage, and
 * the rate request lands on the VDEC gate clock, which reparents the
 * topckgen VDEC mux through CLK_SET_RATE_PARENT.
 *
 * The hardware serves the highest request, so a decoder that backs off does
 * not drag the display down with it.
 */
int mtk_vcp_vdec_hw_set_perf(struct mtk_vcp_vdec_hw *hw, u32 width, u32 height,
			     u32 fps)
{
	struct dev_pm_opp *opp;
	unsigned long hz, rate;
	bool increasing;
	int volt = 0, ret = 0;

	if (!width || !height || !fps)
		return -EINVAL;

	hz = (unsigned long)width * height * fps;
	opp = dev_pm_opp_find_freq_ceil(hw->dev, &hz);
	if (IS_ERR(opp))
		return PTR_ERR(opp) == -ERANGE ? 0 : PTR_ERR(opp);
	volt = dev_pm_opp_get_voltage(opp);
	hz = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);

	rate = clk_round_rate(hw->clocks[0].clk, hz) ?: hz;
	increasing = rate > hw->perf_rate;

	mutex_lock(&hw->perf_lock);
	if (increasing && hw->vcore && volt > 0)
		ret = regulator_set_voltage(hw->vcore, volt, INT_MAX);
	if (!ret)
		ret = clk_set_rate(hw->clocks[0].clk, rate);
	if (!ret && !increasing && hw->vcore && volt > 0)
		ret = regulator_set_voltage(hw->vcore, volt, INT_MAX);
	if (!ret) {
		hw->perf_rate = rate;
		dev_info(hw->dev, "VDEC perf: %ux%u@%u -> %lu Hz, %d uV\n",
			 width, height, fps, rate, volt);
	} else {
		dev_warn(hw->dev, "VDEC perf failed: %ux%u@%u: %d\n",
			 width, height, fps, ret);
	}
	mutex_unlock(&hw->perf_lock);

	return ret;
}

int mtk_vcp_vdec_hw_alloc(struct mtk_vcp_vdec_hw *hw, u32 type, size_t size,
			  struct mtk_vcp_mem *mem)
{
	if (!type)
		return mtk_vcp_alloc_workmem(hw->vcp, size, mem);
	if (type > 2)
		return -EINVAL;
	mem->size = size;
	mem->cpu = dma_alloc_coherent(type == 2 ? hw->ube : hw->dev, size,
				      &mem->dma, GFP_KERNEL);
	return mem->cpu ? 0 : -ENOMEM;
}

void mtk_vcp_vdec_hw_free(struct mtk_vcp_vdec_hw *hw, u32 type, struct mtk_vcp_mem *mem)
{
	if (!type)
		mtk_vcp_free_workmem(hw->vcp, mem);
	else if (type <= 2)
		dma_free_coherent(type == 2 ? hw->ube : hw->dev, mem->size,
				  mem->cpu, mem->dma);
	mem->cpu = NULL;
}

int mtk_vcp_vdec_hw_stop(struct mtk_vcp_vdec_hw *hw)
{
	int i;

	if (hw->uncertain || hw->core[0].owned || hw->core[1].owned)
		return -EBUSY;
	if (!hw->powered)
		return 0;
	clk_bulk_disable_unprepare(3, hw->clocks);
	for (i = 1; i >= 0; i--)
		pm_runtime_put_sync(hw->core[i].larb);
	for (i = 1; i >= 0; i--)
		pm_runtime_put_sync(hw->core[i].domain);
	hw->powered = false;
	module_put(THIS_MODULE);
	return 0;
}

static void vdec_detach(void *data)
{
	dev_pm_domain_detach(data, true);
}
static void vdec_put(void *data)
{
	put_device(data);
}

struct mtk_vcp_vdec_hw *mtk_vcp_vdec_hw_create(struct platform_device *pdev,
					    struct mtk_vcp *vcp, struct device *ube)
{
	struct device *dev = &pdev->dev;
	struct mtk_vcp_vdec_hw *hw;
	int i, ret;

	if (!iommu_get_domain_for_dev(dev))
		return ERR_PTR(-ENODEV);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ERR_PTR(ret);
	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return ERR_PTR(-ENOMEM);
	hw->dev = dev;
	hw->vcp = vcp;
	hw->ube = ube;
	hw->clocks[0].id = "soc";
	hw->clocks[1].id = "lat";
	hw->clocks[2].id = "core";
	ret = devm_clk_bulk_get(dev, 3, hw->clocks);
	if (ret)
		return ERR_PTR(ret);
	/* Optional: without an OPP table the decoder keeps its boot rate. */
	ret = dev_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return ERR_PTR(ret);
	mutex_init(&hw->perf_lock);
	hw->vcore = devm_regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(hw->vcore)) {
		if (PTR_ERR(hw->vcore) != -ENODEV)
			return ERR_CAST(hw->vcore);
		hw->vcore = NULL;
	}
	for (i = 0; i < 2; i++) {
		struct vdec_core *c = &hw->core[i];
		struct device_node *node;
		struct platform_device *larb;

		c->misc = devm_platform_ioremap_resource_byname(pdev, i ? "lat-misc" : "misc");
		if (IS_ERR(c->misc))
			return ERR_PTR(PTR_ERR(c->misc));
		c->domain = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR_OR_NULL(c->domain))
			return ERR_PTR(c->domain ? PTR_ERR(c->domain) : -ENODEV);
		ret = devm_add_action_or_reset(dev, vdec_detach, c->domain);
		if (ret)
			return ERR_PTR(ret);
		node = of_parse_phandle(dev->of_node, "mediatek,larbs", i);
		if (!node)
			return ERR_PTR(-EINVAL);
		larb = of_find_device_by_node(node);
		of_node_put(node);
		if (!larb)
			return ERR_PTR(-EPROBE_DEFER);
		c->larb = &larb->dev;
		ret = devm_add_action_or_reset(dev, vdec_put, c->larb);
		if (ret)
			return ERR_PTR(ret);
		if (!device_is_bound(c->larb))
			return ERR_PTR(-EPROBE_DEFER);
		init_completion(&c->irq_done);
		c->irq = platform_get_irq(pdev, i);
		if (c->irq < 0)
			return ERR_PTR(c->irq);
		ret = devm_request_irq(dev, c->irq, vdec_irq, IRQF_NO_AUTOEN, dev_name(dev), c);
		if (ret)
			return ERR_PTR(ret);
	}
	return hw;
}
