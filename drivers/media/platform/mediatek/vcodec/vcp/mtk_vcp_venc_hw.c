// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <dt-bindings/memory/mtk-memory-port.h>

#include "mtk_vcp_venc_hw.h"

#define VENC_IRQ_STATUS 0x05c
#define VENC_IRQ_ACK 0x060
#define VENC_BUSY 0x0ec
#define VENC_BREAK 0x1228
#define VENC_BREAK_MASK 0x7ffffdfc
#define VENC_BREAK_CONTROL 0x5040
#define VENC_VALID_IRQS 0xbf
#define VENC_CORES 2
#define VENC_INITIAL_MAX_RATE 250000000UL

struct venc_hw_core {
	struct mtk_vcp_venc_hw *hw;
	void __iomem *base;
	struct device *domain, *larb;
	int irq;
	spinlock_t irq_lock;
	wait_queue_head_t wait;
	/* Preserve repeated status values until the firmware consumes each one. */
	u32 irq_queue[VCP_VENC_BUFFERS];
	u32 irq_read, irq_write, irq_count;
	bool irq_fault, irq_enabled;
	u64 owner;
};

struct mtk_vcp_venc_hw {
	struct device *dev;
	struct mtk_vcp *vcp;
	struct venc_hw_core core[VENC_CORES];
	struct clk_bulk_data clocks[VENC_CORES];
	struct mutex lock;
	void (*notify)(void *priv, u64 cookie);
	void *notify_priv;
	bool powered;
};

static void venc_ack(struct venc_hw_core *core, u32 status)
{
	/* Match the vendor's individual W1C acknowledgements. */
	while (status) {
		u32 bit = BIT(__ffs(status));

		writel(bit, core->base + VENC_IRQ_ACK);
		status &= ~bit;
	}
}

static irqreturn_t venc_hw_irq(int irq, void *priv)
{
	struct venc_hw_core *core = priv;
	u32 status = readl(core->base + VENC_IRQ_STATUS);
	u32 queued;
	unsigned long flags;

	if (!status)
		return IRQ_NONE;
	venc_ack(core, status);
	spin_lock_irqsave(&core->irq_lock, flags);
	queued = core->irq_count;
	if (core->irq_count == ARRAY_SIZE(core->irq_queue)) {
		core->irq_fault = true;
	} else {
		core->irq_queue[core->irq_write] = status;
		core->irq_write = (core->irq_write + 1) % ARRAY_SIZE(core->irq_queue);
		core->irq_count++;
	}
	if (status & ~VENC_VALID_IRQS)
		core->irq_fault = true;
	spin_unlock_irqrestore(&core->irq_lock, flags);
	dev_info(core->hw->dev,
		 "VENC IRQ: core=%u status=%#x queued_before=%u queued_after=%u\n",
		 (unsigned int)(core - core->hw->core), status, queued,
		 queued < ARRAY_SIZE(core->irq_queue) ? queued + 1 : queued);
	wake_up(&core->wait);
	return IRQ_HANDLED;
}

static int venc_rails_on(struct mtk_vcp_venc_hw *hw)
{
	int domains = 0, larbs = 0, ret;

	if (hw->powered)
		return 0;
	/* Cap the frequency at the lowest vendor OPP. This does not establish
	 * its 575 mV voltage floor; vcore ownership is required before deployment.
	 */
	for (ret = 0; ret < VENC_CORES; ret++) {
		unsigned long rate = clk_get_rate(hw->clocks[ret].clk);

		if (!rate || rate > VENC_INITIAL_MAX_RATE) {
			dev_err(hw->dev, "VENC clock outside initial rate limit: %lu\n", rate);
			return -ERANGE;
		}
	}
	for (; domains < VENC_CORES; domains++) {
		ret = pm_runtime_resume_and_get(hw->core[domains].domain);
		if (ret < 0)
			goto rollback;
	}
	for (; larbs < VENC_CORES; larbs++) {
		ret = pm_runtime_resume_and_get(hw->core[larbs].larb);
		if (ret < 0)
			goto rollback;
	}
	ret = clk_bulk_prepare_enable(VENC_CORES, hw->clocks);
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

/* Called only after both engines are idle and both IRQs are synchronized. */
static int venc_rails_off(struct mtk_vcp_venc_hw *hw)
{
	int i, ret, error = 0;

	if (!hw->powered)
		return 0;
	clk_bulk_disable_unprepare(VENC_CORES, hw->clocks);
	for (i = VENC_CORES - 1; i >= 0; i--) {
		ret = pm_runtime_put_sync(hw->core[i].larb);
		if (ret < 0) {
			dev_warn(hw->dev, "LARB%u suspend failed: %d\n", i + 7, ret);
			error = error ?: ret;
		}
	}
	for (i = VENC_CORES - 1; i >= 0; i--) {
		ret = pm_runtime_put_sync(hw->core[i].domain);
		if (ret < 0) {
			dev_warn(hw->dev, "VENC domain%u suspend failed: %d\n", i, ret);
			error = error ?: ret;
		}
	}
	hw->powered = false;
	module_put(THIS_MODULE);
	return error;
}

static void venc_disable_irq(struct venc_hw_core *core)
{
	if (core->irq_enabled) {
		disable_irq(core->irq);
		core->irq_enabled = false;
	}
}

static int venc_power(void *priv, u64 instance, unsigned int id, bool on)
{
	struct mtk_vcp_venc_hw *hw = priv;
	struct venc_hw_core *core;
	unsigned long flags;
	int ret = 0;

	if (id >= VENC_CORES || !instance)
		return -EINVAL;
	core = &hw->core[id];
	mutex_lock(&hw->lock);
	if (on) {
		if (core->owner) {
			ret = core->owner == instance ? 0 : -EBUSY;
			goto out;
		}
		ret = venc_rails_on(hw);
		if (ret)
			goto out;
		venc_ack(core, readl(core->base + VENC_IRQ_STATUS));
		spin_lock_irqsave(&core->irq_lock, flags);
		core->irq_read = 0;
		core->irq_write = 0;
		core->irq_count = 0;
		core->irq_fault = false;
		spin_unlock_irqrestore(&core->irq_lock, flags);
		core->owner = instance;
		/* VCP has its own VENC ISR. POWER_ON grants hardware access,
		 * not IRQ ownership: an AP ISR here can clear the W1C status
		 * before firmware sees it and stall subsequent frames.
		 */
	} else {
		if (core->owner != instance) {
			ret = -EINVAL;
			goto out;
		}
		/* A normal firmware power-off must follow an idle engine. Do not
		 * interrupt another instance with the shared emergency break control.
		 */
		if (readl(core->base + VENC_BUSY)) {
			ret = -EBUSY;
			goto out;
		}
		venc_disable_irq(core);
		core->owner = 0;
		if (!hw->core[0].owner && !hw->core[1].owner)
			ret = venc_rails_off(hw);
	}
out:
	mutex_unlock(&hw->lock);
	return ret;
}

static int venc_wait_irq(void *priv, u64 instance, unsigned int id, u32 *status)
{
	struct mtk_vcp_venc_hw *hw = priv;
	struct venc_hw_core *core;
	unsigned long flags;
	long ready;
	int ret = 0;

	if (id >= VENC_CORES || !status)
		return -EINVAL;
	core = &hw->core[id];
	mutex_lock(&hw->lock);
	if (core->owner != instance) {
		ret = -EHOSTDOWN;
		goto out;
	}
	/* Only an explicit WAIT_ISR request delegates completion to AP.
	 * Leave a status latched before the request for the IRQ handler;
	 * do not acknowledge it here. Firmware cannot start another
	 * AP-waited operation until this service has replied.
	 */
	enable_irq(core->irq);
	core->irq_enabled = true;
	dev_info(hw->dev, "VENC WAIT_ISR: core=%u queued=%u\n", id,
		 READ_ONCE(core->irq_count));
	/* Hardware IRQ does not take hw->lock. It can precede this request. */
	ready = wait_event_timeout(core->wait, READ_ONCE(core->irq_count),
				   msecs_to_jiffies(1000));
	spin_lock_irqsave(&core->irq_lock, flags);
	if (core->irq_count) {
		*status = core->irq_queue[core->irq_read];
		core->irq_read = (core->irq_read + 1) % ARRAY_SIZE(core->irq_queue);
		core->irq_count--;
	} else {
		*status = 0;
	}
	if (core->irq_fault)
		ret = -EIO;
	else if (!ready)
		ret = -ETIMEDOUT;
	dev_info(hw->dev, "VENC WAIT_ISR done: core=%u status=%#x ret=%d queued=%u\n",
		 id, *status, ret, core->irq_count);
	spin_unlock_irqrestore(&core->irq_lock, flags);
	/* Return IRQ ownership before the caller replies to firmware. */
	venc_disable_irq(core);
out:
	mutex_unlock(&hw->lock);
	return ret;
}

static int venc_alloc(void *priv, u32 type, size_t size, struct mtk_vcp_mem *mem)
{
	struct mtk_vcp_venc_hw *hw = priv;

	if (!type)
		return mtk_vcp_alloc_workmem(hw->vcp, size, mem);
	if (type != 1)
		return -EOPNOTSUPP;
	mem->size = size;
	mem->cpu = dma_alloc_coherent(hw->dev, size, &mem->dma, GFP_KERNEL);
	return mem->cpu ? 0 : -ENOMEM;
}

static void venc_free(void *priv, u32 type, struct mtk_vcp_mem *mem)
{
	struct mtk_vcp_venc_hw *hw = priv;

	if (!type)
		mtk_vcp_free_workmem(hw->vcp, mem);
	else if (type == 1) {
		dma_free_coherent(hw->dev, mem->size, mem->cpu, mem->dma);
		mem->cpu = NULL;
	}
}

static void venc_notify(void *priv, u64 instance)
{
	struct mtk_vcp_venc_hw *hw = priv;

	hw->notify(hw->notify_priv, instance);
}

static const struct mtk_vcp_venc_ops venc_hw_ops = {
	.power = venc_power,
	.wait_irq = venc_wait_irq,
	.alloc = venc_alloc,
	.free = venc_free,
	.buffers_ready = venc_notify,
};

const struct mtk_vcp_venc_ops *mtk_vcp_venc_hw_ops(void)
{
	return &venc_hw_ops;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_hw_ops);

bool mtk_vcp_venc_hw_idle(struct mtk_vcp_venc_hw *hw)
{
	bool idle;

	mutex_lock(&hw->lock);
	idle = !hw->powered;
	mutex_unlock(&hw->lock);
	return idle;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_hw_idle);

int mtk_vcp_venc_hw_quiesce(struct mtk_vcp_venc_hw *hw)
{
	u32 value;
	int i, ret = 0;

	/* Stop the firmware before issuing shared hardware break commands. */
	if (!mtk_vcp_is_offline(hw->vcp))
		return -EBUSY;
	mutex_lock(&hw->lock);
	if (!hw->powered)
		goto out;
	for (i = 0; i < VENC_CORES; i++) {
		struct venc_hw_core *core = &hw->core[i];

		if (readl(core->base + VENC_BUSY)) {
			writel(0xc0000000, hw->core[0].base + VENC_BREAK_CONTROL);
			writel(1, core->base + VENC_BREAK);
			ret = readl_poll_timeout(core->base + VENC_BREAK, value,
						!(value & VENC_BREAK_MASK), 10, 100000);
			if (ret) {
				dev_err(hw->dev, "VENC core%d break timed out: %#x\n", i, value);
				goto out;
			}
		}
	}
	for (i = 0; i < VENC_CORES; i++) {
		venc_disable_irq(&hw->core[i]);
		hw->core[i].owner = 0;
	}
	ret = venc_rails_off(hw);
out:
	mutex_unlock(&hw->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_hw_quiesce);

static void venc_detach_domain(void *data)
{
	dev_pm_domain_detach(data, true);
}

static void venc_put_larb(void *data)
{
	put_device(data);
}

struct mtk_vcp_venc_hw *mtk_vcp_venc_hw_create(struct platform_device *pdev,
	struct mtk_vcp *vcp, void (*notify)(void *, u64), void *priv)
{
	static const char * const regions[] = { "VENC_SYS", "VENC_C1_SYS" };
	struct device *dev = &pdev->dev;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct mtk_vcp_venc_hw *hw;
	u32 ports[VENC_CORES] = {};
	int i, ret;

	if (!vcp || !notify || !fwspec || !iommu_get_domain_for_dev(dev))
		return ERR_PTR(-EINVAL);
	/* Both LARBs must route every declared port through normal domain0. */
	for (i = 0; i < fwspec->num_ids; i++) {
		u32 id = fwspec->ids[i], larb = MTK_M4U_TO_LARB(id);

		if (MTK_M4U_TO_TAB(id) || MTK_M4U_TO_DOM(id) || larb < 7 || larb > 8 ||
		    MTK_M4U_TO_PORT(id) > 30)
			return ERR_PTR(-EINVAL);
		ports[larb - 7] |= BIT(MTK_M4U_TO_PORT(id));
	}
	if (ports[0] != GENMASK(30, 0) || ports[1] != GENMASK(30, 0))
		return ERR_PTR(-EINVAL);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ERR_PTR(ret);
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));
	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw)
		return ERR_PTR(-ENOMEM);
	hw->dev = dev;
	hw->vcp = vcp;
	hw->notify = notify;
	hw->notify_priv = priv;
	mutex_init(&hw->lock);
	hw->clocks[0].id = "venc_sel";
	hw->clocks[1].id = "venc_c1_sel";
	ret = devm_clk_bulk_get(dev, VENC_CORES, hw->clocks);
	if (ret)
		return ERR_PTR(ret);
	for (i = 0; i < VENC_CORES; i++) {
		struct venc_hw_core *core = &hw->core[i];
		struct device_node *node;
		struct platform_device *larb;
		struct resource *res;

		core->hw = hw;
		spin_lock_init(&core->irq_lock);
		init_waitqueue_head(&core->wait);
		core->domain = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR_OR_NULL(core->domain))
			return ERR_PTR(core->domain ? PTR_ERR(core->domain) : -ENODEV);
		ret = devm_add_action_or_reset(dev, venc_detach_domain, core->domain);
		if (ret)
			return ERR_PTR(ret);
		node = of_parse_phandle(dev->of_node, "mediatek,larbs", i);
		if (!node)
			return ERR_PTR(-EINVAL);
		larb = of_find_device_by_node(node);
		of_node_put(node);
		if (!larb)
			return ERR_PTR(-EPROBE_DEFER);
		core->larb = &larb->dev;
		ret = devm_add_action_or_reset(dev, venc_put_larb, core->larb);
		if (ret)
			return ERR_PTR(ret);
		if (!device_is_bound(core->larb))
			return ERR_PTR(-EPROBE_DEFER);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, regions[i]);
		if (!res || resource_size(res) < 0x6000)
			return ERR_PTR(-EINVAL);
		core->base = devm_ioremap_resource(dev, res);
		if (IS_ERR(core->base))
			return ERR_PTR(PTR_ERR(core->base));
		core->irq = platform_get_irq(pdev, i);
		if (core->irq < 0)
			return ERR_PTR(core->irq);
		ret = devm_request_irq(dev, core->irq, venc_hw_irq, IRQF_NO_AUTOEN,
				       dev_name(dev), core);
		if (ret)
			return ERR_PTR(ret);
	}
	return hw;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_hw_create);
