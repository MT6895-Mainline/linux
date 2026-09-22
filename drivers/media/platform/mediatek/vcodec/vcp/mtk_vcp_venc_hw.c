// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
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
/* Highest operating point of the vendor OPP table: 624 MHz at 725 mV. */
#define VENC_MAX_RATE 624000000UL

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
	struct regulator *vcore;
	/*
	 * The step the configured workload needs, and the instance that asked for
	 * it. Firmware powers the cores down between frames, so the step outlives
	 * any single POWER_ON and is only dropped when that instance stops
	 * existing. active_uv is what the rail is asked for right now: it follows
	 * desired_uv across power cycles and is the only value that is relaxed when
	 * the cores go idle, so a power-down never costs the session its step.
	 */
	unsigned long desired_uv;
	u64 perf_owner;
	int active_uv;
	/* Highest step of the declared table, used as the request until a workload
	 * names the step it actually needs.
	 */
	unsigned long max_uv;
	/* Acquired references and consumed puts still awaiting suspend. */
	unsigned long pm_held, pm_pending;
	bool pm_ref;
	bool retained;
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
	dev_dbg(core->hw->dev,
		 "VENC IRQ: core=%u status=%#x queued_before=%u queued_after=%u\n",
		 (unsigned int)(core - core->hw->core), status, queued,
		 queued < ARRAY_SIZE(core->irq_queue) ? queued + 1 : queued);
	wake_up(&core->wait);
	return IRQ_HANDLED;
}

/*
 * Ask the shared rail for the step this session needs. Called with hw->lock
 * held and before the gates are enabled, so the cores never come up below it.
 *
 * The rail serves the highest request among its clients, so raising this
 * codec's floor cannot lower anyone else's and re-asking for the same step is
 * a no-op.
 */
static int venc_vote_apply(struct mtk_vcp_venc_hw *hw)
{
	int ret;

	if (!hw->vcore || !hw->desired_uv || hw->active_uv == (int)hw->desired_uv)
		return 0;
	ret = regulator_set_voltage(hw->vcore, hw->desired_uv, INT_MAX);
	if (ret) {
		dev_err(hw->dev, "VENC %lu uV VCORE request failed: %d\n",
			hw->desired_uv, ret);
		return ret;
	}
	hw->active_uv = hw->desired_uv;
	dev_dbg(hw->dev, "VENC VCORE request: %lu uV\n", hw->desired_uv);
	return 0;
}

/*
 * Highest step of the declared table. Firmware powers the cores up on its own
 * schedule, including during INIT and while it handles CONFIG, so the request
 * starts here and is only narrowed once a workload names its step. Without
 * this the rail would be asked for nothing at all in that window.
 */
static int venc_bootstrap_voltage(struct mtk_vcp_venc_hw *hw)
{
	struct dev_pm_opp *opp;
	unsigned long hz = ULONG_MAX, volt;

	opp = dev_pm_opp_find_freq_floor(hw->dev, &hz);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (!volt || volt > INT_MAX)
		return -EINVAL;
	hw->max_uv = volt;
	hw->desired_uv = volt;
	return 0;
}

/*
 * Stop asking the rail for this session's step. Called with hw->lock held and
 * only once every core is idle, because relaxing the rail underneath running
 * hardware is exactly what the request exists to prevent.
 *
 * Only the active request is given up here. desired_uv is left untouched, so
 * the next power-up restores the step the workload asked for; it is returned
 * to the top of the table only when the instance itself goes away. The devm
 * regulator reference outlives the gate being switched off, and the DVFSRC
 * provider keeps its own board floor underneath whatever its clients ask for.
 */
static void venc_vote_idle(struct mtk_vcp_venc_hw *hw)
{
	int ret;

	if (!hw->vcore || !hw->active_uv)
		return;
	if (hw->retained) {
		dev_warn(hw->dev, "VENC keeping the %d uV VCORE request: hardware was not confirmed idle\n",
			 hw->active_uv);
		return;
	}
	ret = regulator_set_voltage(hw->vcore, 0, INT_MAX);
	if (ret) {
		dev_warn(hw->dev, "VENC VCORE request release failed: %d (keeping %d uV)\n",
			 ret, hw->active_uv);
		return;
	}
	dev_dbg(hw->dev, "VENC VCORE request idle (was %d uV)\n", hw->active_uv);
	hw->active_uv = 0;
}

static int venc_pm_release(struct mtk_vcp_venc_hw *hw)
{
	int i, ret;

	for (i = 2 * VENC_CORES - 1; i >= 0; i--) {
		struct device *dev = i < VENC_CORES ? hw->core[i].domain :
				    hw->core[i - VENC_CORES].larb;

		if (hw->pm_held & BIT(i)) {
			ret = pm_runtime_put_sync_suspend(dev);
			hw->pm_held &= ~BIT(i);
		} else if (hw->pm_pending & BIT(i)) {
			ret = pm_runtime_suspend(dev);
		} else {
			continue;
		}
		if (ret < 0) {
			hw->pm_pending |= BIT(i);
			hw->retained = true;
			dev_warn(hw->dev, "VENC PM resource %d suspend failed: %d; retaining resources\n",
				 i, ret);
			return ret;
		}
		hw->pm_pending &= ~BIT(i);
	}
	hw->retained = false;
	if (hw->pm_ref) {
		hw->pm_ref = false;
		module_put(THIS_MODULE);
	}
	return 0;
}

static int venc_rails_on(struct mtk_vcp_venc_hw *hw)
{
	int domains = 0, larbs = 0, ret;

	if (hw->powered)
		return 0;
	/* A shutdown that did not complete left the cores in an unknown state.
	 * Powering them again would run at a step this driver cannot vouch for.
	 */
	if (hw->retained)
		return -EIO;
	/* venc_set_perf() only asks for a voltage; the DVFSRC provider owns the
	 * mux parents. Refuse to run above the top step, where no request this
	 * driver can make would cover the gate rate.
	 */
	for (ret = 0; ret < VENC_CORES; ret++) {
		unsigned long rate = clk_get_rate(hw->clocks[ret].clk);

		if (!rate || rate > VENC_MAX_RATE) {
			dev_err(hw->dev, "VENC clock above the top OPP: %lu\n", rate);
			return -ERANGE;
		}
	}
	/* Raise the rail before the gates: firmware powers the cores down between
	 * frames, so the step has to be restored on every power-up rather than
	 * only when the workload was configured.
	 */
	ret = venc_vote_apply(hw);
	if (ret)
		return ret;
	__module_get(THIS_MODULE);
	hw->pm_ref = true;
	for (; domains < VENC_CORES; domains++) {
		ret = pm_runtime_resume_and_get(hw->core[domains].domain);
		if (ret < 0)
			goto rollback;
		hw->pm_held |= BIT(domains);
	}
	for (; larbs < VENC_CORES; larbs++) {
		ret = pm_runtime_resume_and_get(hw->core[larbs].larb);
		if (ret < 0)
			goto rollback;
		hw->pm_held |= BIT(VENC_CORES + larbs);
	}
	ret = clk_bulk_prepare_enable(VENC_CORES, hw->clocks);
	if (ret)
		goto rollback;
	hw->powered = true;
	return 0;
rollback:
	venc_pm_release(hw);
	return ret;
}

/* Called only after both engines are idle and both IRQs are synchronized. */
static int venc_rails_off(struct mtk_vcp_venc_hw *hw)
{
	int ret;

	if (hw->powered) {
		clk_bulk_disable_unprepare(VENC_CORES, hw->clocks);
		hw->powered = false;
	}
	ret = venc_pm_release(hw);
	if (ret)
		return ret;
	venc_vote_idle(hw);
	return 0;
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
		/* Another instance holds the step that is in force. Letting this one
		 * run would execute it at a point it never requested, so it waits
		 * until that instance releases the step.
		 */
		if (hw->perf_owner && hw->perf_owner != instance) {
			ret = -EBUSY;
			goto out;
		}
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
	dev_dbg(hw->dev, "VENC WAIT_ISR: core=%u queued=%u\n", id,
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
	dev_dbg(hw->dev, "VENC WAIT_ISR done: core=%u status=%#x ret=%d queued=%u\n",
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

/*
 * Ask the shared DVFSRC rail for the operating point this workload needs.
 *
 * The vendor OPP table pairs a multimedia mux rate with the VCORE step the
 * DVFSRC needs for it, so the pixel rate of the configured geometry selects
 * the smallest step at or above it. That mapping is a property of the table:
 * no pixel-per-clock ratio is assumed here, and a workload above the top step
 * is reported as -ERANGE rather than run at a point nobody asked for.
 *
 * Only the voltage is requested. The mux parents are owned by the DVFSRC
 * provider, which coordinates them across all of its clients, so this driver
 * never calls clk_set_rate() or clk_set_parent(). Its clocks are kept for gate
 * control and for the rate sanity check in venc_rails_on().
 *
 * The step is held for this instance until release_perf() says the instance is
 * gone; see the comment on venc_hw_ops.
 */
static int venc_set_perf(void *priv, u64 instance, u32 width, u32 height, u32 fps)
{
	struct mtk_vcp_venc_hw *hw = priv;
	struct dev_pm_opp *opp;
	unsigned long hz, volt, previous;
	u64 pixels;
	int ret;

	if (!instance || !width || !height || !fps)
		return -EINVAL;
	if (check_mul_overflow((u64)width, (u64)height, &pixels) ||
	    check_mul_overflow(pixels, (u64)fps, &pixels))
		return -ERANGE;
	if (pixels > ULONG_MAX)
		return -ERANGE;
	/* No OPP table means this device has no way to name the step it needs. */
	if (!hw->vcore)
		return -EOPNOTSUPP;

	hz = (unsigned long)pixels;
	opp = dev_pm_opp_find_freq_ceil(hw->dev, &hz);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		dev_err(hw->dev, "VENC %ux%u@%u: no operating point at or above a %llu pixel/s rate: %d\n",
			width, height, fps, pixels, ret);
		return ret;
	}
	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (!volt || volt > INT_MAX)
		return -EINVAL;

	mutex_lock(&hw->lock);
	/* Hardware state is uncertain after a failed shutdown, so no step can be
	 * guaranteed from here on.
	 */
	if (hw->retained) {
		ret = -EIO;
		goto out;
	}
	/* One instance owns the step at a time. A second configured instance
	 * would otherwise overwrite the workload the first one is running.
	 */
	if (hw->perf_owner && hw->perf_owner != instance) {
		ret = -EBUSY;
		goto out;
	}
	if (hw->desired_uv == volt && hw->active_uv == (int)volt) {
		/* The step is already in force, so only the ownership record is
		 * missing. Without it this instance would not be protected from a
		 * second one taking the step over.
		 */
		hw->perf_owner = instance;
		ret = 0;
		goto out;
	}
	/* Every step that is not already in force is written through, including a
	 * lower one: the rail serves the highest request among all of its clients,
	 * so lowering this codec's own floor cannot pull anyone else down.
	 */
	previous = hw->desired_uv;
	hw->desired_uv = volt;
	ret = regulator_set_voltage(hw->vcore, volt, INT_MAX);
	if (ret) {
		hw->desired_uv = previous;
		goto out;
	}
	hw->perf_owner = instance;
	hw->active_uv = volt;
out:
	mutex_unlock(&hw->lock);

	if (ret)
		dev_warn(hw->dev, "VENC perf failed: %ux%u@%u -> %lu uV: %d\n",
			 width, height, fps, volt, ret);
	else
		dev_dbg(hw->dev, "VENC perf: %ux%u@%u -> %llu pixel/s, %lu uV\n",
			 width, height, fps, pixels, volt);
	return ret;
}

/*
 * Drop the step this instance held. Called when the instance stops existing,
 * which does not mean the VCP is offline: a decoder session keeps the firmware
 * running. What matters is this encoder's own hardware, so the callback checks
 * that itself and keeps the step when a core is still powered or owned.
 *
 * The request falls back to the top of the table rather than to nothing, so a
 * later instance that powers up before configuring is still covered.
 */
static void venc_release_perf(void *priv, u64 instance)
{
	struct mtk_vcp_venc_hw *hw = priv;

	mutex_lock(&hw->lock);
	if (hw->perf_owner != instance) {
		mutex_unlock(&hw->lock);
		return;
	}
	if (hw->retained) {
		dev_warn(hw->dev, "VENC keeping the %d uV VCORE request: hardware was not confirmed idle\n",
			 hw->active_uv);
		mutex_unlock(&hw->lock);
		return;
	}
	if (hw->powered || hw->core[0].owner || hw->core[1].owner) {
		dev_warn(hw->dev, "VENC keeping the %d uV VCORE request: a core is still owned\n",
			 hw->active_uv);
		mutex_unlock(&hw->lock);
		return;
	}
	venc_vote_idle(hw);
	hw->desired_uv = hw->max_uv;
	hw->perf_owner = 0;
	mutex_unlock(&hw->lock);
}

static const struct mtk_vcp_venc_ops venc_hw_ops = {
	.power = venc_power,
	.wait_irq = venc_wait_irq,
	.alloc = venc_alloc,
	.free = venc_free,
	.buffers_ready = venc_notify,
	.set_perf = venc_set_perf,
	.release_perf = venc_release_perf,
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
	idle = !hw->powered && !hw->retained && !hw->pm_held &&
	       !hw->pm_pending && !hw->core[0].owner && !hw->core[1].owner;
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
	if (!hw->powered) {
		/* Includes partial power-on rollback and failed suspend retries. */
		ret = venc_rails_off(hw);
		goto out;
	}
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
	bool table;
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
	/* Managed table: the OPPs live exactly as long as this device does. */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return ERR_PTR(ret);
	table = !ret;
	hw->vcore = devm_regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(hw->vcore))
		return ERR_CAST(hw->vcore);
	/* A table and the rail it names are only useful together: the table is
	 * what maps a workload to a step, and the rail is what the step is asked
	 * of. A device that declares one without the other is misdescribed, and
	 * venc_set_perf() reports the same mismatch to its callers.
	 */
	if (table != !!hw->vcore) {
		dev_err(dev, "VENC needs both an OPP table and its supply\n");
		return ERR_PTR(-EINVAL);
	}
	if (table) {
		/* Start from the top of the table so a power-up that happens
		 * before any workload is known is still covered by a request.
		 */
		ret = venc_bootstrap_voltage(hw);
		if (ret)
			return ERR_PTR(ret);
	}
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
