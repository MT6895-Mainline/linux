// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/completion.h>
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
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include "mtk_vcp_vdec_hw.h"

#define VDEC_CORES 2
#define VDEC_BREAK 0x100
#define VDEC_BREAK_STATUS 0x104
#define VDEC_BREAK_IDLE 0x11
#define VDEC_SW_RESET 0x108
#define VDEC_UFO_CONTROL 0x1c
#define VDEC_UFO_STATUS 0x8c
#define VDEC_UFO_IDLE 0x11000

/* Highest operating point of the vendor OPP table: 660 MHz at 750 mV. */
#define VDEC_MAX_RATE 660000000UL

/* Hardware tracing is opt-in through dynamic debug. */
#define VCPDBG(fmt, ...) pr_debug("VCPDBG:%s: " fmt, __func__, ##__VA_ARGS__)

struct vdec_core {
	void __iomem *misc, *vld;
	struct device *domain, *larb;
	struct completion irq_done;
	int irq;
	bool owned;
};
struct mtk_vcp_vdec_hw {
	struct device *dev, *ube;
	struct mtk_vcp *vcp;
	void __iomem *ufo;
	struct clk_bulk_data clocks[3];
	struct vdec_core core[2];
	struct regulator *vcore;
	/*
	 * The step the current stream needs, and what the rail is asked for right
	 * now. The request follows desired_uv across power cycles: firmware powers
	 * the cores down between frames, so a power-down must not cost the session
	 * the step it was granted.
	 */
	unsigned long desired_uv;
	int active_uv;
	/* Highest step of the declared table, used as the request until a stream
	 * names the step it actually needs.
	 */
	unsigned long max_uv;
	/* PM references are acquired domains first, then LARBs. A failed put
	 * consumes its reference, so pending suspend retries must not put again.
	 * Keep the module and rail until all acquired resources are released.
	 */
	unsigned long pm_held, pm_pending;
	bool pm_ref;
	bool retained;
	struct mutex perf_lock;
	bool powered, uncertain;
};

static irqreturn_t vdec_irq(int irq, void *priv)
{
	struct vdec_core *c = priv;
	u32 status = readl(c->misc + 0xa4);

	if (!(status & BIT(16)))
		return IRQ_NONE;
	VCPDBG("irq: irq=%d status=%#x\n", c->irq, status);
	writel(status | 0x11, c->misc + 0xa4);
	writel(readl(c->misc + 0xa4) & ~0x10, c->misc + 0xa4);
	complete(&c->irq_done);
	return IRQ_HANDLED;
}

/*
 * Ask the shared rail for the step the current stream needs. Called with
 * perf_lock held and before the gates are enabled, so the cores never come up
 * below the step this session was granted.
 *
 * The rail serves the highest request among its clients, so raising this
 * codec's floor cannot lower anyone else's and re-asking for the same step is
 * a no-op.
 */
static int vdec_vote_apply(struct mtk_vcp_vdec_hw *hw)
{
	int ret;

	if (!hw->vcore || !hw->desired_uv || hw->active_uv == (int)hw->desired_uv)
		return 0;
	ret = regulator_set_voltage(hw->vcore, hw->desired_uv, INT_MAX);
	if (ret) {
		dev_err(hw->dev, "VDEC %lu uV VCORE request failed: %d\n",
			hw->desired_uv, ret);
		return ret;
	}
	hw->active_uv = hw->desired_uv;
	dev_dbg(hw->dev, "VDEC VCORE request: %lu uV\n", hw->desired_uv);
	return 0;
}

/*
 * Highest step of the declared table. Firmware powers the cores up on its own
 * schedule, including during INIT before any stream geometry is known, so the
 * request starts here and is only narrowed once a stream names its step.
 * Without this the rail would be asked for nothing at all in that window.
 */
static int vdec_bootstrap_voltage(struct mtk_vcp_vdec_hw *hw)
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

/* perf_lock held. Only safe once both cores are idle. */
static void vdec_vote_idle(struct mtk_vcp_vdec_hw *hw)
{
	int ret;

	if (!hw->vcore || !hw->active_uv)
		return;
	if (hw->retained) {
		dev_warn(hw->dev, "VDEC keeping the %d uV VCORE request: hardware was not confirmed idle\n",
			 hw->active_uv);
		return;
	}
	ret = regulator_set_voltage(hw->vcore, 0, INT_MAX);
	if (ret) {
		dev_warn(hw->dev, "VDEC VCORE request release failed: %d (keeping %d uV)\n",
			 ret, hw->active_uv);
		return;
	}
	dev_dbg(hw->dev, "VDEC VCORE request idle (was %d uV)\n", hw->active_uv);
	hw->active_uv = 0;
}

/* Engines must be idle, or this must be rollback before their clocks were
 * enabled. Stop at the first failure: parent domains must remain referenced
 * while a LARB's shutdown is unconfirmed. PM serializes each device's retry
 * and preserves fatal runtime_error; never override that state here.
 */
static int vdec_pm_release(struct mtk_vcp_vdec_hw *hw)
{
	int i, ret;

	for (i = 2 * VDEC_CORES - 1; i >= 0; i--) {
		struct device *dev = i < VDEC_CORES ? hw->core[i].domain :
				    hw->core[i - VDEC_CORES].larb;

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
			dev_warn(hw->dev, "VDEC PM resource %d suspend failed: %d; retaining resources\n",
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

static int vdec_power_on(struct mtk_vcp_vdec_hw *hw)
{
	int domains = 0, larbs = 0, i, ret;

	if (hw->retained)
		return -EIO;
	if (hw->powered)
		return 0;
	for (i = 0; i < 3; i++) {
		unsigned long rate = clk_get_rate(hw->clocks[i].clk);

		if (!rate || rate > VDEC_MAX_RATE)
			return -ERANGE;
	}
	/* Raise the rail before the gates. Firmware powers the cores down
	 * between frames, so the step is restored on every power-up rather than
	 * only when the stream was parsed.
	 */
	mutex_lock(&hw->perf_lock);
	ret = vdec_vote_apply(hw);
	mutex_unlock(&hw->perf_lock);
	if (ret)
		return ret;
	__module_get(THIS_MODULE);
	hw->pm_ref = true;
	for (; domains < 2; domains++) {
		ret = pm_runtime_resume_and_get(hw->core[domains].domain);
		if (ret < 0)
			goto rollback;
		hw->pm_held |= BIT(domains);
	}
	for (; larbs < 2; larbs++) {
		ret = pm_runtime_resume_and_get(hw->core[larbs].larb);
		if (ret < 0)
			goto rollback;
		hw->pm_held |= BIT(VDEC_CORES + larbs);
	}
	ret = clk_bulk_prepare_enable(3, hw->clocks);
	if (ret)
		goto rollback;
	hw->powered = true;
	return 0;
rollback:
	/* Preserve the bring-up error. Failed rollback is recorded separately
	 * and session teardown will retry it without consuming another PM ref.
	 */
	vdec_pm_release(hw);
	return ret;
}

int mtk_vcp_vdec_hw_power(struct mtk_vcp_vdec_hw *hw, unsigned int core, bool on)
{
	int ret;

	VCPDBG("power: core%u on=%d (owned=%d/%d retained=%d uncertain=%d)\n",
	       core, on, hw->core[0].owned, hw->core[1].owned, hw->retained,
	       hw->uncertain);
	if (core >= 2 || hw->uncertain)
		return -EINVAL;
	if (on) {
		/* A shutdown that did not complete left the cores in an unknown
		 * state; powering them again would run at a step this driver cannot
		 * vouch for.
		 */
		if (hw->retained) {
			VCPDBG("power: core%u refused, previous shutdown uncertain\n",
			       core);
			return -EIO;
		}
		if (hw->core[core].owned) {
			VCPDBG("power: core%u already owned\n", core);
			return -EBUSY;
		}
		ret = vdec_power_on(hw);
		if (ret) {
			VCPDBG("power: core%u bring-up failed: %d\n", core, ret);
			return ret;
		}
	} else if (!hw->core[core].owned) {
		VCPDBG("power: core%u was not owned\n", core);
		return -EINVAL;
	}
	/* Firmware owns the interrupt except for explicit WAITISR requests.
	 * Keep the rails until the complete session has been deinitialized.
	 */
	hw->core[core].owned = on;
	VCPDBG("power: core%u now %s\n", core, on ? "running" : "stopped");
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
	VCPDBG("wait: core%u -> %d\n", core, ret);
	if (ret) {
		VCPDBG("wait: core%u interrupt timed out, state now uncertain\n",
		       core);
		hw->uncertain = true;
	}
	return ret;
}

/*
 * Ask the shared DVFSRC rail for the operating point this stream needs.
 *
 * The frame geometry and the stream's frame rate give a pixel rate, and the OPP
 * table maps that to the VCORE step the DVFSRC needs, so the smallest step at
 * or above it is the one that is asked for. That mapping is a property of the
 * table: no pixel-per-clock ratio is assumed here, and a stream above the top
 * step is reported as -ERANGE rather than decoded at a point nobody asked for.
 *
 * Only the voltage is requested: the mux parents belong to the DVFSRC
 * provider, which coordinates them with the other multimedia clients, so this
 * driver never calls clk_set_rate() or clk_set_parent(). Its clocks are kept
 * for gate control and for the rate check in vdec_power_on().
 *
 * The step is held until the session stops existing, not just until the next
 * power-down, because firmware powers the cores down between frames.
 */
int mtk_vcp_vdec_hw_set_perf(struct mtk_vcp_vdec_hw *hw, u32 width, u32 height,
				     u32 fps)
{
	struct dev_pm_opp *opp;
	unsigned long hz, volt, previous;
	u64 pixels;
	int ret;

	VCPDBG("perf: requested %ux%u at %u fps\n", width, height, fps);
	if (!width || !height || !fps) {
		VCPDBG("perf: incomplete geometry\n");
		return -EINVAL;
	}
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
		dev_err(hw->dev, "VDEC %ux%u@%u: no operating point at or above a %llu pixel/s rate: %d\n",
			width, height, fps, pixels, ret);
		return ret;
	}
	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	if (!volt || volt > INT_MAX)
		return -EINVAL;

	mutex_lock(&hw->perf_lock);
	/* Hardware state is uncertain after a failed shutdown, so no step can be
	 * guaranteed from here on.
	 */
	if (hw->retained) {
		ret = -EIO;
		goto out;
	}
	if (hw->desired_uv == volt && hw->active_uv == (int)volt) {
		ret = 0;
		goto out;
	}
	/* Every step that is not already in force is written through, including
	 * a lower one: the rail serves the highest request among all of its
	 * clients, so lowering this codec's own floor cannot pull anyone else
	 * down.
	 */
	previous = hw->desired_uv;
	hw->desired_uv = volt;
	ret = regulator_set_voltage(hw->vcore, volt, INT_MAX);
	if (ret) {
		hw->desired_uv = previous;
		goto out;
	}
	hw->active_uv = volt;
out:
	mutex_unlock(&hw->perf_lock);

	if (ret)
		dev_warn(hw->dev, "VDEC perf failed: %ux%u@%u -> %lu uV: %d\n",
			 width, height, fps, volt, ret);
	else
		dev_dbg(hw->dev, "VDEC perf: %ux%u@%u -> %llu pixel/s, %lu uV\n",
			 width, height, fps, pixels, volt);
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

/* MT6895 xaga vendor mtk_vcodec_dec_hw_break(): request MISC break,
 * wait for both idle bits (and UFO when enabled), then pulse VLD reset.
 * Unlike the vendor timeout path, never reset or release DMA on timeout.
 * Firmware must be offline so no callback can restart an engine mid-reset.
 */
static int vdec_break_core(struct mtk_vcp_vdec_hw *hw, unsigned int index)
{
	struct vdec_core *core = &hw->core[index];
	u32 value;
	bool ufo = !index && (readl(hw->ufo + VDEC_UFO_STATUS) & BIT(0));
	int ret;

	writel(readl(core->misc + VDEC_BREAK) | BIT(0), core->misc + VDEC_BREAK);
	if (ufo)
		writel(readl(hw->ufo + VDEC_UFO_CONTROL) & ~BIT(1),
		       hw->ufo + VDEC_UFO_CONTROL);
	ret = readl_poll_timeout(core->misc + VDEC_BREAK_STATUS, value,
				(value & VDEC_BREAK_IDLE) == VDEC_BREAK_IDLE,
				10, 1000000);
	if (ret)
		return ret;
	if (ufo) {
		ret = readl_poll_timeout(hw->ufo + VDEC_UFO_STATUS, value,
					(value & VDEC_UFO_IDLE) == VDEC_UFO_IDLE,
					10, 1000000);
		if (ret)
			return ret;
		writel(readl(hw->ufo + VDEC_UFO_CONTROL) | BIT(1),
		       hw->ufo + VDEC_UFO_CONTROL);
	}
	writel(1, core->vld + VDEC_SW_RESET);
	writel(0, core->vld + VDEC_SW_RESET);
	/* Complete the reset write before disabling its clock or PM domain. */
	readl(core->vld + VDEC_SW_RESET);
	reinit_completion(&core->irq_done);
	core->owned = false;
	return 0;
}

static int vdec_recover_engines(struct mtk_vcp_vdec_hw *hw)
{
	int i, ret;

	if (!hw->uncertain && !hw->core[0].owned && !hw->core[1].owned)
		return 0;
	if (!hw->powered || !mtk_vcp_is_offline(hw->vcp))
		return -EBUSY;
	/* Stop LAT before CORE. Mark the whole engine set uncertain until both
	 * resets complete; a partial failure must keep clocks, DMA and PM refs.
	 */
	hw->uncertain = true;
	for (i = VDEC_CORES - 1; i >= 0; i--) {
		ret = vdec_break_core(hw, i);
		if (ret) {
			dev_err(hw->dev, "VDEC core%d break failed: %d; retaining DMA\n", i, ret);
			return ret;
		}
	}
	hw->uncertain = false;
	return 0;
}

int mtk_vcp_vdec_hw_stop(struct mtk_vcp_vdec_hw *hw)
{
	int ret;

	VCPDBG("stop: powered=%d owned=%d/%d retained=%d uncertain=%d\n",
	       hw->powered, hw->core[0].owned, hw->core[1].owned, hw->retained,
	       hw->uncertain);
	ret = vdec_recover_engines(hw);
	if (ret)
		return ret;
	if (hw->powered) {
		clk_bulk_disable_unprepare(3, hw->clocks);
		hw->powered = false;
	}
	ret = vdec_pm_release(hw);
	if (ret)
		return ret;
	mutex_lock(&hw->perf_lock);
	vdec_vote_idle(hw);
	hw->desired_uv = hw->max_uv;
	mutex_unlock(&hw->perf_lock);
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
	bool table;
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
	/* Managed table: the OPPs live exactly as long as this device does. */
	ret = devm_pm_opp_of_add_table(dev);
	if (ret && ret != -ENODEV)
		return ERR_PTR(ret);
	table = !ret;
	mutex_init(&hw->perf_lock);
	hw->vcore = devm_regulator_get_optional(dev, "dvfsrc-vcore");
	if (IS_ERR(hw->vcore))
		return ERR_CAST(hw->vcore);
	/* A table and the rail it names are only useful together: the table is
	 * what maps a stream to a step, and the rail is what the step is asked
	 * of. A device that declares one without the other is misdescribed, and
	 * set_perf() reports the same mismatch to its callers.
	 */
	if (table != !!hw->vcore) {
		dev_err(dev, "VDEC needs both an OPP table and its supply\n");
		return ERR_PTR(-EINVAL);
	}
	if (table) {
		/* Start from the top of the table so a power-up that happens
		 * before any stream geometry is known is still covered by a
		 * request.
		 */
		ret = vdec_bootstrap_voltage(hw);
		if (ret)
			return ERR_PTR(ret);
	}
	hw->ufo = devm_platform_ioremap_resource_byname(pdev, "base");
	if (IS_ERR(hw->ufo))
		return ERR_CAST(hw->ufo);
	hw->ufo += 0x800;
	for (i = 0; i < 2; i++) {
		struct vdec_core *c = &hw->core[i];
		struct device_node *node;
		struct platform_device *larb;

		c->misc = devm_platform_ioremap_resource_byname(pdev, i ? "lat-misc" : "misc");
		if (IS_ERR(c->misc))
			return ERR_PTR(PTR_ERR(c->misc));
		c->vld = devm_platform_ioremap_resource_byname(pdev, i ? "lat-vld" : "vld");
		if (IS_ERR(c->vld))
			return ERR_CAST(c->vld);
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
