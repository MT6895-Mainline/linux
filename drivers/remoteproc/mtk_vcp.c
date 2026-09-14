// SPDX-License-Identifier: GPL-2.0-only
/* MT6895 VCP: bootloader-owned firmware and vendor codec mailboxes. */
#include <linux/arm-smccc.h>
#include <linux/unaligned.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_vcp.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#define VCP_CONTROL             MTK_SIP_SMC_CMD(0x52c)
#define VCP_DAPC_CONTROL        MTK_SIP_SMC_CMD(0x52e)
#define VCP_RESET_SET           2
#define VCP_RESET_RELEASE       3
#define VCP_TCM_SIZE            0x40000
#define VCP_LOADER_SIZE         0x2000
#define VCP_REGION_OFFSET       4
#define VCP_REGION_SIZE         0x38
#define VCP_MBOX_COUNT          5
#define VCP_MBOX_SET            0x100
#define VCP_MBOX_CLR            0x10c
#define VCP_GIPC_SET            0x98
#define VCP_SPM_CLR             0x94
#define VCP_CORE_WDT            0x30
#define VCP_CORE_SHM_ADDR       0x44
#define VCP_CORE_SHM_SIZE       0x48
#define VCP_CORE_REBOOT         0x54
#define VCP_CORE_STATUS         0x70
#define VCP_IOMMU_ID            0x70422 /* table0, domain7, larb33, port2 */
#define VCP_SHM_BASE            0x150000000ULL
#define VCP_SHM_END             0x160000000ULL
#define VCP_CODE_BASE           0x106000000ULL
#define VCP_CODE_END            0x108000000ULL
#define VCP_ENC_MEM_ALLOC       0x3003
#define VCP_ENC_MEM_ALLOC_DONE  0x4003
#define VCP_DEC_MEM_ALLOC       0xc005
#define VCP_DEC_MEM_ALLOC_DONE  0xd005
#define VCP_MEM_SHARED          6

struct vcp_message {
	__le32 id;
	__le32 len;
	u8 data[MTK_VCP_IPI_MAX_PAYLOAD];
};
static_assert(sizeof(struct vcp_message) == 72);

/* xaga vendor ABI: decoder includes ctx_id and an alignment word. */
struct vcp_codec_mem {
	__le32 type, len;
	__le64 iova, pa, va;
};

struct vcp_enc_mem_request {
	__le32 id, status;
	__le64 instance;
	struct vcp_codec_mem mem;
	__le32 property, log;
};

struct vcp_dec_mem_request {
	__le32 id, context, status, padding;
	__le64 instance;
	struct vcp_codec_mem mem;
	__le32 property, log;
};
static_assert(sizeof(struct vcp_enc_mem_request) == 56);
static_assert(sizeof(struct vcp_dec_mem_request) == 64);
static_assert(offsetof(struct vcp_enc_mem_request, mem) == 16);
static_assert(offsetof(struct vcp_dec_mem_request, mem) == 24);

struct vcp_region_info {
	__le32 loader_start, loader_size;
	__le32 firmware_start, firmware_size;
	__le32 dram_start, dram_size, dram_backup_start;
	__le32 struct_size;
	__le32 log_uart, task_context, vcpctl;
	__le32 regdump_start, regdump_size, params_start;
};
static_assert(sizeof(struct vcp_region_info) == VCP_REGION_SIZE);

struct vcp_mbox {
	struct mtk_vcp *vcp;
	void __iomem *base;
	unsigned int index;
	int irq;
};

struct mtk_vcp {
	struct device *dev;
	struct rproc *rproc;
	void __iomem *sram, *cfg, *core;
	struct clk_bulk_data clocks[3];
	struct vcp_mbox mbox[VCP_MBOX_COUNT];
	struct device_link *smi_links[2];
	struct mtk_vcp_mem mem[MTK_VCP_MEM_COUNT];
	struct completion ready;
	struct mutex send_lock;
	struct mutex handler_lock;
	mtk_vcp_ipi_handler_t handler[MTK_VCP_CODEC_COUNT];
	void *handler_priv[MTK_VCP_CODEC_COUNT];
	dma_addr_t shm_lower, shm_upper;
	int wdt_irq;
	int boot_error;
	bool powered, tx_enabled, reset_confirmed, security_enabled;
	bool irqs_enabled, stopping, sram_metadata_verified;
};

static u64 vcp_unpack_iova(u32 packed)
{
	return (packed & ~0xfULL) | ((u64)(packed & 0xf) << 32);
}

static u32 vcp_pack_iova(dma_addr_t addr)
{
	return lower_32_bits(addr) | (upper_32_bits(addr) & 0xf);
}

static int vcp_smc(struct mtk_vcp *vcp, u32 function, u32 op, u32 arg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(function, op, arg, 0, 0, 0, 0, 0, &res);
	if ((long)res.a0) {
		dev_err(vcp->dev, "secure call %#x/%u failed: %ld\n",
			function, op, (long)res.a0);
		return -EIO;
	}
	return 0;
}

/* Both ready variants carry TCM size; only READY_1 establishes readiness. */
static void vcp_ready(struct mtk_vcp *vcp, u32 size, bool ready1)
{
	if (size != VCP_TCM_SIZE) {
		WRITE_ONCE(vcp->boot_error, -EPROTO);
		complete(&vcp->ready);
		return;
	}
	if (!ready1)
		return;
	writel(0xff, vcp->cfg + VCP_SPM_CLR);
	complete(&vcp->ready);
}

/* Service-wide SHM requests precede any AP codec instance. */
static bool vcp_codec_shm_request(struct mtk_vcp *vcp, unsigned int codec,
				  const struct vcp_message *msg, u32 len)
{
	union {
		struct vcp_enc_mem_request enc;
		struct vcp_dec_mem_request dec;
	} reply = {};
	struct vcp_codec_mem *mem;
	struct mtk_vcp_mem *shared;
	__le32 *status, *property, *log, id;
	unsigned int shared_id, prop_id, log_id;
	bool encoder = codec == MTK_VCP_ENCODER;
	size_t size = encoder ? sizeof(reply.enc) : sizeof(reply.dec);
	int ret;

	memcpy(&id, msg->data, sizeof(id));
	if (le32_to_cpu(id) != (encoder ? VCP_ENC_MEM_ALLOC : VCP_DEC_MEM_ALLOC))
		return false;
	if (len != size) {
		dev_err(vcp->dev, "codec%u malformed memory request: %u\n", codec, len);
		WRITE_ONCE(vcp->boot_error, -EPROTO);
		complete(&vcp->ready);
		return true;
	}
	memcpy(&reply, msg->data, len);
	if (encoder) {
		mem = &reply.enc.mem;
		status = &reply.enc.status;
		property = &reply.enc.property;
		log = &reply.enc.log;
		shared_id = MTK_VCP_MEM_VENC;
		prop_id = MTK_VCP_MEM_VENC_PROP;
		log_id = MTK_VCP_MEM_VENC_LOG;
		reply.enc.id = cpu_to_le32(VCP_ENC_MEM_ALLOC_DONE);
	} else {
		mem = &reply.dec.mem;
		status = &reply.dec.status;
		property = &reply.dec.property;
		log = &reply.dec.log;
		shared_id = MTK_VCP_MEM_VDEC;
		prop_id = MTK_VCP_MEM_VDEC_PROP;
		log_id = MTK_VCP_MEM_VDEC_LOG;
		reply.dec.id = cpu_to_le32(VCP_DEC_MEM_ALLOC_DONE);
		reply.dec.padding = 0;
	}
	/* Per-instance SW/HW allocations belong to the codec backend. */
	if (le32_to_cpu(mem->type) != VCP_MEM_SHARED)
		return false;
	shared = &vcp->mem[shared_id];
	*status = 0;
	mem->len = cpu_to_le32(shared->size);
	mem->iova = cpu_to_le64(shared->dma);
	mem->pa = mem->iova;
	/* Opaque AP cookie in the vendor protocol, never accepted as a pointer. */
	mem->va = cpu_to_le64((uintptr_t)shared->cpu);
	*property = cpu_to_le32(vcp_pack_iova(vcp->mem[prop_id].dma));
	*log = cpu_to_le32(vcp_pack_iova(vcp->mem[log_id].dma));
	ret = mtk_vcp_ipi_send(vcp, codec, &reply, len);
	if (ret) {
		dev_err(vcp->dev, "codec%u SHM response failed: %d\n", codec, ret);
		WRITE_ONCE(vcp->boot_error, ret);
		complete(&vcp->ready);
	} else {
		dev_info(vcp->dev, "codec%u shared memory supplied: %pad, %zu bytes\n",
			 codec, &shared->dma, shared->size);
	}
	return true;
}

static void vcp_codec_receive(struct mtk_vcp *vcp, unsigned int codec,
			      const void *data)
{
	const struct vcp_message *msg = data;
	u32 len = le32_to_cpu(msg->len);
	u32 service = le32_to_cpu(msg->id);

	if ((codec == MTK_VCP_ENCODER ? service != 3 :
	     (service != 1 && service != 2)) || len < sizeof(u32) ||
	    len > sizeof(msg->data)) {
		dev_warn_ratelimited(vcp->dev, "invalid codec%u message %u/%u\n",
				     codec, le32_to_cpu(msg->id), len);
		return;
	}
	if (vcp_codec_shm_request(vcp, codec, msg, len))
		return;
	mutex_lock(&vcp->handler_lock);
	if (vcp->handler[codec])
		vcp->handler[codec](vcp->handler_priv[codec], msg->data, len);
	else
		dev_warn_ratelimited(vcp->dev,
				     "codec%u request has no protocol handler\n", codec);
	mutex_unlock(&vcp->handler_lock);
}

static irqreturn_t vcp_mbox_irq(int irq, void *priv)
{
	struct vcp_mbox *mbox = priv;
	struct mtk_vcp *vcp = mbox->vcp;
	u32 words[64], status, handled = 0;

	status = readl(mbox->base + VCP_MBOX_CLR);
	if (!status)
		return IRQ_NONE;
	/* Snapshot before acknowledging: firmware may reuse a slot immediately. */
	__ioread32_copy(words, mbox->base, ARRAY_SIZE(words));
	writel(status, mbox->base + VCP_MBOX_CLR);

	switch (mbox->index) {
	case 0: /* channel1: VDEC, after the 18-word send pin */
		handled = BIT(9);
		if (status & handled)
			vcp_codec_receive(vcp, MTK_VCP_DECODER, &words[18]);
		break;
	case 1: /* channel5: READY_0 */
		handled = BIT(8);
		if (status & handled)
			vcp_ready(vcp, le32_to_cpu((__force __le32)words[16]), false);
		break;
	case 2: /* channel10: VENC */
		handled = BIT(11);
		if (status & handled)
			vcp_codec_receive(vcp, MTK_VCP_ENCODER, &words[22]);
		break;
	case 3: /* channel19: READY_1 */
		handled = BIT(15);
		if (status & handled)
			vcp_ready(vcp, le32_to_cpu((__force __le32)words[30]), true);
		break;
	}
	/* These are not codec acknowledgements. Do not invent successful replies. */
	if (status & ~handled)
		dev_warn_ratelimited(vcp->dev, "unhandled mailbox%u pins %#x\n",
				     mbox->index, status & ~handled);
	return IRQ_HANDLED;
}

static irqreturn_t vcp_wdt_irq(int irq, void *priv)
{
	struct mtk_vcp *vcp = priv;
	u32 value;
	int ret;

	if (!(readl(vcp->core + VCP_CORE_WDT) & BIT(0)))
		return IRQ_NONE;
	/* The firmware must finish its interrupt sequence before AP clears WDT. */
	ret = readl_poll_timeout(vcp->core + VCP_CORE_REBOOT, value,
				 value == 0x34, 10, 50000);
	if (ret)
		dev_err(vcp->dev, "watchdog handshake timed out: %#x\n", value);
	udelay(10);
	writel(BIT(0), vcp->core + VCP_CORE_WDT);
	mutex_lock(&vcp->send_lock);
	vcp->tx_enabled = false;
	mutex_unlock(&vcp->send_lock);
	WRITE_ONCE(vcp->boot_error, -EIO);
	complete(&vcp->ready);
	if (!READ_ONCE(vcp->stopping))
		rproc_report_crash(vcp->rproc, RPROC_WATCHDOG);
	return IRQ_HANDLED;
}

static void vcp_disable_irqs(struct mtk_vcp *vcp)
{
	int i;

	if (!vcp->irqs_enabled)
		return;
	for (i = 0; i < VCP_MBOX_COUNT; i++)
		disable_irq(vcp->mbox[i].irq);
	disable_irq(vcp->wdt_irq);
	vcp->irqs_enabled = false;
}

static void vcp_free_memory(struct mtk_vcp *vcp)
{
	int i;

	for (i = MTK_VCP_MEM_COUNT - 1; i >= 0; i--) {
		struct mtk_vcp_mem *mem = &vcp->mem[i];

		if (mem->cpu)
			dma_free_coherent(vcp->dev, mem->size, mem->cpu, mem->dma);
		mem->cpu = NULL;
	}
}

static int vcp_alloc_memory(struct mtk_vcp *vcp)
{
	static const size_t sizes[MTK_VCP_MEM_COUNT] = {
		0x78000, 0x8000, 0x180000, 0x400, 0x400, 0x400, 0x400, 0x100000,
	};
	int i;

	vcp->shm_lower = VCP_SHM_END;
	vcp->shm_upper = VCP_SHM_BASE;
	for (i = 0; i < MTK_VCP_MEM_COUNT; i++) {
		struct mtk_vcp_mem *mem = &vcp->mem[i];

		mem->size = sizes[i];
		mem->cpu = dma_alloc_coherent(vcp->dev, mem->size, &mem->dma,
					     GFP_KERNEL);
		if (!mem->cpu)
			goto fail;
		if (mem->dma < VCP_SHM_BASE || mem->dma >= VCP_SHM_END ||
		    mem->size > VCP_SHM_END - mem->dma || (mem->dma & 0xf)) {
			dev_err(vcp->dev, "shared buffer %d outside VCP DMA domain\n", i);
			vcp_free_memory(vcp);
			return -ERANGE;
		}
		memset(mem->cpu, 0, mem->size);
		vcp->shm_lower = min(vcp->shm_lower, mem->dma);
		vcp->shm_upper = max(vcp->shm_upper, mem->dma + mem->size);
	}
	return 0;
fail:
	vcp_free_memory(vcp);
	return -ENOMEM;
}

static int vcp_prepare(struct rproc *rproc)
{
	struct mtk_vcp *vcp = rproc->priv;
	int ret;

	if (vcp->powered)
		return -EBUSY;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(vcp->clocks), vcp->clocks);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(vcp->dev);
	if (ret < 0)
		goto clocks_off;
	ret = vcp_alloc_memory(vcp);
	if (ret)
		goto power_off;
	/* Keep resources alive if a later secure reset fails. */
	__module_get(THIS_MODULE);
	vcp->powered = true;
	vcp->reset_confirmed = true; /* no shared address has been published yet */
	return 0;
power_off:
	pm_runtime_put_sync(vcp->dev);
clocks_off:
	clk_bulk_disable_unprepare(ARRAY_SIZE(vcp->clocks), vcp->clocks);
	return ret;
}

static int vcp_unprepare(struct rproc *rproc)
{
	struct mtk_vcp *vcp = rproc->priv;

	if (!vcp->reset_confirmed || vcp->security_enabled) {
		dev_err(vcp->dev, "secure shutdown unconfirmed; retain DMA and power until reboot\n");
		return -EBUSY;
	}
	vcp_free_memory(vcp);
	pm_runtime_put_sync(vcp->dev);
	clk_bulk_disable_unprepare(ARRAY_SIZE(vcp->clocks), vcp->clocks);
	vcp->powered = false;
	module_put(THIS_MODULE);
	return 0;
}

static bool vcp_code_range(__le32 packed, __le32 size)
{
	u64 start = vcp_unpack_iova(le32_to_cpu(packed));
	u32 len = le32_to_cpu(size);

	return len && start >= VCP_CODE_BASE && start < VCP_CODE_END &&
	       len <= VCP_CODE_END - start;
}

static int vcp_sanity_check(struct rproc *rproc, const struct firmware *fw)
{
	const u8 *p = fw->data;

	if (fw->size < sizeof(Elf32_Ehdr))
		return -EINVAL;
	if (p[EI_MAG0] != ELFMAG0 || p[EI_MAG1] != ELFMAG1 ||
	    p[EI_MAG2] != ELFMAG2 || p[EI_MAG3] != ELFMAG3 ||
	    p[EI_CLASS] != ELFCLASS32 || p[EI_DATA] != ELFDATA2LSB)
		return -EINVAL;
	if (get_unaligned_le16(p + offsetof(Elf32_Ehdr, e_machine)) != EM_RISCV ||
	    get_unaligned_le32(p + offsetof(Elf32_Ehdr, e_entry)) != 0x1ea00000)
		return -EINVAL;
	return 0;
}

/* Verify the bootstrap without writing SRAM or touching protected physical RAM. */
static int vcp_load(struct rproc *rproc, const struct firmware *fw)
{
	struct mtk_vcp *vcp = rproc->priv;
	struct vcp_region_info info;
	u8 *loader;
	int ret = -EINVAL;

	/* Secure reset closes DAPC access to VCP SRAM. The bootloader-owned
	 * image is not replaced between remoteproc boots, so verify its live
	 * metadata once while SRAM is accessible and retain that result.
	 */
	if (vcp->sram_metadata_verified)
		return 0;

	loader = kmalloc(VCP_LOADER_SIZE, GFP_KERNEL);
	if (!loader)
		return -ENOMEM;
	memcpy_fromio(loader, vcp->sram, VCP_LOADER_SIZE);
	memcpy(&info, loader + VCP_REGION_OFFSET, sizeof(info));
	if (le32_to_cpu(info.struct_size) != sizeof(info) ||
	    le32_to_cpu(info.loader_size) != VCP_LOADER_SIZE ||
	    !vcp_code_range(info.loader_start, info.loader_size) ||
	    !vcp_code_range(info.firmware_start, info.firmware_size) ||
	    !vcp_code_range(info.dram_start, info.dram_size) ||
	    !vcp_code_range(info.dram_backup_start, info.dram_size))
		goto out;
	/* The bootloader owns the RISC-V image; never copy or compare the ELF
	 * file against SRAM. Only the live SRAM metadata is trusted here.
	 */
	dev_info(vcp->dev, "verified VCP SRAM metadata for bootloader-owned RISC-V image\n");
	vcp->sram_metadata_verified = true;
	ret = 0;
out:
	if (ret)
		dev_err(vcp->dev, "VCP SRAM metadata is invalid\n");
	kfree(loader);
	return ret;
}

static int vcp_assert_reset(struct mtk_vcp *vcp, bool graceful)
{
	int ret;

	ret = vcp_smc(vcp, VCP_CONTROL, VCP_RESET_SET, graceful);
	if (ret)
		return ret;
	vcp->reset_confirmed = true;
	if (vcp->security_enabled) {
		ret = vcp_smc(vcp, VCP_DAPC_CONTROL, 0, 0);
		if (!ret)
			vcp->security_enabled = false;
	}
	return ret;
}

static int vcp_start(struct rproc *rproc)
{
	struct mtk_vcp *vcp = rproc->priv;
	unsigned long ready;
	int ret, i, reset_ret;

	reinit_completion(&vcp->ready);
	WRITE_ONCE(vcp->boot_error, 0);
	WRITE_ONCE(vcp->stopping, false);
	ret = clk_set_parent(vcp->clocks[2].clk, vcp->clocks[0].clk);
	if (ret)
		return ret;
	ret = clk_set_parent(vcp->clocks[2].clk, vcp->clocks[1].clk);
	if (ret)
		return ret;
	ret = vcp_smc(vcp, VCP_DAPC_CONTROL, 1, 0);
	if (ret)
		return ret;
	vcp->security_enabled = true;
	/* Establish exclusive ownership before changing the communication window. */
	vcp->reset_confirmed = false;
	ret = vcp_smc(vcp, VCP_CONTROL, VCP_RESET_SET, 0);
	if (ret)
		goto reset;
	vcp->reset_confirmed = true;
	writel(vcp_pack_iova(vcp->shm_lower), vcp->core + VCP_CORE_SHM_ADDR);
	writel(vcp->shm_upper - vcp->shm_lower, vcp->core + VCP_CORE_SHM_SIZE);
	mutex_lock(&vcp->send_lock);
	vcp->tx_enabled = true;
	mutex_unlock(&vcp->send_lock);
	for (i = 0; i < VCP_MBOX_COUNT; i++) {
		void __iomem *base = vcp->mbox[i].base;

		writel(readl(base + VCP_MBOX_CLR), base + VCP_MBOX_CLR);
		enable_irq(vcp->mbox[i].irq);
	}
	enable_irq(vcp->wdt_irq);
	vcp->irqs_enabled = true;
	vcp->reset_confirmed = false;
	dma_wmb();
	ret = vcp_smc(vcp, VCP_CONTROL, VCP_RESET_RELEASE, 1);
	if (ret)
		goto reset;
	ready = wait_for_completion_timeout(&vcp->ready, msecs_to_jiffies(4000));
	ret = ready ? READ_ONCE(vcp->boot_error) : -ETIMEDOUT;
	if (!ret) {
		dev_info(vcp->dev, "VCP READY_1 received; shared IOVA %pad..%pad\n",
			 &vcp->shm_lower, &vcp->shm_upper);
		return 0;
	}
reset:
	WRITE_ONCE(vcp->stopping, true);
	mutex_lock(&vcp->send_lock);
	vcp->tx_enabled = false;
	mutex_unlock(&vcp->send_lock);
	vcp_disable_irqs(vcp);
	reset_ret = vcp_assert_reset(vcp, false);
	return reset_ret ?: ret;
}

static int vcp_stop(struct rproc *rproc)
{
	struct mtk_vcp *vcp = rproc->priv;
	u32 value;
	int ret;

	WRITE_ONCE(vcp->stopping, true);
	mutex_lock(&vcp->send_lock);
	vcp->tx_enabled = false;
	mutex_unlock(&vcp->send_lock);
	vcp_disable_irqs(vcp);
	/* Vendor stop request: wait for both harts, then use secure reset. */
	writel(BIT(17), vcp->cfg + VCP_GIPC_SET);
	ret = readl_poll_timeout(vcp->core + VCP_CORE_REBOOT, value,
				 value == 0x34, 10, 50000);
	if (!ret)
		ret = readl_poll_timeout(vcp->core + VCP_CORE_STATUS, value,
					value == 7, 1000, 500000);
	return vcp_assert_reset(vcp, !ret);
}

static const struct rproc_ops vcp_ops = {
	.prepare = vcp_prepare,
	.unprepare = vcp_unprepare,
	.sanity_check = vcp_sanity_check,
	.load = vcp_load,
	.start = vcp_start,
	.stop = vcp_stop,
};

struct mtk_vcp *mtk_vcp_get(struct device *dev)
{
	struct device_node *node;
	struct platform_device *pdev;
	struct mtk_vcp *vcp;

	node = of_parse_phandle(dev->of_node, "mediatek,vcp", 0);
	if (!node)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	vcp = platform_get_drvdata(pdev);
	if (!vcp || !try_module_get(THIS_MODULE)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return vcp;
}
EXPORT_SYMBOL_GPL(mtk_vcp_get);

void mtk_vcp_put(struct mtk_vcp *vcp)
{
	put_device(vcp->dev);
	module_put(THIS_MODULE);
}
EXPORT_SYMBOL_GPL(mtk_vcp_put);

int mtk_vcp_boot(struct mtk_vcp *vcp)
{
	return rproc_boot(vcp->rproc);
}
EXPORT_SYMBOL_GPL(mtk_vcp_boot);

int mtk_vcp_shutdown(struct mtk_vcp *vcp)
{
	return rproc_shutdown(vcp->rproc);
}
EXPORT_SYMBOL_GPL(mtk_vcp_shutdown);

bool mtk_vcp_is_offline(struct mtk_vcp *vcp)
{
	bool offline;

	mutex_lock(&vcp->rproc->lock);
	offline = vcp->rproc->state == RPROC_OFFLINE && !vcp->powered;
	mutex_unlock(&vcp->rproc->lock);
	return offline;
}
EXPORT_SYMBOL_GPL(mtk_vcp_is_offline);

int mtk_vcp_alloc_workmem(struct mtk_vcp *vcp, size_t size, struct mtk_vcp_mem *mem)
{
	if (!mem || !size || size > VCP_SHM_END - VCP_SHM_BASE)
		return -EINVAL;
	if (!READ_ONCE(vcp->tx_enabled))
		return -EHOSTDOWN;
	mem->size = size;
	mem->cpu = dma_alloc_coherent(vcp->dev, size, &mem->dma, GFP_KERNEL);
	if (!mem->cpu)
		return -ENOMEM;
	if (mem->dma < VCP_SHM_BASE || mem->dma >= VCP_SHM_END ||
	    size > VCP_SHM_END - mem->dma || (mem->dma & 0xf)) {
		dma_free_coherent(vcp->dev, size, mem->cpu, mem->dma);
		mem->cpu = NULL;
		return -ERANGE;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcp_alloc_workmem);

void mtk_vcp_free_workmem(struct mtk_vcp *vcp, struct mtk_vcp_mem *mem)
{
	if (mem->cpu)
		dma_free_coherent(vcp->dev, mem->size, mem->cpu, mem->dma);
	mem->cpu = NULL;
}
EXPORT_SYMBOL_GPL(mtk_vcp_free_workmem);

int mtk_vcp_get_mem(struct mtk_vcp *vcp, unsigned int id, struct mtk_vcp_mem *mem)
{
	if (id >= MTK_VCP_MEM_COUNT)
		return -EINVAL;
	if (!READ_ONCE(vcp->tx_enabled))
		return -EHOSTDOWN;
	*mem = vcp->mem[id];
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcp_get_mem);

int mtk_vcp_ipi_register(struct mtk_vcp *vcp, unsigned int codec,
			 mtk_vcp_ipi_handler_t handler, void *priv)
{
	int ret = 0;

	if (codec >= MTK_VCP_CODEC_COUNT || !handler)
		return -EINVAL;
	mutex_lock(&vcp->handler_lock);
	if (vcp->handler[codec])
		ret = -EBUSY;
	else {
		vcp->handler_priv[codec] = priv;
		vcp->handler[codec] = handler;
	}
	mutex_unlock(&vcp->handler_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_ipi_register);

void mtk_vcp_ipi_unregister(struct mtk_vcp *vcp, unsigned int codec)
{
	if (codec >= MTK_VCP_CODEC_COUNT)
		return;
	mutex_lock(&vcp->handler_lock);
	vcp->handler[codec] = NULL;
	vcp->handler_priv[codec] = NULL;
	mutex_unlock(&vcp->handler_lock);
}
EXPORT_SYMBOL_GPL(mtk_vcp_ipi_unregister);

static int vcp_ipi_send(struct mtk_vcp *vcp, unsigned int codec,
			u32 service, const void *data, size_t len)
{
	struct vcp_message msg = {};
	void __iomem *base;
	u32 status;
	int ret;

	if (codec >= MTK_VCP_CODEC_COUNT || !data || len < sizeof(u32) ||
	    len > sizeof(msg.data))
		return -EINVAL;
	base = vcp->mbox[codec == MTK_VCP_ENCODER ? 2 : 0].base;
	msg.id = cpu_to_le32(service);
	msg.len = cpu_to_le32(len);
	memcpy(msg.data, data, len);
	mutex_lock(&vcp->send_lock);
	if (!vcp->tx_enabled) {
		ret = -EHOSTDOWN;
		goto out;
	}
	ret = readl_poll_timeout(base + VCP_MBOX_SET, status, !(status & BIT(0)),
				50, 100000);
	if (ret)
		goto out;
	memcpy_toio(base, &msg, sizeof(msg));
	/* Publish the mailbox payload before ringing the doorbell. */
	wmb();
	writel(BIT(0), base + VCP_MBOX_SET);
	/* Transport receipt only; the codec backend must match the real RPC reply. */
	ret = readl_poll_timeout(base + VCP_MBOX_SET, status, !(status & BIT(0)),
				50, 100000);
out:
	mutex_unlock(&vcp->send_lock);
	return ret;
}
int mtk_vcp_ipi_send(struct mtk_vcp *vcp, unsigned int codec,
		     const void *data, size_t len)
{
	return vcp_ipi_send(vcp, codec, codec == MTK_VCP_ENCODER ? 3 : 1,
			    data, len);
}
EXPORT_SYMBOL_GPL(mtk_vcp_ipi_send);

int mtk_vcp_vdec_resource_send(struct mtk_vcp *vcp,
			       const void *data, size_t len)
{
	/* FRAME_BUFFER uses the same mailbox pin but a separate FW service.
	 * Its replies are delivered to the registered decoder handler.
	 */
	return vcp_ipi_send(vcp, MTK_VCP_DECODER, 2, data, len);
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_resource_send);

static void vcp_delete_links(void *data)
{
	struct mtk_vcp *vcp = data;
	int i;

	for (i = 0; i < ARRAY_SIZE(vcp->smi_links); i++)
		if (vcp->smi_links[i])
			device_link_del(vcp->smi_links[i]);
}

static int vcp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iommu_fwspec *fwspec = dev_iommu_fwspec_get(dev);
	struct rproc *rproc;
	struct mtk_vcp *vcp;
	struct resource *res;
	int i, ret;

	if (!fwspec || fwspec->num_ids != 1 || fwspec->ids[0] != VCP_IOMMU_ID ||
	    !iommu_get_domain_for_dev(dev))
		return dev_err_probe(dev, -EINVAL, "VCP requires its domain7 DMA context\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(34));
	if (ret)
		return ret;
	rproc = devm_rproc_alloc(dev, "mt6895-vcp", &vcp_ops,
				"mediatek/mt6895/vcp.img", sizeof(*vcp));
	if (!rproc)
		return -ENOMEM;
	rproc->auto_boot = false;
	rproc->recovery_disabled = true;
	vcp = rproc->priv;
	vcp->rproc = rproc;
	vcp->dev = dev;
	mutex_init(&vcp->send_lock);
	mutex_init(&vcp->handler_lock);
	init_completion(&vcp->ready);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!res || resource_size(res) != VCP_TCM_SIZE)
		return -EINVAL;
	vcp->sram = devm_ioremap_resource(dev, res);
	if (IS_ERR(vcp->sram))
		return PTR_ERR(vcp->sram);
	vcp->cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(vcp->cfg))
		return PTR_ERR(vcp->cfg);
	vcp->core = devm_platform_ioremap_resource_byname(pdev, "core0");
	if (IS_ERR(vcp->core))
		return PTR_ERR(vcp->core);
	vcp->clocks[0].id = "mmup-26m";
	vcp->clocks[1].id = "mmup-clk";
	vcp->clocks[2].id = "mmup-sel";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(vcp->clocks), vcp->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "VCP clocks unavailable\n");
	ret = devm_add_action_or_reset(dev, vcp_delete_links, vcp);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(vcp->smi_links); i++) {
		struct device_node *node;
		struct platform_device *smi;

		node = of_parse_phandle(dev->of_node, "mediatek,smi", i);
		if (!node)
			return -EINVAL;
		smi = of_find_device_by_node(node);
		of_node_put(node);
		if (!smi)
			return -EPROBE_DEFER;
		if (!device_is_bound(&smi->dev)) {
			put_device(&smi->dev);
			return -EPROBE_DEFER;
		}
		vcp->smi_links[i] = device_link_add(dev, &smi->dev,
					DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);
		put_device(&smi->dev);
		if (!vcp->smi_links[i])
			return -ENOMEM;
	}
	for (i = 0; i < VCP_MBOX_COUNT; i++) {
		struct vcp_mbox *mbox = &vcp->mbox[i];
		char name[8];

		snprintf(name, sizeof(name), "mbox%d", i);
		mbox->vcp = vcp;
		mbox->index = i;
		mbox->base = devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(mbox->base))
			return PTR_ERR(mbox->base);
		mbox->irq = platform_get_irq_byname(pdev, name);
		if (mbox->irq < 0)
			return mbox->irq;
		ret = devm_request_threaded_irq(dev, mbox->irq, NULL, vcp_mbox_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, dev_name(dev), mbox);
		if (ret)
			return ret;
	}
	vcp->wdt_irq = platform_get_irq_byname(pdev, "wdt");
	if (vcp->wdt_irq < 0)
		return vcp->wdt_irq;
	ret = devm_request_threaded_irq(dev, vcp->wdt_irq, NULL, vcp_wdt_irq,
				       IRQF_ONESHOT | IRQF_NO_AUTOEN,
				       dev_name(dev), vcp);
	if (ret)
		return ret;
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = rproc_add(rproc);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, vcp);
	return 0;
}

static void vcp_remove(struct platform_device *pdev)
{
	struct mtk_vcp *vcp = platform_get_drvdata(pdev);

	rproc_del(vcp->rproc);
}

static int vcp_suspend(struct device *dev)
{
	struct mtk_vcp *vcp = dev_get_drvdata(dev);

	/* Suspend/resume protocol is not interchangeable with a cold secure boot. */
	return vcp->powered ? -EBUSY : 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(vcp_pm_ops, vcp_suspend, NULL);
static const struct of_device_id vcp_of_match[] = {
	{ .compatible = "mediatek,mt6895-vcp" },
	{}
};
MODULE_DEVICE_TABLE(of, vcp_of_match);

static struct platform_driver vcp_driver = {
	.probe = vcp_probe,
	.remove = vcp_remove,
	.driver = {
		.name = "mt6895-vcp",
		.of_match_table = vcp_of_match,
		.pm = pm_sleep_ptr(&vcp_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(vcp_driver);
MODULE_DESCRIPTION("MT6895 secure VCP boot and codec transport");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("mediatek/mt6895/vcp-loader.bin");
