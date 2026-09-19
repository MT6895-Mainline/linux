// SPDX-License-Identifier: GPL-2.0-only
/* MT6895 vendor VENC RPCs. V4L2 and hardware resource management are callers. */
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <media/videobuf2-v4l2.h>

#include "mtk_vcp_venc_abi.h"
#include "mtk_vcp_vdec_abi.h"
#include "mtk_vcp_venc_dma.h"

#define VCP_VENC_RPC_TIMEOUT_MS 5000
#define VCP_VENC_SW_BASE 0x150000000ULL
#define VCP_VENC_SW_END 0x160000000ULL
#define VCP_VENC_MAX_ALLOCATIONS 128
#define VCP_VENC_MAX_WORK_BYTES SZ_256M
#define VCP_VENC_MAX_BUFFER_BYTES SZ_256M

struct venc_allocation {
	struct list_head list;
	struct mtk_vcp_mem mem;
	u64 cookie;
	u32 type;
};

struct venc_output {
	u64 cookie;
	u32 size;
};

struct mtk_vcp_venc_inst {
	struct list_head list, allocations, dma_buffers;
	struct mtk_vcp_venc *enc;
	u64 cookie;
	u32 firmware_instance, expected, codec_id;
	struct vcp_venc_vsi *vsi;
	struct completion reply;
	u8 response[MTK_VCP_IPI_MAX_PAYLOAD] __aligned(8);
	size_t response_len;
	int error;
	bool initialized, configured, broken, submitted;
	bool synchronous;
	bool raw_buffers, managed_buffers;
	size_t work_bytes;
	u32 allocation_count;
	size_t buffer_bytes;
	u32 buffer_count, input_size[VCP_VENC_PLANES];
	unsigned long cores;
	u64 frames[VCP_VENC_BUFFERS];
	struct venc_output outputs[VCP_VENC_BUFFERS];
	struct vcp_venc_result done[VCP_VENC_BUFFERS];
	u32 done_read, done_count;
};

struct mtk_vcp_venc {
	struct device *dev, *bitstream_dev;
	struct mtk_vcp *vcp;
	const struct mtk_vcp_venc_ops *ops;
	void *priv;
	/* API serialization is separate from callback/instance lifetime locking. */
	struct mutex api_lock, rx_lock;
	struct list_head instances;
	u64 next_cookie;
};

/* api_lock held; the IRQ handler never touches DMA attachment records. */
static void venc_release_dma(struct mtk_vcp_venc_inst *inst, u64 cookie, bool all)
{
	struct vcp_venc_dma_buffer *buffer, *next;
	unsigned int i;

	list_for_each_entry_safe(buffer, next, &inst->dma_buffers, list) {
		if (!all && buffer->cookie != cookie)
			continue;
		for (i = 0; i < buffer->planes; i++)
			inst->buffer_bytes -= buffer->plane[i].size;
		inst->buffer_count--;
		list_del(&buffer->list);
		vcp_venc_dma_release(buffer);
	}
}

static void venc_fail(struct mtk_vcp_venc_inst *inst, int error)
{
	inst->broken = true;
	inst->error = error;
	complete(&inst->reply);
}

static void *venc_shared_pointer(struct mtk_vcp_venc_inst *inst, u32 address,
				 size_t size)
{
	struct mtk_vcp_mem mem;
	u32 start, offset;

	if (mtk_vcp_get_mem(inst->enc->vcp, MTK_VCP_MEM_VENC, &mem))
		return ERR_PTR(-EHOSTDOWN);
	start = mem.dma & 0x0fffffff;
	address &= 0x0fffffff;
	if (address < start)
		return ERR_PTR(-ERANGE);
	offset = address - start;
	if (offset > mem.size || size > mem.size - offset || (offset & 3))
		return ERR_PTR(-ERANGE);
	return mem.cpu + offset;
}

static int venc_collect_buffers(struct mtk_vcp_venc_inst *inst)
{
	struct vcp_venc_ring *ring;
	u32 read, write, count, i, j;

	if (!inst->vsi)
		return -EPROTO;
	ring = &inst->vsi->free;
	dma_rmb();
	read = le32_to_cpu(READ_ONCE(ring->read));
	write = le32_to_cpu(READ_ONCE(ring->write));
	count = le32_to_cpu(READ_ONCE(ring->count));
	dev_info(inst->enc->dev,
		 "VENC RING collect: cookie=%#llx read=%u write=%u count=%u done=%u\n",
		 inst->cookie, read, write, count, inst->done_count);
	if (read >= VCP_VENC_BUFFERS || write >= VCP_VENC_BUFFERS ||
	    count > VCP_VENC_BUFFERS || count > VCP_VENC_BUFFERS - inst->done_count ||
	    (read + count) % VCP_VENC_BUFFERS != write)
		return -EPROTO;
	for (i = 0; i < count; i++) {
		struct vcp_venc_result out = {
			.bitstream_cookie = le64_to_cpu(ring->bitstream[read]),
			.frame_cookie = le64_to_cpu(ring->frame[read]),
			.bytes = le32_to_cpu(ring->bytes[read]),
			.keyframe = !!le32_to_cpu(ring->keyframe[read]),
		};
		int bs = -1, frame = -1;

		dev_info(inst->enc->dev,
			 "VENC RING item: index=%u frame=%#llx bitstream=%#llx bytes=%u keyframe=%u\n",
			 read, out.frame_cookie, out.bitstream_cookie, out.bytes,
			 out.keyframe);

		if (!out.bitstream_cookie && !out.frame_cookie)
			return -EPROTO;
		for (j = 0; j < VCP_VENC_BUFFERS; j++) {
			if (out.bitstream_cookie && inst->outputs[j].cookie == out.bitstream_cookie)
				bs = j;
			if (out.frame_cookie && inst->frames[j] == out.frame_cookie)
				frame = j;
		}
		if ((out.bitstream_cookie && bs < 0) || (out.frame_cookie && frame < 0) ||
		    (bs >= 0 && out.bytes > inst->outputs[bs].size) ||
		    (bs < 0 && out.bytes))
			return -EPROTO;
		if (bs >= 0)
			inst->outputs[bs].cookie = 0;
		if (frame >= 0)
			inst->frames[frame] = 0;
		inst->done[(inst->done_read + inst->done_count++) % VCP_VENC_BUFFERS] = out;
		read = (read + 1) % VCP_VENC_BUFFERS;
	}
	WRITE_ONCE(ring->read, cpu_to_le32(read));
	WRITE_ONCE(ring->count, 0);
	dma_wmb();
	dev_info(inst->enc->dev,
		 "VENC RING collected: cookie=%#llx read=%u write=%u count=%u done=%u\n",
		 inst->cookie, read, write, 0, inst->done_count);
	return count;
}

static int venc_memory_service(struct mtk_vcp_venc_inst *inst,
			       struct vcp_venc_mem_msg *msg, bool allocate)
{
	struct mtk_vcp_venc *enc = inst->enc;
	struct venc_allocation *buf;
	u32 type = le32_to_cpu(msg->type), size = le32_to_cpu(msg->len);
	int ret;

	if (!allocate) {
		list_for_each_entry(buf, &inst->allocations, list) {
			if (buf->cookie != le64_to_cpu(msg->cookie))
				continue;
			if (buf->type != type || buf->mem.size != size ||
			    buf->mem.dma != le64_to_cpu(msg->iova) ||
			    buf->mem.dma != le64_to_cpu(msg->pa))
				return -EINVAL;
			inst->work_bytes -= buf->mem.size;
			inst->allocation_count--;
			enc->ops->free(enc->priv, type, &buf->mem);
			list_del(&buf->list);
			kfree(buf);
			return 0;
		}
		return -ENOENT;
	}
	if (type > 1)
		return -EOPNOTSUPP;
	if (!size || size > SZ_128M || enc->next_cookie == U64_MAX)
		return -ERANGE;
	if (inst->allocation_count >= VCP_VENC_MAX_ALLOCATIONS ||
	    size > VCP_VENC_MAX_WORK_BYTES - inst->work_bytes)
		return -ENOMEM;
	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;
	ret = enc->ops->alloc(enc->priv, type, size, &buf->mem);
	if (ret)
		goto free_record;
	if (!buf->mem.cpu || buf->mem.size != size || (buf->mem.dma & 0xf) ||
	    buf->mem.dma >= BIT_ULL(34) || size > BIT_ULL(34) - buf->mem.dma ||
	    (!type && (buf->mem.dma < VCP_VENC_SW_BASE ||
		       buf->mem.dma >= VCP_VENC_SW_END ||
		       size > VCP_VENC_SW_END - buf->mem.dma))) {
		ret = -ERANGE;
		enc->ops->free(enc->priv, type, &buf->mem);
		goto free_record;
	}
	memset(buf->mem.cpu, 0, size);
	buf->type = type;
	buf->cookie = ++enc->next_cookie;
	list_add_tail(&buf->list, &inst->allocations);
	inst->work_bytes += size;
	inst->allocation_count++;
	msg->iova = cpu_to_le64(buf->mem.dma);
	msg->pa = msg->iova;
	msg->cookie = cpu_to_le64(buf->cookie);
	dma_wmb();
	return 0;
free_record:
	kfree(buf);
	return ret;
}

static int venc_service(struct mtk_vcp_venc_inst *inst, u32 id,
			const void *data, size_t len)
{
	struct mtk_vcp_venc *enc = inst->enc;
	union {
		struct vcp_venc_mem_msg mem;
		struct vcp_venc_service hw;
	} response = {};
	u32 irq_status = 0, request_value;
	int core, ret = 0, send_ret;
	bool memory = id == VCP_ENC_ALLOC || id == VCP_ENC_FREE;

	if (len != (memory ? sizeof(response.mem) : sizeof(response.hw)))
		return -EPROTO;
	memcpy(&response, data, len);
	core = (s32)le32_to_cpu(response.hw.hdr.status);
	request_value = le32_to_cpu(response.hw.codec_or_irq);
	if (memory) {
		ret = venc_memory_service(inst, &response.mem, id == VCP_ENC_ALLOC);
		response.mem.hdr.status = cpu_to_le32(ret);
		if (ret && id == VCP_ENC_ALLOC) {
			response.mem.iova = 0;
			response.mem.pa = 0;
			response.mem.cookie = 0;
		}
	} else {
		switch (id) {
		case VCP_ENC_POWER_ON:
		case VCP_ENC_POWER_OFF:
			if (core < 0 || core > 1 ||
			    (id == VCP_ENC_POWER_OFF && !(inst->cores & BIT(core)))) {
				ret = -EINVAL;
				break;
			}
			if (id == VCP_ENC_POWER_ON && (inst->cores & BIT(core)))
				break;
			ret = enc->ops->power(enc->priv, inst->cookie, core,
					     id == VCP_ENC_POWER_ON);
			if (!ret) {
				if (id == VCP_ENC_POWER_ON)
					inst->cores |= BIT(core);
				else
					inst->cores &= ~BIT(core);
			}
			break;
		case VCP_ENC_WAIT_ISR:
			if (core < 0 || core > 1 || !(inst->cores & BIT(core)))
				ret = -EINVAL;
			else
				ret = enc->ops->wait_irq(enc->priv, inst->cookie,
							core, &irq_status);
			response.hw.codec_or_irq = cpu_to_le32(irq_status);
			response.hw.timeout = cpu_to_le32(!!ret);
			dev_info(enc->dev,
				 "VENC service WAIT_ISR: cookie=%#llx core=%d irq=%#x timeout=%u ret=%d\n",
				 inst->cookie, core, irq_status, !!ret, ret);
			/* WAIT_ISR status is the core ID, not its return code. */
			break;
		case VCP_ENC_PUT_BUFFER:
			dev_info(enc->dev,
				 "VENC PUT_BUFFER: cookie=%#llx read=%u write=%u count=%u done=%u\n",
				 inst->cookie,
				 inst->vsi ? le32_to_cpu(READ_ONCE(inst->vsi->free.read)) : 0,
				 inst->vsi ? le32_to_cpu(READ_ONCE(inst->vsi->free.write)) : 0,
				 inst->vsi ? le32_to_cpu(READ_ONCE(inst->vsi->free.count)) : 0,
				 inst->done_count);
			ret = venc_collect_buffers(inst);
			if (ret > 0) {
				enc->ops->buffers_ready(enc->priv, inst->cookie);
				ret = 0;
			}
			break;
		case VCP_ENC_CHECK_ID:
			ret = request_value == inst->codec_id ?
				0 : -1;
			response.hw.hdr.status = cpu_to_le32(ret);
			dev_info_ratelimited(enc->dev,
					     "VENC CHECK_ID: cookie=%#llx codec=%#x reply=%d initialized=%d configured=%d broken=%d\n",
				inst->cookie, request_value, ret, inst->initialized,
				inst->configured, inst->broken);
			break;
		default:
			return -EOPNOTSUPP;
		}
		if (ret && id != VCP_ENC_WAIT_ISR)
			response.hw.hdr.status = cpu_to_le32(ret);
	}
	if (ret)
		dev_err_ratelimited(enc->dev,
			"VENC service %#x failed: %d (status/core=%d value=%#x len=%zu)\n",
			id, ret, core, request_value, len);
	response.hw.hdr.id = cpu_to_le32(id + 0x1000);
	send_ret = mtk_vcp_ipi_send(enc->vcp, MTK_VCP_ENCODER, &response, len);
	/* The service result belongs in the reply status. The firmware decides
	 * whether it fails the outstanding RPC and reports that in *_DONE.
	 */
	return send_ret;
}

static void venc_receive(void *priv, const void *data, size_t len)
{
	struct mtk_vcp_venc *enc = priv;
	struct mtk_vcp_venc_inst *inst;
	u64 cookie;
	u32 id;
	s32 status;
	int ret;

	if (len < sizeof(struct vcp_venc_ack)) {
		dev_err_ratelimited(enc->dev, "short VENC reply: %zu\n", len);
		return;
	}
	id = get_unaligned_le32(data);
	status = (s32)get_unaligned_le32(data + 4);
	cookie = get_unaligned_le64(data + 8);
	mutex_lock(&enc->rx_lock);
	list_for_each_entry(inst, &enc->instances, list) {
		if (inst->cookie != cookie)
			continue;
		if (inst->broken)
			goto out;
		if (id >= VCP_ENC_POWER_ON && id <= VCP_ENC_CHECK_ID) {
			ret = venc_service(inst, id, data, len);
			if (ret)
				venc_fail(inst, ret);
			goto out;
		}
		if (id == VCP_ENC_TRACE)
			goto out;
		if (id != inst->expected || len > sizeof(inst->response)) {
			dev_err_ratelimited(enc->dev,
					    "unexpected VENC reply: id=%#x status=%d cookie=%#llx len=%zu expected=%#x\n",
				id, status, cookie, len, inst->expected);
			venc_fail(inst, -EPROTO);
			goto out;
		}
		if (id == VCP_ENC_INIT_DONE)
			dev_info(enc->dev,
				 "VENC INIT_DONE: status=%d cookie=%#llx len=%zu vsi=%#x reserved=%#x\n",
				 status, cookie, len,
				 len >= 20 ? get_unaligned_le32(data + 16) : 0,
				 len >= 24 ? get_unaligned_le32(data + 20) : 0);
		if (id == VCP_ENC_ENCODE_DONE) {
			if (len >= sizeof(struct vcp_venc_encode_ack))
				dev_info(enc->dev,
					 "VENC ENCODE_DONE: status=%d cookie=%#llx len=%zu state=%u keyframe=%u bytes=%u reserved=%#x\n",
					 status, cookie, len,
					 get_unaligned_le32(data + 16),
					 get_unaligned_le32(data + 20),
					 get_unaligned_le32(data + 24),
					 get_unaligned_le32(data + 28));
			else
				dev_info(enc->dev,
					 "VENC ENCODE_DONE: status=%d cookie=%#llx len=%zu (short ACK)\n",
					 status, cookie, len);
		}
		memcpy(inst->response, data, len);
		inst->response_len = len;
		inst->error = status ? -EIO : 0;
		inst->expected = 0;
		complete(&inst->reply);
		goto out;
	}
	dev_warn_ratelimited(enc->dev, "VENC reply %#x for unknown cookie %#llx\n", id, cookie);
out:
	mutex_unlock(&enc->rx_lock);
}

/* api_lock held. Mailbox receipt does not complete this RPC. */
static int venc_call(struct mtk_vcp_venc_inst *inst, void *data, size_t size, u32 ack)
{
	struct mtk_vcp_venc *enc = inst->enc;
	u32 request = get_unaligned_le32(data);
	int ret;

	mutex_lock(&enc->rx_lock);
	if (inst->broken) {
		ret = inst->error ?: -EIO;
		mutex_unlock(&enc->rx_lock);
		return ret;
	}
	reinit_completion(&inst->reply);
	inst->expected = ack;
	inst->response_len = 0;
	inst->error = 0;
	inst->submitted = true;
	mutex_unlock(&enc->rx_lock);
	dma_wmb();
	ret = mtk_vcp_ipi_send(enc->vcp, MTK_VCP_ENCODER, data, size);
	if (!ret && !wait_for_completion_timeout(&inst->reply,
					msecs_to_jiffies(VCP_VENC_RPC_TIMEOUT_MS)))
		ret = -ETIMEDOUT;
	mutex_lock(&enc->rx_lock);
	if (ret)
		venc_fail(inst, ret);
	else
		ret = inst->error;
	inst->expected = 0;
	mutex_unlock(&enc->rx_lock);
	if (ret)
		dev_err_ratelimited(enc->dev,
				    "VENC RPC failed: request=%#x expected=%#x cookie=%#llx ret=%d response_len=%zu\n",
			request, ack, inst->cookie, ret, inst->response_len);
	return ret;
}

struct mtk_vcp_venc *mtk_vcp_venc_create(struct device *dev,
					 struct device *bitstream_dev,
					 struct mtk_vcp *vcp,
					 const struct mtk_vcp_venc_ops *ops,
					 void *priv)
{
	struct mtk_vcp_venc *enc;
	int ret;

	if (!dev || !bitstream_dev || !vcp || !ops || !ops->power ||
	    !ops->wait_irq || !ops->alloc || !ops->free || !ops->buffers_ready ||
	    !ops->set_perf)
		return ERR_PTR(-EINVAL);
	enc = kzalloc_obj(*enc);
	if (!enc)
		return ERR_PTR(-ENOMEM);
	enc->dev = dev;
	enc->bitstream_dev = bitstream_dev;
	enc->vcp = vcp;
	enc->ops = ops;
	enc->priv = priv;
	mutex_init(&enc->api_lock);
	mutex_init(&enc->rx_lock);
	INIT_LIST_HEAD(&enc->instances);
	ret = mtk_vcp_ipi_register(vcp, MTK_VCP_ENCODER, venc_receive, enc);
	if (ret) {
		kfree(enc);
		return ERR_PTR(ret);
	}
	return enc;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_create);

struct mtk_vcp_venc_inst *mtk_vcp_venc_new(struct mtk_vcp_venc *enc)
{
	struct mtk_vcp_venc_inst *inst;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return ERR_PTR(-ENOMEM);
	inst->enc = enc;
	inst->codec_id = VCP_CODEC_H264_ENCODER;
	INIT_LIST_HEAD(&inst->allocations);
	INIT_LIST_HEAD(&inst->dma_buffers);
	init_completion(&inst->reply);
	mutex_lock(&enc->rx_lock);
	if (enc->next_cookie == U64_MAX) {
		mutex_unlock(&enc->rx_lock);
		kfree(inst);
		return ERR_PTR(-EOVERFLOW);
	}
	inst->cookie = ++enc->next_cookie;
	list_add_tail(&inst->list, &enc->instances);
	mutex_unlock(&enc->rx_lock);
	return inst;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_new);

u64 mtk_vcp_venc_cookie(struct mtk_vcp_venc_inst *inst)
{
	return inst->cookie;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_cookie);

/* One-shot bring-up probe: ask the firmware which video formats and frame
 * sizes it accepts, so the driver's static tables can be checked against the
 * firmware instead of against the vendor header.
 */
static void mtk_vcp_venc_dump_caps(struct mtk_vcp_venc_inst *inst)
{
	struct vcp_venc_video_format *formats;
	struct vcp_venc_frame_sizes *sizes;
	int ret, i;

	formats = kzalloc(sizeof(*formats) * VCP_VENC_MAX_CAPS, GFP_KERNEL);
	sizes = kzalloc(sizeof(*sizes) * VCP_VENC_MAX_CAPS, GFP_KERNEL);
	if (!formats || !sizes)
		goto out;
	ret = mtk_vcp_venc_query_caps(inst, formats, sizes);
	if (ret) {
		dev_warn(inst->enc->dev, "VENC caps query failed: %d\n", ret);
		goto out;
	}
	for (i = 0; i < VCP_VENC_MAX_CAPS && formats[i].fourcc; i++)
		dev_info(inst->enc->dev,
			 "VENC cap fmt[%d]: fourcc=%#x type=%u planes=%u\n", i,
			 le32_to_cpu(formats[i].fourcc), le32_to_cpu(formats[i].type),
			 le32_to_cpu(formats[i].num_planes));
	for (i = 0; i < VCP_VENC_MAX_CAPS && sizes[i].fourcc; i++)
		dev_info(inst->enc->dev,
			 "VENC cap size[%d]: fourcc=%#x profile=%u level=%u %ux%u..%ux%u\n",
			 i, le32_to_cpu(sizes[i].fourcc), le32_to_cpu(sizes[i].profile),
			 le32_to_cpu(sizes[i].level),
			 le32_to_cpu(sizes[i].stepwise.min_width),
			 le32_to_cpu(sizes[i].stepwise.min_height),
			 le32_to_cpu(sizes[i].stepwise.max_width),
			 le32_to_cpu(sizes[i].stepwise.max_height));
out:
	kfree(formats);
	kfree(sizes);
}

int mtk_vcp_venc_set_codec(struct mtk_vcp_venc_inst *inst, u32 fourcc)
{
	u32 id;
	int ret = 0;

	switch (fourcc) {
	case V4L2_PIX_FMT_H264:
		id = VCP_CODEC_H264_ENCODER;
		break;
	case V4L2_PIX_FMT_HEVC:
		id = VCP_CODEC_HEVC_ENCODER;
		break;
	case V4L2_PIX_FMT_MPEG4:
		id = VCP_CODEC_MPEG4_ENCODER;
		break;
	case V4L2_PIX_FMT_H263:
		id = VCP_CODEC_H263_ENCODER;
		break;
	/* V4L2_PIX_FMT_HEIF, defined in venc_drv_if.h; spelled out here so
	 * the protocol layer keeps no frontend dependency.
	 */
	case v4l2_fourcc('H', 'E', 'I', 'F'):
		id = VCP_CODEC_HEIF_ENCODER;
		break;
	default:
		return -EINVAL;
	}
	mutex_lock(&inst->enc->api_lock);
	mutex_lock(&inst->enc->rx_lock);
	if (inst->initialized || inst->broken)
		ret = -EBUSY;
	else
		inst->codec_id = id;
	mutex_unlock(&inst->enc->rx_lock);
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_set_codec);

int mtk_vcp_venc_init(struct mtk_vcp_venc_inst *inst)
{
	struct vcp_venc_init_msg msg = {
		.id = cpu_to_le32(VCP_ENC_INIT), .instance = cpu_to_le64(inst->cookie),
	};
	struct vcp_venc_init_ack *ack = (void *)inst->response;
	int ret;

	mutex_lock(&inst->enc->api_lock);
	if (inst->initialized) {
		ret = -EALREADY;
		goto out;
	}
	ret = venc_call(inst, &msg, sizeof(msg), VCP_ENC_INIT_DONE);
	if (ret)
		goto bad_ack;
	if (inst->response_len != sizeof(*ack)) {
		ret = -EPROTO;
		goto bad_ack;
	}
	inst->firmware_instance = le32_to_cpu(ack->vsi);
	inst->vsi = venc_shared_pointer(inst, inst->firmware_instance, sizeof(*inst->vsi));
	if (IS_ERR(inst->vsi)) {
		ret = PTR_ERR(inst->vsi);
		inst->vsi = NULL;
		goto bad_ack;
	}
	if (!IS_ALIGNED((uintptr_t)inst->vsi, 8)) {
		ret = -EPROTO;
		inst->vsi = NULL;
		goto bad_ack;
	}
	dev_info(inst->enc->dev,
		 "VENC INIT ready: cookie=%#llx firmware_instance=%#x vsi=%p\n",
		 inst->cookie, inst->firmware_instance, inst->vsi);
	inst->initialized = true;
	goto out;
bad_ack:
	mutex_lock(&inst->enc->rx_lock);
	venc_fail(inst, ret);
	mutex_unlock(&inst->enc->rx_lock);
out:
	mutex_unlock(&inst->enc->api_lock);
	if (!ret)
		mtk_vcp_venc_dump_caps(inst);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_init);

static int venc_set_param(struct mtk_vcp_venc_inst *inst, u32 id,
			  const u32 *data, size_t count)
{
	struct vcp_venc_param_msg msg = {
		.id = cpu_to_le32(VCP_ENC_SET_PARAM),
		.firmware_instance = cpu_to_le32(inst->firmware_instance),
		.parameter = cpu_to_le32(id), .count = cpu_to_le32(count),
	};
	int ret, i;

	if (!inst->initialized || count > ARRAY_SIZE(msg.data) || (count && !data))
		return -EINVAL;
	for (i = 0; i < count; i++)
		msg.data[i] = cpu_to_le32(data[i]);
	/* A frame rate is part of the workload the DVFSRC step was chosen for:
	 * re-vote and re-check it before firmware is told about the new rate,
	 * so a rate the rail cannot serve fails the request instead of
	 * encoding at a rate nobody granted.
	 */
	if (id == VCP_VENC_PARAM_FRAMERATE && count == 1 && inst->vsi) {
		ret = inst->enc->ops->set_perf(inst->enc->priv, inst->cookie,
				       le32_to_cpu(inst->vsi->config.pic_w),
				       le32_to_cpu(inst->vsi->config.pic_h),
				       le32_to_cpu(msg.data[0]));
		if (ret)
			return ret;
	}
	ret = venc_call(inst, &msg, sizeof(msg), VCP_ENC_SET_PARAM_DONE);
	if (!ret && inst->response_len != 16 && inst->response_len != 24 &&
	    inst->response_len != 48)
		ret = -EPROTO;
	/* VCP also uses the common ACK; only the extended reply echoes a param. */
	if (!ret && inst->response_len == 48 &&
	    (get_unaligned_le32(inst->response + 16) != id ||
	     get_unaligned_le32(inst->response + 20) > 6))
		ret = -EPROTO;
	if (ret == -EPROTO) {
		mutex_lock(&inst->enc->rx_lock);
		venc_fail(inst, ret);
		mutex_unlock(&inst->enc->rx_lock);
	}
	return ret;
}

int mtk_vcp_venc_set_param(struct mtk_vcp_venc_inst *inst, u32 id,
			 const u32 *data, size_t count)
{
	int ret;

	mutex_lock(&inst->enc->api_lock);
	ret = venc_set_param(inst, id, data, count);
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_set_param);

int mtk_vcp_venc_configure(struct mtk_vcp_venc_inst *inst,
			   const struct vcp_venc_config *config,
			   u32 sizeimage[VCP_VENC_PLANES], bool *synchronous)
{
	int ret, i;

	if (!config || !sizeimage || !synchronous)
		return -EINVAL;
	mutex_lock(&inst->enc->api_lock);
	if (!inst->initialized || inst->broken) {
		ret = -EIO;
		goto out;
	}
	mutex_lock(&inst->enc->rx_lock);
	if (inst->done_count || inst->buffer_count) {
		mutex_unlock(&inst->enc->rx_lock);
		ret = -EBUSY;
		goto out;
	}
	for (i = 0; i < VCP_VENC_BUFFERS; i++) {
		if (inst->frames[i] || inst->outputs[i].cookie) {
			mutex_unlock(&inst->enc->rx_lock);
			ret = -EBUSY;
			goto out;
		}
	}
	inst->configured = false;
	inst->synchronous = false;
	mutex_unlock(&inst->enc->rx_lock);
	dev_info(inst->enc->dev, "VENC CONFIG: cookie=%#llx size=%zu\n",
		 inst->cookie, sizeof(*config));
	print_hex_dump(KERN_INFO, "VENC CONFIG: ", DUMP_PREFIX_OFFSET,
		       16, 4, config, sizeof(*config), false);
	/* The configuration already carries the workload, and firmware may power
	 * the cores up while it handles the CONFIG call, so the step is requested
	 * before that call is made. A workload the rail cannot be asked for fails
	 * the configuration: inst->configured stays false, so no frame can be
	 * submitted at a step nobody was granted.
	 */
	ret = inst->enc->ops->set_perf(inst->enc->priv, inst->cookie,
				       le32_to_cpu(config->pic_w),
				       le32_to_cpu(config->pic_h),
				       le32_to_cpu(config->framerate));
	if (ret)
		goto out;
	memcpy(&inst->vsi->config, config, sizeof(*config));
	ret = venc_set_param(inst, 0, NULL, 0);
	if (!ret) {
		dma_rmb();
		for (i = 0; i < VCP_VENC_PLANES; i++) {
			sizeimage[i] = le32_to_cpu(inst->vsi->sizeimage[i]);
			inst->input_size[i] = sizeimage[i];
		}
		*synchronous = !!le32_to_cpu(inst->vsi->sync_mode);
		inst->synchronous = *synchronous;
		dev_info(inst->enc->dev,
			 "VENC CONFIG_DONE: cookie=%#llx sync=%u sizeimage=%u/%u/%u/%u/%u/%u/%u/%u\n",
			 inst->cookie, *synchronous, sizeimage[0], sizeimage[1],
			 sizeimage[2], sizeimage[3], sizeimage[4], sizeimage[5],
			 sizeimage[6], sizeimage[7]);
		inst->configured = true;
	}
out:
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_configure);

int mtk_vcp_venc_query(struct mtk_vcp_venc_inst *inst, u32 id, void *output, size_t size)
{
	struct mtk_vcp_venc *enc = inst->enc;
	struct vcp_venc_query_msg msg = {
		.id = cpu_to_le32(VCP_ENC_QUERY), .query = cpu_to_le32(id),
		.instance = cpu_to_le64(inst->cookie),
	};
	struct vcp_venc_query_ack *ack = (void *)inst->response;
	void *shared;
	int ret;

	if (id > 1 || !output || !size)
		return -EINVAL;
	mutex_lock(&enc->api_lock);
	mutex_lock(&enc->rx_lock);
	if (enc->next_cookie == U64_MAX) {
		mutex_unlock(&enc->rx_lock);
		ret = -EOVERFLOW;
		goto out;
	}
	msg.cookie = cpu_to_le64(++enc->next_cookie);
	mutex_unlock(&enc->rx_lock);
	ret = venc_call(inst, &msg, sizeof(msg), VCP_ENC_QUERY_DONE);
	if (ret)
		goto out;
	if (inst->response_len != sizeof(*ack) || ack->query != msg.query ||
	    ack->cookie != msg.cookie) {
		ret = -EPROTO;
		goto out;
	}
	shared = venc_shared_pointer(inst, le32_to_cpu(ack->address), size);
	if (IS_ERR(shared)) {
		ret = PTR_ERR(shared);
		goto out;
	}
	dma_rmb();
	memcpy(output, shared, size);
	mutex_lock(&enc->rx_lock);
	if (!inst->initialized)
		inst->submitted = false;
	mutex_unlock(&enc->rx_lock);
out:
	if (ret == -EPROTO || ret == -ERANGE) {
		mutex_lock(&enc->rx_lock);
		venc_fail(inst, ret);
		mutex_unlock(&enc->rx_lock);
	}
	mutex_unlock(&enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_query);

int mtk_vcp_venc_query_caps(struct mtk_vcp_venc_inst *inst,
	struct vcp_venc_video_format *formats,
	struct vcp_venc_frame_sizes *sizes)
{
	int ret;

	if (!formats || !sizes)
		return -EINVAL;
	ret = mtk_vcp_venc_query(inst, VCP_VENC_QUERY_SUPPORTED_FORMATS,
				 formats, sizeof(*formats) * VCP_VENC_MAX_CAPS);
	if (ret)
		return ret;
	return mtk_vcp_venc_query(inst, VCP_VENC_QUERY_FRAME_SIZES,
				 sizes, sizeof(*sizes) * VCP_VENC_MAX_CAPS);
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_query_caps);

static int venc_track_frame(struct mtk_vcp_venc_inst *inst, const struct vcp_venc_frame *f)
{
	int i, src = -1, dst = -1;

	for (i = 0; i < VCP_VENC_BUFFERS; i++) {
		if ((f->frame_cookie && inst->frames[i] == f->frame_cookie) ||
		    (f->bitstream_cookie && inst->outputs[i].cookie == f->bitstream_cookie))
			return -EBUSY;
		if (!inst->frames[i])
			src = i;
		if (!inst->outputs[i].cookie)
			dst = i;
	}
	if ((f->frame_cookie && src < 0) || (f->bitstream_cookie && dst < 0))
		return -ENOSPC;
	if (f->frame_cookie)
		inst->frames[src] = f->frame_cookie;
	if (f->bitstream_cookie) {
		inst->outputs[dst].cookie = f->bitstream_cookie;
		inst->outputs[dst].size = f->bitstream_size;
	}
	return 0;
}

/* api_lock held. tracked distinguishes a local rejection from uncertain
 * firmware acceptance; only the former permits immediate DMA release.
 */
static int venc_submit(struct mtk_vcp_venc_inst *inst, unsigned int mode,
		       const struct vcp_venc_frame *frame, bool *tracked)
{
	struct vcp_venc_encode_msg msg = {
		.id = cpu_to_le32(VCP_ENC_ENCODE),
	};
	struct vcp_venc_info *info;
	int ret, i;

	*tracked = false;
	if (!frame || mode < 2 || mode > 4 || frame->planes > 3 ||
	    (mode == 3 && !frame->planes) || (mode == 2 && frame->planes) ||
	    (!!frame->planes != !!frame->frame_cookie) ||
	    (!!frame->bitstream_size != !!frame->bitstream_cookie) ||
	    (mode != 4 && !frame->bitstream_size) ||
	    frame->bitstream >= BIT_ULL(34) ||
	    frame->bitstream_size > BIT_ULL(34) - frame->bitstream)
		return -EINVAL;
	for (i = 0; i < frame->planes; i++)
		if (!frame->input_size[i] ||
		    frame->input[i] >= BIT_ULL(34) ||
		    frame->input_size[i] > BIT_ULL(34) - frame->input[i])
			return -EINVAL;
	mutex_lock(&inst->enc->rx_lock);
	if (!inst->initialized || !inst->configured || inst->broken) {
		ret = -EIO;
		goto unlock_rx;
	}
	ret = venc_track_frame(inst, frame);
	if (ret)
		goto unlock_rx;
	*tracked = true;
	msg.firmware_instance = cpu_to_le32(inst->firmware_instance);
	info = &inst->vsi->info;
	info->bs_dma = cpu_to_le64(frame->bitstream);
	/* The firmware snapshots these vendor ABI fields into the completion
	 * ring.  Our cookies serve as opaque identities because the standalone
	 * driver has no vendor AP buffer structs for the firmware to dereference;
	 * DMA ownership remains tracked by inst->outputs/inst->frames.
	 */
	info->venc_bs_va = cpu_to_le64(frame->bitstream_cookie);
	info->venc_fb_va = cpu_to_le64(frame->frame_cookie);
	info->timestamp = cpu_to_le64(frame->timestamp);
	for (i = 0; i < frame->planes; i++) {
		info->fb_dma[i] = cpu_to_le64(frame->input[i]);
		msg.input[i] = cpu_to_le32(lower_32_bits(frame->input[i]));
		msg.size[i] = cpu_to_le32(frame->input_size[i]);
		msg.offset[i] = cpu_to_le32(frame->data_offset[i]);
	}
	msg.output = cpu_to_le32(lower_32_bits(frame->bitstream));
	msg.output_size = cpu_to_le32(frame->bitstream_size);
	msg.planes = frame->planes;
	msg.mode = mode;
	mutex_unlock(&inst->enc->rx_lock);
	dev_info(inst->enc->dev,
		 "VENC ENCODE: cookie=%#llx mode=%u planes=%u output=%#llx wire=%#x size=%u bs_cookie=%#llx frame_cookie=%#llx\n",
		 inst->cookie, mode, frame->planes, (u64)frame->bitstream,
		 lower_32_bits(frame->bitstream), frame->bitstream_size,
		 frame->bitstream_cookie, frame->frame_cookie);
	for (i = 0; i < frame->planes; i++)
		dev_info(inst->enc->dev,
			 "VENC ENCODE input[%d]: dma=%#llx wire=%#x size=%u offset=%u required=%u\n",
			 i, (u64)frame->input[i], lower_32_bits(frame->input[i]),
			 frame->input_size[i], frame->data_offset[i],
			 inst->input_size[i]);
	ret = venc_call(inst, &msg, sizeof(msg), VCP_ENC_ENCODE_DONE);
	if (!ret && inst->response_len != sizeof(struct vcp_venc_ack) &&
	    inst->response_len != sizeof(struct vcp_venc_service) &&
	    inst->response_len != sizeof(struct vcp_venc_encode_ack)) {
		ret = -EPROTO;
		mutex_lock(&inst->enc->rx_lock);
		venc_fail(inst, ret);
		mutex_unlock(&inst->enc->rx_lock);
	}
	if (!ret && (mode == 2 || inst->synchronous)) {
		int collected;
		u32 done_count;

		/*
		 * In vendor async mode, a normal frame is returned only from
		 * VCU_IPIMSG_ENC_PUT_BUFFER. Sequence headers and synchronous
		 * operation are the exceptions: their caller consumes the ring
		 * after the ENCODE_DONE ACK, outside the mailbox callback.
		 */
		mutex_lock(&inst->enc->rx_lock);
		collected = inst->broken ? (inst->error ?: -EIO) :
			venc_collect_buffers(inst);
		done_count = inst->done_count;
		if (!collected && !done_count && mode != 4)
			collected = -EPROTO;
		if (collected < 0)
			venc_fail(inst, collected);
		mutex_unlock(&inst->enc->rx_lock);
		dev_info(inst->enc->dev,
			 "VENC ACK buffers: cookie=%#llx mode=%u collected=%d done=%u\n",
			 inst->cookie, mode, collected, done_count);
		ret = collected < 0 ? collected : 0;
	}
	return ret;
unlock_rx:
	mutex_unlock(&inst->enc->rx_lock);
	return ret;
}

int mtk_vcp_venc_submit(struct mtk_vcp_venc_inst *inst, unsigned int mode,
			const struct vcp_venc_frame *frame)
{
	bool tracked;
	int ret;

	mutex_lock(&inst->enc->api_lock);
	if (inst->managed_buffers) {
		ret = -EBUSY;
	} else {
		ret = venc_submit(inst, mode, frame, &tracked);
		if (tracked)
			inst->raw_buffers = true;
	}
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_submit);

int mtk_vcp_venc_submit_vb2(struct mtk_vcp_venc_inst *inst, unsigned int mode,
	struct vb2_buffer *source, struct vb2_buffer *destination,
	const struct vcp_venc_input_layout *layout,
	struct vcp_venc_buffer_ids *ids)
{
	struct mtk_vcp_venc *enc = inst->enc;
	struct vcp_venc_dma_buffer *src = NULL, *dst = NULL;
	struct vcp_venc_frame frame = {};
	size_t bytes = 0;
	unsigned int i, count = !!source + !!destination;
	bool tracked;
	int ret;

	if (!ids)
		return -EINVAL;
	memset(ids, 0, sizeof(*ids));
	if (mode < 2 || mode > 4 || (mode == 2 && source) ||
	    (mode == 3 && !source) || (mode != 4 && !destination) ||
	    (source && source->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) ||
	    (destination && destination->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE))
		return -EINVAL;
	mutex_lock(&enc->api_lock);
	/* Caller-chosen raw cookies must not alias managed attachment cookies. */
	if (inst->raw_buffers) {
		ret = -EBUSY;
		goto out;
	}
	if (!inst->initialized || !inst->configured || inst->broken) {
		ret = -EIO;
		goto out;
	}
	if (count > 2 * VCP_VENC_BUFFERS - inst->buffer_count) {
		ret = -ENOSPC;
		goto out;
	}
	if (source) {
		if (!layout || !source->num_planes || source->num_planes > 3 ||
		    source->num_planes != layout->planes) {
			ret = -EINVAL;
			goto out;
		}
		for (i = 0; i < source->num_planes; i++) {
			struct vb2_plane *p = &source->planes[i];

			/* Source payload and firmware DMA span use different layouts. */
			if (p->bytesused > p->length || p->data_offset >= p->bytesused ||
			    layout->src_size[i] > p->bytesused - p->data_offset ||
			    inst->input_size[i] > layout->dst_size[i]) {
				dev_err(enc->dev,
					"invalid VENC source plane %u: required=%u bytesused=%u length=%u offset=%u\n",
					i, inst->input_size[i], p->bytesused,
					p->length, p->data_offset);
				ret = -EINVAL;
				goto out;
			}
			bytes += layout->dst_size[i];
		}
		for (; i < VCP_VENC_PLANES; i++) {
			if (inst->input_size[i]) {
				ret = -EINVAL;
				goto out;
			}
		}
	}
	if (destination) {
		if (destination->num_planes != 1) {
			ret = -EINVAL;
			goto out;
		}
		bytes += destination->planes[0].length;
	}
	if (bytes > VCP_VENC_MAX_BUFFER_BYTES - inst->buffer_bytes) {
		ret = -ENOMEM;
		goto out;
	}
	if (source) {
		src = vcp_venc_dma_stage_input(enc->dev, source, layout);
		if (IS_ERR(src)) {
			ret = PTR_ERR(src);
			src = NULL;
			goto release;
		}
		frame.planes = src->planes;
		frame.timestamp = source->timestamp;
		for (i = 0; i < src->planes; i++) {
			/* Input was repacked into private storage with no prefix. */
			frame.input[i] = src->plane[i].address;
			frame.input_size[i] = src->plane[i].size;
			frame.data_offset[i] = 0;
		}
	}
	if (destination) {
		dst = vcp_venc_dma_stage(enc->bitstream_dev, destination,
					 DMA_FROM_DEVICE);
		if (IS_ERR(dst)) {
			ret = PTR_ERR(dst);
			dst = NULL;
			goto release;
		}
		if (dst->plane[0].address < VCP_VENC_BITSTREAM_BASE ||
		    dst->plane[0].address >= VCP_VENC_BITSTREAM_END ||
		    dst->plane[0].size >
				VCP_VENC_BITSTREAM_END - dst->plane[0].address) {
			dev_err(enc->dev,
				"VENC bitstream staging outside VCP domain: dma=%#llx size=%u\n",
				(u64)dst->plane[0].address, dst->plane[0].size);
			ret = -ERANGE;
			goto release;
		}
		if (src) {
			for (i = 0; i < src->planes; i++) {
				if (src->plane[i].dbuf == dst->plane[0].dbuf) {
					ret = -EINVAL;
					goto release;
				}
			}
		}
		frame.bitstream = dst->plane[0].address;
		frame.bitstream_size = dst->plane[0].size;
	}
	mutex_lock(&enc->rx_lock);
	if (enc->next_cookie > U64_MAX - count) {
		mutex_unlock(&enc->rx_lock);
		ret = -EOVERFLOW;
		goto release;
	}
	if (src)
		src->cookie = frame.frame_cookie = ++enc->next_cookie;
	if (dst)
		dst->cookie = frame.bitstream_cookie = ++enc->next_cookie;
	mutex_unlock(&enc->rx_lock);
	if (src)
		list_add_tail(&src->list, &inst->dma_buffers);
	if (dst)
		list_add_tail(&dst->list, &inst->dma_buffers);
	inst->buffer_bytes += bytes;
	inst->buffer_count += count;
	inst->managed_buffers = true;
	/* Firmware may return a buffer before the send call returns. Link the
	 * ownership records before issuing the RPC so that return is matchable.
	 */
	ret = venc_submit(inst, mode, &frame, &tracked);
	if (!tracked) {
		if (src)
			list_del_init(&src->list);
		if (dst)
			list_del_init(&dst->list);
		inst->buffer_bytes -= bytes;
		inst->buffer_count -= count;
		inst->managed_buffers = false;
		goto release;
	}
	/* dequeue is serialized by api_lock, including when PUT_BUFFER arrived
	 * before the ENCODE ACK. Never expose vb2 pointers to the IRQ thread.
	 */
	ids->frame = frame.frame_cookie;
	ids->bitstream = frame.bitstream_cookie;
	goto out;
release:
	vcp_venc_dma_release(dst);
	vcp_venc_dma_release(src);
out:
	mutex_unlock(&enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_submit_vb2);

int mtk_vcp_venc_dequeue(struct mtk_vcp_venc_inst *inst, struct vcp_venc_result *result)
{
	struct vcp_venc_dma_buffer *buffer, *frame = NULL, *bitstream = NULL;
	int ret = 0;

	if (!result)
		return -EINVAL;
	mutex_lock(&inst->enc->api_lock);
	mutex_lock(&inst->enc->rx_lock);
	if (!inst->done_count)
		ret = inst->broken ? inst->error : -EAGAIN;
	else {
		*result = inst->done[inst->done_read];
		inst->done_read = (inst->done_read + 1) % VCP_VENC_BUFFERS;
		inst->done_count--;
	}
	mutex_unlock(&inst->enc->rx_lock);
	if (!ret) {
		list_for_each_entry(buffer, &inst->dma_buffers, list) {
			if (buffer->cookie == result->frame_cookie)
				frame = buffer;
			if (buffer->cookie == result->bitstream_cookie)
				bitstream = buffer;
		}
		if ((result->frame_cookie && !frame) ||
		    (result->bitstream_cookie && !bitstream)) {
			ret = -EPROTO;
			mutex_lock(&inst->enc->rx_lock);
			venc_fail(inst, ret);
			mutex_unlock(&inst->enc->rx_lock);
			goto out;
		}
		/* Only a validated firmware return allows copying into user storage. */
		if (bitstream) {
			dma_rmb();
			ret = vcp_venc_dma_copy_output(bitstream, result->bytes);
		}
		venc_release_dma(inst, result->frame_cookie, false);
		venc_release_dma(inst, result->bitstream_cookie, false);
	}
out:
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_dequeue);

int mtk_vcp_venc_deinit(struct mtk_vcp_venc_inst *inst)
{
	struct vcp_venc_cmd_msg msg = {
		.id = cpu_to_le32(VCP_ENC_DEINIT),
	};
	int ret;

	mutex_lock(&inst->enc->api_lock);
	msg.firmware_instance = cpu_to_le32(inst->firmware_instance);
	ret = inst->initialized ? venc_call(inst, &msg, sizeof(msg), VCP_ENC_DEINIT_DONE) : -EINVAL;
	mutex_lock(&inst->enc->rx_lock);
	if (!ret && (inst->cores ||
		     (inst->response_len != sizeof(struct vcp_venc_ack) &&
		      inst->response_len != sizeof(struct vcp_venc_service)))) {
		ret = -EPROTO;
		venc_fail(inst, ret);
	}
	if (!ret) {
		inst->initialized = false;
		inst->configured = false;
		inst->synchronous = false;
		inst->submitted = false;
		inst->vsi = NULL;
		memset(inst->frames, 0, sizeof(inst->frames));
		memset(inst->outputs, 0, sizeof(inst->outputs));
		inst->done_read = 0;
		inst->done_count = 0;
		inst->raw_buffers = false;
		inst->managed_buffers = false;
	}
	mutex_unlock(&inst->enc->rx_lock);
	if (!ret)
		venc_release_dma(inst, 0, true);
	mutex_unlock(&inst->enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_deinit);

int mtk_vcp_venc_free(struct mtk_vcp_venc_inst *inst, bool after_reset)
{
	struct mtk_vcp_venc *enc = inst->enc;
	struct venc_allocation *buf, *next;
	int ret = 0;

	mutex_lock(&enc->api_lock);
	if (after_reset && !mtk_vcp_is_offline(enc->vcp)) {
		ret = -EBUSY;
		goto out;
	}
	mutex_lock(&enc->rx_lock);
	if (!after_reset && (inst->initialized || inst->submitted || inst->cores)) {
		ret = -EBUSY;
		goto unlock_rx;
	}
	list_del(&inst->list);
	/* The instance is about to stop existing: this is the only point where the
	 * operating point it holds may be given up. The VCP itself may still be
	 * running for another codec session, so the callback decides from this
	 * encoder's own hardware state and keeps the step if it is not idle.
	 */
	if (enc->ops->release_perf)
		enc->ops->release_perf(enc->priv, inst->cookie);
	venc_release_dma(inst, 0, true);
	list_for_each_entry_safe(buf, next, &inst->allocations, list) {
		enc->ops->free(enc->priv, buf->type, &buf->mem);
		list_del(&buf->list);
		kfree(buf);
	}
	kfree(inst);
unlock_rx:
	mutex_unlock(&enc->rx_lock);
out:
	mutex_unlock(&enc->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_free);

int mtk_vcp_venc_destroy(struct mtk_vcp_venc *enc)
{
	/* Caller has stopped new API calls before destroying the engine. */
	mutex_lock(&enc->rx_lock);
	if (!list_empty(&enc->instances)) {
		mutex_unlock(&enc->rx_lock);
		return -EBUSY;
	}
	mutex_unlock(&enc->rx_lock);
	mtk_vcp_ipi_unregister(enc->vcp, MTK_VCP_ENCODER);
	kfree(enc);
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcp_venc_destroy);

MODULE_DESCRIPTION("MT6895 VCP vendor encoder protocol");
MODULE_LICENSE("GPL");
