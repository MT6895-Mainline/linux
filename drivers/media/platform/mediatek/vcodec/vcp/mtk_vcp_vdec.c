// SPDX-License-Identifier: GPL-2.0-only
/* MT6895 firmware protocol for the stateful decoder frontend. */
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "mtk_vcp_vdec.h"
#include "mtk_vcp_vdec_bitstream.h"

#define DEC_EVENTS 192
#define DEC_ALLOCATIONS 128
#define DEC_WORK_LIMIT SZ_256M
#define DEC_RPC_TIMEOUT 5000

struct dec_memory {
	struct list_head list;
	struct mtk_vcp_mem mem;
	u64 cookie;
	u32 type;
};

struct dec_surface {
	u64 cookie;
	dma_addr_t y, c;
	/* Pending surfaces are AP-owned until a synchronous START selects one. */
	bool displayed, pending;
	u32 index;
};

struct mtk_vcp_vdec {
	struct device *dev;
	struct mtk_vcp *vcp;
	const struct mtk_vcp_vdec_ops *ops;
	void *priv;
	struct mutex api_lock, rx_lock;
	struct completion reply;
	struct vcp_vdec_vsi *vsi;
	u64 cookie, next_memory;
	u32 address, expected, codec_id;
	int error;
	bool initialized, broken, firmware_live, picture_known;
	unsigned long cores;
	u8 response[64] __aligned(8);
	size_t response_size, work_bytes;
	unsigned int allocation_count;
	struct list_head allocations;
	struct dec_surface surfaces[64];
	u64 bitstreams[64];
	struct vcp_vdec_event events[DEC_EVENTS];
	u32 event_read, event_count;
};

static atomic64_t session_cookie = ATOMIC64_INIT(0);

/* DEBUG: temporary protocol tracing, remove before submission. */
#define VCPDBG(fmt, ...) pr_info("VCPDBG:%s: " fmt, __func__, ##__VA_ARGS__)

static void dec_fail(struct mtk_vcp_vdec *d, int error)
{
	/* Keep the first failure even if an ACK or another error arrives late. */
	if (d->broken)
		return;
	VCPDBG("failure: %d (kept %d), initialized=%d firmware_live=%d cores=%#lx\n",
	       error, error ?: -EIO, d->initialized, d->firmware_live,
	       d->cores);
	d->broken = true;
	d->error = error ?: -EIO;
	complete(&d->reply);
	d->ops->notify(d->priv);
}

static void *dec_shared(struct mtk_vcp_vdec *d, u32 address, size_t size)
{
	struct mtk_vcp_mem mem;
	u32 start, offset;

	if (mtk_vcp_get_mem(d->vcp, MTK_VCP_MEM_VDEC, &mem))
		return ERR_PTR(-EHOSTDOWN);
	start = mem.dma & 0xfffffff;
	address &= 0xfffffff;
	if (address < start)
		return ERR_PTR(-ERANGE);
	offset = address - start;
	if ((offset & 7) || offset > mem.size || size > mem.size - offset)
		return ERR_PTR(-ERANGE);
	return mem.cpu + offset;
}

static int dec_memory(struct mtk_vcp_vdec *d, struct vcp_vdec_mem_op *m)
{
	struct dec_memory *a;
	u32 type = le32_to_cpu(m->mem_type), size = le32_to_cpu(m->mem_len);
	int ret;

	if (le32_to_cpu(m->msg_id) == VCP_VDEC_MEM_FREE) {
		list_for_each_entry(a, &d->allocations, list) {
			if (a->cookie != le64_to_cpu(m->mem_va))
				continue;
			if (a->type != type || a->mem.size != size ||
			    a->mem.dma != le64_to_cpu(m->mem_iova) ||
			    a->mem.dma != le64_to_cpu(m->mem_pa))
				return -EINVAL;
			d->ops->free(d->priv, type, &a->mem);
			d->work_bytes -= size;
			d->allocation_count--;
			list_del(&a->list);
			kfree(a);
			return 0;
		}
		return -ENOENT;
	}
	if (type > 2 || !size || size > DEC_WORK_LIMIT - d->work_bytes ||
	    d->allocation_count >= DEC_ALLOCATIONS)
		return -EINVAL;
	a = kzalloc_obj(*a);
	if (!a)
		return -ENOMEM;
	ret = d->ops->alloc(d->priv, type, size, &a->mem);
	if (ret)
		goto fail;
	if (!a->mem.cpu || a->mem.size != size || !a->mem.dma ||
	    a->mem.dma + size < a->mem.dma ||
	    (type == 0 && (a->mem.dma < 0x150000000ULL ||
			 a->mem.dma + size > 0x160000000ULL)) ||
	    (type == 1 && a->mem.dma + size > 0x400000000ULL) ||
	    (type == 2 && (a->mem.dma < 0x20000000ULL ||
			 a->mem.dma + size > 0x32c00000ULL))) {
		ret = -ERANGE;
		d->ops->free(d->priv, type, &a->mem);
		goto fail;
	}
	memset(a->mem.cpu, 0, size);
	a->cookie = ++d->next_memory;
	a->type = type;
	list_add_tail(&a->list, &d->allocations);
	d->work_bytes += size;
	d->allocation_count++;
	m->mem_iova = cpu_to_le64(a->mem.dma);
	m->mem_pa = m->mem_iova;
	m->mem_va = cpu_to_le64(a->cookie);
	return 0;
fail:
	kfree(a);
	return ret;
}

static int dec_push_event(struct mtk_vcp_vdec *d, enum vcp_vdec_event_type type,
			  u64 cookie, u64 timestamp)
{
	struct vcp_vdec_event *e;

	if (d->event_count == DEC_EVENTS)
		return -ENOSPC;
	e = &d->events[(d->event_read + d->event_count++) % DEC_EVENTS];
	e->type = type;
	e->cookie = cookie;
	e->timestamp = timestamp;
	return 0;
}

/* Consume at PUT_FRAME_BUFFER, or after START/RESET for synchronous codecs. */
static int dec_frames(struct mtk_vcp_vdec *d, struct vcp_vdec_fb_ring *r, bool display)
{
	u32 read = le32_to_cpu(READ_ONCE(r->read));
	u32 write = le32_to_cpu(READ_ONCE(r->write));
	u32 count = le32_to_cpu(READ_ONCE(r->count));
	unsigned int i;
	int ret;

	if (read >= 64 || write >= 64 || count > 64 || (read + count) % 64 != write)
		return -EPROTO;
	while (count--) {
		struct vcp_vdec_fb *f = &r->frame[read];
		u64 cookie = le64_to_cpu(f->cookie);
		struct dec_surface *s = NULL;

		for (i = 0; cookie && i < 64; i++)
			if (d->surfaces[i].cookie == cookie) {
				s = &d->surfaces[i];
				break;
			}
		/* xaga firmware returns cookies/timestamps but zeroes the DMA
		 * fields in display/free entries. Never interpret a cookie as
		 * a pointer; any supplied address must match our submitted record.
		 */
		if (!s || s->pending || (f->y && s->y != le64_to_cpu(f->y)) ||
		    (f->c && s->c != le64_to_cpu(f->c)) ||
		    (display && s->displayed))
			return -EPROTO;
		ret = dec_push_event(d, display ? VCP_VDEC_DISPLAY : VCP_VDEC_FREE_FRAME,
				     cookie, le64_to_cpu(f->timestamp));
		if (ret)
			return ret;
		if (display)
			s->displayed = true;
		else
			memset(s, 0, sizeof(*s));
		read = (read + 1) % 64;
	}
	WRITE_ONCE(r->read, cpu_to_le32(read));
	WRITE_ONCE(r->count, 0);
	return 0;
}

static int dec_collect(struct mtk_vcp_vdec *d)
{
	struct vcp_vdec_bs_ring *r;
	u32 read, write, count;
	unsigned int i;
	int ret;

	if (!d->vsi)
		return -EPROTO;
	dma_rmb();
	ret = dec_frames(d, &d->vsi->display, true);
	if (!ret)
		ret = dec_frames(d, &d->vsi->free_fb, false);
	if (ret)
		return ret;
	r = &d->vsi->free_bs;
	read = le32_to_cpu(READ_ONCE(r->read));
	write = le32_to_cpu(READ_ONCE(r->write));
	count = le32_to_cpu(READ_ONCE(r->count));
	if (read >= 64 || write >= 64 || count > 64 || (read + count) % 64 != write)
		return -EPROTO;
	while (count--) {
		u64 cookie = le64_to_cpu(r->cookie[read]);

		for (i = 0; i < 64; i++)
			if (cookie && d->bitstreams[i] == cookie)
				break;
		if (i == 64)
			return -EPROTO;
		ret = dec_push_event(d, VCP_VDEC_FREE_BITSTREAM, cookie, 0);
		if (ret)
			return ret;
		d->bitstreams[i] = 0;
		read = (read + 1) % 64;
	}
	WRITE_ONCE(r->read, cpu_to_le32(read));
	WRITE_ONCE(r->count, 0);
	dma_wmb();
	return 0;
}

static void dec_receive(void *priv, const void *data, size_t size)
{
	struct mtk_vcp_vdec *d = priv;
	u8 buf[64] __aligned(8);
	struct vcp_vdec_ack *a = (void *)buf;
	u32 id, reply_id;
	int ret = 0;

	VCPDBG("rx: %zu bytes\n", size);
	if (size < sizeof(*a) || size > sizeof(buf)) {
		VCPDBG("rx: unexpected size %zu (ack is %zu)\n", size, sizeof(*a));
		return;
	}
	memcpy(buf, data, size);
	if (le64_to_cpu(a->ap_inst_addr) != d->cookie) {
		VCPDBG("rx: foreign context %#llx (mine %#llx)\n",
		       (u64)le64_to_cpu(a->ap_inst_addr), d->cookie);
		return;
	}
	id = le32_to_cpu(a->msg_id);
	VCPDBG("rx: id=%#x expected=%#x status=%d\n", id, d->expected,
	       (s32)le32_to_cpu(a->status));
	mutex_lock(&d->rx_lock);
	if (id == d->expected) {
		/* Plain acks are a fixed size, but some commands answer with a
		 * larger structure (the capability query returns its own layout).
		 * Keep whatever the receive buffer can hold and let the caller
		 * validate the size it expects.
		 */
		if (size < sizeof(*a)) {
			dec_fail(d, -EPROTO);
			goto out;
		}
		memcpy(d->response, buf, size);
		d->response_size = size;
		if (!d->broken)
			d->error = (s32)le32_to_cpu(a->status);
		complete(&d->reply);
		goto out;
	}
	if ((id & 0xf000) != VCP_VDEC_VCP_SEND_BASE) {
		dec_fail(d, -EPROTO);
		goto out;
	}
	reply_id = id + 0x1000;
	switch (id) {
	case VCP_VDEC_CHECK_CODEC_ID:
		reply_id = VCP_VDEC_CHECK_ID_DONE;
		ret = le32_to_cpu(a->codec_id) == READ_ONCE(d->codec_id) &&
		      !a->status ? 0 : -EINVAL;
		break;
	case VCP_VDEC_MEM_ALLOC:
	case VCP_VDEC_MEM_FREE:
		if (size != sizeof(struct vcp_vdec_mem_op)) {
			dec_fail(d, -EPROTO);
			goto out;
		}
		ret = dec_memory(d, (void *)buf);
		if (ret && id == VCP_VDEC_MEM_ALLOC) {
			struct vcp_vdec_mem_op *m = (void *)buf;

			m->mem_iova = 0;
			m->mem_pa = 0;
			m->mem_va = 0;
		}
		break;
	case VCP_VDEC_PUT_FRAME_BUFFER:
		ret = dec_collect(d);
		if (!ret)
			d->ops->notify(d->priv);
		break;
	case VCP_VDEC_LOCK_CORE:
	case VCP_VDEC_LOCK_LAT: {
		unsigned int core = id == VCP_VDEC_LOCK_LAT;

		if (test_bit(core, &d->cores))
			ret = -EBUSY;
		else
			ret = d->ops->power(d->priv, core, true);
		if (!ret)
			set_bit(core, &d->cores);
		break;
	}
	case VCP_VDEC_UNLOCK_CORE:
	case VCP_VDEC_UNLOCK_LAT: {
		unsigned int core = id == VCP_VDEC_UNLOCK_LAT;

		if (!test_bit(core, &d->cores))
			ret = -EINVAL;
		else
			ret = d->ops->power(d->priv, core, false);
		if (!ret)
			clear_bit(core, &d->cores);
		break;
	}
	case VCP_VDEC_WAITISR: {
		u32 core = le32_to_cpu(a->status);

		ret = core < 2 && test_bit(core, &d->cores) ?
			d->ops->wait_irq(d->priv, core) : -EINVAL;
		break;
	}
	default:
		ret = -EOPNOTSUPP;
	}
	a->msg_id = cpu_to_le32(reply_id);
	a->ctx_id = cpu_to_le32(lower_32_bits(d->cookie));
	a->status = cpu_to_le32(ret);
	dma_wmb();
	ret = mtk_vcp_ipi_send(d->vcp, MTK_VCP_DECODER, buf, size);
	VCPDBG("rx: replying id=%#x status=%d (send=%d)\n", reply_id,
	       (s32)le32_to_cpu(a->status), ret);
	if (ret)
		dec_fail(d, ret);
out:
	mutex_unlock(&d->rx_lock);
}

/* api_lock held, rx_lock must remain available while waiting for replies. */
static int dec_command(struct mtk_vcp_vdec *d, const void *msg, size_t size, u32 ack)
{
	unsigned long started = jiffies;
	u32 id = get_unaligned_le32(msg);
	int ret;

	mutex_lock(&d->rx_lock);
	if (d->broken) {
		ret = d->error ?: -EIO;
		mutex_unlock(&d->rx_lock);
		VCPDBG("cmd: id=%#x ack=%#x refused, session broken (%d)\n", id,
		       ack, ret);
		return ret;
	}
	d->expected = ack;
	d->error = 0;
	d->response_size = 0;
	reinit_completion(&d->reply);
	mutex_unlock(&d->rx_lock);
	dma_wmb();
	ret = get_unaligned_le32(msg) == VCP_VDEC_AP_FRAME_BUFFER ?
		mtk_vcp_vdec_resource_send(d->vcp, msg, size) :
		mtk_vcp_ipi_send(d->vcp, MTK_VCP_DECODER, msg, size);
	VCPDBG("cmd: id=%#x ack=%#x size=%zu sent (ret=%d), waiting %u ms\n",
	       id, ack, size, ret, DEC_RPC_TIMEOUT);
	if (!ret && !wait_for_completion_timeout(&d->reply, msecs_to_jiffies(DEC_RPC_TIMEOUT)))
		ret = -ETIMEDOUT;
	mutex_lock(&d->rx_lock);
	if (ret)
		dec_fail(d, ret);
	else
		ret = d->error;
	d->expected = 0;
	mutex_unlock(&d->rx_lock);
	dma_rmb();
	VCPDBG("cmd: id=%#x ack=%#x -> %d in %u ms\n", id, ack, ret,
	       jiffies_to_msecs(jiffies - started));
	return ret;
}

struct mtk_vcp_vdec *mtk_vcp_vdec_create(struct device *dev, struct mtk_vcp *vcp,
				      const struct mtk_vcp_vdec_ops *ops, void *priv)
{
	struct mtk_vcp_vdec *d;
	int ret;

	if (!dev || !vcp || !ops || !ops->power || !ops->wait_irq || !ops->alloc ||
	    !ops->free || !ops->notify)
		return ERR_PTR(-EINVAL);
	d = kzalloc_obj(*d);
	if (!d)
		return ERR_PTR(-ENOMEM);
	d->dev = dev;
	d->vcp = vcp;
	d->ops = ops;
	d->priv = priv;
	d->cookie = atomic64_inc_return(&session_cookie);
	mutex_init(&d->api_lock);
	mutex_init(&d->rx_lock);
	init_completion(&d->reply);
	INIT_LIST_HEAD(&d->allocations);
	ret = mtk_vcp_ipi_register(vcp, MTK_VCP_DECODER, dec_receive, d);
	if (ret) {
		kfree(d);
		return ERR_PTR(ret);
	}
	return d;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_create);

int mtk_vcp_vdec_init(struct mtk_vcp_vdec *d)
{
	static const char layout[] = "vendor.mtk.vdec.frame.layout.mode 0";
	struct mtk_vcp_mem properties;
	struct vcp_vdec_init msg = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_INIT),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.ap_inst_addr = cpu_to_le64(d->cookie),
	};
	int ret;

	mutex_lock(&d->api_lock);
	if (d->initialized) {
		ret = -EBUSY;
		goto out;
	}
	/* Firmware reads this property table when the decoder service starts.
	 * Request uncompressed MM21 for the CPU conversion to standard NV12M.
	 */
	ret = mtk_vcp_get_mem(d->vcp, MTK_VCP_MEM_VDEC_PROP, &properties);
	if (ret)
		goto out;
	if (properties.size < sizeof(layout)) {
		ret = -ENOSPC;
		goto out;
	}
	memcpy(properties.cpu, layout, sizeof(layout));
	dma_wmb();
	/* Even a lost INIT reply can leave firmware allocations live. */
	d->firmware_live = true;
	ret = dec_command(d, &msg, sizeof(msg), VCP_VDEC_INIT_DONE);
	if (ret)
		goto out;
	if (d->response_size != sizeof(struct vcp_vdec_init_ack)) {
		ret = -EPROTO;
		goto out;
	}
	d->address = get_unaligned_le32(d->response + 24);
	d->vsi = dec_shared(d, d->address, sizeof(*d->vsi));
	if (IS_ERR(d->vsi)) {
		ret = PTR_ERR(d->vsi);
		d->vsi = NULL;
		goto out;
	}
	d->initialized = true;
	d->vsi->dec.planes = cpu_to_le32(2);
	d->vsi->ipi_blocked = 0;
	d->vsi->general_dma = cpu_to_le64(~0ULL);
	d->vsi->general_fd = cpu_to_le32(-1);
	dma_wmb();
out:
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_init);

/* VCP_VDEC_CHECK_CODEC_ID asks the AP to confirm the codec of the session.
 * The frontend knows the negotiated format and hands the id down before the
 * session starts.
 */
int mtk_vcp_vdec_set_codec(struct mtk_vcp_vdec *d, u32 codec_id)
{
	if (codec_id == VCP_VDEC_UNKNOWN || codec_id > VCP_VDEC_AV1)
		return -EINVAL;
	mutex_lock(&d->api_lock);
	WRITE_ONCE(d->codec_id, codec_id);
	mutex_unlock(&d->api_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_set_codec);

/* Ask the firmware to publish one of its capability tables. The firmware
 * leaves the data in its own memory and reports the address back, so the AP
 * copies it out; nothing is queued on the data address in the request.
 */
int mtk_vcp_vdec_query_cap(struct mtk_vcp_vdec *d, u32 id, void *out, size_t size)
{
	struct vcp_vdec_query_cap msg = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_QUERY_CAP),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.id = cpu_to_le32(id),
		.ap_inst_addr = cpu_to_le64(d->cookie),
	};
	struct vcp_vdec_query_ack *ack = (void *)d->response;
	void *shared;
	int ret;

	if (!out || !size)
		return -EINVAL;
	mutex_lock(&d->api_lock);
	if (!d->initialized || d->broken) {
		ret = -EIO;
		goto out;
	}
	ret = dec_command(d, &msg, sizeof(msg), VCP_VDEC_QUERY_CAP_DONE);
	if (ret)
		goto out;
	if (d->response_size != sizeof(*ack)) {
		dev_info(d->dev,
			 "VDEC query %u: reply size %zu, want %zu, head=%*phN\n", id,
			 d->response_size, sizeof(*ack),
			 (int)min_t(size_t, d->response_size, 24), d->response);
		ret = -EPROTO;
		goto out;
	}
	shared = dec_shared(d, get_unaligned_le32(d->response + 40), size);
	if (IS_ERR(shared)) {
		ret = PTR_ERR(shared);
		goto out;
	}
	dma_rmb();
	memcpy(out, shared, size);
out:
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_query_cap);

/* The firmware only delivers 8-bit MM21 and 10-bit MT2T pictures; anything
 * else (other bit depths, 4:2:2/4:4:4, compressed layouts) has no frontend
 * conversion path.
 */
static bool vcp_vdec_layout_ok(struct vcp_vdec_vsi *v, struct vcp_vdec_picture *p)
{
	u32 depth = le32_to_cpu(v->pic.bitdepth);

	if (depth == 10) {
		u64 luma = (u64)p->stride * p->buffer_height * 5 / 4;

		return p->fourcc == V4L2_PIX_FMT_MT2T && !(p->stride & 31) &&
		       !(p->buffer_height & 63) &&
		       !le32_to_cpu(v->pic.layout) && p->size[0] == luma &&
		       p->size[1] == p->size[0] / 2;
	}
	if (depth == 8)
		return p->fourcc == V4L2_PIX_FMT_MM21 &&
		       p->size[0] == (u64)p->stride * p->buffer_height &&
		       p->size[1] == p->size[0] / 2;
	return false;
}

int mtk_vcp_vdec_picture(struct mtk_vcp_vdec *d, struct vcp_vdec_picture *p)
{
	struct vcp_vdec_vsi *v;
	int ret = 0;

	mutex_lock(&d->api_lock);
	v = d->vsi;
	if (!v || d->broken) {
		ret = -EIO;
		goto out;
	}
	dma_rmb();
	p->width = le32_to_cpu(v->pic.width);
	p->height = le32_to_cpu(v->pic.height);
	p->stride = le32_to_cpu(v->pic.buffer_width);
	p->buffer_height = le32_to_cpu(v->pic.buffer_height);
	p->size[0] = le32_to_cpu(v->pic.plane_size[0]);
	p->size[1] = le32_to_cpu(v->pic.plane_size[1]);
	p->dpb = le32_to_cpu(v->dec.dpb_size);
	p->fourcc = le32_to_cpu(v->pic.fourcc);
	p->input_driven = v->input_driven;
	p->crop_left = le32_to_cpu(v->crop_left);
	p->crop_top = le32_to_cpu(v->crop_top);
	p->crop_width = le32_to_cpu(v->crop_width);
	p->crop_height = le32_to_cpu(v->crop_height);
	/* 10-bit pictures arrive in the MT2T tile layout at 10 bits per sample:
	 * five bytes per four samples. Chroma tiles need an even tile column
	 * count, hence the 32-aligned stride.
	 */
	if (!p->width || p->width > 4096 || !p->height || p->height > 2176 ||
	    p->stride < p->width || p->stride > 4096 || (p->stride & 15) ||
	    p->buffer_height < p->height || p->buffer_height > 2176 ||
	    (p->buffer_height & 31) || !p->dpb || p->dpb > 32 ||
	    (p->input_driven != 0 && p->input_driven != 2) ||
	    !vcp_vdec_layout_ok(v, p)) {
		dev_err(d->dev,
			"unsupported picture %ux%u buffer %ux%u size %u/%u dpb %u format %#x depth %u layout %u input %u\n",
			p->width, p->height, p->stride, p->buffer_height,
			p->size[0], p->size[1], p->dpb, p->fourcc,
			le32_to_cpu(v->pic.bitdepth), le32_to_cpu(v->pic.layout),
			p->input_driven);
		ret = -EOPNOTSUPP;
	}
	if (!ret)
		d->picture_known = true;
out:
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_picture);

int mtk_vcp_vdec_frame(struct mtk_vcp_vdec *d, u64 cookie, unsigned int index,
		       dma_addr_t y, dma_addr_t c)
{
	struct vcp_vdec_set_param msg = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_FRAME_BUFFER),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.vcp_inst_addr = cpu_to_le32(d->address),
	};
	struct vcp_vdec_fb f = {
		.cookie = cpu_to_le64(cookie), .y = cpu_to_le64(y), .c = cpu_to_le64(c),
		.general = cpu_to_le64(~0ULL), .reserved = cpu_to_le32(index),
	};
	unsigned int i, slot = 64;
	int ret;

	if (!cookie || index >= 64 || !y || !c || y >= 0x400000000ULL || c >= 0x400000000ULL)
		return -EINVAL;
	mutex_lock(&d->api_lock);
	mutex_lock(&d->rx_lock);
	ret = -EINVAL;
	if (!d->initialized || d->broken)
		goto unlock;
	for (i = 0; i < 64; i++) {
		if (d->surfaces[i].cookie == cookie)
			goto unlock;
		if (!d->surfaces[i].cookie)
			slot = i;
	}
	ret = -ENOSPC;
	if (slot == 64)
		goto unlock;
	d->surfaces[slot] = (struct dec_surface){
		.cookie = cookie, .y = y, .c = c, .index = index,
		.pending = !d->vsi->input_driven,
	};
	mutex_unlock(&d->rx_lock);
	if (!d->vsi->input_driven) {
		/* NON_INPUT_DRIVEN supplies one frame in the next START VSI. */
		ret = 0;
		goto out;
	}
	memcpy(msg.data, &f, sizeof(f));
	ret = dec_command(d, &msg, sizeof(msg), VCP_VDEC_DONE);
	goto out;
unlock:
	mutex_unlock(&d->rx_lock);
out:
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_frame);

int mtk_vcp_vdec_start(struct mtk_vcp_vdec *d, u64 cookie, dma_addr_t dma,
		       u32 bytes, u32 capacity, u64 timestamp, u32 *changed)
{
	struct vcp_vdec_start msg = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_START),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.vcp_inst_addr = cpu_to_le32(d->address),
		.data = { cpu_to_le32(bytes), cpu_to_le32(capacity), 0 },
	};
	unsigned int i, slot = 64, frames = 0, frame_slot = 64;
	bool parsing_header;
	int ret = -EINVAL;

	if (!cookie || !bytes || bytes > capacity || capacity > SZ_16M ||
	    dma < 0x110000000ULL || dma >= 0x130000000ULL ||
	    capacity > 0x130000000ULL - dma || !changed)
		return -EINVAL;
	mutex_lock(&d->api_lock);
	mutex_lock(&d->rx_lock);
	if (!d->initialized || d->broken)
		goto unlock;
	for (i = 0; i < 64; i++) {
		if (d->bitstreams[i] == cookie)
			goto unlock;
		if (!d->bitstreams[i])
			slot = i;
		if (d->surfaces[i].cookie)
			frames++;
		if (d->surfaces[i].pending)
			frame_slot = i;
	}
	ret = -ENOSPC;
	if (slot == 64)
		goto unlock;
	parsing_header = !d->picture_known;
	if (!parsing_header && !d->vsi->input_driven && frame_slot == 64)
		goto unlock;
	d->bitstreams[slot] = cookie;
	d->vsi->dec.bs_dma = cpu_to_le64(dma);
	d->vsi->dec.bs_cookie = cpu_to_le64(cookie);
	d->vsi->dec.fb_cookie = 0;
	d->vsi->dec.index = cpu_to_le32(0xff);
	memset(d->vsi->dec.fb_dma, 0, sizeof(d->vsi->dec.fb_dma));
	if (!parsing_header && !d->vsi->input_driven) {
		struct dec_surface *s = &d->surfaces[frame_slot];

		d->vsi->dec.fb_cookie = cpu_to_le64(s->cookie);
		d->vsi->dec.index = cpu_to_le32(s->index);
		d->vsi->dec.fb_dma[0] = cpu_to_le64(s->y);
		d->vsi->dec.fb_dma[1] = cpu_to_le64(s->c);
		s->pending = false;
	}
	d->vsi->dec.timestamp = cpu_to_le64(timestamp);
	d->vsi->dec.queued_frames = cpu_to_le32(frames);
	mutex_unlock(&d->rx_lock);
	ret = dec_command(d, &msg, sizeof(msg), VCP_VDEC_START_DONE);
	*changed = le32_to_cpu(d->vsi->dec.changed);
	if (!ret && !d->vsi->input_driven) {
		mutex_lock(&d->rx_lock);
		ret = dec_collect(d);
		/* START_DONE releases the synchronous input even if this firmware
		 * omits it from free_bs. A callback may already have returned it.
		 */
		if (!ret && d->bitstreams[slot]) {
			d->bitstreams[slot] = 0;
			ret = dec_push_event(d, VCP_VDEC_FREE_BITSTREAM, cookie, 0);
		}
		mutex_unlock(&d->rx_lock);
	} else if (!ret && parsing_header && (*changed & BIT(0))) {
		/* The initial resolution notification requires resubmitting this
		 * access unit. Firmware has not queued the bitstream for decoding.
		 */
		mutex_lock(&d->rx_lock);
		if (d->bitstreams[slot]) {
			d->bitstreams[slot] = 0;
			ret = dec_push_event(d, VCP_VDEC_FREE_BITSTREAM, cookie, 0);
		}
		mutex_unlock(&d->rx_lock);
	}
	goto out;
unlock:
	mutex_unlock(&d->rx_lock);
out:
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_start);

static int dec_simple_command(struct mtk_vcp_vdec *d, u32 command, u32 ack, u32 flags)
{
	struct vcp_vdec_cmd msg = {
		.msg_id = cpu_to_le32(command),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.vcp_inst_addr = cpu_to_le32(d->address),
		.reserved = cpu_to_le32(flags),
	};

	return dec_command(d, &msg, sizeof(msg), ack);
}

int mtk_vcp_vdec_reset(struct mtk_vcp_vdec *d, bool drain)
{
	struct vcp_vdec_set_param flush = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_FRAME_BUFFER),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.vcp_inst_addr = cpu_to_le32(d->address),
	};
	int ret;

	mutex_lock(&d->api_lock);
	ret = -EINVAL;
	if (!d->initialized)
		goto out;
	/* Drain returns decoded references but leaves unused resources queued.
	 * A null FRAME_BUFFER plus RESET flush returns those resources as well.
	 */
	ret = drain || !d->vsi->input_driven ? 0 :
		dec_command(d, &flush, sizeof(flush), VCP_VDEC_DONE);
	if (!ret)
		ret = dec_simple_command(d, VCP_VDEC_AP_RESET, VCP_VDEC_RESET_DONE, drain);
	if (!ret && !d->vsi->input_driven) {
		unsigned int i;

		mutex_lock(&d->rx_lock);
		ret = dec_collect(d);
		for (i = 0; !ret && !drain && i < 64; i++) {
			struct dec_surface *s = &d->surfaces[i];

			if (!s->pending)
				continue;
			ret = dec_push_event(d, VCP_VDEC_FREE_FRAME, s->cookie, 0);
			if (!ret)
				memset(s, 0, sizeof(*s));
		}
		mutex_unlock(&d->rx_lock);
	}
	if (!ret && !drain)
		d->picture_known = false;
out:
	VCPDBG("reset: drain=%d initialized=%d -> %d\n", drain, d->initialized,
	       ret);
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_reset);

int mtk_vcp_vdec_event(struct mtk_vcp_vdec *d, struct vcp_vdec_event *e)
{
	int ret = -EAGAIN;

	mutex_lock(&d->rx_lock);
	if (d->event_count) {
		*e = d->events[d->event_read];
		d->event_read = (d->event_read + 1) % DEC_EVENTS;
		d->event_count--;
		ret = 0;
	} else if (d->broken) {
		ret = d->error ?: -EIO;
		VCPDBG("event: session broken, returning %d\n", ret);
	}
	mutex_unlock(&d->rx_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_event);

int mtk_vcp_vdec_deinit(struct mtk_vcp_vdec *d)
{
	struct vcp_vdec_set_param flush = {
		.msg_id = cpu_to_le32(VCP_VDEC_AP_FRAME_BUFFER),
		.ctx_id = cpu_to_le32(lower_32_bits(d->cookie)),
		.vcp_inst_addr = cpu_to_le32(d->address),
	};
	int ret;

	mutex_lock(&d->api_lock);
	ret = -EINVAL;
	if (!d->initialized)
		goto out;
	/* Deinitializing with unused frames in service2 crashes xaga firmware. */
	ret = !d->vsi->input_driven ? 0 :
		dec_command(d, &flush, sizeof(flush), VCP_VDEC_DONE);
	if (!ret)
		ret = dec_simple_command(d, VCP_VDEC_AP_RESET, VCP_VDEC_RESET_DONE, 0);
	if (!ret)
		ret = dec_simple_command(d, VCP_VDEC_AP_DEINIT, VCP_VDEC_DEINIT_DONE, 0);
	if (!ret) {
		mutex_lock(&d->rx_lock);
		d->initialized = false;
		d->firmware_live = false;
		d->vsi = NULL;
		mutex_unlock(&d->rx_lock);
	}
out:
	VCPDBG("deinit: initialized=%d -> %d\n", d->initialized, ret);
	mutex_unlock(&d->api_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_deinit);

int mtk_vcp_vdec_destroy(struct mtk_vcp_vdec *d, bool after_reset)
{
	struct dec_memory *a, *next;

	if (!d)
		return 0;
	if (after_reset ? !mtk_vcp_is_offline(d->vcp) :
	    (d->firmware_live || d->broken || d->cores)) {
		VCPDBG("destroy: busy, after_reset=%d live=%d broken=%d cores=%#lx offline=%d\n",
		       after_reset, d->firmware_live, d->broken, d->cores,
		       mtk_vcp_is_offline(d->vcp));
		return -EBUSY;
	}
	mtk_vcp_ipi_unregister(d->vcp, MTK_VCP_DECODER);
	list_for_each_entry_safe(a, next, &d->allocations, list) {
		d->ops->free(d->priv, a->type, &a->mem);
		list_del(&a->list);
		kfree(a);
	}
	kfree(d);
	VCPDBG("destroy: released\n");
	return 0;
}
EXPORT_SYMBOL_GPL(mtk_vcp_vdec_destroy);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6895 VCP decoder protocol");
