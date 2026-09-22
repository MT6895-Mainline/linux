/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTK_VCP_VP9_BITSTREAM_H
#define MTK_VCP_VP9_BITSTREAM_H

#include <linux/errno.h>
#include <linux/types.h>

/* VP9 uncompressed headers use MSB-first bits, without RBSP escaping. */
struct vcp_vp9_bits {
	const u8 *data;
	size_t size, bit;
	bool error;
};

static inline u32 vcp_vp9_read(struct vcp_vp9_bits *r, unsigned int n)
{
	u32 value = 0;

	while (n--) {
		if (r->bit / 8 >= r->size) {
			r->error = true;
			return 0;
		}
		value = (value << 1) | ((r->data[r->bit / 8] >> (7 - r->bit % 8)) & 1);
		r->bit++;
	}
	return value;
}

/* Check the supported profile and explicit geometry before firmware DMA.
 * Reference-dependent inter-frame syntax remains the firmware's responsibility.
 */
static inline int vcp_vp9_frame_guard(const u8 *data, size_t size)
{
	struct vcp_vp9_bits r = { .data = data, .size = size };
	u32 profile, frame_type, show, resilient, intra = 0, width, height;
	bool explicit_size = true;
	unsigned int i;

	if (vcp_vp9_read(&r, 2) != 2)
		return -EINVAL;
	profile = vcp_vp9_read(&r, 1);
	profile |= vcp_vp9_read(&r, 1) << 1;
	if (r.error)
		return -EINVAL;
	if (profile)
		return -EOPNOTSUPP;
	if (vcp_vp9_read(&r, 1)) {
		vcp_vp9_read(&r, 3); /* reference slot for show_existing_frame */
		return r.error ? -EINVAL : 0;
	}
	frame_type = vcp_vp9_read(&r, 1);
	show = vcp_vp9_read(&r, 1);
	resilient = vcp_vp9_read(&r, 1);
	if (frame_type) {
		if (!show)
			intra = vcp_vp9_read(&r, 1);
		if (!resilient)
			vcp_vp9_read(&r, 2);
	}
	if (!frame_type || intra) {
		if (vcp_vp9_read(&r, 24) != 0x498342)
			return -EINVAL;
		if (!frame_type) {
			if (vcp_vp9_read(&r, 3) == 7) /* sRGB requires profile 1/3 */
				return -EOPNOTSUPP;
			vcp_vp9_read(&r, 1); /* range */
		} else {
			vcp_vp9_read(&r, 8); /* refresh_frame_flags */
		}
	} else {
		vcp_vp9_read(&r, 8); /* refresh flags */
		for (i = 0; i < 3; i++)
			vcp_vp9_read(&r, 4); /* reference index and sign bias */
		for (i = 0; i < 3; i++) {
			if (vcp_vp9_read(&r, 1)) {
				explicit_size = false;
				break;
			}
		}
	}
	if (explicit_size) {
		width = vcp_vp9_read(&r, 16) + 1;
		height = vcp_vp9_read(&r, 16) + 1;
		if (r.error)
			return -EINVAL;
		if (width > 4096 || height > 2176)
			return -EOPNOTSUPP;
	}
	return r.error || size < 2 ? -EINVAL : 0;
}

static inline int vcp_vp9_guard(const u8 *data, size_t size)
{
	size_t index, offset = 0;
	unsigned int frames, magnitude, i, j;
	u8 marker;
	int ret;

	if (!data || !size)
		return -EINVAL;
	marker = data[size - 1];
	if ((marker & 0xe0) != 0xc0)
		return vcp_vp9_frame_guard(data, size);
	frames = (marker & 7) + 1;
	magnitude = ((marker >> 3) & 3) + 1;
	index = 2 + frames * magnitude;
	if (size < index || data[size - index] != marker)
		return -EINVAL;
	for (i = 0; i < frames; i++) {
		u32 bytes = 0;

		for (j = 0; j < magnitude; j++)
			bytes |= (u32)data[size - index + 1 + i * magnitude + j] << (j * 8);
		if (!bytes || bytes > size - index - offset)
			return -EINVAL;
		ret = vcp_vp9_frame_guard(data + offset, bytes);
		if (ret)
			return ret;
		offset += bytes;
	}
	return offset == size - index ? 0 : -EINVAL;
}

#endif
