/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MTK_VCP_VDEC_BITSTREAM_H
#define MTK_VCP_VDEC_BITSTREAM_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include "mtk_vcp_vp9_bitstream.h"

/* Upstream V4L2 has no HEIF fourcc; matches vendor V4L2_PIX_FMT_HEIF and the
 * firmware format table ('HEIF'). Upstream spells AV1 'AV01' while the
 * firmware table spells it 'AV10'; the driver bridges the two, the guard
 * only sees the V4L2 spelling. MT2T is vendor V4L2_PIX_FMT_MT2110T, the
 * firmware 10-bit tile layout reported for 10-bit pictures.
 */
#ifndef V4L2_PIX_FMT_HEIF
#define V4L2_PIX_FMT_HEIF v4l2_fourcc('H', 'E', 'I', 'F')
#endif
#ifndef V4L2_PIX_FMT_MT2T
#define V4L2_PIX_FMT_MT2T v4l2_fourcc('M', 'T', '2', 'T')
#endif

/* MPEG-1/2/4 and H.263 use plain MSB-first fields with no RBSP emulation
 * prevention. Single reads stay at 16 bits or less so the 32-bit cache
 * below never overflows.
 */
struct vcp_bs {
	const u8 *data;
	size_t size, pos;
	u32 cache;
	unsigned int left;
	bool error;
};

/* Start-code prefix tolerance: encoders emit 00 00 01 or 00 00 00 01, and
 * byte stuffing can add further leading zeros. The scanners rediscover
 * every code themselves; the prefix only rejects buffers that do not open
 * with one at all.
 */
static inline int vcp_bs_prefix(const u8 *data, size_t size, u8 limit)
{
	size_t zeros = 0;

	if (!data || size < 4)
		return -EINVAL;
	while (zeros < size && !data[zeros])
		zeros++;
	if (zeros < 2 || zeros >= size || data[zeros] < limit)
		return -EINVAL;
	return 0;
}

static inline u32 vcp_bs_bits(struct vcp_bs *b, unsigned int n)
{
	u32 value;

	if (n > 16) {
		b->error = true;
		return 0;
	}
	while (b->left < n) {
		if (b->pos >= b->size) {
			b->error = true;
			return 0;
		}
		b->cache = (b->cache << 8) | b->data[b->pos++];
		b->left += 8;
	}
	if (!n)
		return 0;
	b->left -= n;
	value = (b->cache >> b->left) & (n == 16 ? 0xffffU : ((1U << n) - 1U));
	return value;
}

static inline void vcp_bs_skip(struct vcp_bs *b, unsigned int n)
{
	while (n > 16) {
		vcp_bs_bits(b, 16);
		n -= 16;
	}
	vcp_bs_bits(b, n);
}

/* A bounded SPS-prefix reader, not a complete codec parser. The frontend can
 * only deliver 8-bit 4:2:0. Reject incompatible sequence headers before the
 * firmware sees them, including headers replacing an already running sequence.
 * The remaining syntax and parameter-set references are still firmware-owned.
 */
struct vcp_rbsp {
	const u8 *data;
	size_t size, pos;
	u32 byte, bits, zeros;
	bool error;
};

static inline u32 vcp_rbsp_bits(struct vcp_rbsp *r, unsigned int count)
{
	u32 value = 0;

	while (count-- && !r->error) {
		if (!r->bits) {
			if (r->pos >= r->size)
				goto truncated;
			r->byte = r->data[r->pos++];
			if (r->zeros == 2 && r->byte == 3) {
				if (r->pos >= r->size || r->data[r->pos] > 3)
					goto truncated;
				r->byte = r->data[r->pos++];
				r->zeros = 0;
			}
			r->zeros = r->byte ? 0 : (r->zeros < 2 ? r->zeros + 1 : 2);
			r->bits = 8;
		}
		value = (value << 1) | ((r->byte >> --r->bits) & 1);
	}
	return value;

truncated:
	r->error = true;
	return 0;
}

static inline u32 vcp_rbsp_ue(struct vcp_rbsp *r)
{
	unsigned int zeros = 0;
	u32 suffix;

	while (!vcp_rbsp_bits(r, 1)) {
		if (r->error || ++zeros > 31) {
			r->error = true;
			return 0;
		}
	}
	/* Separate the read from the expression: it changes the reader state. */
	suffix = vcp_rbsp_bits(r, zeros);
	return ((1U << zeros) - 1) + suffix;
}

static inline int vcp_h264_sps_guard(const u8 *data, size_t size)
{
	struct vcp_rbsp r = { .data = data, .size = size };
	u32 profile, constraints, id, chroma = 1, luma = 0, colour = 0;

	profile = vcp_rbsp_bits(&r, 8);
	constraints = vcp_rbsp_bits(&r, 8);
	vcp_rbsp_bits(&r, 8); /* level_idc */
	id = vcp_rbsp_ue(&r);
	if (r.error || (constraints & 3) || id > 31)
		return -EINVAL;

	switch (profile) {
	case 66: /* Baseline */
	case 77: /* Main */
	case 88: /* Extended: implicit 8-bit 4:2:0 */
		break;
	case 100: /* High */
		chroma = vcp_rbsp_ue(&r);
		if (r.error)
			return -EINVAL;
		if (chroma != 1)
			return -EOPNOTSUPP;
		luma = vcp_rbsp_ue(&r);
		colour = vcp_rbsp_ue(&r);
		break;
	default:
		/* High 10/422/444, scalable and multiview paths are not supported. */
		return -EOPNOTSUPP;
	}
	if (r.error)
		return -EINVAL;
	return luma || colour ? -EOPNOTSUPP : 0;
}

static inline int vcp_hevc_sps_guard(const u8 *data, size_t size)
{
	struct vcp_rbsp r = { .data = data, .size = size };
	u32 layers, profile, space, id, chroma, width, height, luma, colour;
	bool sub_profile[7] = {}, sub_level[7] = {};
	unsigned int i;

	vcp_rbsp_bits(&r, 4); /* sps_video_parameter_set_id */
	layers = vcp_rbsp_bits(&r, 3); /* sps_max_sub_layers_minus1 */
	vcp_rbsp_bits(&r, 1); /* sps_temporal_id_nesting_flag */
	if (r.error || layers > 6)
		return -EINVAL;
	space = vcp_rbsp_bits(&r, 2);
	vcp_rbsp_bits(&r, 1); /* general_tier_flag */
	profile = vcp_rbsp_bits(&r, 5);
	/* The rest of general_profile occupies 80 bits for every profile. */
	vcp_rbsp_bits(&r, 32);
	vcp_rbsp_bits(&r, 32);
	vcp_rbsp_bits(&r, 16);
	vcp_rbsp_bits(&r, 8); /* general_level_idc */
	for (i = 0; i < layers; i++) {
		sub_profile[i] = vcp_rbsp_bits(&r, 1);
		sub_level[i] = vcp_rbsp_bits(&r, 1);
	}
	if (layers)
		for (i = layers; i < 8; i++)
			if (vcp_rbsp_bits(&r, 2))
				return -EINVAL;
	for (i = 0; i < layers; i++) {
		if (sub_profile[i]) {
			vcp_rbsp_bits(&r, 32);
			vcp_rbsp_bits(&r, 32);
			vcp_rbsp_bits(&r, 24);
		}
		if (sub_level[i])
			vcp_rbsp_bits(&r, 8);
	}
	id = vcp_rbsp_ue(&r);
	chroma = vcp_rbsp_ue(&r);
	if (r.error || id > 15 || chroma > 3)
		return -EINVAL;
	if (space || profile < 1 || profile > 3 || chroma != 1)
		return -EOPNOTSUPP;
	width = vcp_rbsp_ue(&r);
	height = vcp_rbsp_ue(&r);
	if (vcp_rbsp_bits(&r, 1)) /* conformance_window_flag */
		for (i = 0; i < 4; i++)
			vcp_rbsp_ue(&r);
	luma = vcp_rbsp_ue(&r);
	colour = vcp_rbsp_ue(&r);
	if (r.error || !width || !height)
		return -EINVAL;
	/* 8-bit and 10-bit 4:2:0 only; the firmware has no 4:2:2/4:4:4 or
	 * 12-bit output layout, and mixed bit depths are not valid streams.
	 */
	if (luma != colour || (luma != 0 && luma != 2))
		return -EOPNOTSUPP;
	return 0;
}

/* Firmware MPG2 box: 16x16..2048x1088. Progressive frames only: the frontend
 * delivers whole pictures, so interlaced sequences, field pictures and
 * D-pictures are refused before the firmware sees them. Extension sizes are
 * the high bits of the base dimensions; without cross-AU state only the
 * base header carries a size check, while the extension still gates
 * progressive/chroma. The remaining geometry always comes from firmware.
 */
static inline int vcp_mpeg2_seq_guard(struct vcp_bs *b)
{
	u32 width, height, aspect, rate;
	unsigned int i;

	width = vcp_bs_bits(b, 12);
	height = vcp_bs_bits(b, 12);
	aspect = vcp_bs_bits(b, 4);
	rate = vcp_bs_bits(b, 4);
	vcp_bs_skip(b, 18); /* bit_rate_value */
	if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
		return -EINVAL;
	vcp_bs_skip(b, 11); /* vbv_buffer_size + constrained_parameters_flag */
	if (b->error || !aspect || !rate || width < 16 || width > 2048 ||
	    height < 16 || height > 1088)
		return -EINVAL;
	if (vcp_bs_bits(b, 1)) /* load_intra_quantiser_matrix */
		for (i = 0; i < 64 && !b->error; i++)
			vcp_bs_skip(b, 8);
	if (!b->error && vcp_bs_bits(b, 1)) /* load_non_intra_quantiser_matrix */
		for (i = 0; i < 64 && !b->error; i++)
			vcp_bs_skip(b, 8);
	return b->error ? -EINVAL : 0;
}

static inline int vcp_mpeg2_ext_guard(struct vcp_bs *b)
{
	u32 id, chroma;
	unsigned int i;

	id = vcp_bs_bits(b, 4);
	if (b->error)
		return -EINVAL;
	switch (id) {
	case 1: /* sequence_extension */
		vcp_bs_skip(b, 8); /* profile_and_level_indication */
		if (!vcp_bs_bits(b, 1)) /* progressive_sequence */
			return -EOPNOTSUPP;
		chroma = vcp_bs_bits(b, 2);
		if (chroma != 1)
			return -EOPNOTSUPP;
		vcp_bs_skip(b, 2 + 2 + 12); /* size/bitrate high bits */
		if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
			return -EINVAL;
		vcp_bs_skip(b, 8 + 1 + 2 + 5); /* vbv extension and frame rate extension */
		return b->error ? -EINVAL : 0;
	case 2: /* sequence_display_extension */
		vcp_bs_skip(b, 3); /* video_format */
		if (vcp_bs_bits(b, 1)) /* colour_description */
			vcp_bs_skip(b, 24);
		vcp_bs_skip(b, 14); /* display_horizontal_size */
		if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
			return -EINVAL;
		vcp_bs_skip(b, 14); /* display_vertical_size */
		return b->error ? -EINVAL : 0;
	case 3: /* quant_matrix_extension */
		for (i = 0; i < 4 && !b->error; i++)
			if (vcp_bs_bits(b, 1))
				vcp_bs_skip(b, 64 * 8);
		return b->error ? -EINVAL : 0;
	case 8: /* picture_coding_extension */
		vcp_bs_skip(b, 16); /* f_code[0][0..1], f_code[1][0..1] */
		vcp_bs_skip(b, 2); /* intra_dc_precision */
		if (vcp_bs_bits(b, 2) != 3) /* picture_structure: frame only */
			return -EOPNOTSUPP;
		vcp_bs_skip(b, 10); /* prediction/concealment/scan flags */
		if (!b->error && vcp_bs_bits(b, 1)) /* composite_display_flag */
			vcp_bs_skip(b, 1 + 3 + 1 + 7 + 8);
		return b->error ? -EINVAL : 0;
	case 5: /* sequence_scalable_extension */
	case 9: /* picture_spatial_scalable_extension */
	case 10: /* picture_temporal_scalable_extension */
	case 6: /* picture_camera_extension */
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
}

static inline int vcp_mpeg2_code_guard(u8 code, const u8 *data, size_t size)
{
	struct vcp_bs b = { .data = data, .size = size };
	u32 type;

	switch (code) {
	case 0x00: /* picture_header */
		vcp_bs_skip(&b, 10); /* temporal_reference */
		type = vcp_bs_bits(&b, 3);
		if (b.error || !type || type > 4)
			return -EINVAL;
		if (type == 4) /* D-pictures: obsolete coding type */
			return -EOPNOTSUPP;
		vcp_bs_skip(&b, 16); /* vbv_delay */
		return b.error ? -EINVAL : 0;
	case 0xb3:
		return vcp_mpeg2_seq_guard(&b);
	case 0xb5:
		return vcp_mpeg2_ext_guard(&b);
	case 0xb2: /* user_data */
	case 0xb4: /* sequence_error_code */
	case 0xb7: /* sequence_end_code */
		return 0;
	case 0xb8: /* group_of_pictures_header */
		if (size < 4)
			return -EINVAL;
		vcp_bs_skip(&b, 25 + 2); /* 25-bit time code, closed, broken_link */
		return b.error ? -EINVAL : 0;
	default:
		break;
	}
	if (code >= 0x01 && code <= 0xaf) /* slice_start_code */
		return 0;
	return -EINVAL;
}

/* One MPEG-2 access unit: start-code delimited segments, each validated.
 * Compliant encoders never emulate start codes, so an unknown code fails
 * the unit closed rather than being skipped over.
 */
static inline int vcp_mpeg2_guard(const u8 *data, size_t size)
{
	size_t i = 0, start = 0;
	bool found = false, useful = false;
	int ret;

	if (vcp_bs_prefix(data, size, 0))
		return -EINVAL;
	for (i = 0; i + 4 <= size;) {
		if (!data[i] && !data[i + 1] && data[i + 2] == 1) {
			if (found) {
				ret = vcp_mpeg2_code_guard(data[start + 3],
							   data + start + 4,
							   i - start - 4);
				if (ret)
					return ret;
				useful = useful || data[start + 3] == 0x00 ||
					 data[start + 3] == 0xb3;
			}
			start = i;
			found = true;
			i += 4;
			continue;
		}
		i++;
	}
	if (!found)
		return -EINVAL;
	ret = vcp_mpeg2_code_guard(data[start + 3], data + start + 4,
				   size - start - 4);
	if (ret)
		return ret;
	useful = useful || data[start + 3] == 0x00 || data[start + 3] == 0xb3;
	return useful ? 0 : -EINVAL;
}

/* Firmware MPG4 box: 16x16..2048x1088. Rectangular 8-bit 4:2:0 only: GMC,
 * sprites, studio, newpred, reduced resolution, scalability and interlace
 * change the decode itself and are refused. VOP payloads need VOL state,
 * so only the coding type is checked there; B-pictures are refused because
 * ones with a P forward reference come back stale, and firmware reports
 * stream errors through its changed flags.
 */
static inline int vcp_mpeg4_vol_tail(struct vcp_bs *b, u32 ver)
{
	u32 width, height, sprite, method;

	width = vcp_bs_bits(b, 13);
	if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
		return -EINVAL;
	height = vcp_bs_bits(b, 13);
	if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
		return -EINVAL;
	if (b->error || width < 16 || width > 2048 || height < 16 || height > 1088)
		return -EINVAL;
	if (vcp_bs_bits(b, 1)) /* interlaced */
		return -EOPNOTSUPP;
	vcp_bs_skip(b, 1); /* obmc_disable */
	sprite = ver == 1 ? vcp_bs_bits(b, 1) : vcp_bs_bits(b, 2);
	if (sprite)
		return -EOPNOTSUPP;
	if (vcp_bs_bits(b, 1)) /* not_8_bit */
		return -EOPNOTSUPP;
	if (vcp_bs_bits(b, 1)) { /* quant_type */
		if (vcp_bs_bits(b, 1))
			vcp_bs_skip(b, 64 * 8);
		if (!b->error && vcp_bs_bits(b, 1))
			vcp_bs_skip(b, 64 * 8);
	}
	if (ver != 1)
		vcp_bs_skip(b, 1); /* quarter_sample: version 2 addition */
	if (b->error)
		return -EINVAL;
	if (!vcp_bs_bits(b, 1)) { /* complexity_estimation_disable */
		method = vcp_bs_bits(b, 2);
		if (method > 2)
			return -EINVAL;
		if (method < 2) {
			if (!vcp_bs_bits(b, 1)) /* shape_complexity */
				vcp_bs_skip(b, 6);
			if (!b->error && !vcp_bs_bits(b, 1)) /* texture set 1 */
				vcp_bs_skip(b, 4);
			if (!b->error && !vcp_bs_bits(b, 1)) /* motion comp */
				vcp_bs_skip(b, 6);
			if (!b->error && !vcp_bs_bits(b, 1)) /* texture set 2 */
				vcp_bs_skip(b, 5);
		} else if (!vcp_bs_bits(b, 1)) { /* version 2 estimation set */
			vcp_bs_skip(b, 19);
		}
	}
	if (b->error)
		return -EINVAL;
	vcp_bs_skip(b, 1); /* resync_marker_disable */
	if (vcp_bs_bits(b, 1)) /* data_partitioned */
		vcp_bs_skip(b, 1); /* reversible_vlc */
	if (b->error)
		return -EINVAL;
	if (ver != 1) {
		if (vcp_bs_bits(b, 2)) /* newpred_enable */
			return -EOPNOTSUPP;
		if (vcp_bs_bits(b, 1)) /* reduced_resolution_enable */
			return -EOPNOTSUPP;
	}
	if (vcp_bs_bits(b, 1)) /* scalability */
		return -EOPNOTSUPP;
	return b->error ? -EINVAL : 0;
}

static inline int vcp_mpeg4_vol_guard(struct vcp_bs *b)
{
	u32 aspect, shape, chroma = 1, res;
	u32 ver = 1;
	bool controlled = false, eop = false;
	unsigned int i, n;
	int ret;

	vcp_bs_skip(b, 1); /* random_accessible_vol */
	vcp_bs_skip(b, 8); /* video_object_type_indication */
	if (vcp_bs_bits(b, 1)) { /* is_object_layer_identifier */
		ver = vcp_bs_bits(b, 4);
		vcp_bs_skip(b, 3); /* visual_object_priority */
		if (ver < 1 || ver > 5)
			return -EINVAL;
	}
	aspect = vcp_bs_bits(b, 4);
	if (aspect == 15) /* extended_par */
		vcp_bs_skip(b, 16);
	if (vcp_bs_bits(b, 1)) { /* vol_control_parameters */
		chroma = vcp_bs_bits(b, 2);
		controlled = true;
		vcp_bs_skip(b, 1); /* low_delay */
		if (vcp_bs_bits(b, 1)) { /* vbv_parameters */
			for (i = 0; i < 6 && !b->error; i++) {
				vcp_bs_skip(b, 15);
				if (vcp_bs_bits(b, 1) != 1)
					return -EINVAL;
			}
			vcp_bs_skip(b, 11);
			if (vcp_bs_bits(b, 1) != 1)
				return -EINVAL;
			vcp_bs_skip(b, 15);
			if (!b->error && vcp_bs_bits(b, 1) != 1)
				return -EINVAL;
		}
	}
	if (b->error || (controlled && chroma != 1))
		return -EINVAL;
	shape = vcp_bs_bits(b, 2);
	if (shape) /* rectangular pictures only */
		return -EOPNOTSUPP;
	if (vcp_bs_bits(b, 1) != 1) /* marker_bit */
		return -EINVAL;
	res = vcp_bs_bits(b, 16); /* vop_time_increment_resolution */
	if (!res || vcp_bs_bits(b, 1) != 1) /* marker_bit */
		return -EINVAL;
	/* The fixed_vop_time_increment width varies by encoder: the standard
	 * derives it from the resolution, Xvid writes wider fields, and the
	 * native ffmpeg encoder emits one extra '1' bit after a zero
	 * fixed_vop_rate. Every candidate still has to validate end to end.
	 */
	if (vcp_bs_bits(b, 1)) { /* fixed_vop_rate */
		for (n = 1; n <= 16; n++) {
			struct vcp_bs t = *b;

			vcp_bs_skip(&t, n);
			ret = vcp_mpeg4_vol_tail(&t, ver);
			if (!ret)
				return 0;
			if (ret == -EOPNOTSUPP)
				eop = true;
		}
		return eop ? -EOPNOTSUPP : -EINVAL;
	}
	for (n = 0; n <= 1; n++) {
		struct vcp_bs t = *b;

		if (n && vcp_bs_bits(&t, 1) != 1)
			continue;
		ret = vcp_mpeg4_vol_tail(&t, ver);
		if (!ret)
			return 0;
		if (ret == -EOPNOTSUPP)
			eop = true;
	}
	return eop ? -EOPNOTSUPP : -EINVAL;
}

static inline int vcp_mpeg4_vo_guard(struct vcp_bs *b)
{
	if (vcp_bs_bits(b, 1)) { /* is_visual_object_identifier */
		if (vcp_bs_bits(b, 4) < 1) /* visual_object_verid */
			return -EINVAL;
		vcp_bs_skip(b, 3); /* visual_object_priority */
	}
	if (vcp_bs_bits(b, 4) != 1) /* visual_object_type: video only */
		return -EOPNOTSUPP;
	if (vcp_bs_bits(b, 1)) { /* video_signal_type */
		vcp_bs_skip(b, 4); /* video_format + video_range */
		if (vcp_bs_bits(b, 1)) /* colour_description */
			vcp_bs_skip(b, 24);
	}
	return b->error ? -EINVAL : 0;
}

static inline int vcp_mpeg4_code_guard(u32 code, const u8 *data, size_t size)
{
	struct vcp_bs b = { .data = data, .size = size };
	u32 type;

	switch (code) {
	case 0xb0: /* visual_object_sequence_start */
		return size < 1 ? -EINVAL : 0;
	case 0xb5: /* visual_object_start */
		return size ? vcp_mpeg4_vo_guard(&b) : -EINVAL;
	case 0xb2: /* user_data */
	case 0xb1: /* visual_object_sequence_end */
		return 0;
	case 0xb3: /* group_of_vop_start */
		if (size < 3)
			return -EINVAL;
		vcp_bs_skip(&b, 20); /* closed, broken, 18-bit time code */
		return b.error ? -EINVAL : 0;
	case 0xb6: /* visual_object_plane_start */
		type = vcp_bs_bits(&b, 2);
		if (b.error || type > 2)
			return -EINVAL;
		if (type == 2)
			/* B-pictures whose forward reference is a P-picture come
			 * back with the previous reference image under their own
			 * timestamp (I-forward B-pictures are exact), so refuse
			 * B-content at queue time instead of showing stale frames.
			 */
			return -EOPNOTSUPP;
		return 0;
	case 0xb4: /* video_session_error */
		return -EPIPE;
	default:
		break;
	}
	if (code <= 0x1f) /* video_object_start_code */
		return 0;
	if (code >= 0x20 && code <= 0x3f) { /* video_object_layer_start_code */
		struct vcp_bs v = { .data = data, .size = size };

		return vcp_mpeg4_vol_guard(&v);
	}
	return -EINVAL;
}

/* MPEG-4 Part 2 elementary stream. Long headers use 00 00 01 xx codes; the
 * short_video_header mode instead starts VOPs with the 22-bit marker also
 * used by H.263 and is refused until a firmware path is proven for it.
 */
static inline int vcp_mpeg4_guard(const u8 *data, size_t size)
{
	size_t i = 0, start = 0, zeros = 0;
	bool found = false, useful = false;
	int ret;

	if (vcp_bs_prefix(data, size, 0))
		return -EINVAL;
	while (!data[zeros])
		zeros++;
	if (data[zeros] == 1) {
		found = true;
	} else if (data[zeros] < 0x80 || (data[zeros] & 0xfc) != 0x80) {
		return -EINVAL;
	} else {
		/* Short-header VOP at the buffer start. */
		return -EOPNOTSUPP;
	}
	for (i = 3; i + 4 <= size;) {
		if (!data[i] && !data[i + 1]) {
			if (data[i + 2] == 1) {
				u32 code = data[start + 3];

				ret = vcp_mpeg4_code_guard(code, data + start + 4,
							   i - start - 4);
				if (ret)
					return ret;
				useful = useful || code == 0xb6 ||
					 (code >= 0x20 && code <= 0x3f);
				start = i;
				i += 4;
				continue;
			}
			if (data[i + 2] < 0x80 || (data[i + 2] & 0xfc) != 0x80) {
				i++;
				continue;
			}
			/* Short-header marker inside a long-header stream. */
			return -EOPNOTSUPP;
		}
		i++;
	}
	if (!found)
		return -EINVAL;
	ret = vcp_mpeg4_code_guard(data[start + 3], data + start + 4,
				   size - start - 4);
	if (ret)
		return ret;
	useful = useful || data[start + 3] == 0xb6 ||
		 (data[start + 3] >= 0x20 && data[start + 3] <= 0x3f);
	return useful ? 0 : -EINVAL;
}

/* Firmware H263 box: 16x16..1408x1152. H.263 start codes are 17- or 22-bit
 * codes inside 00 00 8x bytes, not Annex-B 00 00 01 codes. Picture headers
 * are parsed fully; GOB headers are accepted structurally and their slices
 * stay firmware-owned.
 */
static inline int vcp_h263_pic_guard(struct vcp_bs *b)
{
	u32 format;

	if (vcp_bs_bits(b, 6) != 0x20) /* picture_start_code low 6 bits */
		return -EINVAL;
	vcp_bs_skip(b, 8); /* temporal_reference */
	if (vcp_bs_bits(b, 1) != 1 || vcp_bs_bits(b, 1) != 0)
		return -EINVAL; /* PTYPE spare bits */
	if (vcp_bs_bits(b, 1)) /* split_screen_indicator */
		return -EOPNOTSUPP;
	vcp_bs_skip(b, 2); /* document_camera + freeze_picture_release */
	format = vcp_bs_bits(b, 3);
	if (b->error || !format || format == 6)
		return -EINVAL;
	if (format == 7)
		/* H.263+ extended PTYPE: the firmware START-rejects every
		 * PLUSPTYPE picture (standard or custom size) with status -1
		 * while baseline CIF decodes, so refuse it at queue time.
		 * Baseline formats 1-5 already cover up to 16CIF (1408x1152).
		 */
		return -EOPNOTSUPP;
	vcp_bs_skip(b, 5); /* picture_coding_type, UMV, SAC, AP, PB */
	return b->error ? -EINVAL : 0;
}

/* Group-of-blocks header: 17-bit start code, number, frame id, quantiser.
 * Macroblock contents stay firmware-owned, so parsing stops at GQUANT;
 * there is no extra-insertion flag on a baseline GOB.
 */
static inline int vcp_h263_gob_guard(struct vcp_bs *b)
{
	if (vcp_bs_bits(b, 16) || !vcp_bs_bits(b, 1))
		return -EINVAL;
	vcp_bs_skip(b, 5 + 2 + 5); /* gob_number, gob_frame_id, gquant */
	return b->error ? -EINVAL : 0;
}

/* Classify one 00 00 xx candidate. A 0x80-0x83 third byte is a picture header
 * or a GOB with a small number; EOS shares the high GOB range and GOV/EOS
 * markers are accepted structurally, so an end-of-sequence can never turn a
 * good access unit into a rejected one. A split-screen picture fails the
 * picture parse with EOPNOTSUPP and is then absorbed by the GOB fallback,
 * so the AU fails on having no validated picture (EINVAL): propagating the
 * feature refusal instead would mislabel small-number GOBs that happen to
 * match the split-screen bit pattern.
 */
static inline int vcp_h263_seg_guard(const u8 *data, size_t size, bool *picture)
{
	struct vcp_bs b = { .data = data + 2, .size = size - 2 };
	u8 third = data[2];

	if (size < 3)
		return -EINVAL;
	if (third >= 0x80 && third <= 0x83) {
		if (!vcp_h263_pic_guard(&b)) {
			*picture = true;
			return 0;
		}
		b = (struct vcp_bs){ .data = data, .size = size };
		return vcp_h263_gob_guard(&b);
	}
	if (third >= 0x84 && third <= 0xfb) {
		struct vcp_bs g = { .data = data, .size = size };

		return vcp_h263_gob_guard(&g);
	}
	/* GOV (0x78-0x7b), EOS (0x7c-0x7f) and the top GOB/EOS range. */
	return 0;
}

static inline int vcp_h263_guard(const u8 *data, size_t size)
{
	size_t i = 2, start = 0;
	bool found = false, picture = false;
	int ret;

	if (vcp_bs_prefix(data, size, 0x78))
		return -EINVAL;
	for (; i + 1 <= size;) {
		if (!data[i - 2] && !data[i - 1] && data[i] >= 0x78) {
			if (found) {
				ret = vcp_h263_seg_guard(data + start,
							 i - 2 - start, &picture);
				if (ret)
					return ret;
			}
			start = i - 2;
			found = true;
		}
		i++;
	}
	if (!found)
		return -EINVAL;
	ret = vcp_h263_seg_guard(data + start, size - start, &picture);
	if (ret)
		return ret;
	return picture ? 0 : -EINVAL;
}

/* AV1 temporal unit: a chain of OBUs. Lengths are validated so a claimed
 * size can never run past the submitted buffer; tile and frame contents
 * stay firmware-owned. Temporal units carrying more than three frame OBUs
 * (hidden alt-ref packing from deep encoder lag) stall the firmware
 * without answering, so they are refused: single, dual and triple frame
 * units decode exactly up to 1280x720.
 */
static inline int vcp_av1_guard(const u8 *data, size_t size)
{
	size_t pos = 0;
	unsigned int frames = 0;
	bool obus = false;

	if (!data || !size)
		return -EINVAL;
	while (pos < size) {
		u8 header = data[pos++];
		u32 type = (header >> 3) & 0xf;
		size_t length;
		unsigned int shift;

		if (header & 0x81) /* obu_forbidden_bit + obu_reserved_1bit */
			return -EINVAL;
		if (!type || (type >= 8 && type <= 14))
			return -EINVAL;
		if (type == 3 || type == 6 || type == 7) {
			if (++frames > 3)
				return -EOPNOTSUPP;
		}
		if (header & 0x4) { /* obu_extension_flag */
			if (pos >= size || (data[pos] & 0x7))
				return -EINVAL;
			pos++;
		}
		if (!(header & 0x2)) {
			/* No size field: only the trailing OBU may do this. */
			if (type == 6 && pos == size)
				return -EINVAL;
			obus = true;
			pos = size;
			break;
		}
		length = 0;
		shift = 0;
		for (;;) {
			u8 cont;

			if (pos >= size || shift >= 56)
				return -EINVAL;
			cont = data[pos++];
			length |= (size_t)(cont & 0x7f) << shift;
			if (!(cont & 0x80))
				break;
			shift += 7;
		}
		/* A frame OBU must contain a frame header and tile data. */
		if (length > size - pos || (type == 6 && !length))
			return -EINVAL;
		pos += length;
		obus = true;
	}
	return obus && pos == size ? 0 : -EINVAL;
}

/* HEIF still input: either an ISOBMFF file with an HEIF brand or a raw HEVC
 * still bitstream. Only the container brand (or Annex-B framing) is checked
 * here; the image item layout stays firmware-owned.
 */
static inline int vcp_heif_guard(const u8 *data, size_t size)
{
	static const char brands[] = "mif1msf1heic"
				     "heixhevchevx"
				     "heimheishevm"
				     "hevs";
	u32 box;
	unsigned int i;
	size_t zeros = 0;

	if (!data || !size)
		return -EINVAL;
	while (zeros < size && !data[zeros])
		zeros++;
	if (zeros >= 2 && zeros < size && data[zeros] == 1)
		return 0; /* Annex-B HEVC still; the dispatcher reuses the NAL scan */
	if (size < 12)
		return -EINVAL;
	box = (u32)data[4] << 24 | (u32)data[5] << 16 | (u32)data[6] << 8 | data[7];
	if (box != 0x66747970) /* 'ftyp' */
		return -EINVAL;
	for (i = 0; i + 4 <= sizeof(brands) - 1; i += 4) {
		if (!memcmp(data + 8, brands + i, 4))
			return 0;
	}
	return -EOPNOTSUPP;
}

static inline int vcp_vdec_nal_guard(u32 fourcc, const u8 *data, size_t size)
{
	u32 type;

	if (!size || (data[0] & 0x80))
		return -EINVAL;
	if (fourcc == V4L2_PIX_FMT_H264) {
		type = data[0] & 0x1f;
		if (type == 7)
			return vcp_h264_sps_guard(data + 1, size - 1);
		if (type == 14 || type == 15 || type == 20 || type == 21)
			return -EOPNOTSUPP;
		return 0;
	}
	if (fourcc != V4L2_PIX_FMT_HEVC)
		return -EINVAL;
	if (size < 2 || !(data[1] & 7))
		return -EINVAL;
	if ((data[0] & 1) || (data[1] & 0xf8)) /* nuh_layer_id */
		return -EOPNOTSUPP;
	type = (data[0] >> 1) & 0x3f;
	if (type == 33)
		return vcp_hevc_sps_guard(data + 2, size - 2);
	return 0;
}

/* V4L2 H264/HEVC OUTPUT carries Annex B, with complete NAL units. Inspect
 * every NAL: an AU may contain several SPSs or an AUD/SEI before the SPS.
 * No sequence cache is used, so queueing ahead and DRC cannot bypass checks.
 */
/* VP8 frame tag and key-frame geometry precede the bool-coded partitions. */
static inline int vcp_vp8_guard(const u8 *data, size_t size)
{
	u32 tag, width, height;
	size_t header;

	if (!data || size < 3)
		return -EINVAL;
	tag = data[0] | (u32)data[1] << 8 | (u32)data[2] << 16;
	if (((tag >> 1) & 7) > 3)
		return -EOPNOTSUPP;
	header = tag & 1 ? 3 : 10;
	if (size < header || !(tag >> 5) || (tag >> 5) > size - header)
		return -EINVAL;
	if (!(tag & 1)) {
		if (data[3] != 0x9d || data[4] != 0x01 || data[5] != 0x2a)
			return -EINVAL;
		width = (data[6] | (u32)data[7] << 8) & 0x3fff;
		height = (data[8] | (u32)data[9] << 8) & 0x3fff;
		if (!width || !height)
			return -EINVAL;
		if (width > 4096 || height > 2176)
			return -EOPNOTSUPP;
	}
	return 0;
}

static inline int vcp_vdec_bitstream_guard(u32 fourcc, const u8 *data, size_t size)
{
	size_t i, start = 0, zeros = 0;
	bool found = false;
	int ret;

	if (fourcc == V4L2_PIX_FMT_VP8)
		return vcp_vp8_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_VP9)
		return vcp_vp9_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_MPEG2)
		return vcp_mpeg2_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_MPEG4)
		return vcp_mpeg4_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_H263)
		return vcp_h263_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_AV1)
		return vcp_av1_guard(data, size);
	if (fourcc == V4L2_PIX_FMT_HEIF) {
		size_t zeros = 0;

		ret = vcp_heif_guard(data, size);
		if (ret)
			return ret;
		if (data)
			while (zeros < size && !data[zeros])
				zeros++;
		if (!data || zeros < 2 || zeros >= size || data[zeros] != 1)
			return 0; /* ISOBMFF container: item layout is firmware-owned */
		fourcc = V4L2_PIX_FMT_HEVC; /* raw HEVC still: reuse the NAL scan */
	}
	if (!data)
		return -EINVAL;
	for (i = 0; i < size; i++) {
		if (!data[i]) {
			zeros++;
			continue;
		}
		if (data[i] == 1 && zeros >= 2) {
			if (found) {
				ret = vcp_vdec_nal_guard(fourcc, data + start,
						 i - zeros - start);
				if (ret)
					return ret;
			}
			start = i + 1;
			found = true;
		} else if (!found) {
			return -EINVAL;
		}
		zeros = 0;
	}
	if (!found)
		return -EINVAL;
	return vcp_vdec_nal_guard(fourcc, data + start, size - start);
}

#endif
