/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_VCP_VENC_LAYOUT_H
#define __MTK_VCP_VENC_LAYOUT_H

#include <linux/align.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/videodev2.h>

/* Component offsets differ between the public source and the padded private
 * DMA image, including when several components share one V4L2 plane.
 */
struct vcp_venc_component {
	u32 plane, src_offset, dst_offset, stride, row_bytes, rows;
};

struct vcp_venc_input_layout {
	u32 planes, components, buf_width, buf_height;
	u32 stride[3], src_size[3], dst_size[3];
	struct vcp_venc_component component[3];
};

static inline int vcp_venc_calc_layout(u32 fourcc, u32 width, u32 height,
				      struct vcp_venc_input_layout *layout)
{
	u32 i, bps = 1, planes = 1, components = 2;

	/* Bound all multiplications, including P010 and guard rows. */
	if (!width || !height || width > 7680 || height > 4320 ||
	    (width & 1) || (height & 1))
		return -EINVAL;
	switch (fourcc) {
	case V4L2_PIX_FMT_P010:
		bps = 2;
		break;
	/* Packed 32-bit RGB. The firmware advertises these as raw inputs
	 * (VENC cap fmt[12..21]: BGR3/RBG3/AR24/BA24/BGR4/RBG4/BA30/RA30/
	 * AR30/AB30), so the encoder converts RGB to YUV itself and there is
	 * no chroma plane to lay out: one component, four bytes per sample.
	 */
	case V4L2_PIX_FMT_ABGR32:
	case V4L2_PIX_FMT_ARGB32:
		bps = 4;
		components = 1;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		planes = 2;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		components = 3;
		break;
	case V4L2_PIX_FMT_YUV420M:
	case V4L2_PIX_FMT_YVU420M:
		planes = components = 3;
		break;
	default:
		return -EINVAL;
	}

	memset(layout, 0, sizeof(*layout));
	layout->planes = planes;
	layout->components = components;
	layout->buf_width = ALIGN(width, 16);
	layout->buf_height = ALIGN(height, 32);
	for (i = 0; i < components; i++) {
		struct vcp_venc_component *c = &layout->component[i];
		u32 p = planes == 1 ? 0 : i;
		u32 div = i && components == 3 ? 2 : 1;
		u32 coded_rows = i ? layout->buf_height / 2 : layout->buf_height;

		c->plane = p;
		c->src_offset = layout->src_size[p];
		c->dst_offset = layout->dst_size[p];
		c->stride = layout->buf_width * bps / div;
		c->row_bytes = width * bps / div;
		c->rows = i ? height / 2 : height;
		if (!layout->stride[p])
			layout->stride[p] = c->stride;
		layout->src_size[p] += c->stride * c->rows;
		layout->dst_size[p] += c->stride * coded_rows;
	}
	/* Keep the legacy hardware overread allowance outside the image. */
	for (i = 0; i < components; i++) {
		const struct vcp_venc_component *c = &layout->component[i];

		layout->dst_size[c->plane] += c->stride * (i ? 16 : 32);
	}
	return 0;
}

/* S_FMT describes the allocated/coded image; S_SELECTION may later narrow
 * the visible picture. The public contiguous chroma plane immediately
 * follows the visible luma rows, while the private firmware image remains
 * padded to buf_height. Recalculate only source offsets/sizes and row counts.
 */
static inline int vcp_venc_crop_source_layout(struct vcp_venc_input_layout *layout,
					     u32 coded_width, u32 coded_height,
					     u32 visible_width, u32 visible_height,
					     bool padded_nv12_chroma)
{
	u32 i;

	if (!layout || !coded_width || !coded_height ||
	    !visible_width || !visible_height ||
	    visible_width > coded_width || visible_height > coded_height ||
	    (visible_width & 1) || (visible_height & 1))
		return -EINVAL;
	if (padded_nv12_chroma &&
	    (layout->planes != 1 || layout->components != 2 ||
	     coded_height != layout->buf_height))
		return -EINVAL;
	memset(layout->src_size, 0, sizeof(layout->src_size));
	for (i = 0; i < layout->components; i++) {
		struct vcp_venc_component *c = &layout->component[i];

		c->src_offset = padded_nv12_chroma && i == 1 ?
				layout->component[i].dst_offset :
				layout->src_size[c->plane];
		c->rows = i ? visible_height / 2 : visible_height;
		c->row_bytes = (u32)((u64)c->row_bytes * visible_width /
				     coded_width);
		if (c->row_bytes > c->stride)
			return -EINVAL;
		layout->src_size[c->plane] = c->src_offset + c->stride * c->rows;
	}
	return 0;
}

#endif
