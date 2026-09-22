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

#endif
