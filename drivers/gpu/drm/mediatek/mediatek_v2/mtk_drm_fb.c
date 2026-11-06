// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_fourcc.h>
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>

#include "mtk_drm_drv.h"
#include "mtk_drm_fb.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_crtc.h"

/*
 * mtk specific framebuffer structure.
 *
 * @fb: drm framebuffer object.
 * @gem_obj: array of gem objects.
 */
struct mtk_drm_fb {
	struct drm_framebuffer base;
	/* For now we only support a single plane */
	struct drm_gem_object *gem_obj;
};

#define to_mtk_fb(x) container_of(x, struct mtk_drm_fb, base)

struct drm_gem_object *mtk_fb_get_gem_obj(struct drm_framebuffer *fb)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);

	return mtk_fb->gem_obj;
}

size_t mtk_fb_get_size(struct drm_framebuffer *fb)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);
	struct mtk_drm_gem_obj *mtk_gem = NULL;

	if (!mtk_fb->gem_obj)
		return 0;

	mtk_gem = to_mtk_gem_obj(mtk_fb->gem_obj);
	if (!mtk_gem)
		return 0;

	return mtk_gem->size;
}

dma_addr_t mtk_fb_get_dma(struct drm_framebuffer *fb)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);
	struct mtk_drm_gem_obj *mtk_gem = NULL;

	if (!mtk_fb->gem_obj)
		return 0;

	mtk_gem = to_mtk_gem_obj(mtk_fb->gem_obj);
	if (!mtk_gem)
		return 0;

	return mtk_gem->dma_addr;
}

bool mtk_drm_fb_is_secure(struct drm_framebuffer *fb)
{
	struct drm_gem_object *gem = NULL;
	struct mtk_drm_gem_obj *mtk_gem = NULL;


	if (!fb)
		return false;
	gem = mtk_fb_get_gem_obj(fb);
	if (!gem)
		return false;
	mtk_gem = to_mtk_gem_obj(gem);
	return mtk_gem->sec;
}

static int mtk_drm_fb_create_handle(struct drm_framebuffer *fb,
				    struct drm_file *file_priv,
				    unsigned int *handle)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);

	return drm_gem_handle_create(file_priv, mtk_fb->gem_obj, handle);
}

static void mtk_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct mtk_drm_fb *mtk_fb = to_mtk_fb(fb);

	drm_framebuffer_cleanup(fb);
	drm_gem_object_put(mtk_fb->gem_obj);

	kfree(mtk_fb);
}

static const struct drm_framebuffer_funcs mtk_drm_fb_funcs = {
	.create_handle = mtk_drm_fb_create_handle,
	.destroy = mtk_drm_fb_destroy,
};

static struct mtk_drm_fb *
mtk_drm_framebuffer_init(struct drm_device *dev,
			 const struct drm_mode_fb_cmd2 *mode,
			 struct drm_gem_object *obj)
{
	struct mtk_drm_fb *mtk_fb;
	int ret;

	mtk_fb = kzalloc(sizeof(*mtk_fb), GFP_KERNEL);
	if (!mtk_fb)
		return ERR_PTR(-ENOMEM);

	drm_helper_mode_fill_fb_struct(dev, &mtk_fb->base,
				   drm_format_info(mode->pixel_format), mode);

	mtk_fb->gem_obj = obj;
	mtk_fb->base.obj[0] = obj;

	ret = drm_framebuffer_init(dev, &mtk_fb->base, &mtk_drm_fb_funcs);
	if (ret) {
		pr_err("failed to initialize framebuffer\n");
		kfree(mtk_fb);
		return ERR_PTR(ret);
	}

	return mtk_fb;
}

struct drm_framebuffer *
mtk_drm_framebuffer_create(struct drm_device *dev,
			   const struct drm_mode_fb_cmd2 *mode,
			   struct drm_gem_object *obj)
{
	struct mtk_drm_fb *mtk_fb;

	mtk_fb = mtk_drm_framebuffer_init(dev, mode, obj);
	if (IS_ERR(mtk_fb))
		return ERR_CAST(mtk_fb);

	return &mtk_fb->base;
}

/*
 * Wait for any exclusive fence in fb's gem object's reservation object.
 *
 * Returns -ERESTARTSYS if interrupted, else 0.
 */
int mtk_fb_wait(struct drm_framebuffer *fb)
{
	struct drm_gem_object *gem;
	struct dma_resv *resv;
	long ret;

	if (!fb)
		return 0;

	gem = mtk_fb_get_gem_obj(fb);
	if (!gem || !gem->dma_buf || !gem->dma_buf->resv)
		return 0;

	resv = gem->dma_buf->resv;
	ret = dma_resv_wait_timeout(resv, DMA_RESV_USAGE_WRITE, true,
				    msecs_to_jiffies(MTK_FENCE_WAIT_TIMEOUT_MS));
	/* > 0 on signal, 0 on timeout, -ERESTARTSYS if interrupted */
	if (ret < 0) {
		DDPAEE("%s:%d, invalid ret:%ld\n",
			__func__, __LINE__,
			ret);
		return ret;
	}
	if (ret == 0) {
		/*
		 * The buffer's exclusive fence never signalled.  Waiting for
		 * it forever leaves the composer in D state and the whole
		 * display frozen (SurfaceFlinger then blocks in present()),
		 * so give up and scan the buffer out anyway: a frame that may
		 * be incomplete is far better than a dead panel.
		 */
		DDPAEE("%s:%d, exclusive fence wait timed out after %d ms, committing anyway\n",
		       __func__, __LINE__, MTK_FENCE_WAIT_TIMEOUT_MS);
	}

	return 0;
}

struct drm_framebuffer *
mtk_drm_mode_fb_create(struct drm_device *dev, struct drm_file *file,
		      const struct drm_format_info *info,
		       const struct drm_mode_fb_cmd2 *cmd)
{
	struct mtk_drm_fb *mtk_fb = NULL;
	struct drm_gem_object *gem = NULL;
	struct mtk_drm_gem_obj *mtk_gem = NULL;
	unsigned int width = cmd->width;
	unsigned int height = cmd->height;
	unsigned int size, bpp;
	int ret;

	if (cmd->pixel_format == DRM_FORMAT_C8)
		goto fb_init;

	gem = drm_gem_object_lookup(file, cmd->handles[0]);
	if (!gem)
		return ERR_PTR(-ENOENT);

	bpp = mtk_drm_format_plane_cpp(cmd->pixel_format, 0);
	size = (height - 1) * cmd->pitches[0] + width * bpp;
	size += cmd->offsets[0];

	mtk_gem = to_mtk_gem_obj(gem);

	if (!drm_is_afbc(cmd->modifier[0]) &&
	    (cmd->modifier[0] & MTK_FMT_SECURE))
		mtk_gem->sec = true;

	/*
	 * AFBC buffers have a header + compressed payload layout, so the
	 * linear-size sanity check below does not apply.  Leave validation to
	 * the exporter and the plane format-modifier check.
	 */
	//TO-DO: should need remove "!mtk_gem->sec"
	if (!drm_is_afbc(cmd->modifier[0]) &&
	    gem->size < size && !mtk_gem->sec) {
		pr_err("%s:%d, size:(%ld,%d), sec:%d\n",
			__func__, __LINE__,
			gem->size, size,
			mtk_gem->sec);
		pr_err("w:%d, h:%d, bpp:(%d,%d), pitch:%d, offset:%d\n",
			width, height,
			cmd->pixel_format, bpp,
			cmd->pitches[0],
			cmd->offsets[0]);
		ret = -EINVAL;
		goto unreference;
	}

fb_init:
	mtk_fb = mtk_drm_framebuffer_init(dev, cmd, gem);
	if (IS_ERR(mtk_fb)) {
		ret = PTR_ERR(mtk_fb);
		goto unreference;
	}

	return &mtk_fb->base;

unreference:
	drm_gem_object_put(gem);
	return ERR_PTR(ret);
}
