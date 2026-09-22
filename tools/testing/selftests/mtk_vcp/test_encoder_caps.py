#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Check actual per-platform tables, format ioctls and HEADER_MODE registration."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
ENC = ROOT / 'drivers/media/platform/mediatek/vcodec/encoder'
source = (ENC / 'mtk_vcodec_enc.c').read_text()
driver = (ENC / 'mtk_vcodec_enc_drv.c').read_text()


def function(name):
    match = re.search(r'(?:static )?(?:int|const struct mtk_video_fmt \*)\s*' + name +
                      r'\([^;{]*\)\n\{', source)
    return source[match.start():source.index('\n}\n', match.start()) + 3]


tables = '\n'.join(re.findall(r'static const struct (?:mtk_video_fmt|mtk_vcodec_enc_pdata) '
                              r'\w+(?:\[\])? =\s*\{.*?\n};', driver, re.S))
layout = re.sub(r'^#include.*$', '', (ENC.parent / 'vcp/mtk_vcp_venc_layout.h').read_text(), flags=re.M)
header_ctrl = re.search(r'v4l2_ctrl_new_std_menu\(handler, ops,\s*'
                        r'V4L2_CID_MPEG_VIDEO_HEADER_MODE,.*?\);', source, re.S)[0]
SHIM = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <linux/videodev2.h>
typedef uint32_t u32;
#ifndef V4L2_PIX_FMT_HEIF
#define V4L2_PIX_FMT_HEIF v4l2_fourcc('H','E','I','F')
#endif
#define BIT(x) (1U << (x))
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define ALIGN(v,a) (((v)+(a)-1)&~((a)-1))
#define clamp(v,lo,hi) ((v)<(lo)?(lo):(v)>(hi)?(hi):(v))
#define MTK_FMT_FRAME 0
#define MTK_FMT_ENC 1
#define VENC_SYS 0
#define VENC_LT_SYS 1
#define MTK_VENC_MIN_W 160
#define MTK_VENC_MIN_H 128
#define mtk_v4l2_venc_dbg(...) do {} while (0)
#define mtk_v4l2_venc_err(...) do {} while (0)
struct mtk_video_fmt { u32 fourcc, type, num_planes; };
struct mtk_vcodec_enc_pdata {
    const struct mtk_video_fmt *capture_formats, *output_formats;
    size_t num_capture_formats, num_output_formats;
    unsigned min_bitrate, max_bitrate, core_id;
    bool uses_vcp, uses_ext, uses_34bit;
};
struct mtk_q_data {
    const struct mtk_video_fmt *fmt;
    unsigned coded_width, coded_height, visible_width, visible_height, field;
    unsigned bytesperline[3], sizeimage[3];
};
struct mock_dev { const struct mtk_vcodec_enc_pdata *venc_pdata; };
struct mtk_vcodec_enc_ctx {
    struct mock_dev *dev;
    void *m2m_ctx;
    struct mtk_q_data q_data;
    unsigned colorspace, ycbcr_enc, quantization, xfer_func;
};
struct file { struct mtk_vcodec_enc_ctx *ctx; };
struct vb2_queue { int unused; };
static struct mtk_vcodec_enc_ctx *file_to_enc_ctx(struct file *f) { return f->ctx; }
static struct mtk_q_data *mtk_venc_get_q_data(struct mtk_vcodec_enc_ctx *c, unsigned type)
{ assert(type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE); return &c->q_data; }
static struct vb2_queue *v4l2_m2m_get_vq(void *c, unsigned type)
{ (void)c; (void)type; return NULL; }
static bool vb2_is_busy(struct vb2_queue *q) { (void)q; return false; }
static void mtk_venc_max_size(struct mtk_vcodec_enc_ctx *c, unsigned *w, unsigned *h)
{ (void)c; *w = 1920; *h = 1088; }
static void v4l_bound_align_image(unsigned *w, unsigned wmin, unsigned wmax, unsigned wa,
                                unsigned *h, unsigned hmin, unsigned hmax, unsigned ha, unsigned sa)
{ (void)wmin; (void)wmax; (void)hmin; (void)hmax; (void)sa;
  *w = ALIGN(*w, 1U << wa); *h = ALIGN(*h, 1U << ha); }
static unsigned header_max, header_skip, header_default;
static void v4l2_ctrl_new_std_menu(void *handler, void *ops, unsigned id,
                                 unsigned max, unsigned skip, unsigned def)
{
    (void)handler; (void)ops;
    assert(id == V4L2_CID_MPEG_VIDEO_HEADER_MODE);
    header_max = max; header_skip = skip; header_default = def;
}
'''
TEST = r'''
int main(void)
{
    const struct mtk_vcodec_enc_pdata *platforms[] = {&mt8173_avc_pdata,
        &mt8173_vp8_pdata, &mt8183_pdata, &mt8188_pdata, &mt8192_pdata,
        &mt8195_pdata, &mt6895_pdata};
    const u32 formats[] = {V4L2_PIX_FMT_NV12M, V4L2_PIX_FMT_NV21M,
        V4L2_PIX_FMT_YUV420M, V4L2_PIX_FMT_YVU420M, V4L2_PIX_FMT_NV12,
        V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_P010};
    for (unsigned p = 0; p < ARRAY_SIZE(platforms); p++) {
        const struct mtk_vcodec_enc_pdata *pd = platforms[p];
        bool vcp = pd->uses_vcp;
        struct mock_dev dev = {pd};
        struct mtk_vcodec_enc_ctx c = {.dev = &dev};
        struct file file = {&c};
        assert(pd->num_output_formats == (vcp ? 9 : 4));
        for (unsigned i = 0; i < ARRAY_SIZE(formats); i++) {
            struct v4l2_fmtdesc desc = {.index = i};
            bool supported = vcp || i < 4;
            int ret = vidioc_enum_fmt(&desc, pd->output_formats, pd->num_output_formats);
            assert(ret == (supported ? 0 : -EINVAL));
            if (supported) assert(desc.pixelformat == formats[i]);
            assert(!!mtk_venc_find_format(formats[i], pd) == supported);
            struct v4l2_format f = {.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                .fmt.pix_mp = {.width = 320, .height = 240, .pixelformat = formats[i]}};
            struct v4l2_format sf = f;
            assert(!vidioc_try_fmt_vid_out_mplane(&file, NULL, &f));
            assert(!vidioc_venc_s_fmt_out(&file, NULL, &sf));
            u32 expected = supported ? formats[i] : V4L2_PIX_FMT_NV12M;
            assert(f.fmt.pix_mp.pixelformat == expected && sf.fmt.pix_mp.pixelformat == expected);
            assert(c.q_data.fmt->fourcc == expected);
            assert(f.fmt.pix_mp.num_planes == c.q_data.fmt->num_planes);
            assert(sf.fmt.pix_mp.num_planes == f.fmt.pix_mp.num_planes);
            if (!vcp) assert(f.fmt.pix_mp.num_planes > 1);
        }
        setup_header_mode(vcp);
        assert(header_max == V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
        assert(header_default == (vcp ? V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME :
                                       V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE));
        assert(!!(header_skip & BIT(V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE)) == vcp);
        assert(!(header_skip & BIT(V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME)));
    }
    puts("PASS: 7 encoder platforms, ENUM/TRY/S_FMT for 9 inputs and HEADER_MODE menus/defaults");
}
'''
with tempfile.TemporaryDirectory(prefix='vcp-encoder-caps-') as tmp:
    p = Path(tmp)
    control = 'static void setup_header_mode(bool vcp) { void *handler = NULL, *ops = NULL;\n' + header_ctrl + '\n}\n'
    (p / 'test.c').write_text(SHIM + tables + layout + '\n'.join(function(n) for n in
        ('vidioc_enum_fmt', 'mtk_venc_find_format', 'vidioc_try_fmt_out',
         'vidioc_try_fmt_vid_out_mplane', 'vidioc_venc_s_fmt_out')) + control + TEST)
    subprocess.run([shutil.which('clang') or 'cc', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-parameter', '-Wno-sign-compare', '-Wno-address-of-packed-member',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(p / 'test.c'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
