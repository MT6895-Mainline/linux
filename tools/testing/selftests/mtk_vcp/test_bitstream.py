#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise actual OUTPUT reads and parsers with a cache-maintaining exporter."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
VCP = ROOT / 'drivers/media/platform/mediatek/vcodec/vcp'
source = (VCP / 'mtk_vcp_vdec_drv.c').read_text()


def function(name):
    start = re.search(r'static (?:int|void) ' + name + r'\([^;{]*\)\n\{', source).start()
    return source[start:source.index('\n}\n', start) + 3]


headers = '\n'.join(re.sub(r'^#include.*$', '', (VCP / name).read_text(), flags=re.M)
                    for name in ('mtk_vcp_vp9_bitstream.h', 'mtk_vcp_vdec_bitstream.h'))
SHIM = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <linux/videodev2.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef unsigned long long u64;
#ifndef V4L2_PIX_FMT_AV1
#define V4L2_PIX_FMT_AV1 v4l2_fourcc('A','V','0','1')
#endif
#define VB2_MEMORY_DMABUF 4
#define DMA_FROM_DEVICE 2
#define READ_ONCE(x) (x)
#define VCPDBG(...) do {} while (0)
#define pr_info(...) do { if (0) printf(__VA_ARGS__); } while (0)
static int perf_frames;
struct dma_buf { int unused; };
struct vb2_plane { u32 bytesused, data_offset; struct dma_buf *dbuf; };
struct vb2_queue { void *priv; };
struct vb2_buffer {
    struct vb2_plane planes[1];
    unsigned memory, type, index;
    u64 timestamp;
    struct vb2_queue *vb2_queue;
};
struct vb2_v4l2_buffer { struct vb2_buffer vb2_buf; };
struct vdec_ctx {
    struct v4l2_pix_format_mplane src_fmt, dst_fmt;
    struct { void *cpu; size_t size; u64 dma; } bs;
    void *bs_snapshot, *decoder;
    bool header, source_done, last_pending, wait_capture;
    unsigned source_sequence, prev_dst_planes, prev_dst_size[2];
    u64 source_cookie, next_cookie;
};
static u8 producer[128], mapped[128], snapshot[128], dma_bytes[128];
static bool owned, no_mapping, mutate_on_end;
static int begins, ends, begin_error, end_error, starts;
static int dma_buf_begin_cpu_access(struct dma_buf *b, int direction)
{
    (void)b;
    assert(direction == DMA_FROM_DEVICE && !owned);
    begins++;
    if (begin_error) return begin_error;
    memcpy(mapped, producer, sizeof(mapped));
    owned = true;
    return 0;
}
static int dma_buf_end_cpu_access(struct dma_buf *b, int direction)
{
    (void)b;
    assert(direction == DMA_FROM_DEVICE && owned);
    owned = false;
    ends++;
    if (mutate_on_end) memset(mapped, 0xff, sizeof(mapped));
    return end_error;
}
static void *vb2_plane_vaddr(struct vb2_buffer *v, unsigned plane)
{
    assert(!plane);
    assert(v->memory != VB2_MEMORY_DMABUF || owned);
    return no_mapping ? NULL : mapped;
}
static size_t vb2_plane_size(struct vb2_buffer *v, unsigned plane)
{ (void)v; assert(!plane); return sizeof(mapped); }
static unsigned vb2_get_plane_payload(struct vb2_buffer *v, unsigned plane)
{ assert(!plane); return v->planes[0].bytesused; }
static void *vb2_get_drv_priv(struct vb2_queue *q) { return q->priv; }
static bool is_output(unsigned type) { return type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE; }
static struct v4l2_pix_format_mplane *queue_format(struct vdec_ctx *c, unsigned type)
{ return is_output(type) ? &c->src_fmt : &c->dst_fmt; }
static u64 ktime_get_ns(void) { return 0; }
static void dma_wmb(void) {}
static int mtk_vcp_vdec_start(void *dec, u64 cookie, u64 dma, u32 bytes,
                              u32 capacity, u64 timestamp, u32 *changed)
{
    (void)dec; (void)cookie; (void)dma; (void)timestamp;
    assert(!owned && bytes <= capacity);
    assert(!memcmp(dma_bytes, snapshot, bytes));
    starts++;
    *changed = 0;
    return 0;
}
'''
TEST = r'''
static void vector(struct vdec_ctx *c, struct vb2_v4l2_buffer *src,
                   u32 format, const u8 *data, size_t bytes, int expected)
{
    const unsigned offset = 3;
    u32 changed;
    int old_begins = begins, old_ends = ends, old_starts = starts;
    assert(bytes + offset <= sizeof(producer));
    memset(producer, 0xcc, sizeof(producer));
    memcpy(producer + offset, data, bytes);
    c->src_fmt.pixelformat = format;
    src->vb2_buf.planes[0].bytesused = bytes + offset;
    src->vb2_buf.planes[0].data_offset = offset;
    memset(mapped, 0xff, sizeof(mapped));
    assert(buffer_prepare(&src->vb2_buf) == expected);
    assert(!owned && begins == old_begins + 1 && ends == old_ends + 1);
    memset(mapped, 0xff, sizeof(mapped));
    mutate_on_end = true;
    assert(submit_source(c, src, &changed) == expected);
    mutate_on_end = false;
    assert(!owned && begins == old_begins + 2 && ends == old_ends + 2);
    assert(starts == old_starts + !expected);
    if (!expected) assert(!memcmp(dma_bytes, data, bytes));
}
int main(void)
{
    struct dma_buf dbuf = {0};
    struct vdec_ctx c = {.bs = {dma_bytes, sizeof(dma_bytes), 0}, .bs_snapshot = snapshot};
    struct vb2_queue q = {&c};
    struct vb2_v4l2_buffer src = {.vb2_buf = {.vb2_queue = &q,
        .memory = VB2_MEMORY_DMABUF, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
        .planes = {{0, 0, &dbuf}}}};
    c.src_fmt.num_planes = 1;
    c.src_fmt.plane_fmt[0].sizeimage = sizeof(mapped);
    const u8 mpeg4[] = {0,0,1,0xb5,0x08, 0,0,1,0xb6,0x00};
    const u8 mpeg4_end[] = {0,0,1,0xb5,0x08, 0,0,1,0xb6,0x00, 0,0,1,0xb1};
    const u8 bad_vo[] = {0,0,1,0xb5};
    const u8 non_video_vo[] = {0,0,1,0xb5,0x10};
    const u8 end[] = {0,0,1,0xb1};
    vector(&c, &src, V4L2_PIX_FMT_MPEG4, mpeg4, sizeof(mpeg4), 0);
    vector(&c, &src, V4L2_PIX_FMT_MPEG4, mpeg4_end, sizeof(mpeg4_end), 0);
    vector(&c, &src, V4L2_PIX_FMT_MPEG4, bad_vo, sizeof(bad_vo), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_MPEG4, non_video_vo, sizeof(non_video_vo), -EOPNOTSUPP);
    assert(!vcp_mpeg4_code_guard(0xb1, end + sizeof(end), 0));
    const u8 truncated[] = {0x12,0x00,0x7a,0x00,0x34};
    const u8 empty[] = {0x12,0x00,0x34,0x00};
    const u8 empty_sized[] = {0x12,0x00,0x36,0x00,0x00};
    const u8 bad_reserved[] = {0x12,0x00,0x34,0x01,0x00};
    const u8 good[] = {0x12,0x00,0x34,0x00,0x80};
    const u8 good_sized[] = {0x12,0x00,0x36,0x20,0x01,0x80};
    const u8 missing_size[] = {0x12,0x00,0x36,0x00};
    const u8 oversized[] = {0x12,0x00,0x36,0x00,0x02,0x80};
    vector(&c, &src, V4L2_PIX_FMT_AV1, truncated, sizeof(truncated), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, empty, sizeof(empty), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, empty_sized, sizeof(empty_sized), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, bad_reserved, sizeof(bad_reserved), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, missing_size, sizeof(missing_size), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, oversized, sizeof(oversized), -EINVAL);
    vector(&c, &src, V4L2_PIX_FMT_AV1, good, sizeof(good), 0);
    vector(&c, &src, V4L2_PIX_FMT_AV1, good_sized, sizeof(good_sized), 0);
    u32 changed;
    int old_ends = ends, old_starts = starts;
    begin_error = -EIO;
    assert(buffer_prepare(&src.vb2_buf) == -EIO);
    assert(submit_source(&c, &src, &changed) == -EIO);
    assert(ends == old_ends && starts == old_starts && !owned);
    begin_error = 0;
    no_mapping = true;
    assert(buffer_prepare(&src.vb2_buf) == -EINVAL);
    assert(submit_source(&c, &src, &changed) == -EINVAL);
    no_mapping = false;
    assert(ends == old_ends + 2 && starts == old_starts && !owned);
    end_error = -EIO;
    assert(buffer_prepare(&src.vb2_buf) == -EIO);
    assert(submit_source(&c, &src, &changed) == -EIO);
    assert(ends == old_ends + 4 && starts == old_starts && !owned);
    end_error = 0;
    // MMAP takes no exporter ownership; a queue-time pass cannot replace
    // validating the worker snapshot if the application changes the bytes.
    src.vb2_buf.memory = V4L2_MEMORY_MMAP;
    memcpy(mapped, producer, sizeof(mapped));
    int old_begins = begins;
    assert(!buffer_prepare(&src.vb2_buf));
    memset(mapped, 0xff, sizeof(mapped));
    assert(submit_source(&c, &src, &changed) == -EINVAL);
    assert(begins == old_begins && starts == old_starts);
    src.vb2_buf.planes[0].bytesused = 0;
    src.vb2_buf.planes[0].data_offset = 0;
    assert(buffer_prepare(&src.vb2_buf) == -EINVAL);
    src.vb2_buf.planes[0].data_offset = 1;
    assert(buffer_prepare(&src.vb2_buf) == -EINVAL);
    puts("PASS: MPEG4/AV1 queue and snapshot guards; OUTPUT cache visibility, ownership and failures");
}
'''
with tempfile.TemporaryDirectory(prefix='vcp-bitstream-') as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(SHIM + headers + '\n'.join(function(n) for n in
        ('read_source', 'submit_source', 'buffer_prepare')) + TEST)
    subprocess.run([shutil.which('clang') or 'cc', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-Wno-sign-compare',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(p / 'test.c'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
