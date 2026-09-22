#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Run actual header/event/worker functions against deterministic firmware events."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
source = (ROOT / 'drivers/media/platform/mediatek/vcodec/vcp/mtk_vcp_vdec_drv.c').read_text()


def function(name):
    start = re.search(r'static (?:int|void) ' + name + r'\([^;{]*\)\n\{', source).start()
    return source[start:source.index('\n}\n', start) + 3]


SHIM = r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
typedef uint32_t u32;
typedef unsigned long long u64;
#define DEC_SURFACES 36
#define BIT(n) (1U << (n))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define VCPDBG(...) do { if (0) printf(__VA_ARGS__); } while (0)
#define dev_info(dev,...) do { (void)(dev); if (0) printf(__VA_ARGS__); } while (0)
#define dev_err dev_info
#define VCP_VDEC_FREE_BITSTREAM 1
#define VCP_VDEC_FREE_FRAME 2
#define V4L2_EVENT_SOURCE_CHANGE 1
#define V4L2_EVENT_SRC_CH_RESOLUTION 1
#define V4L2_EVENT_EOS 2
#define VCP_VDEC_H265 2
#define V4L2_PIX_FMT_MT2T 2
#define VB2_BUF_STATE_DONE 1
#define VB2_BUF_STATE_ERROR 2
#define container_of(ptr,type,member) ((type *)((char *)(ptr)-offsetof(type,member)))
struct work_struct { int unused; };
struct vcp_vdec_event { int type; u64 cookie, timestamp; };
struct vdec_pending { unsigned surface; u64 timestamp, fw_done_ns; };
struct v4l2_event { int type; union { struct { int changes; } src_change; } u; };
struct vb2_v4l2_buffer { struct { unsigned index; } vb2_buf; unsigned sequence; };
struct picture { u32 width, height, dpb, stride, buffer_height, fourcc; };
struct v4l2_m2m_ctx { bool ignore_cap_streaming; struct vb2_v4l2_buffer *src; };
struct mock_fh { struct v4l2_m2m_ctx *m2m_ctx; };
struct mock_device { void *dev, *hw, *m2m; };
struct vdec_ctx {
    void *decoder;
    u64 source_cookie, next_cookie;
    bool source_done, submitted, header, stopping, failed;
    bool last_pending, wait_capture, drained, draining;
    u32 header_changed, source_sequence, sequence;
    unsigned pool_count, pending_count, pending_read, codec_id;
    struct { u64 cookie; bool free, pending; } surfaces[DEC_SURFACES];
    struct vdec_pending pending[DEC_SURFACES];
    struct picture pic;
    struct mock_fh fh;
    struct mock_device *dev;
    struct work_struct work;
    int notification, wait;
    struct { unsigned num_planes; } dst_fmt;
};
static struct vcp_vdec_event events[64];
static unsigned event_write, event_read, submissions, completions, errors, job_finishes;
static unsigned incomplete, delay, waits, stop_at, notify_events;
static bool timeout_release, wrong_cookie, stream_error, fw_error;
static struct vdec_ctx *active;
static unsigned long jiffies;
static int atomic_read(int *a) { return *a; }
#define msecs_to_jiffies(x) (x)
#define time_after_eq(a,b) ((a)>=(b))
static void push_release(void)
{
    events[event_write++] = (struct vcp_vdec_event){VCP_VDEC_FREE_BITSTREAM,
        active->source_cookie + wrong_cookie, 0};
    active->notification++;
}
static void mock_wait(void)
{
    jiffies += 20;
    waits++;
    assert(active->submitted && !active->source_done);
    if (stop_at && waits == stop_at) active->stopping = true;
    if (!timeout_release && waits == delay) push_release();
}
#define wait_event_timeout(w,cond,t) do { (void)(w); (void)(t); if (!(cond)) mock_wait(); } while (0)
static unsigned long ktime_get_ns(void) { return 0; }
static int mtk_vcp_vdec_event(void *decoder, struct vcp_vdec_event *event)
{
    (void)decoder;
    if (fw_error) return -EIO;
    if (event_read == event_write) return -EAGAIN;
    *event = events[event_read++];
    return 0;
}
static int submit_source(struct vdec_ctx *c, struct vb2_v4l2_buffer *src, u32 *changed)
{
    (void)src;
    // One firmware DMA area: overwriting an unreleased input is a test failure.
    assert(!c->submitted && (!submissions || c->source_done));
    assert(event_read == event_write);
    c->source_cookie = ++c->next_cookie;
    c->source_done = false;
    submissions++;
    *changed = submissions <= incomplete ? 0 : BIT(0);
    if (stream_error) *changed |= BIT(2);
    waits = 0;
    if (!delay && !timeout_release) push_release();
    return 0;
}
static int mtk_vcp_vdec_picture(void *decoder, struct picture *pic)
{ (void)decoder; (void)pic; assert(active->source_done); return 0; }
static int mtk_vcp_vdec_hw_set_perf(void *hw, u32 w, u32 h, u32 fps)
{ (void)hw; (void)w; (void)h; (void)fps; return 0; }
static int allocate_surfaces(struct vdec_ctx *c) { (void)c; return 0; }
static void capture_format(struct vdec_ctx *c) { (void)c; }
static void v4l2_event_queue_fh(struct mock_fh *fh, const struct v4l2_event *event)
{ (void)fh; (void)event; notify_events++; }
static void vdec_state(struct vdec_ctx *c, const char *tag) { (void)c; (void)tag; }
static int session_start(struct vdec_ctx *c) { (void)c; return 0; }
static int finish_old_sequence(struct vdec_ctx *c) { (void)c; return 0; }
static struct vb2_v4l2_buffer *v4l2_m2m_next_src_buf(struct v4l2_m2m_ctx *m) { return m->src; }
static struct vb2_v4l2_buffer *v4l2_m2m_src_buf_remove(struct v4l2_m2m_ctx *m)
{
    assert(active->source_done && !active->submitted);
    struct vb2_v4l2_buffer *src = m->src;
    assert(src);
    m->src = NULL;
    return src;
}
static void v4l2_m2m_buf_done(struct vb2_v4l2_buffer *src, int state)
{ assert(src && state == VB2_BUF_STATE_DONE); completions++; }
static int deliver_frames(struct vdec_ctx *c) { (void)c; return 0; }
static int queue_surfaces(struct vdec_ctx *c) { (void)c; return 0; }
static int res_change_restart(struct vdec_ctx *c, struct vb2_v4l2_buffer *s)
{ (void)c; (void)s; assert(0); return 0; }
static unsigned v4l2_m2m_num_src_bufs_ready(struct v4l2_m2m_ctx *m) { return !!m->src; }
static int mtk_vcp_vdec_reset(void *d, bool drain) { (void)d; (void)drain; return 0; }
static struct vb2_v4l2_buffer *v4l2_m2m_dst_buf_remove(struct v4l2_m2m_ctx *m)
{ (void)m; return NULL; }
#define vb2_set_plane_payload(...) do {} while (0)
static void v4l2_m2m_last_buffer_done(struct v4l2_m2m_ctx *m, struct vb2_v4l2_buffer *d)
{ (void)m; (void)d; }
static void *v4l2_m2m_get_src_vq(struct v4l2_m2m_ctx *m) { return m; }
static void *v4l2_m2m_get_dst_vq(struct v4l2_m2m_ctx *m) { return m; }
static void vb2_queue_error(void *q) { (void)q; errors++; }
static void v4l2_m2m_job_finish(void *d, struct v4l2_m2m_ctx *m)
{ (void)d; (void)m; job_finishes++; }
'''
TEST = r'''
static void init(struct vdec_ctx *c, struct v4l2_m2m_ctx *m, struct mock_device *d)
{
    memset(c, 0, sizeof(*c));
    memset(m, 0, sizeof(*m));
    c->fh.m2m_ctx = m;
    c->dev = d;
    active = c;
    event_write = event_read = submissions = completions = errors = job_finishes = 0;
    incomplete = delay = waits = stop_at = notify_events = 0;
    timeout_release = wrong_cookie = stream_error = fw_error = false;
    jiffies = 0;
}
int main(void)
{
    struct vdec_ctx c;
    struct v4l2_m2m_ctx m;
    struct mock_device d = {0};
    struct vb2_v4l2_buffer src = {0};
    // Immediate and delayed release, one and multiple incomplete headers.
    for (unsigned n = 1; n <= 4; n++) for (unsigned lag = 0; lag <= 3; lag++) {
        init(&c, &m, &d);
        incomplete = n;
        delay = lag;
        for (unsigned i = 0; i < n; i++) {
            m.src = &src;
            decode_work(&c.work);
            assert(!m.src && completions == i + 1 && submissions == i + 1);
            assert(c.source_done && !c.submitted && !c.header && !errors);
            assert(event_read == event_write);
        }
        m.src = &src;
        decode_work(&c.work);
        assert(c.header && c.source_done && !c.submitted && !errors);
        assert(m.src == &src && completions == n && submissions == n + 1);
        assert(notify_events == 1 && event_read == event_write);
        // The successful parse retains this OUTPUT for exactly one decode
        // submission, which must wait for its own release before DQBUF.
        incomplete = submissions + 1;
        decode_work(&c.work);
        assert(!m.src && completions == n + 1 && submissions == n + 2);
        assert(!c.submitted && c.source_done && !errors);
    }
    // Interrupt the wait on either incomplete or complete sequence headers.
    // A CAPTURE restart preserves the pending OUTPUT and resumes its wait.
    for (unsigned n = 0; n <= 1; n++) {
        init(&c, &m, &d);
        incomplete = n;
        delay = 3;
        stop_at = 1;
        m.src = &src;
        decode_work(&c.work);
        u64 cookie = c.source_cookie;
        assert(c.stopping && c.submitted && !c.source_done && !c.failed);
        assert(m.src == &src && !completions && !errors && submissions == 1);
        c.stopping = false;
        stop_at = 0;
        decode_work(&c.work);
        assert(c.source_cookie == cookie && submissions == 1 && !c.submitted);
        assert(c.source_done && !errors && completions == n);
        assert(c.header == !n && (!!m.src) == !n);
    }
    // If stop and release coincide, preserve the output until restart too.
    init(&c, &m, &d);
    incomplete = 1; delay = stop_at = 1; m.src = &src;
    decode_work(&c.work);
    assert(c.submitted && c.source_done && m.src && !completions && !errors);
    c.stopping = false; stop_at = 0;
    decode_work(&c.work);
    assert(submissions == 1 && completions == 1 && !m.src);
    // Timeout/watchdog/cookie errors never complete or overwrite the input.
    for (unsigned mode = 0; mode < 3; mode++) {
        init(&c, &m, &d); m.src = &src;
        timeout_release = mode == 0; wrong_cookie = mode == 1; fw_error = mode == 2;
        decode_work(&c.work);
        assert(c.failed && errors == 2 && m.src == &src && !completions);
        assert(c.submitted && submissions == 1 && job_finishes == 1);
    }
    init(&c, &m, &d); m.src = &src; stream_error = true;
    decode_work(&c.work);
    assert(c.failed && !c.header && errors == 2 && m.src && !completions);
    puts("PASS: actual header worker, immediate/delayed release, repeated EAGAIN, stop/resume and faults");
}
'''
with tempfile.TemporaryDirectory(prefix='vcp-headers-') as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(SHIM + '\n'.join(function(n) for n in
        ('collect_events', 'wait_header_source', 'parse_headers', 'decode_work')) + TEST)
    subprocess.run([shutil.which('clang') or 'cc', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(p / 'test.c'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
