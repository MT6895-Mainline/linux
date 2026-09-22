#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Check actual MM21 conversion, format bounds and imported CPU ownership."""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
source = (ROOT / 'drivers/media/platform/mediatek/vcodec/vcp/mtk_vcp_vdec_drv.c').read_text()


def function(name):
    start = source.rindex('\nstatic ', 0, source.index(name + '(')) + 1
    pos = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos]


SHIM = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define min(a,b) ((a)<(b)?(a):(b))
#define clamp_t(t,v,lo,hi) ((v)<(lo)?(lo):(v)>(hi)?(hi):(v))
#define VB2_MEMORY_DMABUF 4
#define DMA_BIDIRECTIONAL 0
struct dma_buf { int id; };
struct sg_table { int id; };
struct vb2_queue { void *dev; };
struct vb2_buffer {
    unsigned num_planes, memory;
    struct vb2_queue *vb2_queue;
    struct { struct dma_buf *dbuf; } planes[2];
};
static struct sg_table tables[2] = {{0},{1}};
static int owned[2], begin_fail = -1, end_fail = -1, calls[2];
static struct sg_table *vb2_dma_sg_plane_desc(struct vb2_buffer *v, unsigned i)
{ (void)v; return &tables[i]; }
static int dma_buf_begin_cpu_access(struct dma_buf *b, int d)
{ assert(!d && !owned[b->id]); if (b->id==begin_fail) return -EIO;
  owned[b->id]=1; return 0; }
static int dma_buf_end_cpu_access(struct dma_buf *b, int d)
{ assert(!d && owned[b->id]); owned[b->id]=0; calls[b->id]++;
  return b->id==end_fail ? -EIO : 0; }
static void dma_sync_sgtable_for_cpu(void *dev, struct sg_table *s, int d)
{ (void)dev; assert(!d && !owned[s->id]); owned[s->id]=1; }
static void dma_sync_sgtable_for_device(void *dev, struct sg_table *s, int d)
{ (void)dev; assert(!d && owned[s->id]); owned[s->id]=0; }
'''

TEST = r'''
static void layout(unsigned w, unsigned h, unsigned th, unsigned misalign)
{
    size_t n=(size_t)w*h, tiled=(size_t)w*((h+th-1)/th)*th;
    u8 *src=malloc(tiled+misalign), *dst=malloc(n+misalign), *expected=malloc(n);
    assert(src && dst && expected);
    for(size_t i=0;i<tiled;i++) src[misalign+i]=(i*37+i/251)%256;
    for(unsigned y=0;y<h;y++) for(unsigned x=0;x<w;x++)
        expected[(size_t)y*w+x]=src[misalign+((size_t)(y/th)*(w/16)+x/16)*th*16+(y%th)*16+x%16];
    detile(dst+misalign,src+misalign,w,h,th);
    assert(!memcmp(dst+misalign,expected,n));
    free(src); free(dst); free(expected);
}
int main(void)
{
    const unsigned widths[]={16,32,320,1280,1920,3840,4096};
    const unsigned heights[]={1,15,16,31,32,64,736,2176};
    for(unsigned i=0;i<sizeof(widths)/sizeof(widths[0]);i++)
        for(unsigned j=0;j<sizeof(heights)/sizeof(heights[0]);j++)
            for(unsigned th=16;th<=32;th+=16)
                for(unsigned off=0;off<2;off++) layout(widths[i],heights[j],th,off);
    for(unsigned step=16;step<=32;step+=16) {
        for(unsigned v=0;v<5000;v++) {
            unsigned n=vdec_dimension(v,16,2176,step);
            assert(n>=16 && n<=2176 && (n-16)%step==0);
        }
        assert(vdec_dimension(UINT32_MAX,16,2176,step)<=2176);
    }
    struct vb2_queue q={0}; struct dma_buf a={0},b={1};
    struct vb2_buffer v={.num_planes=2,.vb2_queue=&q,.planes={{&a},{&b}}};
    assert(!capture_sync(&v,true) && owned[0] && owned[1]);
    assert(!capture_sync(&v,false) && !owned[0] && !owned[1]);
    v.memory=VB2_MEMORY_DMABUF;
    for(int i=0;i<2;i++) {
        begin_fail=i;
        assert(capture_sync(&v,true)==-EIO && !owned[0] && !owned[1]);
    }
    begin_fail=-1;
    assert(!capture_sync(&v,true));
    calls[0]=calls[1]=0; end_fail=0;
    assert(capture_sync(&v,false)==-EIO && calls[0]==1 && calls[1]==1);
    assert(!owned[0] && !owned[1]);
    end_fail=-1;
    assert(!capture_sync(&v,true) && !capture_sync(&v,false));
    puts("PASS: MM21 224 layouts, dimension bounds, imported CPU access/rollback under ASan/UBSan");
}
'''

with tempfile.TemporaryDirectory(prefix='vcp-capture-') as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(SHIM + '\n'.join(function(n) for n in
        ('detile', 'vdec_dimension', 'capture_sync')) + TEST)
    subprocess.run([shutil.which('clang') or 'cc', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                    str(p / 'test.c'), '-o', str(p / 'test')], check=True)
    subprocess.run([str(p / 'test')], check=True)
