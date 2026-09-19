#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise actual MT2T conversion with independent packed low-bit stimuli."""
from pathlib import Path
import random
import re
import shutil
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
source = (ROOT/'drivers/media/platform/mediatek/vcodec/vcp/mtk_vcp_vdec_drv.c').read_text()
start = source.index('static void detile_10_chroma(')
end = source.index('static bool capture_fits(', start)
actual = source[start:end]
# Only the two pure conversion functions are needed.
last = actual.index('static void detile_10_plane(')
pos = actual.index('{', last) + 1
depth = 1
while depth:
    depth += (actual[pos] == '{') - (actual[pos] == '}')
    pos += 1
actual = actual[:pos]
shim = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint16_t __le16;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define cpu_to_le16(x) (x)
#else
#define cpu_to_le16(x) __builtin_bswap16(x)
#endif
'''
main = r'''
int main(int argc, char **argv) {
    assert(argc == 3);
    unsigned w = atoi(argv[1]), h = atoi(argv[2]);
    size_t luma = (size_t)w*h*5/4, total = luma*3/2;
    u8 *tile = malloc(total);
    __le16 *out = calloc((size_t)w*h*3/2, sizeof(*out));
    assert(tile && out && fread(tile, 1, total, stdin) == total);
    detile_10_plane(out, tile, w, 1, w, h);
    detile_10_chroma(out+w*h, tile+luma, w, h);
    assert(fwrite(out, 2, (size_t)w*h*3/2, stdout) == (size_t)w*h*3/2);
    free(out); free(tile);
}
'''


def pack(y, u, v, w, h):
    tile = bytearray(w*h*15//8)
    for row in range(h):
        for col in range(w):
            base = ((row//32)*(w//16)+col//16)*640 + (row%32//4)*80
            value = y[row*w+col]
            tile[base+col%16] |= (value & 3) << (2*(row%4))
            tile[base+16+(row%4)*16+col%16] = value >> 2
    for plane, data in enumerate((u, v)):
        for row in range(h//2):
            for col in range(w//2):
                group = (row//16)*(w//4)+(col//8)*4+(row%16)//4
                base = w*h*5//4 + group*80
                value = data[row*(w//2)+col]
                tile[base+2*(col%8)+plane] |= (value & 3) << (2*(row%4))
                tile[base+16+2*((row%4)*8+col%8)+plane] = value >> 2
    return tile


with tempfile.TemporaryDirectory(prefix='vcp-mt2t-') as tmp:
    p = Path(tmp)
    (p/'test.c').write_text(shim+actual+main)
    subprocess.run([shutil.which('clang') or 'cc', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', str(p/'test.c'), '-o', str(p/'test')], check=True)
    rng = random.Random(6895)
    for w, h in [(32, 64), (64, 128), (320, 256), (640, 512)]:
        y = [rng.randrange(1024) for _ in range(w*h)]
        u = [rng.randrange(1024) for _ in range(w*h//4)]
        v = [rng.randrange(1024) for _ in u]
        expected = y + [value for uv in zip(u, v) for value in uv]
        expected = struct.pack('<'+'H'*len(expected), *(value << 6 for value in expected))
        result = subprocess.run([str(p/'test'), str(w), str(h)], input=pack(y,u,v,w,h),
                                stdout=subprocess.PIPE, check=True)
        assert result.stdout == expected, (w,h)
    print('PASS: MT2T Y/UV all 10 bits, four geometries, exact P010 under ASan/UBSan')
