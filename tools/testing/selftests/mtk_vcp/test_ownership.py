#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Race the actual VCP claim/release functions with two codec owners."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]


def function(source, name):
    start = re.search(r'^(?:static )?(?:int|void) ' + name + r'\([^;]*?\n\{', source, re.M).start()
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


SHIM = r'''
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#define RPROC_OFFLINE 0
#define WARN_ON(x) (assert(!(x)), 0)
#define mutex_lock(p) pthread_mutex_lock(p)
#define mutex_unlock(p) pthread_mutex_unlock(p)
struct rproc { pthread_mutex_t lock; int state; };
struct mtk_vcp { struct rproc *rproc; const void *codec_owner; bool powered; };
'''
TEST = r'''
static struct rproc rp = { .lock = PTHREAD_MUTEX_INITIALIZER };
static struct mtk_vcp vcp = { .rproc = &rp };
static pthread_barrier_t barrier;
static int owners[2], results[2];
static void *race(void *arg) {
    int id = *(int *)arg;
    pthread_barrier_wait(&barrier);
    results[id] = mtk_vcp_claim(&vcp, &owners[id]);
    return NULL;
}
int main(void) {
    assert(mtk_vcp_claim(&vcp, NULL) == -EINVAL);
    for (int n = 0; n < 200; n++) {
        pthread_t a, b;
        owners[0] = 0; owners[1] = 1;
        pthread_barrier_init(&barrier, NULL, 2);
        pthread_create(&a, NULL, race, &owners[0]);
        pthread_create(&b, NULL, race, &owners[1]);
        pthread_join(a, NULL); pthread_join(b, NULL);
        assert((results[0] == 0 && results[1] == -EBUSY) ||
               (results[1] == 0 && results[0] == -EBUSY));
        const void *winner = &owners[results[0] == 0 ? 0 : 1];
        const void *loser = &owners[results[0] == 0 ? 1 : 0];
        assert(!mtk_vcp_claim(&vcp, winner));
        rp.state = 1; vcp.powered = true;
        assert(!mtk_vcp_claim(&vcp, winner));
        assert(mtk_vcp_claim(&vcp, loser) == -EBUSY);
        rp.state = RPROC_OFFLINE; vcp.powered = false;
        /* Failed hardware cleanup keeps the reservation even when offline. */
        assert(mtk_vcp_claim(&vcp, loser) == -EBUSY);
        mtk_vcp_release(&vcp, winner);
        assert(!mtk_vcp_claim(&vcp, loser));
        mtk_vcp_release(&vcp, loser);
        pthread_barrier_destroy(&barrier);
    }
    rp.state = 1;
    assert(mtk_vcp_claim(&vcp, &owners[0]) == -EBUSY);
    rp.state = RPROC_OFFLINE; vcp.powered = true;
    assert(mtk_vcp_claim(&vcp, &owners[0]) == -EBUSY);
    puts("PASS: 200 simultaneous claims, both codec orders, restart and retained cleanup");
}
'''
source = (ROOT / 'drivers/remoteproc/mtk_vcp.c').read_text()
with tempfile.TemporaryDirectory(prefix='vcp-ownership-') as tmp:
    p = Path(tmp)
    (p / 'test.c').write_text(SHIM + function(source, 'mtk_vcp_claim') +
                             function(source, 'mtk_vcp_release') + TEST)
    subprocess.run([shutil.which('clang') or 'cc', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-pthread', str(p/'test.c'),
                    '-o', str(p/'test')], check=True)
    subprocess.run([str(p/'test')], check=True)
