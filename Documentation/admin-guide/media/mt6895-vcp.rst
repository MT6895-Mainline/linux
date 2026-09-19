.. SPDX-License-Identifier: GPL-2.0

MT6895 VCP codec integration
===========================

The MT6895 implementation uses bootloader-owned VCP firmware. Linux verifies
the live SRAM metadata and provides mailbox transport, private firmware DMA
allocations and stateful V4L2 codec interfaces. It does not replace the VCP
image. The userspace VA-API adapter is a separate project.

Subsystem boundaries
--------------------

* DVFSRC owns the multimedia voltage/clock transaction. Voltage must be
  confirmed before increasing clocks; clocks decrease before lowering voltage.
  Failed clock handover retains a conservative voltage floor. Display and
  codec requests share this provider rather than independently moving muxes.
* The remoteproc transport validates message lengths, manages shared memory
  and serializes codec ownership. Encoder and decoder reserve the same VCP
  before boot; a competing session receives ``EBUSY``. An unconfirmed secure
  shutdown retains memory and power rather than allowing DMA into freed pages.
* The encoder adapts V4L2 controls and layouts to firmware RPCs. Private staging
  allocations separate client buffers from firmware DMA lifetime. H.264, HEVC
  and the vendor HEIF output have separate size/profile constraints. B-frame
  sessions use the asynchronous completion path.
* The decoder tracks compressed access units, picture timestamps, reference
  surfaces and resolution changes. Firmware MM21/MT2T pictures are converted
  to NV12, NV12M or single-plane P010. Bitstream guards check the exact private
  DMA copy as well as the queued input.
* The xaga device tree supplies the preserved memory map, DMA apertures,
  hardware resources and OPP tables; ``xaga.config`` enables the consumers.

Buffer sharing and lifetime
---------------------------

Linear CAPTURE buffers use videobuf2-dma-sg with bidirectional DMA mappings.
CPU conversion acquires cache ownership before writing and releases it before
publishing a completed buffer. Imported DMA-BUFs additionally use the exporter
CPU-access API, including rollback when a later plane cannot be acquired.
Persistent GPU imports therefore observe new pixels when a pool slot is reused.

Clients must finish consuming an exported allocation before requeuing it.
Exported file descriptors refer to allocations, not to a permanent VA surface
identity. Internal tiled-to-linear conversion remains a CPU copy; DMA-BUF
display consumption does not by itself prove compositor direct scanout.

Diagnostics and regression tests
--------------------------------

Normal frame processing does not print transport, IRQ or queue traces at
information level. With ``CONFIG_DYNAMIC_DEBUG``, enable individual source
files through ``/sys/kernel/debug/dynamic_debug/control``. Failures continue
to have error/warning diagnostics. Capability dumps are explicitly requested
with the corresponding codec ``caps_dump`` module parameter.

The decoder ``perf_frames`` parameter samples a bounded number of pictures::

    echo 8 > /sys/module/mtk_vcp_vdec/parameters/perf_frames

``VCPERF ready_us`` measures time from collecting a DISPLAY notification to
starting conversion, including queueing. It is not firmware decode time.
``y_us`` and ``c_us`` measure the two conversion sections. Disable measurement
for end-to-end throughput comparisons.

Run the host checks from the source tree::

    python3 tools/testing/selftests/mtk_vcp/test_layout.py
    python3 tools/testing/selftests/mtk_vcp/test_capture.py
    python3 tools/testing/selftests/mtk_vcp/test_ownership.py

They exercise actual C functions under address/undefined sanitizers, with an
independent ten-bit packing oracle, 224 MM21 layout/alignment cases, format
bounds, imported buffer acquisition failures and simultaneous engine claims.
The ownership probe in the same directory is intended for the device.
Host tests do not replace pixel, timestamp, drain, DRC and GPU coherency tests.

Performance review, 2026-09-19
-----------------------------

MT2T conversion previously reread packed low bits directly from coherent DMA
memory for each sample. The converter now snapshots each 80-byte group to a
small stack buffer, then unpacks it from cached memory, preserving every bit.
No SIMD state or whole-frame intermediate allocation is needed.

Two alternative MM21 traversal implementations had no demonstrated benefit
and were discarded. The retained MM21 conversion is unchanged. Earlier
30-picture complete-session results were dominated by startup, teardown and
userspace readback and are superseded by the sustained measurements below.

Resolution throughput check, 2026-09-19
----------------------------------------

The final branch was measured with asynchronous long-stream probes and output
to ``/dev/null``. Decode runs discarded 30 warm-up intervals, then measured
329 H.264 intervals from a 360-picture stream and 269 HEVC intervals from a
300-picture stream. Encoder runs submitted 180 pictures and discarded the
first 30 intervals. Every run returned all submitted buffers and completed
LAST/EOS handling. These figures cover the V4L2 client path, including CPU
detiling and userspace dequeue, rather than isolated firmware engine time.

.. list-table:: Sustained 4K throughput
   :header-rows: 1

   * - CPU governor
     - Workload
     - Frames
     - Steady fps
     - p50 interval
     - p95 interval
   * - schedutil
     - H.264 decode
     - 360
     - 39.583
     - 24.794 ms
     - 28.310 ms
   * - schedutil
     - HEVC decode
     - 300
     - 35.495
     - 28.089 ms
     - 31.707 ms
   * - schedutil
     - HEVC encode
     - 180
     - 22.362
     - not sampled
     - not sampled
   * - performance
     - H.264 decode
     - 360
     - 72.906
     - 13.727 ms
     - 14.052 ms
   * - performance
     - HEVC decode
     - 300
     - 64.990
     - 15.334 ms
     - 15.819 ms
   * - performance
     - HEVC encode
     - 180
     - 22.275
     - not sampled
     - not sampled

With the performance governor the CPU policies stayed at 2.0, 2.85 and
2.85 GHz, and both 8-bit decoder paths sustained 4K60. Schedutil repeatedly
reduced CPU frequency and limited the same paths to about 35--40 fps, showing
that CPU detiling and CPU frequency policy remain part of the deployment
contract. HEVC encoding remained near 22 fps under both governors, so its 4K
bottleneck is not CPU frequency. Main10 4K pixel correctness is verified, but
there is no replacement sustained-throughput measurement for it; the obsolete
5 fps short-session figure must not be used as a current capability result.

Remaining limits
----------------

Firmware capability tables are not proof of working codecs, layouts or rate
control. Unsupported layouts must fail closed. H.264 keeps the measured
594 MHz decoder request; HEVC and MT2T/Main10 apply a 1.25 complexity allowance
to request the 660 MHz / 750 mV operating point for 4K60 workloads. Codec
engine access is single-session. DMA-BUF sharing removes an avoidable display
copy but does not remove internal CPU detiling. Default-governor 4K60 decode,
4K60 encode, sustained Main10 throughput, idle suspend/resume, energy
measurements and end-to-end remote-desktop integration remain open work.
