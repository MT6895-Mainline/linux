/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 *
 * Minimal MediaTek "GED bridge" ABI for the gralloc-extra (GE) region store.
 *
 * This is the wire format spoken by vendor/libgralloc_extra.so and libged.so
 * on the device.  It is reproduced verbatim so the proprietary userspace blobs
 * work unmodified:
 *
 *   open("/proc/ged")                     -> bridge fd
 *   ioctl(fd, 0xc0286764, &package)       -> GE_ALLOC  (nr 100)
 *   ioctl(fd, 0xc0286765, &package)       -> GE_GET    (nr 101)
 *   ioctl(fd, 0xc0286766, &package)       -> GE_SET    (nr 102)
 *
 * GED_MAGIC is 'g' (0x67), so _IOWR('g', 100, struct GED_BRIDGE_PACKAGE)
 * == 0xc0286764.  Keep these in sync with the .so or the ioctl numbers drift.
 */
#ifndef _GED_GE_ABI_H
#define _GED_GE_ABI_H

#include <linux/types.h>

#define GED_MAGIC 'g'

/* Region count fixed by the vendor userspace region_sizes[] array. */
#define GED_GE_ALLOC_STRUCT_MAX 18

/* Bridge command IDs (from the vendor ged_bridge_id.h). */
#define GED_BRIDGE_COMMAND_GE_ALLOC   100
#define GED_BRIDGE_COMMAND_GE_GET     101
#define GED_BRIDGE_COMMAND_GE_SET     102
#define GED_BRIDGE_COMMAND_GE_INFO    105

#define GED_IOWR(INDEX)  _IOWR(GED_MAGIC, INDEX, struct GED_BRIDGE_PACKAGE)
#define GED_GET_BRIDGE_ID(X)  _IOC_NR(X)

#define GED_IO_GE_ALLOC  GED_IOWR(GED_BRIDGE_COMMAND_GE_ALLOC)
#define GED_IO_GE_GET    GED_IOWR(GED_BRIDGE_COMMAND_GE_GET)
#define GED_IO_GE_SET    GED_IOWR(GED_BRIDGE_COMMAND_GE_SET)

/*
 * Opaque package passed by pointer as the ioctl arg.  The kernel copies this
 * in, then dispatches on ui32FunctionID and copies pvParamIn/pvParamOut.
 */
struct GED_BRIDGE_PACKAGE {
	uint32_t ui32FunctionID;
	int32_t  i32Size;
	void    *pvParamIn;
	int32_t  i32InBufferSize;
	void    *pvParamOut;
	int32_t  i32OutBufferSize;
};

/* GE - gralloc_extra functions */
struct GED_BRIDGE_IN_GE_ALLOC {
	int      region_num;
	uint32_t region_sizes[0];
};

struct GED_BRIDGE_OUT_GE_ALLOC {
	int ge_fd;
	int eError;
};

struct GED_BRIDGE_IN_GE_GET {
	int ge_fd;
	int region_id;
	/* uint32_* is counted in 32-bit words, not bytes. */
	int uint32_offset;
	int uint32_size;
};

struct GED_BRIDGE_OUT_GE_GET {
	int      eError;
	uint32_t data[0];
};

struct GED_BRIDGE_IN_GE_SET {
	int      ge_fd;
	int      region_id;
	int      uint32_offset;
	int      uint32_size;
	uint32_t data[0];
};

struct GED_BRIDGE_OUT_GE_SET {
	int eError;
};

struct GED_BRIDGE_IN_GE_INFO {
	int ge_fd;
};

struct GED_BRIDGE_OUT_GE_INFO {
	uint64_t unique_id;
	int      eError;
};

#endif /* _GED_GE_ABI_H */
