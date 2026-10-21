/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 *
 * Exported prototypes for the GE region store.
 */
#ifndef _GED_GE_H
#define _GED_GE_H

#include <linux/types.h>

int ged_ge_store_init(void);
void ged_ge_store_exit(void);
int ged_ge_alloc(int region_num, uint32_t *region_sizes);
int ged_ge_get(int ge_fd, int region_id, int u32_offset,
	       int u32_size, uint32_t *output_data);
int ged_ge_set(int ge_fd, int region_id, int u32_offset,
	       int u32_size, uint32_t *input_data);
int ged_ge_info(int ge_fd, uint64_t *unique_id);

#endif /* _GED_GE_H */
