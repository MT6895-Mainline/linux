/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub: mmprofile.h already provides the full no-op API when
 * CONFIG_MMPROFILE is off. This header only adds the two dump helpers that
 * mmprofile.h does not declare, so nothing is redefined.
 */

#ifndef __MMPROFILE_FUNCTION_H__
#define __MMPROFILE_FUNCTION_H__

#include "mmprofile.h"

#if IS_ENABLED(CONFIG_MMPROFILE)
unsigned int mmprofile_get_dump_size(void);
void mmprofile_get_dump_buffer(unsigned int start, unsigned long *p_addr,
	unsigned int *p_size);
#else
static inline unsigned int mmprofile_get_dump_size(void)
{
	return 0;
}

static inline void mmprofile_get_dump_buffer(unsigned int start,
	unsigned long *p_addr, unsigned int *p_size)
{
}
#endif

#endif
