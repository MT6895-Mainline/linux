/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _COMPAT_MTK_CCCI_COMMON_H
#define _COMPAT_MTK_CCCI_COMMON_H

#include <linux/types.h>

#define MD_SYS1 1
#define SMEM_USER_RAW_MD_CONSYS 0

static inline phys_addr_t get_smem_phy_start_addr(int md_id, int user, int *size)
{
if (size)
*size = 0;
return 0;
}

#endif /* _COMPAT_MTK_CCCI_COMMON_H */
