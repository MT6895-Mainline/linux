/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal stub for downstream tinysys SCMI interface.
 * The full SCMI/tinysys driver is not ported yet.
 */
#ifndef __TINYSYS_SCMI_H__
#define __TINYSYS_SCMI_H__

#include <linux/device.h>
#include <linux/scmi_protocol.h>

struct scmi_tinysys_info_st {
void *ph;
struct scmi_device *sdev;
};

static inline struct scmi_tinysys_info_st *get_scmi_tinysys_info(void)
{
return NULL;
}

static inline int scmi_tinysys_common_set(void *ph, int feature_id,
  int cmd, int arg0, int arg1,
  int arg2, int arg3)
{
return -ENOTSUPP;
}

#endif
