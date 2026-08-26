/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MT6895_GPUEB_H
#define __MT6895_GPUEB_H

int mt6895_gpueb_power_control(unsigned int power_on);
int mt6895_gpueb_power_on(void);
bool mt6895_gpueb_available(void);

/*
 * Commit a working-table OPP index to the GPUEB (CMD_COMMIT).
 * Target values match enum gpufreq_target in
 * drivers/gpu/mediatek/gpueb/include/gpufreq_ipi.h: 1 = TARGET_GPU,
 * 2 = TARGET_STACK. The EB applies clock, buck and its private VSRAM
 * rails with the proper sequencing for that domain.
 */
int mt6895_gpueb_commit(unsigned int target, unsigned int oppidx);

#endif
