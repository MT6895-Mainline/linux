/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MT6895_GPUEB_H
#define __MT6895_GPUEB_H

int mt6895_gpueb_power_control(unsigned int power_on);
int mt6895_gpueb_power_on(void);
bool mt6895_gpueb_available(void);

#endif
