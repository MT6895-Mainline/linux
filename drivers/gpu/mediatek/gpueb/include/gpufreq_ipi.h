/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal GPUFREQ <-> GPUEB IPI definitions for MT6895 panthor bring-up.
 * Layout must stay aligned with the GPUEB firmware's gpufreq_ipi.h.
 */
#ifndef __GPUFREQ_IPI_H__
#define __GPUFREQ_IPI_H__

#include <linux/types.h>

#define GPUFREQ_IPI_DATA_LEN \
	(sizeof(struct gpufreq_ipi_data) / sizeof(unsigned int))
#define GPUFREQ_STATUS_MEM_SZ	0x400

enum gpufreq_ipi_cmd {
	CMD_INIT_SHARED_MEM = 0,
	CMD_GET_FREQ_BY_IDX = 1,
	CMD_GET_POWER_BY_IDX = 2,
	CMD_GET_OPPIDX_BY_FREQ = 3,
	CMD_GET_LEAKAGE_POWER = 4,
	CMD_SET_LIMIT = 5,
	CMD_POWER_CONTROL = 6,
	CMD_COMMIT = 7,
	CMD_GET_DEBUG_OPP_INFO = 8,
	CMD_GET_DEBUG_LIMIT_INFO = 9,
	CMD_GET_WORKING_TABLE = 10,
	CMD_GET_SIGNED_TABLE = 11,
	CMD_GET_LIMIT_TABLE = 12,
	CMD_SWITCH_LIMIT = 13,
	CMD_FIX_TARGET_OPPIDX = 14,
	CMD_FIX_CUSTOM_FREQ_VOLT = 15,
	CMD_SET_STRESS_TEST = 16,
	CMD_SET_AGING_MODE = 17,
	CMD_SET_GPM_MODE = 18,
	CMD_SET_TEST_MODE = 19,
	CMD_NUM = 20,
};

enum gpufreq_target {
	TARGET_DEFAULT = 0,
	TARGET_GPU = 1,
	TARGET_STACK,
	TARGET_INVALID,
};

enum gpufreq_power_state {
	POWER_OFF = 0,
	POWER_ON,
};

struct gpufreq_ipi_data {
	enum gpufreq_ipi_cmd cmd_id;
	unsigned int target;
	union {
		int oppidx;
		int return_value;
		unsigned int freq;
		unsigned int volt;
		unsigned int power;
		unsigned int power_state;
		unsigned int mode;
		struct {
			unsigned long long status_base;
			unsigned long long debug_base;
			unsigned int status_size;
			unsigned int debug_size;
		} addr;
		struct {
			unsigned int freq;
			unsigned int volt;
		} custom;
		struct {
			unsigned int limiter;
			int ceiling_info;
			int floor_info;
		} setlimit;
	} u;
};

#endif /* __GPUFREQ_IPI_H__ */
