/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2024 The Linux Foundation
 *
 * Internal interface between the SCP sensor-hub transport and the Linux
 * consumers (currently the IIO frontend).
 */

#ifndef _SENSORHUB_H_
#define _SENSORHUB_H_

#include <linux/types.h>

#include "hf_sensor_type.h"
#include "hf_sensor_io.h"   /* struct sensor_info */

/* A single decoded DATA_ACTION sample from the SCP. */
struct sensorhub_sample {
	int sensor_type;
	int64_t timestamp; /* host boot time, nanoseconds */
	int32_t value[6];
};

typedef void (*sensorhub_sample_cb_t)(const struct sensorhub_sample *s);

int sensorhub_register_sample_cb(sensorhub_sample_cb_t cb);
void sensorhub_unregister_sample_cb(sensorhub_sample_cb_t cb);

int sensorhub_sensor_enable(int sensor_type, int64_t delay, int64_t latency);
int sensorhub_sensor_disable(int sensor_type);

unsigned int sensorhub_sensor_count(void);
const struct sensor_info *sensorhub_sensor_at(unsigned int index);
const struct sensor_info *sensorhub_find_sensor(int sensor_type);

/* IIO frontend (sensorhub_iio.c) */
int sensorhub_iio_init(struct device *parent);
void sensorhub_iio_exit(void);

#endif
