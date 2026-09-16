// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * CRC8 over the first four header bytes of a sensor-hub control/notify
 * message. Reflected polynomial 0x8c, initial value 0. This is part of the
 * SCP wire protocol and must stay bit-exact.
 */

#define pr_fmt(fmt) "tiny_crc8 " fmt

#include <linux/types.h>

#include "tiny_crc8.h"

/*
 * CRC using polynomial:
 *     1. X^8 + X^5 + X^4 + X^0
 *     2. Little Endian XOR
 *     3. Polynomial = 0x8c
 */
static const uint8_t tiny_crc8_table[] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x00, 0x9d, 0x23, 0xbe, 0x46, 0xdb, 0x65, 0xf8,
	0x8c, 0x11, 0xaf, 0x32, 0xca, 0x57, 0xe9, 0x74
};

uint8_t tiny_crc8(const uint8_t *ptr, uint8_t len)
{
	uint8_t crc = 0;

	while (len-- > 0) {
		crc = *ptr++ ^ crc;
		crc = tiny_crc8_table[crc & 0x0f] ^
			tiny_crc8_table[16 + ((crc >> 4) & 0x0f)];
	}

	return crc;
}
