// SPDX-License-Identifier: GPL-2.0-only
/* Dump the newest committed record written by rubens-bootlog. */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define RUBENS_BOOTLOG_MAGIC 0x474f4c42U
#define RUBENS_BOOTLOG_COMMIT 0x54494d43U
#define RUBENS_BOOTLOG_VERSION 1U
#define RUBENS_BOOTLOG_SLOT_SIZE (64U * 1024U)
#define RUBENS_BOOTLOG_HEADER_SIZE 4096U
#define RUBENS_BOOTLOG_PAYLOAD_SIZE (RUBENS_BOOTLOG_SLOT_SIZE - RUBENS_BOOTLOG_HEADER_SIZE)
#define RUBENS_BOOTLOG_SLOTS 2U

struct rubens_bootlog_header {
	uint32_t magic;
	uint32_t version;
	uint32_t generation;
	uint32_t payload_len;
	uint32_t payload_crc;
	uint32_t boot_stage;
	uint32_t commit;
};

static uint32_t crc32_le(uint32_t crc, const unsigned char *buf, size_t len)
{
	while (len--) {
		unsigned int bit;

		crc ^= *buf++;
		for (bit = 0; bit < 8; bit++)
			crc = (crc >> 1) ^ (0xedb88320U & -(crc & 1U));
	}
	return crc;
}

static int read_all(int fd, void *buf, size_t len, off_t offset)
{
	size_t done = 0;

	if (lseek(fd, offset, SEEK_SET) < 0)
		return -1;
	while (done < len) {
		ssize_t ret = read(fd, (unsigned char *)buf + done, len - done);

		if (ret <= 0)
			return -1;
		done += ret;
	}
	return 0;
}

static void usage(const char *name)
{
	fprintf(stderr, "usage: %s <oops-block-device> [output-file]\n", name);
}

int main(int argc, char **argv)
{
	struct rubens_bootlog_header headers[RUBENS_BOOTLOG_SLOTS];
	unsigned char *payload;
	unsigned int newest = 0;
	int fd;
	int found = 0;
	unsigned int slot;
	int out = STDOUT_FILENO;

	if (argc < 2 || argc > 3) {
		usage(argv[0]);
		return 2;
	}

	fd = open(argv[1], O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", argv[1], strerror(errno));
		return 1;
	}

	for (slot = 0; slot < RUBENS_BOOTLOG_SLOTS; slot++) {
		if (read_all(fd, &headers[slot], sizeof(headers[slot]),
				     (off_t)slot * RUBENS_BOOTLOG_SLOT_SIZE))
			continue;
		if (headers[slot].magic != RUBENS_BOOTLOG_MAGIC ||
		    headers[slot].version != RUBENS_BOOTLOG_VERSION ||
		    headers[slot].commit != RUBENS_BOOTLOG_COMMIT ||
		    headers[slot].payload_len > RUBENS_BOOTLOG_PAYLOAD_SIZE)
			continue;
		if (!found || headers[slot].generation > headers[newest].generation) {
			newest = slot;
			found = 1;
		}
	}

	if (!found) {
		fprintf(stderr, "no valid Rubens bootlog records found\n");
		close(fd);
		return 1;
	}

	payload = malloc(headers[newest].payload_len);
	if (!payload) {
		fprintf(stderr, "allocation failed\n");
		close(fd);
		return 1;
	}
	if (read_all(fd, payload, headers[newest].payload_len,
			     (off_t)newest * RUBENS_BOOTLOG_SLOT_SIZE +
			     RUBENS_BOOTLOG_HEADER_SIZE)) {
		fprintf(stderr, "payload read failed: %s\n", strerror(errno));
		free(payload);
		close(fd);
		return 1;
	}
	if (crc32_le(~0U, payload, headers[newest].payload_len) !=
	    headers[newest].payload_crc) {
		fprintf(stderr, "payload CRC mismatch\n");
		free(payload);
		close(fd);
		return 1;
	}

	if (argc == 3) {
		out = open(argv[2], O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
		if (out < 0) {
			fprintf(stderr, "open %s: %s\n", argv[2], strerror(errno));
			free(payload);
			close(fd);
			return 1;
		}
	}
	if (write(out, payload, headers[newest].payload_len) !=
	    (ssize_t)headers[newest].payload_len) {
		fprintf(stderr, "payload write failed: %s\n", strerror(errno));
		if (out != STDOUT_FILENO)
			close(out);
		free(payload);
		close(fd);
		return 1;
	}

	fprintf(stderr, "record=%u generation=%u stage=%u bytes=%u\n",
		newest, headers[newest].generation, headers[newest].boot_stage,
		headers[newest].payload_len);
	if (out != STDOUT_FILENO)
		close(out);
	free(payload);
	close(fd);
	return 0;
}
