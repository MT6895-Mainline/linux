/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CCIF ring-buffer layout for MT6895 (official md_gen >= 6298 path).
 *
 * Queue-size tables and the block layout mirror the official OnePlus 5.10
 * mt6895 tree: drivers/misc/mediatek/eccci/hif/ccci_hif_ccif.c
 * (md_ccif_ring_buf_init / md_ccif_exp_ring_buf_init, up_98 normal tables,
 * up_95 exception tables) and hif/ccci_ringbuf.c (ccci_create_ringbuf:
 * 8-byte header magic, 24-byte control struct, buffers, 8-byte footer
 * magic; CCI_RINGBUF_CTL_LEN = 8 + 24 + 8 = 40).
 *
 * On the qqcandy device the tables fit the LK-provided regions exactly:
 * 16 normal queues use 737,920 bytes of the 738,304-byte CCISM_MCU region
 * and 16 exception queues use 123,520 bytes of the 123,904-byte
 * CCISM_MCU_EXP region.
 *
 * Pure computation shared by the kernel module and the host tests.
 */
#ifndef _CCCI_CCIF_RINGBUF_H
#define _CCCI_CCIF_RINGBUF_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#else
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#endif

#ifndef SIZE_MAX
#define SIZE_MAX ((size_t)-1)
#endif

#define CCCI_CCIF_QUEUE_NUM	16u

/* Official CCI ring-buffer magic values (hif/ccci_ringbuf.c). */
#define CCCI_RBF_HEADER		0xEE0000EEu
#define CCCI_RBF_FOOTER		0xFF0000FFu

/* 8B header + 24B control + 8B footer. */
#define CCI_RINGBUF_CTL_LEN	40u

#define CCCI_KIB(n)		((unsigned int)(n) * 1024u)

/* rx_queue_buffer_size_up_98[] (normal queues, md_gen >= 6298). */
static const unsigned int ccci_ccif_rx_up_98[CCCI_CCIF_QUEUE_NUM] = {
	CCCI_KIB(80), CCCI_KIB(80), CCCI_KIB(40), CCCI_KIB(80),
	CCCI_KIB(20), CCCI_KIB(20), CCCI_KIB(48), 0,
	CCCI_KIB(8),  CCCI_KIB(16), 0, 0, 0, 0, 0, 0,
};

/* tx_queue_buffer_size_up_98[]. */
static const unsigned int ccci_ccif_tx_up_98[CCCI_CCIF_QUEUE_NUM] = {
	CCCI_KIB(128), CCCI_KIB(40), CCCI_KIB(8), CCCI_KIB(40),
	CCCI_KIB(20),  CCCI_KIB(20), CCCI_KIB(48), 0,
	CCCI_KIB(8),   CCCI_KIB(16), 0, 0, 0, 0, 0, 0,
};

/* rx/tx_exp_buffer_size_up_95[] (exception queues, md_gen >= 6295). */
static const unsigned int ccci_ccif_rx_exp_up_95[CCCI_CCIF_QUEUE_NUM] = {
	CCCI_KIB(12), CCCI_KIB(32), CCCI_KIB(8), 0,
	0, 0, CCCI_KIB(8), 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

static const unsigned int ccci_ccif_tx_exp_up_95[CCCI_CCIF_QUEUE_NUM] = {
	CCCI_KIB(12), CCCI_KIB(32), CCCI_KIB(8), 0,
	0, 0, CCCI_KIB(8), 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

/* One queue block: [header 8][control 24][rx buffer][tx buffer][footer 8]. */
static unsigned int ccci_ccif_rb_block(unsigned int rx, unsigned int tx)
{
	return CCI_RINGBUF_CTL_LEN + rx + tx;
}

/* Total bytes of all 16 queue blocks; SIZE_MAX if it overflows. */
static size_t ccci_ccif_rb_total(const unsigned int *rx, const unsigned int *tx)
{
	size_t total = 0;
	unsigned int i;

	for (i = 0; i < CCCI_CCIF_QUEUE_NUM; i++) {
		size_t blk = ccci_ccif_rb_block(rx[i], tx[i]);

		if (total > SIZE_MAX - blk)
			return SIZE_MAX;
		total += blk;
	}
	return total;
}

/* 0 if all 16 blocks fit in region_size, -1 otherwise. */
static int ccci_ccif_rb_fits(const unsigned int *rx, const unsigned int *tx,
			     size_t region_size)
{
	return ccci_ccif_rb_total(rx, tx) <= region_size ? 0 : -1;
}

static void put_u32(unsigned char *p, unsigned int v)
{
	memcpy(p, &v, sizeof(v));
}

/*
 * Fill one queue block exactly like ccci_create_ringbuf(): zeroes,
 * header/footer magic, rx/tx lengths set, read/write pointers zero.
 * Return 0 on success, -1 if block_size is smaller than needed.
 */
static int ccci_ccif_rb_fill(void *block, size_t block_size,
			     unsigned int rx_size, unsigned int tx_size)
{
	unsigned char *p = block;
	unsigned int need = ccci_ccif_rb_block(rx_size, tx_size);

	if (block_size < need)
		return -1;

	memset(p, 0, need);
	put_u32(p + 0, CCCI_RBF_HEADER);
	put_u32(p + 4, CCCI_RBF_HEADER);
	/* struct ccci_ringbuf at +8: rx_control{read,write,length} then tx. */
	put_u32(p + 8 + 8, rx_size);
	put_u32(p + 8 + 20, tx_size);
	put_u32(p + need - 4, CCCI_RBF_FOOTER);
	put_u32(p + need - 8, CCCI_RBF_FOOTER);
	return 0;
}

/*
 * Fill a whole 16-queue region starting at region[0]; queue offsets are
 * returned in offsets[] (byte offset of each queue block). Return the
 * total bytes written, or -1 if any block would not fit.
 */
static long ccci_ccif_rb_region_fill(void *region, size_t region_size,
				     const unsigned int *rx,
				     const unsigned int *tx,
				     size_t offsets[CCCI_CCIF_QUEUE_NUM])
{
	size_t off = 0;
	unsigned int i;

	for (i = 0; i < CCCI_CCIF_QUEUE_NUM; i++) {
		unsigned int blk = ccci_ccif_rb_block(rx[i], tx[i]);

		if (off > region_size || blk > region_size - off)
			return -1;
		offsets[i] = off;
		if (ccci_ccif_rb_fill((unsigned char *)region + off, blk,
				      rx[i], tx[i]))
			return -1;
		off += blk;
	}
	return (long)off;
}

#endif /* _CCCI_CCIF_RINGBUF_H */
