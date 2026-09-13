// SPDX-License-Identifier: GPL-2.0
/*
 * Host tests for the CCIF ring-buffer layout (ccci_ccif_ringbuf.h): the
 * official up_98/up_95 queue tables, the block layout written by
 * ccci_ccif_rb_region_fill(), and the fit checks. Run under ASan+UBSan
 * via the Makefile.
 */
#include <stdio.h>
#include <string.h>

#include "../../../drivers/misc/mediatek/ccci_ccif/ccci_ccif_ringbuf.h"

static unsigned char region[1024 * 1024];
static size_t offsets[CCCI_CCIF_QUEUE_NUM];

static unsigned int checks, failures;

static void expect(const char *name, long actual, long expected)
{
	checks++;
	if (actual != expected) {
		failures++;
		printf("not ok %u - %s: got %ld, expected %ld\n",
		       checks, name, actual, expected);
	} else {
		printf("ok %u - %s\n", checks, name);
	}
}

static void test_totals_match_lk_regions(void)
{
	expect("normal total == 737920 (fits CCISM_MCU 738304)",
	       (long)ccci_ccif_rb_total(ccci_ccif_rx_up_98,
					ccci_ccif_tx_up_98), 737920L);
	expect("exception total == 123520 (fits CCISM_MCU_EXP 123904)",
	       (long)ccci_ccif_rb_total(ccci_ccif_rx_exp_up_95,
					ccci_ccif_tx_exp_up_95), 123520L);
	expect("normal fits in 738304",
	       ccci_ccif_rb_fits(ccci_ccif_rx_up_98, ccci_ccif_tx_up_98,
				 738304), 0);
	expect("normal does NOT fit in 737919",
	       ccci_ccif_rb_fits(ccci_ccif_rx_up_98, ccci_ccif_tx_up_98,
				 737919), -1);
	expect("zero queues are empty blocks",
	       (long)ccci_ccif_rb_block(0, 0), CCI_RINGBUF_CTL_LEN);
}

static void test_fill_normal_region(void)
{
	long used = ccci_ccif_rb_region_fill(region, sizeof(region),
					     ccci_ccif_rx_up_98,
					     ccci_ccif_tx_up_98, offsets);
	unsigned int v;
	size_t q1;

	expect("normal fill used", used, 737920L);
	expect("q0 at offset 0", (long)offsets[0], 0);
	/* Block 0: rx 80K + tx 128K + 40 control bytes. */
	q1 = CCI_RINGBUF_CTL_LEN + 80u * 1024u + 128u * 1024u;
	expect("q1 offset chains from block 0", (long)offsets[1], (long)q1);

	memcpy(&v, region + 0, 4);
	expect("q0 header magic[0]", (long)v, (long)CCCI_RBF_HEADER);
	memcpy(&v, region + 4, 4);
	expect("q0 header magic[1]", (long)v, (long)CCCI_RBF_HEADER);
	memcpy(&v, region + 8 + 8, 4);	/* rx_control.length */
	expect("q0 rx length", (long)v, 80L * 1024L);
	memcpy(&v, region + 8 + 12, 4);	/* rx_control.read */
	expect("q0 rx read == 0", (long)v, 0);
	memcpy(&v, region + 8 + 16, 4);	/* rx_control.write */
	expect("q0 rx write == 0", (long)v, 0);
	memcpy(&v, region + 8 + 20, 4);	/* tx_control.length */
	expect("q0 tx length", (long)v, 128L * 1024L);
	memcpy(&v, region + q1 - 8, 4);
	expect("q0 footer magic[-2]", (long)v, (long)CCCI_RBF_FOOTER);
	memcpy(&v, region + q1 - 4, 4);
	expect("q0 footer magic[-1]", (long)v, (long)CCCI_RBF_FOOTER);

	memcpy(&v, region + q1, 4);
	expect("q1 header magic", (long)v, (long)CCCI_RBF_HEADER);
	memcpy(&v, region + q1 + 8 + 8, 4);
	expect("q1 rx length", (long)v, 80L * 1024L);
}

static void test_fill_exception_region(void)
{
	long used = ccci_ccif_rb_region_fill(region, sizeof(region),
					     ccci_ccif_rx_exp_up_95,
					     ccci_ccif_tx_exp_up_95, offsets);

	expect("exception fill used", used, 123520L);
	/* Queue 6 is the last non-empty one: rx 8K + tx 8K. */
	expect("exp q6 block size",
	       (long)ccci_ccif_rb_block(8u * 1024u, 8u * 1024u),
	       2L * 8192L + CCI_RINGBUF_CTL_LEN);
}

static void test_fill_rejects_short_region(void)
{
	long used = ccci_ccif_rb_region_fill(region, 39,
					     ccci_ccif_rx_up_98,
					     ccci_ccif_tx_up_98, offsets);

	expect("fill rejects region smaller than one block", used, -1L);
}

int main(void)
{
	test_totals_match_lk_regions();
	test_fill_normal_region();
	test_fill_exception_region();
	test_fill_rejects_short_region();

	printf("%u checks, %u failures\n", checks, failures);
	return failures ? 1 : 0;
}
