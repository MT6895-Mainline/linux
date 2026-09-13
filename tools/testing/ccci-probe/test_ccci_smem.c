// SPDX-License-Identifier: GPL-2.0
/*
 * Host tests for the ccci_smem_dump layout logic (ccci_smem_layout.h):
 * the official fat table's sequential-offset fill, the LK override merge,
 * the bounds check and the id lookup. Run under ASan+UBSan via the Makefile.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../../drivers/misc/mediatek/ccci_smem_dump/ccci_smem_layout.h"

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

static void expect_true(const char *name, int actual)
{
	expect(name, actual != 0, 1);
}

static void expect_false(const char *name, int actual)
{
	expect(name, actual != 0, 0);
}

static int off_of(const struct ccci_smem_region_tbl *tbl, unsigned int id)
{
	int i = ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM, id);

	return i < 0 ? -1 : (int)tbl[i].offset;
}

static void test_default_fill(void)
{
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, NULL, 0);

	expect("default: MDCCCI_DBG offset", off_of(tbl, SMEM_USER_RAW_MDCCCI_DBG), 0);
	expect("default: MDSS_DBG offset", off_of(tbl, SMEM_USER_RAW_MDSS_DBG), 2048);
	expect("default: RAW_RESERVED offset", off_of(tbl, SMEM_USER_RAW_RESERVED), 16384);
	expect("default: RUNTIME_DATA offset", off_of(tbl, SMEM_USER_RAW_RUNTIME_DATA), 59392);
	expect("default: CCISM_MCU offset", off_of(tbl, SMEM_USER_CCISM_MCU), 167936);
	expect("default: CCISM_MCU_EXP offset", off_of(tbl, SMEM_USER_CCISM_MCU_EXP), 906240);
	expect("default: MD_DRDI offset", off_of(tbl, SMEM_USER_MD_DRDI), 1048576);
	expect("default: MD_DATA offset", off_of(tbl, SMEM_USER_MD_DATA), 1114112);
	expect("default: span", (long)ccci_smem_layout_end(tbl, CCCI_SMEM_FAT_NUM),
	       0x110000);
	expect_false("default: check against 0x110000 passes (span == total, empty tail)",
		     ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, 0x110000) < 0);
	expect_false("default: check against 0x120000 passes",
		     ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, 0x120000) < 0);
}

static void test_dfd_override_matches_device_span(void)
{
	/* Synthetic scenario: LK gives DFD 64 KiB, everything shifts by 0x10000
	 * and the span reaches the device's 0x120000 total. */
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_override ov[1] = {
		{ 0, 0, 0x10000u, SMEM_USER_RAW_DFD },
	};

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, 1);

	expect("dfd: DFD size", (long)tbl[0].size, 0x10000);
	expect("dfd: MDCCCI_DBG shifted", off_of(tbl, SMEM_USER_RAW_MDCCCI_DBG),
	       0x10000);
	expect("dfd: CCISM_MCU shifted", off_of(tbl, SMEM_USER_CCISM_MCU),
	       167936 + 0x10000);
	expect("dfd: span reaches 0x120000",
	       (long)ccci_smem_layout_end(tbl, CCCI_SMEM_FAT_NUM), 0x120000);
	expect_false("dfd: check against 0x120000 passes",
		     ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, 0x120000) < 0);
}

static void test_first_override_wins(void)
{
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_override ov[2] = {
		{ 0x21000u, 0, 0x8000u, SMEM_USER_CCISM_SCP },
		{ 0x90000u, 0, 0x1000u, SMEM_USER_CCISM_SCP },
	};

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, 2);

	expect("first-match: CCISM_SCP offset", off_of(tbl, SMEM_USER_CCISM_SCP),
	       0x21000);
	expect("first-match: CCISM_SCP size",
	       (long)tbl[ccci_smem_layout_find(tbl, CCCI_SMEM_FAT_NUM,
					       SMEM_USER_CCISM_SCP)].size,
	       0x8000);
	expect("first-match: next entry chains from override",
	       off_of(tbl, SMEM_USER_RAW_CCB_CTRL), 0x21000 + 0x8000);
}

static void test_check_rejects_overflow(void)
{
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_override ov[1] = {
		{ 0, 0, 0x200000u, SMEM_USER_CCISM_MCU },
	};

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, 1);
	expect_true("overflow: check rejects table beyond 0x120000",
		    ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, 0x120000) < 0);
}

static void test_check_allows_zero_size_beyond(void)
{
	/* Zero-size regions carry no reads; their offsets must not fail the
	 * check even when they sit at the very end of the map. */
	struct ccci_smem_region_tbl tbl[CCCI_SMEM_FAT_NUM];
	struct ccci_smem_override ov[1] = {
		{ 0x120000u, 0, 0, SMEM_USER_MD_DATA },
	};

	memcpy(tbl, ccci_smem_fat_default, sizeof(tbl));
	ccci_smem_layout_build(tbl, CCCI_SMEM_FAT_NUM, ov, 1);
	expect_false("zero-size: check passes with empty tail region",
		     ccci_smem_layout_check(tbl, CCCI_SMEM_FAT_NUM, 0x120000) < 0);
}

static void test_find(void)
{
	expect("find: MDCCCI_DBG index", (long)ccci_smem_layout_find(
		ccci_smem_fat_default, CCCI_SMEM_FAT_NUM,
		SMEM_USER_RAW_MDCCCI_DBG), 5);
	expect("find: CCISM_MCU_EXP index", (long)ccci_smem_layout_find(
		ccci_smem_fat_default, CCCI_SMEM_FAT_NUM,
		SMEM_USER_CCISM_MCU_EXP), 18);
	expect("find: absent id", (long)ccci_smem_layout_find(
		ccci_smem_fat_default, CCCI_SMEM_FAT_NUM, 999u), -1);
}

int main(void)
{
	test_default_fill();
	test_dfd_override_matches_device_span();
	test_first_override_wins();
	test_check_rejects_overflow();
	test_check_allows_zero_size_beyond();
	test_find();

	printf("%u checks, %u failures\n", checks, failures);
	return failures ? 1 : 0;
}
