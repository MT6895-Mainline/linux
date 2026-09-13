/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * CCCI tag parser shared library - header extracted from probe module
 */
#ifndef _CCCI_TAG_PARSE_H
#define _CCCI_TAG_PARSE_H

#include <linux/types.h>

#define CCCI_TAG_NAME_LEN_V2	64
#define MAX_MD_NUM_AT_LK	4

/**
 * struct ccci_tag_hdr - LK modem_info header (v2/v3)
 * @base_addr: Physical address of tag blob
 * @size: Total size of tag region in bytes
 * @err_no: LK-side error code
 * @version: Tag format version (2 or 3)
 * @tag_num: Number of tags in chain
 * @ld_flag: Load flags from LK
 * @ld_md_errno: Per-MD load error codes
 *
 * Native arm64 layout from official OnePlus 5.10 ccci_util.
 */
struct ccci_tag_hdr {
	unsigned long long base_addr;
	unsigned int size;
	int err_no;
	int version;
	int tag_num;
	unsigned int ld_flag;
	int ld_md_errno[MAX_MD_NUM_AT_LK];
};

/**
 * struct ccci_tag - Single tag in LK chain
 * @tag_name: NUL-terminated tag identifier
 * @data_offset: Offset to payload within tag blob
 * @data_size: Payload size in bytes
 * @next_tag_offset: Offset to next tag (0 for last)
 */
struct ccci_tag {
	char tag_name[CCCI_TAG_NAME_LEN_V2];
	unsigned int data_offset;
	unsigned int data_size;
	unsigned int next_tag_offset;
};

/**
 * struct ccci_smem_layout - Shared memory layout tag payload
 * @base_addr: SMEM physical base
 * @ap_md1_smem_offset: AP<->MD1 region offset
 * @ap_md1_smem_size: AP<->MD1 region size
 * @ap_md3_smem_offset: AP<->MD3 region offset
 * @ap_md3_smem_size: AP<->MD3 region size
 * @md1_md3_smem_offset: MD1<->MD3 region offset
 * @md1_md3_smem_size: MD1<->MD3 region size
 * @total_smem_size: Total SMEM size
 */
struct ccci_smem_layout {
	unsigned long long base_addr;
	unsigned int ap_md1_smem_offset;
	unsigned int ap_md1_smem_size;
	unsigned int ap_md3_smem_offset;
	unsigned int ap_md3_smem_size;
	unsigned int md1_md3_smem_offset;
	unsigned int md1_md3_smem_size;
	unsigned int total_smem_size;
};

/**
 * struct ccci_ccb_layout - Cross-core buffer layout tag payload
 * @addr: CCB physical base
 * @size: CCB total size
 */
struct ccci_ccb_layout {
	unsigned long long addr;
	unsigned int size;
};

/**
 * struct ccci_md_sib - MD secure image binding tag payload
 * @addr: MD1 SIB physical address
 * @size: MD1 SIB size
 */
struct ccci_md_sib {
	unsigned long long addr;
	unsigned int size;
};

/**
 * struct ccci_tag_result - Parsed tag chain results
 * @tags_walked: Number of tags successfully traversed
 * @tag_offset: Offset of last tag processed
 * @smem_found: True if smem_layout tag was found
 * @ccb_found: True if ccb_info tag was found
 * @sib_found: True if md1_sib_info tag was found
 * @smem: Parsed SMEM layout (valid if smem_found)
 * @ccb: Parsed CCB layout (valid if ccb_found)
 * @sib: Parsed SIB info (valid if sib_found)
 */
struct ccci_tag_result {
	unsigned int tags_walked;
	unsigned int tag_offset;
	bool smem_found;
	bool ccb_found;
	bool sib_found;
	struct ccci_smem_layout smem;
	struct ccci_ccb_layout ccb;
	struct ccci_md_sib sib;
};

/**
 * ccci_parse_tag_chain - Parse LK modem tag chain
 * @hdr: Validated tag header
 * @buffer: Mapped tag region
 * @buffer_size: Size of mapped region
 * @result: Output structure for parsed layouts
 *
 * Walks the tag chain and extracts smem_layout, ccb_info, and md1_sib_info
 * tags. All reads stay within buffer boundaries. Partial results on error
 * are diagnostic only, never a usable modem layout.
 *
 * Return: 0 on success (smem_layout found), negative errno otherwise.
 *         -EINVAL: NULL result pointer
 *         -EMSGSIZE: Buffer too small or tag extends beyond bounds
 *         -EBADMSG: Malformed tag (unterminated name, cycle detected)
 *         -ERANGE: Invalid address/offset in tag payload
 *         -EEXIST: Duplicate tag
 *         -ENODATA: No smem_layout tag found
 */
int ccci_parse_tag_chain(const struct ccci_tag_hdr *hdr,
			 const void *buffer, size_t buffer_size,
			 struct ccci_tag_result *result);

/**
 * ccci_validate_tag_hdr - Validate LK tag header
 * @hdr: Header to validate
 * @expected_base: Expected physical base address (0 to skip check)
 * @max_size: Maximum allowed tag region size
 *
 * Return: 0 if valid, negative errno otherwise.
 */
int ccci_validate_tag_hdr(const struct ccci_tag_hdr *hdr,
			  unsigned long long expected_base,
			  unsigned int max_size);

/**
 * ccci_get_lk_tag_hdr - Fetch the LK modem_info header
 * @hdr: Output header, filled on success
 * @source: Out param, set to a static string naming where the header came from
 *
 * The header is NOT stored inside the tag region - LK passes it as the
 * "ccci,modem_info_v2" property of /soc/mddriver, and the region itself holds
 * only the tag chain. Prefers the runtime DT node and falls back to the stash
 * taken from the LK FDT in setup_arch().
 *
 * The returned header is unvalidated; callers must run ccci_validate_tag_hdr()
 * against the bounds of the region they intend to read.
 *
 * Return: 0 on success, negative errno otherwise.
 */
int ccci_get_lk_tag_hdr(struct ccci_tag_hdr *hdr, const char **source);

#endif /* _CCCI_TAG_PARSE_H */
