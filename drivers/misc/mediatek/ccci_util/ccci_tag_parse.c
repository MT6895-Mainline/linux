// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2026 MediaTek Inc.
 *
 * CCCI tag parser shared library - extracted from probe module
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/types.h>

#include "ccci_tag_parse.h"

/* Stashed by setup_arch() from the LK FDT before it is discarded. */
extern u8 xaga_ccci_lk_prop[64];
extern int xaga_ccci_lk_prop_len;
extern char xaga_ccci_lk_prop_name[32];

/* ABI size assertions match official 5.10 ccci_util */
static_assert(sizeof(struct ccci_tag_hdr) == 48, "LK header ABI");
static_assert(sizeof(struct ccci_tag) == 76, "LK tag ABI");
static_assert(sizeof(struct ccci_smem_layout) == 40, "LK smem ABI");
static_assert(sizeof(struct ccci_ccb_layout) == 16, "LK CCB ABI");
static_assert(sizeof(struct ccci_md_sib) == 16, "LK SIB ABI");

#define CCCI_TAG_MAX_COUNT	64

static bool range_valid(unsigned int offset, unsigned int size,
			unsigned int limit)
{
	return offset <= limit && size <= limit - offset;
}

static bool address_valid(unsigned long long addr, unsigned int size)
{
	return (!size || addr) && size <= ~0ULL - addr;
}

/**
 * ccci_validate_tag_hdr - Validate LK tag header
 */
int ccci_validate_tag_hdr(const struct ccci_tag_hdr *hdr,
			  unsigned long long expected_base,
			  unsigned int max_size)
{
	if (!hdr)
		return -EINVAL;

	if (expected_base && hdr->base_addr != expected_base)
		return -ERANGE;

	if (hdr->size < sizeof(struct ccci_tag) || hdr->size > max_size)
		return -EMSGSIZE;

	/* v2 and v3 use the same tag format; unknown versions need review */
	if (hdr->version != 2 && hdr->version != 3)
		return -EOPNOTSUPP;

	if (hdr->tag_num < 0)
		return -EINVAL;

	if (!hdr->tag_num)
		return -ENODATA;

	if (hdr->tag_num > CCCI_TAG_MAX_COUNT)
		return -E2BIG;

	if ((unsigned int)hdr->tag_num >
	    hdr->size / sizeof(struct ccci_tag))
		return -EMSGSIZE;

	if (hdr->err_no || hdr->ld_md_errno[0])
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(ccci_validate_tag_hdr);

static int parse_smem_tag(const void *payload, struct ccci_smem_layout *smem)
{
	memcpy(smem, payload, sizeof(*smem));

	if (!smem->total_smem_size || !smem->ap_md1_smem_size)
		return -ENODATA;

	if (!address_valid(smem->base_addr, smem->total_smem_size))
		return -ERANGE;

	if (!range_valid(smem->ap_md1_smem_offset, smem->ap_md1_smem_size,
			 smem->total_smem_size))
		return -ERANGE;

	if (!range_valid(smem->ap_md3_smem_offset, smem->ap_md3_smem_size,
			 smem->total_smem_size))
		return -ERANGE;

	if (!range_valid(smem->md1_md3_smem_offset, smem->md1_md3_smem_size,
			 smem->total_smem_size))
		return -ERANGE;

	return 0;
}

/**
 * ccci_parse_tag_chain - Parse LK modem tag chain
 *
 * All reads stay within the supplied buffer. No address chasing.
 * Boundary and cycle checks prevent infinite loops and buffer overruns.
 */
int ccci_parse_tag_chain(const struct ccci_tag_hdr *hdr,
			 const void *buffer, size_t buffer_size,
			 struct ccci_tag_result *result)
{
	const char *blob = buffer;
	unsigned int offset = 0, i;
	int ret;

	if (!result)
		return -EINVAL;

	memset(result, 0, sizeof(*result));

	ret = ccci_validate_tag_hdr(hdr, 0, buffer_size);
	if (ret)
		return ret;

	if (!blob || buffer_size < hdr->size)
		return -EMSGSIZE;

	for (i = 0; i < (unsigned int)hdr->tag_num; i++) {
		struct ccci_tag tag;
		const void *payload;
		unsigned int next;

		result->tag_offset = offset;

		/* Boundary check: tag header must fit */
		if (!range_valid(offset, sizeof(tag), hdr->size))
			return -EMSGSIZE;

		memcpy(&tag, blob + offset, sizeof(tag));

		/* Tag name must be NUL-terminated */
		if (!memchr(tag.tag_name, '\0', sizeof(tag.tag_name)))
			return -EBADMSG;

		/* Boundary check: payload must fit */
		if (!range_valid(tag.data_offset, tag.data_size, hdr->size))
			return -ERANGE;

		payload = blob + tag.data_offset;

		/* Parse known tags */
		if (!strcmp(tag.tag_name, "smem_layout")) {
			if (result->smem_found)
				return -EEXIST;
			if (tag.data_size < sizeof(result->smem))
				return -EMSGSIZE;
			ret = parse_smem_tag(payload, &result->smem);
			if (ret)
				return ret;
			result->smem_found = true;

		} else if (!strcmp(tag.tag_name, "ccb_info")) {
			if (result->ccb_found)
				return -EEXIST;
			if (tag.data_size < sizeof(result->ccb))
				return -EMSGSIZE;
			memcpy(&result->ccb, payload, sizeof(result->ccb));
			if (!address_valid(result->ccb.addr, result->ccb.size))
				return -ERANGE;
			result->ccb_found = true;

		} else if (!strcmp(tag.tag_name, "md1_sib_info")) {
			if (result->sib_found)
				return -EEXIST;
			if (tag.data_size < sizeof(result->sib))
				return -EMSGSIZE;
			memcpy(&result->sib, payload, sizeof(result->sib));
			if (!address_valid(result->sib.addr, result->sib.size))
				return -ERANGE;
			result->sib_found = true;
		}

		result->tags_walked++;
		next = tag.next_tag_offset;

		/* Last tag: next_tag_offset may be zero or forward pointer */
		if (i + 1 == (unsigned int)hdr->tag_num && !next)
			break;

		/* Cycle detection: next must advance forward */
		if (next <= offset || next - offset < sizeof(tag))
			return -EBADMSG;

		/* Boundary check: next offset must be in range */
		if (next > hdr->size)
			return -ERANGE;

		offset = next;
	}

	/* Success requires at least smem_layout */
	return result->smem_found ? 0 : -ENODATA;
}
EXPORT_SYMBOL_GPL(ccci_parse_tag_chain);

/**
 * ccci_get_lk_tag_hdr - Fetch the LK modem_info header
 */
int ccci_get_lk_tag_hdr(struct ccci_tag_hdr *hdr, const char **source)
{
	struct device_node *node;
	const void *raw = NULL;
	int len = 0, ret = -ENODEV;

	if (!hdr || !source)
		return -EINVAL;

	*source = "none";

	node = of_find_compatible_node(NULL, NULL, "mediatek,mddriver");
	if (node) {
		raw = of_get_property(node, "ccci,modem_info_v2", &len);
		if (raw) {
			*source = "runtime DT";
			if (len < (int)sizeof(*hdr)) {
				ret = -EMSGSIZE;
			} else {
				memcpy(hdr, raw, sizeof(*hdr));
				ret = 0;
			}
		}
		of_node_put(node);
		/* A malformed present property is an error, not a fallback. */
		if (raw)
			return ret;
	}

	if (!xaga_ccci_lk_prop_len)
		return -ENODEV;
	if (strcmp(xaga_ccci_lk_prop_name, "ccci,modem_info_v2"))
		return -EOPNOTSUPP;
	if (xaga_ccci_lk_prop_len < 0 ||
	    xaga_ccci_lk_prop_len > sizeof(xaga_ccci_lk_prop))
		return -EMSGSIZE;
	if (xaga_ccci_lk_prop_len < (int)sizeof(*hdr))
		return -EMSGSIZE;

	*source = "LKINFO stash";
	memcpy(hdr, xaga_ccci_lk_prop, sizeof(*hdr));
	return 0;
}
EXPORT_SYMBOL_GPL(ccci_get_lk_tag_hdr);
