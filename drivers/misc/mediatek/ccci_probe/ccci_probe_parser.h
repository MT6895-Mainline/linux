/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _CCCI_PROBE_PARSER_H
#define _CCCI_PROBE_PARSER_H

#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#else
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#endif

#define CCCI_TAG_NAME_LEN_V2	64
#define MAX_MD_NUM_AT_LK		4
#define CCCI_TAG_MEM_BASE		0x00000000bdbf0000ULL
#define CCCI_TAG_MEM_SIZE		0x00010000U
#define CCCI_PROBE_MAX_TAGS	64

/* Native arm64 LK layouts from the official OnePlus 5.10 ccci_util. */
struct _ccci_lk_info_v2 {
	unsigned long long lk_info_base_addr;
	unsigned int lk_info_size;
	int lk_info_err_no;
	int lk_info_version;
	int lk_info_tag_num;
	unsigned int lk_info_ld_flag;
	int lk_info_ld_md_errno[MAX_MD_NUM_AT_LK];
};

struct _ccci_tag_v2 {
	char tag_name[CCCI_TAG_NAME_LEN_V2];
	unsigned int data_offset;
	unsigned int data_size;
	unsigned int next_tag_offset;
};

struct _smem_layout {
	unsigned long long base_addr;
	unsigned int ap_md1_smem_offset;
	unsigned int ap_md1_smem_size;
	unsigned int ap_md3_smem_offset;
	unsigned int ap_md3_smem_size;
	unsigned int md1_md3_smem_offset;
	unsigned int md1_md3_smem_size;
	unsigned int total_smem_size;
};

struct _ccb_layout {
	unsigned long long ccb_data_buffer_addr;
	unsigned int ccb_data_buffer_size;
};

struct _sib_item {
	unsigned long long md1_sib_addr;
	unsigned int md1_sib_size;
};

_Static_assert(sizeof(struct _ccci_lk_info_v2) == 48, "LK header ABI");
_Static_assert(sizeof(struct _ccci_tag_v2) == 76, "LK tag ABI");
_Static_assert(sizeof(struct _smem_layout) == 40, "LK smem ABI");
_Static_assert(sizeof(struct _ccb_layout) == 16, "LK CCB ABI");
_Static_assert(sizeof(struct _sib_item) == 16, "LK SIB ABI");

struct ccci_probe_result {
	unsigned int tags_walked;
	unsigned int tag_offset;
	bool smem_found;
	bool ccb_found;
	bool sib_found;
	struct _smem_layout smem;
	struct _ccb_layout ccb;
	struct _sib_item sib;
};

enum ccci_probe_state {
	CCCI_PROBE_UNAVAILABLE,
	CCCI_PROBE_READY,
	CCCI_PROBE_QUEUED,
	CCCI_PROBE_RUNNING,
	CCCI_PROBE_DONE,
	CCCI_PROBE_FAILED,
};

/*
 * Caller serializes state changes. Return 1 to queue, 0 for a no-op, or an
 * errno. COMING/GOING modules cannot arm a read; writing 0 never resets it.
 */
static inline int ccci_probe_request_read(enum ccci_probe_state *state,
					 bool live, bool on)
{
	if (!on)
		return 0;
	if (!live)
		return -EAGAIN;
	if (*state == CCCI_PROBE_UNAVAILABLE)
		return -ENODATA;
	if (*state != CCCI_PROBE_READY)
		return -EALREADY;
	*state = CCCI_PROBE_QUEUED;
	return 1;
}

static inline bool ccci_probe_range_valid(unsigned int offset,
					  unsigned int size,
					  unsigned int limit)
{
	return offset <= limit && size <= limit - offset;
}

static inline bool ccci_probe_address_valid(unsigned long long addr,
					    unsigned int size)
{
	return (!size || addr) && size <= ~0ULL - addr;
}

static inline int ccci_probe_validate_header(const struct _ccci_lk_info_v2 *hdr)
{
	if (!hdr)
		return -EINVAL;
	if (hdr->lk_info_base_addr != CCCI_TAG_MEM_BASE)
		return -ERANGE;
	if (hdr->lk_info_size < sizeof(struct _ccci_tag_v2) ||
	    hdr->lk_info_size > CCCI_TAG_MEM_SIZE)
		return -EMSGSIZE;
	/* v2 and v3 use the same tags; unknown revisions need an ABI review. */
	if (hdr->lk_info_version != 2 && hdr->lk_info_version != 3)
		return -EOPNOTSUPP;
	if (hdr->lk_info_tag_num < 0)
		return -EINVAL;
	if (!hdr->lk_info_tag_num)
		return -ENODATA;
	if (hdr->lk_info_tag_num > CCCI_PROBE_MAX_TAGS)
		return -E2BIG;
	if ((unsigned int)hdr->lk_info_tag_num >
	    hdr->lk_info_size / sizeof(struct _ccci_tag_v2))
		return -EMSGSIZE;
	if (hdr->lk_info_err_no || hdr->lk_info_ld_md_errno[0])
		return -EIO;
	return 0;
}

static inline int ccci_probe_decode_header(const void *raw, int len,
					   struct _ccci_lk_info_v2 *hdr)
{
	if (!hdr)
		return -EINVAL;
	memset(hdr, 0, sizeof(*hdr));
	if (!raw)
		return -ENODEV;
	if (len < (int)sizeof(*hdr))
		return -EMSGSIZE;
	memcpy(hdr, raw, sizeof(*hdr));
	return ccci_probe_validate_header(hdr);
}

static inline int ccci_probe_parse_smem(const void *payload,
					struct _smem_layout *smem)
{
	memcpy(smem, payload, sizeof(*smem));
	if (!smem->total_smem_size || !smem->ap_md1_smem_size)
		return -ENODATA;
	if (!ccci_probe_address_valid(smem->base_addr, smem->total_smem_size) ||
	    !ccci_probe_range_valid(smem->ap_md1_smem_offset,
				    smem->ap_md1_smem_size, smem->total_smem_size) ||
	    !ccci_probe_range_valid(smem->ap_md3_smem_offset,
				    smem->ap_md3_smem_size, smem->total_smem_size) ||
	    !ccci_probe_range_valid(smem->md1_md3_smem_offset,
				    smem->md1_md3_smem_size, smem->total_smem_size))
		return -ERANGE;
	return 0;
}

typedef void (*ccci_probe_trace_tag)(unsigned int index, unsigned int offset,
				   const struct _ccci_tag_v2 *tag);

/*
 * No mapping or address chasing here. All reads stay within the supplied
 * buffer and the declared blob size. trace only receives copied metadata.
 * Partial results on error are diagnostic, never a usable modem layout.
 */
static inline int
ccci_probe_parse_tags(const struct _ccci_lk_info_v2 *hdr, const void *buffer,
		     size_t buffer_size, struct ccci_probe_result *out,
		     ccci_probe_trace_tag trace)
{
	const char *blob = buffer;
	unsigned int offset = 0, i;
	int ret;

	if (!out)
		return -EINVAL;
	memset(out, 0, sizeof(*out));
	ret = ccci_probe_validate_header(hdr);
	if (ret)
		return ret;
	if (!blob || buffer_size < hdr->lk_info_size)
		return -EMSGSIZE;

	for (i = 0; i < (unsigned int)hdr->lk_info_tag_num; i++) {
		struct _ccci_tag_v2 tag;
		const void *payload;
		unsigned int next;

		out->tag_offset = offset;
		if (!ccci_probe_range_valid(offset, sizeof(tag), hdr->lk_info_size))
			return -EMSGSIZE;
		memcpy(&tag, blob + offset, sizeof(tag));
		if (!memchr(tag.tag_name, '\0', sizeof(tag.tag_name)))
			return -EBADMSG;
		if (!ccci_probe_range_valid(tag.data_offset, tag.data_size,
					    hdr->lk_info_size))
			return -ERANGE;
		if (trace)
			trace(i, offset, &tag);
		payload = blob + tag.data_offset;

		if (!strcmp(tag.tag_name, "smem_layout")) {
			if (out->smem_found)
				return -EEXIST;
			if (tag.data_size < sizeof(out->smem))
				return -EMSGSIZE;
			ret = ccci_probe_parse_smem(payload, &out->smem);
			if (ret)
				return ret;
			out->smem_found = true;
		} else if (!strcmp(tag.tag_name, "ccb_info")) {
			if (out->ccb_found)
				return -EEXIST;
			if (tag.data_size < sizeof(out->ccb))
				return -EMSGSIZE;
			memcpy(&out->ccb, payload, sizeof(out->ccb));
			if (!ccci_probe_address_valid(out->ccb.ccb_data_buffer_addr,
						      out->ccb.ccb_data_buffer_size))
				return -ERANGE;
			out->ccb_found = true;
		} else if (!strcmp(tag.tag_name, "md1_sib_info")) {
			if (out->sib_found)
				return -EEXIST;
			if (tag.data_size < sizeof(out->sib))
				return -EMSGSIZE;
			memcpy(&out->sib, payload, sizeof(out->sib));
			if (!ccci_probe_address_valid(out->sib.md1_sib_addr,
						      out->sib.md1_sib_size))
				return -ERANGE;
			out->sib_found = true;
		}
		out->tags_walked++;
		next = tag.next_tag_offset;

		/* LK walks by count. The last link may be zero or a forward end. */
		if (i + 1 == (unsigned int)hdr->lk_info_tag_num && !next)
			break;
		if (next <= offset || next - offset < sizeof(tag))
			return -EBADMSG;
		if (next > hdr->lk_info_size)
			return -ERANGE;
		offset = next;
	}

	return out->smem_found ? 0 : -ENODATA;
}

#endif /* _CCCI_PROBE_PARSER_H */
