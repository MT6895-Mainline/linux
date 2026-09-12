// SPDX-License-Identifier: GPL-2.0
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../../drivers/misc/mediatek/ccci_probe/ccci_probe_parser.h"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

enum {
	TAG_BYTES = sizeof(struct _ccci_tag_v2),
	PAYLOAD_OFFSET = 3 * TAG_BYTES,
	BLOB_BYTES = PAYLOAD_OFFSET + sizeof(struct _smem_layout) +
		     sizeof(struct _ccb_layout) + sizeof(struct _sib_item),
	MUTATION_RUNS = 50000,
};

struct fixture {
	struct _ccci_lk_info_v2 hdr;
	unsigned char blob[BLOB_BYTES];
};

static unsigned int checks, failures, traced;

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

static struct _ccci_tag_v2 get_tag(const struct fixture *f, unsigned int i)
{
	struct _ccci_tag_v2 tag;

	memcpy(&tag, f->blob + i * TAG_BYTES, sizeof(tag));
	return tag;
}

static void put_tag(struct fixture *f, unsigned int i,
		    const struct _ccci_tag_v2 *tag)
{
	memcpy(f->blob + i * TAG_BYTES, tag, sizeof(*tag));
}

static struct _smem_layout valid_smem(void)
{
	return (struct _smem_layout) {
		.base_addr = 0x88000000ULL,
		.ap_md1_smem_offset = 0,
		.ap_md1_smem_size = 0x1000,
		.total_smem_size = 0x1000,
	};
}

static struct fixture valid_fixture(void)
{
	struct fixture f = {
		.hdr = {
			.lk_info_base_addr = CCCI_TAG_MEM_BASE,
			.lk_info_size = BLOB_BYTES,
			.lk_info_version = 3,
			.lk_info_tag_num = 3,
			.lk_info_ld_flag = 1,
		},
	};
	const struct _smem_layout smem = valid_smem();
	const struct _ccb_layout ccb = {
		.ccb_data_buffer_addr = 0x88001000ULL,
		.ccb_data_buffer_size = 0x1000,
	};
	const struct _sib_item sib = {
		.md1_sib_addr = 0x88002000ULL,
		.md1_sib_size = 0x1000,
	};
	const struct _ccci_tag_v2 tags[] = {
		{
			.tag_name = "smem_layout",
			.data_offset = PAYLOAD_OFFSET,
			.data_size = sizeof(smem),
			.next_tag_offset = TAG_BYTES,
		},
		{
			.tag_name = "ccb_info",
			.data_offset = PAYLOAD_OFFSET + sizeof(smem),
			.data_size = sizeof(ccb),
			.next_tag_offset = 2 * TAG_BYTES,
		},
		{
			.tag_name = "md1_sib_info",
			.data_offset = PAYLOAD_OFFSET + sizeof(smem) + sizeof(ccb),
			.data_size = sizeof(sib),
		},
	};

	memcpy(f.blob, tags, sizeof(tags));
	memcpy(f.blob + tags[0].data_offset, &smem, sizeof(smem));
	memcpy(f.blob + tags[1].data_offset, &ccb, sizeof(ccb));
	memcpy(f.blob + tags[2].data_offset, &sib, sizeof(sib));
	return f;
}

static int parse(const struct fixture *f, struct ccci_probe_result *out)
{
	return ccci_probe_parse_tags(&f->hdr, f->blob, sizeof(f->blob), out, NULL);
}

static void trace_tag(unsigned int index, unsigned int offset,
		      const struct _ccci_tag_v2 *tag)
{
	(void)index;
	(void)offset;
	if (memchr(tag->tag_name, '\0', sizeof(tag->tag_name)))
		traced++;
}

#define BAD_HEADER(label, field, value, error) do {			\
	struct _ccci_lk_info_v2 bad = f.hdr;				\
	bad.field = (value);						\
	expect(label, ccci_probe_decode_header(&bad, sizeof(bad), &h),	\
	       error);							\
} while (0)

static void test_headers(void)
{
	struct fixture f = valid_fixture();
	struct _ccci_lk_info_v2 h;
	unsigned char unaligned[sizeof(h) + 1];
	bool short_ok = true;
	size_t i;

	expect("valid v3 header",
	       ccci_probe_decode_header(&f.hdr, sizeof(f.hdr), &h), 0);
	expect("missing header", ccci_probe_decode_header(NULL, 0, &h), -ENODEV);
	expect("missing output",
	       ccci_probe_decode_header(&f.hdr, sizeof(f.hdr), NULL), -EINVAL);
	expect("negative property length",
	       ccci_probe_decode_header(&f.hdr, -1, &h), -EMSGSIZE);
	for (i = 1; i < sizeof(h); i++) {
		unsigned char *short_header = malloc(i);

		if (!short_header)
			exit(2);
		memset(short_header, 0, i);
		if (ccci_probe_decode_header(short_header, i, &h) != -EMSGSIZE)
			short_ok = false;
		free(short_header);
	}
	expect("every short header rejected without overread", short_ok, true);
	memcpy(unaligned + 1, &f.hdr, sizeof(f.hdr));
	expect("unaligned property",
	       ccci_probe_decode_header(unaligned + 1, sizeof(h), &h), 0);
	BAD_HEADER("valid v2 header", lk_info_version, 2, 0);
	BAD_HEADER("negative version", lk_info_version, -1, -EOPNOTSUPP);
	BAD_HEADER("v1 format", lk_info_version, 1, -EOPNOTSUPP);
	BAD_HEADER("unknown version", lk_info_version, 4, -EOPNOTSUPP);
	BAD_HEADER("huge version", lk_info_version, INT_MAX, -EOPNOTSUPP);
	BAD_HEADER("wrong physical region", lk_info_base_addr, 0x88000000, -ERANGE);
	BAD_HEADER("missing physical region", lk_info_base_addr, 0, -ERANGE);
	BAD_HEADER("zero blob size", lk_info_size, 0, -EMSGSIZE);
	BAD_HEADER("short first tag", lk_info_size, TAG_BYTES - 1, -EMSGSIZE);
	BAD_HEADER("oversized blob", lk_info_size, CCCI_TAG_MEM_SIZE + 1, -EMSGSIZE);
	BAD_HEADER("negative count", lk_info_tag_num, -1, -EINVAL);
	BAD_HEADER("zero count", lk_info_tag_num, 0, -ENODATA);
	BAD_HEADER("count is not silently clamped", lk_info_tag_num,
		   CCCI_PROBE_MAX_TAGS + 1, -E2BIG);
	BAD_HEADER("huge count", lk_info_tag_num, INT_MAX, -E2BIG);
	BAD_HEADER("count cannot fit in blob", lk_info_tag_num, 4, -EMSGSIZE);
	BAD_HEADER("LK global error", lk_info_err_no, -1, -EIO);
	BAD_HEADER("LK MD1 error", lk_info_ld_md_errno[0], -1, -EIO);
}

static void test_tags(void)
{
	struct fixture f = valid_fixture();
	struct ccci_probe_result out;
	struct _ccci_tag_v2 tag;
	struct _smem_layout smem;
	struct _ccb_layout ccb;
	struct _sib_item sib;
	unsigned char unaligned[BLOB_BYTES + 1];

	expect("complete three-tag chain", parse(&f, &out), 0);
	expect("exact tag count", out.tags_walked, 3);
	expect("SMEM decoded", out.smem.base_addr, 0x88000000);
	expect("CCB decoded", out.ccb_found, true);
	expect("SIB decoded", out.sib_found, true);
	traced = 0;
	expect("tag observer", ccci_probe_parse_tags(&f.hdr, f.blob,
	       sizeof(f.blob), &out, trace_tag), 0);
	expect("observer sees all copied names", traced, 3);
	memcpy(unaligned + 1, f.blob, sizeof(f.blob));
	expect("unaligned tag buffer", ccci_probe_parse_tags(&f.hdr,
	       unaligned + 1, BLOB_BYTES, &out, NULL), 0);
	expect("missing blob", ccci_probe_parse_tags(&f.hdr, NULL,
	       BLOB_BYTES, &out, NULL), -EMSGSIZE);
	expect("short supplied buffer", ccci_probe_parse_tags(&f.hdr, f.blob,
	       BLOB_BYTES - 1, &out, NULL), -EMSGSIZE);
	expect("null result", ccci_probe_parse_tags(&f.hdr, f.blob,
	       BLOB_BYTES, NULL, NULL), -EINVAL);
	expect("null header", ccci_probe_parse_tags(NULL, f.blob,
	       BLOB_BYTES, &out, NULL), -EINVAL);

	tag = get_tag(&f, 0);
	tag.data_size = BLOB_BYTES;
	put_tag(&f, 0, &tag);
	expect("valid prefix but oversized declared payload", parse(&f, &out), -ERANGE);
	tag.data_size = UINT_MAX;
	put_tag(&f, 0, &tag);
	expect("payload size overflow", parse(&f, &out), -ERANGE);
	tag.data_size = sizeof(smem);
	tag.data_offset = UINT_MAX;
	put_tag(&f, 0, &tag);
	expect("payload offset overflow", parse(&f, &out), -ERANGE);
	tag.data_offset = PAYLOAD_OFFSET;
	tag.data_size = sizeof(smem) - 1;
	put_tag(&f, 0, &tag);
	expect("short known payload", parse(&f, &out), -EMSGSIZE);

	f = valid_fixture();
	tag = get_tag(&f, 0);
	tag.next_tag_offset = 0;
	put_tag(&f, 0, &tag);
	expect("early termination is not success", parse(&f, &out), -EBADMSG);
	tag.next_tag_offset = 4;
	put_tag(&f, 0, &tag);
	expect("overlapping next descriptor", parse(&f, &out), -EBADMSG);
	tag.next_tag_offset = BLOB_BYTES + 1;
	put_tag(&f, 0, &tag);
	expect("next link beyond declared buffer", parse(&f, &out), -ERANGE);
	tag.next_tag_offset = BLOB_BYTES;
	put_tag(&f, 0, &tag);
	expect("truncated next descriptor", parse(&f, &out), -EMSGSIZE);
	expect("partial parsed layout is not a successful result", out.tags_walked, 1);

	f = valid_fixture();
	tag = get_tag(&f, 1);
	tag.next_tag_offset = TAG_BYTES;
	put_tag(&f, 1, &tag);
	expect("self loop", parse(&f, &out), -EBADMSG);
	tag.next_tag_offset = 1;
	put_tag(&f, 1, &tag);
	expect("backward link", parse(&f, &out), -EBADMSG);
	f = valid_fixture();
	tag = get_tag(&f, 2);
	tag.next_tag_offset = TAG_BYTES;
	put_tag(&f, 2, &tag);
	expect("terminal backward link", parse(&f, &out), -EBADMSG);
	tag.next_tag_offset = BLOB_BYTES;
	put_tag(&f, 2, &tag);
	expect("count-based forward terminal", parse(&f, &out), 0);

	f = valid_fixture();
	tag = get_tag(&f, 2);
	memset(tag.tag_name, 'x', sizeof(tag.tag_name));
	put_tag(&f, 2, &tag);
	expect("unterminated name", parse(&f, &out), -EBADMSG);
	strcpy(tag.tag_name, "vendor_optional");
	tag.data_size = 0;
	tag.data_offset = BLOB_BYTES;
	put_tag(&f, 2, &tag);
	expect("unknown empty tag at buffer end", parse(&f, &out), 0);
	tag.data_size = 1;
	put_tag(&f, 2, &tag);
	expect("unknown payload still bounds checked", parse(&f, &out), -ERANGE);

	f = valid_fixture();
	tag = get_tag(&f, 0);
	strcpy(tag.tag_name, "vendor_optional");
	put_tag(&f, 0, &tag);
	expect("complete chain without usable SMEM", parse(&f, &out), -ENODATA);
	expect("no-layout error still walks all tags", out.tags_walked, 3);
	f = valid_fixture();
	tag = get_tag(&f, 0);
	tag.next_tag_offset = 2 * TAG_BYTES;
	put_tag(&f, 1, &tag);
	expect("ambiguous duplicate SMEM", parse(&f, &out), -EEXIST);

	f = valid_fixture();
	smem = valid_smem();
	smem.ap_md1_smem_size = 0;
	memcpy(f.blob + PAYLOAD_OFFSET, &smem, sizeof(smem));
	expect("SMEM without MD1 data", parse(&f, &out), -ENODATA);
	smem = valid_smem();
	smem.ap_md1_smem_offset = UINT_MAX;
	memcpy(f.blob + PAYLOAD_OFFSET, &smem, sizeof(smem));
	expect("SMEM offset outside total", parse(&f, &out), -ERANGE);
	smem = valid_smem();
	smem.base_addr = ~0ULL - 8;
	memcpy(f.blob + PAYLOAD_OFFSET, &smem, sizeof(smem));
	expect("SMEM physical address overflow", parse(&f, &out), -ERANGE);
	smem = valid_smem();
	smem.ap_md3_smem_size = UINT_MAX;
	memcpy(f.blob + PAYLOAD_OFFSET, &smem, sizeof(smem));
	expect("SMEM MD3 bounds", parse(&f, &out), -ERANGE);
	smem = valid_smem();
	smem.md1_md3_smem_offset = UINT_MAX;
	memcpy(f.blob + PAYLOAD_OFFSET, &smem, sizeof(smem));
	expect("SMEM MD1-MD3 bounds", parse(&f, &out), -ERANGE);

	f = valid_fixture();
	tag = get_tag(&f, 1);
	tag.data_size--;
	put_tag(&f, 1, &tag);
	expect("short CCB payload", parse(&f, &out), -EMSGSIZE);
	f = valid_fixture();
	tag = get_tag(&f, 2);
	tag.data_size--;
	put_tag(&f, 2, &tag);
	expect("short SIB payload", parse(&f, &out), -EMSGSIZE);
	f = valid_fixture();
	tag = get_tag(&f, 1);
	tag.next_tag_offset = 0;
	put_tag(&f, 2, &tag);
	expect("duplicate CCB", parse(&f, &out), -EEXIST);
	f = valid_fixture();
	tag = get_tag(&f, 2);
	tag.next_tag_offset = 2 * TAG_BYTES;
	put_tag(&f, 1, &tag);
	expect("duplicate SIB", parse(&f, &out), -EEXIST);

	f = valid_fixture();
	tag = get_tag(&f, 1);
	ccb = (struct _ccb_layout) {
		.ccb_data_buffer_addr = ~0ULL - 8,
		.ccb_data_buffer_size = 16,
	};
	memcpy(f.blob + tag.data_offset, &ccb, sizeof(ccb));
	expect("CCB address overflow", parse(&f, &out), -ERANGE);
	f = valid_fixture();
	tag = get_tag(&f, 2);
	sib = (struct _sib_item) {
		.md1_sib_addr = 0,
		.md1_sib_size = 16,
	};
	memcpy(f.blob + tag.data_offset, &sib, sizeof(sib));
	expect("SIB missing address", parse(&f, &out), -ERANGE);
}

static void test_counts(void)
{
	const unsigned int counts[] = { 1, 27, CCCI_PROBE_MAX_TAGS };
	struct _smem_layout smem = valid_smem();
	unsigned int c, i;

	for (c = 0; c < ARRAY_SIZE(counts); c++) {
		unsigned int count = counts[c];
		unsigned int payload = count * TAG_BYTES;
		unsigned int size = payload + sizeof(smem);
		unsigned char *buffer = calloc(1, size);
		struct _ccci_lk_info_v2 hdr = {
			.lk_info_base_addr = CCCI_TAG_MEM_BASE,
			.lk_info_size = size,
			.lk_info_version = 3,
			.lk_info_tag_num = count,
		};
		struct ccci_probe_result out;
		char label[80];

		if (!buffer)
			exit(2);
		for (i = 0; i < count; i++) {
			struct _ccci_tag_v2 tag = {
				.data_offset = payload,
				.data_size = i ? 0 : sizeof(smem),
				.next_tag_offset = i + 1 == count ? 0 : (i + 1) * TAG_BYTES,
			};

			strcpy(tag.tag_name, i ? "vendor_optional" : "smem_layout");
			memcpy(buffer + i * TAG_BYTES, &tag, sizeof(tag));
		}
		memcpy(buffer + payload, &smem, sizeof(smem));
		snprintf(label, sizeof(label), "valid %u-tag chain", count);
		expect(label, ccci_probe_parse_tags(&hdr, buffer, size, &out, NULL), 0);
		snprintf(label, sizeof(label), "all %u tags visited", count);
		expect(label, out.tags_walked, count);
		free(buffer);
	}
}

static void test_gate(void)
{
	enum ccci_probe_state state = CCCI_PROBE_UNAVAILABLE;
	enum ccci_probe_state used[] = {
		CCCI_PROBE_QUEUED, CCCI_PROBE_RUNNING,
		CCCI_PROBE_DONE, CCCI_PROBE_FAILED,
	};
	unsigned int i;

	expect("load-time zero is inert", ccci_probe_request_read(&state, false, false), 0);
	expect("zero leaves unavailable unchanged", state, CCCI_PROBE_UNAVAILABLE);
	expect("load-time trigger rejected",
	       ccci_probe_request_read(&state, false, true), -EAGAIN);
	expect("load-time state unchanged", state, CCCI_PROBE_UNAVAILABLE);
	expect("invalid header cannot arm",
	       ccci_probe_request_read(&state, true, true), -ENODATA);
	state = CCCI_PROBE_READY;
	expect("COMING even after init cannot arm",
	       ccci_probe_request_read(&state, false, true), -EAGAIN);
	expect("COMING does not consume attempt", state, CCCI_PROBE_READY);
	expect("live zero is inert", ccci_probe_request_read(&state, true, false), 0);
	expect("zero leaves ready unchanged", state, CCCI_PROBE_READY);
	expect("live explicit trigger accepted", ccci_probe_request_read(&state, true, true), 1);
	expect("accepted read becomes queued", state, CCCI_PROBE_QUEUED);
	for (i = 0; i < ARRAY_SIZE(used); i++) {
		state = used[i];
		expect("one-shot includes running, success and failure",
		       ccci_probe_request_read(&state, true, true), -EALREADY);
		expect("repeat request preserves state", state, used[i]);
		expect("zero is not cancellation or rearming",
		       ccci_probe_request_read(&state, true, false), 0);
		expect("zero leaves used state unchanged", state, used[i]);
	}
	state = CCCI_PROBE_READY;
	expect("GOING cannot arm", ccci_probe_request_read(&state, false, true), -EAGAIN);
}

static uint32_t next_random(uint32_t *state)
{
	*state ^= *state << 13;
	*state ^= *state >> 17;
	*state ^= *state << 5;
	return *state;
}

static void test_mutations(void)
{
	struct fixture seed = valid_fixture();
	uint32_t random = 0x6895cc01U;
	unsigned int i, bad_successes = 0;

	for (i = 0; i < MUTATION_RUNS; i++) {
		struct _ccci_lk_info_v2 hdr = seed.hdr;
		struct ccci_probe_result out;
		unsigned char *blob = malloc(BLOB_BYTES);
		unsigned int j, edits = 1 + next_random(&random) % 8;
		int ret;

		if (!blob)
			exit(2);
		memcpy(blob, seed.blob, BLOB_BYTES);
		for (j = 0; j < edits; j++) {
			unsigned int offset = next_random(&random) % BLOB_BYTES;

			blob[offset] = next_random(&random) & 0xff;
		}
		/* Include truncated supplied buffers and corrupt signed header fields. */
		if (i % 4 == 0)
			hdr.lk_info_tag_num = (int)next_random(&random);
		if (i % 7 == 0)
			hdr.lk_info_size = next_random(&random);
		ret = ccci_probe_parse_tags(&hdr, blob,
					   i % 3 ? BLOB_BYTES : i % BLOB_BYTES,
					   &out, NULL);
		if (!ret && (!out.smem_found ||
			     out.tags_walked != (unsigned int)hdr.lk_info_tag_num))
			bad_successes++;
		free(blob);
	}
	expect("50000 deterministic mutations preserve success invariants",
	       bad_successes, 0);
}

int main(void)
{
	puts("TAP version 13");
	test_headers();
	test_tags();
	test_counts();
	test_gate();
	test_mutations();
	printf("1..%u\n", checks);
	return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
