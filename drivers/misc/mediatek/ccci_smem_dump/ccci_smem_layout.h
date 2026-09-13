/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CCCI non-cacheable SMEM layout for MT6895 (official md_gen >= 6297 path).
 *
 * Region table ported from the official OnePlus 5.10 mt6895 tree,
 * drivers/misc/mediatek/eccci/ccci_modem.c md1_6297_noncacheable_fat, with
 * the sequential-offset fill of ccci_6297_md_smem_layout_config() and the LK
 * override semantics of update_smem_region()/get_nc_smem_region_info() (first
 * matching id wins, applied before each entry's offset fixup).
 *
 * Pure computation, shared by the kernel module and the host tests: no kernel
 * headers, only fundamental types.
 */
#ifndef _CCCI_SMEM_LAYOUT_H
#define _CCCI_SMEM_LAYOUT_H

/*
 * enum SMEM_USER_ID values, verbatim from
 * drivers/misc/mediatek/ccci_util/mtk_ccci_common.h (SMEM_USER_RAW_DBM = 0).
 * The LK nc_smem tags reference these ids.
 */
#define SMEM_USER_RAW_DBM		0u
#define SMEM_USER_CCB_START		1u
#define SMEM_USER_CCB_DHL		1u
#define SMEM_USER_CCB_MD_MONITOR	2u
#define SMEM_USER_CCB_META		3u
#define SMEM_USER_RAW_CCB_CTRL		4u
#define SMEM_USER_RAW_DHL		5u
#define SMEM_USER_RAW_MDM		6u
#define SMEM_USER_RAW_NETD		7u
#define SMEM_USER_RAW_USB		8u
#define SMEM_USER_RAW_AUDIO		9u
#define SMEM_USER_RAW_DFD		10u
#define SMEM_USER_RAW_LWA		11u
#define SMEM_USER_RAW_MDCCCI_DBG	12u
#define SMEM_USER_RAW_MDSS_DBG		13u
#define SMEM_USER_RAW_RUNTIME_DATA	14u
#define SMEM_USER_RAW_FORCE_ASSERT	15u
#define SMEM_USER_CCISM_SCP		16u
#define SMEM_USER_RAW_MD2MD		17u
#define SMEM_USER_RAW_RESERVED		18u
#define SMEM_USER_CCISM_MCU		19u
#define SMEM_USER_CCISM_MCU_EXP		20u
#define SMEM_USER_SMART_LOGGING		21u
#define SMEM_USER_RAW_MD_CONSYS		22u
#define SMEM_USER_RAW_PHY_CAP		23u
#define SMEM_USER_RAW_USIP		24u
#define SMEM_USER_RESV_0		25u
#define SMEM_USER_ALIGN_PADDING		26u
#define SMEM_USER_RAW_UDC_DATA		27u
#define SMEM_USER_RAW_UDC_DESCTAB	28u
#define SMEM_USER_RAW_AMMS_POS		29u
#define SMEM_USER_RAW_ALIGN_PADDING	30u
#define SMEM_USER_MD_WIFI_PROXY		31u
#define SMEM_USER_MD_NVRAM_CACHE	32u
#define SMEM_USER_LOW_POWER		33u
#define SMEM_USER_SECURITY_SMEM		34u
#define SMEM_USER_RESERVED		40u
#define SMEM_USER_MD_DRDI		41u
#define SMEM_USER_MD_DATA		42u

/* CCCI_SMEM_SIZE_DBM/GUARD (ccci_common_config.h) and BANK4_DRDI_SMEM_SIZE
 * (ccci_config.h) from the official tree. */
#define CCCI_SMEM_DBM_S		(160u + 8u * 2u)
#define BANK4_DRDI_SMEM_SIZE	(64u * 1024u)

struct ccci_smem_region_tbl {
	unsigned int id;
	unsigned int offset;	/* in bank4 (the non-cacheable SMEM region) */
	unsigned int size;
	unsigned int flags;	/* SMF_* in the official driver; unused here */
	const char *name;
};

/* One entry of the LK "nc_smem_info_ext" / "nc_smem_layout" tags (16 bytes). */
struct ccci_smem_override {
	unsigned int ap_offset;
	unsigned int md_offset;
	unsigned int size;
	unsigned int id;
};

/* One entry of the LK "md1_bank4_cache_info"/"_layout" tags (24 bytes). */
struct ccci_smem_csmem_item {
	unsigned long long addr;	/* AP-view physical address */
	unsigned int md_offset;
	unsigned int size;
	unsigned int item_cnt;
};

/* Default table, md1_6297_noncacheable_fat in official ccci_modem.c. */
static const struct ccci_smem_region_tbl ccci_smem_fat_default[] = {
	{ SMEM_USER_RAW_DFD,		0, 0,		0, "RAW_DFD" },
	{ SMEM_USER_RAW_UDC_DATA,	0, 0,		0, "RAW_UDC_DATA" },
	{ SMEM_USER_MD_WIFI_PROXY,	0, 0,		0, "MD_WIFI_PROXY" },
	{ SMEM_USER_SECURITY_SMEM,	0, 0,		0, "SECURITY_SMEM" },
	{ SMEM_USER_RAW_AMMS_POS,	0, 0,		0, "RAW_AMMS_POS" },
	{ SMEM_USER_RAW_MDCCCI_DBG,	0, 2 * 1024,	0, "RAW_MDCCCI_DBG" },
	{ SMEM_USER_RAW_MDSS_DBG,	0, 14 * 1024,	0, "RAW_MDSS_DBG" },
	{ SMEM_USER_RAW_RESERVED,	0, 42 * 1024,	0, "RAW_RESERVED" },
	{ SMEM_USER_RAW_RUNTIME_DATA,	0, 4 * 1024,	0, "RAW_RUNTIME_DATA" },
	{ SMEM_USER_RAW_FORCE_ASSERT,	0, 1 * 1024,	0, "RAW_FORCE_ASSERT" },
	{ SMEM_USER_LOW_POWER,		0, 512,		0, "LOW_POWER" },
	{ SMEM_USER_RAW_DBM,		0, 512,		0, "RAW_DBM" },
	{ SMEM_USER_CCISM_SCP,		0, 32 * 1024,	0, "CCISM_SCP" },
	{ SMEM_USER_RAW_CCB_CTRL,	0, 4 * 1024,	0, "RAW_CCB_CTRL" },
	{ SMEM_USER_RAW_NETD,		0, 8 * 1024,	0, "RAW_NETD" },
	{ SMEM_USER_RAW_USB,		0, 4 * 1024,	0, "RAW_USB" },
	{ SMEM_USER_RAW_AUDIO,		0, 52 * 1024,	0, "RAW_AUDIO" },
	{ SMEM_USER_CCISM_MCU,		0, (720u + 1u) * 1024u, 0, "CCISM_MCU" },
	{ SMEM_USER_CCISM_MCU_EXP,	0, (120u + 1u) * 1024u, 0, "CCISM_MCU_EXP" },
	{ SMEM_USER_RESERVED,		0, 18 * 1024,	0, "RESERVED" },
	{ SMEM_USER_MD_DRDI,		0, BANK4_DRDI_SMEM_SIZE, 0, "MD_DRDI" },
	{ SMEM_USER_MD_DATA,		0, 0,		0, "MD_DATA" },
};

#define CCCI_SMEM_FAT_NUM \
	(sizeof(ccci_smem_fat_default) / sizeof(ccci_smem_fat_default[0]))

/*
 * Build the region table: apply the first matching LK override to entry i,
 * then fix up a zero offset from the previous entry (official order: the
 * override of entry i-1 is already in place when entry i is fixed up).
 */
static void ccci_smem_layout_build(struct ccci_smem_region_tbl *tbl,
				   unsigned int n,
				   const struct ccci_smem_override *ov,
				   unsigned int ov_num)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		unsigned int j;

		for (j = 0; j < ov_num; j++) {
			if (ov[j].id == tbl[i].id) {
				tbl[i].offset = ov[j].ap_offset;
				tbl[i].size = ov[j].size;
				break;
			}
		}
		if (i == 0)
			continue;
		if (tbl[i].offset == 0)
			tbl[i].offset = tbl[i - 1].offset + tbl[i - 1].size;
	}
}

/* 0 if every non-empty region fits in [0, total); -1 otherwise. */
static int ccci_smem_layout_check(const struct ccci_smem_region_tbl *tbl,
				  unsigned int n, unsigned int total)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (tbl[i].size == 0)
			continue;
		if (tbl[i].offset >= total ||
		    tbl[i].size > total - tbl[i].offset)
			return -1;
	}
	return 0;
}

/* Index of id in the table, or -1. */
static int ccci_smem_layout_find(const struct ccci_smem_region_tbl *tbl,
				 unsigned int n, unsigned int id)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (tbl[i].id == id)
			return (int)i;
	}
	return -1;
}

/* End of the table (last offset + size), the used SMEM span. */
static inline unsigned int ccci_smem_layout_end(const struct ccci_smem_region_tbl *tbl,
					 unsigned int n)
{
	return tbl[n - 1].offset + tbl[n - 1].size;
}

#endif /* _CCCI_SMEM_LAYOUT_H */
