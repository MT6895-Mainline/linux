/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2015 MediaTek Inc.
 */
#ifndef __CCCI_UTIL_LIB_MAIN_H__
#define __CCCI_UTIL_LIB_MAIN_H__
#include <linux/types.h>
/* qqcandy: local copy of mt-plat/mtk_ccci_common.h (avoids shadowing the
 * connectivity/compat header of the same basename) */
#include "mtk_ccci_common.h"

struct md_check_header_v3 {
	/* magic number is "CHECK_HEADER"*/
	unsigned char check_header[12];
	/* header structure version number */
	unsigned int  header_verno;
	/* 0x0:invalid;
	 * 0x1:debug version;
	 * 0x2:release version
	 */
	unsigned int  product_ver;
	/* 0x0:invalid;
	 * 0x1:2G modem;
	 * 0x2: 3G modem
	 */
	unsigned int  image_type;
	/* MT6573_S01 or MT6573_S02 */
	unsigned char platform[16];
	/* build time string */
	unsigned char build_time[64];
	/* project version, ex:11A_MD.W11.28 */
	unsigned char build_ver[64];

	/* bind to md sys id,
	 * MD SYS1: 1,
	 * MD SYS2: 2,
	 * MD SYS5: 5
	 */
	unsigned char bind_sys_id;
	/* no shrink: 0, shrink: 1 */
	unsigned char ext_attr;
	/* for reserved */
	unsigned char reserved[2];

	/* md ROM/RAM image size requested by md */
	unsigned int  mem_size;
	/* md image size, exclude head size */
	unsigned int  md_img_size;
	/* RPC secure memory address */
	unsigned int  rpc_sec_mem_addr;

	unsigned int  dsp_img_offset;
	unsigned int  dsp_img_size;
	unsigned char reserved2[88];
	/* the size of this structure */
	unsigned int  size;
} __packed;

struct md_check_header_v4 {
	/* magic number is "CHECK_HEADER"*/
	unsigned char check_header[12];
	/* header structure version number */
	unsigned int header_verno;
	/* 0x0:invalid;
	 * 0x1:debug version;
	 * 0x2:release version
	 */
	unsigned int product_ver;
	/* 0x0:invalid;
	 * 0x1:2G modem;
	 * 0x2: 3G modem
	 */
	unsigned int image_type;
	/* MT6573_S01 or MT6573_S02 */
	unsigned char platform[16];
	/* build time string */
	unsigned char build_time[64];
	/* project version, ex:11A_MD.W11.28 */
	unsigned char build_ver[64];

	/* bind to md sys id,
	 * MD SYS1: 1,
	 * MD SYS2: 2,
	 * MD SYS5: 5
	 */
	unsigned char bind_sys_id;
	/* no shrink: 0, shrink: 1 */
	unsigned char ext_attr;
	/* for reserved */
	unsigned char reserved[2];

	/* md ROM/RAM image size requested by md */
	unsigned int mem_size;
	/* md image size, exclude head size */
	unsigned int md_img_size;
	/* RPC secure memory address */
	unsigned int rpc_sec_mem_addr;

	unsigned int dsp_img_offset;
	unsigned int dsp_img_size;

	/* total region number */
	unsigned int region_num;
	/* max support 8 regions */
	struct _md_regin_info region_info[8];
	/* max support 4 domain settings,
	 * each region has 4 control bits
	 */
	unsigned int domain_attr[4];

	unsigned char reserved2[4];
	/* the size of this structure */
	unsigned int size;
} __packed;

struct md_check_header_v5 {
	/* magic number is "CHECK_HEADER" */
	unsigned char check_header[12];
	/* header structure version number */
	unsigned int  header_verno;
	/* 0x0:invalid;
	 * 0x1:debug version;
	 * 0x2:release version
	 */
	unsigned int  product_ver;
	/* 0x0:invalid;
	 * 0x1:2G modem;
	 * 0x2: 3G modem
	 */
	unsigned int  image_type;
	/* MT6573_S01 or MT6573_S02 */
	unsigned char platform[16];
	/* build time string */
	unsigned char build_time[64];
	/* project version, ex:11A_MD.W11.28 */
	unsigned char build_ver[64];

	/* bind to md sys id,
	 * MD SYS1: 1,
	 * MD SYS2: 2,
	 * MD SYS5: 5
	 */
	unsigned char bind_sys_id;
	/* no shrink: 0, shrink: 1 */
	unsigned char ext_attr;
	/* for reserved */
	unsigned char reserved[2];

	/* md ROM/RAM image size requested by md */
	unsigned int  mem_size;
	/* md image size, exclude head size */
	unsigned int  md_img_size;
	/* RPC secure memory address */
	unsigned int  rpc_sec_mem_addr;

	unsigned int  dsp_img_offset;
	unsigned int  dsp_img_size;

	/* total region number */
	unsigned int  region_num;
	/* max support 8 regions */
	struct _md_regin_info region_info[8];
	/* max support 4 domain settings,
	 * each region has 4 control bits
	 */
	unsigned int  domain_attr[4];

	unsigned int  arm7_img_offset;
	unsigned int  arm7_img_size;

	unsigned char reserved_1[56];

	/* the size of this structure */
	unsigned int  size;
} __packed;

struct md_check_header_struct {
	/* magic number is "CHECK_HEADER"*/
	unsigned char check_header[12];
	/* header structure version number */
	unsigned int  header_verno;
	/* 0x0:invalid;
	 * 0x1:debug version;
	 * 0x2:release version
	 */
	unsigned int  product_ver;
	/* 0x0:invalid;
	 * 0x1:2G modem;
	 * 0x2: 3G modem
	 */
	unsigned int  image_type;
	/* MT6573_S01 or MT6573_S02 */
	unsigned char platform[16];
	/* build time string */
	unsigned char build_time[64];
	/* project version, ex:11A_MD.W11.28 */
	unsigned char build_ver[64];

	/* bind to md sys id,
	 * MD SYS1: 1,
	 * MD SYS2: 2
	 */
	unsigned char bind_sys_id;
	/* no shrink: 0, shrink: 1*/
	unsigned char ext_attr;
	/* for reserved */
	unsigned char reserved[2];

	/* md ROM/RAM image size requested by md */
	unsigned int  mem_size;
	/* md image size, exclude head size*/
	unsigned int  md_img_size;

} __packed;

struct _free_padding_block {
	unsigned int start_offset;
	unsigned int length;
};

struct md_check_header_v6 {
	/* magic number is "CHECK_HEADER"*/
	unsigned char check_header[12];
	/* header structure version number */
	unsigned int  header_verno;
	/* 0x0:invalid;
	 * 0x1:debug version;
	 * 0x2:release version
	 */
	unsigned int  product_ver;
	/* 0x0:invalid;
	 * 0x1:2G modem;
	 * 0x2: 3G modem
	 */
	unsigned int  image_type;
	/* MT6573_S01 or MT6573_S02 */
	unsigned char platform[16];
	/* build time string */
	unsigned char build_time[64];
	/* project version, ex:11A_MD.W11.28 */
	unsigned char build_ver[64];

	/* bind to md sys id,
	 * MD SYS1: 1,
	 * MD SYS2: 2,
	 * MD SYS5: 5
	 */
	unsigned char bind_sys_id;
	/* no shrink: 0, shrink: 1 */
	unsigned char ext_attr;
	/* for reserved */
	unsigned char reserved[2];

	/* md ROM/RAM image size requested by md */
	unsigned int  mem_size;
	/* md image size, exclude head size */
	unsigned int  md_img_size;
	/* RPC secure memory address */
	unsigned int  rpc_sec_mem_addr;

	unsigned int  dsp_img_offset;
	unsigned int  dsp_img_size;

	/* total region number */
	unsigned int  region_num;
	/* max support 8 regions */
	struct _md_regin_info region_info[8];
	/* max support 4 domain settings,
	 * each region has 4 control bits
	 */
	unsigned int  domain_attr[4];

	unsigned int  arm7_img_offset;
	unsigned int  arm7_img_size;

	struct _free_padding_block padding_blk[8];

	unsigned int  ap_md_smem_size;
	unsigned int  md_to_md_smem_size;

	unsigned int ramdisk_offset;
	unsigned int ramdisk_size;

	unsigned char reserved_1[144];

	/* the size of this structure */
	unsigned int  size;
};

extern int ccci_common_sysfs_init(void);
extern void ccci_log_init(void);
extern int __init ccci_util_fo_init(void);
extern void ccci_timer_for_md_init(void);
extern const char *ld_md_errno_to_str(int errno);
extern int ccci_util_broadcast_init(void);
/* qqcandy: definition is void; the vendor header said int */
extern void ccci_sib_init(void);
extern int ccci_util_pin_broadcast_init(void);
/* qqcandy: defined in ccci_private_log.c, declared by ccci_util_lib_sys.h */
int get_dump_buf_usage(char buf[], int size);

/*
 * qqcandy: the vendor build did not enable -Wmissing-prototypes, so these had
 * no declaration anywhere. Keep their external linkage, just declare them.
 */
struct reserved_mem;
unsigned int get_md_smem_align(int md_id);
int ccci_reserve_mem_of_init(struct reserved_mem *rmem);

/* qqcandy: broadcast/pin helpers, normally consumed by the ECCCI core. */
void inject_md_status_event(int md_id, int event_type, char reason[]);
int get_lock_rst_user_cnt(int md_id);
int get_lock_rst_user_list(int md_id, char list_buff[], int size);
void set_soc_md_rt_rat(int md_id, unsigned int bitmap, unsigned int id);

#define MAX_MD_NUM_AT_LK	(4)

/*
 * qqcandy: gate for the unsafe LK/modem tag parse in ccci_util_lib_fo.c.
 *
 * It is a plain built-in bool and is never set anywhere in this tree: no
 * parameter, no sysfs hook. The tunable, bounded, read-only runtime parse
 * (and its module_param_cb + workqueue trigger) was moved out into the
 * separate ccci_probe.ko module (drivers/misc/mediatek/ccci_probe/) so that
 * no part of the reserved-memory dereference lives in the boot image. With
 * this flag permanently false, ccci_util_fo_init() and
 * collect_lk_boot_arguments() can never touch a reserved region.
 */
extern bool ccci_util_probe;

#endif
