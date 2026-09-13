/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __CCCI_INTERNAL_H__
#define __CCCI_INTERNAL_H__

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include "../ccci_util/ccci_tag_parse.h"

enum ccci_state {
	CCCI_STATE_INVALID = 0,
	CCCI_STATE_IDLE,
	CCCI_STATE_READY,
	CCCI_STATE_BOOT,
	CCCI_STATE_RUNNING,
	CCCI_STATE_EXCEPTION,
};

struct ccci_device {
	/* Tag region */
	phys_addr_t tag_base;
	void *tag_virt;
	size_t tag_size;

	/* SMEM region - described by tags, not mapped yet */
	phys_addr_t smem_base;
	size_t smem_size;

	/* CCB region - described by tags, not mapped yet */
	phys_addr_t ccb_base;
	size_t ccb_size;

	/* Tag parsing result */
	struct ccci_tag_result parse_result;

	/* State */
	enum ccci_state state;
	struct mutex state_lock;

	/* Port management */
	struct list_head ports;
	struct mutex port_lock;

	/* Debugfs */
	struct dentry *debugfs_root;
};

struct ccci_port {
	struct list_head node;
	const char *name;
	int port_id;
};

/* ccci_main.c */
extern struct ccci_device *g_ccci_dev;

/* ccci_debugfs.c */
int ccci_debugfs_init(struct ccci_device *cdev);
void ccci_debugfs_exit(struct ccci_device *cdev);

/* ccci_port.c */
int ccci_port_register(struct ccci_port *port);
void ccci_port_unregister(struct ccci_port *port);

#endif /* __CCCI_INTERNAL_H__ */
