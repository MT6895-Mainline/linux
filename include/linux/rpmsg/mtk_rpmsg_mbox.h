/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 Google LLC.
 */

#ifndef __LINUX_RPMSG_MTK_RPMSG_MBOX_H
#define __LINUX_RPMSG_MTK_RPMSG_MBOX_H

#include <linux/platform_device.h>
#include <linux/rpmsg.h>
#include <linux/spinlock.h>

struct mtk_rpmsg_channel_info {
	struct rpmsg_channel_info info;
	unsigned int send_slot; /* send slot offset */
	unsigned int recv_slot; /* recv slot offset */
	unsigned int send_slot_size; /* send slot count */
	unsigned int recv_slot_size; /* recv slot count */
	unsigned int send_pin_index; /* pin irq index */
	unsigned int recv_pin_index; /* pin irq index */
	unsigned int send_pin_offset; /* pin array offset */
	unsigned int recv_pin_offset; /* pin array offset */
	unsigned int mbox; /* mbox */
	spinlock_t channel_lock;
};

struct mtk_rpmsg_endpoint {
	struct rpmsg_endpoint ept;
	struct mtk_rpmsg_device *mdev;
	struct mtk_rpmsg_channel_info *mchan;
};

struct mtk_rpmsg_operations {
	int (*mbox_send)(struct mtk_rpmsg_endpoint *mept,
			 struct mtk_rpmsg_channel_info *mchan,
			 void *buf, unsigned int len, unsigned int wait);
};

struct mtk_rpmsg_device {
	struct rpmsg_device rpdev;
	struct platform_device *pdev;
	struct mtk_rpmsg_operations *ops;
	struct mtk_mbox_device *mbdev;
};

struct mtk_rpmsg_device *mtk_rpmsg_create_device(struct platform_device *pdev,
		struct mtk_mbox_device *mbdev, unsigned int ipc_chan_id);
struct mtk_rpmsg_channel_info *
mtk_rpmsg_create_channel(struct mtk_rpmsg_device *mdev, u32 chan_id,
		char *name);

#endif
