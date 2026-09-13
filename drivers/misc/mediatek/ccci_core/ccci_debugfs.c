// SPDX-License-Identifier: GPL-2.0
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "ccci_internal.h"

static struct dentry *ccci_debugfs_root;

static int ccci_state_show(struct seq_file *m, void *v)
{
	struct ccci_device *cdev = m->private;
	const char *state_str;

	mutex_lock(&cdev->state_lock);
	switch (cdev->state) {
	case CCCI_STATE_INVALID: state_str = "INVALID"; break;
	case CCCI_STATE_IDLE: state_str = "IDLE"; break;
	case CCCI_STATE_READY: state_str = "READY"; break;
	case CCCI_STATE_BOOT: state_str = "BOOT"; break;
	case CCCI_STATE_RUNNING: state_str = "RUNNING"; break;
	case CCCI_STATE_EXCEPTION: state_str = "EXCEPTION"; break;
	default: state_str = "UNKNOWN"; break;
	}
	mutex_unlock(&cdev->state_lock);

	seq_printf(m, "%s\n", state_str);
	return 0;
}

static int ccci_tag_info_show(struct seq_file *m, void *v)
{
	struct ccci_device *cdev = m->private;
	struct ccci_tag_result *result = &cdev->parse_result;

	seq_printf(m, "Tag base: 0x%llx\n", (unsigned long long)cdev->tag_base);
	seq_printf(m, "Tags walked: %u\n", result->tags_walked);

	if (result->smem_found) {
		seq_puts(m, "\nSMEM layout:\n");
		seq_printf(m, "  Base: 0x%llx\n", result->smem.base_addr);
		seq_printf(m, "  Size: 0x%x\n", result->smem.total_smem_size);
	}

	if (result->ccb_found) {
		seq_puts(m, "\nCCB layout:\n");
		seq_printf(m, "  Addr: 0x%llx\n", result->ccb.addr);
		seq_printf(m, "  Size: 0x%x\n", result->ccb.size);
	}

	return 0;
}

static int ccci_memory_layout_show(struct seq_file *m, void *v)
{
	struct ccci_device *cdev = m->private;

	seq_printf(m, "TAG:  phys=0x%llx size=%zu (mapped)\n",
		   (unsigned long long)cdev->tag_base, cdev->tag_size);
	seq_printf(m, "SMEM: phys=0x%llx size=%zu (not mapped)\n",
		   (unsigned long long)cdev->smem_base, cdev->smem_size);
	seq_printf(m, "CCB:  phys=0x%llx size=%zu (not mapped)\n",
		   (unsigned long long)cdev->ccb_base, cdev->ccb_size);

	return 0;
}

static int ccci_ports_show(struct seq_file *m, void *v)
{
	struct ccci_device *cdev = m->private;
	struct ccci_port *port;
	int count = 0;

	mutex_lock(&cdev->port_lock);
	list_for_each_entry(port, &cdev->ports, node) {
		seq_printf(m, "[%d] %s (id=%d)\n", count++, port->name, port->port_id);
	}
	mutex_unlock(&cdev->port_lock);

	if (count == 0)
		seq_puts(m, "(no ports registered)\n");

	return 0;
}

DEFINE_SHOW_ATTRIBUTE(ccci_state);
DEFINE_SHOW_ATTRIBUTE(ccci_tag_info);
DEFINE_SHOW_ATTRIBUTE(ccci_memory_layout);
DEFINE_SHOW_ATTRIBUTE(ccci_ports);

int ccci_debugfs_init(struct ccci_device *cdev)
{
	if (!ccci_debugfs_root) {
		ccci_debugfs_root = debugfs_create_dir("ccci", NULL);
		if (IS_ERR(ccci_debugfs_root)) {
			ccci_debugfs_root = NULL;
			return -ENODEV;
		}
	}

	cdev->debugfs_root = ccci_debugfs_root;

	debugfs_create_file("state", 0444, cdev->debugfs_root, cdev,
			    &ccci_state_fops);
	debugfs_create_file("tag_info", 0444, cdev->debugfs_root, cdev,
			    &ccci_tag_info_fops);
	debugfs_create_file("memory_layout", 0444, cdev->debugfs_root, cdev,
			    &ccci_memory_layout_fops);
	debugfs_create_file("ports", 0444, cdev->debugfs_root, cdev,
			    &ccci_ports_fops);

	return 0;
}

void ccci_debugfs_exit(struct ccci_device *cdev)
{
	if (ccci_debugfs_root) {
		debugfs_remove_recursive(ccci_debugfs_root);
		ccci_debugfs_root = NULL;
	}
}
