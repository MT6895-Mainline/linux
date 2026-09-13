// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include "ccci_internal.h"

int ccci_port_register(struct ccci_port *port)
{
	struct ccci_device *cdev = g_ccci_dev;

	if (!cdev) {
		pr_err("CCCI core not initialized\n");
		return -ENODEV;
	}

	mutex_lock(&cdev->port_lock);
	list_add_tail(&port->node, &cdev->ports);
	mutex_unlock(&cdev->port_lock);

	pr_info("CCCI: port '%s' registered\n", port->name);
	return 0;
}
EXPORT_SYMBOL_GPL(ccci_port_register);

void ccci_port_unregister(struct ccci_port *port)
{
	struct ccci_device *cdev = g_ccci_dev;

	if (!cdev)
		return;

	mutex_lock(&cdev->port_lock);
	list_del(&port->node);
	mutex_unlock(&cdev->port_lock);

	pr_info("CCCI: port '%s' unregistered\n", port->name);
}
EXPORT_SYMBOL_GPL(ccci_port_unregister);
