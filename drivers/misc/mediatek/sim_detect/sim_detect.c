// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define SIM_DETECT_NAME "sim_detect"

MODULE_DESCRIPTION("Oplus SIM card detect");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qicai.gu <qicai.gu>");

static const struct of_device_id sim_detect_of_match[] = {
	{ .compatible = "oplus, sim_detect" },
	{}
};
MODULE_DEVICE_TABLE(of, sim_detect_of_match);

struct sim_detect_data {
	int gpio;
};

static ssize_t sim_detect_read(struct file *file, char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	struct sim_detect_data *data = pde_data(file_inode(file));
	char page[16];
	int value;
	int len;

	if (!data)
		return 0;

	value = gpio_get_value(data->gpio);
	len = scnprintf(page, sizeof(page), "%d\n", value);

	return simple_read_from_buffer(user_buf, count, ppos, page, len);
}

static const struct proc_ops sim_detect_proc_ops = {
	.proc_read = sim_detect_read,
	.proc_open = simple_open,
	.proc_lseek = default_llseek,
};

static int sim_detect_probe(struct platform_device *pdev)
{
	struct sim_detect_data *data;
	struct proc_dir_entry *entry;
	int gpio;

	gpio = of_get_named_gpio(pdev->dev.of_node, "Hw,sim_det", 0);
	if (gpio < 0)
		return dev_err_probe(&pdev->dev, gpio,
				    "missing Hw,sim_det GPIO\n");

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->gpio = gpio;

	entry = proc_create_data(SIM_DETECT_NAME, 0444, NULL,
				 &sim_detect_proc_ops, data);
	if (!entry)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	return 0;
}

static void sim_detect_remove(struct platform_device *pdev)
{
	remove_proc_entry(SIM_DETECT_NAME, NULL);
}

static struct platform_driver sim_detect_driver = {
	.probe = sim_detect_probe,
	.remove = sim_detect_remove,
	.driver = {
		.name = SIM_DETECT_NAME,
		.of_match_table = sim_detect_of_match,
	},
};
module_platform_driver(sim_detect_driver);
