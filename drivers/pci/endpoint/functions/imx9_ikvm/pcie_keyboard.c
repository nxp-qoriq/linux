// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2026 NXP
 *
 * PCIe Endpoint Virtual Keyboard Driver
 *
 * Provides a character device interface for sending keyboard HID reports
 * over the PCIe endpoint using the EPF HID helper library.
 *
 */

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>

#include "pci-epf-hid.h"

#define DEVICE_NAME	"pcie-keyboard"

static dev_t keyboard_devno;
static struct cdev keyboard_cdev;
static struct class *keyboard_class;

static int keyboard_open(struct inode *inode, struct file *file)
{
	pr_debug("device opened\n");

	return 0;
}

static int keyboard_release(struct inode *inode, struct file *file)
{
	pr_debug("device closed\n");

	return 0;
}

static ssize_t keyboard_write(struct file *file,
			      const char __user *buf,
			      size_t len,
			      loff_t *ppos)
{
	struct hid_keyboard_report rpt;
	int ret;

	if (len != sizeof(rpt))
		return -EINVAL;

	if (copy_from_user(&rpt, buf, sizeof(rpt)))
		return -EFAULT;

	ret = epf_hid_send_keyboard(&rpt);
	if (ret)
		return ret;

	return sizeof(rpt);
}

static const struct file_operations keyboard_fops = {
	.owner		= THIS_MODULE,
	.open		= keyboard_open,
	.release	= keyboard_release,
	.write		= keyboard_write,
};

int keyboard_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&keyboard_devno, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&keyboard_cdev, &keyboard_fops);
	keyboard_cdev.owner = THIS_MODULE;

	ret = cdev_add(&keyboard_cdev, keyboard_devno, 1);
	if (ret)
		goto err_chrdev;

	keyboard_class = class_create(DEVICE_NAME);
	if (IS_ERR(keyboard_class)) {
		ret = PTR_ERR(keyboard_class);
		goto err_cdev;
	}

	if (IS_ERR(device_create(keyboard_class, NULL,
				 keyboard_devno, NULL, "pcikbd"))) {
		ret = -ENODEV;
		goto err_class;
	}

	pr_info("virtual keyboard driver registered\n");

	return 0;

err_class:
	class_destroy(keyboard_class);

err_cdev:
	cdev_del(&keyboard_cdev);

err_chrdev:
	unregister_chrdev_region(keyboard_devno, 1);

	return ret;
}

void keyboard_exit(void)
{
	device_destroy(keyboard_class, keyboard_devno);

	class_destroy(keyboard_class);

	cdev_del(&keyboard_cdev);

	unregister_chrdev_region(keyboard_devno, 1);

	pr_info("virtual keyboard driver removed\n");
}

