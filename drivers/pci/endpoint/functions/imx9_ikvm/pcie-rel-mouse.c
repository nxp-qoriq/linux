// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 *
 * PCIe Endpoint Relative Mouse Driver for i.MX95 iKVM
 *
 * Captures relative mouse input events from local and remote devices and
 * forwards them over the PCIe endpoint through the PCIe HID helper.
 *
 */

#define pr_fmt(fmt) "pcie-rel-mouse: " fmt

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "pci-epf-hid.h"

#define DEVICE_NAME "pcie-rel-mouse"

static dev_t pcie_rel_mouse_devno;
static struct cdev pcie_rel_mouse_cdev;
static struct class *pcie_rel_mouse_class;

struct pcie_rel_mouse {
	struct input_handle handle;
	struct work_struct work;
	spinlock_t lock;
	struct hid_rel_mouse_report report;
};

static void pcie_rel_mouse_send_work(struct work_struct *work)
{
	struct pcie_rel_mouse *mouse;
	struct hid_rel_mouse_report rpt;
	unsigned long flags;
	int ret;

	mouse = container_of(work, struct pcie_rel_mouse, work);

	spin_lock_irqsave(&mouse->lock, flags);
	rpt = mouse->report;
	memset(&mouse->report, 0, sizeof(mouse->report));
	spin_unlock_irqrestore(&mouse->lock, flags);

	ret = epf_hid_send_rel_mouse(&rpt);
	if (ret)
		pr_err("failed to send HID report (%d)\n", ret);
}

static void pcie_rel_mouse_event(struct input_handle *handle,
				 unsigned int type,
				 unsigned int code,
				 int value)
{
	struct pcie_rel_mouse *mouse;
	unsigned long flags;

	mouse = container_of(handle, struct pcie_rel_mouse, handle);

	spin_lock_irqsave(&mouse->lock, flags);

	switch (type) {
	case EV_REL:
		switch (code) {
		case REL_X:
			mouse->report.x = value;
			break;
		case REL_Y:
			mouse->report.y = value;
			break;
		case REL_WHEEL:
			mouse->report.wheel = value;
			break;
		default:
			spin_unlock_irqrestore(&mouse->lock, flags);
			return;
		}
		break;

	case EV_KEY:
		switch (code) {
		case BTN_LEFT:
			if (value)
				mouse->report.buttons |= BIT(0);
			else
				mouse->report.buttons &= ~BIT(0);
			break;

		case BTN_RIGHT:
			if (value)
				mouse->report.buttons |= BIT(1);
			else
				mouse->report.buttons &= ~BIT(1);
			break;

		case BTN_MIDDLE:
			if (value)
				mouse->report.buttons |= BIT(2);
			else
				mouse->report.buttons &= ~BIT(2);
			break;

		default:
			spin_unlock_irqrestore(&mouse->lock, flags);
			return;
		}
		break;

	case EV_SYN:
		spin_unlock_irqrestore(&mouse->lock, flags);

		if (code == SYN_REPORT && !work_pending(&mouse->work))
			schedule_work(&mouse->work);

		return;

	default:
		spin_unlock_irqrestore(&mouse->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&mouse->lock, flags);
}

static int pcie_rel_mouse_connect(struct input_handler *handler,
				  struct input_dev *dev,
				  const struct input_device_id *id)
{
	struct pcie_rel_mouse *mouse;
	int ret;

	pr_info("connect %s\n", dev_name(&dev->dev));

	mouse = kzalloc(sizeof(*mouse), GFP_KERNEL);
	if (!mouse)
		return -ENOMEM;

	spin_lock_init(&mouse->lock);
	INIT_WORK(&mouse->work, pcie_rel_mouse_send_work);

	mouse->handle.dev = dev;
	mouse->handle.handler = handler;
	mouse->handle.name = "pcie_rel_mouse";

	ret = input_register_handle(&mouse->handle);
	if (ret)
		goto err_free;

	ret = input_open_device(&mouse->handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(&mouse->handle);
err_free:
	kfree(mouse);
	return ret;
}

static void pcie_rel_mouse_disconnect(struct input_handle *handle)
{
	struct pcie_rel_mouse *mouse;

	mouse = container_of(handle, struct pcie_rel_mouse, handle);

	cancel_work_sync(&mouse->work);

	input_close_device(handle);
	input_unregister_handle(handle);

	kfree(mouse);

	pr_info("disconnected\n");
}

static const struct input_device_id pcie_rel_mouse_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = {
			BIT_MASK(EV_KEY) |
			BIT_MASK(EV_REL),
		},
	},
	{ }
};

MODULE_DEVICE_TABLE(input, pcie_rel_mouse_ids);

static struct input_handler pcie_rel_mouse_handler = {
	.event		= pcie_rel_mouse_event,
	.connect	= pcie_rel_mouse_connect,
	.disconnect	= pcie_rel_mouse_disconnect,
	.name		= "pcie_rel_mouse_handler",
	.id_table	= pcie_rel_mouse_ids,
};

static int pcie_rel_mouse_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int pcie_rel_mouse_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t
pcie_rel_mouse_write(struct file *file,
		     const char __user *buf,
		     size_t len,
		     loff_t *ppos)
{
	struct hid_rel_mouse_report rpt;
	int ret;

	if (len != sizeof(rpt))
		return -EINVAL;

	if (copy_from_user(&rpt, buf, sizeof(rpt)))
		return -EFAULT;

	ret = epf_hid_send_rel_mouse(&rpt);
	if (ret)
		return ret;

	return sizeof(rpt);
}

static const struct file_operations pcie_rel_mouse_fops = {
	.owner		= THIS_MODULE,
	.open		= pcie_rel_mouse_open,
	.release	= pcie_rel_mouse_release,
	.write		= pcie_rel_mouse_write,
};

int rel_mouse_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&pcie_rel_mouse_devno, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&pcie_rel_mouse_cdev, &pcie_rel_mouse_fops);
	pcie_rel_mouse_cdev.owner = THIS_MODULE;

	ret = cdev_add(&pcie_rel_mouse_cdev, pcie_rel_mouse_devno, 1);
	if (ret)
		goto err_chrdev;

	pcie_rel_mouse_class = class_create(DEVICE_NAME);
	if (IS_ERR(pcie_rel_mouse_class)) {
		ret = PTR_ERR(pcie_rel_mouse_class);
		goto err_cdev;
	}

	if (IS_ERR(device_create(pcie_rel_mouse_class, NULL,
				 pcie_rel_mouse_devno, NULL, "pcirms"))) {
		ret = -ENODEV;
		goto err_class;
	}

	ret = input_register_handler(&pcie_rel_mouse_handler);
	if (ret)
		goto err_device;

	pr_info("relative mouse initialized\n");

	return 0;

err_device:
	device_destroy(pcie_rel_mouse_class, pcie_rel_mouse_devno);
err_class:
	class_destroy(pcie_rel_mouse_class);
err_cdev:
	cdev_del(&pcie_rel_mouse_cdev);
err_chrdev:
	unregister_chrdev_region(pcie_rel_mouse_devno, 1);

	return ret;
}

void rel_mouse_exit(void)
{
	input_unregister_handler(&pcie_rel_mouse_handler);

	device_destroy(pcie_rel_mouse_class, pcie_rel_mouse_devno);
	class_destroy(pcie_rel_mouse_class);

	cdev_del(&pcie_rel_mouse_cdev);

	unregister_chrdev_region(pcie_rel_mouse_devno, 1);

	pr_info("relative mouse removed\n");
}

