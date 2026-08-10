// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe Endpoint Absolute Mouse Driver
 *
 * Captures absolute mouse input events from local devices and forwards
 * HID reports over the PCIe endpoint using the EPF HID helper library.
 *
 * Copyright 2026 NXP
 */

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/pci-epf-hid.h>

#define DEVICE_NAME "pcie-abs-mouse"

static dev_t abs_mouse_devno;
static struct cdev abs_mouse_cdev;
static struct class *abs_mouse_class;

struct abs_mouse {
	struct input_handle handle;
	struct work_struct work;
	spinlock_t lock;
	struct hid_abs_mouse_report report;
};

static void abs_mouse_send_work(struct work_struct *work)
{
	struct abs_mouse *mouse =
		container_of(work, struct abs_mouse, work);
	struct hid_abs_mouse_report rpt;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mouse->lock, flags);

	rpt = mouse->report;
	memset(&mouse->report, 0, sizeof(mouse->report));

	spin_unlock_irqrestore(&mouse->lock, flags);

	ret = epf_hid_send_abs_mouse(&rpt);
	if (ret)
		pr_err("failed to send absolute mouse report: %d\n", ret);
}

static void abs_mouse_event(struct input_handle *handle,
			    unsigned int type,
			    unsigned int code,
			    int value)
{
	struct abs_mouse *mouse =
		container_of(handle, struct abs_mouse, handle);
	unsigned long flags;

	spin_lock_irqsave(&mouse->lock, flags);

	switch (type) {
	case EV_ABS:
		switch (code) {
		case ABS_X:
			mouse->report.x = cpu_to_le16(value);
			break;

		case ABS_Y:
			mouse->report.y = cpu_to_le16(value);
			break;

		case ABS_WHEEL:
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
				mouse->report.buttons |= 1;
			else
				mouse->report.buttons &= ~1;
			break;

		case BTN_RIGHT:
			if (value)
				mouse->report.buttons |= 2;
			else
				mouse->report.buttons &= ~2;
			break;

		case BTN_MIDDLE:
			if (value)
				mouse->report.buttons |= 4;
			else
				mouse->report.buttons &= ~4;
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

static int abs_mouse_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct abs_mouse *mouse;
	int ret;

	pr_info("connect %s\n", dev_name(&dev->dev));

	mouse = kzalloc(sizeof(*mouse), GFP_KERNEL);
	if (!mouse)
		return -ENOMEM;

	spin_lock_init(&mouse->lock);
	INIT_WORK(&mouse->work, abs_mouse_send_work);

	mouse->handle.dev = dev;
	mouse->handle.handler = handler;
	mouse->handle.name = "epf_abs_mouse";

	ret = input_register_handle(&mouse->handle);
	if (ret)
		goto err_free;

	ret = input_open_device(&mouse->handle);
	if (ret)
		goto err_unregister;

	pr_info("absolute mouse connected\n");

	return 0;

err_unregister:
	input_unregister_handle(&mouse->handle);
err_free:
	kfree(mouse);

	return ret;
}

static void abs_mouse_disconnect(struct input_handle *handle)
{
	struct abs_mouse *mouse =
		container_of(handle, struct abs_mouse, handle);

	cancel_work_sync(&mouse->work);

	input_close_device(handle);
	input_unregister_handle(handle);

	kfree(mouse);

	pr_info("absolute mouse disconnected\n");
}

static const struct input_device_id abs_mouse_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = {
			BIT_MASK(EV_KEY) |
			BIT_MASK(EV_ABS),
		},
		.absbit = {
			BIT_MASK(ABS_X) |
			BIT_MASK(ABS_Y),
		},
	},
	{ }
};

MODULE_DEVICE_TABLE(input, abs_mouse_ids);

static struct input_handler abs_mouse_handler = {
	.event		= abs_mouse_event,
	.connect	= abs_mouse_connect,
	.disconnect	= abs_mouse_disconnect,
	.name		= "epf_abs_mouse",
	.id_table	= abs_mouse_ids,
};

static int abs_mouse_open(struct inode *inode, struct file *file)
{
	pr_debug("device opened\n");

	return 0;
}

static int abs_mouse_release(struct inode *inode, struct file *file)
{
	pr_debug("device closed\n");

	return 0;
}

static ssize_t abs_mouse_write(struct file *file,
			       const char __user *buf,
			       size_t len,
			       loff_t *ppos)
{
	struct hid_abs_mouse_report rpt;
	int ret;

	if (len != sizeof(rpt))
		return -EINVAL;

	if (copy_from_user(&rpt, buf, sizeof(rpt)))
		return -EFAULT;

	ret = epf_hid_send_abs_mouse(&rpt);
	if (ret)
		return ret;

	return sizeof(rpt);
}

static const struct file_operations abs_mouse_fops = {
	.owner		= THIS_MODULE,
	.open		= abs_mouse_open,
	.release	= abs_mouse_release,
	.write		= abs_mouse_write,
};

int abs_mouse_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&abs_mouse_devno, 0, 1, DEVICE_NAME);
	if (ret)
		return ret;

	cdev_init(&abs_mouse_cdev, &abs_mouse_fops);
	abs_mouse_cdev.owner = THIS_MODULE;

	ret = cdev_add(&abs_mouse_cdev, abs_mouse_devno, 1);
	if (ret)
		goto err_chrdev;

	abs_mouse_class = class_create(DEVICE_NAME);
	if (IS_ERR(abs_mouse_class)) {
		ret = PTR_ERR(abs_mouse_class);
		goto err_cdev;
	}

	if (IS_ERR(device_create(abs_mouse_class, NULL,
				 abs_mouse_devno, NULL, "pciams"))) {
		ret = -ENODEV;
		goto err_class;
	}

	ret = input_register_handler(&abs_mouse_handler);
	if (ret)
		goto err_device;

	pr_info("absolute mouse driver registered\n");

	return 0;

err_device:
	device_destroy(abs_mouse_class, abs_mouse_devno);

err_class:
	class_destroy(abs_mouse_class);

err_cdev:
	cdev_del(&abs_mouse_cdev);

err_chrdev:
	unregister_chrdev_region(abs_mouse_devno, 1);

	return ret;
}

void abs_mouse_exit(void)
{
	input_unregister_handler(&abs_mouse_handler);

	device_destroy(abs_mouse_class, abs_mouse_devno);
	class_destroy(abs_mouse_class);

	cdev_del(&abs_mouse_cdev);
	unregister_chrdev_region(abs_mouse_devno, 1);

	pr_info("absolute mouse driver removed\n");
}