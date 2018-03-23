// SPDX-License-Identifier: GPL-2.0
/*
 * Management Complex (MC) restool support
 *
 * Copyright 2018 NXP
 *
 */

#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "fsl-mc-private.h"

#define FSL_MC_BUS_MAX_MINORS	1

static struct class *fsl_mc_bus_class;
static int fsl_mc_bus_major;

static int fsl_mc_restool_send_command(unsigned long arg,
				       struct fsl_mc_io *mc_io)
{
	struct fsl_mc_command mc_cmd;
	int error;

	error = copy_from_user(&mc_cmd, (void __user *)arg, sizeof(mc_cmd));
	if (error)
		return -EFAULT;

	error = mc_send_command(mc_io, &mc_cmd);
	if (error)
		return error;

	error = copy_to_user((void __user *)arg, &mc_cmd, sizeof(mc_cmd));
	if (error)
		return -EFAULT;

	return 0;
}

int fsl_mc_restool_init(void)
{
	dev_t dev;
	int error;

	fsl_mc_bus_class = class_create(THIS_MODULE, "fsl_mc_bus");
	if (IS_ERR(fsl_mc_bus_class)) {
		error = PTR_ERR(fsl_mc_bus_class);
		return error;
	}

	error = alloc_chrdev_region(&dev, 0,
				    FSL_MC_BUS_MAX_MINORS,
				    "fsl_mc_bus");
	if (error < 0)
		return error;

	fsl_mc_bus_major = MAJOR(dev);

	return 0;
}

static int fsl_mc_restool_dev_open(struct inode *inode, struct file *filep)
{
	struct fsl_mc_device *root_mc_device;
	struct fsl_mc_restool *mc_restool;
	struct fsl_mc_bus *mc_bus;
	struct fsl_mc_io *dynamic_mc_io;
	int error;

	mc_restool = container_of(inode->i_cdev, struct fsl_mc_restool, cdev);
	mc_bus = container_of(mc_restool, struct fsl_mc_bus, restool_misc);
	root_mc_device = &mc_bus->mc_dev;

	mutex_lock(&mc_restool->mutex);

	if (!mc_restool->local_instance_in_use) {
		filep->private_data = root_mc_device->mc_io;
		mc_restool->local_instance_in_use = true;
	} else {
		dynamic_mc_io = kzalloc(sizeof(*dynamic_mc_io), GFP_KERNEL);
		if (!dynamic_mc_io) {
			error = -ENOMEM;
			goto error_alloc_mc_io;
		}

		error = fsl_mc_portal_allocate(root_mc_device, 0,
					       &dynamic_mc_io);
		if (error) {
			pr_err("Could not allocate MC portal\n");
			goto error_portal_allocate;
		}

		mc_restool->dynamic_instance_count++;
		filep->private_data = dynamic_mc_io;
	}

	mutex_unlock(&mc_restool->mutex);

	return 0;

error_portal_allocate:
	kfree(dynamic_mc_io);

error_alloc_mc_io:
	mutex_unlock(&mc_restool->mutex);

	return error;
}

static int fsl_mc_restool_dev_release(struct inode *inode, struct file *filep)
{
	struct fsl_mc_device *root_mc_device;
	struct fsl_mc_restool *mc_restool;
	struct fsl_mc_bus *mc_bus;
	struct fsl_mc_io *mc_io;

	mc_restool = container_of(inode->i_cdev, struct fsl_mc_restool, cdev);
	mc_bus = container_of(mc_restool, struct fsl_mc_bus, restool_misc);
	root_mc_device = &mc_bus->mc_dev;
	mc_io = filep->private_data;

	mutex_lock(&mc_restool->mutex);

	if (WARN_ON(!mc_restool->local_instance_in_use &&
		    mc_restool->dynamic_instance_count == 0)) {
		mutex_unlock(&mc_restool->mutex);
		return -EINVAL;
	}

	if (filep->private_data == root_mc_device->mc_io) {
		mc_restool->local_instance_in_use = false;
	} else {
		fsl_mc_portal_free(mc_io);
		kfree(mc_io);
		mc_restool->dynamic_instance_count--;
	}

	filep->private_data = NULL;
	mutex_unlock(&mc_restool->mutex);

	return 0;
}

static long fsl_mc_restool_dev_ioctl(struct file *file,
				     unsigned int cmd,
				     unsigned long arg)
{
	int error;

	switch (cmd) {
	case RESTOOL_SEND_MC_COMMAND:
		error = fsl_mc_restool_send_command(arg, file->private_data);
		break;
	default:
		pr_err("%s: unexpected ioctl call number\n", __func__);
		error = -EINVAL;
	}

	return error;
}

static const struct file_operations fsl_mc_restool_dev_fops = {
	.owner = THIS_MODULE,
	.open = fsl_mc_restool_dev_open,
	.release = fsl_mc_restool_dev_release,
	.unlocked_ioctl = fsl_mc_restool_dev_ioctl,
};

int fsl_mc_restool_create_device_file(struct fsl_mc_bus *mc_bus)
{
	struct fsl_mc_device *mc_dev = &mc_bus->mc_dev;
	struct fsl_mc_restool *mc_restool = &mc_bus->restool_misc;
	int error;

	mc_restool = &mc_bus->restool_misc;
	mc_restool->dev = MKDEV(fsl_mc_bus_major, 0);
	cdev_init(&mc_restool->cdev, &fsl_mc_restool_dev_fops);

	error = cdev_add(&mc_restool->cdev,
			 mc_restool->dev,
			 FSL_MC_BUS_MAX_MINORS);
	if (error)
		return error;

	mc_restool->device = device_create(fsl_mc_bus_class,
					   NULL,
					   mc_restool->dev,
					   NULL,
					   "%s",
					   dev_name(&mc_dev->dev));
	if (IS_ERR(mc_restool->device)) {
		error = PTR_ERR(mc_restool->device);
		goto error_device_create;
	}

	mutex_init(&mc_restool->mutex);

	return 0;

error_device_create:
	cdev_del(&mc_restool->cdev);

	return error;
}

void fsl_mc_restool_remove_device_file(struct fsl_mc_bus *mc_bus)
{
	struct fsl_mc_restool *mc_restool = &mc_bus->restool_misc;

	if (WARN_ON(mc_restool->local_instance_in_use))
		return;

	if (WARN_ON(mc_restool->dynamic_instance_count != 0))
		return;

	cdev_del(&mc_restool->cdev);
}
