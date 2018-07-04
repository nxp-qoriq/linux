// SPDX-License-Identifier: GPL-2.0
/*
 * Management Complex (MC) userspace support
 *
 * Copyright 2018 NXP
 *
 */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include "fsl-mc-private.h"
#include "mc-ioctl.h"
#include "../include/mc-sys.h"

struct uapi_priv_data {
	struct fsl_mc_uapi *uapi;
	struct fsl_mc_io *mc_io;
};

static int fsl_mc_uapi_send_command(unsigned long arg,
				       struct fsl_mc_io *mc_io)
{
	struct mc_command mc_cmd;
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

static int fsl_mc_uapi_dev_open(struct inode *inode, struct file *filep)
{
	struct uapi_priv_data *priv_data;
	struct fsl_mc_device *root_mc_device;
	struct fsl_mc_uapi *mc_uapi;
	struct fsl_mc_io *dynamic_mc_io;
	int error;

	priv_data = kzalloc(sizeof(*priv_data), GFP_KERNEL);
	if (!priv_data) {
		return -ENOMEM;
	}

	mc_uapi = container_of(filep->private_data, struct fsl_mc_uapi, misc);
	root_mc_device = container_of(mc_uapi, struct fsl_mc_device, uapi_misc);

	mutex_lock(&mc_uapi->mutex);

	if (!mc_uapi->root_mc_io_in_use) {
		priv_data->mc_io = mc_uapi->root_mc_io;
		mc_uapi->root_mc_io_in_use = true;
	} else {
		error = fsl_mc_portal_allocate(root_mc_device, 0,
					       &dynamic_mc_io);
		if (error)
			goto error_portal_allocate;

		mc_uapi->dynamic_instance_count++;
		priv_data->mc_io = dynamic_mc_io;
	}
	priv_data->uapi = mc_uapi;
	filep->private_data = priv_data;

	mutex_unlock(&mc_uapi->mutex);

	return 0;

error_portal_allocate:
	mutex_unlock(&mc_uapi->mutex);

	return error;
}

static int fsl_mc_uapi_dev_release(struct inode *inode, struct file *filep)
{
	struct uapi_priv_data *priv_data;
	struct fsl_mc_device *root_mc_device;
	struct fsl_mc_uapi *mc_uapi;
	struct fsl_mc_io *mc_io;

	priv_data = filep->private_data;
	mc_uapi = priv_data->uapi;
	mc_io = priv_data->mc_io;
	root_mc_device = container_of(mc_uapi, struct fsl_mc_device, uapi_misc);

	mutex_lock(&mc_uapi->mutex);

	if (WARN_ON(!mc_uapi->root_mc_io_in_use &&
		    mc_uapi->dynamic_instance_count == 0)) {
		mutex_unlock(&mc_uapi->mutex);
		return -EINVAL;
	}

	if (mc_io == mc_uapi->root_mc_io) {
		mc_uapi->root_mc_io_in_use = false;
	} else {
		fsl_mc_portal_free(mc_io);
		mc_uapi->dynamic_instance_count--;
	}

	kfree(filep->private_data);
	filep->private_data =  NULL;

	mutex_unlock(&mc_uapi->mutex);

	return 0;
}

static long fsl_mc_uapi_dev_ioctl(struct file *file,
				     unsigned int cmd,
				     unsigned long arg)
{
	struct uapi_priv_data *priv_data = file->private_data;
	int error;

	switch (cmd) {
	case RESTOOL_SEND_MC_COMMAND:
		error = fsl_mc_uapi_send_command(arg, priv_data->mc_io);
		break;
	default:
		pr_err("%s: unexpected ioctl call number\n", __func__);
		error = -EINVAL;
	}

	return error;
}

static const struct file_operations fsl_mc_uapi_dev_fops = {
	.owner = THIS_MODULE,
	.open = fsl_mc_uapi_dev_open,
	.release = fsl_mc_uapi_dev_release,
	.unlocked_ioctl = fsl_mc_uapi_dev_ioctl,
};

int fsl_mc_uapi_create_device_file(struct fsl_mc_device *root_dprc)
{
	struct fsl_mc_uapi *mc_uapi = &root_dprc->uapi_misc;
	int error;

	mc_uapi->misc.minor = MISC_DYNAMIC_MINOR;
	mc_uapi->misc.name = dev_name(&root_dprc->dev);
	mc_uapi->misc.fops = &fsl_mc_uapi_dev_fops;

	error = misc_register(&mc_uapi->misc);
	if (error) {
		return -EPROBE_DEFER;
	}

	mc_uapi->root_mc_io = root_dprc->mc_io;

	mutex_init(&mc_uapi->mutex);

	return 0;
}

void fsl_mc_uapi_remove_device_file(struct fsl_mc_device *root_dprc)
{
	struct fsl_mc_uapi *mc_uapi = &root_dprc->uapi_misc;

	if (WARN_ON(mc_uapi->root_mc_io_in_use))
		return;

	if (WARN_ON(mc_uapi->dynamic_instance_count != 0))
		return;

	misc_deregister(&mc_uapi->misc);
}
