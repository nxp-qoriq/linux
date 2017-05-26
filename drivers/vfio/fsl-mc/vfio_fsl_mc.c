/*
 * Freescale Management Complex (MC) device passthrough using VFIO
 *
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vfio.h>

#include "../../staging/fsl-mc/include/mc.h"
#include "../../staging/fsl-mc/include/mc-bus.h"
#include "../../staging/fsl-mc/include/mc-sys.h"

#include "vfio_fsl_mc_private.h"

#define DRIVER_VERSION	"0.10"
#define DRIVER_AUTHOR	"Bharat Bhushan <bharat.bhushan@nxp.com>"
#define DRIVER_DESC	"VFIO for FSL-MC devices - User Level meta-driver"

static int vfio_fsl_mc_open(void *device_data)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static void vfio_fsl_mc_release(void *device_data)
{
	module_put(THIS_MODULE);
}

static long vfio_fsl_mc_ioctl(void *device_data, unsigned int cmd,
			      unsigned long arg)
{
	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		return -EINVAL;
	}
	case VFIO_DEVICE_RESET:
	{
		return -EINVAL;
	}
	default:
		return -EINVAL;
	}
}

static ssize_t vfio_fsl_mc_read(void *device_data, char __user *buf,
				size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t vfio_fsl_mc_write(void *device_data, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	return -EINVAL;
}

static int vfio_fsl_mc_mmap(void *device_data, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static const struct vfio_device_ops vfio_fsl_mc_ops = {
	.name		= "vfio-fsl-mc",
	.open		= vfio_fsl_mc_open,
	.release	= vfio_fsl_mc_release,
	.ioctl		= vfio_fsl_mc_ioctl,
	.read		= vfio_fsl_mc_read,
	.write		= vfio_fsl_mc_write,
	.mmap		= vfio_fsl_mc_mmap,
};

static int vfio_fsl_mc_initialize_dprc(struct vfio_fsl_mc_device *vdev)
{
	struct device *root_dprc_dev;
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct device *dev = &mc_dev->dev;
	struct fsl_mc_bus *mc_bus;
	unsigned int irq_count;
	int ret;

	/* device must be DPRC */
	if (strcmp(mc_dev->obj_desc.type, "dprc"))
		return -EINVAL;

	/* mc_io must be un-initialized */
	WARN_ON(mc_dev->mc_io);

	/* allocate a portal from the root DPRC for vfio use */
	fsl_mc_get_root_dprc(dev, &root_dprc_dev);
	if (WARN_ON(!root_dprc_dev))
		return -EINVAL;

	ret = fsl_mc_portal_allocate(to_fsl_mc_device(root_dprc_dev),
				     FSL_MC_IO_ATOMIC_CONTEXT_PORTAL,
				     &mc_dev->mc_io);
	if (ret < 0)
		return ret;

	/* Reset MCP before move on */
	ret = fsl_mc_portal_reset(mc_dev->mc_io);
	if (ret < 0) {
		dev_err(dev, "dprc portal reset failed: error = %d\n", ret);
		goto free_mc_portal;
	}

	ret = dprc_open(mc_dev->mc_io, 0, mc_dev->obj_desc.id,
			&mc_dev->mc_handle);
	if (ret) {
		dev_err(dev, "dprc_open() failed: error = %d\n", ret);
		goto free_mc_portal;
	}

	/* Initialize resource pool */
	fsl_mc_init_all_resource_pools(mc_dev);

	mc_bus = to_fsl_mc_bus(mc_dev);

	mutex_init(&mc_bus->scan_mutex);

	mutex_lock(&mc_bus->scan_mutex);
	ret = dprc_scan_objects(mc_dev, mc_dev->driver_override,
				&irq_count);
	mutex_unlock(&mc_bus->scan_mutex);
	if (ret) {
		dev_err(dev, "dprc_scan_objects() fails (%d)\n", ret);
		goto clean_resource_pool;
	}

	return 0;

clean_resource_pool:
	fsl_mc_cleanup_all_resource_pools(mc_dev);
	dprc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);

free_mc_portal:
	fsl_mc_portal_free(mc_dev->mc_io);
	return ret;
}

static int vfio_fsl_mc_device_remove(struct device *dev, void *data)
{
	struct fsl_mc_device *mc_dev;

	WARN_ON(dev == NULL);

	mc_dev = to_fsl_mc_device(dev);
	if (WARN_ON(mc_dev == NULL))
		return -ENODEV;

	fsl_mc_device_remove(mc_dev);
	return 0;
}

static void vfio_fsl_mc_cleanup_dprc(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;

	/* device must be DPRC */
	if (strcmp(mc_dev->obj_desc.type, "dprc"))
		return;

	device_for_each_child(&mc_dev->dev, NULL, vfio_fsl_mc_device_remove);

	fsl_mc_cleanup_all_resource_pools(mc_dev);
	dprc_close(mc_dev->mc_io, 0, mc_dev->mc_handle);
	fsl_mc_portal_free(mc_dev->mc_io);
}

static int vfio_fsl_mc_probe(struct fsl_mc_device *mc_dev)
{
	struct iommu_group *group;
	struct vfio_fsl_mc_device *vdev;
	struct device *dev = &mc_dev->dev;
	int ret;

	group = iommu_group_get(dev);
	if (!group) {
		dev_err(dev, "%s: VFIO: No IOMMU group\n", __func__);
		return -EINVAL;
	}

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev) {
		iommu_group_put(group);
		return -ENOMEM;
	}

	vdev->mc_dev = mc_dev;

	ret = vfio_add_group_dev(dev, &vfio_fsl_mc_ops, vdev);
	if (ret) {
		dev_err(dev, "%s: Failed to add to vfio group\n", __func__);
		goto free_vfio_device;
	}

	/* DPRC container scanned and it's chilren bound with vfio driver */
	if (strcmp(mc_dev->obj_desc.type, "dprc") == 0) {
		ret = vfio_fsl_mc_initialize_dprc(vdev);
		if (ret) {
			vfio_del_group_dev(dev);
			goto free_vfio_device;
		}
	} else {
		struct fsl_mc_device *mc_bus_dev;

		/* Non-dprc devices share mc_io from the parent dprc */
		mc_bus_dev = to_fsl_mc_device(mc_dev->dev.parent);
		if (mc_bus_dev == NULL) {
			vfio_del_group_dev(dev);
			goto free_vfio_device;
		}

		mc_dev->mc_io = mc_bus_dev->mc_io;
	}
	return 0;

free_vfio_device:
	kfree(vdev);
	iommu_group_put(group);
	return ret;
}

static int vfio_fsl_mc_remove(struct fsl_mc_device *mc_dev)
{
	struct vfio_fsl_mc_device *vdev;
	struct device *dev = &mc_dev->dev;

	vdev = vfio_del_group_dev(dev);
	if (!vdev)
		return -EINVAL;

	if (strcmp(mc_dev->obj_desc.type, "dprc") == 0)
		vfio_fsl_mc_cleanup_dprc(vdev);

	mc_dev->mc_io = NULL;

	iommu_group_put(mc_dev->dev.iommu_group);
	kfree(vdev);

	return 0;
}

/*
 * vfio-fsl_mc is a meta-driver, so use driver_override interface to
 * bind a fsl_mc container with this driver and match_id_table is NULL.
 */
static struct fsl_mc_driver vfio_fsl_mc_driver = {
	.probe		= vfio_fsl_mc_probe,
	.remove		= vfio_fsl_mc_remove,
	.match_id_table = NULL,
	.driver	= {
		.name	= "vfio-fsl-mc",
		.owner	= THIS_MODULE,
	},
};

static int __init vfio_fsl_mc_driver_init(void)
{
	return fsl_mc_driver_register(&vfio_fsl_mc_driver);
}

static void __exit vfio_fsl_mc_driver_exit(void)
{
	fsl_mc_driver_unregister(&vfio_fsl_mc_driver);
}

module_init(vfio_fsl_mc_driver_init);
module_exit(vfio_fsl_mc_driver_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
