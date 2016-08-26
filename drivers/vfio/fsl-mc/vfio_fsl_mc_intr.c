/*
 * Freescale Management Complex (MC) device passthrough using VFIO
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Author: Bharat Bhushan <bharat.bhushan@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/uaccess.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/vfio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include "../../staging/fsl-mc/include/mc.h"
#include "../../staging/fsl-mc/include/mc-sys.h"
#include "../../staging/fsl-mc/include/mc-private.h"
#include <linux/fs.h>

#include "vfio_fsl_mc_private.h"

static irqreturn_t vfio_fsl_mc_irq_handler(int irq_num, void *arg)
{
	struct vfio_fsl_mc_irq *mc_irq = (struct vfio_fsl_mc_irq *)arg;

	eventfd_signal(mc_irq->trigger, 1);
	return IRQ_HANDLED;
}

int vfio_fsl_mc_configure_irq(struct vfio_fsl_mc_device *vdev,
			      int irq_index)
{
	int error;
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct fsl_mc_device_irq *irq = mc_dev->irqs[irq_index];
	struct vfio_fsl_mc_irq *mc_irq = &vdev->mc_irqs[irq_index];
	struct device *dev = &mc_dev->dev;

	if (WARN_ON(!mc_irq->irq_initialized))
		return -EOPNOTSUPP;

	if (WARN_ON(mc_irq->irq_configured))
		return -EINVAL;

	mc_irq->name = kasprintf(GFP_KERNEL, "%s-%s-%d", "vfio-fsl-mc",
				 dev_name(dev), irq->msi_desc->irq);

	error = request_irq(irq->msi_desc->irq, vfio_fsl_mc_irq_handler,
			    0, mc_irq->name, mc_irq);
	if (error < 0) {
		dev_err(&mc_dev->dev,
			"IRQ registration fails with error: %d\n", error);
		kfree(mc_irq->name);
		return error;
	}

	mc_irq->irq_configured = true;
	return 0;
}

static void vfio_fsl_mc_unconfigure_irq(struct vfio_fsl_mc_device *vdev,
				       int irq_index)
{
	struct fsl_mc_device_irq *irq = vdev->mc_dev->irqs[irq_index];
	struct vfio_fsl_mc_irq *mc_irq = &vdev->mc_irqs[irq_index];

	if (!vdev->mc_irqs[irq_index].irq_configured)
		return;

	free_irq(irq->msi_desc->irq, mc_irq);
	kfree(vdev->mc_irqs[irq_index].name);
	vdev->mc_irqs[irq_index].irq_configured = false;
}

static int vfio_fsl_mc_setup_irqs(struct fsl_mc_device *mc_dev)
{
	int ret;
	int irq_count = mc_dev->obj_desc.irq_count;
	int hwirq;
	int i;

	/* Allocate IRQs */
	ret = fsl_mc_allocate_irqs(mc_dev);
	if  (ret)
		return ret;

	/* Disable IRQs */
	for (i = 0; i < irq_count; i++) {
		hwirq = mc_dev->irqs[i]->msi_desc->irq;
		disable_irq_nosync(hwirq);
	}

	return 0;
}

int vfio_fsl_mc_init_irqs(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct device *dev = &mc_dev->dev;
	int irq_count = mc_dev->obj_desc.irq_count;
	struct vfio_fsl_mc_irq *mc_irq;
	int ret, i;

	mc_irq = kcalloc(irq_count, sizeof(*mc_irq), GFP_KERNEL);
	if (mc_irq == NULL)
		return -ENOMEM;

	ret = vfio_fsl_mc_setup_irqs(mc_dev);
	if (ret) {
		kfree(mc_irq);
		dev_err(dev, "vfio_fsl_mc_setup_irqs Fails  %d\n", ret);
		return ret;
	}

	for (i = 0; i < irq_count; i++) {
		mc_irq[i].count = 1;
		mc_irq[i].flags = VFIO_IRQ_INFO_EVENTFD |
					VFIO_IRQ_INFO_MASKABLE;
		mc_irq[i].irq_initialized = true;
	}

	vdev->mc_irqs = mc_irq;

	return 0;
}

int vfio_fsl_mc_unconfigure_irqs(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	int i;

	for (i = 0; i < mc_dev->obj_desc.irq_count; i++) {
		if (!vdev->mc_irqs[i].irq_initialized)
			continue;

		vfio_fsl_mc_unconfigure_irq(vdev, i);
	}
	return 0;
}

/* Free All IRQs for the given MC object */
void vfio_fsl_mc_free_irqs(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;

	vfio_fsl_mc_unconfigure_irqs(vdev);
	fsl_mc_free_irqs(mc_dev);

	kfree(vdev->mc_irqs);
}

static int vfio_fsl_mc_irq_mask(struct vfio_fsl_mc_device *vdev,
				    unsigned index, unsigned start,
				    unsigned count, uint32_t flags, void *data,
				    uint32_t mask)
{
	uint8_t arr;

	if (start != 0 || count != 1)
		return -EINVAL;

	switch (flags & VFIO_IRQ_SET_DATA_TYPE_MASK) {
	case VFIO_IRQ_SET_DATA_BOOL:
		arr = *(uint8_t *) data;
		if (arr != 0x1)
			return -EINVAL;

	case VFIO_IRQ_SET_DATA_NONE:
		return -ENOTTY; /* To be Implemented */
	case VFIO_IRQ_SET_DATA_EVENTFD:
		return -ENOTTY; /* To be Implemented */

	default:
		return -ENOTTY;
	}

	return 0;
}

static int vfio_fsl_mc_config_irq_signal(struct vfio_fsl_mc_device *vdev,
					 int irq_index, int32_t fd)
{
	struct eventfd_ctx *trigger;
	struct vfio_fsl_mc_irq *mc_irq = &vdev->mc_irqs[irq_index];
	int ret;

	if (vdev->mc_irqs[irq_index].trigger) {
		eventfd_ctx_put(vdev->mc_irqs[irq_index].trigger);
		vdev->mc_irqs[irq_index].trigger = NULL;
	}

	if (fd < 0)
		return 0;

	trigger = eventfd_ctx_fdget(fd);
	if (IS_ERR(trigger))
		return PTR_ERR(trigger);

	/* If IRQ not configured the configure */
	if (!mc_irq->irq_configured) {
		ret = vfio_fsl_mc_configure_irq(vdev, irq_index);
		if (ret) {
			eventfd_ctx_put(trigger);
			return ret;
		}
	}

	vdev->mc_irqs[irq_index].trigger = trigger;
	return 0;
}

static int vfio_fsl_mc_set_irq_trigger(struct vfio_fsl_mc_device *vdev,
				    unsigned index, unsigned start,
				    unsigned count, uint32_t flags, void *data)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	int32_t fd;
	int hwirq;

	/* If count = 0 and DATA_NONE, disable interrupt */
	if (!count && (flags & VFIO_IRQ_SET_DATA_NONE)) {
		hwirq = mc_dev->irqs[index]->msi_desc->irq;
		disable_irq_nosync(hwirq);
		return 0;
	}

	if (flags & VFIO_IRQ_SET_DATA_BOOL)
		fd = *(int8_t *)data;
	else if (flags & VFIO_IRQ_SET_DATA_EVENTFD)
		fd = *(int32_t *)data;
	else
		return -EINVAL;

	if (start != 0 || count != 1)
		return -EINVAL;

	return vfio_fsl_mc_config_irq_signal(vdev, index, fd);
}

int vfio_fsl_mc_set_irqs_ioctl(struct vfio_fsl_mc_device *vdev,
			       uint32_t flags, unsigned index, unsigned start,
			       unsigned count, void *data)
{
	int ret = -ENOTTY;

	switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
	case VFIO_IRQ_SET_ACTION_MASK:
		/* mask all sources */
		ret = vfio_fsl_mc_irq_mask(vdev, index, start,
					       count, flags, data, 0);
		break;
	case VFIO_IRQ_SET_ACTION_UNMASK:
		/* unmask all sources */
		ret = vfio_fsl_mc_irq_mask(vdev, index, start,
					       count, flags, data, ~0);
		break;
	case VFIO_IRQ_SET_ACTION_TRIGGER:
		ret = vfio_fsl_mc_set_irq_trigger(vdev, index, start,
						  count, flags, data);
		break;
	}

	return ret;
}
