/*
 * Freescale Management Complex (MC) device passthrough using VFIO
 *
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/vfio.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "vfio_fsl_mc_private.h"
#include "../../staging/fsl-mc/include/mc.h"

int vfio_fsl_mc_irqs_init(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	struct vfio_fsl_mc_irq *mc_irq;
	int irq_count;
	int ret, i;

	/* Device does not support any interrupt */
	if (mc_dev->obj_desc.irq_count == 0)
		return 0;

	irq_count = mc_dev->obj_desc.irq_count;

	mc_irq = kcalloc(irq_count, sizeof(*mc_irq), GFP_KERNEL);
	if (mc_irq == NULL)
		return -ENOMEM;

	/* Allocate IRQs */
	ret = fsl_mc_allocate_irqs(mc_dev);
	if  (ret) {
		kfree(mc_irq);
		return ret;
	}

	for (i = 0; i < irq_count; i++) {
		mc_irq[i].count = 1;
		mc_irq[i].flags = 0;
	}

	vdev->mc_irqs = mc_irq;

	return 0;
}

/* Free All IRQs for the given MC object */
void vfio_fsl_mc_irqs_cleanup(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;

	/* Device does not support any interrupt */
	if (mc_dev->obj_desc.irq_count == 0)
		return;

	fsl_mc_free_irqs(mc_dev);
	kfree(vdev->mc_irqs);
}

static int vfio_fsl_mc_irq_mask(struct vfio_fsl_mc_device *vdev,
				unsigned int index, unsigned int start,
				unsigned int count, uint32_t flags,
				void *data)
{
	return -EINVAL;
}

static int vfio_fsl_mc_irq_unmask(struct vfio_fsl_mc_device *vdev,
				unsigned int index, unsigned int start,
				unsigned int count, uint32_t flags,
				void *data)
{
	return -EINVAL;
}

static int vfio_fsl_mc_set_irq_trigger(struct vfio_fsl_mc_device *vdev,
				       unsigned int index, unsigned int start,
				       unsigned int count, uint32_t flags,
				       void *data)
{
	return -EINVAL;
}

int vfio_fsl_mc_set_irqs_ioctl(struct vfio_fsl_mc_device *vdev,
			       uint32_t flags, unsigned int index,
			       unsigned int start, unsigned int count,
			       void *data)
{
	int ret = -ENOTTY;

	switch (flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) {
	case VFIO_IRQ_SET_ACTION_MASK:
		ret = vfio_fsl_mc_irq_mask(vdev, index, start, count,
					   flags, data);
		break;
	case VFIO_IRQ_SET_ACTION_UNMASK:
		ret = vfio_fsl_mc_irq_unmask(vdev, index, start, count,
					     flags, data);
		break;
	case VFIO_IRQ_SET_ACTION_TRIGGER:
		ret = vfio_fsl_mc_set_irq_trigger(vdev, index, start,
						  count, flags, data);
		break;
	}

	return ret;
}
