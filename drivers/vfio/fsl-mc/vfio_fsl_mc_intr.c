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
#include <linux/eventfd.h>
#include <linux/msi.h>

#include "linux/fsl/mc.h"
#include "vfio_fsl_mc_private.h"

static irqreturn_t vfio_fsl_mc_irq_handler(int irq_num, void *arg)
{
	struct vfio_fsl_mc_irq *mc_irq = (struct vfio_fsl_mc_irq *)arg;

	eventfd_signal(mc_irq->trigger, 1);
	return IRQ_HANDLED;
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

static int vfio_set_trigger(struct vfio_fsl_mc_device *vdev,
			    int index, int fd)
{
	struct vfio_fsl_mc_irq *irq = &vdev->mc_irqs[index];
	struct eventfd_ctx *trigger;
	int hwirq;
	int ret;

	hwirq = vdev->mc_dev->irqs[index]->msi_desc->irq;
	if (irq->trigger) {
		free_irq(hwirq, irq);
		kfree(irq->name);
		eventfd_ctx_put(irq->trigger);
		irq->trigger = NULL;
	}

	if (fd < 0) /* Disable only */
		return 0;

	irq->name = kasprintf(GFP_KERNEL, "vfio-irq[%d](%s)",
					hwirq, dev_name(&vdev->mc_dev->dev));
	if (!irq->name)
		return -ENOMEM;

	trigger = eventfd_ctx_fdget(fd);
	if (IS_ERR(trigger)) {
		kfree(irq->name);
		return PTR_ERR(trigger);
	}

	irq->trigger = trigger;

	ret = request_irq(hwirq, vfio_fsl_mc_irq_handler, 0,
			  irq->name, irq);
	if (ret) {
		kfree(irq->name);
		eventfd_ctx_put(trigger);
		irq->trigger = NULL;
		return ret;
	}

	return 0;
}

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
		mc_irq[i].flags = VFIO_IRQ_INFO_EVENTFD;
	}

	vdev->mc_irqs = mc_irq;

	return 0;
}

/* Free All IRQs for the given MC object */
void vfio_fsl_mc_irqs_cleanup(struct vfio_fsl_mc_device *vdev)
{
	struct fsl_mc_device *mc_dev = vdev->mc_dev;
	int irq_count = mc_dev->obj_desc.irq_count;
	int i;

	/* Device does not support any interrupt */
	if (mc_dev->obj_desc.irq_count == 0)
		return;

	for (i = 0; i < irq_count; i++)
		vfio_set_trigger(vdev, i, -1);

	fsl_mc_free_irqs(mc_dev);
	kfree(vdev->mc_irqs);
}

static int vfio_fsl_mc_set_irq_trigger(struct vfio_fsl_mc_device *vdev,
				       unsigned int index, unsigned int start,
				       unsigned int count, uint32_t flags,
				       void *data)
{
	struct vfio_fsl_mc_irq *irq = &vdev->mc_irqs[index];
	int hwirq;

	if (!count && (flags & VFIO_IRQ_SET_DATA_NONE))
		return vfio_set_trigger(vdev, index, -1);

	if (start != 0 || count != 1)
		return -EINVAL;

	if (flags & VFIO_IRQ_SET_DATA_EVENTFD) {
		int32_t fd = *(int32_t *)data;

		return vfio_set_trigger(vdev, index, fd);
	}

	hwirq = vdev->mc_dev->irqs[index]->msi_desc->irq;

	if (flags & VFIO_IRQ_SET_DATA_NONE) {
		vfio_fsl_mc_irq_handler(hwirq, irq);

	} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
		uint8_t trigger = *(uint8_t *)data;

		if (trigger)
			vfio_fsl_mc_irq_handler(hwirq, irq);
	}

	return 0;
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
