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
