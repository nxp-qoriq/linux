/*
 * Freescale Management Complex VFIO private declarations
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 * Author: Bharat Bhushan <Bharat.Bhushan@freescale.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include "../../staging/fsl-mc/include/mc.h"

#ifndef VFIO_FSL_MC_PRIVATE_H
#define VFIO_FSL_MC_PRIVATE_H

struct vfio_fsl_mc_irq {
	struct eventfd_ctx	*trigger;
	u32			flags;
	u32			count;
	char			*name;
	bool			irq_initialized;
	bool			irq_configured;
};

struct vfio_fsl_mc_device {
	struct fsl_mc_device	*mc_dev;
	int			refcnt;
	struct vfio_fsl_mc_irq	*mc_irqs;
};

int vfio_fsl_mc_init_irqs(struct vfio_fsl_mc_device *vdev);

void vfio_fsl_mc_free_irqs(struct vfio_fsl_mc_device *vdev);

int vfio_fsl_mc_configure_irq(struct vfio_fsl_mc_device *vdev, int irq_idx);

int vfio_fsl_mc_unconfigure_irqs(struct vfio_fsl_mc_device *vdev);

int vfio_fsl_mc_set_irqs_ioctl(struct vfio_fsl_mc_device *vdev,
			       uint32_t flags, unsigned index, unsigned start,
			       unsigned count, void *data);
#endif /* VFIO_PCI_PRIVATE_H */
