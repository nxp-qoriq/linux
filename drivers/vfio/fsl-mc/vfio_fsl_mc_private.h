/*
 * Freescale Management Complex VFIO private declarations
 *
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * Author: Bharat Bhushan <bharat.bhushan@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#ifndef VFIO_FSL_MC_PRIVATE_H
#define VFIO_FSL_MC_PRIVATE_H

#define VFIO_FSL_MC_OFFSET_SHIFT    40
#define VFIO_FSL_MC_OFFSET_MASK (((u64)(1) << VFIO_FSL_MC_OFFSET_SHIFT) - 1)

#define VFIO_FSL_MC_OFFSET_TO_INDEX(off) (off >> VFIO_FSL_MC_OFFSET_SHIFT)

#define VFIO_FSL_MC_INDEX_TO_OFFSET(index)	\
	((u64)(index) << VFIO_FSL_MC_OFFSET_SHIFT)

struct vfio_fsl_mc_irq {
	u32			flags;
	u32			count;
	struct eventfd_ctx	*trigger;
	char			*name;
};

struct vfio_fsl_mc_region {
	u32			flags;
#define VFIO_FSL_MC_REGION_TYPE_MMIO  1
#define VFIO_FSL_MC_REGION_TYPE_CACHEABLE  2
#define VFIO_FSL_MC_REGION_TYPE_SHAREABLE  4

	u32			type;
	u64			addr;
	resource_size_t		size;
	void __iomem		*ioaddr;
};

struct vfio_fsl_mc_device {
	struct fsl_mc_device		*mc_dev;
	int				refcnt;
	u32				num_regions;
	struct vfio_fsl_mc_region	*regions;
	struct vfio_fsl_mc_irq		*mc_irqs;
};

int vfio_fsl_mc_irqs_init(struct vfio_fsl_mc_device *vdev);
void vfio_fsl_mc_irqs_cleanup(struct vfio_fsl_mc_device *vdev);
int vfio_fsl_mc_set_irqs_ioctl(struct vfio_fsl_mc_device *vdev,
			       uint32_t flags, unsigned int index,
			       unsigned int start, unsigned int count,
			       void *data);
#endif /* VFIO_PCI_PRIVATE_H */
