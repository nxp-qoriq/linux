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

struct vfio_fsl_mc_region {
	u32			flags;
#define VFIO_FSL_MC_REGION_TYPE_MMIO  1
#define VFIO_FSL_MC_REGION_TYPE_CACHEABLE  2
	u32			type;
	u64			addr;
	resource_size_t		size;
};

struct vfio_fsl_mc_device {
	struct fsl_mc_device		*mc_dev;
	int				refcnt;
	u32				num_regions;
	struct vfio_fsl_mc_region	*regions;
};

#endif /* VFIO_PCI_PRIVATE_H */
