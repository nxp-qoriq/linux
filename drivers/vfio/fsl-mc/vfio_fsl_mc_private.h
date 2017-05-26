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

struct vfio_fsl_mc_device {
	struct fsl_mc_device		*mc_dev;
};

#endif /* VFIO_PCI_PRIVATE_H */
