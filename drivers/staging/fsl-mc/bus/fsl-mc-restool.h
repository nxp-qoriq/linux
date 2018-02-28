/*
 * Freescale Management Complex (MC) bus driver
 *
 * Copyright (C) 2017 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#ifndef _FSL_MC_RESTOOL_H_
#define _FSL_MC_RESTOOL_H_

#include "fsl-mc-private.h"

#ifdef CONFIG_FSL_MC_RESTOOL

int fsl_mc_restool_create_device_file(struct fsl_mc_bus *mc_bus);

void fsl_mc_restool_remove_device_file(struct fsl_mc_bus *mc_bus);

int fsl_mc_restool_init(void);

#else

static inline int fsl_mc_restool_create_device_file(struct fsl_mc_bus *mc_bus)
{
	return 0;
}

static inline void fsl_mc_restool_remove_device_file(struct fsl_mc_bus *mc_bus)
{
}

static inline int fsl_mc_restool_init(void)
{
	return 0;
}

#endif

#endif /* _FSL_MC_RESTOOL_H_ */
