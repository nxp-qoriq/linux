/*
 * include/linux/fsl/qixis_ctrl.h
 *
 * Definitions for Freescale QIXIS system controller.
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#ifndef _FSL_QIXIS_CTRL_H_
#define _FSL_QIXIS_CTRL_H_

/* QIXIS MAP */
struct fsl_qixis_regs {
	u8		version;	/* Identification Registers */
	u8		reserved1[0x20];
	u8		pwr_ctrl2;	/* Power Control 2 Register */
};

#endif
