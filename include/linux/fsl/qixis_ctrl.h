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
	u8		id;		/* Identification Registers */
	u8		version;	/* Version Register */
	u8		qixis_ver;	/* QIXIS Version Register */
	u8		reserved1[0x1e];
	u8		pwr_ctrl2;	/* Power Control 2 Register */
};

#endif
