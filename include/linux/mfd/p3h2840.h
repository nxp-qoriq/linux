/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025-2026 NXP
 */

#ifndef _LINUX_MFD_P3H2840_H
#define _LINUX_MFD_P3H2840_H

#include <linux/bits.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* Device Information Registers */
#define P3H2X4X_DEV_CAPAB					0x0a
#define P3H2X4X_TARGET_PORT_COUNT				BIT(3)

/* Downstream target port counts per variant. */
#define P3H2X4X_TARGET_PORTS_4					4
#define P3H2X4X_TARGET_PORTS_8					8

/* Device Configuration Registers */
#define P3H2X4X_DEV_REG_PROTECTION_CODE				0x10
#define P3H2X4X_REGISTERS_LOCK_CODE				0x00
#define P3H2X4X_REGISTERS_UNLOCK_CODE				0x69
#define P3H2X4X_CP1_REGISTERS_UNLOCK_CODE			0x6a

#define I3C_MANUF_ID_NXP					0x011b

struct p3h2x4x_i3c_hub_dev;

struct p3h2x4x {
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	/* Number of downstream target ports (4 or 8). */
	u8 num_target_ports;
	/* Serializes protected register unlock/lock sequences across MFD children. */
	struct mutex protected_reg_lock;
	/* Hub context for the IBI handler to reach hub state via the parent i3cdev. */
	struct p3h2x4x_i3c_hub_dev *i3c_hub_priv;
};
#endif /* _LINUX_MFD_P3H2840_H */
