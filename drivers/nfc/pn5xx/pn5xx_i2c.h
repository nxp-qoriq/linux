// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * Copyright 2015,2019-2023 NXP
 *
 */

#define PN544_MAGIC	0xE9

/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled
 */

#define PWR_OFF 0
#define PWR_ON  1
#define PWR_FW  2

#define CLK_OFF 0
#define CLK_ON  1

#define GPIO_UNUSED -1

#define PN544_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)
#define PN54X_CLK_REQ	_IOW(PN544_MAGIC, 0x02, unsigned int)

struct pn544_i2c_platform_data {
	unsigned int irq_gpio;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
	unsigned int clkreq_gpio;
	struct regulator *pvdd_reg;
	struct regulator *vbat_reg;
	struct regulator *pmuvcc_reg;
	struct regulator *sevdd_reg;
};
