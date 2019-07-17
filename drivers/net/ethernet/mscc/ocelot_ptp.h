/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/* Microsemi Ocelot Switch driver
 *
 * Copyright (c) 2017 Microsemi Corporation
 * Copyright 2018-2019 NXP
 */

#ifndef _MSCC_OCELOT_PTP_H_
#define _MSCC_OCELOT_PTP_H_

/* PTP_PINS register group */
#define PTP_PIN_CFG_RSZ			0x20
#define PTP_TOD_SEC_MSB_RSZ		PTP_PIN_CFG_RSZ
#define PTP_TOD_SEC_LSB_RSZ		PTP_PIN_CFG_RSZ
#define PTP_TOD_NSEC_RSZ		PTP_PIN_CFG_RSZ
#define PTP_NSF_RSZ			PTP_PIN_CFG_RSZ
#define PTP_PIN_WF_HIGH_PERIOD_RSZ	PTP_PIN_CFG_RSZ
#define PTP_PIN_WF_LOW_PERIOD_RSZ	PTP_PIN_CFG_RSZ

/* PTP_PIN_CFG register */
#define PTP_PIN_CFG_DOM			BIT(0)
#define PTP_PIN_CFG_SYNC		BIT(2)
#define PTP_PIN_CFG_ACTION(x)		((x) << 3)
#define PTP_PIN_CFG_ACTION_MASK		PTP_PIN_CFG_ACTION(0x7)

enum {
	PTP_PIN_ACTION_IDLE = 0,
	PTP_PIN_ACTION_LOAD,
	PTP_PIN_ACTION_SAVE,
	PTP_PIN_ACTION_CLOCK,
	PTP_PIN_ACTION_DELTA,
};

/* PTP_MISC_CFG register */
#define PTP_MISC_CFG_PTP_ENA		BIT(2)

/* PTP_CLK_ADJ_CFG register */
#define PTP_CLK_ADJ_CFG_ENA		BIT(0)
#define PTP_CLK_ADJ_CFG_DIR		BIT(1)

/* PTP_CLK_ADJ_FRQ register */
#define PTP_CLK_ADJ_FRQ_UNIT		BIT(30)

/* PTP_SYS_CLK_CFG register
 *
 * Default system clock period 6.4ns (Frequency 156.25MHz)
 */
#define PTP_SYS_CLK_CFG_PER_NS_DEFAULT		0x6
#define PTP_SYS_CLK_CFG_PER_NS_SHIFT		4
#define PTP_SYS_CLK_CFG_PER_PS100_DEFAULT	0x4

#define PSEC_PER_SEC			1000000000000LL

#define NSEC_MASK			0x3fffffff
#define SEC_MSB_MASK			0x0000ffff

#endif
