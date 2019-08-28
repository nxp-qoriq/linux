/* SPDX-License-Identifier: (GPL-2.0 OR MIT) */
/* Copyright (c) 2017 Microsemi Corporation
 * Copyright 2019 NXP
 */

#ifndef _MSCC_OCELOT_PTP_H_
#define _MSCC_OCELOT_PTP_H_

#define PTP_PIN_CFG_RSZ			0x20
#define PTP_TOD_SEC_MSB_RSZ		PTP_PIN_CFG_RSZ
#define PTP_TOD_SEC_LSB_RSZ		PTP_PIN_CFG_RSZ
#define PTP_TOD_NSEC_RSZ		PTP_PIN_CFG_RSZ
#define PTP_NSF_RSZ			PTP_PIN_CFG_RSZ
#define PTP_PIN_WF_HIGH_PERIOD_RSZ	PTP_PIN_CFG_RSZ
#define PTP_PIN_WF_LOW_PERIOD_RSZ	PTP_PIN_CFG_RSZ

#define PTP_PIN_CFG_DOM			BIT(0)
#define PTP_PIN_CFG_SYNC		BIT(2)
#define PTP_PIN_CFG_ACTION(x)		((x) << 3)
#define PTP_PIN_CFG_ACTION_MASK		PTP_PIN_CFG_ACTION(0x7)

#define PTP_CLK_ADJ_DIR			BIT(1)
#define PTP_CLK_ADJ_ENA			BIT(0)

#define PTP_CLK_ADJ_UNIT_NS		BIT(30)

enum {
	PTP_PIN_ACTION_IDLE = 0,
	PTP_PIN_ACTION_LOAD,
	PTP_PIN_ACTION_SAVE,
	PTP_PIN_ACTION_CLOCK,
	PTP_PIN_ACTION_DELTA,
};

#define SEC_MSB_MASK           0x0000ffff
#define NSEC_MASK              0x3fffffff

// System clock period 6.4 ns (Frequency 156.25MHz)
#define SYS_CLK_PER_NS		0x6
#define SYS_CLK_PER_PS100	0x4

#define PSEC_PER_SEC		1000000000000LL
#endif
