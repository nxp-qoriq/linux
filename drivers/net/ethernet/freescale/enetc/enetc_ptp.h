/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2018 NXP */

#include "enetc.h"

#define ENETC_DEV_ID_PTP	0xee02

/* Registers definition */
#define TMR_ID			0x0
#define TMR_ID_2		0x4
#define TMR_CTRL		0x80
#define TMR_TEVENT		0x84
#define TMR_TEMASK		0x88
#define TMR_MSIVEC		0x8c
#define TMR_STAT		0x94
#define TMR_CNT_H		0x98
#define TMR_CNT_L		0x9c
#define TMR_ADD			0xa0
#define TMR_ACC			0xa4
#define TMR_PRSC		0xa8
#define TMR_ECTRL		0xac
#define TMR_OFF_H		0xb0
#define TMR_OFF_L		0xb4
#define TMR_ALARM_1_H		0xb8
#define TMR_ALARM_1_L		0xbc
#define TMR_ALARM_2_H		0xc0
#define TMR_ALARM_2_L		0xc4
#define TMR_FIPER_1		0xd0
#define TMR_FIPER_2		0xd4
#define TMR_FIPER_3		0xd8
#define TMR_ETTS_1_H		0xe0
#define TMR_ETTS_1_L		0xe4
#define TMR_ETTS_2_H		0xe8
#define TMR_ETTS_2_L		0xec
#define TMR_CUR_TIME_H		0xf0
#define TMR_CUR_TIME_L		0xf4

/* Bits definition for the TMR_CTRL register */
#define FS			BIT(28)
#define FRD			BIT(14)
#define TE			BIT(2)
#define TCLK_PERIOD_MASK	0x3ff
#define TCLK_PERIOD_SHIFT	16
#define CK_SEL_MASK		0x3

/* Bits definition for the TMR_TEMASK register */
#define ETS2EN                BIT(25)
#define ETS1EN                BIT(24)
#define PP1EN                 BIT(7)
#define PP2EN                 BIT(6)
#define PP3EN                 BIT(5)
