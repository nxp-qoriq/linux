/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 *
 * API_AFE.h
 *
 ******************************************************************************
 */

#ifndef API_AFE_H_
#define API_AFE_H_

#include "util.h"

typedef enum {
	AFE_LINK_RATE_1_6 = 0x6,
	AFE_LINK_RATE_2_1 = 0x8,
	AFE_LINK_RATE_2_4 = 0x9,
	AFE_LINK_RATE_2_7 = 0xA,
	AFE_LINK_RATE_3_2 = 0xC,
	AFE_LINK_RATE_4_3 = 0x10,
	AFE_LINK_RATE_5_4 = 0x14,
	AFE_LINK_RATE_8_1 = 0x1A,
} ENUM_AFE_LINK_RATE;

/*
 * Some of the PHY programming sequences
 * depend on the reference clock frequency.
 * Variable of this type is used to control
 * the programming flow.
 */
typedef enum {
	REFCLK_24MHZ,
	REFCLK_27MHZ
} REFCLK_FREQ;

typedef enum {
	CLK_RATIO_1_1,
	CLK_RATIO_5_4,
	CLK_RATIO_3_2,
	CLK_RATIO_2_1,
	CLK_RATIO_1_2,
	CLK_RATIO_5_8,
	CLK_RATIO_3_4
} clk_ratio_t;

typedef struct {
	u32 value;
	u8 lsb;
	u8 msb;
} reg_field_t;

unsigned char AFE_check_rate_supported(ENUM_AFE_LINK_RATE rate);
void Afe_write(state_struct *state, u32 offset, u16 val);
u16 Afe_read(state_struct *state, u32 offset);
void AFE_init(state_struct *state, int num_lanes,
	      ENUM_AFE_LINK_RATE link_rate);
void afe_init_t28hpc(state_struct *state, int num_lanes,
	      ENUM_AFE_LINK_RATE link_rate);
void AFE_power(state_struct *state, int num_lanes,
	       ENUM_AFE_LINK_RATE link_rate);
void afe_power_t28hpc(state_struct *state, int num_lanes,
	       ENUM_AFE_LINK_RATE link_rate);
void set_field_value(reg_field_t *reg_field, u32 value);
int set_reg_value(reg_field_t reg_field);

#endif
