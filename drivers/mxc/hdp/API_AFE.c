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
 * API_AFE.c
 *
 ******************************************************************************
 */

#include "address.h"
#include "API_AFE.h"
#include "API_General.h"

void Afe_write(state_struct *state, u32 offset, u16 val)
{
	CDN_API_STATUS sts;

	sts =
	    CDN_API_General_Write_Register_blocking(state,
						    ADDR_AFE + (offset << 2),
						    val);

	if (sts != CDN_OK) {
		pr_err
		    ("CDN_API_General_Write_Register_blocking(0x%.8X, 0x%.8X) returned %d\n",
		     offset, val, (int)sts);
	}
}

u16 Afe_read(state_struct *state, u32 offset)
{
	GENERAL_Read_Register_response resp;
	CDN_API_STATUS sts;

	sts =
	    CDN_API_General_Read_Register_blocking(state,
						   ADDR_AFE + (offset << 2),
						   &resp);

	if (sts != CDN_OK) {
		pr_err
		    ("CDN_API_General_Read_Register_blocking(0x%.8X) returned %d\n",
		     offset, (int)sts);
	}
	return resp.val;
}

void set_field_value(reg_field_t *reg_field, u32 value)
{
	u8 length;
	u32 max_value;
	u32 trunc_val;

	length = (reg_field->msb - reg_field->lsb + 1);

	max_value = (1 << length) - 1;
	if (value > max_value) {
		trunc_val = value;
		trunc_val &= (1 << length) - 1;
		pr_err("set_field_value() Error! Specified value (0x%0X)\
				exceeds field capacity - it will by truncated to\
				0x%0X (%0d-bit field - max value: %0d dec)\n",
				value, trunc_val, length, max_value);
	} else {
		pr_info("set_field_value() Setting field to 0x%0X\n", value);
		reg_field->value = value;
	}
}

int set_reg_value(reg_field_t reg_field)
{
	return reg_field.value << reg_field.lsb;
}

u8 AFE_check_rate_supported(ENUM_AFE_LINK_RATE rate)
{
	switch (rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_1:
	case AFE_LINK_RATE_2_4:
	case AFE_LINK_RATE_2_7:
	case AFE_LINK_RATE_3_2:
	case AFE_LINK_RATE_4_3:
	case AFE_LINK_RATE_5_4:
		return 1;
	default:
		return 0;
	}
}
