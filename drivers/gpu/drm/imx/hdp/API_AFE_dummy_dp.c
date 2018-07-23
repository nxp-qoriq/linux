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
 * API_AFE_dummy_dp.c
 *
 ******************************************************************************
 */

#include <linux/delay.h>
#include "API_AFE_dummy_dp.h"
#include "./cdn_hdp/all.h"

u8 AFE_check_rate_supported(ENUM_AFE_LINK_RATE rate)
{
	switch (rate) {
	case AFE_LINK_RATE_1_6:
	case AFE_LINK_RATE_2_7:
	case AFE_LINK_RATE_5_4:
	case AFE_LINK_RATE_8_1:
		return 1;
	default:
		return 0;
	}
}

void AFE_init(state_struct *state, int num_lanes,
	      ENUM_AFE_LINK_RATE link_rate)
{
	/*
	 * Internal reg Addr 2 controls line for link rate
	 * B16 (0x8----) enables control from regs / otherway dummy phy is controlled
	 * by state on the lines (This fearure is left for legacy test compatibility
	 */
	if (AFE_check_rate_supported(link_rate) == 0) {
		printk("%s *E: Selected link rate not supported: 0x%x\n",
		       __func__, link_rate);
		return;
	}

	switch (link_rate) {
	case AFE_LINK_RATE_1_6:
		Afe_write(state, 2, 0x8000);
		break;
	case AFE_LINK_RATE_2_7:
		Afe_write(state, 2, 0x8001);
		break;
	case AFE_LINK_RATE_5_4:
		Afe_write(state, 2, 0x8002);
		break;
	case AFE_LINK_RATE_8_1:
		Afe_write(state, 2, 0x8004);
		break;
	default:
		break;
	}
}

void AFE_power(state_struct *state, int num_lanes,
	       ENUM_AFE_LINK_RATE link_rate)
{

}
