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
 * This file was auto-generated. Do not edit it manually.
 *
 ******************************************************************************
 *
 * vic_table.c
 *
 ******************************************************************************
 */
#include "vic_table.h"

const unsigned int vic_table[VIC_MODE_COUNT][27] = {
	{858, 720, 138, 62, 16, 60, 525, 480, 45, 6, 9, 30, 59, 27000,
	 PROGRESSIVE, ACTIVE_LOW, ACTIVE_LOW, 1, 65535, 1, 46, 65535, 65535, 3,
	 8, 0},
	{1650, 1280, 370, 40, 110, 220, 750, 720, 30, 5, 5, 20, 60, 74250,
	 PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 1, 65535, 1, 31, 65535, 65535,
	 4, 8, 0},
	{2200, 1920, 280, 44, 88, 148, 1125, 1080, 45, 5, 4,
	 36, 60, 148500, PROGRESSIVE, ACTIVE_HIGH,
	 ACTIVE_HIGH, 1, 65535, 1, 46, 65535, 65535, 16, 8, 0},
	{4400, 3840, 560, 88, 176, 296, 2250, 2160, 90, 10, 8, 72, 60,
	 594000, PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 4, 266, 262, 22, 525,
	 285, 97, 8, 0},
	{4400, 3840, 560, 88, 176, 296, 2250, 2160, 90, 10, 8, 72, 30,
	 297000, PROGRESSIVE, ACTIVE_HIGH, ACTIVE_HIGH, 4, 266, 262, 22, 525,
	 285, 95, 8, 0},
};
