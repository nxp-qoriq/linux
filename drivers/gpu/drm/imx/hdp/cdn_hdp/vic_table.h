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
 * vic_table.h
 *
 ******************************************************************************
 */

#ifndef VIC_TABLE_H_
#define VIC_TABLE_H_

#define PROGRESSIVE 0
#define INTERLACED 1

#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1

typedef enum {
	H_TOTAL,
	H_ACTIVE,
	H_BLANK,
	HSYNC,
	FRONT_PORCH,
	BACK_PORCH,
	/* H_FREQ_KHZ, */
	V_TOTAL,
	V_ACTIVE,
	V_BLANK,
	VSYNC,
	TYPE_EOF,
	SOF,
	V_FREQ_HZ,
	PIXEL_FREQ_KHZ,
	I_P,
	HSYNC_POL,
	VSYNC_POL,
	START_OF_F0,
	START_OF_F1,
	VSYNC_START_INTERLACED_F0,
	VSYNC_END_INTERLACED_F0,
	VSYNC_START_INTERLACED_F1,
	VSYNC_END_INTERLACED_F1,
	VIC,
	VIC_R3_0,
	VIC_PR,
} MSA_PARAM;

typedef enum {
	NUM_OF_LANES_1 = 1,
	NUM_OF_LANES_2 = 2,
	NUM_OF_LANES_4 = 4,
} VIC_NUM_OF_LANES;

typedef enum {
	RATE_1_6 = 162,
	RATE_2_7 = 270,
	RATE_5_4 = 540,
	RATE_8_1 = 810,
} VIC_SYMBOL_RATE;

typedef enum {
	PXL_RGB = 0x1,
	YCBCR_4_4_4 = 0x2,
	YCBCR_4_2_2 = 0x4,
	YCBCR_4_2_0 = 0x8,
	Y_ONLY = 0x10,
} VIC_PXL_ENCODING_FORMAT;

typedef enum {
	BCS_6 = 0x1,
	BCS_8 = 0x2,
	BCS_10 = 0x4,
	BCS_12 = 0x8,
	BCS_16 = 0x10,
} VIC_COLOR_DEPTH;

typedef enum {
	STEREO_VIDEO_LEFT = 0x0,
	STEREO_VIDEO_RIGHT = 0x1,
} STEREO_VIDEO_ATTR;

typedef enum {
	BT_601 = 0x0,
	BT_709 = 0x1,
} BT_TYPE;

typedef enum {
	VIC_MODE_3_59_94Hz,
	VIC_MODE_4_60Hz,
	VIC_MODE_16_60Hz,
	VIC_MODE_97_60Hz,
	VIC_MODE_95_30Hz,
	VIC_MODE_COUNT
} VIC_MODES;

extern const unsigned int vic_table[VIC_MODE_COUNT][27];

#endif
