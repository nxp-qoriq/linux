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
 * clock_meters.h
 *
 ******************************************************************************
 */

#ifndef CLOCK_METERS_H_
#define CLOCK_METERS_H_

/* register CM_CTRL */
#define CM_CTRL 0
#define F_NMVID_SEL_EXTERNAL(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_NMVID_SEL_EXTERNAL_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SPDIF_SEL_EXTERNAL(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SPDIF_SEL_EXTERNAL_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_I2S_SEL_EXTERNAL(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_I2S_SEL_EXTERNAL_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SEL_AUD_LANE_REF(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SEL_AUD_LANE_REF_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_I2S_MULT(x) (((x) & ((1 << 3) - 1)) << 4)
#define F_I2S_MULT_RD(x) (((x) & (((1 << 3) - 1) << 4)) >> 4)

/* register CM_I2S_CTRL */
#define CM_I2S_CTRL 1
#define F_I2S_REF_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_I2S_REF_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_I2S_MEAS_TOLERANCE(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_I2S_MEAS_TOLERANCE_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CM_SPDIF_CTRL */
#define CM_SPDIF_CTRL 2
#define F_SPDIF_REF_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_SPDIF_REF_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_SPDIF_MEAS_TOLERANCE(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_SPDIF_MEAS_TOLERANCE_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CM_VID_CTRL */
#define CM_VID_CTRL 3
#define F_NMVID_REF_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_NMVID_REF_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_NMVID_MEAS_TOLERANCE(x) (((x) & ((1 << 4) - 1)) << 24)
#define F_NMVID_MEAS_TOLERANCE_RD(x) (((x) & (((1 << 4) - 1) << 24)) >> 24)

/* register CM_LANE_CTRL */
#define CM_LANE_CTRL 4
#define F_LANE_REF_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_LANE_REF_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register I2S_NM_STABLE */
#define I2S_NM_STABLE 5
#define F_I2S_MNAUD_STABLE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_I2S_MNAUD_STABLE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register I2S_NCTS_STABLE */
#define I2S_NCTS_STABLE 6
#define F_I2S_NCTS_STABLE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_I2S_NCTS_STABLE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SPDIF_NM_STABLE */
#define SPDIF_NM_STABLE 7
#define F_SPDIF_MNAUD_STABLE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SPDIF_MNAUD_STABLE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SPDIF_NCTS_STABLE */
#define SPDIF_NCTS_STABLE 8
#define F_SPDIF_NCTS_STABLE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SPDIF_NCTS_STABLE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register NMVID_MEAS_STABLE */
#define NMVID_MEAS_STABLE 9
#define F_ST_NMVID_MEAS_STABLE(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_ST_NMVID_MEAS_STABLE_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register CM_VID_MEAS */
#define CM_VID_MEAS 10
#define F_NMVID_MEAS_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_NMVID_MEAS_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_NMVID_MEAS_VALID_INDC(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_NMVID_MEAS_VALID_INDC_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register CM_AUD_MEAS */
#define CM_AUD_MEAS 11
#define F_NMAUD_MEAS_CYC(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_NMAUD_MEAS_CYC_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)
#define F_NMAUD_MEAS_VALID_INDC(x) (((x) & ((1 << 1) - 1)) << 24)
#define F_NMAUD_MEAS_VALID_INDC_RD(x) (((x) & (((1 << 1) - 1) << 24)) >> 24)

/* register I2S_MEAS */
#define I2S_MEAS 16
#define F_I2_MEAS(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_I2_MEAS_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register I2S_DP_MEAS */
#define I2S_DP_MEAS 17
#define F_I2_DP_MEAS(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_I2_DP_MEAS_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register SPDIF_DP_MEAS */
#define SPDIF_DP_MEAS 32
#define F_SPDIF_DP_MEAS(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_SPDIF_DP_MEAS_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register SPDIF_MEAS */
#define SPDIF_MEAS 33
#define F_SPDIF_MEAS(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_SPDIF_MEAS_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

/* register NMVID_MEAS */
#define NMVID_MEAS 48
#define F_NMVID_MEAS(x) (((x) & ((1 << 24) - 1)) << 0)
#define F_NMVID_MEAS_RD(x) (((x) & (((1 << 24) - 1) << 0)) >> 0)

#endif //CLOCK_METERS
