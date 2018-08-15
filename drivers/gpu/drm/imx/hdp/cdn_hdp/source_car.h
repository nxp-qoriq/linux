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
 * source_car.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_CAR_H_
#define SOURCE_CAR_H_

/* register SOURCE_HDTX_CAR */
#define SOURCE_HDTX_CAR 0
#define F_HDTX_PIXEL_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HDTX_PIXEL_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_HDTX_PIXEL_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_HDTX_PIXEL_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_HDTX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_HDTX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_HDTX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_HDTX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_HDTX_PHY_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_HDTX_PHY_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_HDTX_PHY_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_HDTX_PHY_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_HDTX_PHY_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_HDTX_PHY_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_HDTX_PHY_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_HDTX_PHY_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register SOURCE_DPTX_CAR */
#define SOURCE_DPTX_CAR 1
#define F_CFG_DPTX_VIF_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_CFG_DPTX_VIF_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_CFG_DPTX_VIF_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_CFG_DPTX_VIF_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_DPTX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_DPTX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_DPTX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_DPTX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SOURCE_AUX_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SOURCE_AUX_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SOURCE_AUX_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SOURCE_AUX_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_DPTX_PHY_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_DPTX_PHY_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_DPTX_PHY_CHAR_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_DPTX_PHY_CHAR_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_DPTX_PHY_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_DPTX_PHY_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_DPTX_PHY_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_DPTX_PHY_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)
#define F_DPTX_FRMR_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 10)
#define F_DPTX_FRMR_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 10)) >> 10)
#define F_DPTX_FRMR_DATA_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 11)
#define F_DPTX_FRMR_DATA_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 11)) >> 11)

/* register SOURCE_PHY_CAR */
#define SOURCE_PHY_CAR 2
#define F_SOURCE_PHY_DATA_OUT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_PHY_DATA_OUT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_PHY_DATA_OUT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_PHY_DATA_OUT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_PHY_CHAR_OUT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_PHY_CHAR_OUT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_PHY_CHAR_OUT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_PHY_CHAR_OUT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_CEC_CAR */
#define SOURCE_CEC_CAR 3
#define F_SOURCE_CEC_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CEC_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CEC_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CEC_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SOURCE_CBUS_CAR */
#define SOURCE_CBUS_CAR 4
#define F_SOURCE_CBUS_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CBUS_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CBUS_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CBUS_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SOURCE_PKT_CAR */
#define SOURCE_PKT_CAR 6
#define F_SOURCE_PKT_DATA_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_PKT_DATA_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_PKT_DATA_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_PKT_DATA_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_PKT_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_PKT_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_PKT_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_PKT_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_AIF_CAR */
#define SOURCE_AIF_CAR 7
#define F_SOURCE_AIF_PKT_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_AIF_PKT_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_AIF_PKT_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_AIF_PKT_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_AIF_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_AIF_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_AIF_SYS_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_AIF_SYS_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_SPDIF_CDR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_SPDIF_CDR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_SPDIF_CDR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_SPDIF_CDR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_SPDIF_MCLK_EN(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_SPDIF_MCLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_SPDIF_MCLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_SPDIF_MCLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)

/* register SOURCE_CIPHER_CAR */
#define SOURCE_CIPHER_CAR 8
#define F_SOURCE_CIPHER_CHAR_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CIPHER_CHAR_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CIPHER_CHAR_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CIPHER_CHAR_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_SOURCE_CIPHER_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_SOURCE_CIPHER_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_CRYPTO_CAR */
#define SOURCE_CRYPTO_CAR 9
#define F_SOURCE_CRYPTO_SYS_CLK_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SOURCE_CRYPTO_SYS_CLK_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SOURCE_CRYPTO_SYS_CLK_RSTN_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SOURCE_CRYPTO_SYS_CLK_RSTN_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

#endif //SOURCE_CAR
