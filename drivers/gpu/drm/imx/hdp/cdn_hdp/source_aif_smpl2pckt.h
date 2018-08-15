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
 * source_aif_smpl2pckt.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_AIF_SMPL2PCKT_H_
#define SOURCE_AIF_SMPL2PCKT_H_

/* register SMPL2PKT_CNTL */
#define SMPL2PKT_CNTL 0
#define F_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SMPL2PKT_EN(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SMPL2PKT_EN_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)

/* register SMPL2PKT_CNFG */
#define SMPL2PKT_CNFG 1
#define F_MAX_NUM_CH(x) (((x) & ((1 << 5) - 1)) << 0)
#define F_MAX_NUM_CH_RD(x) (((x) & (((1 << 5) - 1) << 0)) >> 0)
#define F_NUM_OF_I2S_PORTS_S(x) (((x) & ((1 << 2) - 1)) << 5)
#define F_NUM_OF_I2S_PORTS_RD_S(x) (((x) & (((1 << 2) - 1) << 5)) >> 5)
#define F_AUDIO_TYPE(x) (((x) & ((1 << 4) - 1)) << 7)
#define F_AUDIO_TYPE_RD(x) (((x) & (((1 << 4) - 1) << 7)) >> 7)
#define F_CFG_SUB_PCKT_NUM(x) (((x) & ((1 << 3) - 1)) << 11)
#define F_CFG_SUB_PCKT_NUM_RD(x) (((x) & (((1 << 3) - 1) << 11)) >> 11)
#define F_CFG_BLOCK_LPCM_FIRST_PKT(x) (((x) & ((1 << 1) - 1)) << 14)
#define F_CFG_BLOCK_LPCM_FIRST_PKT_RD(x) (((x) & (((1 << 1) - 1) << 14)) >> 14)
#define F_CFG_EN_AUTO_SUB_PCKT_NUM(x) (((x) & ((1 << 1) - 1)) << 15)
#define F_CFG_EN_AUTO_SUB_PCKT_NUM_RD(x) (((x) & (((1 << 1) - 1) << 15)) >> 15)
#define F_CFG_SAMPLE_PRESENT(x) (((x) & ((1 << 4) - 1)) << 16)
#define F_CFG_SAMPLE_PRESENT_RD(x) (((x) & (((1 << 4) - 1) << 16)) >> 16)
#define F_CFG_SAMPLE_PRESENT_FORCE(x) (((x) & ((1 << 1) - 1)) << 20)
#define F_CFG_SAMPLE_PRESENT_FORCE_RD(x) (((x) & (((1 << 1) - 1) << 20)) >> 20)

/* register FIFO_CNTL */
#define FIFO_CNTL 2
#define F_FIFO_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_FIFO_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_SYNC_WR_TO_CH_ZERO(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_SYNC_WR_TO_CH_ZERO_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_FIFO_DIR(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_FIFO_DIR_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_FIFO_EMPTY_CALC(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_FIFO_EMPTY_CALC_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_CFG_DIS_PORT3(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_CFG_DIS_PORT3_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)

/* register FIFO_STTS */
#define FIFO_STTS 3
#define F_WFULL(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_WFULL_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_REMPTY(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_REMPTY_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_OVERRUN(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_OVERRUN_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_UNDERRUN(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_UNDERRUN_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SUB_PCKT_THRSH */
#define SUB_PCKT_THRSH 4
#define F_CFG_MEM_FIFO_THRSH1(x) (((x) & ((1 << 8) - 1)) << 0)
#define F_CFG_MEM_FIFO_THRSH1_RD(x) (((x) & (((1 << 8) - 1) << 0)) >> 0)
#define F_CFG_MEM_FIFO_THRSH2(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_CFG_MEM_FIFO_THRSH2_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_CFG_MEM_FIFO_THRSH3(x) (((x) & ((1 << 8) - 1)) << 16)
#define F_CFG_MEM_FIFO_THRSH3_RD(x) (((x) & (((1 << 8) - 1) << 16)) >> 16)

#endif //SOURCE_AIF_SMPL2PCKT
