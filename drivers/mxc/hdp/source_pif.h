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
 * source_pif.h
 *
 ******************************************************************************
 */

#ifndef SOURCE_PIF_H_
#define SOURCE_PIF_H_

/* register SOURCE_PIF_WR_ADDR */
#define SOURCE_PIF_WR_ADDR 0
#define F_WR_ADDR(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_WR_ADDR_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register SOURCE_PIF_WR_REQ */
#define SOURCE_PIF_WR_REQ 1
#define F_HOST_WR(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HOST_WR_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SOURCE_PIF_RD_ADDR */
#define SOURCE_PIF_RD_ADDR 2
#define F_RD_ADDR(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_RD_ADDR_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)

/* register SOURCE_PIF_RD_REQ */
#define SOURCE_PIF_RD_REQ 3
#define F_HOST_RD(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HOST_RD_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SOURCE_PIF_DATA_WR */
#define SOURCE_PIF_DATA_WR 4
#define F_DATA_WR(x) (x)
#define F_DATA_WR_RD(x) (x)

/* register SOURCE_PIF_DATA_RD */
#define SOURCE_PIF_DATA_RD 5
#define F_FIFO2_DATA_OUT(x) (x)
#define F_FIFO2_DATA_OUT_RD(x) (x)

/* register SOURCE_PIF_FIFO1_FLUSH */
#define SOURCE_PIF_FIFO1_FLUSH 6
#define F_FIFO1_FLUSH(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_FIFO1_FLUSH_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SOURCE_PIF_FIFO2_FLUSH */
#define SOURCE_PIF_FIFO2_FLUSH 7
#define F_FIFO2_FLUSH(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_FIFO2_FLUSH_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SOURCE_PIF_STATUS */
#define SOURCE_PIF_STATUS 8
#define F_SOURCE_PKT_MEM_CTRL_FSM_STATE(x) (((x) & ((1 << 2) - 1)) << 0)
#define F_SOURCE_PKT_MEM_CTRL_FSM_STATE_RD(x) (((x) & (((1 << 2) - 1) << 0)) >> 0)
#define F_FIFO1_FULL(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_FIFO1_FULL_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_FIFO2_EMPTY(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_FIFO2_EMPTY_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)

/* register SOURCE_PIF_INTERRUPT_SOURCE */
#define SOURCE_PIF_INTERRUPT_SOURCE 9
#define F_HOST_WR_DONE_INT(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HOST_WR_DONE_INT_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_HOST_RD_DONE_INT(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_HOST_RD_DONE_INT_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_NONVALID_TYPE_REQUESTED_INT(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_NONVALID_TYPE_REQUESTED_INT_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_PSLVERR(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_PSLVERR_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_ALLOC_WR_DONE(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_ALLOC_WR_DONE_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_ALLOC_WR_ERROR(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_ALLOC_WR_ERROR_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_FIFO1_OVERFLOW(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_FIFO1_OVERFLOW_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_FIFO1_UNDERFLOW(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_FIFO1_UNDERFLOW_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_FIFO2_OVERFLOW(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_FIFO2_OVERFLOW_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_FIFO2_UNDERFLOW(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_FIFO2_UNDERFLOW_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)

/* register SOURCE_PIF_INTERRUPT_MASK */
#define SOURCE_PIF_INTERRUPT_MASK 10
#define F_HOST_WR_DONE_INT_MASK(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_HOST_WR_DONE_INT_MASK_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)
#define F_HOST_RD_DONE_INT_MASK(x) (((x) & ((1 << 1) - 1)) << 1)
#define F_HOST_RD_DONE_INT_MASK_RD(x) (((x) & (((1 << 1) - 1) << 1)) >> 1)
#define F_NONVALID_TYPE_REQUESTED_INT_MASK(x) (((x) & ((1 << 1) - 1)) << 2)
#define F_NONVALID_TYPE_REQUESTED_INT_MASK_RD(x) (((x) & (((1 << 1) - 1) << 2)) >> 2)
#define F_PSLVERR_MASK(x) (((x) & ((1 << 1) - 1)) << 3)
#define F_PSLVERR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 3)) >> 3)
#define F_ALLOC_WR_DONE_MASK(x) (((x) & ((1 << 1) - 1)) << 4)
#define F_ALLOC_WR_DONE_MASK_RD(x) (((x) & (((1 << 1) - 1) << 4)) >> 4)
#define F_ALLOC_WR_ERROR_MASK(x) (((x) & ((1 << 1) - 1)) << 5)
#define F_ALLOC_WR_ERROR_MASK_RD(x) (((x) & (((1 << 1) - 1) << 5)) >> 5)
#define F_FIFO1_OVERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 6)
#define F_FIFO1_OVERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 6)) >> 6)
#define F_FIFO1_UNDERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 7)
#define F_FIFO1_UNDERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 7)) >> 7)
#define F_FIFO2_OVERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 8)
#define F_FIFO2_OVERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 8)) >> 8)
#define F_FIFO2_UNDERFLOW_MASK(x) (((x) & ((1 << 1) - 1)) << 9)
#define F_FIFO2_UNDERFLOW_MASK_RD(x) (((x) & (((1 << 1) - 1) << 9)) >> 9)

/* register SOURCE_PIF_PKT_ALLOC_REG */
#define SOURCE_PIF_PKT_ALLOC_REG 11
#define F_PKT_ALLOC_ADDRESS(x) (((x) & ((1 << 4) - 1)) << 0)
#define F_PKT_ALLOC_ADDRESS_RD(x) (((x) & (((1 << 4) - 1) << 0)) >> 0)
#define F_PACKET_TYPE(x) (((x) & ((1 << 8) - 1)) << 8)
#define F_PACKET_TYPE_RD(x) (((x) & (((1 << 8) - 1) << 8)) >> 8)
#define F_TYPE_VALID(x) (((x) & ((1 << 1) - 1)) << 16)
#define F_TYPE_VALID_RD(x) (((x) & (((1 << 1) - 1) << 16)) >> 16)
#define F_ACTIVE_IDLE_TYPE(x) (((x) & ((1 << 1) - 1)) << 17)
#define F_ACTIVE_IDLE_TYPE_RD(x) (((x) & (((1 << 1) - 1) << 17)) >> 17)

/* register SOURCE_PIF_PKT_ALLOC_WR_EN */
#define SOURCE_PIF_PKT_ALLOC_WR_EN 12
#define F_PKT_ALLOC_WR_EN(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_PKT_ALLOC_WR_EN_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

/* register SOURCE_PIF_SW_RESET */
#define SOURCE_PIF_SW_RESET 13
#define F_SW_RST(x) (((x) & ((1 << 1) - 1)) << 0)
#define F_SW_RST_RD(x) (((x) & (((1 << 1) - 1) << 0)) >> 0)

#endif //SOURCE_PIF
