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
 * mailBox.h
 *
 ******************************************************************************
 */

#ifndef MAIL_BOX_H
#define MAIL_BOX_H

#define MAIL_BOX_MAX_SIZE 600
#define MAIL_BOX_MAX_TX_SIZE 160

 /**
 * \file mailBox.h
 * \brief Implementation mail box communication channel between IP and external host
 */

#define MB_MODULE_ID_DP_TX          0x01
#define MB_MODULE_ID_DP_RX          0x02
#define MB_MODULE_ID_HDMI_TX        0x03
#define MB_MODULE_ID_HDMI_RX        0x04
#define MB_MODULE_ID_MHL_TX         0x05
#define MB_MODULE_ID_MHL_RX         0x06
#define MB_MODULE_ID_HDCP_TX        0x07
#define MB_MODULE_ID_HDCP_RX        0x08
#define MB_MODULE_ID_HDCP_GENERAL   0x09
#define MB_MODULE_ID_GENERAL        0x0A

typedef enum {
	MB_TYPE_REGULAR,
	MB_TYPE_SECURE,
	MB_TYPE_COUNT,
} MB_TYPE;

typedef enum {
	MB_SUCCESS,
	MB_BUSY,
	MB_NO_MEMORY
} MB_RET;

typedef enum {
	MB_TO_HOST,
	MB_TO_CONTROLLER,
} MB_IDX;

typedef enum {
	MB_STATE_EMPTY,
	MB_STATE_WAIT_MODULE_ID,
	MB_STATE_WAIT_SIZE_MSB,
	MB_STATE_WAIT_SIZE_LSB,
	MB_STATE_READ_DATA,
	MB_STATE_MSG_READY,
} MB_RX_STATE;

#define MB_OPCODE_ID 0
#define MB_MODULE_ID 1
#define MB_SIZE_MSB_ID 2
#define MB_SIZE_LSB_ID 3
#define MB_DATA_ID 4

typedef struct {
	MB_RX_STATE rxState;
	u32 rx_data_idx;
	u32 rx_final_msgSize;
	u8 rxBuff[MAIL_BOX_MAX_SIZE];
	u8 txBuff[MAIL_BOX_MAX_TX_SIZE];
	u32 txTotal;
	u32 txCur;
} S_MAIL_BOX_PORT_DATA;

typedef struct {
	S_MAIL_BOX_PORT_DATA portData;
	u8 portsTxBusy;		//bit for each port (0-7)
} S_MAIL_BOX_DATA;

#endif //MAIL_BOX_H
