// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 *
 */

#ifndef _DT_BINDINGS_RPMSG_IMX_SRTM_H
#define _DT_BINDINGS_RPMSG_IMX_SRTM_H

/* Bit 0 as RPMSG Over UART flag */
#define IMX_SRTM_RPMSG_OVER_UART_FLAG (1 << 0)
#define IMX_SRTM_UART_SUPPORT_MULTI_UART_MSG_FLAG (1 << 1)
/* [15:11]: port number, such as /dev/ttySRTM3, 3 is the port number */
#define IMX_SRTM_UART_PORT_NUM_SHIFT (11U)
#define IMX_SRTM_UART_PORT_NUM_MASK (0x1F << 11U)
/* [10]: 0b1, specify port number; 0b0, not specify port number */
#define IMX_SRTM_UART_SPECIFY_PORT_NUM_SHIFT (10U)
#define IMX_SRTM_UART_SPECIFY_PORT_NUM_MASK (1 << IMX_SRTM_UART_SPECIFY_PORT_NUM_SHIFT)

#endif
