/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright 2022 NXP
 */

#ifndef _LINUX_VIRTIO_TRANS_H
#define _LINUX_VIRTIO_TRANS_H

#include <linux/types.h>

/*
 * status:
 *	bit0: TX ready
 *	bit1: RX ready
 * config:
 *	bit0: TX
 *	bit1: RX
 *	bit2: 0: Backend do NOT copy buffer
 *	      1: Backend do copy buffer
 *	bit3: 0: Backend interrupt mode
 *	      1: Backend polling mode
 *	bit4: 0: Frontend interrupt mode
 *	      1: Frontend polling mode
 * control:
 *	bit0: start transfer
 *	bit1: reset
 * pkt_size: packet size in Byte
 * tx_count: Completed count for TX packet, update by Backend
 * rx_count: Completed count for RX packet, update by Backend
 * regression: Packets to test, set by Frontend
 */

#define VT_STATUS	0x0
#define VT_CONFIG	0x4
#define VT_CONTROL	0x8
#define VT_PKT_SIZE	0xc
#define VT_TX_COUNT	0x10
#define VT_RX_COUNT	0x14
#define VT_REGRESSION	0x18

#define VT_CFG_TX	BIT(0)
#define VT_CFG_RX	BIT(1)
#define VT_CFG_COPY	BIT(2)
#define VT_CFG_B_POLL	BIT(3)
#define VT_CFG_F_POLL	BIT(4)

#define VT_CTRL_START	BIT(0)
#define VT_CTRL_RESET	BIT(1)

struct virtio_trans_config {
	__le32 status;
	__le32 config;
	__le32 control;
	__le32 pkt_size;
	__le32 tx_count;
	__le32 rx_count;
	__le32 regression;
};

#endif /* _LINUX_VIRTIO_TRANS_H */
