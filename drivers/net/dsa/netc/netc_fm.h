/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2025 NXP
 */

#ifndef __NETC_FM_H
#define __NETC_FM_H

#include <linux/bitops.h>
#include <linux/ioctl.h>

struct fm_filter {
	__le16 ethertype;
	__le16 rev1;
};

/* this is the structure of the argument of the IOCTL NETC_FM_CMD_CREATE command */
struct netc_fm_conf {
	__le32 fm_action;       /* specify what action the Frame Modification will do */
	__le32 flags;
	__le16 ingress_port;    /* the frames flow into the switch from this port */
	__le16 egress_port;     /* the frames leave the switch through this port */
	struct fm_filter filter;    /* the filter parameters */
	/* the offset of the Message Count field in UADP NetworkMessage */
	__le16 message_count_offset;
	__le16 rev1;
};

/* IOCTL command number */
#define NETC_FM_CMD_CREATE      _IOW(0xE2, 0xC0, struct netc_fm_conf)
#define NETC_FM_CMD_DESTROY     _IO(0xE2, 0xC1)

#endif
