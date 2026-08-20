// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#ifndef __STUB_USBIP_PCIE_TRANSPORT_H
#define __STUB_USBIP_PCIE_TRANSPORT_H

#include <linux/socket.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/uio.h>
#include <linux/pci-epf-usbip.h>

/*
 * One queued physical PCIe packet for a logical per-devid channel.
 */
struct usbip_pcie_rx_node {
	struct list_head list;
	void *buf;
	u32 len;
	u32 off;
};

/*
 * Virtual PCIe stream for one exported USB/IP device.
 *
 * This intentionally does not expose struct stub_device. The owner pointer is
 * opaque to the PCIe glue layer.
 */
struct usbip_pcie_channel {
	struct list_head node;
	struct usbip_pcie_ctx *parent;

	void *owner;
	__u32 devid;

	struct list_head rx_queue;
	spinlock_t rx_lock;
	wait_queue_head_t rx_wait;

	bool shutting_down;
};

struct usbip_pcie_ctx *usbip_stub_get_ctx(void);
void usbip_stub_set_ctx(struct usbip_pcie_ctx *ctx);
void usbip_pcie_channel_unregister(struct usbip_pcie_channel *ch);
int usbip_pcie_recv_stream(struct usbip_pcie_channel *ch, void *buf, int size);
int usbip_xmit_pcie(struct usbip_pcie_channel *ch, struct msghdr *msg, size_t txsize);
struct usbip_pcie_channel *usbip_pcie_channel_register(struct usbip_pcie_ctx *ctx, void *owner, __u32 devid);

#endif /* __STUB_USBIP_PCIE_TRANSPORT_H */
