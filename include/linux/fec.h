/* SPDX-License-Identifier: GPL-2.0-only */
/* include/linux/fec.h
 *
 * Copyright (c) 2009 Orex Computed Radiography
 *   Baruch Siach <baruch@tkos.co.il>
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 *
 * Header file for the FEC platform data
 */
#ifndef __LINUX_FEC_H__
#define __LINUX_FEC_H__

#include <linux/phy.h>

struct fec_platform_data {
	phy_interface_t phy;
	unsigned char mac[ETH_ALEN];
	void (*sleep_mode_enable)(int enabled);
};

/* The number of Tx and Rx buffers.  These are allocated from the page
 * pool.  The code may assume these are power of two, so it it best
 * to keep them that size.
 * We don't need to allocate pages for the transmitter.  We just use
 * the skbuffer directly.
 */

#define FEC_ENET_XDP_HEADROOM (XDP_PACKET_HEADROOM)
#define FEC_ENET_RX_PAGES	256
#define FEC_ENET_RX_FRSIZE    (PAGE_SIZE - FEC_ENET_XDP_HEADROOM \
		- SKB_DATA_ALIGN(sizeof(struct skb_shared_info)))
#define FEC_ENET_RX_FRPPG	(PAGE_SIZE / FEC_ENET_RX_FRSIZE)
#ifdef CONFIG_AVB_SUPPORT
#define FEC_RX_RING_SIZE	256
#define FEC_ENET_AVB_RX_FRSIZE	1522
#else
#define FEC_RX_RING_SIZE	(FEC_ENET_RX_FRPPG * FEC_ENET_RX_PAGES)
#endif
#define FEC_ENET_TX_FRSIZE	2048
#define FEC_ENET_TX_FRPPG	(PAGE_SIZE / FEC_ENET_TX_FRSIZE)

#ifdef CONFIG_AVB_SUPPORT
#define FEC_TX_RING_SIZE	128	/* Must be power of two */
#else
#define FEC_TX_RING_SIZE	1024	/* Must be power of two */
#endif

#ifdef CONFIG_AVB_SUPPORT
struct avb_desc {
	u16 offset;
	u16 len;
	u32 ts;
	u32 flags:28;
	u32 pool_type:4;
	u32 private; /* Will be used for saving userspace private value on TX and esc hw descriptor value on RX */
};

struct avb_tx_desc {
	struct avb_desc common;

	unsigned long dma_addr;
	void *data;
	u32 esc;
	unsigned short queue_id;
	unsigned short sc;
	unsigned long bufaddr;
	unsigned short datlen;
};

struct avb_rx_desc {
	struct avb_desc common;

	/* end of common rx fields */
	unsigned long dma_addr;
	unsigned short sc;	/* Control and status info */
	unsigned short queue_id;
};

#define AVB_WAKE_THREAD    (1 << 0)
#define AVB_WAKE_NAPI      (1 << 1)

#define AVB_TX_FLAG_SKB		(1 << 0)
#define AVB_TX_FLAG_HW_TS 	(1 << 1)
#define AVB_TX_FLAG_HW_CSUM 	(1 << 2)
#define AVB_TX_FLAG_TS	 	(1 << 3)

struct avb_ops {
	void (*open)(void *, void *, int);
	void (*close)(void *);

	void * (*alloc)(void *);
	void (*free)(void *, struct avb_desc *);

	int (*rx)(void *, struct avb_rx_desc *);
	void * (*dequeue)(void *);

	int (*tx)(void *, struct avb_tx_desc *);
	int (*tx_full)(void *);

	int (*tx_cleanup)(void *, struct avb_tx_desc *);
	int (*tx_cleanup_ready)(void *);
	void * (*tx_cleanup_dequeue)(void *);

	int (*tx_ts)(void *, struct avb_desc *);

	struct module *owner;
};

#define TX_QUEUE_FLAGS_STRICT_PRIORITY	BIT(0)
#define TX_QUEUE_FLAGS_CREDIT_SHAPER	BIT(1)

#define TX_QUEUE_PROP_MAX		8

struct tx_queue_property {
	unsigned int priority;
	unsigned int flags;
};

struct tx_queue_properties {
	int num_queues;
	struct tx_queue_property queue[TX_QUEUE_PROP_MAX];
};

int fec_enet_get_tx_queue_properties(int ifindex, struct tx_queue_properties *prop);
int fec_enet_avb_register(const char *ifname, const struct avb_ops *avb, void *data);
struct device *fec_enet_avb_get_device(const char *ifname);
int fec_enet_avb_unregister(int ifindex, const struct avb_ops *avb);
int fec_enet_rx_poll_avb(void *data);
int fec_enet_start_xmit_avb(void *data, struct avb_tx_desc *desc);
void fec_enet_finish_xmit_avb(void *data, unsigned int queue_id);
int fec_enet_tx_avb(void *data);

int fec_ptp_read_cnt(void *data, u32 *cnt);
int fec_ptp_tc_start(void *data, u8 id, u32 ts_0, u32 ts_1, u32 tcsr_val);
void fec_ptp_tc_stop(void *data, u8 id);
int fec_ptp_tc_reload(void *data, u8 id, u32 ts);

#endif

#endif
