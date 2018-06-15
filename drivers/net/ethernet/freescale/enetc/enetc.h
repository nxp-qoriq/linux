/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/timer.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>

#include "enetc_hw.h"

#define ENETC_MAC_MAXFRM_SIZE	9600
#define ENETC_MAX_MTU		(ENETC_MAC_MAXFRM_SIZE - \
				(ETH_FCS_LEN + ETH_HLEN + VLAN_HLEN))

struct enetc_tx_swbd {
	struct sk_buff *skb;
	dma_addr_t dma;
	u16 len;
	u16 is_dma_page;
};

#define ENETC_RX_MAXFRM_SIZE	ENETC_MAC_MAXFRM_SIZE
#define ENETC_RXB_TRUESIZE	2048 // PAGE_SIZE >> 1
#define ENETC_RXB_PAD		NET_SKB_PAD // TODO: extra space? IP_ALIGN?
#define ENETC_RXB_DMA_SIZE	\
	(SKB_WITH_OVERHEAD(ENETC_RXB_TRUESIZE) - ENETC_RXB_PAD)

struct enetc_rx_swbd {
	dma_addr_t dma;
	struct page *page;
	u16 page_offset;
};

struct enetc_ring_stats {
	unsigned int packets;
	unsigned int bytes;
};

struct enetc_bdr {
	struct device *dev; /* for DMA mapping */
	struct net_device *ndev;
	void *bd_base; /* points to Rx or Tx BD ring */
	union {
		void __iomem *tcir;
		void __iomem *rcir;
	};
	u16 index;
	int bd_count; /* # of BDs */
	int next_to_use;
	int next_to_clean;
	union {
		struct enetc_tx_swbd *tx_swbd;
		struct enetc_rx_swbd *rx_swbd;
	};
	union {
		void __iomem *tcisr; /* Tx */
		int next_to_alloc; /* Rx */
	};
	void __iomem *idr; /* Interrupt Detect Register pointer */

	struct enetc_ring_stats stats;

	dma_addr_t bd_dma_base;
} ____cacheline_aligned_in_smp;

static inline void enetc_bdr_idx_inc(struct enetc_bdr *bdr, int *i)
{
	if (unlikely(++*i == bdr->bd_count))
		*i = 0;
}

static inline int enetc_bd_unused(struct enetc_bdr *bdr)
{
	if (bdr->next_to_clean > bdr->next_to_use)
		return bdr->next_to_clean - bdr->next_to_use - 1;

	return bdr->bd_count + bdr->next_to_clean - bdr->next_to_use - 1;
}

/* Control BD ring */
struct enetc_cbdr {
	void *bd_base; /* points to Rx or Tx BD ring */
	void __iomem *cir;
	void __iomem *cisr;

	int bd_count; /* # of BDs */
	int next_to_use;
	int next_to_clean;

	dma_addr_t bd_dma_base;
};

#define ENETC_TXBD(BDR, i) (&(((union enetc_tx_bd *)((BDR).bd_base))[i]))
#define ENETC_RXBD(BDR, i) (&(((union enetc_rx_bd *)((BDR).bd_base))[i]))

struct enetc_msg_swbd {
	void *vaddr;
	dma_addr_t dma;
	int size;
};

/* PCI IEP device data */
struct enetc_si {
	struct pci_dev *pdev;
	struct enetc_hw hw;

	struct net_device *ndev; /* back ref. */

	struct enetc_cbdr cbd_ring;

	int num_rx_rings; /* how many rings are available in the SI */
	int num_tx_rings;
	int num_fs_entries;
	unsigned short pad;
};

#define ENETC_SI_ALIGN	32

static inline void *enetc_si_priv(const struct enetc_si *si)
{
	return (char *)si + ALIGN(sizeof(struct enetc_si), ENETC_SI_ALIGN);
}

static inline bool enetc_si_is_pf(struct enetc_si *si)
{
	return !!(si->hw.port);
}

#define ENETC_MAX_TCS		8
#define ENETC_TXQ_PER_TC	num_online_cpus()
#define ENETC_MAX_NUM_TXQS	(ENETC_MAX_TCS * ENETC_TXQ_PER_TC)

struct enetc_int_vector {
	void __iomem *tbier;
	void __iomem *rbier;
	struct napi_struct napi;
	char name[IFNAMSIZ + 8];

	struct enetc_bdr tx_ring ____cacheline_aligned_in_smp;
	struct enetc_bdr rx_ring;
};

struct enetc_cls_rule {
	struct ethtool_rx_flow_spec fs;
	bool used;
};

struct enetc_ndev_priv {
	struct net_device *ndev;
	struct device *dev; /* dma-mapping device */
	struct enetc_si *si;

	int bdr_int_num; /* number of Rx/Tx ring interrupts */
	struct enetc_int_vector *int_vector;
	u16 num_rx_rings, num_tx_rings;
	u16 rx_bd_count, tx_bd_count;

	u16 msg_enable;

	struct enetc_bdr *tx_ring[16];
	struct enetc_bdr *rx_ring[16];

	struct enetc_cls_rule *cls_rules;
};

/* Messaging */

/* VF-PF set primary MAC address message format */
struct enetc_msg_cmd_set_primary_mac {
	struct enetc_msg_cmd_header header;
	struct sockaddr mac;
};

#define ENETC_RING_UNUSED(R)	(((R)->next_to_clean - (R)->next_to_use - 1 \
				 + (R)->bd_count) % (R)->bd_count)

#define ENETC_CBD(R, i)	(&(((struct enetc_cbd *)((R).bd_base))[i]))

#define ENETC_CBDR_TIMEOUT	1000 /* usecs */

enum enetc_cbdr_stat {
	ENETC_CMD_OK,
	ENETC_CMD_BUSY,
	ENETC_CMD_TIMEOUT,
};

/* SI common */
int enetc_pci_probe(struct pci_dev *pdev, const char *name, int sizeof_priv);
void enetc_pci_remove(struct pci_dev *pdev);
int enetc_alloc_msix(struct enetc_ndev_priv *priv);
void enetc_free_msix(struct enetc_ndev_priv *priv);
int enetc_setup_irqs(struct enetc_ndev_priv *priv);
void enetc_free_irqs(struct enetc_ndev_priv *priv);
void enetc_get_si_caps(struct enetc_si *si);
int enetc_alloc_si_resources(struct enetc_ndev_priv *priv);
void enetc_free_si_resources(struct enetc_ndev_priv *priv);

int enetc_open(struct net_device *ndev);
int enetc_close(struct net_device *ndev);
netdev_tx_t enetc_xmit(struct sk_buff *skb, struct net_device *ndev);
struct net_device_stats *enetc_get_stats(struct net_device *ndev);
int enetc_set_features(struct net_device *ndev,
		       netdev_features_t features);
int enetc_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		   void *type_data);
/* ethtool */
void enetc_set_ethtool_ops(struct net_device *ndev);

/* control buffer descriptor ring (CBDR) */
void enetc_set_mac_flt_entry(struct enetc_si *si, int index,
			     char *mac_addr, int si_map);
void enetc_clear_mac_flt_entry(struct enetc_si *si, int index);
int enetc_set_fs_entry(struct enetc_si *si, struct enetc_cmd_rfse *rfse,
		       int index);
void enetc_set_rss_key(struct enetc_hw *hw, const u8 *bytes);
int enetc_get_rss_table(struct enetc_si *si, u32 *table, int count);
int enetc_set_rss_table(struct enetc_si *si, const u32 *table, int count);

#ifdef CONFIG_ENETC_TSN
void enetc_tsn_init(struct enetc_si *si);
#else
#define enetc_tsn_init(si) (void)0
#endif

/* common PF and VF module params */
extern unsigned int debug;
