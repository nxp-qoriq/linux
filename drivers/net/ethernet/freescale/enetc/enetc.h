#include <linux/timer.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>

#include "enetc_hw.h"

struct enetc_tx_swbd {
	struct sk_buff *skb;
	dma_addr_t dma;
	u16 len;
	u16 last_in_frame;
	u16 is_dma_page;
};

#define ENETC_RX_MAXFRM_SIZE	9600
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

#define ENETC_TXBD(BDR, i) (&(((struct enetc_tx_bd *)((BDR).bd_base))[i]))
#define ENETC_RXBD(BDR, i) (&(((union enetc_rx_bd *)((BDR).bd_base))[i]))

#define ENETC_MADDR_HASH_TBL_SZ	64
enum enetc_mac_addr_type {UC, MC, MADDR_TYPE};
struct enetc_mac_filter {
	union {
		char mac_addr[ETH_ALEN];
		DECLARE_BITMAP(mac_hash_table, ENETC_MADDR_HASH_TBL_SZ);
	};
	int mac_addr_cnt;
};

/* PCI IEP device data */
struct enetc_si {
	struct pci_dev *pdev;
	struct enetc_hw hw;

	struct net_device *ndev; /* back ref. */

	int num_vfs; /* number of active VFs, after sriov_init */
	struct enetc_mac_filter mac_filter[ENETC_MAC_ADDR_FILT_CNT];

	struct enetc_cbdr cbd_ring;
};

#define ENETC_MAX_NUM_TXQS	8

struct enetc_int_vector {
	struct napi_struct napi;
	char name[IFNAMSIZ + 8];

	struct enetc_bdr tx_ring ____cacheline_aligned_in_smp;
	struct enetc_bdr rx_ring;
};

struct enetc_ndev_priv {
	struct net_device *ndev;
	struct device *dev; /* dma-mapping device */
	struct enetc_si *si;

	int num_int_vectors;
	struct enetc_int_vector *int_vector;
	struct msix_entry *msix_entries;
	u16 num_rx_rings, num_tx_rings;
	u16 rx_bd_count, tx_bd_count;

	u16 msg_enable;

	struct enetc_bdr *tx_ring[16];
	struct enetc_bdr *rx_ring[16];
};

void enetc_set_ethtool_ops(struct net_device *ndev);

void enetc_sync_mac_filters(struct enetc_si *si, int si_idx);
