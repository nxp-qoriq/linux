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
	unsigned int page_offset;
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
	unsigned int bd_count; /* # of BDs */
	unsigned int next_to_use;
	unsigned int next_to_clean;
	union {
		struct enetc_tx_swbd *tx_swbd;
		struct enetc_rx_swbd *rx_swbd;
	};
	union {
		void __iomem *tcisr; /* Tx */
		unsigned int next_to_alloc; /* Rx */
	};

	struct enetc_ring_stats stats;

	dma_addr_t bd_dma_base;
};

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

#define ENETC_TXBD(BDR, i) (&(((struct enetc_tx_bd *)((BDR).bd_base))[i]))
#define ENETC_RXBD(BDR, i) (&(((union enetc_rx_bd *)((BDR).bd_base))[i]))

/* PCI IEP device data */
struct enetc_si {
	struct pci_dev *pdev;
	struct enetc_hw hw;

	struct net_device *ndev; /* back ref. */
};

struct enetc_ndev_priv {
	struct net_device *ndev;
	struct device *dev; /* dma-mapping device */
	struct enetc_si *si;

	struct enetc_bdr tx_ring ____cacheline_aligned_in_smp;
	struct enetc_bdr rx_ring;

	struct napi_struct napi;

	u16 msg_enable;
	struct timer_list rxtx_int_timer;
};

void enetc_set_ethtool_ops(struct net_device *ndev);
