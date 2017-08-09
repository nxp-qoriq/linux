#include <linux/module.h>

#include "enetc.h"

#define ENETC_DRV_VER_MAJ 0
#define ENETC_DRV_VER_MIN 1

#define ENETC_DRV_VER_STR __stringify(ENETC_DRV_VER_MAJ) "." \
			  __stringify(ENETC_DRV_VER_MIN)
static const char enetc_drv_ver[] = ENETC_DRV_VER_STR;
static const char enetc_drv_name[] = "ENETC driver";

static int enetc_map_tx_buffs(struct enetc_bdr *tx_ring, struct sk_buff *skb);
static void enetc_unmap_tx_buff(struct enetc_bdr *tx_ring,
				struct enetc_tx_swbd *tx_swbd);
static void enetc_update_txbdr(struct enetc_bdr *tx_ring, int count,
			       unsigned int frm_len);
static bool enetc_clean_tx_ring(struct enetc_bdr *tx_ring);

static struct sk_buff *enetc_map_rx_buff_to_skb(struct enetc_bdr *rx_ring,
						int i, unsigned int size);
static void enetc_add_rx_buff_to_skb(struct enetc_bdr *rx_ring, int i,
				     unsigned int size, struct sk_buff *skb);
static void enetc_process_skb(struct enetc_bdr *rx_ring, struct sk_buff *skb);
static int enetc_clean_rx_ring(struct enetc_bdr *rx_ring, int work_limit);

static void rxtx_int_poll(unsigned long data)
{
	struct enetc_ndev_priv *priv = (struct enetc_ndev_priv *)data;

	if (enetc_txbdr_rd(&priv->si->hw, 0, ENETC_TBIDR) ||
	    enetc_rxbdr_rd(&priv->si->hw, 0, ENETC_RBIDR)) {
		if (napi_schedule_prep(&priv->napi)) {
			del_timer(&priv->rxtx_int_timer);
			__napi_schedule(&priv->napi);
		} else {
			WARN_ON(1);
		}
	} else {
		/* poll in another 20 ms */
		mod_timer(&priv->rxtx_int_timer, jiffies + 20 * HZ / 1000);
	}
}

#define ENETC_FREE_TXBD_NEEDED MAX_SKB_FRAGS

static netdev_tx_t enetc_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_bdr *tx_ring = &priv->tx_ring; // TODO: Tx multi-queue
	int count;

	// TODO: guard against runt (invalid) packets (?)

	if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED) {
		// TODO: check h/w index (CISR) for more acurate status
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

	count = enetc_map_tx_buffs(tx_ring, skb);

	if (likely(count)) {
		enetc_update_txbdr(tx_ring, count, skb->len);

		if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED)
			// TODO: check h/w index (CISR) for more acurate status
			netif_stop_queue(ndev);
	} else {
		dev_kfree_skb_any(skb);
	}

	return NETDEV_TX_OK;
}

static int enetc_map_tx_buffs(struct enetc_bdr *tx_ring, struct sk_buff *skb)
{
	unsigned int nr_frags = skb_shinfo(skb)->nr_frags;
	struct enetc_tx_swbd *tx_swbd;
	struct skb_frag_struct *frag;
	int len = skb_headlen(skb);
	int i, start, count = 0;
	unsigned int f;

	i = tx_ring->next_to_use;
	start = tx_ring->next_to_use;
	tx_swbd = &tx_ring->tx_swbd[i];

	tx_swbd->len = len;
	tx_swbd->dma = dma_map_single(tx_ring->dev, skb->data,
				      len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(tx_ring->dev, tx_swbd->dma)))
		goto dma_err;
	tx_swbd->is_dma_page = 0;
	count++;

	frag = &skb_shinfo(skb)->frags[0];
	for (f = 0; f < nr_frags; f++, frag++) {
		len = skb_frag_size(frag);

		tx_swbd++;
		i++;
		if (unlikely(i == tx_ring->bd_count)) {
			i = 0;
			tx_swbd = tx_ring->tx_swbd;
		}

		tx_swbd->len = len;
		tx_swbd->is_dma_page = 1;
		tx_swbd->dma = skb_frag_dma_map(tx_ring->dev, frag, 0, len,
						DMA_TO_DEVICE);
		if (dma_mapping_error(tx_ring->dev, tx_swbd->dma))
			goto dma_err;
		count++;
	}
	tx_ring->tx_swbd[i].skb = skb;
	tx_ring->tx_swbd[start].last_in_frame = i;

	return count;

dma_err:
	dev_err(tx_ring->dev, "DMA map error");

	do {
		tx_swbd = &tx_ring->tx_swbd[i];
		enetc_unmap_tx_buff(tx_ring, tx_swbd);
		if (i == 0)
			i = tx_ring->bd_count;
		i--;
	} while (count--);

	return 0;
}

static void enetc_update_txbdr(struct enetc_bdr *tx_ring, int count,
			       unsigned int frm_len)
{
	struct enetc_tx_swbd *tx_swbd;
	struct enetc_tx_bd *txbd;
	int i;

	i = tx_ring->next_to_use;
	txbd = ENETC_TXBD(*tx_ring, i);
	tx_swbd = &tx_ring->tx_swbd[i];

	/* first BD needs frm_len set */
	txbd->frm_len = cpu_to_le16(frm_len);
	txbd->flags = cpu_to_le16(ENETC_TXBD_FLAGS_IE);

	while (count--) {
		txbd->addr = cpu_to_le64(tx_swbd->dma);
		txbd->buf_len = cpu_to_le16(tx_swbd->len);

		/* last BD needs 'F' bit set */
		if (!count)
			txbd->flags = cpu_to_le16(ENETC_TXBD_FLAGS_F);

		tx_swbd++;
		txbd++;
		i++;
		if (unlikely(i == tx_ring->bd_count)) {
			i = 0;
			tx_swbd = tx_ring->tx_swbd;
			txbd = ENETC_TXBD(*tx_ring, 0);
		}
	}

	tx_ring->next_to_use = i;
	/* let H/W know BD ring has been updated */
	enetc_wr_reg(tx_ring->tcir, i); /* includes wmb() */
}

static int enetc_poll(struct napi_struct *napi, int budget)
{
	struct enetc_ndev_priv
		*priv = container_of(napi, struct enetc_ndev_priv, napi);
	bool complete = true;
	int work_done;

	enetc_clean_tx_ring(&priv->tx_ring);

	work_done = enetc_clean_rx_ring(&priv->rx_ring, budget);
	if (work_done == budget)
		complete = false;

	if (!complete)
		return budget;

	napi_complete_done(napi, work_done);

	mod_timer(&priv->rxtx_int_timer, jiffies); /* re-enable "int" polling */

	return work_done;
}

static void enetc_unmap_tx_buff(struct enetc_bdr *tx_ring,
				struct enetc_tx_swbd *tx_swbd)
{
	if (tx_swbd->dma) {
		if (tx_swbd->is_dma_page)
			dma_unmap_page(tx_ring->dev, tx_swbd->dma,
				       tx_swbd->len, DMA_TO_DEVICE);
		else
			dma_unmap_single(tx_ring->dev, tx_swbd->dma,
					 tx_swbd->len, DMA_TO_DEVICE);
		tx_swbd->dma = 0;
	}

	if (tx_swbd->skb) {
		dev_kfree_skb_any(tx_swbd->skb);
		tx_swbd->skb = NULL;
	}
}

static bool enetc_clean_tx_ring(struct enetc_bdr *tx_ring)
{
	struct net_device *ndev = tx_ring->ndev;
	int tx_frm_cnt = 0, tx_byte_cnt = 0;
	struct enetc_tx_swbd *tx_swbd;
	bool frame_cleaned = false;
	unsigned int i, last;

	i = tx_ring->next_to_clean;
	tx_swbd = &tx_ring->tx_swbd[i];
	last = tx_swbd->last_in_frame;

	while ((enetc_rd_reg(tx_ring->tcisr) & ENETC_TBCISR_IDX_MASK) != i) {
		do {
			enetc_unmap_tx_buff(tx_ring, tx_swbd);
			tx_byte_cnt += tx_swbd->len;
			frame_cleaned = (i == last);

			tx_swbd++;
			i++;
			if (unlikely(i == tx_ring->bd_count)) {
				i = 0;
				tx_swbd = tx_ring->tx_swbd;
			}
		} while (!frame_cleaned);

		tx_frm_cnt++;

		if (!tx_swbd->skb)
			break;

		last = tx_swbd->last_in_frame;
	}

	tx_ring->next_to_clean = i;
	tx_ring->stats.packets += tx_frm_cnt;
	tx_ring->stats.bytes += tx_byte_cnt;

	if (unlikely(frame_cleaned && netif_carrier_ok(ndev) &&
		     netif_queue_stopped(ndev) &&
		     (enetc_bd_unused(tx_ring) >= ENETC_FREE_TXBD_NEEDED))) {
		netif_wake_queue(ndev);
	}

	return frame_cleaned;
}

static bool enetc_new_page(struct enetc_bdr *rx_ring,
			   struct enetc_rx_swbd *rx_swbd)
{
	struct page *page;
	dma_addr_t addr;

	page = dev_alloc_page();
	if (unlikely(!page))
		return false;

	addr = dma_map_page(rx_ring->dev, page, 0, PAGE_SIZE, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(rx_ring->dev, addr))) {
		__free_page(page);

		return false;
	}

	rx_swbd->dma = addr;
	rx_swbd->page = page;
	rx_swbd->page_offset = ENETC_RXB_PAD;

	return true;
}

static int enetc_refill_rx_ring(struct enetc_bdr *rx_ring, const int buff_cnt)
{
	struct enetc_rx_swbd *rx_swbd;
	union enetc_rx_bd *rxbd;
	int i, j;

	i = rx_ring->next_to_use;
	rx_swbd = &rx_ring->rx_swbd[i];
	rxbd = ENETC_RXBD(*rx_ring, i);

	for (j = 0; j < buff_cnt; j++) {
		/* try reuse page */
		if (unlikely(!rx_swbd->page)) {
			if (unlikely(!enetc_new_page(rx_ring, rx_swbd))) {
				// TODO: alloc error
				WARN_ON(1);
				break;
			}
		}

		/* update RxBD */
		rxbd->w.addr = cpu_to_le64(rx_swbd->dma +
					   rx_swbd->page_offset);
		/* clear 'R" as well */
		rxbd->r.lstatus = 0;

		rx_swbd++;
		rxbd++;
		i++;
		if (unlikely(i == rx_ring->bd_count)) {
			i = 0;
			rx_swbd = rx_ring->rx_swbd;
			rxbd = ENETC_RXBD(*rx_ring, 0);
		}
	}

	if (likely(j)) {
		rx_ring->next_to_alloc = i; /* keep track from page reuse */
		rx_ring->next_to_use = i;
		/* update ENETC's consumer index */
		enetc_wr_reg(rx_ring->rcir, i);
	}

	return j;
}

#define ENETC_RXBD_BUNDLE 16 /* recommended # of BDs to update at once */

static int enetc_clean_rx_ring(struct enetc_bdr *rx_ring, int work_limit)
{
	struct enetc_ndev_priv *priv = netdev_priv(rx_ring->ndev);
	int rx_frm_cnt = 0, rx_byte_cnt = 0;
	int cleaned_cnt, i;

	cleaned_cnt = enetc_bd_unused(rx_ring);
	/* next descriptor to process */
	i = rx_ring->next_to_clean;

	while (likely(rx_frm_cnt < work_limit)) {
		union enetc_rx_bd *rxbd;
		struct sk_buff *skb;
		unsigned int size;
		u32 bd_status;

		if (cleaned_cnt >= ENETC_RXBD_BUNDLE) {
			int count = enetc_refill_rx_ring(rx_ring, cleaned_cnt);

			cleaned_cnt -= count;
		}

		rxbd = ENETC_RXBD(*rx_ring, i);
		bd_status = le32_to_cpu(rxbd->r.lstatus);
		if (!bd_status)
			break;

		dma_rmb(); /* for readig other rxbd fields */
		size = le16_to_cpu(rxbd->r.buf_len);
		skb = enetc_map_rx_buff_to_skb(rx_ring, i, size);
		if (!skb) {
			// TODO: increase alloc error counter
			break;
		}

		cleaned_cnt++;
		rxbd++;
		i++;
		if (unlikely(i == rx_ring->bd_count)) {
			i = 0;
			rxbd = ENETC_RXBD(*rx_ring, 0);
		}

		if (unlikely(bd_status &
			     ENETC_RXBD_LSTATUS(ENETC_RXBD_ERR_MASK))) {
			// TODO: rx error statistics
			dev_kfree_skb(skb);
			break;
		}

		/* not last BD in frame? */
		while (!(bd_status & ENETC_RXBD_LSTATUS_F)) {
			bd_status = le32_to_cpu(rxbd->r.lstatus);
			size = ENETC_RXB_DMA_SIZE;

			if (bd_status & ENETC_RXBD_LSTATUS_F) {
				dma_rmb();
				size = le16_to_cpu(rxbd->r.buf_len);
			}

			enetc_add_rx_buff_to_skb(rx_ring, i, size, skb);

			cleaned_cnt++;
			rxbd++;
			i++;
			if (unlikely(i == rx_ring->bd_count)) {
				i = 0;
				rxbd = ENETC_RXBD(*rx_ring, 0);
			}
		}

		rx_byte_cnt += skb->len;

		enetc_process_skb(rx_ring, skb);

		napi_gro_receive(&priv->napi, skb);

		rx_frm_cnt++;
	}

	rx_ring->next_to_clean = i;
	// TODO: 64-bit stats
	rx_ring->stats.packets += rx_frm_cnt;
	rx_ring->stats.bytes += rx_byte_cnt;

	return rx_frm_cnt;
}

static bool enetc_page_reusable(struct page *page)
{
	return (!page_is_pfmemalloc(page) && page_ref_count(page) == 1);
}

static void enetc_reuse_page(struct enetc_bdr *rx_ring,
			     struct enetc_rx_swbd *old)
{
	struct enetc_rx_swbd *new;

	new = &rx_ring->rx_swbd[rx_ring->next_to_alloc];

	/* next buf that may reuse a page */
	enetc_bdr_idx_inc(rx_ring, &rx_ring->next_to_alloc);

	/* copy page reference */
	*new = *old;
}

struct enetc_rx_swbd *enetc_get_rx_buff(struct enetc_bdr *rx_ring, int i,
					unsigned int size)
{
	struct enetc_rx_swbd *rx_swbd = &rx_ring->rx_swbd[i];

	dma_sync_single_range_for_cpu(rx_ring->dev, rx_swbd->dma,
				      rx_swbd->page_offset,
				      size, DMA_FROM_DEVICE);
	return rx_swbd;
}

static void enetc_put_rx_buff(struct enetc_bdr *rx_ring,
			      struct enetc_rx_swbd *rx_swbd)
{
	if (likely(enetc_page_reusable(rx_swbd->page))) {
		rx_swbd->page_offset ^= ENETC_RXB_TRUESIZE;
		page_ref_inc(rx_swbd->page);

		enetc_reuse_page(rx_ring, rx_swbd);

		/* sync for use by the device */
		dma_sync_single_range_for_device(rx_ring->dev, rx_swbd->dma,
						 rx_swbd->page_offset,
						 ENETC_RXB_DMA_SIZE,
						 DMA_FROM_DEVICE);
	} else {
		dma_unmap_page(rx_ring->dev, rx_swbd->dma,
			       PAGE_SIZE, DMA_FROM_DEVICE);
	}

	rx_swbd->page = NULL;
}

static struct sk_buff *enetc_map_rx_buff_to_skb(struct enetc_bdr *rx_ring,
						int i, unsigned int size)
{
	struct enetc_rx_swbd *rx_swbd = enetc_get_rx_buff(rx_ring, i, size);
	struct sk_buff *skb;
	void *ba;

	ba = page_address(rx_swbd->page) + rx_swbd->page_offset;
	skb = build_skb(ba - ENETC_RXB_PAD, ENETC_RXB_TRUESIZE);
	if (unlikely(!skb)) {
		// TODO: alloc err counter
		return NULL;
	}

	skb_reserve(skb, ENETC_RXB_PAD);
	__skb_put(skb, size);

	enetc_put_rx_buff(rx_ring, rx_swbd);

	return skb;
}

static void enetc_add_rx_buff_to_skb(struct enetc_bdr *rx_ring, int i,
				     unsigned int size, struct sk_buff *skb)
{
	struct enetc_rx_swbd *rx_swbd = enetc_get_rx_buff(rx_ring, i, size);

	skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, rx_swbd->page,
			rx_swbd->page_offset, size, ENETC_RXB_TRUESIZE);

	enetc_put_rx_buff(rx_ring, rx_swbd);
}

static void enetc_process_skb(struct enetc_bdr *rx_ring,
			      struct sk_buff *skb)
{
	// TODO: checksum, tstamp, VLAN, hash

	skb_record_rx_queue(skb, 0); // TODO: use queue_idx for multi-queue
	skb->protocol = eth_type_trans(skb, rx_ring->ndev);
}

/* Probing and Init */

static void enetc_sw_init(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	priv->tx_ring.bd_count = 1024; //TODO: use defines for defaults
	priv->rx_ring.bd_count = 1024;
}

static int enetc_alloc_tx_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *txr = &priv->tx_ring;
	int size;

	txr->tx_swbd = vzalloc(txr->bd_count * sizeof(struct enetc_tx_swbd));
	if (!txr->tx_swbd)
		return -ENOMEM;

	size = txr->bd_count * sizeof(struct enetc_tx_bd);
	txr->bd_base = dma_zalloc_coherent(priv->dev, size, &txr->bd_dma_base,
					   GFP_KERNEL);
	if (!txr->bd_base) {
		vfree(txr->tx_swbd);
		return -ENOMEM;
	}

	txr->next_to_clean = 0;
	txr->next_to_use = 0;

	txr->ndev = priv->ndev;
	txr->dev = priv->dev;

	return 0;
}

static void enetc_free_tx_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *txr = &priv->tx_ring;
	int size;

	size = txr->bd_count * sizeof(struct enetc_tx_bd);

	dma_free_coherent(priv->dev, size, txr->bd_base, txr->bd_dma_base);
	txr->bd_base = NULL;

	vfree(txr->tx_swbd);
	txr->tx_swbd = 0;
	// TODO: free tx_ring dma mappings and skbs
}

static int enetc_alloc_rx_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *rxr = &priv->rx_ring;
	int size;

	rxr->rx_swbd = vzalloc(rxr->bd_count * sizeof(struct enetc_rx_swbd));
	if (!rxr->rx_swbd)
		return -ENOMEM;

	size = rxr->bd_count * sizeof(union enetc_rx_bd);
	rxr->bd_base = dma_zalloc_coherent(priv->dev, size, &rxr->bd_dma_base,
					   GFP_KERNEL);
	if (!rxr->bd_base) {
		vfree(rxr->rx_swbd);
		return -ENOMEM;
	}

	rxr->next_to_clean = 0;
	rxr->next_to_use = 0;
	rxr->next_to_alloc = 0;

	rxr->ndev = priv->ndev;
	rxr->dev = priv->dev;

	return 0;
}

static void enetc_free_rx_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *rxr = &priv->rx_ring;
	int size;

	size = rxr->bd_count * sizeof(union enetc_rx_bd);

	dma_free_coherent(priv->dev, size, rxr->bd_base, rxr->bd_dma_base);
	rxr->bd_base = NULL;

	vfree(rxr->rx_swbd);
	rxr->rx_swbd = NULL;
}

static void enetc_free_tx_ring(struct enetc_bdr *tx_ring)
{
	int i;

	if (!tx_ring->tx_swbd)
		return;

	for (i = 0; i < tx_ring->bd_count; i++) {
		struct enetc_tx_swbd *tx_swbd = &tx_ring->tx_swbd[i];

		enetc_unmap_tx_buff(tx_ring, tx_swbd);
	}

	tx_ring->next_to_clean = 0;
	tx_ring->next_to_use = 0;
}

static void enetc_free_rx_ring(struct enetc_bdr *rx_ring)
{
	int i;

	if (!rx_ring->rx_swbd)
		return;

	for (i = 0; i < rx_ring->bd_count; i++) {
		struct enetc_rx_swbd *rx_swbd = &rx_ring->rx_swbd[i];

		if (!rx_swbd->page)
			continue;

		dma_unmap_page(rx_ring->dev, rx_swbd->dma,
			       PAGE_SIZE, DMA_FROM_DEVICE);
		__free_page(rx_swbd->page);
		rx_swbd->page = NULL;
	}

	// TODO: zero out rx_swbd and BD ring?
	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
	rx_ring->next_to_alloc = 0;
}

static void enetc_setup_tx_ring(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *tx_ring = &priv->tx_ring;
	struct enetc_hw *hw = &priv->si->hw;
	u32 tbmr;

	WARN_ON(lower_32_bits(tx_ring->bd_dma_base) & 0x1f);

	enetc_txbdr_wr(hw, 0, ENETC_TBBAR0,
		       lower_32_bits(tx_ring->bd_dma_base));

	enetc_txbdr_wr(hw, 0, ENETC_TBBAR1,
		       upper_32_bits(tx_ring->bd_dma_base));

	WARN_ON(tx_ring->bd_count & 0x3f); // must be multiple of 64

	tbmr = ENETC_RTBMR_RSIZE(tx_ring->bd_count);
	tbmr |= ENETC_TBMR_EN;
	/* enable ring */
	enetc_txbdr_wr(hw, 0, ENETC_TBMR, tbmr);

	enetc_txbdr_wr(hw, 0, ENETC_TBCIR, 0);
	enetc_txbdr_wr(hw, 0, ENETC_TBCISR, 0);
	tx_ring->tcir = hw->reg + ENETC_BDR(TX, 0, ENETC_TBCIR);
	tx_ring->tcisr = hw->reg + ENETC_BDR(TX, 0, ENETC_TBCISR);
}

static void enetc_setup_rx_ring(struct enetc_ndev_priv *priv)
{
	struct enetc_bdr *rx_ring = &priv->rx_ring;
	struct enetc_hw *hw = &priv->si->hw;
	u32 rbmr;

	WARN_ON(lower_32_bits(rx_ring->bd_dma_base) & 0x1f);

	enetc_rxbdr_wr(hw, 0, ENETC_RBBAR0,
		       lower_32_bits(rx_ring->bd_dma_base));

	enetc_rxbdr_wr(hw, 0, ENETC_RBBAR1,
		       upper_32_bits(rx_ring->bd_dma_base));

	WARN_ON(rx_ring->bd_count & 0x3f); // must be multiple of 64

	enetc_rxbdr_wr(hw, 0, ENETC_RBBSR, ENETC_RXB_DMA_SIZE);

	/* enable Rx ints by setting pkt thr to 1 (BG 0.7) */
	enetc_rxbdr_wr(hw, 0, ENETC_RBICIR0, ENETC_RBICIR0_ICEN	| 0x1);

	rbmr = ENETC_RTBMR_RSIZE(rx_ring->bd_count);
	rbmr |= ENETC_RBMR_EN;
	/* enable ring */
	enetc_rxbdr_wr(hw, 0, ENETC_RBMR, rbmr);

	enetc_rxbdr_wr(hw, 0, ENETC_RBPIR, 0);
	rx_ring->rcir = hw->reg + ENETC_BDR(RX, 0, ENETC_RBCIR);
	enetc_refill_rx_ring(rx_ring, enetc_bd_unused(rx_ring));
}

static void enetc_enable_port(struct enetc_si *si)
{
	enetc_wr(&si->hw, ENETC_SIMR, ENETC_SIMR_EN);
	enetc_wr(&si->hw, ENETC_PMR, ENETC_PMR_EN);
}

static void enetc_setup_mac(struct enetc_si *si)
{
	enetc_wr(&si->hw, ENETC_PM0_MAXFRM,
		 ENETC_SET_MAXFRM(ENETC_RX_MAXFRM_SIZE));

	enetc_wr(&si->hw, ENETC_PM0_CMD_CFG,
		 ENETC_PM0_TX_EN | ENETC_PM0_RX_EN);

	/* enable promisc for now (FIXME) */
	enetc_wr(&si->hw, ENETC_PSIPMR, 0x1);
}

static void enetc_setup_interrupts(struct enetc_ndev_priv *priv)
{
	/* enable Tx & Rx event indication */
	enetc_txbdr_wr(&priv->si->hw, 0, ENETC_TBIER, ENETC_TBIER_TXFIE);
	enetc_rxbdr_wr(&priv->si->hw, 0, ENETC_RBIER, ENETC_RBIER_RXTIE);
}

static int enetc_open(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int err;

	netif_carrier_on(ndev);

	err = enetc_alloc_tx_resources(priv);
	if (err)
		goto err_alloc_tx;

	err = enetc_alloc_rx_resources(priv);
	if (err)
		goto err_alloc_rx;

	//TODO: enable h/w port for tx/rx
	enetc_setup_tx_ring(priv);
	enetc_setup_rx_ring(priv);
	enetc_setup_mac(priv->si);
	enetc_setup_interrupts(priv);

	napi_enable(&priv->napi);

	//TODO: start "interrupt" processing
	mod_timer(&priv->rxtx_int_timer, jiffies);

	netif_start_queue(ndev);

	return 0;

err_alloc_rx:
	enetc_free_tx_resources(priv);
err_alloc_tx:

	return err;
}

static int enetc_close(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	netif_carrier_off(ndev);

	napi_disable(&priv->napi);

	//TODO: stop "interrupt" processing
	del_timer_sync(&priv->rxtx_int_timer);

	netif_stop_queue(ndev);

	enetc_free_rx_ring(&priv->rx_ring);
	enetc_free_tx_ring(&priv->tx_ring);

	enetc_free_rx_resources(priv);
	enetc_free_tx_resources(priv);

	return 0;
}

static int enetc_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct sockaddr *saddr = addr;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, saddr->sa_data, ndev->addr_len);

	return 0;
}

static struct net_device_stats *enetc_get_stats(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	stats->rx_packets	= priv->rx_ring.stats.packets;
	stats->rx_bytes		= priv->rx_ring.stats.bytes;

	stats->tx_packets	= priv->tx_ring.stats.packets;
	stats->tx_bytes		= priv->tx_ring.stats.bytes;

	return stats;
}

static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_set_mac_addr,
};

static int enetc_netdev_setup(struct enetc_si *si)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	int err;

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_IFUP << 1) - 1; //TODO: netif_msg_init()
	ndev->netdev_ops = &enetc_ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;
	netif_napi_add(ndev, &priv->napi, enetc_poll, 64);

	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG;

	init_timer(&priv->rxtx_int_timer);
	priv->rxtx_int_timer.function = rxtx_int_poll;
	priv->rxtx_int_timer.data = (unsigned long)priv;

	err = register_netdev(ndev);
	if (err)
		goto err_register;

	netif_carrier_off(ndev);

	netif_info(priv, probe, ndev, "%s v%s\n",
		   enetc_drv_name, enetc_drv_ver);

	return 0;

err_register:
	si->ndev = NULL;
	free_netdev(ndev);

	return err;
}

static void enetc_netdev_teardown(struct net_device *ndev)
{
	unregister_netdev(ndev);
	free_netdev(ndev);
}

static int enetc_hw_init(struct enetc_si *si)
{
	//TODO: One-time device h/w settings if needed (@probe())
	enetc_enable_port(si);

	return 0;
}

static int enetc_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct enetc_si *si;
	struct enetc_hw *hw;
	int err;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);
	si = kzalloc(sizeof(*si), GFP_KERNEL);
	if (!si) {
		err = -ENOMEM;
		goto err_alloc_si;
	}

	pci_set_drvdata(pdev, si);
	si->pdev = pdev;
	hw = &si->hw;

	hw->reg = ioremap(pci_resource_start(pdev, 0),
			  pci_resource_len(pdev, 0));
	if (!hw->reg) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_ioremap;
	}

	enetc_hw_init(si);

	err = enetc_netdev_setup(si);
	if (err) {
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	enetc_sw_init(si->ndev);

	return 0;

err_alloc_netdev:
	iounmap(hw->reg);
err_ioremap:
	kfree(si);
err_alloc_si:
	pci_release_mem_regions(pdev);
err_pci_mem_reg:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void enetc_pci_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_hw *hw = &si->hw;

	dev_info(&pdev->dev, "enetc_pci_remove()\n");

	if (si->ndev)
		enetc_netdev_teardown(si->ndev);

	iounmap(hw->reg);
	kfree(si);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id enetc_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, 0xe001) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_id_table);

static struct pci_driver enetc_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_id_table,
	.probe = enetc_pci_probe,
	.remove = enetc_pci_remove,
};
module_pci_driver(enetc_driver);

MODULE_DESCRIPTION("ENETC driver");
MODULE_LICENSE("GPL");
