#include <linux/module.h>

#include "enetc.h"

#define ENETC_DRV_VER_MAJ 0
#define ENETC_DRV_VER_MIN 2

#define ENETC_DRV_VER_STR __stringify(ENETC_DRV_VER_MAJ) "." \
			  __stringify(ENETC_DRV_VER_MIN)
static const char enetc_drv_ver[] = ENETC_DRV_VER_STR;
static const char enetc_drv_name[] = "ENETC driver";

static int enetc_map_tx_buffs(struct enetc_bdr *tx_ring, struct sk_buff *skb);
static void enetc_unmap_tx_buff(struct enetc_bdr *tx_ring,
				struct enetc_tx_swbd *tx_swbd);
static void enetc_update_txbdr(struct enetc_bdr *tx_ring, int count, u16 len);
static bool enetc_clean_tx_ring(struct enetc_bdr *tx_ring);

static struct sk_buff *enetc_map_rx_buff_to_skb(struct enetc_bdr *rx_ring,
						int i, u16 size);
static void enetc_add_rx_buff_to_skb(struct enetc_bdr *rx_ring, int i,
				     u16 size, struct sk_buff *skb);
static void enetc_process_skb(struct enetc_bdr *rx_ring, struct sk_buff *skb);
static int enetc_clean_rx_ring(struct enetc_bdr *rx_ring,
			       struct napi_struct *napi, int work_limit);

static irqreturn_t enetc_msix(int irq, void *data)
{
	struct napi_struct *napi = data;

	napi_schedule(napi);

	return IRQ_HANDLED;
}

#define ENETC_FREE_TXBD_NEEDED MAX_SKB_FRAGS

static netdev_tx_t enetc_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_bdr *tx_ring;
	int count;

	// TODO: guard against runt (invalid) packets (?)

	tx_ring = priv->tx_ring[skb->queue_mapping];

	if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED) {
		// TODO: check h/w index (CISR) for more acurate status
		netif_stop_subqueue(ndev, tx_ring->index);
		return NETDEV_TX_BUSY;
	}

	count = enetc_map_tx_buffs(tx_ring, skb);

	if (likely(count)) {
		enetc_update_txbdr(tx_ring, count, skb->len);

		if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED)
			// TODO: check h/w index (CISR) for more acurate status
			netif_stop_subqueue(ndev, tx_ring->index);
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

static void enetc_update_txbdr(struct enetc_bdr *tx_ring, int count, u16 len)
{
	struct enetc_tx_swbd *tx_swbd;
	struct enetc_tx_bd *txbd;
	int i;

	i = tx_ring->next_to_use;
	txbd = ENETC_TXBD(*tx_ring, i);
	tx_swbd = &tx_ring->tx_swbd[i];

	/* first BD needs frm_len set */
	txbd->frm_len = cpu_to_le16(len);
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
	struct enetc_int_vector
		*v = container_of(napi, struct enetc_int_vector, napi);
	bool complete = true;
	int work_done;

	enetc_clean_tx_ring(&v->tx_ring);

	work_done = enetc_clean_rx_ring(&v->rx_ring, napi, budget);
	if (work_done == budget)
		complete = false;

	if (!complete)
		return budget;

	napi_complete_done(napi, work_done);

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
	int i, last;

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
		     __netif_subqueue_stopped(ndev, tx_ring->index) &&
		     (enetc_bd_unused(tx_ring) >= ENETC_FREE_TXBD_NEEDED))) {
		netif_wake_subqueue(ndev, tx_ring->index);
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

static int enetc_clean_rx_ring(struct enetc_bdr *rx_ring,
			       struct napi_struct *napi, int work_limit)
{
	int rx_frm_cnt = 0, rx_byte_cnt = 0;
	int cleaned_cnt, i;

	cleaned_cnt = enetc_bd_unused(rx_ring);
	/* next descriptor to process */
	i = rx_ring->next_to_clean;

	while (likely(rx_frm_cnt < work_limit)) {
		union enetc_rx_bd *rxbd;
		struct sk_buff *skb;
		u32 bd_status;
		u16 size;

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

		napi_gro_receive(napi, skb);

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
					u16 size)
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
						int i, u16 size)
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
				     u16 size, struct sk_buff *skb)
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

static void enetc_sw_init(struct enetc_ndev_priv *priv)
{
	priv->tx_bd_count = 1024; //TODO: use defines for defaults
	priv->rx_bd_count = 1024;

	priv->num_rx_rings = num_online_cpus();
	/* support queue pairs only for now */
	priv->num_tx_rings = priv->num_rx_rings;
	priv->num_int_vectors = priv->num_rx_rings;
}

static int enetc_alloc_txbdr(struct enetc_bdr *txr)
{
	int size;

	txr->tx_swbd = vzalloc(txr->bd_count * sizeof(struct enetc_tx_swbd));
	if (!txr->tx_swbd)
		return -ENOMEM;

	size = txr->bd_count * sizeof(struct enetc_tx_bd);
	txr->bd_base = dma_zalloc_coherent(txr->dev, size, &txr->bd_dma_base,
					   GFP_KERNEL);
	if (!txr->bd_base) {
		vfree(txr->tx_swbd);
		return -ENOMEM;
	}

	txr->next_to_clean = 0;
	txr->next_to_use = 0;

	return 0;
}

static void enetc_free_txbdr(struct enetc_bdr *txr)
{
	int size;

	size = txr->bd_count * sizeof(struct enetc_tx_bd);

	dma_free_coherent(txr->dev, size, txr->bd_base, txr->bd_dma_base);
	txr->bd_base = NULL;

	vfree(txr->tx_swbd);
	txr->tx_swbd = 0;
	// TODO: free tx_ring dma mappings and skbs
}

static int enetc_alloc_tx_resources(struct enetc_ndev_priv *priv)
{
	int i, err;

	for (i = 0; i < priv->num_tx_rings; i++) {
		err = enetc_alloc_txbdr(priv->tx_ring[i]);

		if (err)
			goto fail;
	}

	return 0;

fail:
	while (i-- > 0)
		enetc_free_txbdr(priv->tx_ring[i]);

	return err;
}

static void enetc_free_tx_resources(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		enetc_free_txbdr(priv->tx_ring[i]);
}

static int enetc_alloc_rxbdr(struct enetc_bdr *rxr)
{
	int size;

	rxr->rx_swbd = vzalloc(rxr->bd_count * sizeof(struct enetc_rx_swbd));
	if (!rxr->rx_swbd)
		return -ENOMEM;

	size = rxr->bd_count * sizeof(union enetc_rx_bd);
	rxr->bd_base = dma_zalloc_coherent(rxr->dev, size, &rxr->bd_dma_base,
					   GFP_KERNEL);
	if (!rxr->bd_base) {
		vfree(rxr->rx_swbd);
		return -ENOMEM;
	}

	rxr->next_to_clean = 0;
	rxr->next_to_use = 0;
	rxr->next_to_alloc = 0;

	return 0;
}

static void enetc_free_rxbdr(struct enetc_bdr *rxr)
{
	int size;

	size = rxr->bd_count * sizeof(union enetc_rx_bd);

	dma_free_coherent(rxr->dev, size, rxr->bd_base, rxr->bd_dma_base);
	rxr->bd_base = NULL;

	vfree(rxr->rx_swbd);
	rxr->rx_swbd = NULL;
}

static int enetc_alloc_rx_resources(struct enetc_ndev_priv *priv)
{
	int i, err;

	for (i = 0; i < priv->num_rx_rings; i++) {
		err = enetc_alloc_rxbdr(priv->rx_ring[i]);

		if (err)
			goto fail;
	}

	return 0;

fail:
	while (i-- > 0)
		enetc_free_rxbdr(priv->rx_ring[i]);

	return err;
}

static void enetc_free_rx_resources(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_rx_rings; i++)
		enetc_free_rxbdr(priv->rx_ring[i]);
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

static void enetc_free_rxtx_rings(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_rx_rings; i++)
		enetc_free_rx_ring(priv->rx_ring[i]);

	for (i = 0; i < priv->num_tx_rings; i++)
		enetc_free_tx_ring(priv->tx_ring[i]);
}

static void enetc_setup_txbdr(struct enetc_hw *hw, struct enetc_bdr *tx_ring)
{
	int idx = tx_ring->index;
	u32 tbmr;

	WARN_ON(lower_32_bits(tx_ring->bd_dma_base) & 0x1f);

	enetc_txbdr_wr(hw, idx, ENETC_TBBAR0,
		       lower_32_bits(tx_ring->bd_dma_base));

	enetc_txbdr_wr(hw, idx, ENETC_TBBAR1,
		       upper_32_bits(tx_ring->bd_dma_base));

	WARN_ON(tx_ring->bd_count & 0x3f); //FIXME: must be multiple of 64

	enetc_txbdr_wr(hw, idx, ENETC_TBLENR,
		       ENETC_RTBLENR_LEN(tx_ring->bd_count));

	tbmr = ENETC_TBMR_EN;
	/* enable ring */
	enetc_txbdr_wr(hw, idx, ENETC_TBMR, tbmr);

	enetc_txbdr_wr(hw, idx, ENETC_TBCIR, 0);
	enetc_txbdr_wr(hw, idx, ENETC_TBCISR, 0);
	tx_ring->tcir = hw->reg + ENETC_BDR(TX, idx, ENETC_TBCIR);
	tx_ring->tcisr = hw->reg + ENETC_BDR(TX, idx, ENETC_TBCISR);
}

static void enetc_setup_rxbdr(struct enetc_hw *hw, struct enetc_bdr *rx_ring)
{
	int idx = rx_ring->index;
	u32 rbmr;

	WARN_ON(lower_32_bits(rx_ring->bd_dma_base) & 0x1f);

	enetc_rxbdr_wr(hw, idx, ENETC_RBBAR0,
		       lower_32_bits(rx_ring->bd_dma_base));

	enetc_rxbdr_wr(hw, idx, ENETC_RBBAR1,
		       upper_32_bits(rx_ring->bd_dma_base));

	WARN_ON(rx_ring->bd_count & 0x3f); //FIXME: must be multiple of 64

	enetc_rxbdr_wr(hw, idx, ENETC_RBLENR,
		       ENETC_RTBLENR_LEN(rx_ring->bd_count));

	enetc_rxbdr_wr(hw, idx, ENETC_RBBSR, ENETC_RXB_DMA_SIZE);

	/* enable Rx ints by setting pkt thr to 1 (BG 0.7) */
	enetc_rxbdr_wr(hw, idx, ENETC_RBICIR0, ENETC_RBICIR0_ICEN | 0x1);

	rbmr = ENETC_RBMR_EN;
	/* enable ring */
	enetc_rxbdr_wr(hw, idx, ENETC_RBMR, rbmr);

	enetc_rxbdr_wr(hw, idx, ENETC_RBPIR, 0);
	rx_ring->rcir = hw->reg + ENETC_BDR(RX, idx, ENETC_RBCIR);

	enetc_refill_rx_ring(rx_ring, enetc_bd_unused(rx_ring));
}

static void enetc_setup_bdrs(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		enetc_setup_txbdr(&priv->si->hw, priv->tx_ring[i]);

	for (i = 0; i < priv->num_rx_rings; i++)
		enetc_setup_rxbdr(&priv->si->hw, priv->rx_ring[i]);
}

static void enetc_configure_port_mac(struct enetc_si *si)
{
	enetc_port_wr(&si->hw, ENETC_PM0_MAXFRM,
		      ENETC_SET_MAXFRM(ENETC_RX_MAXFRM_SIZE));

	enetc_port_wr(&si->hw, ENETC_PM0_CMD_CFG,
		      ENETC_PM0_TX_EN | ENETC_PM0_RX_EN);
}

static void enetc_configure_hw_vector(struct enetc_hw *hw, int idx, u16 entry)
{
	/* TODO: Only queue pairs supported for now */
	enetc_wr(hw, ENETC_SIMSITRV(idx), entry);
	enetc_wr(hw, ENETC_SIMSIRRV(idx), entry);
}

static int enetc_setup_irqs(struct enetc_ndev_priv *priv)
{
	int vectors = priv->num_int_vectors;
	int i, n, err;

	n = pci_enable_msix_range(priv->si->pdev, priv->msix_entries,
				  vectors, vectors);
	if (n < 0)
		return n;

	if (n != vectors)
		return -EPERM;

	for (i = 0; i < vectors; i++) {
		struct enetc_int_vector *v = &priv->int_vector[i];

		sprintf(v->name, "%s-rxtx%d", priv->ndev->name, i);

		err = request_irq(priv->msix_entries[i].vector, enetc_msix, 0,
				  v->name, &v->napi);
		if (err) {
			dev_err(priv->dev, "request_irq() failed!\n");
			goto irq_err;
		}

		enetc_configure_hw_vector(&priv->si->hw, i,
					  priv->msix_entries[i].entry);
	}

	return 0;

irq_err:
	while (i-- > 0)
		free_irq(priv->msix_entries[i].vector,
			 &priv->int_vector[i].napi);

	pci_disable_msix(priv->si->pdev);

	return err;
}

static void enetc_free_irqs(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_int_vectors; i++)
		free_irq(priv->msix_entries[i].vector,
			 &priv->int_vector[i].napi);

	pci_disable_msix(priv->si->pdev);
}

static void enetc_enable_interrupts(struct enetc_ndev_priv *priv)
{
	int i;

	/* enable Tx & Rx event indication */
	for (i = 0; i < priv->num_int_vectors; i++) {
		enetc_txbdr_wr(&priv->si->hw, i,
			       ENETC_TBIER, ENETC_TBIER_TXFIE);
		enetc_rxbdr_wr(&priv->si->hw, i,
			       ENETC_RBIER, ENETC_RBIER_RXTIE);
	}
}

static void enetc_disable_interrupts(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_int_vectors; i++) {
		enetc_txbdr_wr(&priv->si->hw, i, ENETC_TBIER, 0);
		enetc_rxbdr_wr(&priv->si->hw, i, ENETC_RBIER, 0);

		synchronize_irq(priv->msix_entries[i].vector);
	}
}

static int enetc_open(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int i, err;

	netif_carrier_on(ndev);

	err = enetc_alloc_tx_resources(priv);
	if (err)
		goto err_alloc_tx;

	err = enetc_alloc_rx_resources(priv);
	if (err)
		goto err_alloc_rx;

	enetc_setup_bdrs(priv);

	err = enetc_setup_irqs(priv);
	if (err)
		goto err_setup_irqs;

	err = netif_set_real_num_tx_queues(ndev, priv->num_tx_rings);
	if (err)
		goto err_set_queues;

	err = netif_set_real_num_rx_queues(ndev, priv->num_rx_rings);
	if (err)
		goto err_set_queues;

	for (i = 0; i < priv->num_int_vectors; i++)
		napi_enable(&priv->int_vector[i].napi);

	enetc_enable_interrupts(priv);

	netif_tx_start_all_queues(ndev);

	return 0;

err_set_queues:
	enetc_free_irqs(priv);
err_setup_irqs:
	enetc_free_rx_resources(priv);
err_alloc_rx:
	enetc_free_tx_resources(priv);
err_alloc_tx:

	return err;
}

static int enetc_close(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int i;

	netif_carrier_off(ndev);
	netif_tx_stop_all_queues(ndev);

	enetc_disable_interrupts(priv);

	for (i = 0; i < priv->num_int_vectors; i++) {
		napi_synchronize(&priv->int_vector[i].napi);
		napi_disable(&priv->int_vector[i].napi);
	}

	enetc_free_irqs(priv);

	enetc_free_rxtx_rings(priv);
	enetc_free_rx_resources(priv);
	enetc_free_tx_resources(priv);

	return 0;
}

static void enetc_set_primary_mac_addr(struct enetc_hw *hw, const u8 *addr)
{
	u16 upper = ntohs(*(const u16 *)addr);
	u32 lower = ntohl(*(const u32 *)(addr + 2));

	enetc_port_wr(hw, ENETC_PSIPMAR0(0), lower);
	enetc_port_wr(hw, ENETC_PSIPMAR1(0), upper << 16);
}

static void enetc_get_primary_mac_addr(struct enetc_hw *hw, u8 *addr)
{
	*(u32 *)(addr + 2) = htonl((u32)enetc_rd(hw, ENETC_SIPMAR0));
	*(u16 *)addr = htons(enetc_rd(hw, ENETC_SIPMAR1) >> 16);
}

static int enetc_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sockaddr *saddr = addr;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, saddr->sa_data, ndev->addr_len);
	enetc_set_primary_mac_addr(&priv->si->hw, saddr->sa_data);

	return 0;
}

static void enetc_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	u32 psipmr = 0;

	if (ndev->flags & IFF_PROMISC) {
		/* enable promisc mode for SI0 (PF) */
		psipmr = ENETC_PSIPMR_SET_UP(0) | ENETC_PSIPMR_SET_MP(0);
	} else if (ndev->flags & IFF_ALLMULTI) {
		/* enable multi cast promisc mode for SI0 (PF) */
		psipmr = ENETC_PSIPMR_SET_MP(0);
	}

	psipmr |= enetc_port_rd(hw, ENETC_PSIPMR) &
		  ~(ENETC_PSIPMR_SET_UP(0) | ENETC_PSIPMR_SET_MP(0));
	enetc_port_wr(hw, ENETC_PSIPMR, psipmr);
}

static struct net_device_stats *enetc_get_stats(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	unsigned long packets = 0, bytes = 0;
	int i;

	for (i = 0; i < priv->num_rx_rings; i++) {
		packets += priv->rx_ring[i]->stats.packets;
		bytes	+= priv->rx_ring[i]->stats.bytes;
	}

	stats->rx_packets = packets;
	stats->rx_bytes = bytes;
	bytes = 0;
	packets = 0;

	for (i = 0; i < priv->num_tx_rings; i++) {
		packets += priv->tx_ring[i]->stats.packets;
		bytes	+= priv->tx_ring[i]->stats.bytes;
	}

	stats->tx_packets = packets;
	stats->tx_bytes = bytes;

	return stats;
}

static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_set_mac_addr,
	.ndo_set_rx_mode	= enetc_set_rx_mode,
};

static void enetc_netdev_setup(struct enetc_si *si, struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_IFUP << 1) - 1; //TODO: netif_msg_init()
	ndev->netdev_ops = &enetc_ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;

	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG;
}

static void enetc_port_setup_primary_mac_address(struct enetc_ndev_priv *priv)
{
	unsigned char mac_addr[MAX_ADDR_LEN];
	struct enetc_hw *hw = &priv->si->hw;

	enetc_get_primary_mac_addr(hw, mac_addr);
	if (is_zero_ether_addr(mac_addr)) {
		eth_random_addr(mac_addr);
		dev_info(priv->dev, "no MAC address specified, using %pM\n",
			 mac_addr);
		enetc_set_primary_mac_addr(hw, mac_addr);
	}
}

static void enetc_configure_port(struct enetc_ndev_priv *priv)
{
	struct enetc_hw *hw = &priv->si->hw;
	u32 val;

	val = ENETC_PVCFGR_SET_TXBDR(priv->num_tx_rings);
	val |= ENETC_PVCFGR_SET_RXBDR(priv->num_rx_rings);
	enetc_port_wr(hw, ENETC_PV0CFGR, val);

	enetc_configure_port_mac(priv->si);
	/* enable port */
	enetc_port_wr(hw, ENETC_PMR, ENETC_PMR_EN);

	enetc_port_setup_primary_mac_address(priv);
}

static void enetc_configure_si(struct enetc_ndev_priv *priv)
{
	struct enetc_hw *hw = &priv->si->hw;

	/* enable SI */
	enetc_wr(hw, ENETC_SIMR, ENETC_SIMR_EN);

	/* pick up primary MAC address from SI */
	enetc_get_primary_mac_addr(hw, priv->ndev->dev_addr);
}

static int enetc_alloc_msix(struct enetc_ndev_priv *priv)
{
	int vectors = priv->num_int_vectors;
	int i;

	priv->msix_entries = kcalloc(vectors, sizeof(struct msix_entry),
				     GFP_KERNEL);
	if (!priv->msix_entries)
		return -ENOMEM;

	for (i = 0; i < vectors; i++)
		priv->msix_entries[i].entry = i;

	priv->int_vector = kcalloc(vectors, sizeof(struct enetc_int_vector),
				   GFP_KERNEL);
	if (!priv->int_vector) {
		kfree(priv->msix_entries);
		priv->msix_entries = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < vectors; i++) {
		struct enetc_int_vector *v = &priv->int_vector[i];
		struct enetc_bdr *bdr;

		netif_napi_add(priv->ndev, &v->napi, enetc_poll, 64);

		bdr = &v->tx_ring;
		bdr->index = i;
		bdr->ndev = priv->ndev;
		bdr->dev = priv->dev;
		bdr->bd_count = priv->tx_bd_count;
		priv->tx_ring[i] = bdr;

		bdr = &v->rx_ring;
		bdr->index = i;
		bdr->ndev = priv->ndev;
		bdr->dev = priv->dev;
		bdr->bd_count = priv->rx_bd_count;
		priv->rx_ring[i] = bdr;
	}

	return 0;
}

static void enetc_free_msix(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_int_vectors; i++) {
		struct enetc_int_vector *v = &priv->int_vector[i];

		priv->tx_ring[v->tx_ring.index] = NULL;
		priv->rx_ring[v->rx_ring.index] = NULL;
		netif_napi_del(&v->napi);
	}

	kfree(priv->int_vector);
	kfree(priv->msix_entries);
}

static int enetc_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	struct enetc_hw *hw;
	int err;
	int len;

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

	len = pci_resource_len(pdev, 0);
	hw->reg = ioremap(pci_resource_start(pdev, 0), len);
	if (!hw->reg) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_ioremap;
	}
	if (len > ENETC_PORT_BASE)
		hw->port = hw->reg + ENETC_PORT_BASE;
	if (len > ENETC_GLOBAL_BASE)
		hw->global = hw->reg + ENETC_GLOBAL_BASE;

	ndev = alloc_etherdev_mq(sizeof(*priv), ENETC_MAX_NUM_TXQS);
	if (!ndev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	priv = netdev_priv(ndev);
	enetc_netdev_setup(si, ndev);

	enetc_sw_init(priv);

	enetc_configure_port(priv);
	enetc_configure_si(priv);

	err = enetc_alloc_msix(priv);
	if (err) {
		netif_err(priv, probe, ndev, "MSIX allocation failed\n");
		goto err_alloc_msix;
	}

	err = register_netdev(ndev);
	if (err)
		goto err_reg_netdev;

	netif_carrier_off(ndev);

	netif_info(priv, probe, ndev, "%s v%s\n",
		   enetc_drv_name, enetc_drv_ver);

	return 0;

err_reg_netdev:
	enetc_free_msix(priv);
err_alloc_msix:
	si->ndev = NULL;
	free_netdev(ndev);
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

	unregister_netdev(si->ndev);

	enetc_free_msix(netdev_priv(si->ndev));

	free_netdev(si->ndev);

	iounmap(hw->reg);
	kfree(si);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PCI_IOV
static int enetc_sriov_configure(struct pci_dev *dev, int num_vfs)
{
	int err;

	if (!num_vfs) {
		dev_info(&dev->dev, "SR-IOV stop\n");
		pci_disable_sriov(dev);
	} else {
		dev_info(&dev->dev, "SR-IOV start, %d VFs\n", num_vfs);
		err = pci_enable_sriov(dev, num_vfs);
		if (err) {
			dev_err(&dev->dev, "pci_enable_sriov err %d\n", err);
			return err;
		}
	}

	return num_vfs;
}
#endif

static const struct pci_device_id enetc_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, 0xe100) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_id_table);

static struct pci_driver enetc_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_id_table,
	.probe = enetc_pci_probe,
	.remove = enetc_pci_remove,
#ifdef CONFIG_PCI_IOV
	.sriov_configure = enetc_sriov_configure,
#endif
};
module_pci_driver(enetc_driver);

MODULE_DESCRIPTION("ENETC driver");
MODULE_LICENSE("GPL");
