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


#include "enetc.h"
#include <linux/tcp.h>
#include <linux/udp.h>

static int enetc_map_tx_buffs(struct enetc_bdr *tx_ring, struct sk_buff *skb);
static void enetc_unmap_tx_buff(struct enetc_bdr *tx_ring,
				struct enetc_tx_swbd *tx_swbd);
static int enetc_clean_tx_ring(struct enetc_bdr *tx_ring);

static struct sk_buff *enetc_map_rx_buff_to_skb(struct enetc_bdr *rx_ring,
						int i, u16 size);
static void enetc_add_rx_buff_to_skb(struct enetc_bdr *rx_ring, int i,
				     u16 size, struct sk_buff *skb);
static void enetc_process_skb(struct enetc_bdr *rx_ring, struct sk_buff *skb);
static int enetc_clean_rx_ring(struct enetc_bdr *rx_ring,
			       struct napi_struct *napi, int work_limit);

unsigned int debug = 0;

static void enetc_dbg_print_skb(struct sk_buff *skb, int type)
{
	char *c = skb->data;
	int i;

	netdev_info(skb->dev, "\n[DBG] %s skb->data: %p, len: %d\n",
		(type == RX) ? "RX" : "TX" ,skb->data, skb->len);

	for (i = 0; i < skb->len; i++) {
		if (i % 32 == 0)
			pr_info("%02d: ", i / 32);
		pr_cont("%02X%s", *c++, ((i + 1) % 4 == 0) ? " " : "");
	}
	if (skb_vlan_tag_present(skb))
		pr_info("VLAN tag %04x\n", skb_vlan_tag_get(skb));

	pr_info("\n");
}

static irqreturn_t enetc_msix(int irq, void *data)
{
	struct enetc_int_vector	*v = data;

	/* disable interrupts */
	enetc_wr_reg(v->tbier, 0);
	enetc_wr_reg(v->rbier, 0);

	napi_schedule_irqoff(&v->napi);

	return IRQ_HANDLED;
}

/* max number of fragments + optional extension BD */
#define ENETC_FREE_TXBD_NEEDED (MAX_SKB_FRAGS + 1)

netdev_tx_t enetc_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_bdr *tx_ring;
	int count, prio;

	// TODO: guard against runt (invalid) packets (?)

	if (ndev->num_tc) {
		/* Choose the TX BD ring based on the skb's priority mapping */
		prio = netdev_get_prio_tc_map(ndev, skb->priority);
		tx_ring = priv->tx_ring[prio];
	} else {
		tx_ring = priv->tx_ring[skb->queue_mapping];
	}

	if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED) {
		// TODO: check h/w index (CISR) for more acurate status
		netif_stop_subqueue(ndev, tx_ring->index);
		return NETDEV_TX_BUSY;
	}

	count = enetc_map_tx_buffs(tx_ring, skb);
	if (unlikely(!count)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (enetc_bd_unused(tx_ring) < ENETC_FREE_TXBD_NEEDED)
		// TODO: check h/w index (CISR) for more acurate status
		netif_stop_subqueue(ndev, tx_ring->index);

	return NETDEV_TX_OK;
}

static bool enetc_tx_csum(struct sk_buff *skb, union enetc_tx_bd *txbd)
{
	int l3_start, l3_hsize;
	u16 l3_flags, l4_flags;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return false;

	switch (skb->csum_offset) {
	case offsetof(struct tcphdr, check):
		l4_flags = ENETC_TXBD_L4_TCP;
		break;
	case offsetof(struct udphdr, check):
		l4_flags = ENETC_TXBD_L4_UDP;
		break;
	default:
		skb_checksum_help(skb);
		return false;
	}

	l3_start = skb_network_offset(skb);
	l3_hsize = skb_network_header_len(skb);

	l3_flags = 0;
	if (skb->protocol == htons(ETH_P_IPV6))
		l3_flags = ENETC_TXBD_L3_IPV6;
	else if (skb->protocol != htons(ETH_P_IP))
		WARN_ON(1); //FIXME: Debug only (remove from final code)

	/* write BD fields */
	txbd->l3_csoff = enetc_txbd_l3_csoff(l3_start, l3_hsize, l3_flags);
	txbd->l4_csoff = l4_flags;

	return true;
}

static int enetc_map_tx_buffs(struct enetc_bdr *tx_ring, struct sk_buff *skb)
{
	unsigned int nr_frags = skb_shinfo(skb)->nr_frags;
	struct enetc_tx_swbd *tx_swbd;
	struct skb_frag_struct *frag;
	int len = skb_headlen(skb);
	union enetc_tx_bd *txbd;
	int i, start, count = 0;
	bool do_vlan, do_ts;
	unsigned int f;
	dma_addr_t dma;
	u8 flags = 0;

	i = tx_ring->next_to_use;
	start = tx_ring->next_to_use;

	dma = dma_map_single(tx_ring->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(tx_ring->dev, dma)))
		goto dma_err;

	txbd = ENETC_TXBD(*tx_ring, i);
	txbd->addr = cpu_to_le64(dma);
	txbd->buf_len = cpu_to_le16(len);

	tx_swbd = &tx_ring->tx_swbd[i];
	tx_swbd->dma = dma;
	tx_swbd->len = len;
	tx_swbd->is_dma_page = 0;
	count++;

	do_vlan = skb_vlan_tag_present(skb);
	do_ts = skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP;

	if (do_vlan || do_ts)
		flags |= ENETC_TXBD_FLAGS_EX;

	if (enetc_tx_csum(skb, txbd))
		flags |= ENETC_TXBD_FLAGS_CSUM | ENETC_TXBD_FLAGS_L4CS;

	/* first BD needs frm_len set */
	txbd->frm_len = cpu_to_le16(skb->len);
	/* last BD needs 'F' bit set */
	if (!nr_frags && !(flags & ENETC_TXBD_FLAGS_EX))
		flags |= ENETC_TXBD_FLAGS_F;

	txbd->flags = flags;

	if (flags & ENETC_TXBD_FLAGS_EX) {
		/* add extension BD for VLAN and/or timestamping */
		tx_swbd++;
		txbd++;
		i++;
		if (unlikely(i == tx_ring->bd_count)) {
			i = 0;
			tx_swbd = tx_ring->tx_swbd;
			txbd = ENETC_TXBD(*tx_ring, 0);
		}

		if (do_vlan) {
			txbd->ext.vid = cpu_to_le16(skb_vlan_tag_get(skb));
			txbd->ext.tpid = 0; /* < C-TAG */
			txbd->ext.e_flags |= 1; /* < do VLAN */
		}

		if (do_ts) {
			// TODO: Tx timestamp offload h/w settings
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		}

		if (!nr_frags)
			txbd->ext.flags = ENETC_TXBD_FLAGS_F;

		count++;
	}

	frag = &skb_shinfo(skb)->frags[0];
	for (f = 0; f < nr_frags; f++, frag++) {
		len = skb_frag_size(frag);
		dma = skb_frag_dma_map(tx_ring->dev, frag, 0, len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(tx_ring->dev, dma))
			goto dma_err;

		tx_swbd++;
		txbd++;
		i++;
		if (unlikely(i == tx_ring->bd_count)) {
			i = 0;
			tx_swbd = tx_ring->tx_swbd;
			txbd = ENETC_TXBD(*tx_ring, 0);
		}

		txbd->addr = cpu_to_le64(dma);
		txbd->buf_len = cpu_to_le16(len);

		tx_swbd->dma = dma;
		tx_swbd->len = len;
		tx_swbd->is_dma_page = 1;
		count++;
	}

	if (nr_frags)
		/* last BD needs 'F' bit set */
		txbd->flags = ENETC_TXBD_FLAGS_F;

	tx_ring->tx_swbd[i].skb = skb;

	enetc_bdr_idx_inc(tx_ring, &i);
	tx_ring->next_to_use = i;

	if (debug)
		enetc_dbg_print_skb(skb, TX);
	/* let H/W know BD ring has been updated */
	enetc_wr_reg(tx_ring->tcir, i); /* includes wmb() */

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

	/* enable interrupts */
	enetc_wr_reg(v->tbier, ENETC_TBIER_TXTIE);
	enetc_wr_reg(v->rbier, ENETC_RBIER_RXTIE);

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

static int enetc_bd_ready_count(struct enetc_bdr *tx_ring, int ci)
{
	int pi = enetc_rd_reg(tx_ring->tcisr) & ENETC_TBCISR_IDX_MASK;

	return pi >= ci ? pi - ci : tx_ring->bd_count - ci + pi;
}

static int enetc_clean_tx_ring(struct enetc_bdr *tx_ring)
{
	struct net_device *ndev = tx_ring->ndev;
	int tx_frm_cnt = 0, tx_byte_cnt = 0;
	struct enetc_tx_swbd *tx_swbd;
	int i, bds_to_clean;

	i = tx_ring->next_to_clean;
	tx_swbd = &tx_ring->tx_swbd[i];
	bds_to_clean = enetc_bd_ready_count(tx_ring, i);

	while (bds_to_clean) {
		bool is_eof = !!tx_swbd->skb;

		enetc_unmap_tx_buff(tx_ring, tx_swbd);
		tx_byte_cnt += tx_swbd->len;

		bds_to_clean--;
		tx_swbd++;
		i++;
		if (unlikely(i == tx_ring->bd_count)) {
			i = 0;
			tx_swbd = tx_ring->tx_swbd;
		}

		if (is_eof) {
			tx_frm_cnt++;
			/* re-arm interrupt source */
			enetc_wr_reg(tx_ring->idr, BIT(tx_ring->index) |
				     BIT(16 + tx_ring->index));
		}

		if (unlikely(!bds_to_clean))
			bds_to_clean = enetc_bd_ready_count(tx_ring, i);
	}

	tx_ring->next_to_clean = i;
	tx_ring->stats.packets += tx_frm_cnt;
	tx_ring->stats.bytes += tx_byte_cnt;

	if (unlikely(tx_frm_cnt && netif_carrier_ok(ndev) &&
		     __netif_subqueue_stopped(ndev, tx_ring->index) &&
		     (enetc_bd_unused(tx_ring) >= ENETC_FREE_TXBD_NEEDED))) {
		netif_wake_subqueue(ndev, tx_ring->index);
	}

	return tx_frm_cnt;
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

static void enetc_get_offloads(struct enetc_bdr *rx_ring,
			       union enetc_rx_bd *rxbd, struct sk_buff *skb)
{
	// TODO: checksum, tstamp, VLAN, hash
	if (rx_ring->ndev->features & NETIF_F_RXCSUM) {
		u16 inet_csum = le16_to_cpu(rxbd->r.inet_csum);

		skb->csum = csum_unfold((__force __sum16)~htons(inet_csum));
		skb->ip_summed = CHECKSUM_COMPLETE;
	}

	/* copy VLAN to skb, if one is extracted, for now we assume it's a
	 * standard TPID, but HW also supports custom values
	 */
	if (rxbd->r.flags & ENETC_RXBD_FLAG_VLAN)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
				       rxbd->r.vlan_opt);
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

		enetc_wr_reg(rx_ring->idr, BIT(rx_ring->index));
		dma_rmb(); /* for readig other rxbd fields */
		size = le16_to_cpu(rxbd->r.buf_len);
		skb = enetc_map_rx_buff_to_skb(rx_ring, i, size);
		if (!skb) {
			// TODO: increase alloc error counter
			break;
		}

		enetc_get_offloads(rx_ring, rxbd, skb);

		cleaned_cnt++;
		rxbd++;
		i++;
		if (unlikely(i == rx_ring->bd_count)) {
			i = 0;
			rxbd = ENETC_RXBD(*rx_ring, 0);
		}

		if (unlikely(bd_status &
			     ENETC_RXBD_LSTATUS(ENETC_RXBD_ERR_MASK))) {
			dev_kfree_skb(skb);
			while (!(bd_status & ENETC_RXBD_LSTATUS_F)) {
				dma_rmb();
				bd_status = le32_to_cpu(rxbd->r.lstatus);
				rxbd++;
				i++;
				if (unlikely(i == rx_ring->bd_count)) {
					i = 0;
					rxbd = ENETC_RXBD(*rx_ring, 0);
				}
			}

			// FIXME: driver ethtool stats instead?
			rx_ring->ndev->stats.rx_dropped++;
			rx_ring->ndev->stats.rx_errors++;

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

		if (debug) {
			skb->dev = rx_ring->ndev;
			enetc_dbg_print_skb(skb, RX);
		}

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
	skb_record_rx_queue(skb, rx_ring->index);
	skb->protocol = eth_type_trans(skb, rx_ring->ndev);
}

/* Probing and Init */
#define ENETC_MAX_RFS_SIZE 64
void enetc_get_si_caps(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	/* find out how many of various resources we have to work with */
	val = enetc_rd(hw, ENETC_SICAPR0);
	/* we expect to have the same number of Rx and Tx rings, but in case
	 * that's not true use the min value
	 */
	si->num_rx_rings = (val >> 16) & 0xff;
	si->num_tx_rings = val & 0xff;
	si->num_fs_entries = enetc_rd(hw, ENETC_SIRFSCAPR) & 0x7f;
	si->num_fs_entries = min(si->num_fs_entries, ENETC_MAX_RFS_SIZE);
}

static int enetc_alloc_txbdr(struct enetc_bdr *txr)
{
	int size;

	txr->tx_swbd = vzalloc(txr->bd_count * sizeof(struct enetc_tx_swbd));
	if (!txr->tx_swbd)
		return -ENOMEM;

	size = txr->bd_count * sizeof(union enetc_tx_bd);
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

	size = txr->bd_count * sizeof(union enetc_tx_bd);

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

static int enetc_alloc_cbdr(struct device *dev, struct enetc_cbdr *cbdr)
{
	int size = cbdr->bd_count * sizeof(struct enetc_cbd);

	cbdr->bd_base = dma_zalloc_coherent(dev, size, &cbdr->bd_dma_base,
					    GFP_KERNEL);
	if (!cbdr->bd_base)
		return -ENOMEM;

	cbdr->next_to_clean = 0;
	cbdr->next_to_use = 0;

	return 0;
}

static void enetc_free_cbdr(struct device *dev, struct enetc_cbdr *cbdr)
{
	int size = cbdr->bd_count * sizeof(struct enetc_cbd);

	dma_free_coherent(dev, size, cbdr->bd_base, cbdr->bd_dma_base);
	cbdr->bd_base = NULL;
}

static void enetc_setup_cbdr(struct enetc_hw *hw, struct enetc_cbdr *cbdr)
{
	WARN_ON(lower_32_bits(cbdr->bd_dma_base) & 0x7f);

	/* set CBDR cache attributes */
	enetc_wr(hw, ENETC_SICAR2,
		 ENETC_SICAR_RD_COHERENT | ENETC_SICAR_WR_COHERENT);

	enetc_wr(hw, ENETC_SICBDRBAR0, lower_32_bits(cbdr->bd_dma_base));
	enetc_wr(hw, ENETC_SICBDRBAR1, upper_32_bits(cbdr->bd_dma_base));
	enetc_wr(hw, ENETC_SICBDRLENR, ENETC_RTBLENR_LEN(cbdr->bd_count));

	enetc_wr(hw, ENETC_SICBDRCIR, 0);
	enetc_wr(hw, ENETC_SICBDRCISR, 0);

	/* enable ring */
	enetc_wr(hw, ENETC_SICBDRMR, BIT(31));

	cbdr->cir = hw->reg + ENETC_SICBDRCIR;
	cbdr->cisr = hw->reg + ENETC_SICBDRCISR;
}

static void enetc_configure_si(struct enetc_ndev_priv *priv)
{
	int rss_table[ENETC_RSS_TABLE_SIZE];
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	int i;

	enetc_setup_cbdr(hw, &si->cbd_ring);
	/* set SI cache attributes */
	enetc_wr(hw, ENETC_SICAR0,
		 ENETC_SICAR_RD_COHERENT | ENETC_SICAR_WR_COHERENT);
	enetc_wr(hw, ENETC_SICAR1, ENETC_SICAR_MSI);
	/* enable SI, TODO: start RSS by default */
	enetc_wr(hw, ENETC_SIMR, ENETC_SIMR_EN /*| ENETC_SIMR_RSSE*/);

	/* Set up RSS table defaults */
	for (i = 0; i < ENETC_RSS_TABLE_SIZE; i++)
		rss_table[i] = i % priv->num_rx_rings;
	/* TODO: fix the size, *2 is just to keep sim happy */
	enetc_set_rss_table(si, rss_table, ENETC_RSS_TABLE_SIZE * 2);

}

int enetc_alloc_si_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_si *si = priv->si;
	int err;

	err = enetc_alloc_cbdr(priv->dev, &si->cbd_ring);
	if (err)
		goto err_alloc_cbdr;

	priv->cls_rules = kcalloc(si->num_fs_entries, sizeof(*priv->cls_rules),
				  GFP_KERNEL);
	if (!priv->cls_rules) {
		err = -ENOMEM;
		goto err_alloc_cls;
	}

	enetc_configure_si(priv);

	return 0;

err_alloc_cls:
	enetc_free_cbdr(priv->dev, &si->cbd_ring);
err_alloc_cbdr:

	return err;
}

void enetc_free_si_resources(struct enetc_ndev_priv *priv)
{
	struct enetc_si *si = priv->si;

	enetc_free_cbdr(priv->dev, &si->cbd_ring);

	kfree(priv->cls_rules);
}

static void enetc_setup_txbdr(struct enetc_hw *hw, struct enetc_bdr *tx_ring)
{
	int idx = tx_ring->index;
	u32 tbmr;

	/* 128B alignment required */
	WARN_ON(lower_32_bits(tx_ring->bd_dma_base) & 0x7f);

	enetc_txbdr_wr(hw, idx, ENETC_TBBAR0,
		       lower_32_bits(tx_ring->bd_dma_base));

	enetc_txbdr_wr(hw, idx, ENETC_TBBAR1,
		       upper_32_bits(tx_ring->bd_dma_base));

	WARN_ON(tx_ring->bd_count & 0x3f); //FIXME: must be multiple of 64

	enetc_txbdr_wr(hw, idx, ENETC_TBLENR,
		       ENETC_RTBLENR_LEN(tx_ring->bd_count));

	enetc_txbdr_wr(hw, idx, ENETC_TBCIR, 0);
	enetc_txbdr_wr(hw, idx, ENETC_TBCISR, 0);

	/* enable Tx ints by setting pkt thr to 1 */
	enetc_txbdr_wr(hw, idx, ENETC_TBICIR0, ENETC_TBICIR0_ICEN | 0x1);

	tbmr = ENETC_TBMR_EN;
	if (tx_ring->ndev->features & NETIF_F_HW_VLAN_CTAG_TX)
		tbmr |= ENETC_TBMR_VIH;

	/* enable ring */
	enetc_txbdr_wr(hw, idx, ENETC_TBMR, tbmr);

	tx_ring->tcir = hw->reg + ENETC_BDR(TX, idx, ENETC_TBCIR);
	tx_ring->tcisr = hw->reg + ENETC_BDR(TX, idx, ENETC_TBCISR);
	tx_ring->idr = hw->reg + ENETC_SITXIDR;
}

static void enetc_setup_rxbdr(struct enetc_hw *hw, struct enetc_bdr *rx_ring)
{
	int idx = rx_ring->index;
	u32 rbmr;

	/* 128B alignment required */
	WARN_ON(lower_32_bits(rx_ring->bd_dma_base) & 0x7f);

	enetc_rxbdr_wr(hw, idx, ENETC_RBBAR0,
		       lower_32_bits(rx_ring->bd_dma_base));

	enetc_rxbdr_wr(hw, idx, ENETC_RBBAR1,
		       upper_32_bits(rx_ring->bd_dma_base));

	WARN_ON(rx_ring->bd_count & 0x3f); //FIXME: must be multiple of 64

	enetc_rxbdr_wr(hw, idx, ENETC_RBLENR,
		       ENETC_RTBLENR_LEN(rx_ring->bd_count));

	enetc_rxbdr_wr(hw, idx, ENETC_RBBSR, ENETC_RXB_DMA_SIZE);

	enetc_rxbdr_wr(hw, idx, ENETC_RBPIR, 0);

	/* enable Rx ints by setting pkt thr to 1 (BG 0.7) */
	enetc_rxbdr_wr(hw, idx, ENETC_RBICIR0, ENETC_RBICIR0_ICEN | 0x1);

	rbmr = ENETC_RBMR_EN;
	if (rx_ring->ndev->features & NETIF_F_HW_VLAN_CTAG_RX)
		rbmr |= ENETC_RBMR_VTE;

	rx_ring->rcir = hw->reg + ENETC_BDR(RX, idx, ENETC_RBCIR);
	rx_ring->idr = hw->reg + ENETC_SIRXIDR;

	enetc_refill_rx_ring(rx_ring, enetc_bd_unused(rx_ring));

	/* enable ring */
	enetc_rxbdr_wr(hw, idx, ENETC_RBMR, rbmr);
}

static void enetc_setup_bdrs(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_tx_rings; i++)
		enetc_setup_txbdr(&priv->si->hw, priv->tx_ring[i]);

	for (i = 0; i < priv->num_rx_rings; i++)
		enetc_setup_rxbdr(&priv->si->hw, priv->rx_ring[i]);
}

int enetc_setup_irqs(struct enetc_ndev_priv *priv)
{
	struct pci_dev *pdev = priv->si->pdev;
	int i, err;

	for (i = 0; i < priv->bdr_int_num; i++) {
		int irq = pci_irq_vector(pdev, ENETC_BDR_INT_BASE_IDX + i);
		struct enetc_int_vector *v = &priv->int_vector[i];
		struct enetc_hw *hw = &priv->si->hw;

		sprintf(v->name, "%s-rxtx%d", priv->ndev->name, i);
		err = request_irq(irq, enetc_msix, 0, v->name, v);
		if (err) {
			dev_err(priv->dev, "request_irq() failed!\n");
			goto irq_err;
		}

		v->tbier = hw->reg + ENETC_BDR(TX, i, ENETC_TBIER);
		v->rbier = hw->reg + ENETC_BDR(RX, i, ENETC_RBIER);

		enetc_configure_hw_vector(hw, ENETC_BDR_INT_BASE_IDX + i);
	}

	return 0;

irq_err:
	while (i-- > 0)
		free_irq(pci_irq_vector(pdev, ENETC_BDR_INT_BASE_IDX + i),
			 &priv->int_vector[i]);

	return err;
}

void enetc_free_irqs(struct enetc_ndev_priv *priv)
{
	struct pci_dev *pdev = priv->si->pdev;
	int i;

	for (i = 0; i < priv->bdr_int_num; i++)
		free_irq(pci_irq_vector(pdev, ENETC_BDR_INT_BASE_IDX + i),
			 &priv->int_vector[i]);
}

static void enetc_enable_interrupts(struct enetc_ndev_priv *priv)
{
	int i;

	/* enable Tx & Rx event indication */
	for (i = 0; i < priv->bdr_int_num; i++) {
		enetc_txbdr_wr(&priv->si->hw, i,
			       ENETC_TBIER, ENETC_TBIER_TXTIE);
		enetc_rxbdr_wr(&priv->si->hw, i,
			       ENETC_RBIER, ENETC_RBIER_RXTIE);
	}
}

static void enetc_disable_interrupts(struct enetc_ndev_priv *priv)
{
	struct pci_dev *pdev = priv->si->pdev;
	int i;

	for (i = 0; i < priv->bdr_int_num; i++) {
		int irq = pci_irq_vector(pdev, ENETC_BDR_INT_BASE_IDX + i);

		enetc_txbdr_wr(&priv->si->hw, i, ENETC_TBIER, 0);
		enetc_rxbdr_wr(&priv->si->hw, i, ENETC_RBIER, 0);

		synchronize_irq(irq);
	}
}

int enetc_open(struct net_device *ndev)
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

	err = netif_set_real_num_tx_queues(ndev, ENETC_TXQ_PER_TC);
	if (err)
		goto err_set_queues;

	err = netif_set_real_num_rx_queues(ndev, priv->num_rx_rings);
	if (err)
		goto err_set_queues;

	for (i = 0; i < priv->bdr_int_num; i++)
		napi_enable(&priv->int_vector[i].napi);

	enetc_enable_interrupts(priv);

	netif_tx_start_all_queues(ndev);

	return 0;

err_set_queues:
	enetc_free_rx_resources(priv);
err_alloc_rx:
	enetc_free_tx_resources(priv);
err_alloc_tx:

	return err;
}

int enetc_close(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int i;

	netif_carrier_off(ndev);
	netif_tx_stop_all_queues(ndev);

	enetc_disable_interrupts(priv);

	for (i = 0; i < priv->bdr_int_num; i++) {
		napi_synchronize(&priv->int_vector[i].napi);
		napi_disable(&priv->int_vector[i].napi);
	}

	enetc_free_rxtx_rings(priv);
	enetc_free_rx_resources(priv);
	enetc_free_tx_resources(priv);

	return 0;
}

int enetc_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		   void *type_data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct tc_mqprio_qopt *mqprio = type_data;
	struct enetc_bdr *tx_ring;
	int i;
	u8 num_tc;
	u32 val;

	if (type != TC_SETUP_MQPRIO)
		return -EOPNOTSUPP;

	mqprio->hw = TC_MQPRIO_HW_OFFLOAD_TCS;
	num_tc = mqprio->num_tc;

	if (!num_tc) {
		netdev_reset_tc(ndev);
		netif_set_real_num_tx_queues(ndev, ENETC_TXQ_PER_TC);

		/* Reset all ring priorities to 0 */
		for (i = 0; i < priv->num_tx_rings; i++) {
			tx_ring = priv->tx_ring[i];
			val = enetc_txbdr_rd(&priv->si->hw, tx_ring->index,
					     ENETC_TBMR);
			val &= ~ENETC_TBMR_PRIO_MASK;
			enetc_txbdr_wr(&priv->si->hw, tx_ring->index,
				       ENETC_TBMR, val);
		}

		return 0;
	}

	/* Check if we have enough BD rings available to accommodate all TCs */
	if (num_tc > priv->num_tx_rings) {
		netdev_err(ndev, "Max %d traffic classes supported\n",
			   priv->num_tx_rings);
		return -EINVAL;
	}

	/* For the moment, we use only one BD ring per TC.
	 *
	 * Configure num_tc BD rings with increasing priorities.
	 */
	for (i = 0; i < num_tc; i++) {
		tx_ring = priv->tx_ring[i];
		val = enetc_txbdr_rd(&priv->si->hw, tx_ring->index, ENETC_TBMR);
		/* Clear the old priority and set the new one */
		val &= ~ENETC_TBMR_PRIO_MASK;
		val |= ENETC_TBMR_PRIO_SET(i);
		enetc_txbdr_wr(&priv->si->hw, tx_ring->index, ENETC_TBMR, val);
	}

	/* Reset the number of netdev queues based on the TC count */
	netif_set_real_num_tx_queues(ndev, num_tc * ENETC_TXQ_PER_TC);

	netdev_set_num_tc(ndev, num_tc);

	/* Each TC is associated with ENETC_TXQ_PER_TC netdev queues */
	for (i = 0; i < num_tc; i++)
		netdev_set_tc_queue(ndev, i, ENETC_TXQ_PER_TC,
				    i * ENETC_TXQ_PER_TC);

	return 0;
}

struct net_device_stats *enetc_get_stats(struct net_device *ndev)
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

static int enetc_set_rss(struct net_device *ndev, int en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	int cpus;
	u32 reg;

	cpus = min_t(int, num_online_cpus(), priv->si->num_rx_rings);
	enetc_wr(hw, ENETC_SIRBGCR, cpus);

	reg = enetc_rd(hw, ENETC_SIMR);
	reg &= ~ENETC_SIMR_RSSE;
	reg |= (en) ? ENETC_SIMR_RSSE : 0;
	enetc_wr(hw, ENETC_SIMR, reg);

	return 0;
}

int enetc_set_features(struct net_device *ndev,
		       netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;

	if (changed & NETIF_F_RXHASH)
		enetc_set_rss(ndev, !!(features & NETIF_F_RXHASH));

	return 0;
}

int enetc_alloc_msix(struct enetc_ndev_priv *priv)
{
	struct pci_dev *pdev = priv->si->pdev;
	int i, n, nvec;

	nvec = ENETC_BDR_INT_BASE_IDX + priv->bdr_int_num;
	/* allocate MSIX for both messaging and Rx/Tx interrupts */
	n = pci_alloc_irq_vectors(pdev, nvec, nvec, PCI_IRQ_MSIX);

	if (n < 0)
		return n;

	if (n != nvec)
		return -EPERM;

	priv->int_vector = kcalloc(priv->bdr_int_num,
				   sizeof(struct enetc_int_vector), GFP_KERNEL);
	if (!priv->int_vector) {
		pci_free_irq_vectors(pdev);
		return -ENOMEM;
	}

	for (i = 0; i < priv->bdr_int_num; i++) {
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

void enetc_free_msix(struct enetc_ndev_priv *priv)
{
	int i;

	for (i = 0; i < priv->bdr_int_num; i++) {
		struct enetc_int_vector *v = &priv->int_vector[i];

		priv->tx_ring[v->tx_ring.index] = NULL;
		priv->rx_ring[v->rx_ring.index] = NULL;
		netif_napi_del(&v->napi);
	}

	kfree(priv->int_vector);

	/* disable all MSIX for this device */
	pci_free_irq_vectors(priv->si->pdev);
}

static void enetc_kfree_si(struct enetc_si *si)
{
	char *p = (char *)si - si->pad;

	kfree(p);
}

int enetc_pci_probe(struct pci_dev *pdev, const char *name, int sizeof_priv)
{
	struct enetc_si *si, *p;
	struct enetc_hw *hw;
	size_t alloc_size;
	int err, len;

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

	err = pci_request_mem_regions(pdev, name);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);

	alloc_size = sizeof(struct enetc_si);
	if (sizeof_priv) {
		/* align priv to 32B */
		alloc_size = ALIGN(alloc_size, ENETC_SI_ALIGN);
		alloc_size += sizeof_priv;
	}
	/* force 32B alignment for enetc_si */
	alloc_size += ENETC_SI_ALIGN - 1;

	p = kzalloc(alloc_size, GFP_KERNEL);
	if (!p) {
		err = -ENOMEM;
		goto err_alloc_si;
	}

	si = PTR_ALIGN(p, ENETC_SI_ALIGN);
	si->pad = (char *)si - (char *)p;

	pci_set_drvdata(pdev, si);
	si->pdev = pdev;
	hw = &si->hw;

	len = pci_resource_len(pdev, ENETC_BAR_REGS);
	hw->reg = ioremap(pci_resource_start(pdev, ENETC_BAR_REGS), len);
	if (!hw->reg) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_ioremap;
	}
	if (len > ENETC_PORT_BASE)
		hw->port = hw->reg + ENETC_PORT_BASE;
	if (len > ENETC_GLOBAL_BASE)
		hw->global = hw->reg + ENETC_GLOBAL_BASE;

	return 0;

err_ioremap:
	enetc_kfree_si(si);
err_alloc_si:
	pci_release_mem_regions(pdev);
err_pci_mem_reg:
err_dma:
	pci_disable_device(pdev);

	return err;
}

void enetc_pci_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_hw *hw = &si->hw;

	iounmap(hw->reg);
	enetc_kfree_si(si);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}
