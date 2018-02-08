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

#define ENETC_RING_UNUSED(R)	(((R)->next_to_clean - (R)->next_to_use - 1 \
				 + (R)->bd_count) % (R)->bd_count)

#define ENETC_CBD(R, i)	(&(((struct enetc_cbd *)((R).bd_base))[i]))

#define ENETC_CBDR_TIMEOUT	1000 /* usecs */

enum enetc_cbdr_stat {
	ENETC_CMD_OK,
	ENETC_CMD_BUSY,
	ENETC_CMD_TIMEOUT,
};

static void enetc_clean_cbdr(struct enetc_si *si)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	struct enetc_cbd *dest_cbd;
	int i;

	i = ring->next_to_clean;

	while (enetc_rd_reg(ring->cisr) != i) {
		dest_cbd = ENETC_CBD(*ring, i);
		if (dest_cbd->status_flags & ENETC_CBD_STATUS_MASK)
			WARN_ON(1);

		memset(dest_cbd, 0, sizeof(*dest_cbd));

		i = (i + 1) % ring->bd_count;
	}

	ring->next_to_clean = i;
}

static int enetc_send_cmd(struct enetc_si *si, struct enetc_cbd *cbd,
			  bool async)
{
	struct enetc_cbdr *ring = &si->cbd_ring;
	struct enetc_cbd *dest_cbd;
	int i;

	if (!ring->bd_base)
		return -EIO;

	if (async && !ENETC_RING_UNUSED(ring)) {
		// TODO: support true async mode, with interrupts
		// and separate cleanup task

		enetc_clean_cbdr(si);
		return ENETC_CMD_BUSY;
	}

	i = ring->next_to_use;
	dest_cbd = ENETC_CBD(*ring, i);

	/* copy command to the ring */
	*dest_cbd = *cbd;
	i = (i + 1) % ring->bd_count;

	ring->next_to_use = i;
	/* let H/W know BD ring has been updated */
	enetc_wr_reg(ring->cir, i);

	if (!async) {
		int timeout = ENETC_CBDR_TIMEOUT;

		do {
			if (enetc_rd_reg(ring->cisr) == i)
				break;
			udelay(10);
			timeout -= 10;
		} while (timeout);

		if (!timeout)
			return ENETC_CMD_TIMEOUT;
	}

	if (!async)
		enetc_clean_cbdr(si);

	return ENETC_CMD_OK;
}

static void enetc_clear_mac_flt_entry(struct enetc_si *si, int index)
{
	struct enetc_cbd cbd;
	bool async = false;
	int ret;

	memset(&cbd, 0, sizeof(cbd));

	cbd.cls = 1;
	cbd.status_flags = ENETC_CBD_FLAGS_SF;
	if (async)
		cbd.status_flags |= ENETC_CBD_FLAGS_IE;

	cbd.index = cpu_to_le16(index);

	ret = enetc_send_cmd(si, &cbd, async);
	if (ret) {
		pr_err("MAC filter clear failed (%d)!\n", ret);
		WARN_ON(1);
	}
}

static void enetc_set_mac_flt_entry(struct enetc_si *si, int index,
				    char *mac_addr, int si_map)
{
	struct enetc_cbd cbd;
	bool async = false;
	u16 upper;
	u32 lower;
	int ret;

	memset(&cbd, 0, sizeof(cbd));

	/* fill up the "set" descriptor */
	cbd.cls = 1;
	cbd.status_flags = ENETC_CBD_FLAGS_SF;
	if (async)
		cbd.status_flags |= ENETC_CBD_FLAGS_IE;

	cbd.index = cpu_to_le16(index);
	cbd.opt[3] = cpu_to_le32(si_map);
	/* enable entry */
	cbd.opt[0] = cpu_to_le32(BIT(31));

	upper = ntohs(*(const u16 *)mac_addr);
	lower = ntohl(*(const u32 *)(mac_addr + 2));
	cbd.addr[0] = cpu_to_le32(lower);
	cbd.addr[1] = cpu_to_le16(upper);

	ret = enetc_send_cmd(si, &cbd, async);
	if (ret) {
		pr_err("MAC filter update failed (%d)!\n", ret);
		WARN_ON(1);
		// TODO: fallback to promisc mode
	}
}

static void enetc_clear_mac_ht_flt(struct enetc_hw *hw, bool mc)
{
	if (mc) {
		enetc_wr(hw, ENETC_MMHFTR0, 0);
		enetc_wr(hw, ENETC_MMHFTR1, 0);
	} else {
		enetc_wr(hw, ENETC_UMHFTR0, 0);
		enetc_wr(hw, ENETC_UMHFTR1, 0);
	}
}

static void enetc_set_mac_ht_flt(struct enetc_hw *hw, u32 *hash, bool mc)
{
	if (mc) {
		enetc_wr(hw, ENETC_MMHFTR0, *hash);
		enetc_wr(hw, ENETC_MMHFTR1, *(hash + 1));
	} else {
		enetc_wr(hw, ENETC_UMHFTR0, *hash);
		enetc_wr(hw, ENETC_UMHFTR1, *(hash + 1));
	}
}

/* MAC Address Filter Table Entry Set Descriptor */
void enetc_sync_mac_filters(struct enetc_si *si, struct enetc_mac_filter *tbl,
			    int si_idx)
{
	struct enetc_mac_filter *f;
	int i, pos;

	if (!enetc_si_is_pf(si)) {
		pr_err("VFs not allowed to change MAC addr filters!\n");
		return;
	}

	f = &tbl[si_idx * MADDR_TYPE];
	pos = EMETC_MAC_ADDR_FILT_RES + si_idx;

	for (i = 0; i < MADDR_TYPE; i++, f++) {
		bool em = (f->mac_addr_cnt == 1) && (i == UC);
		bool clear = !f->mac_addr_cnt;
		bool mc = (i == MC);

		if (clear) {
			if (!mc)
				enetc_clear_mac_flt_entry(si, pos);

			enetc_clear_mac_ht_flt(&si->hw, mc);
			continue;
		}

		/* exact match filter */
		if (em) {
			enetc_set_mac_flt_entry(si, pos, f->mac_addr,
						BIT(si_idx));
			continue;
		}

		/* hash table filter */
		enetc_set_mac_ht_flt(&si->hw, (u32 *)f->mac_hash_table, mc);
	}
}

/* Set entry in RFS table */
int enetc_set_fs_entry(struct enetc_si *si, struct enetc_cmd_rfse *rfse,
		       int index)
{
	struct enetc_cbd cbd = {.cmd = 0};
	bool async = false;
	dma_addr_t dma;
	int err;

	/* fill up the "set" descriptor */
	cbd.cmd = 0;
	cbd.cls = 4;
	cbd.index = cpu_to_le16(index);
	cbd.length = cpu_to_le16(sizeof(*rfse));
	cbd.opt[3] = cpu_to_le32(0); /* SI */

	dma = dma_map_single(&si->pdev->dev, rfse, cbd.length,
			     DMA_TO_DEVICE);
	if (dma_mapping_error(&si->pdev->dev, dma)) {
		netdev_err(si->ndev, "DMA mapping of RFS entry failed!\n");
		return -ENOMEM;
	}

	cbd.addr[0] = lower_32_bits(dma);
	cbd.addr[1] = upper_32_bits(dma);

	if (async)
		cbd.status_flags |= ENETC_CBD_FLAGS_IE;

	err = enetc_send_cmd(si, &cbd, async);
	if (err)
		netdev_err(si->ndev, "FS entry add failed (%d)!", err);
	dma_unmap_single(&si->pdev->dev, dma, cbd.length, DMA_TO_DEVICE);

	return err;
}

/* Set RSS table */
int enetc_set_rss_table(struct enetc_si *si, u16 *table, int len)
{
	struct enetc_cbd cbd = {.cmd = 0};
	dma_addr_t dma;
	int err;

	if (len < 0x80)
		/* HW only takes in a full 64 entry/128B table */
		return -EINVAL;

	/* fill up the "set" descriptor */
	cbd.cmd = 1;
	cbd.cls = 3;
	cbd.length = len;

	dma = dma_map_single(&si->pdev->dev, table, len, DMA_TO_DEVICE);
	if (dma_mapping_error(&si->pdev->dev, dma)) {
		netdev_err(si->ndev, "DMA mapping of RSS table failed!\n");
		return -ENOMEM;
	}

	cbd.addr[0] = lower_32_bits(dma);
	cbd.addr[1] = upper_32_bits(dma);

	err = enetc_send_cmd(si, &cbd, false);
	if (err)
		netdev_err(si->ndev, "RSS table update failed (%d)!", err);
	dma_unmap_single(&si->pdev->dev, dma, cbd.length, DMA_TO_DEVICE);

	return err;
}
