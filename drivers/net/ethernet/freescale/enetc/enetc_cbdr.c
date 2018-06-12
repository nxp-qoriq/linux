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

void enetc_clear_mac_flt_entry(struct enetc_si *si, int index)
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

void enetc_set_mac_flt_entry(struct enetc_si *si, int index,
			     char *mac_addr, int si_map)
{
	struct enetc_cbd cbd;
	bool async = false;
	u32 upper;
	u16 lower;
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

	upper = *(const u32 *)mac_addr;
	lower = *(const u16 *)(mac_addr + 4);
	cbd.addr[0] = upper;
	cbd.addr[1] = lower;

	ret = enetc_send_cmd(si, &cbd, async);
	if (ret) {
		pr_err("MAC filter update failed (%d)!\n", ret);
		WARN_ON(1);
		// TODO: fallback to promisc mode
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

static int enetc_cmd_rss_table(struct enetc_si *si, u32 *table, int count,
			       int read)
{
	struct enetc_cbd cbd = {.cmd = 0};
	enum dma_data_direction dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t dma, dma_align;
	const size_t align = 64;
	u8 *tmp, *tmp_align;
	int err, i;

	if (count < 0x40)
		/* HW only takes in a full 64 entry table */
		return -EINVAL;

	tmp = dma_alloc_coherent(&si->pdev->dev, count + align - 1, &dma, dir);
	if (!tmp) {
		netdev_err(si->ndev, "DMA mapping of RSS table failed!\n");
		return -ENOMEM;
	}
	dma_align = ALIGN(dma, align);
	tmp_align = PTR_ALIGN(tmp, align);

	if (!read)
		for (i = 0; i < count; i++)
			tmp_align[i] = (u8)(table[i]);

	/* fill up the descriptor */
	cbd.cmd = read ? 2 : 1;
	cbd.cls = 3;
	cbd.length = count;

	cbd.addr[0] = lower_32_bits(dma_align);
	cbd.addr[1] = upper_32_bits(dma_align);

	err = enetc_send_cmd(si, &cbd, false);
	if (err)
		netdev_err(si->ndev, "RSS cmd failed (%d)!", err);

	if (read)
		for (i = 0; i < count; i++)
			table[i] = tmp_align[i];

	dma_free_coherent(&si->pdev->dev, count, tmp, dma);

	return err;
}

/* Get RSS table */
int enetc_get_rss_table(struct enetc_si *si, u32 *table, int count)
{
	return enetc_cmd_rss_table(si, table, count, true);
}

/* Set RSS table */
int enetc_set_rss_table(struct enetc_si *si, const u32 *table, int count)
{
	return enetc_cmd_rss_table(si, (u32 *)table, count, false);
}
