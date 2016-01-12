/* Copyright 2014-2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/of_net.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/net_tstamp.h>

#include "../../fsl-mc/include/mc.h"
#include "../../fsl-mc/include/mc-sys.h"
#include "dpaa2-eth.h"

/* CREATE_TRACE_POINTS only needs to be defined once. Other dpa files
 * using trace events only need to #include <trace/events/sched.h>
 */
#define CREATE_TRACE_POINTS
#include "dpaa2-eth-trace.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("Freescale DPAA2 Ethernet Driver");

/* Oldest DPAA2 objects version we are compatible with */
#define DPAA2_SUPPORTED_DPNI_VERSION	6
#define DPAA2_SUPPORTED_DPBP_VERSION	2
#define DPAA2_SUPPORTED_DPCON_VERSION	2

static void validate_rx_csum(struct dpaa2_eth_priv *priv,
			     u32 fd_status,
			     struct sk_buff *skb)
{
	skb_checksum_none_assert(skb);

	/* HW checksum validation is disabled, nothing to do here */
	if (!(priv->net_dev->features & NETIF_F_RXCSUM))
		return;

	/* Read checksum validation bits */
	if (!((fd_status & DPAA2_FAS_L3CV) &&
	      (fd_status & DPAA2_FAS_L4CV)))
		return;

	/* Inform the stack there's no need to compute L3/L4 csum anymore */
	skb->ip_summed = CHECKSUM_UNNECESSARY;
}

/* Free a received FD.
 * Not to be used for Tx conf FDs or on any other paths.
 */
static void free_rx_fd(struct dpaa2_eth_priv *priv,
		       const struct dpaa2_fd *fd,
		       void *vaddr)
{
	struct device *dev = priv->net_dev->dev.parent;
	dma_addr_t addr = dpaa2_fd_get_addr(fd);
	u8 fd_format = dpaa2_fd_get_format(fd);
	struct dpaa2_sg_entry *sgt;
	void *sg_vaddr;
	int i;

	/* If single buffer frame, just free the data buffer */
	if (fd_format == dpaa2_fd_single)
		goto free_buf;

	/* For S/G frames, we first need to free all SG entries */
	sgt = vaddr + dpaa2_fd_get_offset(fd);
	for (i = 0; i < DPAA2_ETH_MAX_SG_ENTRIES; i++) {
		dpaa2_sg_le_to_cpu(&sgt[i]);

		addr = dpaa2_sg_get_addr(&sgt[i]);
		dma_unmap_single(dev, addr, DPAA2_ETH_RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		sg_vaddr = phys_to_virt(addr);
		put_page(virt_to_head_page(sg_vaddr));

		if (dpaa2_sg_is_final(&sgt[i]))
			break;
	}

free_buf:
	put_page(virt_to_head_page(vaddr));
}

/* Build a linear skb based on a single-buffer frame descriptor */
static struct sk_buff *build_linear_skb(struct dpaa2_eth_priv *priv,
					struct dpaa2_eth_channel *ch,
					const struct dpaa2_fd *fd,
					void *fd_vaddr)
{
	struct sk_buff *skb = NULL;
	u16 fd_offset = dpaa2_fd_get_offset(fd);
	u32 fd_length = dpaa2_fd_get_len(fd);

	skb = build_skb(fd_vaddr, DPAA2_ETH_RX_BUF_SIZE +
			SKB_DATA_ALIGN(sizeof(struct skb_shared_info)));
	if (unlikely(!skb))
		return NULL;

	skb_reserve(skb, fd_offset);
	skb_put(skb, fd_length);

	ch->buf_count--;

	return skb;
}

/* Build a non linear (fragmented) skb based on a S/G table */
static struct sk_buff *build_frag_skb(struct dpaa2_eth_priv *priv,
				      struct dpaa2_eth_channel *ch,
				      struct dpaa2_sg_entry *sgt)
{
	struct sk_buff *skb = NULL;
	struct device *dev = priv->net_dev->dev.parent;
	void *sg_vaddr;
	dma_addr_t sg_addr;
	u16 sg_offset;
	u32 sg_length;
	struct page *page, *head_page;
	int page_offset;
	int i;

	for (i = 0; i < DPAA2_ETH_MAX_SG_ENTRIES; i++) {
		struct dpaa2_sg_entry *sge = &sgt[i];

		dpaa2_sg_le_to_cpu(sge);

		/* NOTE: We only support SG entries in dpaa2_sg_single format,
		 * but this is the only format we may receive from HW anyway
		 */

		/* Get the address and length from the S/G entry */
		sg_addr = dpaa2_sg_get_addr(sge);
		dma_unmap_single(dev, sg_addr, DPAA2_ETH_RX_BUF_SIZE,
				 DMA_FROM_DEVICE);

		sg_vaddr = phys_to_virt(sg_addr);
		sg_length = dpaa2_sg_get_len(sge);

		if (i == 0) {
			/* We build the skb around the first data buffer */
			skb = build_skb(sg_vaddr, DPAA2_ETH_RX_BUF_SIZE +
				SKB_DATA_ALIGN(sizeof(struct skb_shared_info)));
			if (unlikely(!skb))
				return NULL;

			sg_offset = dpaa2_sg_get_offset(sge);
			skb_reserve(skb, sg_offset);
			skb_put(skb, sg_length);
		} else {
			/* Rest of the data buffers are stored as skb frags */
			page = virt_to_page(sg_vaddr);
			head_page = virt_to_head_page(sg_vaddr);

			/* Offset in page (which may be compound).
			 * Data in subsequent SG entries is stored from the
			 * beginning of the buffer, so we don't need to add the
			 * sg_offset.
			 */
			page_offset = ((unsigned long)sg_vaddr &
				(PAGE_SIZE - 1)) +
				(page_address(page) - page_address(head_page));

			skb_add_rx_frag(skb, i - 1, head_page, page_offset,
					sg_length, DPAA2_ETH_RX_BUF_SIZE);
		}

		if (dpaa2_sg_is_final(sge))
			break;
	}

	/* Count all data buffers + SG table buffer */
	ch->buf_count -= i + 2;

	return skb;
}

/* Main Rx frame processing routine */
static void dpaa2_eth_rx(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_channel *ch,
			 const struct dpaa2_fd *fd,
			 struct napi_struct *napi)
{
	dma_addr_t addr = dpaa2_fd_get_addr(fd);
	u8 fd_format = dpaa2_fd_get_format(fd);
	void *vaddr;
	struct sk_buff *skb;
	struct rtnl_link_stats64 *percpu_stats;
	struct dpaa2_eth_drv_stats *percpu_extras;
	struct device *dev = priv->net_dev->dev.parent;
	struct dpaa2_fas *fas;
	u32 status = 0;

	/* Tracing point */
	trace_dpaa2_rx_fd(priv->net_dev, fd);

	dma_unmap_single(dev, addr, DPAA2_ETH_RX_BUF_SIZE, DMA_FROM_DEVICE);
	vaddr = phys_to_virt(addr);

	prefetch(vaddr + priv->buf_layout.private_data_size);
	prefetch(vaddr + dpaa2_fd_get_offset(fd));

	percpu_stats = this_cpu_ptr(priv->percpu_stats);
	percpu_extras = this_cpu_ptr(priv->percpu_extras);

	if (fd_format == dpaa2_fd_single) {
		skb = build_linear_skb(priv, ch, fd, vaddr);
	} else if (fd_format == dpaa2_fd_sg) {
		struct dpaa2_sg_entry *sgt =
				vaddr + dpaa2_fd_get_offset(fd);
		skb = build_frag_skb(priv, ch, sgt);
		put_page(virt_to_head_page(vaddr));
		percpu_extras->rx_sg_frames++;
		percpu_extras->rx_sg_bytes += dpaa2_fd_get_len(fd);
	} else {
		/* We don't support any other format */
		goto err_frame_format;
	}

	if (unlikely(!skb))
		goto err_build_skb;

	prefetch(skb->data);

	/* Get the timestamp value */
	if (priv->ts_rx_en) {
		struct skb_shared_hwtstamps *shhwtstamps = skb_hwtstamps(skb);
		u64 *ns = (u64 *)(vaddr +
				  priv->buf_layout.private_data_size +
				  sizeof(struct dpaa2_fas));

		*ns = DPAA2_PTP_NOMINAL_FREQ_PERIOD_NS * (*ns);
		memset(shhwtstamps, 0, sizeof(*shhwtstamps));
		shhwtstamps->hwtstamp = ns_to_ktime(*ns);
	}

	/* Check if we need to validate the L4 csum */
	if (likely(fd->simple.frc & DPAA2_FD_FRC_FASV)) {
		fas = (struct dpaa2_fas *)
				(vaddr + priv->buf_layout.private_data_size);
		status = le32_to_cpu(fas->status);
		validate_rx_csum(priv, status, skb);
	}

	skb->protocol = eth_type_trans(skb, priv->net_dev);

	percpu_stats->rx_packets++;
	percpu_stats->rx_bytes += skb->len;

	if (priv->net_dev->features & NETIF_F_GRO)
		napi_gro_receive(napi, skb);
	else
		netif_receive_skb(skb);

	return;
err_frame_format:
err_build_skb:
	free_rx_fd(priv, fd, vaddr);
	percpu_stats->rx_dropped++;
}

#ifdef CONFIG_FSL_DPAA2_ETH_USE_ERR_QUEUE
/* Processing of Rx frames received on the error FQ
 * We check and print the error bits and then free the frame
 */
static void dpaa2_eth_rx_err(struct dpaa2_eth_priv *priv,
			     struct dpaa2_eth_channel *ch,
			     const struct dpaa2_fd *fd,
			     struct napi_struct *napi __always_unused)
{
	struct device *dev = priv->net_dev->dev.parent;
	dma_addr_t addr = dpaa2_fd_get_addr(fd);
	void *vaddr;
	struct rtnl_link_stats64 *percpu_stats;
	struct dpaa2_fas *fas;
	u32 status = 0;

	dma_unmap_single(dev, addr, DPAA2_ETH_RX_BUF_SIZE, DMA_FROM_DEVICE);
	vaddr = phys_to_virt(addr);

	if (fd->simple.frc & DPAA2_FD_FRC_FASV) {
		fas = (struct dpaa2_fas *)
			(vaddr + priv->buf_layout.private_data_size);
		status = le32_to_cpu(fas->status);
		if (net_ratelimit())
			netdev_warn(priv->net_dev, "Rx frame error: 0x%08x\n",
				    status & DPAA2_ETH_RX_ERR_MASK);
	}
	free_rx_fd(priv, fd, vaddr);

	percpu_stats = this_cpu_ptr(priv->percpu_stats);
	percpu_stats->rx_errors++;
}
#endif

/* Consume all frames pull-dequeued into the store. This is the simplest way to
 * make sure we don't accidentally issue another volatile dequeue which would
 * overwrite (leak) frames already in the store.
 *
 * Observance of NAPI budget is not our concern, leaving that to the caller.
 */
static int consume_frames(struct dpaa2_eth_channel *ch)
{
	struct dpaa2_eth_priv *priv = ch->priv;
	struct dpaa2_eth_fq *fq;
	struct dpaa2_dq *dq;
	const struct dpaa2_fd *fd;
	int cleaned = 0;
	int is_last;

	do {
		dq = dpaa2_io_store_next(ch->store, &is_last);
		if (unlikely(!dq)) {
			/* If we're here, we *must* have placed a
			 * volatile dequeue comnmand, so keep reading through
			 * the store until we get some sort of valid response
			 * token (either a valid frame or an "empty dequeue")
			 */
			continue;
		}

		fd = dpaa2_dq_fd(dq);
		fq = (struct dpaa2_eth_fq *)dpaa2_dq_fqd_ctx(dq);
		fq->stats.frames++;

		fq->consume(priv, ch, fd, &ch->napi);
		cleaned++;
	} while (!is_last);

	return cleaned;
}

/* Create a frame descriptor based on a fragmented skb */
static int build_sg_fd(struct dpaa2_eth_priv *priv,
		       struct sk_buff *skb,
		       struct dpaa2_fd *fd)
{
	struct device *dev = priv->net_dev->dev.parent;
	void *sgt_buf = NULL;
	dma_addr_t addr;
	int nr_frags = skb_shinfo(skb)->nr_frags;
	struct dpaa2_sg_entry *sgt;
	int i, j, err;
	int sgt_buf_size;
	struct scatterlist *scl, *crt_scl;
	int num_sg;
	int num_dma_bufs;
	struct dpaa2_eth_swa *swa;

	/* Create and map scatterlist.
	 * We don't advertise NETIF_F_FRAGLIST, so skb_to_sgvec() will not have
	 * to go beyond nr_frags+1.
	 * Note: We don't support chained scatterlists
	 */
	if (unlikely(PAGE_SIZE / sizeof(struct scatterlist) < nr_frags + 1))
		return -EINVAL;

	scl = kcalloc(nr_frags + 1, sizeof(struct scatterlist), GFP_ATOMIC);
	if (unlikely(!scl))
		return -ENOMEM;

	sg_init_table(scl, nr_frags + 1);
	num_sg = skb_to_sgvec(skb, scl, 0, skb->len);
	num_dma_bufs = dma_map_sg(dev, scl, num_sg, DMA_TO_DEVICE);
	if (unlikely(!num_dma_bufs)) {
		err = -ENOMEM;
		goto dma_map_sg_failed;
	}

	/* Prepare the HW SGT structure */
	sgt_buf_size = priv->tx_data_offset +
		       sizeof(struct dpaa2_sg_entry) * (1 + num_dma_bufs);
	sgt_buf = kzalloc(sgt_buf_size + DPAA2_ETH_TX_BUF_ALIGN, GFP_ATOMIC);
	if (unlikely(!sgt_buf)) {
		err = -ENOMEM;
		goto sgt_buf_alloc_failed;
	}
	sgt_buf = PTR_ALIGN(sgt_buf, DPAA2_ETH_TX_BUF_ALIGN);

	/* PTA from egress side is passed as is to the confirmation side so
	 * we need to clear some fields here in order to find consistent values
	 * on TX confirmation. We are clearing FAS (Frame Annotation Status)
	 * field here.
	 */
	memset(sgt_buf + priv->buf_layout.private_data_size, 0, 8);

	sgt = (struct dpaa2_sg_entry *)(sgt_buf + priv->tx_data_offset);

	/* Fill in the HW SGT structure.
	 *
	 * sgt_buf is zeroed out, so the following fields are implicit
	 * in all sgt entries:
	 *   - offset is 0
	 *   - format is 'dpaa2_sg_single'
	 */
	for_each_sg(scl, crt_scl, num_dma_bufs, i) {
		dpaa2_sg_set_addr(&sgt[i], sg_dma_address(crt_scl));
		dpaa2_sg_set_len(&sgt[i], sg_dma_len(crt_scl));
	}
	dpaa2_sg_set_final(&sgt[i - 1], true);

	/* Store the skb backpointer in the SGT buffer.
	 * Fit the scatterlist and the number of buffers alongside the
	 * skb backpointer in the SWA. We'll need all of them on Tx Conf.
	 */
	swa = (struct dpaa2_eth_swa *)sgt_buf;
	swa->skb = skb;
	swa->scl = scl;
	swa->num_sg = num_sg;
	swa->num_dma_bufs = num_dma_bufs;

	/* Hardware expects the SG table to be in little endian format */
	for (j = 0; j < i; j++)
		dpaa2_sg_cpu_to_le(&sgt[j]);

	/* Separately map the SGT buffer */
	addr = dma_map_single(dev, sgt_buf, sgt_buf_size, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, addr))) {
		err = -ENOMEM;
		goto dma_map_single_failed;
	}
	dpaa2_fd_set_offset(fd, priv->tx_data_offset);
	dpaa2_fd_set_format(fd, dpaa2_fd_sg);
	dpaa2_fd_set_addr(fd, addr);
	dpaa2_fd_set_len(fd, skb->len);

	fd->simple.ctrl = DPAA2_FD_CTRL_ASAL | DPAA2_FD_CTRL_PTA |
			  DPAA2_FD_CTRL_PTV1;

	return 0;

dma_map_single_failed:
	kfree(sgt_buf);
sgt_buf_alloc_failed:
	dma_unmap_sg(dev, scl, num_sg, DMA_TO_DEVICE);
dma_map_sg_failed:
	kfree(scl);
	return err;
}

/* Create a frame descriptor based on a linear skb */
static int build_single_fd(struct dpaa2_eth_priv *priv,
			   struct sk_buff *skb,
			   struct dpaa2_fd *fd)
{
	struct device *dev = priv->net_dev->dev.parent;
	u8 *buffer_start;
	struct sk_buff **skbh;
	dma_addr_t addr;

	buffer_start = PTR_ALIGN(skb->data - priv->tx_data_offset -
				 DPAA2_ETH_TX_BUF_ALIGN,
				 DPAA2_ETH_TX_BUF_ALIGN);

	/* PTA from egress side is passed as is to the confirmation side so
	 * we need to clear some fields here in order to find consistent values
	 * on TX confirmation. We are clearing FAS (Frame Annotation Status)
	 * field here.
	 */
	memset(buffer_start + priv->buf_layout.private_data_size, 0, 8);

	/* Store a backpointer to the skb at the beginning of the buffer
	 * (in the private data area) such that we can release it
	 * on Tx confirm
	 */
	skbh = (struct sk_buff **)buffer_start;
	*skbh = skb;

	addr = dma_map_single(dev, buffer_start,
			      skb_tail_pointer(skb) - buffer_start,
			      DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, addr)))
		return -ENOMEM;

	dpaa2_fd_set_addr(fd, addr);
	dpaa2_fd_set_offset(fd, (u16)(skb->data - buffer_start));
	dpaa2_fd_set_len(fd, skb->len);
	dpaa2_fd_set_format(fd, dpaa2_fd_single);

	fd->simple.ctrl = DPAA2_FD_CTRL_ASAL | DPAA2_FD_CTRL_PTA |
			  DPAA2_FD_CTRL_PTV1;

	return 0;
}

/* FD freeing routine on the Tx path
 *
 * DMA-unmap and free FD and possibly SGT buffer allocated on Tx. The skb
 * back-pointed to is also freed.
 * This can be called either from dpaa2_eth_tx_conf() or on the error path of
 * dpaa2_eth_tx().
 * Optionally, return the frame annotation status word (FAS), which needs
 * to be checked if we're on the confirmation path.
 */
static void free_tx_fd(const struct dpaa2_eth_priv *priv,
		       const struct dpaa2_fd *fd,
		       u32 *status)
{
	struct device *dev = priv->net_dev->dev.parent;
	dma_addr_t fd_addr;
	struct sk_buff **skbh, *skb;
	unsigned char *buffer_start;
	int unmap_size;
	struct scatterlist *scl;
	int num_sg, num_dma_bufs;
	struct dpaa2_eth_swa *swa;
	bool fd_single;
	struct dpaa2_fas *fas;

	fd_addr = dpaa2_fd_get_addr(fd);
	skbh = phys_to_virt(fd_addr);
	fd_single = (dpaa2_fd_get_format(fd) == dpaa2_fd_single);

	if (fd_single) {
		skb = *skbh;
		buffer_start = (unsigned char *)skbh;
		/* Accessing the skb buffer is safe before dma unmap, because
		 * we didn't map the actual skb shell.
		 */
		dma_unmap_single(dev, fd_addr,
				 skb_tail_pointer(skb) - buffer_start,
				 DMA_TO_DEVICE);
	} else {
		swa = (struct dpaa2_eth_swa *)skbh;
		skb = swa->skb;
		scl = swa->scl;
		num_sg = swa->num_sg;
		num_dma_bufs = swa->num_dma_bufs;

		/* Unmap the scatterlist */
		dma_unmap_sg(dev, scl, num_sg, DMA_TO_DEVICE);
		kfree(scl);

		/* Unmap the SGT buffer */
		unmap_size = priv->tx_data_offset +
		       sizeof(struct dpaa2_sg_entry) * (1 + num_dma_bufs);
		dma_unmap_single(dev, fd_addr, unmap_size, DMA_TO_DEVICE);
	}

	/* Get the timestamp value */
	if (priv->ts_tx_en && skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
		struct skb_shared_hwtstamps shhwtstamps;
		u64 *ns;

		memset(&shhwtstamps, 0, sizeof(shhwtstamps));

		ns = (u64 *)((void *)skbh +
			priv->buf_layout.private_data_size +
			sizeof(struct dpaa2_fas));
		*ns = DPAA2_PTP_NOMINAL_FREQ_PERIOD_NS * (*ns);
		shhwtstamps.hwtstamp = ns_to_ktime(*ns);
		skb_tstamp_tx(skb, &shhwtstamps);
	}

	/* Read the status from the Frame Annotation after we unmap the first
	 * buffer but before we free it. The caller function is responsible
	 * for checking the status value.
	 */
	if (status && (fd->simple.frc & DPAA2_FD_FRC_FASV)) {
		fas = (struct dpaa2_fas *)
			((void *)skbh + priv->buf_layout.private_data_size);
		*status = le32_to_cpu(fas->status);
	}

	/* Free SGT buffer kmalloc'ed on tx */
	if (!fd_single)
		kfree(skbh);

	/* Move on with skb release */
	dev_kfree_skb(skb);
}

static int dpaa2_eth_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpaa2_fd fd;
	struct rtnl_link_stats64 *percpu_stats;
	struct dpaa2_eth_drv_stats *percpu_extras;
	u16 queue_mapping, flow_id;
	int err, i;

	percpu_stats = this_cpu_ptr(priv->percpu_stats);
	percpu_extras = this_cpu_ptr(priv->percpu_extras);

	if (unlikely(skb_headroom(skb) < DPAA2_ETH_NEEDED_HEADROOM(priv))) {
		struct sk_buff *ns;

		ns = skb_realloc_headroom(skb, DPAA2_ETH_NEEDED_HEADROOM(priv));
		if (unlikely(!ns)) {
			percpu_stats->tx_dropped++;
			goto err_alloc_headroom;
		}
		dev_kfree_skb(skb);
		skb = ns;
	}

	/* We'll be holding a back-reference to the skb until Tx Confirmation;
	 * we don't want that overwritten by a concurrent Tx with a cloned skb.
	 */
	skb = skb_unshare(skb, GFP_ATOMIC);
	if (unlikely(!skb)) {
		/* skb_unshare() has already freed the skb */
		percpu_stats->tx_dropped++;
		return NETDEV_TX_OK;
	}

	/* Setup the FD fields */
	memset(&fd, 0, sizeof(fd));

	if (skb_is_nonlinear(skb)) {
		err = build_sg_fd(priv, skb, &fd);
		percpu_extras->tx_sg_frames++;
		percpu_extras->tx_sg_bytes += skb->len;
	} else {
		err = build_single_fd(priv, skb, &fd);
	}

	if (unlikely(err)) {
		percpu_stats->tx_dropped++;
		goto err_build_fd;
	}

	/* Tracing point */
	trace_dpaa2_tx_fd(net_dev, &fd);

	/* TxConf FQ selection primarily based on cpu affinity; this is
	 * non-migratable context, so it's safe to call smp_processor_id().
	 */
	queue_mapping = smp_processor_id() % priv->dpni_attrs.max_senders;
	flow_id = priv->fq[queue_mapping].flowid;
	for (i = 0; i < (DPAA2_ETH_MAX_TX_QUEUES << 1); i++) {
		err = dpaa2_io_service_enqueue_qd(NULL, priv->tx_qdid, 0,
						  flow_id, &fd);
		if (err != -EBUSY)
			break;
	}
	percpu_extras->tx_portal_busy += i;
	if (unlikely(err < 0)) {
		percpu_stats->tx_errors++;
		/* Clean up everything, including freeing the skb */
		free_tx_fd(priv, &fd, NULL);
	} else {
		percpu_stats->tx_packets++;
		percpu_stats->tx_bytes += skb->len;
	}

	return NETDEV_TX_OK;

err_build_fd:
err_alloc_headroom:
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

/* Tx confirmation frame processing routine */
static void dpaa2_eth_tx_conf(struct dpaa2_eth_priv *priv,
			      struct dpaa2_eth_channel *ch,
			      const struct dpaa2_fd *fd,
			      struct napi_struct *napi __always_unused)
{
	struct rtnl_link_stats64 *percpu_stats;
	struct dpaa2_eth_drv_stats *percpu_extras;
	u32 status = 0;

	/* Tracing point */
	trace_dpaa2_tx_conf_fd(priv->net_dev, fd);

	percpu_extras = this_cpu_ptr(priv->percpu_extras);
	percpu_extras->tx_conf_frames++;
	percpu_extras->tx_conf_bytes += dpaa2_fd_get_len(fd);

	free_tx_fd(priv, fd, &status);

	if (unlikely(status & DPAA2_ETH_TXCONF_ERR_MASK)) {
		percpu_stats = this_cpu_ptr(priv->percpu_stats);
		/* Tx-conf logically pertains to the egress path. */
		percpu_stats->tx_errors++;
	}
}

static int set_rx_csum(struct dpaa2_eth_priv *priv, bool enable)
{
	int err;

	err = dpni_set_l3_chksum_validation(priv->mc_io, 0, priv->mc_token,
					    enable);
	if (err) {
		netdev_err(priv->net_dev,
			   "dpni_set_l3_chksum_validation() failed\n");
		return err;
	}

	err = dpni_set_l4_chksum_validation(priv->mc_io, 0, priv->mc_token,
					    enable);
	if (err) {
		netdev_err(priv->net_dev,
			   "dpni_set_l4_chksum_validation failed\n");
		return err;
	}

	return 0;
}

static int set_tx_csum(struct dpaa2_eth_priv *priv, bool enable)
{
	struct dpaa2_eth_fq *fq;
	struct dpni_tx_flow_cfg tx_flow_cfg;
	int err;
	int i;

	memset(&tx_flow_cfg, 0, sizeof(tx_flow_cfg));
	tx_flow_cfg.options = DPNI_TX_FLOW_OPT_L3_CHKSUM_GEN |
			      DPNI_TX_FLOW_OPT_L4_CHKSUM_GEN;
	tx_flow_cfg.l3_chksum_gen = enable;
	tx_flow_cfg.l4_chksum_gen = enable;

	for (i = 0; i < priv->num_fqs; i++) {
		fq = &priv->fq[i];
		if (fq->type != DPAA2_TX_CONF_FQ)
			continue;

		/* The Tx flowid is kept in the corresponding TxConf FQ. */
		err = dpni_set_tx_flow(priv->mc_io, 0, priv->mc_token,
				       &fq->flowid, &tx_flow_cfg);
		if (err) {
			netdev_err(priv->net_dev, "dpni_set_tx_flow failed\n");
			return err;
		}
	}

	return 0;
}

/* Perform a single release command to add buffers
 * to the specified buffer pool
 */
static int add_bufs(struct dpaa2_eth_priv *priv, u16 bpid)
{
	struct device *dev = priv->net_dev->dev.parent;
	u64 buf_array[DPAA2_ETH_BUFS_PER_CMD];
	void *buf;
	dma_addr_t addr;
	int i;

	for (i = 0; i < DPAA2_ETH_BUFS_PER_CMD; i++) {
		/* Allocate buffer visible to WRIOP + skb shared info +
		 * alignment padding
		 */
		buf = napi_alloc_frag(DPAA2_ETH_BUF_RAW_SIZE);
		if (unlikely(!buf))
			goto err_alloc;

		buf = PTR_ALIGN(buf, DPAA2_ETH_RX_BUF_ALIGN);

		addr = dma_map_single(dev, buf, DPAA2_ETH_RX_BUF_SIZE,
				      DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(dev, addr)))
			goto err_map;

		buf_array[i] = addr;

		/* tracing point */
		trace_dpaa2_eth_buf_seed(priv->net_dev,
					 buf, DPAA2_ETH_BUF_RAW_SIZE,
					 addr, DPAA2_ETH_RX_BUF_SIZE,
					 bpid);
	}

release_bufs:
	/* In case the portal is busy, retry until successful.
	 * The buffer release function would only fail if the QBMan portal
	 * was busy, which implies portal contention (i.e. more CPUs than
	 * portals, i.e. GPPs w/o affine DPIOs). For all practical purposes,
	 * there is little we can realistically do, short of giving up -
	 * in which case we'd risk depleting the buffer pool and never again
	 * receiving the Rx interrupt which would kick-start the refill logic.
	 * So just keep retrying, at the risk of being moved to ksoftirqd.
	 */
	while (dpaa2_io_service_release(NULL, bpid, buf_array, i))
		cpu_relax();
	return i;

err_map:
	put_page(virt_to_head_page(buf));
err_alloc:
	if (i)
		goto release_bufs;

	return 0;
}

static int seed_pool(struct dpaa2_eth_priv *priv, u16 bpid)
{
	int i, j;
	int new_count;

	/* This is the lazy seeding of Rx buffer pools.
	 * dpaa2_add_bufs() is also used on the Rx hotpath and calls
	 * napi_alloc_frag(). The trouble with that is that it in turn ends up
	 * calling this_cpu_ptr(), which mandates execution in atomic context.
	 * Rather than splitting up the code, do a one-off preempt disable.
	 */
	preempt_disable();
	for (j = 0; j < priv->num_channels; j++) {
		for (i = 0; i < DPAA2_ETH_NUM_BUFS;
		     i += DPAA2_ETH_BUFS_PER_CMD) {
			new_count = add_bufs(priv, bpid);
			priv->channel[j]->buf_count += new_count;

			if (new_count < DPAA2_ETH_BUFS_PER_CMD) {
				preempt_enable();
				return -ENOMEM;
			}
		}
	}
	preempt_enable();

	return 0;
}

/**
 * Drain the specified number of buffers from the DPNI's private buffer pool.
 * @count must not exceeed DPAA2_ETH_BUFS_PER_CMD
 */
static void drain_bufs(struct dpaa2_eth_priv *priv, int count)
{
	struct device *dev = priv->net_dev->dev.parent;
	u64 buf_array[DPAA2_ETH_BUFS_PER_CMD];
	void *vaddr;
	int ret, i;

	do {
		ret = dpaa2_io_service_acquire(NULL, priv->dpbp_attrs.bpid,
					       buf_array, count);
		if (ret < 0) {
			netdev_err(priv->net_dev, "dpaa2_io_service_acquire() failed\n");
			return;
		}
		for (i = 0; i < ret; i++) {
			/* Same logic as on regular Rx path */
			dma_unmap_single(dev, buf_array[i],
					 DPAA2_ETH_RX_BUF_SIZE,
					 DMA_FROM_DEVICE);
			vaddr = phys_to_virt(buf_array[i]);
			put_page(virt_to_head_page(vaddr));
		}
	} while (ret);
}

static void drain_pool(struct dpaa2_eth_priv *priv)
{
	int i;

	drain_bufs(priv, DPAA2_ETH_BUFS_PER_CMD);
	drain_bufs(priv, 1);

	for (i = 0; i < priv->num_channels; i++)
		priv->channel[i]->buf_count = 0;
}

/* Function is called from softirq context only, so we don't need to guard
 * the access to percpu count
 */
static int refill_pool(struct dpaa2_eth_priv *priv,
		       struct dpaa2_eth_channel *ch,
		       u16 bpid)
{
	int new_count;

	if (likely(ch->buf_count >= DPAA2_ETH_REFILL_THRESH))
		return 0;

	do {
		new_count = add_bufs(priv, bpid);
		if (unlikely(!new_count)) {
			/* Out of memory; abort for now, we'll try later on */
			break;
		}
		ch->buf_count += new_count;
	} while (ch->buf_count < DPAA2_ETH_NUM_BUFS);

	if (unlikely(ch->buf_count < DPAA2_ETH_NUM_BUFS))
		return -ENOMEM;

	return 0;
}

static int pull_channel(struct dpaa2_eth_channel *ch)
{
	int err;
	int dequeues = -1;

	/* Retry while portal is busy */
	do {
		err = dpaa2_io_service_pull_channel(NULL, ch->ch_id, ch->store);
		dequeues++;
		cpu_relax();
	} while (err == -EBUSY);

	ch->stats.dequeue_portal_busy += dequeues;
	if (unlikely(err))
		ch->stats.pull_err++;

	return err;
}

/* NAPI poll routine
 *
 * Frames are dequeued from the QMan channel associated with this NAPI context.
 * Rx, Tx confirmation and (if configured) Rx error frames all count
 * towards the NAPI budget.
 */
static int dpaa2_eth_poll(struct napi_struct *napi, int budget)
{
	struct dpaa2_eth_channel *ch;
	int cleaned = 0, store_cleaned;
	struct dpaa2_eth_priv *priv;
	int err;

	ch = container_of(napi, struct dpaa2_eth_channel, napi);
	priv = ch->priv;

	while (cleaned < budget) {
		err = pull_channel(ch);
		if (unlikely(err))
			break;

		/* Refill pool if appropriate */
		refill_pool(priv, ch, priv->dpbp_attrs.bpid);

		store_cleaned = consume_frames(ch);
		cleaned += store_cleaned;

		/* If we have enough budget left for a full store,
		 * try a new pull dequeue, otherwise we're done here
		 */
		if (store_cleaned == 0 ||
		    cleaned > budget - DPAA2_ETH_STORE_SIZE)
			break;
	}

	if (cleaned < budget) {
		napi_complete_done(napi, cleaned);
		/* Re-enable data available notifications */
		do {
			err = dpaa2_io_service_rearm(NULL, &ch->nctx);
			cpu_relax();
		} while (err == -EBUSY);
	}

	ch->stats.frames += cleaned;

	return cleaned;
}

static void enable_ch_napi(struct dpaa2_eth_priv *priv)
{
	struct dpaa2_eth_channel *ch;
	int i;

	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		napi_enable(&ch->napi);
	}
}

static void disable_ch_napi(struct dpaa2_eth_priv *priv)
{
	struct dpaa2_eth_channel *ch;
	int i;

	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		napi_disable(&ch->napi);
	}
}

static int link_state_update(struct dpaa2_eth_priv *priv)
{
	struct dpni_link_state state;
	int err;

	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (unlikely(err)) {
		netdev_err(priv->net_dev,
			   "dpni_get_link_state() failed\n");
		return err;
	}

	/* Chech link state; speed / duplex changes are not treated yet */
	if (priv->link_state.up == state.up)
		return 0;

	priv->link_state = state;
	if (state.up) {
		netif_carrier_on(priv->net_dev);
		netif_tx_start_all_queues(priv->net_dev);
	} else {
		netif_tx_stop_all_queues(priv->net_dev);
		netif_carrier_off(priv->net_dev);
	}

	netdev_info(priv->net_dev, "Link Event: state %s",
		    state.up ? "up" : "down");

	return 0;
}

static int dpaa2_eth_open(struct net_device *net_dev)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int err;

	err = seed_pool(priv, priv->dpbp_attrs.bpid);
	if (err) {
		/* Not much to do; the buffer pool, though not filled up,
		 * may still contain some buffers which would enable us
		 * to limp on.
		 */
		netdev_err(net_dev, "Buffer seeding failed for DPBP %d (bpid=%d)\n",
			   priv->dpbp_dev->obj_desc.id, priv->dpbp_attrs.bpid);
	}

	/* We'll only start the txqs when the link is actually ready; make sure
	 * we don't race against the link up notification, which may come
	 * immediately after dpni_enable();
	 */
	netif_tx_stop_all_queues(net_dev);
	enable_ch_napi(priv);
	/* Also, explicitly set carrier off, otherwise netif_carrier_ok() will
	 * return true and cause 'ip link show' to report the LOWER_UP flag,
	 * even though the link notification wasn't even received.
	 */
	netif_carrier_off(net_dev);

	err = dpni_enable(priv->mc_io, 0, priv->mc_token);
	if (err < 0) {
		netdev_err(net_dev, "dpni_enable() failed\n");
		goto enable_err;
	}

	/* If the DPMAC object has already processed the link up interrupt,
	 * we have to learn the link state ourselves.
	 */
	err = link_state_update(priv);
	if (err < 0) {
		netdev_err(net_dev, "Can't update link state\n");
		goto link_state_err;
	}

	return 0;

link_state_err:
enable_err:
	disable_ch_napi(priv);
	drain_pool(priv);
	return err;
}

/* The DPIO store must be empty when we call this,
 * at the end of every NAPI cycle.
 */
static u32 drain_channel(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_channel *ch)
{
	u32 drained = 0, total = 0;

	do {
		pull_channel(ch);
		drained = consume_frames(ch);
		total += drained;
	} while (drained);

	return total;
}

static u32 drain_ingress_frames(struct dpaa2_eth_priv *priv)
{
	struct dpaa2_eth_channel *ch;
	int i;
	u32 drained = 0;

	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		drained += drain_channel(priv, ch);
	}

	return drained;
}

static int dpaa2_eth_stop(struct net_device *net_dev)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int dpni_enabled;
	int retries = 10;
	u32 drained;

	netif_tx_stop_all_queues(net_dev);
	netif_carrier_off(net_dev);

	/* Loop while dpni_disable() attempts to drain the egress FQs
	 * and confirm them back to us.
	 */
	do {
		dpni_disable(priv->mc_io, 0, priv->mc_token);
		dpni_is_enabled(priv->mc_io, 0, priv->mc_token, &dpni_enabled);
		if (dpni_enabled)
			/* Allow the MC some slack */
			msleep(100);
	} while (dpni_enabled && --retries);
	if (!retries) {
		netdev_warn(net_dev, "Retry count exceeded disabling DPNI\n");
		/* Must go on and disable NAPI nonetheless, so we don't crash at
		 * the next "ifconfig up"
		 */
	}

	/* Wait for NAPI to complete on every core and disable it.
	 * In particular, this will also prevent NAPI from being rescheduled if
	 * a new CDAN is serviced, effectively discarding the CDAN. We therefore
	 * don't even need to disarm the channels, except perhaps for the case
	 * of a huge coalescing value.
	 */
	disable_ch_napi(priv);

	 /* Manually drain the Rx and TxConf queues */
	drained = drain_ingress_frames(priv);
	if (drained)
		netdev_dbg(net_dev, "Drained %d frames.\n", drained);

	/* Empty the buffer pool */
	drain_pool(priv);

	return 0;
}

static int dpaa2_eth_init(struct net_device *net_dev)
{
	u64 supported = 0;
	u64 not_supported = 0;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	u32 options = priv->dpni_attrs.options;

	/* Capabilities listing */
	supported |= IFF_LIVE_ADDR_CHANGE | IFF_PROMISC | IFF_ALLMULTI;

	if (options & DPNI_OPT_UNICAST_FILTER)
		supported |= IFF_UNICAST_FLT;
	else
		not_supported |= IFF_UNICAST_FLT;

	if (options & DPNI_OPT_MULTICAST_FILTER)
		supported |= IFF_MULTICAST;
	else
		not_supported |= IFF_MULTICAST;

	net_dev->priv_flags |= supported;
	net_dev->priv_flags &= ~not_supported;

	/* Features */
	net_dev->features = NETIF_F_RXCSUM |
			    NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			    NETIF_F_SG | NETIF_F_HIGHDMA |
			    NETIF_F_LLTX;
	net_dev->hw_features = net_dev->features;

	return 0;
}

static int dpaa2_eth_set_addr(struct net_device *net_dev, void *addr)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;
	int err;

	err = eth_mac_addr(net_dev, addr);
	if (err < 0) {
		dev_err(dev, "eth_mac_addr() failed with error %d\n", err);
		return err;
	}

	err = dpni_set_primary_mac_addr(priv->mc_io, 0, priv->mc_token,
					net_dev->dev_addr);
	if (err) {
		dev_err(dev, "dpni_set_primary_mac_addr() failed (%d)\n", err);
		return err;
	}

	return 0;
}

/** Fill in counters maintained by the GPP driver. These may be different from
 * the hardware counters obtained by ethtool.
 */
static struct rtnl_link_stats64
*dpaa2_eth_get_stats(struct net_device *net_dev,
		     struct rtnl_link_stats64 *stats)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct rtnl_link_stats64 *percpu_stats;
	u64 *cpustats;
	u64 *netstats = (u64 *)stats;
	int i, j;
	int num = sizeof(struct rtnl_link_stats64) / sizeof(u64);

	for_each_possible_cpu(i) {
		percpu_stats = per_cpu_ptr(priv->percpu_stats, i);
		cpustats = (u64 *)percpu_stats;
		for (j = 0; j < num; j++)
			netstats[j] += cpustats[j];
	}

	return stats;
}

static int dpaa2_eth_change_mtu(struct net_device *net_dev, int mtu)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int err;

	if (mtu < 68 || mtu > DPAA2_ETH_MAX_MTU) {
		netdev_err(net_dev, "Invalid MTU %d. Valid range is: 68..%d\n",
			   mtu, DPAA2_ETH_MAX_MTU);
		return -EINVAL;
	}

	/* Set the maximum Rx frame length to match the transmit side;
	 * account for L2 headers when computing the MFL
	 */
	err = dpni_set_max_frame_length(priv->mc_io, 0, priv->mc_token,
					(u16)DPAA2_ETH_L2_MAX_FRM(mtu));
	if (err) {
		netdev_err(net_dev, "dpni_set_max_frame_length() failed\n");
		return err;
	}

	net_dev->mtu = mtu;
	return 0;
}

/* Copy mac unicast addresses from @net_dev to @priv.
 * Its sole purpose is to make dpaa2_eth_set_rx_mode() more readable.
 */
static void add_uc_hw_addr(const struct net_device *net_dev,
			   struct dpaa2_eth_priv *priv)
{
	struct netdev_hw_addr *ha;
	int err;

	netdev_for_each_uc_addr(ha, net_dev) {
		err = dpni_add_mac_addr(priv->mc_io, 0, priv->mc_token,
					ha->addr);
		if (err)
			netdev_warn(priv->net_dev,
				    "Could not add ucast MAC %pM to the filtering table (err %d)\n",
				    ha->addr, err);
	}
}

/* Copy mac multicast addresses from @net_dev to @priv
 * Its sole purpose is to make dpaa2_eth_set_rx_mode() more readable.
 */
static void add_mc_hw_addr(const struct net_device *net_dev,
			   struct dpaa2_eth_priv *priv)
{
	struct netdev_hw_addr *ha;
	int err;

	netdev_for_each_mc_addr(ha, net_dev) {
		err = dpni_add_mac_addr(priv->mc_io, 0, priv->mc_token,
					ha->addr);
		if (err)
			netdev_warn(priv->net_dev,
				    "Could not add mcast MAC %pM to the filtering table (err %d)\n",
				    ha->addr, err);
	}
}

static void dpaa2_eth_set_rx_mode(struct net_device *net_dev)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int uc_count = netdev_uc_count(net_dev);
	int mc_count = netdev_mc_count(net_dev);
	u8 max_uc = priv->dpni_attrs.max_unicast_filters;
	u8 max_mc = priv->dpni_attrs.max_multicast_filters;
	u32 options = priv->dpni_attrs.options;
	u16 mc_token = priv->mc_token;
	struct fsl_mc_io *mc_io = priv->mc_io;
	int err;

	/* Basic sanity checks; these probably indicate a misconfiguration */
	if (!(options & DPNI_OPT_UNICAST_FILTER) && max_uc != 0)
		netdev_info(net_dev,
			    "max_unicast_filters=%d, DPNI_OPT_UNICAST_FILTER option must be enabled\n",
			    max_uc);
	if (!(options & DPNI_OPT_MULTICAST_FILTER) && max_mc != 0)
		netdev_info(net_dev,
			    "max_multicast_filters=%d, DPNI_OPT_MULTICAST_FILTER option must be enabled\n",
			    max_mc);

	/* Force promiscuous if the uc or mc counts exceed our capabilities. */
	if (uc_count > max_uc) {
		netdev_info(net_dev,
			    "Unicast addr count reached %d, max allowed is %d; forcing promisc\n",
			    uc_count, max_uc);
		goto force_promisc;
	}
	if (mc_count > max_mc) {
		netdev_info(net_dev,
			    "Multicast addr count reached %d, max allowed is %d; forcing promisc\n",
			    mc_count, max_mc);
		goto force_mc_promisc;
	}

	/* Adjust promisc settings due to flag combinations */
	if (net_dev->flags & IFF_PROMISC)
		goto force_promisc;
	if (net_dev->flags & IFF_ALLMULTI) {
		/* First, rebuild unicast filtering table. This should be done
		 * in promisc mode, in order to avoid frame loss while we
		 * progressively add entries to the table.
		 * We don't know whether we had been in promisc already, and
		 * making an MC call to find it is expensive; so set uc promisc
		 * nonetheless.
		 */
		err = dpni_set_unicast_promisc(mc_io, 0, mc_token, 1);
		if (err)
			netdev_warn(net_dev, "Can't set uc promisc\n");

		/* Actual uc table reconstruction. */
		err = dpni_clear_mac_filters(mc_io, 0, mc_token, 1, 0);
		if (err)
			netdev_warn(net_dev, "Can't clear uc filters\n");
		add_uc_hw_addr(net_dev, priv);

		/* Finally, clear uc promisc and set mc promisc as requested. */
		err = dpni_set_unicast_promisc(mc_io, 0, mc_token, 0);
		if (err)
			netdev_warn(net_dev, "Can't clear uc promisc\n");
		goto force_mc_promisc;
	}

	/* Neither unicast, nor multicast promisc will be on... eventually.
	 * For now, rebuild mac filtering tables while forcing both of them on.
	 */
	err = dpni_set_unicast_promisc(mc_io, 0, mc_token, 1);
	if (err)
		netdev_warn(net_dev, "Can't set uc promisc (%d)\n", err);
	err = dpni_set_multicast_promisc(mc_io, 0, mc_token, 1);
	if (err)
		netdev_warn(net_dev, "Can't set mc promisc (%d)\n", err);

	/* Actual mac filtering tables reconstruction */
	err = dpni_clear_mac_filters(mc_io, 0, mc_token, 1, 1);
	if (err)
		netdev_warn(net_dev, "Can't clear mac filters\n");
	add_mc_hw_addr(net_dev, priv);
	add_uc_hw_addr(net_dev, priv);

	/* Now we can clear both ucast and mcast promisc, without risking
	 * to drop legitimate frames anymore.
	 */
	err = dpni_set_unicast_promisc(mc_io, 0, mc_token, 0);
	if (err)
		netdev_warn(net_dev, "Can't clear ucast promisc\n");
	err = dpni_set_multicast_promisc(mc_io, 0, mc_token, 0);
	if (err)
		netdev_warn(net_dev, "Can't clear mcast promisc\n");

	return;

force_promisc:
	err = dpni_set_unicast_promisc(mc_io, 0, mc_token, 1);
	if (err)
		netdev_warn(net_dev, "Can't set ucast promisc\n");
force_mc_promisc:
	err = dpni_set_multicast_promisc(mc_io, 0, mc_token, 1);
	if (err)
		netdev_warn(net_dev, "Can't set mcast promisc\n");
}

static int dpaa2_eth_set_features(struct net_device *net_dev,
				  netdev_features_t features)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	netdev_features_t changed = features ^ net_dev->features;
	bool enable;
	int err;

	if (changed & NETIF_F_RXCSUM) {
		enable = !!(features & NETIF_F_RXCSUM);
		err = set_rx_csum(priv, enable);
		if (err)
			return err;
	}

	if (changed & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM)) {
		enable = !!(features & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM));
		err = set_tx_csum(priv, enable);
		if (err)
			return err;
	}

	return 0;
}

static int dpaa2_eth_ts_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct dpaa2_eth_priv *priv = netdev_priv(dev);
	struct hwtstamp_config config;

	if (copy_from_user(&config, rq->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		priv->ts_tx_en = false;
		break;
	case HWTSTAMP_TX_ON:
		priv->ts_tx_en = true;
		break;
	default:
		return -ERANGE;
	}

	if (config.rx_filter == HWTSTAMP_FILTER_NONE) {
		priv->ts_rx_en = false;
	} else {
		priv->ts_rx_en = true;
		/* TS is set for all frame types, not only those requested */
		config.rx_filter = HWTSTAMP_FILTER_ALL;
	}

	return copy_to_user(rq->ifr_data, &config, sizeof(config)) ?
			-EFAULT : 0;
}

static int dpaa2_eth_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	if (cmd == SIOCSHWTSTAMP)
		return dpaa2_eth_ts_ioctl(dev, rq, cmd);

	return -EINVAL;
}

static const struct net_device_ops dpaa2_eth_ops = {
	.ndo_open = dpaa2_eth_open,
	.ndo_start_xmit = dpaa2_eth_tx,
	.ndo_stop = dpaa2_eth_stop,
	.ndo_init = dpaa2_eth_init,
	.ndo_set_mac_address = dpaa2_eth_set_addr,
	.ndo_get_stats64 = dpaa2_eth_get_stats,
	.ndo_change_mtu = dpaa2_eth_change_mtu,
	.ndo_set_rx_mode = dpaa2_eth_set_rx_mode,
	.ndo_set_features = dpaa2_eth_set_features,
	.ndo_do_ioctl = dpaa2_eth_ioctl,
};

static void cdan_cb(struct dpaa2_io_notification_ctx *ctx)
{
	struct dpaa2_eth_channel *ch;

	ch = container_of(ctx, struct dpaa2_eth_channel, nctx);

	/* Update NAPI statistics */
	ch->stats.cdan++;

	napi_schedule_irqoff(&ch->napi);
}

/* Verify that the FLIB API version of various MC objects is supported
 * by our driver
 */
static int check_obj_version(struct fsl_mc_device *ls_dev, u16 mc_version)
{
	char *name = ls_dev->obj_desc.type;
	struct device *dev = &ls_dev->dev;
	u16 supported_version, flib_version;

	if (strcmp(name, "dpni") == 0) {
		flib_version = DPNI_VER_MAJOR;
		supported_version = DPAA2_SUPPORTED_DPNI_VERSION;
	} else if (strcmp(name, "dpbp") == 0) {
		flib_version = DPBP_VER_MAJOR;
		supported_version = DPAA2_SUPPORTED_DPBP_VERSION;
	} else if (strcmp(name, "dpcon") == 0) {
		flib_version = DPCON_VER_MAJOR;
		supported_version = DPAA2_SUPPORTED_DPCON_VERSION;
	} else {
		dev_err(dev, "invalid object type (%s)\n", name);
		return -EINVAL;
	}

	/* Check that the FLIB-defined version matches the one reported by MC */
	if (mc_version != flib_version) {
		dev_err(dev, "%s FLIB version mismatch: MC reports %d, we have %d\n",
			name, mc_version, flib_version);
		return -EINVAL;
	}

	/* ... and that we actually support it */
	if (mc_version < supported_version) {
		dev_err(dev, "Unsupported %s FLIB version (%d)\n",
			name, mc_version);
		return -EINVAL;
	}
	dev_dbg(dev, "Using %s FLIB version %d\n", name, mc_version);

	return 0;
}

/* Allocate and configure a DPCON object */
static struct fsl_mc_device *setup_dpcon(struct dpaa2_eth_priv *priv)
{
	struct fsl_mc_device *dpcon;
	struct device *dev = priv->net_dev->dev.parent;
	struct dpcon_attr attrs;
	int err;

	err = fsl_mc_object_allocate(to_fsl_mc_device(dev),
				     FSL_MC_POOL_DPCON, &dpcon);
	if (err) {
		dev_info(dev, "Not enough DPCONs, will go on as-is\n");
		return NULL;
	}

	err = dpcon_open(priv->mc_io, 0, dpcon->obj_desc.id, &dpcon->mc_handle);
	if (err) {
		dev_err(dev, "dpcon_open() failed\n");
		goto err_open;
	}

	err = dpcon_get_attributes(priv->mc_io, 0, dpcon->mc_handle, &attrs);
	if (err) {
		dev_err(dev, "dpcon_get_attributes() failed\n");
		goto err_get_attr;
	}

	err = check_obj_version(dpcon, attrs.version.major);
	if (err)
		goto err_dpcon_ver;

	err = dpcon_enable(priv->mc_io, 0, dpcon->mc_handle);
	if (err) {
		dev_err(dev, "dpcon_enable() failed\n");
		goto err_enable;
	}

	return dpcon;

err_enable:
err_dpcon_ver:
err_get_attr:
	dpcon_close(priv->mc_io, 0, dpcon->mc_handle);
err_open:
	fsl_mc_object_free(dpcon);

	return NULL;
}

static void free_dpcon(struct dpaa2_eth_priv *priv,
		       struct fsl_mc_device *dpcon)
{
	dpcon_disable(priv->mc_io, 0, dpcon->mc_handle);
	dpcon_close(priv->mc_io, 0, dpcon->mc_handle);
	fsl_mc_object_free(dpcon);
}

static struct dpaa2_eth_channel *
alloc_channel(struct dpaa2_eth_priv *priv)
{
	struct dpaa2_eth_channel *channel;
	struct dpcon_attr attr;
	struct device *dev = priv->net_dev->dev.parent;
	int err;

	channel = kzalloc(sizeof(*channel), GFP_ATOMIC);
	if (!channel)
		return NULL;

	channel->dpcon = setup_dpcon(priv);
	if (!channel->dpcon)
		goto err_setup;

	err = dpcon_get_attributes(priv->mc_io, 0, channel->dpcon->mc_handle,
				   &attr);
	if (err) {
		dev_err(dev, "dpcon_get_attributes() failed\n");
		goto err_get_attr;
	}

	channel->dpcon_id = attr.id;
	channel->ch_id = attr.qbman_ch_id;
	channel->priv = priv;

	return channel;

err_get_attr:
	free_dpcon(priv, channel->dpcon);
err_setup:
	kfree(channel);
	return NULL;
}

static void free_channel(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_channel *channel)
{
	free_dpcon(priv, channel->dpcon);
	kfree(channel);
}

/* DPIO setup: allocate and configure QBMan channels, setup core affinity
 * and register data availability notifications
 */
static int setup_dpio(struct dpaa2_eth_priv *priv)
{
	struct dpaa2_io_notification_ctx *nctx;
	struct dpaa2_eth_channel *channel;
	struct dpcon_notification_cfg dpcon_notif_cfg;
	struct device *dev = priv->net_dev->dev.parent;
	int i, err;

	/* Don't allocate more channels than strictly necessary and assign
	 * them to cores starting from the first one available in
	 * cpu_online_mask.
	 * If the number of channels is lower than the number of cores,
	 * there will be no rx/tx conf processing on the last cores in the mask.
	 */
	cpumask_clear(&priv->dpio_cpumask);
	for_each_online_cpu(i) {
		/* Try to allocate a channel */
		channel = alloc_channel(priv);
		if (!channel)
			goto err_alloc_ch;

		priv->channel[priv->num_channels] = channel;

		nctx = &channel->nctx;
		nctx->is_cdan = 1;
		nctx->cb = cdan_cb;
		nctx->id = channel->ch_id;
		nctx->desired_cpu = i;

		/* Register the new context */
		err = dpaa2_io_service_register(NULL, nctx);
		if (err) {
			dev_info(dev, "No affine DPIO for core %d\n", i);
			/* This core doesn't have an affine DPIO, but there's
			 * a chance another one does, so keep trying
			 */
			free_channel(priv, channel);
			continue;
		}

		/* Register DPCON notification with MC */
		dpcon_notif_cfg.dpio_id = nctx->dpio_id;
		dpcon_notif_cfg.priority = 0;
		dpcon_notif_cfg.user_ctx = nctx->qman64;
		err = dpcon_set_notification(priv->mc_io, 0,
					     channel->dpcon->mc_handle,
					     &dpcon_notif_cfg);
		if (err) {
			dev_err(dev, "dpcon_set_notification failed()\n");
			goto err_set_cdan;
		}

		/* If we managed to allocate a channel and also found an affine
		 * DPIO for this core, add it to the final mask
		 */
		cpumask_set_cpu(i, &priv->dpio_cpumask);
		priv->num_channels++;

		if (priv->num_channels == dpaa2_eth_max_channels(priv))
			break;
	}

	/* Tx confirmation queues can only be serviced by cpus
	 * with an affine DPIO/channel
	 */
	cpumask_copy(&priv->txconf_cpumask, &priv->dpio_cpumask);

	return 0;

err_set_cdan:
	dpaa2_io_service_deregister(NULL, nctx);
	free_channel(priv, channel);
err_alloc_ch:
	if (cpumask_empty(&priv->dpio_cpumask)) {
		dev_err(dev, "No cpu with an affine DPIO/DPCON\n");
		return -ENODEV;
	}
	cpumask_copy(&priv->txconf_cpumask, &priv->dpio_cpumask);

	return 0;
}

static void free_dpio(struct dpaa2_eth_priv *priv)
{
	int i;
	struct dpaa2_eth_channel *ch;

	/* deregister CDAN notifications and free channels */
	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		dpaa2_io_service_deregister(NULL, &ch->nctx);
		free_channel(priv, ch);
	}
}

static struct dpaa2_eth_channel *get_affine_channel(struct dpaa2_eth_priv *priv,
						    int cpu)
{
	struct device *dev = priv->net_dev->dev.parent;
	int i;

	for (i = 0; i < priv->num_channels; i++)
		if (priv->channel[i]->nctx.desired_cpu == cpu)
			return priv->channel[i];

	/* We should never get here. Issue a warning and return
	 * the first channel, because it's still better than nothing
	 */
	dev_warn(dev, "No affine channel found for cpu %d\n", cpu);

	return priv->channel[0];
}

static void set_fq_affinity(struct dpaa2_eth_priv *priv)
{
	struct device *dev = priv->net_dev->dev.parent;
	struct dpaa2_eth_fq *fq;
	int rx_cpu, txc_cpu;
	int i;

	/* For each FQ, pick one channel/CPU to deliver frames to.
	 * This may well change at runtime, either through irqbalance or
	 * through direct user intervention.
	 */
	rx_cpu = cpumask_first(&priv->dpio_cpumask);
	txc_cpu = cpumask_first(&priv->txconf_cpumask);

	for (i = 0; i < priv->num_fqs; i++) {
		fq = &priv->fq[i];
		switch (fq->type) {
		case DPAA2_RX_FQ:
		case DPAA2_RX_ERR_FQ:
			fq->target_cpu = rx_cpu;
			rx_cpu = cpumask_next(rx_cpu, &priv->dpio_cpumask);
			if (rx_cpu >= nr_cpu_ids)
				rx_cpu = cpumask_first(&priv->dpio_cpumask);
			break;
		case DPAA2_TX_CONF_FQ:
			fq->target_cpu = txc_cpu;
			txc_cpu = cpumask_next(txc_cpu, &priv->txconf_cpumask);
			if (txc_cpu >= nr_cpu_ids)
				txc_cpu = cpumask_first(&priv->txconf_cpumask);
			break;
		default:
			dev_err(dev, "Unknown FQ type: %d\n", fq->type);
		}
		fq->channel = get_affine_channel(priv, fq->target_cpu);
	}
}

static void setup_fqs(struct dpaa2_eth_priv *priv)
{
	int i;

	/* We have one TxConf FQ per Tx flow */
	for (i = 0; i < priv->dpni_attrs.max_senders; i++) {
		priv->fq[priv->num_fqs].type = DPAA2_TX_CONF_FQ;
		priv->fq[priv->num_fqs].consume = dpaa2_eth_tx_conf;
		priv->fq[priv->num_fqs++].flowid = DPNI_NEW_FLOW_ID;
	}

	/* The number of Rx queues (Rx distribution width) may be different from
	 * the number of cores.
	 * We only support one traffic class for now.
	 */
	for (i = 0; i < dpaa2_eth_queue_count(priv); i++) {
		priv->fq[priv->num_fqs].type = DPAA2_RX_FQ;
		priv->fq[priv->num_fqs].consume = dpaa2_eth_rx;
		priv->fq[priv->num_fqs++].flowid = (u16)i;
	}

#ifdef CONFIG_FSL_DPAA2_ETH_USE_ERR_QUEUE
	/* We have exactly one Rx error queue per DPNI */
	priv->fq[priv->num_fqs].type = DPAA2_RX_ERR_FQ;
	priv->fq[priv->num_fqs++].consume = dpaa2_eth_rx_err;
#endif

	/* For each FQ, decide on which core to process incoming frames */
	set_fq_affinity(priv);
}

/* Allocate and configure one buffer pool for each interface */
static int setup_dpbp(struct dpaa2_eth_priv *priv)
{
	int err;
	struct fsl_mc_device *dpbp_dev;
	struct device *dev = priv->net_dev->dev.parent;

	err = fsl_mc_object_allocate(to_fsl_mc_device(dev), FSL_MC_POOL_DPBP,
				     &dpbp_dev);
	if (err) {
		dev_err(dev, "DPBP device allocation failed\n");
		return err;
	}

	priv->dpbp_dev = dpbp_dev;

	err = dpbp_open(priv->mc_io, 0, priv->dpbp_dev->obj_desc.id,
			&dpbp_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpbp_open() failed\n");
		goto err_open;
	}

	err = dpbp_enable(priv->mc_io, 0, dpbp_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpbp_enable() failed\n");
		goto err_enable;
	}

	err = dpbp_get_attributes(priv->mc_io, 0, dpbp_dev->mc_handle,
				  &priv->dpbp_attrs);
	if (err) {
		dev_err(dev, "dpbp_get_attributes() failed\n");
		goto err_get_attr;
	}

	err = check_obj_version(dpbp_dev, priv->dpbp_attrs.version.major);
	if (err)
		goto err_dpbp_ver;

	return 0;

err_dpbp_ver:
err_get_attr:
	dpbp_disable(priv->mc_io, 0, dpbp_dev->mc_handle);
err_enable:
	dpbp_close(priv->mc_io, 0, dpbp_dev->mc_handle);
err_open:
	fsl_mc_object_free(dpbp_dev);

	return err;
}

static void free_dpbp(struct dpaa2_eth_priv *priv)
{
	drain_pool(priv);
	dpbp_disable(priv->mc_io, 0, priv->dpbp_dev->mc_handle);
	dpbp_close(priv->mc_io, 0, priv->dpbp_dev->mc_handle);
	fsl_mc_object_free(priv->dpbp_dev);
}

/* Configure the DPNI object this interface is associated with */
static int setup_dpni(struct fsl_mc_device *ls_dev)
{
	struct device *dev = &ls_dev->dev;
	struct dpaa2_eth_priv *priv;
	struct net_device *net_dev;
	void *dma_mem;
	int err;

	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	priv->dpni_id = ls_dev->obj_desc.id;

	/* get a handle for the DPNI object */
	err = dpni_open(priv->mc_io, 0, priv->dpni_id, &priv->mc_token);
	if (err) {
		dev_err(dev, "dpni_open() failed\n");
		goto err_open;
	}

	ls_dev->mc_io = priv->mc_io;
	ls_dev->mc_handle = priv->mc_token;

	/* Map a memory region which will be used by MC to pass us an
	 * attribute structure
	 */
	dma_mem = kzalloc(DPAA2_EXT_CFG_SIZE, GFP_DMA | GFP_KERNEL);
	if (!dma_mem)
		goto err_alloc;

	priv->dpni_attrs.ext_cfg_iova = dma_map_single(dev, dma_mem,
						       DPAA2_EXT_CFG_SIZE,
						       DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, priv->dpni_attrs.ext_cfg_iova)) {
		dev_err(dev, "dma mapping for dpni_ext_cfg failed\n");
		goto err_dma_map;
	}

	err = dpni_get_attributes(priv->mc_io, 0, priv->mc_token,
				  &priv->dpni_attrs);

	/* We'll check the return code after unmapping, as we need to
	 * do this anyway
	 */
	dma_unmap_single(dev, priv->dpni_attrs.ext_cfg_iova,
			 DPAA2_EXT_CFG_SIZE, DMA_FROM_DEVICE);

	if (err) {
		dev_err(dev, "dpni_get_attributes() failed (err=%d)\n", err);
		goto err_get_attr;
	}

	err = check_obj_version(ls_dev, priv->dpni_attrs.version.major);
	if (err)
		goto err_dpni_ver;

	memset(&priv->dpni_ext_cfg, 0, sizeof(priv->dpni_ext_cfg));
	err = dpni_extract_extended_cfg(&priv->dpni_ext_cfg, dma_mem);
	if (err) {
		dev_err(dev, "dpni_extract_extended_cfg() failed\n");
		goto err_extract;
	}

	/* Configure our buffers' layout */
	priv->buf_layout.options = DPNI_BUF_LAYOUT_OPT_PARSER_RESULT |
				   DPNI_BUF_LAYOUT_OPT_FRAME_STATUS |
				   DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE |
				   DPNI_BUF_LAYOUT_OPT_DATA_ALIGN;
	priv->buf_layout.pass_parser_result = true;
	priv->buf_layout.pass_frame_status = true;
	priv->buf_layout.private_data_size = DPAA2_ETH_SWA_SIZE;
	/* HW erratum mandates data alignment in multiples of 256 */
	priv->buf_layout.data_align = DPAA2_ETH_RX_BUF_ALIGN;

	/* rx buffer */
	err = dpni_set_rx_buffer_layout(priv->mc_io, 0, priv->mc_token,
					&priv->buf_layout);
	if (err) {
		dev_err(dev, "dpni_set_rx_buffer_layout() failed");
		goto err_buf_layout;
	}
	/* tx buffer: remove Rx-only options */
	priv->buf_layout.options &= ~(DPNI_BUF_LAYOUT_OPT_DATA_ALIGN |
				      DPNI_BUF_LAYOUT_OPT_PARSER_RESULT);
	err = dpni_set_tx_buffer_layout(priv->mc_io, 0, priv->mc_token,
					&priv->buf_layout);
	if (err) {
		dev_err(dev, "dpni_set_tx_buffer_layout() failed");
		goto err_buf_layout;
	}
	/* tx-confirm: same options as tx */
	priv->buf_layout.options &= ~DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE;
	priv->buf_layout.options |= DPNI_BUF_LAYOUT_OPT_TIMESTAMP;
	priv->buf_layout.pass_timestamp = 1;
	err = dpni_set_tx_conf_buffer_layout(priv->mc_io, 0, priv->mc_token,
					     &priv->buf_layout);
	if (err) {
		dev_err(dev, "dpni_set_tx_conf_buffer_layout() failed");
		goto err_buf_layout;
	}
	/* Now that we've set our tx buffer layout, retrieve the minimum
	 * required tx data offset.
	 */
	err = dpni_get_tx_data_offset(priv->mc_io, 0, priv->mc_token,
				      &priv->tx_data_offset);
	if (err) {
		dev_err(dev, "dpni_get_tx_data_offset() failed\n");
		goto err_data_offset;
	}

	if ((priv->tx_data_offset % 64) != 0)
		dev_warn(dev, "Tx data offset (%d) not a multiple of 64B",
			 priv->tx_data_offset);

	/* Accommodate SWA space. */
	priv->tx_data_offset += DPAA2_ETH_SWA_SIZE;

	/* allocate classification rule space */
	priv->cls_rule = kzalloc(sizeof(*priv->cls_rule) *
				 DPAA2_CLASSIFIER_ENTRY_COUNT, GFP_KERNEL);
	if (!priv->cls_rule)
		goto err_cls_rule;

	kfree(dma_mem);

	return 0;

err_cls_rule:
err_data_offset:
err_buf_layout:
err_extract:
err_dpni_ver:
err_get_attr:
err_dma_map:
	kfree(dma_mem);
err_alloc:
	dpni_close(priv->mc_io, 0, priv->mc_token);
err_open:
	return err;
}

static void free_dpni(struct dpaa2_eth_priv *priv)
{
	int err;

	err = dpni_reset(priv->mc_io, 0, priv->mc_token);
	if (err)
		netdev_warn(priv->net_dev, "dpni_reset() failed (err %d)\n",
			    err);

	dpni_close(priv->mc_io, 0, priv->mc_token);
}

static int setup_rx_flow(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_fq *fq)
{
	struct device *dev = priv->net_dev->dev.parent;
	struct dpni_queue_attr rx_queue_attr;
	struct dpni_queue_cfg queue_cfg;
	int err;

	memset(&queue_cfg, 0, sizeof(queue_cfg));
	queue_cfg.options = DPNI_QUEUE_OPT_USER_CTX | DPNI_QUEUE_OPT_DEST |
			    DPNI_QUEUE_OPT_TAILDROP_THRESHOLD;
	queue_cfg.dest_cfg.dest_type = DPNI_DEST_DPCON;
	queue_cfg.dest_cfg.priority = 1;
	queue_cfg.user_ctx = (u64)fq;
	queue_cfg.dest_cfg.dest_id = fq->channel->dpcon_id;
	queue_cfg.tail_drop_threshold = DPAA2_ETH_TAILDROP_THRESH;
	err = dpni_set_rx_flow(priv->mc_io, 0, priv->mc_token, 0, fq->flowid,
			       &queue_cfg);
	if (err) {
		dev_err(dev, "dpni_set_rx_flow() failed\n");
		return err;
	}

	/* Get the actual FQID that was assigned by MC */
	err = dpni_get_rx_flow(priv->mc_io, 0, priv->mc_token, 0, fq->flowid,
			       &rx_queue_attr);
	if (err) {
		dev_err(dev, "dpni_get_rx_flow() failed\n");
		return err;
	}
	fq->fqid = rx_queue_attr.fqid;

	return 0;
}

static int setup_tx_flow(struct dpaa2_eth_priv *priv,
			 struct dpaa2_eth_fq *fq)
{
	struct device *dev = priv->net_dev->dev.parent;
	struct dpni_tx_flow_cfg tx_flow_cfg;
	struct dpni_tx_conf_cfg tx_conf_cfg;
	struct dpni_tx_conf_attr tx_conf_attr;
	int err;

	memset(&tx_flow_cfg, 0, sizeof(tx_flow_cfg));
	tx_flow_cfg.options = DPNI_TX_FLOW_OPT_TX_CONF_ERROR;
	tx_flow_cfg.use_common_tx_conf_queue = 0;
	err = dpni_set_tx_flow(priv->mc_io, 0, priv->mc_token,
			       &fq->flowid, &tx_flow_cfg);
	if (err) {
		dev_err(dev, "dpni_set_tx_flow() failed\n");
		return err;
	}

	tx_conf_cfg.errors_only = 0;
	tx_conf_cfg.queue_cfg.options = DPNI_QUEUE_OPT_USER_CTX |
					DPNI_QUEUE_OPT_DEST;
	tx_conf_cfg.queue_cfg.user_ctx = (u64)fq;
	tx_conf_cfg.queue_cfg.dest_cfg.dest_type = DPNI_DEST_DPCON;
	tx_conf_cfg.queue_cfg.dest_cfg.dest_id = fq->channel->dpcon_id;
	tx_conf_cfg.queue_cfg.dest_cfg.priority = 0;

	err = dpni_set_tx_conf(priv->mc_io, 0, priv->mc_token, fq->flowid,
			       &tx_conf_cfg);
	if (err) {
		dev_err(dev, "dpni_set_tx_conf() failed\n");
		return err;
	}

	err = dpni_get_tx_conf(priv->mc_io, 0, priv->mc_token, fq->flowid,
			       &tx_conf_attr);
	if (err) {
		dev_err(dev, "dpni_get_tx_conf() failed\n");
		return err;
	}

	fq->fqid = tx_conf_attr.queue_attr.fqid;

	return 0;
}

#ifdef CONFIG_FSL_DPAA2_ETH_USE_ERR_QUEUE
static int setup_rx_err_flow(struct dpaa2_eth_priv *priv,
			     struct dpaa2_eth_fq *fq)
{
	struct dpni_queue_attr queue_attr;
	struct dpni_queue_cfg queue_cfg;
	int err;

	/* Configure the Rx error queue to generate CDANs,
	 * just like the Rx queues
	 */
	queue_cfg.options = DPNI_QUEUE_OPT_USER_CTX | DPNI_QUEUE_OPT_DEST;
	queue_cfg.dest_cfg.dest_type = DPNI_DEST_DPCON;
	queue_cfg.dest_cfg.priority = 1;
	queue_cfg.user_ctx = (u64)fq;
	queue_cfg.dest_cfg.dest_id = fq->channel->dpcon_id;
	err = dpni_set_rx_err_queue(priv->mc_io, 0, priv->mc_token, &queue_cfg);
	if (err) {
		netdev_err(priv->net_dev, "dpni_set_rx_err_queue() failed\n");
		return err;
	}

	/* Get the FQID */
	err = dpni_get_rx_err_queue(priv->mc_io, 0, priv->mc_token,
				    &queue_attr);
	if (err) {
		netdev_err(priv->net_dev, "dpni_get_rx_err_queue() failed\n");
		return err;
	}
	fq->fqid = queue_attr.fqid;

	return 0;
}
#endif

/* Bind the DPNI to its needed objects and resources: buffer pool, DPIOs,
 * frame queues and channels
 */
static int bind_dpni(struct dpaa2_eth_priv *priv)
{
	struct net_device *net_dev = priv->net_dev;
	struct device *dev = net_dev->dev.parent;
	struct dpni_pools_cfg pools_params;
	struct dpni_error_cfg err_cfg;
	int err = 0;
	int i;

	pools_params.num_dpbp = 1;
	pools_params.pools[0].dpbp_id = priv->dpbp_dev->obj_desc.id;
	pools_params.pools[0].backup_pool = 0;
	pools_params.pools[0].buffer_size = DPAA2_ETH_RX_BUF_SIZE;
	err = dpni_set_pools(priv->mc_io, 0, priv->mc_token, &pools_params);
	if (err) {
		dev_err(dev, "dpni_set_pools() failed\n");
		return err;
	}

	check_fs_support(net_dev);

	/* have the interface implicitly distribute traffic based on supported
	 * header fields
	 */
	if (dpaa2_eth_hash_enabled(priv)) {
		err = dpaa2_eth_set_hash(net_dev, DPAA2_RXH_SUPPORTED);
		if (err)
			return err;
	}

	/* Configure handling of error frames */
	err_cfg.errors = DPAA2_ETH_RX_ERR_MASK;
	err_cfg.set_frame_annotation = 1;
#ifdef CONFIG_FSL_DPAA2_ETH_USE_ERR_QUEUE
	err_cfg.error_action = DPNI_ERROR_ACTION_SEND_TO_ERROR_QUEUE;
#else
	err_cfg.error_action = DPNI_ERROR_ACTION_DISCARD;
#endif
	err = dpni_set_errors_behavior(priv->mc_io, 0, priv->mc_token,
				       &err_cfg);
	if (err) {
		dev_err(dev, "dpni_set_errors_behavior failed\n");
		return err;
	}

	/* Configure Rx and Tx conf queues to generate CDANs */
	for (i = 0; i < priv->num_fqs; i++) {
		switch (priv->fq[i].type) {
		case DPAA2_RX_FQ:
			err = setup_rx_flow(priv, &priv->fq[i]);
			break;
		case DPAA2_TX_CONF_FQ:
			err = setup_tx_flow(priv, &priv->fq[i]);
			break;
#ifdef CONFIG_FSL_DPAA2_ETH_USE_ERR_QUEUE
		case DPAA2_RX_ERR_FQ:
			err = setup_rx_err_flow(priv, &priv->fq[i]);
			break;
#endif
		default:
			dev_err(dev, "Invalid FQ type %d\n", priv->fq[i].type);
			return -EINVAL;
		}
		if (err)
			return err;
	}

	err = dpni_get_qdid(priv->mc_io, 0, priv->mc_token, &priv->tx_qdid);
	if (err) {
		dev_err(dev, "dpni_get_qdid() failed\n");
		return err;
	}

	return 0;
}

/* Allocate rings for storing incoming frame descriptors */
static int alloc_rings(struct dpaa2_eth_priv *priv)
{
	struct net_device *net_dev = priv->net_dev;
	struct device *dev = net_dev->dev.parent;
	int i;

	for (i = 0; i < priv->num_channels; i++) {
		priv->channel[i]->store =
			dpaa2_io_store_create(DPAA2_ETH_STORE_SIZE, dev);
		if (!priv->channel[i]->store) {
			netdev_err(net_dev, "dpaa2_io_store_create() failed\n");
			goto err_ring;
		}
	}

	return 0;

err_ring:
	for (i = 0; i < priv->num_channels; i++) {
		if (!priv->channel[i]->store)
			break;
		dpaa2_io_store_destroy(priv->channel[i]->store);
	}

	return -ENOMEM;
}

static void free_rings(struct dpaa2_eth_priv *priv)
{
	int i;

	for (i = 0; i < priv->num_channels; i++)
		dpaa2_io_store_destroy(priv->channel[i]->store);
}

static int netdev_init(struct net_device *net_dev)
{
	int err;
	struct device *dev = net_dev->dev.parent;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	u8 mac_addr[ETH_ALEN];
	u8 bcast_addr[ETH_ALEN];

	net_dev->netdev_ops = &dpaa2_eth_ops;

	/* If the DPNI attributes contain an all-0 mac_addr,
	 * set a random hardware address
	 */
	err = dpni_get_primary_mac_addr(priv->mc_io, 0, priv->mc_token,
					mac_addr);
	if (err) {
		dev_err(dev, "dpni_get_primary_mac_addr() failed (%d)", err);
		return err;
	}
	if (is_zero_ether_addr(mac_addr)) {
		/* Fills in net_dev->dev_addr, as required by
		 * register_netdevice()
		 */
		eth_hw_addr_random(net_dev);
		/* Make the user aware, without cluttering the boot log */
		pr_info_once(KBUILD_MODNAME " device(s) have all-zero hwaddr, replaced with random");
		err = dpni_set_primary_mac_addr(priv->mc_io, 0, priv->mc_token,
						net_dev->dev_addr);
		if (err) {
			dev_err(dev, "dpni_set_primary_mac_addr(): %d\n", err);
			return err;
		}
		/* Override NET_ADDR_RANDOM set by eth_hw_addr_random(); for all
		 * practical purposes, this will be our "permanent" mac address,
		 * at least until the next reboot. This move will also permit
		 * register_netdevice() to properly fill up net_dev->perm_addr.
		 */
		net_dev->addr_assign_type = NET_ADDR_PERM;
	} else {
		/* NET_ADDR_PERM is default, all we have to do is
		 * fill in the device addr.
		 */
		memcpy(net_dev->dev_addr, mac_addr, net_dev->addr_len);
	}

	/* Explicitly add the broadcast address to the MAC filtering table;
	 * the MC won't do that for us.
	 */
	eth_broadcast_addr(bcast_addr);
	err = dpni_add_mac_addr(priv->mc_io, 0, priv->mc_token, bcast_addr);
	if (err) {
		dev_warn(dev, "dpni_add_mac_addr() failed (%d)\n", err);
		/* Won't return an error; at least, we'd have egress traffic */
	}

	/* Reserve enough space to align buffer as per hardware requirement;
	 * NOTE: priv->tx_data_offset MUST be initialized at this point.
	 */
	net_dev->needed_headroom = DPAA2_ETH_NEEDED_HEADROOM(priv);

	/* Our .ndo_init will be called herein */
	err = register_netdev(net_dev);
	if (err < 0) {
		dev_err(dev, "register_netdev() = %d\n", err);
		return err;
	}

	return 0;
}

static int poll_link_state(void *arg)
{
	struct dpaa2_eth_priv *priv = (struct dpaa2_eth_priv *)arg;
	int err;

	while (!kthread_should_stop()) {
		err = link_state_update(priv);
		if (unlikely(err))
			return err;

		msleep(DPAA2_ETH_LINK_STATE_REFRESH);
	}

	return 0;
}

static irqreturn_t dpni_irq0_handler(int irq_num, void *arg)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t dpni_irq0_handler_thread(int irq_num, void *arg)
{
	u8 irq_index = DPNI_IRQ_INDEX;
	u32 status, clear = 0;
	struct device *dev = (struct device *)arg;
	struct fsl_mc_device *dpni_dev = to_fsl_mc_device(dev);
	struct net_device *net_dev = dev_get_drvdata(dev);
	int err;

	err = dpni_get_irq_status(dpni_dev->mc_io, 0, dpni_dev->mc_handle,
				  irq_index, &status);
	if (unlikely(err)) {
		netdev_err(net_dev, "Can't get irq status (err %d)", err);
		clear = 0xffffffff;
		goto out;
	}

	if (status & DPNI_IRQ_EVENT_LINK_CHANGED) {
		clear |= DPNI_IRQ_EVENT_LINK_CHANGED;
		link_state_update(netdev_priv(net_dev));
	}

out:
	dpni_clear_irq_status(dpni_dev->mc_io, 0, dpni_dev->mc_handle,
			      irq_index, clear);
	return IRQ_HANDLED;
}

static int setup_irqs(struct fsl_mc_device *ls_dev)
{
	int err = 0;
	struct fsl_mc_device_irq *irq;
	u8 irq_index = DPNI_IRQ_INDEX;
	u32 mask = DPNI_IRQ_EVENT_LINK_CHANGED;

	err = fsl_mc_allocate_irqs(ls_dev);
	if (err) {
		dev_err(&ls_dev->dev, "MC irqs allocation failed\n");
		return err;
	}

	irq = ls_dev->irqs[0];
	err = devm_request_threaded_irq(&ls_dev->dev, irq->msi_desc->irq,
					dpni_irq0_handler,
					dpni_irq0_handler_thread,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(&ls_dev->dev), &ls_dev->dev);
	if (err < 0) {
		dev_err(&ls_dev->dev, "devm_request_threaded_irq(): %d", err);
		goto free_mc_irq;
	}

	err = dpni_set_irq_mask(ls_dev->mc_io, 0, ls_dev->mc_handle,
				irq_index, mask);
	if (err < 0) {
		dev_err(&ls_dev->dev, "dpni_set_irq_mask(): %d", err);
		goto free_irq;
	}

	err = dpni_set_irq_enable(ls_dev->mc_io, 0, ls_dev->mc_handle,
				  irq_index, 1);
	if (err < 0) {
		dev_err(&ls_dev->dev, "dpni_set_irq_enable(): %d", err);
		goto free_irq;
	}

	return 0;

free_irq:
	devm_free_irq(&ls_dev->dev, irq->msi_desc->irq, &ls_dev->dev);
free_mc_irq:
	fsl_mc_free_irqs(ls_dev);

	return err;
}

static void add_ch_napi(struct dpaa2_eth_priv *priv)
{
	int i;
	struct dpaa2_eth_channel *ch;

	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		/* NAPI weight *MUST* be a multiple of DPAA2_ETH_STORE_SIZE */
		netif_napi_add(priv->net_dev, &ch->napi, dpaa2_eth_poll,
			       NAPI_POLL_WEIGHT);
	}
}

static void del_ch_napi(struct dpaa2_eth_priv *priv)
{
	int i;
	struct dpaa2_eth_channel *ch;

	for (i = 0; i < priv->num_channels; i++) {
		ch = priv->channel[i];
		netif_napi_del(&ch->napi);
	}
}

/* SysFS support */
static ssize_t dpaa2_eth_show_tx_shaping(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct dpaa2_eth_priv *priv = netdev_priv(to_net_dev(dev));
	/* No MC API for getting the shaping config. We're stateful. */
	struct dpni_tx_shaping_cfg *scfg = &priv->shaping_cfg;

	return sprintf(buf, "%u %hu\n", scfg->rate_limit, scfg->max_burst_size);
}

static ssize_t dpaa2_eth_write_tx_shaping(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t count)
{
	int err, items;
	struct dpaa2_eth_priv *priv = netdev_priv(to_net_dev(dev));
	struct dpni_tx_shaping_cfg scfg;

	items = sscanf(buf, "%u %hu", &scfg.rate_limit, &scfg.max_burst_size);
	if (items != 2) {
		pr_err("Expected format: \"rate_limit(Mbps) max_burst_size(bytes)\"\n");
		return -EINVAL;
	}
	/* Size restriction as per MC API documentation */
	if (scfg.max_burst_size > 64000) {
		pr_err("max_burst_size must be <= 64000, thanks.\n");
		return -EINVAL;
	}

	err = dpni_set_tx_shaping(priv->mc_io, 0, priv->mc_token, &scfg);
	if (err) {
		dev_err(dev, "dpni_set_tx_shaping() failed\n");
		return -EPERM;
	}
	/* If successful, save the current configuration for future inquiries */
	priv->shaping_cfg = scfg;

	return count;
}

static ssize_t dpaa2_eth_show_txconf_cpumask(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct dpaa2_eth_priv *priv = netdev_priv(to_net_dev(dev));

	return cpumap_print_to_pagebuf(1, buf, &priv->txconf_cpumask);
}

static ssize_t dpaa2_eth_write_txconf_cpumask(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf,
					      size_t count)
{
	struct dpaa2_eth_priv *priv = netdev_priv(to_net_dev(dev));
	struct dpaa2_eth_fq *fq;
	bool running = netif_running(priv->net_dev);
	int i, err;

	err = cpulist_parse(buf, &priv->txconf_cpumask);
	if (err)
		return err;

	/* Only accept CPUs that have an affine DPIO */
	if (!cpumask_subset(&priv->txconf_cpumask, &priv->dpio_cpumask)) {
		netdev_info(priv->net_dev,
			    "cpumask must be a subset of 0x%lx\n",
			    *cpumask_bits(&priv->dpio_cpumask));
		cpumask_and(&priv->txconf_cpumask, &priv->dpio_cpumask,
			    &priv->txconf_cpumask);
	}

	/* Rewiring the TxConf FQs requires interface shutdown.
	 */
	if (running) {
		err = dpaa2_eth_stop(priv->net_dev);
		if (err)
			return -ENODEV;
	}

	/* Set the new TxConf FQ affinities */
	set_fq_affinity(priv);

	/* dpaa2_eth_open() below will *stop* the Tx queues until an explicit
	 * link up notification is received. Give the polling thread enough time
	 * to detect the link state change, or else we'll end up with the
	 * transmission side forever shut down.
	 */
	if (priv->do_link_poll)
		msleep(2 * DPAA2_ETH_LINK_STATE_REFRESH);

	for (i = 0; i < priv->num_fqs; i++) {
		fq = &priv->fq[i];
		if (fq->type != DPAA2_TX_CONF_FQ)
			continue;
		setup_tx_flow(priv, fq);
	}

	if (running) {
		err = dpaa2_eth_open(priv->net_dev);
		if (err)
			return -ENODEV;
	}

	return count;
}

static struct device_attribute dpaa2_eth_attrs[] = {
	__ATTR(txconf_cpumask,
	       S_IRUSR | S_IWUSR,
	       dpaa2_eth_show_txconf_cpumask,
	       dpaa2_eth_write_txconf_cpumask),

	__ATTR(tx_shaping,
	       S_IRUSR | S_IWUSR,
	       dpaa2_eth_show_tx_shaping,
	       dpaa2_eth_write_tx_shaping),
};

void dpaa2_eth_sysfs_init(struct device *dev)
{
	int i, err;

	for (i = 0; i < ARRAY_SIZE(dpaa2_eth_attrs); i++) {
		err = device_create_file(dev, &dpaa2_eth_attrs[i]);
		if (err) {
			dev_err(dev, "ERROR creating sysfs file\n");
			goto undo;
		}
	}
	return;

undo:
	while (i > 0)
		device_remove_file(dev, &dpaa2_eth_attrs[--i]);
}

void dpaa2_eth_sysfs_remove(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dpaa2_eth_attrs); i++)
		device_remove_file(dev, &dpaa2_eth_attrs[i]);
}

static int dpaa2_eth_probe(struct fsl_mc_device *dpni_dev)
{
	struct device			*dev;
	struct net_device		*net_dev = NULL;
	struct dpaa2_eth_priv		*priv = NULL;
	int				err = 0;

	dev = &dpni_dev->dev;

	/* Net device */
	net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA2_ETH_MAX_TX_QUEUES);
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev_mq() failed\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(net_dev, dev);
	dev_set_drvdata(dev, net_dev);

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;

	/* Obtain a MC portal */
	err = fsl_mc_portal_allocate(dpni_dev, FSL_MC_IO_ATOMIC_CONTEXT_PORTAL,
				     &priv->mc_io);
	if (err) {
		dev_err(dev, "MC portal allocation failed\n");
		goto err_portal_alloc;
	}

	/* MC objects initialization and configuration */
	err = setup_dpni(dpni_dev);
	if (err)
		goto err_dpni_setup;

	err = setup_dpio(priv);
	if (err)
		goto err_dpio_setup;

	setup_fqs(priv);

	err = setup_dpbp(priv);
	if (err)
		goto err_dpbp_setup;

	err = bind_dpni(priv);
	if (err)
		goto err_bind;

	/* Add a NAPI context for each channel */
	add_ch_napi(priv);

	/* Percpu statistics */
	priv->percpu_stats = alloc_percpu(*priv->percpu_stats);
	if (!priv->percpu_stats) {
		dev_err(dev, "alloc_percpu(percpu_stats) failed\n");
		err = -ENOMEM;
		goto err_alloc_percpu_stats;
	}
	priv->percpu_extras = alloc_percpu(*priv->percpu_extras);
	if (!priv->percpu_extras) {
		dev_err(dev, "alloc_percpu(percpu_extras) failed\n");
		err = -ENOMEM;
		goto err_alloc_percpu_extras;
	}

	snprintf(net_dev->name, IFNAMSIZ, "ni%d", dpni_dev->obj_desc.id);
	if (!dev_valid_name(net_dev->name)) {
		dev_warn(&net_dev->dev,
			 "netdevice name \"%s\" cannot be used, reverting to default..\n",
			 net_dev->name);
		dev_alloc_name(net_dev, "eth%d");
		dev_warn(&net_dev->dev, "using name \"%s\"\n", net_dev->name);
	}

	err = netdev_init(net_dev);
	if (err)
		goto err_netdev_init;

	/* Configure checksum offload based on current interface flags */
	err = set_rx_csum(priv, !!(net_dev->features & NETIF_F_RXCSUM));
	if (err)
		goto err_csum;

	err = set_tx_csum(priv, !!(net_dev->features &
				   (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM)));
	if (err)
		goto err_csum;

	err = alloc_rings(priv);
	if (err)
		goto err_alloc_rings;

	net_dev->ethtool_ops = &dpaa2_ethtool_ops;

	err = setup_irqs(dpni_dev);
	if (err) {
		netdev_warn(net_dev, "Failed to set link interrupt, fall back to polling\n");
		priv->poll_thread = kthread_run(poll_link_state, priv,
						"%s_poll_link", net_dev->name);
		if (IS_ERR(priv->poll_thread)) {
			netdev_err(net_dev, "Error starting polling thread\n");
			goto err_poll_thread;
		}
		priv->do_link_poll = true;
	}

	dpaa2_eth_sysfs_init(&net_dev->dev);
	dpaa2_dbg_add(priv);

	dev_info(dev, "Probed interface %s\n", net_dev->name);
	return 0;

err_poll_thread:
	free_rings(priv);
err_alloc_rings:
err_csum:
	unregister_netdev(net_dev);
err_netdev_init:
	free_percpu(priv->percpu_extras);
err_alloc_percpu_extras:
	free_percpu(priv->percpu_stats);
err_alloc_percpu_stats:
	del_ch_napi(priv);
err_bind:
	free_dpbp(priv);
err_dpbp_setup:
	free_dpio(priv);
err_dpio_setup:
	kfree(priv->cls_rule);
	dpni_close(priv->mc_io, 0, priv->mc_token);
err_dpni_setup:
	fsl_mc_portal_free(priv->mc_io);
err_portal_alloc:
	dev_set_drvdata(dev, NULL);
	free_netdev(net_dev);

	return err;
}

static int dpaa2_eth_remove(struct fsl_mc_device *ls_dev)
{
	struct device		*dev;
	struct net_device	*net_dev;
	struct dpaa2_eth_priv *priv;

	dev = &ls_dev->dev;
	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	dpaa2_dbg_remove(priv);
	dpaa2_eth_sysfs_remove(&net_dev->dev);

	unregister_netdev(net_dev);
	dev_info(net_dev->dev.parent, "Removed interface %s\n", net_dev->name);

	free_dpio(priv);
	free_rings(priv);
	del_ch_napi(priv);
	free_dpbp(priv);
	free_dpni(priv);

	fsl_mc_portal_free(priv->mc_io);

	free_percpu(priv->percpu_stats);
	free_percpu(priv->percpu_extras);

	if (priv->do_link_poll)
		kthread_stop(priv->poll_thread);
	else
		fsl_mc_free_irqs(ls_dev);

	kfree(priv->cls_rule);

	dev_set_drvdata(dev, NULL);
	free_netdev(net_dev);

	return 0;
}

static const struct fsl_mc_device_match_id dpaa2_eth_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpni",
		.ver_major = DPNI_VER_MAJOR,
		.ver_minor = DPNI_VER_MINOR
	},
	{ .vendor = 0x0 }
};

static struct fsl_mc_driver dpaa2_eth_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa2_eth_probe,
	.remove		= dpaa2_eth_remove,
	.match_id_table = dpaa2_eth_match_id_table
};

static int __init dpaa2_eth_driver_init(void)
{
	int err;

	dpaa2_eth_dbg_init();

	err = fsl_mc_driver_register(&dpaa2_eth_driver);
	if (err) {
		dpaa2_eth_dbg_exit();
		return err;
	}

	return 0;
}

static void __exit dpaa2_eth_driver_exit(void)
{
	fsl_mc_driver_unregister(&dpaa2_eth_driver);
	dpaa2_eth_dbg_exit();
}

module_init(dpaa2_eth_driver_init);
module_exit(dpaa2_eth_driver_exit);
