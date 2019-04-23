/* Copyright 2008-2013 Freescale Semiconductor Inc.
 * Copyright 2019 NXP
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

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__
#else
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": " fmt
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/highmem.h>
#include <linux/fsl_qman.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "dpaa_eth_base.h"
#include "lnxwrp_fsl_fman.h" /* fm_get_rx_extra_headroom(), fm_get_max_frm() */
#include "mac.h"

/* forward declarations */
static enum qman_cb_dqrr_result __hot
shared_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq);
static enum qman_cb_dqrr_result __hot
shared_tx_default_dqrr(struct qman_portal              *portal,
		       struct qman_fq                  *fq,
		       const struct qm_dqrr_entry      *dq);
static enum qman_cb_dqrr_result
shared_tx_error_dqrr(struct qman_portal                *portal,
		     struct qman_fq                    *fq,
		     const struct qm_dqrr_entry        *dq);
static void shared_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg);

#define DPA_DESCRIPTION "FSL DPAA Shared Ethernet driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

/* This has to work in tandem with the DPA_CS_THRESHOLD_xxx values. */
static uint16_t shared_tx_timeout = 1000;
module_param(shared_tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(shared_tx_timeout, "The Tx timeout in ms");

static const struct of_device_id dpa_shared_match[];

static const struct net_device_ops dpa_shared_ops = {
	.ndo_open = dpa_start,
	.ndo_start_xmit = dpa_shared_tx,
	.ndo_stop = dpa_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_rx_mode = dpa_set_rx_mode,
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_fix_features,
	.ndo_do_ioctl = dpa_ioctl,
};

const struct dpa_fq_cbs_t shared_fq_cbs = {
	.rx_defq = { .cb = { .dqrr = shared_rx_dqrr } },
	.tx_defq = { .cb = { .dqrr = shared_tx_default_dqrr } },
	.rx_errq = { .cb = { .dqrr = shared_rx_dqrr } },
	.tx_errq = { .cb = { .dqrr = shared_tx_error_dqrr } },
	.egress_ern = { .cb = { .ern = shared_ern } }
};
EXPORT_SYMBOL(shared_fq_cbs);

static inline void * __must_check __attribute__((nonnull))
dpa_phys2virt(const struct dpa_bp *dpa_bp, dma_addr_t addr)
{
	return dpa_bp->vaddr + (addr - dpa_bp->paddr);
}

static struct dpa_bp *dpa_size2pool(struct dpa_priv_s *priv, size_t size)
{
	int i;

	for (i = 0; i < priv->bp_count; i++)
		if ((size + priv->tx_headroom) <= priv->dpa_bp[i].size)
			return dpa_bpid2pool(priv->dpa_bp[i].bpid);
	return ERR_PTR(-ENODEV);
}

/* Copy to a memory region that requires kmapping from a linear buffer,
 * taking into account page boundaries in the destination
 */
static void
copy_to_unmapped_area(dma_addr_t phys_start, void *src, size_t buf_size)
{
	struct page *page;
	size_t size, offset;
	void *page_vaddr;

	while (buf_size > 0) {
		offset = offset_in_page(phys_start);
		size = (offset + buf_size > PAGE_SIZE) ?
				PAGE_SIZE - offset : buf_size;

		page = pfn_to_page(phys_start >> PAGE_SHIFT);
		page_vaddr = kmap_atomic(page);

		memcpy(page_vaddr + offset, src, size);

		kunmap_atomic(page_vaddr);

		phys_start += size;
		src += size;
		buf_size -= size;
	}
}

/* Copy from a memory region that requires kmapping to a linear buffer,
 * taking into account page boundaries in the source
 */
static void
copy_from_unmapped_area(void *dest, dma_addr_t phys_start, size_t buf_size)
{
	struct page *page;
	size_t size, offset;
	void *page_vaddr;

	while (buf_size > 0) {
		offset = offset_in_page(phys_start);
		size = (offset + buf_size > PAGE_SIZE) ?
			PAGE_SIZE - offset : buf_size;

		page = pfn_to_page(phys_start >> PAGE_SHIFT);
		page_vaddr = kmap_atomic(page);

		memcpy(dest, page_vaddr + offset, size);

		kunmap_atomic(page_vaddr);

		phys_start += size;
		dest += size;
		buf_size -= size;
	}
}

static void
dpa_fd_release_sg(const struct net_device *net_dev,
			const struct qm_fd *fd)
{
	const struct dpa_priv_s		*priv;
	struct qm_sg_entry		*sgt;
	struct dpa_bp			*_dpa_bp;
	struct bm_buffer		 _bmb;

	priv = netdev_priv(net_dev);

	_bmb.hi	= fd->addr_hi;
	_bmb.lo	= fd->addr_lo;

	_dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(!_dpa_bp);

	if (_dpa_bp->vaddr) {
		sgt = dpa_phys2virt(_dpa_bp, bm_buf_addr(&_bmb)) +
					dpa_fd_offset(fd);
		dpa_release_sgt(sgt);
	} else {
		sgt = kmalloc(DPA_SGT_MAX_ENTRIES * sizeof(*sgt), GFP_ATOMIC);
		if (sgt == NULL) {
			if (netif_msg_tx_err(priv) && net_ratelimit())
				netdev_err(net_dev,
					"Memory allocation failed\n");
			return;
		}

		copy_from_unmapped_area(sgt, bm_buf_addr(&_bmb) +
						dpa_fd_offset(fd),
					min(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
						_dpa_bp->size));
		dpa_release_sgt(sgt);
		kfree(sgt);
	}

	while (bman_release(_dpa_bp->pool, &_bmb, 1, 0))
		cpu_relax();
}

static enum qman_cb_dqrr_result __hot
shared_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	const struct qm_fd *fd = &dq->fd;
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	struct qm_sg_entry *sgt;
	int i;
	void *frag_addr;
	u32 frag_length;
	u32 offset;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(!dpa_bp);

	if (unlikely(fd->status & FM_FD_STAT_RX_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_RX_ERRORS);

		percpu_priv->stats.rx_errors++;

		goto out;
	}

	skb = __netdev_alloc_skb(net_dev,
				 priv->tx_headroom + dpa_fd_length(fd),
				 GFP_ATOMIC);
	if (unlikely(skb == NULL)) {
		if (netif_msg_rx_err(priv) && net_ratelimit())
			netdev_err(net_dev, "Could not alloc skb\n");

		percpu_priv->stats.rx_dropped++;

		goto out;
	}

	skb_reserve(skb, priv->tx_headroom);

	if (fd->format == qm_fd_sg) {
		if (dpa_bp->vaddr) {
			sgt = dpa_phys2virt(dpa_bp,
					    qm_fd_addr(fd)) + dpa_fd_offset(fd);

			for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
				offset = qm_sg_entry_get_offset(&sgt[i]);
				frag_addr = dpa_phys2virt(dpa_bp,
							  qm_sg_addr(&sgt[i]) +
							  offset);
				DPA_BUG_ON(qm_sg_entry_get_ext(&sgt[i]));
				frag_length = qm_sg_entry_get_len(&sgt[i]);

				/* copy from sgt[i] */
				memcpy(skb_put(skb, frag_length), frag_addr,
				       frag_length);
				if (qm_sg_entry_get_final(&sgt[i]))
					break;
			}
		} else {
			sgt = kmalloc(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
					GFP_ATOMIC);
			if (unlikely(sgt == NULL)) {
				if (netif_msg_tx_err(priv) && net_ratelimit())
					netdev_err(net_dev,
						"Memory allocation failed\n");
				dev_kfree_skb_any(skb);
				return -ENOMEM;
			}

			copy_from_unmapped_area(sgt,
					qm_fd_addr(fd) + dpa_fd_offset(fd),
					min(DPA_SGT_MAX_ENTRIES * sizeof(*sgt),
							dpa_bp->size));

			for (i = 0; i < DPA_SGT_MAX_ENTRIES; i++) {
				DPA_BUG_ON(qm_sg_entry_get_ext(&sgt[i]));
				frag_length = qm_sg_entry_get_len(&sgt[i]);
				copy_from_unmapped_area(
						skb_put(skb, frag_length),
						qm_sg_addr(&sgt[i]) +
						qm_sg_entry_get_offset(&sgt[i]),
						frag_length);

				if (qm_sg_entry_get_final(&sgt[i]))
					break;
			}

			kfree(sgt);
		}
		goto skb_copied;
	}

	/* otherwise fd->format == qm_fd_contig */
	if (dpa_bp->vaddr) {
		/* Fill the SKB */
		memcpy(skb_put(skb, dpa_fd_length(fd)),
		       dpa_phys2virt(dpa_bp, qm_fd_addr(fd)) +
		       dpa_fd_offset(fd), dpa_fd_length(fd));
	} else {
		copy_from_unmapped_area(skb_put(skb, dpa_fd_length(fd)),
					qm_fd_addr(fd) + dpa_fd_offset(fd),
					dpa_fd_length(fd));
	}

skb_copied:
	skb->protocol = eth_type_trans(skb, net_dev);

	if (unlikely(netif_rx(skb) != NET_RX_SUCCESS))
		goto out;
	else {
		percpu_priv->stats.rx_packets++;
		percpu_priv->stats.rx_bytes += dpa_fd_length(fd);
	}

out:
	if (fd->format == qm_fd_sg)
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result
shared_tx_error_dqrr(struct qman_portal                *portal,
		     struct qman_fq                    *fq,
		     const struct qm_dqrr_entry        *dq)
{
	struct net_device               *net_dev;
	struct dpa_priv_s               *priv;
	struct dpa_percpu_priv_s        *percpu_priv;
	struct dpa_bp			*dpa_bp;
	const struct qm_fd		*fd = &dq->fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(!dpa_bp);

	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	if (netif_msg_hw(priv) && net_ratelimit())
		netdev_warn(net_dev, "FD status = 0x%08x\n",
				fd->status & FM_FD_STAT_TX_ERRORS);

	if ((fd->format == qm_fd_sg) && (!dpa_bp->vaddr))
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	percpu_priv->stats.tx_errors++;

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result __hot
shared_tx_default_dqrr(struct qman_portal              *portal,
		       struct qman_fq                  *fq,
		       const struct qm_dqrr_entry      *dq)
{
	struct net_device               *net_dev;
	struct dpa_priv_s               *priv;
	struct dpa_percpu_priv_s        *percpu_priv;
	struct dpa_bp			*dpa_bp;
	const struct qm_fd		*fd = &dq->fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(!dpa_bp);

	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	if (unlikely(fd->status & FM_FD_STAT_TX_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_TX_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	if ((fd->format == qm_fd_sg) && (!dpa_bp->vaddr))
		dpa_fd_release_sg(net_dev, fd);
	else
		dpa_fd_release(net_dev, fd);

	percpu_priv->tx_confirm++;

	return qman_cb_dqrr_consume;
}

static void shared_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *dpa_fq = (struct dpa_fq *)fq;

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	dpa_fd_release(net_dev, &msg->ern.fd);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);
}

int __hot dpa_shared_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_bp *dpa_bp;
	struct bm_buffer bmb;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_priv_s *priv;
	struct qm_fd fd;
	int queue_mapping;
	int err;
	void *dpa_bp_vaddr;
	fm_prs_result_t parse_results;
	fm_prs_result_t *parse_results_ref;
	struct qman_fq *egress_fq, *conf_fq;

	priv = netdev_priv(net_dev);
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	memset(&fd, 0, sizeof(fd));
	fd.format = qm_fd_contig;

	queue_mapping = smp_processor_id();

	dpa_bp = dpa_size2pool(priv, skb_headlen(skb));
	if (unlikely(!dpa_bp)) {
		percpu_priv->stats.tx_errors++;
		err = PTR_ERR(dpa_bp);
		goto bpools_too_small_error;
	}

	err = bman_acquire(dpa_bp->pool, &bmb, 1, 0);
	if (unlikely(err <= 0)) {
		percpu_priv->stats.tx_errors++;
		if (err == 0)
			err = -ENOMEM;
		goto buf_acquire_failed;
	}
	fd.bpid = dpa_bp->bpid;

	fd.length20 = skb_headlen(skb);
	fd.addr_hi = (uint8_t)bmb.hi;
	fd.addr_lo = bmb.lo;
	fd.offset = priv->tx_headroom;

	/* The virtual address of the buffer pool is expected to be NULL
	 * in scenarios like MAC-less or Shared-MAC between Linux and
	 * USDPAA. In this case the buffers are dynamically mapped/unmapped.
	 */
	if (dpa_bp->vaddr) {
		dpa_bp_vaddr = dpa_phys2virt(dpa_bp, bm_buf_addr(&bmb));

		/* Copy the packet payload */
		skb_copy_from_linear_data(skb,
					  dpa_bp_vaddr + dpa_fd_offset(&fd),
					  dpa_fd_length(&fd));

		/* if no mac device or peer set it's macless */
		if (!priv->mac_dev || priv->peer) {
			parse_results_ref = (fm_prs_result_t *) (dpa_bp_vaddr +
				DPA_TX_PRIV_DATA_SIZE);
			/* Default values; FMan will not generate/validate
			 * CSUM;
			 */
			parse_results_ref->l3r = 0;
			parse_results_ref->l4r = 0;
			parse_results_ref->ip_off[0] = 0xff;
			parse_results_ref->ip_off[1] = 0xff;
			parse_results_ref->l4_off = 0xff;

			fd.cmd |= FM_FD_CMD_DTC | FM_FD_CMD_RPD;
		} else {
			/* Enable L3/L4 hardware checksum computation,
			* if applicable
			*/
			err = dpa_enable_tx_csum(priv, skb, &fd,
					 dpa_bp_vaddr + DPA_TX_PRIV_DATA_SIZE);

			if (unlikely(err < 0)) {
				if (netif_msg_tx_err(priv) && net_ratelimit())
					netdev_err(net_dev,
						"Tx HW csum error: %d\n", err);
				percpu_priv->stats.tx_errors++;
				goto l3_l4_csum_failed;
			}
		}

	} else {
		if (!priv->mac_dev || priv->peer) {
			/* Default values; FMan will not generate/validate
			 * CSUM;
			 */
			parse_results.l3r = 0;
			parse_results.l4r = 0;
			parse_results.ip_off[0] = 0xff;
			parse_results.ip_off[1] = 0xff;
			parse_results.l4_off = 0xff;

			fd.cmd |= FM_FD_CMD_DTC | FM_FD_CMD_RPD;
		} else {
			/* Enable L3/L4 hardware checksum computation,
			 * if applicable
			 */
			err = dpa_enable_tx_csum(priv, skb, &fd,
						(char *)&parse_results);

			if (unlikely(err < 0)) {
				if (netif_msg_tx_err(priv) && net_ratelimit())
					netdev_err(net_dev,
						"Tx HW csum error: %d\n", err);
				percpu_priv->stats.tx_errors++;
				goto l3_l4_csum_failed;
			}

		}

		copy_to_unmapped_area(bm_buf_addr(&bmb) + DPA_TX_PRIV_DATA_SIZE,
				&parse_results,
				DPA_PARSE_RESULTS_SIZE);

		copy_to_unmapped_area(bm_buf_addr(&bmb) + dpa_fd_offset(&fd),
				skb->data,
				dpa_fd_length(&fd));
	}

	egress_fq = priv->egress_fqs[queue_mapping];
	conf_fq = priv->conf_fqs[queue_mapping];

	err = dpa_xmit(priv, &percpu_priv->stats, &fd, egress_fq, conf_fq);

l3_l4_csum_failed:
bpools_too_small_error:
buf_acquire_failed:
	/* We're done with the skb */
	dev_kfree_skb(skb);

	/* err remains unused, NETDEV_TX_OK must be returned here */
	return NETDEV_TX_OK;
}
EXPORT_SYMBOL(dpa_shared_tx);

static int dpa_shared_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	const uint8_t *mac_addr;

	net_dev->netdev_ops = &dpa_shared_ops;

	net_dev->mem_start = priv->mac_dev->res->start;
	net_dev->mem_end = priv->mac_dev->res->end;

	/* Configure the maximum MTU according to the FMan's MAXFRM */
	net_dev->min_mtu = ETH_MIN_MTU;
	net_dev->max_mtu = dpa_get_max_mtu();

	mac_addr = priv->mac_dev->addr;

	net_dev->hw_features |= (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		NETIF_F_LLTX);

	return dpa_netdev_init(net_dev, mac_addr, shared_tx_timeout);
}

#ifdef CONFIG_PM

static int dpa_shared_suspend(struct device *dev)
{
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;
	struct mac_device	*mac_dev;
	int			err = 0;

	net_dev = dev_get_drvdata(dev);
	if (net_dev->flags & IFF_UP) {
		priv = netdev_priv(net_dev);
		mac_dev = priv->mac_dev;

		err = fm_port_suspend(mac_dev->port_dev[RX]);
		if (err)
			goto port_suspend_failed;

		err = fm_port_suspend(mac_dev->port_dev[TX]);
		if (err)
			err = fm_port_resume(mac_dev->port_dev[RX]);
	}

port_suspend_failed:
	return err;
}

static int dpa_shared_resume(struct device *dev)
{
	struct net_device	*net_dev;
	struct dpa_priv_s	*priv;
	struct mac_device	*mac_dev;
	int			err = 0;

	net_dev = dev_get_drvdata(dev);
	if (net_dev->flags & IFF_UP) {
		priv = netdev_priv(net_dev);
		mac_dev = priv->mac_dev;

		err = fm_port_resume(mac_dev->port_dev[TX]);
		if (err)
			goto port_resume_failed;

		err = fm_port_resume(mac_dev->port_dev[RX]);
		if (err)
			err = fm_port_suspend(mac_dev->port_dev[TX]);
	}

port_resume_failed:
	return err;
}

static const struct dev_pm_ops shared_pm_ops = {
	.suspend = dpa_shared_suspend,
	.resume = dpa_shared_resume,
};

#define SHARED_PM_OPS (&shared_pm_ops)

#else /* CONFIG_PM */

#define SHARED_PM_OPS NULL

#endif /* CONFIG_PM */

static int
dpaa_eth_shared_probe(struct platform_device *_of_dev)
{
	int err = 0, i, channel;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	size_t count;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *priv = NULL;
	struct dpa_percpu_priv_s *percpu_priv;
	struct fm_port_fqs port_fqs;
	struct dpa_buffer_layout_s *buf_layout = NULL;
	struct mac_device *mac_dev;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/* Get the buffer pools assigned to this interface */
	dpa_bp = dpa_bp_probe(_of_dev, &count);
	if (IS_ERR(dpa_bp))
		return PTR_ERR(dpa_bp);

	for (i = 0; i < count; i++)
		dpa_bp[i].seed_cb = dpa_bp_shared_port_seed;

	/* Allocate this early, so we can store relevant information in
	 * the private area (needed by 1588 code in dpa_mac_probe)
	 */
	net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev_mq() failed\n");
		return -ENOMEM;
	}

	/* Do this here, so we can be verbose early */
	SET_NETDEV_DEV(net_dev, dev);
	dev_set_drvdata(dev, net_dev);

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	strcpy(priv->if_type, "shared");

	priv->msg_enable = netif_msg_init(advanced_debug, -1);

	mac_dev = dpa_mac_probe(_of_dev);
	if (IS_ERR(mac_dev) || !mac_dev) {
		err = PTR_ERR(mac_dev);
		goto mac_probe_failed;
	}

	/* We have physical ports, so we need to establish
	 * the buffer layout.
	 */
	buf_layout = devm_kzalloc(dev, 2 * sizeof(*buf_layout),
				  GFP_KERNEL);
	if (!buf_layout) {
		dev_err(dev, "devm_kzalloc() failed\n");
		goto alloc_failed;
	}
	dpa_set_buffers_layout(mac_dev, buf_layout);

	INIT_LIST_HEAD(&priv->dpa_fq_list);

	memset(&port_fqs, 0, sizeof(port_fqs));

	err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list, &port_fqs,
			       false, RX);
	if (!err)
		err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list,
				       &port_fqs, false, TX);
	if (err < 0)
		goto fq_probe_failed;

	/* bp init */
	priv->bp_count = count;
	err = dpa_bp_create(net_dev, dpa_bp, count);
	if (err < 0)
		goto bp_create_failed;

	priv->mac_dev = mac_dev;

	channel = dpa_get_channel();

	if (channel < 0) {
		err = channel;
		goto get_channel_failed;
	}

	priv->channel = (uint16_t)channel;

	dpaa_eth_add_channel(priv->channel);

	dpa_fq_setup(priv, &shared_fq_cbs, priv->mac_dev->port_dev[TX]);

	/* Create a congestion group for this netdev, with
	 * dynamically-allocated CGR ID.
	 * Must be executed after probing the MAC, but before
	 * assigning the egress FQs to the CGRs.
	 */
	err = dpaa_eth_cgr_init(priv);
	if (err < 0) {
		dev_err(dev, "Error initializing CGR\n");
		goto cgr_init_failed;
	}

	/* Add the FQs to the interface, and make them active */
	err = dpa_fqs_init(dev,  &priv->dpa_fq_list, false);
	if (err < 0)
		goto fq_alloc_failed;

	priv->buf_layout = buf_layout;
	priv->tx_headroom =
		dpa_get_headroom(&priv->buf_layout[TX]);

	/* All real interfaces need their ports initialized */
	dpaa_eth_init_ports(mac_dev, dpa_bp, count, &port_fqs,
			buf_layout, dev);

	/* Now we need to initialize either a private or shared interface */
	priv->percpu_priv = devm_alloc_percpu(dev, *priv->percpu_priv);

	if (priv->percpu_priv == NULL) {
		dev_err(dev, "devm_alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}
	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		memset(percpu_priv, 0, sizeof(*percpu_priv));
	}

	err = dpa_shared_netdev_init(dpa_node, net_dev);

	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

	pr_info("fsl_dpa_shared: Probed shared interface %s\n",
			net_dev->name);

	return 0;

netdev_init_failed:
alloc_percpu_failed:
fq_alloc_failed:
	if (net_dev) {
		dpa_fq_free(dev, &priv->dpa_fq_list);
		qman_release_cgrid(priv->cgr_data.cgr.cgrid);
		qman_delete_cgr(&priv->cgr_data.cgr);
	}
cgr_init_failed:
get_channel_failed:
	if (net_dev)
		dpa_bp_free(priv);
bp_create_failed:
fq_probe_failed:
	devm_kfree(dev, buf_layout);
alloc_failed:
mac_probe_failed:
	dev_set_drvdata(dev, NULL);
	if (net_dev)
		free_netdev(net_dev);

	return err;
}

static const struct of_device_id dpa_shared_match[] = {
	{
		.compatible	= "fsl,dpa-ethernet-shared"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_shared_match);

static struct platform_driver dpa_shared_driver = {
	.driver = {
		.name		= KBUILD_MODNAME "-shared",
		.of_match_table	= dpa_shared_match,
		.owner		= THIS_MODULE,
		.pm		= SHARED_PM_OPS,
	},
	.probe		= dpaa_eth_shared_probe,
	.remove		= dpa_remove
};

static int __init __cold dpa_shared_load(void)
{
	int	 _errno;

	pr_info(DPA_DESCRIPTION "\n");

	/* Initialize dpaa_eth mirror values */
	dpa_rx_extra_headroom = fm_get_rx_extra_headroom();
	dpa_max_frm = fm_get_max_frm();

	_errno = platform_driver_register(&dpa_shared_driver);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);

	return _errno;
}
module_init(dpa_shared_load);

static void __exit __cold dpa_shared_unload(void)
{
	pr_debug(KBUILD_MODNAME ": -> %s:%s()\n",
		KBUILD_BASENAME".c", __func__);

	platform_driver_unregister(&dpa_shared_driver);
}
module_exit(dpa_shared_unload);
