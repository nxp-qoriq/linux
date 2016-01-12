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

#include "dpni.h"	/* DPNI_LINK_OPT_* */
#include "dpaa2-eth.h"

/* size of DMA memory used to pass configuration to classifier, in bytes */
#define DPAA2_CLASSIFIER_DMA_SIZE 256

/* To be kept in sync with 'enum dpni_counter' */
char dpaa2_ethtool_stats[][ETH_GSTRING_LEN] = {
	"rx frames",
	"rx bytes",
	"rx frames dropped",
	"rx err frames",
	"rx mcast frames",
	"rx mcast bytes",
	"rx bcast frames",
	"rx bcast bytes",
	"tx frames",
	"tx bytes",
	"tx err frames",
};

#define DPAA2_ETH_NUM_STATS	ARRAY_SIZE(dpaa2_ethtool_stats)

/* To be kept in sync with 'struct dpaa2_eth_drv_stats' */
char dpaa2_ethtool_extras[][ETH_GSTRING_LEN] = {
	/* per-cpu stats */

	"tx conf frames",
	"tx conf bytes",
	"tx sg frames",
	"tx sg bytes",
	"rx sg frames",
	"rx sg bytes",
	/* how many times we had to retry the enqueue command */
	"enqueue portal busy",

	/* Channel stats */
	/* How many times we had to retry the volatile dequeue command */
	"dequeue portal busy",
	"channel pull errors",
	/* Number of notifications received */
	"cdan",
#ifdef CONFIG_FSL_QBMAN_DEBUG
	/* FQ stats */
	"rx pending frames",
	"rx pending bytes",
	"tx conf pending frames",
	"tx conf pending bytes",
	"buffer count"
#endif
};

#define DPAA2_ETH_NUM_EXTRA_STATS	ARRAY_SIZE(dpaa2_ethtool_extras)

static void dpaa2_eth_get_drvinfo(struct net_device *net_dev,
				  struct ethtool_drvinfo *drvinfo)
{
	struct mc_version mc_ver;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	char fw_version[ETHTOOL_FWVERS_LEN];
	char version[32];
	int err;

	err = mc_get_version(priv->mc_io, 0, &mc_ver);
	if (err) {
		strlcpy(drvinfo->fw_version, "Error retrieving MC version",
			sizeof(drvinfo->fw_version));
	} else {
		scnprintf(fw_version, sizeof(fw_version), "%d.%d.%d",
			  mc_ver.major, mc_ver.minor, mc_ver.revision);
		strlcpy(drvinfo->fw_version, fw_version,
			sizeof(drvinfo->fw_version));
	}

	scnprintf(version, sizeof(version), "%d.%d", DPNI_VER_MAJOR,
		  DPNI_VER_MINOR);
	strlcpy(drvinfo->version, version, sizeof(drvinfo->version));

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int dpaa2_eth_get_settings(struct net_device *net_dev,
				  struct ethtool_cmd *cmd)
{
	struct dpni_link_state state = {0};
	int err = 0;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (err) {
		netdev_err(net_dev, "ERROR %d getting link state", err);
		goto out;
	}

	/* At the moment, we have no way of interrogating the DPMAC
	 * from the DPNI side - and for that matter there may exist
	 * no DPMAC at all. So for now we just don't report anything
	 * beyond the DPNI attributes.
	 */
	if (state.options & DPNI_LINK_OPT_AUTONEG)
		cmd->autoneg = AUTONEG_ENABLE;
	if (!(state.options & DPNI_LINK_OPT_HALF_DUPLEX))
		cmd->duplex = DUPLEX_FULL;
	ethtool_cmd_speed_set(cmd, state.rate);

out:
	return err;
}

static int dpaa2_eth_set_settings(struct net_device *net_dev,
				  struct ethtool_cmd *cmd)
{
	struct dpni_link_cfg cfg = {0};
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int err = 0;

	netdev_dbg(net_dev, "Setting link parameters...");

	/* Due to a temporary firmware limitation, the DPNI must be down
	 * in order to be able to change link settings. Taking steps to let
	 * the user know that.
	 */
	if (netif_running(net_dev)) {
		netdev_info(net_dev, "Sorry, interface must be brought down first.\n");
		return -EACCES;
	}

	cfg.rate = ethtool_cmd_speed(cmd);
	if (cmd->autoneg == AUTONEG_ENABLE)
		cfg.options |= DPNI_LINK_OPT_AUTONEG;
	else
		cfg.options &= ~DPNI_LINK_OPT_AUTONEG;
	if (cmd->duplex  == DUPLEX_HALF)
		cfg.options |= DPNI_LINK_OPT_HALF_DUPLEX;
	else
		cfg.options &= ~DPNI_LINK_OPT_HALF_DUPLEX;

	err = dpni_set_link_cfg(priv->mc_io, 0, priv->mc_token, &cfg);
	if (err)
		/* ethtool will be loud enough if we return an error; no point
		 * in putting our own error message on the console by default
		 */
		netdev_dbg(net_dev, "ERROR %d setting link cfg", err);

	return err;
}

static void dpaa2_eth_get_strings(struct net_device *netdev, u32 stringset,
				  u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < DPAA2_ETH_NUM_STATS; i++) {
			strlcpy(p, dpaa2_ethtool_stats[i], ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		for (i = 0; i < DPAA2_ETH_NUM_EXTRA_STATS; i++) {
			strlcpy(p, dpaa2_ethtool_extras[i], ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
}

static int dpaa2_eth_get_sset_count(struct net_device *net_dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS: /* ethtool_get_stats(), ethtool_get_drvinfo() */
		return DPAA2_ETH_NUM_STATS + DPAA2_ETH_NUM_EXTRA_STATS;
	default:
		return -EOPNOTSUPP;
	}
}

/** Fill in hardware counters, as returned by the MC firmware.
 */
static void dpaa2_eth_get_ethtool_stats(struct net_device *net_dev,
					struct ethtool_stats *stats,
					u64 *data)
{
	int i; /* Current index in the data array */
	int j, k, err;

#ifdef CONFIG_FSL_QBMAN_DEBUG
	u32 fcnt, bcnt;
	u32 fcnt_rx_total = 0, fcnt_tx_total = 0;
	u32 bcnt_rx_total = 0, bcnt_tx_total = 0;
	u32 buf_cnt;
#endif
	u64 cdan = 0;
	u64 portal_busy = 0, pull_err = 0;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpaa2_eth_drv_stats *extras;
	struct dpaa2_eth_ch_stats *ch_stats;

	memset(data, 0,
	       sizeof(u64) * (DPAA2_ETH_NUM_STATS + DPAA2_ETH_NUM_EXTRA_STATS));

	/* Print standard counters, from DPNI statistics */
	for (i = 0; i < DPAA2_ETH_NUM_STATS; i++) {
		err = dpni_get_counter(priv->mc_io, 0, priv->mc_token, i,
				       data + i);
		if (err != 0)
			netdev_warn(net_dev, "Err %d getting DPNI counter %d",
				    err, i);
	}

	/* Print per-cpu extra stats */
	for_each_online_cpu(k) {
		extras = per_cpu_ptr(priv->percpu_extras, k);
		for (j = 0; j < sizeof(*extras) / sizeof(__u64); j++)
			*((__u64 *)data + i + j) += *((__u64 *)extras + j);
	}
	i += j;

	/* We may be using fewer DPIOs than actual CPUs */
	for_each_cpu(j, &priv->dpio_cpumask) {
		ch_stats = &priv->channel[j]->stats;
		cdan += ch_stats->cdan;
		portal_busy += ch_stats->dequeue_portal_busy;
		pull_err += ch_stats->pull_err;
	}

	*(data + i++) = portal_busy;
	*(data + i++) = pull_err;
	*(data + i++) = cdan;

#ifdef CONFIG_FSL_QBMAN_DEBUG
	for (j = 0; j < priv->num_fqs; j++) {
		/* Print FQ instantaneous counts */
		err = dpaa2_io_query_fq_count(NULL, priv->fq[j].fqid,
					      &fcnt, &bcnt);
		if (err) {
			netdev_warn(net_dev, "FQ query error %d", err);
			return;
		}

		if (priv->fq[j].type == DPAA2_TX_CONF_FQ) {
			fcnt_tx_total += fcnt;
			bcnt_tx_total += bcnt;
		} else {
			fcnt_rx_total += fcnt;
			bcnt_rx_total += bcnt;
		}
	}
	*(data + i++) = fcnt_rx_total;
	*(data + i++) = bcnt_rx_total;
	*(data + i++) = fcnt_tx_total;
	*(data + i++) = bcnt_tx_total;

	err = dpaa2_io_query_bp_count(NULL, priv->dpbp_attrs.bpid, &buf_cnt);
	if (err) {
		netdev_warn(net_dev, "Buffer count query error %d\n", err);
		return;
	}
	*(data + i++) = buf_cnt;
#endif
}

static const struct dpaa2_eth_hash_fields {
	u64 rxnfc_field;
	enum net_prot cls_prot;
	int cls_field;
	int size;
} hash_fields[] = {
	{
		/* L2 header */
		.rxnfc_field = RXH_L2DA,
		.cls_prot = NET_PROT_ETH,
		.cls_field = NH_FLD_ETH_DA,
		.size = 6,
	}, {
		/* VLAN header */
		.rxnfc_field = RXH_VLAN,
		.cls_prot = NET_PROT_VLAN,
		.cls_field = NH_FLD_VLAN_TCI,
		.size = 2,
	}, {
		/* IP header */
		.rxnfc_field = RXH_IP_SRC,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_SRC,
		.size = 4,
	}, {
		.rxnfc_field = RXH_IP_DST,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_DST,
		.size = 4,
	}, {
		.rxnfc_field = RXH_L3_PROTO,
		.cls_prot = NET_PROT_IP,
		.cls_field = NH_FLD_IP_PROTO,
		.size = 1,
	}, {
		/* Using UDP ports, this is functionally equivalent to raw
		 * byte pairs from L4 header.
		 */
		.rxnfc_field = RXH_L4_B_0_1,
		.cls_prot = NET_PROT_UDP,
		.cls_field = NH_FLD_UDP_PORT_SRC,
		.size = 2,
	}, {
		.rxnfc_field = RXH_L4_B_2_3,
		.cls_prot = NET_PROT_UDP,
		.cls_field = NH_FLD_UDP_PORT_DST,
		.size = 2,
	},
};

static int cls_is_enabled(struct net_device *net_dev, u64 flag)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	return !!(priv->rx_hash_fields & flag);
}

static int cls_key_off(struct net_device *net_dev, u64 flag)
{
	int i, off = 0;

	for (i = 0; i < ARRAY_SIZE(hash_fields); i++) {
		if (hash_fields[i].rxnfc_field & flag)
			return off;
		if (cls_is_enabled(net_dev, hash_fields[i].rxnfc_field))
			off += hash_fields[i].size;
	}

	return -1;
}

static u8 cls_key_size(struct net_device *net_dev)
{
	u8 i, size = 0;

	for (i = 0; i < ARRAY_SIZE(hash_fields); i++) {
		if (!cls_is_enabled(net_dev, hash_fields[i].rxnfc_field))
			continue;
		size += hash_fields[i].size;
	}

	return size;
}

static u8 cls_max_key_size(struct net_device *net_dev)
{
	u8 i, size = 0;

	for (i = 0; i < ARRAY_SIZE(hash_fields); i++)
		size += hash_fields[i].size;

	return size;
}

void check_fs_support(struct net_device *net_dev)
{
	u8 key_size = cls_max_key_size(net_dev);
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	if (priv->dpni_attrs.options & DPNI_OPT_DIST_FS &&
	    priv->dpni_attrs.max_dist_key_size < key_size) {
		dev_err(&net_dev->dev,
			"max_dist_key_size = %d, expected %d.  Steering is disabled\n",
			priv->dpni_attrs.max_dist_key_size,
			key_size);
		priv->dpni_attrs.options &= ~DPNI_OPT_DIST_FS;
	}
}

/* Set RX hash options
 * flags is a combination of RXH_ bits
 */
int dpaa2_eth_set_hash(struct net_device *net_dev, u64 flags)
{
	struct device *dev = net_dev->dev.parent;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpkg_profile_cfg cls_cfg;
	struct dpni_rx_tc_dist_cfg dist_cfg;
	u8 *dma_mem;
	u64 enabled_flags = 0;
	int i;
	int err = 0;

	if (!dpaa2_eth_hash_enabled(priv)) {
		dev_err(dev, "Hashing support is not enabled\n");
		return -EOPNOTSUPP;
	}

	if (flags & ~DPAA2_RXH_SUPPORTED) {
		/* RXH_DISCARD is not supported */
		dev_err(dev, "unsupported option selected, supported options are: mvtsdfn\n");
		return -EOPNOTSUPP;
	}

	memset(&cls_cfg, 0, sizeof(cls_cfg));

	for (i = 0; i < ARRAY_SIZE(hash_fields); i++) {
		struct dpkg_extract *key =
			&cls_cfg.extracts[cls_cfg.num_extracts];

		if (!(flags & hash_fields[i].rxnfc_field))
			continue;

		if (cls_cfg.num_extracts >= DPKG_MAX_NUM_OF_EXTRACTS) {
			dev_err(dev, "error adding key extraction rule, too many rules?\n");
			return -E2BIG;
		}

		key->type = DPKG_EXTRACT_FROM_HDR;
		key->extract.from_hdr.prot = hash_fields[i].cls_prot;
		key->extract.from_hdr.type = DPKG_FULL_FIELD;
		key->extract.from_hdr.field = hash_fields[i].cls_field;
		cls_cfg.num_extracts++;

		enabled_flags |= hash_fields[i].rxnfc_field;
	}

	dma_mem = kzalloc(DPAA2_CLASSIFIER_DMA_SIZE, GFP_DMA | GFP_KERNEL);
	if (!dma_mem)
		return -ENOMEM;

	err = dpni_prepare_key_cfg(&cls_cfg, dma_mem);
	if (err) {
		dev_err(dev, "dpni_prepare_key_cfg error %d", err);
		return err;
	}

	memset(&dist_cfg, 0, sizeof(dist_cfg));

	/* Prepare for setting the rx dist */
	dist_cfg.key_cfg_iova = dma_map_single(net_dev->dev.parent, dma_mem,
					       DPAA2_CLASSIFIER_DMA_SIZE,
					       DMA_TO_DEVICE);
	if (dma_mapping_error(net_dev->dev.parent, dist_cfg.key_cfg_iova)) {
		dev_err(dev, "DMA mapping failed\n");
		kfree(dma_mem);
		return -ENOMEM;
	}

	dist_cfg.dist_size = dpaa2_eth_queue_count(priv);
	if (dpaa2_eth_fs_enabled(priv)) {
		dist_cfg.dist_mode = DPNI_DIST_MODE_FS;
		dist_cfg.fs_cfg.miss_action = DPNI_FS_MISS_HASH;
	} else {
		dist_cfg.dist_mode = DPNI_DIST_MODE_HASH;
	}

	err = dpni_set_rx_tc_dist(priv->mc_io, 0, priv->mc_token, 0, &dist_cfg);
	dma_unmap_single(net_dev->dev.parent, dist_cfg.key_cfg_iova,
			 DPAA2_CLASSIFIER_DMA_SIZE, DMA_TO_DEVICE);
	kfree(dma_mem);
	if (err) {
		dev_err(dev, "dpni_set_rx_tc_dist() error %d\n", err);
		return err;
	}

	priv->rx_hash_fields = enabled_flags;

	return 0;
}

static int prep_cls_rule(struct net_device *net_dev,
			 struct ethtool_rx_flow_spec *fs,
			 void *key)
{
	struct ethtool_tcpip4_spec *l4ip4_h, *l4ip4_m;
	struct ethhdr *eth_h, *eth_m;
	struct ethtool_flow_ext *ext_h, *ext_m;
	const u8 key_size = cls_key_size(net_dev);
	void *msk = key + key_size;

	memset(key, 0, key_size * 2);

	/* This code is a major mess, it has to be cleaned up after the
	 * classification mask issue is fixed and key format will be made static
	 */

	switch (fs->flow_type & 0xff) {
	case TCP_V4_FLOW:
		l4ip4_h = &fs->h_u.tcp_ip4_spec;
		l4ip4_m = &fs->m_u.tcp_ip4_spec;
		/* TODO: ethertype to match IPv4 and protocol to match TCP */
		goto l4ip4;

	case UDP_V4_FLOW:
		l4ip4_h = &fs->h_u.udp_ip4_spec;
		l4ip4_m = &fs->m_u.udp_ip4_spec;
		goto l4ip4;

	case SCTP_V4_FLOW:
		l4ip4_h = &fs->h_u.sctp_ip4_spec;
		l4ip4_m = &fs->m_u.sctp_ip4_spec;

l4ip4:
		if (l4ip4_m->tos) {
			netdev_err(net_dev,
				   "ToS is not supported for IPv4 L4\n");
			return -EOPNOTSUPP;
		}
		if (l4ip4_m->ip4src && !cls_is_enabled(net_dev, RXH_IP_SRC)) {
			netdev_err(net_dev, "IP SRC not supported!\n");
			return -EOPNOTSUPP;
		}
		if (l4ip4_m->ip4dst && !cls_is_enabled(net_dev, RXH_IP_DST)) {
			netdev_err(net_dev, "IP DST not supported!\n");
			return -EOPNOTSUPP;
		}
		if (l4ip4_m->psrc && !cls_is_enabled(net_dev, RXH_L4_B_0_1)) {
			netdev_err(net_dev, "PSRC not supported, ignored\n");
			return -EOPNOTSUPP;
		}
		if (l4ip4_m->pdst && !cls_is_enabled(net_dev, RXH_L4_B_2_3)) {
			netdev_err(net_dev, "PDST not supported, ignored\n");
			return -EOPNOTSUPP;
		}

		if (cls_is_enabled(net_dev, RXH_IP_SRC)) {
			*(u32 *)(key + cls_key_off(net_dev, RXH_IP_SRC))
				= l4ip4_h->ip4src;
			*(u32 *)(msk + cls_key_off(net_dev, RXH_IP_SRC))
				= l4ip4_m->ip4src;
		}
		if (cls_is_enabled(net_dev, RXH_IP_DST)) {
			*(u32 *)(key + cls_key_off(net_dev, RXH_IP_DST))
				= l4ip4_h->ip4dst;
			*(u32 *)(msk + cls_key_off(net_dev, RXH_IP_DST))
				= l4ip4_m->ip4dst;
		}

		if (cls_is_enabled(net_dev, RXH_L4_B_0_1)) {
			*(u32 *)(key + cls_key_off(net_dev, RXH_L4_B_0_1))
				= l4ip4_h->psrc;
			*(u32 *)(msk + cls_key_off(net_dev, RXH_L4_B_0_1))
				= l4ip4_m->psrc;
		}

		if (cls_is_enabled(net_dev, RXH_L4_B_2_3)) {
			*(u32 *)(key + cls_key_off(net_dev, RXH_L4_B_2_3))
				= l4ip4_h->pdst;
			*(u32 *)(msk + cls_key_off(net_dev, RXH_L4_B_2_3))
				= l4ip4_m->pdst;
		}
		break;

	case ETHER_FLOW:
		eth_h = &fs->h_u.ether_spec;
		eth_m = &fs->m_u.ether_spec;

		if (eth_m->h_proto) {
			netdev_err(net_dev, "Ethertype is not supported!\n");
			return -EOPNOTSUPP;
		}

		if (!is_zero_ether_addr(eth_m->h_source)) {
			netdev_err(net_dev, "ETH SRC is not supported!\n");
			return -EOPNOTSUPP;
		}

		if (cls_is_enabled(net_dev, RXH_L2DA)) {
			ether_addr_copy(key + cls_key_off(net_dev, RXH_L2DA),
					eth_h->h_dest);
			ether_addr_copy(msk + cls_key_off(net_dev, RXH_L2DA),
					eth_m->h_dest);
		} else {
			if (!is_zero_ether_addr(eth_m->h_dest)) {
				netdev_err(net_dev,
					   "ETH DST is not supported!\n");
				return -EOPNOTSUPP;
			}
		}
		break;

	default:
		/* TODO: IP user flow, AH, ESP */
		return -EOPNOTSUPP;
	}

	if (fs->flow_type & FLOW_EXT) {
		/* TODO: ETH data, VLAN ethertype, VLAN TCI .. */
		return -EOPNOTSUPP;
	}

	if (fs->flow_type & FLOW_MAC_EXT) {
		ext_h = &fs->h_ext;
		ext_m = &fs->m_ext;

		if (cls_is_enabled(net_dev, RXH_L2DA)) {
			ether_addr_copy(key + cls_key_off(net_dev, RXH_L2DA),
					ext_h->h_dest);
			ether_addr_copy(msk + cls_key_off(net_dev, RXH_L2DA),
					ext_m->h_dest);
		} else {
			if (!is_zero_ether_addr(ext_m->h_dest)) {
				netdev_err(net_dev,
					   "ETH DST is not supported!\n");
				return -EOPNOTSUPP;
			}
		}
	}
	return 0;
}

static int do_cls(struct net_device *net_dev,
		  struct ethtool_rx_flow_spec *fs,
		  bool add)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	const int rule_cnt = DPAA2_CLASSIFIER_ENTRY_COUNT;
	struct dpni_rule_cfg rule_cfg;
	void *dma_mem;
	int err = 0;

	if (!dpaa2_eth_fs_enabled(priv)) {
		netdev_err(net_dev, "dev does not support steering!\n");
		/* dev doesn't support steering */
		return -EOPNOTSUPP;
	}

	if ((fs->ring_cookie != RX_CLS_FLOW_DISC &&
	     fs->ring_cookie >= dpaa2_eth_queue_count(priv)) ||
	     fs->location >= rule_cnt)
		return -EINVAL;

	memset(&rule_cfg, 0, sizeof(rule_cfg));
	rule_cfg.key_size = cls_key_size(net_dev);

	/* allocate twice the key size, for the actual key and for mask */
	dma_mem =  kzalloc(rule_cfg.key_size * 2, GFP_DMA | GFP_KERNEL);
	if (!dma_mem)
		return -ENOMEM;

	err = prep_cls_rule(net_dev, fs, dma_mem);
	if (err)
		goto err_free_mem;

	rule_cfg.key_iova = dma_map_single(net_dev->dev.parent, dma_mem,
					   rule_cfg.key_size * 2,
					   DMA_TO_DEVICE);

	rule_cfg.mask_iova = rule_cfg.key_iova + rule_cfg.key_size;

	if (!(priv->dpni_attrs.options & DPNI_OPT_FS_MASK_SUPPORT)) {
		int i;
		u8 *mask = dma_mem + rule_cfg.key_size;

		/* check that nothing is masked out, otherwise it won't work */
		for (i = 0; i < rule_cfg.key_size; i++) {
			if (mask[i] == 0xff)
				continue;
			netdev_err(net_dev, "dev does not support masking!\n");
			err = -EOPNOTSUPP;
			goto err_free_mem;
		}
		rule_cfg.mask_iova = 0;
	}

	/* No way to control rule order in firmware */
	if (add)
		err = dpni_add_fs_entry(priv->mc_io, 0, priv->mc_token, 0,
					&rule_cfg, (u16)fs->ring_cookie);
	else
		err = dpni_remove_fs_entry(priv->mc_io, 0, priv->mc_token, 0,
					   &rule_cfg);

	dma_unmap_single(net_dev->dev.parent, rule_cfg.key_iova,
			 rule_cfg.key_size * 2, DMA_TO_DEVICE);
	if (err) {
		netdev_err(net_dev, "dpaa2_add_cls() error %d\n", err);
		goto err_free_mem;
	}

	priv->cls_rule[fs->location].fs = *fs;
	priv->cls_rule[fs->location].in_use = true;

err_free_mem:
	kfree(dma_mem);

	return err;
}

static int add_cls(struct net_device *net_dev,
		   struct ethtool_rx_flow_spec *fs)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int err;

	err = do_cls(net_dev, fs, true);
	if (err)
		return err;

	priv->cls_rule[fs->location].in_use = true;
	priv->cls_rule[fs->location].fs = *fs;

	return 0;
}

static int del_cls(struct net_device *net_dev, int location)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int err;

	err = do_cls(net_dev, &priv->cls_rule[location].fs, false);
	if (err)
		return err;

	priv->cls_rule[location].in_use = false;

	return 0;
}

static void clear_cls(struct net_device *net_dev)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int i, err;

	for (i = 0; i < DPAA2_CLASSIFIER_ENTRY_COUNT; i++) {
		if (!priv->cls_rule[i].in_use)
			continue;

		err = del_cls(net_dev, i);
		if (err)
			netdev_warn(net_dev,
				    "err trying to delete classification entry %d\n",
				    i);
	}
}

static int dpaa2_eth_set_rxnfc(struct net_device *net_dev,
			       struct ethtool_rxnfc *rxnfc)
{
	int err = 0;

	switch (rxnfc->cmd) {
	case ETHTOOL_SRXFH:
		/* first off clear ALL classification rules, chaging key
		 * composition will break them anyway
		 */
		clear_cls(net_dev);
		/* we purposely ignore cmd->flow_type for now, because the
		 * classifier only supports a single set of fields for all
		 * protocols
		 */
		err = dpaa2_eth_set_hash(net_dev, rxnfc->data);
		break;
	case ETHTOOL_SRXCLSRLINS:
		err = add_cls(net_dev, &rxnfc->fs);
		break;

	case ETHTOOL_SRXCLSRLDEL:
		err = del_cls(net_dev, rxnfc->fs.location);
		break;

	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

static int dpaa2_eth_get_rxnfc(struct net_device *net_dev,
			       struct ethtool_rxnfc *rxnfc, u32 *rule_locs)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	const int rule_cnt = DPAA2_CLASSIFIER_ENTRY_COUNT;
	int i, j;

	switch (rxnfc->cmd) {
	case ETHTOOL_GRXFH:
		/* we purposely ignore cmd->flow_type for now, because the
		 * classifier only supports a single set of fields for all
		 * protocols
		 */
		rxnfc->data = priv->rx_hash_fields;
		break;

	case ETHTOOL_GRXRINGS:
		rxnfc->data = dpaa2_eth_queue_count(priv);
		break;

	case ETHTOOL_GRXCLSRLCNT:
		for (i = 0, rxnfc->rule_cnt = 0; i < rule_cnt; i++)
			if (priv->cls_rule[i].in_use)
				rxnfc->rule_cnt++;
		rxnfc->data = rule_cnt;
		break;

	case ETHTOOL_GRXCLSRULE:
		if (!priv->cls_rule[rxnfc->fs.location].in_use)
			return -EINVAL;

		rxnfc->fs = priv->cls_rule[rxnfc->fs.location].fs;
		break;

	case ETHTOOL_GRXCLSRLALL:
		for (i = 0, j = 0; i < rule_cnt; i++) {
			if (!priv->cls_rule[i].in_use)
				continue;
			if (j == rxnfc->rule_cnt)
				return -EMSGSIZE;
			rule_locs[j++] = i;
		}
		rxnfc->rule_cnt = j;
		rxnfc->data = rule_cnt;
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

const struct ethtool_ops dpaa2_ethtool_ops = {
	.get_drvinfo = dpaa2_eth_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_settings = dpaa2_eth_get_settings,
	.set_settings = dpaa2_eth_set_settings,
	.get_sset_count = dpaa2_eth_get_sset_count,
	.get_ethtool_stats = dpaa2_eth_get_ethtool_stats,
	.get_strings = dpaa2_eth_get_strings,
	.get_rxnfc = dpaa2_eth_get_rxnfc,
	.set_rxnfc = dpaa2_eth_set_rxnfc,
};
