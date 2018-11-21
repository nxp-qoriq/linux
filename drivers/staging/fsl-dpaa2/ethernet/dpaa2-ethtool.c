// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2016-2017 NXP
 */

#include <linux/net_tstamp.h>

#include "dpni.h"	/* DPNI_LINK_OPT_* */
#include "dpaa2-eth.h"

/* To be kept in sync with DPNI statistics */
static char dpaa2_ethtool_stats[][ETH_GSTRING_LEN] = {
	"[hw] rx frames",
	"[hw] rx bytes",
	"[hw] rx mcast frames",
	"[hw] rx mcast bytes",
	"[hw] rx bcast frames",
	"[hw] rx bcast bytes",
	"[hw] tx frames",
	"[hw] tx bytes",
	"[hw] tx mcast frames",
	"[hw] tx mcast bytes",
	"[hw] tx bcast frames",
	"[hw] tx bcast bytes",
	"[hw] rx filtered frames",
	"[hw] rx discarded frames",
	"[hw] rx nobuffer discards",
	"[hw] tx discarded frames",
	"[hw] tx confirmed frames",
};

#define DPAA2_ETH_NUM_STATS	ARRAY_SIZE(dpaa2_ethtool_stats)

static char dpaa2_ethtool_extras[][ETH_GSTRING_LEN] = {
	/* per-cpu stats */
	"[drv] tx conf frames",
	"[drv] tx conf bytes",
	"[drv] tx sg frames",
	"[drv] tx sg bytes",
	"[drv] tx realloc frames",
	"[drv] rx sg frames",
	"[drv] rx sg bytes",
	"[drv] enqueue portal busy",
	/* Channel stats */
	"[drv] dequeue portal busy",
	"[drv] channel pull errors",
	"[drv] cdan",
	"[drv] tx congestion state",
	/* FQ stats */
	"rx pending frames",
	"rx pending bytes",
	"tx conf pending frames",
	"tx conf pending bytes",
	"buffer count"
};

#define DPAA2_ETH_NUM_EXTRA_STATS	ARRAY_SIZE(dpaa2_ethtool_extras)

static void dpaa2_eth_get_drvinfo(struct net_device *net_dev,
				  struct ethtool_drvinfo *drvinfo)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));

	snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
		 "%u.%u", priv->dpni_ver_major, priv->dpni_ver_minor);

	strlcpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int
dpaa2_eth_get_link_ksettings(struct net_device *net_dev,
			     struct ethtool_link_ksettings *link_settings)
{
	struct dpni_link_state state = {0};
	int err = 0;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (err) {
		netdev_err(net_dev, "ERROR %d getting link state\n", err);
		goto out;
	}

	/* At the moment, we have no way of interrogating the DPMAC
	 * from the DPNI side - and for that matter there may exist
	 * no DPMAC at all. So for now we just don't report anything
	 * beyond the DPNI attributes.
	 */
	if (state.options & DPNI_LINK_OPT_AUTONEG)
		link_settings->base.autoneg = AUTONEG_ENABLE;
	if (!(state.options & DPNI_LINK_OPT_HALF_DUPLEX))
		link_settings->base.duplex = DUPLEX_FULL;
	link_settings->base.speed = state.rate;

out:
	return err;
}

#define DPNI_DYNAMIC_LINK_SET_VER_MAJOR		7
#define DPNI_DYNAMIC_LINK_SET_VER_MINOR		1
static int
dpaa2_eth_set_link_ksettings(struct net_device *net_dev,
			     const struct ethtool_link_ksettings *link_settings)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpni_link_state state = {0};
	struct dpni_link_cfg cfg = {0};
	int err = 0;

	/* If using an older MC version, the DPNI must be down
 	 * in order to be able to change link settings. Taking steps to let
 	 * the user know that.
 	 */
	if (dpaa2_eth_cmp_dpni_ver(priv, DPNI_DYNAMIC_LINK_SET_VER_MAJOR,
				   DPNI_DYNAMIC_LINK_SET_VER_MINOR) < 0) {
		if (netif_running(net_dev)) {
			netdev_info(net_dev, "Interface must be brought down first.\n");
			return -EACCES;
		}
	}

	/* Need to interrogate link state to get flow control params */
	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (err) {
		netdev_err(net_dev, "Error getting link state\n");
		goto out;
	}

	cfg.options = state.options;
	cfg.rate = link_settings->base.speed;
	if (link_settings->base.autoneg == AUTONEG_ENABLE)
		cfg.options |= DPNI_LINK_OPT_AUTONEG;
	else
		cfg.options &= ~DPNI_LINK_OPT_AUTONEG;
	if (link_settings->base.duplex  == DUPLEX_HALF)
		cfg.options |= DPNI_LINK_OPT_HALF_DUPLEX;
	else
		cfg.options &= ~DPNI_LINK_OPT_HALF_DUPLEX;

	err = dpni_set_link_cfg(priv->mc_io, 0, priv->mc_token, &cfg);
	if (err)
		/* ethtool will be loud enough if we return an error; no point
		 * in putting our own error message on the console by default
		 */
		netdev_dbg(net_dev, "ERROR %d setting link cfg\n", err);

out:
	return err;
}

static void dpaa2_eth_get_pauseparam(struct net_device *net_dev,
				     struct ethtool_pauseparam *pause)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpni_link_state state = {0};
	int err;

	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (err)
		netdev_dbg(net_dev, "Error getting link state\n");

	/* Report general port autonegotiation status */
	pause->autoneg = !!(state.options & DPNI_LINK_OPT_AUTONEG);
	pause->rx_pause = !!(state.options & DPNI_LINK_OPT_PAUSE);
	pause->tx_pause = pause->rx_pause ^
			  !!(state.options & DPNI_LINK_OPT_ASYM_PAUSE);
}

static int dpaa2_eth_set_pauseparam(struct net_device *net_dev,
				    struct ethtool_pauseparam *pause)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpni_link_state state = {0};
	struct dpni_link_cfg cfg = {0};
	u32 current_tx_pause;
	int err = 0;

	err = dpni_get_link_state(priv->mc_io, 0, priv->mc_token, &state);
	if (err) {
		netdev_dbg(net_dev, "Error getting link state\n");
		goto out;
	}

	cfg.rate = state.rate;
	cfg.options = state.options;
	current_tx_pause = !!(cfg.options & DPNI_LINK_OPT_PAUSE) ^
			   !!(cfg.options & DPNI_LINK_OPT_ASYM_PAUSE);

	/* We don't support changing pause frame autonegotiation separately
	 * from general port autoneg
	 */
	if (pause->autoneg != !!(state.options & DPNI_LINK_OPT_AUTONEG))
		netdev_warn(net_dev,
			    "Cannot change pause frame autoneg separately\n");

	if (pause->rx_pause)
		cfg.options |= DPNI_LINK_OPT_PAUSE;
	else
		cfg.options &= ~DPNI_LINK_OPT_PAUSE;

	if (pause->rx_pause ^ pause->tx_pause)
		cfg.options |= DPNI_LINK_OPT_ASYM_PAUSE;
	else
		cfg.options &= ~DPNI_LINK_OPT_ASYM_PAUSE;

	err = dpni_set_link_cfg(priv->mc_io, 0, priv->mc_token, &cfg);
	if (err) {
		netdev_dbg(net_dev, "Error setting link\n");
		goto out;
	}

	/* Enable/disable Rx FQ taildrop if Tx pause frames have changed */
	if (current_tx_pause == pause->tx_pause)
		goto out;

	priv->tx_pause_frames = pause->tx_pause;
	err = set_rx_taildrop(priv);
	if (err)
		netdev_dbg(net_dev, "Error configuring taildrop\n");

out:
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

/** Fill in hardware counters, as returned by MC.
 */
static void dpaa2_eth_get_ethtool_stats(struct net_device *net_dev,
					struct ethtool_stats *stats,
					u64 *data)
{
	int i = 0;
	int j, k, err;
	int num_cnt;
	union dpni_statistics dpni_stats;
	u32 fcnt, bcnt;
	u32 fcnt_rx_total = 0, fcnt_tx_total = 0;
	u32 bcnt_rx_total = 0, bcnt_tx_total = 0;
	u32 buf_cnt;
	u64 cdan = 0;
	u64 portal_busy = 0, pull_err = 0;
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpaa2_eth_drv_stats *extras;
	struct dpaa2_eth_ch_stats *ch_stats;

	memset(data, 0,
	       sizeof(u64) * (DPAA2_ETH_NUM_STATS + DPAA2_ETH_NUM_EXTRA_STATS));

	/* Print standard counters, from DPNI statistics */
	for (j = 0; j <= 2; j++) {
		err = dpni_get_statistics(priv->mc_io, 0, priv->mc_token,
					  j, 0, &dpni_stats);
		if (err != 0)
			netdev_warn(net_dev, "dpni_get_stats(%d) failed\n", j);
		switch (j) {
		case 0:
			num_cnt = sizeof(dpni_stats.page_0) / sizeof(u64);
			break;
		case 1:
			num_cnt = sizeof(dpni_stats.page_1) / sizeof(u64);
			break;
		case 2:
			num_cnt = sizeof(dpni_stats.page_2) / sizeof(u64);
			break;
		}
		for (k = 0; k < num_cnt; k++)
			*(data + i++) = dpni_stats.raw.counter[k];
	}

	/* Print per-cpu extra stats */
	for_each_online_cpu(k) {
		extras = per_cpu_ptr(priv->percpu_extras, k);
		for (j = 0; j < sizeof(*extras) / sizeof(__u64); j++)
			*((__u64 *)data + i + j) += *((__u64 *)extras + j);
	}
	i += j;

	for (j = 0; j < priv->num_channels; j++) {
		ch_stats = &priv->channel[j]->stats;
		cdan += ch_stats->cdan;
		portal_busy += ch_stats->dequeue_portal_busy;
		pull_err += ch_stats->pull_err;
	}

	*(data + i++) = portal_busy;
	*(data + i++) = pull_err;
	*(data + i++) = cdan;

	*(data + i++) = dpaa2_cscn_state_congested(priv->cscn_mem);

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

	err = dpaa2_io_query_bp_count(NULL, priv->bpid, &buf_cnt);
	if (err) {
		netdev_warn(net_dev, "Buffer count query error %d\n", err);
		return;
	}
	*(data + i++) = buf_cnt;
}

static int prep_eth_rule(struct ethhdr *eth_value, struct ethhdr *eth_mask,
			 void *key, void *mask)
{
	int off;

	if (eth_mask->h_proto) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_TYPE);
		*(__be16 *)(key + off) = eth_value->h_proto;
		*(__be16 *)(mask + off) = eth_mask->h_proto;
	}

	if (!is_zero_ether_addr(eth_mask->h_source)) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_SA);
		ether_addr_copy(key + off, eth_value->h_source);
		ether_addr_copy(mask + off, eth_mask->h_source);
	}

	if (!is_zero_ether_addr(eth_mask->h_dest)) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_DA);
		ether_addr_copy(key + off, eth_value->h_dest);
		ether_addr_copy(mask + off, eth_mask->h_dest);
	}

	return 0;
}

static int prep_user_ip_rule(struct ethtool_usrip4_spec *uip_value,
			     struct ethtool_usrip4_spec *uip_mask,
			     void *key, void *mask)
{
	int off;
	u32 tmp_value, tmp_mask;

	if (uip_mask->tos || uip_mask->ip_ver)
		return -EOPNOTSUPP;

	if (uip_mask->ip4src) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_SRC);
		*(__be32 *)(key + off) = uip_value->ip4src;
		*(__be32 *)(mask + off) = uip_mask->ip4src;
	}

	if (uip_mask->ip4dst) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_DST);
		*(__be32 *)(key + off) = uip_value->ip4dst;
		*(__be32 *)(mask + off) = uip_mask->ip4dst;
	}

	if (uip_mask->proto) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_PROTO);
		*(u8 *)(key + off) = uip_value->proto;
		*(u8 *)(mask + off) = uip_mask->proto;
	}

	if (uip_mask->l4_4_bytes) {
		tmp_value = be32_to_cpu(uip_value->l4_4_bytes);
		tmp_mask = be32_to_cpu(uip_mask->l4_4_bytes);

		off = dpaa2_eth_cls_fld_off(NET_PROT_UDP, NH_FLD_UDP_PORT_SRC);
		*(__be16 *)(key + off) = htons(tmp_value >> 16);
		*(__be16 *)(mask + off) = htons(tmp_mask >> 16);

		off = dpaa2_eth_cls_fld_off(NET_PROT_UDP, NH_FLD_UDP_PORT_DST);
		*(__be16 *)(key + off) = htons(tmp_value & 0xFFFF);
		*(__be16 *)(mask + off) = htons(tmp_mask & 0xFFFF);
	}

	/* Only apply the rule for IPv4 frames */
	off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_TYPE);
	*(__be16 *)(key + off) = htons(ETH_P_IP);
	*(__be16 *)(mask + off) = htons(0xFFFF);

	return 0;
}

static int prep_l4_rule(struct ethtool_tcpip4_spec *l4_value,
			struct ethtool_tcpip4_spec *l4_mask,
			void *key, void *mask, u8 l4_proto)
{
	int off;

	if (l4_mask->tos)
		return -EOPNOTSUPP;
	if (l4_mask->ip4src) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_SRC);
		*(__be32 *)(key + off) = l4_value->ip4src;
		*(__be32 *)(mask + off) = l4_mask->ip4src;
	}

	if (l4_mask->ip4dst) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_DST);
		*(__be32 *)(key + off) = l4_value->ip4dst;
		*(__be32 *)(mask + off) = l4_mask->ip4dst;
	}

	if (l4_mask->psrc) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_UDP, NH_FLD_UDP_PORT_SRC);
		*(__be16 *)(key + off) = l4_value->psrc;
		*(__be16 *)(mask + off) = l4_mask->psrc;
	}

	if (l4_mask->pdst) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_UDP, NH_FLD_UDP_PORT_DST);
		*(__be16 *)(key + off) = l4_value->pdst;
		*(__be16 *)(mask + off) = l4_mask->pdst;
	}

	/* Only apply the rule for the user-specified L4 protocol
	 * and if ethertype matches IPv4
	 */
	off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_TYPE);
	*(__be16 *)(key + off) = htons(ETH_P_IP);
	*(__be16 *)(mask + off) = htons(0xFFFF);

	off = dpaa2_eth_cls_fld_off(NET_PROT_IP, NH_FLD_IP_PROTO);
	*(u8 *)(key + off) = l4_proto;
	*(u8 *)(mask + off) = 0xFF;

	return 0;
}

static int prep_ext_rule(struct ethtool_flow_ext *ext_value,
			 struct ethtool_flow_ext *ext_mask,
			 void *key, void *mask)
{
	int off;

	if (ext_mask->vlan_etype)
		return -EOPNOTSUPP;

	if (ext_mask->vlan_tci) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_VLAN, NH_FLD_VLAN_TCI);
		*(__be16 *)(key + off) = ext_value->vlan_tci;
		*(__be16 *)(mask + off) = ext_mask->vlan_tci;
	}

	return 0;
}

static int prep_mac_ext_rule(struct ethtool_flow_ext *ext_value,
			     struct ethtool_flow_ext *ext_mask,
			     void *key, void *mask)
{
	int off;

	if (!is_zero_ether_addr(ext_mask->h_dest)) {
		off = dpaa2_eth_cls_fld_off(NET_PROT_ETH, NH_FLD_ETH_DA);
		ether_addr_copy(key + off, ext_value->h_dest);
		ether_addr_copy(mask + off, ext_mask->h_dest);
	}

	return 0;
}

static int prep_cls_rule(struct ethtool_rx_flow_spec *fs, void *key, void *mask)
{
	int err;

	switch (fs->flow_type & 0xFF) {
	case ETHER_FLOW:
		err = prep_eth_rule(&fs->h_u.ether_spec, &fs->m_u.ether_spec,
				    key, mask);
		break;
	case IP_USER_FLOW:
		err = prep_user_ip_rule(&fs->h_u.usr_ip4_spec,
				    &fs->m_u.usr_ip4_spec, key, mask);
		break;
	case TCP_V4_FLOW:
		err = prep_l4_rule(&fs->h_u.tcp_ip4_spec, &fs->m_u.tcp_ip4_spec,
				   key, mask, IPPROTO_TCP);
		break;
	case UDP_V4_FLOW:
		err = prep_l4_rule(&fs->h_u.udp_ip4_spec, &fs->m_u.udp_ip4_spec,
				   key, mask, IPPROTO_UDP);
		break;
	case SCTP_V4_FLOW:
		err = prep_l4_rule(&fs->h_u.sctp_ip4_spec, &fs->m_u.sctp_ip4_spec,
				   key, mask, IPPROTO_SCTP);
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (err)
		return err;

	if (fs->flow_type & FLOW_EXT) {
		err = prep_ext_rule(&fs->h_ext, &fs->m_ext, key, mask);
		if (err)
			return err;
	}

	if (fs->flow_type & FLOW_MAC_EXT) {
		err = prep_mac_ext_rule(&fs->h_ext, &fs->m_ext, key, mask);
		if (err)
			return err;
	}

	return 0;
}

static int do_cls_rule(struct net_device *net_dev,
		       struct ethtool_rx_flow_spec *fs,
		       bool add)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;
	struct dpni_rule_cfg rule_cfg = { 0 };
	struct dpni_fs_action_cfg fs_act = { 0 };
	dma_addr_t key_iova;
	void *key_buf;
	int i, err = 0;

	if (fs->ring_cookie != RX_CLS_FLOW_DISC &&
	    fs->ring_cookie >= dpaa2_eth_queue_count(priv))
		return -EINVAL;

	rule_cfg.key_size = dpaa2_eth_cls_key_size();

	/* allocate twice the key size, for the actual key and for mask */
	key_buf = kzalloc(rule_cfg.key_size * 2, GFP_KERNEL);
	if (!key_buf)
		return -ENOMEM;

	/* Fill the key and mask memory areas */
	err = prep_cls_rule(fs, key_buf, key_buf + rule_cfg.key_size);
	if (err)
		goto free_mem;

	key_iova = dma_map_single(dev, key_buf, rule_cfg.key_size * 2,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, key_iova)) {
		err = -ENOMEM;
		goto free_mem;
	}

	rule_cfg.key_iova = key_iova;
	rule_cfg.mask_iova = key_iova + rule_cfg.key_size;

	if (add) {
		if (fs->ring_cookie == RX_CLS_FLOW_DISC)
			fs_act.options |= DPNI_FS_OPT_DISCARD;
		else
			fs_act.flow_id = fs->ring_cookie;
	}
	for (i = 0; i < dpaa2_eth_tc_count(priv); i++) {
		if (add)
			err = dpni_add_fs_entry(priv->mc_io, 0, priv->mc_token,
						i, fs->location, &rule_cfg,
						&fs_act);
		else
			err = dpni_remove_fs_entry(priv->mc_io, 0,
						   priv->mc_token, i,
						   &rule_cfg);
		if (err)
			break;
	}

	dma_unmap_single(dev, key_iova, rule_cfg.key_size * 2, DMA_TO_DEVICE);

free_mem:
	kfree(key_buf);

	return err;
}

static int update_cls_rule(struct net_device *net_dev,
			   struct ethtool_rx_flow_spec *new_fs,
			   int location)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct dpaa2_eth_cls_rule *rule;
	int err = -EINVAL;

	if (!priv->rx_cls_enabled)
		return -EOPNOTSUPP;

	if (location >= dpaa2_eth_fs_count(priv))
		return -EINVAL;

	rule = &priv->cls_rule[location];

	/* If a rule is present at the specified location, delete it. */
	if (rule->in_use) {
		err = do_cls_rule(net_dev, &rule->fs, false);
		if (err)
			return err;

		rule->in_use = 0;
	}

	/* If no new entry to add, return here */
	if (!new_fs)
		return err;

	err = do_cls_rule(net_dev, new_fs, true);
	if (err)
		return err;

	rule->in_use = 1;
	rule->fs = *new_fs;

	return 0;
}

static int dpaa2_eth_get_rxnfc(struct net_device *net_dev,
			       struct ethtool_rxnfc *rxnfc, u32 *rule_locs)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	int rule_cnt = dpaa2_eth_fs_count(priv);
	int i, j = 0;

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
		rxnfc->rule_cnt = 0;
		for (i = 0; i < rule_cnt; i++)
			if (priv->cls_rule[i].in_use)
				rxnfc->rule_cnt++;
		rxnfc->data = rule_cnt;
		break;
	case ETHTOOL_GRXCLSRULE:
		if (rxnfc->fs.location >= rule_cnt)
			return -EINVAL;
		if (!priv->cls_rule[rxnfc->fs.location].in_use)
			return -EINVAL;
		rxnfc->fs = priv->cls_rule[rxnfc->fs.location].fs;
		break;
	case ETHTOOL_GRXCLSRLALL:
		for (i = 0; i < rule_cnt; i++) {
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

int dpaa2_phc_index = -1;
EXPORT_SYMBOL(dpaa2_phc_index);

static int dpaa2_eth_get_ts_info(struct net_device *dev,
				 struct ethtool_ts_info *info)
{
	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = dpaa2_phc_index;

	info->tx_types = (1 << HWTSTAMP_TX_OFF) |
			 (1 << HWTSTAMP_TX_ON);

	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);
	return 0;
}

static int dpaa2_eth_set_rxnfc(struct net_device *net_dev,
			       struct ethtool_rxnfc *rxnfc)
{
	int err = 0;

	switch (rxnfc->cmd) {
	case ETHTOOL_SRXFH:
		if ((rxnfc->data & DPAA2_RXH_SUPPORTED) != rxnfc->data)
			return -EOPNOTSUPP;
		err = dpaa2_eth_set_hash(net_dev, rxnfc->data);
		break;
	case ETHTOOL_SRXCLSRLINS:
		err = update_cls_rule(net_dev, &rxnfc->fs, rxnfc->fs.location);
		break;
	case ETHTOOL_SRXCLSRLDEL:
		err = update_cls_rule(net_dev, NULL, rxnfc->fs.location);
		break;
	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

const struct ethtool_ops dpaa2_ethtool_ops = {
	.get_drvinfo = dpaa2_eth_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = dpaa2_eth_get_link_ksettings,
	.set_link_ksettings = dpaa2_eth_set_link_ksettings,
	.get_pauseparam = dpaa2_eth_get_pauseparam,
	.set_pauseparam = dpaa2_eth_set_pauseparam,
	.get_sset_count = dpaa2_eth_get_sset_count,
	.get_ethtool_stats = dpaa2_eth_get_ethtool_stats,
	.get_strings = dpaa2_eth_get_strings,
	.get_rxnfc = dpaa2_eth_get_rxnfc,
	.set_rxnfc = dpaa2_eth_set_rxnfc,
	.get_ts_info = dpaa2_eth_get_ts_info,
};
