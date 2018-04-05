/* Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2016-2017 NXP
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
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, dpaa2_eth_drv_version,
		sizeof(drvinfo->version));

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

	err = dpaa2_io_query_bp_count(NULL, priv->bpid, &buf_cnt);
	if (err) {
		netdev_warn(net_dev, "Buffer count query error %d\n", err);
		return;
	}
	*(data + i++) = buf_cnt;
#endif
}

static int cls_key_off(struct dpaa2_eth_priv *priv, int prot, int field)
{
	int i, off = 0;

	for (i = 0; i < priv->num_dist_fields; i++) {
		if (priv->dist_fields[i].cls_prot == prot &&
		    priv->dist_fields[i].cls_field == field)
			return off;
		off += priv->dist_fields[i].size;
	}

	return -1;
}

static u8 cls_key_size(struct dpaa2_eth_priv *priv)
{
	u8 i, size = 0;

	for (i = 0; i < priv->num_dist_fields; i++)
		size += priv->dist_fields[i].size;

	return size;
}

void check_cls_support(struct dpaa2_eth_priv *priv)
{
	u8 key_size = cls_key_size(priv);
	struct device *dev = priv->net_dev->dev.parent;

	if (dpaa2_eth_hash_enabled(priv)) {
		if (priv->dpni_attrs.fs_key_size < key_size) {
			dev_info(dev, "max_dist_key_size = %d, expected %d. Hashing and steering are disabled\n",
				 priv->dpni_attrs.fs_key_size,
				 key_size);
			goto disable_fs;
		}
		if (priv->num_dist_fields > DPKG_MAX_NUM_OF_EXTRACTS) {
			dev_info(dev, "Too many key fields (max = %d). Hashing and steering are disabled\n",
				 DPKG_MAX_NUM_OF_EXTRACTS);
			goto disable_fs;
		}
	}

	if (dpaa2_eth_fs_enabled(priv)) {
		if (!dpaa2_eth_hash_enabled(priv)) {
			dev_info(dev, "Insufficient queues. Steering is disabled\n");
			goto disable_fs;
		}

		if (!dpaa2_eth_fs_mask_enabled(priv)) {
			dev_info(dev, "Key masks not supported. Steering is disabled\n");
			goto disable_fs;
		}
	}

	return;

disable_fs:
	priv->dpni_attrs.options |= DPNI_OPT_NO_FS;
	priv->dpni_attrs.options &= ~DPNI_OPT_HAS_KEY_MASKING;
}

static int prep_l4_rule(struct dpaa2_eth_priv *priv,
			struct ethtool_tcpip4_spec *l4_value,
			struct ethtool_tcpip4_spec *l4_mask,
			void *key, void *mask, u8 l4_proto)
{
	int offset;

	if (l4_mask->tos) {
		netdev_err(priv->net_dev, "ToS is not supported for IPv4 L4\n");
		return -EOPNOTSUPP;
	}

	if (l4_mask->ip4src) {
		offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_SRC);
		*(u32 *)(key + offset) = l4_value->ip4src;
		*(u32 *)(mask + offset) = l4_mask->ip4src;
	}

	if (l4_mask->ip4dst) {
		offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_DST);
		*(u32 *)(key + offset) = l4_value->ip4dst;
		*(u32 *)(mask + offset) = l4_mask->ip4dst;
	}

	if (l4_mask->psrc) {
		offset = cls_key_off(priv, NET_PROT_UDP, NH_FLD_UDP_PORT_SRC);
		*(u32 *)(key + offset) = l4_value->psrc;
		*(u32 *)(mask + offset) = l4_mask->psrc;
	}

	if (l4_mask->pdst) {
		offset = cls_key_off(priv, NET_PROT_UDP, NH_FLD_UDP_PORT_DST);
		*(u32 *)(key + offset) = l4_value->pdst;
		*(u32 *)(mask + offset) = l4_mask->pdst;
	}

	/* Only apply the rule for the user-specified L4 protocol
	 * and if ethertype matches IPv4
	 */
	offset = cls_key_off(priv, NET_PROT_ETH, NH_FLD_ETH_TYPE);
	*(u16 *)(key + offset) = htons(ETH_P_IP);
	*(u16 *)(mask + offset) = 0xFFFF;

	offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_PROTO);
	*(u8 *)(key + offset) = l4_proto;
	*(u8 *)(mask + offset) = 0xFF;

	/* TODO: check IP version */

	return 0;
}

static int prep_eth_rule(struct dpaa2_eth_priv *priv,
			 struct ethhdr *eth_value, struct ethhdr *eth_mask,
			 void *key, void *mask)
{
	int offset;

	if (eth_mask->h_proto) {
		netdev_err(priv->net_dev, "Ethertype is not supported!\n");
		return -EOPNOTSUPP;
	}

	if (!is_zero_ether_addr(eth_mask->h_source)) {
		offset = cls_key_off(priv, NET_PROT_ETH, NH_FLD_ETH_SA);
		ether_addr_copy(key + offset, eth_value->h_source);
		ether_addr_copy(mask + offset, eth_mask->h_source);
	}

	if (!is_zero_ether_addr(eth_mask->h_dest)) {
		offset = cls_key_off(priv, NET_PROT_ETH, NH_FLD_ETH_DA);
		ether_addr_copy(key + offset, eth_value->h_dest);
		ether_addr_copy(mask + offset, eth_mask->h_dest);
	}

	return 0;
}

static int prep_user_ip_rule(struct dpaa2_eth_priv *priv,
			     struct ethtool_usrip4_spec *uip_value,
			     struct ethtool_usrip4_spec *uip_mask,
			     void *key, void *mask)
{
	int offset;

	if (uip_mask->tos)
		return -EOPNOTSUPP;

	if (uip_mask->ip4src) {
		offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_SRC);
		*(u32 *)(key + offset) = uip_value->ip4src;
		*(u32 *)(mask + offset) = uip_mask->ip4src;
	}

	if (uip_mask->ip4dst) {
		offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_DST);
		*(u32 *)(key + offset) = uip_value->ip4dst;
		*(u32 *)(mask + offset) = uip_mask->ip4dst;
	}

	if (uip_mask->proto) {
		offset = cls_key_off(priv, NET_PROT_IP, NH_FLD_IP_PROTO);
		*(u32 *)(key + offset) = uip_value->proto;
		*(u32 *)(mask + offset) = uip_mask->proto;
	}
	if (uip_mask->l4_4_bytes) {
		offset = cls_key_off(priv, NET_PROT_UDP, NH_FLD_UDP_PORT_SRC);
		*(u16 *)(key + offset) = uip_value->l4_4_bytes << 16;
		*(u16 *)(mask + offset) = uip_mask->l4_4_bytes << 16;

		offset = cls_key_off(priv, NET_PROT_UDP, NH_FLD_UDP_PORT_DST);
		*(u16 *)(key + offset) = uip_value->l4_4_bytes & 0xFFFF;
		*(u16 *)(mask + offset) = uip_mask->l4_4_bytes & 0xFFFF;
	}

	/* Ethertype must be IP */
	offset = cls_key_off(priv, NET_PROT_ETH, NH_FLD_ETH_TYPE);
	*(u16 *)(key + offset) = htons(ETH_P_IP);
	*(u16 *)(mask + offset) = 0xFFFF;

	return 0;
}

static int prep_ext_rule(struct dpaa2_eth_priv *priv,
			 struct ethtool_flow_ext *ext_value,
			 struct ethtool_flow_ext *ext_mask,
			 void *key, void *mask)
{
	int offset;

	if (ext_mask->vlan_etype)
		return -EOPNOTSUPP;

	if (ext_mask->vlan_tci) {
		offset = cls_key_off(priv, NET_PROT_VLAN, NH_FLD_VLAN_TCI);
		*(u16 *)(key + offset) = ext_value->vlan_tci;
		*(u16 *)(mask + offset) = ext_mask->vlan_tci;
	}

	return 0;
}

static int prep_mac_ext_rule(struct dpaa2_eth_priv *priv,
			     struct ethtool_flow_ext *ext_value,
			     struct ethtool_flow_ext *ext_mask,
			     void *key, void *mask)
{
	int offset;

	if (!is_zero_ether_addr(ext_mask->h_dest)) {
		offset = cls_key_off(priv, NET_PROT_ETH, NH_FLD_ETH_DA);
		ether_addr_copy(key + offset, ext_value->h_dest);
		ether_addr_copy(mask + offset, ext_mask->h_dest);
	}

	return 0;
}

static int prep_cls_rule(struct net_device *net_dev,
			 struct ethtool_rx_flow_spec *fs,
			 void *key)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	const u8 key_size = cls_key_size(priv);
	void *msk = key + key_size;
	int err;

	memset(key, 0, key_size * 2);

	switch (fs->flow_type & 0xff) {
	case TCP_V4_FLOW:
		err = prep_l4_rule(priv, &fs->h_u.tcp_ip4_spec,
				   &fs->m_u.tcp_ip4_spec, key, msk,
				   IPPROTO_TCP);
		break;
	case UDP_V4_FLOW:
		err = prep_l4_rule(priv, &fs->h_u.udp_ip4_spec,
				   &fs->m_u.udp_ip4_spec, key, msk,
				   IPPROTO_UDP);
		break;
	case SCTP_V4_FLOW:
		err = prep_l4_rule(priv, &fs->h_u.sctp_ip4_spec,
				   &fs->m_u.sctp_ip4_spec, key, msk,
				   IPPROTO_SCTP);
		break;
	case ETHER_FLOW:
		err = prep_eth_rule(priv, &fs->h_u.ether_spec,
				    &fs->m_u.ether_spec, key, msk);
		break;
	case IP_USER_FLOW:
		err = prep_user_ip_rule(priv, &fs->h_u.usr_ip4_spec,
					&fs->m_u.usr_ip4_spec, key, msk);
		break;
	default:
		/* TODO: AH, ESP */
		return -EOPNOTSUPP;
	}
	if (err)
		return err;

	if (fs->flow_type & FLOW_EXT) {
		err = prep_ext_rule(priv, &fs->h_ext, &fs->m_ext, key, msk);
		if (err)
			return err;
	}

	if (fs->flow_type & FLOW_MAC_EXT) {
		err = prep_mac_ext_rule(priv, &fs->h_ext, &fs->m_ext, key, msk);
		if (err)
			return err;
	}

	return 0;
}

static int del_cls(struct net_device *net_dev, int location);

static int do_cls(struct net_device *net_dev,
		  struct ethtool_rx_flow_spec *fs,
		  bool add)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	struct device *dev = net_dev->dev.parent;
	const int rule_cnt = dpaa2_eth_fs_count(priv);
	struct dpni_rule_cfg rule_cfg;
	struct dpni_fs_action_cfg fs_act = { 0 };
	void *dma_mem;
	int err = 0, tc;

	if (!dpaa2_eth_fs_enabled(priv)) {
		netdev_err(net_dev, "dev does not support steering!\n");
		/* dev doesn't support steering */
		return -EOPNOTSUPP;
	}

	if ((fs->ring_cookie != RX_CLS_FLOW_DISC &&
	     fs->ring_cookie >= dpaa2_eth_queue_count(priv)) ||
	     fs->location >= rule_cnt)
		return -EINVAL;

	/* When adding a new rule, check if location if available
	 * and if not, free the existing table entry before inserting
	 * the new one
	 */
	if (add && (priv->cls_rule[fs->location].in_use == true))
		del_cls(net_dev, fs->location);

	memset(&rule_cfg, 0, sizeof(rule_cfg));
	rule_cfg.key_size = cls_key_size(priv);

	/* allocate twice the key size, for the actual key and for mask */
	dma_mem = kzalloc(rule_cfg.key_size * 2, GFP_DMA | GFP_KERNEL);
	if (!dma_mem)
		return -ENOMEM;

	err = prep_cls_rule(net_dev, fs, dma_mem);
	if (err)
		goto err_free_mem;

	rule_cfg.key_iova = dma_map_single(dev, dma_mem,
					   rule_cfg.key_size * 2,
					   DMA_TO_DEVICE);

	rule_cfg.mask_iova = rule_cfg.key_iova + rule_cfg.key_size;

	if (fs->ring_cookie == RX_CLS_FLOW_DISC)
		fs_act.options |= DPNI_FS_OPT_DISCARD;
	else
		fs_act.flow_id = fs->ring_cookie;

	for (tc = 0; tc < dpaa2_eth_tc_count(priv); tc++) {
		if (add)
			err = dpni_add_fs_entry(priv->mc_io, 0, priv->mc_token,
						tc, fs->location, &rule_cfg,
						&fs_act);
		else
			err = dpni_remove_fs_entry(priv->mc_io, 0,
						   priv->mc_token, tc,
						   &rule_cfg);

		if (err)
			break;
	}

	dma_unmap_single(dev, rule_cfg.key_iova,
			 rule_cfg.key_size * 2, DMA_TO_DEVICE);

	if (err)
		netdev_err(net_dev, "dpaa2_add/remove_cls() error %d\n", err);

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

static int set_hash(struct net_device *net_dev, u64 data)
{
	struct dpaa2_eth_priv *priv = netdev_priv(net_dev);
	u32 key = 0;
	int i;

	if (data & RXH_DISCARD)
		return -EOPNOTSUPP;

	for (i = 0; i < priv->num_dist_fields; i++)
		if (priv->dist_fields[i].rxnfc_field & data)
			key |= priv->dist_fields[i].id;

	return dpaa2_eth_set_dist_key(priv, DPAA2_ETH_RX_DIST_HASH, key);
}

static int dpaa2_eth_set_rxnfc(struct net_device *net_dev,
			       struct ethtool_rxnfc *rxnfc)
{
	int err = 0;

	switch (rxnfc->cmd) {
	case ETHTOOL_SRXCLSRLINS:
		err = add_cls(net_dev, &rxnfc->fs);
		break;
	case ETHTOOL_SRXCLSRLDEL:
		err = del_cls(net_dev, rxnfc->fs.location);
		break;
	case ETHTOOL_SRXFH:
		err = set_hash(net_dev, rxnfc->data);
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
	const int rule_cnt = dpaa2_eth_fs_count(priv);
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
	.get_link_ksettings = dpaa2_eth_get_link_ksettings,
	.set_link_ksettings = dpaa2_eth_set_link_ksettings,
	.get_pauseparam = dpaa2_eth_get_pauseparam,
	.set_pauseparam = dpaa2_eth_set_pauseparam,
	.get_sset_count = dpaa2_eth_get_sset_count,
	.get_ethtool_stats = dpaa2_eth_get_ethtool_stats,
	.get_strings = dpaa2_eth_get_strings,
	.get_rxnfc = dpaa2_eth_get_rxnfc,
	.set_rxnfc = dpaa2_eth_set_rxnfc,
};
