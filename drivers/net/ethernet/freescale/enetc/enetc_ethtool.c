#include "enetc.h"

#define PR_REG_FMT_STR(NAME) __stringify(NAME) "[0x%X]: %08x\n"
#define PR_BDR_REG_FMT_STR(NAME) "ring %d: " PR_REG_FMT_STR(NAME)

#define PR_REG(hw, NAME)	pr_info(PR_REG_FMT_STR(NAME), ENETC_##NAME, \
					enetc_rd(hw, ENETC_##NAME))
#define PR_PREG(hw, NAME)	pr_info(PR_REG_FMT_STR(NAME), ENETC_##NAME, \
					enetc_port_rd(hw, ENETC_##NAME))
#define PR_GREG(hw, NAME)	pr_info(PR_REG_FMT_STR(NAME), ENETC_##NAME, \
					enetc_global_rd(hw, ENETC_##NAME))
#define enetc_rxbdr_off(i, off) ENETC_BDR(RX, i, (off))
#define enetc_txbdr_off(i, off) ENETC_BDR(TX, i, (off))
#define PR_BDR_REG(hw, t, i, NAME) \
			pr_info(PR_BDR_REG_FMT_STR(NAME), i, \
				enetc_##t##bdr_off(i, ENETC_##NAME), \
				enetc_##t##bdr_rd(hw, i, ENETC_##NAME))

static int enetc_get_reglen(struct net_device *ndev)
{
	return 0;
}

static void enetc_get_regs(struct net_device *ndev, struct ethtool_regs *regs,
			   void *regbuf)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	int i;

	/* SI regs */
	PR_REG(hw, SIMR);
	PR_REG(hw, SIPMAR0);

	/** Control ring regs */
	PR_REG(hw, SICBDRMR);
	PR_REG(hw, SICBDRSR);
	PR_REG(hw, SICBDRBAR0);
	PR_REG(hw, SICBDRBAR1);
	PR_REG(hw, SICBDRCIR);
	PR_REG(hw, SICBDRCISR);
	PR_REG(hw, SICBDRLENR);

	PR_REG(hw, SIPMAR1);
	PR_REG(hw, SICAPR0);
	PR_REG(hw, SICAPR1);

	PR_REG(hw, SIMSIVR);
	for (i = 0; i < priv->bdr_int_num; i++) {
		PR_REG(hw, SIMSITRV(i));
		PR_REG(hw, SIMSIRRV(i));
	}

	PR_REG(hw, PRFSCAPR);
	/** Tx BDR dump */
	for (i = 0; i < priv->num_tx_rings; i++) {
		PR_BDR_REG(hw, tx, i, TBMR);
		PR_BDR_REG(hw, tx, i, TBSR);
		PR_BDR_REG(hw, tx, i, TBBAR0);
		PR_BDR_REG(hw, tx, i, TBBAR1);
		PR_BDR_REG(hw, tx, i, TBCIR);
		PR_BDR_REG(hw, tx, i, TBCISR);
		PR_BDR_REG(hw, tx, i, TBLENR);
		PR_BDR_REG(hw, tx, i, TBIER);
	}
	/** Rx BDR dump */
	for (i = 0; i < priv->num_rx_rings; i++) {
		PR_BDR_REG(hw, rx, i, RBMR);
		PR_BDR_REG(hw, rx, i, RBSR);
		PR_BDR_REG(hw, rx, i, RBBSR);
		PR_BDR_REG(hw, rx, i, RBCIR);
		PR_BDR_REG(hw, rx, i, RBBAR0);
		PR_BDR_REG(hw, rx, i, RBBAR1);
		PR_BDR_REG(hw, rx, i, RBPIR);
		PR_BDR_REG(hw, rx, i, RBLENR);
		PR_BDR_REG(hw, rx, i, RBICIR0);
		PR_BDR_REG(hw, rx, i, RBIER);
	}

	if (hw->port) {
		/* Port regs */
		PR_PREG(hw, PMR);
		PR_PREG(hw, PSR);
		PR_PREG(hw, PSIPMR);
		PR_PREG(hw, PSIPMAR0(0));
		PR_PREG(hw, PSIPMAR1(0));
		PR_PREG(hw, PCAPR0);
		PR_PREG(hw, PCAPR1);
		PR_PREG(hw, PV0CFGR(0));
		PR_PREG(hw, PM0_CMD_CFG);
		PR_PREG(hw, PM0_MAXFRM);
	}

	if (hw->global) {
		/* GLobal regs */
		PR_GREG(hw, G_EIPBRR0);
		PR_GREG(hw, G_EIPBRR1);
	}
}

static struct {
	int reg;
	char name[ETH_GSTRING_LEN];
} enetc_si_counters[] =  {
	{ ENETC_SIROCT, "SI received octets" },
	{ ENETC_SIRFRM, "SI received frames" },
	{ ENETC_SIRUCA, "SI received unicast frames" },
	{ ENETC_SIRMCA, "SI received multicast frames" },
	{ ENETC_SITOCT, "SI transmit octets" },
	{ ENETC_SITFRM, "SI transmit frames" },
	{ ENETC_SITUCA, "SI transmit unicast frames" },
	{ ENETC_SITMCA, "SI transmit multicast frames" },
};

static int enetc_get_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(enetc_si_counters);
	default:
		return -EOPNOTSUPP;
	}
}

static void enetc_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(enetc_si_counters); i++) {
			strlcpy(p, enetc_si_counters[i].name, ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	}
}

static void enetc_get_ethtool_stats(struct net_device *ndev,
				    struct ethtool_stats *stats, u64 *data)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	int i;

	for (i = 0; i < ARRAY_SIZE(enetc_si_counters); i++)
		data[i] = enetc_rd64(hw, enetc_si_counters[i].reg);
}

/* current HW spec does byte reversal on everything including MAC addresses */
static void ether_addr_copy_swap(u8 *dst, const u8 *src)
{
	int i;

	for (i = 0; i < 6; i++)
		dst[i] = src[5 - i];
}

static int enetc_set_cls_entry(struct enetc_si *si,
			       struct ethtool_rx_flow_spec *fs, bool en)
{
	struct ethtool_tcpip4_spec *l4ip4_h, *l4ip4_m;
	struct ethtool_usrip4_spec *l3ip4_h, *l3ip4_m;
	struct ethhdr *eth_h, *eth_m;
	struct enetc_cmd_rfse rfse = { {0} };

	if (!en)
		goto done;

	switch (fs->flow_type & 0xff) {
	case TCP_V4_FLOW:
		l4ip4_h = &fs->h_u.tcp_ip4_spec;
		l4ip4_m = &fs->m_u.tcp_ip4_spec;
		goto l4ip4;
	case UDP_V4_FLOW:
		l4ip4_h = &fs->h_u.udp_ip4_spec;
		l4ip4_m = &fs->m_u.udp_ip4_spec;
		goto l4ip4;
	case SCTP_V4_FLOW:
		l4ip4_h = &fs->h_u.sctp_ip4_spec;
		l4ip4_m = &fs->m_u.sctp_ip4_spec;
l4ip4:
		rfse.sip_h[0] = ntohl(l4ip4_h->ip4src);
		rfse.sip_m[0] = ntohl(l4ip4_m->ip4src);
		rfse.dip_h[0] = ntohl(l4ip4_h->ip4dst);
		rfse.dip_m[0] = ntohl(l4ip4_m->ip4dst);
		rfse.sport_h = ntohs(l4ip4_h->psrc);
		rfse.sport_m = ntohs(l4ip4_m->psrc);
		rfse.dport_h = ntohs(l4ip4_h->pdst);
		rfse.dport_m = ntohs(l4ip4_m->pdst);
		if (l4ip4_m->tos)
			netdev_warn(si->ndev, "ToS field is not supported and was ignored\n");
		rfse.ethtype_h = 0x0800; /* IPv4 */
		rfse.ethtype_m = 0xffff;
		break;
	case IP_USER_FLOW:
		l3ip4_h = &fs->h_u.usr_ip4_spec;
		l3ip4_m = &fs->m_u.usr_ip4_spec;

		rfse.sip_h[0] = ntohl(l3ip4_h->ip4src);
		rfse.sip_m[0] = ntohl(l3ip4_m->ip4src);
		rfse.dip_h[0] = ntohl(l3ip4_h->ip4dst);
		rfse.dip_m[0] = ntohl(l3ip4_m->ip4dst);
		if (l3ip4_m->tos)
			netdev_warn(si->ndev, "ToS field is not supported and was ignored\n");
		rfse.ethtype_h = 0x0800; /* IPv4 */
		rfse.ethtype_m = 0xffff;
		break;
	case ETHER_FLOW:
		eth_h = &fs->h_u.ether_spec;
		eth_m = &fs->m_u.ether_spec;

		ether_addr_copy_swap(rfse.smac_h, eth_h->h_source);
		ether_addr_copy_swap(rfse.smac_m, eth_m->h_source);
		ether_addr_copy_swap(rfse.dmac_h, eth_h->h_dest);
		ether_addr_copy_swap(rfse.dmac_m, eth_m->h_dest);
		rfse.ethtype_h = ntohs(eth_h->h_proto);
		rfse.ethtype_m = ntohs(eth_m->h_proto);
		break;
	default:
		return -EOPNOTSUPP;
	}

	rfse.mode |= ENETC_RFSE_EN;
	if (fs->ring_cookie != RX_CLS_FLOW_DISC) {
		rfse.mode |= ENETC_RFSE_MODE_BD;
		rfse.result = fs->ring_cookie;
	}
done:
	return enetc_set_fs_entry(si, &rfse, fs->location);
}

static int enetc_get_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *rxnfc,
			   u32 *rule_locs)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int i, j;

	switch (rxnfc->cmd) {
	case ETHTOOL_GRXRINGS:
		rxnfc->data = priv->num_rx_rings;
		break;
	case ETHTOOL_GRXCLSRLCNT:
		/* total number of entries */
		rxnfc->data = priv->si->num_fs_entries;
		/* number of entries in use */
		rxnfc->rule_cnt = 0;
		for (i = 0; i < priv->si->num_fs_entries; i++)
			if (priv->cls_rules[i].used)
				rxnfc->rule_cnt++;
		break;
	case ETHTOOL_GRXCLSRULE:
		/* get entry x */
		rxnfc->fs = priv->cls_rules[rxnfc->fs.location].fs;
		break;
	case ETHTOOL_GRXCLSRLALL:
		/* total number of entries */
		rxnfc->data = priv->si->num_fs_entries;
		/* array of indexes of used entries */
		j = 0;
		for (i = 0; i < priv->si->num_fs_entries; i++) {
			if (!priv->cls_rules[i].used)
				continue;
			if (j == rxnfc->rule_cnt)
				return -EMSGSIZE;
			rule_locs[j++] = i;
		}
		/* number of entries in use */
		rxnfc->rule_cnt = j;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int enetc_set_rxnfc(struct net_device *ndev, struct ethtool_rxnfc *rxnfc)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	int err;

	switch (rxnfc->cmd) {
	case ETHTOOL_SRXCLSRLINS:
		err = enetc_set_cls_entry(priv->si, &rxnfc->fs, true);
		if (err)
			return err;
		priv->cls_rules[rxnfc->fs.location].fs = rxnfc->fs;
		priv->cls_rules[rxnfc->fs.location].used = 1;
		break;
	case ETHTOOL_SRXCLSRLDEL:
		err = enetc_set_cls_entry(priv->si, &rxnfc->fs, false);
		if (err)
			return err;
		priv->cls_rules[rxnfc->fs.location].used = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

const struct ethtool_ops enetc_ethtool_ops = {
	.get_regs_len = enetc_get_reglen,
	.get_regs = enetc_get_regs,
	.get_sset_count = enetc_get_sset_count,
	.get_strings = enetc_get_strings,
	.get_ethtool_stats = enetc_get_ethtool_stats,
	.get_rxnfc = enetc_get_rxnfc,
	.set_rxnfc = enetc_set_rxnfc,
};

void enetc_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &enetc_ethtool_ops;
}
