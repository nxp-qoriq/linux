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

	for (i = 0; i < priv->num_int_vectors; i++) {
		PR_REG(hw, SIMSITRV(i));
		PR_REG(hw, SIMSIRRV(i));
	}

	PR_REG(hw, SICCAPR);
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

const struct ethtool_ops enetc_ethtool_ops = {
	.get_regs_len = enetc_get_reglen,
	.get_regs = enetc_get_regs,
	.get_sset_count = enetc_get_sset_count,
	.get_strings = enetc_get_strings,
	.get_ethtool_stats = enetc_get_ethtool_stats,
};

void enetc_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &enetc_ethtool_ops;
}
