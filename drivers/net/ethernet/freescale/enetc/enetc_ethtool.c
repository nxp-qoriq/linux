#include "enetc.h"

#define PR_REG_FMT_STR(NAME) __stringify(NAME) "[0x%X]: %08x\n"
#define PR_BDR_REG_FMT_STR(i, NAME) "ring " __stringify(i) ": " \
				    PR_REG_FMT_STR(NAME)

#define PR_REG(hw, NAME)	pr_info(PR_REG_FMT_STR(NAME), ENETC_##NAME, \
					enetc_rd(hw, ENETC_##NAME))
#define enetc_rxbdr_off(i, off) ENETC_BDR(RX, i, (off))
#define enetc_txbdr_off(i, off) ENETC_BDR(TX, i, (off))
#define PR_BDR_REG(hw, t, i, NAME) \
			pr_info(PR_BDR_REG_FMT_STR(i, NAME), \
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

	/* SI regs */
	PR_REG(hw, SIMR);
	PR_REG(hw, SICAPR0);
	PR_REG(hw, SICAPR1);
	PR_REG(hw, SICCAPR);
	/** Tx BDR dump */
	PR_BDR_REG(hw, tx, 0, TBMR);
	PR_BDR_REG(hw, tx, 0, TBSR);
	PR_BDR_REG(hw, tx, 0, TBBAR0);
	PR_BDR_REG(hw, tx, 0, TBBAR1);
	PR_BDR_REG(hw, tx, 0, TBCIR);
	PR_BDR_REG(hw, tx, 0, TBCISR);
	PR_BDR_REG(hw, tx, 0, TBIER);
	/** Rx BDR dump */
	PR_BDR_REG(hw, rx, 0, RBMR);
	PR_BDR_REG(hw, rx, 0, RBSR);
	PR_BDR_REG(hw, rx, 0, RBBSR);
	PR_BDR_REG(hw, rx, 0, RBCIR);
	PR_BDR_REG(hw, rx, 0, RBBAR0);
	PR_BDR_REG(hw, rx, 0, RBBAR1);
	PR_BDR_REG(hw, rx, 0, RBPIR);
	PR_BDR_REG(hw, rx, 0, RBICIR0);
	PR_BDR_REG(hw, rx, 0, RBIER);

	/* Port regs */
	PR_REG(hw, PMR);
	PR_REG(hw, PSR);
	PR_REG(hw, PSIPMR);
	PR_REG(hw, PCAPR0);
	PR_REG(hw, PCAPR1);
	PR_REG(hw, PM0_CMD_CFG);
	PR_REG(hw, PM0_MAXFRM);

	/* GLobal regs */
	PR_REG(hw, G_EIPBRR0);
	PR_REG(hw, G_EIPBRR1);
}

const struct ethtool_ops enetc_ethtool_ops = {
	.get_regs_len = enetc_get_reglen,
	.get_regs = enetc_get_regs,
};

void enetc_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &enetc_ethtool_ops;
}
