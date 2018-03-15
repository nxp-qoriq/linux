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

#include <linux/module.h>
#include "enetc.h"

#define ENETC_DRV_VER_MAJ 0
#define ENETC_DRV_VER_MIN 8

#define ENETC_DRV_VER_STR __stringify(ENETC_DRV_VER_MAJ) "." \
			  __stringify(ENETC_DRV_VER_MIN)
static const char enetc_drv_ver[] = ENETC_DRV_VER_STR;
#define ENETC_DRV_NAME_STR "ENETC VF driver"
static const char enetc_drv_name[] = ENETC_DRV_NAME_STR;

/* VF driver params */
module_param(debug, uint, 0);

/* Messaging */
static void enetc_msg_vsi_write_msg(struct enetc_hw *hw,
				    struct enetc_msg_swbd *msg)
{
	u32 val;

	val = enetc_vsi_set_msize(msg->size) | lower_32_bits(msg->dma);
	enetc_wr(hw, ENETC_VSIMSGSNDAR0, val);
	enetc_wr(hw, ENETC_VSIMSGSNDAR1, upper_32_bits(msg->dma));
}

static int enetc_msg_vsi_send(struct enetc_si *si, struct enetc_msg_swbd *msg)
{
	int timeout = 100;
	u32 vsimsgsr;

	enetc_msg_vsi_write_msg(&si->hw, msg);

	do {
		vsimsgsr = enetc_rd(&si->hw, ENETC_VSIMSGSR);
		if (!(vsimsgsr & ENETC_VSIMSGSR_MB))
			break;

		usleep_range(1000, 2000);
	} while (--timeout);

	if (!timeout)
		return -ETIMEDOUT;

	/* check for message delivery error */
	if (vsimsgsr & ENETC_VSIMSGSR_MS) {
		dev_err(&si->pdev->dev, "VSI command execute error: %d\n",
			ENETC_SIMSGSR_GET_MC(vsimsgsr));
		return -EIO;
	}

	return 0;
}

static int enetc_msg_vsi_set_primary_mac_addr(struct enetc_ndev_priv *priv,
					      struct sockaddr *saddr)
{
	struct enetc_msg_cmd_set_primary_mac *cmd;
	struct enetc_msg_swbd msg;
	int err;

	msg.size = ALIGN(sizeof(struct enetc_msg_cmd_set_primary_mac), 64);
	msg.vaddr = dma_zalloc_coherent(priv->dev, msg.size, &msg.dma,
					GFP_KERNEL);
	if (!msg.vaddr) {
		dev_err(priv->dev, "Failed to alloc Tx msg (size: %d)\n",
			msg.size);
		return -ENOMEM;
	}

	cmd = (struct enetc_msg_cmd_set_primary_mac *)msg.vaddr;
	cmd->header.type = ENETC_MSG_CMD_MNG_MAC;
	cmd->header.id = ENETC_MSG_CMD_MNG_ADD;
	memcpy(&cmd->mac, saddr, sizeof(struct sockaddr));

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg);

	dma_free_coherent(priv->dev, msg.size, msg.vaddr, msg.dma);

	return err;
}

static int enetc_vf_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sockaddr *saddr = addr;
	int err;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	err = enetc_msg_vsi_set_primary_mac_addr(priv, saddr);
	if (err)
		return err;

	return 0;
}

/* Probing/ Init */
static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_vf_set_mac_addr,
};

static void enetc_vf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
				  const struct net_device_ops *ndev_ops)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_IFUP << 1) - 1; //TODO: netif_msg_init()
	ndev->netdev_ops = ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ENETC_MAX_MTU;

	ndev->hw_features = NETIF_F_RXCSUM | NETIF_F_HW_CSUM |
			    NETIF_F_HW_VLAN_CTAG_TX |
			    NETIF_F_HW_VLAN_CTAG_RX;
	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG |
			 NETIF_F_RXCSUM | NETIF_F_HW_CSUM |
			 NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_HW_VLAN_CTAG_RX; /* < has to stay on for now */

	ndev->priv_flags |= IFF_UNICAST_FLT;

	/* pick up primary MAC address from SI */
	enetc_get_primary_mac_addr(&si->hw, ndev->dev_addr);
}

static void enetc_vf_sw_init(struct enetc_ndev_priv *priv)
{
	struct enetc_si *si = priv->si;

	priv->tx_bd_count = 1024; //TODO: use defines for defaults
	priv->rx_bd_count = 1024;

	/* we only use one ring per CPU for now */
	priv->num_rx_rings = min_t(u16, num_online_cpus(),
				   priv->si->num_rx_rings);
	priv->num_tx_rings = min_t(u16, num_online_cpus(),
				   priv->si->num_tx_rings);
	priv->bdr_int_num = priv->num_rx_rings; /* int for each Tx/Rx pairs */

	/* SI specific */
	si->cbd_ring.bd_count = 64; //TODO: use defines for defaults
}

static int enetc_vf_probe(struct pci_dev *pdev,
			  const struct pci_device_id *ent)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, 0);
	if (err) {
		dev_err(&pdev->dev, "PCI probing failed\n");
		return err;
	}

	si = pci_get_drvdata(pdev);

	enetc_get_si_caps(si);

	ndev = alloc_etherdev_mq(sizeof(*priv), ENETC_MAX_NUM_TXQS);
	if (!ndev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	enetc_vf_netdev_setup(si, ndev, &enetc_ndev_ops);

	priv = netdev_priv(ndev);

	enetc_vf_sw_init(priv);

	err = enetc_alloc_si_resources(priv);
	if (err) {
		dev_err(&pdev->dev, "SI resource alloc failed\n");
		goto err_alloc_si_res;
	}

	err = enetc_alloc_msix(priv);
	if (err) {
		dev_err(&pdev->dev, "MSIX alloc failed\n");
		goto err_alloc_msix;
	}

	err = register_netdev(ndev);
	if (err)
		goto err_reg_netdev;

	err = enetc_setup_irqs(priv);
	if (err)
		goto err_setup_irq;

	netif_carrier_off(ndev);

	netif_info(priv, probe, ndev, "%s v%s\n",
		   enetc_drv_name, enetc_drv_ver);

	return 0;

err_setup_irq:
	unregister_netdev(ndev);
err_reg_netdev:
	enetc_free_msix(priv);
err_alloc_msix:
	enetc_free_si_resources(priv);
err_alloc_si_res:
	si->ndev = NULL;
	free_netdev(ndev);
err_alloc_netdev:
	enetc_pci_remove(pdev);

	return err;
}

static void enetc_vf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(si->ndev);
	netif_info(priv, drv, si->ndev, "%s v%s remove\n",
		   enetc_drv_name, enetc_drv_ver);
	unregister_netdev(si->ndev);

	enetc_free_irqs(priv);
	enetc_free_msix(priv);

	enetc_free_si_resources(priv);

	free_netdev(si->ndev);

	enetc_pci_remove(pdev);
}

static const struct pci_device_id enetc_vf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_VF) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_vf_id_table);

static struct pci_driver enetc_vf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_vf_id_table,
	.probe = enetc_vf_probe,
	.remove = enetc_vf_remove,
};
module_pci_driver(enetc_vf_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(ENETC_DRV_VER_STR);
