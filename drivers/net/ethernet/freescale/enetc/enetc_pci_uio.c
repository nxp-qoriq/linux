// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2025 NXP */

/* enetc_pci_uio - Generic ENETC PCI UIO driver
 *
 * This driver refers to drivers/uio/uio_pci_generic.c
 *
 * Since the driver does not declare any device ids, you must allocate
 * id and bind the device to the driver yourself.  For example:
 *
 * echo -n 0001:01:10.0 > /sys/bus/pci/drivers/fsl_enetc4/unbind
 * echo -n enetc_pci_uio > /sys/bus/pci/devices/0001:01:10.0/driver_override
 * echo -n 0001:01:10.0 > /sys/bus/pci/drivers/enetc_pci_uio/bind
 *
 * # ls -l /sys/bus/pci/devices/0001:01:10.0/driver
 *  /sys/bus/pci/devices/0001:01:10.0/driver -> ../../../../../../../bus/pci/drivers/enetc_pci_uio
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>

#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/fsl/netc_global.h>

#include "enetc_pf.h"

#define DRIVER_VERSION	"0.01.0"
#define DRIVER_DESC	"Generic ENETC PCI UIO driver"

/*
 * Enable EtherCAT link support
 * speed: 100Mbps
 * pause: disabled
 * EEE: disabled
 */
#define ECAT_LINK_100M

struct enetc_pci_uio_dev {
	struct uio_info info;
	struct pci_dev *pdev;
};

static inline struct enetc_pci_uio_dev *to_enetc_pci_uio_dev(struct uio_info *info)
{
	return container_of(info, struct enetc_pci_uio_dev, info);
}

static int release(struct uio_info *info, struct inode *inode)
{
	struct enetc_pci_uio_dev *gdev = to_enetc_pci_uio_dev(info);

	/*
	 * This driver is insecure when used with devices doing DMA, but some
	 * people (mis)use it with such devices.
	 * Let's at least make sure DMA isn't left enabled after the userspace
	 * driver closes the fd.
	 * Note that there's a non-zero chance doing this will wedge the device
	 * at least until reset.
	 */
	pci_clear_master(gdev->pdev);
	return 0;
}

/* Interrupt handler. Read/modify/write the command register to disable the interrupt. */
static irqreturn_t irqhandler(int irq, struct uio_info *info)
{
	struct enetc_pci_uio_dev *gdev = to_enetc_pci_uio_dev(info);

	if (!pci_check_and_mask_intx(gdev->pdev))
		return IRQ_NONE;

	/* UIO core will signal the user process. */
	return IRQ_HANDLED;
}

static void enetc4_mac_config(struct enetc_pf *pf, unsigned int mode, phy_interface_t phy_mode)
{
	struct enetc_ndev_priv *priv = pf->si->priv;
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val &= ~(PM_IF_MODE_IFMODE | PM_IF_MODE_ENA);

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= IFMODE_RGMII;
		/* We need to enable auto-negotiation for the MAC
		 * if its RGMII interface support In-Band status.
		 */
		if (phylink_autoneg_inband(mode))
			val |= PM_IF_MODE_ENA;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= IFMODE_RMII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		val |= IFMODE_SGMII;
		break;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_XGMII:
	case PHY_INTERFACE_MODE_USXGMII:
		val |= IFMODE_XGMII;
		break;
	default:
		dev_err(priv->dev, "Unsupported PHY mode:%d\n", phy_mode);
		return;
	}

	dev_info(priv->dev, "ENETC PHY mode:%d\n", phy_mode);
	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static struct phylink_pcs *enetc4_pl_mac_select_pcs(struct phylink_config *config,
						    phy_interface_t iface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	return pf->pcs;
}

static void enetc4_pl_mac_config(struct phylink_config *config, unsigned int mode,
				 const struct phylink_link_state *state)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	enetc4_mac_config(pf, mode, state->interface);
}

static void enetc4_set_port_speed(struct enetc_ndev_priv *priv, int speed)
{
	u32 old_speed = priv->speed;
	u32 val;

	if (speed == old_speed)
		return;

	val = enetc_port_rd(&priv->si->hw, ENETC4_PCR);
	val &= ~PCR_PSPEED;

	switch (speed) {
	case SPEED_10:
	case SPEED_100:
	case SPEED_1000:
	case SPEED_2500:
	case SPEED_10000:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(speed));
		break;
	default:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(SPEED_10));
	}

	priv->speed = speed;
	enetc_port_wr(&priv->si->hw, ENETC4_PCR, val);
}

static void enetc4_set_rgmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_M10 | PM_IF_MODE_REVMII);

	switch (speed) {
	case SPEED_1000:
		val = u32_replace_bits(val, SSP_1G, PM_IF_MODE_SSP);
		break;
	case SPEED_100:
		val = u32_replace_bits(val, SSP_100M, PM_IF_MODE_SSP);
		break;
	case SPEED_10:
		val = u32_replace_bits(val, SSP_10M, PM_IF_MODE_SSP);
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1, PM_IF_MODE_HD);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_rmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_SSP);

	switch (speed) {
	case SPEED_100:
		val &= ~PM_IF_MODE_M10;
		break;
	case SPEED_10:
		val |= PM_IF_MODE_M10;
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1, PM_IF_MODE_HD);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_hd_flow_control(struct enetc_pf *pf, bool enable)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	if (!pf->caps.half_duplex)
		return;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, enable ? 1 : 0, PM_CMD_CFG_HD_FCEN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_rx_pause(struct enetc_pf *pf, bool rx_pause)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, rx_pause ? 0 : 1, PM_CMD_CFG_PAUSE_IGN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_tx_pause(struct enetc_pf *pf, int num_rxbdr, bool tx_pause)
{
	u32 pause_off_thresh = 0, pause_on_thresh = 0;
	u32 init_quanta = 0, refresh_quanta = 0;
	struct enetc_hw *hw = &pf->si->hw;
	u32 rbmr, old_rbmr;
	int i;

	for (i = 0; i < num_rxbdr; i++) {
		old_rbmr = enetc_rxbdr_rd(hw, i, ENETC_RBMR);
		rbmr = u32_replace_bits(old_rbmr, tx_pause ? 1 : 0, ENETC_RBMR_CM);
		if (rbmr == old_rbmr)
			continue;

		enetc_rxbdr_wr(hw, i, ENETC_RBMR, rbmr);
	}

	if (tx_pause) {
		/* When the port first enters congestion, send a PAUSE request
		 * with the maximum number of quanta. When the port exits
		 * congestion, it will automatically send a PAUSE frame with
		 * zero quanta.
		 */
		init_quanta = 0xffff;

		/* Also, set up the refresh timer to send follow-up PAUSE
		 * frames at half the quanta value, in case the congestion
		 * condition persists.
		 */
		refresh_quanta = 0xffff / 2;

		/* Start emitting PAUSE frames when 3 large frames (or more
		 * smaller frames) have accumulated in the FIFO waiting to be
		 * DMAed to the RX ring.
		 */
		pause_on_thresh = 3 * ENETC_MAC_MAXFRM_SIZE;
		pause_off_thresh = 1 * ENETC_MAC_MAXFRM_SIZE;
	}

	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_QUANTA(0), init_quanta);
	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_THRESH(0), refresh_quanta);
	enetc_port_wr(hw, ENETC4_PPAUONTR, pause_on_thresh);
	enetc_port_wr(hw, ENETC4_PPAUOFFTR, pause_off_thresh);
}

static void enetc4_enable_mac(struct enetc_pf *pf, bool en)
{
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val &= ~(PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN);
	val |= en ? (PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN) : 0;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_pf_send_link_status_msg(struct enetc_pf *pf, bool up)
{
	struct device *dev = &pf->si->pdev->dev;
	union enetc_pf_msg pf_msg;
	u16 ms_mask = 0;
	int i, err;

	for (i = 0; i < pf->num_vfs; i++)
		if (pf->vf_link_status_notify[i])
			ms_mask |= PSIMSGSR_MS(i);

	if (!ms_mask)
		return;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	pf_msg.class_code = up ? ENETC_PF_NC_LINK_STATUS_UP : ENETC_PF_NC_LINK_STATUS_DOWN;

	err = enetc_pf_send_msg(pf, pf_msg.code, ms_mask);
	if (err)
		dev_err(dev, "PF notifies link status failed\n");
}

static void enetc4_pl_mac_link_up(struct phylink_config *config,
				  struct phy_device *phy, unsigned int mode,
				  phy_interface_t interface, int speed,
				  int duplex, bool tx_pause, bool rx_pause)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;
	bool hd_fc = false;

	priv = si->priv;
	enetc4_set_port_speed(priv, speed);

	if (!phylink_autoneg_inband(mode) && phy_interface_mode_is_rgmii(interface))
		enetc4_set_rgmii_mac(pf, speed, duplex);

	if (interface == PHY_INTERFACE_MODE_RMII)
		enetc4_set_rmii_mac(pf, speed, duplex);

	if (duplex == DUPLEX_FULL) {
		tx_pause = false;
		rx_pause = false;
	} else { /* DUPLEX_HALF */
		if (tx_pause || rx_pause)
			hd_fc = true;

		/* As per 802.3 annex 31B, PAUSE frames are only supported
		 * when the link is configured for full duplex operation.
		 */
		tx_pause = false;
		rx_pause = false;
	}

	enetc4_set_hd_flow_control(pf, hd_fc);
	enetc4_set_tx_pause(pf, priv->num_rx_rings, tx_pause);
	enetc4_set_rx_pause(pf, rx_pause);
	enetc4_enable_mac(pf, true);

#ifdef ECAT_LINK_100M
	enetc_port_mac_wr(si, ENETC4_PM_SLEEP_TIMER(0), 0);
	enetc_port_mac_wr(si, ENETC4_PM_LPWAKE_TIMER(0), 0);
#endif

	enetc4_pf_send_link_status_msg(pf, true);
}

static void enetc4_pl_mac_link_down(struct phylink_config *config, unsigned int mode,
				    phy_interface_t interface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;

	priv = si->priv;

	enetc4_pf_send_link_status_msg(pf, false);
	enetc4_enable_mac(pf, false);
}

static const struct phylink_mac_ops enetc_pl_mac_ops = {
	.mac_select_pcs = enetc4_pl_mac_select_pcs,
	.mac_config = enetc4_pl_mac_config,
	.mac_link_up = enetc4_pl_mac_link_up,
	.mac_link_down = enetc4_pl_mac_link_down,
};

#define MAC_REV1 (MAC_10 | MAC_100 | MAC_1000 | MAC_2500FD)
#define MAC_REV4 (MAC_10 | MAC_100 | MAC_1000FD | MAC_2500FD | MAC_10000FD)
static int enetc_phylink_setup(struct enetc_ndev_priv *priv, struct device_node *node,
			       const struct phylink_mac_ops *pl_mac_ops)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct phylink_config *pc = &pf->phylink_config;
	struct phylink *phylink;
	int err;

	pc->dev = priv->dev;
	pc->type = PHYLINK_DEV;

#ifdef ECAT_LINK_100M
	pc->mac_capabilities = MAC_REV4;
#else
	if (is_enetc_rev1(priv->si))
		pc->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE | MAC_REV1;
	else
		pc->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |	MAC_REV4;

#endif
	__set_bit(PHY_INTERFACE_MODE_INTERNAL, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_SGMII, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_RMII, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_1000BASEX, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_2500BASEX, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_10GBASER, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_USXGMII, pc->supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_XGMII, pc->supported_interfaces);

	phy_interface_set_rgmii(pc->supported_interfaces);

	phylink = phylink_create(pc, of_fwnode_handle(node), pf->if_mode, pl_mac_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		return err;
	}
	priv->phylink = phylink;

	return 0;
}

static int enetc_phylink_start(struct enetc_ndev_priv *priv)
{
	int err;

	err = phylink_of_phy_connect(priv->phylink, priv->dev->of_node, 0);
	if (err) {
		dev_err(priv->dev, "could not attach to PHY\n");
		return err;
	}

	rtnl_lock();
	phylink_start(priv->phylink);
	rtnl_unlock();

	clear_bit(ENETC_TX_DOWN, &priv->flags);

	return 0;
}

static int enetc_phylink_stop(struct enetc_ndev_priv *priv)
{
	set_bit(ENETC_TX_DOWN, &priv->flags);

	if (!priv->phylink)
		return 0;

	rtnl_lock();
	phylink_stop(priv->phylink);
	phylink_disconnect_phy(priv->phylink);
	rtnl_unlock();

	return 0;
}

static int enetc_probe_enetc_port(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct enetc_ndev_priv *priv = NULL;
	struct device_node *phy_node;
	struct phy_device *phy_dev;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err = 0;

	if (!of_device_is_compatible(node, "pci1131,e101") &&
	    !of_device_is_compatible(node, "fsl,imx95-enetc")) {
		dev_err(&pdev->dev, "the port is not compatible with the driver\n");
		return -EINVAL;
	}

	if (enetc_pf_is_owned_by_mcore(pdev)) {
		dev_err(&pdev->dev, "the port is owned by M-core!\n");
		return -EBUSY;
	}

	if (pdev->is_virtfn) {
		dev_err(dev, "virtual function is not supported\n");
		return -ENODEV;
	}

	phy_node = of_parse_phandle(node, "phy-handle", 0);
	phy_dev = of_phy_find_device(phy_node);
	if (!phy_dev) {
		dev_err(dev, "failed to find phy_node: %d\n", err);
		return -ENODEV;
	}
	put_device(&phy_dev->mdio.dev);
	of_node_put(phy_node);

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, sizeof(*pf));
	if (err) {
		dev_err(dev, "PCIe probing failed\n");
		return err;
	}

	/* si is the private data. */
	si = pci_get_drvdata(pdev);
	if (!si->hw.port || !si->hw.global) {
		err = -ENODEV;
		dev_err(dev, "Couldn't map PF only space!\n");
		goto err_map_mem;
	}

	pf = enetc_si_priv(si);
	pf->si = si;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto err_alloc_mem;
	}
	priv->ndev = NULL;
	priv->dev = dev;
	priv->si = si;
	si->ndev = NULL;
	si->priv = priv;

	priv->ref_clk = devm_clk_get_optional(dev, "enet_ref_clk");
	if (IS_ERR(priv->ref_clk)) {
		dev_err(dev, "Get enet_ref_clk failed\n");
		err = PTR_ERR(priv->ref_clk);
		goto err_clk_get;
	}

	err = of_get_phy_mode(node, &pf->if_mode);
	if (err) {
		dev_err(dev, "Failed to get PHY mode\n");
		goto err_clk_get;
	}

	err = enetc_mdiobus_create(pf, node);
	if (err) {
		dev_err(dev, "Failed to create MDIO bus\n");
		goto err_clk_get;
	}

	err = enetc_phylink_setup(priv, node, &enetc_pl_mac_ops);
	if (err) {
		dev_err(dev, "Failed to create phylink\n");
		goto err_phylink_create;
	}

	err = clk_prepare_enable(priv->ref_clk);
	if (err) {
		dev_err(dev, "Failed to enable enet_ref_clk\n");
		goto err_clk_enable;
	}

	enetc_phylink_start(priv);

	return 0;

err_clk_enable:
	enetc_phylink_destroy(priv);
err_phylink_create:
	enetc_mdiobus_destroy(pf);
err_clk_get:
	kfree(priv);
err_alloc_mem:
err_map_mem:
	enetc_pci_remove(pdev);

	return err;
}

static int enetc_remove_enetc_port(struct pci_dev *pdev)
{
	struct enetc_ndev_priv *priv;
	struct enetc_si *si;
	struct enetc_pf *pf;

	si = pci_get_drvdata(pdev);
	pf = enetc_si_priv(si);
	priv = si->priv;

	enetc_phylink_stop(priv);
	clk_disable_unprepare(priv->ref_clk);

	enetc_phylink_destroy(priv);
	enetc_mdiobus_destroy(pf);
	enetc_pci_remove(pdev);

	kfree(priv);

	return 0;
}

static int enetc_pci_uio_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct enetc_pci_uio_dev *gdev;
	struct uio_mem *uiomem;
	int err;
	int i;

	err = enetc_probe_enetc_port(pdev);
	if (err)
		return err;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n", __func__, err);
		return err;
	}

	if (pdev->irq && !pci_intx_mask_supported(pdev))
		return -ENODEV;

	gdev = devm_kzalloc(&pdev->dev, sizeof(struct enetc_pci_uio_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	gdev->info.name = "enetc_pci_uio";
	gdev->info.version = DRIVER_VERSION;
	gdev->info.release = release;
	gdev->pdev = pdev;
	if (pdev->irq && pdev->irq != IRQ_NOTCONNECTED) {
		gdev->info.irq = pdev->irq;
		gdev->info.irq_flags = IRQF_SHARED;
		gdev->info.handler = irqhandler;
	}

	uiomem = &gdev->info.mem[0];
	for (i = 0; i < MAX_UIO_MAPS; ++i) {
		struct resource *r = &pdev->resource[i];

		if (r->flags != (IORESOURCE_SIZEALIGN | IORESOURCE_MEM))
			continue;

		if (uiomem >= &gdev->info.mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
				__stringify(MAX_UIO_MAPS) " I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start & PAGE_MASK;
		uiomem->offs = r->start & ~PAGE_MASK;
		uiomem->size = (uiomem->offs + resource_size(r) + PAGE_SIZE - 1) & PAGE_MASK;
		uiomem->name = r->name;
		++uiomem;
	}

	while (uiomem < &gdev->info.mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	return devm_uio_register_device(&pdev->dev, &gdev->info);
}

static void enetc_pci_uio_remove(struct pci_dev *pdev)
{
	enetc_remove_enetc_port(pdev);

	pci_disable_device(pdev);
}

static struct pci_driver enetc_pci_uio_driver = {
	.name = "enetc_pci_uio",
	.id_table = NULL, /* only dynamic id's */
	.probe = enetc_pci_uio_probe,
	.remove = enetc_pci_uio_remove,
};
module_pci_driver(enetc_pci_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
