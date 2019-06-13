// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix Switch driver
 *
 * Copyright 2018-2019 NXP
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/phy_fixed.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/iopoll.h>
#include <net/switchdev.h>
#include "ocelot.h"

#define FELIX_DRV_VER_MAJ 1
#define FELIX_DRV_VER_MIN 0

#define FELIX_DRV_STR	"Felix Switch driver"
#define FELIX_DRV_VER_STR __stringify(FELIX_DRV_VER_MAJ) "." \
			  __stringify(FELIX_DRV_VER_MIN)

#define PCI_DEVICE_ID_FELIX	0xEEF0

/* Switch register block BAR */
#define FELIX_SWITCH_BAR	4

static struct pci_device_id felix_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FELIX) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, felix_ids);

static struct {
	enum ocelot_target id;
	struct resource res;
} felix_io_res[] = {
	{	.id = ANA,
		{
			.start = 0x0280000,
			.end = 0x028ffff,
			.name = "ana",
		}
	},
	{	.id = QS,
		{
			.start = 0x0080000,
			.end = 0x00800ff,
			.name = "qs",
		}
	},
	{	.id = QSYS,
		{
			.start = 0x0200000,
			.end = 0x021ffff,
			.name = "qsys",
		}
	},
	{	.id = REW,
		{
			.start = 0x0030000,
			.end = 0x003ffff,
			.name = "rew",
		}
	},
	{	.id = SYS,
		{
			.start = 0x0010000,
			.end = 0x001ffff,
			.name = "sys",
		}
	},
	{	.id = S2,
		{
			.start = 0x0060000,
			.end = 0x00603ff,
			.name = "s2",
		}
	},
	{	.id = GCB,
		{
			.start = 0x0070000,
			.end = 0x00701ff,
			.name = "devcpu_gcb",
		}
	}
};

#define FELIX_MAX_NUM_PHY_PORTS	6

#define FELIX_PORT_RES_START	0x0100000
#define FELIX_PORT_RES_SIZE	0x10000

static void __iomem *regs;

static void felix_release_ports(struct ocelot *ocelot)
{
	struct ocelot_port *ocelot_port;
	struct phy_device *phydev;
	struct device_node *dn;
	int i;

	for (i = 0; i < ocelot->num_phys_ports; i++) {
		ocelot_port = ocelot->ports[i];
		if (!ocelot_port || !ocelot_port->phy || !ocelot_port->dev)
			continue;

		phydev = ocelot_port->phy;
		unregister_netdev(ocelot_port->dev);
		free_netdev(ocelot_port->dev);

		if (phy_is_pseudo_fixed_link(phydev)) {
			dn = phydev->mdio.dev.of_node;
			/* decr refcnt: of_phy_register_fixed_link */
			of_phy_deregister_fixed_link(dn);
		}
		phy_device_free(phydev); /* decr refcnt: of_phy_find_device */
	}
}

static int felix_ports_init(struct pci_dev *pdev)
{
	struct ocelot *ocelot = pci_get_drvdata(pdev);
	struct device_node *np = ocelot->dev->of_node;
	struct device_node *phy_node, *portnp;
	struct phy_device *phydev;
	void __iomem *port_regs;
	resource_size_t base;
	u32 port;
	int err;

	ocelot->num_phys_ports = FELIX_MAX_NUM_PHY_PORTS;

	np = of_get_child_by_name(np, "ports");
	if (!np) {
		dev_err(&pdev->dev, "ports sub-node not found\n");
		return -ENODEV;
	}

	/* alloc netdev for each port */
	err = ocelot_init(ocelot);
	if (err)
		return err;

	base = pci_resource_start(pdev, FELIX_SWITCH_BAR);
	for_each_available_child_of_node(np, portnp) {
		struct resource res = {};
		int phy_mode;

		if (!portnp || !portnp->name ||
		    of_node_cmp(portnp->name, "port") ||
		    of_property_read_u32(portnp, "reg", &port))
			continue;

		if (port >= FELIX_MAX_NUM_PHY_PORTS) {
			dev_err(ocelot->dev, "invalid port num: %d\n", port);
			continue;
		}

		res.start = base + FELIX_PORT_RES_START +
			    FELIX_PORT_RES_SIZE * port;
		res.end = res.start + FELIX_PORT_RES_SIZE - 1;
		res.flags = IORESOURCE_MEM;
		res.name = "port";

		port_regs = devm_ioremap_resource(ocelot->dev, &res);
		if (IS_ERR(port_regs)) {
			dev_err(ocelot->dev,
				"failed to map registers for port %d\n", port);
			continue;
		}

		phy_node = of_parse_phandle(portnp, "phy-handle", 0);
		if (!phy_node) {
			if (!of_phy_is_fixed_link(portnp))
				continue;

			err = of_phy_register_fixed_link(portnp);
			if (err < 0) {
				dev_err(ocelot->dev,
					"can't create fixed link for port:%d\n",
					port);
				continue;
			}
			phydev = of_phy_find_device(portnp);
		} else {
			phydev = of_phy_find_device(phy_node);
		}

		of_node_put(phy_node);

		if (!phydev)
			continue;

		phy_mode = of_get_phy_mode(portnp);
		if (phy_mode < 0)
			phy_mode = PHY_INTERFACE_MODE_NA;

		err = ocelot_probe_port(ocelot, port, port_regs, phydev);
		if (err) {
			dev_err(ocelot->dev, "failed to probe ports\n");
			goto release_ports;
		}

		/* Felix configs */
		ocelot->ports[port]->phy_mode = phy_mode;
	}

	return 0;

release_ports:
	felix_release_ports(ocelot);

	return err;
}

#define FELIX_INIT_TIMEOUT	50000
#define FELIX_GCB_RST_SLEEP	100
#define FELIX_SYS_RAMINIT_SLEEP	80

static int felix_gcb_soft_rst_status(struct ocelot *ocelot)
{
	int val;

	regmap_field_read(ocelot->regfields[GCB_SOFT_RST_SWC_RST], &val);
	return val;
}

static int felix_sys_ram_init_status(struct ocelot *ocelot)
{
	return ocelot_read(ocelot, SYS_RAM_INIT);
}

static int felix_init_switch_core(struct ocelot *ocelot)
{
	int val, err;

	/* soft-reset the switch core */
	regmap_field_write(ocelot->regfields[GCB_SOFT_RST_SWC_RST], 1);

	err = readx_poll_timeout(felix_gcb_soft_rst_status, ocelot, val, !val,
				 FELIX_GCB_RST_SLEEP, FELIX_INIT_TIMEOUT);
	if (err) {
		dev_err(ocelot->dev, "timeout: switch core reset\n");
		return err;
	}

	/* initialize switch mem ~40us */
	ocelot_write(ocelot, SYS_RAM_INIT_RAM_INIT, SYS_RAM_INIT);
	err = readx_poll_timeout(felix_sys_ram_init_status, ocelot, val, !val,
				 FELIX_SYS_RAMINIT_SLEEP, FELIX_INIT_TIMEOUT);
	if (err) {
		dev_err(ocelot->dev, "timeout: switch sram init\n");
		return err;
	}

	/* enable switch core */
	regmap_field_write(ocelot->regfields[SYS_RESET_CFG_CORE_ENA], 1);

	return 0;
}

static int felix_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ocelot *ocelot;
	resource_size_t base;
	size_t len;
	int i, err;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	base = pci_resource_start(pdev, FELIX_SWITCH_BAR);

	pci_set_master(pdev);

	ocelot = devm_kzalloc(&pdev->dev, sizeof(*ocelot), GFP_KERNEL);
	if (!ocelot) {
		err = -ENOMEM;
		goto err_alloc_ocelot;
	}

	pci_set_drvdata(pdev, ocelot);
	ocelot->dev = &pdev->dev;

	len = pci_resource_len(pdev, FELIX_SWITCH_BAR);
	if (!len) {
		err = -EINVAL;
		goto err_resource_len;
	}

	regs = pci_iomap(pdev, FELIX_SWITCH_BAR, len);
	if (!regs) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_iomap;
	}

	for (i = 0; i < ARRAY_SIZE(felix_io_res); i++) {
		struct resource *res = &felix_io_res[i].res;
		struct regmap *target;

		res->flags = IORESOURCE_MEM;
		res->start += base;
		res->end += base;

		target = ocelot_io_init(ocelot, res);
		if (IS_ERR(target)) {
			err = PTR_ERR(target);
			goto err_iomap;
		}

		ocelot->targets[felix_io_res[i].id] = target;
	}

	err = felix_chip_init(ocelot);
	if (err)
		goto err_chip_init;

	err = felix_init_switch_core(ocelot);
	if (err)
		goto err_sw_core_init;

	err = felix_ports_init(pdev);
	if (err)
		goto err_ports_init;

	register_netdevice_notifier(&ocelot_netdevice_nb);
	register_switchdev_notifier(&ocelot_switchdev_nb);
	register_switchdev_blocking_notifier(&ocelot_switchdev_blocking_nb);

	dev_info(&pdev->dev, "%s v%s\n", FELIX_DRV_STR, FELIX_DRV_VER_STR);
	return 0;

err_ports_init:
err_chip_init:
err_sw_core_init:
	pci_iounmap(pdev, regs);
err_iomap:
err_resource_len:
err_alloc_ocelot:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void felix_pci_remove(struct pci_dev *pdev)
{
	struct ocelot *ocelot;

	ocelot = pci_get_drvdata(pdev);

	/* stop workqueue thread */
	ocelot_deinit(ocelot);
	unregister_switchdev_blocking_notifier(&ocelot_switchdev_blocking_nb);
	unregister_switchdev_notifier(&ocelot_switchdev_nb);
	unregister_netdevice_notifier(&ocelot_netdevice_nb);

	felix_release_ports(ocelot);

	pci_iounmap(pdev, regs);
	pci_disable_device(pdev);
}

static struct pci_driver felix_pci_driver = {
	.name = KBUILD_MODNAME,
	.id_table = felix_ids,
	.probe = felix_pci_probe,
	.remove = felix_pci_remove,
};

module_pci_driver(felix_pci_driver);

MODULE_DESCRIPTION(FELIX_DRV_STR);
MODULE_LICENSE("Dual MIT/GPL");
