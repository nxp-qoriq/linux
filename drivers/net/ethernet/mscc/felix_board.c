// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix Switch driver
 *
 * Copyright (c) 2018 NXP
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/phy_fixed.h>
#include <linux/phy.h>

#include "ocelot.h"

static const char felix_driver_string[] = "Felix Switch Driver";
#define DRV_VERSION "0.2"
static const char felix_driver_version[] = DRV_VERSION;

#define NUM_PHY_PORTS		6
#define FELIX_EXT_CPU_PORT	5
#define PORT_RES_START		(SYS + 1)

#define PCI_DEVICE_ID_FELIX_PF5	0xEEF0

/* Switch register block BAR */
#define FELIX_SWITCH_BAR	4

static struct pci_device_id felix_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_FELIX_PF5) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, felix_ids);

/* Mimic the order of ocelot_target */
static struct resource felix_switch_res[] = {
	{
		/* Nothing here */
	},
	{
		.start = 0x0280000,
		.end = 0x028ffff,
		.name = "ana",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0080000,
		.end = 0x00800ff,
		.name = "qs",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0200000,
		.end = 0x021ffff,
		.name = "qsys",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0030000,
		.end = 0x003ffff,
		.name = "rew",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0010000,
		.end = 0x001ffff,
		.name = "sys",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0100000,
		.end = 0x010ffff,
		.name = "port0",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0110000,
		.end = 0x011ffff,
		.name = "port1",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0120000,
		.end = 0x012ffff,
		.name = "port2",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0130000,
		.end = 0x013ffff,
		.name = "port3",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0140000,
		.end = 0x014ffff,
		.name = "port4",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = 0x0150000,
		.end = 0x015ffff,
		.name = "port5",
		.flags = IORESOURCE_MEM,
	},
};

static void __iomem *regs;

int felix_chip_init(struct ocelot *ocelot);

static struct regmap *felix_io_init(struct ocelot *ocelot, u8 target)
{
	void __iomem *target_regs;
	struct regmap_config felix_regmap_config = {
		.reg_bits	= 32,
		.val_bits	= 32,
		.reg_stride	= 4,
	};

	felix_regmap_config.name = felix_switch_res[target].name;
	target_regs = devm_ioremap_resource(ocelot->dev,
					    &felix_switch_res[target]);
	if (IS_ERR(target_regs))
		return ERR_CAST(target_regs);

	return devm_regmap_init_mmio(ocelot->dev, target_regs,
				     &felix_regmap_config);
}

static struct phy_device *felix_fixed_phy_register(struct device *dev)
{
	struct phy_device *fixed_phy;
	struct fixed_phy_status status = {
			.link = 1,
			.speed = 1000,
			.duplex = 1,
	};

	fixed_phy = fixed_phy_register(PHY_POLL, &status, -1, NULL);
	if (!fixed_phy || IS_ERR(fixed_phy)) {
		dev_err(dev, "error trying to register fixed PHY\n");
		fixed_phy = NULL;
	}
	phy_start(fixed_phy);

	return fixed_phy;
}

static int felix_ports_init(struct ocelot *ocelot)
{
	struct phy_device *fixed_phy;
	void __iomem *port_regs;
	int port;
	int err;

	for (port = 0; port < ocelot->num_phys_ports; port++) {
		port_regs = devm_ioremap_resource(ocelot->dev,
				&felix_switch_res[PORT_RES_START + port]);
		if (IS_ERR(port_regs)) {
			dev_err(ocelot->dev,
				"failed to map registers for port %d\n", port);
			goto release_ports;
		}

		fixed_phy = felix_fixed_phy_register(ocelot->dev);
		if (!fixed_phy)
			goto release_ports;

		err = ocelot_probe_port(ocelot, port, port_regs, fixed_phy);
		if (err) {
			dev_err(ocelot->dev, "failed to probe port %d\n", port);
			goto release_ports;
		}
	}

	return 0;

release_ports:
	for (port--; port >= 0; port--) {
		unregister_netdev(ocelot->ports[port]->dev);
		free_netdev(ocelot->ports[port]->dev);
	}

	return err;
}

static int felix_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ocelot *ocelot;
	resource_size_t offset;
	size_t len;
	int timeout;
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

	offset = pci_resource_start(pdev, FELIX_SWITCH_BAR);

	pci_set_master(pdev);

	ocelot = kzalloc(sizeof(*ocelot), GFP_KERNEL);
	if (!ocelot) {
		err = -ENOMEM;
		goto err_alloc_ocelot;
	}

	pci_set_drvdata(pdev, ocelot);
	ocelot->dev = &pdev->dev;

	len = pci_resource_len(pdev, FELIX_SWITCH_BAR);
	if (len == 0) {
		err = -EINVAL;
		goto err_resource_len;
	}

	regs = pci_iomap(pdev, FELIX_SWITCH_BAR, len);
	if (!regs) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_iomap;
	}

	for (i = 0; i < ARRAY_SIZE(felix_switch_res); i++)
		if (felix_switch_res[i].flags == IORESOURCE_MEM) {
			felix_switch_res[i].start += offset;
			felix_switch_res[i].end += offset;
		}

	for (i = ANA; i <= SYS; i++) {
		struct regmap *target;

		target = felix_io_init(ocelot, i);
		if (IS_ERR(target))
			return PTR_ERR(target);

		ocelot->targets[i] = target;
	}

	err = felix_chip_init(ocelot);
	if (err)
		goto err_chip_init;

	ocelot_write(ocelot, SYS_RAM_INIT_RAM_INIT, SYS_RAM_INIT);

	timeout = 50000;
	while (ocelot_read(ocelot, SYS_RAM_INIT) && --timeout)
		udelay(1); /* busy wait for memory init */
	if (timeout == 0)
		dev_err(&pdev->dev, "Timeout waiting for memory to initialize\n");

	regmap_field_write(ocelot->regfields[SYS_RESET_CFG_CORE_ENA], 1);

	ocelot->num_cpu_ports = 1; /* 1 port on the switch, two groups */
	ocelot->num_phys_ports = NUM_PHY_PORTS;
	ocelot->ports = devm_kcalloc(&pdev->dev, ocelot->num_phys_ports,
				     sizeof(struct ocelot_port *), GFP_KERNEL);

	ocelot_init(ocelot);

	err = felix_ports_init(ocelot);
	if (err)
		goto err_ports_init;

	/* set port for external CPU frame extraction/injection */
	ocelot_write(ocelot, QSYS_EXT_CPU_CFG_EXT_CPUQ_MSK_M |
		     QSYS_EXT_CPU_CFG_EXT_CPU_PORT(FELIX_EXT_CPU_PORT),
		     QSYS_EXT_CPU_CFG);

	register_netdevice_notifier(&ocelot_netdevice_nb);

	dev_info(&pdev->dev, "%s - version %s probed\n", felix_driver_string,
		 felix_driver_version);
	return 0;

err_ports_init:
err_chip_init:
	pci_iounmap(pdev, regs);
err_iomap:
err_resource_len:
	kfree(ocelot);
err_alloc_ocelot:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void felix_pci_remove(struct pci_dev *pdev)
{
	struct ocelot *ocelot;

	unregister_netdevice_notifier(&ocelot_netdevice_nb);

	ocelot = pci_get_drvdata(pdev);

	ocelot_deinit(ocelot);
	pci_iounmap(pdev, regs);
	kfree(ocelot);
	pci_disable_device(pdev);
	pr_debug("%s - version %s removed\n", felix_driver_string,
		 felix_driver_version);
}

static struct pci_driver felix_pci_driver = {
	.name = "mscc_felix",
	.id_table = felix_ids,
	.probe = felix_pci_probe,
	.remove = felix_pci_remove,
};

module_pci_driver(felix_pci_driver);

MODULE_DESCRIPTION("Felix switch driver");
MODULE_AUTHOR("Razvan Stefanescu <razvan.stefanescu@nxp.com>");
MODULE_LICENSE("Dual MIT/GPL");
