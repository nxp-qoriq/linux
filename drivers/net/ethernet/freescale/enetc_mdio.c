// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2019 NXP */

#include <linux/module.h>
#include <linux/pci.h>
#include "xgmac_mdio.h"

#define ENETC_MDIO_DEV_ID	0xee01
#define ENETC_DRV_NAME_STR "ENETC MDIO driver"

static int enetc_pci_mdio_probe(struct pci_dev *pdev,
				const struct pci_device_id *ent)
{
	const struct xgmac_mdio_cfg cfg = {
		.bus_name = "Freescale PCI ENETC-RCIE MDIO Bus",
		.regs_offset = 0x1C00,
	};
	int err;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);

	err = xgmac_mdio_probe(&pdev->dev, &pdev->resource[0], &cfg);
	if (err)
		goto err_xgmac_probe;

	return 0;

err_xgmac_probe:
	pci_release_mem_regions(pdev);
err_pci_mem_reg:
	pci_disable_device(pdev);

	return err;
}

static void enetc_pci_mdio_remove(struct pci_dev *pdev)
{
	xgmac_mdio_remove(dev_get_drvdata(&pdev->dev));
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id enetc_pci_mdio_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_MDIO_DEV_ID) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_mdio_id_table);

static struct pci_driver enetc_pci_mdio_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_pci_mdio_id_table,
	.probe = enetc_pci_mdio_probe,
	.remove = enetc_pci_mdio_remove,
};
module_pci_driver(enetc_pci_mdio_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
