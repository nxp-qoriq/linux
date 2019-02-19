/*
 * Copyright 2018 NXP
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
