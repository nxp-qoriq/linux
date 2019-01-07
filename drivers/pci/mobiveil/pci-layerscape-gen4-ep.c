// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe controller EP driver for Freescale Layerscape SoCs
 *
 * Copyright (C) 2018 NXP Semiconductor.
 *
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include "pcie-mobiveil.h"

struct ls_pcie_g4_ep {
	struct mobiveil_pcie		*pci;
};

#define to_ls_pcie_g4_ep(x)	dev_get_drvdata((x)->dev)

static const struct of_device_id ls_pcie_g4_ep_of_match[] = {
	{ .compatible = "fsl,lx2160a-pcie-ep",},
	{ },
};

static void ls_pcie_g4_get_bar_num(struct mobiveil_pcie_ep *ep)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	u32 type, reg;
	u8 bar;

	ep->bar_num = BAR_5 + 1;

	for (bar = BAR_0; bar <= BAR_5; bar++) {
		reg = PCI_BASE_ADDRESS_0 + (4 * bar);
		type = csr_readl(pci, reg) &
			PCI_BASE_ADDRESS_MEM_TYPE_MASK;
		if (type & PCI_BASE_ADDRESS_MEM_TYPE_64)
			ep->bar_num--;
	}
}

static void ls_pcie_g4_ep_init(struct mobiveil_pcie_ep *ep)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	enum pci_barno bar;
	int i, val;

	/*
	 * Errata: unsupported request error on inbound posted write
	 * transaction, PCIe controller reports advisory error instead
	 * of uncorrectable error message to RC.
	 * workaround: set the bit20(unsupported_request_Error_severity) with
	 * value 1 in uncorrectable_Error_Severity_Register, make the
	 * unsupported request error generate the fatal error.
	 */
	val =  csr_readl(pci, CFG_UNCORRECTABLE_ERROR_SEVERITY);
	val |= 1 << UNSUPPORTED_REQUEST_ERROR_SHIFT;
	csr_writel(pci, val, CFG_UNCORRECTABLE_ERROR_SEVERITY);

	ls_pcie_g4_get_bar_num(ep);

	for (bar = BAR_0; bar < (ep->bar_num * ep->pf_num); bar++)
		mobiveil_pcie_ep_reset_bar(pci, bar);

	for (i = 0; i < MAX_IATU_OUT; i++)
		mobiveil_pcie_disable_ob_win(pci, i);
}

static int ls_pcie_g4_ep_raise_irq(struct mobiveil_pcie_ep *ep,
				   enum pci_epc_irq_type type,
				   u8 interrupt_num)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return mobiveil_pcie_ep_raise_legacy_irq(ep);
	case PCI_EPC_IRQ_MSI:
		return mobiveil_pcie_ep_raise_msi_irq(ep,
						      interrupt_num);
	default:
		dev_err(&pci->pdev->dev, "UNKNOWN IRQ type\n");
	}

	return 0;
}

static struct mobiveil_pcie_ep_ops pcie_ep_ops = {
	.ep_init = ls_pcie_g4_ep_init,
	.raise_irq = ls_pcie_g4_ep_raise_irq,
};

static int __init ls_pcie_gen4_add_pcie_ep(struct ls_pcie_g4_ep *pcie,
					struct platform_device *pdev)
{
	struct mobiveil_pcie *pci = pcie->pci;
	struct device *dev = &pdev->dev;
	struct mobiveil_pcie_ep *ep;
	struct resource *res;
	int ret;
	struct device_node *np = dev->of_node;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ret = of_property_read_u32(np, "max-functions", &ep->pf_num);
	if (ret < 0)
		ep->pf_num = 1;

	ret = mobiveil_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}

	return 0;
}

static int __init ls_pcie_g4_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mobiveil_pcie *pci;
	struct ls_pcie_g4_ep *pcie;
	struct resource *res;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	pci->csr_axi_slave_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->csr_axi_slave_base))
		return PTR_ERR(pci->csr_axi_slave_base);
	pci->pcie_reg_base = res->start;

	pci->pdev = pdev;
	pcie->pci = pci;

	platform_set_drvdata(pdev, pcie);

	ret = ls_pcie_gen4_add_pcie_ep(pcie, pdev);

	return ret;
}

static struct platform_driver ls_pcie_g4_ep_driver = {
	.driver = {
		.name = "layerscape-pcie-gen4-ep",
		.of_match_table = ls_pcie_g4_ep_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver_probe(ls_pcie_g4_ep_driver, ls_pcie_g4_ep_probe);
