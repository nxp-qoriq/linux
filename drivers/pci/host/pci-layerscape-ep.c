/*
 * PCIe Endpoint driver for Freescale Layerscape SoCs
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
  * Author: Minghuan Lian <Minghuan.Lian@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include "pci-layerscape-ep.h"

struct ls_ep_dev *
ls_pci_ep_find(struct ls_pcie *pcie, int dev_id)
{
	struct ls_ep_dev *ep;

	list_for_each_entry(ep, &pcie->ep_list, node) {
		if (ep->dev_id == dev_id)
			return ep;
	}

	return NULL;
}

static void ls_pcie_try_cfg2(struct ls_pcie *pcie, int pf, int vf)
{
	if (pcie->sriov)
		writel(PCIE_LCTRL0_VAL(pf, vf),
		       pcie->dbi + PCIE_LUT_BASE + PCIE_LUT_LCTRL0);
}

static bool ls_pcie_is_bridge(struct ls_pcie *pcie)
{
	u32 header_type = 0;

	header_type = readl(pcie->dbi + (PCI_HEADER_TYPE & ~0x3));
	header_type = (header_type >> 16) & 0x7f;

	return header_type == PCI_HEADER_TYPE_BRIDGE;
}

void ls_pcie_iatu_outbound_set(struct ls_pcie *pcie, int idx, int type,
			       u64 cpu_addr, u64 pci_addr, u32 size)
{
	writel(PCIE_ATU_REGION_OUTBOUND | idx,
	       pcie->dbi + PCIE_ATU_VIEWPORT);
	writel(lower_32_bits(cpu_addr),
	       pcie->dbi +  PCIE_ATU_LOWER_BASE);
	writel(upper_32_bits(cpu_addr),
	       pcie->dbi + PCIE_ATU_UPPER_BASE);
	writel(lower_32_bits(cpu_addr + size - 1),
	       pcie->dbi + PCIE_ATU_LIMIT);
	writel(lower_32_bits(pci_addr),
	       pcie->dbi + PCIE_ATU_LOWER_TARGET);
	writel(upper_32_bits(pci_addr),
	       pcie->dbi + PCIE_ATU_UPPER_TARGET);
	writel(type, pcie->dbi + PCIE_ATU_CR1);
	writel(PCIE_ATU_ENABLE, pcie->dbi + PCIE_ATU_CR2);
}

/* Use bar match mode and MEM type as default */
void ls_pcie_iatu_inbound_set(struct ls_pcie *pcie, int idx,
				     int bar, u64 phys)
{
	writel(PCIE_ATU_REGION_INBOUND | idx, pcie->dbi + PCIE_ATU_VIEWPORT);
	writel((u32)phys, pcie->dbi + PCIE_ATU_LOWER_TARGET);
	writel(phys >> 32, pcie->dbi + PCIE_ATU_UPPER_TARGET);
	writel(PCIE_ATU_TYPE_MEM, pcie->dbi + PCIE_ATU_CR1);
	writel(PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE |
	       PCIE_ATU_BAR_NUM(bar), pcie->dbi + PCIE_ATU_CR2);
}

void ls_pcie_ep_dev_cfg_enable(struct ls_ep_dev *ep)
{
	ls_pcie_try_cfg2(ep->pcie, ep->pf_idx, ep->vf_idx);
}

void ls_pcie_ep_setup_bar(void *bar_base, int bar, u32 size)
{
	if (size < 4 * 1024)
		return;

	switch (bar) {
	case 0:
		writel(size - 1, bar_base + PCI_BASE_ADDRESS_0);
		break;
	case 1:
		writel(size - 1, bar_base + PCI_BASE_ADDRESS_1);
		break;
	case 2:
		writel(size - 1, bar_base + PCI_BASE_ADDRESS_2);
		writel(0, bar_base + PCI_BASE_ADDRESS_3);
		break;
	case 4:
		writel(size - 1, bar_base + PCI_BASE_ADDRESS_4);
		writel(0, bar_base + PCI_BASE_ADDRESS_5);
		break;
	default:
		break;
	}
}

void ls_pcie_ep_dev_setup_bar(struct ls_ep_dev *ep, int bar, u32 size)
{
	struct ls_pcie *pcie = ep->pcie;
	void *bar_base;

	if (size < 4 * 1024)
		return;

	if (pcie->sriov)
		bar_base = pcie->dbi;
	else
		bar_base = pcie->dbi + PCIE_NO_SRIOV_BAR_BASE;

	ls_pcie_ep_dev_cfg_enable(ep);
	ls_pcie_ep_setup_bar(bar_base, bar, size);
}

static int ls_pcie_ep_dev_init(struct ls_pcie *pcie, int pf_idx, int vf_idx)
{
	struct ls_ep_dev *ep;

	ep = devm_kzalloc(pcie->dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pcie = pcie;
	ep->pf_idx = pf_idx;
	ep->vf_idx = vf_idx;
	if (vf_idx)
		ep->dev_id = pf_idx + 4 + 4 * (vf_idx - 1);
	else
		ep->dev_id = pf_idx;

	if (ep->vf_idx)
		dev_set_name(&ep->dev, "pf%d-vf%d",
			     ep->pf_idx,
			     ep->vf_idx);
	else
		dev_set_name(&ep->dev, "pf%d",
			     ep->pf_idx);

	list_add_tail(&ep->node, &pcie->ep_list);

	return 0;
}

static int ls_pcie_ep_init(struct ls_pcie *pcie)
{
	u32 sriov_header;
	int pf, vf, i, j;

	sriov_header = readl(pcie->dbi + PCIE_SRIOV_POS);

	if (PCI_EXT_CAP_ID(sriov_header) == PCI_EXT_CAP_ID_SRIOV) {
		pcie->sriov = PCIE_SRIOV_POS;
		pf = PCIE_PF_NUM;
		vf = PCIE_VF_NUM;
	} else {
		pcie->sriov = 0;
		pf = 0;
		vf = 0;
	}

	for (i = 0; i < pf; i++) {
		for (j = 0; j <= vf; j++)
			ls_pcie_ep_dev_init(pcie, i, j);
	}

	return 0;
}

static int ls_pcie_ep_probe(struct platform_device *pdev)
{
	struct ls_pcie *pcie;
	struct resource *dbi_base, *cfg_res;
	int ret;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;
	INIT_LIST_HEAD(&pcie->ep_list);

	dbi_base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	pcie->dbi = devm_ioremap_resource(&pdev->dev, dbi_base);
	if (IS_ERR(pcie->dbi)) {
		dev_err(&pdev->dev, "missing *regs* space\n");
		return PTR_ERR(pcie->dbi);
	}

	pcie->lut = pcie->dbi + PCIE_LUT_BASE;

	if (ls_pcie_is_bridge(pcie))
		return -ENODEV;

	dev_info(pcie->dev, "in EP mode\n");

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (cfg_res)
		pcie->out_base = cfg_res->start;
	else {
		dev_err(&pdev->dev, "missing *config* space\n");
		return -ENODEV;
	}

	ret = ls_pcie_ep_init(pcie);
	if (ret)
		return ret;

	ls_pcie_ep_dbgfs_init(pcie);

	platform_set_drvdata(pdev, pcie);

	return 0;
}

static int ls_pcie_ep_dev_remove(struct ls_ep_dev *ep)
{
	list_del(&ep->node);

	return 0;
}

static int ls_pcie_ep_remove(struct platform_device *pdev)
{
	struct ls_pcie *pcie = platform_get_drvdata(pdev);
	struct ls_ep_dev *ep, *tmp;

	if (!pcie)
		return 0;

	ls_pcie_ep_dbgfs_remove(pcie);

	list_for_each_entry_safe(ep, tmp, &pcie->ep_list, node)
		ls_pcie_ep_dev_remove(ep);

	return 0;
}

static const struct of_device_id ls_pcie_ep_of_match[] = {
	{ .compatible = "fsl,ls2085a-pcie" },
	{ .compatible = "fsl,ls2080a-pcie" },
	{ },
};
MODULE_DEVICE_TABLE(of, ls_pcie_ep_of_match);

static struct platform_driver ls_pcie_ep_driver = {
	.driver = {
		.name = "ls-pcie-ep",
		.owner = THIS_MODULE,
		.of_match_table = ls_pcie_ep_of_match,
	},
	.probe = ls_pcie_ep_probe,
	.remove = ls_pcie_ep_remove,
};

module_platform_driver(ls_pcie_ep_driver);

MODULE_AUTHOR("Minghuan Lian <Minghuan.Lian@freescale.com>");
MODULE_DESCRIPTION("Freescale Layerscape PCIe EP driver");
MODULE_LICENSE("GPL v2");
