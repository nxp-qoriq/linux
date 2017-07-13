/*
 * PCIe host controller driver for NXP LX SoCs
 *
 * Copyright 2018 NXP
 *
 * Author: Hou Zhiqiang <Minder.Hou@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "pcie-mobiveil.h"

/* LUT registers */
#define PCIE_LUT_GCR			(0x28)
#define PCIE_LUT_GCR_RRE		(0)

#define LX_PCIE_LTSSM_L0		0x2d /* L0 state */

struct lx_pcie_drvdata {
	u32 lut_offset;
	u32 ltssm_shift;
	u32 ltssm_mask;
	u32 lut_dbg;
	bool lut_big_endian;
	struct mv_pcie_rp_ops *rp_ops;
	const struct mv_pcie_pab_ops *pab_ops;
};

struct lx_pcie {
	struct mv_pcie *pci;
	void __iomem *lut;
	const struct lx_pcie_drvdata *drvdata;
};

#define to_lx_pcie(x)	dev_get_drvdata((x)->dev)

static inline u32 lx_pcie_lut_readl(struct lx_pcie *pcie, u32 off)
{
	if (pcie->drvdata->lut_big_endian)
		return ioread32be(pcie->lut + off);
	else
		return ioread32(pcie->lut + off);
}

static inline void lx_pcie_lut_writel(struct lx_pcie *pcie, u32 off, u32 val)
{
	if (pcie->drvdata->lut_big_endian)
		iowrite32be(val, pcie->lut + off);
	else
		iowrite32(val, pcie->lut + off);
}

static bool lx_pcie_is_bridge(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	u32 header_type;

	header_type = mv_pcie_readb_csr(mv_pci, PCI_HEADER_TYPE);
	header_type &= 0x7f;

	return header_type == PCI_HEADER_TYPE_BRIDGE;
}

static int lx_pcie_link_up(struct mv_pcie *pci)
{
	struct lx_pcie *pcie = to_lx_pcie(pci);
	const struct lx_pcie_drvdata *dd = pcie->drvdata;
	u32 state;

	state = lx_pcie_lut_readl(pcie, dd->lut_dbg);
	state =	(state >> dd->ltssm_shift) & dd->ltssm_mask;

	if (state == LX_PCIE_LTSSM_L0)
		return 1;

	return 0;
}

static void lx_pcie_disable_outbound_wins(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	int i;

	for (i = 0; i < mv_pci->apio_wins; i++)
		mv_pcie_disable_outbound_win(mv_pci, i);
}

static void lx_pcie_init(struct root_port *rp)
{
	struct mv_pcie *mv_pci = rp_to_mv_pcie(rp);
	struct lx_pcie *pcie = to_lx_pcie(mv_pci);

	/* Disable all outbound windows configured by bootloader */
	lx_pcie_disable_outbound_wins(pcie);

	mv_pcie_setup_rp_hw(rp);
	mv_pcie_wait_for_link(mv_pci);
}

static int lx_pcie_read_other_conf(struct root_port *rp, int where,
				   int size, u32 *val)
{
	struct mv_pcie *mv_pci = rp_to_mv_pcie(rp);
	struct lx_pcie *pcie = to_lx_pcie(mv_pci);
	int ret;

	if (where == PCI_VENDOR_ID)
		lx_pcie_lut_writel(pcie, PCIE_LUT_GCR, 0 << PCIE_LUT_GCR_RRE);

	ret = mv_pcie_read(rp->va_cfg_base + where, size, val);

	if (where == PCI_VENDOR_ID)
		lx_pcie_lut_writel(pcie, PCIE_LUT_GCR, 1 << PCIE_LUT_GCR_RRE);

	return ret;
}

static struct mv_pcie_rp_ops lx_pcie_rp_ops = {
	.init = lx_pcie_init,
	.read_other_conf = lx_pcie_read_other_conf,
};

static const struct mv_pcie_pab_ops lx_pcie_pab_ops = {
	.link_up = lx_pcie_link_up,
};

static struct lx_pcie_drvdata lx2160_drvdata = {
	.lut_offset = 0x80000,
	.lut_big_endian = false,
	.lut_dbg = 0x407fc,
	.ltssm_shift = 0,
	.ltssm_mask = 0x3f,
	.rp_ops = &lx_pcie_rp_ops,
	.pab_ops = &lx_pcie_pab_ops,
};

static const struct of_device_id lx_pcie_of_match[] = {
	{ .compatible = "fsl,lx2160a-pcie", .data = &lx2160_drvdata },
	{ },
};

static int __init lx_add_root_port(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	struct root_port *rp = &mv_pci->rp;
	struct device *dev = mv_pci->dev;
	int ret;

	rp->ops = pcie->drvdata->rp_ops;

	ret = mv_pcie_init_rp(rp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int __init lx_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mv_pcie *mv_pci;
	struct lx_pcie *pcie;
	struct resource *csr_base;
	struct device_node *np = dev->of_node;
	int ret;

	if (!of_parse_phandle(np, "msi-parent", 0)) {
		dev_err(dev, "failed to find msi-parent\n");
		return -EINVAL;
	}

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	mv_pci = devm_kzalloc(dev, sizeof(*mv_pci), GFP_KERNEL);
	if (!mv_pci)
		return -ENOMEM;

	pcie->drvdata = of_device_get_match_data(dev);

	mv_pci->dev = dev;
	mv_pci->ops = pcie->drvdata->pab_ops;

	pcie->pci = mv_pci;

	csr_base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	mv_pci->csr_base = devm_ioremap_resource(dev, csr_base);
	if (IS_ERR(mv_pci->csr_base))
		return PTR_ERR(mv_pci->csr_base);

	pcie->lut = mv_pci->csr_base + pcie->drvdata->lut_offset;

	if (!lx_pcie_is_bridge(pcie))
		return -ENODEV;

	platform_set_drvdata(pdev, pcie);

	ret = lx_add_root_port(pcie);
	if (ret < 0)
		return ret;

	return 0;
}

static struct platform_driver lx_pcie_driver = {
	.driver = {
		.name = "lx-pcie",
		.of_match_table = lx_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver_probe(lx_pcie_driver, lx_pcie_probe);
