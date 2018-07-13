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

#define PABRST				(31)
#define WE				(31)
#define PABR				(27)

#define LX_PCIE_LTSSM_L0		0x2d /* L0 state */

struct lx_pcie_drvdata {
	u32 lut_offset;
	u32 ltssm_shift;
	u32 ltssm_mask;
	u32 lut_dbg;
	u32 pf_int_stat;
	bool lut_big_endian;
	struct mv_pcie_rp_ops *rp_ops;
	const struct mv_pcie_pab_ops *pab_ops;
};

struct lx_pcie {
	struct mv_pcie *pci;
	void __iomem *lut;
	const struct lx_pcie_drvdata *drvdata;
	int			irq;
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
	u32 val;

	/* Set ACK latency timeout */
	val = mv_pcie_readl_csr(mv_pci, GPEX_ACK_REPLAY_TO);
	val &= ~(ACK_LAT_TO_VAL_MASK << ACK_LAT_TO_VAL_SHIFT);
	val |= (4 << ACK_LAT_TO_VAL_SHIFT);
	mv_pcie_writel_csr(mv_pci, GPEX_ACK_REPLAY_TO, val);

	/* Disable all outbound windows configured by bootloader */
	lx_pcie_disable_outbound_wins(pcie);

	mv_pcie_setup_rp_hw(rp, false);
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
	.pf_int_stat = 0x40018,
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

static void lx_pcie_reinit_hw(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	struct root_port *rp = &mv_pci->rp;
	const struct lx_pcie_drvdata *dd = pcie->drvdata;
	u32 val, act_stat;

	/* Poll for pab_csb_reset to clear , PAB activity to set */
	do {
		val = lx_pcie_lut_readl(pcie, dd->pf_int_stat);
		act_stat = mv_pcie_readl_csr(mv_pci, PAB_ACTIVITY_STAT);

	} while (((val & (1 << PABRST)) == 0) || act_stat);

	while (!lx_pcie_link_up(mv_pci))
		;

	/* clear PEX_RESET bit in PEX_PF0_DBG register */
	val = lx_pcie_lut_readl(pcie, dd->lut_dbg);
	val |= 1 << WE;
	lx_pcie_lut_writel(pcie, dd->lut_dbg, val);

	val = lx_pcie_lut_readl(pcie, dd->lut_dbg);
	val |= 1 << PABR;
	lx_pcie_lut_writel(pcie, dd->lut_dbg, val);

	val = lx_pcie_lut_readl(pcie, dd->lut_dbg);
	val &= ~(1 << WE);
	lx_pcie_lut_writel(pcie, dd->lut_dbg, val);

	mv_pcie_setup_rp_hw(rp, true);
}
static irqreturn_t lx_pcie_handler(int irq, void *dev_id)
{
	struct lx_pcie *pcie = (struct lx_pcie *)dev_id;
	struct mv_pcie *mv_pci = pcie->pci;
	u32 val;
	u16 ctrl;

	val = mv_pcie_readl_csr(mv_pci, PAB_INTP_AXI_MISC_STAT);
	if (!val)
		return IRQ_NONE;

	if (val & RESET) {
		ctrl = mv_pcie_readw_csr(mv_pci, PCI_BRIDGE_CONTROL);
		ctrl &= ~PCI_BRIDGE_CTL_BUS_RESET;
		mv_pcie_writew_csr(mv_pci, PCI_BRIDGE_CONTROL, ctrl);
		lx_pcie_reinit_hw(pcie);
	}

	mv_pcie_writel_csr(mv_pci, PAB_INTP_AXI_MISC_STAT, val);

	return IRQ_HANDLED;
}

static int __init lx_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mv_pcie *mv_pci;
	struct lx_pcie *pcie;
	struct resource *csr_base;
	struct device_node *np = dev->of_node;
	int ret;
	u32 val;

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
	mv_pci->csr_base = devm_pci_remap_cfg_resource(dev, csr_base);
	if (IS_ERR(mv_pci->csr_base))
		return PTR_ERR(mv_pci->csr_base);

	pcie->lut = mv_pci->csr_base + pcie->drvdata->lut_offset;

	if (!lx_pcie_is_bridge(pcie))
		return -ENODEV;

	platform_set_drvdata(pdev, pcie);

	pcie->irq = platform_get_irq_byname(pdev, "intr");
	if (pcie->irq < 0) {
		dev_err(&pdev->dev, "Can't get intr irq.\n");
		return pcie->irq;
	}
	ret = devm_request_irq(&pdev->dev, pcie->irq,
			lx_pcie_handler, IRQF_SHARED, pdev->name, pcie);
	if (ret) {
		dev_err(&pdev->dev, "Can't register LX PCIe IRQ.\n");
		return  ret;
	}

	ret = lx_add_root_port(pcie);
	if (ret < 0)
		return ret;

	if ((csr_base->start != 0x3600000) && (csr_base->start != 0x3800000)) {
		/* disable the uncorrectable DLP error report */
		val = mv_pcie_readl_csr(mv_pci, CFG_UNCORRECTABLE_ERROR_MASK);
		val |= (1 << DATA_LINK_PROTOCOL_ERR_SHIFT);
		mv_pcie_writel_csr(mv_pci, CFG_UNCORRECTABLE_ERROR_MASK, val);
	}

	/* disable the correctable RTT error report */
	val = mv_pcie_readl_csr(mv_pci, CFG_CORRECTABLE_ERROR_MASK);
	val |= (1 << REPLAY_TIMER_TIMEOUT_SHIFT);
	mv_pcie_writel_csr(mv_pci, CFG_CORRECTABLE_ERROR_MASK, val);

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
