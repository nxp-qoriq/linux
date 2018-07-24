/*
 * PCIe Endpoint driver for Freescale Layerscape SoCs
 *
 * Copyright 2018 NXP.
 *
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
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

#include "pci-lx-ep.h"

static int lx_pcie_ep_link_up(struct mv_pcie *pci)
{
	u32 state;

	state = mv_pcie_readl_csr(pci, GPEX_LTSSM_STATE_STATUS);
	state =	(state >> LTSSM_STAT_CODE_SHIFT) & LTSSM_STAT_CODE_MASK;

	if (state == LX_PCIE_LTSSM_L0)
		return 1;

	return 0;
}

struct lx_ep_dev *
lx_pci_ep_find(struct lx_pcie *pcie, int dev_id)
{
	struct lx_ep_dev *ep;

	list_for_each_entry(ep, &pcie->ep_list, node) {
		if (ep->dev_id == dev_id)
			return ep;
	}

	return NULL;
}

static bool lx_pcie_is_bridge(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	u32 header_type;

	header_type = mv_pcie_readb_csr(mv_pci, PCI_HEADER_TYPE);
	header_type &= 0x7f;

	return header_type == PCI_HEADER_TYPE_BRIDGE;
}

int lx_pcie_ep_outbound_win_set(struct lx_pcie *pcie, int idx, int type,
				u64 phys, u64 bus_addr, u32 func, u64 size)
{
	u32 val;
	u32 size_h, size_l;
	struct mv_pcie *mv_pci = pcie->pci;

	size_h = upper_32_bits(~((u64)size - 1));
	size_l = lower_32_bits(~((u64)size - 1));

	val = mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_CTRL(idx));
	val &= ~((AXI_AMAP_CTRL_TYPE_MASK << AXI_AMAP_CTRL_TYPE_SHIFT) |
		(AXI_AMAP_CTRL_SIZE_MASK << AXI_AMAP_CTRL_SIZE_SHIFT) |
		AXI_AMAP_CTRL_EN);
	val |= ((type & AXI_AMAP_CTRL_TYPE_MASK) << AXI_AMAP_CTRL_TYPE_SHIFT) |
		((size_l >> AXI_AMAP_CTRL_SIZE_SHIFT) <<
		AXI_AMAP_CTRL_SIZE_SHIFT) | AXI_AMAP_CTRL_EN;

	mv_pcie_writel_csr(mv_pci, PAB_AXI_AMAP_CTRL(idx), val);

	mv_pcie_writel_csr(mv_pci, PAB_AXI_AMAP_PCI_HDR_PARAM(idx), func);
	mv_pcie_writel_csr(mv_pci, PAB_AXI_AMAP_AXI_WIN(idx),
			lower_32_bits(phys));
	mv_pcie_writel_csr(mv_pci, PAB_EXT_AXI_AMAP_AXI_WIN(idx),
			upper_32_bits(phys));
	mv_pcie_writel_csr(mv_pci, PAB_AXI_AMAP_PEX_WIN_L(idx),
			lower_32_bits(bus_addr));
	mv_pcie_writel_csr(mv_pci, PAB_AXI_AMAP_PEX_WIN_H(idx),
			upper_32_bits(bus_addr));
	mv_pcie_writel_csr(mv_pci, PAB_EXT_AXI_AMAP_SIZE(idx), size_h);

	return 0;
}

void lx_pcie_ep_enable_inbound_win(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	int bar, pf;

	if (pcie->sriov) {
		for (pf = 0; pf < PCIE_PF_NUM; pf++) {
			for (bar = 0; bar < PCIE_BAR_NUM_SRIOV; bar++)
				mv_pcie_writel_csr(mv_pci,
						PAB_PEX_BAR_AMAP(pf, bar), 1);
		}
	} else {
		for (bar = 0; bar < PCIE_BAR_NUM; bar++)
			mv_pcie_writel_csr(mv_pci,
					PAB_PEX_BAR_AMAP(0, bar), 1);
	}
}

void lx_pcie_ep_inbound_win_set(struct lx_pcie *pcie, int func,
			int bar, u64 phys)
{
	struct mv_pcie *mv_pci = pcie->pci;

	mv_pcie_writel_csr(mv_pci, PAB_EXT_PEX_BAR_AMAP(func, bar),
			upper_32_bits(phys));
	mv_pcie_writel_csr(mv_pci, PAB_PEX_BAR_AMAP(func, bar),
			lower_32_bits(phys) | 1);
}

static void lx_pcie_ep_setup_wins(struct lx_pcie *pcie, int bar_num, int pf)
{
	int bar;
	dma_addr_t buf_addr;

	for (bar = 0; bar < bar_num; bar++) {
		if (pcie->sriov) {
			if (bar < PCIE_BAR_NUM)
				buf_addr = pcie->buf_addr_pf +
					bar * PCIE_BAR_SIZE +
					pf * PCIE_BAR_NUM * PCIE_BAR_SIZE;
			else
				buf_addr = pcie->buf_addr_vf +
					(bar - PCIE_BAR_NUM) *
					PCIE_BAR_SIZE * PCIE_VF_NUM +
					pf * PCIE_BAR_NUM * PCIE_BAR_SIZE *
					PCIE_VF_NUM;
		} else
			buf_addr = pcie->buf_addr_pf + bar * PCIE_BAR_SIZE +
				pf * PCIE_BAR_NUM * PCIE_BAR_SIZE;

		if ((bar != 1) && (bar != 5))
			lx_pcie_ep_inbound_win_set(pcie, pf, bar, buf_addr);
	}
}

static int lx_pcie_setup_ep(struct lx_pcie *pcie)
{
	u32 pf;

	if (pcie->sriov) {
		pcie->buf_pf = dma_alloc_coherent(pcie->dev,
				PCIE_BAR_SIZE * PCIE_PF_NUM * PCIE_BAR_NUM,
				&pcie->buf_addr_pf,
				GFP_KERNEL);

		if (pcie->buf_pf == NULL) {
			dev_err(pcie->dev, "PF buff alloc failed !\n");
			return -ENOMEM;
		}

		pcie->buf_vf = dma_alloc_coherent(pcie->dev,
				PCIE_BAR_SIZE * PCIE_VF_NUM * PCIE_PF_NUM *
				PCIE_BAR_NUM,
				&pcie->buf_addr_vf,
				GFP_KERNEL);

		if (pcie->buf_vf == NULL) {
			dev_err(pcie->dev, "VF buff alloc failed !\n");
			if (pcie->buf_pf)
				free_pages((unsigned long)pcie->buf_pf,
					PCIE_BAR_SIZE * PCIE_PF_NUM *
					PCIE_BAR_NUM);
			return -ENOMEM;
		}

		for (pf = 0; pf < PCIE_PF_NUM; pf++)
			lx_pcie_ep_setup_wins(pcie, PCIE_BAR_NUM_SRIOV, pf);
	} else {
		pcie->buf_pf = dma_alloc_coherent(pcie->dev,
					PCIE_BAR_SIZE * PCIE_BAR_NUM,
					&pcie->buf_addr_pf,
					GFP_KERNEL);

		if (pcie->buf_pf == NULL) {
			dev_err(pcie->dev, "PF buff alloc failed !\n");
			return -ENOMEM;
		}

		lx_pcie_ep_setup_wins(pcie, PCIE_BAR_NUM, 0);
	}

	return 0;
}

static int lx_pcie_ep_dev_init(struct lx_pcie *pcie, int pf_idx, int vf_idx)
{
	struct lx_ep_dev *ep;

	ep = devm_kzalloc(pcie->dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pcie = pcie;
	ep->pf_idx = pf_idx;
	ep->vf_idx = vf_idx;

	if (vf_idx) {
		ep->dev_id = PCIE_PF_NUM + pf_idx * PCIE_VF_NUM_TOTAL +
				(vf_idx - 1);
		ep->win_idx = PCIE_PF_NUM + pf_idx * PCIE_VF_NUM_ENABLED +
				(vf_idx - 1);
	} else {
		ep->dev_id = pf_idx;
		ep->win_idx = pf_idx;
	}

	if (vf_idx)
		dev_set_name(&ep->dev, "pf%d-vf%d",
			     ep->pf_idx,
			     ep->vf_idx);
	else
		dev_set_name(&ep->dev, "pf%d",
			     ep->pf_idx);

	list_add_tail(&ep->node, &pcie->ep_list);

	return 0;
}

static int lx_pcie_ep_init(struct lx_pcie *pcie)
{
	u32 sriov_header;
	int pf, vf, i, j;
	struct mv_pcie *mv_pci = pcie->pci;
	int ret;

	sriov_header = mv_pcie_readl_csr(mv_pci, PCIE_SRIOV_CAPABILITY);
	if (PCI_EXT_CAP_ID(sriov_header) == PCI_EXT_CAP_ID_SRIOV) {
		pcie->sriov = PCIE_SRIOV_POS;
		pf = PCIE_PF_NUM;
		vf = PCIE_VF_NUM;
	} else {
		pcie->sriov = 0;
		pf = 1;
		vf = 0;
	}

	for (i = 0; i < pf; i++) {
		for (j = 0; j <= vf; j++)
			lx_pcie_ep_dev_init(pcie, i, j);
	}

	ret = lx_pcie_setup_ep(pcie);
	if (ret) {
		dev_err(pcie->dev, "EP device setup failed !\n");
		return -1;
	}

	return 0;
}

static void lx_pcie_ep_enable_ppio_apio_msi(struct mv_pcie *mv_pci)
{
	u32 val;

	val = mv_pcie_readl_csr(mv_pci, PAB_CTRL);
	val |= PAB_CTRL_APIO_EN | PAB_CTRL_PPIO_EN |
		PAB_CTRL_MSI_SW_CTRL_EN;
	mv_pcie_writel_csr(mv_pci, PAB_CTRL, val);

	val = mv_pcie_readl_csr(mv_pci, PAB_AXI_PIO_CTRL(0));
	val |= APIO_EN | MEM_WIN_EN;
	mv_pcie_writel_csr(mv_pci, PAB_AXI_PIO_CTRL(0), val);

	val = mv_pcie_readl_csr(mv_pci, PAB_PEX_PIO_CTRL(0));
	val |= PPIO_EN;
	mv_pcie_writel_csr(mv_pci, PAB_PEX_PIO_CTRL(0), val);

	val =  mv_pcie_readl_csr(mv_pci, PAB_INTP_AXI_MISC_ENB);
	val |= 1 << 0;
	mv_pcie_writel_csr(mv_pci, PAB_INTP_AXI_MISC_ENB, val);
}

static void lx_pcie_ep_reinit_hw(struct lx_pcie *pcie)
{
	struct mv_pcie *mv_pci = pcie->pci;
	const struct lx_pcie_ep_drvdata *dd = pcie->drvdata;
	u32 val;
	int pf;

	/* Poll for pab_csb_reset to clear */
	do
		val = ioread32(pcie->lut + dd->pf_int_stat);
	while ((val & (1 << PABRST)) == 0);

	while (!lx_pcie_ep_link_up(mv_pci))
		;

	/* clear PEX_RESET bit in PEX_PF0_DBG register */
	val = ioread32(pcie->lut + dd->lut_dbg);
	val |= 1 << WE;
	iowrite32(val, pcie->lut + dd->lut_dbg);

	val = ioread32(pcie->lut + dd->lut_dbg);
	val |= 1 << PABR;
	iowrite32(val, pcie->lut + dd->lut_dbg);

	val = ioread32(pcie->lut + dd->lut_dbg);
	val &= ~(1 << WE);
	iowrite32(val, pcie->lut + dd->lut_dbg);

	lx_pcie_ep_enable_ppio_apio_msi(mv_pci);

	lx_pcie_ep_enable_inbound_win(pcie);

	if (pcie->sriov) {
		for (pf = 0; pf < PCIE_PF_NUM; pf++)
			lx_pcie_ep_setup_wins(pcie, PCIE_BAR_NUM_SRIOV, pf);
	} else {
		lx_pcie_ep_setup_wins(pcie, PCIE_BAR_NUM, 0);
	}

}

static irqreturn_t lx_pcie_ep_handler(int irq, void *dev_id)
{
	struct lx_pcie *pcie = (struct lx_pcie *)dev_id;
	struct mv_pcie *mv_pci = pcie->pci;
	u32 val;

	val = mv_pcie_readl_csr(mv_pci, PAB_INTP_AXI_MISC_STAT);
	if (!val)
		return IRQ_NONE;

	if (val & RESET)
		lx_pcie_ep_reinit_hw(pcie);

	mv_pcie_writel_csr(mv_pci, PAB_INTP_AXI_MISC_STAT, val);

	return IRQ_HANDLED;
}

static const struct mv_pcie_pab_ops lx_pcie_pab_ops = {
	.link_up = lx_pcie_ep_link_up,
};

static struct lx_pcie_ep_drvdata lx2160_drvdata = {
	.lut_offset = 0x80000,
	.lut_big_endian = false,
	.lut_dbg = 0x407fc,
	.pf_int_stat = 0x40018,
	.ltssm_shift = 0,
	.ltssm_mask = 0x3f,
	.pab_ops = &lx_pcie_pab_ops,
};

static const struct of_device_id lx_pcie_ep_of_match[] = {
	{ .compatible = "fsl,lx2160a-pcie", .data = &lx2160_drvdata },
	{ },
};
MODULE_DEVICE_TABLE(of, lx_pcie_ep_of_match);

static int lx_pcie_ep_probe(struct platform_device *pdev)
{
	struct lx_pcie *pcie;
	struct mv_pcie *mv_pci;
	struct resource *csr_base, *cfg_res;
	const struct of_device_id *match;
	int ret, val;

	match = of_match_device(lx_pcie_ep_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	mv_pci = devm_kzalloc(&pdev->dev, sizeof(*mv_pci), GFP_KERNEL);
	if (!mv_pci)
		return -ENOMEM;

	pcie->pci = mv_pci;
	pcie->dev = &pdev->dev;
	INIT_LIST_HEAD(&pcie->ep_list);

	csr_base = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	mv_pci->csr_base = devm_ioremap_resource(&pdev->dev, csr_base);
	if (IS_ERR(mv_pci->csr_base))
		return PTR_ERR(mv_pci->csr_base);

	pcie->drvdata = match->data;
	mv_pci->ops = pcie->drvdata->pab_ops;
	pcie->lut = mv_pci->csr_base + pcie->drvdata->lut_offset;

	pcie->irq = platform_get_irq_byname(pdev, "intr");
	if (pcie->irq < 0) {
		dev_err(&pdev->dev, "Can't get intr irq.\n");
		return pcie->irq;
	}
	ret = devm_request_irq(&pdev->dev, pcie->irq,
			lx_pcie_ep_handler, IRQF_SHARED, pdev->name, pcie);
	if (ret) {
		dev_err(&pdev->dev, "Can't register LX PCIe IRQ.\n");
		return  ret;
	}

	cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	if (cfg_res)
		pcie->out_base = cfg_res->start;
	else {
		dev_err(&pdev->dev, "missing *config* space\n");
		return -ENODEV;
	}

	if (lx_pcie_is_bridge(pcie))
		return -ENODEV;

	dev_info(pcie->dev, "in EP mode\n");

	val =  mv_pcie_readl_csr(mv_pci, PAB_CTRL);
	val |= PAB_CTRL_MSI_SW_CTRL_EN;
	mv_pcie_writel_csr(mv_pci, PAB_CTRL, val);

	val =  mv_pcie_readl_csr(mv_pci, PAB_INTP_AXI_MISC_ENB);
	val |= 1 << 0;
	mv_pcie_writel_csr(mv_pci, PAB_INTP_AXI_MISC_ENB, val);

	ret = lx_pcie_ep_init(pcie);
	if (ret)
		return ret;

	lx_pcie_ep_dbgfs_init(pcie);

	platform_set_drvdata(pdev, pcie);

	return 0;
}

static int lx_pcie_ep_dev_remove(struct lx_ep_dev *ep)
{
	list_del(&ep->node);

	return 0;
}

static int lx_pcie_ep_remove(struct platform_device *pdev)
{
	struct lx_pcie *pcie = platform_get_drvdata(pdev);
	struct lx_ep_dev *ep, *tmp;

	if (!pcie)
		return 0;

	lx_pcie_ep_dbgfs_remove(pcie);

	if (pcie->buf_pf)
		free_pages((unsigned long)pcie->buf_pf,
			PCIE_BAR_SIZE * PCIE_PF_NUM * PCIE_BAR_NUM);

	if (pcie->buf_vf)
		free_pages((unsigned long)pcie->buf_vf,
				PCIE_BAR_SIZE * PCIE_VF_NUM * PCIE_PF_NUM *
				PCIE_BAR_NUM);

	list_for_each_entry_safe(ep, tmp, &pcie->ep_list, node)
		lx_pcie_ep_dev_remove(ep);

	return 0;
}

static struct platform_driver lx_pcie_ep_driver = {
	.driver = {
		.name = "lx-pcie-ep",
		.owner = THIS_MODULE,
		.of_match_table = lx_pcie_ep_of_match,
	},
	.probe = lx_pcie_ep_probe,
	.remove = lx_pcie_ep_remove,
};

module_platform_driver(lx_pcie_ep_driver);

MODULE_AUTHOR("Xiaowei Bao <xiaowei.bao@nxp.com>");
MODULE_DESCRIPTION("Freescale Layerscape PCIe EP driver");
MODULE_LICENSE("GPL v2");
