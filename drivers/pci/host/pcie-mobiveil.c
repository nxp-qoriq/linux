/*
 * Mobiveil PCIe host controller driver
 *
 * Copyright 2018 NXP
 *
 * Author: Hou Zhiqiang <Minder.Hou@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/types.h>

#include "pcie-mobiveil.h"

static void mv_pcie_set_page(void __iomem *base, u8 pg_idx)
{
	u32 val;

	val = readl(base + PAB_CTRL);
	val &= ~(PAB_CTRL_PAGE_SEL_MASK << PAB_CTRL_PAGE_SEL_SHIFT);
	val |= (pg_idx & PAB_CTRL_PAGE_SEL_MASK) << PAB_CTRL_PAGE_SEL_SHIFT;

	writel(val, base + PAB_CTRL);
}

static void *mv_pcie_comp_addr(void __iomem *base, u32 off)
{
	if (off < INDIRECT_ADDR_BNDRY) {
		mv_pcie_set_page(base, 0);
		return base + off;
	}

	mv_pcie_set_page(base, OFFSET_TO_PAGE_IDX(off));
	return base + OFFSET_TO_PAGE_ADDR(off);
}

int mv_pcie_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	switch (size) {
	case 4:
		*val = readl(addr);
		break;
	case 2:
		*val = readw(addr);
		break;
	case 1:
		*val = readb(addr);
		break;
	default:
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

int mv_pcie_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	switch (size) {
	case 4:
		writel(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	case 1:
		writeb(val, addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

u32 mv_pcie_read_csr(struct mv_pcie *pci, void __iomem *base, u32 reg,
		       size_t size)
{
	void *addr;
	u32 val;
	int ret;

	if (pci->ops->read_csr)
		return pci->ops->read_csr(pci, base, reg, size);

	addr = mv_pcie_comp_addr(base, reg);

	ret = mv_pcie_read(addr, size, &val);
	if (ret)
		dev_err(pci->dev, "read CSR address failed\n");

	return val;
}

void mv_pcie_write_csr(struct mv_pcie *pci, void __iomem *base, u32 reg,
			 size_t size, u32 val)
{
	void *addr;
	int ret;

	if (pci->ops->write_csr) {
		pci->ops->write_csr(pci, base, reg, size, val);
		return;
	}

	addr = mv_pcie_comp_addr(base, reg);

	ret = mv_pcie_write(addr, size, val);
	if (ret)
		dev_err(pci->dev, "write CSR address failed\n");
}

int mv_pcie_outbound_win_setup(struct mv_pcie *pci, int idx, int type,
			       u64 cpu_addr, u64 pci_addr, u64 size)
{
	u32 val;
	u32 size_l, size_h;

	if (pci->ops->outbound_win_setup)
		return pci->ops->outbound_win_setup(pci, idx, type, cpu_addr,
						    pci_addr, size);
	if (idx > pci->apio_wins)
		return -EINVAL;

	size_l = lower_32_bits(~(size - 1));
	size_h = upper_32_bits(~(size - 1));

	mv_pcie_writel_csr(pci, PAB_AXI_AMAP_AXI_WIN(idx),
			   lower_32_bits(cpu_addr));
	mv_pcie_writel_csr(pci, PAB_EXT_AXI_AMAP_AXI_WIN(idx),
			   upper_32_bits(cpu_addr));
	mv_pcie_writel_csr(pci, PAB_AXI_AMAP_PEX_WIN_L(idx),
			   lower_32_bits(pci_addr));
	mv_pcie_writel_csr(pci, PAB_AXI_AMAP_PEX_WIN_H(idx),
			   upper_32_bits(pci_addr));
	mv_pcie_writel_csr(pci, PAB_EXT_AXI_AMAP_SIZE(idx), size_h);

	val = mv_pcie_readl_csr(pci, PAB_AXI_AMAP_CTRL(idx));
	val &= ~((AXI_AMAP_CTRL_TYPE_MASK << AXI_AMAP_CTRL_TYPE_SHIFT) |
		(AXI_AMAP_CTRL_SIZE_MASK << AXI_AMAP_CTRL_SIZE_SHIFT) |
		AXI_AMAP_CTRL_EN);
	val |= ((type & AXI_AMAP_CTRL_TYPE_MASK) << AXI_AMAP_CTRL_TYPE_SHIFT) |
		((size_l >> AXI_AMAP_CTRL_SIZE_SHIFT) <<
		AXI_AMAP_CTRL_SIZE_SHIFT) | AXI_AMAP_CTRL_EN;

	mv_pcie_writel_csr(pci, PAB_AXI_AMAP_CTRL(idx), val);

	return 0;
}

int mv_pcie_inbound_win_setup_rc(struct mv_pcie *pci, int idx, int type,
			       u64 cpu_addr, u64 pci_addr, u64 size)
{
	u32 val;
	u64 win_size = ~(size - 1);

	if (pci->ops->inbound_win_setup_rc)
		return pci->ops->inbound_win_setup_rc(pci, idx, type, cpu_addr,
						      pci_addr, size);

	val = mv_pcie_readl_csr(pci, PAB_PEX_AMAP_CTRL(idx));

	val &= ~(PEX_AMAP_CTRL_TYPE_MASK << PEX_AMAP_CTRL_TYPE_SHIFT);
	val &= ~(PEX_AMAP_CTRL_EN_MASK << PEX_AMAP_CTRL_EN_SHIFT);
	val = (val | (type << PEX_AMAP_CTRL_TYPE_SHIFT));
	val = (val | (1 << PEX_AMAP_CTRL_EN_SHIFT));

	mv_pcie_writel_csr(pci, PAB_PEX_AMAP_CTRL(idx),
			   val | lower_32_bits(win_size));

	mv_pcie_writel_csr(pci, PAB_EXT_PEX_AMAP_SIZE(idx),
			   upper_32_bits(win_size));
	mv_pcie_writel_csr(pci, PAB_PEX_AMAP_AXI_WIN(idx),
			   lower_32_bits(cpu_addr));
	mv_pcie_writel_csr(pci, PAB_EXT_PEX_AMAP_AXI_WIN(idx),
			   upper_32_bits(cpu_addr));
	mv_pcie_writel_csr(pci, PAB_PEX_AMAP_PEX_WIN_L(idx),
			   lower_32_bits(pci_addr));
	mv_pcie_writel_csr(pci, PAB_PEX_AMAP_PEX_WIN_H(idx),
			   upper_32_bits(pci_addr));

	return 0;
}

void mv_pcie_disable_outbound_win(struct mv_pcie *pci, int idx)
{
	u32 val;

	val = mv_pcie_readl_csr(pci, PAB_AXI_AMAP_CTRL(idx));
	val &= ~AXI_AMAP_CTRL_EN;
	mv_pcie_writel_csr(pci, PAB_AXI_AMAP_CTRL(idx), val);
}

int mv_pcie_wait_for_link(struct mv_pcie *pci)
{
	int retries;

	/* check if the link is up or not */
	for (retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		if (mv_pcie_link_up(pci)) {
			dev_info(pci->dev, "link up\n");
			return 0;
		}
		usleep_range(LINK_WAIT_USLEEP_MIN, LINK_WAIT_USLEEP_MAX);
	}

	dev_err(pci->dev, "phy link never came up\n");

	return -ETIMEDOUT;
}

int mv_pcie_link_up(struct mv_pcie *pci)
{
	u32 val;

	if (pci->ops->link_up)
		return pci->ops->link_up(pci);

	val = readl(pci->csr_base + PAB_LTSSM_STA);
	return (val & PAB_LTSSM_MASK) == PAB_LTSSM_L0;
}

void mv_pcie_get_engines(struct mv_pcie *pci)
{
	u32 val;

	val = mv_pcie_readl_csr(pci, PAB_CAP);

	pci->apio_engines = (val >> 4) & 0x7;
	pci->ppio_engines = (val >> 7) & 0x7;
}

void mv_pcie_enable_bridge_pio(struct mv_pcie *pci)
{
	u32 val;

	val = mv_pcie_readl_csr(pci, PAB_CTRL);
	val |= PAB_CTRL_APIO_EN | PAB_CTRL_PPIO_EN;
	mv_pcie_writel_csr(pci, PAB_CTRL, val);
}

void mv_pcie_enable_engine_apio(struct mv_pcie *pci, int idx)
{
	u32 val;

	val = mv_pcie_readl_csr(pci, PAB_AXI_PIO_CTRL(idx));
	val |= APIO_EN | MEM_WIN_EN | IO_WIN_EN | CFG_WIN_EN;
	mv_pcie_writel_csr(pci, PAB_AXI_PIO_CTRL(idx), val);
}

void mv_pcie_enable_engine_ppio(struct mv_pcie *pci, int idx)
{
	u32 val;

	val = mv_pcie_readl_csr(pci, PAB_PEX_PIO_CTRL(idx));
	val |= PPIO_EN;
	mv_pcie_writel_csr(pci, PAB_PEX_PIO_CTRL(idx), val);
}
