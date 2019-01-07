// SPDX-License-Identifier: GPL-2.0
/**
 * Mobiveil PCIe Endpoint controller driver
 *
 * Copyright (C) 2018 NXP Semiconductor.
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
 */

#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/platform_device.h>
#include "pcie-mobiveil.h"

void mobiveil_pcie_ep_linkup(struct mobiveil_pcie_ep *ep)
{
	struct pci_epc *epc = ep->epc;

	pci_epc_linkup(epc);
}

static void __mobiveil_pcie_ep_reset_bar(struct mobiveil_pcie *pci,
					 enum pci_barno bar)
{
	csr_writel(pci, bar, GPEX_BAR_SELECT);
	csr_writel(pci, 0, GPEX_BAR_SIZE_LDW);
	csr_writel(pci, 0, GPEX_BAR_SIZE_UDW);
}

void mobiveil_pcie_ep_reset_bar(struct mobiveil_pcie *pci, enum pci_barno bar)
{
	__mobiveil_pcie_ep_reset_bar(pci, bar);
}

static u8 __mobiveil_pcie_ep_find_next_cap(struct mobiveil_pcie *pci,
					   u8 cap_ptr, u8 cap)
{
	u8 cap_id, next_cap_ptr;
	u16 reg;

	reg = csr_readw(pci, cap_ptr);
	next_cap_ptr = (reg & 0xff00) >> 8;
	cap_id = (reg & 0x00ff);

	if (cap_id == cap)
		return cap_ptr;

	if (!next_cap_ptr || cap_id > PCI_CAP_ID_MAX)
		return 0;

	return __mobiveil_pcie_ep_find_next_cap(pci, next_cap_ptr, cap);
}

static u8 mobiveil_pcie_ep_find_capability(struct mobiveil_pcie *pci, u8 cap)
{
	u8 next_cap_ptr;
	u16 reg;

	reg = csr_readw(pci, PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & 0x00ff);

	if (!next_cap_ptr)
		return 0;

	return __mobiveil_pcie_ep_find_next_cap(pci, next_cap_ptr, cap);
}

static int mobiveil_pcie_ep_write_header(struct pci_epc *epc,
					 struct pci_epf_header *hdr)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	csr_writew(pci, hdr->vendorid, PCI_VENDOR_ID);
	csr_writew(pci, hdr->deviceid, PCI_DEVICE_ID);
	csr_writeb(pci, hdr->revid, PCI_REVISION_ID);
	csr_writeb(pci, hdr->progif_code, PCI_CLASS_PROG);
	csr_writew(pci, hdr->subclass_code | hdr->baseclass_code << 8,
		   PCI_CLASS_DEVICE);
	csr_writeb(pci, hdr->cache_line_size, PCI_CACHE_LINE_SIZE);
	csr_writew(pci, hdr->subsys_vendor_id, PCI_SUBSYSTEM_VENDOR_ID);
	csr_writew(pci, hdr->subsys_id, PCI_SUBSYSTEM_ID);
	csr_writeb(pci, hdr->interrupt_pin, PCI_INTERRUPT_PIN);

	return 0;
}

static int mobiveil_pcie_ep_inbound_atu(struct mobiveil_pcie_ep *ep,
					enum pci_barno bar,
					dma_addr_t cpu_addr)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	program_ib_windows_ep(pci, 0, bar, cpu_addr);

	return 0;
}

static int mobiveil_pcie_ep_outbound_atu(struct mobiveil_pcie_ep *ep,
					 phys_addr_t phys_addr,
					 u64 pci_addr,
					 size_t size)
{
	int ret;
	u32 free_win;
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	free_win = find_first_zero_bit(ep->ob_window_map, ep->num_ob_windows);
	if (free_win >= ep->num_ob_windows) {
		dev_err(&pci->pdev->dev, "No free outbound window\n");
		return -EINVAL;
	}

	ret = program_ob_windows_ep(pci, free_win, MEM_WINDOW_TYPE,
				  phys_addr, pci_addr, 0, size);
	if (ret < 0) {
		dev_err(&pci->pdev->dev, "Failed to program IB window\n");
		return ret;
	}

	set_bit(free_win, ep->ob_window_map);
	ep->outbound_addr[free_win] = phys_addr;

	return 0;
}

static void mobiveil_pcie_ep_clear_bar(struct pci_epc *epc,
				       enum pci_barno bar)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	if (bar < ep->bar_num) {
		__mobiveil_pcie_ep_reset_bar(pci, bar);

		mobiveil_pcie_disable_ib_win_ep(pci, 0, bar);
	}
}

static int mobiveil_pcie_ep_set_bar(struct pci_epc *epc,
				    enum pci_barno bar,
				    dma_addr_t bar_phys,
				    size_t size, int flags)
{
	int ret;
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	if (bar < ep->bar_num) {
		ret = mobiveil_pcie_ep_inbound_atu(ep, bar,
						   bar_phys);
		if (ret)
			return ret;

		csr_writel(pci, bar, GPEX_BAR_SELECT);
		csr_writel(pci, lower_32_bits(~(size - 1)), GPEX_BAR_SIZE_LDW);
		csr_writel(pci, upper_32_bits(~(size - 1)), GPEX_BAR_SIZE_UDW);
	}

	return 0;
}

static int mobiveil_pcie_find_index(struct mobiveil_pcie_ep *ep,
				    phys_addr_t addr,
				    u32 *atu_index)
{
	u32 index;

	for (index = 0; index < ep->num_ob_windows; index++) {
		if (ep->outbound_addr[index] != addr)
			continue;
		*atu_index = index;
		return 0;
	}

	return -EINVAL;
}

static void mobiveil_pcie_ep_unmap_addr(struct pci_epc *epc,
					phys_addr_t addr)
{
	int ret;
	u32 atu_index;
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	ret = mobiveil_pcie_find_index(ep, addr, &atu_index);
	if (ret < 0)
		return;

	mobiveil_pcie_disable_ob_win(pci, atu_index);
	clear_bit(atu_index, ep->ob_window_map);
}

static int mobiveil_pcie_ep_map_addr(struct pci_epc *epc,
				     phys_addr_t addr,
				     u64 pci_addr, size_t size)
{
	int ret;
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	ret = mobiveil_pcie_ep_outbound_atu(ep, addr, pci_addr, size);
	if (ret) {
		dev_err(&pci->pdev->dev, "Failed to enable address\n");
		return ret;
	}

	return 0;
}

static int mobiveil_pcie_ep_get_msi(struct pci_epc *epc)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;

	if (!ep->msi_cap)
		return -EINVAL;

	reg = ep->msi_cap + PCI_MSI_FLAGS;
	val = csr_readw(pci, reg);
	if (!(val & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	val = (val & PCI_MSI_FLAGS_QSIZE) >> 4;

	return val;
}

static int mobiveil_pcie_ep_set_msi(struct pci_epc *epc,
				    u8 interrupts)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	u32 val, reg;

	if (!ep->msi_cap)
		return -EINVAL;

	reg = ep->msi_cap + PCI_MSI_FLAGS;
	val = csr_readw(pci, reg);
	val &= ~PCI_MSI_FLAGS_QMASK;
	val |= (interrupts << 1) & PCI_MSI_FLAGS_QMASK;
	csr_writew(pci, val, reg);

	return 0;
}

static int mobiveil_pcie_ep_raise_irq(struct pci_epc *epc,
				      enum pci_epc_irq_type type,
				      u8 interrupt_num)
{
	struct mobiveil_pcie_ep *ep = epc_get_drvdata(epc);

	if (!ep->ops->raise_irq)
		return -EINVAL;

	return ep->ops->raise_irq(ep, type, interrupt_num);
}

static const struct pci_epc_ops epc_ops = {
	.write_header		= mobiveil_pcie_ep_write_header,
	.set_bar		= mobiveil_pcie_ep_set_bar,
	.clear_bar		= mobiveil_pcie_ep_clear_bar,
	.map_addr		= mobiveil_pcie_ep_map_addr,
	.unmap_addr		= mobiveil_pcie_ep_unmap_addr,
	.set_msi		= mobiveil_pcie_ep_set_msi,
	.get_msi		= mobiveil_pcie_ep_get_msi,
	.raise_irq		= mobiveil_pcie_ep_raise_irq,
};

int mobiveil_pcie_ep_raise_legacy_irq(struct mobiveil_pcie_ep *ep)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);

	dev_err(&pci->pdev->dev, "EP cannot trigger legacy IRQs\n");

	return -EINVAL;
}

int mobiveil_pcie_ep_raise_msi_irq(struct mobiveil_pcie_ep *ep,
				   u8 interrupt_num)
{
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	struct pci_epc *epc = ep->epc;
	u16 msg_ctrl, msg_data;
	u32 msg_addr_lower, msg_addr_upper, reg;
	u64 msg_addr;
	u32 func_num;
	bool has_upper;
	int ret;

	if (!ep->msi_cap)
		return -EINVAL;

	func_num = csr_readl(pci, PAB_CTRL);
	func_num &= ~(FUNC_SEL_MASK << FUNC_SEL_SHIFT);
	func_num |= (0 & FUNC_SEL_MASK) << FUNC_SEL_SHIFT;
	csr_writel(pci, func_num, PAB_CTRL);

	/* Raise MSI per the PCI Local Bus Specification Revision 3.0, 6.8.1. */
	reg = ep->msi_cap + PCI_MSI_FLAGS;
	msg_ctrl = csr_readw(pci, reg);
	has_upper = !!(msg_ctrl & PCI_MSI_FLAGS_64BIT);
	reg = ep->msi_cap + PCI_MSI_ADDRESS_LO;
	msg_addr_lower = csr_readl(pci, reg);
	if (has_upper) {
		reg = ep->msi_cap + PCI_MSI_ADDRESS_HI;
		msg_addr_upper = csr_readl(pci, reg);
		reg = ep->msi_cap + PCI_MSI_DATA_64;
		msg_data = csr_readw(pci, reg);
	} else {
		msg_addr_upper = 0;
		reg = ep->msi_cap + PCI_MSI_DATA_32;
		msg_data = csr_readw(pci, reg);
	}
	msg_addr = ((u64) msg_addr_upper) << 32 | msg_addr_lower;

	func_num = csr_readl(pci, PAB_CTRL);
	func_num &= ~(FUNC_SEL_MASK << FUNC_SEL_SHIFT);
	csr_writel(pci, func_num, PAB_CTRL);

	ret = mobiveil_pcie_ep_map_addr(epc, ep->msi_mem_phys,
					msg_addr, epc->mem->page_size);
	if (ret)
		return ret;

	writel(msg_data | (interrupt_num - 1), ep->msi_mem);

	mobiveil_pcie_ep_unmap_addr(epc, ep->msi_mem_phys);

	return 0;
}

void mobiveil_pcie_ep_exit(struct mobiveil_pcie_ep *ep)
{
	struct pci_epc *epc = ep->epc;

	pci_epc_mem_free_addr(epc, ep->msi_mem_phys, ep->msi_mem,
			      epc->mem->page_size);

	pci_epc_mem_exit(epc);
}

int mobiveil_pcie_ep_init(struct mobiveil_pcie_ep *ep)
{
	int ret;
	void *addr;
	struct pci_epc *epc;
	struct mobiveil_pcie *pci = to_mobiveil_pcie_from_ep(ep);
	struct device *dev = &pci->pdev->dev;
	struct device_node *np = dev->of_node;

	if (!pci->csr_axi_slave_base) {
		dev_err(dev, "csr_base is not populated\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "num-ob-windows", &ep->num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "Unable to read *num-ob-windows* property\n");
		return ret;
	}

	if (ep->num_ob_windows > MAX_IATU_OUT) {
		dev_err(dev, "Invalid *num-ob-windows*\n");
		return -EINVAL;
	}
	ep->ob_window_map = devm_kcalloc(dev,
					 BITS_TO_LONGS(ep->num_ob_windows),
					 sizeof(long),
					 GFP_KERNEL);
	if (!ep->ob_window_map)
		return -ENOMEM;

	addr = devm_kcalloc(dev, ep->num_ob_windows, sizeof(phys_addr_t),
			    GFP_KERNEL);
	if (!addr)
		return -ENOMEM;
	ep->outbound_addr = addr;

	mobiveil_pcie_enable_bridge_pio(pci);
	mobiveil_pcie_enable_engine_apio(pci);
	mobiveil_pcie_enable_engine_ppio(pci);
	mobiveil_pcie_enable_msi_ep(pci);

	epc = devm_pci_epc_create(dev, &epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "Failed to create epc device\n");
		return PTR_ERR(epc);
	}

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	ep->msi_cap = mobiveil_pcie_ep_find_capability(pci, PCI_CAP_ID_MSI);

	if (ep->ops->ep_init)
		ep->ops->ep_init(ep);

	epc->max_functions = ep->pf_num;

	ret = __pci_epc_mem_init(epc, ep->phys_base, ep->addr_size,
				 ep->page_size);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize address space\n");
		return ret;
	}

	ep->msi_mem = pci_epc_mem_alloc_addr(epc, &ep->msi_mem_phys,
					     epc->mem->page_size);
	if (!ep->msi_mem) {
		dev_err(dev, "Failed to reserve memory for MSI/MSI-X\n");
		return -ENOMEM;
	}

	return 0;
}
