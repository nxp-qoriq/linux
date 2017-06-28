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

#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>

#include "pcie-mobiveil.h"

static int mv_pcie_read_own_conf(struct root_port *rp, int where, int size,
				 u32 *val)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);

	if (rp->ops->read_own_conf)
		return rp->ops->read_own_conf(rp, where, size, val);

	*val = mv_pcie_read_csr(pci, pci->csr_base, where, size);
	return 0;
}

static int mv_pcie_write_own_conf(struct root_port *rp, int where, int size,
				  u32 val)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);

	if (rp->ops->write_own_conf)
		return rp->ops->write_own_conf(rp, where, size, val);

	mv_pcie_write_csr(pci, pci->csr_base, where, size, val);

	return 0;
}

static int mv_pcie_read_other_conf(struct root_port *rp, struct pci_bus *bus,
				   u32 devfn, int where, int size, u32 *val)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);
	u32 target;
	int ret;

	target = PAB_TARGET_BUS(bus->number) |
		 PAB_TARGET_DEV(PCI_SLOT(devfn)) |
		 PAB_TARGET_FUNC(PCI_FUNC(devfn));

	mv_pcie_outbound_win_setup(pci, 1, TYPE_CFG, rp->cfg_base,
				   target, rp->cfg_size);

	if (rp->ops->read_other_conf)
		ret = rp->ops->read_other_conf(rp, where, size, val);
	else
		ret = mv_pcie_read(rp->va_cfg_base + where, size, val);

	if (pci->apio_wins <= 2)
		mv_pcie_outbound_win_setup(pci, 1, TYPE_IO, rp->io_base,
					   rp->io_bus_addr, rp->io_size);

	return ret;
}

static int mv_pcie_write_other_conf(struct root_port *rp, struct pci_bus *bus,
				    u32 devfn, int where, int size, u32 val)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);
	u32 target;
	int ret;

	target = PAB_TARGET_BUS(bus->number) |
		 PAB_TARGET_DEV(PCI_SLOT(devfn)) |
		 PAB_TARGET_FUNC(PCI_FUNC(devfn));


	mv_pcie_outbound_win_setup(pci, 1, TYPE_CFG, rp->cfg_base,
				   target, rp->cfg_size);
	ret = mv_pcie_write(rp->va_cfg_base + where, size, val);
	if (pci->apio_wins <= 2)
		mv_pcie_outbound_win_setup(pci, 1, TYPE_IO, rp->io_base,
					   rp->io_bus_addr, rp->io_size);

	return ret;
}

static int mv_pcie_valid_device(struct root_port *rp, struct pci_bus *bus,
				int dev)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);

	/* If there is no link, then there is no device */
	if (bus->number > rp->root_bus_nr && !mv_pcie_link_up(pci))
		return 0;

	/* access only one slot on each root port */
	if (bus->number <= rp->root_bus_nr + 1 && dev > 0)
		return 0;

	return 1;
}

static int mv_pcie_read_conf(struct pci_bus *bus, u32 devfn, int where,
			     int size, u32 *val)
{
	struct root_port *rp = bus->sysdata;

	if (!mv_pcie_valid_device(rp, bus, PCI_SLOT(devfn))) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (bus->number == rp->root_bus_nr)
		return mv_pcie_read_own_conf(rp, where, size, val);

	return mv_pcie_read_other_conf(rp, bus, devfn, where, size, val);
}

static int mv_pcie_write_conf(struct pci_bus *bus, u32 devfn,
			      int where, int size, u32 val)
{
	struct root_port *rp = bus->sysdata;

	if (!mv_pcie_valid_device(rp, bus, PCI_SLOT(devfn)))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (bus->number == rp->root_bus_nr)
		return mv_pcie_write_own_conf(rp, where, size, val);

	return mv_pcie_write_other_conf(rp, bus, devfn, where, size, val);
}

static struct pci_ops mv_pcie_ops = {
	.read = mv_pcie_read_conf,
	.write = mv_pcie_write_conf,
};

void mv_pcie_setup_rp_hw(struct root_port *rp)
{
	u32 val;
	struct mv_pcie *pci = rp_to_mv_pcie(rp);
	int i;

	/* setup RC BARs */
	mv_pcie_writel_csr(pci, PCI_BASE_ADDRESS_0, 0x0);
	mv_pcie_writel_csr(pci, PCI_BASE_ADDRESS_1, 0x0);

	/* setup bus numbers */
	val = mv_pcie_readl_csr(pci, PCI_PRIMARY_BUS);
	val &= 0xff000000;
	val |= 0x00010100;
	mv_pcie_writel_csr(pci, PCI_PRIMARY_BUS, val);

	/* setup command register */
	val = mv_pcie_readl_csr(pci, PCI_COMMAND);
	val &= 0xffff0000;
	val |= PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SERR;
	mv_pcie_writel_csr(pci, PCI_COMMAND, val);

	/* enable INTx and MSI */
	val = mv_pcie_readl_csr(pci, PAB_INTP_AXI_MISC_ENB);
	val |= MSI | INTA | INTB | INTC | INTD;
	mv_pcie_writel_csr(pci, PAB_INTP_AXI_MISC_ENB, val);

	mv_pcie_outbound_win_setup(pci, 0, TYPE_MEM, rp->mem_base,
				   rp->mem_bus_addr, rp->mem_size);
	if (pci->apio_wins > 2)
		mv_pcie_outbound_win_setup(pci, 2, TYPE_IO, rp->io_base,
					   rp->io_bus_addr, rp->io_size);

	mv_pcie_inbound_win_setup_rc(pci, 0, IB_TYPE_MEM_NF, 0, 0, 1ULL << 40);

	/* program correct class for RC */
	mv_pcie_read_own_conf(rp, GPEX_CLASSCODE, 4, &val);
	val &= ~(GPEX_CLASSCODE_MASK << GPEX_CLASSCODE_SHIFT);
	val |= PCI_CLASS_BRIDGE_PCI << GPEX_CLASSCODE_SHIFT;
	mv_pcie_write_own_conf(rp, GPEX_CLASSCODE, 4, val);

	mv_pcie_enable_bridge_pio(pci);

	mv_pcie_get_engines(pci);
	val = pci->apio_engines ? pci->apio_engines : 1;
	for (i = 0; i < val; i++)
		mv_pcie_enable_engine_apio(pci, i);

	val = pci->ppio_engines ? pci->ppio_engines : 1;
	for (i = 0; i < val; i++)
		mv_pcie_enable_engine_ppio(pci, i);
}

int mv_pcie_init_rp(struct root_port *rp)
{
	struct mv_pcie *pci = rp_to_mv_pcie(rp);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct platform_device *pdev = to_platform_device(dev);
	struct pci_bus *bus, *child;
	struct resource *cfg_res;
	LIST_HEAD(res);
	struct resource_entry *win, *tmp;
	int ret;

	if (!rp->va_cfg_base) {
		cfg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "config");
		if (!cfg_res) {
			dev_err(dev, "failed to get config reg\n");
			return -ENOENT;
		}

		rp->cfg_size = resource_size(cfg_res);
		rp->cfg_base = cfg_res->start;

		rp->va_cfg_base = devm_ioremap(dev, rp->cfg_base, rp->cfg_size);
		if (!rp->va_cfg_base) {
			dev_err(dev, "error with ioremap\n");
			return -ENOMEM;
		}
	}

	if (!pci->apio_wins &&
	    of_property_read_u32(np, "apio-wins", &pci->apio_wins)) {
		pci->apio_wins = 2;
	}

	if (!pci->ppio_wins &&
	    of_property_read_u32(np, "ppio-wins-rc", &pci->ppio_wins)) {
		pci->ppio_wins = 2;
	}

	ret = of_pci_get_host_bridge_resources(np, 0, 0xff, &res, &rp->io_base);
	if (ret)
		return ret;

	ret = devm_request_pci_bus_resources(dev, &res);
	if (ret)
		goto error;

	/* Get the I/O and memory ranges from DT */
	resource_list_for_each_entry_safe(win, tmp, &res) {
		switch (resource_type(win->res)) {
		case IORESOURCE_IO:
			ret = pci_remap_iospace(win->res, rp->io_base);
			if (ret) {
				dev_warn(dev, "error %d: failed to map resource %pR\n",
					 ret, win->res);
				resource_list_destroy_entry(win);
			} else {
				rp->io = win->res;
				rp->io->name = "I/O";
				rp->io_size = resource_size(rp->io);
				rp->io_bus_addr = rp->io->start - win->offset;
			}
			break;
		case IORESOURCE_MEM:
			rp->mem = win->res;
			rp->mem->name = "MEM";
			rp->mem_size = resource_size(rp->mem);
			rp->mem_bus_addr = rp->mem->start - win->offset;
			break;
		case IORESOURCE_BUS:
			rp->busn = win->res;
			break;
		}
	}

	rp->root_bus_nr = rp->busn->start;
	rp->mem_base = rp->mem->start;

	if (rp->ops->init)
		rp->ops->init(rp);

	bus = pci_scan_root_bus(dev, rp->root_bus_nr, &mv_pcie_ops, rp, &res);
	if (!bus) {
		ret = -ENOMEM;
		goto error;
	}

	if (rp->ops->scan_bus)
		rp->ops->scan_bus(rp);

#ifdef CONFIG_ARM
	pci_fixup_irqs(pci_common_swizzle, of_irq_parse_and_map_pci);
#endif

	pci_bus_size_bridges(bus);
	pci_bus_assign_resources(bus);

	list_for_each_entry(child, &bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(bus);
	return 0;

error:
	pci_free_resource_list(&res);
	return ret;
}
