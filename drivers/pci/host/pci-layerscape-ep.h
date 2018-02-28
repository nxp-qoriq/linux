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


#ifndef _PCIE_LAYERSCAPE_EP_H
#define _PCIE_LAYERSCAPE_EP_H

#include <linux/device.h>

/* Synopsis specific PCIE configuration registers */
#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX3		(0x3 << 0)
#define PCIE_ATU_REGION_INDEX2		(0x2 << 0)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

/* PEX internal configuration registers */
#define PCIE_DBI_RO_WR_EN	0x8bc /* DBI Read-Only Write Enable Register */

/* PEX LUT registers */
#define PCIE_LUT_BASE		0x80000
#define PCIE_LUT_DBG		0x7FC /* PEX LUT Debug register */

#define PCIE_LUT_LCTRL0		0x7F8

#define PCIE_ATU_BAR_NUM(bar)	((bar) << 8)
#define PCIE_LCTRL0_CFG2_ENABLE	(1 << 31)
#define PCIE_LCTRL0_VF(vf)	((vf) << 22)
#define PCIE_LCTRL0_PF(pf)	((pf) << 16)
#define PCIE_LCTRL0_VF_ACTIVE	(1 << 21)
#define PCIE_LCTRL0_VAL(pf, vf)	(PCIE_LCTRL0_PF(pf) |			   \
				 PCIE_LCTRL0_VF(vf) |			   \
				 ((vf) == 0 ? 0 : PCIE_LCTRL0_VF_ACTIVE) | \
				 PCIE_LCTRL0_CFG2_ENABLE)

#define PCIE_NO_SRIOV_BAR_BASE	0x1000

#define PCIE_SRIOV_POS		0x178
#define PCIE_PF_NUM		2
#define PCIE_VF_NUM		64

struct ls_pcie_ep_drvdata {
	u32 lut_offset;
	u32 ltssm_shift;
	u32 lut_dbg;
};

struct ls_pcie {
	struct list_head	ep_list;
	struct device		*dev;
	struct dentry		*dir;
	const struct ls_pcie_ep_drvdata *drvdata;
	void __iomem		*dbi;
	void __iomem		*lut;
	phys_addr_t		out_base;
	int			sriov;
	int			index;
};

struct ls_ep_dev {
	struct list_head	node;
	struct ls_pcie		*pcie;
	struct device		dev;
	struct dentry		*dir;
	int			pf_idx;
	int			vf_idx;
	int			dev_id;
	void			*driver_data;
};

struct ls_ep_dev *ls_pci_ep_find(struct ls_pcie *pcie, int dev_id);

void ls_pcie_iatu_outbound_set(struct ls_pcie *pcie, int idx, int type,
			      u64 cpu_addr, u64 pci_addr, u32 size);

/* Use bar match mode and MEM type as default */
void ls_pcie_iatu_inbound_set(struct ls_pcie *pcie, int idx,
				     int bar, u64 phys);

void ls_pcie_ep_dev_setup_bar(struct ls_ep_dev *ep, int bar, u32 size);


void ls_pcie_ep_dev_cfg_enable(struct ls_ep_dev *ep);

int ls_pcie_ep_dbgfs_init(struct ls_pcie *pcie);
int ls_pcie_ep_dbgfs_remove(struct ls_pcie *pcie);

#endif /* _PCIE_LAYERSCAPE_EP_H */
