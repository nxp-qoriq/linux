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


#ifndef _PCIE_LX_EP_H
#define _PCIE_LX_EP_H

#include <linux/device.h>
#include "pcie-mobiveil.h"
/* Synopsis specific PCIE configuration registers */

#define PCIE_SRIOV_POS		0x178
#define PCIE_PF_NUM		2
#define PCIE_VF_NUM		4
#define PCIE_VF_NUM_TOTAL	32

#define PCIE_SRIOV_CAPABILITY	0x2a0

#define PCIE_BAR_NUM		4
#define PCIE_BAR_NUM_SRIOV	8
#define PCIE_BAR_SIZE           (8 * 1024) /* 1M */
#define SIZE_1T            (1024 * 1024 * 1024 * 1024ULL)
#define SIZE_1G            (1024 * 1024 * 1024)
#define SIZE_1M            (1024 * 1024)

struct lx_pcie_ep_drvdata {
	u32 lut_offset;
	u32 ltssm_shift;
	u32 ltssm_mask;
	u32 lut_dbg;
	bool lut_big_endian;
	struct mv_pcie_rp_ops *rp_ops;
	const struct mv_pcie_pab_ops *pab_ops;
};

struct lx_pcie {
	struct mv_pcie          *pci;
	struct list_head        ep_list;
	struct device           *dev;
	struct dentry           *dir;
	const struct lx_pcie_ep_drvdata *drvdata;
	void __iomem            *buf_pf;
	dma_addr_t              buf_addr_pf;
	void __iomem            *buf_vf;
	dma_addr_t              buf_addr_vf;
	void __iomem            *lut;
	phys_addr_t             out_base;
	int                     sriov;
	int                     index;
};

struct lx_ep_dev {
	struct list_head	node;
	struct lx_pcie		*pcie;
	struct device		dev;
	struct dentry		*dir;
	int			pf_idx;
	int			vf_idx;
	int			dev_id;
	void			*driver_data;
};

struct lx_ep_dev *lx_pci_ep_find(struct lx_pcie *pcie, int dev_id);

int lx_pcie_ep_outbound_win_set(struct lx_pcie *pcie, int idx, int type,
			      u64 phys, u64 bus_addr, u32 func, u64 size);

/* Use bar match mode and MEM type as default */
void lx_pcie_ep_inbound_win_set(struct lx_pcie *pcie, int func, int bar,
				     u64 phys);

void lx_pcie_ep_dev_setup_bar(struct lx_ep_dev *ep, int bar, u32 size);

int lx_pcie_ep_dbgfs_init(struct lx_pcie *pcie);
int lx_pcie_ep_dbgfs_remove(struct lx_pcie *pcie);

#endif /* _PCIE_LX_EP_H */
