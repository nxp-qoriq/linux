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

#ifndef _PCIE_MOBIVEIL_H
#define _PCIE_MOBIVEIL_H

#include <linux/pci.h>

/* GPEX CSR */
#define GPEX_CLASSCODE				0x474
#define GPEX_CLASSCODE_SHIFT			16
#define GPEX_CLASSCODE_MASK			0xffff

/* PAB CSR */
#define PAB_CAP					0x804
#define PAB_CTRL				0x808
#define PAB_CTRL_APIO_EN			(0x1 << 0)
#define PAB_CTRL_PPIO_EN			(0x1 << 1)
#define PAB_CTRL_MAX_BRST_LEN_SHIFT		4
#define PAB_CTRL_MAX_BRST_LEN_MASK		0x3
#define PAB_CTRL_PAGE_SEL_SHIFT			13
#define PAB_CTRL_PAGE_SEL_MASK			0x3f
#define PAB_CTRL_FUNC_SEL_SHIFT			19
#define PAB_CTRL_FUNC_SEL_MASK			0x1ff

#define PAB_RST_CTRL				0x820
#define PAB_BR_STAT				0x80c
#define PAB_INTP_AXI_MISC_ENB			0xb0c
#define MSI					(0x1 << 3)
#define INTA					(0x1 << 5)
#define INTB					(0x1 << 6)
#define INTC					(0x1 << 7)
#define INTD					(0x1 << 8)

/* AXI PIO Engines */
#define PAB_AXI_PIO_CTRL(idx)			(0x840 + 0x10 * idx)
#define APIO_EN					(0x1 << 0)
#define MEM_WIN_EN				(0x1 << 1)
#define IO_WIN_EN				(0x1 << 2)
#define CFG_WIN_EN				(0x1 << 3)
#define PAB_AXI_PIO_STAT(idx)			(0x844 + 0x10 * idx)
#define PAB_AXI_PIO_SL_CMD_STAT(idx)		(0x848 + 0x10 * idx)
#define PAB_AXI_PIO_SL_ADDR_STAT(idx)		(0x84c + 0x10 * idx)
#define PAB_AXI_PIO_SL_EXT_ADDR_STAT(idx)	(0xb8a0 + 0x4 * idx)

/* PEX PIO Engines */
#define PAB_PEX_PIO_CTRL(idx)			(0x8c0 + 0x10 * idx)
#define PPIO_EN					(0x1 << 0)
#define PAB_PEX_PIO_STAT(idx)			(0x8c4 + 0x10 * idx)
#define PAB_PEX_PIO_MT_STAT(idx)		(0x8c8 + 0x10 * idx)

#define INDIRECT_ADDR_BNDRY			0xc00
#define PAGE_IDX_SHIFT				10
#define PAGE_ADDR_MASK				0x3ff

#define OFFSET_TO_PAGE_IDX(off)			\
	((off >> PAGE_IDX_SHIFT) & PAB_CTRL_PAGE_SEL_MASK)

#define OFFSET_TO_PAGE_ADDR(off)		\
	((off & PAGE_ADDR_MASK) | INDIRECT_ADDR_BNDRY)

/* APIO WINs */
#define PAB_AXI_AMAP_CTRL(idx)			(0xba0 + 0x10 * idx)
#define PAB_EXT_AXI_AMAP_SIZE(idx)		(0xbaf0 + 0x4 * idx)
#define PAB_AXI_AMAP_AXI_WIN(idx)		(0xba4 + 0x10 * idx)
#define PAB_EXT_AXI_AMAP_AXI_WIN(idx)		(0x80a0 + 0x4 * idx)
#define PAB_AXI_AMAP_PEX_WIN_L(idx)		(0xba8 + 0x10 * idx)
#define PAB_AXI_AMAP_PEX_WIN_H(idx)		(0xbac + 0x10 * idx)

#define AXI_AMAP_CTRL_EN			(0x1 << 0)
#define AXI_AMAP_CTRL_TYPE_SHIFT		1
#define AXI_AMAP_CTRL_TYPE_MASK			0x3
#define AXI_AMAP_CTRL_SIZE_SHIFT		10
#define AXI_AMAP_CTRL_SIZE_MASK			0x3fffff

#define PAB_TARGET_BUS(x)			(((x) & 0xff) << 24)
#define PAB_TARGET_DEV(x)			(((x) & 0x1f) << 19)
#define PAB_TARGET_FUNC(x)			(((x) & 0x7) << 16)

#define TYPE_CFG			0x00
#define TYPE_IO				0x01
#define TYPE_MEM			0x02
#define TYPE_ATOM			0x03

/* PPIO WINs RC mode */
#define PAB_PEX_AMAP_CTRL(idx)			(0x4ba0 + 0x10 * idx)
#define PAB_EXT_PEX_AMAP_SIZE(idx)		(0xbef0 + 0x04 * idx)
#define PAB_PEX_AMAP_AXI_WIN(idx)		(0x4ba4 + 0x10 * idx)
#define PAB_EXT_PEX_AMAP_AXI_WIN(idx)		(0xb4a0 + 0x04 * idx)
#define PAB_PEX_AMAP_PEX_WIN_L(idx)		(0x4ba8 + 0x10 * idx)
#define PAB_PEX_AMAP_PEX_WIN_H(idx)		(0x4bac + 0x10 * idx)

#define IB_TYPE_MEM_F				0x2
#define IB_TYPE_MEM_NF				0x3

#define PEX_AMAP_CTRL_TYPE_SHIFT		0x1
#define PEX_AMAP_CTRL_EN_SHIFT			0x0
#define PEX_AMAP_CTRL_TYPE_MASK			0x3
#define PEX_AMAP_CTRL_EN_MASK			0x1

/* PPIO WINs EP mode */
#define PAB_PEX_BAR_AMAP(func, bar)		(0x1ba0 + 0x4 * func * bar)
#define PAB_EXT_PEX_BAR_AMAP(func, bar)		(0x84a0 + 0x4 * func * bar)

#define PEX_BAR_AMAP_EN_SHIFT			(0)
#define PEX_BAR_AMAP_EN_MASK			(1)

#define PAB_LTSSM_STA				0x404
#define PAB_LTSSM_MASK			0x7f
#define PAB_LTSSM_L0				0x2d /* L0 state */

/* Parameters for the waiting for link up routine */
#define LINK_WAIT_MAX_RETRIES		10
#define LINK_WAIT_USLEEP_MIN		90000
#define LINK_WAIT_USLEEP_MAX		100000

struct root_port;
struct mv_pcie;

struct mv_pcie_rp_ops {
	int (*read_own_conf)(struct root_port *rp, int where,
			     int size, u32 *val);
	int (*write_own_conf)(struct root_port *rp, int where,
			      int size, u32 val);
	int (*read_other_conf)(struct root_port *rp, int where,
			       int size, u32 *val);
	void (*init)(struct root_port *rp);
	void (*scan_bus)(struct root_port *rp);
};

struct root_port {
	u8			root_bus_nr;
	u64			cfg_base;
	void __iomem		*va_cfg_base;
	u32			cfg_size;
	resource_size_t		io_base;
	phys_addr_t		io_bus_addr;
	u32			io_size;
	u64			mem_base;
	phys_addr_t		mem_bus_addr;
	u32			mem_size;
	struct resource		*cfg;
	struct resource		*io;
	struct resource		*mem;
	struct resource		*busn;
	struct mv_pcie_rp_ops	*ops;
};

struct mv_pcie_pab_ops {
	u32 (*read_csr)(struct mv_pcie *pcie, void __iomem *base, u32 reg,
			    size_t size);
	void (*write_csr)(struct mv_pcie *pcie, void __iomem *base, u32 reg,
			     size_t size, u32 val);
	int (*link_up)(struct mv_pcie *pcie);
	int (*outbound_win_setup)(struct mv_pcie *pcie, int idx, int type,
				  u64 cpu_addr, u64 pci_addr, u64 size);
	int (*inbound_win_setup_rc)(struct mv_pcie *pcie, int idx, int type,
				    u64 cpu_addr, u64 pci_addr, u64 size);
};

struct mv_pcie {
	struct device		*dev;
	void __iomem		*csr_base;
	u32			apio_engines;
	u32			ppio_engines;
	u32			apio_wins;
	u32			ppio_wins;
	struct root_port	rp;
	const struct mv_pcie_pab_ops *ops;
};

#define rp_to_mv_pcie(rp) container_of((rp), struct mv_pcie, rp)


int mv_pcie_read(void __iomem *addr, int size, u32 *val);
int mv_pcie_write(void __iomem *addr, int size, u32 val);

u32 mv_pcie_read_csr(struct mv_pcie *pci, void __iomem *base, u32 reg,
		     size_t size);
void mv_pcie_write_csr(struct mv_pcie *pci, void __iomem *base, u32 reg,
		       size_t size, u32 val);
int mv_pcie_link_up(struct mv_pcie *pci);
int mv_pcie_wait_for_link(struct mv_pcie *pci);
int mv_pcie_outbound_win_setup(struct mv_pcie *pci, int index,
			       int type, u64 cpu_addr, u64 pci_addr,
			       u64 size);
int mv_pcie_inbound_win_setup_rc(struct mv_pcie *pci, int index,
				 int type, u64 cpu_addr, u64 pci_addr,
				 u64 size);
void mv_pcie_disable_outbound_win(struct mv_pcie *pci, int index);
void mv_pcie_setup(struct mv_pcie *pci);

void mv_pcie_get_engines(struct mv_pcie *pci);
void mv_pcie_enable_bridge_pio(struct mv_pcie *pci);
void mv_pcie_enable_engine_apio(struct mv_pcie *pci, int idx);
void mv_pcie_enable_engine_ppio(struct mv_pcie *pci, int idx);

static inline u32 mv_pcie_readl_csr(struct mv_pcie *pci, u32 reg)
{
	return mv_pcie_read_csr(pci, pci->csr_base, reg, 0x4);
}

static inline u16 mv_pcie_readw_csr(struct mv_pcie *pci, u32 reg)
{
	return mv_pcie_read_csr(pci, pci->csr_base, reg, 0x2);
}

static inline u8 mv_pcie_readb_csr(struct mv_pcie *pci, u32 reg)
{
	return mv_pcie_read_csr(pci, pci->csr_base, reg, 0x1);
}

static inline void mv_pcie_writel_csr(struct mv_pcie *pci, u32 reg, u32 val)
{
	mv_pcie_write_csr(pci, pci->csr_base, reg, 0x4, val);
}

static inline void mv_pcie_writew_csr(struct mv_pcie *pci, u32 reg, u16 val)
{
	mv_pcie_write_csr(pci, pci->csr_base, reg, 0x2, val);
}

static inline void mv_pcie_writeb_csr(struct mv_pcie *pci, u32 reg, u8 val)
{
	mv_pcie_write_csr(pci, pci->csr_base, reg, 0x1, val);
}

#ifdef CONFIG_PCIE_MOBIVEIL_HOST
void mv_pcie_setup_rp_hw(struct root_port *rp);
int mv_pcie_init_rp(struct root_port *rp);
#endif

#endif /* _PCIE_MOBIVEIL_H */
