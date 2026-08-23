/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Endpoint USB (EPF) header file
 *
 * Copyright 2026 NXP
 */

#ifndef __PCI_EPF_USB_USBIP_H__
#define __PCI_EPF_USB_USBIP_H__

#include <linux/types.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>



/* =========================
 * Context (shared with transport layer)
 * ========================= */

struct usbip_pcie_ctx {

    struct pci_epf *epf;

    void *priv;  /* back pointer to EPF */
    const struct usbip_pcie_ops *ops;
};


struct usbip_pcie_ops {
    int (*recv_stream)(struct usbip_pcie_ctx *ctx, void *buf, int size);
    int (*send_stream)(struct usbip_pcie_ctx *ctx, const void *buf, int size);
};

/* Publish / access the currently active PCIe transport */
void usbip_stub_set_ctx(struct usbip_pcie_ctx *ctx);
struct usbip_pcie_ctx *usbip_stub_get_ctx(void);


#endif /* __PCI_EPF_USB_USBIP_H__ */
