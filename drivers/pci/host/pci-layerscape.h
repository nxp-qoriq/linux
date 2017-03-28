/*
 * Copyright (C) 2015 Freescale Semiconductor.
 *
 * Author: Varun Sethi <Varun.Sethi@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PCI_LAYERSCAPE_H
#define _PCI_LAYERSCAPE_H

/* function for setting up stream id to device id translation */
u32 set_pcie_streamid_translation(struct pci_dev *pdev, u32 devid);

#endif /* _PCI_LAYERSCAPE_H */
