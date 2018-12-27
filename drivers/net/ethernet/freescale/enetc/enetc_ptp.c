// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2018 NXP */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fsl/ptp_qoriq.h>

#include "enetc.h"

int enetc_phc_index = ENETC_PHC_INDEX_DEFAULT;
EXPORT_SYMBOL(enetc_phc_index);

static struct ptp_clock_info enetc_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "ENETC PTP clock",
	.max_adj	= 512000,
	.n_alarm	= 2,
	.n_ext_ts	= 2,
	.n_per_out	= 3,
	.n_pins		= 0,
	.pps		= 1,
	.adjfine	= ptp_qoriq_adjfine,
	.adjtime	= ptp_qoriq_adjtime,
	.gettime64	= ptp_qoriq_gettime,
	.settime64	= ptp_qoriq_settime,
	.enable		= ptp_qoriq_enable,
};

static int enetc_ptp_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct device *ptp_dev = &pdev->dev;
	struct qoriq_ptp *qoriq_ptp;
	void __iomem *base;
	int err, len, n;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);

	qoriq_ptp = kzalloc(sizeof(*qoriq_ptp), GFP_KERNEL);
	if (!qoriq_ptp) {
		err = -ENOMEM;
		goto err_alloc_ptp;
	}

	len = pci_resource_len(pdev, ENETC_BAR_REGS);

	base = ioremap(pci_resource_start(pdev, ENETC_BAR_REGS), len);
	if (!base) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_ioremap;
	}

	/* Allocate 1 interrupt */
	n = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSIX);
	if (n != 1) {
		err = -EPERM;
		goto err_irq;
	}

	qoriq_ptp->irq = pci_irq_vector(pdev, 0);

	err = request_irq(qoriq_ptp->irq, ptp_qoriq_isr, 0, DRIVER, qoriq_ptp);
	if (err) {
		dev_err(&pdev->dev, "request_irq() failed!\n");
		goto err_irq;
	}

	err = qoriq_ptp_init(ptp_dev, qoriq_ptp, base, enetc_ptp_caps);
	if (err)
		goto err_no_clock;

	enetc_phc_index = qoriq_ptp->phc_index;
	pci_set_drvdata(pdev, qoriq_ptp);

	return 0;

err_no_clock:
	free_irq(qoriq_ptp->irq, qoriq_ptp);
err_irq:
	iounmap(base);
err_ioremap:
	kfree(qoriq_ptp);
err_alloc_ptp:
	pci_release_mem_regions(pdev);
err_pci_mem_reg:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void enetc_ptp_remove(struct pci_dev *pdev)
{
	struct qoriq_ptp *qoriq_ptp = pci_get_drvdata(pdev);
	struct qoriq_ptp_registers *regs = &qoriq_ptp->regs;

	qoriq_write(qoriq_ptp, &regs->ctrl_regs->tmr_temask, 0);
	qoriq_write(qoriq_ptp, &regs->ctrl_regs->tmr_ctrl,   0);

	ptp_clock_unregister(qoriq_ptp->clock);
	iounmap(qoriq_ptp->base);
	kfree(qoriq_ptp);

	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id enetc_ptp_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_PTP) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_ptp_id_table);

static struct pci_driver enetc_ptp_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_ptp_id_table,
	.probe = enetc_ptp_probe,
	.remove = enetc_ptp_remove,
};
module_pci_driver(enetc_ptp_driver);

MODULE_DESCRIPTION("ENETC PTP clock driver");
MODULE_LICENSE("GPL");
