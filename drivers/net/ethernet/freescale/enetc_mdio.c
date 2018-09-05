#include <linux/module.h>
#include <linux/pci.h>

#define ENETC_MDIO_DEV_ID	0xee01
#define ENETC_DRV_NAME_STR "ENETC MDIO driver"

static int enetc_mdio_probe(struct pci_dev *pdev,
			    const struct pci_device_id *ent)
{
	int err;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);

	return 0;

err_pci_mem_reg:
	pci_disable_device(pdev);

	return err;
}

void enetc_mdio_remove(struct pci_dev *pdev)
{
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id enetc_mdio_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_MDIO_DEV_ID) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_mdio_id_table);

static struct pci_driver enetc_mdio_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_mdio_id_table,
	.probe = enetc_mdio_probe,
	.remove = enetc_mdio_remove,
};
module_pci_driver(enetc_mdio_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
