/*
 * Freescale QIXIS system controller driver.
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/fsl/qixis_ctrl.h>

#ifdef CONFIG_PPC
#include <asm/machdep.h>
#endif

static struct fsl_qixis_regs __iomem *qixis;

/* QIXIS Power Management Control*/
#define QIXIS_PWR_CTL2_PWR	0x80
static void fsl_qixis_power_off(void)
{
	local_irq_disable();

	iowrite8(ioread8(&qixis->pwr_ctrl2) | QIXIS_PWR_CTL2_PWR,
		 &qixis->pwr_ctrl2);

	while (1)
		;
}

static void fsl_qixis_pm_init(void)
{
#if defined(CONFIG_PPC)
	ppc_md.power_off = fsl_qixis_power_off;
	ppc_md.halt = fsl_qixis_power_off;
#elif defined(CONFIG_ARM)
	pm_power_off = fsl_qixis_power_off;
#endif
}

static void fsl_qixis_pm_release(void)
{
#if defined(CONFIG_PPC)
	ppc_md.power_off = NULL;
	ppc_md.halt = NULL;
#elif defined(CONFIG_ARM)
	pm_power_off = NULL;
#endif
}

static int fsl_qixis_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	qixis = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qixis)) {
		pr_err("%s: Could not map qixis registers\n", __func__);
		return PTR_ERR(qixis);
	}

	fsl_qixis_pm_init();

	return 0;
}

static int fsl_qixis_remove(struct platform_device *pdev)
{
	fsl_qixis_pm_release();

	iounmap(qixis);

	return 0;
}

static const struct of_device_id fsl_qixis_table[] = {
	{ .compatible = "fsl,fpga-qixis", },
	{ .compatible = "fsl,ls1021aqds-fpga", },
	{},
};

static struct platform_driver fsl_qixis_driver = {
	.driver = {
		.name = "fsl-qixis",
		.owner = THIS_MODULE,
		.of_match_table = fsl_qixis_table,
	},
	.probe = fsl_qixis_probe,
	.remove = fsl_qixis_remove,
};
module_platform_driver(fsl_qixis_driver);

MODULE_AUTHOR("Wang Dongsheng <dongsheng.wang@freescale.com>");
MODULE_DESCRIPTION("Freescale QIXIS system controller driver");
MODULE_LICENSE("GPL v2");
