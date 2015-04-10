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
#include <linux/fsl/qixis_ctrl.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

#ifdef CONFIG_PPC
#include <asm/machdep.h>
#endif

static struct regmap *qixis_regmap;

/* QIXIS Power Management Control*/
#define QIXIS_PWR_CTL2_PWR	0x80
static void fsl_qixis_power_off(void)
{
	u32 val;

	local_irq_disable();

	regmap_read(qixis_regmap, offsetof(struct fsl_qixis_regs, pwr_ctrl2),
		    &val);
	val |= QIXIS_PWR_CTL2_PWR;
	regmap_write(qixis_regmap, offsetof(struct fsl_qixis_regs, pwr_ctrl2),
		     val);

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

static struct regmap_config qixis_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int fsl_qixis_probe(struct platform_device *pdev)
{
	static struct fsl_qixis_regs __iomem *qixis;
	struct resource *res;
	u32 qver;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	qixis = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(qixis)) {
		pr_err("%s: Could not map qixis registers\n", __func__);
		return PTR_ERR(qixis);
	}

	qixis_regmap = devm_regmap_init_mmio_clk(&pdev->dev, NULL, qixis,
						 &qixis_regmap_config);

	fsl_qixis_pm_init();

	regmap_read(qixis_regmap, offsetof(struct fsl_qixis_regs, qixis_ver),
		    &qver);

	pr_info("Freescale QIXIS Version: 0x%08x\n", qver);

	return 0;
}

static int fsl_qixis_remove(struct platform_device *pdev)
{
	fsl_qixis_pm_release();

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

static int fsl_qixis_i2c_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	u32 qver;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	qixis_regmap = regmap_init_i2c(client, &qixis_regmap_config);

	fsl_qixis_pm_init();

	regmap_read(qixis_regmap, offsetof(struct fsl_qixis_regs, qixis_ver),
		    &qver);

	pr_info("Freescale QIXIS Version: 0x%08x\n", qver);

	return 0;
}

static int fsl_qixis_i2c_remove(struct i2c_client *client)
{
	fsl_qixis_pm_release();

	return 0;
}

static const struct i2c_device_id fsl_qixis_id[] = {
	{ "fpga-qixis-i2c", 0 },
	{ "bsc9132qds-fpga", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, fsl_qixis_id);

static struct i2c_driver fsl_qixis_i2c_driver = {
	.driver = {
		.name	= "fpga-qixis-i2c",
		.owner	= THIS_MODULE,
	},
	.probe		= fsl_qixis_i2c_probe,
	.remove		= fsl_qixis_i2c_remove,
	.id_table	= fsl_qixis_id,
};
module_i2c_driver(fsl_qixis_i2c_driver);

MODULE_AUTHOR("Wang Dongsheng <dongsheng.wang@freescale.com>");
MODULE_DESCRIPTION("Freescale QIXIS system controller driver");
MODULE_LICENSE("GPL v2");
