// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 *
 * Authors:
 *	Aman Kumar Pandey <aman.kumarpandey@nxp.com>
 *	Vikash Bansal <vikash.bansal@nxp.com>
 *	Lakshay Piplani <lakshay.piplani@nxp.com>
 *
 * NXP P3H2x4x multi-port I3C hub.
 */
#include <linux/i2c.h>
#include <linux/i3c/device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/p3h2840.h>
#include <linux/regmap.h>

static const struct mfd_cell p3h2x4x_devs[] = {
	MFD_CELL_NAME("p3h2x4x-regulator"),
	MFD_CELL_NAME("p3h2x4x-i3c-hub"),
};

static const struct regmap_config p3h2x4x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

/* Read port count from the device capability register (4- or 8-port variant). */
static int p3h2x4x_read_num_target_ports(struct device *dev,
					 struct p3h2x4x *ddata)
{
	unsigned int val;
	int ret;

	ret = regmap_read(ddata->regmap, P3H2X4X_DEV_CAPAB, &val);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read device capability\n");

	ddata->num_target_ports = (val & P3H2X4X_TARGET_PORT_COUNT) ?
		P3H2X4X_TARGET_PORTS_8 : P3H2X4X_TARGET_PORTS_4;

	return 0;
}

static int p3h2x4x_device_probe_i3c(struct i3c_device *i3cdev)
{
	struct device *dev = i3cdev_to_dev(i3cdev);
	struct i3c_device_info devinfo;
	struct p3h2x4x *ddata;
	int ret;

	i3c_device_get_info(i3cdev, &devinfo);

	if (I3C_PID_MANUF_ID(devinfo.pid) != I3C_MANUF_ID_NXP)
		return -ENODEV;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ret = devm_mutex_init(dev, &ddata->protected_reg_lock);
	if (ret)
		return ret;

	i3cdev_set_drvdata(i3cdev, ddata);

	ddata->regmap = devm_regmap_init_i3c(i3cdev, &p3h2x4x_regmap_config);
	if (IS_ERR(ddata->regmap))
		return dev_err_probe(dev, PTR_ERR(ddata->regmap),
				     "Failed to register HUB regmap\n");

	/* The hub child driver retrieves information from i3cdev. */
	ddata->i3cdev = i3cdev;

	ret = p3h2x4x_read_num_target_ports(dev, ddata);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				   p3h2x4x_devs, ARRAY_SIZE(p3h2x4x_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add sub devices\n");

	return 0;
}

static int p3h2x4x_device_probe_i2c(struct i2c_client *client)
{
	struct p3h2x4x *ddata;
	int ret;

	ddata = devm_kzalloc(&client->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ret = devm_mutex_init(&client->dev, &ddata->protected_reg_lock);
	if (ret)
		return ret;

	i2c_set_clientdata(client, ddata);

	ddata->regmap = devm_regmap_init_i2c(client, &p3h2x4x_regmap_config);
	if (IS_ERR(ddata->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(ddata->regmap),
				     "Failed to register HUB regmap\n");

	ddata->i3cdev = NULL;

	ret = p3h2x4x_read_num_target_ports(&client->dev, ddata);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   p3h2x4x_devs, ARRAY_SIZE(p3h2x4x_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(&client->dev, ret, "Failed to add sub devices\n");

	return 0;
}

static const struct i3c_device_id p3h2x4x_i3c_ids[] = {
	I3C_CLASS(I3C_DCR_HUB, NULL),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, p3h2x4x_i3c_ids);

static const struct i2c_device_id p3h2x4x_i2c_id_table[] = {
	{ .name = "nxp-i3c-hub" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, p3h2x4x_i2c_id_table);

static const struct of_device_id p3h2x4x_i2c_of_match[] = {
	{ .compatible = "nxp,p3h2440", },
	{ .compatible = "nxp,p3h2441", },
	{ .compatible = "nxp,p3h2840", },
	{ .compatible = "nxp,p3h2841", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, p3h2x4x_i2c_of_match);

static struct i3c_driver p3h2x4x_i3c = {
	.driver = {
		.name = "p3h2x4x-i3c",
	},
	.probe = p3h2x4x_device_probe_i3c,
	.id_table = p3h2x4x_i3c_ids,
};

static struct i2c_driver p3h2x4x_i2c = {
	.driver = {
		.name = "p3h2x4x-i2c",
		.of_match_table = p3h2x4x_i2c_of_match,
	},
	.probe = p3h2x4x_device_probe_i2c,
	.id_table = p3h2x4x_i2c_id_table,
};
module_i3c_i2c_driver(p3h2x4x_i3c, &p3h2x4x_i2c);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("NXP P3H2X4X I3C HUB multi function driver");
MODULE_LICENSE("GPL");
