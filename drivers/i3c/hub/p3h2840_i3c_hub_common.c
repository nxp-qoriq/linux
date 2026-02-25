// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * This P3H2X4X driver file implements functions for Hub probe and DT parsing.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/mfd/p3h2840.h>
#include <linux/util_macros.h>

#include "p3h2840_i3c_hub.h"

/* LDO voltage DT settings */
#define P3H2X4X_DT_LDO_VOLT_1_0V		1000000
#define P3H2X4X_DT_LDO_VOLT_1_1V		1100000
#define P3H2X4X_DT_LDO_VOLT_1_2V		1200000
#define P3H2X4X_DT_LDO_VOLT_1_8V		1800000

static const int p3h2x4x_pullup_tbl[] = {
	250, 500, 1000, 2000
};

static const int p3h2x4x_io_strength_tbl[] = {
	20, 30, 40, 50
};

static u8 p3h2x4x_pullup_dt_to_reg(int dt_value)
{
	return find_closest(dt_value, p3h2x4x_pullup_tbl,
			  ARRAY_SIZE(p3h2x4x_pullup_tbl));
}

static u8 p3h2x4x_io_strength_dt_to_reg(int dt_value)
{
	return find_closest(dt_value, p3h2x4x_io_strength_tbl,
			  ARRAY_SIZE(p3h2x4x_io_strength_tbl));
}

static int p3h2x4x_configure_pullup(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 pullup;

	pullup = P3H2X4X_TP0145_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp0145_pullup));

	pullup |= P3H2X4X_TP2367_PULLUP_CONF(p3h2x4x_pullup_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp2367_pullup));

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_LDO_AND_PULLUP_CONF,
							  P3H2X4X_PULLUP_CONF_MASK, pullup);
}

static int p3h2x4x_configure_io_strength(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	u8 io_strength;

	io_strength = P3H2X4X_CP0_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.cp0_io_strength));

	io_strength |= P3H2X4X_CP1_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.cp1_io_strength));

	io_strength |= P3H2X4X_TP0145_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp0145_io_strength));

	io_strength |= P3H2X4X_TP2367_IO_STRENGTH(p3h2x4x_io_strength_dt_to_reg
						(p3h2x4x_i3c_hub->hub_config.tp2367_io_strength));

	return regmap_update_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_IO_STRENGTH,
							  P3H2X4X_IO_STRENGTH_MASK, io_strength);
}

static int p3h2x4x_configure_ldo(struct device *dev)
{
	static const char * const supplies[] = {
		"vcc1",
		"vcc2",
		"vcc3",
		"vcc4"
	};
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(supplies); i++) {
		ret = devm_regulator_get_enable_optional(dev, supplies[i]);
		if (ret && ret != -ENODEV)
			return dev_err_probe(dev, ret, "Failed to enable %s\n",
					     supplies[i]);
	}

	/* This delay is required for the regulator to stabilize its output voltage */
	fsleep(5000);

	return 0;
}

static int p3h2x4x_configure_tp(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *hub = dev_get_drvdata(dev);
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(dev->parent);
	u8 mode = 0, smbus = 0, pullup = 0, target_port = 0;
	u8 tp_mask;
	int tp, ret;

	for (tp = 0; tp < p3h2x4x->num_target_ports; tp++) {
		pullup |= hub->hub_config.tp_config[tp].pullup_en ? P3H2X4X_SET_BIT(tp) : 0;
		mode |= (hub->hub_config.tp_config[tp].mode != P3H2X4X_TP_MODE_I3C) ?
			P3H2X4X_SET_BIT(tp) : 0;
		smbus |= (hub->hub_config.tp_config[tp].mode == P3H2X4X_TP_MODE_SMBUS) ?
			 P3H2X4X_SET_BIT(tp) : 0;
		target_port |= (hub->tp_bus[tp].tp_mask == P3H2X4X_SET_BIT(tp)) ?
			       hub->tp_bus[tp].tp_mask : 0;
	}

	/* Only touch the bits for the target ports this variant provides. */
	tp_mask = GENMASK(p3h2x4x->num_target_ports - 1, 0);

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_PULLUP_EN, tp_mask, pullup);
	if (ret)
		return ret;

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_IO_MODE_CONF, tp_mask, mode);
	if (ret)
		return ret;

	ret = regmap_update_bits(hub->regmap, P3H2X4X_TP_SMBUS_AGNT_EN, tp_mask, smbus);
	if (ret)
		return ret;

	if (target_port & ~smbus) {
		ret = regmap_write(hub->regmap, P3H2X4X_CP_MUX_SET,
				   P3H2X4X_CONTROLLER_PORT_MUX_REQ);
		if (ret)
			return ret;
	}

	return regmap_update_bits(hub->regmap, P3H2X4X_TP_ENABLE, tp_mask, target_port);
}

static int p3h2x4x_configure_hw(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *hub = dev_get_drvdata(dev);
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(dev->parent);
	int ret, ret2;

	ret = p3h2x4x_configure_ldo(dev);
	if (ret)
		return ret;

	/* Protect the unlock-modify-lock sequence with the shared MFD lock */
	scoped_guard(mutex, &p3h2x4x->protected_reg_lock) {
		ret = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				   P3H2X4X_REGISTERS_UNLOCK_CODE);
		if (ret)
			break;

		ret = p3h2x4x_configure_pullup(dev);
		if (!ret)
			ret = p3h2x4x_configure_io_strength(dev);
		if (!ret)
			ret = p3h2x4x_configure_tp(dev);

		ret2 = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				    P3H2X4X_REGISTERS_LOCK_CODE);
		if (!ret && ret2)
			ret = ret2;
	}

	return ret;
}

static void p3h2x4x_get_target_port_dt_conf(struct device *dev,
					    const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(dev->parent);
	u64 tp_port;

	for_each_available_child_of_node_scoped(node, dev_node) {
		if (of_property_read_reg(dev_node, 0, &tp_port, NULL))
			continue;

		if (tp_port < p3h2x4x->num_target_ports) {
			if (p3h2x4x_i3c_hub->tp_bus[tp_port].of_node) {
				dev_warn(dev, "Duplicate target port %llu in DT\n", tp_port);
				continue;
			}

			p3h2x4x_i3c_hub->tp_bus[tp_port].of_node = of_node_get(dev_node);
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_mask = P3H2X4X_SET_BIT(tp_port);
			p3h2x4x_i3c_hub->tp_bus[tp_port].p3h2x4x_i3c_hub = p3h2x4x_i3c_hub;
			p3h2x4x_i3c_hub->tp_bus[tp_port].tp_port = tp_port;
		}
	}
}

static int p3h2x4x_parse_tp_dt_settings(struct device *dev,
					const struct device_node *node,
					struct tp_configuration tp_config[])
{
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(dev->parent);
	u64 id;
	int ret;

	for_each_available_child_of_node_scoped(node, tp_node) {
		enum p3h2x4x_tp_mode mode;

		/*
		 * Only "i3c" and "smbus" children describe target ports. Skip any
		 * other child (for example the MFD "regulators" container), which
		 * has no "reg" property.
		 */
		if (of_node_name_eq(tp_node, "i3c"))
			mode = P3H2X4X_TP_MODE_I3C;
		else if (of_node_name_eq(tp_node, "smbus"))
			mode = P3H2X4X_TP_MODE_SMBUS;
		else
			continue;

		ret = of_property_read_reg(tp_node, 0, &id, NULL);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to read reg for %pOF\n",
					     tp_node);

		if (id >= p3h2x4x->num_target_ports)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid target port index %llu\n",
					     id);

		tp_config[id].mode = mode;
		tp_config[id].pullup_en =
			of_property_read_bool(tp_node, "nxp,pullup-enable");
	}

	return 0;
}

static int p3h2x4x_get_hub_dt_conf(struct device *dev,
				   const struct device_node *node)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);

	of_property_read_u32(node, "nxp,tp0145-pullup-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp0145_pullup);
	of_property_read_u32(node, "nxp,tp2367-pullup-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp2367_pullup);
	of_property_read_u32(node, "nxp,cp0-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.cp0_io_strength);
	of_property_read_u32(node, "nxp,cp1-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.cp1_io_strength);
	of_property_read_u32(node, "nxp,tp0145-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp0145_io_strength);
	of_property_read_u32(node, "nxp,tp2367-io-strength-ohms",
			     &p3h2x4x_i3c_hub->hub_config.tp2367_io_strength);

	return p3h2x4x_parse_tp_dt_settings(dev, node,
					    p3h2x4x_i3c_hub->hub_config.tp_config);
}

static void p3h2x4x_default_configuration(struct device *dev)
{
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = dev_get_drvdata(dev);
	int tp_count;

	p3h2x4x_i3c_hub->hub_config.tp0145_pullup = P3H2X4X_DFT_TP_PULLUP_OHMS;
	p3h2x4x_i3c_hub->hub_config.tp2367_pullup = P3H2X4X_DFT_TP_PULLUP_OHMS;
	p3h2x4x_i3c_hub->hub_config.cp0_io_strength = P3H2X4X_DFT_IO_STRENGTH_OHMS;
	p3h2x4x_i3c_hub->hub_config.cp1_io_strength = P3H2X4X_DFT_IO_STRENGTH_OHMS;
	p3h2x4x_i3c_hub->hub_config.tp0145_io_strength = P3H2X4X_DFT_IO_STRENGTH_OHMS;
	p3h2x4x_i3c_hub->hub_config.tp2367_io_strength = P3H2X4X_DFT_IO_STRENGTH_OHMS;

	for (tp_count = 0; tp_count < P3H2X4X_TP_MAX_COUNT; ++tp_count)
		p3h2x4x_i3c_hub->hub_config.tp_config[tp_count].mode = P3H2X4X_TP_MODE_I3C;
}

static void p3h2x4x_unregister_smbus_adapters_action(void *data)
{
	p3h2x4x_unregister_smbus_adapters(data);
}

static void p3h2x4x_put_target_port_of_nodes(void *data)
{
	struct p3h2x4x_i3c_hub_dev *hub = data;
	int tp;

	for (tp = 0; tp < P3H2X4X_TP_MAX_COUNT; tp++) {
		of_node_put(hub->tp_bus[tp].of_node);
		hub->tp_bus[tp].of_node = NULL;
	}
}

static void p3h2x4x_clear_i3c_hub_priv(void *data)
{
	struct p3h2x4x *p3h2x4x = data;

	/* Drop the IBI handler backpointer; see the ordering note at the registration site. */
	p3h2x4x->i3c_hub_priv = NULL;
}

static int p3h2x4x_i3c_hub_probe(struct platform_device *pdev)
{
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
	struct device *dev = &pdev->dev;
	struct device_node *node;
	int ret, i;

	p3h2x4x_i3c_hub = devm_kzalloc(dev, sizeof(*p3h2x4x_i3c_hub), GFP_KERNEL);
	if (!p3h2x4x_i3c_hub)
		return -ENOMEM;

	p3h2x4x_i3c_hub->regmap = p3h2x4x->regmap;
	p3h2x4x_i3c_hub->dev = dev;

	platform_set_drvdata(pdev, p3h2x4x_i3c_hub);
	device_set_of_node_from_dev(dev, dev->parent);

	p3h2x4x_default_configuration(dev);

	ret = devm_mutex_init(dev, &p3h2x4x_i3c_hub->etx_mutex);
	if (ret)
		return ret;

	for (i = 0; i < P3H2X4X_TP_MAX_COUNT; i++) {
		ret = devm_mutex_init(dev, &p3h2x4x_i3c_hub->tp_bus[i].port_mutex);
		if (ret)
			return ret;
	}

	/* get hub node from DT */
	node = dev_of_node(dev);
	if (!node)
		return dev_err_probe(dev, -ENODEV, "No Device Tree entry found\n");

	ret = p3h2x4x_get_hub_dt_conf(dev, node);
	if (ret)
		return ret;

	p3h2x4x_get_target_port_dt_conf(dev, node);

	ret = devm_add_action_or_reset(dev,
				       p3h2x4x_put_target_port_of_nodes,
				       p3h2x4x_i3c_hub);
	if (ret)
		return ret;

	ret = p3h2x4x_configure_hw(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to configure the HUB\n");

	/* Register virtual I3C master controllers for I3C target ports */
	if (p3h2x4x->i3cdev) {
		p3h2x4x_i3c_hub->i3cdev = p3h2x4x->i3cdev;
		/*
		 * Publish the hub context in the MFD parent struct rather than
		 * via i3cdev_set_drvdata(), which would overwrite the parent's
		 * drvdata (struct p3h2x4x) that the IBI handler and other MFD
		 * callbacks rely on. Publish it before p3h2x4x_tp_i3c_algo()
		 * enables IBI, since the IBI handler dereferences it.
		 */
		p3h2x4x->i3c_hub_priv = p3h2x4x_i3c_hub;

		/*
		 * Register the clear action before enabling IBI so that, on the
		 * devm LIFO unwind (probe failure or removal), the pointer is
		 * cleared only after IBI has been disabled and freed.
		 */
		ret = devm_add_action_or_reset(dev, p3h2x4x_clear_i3c_hub_priv,
					       p3h2x4x);
		if (ret)
			return ret;

		ret = p3h2x4x_tp_i3c_algo(p3h2x4x_i3c_hub);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to register i3c bus\n");
	}

	/* Register virtual I2C adapters for SMBus target ports */
	ret = p3h2x4x_tp_smbus_algo(p3h2x4x_i3c_hub);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add i2c adapter\n");

	ret = devm_add_action_or_reset(dev,
				       p3h2x4x_unregister_smbus_adapters_action,
				       p3h2x4x_i3c_hub);
	if (ret)
		return ret;

	return 0;
}

static const struct platform_device_id p3h2x4x_i3c_hub_id[] = {
	{ "p3h2x4x-i3c-hub" },
	{ }
};
MODULE_DEVICE_TABLE(platform, p3h2x4x_i3c_hub_id);

static struct platform_driver p3h2x4x_i3c_hub_driver = {
	.driver = {
		.name = "p3h2x4x-i3c-hub",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = p3h2x4x_i3c_hub_probe,
	.id_table = p3h2x4x_i3c_hub_id,
};
module_platform_driver(p3h2x4x_i3c_hub_driver);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("P3H2X4X I3C HUB driver");
MODULE_LICENSE("GPL");
