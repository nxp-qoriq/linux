// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * This P3H2X4X driver file contain functions for I3C virtual Bus creation, connect/disconnect
 * hub network and read/write.
 */
#include <linux/i3c/hub.h>
#include <linux/mfd/p3h2840.h>
#include <linux/regmap.h>

#include "p3h2840_i3c_hub.h"

static inline struct tp_bus *
p3h2x4x_bus_from_controller(struct i3c_master_controller *controller)
{
	struct i3c_hub_controller *hub_controller;

	hub_controller = container_of(controller, struct i3c_hub_controller, controller);

	return container_of(hub_controller, struct tp_bus, hub_controller);
}

static void p3h2x4x_hub_enable_port(struct i3c_master_controller *controller)
{
	struct tp_bus *bus = p3h2x4x_bus_from_controller(controller);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = bus->p3h2x4x_i3c_hub;

	if (p3h2x4x_i3c_hub->hub_config.tp_config[bus->tp_port].always_enable)
		return;

	regmap_set_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP_NET_CON_CONF, bus->tp_mask);
}

static void p3h2x4x_hub_disable_port(struct i3c_master_controller *controller)
{
	struct tp_bus *bus = p3h2x4x_bus_from_controller(controller);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = bus->p3h2x4x_i3c_hub;

	if (p3h2x4x_i3c_hub->hub_config.tp_config[bus->tp_port].always_enable)
		return;

	regmap_clear_bits(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP_NET_CON_CONF, bus->tp_mask);
}

static const struct i3c_hub_ops p3h2x4x_hub_ops = {
	.enable_port = p3h2x4x_hub_enable_port,
	.disable_port = p3h2x4x_hub_disable_port,
};

static void p3h2x4x_unregister_i3c_master(void *data)
{
	struct i3c_master_controller *controller = data;

	i3c_master_unregister(controller);
}

/**
 * p3h2x4x_tp_i3c_algo - Register I3C virtual masters for I3C target ports.
 * @p3h2x4x_hub: p3h2x4x device structure.
 * Return: 0 in case of success, negative error code on failure.
 */
int p3h2x4x_tp_i3c_algo(struct p3h2x4x_i3c_hub_dev *p3h2x4x_hub)
{
	struct i3c_master_controller *parent = i3c_dev_get_master(p3h2x4x_hub->i3cdev->desc);
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(p3h2x4x_hub->dev->parent);
	u8 tp, ntwk_mask = 0;
	int ret;

	p3h2x4x_hub->hub = devm_kzalloc(p3h2x4x_hub->dev,
					sizeof(*p3h2x4x_hub->hub),
					GFP_KERNEL);

	if (!p3h2x4x_hub->hub)
		return -ENOMEM;

	i3c_hub_init(p3h2x4x_hub->hub,
		     &p3h2x4x_hub_ops,
		     p3h2x4x_hub->i3cdev);

	for (tp = 0; tp < p3h2x4x->num_target_ports; tp++) {
		if (!p3h2x4x_hub->tp_bus[tp].of_node ||
		    p3h2x4x_hub->hub_config.tp_config[tp].mode != P3H2X4X_TP_MODE_I3C)
			continue;

		struct i3c_hub_controller *hub_controller =
				&p3h2x4x_hub->tp_bus[tp].hub_controller;
		struct i3c_master_controller *controller = &hub_controller->controller;

		hub_controller->parent = parent;
		hub_controller->hub = p3h2x4x_hub->hub;

		dev_set_drvdata(&controller->dev, hub_controller);

		ret = i3c_hub_reserve_parent_addrslots_from_dt(hub_controller,
							       p3h2x4x_hub->tp_bus[tp].of_node);
		if (ret)
			return ret;

		ret = i3c_master_register_fwnode(controller,
						 p3h2x4x_hub->dev,
						 of_fwnode_handle(p3h2x4x_hub->tp_bus[tp].of_node),
						 i3c_hub_master_ops(),
						 false);

		if (ret)
			return ret;

		ret = devm_add_action_or_reset(p3h2x4x_hub->dev,
					       p3h2x4x_unregister_i3c_master,
					       controller);
		if (ret)
			return ret;

		ntwk_mask |= p3h2x4x_hub->tp_bus[tp].tp_mask;
		p3h2x4x_hub->tp_bus[tp].is_registered = true;
		p3h2x4x_hub->hub_config.tp_config[tp].always_enable = true;
	}
	return regmap_write(p3h2x4x_hub->regmap, P3H2X4X_TP_NET_CON_CONF, ntwk_mask);
}
