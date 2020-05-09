// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2014 Sean Cross <xobs@kosagi.com>
 *
 * Rework for mainline: Marek Vasut <marex@denx.de>
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_panel.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct it6251_bridge {
	struct i2c_client	*client;
	struct i2c_client	*lvds_client;
	struct regmap		*regmap;
	struct regmap		*lvds_regmap;
	struct regulator	*regulator;

	struct drm_connector	connector;
	struct drm_bridge	bridge;
	struct drm_panel	*panel;
};

/* Register definitions */
#define IT6251_VENDOR_ID_LOW			0x00
#define IT6251_VENDOR_ID_HIGH			0x01
#define IT6251_DEVICE_ID_LOW			0x02
#define IT6251_DEVICE_ID_HIGH			0x03
#define IT6251_SYSTEM_STATUS			0x0d
#define IT6251_SYSTEM_STATUS_RINTSTATUS		BIT(0)
#define IT6251_SYSTEM_STATUS_RHPDSTATUS		BIT(1)
#define IT6251_SYSTEM_STATUS_RVIDEOSTABLE	BIT(2)
#define IT6251_SYSTEM_STATUS_RPLL_IOLOCK	BIT(3)
#define IT6251_SYSTEM_STATUS_RPLL_XPLOCK	BIT(4)
#define IT6251_SYSTEM_STATUS_RPLL_SPLOCK	BIT(5)
#define IT6251_SYSTEM_STATUS_RAUXFREQ_LOCK	BIT(6)
#define IT6251_REF_STATE			0x0e
#define IT6251_REF_STATE_MAIN_LINK_DISABLED	BIT(0)
#define IT6251_REF_STATE_AUX_CHANNEL_READ	BIT(1)
#define IT6251_REF_STATE_CR_PATTERN		BIT(2)
#define IT6251_REF_STATE_EQ_PATTERN		BIT(3)
#define IT6251_REF_STATE_NORMAL_OPERATION	BIT(4)
#define IT6251_REF_STATE_MUTED			BIT(5)
#define IT6251_RPCLK_CNT_LOW			0x13
#define IT6251_RPCLK_CNT_HIGH			0x14
#define IT6251_RPC_REQ				0x2b
#define IT6251_RPC_REQ_RPC_FIFOFULL		BIT(6)
#define IT6251_RPC_REQ_RPC_FIFOEMPTY		BIT(7)
#define IT6251_PCLK_CNT_LOW			0x57
#define IT6251_PCLK_CNT_HIGH			0x58
#define IT6251_DPHDEW_LOW			0xa5
#define IT6251_DPHDEW_HIGH			0xa6
#define IT6251_DPVDEW_LOW			0xaf
#define IT6251_DPVDEW_HIGH			0xb0
#define IT6251_LVDS_PORT_ADDR			0xfd
#define IT6251_LVDS_PORT_CTRL			0xfe
#define IT6251_LVDS_PORT_CTRL_EN		BIT(0)

/*
 * Register programming sequences.
 * NOTE: There is a lot of registers here which are completely undocumented
 *       and/or their meaning is not clear from the little documentation
 *       that is available for this chip. These values below just seem to
 *       work well enough.
 */
static const struct reg_sequence it6251_lvds_rx_sequence[] = {
	{ 0x05, 0x00 },

	{ 0x3b, 0x42 },	/* reset LVDSRX PLL */
	{ 0x3b, 0x43 },

	{ 0x3c, 0x08 },	/* something with SSC PLL */
	{ 0x0b, 0x88 },	/* don't swap links, writing reserved regs */

	{ 0x2c, 0x01 },	/* JEIDA, 8-bit depth 0x11, original 0x42 */
	{ 0x32, 0x04 },	/* "reserved" */
	{ 0x35, 0xe0 },	/* "reserved" */
	{ 0x2b, 0x24 },	/* "reserved" + clock delay */

	{ 0x05, 0x02 },	/* reset LVDSRX pix clock */
	{ 0x05, 0x00 },
};

static const struct reg_sequence it6251_edp_tx_sequence[] = {
	/* two lane mode, normal operation, no swapping, no downspread */
	{ 0x16, 0x02 },
	{ 0x23, 0x40 },	/* some AUX channel EDID magic */
	{ 0x5c, 0xf3 },	/* power down lanes 3-0 */
	{ 0x5f, 0x06 },	/* enable DP scrambling, change EQ CR phase */
	{ 0x60, 0x02 },	/* color mode RGB, pclk/2 */
	{ 0x61, 0x04 },	/* dual pixel input mode, no EO swap, no RGB swap */
	{ 0x62, 0x01 },	/* M444B24 video format */

	/* vesa range / not interlace / vsync high / hsync high */
	{ 0xa0, 0x0F },

	{ 0xc9, 0xf5 },	/* hpd event timer set to 1.6-ish ms */

	{ 0xca, 0x4d },	/* more reserved magic */
	{ 0xcb, 0x37 },

	/* enhanced framing mode, auto video fifo reset, video mute disable */
	{ 0xd3, 0x03 },
	{ 0xd4, 0x45 },	/* "vidstmp" and some reserved stuff */

	{ 0xe7, 0xa0 },	/* queue number -- reserved */
	{ 0xe8, 0x33 },	/* info frame packets and reserved */
	{ 0xec, 0x00 },	/* more AVI stuff */

	{ 0x23, 0x42 },	/* select PC master reg for aux channel? */

	{ 0x24, 0x00 },	/* send PC request commands */
	{ 0x25, 0x00 },
	{ 0x26, 0x00 },

	{ 0x2b, 0x00 },	/* native aux read */
	{ 0x23, 0x40 },	/* back to internal */

	{ 0x19, 0xff },	/* voltage swing level 3 */
	{ 0x1a, 0xff },	/* pre-emphasis level 3 */

	{ 0x17, 0x01 },	/* start link training */
};

static struct it6251_bridge *bridge_to_it6251(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6251_bridge, bridge);
}

static struct it6251_bridge *conn_to_it6251(struct drm_connector *connector)
{
	return container_of(connector, struct it6251_bridge, connector);
}

static int it6251_is_stable(struct it6251_bridge *it6251)
{
	unsigned int status, rpclkcnt, clkcnt, refstate, rpcreq;
	u16 hactive;
	u16 vactive;
	u8 regs[2];
	int ret;

	ret = regmap_read(it6251->regmap, IT6251_SYSTEM_STATUS, &status);
	if (ret)
		return ret;
	dev_dbg(&it6251->client->dev, "System status: 0x%02x\n", status);

	if (!(status & IT6251_SYSTEM_STATUS_RVIDEOSTABLE))
		return -EINVAL;

	ret = regmap_bulk_read(it6251->regmap, IT6251_RPCLK_CNT_LOW, regs, 2);
	if (ret)
		return ret;
	rpclkcnt = (regs[0] & 0xff) | ((regs[1] & 0x0f) << 8);
	dev_dbg(&it6251->client->dev, "RPCLKCnt: %d\n", rpclkcnt);

	ret = regmap_bulk_read(it6251->lvds_regmap, IT6251_PCLK_CNT_LOW,
			       regs, 2);
	if (ret)
		return ret;
	clkcnt = (regs[0] & 0xff) | ((regs[1] & 0x0f) << 8);
	dev_dbg(&it6251->client->dev, "Clock: 0x%02x\n", clkcnt);

	ret = regmap_read(it6251->lvds_regmap, IT6251_REF_STATE, &refstate);
	if (ret)
		return ret;
	dev_dbg(&it6251->client->dev, "Ref Link State: 0x%02x\n", refstate);

	ret = regmap_read(it6251->lvds_regmap, IT6251_RPC_REQ, &rpcreq);
	if (ret)
		return ret;
	dev_dbg(&it6251->client->dev, "RPC Req: 0x%02x\n", rpcreq);

	ret = regmap_bulk_read(it6251->regmap, IT6251_DPHDEW_LOW, regs, 2);
	if (ret)
		return ret;
	hactive = (regs[0] & 0xff) | ((regs[1] & 0x1f) << 8);
	dev_dbg(&it6251->client->dev, "hactive: %d\n", hactive);

	ret = regmap_bulk_read(it6251->regmap, IT6251_DPVDEW_LOW, regs, 2);
	if (ret)
		return ret;
	vactive = (regs[0] & 0xff) | ((regs[1] & 0x0f) << 8);
	dev_dbg(&it6251->client->dev, "vactive: %d\n", vactive);

	if ((refstate & 0x1f) != 0)
		return -EINVAL;

	if (rpcreq & IT6251_RPC_REQ_RPC_FIFOFULL) {
		dev_err(&it6251->client->dev,
			"RPC fifofull is set, might be an error\n");
		return -EINVAL;
	}

	/* If video is muted, that's a failure */
	if (refstate & IT6251_REF_STATE_MUTED)
		return -EINVAL;

	return 0;
}

static int it6251_init(struct it6251_bridge *it6251)
{
	const struct reg_sequence it6251_reset_reg_sequence[] = {
		{ 0x05, 0x00 },
		{ IT6251_LVDS_PORT_ADDR, it6251->lvds_client->addr << 1 },
		{ IT6251_LVDS_PORT_CTRL, IT6251_LVDS_PORT_CTRL_EN },
	};

	int ret, stable_delays;
	unsigned int reg;

	/*
	 * Reset DisplayPort half. Setting bit 2 causes IT6251 to not
	 * respond over i2c, which is considered "normal". This write
	 * will report failure, but will actually succeed.
	 */
	regmap_write(it6251->regmap, 0x05, 0xff);

	/* Un-reset DisplayPort half and configure LVDS receiver. */
	ret = regmap_multi_reg_write(it6251->regmap, it6251_reset_reg_sequence,
				     ARRAY_SIZE(it6251_reset_reg_sequence));
	if (ret) {
		dev_err(&it6251->client->dev, "cannot setup eDP half\n");
		return ret;
	}

	/* LVDS RX */
	regmap_write(it6251->lvds_regmap, 0x05, 0xff);
	ret = regmap_multi_reg_write(it6251->lvds_regmap,
				     it6251_lvds_rx_sequence,
				     ARRAY_SIZE(it6251_lvds_rx_sequence));
	if (ret) {
		dev_err(&it6251->lvds_client->dev, "cannot setup LVDS RX\n");
		return ret;
	}

	/* eDP TX */
	ret = regmap_multi_reg_write(it6251->regmap,
				     it6251_edp_tx_sequence,
				     ARRAY_SIZE(it6251_edp_tx_sequence));
	if (ret) {
		dev_err(&it6251->client->dev, "cannot setup eDP TX\n");
		return ret;
	}

	for (stable_delays = 0; stable_delays < 100; stable_delays++) {
		ret = regmap_read(it6251->regmap, 0x0e, &reg);
		if (ret || ((reg & 0x1f) != 0x10)) {
			mdelay(2);
			continue;
		}

		ret = regmap_read(it6251->regmap, IT6251_SYSTEM_STATUS, &reg);
		if (ret || !(reg & IT6251_SYSTEM_STATUS_RVIDEOSTABLE)) {
			mdelay(2);
			continue;
		}

		break;
	}

	/*
	 * If we couldn't stabilize, requeue and try again, because it means
	 * that the LVDS channel isn't stable yet.
	 */
	ret = it6251_is_stable(it6251);
	if (ret)
		dev_err(&it6251->client->dev, "bridge is not stable\n");

	return ret;
}

static int it6251_power_down(struct it6251_bridge *it6251)
{
	struct device *dev = &it6251->client->dev;
	int ret = 0;

	if (regulator_is_enabled(it6251->regulator)) {
		ret = regulator_disable(it6251->regulator);
		if (ret)
			dev_err(dev, "unable to disable regulator\n");
	}

	return ret;
}

static int it6251_power_up(struct it6251_bridge *it6251)
{
	struct i2c_client *client = it6251->client;
	u8 regs[4];
	int i, ret;

	ret = regulator_enable(it6251->regulator);
	if (ret) {
		dev_err(&client->dev, "unable to enable regulator\n");
		return ret;
	}

	/* Sometimes it seems like multiple tries are needed */
	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(it6251->regmap, IT6251_VENDOR_ID_LOW,
				       regs, 4);
		if (!ret && regs[0] && regs[1] && regs[2] && regs[3]) {
			dev_info(&client->dev, "found ITE6251 [%04x:%04x]\n",
				 (regs[1] << 8) | regs[0],
				 (regs[3] << 8) | regs[2]);
			return 0;
		}

		usleep_range(100000, 200000);
	}

	dev_err(&client->dev, "unable to read product id\n");
	it6251_power_down(it6251);
	return -EINVAL;
}

/* I2C driver functions */
static void it6251_pre_enable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	if (drm_panel_prepare(it6251->panel)) {
		DRM_ERROR("failed to prepare panel\n");
		return;
	}

	it6251_power_up(it6251);
}

static void it6251_enable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int tries, ret;

	if (drm_panel_enable(it6251->panel)) {
		DRM_ERROR("failed to enable panel\n");
		return;
	}

	for (tries = 0; tries < 5; tries++) {
		ret = it6251_init(it6251);
		if (!ret)
			return;

		/* If the init failed, restart the chip */
		it6251_power_down(it6251);
		it6251_power_up(it6251);
	}
}

static void it6251_disable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	if (drm_panel_disable(it6251->panel))
		DRM_ERROR("failed to disable panel\n");
}

static void it6251_post_disable(struct drm_bridge *bridge)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	if (drm_panel_unprepare(it6251->panel))
		DRM_ERROR("failed to unprepare panel\n");

	it6251_power_down(it6251);
}

static int it6251_get_modes(struct drm_connector *connector)
{
	struct it6251_bridge *it6251 = conn_to_it6251(connector);

	return drm_panel_get_modes(it6251->panel, connector);
}

static const struct drm_connector_helper_funcs it6251_connector_helper_funcs = {
	.get_modes	= it6251_get_modes,
};

static const struct drm_connector_funcs it6251_connector_funcs = {
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static int it6251_attach(struct drm_bridge *bridge, enum drm_bridge_attach_flags flags)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	it6251->connector.polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_init(bridge->dev, &it6251->connector,
				 &it6251_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_atomic_helper_connector_reset(&it6251->connector);
	drm_connector_helper_add(&it6251->connector,
				 &it6251_connector_helper_funcs);
	drm_connector_attach_encoder(&it6251->connector, bridge->encoder);

	if (it6251->panel)
		drm_panel_attach(it6251->panel, &it6251->connector);

	drm_helper_hpd_irq_event(it6251->connector.dev);

	return 0;
}

static const struct drm_bridge_funcs it6251_bridge_funcs = {
	.pre_enable	= it6251_pre_enable,
	.enable		= it6251_enable,
	.disable	= it6251_disable,
	.post_disable	= it6251_post_disable,
	.attach		= it6251_attach,
};

static const struct regmap_config it6251_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0xff,
	.cache_type	= REGCACHE_NONE,
};

static int
it6251_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct it6251_bridge *it6251;
	struct device_node *endpoint, *panel_node;
	int ret;

	it6251 = devm_kzalloc(dev, sizeof(*it6251), GFP_KERNEL);
	if (!it6251)
		return -ENOMEM;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (endpoint) {
		panel_node = of_graph_get_remote_port_parent(endpoint);
		if (panel_node) {
			it6251->panel = of_drm_find_panel(panel_node);
			of_node_put(panel_node);
			if (!it6251->panel)
				return -EPROBE_DEFER;
		}
	}

	it6251->client = client;

	it6251->regmap = devm_regmap_init_i2c(client, &it6251_regmap_config);
	if (IS_ERR(it6251->regmap)) {
		dev_err(dev, "cannot init i2c regmap for IT6251\n");
		return PTR_ERR(it6251->regmap);
	}

	it6251->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(it6251->regulator)) {
		dev_err(dev, "no power regulator found for IT6251\n");
		return PTR_ERR(it6251->regulator);
	}

	/* The LVDS-half of the chip shows up at address 0x5e */
	it6251->lvds_client = i2c_new_ancillary_device(client, "lvds", 0x5e);
	if (IS_ERR(it6251->lvds_client)) {
		dev_err(dev, "cannot create I2C device for IT6251 LVDS\n");
		return PTR_ERR(it6251->lvds_client);
	}

	it6251->lvds_regmap = regmap_init_i2c(it6251->lvds_client,
					      &it6251_regmap_config);
	if (IS_ERR(it6251->lvds_regmap)) {
		dev_err(dev, "cannot init i2c regmap for IT6251 LVDS\n");
		ret = PTR_ERR(it6251->lvds_regmap);
		goto err_lvds_regmap;
	}

	i2c_set_clientdata(client, it6251);

	it6251->bridge.funcs = &it6251_bridge_funcs;
	it6251->bridge.of_node = dev->of_node;
	drm_bridge_add(&it6251->bridge);

	return 0;

err_lvds_regmap:
	i2c_unregister_device(it6251->lvds_client);
	return ret;
}

static int it6251_remove(struct i2c_client *client)
{
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);
	int ret;

	ret = it6251_power_down(it6251);
	if (ret)
		return ret;

	regmap_exit(it6251->lvds_regmap);
	i2c_unregister_device(it6251->lvds_client);

	return 0;
}

static int it6251_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);

	return it6251_power_down(it6251);
}

static int it6251_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct it6251_bridge *it6251 = i2c_get_clientdata(client);

	return it6251_power_up(it6251);
}

static const struct dev_pm_ops it6251_dev_pm_ops = {
	.suspend = it6251_pm_suspend,
	.resume = it6251_pm_resume,
	.restore = it6251_pm_resume,
};

static struct i2c_device_id it6251_ids[] = {
	{ "it6251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it6251_ids);

static const struct of_device_id it6251_of_match[] = {
	{ .compatible = "ite,it6251", },
	{ }
};
MODULE_DEVICE_TABLE(of, it6251_of_match);

static struct i2c_driver it6251_driver = {
	.driver = {
		.name		= "it6251",
		.pm		= &it6251_dev_pm_ops,
		.of_match_table	= it6251_of_match,
	},
	.probe		= it6251_probe,
	.remove		= it6251_remove,
	.id_table	= it6251_ids,
};

module_i2c_driver(it6251_driver);

/* Module initialization */
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("ITE Tech 6251 LVDS to DisplayPort encoder");
MODULE_LICENSE("GPL");
