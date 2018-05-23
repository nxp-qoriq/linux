/*
 * Simple i2c device MDIO MUX driver
 *
 * Author: Pankaj Bansal <pankaj.bansal@nxp.com>
 *
 * Copyright 2018 NXP
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/mdio-mux.h>
#include <linux/regmap.h>
#include <linux/i2c.h>

struct mdio_mux_i2creg_state {
	void 		*mux_handle;
	struct regmap	*regmap;
	uint32_t	mux_reg;
	uint32_t 	mask;
};

/*
 * MDIO multiplexing switch function
 *
 * This function is called by the mdio-mux layer when it thinks the mdio bus
 * multiplexer needs to switch.
 *
 * 'current_child' is the current value of the mux register (masked via
 * s->mask).
 *
 * 'desired_child' is the value of the 'reg' property of the target child MDIO
 * node.
 *
 * The first time this function is called, current_child == -1.
 *
 * If current_child == desired_child, then the mux is already set to the
 * correct bus.
 */
static int mdio_mux_i2creg_switch_fn(int current_child, int desired_child,
				      void *data)
{
	struct mdio_mux_i2creg_state *s = data;
	int ret = 0;

	if (current_child ^ desired_child) {
		uint32_t x, y;

		ret = regmap_read(s->regmap, s->mux_reg, &x);
		if (ret)
			goto out;
		y = (x & ~s->mask) | desired_child;
		if (x != y) {
			ret = regmap_write(s->regmap, s->mux_reg, y);
			if (ret)
				goto out;
			pr_debug("%s: %x -> %x\n", __func__, x, y);
		}
	}

out:
	return ret;
}

static int mdio_mux_i2creg_probe(struct platform_device *pdev)
{
	struct device_node *np2, *np = pdev->dev.of_node;
	struct mdio_mux_i2creg_state *s;
	const __be32 *iprop;
	int len, ret;
	uint32_t val;

	dev_dbg(&pdev->dev, "probing node %pOF\n", np);

	s = devm_kzalloc(&pdev->dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (IS_ERR(s->regmap)) {
		dev_err(&pdev->dev, "Failed to get parent regmap\n");
		ret = PTR_ERR(s->regmap);
		return ret;
	}

	iprop = of_get_property(np, "reg", &len);
	if (!iprop || len != sizeof(uint32_t)) {
		dev_err(&pdev->dev, "missing or invalid reg property\n");
		return -ENODEV;
	}
	s->mux_reg = (uint32_t)be32_to_cpup(iprop);

	/*Test Register read write*/
	ret = regmap_read(s->regmap, s->mux_reg, &val);
	if (ret) {
		dev_err(&pdev->dev, "error while reading reg\n");
		return ret;
	}

	ret = regmap_write(s->regmap, s->mux_reg, val);
	if (ret) {
		dev_err(&pdev->dev, "error while writing reg\n");
		return ret;
	}

	iprop = of_get_property(np, "mux-mask", &len);
	if (!iprop || len != sizeof(uint32_t)) {
		dev_err(&pdev->dev, "missing or invalid mux-mask property\n");
		return -ENODEV;
	}
	s->mask = (uint32_t)be32_to_cpup(iprop);

	/*
	 * Verify that the 'reg' property of each child MDIO bus does not
	 * set any bits outside of the 'mask'.
	 */
	for_each_available_child_of_node(np, np2) {
		iprop = of_get_property(np2, "reg", &len);
		if (!iprop || len != sizeof(uint32_t)) {
			dev_err(&pdev->dev, "mdio-mux child node %pOF is "
				"missing a 'reg' property\n", np2);
			of_node_put(np2);
			return -ENODEV;
		}
		if (be32_to_cpup(iprop) & ~s->mask) {
			dev_err(&pdev->dev, "mdio-mux child node %pOF has "
				"a 'reg' value with unmasked bits\n",
				np2);
			of_node_put(np2);
			return -ENODEV;
		}
	}

	ret = mdio_mux_init(&pdev->dev, mdio_mux_i2creg_switch_fn,
			    &s->mux_handle, s, NULL);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to register mdio-mux bus %pOF\n", np);
		return ret;
	}

	pdev->dev.platform_data = s;

	return 0;
}

static int mdio_mux_i2creg_remove(struct platform_device *pdev)
{
	struct mdio_mux_i2creg_state *s = dev_get_platdata(&pdev->dev);

	mdio_mux_uninit(s->mux_handle);

	return 0;
}

static const struct of_device_id mdio_mux_i2creg_match[] = {
	{
		.compatible = "mdio-mux-i2creg",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mdio_mux_i2creg_match);

static struct platform_driver mdio_mux_i2creg_driver = {
	.driver = {
		.name		= "mdio-mux-i2creg",
		.of_match_table = mdio_mux_i2creg_match,
	},
	.probe		= mdio_mux_i2creg_probe,
	.remove		= mdio_mux_i2creg_remove,
};

module_platform_driver(mdio_mux_i2creg_driver);

MODULE_AUTHOR("Pankaj Bansal <pankaj.bansal@nxp.com>");
MODULE_DESCRIPTION("I2c device MDIO MUX driver");
MODULE_LICENSE("GPL v2");
