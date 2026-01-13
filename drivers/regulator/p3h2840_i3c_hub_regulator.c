// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * NXP P3H2X4X regulator driver file contain functions for enable/disable regulator
 * and voltage set/get.
 */
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/mfd/p3h2840.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define P3H2X4X_LDO_AND_PULLUP_CONF				0x19
#define P3H2X4X_LDO_ENABLE_DISABLE_MASK				GENMASK(3, 0)
#define P3H2X4X_CP0_EN_LDO				        BIT(0)
#define P3H2X4X_CP1_EN_LDO				        BIT(1)
#define P3H2X4X_TP0145_EN_LDO					BIT(2)
#define P3H2X4X_TP2367_EN_LDO					BIT(3)

#define P3H2X4X_NET_OPER_MODE_CONF				0x15
#define P3H2X4X_VCCIO_LDO_CONF					0x16
#define P3H2X4X_CP0_VCCIO_LDO_VOLTAGE_MASK			GENMASK(1, 0)
#define P3H2X4X_CP0_VCCIO_LDO_VOLTAGE(x)	\
		FIELD_PREP(P3H2X4X_CP0_VCCIO_LDO_VOLTAGE_MASK, x)
#define P3H2X4X_CP1_VCCIO_LDO_VOLTAGE_MASK			GENMASK(3, 2)
#define P3H2X4X_CP1_VCCIO_LDO_VOLTAGE(x)	\
		FIELD_PREP(P3H2X4X_CP1_VCCIO_LDO_VOLTAGE_MASK, x)
#define P3H2X4X_TP0145_VCCIO_LDO_VOLTAGE_MASK			GENMASK(5, 4)
#define P3H2X4X_TP0145_VCCIO_LDO_VOLTAGE(x)	\
		FIELD_PREP(P3H2X4X_TP0145_VCCIO_LDO_VOLTAGE_MASK, x)
#define P3H2X4X_TP2367_VCCIO_LDO_VOLTAGE_MASK			GENMASK(7, 6)
#define P3H2X4X_TP2367_VCCIO_LDO_VOLTAGE(x)	\
		FIELD_PREP(P3H2X4X_TP2367_VCCIO_LDO_VOLTAGE_MASK, x)
#define P3H2X4X_LDO_COUNT					4

struct p3h2x4x_regulator_dev {
	struct regulator_dev *rp3h2x4x_dev[P3H2X4X_LDO_COUNT];
	struct p3h2x4x *p3h2x4x;
	struct regmap *regmap;
};

struct p3h2x4x_reg_state {
	unsigned int orig;
	bool restore;
};

static void p3h2x4x_reg_guard_enter(struct regulator_dev *rdev)
{
	struct p3h2x4x_regulator_dev *priv = rdev_get_drvdata(rdev);

	mutex_lock(&priv->p3h2x4x->protected_reg_lock);
}

static void p3h2x4x_reg_guard_exit(struct regulator_dev *rdev)
{
	struct p3h2x4x_regulator_dev *priv = rdev_get_drvdata(rdev);

	mutex_unlock(&priv->p3h2x4x->protected_reg_lock);
}

DEFINE_LOCK_GUARD_1(p3h2x4x_reg, struct regulator_dev,
		    p3h2x4x_reg_guard_enter(_T->lock),
		    p3h2x4x_reg_guard_exit(_T->lock));

static int p3h2x4x_reg_unprotect(struct regulator_dev *rdev,
				 struct p3h2x4x_reg_state *state)
{
	int ret;

	state->restore = false;

	ret = regmap_read(rdev->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
			  &state->orig);
	if (ret)
		return ret;

	if (state->orig == P3H2X4X_REGISTERS_UNLOCK_CODE)
		return 0;

	ret = regmap_write(rdev->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
			   P3H2X4X_REGISTERS_UNLOCK_CODE);
	if (ret)
		return ret;

	state->restore = true;

	return 0;
}

static int p3h2x4x_reg_protect(struct regulator_dev *rdev,
			       struct p3h2x4x_reg_state *state)
{
	if (!state->restore)
		return 0;

	return regmap_write(rdev->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
			    state->orig);
}

static int p3h2x4x_regulator_enable(struct regulator_dev *rdev)
{
	struct p3h2x4x_reg_state state;
	int ret, ret2;

	guard(p3h2x4x_reg)(rdev);

	ret = p3h2x4x_reg_unprotect(rdev, &state);
	if (ret)
		return ret;

	ret = regulator_enable_regmap(rdev);
	ret2 = p3h2x4x_reg_protect(rdev, &state);

	return ret ? ret : ret2;
}

static int p3h2x4x_regulator_disable(struct regulator_dev *rdev)
{
	struct p3h2x4x_reg_state state;
	int ret, ret2;

	guard(p3h2x4x_reg)(rdev);

	ret = p3h2x4x_reg_unprotect(rdev, &state);
	if (ret)
		return ret;

	ret = regulator_disable_regmap(rdev);
	ret2 = p3h2x4x_reg_protect(rdev, &state);

	return ret ? ret : ret2;
}

static int p3h2x4x_regulator_set_voltage_sel(struct regulator_dev *rdev,
					     unsigned int sel)
{
	struct p3h2x4x_reg_state state;
	int ret, ret2;

	guard(p3h2x4x_reg)(rdev);

	ret = p3h2x4x_reg_unprotect(rdev, &state);
	if (ret)
		return ret;

	ret = regulator_set_voltage_sel_regmap(rdev, sel);
	ret2 = p3h2x4x_reg_protect(rdev, &state);

	return ret ? ret : ret2;
}

static const struct regulator_ops p3h2x4x_ldo_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = p3h2x4x_regulator_set_voltage_sel,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = p3h2x4x_regulator_enable,
	.disable = p3h2x4x_regulator_disable,
	.is_enabled = regulator_is_enabled_regmap,
};

static const unsigned int p3h2x4x_voltage_table[] = {
	1000000,
	1100000,
	1200000,
	1800000,
};

static const struct regulator_desc p3h2x4x_regulators[] = {
	{
		.name = "ldo-cp0",
		.of_match = of_match_ptr("ldo-cp0"),
		.regulators_node = of_match_ptr("regulators"),
		.volt_table = p3h2x4x_voltage_table,
		.n_voltages = ARRAY_SIZE(p3h2x4x_voltage_table),
		.ops = &p3h2x4x_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.enable_reg = P3H2X4X_LDO_AND_PULLUP_CONF,
		.enable_mask = P3H2X4X_CP0_EN_LDO,
		.vsel_reg = P3H2X4X_VCCIO_LDO_CONF,
		.vsel_mask = P3H2X4X_CP0_VCCIO_LDO_VOLTAGE_MASK,
	},
	{
		.name = "ldo-cp1",
		.of_match = of_match_ptr("ldo-cp1"),
		.regulators_node = of_match_ptr("regulators"),
		.volt_table = p3h2x4x_voltage_table,
		.n_voltages = ARRAY_SIZE(p3h2x4x_voltage_table),
		.ops = &p3h2x4x_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.enable_reg = P3H2X4X_LDO_AND_PULLUP_CONF,
		.enable_mask = P3H2X4X_CP1_EN_LDO,
		.vsel_reg = P3H2X4X_VCCIO_LDO_CONF,
		.vsel_mask = P3H2X4X_CP1_VCCIO_LDO_VOLTAGE_MASK,
	},
	{
		.name = "ldo-tpg0",
		.of_match = of_match_ptr("ldo-tpg0"),
		.regulators_node = of_match_ptr("regulators"),
		.volt_table = p3h2x4x_voltage_table,
		.n_voltages = ARRAY_SIZE(p3h2x4x_voltage_table),
		.ops = &p3h2x4x_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.enable_reg = P3H2X4X_LDO_AND_PULLUP_CONF,
		.enable_mask = P3H2X4X_TP0145_EN_LDO,
		.vsel_reg = P3H2X4X_VCCIO_LDO_CONF,
		.vsel_mask = P3H2X4X_TP0145_VCCIO_LDO_VOLTAGE_MASK,
	},
	{
		.name = "ldo-tpg1",
		.of_match = of_match_ptr("ldo-tpg1"),
		.regulators_node = of_match_ptr("regulators"),
		.volt_table = p3h2x4x_voltage_table,
		.n_voltages = ARRAY_SIZE(p3h2x4x_voltage_table),
		.ops = &p3h2x4x_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.enable_reg = P3H2X4X_LDO_AND_PULLUP_CONF,
		.enable_mask = P3H2X4X_TP2367_EN_LDO,
		.vsel_reg = P3H2X4X_VCCIO_LDO_CONF,
		.vsel_mask = P3H2X4X_TP2367_VCCIO_LDO_VOLTAGE_MASK,
	},
};

static int p3h2x4x_regulator_probe(struct platform_device *pdev)
{
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(pdev->dev.parent);
	struct p3h2x4x_regulator_dev *p3h2x4x_regulator;
	struct regulator_config rcfg = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;
	int i;

	p3h2x4x_regulator = devm_kzalloc(dev, sizeof(*p3h2x4x_regulator), GFP_KERNEL);
	if (!p3h2x4x_regulator)
		return -ENOMEM;

	p3h2x4x_regulator->p3h2x4x = p3h2x4x;
	p3h2x4x_regulator->regmap = p3h2x4x->regmap;

	platform_set_drvdata(pdev, p3h2x4x_regulator);

	device_set_of_node_from_dev(dev, dev->parent);

	rcfg.dev = dev;
	rcfg.regmap = p3h2x4x_regulator->regmap;
	rcfg.driver_data = p3h2x4x_regulator;

	for (i = 0; i < ARRAY_SIZE(p3h2x4x_regulators); i++) {
		rdev = devm_regulator_register(&pdev->dev, &p3h2x4x_regulators[i], &rcfg);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev), "Failed to register %s\n",
					     p3h2x4x_regulators[i].name);
		p3h2x4x_regulator->rp3h2x4x_dev[i] = rdev;
	}
	return 0;
}

static const struct platform_device_id p3h2x4x_regulator_id[] = {
	{ "p3h2x4x-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, p3h2x4x_regulator_id);

static struct platform_driver p3h2x4x_regulator_driver = {
	.driver = {
		.name = "p3h2x4x-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = p3h2x4x_regulator_probe,
	.id_table = p3h2x4x_regulator_id,
};
module_platform_driver(p3h2x4x_regulator_driver);

MODULE_AUTHOR("Aman Kumar Pandey <aman.kumarpandey@nxp.com>");
MODULE_AUTHOR("Vikash Bansal <vikash.bansal@nxp.com>");
MODULE_AUTHOR("Lakshay Piplani <lakshay.piplani@nxp.com>");
MODULE_DESCRIPTION("NXP P3H2X4X I3C HUB Regulator driver");
MODULE_LICENSE("GPL");
