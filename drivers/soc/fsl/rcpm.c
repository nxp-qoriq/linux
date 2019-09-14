// SPDX-License-Identifier: GPL-2.0
//
// rcpm.c - Freescale QorIQ RCPM driver
//
// Copyright 2019 NXP
//
// Author: Ran Wang <ran.wang_1@nxp.com>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define RCPM_WAKEUP_CELL_MAX_SIZE	7

struct rcpm {
	unsigned int	wakeup_cells;
	void __iomem	*ippdexpcr_base;
	bool		little_endian;
};

static int rcpm_pm_prepare(struct device *dev)
{
	struct device_node	*np = dev->of_node;
	struct wakeup_source	*ws;
	struct rcpm		*rcpm;
	u32 value[RCPM_WAKEUP_CELL_MAX_SIZE + 1], tmp;
	int i, ret, idx;
	struct regmap *scfg_addr_regmap = NULL;
	u32 reg_offset[RCPM_WAKEUP_CELL_MAX_SIZE + 1];
	u32 reg_value = 0;

	rcpm = dev_get_drvdata(dev);
	if (!rcpm)
		return -EINVAL;

	/* Begin with first registered wakeup source */
	ws = wakeup_source_get_start(&idx);
	do {
		/* skip object which is not attached to device */
		if (!ws->dev)
			continue;

		ret = device_property_read_u32_array(ws->dev,
				"fsl,rcpm-wakeup", value, rcpm->wakeup_cells + 1);

		/*  Wakeup source should refer to current rcpm device */
		if (ret || (np->phandle != value[0])) {
			dev_info(dev, "%s doesn't refer to this rcpm\n",
					ws->name);
			continue;
		}

		for (i = 0; i < rcpm->wakeup_cells; i++) {
			/* We can only OR related bits */
			if (value[i + 1]) {
				if (rcpm->little_endian) {
					tmp = ioread32(rcpm->ippdexpcr_base + i * 4);
					tmp |= value[i + 1];
					iowrite32(tmp, rcpm->ippdexpcr_base + i * 4);
				} else {
					tmp = ioread32be(rcpm->ippdexpcr_base + i * 4);
					tmp |= value[i + 1];
					iowrite32be(tmp, rcpm->ippdexpcr_base + i * 4);
				}
				/* Workaround of errata A-008646 on SoC LS1021A:
				 * There is a bug of register ippdexpcr1.
				 * Reading configuration register RCPM_IPPDEXPCR1
				 * always return zero. So save ippdexpcr1's value
				 * to register SCFG_SPARECR8.And the value of
				 * ippdexpcr1 will be read from SCFG_SPARECR8.
				 */
				scfg_addr_regmap = syscon_regmap_lookup_by_phandle(np,
										   "fsl,ippdexpcr1-alt-addr");
				if (scfg_addr_regmap && (1 == i)) {
					if (of_property_read_u32_array(dev->of_node,
					    "fsl,ippdexpcr1-alt-addr",
					    reg_offset,
					    1 + sizeof(u64)/sizeof(u32))) {
						scfg_addr_regmap = NULL;
						continue;
					}
					/* Read value from register SCFG_SPARECR8 */
					regmap_read(scfg_addr_regmap,
						    (u32)(((u64)(reg_offset[1] << (sizeof(u32) * 8) |
						    reg_offset[2])) & 0xffffffff),
						    &reg_value);
					/* Write value to register SCFG_SPARECR8 */
					regmap_write(scfg_addr_regmap,
						     (u32)(((u64)(reg_offset[1] << (sizeof(u32) * 8) |
						     reg_offset[2])) & 0xffffffff),
						     tmp | reg_value);
				}
			}
		}
	} while (ws = wakeup_source_get_next(ws));

	wakeup_source_get_stop(idx);

	return 0;
}

static const struct dev_pm_ops rcpm_pm_ops = {
	.prepare =  rcpm_pm_prepare,
};

static int rcpm_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	struct resource *r;
	struct rcpm	*rcpm;
	int ret;

	rcpm = devm_kzalloc(dev, sizeof(*rcpm), GFP_KERNEL);
	if (!rcpm)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENODEV;

	rcpm->ippdexpcr_base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(rcpm->ippdexpcr_base)) {
		ret =  PTR_ERR(rcpm->ippdexpcr_base);
		return ret;
	}

	rcpm->little_endian = device_property_read_bool(
			&pdev->dev, "little-endian");

	ret = device_property_read_u32(&pdev->dev,
			"#fsl,rcpm-wakeup-cells", &rcpm->wakeup_cells);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, rcpm);

	return 0;
}

static const struct of_device_id rcpm_of_match[] = {
	{ .compatible = "fsl,qoriq-rcpm-2.1+", },
	{}
};
MODULE_DEVICE_TABLE(of, rcpm_of_match);

static struct platform_driver rcpm_driver = {
	.driver = {
		.name = "rcpm",
		.of_match_table = rcpm_of_match,
		.pm	= &rcpm_pm_ops,
	},
	.probe = rcpm_probe,
};

module_platform_driver(rcpm_driver);
