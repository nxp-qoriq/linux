/*
 * Copyright 2017 NXP
 * Author: Nipun Gupta <nipun.gupta@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include "../include/mc.h"

/* Setup the IOMMU for the DPRC container */
static const struct iommu_ops
*fsl_mc_iommu_configure(struct fsl_mc_device *mc_dev,
	struct device_node *fsl_mc_platform_node)
{
	struct of_phandle_args iommu_spec;
	const struct iommu_ops *ops;
	u32 iommu_phandle;
	struct device_node *iommu_node;
	const __be32 *map = NULL;
	int iommu_cells, map_len, ret;

	map = of_get_property(fsl_mc_platform_node, "iommu-map", &map_len);
	if (!map)
		return NULL;

	ops = mc_dev->dev.bus->iommu_ops;
	if (!ops || !ops->of_xlate)
		return NULL;

	iommu_phandle = be32_to_cpup(map + 1);
	iommu_node = of_find_node_by_phandle(iommu_phandle);

	if (of_property_read_u32(iommu_node, "#iommu-cells", &iommu_cells)) {
		pr_err("%s: missing #iommu-cells property\n", iommu_node->name);
		return NULL;
	}

	/* Initialize the fwspec */
	ret = iommu_fwspec_init(&mc_dev->dev, &iommu_node->fwnode, ops);
	if (ret)
		return NULL;

	/*
	 * Fill in the required stream-id before calling the iommu's
	 * ops->xlate callback.
	 */
	iommu_spec.np = iommu_node;
	iommu_spec.args[0] = mc_dev->icid;
	iommu_spec.args_count = 1;

	ret = ops->of_xlate(&mc_dev->dev, &iommu_spec);
	if (ret)
		return NULL;

	of_node_put(iommu_spec.np);

	return ops;
}

/* Set up DMA configuration for fsl-mc devices */
void fsl_mc_dma_configure(struct fsl_mc_device *mc_dev,
	struct device_node *fsl_mc_platform_node, int coherent)
{
	const struct iommu_ops *ops;

	ops = fsl_mc_iommu_configure(mc_dev, fsl_mc_platform_node);

	mc_dev->dev.coherent_dma_mask = DMA_BIT_MASK(48);
	mc_dev->dev.dma_mask = &mc_dev->dev.coherent_dma_mask;
	arch_setup_dma_ops(&mc_dev->dev, 0,
		mc_dev->dev.coherent_dma_mask + 1, ops, coherent);
}

/* Macro to get the container device of a MC device */
//#define fsl_mc_cont_dev(_dev) ((to_fsl_mc_device(_dev)->flags & \
//	FSL_MC_IS_DPRC) ? (_dev) : ((_dev)->parent))

/* Macro to check if a device is a container device */
#define is_cont_dev(_dev) (to_fsl_mc_device(_dev)->flags & FSL_MC_IS_DPRC)

/* Get the IOMMU group for device on fsl-mc bus */
struct iommu_group *fsl_mc_device_group(struct device *dev)
{
	struct device *cont_dev = fsl_mc_cont_dev(dev);
	struct iommu_group *group;

	/* Container device is responsible for creating the iommu group */
	if (is_cont_dev(dev)) {
		group = iommu_group_alloc();
		if (IS_ERR(group))
			return NULL;
	} else {
		get_device(cont_dev);
		group = iommu_group_get(cont_dev);
		put_device(cont_dev);
	}

	return group;
}
