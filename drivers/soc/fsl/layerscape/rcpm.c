/*
 * Run Control and Power Management (RCPM) driver
 *
 * Copyright 2016 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt) "RCPM: %s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/suspend.h>

/* RCPM register offset */
#define RCPM_IPPDEXPCR0			0x140

#define RCPM_WAKEUP_CELL_SIZE	2

struct rcpm_config {
	int ipp_num;
	int ippdexpcr_offset;
	u32 ippdexpcr[2];
	void *rcpm_reg_base;
};

static struct rcpm_config *rcpm;

static inline void rcpm_reg_write(u32 offset, u32 value)
{
	iowrite32be(value, rcpm->rcpm_reg_base + offset);
}

static inline u32 rcpm_reg_read(u32 offset)
{
	return ioread32be(rcpm->rcpm_reg_base + offset);
}

static void rcpm_wakeup_fixup(struct device *dev, void *data)
{
	struct device_node *node = dev ? dev->of_node : NULL;
	u32 value[RCPM_WAKEUP_CELL_SIZE];
	int ret, i;

	if (!dev || !node || !device_may_wakeup(dev))
		return;

	/*
	 * Get the values in the "rcpm-wakeup" property.
	 * Three values are:
	 * The first is a pointer to the RCPM node.
	 * The second is the value of the ippdexpcr0 register.
	 * The third is the value of the ippdexpcr1 register.
	 */
	ret = of_property_read_u32_array(node, "fsl,rcpm-wakeup",
					 value, RCPM_WAKEUP_CELL_SIZE);
	if (ret)
		return;

	pr_debug("wakeup source: the device %s\n", node->full_name);

	for (i = 0; i < rcpm->ipp_num; i++)
		rcpm->ippdexpcr[i] |= value[i + 1];
}

static int rcpm_suspend_prepare(void)
{
	int i;

	BUG_ON(!rcpm);

	for (i = 0; i < rcpm->ipp_num; i++)
		rcpm->ippdexpcr[i] = 0;

	dpm_for_each_dev(NULL, rcpm_wakeup_fixup);

	for (i = 0; i < rcpm->ipp_num; i++) {
		rcpm_reg_write(rcpm->ippdexpcr_offset + 4 * i,
			       rcpm->ippdexpcr[i]);
		pr_debug("ippdexpcr%d = 0x%x\n", i, rcpm->ippdexpcr[i]);
	}

	return 0;
}

static int rcpm_suspend_notifier_call(struct notifier_block *bl,
				      unsigned long state,
				      void *unused)
{
	switch (state) {
	case PM_SUSPEND_PREPARE:
		rcpm_suspend_prepare();
		break;
	}

	return NOTIFY_DONE;
}

static struct rcpm_config rcpm_default_config = {
	.ipp_num = 1,
	.ippdexpcr_offset = RCPM_IPPDEXPCR0,
};

static const struct of_device_id rcpm_matches[] = {
	{
		.compatible = "fsl,qoriq-rcpm-2.1",
		.data = &rcpm_default_config,
	},
	{}
};

static struct notifier_block rcpm_suspend_notifier = {
	.notifier_call = rcpm_suspend_notifier_call,
};

static int __init layerscape_rcpm_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;

	np = of_find_matching_node_and_match(NULL, rcpm_matches, &match);
	if (!np) {
		pr_err("Can't find the RCPM node.\n");
		return -EINVAL;
	}

	if (match->data)
		rcpm = (struct rcpm_config *)match->data;
	else
		return -EINVAL;

	rcpm->rcpm_reg_base = of_iomap(np, 0);
	of_node_put(np);
	if (!rcpm->rcpm_reg_base)
		return -ENOMEM;

	register_pm_notifier(&rcpm_suspend_notifier);

	pr_info("The RCPM driver initialized.\n");

	return 0;
}

subsys_initcall(layerscape_rcpm_init);
