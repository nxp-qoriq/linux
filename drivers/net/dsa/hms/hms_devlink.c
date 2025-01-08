// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#include "hms_switch.h"

static size_t hms_config_get_size(struct hms_private *priv)
{
	return sizeof(struct hms_config);
}

static int
hms_region_config_snapshot(struct devlink *dl,
				   const struct devlink_region_ops *ops,
				   struct netlink_ext_ack *extack,
				   u8 **data)
{
	struct dsa_switch *ds = dsa_devlink_to_ds(dl);
	struct hms_private *priv = ds->priv;
	size_t len;

	len = hms_config_get_size(priv);
	*data = kcalloc(len, sizeof(u8), GFP_KERNEL);
	if (!*data)
		return -ENOMEM;

	return hms_xfer_get_cmd(priv, HMS_CMD_SYS_INFO_GET, 0, *data, len);
}

static struct devlink_region_ops hms_region_config_ops = {
	.name = "config",
	.snapshot = hms_region_config_snapshot,
	.destructor = kfree,
};

enum hms_region_id {
	HMS_REGION_CONFIG = 0,
};

struct hms_region {
	const struct devlink_region_ops *ops;
	size_t (*get_size)(struct hms_private *priv);
};

static struct hms_region hms_regions[] = {
	[HMS_REGION_CONFIG] = {
		.ops = &hms_region_config_ops,
		.get_size = hms_config_get_size,
	},
};

int hms_devlink_info_get(struct dsa_switch *ds,
			  struct devlink_info_req *req,
			  struct netlink_ext_ack *extack)
{
	struct hms_private *priv = ds->priv;
	int rc;

	rc = devlink_info_version_fixed_put(req,
				DEVLINK_INFO_VERSION_GENERIC_ASIC_ID,
				priv->info->name);
	return rc;
}

int hms_devlink_setup(struct dsa_switch *ds)
{
	int i, num_regions = ARRAY_SIZE(hms_regions);
	struct hms_private *priv = ds->priv;
	const struct devlink_region_ops *ops;
	struct devlink_region *region;
	u64 size;

	priv->regions = kcalloc(num_regions, sizeof(struct devlink_region *),
				GFP_KERNEL);
	if (!priv->regions)
		return -ENOMEM;

	for (i = 0; i < num_regions; i++) {
		size = hms_regions[i].get_size(priv);
		ops = hms_regions[i].ops;

		region = dsa_devlink_region_create(ds, ops, 1, size);
		if (IS_ERR(region)) {
			while (--i >= 0)
				dsa_devlink_region_destroy(priv->regions[i]);

			kfree(priv->regions);
			return PTR_ERR(region);
		}

		priv->regions[i] = region;
	}

	return 0;
}

void hms_devlink_teardown(struct dsa_switch *ds)
{
	int i, num_regions = ARRAY_SIZE(hms_regions);
	struct hms_private *priv = ds->priv;

	for (i = 0; i < num_regions; i++)
		dsa_devlink_region_destroy(priv->regions[i]);

	kfree(priv->regions);
}
