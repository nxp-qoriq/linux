// SPDX-License-Identifier: GPL-2.0
/*
 * DPAA2 Ethernet Switch ethtool support
 *
 * Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2017-2018 NXP
 *
 */

#include "ethsw.h"

static struct {
	enum dpsw_counter id;
	char name[ETH_GSTRING_LEN];
} ethsw_ethtool_counters[] =  {
	{DPSW_CNT_ING_FRAME,			"rx frames"},
	{DPSW_CNT_ING_BYTE,			"rx bytes"},
	{DPSW_CNT_ING_FLTR_FRAME,		"rx filtered frames"},
	{DPSW_CNT_ING_FRAME_DISCARD,		"rx discarded frames"},
	{DPSW_CNT_ING_BCAST_FRAME,		"rx b-cast frames"},
	{DPSW_CNT_ING_BCAST_BYTES,		"rx b-cast bytes"},
	{DPSW_CNT_ING_MCAST_FRAME,		"rx m-cast frames"},
	{DPSW_CNT_ING_MCAST_BYTE,		"rx m-cast bytes"},
	{DPSW_CNT_EGR_FRAME,			"tx frames"},
	{DPSW_CNT_EGR_BYTE,			"tx bytes"},
	{DPSW_CNT_EGR_FRAME_DISCARD,		"tx discarded frames"},
	{DPDMUX_CNT_ING_NO_BUFFER_DISCARD, 	"rx discarded no buffer frames"},

};

#define ETHSW_NUM_COUNTERS	ARRAY_SIZE(ethsw_ethtool_counters)

static void ethsw_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	u16 version_major, version_minor;
	int err;

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));

	err = dpsw_get_api_version(port_priv->ethsw_data->mc_io, 0,
				   &version_major,
				   &version_minor);
	if (err)
		strlcpy(drvinfo->fw_version, "N/A",
			sizeof(drvinfo->fw_version));
	else
		snprintf(drvinfo->fw_version, sizeof(drvinfo->fw_version),
			 "%u.%u", version_major, version_minor);

	strlcpy(drvinfo->bus_info, dev_name(netdev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int
ethsw_get_link_ksettings(struct net_device *netdev,
			 struct ethtool_link_ksettings *link_ksettings)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct dpsw_link_state state = {0};
	int err = 0;

	err = dpsw_if_get_link_state(port_priv->ethsw_data->mc_io, 0,
				     port_priv->ethsw_data->dpsw_handle,
				     port_priv->idx,
				     &state);
	if (err) {
		netdev_err(netdev, "ERROR %d getting link state\n", err);
		goto out;
	}

	/* At the moment, we have no way of interrogating the DPMAC
	 * from the DPSW side or there may not exist a DPMAC at all.
	 * Report only autoneg state, duplexity and speed.
	 */
	if (state.options & DPSW_LINK_OPT_AUTONEG)
		link_ksettings->base.autoneg = AUTONEG_ENABLE;
	if (!(state.options & DPSW_LINK_OPT_HALF_DUPLEX))
		link_ksettings->base.duplex = DUPLEX_FULL;
	link_ksettings->base.speed = state.rate;

out:
	return err;
}

static int
ethsw_set_link_ksettings(struct net_device *netdev,
			 const struct ethtool_link_ksettings *link_ksettings)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct ethsw_core *ethsw = port_priv->ethsw_data;
	struct dpsw_link_cfg cfg = {0};
	bool if_running;
	int err = 0, ret;

	/* Interface needs to be down to change link settings */
	if_running = netif_running(netdev);
	if (if_running) {
		err = dpsw_if_disable(ethsw->mc_io, 0,
				      ethsw->dpsw_handle,
				      port_priv->idx);
		if (err) {
			netdev_err(netdev, "dpsw_if_disable err %d\n", err);
			return err;
		}
	}

	cfg.rate = link_ksettings->base.speed;
	if (link_ksettings->base.autoneg == AUTONEG_ENABLE)
		cfg.options |= DPSW_LINK_OPT_AUTONEG;
	else
		cfg.options &= ~DPSW_LINK_OPT_AUTONEG;
	if (link_ksettings->base.duplex  == DUPLEX_HALF)
		cfg.options |= DPSW_LINK_OPT_HALF_DUPLEX;
	else
		cfg.options &= ~DPSW_LINK_OPT_HALF_DUPLEX;

	err = dpsw_if_set_link_cfg(port_priv->ethsw_data->mc_io, 0,
				   port_priv->ethsw_data->dpsw_handle,
				   port_priv->idx,
				   &cfg);

	if (if_running) {
		ret = dpsw_if_enable(ethsw->mc_io, 0,
				     ethsw->dpsw_handle,
				     port_priv->idx);
		if (ret) {
			netdev_err(netdev, "dpsw_if_enable err %d\n", ret);
			return ret;
		}
	}
	return err;
}

static int ethsw_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ETHSW_NUM_COUNTERS;
	default:
		return -EOPNOTSUPP;
	}
}

static void ethsw_ethtool_get_strings(struct net_device *netdev,
				      u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ETHSW_NUM_COUNTERS; i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       ethsw_ethtool_counters[i].name, ETH_GSTRING_LEN);
		break;
	}
}

static void ethsw_ethtool_get_stats(struct net_device *netdev,
				    struct ethtool_stats *stats,
				    u64 *data)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	int i, err;

	for (i = 0; i < ETHSW_NUM_COUNTERS; i++) {
		err = dpsw_if_get_counter(port_priv->ethsw_data->mc_io, 0,
					  port_priv->ethsw_data->dpsw_handle,
					  port_priv->idx,
					  ethsw_ethtool_counters[i].id,
					  &data[i]);
		if (err)
			netdev_err(netdev, "dpsw_if_get_counter[%s] err %d\n",
				   ethsw_ethtool_counters[i].name, err);
	}
}

static void ethsw_ethtool_get_pauseparam(struct net_device *netdev,
					 struct ethtool_pauseparam *pause)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct dpsw_link_state state = {0};
	int err = 0;

	err = dpsw_if_get_link_state(port_priv->ethsw_data->mc_io, 0,
				     port_priv->ethsw_data->dpsw_handle,
				     port_priv->idx,
				     &state);
	if (err) {
		netdev_err(netdev, "dpsw_if_get_link_state err %d\n", err);
		return;
	}

	pause->tx_pause = !!(state.options & DPSW_LINK_OPT_PAUSE);
	pause->rx_pause = !!(state.options & DPSW_LINK_OPT_PAUSE);

	pause->autoneg = AUTONEG_DISABLE;
}

static int ethsw_ethtool_set_pauseparam(struct net_device *netdev,
					struct ethtool_pauseparam *pause)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct dpsw_link_state state = {0};
	struct dpsw_link_cfg cfg = {0};
	bool up;
	int err = 0;

	if (pause->autoneg)
		return -EOPNOTSUPP;

	if (!!pause->rx_pause ^ !!pause->tx_pause)
		return -EOPNOTSUPP;

	err = dpsw_if_get_link_state(port_priv->ethsw_data->mc_io, 0,
				     port_priv->ethsw_data->dpsw_handle,
				     port_priv->idx,
				     &state);
	if (err)
		return err;

	/* Interface needs to be down to change link settings */
	up = ethsw_port_is_up(port_priv);
	if (up) {
		err = dpsw_if_disable(port_priv->ethsw_data->mc_io, 0,
				      port_priv->ethsw_data->dpsw_handle,
				      port_priv->idx);
		if (err) {
			netdev_err(netdev, "dpsw_if_disable err %d\n", err);
			return err;
		}
	}

	cfg.rate = state.rate;
	cfg.options = state.options;
	if (pause->tx_pause && pause->rx_pause)
		cfg.options |= DPSW_LINK_OPT_PAUSE;
	else
		cfg.options &= ~DPSW_LINK_OPT_PAUSE;

	err = dpsw_if_set_link_cfg(port_priv->ethsw_data->mc_io, 0,
				   port_priv->ethsw_data->dpsw_handle,
				   port_priv->idx,
				   &cfg);

	if (up) {
		err = dpsw_if_enable(port_priv->ethsw_data->mc_io, 0,
				     port_priv->ethsw_data->dpsw_handle,
				     port_priv->idx);
		if (err) {
			netdev_err(netdev, "dpsw_if_enable err %d\n", err);
			return err;
		}
	}

	return err;
}

const struct ethtool_ops ethsw_port_ethtool_ops = {
	.get_drvinfo		= ethsw_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= ethsw_get_link_ksettings,
	.set_link_ksettings	= ethsw_set_link_ksettings,
	.get_strings		= ethsw_ethtool_get_strings,
	.get_ethtool_stats	= ethsw_ethtool_get_stats,
	.get_sset_count		= ethsw_ethtool_get_sset_count,
	.get_pauseparam		= ethsw_ethtool_get_pauseparam,
	.set_pauseparam		= ethsw_ethtool_set_pauseparam,
};
