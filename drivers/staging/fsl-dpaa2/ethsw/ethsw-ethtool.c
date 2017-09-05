/* Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ethsw.h"

static struct {
	enum dpsw_counter id;
	char name[ETH_GSTRING_LEN];
} ethsw_ethtool_counters[] =  {
	{DPSW_CNT_ING_FRAME,		"rx frames"},
	{DPSW_CNT_ING_BYTE,		"rx bytes"},
	{DPSW_CNT_ING_FLTR_FRAME,	"rx filtered frames"},
	{DPSW_CNT_ING_FRAME_DISCARD,	"rx discarded frames"},
	{DPSW_CNT_ING_BCAST_FRAME,	"rx b-cast frames"},
	{DPSW_CNT_ING_BCAST_BYTES,	"rx b-cast bytes"},
	{DPSW_CNT_ING_MCAST_FRAME,	"rx m-cast frames"},
	{DPSW_CNT_ING_MCAST_BYTE,	"rx m-cast bytes"},
	{DPSW_CNT_EGR_FRAME,		"tx frames"},
	{DPSW_CNT_EGR_BYTE,		"tx bytes"},
	{DPSW_CNT_EGR_FRAME_DISCARD,	"tx discarded frames"},

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
		netdev_err(netdev, "ERROR %d getting link state", err);
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
	struct dpsw_link_cfg cfg = {0};
	int err = 0;

	netdev_dbg(netdev, "Setting link parameters...");

	/* Due to a temporary MC limitation, the DPSW port must be down
	 * in order to be able to change link settings. Taking steps to let
	 * the user know that.
	 */
	if (netif_running(netdev)) {
		netdev_info(netdev, "Sorry, interface must be brought down first.\n");
		return -EACCES;
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
	if (err)
		/* ethtool will be loud enough if we return an error; no point
		 * in putting our own error message on the console by default
		 */
		netdev_dbg(netdev, "ERROR %d setting link cfg", err);

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

	memset(data, 0,
	       sizeof(u64) * ETHSW_NUM_COUNTERS);

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

const struct ethtool_ops ethsw_port_ethtool_ops = {
	.get_drvinfo		= ethsw_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= ethsw_get_link_ksettings,
	.set_link_ksettings	= ethsw_set_link_ksettings,
	.get_strings		= ethsw_ethtool_get_strings,
	.get_ethtool_stats	= ethsw_ethtool_get_stats,
	.get_sset_count		= ethsw_ethtool_get_sset_count,
};
