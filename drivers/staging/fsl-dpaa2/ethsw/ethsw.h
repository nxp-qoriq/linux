// SPDX-License-Identifier: GPL-2.0
/*
 * DPAA2 Ethernet Switch declarations
 *
 * Copyright 2014-2016 Freescale Semiconductor Inc.
 * Copyright 2017-2018, 2020, 2021, 2023 NXP
 *
 */

#ifndef __ETHSW_H
#define __ETHSW_H

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_vlan.h>
#include <uapi/linux/if_bridge.h>
#include <net/switchdev.h>
#include <linux/if_bridge.h>
#include <net/pkt_cls.h>

#include "dpsw.h"

/* Number of IRQs supported */
#define DPSW_IRQ_NUM	2

/* Port is member of VLAN */
#define ETHSW_VLAN_MEMBER	1
/* VLAN to be treated as untagged on egress */
#define ETHSW_VLAN_UNTAGGED	2
/* Untagged frames will be assigned to this VLAN */
#define ETHSW_VLAN_PVID		4
/* VLAN configured on the switch */
#define ETHSW_VLAN_GLOBAL	8

/* Maximum Frame Length supported by HW (currently 10k) */
#define DPAA2_MFL		(10 * 1024)
#define ETHSW_MAX_FRAME_LENGTH	(DPAA2_MFL - VLAN_ETH_HLEN - ETH_FCS_LEN)
#define ETHSW_L2_MAX_FRM(mtu)	((mtu) + VLAN_ETH_HLEN + ETH_FCS_LEN)

#define ETHSW_FEATURE_MAC_ADDR	BIT(0)

#define DPAA2_ETHSW_PORT_MAX_ACL_ENTRIES	16
#define DPAA2_ETHSW_PORT_DEFAULT_TRAPS		0

#define DPAA2_ETHSW_PORT_ACL_CMD_BUF_SIZE	256

extern const struct ethtool_ops ethsw_port_ethtool_ops;

struct ethsw_core;

struct dpaa2_switch_acl_entry {
	struct list_head	list;
	u32			prio;
	unsigned long		cookie;

	struct dpsw_acl_entry_cfg cfg;
	struct dpsw_acl_key	key;
};

struct dpaa2_switch_acl_tbl {
	struct list_head	entries;
	struct ethsw_core	*ethsw;
	u64			ports;

	u16			id;
	u8			num_rules;
	bool			in_use;
};

/* Per port private data */
struct ethsw_port_priv {
	struct net_device	*netdev;
	u16			idx;
	struct ethsw_core	*ethsw_data;
	u8			link_state;
	u8			stp_state;
	bool			flood;

	u8			vlans[VLAN_VID_MASK + 1];
	u16			pvid;
	struct net_device	*bridge_dev;

	struct dpaa2_switch_acl_tbl *acl_tbl;
};

/* Switch data */
struct ethsw_core {
	struct device			*dev;
	struct fsl_mc_io		*mc_io;
	u16				dpsw_handle;
	struct dpsw_attr		sw_attr;
	u16				major, minor;
	unsigned long			features;
	int				dev_id;
	struct ethsw_port_priv		**ports;

	u8				vlans[VLAN_VID_MASK + 1];
	bool				learning;

	struct notifier_block		port_nb;
	struct notifier_block		port_switchdev_nb;
	struct workqueue_struct		*workqueue;

	struct dpaa2_switch_acl_tbl	*acls;
};

bool dpaa2_switch_port_dev_check(const struct net_device *netdev);

static inline int dpaa2_switch_get_index(struct ethsw_core *ethsw,
					 struct net_device *netdev)
{
	int i;

	for (i = 0; i < ethsw->sw_attr.num_ifs; i++)
		if (ethsw->ports[i]->netdev == netdev)
			return ethsw->ports[i]->idx;

	return -EINVAL;
}

static inline bool
dpaa2_switch_acl_tbl_is_full(struct dpaa2_switch_acl_tbl *acl_tbl)
{
	if ((acl_tbl->num_rules + DPAA2_ETHSW_PORT_DEFAULT_TRAPS) >=
	    DPAA2_ETHSW_PORT_MAX_ACL_ENTRIES)
		return true;
	return false;
}

/* TC offload */
int dpaa2_switch_cls_flower_replace(struct dpaa2_switch_acl_tbl *acl_tbl,
				    struct tc_cls_flower_offload *cls);

int dpaa2_switch_cls_flower_destroy(struct dpaa2_switch_acl_tbl *acl_tbl,
				    struct tc_cls_flower_offload *cls);
#endif	/* __ETHSW_H */
