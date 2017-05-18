/* Copyright 2014-2015 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/msi.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_vlan.h>

#include <uapi/linux/if_bridge.h>
#include <net/netlink.h>

#include "../../fsl-mc/include/mc.h"
#include "dpsw.h"
#include "dpsw-cmd.h"

static const char ethsw_drv_version[] = "0.1";

/* Minimal supported DPSE version */
#define DPSW_MIN_VER_MAJOR	8
#define DPSW_MIN_VER_MINOR	0

/* IRQ index */
#define DPSW_MAX_IRQ_NUM		2

#define ETHSW_VLAN_MEMBER	1
#define ETHSW_VLAN_UNTAGGED	2
#define ETHSW_VLAN_PVID		4
#define ETHSW_VLAN_GLOBAL	8

/* Maximum Frame Length supported by HW (currently 10k) */
#define DPAA2_MFL		(10 * 1024)
#define ETHSW_MAX_FRAME_LENGTH	(DPAA2_MFL - VLAN_ETH_HLEN - ETH_FCS_LEN)
#define ETHSW_L2_MAX_FRM(mtu)	((mtu) + VLAN_ETH_HLEN + ETH_FCS_LEN)

struct ethsw_port_priv {
	struct net_device	*netdev;
	struct list_head	list;
	u16			port_index;
	struct ethsw_dev_priv	*ethsw_priv;
	u8			stp_state;

	char			vlans[VLAN_VID_MASK + 1];

};

struct ethsw_dev_priv {
	struct net_device		*netdev;
	struct fsl_mc_io		*mc_io;
	u16			dpsw_handle;
	struct dpsw_attr		sw_attr;
	int				dev_id;
	/*TODO: redundant, we can use the slave dev list */
	struct list_head		port_list;

	bool				flood;
	bool				learning;

	char				vlans[VLAN_VID_MASK + 1];
};

static int ethsw_port_stop(struct net_device *netdev);
static int ethsw_port_open(struct net_device *netdev);

static inline void __get_priv(struct net_device *netdev,
			      struct ethsw_dev_priv **priv,
			      struct ethsw_port_priv **port_priv)
{
	struct ethsw_dev_priv *_priv = NULL;
	struct ethsw_port_priv *_port_priv = NULL;

	if (netdev->flags & IFF_MASTER) {
		_priv = netdev_priv(netdev);
	} else {
		_port_priv = netdev_priv(netdev);
		_priv = _port_priv->ethsw_priv;
	}

	if (priv)
		*priv = _priv;
	if (port_priv)
		*port_priv = _port_priv;
}

/* -------------------------------------------------------------------------- */
/* ethsw netdevice ops */

static netdev_tx_t ethsw_dropframe(struct sk_buff *skb, struct net_device *dev)
{
	/* we don't support I/O for now, drop the frame */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static int ethsw_open(struct net_device *netdev)
{
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);
	struct list_head	*pos;
	struct ethsw_port_priv	*port_priv = NULL;
	int			err;

	err = dpsw_enable(priv->mc_io, 0, priv->dpsw_handle);
	if (err) {
		netdev_err(netdev, "dpsw_enable err %d\n", err);
		return err;
	}

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct ethsw_port_priv, list);
		err = dev_open(port_priv->netdev);
		if (err)
			netdev_err(port_priv->netdev, "dev_open err %d\n", err);
	}

	return 0;
}

static int ethsw_stop(struct net_device *netdev)
{
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);
	struct list_head	*pos;
	struct ethsw_port_priv	*port_priv = NULL;
	int			err;

	err = dpsw_disable(priv->mc_io, 0, priv->dpsw_handle);
	if (err) {
		netdev_err(netdev, "dpsw_disable err %d\n", err);
		return err;
	}

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct ethsw_port_priv, list);
		err = dev_close(port_priv->netdev);
		if (err)
			netdev_err(port_priv->netdev,
				   "dev_close err %d\n", err);
	}

	return 0;
}

static int ethsw_add_vlan(struct net_device *netdev, u16 vid)
{
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);
	int			err;

	struct dpsw_vlan_cfg	vcfg = {
		/* TODO: add support for VLAN private FDBs */
		.fdb_id = 0,
	};
	if (priv->vlans[vid]) {
		netdev_err(netdev, "VLAN already configured\n");
		return -EEXIST;
	}

	err = dpsw_vlan_add(priv->mc_io, 0, priv->dpsw_handle, vid, &vcfg);
	if (err) {
		netdev_err(netdev, "dpsw_vlan_add err %d\n", err);
		return err;
	}
	priv->vlans[vid] = ETHSW_VLAN_MEMBER;

	return 0;
}

static int ethsw_port_add_vlan(struct net_device *netdev, u16 vid, u16 flags)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	struct ethsw_dev_priv	*priv = port_priv->ethsw_priv;
	int			err;

	struct dpsw_vlan_if_cfg	vcfg = {
		.num_ifs = 1,
		.if_id[0] = port_priv->port_index,
	};

	if (port_priv->vlans[vid]) {
		netdev_err(netdev, "VLAN already configured\n");
		return -EEXIST;
	}

	if (flags & BRIDGE_VLAN_INFO_PVID && netif_oper_up(netdev)) {
		netdev_err(netdev, "interface must be down to change PVID!\n");
		return -EBUSY;
	}

	err = dpsw_vlan_add_if(priv->mc_io, 0, priv->dpsw_handle, vid, &vcfg);
	if (err) {
		netdev_err(netdev, "dpsw_vlan_add_if err %d\n", err);
		return err;
	}
	port_priv->vlans[vid] = ETHSW_VLAN_MEMBER;

	if (flags & BRIDGE_VLAN_INFO_UNTAGGED) {
		err = dpsw_vlan_add_if_untagged(priv->mc_io, 0,
						priv->dpsw_handle, vid, &vcfg);
		if (err) {
			netdev_err(netdev, "dpsw_vlan_add_if_untagged err %d\n",
				   err);
			return err;
		}
		port_priv->vlans[vid] |= ETHSW_VLAN_UNTAGGED;
	}

	if (flags & BRIDGE_VLAN_INFO_PVID) {
		struct dpsw_tci_cfg tci_cfg = {
			/* TODO: at least add better defaults if these cannot
			 * be configured
			 */
			.pcp = 0,
			.dei = 0,
			.vlan_id = vid,
		};

		err = dpsw_if_set_tci(priv->mc_io, 0, priv->dpsw_handle,
				      port_priv->port_index, &tci_cfg);
		if (err) {
			netdev_err(netdev, "dpsw_if_set_tci err %d\n", err);
			return err;
		}
		port_priv->vlans[vid] |= ETHSW_VLAN_PVID;
	}

	return 0;
}

static const struct nla_policy ifla_br_policy[IFLA_MAX + 1] = {
	[IFLA_BRIDGE_FLAGS]	= { .type = NLA_U16 },
	[IFLA_BRIDGE_MODE]	= { .type = NLA_U16 },
	[IFLA_BRIDGE_VLAN_INFO]	= { .type = NLA_BINARY,
				.len = sizeof(struct bridge_vlan_info), },
};

static int ethsw_setlink_af_spec(struct net_device *netdev,
				 struct nlattr **tb)
{
	struct bridge_vlan_info	*vinfo;
	struct ethsw_dev_priv	*priv = NULL;
	struct ethsw_port_priv	*port_priv = NULL;
	int			err = 0;

	if (!tb[IFLA_BRIDGE_VLAN_INFO]) {
		netdev_err(netdev, "no VLAN INFO in nlmsg\n");
		return -EOPNOTSUPP;
	}

	vinfo = nla_data(tb[IFLA_BRIDGE_VLAN_INFO]);

	if (!vinfo->vid || vinfo->vid > VLAN_VID_MASK)
		return -EINVAL;

	__get_priv(netdev, &priv, &port_priv);

	if (!port_priv || !priv->vlans[vinfo->vid]) {
		/* command targets switch device or this is a new VLAN */
		err = ethsw_add_vlan(priv->netdev, vinfo->vid);
		if (err)
			return err;

		/* command targets switch device; mark it*/
		if (!port_priv)
			priv->vlans[vinfo->vid] |= ETHSW_VLAN_GLOBAL;
	}

	if (port_priv) {
		/* command targets switch port */
		err = ethsw_port_add_vlan(netdev, vinfo->vid, vinfo->flags);
		if (err)
			return err;
	}

	return 0;
}

static const struct nla_policy ifla_brport_policy[IFLA_BRPORT_MAX + 1] = {
	[IFLA_BRPORT_STATE]	= { .type = NLA_U8 },
	[IFLA_BRPORT_COST]	= { .type = NLA_U32 },
	[IFLA_BRPORT_PRIORITY]	= { .type = NLA_U16 },
	[IFLA_BRPORT_MODE]	= { .type = NLA_U8 },
	[IFLA_BRPORT_GUARD]	= { .type = NLA_U8 },
	[IFLA_BRPORT_PROTECT]	= { .type = NLA_U8 },
	[IFLA_BRPORT_LEARNING]	= { .type = NLA_U8 },
	[IFLA_BRPORT_UNICAST_FLOOD] = { .type = NLA_U8 },
};

static int ethsw_set_learning(struct net_device *netdev, u8 flag)
{
	struct ethsw_dev_priv		*priv = netdev_priv(netdev);
	enum dpsw_fdb_learning_mode	learn_mode;
	int				err;

	if (flag)
		learn_mode = DPSW_FDB_LEARNING_MODE_HW;
	else
		learn_mode = DPSW_FDB_LEARNING_MODE_DIS;

	err = dpsw_fdb_set_learning_mode(priv->mc_io, 0, priv->dpsw_handle,
					 0, learn_mode);
	if (err) {
		netdev_err(netdev, "dpsw_fdb_set_learning_mode err %d\n", err);
		return err;
	}
	priv->learning = !!flag;

	return 0;
}

static int ethsw_port_set_flood(struct net_device *netdev, u8 flag)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	struct ethsw_dev_priv	*priv = port_priv->ethsw_priv;
	int			err;

	err = dpsw_if_set_flooding(priv->mc_io, 0, priv->dpsw_handle,
				   port_priv->port_index, (int)flag);
	if (err) {
		netdev_err(netdev, "dpsw_fdb_set_learning_mode err %d\n", err);
		return err;
	}
	priv->flood = !!flag;

	return 0;
}

static int ethsw_port_set_state(struct net_device *netdev, u8 state)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	struct ethsw_dev_priv	*priv = port_priv->ethsw_priv;
	u8			old_state = port_priv->stp_state;
	int			err;

	struct dpsw_stp_cfg stp_cfg = {
		.vlan_id = 1,
		.state = state,
	};
	/* TODO: check port state, interface may be down */

	if (state > BR_STATE_BLOCKING)
		return -EINVAL;

	if (state == port_priv->stp_state)
		return 0;

	if (state == BR_STATE_DISABLED) {
		port_priv->stp_state = state;

		err = ethsw_port_stop(netdev);
		if (err)
			goto error;
	} else {
		err = dpsw_if_set_stp(priv->mc_io, 0, priv->dpsw_handle,
				      port_priv->port_index, &stp_cfg);
		if (err) {
			netdev_err(netdev, "dpsw_if_set_stp err %d\n", err);
			return err;
		}

		port_priv->stp_state = state;

		if (old_state == BR_STATE_DISABLED) {
			err = ethsw_port_open(netdev);
			if (err)
				goto error;
		}
	}

	return 0;
error:
	port_priv->stp_state = old_state;
	return err;
}

static int ethsw_setlink_protinfo(struct net_device *netdev,
				  struct nlattr **tb)
{
	struct ethsw_dev_priv	*priv;
	struct ethsw_port_priv	*port_priv = NULL;
	int			err = 0;

	__get_priv(netdev, &priv, &port_priv);

	if (tb[IFLA_BRPORT_LEARNING]) {
		u8 flag = nla_get_u8(tb[IFLA_BRPORT_LEARNING]);

		if (port_priv)
			netdev_warn(netdev,
				    "learning set on whole switch dev\n");

		err = ethsw_set_learning(priv->netdev, flag);
		if (err)
			return err;

	} else if (tb[IFLA_BRPORT_UNICAST_FLOOD] && port_priv) {
		u8 flag = nla_get_u8(tb[IFLA_BRPORT_UNICAST_FLOOD]);

		err = ethsw_port_set_flood(port_priv->netdev, flag);
		if (err)
			return err;

	} else if (tb[IFLA_BRPORT_STATE] && port_priv) {
		u8 state = nla_get_u8(tb[IFLA_BRPORT_STATE]);

		err = ethsw_port_set_state(port_priv->netdev, state);
		if (err)
			return err;

	} else {
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ethsw_setlink(struct net_device *netdev,
			 struct nlmsghdr *nlh,
			 u16 flags)
{
	struct nlattr	*attr;
	struct nlattr	*tb[(IFLA_BRIDGE_MAX > IFLA_BRPORT_MAX) ?
				IFLA_BRIDGE_MAX : IFLA_BRPORT_MAX + 1];
	int err = 0;

	attr = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);
	if (attr) {
		err = nla_parse_nested(tb, IFLA_BRIDGE_MAX, attr,
				       ifla_br_policy);
		if (err) {
			netdev_err(netdev,
				   "nla_parse_nested for br_policy err %d\n",
				   err);
			return err;
		}

		err = ethsw_setlink_af_spec(netdev, tb);
		return err;
	}

	attr = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_PROTINFO);
	if (attr) {
		err = nla_parse_nested(tb, IFLA_BRPORT_MAX, attr,
				       ifla_brport_policy);
		if (err) {
			netdev_err(netdev,
				   "nla_parse_nested for brport_policy err %d\n",
				   err);
			return err;
		}

		err = ethsw_setlink_protinfo(netdev, tb);
		return err;
	}

	netdev_err(netdev, "nlmsg_find_attr found no AF_SPEC/PROTINFO\n");
	return -EOPNOTSUPP;
}

static int __nla_put_netdev(struct sk_buff *skb, struct net_device *netdev,
			    struct ethsw_dev_priv *priv)
{
	u8 operstate = netif_running(netdev) ? netdev->operstate : IF_OPER_DOWN;
	int iflink;
	int err;

	err = nla_put_string(skb, IFLA_IFNAME, netdev->name);
	if (err)
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_MASTER, priv->netdev->ifindex);
	if (err)
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_MTU, netdev->mtu);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_OPERSTATE, operstate);
	if (err)
		goto nla_put_err;
	if (netdev->addr_len) {
		err = nla_put(skb, IFLA_ADDRESS, netdev->addr_len,
			      netdev->dev_addr);
		if (err)
			goto nla_put_err;
	}

	iflink = dev_get_iflink(netdev);
	if (netdev->ifindex != iflink) {
		err = nla_put_u32(skb, IFLA_LINK, iflink);
		if (err)
			goto nla_put_err;
	}

	return 0;

nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	return err;
}

static int __nla_put_port(struct sk_buff *skb, struct net_device *netdev,
			  struct ethsw_port_priv *port_priv)
{
	struct nlattr *nest;
	int err;

	u8 stp_state = port_priv->stp_state;

	if (port_priv->stp_state == DPSW_STP_STATE_BLOCKING)
		stp_state = BR_STATE_BLOCKING;

	nest = nla_nest_start(skb, IFLA_PROTINFO | NLA_F_NESTED);
	if (!nest) {
		netdev_err(netdev, "nla_nest_start failed\n");
		return -ENOMEM;
	}

	err = nla_put_u8(skb, IFLA_BRPORT_STATE, stp_state);
	if (err)
		goto nla_put_err;
	err = nla_put_u16(skb, IFLA_BRPORT_PRIORITY, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u32(skb, IFLA_BRPORT_COST, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_MODE, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_GUARD, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_PROTECT, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_FAST_LEAVE, 0);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_LEARNING,
			 port_priv->ethsw_priv->learning);
	if (err)
		goto nla_put_err;
	err = nla_put_u8(skb, IFLA_BRPORT_UNICAST_FLOOD,
			 port_priv->ethsw_priv->flood);
	if (err)
		goto nla_put_err;
	nla_nest_end(skb, nest);

	return 0;

nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	nla_nest_cancel(skb, nest);
	return err;
}

static int __nla_put_vlan(struct sk_buff *skb,  struct net_device *netdev,
			  struct ethsw_dev_priv *priv,
			  struct ethsw_port_priv *port_priv)
{
	struct nlattr *nest;
	struct bridge_vlan_info vinfo;
	const char *vlans;
	u16 i;
	int err;

	nest = nla_nest_start(skb, IFLA_AF_SPEC);
	if (!nest) {
		netdev_err(netdev, "nla_nest_start failed");
		return -ENOMEM;
	}

	if (port_priv)
		vlans = port_priv->vlans;
	else
		vlans = priv->vlans;

	for (i = 0; i < VLAN_VID_MASK + 1; i++) {
		vinfo.flags = 0;
		vinfo.vid = i;

		if (vlans[i] & ETHSW_VLAN_UNTAGGED)
			vinfo.flags |= BRIDGE_VLAN_INFO_UNTAGGED;

		if (vlans[i] & ETHSW_VLAN_PVID)
			vinfo.flags |= BRIDGE_VLAN_INFO_PVID;

		if (vlans[i] & ETHSW_VLAN_MEMBER) {
			err = nla_put(skb, IFLA_BRIDGE_VLAN_INFO,
				      sizeof(vinfo), &vinfo);
			if (err)
				goto nla_put_err;
		}
	}

	nla_nest_end(skb, nest);

	return 0;
nla_put_err:
	netdev_err(netdev, "nla_put_ err %d\n", err);
	nla_nest_cancel(skb, nest);
	return err;
}

static int ethsw_getlink(struct sk_buff *skb, u32 pid, u32 seq,
			 struct net_device *netdev, u32 filter_mask,
			 int nlflags)
{
	struct ethsw_dev_priv	*priv;
	struct ethsw_port_priv	*port_priv = NULL;
	struct ifinfomsg *hdr;
	struct nlmsghdr *nlh;
	int err;

	__get_priv(netdev, &priv, &port_priv);

	nlh = nlmsg_put(skb, pid, seq, RTM_NEWLINK, sizeof(*hdr), NLM_F_MULTI);
	if (!nlh)
		return -EMSGSIZE;

	hdr = nlmsg_data(nlh);
	memset(hdr, 0, sizeof(*hdr));
	hdr->ifi_family = AF_BRIDGE;
	hdr->ifi_type = netdev->type;
	hdr->ifi_index = netdev->ifindex;
	hdr->ifi_flags = dev_get_flags(netdev);

	err = __nla_put_netdev(skb, netdev, priv);
	if (err)
		goto nla_put_err;

	if (port_priv) {
		err = __nla_put_port(skb, netdev, port_priv);
		if (err)
			goto nla_put_err;
	}

	/* Check if  the VID information is requested */
	if (filter_mask & RTEXT_FILTER_BRVLAN) {
		err = __nla_put_vlan(skb, netdev, priv, port_priv);
		if (err)
			goto nla_put_err;
	}

	nlmsg_end(skb, nlh);
	return skb->len;

nla_put_err:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

static int ethsw_dellink_switch(struct ethsw_dev_priv *priv, u16 vid)
{
	struct list_head	*pos;
	struct ethsw_port_priv	*ppriv_local = NULL;
	int			err = 0;

	if (!priv->vlans[vid])
		return -ENOENT;

	err = dpsw_vlan_remove(priv->mc_io, 0, priv->dpsw_handle, vid);
	if (err) {
		netdev_err(priv->netdev, "dpsw_vlan_remove err %d\n", err);
		return err;
	}
	priv->vlans[vid] = 0;

	list_for_each(pos, &priv->port_list) {
		ppriv_local = list_entry(pos, struct ethsw_port_priv,
					 list);
		ppriv_local->vlans[vid] = 0;
	}

	return 0;
}

static int ethsw_dellink_port(struct ethsw_dev_priv *priv,
			      struct ethsw_port_priv *port_priv,
			      u16 vid)
{
	struct list_head	*pos;
	struct ethsw_port_priv	*ppriv_local = NULL;
	struct dpsw_vlan_if_cfg	vcfg = {
		.num_ifs = 1,
		.if_id[0] = port_priv->port_index,
	};
	unsigned int		count = 0;
	int			err = 0;

	if (!port_priv->vlans[vid])
		return -ENOENT;

	/* VLAN will be deleted from switch if global flag is not set
	 * and is configured on only one port
	 */
	if (!(priv->vlans[vid] & ETHSW_VLAN_GLOBAL)) {
		list_for_each(pos, &priv->port_list) {
			ppriv_local = list_entry(pos, struct ethsw_port_priv,
						 list);
			if (ppriv_local->vlans[vid] & ETHSW_VLAN_MEMBER)
				count++;
		}

		if (count == 1)
			return ethsw_dellink_switch(priv, vid);
	}

	err = dpsw_vlan_remove_if(priv->mc_io, 0, priv->dpsw_handle,
				  vid, &vcfg);
	if (err) {
		netdev_err(priv->netdev, "dpsw_vlan_remove_if err %d\n", err);
		return err;
	}
	port_priv->vlans[vid] = 0;
	return 0;
}

static int ethsw_dellink(struct net_device *netdev,
			 struct nlmsghdr *nlh,
			 u16 flags)
{
	struct nlattr		*tb[IFLA_BRIDGE_MAX + 1];
	struct nlattr		*spec;
	struct bridge_vlan_info	*vinfo;
	struct ethsw_dev_priv	*priv;
	struct ethsw_port_priv	*port_priv = NULL;
	int			err = 0;

	spec = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);
	if (!spec)
		return 0;

	err = nla_parse_nested(tb, IFLA_BRIDGE_MAX, spec, ifla_br_policy);
	if (err)
		return err;

	if (!tb[IFLA_BRIDGE_VLAN_INFO])
		return -EOPNOTSUPP;

	vinfo = nla_data(tb[IFLA_BRIDGE_VLAN_INFO]);

	if (!vinfo->vid || vinfo->vid > VLAN_VID_MASK)
		return -EINVAL;

	__get_priv(netdev, &priv, &port_priv);

	/* decide if command targets switch device or port */
	if (!port_priv)
		err = ethsw_dellink_switch(priv, vinfo->vid);
	else
		err = ethsw_dellink_port(priv, port_priv, vinfo->vid);

	return err;
}

static const struct net_device_ops ethsw_ops = {
	.ndo_open		= &ethsw_open,
	.ndo_stop		= &ethsw_stop,

	.ndo_bridge_setlink	= &ethsw_setlink,
	.ndo_bridge_getlink	= &ethsw_getlink,
	.ndo_bridge_dellink	= &ethsw_dellink,

	.ndo_start_xmit		= &ethsw_dropframe,
};

/*--------------------------------------------------------------------------- */
/* switch port netdevice ops */

static int _ethsw_port_carrier_state_sync(struct net_device *netdev)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	struct dpsw_link_state	state;
	int			err;

	err = dpsw_if_get_link_state(port_priv->ethsw_priv->mc_io, 0,
				     port_priv->ethsw_priv->dpsw_handle,
				     port_priv->port_index, &state);
	if (unlikely(err)) {
		netdev_err(netdev, "dpsw_if_get_link_state() err %d\n", err);
		return err;
	}

	WARN_ONCE(state.up > 1, "Garbage read into link_state");

	if (state.up)
		netif_carrier_on(port_priv->netdev);
	else
		netif_carrier_off(port_priv->netdev);

	return 0;
}

static int ethsw_port_open(struct net_device *netdev)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	int			err;

	if (!netif_oper_up(netdev) ||
	    port_priv->stp_state == BR_STATE_DISABLED)
		return 0;

	err = dpsw_if_enable(port_priv->ethsw_priv->mc_io, 0,
			     port_priv->ethsw_priv->dpsw_handle,
			     port_priv->port_index);
	if (err) {
		netdev_err(netdev, "dpsw_if_enable err %d\n", err);
		return err;
	}

	return 0;
}

static int ethsw_port_stop(struct net_device *netdev)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	int			err;

	err = dpsw_if_disable(port_priv->ethsw_priv->mc_io, 0,
			      port_priv->ethsw_priv->dpsw_handle,
			      port_priv->port_index);
	if (err) {
		netdev_err(netdev, "dpsw_if_disable err %d\n", err);
		return err;
	}

	return 0;
}

static int ethsw_port_fdb_add_uc(struct net_device *netdev,
				 const unsigned char *addr)
{
	struct ethsw_port_priv		*port_priv = netdev_priv(netdev);
	struct dpsw_fdb_unicast_cfg	entry = {0};
	int err;

	entry.if_egress = port_priv->port_index;
	entry.type = DPSW_FDB_ENTRY_STATIC;
	ether_addr_copy(entry.mac_addr, addr);

	err = dpsw_fdb_add_unicast(port_priv->ethsw_priv->mc_io, 0,
				   port_priv->ethsw_priv->dpsw_handle,
				   0, &entry);
	if (err)
		netdev_err(netdev, "dpsw_fdb_add_unicast err %d\n", err);
	return err;
}

static int ethsw_port_fdb_del_uc(struct net_device *netdev,
				 const unsigned char *addr)
{
	struct ethsw_port_priv		*port_priv = netdev_priv(netdev);
	struct dpsw_fdb_unicast_cfg	entry = {0};
	int err;

	entry.if_egress = port_priv->port_index;
	entry.type = DPSW_FDB_ENTRY_STATIC;
	ether_addr_copy(entry.mac_addr, addr);

	err = dpsw_fdb_remove_unicast(port_priv->ethsw_priv->mc_io, 0,
				      port_priv->ethsw_priv->dpsw_handle,
				      0, &entry);
	if (err)
		netdev_err(netdev, "dpsw_fdb_remove_unicast err %d\n", err);
	return err;
}

static int ethsw_port_fdb_add_mc(struct net_device *netdev,
				 const unsigned char *addr)
{
	struct ethsw_port_priv		*port_priv = netdev_priv(netdev);
	struct dpsw_fdb_multicast_cfg	entry = {0};
	int err;

	ether_addr_copy(entry.mac_addr, addr);
	entry.type = DPSW_FDB_ENTRY_STATIC;
	entry.num_ifs = 1;
	entry.if_id[0] = port_priv->port_index;

	err = dpsw_fdb_add_multicast(port_priv->ethsw_priv->mc_io, 0,
				     port_priv->ethsw_priv->dpsw_handle,
				     0, &entry);
	if (err)
		netdev_err(netdev, "dpsw_fdb_add_multicast err %d\n", err);
	return err;
}

static int ethsw_port_fdb_del_mc(struct net_device *netdev,
				 const unsigned char *addr)
{
	struct ethsw_port_priv		*port_priv = netdev_priv(netdev);
	struct dpsw_fdb_multicast_cfg	entry = {0};
	int err;

	ether_addr_copy(entry.mac_addr, addr);
	entry.type = DPSW_FDB_ENTRY_STATIC;
	entry.num_ifs = 1;
	entry.if_id[0] = port_priv->port_index;

	err = dpsw_fdb_remove_multicast(port_priv->ethsw_priv->mc_io, 0,
					port_priv->ethsw_priv->dpsw_handle,
					0, &entry);
	if (err)
		netdev_err(netdev, "dpsw_fdb_remove_multicast err %d\n", err);
	return err;
}

static int _lookup_address(struct net_device *netdev, int is_uc,
			   const unsigned char *addr)
{
	struct netdev_hw_addr *ha;
	struct netdev_hw_addr_list *list = (is_uc) ? &netdev->uc : &netdev->mc;

	netif_addr_lock_bh(netdev);
	list_for_each_entry(ha, &list->list, list) {
		if (ether_addr_equal(ha->addr, addr)) {
			netif_addr_unlock_bh(netdev);
			return 1;
		}
	}
	netif_addr_unlock_bh(netdev);
	return 0;
}

static int ethsw_port_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
			      struct net_device *netdev,
			      const unsigned char *addr, u16 vid,
			      u16 flags)
{
	struct list_head	*pos;
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	struct ethsw_dev_priv	*priv = port_priv->ethsw_priv;
	int err;

	/* TODO: add replace support when added to iproute bridge */
	if (!(flags & NLM_F_REQUEST)) {
		netdev_err(netdev,
			   "ethsw_port_fdb_add unexpected flags value %08x\n",
			   flags);
		return -EINVAL;
	}

	if (is_unicast_ether_addr(addr)) {
		/* if entry cannot be replaced, return error if exists */
		if (flags & NLM_F_EXCL || flags & NLM_F_APPEND) {
			list_for_each(pos, &priv->port_list) {
				port_priv = list_entry(pos,
						       struct ethsw_port_priv,
						       list);
				if (_lookup_address(port_priv->netdev,
						    1, addr))
					return -EEXIST;
			}
		}

		err = ethsw_port_fdb_add_uc(netdev, addr);
		if (err) {
			netdev_err(netdev, "ethsw_port_fdb_add_uc err %d\n",
				   err);
			return err;
		}

		/* we might have replaced an existing entry for a different
		 * switch port, make sure the address doesn't linger in any
		 * port address list
		 */
		list_for_each(pos, &priv->port_list) {
			port_priv = list_entry(pos, struct ethsw_port_priv,
					       list);
			dev_uc_del(port_priv->netdev, addr);
		}

		err = dev_uc_add(netdev, addr);
		if (err) {
			netdev_err(netdev, "dev_uc_add err %d\n", err);
			return err;
		}
	} else {
		struct dpsw_fdb_multicast_cfg entry = {
			.type = DPSW_FDB_ENTRY_STATIC,
			.num_ifs = 0,
		};

		/* check if address is already set on this port */
		if (_lookup_address(netdev, 0, addr))
			return -EEXIST;

		/* check if the address exists on other port */
		ether_addr_copy(entry.mac_addr, addr);
		err = dpsw_fdb_get_multicast(priv->mc_io, 0, priv->dpsw_handle,
					     0, &entry);
		if (!err) {
			/* entry exists, can we replace it? */
			if (flags & NLM_F_EXCL)
				return -EEXIST;
		} else if (err != -ENAVAIL) {
			netdev_err(netdev, "dpsw_fdb_get_unicast err %d\n",
				   err);
			return err;
		}

		err = ethsw_port_fdb_add_mc(netdev, addr);
		if (err) {
			netdev_err(netdev, "ethsw_port_fdb_add_mc err %d\n",
				   err);
			return err;
		}

		err = dev_mc_add(netdev, addr);
		if (err) {
			netdev_err(netdev, "dev_mc_add err %d\n", err);
			return err;
		}
	}

	return 0;
}

static int ethsw_port_fdb_del(struct ndmsg *ndm, struct nlattr *tb[],
			      struct net_device *netdev,
			      const unsigned char *addr, u16 vid)
{
	int err;

	if (is_unicast_ether_addr(addr)) {
		err = ethsw_port_fdb_del_uc(netdev, addr);
		if (err) {
			netdev_err(netdev, "ethsw_port_fdb_del_uc err %d\n",
				   err);
			return err;
		}

		/* also delete if configured on port */
		err = dev_uc_del(netdev, addr);
		if (err && err != -ENOENT) {
			netdev_err(netdev, "dev_uc_del err %d\n", err);
			return err;
		}
	} else {
		if (!_lookup_address(netdev, 0, addr))
			return -ENOENT;

		err = dev_mc_del(netdev, addr);
		if (err) {
			netdev_err(netdev, "dev_mc_del err %d\n", err);
			return err;
		}

		err = ethsw_port_fdb_del_mc(netdev, addr);
		if (err) {
			netdev_err(netdev, "ethsw_port_fdb_del_mc err %d\n",
				   err);
			return err;
		}
	}

	return 0;
}

void ethsw_port_get_stats(struct net_device *netdev,
			  struct rtnl_link_stats64 *storage)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	u64			tmp;
	int			err;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_ING_FRAME, &storage->rx_packets);
	if (err)
		goto error;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_EGR_FRAME, &storage->tx_packets);
	if (err)
		goto error;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_ING_BYTE, &storage->rx_bytes);
	if (err)
		goto error;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_EGR_BYTE, &storage->tx_bytes);
	if (err)
		goto error;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_ING_FRAME_DISCARD,
				  &storage->rx_dropped);
	if (err)
		goto error;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_ING_FLTR_FRAME,
				  &tmp);
	if (err)
		goto error;
	storage->rx_dropped += tmp;

	err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
				  port_priv->ethsw_priv->dpsw_handle,
				  port_priv->port_index,
				  DPSW_CNT_EGR_FRAME_DISCARD,
				  &storage->tx_dropped);
	if (err)
		goto error;

	return;

error:
	netdev_err(netdev, "dpsw_if_get_counter err %d\n", err);
}

static int ethsw_port_change_mtu(struct net_device *netdev, int mtu)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	int			err;

	if (mtu < ETH_ZLEN || mtu > ETHSW_MAX_FRAME_LENGTH) {
		netdev_err(netdev, "Invalid MTU %d. Valid range is: %d..%d\n",
			   mtu, ETH_ZLEN, ETHSW_MAX_FRAME_LENGTH);
		return -EINVAL;
	}

	err = dpsw_if_set_max_frame_length(port_priv->ethsw_priv->mc_io,
					   0,
					   port_priv->ethsw_priv->dpsw_handle,
					   port_priv->port_index,
					   (u16)ETHSW_L2_MAX_FRM(mtu));
	if (err) {
		netdev_err(netdev,
			   "dpsw_if_set_max_frame_length() err %d\n", err);
		return err;
	}

	netdev->mtu = mtu;
	return 0;
}

static const struct net_device_ops ethsw_port_ops = {
	.ndo_open		= &ethsw_port_open,
	.ndo_stop		= &ethsw_port_stop,

	.ndo_fdb_add		= &ethsw_port_fdb_add,
	.ndo_fdb_del		= &ethsw_port_fdb_del,
	.ndo_fdb_dump		= &ndo_dflt_fdb_dump,

	.ndo_get_stats64	= &ethsw_port_get_stats,
	.ndo_change_mtu		= &ethsw_port_change_mtu,

	.ndo_start_xmit		= &ethsw_dropframe,
};

static void ethsw_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	u16 version_major, version_minor;
	int err;

	strlcpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, ethsw_drv_version, sizeof(drvinfo->version));

	err = dpsw_get_api_version(port_priv->ethsw_priv->mc_io, 0,
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

static int ethsw_get_settings(struct net_device *netdev,
			      struct ethtool_cmd *cmd)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct dpsw_link_state state = {0};
	int err = 0;

	err = dpsw_if_get_link_state(port_priv->ethsw_priv->mc_io, 0,
				     port_priv->ethsw_priv->dpsw_handle,
				     port_priv->port_index,
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
		cmd->autoneg = AUTONEG_ENABLE;
	if (!(state.options & DPSW_LINK_OPT_HALF_DUPLEX))
		cmd->autoneg = DUPLEX_FULL;
	ethtool_cmd_speed_set(cmd, state.rate);

out:
	return err;
}

static int ethsw_set_settings(struct net_device *netdev,
			      struct ethtool_cmd *cmd)
{
	struct ethsw_port_priv *port_priv = netdev_priv(netdev);
	struct dpsw_link_state state = {0};
	struct dpsw_link_cfg cfg = {0};
	int err = 0;

	netdev_dbg(netdev, "Setting link parameters...");

	err = dpsw_if_get_link_state(port_priv->ethsw_priv->mc_io, 0,
				     port_priv->ethsw_priv->dpsw_handle,
				     port_priv->port_index,
				     &state);
	if (err) {
		netdev_err(netdev, "ERROR %d getting link state", err);
		goto out;
	}

	/* Due to a temporary MC limitation, the DPSW port must be down
	 * in order to be able to change link settings. Taking steps to let
	 * the user know that.
	 */
	if (netif_running(netdev)) {
		netdev_info(netdev,
			    "Sorry, interface must be brought down first.\n");
		return -EACCES;
	}

	cfg.options = state.options;
	cfg.rate = ethtool_cmd_speed(cmd);
	if (cmd->autoneg == AUTONEG_ENABLE)
		cfg.options |= DPSW_LINK_OPT_AUTONEG;
	else
		cfg.options &= ~DPSW_LINK_OPT_AUTONEG;
	if (cmd->duplex == DUPLEX_HALF)
		cfg.options |= DPSW_LINK_OPT_HALF_DUPLEX;
	else
		cfg.options &= ~DPSW_LINK_OPT_HALF_DUPLEX;

	err = dpsw_if_set_link_cfg(port_priv->ethsw_priv->mc_io, 0,
				   port_priv->ethsw_priv->dpsw_handle,
				   port_priv->port_index,
				   &cfg);
	if (err)
		/* ethtool will be loud enough if we return an error; no point
		 * in putting our own error message on the console by default
		 */
		netdev_dbg(netdev, "ERROR %d setting link cfg", err);

out:
	return err;
}

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

static int ethsw_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(ethsw_ethtool_counters);
	default:
		return -EOPNOTSUPP;
	}
}

static void ethsw_ethtool_get_strings(struct net_device *netdev,
				      u32 stringset, u8 *data)
{
	u32 i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(ethsw_ethtool_counters); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       ethsw_ethtool_counters[i].name, ETH_GSTRING_LEN);
		break;
	}
}

static void ethsw_ethtool_get_stats(struct net_device *netdev,
				    struct ethtool_stats *stats,
				    u64 *data)
{
	struct ethsw_port_priv	*port_priv = netdev_priv(netdev);
	u32			i;
	int			err;

	for (i = 0; i < ARRAY_SIZE(ethsw_ethtool_counters); i++) {
		err = dpsw_if_get_counter(port_priv->ethsw_priv->mc_io, 0,
					  port_priv->ethsw_priv->dpsw_handle,
					  port_priv->port_index,
					  ethsw_ethtool_counters[i].id,
					  &data[i]);
		if (err)
			netdev_err(netdev, "dpsw_if_get_counter[%s] err %d\n",
				   ethsw_ethtool_counters[i].name, err);
	}
}

static const struct ethtool_ops ethsw_port_ethtool_ops = {
	.get_drvinfo		= &ethsw_get_drvinfo,
	.get_link		= &ethtool_op_get_link,
	.get_settings		= &ethsw_get_settings,
	.set_settings		= &ethsw_set_settings,
	.get_strings		= &ethsw_ethtool_get_strings,
	.get_ethtool_stats	= &ethsw_ethtool_get_stats,
	.get_sset_count		= &ethsw_ethtool_get_sset_count,
};

/* -------------------------------------------------------------------------- */
/* ethsw driver functions */

static int ethsw_links_state_update(struct ethsw_dev_priv *priv)
{
	struct list_head	*pos;
	struct ethsw_port_priv	*port_priv;
	int err;

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct ethsw_port_priv,
				       list);

		err = _ethsw_port_carrier_state_sync(port_priv->netdev);
		if (err)
			netdev_err(port_priv->netdev,
				   "_ethsw_port_carrier_state_sync err %d\n",
				   err);
	}

	return 0;
}

static irqreturn_t ethsw_irq0_handler(int irq_num, void *arg)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t _ethsw_irq0_handler_thread(int irq_num, void *arg)
{
	struct device		*dev = (struct device *)arg;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);

	struct fsl_mc_io *io = priv->mc_io;
	u16 token = priv->dpsw_handle;
	int irq_index = DPSW_IRQ_INDEX_IF;

	/* Mask the events and the if_id reserved bits to be cleared on read */
	u32 status = DPSW_IRQ_EVENT_LINK_CHANGED | 0xFFFF0000;
	int err;

	err = dpsw_get_irq_status(io, 0, token, irq_index, &status);
	if (unlikely(err)) {
		netdev_err(netdev, "Can't get irq status (err %d)", err);

		err = dpsw_clear_irq_status(io, 0, token, irq_index,
					    0xFFFFFFFF);
		if (unlikely(err))
			netdev_err(netdev, "Can't clear irq status (err %d)",
				   err);
		goto out;
	}

	if (status & DPSW_IRQ_EVENT_LINK_CHANGED) {
		err = ethsw_links_state_update(priv);
		if (unlikely(err))
			goto out;
	}

out:
	return IRQ_HANDLED;
}

static int ethsw_setup_irqs(struct fsl_mc_device *sw_dev)
{
	struct device		*dev = &sw_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);
	int err = 0;
	struct fsl_mc_device_irq *irq;
	const int irq_index = DPSW_IRQ_INDEX_IF;
	u32 mask = DPSW_IRQ_EVENT_LINK_CHANGED;

	err = fsl_mc_allocate_irqs(sw_dev);
	if (unlikely(err)) {
		dev_err(dev, "MC irqs allocation failed\n");
		return err;
	}

	if (WARN_ON(sw_dev->obj_desc.irq_count != DPSW_MAX_IRQ_NUM)) {
		err = -EINVAL;
		goto free_irq;
	}

	err = dpsw_set_irq_enable(priv->mc_io, 0, priv->dpsw_handle,
				  irq_index, 0);
	if (unlikely(err)) {
		dev_err(dev, "dpsw_set_irq_enable err %d\n", err);
		goto free_irq;
	}

	irq = sw_dev->irqs[irq_index];

	err = devm_request_threaded_irq(dev, irq->msi_desc->irq,
					ethsw_irq0_handler,
					_ethsw_irq0_handler_thread,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(dev), dev);
	if (unlikely(err)) {
		dev_err(dev, "devm_request_threaded_irq(): %d", err);
		goto free_irq;
	}

	err = dpsw_set_irq_mask(priv->mc_io, 0, priv->dpsw_handle,
				irq_index, mask);
	if (unlikely(err)) {
		dev_err(dev, "dpsw_set_irq_mask(): %d", err);
		goto free_devm_irq;
	}

	err = dpsw_set_irq_enable(priv->mc_io, 0, priv->dpsw_handle,
				  irq_index, 1);
	if (unlikely(err)) {
		dev_err(dev, "dpsw_set_irq_enable(): %d", err);
		goto free_devm_irq;
	}

	return 0;

free_devm_irq:
	devm_free_irq(dev, irq->msi_desc->irq, dev);
free_irq:
	fsl_mc_free_irqs(sw_dev);
	return err;
}

static void ethsw_teardown_irqs(struct fsl_mc_device *sw_dev)
{
	struct device		*dev = &sw_dev->dev;
	struct net_device	*netdev = dev_get_drvdata(dev);
	struct ethsw_dev_priv	*priv = netdev_priv(netdev);

	dpsw_set_irq_enable(priv->mc_io, 0, priv->dpsw_handle,
			    DPSW_IRQ_INDEX_IF, 0);
	devm_free_irq(dev,
		      sw_dev->irqs[DPSW_IRQ_INDEX_IF]->msi_desc->irq,
		      dev);
	fsl_mc_free_irqs(sw_dev);
}

static int __cold
ethsw_init(struct fsl_mc_device *sw_dev)
{
	struct device		*dev = &sw_dev->dev;
	struct ethsw_dev_priv	*priv;
	struct net_device	*netdev;
	int			err = 0;
	u16			i;
	u16			version_major, version_minor;
	const struct dpsw_stp_cfg stp_cfg = {
		.vlan_id = 1,
		.state = DPSW_STP_STATE_FORWARDING,
	};

	netdev = dev_get_drvdata(dev);
	priv = netdev_priv(netdev);

	priv->dev_id = sw_dev->obj_desc.id;

	err = dpsw_open(priv->mc_io, 0, priv->dev_id, &priv->dpsw_handle);
	if (err) {
		dev_err(dev, "dpsw_open err %d\n", err);
		goto err_exit;
	}
	if (!priv->dpsw_handle) {
		dev_err(dev, "dpsw_open returned null handle but no error\n");
		err = -EFAULT;
		goto err_exit;
	}

	err = dpsw_get_attributes(priv->mc_io, 0, priv->dpsw_handle,
				  &priv->sw_attr);
	if (err) {
		dev_err(dev, "dpsw_get_attributes err %d\n", err);
		goto err_close;
	}

	err = dpsw_get_api_version(priv->mc_io, 0,
				   &version_major,
				   &version_minor);
	if (err) {
		dev_err(dev, "dpsw_get_api_version err %d\n", err);
		goto err_close;
	}

	/* Minimum supported DPSW version check */
	if (version_major < DPSW_MIN_VER_MAJOR ||
	    (version_major == DPSW_MIN_VER_MAJOR &&
	     version_minor < DPSW_MIN_VER_MINOR)) {
		dev_err(dev, "DPSW version %d:%d not supported. Use %d.%d or greater.\n",
			version_major,
			version_minor,
			DPSW_MIN_VER_MAJOR, DPSW_MIN_VER_MINOR);
		err = -ENOTSUPP;
		goto err_close;
	}

	err = dpsw_reset(priv->mc_io, 0, priv->dpsw_handle);
	if (err) {
		dev_err(dev, "dpsw_reset err %d\n", err);
		goto err_close;
	}

	err = dpsw_fdb_set_learning_mode(priv->mc_io, 0, priv->dpsw_handle, 0,
					 DPSW_FDB_LEARNING_MODE_HW);
	if (err) {
		dev_err(dev, "dpsw_fdb_set_learning_mode err %d\n", err);
		goto err_close;
	}

	for (i = 0; i < priv->sw_attr.num_ifs; i++) {
		err = dpsw_if_set_stp(priv->mc_io, 0, priv->dpsw_handle, i,
				      &stp_cfg);
		if (err) {
			dev_err(dev, "dpsw_if_set_stp err %d for port %d\n",
				err, i);
			goto err_close;
		}

		err = dpsw_if_set_broadcast(priv->mc_io, 0,
					    priv->dpsw_handle, i, 1);
		if (err) {
			dev_err(dev,
				"dpsw_if_set_broadcast err %d for port %d\n",
				err, i);
			goto err_close;
		}
	}

	return 0;

err_close:
	dpsw_close(priv->mc_io, 0, priv->dpsw_handle);
err_exit:
	return err;
}

static int __cold
ethsw_takedown(struct fsl_mc_device *sw_dev)
{
	struct device		*dev = &sw_dev->dev;
	struct net_device	*netdev;
	struct ethsw_dev_priv	*priv;
	int			err;

	netdev = dev_get_drvdata(dev);
	priv = netdev_priv(netdev);

	err = dpsw_close(priv->mc_io, 0, priv->dpsw_handle);
	if (err)
		dev_warn(dev, "dpsw_close err %d\n", err);

	return 0;
}

static int __cold
ethsw_remove(struct fsl_mc_device *sw_dev)
{
	struct device		*dev;
	struct net_device	*netdev;
	struct ethsw_dev_priv	*priv;
	struct ethsw_port_priv	*port_priv;
	struct list_head	*pos;

	dev = &sw_dev->dev;
	netdev = dev_get_drvdata(dev);
	priv = netdev_priv(netdev);

	list_for_each(pos, &priv->port_list) {
		port_priv = list_entry(pos, struct ethsw_port_priv, list);

		rtnl_lock();
		netdev_upper_dev_unlink(port_priv->netdev, netdev);
		rtnl_unlock();

		unregister_netdev(port_priv->netdev);
		free_netdev(port_priv->netdev);
	}

	ethsw_teardown_irqs(sw_dev);

	unregister_netdev(netdev);

	ethsw_takedown(sw_dev);
	fsl_mc_portal_free(priv->mc_io);

	dev_set_drvdata(dev, NULL);
	free_netdev(netdev);

	return 0;
}

static int __cold
ethsw_probe(struct fsl_mc_device *sw_dev)
{
	struct device		*dev;
	struct net_device	*netdev = NULL;
	struct ethsw_dev_priv	*priv = NULL;
	int			err = 0;
	u16			i;
	const char		def_mcast[ETH_ALEN] = {
		0x01, 0x00, 0x5e, 0x00, 0x00, 0x01,
	};
	char			port_name[IFNAMSIZ];

	dev = &sw_dev->dev;

	/* register switch device, it's for management only - no I/O */
	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev) {
		dev_err(dev, "alloc_etherdev error\n");
		return -ENOMEM;
	}
	netdev->netdev_ops = &ethsw_ops;

	SET_NETDEV_DEV(netdev, dev);
	dev_set_drvdata(dev, netdev);

	priv = netdev_priv(netdev);
	priv->netdev = netdev;

	err = fsl_mc_portal_allocate(sw_dev, 0, &priv->mc_io);
	if (err) {
		dev_err(dev, "fsl_mc_portal_allocate err %d\n", err);
		goto err_free_netdev;
	}
	if (!priv->mc_io) {
		dev_err(dev, "fsl_mc_portal_allocate returned null handle but no error\n");
		err = -EFAULT;
		goto err_free_netdev;
	}

	err = ethsw_init(sw_dev);
	if (err) {
		dev_err(dev, "switch init err %d\n", err);
		goto err_free_cmdport;
	}

	netdev->flags = netdev->flags | IFF_PROMISC | IFF_MASTER;

	/* TODO: should we hold rtnl_lock here?  We can't register_netdev under
	 * lock
	 */
	dev_alloc_name(netdev, "sw%d");
	err = register_netdev(netdev);
	if (err < 0) {
		dev_err(dev, "register_netdev error %d\n", err);
		goto err_takedown;
	}
	if (err)
		dev_info(dev, "register_netdev res %d\n", err);

	/* VLAN 1 is implicitly configured on the switch */
	priv->vlans[1] = ETHSW_VLAN_MEMBER;
	/* Flooding, learning are implicitly enabled */
	priv->learning = true;
	priv->flood = true;

	/* register switch ports */
	snprintf(port_name, IFNAMSIZ, "%sp%%d", netdev->name);

	INIT_LIST_HEAD(&priv->port_list);
	for (i = 0; i < priv->sw_attr.num_ifs; i++) {
		struct net_device *port_netdev;
		struct ethsw_port_priv *port_priv;

		port_netdev = alloc_etherdev(sizeof(struct ethsw_port_priv));
		if (!port_netdev) {
			dev_err(dev, "alloc_etherdev error\n");
			goto err_takedown;
		}

		port_priv = netdev_priv(port_netdev);
		port_priv->netdev = port_netdev;
		port_priv->ethsw_priv = priv;

		port_priv->port_index = i;
		port_priv->stp_state = BR_STATE_FORWARDING;
		/* VLAN 1 is configured by default on all switch ports */
		port_priv->vlans[1] = ETHSW_VLAN_MEMBER | ETHSW_VLAN_UNTAGGED |
				      ETHSW_VLAN_PVID;

		SET_NETDEV_DEV(port_netdev, dev);
		port_netdev->netdev_ops = &ethsw_port_ops;
		port_netdev->ethtool_ops = &ethsw_port_ethtool_ops;

		port_netdev->flags = port_netdev->flags |
				IFF_PROMISC | IFF_SLAVE;

		dev_alloc_name(port_netdev, port_name);
		err = register_netdev(port_netdev);
		if (err < 0) {
			dev_err(dev, "register_netdev error %d\n", err);
			free_netdev(port_netdev);
			goto err_takedown;
		}

		rtnl_lock();

		err = netdev_master_upper_dev_link(port_netdev, netdev);
		if (err) {
			dev_err(dev, "netdev_master_upper_dev_link error %d\n",
				err);
			unregister_netdev(port_netdev);
			free_netdev(port_netdev);
			rtnl_unlock();
			goto err_takedown;
		}

		rtmsg_ifinfo(RTM_NEWLINK, port_netdev, IFF_SLAVE, GFP_KERNEL);

		rtnl_unlock();

		list_add(&port_priv->list, &priv->port_list);

		/* TODO: implmenet set_rm_mode instead of this */
		err = ethsw_port_fdb_add_mc(port_netdev, def_mcast);
		if (err)
			dev_warn(&netdev->dev,
				 "ethsw_port_fdb_add_mc err %d\n", err);

		/* sync carrier state */
		err = _ethsw_port_carrier_state_sync(port_netdev);
		if (err)
			netdev_err(netdev,
				   "_ethsw_port_carrier_state_sync err %d\n",
				   err);
	}

	/* the switch starts up enabled */
	rtnl_lock();
	err = dev_open(netdev);
	rtnl_unlock();
	if (err)
		dev_warn(dev, "dev_open err %d\n", err);

	/* setup irqs */
	err = ethsw_setup_irqs(sw_dev);
	if (unlikely(err)) {
		dev_warn(dev, "ethsw_setup_irqs err %d\n", err);
		goto err_takedown;
	}

	dev_info(&netdev->dev,
		 "probed %d port switch\n", priv->sw_attr.num_ifs);
	return 0;

err_takedown:
	ethsw_remove(sw_dev);
err_free_cmdport:
	fsl_mc_portal_free(priv->mc_io);
err_free_netdev:
	dev_set_drvdata(dev, NULL);
	free_netdev(netdev);

	return err;
}

static const struct fsl_mc_device_id ethsw_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpsw",
	},
	{}
};

static struct fsl_mc_driver eth_sw_drv = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= ethsw_probe,
	.remove		= ethsw_remove,
	.match_id_table = ethsw_match_id_table,
};

module_fsl_mc_driver(eth_sw_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DPAA2 Ethernet Switch Driver (prototype)");
