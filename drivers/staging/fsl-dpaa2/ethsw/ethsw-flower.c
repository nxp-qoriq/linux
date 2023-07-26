// SPDX-License-Identifier: GPL-2.0
/*
 * DPAA2 Ethernet Switch flower support
 *
 * Copyright 2021-2023 NXP
 *
 */

#include <net/tc_act/tc_gact.h>
#include <net/tc_act/tc_mirred.h>
#include "ethsw.h"

static void dpaa2_switch_flower_dump_key(struct ethsw_core *ethsw,
					 struct dpsw_acl_key *acl_key)
{
	struct dpsw_acl_fields *acl_h, *acl_m;
	struct device *dev = ethsw->dev;

	acl_h = &acl_key->match;
	acl_m = &acl_key->mask;

	dev_dbg(dev, "Key contents:\n");

	dev_dbg(dev, "\t L2 DST MAC: match = %pM, mask %pM\n",
		acl_h->l2_dest_mac, acl_m->l2_dest_mac);
	dev_dbg(dev, "\t L2 SRC MAC: match = %pM, mask %pM\n",
		acl_h->l2_source_mac, acl_m->l2_source_mac);
	dev_dbg(dev, "\t L2 Ether Type: match = 0x%x, mask 0x%x\n",
		acl_h->l2_ether_type, acl_m->l2_ether_type);

	dev_dbg(dev, "\t L2 TPID: match = 0x%x, mask 0x%x\n",
		acl_h->l2_tpid, acl_m->l2_tpid);
	dev_dbg(dev, "\t L2 PCP+DEI: match = 0x%x, mask 0x%x\n",
		acl_h->l2_pcp_dei, acl_m->l2_pcp_dei);
	dev_dbg(dev, "\t L2 VLAN ID: match = 0x%x, mask 0x%x\n",
		acl_h->l2_vlan_id, acl_m->l2_vlan_id);

	dev_dbg(dev, "\t L3 DSCP: match = 0x%x, mask 0x%x\n",
		acl_h->l3_dscp, acl_m->l3_dscp);
	dev_dbg(dev, "\t L3 Protocol: match = 0x%x, mask 0x%x\n",
		acl_h->l3_protocol, acl_m->l3_protocol);
	dev_dbg(dev, "\t L3 SRC IP: match = 0x%x, mask 0x%x\n",
		acl_h->l3_source_ip, acl_m->l3_source_ip);
	dev_dbg(dev, "\t L3 DST IP: match = 0x%x, mask 0x%x\n",
		acl_h->l3_dest_ip, acl_m->l3_dest_ip);

	dev_dbg(dev, "\t L4 SRC Port: match = 0x%x, mask 0x%x\n",
		acl_h->l4_source_port, acl_m->l4_source_port);
	dev_dbg(dev, "\t L4 DST Port: match = 0x%x, mask 0x%x\n",
		acl_h->l4_dest_port, acl_m->l4_dest_port);
}

static void dpaa2_switch_tc_dump_action(struct ethsw_core *ethsw,
					struct dpsw_acl_result *dpsw_act)
{
	struct device *dev = ethsw->dev;

	switch (dpsw_act->action) {
	case DPSW_ACL_ACTION_DROP:
		dev_dbg(dev, "Action: DPSW_ACL_ACTION_DROP\n");
		break;
	case DPSW_ACL_ACTION_REDIRECT:
		dev_dbg(dev, "Action: DPSW_ACL_ACTION_REDIRECT to egress of %s (dpsw.%d.%d)\n",
			netdev_name(ethsw->ports[dpsw_act->if_id]->netdev),
			ethsw->dev_id, dpsw_act->if_id);
		break;
	default:
		break;
	}
}

static int dpaa2_switch_flower_parse_key(struct tc_cls_flower_offload *cls,
					 struct dpsw_acl_key *acl_key)
{
	struct netlink_ext_ack *extack = cls->common.extack;
	struct flow_dissector *dissector = cls->dissector;
	struct dpsw_acl_fields *acl_h, *acl_m;

	if (dissector->used_keys &
	    ~(BIT(FLOW_DISSECTOR_KEY_BASIC) |
	      BIT(FLOW_DISSECTOR_KEY_CONTROL) |
	      BIT(FLOW_DISSECTOR_KEY_ETH_ADDRS) |
	      BIT(FLOW_DISSECTOR_KEY_VLAN) |
	      BIT(FLOW_DISSECTOR_KEY_PORTS) |
	      BIT(FLOW_DISSECTOR_KEY_IP) |
	      BIT(FLOW_DISSECTOR_KEY_IPV6_ADDRS) |
	      BIT(FLOW_DISSECTOR_KEY_IPV4_ADDRS))) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Unsupported keys used");
		return -EOPNOTSUPP;
	}

	acl_h = &acl_key->match;
	acl_m = &acl_key->mask;

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_BASIC)) {
		struct flow_dissector_key_basic *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_BASIC,
						  cls->key);
		struct flow_dissector_key_basic *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_BASIC,
						  cls->mask);

		acl_h->l3_protocol = key->ip_proto;
		acl_h->l2_ether_type = be16_to_cpu(key->n_proto);
		acl_m->l3_protocol = mask->ip_proto;
		acl_m->l2_ether_type = be16_to_cpu(mask->n_proto);
	}

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		struct flow_dissector_key_eth_addrs *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_ETH_ADDRS,
						  cls->key);
		struct flow_dissector_key_eth_addrs *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_ETH_ADDRS,
						  cls->mask);

		ether_addr_copy(acl_h->l2_dest_mac, &key->dst[0]);
		ether_addr_copy(acl_h->l2_source_mac, &key->src[0]);
		ether_addr_copy(acl_m->l2_dest_mac, &mask->dst[0]);
		ether_addr_copy(acl_m->l2_source_mac, &mask->src[0]);
	}

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_VLAN)) {
		struct flow_dissector_key_vlan *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_VLAN,
						  cls->key);
		struct flow_dissector_key_vlan *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_VLAN,
						  cls->mask);

		acl_h->l2_vlan_id = key->vlan_id;
		acl_h->l2_tpid = be16_to_cpu(key->vlan_tpid);
		acl_h->l2_pcp_dei = key->vlan_priority << 1;

		acl_m->l2_vlan_id = mask->vlan_id;
		acl_m->l2_tpid = be16_to_cpu(mask->vlan_tpid);
		acl_m->l2_pcp_dei = mask->vlan_priority << 1;
	}

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_IPV4_ADDRS)) {
		struct flow_dissector_key_ipv4_addrs *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_IPV4_ADDRS,
						  cls->key);
		struct flow_dissector_key_ipv4_addrs *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_IPV4_ADDRS,
						  cls->mask);

		acl_h->l3_source_ip = be32_to_cpu(key->src);
		acl_h->l3_dest_ip = be32_to_cpu(key->dst);
		acl_m->l3_source_ip = be32_to_cpu(mask->src);
		acl_m->l3_dest_ip = be32_to_cpu(mask->dst);
	}

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_PORTS)) {
		struct flow_dissector_key_ports *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_PORTS,
						  cls->key);
		struct flow_dissector_key_ports *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_PORTS,
						  cls->mask);

		acl_h->l4_source_port = be16_to_cpu(key->src);
		acl_h->l4_dest_port = be16_to_cpu(key->dst);
		acl_m->l4_source_port = be16_to_cpu(mask->src);
		acl_m->l4_dest_port = be16_to_cpu(mask->dst);
	}

	if (dissector_uses_key(dissector, FLOW_DISSECTOR_KEY_IP)) {
		struct flow_dissector_key_ip *key =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_IP,
						  cls->key);
		struct flow_dissector_key_ip *mask =
			skb_flow_dissector_target(dissector,
						  FLOW_DISSECTOR_KEY_IP,
						  cls->mask);

		if (mask->ttl != 0) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Matching on TTL not supported");
			return -EOPNOTSUPP;
		}

		if ((mask->tos & 0x3) != 0) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Matching on ECN not supported, only DSCP");
			return -EOPNOTSUPP;
		}

		acl_h->l3_dscp = key->tos >> 2;
		acl_m->l3_dscp = mask->tos >> 2;
	}

	return 0;
}

static int dpaa2_switch_acl_entry_add(struct dpaa2_switch_acl_tbl *acl_tbl,
				      struct dpaa2_switch_acl_entry *entry)
{
	struct dpsw_acl_entry_cfg *acl_entry_cfg = &entry->cfg;
	struct ethsw_core *ethsw = acl_tbl->ethsw;
	struct dpsw_acl_key *acl_key = &entry->key;
	struct device *dev = ethsw->dev;
	u8 *cmd_buff;
	int err;

	cmd_buff = kzalloc(DPAA2_ETHSW_PORT_ACL_CMD_BUF_SIZE, GFP_KERNEL);
	if (!cmd_buff)
		return -ENOMEM;

	dpsw_acl_prepare_entry_cfg(acl_key, cmd_buff);

	acl_entry_cfg->key_iova = dma_map_single(dev, cmd_buff,
						 DPAA2_ETHSW_PORT_ACL_CMD_BUF_SIZE,
						 DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, acl_entry_cfg->key_iova))) {
		dev_err(dev, "DMA mapping failed\n");
		return -EFAULT;
	}

	err = dpsw_acl_add_entry(ethsw->mc_io, 0, ethsw->dpsw_handle,
				 acl_tbl->id, acl_entry_cfg);

	dma_unmap_single(dev, acl_entry_cfg->key_iova, sizeof(cmd_buff),
			 DMA_TO_DEVICE);
	if (err) {
		dev_err(dev, "dpsw_acl_add_entry() failed %d\n", err);
		return err;
	}

	kfree(cmd_buff);

	return 0;
}

static int dpaa2_switch_acl_entry_remove(struct dpaa2_switch_acl_tbl *acl_tbl,
					 struct dpaa2_switch_acl_entry *entry)
{
	struct dpsw_acl_entry_cfg *acl_entry_cfg = &entry->cfg;
	struct dpsw_acl_key *acl_key = &entry->key;
	struct ethsw_core *ethsw = acl_tbl->ethsw;
	struct device *dev = ethsw->dev;
	u8 *cmd_buff;
	int err;

	cmd_buff = kzalloc(DPAA2_ETHSW_PORT_ACL_CMD_BUF_SIZE, GFP_KERNEL);
	if (!cmd_buff)
		return -ENOMEM;

	dpsw_acl_prepare_entry_cfg(acl_key, cmd_buff);

	acl_entry_cfg->key_iova = dma_map_single(dev, cmd_buff,
						 DPAA2_ETHSW_PORT_ACL_CMD_BUF_SIZE,
						 DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, acl_entry_cfg->key_iova))) {
		dev_err(dev, "DMA mapping failed\n");
		return -EFAULT;
	}

	err = dpsw_acl_remove_entry(ethsw->mc_io, 0, ethsw->dpsw_handle,
				    acl_tbl->id, acl_entry_cfg);

	dma_unmap_single(dev, acl_entry_cfg->key_iova, sizeof(cmd_buff),
			 DMA_TO_DEVICE);
	if (err) {
		dev_err(dev, "dpsw_acl_remove_entry() failed %d\n", err);
		return err;
	}

	kfree(cmd_buff);

	return 0;
}

static int
dpaa2_switch_acl_entry_add_to_list(struct dpaa2_switch_acl_tbl *acl_tbl,
				   struct dpaa2_switch_acl_entry *entry)
{
	struct dpaa2_switch_acl_entry *tmp;
	struct list_head *pos, *n;
	int index = 0;

	if (list_empty(&acl_tbl->entries)) {
		list_add(&entry->list, &acl_tbl->entries);
		return index;
	}

	list_for_each_safe(pos, n, &acl_tbl->entries) {
		tmp = list_entry(pos, struct dpaa2_switch_acl_entry, list);
		if (entry->prio < tmp->prio)
			break;
		index++;
	}
	list_add(&entry->list, pos->prev);
	return index;
}

static struct dpaa2_switch_acl_entry*
dpaa2_switch_acl_entry_get_by_index(struct dpaa2_switch_acl_tbl *acl_tbl,
				    int index)
{
	struct dpaa2_switch_acl_entry *tmp;
	int i = 0;

	list_for_each_entry(tmp, &acl_tbl->entries, list) {
		if (i == index)
			return tmp;
		++i;
	}

	return NULL;
}

static int
dpaa2_switch_acl_entry_set_precedence(struct dpaa2_switch_acl_tbl *acl_tbl,
				      struct dpaa2_switch_acl_entry *entry,
				      int precedence)
{
	struct device *dev = acl_tbl->ethsw->dev;
	int err;

	dev_dbg(dev, "Changing rule (cookie %lu, prio %u) to have precedence %d\n",
		entry->cookie, entry->prio, precedence);

	err = dpaa2_switch_acl_entry_remove(acl_tbl, entry);
	if (err)
		return err;

	entry->cfg.precedence = precedence;
	return dpaa2_switch_acl_entry_add(acl_tbl, entry);
}

static int dpaa2_switch_acl_tbl_add_entry(struct dpaa2_switch_acl_tbl *acl_tbl,
					  struct dpaa2_switch_acl_entry *entry)
{
	struct device *dev = acl_tbl->ethsw->dev;
	struct dpaa2_switch_acl_entry *tmp;
	int index, i, precedence, err;

	/* Add the new ACL entry to the linked list and get its index */
	index = dpaa2_switch_acl_entry_add_to_list(acl_tbl, entry);

	/* Move up in priority the ACL entries to make space
	 * for the new filter.
	 */
	precedence = DPAA2_ETHSW_PORT_MAX_ACL_ENTRIES - acl_tbl->num_rules - 1;
	for (i = 0; i < index; i++) {
		tmp = dpaa2_switch_acl_entry_get_by_index(acl_tbl, i);

		err = dpaa2_switch_acl_entry_set_precedence(acl_tbl, tmp,
							    precedence);
		if (err)
			return err;

		precedence++;
	}

	/* Add the new entry to hardware */
	entry->cfg.precedence = precedence;
	err = dpaa2_switch_acl_entry_add(acl_tbl, entry);
	acl_tbl->num_rules++;

	dev_dbg(dev, "Setting rule (cookie %lu, prio %u) to have precedence %d\n",
		entry->cookie, entry->prio, entry->cfg.precedence);

	return err;
}

static struct dpaa2_switch_acl_entry *
dpaa2_switch_acl_tbl_find_entry_by_cookie(struct dpaa2_switch_acl_tbl *acl_tbl,
					  unsigned long cookie)
{
	struct dpaa2_switch_acl_entry *tmp, *n;

	list_for_each_entry_safe(tmp, n, &acl_tbl->entries, list) {
		if (tmp->cookie == cookie)
			return tmp;
	}
	return NULL;
}

static int
dpaa2_switch_acl_entry_get_index(struct dpaa2_switch_acl_tbl *acl_tbl,
				 struct dpaa2_switch_acl_entry *entry)
{
	struct dpaa2_switch_acl_entry *tmp, *n;
	int index = 0;

	list_for_each_entry_safe(tmp, n, &acl_tbl->entries, list) {
		if (tmp->cookie == entry->cookie)
			return index;
		index++;
	}
	return -ENOENT;
}

static int
dpaa2_switch_acl_tbl_remove_entry(struct dpaa2_switch_acl_tbl *acl_tbl,
				  struct dpaa2_switch_acl_entry *entry)
{
	struct dpaa2_switch_acl_entry *tmp;
	int index, i, precedence, err;

	index = dpaa2_switch_acl_entry_get_index(acl_tbl, entry);

	/* Remove from hardware the ACL entry */
	err = dpaa2_switch_acl_entry_remove(acl_tbl, entry);
	if (err)
		return err;

	acl_tbl->num_rules--;

	/* Remove it from the list also */
	list_del(&entry->list);

	/* Move down in priority the entries over the deleted one */
	precedence = entry->cfg.precedence;
	for (i = index - 1; i >= 0; i--) {
		tmp = dpaa2_switch_acl_entry_get_by_index(acl_tbl, i);
		err = dpaa2_switch_acl_entry_set_precedence(acl_tbl, tmp,
							    precedence);
		if (err)
			return err;

		precedence--;
	}

	kfree(entry);

	return 0;
}

static int dpaa2_switch_tc_parse_action(struct ethsw_core *ethsw,
					struct tc_action *act,
					struct dpsw_acl_result *dpsw_act,
					struct netlink_ext_ack *extack)
{
	struct net_device *out_dev;
	int err = 0;

	if (is_tcf_gact_shot(act)) {
		dpsw_act->action = DPSW_ACL_ACTION_DROP;
	} else if (is_tcf_mirred_egress_redirect(act)) {
		out_dev = tcf_mirred_dev(act);
		if (!dpaa2_switch_port_dev_check(out_dev)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Destination not a DPAA2 switch port");
			return -EOPNOTSUPP;
		}
		dpsw_act->if_id = dpaa2_switch_get_index(ethsw, out_dev);
		dpsw_act->action = DPSW_ACL_ACTION_REDIRECT;
	} else {
		NL_SET_ERR_MSG_MOD(extack,
				   "Action not supported");
		err = -EOPNOTSUPP;
		goto out;
	}

out:
	return err;
}

int dpaa2_switch_cls_flower_replace(struct dpaa2_switch_acl_tbl *acl_tbl,
				    struct tc_cls_flower_offload *cls)
{
	struct netlink_ext_ack *extack = cls->common.extack;
	struct ethsw_core *ethsw = acl_tbl->ethsw;
	struct dpaa2_switch_acl_entry *acl_entry;
	struct tc_action *act;
	int err;

	dev_dbg(ethsw->dev, "Adding rule (cookie %lu, prio %u) into ACL tbl #%d\n",
		cls->cookie, cls->common.prio, acl_tbl->id);

	if (!tcf_exts_has_one_action(cls->exts)) {
		NL_SET_ERR_MSG(extack, "Only singular actions are supported");
		return -EOPNOTSUPP;
	}

	if (dpaa2_switch_acl_tbl_is_full(acl_tbl)) {
		NL_SET_ERR_MSG(extack, "Maximum filter capacity reached");
		return -ENOMEM;
	}

	acl_entry = kzalloc(sizeof(*acl_entry), GFP_KERNEL);
	if (!acl_entry)
		return -ENOMEM;

	err = dpaa2_switch_flower_parse_key(cls, &acl_entry->key);
	if (err)
		goto free_acl_entry;
	dpaa2_switch_flower_dump_key(ethsw, &acl_entry->key);

	act = tcf_exts_first_action(cls->exts);
	err = dpaa2_switch_tc_parse_action(ethsw, act,
					   &acl_entry->cfg.result, extack);
	if (err)
		goto free_acl_entry;
	dpaa2_switch_tc_dump_action(ethsw, &acl_entry->cfg.result);

	acl_entry->prio = cls->common.prio;
	acl_entry->cookie = cls->cookie;

	err = dpaa2_switch_acl_tbl_add_entry(acl_tbl, acl_entry);
	if (err)
		goto free_acl_entry;
	return 0;

free_acl_entry:
	kfree(acl_entry);

	return err;
}

int dpaa2_switch_cls_flower_destroy(struct dpaa2_switch_acl_tbl *acl_tbl,
				    struct tc_cls_flower_offload *cls)
{
	struct device *dev = acl_tbl->ethsw->dev;
	struct dpaa2_switch_acl_entry *entry;

	dev_dbg(dev, "Removing rule (cookie %lu, prio %u) from ACL tbl #%d\n",
		cls->cookie, cls->common.prio, acl_tbl->id);

	entry = dpaa2_switch_acl_tbl_find_entry_by_cookie(acl_tbl, cls->cookie);
	if (!entry)
		return 0;

	return dpaa2_switch_acl_tbl_remove_entry(acl_tbl, entry);
}
