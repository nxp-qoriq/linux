/* Copyright 2013-2016 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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
#ifndef __FSL_DPSW_CMD_H
#define __FSL_DPSW_CMD_H

/* DPSW Version */
#define DPSW_VER_MAJOR		8
#define DPSW_VER_MINOR		0

#define DPSW_CMD_BASE_VERSION	1
#define DPSW_CMD_ID_OFFSET	4

#define DPSW_CMD_ID(id)	(((id) << DPSW_CMD_ID_OFFSET) | DPSW_CMD_BASE_VERSION)

/* Command IDs */
#define DPSW_CMDID_CLOSE                    DPSW_CMD_ID(0x800)
#define DPSW_CMDID_OPEN                     DPSW_CMD_ID(0x802)

#define DPSW_CMDID_GET_API_VERSION          DPSW_CMD_ID(0xa02)

#define DPSW_CMDID_ENABLE                   DPSW_CMD_ID(0x002)
#define DPSW_CMDID_DISABLE                  DPSW_CMD_ID(0x003)
#define DPSW_CMDID_GET_ATTR                 DPSW_CMD_ID(0x004)
#define DPSW_CMDID_RESET                    DPSW_CMD_ID(0x005)
#define DPSW_CMDID_IS_ENABLED               DPSW_CMD_ID(0x006)

#define DPSW_CMDID_SET_IRQ                  DPSW_CMD_ID(0x010)
#define DPSW_CMDID_GET_IRQ                  DPSW_CMD_ID(0x011)
#define DPSW_CMDID_SET_IRQ_ENABLE           DPSW_CMD_ID(0x012)
#define DPSW_CMDID_GET_IRQ_ENABLE           DPSW_CMD_ID(0x013)
#define DPSW_CMDID_SET_IRQ_MASK             DPSW_CMD_ID(0x014)
#define DPSW_CMDID_GET_IRQ_MASK             DPSW_CMD_ID(0x015)
#define DPSW_CMDID_GET_IRQ_STATUS           DPSW_CMD_ID(0x016)
#define DPSW_CMDID_CLEAR_IRQ_STATUS         DPSW_CMD_ID(0x017)

#define DPSW_CMDID_SET_REFLECTION_IF        DPSW_CMD_ID(0x022)

#define DPSW_CMDID_ADD_CUSTOM_TPID          DPSW_CMD_ID(0x024)

#define DPSW_CMDID_REMOVE_CUSTOM_TPID       DPSW_CMD_ID(0x026)

#define DPSW_CMDID_IF_SET_TCI               DPSW_CMD_ID(0x030)
#define DPSW_CMDID_IF_SET_STP               DPSW_CMD_ID(0x031)
#define DPSW_CMDID_IF_SET_ACCEPTED_FRAMES   DPSW_CMD_ID(0x032)
#define DPSW_CMDID_SET_IF_ACCEPT_ALL_VLAN   DPSW_CMD_ID(0x033)
#define DPSW_CMDID_IF_GET_COUNTER           DPSW_CMD_ID(0x034)
#define DPSW_CMDID_IF_SET_COUNTER           DPSW_CMD_ID(0x035)
#define DPSW_CMDID_IF_SET_TX_SELECTION      DPSW_CMD_ID(0x036)
#define DPSW_CMDID_IF_ADD_REFLECTION        DPSW_CMD_ID(0x037)
#define DPSW_CMDID_IF_REMOVE_REFLECTION     DPSW_CMD_ID(0x038)
#define DPSW_CMDID_IF_SET_FLOODING_METERING DPSW_CMD_ID(0x039)
#define DPSW_CMDID_IF_SET_METERING          DPSW_CMD_ID(0x03A)
#define DPSW_CMDID_IF_SET_EARLY_DROP        DPSW_CMD_ID(0x03B)

#define DPSW_CMDID_IF_ENABLE                DPSW_CMD_ID(0x03D)
#define DPSW_CMDID_IF_DISABLE               DPSW_CMD_ID(0x03E)

#define DPSW_CMDID_IF_GET_ATTR              DPSW_CMD_ID(0x042)

#define DPSW_CMDID_IF_SET_MAX_FRAME_LENGTH  DPSW_CMD_ID(0x044)
#define DPSW_CMDID_IF_GET_MAX_FRAME_LENGTH  DPSW_CMD_ID(0x045)
#define DPSW_CMDID_IF_GET_LINK_STATE        DPSW_CMD_ID(0x046)
#define DPSW_CMDID_IF_SET_FLOODING          DPSW_CMD_ID(0x047)
#define DPSW_CMDID_IF_SET_BROADCAST         DPSW_CMD_ID(0x048)
#define DPSW_CMDID_IF_SET_MULTICAST         DPSW_CMD_ID(0x049)
#define DPSW_CMDID_IF_GET_TCI               DPSW_CMD_ID(0x04A)

#define DPSW_CMDID_IF_SET_LINK_CFG          DPSW_CMD_ID(0x04C)

#define DPSW_CMDID_VLAN_ADD                 DPSW_CMD_ID(0x060)
#define DPSW_CMDID_VLAN_ADD_IF              DPSW_CMD_ID(0x061)
#define DPSW_CMDID_VLAN_ADD_IF_UNTAGGED     DPSW_CMD_ID(0x062)
#define DPSW_CMDID_VLAN_ADD_IF_FLOODING     DPSW_CMD_ID(0x063)
#define DPSW_CMDID_VLAN_REMOVE_IF           DPSW_CMD_ID(0x064)
#define DPSW_CMDID_VLAN_REMOVE_IF_UNTAGGED  DPSW_CMD_ID(0x065)
#define DPSW_CMDID_VLAN_REMOVE_IF_FLOODING  DPSW_CMD_ID(0x066)
#define DPSW_CMDID_VLAN_REMOVE              DPSW_CMD_ID(0x067)
#define DPSW_CMDID_VLAN_GET_IF              DPSW_CMD_ID(0x068)
#define DPSW_CMDID_VLAN_GET_IF_FLOODING     DPSW_CMD_ID(0x069)
#define DPSW_CMDID_VLAN_GET_IF_UNTAGGED     DPSW_CMD_ID(0x06A)
#define DPSW_CMDID_VLAN_GET_ATTRIBUTES      DPSW_CMD_ID(0x06B)

#define DPSW_CMDID_FDB_GET_MULTICAST        DPSW_CMD_ID(0x080)
#define DPSW_CMDID_FDB_GET_UNICAST          DPSW_CMD_ID(0x081)
#define DPSW_CMDID_FDB_ADD                  DPSW_CMD_ID(0x082)
#define DPSW_CMDID_FDB_REMOVE               DPSW_CMD_ID(0x083)
#define DPSW_CMDID_FDB_ADD_UNICAST          DPSW_CMD_ID(0x084)
#define DPSW_CMDID_FDB_REMOVE_UNICAST       DPSW_CMD_ID(0x085)
#define DPSW_CMDID_FDB_ADD_MULTICAST        DPSW_CMD_ID(0x086)
#define DPSW_CMDID_FDB_REMOVE_MULTICAST     DPSW_CMD_ID(0x087)
#define DPSW_CMDID_FDB_SET_LEARNING_MODE    DPSW_CMD_ID(0x088)
#define DPSW_CMDID_FDB_GET_ATTR             DPSW_CMD_ID(0x089)

#define DPSW_CMDID_ACL_ADD                  DPSW_CMD_ID(0x090)
#define DPSW_CMDID_ACL_REMOVE               DPSW_CMD_ID(0x091)
#define DPSW_CMDID_ACL_ADD_ENTRY            DPSW_CMD_ID(0x092)
#define DPSW_CMDID_ACL_REMOVE_ENTRY         DPSW_CMD_ID(0x093)
#define DPSW_CMDID_ACL_ADD_IF               DPSW_CMD_ID(0x094)
#define DPSW_CMDID_ACL_REMOVE_IF            DPSW_CMD_ID(0x095)
#define DPSW_CMDID_ACL_GET_ATTR             DPSW_CMD_ID(0x096)

#define DPSW_CMDID_CTRL_IF_GET_ATTR         DPSW_CMD_ID(0x0A0)
#define DPSW_CMDID_CTRL_IF_SET_POOLS        DPSW_CMD_ID(0x0A1)
#define DPSW_CMDID_CTRL_IF_ENABLE           DPSW_CMD_ID(0x0A2)
#define DPSW_CMDID_CTRL_IF_DISABLE          DPSW_CMD_ID(0x0A3)

/* Macros for accessing command fields smaller than 1byte */
#define DPSW_MASK(field)        \
	GENMASK(DPSW_##field##_SHIFT + DPSW_##field##_SIZE - 1, \
		DPSW_##field##_SHIFT)
#define dpsw_set_field(var, field, val) \
	((var) |= (((val) << DPSW_##field##_SHIFT) & DPSW_MASK(field)))
#define dpsw_get_field(var, field)      \
	(((var) & DPSW_MASK(field)) >> DPSW_##field##_SHIFT)
#define dpsw_get_bit(var, bit) \
	(((var)  >> (bit)) & GENMASK(0, 0))

static inline u64 dpsw_set_bit(u64 var, unsigned int bit, u8 val)
{
	var |= (u64)val << bit & GENMASK(bit, bit);
	return var;
}

struct dpsw_cmd_open {
	__le32 dpsw_id;
};

#define DPSW_COMPONENT_TYPE_SHIFT	0
#define DPSW_COMPONENT_TYPE_SIZE	4

struct dpsw_cmd_create {
	/* cmd word 0 */
	__le16 num_ifs;
	u8 max_fdbs;
	u8 max_meters_per_if;
	/* from LSB: only the first 4 bits */
	u8 component_type;
	u8 pad[3];
	/* cmd word 1 */
	__le16 max_vlans;
	__le16 max_fdb_entries;
	__le16 fdb_aging_time;
	__le16 max_fdb_mc_groups;
	/* cmd word 2 */
	__le64 options;
};

struct dpsw_cmd_destroy {
	__le32 dpsw_id;
};

#define DPSW_ENABLE_SHIFT 0
#define DPSW_ENABLE_SIZE  1

struct dpsw_rsp_is_enabled {
	/* from LSB: enable:1 */
	u8 enabled;
};

struct dpsw_cmd_set_irq {
	/* cmd word 0 */
	u8 irq_index;
	u8 pad[3];
	__le32 irq_val;
	/* cmd word 1 */
	__le64 irq_addr;
	/* cmd word 2 */
	__le32 irq_num;
};

struct dpsw_cmd_get_irq {
	__le32 pad;
	u8 irq_index;
};

struct dpsw_rsp_get_irq {
	/* cmd word 0 */
	__le32 irq_val;
	__le32 pad;
	/* cmd word 1 */
	__le64 irq_addr;
	/* cmd word 2 */
	__le32 irq_num;
	__le32 irq_type;
};

struct dpsw_cmd_set_irq_enable {
	u8 enable_state;
	u8 pad[3];
	u8 irq_index;
};

struct dpsw_cmd_get_irq_enable {
	__le32 pad;
	u8 irq_index;
};

struct dpsw_rsp_get_irq_enable {
	u8 enable_state;
};

struct dpsw_cmd_set_irq_mask {
	__le32 mask;
	u8 irq_index;
};

struct dpsw_cmd_get_irq_mask {
	__le32 pad;
	u8 irq_index;
};

struct dpsw_rsp_get_irq_mask {
	__le32 mask;
};

struct dpsw_cmd_get_irq_status {
	__le32 status;
	u8 irq_index;
};

struct dpsw_rsp_get_irq_status {
	__le32 status;
};

struct dpsw_cmd_clear_irq_status {
	__le32 status;
	u8 irq_index;
};

#define DPSW_COMPONENT_TYPE_SHIFT	0
#define DPSW_COMPONENT_TYPE_SIZE	4

struct dpsw_rsp_get_attr {
	/* cmd word 0 */
	__le16 num_ifs;
	u8 max_fdbs;
	u8 num_fdbs;
	__le16 max_vlans;
	__le16 num_vlans;
	/* cmd word 1 */
	__le16 max_fdb_entries;
	__le16 fdb_aging_time;
	__le32 dpsw_id;
	/* cmd word 2 */
	__le16 mem_size;
	__le16 max_fdb_mc_groups;
	u8 max_meters_per_if;
	/* from LSB only the ffirst 4 bits */
	u8 component_type;
	__le16 pad;
	/* cmd word 3 */
	__le64 options;
};

struct dpsw_cmd_set_reflection_if {
	__le16 if_id;
};

struct dpsw_cmd_if_set_flooding {
	__le16 if_id;
	/* from LSB: enable:1 */
	u8 enable;
};

struct dpsw_cmd_if_set_broadcast {
	__le16 if_id;
	/* from LSB: enable:1 */
	u8 enable;
};

struct dpsw_cmd_if_set_multicast {
	__le16 if_id;
	/* from LSB: enable:1 */
	u8 enable;
};

#define DPSW_VLAN_ID_SHIFT	0
#define DPSW_VLAN_ID_SIZE	12
#define DPSW_DEI_SHIFT		12
#define DPSW_DEI_SIZE		1
#define DPSW_PCP_SHIFT		13
#define DPSW_PCP_SIZE		3

struct dpsw_cmd_if_set_tci {
	__le16 if_id;
	/* from LSB: VLAN_ID:12 DEI:1 PCP:3 */
	__le16 conf;
};

struct dpsw_cmd_if_get_tci {
	__le16 if_id;
};

struct dpsw_rsp_if_get_tci {
	__le16 pad;
	__le16 vlan_id;
	u8 dei;
	u8 pcp;
};

#define DPSW_STATE_SHIFT	0
#define DPSW_STATE_SIZE		4

struct dpsw_cmd_if_set_stp {
	__le16 if_id;
	__le16 vlan_id;
	/* only the first LSB 4 bits */
	u8 state;
};

#define DPSW_FRAME_TYPE_SHIFT		0
#define DPSW_FRAME_TYPE_SIZE		4
#define DPSW_UNACCEPTED_ACT_SHIFT	4
#define DPSW_UNACCEPTED_ACT_SIZE	4

struct dpsw_cmd_if_set_accepted_frames {
	__le16 if_id;
	/* from LSB: type:4 unaccepted_act:4 */
	u8 unaccepted;
};

#define DPSW_ACCEPT_ALL_SHIFT	0
#define DPSW_ACCEPT_ALL_SIZE	1

struct dpsw_cmd_if_set_accept_all_vlan {
	__le16 if_id;
	/* only the least significant bit */
	u8 accept_all;
};

#define DPSW_COUNTER_TYPE_SHIFT		0
#define DPSW_COUNTER_TYPE_SIZE		5

struct dpsw_cmd_if_get_counter {
	__le16 if_id;
	/* from LSB: type:5 */
	u8 type;
};

struct dpsw_rsp_if_get_counter {
	__le64 pad;
	__le64 counter;
};

struct dpsw_cmd_if_set_counter {
	/* cmd word 0 */
	__le16 if_id;
	/* from LSB: type:5 */
	u8 type;
	/* cmd word 1 */
	__le64 counter;
};

#define DPSW_PRIORITY_SELECTOR_SHIFT	0
#define DPSW_PRIORITY_SELECTOR_SIZE	3
#define DPSW_SCHED_MODE_SHIFT		0
#define DPSW_SCHED_MODE_SIZE		4

struct dpsw_cmd_if_set_tx_selection {
	__le16 if_id;
	/* from LSB: priority_selector:3 */
	u8 priority_selector;
	u8 pad[5];
	u8 tc_id[8];

	struct dpsw_tc_sched {
		__le16 delta_bandwidth;
		u8 mode;
		u8 pad;
	} tc_sched[8];
};

#define DPSW_FILTER_SHIFT	0
#define DPSW_FILTER_SIZE	2

struct dpsw_cmd_if_reflection {
	__le16 if_id;
	__le16 vlan_id;
	/* only 2 bits from the LSB */
	u8 filter;
};

#define DPSW_MODE_SHIFT		0
#define DPSW_MODE_SIZE		4
#define DPSW_UNITS_SHIFT	4
#define DPSW_UNITS_SIZE		4

struct dpsw_cmd_if_set_flooding_metering {
	/* cmd word 0 */
	__le16 if_id;
	u8 pad;
	/* from LSB: mode:4 units:4 */
	u8 mode_units;
	__le32 cir;
	/* cmd word 1 */
	__le32 eir;
	__le32 cbs;
	/* cmd word 2 */
	__le32 ebs;
};

struct dpsw_cmd_if_set_metering {
	/* cmd word 0 */
	__le16 if_id;
	u8 tc_id;
	/* from LSB: mode:4 units:4 */
	u8 mode_units;
	__le32 cir;
	/* cmd word 1 */
	__le32 eir;
	__le32 cbs;
	/* cmd word 2 */
	__le32 ebs;
};

#define DPSW_EARLY_DROP_MODE_SHIFT	0
#define DPSW_EARLY_DROP_MODE_SIZE	2
#define DPSW_EARLY_DROP_UNIT_SHIFT	2
#define DPSW_EARLY_DROP_UNIT_SIZE	2

struct dpsw_prep_early_drop {
	/* from LSB: mode:2 units:2 */
	u8 conf;
	u8 pad0[3];
	__le32 tail_drop_threshold;
	u8 green_drop_probability;
	u8 pad1[7];
	__le64 green_max_threshold;
	__le64 green_min_threshold;
	__le64 pad2;
	u8 yellow_drop_probability;
	u8 pad3[7];
	__le64 yellow_max_threshold;
	__le64 yellow_min_threshold;
};

struct dpsw_cmd_if_set_early_drop {
	/* cmd word 0 */
	u8 pad0;
	u8 tc_id;
	__le16 if_id;
	__le32 pad1;
	/* cmd word 1 */
	__le64 early_drop_iova;
};

struct dpsw_cmd_custom_tpid {
	__le16 pad;
	__le16 tpid;
};

struct dpsw_cmd_if {
	__le16 if_id;
};

#define DPSW_ADMIT_UNTAGGED_SHIFT	0
#define DPSW_ADMIT_UNTAGGED_SIZE	4
#define DPSW_ENABLED_SHIFT		5
#define DPSW_ENABLED_SIZE		1
#define DPSW_ACCEPT_ALL_VLAN_SHIFT	6
#define DPSW_ACCEPT_ALL_VLAN_SIZE	1

struct dpsw_rsp_if_get_attr {
	/* cmd word 0 */
	/* from LSB: admit_untagged:4 enabled:1 accept_all_vlan:1 */
	u8 conf;
	u8 pad1;
	u8 num_tcs;
	u8 pad2;
	__le16 qdid;
	/* cmd word 1 */
	__le32 options;
	__le32 pad3;
	/* cmd word 2 */
	__le32 rate;
};

struct dpsw_cmd_if_set_max_frame_length {
	__le16 if_id;
	__le16 frame_length;
};

struct dpsw_cmd_if_get_max_frame_length {
	__le16 if_id;
};

struct dpsw_rsp_if_get_max_frame_length {
	__le16 pad;
	__le16 frame_length;
};

struct dpsw_cmd_if_set_link_cfg {
	/* cmd word 0 */
	__le16 if_id;
	u8 pad[6];
	/* cmd word 1 */
	__le32 rate;
	__le32 pad1;
	/* cmd word 2 */
	__le64 options;
};

struct dpsw_cmd_if_get_link_state {
	__le16 if_id;
};

#define DPSW_UP_SHIFT	0
#define DPSW_UP_SIZE	1

struct dpsw_rsp_if_get_link_state {
	/* cmd word 0 */
	__le32 pad0;
	u8 up;
	u8 pad1[3];
	/* cmd word 1 */
	__le32 rate;
	__le32 pad2;
	/* cmd word 2 */
	__le64 options;
};

struct dpsw_vlan_add {
	__le16 fdb_id;
	__le16 vlan_id;
};

struct dpsw_cmd_vlan_manage_if {
	/* cmd word 0 */
	__le16 pad0;
	__le16 vlan_id;
	__le32 pad1;
	/* cmd word 1 */
	__le64 if_id[4];
};

struct dpsw_cmd_vlan_remove {
	__le16 pad;
	__le16 vlan_id;
};

struct dpsw_cmd_vlan_get_attr {
	__le16 vlan_id;
};

struct dpsw_rsp_vlan_get_attr {
	/* cmd word 0 */
	__le64 pad;
	/* cmd word 1 */
	__le16 fdb_id;
	__le16 num_ifs;
	__le16 num_untagged_ifs;
	__le16 num_flooding_ifs;
};

struct dpsw_cmd_vlan_get_if {
	__le16 vlan_id;
};

struct dpsw_rsp_vlan_get_if {
	/* cmd word 0 */
	__le16 pad0;
	__le16 num_ifs;
	u8 pad1[4];
	/* cmd word 1 */
	__le64 if_id[4];
};

struct dpsw_cmd_vlan_get_if_untagged {
	__le16 vlan_id;
};

struct dpsw_rsp_vlan_get_if_untagged {
	/* cmd word 0 */
	__le16 pad0;
	__le16 num_ifs;
	u8 pad1[4];
	/* cmd word 1 */
	__le64 if_id[4];
};

struct dpsw_cmd_vlan_get_if_flooding {
	__le16 vlan_id;
};

struct dpsw_rsp_vlan_get_if_flooding {
	/* cmd word 0 */
	__le16 pad0;
	__le16 num_ifs;
	u8 pad1[4];
	/* cmd word 1 */
	__le64 if_id[4];
};

struct dpsw_cmd_fdb_add {
	__le32 pad;
	__le16 fdb_aging_time;
	__le16 num_fdb_entries;
};

struct dpsw_rsp_fdb_add {
	__le16 fdb_id;
};

struct dpsw_cmd_fdb_remove {
	__le16 fdb_id;
};

#define DPSW_ENTRY_TYPE_SHIFT	0
#define DPSW_ENTRY_TYPE_SIZE	4

struct dpsw_cmd_fdb_add_unicast {
	/* cmd word 0 */
	__le16 fdb_id;
	u8 mac_addr[6];
	/* cmd word 1 */
	u8 if_egress;
	u8 pad;
	/* only the first 4 bits from LSB */
	u8 type;
};

struct dpsw_cmd_fdb_get_unicast {
	__le16 fdb_id;
	u8 mac_addr[6];
};

struct dpsw_rsp_fdb_get_unicast {
	__le64 pad;
	__le16 if_egress;
	/* only first 4 bits from LSB */
	u8 type;
};

struct dpsw_cmd_fdb_remove_unicast {
	/* cmd word 0 */
	__le16 fdb_id;
	u8 mac_addr[6];
	/* cmd word 1 */
	__le16 if_egress;
	/* only the first 4 bits from LSB */
	u8 type;
};

struct dpsw_cmd_fdb_add_multicast {
	/* cmd word 0 */
	__le16 fdb_id;
	__le16 num_ifs;
	/* only the first 4 bits from LSB */
	u8 type;
	u8 pad[3];
	/* cmd word 1 */
	u8 mac_addr[6];
	__le16 pad2;
	/* cmd word 2 */
	__le64 if_id[4];
};

struct dpsw_cmd_fdb_get_multicast {
	__le16 fdb_id;
	u8 mac_addr[6];
};

struct dpsw_rsp_fdb_get_multicast {
	/* cmd word 0 */
	__le64 pad0;
	/* cmd word 1 */
	__le16 num_ifs;
	/* only the first 4 bits from LSB */
	u8 type;
	u8 pad1[5];
	/* cmd word 2 */
	__le64 if_id[4];
};

struct dpsw_cmd_fdb_remove_multicast {
	/* cmd word 0 */
	__le16 fdb_id;
	__le16 num_ifs;
	/* only the first 4 bits from LSB */
	u8 type;
	u8 pad[3];
	/* cmd word 1 */
	u8 mac_addr[6];
	__le16 pad2;
	/* cmd word 2 */
	__le64 if_id[4];
};

#define DPSW_LEARNING_MODE_SHIFT	0
#define DPSW_LEARNING_MODE_SIZE		4

struct dpsw_cmd_fdb_set_learning_mode {
	__le16 fdb_id;
	/* only the first 4 bits from LSB */
	u8 mode;
};

struct dpsw_cmd_fdb_get_attr {
	__le16 fdb_id;
};

struct dpsw_rsp_fdb_get_attr {
	/* cmd word 0 */
	__le16 pad;
	__le16 max_fdb_entries;
	__le16 fdb_aging_time;
	__le16 num_fdb_mc_groups;
	/* cmd word 1 */
	__le16 max_fdb_mc_groups;
	/* only the first 4 bits from LSB */
	u8 learning_mode;
};

struct dpsw_cmd_acl_add {
	__le16 pad;
	__le16 max_entries;
};

struct dpsw_rsp_acl_add {
	__le16 acl_id;
};

struct dpsw_cmd_acl_remove {
	__le16 acl_id;
};

struct dpsw_prep_acl_entry {
	u8 match_l2_dest_mac[6];
	__le16 match_l2_tpid;

	u8 match_l2_source_mac[6];
	__le16 match_l2_vlan_id;

	__le32 match_l3_dest_ip;
	__le32 match_l3_source_ip;

	__le16 match_l4_dest_port;
	__le16 match_l4_source_port;
	__le16 match_l2_ether_type;
	u8 match_l2_pcp_dei;
	u8 match_l3_dscp;

	u8 mask_l2_dest_mac[6];
	__le16 mask_l2_tpid;

	u8 mask_l2_source_mac[6];
	__le16 mask_l2_vlan_id;

	__le32 mask_l3_dest_ip;
	__le32 mask_l3_source_ip;

	__le16 mask_l4_dest_port;
	__le16 mask_l4_source_port;
	__le16 mask_l2_ether_type;
	u8 mask_l2_pcp_dei;
	u8 mask_l3_dscp;

	u8 match_l3_protocol;
	u8 mask_l3_protocol;
};

#define DPSW_RESULT_ACTION_SHIFT	0
#define DPSW_RESULT_ACTION_SIZE		4

struct dpsw_cmd_acl_entry {
	__le16 acl_id;
	__le16 result_if_id;
	__le32 precedence;
	/* from LSB only the first 4 bits */
	u8 result_action;
	u8 pad[7];
	__le64 pad2[4];
	__le64 key_iova;
};

struct dpsw_cmd_acl_if {
	/* cmd word 0 */
	__le16 acl_id;
	__le16 num_ifs;
	__le32 pad;
	/* cmd word 1 */
	__le64 if_id[4];
};

struct dpsw_cmd_acl_get_attr {
	__le16 acl_id;
};

struct dpsw_rsp_acl_get_attr {
	/* cmd word 0 */
	__le64 pad;
	/* cmd word 1 */
	__le16 max_entries;
	__le16 num_entries;
	__le16 num_ifs;
};

struct dpsw_rsp_ctrl_if_get_attr {
	/* cmd word 0 */
	__le64 pad;
	/* cmd word 1 */
	__le32 rx_fqid;
	__le32 rx_err_fqid;
	/* cmd word 2 */
	__le32 tx_err_conf_fqid;
};

struct dpsw_cmd_ctrl_if_set_pools {
	u8 num_dpbp;
	/* from LSB: POOL0_BACKUP_POOL:1 ... POOL7_BACKUP_POOL */
	u8 backup_pool;
	__le16 pad;
	__le32 dpbp_id[8];
	__le16 buffer_size[8];
};

struct dpsw_rsp_get_api_version {
	__le16 version_major;
	__le16 version_minor;
};

#endif /* __FSL_DPSW_CMD_H */
