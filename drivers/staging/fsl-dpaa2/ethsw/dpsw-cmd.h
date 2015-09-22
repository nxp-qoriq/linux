/* Copyright 2013-2015 Freescale Semiconductor Inc.
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
#define DPSW_VER_MAJOR				7
#define DPSW_VER_MINOR				0

/* Command IDs */
#define DPSW_CMDID_CLOSE			0x800
#define DPSW_CMDID_OPEN				0x802
#define DPSW_CMDID_CREATE			0x902
#define DPSW_CMDID_DESTROY			0x900

#define DPSW_CMDID_ENABLE			0x002
#define DPSW_CMDID_DISABLE			0x003
#define DPSW_CMDID_GET_ATTR			0x004
#define DPSW_CMDID_RESET			0x005
#define DPSW_CMDID_IS_ENABLED			0x006

#define DPSW_CMDID_SET_IRQ			0x010
#define DPSW_CMDID_GET_IRQ			0x011
#define DPSW_CMDID_SET_IRQ_ENABLE		0x012
#define DPSW_CMDID_GET_IRQ_ENABLE		0x013
#define DPSW_CMDID_SET_IRQ_MASK			0x014
#define DPSW_CMDID_GET_IRQ_MASK			0x015
#define DPSW_CMDID_GET_IRQ_STATUS		0x016
#define DPSW_CMDID_CLEAR_IRQ_STATUS		0x017

#define DPSW_CMDID_SET_REFLECTION_IF		0x022

#define DPSW_CMDID_ADD_CUSTOM_TPID		0x024

#define DPSW_CMDID_REMOVE_CUSTOM_TPID		0x026

#define DPSW_CMDID_IF_SET_TCI			0x030
#define DPSW_CMDID_IF_SET_STP			0x031
#define DPSW_CMDID_IF_SET_ACCEPTED_FRAMES	0x032
#define DPSW_CMDID_SET_IF_ACCEPT_ALL_VLAN	0x033
#define DPSW_CMDID_IF_GET_COUNTER		0x034
#define DPSW_CMDID_IF_SET_COUNTER		0x035
#define DPSW_CMDID_IF_SET_TX_SELECTION		0x036
#define DPSW_CMDID_IF_ADD_REFLECTION		0x037
#define DPSW_CMDID_IF_REMOVE_REFLECTION		0x038
#define DPSW_CMDID_IF_SET_FLOODING_METERING	0x039
#define DPSW_CMDID_IF_SET_METERING		0x03A
#define DPSW_CMDID_IF_SET_EARLY_DROP		0x03B

#define DPSW_CMDID_IF_ENABLE			0x03D
#define DPSW_CMDID_IF_DISABLE			0x03E

#define DPSW_CMDID_IF_GET_ATTR			0x042

#define DPSW_CMDID_IF_SET_MAX_FRAME_LENGTH	0x044
#define DPSW_CMDID_IF_GET_MAX_FRAME_LENGTH	0x045
#define DPSW_CMDID_IF_GET_LINK_STATE		0x046
#define DPSW_CMDID_IF_SET_FLOODING		0x047
#define DPSW_CMDID_IF_SET_BROADCAST		0x048
#define DPSW_CMDID_IF_SET_MULTICAST		0x049
#define DPSW_CMDID_IF_GET_TCI			0x04A

#define DPSW_CMDID_IF_SET_LINK_CFG		0x04C

#define DPSW_CMDID_VLAN_ADD			0x060
#define DPSW_CMDID_VLAN_ADD_IF			0x061
#define DPSW_CMDID_VLAN_ADD_IF_UNTAGGED		0x062
#define DPSW_CMDID_VLAN_ADD_IF_FLOODING		0x063
#define DPSW_CMDID_VLAN_REMOVE_IF		0x064
#define DPSW_CMDID_VLAN_REMOVE_IF_UNTAGGED	0x065
#define DPSW_CMDID_VLAN_REMOVE_IF_FLOODING	0x066
#define DPSW_CMDID_VLAN_REMOVE			0x067
#define DPSW_CMDID_VLAN_GET_IF			0x068
#define DPSW_CMDID_VLAN_GET_IF_FLOODING		0x069
#define DPSW_CMDID_VLAN_GET_IF_UNTAGGED		0x06A
#define DPSW_CMDID_VLAN_GET_ATTRIBUTES		0x06B

#define DPSW_CMDID_FDB_GET_MULTICAST		0x080
#define DPSW_CMDID_FDB_GET_UNICAST		0x081
#define DPSW_CMDID_FDB_ADD			0x082
#define DPSW_CMDID_FDB_REMOVE			0x083
#define DPSW_CMDID_FDB_ADD_UNICAST		0x084
#define DPSW_CMDID_FDB_REMOVE_UNICAST		0x085
#define DPSW_CMDID_FDB_ADD_MULTICAST		0x086
#define DPSW_CMDID_FDB_REMOVE_MULTICAST		0x087
#define DPSW_CMDID_FDB_SET_LEARNING_MODE	0x088
#define DPSW_CMDID_FDB_GET_ATTR			0x089

#define DPSW_CMDID_ACL_ADD			0x090
#define DPSW_CMDID_ACL_REMOVE			0x091
#define DPSW_CMDID_ACL_ADD_ENTRY		0x092
#define DPSW_CMDID_ACL_REMOVE_ENTRY		0x093
#define DPSW_CMDID_ACL_ADD_IF			0x094
#define DPSW_CMDID_ACL_REMOVE_IF		0x095
#define DPSW_CMDID_ACL_GET_ATTR			0x096

#define DPSW_CMDID_CTRL_IF_GET_ATTR		0x0A0
#define DPSW_CMDID_CTRL_IF_SET_POOLS		0x0A1
#define DPSW_CMDID_CTRL_IF_ENABLE		0x0A2
#define DPSW_CMDID_CTRL_IF_DISABLE		0x0A3

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_OPEN(cmd, dpsw_id) \
	MC_CMD_OP(cmd, 0, 0,  32, int,	 dpsw_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_CREATE(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, cfg->num_ifs);\
	MC_CMD_OP(cmd, 0, 16,  8, uint8_t,  cfg->adv.max_fdbs);\
	MC_CMD_OP(cmd, 0, 24,  8, uint8_t,  cfg->adv.max_meters_per_if);\
	MC_CMD_OP(cmd, 0, 32,  4, enum dpsw_component_type,  \
			cfg->adv.component_type);\
	MC_CMD_OP(cmd, 1, 0,  16, uint16_t, cfg->adv.max_vlans);\
	MC_CMD_OP(cmd, 1, 16, 16, uint16_t, cfg->adv.max_fdb_entries);\
	MC_CMD_OP(cmd, 1, 32, 16, uint16_t, cfg->adv.fdb_aging_time);\
	MC_CMD_OP(cmd, 1, 48, 16, uint16_t, cfg->adv.max_fdb_mc_groups);\
	MC_CMD_OP(cmd, 2, 0,  64, uint64_t, cfg->adv.options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IS_ENABLED(cmd, en) \
	MC_RSP_OP(cmd, 0, 0,  1,  int,	    en)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_SET_IRQ(cmd, irq_index, irq_cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  irq_index);\
	MC_CMD_OP(cmd, 0, 32, 32, uint32_t, irq_cfg->val);\
	MC_CMD_OP(cmd, 1, 0,  64, uint64_t, irq_cfg->addr);\
	MC_CMD_OP(cmd, 2, 0,  32, int,	    irq_cfg->irq_num); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_GET_IRQ(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_GET_IRQ(cmd, type, irq_cfg) \
do { \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, irq_cfg->val); \
	MC_RSP_OP(cmd, 1, 0,  64, uint64_t, irq_cfg->addr);\
	MC_RSP_OP(cmd, 2, 0,  32, int,	    irq_cfg->irq_num); \
	MC_RSP_OP(cmd, 2, 32, 32, int,	    type); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_SET_IRQ_ENABLE(cmd, irq_index, enable_state) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8, uint8_t, enable_state); \
	MC_CMD_OP(cmd, 0, 32, 8, uint8_t, irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_GET_IRQ_ENABLE(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_GET_IRQ_ENABLE(cmd, enable_state) \
	MC_RSP_OP(cmd, 0, 0,  8,  uint8_t,  enable_state)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_SET_IRQ_MASK(cmd, irq_index, mask) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, mask); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_GET_IRQ_MASK(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_GET_IRQ_MASK(cmd, mask) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, mask)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_GET_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_GET_IRQ_STATUS(cmd, status) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, status)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 0,  16, uint16_t, attr->num_ifs);\
	MC_RSP_OP(cmd, 0, 16, 8,  uint8_t,  attr->max_fdbs);\
	MC_RSP_OP(cmd, 0, 24, 8,  uint8_t,  attr->num_fdbs);\
	MC_RSP_OP(cmd, 0, 32, 16, uint16_t, attr->max_vlans);\
	MC_RSP_OP(cmd, 0, 48, 16, uint16_t, attr->num_vlans);\
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, attr->version.major);\
	MC_RSP_OP(cmd, 1, 16, 16, uint16_t, attr->version.minor);\
	MC_RSP_OP(cmd, 1, 32, 16, uint16_t, attr->max_fdb_entries);\
	MC_RSP_OP(cmd, 1, 48, 16, uint16_t, attr->fdb_aging_time);\
	MC_RSP_OP(cmd, 2, 0,  32, int,	 attr->id);\
	MC_RSP_OP(cmd, 2, 32, 16, uint16_t, attr->mem_size);\
	MC_RSP_OP(cmd, 2, 48, 16, uint16_t, attr->max_fdb_mc_groups);\
	MC_RSP_OP(cmd, 3, 0,  64, uint64_t, attr->options);\
	MC_RSP_OP(cmd, 4, 0,  8,  uint8_t, attr->max_meters_per_if);\
	MC_RSP_OP(cmd, 4, 8,  4,  enum dpsw_component_type, \
			attr->component_type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_SET_REFLECTION_IF(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 1,  int,	 en);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_BROADCAST(cmd, if_id, en) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 1,  int,	 en);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_MULTICAST(cmd, if_id, en) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 1,  int,	 en);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_TCI(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 12, uint16_t, cfg->vlan_id);\
	MC_CMD_OP(cmd, 0, 28, 1,  uint8_t,  cfg->dei);\
	MC_CMD_OP(cmd, 0, 29, 3,  uint8_t,  cfg->pcp);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_GET_TCI(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IF_GET_TCI(cmd, cfg) \
do { \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, cfg->vlan_id);\
	MC_RSP_OP(cmd, 0, 32, 8,  uint8_t,  cfg->dei);\
	MC_RSP_OP(cmd, 0, 40, 8,  uint8_t,  cfg->pcp);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_STP(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->vlan_id);\
	MC_CMD_OP(cmd, 0, 32, 4,  enum dpsw_stp_state, cfg->state);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_ACCEPTED_FRAMES(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 4,  enum dpsw_accepted_frames, cfg->type);\
	MC_CMD_OP(cmd, 0, 20, 4,  enum dpsw_action, cfg->unaccept_act);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_ACCEPT_ALL_VLAN(cmd, if_id, accept_all) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 1,  int,	 accept_all);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_GET_COUNTER(cmd, if_id, type) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 5,  enum dpsw_counter, type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IF_GET_COUNTER(cmd, counter) \
	MC_RSP_OP(cmd, 1, 0, 64, uint64_t, counter)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_COUNTER(cmd, if_id, type, counter) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t,	  if_id);\
	MC_CMD_OP(cmd, 0, 16, 5,  enum dpsw_counter, type);\
	MC_CMD_OP(cmd, 1, 0,  64, uint64_t,	  counter);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_TX_SELECTION(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 3,  enum dpsw_priority_selector, \
						  cfg->priority_selector);\
	MC_CMD_OP(cmd, 1, 0,  8,  uint8_t,  cfg->tc_id[0]);\
	MC_CMD_OP(cmd, 1, 8,  8,  uint8_t,  cfg->tc_id[1]);\
	MC_CMD_OP(cmd, 1, 16, 8,  uint8_t,  cfg->tc_id[2]);\
	MC_CMD_OP(cmd, 1, 24, 8,  uint8_t,  cfg->tc_id[3]);\
	MC_CMD_OP(cmd, 1, 32, 8,  uint8_t,  cfg->tc_id[4]);\
	MC_CMD_OP(cmd, 1, 40, 8,  uint8_t,  cfg->tc_id[5]);\
	MC_CMD_OP(cmd, 1, 48, 8,  uint8_t,  cfg->tc_id[6]);\
	MC_CMD_OP(cmd, 1, 56, 8,  uint8_t,  cfg->tc_id[7]);\
	MC_CMD_OP(cmd, 2, 0,  16, uint16_t, cfg->tc_sched[0].delta_bandwidth);\
	MC_CMD_OP(cmd, 2, 16, 4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[0].mode);\
	MC_CMD_OP(cmd, 2, 32, 16, uint16_t, cfg->tc_sched[1].delta_bandwidth);\
	MC_CMD_OP(cmd, 2, 48, 4,  enum dpsw_schedule_mode, \
					    cfg->tc_sched[1].mode);\
	MC_CMD_OP(cmd, 3, 0,  16, uint16_t, cfg->tc_sched[2].delta_bandwidth);\
	MC_CMD_OP(cmd, 3, 16, 4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[2].mode);\
	MC_CMD_OP(cmd, 3, 32, 16, uint16_t, cfg->tc_sched[3].delta_bandwidth);\
	MC_CMD_OP(cmd, 3, 48, 4,  enum dpsw_schedule_mode, \
					    cfg->tc_sched[3].mode);\
	MC_CMD_OP(cmd, 4, 0,  16, uint16_t, cfg->tc_sched[4].delta_bandwidth);\
	MC_CMD_OP(cmd, 4, 16,  4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[4].mode);\
	MC_CMD_OP(cmd, 4, 32, 16, uint16_t, cfg->tc_sched[5].delta_bandwidth);\
	MC_CMD_OP(cmd, 4, 48, 4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[5].mode);\
	MC_CMD_OP(cmd, 5, 0,  16, uint16_t, cfg->tc_sched[6].delta_bandwidth);\
	MC_CMD_OP(cmd, 5, 16, 4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[6].mode);\
	MC_CMD_OP(cmd, 5, 32, 16, uint16_t, cfg->tc_sched[7].delta_bandwidth);\
	MC_CMD_OP(cmd, 5, 48, 4,  enum dpsw_schedule_mode,  \
					    cfg->tc_sched[7].mode);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_ADD_REFLECTION(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->vlan_id);\
	MC_CMD_OP(cmd, 0, 32, 2,  enum dpsw_reflection_filter, cfg->filter);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_REMOVE_REFLECTION(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->vlan_id);\
	MC_CMD_OP(cmd, 0, 32, 2,  enum dpsw_reflection_filter, cfg->filter);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_FLOODING_METERING(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 24, 4,  enum dpsw_metering_mode, cfg->mode);\
	MC_CMD_OP(cmd, 0, 28, 4,  enum dpsw_metering_unit, cfg->units);\
	MC_CMD_OP(cmd, 0, 32, 32, uint32_t, cfg->cir);\
	MC_CMD_OP(cmd, 1, 0,  32, uint32_t, cfg->eir);\
	MC_CMD_OP(cmd, 1, 32, 32, uint32_t, cfg->cbs);\
	MC_CMD_OP(cmd, 2, 0,  32, uint32_t, cfg->ebs);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_METERING(cmd, if_id, tc_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  tc_id);\
	MC_CMD_OP(cmd, 0, 24, 4,  enum dpsw_metering_mode, cfg->mode);\
	MC_CMD_OP(cmd, 0, 28, 4,  enum dpsw_metering_unit, cfg->units);\
	MC_CMD_OP(cmd, 0, 32, 32, uint32_t, cfg->cir);\
	MC_CMD_OP(cmd, 1, 0,  32, uint32_t, cfg->eir);\
	MC_CMD_OP(cmd, 1, 32, 32, uint32_t, cfg->cbs);\
	MC_CMD_OP(cmd, 2, 0,  32, uint32_t, cfg->ebs);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_PREP_EARLY_DROP(ext, cfg) \
do { \
	MC_PREP_OP(ext, 0, 0,  2, enum dpsw_early_drop_mode, cfg->drop_mode); \
	MC_PREP_OP(ext, 0, 2,  2, \
		  enum dpsw_early_drop_unit, cfg->units); \
	MC_PREP_OP(ext, 0, 32, 32, uint32_t, cfg->tail_drop_threshold); \
	MC_PREP_OP(ext, 1, 0,  8,  uint8_t,  cfg->green.drop_probability); \
	MC_PREP_OP(ext, 2, 0,  64, uint64_t, cfg->green.max_threshold); \
	MC_PREP_OP(ext, 3, 0,  64, uint64_t, cfg->green.min_threshold); \
	MC_PREP_OP(ext, 5, 0,  8,  uint8_t,  cfg->yellow.drop_probability);\
	MC_PREP_OP(ext, 6, 0,  64, uint64_t, cfg->yellow.max_threshold); \
	MC_PREP_OP(ext, 7, 0,  64, uint64_t, cfg->yellow.min_threshold); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_EXT_EARLY_DROP(ext, cfg) \
do { \
	MC_EXT_OP(ext, 0, 0,  2, enum dpsw_early_drop_mode, cfg->drop_mode); \
	MC_EXT_OP(ext, 0, 2,  2, \
		  enum dpsw_early_drop_unit, cfg->units); \
	MC_EXT_OP(ext, 0, 32, 32, uint32_t, cfg->tail_drop_threshold); \
	MC_EXT_OP(ext, 1, 0,  8,  uint8_t,  cfg->green.drop_probability); \
	MC_EXT_OP(ext, 2, 0,  64, uint64_t, cfg->green.max_threshold); \
	MC_EXT_OP(ext, 3, 0,  64, uint64_t, cfg->green.min_threshold); \
	MC_EXT_OP(ext, 5, 0,  8,  uint8_t,  cfg->yellow.drop_probability);\
	MC_EXT_OP(ext, 6, 0,  64, uint64_t, cfg->yellow.max_threshold); \
	MC_EXT_OP(ext, 7, 0,  64, uint64_t, cfg->yellow.min_threshold); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_EARLY_DROP(cmd, if_id, tc_id, early_drop_iova) \
do { \
	MC_CMD_OP(cmd, 0, 8,  8,  uint8_t,  tc_id); \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, if_id); \
	MC_CMD_OP(cmd, 1, 0,  64, uint64_t, early_drop_iova); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ADD_CUSTOM_TPID(cmd, cfg) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->tpid)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_REMOVE_CUSTOM_TPID(cmd, cfg) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->tpid)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_ENABLE(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_DISABLE(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_GET_ATTR(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IF_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 0,  4,  enum dpsw_accepted_frames, \
							attr->admit_untagged);\
	MC_RSP_OP(cmd, 0, 5,  1,  int,      attr->enabled);\
	MC_RSP_OP(cmd, 0, 6,  1,  int,      attr->accept_all_vlan);\
	MC_RSP_OP(cmd, 0, 16, 8,  uint8_t,  attr->num_tcs);\
	MC_RSP_OP(cmd, 0, 32, 16, uint16_t,	attr->qdid);\
	MC_RSP_OP(cmd, 1, 0,  32, uint32_t, attr->options);\
	MC_RSP_OP(cmd, 2, 0,  32, uint32_t, attr->rate);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_MAX_FRAME_LENGTH(cmd, if_id, frame_length) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, frame_length);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_GET_MAX_FRAME_LENGTH(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IF_GET_MAX_FRAME_LENGTH(cmd, frame_length) \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, frame_length)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_SET_LINK_CFG(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 1, 0,  32, uint32_t, cfg->rate);\
	MC_CMD_OP(cmd, 2, 0,  64, uint64_t, cfg->options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_IF_GET_LINK_STATE(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_IF_GET_LINK_STATE(cmd, state) \
do { \
	MC_RSP_OP(cmd, 0, 32, 1,  int,      state->up);\
	MC_RSP_OP(cmd, 1, 0,  32, uint32_t, state->rate);\
	MC_RSP_OP(cmd, 2, 0,  64, uint64_t, state->options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_ADD(cmd, vlan_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, cfg->fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_ADD_IF(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_ADD_IF_UNTAGGED(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_ADD_IF_FLOODING(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

#define DPSW_CMD_VLAN_REMOVE_IF(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_REMOVE_IF_UNTAGGED(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_REMOVE_IF_FLOODING(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_REMOVE(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_GET_ATTR(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_VLAN_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, attr->fdb_id); \
	MC_RSP_OP(cmd, 1, 16, 16, uint16_t, attr->num_ifs); \
	MC_RSP_OP(cmd, 1, 32, 16, uint16_t, attr->num_untagged_ifs); \
	MC_RSP_OP(cmd, 1, 48, 16, uint16_t, attr->num_flooding_ifs); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_GET_IF(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_VLAN_GET_IF(cmd, cfg) \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_GET_IF_FLOODING(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_VLAN_GET_IF_FLOODING(cmd, cfg) \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_VLAN_GET_IF_UNTAGGED(cmd, vlan_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, vlan_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_VLAN_GET_IF_UNTAGGED(cmd, cfg) \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs)

/*	param, offset, width,	type,		arg_name */
#define DPSW_CMD_FDB_ADD(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 32, 16, uint16_t, cfg->fdb_aging_time);\
	MC_CMD_OP(cmd, 0, 48, 16, uint16_t, cfg->num_fdb_entries);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_FDB_ADD(cmd, fdb_id) \
	MC_RSP_OP(cmd, 0, 0,  16, uint16_t, fdb_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_REMOVE(cmd, fdb_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_ADD_UNICAST(cmd, fdb_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  cfg->mac_addr[0]);\
	MC_CMD_OP(cmd, 1, 0,  8,  uint16_t, cfg->if_egress);\
	MC_CMD_OP(cmd, 1, 16, 4,  enum dpsw_fdb_entry_type, cfg->type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_GET_UNICAST(cmd, fdb_id) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  cfg->mac_addr[0]);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_FDB_GET_UNICAST(cmd, cfg) \
do { \
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, cfg->if_egress);\
	MC_RSP_OP(cmd, 1, 16, 4,  enum dpsw_fdb_entry_type, cfg->type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_REMOVE_UNICAST(cmd, fdb_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  cfg->mac_addr[0]);\
	MC_CMD_OP(cmd, 1, 0,  16, uint16_t, cfg->if_egress);\
	MC_CMD_OP(cmd, 1, 16, 4,  enum dpsw_fdb_entry_type, cfg->type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_ADD_MULTICAST(cmd, fdb_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs);\
	MC_CMD_OP(cmd, 0, 32, 4,  enum dpsw_fdb_entry_type, cfg->type);\
	MC_CMD_OP(cmd, 1, 0,  8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 1, 8,  8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 1, 16, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 1, 24, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 1, 32, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 1, 40, 8,  uint8_t,  cfg->mac_addr[0]);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_GET_MULTICAST(cmd, fdb_id) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  cfg->mac_addr[0]);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_FDB_GET_MULTICAST(cmd, cfg) \
do { \
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, cfg->num_ifs);\
	MC_RSP_OP(cmd, 1, 16, 4,  enum dpsw_fdb_entry_type, cfg->type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_REMOVE_MULTICAST(cmd, fdb_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs);\
	MC_CMD_OP(cmd, 0, 32, 4,  enum dpsw_fdb_entry_type, cfg->type);\
	MC_CMD_OP(cmd, 1, 0,  8,  uint8_t,  cfg->mac_addr[5]);\
	MC_CMD_OP(cmd, 1, 8,  8,  uint8_t,  cfg->mac_addr[4]);\
	MC_CMD_OP(cmd, 1, 16, 8,  uint8_t,  cfg->mac_addr[3]);\
	MC_CMD_OP(cmd, 1, 24, 8,  uint8_t,  cfg->mac_addr[2]);\
	MC_CMD_OP(cmd, 1, 32, 8,  uint8_t,  cfg->mac_addr[1]);\
	MC_CMD_OP(cmd, 1, 40, 8,  uint8_t,  cfg->mac_addr[0]);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_SET_LEARNING_MODE(cmd, fdb_id, mode) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id);\
	MC_CMD_OP(cmd, 0, 16, 4,  enum dpsw_fdb_learning_mode, mode);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_FDB_GET_ATTR(cmd, fdb_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, fdb_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_FDB_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, attr->max_fdb_entries);\
	MC_RSP_OP(cmd, 0, 32, 16, uint16_t, attr->fdb_aging_time);\
	MC_RSP_OP(cmd, 0, 48, 16, uint16_t, attr->num_fdb_mc_groups);\
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, attr->max_fdb_mc_groups);\
	MC_RSP_OP(cmd, 1, 16, 4,  enum dpsw_fdb_learning_mode, \
							  attr->learning_mode);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_ADD(cmd, cfg) \
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->max_entries)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_ACL_ADD(cmd, acl_id) \
	MC_RSP_OP(cmd, 0, 0,  16, uint16_t, acl_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_REMOVE(cmd, acl_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_PREP_ACL_ENTRY(ext, key) \
do { \
	MC_PREP_OP(ext, 0, 0,  8,  uint8_t,  key->match.l2_dest_mac[5]);\
	MC_PREP_OP(ext, 0, 8,  8,  uint8_t,  key->match.l2_dest_mac[4]);\
	MC_PREP_OP(ext, 0, 16, 8,  uint8_t,  key->match.l2_dest_mac[3]);\
	MC_PREP_OP(ext, 0, 24, 8,  uint8_t,  key->match.l2_dest_mac[2]);\
	MC_PREP_OP(ext, 0, 32, 8,  uint8_t,  key->match.l2_dest_mac[1]);\
	MC_PREP_OP(ext, 0, 40, 8,  uint8_t,  key->match.l2_dest_mac[0]);\
	MC_PREP_OP(ext, 0, 48, 16, uint16_t, key->match.l2_tpid);\
	MC_PREP_OP(ext, 1, 0,  8,  uint8_t,  key->match.l2_source_mac[5]);\
	MC_PREP_OP(ext, 1, 8,  8,  uint8_t,  key->match.l2_source_mac[4]);\
	MC_PREP_OP(ext, 1, 16, 8,  uint8_t,  key->match.l2_source_mac[3]);\
	MC_PREP_OP(ext, 1, 24, 8,  uint8_t,  key->match.l2_source_mac[2]);\
	MC_PREP_OP(ext, 1, 32, 8,  uint8_t,  key->match.l2_source_mac[1]);\
	MC_PREP_OP(ext, 1, 40, 8,  uint8_t,  key->match.l2_source_mac[0]);\
	MC_PREP_OP(ext, 1, 48, 16, uint16_t, key->match.l2_vlan_id);\
	MC_PREP_OP(ext, 2, 0,  32, uint32_t, key->match.l3_dest_ip);\
	MC_PREP_OP(ext, 2, 32, 32, uint32_t, key->match.l3_source_ip);\
	MC_PREP_OP(ext, 3, 0,  16, uint16_t, key->match.l4_dest_port);\
	MC_PREP_OP(ext, 3, 16, 16, uint16_t, key->match.l4_source_port);\
	MC_PREP_OP(ext, 3, 32, 16, uint16_t, key->match.l2_ether_type);\
	MC_PREP_OP(ext, 3, 48, 8,  uint8_t,  key->match.l2_pcp_dei);\
	MC_PREP_OP(ext, 3, 56, 8,  uint8_t,  key->match.l3_dscp);\
	MC_PREP_OP(ext, 4, 0,  8,  uint8_t,  key->mask.l2_dest_mac[5]);\
	MC_PREP_OP(ext, 4, 8,  8,  uint8_t,  key->mask.l2_dest_mac[4]);\
	MC_PREP_OP(ext, 4, 16, 8,  uint8_t,  key->mask.l2_dest_mac[3]);\
	MC_PREP_OP(ext, 4, 24, 8,  uint8_t,  key->mask.l2_dest_mac[2]);\
	MC_PREP_OP(ext, 4, 32, 8,  uint8_t,  key->mask.l2_dest_mac[1]);\
	MC_PREP_OP(ext, 4, 40, 8,  uint8_t,  key->mask.l2_dest_mac[0]);\
	MC_PREP_OP(ext, 4, 48, 16, uint16_t, key->mask.l2_tpid);\
	MC_PREP_OP(ext, 5, 0,  8,  uint8_t,  key->mask.l2_source_mac[5]);\
	MC_PREP_OP(ext, 5, 8,  8,  uint8_t,  key->mask.l2_source_mac[4]);\
	MC_PREP_OP(ext, 5, 16, 8,  uint8_t,  key->mask.l2_source_mac[3]);\
	MC_PREP_OP(ext, 5, 24, 8,  uint8_t,  key->mask.l2_source_mac[2]);\
	MC_PREP_OP(ext, 5, 32, 8,  uint8_t,  key->mask.l2_source_mac[1]);\
	MC_PREP_OP(ext, 5, 40, 8,  uint8_t,  key->mask.l2_source_mac[0]);\
	MC_PREP_OP(ext, 5, 48, 16, uint16_t, key->mask.l2_vlan_id);\
	MC_PREP_OP(ext, 6, 0,  32, uint32_t, key->mask.l3_dest_ip);\
	MC_PREP_OP(ext, 6, 32, 32, uint32_t, key->mask.l3_source_ip);\
	MC_PREP_OP(ext, 7, 0,  16, uint16_t, key->mask.l4_dest_port);\
	MC_PREP_OP(ext, 7, 16, 16, uint16_t, key->mask.l4_source_port);\
	MC_PREP_OP(ext, 7, 32, 16, uint16_t, key->mask.l2_ether_type);\
	MC_PREP_OP(ext, 7, 48, 8,  uint8_t,  key->mask.l2_pcp_dei);\
	MC_PREP_OP(ext, 7, 56, 8,  uint8_t,  key->mask.l3_dscp);\
	MC_PREP_OP(ext, 8, 0,  8,  uint8_t,  key->match.l3_protocol);\
	MC_PREP_OP(ext, 8, 8,  8,  uint8_t,  key->mask.l3_protocol);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_EXT_ACL_ENTRY(ext, key) \
do { \
	MC_EXT_OP(ext, 0, 0,  8,  uint8_t,  key->match.l2_dest_mac[5]);\
	MC_EXT_OP(ext, 0, 8,  8,  uint8_t,  key->match.l2_dest_mac[4]);\
	MC_EXT_OP(ext, 0, 16, 8,  uint8_t,  key->match.l2_dest_mac[3]);\
	MC_EXT_OP(ext, 0, 24, 8,  uint8_t,  key->match.l2_dest_mac[2]);\
	MC_EXT_OP(ext, 0, 32, 8,  uint8_t,  key->match.l2_dest_mac[1]);\
	MC_EXT_OP(ext, 0, 40, 8,  uint8_t,  key->match.l2_dest_mac[0]);\
	MC_EXT_OP(ext, 0, 48, 16, uint16_t, key->match.l2_tpid);\
	MC_EXT_OP(ext, 1, 0,  8,  uint8_t,  key->match.l2_source_mac[5]);\
	MC_EXT_OP(ext, 1, 8,  8,  uint8_t,  key->match.l2_source_mac[4]);\
	MC_EXT_OP(ext, 1, 16, 8,  uint8_t,  key->match.l2_source_mac[3]);\
	MC_EXT_OP(ext, 1, 24, 8,  uint8_t,  key->match.l2_source_mac[2]);\
	MC_EXT_OP(ext, 1, 32, 8,  uint8_t,  key->match.l2_source_mac[1]);\
	MC_EXT_OP(ext, 1, 40, 8,  uint8_t,  key->match.l2_source_mac[0]);\
	MC_EXT_OP(ext, 1, 48, 16, uint16_t, key->match.l2_vlan_id);\
	MC_EXT_OP(ext, 2, 0,  32, uint32_t, key->match.l3_dest_ip);\
	MC_EXT_OP(ext, 2, 32, 32, uint32_t, key->match.l3_source_ip);\
	MC_EXT_OP(ext, 3, 0,  16, uint16_t, key->match.l4_dest_port);\
	MC_EXT_OP(ext, 3, 16, 16, uint16_t, key->match.l4_source_port);\
	MC_EXT_OP(ext, 3, 32, 16, uint16_t, key->match.l2_ether_type);\
	MC_EXT_OP(ext, 3, 48, 8,  uint8_t,  key->match.l2_pcp_dei);\
	MC_EXT_OP(ext, 3, 56, 8,  uint8_t,  key->match.l3_dscp);\
	MC_EXT_OP(ext, 4, 0,  8,  uint8_t,  key->mask.l2_dest_mac[5]);\
	MC_EXT_OP(ext, 4, 8,  8,  uint8_t,  key->mask.l2_dest_mac[4]);\
	MC_EXT_OP(ext, 4, 16, 8,  uint8_t,  key->mask.l2_dest_mac[3]);\
	MC_EXT_OP(ext, 4, 24, 8,  uint8_t,  key->mask.l2_dest_mac[2]);\
	MC_EXT_OP(ext, 4, 32, 8,  uint8_t,  key->mask.l2_dest_mac[1]);\
	MC_EXT_OP(ext, 4, 40, 8,  uint8_t,  key->mask.l2_dest_mac[0]);\
	MC_EXT_OP(ext, 4, 48, 16, uint16_t, key->mask.l2_tpid);\
	MC_EXT_OP(ext, 5, 0,  8,  uint8_t,  key->mask.l2_source_mac[5]);\
	MC_EXT_OP(ext, 5, 8,  8,  uint8_t,  key->mask.l2_source_mac[4]);\
	MC_EXT_OP(ext, 5, 16, 8,  uint8_t,  key->mask.l2_source_mac[3]);\
	MC_EXT_OP(ext, 5, 24, 8,  uint8_t,  key->mask.l2_source_mac[2]);\
	MC_EXT_OP(ext, 5, 32, 8,  uint8_t,  key->mask.l2_source_mac[1]);\
	MC_EXT_OP(ext, 5, 40, 8,  uint8_t,  key->mask.l2_source_mac[0]);\
	MC_EXT_OP(ext, 5, 48, 16, uint16_t, key->mask.l2_vlan_id);\
	MC_EXT_OP(ext, 6, 0,  32, uint32_t, key->mask.l3_dest_ip);\
	MC_EXT_OP(ext, 6, 32, 32, uint32_t, key->mask.l3_source_ip);\
	MC_EXT_OP(ext, 7, 0,  16, uint16_t, key->mask.l4_dest_port);\
	MC_EXT_OP(ext, 7, 16, 16, uint16_t, key->mask.l4_source_port);\
	MC_EXT_OP(ext, 7, 32, 16, uint16_t, key->mask.l2_ether_type);\
	MC_EXT_OP(ext, 7, 48, 8,  uint8_t,  key->mask.l2_pcp_dei);\
	MC_EXT_OP(ext, 7, 56, 8,  uint8_t,  key->mask.l3_dscp);\
	MC_EXT_OP(ext, 8, 0,  8,  uint8_t,  key->match.l3_protocol);\
	MC_EXT_OP(ext, 8, 8,  8,  uint8_t,  key->mask.l3_protocol);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_ADD_ENTRY(cmd, acl_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->result.if_id);\
	MC_CMD_OP(cmd, 0, 32, 32, int,      cfg->precedence);\
	MC_CMD_OP(cmd, 1, 0,  4,  enum dpsw_acl_action, cfg->result.action);\
	MC_CMD_OP(cmd, 6, 0,  64, uint64_t, cfg->key_iova); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_REMOVE_ENTRY(cmd, acl_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->result.if_id);\
	MC_CMD_OP(cmd, 0, 32, 32, int,      cfg->precedence);\
	MC_CMD_OP(cmd, 1, 0,  4,  enum dpsw_acl_action, cfg->result.action);\
	MC_CMD_OP(cmd, 6, 0,  64, uint64_t, cfg->key_iova); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_ADD_IF(cmd, acl_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_REMOVE_IF(cmd, acl_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_ACL_GET_ATTR(cmd, acl_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, acl_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_ACL_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 1, 0,  16, uint16_t, attr->max_entries);\
	MC_RSP_OP(cmd, 1, 16, 16, uint16_t, attr->num_entries);\
	MC_RSP_OP(cmd, 1, 32, 16, uint16_t, attr->num_ifs);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_RSP_CTRL_IF_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 1, 0,  32, uint32_t, attr->rx_fqid);\
	MC_RSP_OP(cmd, 1, 32, 32, uint32_t, attr->rx_err_fqid);\
	MC_RSP_OP(cmd, 2, 0,  32, uint32_t, attr->tx_err_conf_fqid);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPSW_CMD_CTRL_IF_SET_POOLS(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  cfg->num_dpbp); \
	MC_CMD_OP(cmd, 0, 8,  1,  int,      cfg->pools[0].backup_pool); \
	MC_CMD_OP(cmd, 0, 9,  1,  int,      cfg->pools[1].backup_pool); \
	MC_CMD_OP(cmd, 0, 10, 1,  int,      cfg->pools[2].backup_pool); \
	MC_CMD_OP(cmd, 0, 11, 1,  int,      cfg->pools[3].backup_pool); \
	MC_CMD_OP(cmd, 0, 12, 1,  int,      cfg->pools[4].backup_pool); \
	MC_CMD_OP(cmd, 0, 13, 1,  int,      cfg->pools[5].backup_pool); \
	MC_CMD_OP(cmd, 0, 14, 1,  int,      cfg->pools[6].backup_pool); \
	MC_CMD_OP(cmd, 0, 15, 1,  int,      cfg->pools[7].backup_pool); \
	MC_CMD_OP(cmd, 0, 32, 32, int,      cfg->pools[0].dpbp_id); \
	MC_CMD_OP(cmd, 4, 32, 16, uint16_t, cfg->pools[0].buffer_size);\
	MC_CMD_OP(cmd, 1, 0,  32, int,      cfg->pools[1].dpbp_id); \
	MC_CMD_OP(cmd, 4, 48, 16, uint16_t, cfg->pools[1].buffer_size);\
	MC_CMD_OP(cmd, 1, 32, 32, int,      cfg->pools[2].dpbp_id); \
	MC_CMD_OP(cmd, 5, 0,  16, uint16_t, cfg->pools[2].buffer_size);\
	MC_CMD_OP(cmd, 2, 0,  32, int,      cfg->pools[3].dpbp_id); \
	MC_CMD_OP(cmd, 5, 16, 16, uint16_t, cfg->pools[3].buffer_size);\
	MC_CMD_OP(cmd, 2, 32, 32, int,      cfg->pools[4].dpbp_id); \
	MC_CMD_OP(cmd, 5, 32, 16, uint16_t, cfg->pools[4].buffer_size);\
	MC_CMD_OP(cmd, 3, 0,  32, int,      cfg->pools[5].dpbp_id); \
	MC_CMD_OP(cmd, 5, 48, 16, uint16_t, cfg->pools[5].buffer_size);\
	MC_CMD_OP(cmd, 3, 32, 32, int,      cfg->pools[6].dpbp_id); \
	MC_CMD_OP(cmd, 6, 0,  16, uint16_t, cfg->pools[6].buffer_size);\
	MC_CMD_OP(cmd, 4, 0,  32, int,      cfg->pools[7].dpbp_id); \
	MC_CMD_OP(cmd, 6, 16, 16, uint16_t, cfg->pools[7].buffer_size);\
} while (0)

#endif /* __FSL_DPSW_CMD_H */
