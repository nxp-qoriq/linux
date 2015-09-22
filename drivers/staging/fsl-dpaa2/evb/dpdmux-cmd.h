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
#ifndef _FSL_DPDMUX_CMD_H
#define _FSL_DPDMUX_CMD_H

/* DPDMUX Version */
#define DPDMUX_VER_MAJOR				5
#define DPDMUX_VER_MINOR				0

/* Command IDs */
#define DPDMUX_CMDID_CLOSE				0x800
#define DPDMUX_CMDID_OPEN				0x806
#define DPDMUX_CMDID_CREATE				0x906
#define DPDMUX_CMDID_DESTROY				0x900

#define DPDMUX_CMDID_ENABLE				0x002
#define DPDMUX_CMDID_DISABLE				0x003
#define DPDMUX_CMDID_GET_ATTR				0x004
#define DPDMUX_CMDID_RESET				0x005
#define DPDMUX_CMDID_IS_ENABLED				0x006

#define DPDMUX_CMDID_SET_IRQ				0x010
#define DPDMUX_CMDID_GET_IRQ				0x011
#define DPDMUX_CMDID_SET_IRQ_ENABLE			0x012
#define DPDMUX_CMDID_GET_IRQ_ENABLE			0x013
#define DPDMUX_CMDID_SET_IRQ_MASK			0x014
#define DPDMUX_CMDID_GET_IRQ_MASK			0x015
#define DPDMUX_CMDID_GET_IRQ_STATUS			0x016
#define DPDMUX_CMDID_CLEAR_IRQ_STATUS			0x017

#define DPDMUX_CMDID_UL_SET_MAX_FRAME_LENGTH		0x0a1

#define DPDMUX_CMDID_UL_RESET_COUNTERS			0x0a3

#define DPDMUX_CMDID_IF_SET_ACCEPTED_FRAMES		0x0a7
#define DPDMUX_CMDID_IF_GET_ATTR			0x0a8

#define DPDMUX_CMDID_IF_ADD_L2_RULE			0x0b0
#define DPDMUX_CMDID_IF_REMOVE_L2_RULE			0x0b1
#define DPDMUX_CMDID_IF_GET_COUNTER			0x0b2
#define DPDMUX_CMDID_IF_SET_LINK_CFG		0x0b3
#define DPDMUX_CMDID_IF_GET_LINK_STATE		0x0b4

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_OPEN(cmd, dpdmux_id) \
	MC_CMD_OP(cmd, 0, 0,  32,  int,	dpdmux_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_CREATE(cmd, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  enum dpdmux_method, cfg->method);\
	MC_CMD_OP(cmd, 0, 8,  8,  enum dpdmux_manip, cfg->manip);\
	MC_CMD_OP(cmd, 0, 16, 16, uint16_t, cfg->num_ifs);\
	MC_CMD_OP(cmd, 1, 0,  16, uint16_t, cfg->adv.max_dmat_entries);\
	MC_CMD_OP(cmd, 1, 16, 16, uint16_t, cfg->adv.max_mc_groups);\
	MC_CMD_OP(cmd, 1, 32, 16, uint16_t, cfg->adv.max_vlan_ids);\
	MC_CMD_OP(cmd, 2, 0,  64, uint64_t, cfg->adv.options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_IS_ENABLED(cmd, en) \
	MC_RSP_OP(cmd, 0, 0,  1,  int,	    en)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_SET_IRQ(cmd, irq_index, irq_cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  irq_index);\
	MC_CMD_OP(cmd, 0, 32, 32, uint32_t, irq_cfg->val);\
	MC_CMD_OP(cmd, 1, 0,  64, uint64_t, irq_cfg->addr);\
	MC_CMD_OP(cmd, 2, 0,  32, int,	    irq_cfg->irq_num); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_GET_IRQ(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_GET_IRQ(cmd, type, irq_cfg) \
do { \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, irq_cfg->val); \
	MC_RSP_OP(cmd, 1, 0,  64, uint64_t, irq_cfg->addr); \
	MC_RSP_OP(cmd, 2, 0,  32, int,	    irq_cfg->irq_num); \
	MC_RSP_OP(cmd, 2, 32, 32, int,	    type); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_SET_IRQ_ENABLE(cmd, irq_index, en) \
do { \
	MC_CMD_OP(cmd, 0, 0,  8,  uint8_t,  en);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_GET_IRQ_ENABLE(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_GET_IRQ_ENABLE(cmd, en) \
	MC_RSP_OP(cmd, 0, 0,  8,  uint8_t,  en)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_SET_IRQ_MASK(cmd, irq_index, mask) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, mask); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index); \
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_GET_IRQ_MASK(cmd, irq_index) \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_GET_IRQ_MASK(cmd, mask) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, mask)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_GET_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_GET_IRQ_STATUS(cmd, status) \
	MC_RSP_OP(cmd, 0, 0,  32, uint32_t, status) \

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status) \
do { \
	MC_CMD_OP(cmd, 0, 0,  32, uint32_t, status); \
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  irq_index); \
} while (0)

#define DPDMUX_RSP_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 0,  8,  enum dpdmux_method, attr->method);\
	MC_RSP_OP(cmd, 0, 8,  8,  enum dpdmux_manip, attr->manip);\
	MC_RSP_OP(cmd, 0, 16, 16, uint16_t, attr->num_ifs);\
	MC_RSP_OP(cmd, 0, 32, 16, uint16_t, attr->mem_size);\
	MC_RSP_OP(cmd, 2, 0,  32, int,	    attr->id);\
	MC_RSP_OP(cmd, 3, 0,  64, uint64_t, attr->options);\
	MC_RSP_OP(cmd, 4, 0,  16, uint16_t, attr->version.major);\
	MC_RSP_OP(cmd, 4, 16, 16, uint16_t, attr->version.minor);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_UL_SET_MAX_FRAME_LENGTH(cmd, max_frame_length) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, max_frame_length)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_IF_SET_ACCEPTED_FRAMES(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 4,  enum dpdmux_accepted_frames_type, cfg->type);\
	MC_CMD_OP(cmd, 0, 20, 4,  enum dpdmux_unaccepted_frames_action, \
					    cfg->unaccept_act);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_IF_GET_ATTR(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_IF_GET_ATTR(cmd, attr) \
do { \
	MC_RSP_OP(cmd, 0, 56, 4,  enum dpdmux_accepted_frames_type, \
					    attr->accept_frame_type);\
	MC_RSP_OP(cmd, 0, 24,  1, int,	    attr->enabled);\
	MC_RSP_OP(cmd, 1, 0,  32, uint32_t, attr->rate);\
} while (0)

#define DPDMUX_CMD_IF_REMOVE_L2_RULE(cmd, if_id, l2_rule) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  l2_rule->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  l2_rule->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  l2_rule->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  l2_rule->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  l2_rule->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  l2_rule->mac_addr[0]);\
	MC_CMD_OP(cmd, 1, 32, 16, uint16_t, l2_rule->vlan_id);\
} while (0)

#define DPDMUX_CMD_IF_ADD_L2_RULE(cmd, if_id, l2_rule) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  uint8_t,  l2_rule->mac_addr[5]);\
	MC_CMD_OP(cmd, 0, 24, 8,  uint8_t,  l2_rule->mac_addr[4]);\
	MC_CMD_OP(cmd, 0, 32, 8,  uint8_t,  l2_rule->mac_addr[3]);\
	MC_CMD_OP(cmd, 0, 40, 8,  uint8_t,  l2_rule->mac_addr[2]);\
	MC_CMD_OP(cmd, 0, 48, 8,  uint8_t,  l2_rule->mac_addr[1]);\
	MC_CMD_OP(cmd, 0, 56, 8,  uint8_t,  l2_rule->mac_addr[0]);\
	MC_CMD_OP(cmd, 1, 32, 16, uint16_t, l2_rule->vlan_id);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_IF_GET_COUNTER(cmd, if_id, counter_type) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 0, 16, 8,  enum dpdmux_counter_type, counter_type);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_IF_GET_COUNTER(cmd, counter) \
	MC_RSP_OP(cmd, 1, 0,  64, uint64_t, counter)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_IF_SET_LINK_CFG(cmd, if_id, cfg) \
do { \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id);\
	MC_CMD_OP(cmd, 1, 0,  32, uint32_t, cfg->rate);\
	MC_CMD_OP(cmd, 2, 0,  64, uint64_t, cfg->options);\
} while (0)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_CMD_IF_GET_LINK_STATE(cmd, if_id) \
	MC_CMD_OP(cmd, 0, 0,  16, uint16_t, if_id)

/*                cmd, param, offset, width, type, arg_name */
#define DPDMUX_RSP_IF_GET_LINK_STATE(cmd, state) \
do { \
	MC_RSP_OP(cmd, 0, 32, 1,  int,      state->up);\
	MC_RSP_OP(cmd, 1, 0,  32, uint32_t, state->rate);\
	MC_RSP_OP(cmd, 2, 0,  64, uint64_t, state->options);\
} while (0)

#endif /* _FSL_DPDMUX_CMD_H */
