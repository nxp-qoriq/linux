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
#ifndef _FSL_DPRTC_CMD_H
#define _FSL_DPRTC_CMD_H

/* DPRTC Version */
#define DPRTC_VER_MAJOR			2
#define DPRTC_VER_MINOR			0

/* Command versioning */
#define DPRTC_CMD_BASE_VERSION		1
#define DPRTC_CMD_ID_OFFSET		4

#define DPRTC_CMD(id)	(((id) << DPRTC_CMD_ID_OFFSET) | DPRTC_CMD_BASE_VERSION)

/* Command IDs */
#define DPRTC_CMDID_CLOSE			DPRTC_CMD(0x800)
#define DPRTC_CMDID_OPEN			DPRTC_CMD(0x810)
#define DPRTC_CMDID_CREATE			DPRTC_CMD(0x910)
#define DPRTC_CMDID_DESTROY			DPRTC_CMD(0x990)
#define DPRTC_CMDID_GET_API_VERSION		DPRTC_CMD(0xa10)

#define DPRTC_CMDID_ENABLE			DPRTC_CMD(0x002)
#define DPRTC_CMDID_DISABLE			DPRTC_CMD(0x003)
#define DPRTC_CMDID_GET_ATTR			DPRTC_CMD(0x004)
#define DPRTC_CMDID_RESET			DPRTC_CMD(0x005)
#define DPRTC_CMDID_IS_ENABLED			DPRTC_CMD(0x006)

#define DPRTC_CMDID_SET_IRQ_ENABLE		DPRTC_CMD(0x012)
#define DPRTC_CMDID_GET_IRQ_ENABLE		DPRTC_CMD(0x013)
#define DPRTC_CMDID_SET_IRQ_MASK		DPRTC_CMD(0x014)
#define DPRTC_CMDID_GET_IRQ_MASK		DPRTC_CMD(0x015)
#define DPRTC_CMDID_GET_IRQ_STATUS		DPRTC_CMD(0x016)
#define DPRTC_CMDID_CLEAR_IRQ_STATUS		DPRTC_CMD(0x017)

#define DPRTC_CMDID_SET_CLOCK_OFFSET		DPRTC_CMD(0x1d0)
#define DPRTC_CMDID_SET_FREQ_COMPENSATION	DPRTC_CMD(0x1d1)
#define DPRTC_CMDID_GET_FREQ_COMPENSATION	DPRTC_CMD(0x1d2)
#define DPRTC_CMDID_GET_TIME			DPRTC_CMD(0x1d3)
#define DPRTC_CMDID_SET_TIME			DPRTC_CMD(0x1d4)
#define DPRTC_CMDID_SET_ALARM			DPRTC_CMD(0x1d5)
#define DPRTC_CMDID_SET_PERIODIC_PULSE		DPRTC_CMD(0x1d6)
#define DPRTC_CMDID_CLEAR_PERIODIC_PULSE	DPRTC_CMD(0x1d7)
#define DPRTC_CMDID_SET_EXT_TRIGGER		DPRTC_CMD(0x1d8)
#define DPRTC_CMDID_CLEAR_EXT_TRIGGER		DPRTC_CMD(0x1d9)
#define DPRTC_CMDID_GET_EXT_TRIGGER_TIMESTAMP	DPRTC_CMD(0x1dA)

/* Macros for accessing command fields smaller than 1byte */
#define DPRTC_MASK(field)        \
	GENMASK(DPRTC_##field##_SHIFT + DPRTC_##field##_SIZE - 1, \
		DPRTC_##field##_SHIFT)
#define dprtc_get_field(var, field)      \
	(((var) & DPRTC_MASK(field)) >> DPRTC_##field##_SHIFT)

#pragma pack(push, 1)
struct dprtc_cmd_open {
	uint32_t dprtc_id;
};

struct dprtc_cmd_destroy {
	uint32_t object_id;
};

#define DPRTC_ENABLE_SHIFT	0
#define DPRTC_ENABLE_SIZE	1

struct dprtc_rsp_is_enabled {
	uint8_t en;
};

struct dprtc_cmd_get_irq {
	uint32_t pad;
	uint8_t irq_index;
};

struct dprtc_cmd_set_irq_enable {
	uint8_t en;
	uint8_t pad[3];
	uint8_t irq_index;
};

struct dprtc_rsp_get_irq_enable {
	uint8_t en;
};

struct dprtc_cmd_set_irq_mask {
	uint32_t mask;
	uint8_t irq_index;
};

struct dprtc_rsp_get_irq_mask {
	uint32_t mask;
};

struct dprtc_cmd_get_irq_status {
	uint32_t status;
	uint8_t irq_index;
};

struct dprtc_rsp_get_irq_status {
	uint32_t status;
};

struct dprtc_cmd_clear_irq_status {
	uint32_t status;
	uint8_t irq_index;
};

struct dprtc_rsp_get_attributes {
	uint32_t pad;
	uint32_t id;
};

struct dprtc_cmd_set_clock_offset {
	uint64_t offset;
};

struct dprtc_get_freq_compensation {
	uint32_t freq_compensation;
};

struct dprtc_time {
	uint64_t time;
};

struct dprtc_rsp_get_api_version {
	uint16_t major;
	uint16_t minor;
};
#pragma pack(pop)
#endif /* _FSL_DPRTC_CMD_H */
