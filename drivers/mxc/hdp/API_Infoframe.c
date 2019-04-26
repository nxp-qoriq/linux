/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 *
 * API_Infoframe.c
 *
 ******************************************************************************
 */

#include "API_Infoframe.h"
#include "address.h"
#include "source_pif.h"
#include "util.h"

#define BANK_OFFSET 0x0

static CDN_API_STATUS infoframeSet(state_struct *state, u8 entry_id,
				   u8 packet_len,
				   u32 *packet, u8 packet_type, u8 active_idle)
{
	u32 idx;
	u32 activeIdleBit = (active_idle == 0) ? 0 : 0x20000;

	/* invalidate entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     activeIdleBit | F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	/* flush fifo 1 */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_FIFO1_FLUSH << 2),
	     F_FIFO1_FLUSH(1)))
		return CDN_ERR;

	/* write packet into memory */
	for (idx = 0; idx < packet_len; idx++)
		if (cdn_apb_write
		    (state,
		     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_DATA_WR << 2),
		     F_DATA_WR(packet[idx])))
			return CDN_ERR;

	/* write entry id */
	if (cdn_apb_write
	    (state, BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_WR_ADDR << 2),
	     F_WR_ADDR(entry_id)))
		return CDN_ERR;

	/* write request */
	if (cdn_apb_write
	    (state, BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_WR_REQ << 2),
	     F_HOST_WR(1)))
		return CDN_ERR;

	/* update entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     activeIdleBit | F_TYPE_VALID(1) | F_PACKET_TYPE(packet_type) |
	     F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	return CDN_OK;
}

CDN_API_STATUS CDN_API_InfoframeSet(state_struct *state, u8 entry_id,
				    u8 packet_len, u32 *packet, u8 packet_type)
{
	return infoframeSet(state, entry_id, packet_len, packet, packet_type,
			    1);
}

CDN_API_STATUS CDN_API_InfoframeSetNoActiveIdle(state_struct *state,
						u8 entry_id, u8 packet_len,
						u32 *packet, u8 packet_type)
{
	return infoframeSet(state, entry_id, packet_len, packet, packet_type,
			    0);
}

CDN_API_STATUS CDN_API_InfoframeRemove(state_struct *state, u8 entry_id)
{
	/* invalidate entry */
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_REG << 2),
	     0x20000 | F_PKT_ALLOC_ADDRESS(entry_id)))
		return CDN_ERR;
	if (cdn_apb_write
	    (state,
	     BANK_OFFSET | ADDR_SOURCE_PIF | (SOURCE_PIF_PKT_ALLOC_WR_EN << 2),
	     F_PKT_ALLOC_WR_EN(1)))
		return CDN_ERR;

	return CDN_OK;
}
