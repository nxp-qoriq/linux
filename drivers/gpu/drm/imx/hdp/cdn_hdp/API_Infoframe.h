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
 * API_Infoframe.h
 *
 ******************************************************************************
 */

#ifndef API_INFOFRAME_H
#define API_INFOFRAME_H

#include "API_General.h"
/**
 * \addtogroup INFO_FRAME_API
 * \{
 */
CDN_API_STATUS CDN_API_InfoframeSet(state_struct *state, u8 entry_id,
				    u8 packet_len, u32 *packet,
				    u8 packet_type);
CDN_API_STATUS CDN_API_InfoframeSetNoActiveIdle(state_struct *state,
						u8 entry_id, u8 packet_len,
						u32 *packet, u8 packet_type);
CDN_API_STATUS CDN_API_InfoframeRemove(state_struct *state, u8 entry_id);

#endif
