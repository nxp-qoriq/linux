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
 * API_HDMI_Audio.h
 *
 ******************************************************************************
 */

#ifndef API_HDMI_Audio_H_
#define API_HDMI_Audio_H_

#include "API_General.h"
#include "API_Audio.h"

/**
 * \addtogroup HDMI_TX_API
 * \{
 */

 /**
 * \brief send audio info frame according to parameters
 * \returns status
 */
CDN_API_STATUS CDN_API_HDMI_AudioSetInfoFrame(state_struct *state,
					      AUDIO_MUTE_MODE mode,
					      AUDIO_TYPE audioType,
					      int numOfChannels,
					      AUDIO_FREQ freq, int lanes,
					      int ncts);

#endif
