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
 * API_Audio.h
 *
 ******************************************************************************
 */

#ifndef API_AUDIO_H_
#define API_AUDIO_H_

#include "API_General.h"
/**
 * \addtogroup AUDIO_API
 * \{
 */
typedef enum {
	AUDIO_TYPE_I2S,
	AUDIO_TYPE_SPIDIF_INTERNAL,
	AUDIO_TYPE_SPIDIF_EXTERNAL,
} AUDIO_TYPE;

typedef enum {
	AUDIO_FREQ_32,
	AUDIO_FREQ_48,
	AUDIO_FREQ_96,
	AUDIO_FREQ_192,
	AUDIO_FREQ_44_1,
	AUDIO_FREQ_88_2,
	AUDIO_FREQ_176_4,
} AUDIO_FREQ;

typedef enum {
	AUDIO_WIDTH_16,
	AUDIO_WIDTH_24,
	AUDIO_WIDTH_32,
} AUDIO_WIDTH;

typedef enum {
	AUDIO_MODE_OFF,
	AUDIO_MODE_ON
} AUDIO_MODE;

typedef enum {
	AUDIO_MUTE_MODE_MUTE,
	AUDIO_MUTE_MODE_UNMUTE
} AUDIO_MUTE_MODE;

/**
 * \brief mute or unmute audio
 */
CDN_API_STATUS CDN_API_AudioMute(state_struct *state, AUDIO_MUTE_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioMute
 */
CDN_API_STATUS CDN_API_AudioMute_blocking(state_struct *state,
					  AUDIO_MUTE_MODE mode);

/**
 * \brief start playing audio with the input parameters
    ncts and mode are relevant only in HDMI TX mode , not relevant for DPTX mode
 */
CDN_API_STATUS CDN_API_AudioAutoConfig(state_struct *state,
				       AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width,
				       CDN_PROTOCOL_TYPE protocol, int ncts,
				       AUDIO_MUTE_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioAutoConfig
 */
CDN_API_STATUS CDN_API_AudioAutoConfig_blocking(state_struct *state,
						AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width,
						CDN_PROTOCOL_TYPE protocol,
						int ncts, AUDIO_MUTE_MODE mode);

/**
 * \brief audio off (use it to stop current audio and start new one using CDN_API_AudioAutoConfig)
 */
CDN_API_STATUS CDN_API_AudioOff(state_struct *state, AUDIO_TYPE audioType);

/**
 * \brief blocking version of #CDN_API_AudioOff
 */
CDN_API_STATUS CDN_API_AudioOff_blocking(state_struct *state,
					 AUDIO_TYPE audioType);

/**
 * \brief internal function to set audio on or off inside internal registers
 */
CDN_API_STATUS CDN_API_AudioMode(state_struct *state, AUDIO_MODE mode);

/**
 * \brief blocking version of #CDN_API_AudioMode
 */
CDN_API_STATUS CDN_API_AudioMode_blocking(state_struct *state,
					  AUDIO_MODE mode);

/**
 * \brief internal function to set audio core registers
 */
CDN_API_STATUS CDN_API_AudioConfigCore(state_struct *state,
				       AUDIO_TYPE audioType, int numOfChannels,
				       AUDIO_FREQ freq, int lanes,
				       AUDIO_WIDTH width);

/**
 * \brief blocking version of #CDN_API_AudioConfigCore
 */
CDN_API_STATUS CDN_API_AudioConfigCore_blocking(state_struct *state,
						AUDIO_TYPE audioType,
						int numOfChannels,
						AUDIO_FREQ freq, int lanes,
						AUDIO_WIDTH width);

#endif
