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
 * API_AVI.h
 *
 ******************************************************************************
 */

#ifndef API_AVI_H_
#define API_AVI_H_

#include "API_General.h"

CDN_API_STATUS CDN_API_Set_AVI(state_struct *state,
			       struct drm_display_mode *mode,
			       VIC_PXL_ENCODING_FORMAT colorMode,
			       BT_TYPE ITUver);

#endif /* API_AVI_H_ */
