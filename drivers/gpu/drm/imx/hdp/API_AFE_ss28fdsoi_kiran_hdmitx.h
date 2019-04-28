/******************************************************************************
 *
 * Copyright (C) 2016-2017 Cadence Design Systems, Inc.
 * All rights reserved worldwide.
 *
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ******************************************************************************
 *
 * API_AFE_ss28fdsoi_kiran_hdmitx.h
 *
 ******************************************************************************
 */

#ifndef API_AFE_SS28FDSOI_KIRAN_HDMITX_H_
#define API_AFE_SS28FDSOI_KIRAN_HDMITX_H_

#include "../../../../mxc/hdp/all.h"

int phy_cfg_hdp_ss28fdsoi(state_struct *state, int num_lanes,
			  struct drm_display_mode *mode, int bpp,
			  VIC_PXL_ENCODING_FORMAT format);
int hdmi_tx_kiran_power_configuration_seq(state_struct *state, int num_lanes);
int get_table_row_match_column(const u32 *array, u32 table_rows,
			       u32 table_cols, u32 start_row,
			       u32 column_to_search,
			       u32 value_to_search_in_column);
int get_table_row(const u32 *array, u32 table_rows,
		  u32 table_cols, u32 variable_in_range,
		  u32 range_min_column, u32 range_max_column,
		  u32 column_to_search, u32 column_value);

#endif
