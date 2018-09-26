/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */
#ifndef _IMX_DP_H_
#define _IMX_DP_H_

void dp_fw_load(state_struct *state);
int dp_fw_init(state_struct *state, u32 rate);
void dp_mode_set(state_struct *state, int vic, int format, int color_depth, int max_link_rate);
void dp_phy_init(state_struct *state, int num_lanes, int max_link_rate, int tmp);
int dp_get_edid_block(void *data, u8 *buf, u32 block, size_t len);
void dp_get_hpd_state(state_struct *state, u8 *hpd);

#endif
