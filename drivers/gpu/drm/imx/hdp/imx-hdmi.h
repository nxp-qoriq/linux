/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */
#ifndef _IMX_HDMI_H_
#define _IMX_HDMI_H_
void hdmi_fw_load(state_struct *state);
int hdmi_fw_init(state_struct *state, u32 rate);
void hdmi_phy_init(state_struct *state, int vic, int format, int color_depth);
void hdmi_mode_set(state_struct *state, int vic, int format, int color_depth, int max_link);
int hdmi_get_edid_block(void *data, u8 *buf, u32 block, size_t len);
void hdmi_get_hpd_state(state_struct *state, u8 *hpd);

#endif
