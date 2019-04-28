/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */
#ifndef _IMX_DP_H_
#define _IMX_DP_H_

void hdp_fw_load(state_struct *state);
int hdp_fw_init(state_struct *state);
void dp_mode_set(state_struct *state,
		 const struct drm_display_mode *mode,
		 int format, int color_depth,
		 int max_link_rate);
int dp_phy_init(state_struct *state,
		const struct drm_display_mode *mode,
		int format,
		int color_depth);
int dp_phy_init_t28hpc(state_struct *state,
		       const struct drm_display_mode *mode,
		       int format,
		       int color_depth);
int dp_get_edid_block(void *data, u8 *buf, u32 block, size_t len);
int dp_get_hpd_state(state_struct *state, u8 *hpd);

#endif
