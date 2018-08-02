/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  NXP backplane driver.
 *   Author: Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2018 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef FSL_BACKPLANE_H
#define FSL_BACKPLANE_H

/* C(-1) */
#define BIN_M1					0
/* C(1) */
#define BIN_LONG				1

#define BIN_SNAPSHOT_NUM			5
#define BIN_M1_THRESHOLD			3
#define BIN_LONG_THRESHOLD			2

struct backplane_serdes {

	void (*tune_tecr0)(void *reg, u32 ratio_preq, u32 ratio_pst1q, u32 adpt_eq);
	void (*reset_gcr0)(void *reg);
	void (*lane_set_1gkx)(void *reg);
	int (*get_median_gaink2)(u32 *reg);
	bool (*is_bin_early)(int bin_sel, void *reg);
};

void setup_backplane_serdes_10g(struct backplane_serdes *bckpl_serdes);
void setup_backplane_serdes_28g(struct backplane_serdes *bckpl_serdes);


#endif //FSL_BACKPLANE_H
