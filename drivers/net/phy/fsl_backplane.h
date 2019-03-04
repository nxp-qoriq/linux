/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  DPAA backplane driver.
 *   Author: Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2018 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef FSL_BACKPLANE_H
#define FSL_BACKPLANE_H

/* C(-1) */
#define BIN_M1						0
/* C(1) */
#define BIN_LONG					1

#define BIN_SNAPSHOT_NUM			5
#define BIN_M1_THRESHOLD			3
#define BIN_LONG_THRESHOLD			2

struct serdes_access {

	int serdes_type;
	bool is_little_endian;
	u32 (*ioread32)(u32 *reg);
	void (*iowrite32)(u32 value, u32 *reg);
	u32 (*get_lane_memmap_size)(void);
	void (*tune_tecr)(void *reg, u32 ratio_preq, u32 ratio_pst1q, u32 adpt_eq, bool reset);
	void (*reset_lane)(void *reg);
	void (*lane_set_1gkx)(void *reg);
	int (*get_median_gaink2)(u32 *reg);
	bool (*is_bin_early)(int bin_sel, void *reg);
};

struct serdes_access* setup_serdes_access_10g(void);
struct serdes_access* setup_serdes_access_28g(void);


#endif //FSL_BACKPLANE_H
