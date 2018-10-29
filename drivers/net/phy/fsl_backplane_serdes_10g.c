// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA backplane driver for SerDes 10G.
 *   Author: Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2018 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/io.h>
#include <linux/delay.h>

#include "fsl_backplane.h"

#define BIN_M1_SEL					6
#define BIN_Long_SEL				7
#define CDR_SEL_MASK				0x00070000

#define PRE_COE_SHIFT				22
#define POST_COE_SHIFT				16
#define ZERO_COE_SHIFT				8

#define TECR0_INIT					0x24200000

#define GCR0_RESET_MASK				0x00600000

#define GCR1_SNP_START_MASK			0x00000040
#define GCR1_CTL_SNP_START_MASK		0x00002000

#define RECR1_CTL_SNP_DONE_MASK		0x00000002
#define RECR1_SNP_DONE_MASK			0x00000004
#define TCSR1_SNP_DATA_MASK			0x0000ffc0
#define TCSR1_SNP_DATA_SHIFT		6
#define TCSR1_EQ_SNPBIN_SIGN_MASK	0x100

#define RECR1_GAINK2_MASK			0x0f000000
#define RECR1_GAINK2_SHIFT			24

/* Required only for 1000BASE KX */
#define GCR1_REIDL_TH_MASK			0x00700000
#define GCR1_REIDL_EX_SEL_MASK		0x000c0000
#define GCR1_REIDL_ET_MAS_MASK		0x00004000
#define TECR0_AMP_RED_MASK			0x0000003f

struct per_lane_ctrl_status {
	u32 gcr0;	/* 0x.000 - General Control Register 0 */
	u32 gcr1;	/* 0x.004 - General Control Register 1 */
	u32 gcr2;	/* 0x.008 - General Control Register 2 */
	u32 resv1;	/* 0x.00C - Reserved */
	u32 recr0;	/* 0x.010 - Receive Equalization Control Register 0 */
	u32 recr1;	/* 0x.014 - Receive Equalization Control Register 1 */
	u32 tecr0;	/* 0x.018 - Transmit Equalization Control Register 0 */
	u32 resv2;	/* 0x.01C - Reserved */
	u32 tlcr0;	/* 0x.020 - TTL Control Register 0 */
	u32 tlcr1;	/* 0x.024 - TTL Control Register 1 */
	u32 tlcr2;	/* 0x.028 - TTL Control Register 2 */
	u32 tlcr3;	/* 0x.02C - TTL Control Register 3 */
	u32 tcsr0;	/* 0x.030 - Test Control/Status Register 0 */
	u32 tcsr1;	/* 0x.034 - Test Control/Status Register 1 */
	u32 tcsr2;	/* 0x.038 - Test Control/Status Register 2 */
	u32 tcsr3;	/* 0x.03C - Test Control/Status Register 3 */
};

static struct serdes_access srds;

static u32 get_lane_memmap_size(void)
{
	return 0x40;
}

static void reset_lane(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	/* reset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
	
	/* unreset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
}

static void tune_tecr(void *reg, u32 ratio_preq, u32 ratio_pst1q, u32 adpt_eq, bool reset)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	val = TECR0_INIT |
		adpt_eq << ZERO_COE_SHIFT |
		ratio_preq << PRE_COE_SHIFT |
		ratio_pst1q << POST_COE_SHIFT;

	if (reset) {
		/* reset the lane */
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
				&reg_base->gcr0);
		udelay(1);
	}
	
	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);
	
	if (reset) {
		/* unreset the lane */
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
				&reg_base->gcr0);
		udelay(1);
	}
}

static void lane_set_1gkx(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	/* reset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);

	/* set gcr1 for 1GKX */
	val = srds.ioread32(&reg_base->gcr1);
	val &= ~(GCR1_REIDL_TH_MASK | GCR1_REIDL_EX_SEL_MASK |
		 GCR1_REIDL_ET_MAS_MASK);
	srds.iowrite32(val, &reg_base->gcr1);
	udelay(1);

	/* set tecr0 for 1GKX */
	val = srds.ioread32(&reg_base->tecr0);
	val &= ~TECR0_AMP_RED_MASK;
	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);

	/* unreset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
}

static int get_median_gaink2(u32 *reg)
{
	int gaink2_snap_shot[BIN_SNAPSHOT_NUM];
	u32 rx_eq_snp;
	struct per_lane_ctrl_status *reg_base;
	int timeout;
	int i, j, tmp, pos;

	reg_base = (struct per_lane_ctrl_status *)reg;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_CTL_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) |
			    GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		rx_eq_snp = srds.ioread32(&reg_base->recr1);
		gaink2_snap_shot[i] = (rx_eq_snp & RECR1_GAINK2_MASK) >>
					RECR1_GAINK2_SHIFT;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) &
			    ~GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);
	}

	/* get median of the 5 snap shot */
	for (i = 0; i < BIN_SNAPSHOT_NUM - 1; i++) {
		tmp = gaink2_snap_shot[i];
		pos = i;
		for (j = i + 1; j < BIN_SNAPSHOT_NUM; j++) {
			if (gaink2_snap_shot[j] < tmp) {
				tmp = gaink2_snap_shot[j];
				pos = j;
			}
		}

		gaink2_snap_shot[pos] = gaink2_snap_shot[i];
		gaink2_snap_shot[i] = tmp;
	}

	return gaink2_snap_shot[2];
}

static bool is_bin_early(int bin_sel, void *reg)
{
	bool early = false;
	int bin_snap_shot[BIN_SNAPSHOT_NUM];
	int i, negative_count = 0;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set TCSR1[CDR_SEL] to BinM1/BinLong */
		if (bin_sel == BIN_M1) {
			srds.iowrite32((srds.ioread32(&reg_base->tcsr1) &
				    ~CDR_SEL_MASK) | BIN_M1_SEL,
				    &reg_base->tcsr1);
		} else {
			srds.iowrite32((srds.ioread32(&reg_base->tcsr1) &
				    ~CDR_SEL_MASK) | BIN_Long_SEL,
				    &reg_base->tcsr1);
		}

		/* start snap shot */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) | GCR1_SNP_START_MASK,
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		bin_snap_shot[i] = (srds.ioread32(&reg_base->tcsr1) &
				TCSR1_SNP_DATA_MASK) >> TCSR1_SNP_DATA_SHIFT;
		if (bin_snap_shot[i] & TCSR1_EQ_SNPBIN_SIGN_MASK)
			negative_count++;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) & ~GCR1_SNP_START_MASK,
			    &reg_base->gcr1);
	}

	if (((bin_sel == BIN_M1) && (negative_count > BIN_M1_THRESHOLD)) ||
	    ((bin_sel == BIN_LONG && (negative_count > BIN_LONG_THRESHOLD)))) {
		early = true;
	}

	return early;
}

struct serdes_access* setup_serdes_access_10g(void)
{
	srds.get_lane_memmap_size = get_lane_memmap_size;
	srds.tune_tecr = tune_tecr;
	srds.reset_lane = reset_lane;
	srds.lane_set_1gkx = lane_set_1gkx;
	srds.get_median_gaink2 = get_median_gaink2;
	srds.is_bin_early = is_bin_early;

	return &srds;
}

