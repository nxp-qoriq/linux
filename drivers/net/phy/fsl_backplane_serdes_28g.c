// SPDX-License-Identifier: GPL-2.0+
/*
 *  NXP backplane driver for SerDes 28G.
 *   Author: Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2018 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/io.h>
#include <linux/delay.h>

#include "fsl_backplane.h"

#define PRE_COE_SHIFT				16
#define POST_COE_SHIFT				8
#define ZERO_COE_SHIFT				24

#define TECR0_INIT					0x20808000

#define TECR0_AMP_RED_MASK			0x0000003f

#define GCR0_RESET_MASK				0x020000
#define BIN_M1_SEL					0x0000c000
#define BIN_Long_SEL				0x0000d000
#define CDR_SEL_MASK				0x0000f000

#define RECR3_SNP_START_MASK		0x80000000
#define RECR3_SNP_DONE_MASK			0x40000000

#define RECR4_SNP_DATA_MASK			0x000003ff
#define RECR4_SNP_DATA_SHIFT		0
#define RECR4_EQ_SNPBIN_SIGN_MASK	0x200

#define RECR3_GAINK2_MASK			0x1f000000
#define RECR3_GAINK2_SHIFT			24

//used only for 1GKX:
#define GCR1_REIDL_TH_MASK			0x00700000
#define GCR1_REIDL_EX_SEL_MASK		0x000c0000
#define GCR1_REIDL_ET_MAS_MASK		0x04000000
#define TECR0_AMP_RED_MASK			0x0000003f

struct per_lane_ctrl_status {
	__be32 gcr0;	/* 0x.000 - General Control Register 0 */
	__be32 resv1;	/* 0x.004 - Reserved */
	__be32 resv2;	/* 0x.008 - Reserved */
	__be32 resv3;	/* 0x.00C - Reserved */
	__be32 resv4;	/* 0x.010 - Reserved */
	__be32 resv5;	/* 0x.014 - Reserved */
	__be32 resv6;	/* 0x.018 - Reserved */
	__be32 resv7;	/* 0x.01C - Reserved */
	__be32 trstctl;	/* 0x.020 - TX Reset Control Register */
	__be32 tgcr0;	/* 0x.024 - TX General Control Register 0 */
	__be32 tgcr1;	/* 0x.028 - TX General Control Register 1 */
	__be32 tgcr2;	/* 0x.02C - TX General Control Register 2 */
	__be32 tecr0;	/* 0x.030 - Transmit Equalization Control Register 0 */
	__be32 tecr1;	/* 0x.034 - Transmit Equalization Control Register 1 */
	__be32 resv8;	/* 0x.038 - Reserved */
	__be32 resv9;	/* 0x.03C - Reserved */
	__be32 rrstctl;	/* 0x.040 - RX Reset Control Register */
	__be32 rgcr0;	/* 0x.044 - RX General Control Register 0 */
	__be32 rxgcr1; 	/* 0x.048 - RX General Control Register 1 */
	__be32 resv10;	/* 0x.04C - Reserved */
	__be32 recr0;	/* 0x.050 - RX Equalization Register 0 */
	__be32 recr1;	/* 0x.054 - RX Equalization Register 1 */
	__be32 recr2;	/* 0x.058 - RX Equalization Register 2 */
	__be32 recr3;	/* 0x.05C - RX Equalization Register 3 */
	__be32 recr4;	/* 0x.060 - RX Equalization Register 4 */
	__be32 resv11;	/* 0x.064 - Reserved */
	__be32 rccr0;	/* 0x.068 - RX Calibration Register 0 */
	__be32 rccr1;	/* 0x.06C - RX Calibration Register 1 */
	__be32 rcpcr0;	/* 0x.070 - RX Clock Path Register 0 */
	__be32 rsccr0;	/* 0x.074 - RX Sampler Calibration Control Register 0 */
	__be32 rsccr1;	/* 0x.078 - RX Sampler Calibration Control Register 1 */
	__be32 resv12;	/* 0x.07C - Reserved */
	__be32 ttlcr0;	/* 0x.080 - Transition Tracking Loop Register 0 */
	__be32 ttlcr1;	/* 0x.084 - Transition Tracking Loop Register 1 */
	__be32 ttlcr2;	/* 0x.088 - Transition Tracking Loop Register 2 */
	__be32 ttlcr3;	/* 0x.08C - Transition Tracking Loop Register 3 */
	__be32 resv13;	/* 0x.090 - Reserved */
	__be32 resv14;	/* 0x.094 - Reserved */
	__be32 resv15;	/* 0x.098 - Reserved */
	__be32 resv16;	/* 0x.09C - Reserved */
	__be32 tcsr0;	/* 0x.0A0 - Test Control/Status Register 0 */
	__be32 tcsr1;	/* 0x.0A4 - Test Control/Status Register 1 */
	__be32 tcsr2;	/* 0x.0A8 - Test Control/Status Register 2 */
	__be32 tcsr3;	/* 0x.0AC - Test Control/Status Register 3 */
	__be32 tcsr4;	/* 0x.0B0 - Test Control/Status Register 4 */
	__be32 resv17;	/* 0x.0B4 - Reserved */
	__be32 resv18;	/* 0x.0B8 - Reserved */
	__be32 resv19;	/* 0x.0BC - Reserved */
	__be32 rxcb0;	/* 0x.0C0 - RX Control Block Register 0 */
	__be32 rxcb1;	/* 0x.0C4 - RX Control Block Register 1 */
	__be32 resv20;	/* 0x.0C8 - Reserved */
	__be32 resv21;	/* 0x.0CC - Reserved */
	__be32 rxss0;	/* 0x.0D0 - RX Speed Switch Register 0 */
	__be32 rxss1;	/* 0x.0D4 - RX Speed Switch Register 1 */
	__be32 rxss2;	/* 0x.0D8 - RX Speed Switch Register 2 */
	__be32 resv22;	/* 0x.0DC - Reserved */
	__be32 txcb0;	/* 0x.0E0 - TX Control Block Register 0 */
	__be32 txcb1;	/* 0x.0E4 - TX Control Block Register 1 */
	__be32 resv23;	/* 0x.0E8 - Reserved */
	__be32 resv24;	/* 0x.0EC - Reserved */
	__be32 txss0;	/* 0x.0F0 - TX Speed Switch Register 0 */
	__be32 txss1;	/* 0x.0F4 - TX Speed Switch Register 1 */
	__be32 txss2;	/* 0x.0F8 - TX Speed Switch Register 2 */
	__be32 resv25;	/* 0x.0FC - Reserved */
};

static void tune_tecr0(void *reg, u32 ratio_preq, u32 ratio_pst1q, u32 adpt_eq)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	/* reset the lane */
	iowrite32(ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);

	val = TECR0_INIT |
		ratio_preq << PRE_COE_SHIFT |
		ratio_pst1q << POST_COE_SHIFT;
	iowrite32(val, &reg_base->tecr0);

	val = adpt_eq << ZERO_COE_SHIFT;
	iowrite32(val, &reg_base->tecr1);

	udelay(1);

	/* unreset the lane */
	iowrite32(ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
}

static void reset_gcr0(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	iowrite32(ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
	iowrite32(ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
}

static void lane_set_1gkx(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	/* reset the lane */
	iowrite32(ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);

	/* set gcr1 for 1GKX */
	val = ioread32(&reg_base->rxgcr1);
	val &= ~(GCR1_REIDL_TH_MASK | GCR1_REIDL_EX_SEL_MASK |
		 GCR1_REIDL_ET_MAS_MASK);
	iowrite32(val, &reg_base->rxgcr1);
	udelay(1);

	/* set tecr0 for 1GKX */
	val = ioread32(&reg_base->tecr0);
	val &= ~TECR0_AMP_RED_MASK;
	iowrite32(val, &reg_base->tecr0);
	udelay(1);

	/* unreset the lane */
	iowrite32(ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
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
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		iowrite32((ioread32(&reg_base->recr3) |
			    RECR3_SNP_START_MASK),
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		rx_eq_snp = ioread32(&reg_base->recr3);
		gaink2_snap_shot[i] = (rx_eq_snp & RECR3_GAINK2_MASK) >>
					RECR3_GAINK2_SHIFT;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		iowrite32((ioread32(&reg_base->recr3) &
			    ~RECR3_SNP_START_MASK),
			    &reg_base->recr3);
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
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set TCSR1[CDR_SEL] to BinM1/BinLong */
		if (bin_sel == BIN_M1) {
			iowrite32((ioread32(&reg_base->recr4) &
				    ~CDR_SEL_MASK) | BIN_M1_SEL,
				    &reg_base->recr4);
		} else {
			iowrite32((ioread32(&reg_base->recr4) &
				    ~CDR_SEL_MASK) | BIN_Long_SEL,
				    &reg_base->recr4);
		}

		/* start snap shot */
		iowrite32(ioread32(&reg_base->recr3) | RECR3_SNP_START_MASK,
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		bin_snap_shot[i] = (ioread32(&reg_base->recr4) &
				RECR4_SNP_DATA_MASK) >> RECR4_SNP_DATA_SHIFT;
		if (bin_snap_shot[i] & RECR4_EQ_SNPBIN_SIGN_MASK)
			negative_count++;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		iowrite32(ioread32(&reg_base->recr3) & ~RECR3_SNP_START_MASK,
			    &reg_base->recr3);
	}

	if (((bin_sel == BIN_M1) && (negative_count > BIN_M1_THRESHOLD)) ||
	    ((bin_sel == BIN_LONG && (negative_count > BIN_LONG_THRESHOLD)))) {
		early = true;
	}

	return early;
}

void setup_backplane_serdes_28g(struct backplane_serdes *bckpl_serdes)
{
	bckpl_serdes->tune_tecr0 = tune_tecr0;
	bckpl_serdes->reset_gcr0 = reset_gcr0;
	bckpl_serdes->lane_set_1gkx = lane_set_1gkx;
	bckpl_serdes->get_median_gaink2 = get_median_gaink2;
	bckpl_serdes->is_bin_early = is_bin_early;
}
