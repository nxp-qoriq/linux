// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA backplane driver.
 *   Author: Shaohui Xie <Shaohui.Xie@freescale.com>
 *           Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/mdio.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>

#include "fsl_backplane.h"


/* PCS Device Identifier */
#define PCS_PHY_DEVICE_ID			0x0083e400
#define PCS_PHY_DEVICE_ID_MASK		0xffffffff

/* 10G Long cables setup: 1 m to 2 m cables */
#define RATIO_PREQ_10G				0x3
#define RATIO_PST1Q_10G				0xd
#define RATIO_EQ_10G				0x20

/* 10G Short cables setup: up to 30 cm cable */
//#define RATIO_PREQ_10G				0x3
//#define RATIO_PST1Q_10G				0xa
//#define RATIO_EQ_10G				0x29

/* 40G Long cables setup: 1 m to 2 m cables */
#define RATIO_PREQ_40G				0x2
#define RATIO_PST1Q_40G				0xd
#define RATIO_EQ_40G				0x20

/* 40G Short cables setup: up to 30 cm cable */
//#define RATIO_PREQ_40G				0x1
//#define RATIO_PST1Q_40G				0x3
//#define RATIO_EQ_40G				0x29

/* LX2 2x40G default RCW setup */
//#define RATIO_PREQ_40G				0x0
//#define RATIO_PST1Q_40G				0x3
//#define RATIO_EQ_40G				0x30

/* Max/Min coefficient values */
#define PRE_COE_MAX					0x0
#define PRE_COE_MIN					0x8
#define POST_COE_MAX				0x0
#define POST_COE_MIN				0x10
#define ZERO_COE_MAX				0x30
#define ZERO_COE_MIN				0x0

/* KR PMD defines */
#define PMD_RESET					0x1
#define PMD_STATUS_SUP_STAT			0x4
#define PMD_STATUS_FRAME_LOCK		0x2
#define TRAIN_EN					0x3
#define TRAIN_DISABLE				0x1
#define RX_STAT						0x1

/* PCS Link up */
#define XFI_PCS_SR1             	0x20
#define KR_RX_LINK_STAT_MASK		0x1000

/* KX PCS mode register */
#define KX_PCS_IF_MODE				0x8014

/* KX PCS mode register init value */
#define KX_IF_MODE_INIT				0x8

/* KX/KR AN registers */
#define AN_CTRL_INIT				0x1200
#define KX_AN_AD1_INIT				0x25
#define KR_AN_AD1_INIT_10G			0x85
#define KR_AN_AD1_INIT_40G			0x105
#define AN_LNK_UP_MASK				0x4
#define KR_AN_MASK_10G				0x8
#define KR_AN_MASK_40G				0x20
#define TRAIN_FAIL					0x8
#define KR_AN_40G_MDIO_OFFSET		4

/* XGKR Timeouts */
#define XGKR_TIMEOUT				1050
#define XGKR_DENY_RT_INTERVAL		3000
#define XGKR_AN_WAIT_ITERATIONS 	5

/* XGKR Increment/Decrement Requests */
#define INCREMENT					1
#define DECREMENT					2
#define TIMEOUT_LONG				3
#define TIMEOUT_M1					3

/* XGKR Masks */
#define RX_READY_MASK				0x8000
#define PRESET_MASK					0x2000
#define INIT_MASK					0x1000
#define COP1_MASK					0x30
#define COP1_SHIFT					4
#define COZ_MASK					0xc
#define COZ_SHIFT					2
#define COM1_MASK					0x3
#define COM1_SHIFT					0
#define REQUEST_MASK				0x3f
#define LD_ALL_MASK			(PRESET_MASK | INIT_MASK | \
					COP1_MASK | COZ_MASK | COM1_MASK)

/* Lanes definitions */
#define MASTER_LANE					0
#define SINGLE_LANE					0
#define MAX_PHY_LANES_NO			4

/* Invalid value */
#define VAL_INVALID 				0xff

/* New XGKR Training Algorithm */
#define NEW_ALGORITHM_TRAIN_TX

#ifdef	NEW_ALGORITHM_TRAIN_TX
#define	FORCE_INC_COP1_NUMBER		0
#define	FORCE_INC_COM1_NUMBER		1
#endif

/* Link_Training_Registers offsets */
static int lt_MDIO_MMD = 0;
static u32 lt_KR_PMD_CTRL = 0;
static u32 lt_KR_PMD_STATUS = 0;
static u32 lt_KR_LP_CU = 0;
static u32 lt_KR_LP_STATUS = 0;
static u32 lt_KR_LD_CU = 0;
static u32 lt_KR_LD_STATUS = 0;

/* KX/KR AN registers offsets */
static u32 g_an_AD1 = 0;
static u32 g_an_BP_STAT = 0;

static const u32 preq_table[] = {0x0, 0x1, 0x3, 0x5,
				 0x7, 0x9, 0xb, 0xc, VAL_INVALID};
static const u32 pst1q_table[] = {0x0, 0x1, 0x3, 0x5, 0x7,
				  0x9, 0xb, 0xd, 0xf, 0x10, VAL_INVALID};

enum backplane_mode {
	PHY_BACKPLANE_1000BASE_KX,
	PHY_BACKPLANE_10GBASE_KR,
	PHY_BACKPLANE_40GBASE_KR,
	PHY_BACKPLANE_INVAL
};

enum serdes_type {
	SERDES_10G,
	SERDES_28G,
	SERDES_INVAL
};

enum coe_filed {
	COE_COP1,
	COE_COZ,
	COE_COM
};

enum coe_update {
	COE_NOTUPDATED,
	COE_UPDATED,
	COE_MIN,
	COE_MAX,
	COE_INV
};

enum train_state {
	DETECTING_LP,
	TRAINED,
};

struct tx_condition {
	bool bin_m1_late_early;
	bool bin_long_late_early;
	bool bin_m1_stop;
	bool bin_long_stop;
	bool tx_complete;
	bool sent_init;
	int m1_min_max_cnt;
	int long_min_max_cnt;
#ifdef	NEW_ALGORITHM_TRAIN_TX
	int pre_inc;
	int post_inc;
#endif
};

struct xgkr_params {
	void *reg_base;		/* lane memory map: registers base address */
	int idx;			/* lane relative index inside a multi-lane PHY */
	struct phy_device *phydev;
	struct serdes_access *srds;
	struct tx_condition tx_c;
	struct delayed_work xgkr_wk;
	enum train_state state;
	int an_wait_count;
	unsigned long rt_time;
	u32 ld_update;
	u32 ld_status;
	u32 ratio_preq;
	u32 ratio_pst1q;
	u32 adpt_eq;
	u32 tuned_ratio_preq;
	u32 tuned_ratio_pst1q;
	u32 tuned_adpt_eq;
};

struct xgkr_phy_data {
	int bp_mode;
	u32 phy_lanes;
	struct mutex phy_lock;
	bool aneg_done;
	struct xgkr_params xgkr[MAX_PHY_LANES_NO];
};

static void setup_an_lt_ls(void)
{
	/* KR PMD registers */
	lt_MDIO_MMD = MDIO_MMD_PMAPMD;
	lt_KR_PMD_CTRL = 0x96;
	lt_KR_PMD_STATUS = 0x97;
	lt_KR_LP_CU = 0x98;
	lt_KR_LP_STATUS = 0x99;
	lt_KR_LD_CU = 0x9a;
	lt_KR_LD_STATUS = 0x9b;

	/* KX/KR AN registers */
	g_an_AD1 = 0x11;
	g_an_BP_STAT = 0x30;
}

static void setup_an_lt_lx(void)
{
	/* Auto-Negotiation and Link Training Core Registers page 1: 256 = 0x100 */
	lt_MDIO_MMD = MDIO_MMD_AN;
	lt_KR_PMD_CTRL = 0x100;
	lt_KR_PMD_STATUS = 0x101;
	lt_KR_LP_CU = 0x102;
	lt_KR_LP_STATUS = 0x103;
	lt_KR_LD_CU = 0x104;
	lt_KR_LD_STATUS = 0x105;

	/* KX/KR AN registers */
	g_an_AD1 = 0x03;
	g_an_BP_STAT = 0x0F;
}

static u32 le_ioread32(u32 *reg)
{
	return ioread32(reg);
}

static void le_iowrite32(u32 value, u32 *reg)
{
	iowrite32(value, reg);
}

static u32 be_ioread32(u32 *reg)
{
	return ioread32be(reg);
}

static void be_iowrite32(u32 value, u32 *reg)
{
	iowrite32be(value, reg);
}

/**
 * xgkr_phy_write_mmd - Wrapper function for phy_write_mmd
 * for writing a register on an MMD on a given PHY.
 *
 * Same rules as for phy_write_mmd();
 */
static int xgkr_phy_write_mmd(struct xgkr_params *xgkr, int devad, u32 regnum, u16 val)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int mdio_addr = phydev->mdio.addr;
	int err;

	mutex_lock(&xgkr_inst->phy_lock);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		//40G AN: prepare mdio address for writing phydev AN registers for 40G on respective lane
		phydev->mdio.addr = KR_AN_40G_MDIO_OFFSET + xgkr->idx;
	}

	err = phy_write_mmd(phydev, devad, regnum, val);
	if (err)
		dev_err(&phydev->mdio.dev, "Writing PHY (%p) MMD = 0x%02x register = 0x%02x failed with error code: 0x%08x \n", phydev, devad, regnum, err);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		//40G AN: restore mdio address
		phydev->mdio.addr = mdio_addr;
	}

	mutex_unlock(&xgkr_inst->phy_lock);

	return err;
}

/**
 * xgkr_phy_read_mmd - Wrapper function for phy_read_mmd
 * for reading a register from an MMD on a given PHY.
 *
 * Same rules as for phy_read_mmd();
 */
static int xgkr_phy_read_mmd(struct xgkr_params *xgkr, int devad, u32 regnum)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int mdio_addr = phydev->mdio.addr;
	int ret;

	mutex_lock(&xgkr_inst->phy_lock);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		//40G AN: prepare mdio address for reading phydev AN registers for 40G on respective lane
		phydev->mdio.addr = KR_AN_40G_MDIO_OFFSET + xgkr->idx;
	}

	ret = phy_read_mmd(phydev, devad, regnum);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		//40G AN: restore mdio address
		phydev->mdio.addr = mdio_addr;
	}

	mutex_unlock(&xgkr_inst->phy_lock);

	return ret;
}

static void tx_condition_init(struct tx_condition *tx_c)
{
	tx_c->bin_m1_late_early = true;
	tx_c->bin_long_late_early = false;
	tx_c->bin_m1_stop = false;
	tx_c->bin_long_stop = false;
	tx_c->tx_complete = false;
	tx_c->sent_init = false;
	tx_c->m1_min_max_cnt = 0;
	tx_c->long_min_max_cnt = 0;
#ifdef	NEW_ALGORITHM_TRAIN_TX
	tx_c->pre_inc = FORCE_INC_COM1_NUMBER;
	tx_c->post_inc = FORCE_INC_COP1_NUMBER;
#endif
}

void tune_tecr(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	bool reset = false;
	
	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		/* Reset only the Master Lane */
		reset = (xgkr->idx == MASTER_LANE);
	} else {
		reset = true;
	}
	
	xgkr->srds->tune_tecr(xgkr->reg_base, xgkr->ratio_preq, xgkr->ratio_pst1q, xgkr->adpt_eq, reset);

	xgkr->tuned_ratio_preq = xgkr->ratio_preq;
	xgkr->tuned_ratio_pst1q = xgkr->ratio_pst1q;
	xgkr->tuned_adpt_eq = xgkr->adpt_eq;
}

static void start_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_EN);
}

static void stop_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_DISABLE);
}

static void reset_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, MDIO_CTRL1, PMD_RESET);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_DISABLE);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LD_CU, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LD_STATUS, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_STATUS, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_CU, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS, 0);
	
}

static void ld_coe_status(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
		      lt_KR_LD_STATUS, xgkr->ld_status);
}

static void ld_coe_update(struct xgkr_params *xgkr)
{
	dev_dbg(&xgkr->phydev->mdio.dev, "sending request: %x\n", xgkr->ld_update);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
		      lt_KR_LD_CU, xgkr->ld_update);
}

static void start_xgkr_state_machine(struct delayed_work *work)
{
	queue_delayed_work(system_power_efficient_wq, work,
			   msecs_to_jiffies(XGKR_TIMEOUT));
}

static void start_xgkr_an(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;
	int err;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, g_an_AD1, KR_AN_AD1_INIT_10G);
		if (err)
			dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x failed with error code: 0x%08x \n", g_an_AD1, err);
		udelay(1);
		err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
		if (err)
			dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x failed with error code: 0x%08x \n", MDIO_CTRL1, err);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		if (xgkr->idx == MASTER_LANE) {
			for (i = 0; i < xgkr_inst->phy_lanes; i++) {
				err = xgkr_phy_write_mmd(&xgkr_inst->xgkr[i], MDIO_MMD_AN, g_an_AD1, KR_AN_AD1_INIT_40G);
				if (err)
					dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x on lane %d failed with error code: 0x%08x \n", g_an_AD1, xgkr_inst->xgkr[i].idx, err);
			}
			udelay(1);
			err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
			if (err)
				dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x on Master Lane failed with error code: 0x%08x \n", MDIO_CTRL1, err);
		}
		break;
	}
}

static void start_1gkx_an(struct phy_device *phydev)
{
	phy_write_mmd(phydev, MDIO_MMD_PCS, KX_PCS_IF_MODE, KX_IF_MODE_INIT);
	phy_write_mmd(phydev, MDIO_MMD_AN, g_an_AD1, KX_AN_AD1_INIT);
	phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
}

static void reset_tecr(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		xgkr->ratio_preq = RATIO_PREQ_10G;
		xgkr->ratio_pst1q = RATIO_PST1Q_10G;
		xgkr->adpt_eq = RATIO_EQ_10G;
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		xgkr->ratio_preq = RATIO_PREQ_40G;
		xgkr->ratio_pst1q = RATIO_PST1Q_40G;
		xgkr->adpt_eq = RATIO_EQ_40G;
		break;
	}

	tune_tecr(xgkr);
}

static void init_xgkr(struct xgkr_params *xgkr, int reset)
{
	if (reset)
		reset_tecr(xgkr);

	tx_condition_init(&xgkr->tx_c);
	xgkr->state = DETECTING_LP;

	xgkr->ld_status &= RX_READY_MASK;
	ld_coe_status(xgkr);
	xgkr->ld_update = 0;
	xgkr->ld_status &= ~RX_READY_MASK;
	ld_coe_status(xgkr);

}

static void initialize(struct xgkr_params *xgkr)
{
	reset_tecr(xgkr);

	xgkr->ld_status &= ~(COP1_MASK | COZ_MASK | COM1_MASK);
	xgkr->ld_status |= COE_UPDATED << COP1_SHIFT |
			   COE_UPDATED << COZ_SHIFT |
			   COE_UPDATED << COM1_SHIFT;
	ld_coe_status(xgkr);
}

static void train_remote_tx(struct xgkr_params *xgkr)
{
	struct tx_condition *tx_c = &xgkr->tx_c;
	bool bin_m1_early, bin_long_early;
	u32 lp_status, old_ld_update;
	u32 status_cop1, status_coz, status_com1;
	u32 req_cop1, req_coz, req_com1, req_preset, req_init;
	u32 temp;
#ifdef	NEW_ALGORITHM_TRAIN_TX
	u32 median_gaink2;
#endif

recheck:
	if (tx_c->bin_long_stop && tx_c->bin_m1_stop) {
		tx_c->tx_complete = true;
		xgkr->ld_status |= RX_READY_MASK;
		ld_coe_status(xgkr);

		/* tell LP we are ready */
		xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
			      lt_KR_PMD_STATUS, RX_STAT);

		return;
	}

	/* We start by checking the current LP status. If we got any responses,
	 * we can clear up the appropriate update request so that the
	 * subsequent code may easily issue new update requests if needed.
	 */
	lp_status = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS) &
				 REQUEST_MASK;

	status_cop1 = (lp_status & COP1_MASK) >> COP1_SHIFT;
	status_coz = (lp_status & COZ_MASK) >> COZ_SHIFT;
	status_com1 = (lp_status & COM1_MASK) >> COM1_SHIFT;

	old_ld_update = xgkr->ld_update;
	req_cop1 = (old_ld_update & COP1_MASK) >> COP1_SHIFT;
	req_coz = (old_ld_update & COZ_MASK) >> COZ_SHIFT;
	req_com1 = (old_ld_update & COM1_MASK) >> COM1_SHIFT;
	req_preset = old_ld_update & PRESET_MASK;
	req_init = old_ld_update & INIT_MASK;

	/* IEEE802.3-2008, 72.6.10.2.3.1
	 * We may clear PRESET when all coefficients show UPDATED or MAX.
	 */
	if (req_preset) {
		if ((status_cop1 == COE_UPDATED || status_cop1 == COE_MAX) &&
		    (status_coz == COE_UPDATED || status_coz == COE_MAX) &&
		    (status_com1 == COE_UPDATED || status_com1 == COE_MAX)) {
			xgkr->ld_update &= ~PRESET_MASK;
		}
	}

	/* IEEE802.3-2008, 72.6.10.2.3.2
	 * We may clear INITIALIZE when no coefficients show NOT UPDATED.
	 */
	if (req_init) {
		if (status_cop1 != COE_NOTUPDATED &&
		    status_coz != COE_NOTUPDATED &&
		    status_com1 != COE_NOTUPDATED) {
			xgkr->ld_update &= ~INIT_MASK;
		}
	}

	/* IEEE802.3-2008, 72.6.10.2.3.2
	 * we send initialize to the other side to ensure default settings
	 * for the LP. Naturally, we should do this only once.
	 */
	if (!tx_c->sent_init) {
		if (!lp_status && !(old_ld_update & (LD_ALL_MASK))) {
			xgkr->ld_update = INIT_MASK;
			tx_c->sent_init = true;
		}
	}

	/* IEEE802.3-2008, 72.6.10.2.3.3
	 * We set coefficient requests to HOLD when we get the information
	 * about any updates On clearing our prior response, we also update
	 * our internal status.
	 */
	if (status_cop1 != COE_NOTUPDATED) {
		if (req_cop1) {
			xgkr->ld_update &= ~COP1_MASK;
#ifdef	NEW_ALGORITHM_TRAIN_TX
			if (tx_c->post_inc) {
				if (req_cop1 == INCREMENT &&
				    status_cop1 == COE_MAX) {
					tx_c->post_inc = 0;
					tx_c->bin_long_stop = true;
					tx_c->bin_m1_stop = true;
				} else {
					tx_c->post_inc -= 1;
				}

				ld_coe_update(xgkr);
				goto recheck;
			}
#endif
			if ((req_cop1 == DECREMENT && status_cop1 == COE_MIN) ||
			    (req_cop1 == INCREMENT && status_cop1 == COE_MAX)) {
				dev_dbg(&xgkr->phydev->mdio.dev, "COP1 hit limit %s",
					(status_cop1 == COE_MIN) ?
					"DEC MIN" : "INC MAX");
				tx_c->long_min_max_cnt++;
				if (tx_c->long_min_max_cnt >= TIMEOUT_LONG) {
					tx_c->bin_long_stop = true;
					ld_coe_update(xgkr);
					goto recheck;
				}
			}
		}
	}

	if (status_coz != COE_NOTUPDATED) {
		if (req_coz)
			xgkr->ld_update &= ~COZ_MASK;
	}

	if (status_com1 != COE_NOTUPDATED) {
		if (req_com1) {
			xgkr->ld_update &= ~COM1_MASK;
#ifdef	NEW_ALGORITHM_TRAIN_TX
			if (tx_c->pre_inc) {
				if (req_com1 == INCREMENT &&
				    status_com1 == COE_MAX)
					tx_c->pre_inc = 0;
				else
					tx_c->pre_inc -= 1;

				ld_coe_update(xgkr);
				goto recheck;
			}
#endif
			/* Stop If we have reached the limit for a parameter. */
			if ((req_com1 == DECREMENT && status_com1 == COE_MIN) ||
			    (req_com1 == INCREMENT && status_com1 == COE_MAX)) {
				dev_dbg(&xgkr->phydev->mdio.dev, "COM1 hit limit %s",
					(status_com1 == COE_MIN) ?
					"DEC MIN" : "INC MAX");
				tx_c->m1_min_max_cnt++;
				if (tx_c->m1_min_max_cnt >= TIMEOUT_M1) {
					tx_c->bin_m1_stop = true;
					ld_coe_update(xgkr);
					goto recheck;
				}
			}
		}
	}

	if (old_ld_update != xgkr->ld_update) {
		ld_coe_update(xgkr);
		/* Redo these status checks and updates until we have no more
		 * changes, to speed up the overall process.
		 */
		goto recheck;
	}

	/* Do nothing if we have pending request. */
	if ((req_coz || req_com1 || req_cop1))
		return;
	else if (lp_status)
		/* No pending request but LP status was not reverted to
		 * not updated.
		 */
		return;

#ifdef	NEW_ALGORITHM_TRAIN_TX
	if (!(xgkr->ld_update & (PRESET_MASK | INIT_MASK))) {
		if (tx_c->pre_inc) {
			xgkr->ld_update = INCREMENT << COM1_SHIFT;
			ld_coe_update(xgkr);
			return;
		}

		if (status_cop1 != COE_MAX) {
			median_gaink2 = xgkr->srds->get_median_gaink2(xgkr->reg_base);
			if (median_gaink2 == 0xf) {
				tx_c->post_inc = 1;
			} else {
				/* Gaink2 median lower than "F" */
				tx_c->bin_m1_stop = true;
				tx_c->bin_long_stop = true;
				goto recheck;
			}
		} else {
			/* C1 MAX */
			tx_c->bin_m1_stop = true;
			tx_c->bin_long_stop = true;
			goto recheck;
		}

		if (tx_c->post_inc) {
			xgkr->ld_update = INCREMENT << COP1_SHIFT;
			ld_coe_update(xgkr);
			return;
		}
	}
#endif

	/* snapshot and select bin */
	bin_m1_early = xgkr->srds->is_bin_early(BIN_M1, xgkr->reg_base);
	bin_long_early = xgkr->srds->is_bin_early(BIN_LONG, xgkr->reg_base);

	if (!tx_c->bin_m1_stop && !tx_c->bin_m1_late_early && bin_m1_early) {
		tx_c->bin_m1_stop = true;
		goto recheck;
	}

	if (!tx_c->bin_long_stop &&
	    tx_c->bin_long_late_early && !bin_long_early) {
		tx_c->bin_long_stop = true;
		goto recheck;
	}

	/* IEEE802.3-2008, 72.6.10.2.3.3
	 * We only request coefficient updates when no PRESET/INITIALIZE is
	 * pending. We also only request coefficient updates when the
	 * corresponding status is NOT UPDATED and nothing is pending.
	 */
	if (!(xgkr->ld_update & (PRESET_MASK | INIT_MASK))) {
		if (!tx_c->bin_long_stop) {
			/* BinM1 correction means changing COM1 */
			if (!status_com1 && !(xgkr->ld_update & COM1_MASK)) {
				/* Avoid BinM1Late by requesting an
				 * immediate decrement.
				 */
				if (!bin_m1_early) {
					/* request decrement c(-1) */
					temp = DECREMENT << COM1_SHIFT;
					xgkr->ld_update = temp;
					ld_coe_update(xgkr);
					tx_c->bin_m1_late_early = bin_m1_early;
					return;
				}
			}

			/* BinLong correction means changing COP1 */
			if (!status_cop1 && !(xgkr->ld_update & COP1_MASK)) {
				/* Locate BinLong transition point (if any)
				 * while avoiding BinM1Late.
				 */
				if (bin_long_early) {
					/* request increment c(1) */
					temp = INCREMENT << COP1_SHIFT;
					xgkr->ld_update = temp;
				} else {
					/* request decrement c(1) */
					temp = DECREMENT << COP1_SHIFT;
					xgkr->ld_update = temp;
				}

				ld_coe_update(xgkr);
				tx_c->bin_long_late_early = bin_long_early;
			}
			/* We try to finish BinLong before we do BinM1 */
			return;
		}

		if (!tx_c->bin_m1_stop) {
			/* BinM1 correction means changing COM1 */
			if (!status_com1 && !(xgkr->ld_update & COM1_MASK)) {
				/* Locate BinM1 transition point (if any) */
				if (bin_m1_early) {
					/* request increment c(-1) */
					temp = INCREMENT << COM1_SHIFT;
					xgkr->ld_update = temp;
				} else {
					/* request decrement c(-1) */
					temp = DECREMENT << COM1_SHIFT;
					xgkr->ld_update = temp;
				}

				ld_coe_update(xgkr);
				tx_c->bin_m1_late_early = bin_m1_early;
			}
		}
	}
}

static int is_link_up(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int val = 0;
	
	mutex_lock(&xgkr_inst->phy_lock);

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, XFI_PCS_SR1);

	mutex_unlock(&xgkr_inst->phy_lock);

	return (val & KR_RX_LINK_STAT_MASK) ? 1 : 0;
}

static int is_link_training_fail(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	int val;
	int timeout = 100;

	val = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_STATUS);

	if (!(val & TRAIN_FAIL) && (val & RX_STAT)) {
		/* check LNK_STAT for sure */
		while (timeout--) {
			if (is_link_up(phydev))
				return 0;

			usleep_range(100, 500);
		}
	}

	return 1;
}

static int check_rx(struct xgkr_params *xgkr)
{
	return xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS) &
			    RX_READY_MASK;
}

/* Coefficient values have hardware restrictions */
static int is_ld_valid(struct xgkr_params *xgkr)
{
	u32 ratio_pst1q = xgkr->ratio_pst1q;
	u32 adpt_eq = xgkr->adpt_eq;
	u32 ratio_preq = xgkr->ratio_preq;

	if ((ratio_pst1q + adpt_eq + ratio_preq) > 48)
		return 0;

	if (((ratio_pst1q + adpt_eq + ratio_preq) * 4) >=
	    ((adpt_eq - ratio_pst1q - ratio_preq) * 17))
		return 0;

	if (ratio_preq > ratio_pst1q)
		return 0;

	if (ratio_preq > 8)
		return 0;

	if (adpt_eq < 26)
		return 0;

	if (ratio_pst1q > 16)
		return 0;

	return 1;
}

static int is_value_allowed(const u32 *val_table, u32 val)
{
	int i;

	for (i = 0;; i++) {
		if (*(val_table + i) == VAL_INVALID)
			return 0;
		if (*(val_table + i) == val)
			return 1;
	}
}

static enum coe_update inc_dec(struct xgkr_params *xgkr, int field, int request)
{
	u32 ld_limit[3], ld_coe[3], step[3];

	ld_coe[0] = xgkr->ratio_pst1q;
	ld_coe[1] = xgkr->adpt_eq;
	ld_coe[2] = xgkr->ratio_preq;

	/* Information specific to the SerDes for 10GBase-KR:
	 * Incrementing C(+1) means *decrementing* RATIO_PST1Q
	 * Incrementing C(0) means incrementing ADPT_EQ
	 * Incrementing C(-1) means *decrementing* RATIO_PREQ
	 */
	step[0] = -1;
	step[1] = 1;
	step[2] = -1;

	switch (request) {
	case INCREMENT:
		ld_limit[0] = POST_COE_MAX;
		ld_limit[1] = ZERO_COE_MAX;
		ld_limit[2] = PRE_COE_MAX;
		if (ld_coe[field] != ld_limit[field])
			ld_coe[field] += step[field];
		else
			/* MAX */
			return COE_MAX;
		break;
	case DECREMENT:
		ld_limit[0] = POST_COE_MIN;
		ld_limit[1] = ZERO_COE_MIN;
		ld_limit[2] = PRE_COE_MIN;
		if (ld_coe[field] != ld_limit[field])
			ld_coe[field] -= step[field];
		else
			/* MIN */
			return COE_MIN;
		break;
	default:
		break;
	}

	if (is_ld_valid(xgkr)) {
		/* accept new ld */
		xgkr->ratio_pst1q = ld_coe[0];
		xgkr->adpt_eq = ld_coe[1];
		xgkr->ratio_preq = ld_coe[2];
		/* only some values for preq and pst1q can be used.
		 * for preq: 0x0, 0x1, 0x3, 0x5, 0x7, 0x9, 0xb, 0xc.
		 * for pst1q: 0x0, 0x1, 0x3, 0x5, 0x7, 0x9, 0xb, 0xd, 0xf, 0x10.
		 */
		if (!is_value_allowed((const u32 *)&preq_table, ld_coe[2])) {
			dev_dbg(&xgkr->phydev->mdio.dev,
				"preq skipped value: %d\n", ld_coe[2]);
			/* NOT UPDATED */
			return COE_NOTUPDATED;
		}

		if (!is_value_allowed((const u32 *)&pst1q_table, ld_coe[0])) {
			dev_dbg(&xgkr->phydev->mdio.dev,
				"pst1q skipped value: %d\n", ld_coe[0]);
			/* NOT UPDATED */
			return COE_NOTUPDATED;
		}

		tune_tecr(xgkr);
	} else {
		if (request == DECREMENT)
			/* MIN */
			return COE_MIN;
		if (request == INCREMENT)
			/* MAX */
			return COE_MAX;
	}

	/* UPDATED */
	return COE_UPDATED;
}

static void min_max_updated(struct xgkr_params *xgkr, int field, enum coe_update cs)
{
	u32 mask, val;
	u32 ld_cs = cs;

	if (cs == COE_INV)
		return;

	switch (field) {
	case COE_COP1:
		mask = COP1_MASK;
		val = ld_cs << COP1_SHIFT;
		break;
	case COE_COZ:
		mask = COZ_MASK;
		val = ld_cs << COZ_SHIFT;
		break;
	case COE_COM:
		mask = COM1_MASK;
		val = ld_cs << COM1_SHIFT;
		break;
	default:
		return;
	}

	xgkr->ld_status &= ~mask;
	xgkr->ld_status |= val;
}

static void check_request(struct xgkr_params *xgkr, int request)
{
	int cop1_req, coz_req, com_req;
	int old_status;
	enum coe_update cu;

	cop1_req = (request & COP1_MASK) >> COP1_SHIFT;
	coz_req = (request & COZ_MASK) >> COZ_SHIFT;
	com_req = (request & COM1_MASK) >> COM1_SHIFT;

	/* IEEE802.3-2008, 72.6.10.2.5
	 * Ensure we only act on INCREMENT/DECREMENT when we are in NOT UPDATED
	 */
	old_status = xgkr->ld_status;

	if (cop1_req && !(xgkr->ld_status & COP1_MASK)) {
		cu = inc_dec(xgkr, COE_COP1, cop1_req);
		min_max_updated(xgkr, COE_COP1, cu);
	}

	if (coz_req && !(xgkr->ld_status & COZ_MASK)) {
		cu = inc_dec(xgkr, COE_COZ, coz_req);
		min_max_updated(xgkr, COE_COZ, cu);
	}

	if (com_req && !(xgkr->ld_status & COM1_MASK)) {
		cu = inc_dec(xgkr, COE_COM, com_req);
		min_max_updated(xgkr, COE_COM, cu);
	}

	if (old_status != xgkr->ld_status)
		ld_coe_status(xgkr);
}

static void preset(struct xgkr_params *xgkr)
{
	/* These are all MAX values from the IEEE802.3 perspective. */
	xgkr->ratio_pst1q = POST_COE_MAX;
	xgkr->adpt_eq = ZERO_COE_MAX;
	xgkr->ratio_preq = PRE_COE_MAX;

	tune_tecr(xgkr);
	xgkr->ld_status &= ~(COP1_MASK | COZ_MASK | COM1_MASK);
	xgkr->ld_status |= COE_MAX << COP1_SHIFT |
			   COE_MAX << COZ_SHIFT |
			   COE_MAX << COM1_SHIFT;
	ld_coe_status(xgkr);
}

static void train_local_tx(struct xgkr_params *xgkr)
{
	int request, old_ld_status;

	/* get request from LP */
	request = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_CU) &
			      (LD_ALL_MASK);

	old_ld_status = xgkr->ld_status;

	/* IEEE802.3-2008, 72.6.10.2.5
	 * Ensure we always go to NOT UDPATED for status reporting in
	 * response to HOLD requests.
	 * IEEE802.3-2008, 72.6.10.2.3.1/2
	 * ... but only if PRESET/INITIALIZE are not active to ensure
	 * we keep status until they are released.
	 */
	if (!(request & (PRESET_MASK | INIT_MASK))) {
		if (!(request & COP1_MASK))
			xgkr->ld_status &= ~COP1_MASK;

		if (!(request & COZ_MASK))
			xgkr->ld_status &= ~COZ_MASK;

		if (!(request & COM1_MASK))
			xgkr->ld_status &= ~COM1_MASK;

		if (old_ld_status != xgkr->ld_status)
			ld_coe_status(xgkr);
	}

	/* As soon as the LP shows ready, no need to do any more updates. */
	if (check_rx(xgkr)) {
		/* LP receiver is ready */
		if (xgkr->ld_status & (COP1_MASK | COZ_MASK | COM1_MASK)) {
			xgkr->ld_status &= ~(COP1_MASK | COZ_MASK | COM1_MASK);
			ld_coe_status(xgkr);
		}
	} else {
		/* IEEE802.3-2008, 72.6.10.2.3.1/2
		 * only act on PRESET/INITIALIZE if all status is NOT UPDATED.
		 */
		if (request & (PRESET_MASK | INIT_MASK)) {
			if (!(xgkr->ld_status &
			      (COP1_MASK | COZ_MASK | COM1_MASK))) {
				if (request & PRESET_MASK)
					preset(xgkr);

				if (request & INIT_MASK)
					initialize(xgkr);
			}
		}

		/* LP Coefficient are not in HOLD */
		if (request & REQUEST_MASK)
			check_request(xgkr, request & REQUEST_MASK);
	}
}

static void xgkr_start_train(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	struct tx_condition *tx_c = &xgkr->tx_c;
	int val = 0, i, j;
	int lt_state;
	unsigned long dead_line;
	int lp_rx_ready, tx_training_complete;
	u32 lt_timeout = 500;

	init_xgkr(xgkr, 0);
	
	start_lt(xgkr);
	
	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		lt_timeout = 2000;
	}

	for (i = 0; i < 2;) {
		
		dead_line = jiffies + msecs_to_jiffies(lt_timeout);
		
		while (time_before(jiffies, dead_line)) {

			val = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD,
					   lt_KR_PMD_STATUS);

			if (val & TRAIN_FAIL) {
				/* LT failed already, reset lane to avoid
				 * it run into hanging, then start LT again.
				 */
				if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
					/* Reset only the Master Lane */
					if (xgkr->idx == MASTER_LANE)
						xgkr->srds->reset_lane(xgkr->reg_base);
				} else {
					xgkr->srds->reset_lane(xgkr->reg_base);
				}
				
				start_lt(xgkr);
			} else if ((val & PMD_STATUS_SUP_STAT) &&
				   (val & PMD_STATUS_FRAME_LOCK))
				break;
			usleep_range(100, 500);
		}

		if (!((val & PMD_STATUS_FRAME_LOCK) &&
		      (val & PMD_STATUS_SUP_STAT))) {
			i++;
			continue;
		}

		/* init process */
		lp_rx_ready = false;
		tx_training_complete = false;
		/* the LT should be finished in 500ms, failed or OK. */
		dead_line = jiffies + msecs_to_jiffies(lt_timeout);

		while (time_before(jiffies, dead_line)) {
			/* check if the LT is already failed */

			lt_state = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD,
						lt_KR_PMD_STATUS);

			if (lt_state & TRAIN_FAIL) {
				
				if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
					/* Reset only the Master Lane */
					if (xgkr->idx == MASTER_LANE)
						xgkr->srds->reset_lane(xgkr->reg_base);
				} else {
					xgkr->srds->reset_lane(xgkr->reg_base);
				}
				
				break;
			}

			lp_rx_ready = check_rx(xgkr);
			tx_training_complete = tx_c->tx_complete;

			if (lp_rx_ready && tx_training_complete)
				break;

			if (!lp_rx_ready)
				train_local_tx(xgkr);

			if (!tx_training_complete)
				train_remote_tx(xgkr);

			usleep_range(100, 500);
		}

		i++;
		/* check LT result */
		if (is_link_training_fail(xgkr)) {
			init_xgkr(xgkr, 0);
			continue;
		} else {
			stop_lt(xgkr);
			xgkr->state = TRAINED;
			
			switch (xgkr_inst->bp_mode)
			{
			case PHY_BACKPLANE_10GBASE_KR:
				if (phydev->attached_dev == NULL)
					dev_info(&phydev->mdio.dev, "10GBase-KR link trained (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)\n",
							xgkr->tuned_ratio_preq, xgkr->tuned_ratio_pst1q, xgkr->tuned_adpt_eq);
				else
					dev_info(&phydev->mdio.dev, "%s %s: 10GBase-KR link trained (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)\n",
							dev_driver_string(phydev->attached_dev->dev.parent), 
							dev_name(phydev->attached_dev->dev.parent),
							xgkr->tuned_ratio_preq, xgkr->tuned_ratio_pst1q, xgkr->tuned_adpt_eq);
				break;
				
			case PHY_BACKPLANE_40GBASE_KR:
				if (xgkr->idx == xgkr_inst->phy_lanes - 1) {
					if (phydev->attached_dev == NULL)
						dev_info(&phydev->mdio.dev, "40GBase-KR link trained at lanes Tx equalization:\n");
					else
						dev_info(&phydev->mdio.dev, "%s %s: 40GBase-KR link trained at lanes Tx equalization:\n",
								dev_driver_string(phydev->attached_dev->dev.parent), 
								dev_name(phydev->attached_dev->dev.parent));

					for (j = 0; j < xgkr_inst->phy_lanes; j++) {
						if (phydev->attached_dev == NULL)
							dev_info(&phydev->mdio.dev, "40GBase-KR Lane %d: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x\n",
									j, xgkr_inst->xgkr[j].tuned_ratio_preq, xgkr_inst->xgkr[j].tuned_ratio_pst1q, xgkr_inst->xgkr[j].tuned_adpt_eq);
						else
							dev_info(&phydev->mdio.dev, "%s %s: 40GBase-KR Lane %d: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x\n",
									dev_driver_string(phydev->attached_dev->dev.parent),
									dev_name(phydev->attached_dev->dev.parent),
									j, xgkr_inst->xgkr[j].tuned_ratio_preq, xgkr_inst->xgkr[j].tuned_ratio_pst1q, xgkr_inst->xgkr[j].tuned_adpt_eq);
					}
				}
				break;
			}

			break;
		}
	}
}

static void xgkr_request_restart_an(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (time_before(jiffies, xgkr->rt_time))
		return;
	
	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		init_xgkr(xgkr, 0);  
		reset_lt(xgkr);
		xgkr->state = DETECTING_LP;
		start_xgkr_an(xgkr);
		start_xgkr_state_machine(&xgkr->xgkr_wk);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			init_xgkr(&xgkr_inst->xgkr[i], 0);
			reset_lt(&xgkr_inst->xgkr[i]);
			xgkr_inst->xgkr[i].state = DETECTING_LP;
		}
		//Start AN only for Master Lane
		start_xgkr_an(&xgkr_inst->xgkr[MASTER_LANE]);
		//start state machine
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk);
		}
		break;
	}
	
	xgkr->rt_time = jiffies + msecs_to_jiffies(XGKR_DENY_RT_INTERVAL);
}

static void xgkr_state_machine(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct xgkr_params *xgkr = container_of(dwork,
						  struct xgkr_params, xgkr_wk);
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int an_state;
	bool start_train = false;
	bool all_lanes_trained = false;
	int i;

	if (!xgkr_inst->aneg_done) {
		start_xgkr_state_machine(&xgkr->xgkr_wk);
		return;
	}

	mutex_lock(&phydev->lock);
	
	switch (xgkr->state) {
	case DETECTING_LP:

		switch (xgkr_inst->bp_mode)
		{
		case PHY_BACKPLANE_1000BASE_KX:
			dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
			break;

		case PHY_BACKPLANE_10GBASE_KR:
			an_state = xgkr_phy_read_mmd(xgkr, MDIO_MMD_AN, g_an_BP_STAT);
			if (an_state & KR_AN_MASK_10G) {
				//AN acquired: Train the lane
				xgkr->an_wait_count = 0;
				start_train = true;
			} else {
				//AN lost or not yet acquired
				if (!is_link_up(phydev)) {
					//Link is down: restart training
					xgkr->an_wait_count = 0;
					xgkr_request_restart_an(xgkr);
				} else {
					//Link is up: wait few iterations for AN to be acquired
					if (xgkr->an_wait_count >= XGKR_AN_WAIT_ITERATIONS) {
						xgkr->an_wait_count = 0;
						xgkr_request_restart_an(xgkr);
					} else {
						xgkr->an_wait_count++;
					}
				}
			}
			break;

		case PHY_BACKPLANE_40GBASE_KR:
			//Check AN state only on Master Lane
			an_state = xgkr_phy_read_mmd(&xgkr_inst->xgkr[MASTER_LANE], MDIO_MMD_AN, g_an_BP_STAT);
			if (an_state & KR_AN_MASK_40G) {
				//AN acquired: Train all lanes in order starting with Master Lane
				xgkr->an_wait_count = 0;
				if (xgkr->idx == MASTER_LANE) {
					start_train = true;
				}
				else if (xgkr_inst->xgkr[xgkr->idx - 1].state == TRAINED) {
					start_train = true;
				}
			} else {
				//AN lost or not yet acquired
				if (!is_link_up(phydev)) {
					//Link is down: restart training
					xgkr->an_wait_count = 0;
					xgkr_request_restart_an(xgkr);
				} else {
					//Link is up: wait few iterations for AN to be acquired
					if (xgkr->an_wait_count >= XGKR_AN_WAIT_ITERATIONS) {
						xgkr->an_wait_count = 0;
						xgkr_request_restart_an(xgkr);
					} else {
						xgkr->an_wait_count++;
					}
				}
			}
			break;
		}
		break;

	case TRAINED:
		if (!is_link_up(phydev)) {
			switch (xgkr_inst->bp_mode)
			{
			case PHY_BACKPLANE_1000BASE_KX:
				dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
				break;

			case PHY_BACKPLANE_10GBASE_KR:
				dev_info(&phydev->mdio.dev, "Detect hotplug, restart training\n");
				xgkr_request_restart_an(xgkr);
				break;

			case PHY_BACKPLANE_40GBASE_KR:
				if (xgkr->idx == MASTER_LANE) {
					//check if all lanes are trained only on Master Lane
					all_lanes_trained = true;
					for (i = 0; i < xgkr_inst->phy_lanes; i++) {
						if (xgkr_inst->xgkr[i].state != TRAINED) {
							all_lanes_trained = false;
							break;
						}
					}
					if (all_lanes_trained) {
						dev_info(&phydev->mdio.dev, "Detect hotplug, restart training\n");
						xgkr_request_restart_an(xgkr);
					}
				}
				break;
			}
		}
		break;
	}

	if (start_train) {
		xgkr_start_train(xgkr);
	}

	mutex_unlock(&phydev->lock);
	start_xgkr_state_machine(&xgkr->xgkr_wk);
}

static int fsl_backplane_probe(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst;
	struct device_node *phy_node, *lane_node;
	struct resource res_lane;
	struct serdes_access *srds = NULL;
	int serdes_type;
	const char *st;
	const char *bm;
	int ret, i, phy_lanes;
	int bp_mode;
	u32 lane_base_addr[MAX_PHY_LANES_NO], lane_memmap_size;

	phy_node = phydev->mdio.dev.of_node;
	if (!phy_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	bp_mode = of_property_read_string(phy_node, "backplane-mode", &bm);
	if (bp_mode < 0)
		return -EINVAL;

	phy_lanes = 1;
	if (!strcasecmp(bm, "1000base-kx")) {
		bp_mode = PHY_BACKPLANE_1000BASE_KX;
	} else if (!strcasecmp(bm, "10gbase-kr")) {
		bp_mode = PHY_BACKPLANE_10GBASE_KR;
	} else if (!strcasecmp(bm, "40gbase-kr")) {
		bp_mode = PHY_BACKPLANE_40GBASE_KR;
		phy_lanes = 4;
	} else {
		dev_err(&phydev->mdio.dev, "Unknown backplane-mode\n");
		return -EINVAL;
	}

	lane_node = of_parse_phandle(phy_node, "fsl,lane-handle", 0);
	if (!lane_node) {
		dev_err(&phydev->mdio.dev, "parse fsl,lane-handle failed\n");
		return -EINVAL;
	}

	ret = of_property_read_string(lane_node, "compatible", &st);
	if (ret < 0) {
		//assume SERDES-10G if compatible property is not specified
		serdes_type = SERDES_10G;
	}
	else if (!strcasecmp(st, "fsl,serdes-10g")) {
		serdes_type = SERDES_10G;
	} else if (!strcasecmp(st, "fsl,serdes-28g")) {
		serdes_type = SERDES_28G;
	} else {
		dev_err(&phydev->mdio.dev, "Unknown serdes-type\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(lane_node, 0, &res_lane);
	if (ret) {
		dev_err(&phydev->mdio.dev, "could not obtain memory map\n");
		return ret;
	}

	of_node_put(lane_node);
	ret = of_property_read_u32_array(phy_node, "fsl,lane-reg",
					 (u32 *)lane_base_addr, phy_lanes);
	if (ret) {
		dev_err(&phydev->mdio.dev, "could not get fsl,lane-reg\n");
		return -EINVAL;
	}

	switch (serdes_type)
	{
	case SERDES_10G:
		setup_an_lt_ls();
		srds = setup_serdes_access_10g();
		break;

	case SERDES_28G:
		setup_an_lt_lx();
		srds = setup_serdes_access_28g();
		break;

	default:
		dev_err(&phydev->mdio.dev, "Unsupported serdes-type\n");
		return -EINVAL;
	}

	if (!srds) {
		dev_err(&phydev->mdio.dev, "Unsupported serdes-type\n");
		return -EINVAL;
	}

	srds->serdes_type = serdes_type;
	srds->is_little_endian = of_property_read_bool(lane_node, "little-endian");

	if (srds->is_little_endian) {
		srds->ioread32 = le_ioread32;
		srds->iowrite32 = le_iowrite32;
	} else {
		srds->ioread32 = be_ioread32;
		srds->iowrite32 = be_iowrite32;
	}

	xgkr_inst = devm_kzalloc(&phydev->mdio.dev,
				 sizeof(*xgkr_inst), GFP_KERNEL);
	if (!xgkr_inst)
		return -ENOMEM;

	xgkr_inst->phy_lanes = phy_lanes;
	xgkr_inst->bp_mode = bp_mode;
	mutex_init(&xgkr_inst->phy_lock);

	lane_memmap_size = srds->get_lane_memmap_size();
	
	for (i = 0; i < phy_lanes; i++) {
		xgkr_inst->xgkr[i].idx = i;
		xgkr_inst->xgkr[i].phydev = phydev;
		xgkr_inst->xgkr[i].srds = srds;
		xgkr_inst->xgkr[i].reg_base = devm_ioremap_nocache(&phydev->mdio.dev,
						    res_lane.start + lane_base_addr[i],
						    lane_memmap_size);
		if (!xgkr_inst->xgkr[i].reg_base) {
			dev_err(&phydev->mdio.dev, "ioremap_nocache failed\n");
			return -ENOMEM;
		}
		xgkr_inst->xgkr[i].rt_time = jiffies + msecs_to_jiffies(XGKR_DENY_RT_INTERVAL);
	}

	phydev->priv = xgkr_inst;

	switch (bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		phydev->speed = SPEED_1000;
		/* configure the lane for 1000BASE-KX */
		srds->lane_set_1gkx(xgkr_inst->xgkr[SINGLE_LANE].reg_base);
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		phydev->speed = SPEED_10000;
		INIT_DELAYED_WORK(&xgkr_inst->xgkr[SINGLE_LANE].xgkr_wk, xgkr_state_machine);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		phydev->speed = SPEED_40000;
		for (i = 0; i < phy_lanes; i++)
			INIT_DELAYED_WORK(&xgkr_inst->xgkr[i].xgkr_wk, xgkr_state_machine);
		break;
	}

	return 0;
}

static int fsl_backplane_aneg_done(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}
	
	xgkr_inst->aneg_done = true;

	return 1;
}

static int fsl_backplane_config_aneg(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	switch (phydev->speed)
	{
	case SPEED_1000:
		phydev->supported |= SUPPORTED_1000baseKX_Full;
		start_1gkx_an(phydev);
		break;

	case SPEED_10000:
		phydev->supported |= SUPPORTED_10000baseKR_Full;
		reset_lt(&xgkr_inst->xgkr[SINGLE_LANE]);
		start_xgkr_an(&xgkr_inst->xgkr[SINGLE_LANE]);
		/* start state machine*/
		start_xgkr_state_machine(&xgkr_inst->xgkr[SINGLE_LANE].xgkr_wk);
		break;

	case SPEED_40000:
		phydev->supported |= SUPPORTED_40000baseKR4_Full;
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			reset_lt(&xgkr_inst->xgkr[i]);
		}
		//Start AN only for Master Lane
		start_xgkr_an(&xgkr_inst->xgkr[MASTER_LANE]);
		/* start state machine*/
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk);
		}
		
		break;
	}

	phydev->advertising = phydev->supported;
	phydev->duplex = 1;

	return 0;
}

static int fsl_backplane_suspend(struct phy_device *phydev)
{
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	if (phydev->speed == SPEED_10000 || phydev->speed == SPEED_40000) {
		struct xgkr_phy_data *xgkr_inst = phydev->priv;

		for (i = 0; i < xgkr_inst->phy_lanes; i++)
			cancel_delayed_work_sync(&xgkr_inst->xgkr[i].xgkr_wk);
	}
	return 0;
}

static int fsl_backplane_resume(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	if (phydev->speed == SPEED_10000 || phydev->speed == SPEED_40000) {
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			init_xgkr(&xgkr_inst->xgkr[i], 1);
			start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk);
		}
	}
	return 0;
}

static int fsl_backplane_read_status(struct phy_device *phydev)
{
	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	if (is_link_up(phydev))
		phydev->link = 1;
	else
		phydev->link = 0;

	return 0;
}

static int fsl_backplane_match_phy_device(struct phy_device *phydev)
{
	struct device_node *phy_node, *lane_node;
	const char *st;
	int serdes_type, i, ret;
	const int num_ids = ARRAY_SIZE(phydev->c45_ids.device_ids);

	if (!phydev->mdio.dev.of_node) {
		return 0;
	}

	//	 WORKAROUND:
	// Required for LX2 devices
	// where PHY ID cannot be verified in PCS
	// because PCS Device Identifier Upper and Lower registers are hidden
	// and always return 0 when they are read:
	// 2  02 	Device_ID0  RO 		Bits 15:0 	0
	// val = phy_read_mmd(phydev, MDIO_MMD_PCS, 0x2);
	// 3  03 	Device_ID1  RO 		Bits 31:16 	0
	// val = phy_read_mmd(phydev, MDIO_MMD_PCS, 0x3);
	//
	// To be removed: After the issue will be fixed on LX2 devices

	if (!phydev->is_c45)
		return 0;

	phy_node = phydev->mdio.dev.of_node;

	lane_node = of_parse_phandle(phy_node, "fsl,lane-handle", 0);
	if (!lane_node) {
		dev_err(&phydev->mdio.dev, "parse fsl,lane-handle failed\n");
		return 0;
	}

	ret = of_property_read_string(lane_node, "compatible", &st);
	if (ret < 0) {
		//assume SERDES-10G if compatible property is not specified
		serdes_type = SERDES_10G;
	}
	else if (!strcasecmp(st, "fsl,serdes-10g")) {
		serdes_type = SERDES_10G;
	} else if (!strcasecmp(st, "fsl,serdes-28g")) {
		serdes_type = SERDES_28G;
	} else {
		dev_err(&phydev->mdio.dev, "Unknown serdes-type\n");
		return 0;
	}

	if (serdes_type == SERDES_10G) {
		//On LS devices we must find the c45 device with correct PHY ID
		//Implementation similar with the one existent in phy_device: @function: phy_bus_match
		for (i = 1; i < num_ids; i++) {
			if (!(phydev->c45_ids.devices_in_package & (1 << i)))
				continue;

			if ((PCS_PHY_DEVICE_ID & PCS_PHY_DEVICE_ID_MASK) ==
				(phydev->c45_ids.device_ids[i] & PCS_PHY_DEVICE_ID_MASK))
			{
				return 1;
			}
		}
		return 0;
	}

	//On LX devices we cannot verify PHY ID
	//so we are happy only with preliminary verifications already made: mdio.dev.of_node and is_c45
	//because we already filtered other undesired devices: non clause 45

	return 1;
}

static struct phy_driver fsl_backplane_driver[] = {
	{
	.phy_id		= PCS_PHY_DEVICE_ID,
	.name		= "Freescale Backplane",
	.phy_id_mask	= PCS_PHY_DEVICE_ID_MASK,
	.features	= SUPPORTED_Backplane | SUPPORTED_Autoneg |
			  SUPPORTED_MII,
	.probe          = fsl_backplane_probe,
	.aneg_done      = fsl_backplane_aneg_done,
	.config_aneg	= fsl_backplane_config_aneg,
	.read_status	= fsl_backplane_read_status,
	.suspend	= fsl_backplane_suspend,
	.resume		= fsl_backplane_resume,
	.match_phy_device = fsl_backplane_match_phy_device,
	},
};

module_phy_driver(fsl_backplane_driver);

static struct mdio_device_id __maybe_unused freescale_tbl[] = {
	{ PCS_PHY_DEVICE_ID, PCS_PHY_DEVICE_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, freescale_tbl);

MODULE_DESCRIPTION("Freescale Backplane driver");
MODULE_AUTHOR("Shaohui Xie <Shaohui.Xie@freescale.com>");
MODULE_LICENSE("GPL v2");
