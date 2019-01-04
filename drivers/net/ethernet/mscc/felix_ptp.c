// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/* Felix switch PTP clock driver
 *
 * Copyright 2019 NXP
 */

#include "ocelot.h"

static int felix_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct ocelot *ocelot = container_of(ptp, struct ocelot, ptp_caps);
	u32 val, tod_ns, tod_sec_lsb, tod_sec_msb;

	val = ocelot_read_rix(ocelot, PTP_PIN_CFG, TOD_ACC_PIN);
	val &= ~(PTP_PIN_CFG_SYNC | PTP_PIN_CFG_ACTION_MASK | PTP_PIN_CFG_DOM);
	val |= PTP_PIN_CFG_ACTION(PTP_PIN_ACTION_SAVE);
	ocelot_write_rix(ocelot, val, PTP_PIN_CFG, TOD_ACC_PIN);

	tod_sec_msb = ocelot_read_rix(ocelot, PTP_TOD_SEC_MSB, TOD_ACC_PIN);
	tod_sec_lsb = ocelot_read_rix(ocelot, PTP_TOD_SEC_LSB, TOD_ACC_PIN);
	tod_ns = ocelot_read_rix(ocelot, PTP_TOD_NSEC, TOD_ACC_PIN);

	ts->tv_sec = ((u64)(tod_sec_msb & SEC_MSB_MASK) << 32) | tod_sec_lsb;
	ts->tv_nsec = tod_ns & NSEC_MASK;

	/* Deal with negative values -1 and -2 */
	if (ts->tv_nsec == 0x3fffffff) {
		ts->tv_sec -= 1;
		ts->tv_nsec = 999999999;
	} else if (ts->tv_nsec == 0x3ffffffe) {
		ts->tv_sec -= 1;
		ts->tv_nsec = 999999998;
	}

	return 0;
}

static int felix_ptp_settime(struct ptp_clock_info *ptp,
			     const struct timespec64 *ts)
{
	struct ocelot *ocelot = container_of(ptp, struct ocelot, ptp_caps);
	u32 val, tod_ns, tod_sec_lsb, tod_sec_msb;

	val = ocelot_read_rix(ocelot, PTP_PIN_CFG, TOD_ACC_PIN);
	val &= ~(PTP_PIN_CFG_SYNC | PTP_PIN_CFG_ACTION_MASK | PTP_PIN_CFG_DOM);
	val |= PTP_PIN_CFG_ACTION(PTP_PIN_ACTION_IDLE);
	ocelot_write_rix(ocelot, val, PTP_PIN_CFG, TOD_ACC_PIN);

	tod_ns = ts->tv_nsec & NSEC_MASK;
	tod_sec_msb = (u32)(ts->tv_sec >> 32) & SEC_MSB_MASK;
	tod_sec_lsb = (u32)ts->tv_sec;

	ocelot_write_rix(ocelot, tod_ns, PTP_TOD_NSEC, TOD_ACC_PIN);
	ocelot_write_rix(ocelot, tod_sec_msb, PTP_TOD_SEC_MSB, TOD_ACC_PIN);
	ocelot_write_rix(ocelot, tod_sec_lsb, PTP_TOD_SEC_LSB, TOD_ACC_PIN);

	val = ocelot_read_rix(ocelot, PTP_PIN_CFG, TOD_ACC_PIN);
	val &= ~(PTP_PIN_CFG_SYNC | PTP_PIN_CFG_ACTION_MASK | PTP_PIN_CFG_DOM);
	val |= PTP_PIN_CFG_ACTION(PTP_PIN_ACTION_LOAD);
	ocelot_write_rix(ocelot, val, PTP_PIN_CFG, TOD_ACC_PIN);

	return 0;
}

static int felix_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct timespec64 ts;
	struct timespec64 offset;

	offset = ns_to_timespec64(delta);

	felix_ptp_gettime(ptp, &ts);
	ts = timespec64_add(ts, offset);
	felix_ptp_settime(ptp, &ts);

	return 0;
}

static int felix_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct ocelot *ocelot = container_of(ptp, struct ocelot, ptp_caps);
	u64 adj;
	u32 reg_adjfreq = 0, reg_adjcfg = PTP_CLK_ADJ_ENA;

	if (!ppb)
		goto no_adj;

	if (ppb < 0) {
		reg_adjcfg |= PTP_CLK_ADJ_DIR;
		ppb = -ppb;
	}

	adj = PSEC_PER_SEC;
	do_div(adj, ppb);

	if (adj >= (1L << 30)) {
		reg_adjfreq |= PTP_CLK_ADJ_UNIT_NS;
		do_div(adj, 1000);
	}

	if (adj >= (1L << 30))
		goto no_adj;

	reg_adjfreq |= adj;

	ocelot_write(ocelot, reg_adjfreq, PTP_CLK_ADJ_FRQ);
	ocelot_write(ocelot, reg_adjcfg, PTP_CLK_ADJ_CFG);
	return 0;

no_adj:
	ocelot_write(ocelot, 0, PTP_CLK_ADJ_CFG);
	return 0;
}

static const struct ptp_clock_info felix_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "felix ptp clock",
	.max_adj	= 0x7fffffff,
	.n_alarm	= 0,
	.n_ext_ts	= 0,
	.n_per_out	= 0,
	.n_pins		= 0,
	.pps		= 0,
	.gettime64	= felix_ptp_gettime,
	.settime64	= felix_ptp_settime,
	.adjtime	= felix_ptp_adjtime,
	.adjfreq	= felix_ptp_adjfreq,
};

int felix_ptp_init(struct ocelot *ocelot)
{
	/* Reset and enable ptp clock */
	regmap_field_write(ocelot->regfields[PTP_MISC_CFG_ENA], 0);
	regmap_field_write(ocelot->regfields[PTP_SYS_CLK_CFG_PER_NS],
			   SYS_CLK_PER_NS);
	regmap_field_write(ocelot->regfields[PTP_SYS_CLK_CFG_PER_PS100],
			   SYS_CLK_PER_PS100);
	regmap_field_write(ocelot->regfields[PTP_MISC_CFG_ENA], 1);

	ocelot->ptp_caps = felix_ptp_caps;

	ocelot->clock = ptp_clock_register(&ocelot->ptp_caps, ocelot->dev);
	if (IS_ERR(ocelot->clock))
		return PTR_ERR(ocelot->clock);

	ocelot->phc_index = ptp_clock_index(ocelot->clock);
	return 0;
}

void felix_ptp_remove(struct ocelot *ocelot)
{
	ptp_clock_unregister(ocelot->clock);
}
