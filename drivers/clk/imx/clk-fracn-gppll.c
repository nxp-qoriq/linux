// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <asm/div64.h>

#include "clk.h"
#include "clk-pll.h"

#define PLL_CTRL		0x0
#define CLKMUX_BYPASS		BIT(2)
#define CLKMUX_EN		BIT(1)
#define POWERUP_MASK		BIT(0)

#define PLL_ANA_PRG		0x10
#define PLL_SPREAD_SPECTRUM	0x30

#define PLL_NUMERATOR		0x40
#define PLL_MFN_MASK		GENMASK(31, 2)

#define PLL_DENOMINATOR		0x50
#define PLL_MFD_MASK		GENMASK(29, 0)

#define PLL_DIV			0x60
#define PLL_MFI_MASK		GENMASK(24, 16)
#define PLL_RDIV_MASK		GENMASK(15, 13)
#define PLL_ODIV_MASK		GENMASK(7, 0)

#define PLL_DFS_CTRL(x)		(0x70 + (x) * 0x10)

#define PLL_STATUS		0xF0
#define LOCK_STATUS		BIT(0)

#define DFS_STATUS		0xF4

#define LOCK_TIMEOUT_US		200

#define PLL_FRACN_GP(_rate, _mfi, _mfn, _mfd, _rdiv, _odiv)	\
	{							\
		.rate	=	(_rate),			\
		.mfi	=	(_mfi),				\
		.mfn	=	(_mfn),				\
		.mfd	=	(_mfd),				\
		.rdiv	=	(_rdiv),			\
		.odiv	=	(_odiv),			\
	}


/* MFN : Numerator of the fractional part of divider (30-bits, signed) */
#define MFN_MAX_VALUE			((s32)(GENMASK(29, 0) >> 1))
#define MFN_MIN_VALUE			((s32)(-MFN_MAX_VALUE - 1))

struct clk_fracn_gppll {
	struct clk_hw			hw;
	void __iomem			*base;
	const struct imx_fracn_gppll_rate_table *rate_table;
	u32				orig_mfd;
	u32				orig_mfi;
	u32				orig_odiv;
	u32				orig_rdiv;
	s32				orig_mfn;
	struct clk_imx_pll		imx_pll;
	int rate_count;
};

/*
 * Fvco = Fref * (MFI + MFN / MFD)
 * Fout = Fvco / (rdiv * odiv)
 */
static const struct imx_fracn_gppll_rate_table fracn_tbl[] = {
	PLL_FRACN_GP(650000000U, 162, 50, 100, 0, 6),
	PLL_FRACN_GP(594000000U, 198, 0, 1, 0, 8),
	PLL_FRACN_GP(560000000U, 140, 0, 1, 0, 6),
	PLL_FRACN_GP(498000000U, 166, 0, 1, 0, 8),
	PLL_FRACN_GP(484000000U, 121, 0, 1, 0, 6),
	PLL_FRACN_GP(445333333U, 167, 0, 1, 0, 9),
	PLL_FRACN_GP(400000000U, 200, 0, 1, 0, 12),
	PLL_FRACN_GP(393216000U, 163, 1680000, 2000000, 0, 10),
	PLL_FRACN_GP(300000000U, 150, 0, 1, 0, 12)
};

struct imx_fracn_gppll_clk imx_fracn_gppll = {
	.rate_table = fracn_tbl,
	.rate_count = ARRAY_SIZE(fracn_tbl),
};
EXPORT_SYMBOL_GPL(imx_fracn_gppll);

static inline struct clk_fracn_gppll *to_clk_fracn_gppll(struct clk_hw *hw)
{
	return container_of(hw, struct clk_fracn_gppll, hw);
}

static const struct imx_fracn_gppll_rate_table *
imx_get_pll_settings(struct clk_fracn_gppll *pll, unsigned long rate)
{
	const struct imx_fracn_gppll_rate_table *rate_table = pll->rate_table;
	int i;

	for (i = 0; i < pll->rate_count; i++)
		if (rate == rate_table[i].rate)
			return &rate_table[i];

	return NULL;
}

static long clk_fracn_gppll_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *prate)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	const struct imx_fracn_gppll_rate_table *rate_table = pll->rate_table;
	int i;

	/* Assuming rate_table is in descending order */
	for (i = 0; i < pll->rate_count; i++)
		if (rate >= rate_table[i].rate)
			return rate_table[i].rate;

	/* return minimum supported value */
	return rate_table[pll->rate_count - 1].rate;
}

/* This function calculates the actual rate based on the configured PLL registers */
static unsigned long __clk_fracn_gppll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	u32 pll_numerator, pll_denominator, pll_div;
	u32 mfi, mfd, rdiv, odiv;
	s32 mfn;
	u64 fvco = parent_rate;

	pll_numerator = readl_relaxed(pll->base + PLL_NUMERATOR);

	/* Numerator is 30-bits signed value */
	mfn = sign_extend32(FIELD_GET(PLL_MFN_MASK, pll_numerator), 29);

	pll_denominator = readl_relaxed(pll->base + PLL_DENOMINATOR);
	mfd = FIELD_GET(PLL_MFD_MASK, pll_denominator);

	pll_div = readl_relaxed(pll->base + PLL_DIV);
	mfi = FIELD_GET(PLL_MFI_MASK, pll_div);

	rdiv = FIELD_GET(PLL_RDIV_MASK, pll_div);
	odiv = FIELD_GET(PLL_ODIV_MASK, pll_div);

	rdiv = rdiv + 1;

	switch (odiv) {
	case 0:
		odiv = 2;
		break;
	case 1:
		odiv = 3;
		break;
	default:
		break;
	}

	/* Fvco = Fref * (MFI + MFN / MFD) */
	fvco *= (mfi * mfd + mfn);
	do_div(fvco, mfd * rdiv * odiv);

	return (unsigned long)fvco;
}

static unsigned long clk_fracn_gppll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	const struct imx_fracn_gppll_rate_table *rate_table = pll->rate_table;
	u32 pll_numerator, pll_denominator, pll_div;
	u32 mfi, mfn, mfd, rdiv, odiv;
	long rate = 0;
	int i;

	pll_numerator = readl_relaxed(pll->base + PLL_NUMERATOR);
	mfn = FIELD_GET(PLL_MFN_MASK, pll_numerator);

	pll_denominator = readl_relaxed(pll->base + PLL_DENOMINATOR);
	mfd = FIELD_GET(PLL_MFD_MASK, pll_denominator);

	pll_div = readl_relaxed(pll->base + PLL_DIV);
	mfi = FIELD_GET(PLL_MFI_MASK, pll_div);

	rdiv = FIELD_GET(PLL_RDIV_MASK, pll_div);
	odiv = FIELD_GET(PLL_ODIV_MASK, pll_div);

	/*
	 * Sometimes, the recalculated rate has deviation due to
	 * the frac part. So find the accurate pll rate from the table
	 * first, if no match rate in the table, use the rate calculated
	 * from the equation below.
	 */
	for (i = 0; i < pll->rate_count; i++) {
		if (rate_table[i].mfn == mfn && rate_table[i].mfi == mfi &&
		    rate_table[i].mfd == mfd && rate_table[i].rdiv == rdiv &&
		    rate_table[i].odiv == odiv)
			rate = rate_table[i].rate;
	}

	return rate ? (unsigned long) rate : __clk_fracn_gppll_recalc_rate(hw, parent_rate);
}

static int clk_fracn_gppll_wait_lock(struct clk_fracn_gppll *pll)
{
	u32 val;

	return readl_poll_timeout(pll->base + PLL_STATUS, val,
				  val & LOCK_STATUS, 0, LOCK_TIMEOUT_US);
}

static int clk_fracn_gppll_set_rate(struct clk_hw *hw, unsigned long drate,
				    unsigned long prate)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	const struct imx_fracn_gppll_rate_table *rate;
	u32 tmp, pll_div, ana_mfn;
	int ret;

	rate = imx_get_pll_settings(pll, drate);
	if (!rate) {
		pr_err("%s: Invalid rate : %lu for pll clk %s\n", __func__,
			drate, clk_hw_get_name(hw));
		return -EINVAL;
	}

	/* Disable output */
	tmp = readl_relaxed(pll->base + PLL_CTRL);
	tmp &= ~CLKMUX_EN;
	writel_relaxed(tmp, pll->base + PLL_CTRL);

	/* Power Down */
	tmp &= ~POWERUP_MASK;
	writel_relaxed(tmp, pll->base + PLL_CTRL);

	/* Disable BYPASS */
	tmp &= ~CLKMUX_BYPASS;
	writel_relaxed(tmp, pll->base + PLL_CTRL);

	pll_div = FIELD_PREP(PLL_RDIV_MASK, rate->rdiv) | rate->odiv |
		FIELD_PREP(PLL_MFI_MASK, rate->mfi);
	writel_relaxed(pll_div, pll->base + PLL_DIV);
	writel_relaxed(rate->mfd, pll->base + PLL_DENOMINATOR);
	writel_relaxed(FIELD_PREP(PLL_MFN_MASK, rate->mfn), pll->base + PLL_NUMERATOR);

	/* Wait for 5us according to fracn mode pll doc */
	udelay(5);

	/* Enable Powerup */
	tmp |= POWERUP_MASK;
	writel_relaxed(tmp, pll->base + PLL_CTRL);

	/* Wait Lock */
	ret = clk_fracn_gppll_wait_lock(pll);
	if (ret)
		return ret;

	/* Enable output */
	tmp |= CLKMUX_EN;
	writel_relaxed(tmp, pll->base + PLL_CTRL);

	ana_mfn = readl_relaxed(pll->base + PLL_STATUS);
	ana_mfn = FIELD_GET(PLL_MFN_MASK, ana_mfn);

	WARN(ana_mfn != rate->mfn, "ana_mfn != rate->mfn\n");

	return 0;
}

static int clk_fracn_gppll_prepare(struct clk_hw *hw)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	u32 val;
	int ret;

	val = readl_relaxed(pll->base + PLL_CTRL);
	if (val & POWERUP_MASK)
		return 0;

	val |= CLKMUX_BYPASS;
	writel_relaxed(val, pll->base + PLL_CTRL);

	val |= POWERUP_MASK;
	writel_relaxed(val, pll->base + PLL_CTRL);

	val |= CLKMUX_EN;
	writel_relaxed(val, pll->base + PLL_CTRL);

	ret = clk_fracn_gppll_wait_lock(pll);
	if (ret)
		return ret;

	val &= ~CLKMUX_BYPASS;
	writel_relaxed(val, pll->base + PLL_CTRL);

	return 0;
}

static int clk_fracn_gppll_is_prepared(struct clk_hw *hw)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CTRL);

	return (val & POWERUP_MASK) ? 1 : 0;
}

static void clk_fracn_gppll_unprepare(struct clk_hw *hw)
{
	struct clk_fracn_gppll *pll = to_clk_fracn_gppll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CTRL);
	val &= ~POWERUP_MASK;
	writel_relaxed(val, pll->base + PLL_CTRL);
}

static const struct clk_ops clk_fracn_gppll_ops = {
	.prepare	= clk_fracn_gppll_prepare,
	.unprepare	= clk_fracn_gppll_unprepare,
	.is_prepared	= clk_fracn_gppll_is_prepared,
	.recalc_rate	= clk_fracn_gppll_recalc_rate,
	.round_rate	= clk_fracn_gppll_round_rate,
	.set_rate	= clk_fracn_gppll_set_rate,
};

/* This function fetches the original PLL parameters to use
 * them later for ppb adjustment
 */
static void imx_fracn_gppll_init(struct clk_imx_pll *pll)
{
	struct clk_fracn_gppll *fracn_pll;
	u32 pll_numerator, pll_denominator, pll_div, mfn;

	fracn_pll = (struct clk_fracn_gppll *) container_of(pll,
					struct clk_fracn_gppll, imx_pll);

	pll_numerator = readl_relaxed(fracn_pll->base + PLL_NUMERATOR);
	mfn = FIELD_GET(PLL_MFN_MASK, pll_numerator);

	/* Numerator is 30-bits signed value */
	fracn_pll->orig_mfn = sign_extend32(mfn, 29);

	pll_denominator = readl_relaxed(fracn_pll->base + PLL_DENOMINATOR);
	fracn_pll->orig_mfd = FIELD_GET(PLL_MFD_MASK, pll_denominator);

	pll_div = readl_relaxed(fracn_pll->base + PLL_DIV);
	fracn_pll->orig_mfi = FIELD_GET(PLL_MFI_MASK, pll_div);

	fracn_pll->orig_rdiv = FIELD_GET(PLL_RDIV_MASK, pll_div);
	fracn_pll->orig_odiv = FIELD_GET(PLL_ODIV_MASK, pll_div);
}

/**
 * imx_fracn_gppll_adjust - Adjust the Audio pll by ppb.
 *
 * This function adjusts the audio pll by ppb (part per billion) and returns
 * the exact number of ppb adjusted.
 * The adjustment is done by only modifying the numerator of the fractional part
 * of the audio PLL.
 * Since the pllout = (ref * (mfi + mfn/mfd)) / (rdiv * odiv)
 * and the adjusted value is
 *    pllout_new = pllout * (1 + ppb/1e9) which equals:
 *    (ref * (mfi + mfn_new/mfd)) / (rdiv * odiv)
 * The new numerator (mfn_new) is calculated as following:
 *    mfn_new = (1e9 * mfn + (mfi * mfd + mfn) * ppb) / (1e9)
 */

static int imx_fracn_gppll_adjust(struct clk_imx_pll *pll, int *ppb)
{
	s64 temp64;
	s64 applied_ppb;
	struct clk_fracn_gppll *fracn_pll;
	int rc = IMX_CLK_PLL_SUCCESS;

	int req_ppb = *ppb;

	fracn_pll = (struct clk_fracn_gppll *) container_of(pll,
						struct clk_fracn_gppll, imx_pll);

	/* Calculate the new numerator value */
	temp64 = ((s64) fracn_pll->orig_mfn * 1000000000)
			+ ((s64) fracn_pll->orig_mfi * fracn_pll->orig_mfd + (s64) fracn_pll->orig_mfn) * req_ppb;

	temp64 = div_s64(temp64, 1000000000);

	/* Sanity check on the new numerator value:
	 * 	- the value is inside the 30-bits signed values range
	 *	- mfn/mfd should be in the range [-2, 2]
	 */
	if (temp64 > MFN_MAX_VALUE || temp64 < MFN_MIN_VALUE || (abs(temp64) > (2 * fracn_pll->orig_mfd))) {
		rc = -IMX_CLK_PLL_PREC_ERR;
		goto exit;
	}

	/* Write the PLL control settings with the new numerator */

	writel_relaxed(FIELD_PREP(PLL_MFN_MASK, temp64), fracn_pll->base + PLL_NUMERATOR);

	/* Calculate and return the actual applied ppb */
	applied_ppb = div64_s64((s64) (temp64 - fracn_pll->orig_mfn) * 1000000000,
			fracn_pll->orig_mfn + fracn_pll->orig_mfd * (s64) fracn_pll->orig_mfi);

	*ppb = (int) applied_ppb;

 exit:
	return rc;
}

static unsigned long imx_fracn_gppll_get_rate(struct clk_imx_pll *pll,
					   unsigned long parent_rate)
{
	struct clk_fracn_gppll *fracn_pll;

	fracn_pll = (struct clk_fracn_gppll *) container_of(pll,
						struct clk_fracn_gppll, imx_pll);

	return __clk_fracn_gppll_recalc_rate(&fracn_pll->hw, parent_rate);
}

static int imx_fracn_gppll_set_rate(struct clk_imx_pll *pll, unsigned long rate,
				 unsigned long parent_rate)
{
	struct clk_fracn_gppll *fracn_pll;
	int rc = IMX_CLK_PLL_SUCCESS;

	fracn_pll = (struct clk_fracn_gppll *) container_of(pll,
						struct clk_fracn_gppll, imx_pll);

	if (clk_fracn_gppll_set_rate(&fracn_pll->hw, rate, parent_rate) < 0)
		rc = -IMX_CLK_PLL_INVALID_PARAM;

	return rc;
}

static const struct clk_imx_pll_ops imx_clk_fracn_gppll_ops = {
	.set_rate	= imx_fracn_gppll_set_rate,
	.get_rate	= imx_fracn_gppll_get_rate,
	.adjust		= imx_fracn_gppll_adjust,
	.init		= imx_fracn_gppll_init,
};

struct clk_hw *imx_clk_fracn_gppll(const char *name, const char *parent_name, void __iomem *base,
				   const struct imx_fracn_gppll_clk *pll_clk)
{
	struct clk_fracn_gppll *pll;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.flags = pll_clk->flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &clk_fracn_gppll_ops;

	pll->base = base;
	pll->hw.init = &init;
	pll->rate_table = pll_clk->rate_table;
	pll->rate_count = pll_clk->rate_count;

	hw = &pll->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		pr_err("%s: failed to register pll %s %d\n", __func__, name, ret);
		kfree(pll);
		return ERR_PTR(ret);
	}

	pll->imx_pll.ops = &imx_clk_fracn_gppll_ops;

	if (imx_pll_register(&pll->imx_pll, name) < 0)
		pr_warn("%s: failed to register %s into imx pll\n", __func__, name);

	return hw;
}
EXPORT_SYMBOL_GPL(imx_clk_fracn_gppll);
