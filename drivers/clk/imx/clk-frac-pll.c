// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP.
 *
 * This driver supports the fractional plls found in the imx8m SOCs
 *
 * Documentation for this fractional pll can be found at:
 *   https://www.nxp.com/docs/en/reference-manual/IMX8MDQLQRM.pdf#page=834
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/bitfield.h>

#include "clk.h"
#include "clk-pll.h"

#define PLL_CFG0		0x0
#define PLL_CFG1		0x4

#define PLL_LOCK_STATUS		BIT(31)
#define PLL_PD_MASK		BIT(19)
#define PLL_BYPASS_MASK		BIT(14)
#define PLL_NEWDIV_VAL		BIT(12)
#define PLL_NEWDIV_ACK		BIT(11)
#define PLL_FRAC_DIV_MASK	GENMASK(30, 7)
#define PLL_INT_DIV_MASK	GENMASK(6, 0)
#define PLL_OUTPUT_DIV_MASK	GENMASK(4, 0)
#define PLL_FRAC_DENOM		0x1000000

#define PLL_FRAC_LOCK_TIMEOUT	10000
#define PLL_FRAC_ACK_TIMEOUT	500000

struct clk_frac_pll {
	u32 orig_divff;
	u32 orig_divfi;
	u32 orig_pllcfg0;
	struct clk_hw	hw;
	struct clk_imx_pll	imx_pll;
	void __iomem	*base;
};

#define to_clk_frac_pll(_hw) container_of(_hw, struct clk_frac_pll, hw)

static int clk_wait_lock(struct clk_frac_pll *pll)
{
	u32 val;

	return readl_poll_timeout(pll->base, val, val & PLL_LOCK_STATUS, 0,
					PLL_FRAC_LOCK_TIMEOUT);
}

static int clk_wait_ack(struct clk_frac_pll *pll)
{
	u32 val;

	/* return directly if the pll is in powerdown or in bypass */
	if (readl_relaxed(pll->base) & (PLL_PD_MASK | PLL_BYPASS_MASK))
		return 0;

	/* Wait for the pll's divfi and divff to be reloaded */
	return readl_poll_timeout(pll->base, val, val & PLL_NEWDIV_ACK, 0,
					PLL_FRAC_ACK_TIMEOUT);
}

static int clk_pll_prepare(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);

	return clk_wait_lock(pll);
}

static void clk_pll_unprepare(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= PLL_PD_MASK;
	writel_relaxed(val, pll->base + PLL_CFG0);
}

static int clk_pll_is_prepared(struct clk_hw *hw)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val;

	val = readl_relaxed(pll->base + PLL_CFG0);
	return (val & PLL_PD_MASK) ? 0 : 1;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val, divff, divfi, divq;
	u64 temp64 = parent_rate;
	u64 rate;

	val = readl_relaxed(pll->base + PLL_CFG0);
	divq = (FIELD_GET(PLL_OUTPUT_DIV_MASK, val) + 1) * 2;
	val = readl_relaxed(pll->base + PLL_CFG1);
	divff = FIELD_GET(PLL_FRAC_DIV_MASK, val);
	divfi = FIELD_GET(PLL_INT_DIV_MASK, val);

	temp64 *= 8;
	temp64 *= divff;
	do_div(temp64, PLL_FRAC_DENOM);
	do_div(temp64, divq);

	rate = parent_rate * 8 * (divfi + 1);
	do_div(rate, divq);
	rate += temp64;

	return rate;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long *prate)
{
	u64 parent_rate = *prate;
	u32 divff, divfi;
	u64 temp64;

	parent_rate *= 8;
	rate *= 2;
	temp64 = rate;
	do_div(temp64, parent_rate);
	divfi = temp64;
	temp64 = rate - divfi * parent_rate;
	temp64 *= PLL_FRAC_DENOM;
	do_div(temp64, parent_rate);
	divff = temp64;

	temp64 = parent_rate;
	temp64 *= divff;
	do_div(temp64, PLL_FRAC_DENOM);

	rate = parent_rate * divfi + temp64;

	return rate / 2;
}

/*
 * To simplify the clock calculation, we can keep the 'PLL_OUTPUT_VAL' at zero
 * (means the PLL output will be divided by 2). So the PLL output can use
 * the below formula:
 * pllout = parent_rate * 8 / 2 * DIVF_VAL;
 * where DIVF_VAL = 1 + DIVFI + DIVFF / 2^24.
 */
static int clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			    unsigned long parent_rate)
{
	struct clk_frac_pll *pll = to_clk_frac_pll(hw);
	u32 val, divfi, divff;
	u64 temp64;
	int ret;

	parent_rate *= 8;
	rate *= 2;
	divfi = rate / parent_rate;
	temp64 = parent_rate * divfi;
	temp64 = rate - temp64;
	temp64 *= PLL_FRAC_DENOM;
	do_div(temp64, parent_rate);
	divff = temp64;

	val = readl_relaxed(pll->base + PLL_CFG1);
	val &= ~(PLL_FRAC_DIV_MASK | PLL_INT_DIV_MASK);
	val |= (divff << 7) | (divfi - 1);
	writel_relaxed(val, pll->base + PLL_CFG1);

	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~0x1f;
	writel_relaxed(val, pll->base + PLL_CFG0);

	/* Set the NEV_DIV_VAL to reload the DIVFI and DIVFF */
	val = readl_relaxed(pll->base + PLL_CFG0);
	val |= PLL_NEWDIV_VAL;
	writel_relaxed(val, pll->base + PLL_CFG0);

	ret = clk_wait_ack(pll);

	/* clear the NEV_DIV_VAL */
	val = readl_relaxed(pll->base + PLL_CFG0);
	val &= ~PLL_NEWDIV_VAL;
	writel_relaxed(val, pll->base + PLL_CFG0);

	return ret;
}

static const struct clk_ops clk_frac_pll_ops = {
	.prepare	= clk_pll_prepare,
	.unprepare	= clk_pll_unprepare,
	.is_prepared	= clk_pll_is_prepared,
	.recalc_rate	= clk_pll_recalc_rate,
	.round_rate	= clk_pll_round_rate,
	.set_rate	= clk_pll_set_rate,
};

/* This function fetches the original PLL parameters to use
 * them later for ppb adjustment
 */
static void imx_frac_pll_init(struct clk_imx_pll *pll)
{
	struct clk_frac_pll *frac_pll;
	u32 val;

	frac_pll = (struct clk_frac_pll *) container_of(pll,
					struct clk_frac_pll, imx_pll);

	val = readl_relaxed(frac_pll->base + PLL_CFG1);

	frac_pll->orig_divff = (val >> 7) & PLL_FRAC_DIV_MASK;
	frac_pll->orig_divfi = val & PLL_INT_DIV_MASK;

	frac_pll->orig_pllcfg0 = readl_relaxed(frac_pll->base + PLL_CFG0);
}

/**
 * imx_frac_pll_adjust - Adjust the Audio pll by ppb.
 *
 * This function adjust the audio pll by ppb (part per billion) and returns
 * the exact number of ppb adjusted.
 * The adjustment is done by only modifying the Fractional Divide part
 * of the audio PLL.
 * Since the pllout = parent_rate * 8 / 2 * (1 + DIVFI + DIVFF / 2^24)
 * and the adjusted value is
 *    pllout_new = pllout * (1 + ppb/1e9) which equals:
 *    parent_rate * 8 / 2 * (1 + DIVFI + DIVFF_new / 2^24)
 * The new divff is calculated as the following:
 *    DIVFF_new = ((1 + DIVFI) * ppb * 2^24 + DIVFF * 1e9 + DIVFF * ppb) / (1e9)
 */

static int imx_frac_pll_adjust(struct clk_imx_pll *pll, int *ppb)
{
	u64 temp64;
	u32 val;
	s64 applied_ppb;
	struct clk_frac_pll *frac_pll;
	int rc = IMX_CLK_PLL_SUCCESS;

	int req_ppb = *ppb;

	frac_pll = (struct clk_frac_pll *) container_of(pll,
						struct clk_frac_pll, imx_pll);

	/*Calcultate the new PLL Numerator*/
	temp64 = ((u64) frac_pll->orig_divfi + 1) * PLL_FRAC_DENOM * req_ppb
			+ (u64) frac_pll->orig_divff * 1000000000
			+ (u64) frac_pll->orig_divff * req_ppb;

	do_div(temp64, 1000000000);

	if (temp64 >= PLL_FRAC_DENOM) {
		rc = -IMX_CLK_PLL_PREC_ERR;
		goto exit;
	}

	/* clear the NEW_DIV_VAL */
	val = frac_pll->orig_pllcfg0;
	val &= ~PLL_NEWDIV_VAL;
	writel_relaxed(val, frac_pll->base + PLL_CFG0);

	/* Write the PLL control settings with the new DIVFF
	 * NOTE: This sets the reserved bit (bit 31) to zero
	*/

	val = 0;
	val |= (((u32)temp64 << 7) | frac_pll->orig_divfi);
	writel_relaxed(val, frac_pll->base + PLL_CFG1);

	/* Set the NEW_DIV_VAL to reload the DIVFI and DIVFF */
	val = frac_pll->orig_pllcfg0;
	val |= PLL_NEWDIV_VAL;
	writel_relaxed(val, frac_pll->base + PLL_CFG0);

	/*Calculate and return the actual applied ppb*/
	applied_ppb = div64_s64((s64) (temp64 - frac_pll->orig_divff) * 1000000000,
			frac_pll->orig_divff + ((s64) frac_pll->orig_divfi + 1) * PLL_FRAC_DENOM);

	*ppb = (int) applied_ppb;

 exit:
	return rc;
}

static unsigned long imx_frac_pll_get_rate(struct clk_imx_pll *pll,
					   unsigned long parent_rate)
{
	struct clk_frac_pll *frac_pll;

	frac_pll = (struct clk_frac_pll *) container_of(pll,
						struct clk_frac_pll, imx_pll);

	return clk_pll_recalc_rate(&frac_pll->hw, parent_rate);
}

static int imx_frac_pll_set_rate(struct clk_imx_pll *pll, unsigned long rate,
				 unsigned long parent_rate)
{
	struct clk_frac_pll *frac_pll;
	int rc = IMX_CLK_PLL_SUCCESS;

	frac_pll = (struct clk_frac_pll *) container_of(pll,
						struct clk_frac_pll, imx_pll);

	if (clk_pll_set_rate(&frac_pll->hw, rate, parent_rate) < 0)
		rc = -IMX_CLK_PLL_INVALID_PARAM;

	return rc;
}

static const struct clk_imx_pll_ops imx_clk_frac_pll_ops = {
	.set_rate	= imx_frac_pll_set_rate,
	.get_rate	= imx_frac_pll_get_rate,
	.adjust		= imx_frac_pll_adjust,
	.init		= imx_frac_pll_init,
};

struct clk_hw *imx_clk_hw_frac_pll(const char *name,
				   const char *parent_name,
				   void __iomem *base)
{
	struct clk_init_data init;
	struct clk_frac_pll *pll;
	struct clk_hw *hw;
	int ret;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &clk_frac_pll_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->base = base;
	pll->hw.init = &init;

	hw = &pll->hw;

	ret = clk_hw_register(NULL, hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}

	pll->imx_pll.ops = &imx_clk_frac_pll_ops;

	if (imx_pll_register(&pll->imx_pll, name) < 0)
		pr_warn("Failed to register %s into imx pll\n", name);

	return hw;
}
EXPORT_SYMBOL_GPL(imx_clk_hw_frac_pll);
