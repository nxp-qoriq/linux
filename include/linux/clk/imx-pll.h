// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 *
 */

#ifndef __LINUX_CLK_IMX_PLL_H__
#define __LINUX_CLK_IMX_PLL_H__

enum clk_imx_pll_error {
	 IMX_CLK_PLL_SUCCESS = 0,
	 IMX_CLK_PLL_INVALID_PARAM,
	 IMX_CLK_PLL_PREC_ERR,
	 IMX_CLK_PLL_LOCK_ERR,
};

struct clk_imx_pll;

struct clk_imx_pll *clk_imx_pll_get_by_name(const char *name);
int clk_imx_pll_adjust(struct clk_imx_pll *pll, int *ppb);
unsigned long clk_imx_pll_get_rate(struct clk_imx_pll *pll,
				   unsigned long parent_rate);
int clk_imx_pll_set_rate(struct clk_imx_pll *pll, unsigned long rate,
			 unsigned long parent_rate);

#endif /*__LINUX_CLK_IMX_PLL_H__*/
