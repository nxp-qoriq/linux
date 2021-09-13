// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 *
 */

#ifndef __IMX_CLK_PLL_H
#define __IMX_CLK_PLL_H

#include <linux/clk/imx-pll.h>

#define MAX_IMX_PLL_NUM		16
#define MAX_PLL_NAME_SIZE	16

struct clk_imx_pll;

struct clk_imx_pll_ops {
	int     (*set_rate)(struct clk_imx_pll *pll, unsigned long rate,
			    unsigned long parent_rate);
	unsigned long (*get_rate)(struct clk_imx_pll *pll,
				  unsigned long parent_rate);
	int (*adjust)(struct clk_imx_pll *pll, int *ppb);
	void (*init)(struct clk_imx_pll *pll);
};

struct clk_imx_pll {
	char    pll_name[MAX_PLL_NAME_SIZE];
	const struct clk_imx_pll_ops *ops;
};

int imx_pll_register(struct clk_imx_pll *pll, const char *name);

#endif /*__IMX_CLK_PLL_H*/
