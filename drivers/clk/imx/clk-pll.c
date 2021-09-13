// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 *
 */

#include <linux/clk/imx-pll.h>
#include <linux/stddef.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/string.h>
#include "clk-pll.h"


static struct clk_imx_pll *clk_imx_pll_array[MAX_IMX_PLL_NUM] = { NULL };

struct clk_imx_pll *clk_imx_pll_get_by_name(const char *name)
{
	int i;
	struct clk_imx_pll *pll = NULL;

	for (i = 0; i < MAX_IMX_PLL_NUM; i++) {
		if (clk_imx_pll_array[i] && clk_imx_pll_array[i]->ops
			&& !strncmp(clk_imx_pll_array[i]->pll_name, name, MAX_PLL_NAME_SIZE)) {

			pll = clk_imx_pll_array[i];

			/* Init all PLL original parameters*/
			if (pll && pll->ops && pll->ops->init)
				pll->ops->init(pll);

			break;
		}
	}

	return pll;
}
EXPORT_SYMBOL(clk_imx_pll_get_by_name);

int clk_imx_pll_adjust(struct clk_imx_pll *pll, int *ppb)
{
	if (pll && pll->ops && pll->ops->adjust)
		return pll->ops->adjust(pll, ppb);

	return -IMX_CLK_PLL_INVALID_PARAM;
}
EXPORT_SYMBOL(clk_imx_pll_adjust);

unsigned long clk_imx_pll_get_rate(struct clk_imx_pll *pll,
				   unsigned long parent_rate)
{
	if (pll && pll->ops && pll->ops->get_rate)
		return pll->ops->get_rate(pll, parent_rate);

	return 0;
}
EXPORT_SYMBOL(clk_imx_pll_get_rate);

int clk_imx_pll_set_rate(struct clk_imx_pll *pll, unsigned long rate,
			 unsigned long parent_rate)
{
	if (pll && pll->ops && pll->ops->set_rate)
		return pll->ops->set_rate(pll, rate, parent_rate);

	return -IMX_CLK_PLL_INVALID_PARAM;
}
EXPORT_SYMBOL(clk_imx_pll_set_rate);

int imx_pll_register(struct clk_imx_pll *pll, const char *name)
{
	int i;

	strncpy(pll->pll_name, name, MAX_PLL_NAME_SIZE);

	for (i = 0; i < MAX_IMX_PLL_NUM; i++) {
		if (!clk_imx_pll_array[i]) {
			clk_imx_pll_array[i] = pll;
			return 0;
		}
	}

	return -EINVAL;
}
