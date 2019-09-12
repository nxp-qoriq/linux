// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP
 *
 * Clock driver for LS1028A Display output interfaces(LCD, DPHY).
 */

#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* PLLDIG register offsets and bit masks */
#define PLLDIG_REG_PLLSR            0x24
#define PLLDIG_REG_PLLDV            0x28
#define PLLDIG_REG_PLLFM            0x2c
#define PLLDIG_REG_PLLFD            0x30
#define PLLDIG_REG_PLLCAL1          0x38
#define PLLDIG_REG_PLLCAL2          0x3c
#define PLLDIG_LOCK_MASK            BIT(2)
#define PLLDIG_SSCGBYP_ENABLE       BIT(30)
#define PLLDIG_FDEN                 BIT(30)
#define PLLDIG_DTHRCTL              (0x3 << 16)

/* macro to get/set values into register */
#define PLLDIG_GET_MULT(x)          (((x) & ~(0xffffff00)) << 0)
#define PLLDIG_GET_RFDPHI1(x)       ((u32)(x) >> 25)
#define PLLDIG_SET_RFDPHI1(x)       ((u32)(x) << 25)

/* Maximum of the divider */
#define MAX_RFDPHI1          63

/* Best value of multiplication factor divider */
#define PLLDIG_DEFAULE_MULT         44

/*
 * Clock configuration relationship between the PHI1 frequency(fpll_phi) and
 * the output frequency of the PLL is determined by the PLLDV, according to
 * the following equation:
 * fpll_phi = (pll_ref * mfd) / div_rfdphi1
 */
struct plldig_phi1_param {
	unsigned long rate;
	unsigned int rfdphi1;
	unsigned int mfd;
};

enum plldig_phi1_freq_range {
	PHI1_MIN        = 27000000U,
	PHI1_MAX        = 600000000U
};

struct clk_plldig {
	struct clk_hw hw;
	void __iomem *regs;
	struct device *dev;
};

#define to_clk_plldig(_hw)	container_of(_hw, struct clk_plldig, hw)
#define LOCK_TIMEOUT_US		USEC_PER_MSEC

static int plldig_enable(struct clk_hw *hw)
{
	struct clk_plldig *data = to_clk_plldig(hw);
	u32 val;

	val = readl(data->regs + PLLDIG_REG_PLLFM);
	/*
	 * Use Bypass mode with PLL off by default, the frequency overshoot
	 * detector output was disable. SSCG Bypass mode should be enable.
	 */
	val |= PLLDIG_SSCGBYP_ENABLE;
	writel(val, data->regs + PLLDIG_REG_PLLFM);

	val = readl(data->regs + PLLDIG_REG_PLLFD);
	/* Disable dither and Sigma delta modulation in bypass mode */
	val |= (PLLDIG_FDEN | PLLDIG_DTHRCTL);
	writel(val, data->regs + PLLDIG_REG_PLLFD);

	return 0;
}

static void plldig_disable(struct clk_hw *hw)
{
	struct clk_plldig *data = to_clk_plldig(hw);
	u32 val;

	val = readl(data->regs + PLLDIG_REG_PLLFM);

	val &= ~PLLDIG_SSCGBYP_ENABLE;
	writel(val, data->regs + PLLDIG_REG_PLLFM);
}

static int plldig_is_enabled(struct clk_hw *hw)
{
	struct clk_plldig *data = to_clk_plldig(hw);

	return (readl(data->regs + PLLDIG_REG_PLLFM) & PLLDIG_SSCGBYP_ENABLE);
}

static unsigned long plldig_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_plldig *data = to_clk_plldig(hw);
	u32 mult, div, val;

	val = readl(data->regs + PLLDIG_REG_PLLDV);

	/* Check if PLL is bypassed */
	if (val & PLLDIG_SSCGBYP_ENABLE)
		return parent_rate;

	/* Checkout multiplication factor divider value */
	mult = val;
	mult = PLLDIG_GET_MULT(mult);

	/* Checkout divider value of the output frequency */
	div = val;
	div = PLLDIG_GET_RFDPHI1(div);

	return (parent_rate * mult) / div;
}

static int plldig_calc_target_rate(unsigned long target_rate,
				   unsigned long parent_rate,
				   struct plldig_phi1_param *phi1_out)
{
	unsigned int div, mfd, ret;
	unsigned long round_rate;

	/*
	 * Firstly, check the target rate whether is divisible
	 * by the best VCO frequency.
	 */
	mfd = PLLDIG_DEFAULE_MULT;
	round_rate = parent_rate * mfd;
	div = round_rate / target_rate;
	if (!div || div > MAX_RFDPHI1)
		return -EINVAL;

	ret = round_rate % target_rate;
	if (!ret)
		goto out;

	/*
	 * Otherwise, try a rounding algorithm to driven the target rate,
	 * this algorithm allows tolerances between the target rate and
	 * real rate, it based on the best VCO output frequency.
	 *
	 * Add half of the target rate so the result will be rounded to
	 * cloeset instead of rounded down.
	 */
	round_rate += (target_rate / 2);
	div = round_rate / target_rate;
	if (!div || div > MAX_RFDPHI1)
		return -EINVAL;

out:
	phi1_out->rfdphi1 = PLLDIG_SET_RFDPHI1(div);
	phi1_out->mfd = mfd;
	phi1_out->rate = target_rate;

	return 0;
}

static int plldig_determine_rate(struct clk_hw *hw,
				 struct clk_rate_request *req)
{
	int ret;
	struct clk_hw *parent;
	struct plldig_phi1_param phi1_param;
	unsigned long parent_rate;

	if (req->rate == 0 || req->rate < PHI1_MIN || req->rate > PHI1_MAX)
		return -EINVAL;

	parent = clk_hw_get_parent(hw);
	parent_rate = clk_hw_get_rate(parent);

	ret = plldig_calc_target_rate(req->rate, parent_rate, &phi1_param);
	if (ret)
		return ret;

	req->rate = phi1_param.rate;

	return 0;
}

static int plldig_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_plldig *data = to_clk_plldig(hw);
	struct plldig_phi1_param phi1_param;
	unsigned int rfdphi1, val, cond;
	int ret = -ETIMEDOUT;

	ret = plldig_calc_target_rate(rate, parent_rate, &phi1_param);
	if (ret)
		return ret;

	val = phi1_param.mfd;
	rfdphi1 = phi1_param.rfdphi1;
	val |= rfdphi1;

	writel(val, data->regs + PLLDIG_REG_PLLDV);

	/* delay 200us make sure that old lock state is cleared */
	udelay(200);

	/* Wait until PLL is locked or timeout (maximum 1000 usecs) */
	ret = readl_poll_timeout_atomic(data->regs + PLLDIG_REG_PLLSR, cond,
					cond & PLLDIG_LOCK_MASK, 0,
					USEC_PER_MSEC);

	return ret;
}

static const struct clk_ops plldig_clk_ops = {
	.enable = plldig_enable,
	.disable = plldig_disable,
	.is_enabled = plldig_is_enabled,
	.recalc_rate = plldig_recalc_rate,
	.determine_rate = plldig_determine_rate,
	.set_rate = plldig_set_rate,
};

static int plldig_clk_probe(struct platform_device *pdev)
{
	struct clk_plldig *data;
	struct resource *mem;
	struct clk_init_data init = {};
	struct device *dev = &pdev->dev;
	const char *parent_name;
	int ret, val;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(data->regs))
		return PTR_ERR(data->regs);

	init.name = dev->of_node->name;
	init.ops = &plldig_clk_ops;
	parent_name = of_clk_get_parent_name(dev->of_node, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	data->hw.init = &init;
	data->dev = dev;

	/*
	 * The multiplication factor value can't be changed
	 * on the fly, write the fixed value as default.
	 */
	val = PLLDIG_DEFAULE_MULT;
	writel(val, data->regs + PLLDIG_REG_PLLDV);

	ret = devm_clk_hw_register(dev, &data->hw);
	if (ret) {
		dev_err(dev, "failed to register %s clock\n", init.name);
		return ret;
	}

	return of_clk_add_hw_provider(dev->of_node, of_clk_hw_simple_get,
								&data->hw);
}

static const struct of_device_id plldig_clk_id[] = {
	{ .compatible = "fsl,ls1028a-plldig"},
	{ }
};
MODULE_DEVICE_TABLE(of, plldig_clk_id);

static struct platform_driver plldig_clk_driver = {
	.driver = {
		.name = "plldig-clock",
		.of_match_table = plldig_clk_id,
	},
	.probe = plldig_clk_probe,
};
module_platform_driver(plldig_clk_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wen He <wen.he_1@nxp.com>");
MODULE_DESCRIPTION("LS1028A Display output interface pixel clock driver");
