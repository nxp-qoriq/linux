// SPDX-License-Identifier: GPL-2.0
/* Copyright 2018 NXP */

#include <linux/module.h>
#include <linux/ptp_clock_kernel.h>

#include "enetc_ptp.h"

struct enetc_ptp_config {
	u32	ck_sel;
	u32	tclk_period;
	u32	tmr_add;
	u32	tmr_prsc;
	u32	tmr_fiper1;
	u32	tmr_fiper2;
	u32	tmr_fiper3;
};

struct enetc_ptp {
	void __iomem *regs;
	struct ptp_clock *clock;
	struct ptp_clock_info caps;
	struct enetc_ptp_config *config;
	spinlock_t lock; /* protects regs */
};

int enetc_phc_index;

static u64 tmr_cnt_read(struct enetc_ptp *ptp)
{
	u64 ns;
	u32 lo, hi;

	lo = enetc_rd_reg(ptp->regs + TMR_CNT_L);
	hi = enetc_rd_reg(ptp->regs + TMR_CNT_H);
	ns = (u64)hi << 32;
	ns |= lo;
	return ns;
}

static void tmr_cnt_write(struct enetc_ptp *ptp, u64 ns)
{
	u32 hi = ns >> 32;
	u32 lo = ns & 0xffffffff;

	enetc_wr_reg(ptp->regs + TMR_CNT_L, lo);
	enetc_wr_reg(ptp->regs + TMR_CNT_H, hi);
}

static void set_alarm(struct enetc_ptp *ptp)
{
	u64 ns;
	u32 lo, hi;

	ns = tmr_cnt_read(ptp) + 1500000000ULL;
	ns = div_u64(ns, 1000000000UL) * 1000000000ULL;
	ns -= ptp->config->tclk_period;
	hi = ns >> 32;
	lo = ns & 0xffffffff;
	enetc_wr_reg(ptp->regs + TMR_ALARM_1_L, lo);
	enetc_wr_reg(ptp->regs + TMR_ALARM_1_H, hi);
}

static void set_fipers(struct enetc_ptp *ptp)
{
	set_alarm(ptp);
	enetc_wr_reg(ptp->regs + TMR_FIPER_1, ptp->config->tmr_fiper1);
	enetc_wr_reg(ptp->regs + TMR_FIPER_2, ptp->config->tmr_fiper2);
	enetc_wr_reg(ptp->regs + TMR_FIPER_3, ptp->config->tmr_fiper3);
}

static int enetc_ptp_adjfine(struct ptp_clock_info *ptp, long scaled_ppm)
{
	u64 adj, diff;
	u32 tmr_add;
	int neg_adj = 0;
	struct enetc_ptp *ptp_timer = container_of(ptp, struct enetc_ptp, caps);

	if (scaled_ppm < 0) {
		neg_adj = 1;
		scaled_ppm = -scaled_ppm;
	}
	tmr_add = ptp_timer->config->tmr_add;
	adj = tmr_add;

	/* calculate diff as adj*(scaled_ppm/65536)/1000000
	 * and round() to the nearest integer
	 */
	adj *= scaled_ppm;
	diff = div_u64(adj, 8000000);
	diff = (diff >> 13) + ((diff >> 12) & 1);

	tmr_add = neg_adj ? tmr_add - diff : tmr_add + diff;

	enetc_wr_reg(ptp_timer->regs + TMR_ADD, tmr_add);

	return 0;
}

static int enetc_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	unsigned long flags;
	struct enetc_ptp *ptp_timer = container_of(ptp, struct enetc_ptp, caps);

	spin_lock_irqsave(&ptp_timer->lock, flags);

	now = tmr_cnt_read(ptp_timer);
	now += delta;
	tmr_cnt_write(ptp_timer, now);
	set_fipers(ptp_timer);

	spin_unlock_irqrestore(&ptp_timer->lock, flags);

	return 0;
}

static int enetc_ptp_gettime(struct ptp_clock_info *ptp,
			     struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct enetc_ptp *ptp_timer = container_of(ptp, struct enetc_ptp, caps);

	spin_lock_irqsave(&ptp_timer->lock, flags);

	ns = tmr_cnt_read(ptp_timer);

	spin_unlock_irqrestore(&ptp_timer->lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

static int enetc_ptp_settime(struct ptp_clock_info *ptp,
			     const struct timespec64 *ts)
{
	u64 ns;
	unsigned long flags;
	struct enetc_ptp *ptp_timer = container_of(ptp, struct enetc_ptp, caps);

	ns = timespec64_to_ns(ts);

	spin_lock_irqsave(&ptp_timer->lock, flags);

	tmr_cnt_write(ptp_timer, ns);
	set_fipers(ptp_timer);

	spin_unlock_irqrestore(&ptp_timer->lock, flags);

	return 0;
}

static int enetc_ptp_enable(struct ptp_clock_info *ptp,
			    struct ptp_clock_request *rq, int on)
{
	struct enetc_ptp *ptp_timer = container_of(ptp, struct enetc_ptp, caps);
	unsigned long flags;
	u32 bit, mask;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		switch (rq->extts.index) {
		case 0:
			bit = ETS1EN;
			break;
		case 1:
			bit = ETS2EN;
			break;
		default:
			return -EINVAL;
		}
		spin_lock_irqsave(&ptp_timer->lock, flags);
		mask = enetc_rd_reg(ptp_timer->regs + TMR_TEMASK);
		if (on)
			mask |= bit;
		else
			mask &= ~bit;
		enetc_wr_reg(ptp_timer->regs + TMR_TEMASK, mask);
		spin_unlock_irqrestore(&ptp_timer->lock, flags);
		return 0;

	case PTP_CLK_REQ_PPS:
		spin_lock_irqsave(&ptp_timer->lock, flags);
		mask = enetc_rd_reg(ptp_timer->regs + TMR_TEMASK);
		if (on)
			mask |= PP1EN;
		else
			mask &= ~PP1EN;
		enetc_wr_reg(ptp_timer->regs + TMR_TEMASK, mask);
		spin_unlock_irqrestore(&ptp_timer->lock, flags);
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static struct ptp_clock_info enetc_ptp_caps = {
	.owner		= THIS_MODULE,
	.name		= "ENETC PTP clock",
	.max_adj	= 512000,
	.n_alarm	= 2,
	.n_ext_ts	= 2,
	.n_per_out	= 3,
	.n_pins		= 0,
	.pps		= 1,
	.adjfine	= enetc_ptp_adjfine,
	.adjtime	= enetc_ptp_adjtime,
	.gettime64	= enetc_ptp_gettime,
	.settime64	= enetc_ptp_settime,
	.enable		= enetc_ptp_enable,
};

static void enetc_ptp_init(struct enetc_ptp *ptp)
{
	u32 tclk = ptp->config->tclk_period;
	u32 ck_sel = ptp->config->ck_sel;
	u32 tmr_ctrl;
	unsigned long flags;

	tmr_ctrl = (tclk & TCLK_PERIOD_MASK) << TCLK_PERIOD_SHIFT |
		   (ck_sel & CK_SEL_MASK);
	tmr_ctrl |= FRD | FS;

	spin_lock_irqsave(&ptp->lock, flags);

	enetc_wr_reg(ptp->regs + TMR_CTRL, tmr_ctrl);
	enetc_wr_reg(ptp->regs + TMR_ADD, ptp->config->tmr_add);
	enetc_wr_reg(ptp->regs + TMR_PRSC, ptp->config->tmr_prsc);
	enetc_wr_reg(ptp->regs + TMR_FIPER_1, ptp->config->tmr_fiper1);
	enetc_wr_reg(ptp->regs + TMR_FIPER_2, ptp->config->tmr_fiper2);
	enetc_wr_reg(ptp->regs + TMR_FIPER_3, ptp->config->tmr_fiper3);
	set_alarm(ptp);

	enetc_wr_reg(ptp->regs + TMR_CTRL, tmr_ctrl | TE);

	spin_unlock_irqrestore(&ptp->lock, flags);
}

/* Reference clock frequency:	400MHz
 * Nominal frequency:		200MHz
 */
static struct enetc_ptp_config ptp_config = {
	.ck_sel = 1,
	.tclk_period = 5,
	.tmr_add = 0x80000000,
	.tmr_prsc = 2,
	.tmr_fiper1 = 999999995,
	.tmr_fiper2 = 99995,
	.tmr_fiper3 = 99995,
};

static int enetc_ptp_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct enetc_ptp *ptp;
	int err, len;

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	/* set up for high or low dma */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (err) {
		err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev,
				"DMA configuration failed: 0x%x\n", err);
			goto err_dma;
		}
	}

	err = pci_request_mem_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "pci_request_regions failed err=%d\n", err);
		goto err_pci_mem_reg;
	}

	pci_set_master(pdev);

	ptp = kzalloc(sizeof(*ptp), GFP_KERNEL);
	if (!ptp) {
		err = -ENOMEM;
		goto err_alloc_ptp;
	}

	len = pci_resource_len(pdev, ENETC_BAR_REGS);

	spin_lock_init(&ptp->lock);

	ptp->regs = ioremap(pci_resource_start(pdev, ENETC_BAR_REGS), len);
	if (!ptp->regs) {
		err = -ENXIO;
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto err_ioremap;
	}

	ptp->config = &ptp_config;

	enetc_ptp_init(ptp);

	ptp->caps = enetc_ptp_caps;

	ptp->clock = ptp_clock_register(&ptp->caps, &pdev->dev);
	if (IS_ERR(ptp->clock)) {
		err = PTR_ERR(ptp->clock);
		goto err_no_clock;
	}

	enetc_phc_index = ptp_clock_index(ptp->clock);

	pci_set_drvdata(pdev, ptp);

	return 0;

err_no_clock:
	iounmap(ptp->regs);
err_ioremap:
	kfree(ptp);
err_alloc_ptp:
	pci_release_mem_regions(pdev);
err_pci_mem_reg:
err_dma:
	pci_disable_device(pdev);

	return err;
}

static void enetc_ptp_remove(struct pci_dev *pdev)
{
	struct enetc_ptp *ptp = pci_get_drvdata(pdev);

	enetc_wr_reg(ptp->regs + TMR_TEVENT, 0);
	enetc_wr_reg(ptp->regs + TMR_CTRL, 0);

	enetc_phc_index = -1;
	ptp_clock_unregister(ptp->clock);

	iounmap(ptp->regs);
	kfree(ptp);

	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id enetc_ptp_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_PTP) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_ptp_id_table);

static struct pci_driver enetc_ptp_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_ptp_id_table,
	.probe = enetc_ptp_probe,
	.remove = enetc_ptp_remove,
};
module_pci_driver(enetc_ptp_driver);

MODULE_DESCRIPTION("ENETC PTP clock driver");
MODULE_LICENSE("GPL");
