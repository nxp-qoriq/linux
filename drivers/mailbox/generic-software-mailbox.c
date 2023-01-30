// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2023 NXP
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/slab.h>

/*
 * Generic software Registers:
 *
 * TX_STATUS: Each bit for a TX channel
 * RX_STATUS: Each bit for a RX channel
 * RXDB_STATUS: Each bit for a RXDB channel
 * TX_STATUS, RX_STATUS and RXDB_STATUS registers description:
 *     bit31                        bit0
 *     ---------------------------------
 *     | CH31 |     ...    | CH1 | CH0 |
 *     ---------------------------------
 *
 * TX_CH[n]: Transmit data register for channeln
 * RX_CH[n]: Receive data register for channeln, valid if RX_STATUS[bitn] set.
 *
 * To send a message:
 * Update the data register TX_CH[n] with the message, then set the
 * TX_STATUS[bitn], inject a interrupt to remote side.
 *
 * When received a message:
 * Get the received data from RX_CH[n] and then clear the RX_STATUS[bitn] to
 * indicate the remote side transmit done.
 */

#define MBOX_TX_CHAN		(32)
#define MBOX_RX_CHAN		(32)
#define MBOX_RXDB_CHAN		(32)
#define RX_CHAN_SHFT		(MBOX_TX_CHAN)
#define RXDB_CHAN_SHFT		(MBOX_TX_CHAN + MBOX_RX_CHAN)
#define MBOX_CHAN_MAX		(MBOX_TX_CHAN + MBOX_RX_CHAN + MBOX_RXDB_CHAN)

struct sw_mbox_reg {
	uint32_t tx_status;
	uint32_t rx_status;
	uint32_t rxdb_status;
	uint32_t tx_ch[MBOX_TX_CHAN];
	uint32_t rx_ch[MBOX_RX_CHAN];
};

enum sw_mbox_type {
	SW_TYPE_TX,     /* Tx */
	SW_TYPE_RX,     /* Rx */
	SW_TYPE_RXDB,   /* Rx doorbell */
};

struct sw_mbox_con_priv {
	uint32_t idx;
	enum sw_mbox_type type;
	struct sw_mbox *priv;
};

struct sw_mbox {
	struct device *dev;
	struct sw_mbox_reg __iomem *base;
	struct sw_mbox_con_priv cp[MBOX_CHAN_MAX];
	struct mbox_chan chan[MBOX_CHAN_MAX];
	struct mbox_controller controller;
	int irq;
	int remote_irq;
};

static bool sw_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct sw_mbox_con_priv *cp = chan->con_priv;
	struct sw_mbox *mbox = cp->priv;
	uint32_t idx = cp->idx;

	if ((readl(&mbox->base->tx_status) & (1 << idx)) == 0)
		return true;

	return false;
}

static int sw_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	struct sw_mbox_con_priv *cp = chan->con_priv;
	struct sw_mbox *mbox = cp->priv;
	uint32_t idx = cp->idx;
	uint32_t *data = msg;
	int ret;

	if (cp->type != SW_TYPE_TX) {
		dev_err(mbox->dev, "Channel type error\n");
		return -EINVAL;
	}

	writel(*data, &mbox->base->tx_ch[idx]);
	writel(readl(&mbox->base->tx_status) | (1 << idx),
	       &mbox->base->tx_status);
	ret = irq_set_irqchip_state(mbox->remote_irq, IRQCHIP_STATE_PENDING,
				    true);
	if (ret) {
		dev_err(mbox->dev, "Fail to inject IRQ\n");
		return ret;
	}

	return 0;
}

static irqreturn_t sw_mbox_interrupt(int irq, void *dev_id)
{
	struct sw_mbox *mbox = dev_id;
	uint32_t rx_status = readl(&mbox->base->rx_status);
	uint32_t rxdb_status = readl(&mbox->base->rxdb_status);
	uint32_t rx_ch[MBOX_RX_CHAN];
	int i;

	if (!rx_status && !rxdb_status)
		return IRQ_NONE;

	if (rx_status) {
		memcpy_fromio(rx_ch, mbox->base->rx_ch, sizeof(rx_ch));
		writel(0, &mbox->base->rx_status);
	}

	if (rxdb_status)
		writel(0, &mbox->base->rxdb_status);

	for (i = 0; i < MBOX_RX_CHAN && rx_status; i++)
		if (rx_status & (1 << i))
			mbox_chan_received_data(&mbox->chan[i + RX_CHAN_SHFT],
						(void *)&rx_ch[i]);

	for (i = 0; i < MBOX_RXDB_CHAN && rxdb_status; i++)
		if (rxdb_status & (1 << i))
			mbox_chan_received_data(&mbox->chan[i + RXDB_CHAN_SHFT],
						NULL);

	return IRQ_HANDLED;
}

static int sw_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static void sw_mbox_shutdown(struct mbox_chan *chan)
{
}

static const struct mbox_chan_ops sw_mbox_ops = {
	.send_data    = sw_mbox_send_data,
	.startup      = sw_mbox_startup,
	.shutdown     = sw_mbox_shutdown,
	.last_tx_done = sw_mbox_last_tx_done,
};


static struct mbox_chan *sw_mbox_xlate(struct mbox_controller *mbox,
				       const struct of_phandle_args *sp)
{
	uint32_t type, idx, chan;

	if (sp->args_count != 2) {
		dev_err(mbox->dev, "Invalid argument count %d\n",
			sp->args_count);
		return ERR_PTR(-EINVAL);
	}

	type = sp->args[0];
	idx = sp->args[1];

	switch (type) {
	case SW_TYPE_TX:
		chan = idx;
		break;
	case SW_TYPE_RX:
		chan = RX_CHAN_SHFT + idx;
		break;
	case SW_TYPE_RXDB:
		chan = RXDB_CHAN_SHFT + idx;
		break;
	default:
		dev_err(mbox->dev, "Invalid chan type: %d\n", type);
		return ERR_PTR(-EINVAL);
	}

	if (chan >= MBOX_CHAN_MAX) {
		dev_err(mbox->dev, "Not supported channel number: %d. (type: %d, idx: %d)\n",
			chan, type, idx);
		return ERR_PTR(-EINVAL);
	}

	return &mbox->chans[chan];
}

static const struct of_device_id sw_mbox_of_match[] = {
	{ .compatible = "fsl,generic-software-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, sw_mbox_of_match);

static int sw_mailbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sw_mbox *mbox;
	int err, i;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;
	mbox->dev = dev;

	mbox->irq = platform_get_irq_byname(pdev, "irq");
	if (mbox->irq <= 0) {
		dev_err(dev, "Failed to get irq\n");
		return mbox->irq;
	}
	mbox->remote_irq = platform_get_irq_byname(pdev, "remote_irq");
	if (mbox->remote_irq <= 0) {
		dev_err(dev, "Failed to get remote irq\n");
		return mbox->remote_irq;
	}

	err = devm_request_irq(dev, mbox->irq, sw_mbox_interrupt, IRQF_SHARED,
			       pdev->name, mbox);
	if (err)
		return err;

	mbox->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbox->base))
		return PTR_ERR(mbox->base);

	memset_io(&mbox->base->tx_status, 0, 4);
	memset_io(&mbox->base->rx_status, 0, 4);
	memset_io(&mbox->base->rxdb_status, 0, 4);

	mbox->controller.dev = dev;
	mbox->controller.chans = mbox->chan;
	mbox->controller.num_chans = MBOX_CHAN_MAX;
	mbox->controller.ops = &sw_mbox_ops;
	mbox->controller.of_xlate = sw_mbox_xlate;
	mbox->controller.txdone_poll = true;

	for (i = 0; i < MBOX_CHAN_MAX; i++) {
		mbox->chan[i].con_priv = &mbox->cp[i];
		mbox->cp[i].priv = mbox;
		mbox->cp[i].idx = i;
	}

	err = devm_mbox_controller_register(dev, &mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, mbox);

	return 0;
}

static struct platform_driver sw_mbox_driver = {
	.driver = {
		.name = "generic-software-mailbox",
		.of_match_table = sw_mbox_of_match,
	},
	.probe = sw_mailbox_probe,
};

static int __init sw_mbox_init(void)
{
	return platform_driver_register(&sw_mbox_driver);
}

static void __exit sw_mbox_exit(void)
{
	platform_driver_unregister(&sw_mbox_driver);
}

module_init(sw_mbox_init);
module_exit(sw_mbox_exit);

MODULE_DESCRIPTION("Generic Software mailbox driver");
MODULE_LICENSE("GPL v2");
