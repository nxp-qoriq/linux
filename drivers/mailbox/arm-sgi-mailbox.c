// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 NXP
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
 * status register brelf description:
 *
 * bit31                |bit11           |bit7            |bit3         bit0
 * -------------------------------------------------------------------------
 * |      |      |      | RXDB_CH status |  RX_CH status  |  TX_CH status  |
 * -------------------------------------------------------------------------
 * TX_CH field: Each bit for a TX channel
 * RX_CH field: Each bit for a RX channel
 * RXDB_CH field: Each bit for a RXDB channel
 *
 * Set by the initiator to indicate the 'data' in the TX_CH register valid;
 * And clear by the terminator to indicate TX done.
 */

#define SR_RX_SHIFT		(4)
#define SR_RXDB_SHIFT		(8)
#define SR_RX_MASK		(0xf << SR_RX_SHIFT)
#define SR_RXDB_MASK		(0xf << SR_RXDB_SHIFT)

#define MBOX_TX_CHAN		(4)
#define MBOX_RX_CHAN		(4)
#define MBOX_RXDB_CHAN		(4)
#define MBOX_CHAN_MAX		(12)

struct arm_sgi_mbox_reg {
	uint32_t status;
	uint32_t tx_ch[MBOX_TX_CHAN];
	uint32_t rx_ch[MBOX_RX_CHAN];
};

enum sgi_mbox_type {
	ARM_SGI_TYPE_TX,     /* Tx */
	ARM_SGI_TYPE_RX,     /* Rx */
	ARM_SGI_TYPE_RXDB,   /* Rx doorbell */
};

struct arm_sgi_mbox_con_priv {
	uint32_t idx;
	enum sgi_mbox_type type;
	struct arm_sgi_mbox *priv;
};

struct arm_sgi_mbox {
	struct device *dev;
	struct arm_sgi_mbox_reg __iomem *base;
	struct arm_sgi_mbox_con_priv cp[MBOX_CHAN_MAX];
	struct mbox_chan chan[MBOX_CHAN_MAX];
	struct mbox_controller controller;
	uint32_t target_core;
};

static struct arm_sgi_mbox *mbox_global;

static bool arm_sgi_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct arm_sgi_mbox_con_priv *cp = chan->con_priv;
	struct arm_sgi_mbox *mbox = cp->priv;
	uint32_t idx = cp->idx;

	if (((mbox->base->status) & (1 << idx)) == 0)
		return true;

	return false;
}

static int arm_sgi_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	struct arm_sgi_mbox_con_priv *cp = chan->con_priv;
	struct arm_sgi_mbox *mbox = cp->priv;
	uint32_t idx = cp->idx;
	uint32_t *data = msg;

	if (cp->type != ARM_SGI_TYPE_TX)
		return -EINVAL;

	writel(*data, &mbox->base->tx_ch[idx]);
	mbox->base->status |= (1 << idx);

	/* sync before trigger IPI */
	mb();

	sgi_mbox_send_ipi_single(mbox->target_core);

	return 0;
}

int arm_sgi_mbox_interrupt(uint32_t irq)
{
	struct arm_sgi_mbox *mbox = mbox_global;
	uint32_t status = mbox->base->status;
	int i;

	status &= SR_RX_MASK | SR_RXDB_MASK;
	if (!status)
		return IRQ_NONE;

	for (i = 0; i < MBOX_RX_CHAN && status & 1 << (i + SR_RX_SHIFT); i++)
		mbox_chan_received_data(&mbox->chan[i + SR_RX_SHIFT],
					(void *)&mbox->base->rx_ch[i]);

	for (i = 0; i < MBOX_RXDB_CHAN && status & 1 << (i + SR_RXDB_SHIFT);
	     i++)
		mbox_chan_received_data(&mbox->chan[i + SR_RXDB_SHIFT], NULL);

	mbox->base->status &= ~status;

	/* sync */
	mb();

	return IRQ_HANDLED;
}

static int arm_sgi_mbox_startup(struct mbox_chan *chan)
{
	return 0;
}

static void arm_sgi_mbox_shutdown(struct mbox_chan *chan)
{
}

static const struct mbox_chan_ops arm_sgi_mbox_ops = {
	.send_data    = arm_sgi_mbox_send_data,
	.startup      = arm_sgi_mbox_startup,
	.shutdown     = arm_sgi_mbox_shutdown,
	.last_tx_done = arm_sgi_mbox_last_tx_done,
};


static struct mbox_chan *arm_sgi_mbox_xlate(struct mbox_controller *mbox,
					    const struct of_phandle_args *sp)
{
	uint32_t type, idx, chan;

	if (sp->args_count != 2) {
		dev_err(mbox->dev, "Invalid argument count %d\n", sp->args_count);
		return ERR_PTR(-EINVAL);
	}

	type = sp->args[0];
	idx = sp->args[1];

	switch (type) {
	case ARM_SGI_TYPE_TX:
		chan = idx;
		break;
	case ARM_SGI_TYPE_RX:
		chan = MBOX_TX_CHAN + idx;
		break;
	case ARM_SGI_TYPE_RXDB:
		chan = MBOX_TX_CHAN + MBOX_TX_CHAN + idx;
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

static const struct of_device_id arm_sgi_mbox_of_match[] = {
	{ .compatible = "fsl,arm-sgi-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, arm_sgi_mbox_of_match);

static int arm_sgi_mailbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct arm_sgi_mbox *mbox;
	int err, i;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;
	mbox->dev = dev;

	mbox->target_core = CONFIG_ARM_SGI_MAILBOX_TARGET_CORE;

	mbox->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mbox->base))
		return PTR_ERR(mbox->base);

	mbox->controller.dev = dev;
	mbox->controller.chans = mbox->chan;
	mbox->controller.num_chans = MBOX_CHAN_MAX;
	mbox->controller.ops = &arm_sgi_mbox_ops;
	mbox->controller.of_xlate = arm_sgi_mbox_xlate;
	mbox->controller.txdone_poll = true;

	for (i = 0; i < MBOX_CHAN_MAX; i++) {
		mbox->chan[i].con_priv = &mbox->cp[i];
		mbox->cp[i].priv = mbox;
		mbox->cp[i].idx = i;
	}

	/*
	 * TODO: improve
	 * this hack results in the driver cannot support multiple instances
	 */
	mbox_global = mbox;

	err = devm_mbox_controller_register(dev, &mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, mbox);

	return 0;
}

static struct platform_driver arm_sgi_mbox_driver = {
	.driver = {
		.name = "arm-sgi-mailbox",
		.of_match_table = arm_sgi_mbox_of_match,
	},
	.probe = arm_sgi_mailbox_probe,
};

static int __init arm_sgi_mbox_init(void)
{
	return platform_driver_register(&arm_sgi_mbox_driver);
}

static void __exit arm_sgi_mbox_exit(void)
{
	platform_driver_unregister(&arm_sgi_mbox_driver);
}

module_init(arm_sgi_mbox_init);
module_exit(arm_sgi_mbox_exit);

MODULE_DESCRIPTION("ARM SGI mailbox driver");
MODULE_LICENSE("GPL v2");
