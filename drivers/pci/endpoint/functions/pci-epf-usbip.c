// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe Endpoint Function driver that provides the low-level PCIe transport
 * layer used to carry USB urb packets between a PCIe Endpoint and a Root Complex.
 *
 * Copyright 2026 NXP
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/minmax.h>
#include <linux/pci-epf-usbip.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>

#ifndef PCI_USB_DEBUG
#define PCI_USB_DEBUG 0
#endif

#define IRQ_TYPE_INTX			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

#define COMMAND_RAISE_INTX_IRQ		BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ			BIT(3)
#define COMMAND_WRITE			BIT(4)
#define COMMAND_COPY			BIT(5)
#define COMMAND_USB_SUBMIT              BIT(6) /*EP -> RC arm RX mailbox internal */
#define COMMAND_USB_RETURN_ARM          BIT(7) /* RC -> EP arm TX mailbox */

#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)
#define STATUS_USB_SUBMIT_SUCCESS       BIT(9)
#define STATUS_USB_SUBMIT_FAIL          BIT(10)
#define STATUS_USB_RETURN_ARMED         BIT(11)
#define STATUS_USB_RETURN_ARM_FAIL      BIT(12)
#define STATUS_USB_RETURN_SENT          BIT(13)
#define STATUS_USB_RETURN_SEND_FAIL     BIT(14)


#define FLAG_USE_DMA			BIT(0)
#define USBIP_TX_Q_DEPTH                1024

static struct workqueue_struct *kpciusb_workqueue;

struct pci_epf_usb_stream {
	void *buf;
	u32 len;
	u32 off;
	bool ready;
	bool closed;
	wait_queue_head_t wq;
	spinlock_t lock;
};

struct pci_epf_usb_tx_node {
	struct list_head list;
	void *buf;
	u32 len;
};

struct pci_epf_usb_tx_queue {
	struct list_head pending_list;
	spinlock_t lock;
	u32 depth;
	struct work_struct work;
	wait_queue_head_t space_wq;
};

struct pci_epf_usb_tx_mailbox {
	u64 dst_addr;
	u32 size;
	u32 irq_type;
	u32 irq_number;
	u32 flags;
	bool armed;
	spinlock_t lock;
};


struct pci_epf_usb {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		test_reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	struct dma_chan		*dma_chan_tx;
	struct dma_chan		*dma_chan_rx;
	struct dma_chan		*transfer_chan;
	dma_cookie_t		transfer_cookie;
	enum dma_status		transfer_status;
	struct completion	transfer_complete;
	bool			dma_supported;
	bool			dma_private;
	const struct pci_epc_features *epc_features;
	struct usbip_pcie_ctx *usb_ctx;
	struct pci_epf_usb_stream rx_stream;   /* mailbox-backed RX stream */
	struct pci_epf_usb_tx_queue   txq;        /* queued EP->RC return packets */
	struct pci_epf_usb_tx_mailbox tx_mb;      /* RC-armed return mailbox */
	struct mutex pci_xfer_lock;
};

struct pci_epf_usbip_reg {
	u32 magic;
	u32 version;

	u32 submit_command;      /*0x08*/
	u32 submit_status;       /*0x0c*/
	u32 submit_src_addr_lo;  /*0x10*/
	u32 submit_src_addr_hi;  /*0x14*/
	u32 submit_len;          /*0x18*/
	u32 submit_flags;        /*0x1c*/

	u32 return_command;      /*0x20*/
	u32 return_status;       /*0x24*/
	u32 return_dst_addr_lo;  /*0x28*/
	u32 return_dst_addr_hi;  /*0x2c*/
	u32 return_window;       /*0x30*/
	u32 return_flags;        /*0x34*/
	u32 return_actual;       /*0x38*/

	u32 irq_type;            /*0x3c*/
	u32 irq_number;          /*0x40*/

	u32 error_code;          /*0x44*/

	u32 submit_seq;
	u32 submit_done_seq;
	u32 return_arm_seq;
	u32 return_done_seq;
} __packed;

static struct pci_epf_header test_header = {
	.vendorid	= 0x1131,
	.deviceid	= 0x0000,
	.baseclass_code = PCI_BASE_CLASS_INPUT,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576 };

static int pci_epf_usb_stream_recv(struct pci_epf *epf, void *buf, int size);
static int pci_epf_usb_stream_send(struct pci_epf *epf, const void *buf, int size);
static void pci_epf_usb_raise_irq(struct pci_epf_usb *epf_test, struct pci_epf_usbip_reg *reg);
static int pci_epf_usb_ret_submit(struct pci_epf_usb *epf_test,
				   struct pci_epf_usbip_reg *reg,
				   struct pci_epf_usb_tx_node *node);


static int epf_usbip_recv_stream(struct usbip_pcie_ctx *ctx, void *buf, int size)
{
	return pci_epf_usb_stream_recv(ctx->epf, buf, size);
}

static int epf_usbip_send_stream(struct usbip_pcie_ctx *ctx, const void *buf, int size)
{
	return pci_epf_usb_stream_send(ctx->epf, buf, size);
}

static const struct usbip_pcie_ops epf_usbip_ops = {
    .recv_stream = epf_usbip_recv_stream,
    .send_stream = epf_usbip_send_stream,
};

/* ------------------------------------------------------------------------- */
/* TX mailbox helpers (EP -> RC return path)                                 */
/* ------------------------------------------------------------------------- */

static bool pci_epf_usb_txq_has_space(struct pci_epf_usb_tx_queue *q)
{
	unsigned long flags;
	bool has_space;

	spin_lock_irqsave(&q->lock, flags);
	has_space = q->depth < USBIP_TX_Q_DEPTH;
	spin_unlock_irqrestore(&q->lock, flags);

	return has_space;
}

static void pci_epf_usb_txq_init(struct pci_epf_usb_tx_queue *q)
{
	INIT_LIST_HEAD(&q->pending_list);
	spin_lock_init(&q->lock);
	init_waitqueue_head(&q->space_wq);
	q->depth = 0;
}

static void pci_epf_usb_txq_purge(struct pci_epf_usb_tx_queue *q)
{
	struct pci_epf_usb_tx_node *n, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&q->lock, flags);
	list_for_each_entry_safe(n, tmp, &q->pending_list, list) {
		list_del(&n->list);
		kfree(n->buf);
		kfree(n);
	}
	q->depth = 0;
	spin_unlock_irqrestore(&q->lock, flags);
	wake_up_all(&q->space_wq);
}

static void pci_epf_usb_tx_mb_init(struct pci_epf_usb_tx_mailbox *mb)
{
	mb->dst_addr = 0;
	mb->size = 0;
	mb->irq_type = IRQ_TYPE_MSI;
	mb->irq_number = 1;
	mb->flags = 0;
	mb->armed = false;
	spin_lock_init(&mb->lock);
}

static void pci_epf_usb_tx_mb_clear(struct pci_epf_usb_tx_mailbox *mb)
{
	unsigned long flags;

	spin_lock_irqsave(&mb->lock, flags);
	mb->dst_addr = 0;
	mb->size = 0;
	mb->irq_type = IRQ_TYPE_MSI;
	mb->irq_number = 1;
	mb->flags = 0;
	mb->armed = false;
	spin_unlock_irqrestore(&mb->lock, flags);
}

static int pci_epf_usb_arm_tx_mailbox(struct pci_epf_usb *epf_test, struct pci_epf_usbip_reg *reg)
{
	unsigned long flags;
	u64 dst_addr;
	u32 window;

	dst_addr = ((u64)READ_ONCE(reg->return_dst_addr_hi) << 32) | READ_ONCE(reg->return_dst_addr_lo);
	window = READ_ONCE(reg->return_window);

	if (!dst_addr || !window)
		return -EINVAL;

	spin_lock_irqsave(&epf_test->tx_mb.lock, flags);
	epf_test->tx_mb.dst_addr = dst_addr;
	epf_test->tx_mb.size = window;
	epf_test->tx_mb.irq_type = READ_ONCE(reg->irq_type);
	epf_test->tx_mb.irq_number = READ_ONCE(reg->irq_number);
	epf_test->tx_mb.flags = READ_ONCE(reg->return_flags);
	epf_test->tx_mb.armed = true;
	spin_unlock_irqrestore(&epf_test->tx_mb.lock, flags);

	return 0;
}


/* ------------------------------------------------------------------------- */
/* RX mailbox helpers (RC -> EP return path)                                 */
/* ------------------------------------------------------------------------- */
static void pci_epf_usb_stream_init(struct pci_epf_usb_stream *s)
{
	s->buf = NULL;
	s->len = 0;
	s->off = 0;
	s->ready = false;
	s->closed = false;
	init_waitqueue_head(&s->wq);
	spin_lock_init(&s->lock);
}

static void pci_epf_usb_stream_release_locked(struct pci_epf_usb_stream *s)
{
	kfree(s->buf);
	s->buf = NULL;
	s->len = 0;
	s->off = 0;
	s->ready = false;
}

static void pci_epf_usb_stream_close(struct pci_epf_usb *epf_test)
{
	unsigned long flags;

	spin_lock_irqsave(&epf_test->rx_stream.lock, flags);
	epf_test->rx_stream.closed = true;
	pci_epf_usb_stream_release_locked(&epf_test->rx_stream);
	spin_unlock_irqrestore(&epf_test->rx_stream.lock, flags);
	wake_up_interruptible(&epf_test->rx_stream.wq);
}

static int pci_epf_usb_stream_recv(struct pci_epf *epf, void *buf, int size)
{
	struct pci_epf_usb *epf_test;
	struct pci_epf_usb_stream *s;
	struct device *dev = &epf->dev;
	unsigned long flags;
	u32 avail;
	int ret;

	if (!epf || !buf || size <= 0)
		return -EINVAL;

	epf_test = epf_get_drvdata(epf);
	if (!epf_test)
		return -ENODEV;

	s = &epf_test->rx_stream;

	ret = wait_event_interruptible(s->wq,
				       READ_ONCE(s->ready) ||
				       READ_ONCE(s->closed));
	if (ret)
		return ret;

	spin_lock_irqsave(&s->lock, flags);

	if (s->closed) {
		spin_unlock_irqrestore(&s->lock, flags);
		return 0;
	}

	if (!s->ready || !s->buf || s->off >= s->len) {
		spin_unlock_irqrestore(&s->lock, flags);
		return -EAGAIN;
	}

	avail = s->len - s->off;

	/*
	 * Do not merge leftover bytes from this staged packet with bytes from
	 * a future COMMAND_USB_SUBMIT packet.
	 *
	 * Example bad case:
	 *   staged len = 56
	 *   header recv consumes 48
	 *   tail avail = 8
	 *   next header recv asks 48
	 *
	 * Returning 8 stale bytes + 40 bytes from the next packet corrupts the
	 * USB/IP header. Drop the stale tail instead.
	 */
	if (avail < size) {
		dev_dbg(dev, "EPF_STREAM_RECV: drop stale tail avail=%u req=%d len=%u off=%u\n",
				    avail, size, s->len, s->off);

		pci_epf_usb_stream_release_locked(s);
		spin_unlock_irqrestore(&s->lock, flags);

		return -EOVERFLOW;
	}

	memcpy(buf, (u8 *)s->buf + s->off, size);
	s->off += size;

	if (s->off >= s->len)
		pci_epf_usb_stream_release_locked(s);

	spin_unlock_irqrestore(&s->lock, flags);

	return size;
}

static int pci_epf_usb_stream_store(struct pci_epf_usb *epf_test, void *buf,
				     u32 len)
{
	unsigned long flags;
	struct pci_epf_usb_stream *s = &epf_test->rx_stream;
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;

	dev_dbg(dev, "DBG_STREAM_STORE: len=%u buf=%px\n", len, buf);

#if PCI_USB_DEBUG
	if (buf && len) {
		print_hex_dump(KERN_INFO, "DBG_STREAM_STORE_RAW: ",
				DUMP_PREFIX_OFFSET, 16, 1,
				buf,
				min_t(u32, len, 64),
				false);
	}
#endif

	spin_lock_irqsave(&s->lock, flags);
	if (s->closed) {
		spin_unlock_irqrestore(&s->lock, flags);
		kfree(buf);
		return -ESHUTDOWN;
	}

	pci_epf_usb_stream_release_locked(s);
	s->buf = buf;
	s->len = len;
	s->off = 0;
	s->ready = true;
	spin_unlock_irqrestore(&s->lock, flags);

	dev_dbg(dev, "DBG_STREAM_STORE_DONE: len=%u off=%u ready=%d closed=%d\n",
	s->len, s->off, s->ready, s->closed);

	wake_up_interruptible(&s->wq);
	return 0;
}


static int pci_epf_usb_stream_send(struct pci_epf *epf, const void *buf, int size)
{
	struct pci_epf_usb *epf_test;
	struct pci_epf_usb_tx_node *node;
	unsigned long flags;
	struct device *dev = &epf->dev;

	if (!epf || !buf || size <= 0)
		return -EINVAL;

	epf_test = epf_get_drvdata(epf);
	if (!epf_test)
		return -ENODEV;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	node->buf = kmemdup(buf, size, GFP_KERNEL);
	if (!node->buf) {
		kfree(node);
		return -ENOMEM;
	}
	node->len = size;

	/*
	 * Do not drop HID return packets just because the EP -> RC queue is full.
	 * Dropping a keyboard release report causes the host to repeat the last key.
	 *
	 * This function already uses GFP_KERNEL allocation, so this path is expected
	 * to run in sleepable context.
	 */
	if (!pci_epf_usb_txq_has_space(&epf_test->txq)) {
		int wait_ret;

		dev_info_ratelimited(dev, "EP TXQ full depth=%u, waiting for space\n",
				READ_ONCE(epf_test->txq.depth));

		wait_ret = wait_event_interruptible_timeout(epf_test->txq.space_wq,
				pci_epf_usb_txq_has_space(&epf_test->txq),
				msecs_to_jiffies(5000));

		if (wait_ret <= 0) {
			dev_err(dev, "EP TXQ still full, cannot queue return \
					len=%d depth=%u\n", size, READ_ONCE(epf_test->txq.depth));
			kfree(node->buf);
			kfree(node);
			return wait_ret == 0 ? -ETIMEDOUT : wait_ret;
		}
	}

	spin_lock_irqsave(&epf_test->txq.lock, flags);
	if (epf_test->txq.depth >= USBIP_TX_Q_DEPTH) {
		spin_unlock_irqrestore(&epf_test->txq.lock, flags);
		kfree(node->buf);
		kfree(node);
		dev_err(dev, "EP TXQ race: full after wait, dropping return \
				len=%d depth=%u\n", size, READ_ONCE(epf_test->txq.depth));
		return -EBUSY;
	}


	list_add_tail(&node->list, &epf_test->txq.pending_list);
	epf_test->txq.depth++;
	spin_unlock_irqrestore(&epf_test->txq.lock, flags);

	spin_lock_irqsave(&epf_test->tx_mb.lock, flags);
	if (epf_test->tx_mb.armed)
		queue_work(kpciusb_workqueue, &epf_test->txq.work);
	spin_unlock_irqrestore(&epf_test->tx_mb.lock, flags);

	return size;
}

static void pci_epf_usb_tx_node_free(struct pci_epf_usb_tx_node *node)
{
	if (!node)
		return;

	kfree(node->buf);
	kfree(node);
}


static void pci_epf_usb_tx_handler(struct work_struct *work)
{
	struct pci_epf_usb_tx_queue *txq =
		container_of(work, struct pci_epf_usb_tx_queue, work);
	struct pci_epf_usb *epf_test =
		container_of(txq, struct pci_epf_usb, txq);
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epf_usbip_reg *reg = epf_test->reg[epf_test->test_reg_bar];
	struct pci_epf_usb_tx_node *node;
	unsigned long flags;
	int ret;

	/*
	 * One COMMAND_USB_RETURN_ARM gives one return window.
	 * Send at most one queued return node.
	 */
	spin_lock_irqsave(&txq->lock, flags);
	if (list_empty(&txq->pending_list)) {
		spin_unlock_irqrestore(&txq->lock, flags);
		return;
	}

	/*
	 * Peek only. Do not delete yet.
	 * If return mailbox is not armed, node must remain queued.
	 */
	node = list_first_entry(&txq->pending_list,
				struct pci_epf_usb_tx_node, list);
	spin_unlock_irqrestore(&txq->lock, flags);

	ret = pci_epf_usb_ret_submit(epf_test, reg, node);

	if (ret == -EAGAIN || ret == -EBUSY) {
		dev_dbg(dev, "USB RET_SUBMIT not armed ret=%d, keep queued len=%u\n", ret, node->len);
		return;
	}

	/*
	 * Success or non-retryable failure: remove from queue now.
	 */
	spin_lock_irqsave(&txq->lock, flags);
	list_del_init(&node->list);
	if (txq->depth)
		txq->depth--;
	spin_unlock_irqrestore(&txq->lock, flags);
	wake_up_interruptible(&txq->space_wq);

	if (ret < 0) {
		dev_err(dev, "USB RET_SUBMIT TX failed: %d, dropping len=%u\n", ret, node->len);
	}

	/*
	 * Success.
	 * pci_epf_usb_ret_submit() already sets STATUS_USB_RETURN_SENT and
	 * raises IRQ. Do not raise IRQ again here.
	 */
	pci_epf_usb_tx_node_free(node);
}

static void pci_epf_usb_dma_callback(void *param)
{
	struct pci_epf_usb *epf_test = param;
	struct dma_tx_state state;

	epf_test->transfer_status =
		dmaengine_tx_status(epf_test->transfer_chan,
				    epf_test->transfer_cookie, &state);
	if (epf_test->transfer_status == DMA_COMPLETE ||
	    epf_test->transfer_status == DMA_ERROR)
		complete(&epf_test->transfer_complete);
}

/**
 * pci_epf_usb_data_transfer() - Function that uses dmaengine API to transfer
 *				  data between PCIe EP and remote PCIe RC
 * @epf_test: the EPF test device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 * @dma_remote: remote RC physical address
 * @dir: DMA transfer direction
 *
 * Function that uses dmaengine API to transfer data between PCIe EP and remote
 * PCIe RC. The source and destination address can be a physical address given
 * by pci_epc_mem_alloc_addr or the one obtained using DMA mapping APIs.
 *
 * The function returns '0' on success and negative value on failure.
 */
static int pci_epf_usb_data_transfer(struct pci_epf_usb *epf_test,
				      dma_addr_t dma_dst, dma_addr_t dma_src,
				      size_t len, dma_addr_t dma_remote,
				      enum dma_transfer_direction dir)
{
	struct dma_chan *chan = (dir == DMA_MEM_TO_DEV) ?
				 epf_test->dma_chan_tx : epf_test->dma_chan_rx;
	dma_addr_t dma_local = (dir == DMA_MEM_TO_DEV) ? dma_src : dma_dst;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct pci_epf *epf = epf_test->epf;
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	int ret;

	if (IS_ERR_OR_NULL(chan)) {
		dev_err(dev, "Invalid DMA memcpy channel\n");
		return -EINVAL;
	}

	if (epf_test->dma_private) {
		sconf.direction = dir;
		if (dir == DMA_MEM_TO_DEV)
			sconf.dst_addr = dma_remote;
		else
			sconf.src_addr = dma_remote;

		if (dmaengine_slave_config(chan, &sconf)) {
			dev_err(dev, "DMA slave config fail\n");
			return -EIO;
		}
		tx = dmaengine_prep_slave_single(chan, dma_local, len, dir,
						 flags);
	} else {
		tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len,
					       flags);
	}

	if (!tx) {
		dev_err(dev, "Failed to prepare DMA memcpy\n");
		return -EIO;
	}

	reinit_completion(&epf_test->transfer_complete);
	epf_test->transfer_chan = chan;
	tx->callback = pci_epf_usb_dma_callback;
	tx->callback_param = epf_test;
	epf_test->transfer_cookie = dmaengine_submit(tx);

	ret = dma_submit_error(epf_test->transfer_cookie);
	if (ret) {
		dev_err(dev, "Failed to do DMA tx_submit %d\n", ret);
		goto terminate;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_interruptible(&epf_test->transfer_complete);
	if (ret < 0) {
		dev_err(dev, "DMA wait_for_completion interrupted\n");
		goto terminate;
	}

	if (epf_test->transfer_status == DMA_ERROR) {
		dev_err(dev, "DMA transfer failed\n");
		ret = -EIO;
	}

terminate:
	dmaengine_terminate_sync(chan);

	return ret;
}

struct epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static bool epf_dma_filter_fn(struct dma_chan *chan, void *node)
{
	struct epf_dma_filter *filter = node;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev
		&& (filter->dma_mask & caps.directions);
}

/**
 * pci_epf_usb_init_dma_chan() - Function to initialize EPF test DMA channel
 * @epf_test: the EPF test device that performs data transfer operation
 *
 * Function to initialize EPF test DMA channel.
 */
static int pci_epf_usb_init_dma_chan(struct pci_epf_usb *epf_test)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct epf_dma_filter filter;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);
	if (!dma_chan) {
		dev_err(dev, "Failed to get private DMA rx channel. Falling back to generic one\n");
		goto fail_back_tx;
	}

	epf_test->dma_chan_rx = dma_chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);

	if (!dma_chan) {
		dev_err(dev, "Failed to get private DMA tx channel. Falling back to generic one\n");
		goto fail_back_rx;
	}

	epf_test->dma_chan_tx = dma_chan;
	epf_test->dma_private = true;

	init_completion(&epf_test->transfer_complete);

	return 0;

fail_back_rx:
	dma_release_channel(epf_test->dma_chan_rx);
	epf_test->dma_chan_rx = NULL;

fail_back_tx:
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf_test->transfer_complete);

	epf_test->dma_chan_tx = epf_test->dma_chan_rx = dma_chan;

	return 0;
}

/**
 * pci_epf_usb_clean_dma_chan() - Function to cleanup EPF test DMA channel
 * @epf_test: the EPF test device that performs data transfer operation
 *
 * Helper to cleanup EPF test DMA channel.
 */
static void pci_epf_usb_clean_dma_chan(struct pci_epf_usb *epf_test)
{
	if (!epf_test->dma_supported)
		return;

	if (epf_test->dma_chan_tx == epf_test->dma_chan_rx) {
		dma_release_channel(epf_test->dma_chan_tx);
		epf_test->dma_chan_tx = NULL;
		epf_test->dma_chan_rx = NULL;
		return;
	}

	dma_release_channel(epf_test->dma_chan_rx);
	epf_test->dma_chan_rx = NULL;
	dma_release_channel(epf_test->dma_chan_tx);
	epf_test->dma_chan_tx = NULL;

	return;
}

static int pci_epf_usb_read_into_local(struct pci_epf_usb *epf_test, struct pci_epf_usbip_reg *reg, void **out_buf)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	struct device *dma_dev = epf->epc->dev.parent;
	void __iomem *src_addr;
	void *buf = NULL;
	phys_addr_t phys_addr;
	phys_addr_t dst_phys_addr;
	u64 remote_addr;
	u32 len;
	u32 flags;
	int ret = 0;

	if (!out_buf)
		return -EINVAL;

	*out_buf = NULL;

	remote_addr = ((u64)READ_ONCE(reg->submit_src_addr_hi) << 32) | READ_ONCE(reg->submit_src_addr_lo);
	len = READ_ONCE(reg->submit_len);
	flags = READ_ONCE(reg->submit_flags) | FLAG_USE_DMA;

	if (!remote_addr || !len)
		return -EINVAL;

	dev_dbg(dev, "EP_XFER_RET_WRITE: before lock len=%u\n", len);

	mutex_lock(&epf_test->pci_xfer_lock);

	dev_dbg(dev, "EP_XFER_RET_WRITE: acquired lock len=%u\n", len);

	src_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, len);
	if (!src_addr) {
		dev_err(dev, "Failed to allocate address\n");
		reg->submit_status = STATUS_SRC_ADDR_INVALID;
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr, remote_addr, len);
	if (ret) {
		dev_err(dev, "Failed to map address\n");
		reg->submit_status = STATUS_SRC_ADDR_INVALID;
		goto out_free_addr;
	}

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out_unmap_addr;
	}

	if ((flags & FLAG_USE_DMA) && epf_test->dma_supported) {
		dst_phys_addr = dma_map_single(dma_dev, buf, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dma_dev, dst_phys_addr)) {
			ret = -ENOMEM;
			goto out_free_buf;
		}

		ret = pci_epf_usb_data_transfer(epf_test, dst_phys_addr, phys_addr,
						 len, remote_addr, DMA_DEV_TO_MEM);

		dma_unmap_single(dma_dev, dst_phys_addr, len, DMA_FROM_DEVICE);
		if (ret) {
			dev_err(dev, "USB mailbox DMA read failed: %d\n", ret);
			goto out_free_buf;
		}
	} else {
		memcpy_fromio(buf, src_addr, len);
	}

	*out_buf = buf;
	buf = NULL;

out_free_buf:
	kfree(buf);
out_unmap_addr:
	pci_epc_unmap_addr(epc, epf->func_no, epf->vfunc_no, phys_addr);
out_free_addr:
	pci_epc_mem_free_addr(epc, phys_addr, src_addr, len);

out_unlock:
	mutex_unlock(&epf_test->pci_xfer_lock);

	if (ret)
		dev_dbg(dev, "unlock :USB submit read ret=%d len=%u\n", ret, len);

	return ret;
}

static int pci_epf_usb_ret_submit(struct pci_epf_usb *epf_test,
				   struct pci_epf_usbip_reg *reg,
				   struct pci_epf_usb_tx_node *node)
{
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dma_dev = epf->epc->dev.parent;
	struct device *dev = &epf->dev;
	void __iomem *dst_addr = NULL;
	phys_addr_t phys_addr = 0;
	u64 remote_addr;
	u32 window_size;
	u32 tx_flags;
	unsigned long flags;
	bool mapped = false;
	int ret = 0;
	phys_addr_t src_phys_addr;

	spin_lock_irqsave(&epf_test->tx_mb.lock, flags);
	if (!epf_test->tx_mb.armed) {
		spin_unlock_irqrestore(&epf_test->tx_mb.lock, flags);
		return -EAGAIN;
	}

	remote_addr = epf_test->tx_mb.dst_addr;
	window_size = epf_test->tx_mb.size;
	tx_flags = epf_test->tx_mb.flags | FLAG_USE_DMA;
	epf_test->tx_mb.armed = false;
	spin_unlock_irqrestore(&epf_test->tx_mb.lock, flags);

	if (!remote_addr || !window_size || node->len > window_size)
		return -EMSGSIZE;

	dev_dbg(dev, "EP_XFER_RET_WRITE: before lock len=%u dst=0x%llx\n",
		 node->len, remote_addr);

	mutex_lock(&epf_test->pci_xfer_lock);

	dev_dbg(dev, "EP_XFER_RET_WRITE: acquired lock len=%u dst=0x%llx\n",
		 node->len, remote_addr);

	dst_addr = pci_epc_mem_alloc_addr(epc, &phys_addr, node->len);
	if (!dst_addr) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no,
			       phys_addr, remote_addr, node->len);
	if (ret)
		goto out_free_addr;

	mapped = true;

	if ((tx_flags & FLAG_USE_DMA) && epf_test->dma_supported) {
		/* DMA path here if used */
		dev_dbg(dev, "Start DMA: %s: node->len =%d\n", __func__, node->len);
		src_phys_addr = dma_map_single(dma_dev, node->buf, node->len,
				       DMA_TO_DEVICE);
		if (dma_mapping_error(dma_dev, src_phys_addr)) {
			dev_err(dev, "USB RET_SUBMIT: dma_map_single failed\n");
			ret = -ENOMEM;
			goto out_unmap;
		}

		ret = pci_epf_usb_data_transfer(epf_test,
					 phys_addr,
					 src_phys_addr,
					 node->len,
					 remote_addr,
					 DMA_MEM_TO_DEV);

		dma_unmap_single(dma_dev, src_phys_addr, node->len, DMA_TO_DEVICE);
		if (ret) {
			dev_dbg(dev, "USB RET_SUBMIT: DMA transfer failed %d\n", ret);
			goto out_unmap;
		}
		dev_dbg(dev, "End DMA: %s: node->len =%d\n", __func__, node->len);

	} else {
		dev_dbg(dev, "Non-DMA: %s: node->len =%d\n", __func__, node->len);
		memcpy_toio(dst_addr, node->buf, node->len);
		wmb();
	}

out_unmap:
	if (mapped)
		pci_epc_unmap_addr(epc, epf->func_no, epf->vfunc_no,
				    phys_addr);

out_free_addr:
	if (dst_addr)
		pci_epc_mem_free_addr(epc, phys_addr, dst_addr, node->len);

out_unlock:
	dev_dbg(dev, "EP_XFER_RET_WRITE: unlock ret=%d len=%u dst=0x%llx\n",
		 ret, node->len, remote_addr);

	mutex_unlock(&epf_test->pci_xfer_lock);

	if (ret) {
		WRITE_ONCE(reg->return_done_seq,
			   READ_ONCE(reg->return_arm_seq));
		wmb();

		WRITE_ONCE(reg->return_status,
			   READ_ONCE(reg->return_status) |
			   STATUS_USB_RETURN_SEND_FAIL);
		wmb();

		pci_epf_usb_raise_irq(epf_test, reg);
		return ret;
	}

	WRITE_ONCE(reg->return_done_seq, READ_ONCE(reg->return_arm_seq));
	WRITE_ONCE(reg->return_actual, node->len);
	wmb();

	WRITE_ONCE(reg->return_status,
		   READ_ONCE(reg->return_status) |
		   STATUS_USB_RETURN_SENT);
	wmb();

	pci_epf_usb_raise_irq(epf_test, reg);

	return node->len;
}

static void pci_epf_usb_raise_irq(struct pci_epf_usb *epf_test,
		struct pci_epf_usbip_reg *reg)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	int count;
	u32 submit_status;
	u32 return_status;
	u32 irq_number;
	u32 irq_type = le32_to_cpu(reg->irq_type);


	submit_status = READ_ONCE(reg->submit_status);
	return_status = READ_ONCE(reg->return_status);

	if (submit_status)
		WRITE_ONCE(reg->submit_status, submit_status | STATUS_IRQ_RAISED);

	if (return_status)
		WRITE_ONCE(reg->return_status, return_status | STATUS_IRQ_RAISED);

	wmb();

	irq_number = READ_ONCE(reg->irq_number);

	/*
	 * Set the status before raising the IRQ to ensure that the host sees
	 * the updated value when it gets the IRQ.
	 */

	switch (irq_type) {

	case IRQ_TYPE_INTX:
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				PCI_IRQ_INTX, 0);
		break;

	case IRQ_TYPE_MSI:
		count = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSI IRQ number %d / %d\n", irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no, PCI_IRQ_MSI, irq_number);
		break;

	case IRQ_TYPE_MSIX:
		count = pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSIX IRQ number %d / %d\n", irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no, PCI_IRQ_MSIX, irq_number);
		break;

	default:
		dev_err(dev, "Failed to raise IRQ, unknown type\n");
		break;
	}
}

static void pci_epf_usb_cmd_handler(struct work_struct *work)
{
	struct pci_epf_usb *epf_test =
		container_of(work, struct pci_epf_usb, cmd_handler.work);
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_usbip_reg *reg = epf_test->reg[test_reg_bar];
	u32 submit_command;
	u32 return_command;
	u32 submit_seq;

	submit_command = READ_ONCE(reg->submit_command);
	return_command = READ_ONCE(reg->return_command);

	if (!submit_command && !return_command)
		goto reset_handler;

	if (READ_ONCE(reg->irq_type) > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	/*
	 * Process RETURN_ARM first.
	 *
	 * Reason:
	 * If a completed RET_SUBMIT is already queued, arming the return
	 * window first allows EP to send it immediately.
	 */
	if (return_command == COMMAND_USB_RETURN_ARM) {
		int ret;
		u32 return_seq;

		WRITE_ONCE(reg->return_command, 0);
		WRITE_ONCE(reg->return_status, 0);
		WRITE_ONCE(reg->return_actual, 0);

		return_seq = READ_ONCE(reg->return_arm_seq);

#if PCI_USB_DEBUG
		dev_info(dev, "EP_USBIP: RETURN_ARM window=%u dst=0x%08x%08x flags=0x%x, RETURN_ARM seq=%u\n",
			 READ_ONCE(reg->return_window),
			 READ_ONCE(reg->return_dst_addr_hi),
			 READ_ONCE(reg->return_dst_addr_lo),
			 READ_ONCE(reg->return_flags), return_seq);
#endif

		ret = pci_epf_usb_arm_tx_mailbox(epf_test, reg);
		if (ret) {
			WRITE_ONCE(reg->return_done_seq, return_seq);
			wmb();
			dev_dbg(dev, "Failed to arm USB return mailbox: %d\n", ret);
			WRITE_ONCE(reg->return_status, STATUS_USB_RETURN_ARM_FAIL | STATUS_DST_ADDR_INVALID);
			wmb();
			pci_epf_usb_raise_irq(epf_test, reg);
		} else {
			WRITE_ONCE(reg->return_done_seq, return_seq);
			wmb();
			WRITE_ONCE(reg->return_status, STATUS_USB_RETURN_ARMED);
			wmb();
			pci_epf_usb_raise_irq(epf_test, reg);
			/*
			 * RC has armed one return window.
			 * Try to send one queued RET_SUBMIT/RET_UNLINK node.
			 */
			queue_work(kpciusb_workqueue, &epf_test->txq.work);
		}
	} else if (return_command) {
		dev_dbg(dev, "Invalid return_command=0x%x\n", return_command);
		WRITE_ONCE(reg->return_command, 0);
		WRITE_ONCE(reg->return_status, STATUS_USB_RETURN_ARM_FAIL);
		wmb();
		pci_epf_usb_raise_irq(epf_test, reg);
	}

	if (submit_command == COMMAND_USB_SUBMIT) {
		void *local_buf = NULL;
		u32 submit_len;
		u32 submit_seq;
		int ret;

		WRITE_ONCE(reg->submit_command, 0);
		WRITE_ONCE(reg->submit_status, 0);

		submit_len = READ_ONCE(reg->submit_len);
		submit_seq = READ_ONCE(reg->submit_seq);

#if PCI_USB_DEBUG
		dev_info(dev, "EP_USBIP: SUBMIT len=%u src=0x%08x%08x flags=0x%x, seq=%u\n",
			 submit_len,
			 READ_ONCE(reg->submit_src_addr_hi),
			 READ_ONCE(reg->submit_src_addr_lo),
			 READ_ONCE(reg->submit_flags), submit_seq);
#endif

		if ((READ_ONCE(reg->submit_flags) & FLAG_USE_DMA) && !epf_test->dma_supported) {
			dev_dbg(dev, "Cannot transfer submit using DMA\n");
			WRITE_ONCE(reg->submit_done_seq, submit_seq);
			WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_FAIL);
			wmb();
			pci_epf_usb_raise_irq(epf_test, reg);
			goto reset_handler;
		}

		ret = pci_epf_usb_read_into_local(epf_test, reg, &local_buf);
		if (ret) {
			dev_dbg(dev, "USB submit mailbox read failed: %d\n", ret);
			WRITE_ONCE(reg->submit_done_seq, submit_seq);
			WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_FAIL);
			wmb();
			pci_epf_usb_raise_irq(epf_test, reg);
			goto reset_handler;
		}

		if (!local_buf) {
			dev_dbg(dev, "USB submit read returned NULL buffer\n");
			WRITE_ONCE(reg->submit_done_seq, submit_seq);
			WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_FAIL);
			wmb();
			pci_epf_usb_raise_irq(epf_test, reg);
			goto reset_handler;
		}

#if PCI_USB_DEBUG
		print_hex_dump(KERN_INFO, "DBG_SUBMIT_READ_RAW: ",
			       DUMP_PREFIX_OFFSET, 16, 1,
			       local_buf,
			       min_t(u32, submit_len, 64),
			       false);
#endif

		ret = pci_epf_usb_stream_store(epf_test, local_buf, submit_len);
		if (ret) {
			WRITE_ONCE(reg->submit_done_seq, submit_seq);
			dev_dbg(dev, "Failed to stage USB submit payload: %d\n", ret);
			kfree(local_buf);
			WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_FAIL);
		} else {
			WRITE_ONCE(reg->submit_done_seq, submit_seq);
			WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_SUCCESS);
		}

		wmb();
		pci_epf_usb_raise_irq(epf_test, reg);
	} else if (submit_command) {
		dev_dbg(dev, "Invalid submit_command=0x%x\n", submit_command);

		WRITE_ONCE(reg->submit_done_seq, submit_seq);
		wmb();
		WRITE_ONCE(reg->submit_command, 0);
		wmb();
		WRITE_ONCE(reg->submit_status, STATUS_USB_SUBMIT_FAIL);
		wmb();
		pci_epf_usb_raise_irq(epf_test, reg);
	}

reset_handler:
	queue_delayed_work(kpciusb_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));
}

static int pci_epf_usb_set_bar(struct pci_epf *epf)
{
	int bar, ret;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, &epf->bar[bar]);
		if (ret) {
			pci_epf_free_space(epf, epf_test->reg[bar], bar, PRIMARY_INTERFACE);
			epf_test->reg[bar] = NULL;
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == test_reg_bar)
				return ret;
		}
	}

	return 0;
}

static void pci_epf_usb_clear_bar(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, &epf->bar[bar]);
	}
}

static int pci_epf_usb_epc_init(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features = epf_test->epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool linkup_notifier = false;
	int ret;

	epf_test->dma_supported = true;

	ret = pci_epf_usb_init_dma_chan(epf_test);
	if (ret)
		epf_test->dma_supported = false;

	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no, header);
		if (ret) {
			dev_err(dev, "Configuration header write failed\n");
			return ret;
		}
	}

	ret = pci_epf_usb_set_bar(epf);
	if (ret)
		return ret;

	if (epc_features->msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (epc_features->msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       epf_test->test_reg_bar,
				       epf_test->msix_table_offset);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	linkup_notifier = epc_features->linkup_notifier;
	if (!linkup_notifier)
		queue_work(kpciusb_workqueue, &epf_test->cmd_handler.work);

	return 0;
}

static void pci_epf_usb_epc_deinit(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_test->cmd_handler);
	flush_work(&epf_test->txq.work);
	pci_epf_usb_stream_close(epf_test);
	pci_epf_usb_txq_purge(&epf_test->txq);
	pci_epf_usb_tx_mb_clear(&epf_test->tx_mb);
	pci_epf_usb_clean_dma_chan(epf_test);
	pci_epf_usb_clear_bar(epf);
}

static int pci_epf_usb_link_up(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);

	queue_delayed_work(kpciusb_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));

	return 0;
}

static int pci_epf_usb_link_down(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_test->cmd_handler);
	pci_epf_usb_stream_close(epf_test);
	flush_work(&epf_test->txq.work);
	pci_epf_usb_txq_purge(&epf_test->txq);
	pci_epf_usb_tx_mb_clear(&epf_test->tx_mb);
	return 0;
}

static const struct pci_epc_event_ops pci_epf_usb_event_ops = {
	.epc_init = pci_epf_usb_epc_init,
	.epc_deinit = pci_epf_usb_epc_deinit,
	.link_up = pci_epf_usb_link_up,
	.link_down = pci_epf_usb_link_down,
};

static int pci_epf_usb_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	size_t msix_table_size = 0;
	size_t test_reg_bar_size;
	size_t pba_size = 0;
	void *base;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	enum pci_barno bar;
	const struct pci_epc_features *epc_features = epf_test->epc_features;
	size_t test_reg_size;

	test_reg_bar_size = ALIGN(sizeof(struct pci_epf_usbip_reg), 128);

	if (epc_features->msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_test->msix_table_offset = test_reg_bar_size;
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	test_reg_size = test_reg_bar_size + msix_table_size + pba_size;

	base = pci_epf_alloc_space(epf, test_reg_size, test_reg_bar,
				   epc_features, PRIMARY_INTERFACE);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}
	epf_test->reg[test_reg_bar] = base;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		bar = pci_epc_get_next_free_bar(epc_features, bar);
		if (bar == NO_BAR)
			break;

		if (bar == test_reg_bar)
			continue;

		base = pci_epf_alloc_space(epf, bar_size[bar], bar,
					   epc_features, PRIMARY_INTERFACE);
		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n", bar);
		epf_test->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_usb_free_space(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		pci_epf_free_space(epf, epf_test->reg[bar], bar,
				   PRIMARY_INTERFACE);
		epf_test->reg[bar] = NULL;
	}
}

static int pci_epf_usb_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno test_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	test_reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (test_reg_bar < 0)
		return -EINVAL;

	epf_test->test_reg_bar = test_reg_bar;
	epf_test->epc_features = epc_features;

	ret = pci_epf_usb_alloc_space(epf);
	if (ret)
		return ret;

	return 0;
}

static void pci_epf_usb_unbind(struct pci_epf *epf)
{
	struct pci_epf_usb *epf_test = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	cancel_delayed_work_sync(&epf_test->cmd_handler);
	flush_work(&epf_test->txq.work);
	pci_epf_usb_stream_close(epf_test);
	pci_epf_usb_txq_purge(&epf_test->txq);
	pci_epf_usb_tx_mb_clear(&epf_test->tx_mb);
	if (epc->init_complete) {
		pci_epf_usb_clean_dma_chan(epf_test);
		pci_epf_usb_clear_bar(epf);
	}
	pci_epf_usb_free_space(epf);
}

static const struct pci_epf_device_id pci_epf_usb_ids[] = {
	{
		.name = "pci_epf_usb",
	},
	{},
};

static int pci_epf_usb_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_usb *epf_test;
	struct device *dev = &epf->dev;
	struct usbip_pcie_ctx *ctx;

	epf_test = devm_kzalloc(dev, sizeof(*epf_test), GFP_KERNEL);
	if (!epf_test)
		return -ENOMEM;


	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	epf->header = &test_header;
	epf_test->epf = epf;
	ctx->epf = epf;
	ctx->ops = &epf_usbip_ops;
	epf_test->usb_ctx = ctx;

	usbip_stub_set_ctx(ctx);

	pci_epf_usb_stream_init(&epf_test->rx_stream);

	INIT_DELAYED_WORK(&epf_test->cmd_handler, pci_epf_usb_cmd_handler);
	INIT_WORK(&epf_test->txq.work, pci_epf_usb_tx_handler);
	mutex_init(&epf_test->pci_xfer_lock);
	pci_epf_usb_txq_init(&epf_test->txq);
	pci_epf_usb_tx_mb_init(&epf_test->tx_mb);



	epf->event_ops = &pci_epf_usb_event_ops;

	epf_set_drvdata(epf, epf_test);
	return 0;
}


static const struct pci_epf_ops ops = {
	.unbind	= pci_epf_usb_unbind,
	.bind	= pci_epf_usb_bind,
};

static struct pci_epf_driver test_driver = {
	.driver.name	= "pci_epf_usb",
	.probe		= pci_epf_usb_probe,
	.id_table	= pci_epf_usb_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_usb_init(void)
{
	int ret;

	kpciusb_workqueue = alloc_workqueue("kpciusbp",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpciusb_workqueue) {
		pr_err("Failed to allocate the kpciusb work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&test_driver);
	if (ret) {
		destroy_workqueue(kpciusb_workqueue);
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_usb_init);

static void __exit pci_epf_usb_exit(void)
{
	pci_epf_unregister_driver(&test_driver);

	if (kpciusb_workqueue)
		destroy_workqueue(kpciusb_workqueue);
}
module_exit(pci_epf_usb_exit);

MODULE_DESCRIPTION("PCI EPF USB URB TRANSPORT DRIVER");
MODULE_AUTHOR("NXP");
MODULE_LICENSE("GPL v2");
