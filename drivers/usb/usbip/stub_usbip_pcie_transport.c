// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2026 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uio.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/errno.h>

#include "usbip_common.h"

#define USBIP_PCIE_MAX_PACKET_SIZE SZ_4M

struct usbip_pcie_glue_state {
	struct usbip_pcie_ctx *ctx;
	struct task_struct *dispatcher;
	struct list_head channels;
	spinlock_t channels_lock;
	/*
	 * Serialize all physical EP->RC return writes.
	 * Multiple stub_tx_loop threads can call usbip_xmit_pcie()
	 * concurrently for different devid channels.
	 */
	struct mutex tx_lock;
	bool stopping;
};

static struct usbip_pcie_glue_state g_pcie_glue;

static struct usbip_pcie_channel *usbip_pcie_find_channel_locked(__u32 devid)
{
	struct usbip_pcie_channel *ch;

	list_for_each_entry(ch, &g_pcie_glue.channels, node) {
		if (ch->devid == devid)
			return ch;
	}

	return NULL;
}

static void usbip_pcie_channel_rx_purge(struct usbip_pcie_channel *ch)
{
	struct usbip_pcie_rx_node *n, *tmp;
	unsigned long flags;

	if (!ch)
	return;

	spin_lock_irqsave(&ch->rx_lock, flags);

	list_for_each_entry_safe(n, tmp, &ch->rx_queue, list) {
		list_del(&n->list);
	       kfree(n->buf);
		kfree(n);
	}

	spin_unlock_irqrestore(&ch->rx_lock, flags);
}

static int usbip_pcie_channel_enqueue(struct usbip_pcie_channel *ch, void *buf, u32 len)
{
	struct usbip_pcie_rx_node *n;
	unsigned long flags;

	if (!ch || !buf || !len)
		return -EINVAL;

	n = kzalloc(sizeof(*n), GFP_KERNEL);
	if (!n)
		return -ENOMEM;

	n->buf = buf;
	n->len = len;
	n->off = 0;

	spin_lock_irqsave(&ch->rx_lock, flags);
	list_add_tail(&n->list, &ch->rx_queue);
	spin_unlock_irqrestore(&ch->rx_lock, flags);

	pr_debug("EP_PCIE_CH_ENQUEUE: devid=0x%x len=%u\n", ch->devid, len);

	wake_up_interruptible(&ch->rx_wait);

	return 0;
}

static int usbip_pcie_cmd_payload_len(struct usbip_header *hdr)
{
	/*
	 * USBIP_CMD_SUBMIT:
	 *   OUT submit carries payload after the 48-byte header.
	 *   IN submit does not carry transfer payload from RC to EP.
	 */
	if (hdr->base.command == USBIP_CMD_SUBMIT) {
		if (hdr->base.direction == USBIP_DIR_OUT &&
				hdr->u.cmd_submit.transfer_buffer_length > 0)
			return hdr->u.cmd_submit.transfer_buffer_length;
	}

	return 0;
}

static int usbip_pcie_rx_dispatcher(void *data)
{
	struct usbip_pcie_ctx *ctx = data;

	while (!kthread_should_stop()) {
		struct usbip_header raw_hdr;
		struct usbip_header hdr;
		struct usbip_pcie_channel *ch;
		void *pkt = NULL;
		int payload_len;
		int total_len;
		int ret;
		unsigned long flags;

		if (READ_ONCE(g_pcie_glue.stopping))
			break;

		ret = ctx->ops->recv_stream(ctx, &raw_hdr, sizeof(raw_hdr));
		if (ret < 0) {
			if (ret == -ESHUTDOWN || ret == -ENOTCONN) {
				break;
			}

			if (ret == -EOVERFLOW) {
				continue;
			}

			continue;
		}

		if (ret != sizeof(struct usbip_header)) {
			continue;
		}

		memcpy(&hdr, &raw_hdr, sizeof(hdr));
		usbip_header_correct_endian(&hdr, 0);

		if (hdr.base.command != USBIP_CMD_SUBMIT &&
		    hdr.base.command != USBIP_CMD_UNLINK) {
			continue;
		}

		payload_len = usbip_pcie_cmd_payload_len(&hdr);
		if (payload_len < 0) {
			continue;
		}

		/*
		 * Important:
		 * Sometimes lower PCIe layer reports a larger physical packet
		 * than the USB/IP command actually needs. For example:
		 *
		 *   COMMAND_USB_SUBMIT received len = 56
		 *   USBIP_CMD_SUBMIT dir=IN transfer_buffer_length=4
		 *
		 * For IN submit, the USB/IP command itself is only the
		 * 48-byte header. The extra bytes must not be queued to the
		 * per-device virtual stream because stub_rx_pdu() will read
		 * them as the next header and the stream will become corrupt.
		 */
		if (ret > payload_len) {
			pr_debug("EP_PCIE_DISPATCH: physical_len=%d \
					expected=%d discard_tail=%d cmd=%u devid=0x%x\n",
					ret, payload_len, ret-payload_len, hdr.base.command, hdr.base.devid);
		}

		total_len = sizeof(raw_hdr) + payload_len;

		pkt = kmalloc(total_len, GFP_KERNEL);
		if (!pkt)
			continue;

		memcpy(pkt, &raw_hdr, sizeof(raw_hdr));

		if (payload_len) {
			ret = ctx->ops->recv_stream(ctx, (u8 *)pkt + sizeof(raw_hdr), payload_len);
			if (ret != payload_len) {
				if (ret == -EOVERFLOW)
					pr_debug("EP_PCIE_DISPATCH: discarded stale \
							partial payload tail expected=%d\n", payload_len);
				else
					pr_debug("EP_PCIE_DISPATCH: payload ret=%d expected=%d\n", ret, payload_len);
				kfree(pkt);
				continue;
			}
		}

		spin_lock_irqsave(&g_pcie_glue.channels_lock, flags);
		ch = usbip_pcie_find_channel_locked(hdr.base.devid);
		spin_unlock_irqrestore(&g_pcie_glue.channels_lock, flags);

		if (!ch) {
			pr_debug("EP_PCIE_DISPATCH: no channel devid=0x%x cmd=%u len=%d\n",
					hdr.base.devid, hdr.base.command, payload_len);
			kfree(pkt);
			continue;
		}

		ret = usbip_pcie_channel_enqueue(ch, pkt, total_len);
		if (ret < 0) {
			pr_debug("EP_PCIE_DISPATCH: enqueue failed devid=0x%x ret=%d\n",
					hdr.base.devid, ret);
			kfree(pkt);
			continue;
		}
	}

	return 0;
}

static int usbip_pcie_start_dispatcher(struct usbip_pcie_ctx *ctx)
{
	if (!ctx || !ctx->ops || !ctx->ops->recv_stream)
		return -EINVAL;

	if (g_pcie_glue.dispatcher)
		return 0;

	WRITE_ONCE(g_pcie_glue.stopping, false);

	g_pcie_glue.dispatcher = kthread_run(usbip_pcie_rx_dispatcher,
			ctx,
			"usbip_pcie_rx_dispatch");
	if (IS_ERR(g_pcie_glue.dispatcher)) {
		int ret = PTR_ERR(g_pcie_glue.dispatcher);

		g_pcie_glue.dispatcher = NULL;
		return ret;
	}

	return 0;
}

void usbip_stub_set_ctx(struct usbip_pcie_ctx *ctx)
{
	if (WARN_ON_ONCE(!ctx))
		return;

	if (WARN_ON_ONCE(g_pcie_glue.ctx))
		return;

	g_pcie_glue.ctx = ctx;
	INIT_LIST_HEAD(&g_pcie_glue.channels);
	spin_lock_init(&g_pcie_glue.channels_lock);
	mutex_init(&g_pcie_glue.tx_lock);
	g_pcie_glue.dispatcher = NULL;
	g_pcie_glue.stopping = false;
}
EXPORT_SYMBOL_GPL(usbip_stub_set_ctx);

struct usbip_pcie_ctx *usbip_stub_get_ctx(void)
{
	return g_pcie_glue.ctx;
}
EXPORT_SYMBOL_GPL(usbip_stub_get_ctx);

struct usbip_pcie_channel *usbip_pcie_channel_register(struct usbip_pcie_ctx *ctx, void *owner, __u32 devid)
{
	struct usbip_pcie_channel *ch;
	unsigned long flags;
	int ret;

	if (!ctx || !owner || !devid)
		return ERR_PTR(-EINVAL);

	ret = usbip_pcie_start_dispatcher(ctx);
	if (ret)
		return ERR_PTR(ret);

	ch = kzalloc(sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&ch->node);
	INIT_LIST_HEAD(&ch->rx_queue);
	spin_lock_init(&ch->rx_lock);
	init_waitqueue_head(&ch->rx_wait);

	ch->parent = ctx;
	ch->owner = owner;
	ch->devid = devid;
	ch->shutting_down = false;

	spin_lock_irqsave(&g_pcie_glue.channels_lock, flags);
	list_add_tail(&ch->node, &g_pcie_glue.channels);
	spin_unlock_irqrestore(&g_pcie_glue.channels_lock, flags);

	pr_debug("EP_PCIE_CH_REGISTER: devid=0x%x ch=%p owner=%p\n",
		devid, ch, owner);

	return ch;
}
EXPORT_SYMBOL_GPL(usbip_pcie_channel_register);

void usbip_pcie_channel_unregister(struct usbip_pcie_channel *ch)
{
	unsigned long flags;

	if (!ch)
		return;

	WRITE_ONCE(ch->shutting_down, true);
	wake_up_all(&ch->rx_wait);

	spin_lock_irqsave(&g_pcie_glue.channels_lock, flags);
	list_del_init(&ch->node);
	spin_unlock_irqrestore(&g_pcie_glue.channels_lock, flags);

	usbip_pcie_channel_rx_purge(ch);

	pr_debug("EP_PCIE_CH_UNREGISTER: devid=0x%x ch=%p\n",
		ch->devid, ch);

	kfree(ch);
}
EXPORT_SYMBOL_GPL(usbip_pcie_channel_unregister);

int usbip_pcie_recv_stream(struct usbip_pcie_channel *ch, void *buf, int size)
{
	struct usbip_pcie_rx_node *n;
	unsigned long flags;
	int copied = 0;
	int ret;

	if (!ch || !buf || size <= 0)
		return -EINVAL;

	while (copied < size) {
		ret = wait_event_interruptible(ch->rx_wait,
				READ_ONCE(ch->shutting_down) ||
				!list_empty_careful(&ch->rx_queue));
		if (ret)
			return copied ? copied : ret;

		spin_lock_irqsave(&ch->rx_lock, flags);

		if (list_empty(&ch->rx_queue)) {
			bool down = ch->shutting_down;

			spin_unlock_irqrestore(&ch->rx_lock, flags);

			if (down)
				return copied ? copied : -ESHUTDOWN;

			continue;
		}

		n = list_first_entry(&ch->rx_queue,
				struct usbip_pcie_rx_node,
				list);

		if (n->off < n->len) {
			size_t chunk = min_t(size_t,
					size - copied,
					n->len - n->off);

			memcpy((u8 *)buf + copied,
					(u8 *)n->buf + n->off,
					chunk);

			n->off += chunk;
			copied += chunk;
		}

		if (n->off >= n->len) {
			list_del(&n->list);
			spin_unlock_irqrestore(&ch->rx_lock, flags);
			kfree(n->buf);
			kfree(n);
			continue;
		}

		spin_unlock_irqrestore(&ch->rx_lock, flags);
	}

	return copied;
}
EXPORT_SYMBOL_GPL(usbip_pcie_recv_stream);

int usbip_xmit_pcie(struct usbip_pcie_channel *ch, struct msghdr *msg, size_t txsize)
{
	struct usbip_pcie_ctx *ctx;
	void *linear;
	size_t copied;
	int ret;

	if (!ch || !msg || txsize == 0)
		return -EINVAL;

	ctx = ch->parent;
	if (!ctx || !ctx->epf || !ctx->ops || !ctx->ops->send_stream)
		return -ENOTCONN;

	if (txsize > USBIP_PCIE_MAX_PACKET_SIZE || txsize > INT_MAX)
		return -EMSGSIZE;

	linear = kmalloc(txsize, GFP_KERNEL);
	if (!linear)
		return -ENOMEM;

	copied = copy_from_iter(linear, txsize, &msg->msg_iter);
	if (copied != txsize) {
		kfree(linear);
		return -EFAULT;
	}

	/*
	 * Keep generic USB/IP mostly untouched.
	 * If old RET_* path generated devid=0, stamp this channel's devid here.
	 * At this stage PDU is already in network endian, so use be32 helpers.
	 */
	if (txsize >= sizeof(struct usbip_header)) {
		struct usbip_header *pdu = linear;
		__u32 cmd = be32_to_cpu(pdu->base.command);
		__u32 devid = be32_to_cpu(pdu->base.devid);

		if ((cmd == USBIP_RET_SUBMIT || cmd == USBIP_RET_UNLINK) &&
				devid == 0) {
			pdu->base.devid = cpu_to_be32(ch->devid);
			pr_debug("EP_PCIE_TX: stamped devid=0x%x cmd=%u\n", ch->devid, cmd);
		}
	}

	mutex_lock(&g_pcie_glue.tx_lock);
	ret = ctx->ops->send_stream(ctx, linear, (int)txsize);
	mutex_unlock(&g_pcie_glue.tx_lock);

	kfree(linear);

	if (ret < 0)
		return ret;

	return txsize;
}
EXPORT_SYMBOL_GPL(usbip_xmit_pcie);
