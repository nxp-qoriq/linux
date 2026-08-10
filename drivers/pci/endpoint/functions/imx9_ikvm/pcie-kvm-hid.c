// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2026 NXP
 *
 * PCIe Endpoint HID helper library
 *
 * Provides helper APIs to initialize a PCIe HID ring buffer located in
 * BAR3 and to send HID reports (keyboard, relative mouse, and absolute
 * mouse) to the host through shared memory.
 *
 * BAR3 shared-memory layout:
 *
 * BAR3 + 0x0000
 *     struct hid_ring_header
 *
 * BAR3 + 0x0010
 *     struct hid_desc[HID_RING_SIZE]
 *
 * Producer and consumer indices are stored in BAR3 so that both the
 * endpoint and PCIe host can observe ring ownership/progress.
 *
 * Initialization model:
 *
 * module_init() only registers input handlers and cdevs. The HID ring
 * is NOT touched at module load time. The VGA EPF driver calls
 * epf_hid_attach() from epc_init() once BAR3 is mapped, and
 * epf_hid_detach() from epc_deinit()/unbind() before BAR3 is unmapped.
 * This avoids the race where the module loads before the PCIe link is up.
 */

#define pr_fmt(fmt) "pcie-hid-epf: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/string.h>

#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/types.h>

#include "pci-epf-hid.h"


static_assert(HID_SHARED_SIZE <= HID_BAR3_SIZE);

static struct epf_hid g_hid;
static u32 g_hid_seq;

static inline void epf_hid_publish_prod(struct epf_hid *hid, u32 prod)
{
	wmb();
	iowrite32(prod, &hid->hdr->prod);
}

static inline u32 epf_hid_read_cons(struct epf_hid *hid)
{
	return ioread32(&hid->hdr->cons);
}

static int epf_hid_send_report(u32 type, const void *report, size_t len)
{
	struct epf_hid *hid = &g_hid;
	struct hid_desc __iomem *desc;
	unsigned long flags;
	u32 prod;
	u32 cons;
	u32 idx;
	u32 next_prod;
	u32 seq;

	if (!hid->ring || !hid->hdr) {
		pr_err_ratelimited("%s: HID ring not initialized\n", __func__);
		return -ENODEV;
	}

	if (!report) {
		pr_err("%s: report is NULL\n", __func__);
		return -EINVAL;
	}

	if (len > sizeof(desc->data)) {
		pr_err("%s: HID report too large: %zu > %zu\n",
		       __func__, len, sizeof(desc->data));
		return -EMSGSIZE;
	}

	spin_lock_irqsave(&hid->lock, flags);

	prod = ioread32(&hid->hdr->prod);
	cons = epf_hid_read_cons(hid);
	next_prod = prod + 1;

	if ((prod - cons) > HID_RING_SIZE) {
		pr_warn("%s: corrupt cons=%u prod=%u, resetting\n",
							__func__, cons, prod);
		iowrite32(prod, &hid->hdr->cons);
		cons = prod;
	}

	if ((next_prod - cons) >= HID_RING_SIZE) {
		pr_warn("%s: HID ring full: prod=%u cons=%u\n",
			__func__, prod, cons);
		spin_unlock_irqrestore(&hid->lock, flags);
		return -ENOSPC;
	}

	idx = prod % HID_RING_SIZE;
	desc = &hid->ring[idx];
	seq = ++g_hid_seq;

	iowrite32(type,     &desc->type);
	iowrite32((u32)len, &desc->len);
	iowrite32(seq,      &desc->seq);
	iowrite32(0,        &desc->flags);

	memcpy_toio(desc->data, report, len);

	wmb();

	iowrite32(seq, &hid->hdr->seq);
	epf_hid_publish_prod(hid, next_prod);

	spin_unlock_irqrestore(&hid->lock, flags);

	return 0;
}

int epf_hid_send_rel_mouse(struct hid_rel_mouse_report *rpt)
{
	if (!rpt) {
		pr_err("%s: report is NULL\n", __func__);
		return -EINVAL;
	}

	return epf_hid_send_report(HID_TYPE_REL_MOUSE, rpt, sizeof(*rpt));
}

int epf_hid_send_abs_mouse(struct hid_abs_mouse_report *rpt)
{
	if (!rpt) {
		pr_err("%s: report is NULL\n", __func__);
		return -EINVAL;
	}

	return epf_hid_send_report(HID_TYPE_ABS_MOUSE, rpt, sizeof(*rpt));
}

int epf_hid_send_keyboard(struct hid_keyboard_report *rpt)
{
	if (!rpt) {
		pr_err("%s: report is NULL\n", __func__);
		return -EINVAL;
	}

	return epf_hid_send_report(HID_TYPE_KEYBOARD, rpt, sizeof(*rpt));
}

/**
 * epf_hid_attach() - attach the HID ring to a mapped BAR3 region
 * @bar: virtual __iomem address of the BAR3 shared-memory region
 *
 * Called by the VGA EPF driver from epc_init() after BAR3 is mapped.
 * Initializes the shared ring header and wires up the ring pointers.
 * Input handlers and cdevs are already registered at module_init time
 * and will start forwarding reports as soon as this returns.
 *
 * Return: 0 on success, negative error code on failure.
 */
int epf_hid_attach(void __iomem *bar)
{
	struct epf_hid *hid = &g_hid;
	size_t ring_size  = sizeof(struct hid_desc) * HID_RING_SIZE;
	size_t total_size = sizeof(struct hid_ring_header) + ring_size;
	int ret;
	
   /* Singleton guard */
   if (WARN_ON(atomic_read(&hid->attached))) {
       pr_err("%s: already attached — only one EP instance supported\n");
       return -EBUSY;
   }

	if (!bar) {
		pr_err("%s: BAR3 address is NULL\n", __func__);
		return -EINVAL;
	}

	if (total_size > HID_BAR3_SIZE) {
		pr_err("%s: HID ring does not fit BAR3: %zu > %u\n",
		       __func__, total_size, HID_BAR3_SIZE);
		return -EINVAL;
	}

	spin_lock_init(&hid->lock);

	hid->bar  = bar;
	hid->hdr  = (struct hid_ring_header __iomem *)bar;
	hid->ring = (struct hid_desc __iomem *)
		    ((u8 __iomem *)bar + sizeof(struct hid_ring_header));

	/* Reset EP-local sequence counter */
	g_hid_seq = 0;

	/* Clear the shared HID region and initialize header fields */
	memset_io(hid->bar, 0, total_size);
	iowrite32(0, &hid->hdr->prod);
	iowrite32(0, &hid->hdr->cons);
	iowrite32(0, &hid->hdr->seq);
	iowrite32(0, &hid->hdr->reserved);

	pr_info("%s: HID ring attached: %u descriptors, "
		"header=%zu desc=%zu ring=%zu total=%zu BAR3=%u bytes\n",
		__func__, HID_RING_SIZE,
		sizeof(struct hid_ring_header), sizeof(struct hid_desc),
		ring_size, total_size, HID_BAR3_SIZE);

   /* Register input handlers now that BAR3 is ready */
   ret = rel_mouse_init();
   if (ret)
   		goto err_rel_mouse;

   ret = abs_mouse_init();
   if (ret)
   	   goto err_abs_mouse;

   ret = keyboard_init();
   if (ret)
       goto err_keyboard;

   ret = local_keyboard_init();
   if (ret)
       goto err_local_keyboard;

    atomic_set(&hid->attached, 1);

    pr_info("%s: HID ring attached\n", __func__);
    return 0;

err_local_keyboard:
   keyboard_exit();
err_keyboard:
   abs_mouse_exit();
err_abs_mouse:
   rel_mouse_exit();
err_rel_mouse:
   hid->ring = NULL;
   hid->hdr  = NULL;
   hid->bar  = NULL;
   return ret;
}
EXPORT_SYMBOL_GPL(epf_hid_attach);

void epf_hid_detach(void)
{
	struct epf_hid *hid = &g_hid;
	unsigned long flags;

	spin_lock_irqsave(&hid->lock, flags);
	if (!hid->ring && !hid->hdr && !hid->bar) {
		spin_unlock_irqrestore(&hid->lock, flags);
		pr_debug("%s: already detached, skipping\n", __func__);
		return;
	}

	hid->ring = NULL;
	hid->hdr  = NULL;
	hid->bar  = NULL;
	atomic_set(&hid->attached, 0);
	spin_unlock_irqrestore(&hid->lock, flags);

	local_keyboard_exit();
	keyboard_exit();
	abs_mouse_exit();
	rel_mouse_exit();

	pr_info("%s: HID ring detached\n", __func__);
}
EXPORT_SYMBOL_GPL(epf_hid_detach);

static int __init epf_hid_init(void)
{
    memset(&g_hid, 0, sizeof(g_hid));
    spin_lock_init(&g_hid.lock);
	pr_info("%s: PCIe HID helper ready\n", __func__);
	return 0;
}

static void __exit epf_hid_exit(void)
{
	epf_hid_detach();
	pr_info("%s: PCIe HID helper exited\n", __func__);
}

module_init(epf_hid_init);
module_exit(epf_hid_exit);
MODULE_DESCRIPTION("PCIe HID BAR3 helper");
MODULE_AUTHOR("Shreyash Damanwala <shreyash.damanwala@nxp.com>");
MODULE_LICENSE("GPL v2");

