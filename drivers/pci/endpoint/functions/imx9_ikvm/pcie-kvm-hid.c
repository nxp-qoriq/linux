// SPDX-License-Identifier: GPL-2.0
/*
* PCIe Endpoint HID helper library
*
* Provides helper APIs to initialize a PCIe HID ring buffer located in
* BAR3 and to send HID reports (keyboard, relative mouse, and absolute
* mouse) to the host through shared memory.
*
* During module initialization, the driver locates the target PCI device,
* maps BAR3, initializes a global HID context, and exposes helper
* functions for publishing HID reports into the shared ring buffer.
*
* Copyright 2026 NXP
*/

#define pr_fmt(fmt) "pcie-hid-epf: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/string.h>

#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci-epf-hid.h>

extern void* pci_epf_get_hid_address(void);

static struct epf_hid g_hid;

static int epf_hid_send_report(u8 type, const void *report, size_t len)
{
	struct epf_hid *hid = &g_hid;
	struct hid_desc *desc;
	unsigned long flags;
	u32 idx;

	if (!hid->ring) {
		pr_err("%s: HID ring not initialized\n", __func__);
		return -ENODEV;
	}

	if (!report) {
		pr_err("%s: report is NULL\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&hid->lock, flags);

	idx = hid->prod % HID_RING_SIZE;
	desc = &hid->ring[idx];

	desc->type = type;
	desc->len = len;
	desc->seq = ++hid->seq;

	memcpy_toio(desc->data, report, len);

	/* Ensure descriptor contents are visible before producer index */
	smp_wmb();

	WRITE_ONCE(hid->prod, hid->prod + 1);

	spin_unlock_irqrestore(&hid->lock, flags);

	return 0;
}

int epf_hid_send_rel_mouse(struct hid_rel_mouse_report *rpt)
{
	return epf_hid_send_report(HID_TYPE_REL_MOUSE, rpt, sizeof(*rpt));
}

int epf_hid_send_abs_mouse(struct hid_abs_mouse_report *rpt)
{
	return epf_hid_send_report(HID_TYPE_ABS_MOUSE, rpt, sizeof(*rpt));
}

int epf_hid_send_keyboard(struct hid_keyboard_report *rpt)
{
	return epf_hid_send_report(HID_TYPE_KEYBOARD, rpt, sizeof(*rpt));
}

int epf_hid_init(void)
{
	struct epf_hid *hid = &g_hid;
	size_t ring_size;
	int ret;
	
	if (!pci_epf_get_hid_address())
		return -EINVAL;

	memset(hid, 0, sizeof(*hid));

	spin_lock_init(&hid->lock);

	hid->bar = pci_epf_get_hid_address();
	hid->ring = (struct hid_desc *)pci_epf_get_hid_address();

	ring_size = sizeof(struct hid_desc) * HID_RING_SIZE;

	pr_info("Initialized HID ring: %u descriptors (%zu bytes)\n",
		HID_RING_SIZE, ring_size);

	ret = rel_mouse_init();
	if (ret) {
		pr_err("rel_mouse_init failed: %d\n", ret);
		goto err_rel_mouse;
	}

	ret = abs_mouse_init();
	if (ret) {
		pr_err("abs_mouse_init failed: %d\n", ret);
		goto err_abs_mouse;
	}

	ret = keyboard_init();
	if (ret) {
		pr_err("keyboard_init failed: %d\n", ret);
		goto err_keyboard;
	}

	ret = local_keyboard_init();
	if (ret) {
		pr_err("local_keyboard_init failed: %d\n", ret);
		goto err_local_keyboard;
	}

	return 0;


err_local_keyboard:
	keyboard_exit();

err_keyboard:
	abs_mouse_exit();

err_abs_mouse:
	rel_mouse_exit();

err_rel_mouse:
	return ret;
}

void epf_hid_exit(void)
{
	local_keyboard_exit();

	keyboard_exit();

	abs_mouse_exit();

	rel_mouse_exit();

	pr_info("HID helper exited\n");
}
module_init(epf_hid_init);
module_exit(epf_hid_exit);

MODULE_DESCRIPTION("PCIe HID BAR3 helper");
MODULE_AUTHOR("Shreyash Damanwala <shreyash.damanwala@nxp.com>");
MODULE_LICENSE("GPL v2");