/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2026 NXP
 *
 * PCI Endpoint HID Function with pci-epf-imx9-vga function
 *
 * This header defines the common data structures and APIs used by the
 * PCI Endpoint HID function for handling keyboard and mouse reports.
 *
 * Header location rationale (moved from include/linux/ to driver-private):
 *   1. APIs only used within imx9_ikvm — not a public API
 *   2. Singleton g_hid global makes it unsuitable for include/linux/
 *
 * BAR3 ABI contract (shared with host-side imx-pcie-rc-hid.c):
 *   - STABLE ABI — EP and RC must agree on field offsets
 *   - Host driver redefines structures locally (different kernel, RC side)
 *   - static_assert() guards catch EP-side layout changes at compile time
 *   - ABI sizes: hid_desc=80B, hid_ring_header=16B, 256 entries
 *   - BAR3: [hid_ring_header @ 0x0000][hid_desc[256] @ 0x0010]
 *
 * Singleton constraint:
 *   - Only one EP instance at a time
 *   - epf_hid_attach() enforces with WARN_ON + -EBUSY
 *   - Future: pass struct epf_hid * through call chain
 *
 * Initialization model:
 *
 * The pcie-input module registers input handlers and cdevs at module_init
 * time. The HID ring is NOT initialized at module_init. Instead, the VGA
 * EPF driver calls epf_hid_attach() once BAR3 is mapped in epc_init, and
 * epf_hid_detach() when the BAR is torn down in epc_deinit / unbind.
 */

#ifndef __PCI_EPF_HID_H__
#define __PCI_EPF_HID_H__

#include <linux/io.h>
#include <linux/types.h>
#include <linux/spinlock.h>

/*
 * BAR3 size reserved for the PCI Endpoint HID function.
 *
 * The actual PCI BAR must be configured to be at least this size.
 */
#define HID_BAR3_SIZE		32768U

/*
 * HID device types supported by the PCI Endpoint HID function.
 */
#define HID_TYPE_KEYBOARD  1
#define HID_TYPE_REL_MOUSE 2
#define HID_TYPE_ABS_MOUSE 3

/*
 * Number of HID descriptors that can be stored in the ring buffer.
 */
#define HID_RING_SIZE 256U

/*
 * HID event descriptor stored in the shared BAR3 ring.
 *
 * This structure is part of the BAR3 shared-memory ABI.
 *
 * @type:  HID device/report type.
 * @len:   Length of valid data in @data.
 * @seq:   Sequence number of the report.
 * @flags: Report/event flags.
 * @data:  HID report payload.
 *
 * Size: 80 bytes.
 */
struct hid_desc {
	u32 type;
	u32 len;
	u32 seq;
	u32 flags;
	u8 data[64];
} __packed;

/*
 * Shared HID ring header stored at BAR3 + 0x0000.
 *
 * BAR3 layout:
 *
 *   BAR3 + 0x0000 : struct hid_ring_header
 *   BAR3 + 0x0010 : struct hid_desc[HID_RING_SIZE]
 *
 * Ownership:
 *
 *   prod - endpoint producer index
 *   cons - host consumer index
 *   seq  - endpoint sequence number
 *
 * The host must update cons after it has consumed descriptors.
 *
 * Size: 16 bytes.
 */
struct hid_ring_header {
	u32 prod;
	u32 cons;
	u32 seq;
	u32 reserved;
} __packed;

/*
 * Total size occupied by the HID shared-memory region.
 */
#define HID_RING_BYTES		(sizeof(struct hid_desc) * HID_RING_SIZE)
#define HID_SHARED_SIZE		(sizeof(struct hid_ring_header) + \
				 HID_RING_BYTES)

/*
 * Private state for the PCI Endpoint HID function.
 *
 * The producer, consumer and sequence values are NOT stored here.
 * They are stored in the shared BAR3 header so that the PCIe host
 * can observe/update them.
 *
 * @lock: Protects endpoint-side producer updates.
 * @bar:  BAR3 HID shared-memory base.
 * @attached: 1=attached, 0=detached; singleton guard
 * @hdr:  Shared BAR3 ring header.
 * @ring: Shared BAR3 descriptor ring.
 */
struct epf_hid {
	spinlock_t lock;
	void __iomem *bar;
    atomic_t attached;
	struct hid_ring_header __iomem *hdr;
	struct hid_desc __iomem *ring;
};

/*
 * Relative mouse HID report.
 *
 * @buttons: Current mouse button state.
 * @x:       Relative movement along the X axis.
 * @y:       Relative movement along the Y axis.
 * @wheel:   Relative wheel movement.
 */
struct hid_rel_mouse_report {
	u8 buttons;
	s8 x;
	s8 y;
	s8 wheel;
} __packed;

/*
 * Absolute mouse HID report.
 *
 * @buttons: Current mouse button state.
 * @x:       Absolute X coordinate.
 * @y:       Absolute Y coordinate.
 * @wheel:   Relative wheel movement.
 */
struct hid_abs_mouse_report {
	u8 buttons;
	u16 x;
	u16 y;
	s8 wheel;
} __packed;

/*
 * USB/HID boot-protocol keyboard report.
 *
 * @modifier: Modifier key bitmap (Ctrl, Shift, Alt, etc.).
 * @reserved: Reserved byte required by the standard keyboard report.
 * @keycode:  Up to six simultaneously pressed key codes.
 */
struct hid_keyboard_report {
	u8 modifier;
	u8 reserved;
	u8 keycode[6];
} __packed;

/*
 * Relative mouse interface initialization and cleanup.
 */
int rel_mouse_init(void);
void rel_mouse_exit(void);

/*
 * Send a relative mouse report through the PCI Endpoint HID function.
 *
 * @rpt: Pointer to the relative mouse report.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int epf_hid_send_rel_mouse(struct hid_rel_mouse_report *rpt);

/*
 * Absolute mouse interface initialization and cleanup.
 */
int abs_mouse_init(void);
void abs_mouse_exit(void);

/*
 * Send an absolute mouse report through the PCI Endpoint HID function.
 *
 * @rpt: Pointer to the absolute mouse report.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int epf_hid_send_abs_mouse(struct hid_abs_mouse_report *rpt);

/*
 * Remote/PCI Endpoint keyboard interface initialization and cleanup.
 */
int keyboard_init(void);
void keyboard_exit(void);

/*
 * Local keyboard interface initialization and cleanup.
 */
int local_keyboard_init(void);
void local_keyboard_exit(void);

/*
 * Send a keyboard HID report through the PCI Endpoint HID function.
 *
 * @rpt: Pointer to the keyboard report.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int epf_hid_send_keyboard(struct hid_keyboard_report *rpt);

/*
 * epf_hid_attach() - attach the HID ring to a mapped BAR3 region
 * @bar: virtual __iomem address of the BAR3 shared-memory region
 *
 * Called by the VGA EPF driver from epc_init() after BAR3 is mapped.
 * Initializes the shared ring header and registers input handlers.
 *
 * Return: 0 on success, negative error code on failure.
 */
int epf_hid_attach(void __iomem *bar);

/*
 * epf_hid_detach() - detach the HID ring and stop input handlers
 *
 * Called by the VGA EPF driver from epc_deinit() / unbind() before
 * BAR3 is unmapped. Clears all ring pointers so in-flight report
 * submissions return -ENODEV instead of accessing freed memory.
 */
void epf_hid_detach(void);

/*
 * ABI size assertions — catch accidental struct layout changes.
 */
static_assert(sizeof(struct hid_desc) == 80,
	      "hid_desc ABI size changed — update host driver");
static_assert(sizeof(struct hid_ring_header) == 16,
	      "hid_ring_header ABI size changed — update host driver");

#endif /* __PCI_EPF_HID_H__ */

