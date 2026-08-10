// SPDX-License-Identifier: GPL-2.0
// Copyright 2026 NXP

/*
 * PCI Endpoint HID Function with pci-epf-imx9-vga function
 *
 * This header defines the common data structures and APIs used by the
 * PCI Endpoint HID function for handling keyboard and mouse reports.
 */

#ifndef __PCI_EPF_HID_H__
#define __PCI_EPF_HID_H__

#include <linux/types.h>
#include <linux/spinlock.h>

/*
 * HID device types supported by the PCI Endpoint HID function.
 */
#define HID_TYPE_KEYBOARD  1
#define HID_TYPE_REL_MOUSE 2
#define HID_TYPE_ABS_MOUSE 3

/*
 * Number of HID descriptors that can be stored in the ring buffer.
 */
#define HID_RING_SIZE 256

/*
 * HID event descriptor stored in the shared ring buffer.
 *
 * @type:  HID device/report type.
 * @len:   Length of valid data in @data.
 * @seq:   Sequence number of the report.
 * @flags: Report/event flags.
 * @data:  HID report payload.
 */
struct hid_desc {
	u32 type;
	u32 len;
	u32 seq;
	u32 flags;
	u8 data[64];
} __packed;

/*
 * Private state for the PCI Endpoint HID function.
 *
 * @lock: Protects access to the HID ring and producer/consumer state.
 * @bar:  Pointer to the PCI BAR/shared memory region used for HID data.
 * @prod: Producer index of the HID ring buffer.
 * @cons: Consumer index of the HID ring buffer.
 * @seq:  Sequence number assigned to outgoing reports.
 * @ring: Pointer to the HID descriptor ring buffer.
 */
struct epf_hid {
	spinlock_t lock;

	void *bar;

	u32 prod;
	u32 cons;
	u32 seq;

	struct hid_desc *ring;
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

#endif /* __PCI_EPF_HID_H__ */