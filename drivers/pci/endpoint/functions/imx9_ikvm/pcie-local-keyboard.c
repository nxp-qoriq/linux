// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe Endpoint Local Keyboard Driver
 *
 * Captures keyboard HID reports from local HID devices and forwards them
 * over the PCIe endpoint using the EPF HID helper.
 *
 * Copyright 2026 NXP
 */

#include <linux/hid.h>
#include <linux/slab.h>
#include <linux/pci-epf-hid.h>

struct local_keyboard {
	bool keyboard;
};

static bool local_keyboard_match(struct hid_device *hdev)
{
	struct hid_input *hidinput;

	list_for_each_entry(hidinput, &hdev->inputs, list) {
		if (hidinput->application == HID_GD_KEYBOARD)
			return true;
	}

	return false;
}

static int local_keyboard_raw_event(struct hid_device *hdev,
				    struct hid_report *report,
				    u8 *data, int size)
{
	struct local_keyboard *kbd = hid_get_drvdata(hdev);
	struct hid_keyboard_report rpt;
	int ret;

	if (!kbd || !kbd->keyboard)
		return 0;

	if (size != sizeof(rpt))
		return 0;

	rpt.modifier = data[0];
	rpt.reserved = data[1];
	memcpy(rpt.keycode, &data[2], sizeof(rpt.keycode));

	ret = epf_hid_send_keyboard(&rpt);
	if (ret)
		pr_err("failed to send keyboard report: %d\n", ret);

	return 0;
}

static void local_keyboard_remove(struct hid_device *hdev)
{
	pr_info("remove %s\n", dev_name(&hdev->dev));

	hid_hw_stop(hdev);
}

static int local_keyboard_probe(struct hid_device *hdev,
				const struct hid_device_id *id)
{
	struct local_keyboard *kbd;
	int ret;

	pr_info("probe %s (vid=%04x pid=%04x)\n",
		dev_name(&hdev->dev),
		hdev->vendor,
		hdev->product);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	kbd = devm_kzalloc(&hdev->dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd)
		return -ENOMEM;

	hid_set_drvdata(hdev, kbd);

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT | HID_CONNECT_HIDRAW);
	if (ret) {
		pr_err("hid_hw_start failed: %d\n", ret);
		return ret;
	}

	kbd->keyboard = local_keyboard_match(hdev);

	if (!kbd->keyboard) {
		pr_info("ignoring non-keyboard HID device\n");
		return 0;
	}

	pr_info("keyboard detected\n");

	return 0;
}

static const struct hid_device_id local_keyboard_ids[] = {
	{
		HID_DEVICE(HID_BUS_ANY,
			   HID_GROUP_ANY,
			   HID_ANY_ID,
			   HID_ANY_ID)
	},
	{ }
};

MODULE_DEVICE_TABLE(hid, local_keyboard_ids);

static struct hid_driver local_keyboard_driver = {
	.name		= "epf_local_keyboard",
	.id_table	= local_keyboard_ids,
	.probe		= local_keyboard_probe,
	.remove		= local_keyboard_remove,
	.raw_event	= local_keyboard_raw_event,
};

int local_keyboard_init(void)
{
	pr_info("registering local keyboard HID driver\n");

	return hid_register_driver(&local_keyboard_driver);
}

void local_keyboard_exit(void)
{
	hid_unregister_driver(&local_keyboard_driver);

	pr_info("local keyboard HID driver unloaded\n");
}