#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <linux/i2c.h>
#include <linux/printk.h>

#include <linux/crc32.h>
#include <linux/rfnm-shared.h>


typedef unsigned char       uint8_t;
typedef   signed char        int8_t;

/*
	EEPROM memory layout

	0x00: 4 bytes magic header 0x34 0x26 0x94 0x12
	0x10: 1 byte board id
	0x11: 1 byte board revision id
	0x12-0x16: free space
	0x17: 9 byte board serial number (8 + null)
	0x20: 4 byte crc32

*/



void rfnm_bootconfig_read_eeprom(struct i2c_client *client, uint8_t addr, uint8_t *buf)
{

	i2c_master_send(client, &addr, 1);
	i2c_master_recv(client, buf, 1);
}

int rfnm_bootconfig_write_eeprom(struct i2c_client *client, uint8_t addr, uint8_t val)
{

	uint8_t write[2];

	write[0] = addr;
	write[1] = val;

	int retry_id = 0;
	int ret;

retry:
	ret = i2c_master_send(client, write, 2);

	if (ret < 0) {
		if (retry_id++ < 30) {
			goto retry;
		} else {
			return -1;
		}
	}

	return 0;
}

int rfnm_load_board_info(struct device *dev, struct rfnm_eeprom_data *eeprom_data)
{

	int i, ret;
	uint8_t i2cbuf;
	uint8_t *eeprom_data_ptr;

	eeprom_data_ptr = (uint8_t *) eeprom_data;
	struct i2c_client *client = to_i2c_client(dev);

	memset(eeprom_data, 0, sizeof(*eeprom_data));

	for (i = 0; i < sizeof(*eeprom_data); i++) {
		rfnm_bootconfig_read_eeprom(client, i, &i2cbuf);
		//printk("RFNM: %02x\n", i2cbuf);
		*(eeprom_data_ptr + i) = i2cbuf;
	}

	if (eeprom_data->crc != crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4)) {
		//printk("RFNM: crc mismatch, eeprom read failed %x %x\n", eeprom_data->crc, crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4));
		return -1;
	}

	return 0;
}


static ssize_t rfnm_show_board_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct i2c_client *client = to_i2c_client(dev);
	uint8_t i2cbuf;
	int z;
	struct rfnm_eeprom_data eeprom_data;

	if (rfnm_load_board_info(dev, &eeprom_data)) {
		return snprintf(buf, PAGE_SIZE, "Failed to read eeprom\n");
	} else {
		return snprintf(buf, PAGE_SIZE, "board id %d revision %d serial %s\n", eeprom_data.board_id, eeprom_data.board_revision_id, eeprom_data.serial_number);
	}
}

static DEVICE_ATTR_RO(rfnm_show_board_info);

static ssize_t rfnm_factory_use_only_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = to_i2c_client(dev);

	if (count != 15) {
		//printk("RFNM: Invalid length at %d\n", count);
		return -EINVAL;
	}

	if (buf[2] != '-' || buf[5] != '-') {
		//printk("RFNM: invalid chars in string %c %c\n", &buf[2], &buf[5]);
		return -EINVAL;
	}

	const char __user tmpstr[10];
	uint32_t tmpval;
	struct rfnm_eeprom_data eeprom_data;
	int i, ret;

	memset((uint8_t *) &eeprom_data, 0, sizeof(eeprom_data));

	eeprom_data.magic_header[0] = 0x34;
	eeprom_data.magic_header[1] = 0x26;
	eeprom_data.magic_header[2] = 0x94;
	eeprom_data.magic_header[3] = 0x12;

	memset(&tmpstr, 0, 10);
	memcpy(&tmpstr, &buf[0], 2);

    if (kstrtou32(&tmpstr[0], 10, &tmpval) != 0) {
		//printk("RFNM: invalid digit at pos 1");
		return -EINVAL;
	}

	eeprom_data.board_id = tmpval;

	memset(&tmpstr, 0, 10);
	memcpy(&tmpstr, &buf[3], 2);

    if (kstrtou32(&tmpstr[0], 10, &tmpval) != 0) {
		//printk("RFNM: invalid digit at pos 2");
		return -EINVAL;
	}

	eeprom_data.board_revision_id = tmpval;

	memcpy(&eeprom_data.serial_number, &buf[6], 8);
	eeprom_data.serial_number[8] = 0;

	//printk("RFNM: board id %d revision %d serial %s\n", eeprom_data.board_id, eeprom_data.board_revision_id, eeprom_data.serial_number);

	uint8_t *eeprom_data_ptr;

	eeprom_data_ptr = (uint8_t *) &eeprom_data;

	eeprom_data.crc = crc32(0x80000000, eeprom_data_ptr, sizeof(eeprom_data) - 4);

	for (i = 0; i < sizeof(eeprom_data); i++) {
		//printk("RFNM: %d %02x\n", i, eeprom_data_ptr[i]);
		ret = rfnm_bootconfig_write_eeprom(client, i, *(eeprom_data_ptr + i));
		if (ret < 0) {
			printk("RFNM: Write to daughterboard failed");
			return -ENODEV;
		}
	}

	return count;
}

static DEVICE_ATTR_WO(rfnm_factory_use_only);


static int rfnm_bootconfig_probe(struct i2c_client *client)
{

	struct i2c_adapter *adapter = client->adapter;

	struct rfnm_bootconfig *cfg;
	struct rfnm_eeprom_data *eeprom_data;

	cfg = memremap(RFNM_BOOTCONFIG_PHYADDR, SZ_4M, MEMREMAP_WB);

	if (adapter->nr == 0) {
		eeprom_data = &cfg->motherboard_eeprom;
		if (!rfnm_load_board_info(&client->dev, eeprom_data)) {
			printk("RFNM: Motherboard id %d revision %d serial %s\n", eeprom_data->board_id, eeprom_data->board_revision_id, eeprom_data->serial_number);
		}
	} else {
		eeprom_data = &cfg->daughterboard_eeprom[adapter->nr - 1];
		if (!rfnm_load_board_info(&client->dev, eeprom_data)) {
			printk("RFNM: Daughterboard detected on slot %d, board id %d revision %d serial %s\n", adapter->nr, eeprom_data->board_id, eeprom_data->board_revision_id, eeprom_data->serial_number);
			cfg->daughterboard_present[adapter->nr - 1] = RFNM_DAUGHTERBOARD_PRESENT;
		} else {
			cfg->daughterboard_present[adapter->nr - 1] = RFNM_DAUGHTERBOARD_NOT_FOUND;
		}
	}



	int err;

	err = device_create_file(&client->dev, &dev_attr_rfnm_show_board_info);
	if (err < 0) {
		printk("RFNM: failed to create device file for rfnm_show_board_info");
	}

	err = device_create_file(&client->dev, &dev_attr_rfnm_factory_use_only);
	if (err < 0) {
		printk("RFNM: failed to create device file for rfnm_factory_use_only");
	}

	return 0;
}

static int rfnm_bootconfig_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &(dev_attr_rfnm_show_board_info));
	device_remove_file(&client->dev, &(dev_attr_rfnm_factory_use_only));
	return 0;
}







static const struct of_device_id rfnm_bootconfig_match_table[] = {
	{ .compatible = "rfnm,bootconfig", },
	{}
};
MODULE_DEVICE_TABLE(of, rfnm_bootconfig_match_table);

static const struct i2c_device_id rfnm_bootconfig_id_table[] = {
	{ "rfnm_bootconfig", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rfnm_bootconfig_id_table);

static struct i2c_driver rfnm_bootconfig_driver = {
	.driver	= {
		.name	= "rfnm_bootconfig",
		.of_match_table = rfnm_bootconfig_match_table,
	},
	.remove     = rfnm_bootconfig_remove,
	.probe_new	= rfnm_bootconfig_probe,
	.id_table	= rfnm_bootconfig_id_table,
};
module_i2c_driver(rfnm_bootconfig_driver);
MODULE_LICENSE("GPL");
