// SPDX-License-Identifier: GPL-2.0+

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include <linux/i2c.h>
#include <linux/printk.h>

#include <linux/crc32.h>
#include <linux/rfnm-shared.h>

#define MAX_NODE_NAME_LEN 10
typedef unsigned char       uint8_t;
typedef   signed char        int8_t;

int la9310_read_dtb_node_mem_region(const char *node_name, struct resource *get_mem_res)
{
        int rc = 0;
        struct device_node *memnp;
        struct resource mem_res;

        /* Get pointer to device node */
        memnp = of_find_node_by_name(NULL,node_name);;
        if (!memnp) {
                printk("Node %s not found\n",node_name);
                rc = RFNM_DTB_NODE_NOT_FOUND;
        }
        else {
                /* Convert memory region to a struct resource */
                rc = of_address_to_resource(memnp, 0, &mem_res);
                /* finished with memnp */
                of_node_put(memnp);
                if (rc) {
                        printk("Failed to translate memory-region to a resource for node %s\n",node_name);
                        rc = RFNM_DTB_NODE_NOT_FOUND;
                }
                else {
                        pr_info("RFNM: func %s Node Name %s\n",__func__,node_name);
                        pr_info("MemRegion Start 0x%08X\n", mem_res.start);
                        pr_info("MemRegion Size 0x%08X\n", resource_size(&mem_res));
                        get_mem_res->start= mem_res.start;
                        memcpy(get_mem_res,&mem_res, sizeof(struct resource));
                }
        }
        return rc;
}

EXPORT_SYMBOL(la9310_read_dtb_node_mem_region);

/*
	EEPROM memory layout

	0x00: 4 bytes magic header 0x34 0x26 0x94 0x12
	0x10: 1 byte board id
	0x11: 1 byte board revision id
	0x12-0x16: free space
	0x17: 9 byte board serial number (8 + null)
	0x20: 4 byte crc32

*/


// 256 bytes divided into 32 pages, factory config starts at 0, user config starts at 128


void rfnm_bootconfig_read_eeprom(struct i2c_client *client, uint8_t addr, uint8_t * buf) {

	i2c_master_send(client, &addr, 1);
	i2c_master_recv(client, buf, 1);
}

int rfnm_bootconfig_write_eeprom(struct i2c_client *client, uint8_t addr, uint8_t val) {

	uint8_t write[2];
	write[0] = addr;
	write[1] = val;

	int retry_id = 0;
	int ret;

retry:
	ret = i2c_master_send(client, write, 2);

	if(ret < 0) {
		if(retry_id++ < 30) {
			goto retry;
		} else {
			return -1;
		}
	}

	return 0;
}

int rfnm_load_board_info(struct device *dev, struct rfnm_eeprom_data *eeprom_data) {
	
	int i, ret;
	uint8_t i2cbuf;
	uint8_t *eeprom_data_ptr;
	eeprom_data_ptr = (uint8_t*) eeprom_data;
	struct i2c_client *client = to_i2c_client(dev);

	memset(eeprom_data, 0, sizeof(*eeprom_data));
	
	for(i = 0; i < sizeof(*eeprom_data); i++) {
		rfnm_bootconfig_read_eeprom(client, i, &i2cbuf);
		//printk("RFNM: %02x\n", i2cbuf);
		*(eeprom_data_ptr + i) = i2cbuf;
	}

	if(eeprom_data->crc != crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4)) {
		//printk("RFNM: crc mismatch, eeprom read failed %x %x\n", eeprom_data->crc, crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4));
		return -1;
	}

	return 0;
}


int rfnm_load_user_config(struct device *dev, struct rfnm_eeprom_user_config *eeprom_data) {
	
	int i, ret;
	uint8_t i2cbuf;
	uint8_t *eeprom_data_ptr;
	eeprom_data_ptr = (uint8_t*) eeprom_data;
	struct i2c_client *client = to_i2c_client(dev);

	memset(eeprom_data, 0, sizeof(*eeprom_data));
	
	for(i = 0; i < sizeof(*eeprom_data); i++) {
		rfnm_bootconfig_read_eeprom(client, 128+i, &i2cbuf);
		//printk("RFNM: %02x\n", i2cbuf);
		*(eeprom_data_ptr + i) = i2cbuf;
	}

	if(eeprom_data->crc != crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4)) {
		//printk("RFNM: crc mismatch, eeprom read failed %x %x\n", eeprom_data->crc, crc32(0x80000000, eeprom_data_ptr, sizeof(*eeprom_data) - 4));
		return -1;
	}

	return 0;
}

static ssize_t rfnm_display_user_config_show(struct device *dev, struct device_attribute *attr, char *buf) {

	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adapter = client->adapter;
	int adapter_nr = i2c_adapter_id(adapter);
	uint8_t i2cbuf;
	int z;
	struct rfnm_eeprom_user_config eeprom_user;

	if(adapter_nr) {
		return snprintf(buf, PAGE_SIZE, "Not supported for those\n");
	} 

	if(rfnm_load_user_config(dev, &eeprom_user)) {
		return snprintf(buf, PAGE_SIZE, "Failed to read eeprom\n");
	} else {
		return snprintf(buf, PAGE_SIZE, "DCS clock is %d\n", eeprom_user.dcs_clk_tmp );
	}
}

static DEVICE_ATTR_RO(rfnm_display_user_config);


static ssize_t rfnm_show_board_info_show(struct device *dev, struct device_attribute *attr, char *buf) {

	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adapter = client->adapter;
	int adapter_nr = i2c_adapter_id(adapter);
	uint8_t i2cbuf;
	int z;
	struct rfnm_eeprom_data eeprom_data;

	if(rfnm_load_board_info(dev, &eeprom_data)) {
		return snprintf(buf, PAGE_SIZE, "Failed to read eeprom\n");
	} else {
		if(adapter_nr) {
			return snprintf(buf, PAGE_SIZE, "board id %d revision %d serial %s\n", eeprom_data.board_id, eeprom_data.board_revision_id, eeprom_data.serial_number);
		} else {
			return snprintf(buf, PAGE_SIZE, "board id %d revision %d serial %s mac-addr %02x:%02x:%02x:%02x:%02x:%02x\n", 
						eeprom_data.board_id, eeprom_data.board_revision_id, eeprom_data.serial_number,
						eeprom_data.mac_addr[0], eeprom_data.mac_addr[1], eeprom_data.mac_addr[2], eeprom_data.mac_addr[3], eeprom_data.mac_addr[4], eeprom_data.mac_addr[5]);
		}

		
	}
}

static DEVICE_ATTR_RO(rfnm_show_board_info);

static ssize_t rfnm_factory_use_only_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {

	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adapter = client->adapter;
	int adapter_nr = i2c_adapter_id(adapter);

	//01-02-abcdefgh
	//01-02-abcdefgh-00:00:00:00:00:00

	if(adapter_nr) {
		if(count != 15) {
			//printk("RFNM: Invalid length at %d\n", count);
			return -EINVAL;
		}
	} else {
		if(count != (15 + 18)) {
			return -EINVAL;
		}

		if(buf[14] != '-' || buf[17] != ':' || buf[20] != ':' || buf[23] != ':' || buf[26] != ':' || buf[29] != ':') {
			return -EINVAL;
		}
	}
	
	if(buf[2] != '-' || buf[5] != '-') {
		//printk("RFNM: invalid chars in string %c %c\n", &buf[2], &buf[5]);
		return -EINVAL;
	}

	const char __user tmpstr[20];
	uint32_t tmpval;
	struct rfnm_eeprom_data eeprom_data;
	int i, ret;

	memset((uint8_t*) &eeprom_data, 0, sizeof(eeprom_data));

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

	if(!adapter_nr) {
		memset(&tmpstr, 0, 20);
		memcpy(&tmpstr, &buf[15], 17);
		sscanf(&tmpstr[0], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
				&eeprom_data.mac_addr[0], &eeprom_data.mac_addr[1], &eeprom_data.mac_addr[2], 
				&eeprom_data.mac_addr[3], &eeprom_data.mac_addr[4], &eeprom_data.mac_addr[5]);
	}

	//printk("RFNM: board id %d revision %d serial %s\n", eeprom_data.board_id, eeprom_data.board_revision_id, eeprom_data.serial_number);

	uint8_t *eeprom_data_ptr;
	eeprom_data_ptr = (uint8_t*) &eeprom_data;

	eeprom_data.crc = crc32(0x80000000, eeprom_data_ptr, sizeof(eeprom_data) - 4);
	
	for(i = 0; i < sizeof(eeprom_data); i++) {
		//printk("RFNM: %d %02x\n", i, eeprom_data_ptr[i]);
		ret = rfnm_bootconfig_write_eeprom(client, i, *(eeprom_data_ptr + i));
		if(ret < 0) {
			printk("RFNM: Write to daughterboard failed");
			return -ENODEV;
		}
	}

	return count;
}

static DEVICE_ATTR_WO(rfnm_factory_use_only);



static ssize_t rfnm_set_dcs_freq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {

	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_adapter *adapter = client->adapter;
	int adapter_nr = i2c_adapter_id(adapter);

	//01-02-abcdefgh
	//01-02-abcdefgh-00:00:00:00:00:00

	if(adapter_nr) {
		return -EINVAL;
	}
	uint32_t tmpval;
	struct rfnm_eeprom_user_config eeprom_data;
	int i, ret;

	memset((uint8_t*) &eeprom_data, 0, sizeof(eeprom_data));

	eeprom_data.magic_header[0] = 0xA1;
	eeprom_data.magic_header[1] = 0x45;
	eeprom_data.magic_header[2] = 0x8E;
	eeprom_data.magic_header[3] = 0xF8;


    if (kstrtou32(&buf[0], 10, &tmpval) != 0) {
		//printk("RFNM: invalid digit at pos 1");
		return -EINVAL;
	}

	eeprom_data.dcs_clk_tmp = tmpval;

	uint8_t *eeprom_data_ptr;
	eeprom_data_ptr = (uint8_t*) &eeprom_data;

	eeprom_data.crc = crc32(0x80000000, eeprom_data_ptr, sizeof(eeprom_data) - 4);
	
	for(i = 0; i < sizeof(eeprom_data); i++) {
		//printk("RFNM: %d %02x\n", i, eeprom_data_ptr[i]);
		ret = rfnm_bootconfig_write_eeprom(client, 128+i, *(eeprom_data_ptr + i));
		if(ret < 0) {
			printk("RFNM: Write to daughterboard failed");
			return -ENODEV;
		}
	}

	return count;
}

static DEVICE_ATTR_WO(rfnm_set_dcs_freq);




static int rfnm_bootconfig_probe(struct i2c_client *client) {

	struct i2c_adapter *adapter = client->adapter;
	
	struct rfnm_bootconfig *cfg;
	struct rfnm_eeprom_data *eeprom_data;
	
	struct rfnm_eeprom_user_config *eeprom_user;
	struct resource mem_res;
	char node_name[MAX_NODE_NAME_LEN + 1];
	int ret;

	strncpy(node_name,"bootconfig",MAX_NODE_NAME_LEN );
	node_name[MAX_NODE_NAME_LEN] = '\0';
	ret = la9310_read_dtb_node_mem_region(node_name,&mem_res);
	if(ret != RFNM_DTB_NODE_NOT_FOUND) {
		cfg = memremap(mem_res.start, SZ_4M, MEMREMAP_WB);
	}
	else {
		printk("RFNM: func %s Node name %s not found..\n",__func__,node_name);
		return ret;
	}

	if(adapter->nr == 0) {
		eeprom_data = &cfg->motherboard_eeprom;
		if(!rfnm_load_board_info(&client->dev, eeprom_data)) {
			printk("RFNM: Motherboard id %d revision %d serial %s mac-addr %02x:%02x:%02x:%02x:%02x:%02x\n", 
			eeprom_data->board_id, eeprom_data->board_revision_id, eeprom_data->serial_number,
			eeprom_data->mac_addr[0], eeprom_data->mac_addr[1], eeprom_data->mac_addr[2], eeprom_data->mac_addr[3], eeprom_data->mac_addr[4], eeprom_data->mac_addr[5]);
		}

		eeprom_user = &cfg->user_eeprom;		
		if(!rfnm_load_user_config(&client->dev, eeprom_user)) {
			printk("RFNM: DCS clock is %d\n", eeprom_user->dcs_clk_tmp);
		}
	} else {
		eeprom_data = &cfg->daughterboard_eeprom[adapter->nr - 1];
		if(!rfnm_load_board_info(&client->dev, eeprom_data)) {
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

	if(adapter->nr == 0) {
		err = device_create_file(&client->dev, &dev_attr_rfnm_display_user_config);
		if (err < 0) {
			printk("RFNM: failed to create device file for rfnm_display_user_config");
		}

		err = device_create_file(&client->dev, &dev_attr_rfnm_set_dcs_freq);
		if (err < 0) {
			printk("RFNM: failed to create device file for rfnm_set_dcs_freq");
		}
	}

	

	return 0;
}

static int rfnm_bootconfig_remove(struct i2c_client *client) {
	struct i2c_adapter *adapter = client->adapter;

	device_remove_file(&client->dev, &(dev_attr_rfnm_show_board_info));
	device_remove_file(&client->dev, &(dev_attr_rfnm_factory_use_only));

	if(adapter->nr == 0) {
		device_remove_file(&client->dev, &(dev_attr_rfnm_display_user_config));
		device_remove_file(&client->dev, &(dev_attr_rfnm_set_dcs_freq));
	}
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
