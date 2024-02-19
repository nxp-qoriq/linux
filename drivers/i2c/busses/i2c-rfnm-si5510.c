// SPDX-License-Identifier: GPL-2.0+

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include <linux/rfnm-shared.h>
#include <linux/rfnm-si5510.h>
#include <linux/printk.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/ktime.h>


typedef unsigned char       uint8_t;
typedef   signed char        int8_t;

void rfnm_si5510_i2c_read(struct i2c_client *client, uint8_t *buf, int cnt)
{

	uint8_t CTS[6] = {0xf0, 0x0f};

	i2c_master_send(client, CTS, 2);

	i2c_master_recv(client, buf, cnt);
}

void rfnm_si5510_i2c_write(struct i2c_client *client, uint8_t *buf, int cnt)
{

	i2c_master_send(client, buf, cnt);
}

void rfnm_si5510_sio_test(struct i2c_client *client)
{
 uint8_t sio_test_request[5] = { 0xF0, 0x0F, 0x01, 0xAB, 0xCD };

 rfnm_si5510_i2c_write(client, sio_test_request, 5);

 uint8_t i2c_read_buf[100];

 rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 4);

 while (i2c_read_buf[0] != 0x80) {
	 msleep(10);
	 rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 4);
 }

}

void rfnm_si5510_cts(struct i2c_client *client)
{

	uint8_t i2c_read_buf[100];

	do {
		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 1);
		msleep(10);
		//printk("RFNM: %02x\n", i2c_read_buf[0]);
	} while (i2c_read_buf[0] != 0x80);
}

int rfnm_si5510_buffsize(struct i2c_client *client)
{

	uint8_t i2c_read_buf[100];
	uint8_t sio_info_request[3] = { 0xF0, 0x0F, 0x02 };

	rfnm_si5510_i2c_write(client, sio_info_request, 3);

	do {
		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 5);
	} while (i2c_read_buf[0] != 0x80);

	int CMD_BUFFER_SIZE = (i2c_read_buf[2] << 8) + i2c_read_buf[1];

	//printk("RFNM: Command buffer size from SIO_INFO: %d\n", CMD_BUFFER_SIZE);
	return CMD_BUFFER_SIZE;
}


void rfnm_si5510_restart(struct i2c_client *client)
{

	uint8_t i2c_read_buf[100];
	uint8_t restart_request[] = { 0xF0, 0x0F, 0xF0, 0x00 };

	rfnm_si5510_i2c_write(client, restart_request, 4);

	do {

		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 1);

		//printk("RFNM: RESTART returned %02x\n", i2c_read_buf[0]);

		msleep(10);
	} while (i2c_read_buf[0] != 0x80);

	//printk("RFNM: RESTART Sent.\n");
}

void rfnm_si5510_host_load(struct i2c_client *client, char *data, int datalen, int CMD_BUFFER_SIZE)
{

	CMD_BUFFER_SIZE -= 10; // margin for command packets
	uint8_t i2c_read_buf[100];
	int numberOfChunks = (datalen / CMD_BUFFER_SIZE) + 1;

	int chunkNum;

	uint8_t host_load_command_init[3] = { 0xF0, 0x0F, 0x05 };
	uint8_t *host_load_command;

	host_load_command = kmalloc(CMD_BUFFER_SIZE + 10, GFP_KERNEL);
	memcpy(host_load_command, &host_load_command_init[0], 3);

	for (chunkNum = 0; chunkNum < numberOfChunks; ++chunkNum) {
		int chunkSize = CMD_BUFFER_SIZE;

		if (chunkNum == numberOfChunks - 1) {
			chunkSize = datalen % CMD_BUFFER_SIZE;
		}

		memcpy(&host_load_command[3], &data[chunkNum * CMD_BUFFER_SIZE], CMD_BUFFER_SIZE);
		rfnm_si5510_i2c_write(client, &host_load_command[0], chunkSize + 3);

		do {
			rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 1);
			//printk("RFNM: CHUNK %d status is %02x\n", chunkNum, i2c_read_buf[0]);
		} while (i2c_read_buf[0] != 0x80);
	}

	kfree(host_load_command);
}

int
la9310_read_dtb_node_mem_region(const char *node_name, struct resource *get_mem_res)
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

extern void rfnm_wsled_send_chain(uint8_t);
extern void rfnm_wsled_set(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

static void rfnm_si5510_boot(struct i2c_client *client)
{

	uint8_t i2c_read_buf[100];
	uint8_t boot_request[] = { 0xF0, 0x0F, 0x07 };

	rfnm_si5510_i2c_write(client, boot_request, 3);

	do {
		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 1);
		//printk("RFNM: BOOT returned %02x\n", i2c_read_buf[0]);
		msleep(10);
	} while (i2c_read_buf[0] != 0x80);

	//printk("RFNM: BOOT Sent.\n");
}


uint8_t rfnm_si5510_reference_status(struct i2c_client *client)
{
	uint8_t i2c_read_buf[100];

	uint8_t reference_status_request[] = { 0xF0, 0x0F, 0x16 };

	rfnm_si5510_i2c_write(client, reference_status_request, 3);

	uint8_t reference_status_response[5];

	do {
		if (i2c_read_buf[0] == 0x90) {
			printk("RFNM: FWERR triggered. See text under Common Errors.\n");
			while (1) {}
		}
		msleep(10);
		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 5);
	} while (i2c_read_buf[0] != 0x80);

	// return true if reference PLL is locked, otherwise return false.
	// why & and not &&???
	return (i2c_read_buf[0] == 0x80 & i2c_read_buf[1] == 0x00 & i2c_read_buf[2] == 0 & i2c_read_buf[3] == 0 & i2c_read_buf[4] == 0);
}

int can_use_si5510_config(struct rfnm_bootconfig *cfg, int daughterboard_1, int daughterboard_2)
{
	if (daughterboard_1 == daughterboard_2) {
		// relax conditions: only need to match one daughterboard
		// (to account for missing or unsupported daughterboards)
		if (
			(cfg->daughterboard_eeprom[0].board_id == daughterboard_1 && cfg->daughterboard_eeprom[1].board_id == daughterboard_1) ||

			(cfg->daughterboard_eeprom[0].board_id == daughterboard_1 && (
				cfg->daughterboard_present[1] == RFNM_DAUGHTERBOARD_NOT_FOUND || cfg->daughterboard_eeprom[1].board_id == RFNM_DAUGHTERBOARD_BREAKOUT
			)) ||
			(cfg->daughterboard_eeprom[1].board_id == daughterboard_1 && (
				cfg->daughterboard_present[0] == RFNM_DAUGHTERBOARD_NOT_FOUND || cfg->daughterboard_eeprom[0].board_id == RFNM_DAUGHTERBOARD_BREAKOUT
			))

		) {
			//printk("RFNM: board ids %d %d present %d %d", cfg->daughterboard_eeprom[0].board_id, cfg->daughterboard_eeprom[1].board_id, cfg->daughterboard_present[0], cfg->daughterboard_present[1]);
			return 1;
		}
	} else {
		if (cfg->daughterboard_eeprom[0].board_id == daughterboard_1 && cfg->daughterboard_eeprom[1].board_id == daughterboard_2) {
			return 1;
		}
	}

	return 0;
}

void rfnm_si5510_set_output_status(struct i2c_client *client, int output_id, int enable_disable)
{
	uint8_t i2c_read_buf[100];
	uint8_t send_output_status_request[] = { 0xF0, 0x0F, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00 };
	uint32_t output_req = output_req = 1 << output_id;

	memcpy(&send_output_status_request[3], &output_req, 4);

	if (enable_disable) {
		send_output_status_request[7] = 1;
		//printk("RFNM: Enabling clock output %d\n", output_id);
	} else {
		//printk("RFNM: Disabling clock output %d\n", output_id);
	}

	rfnm_si5510_i2c_write(client, send_output_status_request, 8);

	do {
		if (i2c_read_buf[0] == 0x90) {
			printk("RFNM: FWERR triggered. See text under Common Errors.\n");
			while (1) {}
		}
		msleep(10);
		rfnm_si5510_i2c_read(client, &i2c_read_buf[0], 5);
	} while (i2c_read_buf[0] != 0x80);


}


static ssize_t rfnm_ext_ref_out_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = to_i2c_client(dev);

	if (buf[1] != 'n' && buf[1] != 'f') {
		printk("RFNM: Si5510: Valid inputs are either 'on' or 'off'");
		return -EINVAL;
	}

	if (buf[1] == 'n') {
		printk("RFNM: Si5510: enabling ext reference output @ 10 MHz\n");
		rfnm_si5510_set_output_status(client, 9, 1);
	} else {
		printk("RFNM: Si5510: disabling external reference output\n");
		rfnm_si5510_set_output_status(client, 9, 0);
	}

	return count;
}

static DEVICE_ATTR_WO(rfnm_ext_ref_out);




struct gpio_desc *si5510_rst_gpio;
struct gpio_desc *la9310_trst_gpio;
struct gpio_desc *la9310_hrst_gpio;
struct gpio_desc *la9310_bootstrap_en_gpio;

struct gpio_desc *power_en_09_gpio;
struct gpio_desc *la9310_power_en_gpio;

static int rfnm_si5510_probe(struct i2c_client *client)
{

	struct rfnm_bootconfig *cfg;
	struct rfnm_eeprom_data *eeprom_data;
	struct resource mem_res;
	char node_name[10];
	int ret;

	strncpy(node_name,"bootconfig",10 );
	ret = la9310_read_dtb_node_mem_region(node_name,&mem_res);
	if(ret != RFNM_DTB_NODE_NOT_FOUND){
		cfg = memremap(mem_res.start, SZ_4M, MEMREMAP_WB);
	}
	else {
		printk("RFNM: func %s Node name %s not found..\n",__func__,node_name);
		return ret;
	}

	// when rebooted without hard power reset, this memory section doesn't get inited to 0xff...
	// move memory reset to uboot?

	// remove pd negotiation workaround: it gets stuck sometimes (non-PD connected, times out)

	if (//cfg->usb_pd_negotiation_in_progress == 1 ||
		cfg->daughterboard_present[0] == RFNM_DAUGHTERBOARD_NOT_CHECKED_YET ||
		cfg->daughterboard_present[1] == RFNM_DAUGHTERBOARD_NOT_CHECKED_YET) {
		printk("RFNM: Deferring Si5510 probe...\n");
		memunmap(cfg);
		return -EPROBE_DEFER;
	}

	s64  uptime_ms;
    uptime_ms = ktime_to_ms(ktime_get_boottime());

	if (uptime_ms < 1000) {
		// complete hack: most PD devices are going to keep probing between 0.7-1 second, so do not start there...
		printk("RFNM: Deferring Si5510 probe...\n");
		return -EPROBE_DEFER;
	}

	printk("RFNM: Starting up Si5510...\n");

	si5510_rst_gpio = devm_gpiod_get(&client->dev, "si5510-rst", GPIOD_OUT_LOW);
	int error;

	if (IS_ERR(si5510_rst_gpio)) {
		error = PTR_ERR(si5510_rst_gpio);
		printk("RFNM: Failed to get enable gpio: %d\n", error);
		return error;
	}

	la9310_trst_gpio = devm_gpiod_get(&client->dev, "la9310-trst", GPIOD_OUT_LOW);

	if (IS_ERR(la9310_trst_gpio)) {
			error = PTR_ERR(la9310_trst_gpio);
			printk("RFNM: Failed to get enable gpio: %d\n", error);
			return error;
		}

	la9310_hrst_gpio = devm_gpiod_get(&client->dev, "la9310-hrst", GPIOD_OUT_LOW);

	if (IS_ERR(la9310_hrst_gpio)) {
		error = PTR_ERR(la9310_hrst_gpio);
		printk("RFNM: Failed to get enable gpio: %d\n", error);
		return error;
	}

	la9310_bootstrap_en_gpio = devm_gpiod_get(&client->dev, "la9310-bootstrap-en", GPIOD_OUT_HIGH);

	if (IS_ERR(la9310_bootstrap_en_gpio)) {
		error = PTR_ERR(la9310_bootstrap_en_gpio);
		printk("RFNM: Failed to get enable gpio: %d\n", error);
		return error;
	}

	power_en_09_gpio = devm_gpiod_get(&client->dev, "09v-power-en", GPIOD_OUT_LOW);

	if (IS_ERR(power_en_09_gpio)) {
		error = PTR_ERR(power_en_09_gpio);
		printk("RFNM: Failed to get enable gpio: %d\n", error);
		return error;
	}

	la9310_power_en_gpio = devm_gpiod_get(&client->dev, "la9310-power-en", GPIOD_OUT_LOW);

	if (IS_ERR(la9310_power_en_gpio)) {
		error = PTR_ERR(la9310_power_en_gpio);
		printk("RFNM: Failed to get enable gpio: %d\n", error);
		return error;
	}

	gpiod_set_value_cansleep(si5510_rst_gpio, 0);
	msleep(10);
	gpiod_set_value_cansleep(si5510_rst_gpio, 1);

	rfnm_si5510_cts(client);

	rfnm_si5510_sio_test(client);

	int CMD_BUFFER_SIZE = rfnm_si5510_buffsize(client);

	rfnm_si5510_restart(client);

	rfnm_si5510_host_load(client, prod_fw_boot_bin, prod_fw_boot_bin_len, CMD_BUFFER_SIZE);
	rfnm_si5510_host_load(client, Base_Plan_boot_bin, Base_Plan_boot_bin_len, CMD_BUFFER_SIZE);

	rfnm_si5510_boot(client);

	if (can_use_si5510_config(cfg, RFNM_DAUGHTERBOARD_GRANITA, RFNM_DAUGHTERBOARD_GRANITA)) {
		rfnm_si5510_host_load(client, Q_Plan1_boot_bin, Q_Plan1_boot_bin_len, CMD_BUFFER_SIZE);
		printk("RFNM: Selected plan 1 RFNM_DAUGHTERBOARD_GRANITA, RFNM_DAUGHTERBOARD_GRANITA\n");
	} else if (can_use_si5510_config(cfg, RFNM_DAUGHTERBOARD_LIME, RFNM_DAUGHTERBOARD_LIME)) {
		rfnm_si5510_host_load(client, Q_Plan2_boot_bin, Q_Plan2_boot_bin_len, CMD_BUFFER_SIZE);
		printk("RFNM: Selected plan 2 RFNM_DAUGHTERBOARD_LIME, RFNM_DAUGHTERBOARD_LIME\n");
	} else if (can_use_si5510_config(cfg, RFNM_DAUGHTERBOARD_GRANITA, RFNM_DAUGHTERBOARD_LIME)) {
		rfnm_si5510_host_load(client, Q_Plan3_boot_bin, Q_Plan3_boot_bin_len, CMD_BUFFER_SIZE);
		printk("RFNM: Selected plan 3 RFNM_DAUGHTERBOARD_GRANITA, RFNM_DAUGHTERBOARD_LIME\n");
	} else if (can_use_si5510_config(cfg, RFNM_DAUGHTERBOARD_LIME, RFNM_DAUGHTERBOARD_GRANITA)) {
		rfnm_si5510_host_load(client, Q_Plan4_boot_bin, Q_Plan4_boot_bin_len, CMD_BUFFER_SIZE);
		printk("RFNM: Selected plan 4 RFNM_DAUGHTERBOARD_LIME, RFNM_DAUGHTERBOARD_GRANITA\n");
	} else {
		printk("RFNM: Couldn't find Si5510 config to work with the installed daughterboards\n");
	}

	if (cfg->daughterboard_present[0] == RFNM_DAUGHTERBOARD_PRESENT && cfg->daughterboard_eeprom[0].board_id != RFNM_DAUGHTERBOARD_BREAKOUT) {
		rfnm_si5510_set_output_status(client, 11, 1);
		if (cfg->daughterboard_eeprom[0].board_id != RFNM_DAUGHTERBOARD_LIME) {
			rfnm_si5510_set_output_status(client, 15, 1);
		}
		printk("RFNM: Enabling clocks for RBA\n");
	}

	if (cfg->daughterboard_present[1] == RFNM_DAUGHTERBOARD_PRESENT && cfg->daughterboard_eeprom[1].board_id != RFNM_DAUGHTERBOARD_BREAKOUT) {
		rfnm_si5510_set_output_status(client, 0, 1);
		if (cfg->daughterboard_eeprom[0].board_id != RFNM_DAUGHTERBOARD_LIME) {
			rfnm_si5510_set_output_status(client, 2, 1);
		}
		printk("RFNM: Enabling clocks for RBB\n");
	}






	printk("RFNM: Waiting for reference clock to lock...\n");

	while (!rfnm_si5510_reference_status(client)) {
		msleep(10);
	}

	printk("RFNM: Si5510 is ready and providing a PCIe clock!\n");

	cfg->pcie_clock_ready = 1;

	gpiod_set_value(la9310_hrst_gpio, 0);
	gpiod_set_value(la9310_trst_gpio, 0);

	gpiod_set_value(la9310_bootstrap_en_gpio, 0);

	gpiod_set_value(power_en_09_gpio, 1);
	gpiod_set_value(la9310_power_en_gpio, 1);

	msleep(10);

	// merge this into single register write?
	gpiod_set_value(la9310_trst_gpio, 1);
	gpiod_set_value(la9310_hrst_gpio, 1);

	msleep(10);

	gpiod_set_value(la9310_bootstrap_en_gpio, 1);

	printk("RFNM: Performed LA9310 reset\n");

	// release LA9310 GPIOs for people to play with it in userspace (JTAG, etc).

	gpiod_put(la9310_trst_gpio);
	gpiod_put(la9310_hrst_gpio);
	gpiod_put(la9310_bootstrap_en_gpio);
	gpiod_put(la9310_power_en_gpio);

	// cannot load wsled because it's not init'd yet... not sure why the order changed
	//rfnm_wsled_set(0, 0, 0, 0, 0xff);
	//rfnm_wsled_send_chain(0);

	int err;

	err = device_create_file(&client->dev, &dev_attr_rfnm_ext_ref_out);
	if (err < 0) {
		printk("RFNM: failed to create device file for rfnm_ext_ref_out");
	}
	//device_remove_file(&client->dev, &(dev_attr_rfnm_show_board_info));

	return 0;

}



static const struct of_device_id rfnm_si5510_match_table[] = {
	{ .compatible = "rfnm,si5510", },
	{}
};
MODULE_DEVICE_TABLE(of, rfnm_si5510_match_table);

static const struct i2c_device_id rfnm_si5510_id_table[] = {
	{ "rfnm_si5510", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rfnm_si5510_id_table);

static struct i2c_driver rfnm_si5510_driver = {
	.driver	= {
		.name	= "rfnm_si5510",
		.of_match_table = rfnm_si5510_match_table,
	},
	.probe_new	= rfnm_si5510_probe,
	.id_table	= rfnm_si5510_id_table,
};
module_i2c_driver(rfnm_si5510_driver);
MODULE_LICENSE("GPL");
