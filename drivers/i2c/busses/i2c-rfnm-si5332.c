// SPDX-License-Identifier: GPL-2.0+
/*  Copyright 2024 NXP
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/of_address.h>

#include <linux/rfnm-shared.h>
#include <linux/rfnm-si5332.h>
#include <linux/printk.h>
#include <linux/i2c.h>

#include <linux/ktime.h>
#define MAX_NODE_NAME_LEN 10

typedef unsigned char       uint8_t;
typedef   signed char        int8_t;

void rfnm_si5332_i2c_read(struct i2c_client *client, uint8_t * buf, int cnt) {

}

void rfnm_si5332_i2c_write(struct i2c_client *client, uint8_t * buf, int cnt) {

       i2c_master_send(client, buf, cnt);
}

static struct gpio_desc *la9310_trst_gpio;
static struct gpio_desc *la9310_hrst_gpio;
static struct gpio_desc *la9310_bootstrap_en_gpio;

static struct gpio_desc *power_en_09_gpio;
static struct gpio_desc *la9310_power_en_gpio;
static bool attr_deferred_probe_trigger = false;

static ssize_t deferred_probe_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	// Ensure the input buffer not NULL and is exactly "1\n"
	if (!buf || count != 2 || strncmp(buf, "1\n", 2) != 0) {
		return -EINVAL;
	}
	// Perform the deferred probe trigger
	deferred_probe_trigger();
	return count;
}
static DEVICE_ATTR_WO(deferred_probe_trigger);

static int rfnm_si5332_probe(struct i2c_client *client) {

       int err, i;
       struct rfnm_bootconfig *cfg;
       struct rfnm_eeprom_data *eeprom_data;
       struct resource mem_res;
	char node_name[ MAX_NODE_NAME_LEN + 1 ];
	int ret;

	strncpy(node_name,"bootconfig",MAX_NODE_NAME_LEN);
	node_name[MAX_NODE_NAME_LEN] = '\0';
	ret = la9310_read_dtb_node_mem_region(node_name,&mem_res);
	if(ret != RFNM_DTB_NODE_NOT_FOUND){
		cfg = memremap(mem_res.start, SZ_4M, MEMREMAP_WB);
	}
	else {
		printk("RFNM: func %s Node name %s not found..\n",__func__,node_name);
		return ret;
	}

       if(device_property_read_bool(&client->dev, "rfnm,skip-5510-init-quirk")) {
               cfg->pcie_clock_ready = 1;
               printk("RFNM: skip-5510-init-quirk\n");
               return 0;
       }
       // when rebooted without hard power reset, this memory section doesn't get inited to 0xff...
       // move memory reset to uboot?

       // remove pd negotiation workaround: it gets stuck sometimes (non-PD connected, times out)

	if (attr_deferred_probe_trigger) {
		printk("RFNM: device file for deferred probe trigger already exists\n");
	} else {
		// Attempt to create the device file
		err = device_create_file(&client->dev, &dev_attr_deferred_probe_trigger);
		if (err < 0) {
			printk("RFNM: failed to create device file for deferred probe trigger\n");
		} else {
			attr_deferred_probe_trigger = true;
			printk("RFNM: created device file for deferred probe trigger\n");
		}
	}

       s64  uptime_ms;
    uptime_ms = ktime_to_ms(ktime_get_boottime());

       if(uptime_ms < 1000) {
               // complete hack: most PD devices are going to keep probing between 0.7-1 second, so do not start there...
               printk("RFNM: Deferring Si5332 probe...\n");
               return -EPROBE_DEFER;
       }

       printk("RFNM: Starting up Si5332...\n");

       int error;

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

       msleep(10);

       cfg->user_eeprom.dcs_clk_tmp = 122;

       for(i = 0; i < SI5332_GM1_REVD_REG_CONFIG_NUM_REGS; i++) {
               uint8_t buf[2];
               memcpy(&buf[0], &si5332_gm1_revd_registers[i].address, 1);
               memcpy(&buf[1], &si5332_gm1_revd_registers[i].value, 1);

               rfnm_si5332_i2c_write(client, &buf[0], 2);
               //printk("%02x %02x\n", buf[0], buf[1]);
       }

       printk("RFNM: Si5332 is ready and providing a PCIe clock!\n");

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

       memunmap(cfg);

       return 0;

}



static const struct of_device_id rfnm_si5332_match_table[] = {
       { .compatible = "rfnm,si5332", },
       {}
};
MODULE_DEVICE_TABLE(of, rfnm_si5332_match_table);

static const struct i2c_device_id rfnm_si5332_id_table[] = {
       { "rfnm_si5332", 0 },
       { },
};
MODULE_DEVICE_TABLE(i2c, rfnm_si5332_id_table);

static struct i2c_driver rfnm_si5332_driver = {
       .driver = {
               .name   = "rfnm_si5332",
               .of_match_table = rfnm_si5332_match_table,
       },
       .probe_new      = rfnm_si5332_probe,
       .id_table       = rfnm_si5332_id_table,
};
module_i2c_driver(rfnm_si5332_driver);
MODULE_LICENSE("GPL");
