// SPDX-License-Identifier: GPL-2.0+
/*  Copyright 2024-2025 NXP
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

#include <linux/regulator/consumer.h>

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
static struct gpio_desc *mt1_1p67v_en_gpio;
static struct gpio_desc *mt1_nrst_gpio;
static struct gpio_desc *mt1_trx_gpio;

struct regulator *vreg18;
struct regulator *vreg09;
struct regulator *vreg20;
struct regulator *vreg17;


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

       int err, i, error;
       struct rfnm_bootconfig *cfg;
       struct resource mem_res;
       char node_name[ MAX_NODE_NAME_LEN + 1 ];
       int ret;
       s64  uptime_ms;

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

       uptime_ms = ktime_to_ms(ktime_get_boottime());

       if(uptime_ms < 1000) {
               // complete hack: most PD devices are going to keep probing between 0.7-1 second, so do not start there...
               printk("RFNM: Deferring Si5332 probe...\n");
               return -EPROBE_DEFER;
       }

       printk("RFNM: Starting up Si5332...\n");

       // assert jtag reset
       la9310_trst_gpio = devm_gpiod_get(&client->dev, "la9310-trst", GPIOD_OUT_LOW);

       if(IS_ERR(la9310_trst_gpio))
               pr_err("Si5332: Failed to get la9310-trst gpio: %d\n", ret);

       // assert Power On Reset (POR) for LA9310
       la9310_hrst_gpio = devm_gpiod_get(&client->dev, "la9310-hrst", GPIOD_OUT_LOW);

       if(IS_ERR(la9310_hrst_gpio))
               pr_err("Si5332: Failed to get la9310-hrst gpio: %d\n", ret);

       // assert bootstrap mode
       la9310_bootstrap_en_gpio = devm_gpiod_get_optional( &client->dev, "la9310-bootstrap-en", GPIOD_OUT_LOW);

       if(IS_ERR(la9310_bootstrap_en_gpio))
               pr_err("Si5332: Failed to get la9310-bootstrap-en gpio: %d\n", ret);

       // disable 0.9v power
       power_en_09_gpio = devm_gpiod_get_optional(&client->dev, "09v-power-en", GPIOD_OUT_LOW);

       if(IS_ERR(power_en_09_gpio))
               pr_err("Si5332: Failed to get 09v-power-en gpio: %d\n", ret);

       // disable LA9310 power
       la9310_power_en_gpio = devm_gpiod_get(&client->dev, "la9310-power-en", GPIOD_OUT_LOW);

       if(IS_ERR(la9310_power_en_gpio))
               pr_err("Si5332: Failed to get la9310-power-en: %d\n", ret);

       // assert mt1 reset
       mt1_nrst_gpio = devm_gpiod_get_optional(&client->dev, "mt1-nrst", GPIOD_OUT_HIGH);

       if(IS_ERR(mt1_nrst_gpio))
               pr_err("Si5332: Failed to get mt1-nrst: %d\n", ret);


       // disable ldo on mt1
       mt1_1p67v_en_gpio = devm_gpiod_get_optional(&client->dev, "mt1-1p67v-en", GPIOD_OUT_LOW);

       if(IS_ERR(mt1_1p67v_en_gpio))
               pr_err("Si5332: Failed to get mt1-1p67v-en: %d\n", ret);

       // set RX low for spi mode
       mt1_trx_gpio = devm_gpiod_get_optional(&client->dev, "mt1-trx", GPIOD_OUT_LOW);

       if(IS_ERR(mt1_trx_gpio))
               pr_err("Si5332: Failed to get mt1-trx: %d\n", ret);

       // BUCK0 - 1.8v enable
       vreg18 = devm_regulator_get_optional(&client->dev, "lp8758-1v8");
       if (IS_ERR(vreg18)) {
               ret = PTR_ERR(vreg18);
               pr_err("Si5332: Failed to get buck0 regulator: %d\n", ret);
       }
       if (vreg18>0) {
               pr_info("Si5332: Got buck0 regulator (1.8v)\n");
               ret=regulator_is_enabled(vreg18);
               if(ret<0)
                       pr_err("Si5332: Failed to get buck0 regulator_is_enabled: %d\n", ret);
               if (ret>0)
                       pr_info("Si5332: buck0 regulator already enabled\n");

               if (!ret) {
                       ret=regulator_enable(vreg18);
                       if(!ret)  
		               pr_err("Si5332: fail to enale buck0\n");
		       else
                               pr_info("Si5332: enabling buck0 regulator voltage at 1.80v\n");
	       }
       }

       // BUCK1 - disable 0.9v , enable later
       vreg09 = devm_regulator_get_optional(&client->dev, "lp8758-09v");
       if (IS_ERR(vreg09)) {
               ret = PTR_ERR(vreg09);
               pr_err("Si5332: Failed to get buck1 regulator: %d\n", ret);
       } 
       if (vreg09>0) {
               pr_info("Si5332: Got buck1 regulator (0.9v)\n");
               ret=regulator_is_enabled(vreg09);
               if(ret<0)
	                 pr_err("Si5332: Failed to get buck1 regulator_is_enabled: %d\n", ret);
               if (!ret)
                        pr_info("Si5332: buck1 regulator already disabled\n");
               if (ret>0) {
                        ret=regulator_force_disable(vreg09);
                        if(ret)
				pr_err("Si5332: fail to disable bulk1 %d\n",ret);
			else
                                pr_info("Si5332: disabling buck1 regulator voltage at 0.9v\n");
	       }
       }


       // BUCK2 - enable 2.0v - ldos control power to the chips
       vreg20 = devm_regulator_get_optional(&client->dev, "lp8758-2v");
       if (IS_ERR(vreg20)) {
               ret = PTR_ERR(vreg20);
               pr_err("Si5332: Failed to get buck2 regulator: %d\n", ret);
       }
       if (vreg20>0) {
               pr_info("Si5332: Got buck2 regulator (2.0v)\n");
               ret=regulator_is_enabled(vreg20);
               if(ret<0)
                       pr_err("Si5332: Failed to get buck2 regulator_is_enabled: %d\n", ret);
               if(ret > 0)
                       pr_info("Si5332: buck2 regulator already enabled\n");
                if (!ret) {
                       ret=regulator_enable(vreg20);
                       if(!ret)  
			       pr_err("Si5332: fail to enale buck2 : %d\n",ret);
		       else
			       pr_info("Si5332: enabling buck2 regulator voltage at 2.0v\n");
               }
        }

        // BUCK3 - permanently disable 1.67v rail
        vreg17 = devm_regulator_get_optional(&client->dev, "lp8758-1v67");
        if (IS_ERR(vreg17)) {
               ret = PTR_ERR(vreg17);
               pr_err("Si5332: Failed to get buck3 regulator: %d\n", ret);
        } 
	if (vreg17>0) {
               pr_info("Si5332: Got buck3 regulator (1.67v)\n");
               ret=regulator_is_enabled(vreg17);
               if(ret<0)
                       pr_err("Si5332: Failed to get buck3 regulator_is_enabled: %d\n", ret);
              if(!ret)
                      pr_info("Si5332: buck3 regulator already disabled\n");
               if (ret>0) {
                      ret=regulator_force_disable(vreg17);
                      if(ret)
			      pr_err("Si5332: fail to disable bulk3 %d\n",ret);
		      else
			      pr_info("Si5332: disabling buck3 regulator voltage at 1.67v\n");
              }
        } 

       if(la9310_trst_gpio>0) pr_info("Si5332: la9310-trst = %d\n", gpiod_get_raw_value(la9310_trst_gpio));
       if(la9310_hrst_gpio>0) pr_info("Si5332: la9310-hrst = %d\n", gpiod_get_raw_value(la9310_hrst_gpio));
       if(la9310_bootstrap_en_gpio>0) pr_info("Si5332: la9310-nbootstrap-en = %d\n", gpiod_get_raw_value(la9310_bootstrap_en_gpio));
       if(la9310_power_en_gpio>0) pr_info("Si5332: la9310-power-en = %d\n", gpiod_get_raw_value(la9310_power_en_gpio));
       if(mt1_nrst_gpio>0) pr_info("Si5332: mt1-nrst = %d\n", gpiod_get_raw_value(mt1_nrst_gpio));
       if(mt1_1p67v_en_gpio>0) pr_info("Si5332: MT1 1.67v raw val = %d\n", gpiod_get_raw_value(mt1_1p67v_en_gpio));
       if(mt1_trx_gpio>0) pr_info("Si5332: mt1-trx = %d\n", gpiod_get_raw_value(mt1_trx_gpio));
       if(vreg18>0) pr_info("Si5332: vreg18 enbale = %d\n", regulator_is_enabled(vreg18));
       if(vreg09>0) pr_info("Si5332: vreg09 enbale = %d\n", regulator_is_enabled(vreg09));
       if(vreg20>0) pr_info("Si5332: vreg20 enbale = %d\n", regulator_is_enabled(vreg20));
       if(vreg17>0) pr_info("Si5332: vreg17 enbale = %d\n", regulator_is_enabled(vreg17));

        msleep(10);

        cfg->user_eeprom.dcs_clk_tmp = 122;

        for (i = 0; i < SI5332_GM1_REVD_REG_CONFIG_NUM_REGS; i++) {
                uint8_t buf[2];
                memcpy(&buf[0], &si5332_gm1_revd_registers[i].address, 1);
                memcpy(&buf[1], &si5332_gm1_revd_registers[i].value, 1);
                rfnm_si5332_i2c_write(client, &buf[0], 2);
        }

        pr_info("Si5332: chip is ready and providing a PCIe clock!\n");

	cfg->pcie_clock_ready = 1;

       // all reset GPIO and bootstrap asserted (LA9310 and MT3812), bring up Power rails

       if(power_en_09_gpio>0){
               gpiod_set_value(power_en_09_gpio, 1);
               pr_info("Si5332: power_en_09_gpio asserted\n");
       }

       if(vreg09>0){
               ret = regulator_enable(vreg09);
               if (ret)
                       pr_info("RFNM: Failed to enable regulator lp8758-09v: %d\n", error);
               else
                       pr_info("Si5332: Enabling buck1 regulator at 0.9v\n");
       }

       gpiod_set_value(la9310_power_en_gpio, 1);
       pr_info("Si5332: la9310_power_en_gpio asserted\n");

       if(mt1_1p67v_en_gpio>0){
               gpiod_set_value(mt1_1p67v_en_gpio, 1);
               pr_info("Si5332: mt1-1p67v-en asserted\n");
       }

       msleep(10);

       // merge this into single register write?
       gpiod_set_value(la9310_trst_gpio, 1);
       gpiod_set_value(la9310_hrst_gpio, 1);

       msleep(10);

       if(la9310_bootstrap_en_gpio>0){
	       gpiod_set_value(la9310_bootstrap_en_gpio, 1);
               pr_info("Si5332: mla9310_bootstrap_en_gpi deasserted\n");
       }

       if(mt1_nrst_gpio>0){
	       gpiod_set_value(mt1_nrst_gpio, 0);
               pr_info("Si5332: mt1_nrst_gpio deasserted\n");
       }

       if(la9310_trst_gpio>0) pr_info("Si5332: la9310-trst = %d\n", gpiod_get_raw_value(la9310_trst_gpio));
       if(la9310_hrst_gpio>0) pr_info("Si5332: la9310-hrst = %d\n", gpiod_get_raw_value(la9310_hrst_gpio));
       if(la9310_bootstrap_en_gpio>0) pr_info("Si5332: la9310-nbootstrap-en = %d\n", gpiod_get_raw_value(la9310_bootstrap_en_gpio));
       if(la9310_power_en_gpio>0) pr_info("Si5332: la9310-power-en = %d\n", gpiod_get_raw_value(la9310_power_en_gpio));
       if(mt1_nrst_gpio>0) pr_info("Si5332: mt1-nrst = %d\n", gpiod_get_raw_value(mt1_nrst_gpio));
       if(mt1_1p67v_en_gpio>0) pr_info("Si5332: MT1 1.67v raw val = %d\n", gpiod_get_raw_value(mt1_1p67v_en_gpio));
       if(mt1_trx_gpio>0) pr_info("Si5332: mt1-trx = %d\n", gpiod_get_raw_value(mt1_trx_gpio));
       if(vreg18>0) pr_info("Si5332: vreg18 enbale = %d\n", regulator_is_enabled(vreg18));
       if(vreg09>0) pr_info("Si5332: vreg09 enbale = %d\n", regulator_is_enabled(vreg09));
       if(vreg20>0) pr_info("Si5332: vreg20 enbale = %d\n", regulator_is_enabled(vreg20));
       if(vreg17>0) pr_info("Si5332: vreg17 enbale = %d\n", regulator_is_enabled(vreg17));

       // release LA9310 GPIOs for people to play with it in userspace (JTAG, etc).

       if(la9310_trst_gpio>0) gpiod_put(la9310_trst_gpio);
       if(la9310_hrst_gpio>0) gpiod_put(la9310_hrst_gpio);
       if(la9310_bootstrap_en_gpio>0) gpiod_put(la9310_bootstrap_en_gpio);
       if(la9310_power_en_gpio>0) gpiod_put(la9310_power_en_gpio);
       if(mt1_nrst_gpio>0) gpiod_put(mt1_nrst_gpio);
       if(power_en_09_gpio>0) gpiod_put(power_en_09_gpio);
       if(mt1_1p67v_en_gpio>0) gpiod_put(mt1_1p67v_en_gpio);
       if(mt1_trx_gpio>0) gpiod_put(mt1_trx_gpio);
 
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
