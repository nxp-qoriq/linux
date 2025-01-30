/*
* Simple driver for Texas Instruments lp8758 Regulator chip
*
* Copyright (C) 2015 Texas Instruments
* Author: Daniel Jeong  <daniel.jeong@ti.com>
*         Ldd Mlp <ldd-mlp@list.ti.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/lp8758.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct lp8758_chip {
       struct device *dev;
       struct regmap *regmap;
       struct lp8758_platform_data *pdata;

       struct regulator_dev *rdev[LP8758_BUCK_MAX];
       struct regulator_desc regulators[LP8758_BUCK_MAX];
};

/* voltage map */
static const unsigned int lp8758_buck_voltage_map[] = {
       /* 0.50V - 0.73V : 10mV steps */
        500000,  510000,  520000,  530000,  540000,  550000,  560000,  570000,
        580000,  590000,  600000,  610000,  620000,  630000,  640000,  650000,
        660000,  670000,  680000,  690000,  700000,  710000,  720000,  730000,
       /* 0.73V - 1.40V :  5mV steps */
        735000,  740000,  745000,  750000,  755000,  760000,  765000,  770000,
        775000,  780000,  785000,  790000,  795000,  800000,  805000,  810000,
        815000,  820000,  825000,  830000,  835000,  840000,  845000,  850000,
        855000,  860000,  865000,  870000,  875000,  880000,  885000,  890000, 
        895000,  900000,  905000,  910000,  915000,  920000,  925000,  930000,
        935000,  940000,  945000,  950000,  955000,  960000,  965000,  970000,
        975000,  980000,  985000,  990000,  995000, 1000000, 1005000, 1010000,
       1015000, 1020000, 1025000, 1030000, 1035000, 1040000, 1045000, 1050000,
       1055000, 1060000, 1065000, 1070000, 1075000, 1080000, 1085000, 1090000,
       1095000, 1100000, 1105000, 1110000, 1115000, 1120000, 1125000, 1130000,
       1135000, 1140000, 1145000, 1150000,     1155000, 1160000, 1165000, 1170000,
       1175000, 1180000, 1185000, 1190000, 1195000, 1200000, 1205000, 1210000,
       1215000, 1220000, 1225000, 1230000, 1235000, 1240000, 1245000, 1250000,
       1255000, 1260000, 1265000, 1270000, 1275000, 1280000, 1285000, 1290000,
       1295000, 1300000, 1305000, 1310000, 1315000, 1320000, 1325000, 1330000,
       1335000, 1340000, 1345000, 1350000,     1355000, 1360000, 1365000, 1370000,
       1375000, 1380000, 1385000, 1390000, 1395000, 1400000,
       /* 1.40V - 3.36V : 20mV steps */
       1420000, 1440000, 1460000, 1480000, 1500000, 1520000, 1540000, 1560000,
       1580000, 1600000, 1620000, 1640000, 1660000, 1680000, 1700000, 1720000,
       1740000, 1760000, 1780000, 1800000, 1820000, 1840000, 1860000, 1880000,
       1900000, 1920000, 1940000, 1960000, 1980000, 2000000, 2020000, 2040000,
       2060000, 2080000, 2100000, 2120000, 2140000, 2160000, 2180000, 2200000,
       2220000, 2240000, 2260000, 2280000, 2300000, 2320000, 2340000, 2360000,
       2380000, 2400000, 2420000, 2440000, 2460000, 2480000, 2500000, 2520000,
       2540000, 2560000, 2580000, 2600000, 2620000, 2640000, 2660000, 2680000,
       2700000, 2720000, 2740000, 2760000, 2780000, 2800000, 2820000, 2840000,
       2860000, 2880000, 2900000, 2920000, 2940000, 2960000, 2980000, 3000000,
       3020000, 3040000, 3060000, 3080000, 3100000, 3120000, 3140000, 3160000,
       3180000, 3200000, 3220000, 3240000, 3260000, 3280000, 3300000, 3320000,
       3340000, 3360000,
};

/* current limit */
static const unsigned int lp8758_current_limit_uA[] = {
       1500000, 2000000, 2500000, 3000000, 3500000, 4000000, 4500000, 5000000
};

const static int sub_version_config[LP8758_SUB_VER_MAX][LP8758_BUCK_MAX] ={
       [LP8758_SUB_VER_B0] = { LP8758_BUCK_MASTER, LP8758_BUCK_SLAVE,
                                                       LP8758_BUCK_SLAVE, LP8758_BUCK_SLAVE },
       [LP8758_SUB_VER_D0] = { LP8758_BUCK_MASTER, LP8758_BUCK_SLAVE,
                                                       LP8758_BUCK_MASTER, LP8758_BUCK_MASTER },
       [LP8758_SUB_VER_E0] = { LP8758_BUCK_MASTER, LP8758_BUCK_MASTER,
                                                       LP8758_BUCK_MASTER, LP8758_BUCK_MASTER },
       [LP8758_SUB_VER_F0] = { LP8758_BUCK_MASTER, LP8758_BUCK_SLAVE,
                                                       LP8758_BUCK_MASTER, LP8758_BUCK_SLAVE }
};

static bool lp8758_is_master(struct lp8758_chip *pchip, enum lp8758_bucks buck) {
       int ver = pchip->pdata->sub_version;
       if(sub_version_config[ver][buck] == LP8758_BUCK_MASTER)
               return true;
       return false;
}

static unsigned int lp8758_get_ctrl1_address(enum lp8758_bucks buck){
       unsigned int ctrl_register[LP8758_BUCK_MAX] = {
               [LP8758_BUCK0] = LP8758_REG_BUCK0_CTRL1,
               [LP8758_BUCK1] = LP8758_REG_BUCK1_CTRL1,
               [LP8758_BUCK2] = LP8758_REG_BUCK2_CTRL1,
               [LP8758_BUCK3] = LP8758_REG_BUCK3_CTRL1
       };
       return ctrl_register[buck];
}

static unsigned int lp8758_get_ctrl2_address(enum lp8758_bucks buck){
       unsigned int ctrl_register[LP8758_BUCK_MAX] = {
               [LP8758_BUCK0] = LP8758_REG_BUCK0_CTRL2,
               [LP8758_BUCK1] = LP8758_REG_BUCK1_CTRL2,
               [LP8758_BUCK2] = LP8758_REG_BUCK2_CTRL2,
               [LP8758_BUCK3] = LP8758_REG_BUCK3_CTRL2
       };
       return ctrl_register[buck];
}

static int lp8758_buck_set_current_limit(struct regulator_dev *rdev,
                                       int min_uA, int max_uA)
{
       struct lp8758_chip *pchip = rdev_get_drvdata(rdev);
       enum lp8758_bucks buck = rdev_get_id(rdev);
       int icnt;

       if(buck > LP8758_BUCK_MAX-1)
               return -EINVAL;

       for (icnt = ARRAY_SIZE(lp8758_current_limit_uA) - 1; icnt >= 0; icnt--) {
               if (lp8758_current_limit_uA[icnt] >= min_uA &&
                       lp8758_current_limit_uA[icnt] <= max_uA){
                               return regmap_update_bits(pchip->regmap,
                                                       lp8758_get_ctrl2_address(buck),
                                                       LP8758_ILIM_MASK, icnt << LP8758_ILIM_SHIFT);
                       }
       }

       return -EINVAL;
}

static int lp8758_buck_get_current_limit(struct regulator_dev *rdev)
{
       struct lp8758_chip *pchip = rdev_get_drvdata(rdev);
       enum lp8758_bucks buck = rdev_get_id(rdev);
       unsigned int val;
       int ret;

       if(buck > LP8758_BUCK_MAX-1)
               return -EINVAL;

       ret = regmap_read(pchip->regmap, lp8758_get_ctrl2_address(buck), &val);
       if (ret)
               return ret;

       val = (val & LP8758_ILIM_MASK) >> LP8758_ILIM_SHIFT;

       return (val < ARRAY_SIZE(lp8758_current_limit_uA)) ?
                       lp8758_current_limit_uA[val] : -EINVAL;
}

static int lp8758_buck_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
       struct lp8758_chip *pchip = rdev_get_drvdata(rdev);
       enum lp8758_bucks buck = rdev_get_id(rdev);
       unsigned int addr;

       if(buck > LP8758_BUCK_MAX-1)
               return -EINVAL;

       addr = lp8758_get_ctrl1_address(buck);
       if (mode == REGULATOR_MODE_FAST)
               return regmap_update_bits(pchip->regmap, addr,
                                                                       LP8758_BUCK_OP_MODE_MASK,
                                                                       LP8758_BUCK_OP_FPWM);
       else if (mode == REGULATOR_MODE_NORMAL)
               return regmap_update_bits(pchip->regmap, addr,
                                                                       LP8758_BUCK_OP_MODE_MASK,
                                                                       LP8758_BUCK_OP_AUTO);
       else
               return -EINVAL;
}

static unsigned int lp8758_buck_get_mode(struct regulator_dev *rdev)
{
       struct lp8758_chip *pchip = rdev_get_drvdata(rdev);
       enum lp8758_bucks buck = rdev_get_id(rdev);
       unsigned int val;
       int ret;

       if(buck > LP8758_BUCK_MAX-1)
               return -EINVAL;

       ret = regmap_read(pchip->regmap, lp8758_get_ctrl1_address(buck), &val);
       if (ret)
               return ret;

       return (val & LP8758_BUCK_OP_MODE_MASK)
                                       ? REGULATOR_MODE_FAST : REGULATOR_MODE_NORMAL;
}

static int lp8758_buck_set_ramp(struct regulator_dev *rdev, int ramp)
{
       int ret = -EINVAL;
       unsigned int regval = 0x00;
       enum lp8758_bucks buck = rdev_get_id(rdev);
       struct lp8758_chip *pchip = rdev_get_drvdata(rdev);

       if(buck > LP8758_BUCK_MAX-1)
               return -EINVAL;

       if(lp8758_is_master(pchip,buck)) {
               /* uV/us */
               switch (ramp) {
               case 0 ... 470:
                       regval = 0x07;
                       break;
               case 471 ... 940:
                       regval = 0x06;
                       break;
               case 941 ... 1900:
                       regval = 0x05;
                       break;
               case 1901 ... 3800:
                       regval = 0x04;
                       break;
               case 3801 ... 7500:
                       regval = 0x03;
                       break;
               case 7501 ... 10000:
                       regval = 0x02;
                       break;
               case 10001 ... 15000:
                       regval = 0x01;
                       break;
               case 15001 ... 30000:
                       regval = 0x00;
                       break;
               default:
                       dev_err(pchip->dev,
                               "Not supported ramp value %d %s\n", ramp, __func__);
                       return -EINVAL;
               }
               ret = regmap_update_bits(pchip->regmap, lp8758_get_ctrl2_address(buck),
                                                                       LP8758_BUCK_RAMP_MASK, regval);
       }
       return ret;
}

/* regulator operation when it is set to master */
static struct regulator_ops lp8758_buck_master_ops = {
       .map_voltage = regulator_map_voltage_ascend,
       .list_voltage = regulator_list_voltage_table,
       .set_voltage_sel = regulator_set_voltage_sel_regmap,
       .get_voltage_sel = regulator_get_voltage_sel_regmap,
       .enable = regulator_enable_regmap,
       .disable = regulator_disable_regmap,
       .is_enabled = regulator_is_enabled_regmap,
       .set_mode = lp8758_buck_set_mode,
       .get_mode = lp8758_buck_get_mode,
       .set_current_limit = lp8758_buck_set_current_limit,
       .get_current_limit = lp8758_buck_get_current_limit,
       .set_ramp_delay = lp8758_buck_set_ramp,
};

/* regulator operation when it is tied to another as slave */
static struct regulator_ops lp8758_buck_slave_ops = {
       .set_current_limit = lp8758_buck_set_current_limit,
       .get_current_limit = lp8758_buck_get_current_limit,
};

#define lp8758_rail(_id) "lp8755-buck"#_id
/* regulator description when it is set to master */
#define lp8758_buck_master_desc(_id)\
{\
       .name = lp8758_rail(_id),\
       .id   = LP8758_BUCK##_id,\
       .ops  = &lp8758_buck_master_ops,\
       .n_voltages = ARRAY_SIZE(lp8758_buck_voltage_map),\
       .volt_table = lp8758_buck_voltage_map,\
       .type = REGULATOR_VOLTAGE,\
       .owner = THIS_MODULE,\
       .enable_reg = LP8758_REG_BUCK##_id##_CTRL1,\
       .enable_mask = LP8758_BUCK_EN_MASK,\
       .vsel_reg = LP8758_REG_BUCK##_id##_VOUT,\
       .vsel_mask = LP8758_BUCK_VOUT_MASK,\
}

/* regulator description when it is set to master and roof/floor control */
#define lp8758_buck_master_roof_floor_desc(_id)\
{\
       .name = lp8758_rail(_id),\
       .id   = LP8758_BUCK##_id,\
       .ops  = &lp8758_buck_master_ops,\
       .n_voltages = ARRAY_SIZE(lp8758_buck_voltage_map),\
       .volt_table = lp8758_buck_voltage_map,\
       .type = REGULATOR_VOLTAGE,\
       .owner = THIS_MODULE,\
       .enable_reg = LP8758_REG_BUCK##_id##_CTRL1,\
       .enable_mask = LP8758_BUCK_EN_MASK,\
       .vsel_reg = LP8758_REG_BUCK##_id##_FLOORVOUT,\
       .vsel_mask = LP8758_BUCK_VOUT_MASK,\
}

/* regulator description when it is tied to another as slave */
#define lp8758_buck_slave_desc(_id)\
{\
       .name = lp8758_rail(_id),\
       .id   = LP8758_BUCK##_id,\
       .ops  = &lp8758_buck_slave_ops,\
       .type = REGULATOR_CURRENT,\
       .owner = THIS_MODULE,\
}

/* regulators description for all configuration  */
static struct regulator_desc lp8758_buck_master_config[LP8758_BUCK_MAX][LP8758_CTRL_MAX] = {
       {
               [LP8758_CTRL_MODE0] = lp8758_buck_master_desc(0),
               [LP8758_CTRL_MODE1] = lp8758_buck_master_desc(0),
               [LP8758_CTRL_MODE2] = lp8758_buck_master_roof_floor_desc(0),
               [LP8758_CTRL_MODE3] = lp8758_buck_master_desc(0),
               [LP8758_CTRL_MODE4] = lp8758_buck_master_roof_floor_desc(0)
       },
       {
               [LP8758_CTRL_MODE0] = lp8758_buck_master_desc(1),
               [LP8758_CTRL_MODE1] = lp8758_buck_master_desc(1),
               [LP8758_CTRL_MODE2] = lp8758_buck_master_roof_floor_desc(1),
               [LP8758_CTRL_MODE3] = lp8758_buck_master_desc(1),
               [LP8758_CTRL_MODE4] = lp8758_buck_master_roof_floor_desc(1)
       },
       {
               [LP8758_CTRL_MODE0] = lp8758_buck_master_desc(2),
               [LP8758_CTRL_MODE1] = lp8758_buck_master_desc(2),
               [LP8758_CTRL_MODE2] = lp8758_buck_master_roof_floor_desc(2),
               [LP8758_CTRL_MODE3] = lp8758_buck_master_desc(2),
               [LP8758_CTRL_MODE4] = lp8758_buck_master_roof_floor_desc(2)
       },
       {
               [LP8758_CTRL_MODE0] = lp8758_buck_master_desc(3),
               [LP8758_CTRL_MODE1] = lp8758_buck_master_desc(3),
               [LP8758_CTRL_MODE2] = lp8758_buck_master_roof_floor_desc(3),
               [LP8758_CTRL_MODE3] = lp8758_buck_master_desc(3),
               [LP8758_CTRL_MODE4] = lp8758_buck_master_roof_floor_desc(3)
       }
};

static struct regulator_desc lp8758_buck_slave_config[LP8758_BUCK_MAX] = {
       lp8758_buck_slave_desc(0),
       lp8758_buck_slave_desc(1),
       lp8758_buck_slave_desc(2),
       lp8758_buck_slave_desc(3)
};

/*
 * select regulator description for each buck
 * and write configuration value into control register
 */

static struct gpio_desc *lp8758_nreset_gpio;

static int lp8758_regulator_init(struct lp8758_chip *pchip){
       int icnt, ret, bctrl_mode;
       struct regulator_desc *reg;

       lp8758_nreset_gpio  = devm_gpiod_get(pchip->dev, "lp8758-nrst", GPIOD_OUT_LOW);
       if (IS_ERR(lp8758_nreset_gpio)) {
               ret = PTR_ERR(lp8758_nreset_gpio);
               printk("RFNM: Failed to get deassert lp8758-nrst: %d\n", ret);
       }
       if(lp8758_nreset_gpio>0)
	       pr_info("lp8758: lp8758_nreset_gpio = %d\n", gpiod_get_raw_value(lp8758_nreset_gpio));

       msleep(1);

       if(lp8758_nreset_gpio>0){
               gpiod_set_value(lp8758_nreset_gpio, 1);
               pr_info("lp8758: lp8758_nreset_gpio = %d\n", gpiod_get_raw_value(lp8758_nreset_gpio));
       }

       if(pchip->pdata->sub_version > LP8758_SUB_VER_MAX-1)
               return -EINVAL;

       /* select regulator description based on sub version*/
       for(icnt = LP8758_BUCK0 ; icnt < LP8758_BUCK_MAX; icnt++){
               /* select regulator description based on sub version & control mode */
               if(lp8758_is_master(pchip,icnt)) {
                       bctrl_mode = pchip->pdata->buck_ctrl[icnt];
                       if(bctrl_mode > LP8758_CTRL_MAX-1)
                               return -EINVAL;
                       reg = &lp8758_buck_master_config[icnt][bctrl_mode];

                       /* control registser set */
                       if(bctrl_mode != LP8758_CTRL_MODE0)
                               bctrl_mode = (bctrl_mode - 1) | LP8758_CTRL_PIN_EN_MASK;

                       ret = regmap_update_bits(pchip->regmap, lp8758_get_ctrl1_address(icnt),
                                                                       LP8758_CTRL_BUCK_MASK,
                                                                       bctrl_mode << LP8758_CTRL_BUCK_SHIFT);
                       if(ret < 0){
                               dev_err(pchip->dev, "lp8758 %s i2c error \n",__func__);
                               return ret;
                       }
               } else {
                       reg = &lp8758_buck_slave_config[icnt],
                       dev_err(pchip->dev, "lp8758 %d regulator is SLAVE\n",icnt);
               }
               memcpy(&pchip->regulators[icnt], reg, sizeof(struct regulator_desc));
       }
       return 0;
}

static int lp8758_regulator_register(struct lp8758_chip *pchip)
{
       int ret, icnt;
       struct lp8758_platform_data *pdata = pchip->pdata;
       struct regulator_config rconfig = { };

       rconfig.regmap = pchip->regmap;
       rconfig.dev = pchip->dev;
       rconfig.driver_data = pchip;

       for(icnt = 0; icnt < LP8758_BUCK_MAX; icnt++){
               rconfig.init_data = pdata->buck_data[icnt];
               rconfig.of_node = pchip->dev->of_node;
               pchip->rdev[icnt] =
                   devm_regulator_register(pchip->dev,
                                   &pchip->regulators[icnt], &rconfig);
               if (IS_ERR(pchip->rdev[icnt])) {
                       ret = PTR_ERR(pchip->rdev[icnt]);
                       pchip->rdev[icnt] = NULL;
                       dev_err(pchip->dev,
                                               "regulator init failed: buck %d\n",     icnt);
                       return ret;
               }
       }
       return 0;
}

static irqreturn_t lp8758_irq_handler(int irq, void *data)
{
       int ret, icnt;
       unsigned int int_top, rdata;
       struct lp8758_chip *pchip = data;

       ret = regmap_read(pchip->regmap, LP8758_REG_INT_TOP, &int_top);
       if(int_top & LP8758_INT_TMEP_MASK)
               for(icnt = LP8758_BUCK0; icnt < LP8758_BUCK_MAX; icnt++)
                       regulator_notifier_call_chain(pchip->rdev[icnt],
                                                                       REGULATOR_EVENT_OVER_TEMP, NULL);

       if(int_top & LP8758_INT_BUCK01_MASK) {
               ret = regmap_read(pchip->regmap, LP8758_REG_INT_BUCK_01, &rdata);
               if(rdata & LP8758_INT_OVC_BUCK0_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK0],
                                                                       LP8758_EVENT_OCP, NULL);
               if(rdata & LP8758_INT_OVC_BUCK1_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK1],
                                                                       LP8758_EVENT_OCP, NULL);
               if(rdata & LP8758_INT_PWR_FAULT_BUCK0_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK0],
                                                                       LP8758_EVENT_PWR_FAULT, NULL);
               if(rdata & LP8758_INT_PWR_FAULT_BUCK1_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK1],
                                                                       LP8758_EVENT_PWR_FAULT, NULL);
       }

       if(int_top & LP8758_INT_BUCK23_MASK) {
               ret = regmap_read(pchip->regmap, LP8758_REG_INT_BUCK_23, &rdata);
               if(rdata & LP8758_INT_OVC_BUCK2_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK2],
                                                                       LP8758_EVENT_OCP, NULL);
               if(rdata & LP8758_INT_OVC_BUCK3_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK3],
                                                                       LP8758_EVENT_OCP, NULL);
               if(rdata & LP8758_INT_PWR_FAULT_BUCK2_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK2],
                                                                       LP8758_EVENT_PWR_FAULT, NULL);
               if(rdata & LP8758_INT_PWR_FAULT_BUCK3_MASK)
                       regulator_notifier_call_chain(pchip->rdev[LP8758_BUCK3],
                                                                       LP8758_EVENT_PWR_FAULT, NULL);
       }

       /* clear interrupt */
       regmap_write(pchip->regmap, LP8758_REG_INT_BUCK_01, LP8758_INT_CLEAR_BUCK);
       regmap_write(pchip->regmap, LP8758_REG_INT_BUCK_23, LP8758_INT_CLEAR_BUCK);
       regmap_write(pchip->regmap, LP8758_REG_INT_TOP, LP8758_INT_CLEAR_TOP);

       dev_err(pchip->dev, "lp8758 IRQ Handeled");
       return IRQ_HANDLED;
}

static int lp8758_intr_config(struct lp8758_chip *pchip)
{
       int ret, irq;

       if (pchip->pdata->irq == 0) {
               dev_warn(pchip->dev, "not use interrupt : %s\n", __func__);
               return 0;
       }

       /* initially clear interrupt */
       regmap_write(pchip->regmap, LP8758_REG_INT_BUCK_01, LP8758_INT_CLEAR_BUCK);
       regmap_write(pchip->regmap, LP8758_REG_INT_BUCK_23, LP8758_INT_CLEAR_BUCK);
       regmap_write(pchip->regmap, LP8758_REG_INT_TOP, LP8758_INT_CLEAR_TOP);

       gpio_request_one(pchip->pdata->irq, GPIOF_DIR_IN,"lp8758-irq");
       irq = gpio_to_irq(pchip->pdata->irq);
       if(irq < 0){
               dev_warn(pchip->dev, "irq can't be configurated\n");
               return -EINVAL;
       }

       ret = request_threaded_irq(irq, NULL, lp8758_irq_handler,
                                  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                  "lp8758-irq", pchip);
       return ret;
}

static struct of_regulator_match lp8758_matches[LP8758_BUCK_MAX] = {
       { .name = "buck0", .driver_data = (void *)LP8758_BUCK0, },
       { .name = "buck1", .driver_data = (void *)LP8758_BUCK1, },
       { .name = "buck2", .driver_data = (void *)LP8758_BUCK2, },
       { .name = "buck3", .driver_data = (void *)LP8758_BUCK3, },
};

static int lp8758_parse_dt(struct i2c_client *client,
                                                               struct lp8758_chip *pchip)
{
       struct device_node *node = client->dev.of_node;
       int err, icnt;

       pchip->pdata = devm_kzalloc(&client->dev,
                    sizeof(struct lp8758_platform_data), GFP_KERNEL);
       if(pchip->pdata == NULL){
               dev_err(&client->dev, "lp8758 -- platform data is null\n");
               return -ENOMEM;
       }

       err = of_regulator_match(&client->dev, node,
                                                               lp8758_matches, LP8758_BUCK_MAX);
       if (err <= 0){
               dev_err(&client->dev, "lp8758 --ERR - of regulator match\n");
               return -EINVAL;
       }

       for(icnt = 0; icnt < LP8758_BUCK_MAX; icnt++){
               pchip->pdata->buck_data[icnt] = lp8758_matches[icnt].init_data;
       }

       pchip->pdata->irq = of_get_named_gpio(node,"irq-gpio", 0);
       err = of_property_read_u32(node, "sub_version",
                                                               &pchip->pdata->sub_version);
       if(err < 0){
               dev_err(&client->dev, "lp8758 --ERR - of chip version read\n");
               return -EINVAL;
       }

       err = of_property_read_u32_array(node, "buck_ctrl",
                                                                       pchip->pdata->buck_ctrl, LP8758_BUCK_MAX);
       if(err < 0){
               dev_err(&client->dev, "lp8758 --ERR - pin ctrl data\n");
               return -EINVAL;
       }
       return 0;
}

static const struct regmap_config lp8758_regmap = {
       .reg_bits = 8,
       .val_bits = 8,
       .max_register = LP8758_REG_MAX,
};

static const struct of_device_id of_lp8758_bl_match[] = {
       { .compatible = "ti,lp8758", },
       {},
};

static int lp8758_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
       struct lp8758_chip *pchip;
       int ret = 0;

       if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
               dev_err(&client->dev, "i2c functionality check fail.\n");
               return -EOPNOTSUPP;
       }

       pchip = devm_kzalloc(&client->dev,
                            sizeof(struct lp8758_chip), GFP_KERNEL);
       if (!pchip)
               return -ENOMEM;

       pchip->regmap = devm_regmap_init_i2c(client, &lp8758_regmap);
       if (IS_ERR(pchip->regmap)) {
               ret = PTR_ERR(pchip->regmap);
               dev_err(&client->dev, "fail : allocate i2c register map\n");
               return ret;
       }

       ret = lp8758_parse_dt(client, pchip);
       if(ret < 0)
               return ret;
       pchip->dev = &client->dev;
       i2c_set_clientdata(client, pchip);

       ret = lp8758_regulator_init(pchip);
       if (ret < 0) {
               dev_err(&client->dev, "fail to initialize regulators\n");
               return ret;
       }

       ret = lp8758_regulator_register(pchip);
       if (ret < 0) {
               dev_err(&client->dev, "fail to register regulators\n");
               return ret;
       }

       ret = lp8758_intr_config(pchip);
       if (ret < 0) {
               dev_err(&client->dev, "fail to irq config\n");
               return ret;
       }

       dev_info(&client->dev, "lp8758 probe done\n");
       return 0;
}

static const struct i2c_device_id lp8758_id[] = {
       {LP8758_NAME, 0},
       {}
};

MODULE_DEVICE_TABLE(i2c, lp8758_id);
static struct i2c_driver lp8758_i2c_driver = {
       .driver = {
               .name = LP8758_NAME,
               .owner   = THIS_MODULE,
               .of_match_table = of_match_ptr(of_lp8758_bl_match),
       },
       .probe = lp8758_probe,
       .id_table = lp8758_id,
};

module_i2c_driver(lp8758_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments lp8758 driver");
MODULE_AUTHOR("Daniel Jeong <daniel.jeong@ti.com>");
MODULE_AUTHOR("Ldd Mlp <ldd-mlp@list.ti.com>");
MODULE_LICENSE("GPL v2");
