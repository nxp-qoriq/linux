/*
 * Simple driver for Texas Instruments lp8758 Regulator chip
 * Copyright (C) 2015 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __LINUX_LP8758_H
#define __LINUX_LP8758_H

#include <linux/regulator/consumer.h>

#define LP8758_NAME "lp8758"
#define LP8758_ADDR 0x60

enum lp8758_otp_id {
       LP8758_B0 = 0xb0,
       LP8758_D0 = 0xd0,
       LP8758_E0 = 0xe0,
       LP8758_F0 = 0xf0
};

enum lp8758_sub_version {
       LP8758_SUB_VER_B0 = 0,
       LP8758_SUB_VER_D0,
       LP8758_SUB_VER_E0,
       LP8758_SUB_VER_F0,
       LP8758_SUB_VER_MAX
};

enum lp8758_bucks {
       LP8758_BUCK0 = 0,
       LP8758_BUCK1,
       LP8758_BUCK2,
       LP8758_BUCK3,
       LP8758_BUCK_MAX
};

/*
 * MODE0 : Pin  Ctrl disable
 * MODE1 : Pin1 Ctrl + Enable/Disable
 * MODE2 : Pin1 Ctrl + Roof/Floor
 * MODE3 : Pin2 Ctrl + Enable/Diasble
 * MODE4 : Pin2 Ctrl + Roof/Floor
 */
enum lp8758_ctrl_mode {
       LP8758_CTRL_MODE0 = 0,
       LP8758_CTRL_MODE1,
       LP8758_CTRL_MODE2,
       LP8758_CTRL_MODE3,
       LP8758_CTRL_MODE4,
       LP8758_CTRL_MAX
};

enum lp8758_buck_mode {
       LP8758_BUCK_MASTER = 0,
       LP8758_BUCK_SLAVE
};

enum lp8758_op_mode {
       LP8758_BUCK_OP_AUTO = 0x00,
       LP8758_BUCK_OP_FPWM = 0x02,
};

enum lp8758_registers {
       LP8758_REG_DEV_REV = 0x00,
       LP8758_REG_OTP_REV = 0x01,
       LP8758_REG_BUCK0_CTRL1 = 0x02,
       LP8758_REG_BUCK0_CTRL2 = 0x03,
       LP8758_REG_BUCK1_CTRL1 = 0x04,
       LP8758_REG_BUCK1_CTRL2 = 0x05,
       LP8758_REG_BUCK2_CTRL1 = 0x06,
       LP8758_REG_BUCK2_CTRL2 = 0x07,
       LP8758_REG_BUCK3_CTRL1 = 0x08,
       LP8758_REG_BUCK3_CTRL2 = 0x09,
       LP8758_REG_BUCK0_VOUT           = 0x0a,
       LP8758_REG_BUCK0_FLOORVOUT      = 0x0b,
       LP8758_REG_BUCK1_VOUT           = 0x0c,
       LP8758_REG_BUCK1_FLOORVOUT      = 0x0d,
       LP8758_REG_BUCK2_VOUT           = 0x0e,
       LP8758_REG_BUCK2_FLOORVOUT      = 0x0f,
       LP8758_REG_BUCK3_VOUT           = 0x10,
       LP8758_REG_BUCK3_FLOORVOUT      = 0x11,
       LP8758_REG_BUCK0_DELAY = 0x12,
       LP8758_REG_BUCK1_DELAY = 0x13,
       LP8758_REG_BUCK2_DELAY = 0x14,
       LP8758_REG_BUCK3_DELAY = 0x15,
       LP8758_REG_RESET = 0x16,
       LP8758_REG_CONFIG = 0x17,
       LP8758_REG_INT_TOP = 0x18,
       LP8758_REG_INT_BUCK_01 = 0x19,
       LP8758_REG_INT_BUCK_23 = 0x1a,
       LP8758_REG_STAT_TOP = 0x1b,
       LP8758_REG_STAT_BUCK_01 = 0x1c,
       LP8758_REG_STAT_BUCK_23 = 0x1d,
       LP8758_REG_MASK_TOP = 0x1e,
       LP8758_REG_MASK_BUCK_01 = 0x1f,
       LP8758_REG_MASK_BUCK_23 = 0x20,
       LP8758_REG_SEL_I_LOAD = 0x21,
       LP8758_REG_SEL_I_LOAD_2 = 0x22,
       LP8758_REG_SEL_I_LOAD_1 = 0x23,
       LP8758_REG_MAX = 0xff
};

/*
 * PWR FAULT : power fault detected
 * OCP : over current protect activated
 * OVP : over voltage protect activated
 * TEMP_WARN : thermal warning
 * TEMP_SHDN : thermal shutdonw detected
 * I_LOAD : current measured
 */
#define LP8758_EVENT_PWR_FAULT REGULATOR_EVENT_FAIL
#define LP8758_EVENT_OCP REGULATOR_EVENT_OVER_CURRENT
#define LP8758_EVENT_OVP 0x10000
#define LP8758_EVENT_TEMP_WARN 0x2000
#define LP8758_EVENT_TEMP_SHDN REGULATOR_EVENT_OVER_TEMP
#define LP8758_EVENT_I_LOAD    0x40000

#define LP8758_INT_BUCK01_MASK 0x30
#define LP8758_INT_BUCK23_MASK 0xc0
#define LP8758_INT_TEMP_SHDN_MASK 0x08
#define LP8758_INT_TEMP_WARN_MASK 0x04

/* Over Current interrupt mask */
#define LP8758_INT_OVC_BUCK0_MASK 0x01
#define LP8758_INT_OVC_BUCK1_MASK 0x10
#define LP8758_INT_OVC_BUCK2_MASK 0x01
#define LP8758_INT_OVC_BUCK3_MASK 0x10
/* Short Circuit interrupt mask */
#define LP8758_INT_SC_BUCK0_MASK 0x02
#define LP8758_INT_SC_BUCK1_MASK 0x20
#define LP8758_INT_SC_BUCK2_MASK 0x02
#define LP8758_INT_SC_BUCK3_MASK 0x20
/* Power Ground Reach interrupt mask */
#define LP8758_INT_PG_BUCK0_MASK 0x04
#define LP8758_INT_PG_BUCK1_MASK 0x40
#define LP8758_INT_PG_BUCK2_MASK 0x04
#define LP8758_INT_PG_BUCK3_MASK 0x40

#define LP8758_INT_TMEP_MASK\
       (LP8758_INT_TEMP_WARN_MASK | LP8758_INT_TEMP_SHDN_MASK)
#define LP8758_INT_PWR_FAULT_BUCK0_MASK\
       (LP8758_INT_SC_BUCK0_MASK | LP8758_INT_SC_BUCK0_MASK)
#define LP8758_INT_PWR_FAULT_BUCK1_MASK\
       (LP8758_INT_SC_BUCK1_MASK | LP8758_INT_SC_BUCK1_MASK)
#define LP8758_INT_PWR_FAULT_BUCK2_MASK\
       (LP8758_INT_SC_BUCK2_MASK | LP8758_INT_SC_BUCK2_MASK)
#define LP8758_INT_PWR_FAULT_BUCK3_MASK\
       (LP8758_INT_SC_BUCK3_MASK | LP8758_INT_SC_BUCK3_MASK)

#define LP8758_INT_CLEAR_TOP 0x0f
#define LP8758_INT_CLEAR_BUCK 0x77

#define LP8758_ILIM_MASK 0x38
#define LP8758_ILIM_SHIFT 3

#define LP8758_BUCK_EN_MASK 0x80
#define LP8758_BUCK_VOUT_MASK 0xff

#define LP8758_CTRL_BUCK_MASK 0x70
#define LP8758_CTRL_BUCK_SHIFT 4
#define LP8758_CTRL_PIN_EN_MASK 0x04

#define LP8758_BUCK_RAMP_MASK 0x07
#define LP8758_BUCK_OP_MODE_MASK 0x04

/*
 * struct lp8758 platform data
 * @irq         : irq number
 * @sub_version : otp version 0-b0, 1-d0, 2-e0, 3-f0
 * @buck_ctrl[] : [0] buck0 buck control config.
 *                [1] buck1 buck control config.
 *                [2] buck2 buck control config.
 *                [3] buck3 buck control config.
 *                control config
 *                  : 0 - EN_BUCK bit only
 *                  : 1 - EN_BUCK bit & EN_PIN1 ENABLE/DISABLE
 *                  : 2 - EN_BUCK bit & EN_PIN1 ROOF/FLOOR
 *                  : 3 - EN_BUCK bit & EN_PIN2 ENABLE/DISABLE
 *                  : 4 - EN_BUCK bit & EN_PIN2 ROOF/FLOOR
 * @buck_data  : init buck data
 */
struct lp8758_platform_data {

       int irq;
       u32 sub_version;
       u32 buck_ctrl[LP8758_BUCK_MAX];
       struct regulator_init_data *buck_data[LP8758_BUCK_MAX];
};
#endif /* __LINUX_LP8758_H */
