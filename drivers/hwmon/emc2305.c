// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 *
 * Reinhard Pfau <pfau@gdsys.de>
 * Biwen Li <biwen.li@nxp.com>
 *
 * Datasheet available at:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/2305.pdf
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/thermal.h>

/*
 * Addresses scanned.
 * Listed in the same order as they appear in the EMC2305, EMC2303 data sheets.
 *
 * Note: these are the I2C addresses which are possible for EMC2305 and EMC2303
 * chips.
 * The EMC2302 supports only 0x2e (EMC2302-1) and 0x2f (EMC2302-2).
 * The EMC2301 supports only 0x2f.
 */
static const unsigned short i2c_addresses[] = {
	0x2E,
	0x2F,
	0x2C,
	0x2D,
	0x4C,
	0x4D,
	I2C_CLIENT_END
};

/*
 * global registers
 */
enum {
	REG_CFG = 0x20,
	REG_FAN_STATUS = 0x24,
	REG_FAN_STALL_STATUS = 0x25,
	REG_FAN_SPIN_STATUS = 0x26,
	REG_DRIVE_FAIL_STATUS = 0x27,
	REG_FAN_INTERRUPT_ENABLE = 0x29,
	REG_PWM_POLARITY_CONFIG = 0x2a,
	REG_PWM_OUTPUT_CONFIG = 0x2b,
	REG_PWM_BASE_FREQ_1 = 0x2c,
	REG_PWM_BASE_FREQ_2 = 0x2d,
	REG_SOFTWARE_LOCK = 0xef,
	REG_PRODUCT_FEATURES = 0xfc,
	REG_PRODUCT_ID = 0xfd,
	REG_MANUFACTURER_ID = 0xfe,
	REG_REVISION = 0xff
};

/*
 * fan specific registers
 */
enum {
	REG_FAN_SETTING = 0x30,
	REG_PWM_DIVIDE = 0x31,
	REG_FAN_CFG_1 = 0x32,
	REG_FAN_CFG_2 = 0x33,
	REG_GAIN = 0x35,
	REG_FAN_SPIN_UP_CONFIG = 0x36,
	REG_FAN_MAX_STEP = 0x37,
	REG_FAN_MINIMUM_DRIVE = 0x38,
	REG_FAN_VALID_TACH_COUNT = 0x39,
	REG_FAN_DRIVE_FAIL_BAND_LOW = 0x3a,
	REG_FAN_DRIVE_FAIL_BAND_HIGH = 0x3b,
	REG_TACH_TARGET_LOW = 0x3c,
	REG_TACH_TARGET_HIGH = 0x3d,
	REG_TACH_READ_HIGH = 0x3e,
	REG_TACH_READ_LOW = 0x3f,
};

#define SEL_FAN(fan, reg)			(reg + fan * 0x10)

#define EMC2305_MAX_TACH_TARGET			(0x1fff)
#define EMC2305_MIN_TACH_TARGET			(0x0)

#define EMC2305_TACH_READING_HIGH_BYTE_MASK	(0x1fe0)
#define EMC2305_TACH_TARGET_HIGH_BYTE_MASK	(0x1fe0)

#define EMC2305_BASE_FREQ_26KHZ			(0)
#define EMC2305_BASE_FREQ_19531HZ		(1)
#define EMC2305_BASE_FREQ_4882HZ		(2)
#define EMC2305_BASE_FREQ_2441HZ		(3)

#define MIN_RPM					(0)
#define MAX_RPM					(16000)

/*
 * Factor by equations [2] and [3] from data sheet; valid for fans where the
 * number of edges equals (poles * 2 + 1).
 */
#define FAN_RPM_FACTOR				(3932160)

#define MAX_CHN_NUM				(5)

#define MAX_PWM_DIV				(0xff)
#define MIN_PWM_DIV				(0x1)

#define MIN_PWM					(0x0)
#define MAX_PWM					(0xff)

#define FREQ_26KHZ				(26000)
#define FREQ_19531HZ				(19531)
#define FREQ_4882HZ				(4882)
#define FREQ_2441HZ				(2441)

#define MAX_COOLING_DEV_NAME_LEN 16

union fan_cfg1_union {
	u8	fan_cfg1;
	struct {
		u8	update:3; /* B2~B0 */
		u8	edges:2; /* B4~B3 */
		u8	range:2; /* B6~B5 */
		u8	en_algo:1; /* B7 */
	} bits;
};

union pwm_base_freq_1_union {
	u8	pwm_base_freq_1;
	struct {
		u8	pwm_base4:2;
		u8	pwm_base5:2;
		u8	reserved:4;
	} bits;
};

union pwm_base_freq_2_union {
	u8	pwm_base_freq_2;
	struct {
		u8	pwm_base1:2;
		u8	pwm_base2:2;
		u8	pwm_base3:2;
		u8	reserved:2;
	} bits;
};

struct emc2305_cooling_device {
	char name[MAX_COOLING_DEV_NAME_LEN];
	struct emc2305_data *priv;
	struct thermal_cooling_device *tcdev;
	int channel;
	unsigned int *cooling_levels;
	u8 max_state;
	u8 cur_state;
};

struct emc2305_fan_data {
	union fan_cfg1_union		fan_cfg1_un;
	u8				fan_setting; /* direct setting mode */
	u8				pwm_base;
	u8				pwm_div;
	u16				tach;
	u16				target;
	long				multiplier;
	long				pwm_freq;
	struct emc2305_cooling_device	cdev;
};

struct emc2305_data {
	struct i2c_client		*client;
	struct mutex			update_lock;
	bool				valid;
	int				channel_count;
	struct emc2305_fan_data		fan[MAX_CHN_NUM];
	unsigned long			last_updated;	/* in jiffies */
	union pwm_base_freq_1_union	pwm_base_freq_1_un;
	union pwm_base_freq_2_union	pwm_base_freq_2_un;
};

static int emc2305_read_reg(struct i2c_client *client, u8 i2c_reg, u8 *output)
{
	int status = i2c_smbus_read_byte_data(client, i2c_reg);

	if (status < 0) {
		dev_err(&client->dev, "reg 0x%02x, err %d\n",
			 i2c_reg, status);
	} else {
		*output = status;
	}
	return status;
}

static void emc2305_write_reg(struct i2c_client *client, u8 i2c_reg, u8 val)
{
	i2c_smbus_write_byte_data(client, i2c_reg, val);
}

static void emc2305_read_u16_reg(struct i2c_client *client, u16 *output,
			      u8 hi_addr, u8 lo_addr)
{
	u8 high_byte, lo_byte;

	if (emc2305_read_reg(client, hi_addr, &high_byte) < 0)
		return;

	if (emc2305_read_reg(client, lo_addr, &lo_byte) < 0)
		return;

	*output = ((u16)high_byte << 5) | (lo_byte >> 3);
}

static void emc2305_write_u16_reg(struct i2c_client *client,
				    u8 hi_addr, u8 lo_addr,
				    u16 val)
{
	u8 high_byte = (val & 0x1fe0) >> 5;
	u8 low_byte = (val & 0x001f) << 3;

	emc2305_write_reg(client, lo_addr, low_byte);
	emc2305_write_reg(client, hi_addr, high_byte);
}

static struct emc2305_data *
emc2305_update_device(struct device *dev)
{
	struct emc2305_data *data = dev_get_drvdata(dev);
	struct emc2305_fan_data *fan_data;
	struct i2c_client *client = data->client;
	int channel;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		emc2305_read_reg(client,
				 REG_PWM_BASE_FREQ_1,
				 &data->pwm_base_freq_1_un.pwm_base_freq_1);
		emc2305_read_reg(client,
				 REG_PWM_BASE_FREQ_2,
				 &data->pwm_base_freq_2_un.pwm_base_freq_2);

		for (channel = 0; channel < data->channel_count; channel++) {
			fan_data = &data->fan[channel];
			emc2305_read_reg(client,
					 SEL_FAN(channel, REG_FAN_CFG_1),
					 &fan_data->fan_cfg1_un.fan_cfg1);
			emc2305_read_u16_reg(client, &fan_data->target,
					     SEL_FAN(channel, REG_TACH_TARGET_HIGH),
					     SEL_FAN(channel, REG_TACH_TARGET_LOW));
			emc2305_read_u16_reg(client, &fan_data->tach,
					     SEL_FAN(channel, REG_TACH_READ_HIGH),
					     SEL_FAN(channel, REG_TACH_READ_LOW));

			fan_data->multiplier = 1 << fan_data->fan_cfg1_un.bits.range;

			if (channel == 0)
				fan_data->pwm_base = data->pwm_base_freq_2_un.bits.pwm_base1;
			if (channel == 1)
				fan_data->pwm_base = data->pwm_base_freq_2_un.bits.pwm_base2;
			if (channel == 2)
				fan_data->pwm_base = data->pwm_base_freq_2_un.bits.pwm_base3;
			if (channel == 3)
				fan_data->pwm_base = data->pwm_base_freq_1_un.bits.pwm_base4;
			if (channel == 4)
				fan_data->pwm_base = data->pwm_base_freq_1_un.bits.pwm_base5;

			emc2305_read_reg(client,
					 SEL_FAN(channel, REG_PWM_DIVIDE),
					 &fan_data->pwm_div);

			switch (fan_data->pwm_base) {
			case EMC2305_BASE_FREQ_26KHZ:
				fan_data->pwm_freq = FREQ_26KHZ / fan_data->pwm_div;
				break;
			case EMC2305_BASE_FREQ_19531HZ:
				fan_data->pwm_freq = FREQ_19531HZ / fan_data->pwm_div;
				break;
			case EMC2305_BASE_FREQ_4882HZ:
				fan_data->pwm_freq = FREQ_4882HZ / fan_data->pwm_div;
				break;
			case EMC2305_BASE_FREQ_2441HZ:
				fan_data->pwm_freq = FREQ_2441HZ / fan_data->pwm_div;
				break;
			}

			emc2305_read_reg(client,
					 SEL_FAN(channel, REG_FAN_SETTING),
					 &fan_data->fan_setting);
		}

		data->last_updated = jiffies;
		data->valid = true;
	}

	mutex_unlock(&data->update_lock);
	return data;
}

static umode_t emc2305_fan_is_visible(const void *_data, u32 attr, int channel)
{
	const struct emc2305_data *data = _data;

	switch (attr) {
	case hwmon_fan_input:
	case hwmon_fan_fault:
		if (channel < data->channel_count)
			return 0444;
		return 0;
	case hwmon_fan_target:
	case hwmon_fan_div:
		if (channel < data->channel_count)
			return 0644;
		return 0;
	default:
		return 0;
	}
}

static umode_t emc2305_pwm_is_visible(const void *_data, u32 attr, int channel)
{
	const struct emc2305_data *data = _data;

	switch (attr) {
	case hwmon_pwm_freq:
	case hwmon_pwm_input:
		if (channel < data->channel_count)
			return 0644;
		return 0;
	case hwmon_pwm_enable:
		if (channel < data->channel_count)
			return 0644;
		return 0;
	default:
		return 0;
	}
}

static umode_t emc2305_is_visible(const void *data,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		return emc2305_fan_is_visible(data, attr, channel);
	case hwmon_pwm:
		return emc2305_pwm_is_visible(data, attr, channel);
	default:
		return 0;
	}
}

static int emc2305_read_fan(struct device *dev, u32 attr,
			    int channel, long *val)
{
	struct emc2305_data *data = emc2305_update_device(dev);
	struct emc2305_fan_data *fan_data;
	int rpm = 0;

	if (IS_ERR(data))
		return PTR_ERR(data);

	fan_data = &data->fan[channel];

	switch (attr) {
	case hwmon_fan_div:
		*val = fan_data->multiplier;
		return 0;
	case hwmon_fan_input:
		if ((fan_data->tach != 0)
		    && ((fan_data->tach & EMC2305_TACH_READING_HIGH_BYTE_MASK)
			    != EMC2305_TACH_READING_HIGH_BYTE_MASK)) {
			rpm = (FAN_RPM_FACTOR * fan_data->multiplier)
			       / fan_data->tach;
		}
		*val = rpm;
		return 0;
	case hwmon_fan_target:
		/* high byte of 0xff indicates disabled */
		if ((fan_data->target != 0)
		    && ((fan_data->target & EMC2305_TACH_TARGET_HIGH_BYTE_MASK)
			    != EMC2305_TACH_TARGET_HIGH_BYTE_MASK)) {
			rpm = FAN_RPM_FACTOR * fan_data->multiplier
			      / fan_data->target;
		}
		*val = rpm;
		return 0;
	case hwmon_fan_fault:
		*val = !!((fan_data->tach & EMC2305_TACH_READING_HIGH_BYTE_MASK)
				== EMC2305_TACH_READING_HIGH_BYTE_MASK);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int emc2305_write_fan(struct device *dev, u32 attr,
			     int channel, long val)
{
	struct emc2305_data *data = dev_get_drvdata(dev);
	struct emc2305_fan_data *fan_data = &data->fan[channel];
	struct i2c_client *client = data->client;
	u8 old_div = fan_data->multiplier;
	int err = 0;

	mutex_lock(&data->update_lock);

	switch (attr) {
	case hwmon_fan_div:
		if (old_div == val)
			break;

		switch (val) {
		case 1:
			fan_data->fan_cfg1_un.bits.range = 0;
			break;
		case 2:
			fan_data->fan_cfg1_un.bits.range = 1;
			break;
		case 4:
			fan_data->fan_cfg1_un.bits.range = 2;
			break;
		case 8:
			fan_data->fan_cfg1_un.bits.range = 3;
			break;
		default:
			dev_err(&client->dev, "Pls input fan div: 1/2/4/8\n");
			err = -EINVAL;
			break;
		}

		if (err)
			break;

		fan_data->multiplier = val;
		if (!fan_data->fan_cfg1_un.bits.en_algo) {
			/* switch to fan speed control mode */
			fan_data->fan_cfg1_un.bits.en_algo = true;
			emc2305_write_reg(client,
					  SEL_FAN(channel, REG_FAN_CFG_1),
					  fan_data->fan_cfg1_un.fan_cfg1);
		}

		emc2305_write_reg(client,
				  SEL_FAN(channel, REG_FAN_CFG_1),
				  fan_data->fan_cfg1_un.fan_cfg1);

		/* Update tach target */
		fan_data->target = clamp_val(fan_data->target * fan_data->multiplier / old_div,
					     EMC2305_MIN_TACH_TARGET, EMC2305_MAX_TACH_TARGET);
		emc2305_write_u16_reg(client,
				      SEL_FAN(channel, REG_TACH_TARGET_HIGH),
				      SEL_FAN(channel, REG_TACH_TARGET_LOW),
				      fan_data->target);
		break;

	case hwmon_fan_target:
		/*
		 * Datasheet states 16000 as maximum RPM target
		 * (table 2.2 and section 4.3)
		 */
		if ((val < MIN_RPM) || (val > MAX_RPM)) {
			err =  -EINVAL;
			break;
		}
		if (val == 0)
			fan_data->target = EMC2305_MAX_TACH_TARGET;
		else
			fan_data->target = clamp_val(FAN_RPM_FACTOR * fan_data->multiplier / val,
						     EMC2305_MIN_TACH_TARGET, EMC2305_MAX_TACH_TARGET);

		if (!fan_data->fan_cfg1_un.bits.en_algo) {
			/* switch to fan speed control mode */
			fan_data->fan_cfg1_un.bits.en_algo = true;
			emc2305_write_reg(client,
					  SEL_FAN(channel, REG_FAN_CFG_1),
					  fan_data->fan_cfg1_un.fan_cfg1);
		}

		emc2305_write_u16_reg(client,
				      SEL_FAN(channel, REG_TACH_TARGET_HIGH),
				      SEL_FAN(channel, REG_TACH_TARGET_LOW),
				      fan_data->target);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&data->update_lock);

	return err;
}

static int emc2305_read_pwm(struct device *dev, u32 attr, int channel,
			     long *val)
{
	struct emc2305_data *data = emc2305_update_device(dev);
	struct emc2305_fan_data *fan_data = &data->fan[channel];

	switch (attr) {
	case hwmon_pwm_freq:
		*val = fan_data->pwm_freq;
		return 0;
	case hwmon_pwm_input:
		*val = fan_data->fan_setting;
		return 0;
	case hwmon_pwm_enable:
		if (!(fan_data->fan_cfg1_un.bits.en_algo))
			*val = 0;
		else
			*val = 3;
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int emc2305_write_pwm(struct device *dev, u32 attr, int channel,
			      long val)
{
	struct emc2305_data *data = dev_get_drvdata(dev);
	struct emc2305_fan_data *fan_data = &data->fan[channel];
	struct i2c_client *client = data->client;
	int err = 0;
	u8 pwm_div = 0xff;
	long long base_freq;

	mutex_lock(&data->update_lock);

	switch (attr) {
	case hwmon_pwm_freq:
		if (fan_data->pwm_freq == val) /* No change */
			break;

		base_freq = val * pwm_div;
		/* Select a proper pwm divide */
		while (base_freq < FREQ_2441HZ
		       || base_freq > FREQ_26KHZ) {
			pwm_div--;
			if (pwm_div == 0) {
				pwm_div = 1;
				break;
			}
			base_freq = val * pwm_div;

		}

		/* Select a proper pwm base freq */
		if (base_freq > FREQ_26KHZ / MIN_PWM_DIV
		    || base_freq < FREQ_2441HZ / MAX_PWM_DIV + 1) {
			dev_err(&client->dev, "Supported pwm freq range: %d ~ %d\n",
				FREQ_2441HZ / MAX_PWM_DIV + 1, FREQ_26KHZ / MIN_PWM_DIV);
			err = -EINVAL;
			break;
		}

		if (base_freq <= FREQ_2441HZ)
			fan_data->pwm_base = 3;
		else if (base_freq <= FREQ_4882HZ)
			fan_data->pwm_base = 2;
		else if (base_freq <= FREQ_19531HZ)
			fan_data->pwm_base = 1;
		else if (base_freq <= FREQ_26KHZ)
			fan_data->pwm_base = 0;

		fan_data->pwm_freq = val;

		if (fan_data->fan_cfg1_un.bits.en_algo) {
			/* switch to direct setting mode */
			fan_data->fan_cfg1_un.bits.en_algo = false;
			emc2305_write_reg(client,
					  SEL_FAN(channel, REG_FAN_CFG_1),
					  fan_data->fan_cfg1_un.fan_cfg1);
		}

		/* Setting pwm base freq */
		if (channel >= 0 && channel < 3) {
			if (channel == 2)
				data->pwm_base_freq_2_un.bits.pwm_base3 = fan_data->pwm_base;
			if (channel == 1)
				data->pwm_base_freq_2_un.bits.pwm_base2 = fan_data->pwm_base;
			if (channel == 0)
				data->pwm_base_freq_2_un.bits.pwm_base1 = fan_data->pwm_base;
			emc2305_write_reg(client,
					  REG_PWM_BASE_FREQ_2,
					  data->pwm_base_freq_2_un.pwm_base_freq_2);
		} else if (channel == 3 || channel == 4) {
			if (channel == 4)
				data->pwm_base_freq_1_un.bits.pwm_base5 = fan_data->pwm_base;
			if (channel == 3)
				data->pwm_base_freq_1_un.bits.pwm_base4 = fan_data->pwm_base;

			emc2305_write_reg(client,
					  REG_PWM_BASE_FREQ_1,
					  data->pwm_base_freq_1_un.pwm_base_freq_1);
		}

		/* Setting pwm divide */
		fan_data->pwm_div = pwm_div;
		emc2305_write_reg(client,
				  SEL_FAN(channel, REG_PWM_DIVIDE),
				  fan_data->pwm_div);
		break;
	case hwmon_pwm_input:
		if (val < 0 || val > 255) {
			err = -EINVAL;
			break;
		}
		fan_data->fan_setting = val;
		if (fan_data->fan_cfg1_un.bits.en_algo) {
			/* switch to direct setting mode */
			fan_data->fan_cfg1_un.bits.en_algo = false;
			emc2305_write_reg(client,
					  SEL_FAN(channel, REG_FAN_CFG_1),
					  fan_data->fan_cfg1_un.fan_cfg1);
		}

		emc2305_write_reg(client,
				  SEL_FAN(channel, REG_FAN_SETTING),
				  fan_data->fan_setting);
		break;

	case hwmon_pwm_enable:
		if (val == 0) {
			fan_data->fan_cfg1_un.bits.en_algo = false;
		} else if (val == 3) {
			fan_data->fan_cfg1_un.bits.en_algo = true;
		} else {
			err = -EINVAL;
			break;
		}
		emc2305_write_reg(client,
				  SEL_FAN(channel, REG_FAN_CFG_1),
				  fan_data->fan_cfg1_un.fan_cfg1);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	mutex_unlock(&data->update_lock);

	return err;
}

static int emc2305_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_fan:
		return emc2305_read_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return emc2305_read_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int emc2305_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_fan:
		return emc2305_write_fan(dev, attr, channel, val);
	case hwmon_pwm:
		return emc2305_write_pwm(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct hwmon_channel_info *emc2305_info[] = {
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_TARGET | HWMON_F_FAULT | HWMON_F_DIV,
			   HWMON_F_INPUT | HWMON_F_TARGET | HWMON_F_FAULT | HWMON_F_DIV,
			   HWMON_F_INPUT | HWMON_F_TARGET | HWMON_F_FAULT | HWMON_F_DIV,
			   HWMON_F_INPUT | HWMON_F_TARGET | HWMON_F_FAULT | HWMON_F_DIV,
			   HWMON_F_INPUT | HWMON_F_TARGET | HWMON_F_FAULT | HWMON_F_DIV),
	HWMON_CHANNEL_INFO(pwm,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ,
			   HWMON_PWM_INPUT | HWMON_PWM_ENABLE | HWMON_PWM_FREQ),
	NULL
};


static const struct hwmon_ops emc2305_hwmon_ops = {
	.is_visible = emc2305_is_visible,
	.read = emc2305_read,
	.write = emc2305_write,
};

static const struct hwmon_chip_info emc2305_chip_info = {
	.ops = &emc2305_hwmon_ops,
	.info = emc2305_info,
};

static int emc2305_init_client(struct device *dev,
				struct emc2305_data *data)
{
	struct i2c_client *client = data->client;
	u8 byte;
	int status = 0;

	status = emc2305_read_reg(client, REG_PRODUCT_ID, &byte);
	if (status < 0)
		return status;

	switch (byte) {
	case 0x34: /* EMC2305 */
		data->channel_count = 5;
		break;
	case 0x35: /* EMC2303 */
		data->channel_count = 3;
		break;
	case 0x36: /* EMC2302 */
		data->channel_count = 2;
		break;
	case 0x37: /* EMC2301 */
		data->channel_count = 1;
		break;
	default:
		dev_err(dev, "Unknown device, status = %d\n", status);
		return status;
	}

	emc2305_update_device(dev);

	return 0;
}

static int emc2305_get_max_state(struct thermal_cooling_device *tcdev,
				 unsigned long *state)
{
	struct emc2305_cooling_device *cdev = tcdev->devdata;

	*state = cdev->max_state;

	return 0;
}

static int emc2305_get_cur_state(struct thermal_cooling_device *tcdev,
				 unsigned long *state)
{
	struct emc2305_cooling_device *cdev = tcdev->devdata;

	*state = cdev->cur_state;

	return 0;
}

static int emc2305_set_cur_state(struct thermal_cooling_device *tcdev,
				 unsigned long state)
{
	struct emc2305_cooling_device *cdev = tcdev->devdata;
	struct emc2305_data *data = cdev->priv;
	struct i2c_client *client = data->client;
	int channel = cdev->channel;
	int err = 0;
	struct device *dev = &client->dev;

	if (state > cdev->max_state)
		return -EINVAL;

	if (state == cdev->cur_state)
		return 0;

	err = emc2305_write(dev, hwmon_pwm,
		      hwmon_pwm_input, channel,
		      cdev->cooling_levels[state]);
	if (err) {
		dev_err(dev, "Cannot set pwm!\n");
		return err;
	}

	cdev->cur_state = state;

	return err;
}

static const struct thermal_cooling_device_ops emc2305_cooling_ops = {
	.get_max_state = emc2305_get_max_state,
	.get_cur_state = emc2305_get_cur_state,
	.set_cur_state = emc2305_set_cur_state,
};

static int
emc2305_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct emc2305_data *data;
	struct device *hwmon_dev;
	int status;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	data = devm_kzalloc(&client->dev,
			    sizeof(struct emc2305_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->update_lock);

	status = emc2305_init_client(&client->dev, data);
	if (status)
		return status;

	hwmon_dev = devm_hwmon_device_register_with_info(&client->dev,
							 client->name, data,
							 &emc2305_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR_OR_ZERO(hwmon_dev);

	if (IS_ENABLED(CONFIG_THERMAL)) {
		int channel;
		int num_levels;
		struct device_node *child;
		int ret = 0;
		struct device *dev = &client->dev;
		struct device_node *np = dev->of_node;
		for_each_child_of_node(np, child) {
			ret = of_property_read_u32(child, "reg", &channel);
			if (ret)
				return ret;

			num_levels = of_property_count_u32_elems(child, "cooling-levels");
			if (num_levels > 0) {
				data->fan[channel].cdev.cooling_levels = devm_kzalloc(dev, num_levels * sizeof(u32), GFP_KERNEL);
				if (!data->fan[channel].cdev.cooling_levels)
					return -ENOMEM;

				data->fan[channel].cdev.max_state = num_levels - 1;

				ret = of_property_read_u32_array(child, "cooling-levels",
								data->fan[channel].cdev.cooling_levels,
								num_levels);
				if (ret) {
					dev_err(dev, "Property 'cooling-levels' cannot be read.\n");
					return ret;
				}

				snprintf(data->fan[channel].cdev.name, MAX_COOLING_DEV_NAME_LEN, "%pOFn%d", child, channel);
				data->fan[channel].cdev.tcdev = devm_thermal_of_cooling_device_register(dev,
								child, data->fan[channel].cdev.name,
								&data->fan[channel].cdev, &emc2305_cooling_ops);
				if (IS_ERR(data->fan[channel].cdev.tcdev)) {
					dev_warn(dev, "thermal cooling device register failed: %ld\n",
						 PTR_ERR(data->fan[channel].cdev.tcdev));
					of_node_put(child);
					return PTR_ERR(data->fan[channel].cdev.tcdev);
				}
				data->fan[channel].cdev.priv = data;
				data->fan[channel].cdev.channel = channel;
			}
		}
	}

	dev_info(&client->dev, "pwm fan controller: '%s'\n",
		 client->name);

	return 0;
}

static const struct i2c_device_id emc2305_ids[] = {
	{ "emc2301", 0 },
	{ "emc2302", 0 },
	{ "emc2303", 0 },
	{ "emc2305", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, emc2305_ids);

/* Return 0 if detection is successful, -ENODEV otherwise */
static int
emc2305_detect(struct i2c_client *new_client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;
	u8 manufacturer, product;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	if (emc2305_read_reg(new_client, REG_MANUFACTURER_ID, &manufacturer) != 0x5D)
		return -ENODEV;

	product = emc2305_read_reg(new_client, REG_PRODUCT_ID, &product);
	switch (product) {
	case 0x34:
		strlcpy(info->type, "emc2305", I2C_NAME_SIZE);
		break;
	case 0x35:
		strlcpy(info->type, "emc2303", I2C_NAME_SIZE);
		break;
	case 0x36:
		strlcpy(info->type, "emc2302", I2C_NAME_SIZE);
		break;
	case 0x37:
		strlcpy(info->type, "emc2301", I2C_NAME_SIZE);
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

static struct i2c_driver emc2305_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "emc2305",
	},
	.probe		= emc2305_probe,
	.id_table	= emc2305_ids,
	.detect		= emc2305_detect,
	.address_list	= i2c_addresses,
};

module_i2c_driver(emc2305_driver);

MODULE_AUTHOR("Reinhard Pfau <pfau@gdsys.de>,Biwen Li <biwen.li@nxp.com>");
MODULE_DESCRIPTION("SMSC EMC2305 hwmon driver");
MODULE_LICENSE("GPL");
