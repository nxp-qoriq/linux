// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2016 Freescale Semiconductor, Inc.
// Copyright 2022 NXP

#include <linux/clk.h>
#include <linux/device_cooling.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/thermal.h>
#include <linux/units.h>

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/qoriq_thermal_interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#endif

#include "thermal_core.h"
#include "thermal_hwmon.h"

#define SITES_MAX		16
#define TMR_DISABLE		0x0
#define TMR_ME			0x80000000
#define TMR_ALPF		0x0c000000
#define TMR_ALPF_V2		0x03000000
#define TMTMIR_DEFAULT	0x0000000f
#define TIER_DISABLE	0x0
#define TEUMR0_V2		0x51009c00
#define TEUMR0_V21		0x55010c00
#define TMSARA_V2		0xe
#define TMU_VER1		0x1
#define TMU_VER2		0x2
#define TMU_VER93		0x3
#define TMU_TEMP_PASSIVE_COOL_DELTA	10000
#define TMU_NON_IDEALITY_FACTOR	1022
#define DECI_KELVIN_TO_CELSIUS(t)	({			\
	long _t = (t);						\
	((_t-2732 >= 0) ? (_t-2732+5)/10 : (_t-2732-5)/10);	\
})
#define CELSIUS_TO_DECI_KELVIN(t)	((t)*10+2732)
#define TEMP_CELCIUS_TO_KELVIN(val)		((CELSIUS_TO_DECI_KELVIN(val)) / 10)
#define TEMP_KELVIN_TO_CELSIUS(val)		DECI_KELVIN_TO_CELSIUS(val * 10)

#define REGS_TMR	0x000	/* Mode Register */
#define TMR_DISABLE	0x0
#define TMR_ME		0x80000000
#define TMR_ALPF	0x0c000000
#define TMR_MSITE_ALL	GENMASK(15, 0)

#define REGS_TMTMIR	0x008	/* Temperature measurement interval Register */
#define TMTMIR_DEFAULT	0x0000000f

#define REGS_V2_TMSR	0x008	/* monitor site register */

#define REGS_V2_TMTMIR	0x00c	/* Temperature measurement interval Register */

#define REGS_TIER	0x020	/* Interrupt Enable Register */
#define TIER_DISABLE	0x0

#define REGS_TIDR	0x024	/* Interrupt Enable Register */
#define REGS_TISCR	0x28	/* Interrupt site capture register */
#define REGS_TICSCR	0x2C	/* Interrupt critical site capture register */
#define REGS_TMHTATR	0x54	/* Monitor high temperature average threshold register */
#define REGS_TMHTACTR	0x58	/* Monitor high temperature average critical threshold register */

#define REGS_V2_TIASCR	0x34	/* Interrupt average site capture register */
#define REGS_V2_TICSCR	0x38	/* Interrupt critical site capture register */
#define REGS_V2_TMLTATR	0x64	/* Monitor low temperature average threshold register */
#define REGS_V2_TMLTACTR	0x68	/* Monitor low temperature average crit threshold register */

#define REGS_TTCFGR	0x080	/* Temperature Configuration Register */
#define REGS_TSCFGR	0x084	/* Sensor Configuration Register */

#define REGS_TRITSR(n)	(0x100 + 16 * (n)) /* Immediate Temperature
					    * Site Register
					    */
#define TRITSR_V	BIT(31)
#define TRITSR_TP5	BIT(9)
#define REGS_V2_TMSAR(n)	(0x304 + 16 * (n))	/* TMU monitoring
						* site adjustment register
						*/
#define REGS_TTRnCR(n)	(0xf10 + 4 * (n)) /* Temperature Range n
					   * Control Register
					   */
#define REGS_IPBRR(n)		(0xbf8 + 4 * (n)) /* IP Block Revision
						   * Register n
						   */
#define REGS_V2_TEUMR(n)	(0xf00 + 4 * (n))

/*
 * Thermal zone data
 */
struct qoriq_sensor {
	int				id;
	struct thermal_zone_device	*tzd;
	int				temp_passive;
	int				temp_critical;
	struct thermal_cooling_device	*cdev;
};

struct qoriq_tmu_data {
	int ver;
	struct regmap *regmap;
	struct clk *clk;
	struct qoriq_sensor	sensor[SITES_MAX];
};

enum tmu_trip {
	TMU_TRIP_PASSIVE,
	TMU_TRIP_CRITICAL,
	TMU_TRIP_NUM,
};


#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
struct qoriq_prv_data {
	struct swait_queue_head qoriq_tmu_wq;
	raw_spinlock_t qoriq_tmu_wq_lock;
	struct qoriq_tmu_data *qdata;
};

int no_of_threshold;
int ctd_hysteresis_val;
struct task_struct *ts_crit_temp_monitoring_task;
struct task_struct *ts_avg_temp_monitoring_task;
uint32_t qoriq_thermal_event;
int avg_thread_running;
int crit_thread_running;
#endif

static struct qoriq_tmu_data *qoriq_sensor_to_data(struct qoriq_sensor *s)
{
	return container_of(s, struct qoriq_tmu_data, sensor[s->id]);
}

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
static void send_call_back(ctd_event_id_t thermal_event)
{
	struct ctd_thermal_event cbk_data;

	cbk_data.thermal_event_id = thermal_event;
	cbk_data.temp = ctd_get_temp();
	if (ctd_tvd_callback != NULL)
		ctd_tvd_callback(&cbk_data);
}

irqreturn_t qoriq_tmu_irq_handler_avg_temp(int irq, void *data)
{
	struct qoriq_tmu_data *qdata;
	u32 tidr;
	u32 tier;
	u32 tmr;
	struct qoriq_prv_data *qoriq_prv_data = (struct qoriq_prv_data *)data;

	if (qoriq_prv_data) {
		qdata = qoriq_prv_data->qdata;
		if (qdata->ver == TMU_VER1) {
			qoriq_thermal_event |= (1 << CTD_HIGH_AVG_TEMP_EVENT);
			regmap_read(qdata->regmap, REGS_TIDR, &tidr);
			regmap_read(qdata->regmap, REGS_TMR, &tmr);
			regmap_write(qdata->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(qdata->regmap, REGS_TIDR, TIER_AVERAGE_THRESHOLD_ENABLED);
			regmap_read(qdata->regmap, REGS_TIER, &tier);
			regmap_write(qdata->regmap, REGS_TIER, (~TIER_AVERAGE_THRESHOLD_ENABLED & tier));
			regmap_write(qdata->regmap, REGS_TMR, (tmr | TMR_ME));
		} else {
			regmap_read(qdata->regmap, REGS_TIDR, &tidr);
			if (tidr & TIER_AVERAGE_THRESHOLD_ENABLED) {
				qoriq_thermal_event |= (1 << CTD_HIGH_AVG_TEMP_EVENT);
				regmap_read(qdata->regmap, REGS_TIER, &tier);
				regmap_write(qdata->regmap, REGS_TIER, (~TIER_AVERAGE_THRESHOLD_ENABLED & tier));
				regmap_write(qdata->regmap, REGS_TIDR, TIER_AVERAGE_THRESHOLD_ENABLED);
			} else if (tidr & TIER_AVERAGE_LOW_THRESHOLD_ENABLED) {
				qoriq_thermal_event |= (1 << CTD_LOW_AVG_TEMP_EVENT);
				regmap_read(qdata->regmap, REGS_TIER, &tier);
				regmap_write(qdata->regmap, REGS_TIER, (~TIER_AVERAGE_LOW_THRESHOLD_ENABLED & tier));
				regmap_write(qdata->regmap, REGS_TIDR, TIER_AVERAGE_LOW_THRESHOLD_ENABLED);
			}
		}
		raw_spin_lock(&qoriq_prv_data->qoriq_tmu_wq_lock);
		swake_up_all_locked(&qoriq_prv_data->qoriq_tmu_wq);
		raw_spin_unlock(&qoriq_prv_data->qoriq_tmu_wq_lock);
	}

	return IRQ_HANDLED;
}

irqreturn_t qoriq_tmu_irq_handler_crit_temp(int irq, void *data)
{
	struct qoriq_tmu_data *qdata;
	u32 tidr;
	u32 tier;
	u32 tmr;
	struct qoriq_prv_data *qoriq_prv_data = (struct qoriq_prv_data *)data;

	if (qoriq_prv_data) {
		qdata = qoriq_prv_data->qdata;
		if (qdata->ver == TMU_VER1) {
			qoriq_thermal_event |= (1 << CTD_HIGH_CRITICAL_TEMP_EVENT);
			regmap_read(qdata->regmap, REGS_TIDR, &tidr);
			regmap_read(qdata->regmap, REGS_TMR, &tmr);
			regmap_write(qdata->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(qdata->regmap, REGS_TIDR, TIER_CRITICAL_THRESHOLD_ENABLED);
			regmap_read(qdata->regmap, REGS_TIER, &tier);
			regmap_write(qdata->regmap, REGS_TIER, (~TIER_CRITICAL_THRESHOLD_ENABLED & tier));
			regmap_write(qdata->regmap, REGS_TMR, (tmr | TMR_ME));
		} else {
			regmap_read(qdata->regmap, REGS_TIDR, &tidr);
			if (tidr & TIER_CRITICAL_THRESHOLD_ENABLED) {
				qoriq_thermal_event |= (1 << CTD_HIGH_CRITICAL_TEMP_EVENT);
				regmap_read(qdata->regmap, REGS_TIER, &tier);
				regmap_write(qdata->regmap, REGS_TIER, (~TIER_CRITICAL_THRESHOLD_ENABLED & tier));
				regmap_write(qdata->regmap, REGS_TIDR, TIER_CRITICAL_THRESHOLD_ENABLED);
			} else if (tidr & TIER_CRITICAL_LOW_THRESHOLD_ENABLED) {
				qoriq_thermal_event |= (1 << CTD_LOW_CRITICAL_TEMP_EVENT);
				regmap_read(qdata->regmap, REGS_TIER, &tier);
				regmap_write(qdata->regmap, REGS_TIER, (~TIER_CRITICAL_LOW_THRESHOLD_ENABLED & tier));
				regmap_write(qdata->regmap, REGS_TIDR, TIER_CRITICAL_LOW_THRESHOLD_ENABLED);
			}
		}
		raw_spin_lock(&qoriq_prv_data->qoriq_tmu_wq_lock);
		swake_up_all_locked(&qoriq_prv_data->qoriq_tmu_wq);
		raw_spin_unlock(&qoriq_prv_data->qoriq_tmu_wq_lock);
	}

	return IRQ_HANDLED;
}

int ctd_get_temp(void)
{
	int i, max_temp = 0;
	u32 ctd_curent_temp[MAX_TMUv2_CTD_MONITORING_SITE];
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev_tmu);
	u32 tritsr = 0;

	if (data->ver == TMU_VER1) {
		for (i = 0; i < MAX_CTD_MONITORING_SITE; i++) {
			regmap_read(data->regmap, REGS_TRITSR(i), &tritsr);
			if (!(tritsr & 0x80000000))
				break;
			ctd_curent_temp[i] = tritsr & 0xFF;
			if (ctd_curent_temp[i] > max_temp)
				max_temp = ctd_curent_temp[i];
		}

		if (!(tritsr & 0x80000000) || max_temp < MIN_CTD_TEMP ||
			max_temp > MAX_CTD_TEMP) {
			pr_err("%s : Invalid temperature, valid=%d, temp=%d\n",
			    __func__, (tritsr & 0x80000000), max_temp);
			return INVALID_CTD_TEMP;
		}

		return max_temp;
	} else {
		for (i = 0; i < MAX_TMUv2_CTD_MONITORING_SITE; i++) {
			regmap_read(data->regmap, REGS_TRITSR(i), &tritsr);
			if (!(tritsr & 0x80000000))
				break;
			ctd_curent_temp[i] = tritsr & 0x1FF;
			if (max_temp < ctd_curent_temp[i])
				max_temp = ctd_curent_temp[i];
		}

		max_temp = TEMP_KELVIN_TO_CELSIUS(max_temp);

		if (!(tritsr & 0x80000000) || max_temp < MIN_CTD_TEMP ||
			max_temp > MAX_CTD_TEMP) {
			pr_err("%s: Invalid temperature, valid=%d, temp=%d\n",
			    __func__, (tritsr & 0x80000000), max_temp);
			return INVALID_CTD_TEMP;
		}
		return max_temp;
	}
}
EXPORT_SYMBOL_GPL(ctd_get_temp);
#endif

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qoriq_sensor_to_data(qsensor);
	u32 val;
	/*
	 * REGS_TRITSR(id) has the following layout:
	 *
	 * For TMU Rev1:
	 * 31  ... 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 8 bit value representing
	 * temperature in Celsius.

	 * For TMU Rev2:
	 * 31  ... 8 7 6 5 4 3 2 1 0
	 *  V          TEMP
	 *
	 * Where V bit signifies if the measurement is ready and is
	 * within sensor range. TEMP is an 9 bit value representing
	 * temperature in KelVin.
	 */
	if (regmap_read_poll_timeout(qdata->regmap,
				     REGS_TRITSR(qsensor->id),
				     val,
				     val & TRITSR_V,
				     USEC_PER_MSEC,
				     10 * USEC_PER_MSEC))
		return -ENODATA;

	if (qdata->ver == TMU_VER1) {
		*temp = (val & GENMASK(7, 0)) * MILLIDEGREE_PER_DEGREE;
	} else if (qdata->ver == TMU_VER93) {
		if (val & TRITSR_TP5)
			*temp = milli_kelvin_to_millicelsius((val & GENMASK(8, 0)) * MILLIDEGREE_PER_DEGREE + 500);
		else
			*temp = kelvin_to_millicelsius(val & GENMASK(8, 0));
	} else {
		*temp = kelvin_to_millicelsius(val & GENMASK(8, 0));
	}

	return 0;
}

static int tmu_get_trend(void *p, int trip, enum thermal_trend *trend)
{
	struct qoriq_sensor *qsensor = p;
	int trip_temp;

	if (!qsensor->tzd)
		return 0;

	trip_temp = (trip == TMU_TRIP_PASSIVE) ? qsensor->temp_passive :
					     qsensor->temp_critical;

	if (qsensor->tzd->temperature >=
		(trip_temp - TMU_TEMP_PASSIVE_COOL_DELTA))
		*trend = THERMAL_TREND_RAISE_FULL;
	else
		*trend = THERMAL_TREND_DROP_FULL;

	return 0;
}

static int tmu_set_trip_temp(void *p, int trip,
			     int temp)
{
	struct qoriq_sensor *qsensor = p;

	if (trip == TMU_TRIP_CRITICAL)
		qsensor->temp_critical = temp;

	if (trip == TMU_TRIP_PASSIVE)
		qsensor->temp_passive = temp;

	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
	.get_trend = tmu_get_trend,
	.set_trip_temp = tmu_set_trip_temp,
};

static int qoriq_tmu_register_tmu_zone(struct device *dev,
				       struct qoriq_tmu_data *qdata)
{
	int id;
	const struct thermal_trip *trip;

	if (qdata->ver == TMU_VER1) {
		regmap_write(qdata->regmap, REGS_TMR,
			     TMR_MSITE_ALL | TMR_ME | TMR_ALPF);
	} else {
		regmap_write(qdata->regmap, REGS_V2_TMSR, TMR_MSITE_ALL);
		regmap_write(qdata->regmap, REGS_TMR, TMR_ME | TMR_ALPF_V2);
	}

	for (id = 0; id < SITES_MAX; id++) {
		struct thermal_zone_device *tzd;
		struct qoriq_sensor *sensor = &qdata->sensor[id];
		int ret;

		sensor->id = id;

		tzd = devm_thermal_zone_of_sensor_register(dev, id,
							   sensor,
							   &tmu_tz_ops);
		ret = PTR_ERR_OR_ZERO(tzd);
		if (ret) {
			if (ret == -ENODEV)
				continue;

			regmap_write(qdata->regmap, REGS_TMR, TMR_DISABLE);
			return ret;
		}

		sensor->tzd = tzd;

		if (devm_thermal_add_hwmon_sysfs(tzd))
			dev_warn(dev,
				 "Failed to add hwmon sysfs attributes\n");
		/* first thermal zone takes care of system-wide device cooling */
		if (id == 0) {
			sensor->cdev = devfreq_cooling_register();
			if (IS_ERR(sensor->cdev)) {
				ret = PTR_ERR(sensor->cdev);
				pr_err("failed to register devfreq cooling device: %d\n",
					ret);
				return ret;
			}

			ret = thermal_zone_bind_cooling_device(sensor->tzd,
				TMU_TRIP_PASSIVE,
				sensor->cdev,
				THERMAL_NO_LIMIT,
				THERMAL_NO_LIMIT,
				THERMAL_WEIGHT_DEFAULT);
			if (ret) {
				pr_err("binding zone %s with cdev %s failed:%d\n",
					sensor->tzd->type,
					sensor->cdev->type,
					ret);
				devfreq_cooling_unregister(sensor->cdev);
				return ret;
			}

			trip = of_thermal_get_trip_points(sensor->tzd);
			sensor->temp_passive = trip[0].temperature;
			sensor->temp_critical = trip[1].temperature;
		}
	}

	return 0;
}

static int qoriq_tmu_calibration(struct device *dev,
				 struct qoriq_tmu_data *data)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = dev->of_node;

	len = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (len < 0 || len > 4) {
		dev_err(dev, "invalid range data.\n");
		return len;
	}

	val = of_property_read_u32_array(np, "fsl,tmu-range", range, len);
	if (val != 0) {
		dev_err(dev, "failed to read range data.\n");
		return val;
	}

	/* Init temperature range registers */
	for (i = 0; i < len; i++)
		regmap_write(data->regmap, REGS_TTRnCR(i), range[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		regmap_write(data->regmap, REGS_TTCFGR, val);
		val = of_read_number(calibration + 1, 1);
		regmap_write(data->regmap, REGS_TSCFGR, val);
	}

	return 0;
}

static int imx93_tmu_calibration(struct device *dev,
				 struct qoriq_tmu_data *data)
{
	const u32 *calibration = NULL;
	u32 cal_pt = 0;
	u32 val = 0;
	unsigned int len = 0;
	unsigned int i = 0;

	calibration = of_get_property(dev->of_node, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len / 8 > 16 || len % 8) {
		dev_err(dev, "invalid tmu calibration\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 0x8, calibration += 2) {
		cal_pt = i / 8;
		regmap_write(data->regmap, REGS_TTCFGR, cal_pt);
		val = of_read_number(calibration, 1);
		regmap_write(data->regmap, REGS_TSCFGR, val);
		val = of_read_number(calibration + 1, 1);
		regmap_write(data->regmap, REGS_TTRnCR(cal_pt), val);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	/* Disable interrupt, using polling instead */
	regmap_write(data->regmap, REGS_TIER, TIER_DISABLE);

	/* Set update_interval */

	if (data->ver == TMU_VER1) {
		regmap_write(data->regmap, REGS_TMTMIR, TMTMIR_DEFAULT);
	} else if (data->ver == TMU_VER93) {
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
		regmap_write(data->regmap, REGS_V2_TEUMR(0), TEUMR0_V21);
	} else {
		regmap_write(data->regmap, REGS_V2_TMTMIR, TMTMIR_DEFAULT);
		regmap_write(data->regmap, REGS_V2_TEUMR(0), TEUMR0_V2);
	}

	/* Disable monitoring */
	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
}

static const struct regmap_range qoriq_yes_ranges[] = {
	regmap_reg_range(REGS_TMR, REGS_TSCFGR),
	regmap_reg_range(REGS_TTRnCR(0), REGS_TTRnCR(15)),
	regmap_reg_range(REGS_V2_TEUMR(0), REGS_V2_TEUMR(2)),
	regmap_reg_range(REGS_V2_TMSAR(0), REGS_V2_TMSAR(15)),
	regmap_reg_range(REGS_IPBRR(0), REGS_IPBRR(1)),
	/* Read only registers below */
	regmap_reg_range(REGS_TRITSR(0), REGS_TRITSR(15)),
};

static const struct regmap_access_table qoriq_wr_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges) - 1,
};

static const struct regmap_access_table qoriq_rd_table = {
	.yes_ranges	= qoriq_yes_ranges,
	.n_yes_ranges	= ARRAY_SIZE(qoriq_yes_ranges),
};

static void qoriq_tmu_action(void *p)
{
	struct qoriq_tmu_data *data = p;

	regmap_write(data->regmap, REGS_TMR, TMR_DISABLE);
	clk_disable_unprepare(data->clk);
}

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
static int thread_cb(void *arg)
{
	struct qoriq_prv_data *qoriq_prv_data = (struct qoriq_prv_data *) arg;
	unsigned long flags;
	DECLARE_SWAITQUEUE(qoriq_tmu_wait);

	while (1) {
		raw_spin_lock_irqsave(&qoriq_prv_data->qoriq_tmu_wq_lock,
					flags);
		prepare_to_swait_exclusive(&qoriq_prv_data->qoriq_tmu_wq,
				&qoriq_tmu_wait, TASK_INTERRUPTIBLE);
		raw_spin_unlock_irqrestore(&qoriq_prv_data->qoriq_tmu_wq_lock,
						flags);

		/* Wait here*/
		schedule();
		raw_spin_lock_irqsave(&qoriq_prv_data->qoriq_tmu_wq_lock,
					flags);
		finish_swait(&qoriq_prv_data->qoriq_tmu_wq, &qoriq_tmu_wait);
		raw_spin_unlock_irqrestore(&qoriq_prv_data->qoriq_tmu_wq_lock,
						flags);
		if (qoriq_thermal_event & (1<<CTD_HIGH_AVG_TEMP_EVENT)) {
			send_call_back(CTD_HIGH_AVG_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_HIGH_AVG_TEMP_EVENT);
			msleep(100);
		} else if (qoriq_thermal_event & (1<<CTD_LOW_AVG_TEMP_EVENT)) {
			send_call_back(CTD_LOW_AVG_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_LOW_AVG_TEMP_EVENT);
			msleep(100);
		}
		if (qoriq_thermal_event & (1<<CTD_HIGH_CRITICAL_TEMP_EVENT)) {
			send_call_back(CTD_HIGH_CRITICAL_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_HIGH_CRITICAL_TEMP_EVENT);
			msleep(100);
		} else if (qoriq_thermal_event & (1<<CTD_LOW_CRITICAL_TEMP_EVENT)) {
			send_call_back(CTD_LOW_CRITICAL_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_LOW_CRITICAL_TEMP_EVENT);
			msleep(100);
		}
	}

	return 0;
}

int qoriq_tmu_register_interrupt(struct platform_device *pdev,
			struct ctd_thermal_threshold *thermal_threshold)
{
	int irq, low_threshold, high_threshold;
	int low_threshold_low, high_threshold_low;
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);
	u32 tmr;
	struct task_struct *ts;
	struct qoriq_prv_data *qoriq_prv_data = NULL;
	static int irq_initialized = 0;

	if (!irq_initialized) {
		qoriq_prv_data = kmalloc(sizeof(struct qoriq_prv_data),
					GFP_KERNEL);
		if (!qoriq_prv_data) {
			ret = -ENOMEM;
			goto err;
		}
		qoriq_prv_data->qdata = data;
		irq = of_irq_get(np, 1);
		ret = request_irq(irq, qoriq_tmu_irq_handler_crit_temp,
			IRQ_TYPE_LEVEL_HIGH, "TMU critical alarm",
			qoriq_prv_data);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register interrupt.\n");
			goto err_crit_tmp_irq_failed;
		}
		irq = of_irq_get(np, 0);
		ret = request_irq(irq, qoriq_tmu_irq_handler_avg_temp,
			IRQ_TYPE_LEVEL_HIGH, "TMU alarm", qoriq_prv_data);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register interrupt.\n");
			goto err_avg_tmp_irq_failed;
		}

		init_swait_queue_head(&qoriq_prv_data->qoriq_tmu_wq);
		raw_spin_lock_init(&qoriq_prv_data->qoriq_tmu_wq_lock);
		ts = kthread_run(thread_cb, (void *) qoriq_prv_data,
					"call back thread");
		if (IS_ERR(ts)) {
			dev_err(&pdev->dev,
				"ERROR: Cannot create thread thread_cb\n");
			ret = (int)PTR_ERR(ts);
			goto err_thread_create;
		}
		irq_initialized = true;

	}

	no_of_threshold = thermal_threshold->theshold_cnt;
	/* Adding hysteresis value with the threshold and programmed the
	updated threshold value in thermal register for interrupt
	Programmed high threshold = Original threshold + hysteresis */
	low_threshold = thermal_threshold->threshold[0] + ctd_hysteresis_val;
	high_threshold = thermal_threshold->threshold[1] + ctd_hysteresis_val;
	if (thermal_threshold->theshold_cnt == MAX_THRESHOLD) {
		if (data->ver == TMU_VER1) {
			regmap_read(data->regmap, REGS_TMR, &tmr);
			regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(data->regmap, REGS_TICSCR, (tmr | 0xF800));
			regmap_write(data->regmap, REGS_TISCR, (tmr | 0xF800));
			regmap_write(data->regmap, REGS_TMHTACTR, (high_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TMHTATR, (low_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TIDR,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED));
			regmap_write(data->regmap, REGS_TIER,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED));
			regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));
		} else {
			low_threshold_low = thermal_threshold->threshold[0] -
				ctd_hysteresis_val;
			high_threshold_low = thermal_threshold->threshold[1] -
				ctd_hysteresis_val;
			low_threshold_low = TEMP_CELCIUS_TO_KELVIN(low_threshold_low);
			high_threshold_low = TEMP_CELCIUS_TO_KELVIN(high_threshold_low);
			low_threshold = TEMP_CELCIUS_TO_KELVIN(low_threshold);
			high_threshold = TEMP_CELCIUS_TO_KELVIN(high_threshold);
			regmap_read(data->regmap, REGS_TMR, &tmr);
			regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(data->regmap, REGS_V2_TICSCR, 0x0);
			regmap_write(data->regmap, REGS_V2_TIASCR, 0x0);
			regmap_write(data->regmap, REGS_TMHTACTR, (high_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TMHTATR, (low_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_V2_TMLTACTR, (high_threshold_low | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_V2_TMLTATR, (low_threshold_low | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TIDR,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED |
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED |
				TIER_CRITICAL_LOW_THRESHOLD_ENABLED));
			regmap_write(data->regmap, REGS_TIER,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED |
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED |
				TIER_CRITICAL_LOW_THRESHOLD_ENABLED));
			udelay(100);
			regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));


		}
	} else if (thermal_threshold->theshold_cnt == (MAX_THRESHOLD - 1)) {
		if (data->ver == TMU_VER1) {
			regmap_read(data->regmap, REGS_TMR, &tmr);
			regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(data->regmap, REGS_TISCR, (tmr | 0xF800));
			regmap_write(data->regmap, REGS_TMHTATR, (low_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TIDR, TIER_AVERAGE_THRESHOLD_ENABLED);
			regmap_write(data->regmap, REGS_TIER, TIER_AVERAGE_THRESHOLD_ENABLED);
			regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));
		} else {
			low_threshold_low = thermal_threshold->threshold[0] -
				ctd_hysteresis_val;
			low_threshold_low = TEMP_CELCIUS_TO_KELVIN(low_threshold_low);
			low_threshold = TEMP_CELCIUS_TO_KELVIN(low_threshold);
			regmap_read(data->regmap, REGS_TMR, &tmr);
			regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
			regmap_write(data->regmap, REGS_V2_TIASCR, (tmr | 0x1F));
			regmap_write(data->regmap, REGS_TMHTATR, (low_threshold | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_V2_TIASCR, (tmr | 0x1F));
			regmap_write(data->regmap, REGS_V2_TMLTATR, (low_threshold_low | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_TIDR,
					(TIER_AVERAGE_THRESHOLD_ENABLED |
					TIER_AVERAGE_LOW_THRESHOLD_ENABLED));
			regmap_write(data->regmap, REGS_TIER,
					(TIER_AVERAGE_THRESHOLD_ENABLED |
					TIER_AVERAGE_LOW_THRESHOLD_ENABLED));
			regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));
		}
	} else {
		/*Do nothing*/
	}

	return 0;

err_thread_create:
	free_irq(of_irq_get(np, 0), qoriq_prv_data);

err_avg_tmp_irq_failed:
	free_irq(of_irq_get(np, 1), qoriq_prv_data);

err_crit_tmp_irq_failed:
	kfree(qoriq_prv_data);

err:
	return ret;
}

int qoriq_tmu_update_threshold(struct platform_device *pdev, int hysteresis_val)
{
	u32 tmr, threshold, val;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	ctd_hysteresis_val = hysteresis_val;
	if (data->ver == TMU_VER1) {
		regmap_read(data->regmap, REGS_TMR, &tmr);
		regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
		/* Adding hysteresis value with the threshold and programmed the
		 * updated threshold value in thermal register for interrupt
		 * Programmed high threshold = Original threshold + hysteresis
		 **/
		regmap_read(data->regmap, REGS_TMHTATR, &threshold);
		val = threshold & 0xFF;
		if (threshold & THRESHOLD_VALID)
			regmap_write(data->regmap, REGS_TMHTATR, ((val + ctd_hysteresis_val) | THRESHOLD_VALID));
		regmap_read(data->regmap, REGS_TMHTACTR, &threshold);
		val = threshold & 0xFF;
		if (threshold & THRESHOLD_VALID)
			regmap_write(data->regmap, REGS_TMHTACTR, ((val + ctd_hysteresis_val) | THRESHOLD_VALID));
		regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));
	} else {
		regmap_read(data->regmap, REGS_TMR, &tmr);
		regmap_write(data->regmap, REGS_TMR, (tmr & ~TMR_ME));
		/* Adding hysteresis value with the threshold and programmed the
		 * updated threshold value in thermal register for interrupt
		 * Programmed high threshold = Original threshold + hysteresis
		 **/
		regmap_read(data->regmap, REGS_TMHTATR, &threshold);
		val = threshold & 0x1FF;
		if (threshold & THRESHOLD_VALID) {
			regmap_write(data->regmap, REGS_TMHTATR, ((val + ctd_hysteresis_val) | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_V2_TMLTATR, ((val - ctd_hysteresis_val) | THRESHOLD_VALID));
		}
		regmap_read(data->regmap, REGS_TMHTACTR, &threshold);
		val = threshold & 0x1FF;
		if (threshold & THRESHOLD_VALID) {
			regmap_write(data->regmap, REGS_TMHTACTR, ((val + ctd_hysteresis_val) | THRESHOLD_VALID));
			regmap_write(data->regmap, REGS_V2_TMLTACTR, ((val - ctd_hysteresis_val) | THRESHOLD_VALID));
		}
		regmap_write(data->regmap, REGS_TMR, (tmr | TMR_ME));
	}

	return 0;
}
#endif

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 ver;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	const bool little_endian = of_property_read_bool(np, "little-endian");
	const enum regmap_endian format_endian =
		little_endian ? REGMAP_ENDIAN_LITTLE : REGMAP_ENDIAN_BIG;
	const struct regmap_config regmap_config = {
		.reg_bits		= 32,
		.val_bits		= 32,
		.reg_stride		= 4,
		.rd_table		= &qoriq_rd_table,
		.wr_table		= &qoriq_wr_table,
		.val_format_endian	= format_endian,
		.max_register		= SZ_4K,
	};
	void __iomem *base;

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
	pdev_tmu = pdev;
#endif
	data = devm_kzalloc(dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	ret = PTR_ERR_OR_ZERO(base);
	if (ret) {
		dev_err(dev, "Failed to get memory region\n");
		return ret;
	}

	data->regmap = devm_regmap_init_mmio(dev, base, &regmap_config);
	ret = PTR_ERR_OR_ZERO(data->regmap);
	if (ret) {
		dev_err(dev, "Failed to init regmap (%d)\n", ret);
		return ret;
	}

	data->clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, qoriq_tmu_action, data);
	if (ret)
		return ret;

	/* version register offset at: 0xbf8 on both v1 and v2 */
	ret = regmap_read(data->regmap, REGS_IPBRR(0), &ver);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read IP block version\n");
		return ret;
	}
	data->ver = (ver >> 8) & 0xff;

	if (of_find_compatible_node(NULL, NULL, "fsl,imx93-tmu"))
		data->ver = TMU_VER93;

	qoriq_tmu_init_device(data);	/* TMU initialization */

	if (data->ver == TMU_VER93)
		ret = imx93_tmu_calibration(dev, data);
	else
		ret = qoriq_tmu_calibration(dev, data);	/* TMU calibration */
	if (ret < 0)
		return ret;

	ret = qoriq_tmu_register_tmu_zone(dev, data);
	if (ret < 0) {
		dev_err(dev, "Failed to register sensors\n");
		return ret;
	}

	platform_set_drvdata(pdev, data);

	return 0;
}

static int __maybe_unused qoriq_tmu_suspend(struct device *dev)
{
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);
	int ret;

	ret = regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, 0);
	if (ret)
		return ret;

	clk_disable_unprepare(data->clk);

	return 0;
}

static int __maybe_unused qoriq_tmu_resume(struct device *dev)
{
	int ret;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	/* Enable monitoring */
	return regmap_update_bits(data->regmap, REGS_TMR, TMR_ME, TMR_ME);
}

static SIMPLE_DEV_PM_OPS(qoriq_tmu_pm_ops,
			 qoriq_tmu_suspend, qoriq_tmu_resume);

static const struct of_device_id qoriq_tmu_match[] = {
	{ .compatible = "fsl,qoriq-tmu", },
	{ .compatible = "fsl,imx8mq-tmu", },
	{ .compatible = "fsl,imx93-tmu", },
	{},
};
MODULE_DEVICE_TABLE(of, qoriq_tmu_match);

static struct platform_driver qoriq_tmu = {
	.driver	= {
		.name		= "qoriq_thermal",
		.pm		= &qoriq_tmu_pm_ops,
		.of_match_table	= qoriq_tmu_match,
	},
	.probe	= qoriq_tmu_probe,
};
module_platform_driver(qoriq_tmu);

MODULE_AUTHOR("Jia Hongtao <hongtao.jia@nxp.com>");
MODULE_DESCRIPTION("QorIQ Thermal Monitoring Unit driver");
MODULE_LICENSE("GPL v2");
