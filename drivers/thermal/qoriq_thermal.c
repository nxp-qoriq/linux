// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018,2020-2021 NXP
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/thermal.h>

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/qoriq_thermal_interrupt.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#endif

#include "thermal_core.h"

#define SITES_MAX		16
#define TMR_DISABLE		0x0
#define TMR_ME			0x80000000
#define TMR_ALPF		0x0c000000
#define TMR_ALPF_V2		0x03000000
#define TMTMIR_DEFAULT	0x0000000f
#define TIER_DISABLE	0x0
#define TEUMR0_V2		0x51009c00
#define TMSARA_V2		0xe
#define TMU_VER1		0x1
#define TMU_VER2		0x2
#define TEMP_CELCIUS_TO_KELVIN(val)		((CELSIUS_TO_DECI_KELVIN(val)) / 10)
#define TEMP_KELVIN_TO_CELSIUS(val)		DECI_KELVIN_TO_CELSIUS(val * 10)

/*
 * QorIQ TMU Registers
 */
struct qoriq_tmu_site_regs {
	u32 tritsr;		/* Immediate Temperature Site Register */
	u32 tratsr;		/* Average Temperature Site Register */
	u8 res0[0x8];
};

struct qoriq_tmu_tmsar {
	u32 res0;
	u32 tmsar;
	u32 res1;
	u32 res2;
};

struct qoriq_tmu_regs_v1 {
	u32 tmr;		/* Mode Register */
	u32 tsr;		/* Status Register */
	u32 tmtmir;		/* Temperature measurement interval Register */
	u8 res0[0x14];
	u32 tier;		/* Interrupt Enable Register */
	u32 tidr;		/* Interrupt Detect Register */
	u32 tiscr;		/* Interrupt Site Capture Register */
	u32 ticscr;		/* Interrupt Critical Site Capture Register */
	u8 res1[0x10];
	u32 tmhtcrh;		/* High Temperature Capture Register */
	u32 tmhtcrl;		/* Low Temperature Capture Register */
	u8 res2[0x8];
	u32 tmhtitr;		/* High Temperature Immediate Threshold */
	u32 tmhtatr;		/* High Temperature Average Threshold */
	u32 tmhtactr;	/* High Temperature Average Crit Threshold */
	u8 res3[0x24];
	u32 ttcfgr;		/* Temperature Configuration Register */
	u32 tscfgr;		/* Sensor Configuration Register */
	u8 res4[0x78];
	struct qoriq_tmu_site_regs site[SITES_MAX];
	u8 res5[0x9f8];
	u32 ipbrr0;		/* IP Block Revision Register 0 */
	u32 ipbrr1;		/* IP Block Revision Register 1 */
	u8 res6[0x310];
	u32 ttrcr[4];		/* Temperature Range Control Register */
};

struct qoriq_tmu_regs_v2 {
	u32 tmr;		/* Mode Register */
	u32 tsr;		/* Status Register */
	u32 tmsr;		/* monitor site register */
	u32 tmtmir;		/* Temperature measurement interval Register */
	u8 res0[0x10];
	u32 tier;		/* Interrupt Enable Register */
	u32 tidr;		/* Interrupt Detect Register */
	u8 res1[0x8];
	u32 tiiscr;		/* interrupt immediate site capture register */
	u32 tiascr;		/* interrupt average site capture register */
	u32 ticscr;		/* Interrupt Critical Site Capture Register */
	u32 res2;
	u32 tmhtcr;		/* monitor high temperature capture register */
	u32 tmltcr;		/* monitor low temperature capture register */
	u32 tmrtrcr;	/* monitor rising temperature rate capture register */
	u32 tmftrcr;	/* monitor falling temperature rate capture register */
	u32 tmhtitr;	/* High Temperature Immediate Threshold */
	u32 tmhtatr;	/* High Temperature Average Threshold */
	u32 tmhtactr;	/* High Temperature Average Crit Threshold */
	u32 res3;
	u32 tmltitr;	/* monitor low temperature immediate threshold */
	u32 tmltatr;	/* monitor low temperature average threshold register */
	u32 tmltactr;	/* monitor low temperature average critical threshold */
	u32 res4;
	u32 tmrtrctr;	/* monitor rising temperature rate critical threshold */
	u32 tmftrctr;	/* monitor falling temperature rate critical threshold*/
	u8 res5[0x8];
	u32 ttcfgr;	/* Temperature Configuration Register */
	u32 tscfgr;	/* Sensor Configuration Register */
	u8 res6[0x78];
	struct qoriq_tmu_site_regs site[SITES_MAX];
	u8 res10[0x100];
	struct qoriq_tmu_tmsar tmsar[16];
	u8 res7[0x7f8];
	u32 ipbrr0;		/* IP Block Revision Register 0 */
	u32 ipbrr1;		/* IP Block Revision Register 1 */
	u8 res8[0x300];
	u32 teumr0;
	u32 teumr1;
	u32 teumr2;
	u32 res9;
	u32 ttrcr[4];	/* Temperature Range Control Register */
};

struct qoriq_tmu_data;

/*
 * Thermal zone data
 */
struct qoriq_sensor {
	struct thermal_zone_device	*tzd;
	struct qoriq_tmu_data		*qdata;
	int				id;
};

struct qoriq_tmu_data {
	int ver;
	struct qoriq_tmu_regs_v1 __iomem *regs;
	struct qoriq_tmu_regs_v2 __iomem *regs_v2;
	bool little_endian;
	struct qoriq_sensor	*sensor[SITES_MAX];
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

static void tmu_write(struct qoriq_tmu_data *p, u32 val, void __iomem *addr)
{
	if (p->little_endian)
		iowrite32(val, addr);
	else
		iowrite32be(val, addr);
}

static u32 tmu_read(struct qoriq_tmu_data *p, void __iomem *addr)
{
	if (p->little_endian)
		return ioread32(addr);
	else
		return ioread32be(addr);
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
	u32 tmr;
	struct qoriq_prv_data *qoriq_prv_data = (struct qoriq_prv_data *)data;

	if (qoriq_prv_data) {
		qdata = qoriq_prv_data->qdata;
		qoriq_thermal_event |= (1 << CTD_HIGH_AVG_TEMP_EVENT);
		tidr = tmu_read(qdata, &qdata->regs->tidr);
		tmr = tmu_read(qdata, &qdata->regs->tmr);
		tmu_write(qdata, tmr & ~TMR_ME, &qdata->regs->tmr);
		tmu_write(qdata, TIER_AVERAGE_THRESHOLD_ENABLED,
				&qdata->regs->tidr);
		tmu_write(qdata,
			(~TIER_AVERAGE_THRESHOLD_ENABLED &
			tmu_read(qdata, &qdata->regs->tier)),
			&qdata->regs->tier);
		tmu_write(qdata, tmr | TMR_ME, &qdata->regs->tmr);
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
	u32 tmr;
	struct qoriq_prv_data *qoriq_prv_data = (struct qoriq_prv_data *)data;

	if (qoriq_prv_data) {
		qoriq_thermal_event |= (1 << CTD_HIGH_CRITICAL_TEMP_EVENT);
		qdata = qoriq_prv_data->qdata;
		tidr = tmu_read(qdata, &qdata->regs->tidr);
		tmr = tmu_read(qdata, &qdata->regs->tmr);
		tmu_write(qdata, tmr & ~TMR_ME, &qdata->regs->tmr);
		tmu_write(qdata, TIER_CRITICAL_THRESHOLD_ENABLED,
				&qdata->regs->tidr);
		tmu_write(qdata,
			(~TIER_CRITICAL_THRESHOLD_ENABLED &
			tmu_read(qdata, &qdata->regs->tier)),
			&qdata->regs->tier);
		tmu_write(qdata, tmr | TMR_ME, &qdata->regs->tmr);
		raw_spin_lock(&qoriq_prv_data->qoriq_tmu_wq_lock);
		swake_up_all_locked(&qoriq_prv_data->qoriq_tmu_wq);
		raw_spin_unlock(&qoriq_prv_data->qoriq_tmu_wq_lock);
	}

	return IRQ_HANDLED;
}

int ctd_get_temp(void)
{
	int i, max_temp = 0;
	int ctd_curent_temp[MAX_CTD_MONITORING_SITE];
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev_tmu);

	if (data->ver == TMU_VER1) {
		for (i = 0; i < MAX_CTD_MONITORING_SITE; i++) {
			ctd_curent_temp[i] = tmu_read(data,
					&data->regs->site[i].tritsr) & 0xff;
			if (ctd_curent_temp[i] > max_temp)
				max_temp = ctd_curent_temp[i];
		}
		return max_temp;
	} else {
		for (i = 0; i < MAX_CTD_MONITORING_SITE; i++) {
			ctd_curent_temp[i] = tmu_read(data,
					&data->regs_v2->site[i].tritsr);
			if ((ctd_curent_temp[i] & 0x80000000))
				if (max_temp < (ctd_curent_temp[i] & 0x1FF))
					max_temp = ctd_curent_temp[i] & 0x1FF;
		}
		return TEMP_KELVIN_TO_CELSIUS(max_temp);
	}
}
EXPORT_SYMBOL_GPL(ctd_get_temp);
#endif

static int tmu_get_temp(void *p, int *temp)
{
	struct qoriq_sensor *qsensor = p;
	struct qoriq_tmu_data *qdata = qsensor->qdata;
	u32 val;

	val = tmu_read(qdata, &qdata->regs->site[qsensor->id].tritsr);
	if (qdata->ver == TMU_VER1)
		*temp = (val & 0xff) * 1000;
	else
		*temp = (val & 0x1ff) * 1000 - 273150;

	return 0;
}

static const struct thermal_zone_of_device_ops tmu_tz_ops = {
	.get_temp = tmu_get_temp,
};

static int qoriq_tmu_register_tmu_zone(struct platform_device *pdev)
{
	struct qoriq_tmu_data *qdata = platform_get_drvdata(pdev);
	int id, sites = 0;

	for (id = 0; id < SITES_MAX; id++) {
		qdata->sensor[id] = devm_kzalloc(&pdev->dev,
				sizeof(struct qoriq_sensor), GFP_KERNEL);
		if (!qdata->sensor[id])
			return -ENOMEM;

		qdata->sensor[id]->id = id;
		qdata->sensor[id]->qdata = qdata;
		qdata->sensor[id]->tzd = devm_thermal_zone_of_sensor_register(
				&pdev->dev, id, qdata->sensor[id], &tmu_tz_ops);
		if (IS_ERR(qdata->sensor[id]->tzd)) {
			if (PTR_ERR(qdata->sensor[id]->tzd) == -ENODEV)
				continue;
			else
				return PTR_ERR(qdata->sensor[id]->tzd);
		}

		if (qdata->ver == TMU_VER1)
			sites |= 0x1 << (15 - id);
		else
			sites |= 0x1 << id;
	}

	/* Enable monitoring */
	if (sites != 0) {
		if (qdata->ver == TMU_VER1) {
			tmu_write(qdata, sites | TMR_ME | TMR_ALPF,
					&qdata->regs->tmr);
		} else {
			tmu_write(qdata, sites, &qdata->regs_v2->tmsr);
			tmu_write(qdata, TMR_ME | TMR_ALPF_V2,
					&qdata->regs_v2->tmr);
		}
	}

	return 0;
}

static int qoriq_tmu_calibration(struct platform_device *pdev)
{
	int i, val, len;
	u32 range[4];
	const u32 *calibration;
	struct device_node *np = pdev->dev.of_node;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	len = of_property_count_u32_elems(np, "fsl,tmu-range");
	if (len < 0 || len > 4) {
		dev_err(&pdev->dev, "invalid range data.\n");
		return len;
	}

	val = of_property_read_u32_array(np, "fsl,tmu-range", range, len);
	if (val != 0) {
		dev_err(&pdev->dev, "failed to read range data.\n");
		return val;
	}

	/* Init temperature range registers */
	for (i = 0; i < len; i++)
		tmu_write(data, range[i], &data->regs->ttrcr[i]);

	calibration = of_get_property(np, "fsl,tmu-calibration", &len);
	if (calibration == NULL || len % 8) {
		dev_err(&pdev->dev, "invalid calibration data.\n");
		return -ENODEV;
	}

	for (i = 0; i < len; i += 8, calibration += 2) {
		val = of_read_number(calibration, 1);
		tmu_write(data, val, &data->regs->ttcfgr);
		val = of_read_number(calibration + 1, 1);
		tmu_write(data, val, &data->regs->tscfgr);
	}

	return 0;
}

static void qoriq_tmu_init_device(struct qoriq_tmu_data *data)
{
	int i;

	/* Disable interrupt, using polling instead */
	tmu_write(data, TIER_DISABLE, &data->regs->tier);

	/* Set update_interval */
	if (data->ver == TMU_VER1) {
		tmu_write(data, TMTMIR_DEFAULT, &data->regs->tmtmir);
	} else {
		tmu_write(data, TMTMIR_DEFAULT, &data->regs_v2->tmtmir);
		tmu_write(data, TEUMR0_V2, &data->regs_v2->teumr0);
		for (i = 0; i < 7; i++)
			tmu_write(data, TMSARA_V2, &data->regs_v2->tmsar[i].tmsar);
	}

	/* Disable monitoring */
	tmu_write(data, TMR_DISABLE, &data->regs->tmr);
}

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
static int thread_monitoring_avg_low_temp(void *arg)
{
	int curent_temp;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev_tmu);
	int low_temp_threshold;
	(void) arg;
	while (!kthread_should_stop()) {
		low_temp_threshold = (tmu_read(data, &data->regs->tmhtatr) &
					0x1FF);
		low_temp_threshold = TEMP_KELVIN_TO_CELSIUS(low_temp_threshold);
		curent_temp = ctd_get_temp();

		/*
		 * Programmed high threshold = Original threshold + hysteresis
		 * Subtracting hysteresis value twice to get low temp threshold.
		 * where, programmed low threshold = Original threshold - hysteresis
		 */
		if ((curent_temp < (low_temp_threshold - (2 * ctd_hysteresis_val)))) {
			tmu_write(data, TIER_AVERAGE_THRESHOLD_ENABLED,
				&data->regs->tier);
			send_call_back(CTD_LOW_AVG_TEMP_EVENT);
			avg_thread_running = 0;
			kthread_stop(ts_avg_temp_monitoring_task);
			continue;
		}
		msleep(TEMP_POLLING_DEFAULT_SLEEP_TIME);
	}
	return 0;
}

static int thread_monitoring_crit_low_temp(void *arg)
{
	int curent_temp;
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev_tmu);
	int high_temp_threshold;
	(void) arg;
	while (!kthread_should_stop()) {
		high_temp_threshold = (tmu_read(data, &data->regs->tmhtactr) &
					0x1FF);
		high_temp_threshold = TEMP_KELVIN_TO_CELSIUS(high_temp_threshold);
		curent_temp = ctd_get_temp();

		/* Programmed high threshold = Original threshold + hysteresis 
		Subtracting hysteresis value twice to get low temp threshold.
		where, programmed low threshold = Original threshold - hysteresis */
		if ((curent_temp < (high_temp_threshold - (2 * ctd_hysteresis_val)))) {
			tmu_write(data, TIER_CRITICAL_THRESHOLD_ENABLED,
				&data->regs->tier);
			send_call_back(CTD_LOW_CRITICAL_TEMP_EVENT);
			crit_thread_running = 0;
			kthread_stop(ts_crit_temp_monitoring_task);
			continue;
		}
		msleep(TEMP_POLLING_DEFAULT_SLEEP_TIME);
	}

	return 0;
}

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
			if (!avg_thread_running) {
				ts_avg_temp_monitoring_task = kthread_run(
					thread_monitoring_avg_low_temp,
					NULL, "Avg low temp monitoring thread");
				if (IS_ERR(ts_avg_temp_monitoring_task))
					dev_err(&pdev_tmu->dev,
						"ERROR: Cannot create thread\n");
				avg_thread_running = 1;
			}
			send_call_back(CTD_HIGH_AVG_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_HIGH_AVG_TEMP_EVENT);
			msleep(100);
		}
		if (qoriq_thermal_event & (1<<CTD_HIGH_CRITICAL_TEMP_EVENT)) {
			if (!crit_thread_running) {
				ts_crit_temp_monitoring_task = kthread_run(
					thread_monitoring_crit_low_temp,
					NULL, "Crit low temp monitoring thread");
				if (IS_ERR(ts_crit_temp_monitoring_task))
					dev_err(&pdev_tmu->dev,
						"ERROR: Cannot create thread\n");
				crit_thread_running = 1;
			}
			send_call_back(CTD_HIGH_CRITICAL_TEMP_EVENT);
			qoriq_thermal_event &= ~(1 << CTD_HIGH_CRITICAL_TEMP_EVENT);
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
			IRQ_TYPE_EDGE_RISING, "TMU critical alarm",
			qoriq_prv_data);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to register interrupt.\n");
			goto err_crit_tmp_irq_failed;
		}
		irq = of_irq_get(np, 0);
		ret = request_irq(irq, qoriq_tmu_irq_handler_avg_temp,
			IRQ_TYPE_EDGE_RISING, "TMU alarm", qoriq_prv_data);
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
	updated thresold value in thermal register for interrupt 
	Programmed high threshold = Original threshold + hysteresis */
	low_threshold = thermal_threshold->threshold[0] + ctd_hysteresis_val;
	high_threshold = thermal_threshold->threshold[1] + ctd_hysteresis_val;
	if (thermal_threshold->theshold_cnt == MAX_THRESHOLD) {
		if (data->ver == TMU_VER1) {
			tmr = tmu_read(data, &data->regs->tmr);
			tmu_write(data, tmr & ~TMR_ME, &data->regs->tmr);
			tmu_write(data, (tmr | 0xF800), &data->regs->ticscr);
			tmu_write(data, (tmr | 0xF800), &data->regs->tiscr);
			tmu_write(data,
				(high_threshold | THRESHOLD_VALID),
				&data->regs->tmhtactr);
			tmu_write(data,
				(low_threshold | THRESHOLD_VALID),
				&data->regs->tmhtatr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED),
				&data->regs->tidr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED),
				&data->regs->tier);
			tmu_write(data, (tmr | TMR_ME), &data->regs->tmr);
		} else {
			low_threshold_low = thermal_threshold->threshold[0] -
				ctd_hysteresis_val;
			high_threshold_low = thermal_threshold->threshold[1] -
				ctd_hysteresis_val;
			low_threshold_low = TEMP_CELCIUS_TO_KELVIN(low_threshold_low);
			high_threshold_low = TEMP_CELCIUS_TO_KELVIN(high_threshold_low);
			low_threshold = TEMP_CELCIUS_TO_KELVIN(low_threshold);
			high_threshold = TEMP_CELCIUS_TO_KELVIN(high_threshold);
			tmr = tmu_read(data, &data->regs_v2->tmr);
			tmu_write(data, tmr & ~TMR_ME, &data->regs_v2->tmr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->ticscr);
			tmu_write(data,
				(high_threshold | THRESHOLD_VALID),
				&data->regs_v2->tmhtactr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->tiascr);
			tmu_write(data,
				(low_threshold | THRESHOLD_VALID),
				&data->regs_v2->tmhtatr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->ticscr);
			tmu_write(data,
				(high_threshold_low | THRESHOLD_VALID),
				&data->regs_v2->tmltactr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->tiascr);
			tmu_write(data,
				(low_threshold_low | THRESHOLD_VALID),
				&data->regs_v2->tmltatr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED |
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED |
				TIER_CRITICAL_LOW_THRESHOLD_ENABLED),
				&data->regs_v2->tidr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_CRITICAL_THRESHOLD_ENABLED	|
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED |
				TIER_CRITICAL_LOW_THRESHOLD_ENABLED),
				&data->regs_v2->tier);
			tmu_write(data, (tmr | TMR_ME), &data->regs_v2->tmr);


		}
	} else if (thermal_threshold->theshold_cnt == (MAX_THRESHOLD - 1)) {
		if (data->ver == TMU_VER1) {
			tmr = tmu_read(data, &data->regs->tmr);
			tmu_write(data, tmr & ~TMR_ME, &data->regs->tmr);
			tmu_write(data, (tmr | 0xF800), &data->regs->tiscr);
			tmu_write(data, (low_threshold | THRESHOLD_VALID),
				&data->regs->tmhtatr);
			tmu_write(data, TIER_AVERAGE_THRESHOLD_ENABLED,
				&data->regs->tidr);
			tmu_write(data, TIER_AVERAGE_THRESHOLD_ENABLED,
				&data->regs->tier);
			tmu_write(data, (tmr | TMR_ME), &data->regs->tmr);
		} else {
			low_threshold_low = thermal_threshold->threshold[0] -
				ctd_hysteresis_val;
			low_threshold_low = TEMP_CELCIUS_TO_KELVIN(low_threshold_low);
			low_threshold = TEMP_CELCIUS_TO_KELVIN(low_threshold);
			tmr = tmu_read(data, &data->regs_v2->tmr);
			tmu_write(data, tmr & ~TMR_ME, &data->regs_v2->tmr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->tiascr);
			tmu_write(data,
				(low_threshold | THRESHOLD_VALID),
				&data->regs_v2->tmhtatr);
			tmu_write(data, (tmr | 0x1F), &data->regs_v2->tiascr);
			tmu_write(data,
				(low_threshold_low | THRESHOLD_VALID),
				&data->regs_v2->tmltatr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED),
				&data->regs_v2->tidr);
			tmu_write(data,
				(TIER_AVERAGE_THRESHOLD_ENABLED |
				TIER_AVERAGE_LOW_THRESHOLD_ENABLED),
				&data->regs_v2->tier);
			tmu_write(data, (tmr | TMR_ME), &data->regs_v2->tmr);

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
	tmr = tmu_read(data, &data->regs->tmr);
	tmu_write(data, tmr & ~TMR_ME, &data->regs->tmr);

	/* Adding hysteresis value with the threshold and programmed the
	updated thresold value in thermal register for interrupt
	Programmed high threshold = Original threshold + hysteresis */
	threshold = tmu_read(data, &data->regs->tmhtatr);
	val = threshold & 0xFF;
	if (threshold & THRESHOLD_VALID)
		tmu_write(data, ((val + ctd_hysteresis_val) | THRESHOLD_VALID),
                                &data->regs->tmhtatr);
	threshold = tmu_read(data, &data->regs->tmhtactr);
	val = threshold & 0xFF;
	if (threshold & THRESHOLD_VALID)
		tmu_write(data, ((val + ctd_hysteresis_val) | THRESHOLD_VALID),
                                &data->regs->tmhtactr);
	tmu_write(data, (tmr | TMR_ME), &data->regs->tmr);

	return 0;
}
#endif

static int qoriq_tmu_probe(struct platform_device *pdev)
{
	int ret;
	u32 ver;
	struct qoriq_tmu_data *data;
	struct device_node *np = pdev->dev.of_node;

#ifdef CONFIG_QORIQ_THERMAL_INTERRUPT
	pdev_tmu = pdev;
#endif
	data = devm_kzalloc(&pdev->dev, sizeof(struct qoriq_tmu_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	data->little_endian = of_property_read_bool(np, "little-endian");

	data->regs = of_iomap(np, 0);
	if (!data->regs) {
		dev_err(&pdev->dev, "Failed to get memory region\n");
		ret = -ENODEV;
		goto err_iomap;
	}

	/* version register offset at: 0xbf8 on both v1 and v2 */
	ver = tmu_read(data, &data->regs->ipbrr0);
	data->ver = (ver >> 8) & 0xff;
	if (data->ver == TMU_VER2)
		data->regs_v2 = (void __iomem *)data->regs;

	qoriq_tmu_init_device(data);	/* TMU initialization */

	ret = qoriq_tmu_calibration(pdev);	/* TMU calibration */
	if (ret < 0)
		goto err_tmu;

	ret = qoriq_tmu_register_tmu_zone(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register sensors\n");
		ret = -ENODEV;
		goto err_tmu;
	}


	return 0;

err_tmu:
	iounmap(data->regs);

err_iomap:
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int qoriq_tmu_remove(struct platform_device *pdev)
{
	struct qoriq_tmu_data *data = platform_get_drvdata(pdev);

	/* Disable monitoring */
	tmu_write(data, TMR_DISABLE, &data->regs->tmr);

	iounmap(data->regs);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int __maybe_unused qoriq_tmu_suspend(struct device *dev)
{
	u32 tmr;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	/* Disable monitoring */
	tmr = tmu_read(data, &data->regs->tmr);
	tmr &= ~TMR_ME;
	tmu_write(data, tmr, &data->regs->tmr);

	return 0;
}

static int __maybe_unused qoriq_tmu_resume(struct device *dev)
{
	u32 tmr;
	struct qoriq_tmu_data *data = dev_get_drvdata(dev);

	/* Enable monitoring */
	tmr = tmu_read(data, &data->regs->tmr);
	tmr |= TMR_ME;
	tmu_write(data, tmr, &data->regs->tmr);

	return 0;
}

static SIMPLE_DEV_PM_OPS(qoriq_tmu_pm_ops,
			 qoriq_tmu_suspend, qoriq_tmu_resume);

static const struct of_device_id qoriq_tmu_match[] = {
	{ .compatible = "fsl,qoriq-tmu", },
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
	.remove	= qoriq_tmu_remove,
};
module_platform_driver(qoriq_tmu);

MODULE_AUTHOR("Jia Hongtao <hongtao.jia@nxp.com>");
MODULE_DESCRIPTION("QorIQ Thermal Monitoring Unit driver");
MODULE_LICENSE("GPL v2");
