// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2022 NXP
 */

#ifndef _QORIQ_THERMAL_INTERRUPT_
#define _QORIQ_THERMAL_INTERRUPT_

#define MAX_CTD_MONITORING_SITE 5
#define MAX_TMUv2_CTD_MONITORING_SITE 7
#define MAX_THRESHOLD 2
#define TIER_AVERAGE_THRESHOLD_ENABLED 0x40000000
#define TIER_CRITICAL_THRESHOLD_ENABLED 0x20000000
#define TIER_AVERAGE_LOW_THRESHOLD_ENABLED	0x08000000
#define TIER_CRITICAL_LOW_THRESHOLD_ENABLED	0x04000000
#define THRESHOLD_VALID 0x80000000
#define TEMP_POLLING_DEFAULT_SLEEP_TIME 5000

#define INVALID_CTD_TEMP	(0xffff)
#define MIN_CTD_TEMP	(0)
#define MAX_CTD_TEMP	(125)

/*
 * CTD Thermal event
 */
typedef enum {
	CTD_HIGH_CRITICAL_TEMP_EVENT = 1,
	CTD_HIGH_AVG_TEMP_EVENT,
	CTD_LOW_AVG_TEMP_EVENT,
	CTD_LOW_CRITICAL_TEMP_EVENT,
	CTD_MAX_EVENT,
} ctd_event_id_t;

/*
 * This structure is used to send data from CTD to TVD
 */
struct ctd_thermal_event {
	ctd_event_id_t thermal_event_id;
	u16 temp;
};

/*
 * This structure is used to configure thermal threshold
 */
struct ctd_thermal_threshold {
	int theshold_cnt;
	int threshold[MAX_THRESHOLD];
};

typedef void (*ctd_thermal_handler_t) (struct ctd_thermal_event *);
typedef void (*ctd_thermal_temp_t) (int *);

extern ctd_thermal_handler_t ctd_tvd_callback;
extern struct platform_device *pdev_tmu;
/**
 * @brief : Function to register callback to get notification for thermal
 *		event
 * @param[in] ctd_cbk : Pointer to callback function
 *
 * @return : 0 on success, -1 on failure
 */
int8_t ctd_register(ctd_thermal_handler_t ctd_cbk);
/**
 * @brief : Function to deregister callback to get notification for
 *		thermal event
 * @param : Void
 *
 * @return : Void
 */
void ctd_deregister(void);
/**
 * @brief : Function to enable interrupt and configure thermal threshold
 * @param : Pointer to the thermal threshold structure
 *
 * @return : 0 on success, -1 on failure
 */
int8_t ctd_program_threshold(
		struct ctd_thermal_threshold *thermal_threshold);
/**
 * @brief : Function to register tmu interrupt
 * @param : pdev: Pointer to devide structure
 * @param : pdev: Pointer to thermal threshold structure
 *
 * @return : void
 */
int qoriq_tmu_register_interrupt(struct platform_device *pdev,
			struct ctd_thermal_threshold *thermal_threshold);

/**
 * @brief : Function to get current temperature
 *
 * @return : Max temp of all monitoring site
 */
int ctd_get_temp(void);
/**
 * @brief : Function to update hystesis value
 * @param hysteresis_val : Value to be updated
 *
 * @return : On success return 0, else return -1
 */
int ctd_program_hysteresis(int hysteresis_val);
/**
 * @brief : Function to update CTD programmed threshold
 *
 * @return : On success return 0, else return -1
 */
int qoriq_tmu_update_threshold(struct platform_device *pdev, int hysteresis_val);
#endif
