// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 NXP
 */

#include <linux/module.h>
#include <linux/qoriq_thermal_interrupt.h>

ctd_thermal_handler_t ctd_tvd_callback;
struct platform_device *pdev_tmu;

int8_t ctd_register(ctd_thermal_handler_t ctd_cbk)
{
	if (ctd_tvd_callback == NULL) {
		ctd_tvd_callback = ctd_cbk;
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(ctd_register);

void ctd_deregister(void)
{
	ctd_tvd_callback = NULL;
}
EXPORT_SYMBOL_GPL(ctd_deregister);

int8_t ctd_program_threshold(struct ctd_thermal_threshold *thermal_threshold)
{
	int ret;

	if ((!thermal_threshold->theshold_cnt) ||
	(thermal_threshold->theshold_cnt > MAX_THRESHOLD))
		return -1;

	ret = qoriq_tmu_register_interrupt(pdev_tmu, thermal_threshold);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(ctd_program_threshold);

int ctd_program_hysteresis(int hysteresis_val)
{
	int ret;

	ret = qoriq_tmu_update_threshold(pdev_tmu, hysteresis_val);

	return ret;
}
EXPORT_SYMBOL_GPL(ctd_program_hysteresis);
