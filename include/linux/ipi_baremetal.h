// SPDX-License-Identifier: GPL-2.0+
/*
 * include/linux/ipi_baremetal.h
 *
 * Copyright 2018-2023 NXP
 *
 */

#ifndef __LINUX_IPI_BAREMETAL_H
#define __LINUX_IPI_BAREMETAL_H

#include <linux/kernel.h>

#if defined(CONFIG_LS1021A_BAREMETAL) || \
    defined(CONFIG_LS1028A_BAREMETAL) || \
    defined(CONFIG_IMX93_BAREMETAL)
#define CONFIG_MAX_CPUS 2
#elif defined(CONFIG_IMX8M_BAREMETAL)
#define CONFIG_MAX_CPUS 4
#elif defined(CONFIG_LX2160A_BAREMETAL)
#define CONFIG_MAX_CPUS 16
#else
#define CONFIG_MAX_CPUS 4
#endif

int ipi_baremetal_handle(u32 irqnr, u32 irqsrc);
#endif	/* !__LINUX_IPI_BAREMETAL_H */
