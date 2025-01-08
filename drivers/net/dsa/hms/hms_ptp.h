// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#ifndef _HMS_PTP_H
#define _HMS_PTP_H

#include <linux/timer.h>

#if IS_ENABLED(CONFIG_NET_DSA_HMS_PTP)

struct hms_ptp_data {
	struct timer_list extts_timer;
	/* Used on HMS where meta frames are generated only for
	 * 2-step TX timestamps
	 */
	struct sk_buff_head skb_txtstamp_queue;
	struct ptp_clock *clock;
	struct ptp_clock_info caps;
	/* Serializes all operations on the PTP hardware clock */
	struct mutex lock;
	bool extts_enabled;
	u64 ptpsyncts;
};

int hms_hwtstamp_set(struct dsa_switch *ds, int port, struct ifreq *ifr);

int hms_hwtstamp_get(struct dsa_switch *ds, int port, struct ifreq *ifr);

void hms_process_meta_tstamp(struct dsa_switch *ds, int port,
			      u32 ts_id, u64 tstamp);

int hms_get_ts_info(struct dsa_switch *ds, int port,
			struct kernel_ethtool_ts_info *ts);

bool hms_port_rxtstamp(struct dsa_switch *ds, int port,
			   struct sk_buff *skb, unsigned int type);

void hms_port_txtstamp(struct dsa_switch *ds, int port,
			   struct sk_buff *skb);

int hms_ptp_clock_register(struct dsa_switch *ds);

void hms_ptp_clock_unregister(struct dsa_switch *ds);


#else

struct hms_ptp_data {
	struct mutex lock;
};

static inline int hms_ptp_clock_register(struct dsa_switch *ds)
{
	return 0;
}

static inline void hms_ptp_clock_unregister(struct dsa_switch *ds)
{
}

#define hms_get_ts_info NULL

#define hms_port_rxtstamp NULL

#define hms_port_txtstamp NULL

#define hms_hwtstamp_get NULL

#define hms_hwtstamp_set NULL

#define hms_process_meta_tstamp NULL

#endif /* IS_ENABLED(CONFIG_NET_DSA_HMS_PTP) */

#endif /* _HMS_PTP_H */
