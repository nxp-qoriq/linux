// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#ifndef _HMS_H
#define _HMS_H

#include <linux/dsa/8021q.h>
#include <linux/dsa/hms.h>
#include <linux/mutex.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/timecounter.h>
#include <net/dsa.h>

#include "hms_config.h"
#include "hms_ptp.h"

struct hms_private;

enum {
	HMS_SPEED_AUTO,
	HMS_SPEED_10MBPS,
	HMS_SPEED_100MBPS,
	HMS_SPEED_1000MBPS,
	HMS_SPEED_2500MBPS,
	HMS_SPEED_MAX,
};

enum hms_internal_phy_t {
	HMS_NO_PHY = 0,
};

struct hms_info {
	const char *name;
	int device_id;
	int num_ports;
	enum dsa_tag_protocol tag_proto;
	int ptp_ts_bits;
	bool multiple_cascade_ports;
	bool can_limit_mcast_flood;
};

struct hms_psfp_list {
	struct list_head stream_list;
#define MAX_SSIDS 512
	uint16_t ssids[MAX_SSIDS];
	int num_ssids;
	/* Serialize access to the lists */
	struct mutex lock;
};

struct hms_private {
	const struct hms_info *info;
	struct hms_config config;
	int cpu_port;
	phy_interface_t phy_mode[HMS_MAX_NUM_PORTS];
	bool fixed_link[HMS_MAX_NUM_PORTS];
	unsigned long ucast_egress_floods;
	unsigned long bcast_egress_floods;

	size_t max_xfer_len;
	struct spi_device *spidev;
	struct dsa_switch *ds;
	u16 bridge_pvid[HMS_MAX_NUM_PORTS];
	u16 tag_8021q_pvid[HMS_MAX_NUM_PORTS];
	/* Serializes transmission of management frames so that
	 * the switch doesn't confuse them with one another.
	 */
	struct mutex mgmt_lock;
	/* Serializes accesses to the FDB */
	struct mutex fdb_lock;

	struct devlink_region **regions;

	/* PTP two-step TX timestamp ID, and its serialization lock */
	spinlock_t ts_id_lock;
	u32 ts_id;
	unsigned long hwts_tx_en;
	unsigned long hwts_rx_en;
	struct hms_ptp_data ptp_data;

	struct hms_psfp_list psfp;
};

int hms_vlan_filtering(struct dsa_switch *ds, int port, bool enabled,
			struct netlink_ext_ack *extack);
void hms_frame_memory_partitioning(struct hms_private *priv);

/* From hms_devlink.c */
int hms_devlink_setup(struct dsa_switch *ds);
void hms_devlink_teardown(struct dsa_switch *ds);
int hms_devlink_info_get(struct dsa_switch *ds,
			  struct devlink_info_req *req,
			  struct netlink_ext_ack *extack);

/* From hms_spi.c */
int hms_xfer_cmd(const struct hms_private *priv,
		  enum hms_spi_rw_mode rw, enum hms_cmd cmd,
		  void *param, size_t param_len,
		  void *resp, size_t resp_len,
		  struct ptp_system_timestamp *ptp_sts);
int hms_xfer_set_cmd(const struct hms_private *priv,
		      enum hms_cmd cmd,
		      void *param, size_t param_len);
int hms_xfer_get_cmd(const struct hms_private *priv,
		      enum hms_cmd cmd, uint32_t id,
		      void *resp, size_t resp_len);

int hms_xfer_write_reg(const struct hms_private *priv,
			uint32_t reg, uint32_t value);
int hms_xfer_read_reg(const struct hms_private *priv,
		       uint32_t reg, uint32_t *value);
int hms_xfer_write_u64(const struct hms_private *priv,
			enum hms_cmd cmd, uint64_t value,
			struct ptp_system_timestamp *ptp_sts);
int hms_xfer_read_u64(const struct hms_private *priv,
		       enum hms_cmd cmd, uint64_t *value,
		       struct ptp_system_timestamp *ptp_sts);

/* From hms_ethtool.c */
void hms_get_ethtool_stats(struct dsa_switch *ds, int port, uint64_t *data);
void hms_get_strings(struct dsa_switch *ds, int port,
		      uint32_t stringset, uint8_t *data);
int hms_get_sset_count(struct dsa_switch *ds, int port, int sset);

/* From hms_ptp.c */
void hms_ptp_txtstamp_skb(struct dsa_switch *ds, int port,
			   struct sk_buff *skb);

int hms_is_vlan_configured(struct hms_private *priv, uint16_t vid);
#endif /* _HMS_H */
