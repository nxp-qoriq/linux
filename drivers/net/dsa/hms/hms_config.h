// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#ifndef _HMS_CONFIG_H
#define _HMS_CONFIG_H

#include <net/tc_act/tc_gate.h>
#include <net/pkt_sched.h>
#include <linux/types.h>
#include <asm/types.h>

#define HMS_RT1180_DEVICE_ID		0xe001
#define HMS_NUM_PORTS			5
#define HMS_MAX_NUM_PORTS		HMS_NUM_PORTS
#define HMS_NUM_TC			8

#define HMS_ETHTOOL_STATS_NUM_MAX	120
#define HMS_QBV_LIST_MAX_ENTRIES	256

#define HMS_SPI_WORD_BITS		8
#define HMS_SPI_MSG_WORD_BYTES		4
#define HMS_SPI_MSG_HEADER_SIZE	20
#define HMS_SPI_MSG_PARAM_SIZE		16
#define HMS_SPI_MSG_MAXLEN		4096
#define HMS_SPI_MSG_RESPONSE_TIME	1000 /* us */

#define HMS_CMD_DIR_SHIFT 31
#define HMS_CMD_LEN_SHIFT 16

#define HMS_GET_MM_MAX_VERIFY_TIME	(128U)

enum  hms_spi_rw_mode {
	SPI_READ = 0,
	SPI_WRITE = 1,
};

struct hms_cmd_hdr {
	uint32_t cmd;
	uint8_t param[HMS_SPI_MSG_PARAM_SIZE];
};

/* Command */
enum hms_cmd {
	/* port related command */
	HMS_CMD_SYS_INFO_GET = 0x1,
	HMS_CMD_PORT_DSA_ADD,
	HMS_CMD_PORT_DSA_DEL,
	HMS_CMD_PORT_MTU_SET,
	HMS_CMD_PORT_MTU_GET,
	HMS_CMD_PORT_PHYLINK_MODE_SET,
	HMS_CMD_PORT_PHYLINK_STATUS_GET,
	HMS_CMD_PORT_ETHTOOL_STATS_GET,
	HMS_CMD_PORT_PVID_SET,
	HMS_CMD_PORT_LINK_SET,
	HMS_CMD_PORT_DROPUNTAG_SET,

	HMS_CMD_FDB_ADD = 0x1000,
	HMS_CMD_FDB_DEL,
	HMS_CMD_FDB_DUMP,
	HMS_CMD_VLAN_ADD,
	HMS_CMD_VLAN_DEL,
	HMS_CMD_VLAN_DUMP,
	HMS_CMD_FORWARD_MASK_SET,

	HMS_CMD_PTP_SYNC_SET = 0x2000,
	HMS_CMD_TIMER_CUR_SET,
	HMS_CMD_TIMER_CUR_GET,
	HMS_CMD_TIMER_RATE_SET,
	HMS_CMD_TIMER_RATE_GET,
	HMS_CMD_TIMER_ADJTIME_SET,
	HMS_CMD_TIMER_ADJFINE_SET,
	HMS_CMD_TIMER_PPS_START,
	HMS_CMD_TIMER_PPS_STOP,
	HMS_CMD_TIMER_EXTTS_START,
	HMS_CMD_TIMER_EXTTS_STOP,

	HMS_CMD_QBV_SET_P1 = 0x3000,
	HMS_CMD_QBV_SET_P2,
	HMS_CMD_QBV_SET_GCL,
	HMS_CMD_QBU_SET,
	HMS_CMD_MM_SET,
	HMS_CMD_MM_GET,
	HMS_CMD_QCI_SF_SET,
	HMS_CMD_QCI_SG_SET_P1,
	HMS_CMD_QCI_SG_SET_P2,
	HMS_CMD_QCI_SG_SET_GCL,
	HMS_CMD_QCI_FM_SET,
	HMS_CMD_QCI_DEL,
	HMS_CMD_QCI_GET,
	HMS_CMD_FRER_SG_SET,
	HMS_CMD_FRER_SG_DEL,
	HMS_CMD_FRER_SR_SET,
	HMS_CMD_FRER_SR_DEL,
	HMS_CMD_STREAMID_SET,
	HMS_CMD_STREAMID_DEL,
	HMS_CMD_PRIORITY_MAP_SET,

	HMS_CMD_REG_SET = 0x4000,
	HMS_CMD_REG_GET,
	HMS_CMD_MAX_NUM,
};

struct hms_cmd_sysinfo {
	uint16_t device_id;
	uint16_t vendor_id;
	uint8_t  version_major;
	uint8_t  version_minor;
	uint8_t  version_revision;
	uint8_t  cpu_port;
};

/* command data for HMS_CMD_PORT_DSA_ADD */
struct hms_cmd_port_dsa_add {
	uint8_t cpu_port; /* switch port 0, 1, 2 or 3 */
	uint8_t slave_port; /* switch port 0, 1, 2 or 3 */
	uint8_t mac_addr[ETH_ALEN]; /* MAC address of master interface */
};

/* command data for HMS_CMD_PORT_DSA_DEL */
struct hms_cmd_port_dsa_del {
	uint8_t slave_port; /* switch port 0, 1, 2 or 3 */
	uint8_t reserved[3];
};

/* command data for HMS_CMD_PORT_MTU_SET */
struct hms_cmd_port_mtu {
	uint8_t port;  /* switch port 0, 1, 2 or 3 */
	uint8_t reserved;
	uint16_t mtu;
};

/* command data for HMS_CMD_PORT_PHYLINK_MODE_SET */
struct hms_cmd_port_phylink_mode {
	uint8_t port;  /* switch port 0, 1, 2 or 3 */
	bool duplex;   /* 0: half duplex; 1: full duplex */
	uint16_t speed;   /* 10: 10Mbps ; 100: 100Mbps ; 1000: 1000Mbps */
};

/* command data for HMS_CMD_PORT_PVID_SET */
struct hms_cmd_port_pvid {
	uint8_t port; /* switch port 0, 1, 2 or 3 */
	uint8_t reserved;
	uint16_t pvid;
};

/* command data for hms_cmd_port_link */
struct hms_cmd_port_link {
	uint8_t port; /* switch port 0, 1, 2 or 3 */
	bool link; /* 0: down; 1: up */
	uint8_t reserved[2];
};

/* command data for hms_cmd_port_dropuntag */
struct hms_cmd_port_dropuntag {
	uint8_t port; /* switch port 0, 1, 2 or 3 */
	uint8_t reserved;
	uint16_t drop;
};

/* command data for HMS_CMD_FDB_ADD */
struct hms_cmd_fdb {
	uint8_t mac_addr[ETH_ALEN];
	uint16_t vid;
	uint8_t port;  /* switch port 0, 1, 2 or 3 */
	uint8_t reserved[3];
};

/* command data for HMS_CMD_VLAN_ADD */
struct hms_cmd_vlan {
	uint16_t vid;
	uint8_t port;  /* switch port 0, 1, 2 or 3 */
	bool untagged;
};

/* data returned for HMS_CMD_PORT_PHYLINK_STATUS_GET */
struct hms_cmd_port_phylink_status {
	uint8_t port;  /* switch port 0, 1, 2 or 3 */
	bool link;
	uint16_t speed;
	bool duplex; /* 0: down; 1: up */
	uint8_t reserved[3];
};

/* command param */
struct hms_cmd_read_param {
	uint32_t id;
};

/* command data for HMS_CMD_REG_SET */
struct hms_cmd_reg_cmd {
	uint32_t reg;
	uint32_t value;
};

/* data returned for HMS_CMD_FDB_DUMP */
struct hms_cmd_fdb_dump {
	uint8_t mac_addr[ETH_ALEN];
	uint16_t vid;
	/* bit 0: switch port 0 etc. */
	uint32_t port_map;
	bool dynamic;
	uint8_t reserved[3];
	/* non-zero means there are remaining entries, 0 means no more entries */
	uint32_t resume_entry_id;
};

/* data returned for HMS_CMD_VLAN_DUMP */
struct hms_cmd_vlan_dump {
	uint16_t vid;
	bool untagged;
	uint8_t reserved;
	/* bit 0: switch port 0 etc. */
	uint32_t port_map;
	/* non-zero means there are remaining entries, 0 means no more entries */
	uint32_t resume_entry_id;
};

/* command param for HMS_CMD_TIMER_PPS_START */
struct hms_cmd_timer_pps {
	uint64_t pin_start;
	uint32_t pin_duration32;
};

/* command param for HMS PTP */
struct hms_ptp_ctl_param {
	union {
		uint64_t ns;
		int64_t offset;
		int64_t ppb;
	};
	uint8_t clock_id;
};

/* stream identification */
typedef enum {
	STREAMID_RESERVED = 0,
	/* Null Stream identification */
	STREAMID_NULL,
	/* Source MAC and VLAN Stream identification */
	STREAMID_SMAC_VLAN,
	/* Active Destination MAC and VLAN stream identification */
	STREAMID_DMAC_VLAN,
	/* IP stream identification */
	STREAMID_IP,
} tsn_cb_streamid_type;

enum hms_action_type {
        HMS_STREAM_NULL,
        HMS_STREAM_FRER_SEQGEN,
        HMS_STREAM_FRER_SEQREC,
        HMS_STREAM_QCI,
};

struct hms_stream {
	struct list_head list;
	unsigned long id;
	int port_mask;
	uint8_t mac[ETH_ALEN];
	uint16_t vid;
	tsn_cb_streamid_type type;
	enum hms_action_type action;
	uint16_t handle;
	s8 prio;
	bool update;
};

typedef enum {
        HMS_SEQI_RTAG = 1,
        HMS_SEQI_HSR_SEQ_TAG,
        HMS_SEQI_PRP_SEQ_TRAILER,
} hms_encapsulation_t;

typedef enum {
        HMS_SEQR_VECTOR = 0,
        HMS_SEQR_MATCH,
} hms_seqr_algorithm_t;

struct hms_stream_seqgen {
	hms_encapsulation_t enc;
	uint8_t	iport;
};

struct hms_stream_seqrec {
	hms_encapsulation_t enc;
	hms_seqr_algorithm_t alg;
	uint16_t reset_timeout;
	uint8_t his_len;
	uint8_t rtag_pop_en;
	uint8_t	eport;
};

enum tc_frer_tag_action {
        FRER_TAG_NULL,
        FRER_TAG_PUSH,
        FRER_TAG_POP,
};

struct hms_stream_qci {
       uint32_t     maxsdu;
       int8_t priority_spec;
       struct {
	       int32_t prio;
	       uint64_t basetime;
	       uint32_t cycletime;
	       uint32_t cycletimeext;
	       uint16_t num_entries;
	       struct action_gate_entry *entries;
       } gate;
       struct {
	       uint32_t burst;
	       uint64_t rate;
       } police;
};

struct hms_stream_filter {
	struct list_head list;
	uint16_t stream_handle;
	union {
		struct hms_stream_seqgen seqgen;
		struct hms_stream_seqrec seqrec;
		struct hms_stream_qci qci;
	};
};

/* command data for HMS_CMD_STREAMID_SET */
struct hms_cmd_nullstreamid {
	uint8_t mac_addr[ETH_ALEN];
	uint16_t vid;
	uint16_t handle;
	uint8_t type;
	uint8_t port_mask;
};

/* command data for HMS_CMD_FRER_SG_SET */
struct hms_cmd_frer_sg {
	uint16_t stream_handle;
	uint8_t encap;
	uint8_t	iport;
};

/* command data for HMS_CMD_FRER_SR_SET */
struct hms_cmd_frer_sr {
	uint16_t stream_handle;
	uint16_t reset_timeout;
	uint8_t his_len;
	uint8_t encap;
	uint8_t alg;
	uint8_t rtag_pop_en;
	uint8_t	eport;
	uint8_t reserved[3];
};

struct hms_cmd_psfp_sg_p1 {
	uint64_t base_time;
	uint32_t cycle_time;
	uint16_t gcl_len;
	uint16_t index;
};

struct hms_cmd_psfp_sg_p2 {
	uint32_t cycle_time_ext;
	int32_t prio;
	uint8_t reserved[8];
};

struct hms_cmd_psfp_sgl {
	uint32_t interval;
	int32_t maxoctets;
	int32_t ipv;
	uint8_t gate_state;
	uint8_t reserved[3];
};

struct hms_cmd_psfp_fm {
	uint64_t rate;
	uint32_t burst;
	uint16_t index;
	uint8_t reserved[2];
};

struct hms_cmd_psfp_sf {
	uint32_t	maxsdu;
	uint16_t	stream_handle;
	int8_t		priority_spec;
	uint8_t		sg_enable;
	uint8_t		fm_enable;
	uint8_t		port;
	uint8_t		reserved[6];
};

struct hms_cmd_psfp_response {
	uint64_t pkts;
	uint64_t drops;
};

struct hms_cmd_priority_map {
	uint8_t port;
	uint8_t map[8];
	uint8_t reserved[7];
};

struct hms_cmd_qbv_gcl {
	uint32_t interval;
	uint16_t gate_mask;
	uint16_t operation;
};

struct hms_cmd_qbv_set_p1 {
	uint64_t base_time;
	uint32_t cycle_time;
	uint16_t gcl_len;
	uint8_t enabled;
	uint8_t port;
};

struct hms_cmd_qbv_set_p2 {
	uint32_t cycle_time_ext;
	uint8_t reserved[12];
};

struct hms_cmd_set_get_mm {
	uint32_t verify_time;
	uint32_t add_frag_size;
	uint8_t verify_enabled;
	uint8_t verify_status;
	uint8_t tx_enabled;
	uint8_t pmac_enabled;
	uint8_t tx_active;
	uint8_t port;
	uint8_t reserved[2];
};

struct hms_cmd_qbu_set {
	uint8_t preemption_mask;
	uint8_t port;
	uint8_t reserved[2];
};

struct hms_cmd_port_ethtool_stats {
	uint64_t values[HMS_ETHTOOL_STATS_NUM_MAX];
};

struct hms_mac_config {
	uint8_t port;
	uint16_t speed;
	uint16_t vlanid;
	bool link;
	bool egress;
	bool ingress;
	bool duplex;
	bool drptag;
	bool drpuntag;
	bool retag;
};

struct hms_fdb_entry {
	uint8_t mac_addr[ETH_ALEN];
	uint16_t vid;
	uint32_t port_map; /* bit 0: switch port 0 etc. */
	bool dynamic;
};

struct hms_vlan_entry {
	uint16_t vid;
	uint16_t port;
	uint32_t port_map;
	uint32_t tag_ports;
	uint32_t entry_id;
};

struct hms_config {
	uint16_t device_id;
	uint16_t vendor_id;
	uint8_t  version_major;
	uint8_t  version_minor;
	uint8_t  version_revision;
	uint8_t  cpu_port_mode;
	uint16_t tpid;
	uint16_t tpid2;
	struct hms_mac_config mac[HMS_MAX_NUM_PORTS];
	int cpu_port;
	int vlan_count;
	int vlan_max_count;
	struct hms_vlan_entry *vlan;
};

struct hms_private;

int hms_get_devinfo(struct hms_private *priv, struct hms_config *config);

int hms_port_phylink_mode_set(struct hms_private *priv,
			       struct hms_mac_config *mac);
int hms_port_phylink_stats_get(struct hms_private *priv,
				struct hms_mac_config *mac);
int hms_port_pvid_set(struct hms_private *priv, int port, uint16_t pvid);
int hms_port_link_set(struct hms_private *priv, int port, bool up);
int hms_port_dropuntag_set(struct hms_private *priv, int port, bool drop);

int hms_port_mtu_set(struct hms_private *priv, int port, int mtu);
int hms_port_mtu_get(struct hms_private *priv, int port, int *mtu);

int hms_port_pvid_set(struct hms_private *priv, int port, uint16_t pvid);

int hms_port_dsa_add(struct hms_private *priv, int cpu_port,
		      int slave_port, const unsigned char *mac_addr);
int hms_port_dsa_del(struct hms_private *priv, int slave_port);

int hms_fdb_entry_add(struct hms_private *priv,
		       const unsigned char *mac_addr,
		       uint16_t vid, int port);
int hms_fdb_entry_del(struct hms_private *priv,
		       const unsigned char *mac_addr,
		       uint16_t vid, int port);
int hms_fdb_entry_get(struct hms_private *priv,
		       struct hms_fdb_entry *fdb,
		       uint32_t entry_id, uint32_t *next_id);

int hms_vlan_entry_add(struct hms_private *priv,
			uint16_t vid, int port, bool untagged);
int hms_vlan_entry_del(struct hms_private *priv, uint16_t vid, int port);
int hms_vlan_entry_get(struct hms_private *priv,
			struct hms_vlan_entry *vlan,
			uint32_t entry_id, uint32_t *next_id);

int hms_config_setup(struct hms_config *config);
void hms_config_free(struct hms_config *config);

int hms_streamid_set(struct hms_private *priv, int port_mask, uint16_t handle,
		const unsigned char *mac, uint16_t vid, tsn_cb_streamid_type type);
int hms_streamid_del(struct hms_private *priv, uint16_t handle);

int hms_frer_seqgen(struct hms_private *priv,
		struct hms_stream_filter *filter);
int hms_frer_seqrec(struct hms_private *priv,
		struct hms_stream_filter *filter);
int hms_frer_sg_del(struct hms_private *priv, uint16_t handle, uint32_t port);
int hms_frer_sr_del(struct hms_private *priv, uint16_t handle, uint32_t port);

int hms_qci_set(struct hms_private *priv,
		struct hms_stream_filter *filter, int port);
int hms_qci_del(struct hms_private *priv,
		uint16_t handle, uint32_t port);
int hms_qci_get(struct hms_private *priv, uint16_t handle, struct flow_stats *stats);
int hms_qbv_set(struct hms_private *priv, int port, int enable,
		 struct tc_taprio_qopt_offload *taprio);
int hms_port_priority_map(struct hms_private *priv, int port, uint8_t *map);

int hms_port_phylink_status_get(struct hms_private *priv,
				struct hms_mac_config *mac);
int hms_vlan_entry_read(struct hms_private *priv,
			struct hms_vlan_entry *vlan,
			uint32_t entry_id, uint32_t *next_id);

int hms_port_set_preemptible_tcs(struct dsa_switch *ds, int port,
				 unsigned long preemptible_tcs);
int hms_port_set_mm(struct dsa_switch *ds, int port,
		    struct ethtool_mm_cfg *cfg,
		    struct netlink_ext_ack *extack);
int hms_port_get_mm(struct dsa_switch *ds, int port,
		    struct ethtool_mm_state *state);
#endif /* _HMS_CONFIG_H */
