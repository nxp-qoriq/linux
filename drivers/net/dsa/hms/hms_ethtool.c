// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#include "hms_switch.h"

enum hms_stat_index {
	/* RX stats */
	HMS_STAT_RX_BYTES,
	HMS_STAT_RX_VALID_BYTES,
	HMS_STAT_RX_PAUSE_FRAMES,
	HMS_STAT_RX_VALID_FRAMES,
	HMS_STAT_RX_VLAN_FRAMES,
	HMS_STAT_RX_UC_FRAMES,
	HMS_STAT_RX_MC_FRAMES,
	HMS_STAT_RX_BC_FRAMES,
	HMS_STAT_RX_FRAMES,
	HMS_STAT_RX_MIN_FRAMES,
	HMS_STAT_RX_64_FRAMES,
	HMS_STAT_RX_65_127_FRAMES,
	HMS_STAT_RX_128_255_FRAMES,
	HMS_STAT_RX_256_511_FRAMES,
	HMS_STAT_RX_512_1023_FRAMES,
	HMS_STAT_RX_1024_1522_FRAMES,
	HMS_STAT_RX_1523_MAX_FRAMES,
	HMS_STAT_RX_CONTROL_FRAMES,

	/* TX stats */
	HMS_STAT_TX_BYTES,
	HMS_STAT_TX_VALID_BYTES,
	HMS_STAT_TX_PAUSE_FRAMES,
	HMS_STAT_TX_VALID_FRAMES,
	HMS_STAT_TX_VLAN_FRAMES,
	HMS_STAT_TX_UC_FRAMES,
	HMS_STAT_TX_MC_FRAMES,
	HMS_STAT_TX_BC_FRAMES,
	HMS_STAT_TX_FRAMES,
	HMS_STAT_TX_MIN_FRAMES,
	HMS_STAT_TX_64_FRAMES,
	HMS_STAT_TX_65_127_FRAMES,
	HMS_STAT_TX_128_255_FRAMES,
	HMS_STAT_TX_256_511_FRAMES,
	HMS_STAT_TX_512_1023_FRAMES,
	HMS_STAT_TX_1024_1522_FRAMES,
	HMS_STAT_TX_1523_MAX_FRAMES,
	HMS_STAT_TX_CONTROL_FRAMES,

	HMS_STAT_RX_VALID_REASSEMBLED_FRAMES,
	HMS_STAT_RX_ADDITIONAL_MPACKETS,
	HMS_STAT_RX_ERROR_FRAME_REASSEMBLY,
	HMS_STAT_RX_ERROR_FRAME_SMD,
	HMS_STAT_TX_ADDITIONAL_MPACKETS,
	HMS_STAT_TX_HOLD_TRANSITIONS,

	/* Error stats */
	HMS_STAT_RX_ERROR,
	HMS_STAT_RX_ERROR_UNDERSIZE,
	HMS_STAT_RX_ERROR_OVERSIZE,
	HMS_STAT_RX_ERROR_FCS,
	HMS_STAT_RX_ERROR_FRAGMENT,
	HMS_STAT_RX_ERROR_JABBER,
	HMS_STAT_RX_ERROR_DISCARD,
	HMS_STAT_RX_ERROR_NO_TRUNCATED,
	HMS_STAT_TX_ERROR_FCS,
	HMS_STAT_TX_ERROR_UNDERSIZE,

	/* Discard stats */
	HMS_STAT_RX_DISCARD_COUNT,
	HMS_STAT_RX_DISCARD_REASON0,
	HMS_STAT_RX_DISCARD_TABLE_ID,
	HMS_STAT_RX_DISCARD_ENTRY_ID,
	HMS_STAT_TX_DISCARD_COUNT,
	HMS_STAT_TX_DISCARD_REASON0,
	HMS_STAT_TX_DISCARD_TABLE_ID,
	HMS_STAT_TX_DISCARD_ENTRY_ID,
	HMS_STAT_BRIDGE_DISCARD_COUNT,
	HMS_STAT_BRIDGE_DISCARD_REASON0,
	HMS_STAT_BRIDGE_DISCARD_TABLE_ID,
	HMS_STAT_BRIDGE_DISCARD_ENTRY_ID,

	/* Q0 stats */
	HMS_STAT_Q0_REJECTED_BYTES,
	HMS_STAT_Q0_REJECTED_FRAMES,
	HMS_STAT_Q0_DEQUEUE_BYTES,
	HMS_STAT_Q0_DEQUEUE_FRAMES,
	HMS_STAT_Q0_DROPPED_BYTES,
	HMS_STAT_Q0_DROPPED_FRAMES,
	HMS_STAT_Q0_FRAMES,

	/* Q1 stats */
	HMS_STAT_Q1_REJECTED_BYTES,
	HMS_STAT_Q1_REJECTED_FRAMES,
	HMS_STAT_Q1_DEQUEUE_BYTES,
	HMS_STAT_Q1_DEQUEUE_FRAMES,
	HMS_STAT_Q1_DROPPED_BYTES,
	HMS_STAT_Q1_DROPPED_FRAMES,
	HMS_STAT_Q1_FRAMES,

	/* Q2 stats */
	HMS_STAT_Q2_REJECTED_BYTES,
	HMS_STAT_Q2_REJECTED_FRAMES,
	HMS_STAT_Q2_DEQUEUE_BYTES,
	HMS_STAT_Q2_DEQUEUE_FRAMES,
	HMS_STAT_Q2_DROPPED_BYTES,
	HMS_STAT_Q2_DROPPED_FRAMES,
	HMS_STAT_Q2_FRAMES,

	/* Q3 stats */
	HMS_STAT_Q3_REJECTED_BYTES,
	HMS_STAT_Q3_REJECTED_FRAMES,
	HMS_STAT_Q3_DEQUEUE_BYTES,
	HMS_STAT_Q3_DEQUEUE_FRAMES,
	HMS_STAT_Q3_DROPPED_BYTES,
	HMS_STAT_Q3_DROPPED_FRAMES,
	HMS_STAT_Q3_FRAMES,

	/* Q4 stats */
	HMS_STAT_Q4_REJECTED_BYTES,
	HMS_STAT_Q4_REJECTED_FRAMES,
	HMS_STAT_Q4_DEQUEUE_BYTES,
	HMS_STAT_Q4_DEQUEUE_FRAMES,
	HMS_STAT_Q4_DROPPED_BYTES,
	HMS_STAT_Q4_DROPPED_FRAMES,
	HMS_STAT_Q4_FRAMES,

	/* Q5 stats */
	HMS_STAT_Q5_REJECTED_BYTES,
	HMS_STAT_Q5_REJECTED_FRAMES,
	HMS_STAT_Q5_DEQUEUE_BYTES,
	HMS_STAT_Q5_DEQUEUE_FRAMES,
	HMS_STAT_Q5_DROPPED_BYTES,
	HMS_STAT_Q5_DROPPED_FRAMES,
	HMS_STAT_Q5_FRAMES,

	/* Q6 stats */
	HMS_STAT_Q6_REJECTED_BYTES,
	HMS_STAT_Q6_REJECTED_FRAMES,
	HMS_STAT_Q6_DEQUEUE_BYTES,
	HMS_STAT_Q6_DEQUEUE_FRAMES,
	HMS_STAT_Q6_DROPPED_BYTES,
	HMS_STAT_Q6_DROPPED_FRAMES,
	HMS_STAT_Q6_FRAMES,

	/* Q7 stats */
	HMS_STAT_Q7_REJECTED_BYTES,
	HMS_STAT_Q7_REJECTED_FRAMES,
	HMS_STAT_Q7_DEQUEUE_BYTES,
	HMS_STAT_Q7_DEQUEUE_FRAMES,
	HMS_STAT_Q7_DROPPED_BYTES,
	HMS_STAT_Q7_DROPPED_FRAMES,
	HMS_STAT_Q7_FRAMES,
	HMS_STAT_NUM,
};

char hms_stat_name[][ETH_GSTRING_LEN] = {
	/* RX stats */
	[HMS_STAT_RX_BYTES] = "in-bytes",
	[HMS_STAT_RX_VALID_BYTES] = "in-valid-bytes",
	[HMS_STAT_RX_PAUSE_FRAMES] = "in-pause-frames",
	[HMS_STAT_RX_VALID_FRAMES] = "in-valid-frames",
	[HMS_STAT_RX_VLAN_FRAMES] = "in-vlan-frames",
	[HMS_STAT_RX_UC_FRAMES] = "in-uc-frames",
	[HMS_STAT_RX_MC_FRAMES] = "in-mc-frames",
	[HMS_STAT_RX_BC_FRAMES] = "in-bc-frames",
	[HMS_STAT_RX_FRAMES] = "in-frames",
	[HMS_STAT_RX_MIN_FRAMES] = "in-min-frames",
	[HMS_STAT_RX_64_FRAMES] = "in-64-frames",
	[HMS_STAT_RX_65_127_FRAMES] = "in-65-127-frames",
	[HMS_STAT_RX_128_255_FRAMES] = "in-128-255-frames",
	[HMS_STAT_RX_256_511_FRAMES] = "in-256-511-frames",
	[HMS_STAT_RX_512_1023_FRAMES] = "in-512-1023-frames",
	[HMS_STAT_RX_1024_1522_FRAMES] = "in-1024-1522-frames",
	[HMS_STAT_RX_1523_MAX_FRAMES] = "in-1523-max-frames",
	[HMS_STAT_RX_CONTROL_FRAMES] = "in-control-frames",

	/* TX stats */
	[HMS_STAT_TX_BYTES] = "out-bytes",
	[HMS_STAT_TX_VALID_BYTES] = "out-valid-bytes",
	[HMS_STAT_TX_PAUSE_FRAMES] = "out-pause-frames",
	[HMS_STAT_TX_VALID_FRAMES] = "out-valid-frames",
	[HMS_STAT_TX_VLAN_FRAMES] = "out-vlan-frames",
	[HMS_STAT_TX_UC_FRAMES] = "out-uc-frames",
	[HMS_STAT_TX_MC_FRAMES] = "out-mc-frames",
	[HMS_STAT_TX_BC_FRAMES] = "out-bc-frames",
	[HMS_STAT_TX_FRAMES] = "out-frames",
	[HMS_STAT_TX_MIN_FRAMES] = "out-min-frames",
	[HMS_STAT_TX_64_FRAMES] = "out-64-frames",
	[HMS_STAT_TX_65_127_FRAMES] = "out-65-127-frames",
	[HMS_STAT_TX_128_255_FRAMES] = "out-128-255-frames",
	[HMS_STAT_TX_256_511_FRAMES] = "out-256-511-frames",
	[HMS_STAT_TX_512_1023_FRAMES] = "out-512-1023-frames",
	[HMS_STAT_TX_1024_1522_FRAMES] = "out-1024-1522-frames",
	[HMS_STAT_TX_1523_MAX_FRAMES] = "out-1523-max-frames",
	[HMS_STAT_TX_CONTROL_FRAMES] = "out-control-frames",

	[HMS_STAT_RX_VALID_REASSEMBLED_FRAMES] = "in-valid-reassembled-frames",
	[HMS_STAT_RX_ADDITIONAL_MPACKETS] = "in-additional-mPackets",
	[HMS_STAT_RX_ERROR_FRAME_REASSEMBLY] = "in-error-frame-reassembly",
	[HMS_STAT_RX_ERROR_FRAME_SMD] = "in-error-frame-smd",
	[HMS_STAT_TX_ADDITIONAL_MPACKETS] = "out-additional-mPackets",
	[HMS_STAT_TX_HOLD_TRANSITIONS] = "out-hold-transitions",

	/* Error stats */
	[HMS_STAT_RX_ERROR] = "in-error",
	[HMS_STAT_RX_ERROR_UNDERSIZE] = "in-error-undersize",
	[HMS_STAT_RX_ERROR_OVERSIZE] = "in-error-oversize",
	[HMS_STAT_RX_ERROR_FCS] = "in-error-fcs",
	[HMS_STAT_RX_ERROR_FRAGMENT] = "in-error-fragment",
	[HMS_STAT_RX_ERROR_JABBER] = "in-error-jabber",
	[HMS_STAT_RX_ERROR_DISCARD] = "in-error-discard",
	[HMS_STAT_RX_ERROR_NO_TRUNCATED] = "in-error-dicard-no-truncated",
	[HMS_STAT_TX_ERROR_FCS] = "out-error-fcs",
	[HMS_STAT_TX_ERROR_UNDERSIZE] = "out-error-undersize",

	/* Discard stats */
	[HMS_STAT_RX_DISCARD_COUNT] = "in-discard-count",
	[HMS_STAT_RX_DISCARD_REASON0] = "in-discard-reason0",
	[HMS_STAT_RX_DISCARD_TABLE_ID] = "in-discard-table-id",
	[HMS_STAT_RX_DISCARD_ENTRY_ID] = "in-discard-entry-id",
	[HMS_STAT_TX_DISCARD_COUNT] = "out-discard-count",
	[HMS_STAT_TX_DISCARD_REASON0] = "out-discard-reason0",
	[HMS_STAT_TX_DISCARD_TABLE_ID] = "out-discard-table-id",
	[HMS_STAT_TX_DISCARD_ENTRY_ID] = "out-discard-entry-id",
	[HMS_STAT_BRIDGE_DISCARD_COUNT] = "bridge-discard-count",
	[HMS_STAT_BRIDGE_DISCARD_REASON0] = "bridge-discard-reason0",
	[HMS_STAT_BRIDGE_DISCARD_TABLE_ID] = "bridge-discard-table-id",
	[HMS_STAT_BRIDGE_DISCARD_ENTRY_ID] = "bridge-discard-entry-id",

	/* Q0 stats */
	[HMS_STAT_Q0_REJECTED_BYTES] = "q0-rejected-bytes",
	[HMS_STAT_Q0_REJECTED_FRAMES] = "q0-rejected-frames",
	[HMS_STAT_Q0_DEQUEUE_BYTES] = "q0-dequeue-bytes",
	[HMS_STAT_Q0_DEQUEUE_FRAMES] = "q0-dequeue-frames",
	[HMS_STAT_Q0_DROPPED_BYTES] = "q0-dropped-bytes",
	[HMS_STAT_Q0_DROPPED_FRAMES] = "q0-dropped-frames",
	[HMS_STAT_Q0_FRAMES] = "q0-frames",

	/* Q1 stats */
	[HMS_STAT_Q1_REJECTED_BYTES] = "q1-rejected-bytes",
	[HMS_STAT_Q1_REJECTED_FRAMES] = "q1-rejected-frames",
	[HMS_STAT_Q1_DEQUEUE_BYTES] = "q1-dequeue-bytes",
	[HMS_STAT_Q1_DEQUEUE_FRAMES] = "q1-dequeue-frames",
	[HMS_STAT_Q1_DROPPED_BYTES] = "q1-dropped-bytes",
	[HMS_STAT_Q1_DROPPED_FRAMES] = "q1-dropped-frames",
	[HMS_STAT_Q1_FRAMES] = "q1-frames",

	/* Q2 stats */
	[HMS_STAT_Q2_REJECTED_BYTES] = "q2-rejected-bytes",
	[HMS_STAT_Q2_REJECTED_FRAMES] = "q2-rejected-frames",
	[HMS_STAT_Q2_DEQUEUE_BYTES] = "q2-dequeue-bytes",
	[HMS_STAT_Q2_DEQUEUE_FRAMES] = "q2-dequeue-frames",
	[HMS_STAT_Q2_DROPPED_BYTES] = "q2-dropped-bytes",
	[HMS_STAT_Q2_DROPPED_FRAMES] = "q2-dropped-frames",
	[HMS_STAT_Q2_FRAMES] = "q2-frames",

	/* Q3 stats */
	[HMS_STAT_Q3_REJECTED_BYTES] = "q3-rejected-bytes",
	[HMS_STAT_Q3_REJECTED_FRAMES] = "q3-rejected-frames",
	[HMS_STAT_Q3_DEQUEUE_BYTES] = "q3-dequeue-bytes",
	[HMS_STAT_Q3_DEQUEUE_FRAMES] = "q3-dequeue-frames",
	[HMS_STAT_Q3_DROPPED_BYTES] = "q3-dropped-bytes",
	[HMS_STAT_Q3_DROPPED_FRAMES] = "q3-dropped-frames",
	[HMS_STAT_Q3_FRAMES] = "q3-frames",

	/* Q4 stats */
	[HMS_STAT_Q4_REJECTED_BYTES] = "q4-rejected-bytes",
	[HMS_STAT_Q4_REJECTED_FRAMES] = "q4-rejected-frames",
	[HMS_STAT_Q4_DEQUEUE_BYTES] = "q4-dequeue-bytes",
	[HMS_STAT_Q4_DEQUEUE_FRAMES] = "q4-dequeue-frames",
	[HMS_STAT_Q4_DROPPED_BYTES] = "q4-dropped-bytes",
	[HMS_STAT_Q4_DROPPED_FRAMES] = "q4-dropped-frames",
	[HMS_STAT_Q4_FRAMES] = "q4-frames",

	/* Q5 stats */
	[HMS_STAT_Q5_REJECTED_BYTES] = "q5-rejected-bytes",
	[HMS_STAT_Q5_REJECTED_FRAMES] = "q5-rejected-frames",
	[HMS_STAT_Q5_DEQUEUE_BYTES] = "q5-dequeue-bytes",
	[HMS_STAT_Q5_DEQUEUE_FRAMES] = "q5-dequeue-frames",
	[HMS_STAT_Q5_DROPPED_BYTES] = "q5-dropped-bytes",
	[HMS_STAT_Q5_DROPPED_FRAMES] = "q5-dropped-frames",
	[HMS_STAT_Q5_FRAMES] = "q5-frames",

	/* Q6 stats */
	[HMS_STAT_Q6_REJECTED_BYTES] = "q6-rejected-bytes",
	[HMS_STAT_Q6_REJECTED_FRAMES] = "q6-rejected-frames",
	[HMS_STAT_Q6_DEQUEUE_BYTES] = "q6-dequeue-bytes",
	[HMS_STAT_Q6_DEQUEUE_FRAMES] = "q6-dequeue-frames",
	[HMS_STAT_Q6_DROPPED_BYTES] = "q6-dropped-bytes",
	[HMS_STAT_Q6_DROPPED_FRAMES] = "q6-dropped-frames",
	[HMS_STAT_Q6_FRAMES] = "q6-frames",

	/* Q7 stats */
	[HMS_STAT_Q7_REJECTED_BYTES] = "q7-rejected-bytes",
	[HMS_STAT_Q7_REJECTED_FRAMES] = "q7-rejected-frames",
	[HMS_STAT_Q7_DEQUEUE_BYTES] = "q7-dequeue-bytes",
	[HMS_STAT_Q7_DEQUEUE_FRAMES] = "q7-dequeue-frames",
	[HMS_STAT_Q7_DROPPED_BYTES] = "q7-dropped-bytes",
	[HMS_STAT_Q7_DROPPED_FRAMES] = "q7-dropped-frames",
	[HMS_STAT_Q7_FRAMES] = "q7-frames",
};

void hms_get_ethtool_stats(struct dsa_switch *ds, int port, u64 *data)
{
	struct hms_private *priv = ds->priv;
	struct hms_cmd_port_ethtool_stats stats;
	int rc;
	enum hms_stat_index i;

	rc = hms_xfer_get_cmd(priv, HMS_CMD_PORT_ETHTOOL_STATS_GET,
			       port, &stats, sizeof(stats));

	if (rc) {
		dev_err(ds->dev,
			"Failed to get port %d stats\n", port);
		return;
	}

	for (i = 0; i < HMS_STAT_NUM; i++)
		data[i] = stats.values[i];
}

void hms_get_strings(struct dsa_switch *ds, int port,
		      u32 stringset, u8 *data)
{
	enum hms_stat_index i;
	char *p = data;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < HMS_STAT_NUM; i++) {
		strscpy(p, hms_stat_name[i], ETH_GSTRING_LEN);
		p += ETH_GSTRING_LEN;
	}
}

int hms_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return HMS_STAT_NUM;
}
