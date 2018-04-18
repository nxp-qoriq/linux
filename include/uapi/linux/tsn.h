/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __UAPI_GENL_TSN_H
#define __UAPI_GENL_TSN_H

#define	TSN_GENL_NAME		"TSN_GEN_CTRL"
#define	TSN_GENL_VERSION	0x1

#define MAX_USER_SIZE 0
#define MAX_ATTR_SIZE 3072
#define MAX_TOTAL_MSG_SIZE  (MAX_USER_SIZE + MAX_ATTR_SIZE)
#define MAX_ENTRY_SIZE 2048
#define MAX_ENTRY_NUMBER 128
#define MAX_IFNAME_COUNT 64

enum {
	TSN_CAP_QBV = 0x1,
	TSN_CAP_QCI = 0x2,
	TSN_CAP_QBU = 0x4,
	TSN_CAP_CBS = 0x8, /* Credit-based Shapter Qav */
	TSN_CAP_CB  = 0x10, /* 8021CB redundancy and replication */
	TSN_CAP_TBS = 0x20, /* Time Based schedule */
};

/*
 * Commands sent from userspace
 * Not versioned. New commands should only be inserted at the enum's end
 * prior to __TSN_CMD_MAX
 */

enum {
	TSN_CMD_UNSPEC = 0,	/* Reserved */
	TSN_CMD_QBV_SET,
	TSN_CMD_QBV_GET,
	TSN_CMD_QBV_GET_STATUS,
	TSN_CMD_CB_STREAMID_SET,
	TSN_CMD_CB_STREAMID_GET,
	TSN_CMD_CB_STREAMID_GET_COUNTS,
	TSN_CMD_QCI_CAP_GET, /* Qci capability get length capability get */
	TSN_CMD_QCI_SFI_SET,
	TSN_CMD_QCI_SFI_GET,
	TSN_CMD_QCI_SFI_GET_COUNTS,
	TSN_CMD_QCI_SGI_SET,
	TSN_CMD_QCI_SGI_GET,
	TSN_CMD_QCI_SGI_GET_STATUS,
	TSN_CMD_QCI_FMI_SET,
	TSN_CMD_QCI_FMI_GET,
	TSN_CMD_CBS_SET,
	TSN_CMD_CBS_GET,
	TSN_CMD_QBU_SET,
	TSN_CMD_QBU_GET_STATUS,
	TSN_CMD_QAV_SET_CBS,
	TSN_CMD_QAV_GET_CBS,
	TSN_CMD_TSD_SET,
	TSN_CMD_TSD_GET,
	TSN_CMD_ECHO,			/* user->kernel request/get-response */
	TSN_CMD_REPLY,			/* kernel->user event */
	__TSN_CMD_MAX,
};
#define TSN_CMD_MAX (__TSN_CMD_MAX - 1)


enum {
	TSN_CMD_ATTR_UNSPEC = 0,
	TSN_CMD_ATTR_MESG,		/* demo message  */
	TSN_CMD_ATTR_DATA,		/* demo data */
	TSN_ATTR_IFNAME,
	TSN_ATTR_PORT_NUMBER,
	TSN_ATTR_QBV,
	TSN_ATTR_STREAM_IDENTIFY, /* stream identify */
	TSN_ATTR_QCI_SP,		/* psfp port capbility parameters */
	TSN_ATTR_QCI_SFI,		/* psfp stream filter instance */
	TSN_ATTR_QCI_SGI,		/* psfp stream gate instance */
	TSN_ATTR_QCI_FMI,		/* psfp flow meter instance */
	TSN_ATTR_CBS,			/* credit-based shaper */
	TSN_ATTR_TSD,			/* Time Specific Departure */
	TSN_ATTR_QBU,			/* preemption */
	__TSN_CMD_ATTR_MAX,
};
#define TSN_CMD_ATTR_MAX (__TSN_CMD_ATTR_MAX - 1)

enum {
	TSN_QBU_ATTR_UNSPEC,
	TSN_QBU_ATTR_ADMIN_STATE,
	TSN_QBU_ATTR_HOLD_ADVANCE,
	TSN_QBU_ATTR_RELEASE_ADVANCE,
	TSN_QBU_ATTR_ACTIVE,
	TSN_QBU_ATTR_HOLD_REQUEST,
	__TSN_QBU_ATTR_MAX,
	TSN_QBU_ATTR_MAX = __TSN_QBU_ATTR_MAX - 1,
};

enum {
	TSN_CBS_ATTR_UNSPEC,
	TSN_CBS_ATTR_TC_INDEX,
	TSN_CBS_ATTR_BW,
	__TSN_CBS_ATTR_MAX,
	TSN_CBS_ATTR_MAX = __TSN_CBS_ATTR_MAX - 1,
};

enum {
	TSN_TSD_ATTR_UNSPEC,
	TSN_TSD_ATTR_DISABLE,
	TSN_TSD_ATTR_ENABLE,
	TSN_TSD_ATTR_PERIOD,
	TSN_TSD_ATTR_MAX_FRM_NUM,
	TSN_TSD_ATTR_CYCLE_NUM,
	TSN_TSD_ATTR_LOSS_STEPS,
	TSN_TSD_ATTR_SYN_IMME,
	__TSN_TSD_ATTR_MAX,
	TSN_TSD_ATTR_MAX = __TSN_TSD_ATTR_MAX - 1,
};

enum {
	TSN_STREAMID_ATTR_UNSPEC,
	TSN_STREAMID_ATTR_INDEX,
	TSN_STREAMID_ATTR_ENABLE,
	TSN_STREAMID_ATTR_DISABLE,
	TSN_STREAMID_ATTR_STREAM_HANDLE,
	TSN_STREAMID_ATTR_IFOP,
	TSN_STREAMID_ATTR_OFOP,
	TSN_STREAMID_ATTR_IFIP,
	TSN_STREAMID_ATTR_OFIP,
	TSN_STREAMID_ATTR_TYPE,
	TSN_STREAMID_ATTR_NDMAC,
	TSN_STREAMID_ATTR_NTAGGED,
	TSN_STREAMID_ATTR_NVID,
	TSN_STREAMID_ATTR_SMAC,
	TSN_STREAMID_ATTR_STAGGED,
	TSN_STREAMID_ATTR_SVID,
	TSN_STREAMID_ATTR_COUNTERS_PSI,
	TSN_STREAMID_ATTR_COUNTERS_PSO,
	TSN_STREAMID_ATTR_COUNTERS_PSPPI,
	TSN_STREAMID_ATTR_COUNTERS_PSPPO,
	__TSN_STREAMID_ATTR_MAX,
	TSN_STREAMID_ATTR_MAX = __TSN_STREAMID_ATTR_MAX - 1,
};

enum {
	TSN_QCI_SFI_ATTR_UNSPEC = 0,
	TSN_QCI_SFI_ATTR_INDEX,
	TSN_QCI_SFI_ATTR_ENABLE,
	TSN_QCI_SFI_ATTR_DISABLE,
	TSN_QCI_SFI_ATTR_STREAM_HANDLE,
	TSN_QCI_SFI_ATTR_PRIO_SPEC,
	TSN_QCI_SFI_ATTR_GATE_ID,
	TSN_QCI_SFI_ATTR_FILTER_TYPE,
	TSN_QCI_SFI_ATTR_FLOW_ID,
	TSN_QCI_SFI_ATTR_MAXSDU,
	TSN_QCI_SFI_ATTR_COUNTERS,
	TSN_QCI_SFI_ATTR_OVERSIZE_ENABLE,
	TSN_QCI_SFI_ATTR_OVERSIZE,
	__TSN_QCI_SFI_ATTR_MAX,
	TSN_QCI_SFI_ATTR_MAX = __TSN_QCI_SFI_ATTR_MAX - 1,
};

enum {
	TSN_QCI_SFI_ATTR_COUNTERS_UNSPEC = 0,
	TSN_QCI_SFI_ATTR_MATCH,
	TSN_QCI_SFI_ATTR_PASS,
	TSN_QCI_SFI_ATTR_DROP,
	TSN_QCI_SFI_ATTR_SDU_DROP,
	TSN_QCI_SFI_ATTR_SDU_PASS,
	TSN_QCI_SFI_ATTR_RED,
	__TSN_QCI_SFI_ATTR_COUNT_MAX,
	TSN_QCI_SFI_ATTR_COUNT_MAX = __TSN_QCI_SFI_ATTR_COUNT_MAX - 1,
};

enum {
	TSN_QCI_SGI_ATTR_UNSPEC = 0,
	TSN_QCI_SGI_ATTR_INDEX,
	TSN_QCI_SGI_ATTR_ENABLE,
	TSN_QCI_SGI_ATTR_DISABLE,
	TSN_QCI_SGI_ATTR_CONFCHANGE,
	TSN_QCI_SGI_ATTR_IRXEN,		/* Invalid rx enable*/
	TSN_QCI_SGI_ATTR_IRX,
	TSN_QCI_SGI_ATTR_OEXEN,		/* Octet exceed enable */
	TSN_QCI_SGI_ATTR_OEX,
	TSN_QCI_SGI_ATTR_ADMINENTRY,
	TSN_QCI_SGI_ATTR_OPERENTRY,
	TSN_QCI_SGI_ATTR_CCTIME,	/* config change time */
	TSN_QCI_SGI_ATTR_TICKG,
	TSN_QCI_SGI_ATTR_CUTIME,
	TSN_QCI_SGI_ATTR_CPENDING,
	TSN_QCI_SGI_ATTR_CCERROR,
	__TSN_QCI_SGI_ATTR_MAX,
	TSN_QCI_SGI_ATTR_MAX = __TSN_QCI_SGI_ATTR_MAX - 1,
};

enum {
	TSN_SGI_ATTR_CTRL_UNSPEC = 0,
	TSN_SGI_ATTR_CTRL_INITSTATE,
	TSN_SGI_ATTR_CTRL_LEN,
	TSN_SGI_ATTR_CTRL_CYTIME,
	TSN_SGI_ATTR_CTRL_CYTIMEEX,
	TSN_SGI_ATTR_CTRL_BTIME,
	TSN_SGI_ATTR_CTRL_INITIPV,
	TSN_SGI_ATTR_CTRL_GCLENTRY,
	__TSN_SGI_ATTR_CTRL_MAX,
	TSN_SGI_ATTR_CTRL_MAX = __TSN_SGI_ATTR_CTRL_MAX - 1,
};

enum {
	TSN_SGI_ATTR_GCL_UNSPEC = 0,
	TSN_SGI_ATTR_GCL_GATESTATE,
	TSN_SGI_ATTR_GCL_IPV,
	TSN_SGI_ATTR_GCL_INTERVAL,
	TSN_SGI_ATTR_GCL_OCTMAX,
	__TSN_SGI_ATTR_GCL_MAX,
	TSN_SGI_ATTR_GCL_MAX = __TSN_SGI_ATTR_GCL_MAX - 1,
};

enum {
	TSN_QCI_FMI_ATTR_UNSPEC = 0,
	TSN_QCI_FMI_ATTR_INDEX,
	TSN_QCI_FMI_ATTR_ENABLE,
	TSN_QCI_FMI_ATTR_DISABLE,
	TSN_QCI_FMI_ATTR_CIR,
	TSN_QCI_FMI_ATTR_CBS,
	TSN_QCI_FMI_ATTR_EIR,
	TSN_QCI_FMI_ATTR_EBS,
	TSN_QCI_FMI_ATTR_CF,
	TSN_QCI_FMI_ATTR_CM,
	TSN_QCI_FMI_ATTR_DROPYL,
	TSN_QCI_FMI_ATTR_MAREDEN,
	TSN_QCI_FMI_ATTR_MARED,
	__TSN_QCI_FMI_ATTR_MAX,
	TSN_QCI_FMI_ATTR_MAX = __TSN_QCI_FMI_ATTR_MAX - 1,
};

enum {
	TSN_QBV_ATTR_UNSPEC,
	TSN_QBV_ATTR_ENABLE,
	TSN_QBV_ATTR_DISABLE,
	TSN_QBV_ATTR_CONFIGCHANGE,
	TSN_QBV_ATTR_CONFIGCHANGETIME,
	TSN_QBV_ATTR_MAXSDU,
	TSN_QBV_ATTR_GRANULARITY,
	TSN_QBV_ATTR_CURRENTTIME,
	TSN_QBV_ATTR_CONFIGPENDING,
	TSN_QBV_ATTR_CONFIGCHANGEERROR,
	TSN_QBV_ATTR_ADMINENTRY,
	TSN_QBV_ATTR_OPERENTRY,
	TSN_QBV_ATTR_LISTMAX,
	__TSN_QBV_ATTR_MAX,
	TSN_QBV_ATTR_MAX = __TSN_QBV_ATTR_MAX - 1,
};

enum {
	TSN_QBV_ATTR_CTRL_UNSPEC,
	TSN_QBV_ATTR_CTRL_LISTCOUNT,
	TSN_QBV_ATTR_CTRL_GATESTATE,
	TSN_QBV_ATTR_CTRL_CYCLETIME,
	TSN_QBV_ATTR_CTRL_CYCLETIMEEXT,
	TSN_QBV_ATTR_CTRL_BASETIME,
	TSN_QBV_ATTR_CTRL_LISTENTRY,
	__TSN_QBV_ATTR_CTRL_MAX,
	TSN_QBV_ATTR_CTRL_MAX = __TSN_QBV_ATTR_CTRL_MAX - 1,
};

enum {
	TSN_QBV_ATTR_ENTRY_UNSPEC,
	TSN_QBV_ATTR_ENTRY_ID,
	TSN_QBV_ATTR_ENTRY_GC,
	TSN_QBV_ATTR_ENTRY_TM,
	__TSN_QBV_ATTR_ENTRY_MAX,
	TSN_QBV_ATTR_ENTRY_MAX = __TSN_QBV_ATTR_ENTRY_MAX - 1,
};

#define ptptime_t uint64_t

#define MAX_QUEUE_CNT 8

struct tsn_preempt_status {
	/* The value of admin_state shows a 8-bits vector value for showing
	 * the framePreemptionAdminStatus parameter and PreemptionPriority
	 * for the traffic class. Bit-7 is the highest priority traffic class
	 * and the bit-0 is the lowest priority traffic class.
	 * The bit is express (0) and is preemptable (1).
	 */
	uint8_t admin_state;
	/* The value of the holdAdvance parameter for the port in nanoseconds.
	 * There is no default value; the holdAdvance is a property of the
	 * underlying MAC." This parameter corresponds to the holdAdvance
	 * parameter in 802.1Qbu.
	 */
	uint32_t hold_advance;

	/* The value of the releaseAdvance parameter for the port in
	 * nanoseconds.  There is no default value; the releaseAdvance is a
	 * property of the underlying MAC." This parameter corresponds to the
	 * releaseAdvance parameter in 802.1Qbu.
	 */
	uint32_t release_advance;

	/* The value is active (TRUE) when preemption is operationally active
	 * for the port, and idle (FALSE) otherwise.  This parameter corresponds
	 * to the preemptionActive parameter in 802.1Qbu.
	 */
	bool preemption_active;

	/* The value is hold (1) when the sequence of gate operations for
	 * the port has executed a Set-And-Hold-MAC operation, and release
	 * (2) when the sequence of gate operations has executed a
	 * Set-And-Release-MAC operation. The value of this object is release
	 * (FALSE) on system initialization.  This parameter corresponds to the
	 * holdRequest parameter in 802.1Qbu.
	 */
	uint8_t hold_request;
};

enum tsn_tx_mode  {
	TX_MODE_STRICT,
	TX_MODE_CBS,
	TX_MODE_ETS,
	TX_MODE_VENDOR_DEFINE = 255,
};

#define QUEUE_TX_MASK ((1 << TX_MODE_STRICT) | (1 << TX_MODE_CBS) \
						| (1 << TX_MODE_ETS) | (1 << TX_MODE_VENDOR_DEFINE))

struct cbs_status {
	uint8_t delta_bw; /* percentage, 0~100 */
	uint32_t idleslope;
	int32_t sendslope;
	uint32_t maxframesize;
	uint32_t hicredit;
	int32_t locredit;
	uint32_t maxninference;
};

struct tx_queue {
	/* tx_queue_capbility shows the queue's capability mask.
	 * refer the enum tsn_tx_mode
	 */
	uint8_t capability;

	/* tx_queue_mode is current queue working mode */
	uint8_t mode;

	/* prio is showing the queue priority */
	uint8_t prio;

	/* mstat shows the status data of cbs or priority */
	union {
		struct cbs_status cbs;
	};
};

struct port_status {
	/* txqueue_cnt shows how many queues in this port */
	uint8_t queue_cnt;

	/* max_rate(Mbit/s) is the port transmit rate current port is setting */
	uint32_t max_rate;

	/* tsn_capability mask the tsn capability */
	uint32_t tsn_capability;
};

enum tsn_cb_streamid_type {
	STREAMID_RESERVED = 0,
	/* Null Stream identification */
	STREAMID_NULL,
	/* Source MAC and VLAN Stream identification */
	STREAMID_SMAC_VLAN,
	/* Active Destination MAC and VLAN stream identification */
	STREAMID_DMAC_VLAN,
	/* IP stream identification */
	STREAMID_IP,
};

/* When instantiating an instance of the Null Stream identification function 8021CB(6.4)
 * for a particular input Stream, the managed objects in the following subsections
 * serve as the tsnStreamIdParameters managed object 8021CB claus(9.1.1.7).
 */
struct tsn_cb_null_streamid {

	/* tsnCpeNullDownDestMac. Specifies the destination_address that identifies
	 * a packet in an Enhanced Internal Sublayer Service (EISS) indication primitive,
	 * to the Null Stream identification function.
	 */
	uint64_t dmac;

	/* tsnCpeNullDownTagged. It can take the following values:
	 * 1 tagged: A frame must have a VLAN tag to be recognized as belonging to the Stream.
	 * 2 priority: A frame must be untagged, or have a VLAN tag with a VLAN ID = 0 to be
	 * recognized as belonging to the Stream.
	 * 3 all: A frame is recognized as belonging to the Stream whether tagged or not.
	 */
	uint8_t tagged;

	/* tsnCpeNullDownVlan. Specifies the vlan_identifier parameter that identifies
	 * a packet in an EISS indication primitive to the Null Stream identification
	 * function. A value of 0 indicates that the vlan_identifier parameter is ignored
	 * on EISS indication primitives.
	 */
	uint16_t vid;
};

struct tsn_cb_source_streamid {
	uint64_t smac;
	uint8_t tagged;
	uint16_t vid;
};

struct tsn_cb_dest_streamid {
	uint64_t down_dmac;
	uint8_t down_tagged;
	uint16_t down_vid;
	uint8_t down_prio;
	uint64_t up_dmac;
	uint8_t up_tagged;
	uint16_t up_vid;
	uint8_t up_prio;
};

struct tsn_cb_ip_streamid {
	uint64_t dmac;
	uint8_t tagged;
	uint16_t vid;
	uint64_t siph;
	uint64_t sipl;
	uint64_t diph;
	uint64_t dipl;
	uint8_t dscp;
	uint8_t npt;
	uint16_t sport;
	uint16_t dport;
};

/* 802.1CB stream identify table clause 9.1 */
struct tsn_cb_streamid {
	/* The objects in a given entry of the Stream identity table are used to
	 * control packets whose stream_handle subparameter is equal to the entry’s
	 * tsnStreamIdHandle object.
	 */
	int32_t handle;

	/* The list of ports on which an in-facing Stream identification function
	 * in the output (towards the system forwarding function) direction
	 * Only Active Destination MAC and VLAN Stream identification (or nothing)
	 * can be configured.
	 */
	uint32_t ifac_oport;

	/* The list of ports on which an out-facing Stream identification function
	 * in the output (towards the physical interface) direction.
	 * Only Active Destination MAC and VLAN Stream identification (or nothing)
	 * can be configured.
	 */
	uint32_t ofac_oport;

	/* The list of ports on which an in-facing Stream identification function
	 * in the input (coming from the system forwarding function) direction
	 */
	uint32_t ifac_iport;

	/* The list of ports on which an out-facing Stream identification function
	 * in the input (coming from the physical interface) direction.
	 */
	uint32_t ofac_iport;

	/* An enumerated value indicating the method used to identify packets belonging
	 * to the Stream.
	 * The Organizationally Unique Identifier (OUI) or Company Identifier (CID) to
	 * identify the organization defining the enumerated type should be:
	 * 00-80-C2
	 * 1: null stream identification
	 * 2: source mac and vlan stream identification
	 * 3: activ destination mac and vlan stream identification
	 * 4: ip stream identifaciton
	 */
	uint8_t type;

	/* tsnStreamIdParameters The number of controlling parameters for a Stream
	 * identification method, their types and values, are specific to the
	 * tsnStreamIdIdentificationType
	 */
	union {
		struct tsn_cb_null_streamid nid;
		struct tsn_cb_source_streamid sid;
		struct tsn_cb_dest_streamid did;
		struct tsn_cb_ip_streamid iid;
	} para;
};

/* Following counters are instantiated for each port on which the Stream identification
 * function (6.2) is configured. The counters are indexed by port number, facing
 * (in-facing or out-facing), and stream_handle value (tsnStreamIdHandle, 9.1.1.1).
 */
struct tsn_cb_streamid_counters {
	struct {
		uint64_t input;
		uint64_t output;
	} per_stream;

	struct {
		uint64_t input;
		uint64_t output;
	} per_streamport[32];
};

/* 802.1Qci Stream Parameter Table, read from port */
struct tsn_qci_psfp_stream_param {
	/* MaxStreamFilterInstances.
	 * The maximum number of Stream Filter instances supported by this Bridge component.
	 */
	int32_t max_sf_instance;

	/* MaxStreamGateInstances
	 * The maximum number of Stream Gate instances supported by this Bridge component.
	 */
	int32_t max_sg_instance;

	/* MaxFlowMeterInstances
	 * The maximum number of Flow Meter instances supported by this Bridge component.
	 */
	int32_t max_fm_instance;

	/* SupportedListMax
	 * The maximum value supported by this Bridge component of the AdminControlListLength and
	 * OperControlListLength parameters.
	 */
	int32_t supported_list_max;
};

/* 802.1Qci Stream Filter Instance Table, counters part only. */
struct tsn_qci_psfp_sfi_counters{
	/* The MatchingFramesCount counter counts received frames that match
	 * this stream filter.
	 */
	uint64_t matching_frames_count;

	/* The PassingFramesCount counter counts received frames that pass the
	 * gate associated with this stream filter.
	 */
	uint64_t passing_frames_count;

	/* The NotPassingFramesCount counter counts received frames that do not
	 * pass the gate associated with this stream filter.
	 */
	uint64_t not_passing_frames_count;

	/* The PassingSDUCount counter counts received frames that pass the SDU
	 * size filter specification associated with this stream filter.
	 */
	uint64_t passing_sdu_count;

	/* The NotPassingSDUCount counter counts received frames that do not
	 * pass the SDU size filter specification associated with this stream
	 * filter.
	 */
	uint64_t not_passing_sdu_count;

	/* The  REDFramesCount counter counts received random early detection
	 * (RED) frames associated with this stream filter.
	 */
	uint64_t red_frames_count;
};

/* 802.1Qci Stream Filter Instance Table, configuration part only. */
struct tsn_qci_psfp_sfi_conf {

	/* The StreamHandleSpec parameter contains a stream identifier
	 * specification value. A value of -1 denotes the wild card value; zero
	 * or positive values denote stream identifier values.
	 */
	int32_t stream_handle_spec;

	/* The PrioritySpec parameter contains a priority specification value.
	 * A value of -1 denotes the wild card value; zero or positive values
	 * denote priority values.
	 */
	int8_t priority_spec;

	/* The StreamGateInstanceID parameter contains the index of an entry in
	 * the Stream Gate Table.
	 */
	uint32_t stream_gate_instance_id;

	/* The filter specifications. The actions specified in a filter specification
	 * can result in a frame passing or failing the specified filter. Frames that
	 * fail a filter are discarded.
	 */
	struct {
		/* The MaximumSDUSize parameter specifies the maximum allowed frame size
		 * for the stream. Any frame exceeding this value will be dropped.  A
		 * value of 0 denote that the MaximumSDUSize filter is disabled for this
		 * stream.
		 */
		uint16_t maximum_sdu_size;

		/* The FlowMeterInstanceID parameter contains the index of an entry in
		 * the Flow Meter Table.  A value of -1 denotes that no flow meter is
		 * assigned; zero or positive values denote flow meter IDs.
		 */
		int32_t flow_meter_instance_id;
	} stream_filter;

	/* The StreamBlockedDueToOversizeFrameEnable object contains a Boolean
	 * value that indicates whether the StreamBlockedDueToOversizeFrame
	 * function is enabled (TRUE) or disabled (FALSE).
	 */
	bool block_oversize_enable;

	/* The StreamBlockedDueToOversizeFrame object contains a Boolean value
	 * that indicates whether, if the StreamBlockedDueToOversizeFrame
	 * function is enabled, all frames are to be discarded (TRUE) or not
	 * (FALSE).
	 */
	bool block_oversize;
};

/* 802.1Qci Stream Gate Control List Entry. */
struct tsn_qci_psfp_gcl{
	/* The GateState parameter specifies a desired state, open (true) or
	 * closed (false), for the stream gate.
	 */
	bool gate_state;

	/* An IPV is encoded as a signed integer.  A negative denotes the null
	 * value; zero or positive values denote internal priority values.
	 */
	int8_t ipv;

	/* A TimeInterval is encoded in 4 octets as a 32-bit unsigned integer,
	 * representing a number of nanoseconds.
	 */
	uint32_t time_interval;

	/* The maximum number of octets that are permitted to pass the gate
	 * during the specified TimeInterval.  If zero, there is no maximum.
	 */
	uint32_t octet_max;

};

/* 802.1Qci Stream Gate Admin/Operation common list control parameters */
struct tsn_qci_sg_control {
	/* The administrative/operation value of the GateStates parameter for the stream
	 * gate.  A value of false indicates closed; a value of true indicates
	 * open.
	 */
	bool gate_states;

	/* The administrative/operation value of the ListMax parameter for the gate. The
	 * integer value indicates the number of entries (TLVs) in the
	 * AdminControlList/OperControlList.
	 */
	uint8_t control_list_length;

	/* The administrative/operation value of the CycleTime parameter for the
	 * gate.  The value is an unsigned integer number of nanoseconds.
	 */
	uint32_t cycle_time;

	/* The administrative/operation value of the CycleTimeExtension parameter for the
	 * gate.  The value is an unsigned integer number of nanoseconds.
	 */
	uint32_t cycle_time_extension;

	/* The administrative/operation value of the BaseTime parameter for the gate.  The
	 * value is a representation of a PTPtime value, consisting of a 48-bit
	 * integer number of seconds and a 32-bit integer number of nanoseconds.
	 */
	ptptime_t base_time;

	/* The administrative/operation value of the IPV parameter for the gate.  A value
	 * of -1 denotes the null value; zero or positive values denote internal
	 * priority values.
	 */
	int8_t init_ipv;

	/* control_list contend the gate control list of administrative/operation */
	struct tsn_qci_psfp_gcl *gcl;
};

/* 802.1Qci Stream Gate Instance Table, configuration part only. */
struct tsn_qci_psfp_sgi_conf {
	/* The GateEnabled parameter determines whether the stream gate is
	 * active (true) or inactive (false).
	 */
	bool gate_enabled;

	/* The ConfigChange parameter signals the start of a configuration
	 * change when it is set to TRUE. This should only be done when the
	 * various administrative parameters are all set to appropriate values.
	 */
	bool config_change;

	/* admin control parameters with admin control list */
	struct tsn_qci_sg_control admin;

	/* The GateClosedDueToInvalidRxEnable object contains a Boolean value
	 * that indicates whether the GateClosedDueToInvalidRx function is
	 * enabled (TRUE) or disabled (FALSE).
	 */
	bool block_invalid_rx_enable;

	/* The GateClosedDueToInvalidRx object contains a Boolean value that
	 * indicates whether, if the GateClosedDueToInvalidRx function is
	 * enabled, all frames are to be discarded (TRUE) or not (FALSE).
	 */
	bool block_invalid_rx;

	/* The GateClosedDueToOctetsExceededEnable object contains a Boolean
	 * value that indicates whether the GateClosedDueToOctetsExceeded
	 * function is enabled (TRUE) or disabled (FALSE).
	 */
	bool block_octets_exceeded_enable;

	/* The GateClosedDueToOctetsExceeded object contains a Boolean value
	 * that indicates whether, if the GateClosedDueToOctetsExceeded function
	 * is enabled, all frames are to be discarded (TRUE) or not (FALSE).
	 */
	bool block_octets_exceeded;
};

/* 802.1Qci Stream Gate Instance Table, status part only. */
struct tsn_psfp_sgi_status {

	/* admin control parameters with admin control list */
	struct tsn_qci_sg_control oper;

	/* The PTPtime at which the next config change is scheduled to occur.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.
	 */
	ptptime_t config_change_time;

	/* The granularity of the cycle time clock, represented as an unsigned
	 * number of tenths of nanoseconds.
	 */
	uint32_t tick_granularity;

	/* The current time, in PTPtime, as maintained by the local system.  The
	 * value is a representation of a PTPtime value, consisting of a 48-bit
	 * integer number of seconds and a 32-bit integer number of nanoseconds.
	 */
	ptptime_t current_time;

	/* The value of the ConfigPending state machine variable.  The value is
	 * TRUE if a configuration change is in progress but has not yet
	 * completed.
	 */
	bool config_pending;

	/* A counter of the number of times that a re-configuration of the
	 * traffic schedule has been requested with the old schedule still
	 * running and the requested base time was in the past.
	 */
	uint64_t config_change_error;

};

/* 802.1Qci Flow Meter Instance Table. */
struct tsn_qci_psfp_fmi {
	/* The FlowMeterCIR parameter contains an integer value that represents
	 * the CIR value for the flow meter, in kbit/s.
	 */
	uint32_t cir;

	/* The FlowMeterCBS parameter contains an integer value that represents
	 * the CBS value for the flow meter, in octets.
	 */
	uint32_t cbs;

	/* The FlowMeterEIR parameter contains an integer value that represents
	 * the EIR value for the flow meter, in kbit/s.
	 */
	uint32_t eir;

	/* The FlowMeterEBS parameter contains an integer value that represents
	 * the EBS value for the flow meter, in octets.
	 */
	uint32_t ebs;

	/* The FlowMeterCF parameter contains a Boolean value that represents
	 * the CF value for the flow meter, as a Boolean value indicating no
	 * coupling (FALSE) or coupling (TRUE).
	 */
	bool cf;

	/* The FlowMeterCM parameter contains a Boolean value that represents
	 * the CM value for the flow meter, as a Boolean value indicating
	 * colorBlind (FALSE) or colorAware (TRUE).
	 */
	bool cm;

	/* The FlowMeterDropOnYellow parameter contains a Boolean value that
	 * indicates whether yellow frames are dropped (TRUE) or have
	 * drop_eligible set to TRUE (FALSE).
	 */
	bool drop_on_yellow;

	/* The FlowMeterMarkAllFramesRedEnable parameter contains a Boolean
	 * value that indicates whether the MarkAllFramesRed function is enabled
	 * (TRUE) or disabled (FALSE).
	 */
	bool mark_red_enable;

	/* The FlowMeterMarkAllFramesRed parameter contains a Boolean value that
	 * indicates whether, if the MarkAllFramesRed function is enabled, all
	 * frames are to be discarded (TRUE) or not (FALSE).
	 */
	bool mark_red;

};

/* An entry for gate control list */
struct tsn_qbv_entry {
	/* Octet represent the gate states for the corresponding traffic classes
	 * The MS bit corresponds to traffic class 7.
	 * The LS bit to traffic class 0.
	 * A bit value of 0 indicates closed;
	 * A bit value of 1 indicates open.
	 */
	uint8_t gate_state;

	/* A TimeInterval is encoded in 4 octets as a 32-bit unsigned integer,
	 * representing a number of nanoseconds.
	 */
	uint32_t time_interval;
};

/* The administrative/operation time and gate list */
struct tsn_qbv_basic {
	/* The administrative/operation value of the GateStates parameter for the Port.
	 * The bits of the octet represent the gate states for the corresponding
	 * traffic classes; the MS bit corresponds to traffic class 7, the LS
	 * bit to traffic class 0. A bit value of 0 indicates closed; a bit
	 * value of 1 indicates open. The value of this object MUST be retained
	 * across reinitializations of the management system.
	 */
	uint8_t gate_states;

	/* The administrative/operation value of the ListMax parameter for the port. The
	 * integer value indicates the number of entries (TLVs) in the
	 * AdminControlList. The value of this object MUST be retained across
	 * reinitializations of the management system.
	 */
	uint32_t control_list_length;

	/* The administrative/operation value of the AdminCycleTime
	 * parameter for the Port. The numerator and denominator together
	 * represent the cycle time as a rational number of seconds.  The value
	 * of this object MUST be retained across reinitializations of the
	 * management system.
	 */
	uint32_t cycle_time;

	/* The administrative/operation value of the CycleTimeExtension parameter for the
	 * Port. The value is an unsigned integer number of nanoseconds. The
	 * value of this object MUST be retained across reinitializations of the
	 * management system.
	 */

	uint32_t cycle_time_extension;

	/* The administrative/operation value of the BaseTime parameter for the Port.  The
	 * value is a representation of a PTPtime value, consisting of a 48-bit
	 * integer number of seconds and a 32-bit integer number of nanoseconds.
	 * The value of this object MUST be retained across reinitializations of
	 * the management system.
	 */
	ptptime_t base_time;

	/* admin_control_list represent the AdminControlList/OperControlList.
	 * The administrative version of the gate control list for the Port.
	 */
	struct tsn_qbv_entry *control_list;
};

struct tsn_qbv_conf {
	/* The GateEnabled parameter determines whether traffic scheduling is
	 * active (true) or inactive (false).  The value of this object MUST be
	 * retained across reinitializations of the management system. */
	bool gate_enabled;

	/* The maxsdu parameter denoting the maximum SDU size supported by the queue.
	 */
	uint32_t maxsdu;

	/* The ConfigChange parameter signals the start of a configuration
	 * change when it is set to TRUE. This should only be done when the
	 * various administrative parameters are all set to appropriate values.
	 */
	bool config_change;

	/* The admin parameter signals the admin relate cycletime, basictime,
	 * gatelist paraters.
	 */
	struct tsn_qbv_basic admin;
};

/* 802.1Qbv (Time Aware Shaper) port status */
struct tsn_qbv_status {
	/* The PTPtime at which the next config change is scheduled to occur.
	 * The value is a representation of a PTPtime value, consisting of a
	 * 48-bit integer number of seconds and a 32-bit integer number of
	 * nanoseconds.  The value of this object MUST be retained across
	 * reinitializations of the management system.
	 */
	ptptime_t config_change_time;

	/* The granularity of the cycle time clock, represented as an unsigned
	 * number of tenths of nanoseconds.  The value of this object MUST be
	 * retained across reinitializations of the management system.
	 */
	uint32_t tick_granularity;

	/* The current time, in PTPtime, as maintained by the local system.  The
	 * value is a representation of a PTPtime value, consisting of a 48-bit
	 * integer number of seconds and a 32-bit integer number of nanoseconds.
	 */
	ptptime_t  current_time;

	/* The value of the ConfigPending state machine variable.  The value is
	 * TRUE if a configuration change is in progress but has not yet
	 * completed.
	 */
	bool config_pending;

	/* A counter of the number of times that a re-configuration of the
	 * traffic schedule has been requested with the old schedule still
	 * running and the requested base time was in the past.
	 */
	uint64_t config_change_error;

	/* The maximum value supported by this Port of the
	 * AdminControlListLength and OperControlListLength parameters.
	 */
	uint32_t supported_list_max;

	/* Operation settings parameters and Oper gate list */
	struct tsn_qbv_basic oper;
};

/* Time Specific Departure parameters */
struct tsn_tsd {
	bool enable;

	/* The cycle time, in units of microsecond(us)*/
	uint32_t period;

	/* The maximum number of frames which could be transmitted on one cycle.
	 *  The exceeding frames will be transmitted on next cycle.*/
	uint32_t maxFrameNum;

	/* Specify the time of the first cycle begins.
	 *      1:  begin when the queue get the first frame to transmit.
	 *      2:  begin immediately at the end of setting function.*/
	uint32_t syn_flag;
};

struct tsn_tsd_status {
	bool enable;
	uint32_t period;
	uint32_t maxFrameNum;
	uint32_t flag;
	uint32_t cycleNum;
	uint32_t loss_steps;
};

#endif /* _UAPI_GENL_TSN_H */

