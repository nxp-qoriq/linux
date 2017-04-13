/* Copyright 2013-2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the above-listed copyright holders nor the
 * names of any contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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
#ifndef __FSL_DPSW_H
#define __FSL_DPSW_H

/* Data Path L2-Switch API
 * Contains API for handling DPSW topology and functionality
 */

struct fsl_mc_io;

/**
 * DPSW general definitions
 */

/**
 * Maximum number of traffic class priorities
 */
#define DPSW_MAX_PRIORITIES	8
/**
 * Maximum number of interfaces
 */
#define DPSW_MAX_IF		64

int dpsw_open(struct fsl_mc_io *mc_io,
	      u32 cmd_flags,
	      int dpsw_id,
	      u16 *token);

int dpsw_close(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       u16 token);

/**
 * DPSW options
 */

/**
 * Disable flooding
 */
#define DPSW_OPT_FLOODING_DIS		0x0000000000000001ULL
/**
 * Disable Multicast
 */
#define DPSW_OPT_MULTICAST_DIS		0x0000000000000004ULL
/**
 * Support control interface
 */
#define DPSW_OPT_CTRL_IF_DIS		0x0000000000000010ULL
/**
 * Disable flooding metering
 */
#define DPSW_OPT_FLOODING_METERING_DIS  0x0000000000000020ULL
/**
 * Enable metering
 */
#define DPSW_OPT_METERING_EN            0x0000000000000040ULL

/**
 * enum dpsw_component_type - component type of a bridge
 * @DPSW_COMPONENT_TYPE_C_VLAN: A C-VLAN component of an
 *   enterprise VLAN bridge or of a Provider Bridge used
 *   to process C-tagged frames
 * @DPSW_COMPONENT_TYPE_S_VLAN: An S-VLAN component of a
 *   Provider Bridge
 *
 */
enum dpsw_component_type {
	DPSW_COMPONENT_TYPE_C_VLAN = 0,
	DPSW_COMPONENT_TYPE_S_VLAN
};

/**
 * struct dpsw_cfg - DPSW configuration
 * @num_ifs: Number of external and internal interfaces
 * @adv: Advanced parameters; default is all zeros;
 *		 use this structure to change default settings
 */
struct dpsw_cfg {
	u16 num_ifs;
	/**
	 * struct adv - Advanced parameters
	 * @options: Enable/Disable DPSW features (bitmap)
	 * @max_vlans: Maximum Number of VLAN's; 0 - indicates default 16
	 * @max_meters_per_if: Number of meters per interface
	 * @max_fdbs: Maximum Number of FDB's; 0 - indicates default 16
	 * @max_fdb_entries: Number of FDB entries for default FDB table;
	 *			0 - indicates default 1024 entries.
	 * @fdb_aging_time: Default FDB aging time for default FDB table;
	 *			0 - indicates default 300 seconds
	 * @max_fdb_mc_groups: Number of multicast groups in each FDB table;
	 *			0 - indicates default 32
	 * @component_type: Indicates the component type of this bridge
	 */
	struct {
		u64 options;
		u16 max_vlans;
		u8 max_meters_per_if;
		u8 max_fdbs;
		u16 max_fdb_entries;
		u16 fdb_aging_time;
		u16 max_fdb_mc_groups;
		enum dpsw_component_type component_type;
	} adv;
};

int dpsw_create(struct fsl_mc_io *mc_io,
		u16 dprc_token,
		u32 cmd_flags,
		const struct dpsw_cfg *cfg,
		u32 *obj_id);

int dpsw_destroy(struct fsl_mc_io *mc_io,
		 u16 dprc_token,
		 u32 cmd_flags,
		 u32 object_id);

int dpsw_enable(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token);

int dpsw_disable(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token);

int dpsw_is_enabled(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    int *en);

int dpsw_reset(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       u16 token);

/**
 * DPSW IRQ Index and Events
 */

#define DPSW_IRQ_INDEX_IF		0x0000
#define DPSW_IRQ_INDEX_L2SW		0x0001

/**
 * IRQ event - Indicates that the link state changed
 */
#define DPSW_IRQ_EVENT_LINK_CHANGED	0x0001

/**
 * struct dpsw_irq_cfg - IRQ configuration
 * @addr:	Address that must be written to signal a message-based interrupt
 * @val:	Value to write into irq_addr address
 * @irq_num: A user defined number associated with this IRQ
 */
struct dpsw_irq_cfg {
	     u64 addr;
	     u32 val;
	     int irq_num;
};

int dpsw_set_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 struct dpsw_irq_cfg *irq_cfg);

int dpsw_get_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 int *type,
		 struct dpsw_irq_cfg *irq_cfg);

int dpsw_set_irq_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u8 en);

int dpsw_get_irq_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u8 *en);

int dpsw_set_irq_mask(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 irq_index,
		      u32 mask);

int dpsw_get_irq_mask(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 irq_index,
		      u32 *mask);

int dpsw_get_irq_status(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u32 *status);

int dpsw_clear_irq_status(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u32 status);

/**
 * struct dpsw_attr - Structure representing DPSW attributes
 * @id: DPSW object ID
 * @options: Enable/Disable DPSW features
 * @max_vlans: Maximum Number of VLANs
 * @max_meters_per_if:  Number of meters per interface
 * @max_fdbs: Maximum Number of FDBs
 * @max_fdb_entries: Number of FDB entries for default FDB table;
 *			0 - indicates default 1024 entries.
 * @fdb_aging_time: Default FDB aging time for default FDB table;
 *			0 - indicates default 300 seconds
 * @max_fdb_mc_groups: Number of multicast groups in each FDB table;
 *			0 - indicates default 32
 * @mem_size: DPSW frame storage memory size
 * @num_ifs: Number of interfaces
 * @num_vlans: Current number of VLANs
 * @num_fdbs: Current number of FDBs
 * @component_type: Component type of this bridge
 */
struct dpsw_attr {
	int id;
	u64 options;
	u16 max_vlans;
	u8 max_meters_per_if;
	u8 max_fdbs;
	u16 max_fdb_entries;
	u16 fdb_aging_time;
	u16 max_fdb_mc_groups;
	u16 num_ifs;
	u16 mem_size;
	u16 num_vlans;
	u8 num_fdbs;
	enum dpsw_component_type component_type;
};

int dpsw_get_attributes(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			struct dpsw_attr *attr);

int dpsw_set_reflection_if(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id);

/**
 * enum dpsw_action - Action selection for special/control frames
 * @DPSW_ACTION_DROP: Drop frame
 * @DPSW_ACTION_REDIRECT: Redirect frame to control port
 */
enum dpsw_action {
	DPSW_ACTION_DROP = 0,
	DPSW_ACTION_REDIRECT = 1
};

/**
 * Enable auto-negotiation
 */
#define DPSW_LINK_OPT_AUTONEG		0x0000000000000001ULL
/**
 * Enable half-duplex mode
 */
#define DPSW_LINK_OPT_HALF_DUPLEX	0x0000000000000002ULL
/**
 * Enable pause frames
 */
#define DPSW_LINK_OPT_PAUSE		0x0000000000000004ULL
/**
 * Enable a-symmetric pause frames
 */
#define DPSW_LINK_OPT_ASYM_PAUSE	0x0000000000000008ULL

/**
 * struct dpsw_link_cfg - Structure representing DPSW link configuration
 * @rate: Rate
 * @options: Mask of available options; use 'DPSW_LINK_OPT_<X>' values
 */
struct dpsw_link_cfg {
	u32 rate;
	u64 options;
};

int dpsw_if_set_link_cfg(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 struct dpsw_link_cfg *cfg);
/**
 * struct dpsw_link_state - Structure representing DPSW link state
 * @rate: Rate
 * @options: Mask of available options; use 'DPSW_LINK_OPT_<X>' values
 * @up: 0 - covers two cases: down and disconnected, 1 - up
 */
struct dpsw_link_state {
	u32 rate;
	u64 options;
	int up;
};

int dpsw_if_get_link_state(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_link_state *state);

int dpsw_if_set_flooding(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 int en);

int dpsw_if_set_broadcast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en);

int dpsw_if_set_multicast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en);

/**
 * struct dpsw_tci_cfg - Tag Contorl Information (TCI) configuration
 * @pcp: Priority Code Point (PCP): a 3-bit field which refers
 *		 to the IEEE 802.1p priority
 * @dei: Drop Eligible Indicator (DEI): a 1-bit field. May be used
 *		 separately or in conjunction with PCP to indicate frames
 *		 eligible to be dropped in the presence of congestion
 * @vlan_id: VLAN Identifier (VID): a 12-bit field specifying the VLAN
 *			to which the frame belongs. The hexadecimal values
 *			of 0x000 and 0xFFF are reserved;
 *			all other values may be used as VLAN identifiers,
 *			allowing up to 4,094 VLANs
 */
struct dpsw_tci_cfg {
	u8 pcp;
	u8 dei;
	u16 vlan_id;
};

int dpsw_if_set_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_tci_cfg *cfg);

int dpsw_if_get_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    struct dpsw_tci_cfg *cfg);

/**
 * enum dpsw_stp_state - Spanning Tree Protocol (STP) states
 * @DPSW_STP_STATE_BLOCKING: Blocking state
 * @DPSW_STP_STATE_LISTENING: Listening state
 * @DPSW_STP_STATE_LEARNING: Learning state
 * @DPSW_STP_STATE_FORWARDING: Forwarding state
 *
 */
enum dpsw_stp_state {
	DPSW_STP_STATE_BLOCKING = 0,
	DPSW_STP_STATE_LISTENING = 1,
	DPSW_STP_STATE_LEARNING = 2,
	DPSW_STP_STATE_FORWARDING = 3
};

/**
 * struct dpsw_stp_cfg - Spanning Tree Protocol (STP) Configuration
 * @vlan_id: VLAN ID STP state
 * @state: STP state
 */
struct dpsw_stp_cfg {
	u16 vlan_id;
	enum dpsw_stp_state state;
};

int dpsw_if_set_stp(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_stp_cfg *cfg);

/**
 * enum dpsw_accepted_frames - Types of frames to accept
 * @DPSW_ADMIT_ALL: The device accepts VLAN tagged, untagged and
 *			priority tagged frames
 * @DPSW_ADMIT_ONLY_VLAN_TAGGED: The device discards untagged frames or
 *			Priority-Tagged frames received on this interface.
 *
 */
enum dpsw_accepted_frames {
	DPSW_ADMIT_ALL = 1,
	DPSW_ADMIT_ONLY_VLAN_TAGGED = 3
};

/**
 * struct dpsw_accepted_frames_cfg - Types of frames to accept configuration
 * @type: Defines ingress accepted frames
 * @unaccept_act: When a frame is not accepted, it may be discarded or
 *			redirected to control interface depending on this mode
 */
struct dpsw_accepted_frames_cfg {
	enum dpsw_accepted_frames type;
	enum dpsw_action unaccept_act;
};

int dpsw_if_set_accepted_frames(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				const struct dpsw_accepted_frames_cfg *cfg);

int dpsw_if_set_accept_all_vlan(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				int accept_all);

/**
 * enum dpsw_counter  - Counters types
 * @DPSW_CNT_ING_FRAME: Counts ingress frames
 * @DPSW_CNT_ING_BYTE: Counts ingress bytes
 * @DPSW_CNT_ING_FLTR_FRAME: Counts filtered ingress frames
 * @DPSW_CNT_ING_FRAME_DISCARD: Counts discarded ingress frame
 * @DPSW_CNT_ING_MCAST_FRAME: Counts ingress multicast frames
 * @DPSW_CNT_ING_MCAST_BYTE: Counts ingress multicast bytes
 * @DPSW_CNT_ING_BCAST_FRAME: Counts ingress broadcast frames
 * @DPSW_CNT_ING_BCAST_BYTES: Counts ingress broadcast bytes
 * @DPSW_CNT_EGR_FRAME: Counts egress frames
 * @DPSW_CNT_EGR_BYTE: Counts eEgress bytes
 * @DPSW_CNT_EGR_FRAME_DISCARD: Counts discarded egress frames
 * @DPSW_CNT_EGR_STP_FRAME_DISCARD: Counts egress STP discarded frames
 */
enum dpsw_counter {
	DPSW_CNT_ING_FRAME = 0x0,
	DPSW_CNT_ING_BYTE = 0x1,
	DPSW_CNT_ING_FLTR_FRAME = 0x2,
	DPSW_CNT_ING_FRAME_DISCARD = 0x3,
	DPSW_CNT_ING_MCAST_FRAME = 0x4,
	DPSW_CNT_ING_MCAST_BYTE = 0x5,
	DPSW_CNT_ING_BCAST_FRAME = 0x6,
	DPSW_CNT_ING_BCAST_BYTES = 0x7,
	DPSW_CNT_EGR_FRAME = 0x8,
	DPSW_CNT_EGR_BYTE = 0x9,
	DPSW_CNT_EGR_FRAME_DISCARD = 0xa,
	DPSW_CNT_EGR_STP_FRAME_DISCARD = 0xb
};

int dpsw_if_get_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 *counter);

int dpsw_if_set_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 counter);

/**
 * Maximum number of TC
 */
#define DPSW_MAX_TC             8

/**
 * enum dpsw_priority_selector - User priority
 * @DPSW_UP_PCP: Priority Code Point (PCP): a 3-bit field which
 *				 refers to the IEEE 802.1p priority.
 * @DPSW_UP_DSCP: Differentiated services Code Point (DSCP): 6 bit
 *				field from IP header
 *
 */
enum dpsw_priority_selector {
	DPSW_UP_PCP = 0,
	DPSW_UP_DSCP = 1
};

/**
 * enum dpsw_schedule_mode - Traffic classes scheduling
 * @DPSW_SCHED_STRICT_PRIORITY: schedule strict priority
 * @DPSW_SCHED_WEIGHTED: schedule based on token bucket created algorithm
 */
enum dpsw_schedule_mode {
	DPSW_SCHED_STRICT_PRIORITY,
	DPSW_SCHED_WEIGHTED
};

/**
 * struct dpsw_tx_schedule_cfg - traffic class configuration
 * @mode: Strict or weight-based scheduling
 * @delta_bandwidth: weighted Bandwidth in range from 100 to 10000
 */
struct dpsw_tx_schedule_cfg {
	enum dpsw_schedule_mode mode;
	u16 delta_bandwidth;
};

/**
 * struct dpsw_tx_selection_cfg - Mapping user priority into traffic
 *					class configuration
 * @priority_selector: Source for user priority regeneration
 * @tc_id: The Regenerated User priority that the incoming
 *				User Priority is mapped to for this interface
 * @tc_sched: Traffic classes configuration
 */
struct dpsw_tx_selection_cfg {
	enum dpsw_priority_selector priority_selector;
	u8 tc_id[DPSW_MAX_PRIORITIES];
	struct dpsw_tx_schedule_cfg tc_sched[DPSW_MAX_TC];
};

int dpsw_if_set_tx_selection(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     const struct dpsw_tx_selection_cfg *cfg);

/**
 * enum dpsw_reflection_filter - Filter type for frames to reflect
 * @DPSW_REFLECTION_FILTER_INGRESS_ALL: Reflect all frames
 * @DPSW_REFLECTION_FILTER_INGRESS_VLAN: Reflect only frames belong to
 *			particular VLAN defined by vid parameter
 *
 */
enum dpsw_reflection_filter {
	DPSW_REFLECTION_FILTER_INGRESS_ALL = 0,
	DPSW_REFLECTION_FILTER_INGRESS_VLAN = 1
};

/**
 * struct dpsw_reflection_cfg - Structure representing reflection information
 * @filter: Filter type for frames to reflect
 * @vlan_id: Vlan Id to reflect; valid only when filter type is
 *		DPSW_INGRESS_VLAN
 */
struct dpsw_reflection_cfg {
	enum dpsw_reflection_filter filter;
	u16 vlan_id;
};

int dpsw_if_add_reflection(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   const struct dpsw_reflection_cfg *cfg);

int dpsw_if_remove_reflection(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 if_id,
			      const struct dpsw_reflection_cfg *cfg);

/**
 * enum dpsw_metering_mode - Metering modes
 * @DPSW_METERING_MODE_NONE: metering disabled
 * @DPSW_METERING_MODE_RFC2698: RFC 2698
 * @DPSW_METERING_MODE_RFC4115: RFC 4115
 */
enum dpsw_metering_mode {
	DPSW_METERING_MODE_NONE = 0,
	DPSW_METERING_MODE_RFC2698,
	DPSW_METERING_MODE_RFC4115
};

/**
 * enum dpsw_metering_unit - Metering count
 * @DPSW_METERING_UNIT_BYTES: count bytes
 * @DPSW_METERING_UNIT_FRAMES: count frames
 */
enum dpsw_metering_unit {
	DPSW_METERING_UNIT_BYTES = 0,
	DPSW_METERING_UNIT_FRAMES
};

/**
 * struct dpsw_metering_cfg - Metering configuration
 * @mode: metering modes
 * @units: Bytes or frame units
 * @cir: Committed information rate (CIR) in Kbits/s
 * @eir: Peak information rate (PIR) Kbit/s  rfc2698
 *	 Excess information rate (EIR) Kbit/s rfc4115
 * @cbs: Committed burst size (CBS) in bytes
 * @ebs: Peak burst size (PBS) in bytes for rfc2698
 *       Excess bust size (EBS) in bytes rfc4115
 *
 */
struct dpsw_metering_cfg {
	enum dpsw_metering_mode mode;
	enum dpsw_metering_unit units;
	u32 cir;
	u32 eir;
	u32 cbs;
	u32 ebs;
};

int dpsw_if_set_flooding_metering(struct fsl_mc_io *mc_io,
				  u32 cmd_flags,
				  u16 token,
				  u16 if_id,
				  const struct dpsw_metering_cfg *cfg);

int dpsw_if_set_metering(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 u8 tc_id,
			 const struct dpsw_metering_cfg *cfg);

/**
 * enum dpsw_early_drop_unit - DPSW early drop unit
 * @DPSW_EARLY_DROP_UNIT_BYTE: count bytes
 * @DPSW_EARLY_DROP_UNIT_FRAMES: count frames
 */
enum dpsw_early_drop_unit {
	DPSW_EARLY_DROP_UNIT_BYTE = 0,
	DPSW_EARLY_DROP_UNIT_FRAMES
};

/**
 * enum dpsw_early_drop_mode - DPSW early drop mode
 * @DPSW_EARLY_DROP_MODE_NONE: early drop is disabled
 * @DPSW_EARLY_DROP_MODE_TAIL: early drop in taildrop mode
 * @DPSW_EARLY_DROP_MODE_WRED: early drop in WRED mode
 */
enum dpsw_early_drop_mode {
	DPSW_EARLY_DROP_MODE_NONE = 0,
	DPSW_EARLY_DROP_MODE_TAIL,
	DPSW_EARLY_DROP_MODE_WRED
};

/**
 * struct dpsw_wred_cfg - WRED configuration
 * @max_threshold: maximum threshold that packets may be discarded. Above this
 *	  threshold all packets are discarded; must be less than 2^39;
 *	  approximated to be expressed as (x+256)*2^(y-1) due to HW
 *	    implementation.
 * @min_threshold: minimum threshold that packets may be discarded at
 * @drop_probability: probability that a packet will be discarded (1-100,
 *	associated with the maximum threshold)
 */
struct dpsw_wred_cfg {
	u64 min_threshold;
	u64 max_threshold;
	u8 drop_probability;
};

/**
 * struct dpsw_early_drop_cfg - early-drop configuration
 * @drop_mode: drop mode
 * @units: count units
 * @yellow: WRED - 'yellow' configuration
 * @green: WRED - 'green' configuration
 * @tail_drop_threshold: tail drop threshold
 */
struct dpsw_early_drop_cfg {
	enum dpsw_early_drop_mode drop_mode;
	enum dpsw_early_drop_unit units;
	struct dpsw_wred_cfg yellow;
	struct dpsw_wred_cfg green;
	u32 tail_drop_threshold;
};

void dpsw_prepare_early_drop(const struct dpsw_early_drop_cfg *cfg,
			     u8 *early_drop_buf);

int dpsw_if_set_early_drop(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   u8 tc_id,
			   u64 early_drop_iova);

/**
 * struct dpsw_custom_tpid_cfg - Structure representing tag Protocol identifier
 * @tpid: An additional tag protocol identifier
 */
struct dpsw_custom_tpid_cfg {
	u16 tpid;
};

int dpsw_add_custom_tpid(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 const struct dpsw_custom_tpid_cfg *cfg);

int dpsw_remove_custom_tpid(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    const struct dpsw_custom_tpid_cfg *cfg);

int dpsw_if_enable(struct fsl_mc_io *mc_io,
		   u32 cmd_flags,
		   u16 token,
		   u16 if_id);

int dpsw_if_disable(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id);

/**
 * struct dpsw_if_attr - Structure representing DPSW interface attributes
 * @num_tcs: Number of traffic classes
 * @rate: Transmit rate in bits per second
 * @options: Interface configuration options (bitmap)
 * @enabled: Indicates if interface is enabled
 * @accept_all_vlan: The device discards/accepts incoming frames
 *		for VLANs that do not include this interface
 * @admit_untagged: When set to 'DPSW_ADMIT_ONLY_VLAN_TAGGED', the device
 *		discards untagged frames or priority-tagged frames received on
 *		this interface;
 *		When set to 'DPSW_ADMIT_ALL', untagged frames or priority-
 *		tagged frames received on this interface are accepted
 * @qdid: control frames transmit qdid
 */
struct dpsw_if_attr {
	u8 num_tcs;
	u32 rate;
	u32 options;
	int enabled;
	int accept_all_vlan;
	enum dpsw_accepted_frames admit_untagged;
	u16 qdid;
};

int dpsw_if_get_attributes(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_if_attr *attr);

int dpsw_if_set_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 frame_length);

int dpsw_if_get_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 *frame_length);

/**
 * struct dpsw_vlan_cfg - VLAN Configuration
 * @fdb_id: Forwarding Data Base
 */
struct dpsw_vlan_cfg {
	u16 fdb_id;
};

int dpsw_vlan_add(struct fsl_mc_io *mc_io,
		  u32 cmd_flags,
		  u16 token,
		  u16 vlan_id,
		  const struct dpsw_vlan_cfg *cfg);

/**
 * struct dpsw_vlan_if_cfg - Set of VLAN Interfaces
 * @num_ifs: The number of interfaces that are assigned to the egress
 *		list for this VLAN
 * @if_id: The set of interfaces that are
 *		assigned to the egress list for this VLAN
 */
struct dpsw_vlan_if_cfg {
	u16 num_ifs;
	u16 if_id[DPSW_MAX_IF];
};

int dpsw_vlan_add_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_add_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_add_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_remove_if(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 vlan_id,
			const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_remove_if_untagged(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_remove_if_flooding(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_remove(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id);

/**
 * struct dpsw_vlan_attr - VLAN attributes
 * @fdb_id: Associated FDB ID
 * @num_ifs: Number of interfaces
 * @num_untagged_ifs: Number of untagged interfaces
 * @num_flooding_ifs: Number of flooding interfaces
 */
struct dpsw_vlan_attr {
	u16 fdb_id;
	u16 num_ifs;
	u16 num_untagged_ifs;
	u16 num_flooding_ifs;
};

int dpsw_vlan_get_attributes(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 vlan_id,
			     struct dpsw_vlan_attr *attr);

int dpsw_vlan_get_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_get_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg);

int dpsw_vlan_get_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg);

/**
 * struct dpsw_fdb_cfg  - FDB Configuration
 * @num_fdb_entries: Number of FDB entries
 * @fdb_aging_time: Aging time in seconds
 */
struct dpsw_fdb_cfg {
	u16 num_fdb_entries;
	u16 fdb_aging_time;
};

int dpsw_fdb_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *fdb_id,
		 const struct dpsw_fdb_cfg *cfg);

int dpsw_fdb_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 fdb_id);

/**
 * enum dpsw_fdb_entry_type - FDB Entry type - Static/Dynamic
 * @DPSW_FDB_ENTRY_STATIC: Static entry
 * @DPSW_FDB_ENTRY_DINAMIC: Dynamic entry
 */
enum dpsw_fdb_entry_type {
	DPSW_FDB_ENTRY_STATIC = 0,
	DPSW_FDB_ENTRY_DINAMIC = 1
};

/**
 * struct dpsw_fdb_unicast_cfg - Unicast entry configuration
 * @type: Select static or dynamic entry
 * @mac_addr: MAC address
 * @if_egress: Egress interface ID
 */
struct dpsw_fdb_unicast_cfg {
	enum dpsw_fdb_entry_type type;
	u8 mac_addr[6];
	u16 if_egress;
};

int dpsw_fdb_add_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 const struct dpsw_fdb_unicast_cfg *cfg);

int dpsw_fdb_get_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 struct dpsw_fdb_unicast_cfg *cfg);

int dpsw_fdb_remove_unicast(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    const struct dpsw_fdb_unicast_cfg *cfg);

/**
 * struct dpsw_fdb_multicast_cfg - Multi-cast entry configuration
 * @type: Select static or dynamic entry
 * @mac_addr: MAC address
 * @num_ifs: Number of external and internal interfaces
 * @if_id: Egress interface IDs
 */
struct dpsw_fdb_multicast_cfg {
	enum dpsw_fdb_entry_type type;
	u8 mac_addr[6];
	u16 num_ifs;
	u16 if_id[DPSW_MAX_IF];
};

int dpsw_fdb_add_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   const struct dpsw_fdb_multicast_cfg *cfg);

int dpsw_fdb_get_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   struct dpsw_fdb_multicast_cfg *cfg);

int dpsw_fdb_remove_multicast(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 fdb_id,
			      const struct dpsw_fdb_multicast_cfg *cfg);

/**
 * enum dpsw_fdb_learning_mode - Auto-learning modes
 * @DPSW_FDB_LEARNING_MODE_DIS: Disable Auto-learning
 * @DPSW_FDB_LEARNING_MODE_HW: Enable HW auto-Learning
 * @DPSW_FDB_LEARNING_MODE_NON_SECURE: Enable None secure learning by CPU
 * @DPSW_FDB_LEARNING_MODE_SECURE: Enable secure learning by CPU
 *
 *	NONE - SECURE LEARNING
 *	SMAC found	DMAC found	CTLU Action
 *	v		v	Forward frame to
 *						1.  DMAC destination
 *	-		v	Forward frame to
 *						1.  DMAC destination
 *						2.  Control interface
 *	v		-	Forward frame to
 *						1.  Flooding list of interfaces
 *	-		-	Forward frame to
 *						1.  Flooding list of interfaces
 *						2.  Control interface
 *	SECURE LEARING
 *	SMAC found	DMAC found	CTLU Action
 *	v		v		Forward frame to
 *						1.  DMAC destination
 *	-		v		Forward frame to
 *						1.  Control interface
 *	v		-		Forward frame to
 *						1.  Flooding list of interfaces
 *	-		-		Forward frame to
 *						1.  Control interface
 */
enum dpsw_fdb_learning_mode {
	DPSW_FDB_LEARNING_MODE_DIS = 0,
	DPSW_FDB_LEARNING_MODE_HW = 1,
	DPSW_FDB_LEARNING_MODE_NON_SECURE = 2,
	DPSW_FDB_LEARNING_MODE_SECURE = 3
};

int dpsw_fdb_set_learning_mode(struct fsl_mc_io *mc_io,
			       u32 cmd_flags,
			       u16 token,
			       u16 fdb_id,
			       enum dpsw_fdb_learning_mode mode);

/**
 * struct dpsw_fdb_attr - FDB Attributes
 * @max_fdb_entries: Number of FDB entries
 * @fdb_aging_time: Aging time in seconds
 * @learning_mode: Learning mode
 * @num_fdb_mc_groups: Current number of multicast groups
 * @max_fdb_mc_groups: Maximum number of multicast groups
 */
struct dpsw_fdb_attr {
	u16 max_fdb_entries;
	u16 fdb_aging_time;
	enum dpsw_fdb_learning_mode learning_mode;
	u16 num_fdb_mc_groups;
	u16 max_fdb_mc_groups;
};

int dpsw_fdb_get_attributes(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    struct dpsw_fdb_attr *attr);

/**
 * struct dpsw_acl_cfg - ACL Configuration
 * @max_entries: Number of FDB entries
 */
struct dpsw_acl_cfg {
	u16 max_entries;
};

/**
 * struct dpsw_acl_fields - ACL fields.
 * @l2_dest_mac: Destination MAC address: BPDU, Multicast, Broadcast, Unicast,
 *			slow protocols, MVRP, STP
 * @l2_source_mac: Source MAC address
 * @l2_tpid: Layer 2 (Ethernet) protocol type, used to identify the following
 *		protocols: MPLS, PTP, PFC, ARP, Jumbo frames, LLDP, IEEE802.1ae,
 *		Q-in-Q, IPv4, IPv6, PPPoE
 * @l2_pcp_dei: indicate which protocol is encapsulated in the payload
 * @l2_vlan_id: layer 2 VLAN ID
 * @l2_ether_type: layer 2 Ethernet type
 * @l3_dscp: Layer 3 differentiated services code point
 * @l3_protocol: Tells the Network layer at the destination host, to which
 *		Protocol this packet belongs to. The following protocol are
 *		supported: ICMP, IGMP, IPv4 (encapsulation), TCP, IPv6
 *		(encapsulation), GRE, PTP
 * @l3_source_ip: Source IPv4 IP
 * @l3_dest_ip: Destination IPv4 IP
 * @l4_source_port: Source TCP/UDP Port
 * @l4_dest_port: Destination TCP/UDP Port
 */
struct dpsw_acl_fields {
	u8 l2_dest_mac[6];
	u8 l2_source_mac[6];
	u16 l2_tpid;
	u8 l2_pcp_dei;
	u16 l2_vlan_id;
	u16 l2_ether_type;
	u8 l3_dscp;
	u8 l3_protocol;
	u32 l3_source_ip;
	u32 l3_dest_ip;
	u16 l4_source_port;
	u16 l4_dest_port;
};

/**
 * struct dpsw_acl_key - ACL key
 * @match: Match fields
 * @mask: Mask: b'1 - valid, b'0 don't care
 */
struct dpsw_acl_key {
	struct dpsw_acl_fields match;
	struct dpsw_acl_fields mask;
};

/**
 * enum dpsw_acl_action
 * @DPSW_ACL_ACTION_DROP: Drop frame
 * @DPSW_ACL_ACTION_REDIRECT: Redirect to certain port
 * @DPSW_ACL_ACTION_ACCEPT: Accept frame
 * @DPSW_ACL_ACTION_REDIRECT_TO_CTRL_IF: Redirect to control interface
 */
enum dpsw_acl_action {
	DPSW_ACL_ACTION_DROP,
	DPSW_ACL_ACTION_REDIRECT,
	DPSW_ACL_ACTION_ACCEPT,
	DPSW_ACL_ACTION_REDIRECT_TO_CTRL_IF
};

/**
 * struct dpsw_acl_result - ACL action
 * @action: Action should be taken when	ACL entry hit
 * @if_id:  Interface IDs to redirect frame. Valid only if redirect selected for
 *		 action
 */
struct dpsw_acl_result {
	enum dpsw_acl_action action;
	u16 if_id;
};

/**
 * struct dpsw_acl_entry_cfg - ACL entry
 * @key_iova: I/O virtual address of DMA-able memory filled with key after call
 *				to dpsw_acl_prepare_entry_cfg()
 * @result: Required action when entry hit occurs
 * @precedence: Precedence inside ACL 0 is lowest; This priority can not change
 *		during the lifetime of a Policy. It is user responsibility to
 *		space the priorities according to consequent rule additions.
 */
struct dpsw_acl_entry_cfg {
	u64 key_iova;
	struct dpsw_acl_result result;
	int precedence;
};

int dpsw_acl_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *acl_id,
		 const struct dpsw_acl_cfg *cfg);

int dpsw_acl_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id);

void dpsw_acl_prepare_entry_cfg(const struct dpsw_acl_key *key,
				uint8_t *entry_cfg_buf);

int dpsw_acl_add_entry(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_entry_cfg *cfg);

int dpsw_acl_remove_entry(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 acl_id,
			  const struct dpsw_acl_entry_cfg *cfg);

/**
 * struct dpsw_acl_if_cfg - List of interfaces to Associate with ACL
 * @num_ifs: Number of interfaces
 * @if_id: List of interfaces
 */
struct dpsw_acl_if_cfg {
	u16 num_ifs;
	u16 if_id[DPSW_MAX_IF];
};

int dpsw_acl_add_if(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id,
		    const struct dpsw_acl_if_cfg *cfg);

int dpsw_acl_remove_if(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_if_cfg *cfg);

/**
 * struct dpsw_acl_attr -  ACL Attributes
 * @max_entries: Max number of ACL entries
 * @num_entries: Number of used ACL entries
 * @num_ifs: Number of interfaces associated with ACL
 */
struct dpsw_acl_attr {
	u16 max_entries;
	u16 num_entries;
	u16 num_ifs;
};

int dpsw_acl_get_attributes(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 acl_id,
			    struct dpsw_acl_attr *attr);
/**
 * struct dpsw_ctrl_if_attr - Control interface attributes
 * @rx_fqid:		Receive FQID
 * @rx_err_fqid:		Receive error FQID
 * @tx_err_conf_fqid:	Transmit error and confirmation FQID
 */
struct dpsw_ctrl_if_attr {
	u32 rx_fqid;
	u32 rx_err_fqid;
	u32 tx_err_conf_fqid;
};

int dpsw_ctrl_if_get_attributes(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				struct dpsw_ctrl_if_attr *attr);

/**
 * Maximum number of DPBP
 */
#define DPSW_MAX_DPBP     8

/**
 * struct dpsw_ctrl_if_pools_cfg - Control interface buffer pools configuration
 * @num_dpbp: Number of DPBPs
 * @pools: Array of buffer pools parameters; The number of valid entries
 *	must match 'num_dpbp' value
 */
struct dpsw_ctrl_if_pools_cfg {
	u8 num_dpbp;
	/**
	 * struct pools - Buffer pools parameters
	 * @dpbp_id: DPBP object ID
	 * @buffer_size: Buffer size
	 * @backup_pool: Backup pool
	 */
	struct {
		int dpbp_id;
		u16 buffer_size;
		int backup_pool;
	} pools[DPSW_MAX_DPBP];
};

int dpsw_ctrl_if_set_pools(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   const struct dpsw_ctrl_if_pools_cfg *cfg);

int dpsw_ctrl_if_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token);

int dpsw_ctrl_if_disable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token);

int dpsw_get_api_version(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 *major_ver,
			 u16 *minor_ver);

#endif /* __FSL_DPSW_H */
