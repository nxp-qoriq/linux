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
#ifndef __FSL_DPNI_H
#define __FSL_DPNI_H

#include "dpkg.h"

struct fsl_mc_io;

/**
 * Data Path Network Interface API
 * Contains initialization APIs and runtime control APIs for DPNI
 */

/** General DPNI macros */

/**
 * Maximum number of traffic classes
 */
#define DPNI_MAX_TC				8
/**
 * Maximum number of buffer pools per DPNI
 */
#define DPNI_MAX_DPBP				8
/**
 * Maximum number of storage-profiles per DPNI
 */
#define DPNI_MAX_SP				2

/**
 * All traffic classes considered; see dpni_set_rx_flow()
 */
#define DPNI_ALL_TCS				(uint8_t)(-1)
/**
 * All flows within traffic class considered; see dpni_set_rx_flow()
 */
#define DPNI_ALL_TC_FLOWS			(uint16_t)(-1)
/**
 * Generate new flow ID; see dpni_set_tx_flow()
 */
#define DPNI_NEW_FLOW_ID			(uint16_t)(-1)
/* use for common tx-conf queue; see dpni_set_tx_conf_<x>() */
#define DPNI_COMMON_TX_CONF			(uint16_t)(-1)

/**
 * dpni_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpni_id:	DPNI unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpni_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_open(struct fsl_mc_io	*mc_io,
	      uint32_t		cmd_flags,
	      int		dpni_id,
	      uint16_t		*token);

/**
 * dpni_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_close(struct fsl_mc_io	*mc_io,
	       uint32_t		cmd_flags,
	       uint16_t		token);

/* DPNI configuration options */

/**
 * Allow different distribution key profiles for different traffic classes;
 * if not set, a single key profile is assumed
 */
#define DPNI_OPT_ALLOW_DIST_KEY_PER_TC		0x00000001

/**
 * Disable all non-error transmit confirmation; error frames are reported
 * back to a common Tx error queue
 */
#define DPNI_OPT_TX_CONF_DISABLED		0x00000002

/**
 * Disable per-sender private Tx confirmation/error queue
 */
#define DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED	0x00000004

/**
 * Support distribution based on hashed key;
 * allows statistical distribution over receive queues in a traffic class
 */
#define DPNI_OPT_DIST_HASH			0x00000010

/**
 * DEPRECATED - if this flag is selected and and all new 'max_fs_entries' are
 * '0' then backward compatibility is preserved;
 * Support distribution based on flow steering;
 * allows explicit control of distribution over receive queues in a traffic
 * class
 */
#define DPNI_OPT_DIST_FS			0x00000020

/**
 * Unicast filtering support
 */
#define DPNI_OPT_UNICAST_FILTER			0x00000080
/**
 * Multicast filtering support
 */
#define DPNI_OPT_MULTICAST_FILTER		0x00000100
/**
 * VLAN filtering support
 */
#define DPNI_OPT_VLAN_FILTER			0x00000200
/**
 * Support IP reassembly on received packets
 */
#define DPNI_OPT_IPR				0x00000800
/**
 * Support IP fragmentation on transmitted packets
 */
#define DPNI_OPT_IPF				0x00001000
/**
 * VLAN manipulation support
 */
#define DPNI_OPT_VLAN_MANIPULATION		0x00010000
/**
 * Support masking of QoS lookup keys
 */
#define DPNI_OPT_QOS_MASK_SUPPORT		0x00020000
/**
 * Support masking of Flow Steering lookup keys
 */
#define DPNI_OPT_FS_MASK_SUPPORT		0x00040000

/**
 * struct dpni_extended_cfg - Structure representing extended DPNI configuration
 * @tc_cfg: TCs configuration
 * @ipr_cfg: IP reassembly configuration
 */
struct dpni_extended_cfg {
	/**
	 * struct tc_cfg - TC configuration
	 * @max_dist: Maximum distribution size for Rx traffic class;
	 *	supported values: 1,2,3,4,6,7,8,12,14,16,24,28,32,48,56,64,96,
	 *	112,128,192,224,256,384,448,512,768,896,1024;
	 *	value '0' will be treated as '1'.
	 *	other unsupported values will be round down to the nearest
	 *	supported value.
	 * @max_fs_entries: Maximum FS entries for Rx traffic class;
	 *	'0' means no support for this TC;
	 */
	struct {
		uint16_t	max_dist;
		uint16_t	max_fs_entries;
	} tc_cfg[DPNI_MAX_TC];
	/**
	 * struct ipr_cfg - Structure representing IP reassembly configuration
	 * @max_reass_frm_size: Maximum size of the reassembled frame
	 * @min_frag_size_ipv4: Minimum fragment size of IPv4 fragments
	 * @min_frag_size_ipv6: Minimum fragment size of IPv6 fragments
	 * @max_open_frames_ipv4: Maximum concurrent IPv4 packets in reassembly
	 *		process
	 * @max_open_frames_ipv6: Maximum concurrent IPv6 packets in reassembly
	 *		process
	 */
	struct {
		uint16_t max_reass_frm_size;
		uint16_t min_frag_size_ipv4;
		uint16_t min_frag_size_ipv6;
		uint16_t max_open_frames_ipv4;
		uint16_t max_open_frames_ipv6;
	} ipr_cfg;
};

/**
 * dpni_prepare_extended_cfg() - function prepare extended parameters
 * @cfg: extended structure
 * @ext_cfg_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before dpni_create()
 */
int dpni_prepare_extended_cfg(const struct dpni_extended_cfg	*cfg,
			      uint8_t			*ext_cfg_buf);

/**
 * struct dpni_cfg - Structure representing DPNI configuration
 * @mac_addr: Primary MAC address
 * @adv: Advanced parameters; default is all zeros;
 *		use this structure to change default settings
 */
struct dpni_cfg {
	uint8_t mac_addr[6];
	/**
	 * struct adv - Advanced parameters
	 * @options: Mask of available options; use 'DPNI_OPT_<X>' values
	 * @start_hdr: Selects the packet starting header for parsing;
	 *		'NET_PROT_NONE' is treated as default: 'NET_PROT_ETH'
	 * @max_senders: Maximum number of different senders; used as the number
	 *		of dedicated Tx flows; Non-power-of-2 values are rounded
	 *		up to the next power-of-2 value as hardware demands it;
	 *		'0' will be treated as '1'
	 * @max_tcs: Maximum number of traffic classes (for both Tx and Rx);
	 *		'0' will e treated as '1'
	 * @max_unicast_filters: Maximum number of unicast filters;
	 *			'0' is treated	as '16'
	 * @max_multicast_filters: Maximum number of multicast filters;
	 *			'0' is treated as '64'
	 * @max_qos_entries: if 'max_tcs > 1', declares the maximum entries in
	 *			the QoS	table; '0' is treated as '64'
	 * @max_qos_key_size: Maximum key size for the QoS look-up;
	 *			'0' is treated as '24' which is enough for IPv4
	 *			5-tuple
	 * @max_dist_key_size: Maximum key size for the distribution;
	 *		'0' is treated as '24' which is enough for IPv4 5-tuple
	 * @max_policers: Maximum number of policers;
	 *		should be between '0' and max_tcs
	 * @max_congestion_ctrl: Maximum number of congestion control groups
	 *		(CGs); covers early drop and congestion notification
	 *		requirements;
	 *		should be between '0' and ('max_tcs' + 'max_senders')
	 * @ext_cfg_iova: I/O virtual address of 256 bytes DMA-able memory
	 *		filled with the extended configuration by calling
	 *		dpni_prepare_extended_cfg()
	 */
	struct {
		uint32_t	options;
		enum net_prot	start_hdr;
		uint8_t		max_senders;
		uint8_t		max_tcs;
		uint8_t		max_unicast_filters;
		uint8_t		max_multicast_filters;
		uint8_t		max_vlan_filters;
		uint8_t		max_qos_entries;
		uint8_t		max_qos_key_size;
		uint8_t		max_dist_key_size;
		uint8_t		max_policers;
		uint8_t		max_congestion_ctrl;
		uint64_t	ext_cfg_iova;
	} adv;
};

/**
 * dpni_create() - Create the DPNI object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @token:	Returned token; use in subsequent API calls
 *
 * Create the DPNI object, allocate required resources and
 * perform required initialization.
 *
 * The object can be created either by declaring it in the
 * DPL file, or by calling this function.
 *
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent calls to
 * this specific object. For objects that are created using the
 * DPL file, call dpni_open() function to get an authentication
 * token first.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_create(struct fsl_mc_io	*mc_io,
		uint32_t		cmd_flags,
		const struct dpni_cfg	*cfg,
		uint16_t		*token);

/**
 * dpni_destroy() - Destroy the DPNI object and release all its resources.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_destroy(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * struct dpni_pools_cfg - Structure representing buffer pools configuration
 * @num_dpbp: Number of DPBPs
 * @pools: Array of buffer pools parameters; The number of valid entries
 *	must match 'num_dpbp' value
 */
struct dpni_pools_cfg {
	uint8_t		num_dpbp;
	/**
	 * struct pools - Buffer pools parameters
	 * @dpbp_id: DPBP object ID
	 * @buffer_size: Buffer size
	 * @backup_pool: Backup pool
	 */
	struct {
		int		dpbp_id;
		uint16_t	buffer_size;
		int		backup_pool;
	} pools[DPNI_MAX_DPBP];
};

/**
 * dpni_set_pools() - Set buffer pools configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	Buffer pools configuration
 *
 * mandatory for DPNI operation
 * warning:Allowed only when DPNI is disabled
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_pools(struct fsl_mc_io		*mc_io,
		   uint32_t			cmd_flags,
		   uint16_t			token,
		   const struct dpni_pools_cfg	*cfg);

/**
 * dpni_enable() - Enable the DPNI, allow sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPNI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_enable(struct fsl_mc_io	*mc_io,
		uint32_t		cmd_flags,
		uint16_t		token);

/**
 * dpni_disable() - Disable the DPNI, stop sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_disable(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * dpni_is_enabled() - Check if the DPNI is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_is_enabled(struct fsl_mc_io	*mc_io,
		    uint32_t		cmd_flags,
		    uint16_t		token,
		    int		*en);

/**
 * dpni_reset() - Reset the DPNI, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_reset(struct fsl_mc_io	*mc_io,
	       uint32_t		cmd_flags,
	       uint16_t		token);

/**
 * DPNI IRQ Index and Events
 */

/**
 * IRQ index
 */
#define DPNI_IRQ_INDEX				0
/**
 * IRQ event - indicates a change in link state
 */
#define DPNI_IRQ_EVENT_LINK_CHANGED		0x00000001

/**
 * struct dpni_irq_cfg - IRQ configuration
 * @addr:	Address that must be written to signal a message-based interrupt
 * @val:	Value to write into irq_addr address
 * @irq_num: A user defined number associated with this IRQ
 */
struct dpni_irq_cfg {
	     uint64_t		addr;
	     uint32_t		val;
	     int		irq_num;
};

/**
 * dpni_set_irq() - Set IRQ information for the DPNI to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_irq(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 struct dpni_irq_cfg	*irq_cfg);

/**
 * dpni_get_irq() - Get IRQ information from the DPNI.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_irq(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 int			*type,
		 struct dpni_irq_cfg	*irq_cfg);

/**
 * dpni_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state: - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_irq_enable(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint8_t			en);

/**
 * dpni_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_irq_enable(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint8_t			*en);

/**
 * dpni_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @mask:	event mask to trigger interrupt;
 *			each bit:
 *				0 = ignore event
 *				1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_irq_mask(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      uint8_t		irq_index,
		      uint32_t		mask);

/**
 * dpni_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_irq_mask(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      uint8_t		irq_index,
		      uint32_t		*mask);

/**
 * dpni_get_irq_status() - Get the current status of any pending interrupts.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_irq_status(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		*status);

/**
 * dpni_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_clear_irq_status(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint32_t		status);

/**
 * struct dpni_attr - Structure representing DPNI attributes
 * @id: DPNI object ID
 * @version: DPNI version
 * @start_hdr: Indicates the packet starting header for parsing
 * @options: Mask of available options; reflects the value as was given in
 *		object's creation
 * @max_senders: Maximum number of different senders; used as the number
 *		of dedicated Tx flows;
 * @max_tcs: Maximum number of traffic classes (for both Tx and Rx)
 * @max_unicast_filters: Maximum number of unicast filters
 * @max_multicast_filters: Maximum number of multicast filters
 * @max_vlan_filters: Maximum number of VLAN filters
 * @max_qos_entries: if 'max_tcs > 1', declares the maximum entries in QoS table
 * @max_qos_key_size: Maximum key size for the QoS look-up
 * @max_dist_key_size: Maximum key size for the distribution look-up
 * @max_policers: Maximum number of policers;
 * @max_congestion_ctrl: Maximum number of congestion control groups (CGs);
 * @ext_cfg_iova: I/O virtual address of 256 bytes DMA-able memory;
 *	call dpni_extract_extended_cfg() to extract the extended configuration
 */
struct dpni_attr {
	int		id;
	/**
	 * struct version - DPNI version
	 * @major: DPNI major version
	 * @minor: DPNI minor version
	 */
	struct {
		uint16_t major;
		uint16_t minor;
	} version;
	enum net_prot	start_hdr;
	uint32_t	options;
	uint8_t		max_senders;
	uint8_t		max_tcs;
	uint8_t		max_unicast_filters;
	uint8_t		max_multicast_filters;
	uint8_t		max_vlan_filters;
	uint8_t		max_qos_entries;
	uint8_t		max_qos_key_size;
	uint8_t		max_dist_key_size;
	uint8_t		max_policers;
	uint8_t		max_congestion_ctrl;
	uint64_t	ext_cfg_iova;
};

/**
 * dpni_get_attributes() - Retrieve DPNI attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @attr:	Object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_attributes(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			struct dpni_attr	*attr);

/**
 * dpni_extract_extended_cfg() - extract the extended parameters
 * @cfg: extended structure
 * @ext_cfg_buf: 256 bytes of DMA-able memory
 *
 * This function has to be called after dpni_get_attributes()
 */
int dpni_extract_extended_cfg(struct dpni_extended_cfg	*cfg,
			      const uint8_t		*ext_cfg_buf);

/**
 * DPNI errors
 */

/**
 * Extract out of frame header error
 */
#define DPNI_ERROR_EOFHE	0x00020000
/**
 * Frame length error
 */
#define DPNI_ERROR_FLE		0x00002000
/**
 * Frame physical error
 */
#define DPNI_ERROR_FPE		0x00001000
/**
 * Parsing header error
 */
#define DPNI_ERROR_PHE		0x00000020
/**
 * Parser L3 checksum error
 */
#define DPNI_ERROR_L3CE		0x00000004
/**
 * Parser L3 checksum error
 */
#define DPNI_ERROR_L4CE		0x00000001

/**
 * enum dpni_error_action - Defines DPNI behavior for errors
 * @DPNI_ERROR_ACTION_DISCARD: Discard the frame
 * @DPNI_ERROR_ACTION_CONTINUE: Continue with the normal flow
 * @DPNI_ERROR_ACTION_SEND_TO_ERROR_QUEUE: Send the frame to the error queue
 */
enum dpni_error_action {
	DPNI_ERROR_ACTION_DISCARD = 0,
	DPNI_ERROR_ACTION_CONTINUE = 1,
	DPNI_ERROR_ACTION_SEND_TO_ERROR_QUEUE = 2
};

/**
 * struct dpni_error_cfg - Structure representing DPNI errors treatment
 * @errors: Errors mask; use 'DPNI_ERROR__<X>
 * @error_action: The desired action for the errors mask
 * @set_frame_annotation: Set to '1' to mark the errors in frame annotation
 *		status (FAS); relevant only for the non-discard action
 */
struct dpni_error_cfg {
	uint32_t		errors;
	enum dpni_error_action	error_action;
	int			set_frame_annotation;
};

/**
 * dpni_set_errors_behavior() - Set errors behavior
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	Errors configuration
 *
 * this function may be called numerous times with different
 * error masks
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_errors_behavior(struct fsl_mc_io		*mc_io,
			     uint32_t			cmd_flags,
			     uint16_t			token,
			     struct dpni_error_cfg	*cfg);

/**
 * DPNI buffer layout modification options
 */

/**
 * Select to modify the time-stamp setting
 */
#define DPNI_BUF_LAYOUT_OPT_TIMESTAMP		0x00000001
/**
 * Select to modify the parser-result setting; not applicable for Tx
 */
#define DPNI_BUF_LAYOUT_OPT_PARSER_RESULT	0x00000002
/**
 * Select to modify the frame-status setting
 */
#define DPNI_BUF_LAYOUT_OPT_FRAME_STATUS	0x00000004
/**
 * Select to modify the private-data-size setting
 */
#define DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE	0x00000008
/**
 * Select to modify the data-alignment setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_ALIGN		0x00000010
/**
 * Select to modify the data-head-room setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_HEAD_ROOM	0x00000020
/**
 * Select to modify the data-tail-room setting
 */
#define DPNI_BUF_LAYOUT_OPT_DATA_TAIL_ROOM	0x00000040

/**
 * struct dpni_buffer_layout - Structure representing DPNI buffer layout
 * @options: Flags representing the suggested modifications to the buffer
 *		layout; Use any combination of 'DPNI_BUF_LAYOUT_OPT_<X>' flags
 * @pass_timestamp: Pass timestamp value
 * @pass_parser_result: Pass parser results
 * @pass_frame_status: Pass frame status
 * @private_data_size: Size kept for private data (in bytes)
 * @data_align: Data alignment
 * @data_head_room: Data head room
 * @data_tail_room: Data tail room
 */
struct dpni_buffer_layout {
	uint32_t	options;
	int		pass_timestamp;
	int		pass_parser_result;
	int		pass_frame_status;
	uint16_t	private_data_size;
	uint16_t	data_align;
	uint16_t	data_head_room;
	uint16_t	data_tail_room;
};

/**
 * dpni_get_rx_buffer_layout() - Retrieve Rx buffer layout attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Returns buffer layout attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_rx_buffer_layout(struct fsl_mc_io		*mc_io,
			      uint32_t			cmd_flags,
			      uint16_t			token,
			      struct dpni_buffer_layout	*layout);

/**
 * dpni_set_rx_buffer_layout() - Set Rx buffer layout configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Buffer layout configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 *
 * @warning	Allowed only when DPNI is disabled
 */
int dpni_set_rx_buffer_layout(struct fsl_mc_io			*mc_io,
			      uint32_t				cmd_flags,
			      uint16_t				token,
			      const struct dpni_buffer_layout	*layout);

/**
 * dpni_get_tx_buffer_layout() - Retrieve Tx buffer layout attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Returns buffer layout attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_tx_buffer_layout(struct fsl_mc_io		*mc_io,
			      uint32_t			cmd_flags,
			      uint16_t			token,
			      struct dpni_buffer_layout	*layout);

/**
 * dpni_set_tx_buffer_layout() - Set Tx buffer layout configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Buffer layout configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 *
 * @warning	Allowed only when DPNI is disabled
 */
int dpni_set_tx_buffer_layout(struct fsl_mc_io			*mc_io,
			      uint32_t				cmd_flags,
			      uint16_t				token,
			      const struct dpni_buffer_layout	*layout);

/**
 * dpni_get_tx_conf_buffer_layout() - Retrieve Tx confirmation buffer layout
 *				attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Returns buffer layout attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_tx_conf_buffer_layout(struct fsl_mc_io		*mc_io,
				   uint32_t			cmd_flags,
				   uint16_t			token,
				   struct dpni_buffer_layout	*layout);

/**
 * dpni_set_tx_conf_buffer_layout() - Set Tx confirmation buffer layout
 *					configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @layout:	Buffer layout configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 *
 * @warning	Allowed only when DPNI is disabled
 */
int dpni_set_tx_conf_buffer_layout(struct fsl_mc_io		   *mc_io,
				   uint32_t			   cmd_flags,
				   uint16_t			   token,
				   const struct dpni_buffer_layout *layout);

/**
 * dpni_set_l3_chksum_validation() - Enable/disable L3 checksum validation
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_l3_chksum_validation(struct fsl_mc_io	*mc_io,
				  uint32_t		cmd_flags,
				  uint16_t		token,
				  int			en);

/**
 * dpni_get_l3_chksum_validation() - Get L3 checksum validation mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Returns '1' if enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_l3_chksum_validation(struct fsl_mc_io	*mc_io,
				  uint32_t		cmd_flags,
				  uint16_t		token,
				  int			*en);

/**
 * dpni_set_l4_chksum_validation() - Enable/disable L4 checksum validation
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_l4_chksum_validation(struct fsl_mc_io	*mc_io,
				  uint32_t		cmd_flags,
				  uint16_t		token,
				  int			en);

/**
 * dpni_get_l4_chksum_validation() - Get L4 checksum validation mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Returns '1' if enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_l4_chksum_validation(struct fsl_mc_io	*mc_io,
				  uint32_t		cmd_flags,
				  uint16_t		token,
				  int			*en);

/**
 * dpni_get_qdid() - Get the Queuing Destination ID (QDID) that should be used
 *			for enqueue operations
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @qdid:	Returned virtual QDID value that should be used as an argument
 *			in all enqueue operations
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_qdid(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token,
		  uint16_t		*qdid);

/**
 * struct dpni_sp_info - Structure representing DPNI storage-profile information
 * (relevant only for DPNI owned by AIOP)
 * @spids: array of storage-profiles
 */
struct dpni_sp_info {
	uint16_t spids[DPNI_MAX_SP];
};

/**
 * dpni_get_spids() - Get the AIOP storage profile IDs associated with the DPNI
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @sp_info:	Returned AIOP storage-profile information
 *
 * Return:	'0' on Success; Error code otherwise.
 *
 * @warning	Only relevant for DPNI that belongs to AIOP container.
 */
int dpni_get_sp_info(struct fsl_mc_io		*mc_io,
		     uint32_t			cmd_flags,
		     uint16_t			token,
		     struct dpni_sp_info	*sp_info);

/**
 * dpni_get_tx_data_offset() - Get the Tx data offset (from start of buffer)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @data_offset: Tx data offset (from start of buffer)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_tx_data_offset(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    uint16_t		*data_offset);

/**
 * enum dpni_counter - DPNI counter types
 * @DPNI_CNT_ING_FRAME: Counts ingress frames
 * @DPNI_CNT_ING_BYTE: Counts ingress bytes
 * @DPNI_CNT_ING_FRAME_DROP: Counts ingress frames dropped due to explicit
 *		'drop' setting
 * @DPNI_CNT_ING_FRAME_DISCARD: Counts ingress frames discarded due to errors
 * @DPNI_CNT_ING_MCAST_FRAME: Counts ingress multicast frames
 * @DPNI_CNT_ING_MCAST_BYTE: Counts ingress multicast bytes
 * @DPNI_CNT_ING_BCAST_FRAME: Counts ingress broadcast frames
 * @DPNI_CNT_ING_BCAST_BYTES: Counts ingress broadcast bytes
 * @DPNI_CNT_EGR_FRAME: Counts egress frames
 * @DPNI_CNT_EGR_BYTE: Counts egress bytes
 * @DPNI_CNT_EGR_FRAME_DISCARD: Counts egress frames discarded due to errors
 */
enum dpni_counter {
	DPNI_CNT_ING_FRAME = 0x0,
	DPNI_CNT_ING_BYTE = 0x1,
	DPNI_CNT_ING_FRAME_DROP = 0x2,
	DPNI_CNT_ING_FRAME_DISCARD = 0x3,
	DPNI_CNT_ING_MCAST_FRAME = 0x4,
	DPNI_CNT_ING_MCAST_BYTE = 0x5,
	DPNI_CNT_ING_BCAST_FRAME = 0x6,
	DPNI_CNT_ING_BCAST_BYTES = 0x7,
	DPNI_CNT_EGR_FRAME = 0x8,
	DPNI_CNT_EGR_BYTE = 0x9,
	DPNI_CNT_EGR_FRAME_DISCARD = 0xa
};

/**
 * dpni_get_counter() - Read a specific DPNI counter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @counter:	The requested counter
 * @value:	Returned counter's current value
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_counter(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     enum dpni_counter	counter,
		     uint64_t		*value);

/**
 * dpni_set_counter() - Set (or clear) a specific DPNI counter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @counter:	The requested counter
 * @value:	New counter value; typically pass '0' for resetting
 *			the counter.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_counter(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     enum dpni_counter	counter,
		     uint64_t		value);

/**
 * Enable auto-negotiation
 */
#define DPNI_LINK_OPT_AUTONEG		0x0000000000000001ULL
/**
 * Enable half-duplex mode
 */
#define DPNI_LINK_OPT_HALF_DUPLEX	0x0000000000000002ULL
/**
 * Enable pause frames
 */
#define DPNI_LINK_OPT_PAUSE		0x0000000000000004ULL
/**
 * Enable a-symmetric pause frames
 */
#define DPNI_LINK_OPT_ASYM_PAUSE	0x0000000000000008ULL

/**
 * struct - Structure representing DPNI link configuration
 * @rate: Rate
 * @options: Mask of available options; use 'DPNI_LINK_OPT_<X>' values
 */
struct dpni_link_cfg {
	uint32_t rate;
	uint64_t options;
};

/**
 * dpni_set_link_cfg() - set the link configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	Link configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_link_cfg(struct fsl_mc_io			*mc_io,
		      uint32_t				cmd_flags,
		      uint16_t				token,
		      const struct dpni_link_cfg	*cfg);

/**
 * struct dpni_link_state - Structure representing DPNI link state
 * @rate: Rate
 * @options: Mask of available options; use 'DPNI_LINK_OPT_<X>' values
 * @up: Link state; '0' for down, '1' for up
 */
struct dpni_link_state {
	uint32_t	rate;
	uint64_t	options;
	int		up;
};

/**
 * dpni_get_link_state() - Return the link state (either up or down)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @state:	Returned link state;
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_link_state(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			struct dpni_link_state	*state);

/**
 * struct dpni_tx_shaping - Structure representing DPNI tx shaping configuration
 * @rate_limit: rate in Mbps
 * @max_burst_size: burst size in bytes (up to 64KB)
 */
struct dpni_tx_shaping_cfg {
	uint32_t rate_limit;
	uint16_t max_burst_size;
};

/**
 * dpni_set_tx_shaping() - Set the transmit shaping
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tx_shaper:  tx shaping configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_tx_shaping(struct fsl_mc_io			*mc_io,
			uint32_t				cmd_flags,
			uint16_t				token,
			const struct dpni_tx_shaping_cfg	*tx_shaper);

/**
 * dpni_set_max_frame_length() - Set the maximum received frame length.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @max_frame_length:	Maximum received frame length (in
 *				bytes); frame is discarded if its
 *				length exceeds this value
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_max_frame_length(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint16_t		max_frame_length);

/**
 * dpni_get_max_frame_length() - Get the maximum received frame length.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @max_frame_length:	Maximum received frame length (in
 *				bytes); frame is discarded if its
 *				length exceeds this value
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_max_frame_length(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint16_t		*max_frame_length);

/**
 * dpni_set_mtu() - Set the MTU for the interface.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mtu:	MTU length (in bytes)
 *
 * MTU determines the maximum fragment size for performing IP
 * fragmentation on egress packets.
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_mtu(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint16_t		mtu);

/**
 * dpni_get_mtu() - Get the MTU.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mtu:	Returned MTU length (in bytes)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_mtu(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint16_t		*mtu);

/**
 * dpni_set_multicast_promisc() - Enable/disable multicast promiscuous mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_multicast_promisc(struct fsl_mc_io *mc_io,
			       uint32_t	cmd_flags,
			       uint16_t		token,
			       int		en);

/**
 * dpni_get_multicast_promisc() - Get multicast promiscuous mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Returns '1' if enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_multicast_promisc(struct fsl_mc_io *mc_io,
			       uint32_t	cmd_flags,
			       uint16_t		token,
			       int		*en);

/**
 * dpni_set_unicast_promisc() - Enable/disable unicast promiscuous mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_unicast_promisc(struct fsl_mc_io	*mc_io,
			     uint32_t		cmd_flags,
			     uint16_t		token,
			     int		en);

/**
 * dpni_get_unicast_promisc() - Get unicast promiscuous mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Returns '1' if enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_unicast_promisc(struct fsl_mc_io	*mc_io,
			     uint32_t		cmd_flags,
			     uint16_t		token,
			     int		*en);

/**
 * dpni_set_primary_mac_addr() - Set the primary MAC address
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mac_addr:	MAC address to set as primary address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_primary_mac_addr(struct fsl_mc_io *mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      const uint8_t	mac_addr[6]);

/**
 * dpni_get_primary_mac_addr() - Get the primary MAC address
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mac_addr:	Returned MAC address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_primary_mac_addr(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		mac_addr[6]);

/**
 * dpni_add_mac_addr() - Add MAC address filter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mac_addr:	MAC address to add
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_add_mac_addr(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      const uint8_t	mac_addr[6]);

/**
 * dpni_remove_mac_addr() - Remove MAC address filter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @mac_addr:	MAC address to remove
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_remove_mac_addr(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			 uint16_t		token,
			 const uint8_t		mac_addr[6]);

/**
 * dpni_clear_mac_filters() - Clear all unicast and/or multicast MAC filters
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @unicast:	Set to '1' to clear unicast addresses
 * @multicast:	Set to '1' to clear multicast addresses
 *
 * The primary MAC address is not cleared by this operation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_clear_mac_filters(struct fsl_mc_io	*mc_io,
			   uint32_t		cmd_flags,
			   uint16_t		token,
			   int			unicast,
			   int			multicast);

/**
 * dpni_set_vlan_filters() - Enable/disable VLAN filtering mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_vlan_filters(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  int			en);

/**
 * dpni_add_vlan_id() - Add VLAN ID filter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @vlan_id:	VLAN ID to add
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_add_vlan_id(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     uint16_t		vlan_id);

/**
 * dpni_remove_vlan_id() - Remove VLAN ID filter
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @vlan_id:	VLAN ID to remove
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_remove_vlan_id(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint16_t		vlan_id);

/**
 * dpni_clear_vlan_filters() - Clear all VLAN filters
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_clear_vlan_filters(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token);

/**
 * enum dpni_tx_schedule_mode - DPNI Tx scheduling mode
 * @DPNI_TX_SCHED_STRICT_PRIORITY: strict priority
 * @DPNI_TX_SCHED_WEIGHTED: weighted based scheduling
 */
enum dpni_tx_schedule_mode {
	DPNI_TX_SCHED_STRICT_PRIORITY,
	DPNI_TX_SCHED_WEIGHTED,
};

/**
 * struct dpni_tx_schedule_cfg - Structure representing Tx
 *	scheduling configuration
 * @mode: scheduling mode
 * @delta_bandwidth: Bandwidth represented in weights from 100 to 10000;
 *	not applicable for 'strict-priority' mode;
 */
struct dpni_tx_schedule_cfg {
	enum dpni_tx_schedule_mode	mode;
	uint16_t		delta_bandwidth;
};

/**
 * struct dpni_tx_selection_cfg - Structure representing transmission
 *	selection configuration
 * @tc_sched: an array of traffic-classes
 */
struct dpni_tx_selection_cfg {
	struct dpni_tx_schedule_cfg tc_sched[DPNI_MAX_TC];
};

/**
 * dpni_set_tx_selection() - Set transmission selection configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	transmission selection configuration
 *
 * warning:	Allowed only when DPNI is disabled
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_tx_selection(struct fsl_mc_io			*mc_io,
			  uint32_t				cmd_flags,
			  uint16_t				token,
			  const struct dpni_tx_selection_cfg	*cfg);

/**
 * enum dpni_dist_mode - DPNI distribution mode
 * @DPNI_DIST_MODE_NONE: No distribution
 * @DPNI_DIST_MODE_HASH: Use hash distribution; only relevant if
 *		the 'DPNI_OPT_DIST_HASH' option was set at DPNI creation
 * @DPNI_DIST_MODE_FS:  Use explicit flow steering; only relevant if
 *	 the 'DPNI_OPT_DIST_FS' option was set at DPNI creation
 */
enum dpni_dist_mode {
	DPNI_DIST_MODE_NONE = 0,
	DPNI_DIST_MODE_HASH = 1,
	DPNI_DIST_MODE_FS = 2
};

/**
 * enum dpni_fs_miss_action -   DPNI Flow Steering miss action
 * @DPNI_FS_MISS_DROP: In case of no-match, drop the frame
 * @DPNI_FS_MISS_EXPLICIT_FLOWID: In case of no-match, use explicit flow-id
 * @DPNI_FS_MISS_HASH: In case of no-match, distribute using hash
 */
enum dpni_fs_miss_action {
	DPNI_FS_MISS_DROP = 0,
	DPNI_FS_MISS_EXPLICIT_FLOWID = 1,
	DPNI_FS_MISS_HASH = 2
};

/**
 * struct dpni_fs_tbl_cfg - Flow Steering table configuration
 * @miss_action: Miss action selection
 * @default_flow_id: Used when 'miss_action = DPNI_FS_MISS_EXPLICIT_FLOWID'
 */
struct dpni_fs_tbl_cfg {
	enum dpni_fs_miss_action	miss_action;
	uint16_t			default_flow_id;
};

/**
 * dpni_prepare_key_cfg() - function prepare extract parameters
 * @cfg: defining a full Key Generation profile (rule)
 * @key_cfg_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before the following functions:
 *	- dpni_set_rx_tc_dist()
 *		- dpni_set_qos_table()
 */
int dpni_prepare_key_cfg(const struct dpkg_profile_cfg	*cfg,
			 uint8_t			*key_cfg_buf);

/**
 * struct dpni_rx_tc_dist_cfg - Rx traffic class distribution configuration
 * @dist_size: Set the distribution size;
 *	supported values: 1,2,3,4,6,7,8,12,14,16,24,28,32,48,56,64,96,
 *	112,128,192,224,256,384,448,512,768,896,1024
 * @dist_mode: Distribution mode
 * @key_cfg_iova: I/O virtual address of 256 bytes DMA-able memory filled with
 *		the extractions to be used for the distribution key by calling
 *		dpni_prepare_key_cfg() relevant only when
 *		'dist_mode != DPNI_DIST_MODE_NONE', otherwise it can be '0'
 * @fs_cfg: Flow Steering table configuration; only relevant if
 *		'dist_mode = DPNI_DIST_MODE_FS'
 */
struct dpni_rx_tc_dist_cfg {
	uint16_t		dist_size;
	enum dpni_dist_mode	dist_mode;
	uint64_t		key_cfg_iova;
	struct dpni_fs_tbl_cfg	fs_cfg;
};

/**
 * dpni_set_rx_tc_dist() - Set Rx traffic class distribution configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	Traffic class distribution configuration
 *
 * warning: if 'dist_mode != DPNI_DIST_MODE_NONE', call dpni_prepare_key_cfg()
 *			first to prepare the key_cfg_iova parameter
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_rx_tc_dist(struct fsl_mc_io			*mc_io,
			uint32_t				cmd_flags,
			uint16_t				token,
			uint8_t					tc_id,
			const struct dpni_rx_tc_dist_cfg	*cfg);

/**
 * Set to select color aware mode (otherwise - color blind)
 */
#define DPNI_POLICER_OPT_COLOR_AWARE	0x00000001
/**
 * Set to discard frame with RED color
 */
#define DPNI_POLICER_OPT_DISCARD_RED	0x00000002

/**
 * enum dpni_policer_mode - selecting the policer mode
 * @DPNI_POLICER_MODE_NONE: Policer is disabled
 * @DPNI_POLICER_MODE_PASS_THROUGH: Policer pass through
 * @DPNI_POLICER_MODE_RFC_2698: Policer algorithm RFC 2698
 * @DPNI_POLICER_MODE_RFC_4115: Policer algorithm RFC 4115
 */
enum dpni_policer_mode {
	DPNI_POLICER_MODE_NONE = 0,
	DPNI_POLICER_MODE_PASS_THROUGH,
	DPNI_POLICER_MODE_RFC_2698,
	DPNI_POLICER_MODE_RFC_4115
};

/**
 * enum dpni_policer_unit - DPNI policer units
 * @DPNI_POLICER_UNIT_BYTES: bytes units
 * @DPNI_POLICER_UNIT_FRAMES: frames units
 */
enum dpni_policer_unit {
	DPNI_POLICER_UNIT_BYTES = 0,
	DPNI_POLICER_UNIT_FRAMES
};

/**
 * enum dpni_policer_color - selecting the policer color
 * @DPNI_POLICER_COLOR_GREEN: Green color
 * @DPNI_POLICER_COLOR_YELLOW: Yellow color
 * @DPNI_POLICER_COLOR_RED: Red color
 */
enum dpni_policer_color {
	DPNI_POLICER_COLOR_GREEN = 0,
	DPNI_POLICER_COLOR_YELLOW,
	DPNI_POLICER_COLOR_RED
};

/**
 * struct dpni_rx_tc_policing_cfg - Policer configuration
 * @options: Mask of available options; use 'DPNI_POLICER_OPT_<X>' values
 * @mode: policer mode
 * @default_color: For pass-through mode the policer re-colors with this
 *	color any incoming packets. For Color aware non-pass-through mode:
 *	policer re-colors with this color all packets with FD[DROPP]>2.
 * @units: Bytes or Packets
 * @cir: Committed information rate (CIR) in Kbps or packets/second
 * @cbs: Committed burst size (CBS) in bytes or packets
 * @eir: Peak information rate (PIR, rfc2698) in Kbps or packets/second
 *	 Excess information rate (EIR, rfc4115) in Kbps or packets/second
 * @ebs: Peak burst size (PBS, rfc2698) in bytes or packets
 *       Excess burst size (EBS, rfc4115) in bytes or packets
 */
struct dpni_rx_tc_policing_cfg {
	uint32_t		options;
	enum dpni_policer_mode	mode;
	enum dpni_policer_unit	units;
	enum dpni_policer_color default_color;
	uint32_t		cir;
	uint32_t		cbs;
	uint32_t		eir;
	uint32_t		ebs;
};

/**
 * dpni_set_rx_tc_policing() - Set Rx traffic class policing configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	Traffic class policing configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_rx_tc_policing(struct fsl_mc_io			*mc_io,
			    uint32_t				cmd_flags,
			    uint16_t				token,
			    uint8_t				tc_id,
			    const struct dpni_rx_tc_policing_cfg *cfg);

/**
 * dpni_get_rx_tc_policing() - Get Rx traffic class policing configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	Traffic class policing configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_rx_tc_policing(struct fsl_mc_io			*mc_io,
			    uint32_t				cmd_flags,
			    uint16_t				token,
			    uint8_t				tc_id,
			    struct dpni_rx_tc_policing_cfg	*cfg);

/**
 * enum dpni_congestion_unit - DPNI congestion units
 * @DPNI_CONGESTION_UNIT_BYTES: bytes units
 * @DPNI_CONGESTION_UNIT_FRAMES: frames units
 */
enum dpni_congestion_unit {
	DPNI_CONGESTION_UNIT_BYTES = 0,
	DPNI_CONGESTION_UNIT_FRAMES
};

/**
 * enum dpni_early_drop_mode - DPNI early drop mode
 * @DPNI_EARLY_DROP_MODE_NONE: early drop is disabled
 * @DPNI_EARLY_DROP_MODE_TAIL: early drop in taildrop mode
 * @DPNI_EARLY_DROP_MODE_WRED: early drop in WRED mode
 */
enum dpni_early_drop_mode {
	DPNI_EARLY_DROP_MODE_NONE = 0,
	DPNI_EARLY_DROP_MODE_TAIL,
	DPNI_EARLY_DROP_MODE_WRED
};

/**
 * struct dpni_wred_cfg - WRED configuration
 * @max_threshold: maximum threshold that packets may be discarded. Above this
 *	  threshold all packets are discarded; must be less than 2^39;
 *	  approximated to be expressed as (x+256)*2^(y-1) due to HW
 *	  implementation.
 * @min_threshold: minimum threshold that packets may be discarded at
 * @drop_probability: probability that a packet will be discarded (1-100,
 *			associated with the max_threshold).
 */
struct dpni_wred_cfg {
	uint64_t	max_threshold;
	uint64_t	min_threshold;
	uint8_t	drop_probability;
};

/**
 * struct dpni_early_drop_cfg - early-drop configuration
 * @mode: drop mode
 * @units: units type
 * @green: WRED - 'green' configuration
 * @yellow: WRED - 'yellow' configuration
 * @red: WRED - 'red' configuration
 * @tail_drop_threshold: tail drop threshold
 */
struct dpni_early_drop_cfg {
	enum dpni_early_drop_mode	mode;
	enum dpni_congestion_unit	units;

	struct dpni_wred_cfg		green;
	struct dpni_wred_cfg		yellow;
	struct dpni_wred_cfg		red;

	uint32_t			tail_drop_threshold;
};

/**
 * dpni_prepare_early_drop() - prepare an early drop.
 * @cfg: Early-drop configuration
 * @early_drop_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before dpni_set_rx_tc_early_drop or
 * dpni_set_tx_tc_early_drop
 *
 */
void dpni_prepare_early_drop(const struct dpni_early_drop_cfg *cfg,
			     uint8_t	*early_drop_buf);

/**
 * dpni_extract_early_drop() - extract the early drop configuration.
 * @cfg: Early-drop configuration
 * @early_drop_buf: Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called after dpni_get_rx_tc_early_drop or
 * dpni_get_tx_tc_early_drop
 *
 */
void dpni_extract_early_drop(struct dpni_early_drop_cfg *cfg,
			     const uint8_t *early_drop_buf);

/**
 * dpni_set_rx_tc_early_drop() - Set Rx traffic class early-drop configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @early_drop_iova:  I/O virtual address of 256 bytes DMA-able memory filled
 *	with the early-drop configuration by calling dpni_prepare_early_drop()
 *
 * warning: Before calling this function, call dpni_prepare_early_drop() to
 *			prepare the early_drop_iova parameter
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_rx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova);

/**
 * dpni_get_rx_tc_early_drop() - Get Rx traffic class early-drop configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @early_drop_iova:  I/O virtual address of 256 bytes DMA-able memory
 *
 * warning: After calling this function, call dpni_extract_early_drop() to
 *	get the early drop configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_rx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova);

/**
 * dpni_set_tx_tc_early_drop() - Set Tx traffic class early-drop configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @early_drop_iova:  I/O virtual address of 256 bytes DMA-able memory filled
 *	with the early-drop configuration by calling dpni_prepare_early_drop()
 *
 * warning: Before calling this function, call dpni_prepare_early_drop() to
 *			prepare the early_drop_iova parameter
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_tx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova);

/**
 * dpni_get_tx_tc_early_drop() - Get Tx traffic class early-drop configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @early_drop_iova:  I/O virtual address of 256 bytes DMA-able memory
 *
 * warning: After calling this function, call dpni_extract_early_drop() to
 *	get the early drop configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_tx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova);

/**
 * enum dpni_dest - DPNI destination types
 * @DPNI_DEST_NONE: Unassigned destination; The queue is set in parked mode and
 *		does not generate FQDAN notifications; user is expected to
 *		dequeue from the queue based on polling or other user-defined
 *		method
 * @DPNI_DEST_DPIO: The queue is set in schedule mode and generates FQDAN
 *		notifications to the specified DPIO; user is expected to dequeue
 *		from the queue only after notification is received
 * @DPNI_DEST_DPCON: The queue is set in schedule mode and does not generate
 *		FQDAN notifications, but is connected to the specified DPCON
 *		object; user is expected to dequeue from the DPCON channel
 */
enum dpni_dest {
	DPNI_DEST_NONE = 0,
	DPNI_DEST_DPIO = 1,
	DPNI_DEST_DPCON = 2
};

/**
 * struct dpni_dest_cfg - Structure representing DPNI destination parameters
 * @dest_type: Destination type
 * @dest_id: Either DPIO ID or DPCON ID, depending on the destination type
 * @priority: Priority selection within the DPIO or DPCON channel; valid values
 *		are 0-1 or 0-7, depending on the number of priorities in that
 *		channel; not relevant for 'DPNI_DEST_NONE' option
 */
struct dpni_dest_cfg {
	enum dpni_dest	dest_type;
	int		dest_id;
	uint8_t	priority;
};

/* DPNI congestion options */

/**
 * CSCN message is written to message_iova once entering a
 * congestion state (see 'threshold_entry')
 */
#define DPNI_CONG_OPT_WRITE_MEM_ON_ENTER	0x00000001
/**
 * CSCN message is written to message_iova once exiting a
 * congestion state (see 'threshold_exit')
 */
#define DPNI_CONG_OPT_WRITE_MEM_ON_EXIT		0x00000002
/**
 * CSCN write will attempt to allocate into a cache (coherent write);
 * valid only if 'DPNI_CONG_OPT_WRITE_MEM_<X>' is selected
 */
#define DPNI_CONG_OPT_COHERENT_WRITE		0x00000004
/**
 * if 'dest_cfg.dest_type != DPNI_DEST_NONE' CSCN message is sent to
 * DPIO/DPCON's WQ channel once entering a congestion state
 * (see 'threshold_entry')
 */
#define DPNI_CONG_OPT_NOTIFY_DEST_ON_ENTER	0x00000008
/**
 * if 'dest_cfg.dest_type != DPNI_DEST_NONE' CSCN message is sent to
 * DPIO/DPCON's WQ channel once exiting a congestion state
 * (see 'threshold_exit')
 */
#define DPNI_CONG_OPT_NOTIFY_DEST_ON_EXIT	0x00000010
/**
 * if 'dest_cfg.dest_type != DPNI_DEST_NONE' when the CSCN is written to the
 * sw-portal's DQRR, the DQRI interrupt is asserted immediately (if enabled)
 */
#define DPNI_CONG_OPT_INTR_COALESCING_DISABLED	0x00000020

/**
 * struct dpni_congestion_notification_cfg - congestion notification
 *		configuration
 * @units: units type
 * @threshold_entry: above this threshold we enter a congestion state.
 *	set it to '0' to disable it
 * @threshold_exit: below this threshold we exit the congestion state.
 * @message_ctx: The context that will be part of the CSCN message
 * @message_iova: I/O virtual address (must be in DMA-able memory),
 *	must be 16B aligned; valid only if 'DPNI_CONG_OPT_WRITE_MEM_<X>' is
 *	contained in 'options'
 * @dest_cfg: CSCN can be send to either DPIO or DPCON WQ channel
 * @options: Mask of available options; use 'DPNI_CONG_OPT_<X>' values
 */

struct dpni_congestion_notification_cfg {
	enum dpni_congestion_unit	units;
	uint32_t	threshold_entry;
	uint32_t	threshold_exit;
	uint64_t	message_ctx;
	uint64_t	message_iova;
	struct dpni_dest_cfg dest_cfg;
	uint16_t	options;
};

/**
 * dpni_set_rx_tc_congestion_notification() - Set Rx traffic class congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_rx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
			const struct dpni_congestion_notification_cfg *cfg);

/**
 * dpni_get_rx_tc_congestion_notification() - Get Rx traffic class congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_rx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
				struct dpni_congestion_notification_cfg *cfg);

/**
 * dpni_set_tx_tc_congestion_notification() - Set Tx traffic class congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_tx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
			const struct dpni_congestion_notification_cfg *cfg);

/**
 * dpni_get_tx_tc_congestion_notification() - Get Tx traffic class congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	congestion notification configuration
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_tx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
				struct dpni_congestion_notification_cfg *cfg);

/**
 * enum dpni_flc_type - DPNI FLC types
 * @DPNI_FLC_USER_DEFINED: select the FLC to be used for user defined value
 * @DPNI_FLC_STASH: select the FLC to be used for stash control
 */
enum dpni_flc_type {
	DPNI_FLC_USER_DEFINED = 0,
	DPNI_FLC_STASH = 1,
};

/**
 * enum dpni_stash_size - DPNI FLC stashing size
 * @DPNI_STASH_SIZE_0B: no stash
 * @DPNI_STASH_SIZE_64B: stashes 64 bytes
 * @DPNI_STASH_SIZE_128B: stashes 128 bytes
 * @DPNI_STASH_SIZE_192B: stashes 192 bytes
 */
enum dpni_stash_size {
	DPNI_STASH_SIZE_0B = 0,
	DPNI_STASH_SIZE_64B = 1,
	DPNI_STASH_SIZE_128B = 2,
	DPNI_STASH_SIZE_192B = 3,
};

/* DPNI FLC stash options */

/**
 * stashes the whole annotation area (up to 192 bytes)
 */
#define DPNI_FLC_STASH_FRAME_ANNOTATION	0x00000001

/**
 * struct dpni_flc_cfg - Structure representing DPNI FLC configuration
 * @flc_type: FLC type
 * @options: Mask of available options;
 *	use 'DPNI_FLC_STASH_<X>' values
 * @frame_data_size: Size of frame data to be stashed
 * @flow_context_size: Size of flow context to be stashed
 * @flow_context: 1. In case flc_type is 'DPNI_FLC_USER_DEFINED':
 *			this value will be provided in the frame descriptor
 *			(FD[FLC])
 *		  2. In case flc_type is 'DPNI_FLC_STASH':
 *			this value will be I/O virtual address of the
 *			flow-context;
 *			Must be cacheline-aligned and DMA-able memory
 */
struct dpni_flc_cfg {
	enum dpni_flc_type	flc_type;
	uint32_t		options;
	enum dpni_stash_size	frame_data_size;
	enum dpni_stash_size	flow_context_size;
	uint64_t		flow_context;
};

/**
 * DPNI queue modification options
 */

/**
 * Select to modify the user's context associated with the queue
 */
#define DPNI_QUEUE_OPT_USER_CTX		0x00000001
/**
 * Select to modify the queue's destination
 */
#define DPNI_QUEUE_OPT_DEST		0x00000002
/** Select to modify the flow-context parameters;
 * not applicable for Tx-conf/Err queues as the FD comes from the user
 */
#define DPNI_QUEUE_OPT_FLC		0x00000004
/**
 * Select to modify the queue's order preservation
 */
#define DPNI_QUEUE_OPT_ORDER_PRESERVATION 0x00000008
/* Select to modify the queue's tail-drop threshold */
#define DPNI_QUEUE_OPT_TAILDROP_THRESHOLD 0x00000010

/**
 * struct dpni_queue_cfg - Structure representing queue configuration
 * @options: Flags representing the suggested modifications to the queue;
 *		Use any combination of 'DPNI_QUEUE_OPT_<X>' flags
 * @user_ctx: User context value provided in the frame descriptor of each
 *		dequeued frame; valid only if 'DPNI_QUEUE_OPT_USER_CTX'
 *		is contained in 'options'
 * @dest_cfg: Queue destination parameters;
 *		valid only if 'DPNI_QUEUE_OPT_DEST' is contained in 'options'
 * @flc_cfg: Flow context configuration; in case the TC's distribution
 *		is either NONE or HASH the FLC's settings of flow#0 are used.
 *		in the case of FS (flow-steering) the flow's FLC settings
 *		are used.
 *		valid only if 'DPNI_QUEUE_OPT_FLC' is contained in 'options'
 * @order_preservation_en: enable/disable order preservation;
 *		valid only if 'DPNI_QUEUE_OPT_ORDER_PRESERVATION' is contained
 *		in 'options'
 * @tail_drop_threshold: set the queue's tail drop threshold in bytes;
 *		'0' value disable the threshold; maximum value is 0xE000000;
 *		valid only if 'DPNI_QUEUE_OPT_TAILDROP_THRESHOLD' is contained
 *		in 'options'
 */
struct dpni_queue_cfg {
	uint32_t		options;
	uint64_t		user_ctx;
	struct dpni_dest_cfg	dest_cfg;
	struct dpni_flc_cfg	flc_cfg;
	int			order_preservation_en;
	uint32_t		tail_drop_threshold;
};

/**
 * struct dpni_queue_attr - Structure representing queue attributes
 * @user_ctx: User context value provided in the frame descriptor of each
 *	dequeued frame
 * @dest_cfg: Queue destination configuration
 * @flc_cfg: Flow context configuration
 * @order_preservation_en: enable/disable order preservation
 * @tail_drop_threshold: queue's tail drop threshold in bytes;
 * @fqid: Virtual fqid value to be used for dequeue operations
 */
struct dpni_queue_attr {
	uint64_t		user_ctx;
	struct dpni_dest_cfg	dest_cfg;
	struct dpni_flc_cfg	flc_cfg;
	int			order_preservation_en;
	uint32_t		tail_drop_threshold;

	uint32_t		fqid;
};

/**
 * DPNI Tx flow modification options
 */

/**
 * Select to modify the settings for dedicate Tx confirmation/error
 */
#define DPNI_TX_FLOW_OPT_TX_CONF_ERROR	0x00000001
/**
 * Select to modify the L3 checksum generation setting
 */
#define DPNI_TX_FLOW_OPT_L3_CHKSUM_GEN	0x00000010
/**
 * Select to modify the L4 checksum generation setting
 */
#define DPNI_TX_FLOW_OPT_L4_CHKSUM_GEN	0x00000020

/**
 * struct dpni_tx_flow_cfg - Structure representing Tx flow configuration
 * @options: Flags representing the suggested modifications to the Tx flow;
 *	Use any combination 'DPNI_TX_FLOW_OPT_<X>' flags
 * @use_common_tx_conf_queue: Set to '1' to use the common (default) Tx
 *	confirmation and error queue; Set to '0' to use the private
 *	Tx confirmation and error queue; valid only if
 *	'DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED' wasn't set at DPNI creation
 *	and 'DPNI_TX_FLOW_OPT_TX_CONF_ERROR' is contained in 'options'
 * @l3_chksum_gen: Set to '1' to enable L3 checksum generation; '0' to disable;
 *	valid only if 'DPNI_TX_FLOW_OPT_L3_CHKSUM_GEN' is contained in 'options'
 * @l4_chksum_gen: Set to '1' to enable L4 checksum generation; '0' to disable;
 *	valid only if 'DPNI_TX_FLOW_OPT_L4_CHKSUM_GEN' is contained in 'options'
 */
struct dpni_tx_flow_cfg {
	uint32_t	options;
	int		use_common_tx_conf_queue;
	int		l3_chksum_gen;
	int		l4_chksum_gen;
};

/**
 * dpni_set_tx_flow() - Set Tx flow configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	Provides (or returns) the sender's flow ID;
 *	for each new sender set (*flow_id) to 'DPNI_NEW_FLOW_ID' to generate
 *	a new flow_id;	this ID should be used as the QDBIN argument
 *	in enqueue operations
 * @cfg:	Tx flow configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_tx_flow(struct fsl_mc_io			*mc_io,
		     uint32_t				cmd_flags,
		     uint16_t				token,
		     uint16_t				*flow_id,
		     const struct dpni_tx_flow_cfg	*cfg);

/**
 * struct dpni_tx_flow_attr - Structure representing Tx flow attributes
 * @use_common_tx_conf_queue: '1' if using common (default) Tx confirmation and
 *	error queue; '0' if using private Tx confirmation and error queue
 * @l3_chksum_gen: '1' if L3 checksum generation is enabled; '0' if disabled
 * @l4_chksum_gen: '1' if L4 checksum generation is enabled; '0' if disabled
 */
struct dpni_tx_flow_attr {
	int	use_common_tx_conf_queue;
	int	l3_chksum_gen;
	int	l4_chksum_gen;
};

/**
 * dpni_get_tx_flow() - Get Tx flow attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	The sender's flow ID, as returned by the
 *	dpni_set_tx_flow() function
 * @attr:	Returned Tx flow attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_tx_flow(struct fsl_mc_io		*mc_io,
		     uint32_t			cmd_flags,
		     uint16_t			token,
		     uint16_t			flow_id,
		     struct dpni_tx_flow_attr	*attr);

/**
 * struct dpni_tx_conf_cfg - Structure representing Tx conf configuration
 * @errors_only: Set to '1' to report back only error frames;
 *	Set to '0' to confirm transmission/error for all transmitted frames;
 * @queue_cfg: Queue configuration
 */
struct dpni_tx_conf_cfg {
	int			errors_only;
	struct dpni_queue_cfg	queue_cfg;
};

/**
 * dpni_set_tx_conf() - Set Tx confirmation and error queue configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	The sender's flow ID, as returned by the
 *	dpni_set_tx_flow() function;
 *	use 'DPNI_COMMON_TX_CONF' for common tx-conf
 * @cfg:	Queue configuration
 *
 * If either 'DPNI_OPT_TX_CONF_DISABLED' or
 * 'DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED' were selected at DPNI creation,
 * this function can ONLY be used with 'flow_id == DPNI_COMMON_TX_CONF';
 * i.e. only serve the common tx-conf-err queue;
 * if 'DPNI_OPT_TX_CONF_DISABLED' was selected, only error frames are reported
 * back - successfully transmitted frames are not confirmed. Otherwise, all
 * transmitted frames are sent for confirmation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_tx_conf(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     uint16_t		flow_id,
		     const struct dpni_tx_conf_cfg	*cfg);

/**
 * struct dpni_tx_conf_attr - Structure representing Tx conf attributes
 * @errors_only: '1' if only error frames are reported back; '0' if all
 *		transmitted frames are confirmed
 * @queue_attr: Queue attributes
 */
struct dpni_tx_conf_attr {
	int			errors_only;
	struct dpni_queue_attr	queue_attr;
};

/**
 * dpni_get_tx_conf() - Get Tx confirmation and error queue attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	The sender's flow ID, as returned by the
 *	dpni_set_tx_flow() function;
 *	use 'DPNI_COMMON_TX_CONF' for common tx-conf
 * @attr:	Returned tx-conf attributes
 *
 * If either 'DPNI_OPT_TX_CONF_DISABLED' or
 * 'DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED' were selected at DPNI creation,
 * this function can ONLY be used with 'flow_id == DPNI_COMMON_TX_CONF';
 * i.e. only serve the common tx-conf-err queue;
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_tx_conf(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     uint16_t		flow_id,
		     struct dpni_tx_conf_attr	*attr);

/**
 * dpni_set_tx_conf_congestion_notification() - Set Tx conf congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	The sender's flow ID, as returned by the
 *	dpni_set_tx_flow() function;
 *	use 'DPNI_COMMON_TX_CONF' for common tx-conf
 * @cfg:	congestion notification configuration
 *
 * If either 'DPNI_OPT_TX_CONF_DISABLED' or
 * 'DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED' were selected at DPNI creation,
 * this function can ONLY be used with 'flow_id == DPNI_COMMON_TX_CONF';
 * i.e. only serve the common tx-conf-err queue;
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_set_tx_conf_congestion_notification(struct fsl_mc_io	*mc_io,
					     uint32_t		cmd_flags,
					     uint16_t		token,
					     uint16_t		flow_id,
			const struct dpni_congestion_notification_cfg *cfg);

/**
 * dpni_get_tx_conf_congestion_notification() - Get Tx conf congestion
 *	notification configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @flow_id:	The sender's flow ID, as returned by the
 *	dpni_set_tx_flow() function;
 *	use 'DPNI_COMMON_TX_CONF' for common tx-conf
 * @cfg:	congestion notification
 *
 * If either 'DPNI_OPT_TX_CONF_DISABLED' or
 * 'DPNI_OPT_PRIVATE_TX_CONF_ERROR_DISABLED' were selected at DPNI creation,
 * this function can ONLY be used with 'flow_id == DPNI_COMMON_TX_CONF';
 * i.e. only serve the common tx-conf-err queue;
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpni_get_tx_conf_congestion_notification(struct fsl_mc_io	*mc_io,
					     uint32_t		cmd_flags,
					     uint16_t		token,
					     uint16_t		flow_id,
				struct dpni_congestion_notification_cfg *cfg);

/**
 * dpni_set_tx_conf_revoke() - Tx confirmation revocation
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @revoke:	revoke or not
 *
 * This function is useful only when 'DPNI_OPT_TX_CONF_DISABLED' is not
 * selected at DPNI creation.
 * Calling this function with 'revoke' set to '1' disables all transmit
 * confirmation (including the private confirmation queues), regardless of
 * previous settings; Note that in this case, Tx error frames are still
 * enqueued to the general transmit errors queue.
 * Calling this function with 'revoke' set to '0' restores the previous
 * settings for both general and private transmit confirmation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_tx_conf_revoke(struct fsl_mc_io		*mc_io,
			    uint32_t			cmd_flags,
			    uint16_t			token,
			    int			revoke);

/**
 * dpni_set_rx_flow() - Set Rx flow configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7);
 *			use 'DPNI_ALL_TCS' to set all TCs and all flows
 * @flow_id:	Rx flow id within the traffic class; use
 *			'DPNI_ALL_TC_FLOWS' to set all flows within
 *			this tc_id; ignored if tc_id is set to
 *			'DPNI_ALL_TCS';
 * @cfg:	Rx flow configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_rx_flow(struct fsl_mc_io			*mc_io,
		     uint32_t				cmd_flags,
		     uint16_t				token,
		     uint8_t				tc_id,
		     uint16_t				flow_id,
		     const struct dpni_queue_cfg	*cfg);

/**
 * dpni_get_rx_flow() -	Get Rx flow attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @flow_id:	Rx flow id within the traffic class
 * @attr:	Returned Rx flow attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_rx_flow(struct fsl_mc_io		*mc_io,
		     uint32_t			cmd_flags,
		     uint16_t			token,
		     uint8_t			tc_id,
		     uint16_t			flow_id,
		     struct dpni_queue_attr	*attr);

/**
 * dpni_set_rx_err_queue() - Set Rx error queue configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	Queue configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_rx_err_queue(struct fsl_mc_io		*mc_io,
			  uint32_t			cmd_flags,
			  uint16_t			token,
			  const struct dpni_queue_cfg	*cfg);

/**
 * dpni_get_rx_err_queue() - Get Rx error queue attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @attr:	Returned Queue attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_get_rx_err_queue(struct fsl_mc_io		*mc_io,
			  uint32_t			cmd_flags,
			  uint16_t			token,
			  struct dpni_queue_attr	*attr);

/**
 * struct dpni_qos_tbl_cfg - Structure representing QOS table configuration
 * @key_cfg_iova: I/O virtual address of 256 bytes DMA-able memory filled with
 *		key extractions to be used as the QoS criteria by calling
 *		dpni_prepare_key_cfg()
 * @discard_on_miss: Set to '1' to discard frames in case of no match (miss);
 *		'0' to use the 'default_tc' in such cases
 * @default_tc: Used in case of no-match and 'discard_on_miss'= 0
 */
struct dpni_qos_tbl_cfg {
	uint64_t	key_cfg_iova;
	int		discard_on_miss;
	uint8_t	default_tc;
};

/**
 * dpni_set_qos_table() - Set QoS mapping table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	QoS table configuration
 *
 * This function and all QoS-related functions require that
 *'max_tcs > 1' was set at DPNI creation.
 *
 * warning: Before calling this function, call dpni_prepare_key_cfg() to
 *			prepare the key_cfg_iova parameter
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_qos_table(struct fsl_mc_io		*mc_io,
		       uint32_t			cmd_flags,
		       uint16_t				token,
		       const struct dpni_qos_tbl_cfg	*cfg);

/**
 * struct dpni_rule_cfg - Rule configuration for table lookup
 * @key_iova: I/O virtual address of the key (must be in DMA-able memory)
 * @mask_iova: I/O virtual address of the mask (must be in DMA-able memory)
 * @key_size: key and mask size (in bytes)
 */
struct dpni_rule_cfg {
	uint64_t	key_iova;
	uint64_t	mask_iova;
	uint8_t	key_size;
};

/**
 * dpni_add_qos_entry() - Add QoS mapping entry (to select a traffic class)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	QoS rule to add
 * @tc_id:	Traffic class selection (0-7)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_add_qos_entry(struct fsl_mc_io		*mc_io,
		       uint32_t			cmd_flags,
		       uint16_t				token,
		       const struct dpni_rule_cfg	*cfg,
		       uint8_t				tc_id);

/**
 * dpni_remove_qos_entry() - Remove QoS mapping entry
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @cfg:	QoS rule to remove
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_remove_qos_entry(struct fsl_mc_io		*mc_io,
			  uint32_t			cmd_flags,
			  uint16_t			token,
			  const struct dpni_rule_cfg	*cfg);

/**
 * dpni_clear_qos_table() - Clear all QoS mapping entries
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 *
 * Following this function call, all frames are directed to
 * the default traffic class (0)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_clear_qos_table(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			 uint16_t		token);

/**
 * dpni_add_fs_entry() - Add Flow Steering entry for a specific traffic class
 *			(to select a flow ID)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	Flow steering rule to add
 * @flow_id:	Flow id selection (must be smaller than the
 *			distribution size of the traffic class)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_add_fs_entry(struct fsl_mc_io			*mc_io,
		      uint32_t				cmd_flags,
		      uint16_t				token,
		      uint8_t				tc_id,
		      const struct dpni_rule_cfg	*cfg,
		      uint16_t				flow_id);

/**
 * dpni_remove_fs_entry() - Remove Flow Steering entry from a specific
 *			traffic class
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 * @cfg:	Flow steering rule to remove
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_remove_fs_entry(struct fsl_mc_io		*mc_io,
			 uint32_t			cmd_flags,
			 uint16_t			token,
			 uint8_t			tc_id,
			 const struct dpni_rule_cfg	*cfg);

/**
 * dpni_clear_fs_entries() - Clear all Flow Steering entries of a specific
 *			traffic class
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @tc_id:	Traffic class selection (0-7)
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_clear_fs_entries(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		tc_id);

/**
 * dpni_set_vlan_insertion() - Enable/disable VLAN insertion for egress frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Requires that the 'DPNI_OPT_VLAN_MANIPULATION' option is set
 * at DPNI creation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_vlan_insertion(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    int		en);

/**
 * dpni_set_vlan_removal() - Enable/disable VLAN removal for ingress frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Requires that the 'DPNI_OPT_VLAN_MANIPULATION' option is set
 * at DPNI creation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_vlan_removal(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  int			en);

/**
 * dpni_set_ipr() - Enable/disable IP reassembly of ingress frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Requires that the 'DPNI_OPT_IPR' option is set at DPNI creation.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_ipr(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 int			en);

/**
 * dpni_set_ipf() - Enable/disable IP fragmentation of egress frames
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPNI object
 * @en:		Set to '1' to enable; '0' to disable
 *
 * Requires that the 'DPNI_OPT_IPF' option is set at DPNI
 * creation. Fragmentation is performed according to MTU value
 * set by dpni_set_mtu() function
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpni_set_ipf(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 int			en);

#endif /* __FSL_DPNI_H */
