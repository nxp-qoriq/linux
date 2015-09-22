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
#ifndef __FSL_DPDMUX_H
#define __FSL_DPDMUX_H

#include "../../fsl-mc/include/net.h"

struct fsl_mc_io;

/* Data Path Demux API
 * Contains API for handling DPDMUX topology and functionality
 */

/**
 * dpdmux_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpdmux_id:		DPDMUX unique ID
 * @token:		Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpdmux_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_open(struct fsl_mc_io	 *mc_io,
		uint32_t		 cmd_flags,
		int			 dpdmux_id,
		uint16_t		 *token);

/**
 * dpdmux_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMUX object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_close(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * DPDMUX general options
 */

/**
 * Enable bridging between internal interfaces
 */
#define DPDMUX_OPT_BRIDGE_EN		0x0000000000000002ULL

#define DPDMUX_IRQ_INDEX_IF			0x0000
#define DPDMUX_IRQ_INDEX		0x0001

/**
 * IRQ event - Indicates that the link state changed
 */
#define DPDMUX_IRQ_EVENT_LINK_CHANGED	0x0001

/**
 * enum dpdmux_manip - DPDMUX manipulation operations
 * @DPDMUX_MANIP_NONE:	No manipulation on frames
 * @DPDMUX_MANIP_ADD_REMOVE_S_VLAN: Add S-VLAN on egress, remove it on ingress
 */
enum dpdmux_manip {
	DPDMUX_MANIP_NONE = 0x0,
	DPDMUX_MANIP_ADD_REMOVE_S_VLAN = 0x1
};

/**
 * enum dpdmux_method - DPDMUX method options
 * @DPDMUX_METHOD_NONE: no DPDMUX method
 * @DPDMUX_METHOD_C_VLAN_MAC: DPDMUX based on C-VLAN and MAC address
 * @DPDMUX_METHOD_MAC: DPDMUX based on MAC address
 * @DPDMUX_METHOD_C_VLAN: DPDMUX based on C-VLAN
 * @DPDMUX_METHOD_S_VLAN: DPDMUX based on S-VLAN
 */
enum dpdmux_method {
	DPDMUX_METHOD_NONE = 0x0,
	DPDMUX_METHOD_C_VLAN_MAC = 0x1,
	DPDMUX_METHOD_MAC = 0x2,
	DPDMUX_METHOD_C_VLAN = 0x3,
	DPDMUX_METHOD_S_VLAN = 0x4
};

/**
 * struct dpdmux_cfg - DPDMUX configuration parameters
 * @method: Defines the operation method for the DPDMUX address table
 * @manip: Required manipulation operation
 * @num_ifs: Number of interfaces (excluding the uplink interface)
 * @adv: Advanced parameters; default is all zeros;
 *	 use this structure to change default settings
 */
struct dpdmux_cfg {
	enum dpdmux_method	method;
	enum dpdmux_manip	manip;
	uint16_t		num_ifs;
	/**
	 * struct adv - Advanced parameters
	 * @options: DPDMUX options - combination of 'DPDMUX_OPT_<X>' flags
	 * @max_dmat_entries: Maximum entries in DPDMUX address table
	 *		0 - indicates default: 64 entries per interface.
	 * @max_mc_groups: Number of multicast groups in DPDMUX table
	 *		0 - indicates default: 32 multicast groups
	 * @max_vlan_ids: max vlan ids allowed in the system -
	 *		relevant only case of working in mac+vlan method.
	 *		0 - indicates default 16 vlan ids.
	 */
	struct {
		uint64_t options;
		uint16_t max_dmat_entries;
		uint16_t max_mc_groups;
		uint16_t max_vlan_ids;
	} adv;
};

/**
 * dpdmux_create() - Create the DPDMUX object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @token:	Returned token; use in subsequent API calls
 *
 * Create the DPDMUX object, allocate required resources and
 * perform required initialization.
 *
 * The object can be created either by declaring it in the
 * DPL file, or by calling this function.
 *
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent calls to
 * this specific object. For objects that are created using the
 * DPL file, call dpdmux_open() function to get an authentication
 * token first.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_create(struct fsl_mc_io		*mc_io,
		  uint32_t			cmd_flags,
		  const struct dpdmux_cfg	*cfg,
		  uint16_t			*token);

/**
 * dpdmux_destroy() - Destroy the DPDMUX object and release all its resources.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpdmux_destroy(struct fsl_mc_io	*mc_io,
		   uint32_t		cmd_flags,
		   uint16_t		token);

/**
 * dpdmux_enable() - Enable DPDMUX functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_enable(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token);

/**
 * dpdmux_disable() - Disable DPDMUX functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_disable(struct fsl_mc_io	*mc_io,
		   uint32_t		cmd_flags,
		   uint16_t		token);

/**
 * dpdmux_is_enabled() - Check if the DPDMUX is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_is_enabled(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      int		*en);

/**
 * dpdmux_reset() - Reset the DPDMUX, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_reset(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * struct dpdmux_irq_cfg - IRQ configuration
 * @addr:	Address that must be written to signal a message-based interrupt
 * @val:	Value to write into irq_addr address
 * @irq_num: A user defined number associated with this IRQ
 */
struct dpdmux_irq_cfg {
	     uint64_t		addr;
	     uint32_t		val;
	     int		irq_num;
};

/**
 * dpdmux_set_irq() - Set IRQ information for the DPDMUX to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq(struct fsl_mc_io		*mc_io,
		   uint32_t			cmd_flags,
		   uint16_t			token,
		   uint8_t			irq_index,
		   struct dpdmux_irq_cfg	*irq_cfg);

/**
 * dpdmux_get_irq() - Get IRQ information from the DPDMUX.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq(struct fsl_mc_io		*mc_io,
		   uint32_t			cmd_flags,
		   uint16_t			token,
		   uint8_t			irq_index,
		   int				*type,
		   struct dpdmux_irq_cfg	*irq_cfg);

/**
 * dpdmux_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		en);

/**
 * dpdmux_get_irq_enable() - Get overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		*en);

/**
 * dpdmux_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @mask:	event mask to trigger interrupt;
 *		each bit:
 *			0 = ignore event
 *			1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_set_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		mask);

/**
 * dpdmux_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		*mask);

/**
 * dpdmux_get_irq_status() - Get the current status of any pending interrupts.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_irq_status(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint32_t		*status);

/**
 * dpdmux_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_clear_irq_status(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    uint8_t		irq_index,
			    uint32_t		status);

/**
 * struct dpdmux_attr - Structure representing DPDMUX attributes
 * @id: DPDMUX object ID
 * @version: DPDMUX version
 * @options: Configuration options (bitmap)
 * @method: DPDMUX address table method
 * @manip: DPDMUX manipulation type
 * @num_ifs: Number of interfaces (excluding the uplink interface)
 * @mem_size: DPDMUX frame storage memory size
 */
struct dpdmux_attr {
	int			id;
	/**
	 * struct version - DPDMUX version
	 * @major: DPDMUX major version
	 * @minor: DPDMUX minor version
	 */
	struct {
		uint16_t	major;
		uint16_t	minor;
	} version;
	uint64_t		options;
	enum dpdmux_method	method;
	enum dpdmux_manip	manip;
	uint16_t		num_ifs;
	uint16_t		mem_size;
};

/**
 * dpdmux_get_attributes() - Retrieve DPDMUX attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_get_attributes(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  struct dpdmux_attr	*attr);

/**
 * dpdmux_ul_set_max_frame_length() - Set the maximum frame length in DPDMUX
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMUX object
 * @max_frame_length:	The required maximum frame length
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_ul_set_max_frame_length(struct fsl_mc_io	*mc_io,
				   uint32_t		cmd_flags,
				   uint16_t		token,
				   uint16_t		max_frame_length);

/**
 * enum dpdmux_counter_type - Counter types
 * @DPDMUX_CNT_ING_FRAME: Counts ingress frames
 * @DPDMUX_CNT_ING_BYTE: Counts ingress bytes
 * @DPDMUX_CNT_ING_FLTR_FRAME: Counts filtered ingress frames
 * @DPDMUX_CNT_ING_FRAME_DISCARD: Counts discarded ingress frames
 * @DPDMUX_CNT_ING_MCAST_FRAME: Counts ingress multicast frames
 * @DPDMUX_CNT_ING_MCAST_BYTE: Counts ingress multicast bytes
 * @DPDMUX_CNT_ING_BCAST_FRAME: Counts ingress broadcast frames
 * @DPDMUX_CNT_ING_BCAST_BYTES: Counts ingress broadcast bytes
 * @DPDMUX_CNT_EGR_FRAME: Counts egress frames
 * @DPDMUX_CNT_EGR_BYTE: Counts egress bytes
 * @DPDMUX_CNT_EGR_FRAME_DISCARD: Counts discarded egress frames
 */
enum dpdmux_counter_type {
	DPDMUX_CNT_ING_FRAME = 0x0,
	DPDMUX_CNT_ING_BYTE = 0x1,
	DPDMUX_CNT_ING_FLTR_FRAME = 0x2,
	DPDMUX_CNT_ING_FRAME_DISCARD = 0x3,
	DPDMUX_CNT_ING_MCAST_FRAME = 0x4,
	DPDMUX_CNT_ING_MCAST_BYTE = 0x5,
	DPDMUX_CNT_ING_BCAST_FRAME = 0x6,
	DPDMUX_CNT_ING_BCAST_BYTES = 0x7,
	DPDMUX_CNT_EGR_FRAME = 0x8,
	DPDMUX_CNT_EGR_BYTE = 0x9,
	DPDMUX_CNT_EGR_FRAME_DISCARD = 0xa
};

/**
 * enum dpdmux_accepted_frames_type - DPDMUX frame types
 * @DPDMUX_ADMIT_ALL: The device accepts VLAN tagged, untagged and
 *			priority-tagged frames
 * @DPDMUX_ADMIT_ONLY_VLAN_TAGGED: The device discards untagged frames or
 *				priority-tagged frames that are received on this
 *				interface
 * @DPDMUX_ADMIT_ONLY_UNTAGGED: Untagged frames or priority-tagged frames
 *				received on this interface are accepted
 */
enum dpdmux_accepted_frames_type {
	DPDMUX_ADMIT_ALL = 0,
	DPDMUX_ADMIT_ONLY_VLAN_TAGGED = 1,
	DPDMUX_ADMIT_ONLY_UNTAGGED = 2
};

/**
 * enum dpdmux_action - DPDMUX action for un-accepted frames
 * @DPDMUX_ACTION_DROP: Drop un-accepted frames
 * @DPDMUX_ACTION_REDIRECT_TO_CTRL: Redirect un-accepted frames to the
 *					control interface
 */
enum dpdmux_action {
	DPDMUX_ACTION_DROP = 0,
	DPDMUX_ACTION_REDIRECT_TO_CTRL = 1
};

/**
 * struct dpdmux_accepted_frames - Frame types configuration
 * @type: Defines ingress accepted frames
 * @unaccept_act: Defines action on frames not accepted
 */
struct dpdmux_accepted_frames {
	enum dpdmux_accepted_frames_type	type;
	enum dpdmux_action			unaccept_act;
};

/**
 * dpdmux_if_set_accepted_frames() - Set the accepted frame types
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Interface ID (0 for uplink, or 1-num_ifs);
 * @cfg:	Frame types configuration
 *
 * if 'DPDMUX_ADMIT_ONLY_VLAN_TAGGED' is set - untagged frames or
 * priority-tagged frames are discarded.
 * if 'DPDMUX_ADMIT_ONLY_UNTAGGED' is set - untagged frames or
 * priority-tagged frames are accepted.
 * if 'DPDMUX_ADMIT_ALL' is set (default mode) - all VLAN tagged,
 * untagged and priority-tagged frame are accepted;
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_set_accepted_frames(struct fsl_mc_io		      *mc_io,
				  uint32_t			      cmd_flags,
				  uint16_t			      token,
				  uint16_t			      if_id,
				  const struct dpdmux_accepted_frames *cfg);

/**
 * struct dpdmux_if_attr - Structure representing frame types configuration
 * @rate: Configured interface rate (in bits per second)
 * @enabled: Indicates if interface is enabled
 * @accept_frame_type: Indicates type of accepted frames for the interface
 */
struct dpdmux_if_attr {
	uint32_t				rate;
	int					enabled;
	enum dpdmux_accepted_frames_type	accept_frame_type;
};

/**
 * dpdmux_if_get_attributes() - Obtain DPDMUX interface attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Interface ID (0 for uplink, or 1-num_ifs);
 * @attr:	Interface attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_get_attributes(struct fsl_mc_io		*mc_io,
			     uint32_t			cmd_flags,
			     uint16_t			token,
			     uint16_t			if_id,
			     struct dpdmux_if_attr	*attr);

/**
 * struct dpdmux_l2_rule - Structure representing L2 rule
 * @mac_addr: MAC address
 * @vlan_id: VLAN ID
 */
struct dpdmux_l2_rule {
	uint8_t	mac_addr[6];
	uint16_t	vlan_id;
};

/**
 * dpdmux_if_remove_l2_rule() - Remove L2 rule from DPDMUX table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Destination interface ID
 * @rule:	L2 rule
 *
 * Function removes a L2 rule from DPDMUX table
 * or adds an interface to an existing multicast address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_remove_l2_rule(struct fsl_mc_io			*mc_io,
			     uint32_t				cmd_flags,
			     uint16_t				token,
			     uint16_t				if_id,
			     const struct dpdmux_l2_rule	*rule);

/**
 * dpdmux_if_add_l2_rule() - Add L2 rule into DPDMUX table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMUX object
 * @if_id:	Destination interface ID
 * @rule:	L2 rule
 *
 * Function adds a L2 rule into DPDMUX table
 * or adds an interface to an existing multicast address
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_add_l2_rule(struct fsl_mc_io		*mc_io,
			  uint32_t			cmd_flags,
			  uint16_t			token,
			  uint16_t			if_id,
			  const struct dpdmux_l2_rule	*rule);

/**
* dpdmux_if_get_counter() - Functions obtains specific counter of an interface
* @mc_io: Pointer to MC portal's I/O object
* @cmd_flags: Command flags; one or more of 'MC_CMD_FLAG_'
* @token: Token of DPDMUX object
* @if_id:  Interface Id
* @counter_type: counter type
* @counter: Returned specific counter information
*
* Return:	'0' on Success; Error code otherwise.
*/
int dpdmux_if_get_counter(struct fsl_mc_io		*mc_io,
			  uint32_t			cmd_flags,
			  uint16_t			token,
			  uint16_t			if_id,
			  enum dpdmux_counter_type	counter_type,
			  uint64_t			*counter);

/**
* dpdmux_ul_reset_counters() - Function resets the uplink counter
* @mc_io:	Pointer to MC portal's I/O object
* @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
* @token:	Token of DPDMUX object
*
* Return:	'0' on Success; Error code otherwise.
*/
int dpdmux_ul_reset_counters(struct fsl_mc_io	*mc_io,
			     uint32_t		cmd_flags,
			     uint16_t		token);

/**
 * Enable auto-negotiation
 */
#define DPDMUX_LINK_OPT_AUTONEG		0x0000000000000001ULL
/**
 * Enable half-duplex mode
 */
#define DPDMUX_LINK_OPT_HALF_DUPLEX	0x0000000000000002ULL
/**
 * Enable pause frames
 */
#define DPDMUX_LINK_OPT_PAUSE		0x0000000000000004ULL
/**
 * Enable a-symmetric pause frames
 */
#define DPDMUX_LINK_OPT_ASYM_PAUSE	0x0000000000000008ULL

/**
 * struct dpdmux_link_cfg - Structure representing DPDMUX link configuration
 * @rate: Rate
 * @options: Mask of available options; use 'DPDMUX_LINK_OPT_<X>' values
 */
struct dpdmux_link_cfg {
	uint32_t rate;
	uint64_t options;
};

/**
 * dpdmux_if_set_link_cfg() - set the link configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @cfg: Link configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmux_if_set_link_cfg(struct fsl_mc_io		*mc_io,
			   uint32_t			cmd_flags,
			   uint16_t			token,
			   uint16_t			if_id,
			   struct dpdmux_link_cfg	*cfg);
/**
 * struct dpdmux_link_state - Structure representing DPDMUX link state
 * @rate: Rate
 * @options: Mask of available options; use 'DPDMUX_LINK_OPT_<X>' values
 * @up: 0 - down, 1 - up
 */
struct dpdmux_link_state {
	uint32_t rate;
	uint64_t options;
	int      up;
};

/**
 * dpdmux_if_get_link_state - Return the link state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token: Token of DPSW object
 * @if_id: interface id
 * @state: link state
 *
 * @returns	'0' on Success; Error code otherwise.
 */
int dpdmux_if_get_link_state(struct fsl_mc_io		*mc_io,
			     uint32_t			cmd_flags,
			     uint16_t			token,
			     uint16_t			if_id,
			     struct dpdmux_link_state	*state);

#endif /* __FSL_DPDMUX_H */
