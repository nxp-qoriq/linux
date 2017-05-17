/* Copyright 2013-2015 Freescale Semiconductor, Inc.
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
#ifndef __FSL_DPDMAI_H
#define __FSL_DPDMAI_H

struct fsl_mc_io;

/* Data Path DMA Interface API
 * Contains initialization APIs and runtime control APIs for DPDMAI
 */

/* General DPDMAI macros */

/**
 * Maximum number of Tx/Rx priorities per DPDMAI object
 */
#define DPDMAI_PRIO_NUM		2

/**
 * All queues considered; see dpdmai_set_rx_queue()
 */
#define DPDMAI_ALL_QUEUES	(uint8_t)(-1)

/**
 * dpdmai_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpdmai_id:	DPDMAI unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpdmai_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_open(struct fsl_mc_io	*mc_io,
		uint32_t		cmd_flags,
		int			dpdmai_id,
		uint16_t		*token);

/**
 * dpdmai_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_close(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * struct dpdmai_cfg - Structure representing DPDMAI configuration
 * @priorities: Priorities for the DMA hardware processing; valid priorities are
 *	configured with values 1-8; the entry following last valid entry
 *	should be configured with 0
 */
struct dpdmai_cfg {
	uint8_t priorities[DPDMAI_PRIO_NUM];
};

/**
 * dpdmai_create() - Create the DPDMAI object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @token:	Returned token; use in subsequent API calls
 *
 * Create the DPDMAI object, allocate required resources and
 * perform required initialization.
 *
 * The object can be created either by declaring it in the
 * DPL file, or by calling this function.
 *
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent calls to
 * this specific object. For objects that are created using the
 * DPL file, call dpdmai_open() function to get an authentication
 * token first.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_create(struct fsl_mc_io		*mc_io,
		  uint32_t			cmd_flags,
		  const struct dpdmai_cfg	*cfg,
		  uint16_t			*token);

/**
 * dpdmai_destroy() - Destroy the DPDMAI object and release all its resources.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpdmai_destroy(struct fsl_mc_io	*mc_io,
		   uint32_t		cmd_flags,
		   uint16_t		token);

/**
 * dpdmai_enable() - Enable the DPDMAI, allow sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_enable(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token);

/**
 * dpdmai_disable() - Disable the DPDMAI, stop sending and receiving frames.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_disable(struct fsl_mc_io	*mc_io,
		   uint32_t		cmd_flags,
		   uint16_t		token);

/**
 * dpdmai_is_enabled() - Check if the DPDMAI is enabled.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_is_enabled(struct fsl_mc_io	*mc_io,
		      uint32_t		cmd_flags,
		      uint16_t		token,
		      int		*en);

/**
 * dpdmai_reset() - Reset the DPDMAI, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_reset(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * struct dpdmai_irq_cfg - IRQ configuration
 * @addr:	Address that must be written to signal a message-based interrupt
 * @val:	Value to write into irq_addr address
 * @irq_num: A user defined number associated with this IRQ
 */
struct dpdmai_irq_cfg {
	     uint64_t		addr;
	     uint32_t		val;
	     int		irq_num;
};

/**
 * dpdmai_set_irq() - Set IRQ information for the DPDMAI to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_set_irq(struct fsl_mc_io		*mc_io,
		   uint32_t			cmd_flags,
		   uint16_t			token,
		   uint8_t			irq_index,
		   struct dpdmai_irq_cfg	*irq_cfg);

/**
 * dpdmai_get_irq() - Get IRQ information from the DPDMAI
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_irq(struct fsl_mc_io		*mc_io,
		   uint32_t			cmd_flags,
		   uint16_t			token,
		   uint8_t			irq_index,
		   int				*type,
		   struct dpdmai_irq_cfg	*irq_cfg);

/**
 * dpdmai_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @en:			Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_set_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		en);

/**
 * dpdmai_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @en:			Returned Interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_irq_enable(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint8_t		*en);

/**
 * dpdmai_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @mask:		event mask to trigger interrupt;
 *				each bit:
 *					0 = ignore event
 *					1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_set_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t		irq_index,
			uint32_t		mask);

/**
 * dpdmai_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @mask:		Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_irq_mask(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t		irq_index,
			uint32_t		*mask);

/**
 * dpdmai_get_irq_status() - Get the current status of any pending interrupts
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @status:		Returned interrupts status - one bit per cause:
 *					0 = no interrupt pending
 *					1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_irq_status(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint32_t		*status);

/**
 * dpdmai_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_clear_irq_status(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    uint8_t		irq_index,
			    uint32_t		status);

/**
 * struct dpdmai_attr - Structure representing DPDMAI attributes
 * @id: DPDMAI object ID
 * @version: DPDMAI version
 * @num_of_priorities: number of priorities
 */
struct dpdmai_attr {
	int	id;
	/**
	 * struct version - DPDMAI version
	 * @major: DPDMAI major version
	 * @minor: DPDMAI minor version
	 */
	struct {
		uint16_t major;
		uint16_t minor;
	} version;
	uint8_t num_of_priorities;
};

/**
 * dpdmai_get_attributes() - Retrieve DPDMAI attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_attributes(struct fsl_mc_io	*mc_io,
			  uint32_t		cmd_flags,
			  uint16_t		token,
			  struct dpdmai_attr	*attr);

/**
 * enum dpdmai_dest - DPDMAI destination types
 * @DPDMAI_DEST_NONE: Unassigned destination; The queue is set in parked mode
 *	and does not generate FQDAN notifications; user is expected to dequeue
 *	from the queue based on polling or other user-defined method
 * @DPDMAI_DEST_DPIO: The queue is set in schedule mode and generates FQDAN
 *	notifications to the specified DPIO; user is expected to dequeue
 *	from the queue only after notification is received
 * @DPDMAI_DEST_DPCON: The queue is set in schedule mode and does not generate
 *	FQDAN notifications, but is connected to the specified DPCON object;
 *	user is expected to dequeue from the DPCON channel
 */
enum dpdmai_dest {
	DPDMAI_DEST_NONE = 0,
	DPDMAI_DEST_DPIO = 1,
	DPDMAI_DEST_DPCON = 2
};

/**
 * struct dpdmai_dest_cfg - Structure representing DPDMAI destination parameters
 * @dest_type: Destination type
 * @dest_id: Either DPIO ID or DPCON ID, depending on the destination type
 * @priority: Priority selection within the DPIO or DPCON channel; valid values
 *	are 0-1 or 0-7, depending on the number of priorities in that
 *	channel; not relevant for 'DPDMAI_DEST_NONE' option
 */
struct dpdmai_dest_cfg {
	enum dpdmai_dest	dest_type;
	int			dest_id;
	uint8_t		priority;
};

/* DPDMAI queue modification options */

/**
 * Select to modify the user's context associated with the queue
 */
#define DPDMAI_QUEUE_OPT_USER_CTX	0x00000001

/**
 * Select to modify the queue's destination
 */
#define DPDMAI_QUEUE_OPT_DEST		0x00000002

/**
 * struct dpdmai_rx_queue_cfg - DPDMAI RX queue configuration
 * @options: Flags representing the suggested modifications to the queue;
 *	Use any combination of 'DPDMAI_QUEUE_OPT_<X>' flags
 * @user_ctx: User context value provided in the frame descriptor of each
 *	dequeued frame;
 *	valid only if 'DPDMAI_QUEUE_OPT_USER_CTX' is contained in 'options'
 * @dest_cfg: Queue destination parameters;
 *	valid only if 'DPDMAI_QUEUE_OPT_DEST' is contained in 'options'
 */
struct dpdmai_rx_queue_cfg {
	uint32_t		options;
	uint64_t		user_ctx;
	struct dpdmai_dest_cfg	dest_cfg;

};

/**
 * dpdmai_set_rx_queue() - Set Rx queue configuration
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @priority:	Select the queue relative to number of
 *			priorities configured at DPDMAI creation; use
 *			DPDMAI_ALL_QUEUES to configure all Rx queues
 *			identically.
 * @cfg:	Rx queue configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_set_rx_queue(struct fsl_mc_io			*mc_io,
			uint32_t				cmd_flags,
			uint16_t				token,
			uint8_t					priority,
			const struct dpdmai_rx_queue_cfg	*cfg);

/**
 * struct dpdmai_rx_queue_attr - Structure representing attributes of Rx queues
 * @user_ctx:  User context value provided in the frame descriptor of each
 *	 dequeued frame
 * @dest_cfg: Queue destination configuration
 * @fqid: Virtual FQID value to be used for dequeue operations
 */
struct dpdmai_rx_queue_attr {
	uint64_t		user_ctx;
	struct dpdmai_dest_cfg	dest_cfg;
	uint32_t		fqid;
};

/**
 * dpdmai_get_rx_queue() - Retrieve Rx queue attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @priority:	Select the queue relative to number of
 *				priorities configured at DPDMAI creation
 * @attr:	Returned Rx queue attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_rx_queue(struct fsl_mc_io		*mc_io,
			uint32_t			cmd_flags,
			uint16_t			token,
			uint8_t				priority,
			struct dpdmai_rx_queue_attr	*attr);

/**
 * struct dpdmai_tx_queue_attr - Structure representing attributes of Tx queues
 * @fqid: Virtual FQID to be used for sending frames to DMA hardware
 */

struct dpdmai_tx_queue_attr {
	uint32_t fqid;
};

/**
 * dpdmai_get_tx_queue() - Retrieve Tx queue attributes.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPDMAI object
 * @priority:	Select the queue relative to number of
 *			priorities configured at DPDMAI creation
 * @attr:	Returned Tx queue attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpdmai_get_tx_queue(struct fsl_mc_io		*mc_io,
			uint32_t			cmd_flags,
			uint16_t			token,
			uint8_t				priority,
			struct dpdmai_tx_queue_attr	*attr);

#endif /* __FSL_DPDMAI_H */
