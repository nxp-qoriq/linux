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
#ifndef __FSL_DPRTC_H
#define __FSL_DPRTC_H

/* Data Path Real Time Counter API
 * Contains initialization APIs and runtime control APIs for RTC
 */

struct fsl_mc_io;

/**
 * Number of irq's
 */
#define DPRTC_MAX_IRQ_NUM			1
#define DPRTC_IRQ_INDEX				0

/**
 * Interrupt event masks:
 */

/**
 * Interrupt event mask indicating alarm event had occurred
 */
#define DPRTC_EVENT_ALARM			0x40000000
/**
 * Interrupt event mask indicating periodic pulse event had occurred
 */
#define DPRTC_EVENT_PPS				0x08000000

/**
 * dprtc_open() - Open a control session for the specified object.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dprtc_id:	DPRTC unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dprtc_create function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_open(struct fsl_mc_io	*mc_io,
	       uint32_t		cmd_flags,
	      int		dprtc_id,
	      uint16_t		*token);

/**
 * dprtc_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_close(struct fsl_mc_io	*mc_io,
		uint32_t		cmd_flags,
	       uint16_t	token);

/**
 * struct dprtc_cfg - Structure representing DPRTC configuration
 * @options:	place holder
 */
struct dprtc_cfg {
	uint32_t options;
};

/**
 * dprtc_create() - Create the DPRTC object.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @cfg:	Configuration structure
 * @token:	Returned token; use in subsequent API calls
 *
 * Create the DPRTC object, allocate required resources and
 * perform required initialization.
 *
 * The object can be created either by declaring it in the
 * DPL file, or by calling this function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent calls to
 * this specific object. For objects that are created using the
 * DPL file, call dprtc_open function to get an authentication
 * token first.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_create(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		const struct dprtc_cfg	*cfg,
		uint16_t		*token);

/**
 * dprtc_destroy() - Destroy the DPRTC object and release all its resources.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dprtc_destroy(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		 uint16_t		token);

/**
 * dprtc_set_clock_offset() - Sets the clock's offset
 * (usually relative to another clock).
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @offset: New clock offset (in nanoseconds).
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_clock_offset(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
		  uint16_t token,
		  int64_t offset);

/**
 * dprtc_set_freq_compensation() - Sets a new frequency compensation value.
 *
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPRTC object
 * @freq_compensation:
 *				The new frequency compensation value to set.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_freq_compensation(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
		  uint16_t token,
		  uint32_t freq_compensation);

/**
 * dprtc_get_freq_compensation() - Retrieves the frequency compensation value
 *
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPRTC object
 * @freq_compensation:
 *				Frequency compensation value
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_freq_compensation(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
		  uint16_t token,
		  uint32_t *freq_compensation);

/**
 * dprtc_get_time() - Returns the current RTC time.
 *
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPRTC object
 * @timestamp:	Current RTC timestamp.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_time(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		  uint16_t token,
		  uint64_t *timestamp);

/**
 * dprtc_set_time() - Updates current RTC time.
 *
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPRTC object
 * @timestamp:	New RTC timestamp.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_time(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		  uint16_t token,
		  uint64_t timestamp);

/**
 * dprtc_set_alarm() - Defines and sets alarm.
 *
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPRTC object
 * @time:		In nanoseconds, the time when the alarm
 *				should go off - must be a multiple of
 *				1 microsecond
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_alarm(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		  uint16_t token,
		  uint64_t time);

/**
 * struct dprtc_irq_cfg - IRQ configuration
 * @addr:	Address that must be written to signal a message-based interrupt
 * @val:	Value to write into irq_addr address
 * @irq_num: A user defined number associated with this IRQ
 */
struct dprtc_irq_cfg {
	     uint64_t		addr;
	     uint32_t		val;
	     int		irq_num;
};

/**
 * dprtc_set_irq() - Set IRQ information for the DPRTC to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 struct dprtc_irq_cfg	*irq_cfg);

/**
 * dprtc_get_irq() - Get IRQ information from the DPRTC.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 int			*type,
		 struct dprtc_irq_cfg	*irq_cfg);

/**
 * dprtc_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @en:	Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_irq_enable(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint8_t			en);

/**
 * dprtc_get_irq_enable() - Get overall interrupt state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @en:		Returned interrupt state - enable = 1, disable = 0
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_irq_enable(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint8_t			*en);

/**
 * dprtc_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @mask:	Event mask to trigger interrupt;
 *			each bit:
 *				0 = ignore event
 *				1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_set_irq_mask(struct fsl_mc_io	*mc_io,
		       uint32_t		cmd_flags,
		      uint16_t		token,
		      uint8_t		irq_index,
		      uint32_t		mask);

/**
 * dprtc_get_irq_mask() - Get interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @mask:	Returned event mask to trigger interrupt
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_irq_mask(struct fsl_mc_io	*mc_io,
		       uint32_t		cmd_flags,
		      uint16_t		token,
		      uint8_t		irq_index,
		      uint32_t		*mask);

/**
 * dprtc_get_irq_status() - Get the current status of any pending interrupts.
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_irq_status(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			uint16_t		token,
			uint8_t			irq_index,
			uint32_t		*status);

/**
 * dprtc_clear_irq_status() - Clear a pending interrupt's status
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @irq_index:	The interrupt index to configure
 * @status:	Bits to clear (W1C) - one bit per cause:
 *					0 = don't change
 *					1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_clear_irq_status(struct fsl_mc_io	*mc_io,
			   uint32_t		cmd_flags,
			  uint16_t		token,
			  uint8_t		irq_index,
			  uint32_t		status);

/**
 * struct dprtc_attr - Structure representing DPRTC attributes
 * @id:		DPRTC object ID
 * @version:	DPRTC version
 */
struct dprtc_attr {
	int id;
	/**
	 * struct version - Structure representing DPRTC version
	 * @major:	DPRTC major version
	 * @minor:	DPRTC minor version
	 */
	struct {
		uint16_t major;
		uint16_t minor;
	} version;
};

/**
 * dprtc_get_attributes - Retrieve DPRTC attributes.
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPRTC object
 * @attr:	Returned object's attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dprtc_get_attributes(struct fsl_mc_io	*mc_io,
			 uint32_t	cmd_flags,
			uint16_t		token,
			struct dprtc_attr	*attr);

#endif /* __FSL_DPRTC_H */
