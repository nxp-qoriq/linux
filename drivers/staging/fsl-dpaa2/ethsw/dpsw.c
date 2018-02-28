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
#include "../../fsl-mc/include/mc.h"
#include "dpsw.h"
#include "dpsw-cmd.h"

static void build_if_id_bitmap(__le64 *bmap,
			       const u16 *id,
			       const u16 num_ifs) {
	int i;

	for (i = 0; (i < num_ifs) && (i < DPSW_MAX_IF); i++)
		bmap[id[i] / 64] = dpsw_set_bit(bmap[id[i] / 64],
						(id[i] % 64),
						1);
}

static void read_if_id_bitmap(u16 *if_id,
			      u16 *num_ifs,
			      __le64 *bmap) {
	int bitmap[DPSW_MAX_IF] = { 0 };
	int i, j = 0;
	int count = 0;

	for (i = 0; i < DPSW_MAX_IF; i++) {
		bitmap[i] = dpsw_get_bit(le64_to_cpu(bmap[i / 64]),
					 i % 64);
		count += bitmap[i];
	}

	*num_ifs = (u16)count;

	for (i = 0; (i < DPSW_MAX_IF) && (j < count); i++) {
		if (bitmap[i]) {
			if_id[j] = (u16)i;
			j++;
		}
	}
}

/**
 * dpsw_open() - Open a control session for the specified object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @dpsw_id:	DPSW unique ID
 * @token:	Returned token; use in subsequent API calls
 *
 * This function can be used to open a control session for an
 * already created object; an object may have been declared in
 * the DPL or by calling the dpsw_create() function.
 * This function returns a unique authentication token,
 * associated with the specific object ID and the specific MC
 * portal; this token must be used in all subsequent commands for
 * this specific object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_open(struct fsl_mc_io *mc_io,
	      u32 cmd_flags,
	      int dpsw_id,
	      u16 *token)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_open *cmd_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_OPEN,
					  cmd_flags,
					  0);
	cmd_params = (struct dpsw_cmd_open *)cmd.params;
	cmd_params->dpsw_id = cpu_to_le32(dpsw_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = mc_cmd_hdr_read_token(&cmd);

	return 0;
}

/**
 * dpsw_close() - Close the control session of the object
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * After this function is called, no further operations are
 * allowed on the object without opening a new control session.
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_close(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_enable() - Enable DPSW functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_enable(struct fsl_mc_io *mc_io,
		u32 cmd_flags,
		u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_disable() - Disable DPSW functionality
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_disable(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_is_enabled() - Check if the DPSW is enabled
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @en:		Returns '1' if object is enabled; '0' otherwise
 *
 * Return:	'0' on Success; Error code otherwise
 */
int dpsw_is_enabled(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    int *en)
{
	struct mc_command cmd = { 0 };
	struct dpsw_rsp_is_enabled *cmd_rsp;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IS_ENABLED, cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	cmd_rsp = (struct dpsw_rsp_is_enabled *)cmd.params;
	*en = dpsw_get_field(cmd_rsp->enabled, ENABLE);

	return 0;
}

/**
 * dpsw_reset() - Reset the DPSW, returns the object to initial state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_reset(struct fsl_mc_io *mc_io,
	       u32 cmd_flags,
	       u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_set_irq() - Set IRQ information for the DPSW to trigger an interrupt.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @irq_index:	Identifies the interrupt index to configure
 * @irq_cfg:	IRQ configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_set_irq *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_set_irq *)cmd.params;
	cmd_params->irq_index = irq_index;
	cmd_params->irq_val = cpu_to_le32(irq_cfg->val);
	cmd_params->irq_addr = cpu_to_le64(irq_cfg->addr);
	cmd_params->irq_num = cpu_to_le32(irq_cfg->irq_num);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_irq() - Get IRQ information from the DPSW
 *
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @type:	Interrupt type: 0 represents message interrupt
 *		type (both irq_addr and irq_val are valid)
 * @irq_cfg:	IRQ attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u8 irq_index,
		 int *type,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_get_irq *cmd_params;
	struct dpsw_rsp_get_irq *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_get_irq *)cmd.params;
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_get_irq *)cmd.params;
	irq_cfg->addr = le64_to_cpu(rsp_params->irq_addr);
	irq_cfg->val = le32_to_cpu(rsp_params->irq_val);
	irq_cfg->irq_num = le32_to_cpu(rsp_params->irq_num);
	*type = le32_to_cpu(rsp_params->irq_type);

	return 0;
}

/**
 * dpsw_set_irq_enable() - Set overall interrupt state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @en:		Interrupt state - enable = 1, disable = 0
 *
 * Allows GPP software to control when interrupts are generated.
 * Each interrupt can have up to 32 causes.  The enable/disable control's the
 * overall interrupt state. if the interrupt is disabled no causes will cause
 * an interrupt
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u8 en)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_set_irq_enable *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_set_irq_enable *)cmd.params;
	dpsw_set_field(cmd_params->enable_state, ENABLE, en);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_set_irq_mask() - Set interrupt mask.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @mask:	Event mask to trigger interrupt;
 *		each bit:
 *			0 = ignore event
 *			1 = consider event for asserting IRQ
 *
 * Every interrupt can have up to 32 causes and the interrupt model supports
 * masking/unmasking each cause independently
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_set_irq_mask(struct fsl_mc_io *mc_io,
		      u32 cmd_flags,
		      u16 token,
		      u8 irq_index,
		      u32 mask)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_set_irq_mask *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_set_irq_mask *)cmd.params;
	cmd_params->mask = cpu_to_le32(mask);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_irq_status() - Get the current status of any pending interrupts
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @irq_index:	The interrupt index to configure
 * @status:	Returned interrupts status - one bit per cause:
 *			0 = no interrupt pending
 *			1 = interrupt pending
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_get_irq_status(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u8 irq_index,
			u32 *status)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_get_irq_status *cmd_params;
	struct dpsw_rsp_get_irq_status *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_get_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(*status);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_get_irq_status *)cmd.params;
	*status = le32_to_cpu(rsp_params->status);

	return 0;
}

/**
 * dpsw_clear_irq_status() - Clear a pending interrupt's status
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPCI object
 * @irq_index:	The interrupt index to configure
 * @status:	bits to clear (W1C) - one bit per cause:
 *			0 = don't change
 *			1 = clear status bit
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_clear_irq_status(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u8 irq_index,
			  u32 status)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_clear_irq_status *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_clear_irq_status *)cmd.params;
	cmd_params->status = cpu_to_le32(status);
	cmd_params->irq_index = irq_index;

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_attributes() - Retrieve DPSW attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @attr:	Returned DPSW attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_get_attributes(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			struct dpsw_attr *attr)
{
	struct mc_command cmd = { 0 };
	struct dpsw_rsp_get_attr *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_get_attr *)cmd.params;
	attr->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	attr->max_fdbs = rsp_params->max_fdbs;
	attr->num_fdbs = rsp_params->num_fdbs;
	attr->max_vlans = le16_to_cpu(rsp_params->max_vlans);
	attr->num_vlans = le16_to_cpu(rsp_params->num_vlans);
	attr->max_fdb_entries = le16_to_cpu(rsp_params->max_fdb_entries);
	attr->fdb_aging_time = le16_to_cpu(rsp_params->fdb_aging_time);
	attr->id = le32_to_cpu(rsp_params->dpsw_id);
	attr->mem_size = le16_to_cpu(rsp_params->mem_size);
	attr->max_fdb_mc_groups = le16_to_cpu(rsp_params->max_fdb_mc_groups);
	attr->max_meters_per_if = rsp_params->max_meters_per_if;
	attr->options = le64_to_cpu(rsp_params->options);
	attr->component_type = dpsw_get_field(rsp_params->component_type,
					      COMPONENT_TYPE);

	return 0;
}

/**
 * dpsw_set_reflection_if() - Set target interface for reflected interfaces.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Id
 *
 * Only one reflection receive interface is allowed per switch
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_set_reflection_if(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_set_reflection_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_REFLECTION_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_set_reflection_if *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_link_cfg() - Set the link configuration.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface id
 * @cfg:	Link configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_if_set_link_cfg(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 struct dpsw_link_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_link_cfg *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_LINK_CFG,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_link_cfg *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->rate = cpu_to_le32(cfg->rate);
	cmd_params->options = cpu_to_le64(cfg->options);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_link_state - Return the link state
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface id
 * @state:	Link state	1 - linkup, 0 - link down or disconnected
 *
 * @Return	'0' on Success; Error code otherwise.
 */
int dpsw_if_get_link_state(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_link_state *state)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_get_link_state *cmd_params;
	struct dpsw_rsp_if_get_link_state *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_LINK_STATE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_get_link_state *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_if_get_link_state *)cmd.params;
	state->rate = le32_to_cpu(rsp_params->rate);
	state->options = le64_to_cpu(rsp_params->options);
	state->up = dpsw_get_field(rsp_params->up, UP);

	return 0;
}

/**
 * dpsw_if_set_flooding() - Enable Disable flooding for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @en:		1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_flooding(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 int en)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_flooding *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_flooding *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->enable, ENABLE, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_broadcast() - Enable/disable broadcast for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @en:		1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_broadcast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_broadcast *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_BROADCAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_broadcast *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->enable, ENABLE, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_multicast() - Enable/disable multicast for particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @en:		1 - enable, 0 - disable
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_multicast(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 if_id,
			  int en)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_multicast *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MULTICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_multicast *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->enable, ENABLE, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_tci() - Set default VLAN Tag Control Information (TCI)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Tag Control Information Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_tci *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TCI,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_tci *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->conf, VLAN_ID, cfg->vlan_id);
	dpsw_set_field(cmd_params->conf, DEI, cfg->dei);
	dpsw_set_field(cmd_params->conf, PCP, cfg->pcp);
	cmd_params->conf = cpu_to_le16(cmd_params->conf);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_tci() - Get default VLAN Tag Control Information (TCI)
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Tag Control Information Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_tci(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_get_tci *cmd_params;
	struct dpsw_rsp_if_get_tci *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_TCI,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_get_tci *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_if_get_tci *)cmd.params;
	cfg->pcp = rsp_params->pcp;
	cfg->dei = rsp_params->dei;
	cfg->vlan_id = le16_to_cpu(rsp_params->vlan_id);

	return 0;
}

/**
 * dpsw_if_set_stp() - Function sets Spanning Tree Protocol (STP) state.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	STP State configuration parameters
 *
 * The following STP states are supported -
 * blocking, listening, learning, forwarding and disabled.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_stp(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id,
		    const struct dpsw_stp_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_stp *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_STP,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_stp *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->vlan_id = cpu_to_le16(cfg->vlan_id);
	dpsw_set_field(cmd_params->state, STATE, cfg->state);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_accepted_frames()
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Frame types configuration
 *
 * When is admit_only_vlan_tagged- the device will discard untagged
 * frames or Priority-Tagged frames received on this interface.
 * When admit_only_untagged- untagged frames or Priority-Tagged
 * frames received on this interface will be accepted and assigned
 * to a VID based on the PVID and VID Set for this interface.
 * When admit_all - the device will accept VLAN tagged, untagged
 * and priority tagged frames.
 * The default is admit_all
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_accepted_frames(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				const struct dpsw_accepted_frames_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_accepted_frames *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_ACCEPTED_FRAMES,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_accepted_frames *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->unaccepted, FRAME_TYPE, cfg->type);
	dpsw_set_field(cmd_params->unaccepted, UNACCEPTED_ACT,
		       cfg->unaccept_act);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_accept_all_vlan()
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @accept_all:	Accept or drop frames having different VLAN
 *
 * When this is accept (FALSE), the device will discard incoming
 * frames for VLANs that do not include this interface in its
 * Member set. When accept (TRUE), the interface will accept all incoming frames
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_accept_all_vlan(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				u16 if_id,
				int accept_all)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_accept_all_vlan *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IF_ACCEPT_ALL_VLAN,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_accept_all_vlan *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->accept_all, ACCEPT_ALL, accept_all);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_counter() - Get specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @type:	Counter type
 * @counter:	return value
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 *counter)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_get_counter *cmd_params;
	struct dpsw_rsp_if_get_counter *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_COUNTER,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_get_counter *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->type, COUNTER_TYPE, type);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_if_get_counter *)cmd.params;
	*counter = le64_to_cpu(rsp_params->counter);

	return 0;
}

/**
 * dpsw_if_set_counter() - Set specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @type:	Counter type
 * @counter:	New counter value
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_counter(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 if_id,
			enum dpsw_counter type,
			u64 counter)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_counter *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_COUNTER,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_counter *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->counter = cpu_to_le64(counter);
	dpsw_set_field(cmd_params->type, COUNTER_TYPE, type);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_tx_selection() - Function is used for mapping variety
 *				of frame fields
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Traffic class mapping configuration
 *
 * Function is used for mapping variety of frame fields (DSCP, PCP)
 * to Traffic Class. Traffic class is a number
 * in the range from 0 to 7
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_tx_selection(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 if_id,
			     const struct dpsw_tx_selection_cfg *cfg)
{
	struct dpsw_cmd_if_set_tx_selection *cmd_params;
	struct mc_command cmd = { 0 };
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TX_SELECTION,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_tx_selection *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->priority_selector, PRIORITY_SELECTOR,
		       cfg->priority_selector);

	for (i = 0; i < 8; i++) {
		cmd_params->tc_sched[i].delta_bandwidth =
				cpu_to_le16(cfg->tc_sched[i].delta_bandwidth);
		dpsw_set_field(cmd_params->tc_sched[i].mode, SCHED_MODE,
			       cfg->tc_sched[i].mode);
		cmd_params->tc_id[i] = cfg->tc_id[i];
	}

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_add_reflection() - Identify interface to be reflected or mirrored
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @cfg:		Reflection configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_add_reflection(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_reflection *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ADD_REFLECTION,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_reflection *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->vlan_id = cpu_to_le16(cfg->vlan_id);
	dpsw_set_field(cmd_params->filter, FILTER, cfg->filter);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_remove_reflection() - Remove interface to be reflected or mirrored
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Reflection configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_remove_reflection(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 if_id,
			      const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_reflection *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_REMOVE_REFLECTION,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_reflection *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->vlan_id = cpu_to_le16(cfg->vlan_id);
	dpsw_set_field(cmd_params->filter, FILTER, cfg->filter);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_flooding_metering() - Set flooding metering
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @cfg:	Metering parameters
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_flooding_metering(struct fsl_mc_io *mc_io,
				  u32 cmd_flags,
				  u16 token,
				  u16 if_id,
				  const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_flooding_metering *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING_METERING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_flooding_metering *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	dpsw_set_field(cmd_params->mode_units, MODE, cfg->mode);
	dpsw_set_field(cmd_params->mode_units, UNITS, cfg->units);
	cmd_params->cir = cpu_to_le32(cfg->cir);
	cmd_params->eir = cpu_to_le32(cfg->eir);
	cmd_params->cbs = cpu_to_le32(cfg->cbs);
	cmd_params->ebs = cpu_to_le32(cfg->ebs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_set_metering() - Set interface metering for flooding
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @tc_id:	Traffic class ID
 * @cfg:	Metering parameters
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_metering(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 if_id,
			 u8 tc_id,
			 const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_metering *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_METERING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_metering *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->tc_id = tc_id;
	dpsw_set_field(cmd_params->mode_units, MODE, cfg->mode);
	dpsw_set_field(cmd_params->mode_units, UNITS, cfg->units);
	cmd_params->cir = cpu_to_le32(cfg->cir);
	cmd_params->eir = cpu_to_le32(cfg->eir);
	cmd_params->cbs = cpu_to_le32(cfg->cbs);
	cmd_params->ebs = cpu_to_le32(cfg->ebs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_prepare_early_drop() - Prepare an early drop for setting in to interface
 * @cfg:		Early-drop configuration
 * @early_drop_buf:	Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before dpsw_if_tc_set_early_drop
 *
 */
void dpsw_prepare_early_drop(const struct dpsw_early_drop_cfg *cfg,
			     u8 *early_drop_buf)
{
	struct dpsw_prep_early_drop *ext_params;

	ext_params = (struct dpsw_prep_early_drop *)early_drop_buf;
	dpsw_set_field(ext_params->conf, EARLY_DROP_MODE, cfg->drop_mode);
	dpsw_set_field(ext_params->conf, EARLY_DROP_UNIT, cfg->units);
	ext_params->tail_drop_threshold = cpu_to_le32(cfg->tail_drop_threshold);
	ext_params->green_drop_probability = cfg->green.drop_probability;
	ext_params->green_max_threshold = cpu_to_le64(cfg->green.max_threshold);
	ext_params->green_min_threshold = cpu_to_le64(cfg->green.min_threshold);
	ext_params->yellow_drop_probability = cfg->yellow.drop_probability;
	ext_params->yellow_max_threshold =
			cpu_to_le64(cfg->yellow.max_threshold);
	ext_params->yellow_min_threshold =
			cpu_to_le64(cfg->yellow.min_threshold);
}

/**
 * dpsw_if_set_early_drop() - Set interface traffic class early-drop
 *				configuration
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:		Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @tc_id:		Traffic class selection (0-7)
 * @early_drop_iova:	I/O virtual address of 64 bytes;
 * Must be cacheline-aligned and DMA-able memory
 *
 * warning: Before calling this function, call dpsw_prepare_if_tc_early_drop()
 *		to prepare the early_drop_iova parameter
 *
 * Return:	'0' on Success; error code otherwise.
 */
int dpsw_if_set_early_drop(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   u8 tc_id,
			   u64 early_drop_iova)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_early_drop *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_EARLY_DROP,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_early_drop *)cmd.params;
	cmd_params->tc_id = tc_id;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->early_drop_iova = cpu_to_le64(early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_add_custom_tpid() - API Configures a distinct Ethernet type value
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @cfg:	Tag Protocol identifier
 *
 * API Configures a distinct Ethernet type value (or TPID value)
 * to indicate a VLAN tag in addition to the common
 * TPID values 0x8100 and 0x88A8.
 * Two additional TPID's are supported
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_add_custom_tpid(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_custom_tpid *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ADD_CUSTOM_TPID,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_custom_tpid *)cmd.params;
	cmd_params->tpid = cpu_to_le16(cfg->tpid);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_remove_custom_tpid - API removes a distinct Ethernet type value
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @cfg:	Tag Protocol identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_remove_custom_tpid(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_custom_tpid *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_REMOVE_CUSTOM_TPID,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_custom_tpid *)cmd.params;
	cmd_params->tpid = cpu_to_le16(cfg->tpid);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_enable() - Enable Interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_enable(struct fsl_mc_io *mc_io,
		   u32 cmd_flags,
		   u16 token,
		   u16 if_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ENABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_disable() - Disable Interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_disable(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 if_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_DISABLE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_attributes() - Function obtains attributes of interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @if_id:	Interface Identifier
 * @attr:	Returned interface attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_attributes(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 if_id,
			   struct dpsw_if_attr *attr)
{
	struct dpsw_rsp_if_get_attr *rsp_params;
	struct dpsw_cmd_if *cmd_params;
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_ATTR,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_if_get_attr *)cmd.params;
	attr->num_tcs = rsp_params->num_tcs;
	attr->rate = le32_to_cpu(rsp_params->rate);
	attr->options = le32_to_cpu(rsp_params->options);
	attr->enabled = dpsw_get_field(rsp_params->conf, ENABLED);
	attr->accept_all_vlan = dpsw_get_field(rsp_params->conf,
					       ACCEPT_ALL_VLAN);
	attr->admit_untagged = dpsw_get_field(rsp_params->conf, ADMIT_UNTAGGED);
	attr->qdid = le16_to_cpu(rsp_params->qdid);

	return 0;
}

/**
 * dpsw_if_set_max_frame_length() - Set Maximum Receive frame length.
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:		Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @frame_length:	Maximum Frame Length
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_set_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 frame_length)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_set_max_frame_length *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_set_max_frame_length *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);
	cmd_params->frame_length = cpu_to_le16(frame_length);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_if_get_max_frame_length() - Get Maximum Receive frame length.
 * @mc_io:		Pointer to MC portal's I/O object
 * @cmd_flags:		Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:		Token of DPSW object
 * @if_id:		Interface Identifier
 * @frame_length:	Returned maximum Frame Length
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_if_get_max_frame_length(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 if_id,
				 u16 *frame_length)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_if_get_max_frame_length *cmd_params;
	struct dpsw_rsp_if_get_max_frame_length *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_if_get_max_frame_length *)cmd.params;
	cmd_params->if_id = cpu_to_le16(if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpsw_rsp_if_get_max_frame_length *)cmd.params;
	*frame_length = le16_to_cpu(rsp_params->frame_length);

	return 0;
}

/**
 * dpsw_vlan_add() - Adding new VLAN to DPSW.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	VLAN configuration
 *
 * Only VLAN ID and FDB ID are required parameters here.
 * 12 bit VLAN ID is defined in IEEE802.1Q.
 * Adding a duplicate VLAN ID is not allowed.
 * FDB ID can be shared across multiple VLANs. Shared learning
 * is obtained by calling dpsw_vlan_add for multiple VLAN IDs
 * with same fdb_id
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add(struct fsl_mc_io *mc_io,
		  u32 cmd_flags,
		  u16 token,
		  u16 vlan_id,
		  const struct dpsw_vlan_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_vlan_add *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_vlan_add *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(cfg->fdb_id);
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if() - Adding a set of interfaces to an existing VLAN.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces to add
 *
 * It adds only interfaces not belonging to this VLAN yet,
 * otherwise an error is generated and an entire command is
 * ignored. This function can be called numerous times always
 * providing required interfaces delta.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if_untagged() - Defining a set of interfaces that should be
 *				transmitted as untagged.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be transmitted as untagged
 *
 * These interfaces should already belong to this VLAN.
 * By default all interfaces are transmitted as tagged.
 * Providing un-existing interface or untagged interface that is
 * configured untagged already generates an error and the entire
 * command is ignored.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_UNTAGGED,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_add_if_flooding() - Define a set of interfaces that should be
 *			included in flooding when frame with unknown destination
 *			unicast MAC arrived.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be used for flooding
 *
 * These interfaces should belong to this VLAN. By default all
 * interfaces are included into flooding list. Providing
 * un-existing interface or an interface that already in the
 * flooding list generates an error and the entire command is
 * ignored.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_add_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_FLOODING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if() - Remove interfaces from an existing VLAN.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be removed
 *
 * Interfaces must belong to this VLAN, otherwise an error
 * is returned and an the command is ignored
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token,
			u16 vlan_id,
			const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if_untagged() - Define a set of interfaces that should be
 *		converted from transmitted as untagged to transmit as tagged.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces that should be removed
 *
 * Interfaces provided by API have to belong to this VLAN and
 * configured untagged, otherwise an error is returned and the
 * command is ignored
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if_untagged(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_UNTAGGED,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove_if_flooding() - Define a set of interfaces that should be
 *			removed from the flooding list.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Set of interfaces used for flooding
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove_if_flooding(struct fsl_mc_io *mc_io,
				 u32 cmd_flags,
				 u16 token,
				 u16 vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_manage_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_FLOODING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_manage_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_remove() - Remove an entire VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_remove(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_remove *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_remove *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_vlan_get_attributes() - Get VLAN attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @attr:	Returned DPSW attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_attributes(struct fsl_mc_io *mc_io,
			     u32 cmd_flags,
			     u16 token,
			     u16 vlan_id,
			     struct dpsw_vlan_attr *attr)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_get_attr *cmd_params;
	struct dpsw_rsp_vlan_get_attr *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_ATTRIBUTES,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_get_attr *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_vlan_get_attr *)cmd.params;
	attr->fdb_id = le16_to_cpu(rsp_params->fdb_id);
	attr->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	attr->num_untagged_ifs = le16_to_cpu(rsp_params->num_untagged_ifs);
	attr->num_flooding_ifs = le16_to_cpu(rsp_params->num_flooding_ifs);

	return 0;
}

/**
 * dpsw_vlan_get_if() - Get interfaces belong to this VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of interfaces belong to this VLAN
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_if(struct fsl_mc_io *mc_io,
		     u32 cmd_flags,
		     u16 token,
		     u16 vlan_id,
		     struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_get_if *cmd_params;
	struct dpsw_rsp_vlan_get_if *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_get_if *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_vlan_get_if *)cmd.params;
	cfg->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, rsp_params->if_id);

	return 0;
}

/**
 * dpsw_vlan_get_if_flooding() - Get interfaces used in flooding for this VLAN
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of flooding interfaces
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */

int dpsw_vlan_get_if_flooding(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_get_if_flooding *cmd_params;
	struct dpsw_rsp_vlan_get_if_flooding *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_FLOODING,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_get_if_flooding *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_vlan_get_if_flooding *)cmd.params;
	cfg->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, rsp_params->if_id);

	return 0;
}

/**
 * dpsw_vlan_get_if_untagged() - Get interfaces that should be transmitted as
 *				untagged
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @vlan_id:	VLAN Identifier
 * @cfg:	Returned set of untagged interfaces
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_vlan_get_if_untagged(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_vlan_get_if_untagged *cmd_params;
	struct dpsw_rsp_vlan_get_if_untagged *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_UNTAGGED,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_vlan_get_if_untagged *)cmd.params;
	cmd_params->vlan_id = cpu_to_le16(vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_vlan_get_if_untagged *)cmd.params;
	cfg->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, rsp_params->if_id);

	return 0;
}

/**
 * dpsw_fdb_add() - Add FDB to switch and Returns handle to FDB table for
 *		the reference
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Returned Forwarding Database Identifier
 * @cfg:	FDB Configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *fdb_id,
		 const struct dpsw_fdb_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_add *cmd_params;
	struct dpsw_rsp_fdb_add *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_add *)cmd.params;
	cmd_params->fdb_aging_time = cpu_to_le16(cfg->fdb_aging_time);
	cmd_params->num_fdb_entries = cpu_to_le16(cfg->num_fdb_entries);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_fdb_add *)cmd.params;
	*fdb_id = le16_to_cpu(rsp_params->fdb_id);

	return 0;
}

/**
 * dpsw_fdb_remove() - Remove FDB from switch
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 fdb_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_remove *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_remove *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_add_unicast() - Function adds an unicast entry into MAC lookup table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_add_unicast *cmd_params;
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_UNICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_add_unicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	cmd_params->if_egress = cpu_to_le16(cfg->if_egress);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];
	dpsw_set_field(cmd_params->type, ENTRY_TYPE, cfg->type);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_unicast() - Get unicast entry from MAC lookup table by
 *		unicast Ethernet address
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Returned unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_unicast(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token,
			 u16 fdb_id,
			 struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_get_unicast *cmd_params;
	struct dpsw_rsp_fdb_get_unicast *rsp_params;
	int err, i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_UNICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_get_unicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_fdb_get_unicast *)cmd.params;
	cfg->if_egress = le16_to_cpu(rsp_params->if_egress);
	cfg->type = dpsw_get_field(rsp_params->type, ENTRY_TYPE);

	return 0;
}

/**
 * dpsw_fdb_remove_unicast() - removes an entry from MAC lookup table
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Unicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove_unicast(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_remove_unicast *cmd_params;
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_UNICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_remove_unicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];
	cmd_params->if_egress = cpu_to_le16(cfg->if_egress);
	dpsw_set_field(cmd_params->type, ENTRY_TYPE, cfg->type);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_add_multicast() - Add a set of egress interfaces to multi-cast group
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Multicast entry configuration
 *
 * If group doesn't exist, it will be created.
 * It adds only interfaces not belonging to this multicast group
 * yet, otherwise error will be generated and the command is
 * ignored.
 * This function may be called numerous times always providing
 * required interfaces delta.
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_add_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_add_multicast *cmd_params;
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_MULTICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_add_multicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	cmd_params->num_ifs = cpu_to_le16(cfg->num_ifs);
	dpsw_set_field(cmd_params->type, ENTRY_TYPE, cfg->type);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_multicast() - Reading multi-cast group by multi-cast Ethernet
 *				address.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Returned multicast entry configuration
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_multicast(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   u16 fdb_id,
			   struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_get_multicast *cmd_params;
	struct dpsw_rsp_fdb_get_multicast *rsp_params;
	int err, i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_MULTICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_get_multicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_fdb_get_multicast *)cmd.params;
	cfg->num_ifs = le16_to_cpu(rsp_params->num_ifs);
	cfg->type = dpsw_get_field(rsp_params->type, ENTRY_TYPE);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, rsp_params->if_id);

	return 0;
}

/**
 * dpsw_fdb_remove_multicast() - Removing interfaces from an existing multicast
 *				group.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @cfg:	Multicast entry configuration
 *
 * Interfaces provided by this API have to exist in the group,
 * otherwise an error will be returned and an entire command
 * ignored. If there is no interface left in the group,
 * an entire group is deleted
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_remove_multicast(struct fsl_mc_io *mc_io,
			      u32 cmd_flags,
			      u16 token,
			      u16 fdb_id,
			      const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_remove_multicast *cmd_params;
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_MULTICAST,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_remove_multicast *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	cmd_params->num_ifs = cpu_to_le16(cfg->num_ifs);
	dpsw_set_field(cmd_params->type, ENTRY_TYPE, cfg->type);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);
	for (i = 0; i < 6; i++)
		cmd_params->mac_addr[i] = cfg->mac_addr[5 - i];

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_set_learning_mode() - Define FDB learning mode
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @mode:	Learning mode
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_set_learning_mode(struct fsl_mc_io *mc_io,
			       u32 cmd_flags,
			       u16 token,
			       u16 fdb_id,
			       enum dpsw_fdb_learning_mode mode)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_set_learning_mode *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_SET_LEARNING_MODE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_set_learning_mode *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);
	dpsw_set_field(cmd_params->mode, LEARNING_MODE, mode);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_fdb_get_attributes() - Get FDB attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @fdb_id:	Forwarding Database Identifier
 * @attr:	Returned FDB attributes
 *
 * Return:	Completion status. '0' on Success; Error code otherwise.
 */
int dpsw_fdb_get_attributes(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 fdb_id,
			    struct dpsw_fdb_attr *attr)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_fdb_get_attr *cmd_params;
	struct dpsw_rsp_fdb_get_attr *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_ATTR,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_fdb_get_attr *)cmd.params;
	cmd_params->fdb_id = cpu_to_le16(fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_fdb_get_attr *)cmd.params;
	attr->max_fdb_entries = le16_to_cpu(rsp_params->max_fdb_entries);
	attr->fdb_aging_time = le16_to_cpu(rsp_params->fdb_aging_time);
	attr->learning_mode = dpsw_get_field(rsp_params->learning_mode,
					     LEARNING_MODE);
	attr->num_fdb_mc_groups = le16_to_cpu(rsp_params->num_fdb_mc_groups);
	attr->max_fdb_mc_groups = le16_to_cpu(rsp_params->max_fdb_mc_groups);

	return 0;
}

/**
 * dpsw_acl_add() - Adds ACL to L2 switch.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	Returned ACL ID, for the future reference
 * @cfg:	ACL configuration
 *
 * Create Access Control List. Multiple ACLs can be created and
 * co-exist in L2 switch
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add(struct fsl_mc_io *mc_io,
		 u32 cmd_flags,
		 u16 token,
		 u16 *acl_id,
		 const struct dpsw_acl_cfg  *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_add *cmd_params;
	struct dpsw_rsp_acl_add *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_add *)cmd.params;
	cmd_params->max_entries = cpu_to_le16(cfg->max_entries);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_acl_add *)cmd.params;
	*acl_id = le16_to_cpu(rsp_params->acl_id);

	return 0;
}

/**
 * dpsw_acl_remove() - Removes ACL from L2 switch.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_remove *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_remove *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_prepare_entry_cfg() - Set an entry to ACL.
 * @key:		Key
 * @entry_cfg_buf:	Zeroed 256 bytes of memory before mapping it to DMA
 *
 * This function has to be called before adding or removing acl_entry
 *
 */
void dpsw_acl_prepare_entry_cfg(const struct dpsw_acl_key *key,
				u8 *entry_cfg_buf)
{
	struct dpsw_prep_acl_entry *ext_params;
	int i;

	ext_params = (struct dpsw_prep_acl_entry *)entry_cfg_buf;

	for (i = 0; i < 6; i++) {
		ext_params->match_l2_dest_mac[i] =
			key->match.l2_dest_mac[5 - i];
		ext_params->match_l2_source_mac[i] =
			key->match.l2_source_mac[5 - i];
		ext_params->mask_l2_dest_mac[i] =
			key->mask.l2_dest_mac[5 - i];
		ext_params->mask_l2_source_mac[i] =
			key->mask.l2_source_mac[5 - i];
	}

	ext_params->match_l2_tpid = cpu_to_le16(key->match.l2_tpid);
	ext_params->match_l2_vlan_id = cpu_to_le16(key->match.l2_vlan_id);
	ext_params->match_l3_dest_ip = cpu_to_le32(key->match.l3_dest_ip);
	ext_params->match_l3_source_ip = cpu_to_le32(key->match.l3_source_ip);
	ext_params->match_l4_dest_port = cpu_to_le16(key->match.l4_dest_port);
	ext_params->match_l2_ether_type = cpu_to_le16(key->match.l2_ether_type);
	ext_params->match_l2_pcp_dei = key->match.l2_pcp_dei;
	ext_params->match_l3_dscp = key->match.l3_dscp;
	ext_params->match_l4_source_port =
		cpu_to_le16(key->match.l4_source_port);

	ext_params->mask_l2_tpid = cpu_to_le16(key->mask.l2_tpid);
	ext_params->mask_l2_vlan_id = cpu_to_le16(key->mask.l2_vlan_id);
	ext_params->mask_l3_dest_ip = cpu_to_le32(key->mask.l3_dest_ip);
	ext_params->mask_l3_source_ip = cpu_to_le32(key->mask.l3_source_ip);
	ext_params->mask_l4_dest_port = cpu_to_le16(key->mask.l4_dest_port);
	ext_params->mask_l4_source_port = cpu_to_le16(key->mask.l4_source_port);
	ext_params->mask_l2_ether_type = cpu_to_le16(key->mask.l2_ether_type);
	ext_params->mask_l2_pcp_dei = key->mask.l2_pcp_dei;
	ext_params->mask_l3_dscp = key->mask.l3_dscp;
	ext_params->match_l3_protocol = key->match.l3_protocol;
	ext_params->mask_l3_protocol = key->mask.l3_protocol;
}

/**
 * dpsw_acl_add_entry() - Adds an entry to ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	Entry configuration
 *
 * warning: This function has to be called after dpsw_acl_set_entry_cfg()
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add_entry(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_entry *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_ENTRY,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_entry *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);
	cmd_params->result_if_id = cpu_to_le16(cfg->result.if_id);
	cmd_params->precedence = cpu_to_le32(cfg->precedence);
	dpsw_set_field(cmd_params->result_action, RESULT_ACTION,
		       cfg->result.action);
	cmd_params->key_iova = cpu_to_le64(cfg->key_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_remove_entry() - Removes an entry from ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	Entry configuration
 *
 * warning: This function has to be called after dpsw_acl_set_entry_cfg()
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove_entry(struct fsl_mc_io *mc_io,
			  u32 cmd_flags,
			  u16 token,
			  u16 acl_id,
			  const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_entry *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_ENTRY,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_entry *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);
	cmd_params->result_if_id = cpu_to_le16(cfg->result.if_id);
	cmd_params->precedence = cpu_to_le32(cfg->precedence);
	dpsw_set_field(cmd_params->result_action, RESULT_ACTION,
		       cfg->result.action);
	cmd_params->key_iova = cpu_to_le64(cfg->key_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_add_if() - Associate interface/interfaces with ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	Interfaces list
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_add_if(struct fsl_mc_io *mc_io,
		    u32 cmd_flags,
		    u16 token,
		    u16 acl_id,
		    const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_if *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);
	cmd_params->num_ifs = cpu_to_le16(cfg->num_ifs);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_remove_if() - De-associate interface/interfaces from ACL.
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL ID
 * @cfg:	Interfaces list
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_remove_if(struct fsl_mc_io *mc_io,
		       u32 cmd_flags,
		       u16 token,
		       u16 acl_id,
		       const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_if *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_IF,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_if *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);
	cmd_params->num_ifs = cpu_to_le16(cfg->num_ifs);
	build_if_id_bitmap(cmd_params->if_id, cfg->if_id, cfg->num_ifs);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_acl_get_attributes() - Get specific counter of particular interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @acl_id:	ACL Identifier
 * @attr:	Returned ACL attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_acl_get_attributes(struct fsl_mc_io *mc_io,
			    u32 cmd_flags,
			    u16 token,
			    u16 acl_id,
			    struct dpsw_acl_attr *attr)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_acl_get_attr *cmd_params;
	struct dpsw_rsp_acl_get_attr *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_GET_ATTR,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_acl_get_attr *)cmd.params;
	cmd_params->acl_id = cpu_to_le16(acl_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_acl_get_attr *)cmd.params;
	attr->max_entries = le16_to_cpu(rsp_params->max_entries);
	attr->num_entries = le16_to_cpu(rsp_params->num_entries);
	attr->num_ifs = le16_to_cpu(rsp_params->num_ifs);

	return 0;
}

/**
 * dpsw_ctrl_if_get_attributes() - Obtain control interface attributes
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @attr:	Returned control interface attributes
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_get_attributes(struct fsl_mc_io *mc_io,
				u32 cmd_flags,
				u16 token,
				struct dpsw_ctrl_if_attr *attr)
{
	struct mc_command cmd = { 0 };
	struct dpsw_rsp_ctrl_if_get_attr *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpsw_rsp_ctrl_if_get_attr *)cmd.params;
	attr->rx_fqid = le32_to_cpu(rsp_params->rx_fqid);
	attr->rx_err_fqid = le32_to_cpu(rsp_params->rx_err_fqid);
	attr->tx_err_conf_fqid = le32_to_cpu(rsp_params->tx_err_conf_fqid);

	return 0;
}

/**
 * dpsw_ctrl_if_set_pools() - Set control interface buffer pools
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 * @cfg:	Buffer pools configuration
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_set_pools(struct fsl_mc_io *mc_io,
			   u32 cmd_flags,
			   u16 token,
			   const struct dpsw_ctrl_if_pools_cfg *pools)
{
	struct mc_command cmd = { 0 };
	struct dpsw_cmd_ctrl_if_set_pools *cmd_params;
	int i;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_SET_POOLS,
					  cmd_flags,
					  token);
	cmd_params = (struct dpsw_cmd_ctrl_if_set_pools *)cmd.params;
	cmd_params->num_dpbp = pools->num_dpbp;
	for (i = 0; i < 8; i++) {
		cmd_params->backup_pool = dpsw_set_bit(cmd_params->backup_pool,
						i,
						pools->pools[i].backup_pool);
		cmd_params->buffer_size[i] =
			cpu_to_le16(pools->pools[i].buffer_size);
		cmd_params->dpbp_id[i] =
			cpu_to_le32(pools->pools[i].dpbp_id);
	}

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_ctrl_if_enable() - Enable control interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_enable(struct fsl_mc_io *mc_io,
			u32 cmd_flags,
			u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_ctrl_if_disable() - Function disables control interface
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @token:	Token of DPSW object
 *
 * Return:	'0' on Success; Error code otherwise.
 */
int dpsw_ctrl_if_disable(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

/**
 * dpsw_get_api_version() - Get Data Path Switch API version
 * @mc_io:	Pointer to MC portal's I/O object
 * @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
 * @major_ver:	Major version of data path switch API
 * @minor_ver:	Minor version of data path switch API
 *
 * Return:  '0' on Success; Error code otherwise.
 */
int dpsw_get_api_version(struct fsl_mc_io *mc_io,
			 u32 cmd_flags,
			 u16 *major_ver,
			 u16 *minor_ver)
{
	struct mc_command cmd = { 0 };
	struct dpsw_rsp_get_api_version *rsp_params;
	int err;

	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_API_VERSION,
					cmd_flags,
					0);

	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	rsp_params = (struct dpsw_rsp_get_api_version *)cmd.params;
	*major_ver = le16_to_cpu(rsp_params->version_major);
	*minor_ver = le16_to_cpu(rsp_params->version_minor);

	return 0;
}
