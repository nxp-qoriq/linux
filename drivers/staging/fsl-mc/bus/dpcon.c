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
#include "../include/mc-sys.h"
#include "../include/mc-cmd.h"
#include "../include/dpcon.h"
#include "../include/dpcon-cmd.h"

int dpcon_open(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	       int dpcon_id,
	       uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPCON_CMD_OPEN(cmd, dpcon_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}
EXPORT_SYMBOL(dpcon_open);

int dpcon_close(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
EXPORT_SYMBOL(dpcon_close);

int dpcon_create(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 const struct dpcon_cfg *cfg,
		 uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_CREATE,
					  cmd_flags,
					  0);
	DPCON_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpcon_destroy(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_DESTROY,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_enable(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
EXPORT_SYMBOL(dpcon_enable);

int dpcon_disable(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
EXPORT_SYMBOL(dpcon_disable);

int dpcon_is_enabled(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     int *en)
{
	struct mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_IS_ENABLED,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

int dpcon_reset(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_RESET,
					  cmd_flags, token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_set_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token,
		  uint8_t		irq_index,
		  struct dpcon_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPCON_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_get_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		  uint16_t		token,
		  uint8_t		irq_index,
		  int			*type,
		  struct dpcon_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPCON_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

int dpcon_set_irq_enable(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint8_t irq_index,
			 uint8_t en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPCON_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_get_irq_enable(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint8_t irq_index,
			 uint8_t *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPCON_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

int dpcon_set_irq_mask(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       uint8_t irq_index,
		       uint32_t mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPCON_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_get_irq_mask(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       uint8_t irq_index,
		       uint32_t *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPCON_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

int dpcon_get_irq_status(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint8_t irq_index,
			 uint32_t *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPCON_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

int dpcon_clear_irq_status(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint8_t irq_index,
			   uint32_t status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPCON_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpcon_get_attributes(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 struct dpcon_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPCON_RSP_GET_ATTR(cmd, attr);

	return 0;
}
EXPORT_SYMBOL(dpcon_get_attributes);

int dpcon_set_notification(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   struct dpcon_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPCON_CMDID_SET_NOTIFICATION,
					  cmd_flags,
					  token);
	DPCON_CMD_SET_NOTIFICATION(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
EXPORT_SYMBOL(dpcon_set_notification);

