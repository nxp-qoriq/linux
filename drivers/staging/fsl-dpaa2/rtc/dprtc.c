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
#include "../../fsl-mc/include/mc-sys.h"
#include "../../fsl-mc/include/mc-cmd.h"
#include "dprtc.h"
#include "dprtc-cmd.h"

int dprtc_open(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	      int dprtc_id,
	      uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPRTC_CMD_OPEN(cmd, dprtc_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return err;
}

int dprtc_close(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
	       uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_CLOSE, cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_create(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		const struct dprtc_cfg *cfg,
		uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	(void)(cfg); /* unused */

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_CREATE,
					  cmd_flags,
					  0);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dprtc_destroy(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_DESTROY,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_enable(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_ENABLE, cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_disable(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_is_enabled(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		    uint16_t token,
		    int *en)
{
	struct mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_IS_ENABLED, cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

int dprtc_reset(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
	       uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_set_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 struct dprtc_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_IRQ,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_get_irq(struct fsl_mc_io	*mc_io,
		  uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 int			*type,
		 struct dprtc_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_IRQ,
					  cmd_flags,
					  token);

	DPRTC_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

int dprtc_set_irq_enable(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_get_irq_enable(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);

	DPRTC_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

int dprtc_set_irq_mask(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_get_irq_mask(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);

	DPRTC_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

int dprtc_get_irq_status(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint32_t *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);

	DPRTC_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

int dprtc_clear_irq_status(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint32_t status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);

	DPRTC_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_get_attributes(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			uint16_t token,
			struct dprtc_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_ATTRIBUTES(cmd, attr);

	return 0;
}

int dprtc_set_clock_offset(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
		  uint16_t token,
		  int64_t offset)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_CLOCK_OFFSET,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_CLOCK_OFFSET(cmd, offset);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_set_freq_compensation(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
		  uint16_t token,
		  uint32_t freq_compensation)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_FREQ_COMPENSATION,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_FREQ_COMPENSATION(cmd, freq_compensation);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_get_freq_compensation(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
		  uint16_t token,
		  uint32_t *freq_compensation)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_FREQ_COMPENSATION,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_FREQ_COMPENSATION(cmd, *freq_compensation);

	return 0;
}

int dprtc_get_time(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		  uint16_t token,
		  uint64_t *timestamp)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_GET_TIME,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPRTC_RSP_GET_TIME(cmd, *timestamp);

	return 0;
}

int dprtc_set_time(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		  uint16_t token,
		  uint64_t timestamp)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_TIME,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_TIME(cmd, timestamp);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dprtc_set_alarm(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		  uint16_t token, uint64_t time)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPRTC_CMDID_SET_ALARM,
					  cmd_flags,
					  token);

	DPRTC_CMD_SET_ALARM(cmd, time);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
