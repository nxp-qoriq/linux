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
#include <linux/types.h>
#include <linux/io.h>
#include "fsl_dpdmai.h"
#include "fsl_dpdmai_cmd.h"
#include <linux/fsl/mc.h>

struct dpdmai_cmd_open {
	__le32 dpdmai_id;
};

struct dpdmai_rsp_get_attributes {
	__le32 id;
	u8 num_of_priorities;
	u8 pad0[3];
	__le16 major;
	__le16 minor;
};


struct dpdmai_cmd_queue {
	__le32 dest_id;
	u8 priority;
	u8 queue;
	u8 dest_type;
	u8 pad;
	__le64 user_ctx;
	union {
		__le32 options;
		__le32 fqid;
	};
};

struct dpdmai_rsp_get_tx_queue {
	__le64 pad;
	__le32 fqid;
};


int dpdmai_open(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		int dpdmai_id,
		uint16_t *token)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpdmai_cmd_open *cmd_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_OPEN,
					  cmd_flags,
					  0);

	cmd_params = (struct dpdmai_cmd_open *)cmd.params;
	cmd_params->dpdmai_id = cpu_to_le32(dpdmai_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = mc_cmd_hdr_read_token(&cmd);
	return 0;
}

int dpdmai_close(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_CLOSE,
					  cmd_flags, token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_create(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  const struct dpdmai_cfg *cfg,
		  uint16_t *token)
{
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_CREATE,
					  cmd_flags,
					  0);
	DPDMAI_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpdmai_destroy(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_DESTROY,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_enable(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  uint16_t token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_disable(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_is_enabled(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      int *en)
{
	struct fsl_mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_IS_ENABLED,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMAI_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

int dpdmai_reset(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_irq(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token,
		   uint8_t irq_index,
		   int *type,
		   struct dpdmai_irq_cfg	*irq_cfg)
{
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPDMAI_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMAI_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

int dpdmai_set_irq(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token,
		   uint8_t irq_index,
		   struct dpdmai_irq_cfg *irq_cfg)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPDMAI_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_irq_enable(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint8_t *en)
{
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPDMAI_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMAI_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

int dpdmai_set_irq_enable(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint8_t en)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPDMAI_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_irq_mask(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint32_t *mask)
{
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPDMAI_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMAI_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

int dpdmai_set_irq_mask(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint32_t mask)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPDMAI_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_irq_status(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint32_t *status)
{
	struct fsl_mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPDMAI_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPDMAI_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

int dpdmai_clear_irq_status(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    uint8_t irq_index,
			    uint32_t status)
{
	struct fsl_mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPDMAI_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_attributes(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  struct dpdmai_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	int err;
	struct dpdmai_rsp_get_attributes *rsp_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_ATTR,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	rsp_params = (struct dpdmai_rsp_get_attributes *)cmd.params;
	attr->id = le32_to_cpu(rsp_params->id);
	attr->version.major = le16_to_cpu(rsp_params->major);
	attr->version.minor = le16_to_cpu(rsp_params->minor);
	attr->num_of_priorities = rsp_params->num_of_priorities;


	return 0;
}

int dpdmai_set_rx_queue(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t priority,
			const struct dpdmai_rx_queue_cfg *cfg)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpdmai_cmd_queue *cmd_params;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_SET_RX_QUEUE,
					  cmd_flags,
					  token);

	cmd_params = (struct dpdmai_cmd_queue *)cmd.params;
	cmd_params->dest_id = cpu_to_le32(cfg->dest_cfg.dest_id);
	cmd_params->priority = cfg->dest_cfg.priority;
	cmd_params->queue = priority;
	cmd_params->dest_type = cfg->dest_cfg.dest_type;
	cmd_params->user_ctx = cpu_to_le64(cfg->user_ctx);
	cmd_params->options = cpu_to_le32(cfg->options);


	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpdmai_get_rx_queue(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t priority, struct dpdmai_rx_queue_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpdmai_cmd_queue *cmd_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_RX_QUEUE,
					  cmd_flags,
					  token);

	cmd_params = (struct dpdmai_cmd_queue *)cmd.params;
	cmd_params->queue = priority;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	attr->dest_cfg.dest_id = le32_to_cpu(cmd_params->dest_id);
	attr->dest_cfg.priority = cmd_params->priority;
	attr->dest_cfg.dest_type = cmd_params->dest_type;
	attr->user_ctx = le64_to_cpu(cmd_params->user_ctx);
	attr->fqid = le32_to_cpu(cmd_params->fqid);

	return 0;
}

int dpdmai_get_tx_queue(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t priority,
			struct dpdmai_tx_queue_attr *attr)
{
	struct fsl_mc_command cmd = { 0 };
	struct dpdmai_cmd_queue *cmd_params;
	struct dpdmai_rsp_get_tx_queue *rsp_params;
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPDMAI_CMDID_GET_TX_QUEUE,
					  cmd_flags,
					  token);

	cmd_params = (struct dpdmai_cmd_queue *)cmd.params;
	cmd_params->queue = priority;

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */

	rsp_params = (struct dpdmai_rsp_get_tx_queue *)cmd.params;
	attr->fqid = le32_to_cpu(rsp_params->fqid);

	return 0;
}
