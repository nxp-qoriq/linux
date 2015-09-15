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
#include "dpni.h"
#include "dpni-cmd.h"

int dpni_prepare_key_cfg(const struct dpkg_profile_cfg *cfg,
			 uint8_t *key_cfg_buf)
{
	int i, j;
	int offset = 0;
	int param = 1;
	uint64_t *params = (uint64_t *)key_cfg_buf;

	if (!key_cfg_buf || !cfg)
			return -EINVAL;

	params[0] |= mc_enc(0, 8, cfg->num_extracts);
	params[0] = cpu_to_le64(params[0]);

	if (cfg->num_extracts >= DPKG_MAX_NUM_OF_EXTRACTS)
		return -EINVAL;

	for (i = 0; i < cfg->num_extracts; i++) {
		switch (cfg->extracts[i].type) {
		case DPKG_EXTRACT_FROM_HDR:
			params[param] |= mc_enc(0, 8,
					cfg->extracts[i].extract.from_hdr.prot);
			params[param] |= mc_enc(8, 4,
					cfg->extracts[i].extract.from_hdr.type);
			params[param] |= mc_enc(16, 8,
					cfg->extracts[i].extract.from_hdr.size);
			params[param] |= mc_enc(24, 8,
					cfg->extracts[i].extract.
					from_hdr.offset);
			params[param] |= mc_enc(32, 32,
					cfg->extracts[i].extract.
					from_hdr.field);
			params[param] = cpu_to_le64(params[param]);
			param++;
			params[param] |= mc_enc(0, 8,
					cfg->extracts[i].extract.
					from_hdr.hdr_index);
			break;
		case DPKG_EXTRACT_FROM_DATA:
			params[param] |= mc_enc(16, 8,
					cfg->extracts[i].extract.
					from_data.size);
			params[param] |= mc_enc(24, 8,
					cfg->extracts[i].extract.
					from_data.offset);
			params[param] = cpu_to_le64(params[param]);
			param++;
			break;
		case DPKG_EXTRACT_FROM_PARSE:
			params[param] |= mc_enc(16, 8,
					cfg->extracts[i].extract.
					from_parse.size);
			params[param] |= mc_enc(24, 8,
					cfg->extracts[i].extract.
					from_parse.offset);
			params[param] = cpu_to_le64(params[param]);
			param++;
			break;
		default:
			return -EINVAL;
		}
		params[param] |= mc_enc(
			24, 8, cfg->extracts[i].num_of_byte_masks);
		params[param] |= mc_enc(32, 4, cfg->extracts[i].type);
		params[param] = cpu_to_le64(params[param]);
		param++;
		for (offset = 0, j = 0;
			j < DPKG_NUM_OF_MASKS;
			offset += 16, j++) {
			params[param] |= mc_enc(
				(offset), 8, cfg->extracts[i].masks[j].mask);
			params[param] |= mc_enc(
				(offset + 8), 8,
				cfg->extracts[i].masks[j].offset);
		}
		params[param] = cpu_to_le64(params[param]);
		param++;
	}
	return 0;
}

int dpni_prepare_extended_cfg(const struct dpni_extended_cfg	*cfg,
			      uint8_t			*ext_cfg_buf)
{
	uint64_t *ext_params = (uint64_t *)ext_cfg_buf;

	DPNI_PREP_EXTENDED_CFG(ext_params, cfg);

	return 0;
}

int dpni_extract_extended_cfg(struct dpni_extended_cfg	*cfg,
			      const uint8_t		*ext_cfg_buf)
{
	uint64_t *ext_params = (uint64_t *)ext_cfg_buf;

	DPNI_EXT_EXTENDED_CFG(ext_params, cfg);

	return 0;
}

int dpni_open(struct fsl_mc_io *mc_io,
	      uint32_t cmd_flags,
	      int dpni_id,
	      uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPNI_CMD_OPEN(cmd, dpni_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpni_close(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	       uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_create(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		const struct dpni_cfg *cfg,
		uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CREATE,
					  cmd_flags,
					  0);
	DPNI_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpni_destroy(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_DESTROY,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_pools(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token,
		   const struct dpni_pools_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_POOLS,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_POOLS(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_enable(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_disable(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_is_enabled(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    int *en)
{
	struct mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_IS_ENABLED, cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

int dpni_reset(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	       uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_irq(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 struct dpni_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_irq(struct fsl_mc_io	*mc_io,
		 uint32_t		cmd_flags,
		 uint16_t		token,
		 uint8_t		irq_index,
		 int			*type,
		 struct dpni_irq_cfg	*irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

int dpni_set_irq_enable(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_irq_enable(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

int dpni_set_irq_mask(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_irq_mask(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

int dpni_get_irq_status(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint32_t *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

int dpni_clear_irq_status(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint32_t status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPNI_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_attributes(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			struct dpni_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_ATTR,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_ATTR(cmd, attr);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_ATTR(cmd, attr);

	return 0;
}

int dpni_set_errors_behavior(struct fsl_mc_io *mc_io,
			     uint32_t cmd_flags,
			     uint16_t token,
			      struct dpni_error_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_ERRORS_BEHAVIOR,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_ERRORS_BEHAVIOR(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_buffer_layout(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_RX_BUFFER_LAYOUT,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_RX_BUFFER_LAYOUT(cmd, layout);

	return 0;
}

int dpni_set_rx_buffer_layout(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      const struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_BUFFER_LAYOUT,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_BUFFER_LAYOUT(cmd, layout);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_buffer_layout(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_BUFFER_LAYOUT,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_TX_BUFFER_LAYOUT(cmd, layout);

	return 0;
}

int dpni_set_tx_buffer_layout(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      const struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_BUFFER_LAYOUT,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_BUFFER_LAYOUT(cmd, layout);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_conf_buffer_layout(struct fsl_mc_io *mc_io,
				   uint32_t cmd_flags,
				   uint16_t token,
				   struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_CONF_BUFFER_LAYOUT,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_TX_CONF_BUFFER_LAYOUT(cmd, layout);

	return 0;
}

int dpni_set_tx_conf_buffer_layout(struct fsl_mc_io *mc_io,
				   uint32_t cmd_flags,
				   uint16_t token,
				   const struct dpni_buffer_layout *layout)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_CONF_BUFFER_LAYOUT,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_CONF_BUFFER_LAYOUT(cmd, layout);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_l3_chksum_validation(struct fsl_mc_io *mc_io,
				  uint32_t cmd_flags,
				  uint16_t token,
				  int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_L3_CHKSUM_VALIDATION,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_L3_CHKSUM_VALIDATION(cmd, *en);

	return 0;
}

int dpni_set_l3_chksum_validation(struct fsl_mc_io *mc_io,
				  uint32_t cmd_flags,
				  uint16_t token,
				  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_L3_CHKSUM_VALIDATION,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_L3_CHKSUM_VALIDATION(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_l4_chksum_validation(struct fsl_mc_io *mc_io,
				  uint32_t cmd_flags,
				  uint16_t token,
				  int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_L4_CHKSUM_VALIDATION,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_L4_CHKSUM_VALIDATION(cmd, *en);

	return 0;
}

int dpni_set_l4_chksum_validation(struct fsl_mc_io *mc_io,
				  uint32_t cmd_flags,
				  uint16_t token,
				  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_L4_CHKSUM_VALIDATION,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_L4_CHKSUM_VALIDATION(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_qdid(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  uint16_t token,
		  uint16_t *qdid)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_QDID,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_QDID(cmd, *qdid);

	return 0;
}

int dpni_get_sp_info(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     struct dpni_sp_info *sp_info)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_SP_INFO,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_SP_INFO(cmd, sp_info);

	return 0;
}

int dpni_get_tx_data_offset(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    uint16_t *data_offset)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_DATA_OFFSET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_TX_DATA_OFFSET(cmd, *data_offset);

	return 0;
}

int dpni_get_counter(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     enum dpni_counter counter,
		     uint64_t *value)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_COUNTER,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_COUNTER(cmd, counter);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_COUNTER(cmd, *value);

	return 0;
}

int dpni_set_counter(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     enum dpni_counter counter,
		     uint64_t value)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_COUNTER,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_COUNTER(cmd, counter, value);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_link_cfg(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      const struct dpni_link_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_LINK_CFG,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_LINK_CFG(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_link_state(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			struct dpni_link_state *state)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_LINK_STATE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_LINK_STATE(cmd, state);

	return 0;
}

int dpni_set_tx_shaping(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			const struct dpni_tx_shaping_cfg *tx_shaper)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_SHAPING,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_SHAPING(cmd, tx_shaper);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_max_frame_length(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t max_frame_length)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_MAX_FRAME_LENGTH(cmd, max_frame_length);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_max_frame_length(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t *max_frame_length)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_MAX_FRAME_LENGTH(cmd, *max_frame_length);

	return 0;
}

int dpni_set_mtu(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint16_t mtu)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_MTU,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_MTU(cmd, mtu);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_mtu(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint16_t *mtu)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_MTU,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_MTU(cmd, *mtu);

	return 0;
}

int dpni_set_multicast_promisc(struct fsl_mc_io *mc_io,
			       uint32_t cmd_flags,
			       uint16_t token,
			       int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_MCAST_PROMISC,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_MULTICAST_PROMISC(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_multicast_promisc(struct fsl_mc_io *mc_io,
			       uint32_t cmd_flags,
			       uint16_t token,
			       int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_MCAST_PROMISC,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_MULTICAST_PROMISC(cmd, *en);

	return 0;
}

int dpni_set_unicast_promisc(struct fsl_mc_io *mc_io,
			     uint32_t cmd_flags,
			     uint16_t token,
			     int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_UNICAST_PROMISC,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_UNICAST_PROMISC(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_unicast_promisc(struct fsl_mc_io *mc_io,
			     uint32_t cmd_flags,
			     uint16_t token,
			     int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_UNICAST_PROMISC,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_UNICAST_PROMISC(cmd, *en);

	return 0;
}

int dpni_set_primary_mac_addr(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      const uint8_t mac_addr[6])
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_PRIM_MAC,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_PRIMARY_MAC_ADDR(cmd, mac_addr);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_primary_mac_addr(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint8_t mac_addr[6])
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_PRIM_MAC,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_PRIMARY_MAC_ADDR(cmd, mac_addr);

	return 0;
}

int dpni_add_mac_addr(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      const uint8_t mac_addr[6])
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_ADD_MAC_ADDR,
					  cmd_flags,
					  token);
	DPNI_CMD_ADD_MAC_ADDR(cmd, mac_addr);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_remove_mac_addr(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 const uint8_t mac_addr[6])
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_REMOVE_MAC_ADDR,
					  cmd_flags,
					  token);
	DPNI_CMD_REMOVE_MAC_ADDR(cmd, mac_addr);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_clear_mac_filters(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   int unicast,
			   int multicast)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLR_MAC_FILTERS,
					  cmd_flags,
					  token);
	DPNI_CMD_CLEAR_MAC_FILTERS(cmd, unicast, multicast);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_vlan_filters(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_VLAN_FILTERS,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_VLAN_FILTERS(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_add_vlan_id(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t vlan_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_ADD_VLAN_ID,
					  cmd_flags,
					  token);
	DPNI_CMD_ADD_VLAN_ID(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_remove_vlan_id(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint16_t vlan_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_REMOVE_VLAN_ID,
					  cmd_flags,
					  token);
	DPNI_CMD_REMOVE_VLAN_ID(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_clear_vlan_filters(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLR_VLAN_FILTERS,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_tx_selection(struct fsl_mc_io			*mc_io,
			  uint32_t				cmd_flags,
			  uint16_t				token,
			  const struct dpni_tx_selection_cfg	*cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_SELECTION,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_SELECTION(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_rx_tc_dist(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t tc_id,
			const struct dpni_rx_tc_dist_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_TC_DIST,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_TC_DIST(cmd, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_tx_flow(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t *flow_id,
		     const struct dpni_tx_flow_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_FLOW,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_FLOW(cmd, *flow_id, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_SET_TX_FLOW(cmd, *flow_id);

	return 0;
}

int dpni_get_tx_flow(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t flow_id,
		     struct dpni_tx_flow_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_FLOW,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_TX_FLOW(cmd, flow_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_TX_FLOW(cmd, attr);

	return 0;
}

int dpni_set_rx_flow(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint8_t tc_id,
		     uint16_t flow_id,
		     const struct dpni_queue_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_FLOW,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_FLOW(cmd, tc_id, flow_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_flow(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint8_t tc_id,
		     uint16_t flow_id,
		     struct dpni_queue_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;
	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_RX_FLOW,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_RX_FLOW(cmd, tc_id, flow_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_RX_FLOW(cmd, attr);

	return 0;
}

int dpni_set_rx_err_queue(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  const struct dpni_queue_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_ERR_QUEUE,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_ERR_QUEUE(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_err_queue(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  struct dpni_queue_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_RX_ERR_QUEUE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPNI_RSP_GET_RX_ERR_QUEUE(cmd, attr);

	return 0;
}

int dpni_set_tx_conf_revoke(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    int		revoke)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_CONF_REVOKE,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_CONF_REVOKE(cmd, revoke);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_qos_table(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       const struct dpni_qos_tbl_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_QOS_TBL,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_QOS_TABLE(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_add_qos_entry(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       const struct dpni_rule_cfg *cfg,
		       uint8_t tc_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_ADD_QOS_ENT,
					  cmd_flags,
					  token);
	DPNI_CMD_ADD_QOS_ENTRY(cmd, cfg, tc_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_remove_qos_entry(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  const struct dpni_rule_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_REMOVE_QOS_ENT,
					  cmd_flags,
					  token);
	DPNI_CMD_REMOVE_QOS_ENTRY(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_clear_qos_table(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLR_QOS_TBL,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_add_fs_entry(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t tc_id,
		      const struct dpni_rule_cfg *cfg,
		      uint16_t flow_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_ADD_FS_ENT,
					  cmd_flags,
					  token);
	DPNI_CMD_ADD_FS_ENTRY(cmd, tc_id, cfg, flow_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_remove_fs_entry(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint8_t tc_id,
			 const struct dpni_rule_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_REMOVE_FS_ENT,
					  cmd_flags,
					  token);
	DPNI_CMD_REMOVE_FS_ENTRY(cmd, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_clear_fs_entries(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t tc_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_CLR_FS_ENT,
					  cmd_flags,
					  token);
	DPNI_CMD_CLEAR_FS_ENTRIES(cmd, tc_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_vlan_insertion(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_VLAN_INSERTION,
					  cmd_flags, token);
	DPNI_CMD_SET_VLAN_INSERTION(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_vlan_removal(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_VLAN_REMOVAL,
					  cmd_flags, token);
	DPNI_CMD_SET_VLAN_REMOVAL(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_ipr(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_IPR,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_IPR(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_ipf(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_IPF,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_IPF(cmd, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_rx_tc_policing(struct fsl_mc_io	*mc_io,
			    uint32_t		cmd_flags,
			    uint16_t		token,
			    uint8_t		tc_id,
			    const struct dpni_rx_tc_policing_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_TC_POLICING,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_TC_POLICING(cmd, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_tc_policing(struct fsl_mc_io			*mc_io,
			    uint32_t				cmd_flags,
			    uint16_t				token,
			    uint8_t				tc_id,
			    struct dpni_rx_tc_policing_cfg	*cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_RX_TC_POLICING,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_RX_TC_POLICING(cmd, tc_id);

	/* send command to mc*/
	err =  mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPNI_RSP_GET_RX_TC_POLICING(cmd, cfg);

	return 0;
}

void dpni_prepare_early_drop(const struct dpni_early_drop_cfg *cfg,
			     uint8_t *early_drop_buf)
{
	uint64_t *ext_params = (uint64_t *)early_drop_buf;

	DPNI_PREP_EARLY_DROP(ext_params, cfg);
}

void dpni_extract_early_drop(struct dpni_early_drop_cfg *cfg,
			     const uint8_t *early_drop_buf)
{
	uint64_t *ext_params = (uint64_t *)early_drop_buf;

	DPNI_EXT_EARLY_DROP(ext_params, cfg);
}

int dpni_set_rx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_RX_TC_EARLY_DROP,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_RX_TC_EARLY_DROP(cmd, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_RX_TC_EARLY_DROP,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_RX_TC_EARLY_DROP(cmd, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_tx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_TC_EARLY_DROP,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_TC_EARLY_DROP(cmd, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_tc_early_drop(struct fsl_mc_io	*mc_io,
			      uint32_t		cmd_flags,
			      uint16_t		token,
			      uint8_t		tc_id,
			      uint64_t		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_TC_EARLY_DROP,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_TX_TC_EARLY_DROP(cmd, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_set_rx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
			const struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
			DPNI_CMDID_SET_RX_TC_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	DPNI_CMD_SET_RX_TC_CONGESTION_NOTIFICATION(cmd, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_rx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
				struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
			DPNI_CMDID_GET_RX_TC_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	DPNI_CMD_GET_RX_TC_CONGESTION_NOTIFICATION(cmd, tc_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPNI_RSP_GET_RX_TC_CONGESTION_NOTIFICATION(cmd, cfg);

	return 0;
}

int dpni_set_tx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
			const struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
			DPNI_CMDID_SET_TX_TC_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	DPNI_CMD_SET_TX_TC_CONGESTION_NOTIFICATION(cmd, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_tc_congestion_notification(struct fsl_mc_io	*mc_io,
					   uint32_t		cmd_flags,
					   uint16_t		token,
					   uint8_t		tc_id,
				struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
			DPNI_CMDID_GET_TX_TC_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	DPNI_CMD_GET_TX_TC_CONGESTION_NOTIFICATION(cmd, tc_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPNI_RSP_GET_TX_TC_CONGESTION_NOTIFICATION(cmd, cfg);

	return 0;
}

int dpni_set_tx_conf(struct fsl_mc_io	*mc_io,
		     uint32_t		cmd_flags,
		     uint16_t		token,
		     uint16_t		flow_id,
		     const struct dpni_tx_conf_cfg	*cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_SET_TX_CONF,
					  cmd_flags,
					  token);
	DPNI_CMD_SET_TX_CONF(cmd, flow_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_conf(struct fsl_mc_io		*mc_io,
		     uint32_t			cmd_flags,
		     uint16_t			token,
		     uint16_t			flow_id,
		     struct dpni_tx_conf_attr	*attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPNI_CMDID_GET_TX_CONF,
					  cmd_flags,
					  token);
	DPNI_CMD_GET_TX_CONF(cmd, flow_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPNI_RSP_GET_TX_CONF(cmd, attr);

	return 0;
}

int dpni_set_tx_conf_congestion_notification(struct fsl_mc_io	*mc_io,
					     uint32_t		cmd_flags,
					     uint16_t		token,
					     uint16_t		flow_id,
			 const struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
			DPNI_CMDID_SET_TX_CONF_CONGESTION_NOTIFICATION,
			cmd_flags,
			token);
	DPNI_CMD_SET_TX_CONF_CONGESTION_NOTIFICATION(cmd, flow_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpni_get_tx_conf_congestion_notification(struct fsl_mc_io	*mc_io,
					     uint32_t		cmd_flags,
					     uint16_t		token,
					     uint16_t		flow_id,
				 struct dpni_congestion_notification_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(
				DPNI_CMDID_GET_TX_CONF_CONGESTION_NOTIFICATION,
				cmd_flags,
				token);
	DPNI_CMD_GET_TX_CONF_CONGESTION_NOTIFICATION(cmd, flow_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPNI_RSP_GET_TX_CONF_CONGESTION_NOTIFICATION(cmd, cfg);

	return 0;
}
