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
#include "dpsw.h"
#include "dpsw-cmd.h"

/* internal functions */
static void build_if_id_bitmap(const uint16_t *if_id,
			       const uint16_t num_ifs,
			       struct mc_command *cmd,
			       int start_param)
{
	int i;

	for (i = 0; (i < num_ifs) && (i < DPSW_MAX_IF); i++)
		cmd->params[start_param + (if_id[i] / 64)] |= mc_enc(
			(if_id[i] % 64), 1, 1);
}

static int read_if_id_bitmap(uint16_t *if_id,
			     uint16_t *num_ifs,
			     struct mc_command *cmd,
			     int start_param)
{
	int bitmap[DPSW_MAX_IF] = { 0 };
	int i, j = 0;
	int count = 0;

	for (i = 0; i < DPSW_MAX_IF; i++) {
		bitmap[i] = (int)mc_dec(cmd->params[start_param + i / 64],
					 i % 64, 1);
		count += bitmap[i];
	}

	*num_ifs = (uint16_t)count;

	for (i = 0; (i < DPSW_MAX_IF) && (j < count); i++) {
		if (bitmap[i]) {
			if_id[j] = (uint16_t)i;
			j++;
		}
	}

	return 0;
}

/* DPSW APIs */
int dpsw_open(struct fsl_mc_io *mc_io,
	      uint32_t cmd_flags,
	      int dpsw_id,
	      uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_OPEN,
					  cmd_flags,
					  0);
	DPSW_CMD_OPEN(cmd, dpsw_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpsw_close(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLOSE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_create(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		const struct dpsw_cfg *cfg,
		uint16_t *token)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CREATE,
					  cmd_flags,
					  0);
	DPSW_CMD_CREATE(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	*token = MC_CMD_HDR_READ_TOKEN(cmd.header);

	return 0;
}

int dpsw_destroy(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_DESTROY,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_enable(struct fsl_mc_io *mc_io,
		uint32_t cmd_flags,
		uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ENABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_disable(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_is_enabled(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    int *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IS_ENABLED, cmd_flags,
					  token);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IS_ENABLED(cmd, *en);

	return 0;
}

int dpsw_reset(struct fsl_mc_io *mc_io,
	       uint32_t cmd_flags,
	       uint16_t token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_RESET,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_set_irq(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint8_t irq_index,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ(cmd, irq_index, irq_cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_get_irq(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint8_t irq_index,
		 int *type,
		 struct dpsw_irq_cfg *irq_cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ(cmd, *type, irq_cfg);

	return 0;
}

int dpsw_set_irq_enable(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ_ENABLE(cmd, irq_index, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_get_irq_enable(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint8_t *en)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_ENABLE(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_ENABLE(cmd, *en);

	return 0;
}

int dpsw_set_irq_mask(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t mask)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_IRQ_MASK(cmd, irq_index, mask);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_get_irq_mask(struct fsl_mc_io *mc_io,
		      uint32_t cmd_flags,
		      uint16_t token,
		      uint8_t irq_index,
		      uint32_t *mask)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_MASK,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_MASK(cmd, irq_index);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_MASK(cmd, *mask);

	return 0;
}

int dpsw_get_irq_status(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint8_t irq_index,
			uint32_t *status)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_GET_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPSW_CMD_GET_IRQ_STATUS(cmd, irq_index, *status);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_GET_IRQ_STATUS(cmd, *status);

	return 0;
}

int dpsw_clear_irq_status(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint8_t irq_index,
			  uint32_t status)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CLEAR_IRQ_STATUS,
					  cmd_flags,
					  token);
	DPSW_CMD_CLEAR_IRQ_STATUS(cmd, irq_index, status);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_get_attributes(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			struct dpsw_attr *attr)
{
	struct mc_command cmd = { 0 };
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
	DPSW_RSP_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_set_reflection_if(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_REFLECTION_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_SET_REFLECTION_IF(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_link_cfg(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint16_t if_id,
			 struct dpsw_link_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_LINK_CFG,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_LINK_CFG(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_get_link_state(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t if_id,
			   struct dpsw_link_state *state)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_LINK_STATE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_LINK_STATE(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_LINK_STATE(cmd, state);

	return 0;
}

int dpsw_if_set_flooding(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint16_t if_id,
			 int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_broadcast(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint16_t if_id,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_BROADCAST,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_multicast(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint16_t if_id,
			  int en)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING(cmd, if_id, en);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_tci(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t if_id,
		    const struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TCI,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_TCI(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_get_tci(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t if_id,
		    struct dpsw_tci_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err = 0;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_TCI,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_TCI(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_TCI(cmd, cfg);

	return 0;
}

int dpsw_if_set_stp(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t if_id,
		    const struct dpsw_stp_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_STP,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_STP(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_accepted_frames(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
				uint16_t token,
				uint16_t if_id,
				const struct dpsw_accepted_frames_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_ACCEPTED_FRAMES,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_ACCEPTED_FRAMES(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_accept_all_vlan(struct fsl_mc_io *mc_io,
				uint32_t cmd_flags,
				uint16_t token,
				uint16_t if_id,
				int accept_all)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_SET_IF_ACCEPT_ALL_VLAN,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_ACCEPT_ALL_VLAN(cmd, if_id, accept_all);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_get_counter(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint16_t if_id,
			enum dpsw_counter type,
			uint64_t *counter)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_COUNTER,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_COUNTER(cmd, if_id, type);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_COUNTER(cmd, *counter);

	return 0;
}

int dpsw_if_set_counter(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint16_t if_id,
			enum dpsw_counter type,
			uint64_t counter)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_COUNTER,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_COUNTER(cmd, if_id, type, counter);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_tx_selection(struct fsl_mc_io *mc_io,
			     uint32_t cmd_flags,
			     uint16_t token,
			     uint16_t if_id,
			     const struct dpsw_tx_selection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_TX_SELECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_TX_SELECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_add_reflection(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t if_id,
			   const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ADD_REFLECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_ADD_REFLECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_remove_reflection(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t if_id,
			      const struct dpsw_reflection_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_REMOVE_REFLECTION,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_REMOVE_REFLECTION(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_flooding_metering(struct fsl_mc_io *mc_io,
				  uint32_t cmd_flags,
				  uint16_t token,
				  uint16_t if_id,
				  const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_FLOODING_METERING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_FLOODING_METERING(cmd, if_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_set_metering(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint16_t if_id,
			 uint8_t tc_id,
			 const struct dpsw_metering_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_METERING,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_METERING(cmd, if_id, tc_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

void dpsw_prepare_early_drop(const struct dpsw_early_drop_cfg *cfg,
			     uint8_t *early_drop_buf)
{
	uint64_t *ext_params = (uint64_t *)early_drop_buf;

	DPSW_PREP_EARLY_DROP(ext_params, cfg);
}

int dpsw_if_set_early_drop(struct fsl_mc_io	*mc_io,
			   uint32_t		cmd_flags,
			   uint16_t		token,
			   uint16_t		if_id,
			   uint8_t		tc_id,
			   uint64_t		early_drop_iova)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_EARLY_DROP,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_EARLY_DROP(cmd, if_id, tc_id, early_drop_iova);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_add_custom_tpid(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ADD_CUSTOM_TPID,
					  cmd_flags,
					  token);
	DPSW_CMD_ADD_CUSTOM_TPID(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_remove_custom_tpid(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    const struct dpsw_custom_tpid_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_REMOVE_CUSTOM_TPID,
					  cmd_flags,
					  token);
	DPSW_CMD_REMOVE_CUSTOM_TPID(cmd, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_enable(struct fsl_mc_io *mc_io,
		   uint32_t cmd_flags,
		   uint16_t token,
		   uint16_t if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_ENABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_ENABLE(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_disable(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t if_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_DISABLE,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_DISABLE(cmd, if_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_get_attributes(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t if_id,
			   struct dpsw_if_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_ATTR(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_IF_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_if_set_max_frame_length(struct fsl_mc_io *mc_io,
				 uint32_t cmd_flags,
				 uint16_t token,
				 uint16_t if_id,
				 uint16_t frame_length)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_SET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_SET_MAX_FRAME_LENGTH(cmd, if_id, frame_length);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_if_get_max_frame_length(struct fsl_mc_io *mc_io,
				 uint32_t cmd_flags,
				 uint16_t token,
				 uint16_t if_id,
				 uint16_t *frame_length)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_IF_GET_MAX_FRAME_LENGTH,
					  cmd_flags,
					  token);
	DPSW_CMD_IF_GET_MAX_FRAME_LENGTH(cmd, if_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	DPSW_RSP_IF_GET_MAX_FRAME_LENGTH(cmd, *frame_length);

	return 0;
}

int dpsw_vlan_add(struct fsl_mc_io *mc_io,
		  uint32_t cmd_flags,
		  uint16_t token,
		  uint16_t vlan_id,
		  const struct dpsw_vlan_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD(cmd, vlan_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_add_if(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t vlan_id,
		     const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_add_if_untagged(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_add_if_flooding(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t vlan_id,
			      const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_ADD_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_ADD_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_remove_if(struct fsl_mc_io *mc_io,
			uint32_t cmd_flags,
			uint16_t token,
			uint16_t vlan_id,
			const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_remove_if_untagged(struct fsl_mc_io *mc_io,
				 uint32_t cmd_flags,
				 uint16_t token,
				 uint16_t vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_remove_if_flooding(struct fsl_mc_io *mc_io,
				 uint32_t cmd_flags,
				 uint16_t token,
				 uint16_t vlan_id,
				 const struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_remove(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t vlan_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_REMOVE(cmd, vlan_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_vlan_get_attributes(struct fsl_mc_io *mc_io,
			     uint32_t cmd_flags,
			     uint16_t token,
			     uint16_t vlan_id,
			     struct dpsw_vlan_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_ATTRIBUTES,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_ATTR(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_vlan_get_if(struct fsl_mc_io *mc_io,
		     uint32_t cmd_flags,
		     uint16_t token,
		     uint16_t vlan_id,
		     struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

int dpsw_vlan_get_if_flooding(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_FLOODING,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF_FLOODING(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF_FLOODING(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

int dpsw_vlan_get_if_untagged(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t vlan_id,
			      struct dpsw_vlan_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_VLAN_GET_IF_UNTAGGED,
					  cmd_flags,
					  token);
	DPSW_CMD_VLAN_GET_IF_UNTAGGED(cmd, vlan_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_VLAN_GET_IF(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 1);

	return 0;
}

int dpsw_fdb_add(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint16_t *fdb_id,
		 const struct dpsw_fdb_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_ADD(cmd, *fdb_id);

	return 0;
}

int dpsw_fdb_remove(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t fdb_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE(cmd, fdb_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_add_unicast(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint16_t fdb_id,
			 const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD_UNICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_get_unicast(struct fsl_mc_io *mc_io,
			 uint32_t cmd_flags,
			 uint16_t token,
			 uint16_t fdb_id,
			 struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_UNICAST(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_UNICAST(cmd, cfg);

	return 0;
}

int dpsw_fdb_remove_unicast(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    uint16_t fdb_id,
			    const struct dpsw_fdb_unicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_UNICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE_UNICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_add_multicast(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t fdb_id,
			   const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 2);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_ADD_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_ADD_MULTICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_get_multicast(struct fsl_mc_io *mc_io,
			   uint32_t cmd_flags,
			   uint16_t token,
			   uint16_t fdb_id,
			   struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_MULTICAST(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_MULTICAST(cmd, cfg);
	read_if_id_bitmap(cfg->if_id, &cfg->num_ifs, &cmd, 2);

	return 0;
}

int dpsw_fdb_remove_multicast(struct fsl_mc_io *mc_io,
			      uint32_t cmd_flags,
			      uint16_t token,
			      uint16_t fdb_id,
			      const struct dpsw_fdb_multicast_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 2);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_REMOVE_MULTICAST,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_REMOVE_MULTICAST(cmd, fdb_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_set_learning_mode(struct fsl_mc_io *mc_io,
			       uint32_t cmd_flags,
			       uint16_t token,
			       uint16_t fdb_id,
			       enum dpsw_fdb_learning_mode mode)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_SET_LEARNING_MODE,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_SET_LEARNING_MODE(cmd, fdb_id, mode);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_fdb_get_attributes(struct fsl_mc_io *mc_io,
			    uint32_t cmd_flags,
			    uint16_t token,
			    uint16_t fdb_id,
			    struct dpsw_fdb_attr *attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_FDB_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_FDB_GET_ATTR(cmd, fdb_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_FDB_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_acl_add(struct fsl_mc_io *mc_io,
		 uint32_t cmd_flags,
		 uint16_t token,
		 uint16_t *acl_id,
		 const struct dpsw_acl_cfg  *cfg)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD(cmd, cfg);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_ACL_ADD(cmd, *acl_id);

	return 0;
}

int dpsw_acl_remove(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t acl_id)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE(cmd, acl_id);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

void dpsw_acl_prepare_entry_cfg(const struct dpsw_acl_key *key,
				uint8_t *entry_cfg_buf)
{
	uint64_t *ext_params = (uint64_t *)entry_cfg_buf;

	DPSW_PREP_ACL_ENTRY(ext_params, key);
}

int dpsw_acl_add_entry(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       uint16_t acl_id,
		       const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_ENTRY,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD_ENTRY(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_acl_remove_entry(struct fsl_mc_io *mc_io,
			  uint32_t cmd_flags,
			  uint16_t token,
			  uint16_t acl_id,
			  const struct dpsw_acl_entry_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_ENTRY,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE_ENTRY(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_acl_add_if(struct fsl_mc_io *mc_io,
		    uint32_t cmd_flags,
		    uint16_t token,
		    uint16_t acl_id,
		    const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_ADD_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_ADD_IF(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_acl_remove_if(struct fsl_mc_io *mc_io,
		       uint32_t cmd_flags,
		       uint16_t token,
		       uint16_t acl_id,
		       const struct dpsw_acl_if_cfg *cfg)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	build_if_id_bitmap(cfg->if_id, cfg->num_ifs, &cmd, 1);
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_REMOVE_IF,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_REMOVE_IF(cmd, acl_id, cfg);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_acl_get_attributes(struct fsl_mc_io		*mc_io,
			    uint32_t			cmd_flags,
			    uint16_t			token,
			    uint16_t			acl_id,
			    struct dpsw_acl_attr	*attr)
{
	struct mc_command cmd = { 0 };
	int err;

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_ACL_GET_ATTR,
					  cmd_flags,
					  token);
	DPSW_CMD_ACL_GET_ATTR(cmd, acl_id);

	/* send command to mc*/
	err = mc_send_command(mc_io, &cmd);
	if (err)
		return err;

	/* retrieve response parameters */
	DPSW_RSP_ACL_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_ctrl_if_get_attributes(struct fsl_mc_io		*mc_io,
				uint32_t			cmd_flags,
				uint16_t			token,
				struct dpsw_ctrl_if_attr	*attr)
{
	struct mc_command cmd = { 0 };
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
	DPSW_RSP_CTRL_IF_GET_ATTR(cmd, attr);

	return 0;
}

int dpsw_ctrl_if_set_pools(struct fsl_mc_io			*mc_io,
			   uint32_t				cmd_flags,
			   uint16_t				token,
			   const struct dpsw_ctrl_if_pools_cfg *pools)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_SET_POOLS,
					  cmd_flags,
					  token);
	DPSW_CMD_CTRL_IF_SET_POOLS(cmd, pools);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}

int dpsw_ctrl_if_enable(struct fsl_mc_io	*mc_io,
			uint32_t		cmd_flags,
			uint16_t		token)
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
* @brief    Function disables control interface
* @mc_io:	Pointer to MC portal's I/O object
* @cmd_flags:	Command flags; one or more of 'MC_CMD_FLAG_'
* @token:	Token of DPSW object
*
* Return:	'0' on Success; Error code otherwise.
*/
int dpsw_ctrl_if_disable(struct fsl_mc_io	*mc_io,
			 uint32_t		cmd_flags,
			 uint16_t		token)
{
	struct mc_command cmd = { 0 };

	/* prepare command */
	cmd.header = mc_encode_cmd_header(DPSW_CMDID_CTRL_IF_DISABLE,
					  cmd_flags,
					  token);

	/* send command to mc*/
	return mc_send_command(mc_io, &cmd);
}
