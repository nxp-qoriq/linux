// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 NXP
 */

#include <linux/spi/spi.h>
#include "hms_switch.h"

int hms_xfer_cmd(const struct hms_private *priv,
		  enum hms_spi_rw_mode rw, enum hms_cmd cmd,
		  void *param, size_t param_len,
		  void *resp, size_t resp_len,
		  struct ptp_system_timestamp *ptp_sts)
{
	struct hms_cmd_hdr hdr = {0};
	struct spi_device *spi = priv->spidev;
	struct spi_transfer hdr_xfer, resp_xfer;
	int rc;

	if (!IS_ALIGNED(resp_len, HMS_SPI_MSG_WORD_BYTES)) {
		dev_err(&spi->dev, "hms cmd %d data size should be a multiple of 4  : %ld",
			 cmd, resp_len);
		return -EINVAL;
	}

	if (resp_len > priv->max_xfer_len) {
		dev_err(&spi->dev, "hms cmd %d data size is too large\n",
			cmd);
		return -EINVAL;
	}

	if (param_len > HMS_SPI_MSG_PARAM_SIZE) {
		dev_err(&spi->dev, "hms cmd %d param size is too large\n",
			cmd);
		return -EINVAL;
	}

	hdr.cmd = (rw << HMS_CMD_DIR_SHIFT) |
		  ((resp_len / HMS_SPI_MSG_WORD_BYTES) <<
		   HMS_CMD_LEN_SHIFT) |
		  cmd;
	if (param)
		memcpy(hdr.param, param, param_len);

	hdr_xfer.tx_buf = &hdr;
	hdr_xfer.len = HMS_SPI_MSG_HEADER_SIZE;
	hdr_xfer.ptp_sts_word_pre = hdr_xfer.len - 1;
	hdr_xfer.ptp_sts_word_post = hdr_xfer.len - 1;
	hdr_xfer.ptp_sts = ptp_sts;

	rc = spi_sync_transfer(spi, &hdr_xfer, 1);
	if (rc < 0) {
		dev_err(&spi->dev, "hms cmd %d SPI transfer failed: %d\n",
			cmd, rc);
		return rc;
	}

	usleep_range(HMS_SPI_MSG_RESPONSE_TIME,
		     HMS_SPI_MSG_RESPONSE_TIME * 10);

	if (!resp)
		return 0;

	/* Populate the transfer's data buffer */
	if (rw == SPI_READ)
		resp_xfer.rx_buf = resp;
	else
		resp_xfer.tx_buf = resp;
	resp_xfer.len = resp_len;

	resp_xfer.ptp_sts_word_pre = resp_xfer.len - 1;
	resp_xfer.ptp_sts_word_post = resp_xfer.len - 1;
	resp_xfer.ptp_sts = ptp_sts;

	rc = spi_sync_transfer(spi, &resp_xfer, 1);
	if (rc < 0) {
		dev_err(&spi->dev, "hms cmd %d SPI transfer failed: %d\n",
			cmd, rc);
		return rc;
	}

	usleep_range(HMS_SPI_MSG_RESPONSE_TIME,
		     HMS_SPI_MSG_RESPONSE_TIME * 10);

	return 0;
}

int hms_xfer_set_cmd(const struct hms_private *priv,
		      enum hms_cmd cmd,
		      void *param, size_t param_len)
{
	return hms_xfer_cmd(priv, SPI_WRITE, cmd,
			     param, param_len,
			     NULL, 0, NULL);
}

int hms_xfer_get_cmd(const struct hms_private *priv,
			enum hms_cmd cmd, uint32_t id,
			void *resp, size_t resp_len)
{
	struct hms_cmd_read_param param;

	param.id = id;

	return hms_xfer_cmd(priv, SPI_READ, cmd,
			     &param, sizeof(param),
			     resp, resp_len, NULL);
}

int hms_xfer_write_reg(const struct hms_private *priv,
			uint32_t reg, uint32_t value)
{
	struct hms_cmd_reg_cmd reg_cmd;

	reg_cmd.reg = reg;
	reg_cmd.value = value;

	return hms_xfer_set_cmd(priv, HMS_CMD_REG_SET,
				 &reg_cmd, sizeof(reg_cmd));
}

int hms_xfer_read_reg(const struct hms_private *priv,
		       uint32_t reg, uint32_t *value)
{
	return hms_xfer_get_cmd(priv, HMS_CMD_REG_GET, reg,
				 value, sizeof(*value));
}

int hms_xfer_write_u64(const struct hms_private *priv,
			enum hms_cmd cmd, uint64_t value,
			struct ptp_system_timestamp *ptp_sts)
{
	return hms_xfer_cmd(priv, SPI_WRITE, cmd,
			     &value, sizeof(value),
			     NULL, 0,
			     ptp_sts);
}

int hms_xfer_read_u64(const struct hms_private *priv,
		       enum hms_cmd cmd, uint64_t *value,
		       struct ptp_system_timestamp *ptp_sts)
{
	return hms_xfer_cmd(priv, SPI_READ, cmd,
			     NULL, 0,
			     value, sizeof(*value),
			     ptp_sts);
}
