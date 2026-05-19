// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025-2026 NXP
 * This P3H2X4X driver file contain functions for SMBus/I2C virtual Bus creation and read/write.
 */
#include <linux/bitfield.h>
#include <linux/i3c/device.h>
#include <linux/mfd/p3h2840.h>
#include <linux/regmap.h>

#include "p3h2840_i3c_hub.h"

enum p3h2x4x_smbus_desc_idx {
	P3H2X4X_DESC_ADDR,
	P3H2X4X_DESC_TYPE,
	P3H2X4X_DESC_WRITE_LEN,
	P3H2X4X_DESC_READ_LEN,
};

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static void p3h2x4x_read_smbus_agent_rx_buf(struct i3c_device *i3cdev, enum p3h2x4x_rcv_buf rfbuf,
					    enum p3h2x4x_tp tp)
{
	struct p3h2x4x *p3h2x4x = i3cdev_get_drvdata(i3cdev);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
	u8 target_buffer_page, flag_clear, temp = 0, i, addr;
	u32 packet_len, slave_address;
	struct i2c_client *client;
	u8 *slave_rx_buffer;
	int ret;

	if (!p3h2x4x || !p3h2x4x->i3c_hub_priv)
		return;

	p3h2x4x_i3c_hub = p3h2x4x->i3c_hub_priv;
	slave_rx_buffer = p3h2x4x_i3c_hub->slave_rx_buffer;

	switch (rfbuf) {
	case RCV_BUF_0:
		target_buffer_page = P3H2X4X_TARGET_BUFF_0_PAGE;
		flag_clear = P3H2X4X_TARGET_BUF_0_RECEIVE;
		break;
	case RCV_BUF_1:
		target_buffer_page = P3H2X4X_TARGET_BUFF_1_PAGE;
		flag_clear = P3H2X4X_TARGET_BUF_1_RECEIVE;
		break;
	default:
		return;
	}

	target_buffer_page += P3H2X4X_NO_PAGE_PER_TP * tp;

	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_PAGE_PTR, target_buffer_page);
	if (ret)
		goto ibi_err;

	/* read buffer length */
	ret = regmap_read(p3h2x4x_i3c_hub->regmap, P3H2X4X_TARGET_BUFF_LENGTH, &packet_len);
	if (ret)
		goto ibi_err;

	if (packet_len)
		packet_len = packet_len - 1;

	if (packet_len > P3H2X4X_SMBUS_TARGET_PAYLOAD_SIZE) {
		dev_err(&i3cdev->dev, "Received message too big for p3h2x4x buffer\n");
		goto ibi_err;
	}

	/* read slave  address */
	ret = regmap_read(p3h2x4x_i3c_hub->regmap, P3H2X4X_TARGET_BUFF_ADDRESS, &slave_address);
	if (ret)
		goto ibi_err;

	/* read data */
	if (packet_len) {
		ret = regmap_bulk_read(p3h2x4x_i3c_hub->regmap, P3H2X4X_TARGET_BUFF_DATA,
				       slave_rx_buffer, packet_len);
		if (ret)
			goto ibi_err;
	}

	client = p3h2x4x_i3c_hub->tp_bus[tp].tp_smbus_client;
	if (!client)
		goto ibi_err;

	/* notify slave driver about received data */
	if ((client->addr & 0x7f) == (slave_address >> 1)) {
		addr = slave_address >> 1;
		i2c_slave_event(client,
				I2C_SLAVE_WRITE_REQUESTED, &addr);
		for (i = 0; i < packet_len; i++) {
			temp = slave_rx_buffer[i];
			i2c_slave_event(client,
					I2C_SLAVE_WRITE_RECEIVED, &temp);
		}
		i2c_slave_event(client, I2C_SLAVE_STOP, &temp);
	}

ibi_err:
	regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_PAGE_PTR, 0x00);

	regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP0_SMBUS_AGNT_STS + tp, flag_clear);
}

/**
 * p3h2x4x_ibi_handler - IBI handler.
 * @i3cdev: i3c device.
 * @payload: two byte IBI payload data.
 *
 */
void p3h2x4x_ibi_handler(struct i3c_device *i3cdev,
			 const struct i3c_ibi_payload *payload)
{
	struct p3h2x4x *p3h2x4x = i3cdev_get_drvdata(i3cdev);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub;
	u8 payload_byte_one, payload_byte_two;
	u32 target_port_status;
	const u8 *data;
	int ret, i;

	if (!payload || payload->len < P3H2X4X_MAX_PAYLOAD_LEN)
		return;

	data = payload->data;
	payload_byte_one = data[0];

	if (!(payload_byte_one & P3H2X4X_SMBUS_AGENT_EVENT_FLAG_STATUS))
		return;

	p3h2x4x_i3c_hub = p3h2x4x ? p3h2x4x->i3c_hub_priv : NULL;

	if (!p3h2x4x_i3c_hub || !p3h2x4x_i3c_hub->regmap)
		return;

	payload_byte_two = data[1];
	guard(mutex)(&p3h2x4x_i3c_hub->etx_mutex);

	for (i = 0; i < p3h2x4x->num_target_ports; ++i) {
		if (!((payload_byte_two >> i) & 0x01))
			continue;

		if (!p3h2x4x_i3c_hub->tp_bus[i].is_registered) {
			dev_dbg(&i3cdev->dev, "IBI for unregistered SMBus port %u\n", i);
			regmap_write(p3h2x4x_i3c_hub->regmap,
				     P3H2X4X_TP0_SMBUS_AGNT_STS + i,
				     BUF_RECEIVED_FLAG_TF_MASK);
			continue;
		}

		ret = regmap_read(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP0_SMBUS_AGNT_STS + i,
				  &target_port_status);
		if (ret) {
			dev_err(&i3cdev->dev, "target port read status failed %d\n", ret);
			continue;
		}

		if (target_port_status & P3H2X4X_TARGET_BUF_CA_TF)
			regmap_write(p3h2x4x_i3c_hub->regmap,
				     P3H2X4X_TP0_SMBUS_AGNT_STS + i,
				     P3H2X4X_TARGET_BUF_CA_TF);

		/* process data receive buffer */
		switch (FIELD_GET(BUF_RECEIVED_FLAG_MASK, target_port_status)) {
		case P3H2X4X_TARGET_BUF_0_RECEIVE_VAL:
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_0, i);
			break;
		case P3H2X4X_TARGET_BUF_1_RECEIVE_VAL:
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_1, i);
			break;
		case P3H2X4X_TARGET_BUF_0_1_RECEIVE_VAL:
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_0, i);
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_1, i);
			break;
		case P3H2X4X_TARGET_BUF_OVRFL_VAL:
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_0, i);
			p3h2x4x_read_smbus_agent_rx_buf(i3cdev, RCV_BUF_1, i);
			regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP0_SMBUS_AGNT_STS + i,
				     P3H2X4X_TARGET_BUF_OVRFL);
			dev_err(&i3cdev->dev, "Overflow, reading buffer zero and one\n");
			break;
		default:
			regmap_write(p3h2x4x_i3c_hub->regmap,
				     P3H2X4X_TP0_SMBUS_AGNT_STS + i,
				     target_port_status & BUF_RECEIVED_FLAG_MASK);
			break;
		}
	}
}
#endif

static int p3h2x4x_read_smbus_transaction_status(struct p3h2x4x_i3c_hub_dev *hub,
						 u8 target_port_status,
						 u8 data_length)
{
	unsigned int xfer_us, timeout_us, sleep_us;
	u32 status_read;
	u8 status;
	int ret;

	xfer_us = P3H2X4X_SMBUS_400kHz_TRANSFER_TIMEOUT(data_length);
	sleep_us = clamp(xfer_us / P3H2X4X_SMBUS_POLL_COUNT,
			 P3H2X4X_SMBUS_POLL_INTERVAL_MIN_US,
			 P3H2X4X_SMBUS_POLL_INTERVAL_MAX_US);

	/*
	 * Floor the deadline at the SDA-stuck recovery window so an in-spec
	 * bus recovery (agent holding SCL low up to 35 ms) plus status
	 * posting and read-back completes before we return a timeout.
	 */
	timeout_us = max(xfer_us, P3H2X4X_SMBUS_SCL_LOW_RECOVERY_US);

	ret = regmap_read_poll_timeout(hub->regmap, target_port_status,
				       status_read,
				       status_read & P3H2X4X_SMBUS_TRANSACTION_FINISH_FLAG,
				       sleep_us,
				       timeout_us);
	if (ret)
		return ret;

	status = (u8)status_read;

	status = (status & P3H2X4X_TP_TRANSACTION_CODE_MASK)
		  >> P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SHIFT;

	switch (status) {
	case P3H2X4X_SMBUS_CNTRL_STATUS_TXN_OK:
		return 0;
	case P3H2X4X_SMBUS_CNTRL_STATUS_TXN_ADDR_NAK:
		return -ENXIO;
	case P3H2X4X_SMBUS_CNTRL_STATUS_TXN_DATA_NAK:
		return -EIO;
	case P3H2X4X_SMBUS_CNTRL_STATUS_TXN_SCL_TO:
		return -ETIMEDOUT;
	case P3H2X4X_SMBUS_CNTRL_STATUS_TXN_ARB_LOSS:
		return -EAGAIN;
	default:
		return -EIO;
	}
}

/*
 * p3h2x4x_tp_i2c_xfer_msg() - This starts a SMBus write transaction by writing a descriptor
 * and a message to the p3h2x4x registers. Controller buffer page is determined by multiplying the
 * target port index by four and adding the base page number to it.
 */
static int p3h2x4x_tp_i2c_xfer_msg(struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub,
				   struct i2c_msg *xfers,
				   u8 target_port,
				   int nxfers_i, u8 rw)
{
	u8 controller_buffer_page = P3H2X4X_CONTROLLER_BUFFER_PAGE + 4 * target_port;
	u8 target_port_status = P3H2X4X_TP0_SMBUS_AGNT_STS + target_port;
	u8 desc[P3H2X4X_SMBUS_DESCRIPTOR_SIZE] = { 0 };
	u8 transaction_type = P3H2X4X_SMBUS_400kHz;
	int write_length, read_length;
	u8 addr = xfers[nxfers_i].addr;
	u8 rw_address = 2 * addr;
	int ret, ret2;

	if (rw == 2) { /* write and read */
		write_length = xfers[nxfers_i].len;
		read_length =  xfers[nxfers_i + 1].len;
	} else if (rw == 1) {
		rw_address |= P3H2X4X_SET_BIT(0);
		write_length = 0;
		read_length =  xfers[nxfers_i].len;
	} else {
		write_length = xfers[nxfers_i].len;
		read_length = 0;
	}

	desc[P3H2X4X_DESC_ADDR] = rw_address;
	if (rw == 2)
		desc[P3H2X4X_DESC_TYPE] = transaction_type | P3H2X4X_SET_BIT(0);
	else
		desc[P3H2X4X_DESC_TYPE] = transaction_type;
	desc[P3H2X4X_DESC_WRITE_LEN] = write_length;
	desc[P3H2X4X_DESC_READ_LEN] = read_length;

	ret = regmap_write(p3h2x4x_i3c_hub->regmap, target_port_status,
			   P3H2X4X_TP_BUFFER_STATUS_MASK);
	if (ret)
		goto out;

	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_PAGE_PTR, controller_buffer_page);

	if (ret)
		goto out;

	ret = regmap_bulk_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_CONTROLLER_AGENT_BUFF,
				desc, P3H2X4X_SMBUS_DESCRIPTOR_SIZE);

	if (ret)
		goto out;

	if (!(rw % 2) && xfers[nxfers_i].len) {
		ret = regmap_bulk_write(p3h2x4x_i3c_hub->regmap,
					P3H2X4X_CONTROLLER_AGENT_BUFF_DATA,
					xfers[nxfers_i].buf, xfers[nxfers_i].len);
		if (ret)
			goto out;
	}

	ret = regmap_write(p3h2x4x_i3c_hub->regmap, P3H2X4X_TP_SMBUS_AGNT_TRANS_START,
			   p3h2x4x_i3c_hub->tp_bus[target_port].tp_mask);

	if (ret)
		goto out;

	ret = p3h2x4x_read_smbus_transaction_status(p3h2x4x_i3c_hub,
						    target_port_status,
						    (write_length + read_length));
	if (ret)
		goto out;

	if (rw) {
		if (rw == 2)
			nxfers_i += 1;

		if (xfers[nxfers_i].len) {
			u8 *dma_buf = i2c_get_dma_safe_msg_buf(&xfers[nxfers_i], 1);

			if (!dma_buf) {
				ret = -ENOMEM;
				goto out;
			}

			ret = regmap_bulk_read(p3h2x4x_i3c_hub->regmap,
					       P3H2X4X_CONTROLLER_AGENT_BUFF_DATA + write_length,
					       dma_buf, xfers[nxfers_i].len);
			i2c_put_dma_safe_msg_buf(dma_buf, &xfers[nxfers_i], !ret);
			if (ret)
				goto out;
		}
	}
out:
	ret2 = regmap_write(p3h2x4x_i3c_hub->regmap,
			    P3H2X4X_PAGE_PTR, 0x00);
	if (!ret && ret2)
		ret = ret2;

	return ret;
}

/*
 * This function will be called whenever you call I2C read, write APIs like
 * i2c_master_send(), i2c_master_recv() etc.
 */
static s32 p3h2x4x_tp_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	int ret_sum = 0, ret, msg_count;
	u8 rw;

	struct tp_bus *bus = i2c_get_adapdata(adap);
	struct p3h2x4x_i3c_hub_dev *p3h2x4x_i3c_hub = bus->p3h2x4x_i3c_hub;

	guard(mutex)(&p3h2x4x_i3c_hub->etx_mutex);
	guard(mutex)(&bus->port_mutex);

	for (msg_count = 0; msg_count < num; msg_count++) {
		rw = (msgs[msg_count].flags & I2C_M_RD) ? 1 : 0;
		if (!rw) {
			/* If a write message is immediately followed by a read message to
			 * the same address,  consider combining them into a single transaction.
			 */
			if (msg_count + 1 < num &&
			    msgs[msg_count].addr == msgs[msg_count + 1].addr &&
			    (msgs[msg_count + 1].flags & I2C_M_RD)) {
				if (msgs[msg_count].len + msgs[msg_count + 1].len >
				    P3H2X4X_SMBUS_PAYLOAD_SIZE)
					return -EINVAL;

				rw = 2;
				msg_count += 1;
				ret_sum += 1;
			}
		}

		ret = p3h2x4x_tp_i2c_xfer_msg(p3h2x4x_i3c_hub,
					      msgs,
					      bus->tp_port,
					      (rw == 2) ? (msg_count - 1) : msg_count,
					       rw);
		if (ret)
			return ret;

		ret_sum++;
	}
	return ret_sum;
}

static u32 p3h2x4x_tp_smbus_funcs(struct i2c_adapter *adapter)
{
	u32 funcs = I2C_FUNC_I2C | I2C_FUNC_SMBUS_BLOCK_DATA;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct tp_bus *bus = i2c_get_adapdata(adapter);
	struct p3h2x4x_i3c_hub_dev *hub = bus->p3h2x4x_i3c_hub;

	/*
	 * Only advertise slave support when the upstream IBI path is usable.
	 * Otherwise reg_slave() returns -EOPNOTSUPP while functionality()
	 * reports I2C_FUNC_SLAVE, which is inconsistent for callers.
	 */
	if (hub->ibi_ready)
		funcs |= I2C_FUNC_SLAVE;
#endif

	return funcs;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int p3h2x4x_tp_i2c_reg_slave(struct i2c_client *slave)
{
	struct tp_bus *bus = i2c_get_adapdata(slave->adapter);
	struct p3h2x4x_i3c_hub_dev *hub = bus->p3h2x4x_i3c_hub;
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(hub->dev->parent);
	int relock_ret = 0;
	int ret = 0;

	guard(mutex)(&hub->etx_mutex);

	if (!hub->i3cdev || !hub->ibi_ready)
		return -EOPNOTSUPP;

	if (bus->tp_smbus_client)
		return -EBUSY;

	scoped_guard(mutex, &p3h2x4x->protected_reg_lock) {
		/* Unlock access to protected registers */
		ret = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				   P3H2X4X_REGISTERS_UNLOCK_CODE);
		if (ret)
			break;

		ret = regmap_set_bits(hub->regmap,
				      P3H2X4X_TP_SMBUS_AGNT_IBI_CONFIG,
				      bus->tp_mask);

		/* Lock access to protected registers */
		relock_ret = regmap_write(hub->regmap,
					  P3H2X4X_DEV_REG_PROTECTION_CODE,
					  P3H2X4X_REGISTERS_LOCK_CODE);
	}

	/*
	 * A relock failure does not undo the IBI enable, so report it
	 * separately and keep the callback result tied to the IBI operation.
	 */
	if (relock_ret)
		dev_err(hub->dev,
			"failed to restore protected register lock: %d\n",
			relock_ret);

	if (ret)
		return ret;

	/*
	 * Publish the software state only after the hardware IBI has been
	 * enabled successfully.
	 */
	bus->tp_smbus_client = slave;
	hub->hub_config.tp_config[bus->tp_port].ibi_en = true;

	return 0;
}

static int p3h2x4x_tp_i2c_unreg_slave(struct i2c_client *slave)
{
	struct tp_bus *bus = i2c_get_adapdata(slave->adapter);
	struct p3h2x4x_i3c_hub_dev *hub = bus->p3h2x4x_i3c_hub;
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(hub->dev->parent);
	int relock_ret = 0;
	int ret = 0;

	guard(mutex)(&hub->etx_mutex);

	if (bus->tp_smbus_client != slave)
		return -EINVAL;

	scoped_guard(mutex, &p3h2x4x->protected_reg_lock) {
		/* Unlock access to protected registers */
		ret = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				   P3H2X4X_REGISTERS_UNLOCK_CODE);
		if (ret)
			break;

		ret = regmap_clear_bits(hub->regmap,
					P3H2X4X_TP_SMBUS_AGNT_IBI_CONFIG,
					bus->tp_mask);

		/* Lock access to protected registers */
		relock_ret = regmap_write(hub->regmap,
					  P3H2X4X_DEV_REG_PROTECTION_CODE,
					  P3H2X4X_REGISTERS_LOCK_CODE);
	}

	if (relock_ret)
		dev_err(hub->dev,
			"failed to restore protected register lock: %d\n",
			relock_ret);

	if (ret)
		dev_err(hub->dev,
			"failed to disable SMBus agent IBI on port %u: %d\n",
			bus->tp_port, ret);

	/*
	 * Slave backend removal cannot be rolled back. Detach the
	 * callback-visible state even if disabling the hardware IBI failed,
	 * and return 0 so the core completes teardown; the failure is still
	 * visible in the log above. etx_mutex serializes this with the IBI
	 * handler, so later events are acknowledged without invoking the
	 * removed client.
	 */
	bus->tp_smbus_client = NULL;
	hub->hub_config.tp_config[bus->tp_port].ibi_en = false;

	return 0;
}
#endif

static const struct i2c_adapter_quirks p3h2x4x_tp_i2c_quirks = {
	.max_read_len  = P3H2X4X_SMBUS_PAYLOAD_SIZE,
	.max_write_len = P3H2X4X_SMBUS_PAYLOAD_SIZE,
};

/*
 * I2C algorithm Structure
 */
static struct i2c_algorithm p3h2x4x_tp_i2c_algorithm = {
	.master_xfer    = p3h2x4x_tp_i2c_xfer,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave = p3h2x4x_tp_i2c_reg_slave,
	.unreg_slave = p3h2x4x_tp_i2c_unreg_slave,
#endif
	.functionality  = p3h2x4x_tp_smbus_funcs,
};

void p3h2x4x_unregister_smbus_adapters(struct p3h2x4x_i3c_hub_dev *hub)
{
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(hub->dev->parent);
	u8 tp;

	for (tp = 0; tp < p3h2x4x->num_target_ports; tp++) {
		if (!hub->tp_bus[tp].tp_smbus_adapter)
			continue;

		i2c_del_adapter(hub->tp_bus[tp].tp_smbus_adapter);

		guard(mutex)(&hub->etx_mutex);
		hub->tp_bus[tp].tp_smbus_adapter = NULL;
		hub->tp_bus[tp].is_registered = false;
	}
}

/**
 * p3h2x4x_tp_smbus_algo - Register I2C adapters for SMBus target ports.
 * @hub: p3h2x4x device structure.
 * Return: 0 in case of success, negative error code on failure.
 */
int p3h2x4x_tp_smbus_algo(struct p3h2x4x_i3c_hub_dev *hub)
{
	struct p3h2x4x *p3h2x4x = dev_get_drvdata(hub->dev->parent);
	int ret, ret2;
	u8 tp;

	scoped_guard(mutex, &p3h2x4x->protected_reg_lock) {
		ret = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				   P3H2X4X_REGISTERS_UNLOCK_CODE);
		if (ret)
			break;

		ret = regmap_write(hub->regmap, P3H2X4X_TP_SMBUS_AGNT_IBI_CONFIG,
				   P3H2X4X_IBI_DISABLED);

		ret2 = regmap_write(hub->regmap, P3H2X4X_DEV_REG_PROTECTION_CODE,
				    P3H2X4X_REGISTERS_LOCK_CODE);
		if (!ret && ret2)
			ret = ret2;
	}
	if (ret)
		return ret;

	for (tp = 0; tp < p3h2x4x->num_target_ports; tp++) {
		if (!hub->tp_bus[tp].of_node ||
		    hub->hub_config.tp_config[tp].mode != P3H2X4X_TP_MODE_SMBUS)
			continue;

		/* Allocate adapter */
		struct i2c_adapter *smbus_adapter =
			devm_kzalloc(hub->dev, sizeof(*smbus_adapter), GFP_KERNEL);
		if (!smbus_adapter) {
			p3h2x4x_unregister_smbus_adapters(hub);
			return -ENOMEM;
		}

		/* Initialize adapter */
		smbus_adapter->owner = THIS_MODULE;
		smbus_adapter->class = I2C_CLASS_HWMON;
		smbus_adapter->algo = &p3h2x4x_tp_i2c_algorithm;
		smbus_adapter->quirks = &p3h2x4x_tp_i2c_quirks;
		smbus_adapter->dev.parent = hub->dev;
		smbus_adapter->dev.of_node = hub->tp_bus[tp].of_node;
		snprintf(smbus_adapter->name, sizeof(smbus_adapter->name),
			 "p3h2x4x-i3c-hub.tp-port-%d", tp);

		i2c_set_adapdata(smbus_adapter, &hub->tp_bus[tp]);

		/*
		 * Publish the callback-visible state before i2c_add_adapter(),
		 * which can synchronously probe a DT slave and invoke
		 * reg_slave() that inspects is_registered/tp_smbus_client and
		 * sets ibi_en. Seeding defaults here keeps reg_slave()'s view
		 * consistent and avoids clobbering its ibi_en update. Do not
		 * hold etx_mutex across the call, since reg_slave() also takes it.
		 */
		scoped_guard(mutex, &hub->etx_mutex) {
			hub->tp_bus[tp].tp_smbus_adapter = smbus_adapter;
			hub->tp_bus[tp].tp_smbus_client = NULL;
			hub->tp_bus[tp].is_registered = true;
			hub->hub_config.tp_config[tp].ibi_en = false;
		}

		/* Register adapter */
		ret = i2c_add_adapter(smbus_adapter);
		if (ret) {
			scoped_guard(mutex, &hub->etx_mutex) {
				hub->tp_bus[tp].is_registered = false;
				hub->tp_bus[tp].tp_smbus_adapter = NULL;
			}
			p3h2x4x_unregister_smbus_adapters(hub);
			return ret;
		}
	}

	/*
	 * Configure the SMBus Target Agents to hold SDA low when both of a
	 * port's received-data buffers are full. This provides flow control
	 * for MCTP: it prevents MCTP transmitters on the target ports from
	 * timing out when the upstream controller does not service the agent
	 * buffers in time and the port only receives write messages.
	 */
	ret = regmap_update_bits(hub->regmap, P3H2X4X_ONCHIP_TD_AND_SMBUS_AGNT_CONF,
				 P3H2X4X_TARGET_AGENT_DFT_IBI_CONF_MASK,
				 P3H2X4X_TARGET_AGENT_DFT_IBI_CONF);
	if (ret) {
		p3h2x4x_unregister_smbus_adapters(hub);
		return ret;
	}

	return 0;
}
