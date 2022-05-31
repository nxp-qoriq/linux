// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 STMicroelectronics - All Rights Reserved
 * Copyright 2023 NXP
 *
 * The rpmsg tty driver implements serial communication on the RPMsg bus to makes
 * possible for user-space programs to send and receive rpmsg messages as a standard
 * tty protocol.
 *
 * The remote processor can instantiate a new tty by requesting a "srtm-uart-channel" or "rpmsg-tty" RPMsg service.
 * The "srtm-uart-channel" or "rpmsg-tty" service is directly used for data exchange. No flow control is implemented yet.
 */

#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/rpmsg/imx_srtm.h>
#include <linux/of.h>

/*
 * The srtm (simplified real time message) protocol for uart:
 *
 *   +---------------+-------------------------------+
 *   |  Byte Offset  |            Content            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       0       |           Category            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |     1 ~ 2     |           Version             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       3       |             Type              |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       4       |           Command             |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       5       |           Priority            |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       6       |           Reserved1           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       7       |           Reserved2           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       8       |           Reserved3           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       9       |           Reserved4           |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       10      |           UART BUS ID         |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |       11    |         reserved/ret code       |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    12 ~ 13    |           Flags               |
 *   +---------------+---+---+---+---+---+---+---+---+
 *   |    14 ~ 495   |           Data                |
 *   +---------------+---+---+---+---+---+---+---+---+
 * UART BUS ID: real uart instance, such as: the soc has UART0(UART BUS ID = 0), UART1(UART BUS ID = 1), UART2(UART BUS ID = 2), ... , UARTx(UART BUS ID = x)
 */

#define MAX_TTY_RPMSG	32

static DEFINE_IDR(tty_idr);	/* tty instance id */
static DEFINE_MUTEX(idr_lock);	/* protects tty_idr */

static struct tty_driver *rpmsg_tty_driver;

struct rpmsg_tty_port {
	struct tty_port		port;	 /* TTY port data */
	int			id;	 /* TTY rpmsg index */
	int			bus_id; /* used for srtm uart protocol, which real uart will be used, such as, uart0, uart1, uart2, uart3... */
	int			flags; /* used for srtm uart protocol */
	u16			srtm_uart_msg_data_max_sz; /* used for srtm uart protocol */
	bool			use_srtm_uart_protocol; /* used for srtm uart protocol */
	struct rpmsg_device	*rpdev;	 /* rpmsg device */
};

struct srtm_uart_msg_header {
	struct imx_srtm_head common;
	u8 bus_id; /* The bus_id is used when send data from acore to mcore; The bus_id is useless when acore received data that from mcore*/
	union {
		u8 reserved; /* used in request packet */
		u8 retCode; /* used in response packet */
	};
	u16 flags;
} __packed __aligned[1];

struct srtm_uart_msg {
	struct srtm_uart_msg_header header;
	/* srtm uart Payload Start */
	u8 data[1];
} __packed __aligned(1);

struct imx_srtm_uart_data_structure
{
	bool use_srtm_uart_protocol;
};

const static struct imx_srtm_uart_data_structure imx_srtm_uart_data = {
	.use_srtm_uart_protocol = true,
};

const static struct imx_srtm_uart_data_structure rpmsg_tty_data = {
	.use_srtm_uart_protocol = false,
};

static int rpmsg_tty_cb(struct rpmsg_device *rpdev, void *data, int len, void *priv, u32 src)
{
	struct rpmsg_tty_port *cport = dev_get_drvdata(&rpdev->dev);
	struct srtm_uart_msg *msg = (struct srtm_uart_msg *)data;
	int copied;
	u8 *payload = data;
	int payload_len = len;

	if (!payload_len)
		return -EINVAL;
	if (cport->use_srtm_uart_protocol) {
		if (payload_len < (sizeof(struct srtm_uart_msg)))
			return -EINVAL;
		payload_len -= (sizeof(struct srtm_uart_msg) - 1); /* 1: msg->data[0] */

		if (msg->header.common.type != IMX_SRTM_TYPE_RESPONSE && msg->header.common.type != IMX_SRTM_TYPE_NOTIFY) {
			return -EINVAL;
		}

		if (payload_len > cport->srtm_uart_msg_data_max_sz) {
			dev_err(&rpdev->dev,
			"%s failed: data length greater than %d, len=%d\n",
			__func__, cport->srtm_uart_msg_data_max_sz, payload_len);
			return -EINVAL;
		}
		payload = &msg->data[0];
	}

	copied = tty_insert_flip_string(&cport->port, payload, payload_len);
	if (copied != payload_len)
		dev_err_ratelimited(&rpdev->dev, "Trunc buffer: available space is %d\n", copied);
	tty_flip_buffer_push(&cport->port);

	return 0;
}

static int rpmsg_tty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct rpmsg_tty_port *cport = idr_find(&tty_idr, tty->index);

	tty->driver_data = cport;

	return tty_port_install(&cport->port, driver, tty);
}

static int rpmsg_tty_open(struct tty_struct *tty, struct file *filp)
{
	return tty_port_open(tty->port, tty, filp);
}

static void rpmsg_tty_close(struct tty_struct *tty, struct file *filp)
{
	return tty_port_close(tty->port, tty, filp);
}

static int rpmsg_tty_write(struct tty_struct *tty, const u8 *buf, int len)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	struct rpmsg_device *rpdev;
	int msg_max_size, msg_size = -1;
	struct srtm_uart_msg *msg = NULL;
	int ret;
	void *data = NULL;
	int data_len = 0;

	rpdev = cport->rpdev;

	msg_max_size = cport->use_srtm_uart_protocol ? (cport->srtm_uart_msg_data_max_sz) : (rpmsg_get_mtu(rpdev->ept));
	if (msg_max_size < 0)
		return msg_max_size;

	msg_size = min(len, msg_max_size);
	if (cport->use_srtm_uart_protocol) {
		data_len = msg_size + sizeof(struct srtm_uart_msg) - 1; /* srtm uart msg header + data len */
		msg = kmalloc(data_len, GFP_KERNEL);
		if (!msg)
			return -ENOMEM;

		memset(msg, 0, data_len);
		msg->header.common.cate = IMX_SRTM_CATEGORY_UART;
		msg->header.common.major = IMX_SRTM_VER_UART;
		msg->header.common.minor = IMX_SRTM_VER_UART >> 8;
		msg->header.common.type = IMX_SRTM_TYPE_NOTIFY;
		msg->header.common.cmd = IMX_SRTM_UART_COMMAND_SEND;
		msg->header.common.reserved[0] = IMX_SRTM_UART_PRIORITY;
		msg->header.bus_id = cport->bus_id & 0xFF;
		msg->header.flags = cport->flags & 0xFFFF;
		memcpy(&msg->data[0], buf, msg_size);
		data = (void *)msg;
	} else {
		data = (void *)buf;
		data_len = msg_size;
	}
	/*
	 * Use rpmsg_trysend instead of rpmsg_send to send the message so the caller is not
	 * hung until a rpmsg buffer is available. In such case rpmsg_trysend returns -ENOMEM.
	 */
	ret = rpmsg_trysend(rpdev->ept, data, data_len);
	if (ret) {
		dev_dbg_ratelimited(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		msg_size = ret;
	}

	kfree(msg);

	return msg_size;
}

static unsigned int rpmsg_tty_write_room(struct tty_struct *tty)
{
	struct rpmsg_tty_port *cport = tty->driver_data;
	int size;
	struct rpmsg_device *rpdev;

	rpdev = cport->rpdev;
	size = cport->use_srtm_uart_protocol ? (cport->srtm_uart_msg_data_max_sz) : (rpmsg_get_mtu(rpdev->ept));
	if (size < 0)
		return 0;

	return size;
}

static const struct tty_operations rpmsg_tty_ops = {
	.install	= rpmsg_tty_install,
	.open		= rpmsg_tty_open,
	.close		= rpmsg_tty_close,
	.write		= rpmsg_tty_write,
	.write_room	= rpmsg_tty_write_room,
};

static struct rpmsg_tty_port *rpmsg_tty_alloc_cport(void)
{
	struct rpmsg_tty_port *cport;
	int err;

	cport = kzalloc(sizeof(*cport), GFP_KERNEL);
	if (!cport)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&idr_lock);
	cport->id = idr_alloc(&tty_idr, cport, 0, MAX_TTY_RPMSG, GFP_KERNEL);
	mutex_unlock(&idr_lock);

	if (cport->id < 0) {
		err = cport->id;
		kfree(cport);
		return ERR_PTR(err);
	}

	return cport;
}

static void rpmsg_tty_release_cport(struct rpmsg_tty_port *cport)
{
	mutex_lock(&idr_lock);
	idr_remove(&tty_idr, cport->id);
	mutex_unlock(&idr_lock);

	kfree(cport);
}

static const struct tty_port_operations rpmsg_tty_port_ops = { };

static int rpmsg_tty_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_tty_port *cport;
	struct device *dev = &rpdev->dev;
	struct device *tty_dev;
	struct device_node *np;
	struct imx_srtm_uart_data_structure *srtm_uart_data = NULL;
	int ret;
	char buf[64];

	cport = rpmsg_tty_alloc_cport();
	if (IS_ERR(cport)) {
		dev_err(dev, "Failed to alloc tty port\n");
		return PTR_ERR(cport);
	}

	tty_port_init(&cport->port);
	cport->port.ops = &rpmsg_tty_port_ops;

	cport->rpdev = rpdev;

	srtm_uart_data = (struct imx_srtm_uart_data_structure *)rpdev->id.driver_data;
	if (srtm_uart_data && srtm_uart_data->use_srtm_uart_protocol == true) {
		cport->bus_id = 0xFF; /* mcore directly print the data that received from acore */
		cport->use_srtm_uart_protocol = true;
		snprintf(buf, sizeof(buf), "uart-rpbus-%d", cport->id);
		np = of_find_node_by_name(NULL, buf);
		if (np && of_device_is_compatible(np, "fsl,uart-rpbus") && of_device_is_available(np)) {
			of_property_read_u32(np, "bus_id", &cport->bus_id); /* mcore will use the id as uart instance, then write data to the real uart instance */
			of_property_read_u32(np, "flags", &cport->flags);
		}
		cport->srtm_uart_msg_data_max_sz = rpmsg_get_mtu(rpdev->ept) - (sizeof(struct srtm_uart_msg) - 1); /* 1: data[1] of struct srtm_uart_msg */
	} else {
		cport->use_srtm_uart_protocol = false;
	}

	tty_dev = tty_port_register_device(&cport->port, rpmsg_tty_driver,
					   cport->id, dev);
	if (IS_ERR(tty_dev)) {
		dev_err(dev, "Failed to register tty port\n");
		ret = PTR_ERR(tty_dev);
		goto  err_destroy;
	}

	dev_set_drvdata(dev, (void *)cport);

	dev_dbg(dev, "New channel: 0x%x -> 0x%x : ttyRPMSG%d\n",
		rpdev->src, rpdev->dst, cport->id);

	return 0;

err_destroy:
	tty_port_destroy(&cport->port);
	rpmsg_tty_release_cport(cport);

	return ret;
}

static void rpmsg_tty_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_tty_port *cport = dev_get_drvdata(&rpdev->dev);

	dev_dbg(&rpdev->dev, "Removing rpmsg tty device %d\n", cport->id);

	/* User hang up to release the tty */
	if (tty_port_initialized(&cport->port))
		tty_port_tty_hangup(&cport->port, false);

	tty_unregister_device(rpmsg_tty_driver, cport->id);

	tty_port_destroy(&cport->port);
	rpmsg_tty_release_cport(cport);
}

static struct rpmsg_device_id rpmsg_driver_tty_id_table[] = {
	{ .name	= "srtm-uart-channel", .driver_data = (kernel_ulong_t)&imx_srtm_uart_data, },
	{ .name	= "rpmsg-tty", .driver_data = (kernel_ulong_t)&rpmsg_tty_data, },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_tty_id_table);

static struct rpmsg_driver rpmsg_tty_rpmsg_drv = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_driver_tty_id_table,
	.probe		= rpmsg_tty_probe,
	.callback	= rpmsg_tty_cb,
	.remove		= rpmsg_tty_remove,
};

static int __init rpmsg_tty_init(void)
{
	int err;

	rpmsg_tty_driver = tty_alloc_driver(MAX_TTY_RPMSG, TTY_DRIVER_REAL_RAW |
					    TTY_DRIVER_DYNAMIC_DEV);
	if (IS_ERR(rpmsg_tty_driver))
		return PTR_ERR(rpmsg_tty_driver);

	rpmsg_tty_driver->driver_name = "rpmsg_tty";
	rpmsg_tty_driver->name = "ttyRPMSG";
	rpmsg_tty_driver->major = 0;
	rpmsg_tty_driver->type = TTY_DRIVER_TYPE_CONSOLE;

	/* Disable unused mode by default */
	rpmsg_tty_driver->init_termios = tty_std_termios;
	rpmsg_tty_driver->init_termios.c_lflag &= ~(ECHO | ICANON);
	rpmsg_tty_driver->init_termios.c_oflag &= ~(OPOST | ONLCR);

	tty_set_operations(rpmsg_tty_driver, &rpmsg_tty_ops);

	err = tty_register_driver(rpmsg_tty_driver);
	if (err < 0) {
		pr_err("Couldn't install rpmsg tty driver: err %d\n", err);
		goto error_put;
	}

	err = register_rpmsg_driver(&rpmsg_tty_rpmsg_drv);
	if (err < 0) {
		pr_err("Couldn't register rpmsg tty driver: err %d\n", err);
		goto error_unregister;
	}

	return 0;

error_unregister:
	tty_unregister_driver(rpmsg_tty_driver);

error_put:
	tty_driver_kref_put(rpmsg_tty_driver);

	return err;
}

static void __exit rpmsg_tty_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_tty_rpmsg_drv);
	tty_unregister_driver(rpmsg_tty_driver);
	tty_driver_kref_put(rpmsg_tty_driver);
	idr_destroy(&tty_idr);
}

module_init(rpmsg_tty_init);
module_exit(rpmsg_tty_exit);

MODULE_AUTHOR("Arnaud Pouliquen <arnaud.pouliquen@foss.st.com>");
MODULE_AUTHOR("Biwen Li <biwen.li@nxp.com>");
MODULE_DESCRIPTION("remote processor messaging tty driver");
MODULE_LICENSE("GPL v2");
