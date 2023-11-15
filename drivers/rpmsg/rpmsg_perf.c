// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 NXP
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <uapi/linux/rpmsg.h>

#include "rpmsg_internal.h"

#define RPMSG_PERF_AS_SENDER_IOCTL		_IO(0xb5, 0x5)
#define RPMSG_PERF_AS_RECEIVER_IOCTL		_IO(0xb5, 0x6)
#define RPMSG_PERF_AS_RECEIVER_END_ACK_IOCTL	_IO(0xb5, 0x7)
#define RPMSG_PERF_GET_RUNNING_STA_IOCTL	_IO(0xb5, 0x8)

#define RPMSG_DEV_MAX				(MINORMASK + 1)

static dev_t rpmsg_major;

static DEFINE_IDA(rpmsg_perf_minor_ida);

#define dev_to_eptdev(dev) container_of(dev, struct rpmsg_eptdev, dev)
#define cdev_to_eptdev(i_cdev) container_of(i_cdev, struct rpmsg_eptdev, cdev)

struct packet_header {
	uint32_t preamble;
	bool no_copy;
	uint32_t packet_size;
	uint32_t packet_cnt;
	uint32_t test_time; /* unit: second */
};

enum {
	RPMSG_PERF_PREAMBLE_SENDER_START = 0xBECAACEA,
	RPMSG_PERF_PREAMBLE_SENDER_END = 0xBECAACEB,
	RPMSG_PERF_PREAMBLE_SENDER_END_ACK = 0xBECAACEC,
	RPMSG_PERF_PREAMBLE_RECEIVER_START = 0xBECAACED,
	RPMSG_PERF_PREAMBLE_RECEIVER_END = 0xBECAACEE,
	RPMSG_PERF_PREAMBLE_RECEIVER_END_ACK = 0xBECAACEF,
};

enum dev_state {
	RPMSG_DEV_IDLE,
	RPMSG_DEV_SENDING,
	RPMSG_DEV_RECEIVING,
};

struct test_statistic {
	uint32_t recv_packet_cnt;
	uint32_t send_packet_cnt;
	uint32_t packet_size;
	uint32_t test_time;
};

/**
 * struct rpmsg_eptdev - endpoint device context
 * @dev:	endpoint device
 * @cdev:	cdev for the endpoint device
 * @rpdev:	underlaying rpmsg device
 * @chinfo:	info used to open the endpoint
 * @ept_lock:	synchronization of @ept modifications
 * @ept:	rpmsg endpoint reference, when open
 * @default_ept: set to channel default endpoint if the default endpoint should be re-used
 *              on device open to prevent endpoint address update.
 */
struct rpmsg_eptdev {
	struct device dev;
	struct cdev cdev;

	struct rpmsg_device *rpdev;
	struct rpmsg_channel_info chinfo;

	struct mutex ept_lock;
	struct rpmsg_endpoint *ept;
	struct rpmsg_endpoint *default_ept;

	enum dev_state state;
	struct test_statistic statistic;

	struct packet_header param;
};

static int rpmsg_perf_ept_cb(struct rpmsg_device *rpdev, void *buf, int len,
			     void *priv, u32 addr)
{
	struct packet_header *hdr = (struct packet_header *)buf;
	struct rpmsg_eptdev *eptdev = priv;
	struct test_statistic *statistic = &eptdev->statistic;
	uint32_t rate;

	switch (hdr->preamble) {
	case RPMSG_PERF_PREAMBLE_SENDER_END_ACK:
		if (eptdev->state == RPMSG_DEV_SENDING) {
			eptdev->state = RPMSG_DEV_IDLE;
			rate = statistic->send_packet_cnt /
				statistic->test_time / 1000;
			pr_info("packet size: %u, sent packets: %u, time: %u s, rate: %u kpps\n",
				statistic->packet_size,
				statistic->send_packet_cnt,
				statistic->test_time, rate);
			statistic->packet_size = 0;
			statistic->send_packet_cnt = 0;
		}
		break;
	default:
		break;
	}
	statistic->recv_packet_cnt++;

	return 0;
}

static int rpmsg_perf_eptdev_open(struct inode *inode, struct file *filp)
{
	struct rpmsg_eptdev *eptdev = cdev_to_eptdev(inode->i_cdev);
	struct rpmsg_endpoint *ept;
	struct rpmsg_device *rpdev = eptdev->rpdev;
	struct device *dev = &eptdev->dev;

	mutex_lock(&eptdev->ept_lock);
	if (eptdev->ept) {
		mutex_unlock(&eptdev->ept_lock);
		return -EBUSY;
	}

	get_device(dev);

	/*
	 * If the default_ept is set, the rpmsg device default endpoint is used.
	 * Else a new endpoint is created on open that will be destroyed on release.
	 */
	if (eptdev->default_ept)
		ept = eptdev->default_ept;
	else
		ept = rpmsg_create_ept(rpdev, rpmsg_perf_ept_cb, eptdev, eptdev->chinfo);

	if (!ept) {
		dev_err(dev, "failed to open %s\n", eptdev->chinfo.name);
		put_device(dev);
		mutex_unlock(&eptdev->ept_lock);
		return -EINVAL;
	}

	eptdev->ept = ept;
	filp->private_data = eptdev;
	mutex_unlock(&eptdev->ept_lock);

	return 0;
}

static int rpmsg_perf_eptdev_release(struct inode *inode, struct file *filp)
{
	struct rpmsg_eptdev *eptdev = cdev_to_eptdev(inode->i_cdev);
	struct device *dev = &eptdev->dev;

	/* Close the endpoint, if it's not already destroyed by the parent */
	mutex_lock(&eptdev->ept_lock);
	if (eptdev->ept) {
		if (!eptdev->default_ept)
			rpmsg_destroy_ept(eptdev->ept);
		eptdev->ept = NULL;
	}
	mutex_unlock(&eptdev->ept_lock);

	put_device(dev);

	return 0;
}

static int rpmsg_perf_sender_thread(void *p)
{
	struct rpmsg_eptdev *eptdev = (struct rpmsg_eptdev *)p;
	struct packet_header hdr = eptdev->param;
	unsigned long timeout;
	uint8_t *data;
	uint32_t packet_len;
	int ret;

	hdr.preamble = RPMSG_PERF_PREAMBLE_SENDER_START;
	packet_len = hdr.packet_size;
	eptdev->statistic.send_packet_cnt = 0;

	ret = rpmsg_sendto(eptdev->ept, &hdr, sizeof(hdr), eptdev->chinfo.dst);
	if (ret) {
		pr_err("failed to send packet header\n");
		return ret;
	}

	/* Prepare data packets */
	data = kmalloc(packet_len, GFP_KERNEL);
	if (data == NULL) {
		pr_err("allocate data buffer failure\n");
		return -ENOMEM;
	}
	memset(data, 0, packet_len);

	udelay(100);
	timeout = jiffies + msecs_to_jiffies(hdr.test_time * 1000);

	do {
		do {
			ret = rpmsg_trysendto(eptdev->ept, data, packet_len,
					      eptdev->chinfo.dst);
		} while (ret != 0);
		eptdev->statistic.send_packet_cnt++;
	} while (time_before(jiffies, timeout));

	memset(&hdr, 0, sizeof(hdr));
	hdr.preamble = RPMSG_PERF_PREAMBLE_SENDER_END;

	ret = rpmsg_sendto(eptdev->ept, &hdr, sizeof(hdr), eptdev->chinfo.dst);
	if (ret) {
		pr_err("failed to send RPMSG_PERF_PREAMBLE_SENDER_END packet\n");
	}

	kfree(data);

	return ret;
}

static long rpmsg_perf_eptdev_ioctl(struct file *fp, unsigned int cmd,
				    unsigned long arg)
{
	struct rpmsg_eptdev *eptdev = fp->private_data;
	struct test_statistic *statistic = &eptdev->statistic;
	struct task_struct *sender_thread_h;
	struct packet_header hdr = {0};
	uint32_t packet_cnt;
	uint32_t rate;
	uint32_t status;
	long ret;

	switch (cmd) {
	case RPMSG_PERF_GET_RUNNING_STA_IOCTL:
		status = eptdev->state;
		if (copy_to_user((char __user *)arg, &status, sizeof(status))) {
			pr_err("copy_to_user() failed\n");
			return -EFAULT;
		}
		break;
	case RPMSG_PERF_AS_SENDER_IOCTL:
		if (eptdev->state != RPMSG_DEV_IDLE)
			return -EINVAL;

		eptdev->state =  RPMSG_DEV_SENDING;
		ret = copy_from_user((void *)&hdr, (const void __user *)arg,
				     sizeof(struct packet_header));
		if (ret) {
			pr_err("copy_from_user() failed\n");
			return -EFAULT;
		}

		eptdev->param.no_copy = hdr.no_copy;
		eptdev->param.packet_size = hdr.packet_size;
		eptdev->param.test_time = hdr.test_time;
		statistic->packet_size = hdr.packet_size;
		statistic->test_time = hdr.test_time;

		sender_thread_h = kthread_run(rpmsg_perf_sender_thread,
					      eptdev, "rpmsg_sender");
		if (IS_ERR(sender_thread_h)) {
			pr_err("failed to create sender thread\n");
			return -EFAULT;
		}
		break;
	case RPMSG_PERF_AS_RECEIVER_IOCTL:
		if (eptdev->state != RPMSG_DEV_IDLE)
			return -EINVAL;

		eptdev->state = RPMSG_DEV_RECEIVING;
		ret = copy_from_user((void *)&hdr, (const void __user *)arg,
				     sizeof(struct packet_header));
		if (ret) {
			pr_err("copy_from_user() failed\n");
			return -EFAULT;
		}
		hdr.preamble = RPMSG_PERF_PREAMBLE_RECEIVER_START;
		statistic->test_time = hdr.test_time;
		statistic->packet_size = hdr.packet_size;
		statistic->recv_packet_cnt = 0;

		ret = rpmsg_sendto(eptdev->ept, &hdr, sizeof(hdr),
				   eptdev->chinfo.dst);
		if (ret)
			pr_err("failed to send RPMSG_PERF_PREAMBLE_RECEIVER_START packet\n");
		break;
	case RPMSG_PERF_AS_RECEIVER_END_ACK_IOCTL:
		if (eptdev->state !=  RPMSG_DEV_RECEIVING)
			return -EINVAL;

		packet_cnt = statistic->recv_packet_cnt - 1;
		rate = packet_cnt / statistic->test_time / 1000;
		pr_info("packet size: %u, received packets: %u, time: %u s, rate: %u kpps\n",
			statistic->packet_size, packet_cnt,
			statistic->test_time, rate);

		hdr.preamble = RPMSG_PERF_PREAMBLE_RECEIVER_END_ACK;
		hdr.packet_cnt = packet_cnt;
		ret = rpmsg_sendto(eptdev->ept, &hdr, sizeof(hdr),
				   eptdev->chinfo.dst);
		if (ret) {
			pr_err("failed to send RPMSG_PERF_PREAMBLE_RECEIVER_END_ACK packet\n");
			return ret;
		}
		eptdev->state = RPMSG_DEV_IDLE;
		statistic->test_time = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations rpmsg_perf_eptdev_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_perf_eptdev_open,
	.release = rpmsg_perf_eptdev_release,
	.unlocked_ioctl = rpmsg_perf_eptdev_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct rpmsg_eptdev *eptdev = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", eptdev->chinfo.name);
}
static DEVICE_ATTR_RO(name);

static ssize_t src_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct rpmsg_eptdev *eptdev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", eptdev->chinfo.src);
}
static DEVICE_ATTR_RO(src);

static ssize_t dst_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct rpmsg_eptdev *eptdev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", eptdev->chinfo.dst);
}
static DEVICE_ATTR_RO(dst);

static struct attribute *rpmsg_perf_eptdev_attrs[] = {
	&dev_attr_name.attr,
	&dev_attr_src.attr,
	&dev_attr_dst.attr,
	NULL
};
ATTRIBUTE_GROUPS(rpmsg_perf_eptdev);

static void rpmsg_eptdev_release_device(struct device *dev)
{
	struct rpmsg_eptdev *eptdev = dev_to_eptdev(dev);

	ida_simple_remove(&rpmsg_perf_minor_ida, MINOR(eptdev->dev.devt));
	kfree(eptdev);
}

static struct rpmsg_eptdev *rpmsg_perf_eptdev_alloc(struct rpmsg_device *rpdev,
						    struct device *parent)
{
	struct rpmsg_eptdev *eptdev;
	struct device *dev;

	eptdev = kzalloc(sizeof(*eptdev), GFP_KERNEL);
	if (!eptdev)
		return ERR_PTR(-ENOMEM);

	eptdev->state = RPMSG_DEV_IDLE;

	dev = &eptdev->dev;
	eptdev->rpdev = rpdev;

	mutex_init(&eptdev->ept_lock);

	device_initialize(dev);
	dev->class = &rpmsg_class;
	dev->parent = parent;
	dev->groups = rpmsg_perf_eptdev_groups;
	dev_set_drvdata(dev, eptdev);

	cdev_init(&eptdev->cdev, &rpmsg_perf_eptdev_fops);
	eptdev->cdev.owner = THIS_MODULE;

	return eptdev;
}

static int rpmsg_perf_eptdev_add(struct rpmsg_eptdev *eptdev,
				 struct rpmsg_channel_info chinfo)
{
	struct device *dev = &eptdev->dev;
	int dst = eptdev->rpdev->dst;
	int ret;

	eptdev->chinfo = chinfo;

	ret = ida_simple_get(&rpmsg_perf_minor_ida, 0,
			     RPMSG_DEV_MAX, GFP_KERNEL);
	if (ret < 0)
		goto free_eptdev;
	dev->devt = MKDEV(MAJOR(rpmsg_major), ret);

	dev->id = dst;
	dev_set_name(dev, "rpmsg-perf%d", dst);

	ret = cdev_device_add(&eptdev->cdev, &eptdev->dev);
	if (ret)
		goto free_minor_ida;

	/* We can now rely on the release function for cleanup */
	dev->release = rpmsg_eptdev_release_device;

	return ret;

free_minor_ida:
	ida_simple_remove(&rpmsg_perf_minor_ida, MINOR(dev->devt));
free_eptdev:
	put_device(dev);
	kfree(eptdev);

	return ret;
}

static int rpmsg_perf_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_channel_info chinfo;
	struct rpmsg_eptdev *eptdev;
	struct device *dev = &rpdev->dev;

	memcpy(chinfo.name, rpdev->id.name, RPMSG_NAME_SIZE);
	chinfo.src = rpdev->src;
	chinfo.dst = rpdev->dst;

	eptdev = rpmsg_perf_eptdev_alloc(rpdev, dev);
	if (IS_ERR(eptdev))
		return PTR_ERR(eptdev);

	/* Set the default_ept to the rpmsg device endpoint */
	eptdev->default_ept = rpdev->ept;

	/*
	 * The rpmsg_perf_ept_cb uses *priv parameter to get its
	 * rpmsg_eptdev context. Stored it in default_ept *priv field.
	 */
	eptdev->default_ept->priv = eptdev;

	return rpmsg_perf_eptdev_add(eptdev, chinfo);
}

static int rpmsg_perf_eptdev_chrdev_destroy(struct device *dev, void *data)
{
	struct rpmsg_eptdev *eptdev = dev_to_eptdev(dev);

	mutex_lock(&eptdev->ept_lock);
	if (eptdev->ept) {
		/* The default endpoint is released by the rpmsg core */
		if (!eptdev->default_ept)
			rpmsg_destroy_ept(eptdev->ept);
		eptdev->ept = NULL;
	}
	mutex_unlock(&eptdev->ept_lock);

	cdev_device_del(&eptdev->cdev, &eptdev->dev);
	put_device(&eptdev->dev);

	return 0;
}

static void rpmsg_perf_remove(struct rpmsg_device *rpdev)
{
	int ret;

	ret = device_for_each_child(&rpdev->dev, NULL,
				    rpmsg_perf_eptdev_chrdev_destroy);
	if (ret)
		dev_warn(&rpdev->dev, "failed to destroy endpoints: %d\n", ret);
}

static struct rpmsg_device_id rpmsg_perf_id_table[] = {
	{ .name	= "rpmsg-perf" },
	{ },
};

static struct rpmsg_driver rpmsg_perf_driver = {
	.probe = rpmsg_perf_probe,
	.remove = rpmsg_perf_remove,
	.callback = rpmsg_perf_ept_cb,
	.id_table = rpmsg_perf_id_table,
	.drv.name = "rpmsg_perf",
};

static int rpmsg_perf_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&rpmsg_major, 0, RPMSG_DEV_MAX, "rpmsg_perf");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		return ret;
	}

	ret = register_rpmsg_driver(&rpmsg_perf_driver);
	if (ret < 0) {
		pr_err("rpmsg: failed to register rpmsg raw driver\n");
		goto free_region;
	}

	return 0;

free_region:
	unregister_chrdev_region(rpmsg_major, RPMSG_DEV_MAX);

	return ret;
}
postcore_initcall(rpmsg_perf_init);

static void rpmsg_perf_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_perf_driver);
	unregister_chrdev_region(rpmsg_major, RPMSG_DEV_MAX);
}
module_exit(rpmsg_perf_exit);

MODULE_ALIAS("rpmsg:rpmsg_perf");
MODULE_LICENSE("GPL v2");
