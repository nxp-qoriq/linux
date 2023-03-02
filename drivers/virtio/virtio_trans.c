// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2022-2023 NXP
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/virtio_config.h>
#include <uapi/linux/virtio_ids.h>
#include <uapi/linux/virtio_trans.h>

#define MAX_TEST_LEN	2048
#define VT_TIMES	1000
#define INDIRECT_NUM	1
#define WAIT_TO_MS	2000

#define VT_TXRX_BIT	(1 << 0)
#define VT_B_COPY_BIT	(1 << 1)
#define VT_F_COPY_BIT	(1 << 2)
#define VT_B_MODE_BIT	(1 << 3)
#define VT_F_MODE_BIT	(1 << 4)

static u32 poll_delay = 10;

struct virtio_trans {
	/* The virtio device we're associated with */
	struct virtio_device *vdev;
	struct device *dev;
	wait_queue_head_t waitqueue;
	struct work_struct config_work;

	struct virtqueue *tx_vq, *rx_vq;
	void *tx_buf, *rx_buf;
	spinlock_t txvq_lock;
	spinlock_t rxvq_lock;
	struct scatterlist *tx_sgl;
	struct scatterlist *rx_sgl;
	size_t tx_vring_size;
	size_t rx_vring_size;

	bool vt_running;
	u32 regression;
	size_t pkt_size;
	bool tx;
	bool do_copy;
	bool poll_mode;
	bool back_poll_mode;
	void *data_buf;
};

static void tx_intr(struct virtqueue *vq)
{
	struct virtio_trans *vt = vq->vdev->priv;

	if (vt->poll_mode)
		return;

	if (!vt->vt_running)
		return;

	if (!vt->tx)
		return;

	wake_up_interruptible(&vt->waitqueue);
}

static void virttrans_init_sg(struct scatterlist *sg, size_t sg_num, void *base, size_t len)
{
	int i;

	sg_init_table(sg, sg_num);
	if (likely(is_vmalloc_addr(base)))
		for (i = 0; i < sg_num; i++)
			sg_set_page(&sg[i], vmalloc_to_page(base + i * len), len,
					offset_in_page(base));
	else
		for (i = 0; i < sg_num; i++)
			sg_set_buf(&sg[i], base + i * len, len);
}

/* call this func with lock */
static void virttrans_queue_txbuf(struct virtio_trans *vt,
				  struct scatterlist *sg,
				  void *buf, size_t pkt_size)
{
	if (vt->do_copy)
		memcpy(buf, vt->data_buf, pkt_size);
	virtqueue_add_outbuf(vt->tx_vq, sg, INDIRECT_NUM, buf, GFP_ATOMIC);
}

static void virttrans_reclaim_txvq(struct virtio_trans *vt)
{
	void *buf;
	u32 len;

	spin_lock(&vt->txvq_lock);
	while ((buf = virtqueue_get_buf(vt->tx_vq, &len)) != NULL)
		;
	spin_unlock(&vt->txvq_lock);

}

static void virttrans_fill_txvq(struct virtio_trans *vt, size_t pkt_size)
{
	int i;

	spin_lock(&vt->txvq_lock);
	for (i = 0; i < vt->tx_vring_size; i++) {
		virttrans_init_sg(&vt->tx_sgl[i * INDIRECT_NUM], INDIRECT_NUM,
				  vt->tx_buf + i * MAX_TEST_LEN * INDIRECT_NUM,
				  pkt_size);
		virttrans_queue_txbuf(vt, &vt->tx_sgl[i * INDIRECT_NUM],
				      vt->tx_buf + i * MAX_TEST_LEN *
				      INDIRECT_NUM, pkt_size);
	}
	spin_unlock(&vt->txvq_lock);
}

static void *vt_get_tx_buf(struct virtio_trans *vt)
{
	void *buf;
	int len;

	spin_lock(&vt->txvq_lock);
	buf = virtqueue_get_buf(vt->tx_vq, &len);
	spin_unlock(&vt->txvq_lock);

	return buf;
}
static int tx_pkts(struct virtio_trans *vt, size_t pkt_size, u32 times)
{

	ktime_t start_time, stop_time;
	u32 cnt = times;
	u64 time_period;
	u32 tx_count;
	int err;
	int i;
	u64 idx = 0;
	void *buf;

	if (pkt_size > MAX_TEST_LEN) {
		dev_err(vt->dev, "pkt_size must be less than 0x%x\n",
			MAX_TEST_LEN);
		return -EINVAL;
	}

	start_time = ktime_get();

	while (vt->vt_running && cnt > 0) {
		buf = vt_get_tx_buf(vt);
		while (!buf) {
			if (!vt->back_poll_mode)
				virtqueue_kick(vt->tx_vq);

			if (vt->poll_mode) {
				udelay(poll_delay);
				buf = vt_get_tx_buf(vt);
				continue;
			}

			err = wait_event_interruptible_timeout(vt->waitqueue,
						(buf = vt_get_tx_buf(vt)),
						msecs_to_jiffies(WAIT_TO_MS));
			if (err == -ERESTARTSYS) {
				dev_info(vt->dev, "%s: interrupt the waiting for tx buffer by signal\n",
					 __func__);
				goto out;
			}
		}

		spin_lock(&vt->txvq_lock);
		virttrans_queue_txbuf(vt, &vt->tx_sgl[idx % vt->tx_vring_size],
				      buf, vt->pkt_size);
		idx++;
		spin_unlock(&vt->txvq_lock);

		cnt -= INDIRECT_NUM;
	}

out:
	i = 100;
	while (i--) {
		virtqueue_kick(vt->tx_vq);
		virtio_cread_le(vt->vdev, struct virtio_trans_config,
				tx_count, &tx_count);
		if (tx_count >= times - cnt)
			break;
		mdelay(10);
	}

	if (i < 0)
		dev_err(vt->dev, "Wait backend completion timeout\n");

	stop_time = ktime_get();
	time_period =  ktime_us_delta(stop_time, start_time);

	pr_info("tx_test: pkt_size (%zu B), pkt_cnt (%d), period (%llu us)\n",
		pkt_size, times - cnt, time_period);

	return err;
}

/* call this func with lock */
static void virttrans_queue_rxbuf(struct virtio_trans *vt,
				  struct scatterlist *sg,
				  void *buf, size_t pkt_size)
{
	if (vt->do_copy)
		memcpy(vt->data_buf, buf, pkt_size);
	virtqueue_add_inbuf(vt->rx_vq, sg, INDIRECT_NUM, buf, GFP_ATOMIC);
}

static void virttrans_reclaim_rxvq(struct virtio_trans *vt)
{
	void *buf;
	u32 len;

	spin_lock(&vt->rxvq_lock);
	while ((buf = virtqueue_get_buf(vt->rx_vq, &len)) != NULL)
		;
	spin_unlock(&vt->rxvq_lock);

}

static void virttrans_fill_rxvq(struct virtio_trans *vt, size_t pkt_size)
{
	int i;

	spin_lock(&vt->rxvq_lock);
	for (i = 0; i < vt->rx_vring_size; i++) {
		virttrans_init_sg(&vt->rx_sgl[i * INDIRECT_NUM], INDIRECT_NUM,
				  vt->rx_buf + i * MAX_TEST_LEN * INDIRECT_NUM,
				  pkt_size);
		virttrans_queue_rxbuf(vt, &vt->rx_sgl[i * INDIRECT_NUM],
				      vt->rx_buf + i * MAX_TEST_LEN *
				      INDIRECT_NUM, pkt_size);
	}
	spin_unlock(&vt->rxvq_lock);
}

static void rx_intr(struct virtqueue *vq)
{
	struct virtio_trans *vt = vq->vdev->priv;

	if (vt->poll_mode)
		return;

	if (!vt->vt_running)
		return;

	if (vt->tx)
		return;

	wake_up_interruptible(&vt->waitqueue);
}

static void *vt_get_rx_buf(struct virtio_trans *vt)
{
	void *buf;
	int len;

	spin_lock(&vt->rxvq_lock);
	buf = virtqueue_get_buf(vt->rx_vq, &len);
	spin_unlock(&vt->rxvq_lock);

	return buf;
}

static int rx_pkts(struct virtio_trans *vt, size_t pkt_size, u32 times)
{

	ktime_t start_time, stop_time;
	u32 cnt = times;
	u64 time_period;
	int err;
	void *buf;
	u64 idx = 0;

	if (pkt_size > MAX_TEST_LEN) {
		dev_err(vt->dev, "pkt_size must be less than 0x%x\n",
			MAX_TEST_LEN);
		return -EINVAL;
	}

	start_time = ktime_get();
	while (vt->vt_running && cnt > 0) {
		buf = vt_get_rx_buf(vt);
		while (!buf) {
			if (!vt->back_poll_mode)
				virtqueue_kick(vt->rx_vq);

			if (vt->poll_mode) {
				udelay(poll_delay);
				buf = vt_get_rx_buf(vt);
				continue;
			}

			err = wait_event_interruptible_timeout(vt->waitqueue,
						(buf = vt_get_rx_buf(vt)),
						msecs_to_jiffies(WAIT_TO_MS));
			if (err == -ERESTARTSYS) {
				dev_info(vt->dev, "%s: interrupt the waiting for rx buffer by signal\n",
					 __func__);
				goto out;
			}
		}

		spin_lock(&vt->rxvq_lock);
		virttrans_queue_rxbuf(vt, &vt->rx_sgl[idx % vt->rx_vring_size],
				      buf, vt->pkt_size);
		idx++;
		spin_unlock(&vt->rxvq_lock);

		cnt -= INDIRECT_NUM;
	}

out:
	stop_time = ktime_get();
	time_period =  ktime_us_delta(stop_time, start_time);

	pr_info("rx_test: pkt_size (%zu B), pkt_cnt (%d), period (%llu us)\n",
		pkt_size, times - cnt, time_period);

	return 0;
}

static ssize_t vt_control_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 control;

	virtio_cread_le(vt->vdev, struct virtio_trans_config, control,
			&control);

	return sprintf(buf, "0: stop\n1: start\ncontrol = %u\n", control);
}

static ssize_t vt_control_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t cnt)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 control;
	unsigned long val;

	if (kstrtoul(buf, 0, &val) < 0 || (val > 1)) {
		dev_err(vt->dev, "Invalid param\n");
		pr_info("0: stop\n");
		pr_info("1: start\n");
		return -EINVAL;
	}

	virtio_cread_le(vt->vdev, struct virtio_trans_config, control,
			&control);

	if (!val) {
		control	&= ~VT_CTRL_START;
		virtio_cwrite_le(vt->vdev, struct virtio_trans_config, control,
				 &control);
		vt->vt_running = 0;
		if (!vt->tx)
			virttrans_reclaim_rxvq(vt);
		else
			virttrans_reclaim_txvq(vt);

		vt->regression = 0;
		return cnt;
	}

	if (vt->vt_running) {
		dev_err(vt->dev, "Try again after this test completed\n");
		return -EAGAIN;
	}

	vt->vt_running = 1;
	control	|= VT_CTRL_START;

	if (!vt->tx) {
		virttrans_fill_rxvq(vt, vt->pkt_size);
		virtio_cwrite_le(vt->vdev, struct virtio_trans_config, control,
				 &control);
		rx_pkts(vt, vt->pkt_size, vt->regression);
	} else {
		virttrans_fill_txvq(vt, vt->pkt_size);
		virtio_cwrite_le(vt->vdev, struct virtio_trans_config, control,
				 &control);
		tx_pkts(vt, vt->pkt_size, vt->regression);
	}

	control	&= ~VT_CTRL_START;
	virtio_cwrite_le(vt->vdev, struct virtio_trans_config, control,
			 &control);

	vt->vt_running = 0;
	vt->regression = 0;

	return cnt;
}

static ssize_t vt_config_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t cnt)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 config;
	unsigned long val;

	if (vt->vt_running) {
		dev_err(dev, "Try again after this test completed\n");
		return -EAGAIN;
	}

	if (kstrtoul(buf, 0, &val) < 0 || (val & ~0x1f)) {
		dev_err(dev, "Invalid param\n");
		pr_info("bit[0]:\n");
		pr_info("\t0: TX\n");
		pr_info("\t1: RX\n");
		pr_info("bit[1]:\n");
		pr_info("\t0: Backend do NOT copy buffer\n");
		pr_info("\t1: Backend do copy buffer\n");
		pr_info("bit[2]:\n");
		pr_info("\t0: Frontend do NOT copy buffer\n");
		pr_info("\t1: Frontend do copy buffer\n");
		pr_info("bit[3]:\n");
		pr_info("\t0: Backend interrupt mode\n");
		pr_info("\t1: Backend polling mode\n");
		pr_info("bit[4]:\n");
		pr_info("\t0: Frontend interrupt mode\n");
		pr_info("\t1: Frontend polling mode\n");
		return -EINVAL;
	}

	if (!vt->pkt_size)
		vt->pkt_size = 64;

	if (!vt->regression) {
		virtio_cwrite_le(vt->vdev, struct virtio_trans_config,
				 regression, &vt->regression);
		vt->regression = VT_TIMES;
	}

	virtio_cread_le(vt->vdev, struct virtio_trans_config, config, &config);

	config &= ~(VT_CFG_TX | VT_CFG_RX | VT_CFG_COPY |
		    VT_CFG_B_POLL | VT_CFG_F_POLL);
	vt->do_copy = false;
	vt->back_poll_mode = false;
	vt->poll_mode = false;

	if (val & VT_TXRX_BIT) {
		vt->tx = false;
		config |= VT_CFG_RX;
	} else {
		vt->tx = true;
		config |= VT_CFG_TX;
	}

	if (val & VT_B_COPY_BIT)
		config |= VT_CFG_COPY;

	if (val & VT_F_COPY_BIT)
		vt->do_copy = true;

	if (val & VT_B_MODE_BIT) {
		vt->back_poll_mode = true;
		config |= VT_CFG_B_POLL;
	}

	if (val &  VT_F_MODE_BIT) {
		vt->poll_mode = true;
		config |= VT_CFG_F_POLL;
	}

	pr_info("*********************************************************\n");
	pr_info("Front-end: %s mode\n", vt->poll_mode ? "poll" : "interrupt");
	pr_info("Back-end:  %s mode\n", vt->back_poll_mode ? "poll" : "interrupt");
	pr_info("Front-end: do %scopy buffer\n", vt->do_copy ? "" : "NOT ");
	pr_info("Back-end:  do %scopy buffer\n", (config & VT_CFG_COPY) ? "" : "NOT");

	pr_info("\tTest case: %s\n\tpkt_size: <%zu>\n\tregress times: <%d>\n\n",
		vt->tx ? "TX" : "RX", vt->pkt_size, vt->regression);

	virtio_cwrite_le(vt->vdev, struct virtio_trans_config, config, &config);

	return cnt;
}
static ssize_t vt_config_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 config;

	virtio_cread_le(vt->vdev, struct virtio_trans_config, config, &config);

	return sprintf(buf, "0x%x\n", config);
}

static ssize_t vt_pkt_size_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t cnt)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 val;

	if (vt->vt_running) {
		dev_err(dev, "Try again after this test completed\n");
		return -EAGAIN;
	}

	if (kstrtou32(buf, 0, &val) < 0 || (val > MAX_TEST_LEN)) {
		dev_err(dev, "pkt_size must be less then 0x%x\n", MAX_TEST_LEN);
		return -EINVAL;
	}

	vt->pkt_size = val;
	virtio_cwrite_le(vt->vdev, struct virtio_trans_config, pkt_size, &val);

	dev_dbg(dev, "Update pkt_size to %u\n", val);

	return cnt;
}
static ssize_t vt_pkt_size_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);

	return sprintf(buf, "%ld\n", vt->pkt_size);
}

static ssize_t vt_regression_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t cnt)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);
	u32 val;

	if (vt->vt_running) {
		dev_err(dev, "Try again after this test completed\n");
		return -EAGAIN;
	}

	if (kstrtou32(buf, 0, &val) < 0 || val <= 0) {
		dev_err(dev, "Invalid regression\n");
		return -EINVAL;
	}

	vt->regression = val;

	virtio_cwrite_le(vt->vdev, struct virtio_trans_config, regression,
			 &val);
	dev_dbg(dev, "Update regression to %u\n", val);

	return cnt;
}
static ssize_t vt_regression_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct virtio_trans *vt = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", vt->regression);
}

static DEVICE_ATTR_RW(vt_control);
static DEVICE_ATTR_RW(vt_config);
static DEVICE_ATTR_RW(vt_pkt_size);
static DEVICE_ATTR_RW(vt_regression);

static struct attribute *sysfs_entries[] = {
	&dev_attr_vt_config.attr,
	&dev_attr_vt_control.attr,
	&dev_attr_vt_pkt_size.attr,
	&dev_attr_vt_regression.attr,
	NULL
};

static const struct attribute_group vt_attribute_group = {
	.name = NULL,		/* put in device directory */
	.attrs = sysfs_entries,
};

static void config_intr(struct virtio_device *vdev)
{
	struct virtio_trans *vt = vdev->priv;

	schedule_work(&vt->config_work);
}

static void config_work_handler(struct work_struct *work)
{
	/* handle the config change */
}

static int virttrans_init_vqs(struct virtio_trans *vt)
{
	struct virtqueue *vqs[2];
	vq_callback_t *cbs[] = { tx_intr, rx_intr};
	static const char * const names[] = { "tx", "rx" };
	int err;

	err = virtio_find_vqs(vt->vdev, 2, vqs, cbs, names, NULL);
	if (err) {
		dev_err(vt->dev, "Failed to find vqs\n");
		return err;
	}
	vt->tx_vq = vqs[0];
	vt->rx_vq = vqs[1];

	return 0;
}

static int virttrans_probe(struct virtio_device *vdev)
{
	struct virtio_trans *vt;
	int err;
	dma_addr_t tx_buf_dma;

	vt = kzalloc(sizeof(*vt), GFP_KERNEL);
	if (!vt)
		return -ENOMEM;

	vdev->priv = vt;
	vt->vdev = vdev;
	vt->dev = &vdev->dev;
	vt->pkt_size = 64;
	vt->regression = VT_TIMES;

	dev_set_drvdata(vt->dev, vt);

	err = virttrans_init_vqs(vt);
	if (err) {
		dev_err(vt->dev, "Failed to init virtqueues\n");
		goto err_init_vq;
	}

	vt->tx_vring_size = virtqueue_get_vring_size(vt->tx_vq);
	vt->rx_vring_size = virtqueue_get_vring_size(vt->rx_vq);

	vt->data_buf = kzalloc(MAX_TEST_LEN, GFP_KERNEL);
	if (!vt->data_buf) {
		err = -ENOMEM;
		dev_err(vt->dev, "Failed to alloc data buffer\n");
		goto alloc_data_buf;
	}

	memset(vt->data_buf, 0xa5, MAX_TEST_LEN);

	vt->tx_sgl = kzalloc(INDIRECT_NUM * (vt->rx_vring_size +
			     vt->tx_vring_size) * sizeof(struct scatterlist),
			     GFP_KERNEL);

	if (!vt->tx_sgl) {
		err = -ENOMEM;
		dev_err(vt->dev, "Failed to alloc SG descriptors\n");
		goto alloc_sgl;
	}

	vt->rx_sgl = vt->tx_sgl + INDIRECT_NUM * vt->tx_vring_size;

	vt->tx_buf = dma_alloc_coherent(vdev->dev.parent, MAX_TEST_LEN *
					INDIRECT_NUM * (vt->rx_vring_size +
					vt->tx_vring_size), &tx_buf_dma,
					GFP_KERNEL);
	if (!vt->tx_buf) {
		err = -ENOMEM;
		dev_err(vt->dev, "Failed to alloc Vring buffers\n");
		goto alloc_buf;
	}

	vt->rx_buf = vt->tx_buf +
		     MAX_TEST_LEN * INDIRECT_NUM * vt->tx_vring_size;

	spin_lock_init(&vt->txvq_lock);
	spin_lock_init(&vt->rxvq_lock);
	init_waitqueue_head(&vt->waitqueue);
	INIT_WORK(&vt->config_work, &config_work_handler);

	err = sysfs_create_group(&vt->dev->kobj, &vt_attribute_group);
	if (err) {
		dev_err(vt->dev, "Failed to create sysfs device attributes\n");
		goto sysfs;
	} else {
		/*
		 * Generate a udev event so that appropriate
		 * symlinks can be created based on udev
		 * rules.
		 */
		kobject_uevent(&vt->dev->kobj, KOBJ_CHANGE);
	}

	virtio_device_ready(vt->vdev);

	return 0;

sysfs:
	dma_free_coherent(vdev->dev.parent, MAX_TEST_LEN * INDIRECT_NUM *
			  (vt->rx_vring_size + vt->tx_vring_size),
			  vt->tx_buf, tx_buf_dma);
alloc_buf:
	kfree(vt->tx_sgl);
alloc_sgl:
	kfree(vt->data_buf);
alloc_data_buf:
	/* TODO: vtrans_deinit_vq(); */
err_init_vq:
	kfree(vt);

	return err;
}

static void virttrans_remove(struct virtio_device *vdev)
{
	struct virtio_trans *vt = vdev->priv;
	void *buf;

	virtio_break_device(vdev);
	flush_work(&vt->config_work);
	vdev->config->reset(vdev);
	cancel_work_sync(&vt->config_work);

	sysfs_remove_group(&vt->dev->kobj, &vt_attribute_group);

	while ((buf = virtqueue_detach_unused_buf(vt->tx_vq)) != NULL)
		kfree(buf);
	vdev->config->del_vqs(vdev);

	kfree(vt);
}

static const struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_TRANS, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static const unsigned int features[] = {
	/* none */
};


static struct virtio_driver virtio_transfer = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table = id_table,
	.probe = virttrans_probe,
	.remove = virttrans_remove,
	.config_changed = config_intr,
};

static int __init virtio_transfer_init(void)
{
	int err;

	err = register_virtio_driver(&virtio_transfer);
	if (err < 0) {
		pr_err("Failed to register virtio driver\n");
		return err;
	}
	return 0;
}

static void __exit virtio_transfer_exit(void)
{
	unregister_virtio_driver(&virtio_transfer);
}

module_init(virtio_transfer_init);
module_exit(virtio_transfer_exit);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio transfer test driver");
MODULE_LICENSE("GPL");
