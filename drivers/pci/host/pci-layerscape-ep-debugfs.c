/*
 * PCIe Endpoint driver for Freescale Layerscape SoCs
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
  * Author: Minghuan Lian <Minghuan.Lian@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>

#include <linux/completion.h>

#include "pci-layerscape-ep.h"

#define PCIE_ATU_INDEX3		(0x3 << 0)
#define PCIE_ATU_INDEX2		(0x2 << 0)
#define PCIE_ATU_INDEX1		(0x1 << 0)
#define PCIE_ATU_INDEX0		(0x0 << 0)

#define PCIE_BAR0_SIZE		(4 * 1024) /* 4K */
#define PCIE_BAR1_SIZE		(8 * 1024) /* 8K for MSIX */
#define PCIE_BAR2_SIZE		(4 * 1024) /* 4K */
#define PCIE_BAR4_SIZE		(1 * 1024 * 1024) /* 1M */
#define PCIE_MSI_OB_SIZE	(4 * 1024) /* 4K */

#define PCIE_MSI_MSG_ADDR_OFF	0x54
#define PCIE_MSI_MSG_DATA_OFF	0x5c

enum test_type {
	TEST_TYPE_DMA,
	TEST_TYPE_MEMCPY
};

enum test_dirt {
	TEST_DIRT_READ,
	TEST_DIRT_WRITE
};

enum test_status {
	TEST_IDLE,
	TEST_BUSY
};

struct ls_ep_test {
	struct ls_ep_dev	*ep;
	void __iomem		*cfg;
	void __iomem		*buf;
	void __iomem		*out;
	void __iomem		*msi;
	dma_addr_t		cfg_addr;
	dma_addr_t		buf_addr;
	dma_addr_t		out_addr;
	dma_addr_t		bus_addr;
	dma_addr_t		msi_addr;
	u64			msi_msg_addr;
	u16			msi_msg_data;
	struct task_struct	*thread;
	spinlock_t		lock;
	struct completion	done;
	u32			len;
	int			loop;
	char			data;
	enum test_dirt		dirt;
	enum test_type		type;
	enum test_status	status;
	u64			result; /* Mbps */
	char			cmd[256];
};

static int ls_pcie_ep_trigger_msi(struct ls_ep_test *test)
{
	if (!test->msi)
		return -EINVAL;

	iowrite32(test->msi_msg_data, test->msi);

	return 0;
}

static int ls_pcie_ep_test_try_run(struct ls_ep_test *test)
{
	int ret;

	spin_lock(&test->lock);
	if (test->status == TEST_IDLE) {
		test->status = TEST_BUSY;
		ret = 0;
	} else
		ret = -EBUSY;
	spin_unlock(&test->lock);

	return ret;
}

static void ls_pcie_ep_test_done(struct ls_ep_test *test)
{
	spin_lock(&test->lock);
	test->status = TEST_IDLE;
	spin_unlock(&test->lock);
}

static void ls_pcie_ep_test_dma_cb(void *arg)
{
	struct ls_ep_test *test = arg;

	complete(&test->done);
}

static int ls_pcie_ep_test_dma(struct ls_ep_test *test)
{
	dma_cap_mask_t mask;
	struct dma_chan *chan;
	struct dma_device *dma_dev;
	dma_addr_t src, dst;
	enum dma_data_direction direction;
	enum dma_ctrl_flags dma_flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct timespec start, end, period;
	int i = 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	chan = dma_request_channel(mask, NULL, test);
	if (!chan) {
		pr_err("failed to request dma channel\n");
		return -EINVAL;
	}

	memset(test->buf, test->data, test->len);

	if (test->dirt == TEST_DIRT_WRITE) {
		src = test->buf_addr;
		dst = test->out_addr;
		direction = DMA_TO_DEVICE;
	} else {
		src = test->out_addr;
		dst = test->buf_addr;
		direction = DMA_FROM_DEVICE;
	}

	dma_dev = chan->device;
	dma_flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;

	dma_sync_single_for_device(&test->ep->dev, test->buf_addr,
				   test->len, direction);

	set_freezable();

	getrawmonotonic(&start);
	while (!kthread_should_stop() && (i < test->loop)) {
		struct dma_async_tx_descriptor *dma_desc;
		dma_cookie_t	dma_cookie = {0};
		unsigned long tmo;
		int status;

		init_completion(&test->done);

		dma_desc = dma_dev->device_prep_dma_memcpy(chan,
							   dst, src,
							   test->len,
							   dma_flags);
		if (!dma_desc) {
			pr_err("DMA desc constr failed...\n");
			goto _err;
		}

		dma_desc->callback = ls_pcie_ep_test_dma_cb;
		dma_desc->callback_param = test;
		dma_cookie = dmaengine_submit(dma_desc);

		if (dma_submit_error(dma_cookie)) {
			pr_err("DMA submit error....\n");
			goto _err;
		}

		/* Trigger the transaction */
		dma_async_issue_pending(chan);

		tmo = wait_for_completion_timeout(&test->done,
					  msecs_to_jiffies(5 * test->len));
		if (tmo == 0) {
			pr_err("Self-test copy timed out, disabling\n");
			goto _err;
		}

		status = dma_async_is_tx_complete(chan, dma_cookie,
						  NULL, NULL);
		if (status != DMA_COMPLETE) {
			pr_err("got completion callback, but status is %s\n",
			       status == DMA_ERROR ? "error" : "in progress");
			goto _err;
		}

		i++;
	}

	getrawmonotonic(&end);
	period = timespec_sub(end, start);
	test->result = test->len * 8ULL * i * 1000;
	do_div(test->result, period.tv_sec * 1000 * 1000 * 1000 + period.tv_nsec);
	dma_release_channel(chan);

	return 0;

_err:
	dma_release_channel(chan);
	test->result = 0;
	return -EINVAL;
}

static int ls_pcie_ep_test_cpy(struct ls_ep_test *test)
{
	void *dst, *src;
	struct timespec start, end, period;
	int i = 0;

	memset(test->buf, test->data, test->len);

	if (test->dirt == TEST_DIRT_WRITE) {
		dst = test->out;
		src = test->buf;
	} else {
		dst = test->buf;
		src = test->out;
	}

	getrawmonotonic(&start);
	while (!kthread_should_stop() && i < test->loop) {
		memcpy(dst, src, test->len);
		i++;
	}
	getrawmonotonic(&end);

	period = timespec_sub(end, start);
	test->result = test->len * 8ULL * i * 1000;
	do_div(test->result, period.tv_sec * 1000 * 1000 * 1000 + period.tv_nsec);

	return 0;
}

int ls_pcie_ep_test_thread(void *arg)
{
	int ret;

	struct ls_ep_test *test = arg;

	if (test->type == TEST_TYPE_DMA)
		ret = ls_pcie_ep_test_dma(test);
	else
		ret = ls_pcie_ep_test_cpy(test);

	if (ret) {
		pr_err("\n%s \ttest failed\n",
		       test->cmd);
		test->result = 0;
	} else
		pr_err("\n%s \tthroughput:%lluMbps\n",
		       test->cmd, test->result);

	ls_pcie_ep_test_done(test);

	ls_pcie_ep_trigger_msi(test);

	do_exit(0);
}

static int ls_pcie_ep_free_test(struct ls_ep_dev *ep)
{
	struct ls_ep_test *test = ep->driver_data;

	if (!test)
		return 0;

	if (test->status == TEST_BUSY) {
		kthread_stop(test->thread);
		dev_info(&ep->dev,
			 "test is running please wait and run again\n");
		return -EBUSY;
	}

	if (test->buf)
		free_pages((unsigned long)test->buf,
			   get_order(PCIE_BAR4_SIZE));

	if (test->cfg)
		free_pages((unsigned long)test->cfg,
			   get_order(PCIE_BAR2_SIZE));

	if (test->out)
		iounmap(test->out);

	kfree(test);
	ep->driver_data = NULL;

	return 0;
}

static int ls_pcie_ep_init_test(struct ls_ep_dev *ep, u64 bus_addr)
{
	struct ls_pcie *pcie = ep->pcie;
	struct ls_ep_test *test = ep->driver_data;
	int err;

	if (test) {
		dev_info(&ep->dev,
			 "Please use 'free' to remove the exiting test\n");
		return -EBUSY;
	}

	test = kzalloc(sizeof(*test), GFP_KERNEL);
	if (!test)
		return -ENOMEM;
	ep->driver_data = test;
	test->ep = ep;
	spin_lock_init(&test->lock);
	test->status = TEST_IDLE;

	test->buf = dma_alloc_coherent(pcie->dev, get_order(PCIE_BAR4_SIZE),
					&test->buf_addr,
					GFP_KERNEL);
	if (!test->buf) {
		dev_info(&ep->dev, "failed to get mem for bar4\n");
		err = -ENOMEM;
		goto _err;
	}

	test->cfg = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
					     get_order(PCIE_BAR2_SIZE));
	if (!test->cfg) {
		dev_info(&ep->dev, "failed to get mem for bar4\n");
		err = -ENOMEM;
		goto _err;
	}
	test->cfg_addr = virt_to_phys(test->cfg);

	test->out_addr = pcie->out_base;
	test->out = ioremap(test->out_addr, PCIE_BAR4_SIZE);
	if (!test->out) {
		dev_info(&ep->dev, "failed to map out\n");
		err = -ENOMEM;
		goto _err;
	}

	test->bus_addr = bus_addr;

	test->msi_addr = test->out_addr + PCIE_BAR4_SIZE;
	test->msi = ioremap(test->msi_addr, PCIE_MSI_OB_SIZE);
	if (!test->msi)
		dev_info(&ep->dev, "failed to map MSI outbound region\n");

	test->msi_msg_addr = ioread32(pcie->dbi + PCIE_MSI_MSG_ADDR_OFF) |
		(((u64)ioread32(pcie->dbi + PCIE_MSI_MSG_ADDR_OFF + 4)) << 32);
	test->msi_msg_data = ioread16(pcie->dbi + PCIE_MSI_MSG_DATA_OFF);

	ls_pcie_ep_dev_cfg_enable(ep);

	/* outbound iATU for memory */
	ls_pcie_iatu_outbound_set(pcie, 0, PCIE_ATU_TYPE_MEM,
				  test->out_addr, bus_addr, PCIE_BAR4_SIZE);
	/* outbound iATU for MSI */
	ls_pcie_iatu_outbound_set(pcie, 1, PCIE_ATU_TYPE_MEM,
				  test->msi_addr, test->msi_msg_addr,
				  PCIE_MSI_OB_SIZE);

	/* ATU 0 : INBOUND : map BAR0 */
	ls_pcie_iatu_inbound_set(pcie, 0, 0, test->cfg_addr);
	/* ATU 2 : INBOUND : map BAR2 */
	ls_pcie_iatu_inbound_set(pcie, 2, 2, test->cfg_addr);
	/* ATU 3 : INBOUND : map BAR4 */
	ls_pcie_iatu_inbound_set(pcie, 3, 4, test->buf_addr);

	return 0;

_err:
	ls_pcie_ep_free_test(ep);
	return err;
}

static int ls_pcie_ep_start_test(struct ls_ep_dev *ep, char *cmd)
{
	struct ls_ep_test *test = ep->driver_data;
	enum test_type type;
	enum test_dirt dirt;
	u32 cnt, len, loop;
	unsigned int data;
	char dirt_str[2];
	int ret;

	if (strncmp(cmd, "dma", 3) == 0)
		type = TEST_TYPE_DMA;
	else
		type = TEST_TYPE_MEMCPY;

	cnt = sscanf(&cmd[4], "%1s %u %u %x", dirt_str, &len, &loop, &data);
	if (cnt != 4) {
		dev_info(&ep->dev, "format error %s", cmd);
		dev_info(&ep->dev, "dma/cpy <r/w> <packet_size> <loop> <data>\n");
		return -EINVAL;
	}

	if (strncmp(dirt_str, "r", 1) == 0)
		dirt = TEST_DIRT_READ;
	else
		dirt = TEST_DIRT_WRITE;

	if (len > PCIE_BAR4_SIZE) {
		dev_err(&ep->dev, "max len is %d", PCIE_BAR4_SIZE);
		return -EINVAL;
	}

	if (!test) {
		dev_err(&ep->dev, "Please first run init command\n");
		return -EINVAL;
	}

	if (ls_pcie_ep_test_try_run(test)) {
		dev_err(&ep->dev, "There is already a test running\n");
		return -EINVAL;
	}

	test->len = len;
	test->loop = loop;
	test->type = type;
	test->data = (char)data;
	test->dirt = dirt;
	strcpy(test->cmd, cmd);
	test->thread = kthread_run(ls_pcie_ep_test_thread, test,
				   "pcie ep test");
	if (IS_ERR(test->thread)) {
		dev_err(&ep->dev, "fork failed for pcie ep test\n");
		ls_pcie_ep_test_done(test);
		ret = PTR_ERR(test->thread);
	}

	return ret;
}


/**
 * ls_pcie_reg_ops_read - read for regs data
 * @filp: the opened file
 * @buffer: where to write the data for the user to read
 * @count: the size of the user's buffer
 * @ppos: file position offset
 **/
static ssize_t ls_pcie_ep_dbg_regs_read(struct file *filp, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct ls_ep_dev *ep = filp->private_data;
	struct ls_pcie *pcie = ep->pcie;
	char *buf;
	int desc = 0, i, len;

	buf = kmalloc(4 * 1024, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ls_pcie_ep_dev_cfg_enable(ep);

	desc += sprintf(buf + desc, "%s", "reg info:");
	for (i = 0; i < 0x200; i += 4) {
		if (i % 16 == 0)
			desc += sprintf(buf + desc, "\n%08x:", i);
		desc += sprintf(buf + desc, " %08x", readl(pcie->dbi + i));
	}

	desc += sprintf(buf + desc, "\n%s", "outbound iATU info:\n");
	for (i = 0; i < 6; i++) {
		writel(PCIE_ATU_REGION_OUTBOUND | i,
		       pcie->dbi + PCIE_ATU_VIEWPORT);
		desc += sprintf(buf + desc, "iATU%d", i);
		desc += sprintf(buf + desc, "\tLOWER PHYS 0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LOWER_BASE));
		desc += sprintf(buf + desc, "\tUPPER PHYS 0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_UPPER_BASE));
		desc += sprintf(buf + desc, "\tLOWER BUS  0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LOWER_TARGET));
		desc += sprintf(buf + desc, "\tUPPER BUS  0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_UPPER_TARGET));
		desc += sprintf(buf + desc, "\tLIMIT      0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LIMIT));
		desc += sprintf(buf + desc, "\tCR1        0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_CR1));
		desc += sprintf(buf + desc, "\tCR2        0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_CR2));
	}

	desc += sprintf(buf + desc, "\n%s", "inbound iATU info:\n");
	for (i = 0; i < 6; i++) {
		writel(PCIE_ATU_REGION_INBOUND | i,
		       pcie->dbi + PCIE_ATU_VIEWPORT);
		desc += sprintf(buf + desc, "iATU%d", i);
		desc += sprintf(buf + desc, "\tLOWER BUS  0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LOWER_BASE));
		desc += sprintf(buf + desc, "\tUPPER BUSs 0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_UPPER_BASE));
		desc += sprintf(buf + desc, "\tLOWER PHYS 0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LOWER_TARGET));
		desc += sprintf(buf + desc, "\tUPPER PHYS 0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_UPPER_TARGET));
		desc += sprintf(buf + desc, "\tLIMIT      0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_LIMIT));
		desc += sprintf(buf + desc, "\tCR1        0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_CR1));
		desc += sprintf(buf + desc, "\tCR2        0x%08x\n",
		      readl(pcie->dbi + PCIE_ATU_CR2));
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, desc);
	kfree(buf);

	return len;
}

/**
 * ls_pcie_ep_dbg_regs_write - write into regs datum
 * @filp: the opened file
 * @buffer: where to find the user's data
 * @count: the length of the user's data
 * @ppos: file position offset
 **/
static ssize_t ls_pcie_ep_dbg_regs_write(struct file *filp,
					 const char __user *buffer,
					 size_t count, loff_t *ppos)
{
	struct ls_ep_dev *ep = filp->private_data;
	struct ls_pcie *pcie = ep->pcie;
	char buf[256];

	if (count >= sizeof(buf))
		return -ENOSPC;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	ls_pcie_ep_dev_cfg_enable(ep);

	if (strncmp(buf, "reg", 3) == 0) {
		u32 reg, value;
		int cnt;

		cnt = sscanf(&buf[3], "%x %x", &reg, &value);
		if (cnt == 2) {
			writel(value, pcie->dbi + reg);
			value = readl(pcie->dbi + reg);
			dev_info(&ep->dev, "reg 0x%08x: 0x%08x\n",
				 reg, value);
		} else {
			dev_info(&ep->dev, "reg <reg> <value>\n");
		}
	} else if (strncmp(buf, "atu", 3) == 0) {
		/* to do */
		dev_info(&ep->dev, " Not support atu command\n");
	} else {
		dev_info(&ep->dev, "Unknown command %s\n", buf);
		dev_info(&ep->dev, "Available commands:\n");
		dev_info(&ep->dev, "   reg <reg> <value>\n");
	}

	return count;
}

static const struct file_operations ls_pcie_ep_dbg_regs_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read =  ls_pcie_ep_dbg_regs_read,
	.write = ls_pcie_ep_dbg_regs_write,
};

static ssize_t ls_pcie_ep_dbg_test_read(struct file *filp,
				   char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct ls_ep_dev *ep = filp->private_data;
	struct ls_ep_test *test = ep->driver_data;
	char buf[512];
	int desc = 0, len;

	if (!test) {
		dev_info(&ep->dev, " there is NO test\n");
		return 0;
	}

	if (test->status != TEST_IDLE) {
		dev_info(&ep->dev, "test %s is running\n", test->cmd);
		return 0;
	}

	desc = sprintf(buf, "MSI ADDR:0x%llx MSI DATA:0x%x\n",
		test->msi_msg_addr, test->msi_msg_data);

	desc += sprintf(buf + desc, "%s throughput:%lluMbps\n",
			test->cmd, test->result);

	len = simple_read_from_buffer(buffer, count, ppos,
				      buf, desc);

	return len;
}

static ssize_t ls_pcie_ep_dbg_test_write(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *ppos)
{
	struct ls_ep_dev *ep = filp->private_data;
	char buf[256];

	if (count >= sizeof(buf))
		return -ENOSPC;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (strncmp(buf, "init", 4) == 0) {
		int i = 4;
		u64 bus_addr;

		while (buf[i] == ' ')
			i++;

		if (kstrtou64(&buf[i], 0, &bus_addr))
			dev_info(&ep->dev, "command: init <bus_addr>\n");
		else {
			if (ls_pcie_ep_init_test(ep, bus_addr))
				dev_info(&ep->dev, "failed to init test\n");
		}
	} else if (strncmp(buf, "free", 4) == 0)
		ls_pcie_ep_free_test(ep);
	else if (strncmp(buf, "dma", 3) == 0 ||
		 strncmp(buf, "cpy", 3) == 0)
		ls_pcie_ep_start_test(ep, buf);
	else {
		dev_info(&ep->dev, "Unknown command: %s\n", buf);
		dev_info(&ep->dev, "Available commands:\n");
		dev_info(&ep->dev, "\tinit <bus_addr>\n");
		dev_info(&ep->dev, "\t<dma/cpy> <r/w> <packet_size> <loop>\n");
		dev_info(&ep->dev, "\tfree\n");
	}

	return count;
}

static const struct file_operations ls_pcie_ep_dbg_test_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ls_pcie_ep_dbg_test_read,
	.write = ls_pcie_ep_dbg_test_write,
};

static ssize_t ls_pcie_ep_dbg_dump_read(struct file *filp,
				   char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct ls_ep_dev *ep = filp->private_data;
	struct ls_ep_test *test = ep->driver_data;
	char *buf;
	int desc = 0, i, len;

	buf = kmalloc(4 * 1024, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (!test) {
		dev_info(&ep->dev, " there is NO test\n");
		kfree(buf);
		return 0;
	}

	desc += sprintf(buf + desc, "%s", "dump info:");
	for (i = 0; i < 256; i += 4) {
		if (i % 16 == 0)
			desc += sprintf(buf + desc, "\n%08x:", i);
		desc += sprintf(buf + desc, " %08x", readl(test->buf + i));
	}

	desc += sprintf(buf + desc, "\n");
	len = simple_read_from_buffer(buffer, count, ppos, buf, desc);

	kfree(buf);

	return len;
}

static const struct file_operations ls_pcie_ep_dbg_dump_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = ls_pcie_ep_dbg_dump_read,
};

static int ls_pcie_ep_dev_dbgfs_init(struct ls_ep_dev *ep)
{
	struct ls_pcie *pcie = ep->pcie;
	struct dentry *pfile;

	ls_pcie_ep_dev_cfg_enable(ep);

	ep->dir = debugfs_create_dir(dev_name(&ep->dev), pcie->dir);
	if (!ep->dir)
		return -ENOMEM;

	pfile = debugfs_create_file("regs", 0600, ep->dir, ep,
				    &ls_pcie_ep_dbg_regs_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs regs for failed\n");

	pfile = debugfs_create_file("test", 0600, ep->dir, ep,
				    &ls_pcie_ep_dbg_test_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs test for failed\n");

	pfile = debugfs_create_file("dump", 0600, ep->dir, ep,
				    &ls_pcie_ep_dbg_dump_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs dump for failed\n");

	return 0;
}

int ls_pcie_ep_dbgfs_init(struct ls_pcie *pcie)
{
	struct ls_ep_dev *ep;

	pcie->dir = debugfs_create_dir(dev_name(pcie->dev), NULL);
	if (!pcie->dir)
		return -ENOMEM;

	list_for_each_entry(ep, &pcie->ep_list, node)
		ls_pcie_ep_dev_dbgfs_init(ep);

	return 0;
}

int ls_pcie_ep_dbgfs_remove(struct ls_pcie *pcie)
{
	debugfs_remove_recursive(pcie->dir);
	return 0;
}

MODULE_AUTHOR("Minghuan Lian <Minghuan.Lian@freescale.com>");
MODULE_DESCRIPTION("Freescale Layerscape PCIe EP controller driver");
MODULE_LICENSE("GPL v2");
