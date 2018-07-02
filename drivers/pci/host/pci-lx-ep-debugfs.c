/*
 * PCIe Endpoint driver for Freescale Layerscape SoCs
 *
 * Copyright 2018 NXP.
 *
 * Author: Xiaowei Bao <xiaowei.bao@nxp.com>
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

#include "pcie-mobiveil.h"
#include "pci-lx-ep.h"

#define PCIE_MSI_MSG_ADDR_OFF	0x8c
#define PCIE_MSI_MSG_DATA_OFF	0x94
#define PCIE_MSI_MSG_ADDR_OFF_VF	0x84
#define PCIE_MSI_MSG_DATA_OFF_VF	0x8c
#define MSIX_TEST		1

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

struct lx_ep_test {
	struct lx_ep_dev	*ep;
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
#ifdef MSIX_TEST
	u64			msix_msg_addr[3];
	u16			msix_msg_data[3];
#else
	u16			msi_msg_data;
#endif
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

#ifdef MSIX_TEST
static int lx_pcie_ep_trigger_msix(struct lx_ep_test *test, int entry)
{
	int offset;

	if (!test->msi)
		return -EINVAL;

	offset = test->msix_msg_addr[entry] - test->msi_msg_addr;
	if (offset > PCIE_BAR_SIZE)
		return -EINVAL;

	iowrite32(test->msix_msg_data[entry], test->msi + offset);

	return 0;
}
#else
static int lx_pcie_ep_trigger_msi(struct lx_ep_test *test)
{
	if (!test->msi)
		return -EINVAL;

	iowrite32(test->msi_msg_data, test->msi);

	return 0;
}
#endif

static int lx_pcie_ep_test_try_run(struct lx_ep_test *test)
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

static void lx_pcie_ep_test_done(struct lx_ep_test *test)
{
	spin_lock(&test->lock);
	test->status = TEST_IDLE;
	spin_unlock(&test->lock);
}

static void lx_pcie_ep_test_dma_cb(void *arg)
{
	struct lx_ep_test *test = arg;

	complete(&test->done);
}

static int lx_pcie_ep_test_dma(struct lx_ep_test *test)
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

		dma_desc->callback = lx_pcie_ep_test_dma_cb;
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
	do_div(test->result, period.tv_sec * 1000 * 1000 * 1000 +
			period.tv_nsec);
	dma_release_channel(chan);

	return 0;

_err:
	dma_release_channel(chan);
	test->result = 0;
	return -EINVAL;
}

static int lx_pcie_ep_test_cpy(struct lx_ep_test *test)
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
	do_div(test->result, period.tv_sec * 1000 * 1000 * 1000 +
			period.tv_nsec);

	return 0;
}

int lx_pcie_ep_test_thread(void *arg)
{
	int ret;

	struct lx_ep_test *test = arg;

	if (test->type == TEST_TYPE_DMA)
		ret = lx_pcie_ep_test_dma(test);
	else
		ret = lx_pcie_ep_test_cpy(test);

	if (ret) {
		pr_err("\n%s \ttest failed\n",
		       test->cmd);
		test->result = 0;
	} else
		pr_err("\n%s \tthroughput:%lluMbps\n",
		       test->cmd, test->result);

	lx_pcie_ep_test_done(test);

#ifdef MSIX_TEST
	lx_pcie_ep_trigger_msix(test, 0);
	lx_pcie_ep_trigger_msix(test, 1);
	lx_pcie_ep_trigger_msix(test, 2);
#else
	lx_pcie_ep_trigger_msi(test);
#endif

	do_exit(0);
}

static int lx_pcie_ep_free_test(struct lx_ep_dev *ep)
{
	struct lx_ep_test *test = ep->driver_data;

	if (!test)
		return 0;

	if (test->status == TEST_BUSY) {
		kthread_stop(test->thread);
		dev_info(&ep->dev,
			 "test is running please wait and run again\n");
		return -EBUSY;
	}

	if (test->out)
		iounmap(test->out);

	kfree(test);
	ep->driver_data = NULL;

	return 0;
}

static int lx_pcie_ep_init_test(struct lx_ep_dev *ep, u64 bus_addr)
{
	struct lx_pcie *pcie = ep->pcie;
	struct mv_pcie *mv_pci = pcie->pci;
	struct lx_ep_test *test = ep->driver_data;
	int err;
	int val;
	int i;
	u32 msix_addr_high;

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

	if (ep->vf_idx)
		test->buf = pcie->buf_vf + (ep->vf_idx - 1) * PCIE_BAR_SIZE +
				ep->pf_idx * PCIE_VF_NUM * PCIE_BAR_SIZE * 4
				+ 2 * PCIE_BAR_SIZE * PCIE_VF_NUM;
	else
		test->buf = pcie->buf_pf + ep->pf_idx * PCIE_BAR_SIZE * 4 +
				2 * PCIE_BAR_SIZE;

	if (!test->buf) {
		dev_info(&ep->dev, "failed to get mem for bar0\n");
		err = -ENOMEM;
		goto _err;
	}

	test->out_addr = pcie->out_base + ep->dev_id * PCIE_BAR_SIZE * 2;
	test->out = ioremap(test->out_addr, PCIE_BAR_SIZE);
	if (!test->out) {
		dev_info(&ep->dev, "failed to map out\n");
		err = -ENOMEM;
		goto _err;
	}

	test->bus_addr = ALIGN(bus_addr, PCIE_BAR_SIZE);
	test->msi_addr = test->out_addr + PCIE_BAR_SIZE;

	test->msi = ioremap(test->msi_addr, PCIE_BAR_SIZE);
	if (!test->msi)
		dev_info(&ep->dev, "failed to map MSI outbound region\n");

	val =  mv_pcie_readl_csr(mv_pci, PAB_CTRL);
	val &= ~(PAB_CTRL_FUNC_SEL_MASK << PAB_CTRL_FUNC_SEL_SHIFT);
	val |= (ep->dev_id & PAB_CTRL_FUNC_SEL_MASK) << PAB_CTRL_FUNC_SEL_SHIFT;
	mv_pcie_writel_csr(mv_pci, PAB_CTRL, val);

#ifdef MSIX_TEST
	for (i = 0; i < 3; i++) {
		msix_addr_high = mv_pcie_readl_csr(mv_pci,
			PAB_MSIX_TABLE_PBA_ACCESS + 0x4 + 0x10 * i);

		test->msix_msg_addr[i] = mv_pcie_readl_csr(mv_pci,
			 PAB_MSIX_TABLE_PBA_ACCESS + 0x10 * i) |
			(((u64)msix_addr_high) << 32);

		test->msix_msg_data[i] = mv_pcie_readl_csr(mv_pci,
			 PAB_MSIX_TABLE_PBA_ACCESS + 8 + 0x10 * i);
	}
	test->msi_msg_addr = test->msix_msg_addr[0] & (~(PCIE_BAR_SIZE - 1));
#else
	if (ep->dev_id < PCIE_PF_NUM) {
		msix_addr_high =
			mv_pcie_readl_csr(mv_pci, PCIE_MSI_MSG_ADDR_OFF + 0x4);
		test->msi_msg_addr =
			mv_pcie_readl_csr(mv_pci,  PCIE_MSI_MSG_ADDR_OFF) |
			(((u64)msix_addr_high) << 32);
		test->msi_msg_data =
			mv_pcie_readl_csr(mv_pci, PCIE_MSI_MSG_DATA_OFF);
	} else {
		msix_addr_high =
			mv_pcie_readl_csr(mv_pci,
					PCIE_MSI_MSG_ADDR_OFF_VF + 0x4);
		test->msi_msg_addr =
			mv_pcie_readl_csr(mv_pci,  PCIE_MSI_MSG_ADDR_OFF_VF) |
			(((u64)msix_addr_high) << 32);
		test->msi_msg_data =
			mv_pcie_readl_csr(mv_pci, PCIE_MSI_MSG_DATA_OFF_VF);
	}
#endif

	val =  mv_pcie_readl_csr(mv_pci, PAB_CTRL);
	val &= ~(PAB_CTRL_FUNC_SEL_MASK << PAB_CTRL_FUNC_SEL_SHIFT);
	mv_pcie_writel_csr(mv_pci, PAB_CTRL, val);

	/* outbound window set for memory */
	lx_pcie_ep_outbound_win_set(pcie, ep->dev_id * 2, TYPE_MEM,
				  test->out_addr, bus_addr,
				  ep->dev_id, PCIE_BAR_SIZE);

	/* outbound window set for MSI-X and MSI*/
	lx_pcie_ep_outbound_win_set(pcie, ep->dev_id * 2 + 1, TYPE_MEM,
				  test->msi_addr, test->msi_msg_addr,
				  ep->dev_id, PCIE_BAR_SIZE);

	return 0;

_err:
	lx_pcie_ep_free_test(ep);
	return err;
}

static int lx_pcie_ep_start_test(struct lx_ep_dev *ep, char *cmd)
{
	struct lx_ep_test *test = ep->driver_data;
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

	if (len > PCIE_BAR_SIZE) {
		dev_err(&ep->dev, "max len is %d", PCIE_BAR_SIZE);
		return -EINVAL;
	}

	if (!test) {
		dev_err(&ep->dev, "Please first run init command\n");
		return -EINVAL;
	}

	if (lx_pcie_ep_test_try_run(test)) {
		dev_err(&ep->dev, "There is already a test running\n");
		return -EINVAL;
	}

	test->len = len;
	test->loop = loop;
	test->type = type;
	test->data = (char)data;
	test->dirt = dirt;
	strcpy(test->cmd, cmd);
	test->thread = kthread_run(lx_pcie_ep_test_thread, test,
				   "pcie ep test");
	if (IS_ERR(test->thread)) {
		dev_err(&ep->dev, "fork failed for pcie ep test\n");
		lx_pcie_ep_test_done(test);
		ret = PTR_ERR(test->thread);
	}

	return ret;
}

static ssize_t lx_pcie_ep_dbg_test_read(struct file *filp,
				   char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct lx_ep_dev *ep = filp->private_data;
	struct lx_ep_test *test = ep->driver_data;
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

#ifdef MSIX_TEST
	desc = sprintf(buf, "MSI ADDR:0x%llx MSI DATA:0x%x\n",
		test->msi_msg_addr, test->msix_msg_data[0]);
#else
	desc = sprintf(buf, "MSI ADDR:0x%llx MSI DATA:0x%x\n",
		test->msi_msg_addr, test->msi_msg_data);
#endif
	desc += sprintf(buf + desc, "%s throughput:%lluMbps\n",
			test->cmd, test->result);

	len = simple_read_from_buffer(buffer, count, ppos,
				      buf, desc);

	return len;
}

static ssize_t lx_pcie_ep_dbg_test_write(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *ppos)
{
	struct lx_ep_dev *ep = filp->private_data;
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
			if (lx_pcie_ep_init_test(ep, bus_addr))
				dev_info(&ep->dev, "failed to init test\n");
		}
	} else if (strncmp(buf, "free", 4) == 0)
		lx_pcie_ep_free_test(ep);
	else if (strncmp(buf, "dma", 3) == 0 ||
		 strncmp(buf, "cpy", 3) == 0)
		lx_pcie_ep_start_test(ep, buf);
	else {
		dev_info(&ep->dev, "Unknown command: %s\n", buf);
		dev_info(&ep->dev, "Available commands:\n");
		dev_info(&ep->dev, "\tinit <bus_addr>\n");
		dev_info(&ep->dev, "\t<dma/cpy> <r/w> <packet_size> <loop>\n");
		dev_info(&ep->dev, "\tfree\n");
	}

	return count;
}

static const struct file_operations lx_pcie_ep_dbg_test_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = lx_pcie_ep_dbg_test_read,
	.write = lx_pcie_ep_dbg_test_write,
};

static ssize_t lx_pcie_ep_dbg_dump_read(struct file *filp,
				   char __user *buffer,
				   size_t count, loff_t *ppos)
{
	struct lx_ep_dev *ep = filp->private_data;
	struct lx_ep_test *test = ep->driver_data;
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

static const struct file_operations lx_pcie_ep_dbg_dump_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = lx_pcie_ep_dbg_dump_read,
};

static ssize_t ls_pcie_ep_dbg_regs_read(struct file *filp, char __user *buffer,
				    size_t count, loff_t *ppos)
{
	struct lx_ep_dev *ep = filp->private_data;
	struct lx_pcie *pcie = ep->pcie;
	struct mv_pcie *mv_pci = pcie->pci;
	char *buf;
	int desc = 0, i, len;
	int func, bar;

	buf = kmalloc(1024 * 1024, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	desc += sprintf(buf + desc, "\n%s", "outbound info:\n");
	for (i = 0; i < 256; i++) {
		desc += sprintf(buf + desc, "OUB_WIN%d\n", i);
		desc += sprintf(buf + desc, "\tLOWER PHYS	0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_AXI_WIN(i)));
		desc += sprintf(buf + desc, "\tUPPER PHYS	0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_EXT_AXI_AMAP_AXI_WIN(i)));
		desc += sprintf(buf + desc, "\tLOWER BUS	0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_PEX_WIN_L(i)));
		desc += sprintf(buf + desc, "\tUPPER BUS	0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_PEX_WIN_H(i)));
		desc += sprintf(buf + desc, "\tSIZE		0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_CTRL(i)) &
			(AXI_AMAP_CTRL_SIZE_MASK << AXI_AMAP_CTRL_SIZE_SHIFT));
		desc += sprintf(buf + desc, "\tEXT_SIZ		0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_EXT_AXI_AMAP_SIZE(i)));
		desc += sprintf(buf + desc, "\tPARAM		0x%08x\n",
			mv_pcie_readl_csr(mv_pci,
				PAB_AXI_AMAP_PCI_HDR_PARAM(i)));
		desc += sprintf(buf + desc, "\tCTRL		0x%08x\n",
			mv_pcie_readl_csr(mv_pci, PAB_AXI_AMAP_CTRL(i)));
	}

	desc += sprintf(buf + desc, "\n%s", "inbound info:\n");
	for (func = 0; func < 2; func++) {
		for (bar = 0; bar < 8; bar++) {
			desc += sprintf(buf + desc, "INB_WIN%d\n",
					func * 8 + bar);
			desc += sprintf(buf + desc, "\tBAR_AMAP	0x%08x\n",
				mv_pcie_readl_csr(mv_pci,
					PAB_PEX_BAR_AMAP(func, bar)));
			desc += sprintf(buf + desc, "\tEXT_BAR_AMAP	\
				0x%08x\n", mv_pcie_readl_csr(mv_pci,
					PAB_EXT_PEX_BAR_AMAP(func, bar)));
		}
	}

	len = simple_read_from_buffer(buffer, count, ppos, buf, desc);
	kfree(buf);

	return len;
}

static ssize_t ls_pcie_ep_dbg_regs_write(struct file *filp,
					 const char __user *buffer,
					 size_t count, loff_t *ppos)
{
	struct lx_ep_dev *ep = filp->private_data;
	struct lx_pcie *pcie = ep->pcie;
	struct mv_pcie *mv_pci = pcie->pci;
	char buf[256];

	if (count >= sizeof(buf))
		return -ENOSPC;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (strncmp(buf, "reg", 3) == 0) {
		u32 reg, value;
		int cnt;

		cnt = sscanf(&buf[3], "%x %x", &reg, &value);
		if (cnt == 2) {
			mv_pcie_writel_csr(mv_pci, reg, value);
			value = mv_pcie_readl_csr(mv_pci, reg);
			dev_info(&ep->dev, "reg 0x%08x: 0x%08x\n",
				 reg, value);
		} else {
			dev_info(&ep->dev, "reg <reg> <value>\n");
		}
	} else if (strncmp(buf, "lut", 3) == 0) {
		/* to do */
		dev_info(&ep->dev, " Not support lut command\n");
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
static int lx_pcie_ep_dev_dbgfs_init(struct lx_ep_dev *ep)
{
	struct lx_pcie *pcie = ep->pcie;
	struct dentry *pfile;


	ep->dir = debugfs_create_dir(dev_name(&ep->dev), pcie->dir);
	if (!ep->dir)
		return -ENOMEM;

	pfile = debugfs_create_file("regs", 0600, ep->dir, ep,
				    &ls_pcie_ep_dbg_regs_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs regs for failed\n");

	pfile = debugfs_create_file("test", 0600, ep->dir, ep,
				    &lx_pcie_ep_dbg_test_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs test for failed\n");
	pfile = debugfs_create_file("dump", 0600, ep->dir, ep,
				    &lx_pcie_ep_dbg_dump_fops);
	if (!pfile)
		dev_info(&ep->dev, "debugfs dump for failed\n");

	return 0;
}

int lx_pcie_ep_dbgfs_init(struct lx_pcie *pcie)
{
	struct lx_ep_dev *ep;

	pcie->dir = debugfs_create_dir(dev_name(pcie->dev), NULL);
	if (!pcie->dir)
		return -ENOMEM;

	list_for_each_entry(ep, &pcie->ep_list, node)
		lx_pcie_ep_dev_dbgfs_init(ep);

	return 0;
}

int lx_pcie_ep_dbgfs_remove(struct lx_pcie *pcie)
{
	debugfs_remove_recursive(pcie->dir);
	return 0;
}

MODULE_AUTHOR("Xiaowei Bao <xiaowei.bao@nxp.com>");
MODULE_DESCRIPTION("Freescale Lx2160 PCIe EP controller driver");
MODULE_LICENSE("GPL v2");
