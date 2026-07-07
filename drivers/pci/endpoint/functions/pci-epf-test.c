// SPDX-License-Identifier: GPL-2.0
/*
 * Test driver to test endpoint functionality
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 * Copyright 2026 NXP
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/firmware.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci-ep-msi.h>
#include <linux/pci_regs.h>
#include <linux/cdev.h>

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

static dev_t stream_dev;
static struct cdev stream_cdev;
static struct class *stream_class;
static phys_addr_t vram_phys;
static size_t vram_size;
static uint64_t vram_fb_offset;

struct stream_dmabuf {
	phys_addr_t phys;
	size_t size;
	struct device *dev;
};

#define STREAM_IOC_GET_DMABUF	_IOR('S', 1, int)
#define STREAM_IOC_SET_OFFSET	_IOW('S', 2, uint64_t)

#define IRQ_TYPE_INTX			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

#define COMMAND_RAISE_INTX_IRQ		BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_READ			BIT(3)
#define COMMAND_WRITE			BIT(4)
#define COMMAND_COPY			BIT(5)
#define COMMAND_ENABLE_DOORBELL		BIT(6)
#define COMMAND_DISABLE_DOORBELL	BIT(7)
#define COMMAND_CHANGE_RESOLUTION	BIT(8)
#define COMMAND_GET_FB_OFFSET		BIT(9)
#define COMMAND_START_STREAM		BIT(10)
#define COMMAND_STOP_STREAM		BIT(11)

#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)
#define STATUS_DOORBELL_SUCCESS		BIT(9)
#define STATUS_DOORBELL_ENABLE_SUCCESS	BIT(10)
#define STATUS_DOORBELL_ENABLE_FAIL	BIT(11)
#define STATUS_DOORBELL_DISABLE_SUCCESS	BIT(12)
#define STATUS_DOORBELL_DISABLE_FAIL	BIT(13)

#define FLAG_USE_DMA			BIT(0)

#define TIMER_RESOLUTION		1

#define CAP_UNALIGNED_ACCESS		BIT(0)
#define CAP_MSI				BIT(1)
#define CAP_MSIX			BIT(2)
#define CAP_INTX			BIT(3)

#define EDID_PATH	"/sys/class/drm/card0-HDMI-A-1/edid"
#define EDID_SIZE	256

static struct workqueue_struct *kpcitest_workqueue;

struct pci_epf_test {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	struct pci_epf_bar	rom_bar;	/* ROM BAR */
	enum pci_barno		test_reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	struct dma_chan		*dma_chan_tx;
	struct dma_chan		*dma_chan_rx;
	struct dma_chan		*transfer_chan;
	dma_cookie_t		transfer_cookie;
	enum dma_status		transfer_status;
	struct completion	transfer_complete;
	bool			dma_supported;
	bool			dma_private;
	const struct pci_epc_features *epc_features;
	struct pci_epf_bar	db_bar;
};

/* Default EDID fallback (max 4K) */
static unsigned char default_edid[256] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
	0x09, 0xd1, 0x49, 0x79, 0x45, 0x54, 0x00, 0x00,
	0x03, 0x20, 0x01, 0x03, 0x80, 0x3e, 0x22, 0x78,
	0x2e, 0x08, 0xa5, 0xa2, 0x57, 0x4f, 0xa2, 0x28,
	0x0f, 0x50, 0x54, 0xa5, 0x6b, 0x80, 0xd1, 0xc0,
	0x81, 0xc0, 0x81, 0x00, 0x81, 0x80, 0xa9, 0xc0,
	0xb3, 0x00, 0xa9, 0x40, 0x01, 0x01, 0x51, 0xd0,
	0x00, 0xa0, 0xf0, 0x70, 0x3e, 0x80, 0x30, 0x20,
	0x35, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00, 0x1a,
	0x00, 0x00, 0x00, 0xff, 0x00, 0x32, 0x31, 0x4e,
	0x30, 0x37, 0x35, 0x38, 0x36, 0x53, 0x4c, 0x30,
	0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x18,
	0x4c, 0x1e, 0x87, 0x3c, 0x00, 0x0a, 0x20, 0x20,
	0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc,
	0x00, 0x42, 0x65, 0x6e, 0x51, 0x20, 0x45, 0x4c,
	0x32, 0x38, 0x37, 0x30, 0x55, 0x0a, 0x01, 0x63,
	0x02, 0x03, 0x4e, 0xf1, 0x56, 0x61, 0x60, 0x5d,
	0x5e, 0x5f, 0x10, 0x05, 0x04, 0x03, 0x02, 0x07,
	0x06, 0x0f, 0x1f, 0x20, 0x21, 0x22, 0x14, 0x13,
	0x12, 0x16, 0x01, 0x23, 0x09, 0x07, 0x07, 0x6d,
	0x03, 0x0c, 0x00, 0x10, 0x00, 0x38, 0x44, 0x20,
	0x00, 0x60, 0x01, 0x02, 0x03, 0x67, 0xd8, 0x5d,
	0xc4, 0x01, 0x78, 0x80, 0x03, 0xe3, 0x05, 0xc3,
	0x01, 0xe4, 0x0f, 0x03, 0x00, 0x00, 0xe6, 0x06,
	0x05, 0x01, 0x5a, 0x53, 0x44, 0x68, 0x1a, 0x00,
	0x00, 0x01, 0x01, 0x28, 0x3c, 0x00, 0x56, 0x5e,
	0x00, 0xa0, 0xa0, 0xa0, 0x29, 0x50, 0x30, 0x20,
	0x35, 0x00, 0x6d, 0x55, 0x21, 0x00, 0x00, 0x1a,
	0xf4, 0x51, 0x00, 0xa0, 0xf0, 0x70, 0x19, 0x80,
	0x30, 0x20, 0x35, 0x00, 0x6d, 0x55, 0x21, 0x00,
	0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb
};

struct pci_epf_test_reg {
	__le32 magic;
	__le32 command;
	__le32 status;
	__le64 src_addr;
	__le64 dst_addr;
	__le32 size;
	__le32 checksum;
	__le32 irq_type;
	__le32 irq_number;
	__le32 flags;
	__le32 height;
	__le32 width;
	__le64 dpu_fb_offset;
	__le32 caps;
	__le32 doorbell_bar;
	__le32 doorbell_offset;
	__le32 doorbell_data;
} __packed;

static struct pci_epf_header test_header = {
	.vendorid	= 0x1131,
	.deviceid	= 0x1234,
	.baseclass_code	= PCI_BASE_CLASS_DISPLAY,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static size_t bar_size[] = { 512, 512, 33554432, 16384, 4194304, 1048576 };

static void pci_epf_test_dma_callback(void *param)
{
	struct pci_epf_test *epf_test = param;
	struct dma_tx_state state;

	epf_test->transfer_status =
		dmaengine_tx_status(epf_test->transfer_chan,
				    epf_test->transfer_cookie, &state);
	if (epf_test->transfer_status == DMA_COMPLETE ||
	    epf_test->transfer_status == DMA_ERROR)
		complete(&epf_test->transfer_complete);
}

/**
 * pci_epf_test_data_transfer() - Function that uses dmaengine API to transfer
 *				  data between PCIe EP and remote PCIe RC
 * @epf_test: the EPF test device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 * @dma_remote: remote RC physical address
 * @dir: DMA transfer direction
 *
 * Function that uses dmaengine API to transfer data between PCIe EP and remote
 * PCIe RC. The source and destination address can be a physical address given
 * by pci_epc_mem_alloc_addr or the one obtained using DMA mapping APIs.
 *
 * The function returns '0' on success and negative value on failure.
 */
static int pci_epf_test_data_transfer(struct pci_epf_test *epf_test,
				      dma_addr_t dma_dst, dma_addr_t dma_src,
				      size_t len, dma_addr_t dma_remote,
				      enum dma_transfer_direction dir)
{
	struct dma_chan *chan = (dir == DMA_MEM_TO_DEV) ?
				 epf_test->dma_chan_tx : epf_test->dma_chan_rx;
	dma_addr_t dma_local = (dir == DMA_MEM_TO_DEV) ? dma_src : dma_dst;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct pci_epf *epf = epf_test->epf;
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	int ret;

	if (IS_ERR_OR_NULL(chan)) {
		dev_err(dev, "Invalid DMA memcpy channel\n");
		return -EINVAL;
	}

	if (epf_test->dma_private) {
		sconf.direction = dir;
		if (dir == DMA_MEM_TO_DEV)
			sconf.dst_addr = dma_remote;
		else
			sconf.src_addr = dma_remote;

		if (dmaengine_slave_config(chan, &sconf)) {
			dev_err(dev, "DMA slave config fail\n");
			return -EIO;
		}
		tx = dmaengine_prep_slave_single(chan, dma_local, len, dir,
						 flags);
	} else {
		tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len,
					       flags);
	}

	if (!tx) {
		dev_err(dev, "Failed to prepare DMA memcpy\n");
		return -EIO;
	}

	reinit_completion(&epf_test->transfer_complete);
	epf_test->transfer_chan = chan;
	tx->callback = pci_epf_test_dma_callback;
	tx->callback_param = epf_test;
	epf_test->transfer_cookie = dmaengine_submit(tx);

	ret = dma_submit_error(epf_test->transfer_cookie);
	if (ret) {
		dev_err(dev, "Failed to do DMA tx_submit %d\n", ret);
		goto terminate;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_interruptible(&epf_test->transfer_complete);
	if (ret < 0) {
		dev_err(dev, "DMA wait_for_completion interrupted\n");
		goto terminate;
	}

	if (epf_test->transfer_status == DMA_ERROR) {
		dev_err(dev, "DMA transfer failed\n");
		ret = -EIO;
	}

terminate:
	dmaengine_terminate_sync(chan);

	return ret;
}

struct epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static bool epf_dma_filter_fn(struct dma_chan *chan, void *node)
{
	struct epf_dma_filter *filter = node;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev
		&& (filter->dma_mask & caps.directions);
}

/**
 * pci_epf_test_init_dma_chan() - Function to initialize EPF test DMA channel
 * @epf_test: the EPF test device that performs data transfer operation
 *
 * Function to initialize EPF test DMA channel.
 */
static int pci_epf_test_init_dma_chan(struct pci_epf_test *epf_test)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct epf_dma_filter filter;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);
	if (!dma_chan) {
		dev_info(dev, "Failed to get private DMA rx channel. Falling back to generic one\n");
		goto fail_back_tx;
	}

	epf_test->dma_chan_rx = dma_chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);

	if (!dma_chan) {
		dev_info(dev, "Failed to get private DMA tx channel. Falling back to generic one\n");
		goto fail_back_rx;
	}

	epf_test->dma_chan_tx = dma_chan;
	epf_test->dma_private = true;

	init_completion(&epf_test->transfer_complete);

	return 0;

fail_back_rx:
	dma_release_channel(epf_test->dma_chan_rx);
	epf_test->dma_chan_rx = NULL;

fail_back_tx:
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf_test->transfer_complete);

	epf_test->dma_chan_tx = epf_test->dma_chan_rx = dma_chan;

	return 0;
}

/**
 * pci_epf_test_clean_dma_chan() - Function to cleanup EPF test DMA channel
 * @epf_test: the EPF test device that performs data transfer operation
 *
 * Helper to cleanup EPF test DMA channel.
 */
static void pci_epf_test_clean_dma_chan(struct pci_epf_test *epf_test)
{
	if (!epf_test->dma_supported)
		return;

	if (epf_test->dma_chan_tx) {
		dma_release_channel(epf_test->dma_chan_tx);
		if (epf_test->dma_chan_tx == epf_test->dma_chan_rx) {
			epf_test->dma_chan_tx = NULL;
			epf_test->dma_chan_rx = NULL;
			return;
		}
		epf_test->dma_chan_tx = NULL;
	}

	if (epf_test->dma_chan_rx) {
		dma_release_channel(epf_test->dma_chan_rx);
		epf_test->dma_chan_rx = NULL;
	}
}

static void pci_epf_test_print_rate(struct pci_epf_test *epf_test,
				    const char *op, u64 size,
				    struct timespec64 *start,
				    struct timespec64 *end, bool dma)
{
	struct timespec64 ts = timespec64_sub(*end, *start);
	u64 rate = 0, ns;

	/* calculate the rate */
	ns = timespec64_to_ns(&ts);
	if (ns)
		rate = div64_u64(size * NSEC_PER_SEC, ns * 1000);

	dev_info(&epf_test->epf->dev,
		 "%s => Size: %llu B, DMA: %s, Time: %llu.%09u s, Rate: %llu KB/s\n",
		 op, size, dma ? "YES" : "NO",
		 (u64)ts.tv_sec, (u32)ts.tv_nsec, rate);
}

static void pci_epf_test_copy(struct pci_epf_test *epf_test,
			      struct pci_epf_test_reg *reg)
{
	int ret = 0;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epc_map src_map, dst_map;
	u64 src_addr = le64_to_cpu(reg->src_addr);
	u64 dst_addr = le64_to_cpu(reg->dst_addr);
	size_t orig_size, copy_size;
	ssize_t map_size = 0;
	u32 flags = le32_to_cpu(reg->flags);
	u32 status = 0;
	void *copy_buf = NULL, *buf;

	orig_size = copy_size = le32_to_cpu(reg->size);

	if (flags & FLAG_USE_DMA) {
		if (!dma_has_cap(DMA_MEMCPY, epf_test->dma_chan_tx->device->cap_mask)) {
			dev_err(dev, "DMA controller doesn't support MEMCPY\n");
			ret = -EINVAL;
			goto set_status;
		}
	} else {
		copy_buf = kzalloc(copy_size, GFP_KERNEL);
		if (!copy_buf) {
			ret = -ENOMEM;
			goto set_status;
		}
		buf = copy_buf;
	}

	while (copy_size) {
		ret = pci_epc_mem_map(epc, epf->func_no, epf->vfunc_no,
				      src_addr, copy_size, &src_map);
		if (ret) {
			dev_err(dev, "Failed to map source address\n");
			status = STATUS_SRC_ADDR_INVALID;
			goto free_buf;
		}

		ret = pci_epc_mem_map(epf->epc, epf->func_no, epf->vfunc_no,
				      dst_addr, copy_size, &dst_map);
		if (ret) {
			dev_err(dev, "Failed to map destination address\n");
			status = STATUS_DST_ADDR_INVALID;
			pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no,
					  &src_map);
			goto free_buf;
		}

		map_size = min_t(size_t, dst_map.pci_size, src_map.pci_size);

		ktime_get_ts64(&start);
		if (flags & FLAG_USE_DMA) {
			ret = pci_epf_test_data_transfer(epf_test,
					dst_map.phys_addr, src_map.phys_addr,
					map_size, 0, DMA_MEM_TO_MEM);
			if (ret) {
				dev_err(dev, "Data transfer failed\n");
				goto unmap;
			}
		} else {
			memcpy_fromio(buf, src_map.virt_addr, map_size);
			memcpy_toio(dst_map.virt_addr, buf, map_size);
			buf += map_size;
		}
		ktime_get_ts64(&end);

		copy_size -= map_size;
		src_addr += map_size;
		dst_addr += map_size;

		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &dst_map);
		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &src_map);
		map_size = 0;
	}

	pci_epf_test_print_rate(epf_test, "COPY", orig_size, &start, &end,
				flags & FLAG_USE_DMA);

unmap:
	if (map_size) {
		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &dst_map);
		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &src_map);
	}

free_buf:
	kfree(copy_buf);

set_status:
	if (!ret)
		status |= STATUS_COPY_SUCCESS;
	else
		status |= STATUS_COPY_FAIL;
	reg->status = cpu_to_le32(status);
}

static void pci_epf_test_read(struct pci_epf_test *epf_test,
			      struct pci_epf_test_reg *reg)
{
	int ret = 0;
	void *src_buf, *buf;
	u32 crc32;
	struct pci_epc_map map;
	phys_addr_t dst_phys_addr;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct device *dma_dev = epf->epc->dev.parent;
	u64 src_addr = le64_to_cpu(reg->src_addr);
	size_t orig_size, src_size;
	ssize_t map_size = 0;
	u32 flags = le32_to_cpu(reg->flags);
	u32 checksum = le32_to_cpu(reg->checksum);
	u32 status = 0;

	orig_size = src_size = le32_to_cpu(reg->size);

	src_buf = kzalloc(src_size, GFP_KERNEL);
	if (!src_buf) {
		ret = -ENOMEM;
		goto set_status;
	}
	buf = src_buf;

	while (src_size) {
		ret = pci_epc_mem_map(epc, epf->func_no, epf->vfunc_no,
				      src_addr, src_size, &map);
		if (ret) {
			dev_err(dev, "Failed to map address\n");
			status = STATUS_SRC_ADDR_INVALID;
			goto free_buf;
		}

		map_size = map.pci_size;
		if (flags & FLAG_USE_DMA) {
			dst_phys_addr = dma_map_single(dma_dev, buf, map_size,
						       DMA_FROM_DEVICE);
			if (dma_mapping_error(dma_dev, dst_phys_addr)) {
				dev_err(dev,
					"Failed to map destination buffer addr\n");
				ret = -ENOMEM;
				goto unmap;
			}

			ktime_get_ts64(&start);
			ret = pci_epf_test_data_transfer(epf_test,
					dst_phys_addr, map.phys_addr,
					map_size, src_addr, DMA_DEV_TO_MEM);
			if (ret)
				dev_err(dev, "Data transfer failed\n");
			ktime_get_ts64(&end);

			dma_unmap_single(dma_dev, dst_phys_addr, map_size,
					 DMA_FROM_DEVICE);

			if (ret)
				goto unmap;
		} else {
			ktime_get_ts64(&start);
			memcpy_fromio(buf, map.virt_addr, map_size);
			ktime_get_ts64(&end);
		}

		src_size -= map_size;
		src_addr += map_size;
		buf += map_size;

		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &map);
		map_size = 0;
	}

	pci_epf_test_print_rate(epf_test, "READ", orig_size, &start, &end,
				flags & FLAG_USE_DMA);

	crc32 = crc32_le(~0, src_buf, orig_size);
	if (crc32 != checksum)
		ret = -EIO;

unmap:
	if (map_size)
		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &map);

free_buf:
	kfree(src_buf);

set_status:
	if (!ret)
		status |= STATUS_READ_SUCCESS;
	else
		status |= STATUS_READ_FAIL;
	reg->status = cpu_to_le32(status);
}

static void pci_epf_test_write(struct pci_epf_test *epf_test,
			       struct pci_epf_test_reg *reg)
{
	int ret = 0;
	void *dst_buf, *buf;
	struct pci_epc_map map;
	phys_addr_t src_phys_addr;
	struct timespec64 start, end;
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct device *dma_dev = epf->epc->dev.parent;
	u64 dst_addr = le64_to_cpu(reg->dst_addr);
	size_t orig_size, dst_size;
	ssize_t map_size = 0;
	u32 flags = le32_to_cpu(reg->flags);
	u32 status = 0;

	orig_size = dst_size = le32_to_cpu(reg->size);

	dst_buf = kzalloc(dst_size, GFP_KERNEL);
	if (!dst_buf) {
		ret = -ENOMEM;
		goto set_status;
	}
	get_random_bytes(dst_buf, dst_size);
	reg->checksum = cpu_to_le32(crc32_le(~0, dst_buf, dst_size));
	buf = dst_buf;

	while (dst_size) {
		ret = pci_epc_mem_map(epc, epf->func_no, epf->vfunc_no,
				      dst_addr, dst_size, &map);
		if (ret) {
			dev_err(dev, "Failed to map address\n");
			status = STATUS_DST_ADDR_INVALID;
			goto free_buf;
		}

		map_size = map.pci_size;
		if (flags & FLAG_USE_DMA) {
			src_phys_addr = dma_map_single(dma_dev, buf, map_size,
						       DMA_TO_DEVICE);
			if (dma_mapping_error(dma_dev, src_phys_addr)) {
				dev_err(dev,
					"Failed to map source buffer addr\n");
				ret = -ENOMEM;
				goto unmap;
			}

			ktime_get_ts64(&start);

			ret = pci_epf_test_data_transfer(epf_test,
						map.phys_addr, src_phys_addr,
						map_size, dst_addr,
						DMA_MEM_TO_DEV);
			if (ret)
				dev_err(dev, "Data transfer failed\n");
			ktime_get_ts64(&end);

			dma_unmap_single(dma_dev, src_phys_addr, map_size,
					 DMA_TO_DEVICE);

			if (ret)
				goto unmap;
		} else {
			ktime_get_ts64(&start);
			memcpy_toio(map.virt_addr, buf, map_size);
			ktime_get_ts64(&end);
		}

		dst_size -= map_size;
		dst_addr += map_size;
		buf += map_size;

		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &map);
		map_size = 0;
	}

	pci_epf_test_print_rate(epf_test, "WRITE", orig_size, &start, &end,
				flags & FLAG_USE_DMA);

	/*
	 * wait 1ms inorder for the write to complete. Without this delay L3
	 * error in observed in the host system.
	 */
	usleep_range(1000, 2000);

unmap:
	if (map_size)
		pci_epc_mem_unmap(epc, epf->func_no, epf->vfunc_no, &map);

free_buf:
	kfree(dst_buf);

set_status:
	if (!ret)
		status |= STATUS_WRITE_SUCCESS;
	else
		status |= STATUS_WRITE_FAIL;
	reg->status = cpu_to_le32(status);
}

static int epf_read_edid_sysfs(u8 *buf)
{
	struct file *filp;
	loff_t pos = 0;
	ssize_t ret;

	filp = filp_open(EDID_PATH, O_RDONLY, 0);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	ret = kernel_read(filp, buf, EDID_SIZE, &pos);
	filp_close(filp, NULL);

	if (ret != EDID_SIZE)
		return -EINVAL;

	return 0;
}

static void epf_copy_edid_to_bar1(struct pci_epf_test *epf_test)
{
	u8 *bar1;
	int ret;
	u8 edid[EDID_SIZE];

	if (!epf_test->reg[BAR_1]) {
		pr_err("BAR1 not mapped\n");
		return;
	}

	bar1 = (u8 *)epf_test->reg[BAR_1];

	ret = epf_read_edid_sysfs(edid);
	if (ret) {
		pr_warn("EDID missing using fallback with max 4k\n");
		memcpy(edid, default_edid, EDID_SIZE);
	} else {
		pr_info("EDID read from sysfs with max connected display resolution.\n");
	}

	memcpy(bar1, edid, EDID_SIZE);
	pr_info("EDID copied to BAR1\n");
}

static int pci_epf_load_option_rom(struct pci_epf *epf)
{
	const struct firmware *fw;
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	void *rom_base;
	size_t rom_size;
	int ret;

	dev_info(&epf->dev, "Loading PCIe Option ROM\n");

	ret = request_firmware(&fw, "OptionRom.rom", &epf->dev);
	if (ret) {
		dev_err(&epf->dev, "Failed to load Option ROM (%d)\n", ret);
		return ret;
	}

	dev_info(&epf->dev, "Option ROM size: %zu\n", fw->size);

	/* PCI spec requires ROM BAR window >= 64KB */
	rom_size = max_t(size_t, roundup_pow_of_two(fw->size), 65536);

	memset(&epf_test->rom_bar, 0, sizeof(epf_test->rom_bar));

	epf_test->rom_bar.barno = (enum pci_barno)PCI_ROM_RESOURCE;
	epf_test->rom_bar.flags = PCI_BASE_ADDRESS_SPACE_MEMORY |
				  PCI_BASE_ADDRESS_MEM_TYPE_32;
	epf_test->rom_bar.size = rom_size;

	/* Allocate DMA-safe memory accessible by PCIe */
	rom_base = dma_alloc_coherent(epf->epc->dev.parent,
				      rom_size,
				      &epf_test->rom_bar.phys_addr,
				      GFP_KERNEL);
	if (!rom_base) {
		dev_err(&epf->dev, "Failed to allocate ROM memory\n");
		release_firmware(fw);
		return -ENOMEM;
	}

	/* Copy firmware image into allocated memory */
	memcpy(rom_base, fw->data, fw->size);

	epf_test->rom_bar.addr = rom_base;

	dev_info(&epf->dev,
		 "Option ROM mapped: virt=%p phys=%pa size=%zu\n",
		 rom_base, &epf_test->rom_bar.phys_addr, rom_size);

	/* Configure ROM BAR in endpoint controller */
	ret = pci_epc_set_bar(epf->epc, epf->func_no, epf->vfunc_no,
			      &epf_test->rom_bar);
	if (ret) {
		dev_err(&epf->dev, "Failed to configure ROM BAR (%d)\n", ret);
		dma_free_coherent(epf->epc->dev.parent, rom_size, rom_base,
				  epf_test->rom_bar.phys_addr);
		release_firmware(fw);
		return ret;
	}

	dev_info(&epf->dev, "ROM BAR configured successfully (%zu bytes)\n",
		 rom_size);

	release_firmware(fw);

	return 0;
}

static void pci_epf_test_raise_irq(struct pci_epf_test *epf_test,
				   struct pci_epf_test_reg *reg)
{
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	u32 status = le32_to_cpu(reg->status);
	u32 irq_number = le32_to_cpu(reg->irq_number);
	u32 irq_type = le32_to_cpu(reg->irq_type);
	int count;

	/*
	 * Set the status before raising the IRQ to ensure that the host sees
	 * the updated value when it gets the IRQ.
	 */
	status |= STATUS_IRQ_RAISED;
	WRITE_ONCE(reg->status, cpu_to_le32(status));

	switch (irq_type) {
	case IRQ_TYPE_INTX:
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_INTX, 0);
		break;
	case IRQ_TYPE_MSI:
		count = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSI IRQ number %d / %d\n",
				irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_MSI, irq_number);
		break;
	case IRQ_TYPE_MSIX:
		count = pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no);
		if (irq_number > count || count <= 0) {
			dev_err(dev, "Invalid MSI-X IRQ number %d / %d\n",
				irq_number, count);
			return;
		}
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_IRQ_MSIX, irq_number);
		break;
	default:
		dev_err(dev, "Failed to raise IRQ, unknown type\n");
		break;
	}
}

static irqreturn_t pci_epf_test_doorbell_handler(int irq, void *data)
{
	struct pci_epf_test *epf_test = data;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
	u32 status = le32_to_cpu(reg->status);

	status |= STATUS_DOORBELL_SUCCESS;
	reg->status = cpu_to_le32(status);
	pci_epf_test_raise_irq(epf_test, reg);

	return IRQ_HANDLED;
}

static void pci_epf_test_doorbell_cleanup(struct pci_epf_test *epf_test)
{
	struct pci_epf_test_reg *reg = epf_test->reg[epf_test->test_reg_bar];
	struct pci_epf *epf = epf_test->epf;

	free_irq(epf->db_msg[0].virq, epf_test);
	reg->doorbell_bar = cpu_to_le32(NO_BAR);

	pci_epf_free_doorbell(epf);
}

static void pci_epf_test_enable_doorbell(struct pci_epf_test *epf_test,
					 struct pci_epf_test_reg *reg)
{
	u32 status = le32_to_cpu(reg->status);
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	struct msi_msg *msg;
	enum pci_barno bar;
	size_t offset;
	int ret;

	ret = pci_epf_alloc_doorbell(epf, 1);
	if (ret)
		goto set_status_err;

	msg = &epf->db_msg[0].msg;
	bar = pci_epc_get_next_free_bar(epf_test->epc_features,
					epf_test->test_reg_bar + 1);
	if (bar < BAR_0)
		goto err_doorbell_cleanup;

	ret = request_threaded_irq(epf->db_msg[0].virq, NULL,
				   pci_epf_test_doorbell_handler, IRQF_ONESHOT,
				   "pci-ep-test-doorbell", epf_test);
	if (ret) {
		dev_err(&epf->dev,
			"Failed to request doorbell IRQ: %d\n",
			epf->db_msg[0].virq);
		goto err_doorbell_cleanup;
	}

	reg->doorbell_data = cpu_to_le32(msg->data);
	reg->doorbell_bar = cpu_to_le32(bar);

	msg = &epf->db_msg[0].msg;
	ret = pci_epf_align_inbound_addr(epf, bar,
					 ((u64)msg->address_hi << 32) | msg->address_lo,
					 &epf_test->db_bar.phys_addr, &offset);
	if (ret)
		goto err_doorbell_cleanup;

	reg->doorbell_offset = cpu_to_le32(offset);

	epf_test->db_bar.barno = bar;
	epf_test->db_bar.size = epf->bar[bar].size;
	epf_test->db_bar.flags = epf->bar[bar].flags;

	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
			      &epf_test->db_bar);
	if (ret)
		goto err_doorbell_cleanup;

	status |= STATUS_DOORBELL_ENABLE_SUCCESS;
	reg->status = cpu_to_le32(status);
	return;

err_doorbell_cleanup:
	pci_epf_test_doorbell_cleanup(epf_test);
set_status_err:
	status |= STATUS_DOORBELL_ENABLE_FAIL;
	reg->status = cpu_to_le32(status);
}

static void pci_epf_test_disable_doorbell(struct pci_epf_test *epf_test,
					  struct pci_epf_test_reg *reg)
{
	enum pci_barno bar = le32_to_cpu(reg->doorbell_bar);
	u32 status = le32_to_cpu(reg->status);
	struct pci_epf *epf = epf_test->epf;
	struct pci_epc *epc = epf->epc;
	int ret;

	if (bar < BAR_0)
		goto set_status_err;

	pci_epf_test_doorbell_cleanup(epf_test);

	/*
	 * The doorbell feature temporarily overrides the inbound translation
	 * to point to the address stored in epf_test->db_bar.phys_addr, i.e.,
	 * it calls set_bar() twice without ever calling clear_bar(), as
	 * calling clear_bar() would clear the BAR's PCI address assigned by
	 * the host. Thus, when disabling the doorbell, restore the inbound
	 * translation to point to the memory allocated for the BAR.
	 */
	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
			      &epf->bar[bar]);
	if (ret)
		goto set_status_err;

	status |= STATUS_DOORBELL_DISABLE_SUCCESS;
	reg->status = cpu_to_le32(status);

	return;

set_status_err:
	status |= STATUS_DOORBELL_DISABLE_FAIL;
	reg->status = cpu_to_le32(status);
}

static void pci_epf_test_cmd_handler(struct work_struct *work)
{
	u32 command;
	struct pci_epf_test *epf_test = container_of(work, struct pci_epf_test,
						     cmd_handler.work);
	struct pci_epf *epf = epf_test->epf;
	struct device *dev = &epf->dev;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
	u32 irq_type = le32_to_cpu(reg->irq_type);

	command = le32_to_cpu(READ_ONCE(reg->command));
	if (!command)
		goto reset_handler;

	WRITE_ONCE(reg->command, 0);
	WRITE_ONCE(reg->status, 0);

	if ((le32_to_cpu(READ_ONCE(reg->flags)) & FLAG_USE_DMA) &&
	    !epf_test->dma_supported) {
		dev_err(dev, "Cannot transfer data using DMA\n");
		goto reset_handler;
	}

	if (irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	switch (command) {
	case COMMAND_RAISE_INTX_IRQ:
	case COMMAND_RAISE_MSI_IRQ:
	case COMMAND_RAISE_MSIX_IRQ:
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_WRITE:
		pci_epf_test_write(epf_test, reg);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_READ:
		pci_epf_test_read(epf_test, reg);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_COPY:
		pci_epf_test_copy(epf_test, reg);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_ENABLE_DOORBELL:
		pci_epf_test_enable_doorbell(epf_test, reg);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_DISABLE_DOORBELL:
		pci_epf_test_disable_doorbell(epf_test, reg);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	case COMMAND_CHANGE_RESOLUTION:
	{
		char *envp[3];
		char res_str[32];
		u32 width = le32_to_cpu(reg->width);
		u32 height = le32_to_cpu(reg->height);

		dev_info(dev, "Resolution change: %ux%u\n", width, height);

		snprintf(res_str, sizeof(res_str), "RESOLUTION=%ux%u",
			 width, height);
		envp[0] = "EVENT=RESOLUTION_CHANGE";
		envp[1] = res_str;
		envp[2] = NULL;

		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	}
	case COMMAND_GET_FB_OFFSET:
		reg->dpu_fb_offset = cpu_to_le64(vram_fb_offset);
		pci_epf_test_raise_irq(epf_test, reg);
		break;
	default:
		dev_err(dev, "Invalid command 0x%x\n", command);
		break;
	}

reset_handler:
	queue_delayed_work(kpcitest_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));
}

static int pci_epf_test_set_bar(struct pci_epf *epf)
{
	int bar, ret;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
				      &epf->bar[bar]);
		if (ret) {
			pci_epf_free_space(epf, epf_test->reg[bar], bar,
					   PRIMARY_INTERFACE);
			epf_test->reg[bar] = NULL;
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == test_reg_bar)
				return ret;
		}
	}

	return 0;
}

static void pci_epf_test_clear_bar(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
				  &epf->bar[bar]);
	}
}

static void pci_epf_test_set_capabilities(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	struct pci_epf_test_reg *reg = epf_test->reg[test_reg_bar];
	struct pci_epc *epc = epf->epc;
	u32 caps = 0;

	if (epc->ops->align_addr)
		caps |= CAP_UNALIGNED_ACCESS;

	if (epf_test->epc_features->msi_capable)
		caps |= CAP_MSI;

	if (epf_test->epc_features->msix_capable)
		caps |= CAP_MSIX;

	if (epf_test->epc_features->intx_capable)
		caps |= CAP_INTX;

	reg->caps = cpu_to_le32(caps);
}

static int pci_epf_test_epc_init(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features = epf_test->epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool linkup_notifier = false;
	int ret;

	epf_test->dma_supported = true;

	ret = pci_epf_test_init_dma_chan(epf_test);
	if (ret)
		epf_test->dma_supported = false;

	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no,
					   header);
		if (ret) {
			dev_err(dev, "Configuration header write failed\n");
			return ret;
		}
	}

	pci_epf_test_set_capabilities(epf);

	ret = pci_epf_test_set_bar(epf);
	if (ret)
		return ret;

	if (epc_features->msi_capable) {
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (epc_features->msix_capable) {
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       epf_test->test_reg_bar,
				       epf_test->msix_table_offset);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	ret = pci_epf_load_option_rom(epf);
	if (ret)
		dev_err(&epf->dev, "Failed to expose Option ROM\n");

	epf_copy_edid_to_bar1(epf_test);

	linkup_notifier = epc_features->linkup_notifier;
	if (!linkup_notifier)
		queue_work(kpcitest_workqueue, &epf_test->cmd_handler.work);

	return 0;
}

static void pci_epf_test_epc_deinit(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_test->cmd_handler);
	pci_epf_test_clean_dma_chan(epf_test);
	pci_epf_test_clear_bar(epf);
}

static int pci_epf_test_link_up(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);

	queue_delayed_work(kpcitest_workqueue, &epf_test->cmd_handler,
			   msecs_to_jiffies(1));

	return 0;
}

static int pci_epf_test_link_down(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);

	cancel_delayed_work_sync(&epf_test->cmd_handler);

	return 0;
}

static const struct pci_epc_event_ops pci_epf_test_event_ops = {
	.epc_init = pci_epf_test_epc_init,
	.epc_deinit = pci_epf_test_epc_deinit,
	.link_up = pci_epf_test_link_up,
	.link_down = pci_epf_test_link_down,
};

static int pci_epf_test_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	size_t msix_table_size = 0;
	size_t test_reg_bar_size;
	size_t pba_size = 0;
	void *base;
	enum pci_barno test_reg_bar = epf_test->test_reg_bar;
	enum pci_barno bar;
	const struct pci_epc_features *epc_features = epf_test->epc_features;
	size_t test_reg_size;

	test_reg_bar_size = ALIGN(sizeof(struct pci_epf_test_reg), 128);

	if (epc_features->msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_test->msix_table_offset = test_reg_bar_size;
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	test_reg_size = test_reg_bar_size + msix_table_size + pba_size;

	base = pci_epf_alloc_space(epf, test_reg_size, test_reg_bar,
				   epc_features, PRIMARY_INTERFACE);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}
	epf_test->reg[test_reg_bar] = base;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		bar = pci_epc_get_next_free_bar(epc_features, bar);
		if (bar == NO_BAR)
			break;

		if (bar == test_reg_bar)
			continue;

		if (epc_features->bar[bar].type == BAR_FIXED)
			test_reg_size = epc_features->bar[bar].fixed_size;
		else
			test_reg_size = bar_size[bar];

		base = pci_epf_alloc_space(epf, test_reg_size, bar,
					   epc_features, PRIMARY_INTERFACE);
		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n",
				bar);
		epf_test->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_test_free_space(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	int bar;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_test->reg[bar])
			continue;

		pci_epf_free_space(epf, epf_test->reg[bar], bar,
				   PRIMARY_INTERFACE);
		epf_test->reg[bar] = NULL;
	}
}

static int pci_epf_test_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno test_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	test_reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (test_reg_bar < 0)
		return -EINVAL;

	epf_test->test_reg_bar = test_reg_bar;
	epf_test->epc_features = epc_features;

	ret = pci_epf_test_alloc_space(epf);
	if (ret)
		return ret;

	return 0;
}

static void pci_epf_test_unbind(struct pci_epf *epf)
{
	struct pci_epf_test *epf_test = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	cancel_delayed_work_sync(&epf_test->cmd_handler);
	if (epc->init_complete) {
		pci_epf_test_clean_dma_chan(epf_test);
		pci_epf_test_clear_bar(epf);
	}
	pci_epf_test_free_space(epf);
}

static const struct pci_epf_device_id pci_epf_test_ids[] = {
	{
		.name = "pci_epf_test",
	},
	{},
};

static int pci_epf_test_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_test *epf_test;
	struct device *dev = &epf->dev;

	epf_test = devm_kzalloc(dev, sizeof(*epf_test), GFP_KERNEL);
	if (!epf_test)
		return -ENOMEM;

	epf->header = &test_header;
	epf_test->epf = epf;

	INIT_DELAYED_WORK(&epf_test->cmd_handler, pci_epf_test_cmd_handler);

	epf->event_ops = &pci_epf_test_event_ops;

	epf_set_drvdata(epf, epf_test);
	return 0;
}

/* ---- VRAM character device: mmap + DMA-BUF export ---- */

static int stream_init_vram(void)
{
	struct device_node *vram_node;
	struct resource res;

	vram_node = of_find_node_by_name(NULL, "vram");
	if (!vram_node) {
		pr_err("VRAM node not found\n");
		return -ENODEV;
	}

	if (of_address_to_resource(vram_node, 0, &res)) {
		pr_err("Failed to get VRAM resource\n");
		return -EINVAL;
	}

	vram_phys = res.start;
	vram_size = resource_size(&res);

	pr_info("VRAM: phys=%pa size=0x%zx\n", &vram_phys, vram_size);

	return 0;
}

static int stream_attach(struct dma_buf *dmabuf,
			 struct dma_buf_attachment *attach)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void stream_detach(struct dma_buf *dmabuf,
			  struct dma_buf_attachment *attach)
{
	pr_info("%s\n", __func__);
}

static struct sg_table *stream_map_dma_buf(struct dma_buf_attachment *attach,
					   enum dma_data_direction dir)
{
	struct stream_dmabuf *buf = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	pr_info("%s\n", __func__);
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	sg_set_page(sgt->sgl, ZERO_PAGE(0), buf->size, 0);
	sg_dma_address(sgt->sgl) = buf->phys;
	sg_dma_len(sgt->sgl) = buf->size;

	ret = dma_map_sg(attach->dev, sgt->sgl, 1, dir);
	if (ret <= 0) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(-EIO);
	}

	return sgt;
}

static void stream_unmap_dma_buf(struct dma_buf_attachment *attach,
				 struct sg_table *sgt,
				 enum dma_data_direction dir)
{
	pr_info("%s\n", __func__);
	dma_unmap_sg(attach->dev, sgt->sgl, 1, dir);
	sg_free_table(sgt);
	kfree(sgt);
}

static void stream_dma_buf_release(struct dma_buf *dmabuf)
{
	struct stream_dmabuf *buf = dmabuf->priv;

	pr_info("%s\n", __func__);
	kfree(buf);
}

static int stream_dmabuf_mmap(struct dma_buf *dmabuf,
			      struct vm_area_struct *vma)
{
	struct stream_dmabuf *buf = dmabuf->priv;

	pr_info("%s\n", __func__);
	if ((vma->vm_end - vma->vm_start) > buf->size)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return remap_pfn_range(vma,
			       vma->vm_start,
			       buf->phys >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

static const struct dma_buf_ops stream_dmabuf_ops = {
	.attach        = stream_attach,
	.detach        = stream_detach,
	.map_dma_buf   = stream_map_dma_buf,
	.unmap_dma_buf = stream_unmap_dma_buf,
	.mmap          = stream_dmabuf_mmap,
	.release       = stream_dma_buf_release,
};

static struct dma_buf *stream_export_dmabuf(struct device *dev)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp);
	struct stream_dmabuf *buf;

	pr_info("%s\n", __func__);

	if (!vram_phys || !vram_size)
		return ERR_PTR(-EINVAL);

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->phys = vram_phys;
	buf->size = vram_size;
	buf->dev  = dev;

	exp.ops   = &stream_dmabuf_ops;
	exp.size  = buf->size;
	exp.flags = O_RDWR;
	exp.priv  = buf;

	return dma_buf_export(&exp);
}

static long stream_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dma_buf *dmabuf;
	int fd;
	long ret = 0;

	pr_info("%s   %d\n", __func__, cmd);
	switch (cmd) {

	case STREAM_IOC_GET_DMABUF:
		pr_info("STREAM_IOC_GET_DMABUF\n");
		dmabuf = stream_export_dmabuf(file->private_data);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);

		fd = dma_buf_fd(dmabuf, O_CLOEXEC);
		if (fd < 0)
			return fd;

		if (copy_to_user((int __user *)arg, &fd, sizeof(fd)))
			return -EFAULT;

		ret = 0;
		break;

	case STREAM_IOC_SET_OFFSET:
	{
		uint64_t offset;

		if (copy_from_user(&offset, (void __user *)arg, sizeof(offset)))
			return -EFAULT;

		if (offset != 0)
			vram_fb_offset = offset - vram_phys;
		else
			vram_fb_offset = 0;

		pr_info("VRAM offset = 0x%016llx\n", vram_fb_offset);
		ret = 0;
		break;
	}

	default:
		pr_err("%s No ioctl found\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static int stream_mmap_pci(struct file *filp, struct vm_area_struct *vma)
{
	phys_addr_t map_phys;
	unsigned long map_size;

	map_size = vma->vm_end - vma->vm_start;
	map_phys = vram_phys;

	if (map_size > vram_size) {
		pr_err("%s: invalid size req=%lu > vram=%zu\n",
		       __func__, map_size, vram_size);
		return -EINVAL;
	}

	if (vram_fb_offset) {
		map_phys += vram_fb_offset;
		pr_info("%s: applying fb offset -> map_phys=0x%pa\n",
			__func__, &map_phys);
	} else {
		pr_info("%s: no fb offset, using BAR base\n", __func__);
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	pr_info("%s: mmap start=0x%lx end=0x%lx phys=0x%pa size=%lu\n",
		__func__, vma->vm_start, vma->vm_end, &map_phys, map_size);

	return io_remap_pfn_range(vma,
				  vma->vm_start,
				  map_phys >> PAGE_SHIFT,
				  map_size,
				  vma->vm_page_prot);
}

static int stream_open_pci(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations stream_fops = {
	.owner          = THIS_MODULE,
	.open           = stream_open_pci,
	.mmap           = stream_mmap_pci,
	.unlocked_ioctl = stream_ioctl,
};

static const struct pci_epf_ops ops = {
	.unbind	= pci_epf_test_unbind,
	.bind	= pci_epf_test_bind,
};

static struct pci_epf_driver test_driver = {
	.driver.name	= "pci_epf_test",
	.probe		= pci_epf_test_probe,
	.id_table	= pci_epf_test_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_test_init(void)
{
	int ret;

	ret = stream_init_vram();
	if (ret)
		return ret;

	kpcitest_workqueue = alloc_workqueue("kpcitest",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpcitest_workqueue) {
		pr_err("Failed to allocate the kpcitest work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&test_driver);
	if (ret) {
		destroy_workqueue(kpcitest_workqueue);
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	alloc_chrdev_region(&stream_dev, 0, 1, "stream");
	cdev_init(&stream_cdev, &stream_fops);
	cdev_add(&stream_cdev, stream_dev, 1);

	stream_class = class_create("pcie_stream");
	if (IS_ERR(stream_class))
		return PTR_ERR(stream_class);

	device_create(stream_class, NULL, stream_dev, NULL, "stream");

	return 0;
}
module_init(pci_epf_test_init);

static void __exit pci_epf_test_exit(void)
{
	if (kpcitest_workqueue)
		destroy_workqueue(kpcitest_workqueue);

	device_destroy(stream_class, stream_dev);
	class_destroy(stream_class);
	cdev_del(&stream_cdev);
	unregister_chrdev_region(stream_dev, 1);

	pci_epf_unregister_driver(&test_driver);
}
module_exit(pci_epf_test_exit);

MODULE_DESCRIPTION("PCI EPF TEST DRIVER");
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
