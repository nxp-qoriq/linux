/*
 * drivers/dma/fsl-qdma.c
 *
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 *
 * Driver for the Freescale qDMA engine with software command queue mode.
 * Channel virtualization is supported through enqueuing of DMA jobs to,
 * or dequeuing DMA jobs from, different work queues.
 * This module can be found on Freescale LS SoCs.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "virt-dma.h"

#define FSL_QDMA_DMR			0x0
#define FSL_QDMA_DSR			0x4
#define FSL_QDMA_DEIER			0xe00
#define FSL_QDMA_DEDR			0xe04
#define FSL_QDMA_DECFDW0R		0xe10
#define FSL_QDMA_DECFDW1R		0xe14
#define FSL_QDMA_DECFDW2R		0xe18
#define FSL_QDMA_DECFDW3R		0xe1c
#define FSL_QDMA_DECFQIDR		0xe30
#define FSL_QDMA_DECBR			0xe34

#define FSL_QDMA_BCQMR(x)		(0xc0 + 0x100 * (x))
#define FSL_QDMA_BCQSR(x)		(0xc4 + 0x100 * (x))
#define FSL_QDMA_BCQEDPA_SADDR(x)	(0xc8 + 0x100 * (x))
#define FSL_QDMA_BCQDPA_SADDR(x)	(0xcc + 0x100 * (x))
#define FSL_QDMA_BCQEEPA_SADDR(x)	(0xd0 + 0x100 * (x))
#define FSL_QDMA_BCQEPA_SADDR(x)	(0xd4 + 0x100 * (x))
#define FSL_QDMA_BCQIER(x)		(0xe0 + 0x100 * (x))
#define FSL_QDMA_BCQIDR(x)		(0xe4 + 0x100 * (x))

#define FSL_QDMA_SQDPAR			0x80c
#define FSL_QDMA_SQEPAR			0x814
#define FSL_QDMA_BSQMR			0x800
#define FSL_QDMA_BSQSR			0x804
#define FSL_QDMA_BSQICR			0x828
#define FSL_QDMA_CQMR			0xa00
#define FSL_QDMA_CQDSCR1		0xa08
#define FSL_QDMA_CQDSCR2                0xa0c
#define FSL_QDMA_CQIER			0xa10
#define FSL_QDMA_CQEDR			0xa14
#define FSL_QDMA_SQCCMR			0xa20

#define FSL_QDMA_SQICR_ICEN

#define FSL_QDMA_CQIDR_CQT		0xff000000
#define FSL_QDMA_CQIDR_SQPE		0x800000
#define FSL_QDMA_CQIDR_SQT		0x8000

#define FSL_QDMA_BCQIER_CQTIE		0x8000
#define FSL_QDMA_BCQIER_CQPEIE		0x800000
#define FSL_QDMA_BSQICR_ICEN		0x80000000
#define FSL_QDMA_BSQICR_ICST(x)		((x) << 16)
#define FSL_QDMA_CQIER_MEIE		0x80000000
#define FSL_QDMA_CQIER_TEIE		0x1
#define FSL_QDMA_SQCCMR_ENTER_WM	0x200000

#define FSL_QDMA_QUEUE_MAX		8

#define FSL_QDMA_BCQMR_EN		0x80000000
#define FSL_QDMA_BCQMR_EI		0x40000000
#define FSL_QDMA_BCQMR_CD_THLD(x)	((x) << 20)
#define FSL_QDMA_BCQMR_CQ_SIZE(x)	((x) << 16)

#define FSL_QDMA_BCQSR_QF		0x10000
#define FSL_QDMA_BCQSR_XOFF		0x1

#define FSL_QDMA_BSQMR_EN		0x80000000
#define FSL_QDMA_BSQMR_DI		0x40000000
#define FSL_QDMA_BSQMR_CQ_SIZE(x)	((x) << 16)

#define FSL_QDMA_BSQSR_QE		0x20000

#define FSL_QDMA_DMR_DQD		0x40000000
#define FSL_QDMA_DSR_DB			0x80000000

#define FSL_QDMA_BASE_BUFFER_SIZE	96
#define FSL_QDMA_EXPECT_SG_ENTRY_NUM	16
#define FSL_QDMA_CIRCULAR_DESC_SIZE_MIN	64
#define FSL_QDMA_CIRCULAR_DESC_SIZE_MAX	16384
#define FSL_QDMA_QUEUE_NUM_MAX		8

#define FSL_QDMA_CMD_RWTTYPE		0x4
#define FSL_QDMA_CMD_LWC                0x2

#define FSL_QDMA_CMD_RWTTYPE_OFFSET	28
#define FSL_QDMA_CMD_NS_OFFSET		27
#define FSL_QDMA_CMD_DQOS_OFFSET	24
#define FSL_QDMA_CMD_WTHROTL_OFFSET	20
#define FSL_QDMA_CMD_DSEN_OFFSET	19
#define FSL_QDMA_CMD_LWC_OFFSET		16

#define FSL_QDMA_E_SG_TABLE		1
#define FSL_QDMA_E_DATA_BUFFER		0
#define FSL_QDMA_F_LAST_ENTRY		1

u64 pre_addr, pre_queue;

struct fsl_qdma_ccdf {
	u8 status;
	u32 rev1:22;
	u32 ser:1;
	u32 rev2:1;
	u32 rev3:20;
	u32 offset:9;
	u32 format:3;
	union {
		struct {
			u32 addr_lo;	/* low 32-bits of 40-bit address */
			u32 addr_hi:8;	/* high 8-bits of 40-bit address */
			u32 rev4:16;
			u32 queue:3;
			u32 rev5:3;
			u32 dd:2;	/* dynamic debug */
		};
		struct {
			u64 addr:40;
			/* More efficient address accessor */
			u64 __notaddress:24;
		};
	};
} __packed;

struct fsl_qdma_csgf {
	u32 offset:13;
	u32 rev1:19;
	u32 length:30;
	u32 f:1;
	u32 e:1;
	union {
		struct {
			u32 addr_lo;	/* low 32-bits of 40-bit address */
			u32 addr_hi:8;	/* high 8-bits of 40-bit address */
			u32 rev2:24;
		};
		struct {
			u64 addr:40;
			/* More efficient address accessor */
			u64 __notaddress:24;
		};
	};
} __packed;

struct fsl_qdma_sdf {
	u32 rev3:32;
	u32 ssd:12;	/* souce stride distance */
	u32 sss:12;	/* souce stride size */
	u32 rev4:8;
	u32 rev5:32;
	u32 cmd;
} __packed;

struct fsl_qdma_ddf {
	u32 rev1:32;
	u32 dsd:12;	/* Destination stride distance */
	u32 dss:12;	/* Destination stride size */
	u32 rev2:8;
	u32 rev3:32;
	u32 cmd;
} __packed;

struct fsl_qdma_chan {
	struct virt_dma_chan		vchan;
	struct virt_dma_desc		vdesc;
	enum dma_status			status;
	u32				slave_id;
	struct fsl_qdma_engine		*qdma;
	struct fsl_qdma_queue		*queue;
	struct list_head		qcomp;
};

struct fsl_qdma_queue {
	struct fsl_qdma_ccdf	*virt_head;
	struct fsl_qdma_ccdf	*virt_tail;
	struct list_head	comp_used;
	struct list_head	comp_free;
	struct dma_pool		*comp_pool;
	struct dma_pool		*sg_pool;
	spinlock_t		queue_lock;
	dma_addr_t		bus_addr;
	u32                     n_cq;
	u32			id;
	struct fsl_qdma_ccdf	*cq;
};

struct fsl_qdma_sg {
	dma_addr_t		bus_addr;
	void			*virt_addr;
};

struct fsl_qdma_comp {
	dma_addr_t              bus_addr;
	void			*virt_addr;
	struct fsl_qdma_chan	*qchan;
	struct fsl_qdma_sg	*sg_block;
	struct virt_dma_desc    vdesc;
	struct list_head	list;
	u32			sg_block_src;
	u32			sg_block_dst;
};

struct fsl_qdma_engine {
	struct dma_device	dma_dev;
	void __iomem		*ctrl_base;
	void __iomem            *status_base;
	void __iomem		*block_base;
	u32			n_chans;
	u32			n_queues;
	struct mutex            fsl_qdma_mutex;
	int			error_irq;
	int			queue_irq;
	bool			big_endian;
	struct fsl_qdma_queue	*queue;
	struct fsl_qdma_queue	*status;
	struct fsl_qdma_chan	chans[];

};

static u32 qdma_readl(struct fsl_qdma_engine *qdma, void __iomem *addr)
{
	if (qdma->big_endian)
		return ioread32be(addr);
	else
		return ioread32(addr);
}

static void qdma_writel(struct fsl_qdma_engine *qdma, u32 val,
						void __iomem *addr)
{
	if (qdma->big_endian)
		iowrite32be(val, addr);
	else
		iowrite32(val, addr);
}

static struct fsl_qdma_chan *to_fsl_qdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct fsl_qdma_chan, vchan.chan);
}

static struct fsl_qdma_comp *to_fsl_qdma_comp(struct virt_dma_desc *vd)
{
	return container_of(vd, struct fsl_qdma_comp, vdesc);
}

static int fsl_qdma_alloc_chan_resources(struct dma_chan *chan)
{
	/*
	 * In QDMA mode, We don't need to do anything.
	 */
	return 0;
}

static void fsl_qdma_free_chan_resources(struct dma_chan *chan)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&fsl_chan->vchan.lock, flags);
	vchan_get_all_descriptors(&fsl_chan->vchan, &head);
	spin_unlock_irqrestore(&fsl_chan->vchan.lock, flags);

	vchan_dma_desc_free_list(&fsl_chan->vchan, &head);
}

static void fsl_qdma_comp_fill_memcpy(struct fsl_qdma_comp *fsl_comp,
					dma_addr_t dst, dma_addr_t src, u32 len)
{
	struct fsl_qdma_ccdf *ccdf;
	struct fsl_qdma_csgf *csgf_desc, *csgf_src, *csgf_dest;
	struct fsl_qdma_sdf *sdf;
	struct fsl_qdma_ddf *ddf;

	ccdf = (struct fsl_qdma_ccdf *)fsl_comp->virt_addr;
	csgf_desc = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 1;
	csgf_src = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 2;
	csgf_dest = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 3;
	sdf = (struct fsl_qdma_sdf *)fsl_comp->virt_addr + 4;
	ddf = (struct fsl_qdma_ddf *)fsl_comp->virt_addr + 5;

	memset(fsl_comp->virt_addr, 0, FSL_QDMA_BASE_BUFFER_SIZE);
	/* Head Command Descriptor(Frame Descriptor) */
	ccdf->addr = fsl_comp->bus_addr + 16;
	ccdf->format = 1; /* Compound S/G format */
	/* Status notification is enqueued to status queue. */
	ccdf->ser = 1;
	/* Compound Command Descriptor(Frame List Table) */
	csgf_desc->addr = fsl_comp->bus_addr + 64;
	/* It must be 32 as Compound S/G Descriptor */
	csgf_desc->length = 32;
	csgf_src->addr = src;
	csgf_src->length = len;
	csgf_dest->addr = dst;
	csgf_dest->length = len;
	/* This entry is the last entry. */
	csgf_dest->f = FSL_QDMA_F_LAST_ENTRY;
	/* Descriptor Buffer */
	sdf->cmd = FSL_QDMA_CMD_RWTTYPE << FSL_QDMA_CMD_RWTTYPE_OFFSET;
	ddf->cmd = FSL_QDMA_CMD_RWTTYPE << FSL_QDMA_CMD_RWTTYPE_OFFSET;
	ddf->cmd |= FSL_QDMA_CMD_LWC << FSL_QDMA_CMD_LWC_OFFSET;
}

static void fsl_qdma_comp_fill_sg(
		struct fsl_qdma_comp *fsl_comp,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents)
{
	struct fsl_qdma_ccdf *ccdf;
	struct fsl_qdma_csgf *csgf_desc, *csgf_src, *csgf_dest, *csgf_sg;
	struct fsl_qdma_sdf *sdf;
	struct fsl_qdma_ddf *ddf;
	struct fsl_qdma_sg *sg_block, *temp;
	struct scatterlist *sg;
	u64 total_src_len = 0;
	u64 total_dst_len = 0;
	u32 i;

	ccdf = (struct fsl_qdma_ccdf *)fsl_comp->virt_addr;
	csgf_desc = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 1;
	csgf_src = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 2;
	csgf_dest = (struct fsl_qdma_csgf *)fsl_comp->virt_addr + 3;
	sdf = (struct fsl_qdma_sdf *)fsl_comp->virt_addr + 4;
	ddf = (struct fsl_qdma_ddf *)fsl_comp->virt_addr + 5;

	memset(fsl_comp->virt_addr, 0, FSL_QDMA_BASE_BUFFER_SIZE);
	/* Head Command Descriptor(Frame Descriptor) */
	ccdf->addr = fsl_comp->bus_addr + 16;
	ccdf->format = 1; /* Compound S/G format */
	/* Status notification is enqueued to status queue. */
	ccdf->ser = 1;

	/* Compound Command Descriptor(Frame List Table) */
	csgf_desc->addr = fsl_comp->bus_addr + 64;
	/* It must be 32 as Compound S/G Descriptor */
	csgf_desc->length = 32;

	sg_block = fsl_comp->sg_block;
	csgf_src->addr = sg_block->bus_addr;
	/* This entry link to the s/g entry. */
	csgf_src->e = FSL_QDMA_E_SG_TABLE;

	temp = sg_block + fsl_comp->sg_block_src;
	csgf_dest->addr = temp->bus_addr;
	/* This entry is the last entry. */
	csgf_dest->f = FSL_QDMA_F_LAST_ENTRY;
	/* This entry link to the s/g entry. */
	csgf_dest->e = FSL_QDMA_E_SG_TABLE;

	for_each_sg(src_sg, sg, src_nents, i) {
		temp = sg_block + i / (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1);
		csgf_sg = (struct fsl_qdma_csgf *)temp->virt_addr +
			  i % (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1);
		csgf_sg->addr = sg_dma_address(sg);
		csgf_sg->length = sg_dma_len(sg);
		total_src_len += sg_dma_len(sg);

		if (i == src_nents - 1)
			csgf_sg->f = FSL_QDMA_F_LAST_ENTRY;
		if (i % (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) ==
		    FSL_QDMA_EXPECT_SG_ENTRY_NUM - 2) {
			csgf_sg = (struct fsl_qdma_csgf *)temp->virt_addr +
				  FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1;
			temp = sg_block +
				i / (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) + 1;
			csgf_sg->addr = temp->bus_addr;
			csgf_sg->e = FSL_QDMA_E_SG_TABLE;
		}
	}

	sg_block += fsl_comp->sg_block_src;
	for_each_sg(dst_sg, sg, dst_nents, i) {
		temp = sg_block + i / (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1);
		csgf_sg = (struct fsl_qdma_csgf *)temp->virt_addr +
			  i % (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1);
		csgf_sg->addr = sg_dma_address(sg);
		csgf_sg->length = sg_dma_len(sg);
		total_dst_len += sg_dma_len(sg);

		if (i == dst_nents - 1)
			csgf_sg->f = FSL_QDMA_F_LAST_ENTRY;
		if (i % (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) ==
		    FSL_QDMA_EXPECT_SG_ENTRY_NUM - 2) {
			csgf_sg = (struct fsl_qdma_csgf *)temp->virt_addr +
				  FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1;
			temp = sg_block +
				i / (FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) + 1;
			csgf_sg->addr = temp->bus_addr;
			csgf_sg->e = FSL_QDMA_E_SG_TABLE;
		}
	}

	if (total_src_len != total_dst_len)
		dev_err(&fsl_comp->qchan->vchan.chan.dev->device,
			"The data length for src and dst isn't match.\n");

	csgf_src->length = total_src_len;
	csgf_dest->length = total_dst_len;

	/* Descriptor Buffer */
	sdf->cmd = FSL_QDMA_CMD_RWTTYPE << FSL_QDMA_CMD_RWTTYPE_OFFSET;
	ddf->cmd = FSL_QDMA_CMD_RWTTYPE << FSL_QDMA_CMD_RWTTYPE_OFFSET;
}

/*
 * Prei-request full command descriptor for enqueue.
 */
static int fsl_qdma_pre_request_enqueue_desc(struct fsl_qdma_queue *queue)
{
	struct fsl_qdma_comp *comp_temp;
	int i;

	for (i = 0; i < queue->n_cq; i++) {
		comp_temp = kzalloc(sizeof(*comp_temp), GFP_KERNEL);
		if (!comp_temp)
			return -1;
		comp_temp->virt_addr = dma_pool_alloc(queue->comp_pool,
						      GFP_NOWAIT,
						      &comp_temp->bus_addr);
		if (!comp_temp->virt_addr)
			return -1;
		list_add_tail(&comp_temp->list, &queue->comp_free);
	}
	return 0;
}

/*
 * Request a command descriptor for enqueue.
 */
static struct fsl_qdma_comp *fsl_qdma_request_enqueue_desc(
					struct fsl_qdma_chan *fsl_chan,
					unsigned int dst_nents,
					unsigned int src_nents)
{
	struct fsl_qdma_comp *comp_temp;
	struct fsl_qdma_sg *sg_block;
	struct fsl_qdma_queue *queue = fsl_chan->queue;
	unsigned long flags;
	unsigned int dst_sg_entry_block, src_sg_entry_block, sg_entry_total, i;

	spin_lock_irqsave(&queue->queue_lock, flags);
	if (list_empty(&queue->comp_free)) {
		spin_unlock_irqrestore(&queue->queue_lock, flags);
		comp_temp = kzalloc(sizeof(*comp_temp), GFP_KERNEL);
		if (!comp_temp)
			return NULL;
		comp_temp->virt_addr = dma_pool_alloc(queue->comp_pool,
						      GFP_NOWAIT,
						      &comp_temp->bus_addr);
		if (!comp_temp->virt_addr)
			return NULL;
	} else {
		comp_temp = list_first_entry(&queue->comp_free,
					     struct fsl_qdma_comp,
					     list);
		list_del(&comp_temp->list);
		spin_unlock_irqrestore(&queue->queue_lock, flags);
	}

	if (dst_nents != 0)
		dst_sg_entry_block = dst_nents /
					(FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) + 1;
	else
		dst_sg_entry_block = 0;

	if (src_nents != 0)
		src_sg_entry_block = src_nents /
					(FSL_QDMA_EXPECT_SG_ENTRY_NUM - 1) + 1;
	else
		src_sg_entry_block = 0;

	sg_entry_total = dst_sg_entry_block + src_sg_entry_block;
	if (sg_entry_total) {
		sg_block = kzalloc(sizeof(*sg_block) *
					      sg_entry_total,
					      GFP_KERNEL);
		if (!sg_block)
			return NULL;
		comp_temp->sg_block = sg_block;
		for (i = 0; i < sg_entry_total; i++) {
			sg_block->virt_addr = dma_pool_alloc(queue->sg_pool,
							GFP_NOWAIT,
							&sg_block->bus_addr);
			memset(sg_block->virt_addr, 0,
					FSL_QDMA_EXPECT_SG_ENTRY_NUM * 16);
			sg_block++;
		}
	}

	comp_temp->sg_block_src = src_sg_entry_block;
	comp_temp->sg_block_dst = dst_sg_entry_block;
	comp_temp->qchan = fsl_chan;

	return comp_temp;
}

static struct fsl_qdma_queue *fsl_qdma_alloc_queue_resources(
					struct platform_device *pdev,
					unsigned int queue_num)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_queue *queue_head, *queue_temp;
	int ret, len, i;
	unsigned int queue_size[FSL_QDMA_QUEUE_MAX];

	if (queue_num > FSL_QDMA_QUEUE_MAX)
		queue_num = FSL_QDMA_QUEUE_MAX;
	len = sizeof(*queue_head) * queue_num;
	queue_head = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!queue_head)
		return NULL;

	ret = of_property_read_u32_array(np, "queue-sizes", queue_size,
								queue_num);
	if (ret) {
		dev_err(&pdev->dev, "Can't get queue-sizes.\n");
		return NULL;
	}

	for (i = 0; i < queue_num; i++) {
		if (queue_size[i] > FSL_QDMA_CIRCULAR_DESC_SIZE_MAX
			|| queue_size[i] < FSL_QDMA_CIRCULAR_DESC_SIZE_MIN) {
			dev_err(&pdev->dev, "Get wrong queue-sizes.\n");
			return NULL;
		}
		queue_temp = queue_head + i;
		queue_temp->cq = dma_alloc_coherent(&pdev->dev,
						sizeof(struct fsl_qdma_ccdf) *
						queue_size[i],
						&queue_temp->bus_addr,
						GFP_KERNEL);
		if (!queue_temp->cq)
			return NULL;
		queue_temp->n_cq = queue_size[i];
		queue_temp->id = i;
		queue_temp->virt_head = queue_temp->cq;
		queue_temp->virt_tail = queue_temp->cq;
		/*
		 * The dma pool for queue command buffer
		 */
		queue_temp->comp_pool = dma_pool_create("comp_pool",
						&pdev->dev,
						FSL_QDMA_BASE_BUFFER_SIZE,
						16, 0);
		if (!queue_temp->comp_pool) {
			dma_free_coherent(&pdev->dev,
						sizeof(struct fsl_qdma_ccdf) *
						queue_size[i],
						queue_temp->cq,
						queue_temp->bus_addr);
			return NULL;
		}
		/*
		 * The dma pool for queue command buffer
		 */
		queue_temp->sg_pool = dma_pool_create("sg_pool",
					&pdev->dev,
					FSL_QDMA_EXPECT_SG_ENTRY_NUM * 16,
					64, 0);
		if (!queue_temp->sg_pool) {
			dma_free_coherent(&pdev->dev,
						sizeof(struct fsl_qdma_ccdf) *
						queue_size[i],
						queue_temp->cq,
						queue_temp->bus_addr);
			dma_pool_destroy(queue_temp->comp_pool);
			return NULL;
		}
		/*
		 * List for queue command buffer
		 */
		INIT_LIST_HEAD(&queue_temp->comp_used);
		INIT_LIST_HEAD(&queue_temp->comp_free);
		spin_lock_init(&queue_temp->queue_lock);
	}

	return queue_head;
}

static struct fsl_qdma_queue *fsl_qdma_prep_status_queue(
						struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_queue *status_head;
	unsigned int status_size;
	int ret;

	ret = of_property_read_u32(np, "status-sizes", &status_size);
	if (ret) {
		dev_err(&pdev->dev, "Can't get status-sizes.\n");
		return NULL;
	}
	if (status_size > FSL_QDMA_CIRCULAR_DESC_SIZE_MAX
			|| status_size < FSL_QDMA_CIRCULAR_DESC_SIZE_MIN) {
		dev_err(&pdev->dev, "Get wrong status_size.\n");
		return NULL;
	}
	status_head = devm_kzalloc(&pdev->dev, sizeof(*status_head),
								GFP_KERNEL);
	if (!status_head)
		return NULL;

	/*
	 * Buffer for queue command
	 */
	status_head->cq = dma_alloc_coherent(&pdev->dev,
						sizeof(struct fsl_qdma_ccdf) *
						status_size,
						&status_head->bus_addr,
						GFP_KERNEL);
	if (!status_head->cq)
		return NULL;
	status_head->n_cq = status_size;
	status_head->virt_head = status_head->cq;
	status_head->virt_tail = status_head->cq;
	status_head->comp_pool = NULL;

	return status_head;
}

static int fsl_qdma_halt(struct fsl_qdma_engine *fsl_qdma)
{
	void __iomem *ctrl = fsl_qdma->ctrl_base;
	void __iomem *block = fsl_qdma->block_base;
	int i, count = 5;
	u32 reg;

	/* Disable the command queue and wait for idle state. */
	reg = qdma_readl(fsl_qdma, ctrl + FSL_QDMA_DMR);
	reg |= FSL_QDMA_DMR_DQD;
	qdma_writel(fsl_qdma, reg, ctrl + FSL_QDMA_DMR);
	for (i = 0; i < FSL_QDMA_QUEUE_NUM_MAX; i++)
		qdma_writel(fsl_qdma, 0, block + FSL_QDMA_BCQMR(i));

	while (1) {
		reg = qdma_readl(fsl_qdma, ctrl + FSL_QDMA_DSR);
		if (!(reg & FSL_QDMA_DSR_DB))
			break;
		if (count-- < 0)
			return -EBUSY;
		udelay(100);
	}

	/* Disable status queue. */
	qdma_writel(fsl_qdma, 0, block + FSL_QDMA_BSQMR);

	/*
	 * Clear the command queue interrupt detect register for all queues.
	 */
	qdma_writel(fsl_qdma, 0xffffffff, block + FSL_QDMA_BCQIDR(0));

	return 0;
}

static int fsl_qdma_queue_transfer_complete(struct fsl_qdma_engine *fsl_qdma)
{
	struct fsl_qdma_queue *fsl_queue = fsl_qdma->queue;
	struct fsl_qdma_queue *fsl_status = fsl_qdma->status;
	struct fsl_qdma_queue *temp_queue;
	struct fsl_qdma_comp *fsl_comp;
	struct fsl_qdma_ccdf *status_addr;
	struct fsl_qdma_csgf *csgf_src;
	void __iomem *block = fsl_qdma->block_base;
	u32 reg, i;
	bool duplicate, duplicate_handle;

	while (1) {
		duplicate = 0;
		duplicate_handle = 0;
		reg = qdma_readl(fsl_qdma, block + FSL_QDMA_BSQSR);
		if (reg & FSL_QDMA_BSQSR_QE)
			return 0;
		status_addr = fsl_status->virt_head;
		if (status_addr->queue == pre_queue &&
			status_addr->addr == pre_addr)
			duplicate = 1;

		i = status_addr->queue;
		pre_queue = status_addr->queue;
		pre_addr = status_addr->addr;
		temp_queue = fsl_queue + i;
		spin_lock(&temp_queue->queue_lock);
		if (list_empty(&temp_queue->comp_used)) {
			if (duplicate)
				duplicate_handle = 1;
			else {
				spin_unlock(&temp_queue->queue_lock);
				return -1;
			}
		} else {
			fsl_comp = list_first_entry(&temp_queue->comp_used,
							struct fsl_qdma_comp,
							list);
			csgf_src = (struct fsl_qdma_csgf *)fsl_comp->virt_addr
							   + 2;
			if (fsl_comp->bus_addr + 16 !=
					(dma_addr_t)status_addr->addr) {
				if (duplicate)
					duplicate_handle = 1;
				else {
					spin_unlock(&temp_queue->queue_lock);
					return -1;
				}
			}
		}

			if (duplicate_handle) {
			reg = qdma_readl(fsl_qdma, block + FSL_QDMA_BSQMR);
			reg |= FSL_QDMA_BSQMR_DI;
			status_addr->addr = 0x0;
			fsl_status->virt_head++;
			if (fsl_status->virt_head == fsl_status->cq
						   + fsl_status->n_cq)
				fsl_status->virt_head = fsl_status->cq;
			qdma_writel(fsl_qdma, reg, block + FSL_QDMA_BSQMR);
			spin_unlock(&temp_queue->queue_lock);
			continue;
		}
		list_del(&fsl_comp->list);

		reg = qdma_readl(fsl_qdma, block + FSL_QDMA_BSQMR);
		reg |= FSL_QDMA_BSQMR_DI;
		status_addr->addr = 0x0;
		fsl_status->virt_head++;
		if (fsl_status->virt_head == fsl_status->cq + fsl_status->n_cq)
			fsl_status->virt_head = fsl_status->cq;
		qdma_writel(fsl_qdma, reg, block + FSL_QDMA_BSQMR);
		spin_unlock(&temp_queue->queue_lock);

		spin_lock(&fsl_comp->qchan->vchan.lock);
		vchan_cookie_complete(&fsl_comp->vdesc);
		fsl_comp->qchan->status = DMA_COMPLETE;
		spin_unlock(&fsl_comp->qchan->vchan.lock);
	}
	return 0;
}

static irqreturn_t fsl_qdma_error_handler(int irq, void *dev_id)
{
	struct fsl_qdma_engine *fsl_qdma = dev_id;
	unsigned int intr;
	void __iomem *status = fsl_qdma->status_base;

	intr = qdma_readl(fsl_qdma, status + FSL_QDMA_DEDR);

	if (intr)
		dev_err(fsl_qdma->dma_dev.dev, "DMA transaction error!\n");

	qdma_writel(fsl_qdma, 0xffffffff, status + FSL_QDMA_DEDR);
	return IRQ_HANDLED;
}

static irqreturn_t fsl_qdma_queue_handler(int irq, void *dev_id)
{
	struct fsl_qdma_engine *fsl_qdma = dev_id;
	unsigned int intr, reg;
	void __iomem *block = fsl_qdma->block_base;
	void __iomem *ctrl = fsl_qdma->ctrl_base;

	intr = qdma_readl(fsl_qdma, block + FSL_QDMA_BCQIDR(0));

	if ((intr & FSL_QDMA_CQIDR_SQT) != 0)
		intr = fsl_qdma_queue_transfer_complete(fsl_qdma);

	if (intr != 0) {
		reg = qdma_readl(fsl_qdma, ctrl + FSL_QDMA_DMR);
		reg |= FSL_QDMA_DMR_DQD;
		qdma_writel(fsl_qdma, reg, ctrl + FSL_QDMA_DMR);
		qdma_writel(fsl_qdma, 0, block + FSL_QDMA_BCQIER(0));
		dev_err(fsl_qdma->dma_dev.dev, "QDMA: status err!\n");
	}

	qdma_writel(fsl_qdma, 0xffffffff, block + FSL_QDMA_BCQIDR(0));

	return IRQ_HANDLED;
}

static int
fsl_qdma_irq_init(struct platform_device *pdev,
		  struct fsl_qdma_engine *fsl_qdma)
{
	int ret;

	fsl_qdma->error_irq = platform_get_irq_byname(pdev,
							"qdma-error");
	if (fsl_qdma->error_irq < 0) {
		dev_err(&pdev->dev, "Can't get qdma controller irq.\n");
		return fsl_qdma->error_irq;
	}

	fsl_qdma->queue_irq = platform_get_irq_byname(pdev, "qdma-queue");
	if (fsl_qdma->queue_irq < 0) {
		dev_err(&pdev->dev, "Can't get qdma queue irq.\n");
		return fsl_qdma->queue_irq;
	}

	ret = devm_request_irq(&pdev->dev, fsl_qdma->error_irq,
			fsl_qdma_error_handler, 0, "qDMA error", fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't register qDMA controller IRQ.\n");
		return  ret;
	}
	ret = devm_request_irq(&pdev->dev, fsl_qdma->queue_irq,
			fsl_qdma_queue_handler, 0, "qDMA queue", fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't register qDMA queue IRQ.\n");
		return  ret;
	}

	return 0;
}

static int fsl_qdma_reg_init(struct fsl_qdma_engine *fsl_qdma)
{
	struct fsl_qdma_queue *fsl_queue = fsl_qdma->queue;
	struct fsl_qdma_queue *temp;
	void __iomem *ctrl = fsl_qdma->ctrl_base;
	void __iomem *status = fsl_qdma->status_base;
	void __iomem *block = fsl_qdma->block_base;
	int i, ret;
	u32 reg;

	/* Try to halt the qDMA engine first. */
	ret = fsl_qdma_halt(fsl_qdma);
	if (ret) {
		dev_err(fsl_qdma->dma_dev.dev, "DMA halt failed!");
		return ret;
	}

	/*
	 * Clear the command queue interrupt detect register for all queues.
	 */
	qdma_writel(fsl_qdma, 0xffffffff, block + FSL_QDMA_BCQIDR(0));

	for (i = 0; i < fsl_qdma->n_queues; i++) {
		temp = fsl_queue + i;
		/*
		 * Initialize Command Queue registers to point to the first
		 * command descriptor in memory.
		 * Dequeue Pointer Address Registers
		 * Enqueue Pointer Address Registers
		 */
		qdma_writel(fsl_qdma, temp->bus_addr,
				block + FSL_QDMA_BCQDPA_SADDR(i));
		qdma_writel(fsl_qdma, temp->bus_addr,
				block + FSL_QDMA_BCQEPA_SADDR(i));

		/* Initialize the queue mode. */
		reg = FSL_QDMA_BCQMR_EN;
		reg |= FSL_QDMA_BCQMR_CD_THLD(ilog2(temp->n_cq)-4);
		reg |= FSL_QDMA_BCQMR_CQ_SIZE(ilog2(temp->n_cq)-6);
		qdma_writel(fsl_qdma, reg, block + FSL_QDMA_BCQMR(i));
	}

	/*
	 * Workaround for erratum: ERR010812.
	 * We must enable XOFF to avoid the enqueue rejection occurs.
	 * Setting SQCCMR ENTER_WM to 0x20.
	 */
	qdma_writel(fsl_qdma, FSL_QDMA_SQCCMR_ENTER_WM,
			      block + FSL_QDMA_SQCCMR);
	/*
	 * Initialize status queue registers to point to the first
	 * command descriptor in memory.
	 * Dequeue Pointer Address Registers
	 * Enqueue Pointer Address Registers
	 */
	qdma_writel(fsl_qdma, fsl_qdma->status->bus_addr,
					block + FSL_QDMA_SQEPAR);
	qdma_writel(fsl_qdma, fsl_qdma->status->bus_addr,
					block + FSL_QDMA_SQDPAR);
	/* Initialize status queue interrupt. */
	qdma_writel(fsl_qdma, FSL_QDMA_BCQIER_CQTIE,
			      block + FSL_QDMA_BCQIER(0));
	qdma_writel(fsl_qdma, FSL_QDMA_BSQICR_ICEN | FSL_QDMA_BSQICR_ICST(5)
						   | 0x8000,
			      block + FSL_QDMA_BSQICR);
	qdma_writel(fsl_qdma, FSL_QDMA_CQIER_MEIE | FSL_QDMA_CQIER_TEIE,
			      block + FSL_QDMA_CQIER);
	/* Initialize controller interrupt register. */
	qdma_writel(fsl_qdma, 0xffffffff, status + FSL_QDMA_DEDR);
	qdma_writel(fsl_qdma, 0xffffffff, status + FSL_QDMA_DEIER);

	/* Initialize the status queue mode. */
	reg = FSL_QDMA_BSQMR_EN;
	reg |= FSL_QDMA_BSQMR_CQ_SIZE(ilog2(fsl_qdma->status->n_cq)-6);
	qdma_writel(fsl_qdma, reg, block + FSL_QDMA_BSQMR);

	reg = qdma_readl(fsl_qdma, ctrl + FSL_QDMA_DMR);
	reg &= ~FSL_QDMA_DMR_DQD;
	qdma_writel(fsl_qdma, reg, ctrl + FSL_QDMA_DMR);

	return 0;
}

static struct dma_async_tx_descriptor *fsl_qdma_prep_dma_sg(
		struct dma_chan *chan,
		struct scatterlist *dst_sg, unsigned int dst_nents,
		struct scatterlist *src_sg, unsigned int src_nents,
		unsigned long flags)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	struct fsl_qdma_comp *fsl_comp;

	fsl_comp = fsl_qdma_request_enqueue_desc(fsl_chan,
						 dst_nents,
						 src_nents);
	fsl_qdma_comp_fill_sg(fsl_comp, dst_sg, dst_nents, src_sg, src_nents);

	return vchan_tx_prep(&fsl_chan->vchan, &fsl_comp->vdesc, flags);
}

static struct dma_async_tx_descriptor *
fsl_qdma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst,
		dma_addr_t src, size_t len, unsigned long flags)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	struct fsl_qdma_comp *fsl_comp;

	fsl_comp = fsl_qdma_request_enqueue_desc(fsl_chan, 0, 0);
	fsl_qdma_comp_fill_memcpy(fsl_comp, dst, src, len);

	return vchan_tx_prep(&fsl_chan->vchan, &fsl_comp->vdesc, flags);
}

static void fsl_qdma_enqueue_desc(struct fsl_qdma_chan *fsl_chan)
{
	void __iomem *block = fsl_chan->qdma->block_base;
	struct fsl_qdma_queue *fsl_queue = fsl_chan->queue;
	struct fsl_qdma_comp *fsl_comp;
	struct virt_dma_desc *vdesc;
	u32 reg;

	reg = qdma_readl(fsl_chan->qdma, block + FSL_QDMA_BCQSR(fsl_queue->id));
	if (reg & (FSL_QDMA_BCQSR_QF | FSL_QDMA_BCQSR_XOFF))
		return;
	vdesc = vchan_next_desc(&fsl_chan->vchan);
	if (!vdesc)
		return;
	list_del(&vdesc->node);
	fsl_comp = to_fsl_qdma_comp(vdesc);

	memcpy(fsl_queue->virt_head++, fsl_comp->virt_addr, 16);
	if (fsl_queue->virt_head == fsl_queue->cq + fsl_queue->n_cq)
		fsl_queue->virt_head = fsl_queue->cq;

	list_add_tail(&fsl_comp->list, &fsl_queue->comp_used);
	barrier();
	reg = qdma_readl(fsl_chan->qdma, block + FSL_QDMA_BCQMR(fsl_queue->id));
	reg |= FSL_QDMA_BCQMR_EI;
	qdma_writel(fsl_chan->qdma, reg, block + FSL_QDMA_BCQMR(fsl_queue->id));
	fsl_chan->status = DMA_IN_PROGRESS;
}

static enum dma_status fsl_qdma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return dma_cookie_status(chan, cookie, txstate);
}

static void fsl_qdma_free_desc(struct virt_dma_desc *vdesc)
{
	struct fsl_qdma_comp *fsl_comp;
	struct fsl_qdma_queue *fsl_queue;
	struct fsl_qdma_sg *sg_block;
	unsigned long flags;
	unsigned int i;

	fsl_comp = to_fsl_qdma_comp(vdesc);
	fsl_queue = fsl_comp->qchan->queue;

	if (fsl_comp->sg_block) {
		for (i = 0; i < fsl_comp->sg_block_src +
				fsl_comp->sg_block_dst; i++) {
			sg_block = fsl_comp->sg_block + i;
			dma_pool_free(fsl_queue->sg_pool,
				      sg_block->virt_addr,
				      sg_block->bus_addr);
		}
		kfree(fsl_comp->sg_block);
	}

	spin_lock_irqsave(&fsl_queue->queue_lock, flags);
	list_add_tail(&fsl_comp->list, &fsl_queue->comp_free);
	spin_unlock_irqrestore(&fsl_queue->queue_lock, flags);
}

static void fsl_qdma_issue_pending(struct dma_chan *chan)
{
	struct fsl_qdma_chan *fsl_chan = to_fsl_qdma_chan(chan);
	struct fsl_qdma_queue *fsl_queue = fsl_chan->queue;
	unsigned long flags;

	spin_lock_irqsave(&fsl_queue->queue_lock, flags);
	spin_lock(&fsl_chan->vchan.lock);
	if (vchan_issue_pending(&fsl_chan->vchan))
		fsl_qdma_enqueue_desc(fsl_chan);
	spin_unlock(&fsl_chan->vchan.lock);
	spin_unlock_irqrestore(&fsl_queue->queue_lock, flags);
}

static int fsl_qdma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_engine *fsl_qdma;
	struct fsl_qdma_chan *fsl_chan;
	struct resource *res;
	unsigned int len, chans, queues;
	int ret, i;

	ret = of_property_read_u32(np, "channels", &chans);
	if (ret) {
		dev_err(&pdev->dev, "Can't get channels.\n");
		return ret;
	}

	len = sizeof(*fsl_qdma) + sizeof(*fsl_chan) * chans;
	fsl_qdma = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
	if (!fsl_qdma)
		return -ENOMEM;

	ret = of_property_read_u32(np, "queues", &queues);
	if (ret) {
		dev_err(&pdev->dev, "Can't get queues.\n");
		return ret;
	}

	fsl_qdma->queue = fsl_qdma_alloc_queue_resources(pdev, queues);
	if (!fsl_qdma->queue)
		return -ENOMEM;

	fsl_qdma->status = fsl_qdma_prep_status_queue(pdev);
	if (!fsl_qdma->status)
		return -ENOMEM;

	fsl_qdma->n_chans = chans;
	fsl_qdma->n_queues = queues;
	mutex_init(&fsl_qdma->fsl_qdma_mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fsl_qdma->ctrl_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_qdma->ctrl_base))
		return PTR_ERR(fsl_qdma->ctrl_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	fsl_qdma->status_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_qdma->status_base))
		return PTR_ERR(fsl_qdma->status_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	fsl_qdma->block_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_qdma->block_base))
		return PTR_ERR(fsl_qdma->block_base);

	ret = fsl_qdma_irq_init(pdev, fsl_qdma);
	if (ret)
		return ret;

	fsl_qdma->big_endian = of_property_read_bool(np, "big-endian");
	INIT_LIST_HEAD(&fsl_qdma->dma_dev.channels);
	for (i = 0; i < fsl_qdma->n_chans; i++) {
		struct fsl_qdma_chan *fsl_chan = &fsl_qdma->chans[i];

		fsl_chan->qdma = fsl_qdma;
		fsl_chan->queue = fsl_qdma->queue + i % fsl_qdma->n_queues;
		fsl_chan->vchan.desc_free = fsl_qdma_free_desc;
		INIT_LIST_HEAD(&fsl_chan->qcomp);
		vchan_init(&fsl_chan->vchan, &fsl_qdma->dma_dev);
	}
	for (i = 0; i < fsl_qdma->n_queues; i++)
		fsl_qdma_pre_request_enqueue_desc(fsl_qdma->queue + i);

	dma_cap_set(DMA_MEMCPY, fsl_qdma->dma_dev.cap_mask);
	dma_cap_set(DMA_SG, fsl_qdma->dma_dev.cap_mask);

	fsl_qdma->dma_dev.dev = &pdev->dev;
	fsl_qdma->dma_dev.device_alloc_chan_resources
		= fsl_qdma_alloc_chan_resources;
	fsl_qdma->dma_dev.device_free_chan_resources
		= fsl_qdma_free_chan_resources;
	fsl_qdma->dma_dev.device_tx_status = fsl_qdma_tx_status;
	fsl_qdma->dma_dev.device_prep_dma_memcpy = fsl_qdma_prep_memcpy;
	fsl_qdma->dma_dev.device_prep_dma_sg = fsl_qdma_prep_dma_sg;
	fsl_qdma->dma_dev.device_issue_pending = fsl_qdma_issue_pending;

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));

	platform_set_drvdata(pdev, fsl_qdma);

	ret = dma_async_device_register(&fsl_qdma->dma_dev);
	if (ret) {
		dev_err(&pdev->dev, "Can't register Freescale qDMA engine.\n");
		return ret;
	}

	ret = fsl_qdma_reg_init(fsl_qdma);
	if (ret) {
		dev_err(&pdev->dev, "Can't Initialize the qDMA engine.\n");
		return ret;
	}


	return 0;
}

static int fsl_qdma_remove(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fsl_qdma_engine *fsl_qdma = platform_get_drvdata(pdev);
	struct fsl_qdma_queue *queue_temp;
	struct fsl_qdma_queue *status = fsl_qdma->status;
	struct fsl_qdma_comp *comp_temp, *_comp_temp;
	int i;

	of_dma_controller_free(np);
	dma_async_device_unregister(&fsl_qdma->dma_dev);

	/* Free descriptor areas */
	for (i = 0; i < fsl_qdma->n_queues; i++) {
		queue_temp = fsl_qdma->queue + i;
		list_for_each_entry_safe(comp_temp, _comp_temp,
					&queue_temp->comp_used,	list) {
			dma_pool_free(queue_temp->comp_pool,
					comp_temp->virt_addr,
					comp_temp->bus_addr);
			list_del(&comp_temp->list);
			kfree(comp_temp);
		}
		list_for_each_entry_safe(comp_temp, _comp_temp,
					&queue_temp->comp_free, list) {
			dma_pool_free(queue_temp->comp_pool,
					comp_temp->virt_addr,
					comp_temp->bus_addr);
			list_del(&comp_temp->list);
			kfree(comp_temp);
		}
		dma_free_coherent(&pdev->dev, sizeof(struct fsl_qdma_ccdf) *
					queue_temp->n_cq, queue_temp->cq,
					queue_temp->bus_addr);
		dma_pool_destroy(queue_temp->comp_pool);
	}

	dma_free_coherent(&pdev->dev, sizeof(struct fsl_qdma_ccdf) *
				status->n_cq, status->cq, status->bus_addr);
	return 0;
}

static const struct of_device_id fsl_qdma_dt_ids[] = {
	{ .compatible = "fsl,ls1021a-qdma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_qdma_dt_ids);

static struct platform_driver fsl_qdma_driver = {
	.driver		= {
		.name	= "fsl-qdma",
		.owner  = THIS_MODULE,
		.of_match_table = fsl_qdma_dt_ids,
	},
	.probe          = fsl_qdma_probe,
	.remove		= fsl_qdma_remove,
};

static int __init fsl_qdma_init(void)
{
	return platform_driver_register(&fsl_qdma_driver);
}
subsys_initcall(fsl_qdma_init);

static void __exit fsl_qdma_exit(void)
{
	platform_driver_unregister(&fsl_qdma_driver);
}
module_exit(fsl_qdma_exit);

MODULE_ALIAS("platform:fsl-qdma");
MODULE_DESCRIPTION("Freescale qDMA engine driver");
MODULE_LICENSE("GPL v2");
