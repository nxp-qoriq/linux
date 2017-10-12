/*
 * caam support for SG DMA
 *
 * Copyright 2016 Freescale Semiconductor, Inc
 * Copyright 2017 NXP
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/debugfs.h>

#include <linux/dmaengine.h>
#include "dmaengine.h"

#include "../crypto/caam/regs.h"
#include "../crypto/caam/jr.h"
#include "../crypto/caam/error.h"
#include "../crypto/caam/intern.h"
#include "../crypto/caam/desc_constr.h"
#include "../crypto/caam/sg_sw_sec4.h"

#define DESC_DMA_MEMCPY_LEN	((CAAM_DESC_BYTES_MAX - DESC_JOB_IO_LEN) / \
				 CAAM_CMD_SZ)

/* This is max chunk size of a DMA transfer. If a buffer is larger than this
 * value it is internally broken into chunks of max CAAM_DMA_CHUNK_SIZE bytes
 * and for each chunk a DMA transfer request is issued.
 * This value is the largest number on 16 bits that is a multiple of 256 bytes
 * (the largest configurable CAAM DMA burst size).
 */
#define CAAM_DMA_CHUNK_SIZE	65280

struct caam_dma_sh_desc {
	u32 desc[DESC_DMA_MEMCPY_LEN] ____cacheline_aligned;
	dma_addr_t desc_dma;
};

/* caam dma extended descriptor */
struct caam_dma_edesc {
	struct dma_async_tx_descriptor async_tx;
	struct list_head node;
	struct caam_dma_ctx *ctx;
	dma_addr_t src_dma;
	dma_addr_t dst_dma;
	unsigned int src_len;
	unsigned int dst_len;
	struct sec4_sg_entry *sec4_sg;
	u32 jd[] ____cacheline_aligned;
};

/*
 * caam_dma_ctx - per jr/channel context
 * @chan: dma channel used by async_tx API
 * @node: list_head used to attach to the global dma_ctx_list
 * @jrdev: Job Ring device
 * @submit_q: queue of pending (submitted, but not enqueued) jobs
 * @done_not_acked: jobs that have been completed by jr, but maybe not acked
 * @edesc_lock: protects extended descriptor
 */
struct caam_dma_ctx {
	struct dma_chan chan;
	struct list_head node;
	struct device *jrdev;
	struct list_head submit_q;
	struct list_head done_not_acked;
	spinlock_t edesc_lock;
};

static struct dma_device *dma_dev;
static struct caam_dma_sh_desc *dma_sh_desc;
static LIST_HEAD(dma_ctx_list);

static dma_cookie_t caam_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct caam_dma_edesc *edesc = NULL;
	struct caam_dma_ctx *ctx = NULL;
	dma_cookie_t cookie;

	edesc = container_of(tx, struct caam_dma_edesc, async_tx);
	ctx = container_of(tx->chan, struct caam_dma_ctx, chan);

	spin_lock_bh(&ctx->edesc_lock);

	cookie = dma_cookie_assign(tx);
	list_add_tail(&edesc->node, &ctx->submit_q);

	spin_unlock_bh(&ctx->edesc_lock);

	return cookie;
}

static unsigned int caam_dma_sg_dma_len(struct scatterlist *sg,
					unsigned int nents)
{
	unsigned int len;

	for (len = 0; sg && nents; sg = sg_next(sg), nents--)
		len += sg_dma_len(sg);

	return len;
}

static struct caam_dma_edesc *
caam_dma_sg_edesc_alloc(struct dma_chan *chan,
			struct scatterlist *dst_sg, unsigned int dst_nents,
			struct scatterlist *src_sg, unsigned int src_nents,
			unsigned long flags)
{
	struct caam_dma_ctx *ctx = container_of(chan, struct caam_dma_ctx,
						chan);
	struct device *jrdev = ctx->jrdev;
	struct caam_dma_edesc *edesc;
	struct sec4_sg_entry *sec4_sg;
	dma_addr_t sec4_sg_dma_src;
	unsigned int sec4_sg_bytes;

	if (!dst_sg || !src_sg || !dst_nents || !src_nents)
		return NULL;

	sec4_sg_bytes = (src_nents + dst_nents) * sizeof(*sec4_sg);

	edesc = kzalloc(sizeof(*edesc) + DESC_JOB_IO_LEN + sec4_sg_bytes,
			GFP_DMA | GFP_NOWAIT);
	if (!edesc)
		return ERR_PTR(-ENOMEM);

	edesc->src_len = caam_dma_sg_dma_len(src_sg, src_nents);
	edesc->dst_len = caam_dma_sg_dma_len(dst_sg, dst_nents);
	if (edesc->src_len != edesc->dst_len) {
		dev_err(jrdev, "%s: src(%u) and dst(%u) len mismatch.\n",
			__func__, edesc->src_len, edesc->dst_len);
		kfree(edesc);
		return ERR_PTR(-EINVAL);
	}

	dma_async_tx_descriptor_init(&edesc->async_tx, chan);
	edesc->async_tx.tx_submit = caam_dma_tx_submit;
	edesc->async_tx.flags = flags;
	edesc->async_tx.cookie = -EBUSY;

	/* Prepare SEC SGs */
	edesc->sec4_sg = (void *)edesc + offsetof(struct caam_dma_edesc, jd) +
			 DESC_JOB_IO_LEN;

	sec4_sg = edesc->sec4_sg;
	sg_to_sec4_sg_last(src_sg, src_nents, sec4_sg, 0);

	sec4_sg += src_nents;
	sg_to_sec4_sg_last(dst_sg, dst_nents, sec4_sg, 0);

	sec4_sg_dma_src = dma_map_single(jrdev, edesc->sec4_sg, sec4_sg_bytes,
					 DMA_TO_DEVICE);
	if (dma_mapping_error(jrdev, sec4_sg_dma_src)) {
		dev_err(jrdev, "error mapping segments to device\n");
		kfree(edesc);
		return ERR_PTR(-ENOMEM);
	}

	edesc->src_dma = sec4_sg_dma_src;
	edesc->dst_dma = sec4_sg_dma_src + src_nents * sizeof(*sec4_sg);
	edesc->ctx = ctx;

	return edesc;
}

static void caam_jr_chan_free_edesc(struct caam_dma_edesc *edesc)
{
	struct caam_dma_ctx *ctx = edesc->ctx;
	struct caam_dma_edesc *_edesc = NULL;

	spin_lock_bh(&ctx->edesc_lock);

	list_add_tail(&edesc->node, &ctx->done_not_acked);
	list_for_each_entry_safe(edesc, _edesc, &ctx->done_not_acked, node) {
		if (async_tx_test_ack(&edesc->async_tx)) {
			list_del(&edesc->node);
			kfree(edesc);
		}
	}

	spin_unlock_bh(&ctx->edesc_lock);
}

static void caam_dma_done(struct device *dev, u32 *hwdesc, u32 err,
			  void *context)
{
	struct caam_dma_edesc *edesc = context;
	struct caam_dma_ctx *ctx = edesc->ctx;
	dma_async_tx_callback callback;
	void *callback_param;

	if (err)
		caam_jr_strstatus(ctx->jrdev, err);

	dma_run_dependencies(&edesc->async_tx);

	spin_lock_bh(&ctx->edesc_lock);
	dma_cookie_complete(&edesc->async_tx);
	spin_unlock_bh(&ctx->edesc_lock);

	callback = edesc->async_tx.callback;
	callback_param = edesc->async_tx.callback_param;

	dma_descriptor_unmap(&edesc->async_tx);

	caam_jr_chan_free_edesc(edesc);

	if (callback)
		callback(callback_param);
}

static void caam_dma_sg_init_job_desc(struct caam_dma_edesc *edesc)
{
	u32 *jd = edesc->jd;
	u32 *sh_desc = dma_sh_desc->desc;
	dma_addr_t desc_dma = dma_sh_desc->desc_dma;

	/* init the job descriptor */
	init_job_desc_shared(jd, desc_dma, desc_len(sh_desc), HDR_REVERSE);

	/* set SEQIN PTR */
	append_seq_in_ptr(jd, edesc->src_dma, edesc->src_len, LDST_SGF);

	/* set SEQOUT PTR */
	append_seq_out_ptr(jd, edesc->dst_dma, edesc->dst_len, LDST_SGF);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "caam dma desc@" __stringify(__LINE__) ": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, jd, desc_bytes(jd), 1);
#endif
}

/* This function can be called from an interrupt context */
static struct dma_async_tx_descriptor *
caam_dma_prep_sg(struct dma_chan *chan, struct scatterlist *dst_sg,
		 unsigned int dst_nents, struct scatterlist *src_sg,
		 unsigned int src_nents, unsigned long flags)
{
	struct caam_dma_edesc *edesc;

	/* allocate extended descriptor */
	edesc = caam_dma_sg_edesc_alloc(chan, dst_sg, dst_nents, src_sg,
					src_nents, flags);
	if (IS_ERR_OR_NULL(edesc))
		return ERR_CAST(edesc);

	/* Initialize job descriptor */
	caam_dma_sg_init_job_desc(edesc);

	return &edesc->async_tx;
}

static void caam_dma_memcpy_init_job_desc(struct caam_dma_edesc *edesc)
{
	u32 *jd = edesc->jd;
	u32 *sh_desc = dma_sh_desc->desc;
	dma_addr_t desc_dma = dma_sh_desc->desc_dma;

	/* init the job descriptor */
	init_job_desc_shared(jd, desc_dma, desc_len(sh_desc), HDR_REVERSE);

	/* set SEQIN PTR */
	append_seq_in_ptr(jd, edesc->src_dma, edesc->src_len, 0);

	/* set SEQOUT PTR */
	append_seq_out_ptr(jd, edesc->dst_dma, edesc->dst_len, 0);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "caam dma desc@" __stringify(__LINE__) ": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, jd, desc_bytes(jd), 1);
#endif
}

static struct dma_async_tx_descriptor *
caam_dma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		     size_t len, unsigned long flags)
{
	struct caam_dma_edesc *edesc;
	struct caam_dma_ctx *ctx = container_of(chan, struct caam_dma_ctx,
						chan);

	edesc = kzalloc(sizeof(*edesc) + DESC_JOB_IO_LEN, GFP_DMA | GFP_NOWAIT);
	if (!edesc)
		return ERR_PTR(-ENOMEM);

	dma_async_tx_descriptor_init(&edesc->async_tx, chan);
	edesc->async_tx.tx_submit = caam_dma_tx_submit;
	edesc->async_tx.flags = flags;
	edesc->async_tx.cookie = -EBUSY;

	edesc->src_dma = src;
	edesc->src_len = len;
	edesc->dst_dma = dst;
	edesc->dst_len = len;
	edesc->ctx = ctx;

	caam_dma_memcpy_init_job_desc(edesc);

	return &edesc->async_tx;
}

/* This function can be called in an interrupt context */
static void caam_dma_issue_pending(struct dma_chan *chan)
{
	struct caam_dma_ctx *ctx = container_of(chan, struct caam_dma_ctx,
						chan);
	struct caam_dma_edesc *edesc, *_edesc;

	spin_lock_bh(&ctx->edesc_lock);
	list_for_each_entry_safe(edesc, _edesc, &ctx->submit_q, node) {
		if (caam_jr_enqueue(ctx->jrdev, edesc->jd,
				    caam_dma_done, edesc) < 0)
			break;
		list_del(&edesc->node);
	}
	spin_unlock_bh(&ctx->edesc_lock);
}

static void caam_dma_free_chan_resources(struct dma_chan *chan)
{
	struct caam_dma_ctx *ctx = container_of(chan, struct caam_dma_ctx,
						chan);
	struct caam_dma_edesc *edesc, *_edesc;

	spin_lock_bh(&ctx->edesc_lock);
	list_for_each_entry_safe(edesc, _edesc, &ctx->submit_q, node) {
		list_del(&edesc->node);
		kfree(edesc);
	}
	list_for_each_entry_safe(edesc, _edesc, &ctx->done_not_acked, node) {
		list_del(&edesc->node);
		kfree(edesc);
	}
	spin_unlock_bh(&ctx->edesc_lock);
}

static int caam_dma_jr_chan_bind(void)
{
	struct device *jrdev;
	struct caam_dma_ctx *ctx;
	int bonds = 0;
	int i;

	for (i = 0; i < caam_jr_driver_probed(); i++) {
		jrdev = caam_jridx_alloc(i);
		if (IS_ERR(jrdev)) {
			pr_err("job ring device %d allocation failed\n", i);
			continue;
		}

		ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
		if (!ctx) {
			caam_jr_free(jrdev);
			continue;
		}

		ctx->chan.device = dma_dev;
		ctx->chan.private = ctx;

		ctx->jrdev = jrdev;

		INIT_LIST_HEAD(&ctx->submit_q);
		INIT_LIST_HEAD(&ctx->done_not_acked);
		INIT_LIST_HEAD(&ctx->node);
		spin_lock_init(&ctx->edesc_lock);

		dma_cookie_init(&ctx->chan);

		/* add the context of this channel to the context list */
		list_add_tail(&ctx->node, &dma_ctx_list);

		/* add this channel to the device chan list */
		list_add_tail(&ctx->chan.device_node, &dma_dev->channels);

		bonds++;
	}

	return bonds;
}

static inline void caam_jr_dma_free(struct dma_chan *chan)
{
	struct caam_dma_ctx *ctx = container_of(chan, struct caam_dma_ctx,
						chan);

	list_del(&ctx->node);
	list_del(&chan->device_node);
	caam_jr_free(ctx->jrdev);
	kfree(ctx);
}

static void set_caam_dma_desc(u32 *desc)
{
	u32 *jmp_cmd;

	/* dma shared descriptor */
	init_sh_desc(desc, HDR_SHARE_NEVER | (1 << HDR_START_IDX_SHIFT));

	/* REG1 = CAAM_DMA_CHUNK_SIZE */
	append_math_add_imm_u32(desc, REG1, ZERO, IMM, CAAM_DMA_CHUNK_SIZE);

	/* REG0 = SEQINLEN - CAAM_DMA_CHUNK_SIZE */
	append_math_sub_imm_u32(desc, REG0, SEQINLEN, IMM, CAAM_DMA_CHUNK_SIZE);

	/* if (REG0 > 0)
	 *	jmp to LABEL1
	 */
	jmp_cmd = append_jump(desc, JUMP_TEST_INVALL | JUMP_COND_MATH_N |
			      JUMP_COND_MATH_Z);

	/* REG1 = SEQINLEN */
	append_math_sub(desc, REG1, SEQINLEN, ZERO, CAAM_CMD_SZ);

	/* LABEL1 */
	set_jump_tgt_here(desc, jmp_cmd);

	/* VARSEQINLEN = REG1 */
	append_math_add(desc, VARSEQINLEN, REG1, ZERO, CAAM_CMD_SZ);

	/* VARSEQOUTLEN = REG1 */
	append_math_add(desc, VARSEQOUTLEN, REG1, ZERO, CAAM_CMD_SZ);

	/* do FIFO STORE */
	append_seq_fifo_store(desc, 0, FIFOST_TYPE_METADATA | LDST_VLF);

	/* do FIFO LOAD */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS1 |
			     FIFOLD_TYPE_IFIFO | LDST_VLF);

	/* if (REG0 > 0)
	 *	jmp 0xF8 (after shared desc header)
	 */
	append_jump(desc, JUMP_TEST_INVALL | JUMP_COND_MATH_N |
		    JUMP_COND_MATH_Z | 0xF8);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "caam dma shdesc@" __stringify(__LINE__) ": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc), 1);
#endif
}

static int __init caam_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *ctrldev = dev->parent;
	struct dma_chan *chan, *_chan;
	u32 *sh_desc;
	int err = -ENOMEM;
	int bonds;

	if (!caam_jr_driver_probed()) {
		dev_info(dev, "Defer probing after JR driver probing\n");
		return -EPROBE_DEFER;
	}

	dma_dev = kzalloc(sizeof(*dma_dev), GFP_KERNEL);
	if (!dma_dev)
		return -ENOMEM;

	dma_sh_desc = kzalloc(sizeof(*dma_sh_desc), GFP_KERNEL | GFP_DMA);
	if (!dma_sh_desc)
		goto desc_err;

	sh_desc = dma_sh_desc->desc;
	set_caam_dma_desc(sh_desc);
	dma_sh_desc->desc_dma = dma_map_single(ctrldev, sh_desc,
					       desc_bytes(sh_desc),
					       DMA_TO_DEVICE);
	if (dma_mapping_error(ctrldev, dma_sh_desc->desc_dma)) {
		dev_err(dev, "unable to map dma descriptor\n");
		goto map_err;
	}

	INIT_LIST_HEAD(&dma_dev->channels);

	bonds = caam_dma_jr_chan_bind();
	if (!bonds) {
		err = -ENODEV;
		goto jr_bind_err;
	}

	dma_dev->dev = dev;
	dma_dev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dma_cap_set(DMA_SG, dma_dev->cap_mask);
	dma_cap_set(DMA_MEMCPY, dma_dev->cap_mask);
	dma_cap_set(DMA_PRIVATE, dma_dev->cap_mask);
	dma_dev->device_tx_status = dma_cookie_status;
	dma_dev->device_issue_pending = caam_dma_issue_pending;
	dma_dev->device_prep_dma_sg = caam_dma_prep_sg;
	dma_dev->device_prep_dma_memcpy = caam_dma_prep_memcpy;
	dma_dev->device_free_chan_resources = caam_dma_free_chan_resources;

	err = dma_async_device_register(dma_dev);
	if (err) {
		dev_err(dev, "Failed to register CAAM DMA engine\n");
		goto jr_bind_err;
	}

	dev_info(dev, "caam dma support with %d job rings\n", bonds);

	return err;

jr_bind_err:
	list_for_each_entry_safe(chan, _chan, &dma_dev->channels, device_node)
		caam_jr_dma_free(chan);

	dma_unmap_single(ctrldev, dma_sh_desc->desc_dma, desc_bytes(sh_desc),
			 DMA_TO_DEVICE);
map_err:
	kfree(dma_sh_desc);
desc_err:
	kfree(dma_dev);
	return err;
}

static int caam_dma_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *ctrldev = dev->parent;
	struct caam_dma_ctx *ctx, *_ctx;

	dma_async_device_unregister(dma_dev);

	list_for_each_entry_safe(ctx, _ctx, &dma_ctx_list, node) {
		list_del(&ctx->node);
		caam_jr_free(ctx->jrdev);
		kfree(ctx);
	}

	dma_unmap_single(ctrldev, dma_sh_desc->desc_dma,
			 desc_bytes(dma_sh_desc->desc), DMA_TO_DEVICE);

	kfree(dma_sh_desc);
	kfree(dma_dev);

	dev_info(dev, "caam dma support disabled\n");
	return 0;
}

static const struct of_device_id caam_dma_match[] = {
	{ .compatible = "fsl,sec-v5.4-dma", },
	{ .compatible = "fsl,sec-v5.0-dma", },
	{ .compatible = "fsl,sec-v4.0-dma", },
	{},
};
MODULE_DEVICE_TABLE(of, caam_dma_match);

static struct platform_driver caam_dma_driver = {
	.driver = {
		.name = "caam-dma",
		.of_match_table = caam_dma_match,
	},
	.probe  = caam_dma_probe,
	.remove = caam_dma_remove,
};
module_platform_driver(caam_dma_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("NXP CAAM support for SG DMA");
MODULE_AUTHOR("NXP Semiconductors");
