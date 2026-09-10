// SPDX-License-Identifier: GPL-2.0
/*
 * mxc-jpeg-enc-api.c - In-kernel BGR24 -> JPEG encoding API
 *
 * Uses private per-session DMA descriptors - completely isolated from
 * the V4L2 M2M path (no shared slot_data conflict).
 *
 * IRQ flow mirrors mxc_jpeg_dec_irq() exactly:
 *   Config FRMDONE  -> enc_mode_go()  (line 1051-1058 in mxc-jpeg.c)
 *   Encode FRMDONE  -> done_cb()      (line 1067-1068 in mxc-jpeg.c)
 *
 * Copyright 2026 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/completion.h>

#include "mxc-jpeg-hw.h"
#include "mxc-jpeg.h"
#include "mxc-jpeg-enc-api.h"


bool mxc_jpeg_enc_api_is_active(struct mxc_jpeg_dev *jpeg, int slot)
{
    struct mxc_jpeg_enc_session *tmp;
    unsigned long flags;
    bool found = false;

    spin_lock_irqsave(&jpeg->g_session_list_lock, flags);
    list_for_each_entry(tmp, &jpeg->g_session_list, node) {
        if (tmp->slot == slot && tmp->active) {
            found = true;
            break;
        }
    }
    spin_unlock_irqrestore(&jpeg->g_session_list_lock, flags);
    return found;
}
EXPORT_SYMBOL_GPL(mxc_jpeg_enc_api_is_active);

struct device *mxc_jpeg_get_enc_device(struct mxc_jpeg_dev *jpeg_dev)
{
	return jpeg_dev ? jpeg_dev->dev : NULL;
}
EXPORT_SYMBOL_GPL(mxc_jpeg_get_enc_device);

void mxc_jpeg_enc_api_irq_notify(struct mxc_jpeg_dev *jpeg,
				 int slot, u32 slot_status)
{
	struct mxc_jpeg_enc_session *session = NULL;
	struct mxc_jpeg_enc_session *tmp;
	unsigned long flags;
	size_t jpeg_size = 0;
	bool do_conf_go = false;
	bool do_enc_done = false;
	bool do_err = false;
	u8 quality = 0;

	spin_lock_irqsave(&jpeg->g_session_list_lock, flags);

	list_for_each_entry(tmp, &jpeg->g_session_list, node) {
		if (tmp->jpeg != jpeg || tmp->slot != slot || !tmp->active)
			continue;
		session = tmp;
		break;
	}

	if (!session) {
		spin_unlock_irqrestore(&jpeg->g_session_list_lock, flags);
		return;
	}

	dev_dbg(jpeg->dev,
		"enc_api: irq_notify slot=%d status=0x%x active=%d\n",
		slot, slot_status, atomic_read(&jpeg->api_hw_active));

	session->irq_status = slot_status;

	if (slot_status & SLOT_STATUS_ENC_CONFIG_ERR) {
		session->active    = false;
		session->enc_state = MXC_JPEG_ENC_CONF_API;
		atomic_set(&jpeg->api_hw_active, 0);
		do_err = true;
	} else if (session->enc_state == MXC_JPEG_ENC_CONF_API) {
		/* Config FRMDONE: start encode phase */
		session->enc_state = MXC_JPEG_ENCODING_API;
		quality = session->quality;
		do_conf_go = true;
	} else {
		/* Encode FRMDONE: deliver result */
		jpeg_size = readl(jpeg->base_reg +
				  MXC_SLOT_OFFSET(slot, SLOT_BUF_PTR));
        if (jpeg_size > session->out_buf_size) {
            dev_warn(jpeg->dev, "enc_api: overflow %zu > %zu",
                     jpeg_size, session->out_buf_size);
            jpeg_size = 0;
            do_err = true;
        } else {
            do_enc_done = true;
        }
		session->enc_state = MXC_JPEG_ENC_CONF_API;
		session->active    = false;
		atomic_set(&jpeg->api_hw_active, 0);
    }

	spin_unlock_irqrestore(&jpeg->g_session_list_lock, flags);

	if (do_err) {
		if (session->done_cb)
			session->done_cb(-EIO, 0, session->cb_data);
	} else if (do_conf_go) {
		/* Pass session slot, not hardcoded 0 */
		mxc_jpeg_enc_set_quality(jpeg->dev, jpeg->base_reg, quality);
		mxc_jpeg_enc_mode_go(jpeg->dev, jpeg->base_reg, slot);
	} else if (do_enc_done) {
		dev_dbg(jpeg->dev,
			"enc_api: encode done jpeg=%zu bytes\n", jpeg_size);
		if (session->done_cb)
			session->done_cb(0, jpeg_size, session->cb_data);
	}
}
EXPORT_SYMBOL_GPL(mxc_jpeg_enc_api_irq_notify);

struct mxc_jpeg_enc_session *mxc_jpeg_enc_session_create(struct mxc_jpeg_dev *jpeg_dev,
							 u32 width, u32 height,
							 u8 quality)
{
	struct mxc_jpeg_enc_session *session;
	struct device *dev;
	unsigned long flags;
	unsigned int cfg_size;
	int ret;

	if (!jpeg_dev ||
	    width  < MXC_JPEG_MIN_WIDTH  || width  > MXC_JPEG_MAX_WIDTH ||
	    height < MXC_JPEG_MIN_HEIGHT || height > MXC_JPEG_MAX_HEIGHT)
		return ERR_PTR(-EINVAL);

	if (!jpeg_dev->dev) {
		pr_err("enc_api: jpeg_dev->dev is NULL, device not probed yet\n");
		return ERR_PTR(-ENODEV);
	}

	if (quality < 1 || quality > 100) {
		pr_warn("enc_api: quality %u out of range [1,100], clamping to 75\n",
			quality);
		quality = 75;
	}

	dev = jpeg_dev->dev;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return ERR_PTR(-ENOMEM);

	session->jpeg      = jpeg_dev;
	session->dev       = dev;
	session->width     = width;
	session->height    = height;
	session->quality   = quality;

	spin_lock_irqsave(&jpeg_dev->hw_lock, flags);
	if (!jpeg_dev->slot_data.used) {
		jpeg_dev->slot_data.used = true;
		session->slot      = jpeg_dev->slot_data.slot;
		session->enc_state = MXC_JPEG_ENC_CONF_API;
	} else {
		dev_err(dev, "No more free slots\n");
		spin_unlock_irqrestore(&jpeg_dev->hw_lock, flags);
		goto err_desc;
	}
	spin_unlock_irqrestore(&jpeg_dev->hw_lock, flags);
	session->active    = false;

	mutex_init(&session->lock);
	INIT_LIST_HEAD(&session->node);

	/* Private encode descriptor */
	session->desc = dma_alloc_coherent(dev, sizeof(struct mxc_jpeg_desc),
					   &session->desc_dma, GFP_KERNEL);
	if (!session->desc) {
		ret = -ENOMEM;
		goto err_desc;
	}

	/* Private config descriptor */
	session->cfg_desc = dma_alloc_coherent(dev, sizeof(struct mxc_jpeg_desc),
					       &session->cfg_desc_dma, GFP_KERNEL);
	if (!session->cfg_desc) {
		ret = -ENOMEM;
		goto err_cfg_desc;
	}

	/* Private config bitstream buffer */
	session->cfg_stream = dma_alloc_coherent(dev, MXC_JPEG_MAX_CFG_STREAM,
						 &session->cfg_stream_dma,
						 GFP_KERNEL);
	if (!session->cfg_stream) {
		ret = -ENOMEM;
		goto err_cfg_stream;
	}

	/* Build BGR24 config bitstream once */
	cfg_size = mxc_jpeg_setup_cfg_stream(session->cfg_stream,
					     V4L2_PIX_FMT_BGR24,
					     (u16)width, (u16)height);
	if (!cfg_size) {
		ret = -EINVAL;
		goto err_build;
	}

	session->cfg_stream_size = cfg_size;

	/* Pre-fill config descriptor (static per session) */
	session->cfg_desc->next_descpt_ptr = lower_32_bits(session->desc_dma) |
					     MXC_NXT_DESCPT_EN;
	session->cfg_desc->buf_base0   = lower_32_bits(session->cfg_stream_dma);
	session->cfg_desc->buf_base1   = 0;
	session->cfg_desc->line_pitch  = 0;
	session->cfg_desc->stm_bufbase = 0;
	session->cfg_desc->stm_bufsize = 0;
	session->cfg_desc->imgsize     = 0;
	session->cfg_desc->stm_ctrl    = STM_CTRL_CONFIG_MOD(1) | STM_CTRL_BITBUF_PTR_CLR(1);

	/* Pre-fill encode descriptor (in/out DMA patched per frame) */
	session->desc->next_descpt_ptr = 0;

	mxc_jpeg_set_res(session->desc, (u16)width, (u16)height);
	mxc_jpeg_set_line_pitch(session->desc, width * 3u); /* BGR24 */

	session->desc->stm_ctrl  = STM_CTRL_CONFIG_MOD(0) |
				   STM_CTRL_IMAGE_FORMAT(MXC_JPEG_BGR) |
				   STM_CTRL_BITBUF_PTR_CLR(1);
	session->desc->stm_ctrl &= ~STM_CTRL_PIXEL_PRECISION;
	session->desc->buf_base1 = 0;

	/* Power up HW once for the entire session */
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		dev_err(dev, "enc_api: pm_runtime failed (%d)\n", ret);
		goto err_build;
	}

	spin_lock_irqsave(&jpeg_dev->g_session_list_lock, flags);
	list_add(&session->node, &jpeg_dev->g_session_list);
	spin_unlock_irqrestore(&jpeg_dev->g_session_list_lock, flags);

	dev_info(dev, "enc_api: session created %ux%u q=%u slot=%d\n",
		 width, height, quality, session->slot);

	return session;

err_build:
	dma_free_coherent(dev, MXC_JPEG_MAX_CFG_STREAM,
			  session->cfg_stream, session->cfg_stream_dma);
err_cfg_stream:
	dma_free_coherent(dev, sizeof(struct mxc_jpeg_desc),
			  session->cfg_desc, session->cfg_desc_dma);
err_cfg_desc:
	dma_free_coherent(dev, sizeof(struct mxc_jpeg_desc),
			  session->desc, session->desc_dma);
err_desc:
	kfree(session);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(mxc_jpeg_enc_session_create);

void mxc_jpeg_enc_session_destroy(struct mxc_jpeg_enc_session *session)
{
	unsigned long flags;

	if (!session)
		return;

	spin_lock_irqsave(&session->jpeg->g_session_list_lock, flags);
	list_del(&session->node);
	spin_unlock_irqrestore(&session->jpeg->g_session_list_lock, flags);

	spin_lock_irqsave(&session->jpeg->hw_lock, flags);
	if (session->active) {
		dev_warn(session->dev,
			 "enc_api: destroy called with active encode — aborting slot %d\n",
			 session->slot);
		/*
		 * Null done_cb first: irq_notify checks for NULL before
		 * invoking it, so any IRQ already latched in the GIC cannot
		 * call back into memory we are about to free.
		 */
		session->done_cb   = NULL;
		session->cb_data   = NULL;
		session->active    = false;
		session->enc_state = MXC_JPEG_ENC_CONF_API;
		atomic_set(&session->jpeg->api_hw_active, 0);
		mxc_jpeg_disable_irq(session->jpeg->base_reg, session->slot);
		mxc_jpeg_sw_reset(session->jpeg->base_reg);
	}
	spin_unlock_irqrestore(&session->jpeg->hw_lock, flags);

	pm_runtime_put_sync(session->dev);

	dma_free_coherent(session->dev, MXC_JPEG_MAX_CFG_STREAM,
			  session->cfg_stream, session->cfg_stream_dma);
	dma_free_coherent(session->dev, sizeof(struct mxc_jpeg_desc),
			  session->cfg_desc, session->cfg_desc_dma);
	dma_free_coherent(session->dev, sizeof(struct mxc_jpeg_desc),
			  session->desc, session->desc_dma);

	session->jpeg->slot_data.used = false;

	kfree(session);
}
EXPORT_SYMBOL_GPL(mxc_jpeg_enc_session_destroy);

static int mxc_jpeg_enc_session_encode_async(struct mxc_jpeg_enc_session *session,
					     dma_addr_t in_dma, dma_addr_t out_dma,
					     size_t out_buf_size,
					     void (*done_cb)(int ret, size_t jpeg_size,
							     void *data),
					     void *cb_data)
{
	struct mxc_jpeg_dev *jpeg;
	void __iomem *reg;
	struct device *dev;
	int slot;
	unsigned long irq_flags;

	/* NULL check MUST come before any dereference of session */
	if (!session || !done_cb)
		return -EINVAL;

	jpeg = session->jpeg;
	reg  = jpeg->base_reg;
	dev  = session->dev;
	slot = session->slot;

	if (!IS_ALIGNED(in_dma,  MXC_JPEG_ADDR_ALIGNMENT) ||
	    !IS_ALIGNED(out_dma, MXC_JPEG_ADDR_ALIGNMENT))
		return -EINVAL;

	if (mutex_lock_interruptible(&session->lock))
		return -ERESTARTSYS;

	if (session->active) {
		mutex_unlock(&session->lock);
		return -EBUSY;
	}

	session->desc->buf_base0   = lower_32_bits(in_dma);
	session->desc->stm_bufbase = lower_32_bits(out_dma);
	mxc_jpeg_set_bufsize(session->desc, ALIGN(out_buf_size, 1024));

	session->out_buf_size = out_buf_size;

	mutex_unlock(&session->lock);

	spin_lock_irqsave(&jpeg->hw_lock, irq_flags);

	session->done_cb   = done_cb;
	session->cb_data   = cb_data;
	session->enc_state = MXC_JPEG_ENC_CONF_API;
	session->active    = true;

	atomic_set(&jpeg->api_hw_active, 1);

	mxc_jpeg_enable(reg);
	mxc_jpeg_set_l_endian(reg, 1);

	/* Set descriptor BEFORE enabling slot/IRQ to avoid race */
	mxc_jpeg_set_desc(lower_32_bits(session->cfg_desc_dma), reg, slot);

	mxc_jpeg_enable_slot(reg, slot);
	mxc_jpeg_enable_irq(reg, slot);

	mxc_jpeg_enc_mode_conf(dev, reg, slot);

	dev_dbg(dev,
		"enc_api: HW kicked slot=%d cfg_desc_dma=0x%x in=0x%x out=0x%x\n",
		slot, lower_32_bits(session->cfg_desc_dma),
		lower_32_bits(in_dma), lower_32_bits(out_dma));

	spin_unlock_irqrestore(&jpeg->hw_lock, irq_flags);

	return 0;
}

static void enc_sync_cb(int ret, size_t jpeg_size, void *data)
{
	struct enc_sync_ctx *ctx = data;

	ctx->ret       = ret;
	ctx->jpeg_size = jpeg_size;
	complete(&ctx->done);
}

int mxc_jpeg_enc_session_encode(struct mxc_jpeg_enc_session *session,
				dma_addr_t in_dma, dma_addr_t out_dma,
				size_t out_buf_size, size_t *out_size)
{
	struct enc_sync_ctx ctx;
	long timeout;
	int ret;

	if (!session || !out_size)
		return -EINVAL;

	init_completion(&ctx.done);
	ctx.ret       = 0;
	ctx.jpeg_size = 0;

	ret = mxc_jpeg_enc_session_encode_async(session, in_dma, out_dma,
						out_buf_size, enc_sync_cb, &ctx);
	if (ret)
		return ret;

	timeout = wait_for_completion_timeout(&ctx.done, msecs_to_jiffies(500));
	if (timeout == 0) {
		dev_err(session->dev, "enc_api: encode timeout - clearing slot\n");

		spin_lock_irq(&session->jpeg->hw_lock);
		session->done_cb   = NULL;
		session->cb_data   = NULL;
		session->active    = false;
		session->enc_state = MXC_JPEG_ENC_CONF_API;
		atomic_set(&session->jpeg->api_hw_active, 0);
		mxc_jpeg_disable_irq(session->jpeg->base_reg, session->slot);
		mxc_jpeg_sw_reset(session->jpeg->base_reg);
		spin_unlock_irq(&session->jpeg->hw_lock);

		return -ETIMEDOUT;
	}

	if (ctx.ret)
		return ctx.ret;

	*out_size = ctx.jpeg_size;

	return 0;
}
EXPORT_SYMBOL_GPL(mxc_jpeg_enc_session_encode);

MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX8 JPEG encoder in-kernel API - BGR24 to JPEG");
MODULE_LICENSE("GPL v2");
