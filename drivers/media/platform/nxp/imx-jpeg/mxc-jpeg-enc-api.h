/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mxc-jpeg-enc-api.h - In-kernel API for BGR24 -> JPEG encoding
 * Copyright 2026 NXP
 */

#ifndef _MXC_JPEG_ENC_API_H
#define _MXC_JPEG_ENC_API_H

#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/list.h>

struct mxc_jpeg_dev;
struct mxc_jpeg_desc;

/**
 * struct mxc_jpeg_enc_session - per-session state for the in-kernel encode API
 * @jpeg:             pointer to the parent mxc_jpeg_dev
 * @dev:              convenience pointer to the device (same as jpeg->dev)
 * @width:            frame width in pixels
 * @height:           frame height in pixels
 * @quality:          JPEG quality factor (1-100)
 * @slot:             HW slot index used by this session
 * @desc:             private encode DMA descriptor (coherent)
 * @desc_dma:         DMA address of @desc
 * @cfg_desc:         private config DMA descriptor (coherent)
 * @cfg_desc_dma:     DMA address of @cfg_desc
 * @cfg_stream:       config bitstream buffer (coherent)
 * @cfg_stream_dma:   DMA address of @cfg_stream
 * @cfg_stream_size:  valid byte count in @cfg_stream
 * @active:           true while HW is processing a frame
 * @irq_status:       last slot status word from IRQ
 * @done_cb:          completion callback (called from IRQ context)
 * @cb_data:          opaque pointer passed to @done_cb
 * @enc_state:        current state machine state
 * @lock:             serialises concurrent encode calls
 * @node:             linkage in g_session_list
 */
struct mxc_jpeg_enc_session {
	struct mxc_jpeg_dev    *jpeg;
	struct device          *dev;

	u32                     width;
	u32                     height;
	u8                      quality;
	int                     slot;

	/* Private DMA descriptors - isolated from V4L2 slot_data */
	struct mxc_jpeg_desc   *desc;
	dma_addr_t              desc_dma;
	struct mxc_jpeg_desc   *cfg_desc;
	dma_addr_t              cfg_desc_dma;
	void                   *cfg_stream;
	dma_addr_t              cfg_stream_dma;
	unsigned int            cfg_stream_size;

	/* IRQ state */
	bool                    active;
	u32                     irq_status;

	/* Async callback - called from IRQ context */
	void (*done_cb)(int ret, size_t jpeg_size, void *data);
	void                   *cb_data;

	/* Encode state machine */
	enum {
		MXC_JPEG_ENCODING_API	= 0, /* jpeg encode phase */
		MXC_JPEG_ENC_CONF_API	= 1, /* jpeg encoder config phase */
	} enc_state;

	struct mutex            lock;
	struct list_head        node;
	size_t					out_buf_size;
};

struct enc_sync_ctx {
	struct completion done;
	int               ret;
	size_t            jpeg_size;
};

/* Session lifecycle */
struct mxc_jpeg_dev *mxc_jpeg_get_enc_dev(void);
struct device *mxc_jpeg_get_enc_device(struct mxc_jpeg_dev *jpeg_dev);

struct mxc_jpeg_enc_session *
mxc_jpeg_enc_session_create(struct mxc_jpeg_dev *jpeg_dev,
			    u32 width, u32 height, u8 quality);

void mxc_jpeg_enc_session_destroy(struct mxc_jpeg_enc_session *session);

/* Sync encode - blocking wrapper, use from kthread/wq only */
int mxc_jpeg_enc_session_encode(struct mxc_jpeg_enc_session *session,
				dma_addr_t in_dma, dma_addr_t out_dma,
				size_t out_buf_size, size_t *out_size);

/* IRQ hooks */
bool mxc_jpeg_enc_api_is_active(struct mxc_jpeg_dev *jpeg, int slot);
void mxc_jpeg_enc_api_irq_notify(struct mxc_jpeg_dev *jpeg,
				 int slot, u32 slot_status);

#endif /* _MXC_JPEG_ENC_API_H */
