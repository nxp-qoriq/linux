/*
 * Copyright 2015-2016 Freescale Semiconductor Inc.
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *	 names of any contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "compat.h"
#include "regs.h"
#include "caamalg_qi2.h"
#include "dpseci_cmd.h"
#include "desc_constr.h"
#include "error.h"
#include "sg_sw_sec4.h"
#include "sg_sw_qm2.h"
#include "key_gen.h"
#include "caamalg_desc.h"
#include "../../../drivers/staging/fsl-mc/include/mc.h"
#include "../../../drivers/staging/fsl-mc/include/dpaa2-io.h"
#include "../../../drivers/staging/fsl-mc/include/dpaa2-fd.h"

#define CAAM_CRA_PRIORITY	2000

/* max key is sum of AES_MAX_KEY_SIZE, max split key size */
#define CAAM_MAX_KEY_SIZE	(AES_MAX_KEY_SIZE + CTR_RFC3686_NONCE_SIZE + \
				 SHA512_DIGEST_SIZE * 2)

/* Length of a single buffer in the QI driver memory cache */
#define CAAM_QI_MEMCACHE_SIZE	512

#define DPSECI_VERS(maj, min)	(((maj) << 16) | (min))

/* TODO: temporary workaround for MC f/w not providing CAAM endianness info */
#ifndef CONFIG_CRYPTO_DEV_FSL_CAAM
bool caam_little_end;
EXPORT_SYMBOL(caam_little_end);
bool caam_imx;
EXPORT_SYMBOL(caam_imx);
#endif

/*
 * This is a a cache of buffers, from which the users of CAAM QI driver
 * can allocate short buffers. It's speedier than doing kmalloc on the hotpath.
 * NOTE: A more elegant solution would be to have some headroom in the frames
 *       being processed. This can be added by the dpaa2-eth driver. This would
 *       pose a problem for userspace application processing which cannot
 *       know of this limitation. So for now, this will work.
 * NOTE: The memcache is SMP-safe. No need to handle spinlocks in-here
 */
static struct kmem_cache *qi_cache;

struct caam_alg_entry {
	struct device *dev;
	int class1_alg_type;
	int class2_alg_type;
	bool rfc3686;
	bool geniv;
};

struct caam_aead_alg {
	struct aead_alg aead;
	struct caam_alg_entry caam;
	bool registered;
};

/**
 * caam_ctx - per-session context
 * @flc: Flow Contexts array
 * @flc_dma: DMA addresses of the Flow Contexts
 * @key:  virtual address of the key(s): [authentication key], encryption key
 * @key_dma: I/O virtual address of the key
 * @dev: dpseci device
 * @adata: authentication algorithm details
 * @cdata: encryption algorithm details
 * @authsize: authentication tag (a.k.a. ICV / MAC) size
 */
struct caam_ctx {
	/* TODO: Group flc and flc_dma in a struct ?! */
	struct caam_flc flc[NUM_OP];
	dma_addr_t flc_dma[NUM_OP];
	u8 key[CAAM_MAX_KEY_SIZE];
	dma_addr_t key_dma;
	struct device *dev;
	struct alginfo adata;
	struct alginfo cdata;
	unsigned int authsize;
};

/*
 * qi_cache_alloc - Allocate buffers from CAAM-QI cache
 *
 * Allocate data on the hotpath. Instead of using kmalloc, one can use the
 * services of the CAAM QI memory cache (backed by kmem_cache). The buffers
 * will have a size of CAAM_QI_MEMCACHE_SIZE, which should be sufficient for
 * hosting 16 SG entries.
 *
 * @flags - flags that would be used for the equivalent kmalloc(..) call
 *
 * Returns a pointer to a retrieved buffer on success or NULL on failure.
 */
static inline void *qi_cache_alloc(gfp_t flags)
{
	return kmem_cache_alloc(qi_cache, flags);
}

/*
 * qi_cache_free - Frees buffers allocated from CAAM-QI cache
 *
 * @obj - buffer previously allocated by qi_cache_alloc
 *
 * No checking is being done, the call is a passthrough call to
 * kmem_cache_free(...)
 */
static inline void qi_cache_free(void *obj)
{
	kmem_cache_free(qi_cache, obj);
}

static struct caam_request *to_caam_req(struct crypto_async_request *areq)
{
	switch (crypto_tfm_alg_type(areq->tfm)) {
	case CRYPTO_ALG_TYPE_ABLKCIPHER:
	case CRYPTO_ALG_TYPE_GIVCIPHER:
		return ablkcipher_request_ctx(ablkcipher_request_cast(areq));
	case CRYPTO_ALG_TYPE_AEAD:
		return aead_request_ctx(container_of(areq, struct aead_request,
						     base));
	default:
		return ERR_PTR(-EINVAL);
	}
}

static void caam_unmap(struct device *dev, struct scatterlist *src,
		       struct scatterlist *dst, int src_nents,
		       int dst_nents, dma_addr_t iv_dma, int ivsize,
		       enum optype op_type, dma_addr_t qm_sg_dma,
		       int qm_sg_bytes)
{
	if (dst != src) {
		if (src_nents)
			dma_unmap_sg(dev, src, src_nents, DMA_TO_DEVICE);
		dma_unmap_sg(dev, dst, dst_nents, DMA_FROM_DEVICE);
	} else {
		dma_unmap_sg(dev, src, src_nents, DMA_BIDIRECTIONAL);
	}

	if (iv_dma)
		dma_unmap_single(dev, iv_dma, ivsize,
				 op_type == GIVENCRYPT ? DMA_FROM_DEVICE :
							 DMA_TO_DEVICE);

	if (qm_sg_bytes)
		dma_unmap_single(dev, qm_sg_dma, qm_sg_bytes, DMA_TO_DEVICE);
}

static int aead_set_sh_desc(struct crypto_aead *aead)
{
	struct caam_aead_alg *alg = container_of(crypto_aead_alg(aead),
						 typeof(*alg), aead);
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct device *dev = ctx->dev;
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	u32 *desc;
	u32 ctx1_iv_off = 0;
	u32 *nonce = NULL;
	unsigned int data_len[2];
	u32 inl_mask;
	const bool ctr_mode = ((ctx->cdata.algtype & OP_ALG_AAI_MASK) ==
			       OP_ALG_AAI_CTR_MOD128);
	const bool is_rfc3686 = alg->caam.rfc3686;

	if (!ctx->cdata.keylen || !ctx->authsize)
		return 0;

	/*
	 * AES-CTR needs to load IV in CONTEXT1 reg
	 * at an offset of 128bits (16bytes)
	 * CONTEXT1[255:128] = IV
	 */
	if (ctr_mode)
		ctx1_iv_off = 16;

	/*
	 * RFC3686 specific:
	 *	CONTEXT1[255:128] = {NONCE, IV, COUNTER}
	 */
	if (is_rfc3686) {
		ctx1_iv_off = 16 + CTR_RFC3686_NONCE_SIZE;
		nonce = (u32 *)((void *)ctx->key + ctx->adata.keylen_pad +
				ctx->cdata.keylen - CTR_RFC3686_NONCE_SIZE);
	}

	data_len[0] = ctx->adata.keylen_pad;
	data_len[1] = ctx->cdata.keylen;

	if (alg->caam.geniv)
		goto skip_enc;

	/* aead_encrypt shared descriptor */
	if (desc_inline_query(DESC_QI_AEAD_ENC_LEN +
			      (is_rfc3686 ? DESC_AEAD_CTR_RFC3686_LEN : 0),
			      DESC_JOB_IO_LEN, data_len, &inl_mask,
			      ARRAY_SIZE(data_len)) < 0)
		return -EINVAL;

	if (inl_mask & 1)
		ctx->adata.key_virt = ctx->key;
	else
		ctx->adata.key_dma = ctx->key_dma;

	if (inl_mask & 2)
		ctx->cdata.key_virt = ctx->key + ctx->adata.keylen_pad;
	else
		ctx->cdata.key_dma = ctx->key_dma + ctx->adata.keylen_pad;

	ctx->adata.key_inline = !!(inl_mask & 1);
	ctx->cdata.key_inline = !!(inl_mask & 2);

	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_aead_encap(desc, &ctx->cdata, &ctx->adata,
			       ivsize, ctx->authsize, is_rfc3686, nonce,
			       ctx1_iv_off, true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

skip_enc:
	/* aead_decrypt shared descriptor */
	if (desc_inline_query(DESC_QI_AEAD_DEC_LEN +
			      (is_rfc3686 ? DESC_AEAD_CTR_RFC3686_LEN : 0),
			      DESC_JOB_IO_LEN, data_len, &inl_mask,
			      ARRAY_SIZE(data_len)) < 0)
		return -EINVAL;

	if (inl_mask & 1)
		ctx->adata.key_virt = ctx->key;
	else
		ctx->adata.key_dma = ctx->key_dma;

	if (inl_mask & 2)
		ctx->cdata.key_virt = ctx->key + ctx->adata.keylen_pad;
	else
		ctx->cdata.key_dma = ctx->key_dma + ctx->adata.keylen_pad;

	ctx->adata.key_inline = !!(inl_mask & 1);
	ctx->cdata.key_inline = !!(inl_mask & 2);

	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_aead_decap(desc, &ctx->cdata, &ctx->adata,
			       ivsize, ctx->authsize, alg->caam.geniv,
			       is_rfc3686, nonce, ctx1_iv_off, true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	if (!alg->caam.geniv)
		goto skip_givenc;

	/* aead_givencrypt shared descriptor */
	if (desc_inline_query(DESC_QI_AEAD_GIVENC_LEN +
			      (is_rfc3686 ? DESC_AEAD_CTR_RFC3686_LEN : 0),
			      DESC_JOB_IO_LEN, data_len, &inl_mask,
			      ARRAY_SIZE(data_len)) < 0)
		return -EINVAL;

	if (inl_mask & 1)
		ctx->adata.key_virt = ctx->key;
	else
		ctx->adata.key_dma = ctx->key_dma;

	if (inl_mask & 2)
		ctx->cdata.key_virt = ctx->key + ctx->adata.keylen_pad;
	else
		ctx->cdata.key_dma = ctx->key_dma + ctx->adata.keylen_pad;

	ctx->adata.key_inline = !!(inl_mask & 1);
	ctx->cdata.key_inline = !!(inl_mask & 2);

	flc = &ctx->flc[GIVENCRYPT];
	flc_dma = &ctx->flc_dma[GIVENCRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_aead_givencap(desc, &ctx->cdata, &ctx->adata,
				  ivsize, ctx->authsize, is_rfc3686, nonce,
				  ctx1_iv_off, true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

skip_givenc:
	return 0;
}

static int aead_setauthsize(struct crypto_aead *authenc, unsigned int authsize)
{
	struct caam_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	aead_set_sh_desc(authenc);

	return 0;
}

struct split_key_sh_result {
	struct completion completion;
	int err;
	struct device *dev;
};

static void split_key_sh_done(void *cbk_ctx, u32 err)
{
	struct split_key_sh_result *res = cbk_ctx;

#ifdef DEBUG
	dev_err(res->dev, "%s %d: err 0x%x\n", __func__, __LINE__, err);
#endif

	if (err)
		caam_qi2_strstatus(res->dev, err);

	res->err = err;
	complete(&res->completion);
}

static int gen_split_key_sh(struct device *dev, u8 *key_out,
			    struct alginfo * const adata, const u8 *key_in,
			    u32 keylen)
{
	struct caam_request *req_ctx;
	u32 *desc;
	struct split_key_sh_result result;
	dma_addr_t dma_addr_in, dma_addr_out, flc_dma;
	struct caam_flc *flc;
	struct dpaa2_fl_entry *in_fle, *out_fle;
	int ret = -ENOMEM;

	req_ctx = kzalloc(sizeof(*req_ctx), GFP_KERNEL | GFP_DMA);
	if (!req_ctx)
		return -ENOMEM;

	in_fle = &req_ctx->fd_flt[1];
	out_fle = &req_ctx->fd_flt[0];

	flc = kzalloc(sizeof(*flc), GFP_KERNEL | GFP_DMA);
	if (!flc)
		goto err_flc;

	dma_addr_in = dma_map_single(dev, (void *)key_in, keylen,
				     DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr_in)) {
		dev_err(dev, "unable to map key input memory\n");
		goto err_dma_addr_in;
	}

	dma_addr_out = dma_map_single(dev, key_out, adata->keylen_pad,
				      DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, dma_addr_out)) {
		dev_err(dev, "unable to map key output memory\n");
		goto err_dma_addr_out;
	}

	desc = flc->sh_desc;

	init_sh_desc(desc, 0);
	append_key(desc, dma_addr_in, keylen, CLASS_2 | KEY_DEST_CLASS_REG);

	/* Sets MDHA up into an HMAC-INIT */
	append_operation(desc, (adata->algtype & OP_ALG_ALGSEL_MASK) |
			 OP_ALG_AAI_HMAC | OP_TYPE_CLASS2_ALG | OP_ALG_DECRYPT |
			 OP_ALG_AS_INIT);

	/*
	 * do a FIFO_LOAD of zero, this will trigger the internal key expansion
	 * into both pads inside MDHA
	 */
	append_fifo_load_as_imm(desc, NULL, 0, LDST_CLASS_2_CCB |
				FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST2);

	/*
	 * FIFO_STORE with the explicit split-key content store
	 * (0x26 output type)
	 */
	append_fifo_store(desc, dma_addr_out, adata->keylen,
			  LDST_CLASS_2_CCB | FIFOST_TYPE_SPLIT_KEK);

	flc->flc[1] = desc_len(desc); /* SDL */
	flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) + desc_bytes(desc),
				 DMA_TO_DEVICE);
	if (dma_mapping_error(dev, flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		goto err_flc_dma;
	}

	dpaa2_fl_set_final(in_fle, true);
	dpaa2_fl_set_format(in_fle, dpaa2_fl_single);
	dpaa2_fl_set_addr(in_fle, dma_addr_in);
	dpaa2_fl_set_len(in_fle, keylen);
	dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
	dpaa2_fl_set_addr(out_fle, dma_addr_out);
	dpaa2_fl_set_len(out_fle, adata->keylen_pad);

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "ctx.key@" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key_in, keylen, 1);
	print_hex_dump(KERN_ERR, "desc@" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, desc, desc_bytes(desc), 1);
#endif

	result.err = 0;
	init_completion(&result.completion);
	result.dev = dev;

	req_ctx->flc = flc;
	req_ctx->flc_dma = flc_dma;
	req_ctx->cbk = split_key_sh_done;
	req_ctx->ctx = &result;

	ret = dpaa2_caam_enqueue(dev, req_ctx);
	if (ret == -EINPROGRESS) {
		/* in progress */
		wait_for_completion(&result.completion);
		ret = result.err;
#ifdef DEBUG
		print_hex_dump(KERN_ERR, "ctx.key@" __stringify(__LINE__)": ",
			       DUMP_PREFIX_ADDRESS, 16, 4, key_out,
			       adata->keylen_pad, 1);
#endif
	}

	dma_unmap_single(dev, flc_dma, sizeof(flc->flc) + desc_bytes(desc),
			 DMA_TO_DEVICE);
err_flc_dma:
	dma_unmap_single(dev, dma_addr_out, adata->keylen_pad, DMA_FROM_DEVICE);
err_dma_addr_out:
	dma_unmap_single(dev, dma_addr_in, keylen, DMA_TO_DEVICE);
err_dma_addr_in:
	kfree(flc);
err_flc:
	kfree(req_ctx);
	return ret;
}

static int gen_split_aead_key(struct caam_ctx *ctx, const u8 *key_in,
			      u32 authkeylen)
{
	return gen_split_key_sh(ctx->dev, ctx->key, &ctx->adata, key_in,
				authkeylen);
}

static int aead_setkey(struct crypto_aead *aead, const u8 *key,
		       unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	struct crypto_authenc_keys keys;
	int ret;

	if (crypto_authenc_extractkeys(&keys, key, keylen) != 0)
		goto badkey;

#ifdef DEBUG
	dev_err(dev, "keylen %d enckeylen %d authkeylen %d\n",
		keys.authkeylen + keys.enckeylen, keys.enckeylen,
		keys.authkeylen);
	print_hex_dump(KERN_ERR, "key in @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key, keylen, 1);
#endif

	ctx->adata.keylen = split_key_len(ctx->adata.algtype &
					  OP_ALG_ALGSEL_MASK);
	ctx->adata.keylen_pad = split_key_pad_len(ctx->adata.algtype &
						  OP_ALG_ALGSEL_MASK);

#ifdef DEBUG
	dev_err(dev, "split keylen %d split keylen padded %d\n",
		ctx->adata.keylen, ctx->adata.keylen_pad);
	print_hex_dump(KERN_ERR, "ctx.key@" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, keys.authkey, keylen, 1);
#endif

	if (ctx->adata.keylen_pad + keys.enckeylen > CAAM_MAX_KEY_SIZE)
		goto badkey;

	ret = gen_split_aead_key(ctx, keys.authkey, keys.authkeylen);
	if (ret)
		goto badkey;

	/* postpend encryption key to auth split key */
	memcpy(ctx->key + ctx->adata.keylen_pad, keys.enckey, keys.enckeylen);

	ctx->key_dma = dma_map_single(dev, ctx->key, ctx->adata.keylen_pad +
				      keys.enckeylen, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}
#ifdef DEBUG
	print_hex_dump(KERN_ERR, "ctx.key@" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, ctx->key,
		       ctx->adata.keylen_pad + keys.enckeylen, 1);
#endif

	ctx->cdata.keylen = keys.enckeylen;

	ret = aead_set_sh_desc(aead);
	if (ret)
		dma_unmap_single(dev, ctx->key_dma, ctx->adata.keylen_pad +
				 keys.enckeylen, DMA_TO_DEVICE);

	return ret;
badkey:
	crypto_aead_set_flags(aead, CRYPTO_TFM_RES_BAD_KEY_LEN);
	return -EINVAL;
}

static struct aead_edesc *aead_edesc_alloc(struct aead_request *req,
					   bool encrypt)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct caam_request *req_ctx = aead_request_ctx(req);
	struct dpaa2_fl_entry *in_fle = &req_ctx->fd_flt[1];
	struct dpaa2_fl_entry *out_fle = &req_ctx->fd_flt[0];
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct caam_aead_alg *alg = container_of(crypto_aead_alg(aead),
						 typeof(*alg), aead);
	struct device *dev = ctx->dev;
	gfp_t flags = (req->base.flags & (CRYPTO_TFM_REQ_MAY_BACKLOG |
		       CRYPTO_TFM_REQ_MAY_SLEEP)) ? GFP_KERNEL : GFP_ATOMIC;
	int src_nents, mapped_src_nents, dst_nents = 0, mapped_dst_nents = 0;
	struct aead_edesc *edesc;
	dma_addr_t qm_sg_dma, iv_dma = 0;
	int ivsize = 0;
	unsigned int authsize = ctx->authsize;
	int qm_sg_index = 0, qm_sg_nents = 0, qm_sg_bytes;
	int in_len, out_len;
	struct dpaa2_sg_entry *sg_table;
	enum optype op_type = encrypt ? ENCRYPT : DECRYPT;

	/* allocate space for base edesc and link tables */
	edesc = qi_cache_alloc(GFP_DMA | flags);
	if (unlikely(!edesc)) {
		dev_err(dev, "could not allocate extended descriptor\n");
		return ERR_PTR(-ENOMEM);
	}

	if (unlikely(req->dst != req->src)) {
		src_nents = sg_nents_for_len(req->src, req->assoclen +
					     req->cryptlen);
		if (unlikely(src_nents < 0)) {
			dev_err(dev, "Insufficient bytes (%d) in src S/G\n",
				req->assoclen + req->cryptlen);
			qi_cache_free(edesc);
			return ERR_PTR(src_nents);
		}

		dst_nents = sg_nents_for_len(req->dst, req->assoclen +
					     req->cryptlen +
					     (encrypt ? authsize :
							(-authsize)));
		if (unlikely(dst_nents < 0)) {
			dev_err(dev, "Insufficient bytes (%d) in dst S/G\n",
				req->assoclen + req->cryptlen +
				(encrypt ? authsize : (-authsize)));
			qi_cache_free(edesc);
			return ERR_PTR(dst_nents);
		}

		if (src_nents) {
			mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
						      DMA_TO_DEVICE);
			if (unlikely(!mapped_src_nents)) {
				dev_err(dev, "unable to map source\n");
				qi_cache_free(edesc);
				return ERR_PTR(-ENOMEM);
			}
		} else {
			mapped_src_nents = 0;
		}

		mapped_dst_nents = dma_map_sg(dev, req->dst, dst_nents,
					      DMA_FROM_DEVICE);
		if (unlikely(!mapped_dst_nents)) {
			dev_err(dev, "unable to map destination\n");
			dma_unmap_sg(dev, req->src, src_nents, DMA_TO_DEVICE);
			qi_cache_free(edesc);
			return ERR_PTR(-ENOMEM);
		}
	} else {
		src_nents = sg_nents_for_len(req->src, req->assoclen +
					     req->cryptlen +
						(encrypt ? authsize : 0));
		if (unlikely(src_nents < 0)) {
			dev_err(dev, "Insufficient bytes (%d) in src S/G\n",
				req->assoclen + req->cryptlen +
				(encrypt ? authsize : 0));
			qi_cache_free(edesc);
			return ERR_PTR(src_nents);
		}

		mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
					      DMA_BIDIRECTIONAL);
		if (unlikely(!mapped_src_nents)) {
			dev_err(dev, "unable to map source\n");
			qi_cache_free(edesc);
			return ERR_PTR(-ENOMEM);
		}
	}

	if ((alg->caam.rfc3686 && encrypt) || !alg->caam.geniv) {
		ivsize = crypto_aead_ivsize(aead);
		iv_dma = dma_map_single(dev, req->iv, ivsize, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, iv_dma)) {
			dev_err(dev, "unable to map IV\n");
			caam_unmap(dev, req->src, req->dst, src_nents,
				   dst_nents, 0, 0, op_type, 0, 0);
			qi_cache_free(edesc);
			return ERR_PTR(-ENOMEM);
		}
	}

	/*
	 * Create S/G table: req->assoclen, [IV,] req->src [, req->dst].
	 * Input is not contiguous.
	 */
	qm_sg_nents = 1 + !!ivsize + mapped_src_nents +
		      (mapped_dst_nents > 1 ? mapped_dst_nents : 0);
	sg_table = &edesc->sgt[0];
	qm_sg_bytes = qm_sg_nents * sizeof(*sg_table);

	edesc->src_nents = src_nents;
	edesc->dst_nents = dst_nents;
	edesc->iv_dma = iv_dma;

	edesc->assoclen_dma = dma_map_single(dev, &req->assoclen, 4,
					     DMA_TO_DEVICE);
	if (dma_mapping_error(dev, edesc->assoclen_dma)) {
		dev_err(dev, "unable to map assoclen\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, op_type, 0, 0);
		qi_cache_free(edesc);
		return ERR_PTR(-ENOMEM);
	}

	dma_to_qm_sg_one(sg_table, edesc->assoclen_dma, 4, 0);
	qm_sg_index++;
	if (ivsize) {
		dma_to_qm_sg_one(sg_table + qm_sg_index, iv_dma, ivsize, 0);
		qm_sg_index++;
	}
	sg_to_qm_sg_last(req->src, mapped_src_nents, sg_table + qm_sg_index, 0);
	qm_sg_index += mapped_src_nents;

	if (mapped_dst_nents > 1)
		sg_to_qm_sg_last(req->dst, mapped_dst_nents, sg_table +
				 qm_sg_index, 0);

	qm_sg_dma = dma_map_single(dev, sg_table, qm_sg_bytes, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, qm_sg_dma)) {
		dev_err(dev, "unable to map S/G table\n");
		dma_unmap_single(dev, edesc->assoclen_dma, 4, DMA_TO_DEVICE);
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, op_type, 0, 0);
		qi_cache_free(edesc);
		return ERR_PTR(-ENOMEM);
	}

	edesc->qm_sg_dma = qm_sg_dma;
	edesc->qm_sg_bytes = qm_sg_bytes;

	out_len = req->assoclen + req->cryptlen +
		  (encrypt ? ctx->authsize : (-ctx->authsize));
	in_len = 4 + ivsize + req->assoclen + req->cryptlen;

	memset(&req_ctx->fd_flt, 0, sizeof(req_ctx->fd_flt));
	dpaa2_fl_set_final(in_fle, true);
	dpaa2_fl_set_format(in_fle, dpaa2_fl_sg);
	dpaa2_fl_set_addr(in_fle, qm_sg_dma);
	dpaa2_fl_set_len(in_fle, in_len);

	if (req->dst == req->src) {
		if (mapped_src_nents == 1) {
			dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
			dpaa2_fl_set_addr(out_fle, sg_dma_address(req->src));
		} else {
			dpaa2_fl_set_format(out_fle, dpaa2_fl_sg);
			dpaa2_fl_set_addr(out_fle, qm_sg_dma +
					  (1 + !!ivsize) * sizeof(*sg_table));
		}
	} else if (mapped_dst_nents == 1) {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
		dpaa2_fl_set_addr(out_fle, sg_dma_address(req->dst));
	} else {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_sg);
		dpaa2_fl_set_addr(out_fle, qm_sg_dma + qm_sg_index *
				  sizeof(*sg_table));
	}

	dpaa2_fl_set_len(out_fle, out_len);

	return edesc;
}

static int gcm_set_sh_desc(struct crypto_aead *aead)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	u32 *desc;
	int rem_bytes = CAAM_DESC_BYTES_MAX - DESC_JOB_IO_LEN -
			ctx->cdata.keylen;

	if (!ctx->cdata.keylen || !ctx->authsize)
		return 0;

	/*
	 * AES GCM encrypt shared descriptor
	 * Job Descriptor and Shared Descriptor
	 * must fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_GCM_ENC_LEN) {
		ctx->cdata.key_inline = true;
		ctx->cdata.key_virt = ctx->key;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_gcm_encap(desc, &ctx->cdata, ivsize, ctx->authsize, true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/*
	 * Job Descriptor and Shared Descriptors
	 * must all fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_GCM_DEC_LEN) {
		ctx->cdata.key_inline = true;
		ctx->cdata.key_virt = ctx->key;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_gcm_decap(desc, &ctx->cdata, ivsize, ctx->authsize, true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	return 0;
}

static int gcm_setauthsize(struct crypto_aead *authenc, unsigned int authsize)
{
	struct caam_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	gcm_set_sh_desc(authenc);

	return 0;
}

static int gcm_setkey(struct crypto_aead *aead,
		      const u8 *key, unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	int ret;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "key in @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key, keylen, 1);
#endif

	memcpy(ctx->key, key, keylen);
	ctx->key_dma = dma_map_single(dev, ctx->key, keylen, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}
	ctx->cdata.keylen = keylen;

	ret = gcm_set_sh_desc(aead);
	if (ret)
		dma_unmap_single(dev, ctx->key_dma, ctx->cdata.keylen,
				 DMA_TO_DEVICE);

	return ret;
}

static int rfc4106_set_sh_desc(struct crypto_aead *aead)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	u32 *desc;
	int rem_bytes = CAAM_DESC_BYTES_MAX - DESC_JOB_IO_LEN -
			ctx->cdata.keylen;

	if (!ctx->cdata.keylen || !ctx->authsize)
		return 0;

	ctx->cdata.key_virt = ctx->key;

	/*
	 * RFC4106 encrypt shared descriptor
	 * Job Descriptor and Shared Descriptor
	 * must fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_RFC4106_ENC_LEN) {
		ctx->cdata.key_inline = true;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_rfc4106_encap(desc, &ctx->cdata, ivsize, ctx->authsize,
				  true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/*
	 * Job Descriptor and Shared Descriptors
	 * must all fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_RFC4106_DEC_LEN) {
		ctx->cdata.key_inline = true;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_rfc4106_decap(desc, &ctx->cdata, ivsize, ctx->authsize,
				  true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	return 0;
}

static int rfc4106_setauthsize(struct crypto_aead *authenc,
			       unsigned int authsize)
{
	struct caam_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	rfc4106_set_sh_desc(authenc);

	return 0;
}

static int rfc4106_setkey(struct crypto_aead *aead,
			  const u8 *key, unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	int ret;

	if (keylen < 4)
		return -EINVAL;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "key in @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key, keylen, 1);
#endif

	memcpy(ctx->key, key, keylen);
	/*
	 * The last four bytes of the key material are used as the salt value
	 * in the nonce. Update the AES key length.
	 */
	ctx->cdata.keylen = keylen - 4;
	ctx->key_dma = dma_map_single(dev, ctx->key, ctx->cdata.keylen,
				      DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}

	ret = rfc4106_set_sh_desc(aead);
	if (ret)
		dma_unmap_single(dev, ctx->key_dma, ctx->cdata.keylen,
				 DMA_TO_DEVICE);

	return ret;
}

static int rfc4543_set_sh_desc(struct crypto_aead *aead)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	unsigned int ivsize = crypto_aead_ivsize(aead);
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	u32 *desc;
	int rem_bytes = CAAM_DESC_BYTES_MAX - DESC_JOB_IO_LEN -
			ctx->cdata.keylen;

	if (!ctx->cdata.keylen || !ctx->authsize)
		return 0;

	ctx->cdata.key_virt = ctx->key;

	/*
	 * RFC4543 encrypt shared descriptor
	 * Job Descriptor and Shared Descriptor
	 * must fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_RFC4543_ENC_LEN) {
		ctx->cdata.key_inline = true;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_rfc4543_encap(desc, &ctx->cdata, ivsize, ctx->authsize,
				  true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/*
	 * Job Descriptor and Shared Descriptors
	 * must all fit into the 64-word Descriptor h/w Buffer
	 */
	if (rem_bytes >= DESC_QI_RFC4543_DEC_LEN) {
		ctx->cdata.key_inline = true;
	} else {
		ctx->cdata.key_inline = false;
		ctx->cdata.key_dma = ctx->key_dma;
	}

	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_rfc4543_decap(desc, &ctx->cdata, ivsize, ctx->authsize,
				  true);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	return 0;
}

static int rfc4543_setauthsize(struct crypto_aead *authenc,
			       unsigned int authsize)
{
	struct caam_ctx *ctx = crypto_aead_ctx(authenc);

	ctx->authsize = authsize;
	rfc4543_set_sh_desc(authenc);

	return 0;
}

static int rfc4543_setkey(struct crypto_aead *aead,
			  const u8 *key, unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct device *dev = ctx->dev;
	int ret;

	if (keylen < 4)
		return -EINVAL;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "key in @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key, keylen, 1);
#endif

	memcpy(ctx->key, key, keylen);
	/*
	 * The last four bytes of the key material are used as the salt value
	 * in the nonce. Update the AES key length.
	 */
	ctx->cdata.keylen = keylen - 4;
	ctx->key_dma = dma_map_single(dev, ctx->key, ctx->cdata.keylen,
				      DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}

	ret = rfc4543_set_sh_desc(aead);
	if (ret)
		dma_unmap_single(dev, ctx->key_dma, ctx->cdata.keylen,
				 DMA_TO_DEVICE);

	return ret;
}

static int ablkcipher_setkey(struct crypto_ablkcipher *ablkcipher,
			     const u8 *key, unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(ablkcipher);
	const char *alg_name = crypto_tfm_alg_name(tfm);
	struct device *dev = ctx->dev;
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	unsigned int ivsize = crypto_ablkcipher_ivsize(ablkcipher);
	u32 *desc;
	u32 ctx1_iv_off = 0;
	const bool ctr_mode = ((ctx->cdata.algtype & OP_ALG_AAI_MASK) ==
			       OP_ALG_AAI_CTR_MOD128);
	const bool is_rfc3686 = (ctr_mode && strstr(alg_name, "rfc3686"));

	memcpy(ctx->key, key, keylen);
#ifdef DEBUG
	print_hex_dump(KERN_ERR, "key in @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, key, keylen, 1);
#endif
	/*
	 * AES-CTR needs to load IV in CONTEXT1 reg
	 * at an offset of 128bits (16bytes)
	 * CONTEXT1[255:128] = IV
	 */
	if (ctr_mode)
		ctx1_iv_off = 16;

	/*
	 * RFC3686 specific:
	 *	| CONTEXT1[255:128] = {NONCE, IV, COUNTER}
	 *	| *key = {KEY, NONCE}
	 */
	if (is_rfc3686) {
		ctx1_iv_off = 16 + CTR_RFC3686_NONCE_SIZE;
		keylen -= CTR_RFC3686_NONCE_SIZE;
	}

	ctx->key_dma = dma_map_single(dev, ctx->key, keylen, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}
	ctx->cdata.keylen = keylen;
	ctx->cdata.key_virt = ctx->key;
	ctx->cdata.key_inline = true;

	/* ablkcipher_encrypt shared descriptor */
	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_ablkcipher_encap(desc, &ctx->cdata, ivsize,
				     is_rfc3686, ctx1_iv_off);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/* ablkcipher_decrypt shared descriptor */
	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_ablkcipher_decap(desc, &ctx->cdata, ivsize,
				     is_rfc3686, ctx1_iv_off);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/* ablkcipher_givencrypt shared descriptor */
	flc = &ctx->flc[GIVENCRYPT];
	flc_dma = &ctx->flc_dma[GIVENCRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_ablkcipher_givencap(desc, &ctx->cdata,
					ivsize, is_rfc3686, ctx1_iv_off);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	return 0;
}

static int xts_ablkcipher_setkey(struct crypto_ablkcipher *ablkcipher,
				 const u8 *key, unsigned int keylen)
{
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct device *dev = ctx->dev;
	struct caam_flc *flc;
	dma_addr_t *flc_dma;
	u32 *desc;

	if (keylen != 2 * AES_MIN_KEY_SIZE  && keylen != 2 * AES_MAX_KEY_SIZE) {
		dev_err(dev, "key size mismatch\n");
		crypto_ablkcipher_set_flags(ablkcipher,
					    CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	memcpy(ctx->key, key, keylen);
	ctx->key_dma = dma_map_single(dev, ctx->key, keylen, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, ctx->key_dma)) {
		dev_err(dev, "unable to map key i/o memory\n");
		return -ENOMEM;
	}
	ctx->cdata.keylen = keylen;
	ctx->cdata.key_virt = ctx->key;
	ctx->cdata.key_inline = true;

	/* xts_ablkcipher_encrypt shared descriptor */
	flc = &ctx->flc[ENCRYPT];
	flc_dma = &ctx->flc_dma[ENCRYPT];
	desc = flc->sh_desc;
	cnstr_shdsc_xts_ablkcipher_encap(desc, &ctx->cdata);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	/* xts_ablkcipher_decrypt shared descriptor */
	flc = &ctx->flc[DECRYPT];
	flc_dma = &ctx->flc_dma[DECRYPT];
	desc = flc->sh_desc;

	cnstr_shdsc_xts_ablkcipher_decap(desc, &ctx->cdata);

	flc->flc[1] = desc_len(desc); /* SDL */
	*flc_dma = dma_map_single(dev, flc, sizeof(flc->flc) +
				  desc_bytes(desc), DMA_TO_DEVICE);
	if (dma_mapping_error(dev, *flc_dma)) {
		dev_err(dev, "unable to map shared descriptor\n");
		return -ENOMEM;
	}

	return 0;
}

static struct ablkcipher_edesc *ablkcipher_edesc_alloc(struct ablkcipher_request
						       *req, bool encrypt)
{
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_request *req_ctx = ablkcipher_request_ctx(req);
	struct dpaa2_fl_entry *in_fle = &req_ctx->fd_flt[1];
	struct dpaa2_fl_entry *out_fle = &req_ctx->fd_flt[0];
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct device *dev = ctx->dev;
	gfp_t flags = (req->base.flags & (CRYPTO_TFM_REQ_MAY_BACKLOG |
					  CRYPTO_TFM_REQ_MAY_SLEEP)) ?
		       GFP_KERNEL : GFP_ATOMIC;
	int src_nents, mapped_src_nents, dst_nents = 0, mapped_dst_nents = 0;
	struct ablkcipher_edesc *edesc;
	dma_addr_t iv_dma;
	bool in_contig;
	int ivsize = crypto_ablkcipher_ivsize(ablkcipher);
	int dst_sg_idx, qm_sg_ents;
	struct dpaa2_sg_entry *sg_table;
	enum optype op_type = encrypt ? ENCRYPT : DECRYPT;

	src_nents = sg_nents_for_len(req->src, req->nbytes);
	if (unlikely(src_nents < 0)) {
		dev_err(dev, "Insufficient bytes (%d) in src S/G\n",
			req->nbytes);
		return ERR_PTR(src_nents);
	}

	if (unlikely(req->dst != req->src)) {
		dst_nents = sg_nents_for_len(req->dst, req->nbytes);
		if (unlikely(dst_nents < 0)) {
			dev_err(dev, "Insufficient bytes (%d) in dst S/G\n",
				req->nbytes);
			return ERR_PTR(dst_nents);
		}

		mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
					      DMA_TO_DEVICE);
		if (unlikely(!mapped_src_nents)) {
			dev_err(dev, "unable to map source\n");
			return ERR_PTR(-ENOMEM);
		}

		mapped_dst_nents = dma_map_sg(dev, req->dst, dst_nents,
					      DMA_FROM_DEVICE);
		if (unlikely(!mapped_dst_nents)) {
			dev_err(dev, "unable to map destination\n");
			dma_unmap_sg(dev, req->src, src_nents, DMA_TO_DEVICE);
			return ERR_PTR(-ENOMEM);
		}
	} else {
		mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
					      DMA_BIDIRECTIONAL);
		if (unlikely(!mapped_src_nents)) {
			dev_err(dev, "unable to map source\n");
			return ERR_PTR(-ENOMEM);
		}
	}

	iv_dma = dma_map_single(dev, req->info, ivsize, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, iv_dma)) {
		dev_err(dev, "unable to map IV\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents, 0,
			   0, 0, 0, 0);
		return ERR_PTR(-ENOMEM);
	}

	if (mapped_src_nents == 1 &&
	    iv_dma + ivsize == sg_dma_address(req->src)) {
		in_contig = true;
		qm_sg_ents = 0;
	} else {
		in_contig = false;
		qm_sg_ents = 1 + mapped_src_nents;
	}
	dst_sg_idx = qm_sg_ents;

	/* allocate space for base edesc and link tables */
	edesc = qi_cache_alloc(GFP_DMA | flags);
	if (unlikely(!edesc)) {
		dev_err(dev, "could not allocate extended descriptor\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, op_type, 0, 0);
		return ERR_PTR(-ENOMEM);
	}

	edesc->src_nents = src_nents;
	edesc->dst_nents = dst_nents;
	edesc->iv_dma = iv_dma;
	qm_sg_ents += mapped_dst_nents > 1 ? mapped_dst_nents : 0;
	sg_table = &edesc->sgt[0];
	edesc->qm_sg_bytes = qm_sg_ents * sizeof(*sg_table);

	if (!in_contig) {
		dma_to_qm_sg_one(sg_table, iv_dma, ivsize, 0);
		sg_to_qm_sg_last(req->src, mapped_src_nents, sg_table + 1, 0);
	}

	if (mapped_dst_nents > 1)
		sg_to_qm_sg_last(req->dst, mapped_dst_nents, sg_table +
				 dst_sg_idx, 0);

	edesc->qm_sg_dma = dma_map_single(dev, sg_table, edesc->qm_sg_bytes,
					  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, edesc->qm_sg_dma)) {
		dev_err(dev, "unable to map S/G table\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, op_type, 0, 0);
		qi_cache_free(edesc);
		return ERR_PTR(-ENOMEM);
	}

	memset(&req_ctx->fd_flt, 0, sizeof(req_ctx->fd_flt));
	dpaa2_fl_set_final(in_fle, true);
	dpaa2_fl_set_len(in_fle, req->nbytes + ivsize);
	dpaa2_fl_set_len(out_fle, req->nbytes);

	if (!in_contig) {
		dpaa2_fl_set_format(in_fle, dpaa2_fl_sg);
		dpaa2_fl_set_addr(in_fle, edesc->qm_sg_dma);
	} else {
		dpaa2_fl_set_format(in_fle, dpaa2_fl_single);
		dpaa2_fl_set_addr(in_fle, iv_dma);
	}

	if (req->src == req->dst) {
		if (!in_contig) {
			dpaa2_fl_set_format(out_fle, dpaa2_fl_sg);
			dpaa2_fl_set_addr(out_fle, edesc->qm_sg_dma +
					  sizeof(*sg_table));
		} else {
			dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
			dpaa2_fl_set_addr(out_fle, sg_dma_address(req->src));
		}
	} else if (mapped_dst_nents > 1) {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_sg);
		dpaa2_fl_set_addr(out_fle, edesc->qm_sg_dma + dst_sg_idx *
				  sizeof(*sg_table));
	} else {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
		dpaa2_fl_set_addr(out_fle, sg_dma_address(req->dst));
	}

	return edesc;
}

static struct ablkcipher_edesc *ablkcipher_giv_edesc_alloc(
	struct skcipher_givcrypt_request *greq)
{
	struct ablkcipher_request *req = &greq->creq;
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_request *req_ctx = ablkcipher_request_ctx(req);
	struct dpaa2_fl_entry *in_fle = &req_ctx->fd_flt[1];
	struct dpaa2_fl_entry *out_fle = &req_ctx->fd_flt[0];
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct device *dev = ctx->dev;
	gfp_t flags = (req->base.flags & (CRYPTO_TFM_REQ_MAY_BACKLOG |
					  CRYPTO_TFM_REQ_MAY_SLEEP)) ?
		       GFP_KERNEL : GFP_ATOMIC;
	int src_nents, mapped_src_nents, dst_nents, mapped_dst_nents;
	struct ablkcipher_edesc *edesc;
	dma_addr_t iv_dma;
	bool out_contig;
	int ivsize = crypto_ablkcipher_ivsize(ablkcipher);
	struct dpaa2_sg_entry *sg_table;
	int dst_sg_idx, qm_sg_ents;

	src_nents = sg_nents_for_len(req->src, req->nbytes);
	if (unlikely(src_nents < 0)) {
		dev_err(dev, "Insufficient bytes (%d) in src S/G\n",
			req->nbytes);
		return ERR_PTR(src_nents);
	}

	if (unlikely(req->dst != req->src)) {
		dst_nents = sg_nents_for_len(req->dst, req->nbytes);
		if (unlikely(dst_nents < 0)) {
			dev_err(dev, "Insufficient bytes (%d) in dst S/G\n",
				req->nbytes);
			return ERR_PTR(dst_nents);
		}

		mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
					      DMA_TO_DEVICE);
		if (unlikely(!mapped_src_nents)) {
			dev_err(dev, "unable to map source\n");
			return ERR_PTR(-ENOMEM);
		}

		mapped_dst_nents = dma_map_sg(dev, req->dst, dst_nents,
					      DMA_FROM_DEVICE);
		if (unlikely(!mapped_dst_nents)) {
			dev_err(dev, "unable to map destination\n");
			dma_unmap_sg(dev, req->src, src_nents, DMA_TO_DEVICE);
			return ERR_PTR(-ENOMEM);
		}
	} else {
		mapped_src_nents = dma_map_sg(dev, req->src, src_nents,
					      DMA_BIDIRECTIONAL);
		if (unlikely(!mapped_src_nents)) {
			dev_err(dev, "unable to map source\n");
			return ERR_PTR(-ENOMEM);
		}

		dst_nents = src_nents;
		mapped_dst_nents = src_nents;
	}

	iv_dma = dma_map_single(dev, greq->giv, ivsize, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, iv_dma)) {
		dev_err(dev, "unable to map IV\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents, 0,
			   0, 0, 0, 0);
		return ERR_PTR(-ENOMEM);
	}

	qm_sg_ents = mapped_src_nents > 1 ? mapped_src_nents : 0;
	dst_sg_idx = qm_sg_ents;
	if (mapped_dst_nents == 1 &&
	    iv_dma + ivsize == sg_dma_address(req->dst)) {
		out_contig = true;
	} else {
		out_contig = false;
		qm_sg_ents += 1 + mapped_dst_nents;
	}

	/* allocate space for base edesc and link tables */
	edesc = qi_cache_alloc(GFP_DMA | flags);
	if (!edesc) {
		dev_err(dev, "could not allocate extended descriptor\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, GIVENCRYPT, 0, 0);
		return ERR_PTR(-ENOMEM);
	}

	edesc->src_nents = src_nents;
	edesc->dst_nents = dst_nents;
	edesc->iv_dma = iv_dma;
	sg_table = &edesc->sgt[0];
	edesc->qm_sg_bytes = qm_sg_ents * sizeof(*sg_table);

	if (mapped_src_nents > 1)
		sg_to_qm_sg_last(req->src, mapped_src_nents, sg_table, 0);

	if (!out_contig) {
		dma_to_qm_sg_one(sg_table + dst_sg_idx, iv_dma, ivsize, 0);
		sg_to_qm_sg_last(req->dst, mapped_dst_nents, sg_table +
				 dst_sg_idx + 1, 0);
	}

	edesc->qm_sg_dma = dma_map_single(dev, sg_table, edesc->qm_sg_bytes,
					  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, edesc->qm_sg_dma)) {
		dev_err(dev, "unable to map S/G table\n");
		caam_unmap(dev, req->src, req->dst, src_nents, dst_nents,
			   iv_dma, ivsize, GIVENCRYPT, 0, 0);
		qi_cache_free(edesc);
		return ERR_PTR(-ENOMEM);
	}

	memset(&req_ctx->fd_flt, 0, sizeof(req_ctx->fd_flt));
	dpaa2_fl_set_final(in_fle, true);
	dpaa2_fl_set_len(in_fle, req->nbytes);
	dpaa2_fl_set_len(out_fle, ivsize + req->nbytes);

	if (mapped_src_nents > 1) {
		dpaa2_fl_set_format(in_fle, dpaa2_fl_sg);
		dpaa2_fl_set_addr(in_fle, edesc->qm_sg_dma);
	} else {
		dpaa2_fl_set_format(in_fle, dpaa2_fl_single);
		dpaa2_fl_set_addr(in_fle, sg_dma_address(req->src));
	}

	if (!out_contig) {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_sg);
		dpaa2_fl_set_addr(out_fle, edesc->qm_sg_dma + dst_sg_idx *
				  sizeof(*sg_table));
	} else {
		dpaa2_fl_set_format(out_fle, dpaa2_fl_single);
		dpaa2_fl_set_addr(out_fle, sg_dma_address(req->dst));
	}

	return edesc;
}

static void aead_unmap(struct device *dev, struct aead_edesc *edesc,
		       struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	int ivsize = crypto_aead_ivsize(aead);
	struct caam_request *caam_req = aead_request_ctx(req);

	caam_unmap(dev, req->src, req->dst, edesc->src_nents, edesc->dst_nents,
		   edesc->iv_dma, ivsize, caam_req->op_type,
		   edesc->qm_sg_dma, edesc->qm_sg_bytes);
	dma_unmap_single(dev, edesc->assoclen_dma, 4, DMA_TO_DEVICE);
}

static void ablkcipher_unmap(struct device *dev,
			     struct ablkcipher_edesc *edesc,
			     struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	int ivsize = crypto_ablkcipher_ivsize(ablkcipher);
	struct caam_request *caam_req = ablkcipher_request_ctx(req);

	caam_unmap(dev, req->src, req->dst, edesc->src_nents, edesc->dst_nents,
		   edesc->iv_dma, ivsize, caam_req->op_type,
		   edesc->qm_sg_dma, edesc->qm_sg_bytes);
}

static void aead_encrypt_done(void *cbk_ctx, u32 status)
{
	struct crypto_async_request *areq = cbk_ctx;
	struct aead_request *req = container_of(areq, struct aead_request,
						base);
	struct caam_request *req_ctx = to_caam_req(areq);
	struct aead_edesc *edesc = req_ctx->edesc;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	int ecode = 0;

#ifdef DEBUG
	dev_err(ctx->dev, "%s %d: err 0x%x\n", __func__, __LINE__, status);
#endif

	if (unlikely(status)) {
		caam_qi2_strstatus(ctx->dev, status);
		ecode = -EIO;
	}

	aead_unmap(ctx->dev, edesc, req);
	qi_cache_free(edesc);
	aead_request_complete(req, ecode);
}

static void aead_decrypt_done(void *cbk_ctx, u32 status)
{
	struct crypto_async_request *areq = cbk_ctx;
	struct aead_request *req = container_of(areq, struct aead_request,
						base);
	struct caam_request *req_ctx = to_caam_req(areq);
	struct aead_edesc *edesc = req_ctx->edesc;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	int ecode = 0;

#ifdef DEBUG
	dev_err(ctx->dev, "%s %d: err 0x%x\n", __func__, __LINE__, status);
#endif

	if (unlikely(status)) {
		caam_qi2_strstatus(ctx->dev, status);
		/*
		 * verify hw auth check passed else return -EBADMSG
		 */
		if ((status & JRSTA_CCBERR_ERRID_MASK) ==
		     JRSTA_CCBERR_ERRID_ICVCHK)
			ecode = -EBADMSG;
		else
			ecode = -EIO;
	}

	aead_unmap(ctx->dev, edesc, req);
	qi_cache_free(edesc);
	aead_request_complete(req, ecode);
}

static int aead_encrypt(struct aead_request *req)
{
	struct aead_edesc *edesc;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct caam_request *caam_req = aead_request_ctx(req);
	int ret;

	/* allocate extended descriptor */
	edesc = aead_edesc_alloc(req, true);
	if (IS_ERR(edesc))
		return PTR_ERR(edesc);

	caam_req->flc = &ctx->flc[ENCRYPT];
	caam_req->flc_dma = ctx->flc_dma[ENCRYPT];
	caam_req->op_type = ENCRYPT;
	caam_req->cbk = aead_encrypt_done;
	caam_req->ctx = &req->base;
	caam_req->edesc = edesc;
	ret = dpaa2_caam_enqueue(ctx->dev, caam_req);
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
		aead_unmap(ctx->dev, edesc, req);
		qi_cache_free(edesc);
	}

	return ret;
}

static int aead_decrypt(struct aead_request *req)
{
	struct aead_edesc *edesc;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct caam_ctx *ctx = crypto_aead_ctx(aead);
	struct caam_request *caam_req = aead_request_ctx(req);
	int ret;

	/* allocate extended descriptor */
	edesc = aead_edesc_alloc(req, false);
	if (IS_ERR(edesc))
		return PTR_ERR(edesc);

	caam_req->flc = &ctx->flc[DECRYPT];
	caam_req->flc_dma = ctx->flc_dma[DECRYPT];
	caam_req->op_type = DECRYPT;
	caam_req->cbk = aead_decrypt_done;
	caam_req->ctx = &req->base;
	caam_req->edesc = edesc;
	ret = dpaa2_caam_enqueue(ctx->dev, caam_req);
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
		aead_unmap(ctx->dev, edesc, req);
		qi_cache_free(edesc);
	}

	return ret;
}

static int ipsec_gcm_encrypt(struct aead_request *req)
{
	if (req->assoclen < 8)
		return -EINVAL;

	return aead_encrypt(req);
}

static int ipsec_gcm_decrypt(struct aead_request *req)
{
	if (req->assoclen < 8)
		return -EINVAL;

	return aead_decrypt(req);
}

static void ablkcipher_done(void *cbk_ctx, u32 status)
{
	struct crypto_async_request *areq = cbk_ctx;
	struct ablkcipher_request *req = ablkcipher_request_cast(areq);
	struct caam_request *req_ctx = to_caam_req(areq);
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct ablkcipher_edesc *edesc = req_ctx->edesc;
	int ecode = 0;
#ifdef DEBUG
	int ivsize = crypto_ablkcipher_ivsize(ablkcipher);

	dev_err(ctx->dev, "%s %d: err 0x%x\n", __func__, __LINE__, status);
#endif

	if (unlikely(status)) {
		caam_qi2_strstatus(ctx->dev, status);
		ecode = -EIO;
	}

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "dstiv  @" __stringify(__LINE__)": ",
		       DUMP_PREFIX_ADDRESS, 16, 4, req->info,
		       edesc->src_nents > 1 ? 100 : ivsize, 1);
	caam_dump_sg(KERN_ERR, "dst    @" __stringify(__LINE__)": ",
		     DUMP_PREFIX_ADDRESS, 16, 4, req->dst,
		     edesc->dst_nents > 1 ? 100 : req->nbytes, 1);
#endif

	ablkcipher_unmap(ctx->dev, edesc, req);
	qi_cache_free(edesc);
	ablkcipher_request_complete(req, ecode);
}

static int ablkcipher_encrypt(struct ablkcipher_request *req)
{
	struct ablkcipher_edesc *edesc;
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct caam_request *caam_req = ablkcipher_request_ctx(req);
	int ret;

	/* allocate extended descriptor */
	edesc = ablkcipher_edesc_alloc(req, true);
	if (IS_ERR(edesc))
		return PTR_ERR(edesc);

	caam_req->flc = &ctx->flc[ENCRYPT];
	caam_req->flc_dma = ctx->flc_dma[ENCRYPT];
	caam_req->op_type = ENCRYPT;
	caam_req->cbk = ablkcipher_done;
	caam_req->ctx = &req->base;
	caam_req->edesc = edesc;
	ret = dpaa2_caam_enqueue(ctx->dev, caam_req);
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
		ablkcipher_unmap(ctx->dev, edesc, req);
		qi_cache_free(edesc);
	}

	return ret;
}

static int ablkcipher_givencrypt(struct skcipher_givcrypt_request *greq)
{
	struct ablkcipher_request *req = &greq->creq;
	struct ablkcipher_edesc *edesc;
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct caam_request *caam_req = ablkcipher_request_ctx(req);
	int ret;

	/* allocate extended descriptor */
	edesc = ablkcipher_giv_edesc_alloc(greq);
	if (IS_ERR(edesc))
		return PTR_ERR(edesc);

	caam_req->flc = &ctx->flc[GIVENCRYPT];
	caam_req->flc_dma = ctx->flc_dma[GIVENCRYPT];
	caam_req->op_type = GIVENCRYPT;
	caam_req->cbk = ablkcipher_done;
	caam_req->ctx = &req->base;
	caam_req->edesc = edesc;
	ret = dpaa2_caam_enqueue(ctx->dev, caam_req);
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
		ablkcipher_unmap(ctx->dev, edesc, req);
		qi_cache_free(edesc);
	}

	return ret;
}

static int ablkcipher_decrypt(struct ablkcipher_request *req)
{
	struct ablkcipher_edesc *edesc;
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct caam_ctx *ctx = crypto_ablkcipher_ctx(ablkcipher);
	struct caam_request *caam_req = ablkcipher_request_ctx(req);
	int ret;

	/* allocate extended descriptor */
	edesc = ablkcipher_edesc_alloc(req, false);
	if (IS_ERR(edesc))
		return PTR_ERR(edesc);

	caam_req->flc = &ctx->flc[DECRYPT];
	caam_req->flc_dma = ctx->flc_dma[DECRYPT];
	caam_req->op_type = DECRYPT;
	caam_req->cbk = ablkcipher_done;
	caam_req->ctx = &req->base;
	caam_req->edesc = edesc;
	ret = dpaa2_caam_enqueue(ctx->dev, caam_req);
	if (ret != -EINPROGRESS &&
	    !(ret == -EBUSY && req->base.flags & CRYPTO_TFM_REQ_MAY_BACKLOG)) {
		ablkcipher_unmap(ctx->dev, edesc, req);
		qi_cache_free(edesc);
	}

	return ret;
}

struct caam_crypto_alg {
	struct list_head entry;
	struct crypto_alg crypto_alg;
	struct caam_alg_entry caam;
};

static int caam_cra_init(struct crypto_tfm *tfm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct caam_crypto_alg *caam_alg = container_of(alg, typeof(*caam_alg),
							crypto_alg);
	struct caam_ctx *ctx = crypto_tfm_ctx(tfm);

	/* copy descriptor header template value */
	ctx->cdata.algtype = OP_TYPE_CLASS1_ALG |
			     caam_alg->caam.class1_alg_type;
	ctx->adata.algtype = OP_TYPE_CLASS2_ALG |
			     caam_alg->caam.class2_alg_type;

	ctx->dev = caam_alg->caam.dev;

	return 0;
}

static int caam_cra_init_ablkcipher(struct crypto_tfm *tfm)
{
	struct ablkcipher_tfm *ablkcipher_tfm =
		crypto_ablkcipher_crt(__crypto_ablkcipher_cast(tfm));

	ablkcipher_tfm->reqsize = sizeof(struct caam_request);
	return caam_cra_init(tfm);
}

static int caam_cra_init_aead(struct crypto_aead *tfm)
{
	crypto_aead_set_reqsize(tfm, sizeof(struct caam_request));
	return caam_cra_init(crypto_aead_tfm(tfm));
}

static void caam_exit_common(struct crypto_tfm *tfm)
{
	struct caam_ctx *ctx = crypto_tfm_ctx(tfm);
	int i;

	for (i = 0; i < NUM_OP; i++) {
		if (!ctx->flc_dma[i])
			continue;
		dma_unmap_single(ctx->dev, ctx->flc_dma[i],
				 sizeof(ctx->flc[i].flc) +
					desc_bytes(ctx->flc[i].sh_desc),
				 DMA_TO_DEVICE);
	}

	if (ctx->key_dma)
		dma_unmap_single(ctx->dev, ctx->key_dma,
				 ctx->cdata.keylen + ctx->adata.keylen_pad,
				 DMA_TO_DEVICE);
}

static void caam_cra_exit(struct crypto_tfm *tfm)
{
	caam_exit_common(crypto_tfm_ctx(tfm));
}

static void caam_cra_exit_aead(struct crypto_aead *tfm)
{
	caam_exit_common(crypto_aead_ctx(tfm));
}

#define template_ablkcipher	template_u.ablkcipher
struct caam_alg_template {
	char name[CRYPTO_MAX_ALG_NAME];
	char driver_name[CRYPTO_MAX_ALG_NAME];
	unsigned int blocksize;
	u32 type;
	union {
		struct ablkcipher_alg ablkcipher;
	} template_u;
	u32 class1_alg_type;
	u32 class2_alg_type;
};

static struct caam_alg_template driver_algs[] = {
	/* ablkcipher descriptor */
	{
		.name = "cbc(aes)",
		.driver_name = "cbc-aes-caam-qi2",
		.blocksize = AES_BLOCK_SIZE,
		.type = CRYPTO_ALG_TYPE_GIVCIPHER,
		.template_ablkcipher = {
			.setkey = ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.givencrypt = ablkcipher_givencrypt,
			.geniv = "<built-in>",
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
	},
	{
		.name = "cbc(des3_ede)",
		.driver_name = "cbc-3des-caam-qi2",
		.blocksize = DES3_EDE_BLOCK_SIZE,
		.type = CRYPTO_ALG_TYPE_GIVCIPHER,
		.template_ablkcipher = {
			.setkey = ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.givencrypt = ablkcipher_givencrypt,
			.geniv = "<built-in>",
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.ivsize = DES3_EDE_BLOCK_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
	},
	{
		.name = "cbc(des)",
		.driver_name = "cbc-des-caam-qi2",
		.blocksize = DES_BLOCK_SIZE,
		.type = CRYPTO_ALG_TYPE_GIVCIPHER,
		.template_ablkcipher = {
			.setkey = ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.givencrypt = ablkcipher_givencrypt,
			.geniv = "<built-in>",
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.ivsize = DES_BLOCK_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
	},
	{
		.name = "ctr(aes)",
		.driver_name = "ctr-aes-caam-qi2",
		.blocksize = 1,
		.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
		.template_ablkcipher = {
			.setkey = ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.geniv = "chainiv",
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CTR_MOD128,
	},
	{
		.name = "rfc3686(ctr(aes))",
		.driver_name = "rfc3686-ctr-aes-caam-qi2",
		.blocksize = 1,
		.type = CRYPTO_ALG_TYPE_GIVCIPHER,
		.template_ablkcipher = {
			.setkey = ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.givencrypt = ablkcipher_givencrypt,
			.geniv = "<built-in>",
			.min_keysize = AES_MIN_KEY_SIZE +
				       CTR_RFC3686_NONCE_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE +
				       CTR_RFC3686_NONCE_SIZE,
			.ivsize = CTR_RFC3686_IV_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CTR_MOD128,
	},
	{
		.name = "xts(aes)",
		.driver_name = "xts-aes-caam-qi2",
		.blocksize = AES_BLOCK_SIZE,
		.type = CRYPTO_ALG_TYPE_ABLKCIPHER,
		.template_ablkcipher = {
			.setkey = xts_ablkcipher_setkey,
			.encrypt = ablkcipher_encrypt,
			.decrypt = ablkcipher_decrypt,
			.geniv = "eseqiv",
			.min_keysize = 2 * AES_MIN_KEY_SIZE,
			.max_keysize = 2 * AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
		},
		.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_XTS,
	}
};

static struct caam_aead_alg driver_aeads[] = {
	{
		.aead = {
			.base = {
				.cra_name = "rfc4106(gcm(aes))",
				.cra_driver_name = "rfc4106-gcm-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = rfc4106_setkey,
			.setauthsize = rfc4106_setauthsize,
			.encrypt = ipsec_gcm_encrypt,
			.decrypt = ipsec_gcm_decrypt,
			.ivsize = 8,
			.maxauthsize = AES_BLOCK_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_GCM,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "rfc4543(gcm(aes))",
				.cra_driver_name = "rfc4543-gcm-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = rfc4543_setkey,
			.setauthsize = rfc4543_setauthsize,
			.encrypt = ipsec_gcm_encrypt,
			.decrypt = ipsec_gcm_decrypt,
			.ivsize = 8,
			.maxauthsize = AES_BLOCK_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_GCM,
		},
	},
	/* Galois Counter Mode */
	{
		.aead = {
			.base = {
				.cra_name = "gcm(aes)",
				.cra_driver_name = "gcm-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = gcm_setkey,
			.setauthsize = gcm_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = 12,
			.maxauthsize = AES_BLOCK_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_GCM,
		}
	},
	/* single-pass ipsec_esp descriptor */
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(md5),cbc(aes))",
				.cra_driver_name = "authenc-hmac-md5-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(md5),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-hmac-md5-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha1),cbc(aes))",
				.cra_driver_name = "authenc-hmac-sha1-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha1),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha1-cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha224),cbc(aes))",
				.cra_driver_name = "authenc-hmac-sha224-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha224),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha224-cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha256),cbc(aes))",
				.cra_driver_name = "authenc-hmac-sha256-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha256),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha256-cbc-aes-"
						   "caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha384),cbc(aes))",
				.cra_driver_name = "authenc-hmac-sha384-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha384),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha384-cbc-aes-"
						   "caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha512),cbc(aes))",
				.cra_driver_name = "authenc-hmac-sha512-"
						   "cbc-aes-caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha512),"
					    "cbc(aes)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha512-cbc-aes-"
						   "caam-qi2",
				.cra_blocksize = AES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = AES_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(md5),cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-md5-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(md5),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-hmac-md5-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha1),"
					    "cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-sha1-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha1),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha1-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha224),"
					    "cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-sha224-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha224),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha224-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha256),"
					    "cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-sha256-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha256),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha256-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha384),"
					    "cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-sha384-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha384),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha384-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha512),"
					    "cbc(des3_ede))",
				.cra_driver_name = "authenc-hmac-sha512-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha512),"
					    "cbc(des3_ede)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha512-"
						   "cbc-des3_ede-caam-qi2",
				.cra_blocksize = DES3_EDE_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES3_EDE_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_3DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(md5),cbc(des))",
				.cra_driver_name = "authenc-hmac-md5-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(md5),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-hmac-md5-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha1),cbc(des))",
				.cra_driver_name = "authenc-hmac-sha1-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha1),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha1-cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha224),cbc(des))",
				.cra_driver_name = "authenc-hmac-sha224-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha224),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha224-cbc-des-"
						   "caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha256),cbc(des))",
				.cra_driver_name = "authenc-hmac-sha256-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha256),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha256-cbc-desi-"
						   "caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha384),cbc(des))",
				.cra_driver_name = "authenc-hmac-sha384-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha384),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha384-cbc-des-"
						   "caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha512),cbc(des))",
				.cra_driver_name = "authenc-hmac-sha512-"
						   "cbc-des-caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "echainiv(authenc(hmac(sha512),"
					    "cbc(des)))",
				.cra_driver_name = "echainiv-authenc-"
						   "hmac-sha512-cbc-des-"
						   "caam-qi2",
				.cra_blocksize = DES_BLOCK_SIZE,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = DES_BLOCK_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_DES | OP_ALG_AAI_CBC,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.geniv = true,
		}
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(md5),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-md5-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc("
					    "hmac(md5),rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-md5-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = MD5_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_MD5 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha1),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-sha1-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc("
					    "hmac(sha1),rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-sha1-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA1_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA1 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha224),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-sha224-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc("
					    "hmac(sha224),rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-sha224-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA224_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA224 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha256),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-sha256-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc(hmac(sha256),"
					    "rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-sha256-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA256_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA256 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha384),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-sha384-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc(hmac(sha384),"
					    "rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-sha384-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA384_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA384 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "authenc(hmac(sha512),"
					    "rfc3686(ctr(aes)))",
				.cra_driver_name = "authenc-hmac-sha512-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
		},
	},
	{
		.aead = {
			.base = {
				.cra_name = "seqiv(authenc(hmac(sha512),"
					    "rfc3686(ctr(aes))))",
				.cra_driver_name = "seqiv-authenc-hmac-sha512-"
						   "rfc3686-ctr-aes-caam-qi2",
				.cra_blocksize = 1,
			},
			.setkey = aead_setkey,
			.setauthsize = aead_setauthsize,
			.encrypt = aead_encrypt,
			.decrypt = aead_decrypt,
			.ivsize = CTR_RFC3686_IV_SIZE,
			.maxauthsize = SHA512_DIGEST_SIZE,
		},
		.caam = {
			.class1_alg_type = OP_ALG_ALGSEL_AES |
					   OP_ALG_AAI_CTR_MOD128,
			.class2_alg_type = OP_ALG_ALGSEL_SHA512 |
					   OP_ALG_AAI_HMAC_PRECOMP,
			.rfc3686 = true,
			.geniv = true,
		},
	},
};

static struct caam_crypto_alg *caam_alg_alloc(struct caam_alg_template
					      *template)
{
	struct caam_crypto_alg *t_alg;
	struct crypto_alg *alg;

	t_alg = kzalloc(sizeof(*t_alg), GFP_KERNEL);
	if (!t_alg)
		return ERR_PTR(-ENOMEM);

	alg = &t_alg->crypto_alg;

	snprintf(alg->cra_name, CRYPTO_MAX_ALG_NAME, "%s", template->name);
	snprintf(alg->cra_driver_name, CRYPTO_MAX_ALG_NAME, "%s",
		 template->driver_name);
	alg->cra_module = THIS_MODULE;
	alg->cra_exit = caam_cra_exit;
	alg->cra_priority = CAAM_CRA_PRIORITY;
	alg->cra_blocksize = template->blocksize;
	alg->cra_alignmask = 0;
	alg->cra_ctxsize = sizeof(struct caam_ctx);
	alg->cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_KERN_DRIVER_ONLY |
			 template->type;
	switch (template->type) {
	case CRYPTO_ALG_TYPE_GIVCIPHER:
		alg->cra_init = caam_cra_init_ablkcipher;
		alg->cra_type = &crypto_givcipher_type;
		alg->cra_ablkcipher = template->template_ablkcipher;
		break;
	case CRYPTO_ALG_TYPE_ABLKCIPHER:
		alg->cra_init = caam_cra_init_ablkcipher;
		alg->cra_type = &crypto_ablkcipher_type;
		alg->cra_ablkcipher = template->template_ablkcipher;
		break;
	}

	t_alg->caam.class1_alg_type = template->class1_alg_type;
	t_alg->caam.class2_alg_type = template->class2_alg_type;

	return t_alg;
}

static void caam_aead_alg_init(struct caam_aead_alg *t_alg)
{
	struct aead_alg *alg = &t_alg->aead;

	alg->base.cra_module = THIS_MODULE;
	alg->base.cra_priority = CAAM_CRA_PRIORITY;
	alg->base.cra_ctxsize = sizeof(struct caam_ctx);
	alg->base.cra_flags = CRYPTO_ALG_ASYNC | CRYPTO_ALG_KERN_DRIVER_ONLY;

	alg->init = caam_cra_init_aead;
	alg->exit = caam_cra_exit_aead;
}

static void dpaa2_caam_fqdan_cb(struct dpaa2_io_notification_ctx *nctx)
{
	struct dpaa2_caam_priv_per_cpu *ppriv;

	ppriv = container_of(nctx, struct dpaa2_caam_priv_per_cpu, nctx);
	napi_schedule_irqoff(&ppriv->napi);
}

static int __cold dpaa2_dpseci_dpio_setup(struct dpaa2_caam_priv *priv)
{
	struct device *dev = priv->dev;
	struct dpaa2_io_notification_ctx *nctx;
	struct dpaa2_caam_priv_per_cpu *ppriv;
	int err, i, j = 0;

	for_each_online_cpu(i) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		ppriv->priv = priv;
		nctx = &ppriv->nctx;
		nctx->is_cdan = 0;
		nctx->id = ppriv->rsp_fqid;
		nctx->desired_cpu = i;
		nctx->cb = dpaa2_caam_fqdan_cb;

		/* Register notification callbacks */
		err = dpaa2_io_service_register(NULL, nctx);
		if (unlikely(err)) {
			dev_err(dev, "notification register failed\n");
			nctx->cb = NULL;
			goto err;
		}

		ppriv->store = dpaa2_io_store_create(DPAA2_CAAM_STORE_SIZE,
						     dev);
		if (unlikely(!ppriv->store)) {
			dev_err(dev, "dpaa2_io_store_create() failed\n");
			goto err;
		}

		j++;
		if (j == priv->dpseci_attr.num_rx_queues)
			break;
	}

	return 0;

err:
	for_each_online_cpu(i) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		if (!ppriv->nctx.cb)
			break;
		dpaa2_io_service_deregister(NULL, &ppriv->nctx);
	}

	for_each_online_cpu(i) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		if (!ppriv->store)
			break;
		dpaa2_io_store_destroy(ppriv->store);
	}

	return err;
}

static void __cold dpaa2_dpseci_dpio_free(struct dpaa2_caam_priv *priv)
{
	struct dpaa2_caam_priv_per_cpu *ppriv;
	int i, j = 0;

	for_each_online_cpu(i) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		dpaa2_io_service_deregister(NULL, &ppriv->nctx);
		dpaa2_io_store_destroy(ppriv->store);

		j++;
		if (j == priv->dpseci_attr.num_rx_queues)
			return;
	}
}

static int dpaa2_dpseci_bind(struct dpaa2_caam_priv *priv)
{
	struct dpseci_rx_queue_cfg rx_queue_cfg;
	struct device *dev = priv->dev;
	struct fsl_mc_device *ls_dev = to_fsl_mc_device(dev);
	struct dpaa2_caam_priv_per_cpu *ppriv;
	int err = 0, i;
	u8 j = 0;

	/*
	 * Configure Rx queues
	 * TODO: if number of Rx queues > number of (online) cores,
	 * not all Rx queues will be configured/used
	 */
	for_each_online_cpu(i) {
		ppriv = per_cpu_ptr(priv->ppriv, i);

		rx_queue_cfg.options = DPSECI_QUEUE_OPT_DEST |
				       DPSECI_QUEUE_OPT_USER_CTX;
		rx_queue_cfg.order_preservation_en = 0;
		rx_queue_cfg.dest_cfg.dest_type = DPSECI_DEST_DPIO;
		rx_queue_cfg.dest_cfg.dest_id = ppriv->nctx.dpio_id;
		/* TODO: hard-coded Rx priority (WQ) */
		rx_queue_cfg.dest_cfg.priority = 4;
		rx_queue_cfg.user_ctx = ppriv->nctx.qman64;

		err = dpseci_set_rx_queue(priv->mc_io, 0, ls_dev->mc_handle, j,
					  &rx_queue_cfg);
		if (err) {
			dev_err(dev, "dpseci_set_rx_queue() failed with err %d\n",
				err);
			return err;
		}

		j++;
		if (j == priv->dpseci_attr.num_rx_queues)
			break;
	}

	return err;
}

static void dpaa2_dpseci_congestion_free(struct dpaa2_caam_priv *priv)
{
	struct device *dev = priv->dev;

	if (!priv->cscn_mem)
		return;

	dma_unmap_single(dev, priv->cscn_dma, DPAA2_CSCN_SIZE, DMA_FROM_DEVICE);
	kfree(priv->cscn_mem);
}

static void dpaa2_dpseci_free(struct dpaa2_caam_priv *priv)
{
	struct device *dev = priv->dev;
	struct fsl_mc_device *ls_dev = to_fsl_mc_device(dev);

	dpaa2_dpseci_congestion_free(priv);
	dpseci_close(priv->mc_io, 0, ls_dev->mc_handle);
}

static void dpaa2_caam_process_fd(struct dpaa2_caam_priv *priv,
				  const struct dpaa2_fd *fd)
{
	struct caam_request *req;

	if (dpaa2_fd_get_format(fd) != dpaa2_fd_list) {
		dev_err(priv->dev, "Only Frame List FD format is supported!\n");
		return;
	}

	/*
	 * TODO: First check FD[ERR]
	 * see DPAA2RM section "3.4.5 Error handling" for error codes
	 */

	req = phys_to_virt(dma_to_phys(priv->dev, dpaa2_fd_get_addr(fd)));
	dma_unmap_single(priv->dev, req->fd_flt_dma, sizeof(req->fd_flt),
			 DMA_TO_DEVICE);
	req->cbk(req->ctx, dpaa2_fd_get_frc(fd));
}

static int dpaa2_caam_pull_fq(struct dpaa2_caam_priv_per_cpu *ppriv)
{
	int err;

	/* Retry while portal is busy */
	do {
		err = dpaa2_io_service_pull_fq(NULL, ppriv->rsp_fqid,
					       ppriv->store);
	} while (err == -EBUSY);

	if (unlikely(err))
		dev_err(ppriv->priv->dev, "dpaa2_io_service_pull err %d", err);

	return err;
}

static int dpaa2_caam_store_consume(struct dpaa2_caam_priv_per_cpu *ppriv)
{
	struct dpaa2_dq *dq;
	int cleaned = 0, is_last;

	do {
		dq = dpaa2_io_store_next(ppriv->store, &is_last);
		if (unlikely(!dq)) {
			if (unlikely(!is_last)) {
				dev_dbg(ppriv->priv->dev,
					"FQ %d returned no valid frames\n",
					ppriv->rsp_fqid);
				/*
				 * MUST retry until we get some sort of
				 * valid response token (be it "empty dequeue"
				 * or a valid frame).
				 */
				continue;
			}
			break;
		}

		/* Process FD */
		dpaa2_caam_process_fd(ppriv->priv, dpaa2_dq_fd(dq));
		cleaned++;
	} while (!is_last);

	return cleaned;
}

static int dpaa2_dpseci_poll(struct napi_struct *napi, int budget)
{
	struct dpaa2_caam_priv_per_cpu *ppriv;
	struct dpaa2_caam_priv *priv;
	int err, cleaned = 0, store_cleaned;

	ppriv = container_of(napi, struct dpaa2_caam_priv_per_cpu, napi);
	priv = ppriv->priv;

	if (unlikely(dpaa2_caam_pull_fq(ppriv)))
		return 0;

	do {
		store_cleaned = dpaa2_caam_store_consume(ppriv);
		cleaned += store_cleaned;

		if (store_cleaned == 0 ||
		    cleaned > budget - DPAA2_CAAM_STORE_SIZE)
			break;

		/* Try to dequeue some more */
		err = dpaa2_caam_pull_fq(ppriv);
		if (unlikely(err))
			break;
	} while (1);

	if (cleaned < budget) {
		napi_complete_done(napi, cleaned);
		err = dpaa2_io_service_rearm(NULL, &ppriv->nctx);
		if (unlikely(err))
			dev_err(priv->dev, "Notification rearm failed: %d\n",
				err);
	}

	return cleaned;
}

static int dpaa2_dpseci_congestion_setup(struct dpaa2_caam_priv *priv,
					 u16 token)
{
	struct dpseci_congestion_notification_cfg cong_notif_cfg = { 0 };
	struct device *dev = priv->dev;
	int err;

	if (DPSECI_VER(priv->major_ver, priv->minor_ver) < DPSECI_VERSION)
		return 0;

	priv->cscn_mem = kzalloc(DPAA2_CSCN_SIZE + DPAA2_CSCN_ALIGN,
					GFP_KERNEL | GFP_DMA);
	if (!priv->cscn_mem)
		return -ENOMEM;

	priv->cscn_mem_aligned = PTR_ALIGN(priv->cscn_mem, DPAA2_CSCN_ALIGN);
	priv->cscn_dma = dma_map_single(dev, priv->cscn_mem_aligned,
					DPAA2_CSCN_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, priv->cscn_dma)) {
		dev_err(dev, "Error mapping CSCN memory area\n");
		err = -ENOMEM;
		goto err_dma_map;
	}

	cong_notif_cfg.units = DPSECI_CONGESTION_UNIT_BYTES;
	cong_notif_cfg.threshold_entry = DPAA2_SEC_CONG_ENTRY_THRESH;
	cong_notif_cfg.threshold_exit = DPAA2_SEC_CONG_EXIT_THRESH;
	cong_notif_cfg.message_ctx = (u64)priv;
	cong_notif_cfg.message_iova = priv->cscn_dma;
	cong_notif_cfg.notification_mode = DPSECI_CGN_MODE_WRITE_MEM_ON_ENTER |
					DPSECI_CGN_MODE_WRITE_MEM_ON_EXIT |
					DPSECI_CGN_MODE_COHERENT_WRITE;

	err = dpseci_set_congestion_notification(priv->mc_io, 0, token,
						 &cong_notif_cfg);
	if (err) {
		dev_err(dev, "dpseci_set_congestion_notification failed\n");
		goto err_set_cong;
	}

	return 0;

err_set_cong:
	dma_unmap_single(dev, priv->cscn_dma, DPAA2_CSCN_SIZE, DMA_FROM_DEVICE);
err_dma_map:
	kfree(priv->cscn_mem);

	return err;
}

static int __cold dpaa2_dpseci_setup(struct fsl_mc_device *ls_dev)
{
	struct device *dev = &ls_dev->dev;
	struct dpaa2_caam_priv *priv;
	struct dpaa2_caam_priv_per_cpu *ppriv;
	int err;
	u8 i;

	priv = dev_get_drvdata(dev);

	priv->dev = dev;
	priv->dpsec_id = ls_dev->obj_desc.id;

	/* Get a handle for the DPSECI this interface is associate with */
	err = dpseci_open(priv->mc_io, 0, priv->dpsec_id, &ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpsec_open() failed: %d\n", err);
		goto err_open;
	}

	dev_info(dev, "Opened dpseci object successfully\n");

	err = dpseci_get_api_version(priv->mc_io, 0, &priv->major_ver,
				     &priv->minor_ver);
	if (err) {
		dev_err(dev, "dpseci_get_api_version() failed\n");
		goto err_get_vers;
	}

	err = dpseci_get_attributes(priv->mc_io, 0, ls_dev->mc_handle,
				    &priv->dpseci_attr);
	if (err) {
		dev_err(dev, "dpseci_get_attributes() failed\n");
		goto err_get_vers;
	}

	err = dpaa2_dpseci_congestion_setup(priv, ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "setup_congestion() failed\n");
		goto err_get_vers;
	}

	priv->num_pairs = min(priv->dpseci_attr.num_rx_queues,
			      priv->dpseci_attr.num_tx_queues);

	for (i = 0; i < priv->dpseci_attr.num_rx_queues; i++) {
		err = dpseci_get_rx_queue(priv->mc_io, 0, ls_dev->mc_handle, i,
					  &priv->rx_queue_attr[i]);
		if (err) {
			dev_err(dev, "dpseci_get_rx_queue() failed\n");
			goto err_get_rx_queue;
		}
	}

	for (i = 0; i < priv->dpseci_attr.num_tx_queues; i++) {
		err = dpseci_get_tx_queue(priv->mc_io, 0, ls_dev->mc_handle, i,
					  &priv->tx_queue_attr[i]);
		if (err) {
			dev_err(dev, "dpseci_get_tx_queue() failed\n");
			goto err_get_rx_queue;
		}
	}

	for (i = 0; i < priv->num_pairs; i++) {
		dev_info(dev, "prio %d: rx queue %d, tx queue %d\n", i,
			 priv->rx_queue_attr[i].fqid,
			 priv->tx_queue_attr[i].fqid);

		/* TODO: Assumption - number of queues <= number of cores */
		ppriv = per_cpu_ptr(priv->ppriv, i);
		ppriv->req_fqid = priv->tx_queue_attr[i].fqid;
		ppriv->rsp_fqid = priv->rx_queue_attr[i].fqid;
		ppriv->prio = i;

		ppriv->net_dev.dev = *dev;
		INIT_LIST_HEAD(&ppriv->net_dev.napi_list);
		netif_napi_add(&ppriv->net_dev, &ppriv->napi, dpaa2_dpseci_poll,
			       DPAA2_CAAM_NAPI_WEIGHT);
	}

	return 0;

err_get_rx_queue:
	dpaa2_dpseci_congestion_free(priv);
err_get_vers:
	dpseci_close(priv->mc_io, 0, ls_dev->mc_handle);
err_open:
	return err;
}

static int dpaa2_dpseci_enable(struct dpaa2_caam_priv *priv)
{
	struct device *dev = priv->dev;
	struct fsl_mc_device *ls_dev = to_fsl_mc_device(dev);
	struct dpaa2_caam_priv_per_cpu *ppriv;
	int err, i;

	for (i = 0; i < priv->num_pairs; i++) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		napi_enable(&ppriv->napi);
	}

	err = dpseci_enable(priv->mc_io, 0, ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpseci_enable() failed\n");
		return err;
	}

	dev_info(dev, "DPSECI version %d.%d\n",
		 priv->major_ver,
		 priv->minor_ver);

	return 0;
}

static int __cold dpaa2_dpseci_disable(struct dpaa2_caam_priv *priv)
{
	struct device *dev = priv->dev;
	struct dpaa2_caam_priv_per_cpu *ppriv;
	struct fsl_mc_device *ls_dev = to_fsl_mc_device(dev);
	int i, err = 0, enabled;

	err = dpseci_disable(priv->mc_io, 0, ls_dev->mc_handle);
	if (err) {
		dev_err(dev, "dpseci_disable() failed\n");
		return err;
	}

	err = dpseci_is_enabled(priv->mc_io, 0, ls_dev->mc_handle, &enabled);
	if (err) {
		dev_err(dev, "dpseci_is_enabled() failed\n");
		return err;
	}

	dev_dbg(dev, "disable: %s\n", enabled ? "false" : "true");

	for (i = 0; i < priv->num_pairs; i++) {
		ppriv = per_cpu_ptr(priv->ppriv, i);
		napi_disable(&ppriv->napi);
		netif_napi_del(&ppriv->napi);
	}

	return 0;
}

static struct list_head alg_list;

static int dpaa2_caam_probe(struct fsl_mc_device *dpseci_dev)
{
	struct device *dev;
	struct dpaa2_caam_priv *priv;
	int i, err = 0;
	bool registered = false;

	/*
	 * TODO: temporary workaround for MC f/w not providing
	 * CAAM endianness info.
	 */
	caam_little_end = true;

	caam_imx = false;

	dev = &dpseci_dev->dev;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	qi_cache = kmem_cache_create("dpaa2_caamqicache", CAAM_QI_MEMCACHE_SIZE,
				     0, SLAB_CACHE_DMA, NULL);
	if (!qi_cache) {
		dev_err(dev, "Can't allocate SEC cache\n");
		err = -ENOMEM;
		goto err_qicache;
	}

	/* DMA settings, will need later on for zero copy Rx/Tx */
	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(49));
	if (err) {
		dev_err(dev, "dma_set_mask_and_coherent() failed\n");
		goto err_dma_mask;
	}

	/* Obtain a MC portal */
	err = fsl_mc_portal_allocate(dpseci_dev, 0, &priv->mc_io);
	if (err) {
		dev_err(dev, "MC portal allocation failed\n");
		goto err_dma_mask;
	}

	priv->ppriv = alloc_percpu(*priv->ppriv);
	if (!priv->ppriv) {
		dev_err(dev, "alloc_percpu() failed\n");
		goto err_alloc_ppriv;
	}

	/* DPSECI initialization */
	err = dpaa2_dpseci_setup(dpseci_dev);
	if (err < 0) {
		dev_err(dev, "dpaa2_dpseci_setup() failed\n");
		goto err_dpseci_setup;
	}

	/* DPIO */
	err = dpaa2_dpseci_dpio_setup(priv);
	if (err) {
		dev_err(dev, "dpaa2_dpseci_dpio_setup() failed\n");
		goto err_dpio_setup;
	}

	/* DPSECI binding to DPIO */
	err = dpaa2_dpseci_bind(priv);
	if (err) {
		dev_err(dev, "dpaa2_dpseci_bind() failed\n");
		goto err_bind;
	}

	/* DPSECI enable */
	err = dpaa2_dpseci_enable(priv);
	if (err) {
		dev_err(dev, "dpaa2_dpseci_enable() failed");
		goto err_bind;
	}

	/* register crypto algorithms the device supports */
	INIT_LIST_HEAD(&alg_list);
	for (i = 0; i < ARRAY_SIZE(driver_algs); i++) {
		/* TODO: check if h/w supports alg */
		struct caam_crypto_alg *t_alg;
		struct caam_alg_template *alg = driver_algs + i;

		t_alg = caam_alg_alloc(alg);
		if (IS_ERR(t_alg)) {
			err = PTR_ERR(t_alg);
			dev_warn(dev, "%s alg allocation failed: %d\n",
				 alg->driver_name, err);
			continue;
		}
		t_alg->caam.dev = dev;

		err = crypto_register_alg(&t_alg->crypto_alg);
		if (err) {
			dev_warn(dev, "%s alg registration failed: %d\n",
				 t_alg->crypto_alg.cra_driver_name, err);
			kfree(t_alg);
			continue;
		}

		list_add_tail(&t_alg->entry, &alg_list);
		registered = true;
	}

	for (i = 0; i < ARRAY_SIZE(driver_aeads); i++) {
		struct caam_aead_alg *t_alg = driver_aeads + i;

		t_alg->caam.dev = dev;
		caam_aead_alg_init(t_alg);

		err = crypto_register_aead(&t_alg->aead);
		if (err) {
			dev_warn(dev, "%s alg registration failed: %d\n",
				 t_alg->aead.base.cra_driver_name, err);
			continue;
		}

		t_alg->registered = true;
		registered = true;
	}
	if (registered)
		dev_info(dev, "algorithms registered in /proc/crypto\n");

	return err;

err_bind:
	dpaa2_dpseci_dpio_free(priv);
err_dpio_setup:
	dpaa2_dpseci_free(priv);
err_dpseci_setup:
	free_percpu(priv->ppriv);
err_alloc_ppriv:
	fsl_mc_portal_free(priv->mc_io);
err_dma_mask:
	kmem_cache_destroy(qi_cache);
err_qicache:
	dev_set_drvdata(dev, NULL);

	return err;
}

static int __cold dpaa2_caam_remove(struct fsl_mc_device *ls_dev)
{
	struct device *dev;
	struct dpaa2_caam_priv *priv;
	int i;

	dev = &ls_dev->dev;
	priv = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(driver_aeads); i++) {
		struct caam_aead_alg *t_alg = driver_aeads + i;

		if (t_alg->registered)
			crypto_unregister_aead(&t_alg->aead);
	}

	if (alg_list.next) {
		struct caam_crypto_alg *t_alg, *n;

		list_for_each_entry_safe(t_alg, n, &alg_list, entry) {
			crypto_unregister_alg(&t_alg->crypto_alg);
			list_del(&t_alg->entry);
			kfree(t_alg);
		}
	}

	dpaa2_dpseci_disable(priv);
	dpaa2_dpseci_dpio_free(priv);
	dpaa2_dpseci_free(priv);
	free_percpu(priv->ppriv);
	fsl_mc_portal_free(priv->mc_io);
	dev_set_drvdata(dev, NULL);
	kmem_cache_destroy(qi_cache);

	return 0;
}

int dpaa2_caam_enqueue(struct device *dev, struct caam_request *req)
{
	struct dpaa2_fd fd;
	struct dpaa2_caam_priv *priv = dev_get_drvdata(dev);
	int err = 0, i, id;

	if (IS_ERR(req))
		return PTR_ERR(req);

	if (priv->cscn_mem) {
		dma_sync_single_for_cpu(priv->dev, priv->cscn_dma,
					DPAA2_CSCN_SIZE,
					DMA_FROM_DEVICE);
		if (unlikely(dpaa2_cscn_state_congested(priv->cscn_mem_aligned))) {
			dev_dbg_ratelimited(dev, "Dropping request\n");
			return -EBUSY;
		}
	}

	dpaa2_fl_set_flc(&req->fd_flt[1], req->flc_dma);

	req->fd_flt_dma = dma_map_single(dev, req->fd_flt, sizeof(req->fd_flt),
					 DMA_TO_DEVICE);
	if (dma_mapping_error(dev, req->fd_flt_dma)) {
		dev_err(dev, "DMA mapping error for QI enqueue request\n");
		goto err_out;
	}

	memset(&fd, 0, sizeof(fd));
	dpaa2_fd_set_format(&fd, dpaa2_fd_list);
	dpaa2_fd_set_addr(&fd, req->fd_flt_dma);
	dpaa2_fd_set_len(&fd, req->fd_flt[1].len);
	dpaa2_fd_set_flc(&fd, req->flc_dma);

	/*
	 * There is no guarantee that preemption is disabled here,
	 * thus take action.
	 */
	preempt_disable();
	id = smp_processor_id() % priv->dpseci_attr.num_tx_queues;
	for (i = 0; i < (priv->dpseci_attr.num_tx_queues << 1); i++) {
		err = dpaa2_io_service_enqueue_fq(NULL,
						  priv->tx_queue_attr[id].fqid,
						  &fd);
		if (err != -EBUSY)
			break;
	}
	preempt_enable();

	if (unlikely(err < 0)) {
		dev_err(dev, "Error enqueuing frame: %d\n", err);
		goto err_out;
	}

	return -EINPROGRESS;

err_out:
	dma_unmap_single(dev, req->fd_flt_dma, sizeof(req->fd_flt),
			 DMA_TO_DEVICE);
	return -EIO;
}
EXPORT_SYMBOL(dpaa2_caam_enqueue);

const struct fsl_mc_device_id dpaa2_caam_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpseci",
	},
	{ .vendor = 0x0 }
};

static struct fsl_mc_driver dpaa2_caam_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa2_caam_probe,
	.remove		= dpaa2_caam_remove,
	.match_id_table = dpaa2_caam_match_id_table
};

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Freescale Semiconductor, Inc");
MODULE_DESCRIPTION("Freescale DPAA2 CAAM Driver");

module_fsl_mc_driver(dpaa2_caam_driver);
