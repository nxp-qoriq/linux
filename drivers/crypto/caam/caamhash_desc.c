/*
 * Shared descriptors for ahash algorithms
 *
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
#include "desc_constr.h"
#include "caamhash_desc.h"

/**
 * cnstr_shdsc_ahash - ahash shared descriptor
 * @desc: pointer to buffer used for descriptor construction
 * @adata: pointer to authentication transform definitions.
 *         A split key is required for SEC Era < 6; the size of the split key
 *         is specified in this case.
 *         Valid algorithm values - one of OP_ALG_ALGSEL_{MD5, SHA1, SHA224,
 *         SHA256, SHA384, SHA512}.
 * @state: algorithm state OP_ALG_AS_{INIT, FINALIZE, INITFINALIZE, UPDATE}
 * @digestsize: algorithm's digest size
 * @ctx_len: size of Context Register
 * @import_ctx: true if previous Context Register needs to be restored
 *              must be true for ahash update and final
 *              must be false for for ahash first and digest
 * @era: SEC Era
 */
void cnstr_shdsc_ahash(u32 * const desc, struct alginfo *adata, u32 state,
		       int digestsize, int ctx_len, bool import_ctx, int era)
{
	u32 op = adata->algtype;

	init_sh_desc(desc, HDR_SHARE_SERIAL);

	/* Append key if it has been set; ahash update excluded */
	if (state != OP_ALG_AS_UPDATE && adata->keylen) {
		u32 *skip_key_load;

		/* Skip key loading if already shared */
		skip_key_load = append_jump(desc, JUMP_JSL | JUMP_TEST_ALL |
					    JUMP_COND_SHRD);

		if (era < 6)
			append_key_as_imm(desc, adata->key_virt,
					  adata->keylen_pad,
					  adata->keylen, CLASS_2 |
					  KEY_DEST_MDHA_SPLIT | KEY_ENC);
		else
			append_proto_dkp(desc, adata);

		set_jump_tgt_here(desc, skip_key_load);

		op |= OP_ALG_AAI_HMAC_PRECOMP;
	}

	/* If needed, import context from software */
	if (import_ctx)
		append_seq_load(desc, ctx_len, LDST_CLASS_2_CCB |
				LDST_SRCDST_BYTE_CONTEXT);

	/* Class 2 operation */
	append_operation(desc, op | state | OP_ALG_ENCRYPT);

	/*
	 * Load from buf and/or src and write to req->result or state->context
	 * Calculate remaining bytes to read
	 */
	append_math_add(desc, VARSEQINLEN, SEQINLEN, REG0, CAAM_CMD_SZ);
	/* Read remaining bytes */
	append_seq_fifo_load(desc, 0, FIFOLD_CLASS_CLASS2 | FIFOLD_TYPE_LAST2 |
			     FIFOLD_TYPE_MSG | KEY_VLF);
	/* Store class2 context bytes */
	append_seq_store(desc, digestsize, LDST_CLASS_2_CCB |
			 LDST_SRCDST_BYTE_CONTEXT);
}
EXPORT_SYMBOL(cnstr_shdsc_ahash);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL CAAM ahash descriptors support");
MODULE_AUTHOR("NXP Semiconductors");
