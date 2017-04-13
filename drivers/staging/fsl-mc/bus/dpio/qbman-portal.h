/*
 * Copyright (C) 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __FSL_QBMAN_PORTAL_H
#define __FSL_QBMAN_PORTAL_H

#include "qbman_private.h"
#include "../../include/dpaa2-fd.h"

struct dpaa2_dq;
struct qbman_swp;

/* qbman software portal descriptor structure */
struct qbman_swp_desc {
	void *cena_bar; /* Cache-enabled portal base address */
	void *cinh_bar; /* Cache-inhibited portal base address */
	u32 qman_version;
};

#define QBMAN_SWP_INTERRUPT_EQRI 0x01
#define QBMAN_SWP_INTERRUPT_EQDI 0x02
#define QBMAN_SWP_INTERRUPT_DQRI 0x04
#define QBMAN_SWP_INTERRUPT_RCRI 0x08
#define QBMAN_SWP_INTERRUPT_RCDI 0x10
#define QBMAN_SWP_INTERRUPT_VDCI 0x20

/* the structure for pull dequeue descriptor */
struct qbman_pull_desc {
	u8 verb;
	u8 numf;
	u8 tok;
	u8 reserved;
	u32 dq_src;
	u64 rsp_addr;
	u64 rsp_addr_virt;
	u8 padding[40];
};

enum qbman_pull_type_e {
	/* dequeue with priority precedence, respect intra-class scheduling */
	qbman_pull_type_prio = 1,
	/* dequeue with active FQ precedence, respect ICS */
	qbman_pull_type_active,
	/* dequeue with active FQ precedence, no ICS */
	qbman_pull_type_active_noics
};

/* Definitions for parsing dequeue entries */
#define QBMAN_RESULT_MASK      0x7f
#define QBMAN_RESULT_DQ        0x60
#define QBMAN_RESULT_FQRN      0x21
#define QBMAN_RESULT_FQRNI     0x22
#define QBMAN_RESULT_FQPN      0x24
#define QBMAN_RESULT_FQDAN     0x25
#define QBMAN_RESULT_CDAN      0x26
#define QBMAN_RESULT_CSCN_MEM  0x27
#define QBMAN_RESULT_CGCU      0x28
#define QBMAN_RESULT_BPSCN     0x29
#define QBMAN_RESULT_CSCN_WQ   0x2a

/* QBMan FQ management command codes */
#define QBMAN_FQ_SCHEDULE	0x48
#define QBMAN_FQ_FORCE		0x49
#define QBMAN_FQ_XON		0x4d
#define QBMAN_FQ_XOFF		0x4e

/* structure of enqueue descriptor */
struct qbman_eq_desc {
	u8 verb;
	u8 dca;
	u16 seqnum;
	u16 orpid;
	u16 reserved1;
	u32 tgtid;
	u32 tag;
	u16 qdbin;
	u8 qpri;
	u8 reserved[3];
	u8 wae;
	u8 rspid;
	u64 rsp_addr;
	u8 fd[32];
};

/* buffer release descriptor */
struct qbman_release_desc {
	u8 verb;
	u8 reserved;
	u16 bpid;
	u32 reserved2;
	u64 buf[7];
};

/* Management command result codes */
#define QBMAN_MC_RSLT_OK      0xf0

#define CODE_CDAN_WE_EN    0x1
#define CODE_CDAN_WE_CTX   0x4

/* portal data structure */
struct qbman_swp {
	const struct qbman_swp_desc *desc;
	void __iomem *addr_cena;
	void __iomem *addr_cinh;

	/* Management commands */
	struct {
		u32 valid_bit; /* 0x00 or 0x80 */
	} mc;

	/* Push dequeues */
	u32 sdq;

	/* Volatile dequeues */
	struct {
		atomic_t available; /* indicates if a command can be sent */
		u32 valid_bit; /* 0x00 or 0x80 */
		struct dpaa2_dq *storage; /* NULL if DQRR */
	} vdq;

	/* DQRR */
	struct {
		u32 next_idx;
		u32 valid_bit;
		u8 dqrr_size;
		int reset_bug; /* indicates dqrr reset workaround is needed */
	} dqrr;
};

struct qbman_swp *qbman_swp_init(const struct qbman_swp_desc *d);
void qbman_swp_finish(struct qbman_swp *p);
u32 qbman_swp_interrupt_read_status(struct qbman_swp *p);
void qbman_swp_interrupt_clear_status(struct qbman_swp *p, u32 mask);
u32 qbman_swp_interrupt_get_trigger(struct qbman_swp *p);
void qbman_swp_interrupt_set_trigger(struct qbman_swp *p, u32 mask);
int qbman_swp_interrupt_get_inhibit(struct qbman_swp *p);
void qbman_swp_interrupt_set_inhibit(struct qbman_swp *p, int inhibit);

void qbman_swp_push_get(struct qbman_swp *p, u8 channel_idx, int *enabled);
void qbman_swp_push_set(struct qbman_swp *p, u8 channel_idx, int enable);

void qbman_pull_desc_clear(struct qbman_pull_desc *d);
void qbman_pull_desc_set_storage(struct qbman_pull_desc *d,
				 struct dpaa2_dq *storage,
				 dma_addr_t storage_phys,
				 int stash);
void qbman_pull_desc_set_numframes(struct qbman_pull_desc *d, u8 numframes);
void qbman_pull_desc_set_fq(struct qbman_pull_desc *d, u32 fqid);
void qbman_pull_desc_set_wq(struct qbman_pull_desc *d, u32 wqid,
			    enum qbman_pull_type_e dct);
void qbman_pull_desc_set_channel(struct qbman_pull_desc *d, u32 chid,
				 enum qbman_pull_type_e dct);

int qbman_swp_pull(struct qbman_swp *p, struct qbman_pull_desc *d);

const struct dpaa2_dq *qbman_swp_dqrr_next(struct qbman_swp *s);
void qbman_swp_dqrr_consume(struct qbman_swp *s, const struct dpaa2_dq *dq);

int qbman_result_has_new_result(struct qbman_swp *p, const struct dpaa2_dq *dq);

void qbman_eq_desc_clear(struct qbman_eq_desc *d);
void qbman_eq_desc_set_no_orp(struct qbman_eq_desc *d, int respond_success);
void qbman_eq_desc_set_token(struct qbman_eq_desc *d, u8 token);
void qbman_eq_desc_set_fq(struct qbman_eq_desc *d, u32 fqid);
void qbman_eq_desc_set_qd(struct qbman_eq_desc *d, u32 qdid,
			  u32 qd_bin, u32 qd_prio);

int qbman_swp_enqueue(struct qbman_swp *p, const struct qbman_eq_desc *d,
		      const struct dpaa2_fd *fd);

void qbman_release_desc_clear(struct qbman_release_desc *d);
void qbman_release_desc_set_bpid(struct qbman_release_desc *d, u16 bpid);
void qbman_release_desc_set_rcdi(struct qbman_release_desc *d, int enable);

int qbman_swp_release(struct qbman_swp *s, const struct qbman_release_desc *d,
		      const u64 *buffers, unsigned int num_buffers);
int qbman_swp_acquire(struct qbman_swp *s, u16 bpid, u64 *buffers,
		      unsigned int num_buffers);
int qbman_swp_alt_fq_state(struct qbman_swp *s, u32 fqid,
			   u8 alt_fq_verb);
int qbman_swp_CDAN_set(struct qbman_swp *s, u16 channelid,
		       u8 we_mask, u8 cdan_en,
		       u64 ctx);

void *qbman_swp_mc_start(struct qbman_swp *p);
void qbman_swp_mc_submit(struct qbman_swp *p, void *cmd, u8 cmd_verb);
void *qbman_swp_mc_result(struct qbman_swp *p);

/**
 * qbman_result_is_DQ() - check if the dequeue result is a dequeue response
 * @dq: the dequeue result to be checked
 *
 * DQRR entries may contain non-dequeue results, ie. notifications
 */
static inline int qbman_result_is_DQ(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_DQ);
}

/**
 * qbman_result_is_SCN() - Check the dequeue result is notification or not
 * @dq: the dequeue result to be checked
 *
 */
static inline int qbman_result_is_SCN(const struct dpaa2_dq *dq)
{
	return !qbman_result_is_DQ(dq);
}

/* FQ Data Availability */
static inline int qbman_result_is_FQDAN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_FQDAN);
}

/* Channel Data Availability */
static inline int qbman_result_is_CDAN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_CDAN);
}

/* Congestion State Change */
static inline int qbman_result_is_CSCN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_CSCN_WQ);
}

/* Buffer Pool State Change */
static inline int qbman_result_is_BPSCN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_BPSCN);
}

/* Congestion Group Count Update */
static inline int qbman_result_is_CGCU(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_CGCU);
}

/* Retirement */
static inline int qbman_result_is_FQRN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_FQRN);
}

/* Retirement Immediate */
static inline int qbman_result_is_FQRNI(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_FQRNI);
}

 /* Park */
static inline int qbman_result_is_FQPN(const struct dpaa2_dq *dq)
{
	return ((dq->dq.verb & QBMAN_RESULT_MASK) == QBMAN_RESULT_FQPN);
}

/**
 * qbman_result_SCN_state() - Get the state field in State-change notification
 */
static inline u8 qbman_result_SCN_state(const struct dpaa2_dq *scn)
{
	return scn->scn.state;
}

#define SCN_RID_MASK 0x00FFFFFF

/**
 * qbman_result_SCN_rid() - Get the resource id in State-change notification
 */
static inline u32 qbman_result_SCN_rid(const struct dpaa2_dq *scn)
{
	return le32_to_cpu(scn->scn.rid_tok) & SCN_RID_MASK;
}

/**
 * qbman_result_SCN_ctx() - Get the context data in State-change notification
 */
static inline u64 qbman_result_SCN_ctx(const struct dpaa2_dq *scn)
{
	return le64_to_cpu(scn->scn.ctx);
}

/**
 * qbman_swp_fq_schedule() - Move the fq to the scheduled state
 * @s:    the software portal object
 * @fqid: the index of frame queue to be scheduled
 *
 * There are a couple of different ways that a FQ can end up parked state,
 * This schedules it.
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_fq_schedule(struct qbman_swp *s, u32 fqid)
{
	return qbman_swp_alt_fq_state(s, fqid, QBMAN_FQ_SCHEDULE);
}

/**
 * qbman_swp_fq_force() - Force the FQ to fully scheduled state
 * @s:    the software portal object
 * @fqid: the index of frame queue to be forced
 *
 * Force eligible will force a tentatively-scheduled FQ to be fully-scheduled
 * and thus be available for selection by any channel-dequeuing behaviour (push
 * or pull). If the FQ is subsequently "dequeued" from the channel and is still
 * empty at the time this happens, the resulting dq_entry will have no FD.
 * (qbman_result_DQ_fd() will return NULL.)
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_fq_force(struct qbman_swp *s, u32 fqid)
{
	return qbman_swp_alt_fq_state(s, fqid, QBMAN_FQ_FORCE);
}

/**
 * qbman_swp_fq_xon() - sets FQ flow-control to XON
 * @s:    the software portal object
 * @fqid: the index of frame queue
 *
 * This setting doesn't affect enqueues to the FQ, just dequeues.
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_fq_xon(struct qbman_swp *s, u32 fqid)
{
	return qbman_swp_alt_fq_state(s, fqid, QBMAN_FQ_XON);
}

/**
 * qbman_swp_fq_xoff() - sets FQ flow-control to XOFF
 * @s:    the software portal object
 * @fqid: the index of frame queue
 *
 * This setting doesn't affect enqueues to the FQ, just dequeues.
 * XOFF FQs will remain in the tenatively-scheduled state, even when
 * non-empty, meaning they won't be selected for scheduled dequeuing.
 * If a FQ is changed to XOFF after it had already become truly-scheduled
 * to a channel, and a pull dequeue of that channel occurs that selects
 * that FQ for dequeuing, then the resulting dq_entry will have no FD.
 * (qbman_result_DQ_fd() will return NULL.)
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_fq_xoff(struct qbman_swp *s, u32 fqid)
{
	return qbman_swp_alt_fq_state(s, fqid, QBMAN_FQ_XOFF);
}

/* If the user has been allocated a channel object that is going to generate
 * CDANs to another channel, then the qbman_swp_CDAN* functions will be
 * necessary.
 *
 * CDAN-enabled channels only generate a single CDAN notification, after which
 * they need to be reenabled before they'll generate another. The idea is
 * that pull dequeuing will occur in reaction to the CDAN, followed by a
 * reenable step. Each function generates a distinct command to hardware, so a
 * combination function is provided if the user wishes to modify the "context"
 * (which shows up in each CDAN message) each time they reenable, as a single
 * command to hardware.
 */

/**
 * qbman_swp_CDAN_set_context() - Set CDAN context
 * @s:         the software portal object
 * @channelid: the channel index
 * @ctx:       the context to be set in CDAN
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_CDAN_set_context(struct qbman_swp *s, u16 channelid,
					     u64 ctx)
{
	return qbman_swp_CDAN_set(s, channelid,
				  CODE_CDAN_WE_CTX,
				  0, ctx);
}

/**
 * qbman_swp_CDAN_enable() - Enable CDAN for the channel
 * @s:         the software portal object
 * @channelid: the index of the channel to generate CDAN
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_CDAN_enable(struct qbman_swp *s, u16 channelid)
{
	return qbman_swp_CDAN_set(s, channelid,
				  CODE_CDAN_WE_EN,
				  1, 0);
}

/**
 * qbman_swp_CDAN_disable() - disable CDAN for the channel
 * @s:         the software portal object
 * @channelid: the index of the channel to generate CDAN
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_CDAN_disable(struct qbman_swp *s, u16 channelid)
{
	return qbman_swp_CDAN_set(s, channelid,
				  CODE_CDAN_WE_EN,
				  0, 0);
}

/**
 * qbman_swp_CDAN_set_context_enable() - Set CDAN contest and enable CDAN
 * @s:         the software portal object
 * @channelid: the index of the channel to generate CDAN
 * @ctx:i      the context set in CDAN
 *
 * Return 0 for success, or negative error code for failure.
 */
static inline int qbman_swp_CDAN_set_context_enable(struct qbman_swp *s,
						    u16 channelid,
						    u64 ctx)
{
	return qbman_swp_CDAN_set(s, channelid,
				  CODE_CDAN_WE_EN | CODE_CDAN_WE_CTX,
				  1, ctx);
}

/* Wraps up submit + poll-for-result */
static inline void *qbman_swp_mc_complete(struct qbman_swp *swp, void *cmd,
					  u8 cmd_verb)
{
	int loopvar = 1000;

	qbman_swp_mc_submit(swp, cmd, cmd_verb);

	do {
		cmd = qbman_swp_mc_result(swp);
	} while (!cmd && loopvar--);

	WARN_ON(!loopvar);

	return cmd;
}

/* ------------ */
/* qb_attr_code */
/* ------------ */

/* This struct locates a sub-field within a QBMan portal (CENA) cacheline which
 * is either serving as a configuration command or a query result. The
 * representation is inherently little-endian, as the indexing of the words is
 * itself little-endian in nature and layerscape is little endian for anything
 * that crosses a word boundary too (64-bit fields are the obvious examples).
 */
struct qb_attr_code {
	unsigned int word; /* which u32[] array member encodes the field */
	unsigned int lsoffset; /* encoding offset from ls-bit */
	unsigned int width; /* encoding width. (bool must be 1.) */
};

/* Some pre-defined codes */
extern struct qb_attr_code code_generic_verb;
extern struct qb_attr_code code_generic_rslt;

/* Macros to define codes */
#define QB_CODE(a, b, c) { a, b, c}
#define QB_CODE_NULL \
	QB_CODE((unsigned int)-1, (unsigned int)-1, (unsigned int)-1)

/* Rotate a code "ms", meaning that it moves from less-significant bytes to
 * more-significant, from less-significant words to more-significant, etc. The
 * "ls" version does the inverse, from more-significant towards
 * less-significant.
 */
static inline void qb_attr_code_rotate_ms(struct qb_attr_code *code,
					  unsigned int bits)
{
	code->lsoffset += bits;
	while (code->lsoffset > 31) {
		code->word++;
		code->lsoffset -= 32;
	}
}

static inline void qb_attr_code_rotate_ls(struct qb_attr_code *code,
					  unsigned int bits)
{
	/* Don't be fooled, this trick should work because the types are
	 * unsigned. So the case that interests the while loop (the rotate has
	 * gone too far and the word count needs to compensate for it), is
	 * manifested when lsoffset is negative. But that equates to a really
	 * large unsigned value, starting with lots of "F"s. As such, we can
	 * continue adding 32 back to it until it wraps back round above zero,
	 * to a value of 31 or less...
	 */
	code->lsoffset -= bits;
	while (code->lsoffset > 31) {
		code->word--;
		code->lsoffset += 32;
	}
}

/* Implement a loop of code rotations until 'expr' evaluates to FALSE (0). */
#define qb_attr_code_for_ms(code, bits, expr) \
		for (; expr; qb_attr_code_rotate_ms(code, bits))
#define qb_attr_code_for_ls(code, bits, expr) \
		for (; expr; qb_attr_code_rotate_ls(code, bits))

static inline void word_copy(void *d, const void *s, unsigned int cnt)
{
	u32 *dd = d;
	const u32 *ss = s;

	while (cnt--)
		*(dd++) = *(ss++);
}

/*
 * Currently, the CENA support code expects each 32-bit word to be written in
 * host order, and these are converted to hardware (little-endian) order on
 * command submission. However, 64-bit quantities are must be written (and read)
 * as two 32-bit words with the least-significant word first, irrespective of
 * host endianness.
 */
static inline void u64_to_le32_copy(void *d, const u64 *s,
				    unsigned int cnt)
{
	u32 *dd = d;
	const u32 *ss = (const u32 *)s;

	while (cnt--) {
		/*
		 * TBD: the toolchain was choking on the use of 64-bit types up
		 * until recently so this works entirely with 32-bit variables.
		 * When 64-bit types become usable again, investigate better
		 * ways of doing this.
		 */
#if defined(__BIG_ENDIAN)
		*(dd++) = ss[1];
		*(dd++) = ss[0];
		ss += 2;
#else
		*(dd++) = *(ss++);
		*(dd++) = *(ss++);
#endif
	}
}

static inline void u64_from_le32_copy(u64 *d, const void *s,
				      unsigned int cnt)
{
	const u32 *ss = s;
	u32 *dd = (u32 *)d;

	while (cnt--) {
#if defined(__BIG_ENDIAN)
		dd[1] = *(ss++);
		dd[0] = *(ss++);
		dd += 2;
#else
		*(dd++) = *(ss++);
		*(dd++) = *(ss++);
#endif
	}
}

/* decode a field from a cacheline */
static inline u32 qb_attr_code_decode(const struct qb_attr_code *code,
				      const u32 *cacheline)
{
	return d32_u32(code->lsoffset, code->width, cacheline[code->word]);
}

static inline u64 qb_attr_code_decode_64(const struct qb_attr_code *code,
					 const u64 *cacheline)
{
	u64 res;

	u64_from_le32_copy(&res, &cacheline[code->word / 2], 1);
	return res;
}

/* encode a field to a cacheline */
static inline void qb_attr_code_encode(const struct qb_attr_code *code,
				       u32 *cacheline, u32 val)
{
	cacheline[code->word] =
		r32_u32(code->lsoffset, code->width, cacheline[code->word])
		| e32_u32(code->lsoffset, code->width, val);
}

static inline void qb_attr_code_encode_64(const struct qb_attr_code *code,
					  u64 *cacheline, u64 val)
{
	u64_to_le32_copy(&cacheline[code->word / 2], &val, 1);
}

/* Small-width signed values (two's-complement) will decode into medium-width
 * positives. (Eg. for an 8-bit signed field, which stores values from -128 to
 * +127, a setting of -7 would appear to decode to the 32-bit unsigned value
 * 249. Likewise -120 would decode as 136.) This function allows the caller to
 * "re-sign" such fields to 32-bit signed. (Eg. -7, which was 249 with an 8-bit
 * encoding, will become 0xfffffff9 if you cast the return value to u32).
 */
static inline int32_t qb_attr_code_makesigned(const struct qb_attr_code *code,
					      u32 val)
{
	WARN_ON(val >= (1 << code->width));
	/* If the high bit was set, it was encoding a negative */
	if (val >= (1 << (code->width - 1)))
		return (int32_t)0 - (int32_t)(((u32)1 << code->width) -
			val);
	/* Otherwise, it was encoding a positive */
	return (int32_t)val;
}

/* ---------------------- */
/* Descriptors/cachelines */
/* ---------------------- */

/* To avoid needless dynamic allocation, the driver API often gives the caller
 * a "descriptor" type that the caller can instantiate however they like.
 * Ultimately though, it is just a cacheline of binary storage (or something
 * smaller when it is known that the descriptor doesn't need all 64 bytes) for
 * holding pre-formatted pieces of hardware commands. The performance-critical
 * code can then copy these descriptors directly into hardware command
 * registers more efficiently than trying to construct/format commands
 * on-the-fly. The API user sees the descriptor as an array of 32-bit words in
 * order for the compiler to know its size, but the internal details are not
 * exposed. The following macro is used within the driver for converting *any*
 * descriptor pointer to a usable array pointer. The use of a macro (instead of
 * an inline) is necessary to work with different descriptor types and to work
 * correctly with const and non-const inputs (and similarly-qualified outputs).
 */
#define qb_cl(d) (&(d)->dont_manipulate_directly[0])

#endif /* __FSL_QBMAN_PORTAL_H */
