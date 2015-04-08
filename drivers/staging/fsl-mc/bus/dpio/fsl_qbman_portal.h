/* Copyright (C) 2014 Freescale Semiconductor, Inc.
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
#ifndef _FSL_QBMAN_PORTAL_H
#define _FSL_QBMAN_PORTAL_H

#include "fsl_qbman_base.h"

/**
 * qbman_swp_init() - Create a functional object representing the given
 * QBMan portal descriptor.
 * @d: the given qbman swp descriptor
 *
 * Return qbman_swp portal object for success, NULL if the object cannot
 * be created.
 */
struct qbman_swp *qbman_swp_init(const struct qbman_swp_desc *d);
/**
 * qbman_swp_finish() - Create and destroy a functional object representing
 * the given QBMan portal descriptor.
 * @p: the qbman_swp object to be destroyed.
 *
 */
void qbman_swp_finish(struct qbman_swp *p);

/**
 * qbman_swp_get_desc() - Get the descriptor of the given portal object.
 * @p: the given portal object.
 *
 * Return the descriptor for this portal.
 */
const struct qbman_swp_desc *qbman_swp_get_desc(struct qbman_swp *p);

	/**************/
	/* Interrupts */
	/**************/

/* See the QBMan driver API documentation for details on the interrupt
 * mechanisms. */
#define QBMAN_SWP_INTERRUPT_EQRI ((uint32_t)0x00000001)
#define QBMAN_SWP_INTERRUPT_EQDI ((uint32_t)0x00000002)
#define QBMAN_SWP_INTERRUPT_DQRI ((uint32_t)0x00000004)
#define QBMAN_SWP_INTERRUPT_RCRI ((uint32_t)0x00000008)
#define QBMAN_SWP_INTERRUPT_RCDI ((uint32_t)0x00000010)
#define QBMAN_SWP_INTERRUPT_VDCI ((uint32_t)0x00000020)

/**
 * qbman_swp_interrupt_get_vanish()
 * qbman_swp_interrupt_set_vanish() - Get/Set the data in software portal
 * interrupt status disable register.
 * @p: the given software portal object.
 * @mask: The mask to set in SWP_IDSR register.
 *
 * Return the settings in SWP_ISDR register for Get function.
 */
uint32_t qbman_swp_interrupt_get_vanish(struct qbman_swp *p);
void qbman_swp_interrupt_set_vanish(struct qbman_swp *p, uint32_t mask);

/**
 * qbman_swp_interrupt_read_status()
 * qbman_swp_interrupt_clear_status() - Get/Set the data in software portal
 * interrupt status register.
 * @p: the given software portal object.
 * @mask: The mask to set in SWP_ISR register.
 *
 * Return the settings in SWP_ISR register for Get function.
 *
 */
uint32_t qbman_swp_interrupt_read_status(struct qbman_swp *p);
void qbman_swp_interrupt_clear_status(struct qbman_swp *p, uint32_t mask);

/**
 * qbman_swp_interrupt_get_trigger()
 * qbman_swp_interrupt_set_trigger() - Get/Set the data in software portal
 * interrupt enable register.
 * @p: the given software portal object.
 * @mask: The mask to set in SWP_IER register.
 *
 * Return the settings in SWP_IER register for Get function.
 */
uint32_t qbman_swp_interrupt_get_trigger(struct qbman_swp *p);
void qbman_swp_interrupt_set_trigger(struct qbman_swp *p, uint32_t mask);

/**
 * qbman_swp_interrupt_get_inhibit()
 * qbman_swp_interrupt_set_inhibit() - Set/Set the data in software portal
 * interrupt inhibit register.
 * @p: the given software portal object.
 * @mask: The mask to set in SWP_IIR register.
 *
 * Return the settings in SWP_IIR register for Get function.
 */
int qbman_swp_interrupt_get_inhibit(struct qbman_swp *p);
void qbman_swp_interrupt_set_inhibit(struct qbman_swp *p, int inhibit);

	/************/
	/* Dequeues */
	/************/

/* See the QBMan driver API documentation for details on the enqueue
 * mechanisms. NB: the use of a 'dpaa2_' prefix for this type is because it is
 * primarily used by the "DPIO" layer that sits above (and hides) the QBMan
 * driver. The structure is defined in the DPIO interface, but to avoid circular
 * dependencies we just pre/re-declare it here opaquely. */
struct dpaa2_dq;

/* ------------------- */
/* Push-mode dequeuing */
/* ------------------- */

/**
 * qbman_swp_push_get() - Get the push dequeue setup.
 * @p: the software portal object.
 * @channel_idx: the channel index to query.
 * @enabled: returned boolean to show whether the push dequeue is enabled for
 * the given channel.
 */
void qbman_swp_push_get(struct qbman_swp *, uint8_t channel_idx, int *enabled);
/**
 * qbman_swp_push_set() - Enable or disable push dequeue.
 * @p: the software portal object.
 * @channel_idx: the channel index..
 * @enable: enable or disable push dequeue.
 *
 * The user of a portal can enable and disable push-mode dequeuing of up to 16
 * channels independently. It does not specify this toggling by channel IDs, but
 * rather by specifying the index (from 0 to 15) that has been mapped to the
 * desired channel.
 */
void qbman_swp_push_set(struct qbman_swp *, uint8_t channel_idx, int enable);

/* ------------------- */
/* Pull-mode dequeuing */
/* ------------------- */

/**
 * struct qbman_pull_desc - the structure for pull dequeue descriptor
 */
struct qbman_pull_desc {
	uint32_t dont_manipulate_directly[6];
};

enum qbman_pull_type_e {
	/* dequeue with priority precedence, respect intra-class scheduling */
	qbman_pull_type_prio = 1,
	/* dequeue with active FQ precedence, respect ICS */
	qbman_pull_type_active,
	/* dequeue with active FQ precedence, no ICS */
	qbman_pull_type_active_noics
};

/**
 * qbman_pull_desc_clear() - Clear the contents of a descriptor to
 * default/starting state.
 * @d: the pull dequeue descriptor to be cleared.
 */
void qbman_pull_desc_clear(struct qbman_pull_desc *d);

/**
 * qbman_pull_desc_set_storage()- Set the pull dequeue storage
 * @d: the pull dequeue descriptor to be set.
 * @storage: the pointer of the memory to store the dequeue result.
 * @storage_phys: the physical address of the storage memory.
 * @stash: to indicate whether write allocate is enabled.
 *
 * If not called, or if called with 'storage' as NULL, the result pull dequeues
 * will produce results to DQRR. If 'storage' is non-NULL, then results are
 * produced to the given memory location (using the physical/DMA address which
 * the caller provides in 'storage_phys'), and 'stash' controls whether or not
 * those writes to main-memory express a cache-warming attribute.
 */
void qbman_pull_desc_set_storage(struct qbman_pull_desc *d,
				 struct dpaa2_dq *storage,
				 dma_addr_t storage_phys,
				 int stash);
/**
 * qbman_pull_desc_set_numframes() - Set the number of frames to be dequeued.
 * @d: the pull dequeue descriptor to be set.
 * @numframes: number of frames to be set, must be between 1 and 16, inclusive.
 */
void qbman_pull_desc_set_numframes(struct qbman_pull_desc *, uint8_t numframes);

/**
 * qbman_pull_desc_set_fq() - Set fqid from which the dequeue command dequeues.
 * @fqid: the frame queue index of the given FQ.
 *
 * qbman_pull_desc_set_wq() - Set wqid from which the dequeue command dequeues.
 * @wqid: composed of channel id and wqid within the channel.
 * @dct: the dequeue command type.
 *
 * qbman_pull_desc_set_channel() - Set channelid from which the dequeue command
 * dequeues.
 * @chid: the channel id to be dequeued.
 * @dct: the dequeue command type.
 *
 * Exactly one of the following descriptor "actions" should be set. (Calling any
 * one of these will replace the effect of any prior call to one of these.)
 * - pull dequeue from the given frame queue (FQ)
 * - pull dequeue from any FQ in the given work queue (WQ)
 * - pull dequeue from any FQ in any WQ in the given channel
 */
void qbman_pull_desc_set_fq(struct qbman_pull_desc *, uint32_t fqid);
void qbman_pull_desc_set_wq(struct qbman_pull_desc *, uint32_t wqid,
			    enum qbman_pull_type_e dct);
void qbman_pull_desc_set_channel(struct qbman_pull_desc *, uint32_t chid,
				 enum qbman_pull_type_e dct);

/**
 * qbman_swp_pull() - Issue the pull dequeue command
 * @s: the software portal object.
 * @d: the software portal descriptor which has been configured with
 * the set of qbman_pull_desc_set_*() calls.
 *
 * Return 0 for success, and -EBUSY if the software portal is not ready
 * to do pull dequeue.
 */
int qbman_swp_pull(struct qbman_swp *, struct qbman_pull_desc *d);

/* -------------------------------- */
/* Polling DQRR for dequeue results */
/* -------------------------------- */

/**
 * qbman_swp_dqrr_next() - Get an valid DQRR entry.
 * @s: the software portal object.
 *
 * Return NULL if there are no unconsumed DQRR entries. Return a DQRR entry
 * only once, so repeated calls can return a sequence of DQRR entries, without
 * requiring they be consumed immediately or in any particular order.
 */
const struct dpaa2_dq *qbman_swp_dqrr_next(struct qbman_swp *s);

/**
 * qbman_swp_dqrr_consume() -  Consume DQRR entries previously returned from
 * qbman_swp_dqrr_next().
 * @s: the software portal object.
 * @dq: the DQRR entry to be consumed.
 */
void qbman_swp_dqrr_consume(struct qbman_swp *s, const struct dpaa2_dq *dq);

/* ------------------------------------------------- */
/* Polling user-provided storage for dequeue results */
/* ------------------------------------------------- */
/**
 * qbman_result_has_new_result() - Check and get the dequeue response from the
 * dq storage memory set in pull dequeue command
 * @s: the software portal object.
 * @dq: the dequeue result read from the memory.
 *
 * Only used for user-provided storage of dequeue results, not DQRR. For
 * efficiency purposes, the driver will perform any required endianness
 * conversion to ensure that the user's dequeue result storage is in host-endian
 * format (whether or not that is the same as the little-endian format that
 * hardware DMA'd to the user's storage). As such, once the user has called
 * qbman_result_has_new_result() and been returned a valid dequeue result,
 * they should not call it again on the same memory location (except of course
 * if another dequeue command has been executed to produce a new result to that
 * location).
 *
 * Return 1 for getting a valid dequeue result, or 0 for not getting a valid
 * dequeue result.
 */
int qbman_result_has_new_result(struct qbman_swp *,
				  const struct dpaa2_dq *);

/* -------------------------------------------------------- */
/* Parsing dequeue entries (DQRR and user-provided storage) */
/* -------------------------------------------------------- */

/**
 * qbman_result_is_DQ() - check the dequeue result is a dequeue response or not
 * @dq: the dequeue result to be checked.
 *
 * DQRR entries may contain non-dequeue results, ie. notifications
 */
int qbman_result_is_DQ(const struct dpaa2_dq *);

/**
 * qbman_result_is_SCN() - Check the dequeue result is notification or not
 * @dq: the dequeue result to be checked.
 *
 * All the non-dequeue results (FQDAN/CDAN/CSCN/...) are "state change
 * notifications" of one type or another. Some APIs apply to all of them, of the
 * form qbman_result_SCN_***().
 */
static inline int qbman_result_is_SCN(const struct dpaa2_dq *dq)
{
	return !qbman_result_is_DQ(dq);
}

/**
 * Recognise different notification types, only required if the user allows for
 * these to occur, and cares about them when they do.
 */
int qbman_result_is_FQDAN(const struct dpaa2_dq *);
				/* FQ Data Availability */
int qbman_result_is_CDAN(const struct dpaa2_dq *);
				/* Channel Data Availability */
int qbman_result_is_CSCN(const struct dpaa2_dq *);
				/* Congestion State Change */
int qbman_result_is_BPSCN(const struct dpaa2_dq *);
				/* Buffer Pool State Change */
int qbman_result_is_CGCU(const struct dpaa2_dq *);
				/* Congestion Group Count Update */
/* Frame queue state change notifications; (FQDAN in theory counts too as it
 * leaves a FQ parked, but it is primarily a data availability notification) */
int qbman_result_is_FQRN(const struct dpaa2_dq *); /* Retirement */
int qbman_result_is_FQRNI(const struct dpaa2_dq *);
				/* Retirement Immediate */
int qbman_result_is_FQPN(const struct dpaa2_dq *); /* Park */

/* NB: for parsing dequeue results (when "is_DQ" is TRUE), use the higher-layer
 * dpaa2_dq_*() functions. */

/* State-change notifications (FQDAN/CDAN/CSCN/...). */
/**
 * qbman_result_SCN_state() - Get the state field in State-change notification
 */
uint8_t qbman_result_SCN_state(const struct dpaa2_dq *);
/**
 * qbman_result_SCN_rid() - Get the resource id in State-change notification
 */
uint32_t qbman_result_SCN_rid(const struct dpaa2_dq *);
/**
 * qbman_result_SCN_ctx() - Get the context data in State-change notification
 */
uint64_t qbman_result_SCN_ctx(const struct dpaa2_dq *);
/**
 * qbman_result_SCN_state_in_mem() - Get the state field in State-change
 * notification which is written to memory instead of DQRR.
 */
uint8_t qbman_result_SCN_state_in_mem(const struct dpaa2_dq *);
/**
 * qbman_result_SCN_rid_in_mem() - Get the resource id in State-change
 * notification which is written to memory instead of DQRR.
 */
uint32_t qbman_result_SCN_rid_in_mem(const struct dpaa2_dq *);

/* Type-specific "resource IDs". Mainly for illustration purposes, though it
 * also gives the appropriate type widths. */
#define qbman_result_FQDAN_fqid(dq) qbman_result_SCN_rid(dq)
#define qbman_result_FQRN_fqid(dq) qbman_result_SCN_rid(dq)
#define qbman_result_FQRNI_fqid(dq) qbman_result_SCN_rid(dq)
#define qbman_result_FQPN_fqid(dq) qbman_result_SCN_rid(dq)
#define qbman_result_CDAN_cid(dq) ((uint16_t)qbman_result_SCN_rid(dq))
#define qbman_result_CSCN_cgid(dq) ((uint16_t)qbman_result_SCN_rid(dq))

/**
 * qbman_result_bpscn_bpid() - Get the bpid from BPSCN
 *
 * Return the buffer pool id.
 */
uint16_t qbman_result_bpscn_bpid(const struct dpaa2_dq *);
/**
 * qbman_result_bpscn_has_free_bufs() - Check whether there are free
 * buffers in the pool from BPSCN.
 *
 * Return the number of free buffers.
 */
int qbman_result_bpscn_has_free_bufs(const struct dpaa2_dq *);
/**
 * qbman_result_bpscn_is_depleted() - Check BPSCN to see whether the
 * buffer pool is depleted.
 *
 * Return the status of buffer pool depletion.
 */
int qbman_result_bpscn_is_depleted(const struct dpaa2_dq *);
/**
 * qbman_result_bpscn_is_surplus() - Check BPSCN to see whether the buffer
 * pool is surplus or not.
 *
 * Return the status of buffer pool surplus.
 */
int qbman_result_bpscn_is_surplus(const struct dpaa2_dq *);
/**
 * qbman_result_bpscn_ctx() - Get the BPSCN CTX from BPSCN message
 *
 * Return the BPSCN context.
 */
uint64_t qbman_result_bpscn_ctx(const struct dpaa2_dq *);

/* Parsing CGCU */
/**
 * qbman_result_cgcu_cgid() - Check CGCU resouce id, i.e. cgid
 *
 * Return the CGCU resource id.
 */
uint16_t qbman_result_cgcu_cgid(const struct dpaa2_dq *);
/**
 * qbman_result_cgcu_icnt() - Get the I_CNT from CGCU
 *
 * Return instantaneous count in the CGCU notification.
 */
uint64_t qbman_result_cgcu_icnt(const struct dpaa2_dq *);

	/************/
	/* Enqueues */
	/************/
/**
 * struct qbman_eq_desc - structure of enqueue descriptor
 */
struct qbman_eq_desc {
	uint32_t dont_manipulate_directly[8];
};

/**
 * struct qbman_eq_response - structure of enqueue response
 */
struct qbman_eq_response {
	uint32_t dont_manipulate_directly[16];
};

/**
 * qbman_eq_desc_clear() - Clear the contents of a descriptor to
 * default/starting state.
 */
void qbman_eq_desc_clear(struct qbman_eq_desc *);

/* Exactly one of the following descriptor "actions" should be set. (Calling
 * any one of these will replace the effect of any prior call to one of these.)
 * - enqueue without order-restoration
 * - enqueue with order-restoration
 * - fill a hole in the order-restoration sequence, without any enqueue
 * - advance NESN (Next Expected Sequence Number), without any enqueue
 * 'respond_success' indicates whether an enqueue response should be DMA'd
 * after success (otherwise a response is DMA'd only after failure).
 * 'incomplete' indicates that other fragments of the same 'seqnum' are yet to
 * be enqueued.
 */
/**
 * qbman_eq_desc_set_no_orp() - Set enqueue descriptor without orp
 * @d: the enqueue descriptor.
 * @response_success: 1 = enqueue with response always; 0 = enqueue with
 * rejections returned on a FQ.
 */
void qbman_eq_desc_set_no_orp(struct qbman_eq_desc *d, int respond_success);

/**
 * qbman_eq_desc_set_orp() - Set order-resotration in the enqueue descriptor
 * @d: the enqueue descriptor.
 * @response_success: 1 = enqueue with response always; 0 = enqueue with
 * rejections returned on a FQ.
 * @opr_id: the order point record id.
 * @seqnum: the order restoration sequence number.
 * @incomplete: indiates whether this is the last fragments using the same
 * sequeue number.
 */
void qbman_eq_desc_set_orp(struct qbman_eq_desc *d, int respond_success,
			   uint32_t opr_id, uint32_t seqnum, int incomplete);

/**
 * qbman_eq_desc_set_orp_hole() - fill a hole in the order-restoration sequence
 * without any enqueue
 * @d: the enqueue descriptor.
 * @opr_id: the order point record id.
 * @seqnum: the order restoration sequence number.
 */
void qbman_eq_desc_set_orp_hole(struct qbman_eq_desc *d, uint32_t opr_id,
				uint32_t seqnum);

/**
 * qbman_eq_desc_set_orp_nesn() -  advance NESN (Next Expected Sequence Number)
 * without any enqueue
 * @d: the enqueue descriptor.
 * @opr_id: the order point record id.
 * @seqnum: the order restoration sequence number.
 */
void qbman_eq_desc_set_orp_nesn(struct qbman_eq_desc *d, uint32_t opr_id,
				uint32_t seqnum);

/**
 * qbman_eq_desc_set_response() - Set the enqueue response info.
 * @d: the enqueue descriptor
 * @storage_phys: the physical address of the enqueue response in memory.
 * @stash: indicate that the write allocation enabled or not.
 *
 * In the case where an enqueue response is DMA'd, this determines where that
 * response should go. (The physical/DMA address is given for hardware's
 * benefit, but software should interpret it as a "struct qbman_eq_response"
 * data structure.) 'stash' controls whether or not the write to main-memory
 * expresses a cache-warming attribute.
 */
void qbman_eq_desc_set_response(struct qbman_eq_desc *d,
				dma_addr_t storage_phys,
				int stash);
/**
 * qbman_eq_desc_set_token() - Set token for the enqueue command
 * @d: the enqueue descriptor
 * @token: the token to be set.
 *
 * token is the value that shows up in an enqueue response that can be used to
 * detect when the results have been published. The easiest technique is to zero
 * result "storage" before issuing an enqueue, and use any non-zero 'token'
 * value.
 */
void qbman_eq_desc_set_token(struct qbman_eq_desc *d, uint8_t token);

/**
 * qbman_eq_desc_set_fq()
 * qbman_eq_desc_set_qd() - Set eithe FQ or Queuing Destination for the enqueue
 * command.
 * @d: the enqueue descriptor
 * @fqid: the id of the frame queue to be enqueued.
 * @qdid: the id of the queuing destination to be enqueued.
 * @qd_bin: the queuing destination bin
 * @qd_prio: the queuing destination priority.
 *
 * Exactly one of the following descriptor "targets" should be set. (Calling any
 * one of these will replace the effect of any prior call to one of these.)
 * - enqueue to a frame queue
 * - enqueue to a queuing destination
 * Note, that none of these will have any affect if the "action" type has been
 * set to "orp_hole" or "orp_nesn".
 */
void qbman_eq_desc_set_fq(struct qbman_eq_desc *, uint32_t fqid);
void qbman_eq_desc_set_qd(struct qbman_eq_desc *, uint32_t qdid,
			  uint32_t qd_bin, uint32_t qd_prio);

/**
 * qbman_eq_desc_set_eqdi() - enable/disable EQDI interrupt
 * @d: the enqueue descriptor
 * @enable: boolean to enable/disable EQDI
 *
 * Determines whether or not the portal's EQDI interrupt source should be
 * asserted after the enqueue command is completed.
 */
void qbman_eq_desc_set_eqdi(struct qbman_eq_desc *, int enable);

/**
 * qbman_eq_desc_set_dca() - Set DCA mode in the enqueue command.
 * @d: the enqueue descriptor.
 * @enable: enabled/disable DCA mode.
 * @dqrr_idx: DCAP_CI, the DCAP consumer index.
 * @park: determine the whether park the FQ or not
 *
 * Determines whether or not a portal DQRR entry should be consumed once the
 * enqueue command is completed.  (And if so, and the DQRR entry corresponds
 * to a held-active (order-preserving) FQ, whether the FQ should be parked
 * instead of being rescheduled.)
 */
void qbman_eq_desc_set_dca(struct qbman_eq_desc *, int enable,
				uint32_t dqrr_idx, int park);

/**
 * qbman_swp_enqueue() - Issue an enqueue command.
 * @s: the software portal used for enqueue.
 * @d: the enqueue descriptor.
 * @fd: the frame descriptor to be enqueued.
 *
 * Please note that 'fd' should only be NULL if the "action" of the
 * descriptor is "orp_hole" or "orp_nesn".
 *
 * Return 0 for successful enqueue, -EBUSY if the EQCR is not ready.
 */
int qbman_swp_enqueue(struct qbman_swp *, const struct qbman_eq_desc *,
		      const struct qbman_fd *fd);

/**
 * qbman_swp_enqueue_thresh() - Set the threshold for EQRI interrupt.
 *
 * An EQRI interrupt can be generated when the fill-level of EQCR falls below
 * the 'thresh' value set here. Setting thresh==0 (the default) disables.
 */
int qbman_swp_enqueue_thresh(struct qbman_swp *, unsigned int thresh);

	/*******************/
	/* Buffer releases */
	/*******************/
/**
 * struct qbman_release_desc - The structure for buffer release descriptor
 */
struct qbman_release_desc {
	uint32_t dont_manipulate_directly[1];
};

/**
 * qbman_release_desc_clear() - Clear the contents of a descriptor to
 * default/starting state.
 */
void qbman_release_desc_clear(struct qbman_release_desc *);

/**
 * qbman_release_desc_set_bpid() - Set the ID of the buffer pool to release to
 */
void qbman_release_desc_set_bpid(struct qbman_release_desc *, uint32_t bpid);

/**
 * qbman_release_desc_set_rcdi() - Determines whether or not the portal's RCDI
 * interrupt source should be asserted after the release command is completed.
 */
void qbman_release_desc_set_rcdi(struct qbman_release_desc *, int enable);

/**
 * qbman_swp_release() - Issue a buffer release command.
 * @s: the software portal object.
 * @d: the release descriptor.
 * @buffers: a pointer pointing to the buffer address to be released.
 * @num_buffers: number of buffers to be released,  must be less than 8.
 *
 * Return 0 for success, -EBUSY if the release command ring is not ready.
 */
int qbman_swp_release(struct qbman_swp *s, const struct qbman_release_desc *d,
		      const uint64_t *buffers, unsigned int num_buffers);

	/*******************/
	/* Buffer acquires */
	/*******************/

/**
 * qbman_swp_acquire() - Issue a buffer acquire command.
 * @s: the software portal object.
 * @bpid: the buffer pool index.
 * @buffers: a pointer pointing to the acquired buffer address|es.
 * @num_buffers: number of buffers to be acquired, must be less than 8.
 *
 * Return 0 for success, or negative error code if the acquire command
 * fails.
 */
int qbman_swp_acquire(struct qbman_swp *, uint32_t bpid, uint64_t *buffers,
		      unsigned int num_buffers);

	/*****************/
	/* FQ management */
	/*****************/

/**
 * qbman_swp_fq_schedule() - Move the fq to the scheduled state.
 * @s: the software portal object.
 * @fqid: the index of frame queue to be scheduled.
 *
 * There are a couple of different ways that a FQ can end up parked state,
 * This schedules it.
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_fq_schedule(struct qbman_swp *s, uint32_t fqid);

/**
 * qbman_swp_fq_force() - Force the FQ to fully scheduled state.
 * @s: the software portal object.
 * @fqid: the index of frame queue to be forced.
 *
 * Force eligible will force a tentatively-scheduled FQ to be fully-scheduled
 * and thus be available for selection by any channel-dequeuing behaviour (push
 * or pull). If the FQ is subsequently "dequeued" from the channel and is still
 * empty at the time this happens, the resulting dq_entry will have no FD.
 * (qbman_result_DQ_fd() will return NULL.)
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_fq_force(struct qbman_swp *s, uint32_t fqid);

/**
 * qbman_swp_fq_xon()
 * qbman_swp_fq_xoff() - XON/XOFF the frame queue.
 * @s: the software portal object.
 * @fqid: the index of frame queue.
 *
 * These functions change the FQ flow-control stuff between XON/XOFF. (The
 * default is XON.) This setting doesn't affect enqueues to the FQ, just
 * dequeues. XOFF FQs will remain in the tenatively-scheduled state, even when
 * non-empty, meaning they won't be selected for scheduled dequeuing. If a FQ is
 * changed to XOFF after it had already become truly-scheduled to a channel, and
 * a pull dequeue of that channel occurs that selects that FQ for dequeuing,
 * then the resulting dq_entry will have no FD. (qbman_result_DQ_fd() will
 * return NULL.)
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_fq_xon(struct qbman_swp *s, uint32_t fqid);
int qbman_swp_fq_xoff(struct qbman_swp *s, uint32_t fqid);

	/**********************/
	/* Channel management */
	/**********************/

/* If the user has been allocated a channel object that is going to generate
 * CDANs to another channel, then these functions will be necessary.
 * CDAN-enabled channels only generate a single CDAN notification, after which
 * it they need to be reenabled before they'll generate another. (The idea is
 * that pull dequeuing will occur in reaction to the CDAN, followed by a
 * reenable step.) Each function generates a distinct command to hardware, so a
 * combination function is provided if the user wishes to modify the "context"
 * (which shows up in each CDAN message) each time they reenable, as a single
 * command to hardware. */
/**
 * qbman_swp_CDAN_set_context() - Set CDAN context
 * @s: the software portal object.
 * @channelid: the channel index.
 * @ctx: the context to be set in CDAN.
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_CDAN_set_context(struct qbman_swp *, uint16_t channelid,
				uint64_t ctx);

/**
 * qbman_swp_CDAN_enable() - Enable CDAN for the channel.
 * @s: the software portal object.
 * @channelid: the index of the channel to generate CDAN.
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_CDAN_enable(struct qbman_swp *, uint16_t channelid);

/**
 * qbman_swp_CDAN_disable() - disable CDAN for the channel.
 * @s: the software portal object.
 * @channelid: the index of the channel to generate CDAN.
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_CDAN_disable(struct qbman_swp *, uint16_t channelid);

/**
 * qbman_swp_CDAN_set_context_enable() - Set CDAN contest and enable CDAN
 * @s: the software portal object.
 * @channelid: the index of the channel to generate CDAN.
 * @ctx: the context set in CDAN.
 *
 * Return 0 for success, or negative error code for failure.
 */
int qbman_swp_CDAN_set_context_enable(struct qbman_swp *, uint16_t channelid,
				      uint64_t ctx);

#endif /* !_FSL_QBMAN_PORTAL_H */
