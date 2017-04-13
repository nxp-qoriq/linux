/* Copyright (C) 2015 Freescale Semiconductor, Inc.
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

#include <linux/errno.h>

#include "../../include/dpaa2-global.h"
#include "qbman-portal.h"
#include "qbman_debug.h"

/* QBMan portal management command code */
#define QBMAN_BP_QUERY            0x32
#define QBMAN_FQ_QUERY            0x44
#define QBMAN_FQ_QUERY_NP         0x45
#define QBMAN_CGR_QUERY           0x51
#define QBMAN_WRED_QUERY          0x54
#define QBMAN_CGR_STAT_QUERY      0x55
#define QBMAN_CGR_STAT_QUERY_CLR  0x56

enum qbman_attr_usage_e {
	qbman_attr_usage_fq,
	qbman_attr_usage_bpool,
	qbman_attr_usage_cgr,
};

struct int_qbman_attr {
	u32 words[32];
	enum qbman_attr_usage_e usage;
};

#define attr_type_set(a, e) \
{ \
	struct qbman_attr *__attr = a; \
	enum qbman_attr_usage_e __usage = e; \
	((struct int_qbman_attr *)__attr)->usage = __usage; \
}

#define ATTR32(d) (&(d)->dont_manipulate_directly[0])
#define ATTR32_1(d) (&(d)->dont_manipulate_directly[16])

static struct qb_attr_code code_bp_bpid = QB_CODE(0, 16, 16);
static struct qb_attr_code code_bp_bdi = QB_CODE(1, 16, 1);
static struct qb_attr_code code_bp_va = QB_CODE(1, 17, 1);
static struct qb_attr_code code_bp_wae = QB_CODE(1, 18, 1);
static struct qb_attr_code code_bp_swdet = QB_CODE(4, 0, 16);
static struct qb_attr_code code_bp_swdxt = QB_CODE(4, 16, 16);
static struct qb_attr_code code_bp_hwdet = QB_CODE(5, 0, 16);
static struct qb_attr_code code_bp_hwdxt = QB_CODE(5, 16, 16);
static struct qb_attr_code code_bp_swset = QB_CODE(6, 0, 16);
static struct qb_attr_code code_bp_swsxt = QB_CODE(6, 16, 16);
static struct qb_attr_code code_bp_vbpid = QB_CODE(7, 0, 14);
static struct qb_attr_code code_bp_icid = QB_CODE(7, 16, 15);
static struct qb_attr_code code_bp_pl = QB_CODE(7, 31, 1);
static struct qb_attr_code code_bp_bpscn_addr_lo = QB_CODE(8, 0, 32);
static struct qb_attr_code code_bp_bpscn_addr_hi = QB_CODE(9, 0, 32);
static struct qb_attr_code code_bp_bpscn_ctx_lo = QB_CODE(10, 0, 32);
static struct qb_attr_code code_bp_bpscn_ctx_hi = QB_CODE(11, 0, 32);
static struct qb_attr_code code_bp_hw_targ = QB_CODE(12, 0, 16);
static struct qb_attr_code code_bp_state = QB_CODE(1, 24, 3);
static struct qb_attr_code code_bp_fill = QB_CODE(2, 0, 32);
static struct qb_attr_code code_bp_hdptr = QB_CODE(3, 0, 32);
static struct qb_attr_code code_bp_sdcnt = QB_CODE(13, 0, 8);
static struct qb_attr_code code_bp_hdcnt = QB_CODE(13, 1, 8);
static struct qb_attr_code code_bp_sscnt = QB_CODE(13, 2, 8);

void qbman_bp_attr_clear(struct qbman_attr *a)
{
	memset(a, 0, sizeof(*a));
	attr_type_set(a, qbman_attr_usage_bpool);
}

int qbman_bp_query(struct qbman_swp *s, u32 bpid,
		   struct qbman_attr *a)
{
	u32 *p;
	u32 verb, rslt;
	u32 *attr = ATTR32(a);

	qbman_bp_attr_clear(a);

	/* Start the management command */
	p = qbman_swp_mc_start(s);
	if (!p)
		return -EBUSY;

	/* Encode the caller-provided attributes */
	qb_attr_code_encode(&code_bp_bpid, p, bpid);

	/* Complete the management command */
	p = qbman_swp_mc_complete(s, p, QBMAN_BP_QUERY);

	/* Decode the outcome */
	verb = qb_attr_code_decode(&code_generic_verb, p);
	rslt = qb_attr_code_decode(&code_generic_rslt, p);
	WARN_ON(verb != QBMAN_BP_QUERY);

	/* Determine success or failure */
	if (unlikely(rslt != QBMAN_MC_RSLT_OK)) {
		pr_err("Query of BPID 0x%x failed, code=0x%02x\n", bpid, rslt);
		return -EIO;
	}

	/* For the query, word[0] of the result contains only the
	 * verb/rslt fields, so skip word[0].
	 */
	word_copy(&attr[1], &p[1], 15);
	return 0;
}

void qbman_bp_attr_get_bdi(struct qbman_attr *a, int *bdi, int *va, int *wae)
{
	u32 *p = ATTR32(a);

	*bdi = !!qb_attr_code_decode(&code_bp_bdi, p);
	*va = !!qb_attr_code_decode(&code_bp_va, p);
	*wae = !!qb_attr_code_decode(&code_bp_wae, p);
}

static u32 qbman_bp_thresh_to_value(u32 val)
{
	return (val & 0xff) << ((val & 0xf00) >> 8);
}

void qbman_bp_attr_get_swdet(struct qbman_attr *a, u32 *swdet)
{
	u32 *p = ATTR32(a);

	*swdet = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_swdet,
					  p));
}

void qbman_bp_attr_get_swdxt(struct qbman_attr *a, u32 *swdxt)
{
	u32 *p = ATTR32(a);

	*swdxt = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_swdxt,
					  p));
}

void qbman_bp_attr_get_hwdet(struct qbman_attr *a, u32 *hwdet)
{
	u32 *p = ATTR32(a);

	*hwdet = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_hwdet,
					  p));
}

void qbman_bp_attr_get_hwdxt(struct qbman_attr *a, u32 *hwdxt)
{
	u32 *p = ATTR32(a);

	*hwdxt = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_hwdxt,
					  p));
}

void qbman_bp_attr_get_swset(struct qbman_attr *a, u32 *swset)
{
	u32 *p = ATTR32(a);

	*swset = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_swset,
					  p));
}

void qbman_bp_attr_get_swsxt(struct qbman_attr *a, u32 *swsxt)
{
	u32 *p = ATTR32(a);

	*swsxt = qbman_bp_thresh_to_value(qb_attr_code_decode(&code_bp_swsxt,
					  p));
}

void qbman_bp_attr_get_vbpid(struct qbman_attr *a, u32 *vbpid)
{
	u32 *p = ATTR32(a);

	*vbpid = qb_attr_code_decode(&code_bp_vbpid, p);
}

void qbman_bp_attr_get_icid(struct qbman_attr *a, u32 *icid, int *pl)
{
	u32 *p = ATTR32(a);

	*icid = qb_attr_code_decode(&code_bp_icid, p);
	*pl = !!qb_attr_code_decode(&code_bp_pl, p);
}

void qbman_bp_attr_get_bpscn_addr(struct qbman_attr *a, u64 *bpscn_addr)
{
	u32 *p = ATTR32(a);

	*bpscn_addr = ((u64)qb_attr_code_decode(&code_bp_bpscn_addr_hi,
			p) << 32) |
			(u64)qb_attr_code_decode(&code_bp_bpscn_addr_lo,
			p);
}

void qbman_bp_attr_get_bpscn_ctx(struct qbman_attr *a, u64 *bpscn_ctx)
{
	u32 *p = ATTR32(a);

	*bpscn_ctx = ((u64)qb_attr_code_decode(&code_bp_bpscn_ctx_hi, p)
			<< 32) |
			(u64)qb_attr_code_decode(&code_bp_bpscn_ctx_lo,
			p);
}

void qbman_bp_attr_get_hw_targ(struct qbman_attr *a, u32 *hw_targ)
{
	u32 *p = ATTR32(a);

	*hw_targ = qb_attr_code_decode(&code_bp_hw_targ, p);
}

int qbman_bp_info_has_free_bufs(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return !(int)(qb_attr_code_decode(&code_bp_state, p) & 0x1);
}

int qbman_bp_info_is_depleted(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return (int)(qb_attr_code_decode(&code_bp_state, p) & 0x2);
}

int qbman_bp_info_is_surplus(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return (int)(qb_attr_code_decode(&code_bp_state, p) & 0x4);
}

u32 qbman_bp_info_num_free_bufs(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return qb_attr_code_decode(&code_bp_fill, p);
}

u32 qbman_bp_info_hdptr(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return qb_attr_code_decode(&code_bp_hdptr, p);
}

u32 qbman_bp_info_sdcnt(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return qb_attr_code_decode(&code_bp_sdcnt, p);
}

u32 qbman_bp_info_hdcnt(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return qb_attr_code_decode(&code_bp_hdcnt, p);
}

u32 qbman_bp_info_sscnt(struct qbman_attr *a)
{
	u32 *p = ATTR32(a);

	return qb_attr_code_decode(&code_bp_sscnt, p);
}

static struct qb_attr_code code_fq_fqid = QB_CODE(1, 0, 24);
static struct qb_attr_code code_fq_cgrid = QB_CODE(2, 16, 16);
static struct qb_attr_code code_fq_destwq = QB_CODE(3, 0, 15);
static struct qb_attr_code code_fq_fqctrl = QB_CODE(3, 24, 8);
static struct qb_attr_code code_fq_icscred = QB_CODE(4, 0, 15);
static struct qb_attr_code code_fq_tdthresh = QB_CODE(4, 16, 13);
static struct qb_attr_code code_fq_oa_len = QB_CODE(5, 0, 12);
static struct qb_attr_code code_fq_oa_ics = QB_CODE(5, 14, 1);
static struct qb_attr_code code_fq_oa_cgr = QB_CODE(5, 15, 1);
static struct qb_attr_code code_fq_mctl_bdi = QB_CODE(5, 24, 1);
static struct qb_attr_code code_fq_mctl_ff = QB_CODE(5, 25, 1);
static struct qb_attr_code code_fq_mctl_va = QB_CODE(5, 26, 1);
static struct qb_attr_code code_fq_mctl_ps = QB_CODE(5, 27, 1);
static struct qb_attr_code code_fq_ctx_lower32 = QB_CODE(6, 0, 32);
static struct qb_attr_code code_fq_ctx_upper32 = QB_CODE(7, 0, 32);
static struct qb_attr_code code_fq_icid = QB_CODE(8, 0, 15);
static struct qb_attr_code code_fq_pl = QB_CODE(8, 15, 1);
static struct qb_attr_code code_fq_vfqid = QB_CODE(9, 0, 24);
static struct qb_attr_code code_fq_erfqid = QB_CODE(10, 0, 24);

void qbman_fq_attr_clear(struct qbman_attr *a)
{
	memset(a, 0, sizeof(*a));
	attr_type_set(a, qbman_attr_usage_fq);
}

/* FQ query function for programmable fields */
int qbman_fq_query(struct qbman_swp *s, u32 fqid, struct qbman_attr *desc)
{
	u32 *p;
	u32 verb, rslt;
	u32 *d = ATTR32(desc);

	qbman_fq_attr_clear(desc);

	p = qbman_swp_mc_start(s);
	if (!p)
		return -EBUSY;
	qb_attr_code_encode(&code_fq_fqid, p, fqid);
	p = qbman_swp_mc_complete(s, p, QBMAN_FQ_QUERY);

	/* Decode the outcome */
	verb = qb_attr_code_decode(&code_generic_verb, p);
	rslt = qb_attr_code_decode(&code_generic_rslt, p);
	WARN_ON(verb != QBMAN_FQ_QUERY);

	/* Determine success or failure */
	if (unlikely(rslt != QBMAN_MC_RSLT_OK)) {
		pr_err("Query of FQID 0x%x failed, code=0x%02x\n",
		       fqid, rslt);
		return -EIO;
	}
	/*
	 * For the configure, word[0] of the command contains only the WE-mask.
	 * For the query, word[0] of the result contains only the verb/rslt
	 * fields. Skip word[0] in the latter case.
	 */
	word_copy(&d[1], &p[1], 15);
	return 0;
}

void qbman_fq_attr_get_fqctrl(struct qbman_attr *d, u32 *fqctrl)
{
	u32 *p = ATTR32(d);

	*fqctrl = qb_attr_code_decode(&code_fq_fqctrl, p);
}

void qbman_fq_attr_get_cgrid(struct qbman_attr *d, u32 *cgrid)
{
	u32 *p = ATTR32(d);

	*cgrid = qb_attr_code_decode(&code_fq_cgrid, p);
}

void qbman_fq_attr_get_destwq(struct qbman_attr *d, u32 *destwq)
{
	u32 *p = ATTR32(d);

	*destwq = qb_attr_code_decode(&code_fq_destwq, p);
}

void qbman_fq_attr_get_icscred(struct qbman_attr *d, u32 *icscred)
{
	u32 *p = ATTR32(d);

	*icscred = qb_attr_code_decode(&code_fq_icscred, p);
}

static struct qb_attr_code code_tdthresh_exp = QB_CODE(0, 0, 5);
static struct qb_attr_code code_tdthresh_mant = QB_CODE(0, 5, 8);
static u32 qbman_thresh_to_value(u32 val)
{
	u32 m, e;

	m = qb_attr_code_decode(&code_tdthresh_mant, &val);
	e = qb_attr_code_decode(&code_tdthresh_exp, &val);
	return m << e;
}

void qbman_fq_attr_get_tdthresh(struct qbman_attr *d, u32 *tdthresh)
{
	u32 *p = ATTR32(d);

	*tdthresh = qbman_thresh_to_value(qb_attr_code_decode(&code_fq_tdthresh,
					p));
}

void qbman_fq_attr_get_oa(struct qbman_attr *d,
			  int *oa_ics, int *oa_cgr, int32_t *oa_len)
{
	u32 *p = ATTR32(d);

	*oa_ics = !!qb_attr_code_decode(&code_fq_oa_ics, p);
	*oa_cgr = !!qb_attr_code_decode(&code_fq_oa_cgr, p);
	*oa_len = qb_attr_code_makesigned(&code_fq_oa_len,
			qb_attr_code_decode(&code_fq_oa_len, p));
}

void qbman_fq_attr_get_mctl(struct qbman_attr *d,
			    int *bdi, int *ff, int *va, int *ps)
{
	u32 *p = ATTR32(d);

	*bdi = !!qb_attr_code_decode(&code_fq_mctl_bdi, p);
	*ff = !!qb_attr_code_decode(&code_fq_mctl_ff, p);
	*va = !!qb_attr_code_decode(&code_fq_mctl_va, p);
	*ps = !!qb_attr_code_decode(&code_fq_mctl_ps, p);
}

void qbman_fq_attr_get_ctx(struct qbman_attr *d, u32 *hi, u32 *lo)
{
	u32 *p = ATTR32(d);

	*hi = qb_attr_code_decode(&code_fq_ctx_upper32, p);
	*lo = qb_attr_code_decode(&code_fq_ctx_lower32, p);
}

void qbman_fq_attr_get_icid(struct qbman_attr *d, u32 *icid, int *pl)
{
	u32 *p = ATTR32(d);

	*icid = qb_attr_code_decode(&code_fq_icid, p);
	*pl = !!qb_attr_code_decode(&code_fq_pl, p);
}

void qbman_fq_attr_get_vfqid(struct qbman_attr *d, u32 *vfqid)
{
	u32 *p = ATTR32(d);

	*vfqid = qb_attr_code_decode(&code_fq_vfqid, p);
}

void qbman_fq_attr_get_erfqid(struct qbman_attr *d, u32 *erfqid)
{
	u32 *p = ATTR32(d);

	*erfqid = qb_attr_code_decode(&code_fq_erfqid, p);
}

/* Query FQ Non-Programmalbe Fields */
static struct qb_attr_code code_fq_np_state = QB_CODE(0, 16, 3);
static struct qb_attr_code code_fq_np_fe = QB_CODE(0, 19, 1);
static struct qb_attr_code code_fq_np_x = QB_CODE(0, 20, 1);
static struct qb_attr_code code_fq_np_r = QB_CODE(0, 21, 1);
static struct qb_attr_code code_fq_np_oe = QB_CODE(0, 22, 1);
static struct qb_attr_code code_fq_np_frm_cnt = QB_CODE(6, 0, 24);
static struct qb_attr_code code_fq_np_byte_cnt = QB_CODE(7, 0, 32);

int qbman_fq_query_state(struct qbman_swp *s, u32 fqid,
			 struct qbman_attr *state)
{
	u32 *p;
	u32 verb, rslt;
	u32 *d = ATTR32(state);

	qbman_fq_attr_clear(state);

	p = qbman_swp_mc_start(s);
	if (!p)
		return -EBUSY;
	qb_attr_code_encode(&code_fq_fqid, p, fqid);
	p = qbman_swp_mc_complete(s, p, QBMAN_FQ_QUERY_NP);

	/* Decode the outcome */
	verb = qb_attr_code_decode(&code_generic_verb, p);
	rslt = qb_attr_code_decode(&code_generic_rslt, p);
	WARN_ON(verb != QBMAN_FQ_QUERY_NP);

	/* Determine success or failure */
	if (unlikely(rslt != QBMAN_MC_RSLT_OK)) {
		pr_err("Query NP fields of FQID 0x%x failed, code=0x%02x\n",
		       fqid, rslt);
		return -EIO;
	}
	word_copy(&d[0], &p[0], 16);
	return 0;
}

u32 qbman_fq_state_schedstate(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return qb_attr_code_decode(&code_fq_np_state, p);
}

int qbman_fq_state_force_eligible(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return !!qb_attr_code_decode(&code_fq_np_fe, p);
}

int qbman_fq_state_xoff(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return !!qb_attr_code_decode(&code_fq_np_x, p);
}

int qbman_fq_state_retirement_pending(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return !!qb_attr_code_decode(&code_fq_np_r, p);
}

int qbman_fq_state_overflow_error(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return !!qb_attr_code_decode(&code_fq_np_oe, p);
}

u32 qbman_fq_state_frame_count(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return qb_attr_code_decode(&code_fq_np_frm_cnt, p);
}

u32 qbman_fq_state_byte_count(const struct qbman_attr *state)
{
	const u32 *p = ATTR32(state);

	return qb_attr_code_decode(&code_fq_np_byte_cnt, p);
}

/* Query CGR */
static struct qb_attr_code code_cgr_cgid = QB_CODE(0, 16, 16);
static struct qb_attr_code code_cgr_cscn_wq_en_enter = QB_CODE(2, 0, 1);
static struct qb_attr_code code_cgr_cscn_wq_en_exit = QB_CODE(2, 1, 1);
static struct qb_attr_code code_cgr_cscn_wq_icd = QB_CODE(2, 2, 1);
static struct qb_attr_code code_cgr_mode = QB_CODE(3, 16, 2);
static struct qb_attr_code code_cgr_rej_cnt_mode = QB_CODE(3, 18, 1);
static struct qb_attr_code code_cgr_cscn_bdi = QB_CODE(3, 19, 1);
static struct qb_attr_code code_cgr_cscn_wr_en_enter = QB_CODE(3, 24, 1);
static struct qb_attr_code code_cgr_cscn_wr_en_exit = QB_CODE(3, 25, 1);
static struct qb_attr_code code_cgr_cg_wr_ae = QB_CODE(3, 26, 1);
static struct qb_attr_code code_cgr_cscn_dcp_en = QB_CODE(3, 27, 1);
static struct qb_attr_code code_cgr_cg_wr_va = QB_CODE(3, 28, 1);
static struct qb_attr_code code_cgr_i_cnt_wr_en = QB_CODE(4, 0, 1);
static struct qb_attr_code code_cgr_i_cnt_wr_bnd = QB_CODE(4, 1, 5);
static struct qb_attr_code code_cgr_td_en = QB_CODE(4, 8, 1);
static struct qb_attr_code code_cgr_cs_thres = QB_CODE(4, 16, 13);
static struct qb_attr_code code_cgr_cs_thres_x = QB_CODE(5, 0, 13);
static struct qb_attr_code code_cgr_td_thres = QB_CODE(5, 16, 13);
static struct qb_attr_code code_cgr_cscn_tdcp = QB_CODE(6, 0, 16);
static struct qb_attr_code code_cgr_cscn_wqid = QB_CODE(6, 16, 16);
static struct qb_attr_code code_cgr_cscn_vcgid = QB_CODE(7, 0, 16);
static struct qb_attr_code code_cgr_cg_icid = QB_CODE(7, 16, 15);
static struct qb_attr_code code_cgr_cg_pl = QB_CODE(7, 31, 1);
static struct qb_attr_code code_cgr_cg_wr_addr_lo = QB_CODE(8, 0, 32);
static struct qb_attr_code code_cgr_cg_wr_addr_hi = QB_CODE(9, 0, 32);
static struct qb_attr_code code_cgr_cscn_ctx_lo = QB_CODE(10, 0, 32);
static struct qb_attr_code code_cgr_cscn_ctx_hi = QB_CODE(11, 0, 32);

void qbman_cgr_attr_clear(struct qbman_attr *a)
{
	memset(a, 0, sizeof(*a));
	attr_type_set(a, qbman_attr_usage_cgr);
}

int qbman_cgr_query(struct qbman_swp *s, u32 cgid, struct qbman_attr *attr)
{
	u32 *p;
	u32 verb, rslt;
	u32 *d[2];
	int i;
	u32 query_verb;

	d[0] = ATTR32(attr);
	d[1] = ATTR32_1(attr);

	qbman_cgr_attr_clear(attr);

	for (i = 0; i < 2; i++) {
		p = qbman_swp_mc_start(s);
		if (!p)
			return -EBUSY;
		query_verb = i ? QBMAN_WRED_QUERY : QBMAN_CGR_QUERY;

		qb_attr_code_encode(&code_cgr_cgid, p, cgid);
		p = qbman_swp_mc_complete(s, p, p[0] | query_verb);

		/* Decode the outcome */
		verb = qb_attr_code_decode(&code_generic_verb, p);
		rslt = qb_attr_code_decode(&code_generic_rslt, p);
		WARN_ON(verb != query_verb);

		/* Determine success or failure */
		if (unlikely(rslt != QBMAN_MC_RSLT_OK)) {
			pr_err("Query CGID 0x%x failed,", cgid);
			pr_err(" verb=0x%02x, code=0x%02x\n", verb, rslt);
			return -EIO;
		}
		/* For the configure, word[0] of the command contains only the
		 * verb/cgid. For the query, word[0] of the result contains
		 * only the verb/rslt fields. Skip word[0] in the latter case.
		 */
		word_copy(&d[i][1], &p[1], 15);
	}
	return 0;
}

void qbman_cgr_attr_get_ctl1(struct qbman_attr *d, int *cscn_wq_en_enter,
			     int *cscn_wq_en_exit, int *cscn_wq_icd)
	{
	u32 *p = ATTR32(d);
	*cscn_wq_en_enter = !!qb_attr_code_decode(&code_cgr_cscn_wq_en_enter,
									 p);
	*cscn_wq_en_exit = !!qb_attr_code_decode(&code_cgr_cscn_wq_en_exit, p);
	*cscn_wq_icd = !!qb_attr_code_decode(&code_cgr_cscn_wq_icd, p);
}

void qbman_cgr_attr_get_mode(struct qbman_attr *d, u32 *mode,
			     int *rej_cnt_mode, int *cscn_bdi)
{
	u32 *p = ATTR32(d);
	*mode = qb_attr_code_decode(&code_cgr_mode, p);
	*rej_cnt_mode = !!qb_attr_code_decode(&code_cgr_rej_cnt_mode, p);
	*cscn_bdi = !!qb_attr_code_decode(&code_cgr_cscn_bdi, p);
}

void qbman_cgr_attr_get_ctl2(struct qbman_attr *d, int *cscn_wr_en_enter,
			     int *cscn_wr_en_exit, int *cg_wr_ae,
			     int *cscn_dcp_en, int *cg_wr_va)
{
	u32 *p = ATTR32(d);
	*cscn_wr_en_enter = !!qb_attr_code_decode(&code_cgr_cscn_wr_en_enter,
									p);
	*cscn_wr_en_exit = !!qb_attr_code_decode(&code_cgr_cscn_wr_en_exit, p);
	*cg_wr_ae = !!qb_attr_code_decode(&code_cgr_cg_wr_ae, p);
	*cscn_dcp_en = !!qb_attr_code_decode(&code_cgr_cscn_dcp_en, p);
	*cg_wr_va = !!qb_attr_code_decode(&code_cgr_cg_wr_va, p);
}

void qbman_cgr_attr_get_iwc(struct qbman_attr *d, int *i_cnt_wr_en,
			    u32 *i_cnt_wr_bnd)
{
	u32 *p = ATTR32(d);
	*i_cnt_wr_en = !!qb_attr_code_decode(&code_cgr_i_cnt_wr_en, p);
	*i_cnt_wr_bnd = qb_attr_code_decode(&code_cgr_i_cnt_wr_bnd, p);
}

void qbman_cgr_attr_get_tdc(struct qbman_attr *d, int *td_en)
{
	u32 *p = ATTR32(d);
	*td_en = !!qb_attr_code_decode(&code_cgr_td_en, p);
}

void qbman_cgr_attr_get_cs_thres(struct qbman_attr *d, u32 *cs_thres)
{
	u32 *p = ATTR32(d);
	*cs_thres = qbman_thresh_to_value(qb_attr_code_decode(
						&code_cgr_cs_thres, p));
}

void qbman_cgr_attr_get_cs_thres_x(struct qbman_attr *d,
				   u32 *cs_thres_x)
{
	u32 *p = ATTR32(d);
	*cs_thres_x = qbman_thresh_to_value(qb_attr_code_decode(
					    &code_cgr_cs_thres_x, p));
}

void qbman_cgr_attr_get_td_thres(struct qbman_attr *d, u32 *td_thres)
{
	u32 *p = ATTR32(d);
	*td_thres = qbman_thresh_to_value(qb_attr_code_decode(
					  &code_cgr_td_thres, p));
}

void qbman_cgr_attr_get_cscn_tdcp(struct qbman_attr *d, u32 *cscn_tdcp)
{
	u32 *p = ATTR32(d);
	*cscn_tdcp = qb_attr_code_decode(&code_cgr_cscn_tdcp, p);
}

void qbman_cgr_attr_get_cscn_wqid(struct qbman_attr *d, u32 *cscn_wqid)
{
	u32 *p = ATTR32(d);
	*cscn_wqid = qb_attr_code_decode(&code_cgr_cscn_wqid, p);
}

void qbman_cgr_attr_get_cscn_vcgid(struct qbman_attr *d,
				   u32 *cscn_vcgid)
{
	u32 *p = ATTR32(d);
	*cscn_vcgid = qb_attr_code_decode(&code_cgr_cscn_vcgid, p);
}

void qbman_cgr_attr_get_cg_icid(struct qbman_attr *d, u32 *icid,
				int *pl)
{
	u32 *p = ATTR32(d);
	*icid = qb_attr_code_decode(&code_cgr_cg_icid, p);
	*pl = !!qb_attr_code_decode(&code_cgr_cg_pl, p);
}

void qbman_cgr_attr_get_cg_wr_addr(struct qbman_attr *d,
				   u64 *cg_wr_addr)
{
	u32 *p = ATTR32(d);
	*cg_wr_addr = ((u64)qb_attr_code_decode(&code_cgr_cg_wr_addr_hi,
			p) << 32) |
			(u64)qb_attr_code_decode(&code_cgr_cg_wr_addr_lo,
			p);
}

void qbman_cgr_attr_get_cscn_ctx(struct qbman_attr *d, u64 *cscn_ctx)
{
	u32 *p = ATTR32(d);
	*cscn_ctx = ((u64)qb_attr_code_decode(&code_cgr_cscn_ctx_hi, p)
			<< 32) |
			(u64)qb_attr_code_decode(&code_cgr_cscn_ctx_lo, p);
}

#define WRED_EDP_WORD(n) (18 + (n) / 4)
#define WRED_EDP_OFFSET(n) (8 * ((n) % 4))
#define WRED_PARM_DP_WORD(n) ((n) + 20)
#define WRED_WE_EDP(n) (16 + (n) * 2)
#define WRED_WE_PARM_DP(n) (17 + (n) * 2)
void qbman_cgr_attr_wred_get_edp(struct qbman_attr *d, u32 idx,
				 int *edp)
{
	u32 *p = ATTR32(d);
	struct qb_attr_code code_wred_edp = QB_CODE(WRED_EDP_WORD(idx),
						WRED_EDP_OFFSET(idx), 8);
	*edp = (int)qb_attr_code_decode(&code_wred_edp, p);
}

void qbman_cgr_attr_wred_dp_decompose(u32 dp, u64 *minth,
				      u64 *maxth, u8 *maxp)
{
	u8 ma, mn, step_i, step_s, pn;

	ma = (u8)(dp >> 24);
	mn = (u8)(dp >> 19) & 0x1f;
	step_i = (u8)(dp >> 11);
	step_s = (u8)(dp >> 6) & 0x1f;
	pn = (u8)dp & 0x3f;

	*maxp = ((pn << 2) * 100) / 256;

	if (mn == 0)
		*maxth = ma;
	else
		*maxth = ((ma + 256) * (1 << (mn - 1)));

	if (step_s == 0)
		*minth = *maxth - step_i;
	else
		*minth = *maxth - (256 + step_i) * (1 << (step_s - 1));
}

void qbman_cgr_attr_wred_get_parm_dp(struct qbman_attr *d, u32 idx,
				     u32 *dp)
{
	u32 *p = ATTR32(d);
	struct qb_attr_code code_wred_parm_dp = QB_CODE(WRED_PARM_DP_WORD(idx),
						0, 8);
	*dp = qb_attr_code_decode(&code_wred_parm_dp, p);
}

/* Query CGR/CCGR/CQ statistics */
static struct qb_attr_code code_cgr_stat_ct = QB_CODE(4, 0, 32);
static struct qb_attr_code code_cgr_stat_frame_cnt_lo = QB_CODE(4, 0, 32);
static struct qb_attr_code code_cgr_stat_frame_cnt_hi = QB_CODE(5, 0, 8);
static struct qb_attr_code code_cgr_stat_byte_cnt_lo = QB_CODE(6, 0, 32);
static struct qb_attr_code code_cgr_stat_byte_cnt_hi = QB_CODE(7, 0, 16);
static int qbman_cgr_statistics_query(struct qbman_swp *s, u32 cgid,
				      int clear, u32 command_type,
				      u64 *frame_cnt, u64 *byte_cnt)
{
	u32 *p;
	u32 verb, rslt;
	u32 query_verb;
	u32 hi, lo;

	p = qbman_swp_mc_start(s);
	if (!p)
		return -EBUSY;

	qb_attr_code_encode(&code_cgr_cgid, p, cgid);
	if (command_type < 2)
		qb_attr_code_encode(&code_cgr_stat_ct, p, command_type);
	query_verb = clear ?
			QBMAN_CGR_STAT_QUERY_CLR : QBMAN_CGR_STAT_QUERY;
	p = qbman_swp_mc_complete(s, p, p[0] | query_verb);

	/* Decode the outcome */
	verb = qb_attr_code_decode(&code_generic_verb, p);
	rslt = qb_attr_code_decode(&code_generic_rslt, p);
	WARN_ON(verb != query_verb);

	/* Determine success or failure */
	if (unlikely(rslt != QBMAN_MC_RSLT_OK)) {
		pr_err("Query statistics of CGID 0x%x failed,", cgid);
		pr_err(" verb=0x%02x code=0x%02x\n", verb, rslt);
		return -EIO;
	}

	if (*frame_cnt) {
		hi = qb_attr_code_decode(&code_cgr_stat_frame_cnt_hi, p);
		lo = qb_attr_code_decode(&code_cgr_stat_frame_cnt_lo, p);
		*frame_cnt = ((u64)hi << 32) | (u64)lo;
	}
	if (*byte_cnt) {
		hi = qb_attr_code_decode(&code_cgr_stat_byte_cnt_hi, p);
		lo = qb_attr_code_decode(&code_cgr_stat_byte_cnt_lo, p);
		*byte_cnt = ((u64)hi << 32) | (u64)lo;
	}

	return 0;
}

int qbman_cgr_reject_statistics(struct qbman_swp *s, u32 cgid, int clear,
				u64 *frame_cnt, u64 *byte_cnt)
{
	return qbman_cgr_statistics_query(s, cgid, clear, 0xff,
					  frame_cnt, byte_cnt);
}

int qbman_ccgr_reject_statistics(struct qbman_swp *s, u32 cgid, int clear,
				 u64 *frame_cnt, u64 *byte_cnt)
{
	return qbman_cgr_statistics_query(s, cgid, clear, 1,
					  frame_cnt, byte_cnt);
}

int qbman_cq_dequeue_statistics(struct qbman_swp *s, u32 cgid, int clear,
				u64 *frame_cnt, u64 *byte_cnt)
{
	return qbman_cgr_statistics_query(s, cgid, clear, 0,
					  frame_cnt, byte_cnt);
}
