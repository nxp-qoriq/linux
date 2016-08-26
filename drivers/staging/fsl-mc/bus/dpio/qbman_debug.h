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

struct qbman_attr {
	uint32_t dont_manipulate_directly[40];
};

/* Buffer pool query commands */
int qbman_bp_query(struct qbman_swp *s, uint32_t bpid,
		   struct qbman_attr *a);
void qbman_bp_attr_get_bdi(struct qbman_attr *a, int *bdi, int *va, int *wae);
void qbman_bp_attr_get_swdet(struct qbman_attr *a, uint32_t *swdet);
void qbman_bp_attr_get_swdxt(struct qbman_attr *a, uint32_t *swdxt);
void qbman_bp_attr_get_hwdet(struct qbman_attr *a, uint32_t *hwdet);
void qbman_bp_attr_get_hwdxt(struct qbman_attr *a, uint32_t *hwdxt);
void qbman_bp_attr_get_swset(struct qbman_attr *a, uint32_t *swset);
void qbman_bp_attr_get_swsxt(struct qbman_attr *a, uint32_t *swsxt);
void qbman_bp_attr_get_vbpid(struct qbman_attr *a, uint32_t *vbpid);
void qbman_bp_attr_get_icid(struct qbman_attr *a, uint32_t *icid, int *pl);
void qbman_bp_attr_get_bpscn_addr(struct qbman_attr *a, uint64_t *bpscn_addr);
void qbman_bp_attr_get_bpscn_ctx(struct qbman_attr *a, uint64_t *bpscn_ctx);
void qbman_bp_attr_get_hw_targ(struct qbman_attr *a, uint32_t *hw_targ);
int qbman_bp_info_has_free_bufs(struct qbman_attr *a);
int qbman_bp_info_is_depleted(struct qbman_attr *a);
int qbman_bp_info_is_surplus(struct qbman_attr *a);
uint32_t qbman_bp_info_num_free_bufs(struct qbman_attr *a);
uint32_t qbman_bp_info_hdptr(struct qbman_attr *a);
uint32_t qbman_bp_info_sdcnt(struct qbman_attr *a);
uint32_t qbman_bp_info_hdcnt(struct qbman_attr *a);
uint32_t qbman_bp_info_sscnt(struct qbman_attr *a);

/* FQ query function for programmable fields */
int qbman_fq_query(struct qbman_swp *s, uint32_t fqid,
		   struct qbman_attr *desc);
void qbman_fq_attr_get_fqctrl(struct qbman_attr *d, uint32_t *fqctrl);
void qbman_fq_attr_get_cgrid(struct qbman_attr *d, uint32_t *cgrid);
void qbman_fq_attr_get_destwq(struct qbman_attr *d, uint32_t *destwq);
void qbman_fq_attr_get_icscred(struct qbman_attr *d, uint32_t *icscred);
void qbman_fq_attr_get_tdthresh(struct qbman_attr *d, uint32_t *tdthresh);
void qbman_fq_attr_get_oa(struct qbman_attr *d,
			  int *oa_ics, int *oa_cgr, int32_t *oa_len);
void qbman_fq_attr_get_mctl(struct qbman_attr *d,
			    int *bdi, int *ff, int *va, int *ps);
void qbman_fq_attr_get_ctx(struct qbman_attr *d, uint32_t *hi, uint32_t *lo);
void qbman_fq_attr_get_icid(struct qbman_attr *d, uint32_t *icid, int *pl);
void qbman_fq_attr_get_vfqid(struct qbman_attr *d, uint32_t *vfqid);
void qbman_fq_attr_get_erfqid(struct qbman_attr *d, uint32_t *erfqid);

/* FQ query command for non-programmable fields*/
enum qbman_fq_schedstate_e {
	qbman_fq_schedstate_oos = 0,
	qbman_fq_schedstate_retired,
	qbman_fq_schedstate_tentatively_scheduled,
	qbman_fq_schedstate_truly_scheduled,
	qbman_fq_schedstate_parked,
	qbman_fq_schedstate_held_active,
};

int qbman_fq_query_state(struct qbman_swp *s, uint32_t fqid,
			 struct qbman_attr *state);
uint32_t qbman_fq_state_schedstate(const struct qbman_attr *state);
int qbman_fq_state_force_eligible(const struct qbman_attr *state);
int qbman_fq_state_xoff(const struct qbman_attr *state);
int qbman_fq_state_retirement_pending(const struct qbman_attr *state);
int qbman_fq_state_overflow_error(const struct qbman_attr *state);
uint32_t qbman_fq_state_frame_count(const struct qbman_attr *state);
uint32_t qbman_fq_state_byte_count(const struct qbman_attr *state);

/* CGR query */
int qbman_cgr_query(struct qbman_swp *s, uint32_t cgid,
		    struct qbman_attr *attr);
void qbman_cgr_attr_get_ctl1(struct qbman_attr *d, int *cscn_wq_en_enter,
			     int *cscn_wq_en_exit, int *cscn_wq_icd);
void qbman_cgr_attr_get_mode(struct qbman_attr *d, uint32_t *mode,
			     int *rej_cnt_mode, int *cscn_bdi);
void qbman_cgr_attr_get_ctl2(struct qbman_attr *d, int *cscn_wr_en_enter,
			     int *cscn_wr_en_exit, int *cg_wr_ae,
			     int *cscn_dcp_en, int *cg_wr_va);
void qbman_cgr_attr_get_iwc(struct qbman_attr *d, int *i_cnt_wr_en,
			    uint32_t *i_cnt_wr_bnd);
void qbman_cgr_attr_get_tdc(struct qbman_attr *d, int *td_en);
void qbman_cgr_attr_get_cs_thres(struct qbman_attr *d, uint32_t *cs_thres);
void qbman_cgr_attr_get_cs_thres_x(struct qbman_attr *d,
				   uint32_t *cs_thres_x);
void qbman_cgr_attr_get_td_thres(struct qbman_attr *d, uint32_t *td_thres);
void qbman_cgr_attr_get_cscn_tdcp(struct qbman_attr *d, uint32_t *cscn_tdcp);
void qbman_cgr_attr_get_cscn_wqid(struct qbman_attr *d, uint32_t *cscn_wqid);
void qbman_cgr_attr_get_cscn_vcgid(struct qbman_attr *d,
				   uint32_t *cscn_vcgid);
void qbman_cgr_attr_get_cg_icid(struct qbman_attr *d, uint32_t *icid,
				int *pl);
void qbman_cgr_attr_get_cg_wr_addr(struct qbman_attr *d,
				   uint64_t *cg_wr_addr);
void qbman_cgr_attr_get_cscn_ctx(struct qbman_attr *d, uint64_t *cscn_ctx);
void qbman_cgr_attr_wred_get_edp(struct qbman_attr *d, uint32_t idx,
				 int *edp);
void qbman_cgr_attr_wred_dp_decompose(uint32_t dp, uint64_t *minth,
				      uint64_t *maxth, uint8_t *maxp);
void qbman_cgr_attr_wred_get_parm_dp(struct qbman_attr *d, uint32_t idx,
				     uint32_t *dp);

/* CGR/CCGR/CQ statistics query */
int qbman_cgr_reject_statistics(struct qbman_swp *s, uint32_t cgid, int clear,
				uint64_t *frame_cnt, uint64_t *byte_cnt);
int qbman_ccgr_reject_statistics(struct qbman_swp *s, uint32_t cgid, int clear,
				 uint64_t *frame_cnt, uint64_t *byte_cnt);
int qbman_cq_dequeue_statistics(struct qbman_swp *s, uint32_t cgid, int clear,
				uint64_t *frame_cnt, uint64_t *byte_cnt);
