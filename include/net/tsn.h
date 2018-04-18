/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the names of the above-listed copyright holders nor the
 *       names of any contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

#ifndef __TSN_H__
#define __TSN_H__

#include <uapi/linux/tsn.h>

struct tsn_ops {
	int (*get_capability)(struct net_device *, struct port_status *);
	/* Qbv standard */
	int (*qbv_set)(struct net_device *ndev, struct tsn_qbv_conf *qbvconf);
	int (*qbv_get)(struct net_device *ndev, struct tsn_qbv_conf *qbvconf);
	int (*qbv_get_status)(struct net_device *ndev,
							struct tsn_qbv_status *qbvstat);
	int (*cb_streamid_set)(struct net_device *ndev, u32 index,
							bool enable, struct tsn_cb_streamid *sid);
	int (*cb_streamid_get)(struct net_device *ndev, u32 index,
							struct tsn_cb_streamid *sid);
	int (*cb_streamid_counters_get)(struct net_device *ndev, u32 index,
							struct tsn_cb_streamid_counters *sidcounter);
	int (*qci_get_maxcap)(struct net_device *ndev,
							struct tsn_qci_psfp_stream_param *qcicapa);
	int (*qci_sfi_set)(struct net_device *ndev, u32 index, bool enable,
							struct tsn_qci_psfp_sfi_conf *sficonf);
	/* return: 0 stream filter instance not valid
	 * 1 stream filter instance valid
	 * -1 error happened
	 */
	int (*qci_sfi_get)(struct net_device *ndev, u32 index,
						struct tsn_qci_psfp_sfi_conf *sficonf);
	int (*qci_sfi_counters_get)(struct net_device *ndev, u32 index,
								struct tsn_qci_psfp_sfi_counters *sficounter);
	int (*qci_sgi_set)(struct net_device *ndev, u32 index,
							struct tsn_qci_psfp_sgi_conf *sgiconf);
	int (*qci_sgi_get)(struct net_device *ndev, u32 index,
							struct tsn_qci_psfp_sgi_conf *sgiconf);
	int (*qci_sgi_status_get)(struct net_device *ndev, u16 index,
							struct tsn_psfp_sgi_status *sgistat);
	int (*qci_fmi_set)(struct net_device *ndev, u32 index, bool enable,
							struct tsn_qci_psfp_fmi *fmi);
	int (*qci_fmi_get)(struct net_device *ndev, u32 index,
							struct tsn_qci_psfp_fmi *fmi);
	int (*cbs_set)(struct net_device *ndev, u8 tc, u8 bw);
	int (*cbs_get)(struct net_device *ndev, u8 tc);
	/* To set a 8 bits vector shows 8 traffic classes
	 * preemtable(1) or express(0)
	 */
	int (*qbu_set)(struct net_device *ndev, u8 ptvector);
	/* To get port preemtion status */
	int (*qbu_get)(struct net_device *ndev,
						struct tsn_preempt_status *preemptstat);
	int (*tsd_set)(struct net_device *, struct tsn_tsd *);
	int (*tsd_get)(struct net_device *, struct tsn_tsd_status *);
};

#endif
