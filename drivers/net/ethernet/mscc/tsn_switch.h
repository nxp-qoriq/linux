/* SPDX-License-Identifier: (GPL-2.0 OR MIT)
 *
 * TSN_SWITCH driver
 *
 * Copyright 2018 NXP
 */

#ifndef _MSCC_FELIX_SWITCH_TSN_H_
#define _MSCC_FELIX_SWITCH_TSN_H_

#define TRUE 1
#define FALSE 0
#define SUCCESS 1
#define FAILED 0

#define SWITCH_TAS_GCL_MAX 64
#define SWITCH_TAS_CT_MAX 1000000000
#define SWITCH_TAS_CT_MIN 100
#define SWITCH_TAS_CTE_MAX 999999999

int switch_qbv_set(struct net_device *ndev,
		   struct tsn_qbv_conf *shaper_config);
int switch_qbv_get(struct net_device *ndev,
		   struct tsn_qbv_conf *shaper_config);
int switch_qbv_get_status(struct net_device *ndev,
			  struct tsn_qbv_status *qbvstatus);
int switch_cut_thru_set(struct net_device *ndev, u8 cut_thru);
int switch_cbs_set(struct net_device *ndev, u8 tc, u8 bw);
int switch_qbu_set(struct net_device *ndev, u8 preemptable);
int switch_cb_streamid_get(struct net_device *ndev, u32 index,
			   struct tsn_cb_streamid *streamid);
int switch_cb_streamid_set(struct net_device *ndev, u32 index,
			   bool enable, struct tsn_cb_streamid *streamid);
int switch_qci_sfi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sfi_conf *sfi);
int switch_qci_sfi_set(struct net_device *ndev, u32 index,
		       bool enable, struct tsn_qci_psfp_sfi_conf *sfi);
int switch_cb_streamid_counters_get(struct net_device *ndev, u32 index,
				    struct tsn_cb_streamid_counters *s_counters);
int switch_qci_sfi_counters_get(struct net_device *ndev, u32 index,
				struct tsn_qci_psfp_sfi_counters *sfi_counters);
int switch_qci_sgi_set(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sgi_conf *sgi_conf);
int switch_qci_sgi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_sgi_conf *sgi_conf);
int switch_qci_sgi_status_get(struct net_device *ndev, u16 index,
			      struct tsn_psfp_sgi_status *sgi_status);
int switch_qci_fmi_set(struct net_device *ndev, u32 index,
		       bool enable, struct tsn_qci_psfp_fmi *fmi);
int switch_qci_fmi_get(struct net_device *ndev, u32 index,
		       struct tsn_qci_psfp_fmi *fmi);
int switch_seq_gen_set(struct net_device *ndev, u32 index,
		       struct tsn_seq_gen_conf *sg_conf);
int switch_seq_rec_set(struct net_device *ndev, u32 index,
		       struct tsn_seq_rec_conf *sr_conf);
int switch_pcp_map_set(struct net_device *ndev, bool enable);

#endif
