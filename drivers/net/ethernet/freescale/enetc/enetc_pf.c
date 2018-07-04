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

#include <linux/module.h>
#include "enetc_pf.h"

#define ENETC_DRV_VER_MAJ 0
#define ENETC_DRV_VER_MIN 8

#define ENETC_DRV_VER_STR __stringify(ENETC_DRV_VER_MAJ) "." \
			  __stringify(ENETC_DRV_VER_MIN)
static const char enetc_drv_ver[] = ENETC_DRV_VER_STR;
#define ENETC_DRV_NAME_STR "ENETC PF driver"
static const char enetc_drv_name[] = ENETC_DRV_NAME_STR;

/* PF driver params */
module_param(debug, uint, 0);
static uint prune;
module_param(prune, uint, 0);

static void enetc_pf_get_primary_mac_addr(struct enetc_hw *hw, int si, u8 *addr)
{
	u32 upper = enetc_port_rd(hw, ENETC_PSIPMAR0(si));
	u16 lower = enetc_port_rd(hw, ENETC_PSIPMAR1(si));

	*(u32 *)addr = upper;
	*(u16 *)(addr + 4) = lower;
}

static void enetc_pf_set_primary_mac_addr(struct enetc_hw *hw, int si,
				       const u8 *addr)
{
	u32 upper = *(const u32 *)addr;
	u16 lower = *(const u16 *)(addr + 4);

	enetc_port_wr(hw, ENETC_PSIPMAR0(si), upper);
	enetc_port_wr(hw, ENETC_PSIPMAR1(si), lower);
}

static int enetc_pf_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sockaddr *saddr = addr;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, saddr->sa_data, ndev->addr_len);
	enetc_pf_set_primary_mac_addr(&priv->si->hw, 0, saddr->sa_data);

	return 0;
}

static void enetc_set_vlan_promisc(struct enetc_hw *hw, char si_map)
{
	u32 val = enetc_port_rd(hw, ENETC_PSIPVMR);

	val &= ~ENETC_PSIPVMR_SET_VP(ENETC_VLAN_PROMISC_MAP_ALL);
	enetc_port_wr(hw, ENETC_PSIPVMR, ENETC_PSIPVMR_SET_VP(si_map) | val);
}

static bool enetc_si_vlan_promisc_is_on(struct enetc_pf *pf, int si_idx)
{
	return pf->vlan_promisc_simap & BIT(si_idx);
}

static bool enetc_vlan_filter_is_on(struct enetc_pf *pf)
{
	int i;

	for_each_set_bit(i, pf->active_vlans, VLAN_N_VID)
		return true;

	return false;
}

static void enetc_enable_si_vlan_promisc(struct enetc_pf *pf, int si_idx)
{
	pf->vlan_promisc_simap |= BIT(si_idx);
	enetc_set_vlan_promisc(&pf->si->hw, pf->vlan_promisc_simap);
}

static void enetc_disable_si_vlan_promisc(struct enetc_pf *pf, int si_idx)
{
	pf->vlan_promisc_simap &= ~BIT(si_idx);
	enetc_set_vlan_promisc(&pf->si->hw, pf->vlan_promisc_simap);
}

static void enetc_set_isol_vlan(struct enetc_hw *hw, int si, u16 vlan, u8 qos)
{
	u32 val = 0;

	if (vlan)
		val = ENETC_PSIVLAN_EN | ENETC_PSIVLAN_SET_QOS(qos) | vlan;

	enetc_port_wr(hw, ENETC_PSIVLANR(si), val);
}

static int enetc_mac_addr_hash_idx(const u8 *addr)
{
	u64 fold = __swab64(ether_addr_to_u64(addr)) >> 16;
	u64 mask = 0;
	int res = 0;
	int i;

	for (i = 0; i < 8; i++)
		mask |= BIT_ULL(i * 6);

	for (i = 0; i < 6; i++)
		res |= (__sw_hweight64(fold & (mask << i)) & 0x1) << i;

	return res;
}

static void enetc_reset_mac_addr_filter(struct enetc_mac_filter *filter)
{
	filter->mac_addr_cnt = 0;

	bitmap_zero(filter->mac_hash_table,
		    ENETC_MADDR_HASH_TBL_SZ);
}

static void enetc_add_mac_addr_em_filter(struct enetc_mac_filter *filter,
					 const unsigned char *addr)
{
	/* add exact match addr */
	ether_addr_copy(filter->mac_addr, addr);
	filter->mac_addr_cnt++;
}

static void enetc_add_mac_addr_ht_filter(struct enetc_mac_filter *filter,
					 const unsigned char *addr)
{
	int idx = enetc_mac_addr_hash_idx(addr);

	/* add hash table entry */
	__set_bit(idx, filter->mac_hash_table);
	filter->mac_addr_cnt++;
}

static void enetc_clear_mac_ht_flt(struct enetc_hw *hw, int si_idx, int type)
{
	if (type == UC) { // FIXME: Swap UC with MC low bits, TKT381557
		enetc_port_wr(hw, ENETC_PSIMMHFR0(si_idx), 0);
		enetc_port_wr(hw, ENETC_PSIUMHFR1(si_idx), 0);
	} else { /* MC */
		enetc_port_wr(hw, ENETC_PSIUMHFR0(si_idx), 0);
		enetc_port_wr(hw, ENETC_PSIMMHFR1(si_idx), 0);
	}
}

static void enetc_set_mac_ht_flt(struct enetc_hw *hw, int si_idx, int type,
				 u32 *hash)
{
	if (type == UC) { // FIXME: Swap UC with MC low bits, TKT381557
		enetc_port_wr(hw, ENETC_PSIMMHFR0(si_idx), *hash);
		enetc_port_wr(hw, ENETC_PSIUMHFR1(si_idx), *(hash + 1));
	} else { /* MC */
		enetc_port_wr(hw, ENETC_PSIUMHFR0(si_idx), *hash);
		enetc_port_wr(hw, ENETC_PSIMMHFR1(si_idx), *(hash + 1));
	}
}

static void enetc_sync_mac_filters(struct enetc_pf *pf)
{
	struct enetc_mac_filter *f = pf->mac_filter;
	struct enetc_si *si = pf->si;
	int i, pos;

	pos = EMETC_MAC_ADDR_FILT_RES;

	for (i = 0; i < MADDR_TYPE; i++, f++) {
		bool em = (f->mac_addr_cnt == 1) && (i == UC);
		bool clear = !f->mac_addr_cnt;

		if (clear) {
			if (i == UC)
				enetc_clear_mac_flt_entry(si, pos);

			enetc_clear_mac_ht_flt(&si->hw, 0, i);
			continue;
		}

		/* exact match filter */
		if (em) {
			enetc_clear_mac_ht_flt(&si->hw, 0, UC);

			enetc_set_mac_flt_entry(si, pos, f->mac_addr,
						BIT(0));
			continue;
		}

		/* hash table filter, clear EM filter for UC entries */
		if (i == UC)
			enetc_clear_mac_flt_entry(si, pos);

		enetc_set_mac_ht_flt(&si->hw, 0, i, (u32 *)f->mac_hash_table);
	}
}

static void enetc_pf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &priv->si->hw;
	bool uprom = false, mprom = false;
	struct enetc_mac_filter *filter;
	struct netdev_hw_addr *ha;
	u32 psipmr = 0;
	bool em;

	if (ndev->flags & IFF_PROMISC) {
		/* enable promisc mode for SI0 (PF) */
		psipmr = ENETC_PSIPMR_SET_UP(0) | ENETC_PSIPMR_SET_MP(0);
		uprom = true;
		mprom = true;
		/* enable VLAN promisc mode for SI0 */
		if (!enetc_si_vlan_promisc_is_on(pf, 0))
			enetc_enable_si_vlan_promisc(pf, 0);

	} else if (ndev->flags & IFF_ALLMULTI) {
		/* enable multi cast promisc mode for SI0 (PF) */
		psipmr = ENETC_PSIPMR_SET_MP(0);
		mprom = true;
	}

	/* first 2 filter entries belong to PF */
	if (!uprom) {
		/* Update unicast filters */
		filter = &pf->mac_filter[UC];
		enetc_reset_mac_addr_filter(filter);

		em = (netdev_uc_count(ndev) == 1);
		netdev_for_each_uc_addr(ha, ndev) {
			if (em) {
				enetc_add_mac_addr_em_filter(filter, ha->addr);
				break;
			}

			enetc_add_mac_addr_ht_filter(filter, ha->addr);
		}
	}

	if (!mprom) {
		/* Update multicast filters */
		filter = &pf->mac_filter[MC];
		enetc_reset_mac_addr_filter(filter);

		netdev_for_each_mc_addr(ha, ndev) {
			if (!is_multicast_ether_addr(ha->addr))
				continue;

			enetc_add_mac_addr_ht_filter(filter, ha->addr);
		}
	}

	if (!uprom || !mprom)
		/* update PF entries */
		enetc_sync_mac_filters(pf);

	psipmr |= enetc_port_rd(hw, ENETC_PSIPMR) &
		  ~(ENETC_PSIPMR_SET_UP(0) | ENETC_PSIPMR_SET_MP(0));
	enetc_port_wr(hw, ENETC_PSIPMR, psipmr);
}

static void enetc_set_vlan_ht_filter(struct enetc_hw *hw, int si_idx,
				     u32 *hash)
{
	enetc_port_wr(hw, ENETC_PSIVHFR0(si_idx), *hash);
	enetc_port_wr(hw, ENETC_PSIVHFR1(si_idx), *(hash + 1));
}

static int enetc_vid_hash_idx(unsigned int vid)
{
	int res = 0;
	int i;

	for (i = 0; i < 6; i++)
		res |= (__sw_hweight8(vid & (BIT(i) | BIT(i + 6))) & 0x1) << i;

	return res;
}

static void enetc_sync_vlan_ht_filter(struct enetc_pf *pf, bool rehash)
{
	int i;

	if (rehash) {
		bitmap_zero(pf->vlan_ht_filter, ENETC_VLAN_HT_SIZE);

		for_each_set_bit(i, pf->active_vlans, VLAN_N_VID) {
			int hidx = enetc_vid_hash_idx(i);

			__set_bit(hidx, pf->vlan_ht_filter);
		}
	}

	enetc_set_vlan_ht_filter(&pf->si->hw, 0, (u32 *)pf->vlan_ht_filter);
}

static int enetc_vlan_rx_add_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	int idx;

	if (enetc_si_vlan_promisc_is_on(pf, 0))
		enetc_disable_si_vlan_promisc(pf, 0);

	__set_bit(vid, pf->active_vlans);

	idx = enetc_vid_hash_idx(vid);
	if (!__test_and_set_bit(idx, pf->vlan_ht_filter))
		enetc_sync_vlan_ht_filter(pf, false);

	return 0;
}

static int enetc_vlan_rx_del_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	__clear_bit(vid, pf->active_vlans);
	enetc_sync_vlan_ht_filter(pf, true);

	if (!enetc_vlan_filter_is_on(pf))
		enetc_enable_si_vlan_promisc(pf, 0);

	return 0;
}

static void enetc_set_loopback(struct net_device *ndev, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_hw *hw = &priv->si->hw;
	u32 reg;

	reg = enetc_port_rd(hw, ENETC_PM0_IF_MODE);
	if (reg & ENETC_PMO_IFM_RG) {
		/* RGMII mode */
		reg = en ? reg | ENETC_PM0_IFM_RLP : reg & ~ENETC_PM0_IFM_RLP;
		enetc_port_wr(hw, ENETC_PM0_IF_MODE, reg);
	} else {
		/* assume SGMII mode */
		reg = enetc_port_rd(hw, ENETC_PM0_CMD_CFG);
		reg = en ? reg | ENETC_PM0_CMD_XGLP : reg & ~ENETC_PM0_CMD_XGLP;
		enetc_port_wr(hw, ENETC_PM0_CMD_CFG, reg);
	}
}

static int enetc_pf_set_vf_mac(struct net_device *ndev, int vf, u8 *mac)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	if (vf >= pf->total_vfs)
		return -EINVAL;

	enetc_pf_set_primary_mac_addr(&priv->si->hw, vf + 1, mac);
	return 0;
}

static int enetc_pf_set_vf_vlan(struct net_device *ndev, int vf, u16 vlan,
				u8 qos, __be16 proto)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	if (vf >= pf->total_vfs)
		return -EINVAL;

	if (proto != htons(0x8100))
		/* only C-tags supported for now */
		return -EPROTONOSUPPORT;

	enetc_set_isol_vlan(&priv->si->hw, vf + 1, vlan, qos);
	return 0;
}

static int enetc_pf_set_vf_spoofchk(struct net_device *ndev, int vf, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	u32 cfgr;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	cfgr = enetc_port_rd(&priv->si->hw, ENETC_PSICFGR0(vf + 1));
	cfgr = (cfgr & ~ENETC_PSICFGR0_ASE) | (en ? ENETC_PSICFGR0_ASE : 0);
	enetc_port_wr(&priv->si->hw, ENETC_PSICFGR0(vf + 1), cfgr);

	return 0;
}

static void enetc_port_setup_primary_mac_address(struct enetc_si *si)
{
	unsigned char mac_addr[MAX_ADDR_LEN];
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int i;

	/* check MAC addresses for PF and all VFs, if any is 0 set it ro rand */
	for (i=0; i<pf->total_vfs + 1; i++) {
		enetc_pf_get_primary_mac_addr(hw, i, mac_addr);
		if (!is_zero_ether_addr(mac_addr))
			continue;
		eth_random_addr(mac_addr);
		dev_info(&si->pdev->dev, "no MAC address specified for SI%d, using %pM\n",
			 i, mac_addr);
		enetc_pf_set_primary_mac_addr(hw, i, mac_addr);
	}
}

static void enetc_port_alloc_rfs(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int num_entries, vf_entries, i;

	/* split RFS entries between functions */
	num_entries = enetc_port_rd(hw, ENETC_PRFSCAPR) & 0xf;
	num_entries = 32 * (1 << num_entries);
	vf_entries = num_entries / (pf->total_vfs + 1);

	for (i = 0; i < pf->total_vfs; i++)
		enetc_port_wr(hw, ENETC_PSIRFSCFGR(i + 1), vf_entries);
	enetc_port_wr(hw, ENETC_PSIRFSCFGR(0),
		      num_entries - vf_entries * pf->total_vfs);

	/* enable RFS on port */
	enetc_port_wr(hw, ENETC_PRFSMR, ENETC_PRFSMR_RFSE);
}

static void enetc_port_si_configure(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int num_rings, i;
	u32 val;

	val = enetc_port_rd(hw, ENETC_PCAPR0);
	num_rings = min(ENETC_PCAPR0_RXBDR(val), ENETC_PCAPR0_TXBDR(val));

	val = ENETC_PSICFGR0_SET_TXBDR(ENETC_PF_NUM_RINGS);
	val |= ENETC_PSICFGR0_SET_RXBDR(ENETC_PF_NUM_RINGS);

	if (unlikely(num_rings < ENETC_PF_NUM_RINGS)) {
		val = ENETC_PSICFGR0_SET_TXBDR(num_rings);
		val |= ENETC_PSICFGR0_SET_RXBDR(num_rings);

		dev_warn(&si->pdev->dev, "Found %d rings, expected %d!\n",
			 num_rings, ENETC_PF_NUM_RINGS);

		num_rings = 0;
	}

	/* Add default one-time settings for SI0 (PF) */
	val |= ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);
	if (prune)
		val |= ENETC_PSICFGR0_SPE;

	enetc_port_wr(hw, ENETC_PSICFGR0(0), val);

	if (num_rings)
		num_rings -= ENETC_PF_NUM_RINGS;

	/* Configure the SIs for each available VF */
	val = ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);
	val |= ENETC_PSICFGR0_VTE | ENETC_PSICFGR0_SIVIE;
	if (prune)
		val |= ENETC_PSICFGR0_SPE;

	if (num_rings) {
		num_rings /= pf->total_vfs;
		val |= ENETC_PSICFGR0_SET_TXBDR(num_rings);
		val |= ENETC_PSICFGR0_SET_RXBDR(num_rings);
	}

	for (i = 0; i < pf->total_vfs; i++)
		enetc_port_wr(hw, ENETC_PSICFGR0(i+1), val);

	/* Port level VLAN settings */
	val = ENETC_PVCLCTR_OVTPIDL(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);
	enetc_port_wr(hw, ENETC_PVCLCTR, val);
	/* use outer tag for VLAN filtering */
	enetc_port_wr(hw, ENETC_PSIVLANFMR, ENETC_PSIVLANFMR_VS);
}

static void enetc_configure_port_mac(struct enetc_hw *hw)
{
	enetc_port_wr(hw, ENETC_PM0_MAXFRM,
		      ENETC_SET_MAXFRM(ENETC_RX_MAXFRM_SIZE));

	enetc_port_wr(hw, ENETC_PTCMSDUR(0), ENETC_MAC_MAXFRM_SIZE);
	enetc_port_wr(hw, ENETC_PTXMBAR, 2 * ENETC_MAC_MAXFRM_SIZE);

	enetc_port_wr(hw, ENETC_PM0_CMD_CFG, ENETC_PM0_CMD_PHY_TX_EN |
		      ENETC_PM0_CMD_TXP	| ENETC_PM0_PROMISC |
		      ENETC_PM0_TX_EN | ENETC_PM0_RX_EN);
}

static void enetc_configure_port(struct enetc_pf *pf)
{
	u8 hash_key[ENETC_RSSHASH_KEY_SIZE];
	struct enetc_hw *hw = &pf->si->hw;

	enetc_configure_port_mac(hw);

	enetc_port_si_configure(pf->si);

	/* set up hash key */
	get_random_bytes(hash_key, ENETC_RSSHASH_KEY_SIZE);
	enetc_set_rss_key(hw, hash_key);

	/* split up RFS entries */
	enetc_port_alloc_rfs(pf->si);

	/* fix-up primary MAC addresses, if not set already */
	enetc_port_setup_primary_mac_address(pf->si);

	/* enforce VLAN promisc mode for all SIs */
	pf->vlan_promisc_simap = ENETC_VLAN_PROMISC_MAP_ALL;
	enetc_set_vlan_promisc(hw, pf->vlan_promisc_simap);

	enetc_port_wr(hw, ENETC_PSIPMR, 0);

	/* enable port */
	enetc_port_wr(hw, ENETC_PMR, ENETC_PMR_EN);
}

#ifdef CONFIG_PCI_IOV
static int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	int err;

	if (!num_vfs) {
		dev_info(&pdev->dev, "SR-IOV stop\n");
		enetc_msg_psi_free(pf);
		pci_disable_sriov(pdev);
		pf->num_vfs = 0;
	} else {
		dev_info(&pdev->dev, "SR-IOV start, %d VFs\n", num_vfs);
		err = pci_enable_sriov(pdev, num_vfs);
		if (err) {
			dev_err(&pdev->dev, "pci_enable_sriov err %d\n", err);
			return err;
		}

		pf->num_vfs = num_vfs;
		err = enetc_msg_psi_init(pf);
		if (err) {
			dev_err(&pdev->dev, "enetc_msg_psi_init (%d)\n", err);
			goto err_msg_psi;
		}
	}

	return num_vfs;

err_msg_psi:
	pci_disable_sriov(pdev);
	pf->num_vfs = 0;

	return err;
}
#else
#define enetc_sriov_configure(pdev, num_vfs)	(void)0
#endif

static int enetc_pf_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	if (changed & NETIF_F_HW_VLAN_CTAG_RX)
		enetc_enable_rxvlan(&priv->si->hw, 0,
				    !!(features & NETIF_F_HW_VLAN_CTAG_RX));

	if (changed & NETIF_F_HW_VLAN_CTAG_TX)
		enetc_enable_txvlan(&priv->si->hw, 0,
				    !!(features & NETIF_F_HW_VLAN_CTAG_TX));

	if (changed & NETIF_F_LOOPBACK)
		enetc_set_loopback(ndev, !!(features & NETIF_F_LOOPBACK));

	return enetc_set_features(ndev, features);
}

static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_pf_set_mac_addr,
	.ndo_set_rx_mode	= enetc_pf_set_rx_mode,
	.ndo_vlan_rx_add_vid	= enetc_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= enetc_vlan_rx_del_vid,
	.ndo_set_vf_mac		= enetc_pf_set_vf_mac,
	.ndo_set_vf_vlan	= enetc_pf_set_vf_vlan,
	.ndo_set_vf_spoofchk	= enetc_pf_set_vf_spoofchk,
	.ndo_set_features	= enetc_pf_set_features,
};

static void enetc_pf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
				  const struct net_device_ops *ndev_ops)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_WOL << 1) - 1; //TODO: netif_msg_init()
	ndev->netdev_ops = ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = ENETC_MAX_MTU;

	ndev->hw_features = NETIF_F_RXCSUM | NETIF_F_HW_CSUM |
			    NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_LOOPBACK | NETIF_F_RXHASH;
	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG |
			 NETIF_F_RXCSUM | NETIF_F_HW_CSUM |
			 NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_HW_VLAN_CTAG_RX |
			 NETIF_F_HW_VLAN_CTAG_FILTER;

	ndev->priv_flags |= IFF_UNICAST_FLT;

	/* pick up primary MAC address from SI */
	enetc_get_primary_mac_addr(&si->hw, ndev->dev_addr);
}

static void enetc_pf_sw_init(struct enetc_ndev_priv *priv)
{
	struct enetc_si *si = priv->si;

	priv->tx_bd_count = 1024; //TODO: use defines for defaults
	priv->rx_bd_count = 1024;

	/* we only use one ring per CPU for now */
	priv->num_rx_rings = min_t(u16, num_online_cpus(),
				   si->num_rx_rings);
	priv->num_tx_rings = min_t(u16, num_online_cpus(),
				   si->num_tx_rings);
	 // TODO: fixed to Rx/TX pair, make configurable
	priv->bdr_int_num = priv->num_rx_rings;

	/* SI specific */
	si->cbd_ring.bd_count = 64; //TODO: use defines for defaults
}

static int enetc_pf_probe(struct pci_dev *pdev,
			  const struct pci_device_id *ent)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, sizeof(*pf));
	if (err) {
		dev_err(&pdev->dev, "PCI probing failed\n");
		return err;
	}

	si = pci_get_drvdata(pdev);
	if (!si->hw.port || !si->hw.global) {
		err = -ENODEV;
		dev_err(&pdev->dev, "could not map PF space, probing a VF?\n");
		goto err_map_pf_space;
	}

	pf = enetc_si_priv(si);
	pf->si = si;
	pf->total_vfs = pci_sriov_get_totalvfs(pdev);

	enetc_configure_port(pf);

	enetc_get_si_caps(si);

	ndev = alloc_etherdev_mq(sizeof(*priv), ENETC_MAX_NUM_TXQS);
	if (!ndev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	enetc_pf_netdev_setup(si, ndev, &enetc_ndev_ops);

	priv = netdev_priv(ndev);

	enetc_pf_sw_init(priv);

	err = enetc_alloc_si_resources(priv);
	if (err) {
		dev_err(&pdev->dev, "SI resource alloc failed\n");
		goto err_alloc_si_res;
	}

	err = enetc_alloc_msix(priv);
	if (err) {
		dev_err(&pdev->dev, "MSIX alloc failed\n");
		goto err_alloc_msix;
	}

	enetc_tsn_init(si);

	err = register_netdev(ndev);
	if (err)
		goto err_reg_netdev;

	err = enetc_setup_irqs(priv);
	if (err)
		goto err_setup_irq;

	netif_carrier_off(ndev);

	netif_info(priv, probe, ndev, "%s v%s\n",
		   enetc_drv_name, enetc_drv_ver);

	return 0;

err_setup_irq:
	unregister_netdev(ndev);
err_reg_netdev:
	enetc_free_msix(priv);
err_alloc_msix:
	enetc_free_si_resources(priv);
err_alloc_si_res:
	si->ndev = NULL;
	free_netdev(ndev);
err_alloc_netdev:
err_map_pf_space:
	enetc_pci_remove(pdev);

	return err;
}

static void enetc_pf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_ndev_priv *priv;

	if (pf->num_vfs)
		enetc_sriov_configure(pdev, 0);

	priv = netdev_priv(si->ndev);
	netif_info(priv, drv, si->ndev, "%s v%s remove\n",
		   enetc_drv_name, enetc_drv_ver);
	unregister_netdev(si->ndev);

	enetc_free_irqs(priv);
	enetc_free_msix(priv);

	enetc_free_si_resources(priv);

	free_netdev(si->ndev);

	enetc_pci_remove(pdev);
}

static const struct pci_device_id enetc_pf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_PF) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_pf_id_table);

static struct pci_driver enetc_pf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_pf_id_table,
	.probe = enetc_pf_probe,
	.remove = enetc_pf_remove,
#ifdef CONFIG_PCI_IOV
	.sriov_configure = enetc_sriov_configure,
#endif
};
module_pci_driver(enetc_pf_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(ENETC_DRV_VER_STR);
