/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#ifdef DEBUG_FW_LOAD
#include "mhdp_firmware.h"
#endif
#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "imx-dp.h"

#define EDP_PHY_RESET	0x230

#ifdef DEBUG_FW_LOAD
void dp_fw_load(state_struct *state)
{
	printk("Loading DP Firmware\n");
	CDN_API_LoadFirmware(state,
		(u8 *)mhdp_iram0_get_ptr(),
		mhdp_iram0_get_size(),
		(u8 *)mhdp_dram0_get_ptr(),
		mhdp_dram0_get_size());
}
#endif

int dp_fw_init(state_struct *state, u32 core_rate)
{
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	int ret;
	u8 resp;
	u16 ver, verlib;

	/* configure the clock */
	CDN_API_SetClock(state, core_rate/1000000);

	cdn_apb_write(state, APB_CTRL << 2, 0);

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		DRM_ERROR("CDN_API_CheckAlive failed - check firmware!\n");
		return -ENXIO;
	}

	CDN_API_General_getCurVersion(state, &ver, &verlib);
	printk("FIRMWARE VERSION: %d, LIB VERSION: %d\n", ver, verlib);

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &resp);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		sizeof(echo_msg), CDN_BUS_TYPE_APB);

	return 0;
}

static const struct of_device_id scfg_device_ids[] = {
	{ .compatible = "fsl,ls1028a-scfg", },
	{}
};

void dp_phy_init(state_struct *state, int num_lanes, int max_link_rate, int tmp)
{
	int ret;
	struct device_node *scfg_node;
	void __iomem *scfg_base = NULL;

	scfg_node = of_find_matching_node(NULL, scfg_device_ids);
	if (scfg_node)
		scfg_base = of_iomap(scfg_node, 0);

	iowrite32(0, scfg_base + EDP_PHY_RESET);

	/* PHY initialization while phy reset pin is active */
	AFE_init(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);

#ifdef arch_imx
	/* In this point the phy reset should be deactivated */
	hdp_phy_reset(1);
#endif

	iowrite32(0x1, scfg_base + EDP_PHY_RESET);
	iounmap(scfg_base);

	/* PHY power set */
	AFE_power(state, num_lanes, (ENUM_AFE_LINK_RATE)max_link_rate);

	/* Video off */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 0);
}

/* Max Link Rate: 06h (1.62Gbps), 0Ah (2.7Gbps), 14h (5.4Gbps), 1Eh (8.1Gbps)--N/A */
void dp_mode_set(state_struct *state, int vic, int format, int color_depth, int max_link_rate)
{
	int ret;

	/* Set Host capabilities */
	/* Number of lanes and SSC */
	u8 num_lanes = 4;
	u8 ssc = 0;
	u8 scrambler = 1;
	/* Max voltage swing */
	u8 max_vswing = 3;
	u8 force_max_vswing = 0;
	/* Max pre-emphasis */
	u8 max_preemph = 2;
	u8 force_max_preemph = 0;
	/* Supported test patterns mask */
	u8 supp_test_patterns = 0x0F;
	/* AUX training? */
	u8 no_aux_training = 0;
	/* Lane mapping */
	u8 lane_mapping = 0x1B; /*  we have 4 lane, so it's OK */
	/* Extended Host capabilities */
	u8 ext_host_cap = 1;
	/* Bits per sub-pixel */
	u8 bits_per_subpixel = 8;
	/* Stereoscopic video */
	STEREO_VIDEO_ATTR stereo = 0;
	/* B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bt_type = 0;
	/* Transfer Unit */
	u8 transfer_unit = 64;
	VIC_SYMBOL_RATE sym_rate;
	S_LINK_STAT rls;
	u32 evt;
	u8 eventId;
	u8 HPDevents;

	CDN_API_DPTX_SetDbg_blocking(state, DPTX_DBG_SET_PWR_SKIP_SLEEP);

	ret = CDN_API_DPTX_SetHostCap_blocking(state,
		max_link_rate,
		(num_lanes & 0x7) | ((ssc & 1) << 3) | ((scrambler & 1) << 4),
		(max_vswing & 0x3) | ((force_max_vswing & 1) << 4),
		(max_preemph & 0x3) | ((force_max_preemph & 1) << 4),
		supp_test_patterns,
		no_aux_training, //fast link training
		lane_mapping,
		ext_host_cap
		);

	ret = CDN_API_DPTX_TrainingControl_blocking(state, 1);

	do {
		do {
			CDN_API_Get_Event(state, &evt);
		} while ((evt & 2) == 0);

		CDN_API_DPTX_ReadEvent_blocking(state, &eventId, &HPDevents);
		switch (eventId) {
		case 0x01:
			printk("INFO: Full link training started\n");
			break;
		case 0x02:
			printk("INFO: Fast link training started\n");
			break;
		case 0x04:
			printk("INFO: Clock recovery phase finished\n");
			break;
		case 0x08:
			printk("INFO: Channel equalization phase finished (this is last part meaning training finished)\n");
			break;
		case 0x10:
			printk("INFO: Fast link training finished\n");
			break;
		case 0x20:
			printk("ERROR: Clock recovery phase failed\n");
			break;;
		case 0x40:
			printk("ERROR: Channel equalization phase failed\n");
			break;
		case 0x80:
			printk("ERROR: Fast link training failed\n");
			break;
		default:
			printk("ERROR: Invalid ID:0x%.4X\n", eventId);
			break;
		}
	} while (eventId != 0x08 && eventId != 0x10);

	ret = CDN_API_DPTX_ReadLinkStat_blocking(state, &rls);
	printk("INFO: Get Read Link Status (ret = %d resp:\n"
	       "rate: %d, lanes: %d\n"
	       "vswing 0..3: %d %d %d\n"
	       "preemp 0..3: %d %d %d\n",
	       ret, rls.rate, rls.lanes,
	       rls.swing[0], rls.swing[1], rls.swing[2],
	       rls.preemphasis[0], rls.preemphasis[1], rls.preemphasis[2]);

	switch (rls.rate) {
	case 0x0a:
		sym_rate = RATE_2_7;
		break;
	case 0x14:
		sym_rate = RATE_5_4;
		break;
	default:
		sym_rate = RATE_1_6;
	}

	ret = CDN_API_DPTX_Set_VIC_blocking(state,
		vic,
		bits_per_subpixel,
		num_lanes,
		sym_rate,
		format,
		stereo,
		bt_type,
		transfer_unit
		);

	/* Set video on */
	ret = CDN_API_DPTX_SetVideo_blocking(state, 1);

	udelay(1000);
}

int dp_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	DPTX_Read_EDID_response edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = 0;

	memset(&edidResp, 0, sizeof(edidResp));
	switch (block) {
	case 0:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 0, 0, &edidResp);
		break;
	case 1:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 0, 1, &edidResp);
		break;
	case 2:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 1, 0, &edidResp);
		break;
	case 3:
		ret = CDN_API_DPTX_Read_EDID_blocking(state, 1, 1, &edidResp);
		break;
	default:
		pr_warn("EDID block %x read not support\n", block);
	}

	memcpy(buf, edidResp.buff, 128);

	return ret;
}

void dp_get_hpd_state(state_struct *state, u8 *hpd)
{
	CDN_API_DPTX_GetHpdStatus_blocking(state, hpd);
}
