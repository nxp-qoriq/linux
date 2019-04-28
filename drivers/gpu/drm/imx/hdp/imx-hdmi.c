/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */

#include <linux/clk.h>
#include "hdmitx_firmware.h"
#include "imx-hdp.h"
#include "imx-hdmi.h"
#include "API_AFE_ss28fdsoi_kiran_hdmitx.h"

static int character_freq_khz;

void hdmi_fw_load(state_struct *state)
{
	CDN_API_LoadFirmware(state,
		(u8 *)hdmitx_iram0_get_ptr(),
		hdmitx_iram0_get_size(),
		(u8 *)hdmitx_dram0_get_ptr(),
		hdmitx_dram0_get_size());
}

int hdmi_fw_init(state_struct *state)
{
	u8 echo_msg[] = "echo test";
	u8 echo_resp[sizeof(echo_msg) + 1];
	struct imx_hdp *hdp = state_to_imx_hdp(state);
	u32 core_rate;
	int ret;
	u8 sts;

	core_rate = clk_get_rate(hdp->clks.clk_core);

	/* configure the clock */
	CDN_API_SetClock(state, core_rate/1000000);

	/* moved from CDN_API_LoadFirmware */
	cdn_apb_write(state, APB_CTRL << 2, 0);

	ret = CDN_API_CheckAlive_blocking(state);
	if (ret != 0) {
		DRM_ERROR("CDN_API_CheckAlive failed - check firmware!\n");
		return -ENXIO;
	}

	/* turn on IP activity */
	ret = CDN_API_MainControl_blocking(state, 1, &sts);

	ret = CDN_API_General_Test_Echo_Ext_blocking(state, echo_msg, echo_resp,
		 sizeof(echo_msg), CDN_BUS_TYPE_APB);

	return 0;
}

int hdmi_phy_init(state_struct *state, struct drm_display_mode *mode,
		  int format, int color_depth)
{
        struct imx_hdp *hdp = state_to_imx_hdp(state);
	int ret;

	/* Configure PHY */
	character_freq_khz = phy_cfg_hdp_ss28fdsoi(state, 4, mode,
						   color_depth, format);

	imx_hdp_call(hdp, phy_reset, hdp->ipcHndl, NULL, 0);

	hdmi_tx_kiran_power_configuration_seq(state, 4);

	/* Set the lane swapping */
	ret = CDN_API_General_Write_Register_blocking(state, ADDR_SOURCD_PHY + (LANES_CONFIG << 2),
		F_SOURCE_PHY_LANE0_SWAP(3) | F_SOURCE_PHY_LANE1_SWAP(0) |
		F_SOURCE_PHY_LANE2_SWAP(1) | F_SOURCE_PHY_LANE3_SWAP(2) |
		F_SOURCE_PHY_COMB_BYPASS(0) | F_SOURCE_PHY_20_10(1));

	return true;
}

void hdmi_mode_set(state_struct *state, struct drm_display_mode *mode,
		   int format, int color_depth, int temp)
{
	int ret;

	/* B/W Balance Type: 0 no data, 1 IT601, 2 ITU709 */
	BT_TYPE bw_type = 0;
	/* Mode = 0 - DVI, 1 - HDMI1.4, 2 HDMI 2.0 */
	HDMI_TX_MAIL_HANDLER_PROTOCOL_TYPE ptype = 1;

	if (drm_match_cea_mode(mode) == VIC_MODE_97_60Hz)
		ptype = 2;

	ret = CDN_API_HDMITX_Init_blocking(state);

	/* Set HDMI TX Mode */
	ret = CDN_API_HDMITX_Set_Mode_blocking(state, ptype, character_freq_khz);

	ret = CDN_API_Set_AVI(state, mode, format, bw_type);

	ret =  CDN_API_HDMITX_SetVic_blocking(state, mode, color_depth, format);

	msleep(50);
}

int hdmi_get_edid_block(void *data, u8 *buf, u32 block, size_t len)
{
	HDMITX_TRANS_DATA edidResp;
	state_struct *state = data;
	CDN_API_STATUS ret = 0;

	memset(&edidResp, 0, sizeof(edidResp));
	switch (block) {
	case 0:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 0, 0, &edidResp);
		break;
	case 1:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 0, 1, &edidResp);
		break;
	case 2:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 1, 0, &edidResp);
		break;
	case 3:
		ret = CDN_API_HDMITX_READ_EDID_blocking(state, 1, 1, &edidResp);
		break;
	default:
		printk(KERN_ERR "EDID block %x read not support\n", block);
	}

	memcpy(buf, edidResp.buff, 128);

	return ret;
}

void hdmi_get_hpd_state(state_struct *state, u8 *hpd)
{
	CDN_API_HDMITX_GetHpdStatus_blocking(state, hpd);
}
