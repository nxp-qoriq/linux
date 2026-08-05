// SPDX-License-Identifier: GPL-2.0-only
/*
 * ITE IT6251 LVDS-to-eDP bridge driver.
 *
 * Copyright (C) 2014 Sean Cross <xobs@kosagi.com>
 *
 * Rework for mainline: Marek Vasut <marex@denx.de>
 * Modernized to atomic bridge + drm_bridge_connector: NXP 2024
 * Full baremetal feature port (AUX EDID, HPD/vidstable poll, retrain,
 * power-down sequence, DT config knobs): NXP 2025
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>

/* eDP TX register map (I2C address 0x4C) */
#define IT6251_VENDOR_ID_LOW                    0x00
#define IT6251_VENDOR_ID_HIGH                   0x01
#define IT6251_DEVICE_ID_LOW                    0x02
#define IT6251_DEVICE_ID_HIGH                   0x03
#define IT6251_DP_SW_RESET                      0x05
#define IT6251_DP_SWRST_ALL                     0xff
#define IT6251_DP_SWRST_AUX                     BIT(3)
#define IT6251_INT_STATUS                       0x06
#define IT6251_INT_MASK                         0x09
#define IT6251_INT_HPD_IRQ                      BIT(0)
#define IT6251_INT_HPD_CHG                      BIT(1)
#define IT6251_INT_VIDSTABLE                    BIT(2)
#define IT6251_INT_ENABLE_MASK                  0x07
#define IT6251_SYSTEM_STATUS                    0x0d
#define IT6251_SYSTEM_STATUS_RINTSTATUS         BIT(0)
#define IT6251_SYSTEM_STATUS_RHPDSTATUS         BIT(1)
#define IT6251_SYSTEM_STATUS_RVIDEOSTABLE       BIT(2)
#define IT6251_SYSTEM_STATUS_RPLL_IOLOCK        BIT(3)
#define IT6251_SYSTEM_STATUS_RPLL_XPLOCK        BIT(4)
#define IT6251_SYSTEM_STATUS_RPLL_SPLOCK        BIT(5)
#define IT6251_SYSTEM_STATUS_RAUXFREQ_LOCK      BIT(6)
#define IT6251_REF_STATE                        0x0e
#define IT6251_REF_STATE_NORMAL_OPERATION       BIT(4)
#define IT6251_REF_STATE_MUTED                  BIT(5)
#define IT6251_LINK_STATE_MASK                  0x1f
#define IT6251_LINK_STATE_NORMAL                0x10
#define IT6251_LINK_STATE_RATE_HBR              BIT(7)
#define IT6251_RPCLK_CNT_LOW                    0x13
#define IT6251_RPCLK_CNT_HIGH                   0x14
#define IT6251_DP_LANE_CFG                      0x16
#define IT6251_DP_LN_CFG_LBR                    BIT(0)
#define IT6251_DP_LN_CFG_1_LANE                 (0 << 1)
#define IT6251_DP_LN_CFG_2_LANE                 (1 << 1)
#define IT6251_DP_LN_CFG_4_LANE                 (3 << 1)
#define IT6251_DP_LN_CFG_SSC                    BIT(4)
#define IT6251_DP_TRAIN_CTRL                    0x17
#define IT6251_DP_VSWING                        0x19
#define IT6251_DP_PREEMPH                       0x1a
#define IT6251_AUX_CTRL                         0x23
#define IT6251_AUX_CTRL_PC_EN                   BIT(6)
#define IT6251_AUX_CTRL_NATIVE_RD               0x02
#define IT6251_AUX_CTRL_EDID_EN                 0x03
#define IT6251_AUX_CTRL_EDID_FIFO_CLR           0x82
#define IT6251_AUX_ADDR0                        0x24
#define IT6251_AUX_ADDR1                        0x25
#define IT6251_AUX_ADDR2                        0x26
#define IT6251_AUX_FIRE                         0x2b
#define IT6251_AUX_FIRE_NATIVE_RD               0x00
#define IT6251_AUX_FIRE_EDID_RD                 0x0b
#define IT6251_AUX_FIRE_BUSY                    BIT(5)
#define IT6251_AUX_RD_DATA                      0x2c
#define IT6251_AUX_EDID_FIFO                    0x2f
#define IT6251_DP_PCLK_CFG                      0x60
#define IT6251_DP_PCLK_HALF                     BIT(1)
#define IT6251_DP_IN_CFG                        0x61
#define IT6251_DP_IN_DUAL_PIX                   BIT(2)
#define IT6251_DP_IN_CFG2                       0x62
#define IT6251_DP_IN_6BPP                       0x00
#define IT6251_DP_IN_8BPP                       0x01
#define IT6251_DP_IN_10BPP                      0x02
#define IT6251_DP_IN_12BPP                      0x03
#define IT6251_DP_SYNC_CFG                      0xa0
#define IT6251_HPD_TIMER0                       0xc9
#define IT6251_HPD_TIMER1                       0xca
#define IT6251_HPD_TIMER2                       0xcb
#define IT6251_DP_FRAME_CFG                     0xd3
#define IT6251_DP_VID_CFG                       0xd4
#define IT6251_DP_QUEUE_CFG                     0xe7
#define IT6251_DP_PKT_CFG                       0xe8
#define IT6251_DP_AVI_CFG                       0xec
#define IT6251_DPHDEW_LOW                       0xa5
#define IT6251_DPHDEW_HIGH                      0xa6
#define IT6251_DPVDEW_LOW                       0xaf
#define IT6251_DPVDEW_HIGH                      0xb0
#define IT6251_LVDS_PORT_ADDR                   0xfd
#define IT6251_LVDS_PORT_CTRL                   0xfe
#define IT6251_LVDS_PORT_CTRL_EN                BIT(0)

/* LVDS RX register map (I2C address 0x4D) */
#define IT6251_LVDS_SW_RESET                    0x05
#define IT6251_LVDS_SWRST_ALL                   0xff
#define IT6251_LVDS_SWRST_PCLK                  0x02
#define IT6251_LVDS_LINK_CFG                    0x0b
#define IT6251_LVDS_FMT                         0x2c
#define IT6251_LVDS_FMT_6BPP                    0x00
#define IT6251_LVDS_FMT_8BPP                    0x01
#define IT6251_LVDS_FMT_10BPP                   0x02
#define IT6251_LVDS_FMT_12BPP                   0x03
#define IT6251_LVDS_FMT_VESA                    BIT(4)
#define IT6251_LVDS_FMT_DESSC_DIS               BIT(6)
#define IT6251_LVDS_RPC_REQ                     0x2b
#define IT6251_LVDS_DCBAL_CFG                   0x32
#define IT6251_LVDS_PLL_CFG                     0x3b
#define IT6251_LVDS_PLL_RESET                   0x42
#define IT6251_LVDS_PLL_NORMAL                  0x43
#define IT6251_LVDS_SSC_CFG                     0x3c
#define IT6251_LVDS_RPC_REQ_FIFOFULL            BIT(6)
#define IT6251_LVDS_RPC_REQ_FIFOEMPTY           BIT(7)
#define IT6251_LVDS_PCLK_CNT_LOW                0x57
#define IT6251_LVDS_PCLK_CNT_HIGH               0x58

/*
 * Maximum pixel clock supported by the LVDS RX input stage
 * per IT6251 data sheet Rev 1.0.
 */
#define IT6251_MAX_INPUT_PCLK_KHZ               150000


/*
 * devm-friendly cleanup for i2c_new_ancillary_device(). The kernel does
 * not yet provide devm_i2c_new_ancillary_device(), so wrap the unregister
 * with devm_add_action_or_reset() to get the same lifetime behaviour.
 */
static void it6251_unregister_lvds(void *data)
{
	i2c_unregister_device(data);
}

#define IT6251_AUX_WAIT_LOOPS                   200
#define IT6251_AUX_WAIT_MS                      1
#define IT6251_UNSTABLE_THRESHOLD               10
#define IT6251_POLL_INTERVAL_MS                 200

#ifdef CONFIG_DRM_ITE_IT6251_STATIC_EDID
/*
 * Minimal static EDID advertising a single 1920x1080 @ 60 Hz preferred mode.
 *
 * Used as fallback when the live AUX EDID read fails.
 * Timing parameters (CEA 861 VIC 16 / DMT 0x52):
 *   Pixel clock   : 148.500 MHz
 *   Horizontal    : 1920 active + 88 FP + 44 sync + 148 BP = 2200 total
 *   Vertical      : 1080 active +  4 FP +  5 sync +  36 BP = 1125 total
 *   Sync polarity : +H +V
 *
 * Checksum: 0xF3 (verified: sum of all 128 bytes == 0 mod 256).
 */
static const u8 it6251_edid_1920x1080[EDID_LENGTH] = {
	/* header */
	0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	/* manufacturer "ITE" (0x2264), product 0x6251 LE, S/N 0, week 1, year 2023 */
	0x22, 0x64, 0x51, 0x62, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x21,
	/* EDID version 1.3 */
	0x01, 0x03,
	/* digital input, 53 cm x 30 cm, gamma 2.2, preferred timing in first DTD */
	0x80, 0x35, 0x1E, 0x78, 0x0A,
	/* chromaticity coordinates (generic) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* established timings: none */
	0x00, 0x00, 0x00,
	/* standard timings: all unused */
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	/* DTD 1: 1920x1080 @ 60 Hz, 148.5 MHz pixel clock, +H +V */
	0x02, 0x3A, 0x80, 0x18, 0x71, 0x38, 0x2D, 0x40,
	0x58, 0x2C, 0x45, 0x00, 0x0F, 0x28, 0x21, 0x00,
	0x00, 0x1E,
	/* DTD 2: monitor name descriptor "IT6251" */
	0x00, 0x00, 0x00, 0xFC, 0x00,
	0x49, 0x54, 0x36, 0x32, 0x35, 0x31,
	0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	/* DTD 3: monitor range limits, 50-76 Hz / 30-83 kHz / max 150 MHz */
	0x00, 0x00, 0x00, 0xFD, 0x00,
	0x32, 0x4C, 0x1E, 0x53, 0x0F, 0x0A,
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
	/* DTD 4: unused descriptor */
	0x00, 0x00, 0x00, 0x10, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* extension count, checksum */
	0x00, 0xF3,
};
#endif /* CONFIG_DRM_ITE_IT6251_STATIC_EDID */

struct it6251_bridge {
	struct device           *dev;
	struct i2c_client       *client;
	struct i2c_client       *lvds_client;
	struct regmap           *regmap;
	struct regmap           *lvds_regmap;
	struct regulator        *regulator;
	struct gpio_desc        *reset_gpio;
	struct drm_bridge        bridge;
	struct drm_bridge       *next_bridge;
	/* delayed-work HPD/vidstable poll */
	struct delayed_work      poll_work;
	/* DT-parsed configuration knobs */
	u32                      lvds_data_mapping;
	u8                       color_depth;
	u8                       dp_lanes;
	bool                     dual_pixel;
	bool                     dp_ssc;
	bool                     force_lbr;
	/* runtime state */
	u8                       unstable_count;
	bool                     prev_hpd;
	bool                     enabled;
};

static inline struct it6251_bridge *
bridge_to_it6251(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6251_bridge, bridge);
}

/* ---------------------------------------------------------------------------
 * AUX channel helpers
 */

static int it6251_aux_wait(struct it6251_bridge *it)
{
	unsigned int val;
	int i;

	for (i = 0; i < IT6251_AUX_WAIT_LOOPS; i++) {
		regmap_read(it->regmap, IT6251_AUX_FIRE, &val);
		if (!(val & IT6251_AUX_FIRE_BUSY))
			return 0;
		mdelay(IT6251_AUX_WAIT_MS);
	}
	dev_warn(it->dev, "AUX channel timeout\n");
	return -ETIMEDOUT;
}

static int it6251_read_edid_block(void *data, u8 *buf,
				  unsigned int block, size_t len)
{
	struct it6251_bridge *it = data;
	u8 offset;
	int i, ret = 0;

	/* reset AUX channel before starting EDID read */
	regmap_update_bits(it->regmap, IT6251_DP_SW_RESET,
			   IT6251_DP_SWRST_AUX, IT6251_DP_SWRST_AUX);
	regmap_update_bits(it->regmap, IT6251_DP_SW_RESET,
			   IT6251_DP_SWRST_AUX, 0);
	regmap_update_bits(it->regmap, 0x07, BIT(0), BIT(0));
	regmap_update_bits(it->regmap, 0x0a, BIT(0), BIT(0));

	for (offset = 0; offset < 0x80; offset += 0x10) {
		u8 start_addr = (u8)((block & 1) * 0x80 + offset);

		regmap_write(it->regmap, IT6251_AUX_CTRL,
			     IT6251_AUX_CTRL_PC_EN | IT6251_AUX_CTRL_EDID_EN);
		regmap_write(it->regmap, IT6251_AUX_CTRL,
			     IT6251_AUX_CTRL_PC_EN | IT6251_AUX_CTRL_EDID_FIFO_CLR);
		regmap_write(it->regmap, IT6251_AUX_ADDR0, start_addr);
		regmap_write(it->regmap, IT6251_AUX_ADDR1, block / 256);
		regmap_write(it->regmap, IT6251_AUX_ADDR2, 0xf0);
		regmap_write(it->regmap, IT6251_AUX_FIRE,  IT6251_AUX_FIRE_EDID_RD);
		ret = it6251_aux_wait(it);
		if (ret) {
			dev_warn(it->dev, "EDID AUX timeout block %u off 0x%02x\n",
				 block, offset);
			goto done;
		}
		for (i = 0; i < 16 && (size_t)(offset + i) < len; i++) {
			unsigned int val;

			regmap_read(it->regmap, IT6251_AUX_EDID_FIFO, &val);
			buf[offset + i] = (u8)val;
		}
	}
done:
	regmap_write(it->regmap, IT6251_AUX_CTRL, IT6251_AUX_CTRL_PC_EN);
	return ret;
}

/* ---------------------------------------------------------------------------
 * Stability check
 */

static int it6251_is_stable(struct it6251_bridge *it6251)
{
	unsigned int status, rpclkcnt, clkcnt, refstate, rpcreq;
	u16 hactive, vactive;
	u8 regs[2];
	int ret;

	ret = regmap_read(it6251->regmap, IT6251_SYSTEM_STATUS, &status);
	if (ret)
		return ret;
	dev_dbg(it6251->dev, "System status: 0x%02x\n", status);

	if (!(status & IT6251_SYSTEM_STATUS_RVIDEOSTABLE))
		return -EINVAL;

	ret = regmap_bulk_read(it6251->regmap, IT6251_RPCLK_CNT_LOW, regs, 2);
	if (ret)
		return ret;
	rpclkcnt = (regs[0] & 0xff) | ((regs[1] & 0x0f) << 8);
	dev_dbg(it6251->dev, "RPCLKCnt: %d\n", rpclkcnt);

	ret = regmap_bulk_read(it6251->lvds_regmap, IT6251_LVDS_PCLK_CNT_LOW,
			       regs, 2);
	if (ret)
		return ret;
	clkcnt = (regs[0] & 0xff) | ((regs[1] & 0x0f) << 8);
	dev_dbg(it6251->dev, "LVDS CLKCnt: 0x%02x\n", clkcnt);

	ret = regmap_read(it6251->lvds_regmap, IT6251_REF_STATE, &refstate);
	if (ret)
		return ret;
	dev_dbg(it6251->dev, "Ref link state: 0x%02x\n", refstate);

	ret = regmap_read(it6251->lvds_regmap, IT6251_LVDS_RPC_REQ, &rpcreq);
	if (ret)
		return ret;
	dev_dbg(it6251->dev, "RPC req: 0x%02x\n", rpcreq);

	ret = regmap_bulk_read(it6251->regmap, IT6251_DPHDEW_LOW, regs, 2);
	if (ret)
		return ret;
	hactive = (u16)((regs[0] & 0xff) | ((regs[1] & 0x1f) << 8));
	dev_dbg(it6251->dev, "hactive: %d\n", hactive);

	ret = regmap_bulk_read(it6251->regmap, IT6251_DPVDEW_LOW, regs, 2);
	if (ret)
		return ret;
	vactive = (u16)((regs[0] & 0xff) | ((regs[1] & 0x0f) << 8));
	dev_dbg(it6251->dev, "vactive: %d\n", vactive);

	if ((refstate & IT6251_LINK_STATE_MASK) != 0)
		return -EINVAL;

	if (rpcreq & IT6251_LVDS_RPC_REQ_FIFOFULL) {
		dev_err(it6251->dev, "LVDS RPC FIFO full\n");
		return -EINVAL;
	}

	if (refstate & IT6251_REF_STATE_MUTED)
		return -EINVAL;

	return 0;
}

/* ---------------------------------------------------------------------------
 * Chip init / power-down
 */

/*
 * Build the LVDS RX format register value from DT config.
 * Bit layout: [6]=DESSC_DIS [4]=VESA [1:0]=BPP
 */
static u8 it6251_lvds_fmt_reg(const struct it6251_bridge *it)
{
	u8 val = IT6251_LVDS_FMT_DESSC_DIS;

	if (it->lvds_data_mapping == MEDIA_BUS_FMT_RGB888_1X7X4_SPWG)
		val |= IT6251_LVDS_FMT_VESA;

	switch (it->color_depth) {
	case 6:  val |= IT6251_LVDS_FMT_6BPP;  break;
	case 10: val |= IT6251_LVDS_FMT_10BPP; break;
	case 12: val |= IT6251_LVDS_FMT_12BPP; break;
	default: val |= IT6251_LVDS_FMT_8BPP;  break;
	}
	return val;
}

static int it6251_chip_init(struct it6251_bridge *it6251)
{
	u8 lane_cfg, pclk_cfg, in_cfg, bpp_cfg;
	int i, ret;
	unsigned int reg;

	const struct reg_sequence it6251_reset_reg_sequence[] = {
		{ IT6251_DP_SW_RESET,    0x00 },
		{ IT6251_LVDS_PORT_ADDR, (u8)(it6251->lvds_client->addr << 1) },
		{ IT6251_LVDS_PORT_CTRL, IT6251_LVDS_PORT_CTRL_EN },
	};

	/*
	 * Reset DisplayPort half. Setting bit 2 causes IT6251 to not
	 * respond over I2C, which is considered normal. This write will
	 * report failure but will actually succeed.
	 */
	regmap_write(it6251->regmap, IT6251_DP_SW_RESET, IT6251_DP_SWRST_ALL);

	/* Un-reset and set LVDS sub-address pointer. */
	ret = regmap_multi_reg_write(it6251->regmap, it6251_reset_reg_sequence,
				     ARRAY_SIZE(it6251_reset_reg_sequence));
	if (ret) {
		dev_err(it6251->dev, "failed to configure eDP TX half\n");
		return ret;
	}

	/* LVDS RX -- reset, configure, release. */
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_SW_RESET,
		     IT6251_LVDS_SWRST_ALL);
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_PLL_CFG,
		     IT6251_LVDS_PLL_RESET);
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_PLL_CFG,
		     IT6251_LVDS_PLL_NORMAL);
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_SSC_CFG,  0x08);
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_LINK_CFG, 0x88);
	/* data-mapping and color-depth from DT */
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_FMT,
		     it6251_lvds_fmt_reg(it6251));
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_DCBAL_CFG, 0x04);
	regmap_write(it6251->lvds_regmap, 0x35, 0xe0);
	/* reset pixel clock domain, then release */
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_SW_RESET,
		     IT6251_LVDS_SWRST_PCLK);
	regmap_write(it6251->lvds_regmap, IT6251_LVDS_SW_RESET, 0x00);

	/* Clear and unmask interrupts. */
	regmap_write(it6251->regmap, IT6251_INT_MASK, IT6251_INT_ENABLE_MASK);
	regmap_write(it6251->regmap, 0x0a, 0x00);
	regmap_write(it6251->regmap, 0x0b, 0x00);
	regmap_write(it6251->regmap, 0x06, 0xff);
	regmap_write(it6251->regmap, 0x07, 0xff);
	regmap_write(it6251->regmap, 0x08, 0xff);

	/* DP lane config from DT: lane count, SSC, LBR. */
	lane_cfg = (u8)(it6251->dp_lanes == 4 ? IT6251_DP_LN_CFG_4_LANE :
		        it6251->dp_lanes == 2 ? IT6251_DP_LN_CFG_2_LANE :
						IT6251_DP_LN_CFG_1_LANE);
	if (it6251->dp_ssc)    lane_cfg |= IT6251_DP_LN_CFG_SSC;
	if (it6251->force_lbr) lane_cfg |= IT6251_DP_LN_CFG_LBR;
	regmap_write(it6251->regmap, IT6251_DP_LANE_CFG, lane_cfg);

	regmap_write(it6251->regmap, IT6251_AUX_CTRL, IT6251_AUX_CTRL_PC_EN);
	regmap_write(it6251->regmap, 0x5c, 0xf3);
	regmap_write(it6251->regmap, 0x5f, 0x06);

	/*
	 * pclk/2 mode (reg 0x60 bit1) and dual-pixel input (reg 0x61 bit2)
	 * must be set together for LATU 1920x1080 single-link LVDS: the chip
	 * processes 2 LVDS pixels per internal clock cycle.
	 */
	pclk_cfg = it6251->dual_pixel ? IT6251_DP_PCLK_HALF    : 0;
	in_cfg   = it6251->dual_pixel ? IT6251_DP_IN_DUAL_PIX  : 0;
	regmap_write(it6251->regmap, IT6251_DP_PCLK_CFG, pclk_cfg);
	regmap_write(it6251->regmap, IT6251_DP_IN_CFG,   in_cfg);

	switch (it6251->color_depth) {
	case 6:  bpp_cfg = IT6251_DP_IN_6BPP;  break;
	case 10: bpp_cfg = IT6251_DP_IN_10BPP; break;
	case 12: bpp_cfg = IT6251_DP_IN_12BPP; break;
	default: bpp_cfg = IT6251_DP_IN_8BPP;  break;
	}
	regmap_write(it6251->regmap, IT6251_DP_IN_CFG2, bpp_cfg);

	/* Video / packet configuration (fixed values). */
	regmap_write(it6251->regmap, IT6251_DP_SYNC_CFG,  0x0f);
	regmap_write(it6251->regmap, IT6251_HPD_TIMER0,   0xf5);
	regmap_write(it6251->regmap, IT6251_HPD_TIMER1,   0x4d);
	regmap_write(it6251->regmap, IT6251_HPD_TIMER2,   0x37);
	regmap_write(it6251->regmap, IT6251_DP_FRAME_CFG, 0x03);
	regmap_write(it6251->regmap, IT6251_DP_VID_CFG,   0x45);
	regmap_write(it6251->regmap, IT6251_DP_QUEUE_CFG, 0xa0);
	regmap_write(it6251->regmap, IT6251_DP_PKT_CFG,   0x33);
	regmap_write(it6251->regmap, IT6251_DP_AVI_CFG,   0x00);

	/* AUX seed + trigger. */
	regmap_write(it6251->regmap, IT6251_AUX_CTRL,  0x42);
	regmap_write(it6251->regmap, IT6251_AUX_ADDR0, 0x00);
	regmap_write(it6251->regmap, IT6251_AUX_ADDR1, 0x00);
	regmap_write(it6251->regmap, IT6251_AUX_ADDR2, 0x00);
	regmap_write(it6251->regmap, IT6251_AUX_FIRE,  IT6251_AUX_FIRE_NATIVE_RD);
	regmap_write(it6251->regmap, IT6251_AUX_CTRL,  IT6251_AUX_CTRL_PC_EN);

	/* Swing / pre-emphasis + link training. */
	regmap_write(it6251->regmap, IT6251_DP_VSWING,     0xff);
	regmap_write(it6251->regmap, IT6251_DP_PREEMPH,    0xff);
	regmap_write(it6251->regmap, IT6251_DP_TRAIN_CTRL, 0x04);
	regmap_write(it6251->regmap, IT6251_DP_TRAIN_CTRL, 0x01);

	/* Wait up to 200 ms for link + video stable. */
	for (i = 0; i < 100; i++) {
		ret = regmap_read(it6251->regmap, IT6251_REF_STATE, &reg);
		if (!ret &&
		    (reg & IT6251_LINK_STATE_MASK) == IT6251_LINK_STATE_NORMAL) {
			ret = regmap_read(it6251->regmap,
					  IT6251_SYSTEM_STATUS, &reg);
			if (!ret &&
			    (reg & IT6251_SYSTEM_STATUS_RVIDEOSTABLE))
				break;
		}
		mdelay(2);
	}

	/*
	 * Retrain if the achieved link rate does not match the desired
	 * HBR/LBR setting.
	 */
	ret = regmap_read(it6251->regmap, IT6251_REF_STATE, &reg);
	if (!ret &&
	    (reg & IT6251_LINK_STATE_MASK) == IT6251_LINK_STATE_NORMAL) {
		bool trained_hbr = !!(reg & IT6251_LINK_STATE_RATE_HBR);
		bool want_hbr    = !it6251->force_lbr;

		if (trained_hbr != want_hbr) {
			dev_dbg(it6251->dev, "rate mismatch, re-training\n");
			regmap_write(it6251->regmap, IT6251_DP_TRAIN_CTRL, 0x04);
			regmap_write(it6251->regmap, IT6251_DP_TRAIN_CTRL, 0x01);
		}
	}

	/*
	 * If we could not stabilize, warn but return 0 so the caller can
	 * decide whether to retry.
	 */
	ret = it6251_is_stable(it6251);
	if (ret)
		dev_warn(it6251->dev, "bridge did not stabilize after init\n");

	return ret;
}

/*
 * Full hardware power-down sequence from baremetal reference.
 * Must be called before disabling the power regulator.
 */
static void it6251_hw_power_down(struct it6251_bridge *it)
{
	/* eDP TX: assert software reset, power down lanes. */
	regmap_write(it->regmap, 0x05, 0xff);
	regmap_write(it->regmap, 0x05, 0x3b);
	regmap_write(it->regmap, 0xfd, 0xbc);
	regmap_write(it->regmap, 0xfe, 0x01);

	/* LVDS RX: power down. */
	regmap_write(it->lvds_regmap, 0x05, 0x02);
	regmap_write(it->lvds_regmap, 0x0b, 0x08);
	regmap_write(it->lvds_regmap, 0x3b, 0x40);
	regmap_write(it->lvds_regmap, 0x3c, 0x08);
	regmap_write(it->lvds_regmap, 0x52, 0x00);

	/* eDP TX: power down each lane driver. */
	regmap_write(it->regmap, 0x0c, 0x00);
	regmap_write(it->regmap, 0x58, 0x10);
	regmap_write(it->regmap, 0x59, 0x20);
	regmap_write(it->regmap, 0x5a, 0x0c);
	regmap_write(it->regmap, 0x5b, 0x42);
	regmap_write(it->regmap, 0x5c, 0x02);
	regmap_write(it->regmap, 0x5e, 0x14);
	regmap_write(it->regmap, 0x10, 0x7c);
}

/* ---------------------------------------------------------------------------
 * HPD / video-stable poll worker
 */

static void it6251_poll_work(struct work_struct *work)
{
	struct it6251_bridge *it =
		container_of(work, struct it6251_bridge, poll_work.work);
	unsigned int reg06, stat;
	bool hpd;
	int ret;

	if (!it->enabled)
		return;

	ret = regmap_read(it->regmap, IT6251_INT_STATUS, &reg06);
	if (ret || !reg06)
		goto reschedule;

	/* Acknowledge all pending interrupts. */
	regmap_write(it->regmap, IT6251_INT_STATUS, reg06);

	if (!(reg06 & (IT6251_INT_HPD_IRQ | IT6251_INT_HPD_CHG |
		       IT6251_INT_VIDSTABLE)))
		goto reschedule;

	ret = regmap_read(it->regmap, IT6251_SYSTEM_STATUS, &stat);
	if (ret)
		goto reschedule;

	hpd = !!(stat & IT6251_SYSTEM_STATUS_RHPDSTATUS);

	if ((reg06 & (IT6251_INT_HPD_CHG | IT6251_INT_HPD_IRQ)) && hpd) {
		if (hpd != it->prev_hpd) {
			it->prev_hpd = true;
			drm_bridge_hpd_notify(&it->bridge,
					      connector_status_connected);
		}
		it6251_chip_init(it);
		it->unstable_count = 0;
		goto reschedule;
	}

	if ((reg06 & IT6251_INT_HPD_CHG) && !hpd) {
		it->prev_hpd = false;
		drm_bridge_hpd_notify(&it->bridge,
				      connector_status_disconnected);
		goto reschedule;
	}

	if (reg06 & IT6251_INT_VIDSTABLE) {
		bool stable = !!(stat & IT6251_SYSTEM_STATUS_RVIDEOSTABLE);

		if (stable) {
			if (it->unstable_count > 0)
				it->unstable_count--;
		} else {
			if (it->unstable_count < IT6251_UNSTABLE_THRESHOLD)
				it->unstable_count++;
			if (it->unstable_count >= IT6251_UNSTABLE_THRESHOLD) {
				it6251_chip_init(it);
				it->unstable_count = 0;
			}
		}
	}

reschedule:
	if (it->enabled)
		schedule_delayed_work(&it->poll_work,
				      msecs_to_jiffies(IT6251_POLL_INTERVAL_MS));
}

/* ---------------------------------------------------------------------------
 * Power helpers
 */

static int it6251_power_down(struct it6251_bridge *it6251)
{
	struct device *dev = &it6251->client->dev;
	int ret = 0;

	if (regulator_is_enabled(it6251->regulator)) {
		ret = regulator_disable(it6251->regulator);
		if (ret)
			dev_err(dev, "unable to disable regulator\n");
	}

	return ret;
}

static int it6251_power_up(struct it6251_bridge *it6251)
{
	u8 regs[4];
	int i, ret;

	ret = regulator_enable(it6251->regulator);
	if (ret) {
		dev_err(it6251->dev, "failed to enable power regulator\n");
		return ret;
	}

	/* Sometimes multiple tries are needed after power-on. */
	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(it6251->regmap, IT6251_VENDOR_ID_LOW,
				       regs, 4);
		if (!ret && regs[0] && regs[1] && regs[2] && regs[3]) {
			dev_info(it6251->dev, "found IT6251 [%04x:%04x]\n",
				 (regs[1] << 8) | regs[0],
				 (regs[3] << 8) | regs[2]);
			return 0;
		}
		usleep_range(100000, 200000);
	}

	dev_err(it6251->dev, "unable to read product id\n");
	it6251_power_down(it6251);
	return -EINVAL;
}

/* ---------------------------------------------------------------------------
 * DRM bridge callbacks
 */

static void it6251_bridge_atomic_pre_enable(struct drm_bridge *bridge,
					    struct drm_atomic_state *state)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int ret;

	ret = it6251_power_up(it6251);
	if (ret)
		dev_err(it6251->dev, "power-on failed: %d\n", ret);
}

static void it6251_bridge_atomic_enable(struct drm_bridge *bridge,
					struct drm_atomic_state *state)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = it6251_chip_init(it6251);
		if (!ret)
			goto started;

		/* Re-cycle power and retry if init did not stabilize. */
		it6251_power_down(it6251);
		ret = it6251_power_up(it6251);
		if (ret)
			return;
	}

	dev_err(it6251->dev, "chip did not stabilize after %d retries\n", i);
	return;

started:
	it6251->unstable_count = 0;
	it6251->enabled = true;
	schedule_delayed_work(&it6251->poll_work,
			      msecs_to_jiffies(IT6251_POLL_INTERVAL_MS));
}

static void it6251_bridge_atomic_disable(struct drm_bridge *bridge,
					 struct drm_atomic_state *state)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	it6251->enabled = false;
	cancel_delayed_work_sync(&it6251->poll_work);
	it6251_hw_power_down(it6251);
}

static void it6251_bridge_atomic_post_disable(struct drm_bridge *bridge,
					      struct drm_atomic_state *state)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);

	it6251_power_down(it6251);
}

static int it6251_bridge_attach(struct drm_bridge *bridge,
				struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	struct drm_connector *connector;
	int ret;

	/*
	 * Attach the downstream bridge (dp-connector) first. Always pass
	 * NO_CONNECTOR so the downstream bridge does not create its own
	 * connector; we create the aggregate one below.
	 */
	ret = drm_bridge_attach(encoder, it6251->next_bridge, bridge,
				flags | DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	if (flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR)
		return 0;

	/*
	 * Build a single drm_connector that aggregates properties from the
	 * whole bridge chain.
	 */
	connector = drm_bridge_connector_init(bridge->dev, encoder);
	if (IS_ERR(connector))
		return dev_err_probe(it6251->dev, PTR_ERR(connector),
				     "failed to init bridge connector\n");

	drm_connector_attach_encoder(connector, encoder);

	return 0;
}

static enum drm_connector_status
it6251_bridge_detect(struct drm_bridge *bridge,
		     struct drm_connector *connector)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	unsigned int status;

	if (regmap_read(it6251->regmap, IT6251_SYSTEM_STATUS, &status))
		return connector_status_unknown;

	return (status & IT6251_SYSTEM_STATUS_RHPDSTATUS) ?
		connector_status_connected : connector_status_disconnected;
}

static const struct drm_edid *
it6251_bridge_edid_read(struct drm_bridge *bridge,
			struct drm_connector *connector)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	const struct drm_edid *edid;

	/* Attempt a live AUX EDID read first. */
	edid = drm_edid_read_custom(connector, it6251_read_edid_block, it6251);
	if (edid)
		return edid;

#ifdef CONFIG_DRM_ITE_IT6251_STATIC_EDID
	dev_warn(it6251->dev,
		 "AUX EDID read failed, using static 1920x1080 fallback\n");
	return drm_edid_alloc(it6251_edid_1920x1080,
			      sizeof(it6251_edid_1920x1080));
#else
	dev_warn(it6251->dev, "AUX EDID read failed\n");
	return NULL;
#endif
}

static enum drm_mode_status
it6251_bridge_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	unsigned long long bw_req_kbps, bw_avail_kbps;

	if (mode->clock > IT6251_MAX_INPUT_PCLK_KHZ)
		return MODE_CLOCK_HIGH;

	/* Check DP bandwidth: 24-bit RGB at pixel clock vs available lane BW. */
	bw_req_kbps   = (unsigned long long)mode->clock * 24;
	bw_avail_kbps = (unsigned long long)it6251->dp_lanes *
			(it6251->force_lbr ? 1296000ULL : 2160000ULL);
	if (bw_req_kbps > bw_avail_kbps)
		return MODE_BAD;

	return MODE_OK;
}

static u32 *
it6251_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
					struct drm_bridge_state *bridge_state,
					struct drm_crtc_state *crtc_state,
					struct drm_connector_state *conn_state,
					u32 output_fmt,
					unsigned int *num_input_fmts)
{
	struct it6251_bridge *it6251 = bridge_to_it6251(bridge);
	u32 *input_fmts;

	*num_input_fmts = 0;

	if (it6251->lvds_data_mapping != MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA &&
	    it6251->lvds_data_mapping != MEDIA_BUS_FMT_RGB888_1X7X4_SPWG)
		return NULL;

	input_fmts = kmalloc(sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	input_fmts[0]   = it6251->lvds_data_mapping;
	*num_input_fmts = 1;
	return input_fmts;
}

static const struct drm_bridge_funcs it6251_bridge_funcs = {
	.atomic_reset              = drm_atomic_helper_bridge_reset,
	.atomic_duplicate_state    = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state      = drm_atomic_helper_bridge_destroy_state,
	.attach                    = it6251_bridge_attach,
	.mode_valid                = it6251_bridge_mode_valid,
	.atomic_pre_enable         = it6251_bridge_atomic_pre_enable,
	.atomic_enable             = it6251_bridge_atomic_enable,
	.atomic_disable            = it6251_bridge_atomic_disable,
	.atomic_post_disable       = it6251_bridge_atomic_post_disable,
	.detect                    = it6251_bridge_detect,
	.edid_read                 = it6251_bridge_edid_read,
	.atomic_get_input_bus_fmts = it6251_bridge_atomic_get_input_bus_fmts,
};

/* ---------------------------------------------------------------------------
 * Device-tree parsing
 */

static int it6251_parse_dt(struct it6251_bridge *it)
{
	struct device_node *np = it->dev->of_node;
	u32 val;
	int ret;

	ret = drm_of_lvds_get_data_mapping(np);
	if (ret < 0) {
		dev_err(it->dev,
			"%pOF: invalid or missing data-mapping: %d\n", np, ret);
		return ret;
	}
	it->lvds_data_mapping = (u32)ret;

	it->color_depth = 8;
	if (!of_property_read_u32(np, "ite,color-depth", &val)) {
		if (val == 6 || val == 8 || val == 10 || val == 12)
			it->color_depth = (u8)val;
		else
			dev_warn(it->dev,
				 "invalid ite,color-depth %u, defaulting to 8\n",
				 val);
	}

	it->dp_lanes = 4;
	if (!of_property_read_u32(np, "ite,dp-lanes", &val)) {
		if (val == 1 || val == 2 || val == 4)
			it->dp_lanes = (u8)val;
		else
			dev_warn(it->dev,
				 "invalid ite,dp-lanes %u, defaulting to 4\n",
				 val);
	}

	/*
	 * Default is dual-pixel input, required for LATU 1920x1080.
	 * Add 'ite,single-pixel-input' to DT node to override.
	 */
	it->dual_pixel = !of_property_read_bool(np, "ite,single-pixel-input");
	it->dp_ssc     =  of_property_read_bool(np, "ite,dp-spread-spectrum");
	it->force_lbr  =  of_property_read_bool(np, "ite,force-lbr");

	return 0;
}

/* ---------------------------------------------------------------------------
 * I2C driver
 */

static const struct regmap_config it6251_regmap_config = {
	.reg_bits     = 8,
	.val_bits     = 8,
	.max_register = 0xff,
	.cache_type   = REGCACHE_NONE,
};

static int it6251_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct it6251_bridge *it6251;
	int ret;

	it6251 = devm_drm_bridge_alloc(dev, struct it6251_bridge, bridge,
				       &it6251_bridge_funcs);
	if (IS_ERR(it6251))
		return PTR_ERR(it6251);

	it6251->dev    = dev;
	it6251->client = client;

	it6251->regmap = devm_regmap_init_i2c(client, &it6251_regmap_config);
	if (IS_ERR(it6251->regmap))
		return dev_err_probe(dev, PTR_ERR(it6251->regmap),
				     "failed to init eDP TX regmap\n");

	it6251->regulator = devm_regulator_get(dev, "power");
	if (IS_ERR(it6251->regulator))
		return dev_err_probe(dev, PTR_ERR(it6251->regulator),
				     "failed to get power regulator\n");

	/*
	 * Retrieve the downstream bridge (dp-connector node) via port@1.
	 * Returns -EPROBE_DEFER if dp-connector has not probed yet.
	 */
	it6251->next_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 1, 0);
	if (IS_ERR(it6251->next_bridge))
		return dev_err_probe(dev, PTR_ERR(it6251->next_bridge),
				     "failed to get downstream bridge\n");

	/*
	 * The LVDS RX half of the chip is accessed at a secondary I2C address.
	 * reg-names = "bridge", "lvds" with reg = <0x4c>, <0x4d>.
	 */
	it6251->lvds_client = i2c_new_ancillary_device(client, "lvds", 0x4d);
	if (IS_ERR(it6251->lvds_client))
		return dev_err_probe(dev, PTR_ERR(it6251->lvds_client),
				     "failed to create LVDS RX I2C device\n");

	ret = devm_add_action_or_reset(dev, it6251_unregister_lvds,
				       it6251->lvds_client);
	if (ret)
		return ret;

	it6251->lvds_regmap = devm_regmap_init_i2c(it6251->lvds_client,
						   &it6251_regmap_config);
	if (IS_ERR(it6251->lvds_regmap))
		return dev_err_probe(dev, PTR_ERR(it6251->lvds_regmap),
				     "failed to init LVDS RX regmap\n");

	ret = it6251_parse_dt(it6251);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&it6251->poll_work, it6251_poll_work);
	i2c_set_clientdata(client, it6251);

	it6251->bridge.of_node = dev->of_node;
	it6251->bridge.type    = DRM_MODE_CONNECTOR_DisplayPort;
	it6251->bridge.ops     = DRM_BRIDGE_OP_DETECT |
				 DRM_BRIDGE_OP_EDID   |
				 DRM_BRIDGE_OP_HPD;

	return devm_drm_bridge_add(dev, &it6251->bridge);
}

static void it6251_remove(struct i2c_client *client)
{
	/* All resources are managed with devm; nothing to do here. */
}

/* ---------------------------------------------------------------------------
 * PM
 */

static int it6251_pm_suspend(struct device *dev)
{
	struct it6251_bridge *it6251 = i2c_get_clientdata(to_i2c_client(dev));

	it6251->enabled = false;
	cancel_delayed_work_sync(&it6251->poll_work);

	if (regulator_is_enabled(it6251->regulator)) {
		it6251_hw_power_down(it6251);
		return it6251_power_down(it6251);
	}
	return 0;
}

static int it6251_pm_resume(struct device *dev)
{
	struct it6251_bridge *it6251 = i2c_get_clientdata(to_i2c_client(dev));

	return it6251_power_up(it6251);
}

static const struct dev_pm_ops it6251_pm_ops = {
	.suspend = it6251_pm_suspend,
	.resume  = it6251_pm_resume,
	.restore = it6251_pm_resume,
};

/* ---------------------------------------------------------------------------
 * Module boilerplate
 */

static const struct i2c_device_id it6251_i2c_ids[] = {
	{ "it6251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it6251_i2c_ids);

static const struct of_device_id it6251_of_match[] = {
	{ .compatible = "ite,it6251", },
	{ }
};
MODULE_DEVICE_TABLE(of, it6251_of_match);

static struct i2c_driver it6251_driver = {
	.driver = {
		.name           = "it6251",
		.pm             = &it6251_pm_ops,
		.of_match_table = it6251_of_match,
	},
	.probe    = it6251_probe,
	.remove   = it6251_remove,
	.id_table = it6251_i2c_ids,
};
module_i2c_driver(it6251_driver);

MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("ITE Tech IT6251 LVDS-to-eDP bridge driver");
MODULE_LICENSE("GPL");
