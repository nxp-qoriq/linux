/*
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier: GPL-2.0+
 *
 */
#ifndef _IMX_HDP_H_
#define _IMX_HDP_H_

#include <linux/regmap.h>
#include <linux/mutex.h>
#include <drm/drm_of.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#ifndef CONFIG_ARCH_LAYERSCAPE
#include <soc/imx8/sc/sci.h>
#endif

#include <drm/drm_dp_helper.h>
#include "../../../../mxc/hdp/all.h"

/* For testing hdp firmware define DEBUG_FW_LOAD */
#undef DEBUG_FW_LOAD
#define PLL_1188MHZ (1188000000)
#define PLL_800MHZ (800000000)
#define PLL_675MHZ (675000000)

#define HDP_TX_SS_LIS_BASE   0x0000
#define HDP_TX_SS_CSR_BASE   0x1000
#define HDP_TX_SS_GPIO_BASE  0x2000
#define HDP_TX_SS_CTRL0_BASE 0x8000

#define CSR_PIXEL_LINK_MUX_CTL		0x00
#define PL_MUX_CTL_VCP_OFFSET		5
#define PL_MUX_CTL_HCP_OFFSET		4
#define PL_MUX_CTL_PL_MUX_OFFSET	2
#define PL_MUX_CTL_PL_SEL_OFFSET	0

#define CSR_PIXEL_LINK_MUX_STATUS	0x04
#define PL_MUX_STATUS_PL1_INT_OFFSET 18
#define PL_MUX_STATUS_PL1_ADD_OFFSET 16
#define PL_MUX_STATUS_PL1_TYP_OFFSET 11
#define PL_MUX_STATUS_PL0_INT_OFFSET 9
#define PL_MUX_STATUS_PL0_ADD_OFFSET 7
#define PL_MUX_STATUS_PL0_TYP_OFFSET 2
#define PL_MUX_STATUS_PL_DLY_OFFSET 0

#define CSR_HDP_TX_CTRL_CTRL0		0x08
#define CSR_HDP_TX_CTRL_CTRL1		0x0c

#define HOTPLUG_DEBOUNCE_MS		200

#define VIC_MODE_96_50Hz 96
#define VIC_MODE_97_60Hz 97

#ifdef CONFIG_ARCH_LAYERSCAPE
#define EDP_PHY_RESET	0x230
#define CSR_PLLDIG_PLLDV 0x28
#define CSR_PLLDIG_PLLFM 0x2c
#define CSR_PLLDIG_PLLFD 0x30
#define CSR_PLLDIG_PLLCAL1 0x38
#define CSR_PLLDIG_PLLCAL2 0x3c
#endif

/**
 * imx_hdp_call - Calls a struct imx hdp_operations operation on
 *	an entity
 *
 * @entity: entity where the @operation will be called
 * @operation: type of the operation. Should be the name of a member of
 *	struct &media_entity_operations.
 *
 * This helper function will check if @operation is not %NULL. On such case,
 * it will issue a call to @operation\(@args\).
 */

#define imx_hdp_call(hdp, operation, args...)			\
	(!(hdp) ? -ENODEV : (((hdp)->ops && (hdp)->ops->operation) ?	\
	 (hdp)->ops->operation(args) : -ENOIOCTLCMD))

#define clks_to_imx_hdp(env) \
	container_of(env, struct imx_hdp, clks)

#define state_to_imx_hdp(env) \
	container_of(env, struct imx_hdp, state)

struct hdp_clks;

struct hdp_ops {
	void (*fw_load)(state_struct *state);
	int (*fw_init)(state_struct *state);
	int (*phy_init)(state_struct *state, const struct drm_display_mode *mode,
			int format, int color_depth);
	void (*mode_set)(state_struct *state, const struct drm_display_mode *mode,
			 int format, int color_depth, int max_link);
	int (*get_edid_block)(void *data, u8 *buf, u32 block, size_t len);
	int (*get_hpd_state)(state_struct *state, u8 *hpd);
#ifndef CONFIG_ARCH_LAYERSCAPE
	void (*phy_reset)(sc_ipc_t ipcHndl, struct hdp_mem *mem, u8 reset);
#else
	void (*phy_reset)(uint32_t ipcHndl, struct hdp_mem *mem, u8 reset);
#endif
	int (*pixel_link_validate)(state_struct *state);
	int (*pixel_link_invalidate)(state_struct *state);
	int (*pixel_link_sync_ctrl_enable)(state_struct *state);
	int (*pixel_link_sync_ctrl_disable)(state_struct *state);
	void (*pixel_link_mux)(state_struct *state,
			       const struct drm_display_mode *mode);
	void (*pixel_engine_reset)(state_struct *state);

	int (*clock_init)(struct hdp_clks *clks);
	int (*ipg_clock_enable)(struct hdp_clks *clks);
	void (*ipg_clock_disable)(struct hdp_clks *clks);
	void (*ipg_clock_set_rate)(struct hdp_clks *clks);
	int (*pixel_clock_enable)(struct hdp_clks *clks);
	void (*pixel_clock_disable)(struct hdp_clks *clks);
	void (*pixel_clock_set_rate)(struct hdp_clks *clks);
	int (*pixel_clock_range)(struct drm_display_mode *mode);
};

struct hdp_devtype {
	struct hdp_ops *ops;
	struct hdp_rw_func *rw;
	u32 connector_type;
};

struct hdp_video {
	u32 bpp;
	u32 format;
	u32 lanes;
	u32 color_type; /* bt */
	u32 color_depth;  /* bpc */
	struct drm_display_mode cur_mode;
	struct drm_display_mode pre_mode;
	void __iomem *regs_base;
};

struct hdp_audio {
	u32 interface;  /* I2S SPDIF  */
	u32 freq;
	u32 nlanes;
	u32 nChannels;
	u32 sample_width;
	u32 sample_rate;
	u32 audio_cts;
	u32 audio_n;
	bool audio_enable;
	spinlock_t audio_lock;
	struct mutex audio_mutex;
	void __iomem *regs_base;
};

struct hdp_hdcp {
	void __iomem *regs_base;
};

struct hdp_phy {
	u32 index;
	u32 number;
	bool enabled;
	struct phy *phy;
	void __iomem *regs_base;
};

struct hdp_clks {
	struct clk *av_pll;
	struct clk *dig_pll;
	struct clk *clk_ipg;
	struct clk *clk_core;
	struct clk *clk_pxl;
	struct clk *clk_pxl_mux;
	struct clk *clk_pxl_link;

	struct clk *clk_hdp;
	struct clk *clk_phy;
	struct clk *clk_apb;

	struct clk *clk_lis;
	struct clk *clk_msi;
	struct clk *clk_lpcg;
	struct clk *clk_even;
	struct clk *clk_dbl;
	struct clk *clk_vif;
	struct clk *clk_apb_csr;
	struct clk *clk_apb_ctrl;
	struct clk *av_pll_div;
	struct clk *dig_pll_div;
};

enum hdp_tx_irq {
	HPD_IRQ_IN,
	HPD_IRQ_OUT,
	HPD_IRQ_NUM,
};

struct imx_hdp {
	struct device *dev;
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_bridge bridge;

	struct edid *edid;
	char cable_state;
	char fw_running;

	struct hdp_mem mem;

	u8 is_edp;
	u8 is_digpll_dp_pclock;
	u8 no_edid;
#ifdef CONFIG_ARCH_LAYERSCAPE
	u8 is_hpd_irq;
	int num_res;
#endif
	u32 lane_mapping;
	u32 edp_link_rate;
	u32 edp_num_lanes;

	struct mutex mutex;		/* for state below and previous_mode */
	enum drm_connector_force force;	/* mutex-protected force state */

	struct hdp_video video;

	struct drm_dp_aux aux;
	struct mutex aux_mutex;

	struct drm_dp_link dp_link;
	S_LINK_STAT lkstat;
	ENUM_AFE_LINK_RATE link_rate;
#ifndef CONFIG_ARCH_LAYERSCAPE
	sc_ipc_t ipcHndl;
#else
	uint32_t ipcHndl;
#endif
	u32 mu_id;
	u32 dual_mode;
	struct hdp_ops *ops;
	struct hdp_rw_func *rw;
	struct hdp_clks clks;
	state_struct state;
	int vic;
	int irq[HPD_IRQ_NUM];
	struct delayed_work hotplug_work;

	int bpc;
	VIC_PXL_ENCODING_FORMAT format;
};


#endif
