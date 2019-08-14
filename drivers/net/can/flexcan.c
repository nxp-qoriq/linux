/*
 * flexcan.c - FLEXCAN CAN controller driver
 *
 * Copyright (c) 2005-2006 Varma Electronics Oy
 * Copyright (c) 2009 Sascha Hauer, Pengutronix
 * Copyright (c) 2010-2017 Pengutronix, Marc Kleine-Budde <kernel@pengutronix.de>
 * Copyright (c) 2014 David Jander, Protonic Holland
 *
 * Based on code originally by Andrey Volkov <avolkov@varma-el.com>
 *
 * LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/led.h>
#include <linux/can/rx-offload.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>

#define DRV_NAME			"flexcan"

/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT		(8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS		BIT(31)
#define FLEXCAN_MCR_FRZ			BIT(30)
#define FLEXCAN_MCR_FEN			BIT(29)
#define FLEXCAN_MCR_HALT		BIT(28)
#define FLEXCAN_MCR_NOT_RDY		BIT(27)
#define FLEXCAN_MCR_WAK_MSK		BIT(26)
#define FLEXCAN_MCR_SOFTRST		BIT(25)
#define FLEXCAN_MCR_FRZ_ACK		BIT(24)
#define FLEXCAN_MCR_SUPV		BIT(23)
#define FLEXCAN_MCR_SLF_WAK		BIT(22)
#define FLEXCAN_MCR_WRN_EN		BIT(21)
#define FLEXCAN_MCR_LPM_ACK		BIT(20)
#define FLEXCAN_MCR_WAK_SRC		BIT(19)
#define FLEXCAN_MCR_DOZE		BIT(18)
#define FLEXCAN_MCR_SRX_DIS		BIT(17)
#define FLEXCAN_MCR_IRMQ		BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
#define FLEXCAN_MCR_FDEN		BIT(11)
/* MCR_MAXMB: maximum used MBs is MAXMB + 1 */
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0x7f)
#define FLEXCAN_MCR_IDAM_A		(0x0 << 8)
#define FLEXCAN_MCR_IDAM_B		(0x1 << 8)
#define FLEXCAN_MCR_IDAM_C		(0x2 << 8)
#define FLEXCAN_MCR_IDAM_D		(0x3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)		(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK		BIT(15)
#define FLEXCAN_CTRL_ERR_MSK		BIT(14)
#define FLEXCAN_CTRL_CLK_SRC		BIT(13)
#define FLEXCAN_CTRL_LPB		BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK		BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK		BIT(10)
#define FLEXCAN_CTRL_SMP		BIT(7)
#define FLEXCAN_CTRL_BOFF_REC		BIT(6)
#define FLEXCAN_CTRL_TSYN		BIT(5)
#define FLEXCAN_CTRL_LBUF		BIT(4)
#define FLEXCAN_CTRL_LOM		BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)		((x) & 0x07)
#define FLEXCAN_CTRL_ERR_BUS		(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
	(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | \
	 FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL \
	(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN control register 2 (CTRL2) bits */
#define FLEXCAN_CTRL2_ECRWRE		BIT(29)
#define FLEXCAN_CTRL2_WRMFRZ		BIT(28)
#define FLEXCAN_CTRL2_RFFN(x)		(((x) & 0x0f) << 24)
#define FLEXCAN_CTRL2_TASD(x)		(((x) & 0x1f) << 19)
#define FLEXCAN_CTRL2_MRP		BIT(18)
#define FLEXCAN_CTRL2_RRS		BIT(17)
#define FLEXCAN_CTRL2_EACEN		BIT(16)

/* FLEXCAN memory error control register (MECR) bits */
#define FLEXCAN_MECR_ECRWRDIS		BIT(31)
#define FLEXCAN_MECR_HANCEI_MSK		BIT(19)
#define FLEXCAN_MECR_FANCEI_MSK		BIT(18)
#define FLEXCAN_MECR_CEI_MSK		BIT(16)
#define FLEXCAN_MECR_HAERRIE		BIT(15)
#define FLEXCAN_MECR_FAERRIE		BIT(14)
#define FLEXCAN_MECR_EXTERRIE		BIT(13)
#define FLEXCAN_MECR_RERRDIS		BIT(9)
#define FLEXCAN_MECR_ECCDIS		BIT(8)
#define FLEXCAN_MECR_NCEFAFRZ		BIT(7)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT		BIT(17)
#define FLEXCAN_ESR_RWRN_INT		BIT(16)
#define FLEXCAN_ESR_BIT1_ERR		BIT(15)
#define FLEXCAN_ESR_BIT0_ERR		BIT(14)
#define FLEXCAN_ESR_ACK_ERR		BIT(13)
#define FLEXCAN_ESR_CRC_ERR		BIT(12)
#define FLEXCAN_ESR_FRM_ERR		BIT(11)
#define FLEXCAN_ESR_STF_ERR		BIT(10)
#define FLEXCAN_ESR_TX_WRN		BIT(9)
#define FLEXCAN_ESR_RX_WRN		BIT(8)
#define FLEXCAN_ESR_IDLE		BIT(7)
#define FLEXCAN_ESR_TXRX		BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT	(4)
#define FLEXCAN_ESR_FLT_CONF_MASK	(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE	(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT		BIT(2)
#define FLEXCAN_ESR_ERR_INT		BIT(1)
#define FLEXCAN_ESR_WAK_INT		BIT(0)
#define FLEXCAN_ESR_ERR_BUS \
	(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | \
	 FLEXCAN_ESR_ACK_ERR | FLEXCAN_ESR_CRC_ERR | \
	 FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL \
	(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | \
	 FLEXCAN_ESR_BOFF_INT | FLEXCAN_ESR_ERR_INT | \
	 FLEXCAN_ESR_WAK_INT)

/* FLEXCAN CAN Bit Timing Register (CBT) bits */
#define FLEXCAN_CBT_BTF			BIT(31)
#define FLEXCAN_CBT_EPRESDIV(x)		(((x) & 0x3ff) << 21)
#define FLEXCAN_CBT_ERJW(x)		(((x) & 0x0f) << 16)
#define FLEXCAN_CBT_EPROPSEG(x)		(((x) & 0x3f) << 10)
#define FLEXCAN_CBT_EPSEG1(x)		(((x) & 0x1f) << 5)
#define FLEXCAN_CBT_EPSEG2(x)		((x) & 0x1f)

/* FLEXCAN CAN FD control Register (FDCTRL) bits */
#define FLEXCAN_FDCTRL_FDRATE		BIT(31)
#define FLEXCAN_FDCTRL_MBDSR1(x)	(((x) & 0x03) << 19)
#define FLEXCAN_FDCTRL_MBDSR0(x)	(((x) & 0x03) << 16)
#define FLEXCAN_FDCTRL_TDCEN		BIT(15)
#define FLEXCAN_FDCTRL_TDCFAIL		BIT(14)
#define FLEXCAN_FDCTRL_TDCOFF(x)	(((x) & 0x1f) << 8)

/* FLEXCAN CAN FD Bit Timing Register (FDCBT) bits */
#define FLEXCAN_FDCBT_FPRESDIV(x)	(((x) & 0x3ff) << 20)
#define FLEXCAN_FDCBT_FRJW(x)		(((x) & 0x03) << 16)
#define FLEXCAN_FDCBT_FPROPSEG(x)	(((x) & 0x1f) << 10)
#define FLEXCAN_FDCBT_FPSEG1(x)		(((x) & 0x07) << 5)
#define FLEXCAN_FDCBT_FPSEG2(x)		((x) & 0x07)

/* FLEXCAN interrupt flag register (IFLAG) bits */
/* Errata ERR005829 step7: Reserve first valid MB */
#define FLEXCAN_TX_MB_RESERVED_OFF_FIFO	8
#define FLEXCAN_TX_MB_OFF_FIFO		9
#define FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP	0
#define FLEXCAN_TX_MB_OFF_TIMESTAMP		1
#define FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST	(FLEXCAN_TX_MB_OFF_TIMESTAMP + 1)
#define FLEXCAN_IFLAG_MB(x)		BIT((x) & 0x3f)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CODE_MASK		(0xf << 24)
#define FLEXCAN_MB_CODE_RX_BUSY_BIT	(0x1 << 24)
#define FLEXCAN_MB_CODE_RX_INACTIVE	(0x0 << 24)
#define FLEXCAN_MB_CODE_RX_EMPTY	(0x4 << 24)
#define FLEXCAN_MB_CODE_RX_FULL		(0x2 << 24)
#define FLEXCAN_MB_CODE_RX_OVERRUN	(0x6 << 24)
#define FLEXCAN_MB_CODE_RX_RANSWER	(0xa << 24)

#define FLEXCAN_MB_CODE_TX_INACTIVE	(0x8 << 24)
#define FLEXCAN_MB_CODE_TX_ABORT	(0x9 << 24)
#define FLEXCAN_MB_CODE_TX_DATA		(0xc << 24)
#define FLEXCAN_MB_CODE_TX_TANSWER	(0xe << 24)

#define FLEXCAN_MB_CNT_EDL		BIT(31)
#define FLEXCAN_MB_CNT_BRS		BIT(30)
#define FLEXCAN_MB_CNT_ESI		BIT(29)
#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_TIMEOUT_US		(250)

/* FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch- [TR]WRN_INT IRQ Err Memory err RTR rece-     FD Mode
 *                                Filter? connected?  Passive detection  ption in MB   Supported?
 *   MX25  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX28  FlexCAN2  03.00.04.00    yes       yes        no       no        no           no
 *   MX35  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX53  FlexCAN2  03.00.00.00    yes        no        no       no        no           no
 *   MX6s  FlexCAN3  10.00.12.00    yes       yes        no       no       yes           no
 *   VF610 FlexCAN3  ?               no       yes        no      yes       yes?          no
 * LS1021A FlexCAN2  03.00.04.00     no       yes        no       no       yes           no
 * LX2160A FlexCAN3  03.00.23.00     no       yes        no       no       yes          yes
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_QUIRK_BROKEN_WERR_STATE	BIT(1) /* [TR]WRN_INT not connected */
#define FLEXCAN_QUIRK_DISABLE_RXFG	BIT(2) /* Disable RX FIFO Global mask */
#define FLEXCAN_QUIRK_ENABLE_EACEN_RRS	BIT(3) /* Enable EACEN and RRS bit in ctrl2 */
#define FLEXCAN_QUIRK_DISABLE_MECR	BIT(4) /* Disable Memory error detection */
#define FLEXCAN_QUIRK_USE_OFF_TIMESTAMP	BIT(5) /* Use timestamp based offloading */
#define FLEXCAN_QUIRK_BROKEN_PERR_STATE	BIT(6) /* No interrupt for error passive */
#define FLEXCAN_QUIRK_SETUP_STOP_MODE	BIT(8) /* Setup stop mode to support wakeup */
#define FLEXCAN_QUIRK_USE_FD		BIT(9)

/* Structure of the message buffer */
struct flexcan_mb {
	u32 can_ctrl;
	u32 can_id;
	u32 data[];
};

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 */
	u32 timer;		/* 0x08 */
	u32 _reserved1;		/* 0x0c */
	u32 rxgmask;		/* 0x10 */
	u32 rx14mask;		/* 0x14 */
	u32 rx15mask;		/* 0x18 */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	union {			/* 0x34 */
		u32 gfwr_mx28;	/* MX28, MX53 */
		u32 ctrl2;	/* MX6, VF610 */
	};
	u32 esr2;		/* 0x38 */
	u32 imeur;		/* 0x3c */
	u32 lrfr;		/* 0x40 */
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 cbt;		/* 0x50 */
	u32 _reserved3[11];	/* 0x54 */
	u8 mb[2][512];		/* 0x80 */
	/* FIFO-mode:
	 *			MB
	 * 0x080...0x08f	0	RX message buffer
	 * 0x090...0x0df	1-5	reserverd
	 * 0x0e0...0x0ff	6-7	8 entry ID table
	 *				(mx25, mx28, mx35, mx53)
	 * 0x0e0...0x2df	6-7..37	8..128 entry ID table
	 *				size conf'ed via ctrl2::RFFN
	 *				(mx6, vf610)
	 */
	u32 _reserved4[256];	/* 0x480 */
	u32 rximr[64];		/* 0x880 */
	u32 _reserved5[24];	/* 0x980 */
	u32 gfwr_mx6;		/* 0x9e0 - MX6 */
	u32 _reserved6[63];	/* 0x9e4 */
	u32 mecr;		/* 0xae0 */
	u32 erriar;		/* 0xae4 */
	u32 erridpr;		/* 0xae8 */
	u32 errippr;		/* 0xaec */
	u32 rerrar;		/* 0xaf0 */
	u32 rerrdr;		/* 0xaf4 */
	u32 rerrsynr;		/* 0xaf8 */
	u32 errsr;		/* 0xafc */
	u32 _reserved7[64];	/* 0xb00 */
	u32 fdctrl;		/* 0xc00 */
	u32 fdcbt;		/* 0xc04 */
	u32 fdcrc;		/* 0xc08 */
};

struct flexcan_devtype_data {
	u32 quirks;		/* quirks needed for different IP cores */
};

struct flexcan_stop_mode {
	struct regmap *gpr;
	u8 req_gpr;
	u8 req_bit;
	u8 ack_gpr;
	u8 ack_bit;
};

struct flexcan_priv {
	struct can_priv can;
	struct can_rx_offload offload;

	struct flexcan_regs __iomem *regs;
	struct flexcan_mb __iomem *tx_mb;
	struct flexcan_mb __iomem *tx_mb_reserved;
	u8 tx_mb_idx;
	u8 mb_count;
	u8 mb_size;
	u32 reg_ctrl_default;
	u64 reg_imask_default;

	struct clk *clk_ipg;
	struct clk *clk_per;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;
	struct flexcan_stop_mode stm;

	/* Read and Write APIs */
	u32 (*read)(void __iomem *addr);
	void (*write)(u32 val, void __iomem *addr);
};

static const struct flexcan_devtype_data fsl_p1010_devtype_data = {
	.quirks = FLEXCAN_QUIRK_BROKEN_WERR_STATE |
		FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_imx28_devtype_data = {
	.quirks = FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_imx6q_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_USE_OFF_TIMESTAMP | FLEXCAN_QUIRK_BROKEN_PERR_STATE |
		FLEXCAN_QUIRK_SETUP_STOP_MODE,
};

static const struct flexcan_devtype_data fsl_vf610_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_DISABLE_MECR | FLEXCAN_QUIRK_USE_OFF_TIMESTAMP |
		FLEXCAN_QUIRK_BROKEN_PERR_STATE,
};

static const struct flexcan_devtype_data fsl_ls1021a_r2_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_DISABLE_MECR | FLEXCAN_QUIRK_USE_OFF_TIMESTAMP,
};

static const struct flexcan_devtype_data fsl_lx2160a_r1_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_DISABLE_MECR | FLEXCAN_QUIRK_BROKEN_PERR_STATE |
		FLEXCAN_QUIRK_USE_OFF_TIMESTAMP | FLEXCAN_QUIRK_USE_FD,
};

static const struct can_bittiming_const flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const extended_flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 96,
	.tseg2_min = 2,
	.tseg2_max = 32,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static const struct can_bittiming_const flexcan_data_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 39,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

/* FlexCAN module is essentially modelled as a little-endian IP in most
 * SoCs, i.e the registers as well as the message buffer areas are
 * implemented in a little-endian fashion.
 *
 * However there are some SoCs (e.g. LS1021A) which implement the FlexCAN
 * module in a big-endian fashion (i.e the registers as well as the
 * message buffer areas are implemented in a big-endian way).
 *
 * In addition, the FlexCAN module can be found on SoCs having ARM or
 * PPC cores. So, we need to abstract off the register read/write
 * functions, ensuring that these cater to all the combinations of module
 * endianness and underlying CPU endianness.
 */
static inline u32 flexcan_read_be(void __iomem *addr)
{
	return ioread32be(addr);
}

static inline void flexcan_write_be(u32 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static inline u32 flexcan_read_le(void __iomem *addr)
{
	return ioread32(addr);
}

static inline void flexcan_write_le(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static struct flexcan_mb __iomem *flexcan_get_mb(const struct flexcan_priv *priv,
						 u8 mb_index)
{
	u8 bank_size;
	bool bank;

	if (WARN_ON(mb_index >= priv->mb_count))
		return NULL;

	bank_size = sizeof(priv->regs->mb[0]) / priv->mb_size;

	bank = mb_index >= bank_size;
	if (bank)
		mb_index -= bank_size;

	return (struct flexcan_mb __iomem *)
		(&priv->regs->mb[bank][priv->mb_size * mb_index]);
}

static void flexcan_enable_wakeup_irq(struct flexcan_priv *priv, bool enable)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr;

	reg_mcr = priv->read(&regs->mcr);

	if (enable)
		reg_mcr |= FLEXCAN_MCR_WAK_MSK;
	else
		reg_mcr &= ~FLEXCAN_MCR_WAK_MSK;

	priv->write(reg_mcr, &regs->mcr);
}

static inline void flexcan_enter_stop_mode(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr;

	reg_mcr = priv->read(&regs->mcr);
	reg_mcr |= FLEXCAN_MCR_SLF_WAK;
	priv->write(reg_mcr, &regs->mcr);

	/* enable stop request */
	regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			   1 << priv->stm.req_bit, 1 << priv->stm.req_bit);
}

static inline void flexcan_exit_stop_mode(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr;

	/* remove stop request */
	regmap_update_bits(priv->stm.gpr, priv->stm.req_gpr,
			   1 << priv->stm.req_bit, 0);

	reg_mcr = priv->read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_SLF_WAK;
	priv->write(reg_mcr, &regs->mcr);
}

static inline void flexcan_error_irq_enable(const struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl = (priv->reg_ctrl_default | FLEXCAN_CTRL_ERR_MSK);

	priv->write(reg_ctrl, &regs->ctrl);
}

static inline void flexcan_error_irq_disable(const struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl = (priv->reg_ctrl_default & ~FLEXCAN_CTRL_ERR_MSK);

	priv->write(reg_ctrl, &regs->ctrl);
}

static inline int flexcan_transceiver_enable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_enable(priv->reg_xceiver);
}

static inline int flexcan_transceiver_disable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_disable(priv->reg_xceiver);
}

static int flexcan_chip_enable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	priv->write(reg, &regs->mcr);

	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_disable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	priv->write(reg, &regs->mcr);

	while (timeout-- && !(priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (!(priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_freeze(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = 1000 * 1000 * 10 / priv->can.bittiming.bitrate;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_HALT;
	priv->write(reg, &regs->mcr);

	while (timeout-- && !(priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(100);

	if (!(priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_unfreeze(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_HALT;
	priv->write(reg, &regs->mcr);

	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(10);

	if (priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_softreset(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	priv->write(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_SOFTRST))
		udelay(10);

	if (priv->read(&regs->mcr) & FLEXCAN_MCR_SOFTRST)
		return -ETIMEDOUT;

	return 0;
}

static int __flexcan_get_berr_counter(const struct net_device *dev,
				      struct can_berr_counter *bec)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg = priv->read(&regs->ecr);

	bec->txerr = (reg >> 0) & 0xff;
	bec->rxerr = (reg >> 8) & 0xff;

	return 0;
}

static int flexcan_get_berr_counter(const struct net_device *dev,
				    struct can_berr_counter *bec)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	err = __flexcan_get_berr_counter(dev, bec);

	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static int flexcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	const struct flexcan_priv *priv = netdev_priv(dev);
	struct canfd_frame *cfd = (struct canfd_frame *)skb->data;
	u32 can_id;
	u32 data;
	u32 ctrl = FLEXCAN_MB_CODE_TX_DATA | (can_len2dlc(cfd->len) << 16);
	int i;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	if (cfd->can_id & CAN_EFF_FLAG) {
		can_id = cfd->can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} else {
		can_id = (cfd->can_id & CAN_SFF_MASK) << 18;
	}

	if (can_is_canfd_skb(skb)) {
		ctrl |= FLEXCAN_MB_CNT_EDL;
		if (cfd->flags & CANFD_BRS)
			ctrl |= FLEXCAN_MB_CNT_BRS;
		if (cfd->flags & CANFD_ESI)
			ctrl |= FLEXCAN_MB_CNT_ESI;
	} else {
		if (cfd->can_id & CAN_RTR_FLAG)
			ctrl |= FLEXCAN_MB_CNT_RTR;
	}

	for (i = 0; i < can_len2dlc(cfd->len); i += sizeof(u32)) {
		data = be32_to_cpup((__be32 *)&cfd->data[i]);
		priv->write(data, &priv->tx_mb->data[i / sizeof(u32)]);
	}

	can_put_echo_skb(skb, dev, 0);

	priv->write(can_id, &priv->tx_mb->can_id);
	priv->write(ctrl, &priv->tx_mb->can_ctrl);

	/* Errata ERR005829 step8:
	 * Write twice INACTIVE(0x8) code to first MB.
	 */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb_reserved->can_ctrl);
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		      &priv->tx_mb_reserved->can_ctrl);

	return NETDEV_TX_OK;
}

static void flexcan_irq_bus_err(struct net_device *dev, u32 reg_esr)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	struct can_frame *cf;
	bool rx_errors = false, tx_errors = false;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return;

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
		netdev_dbg(dev, "BIT1_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		tx_errors = true;
	}
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
		netdev_dbg(dev, "BIT0_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		tx_errors = true;
	}
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		netdev_dbg(dev, "ACK_ERR irq\n");
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] = CAN_ERR_PROT_LOC_ACK;
		tx_errors = true;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		netdev_dbg(dev, "CRC_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
		rx_errors = true;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
		netdev_dbg(dev, "FRM_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_FORM;
		rx_errors = true;
	}
	if (reg_esr & FLEXCAN_ESR_STF_ERR) {
		netdev_dbg(dev, "STF_ERR irq\n");
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		rx_errors = true;
	}

	priv->can.can_stats.bus_error++;
	if (rx_errors)
		dev->stats.rx_errors++;
	if (tx_errors)
		dev->stats.tx_errors++;

	can_rx_offload_queue_tail(&priv->offload, skb);
}

static void flexcan_irq_state(struct net_device *dev, u32 reg_esr)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	struct can_frame *cf;
	enum can_state new_state, rx_state, tx_state;
	int flt;
	struct can_berr_counter bec;

	flt = reg_esr & FLEXCAN_ESR_FLT_CONF_MASK;
	if (likely(flt == FLEXCAN_ESR_FLT_CONF_ACTIVE)) {
		tx_state = unlikely(reg_esr & FLEXCAN_ESR_TX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		rx_state = unlikely(reg_esr & FLEXCAN_ESR_RX_WRN) ?
			CAN_STATE_ERROR_WARNING : CAN_STATE_ERROR_ACTIVE;
		new_state = max(tx_state, rx_state);
	} else {
		__flexcan_get_berr_counter(dev, &bec);
		new_state = flt == FLEXCAN_ESR_FLT_CONF_PASSIVE ?
			CAN_STATE_ERROR_PASSIVE : CAN_STATE_BUS_OFF;
		rx_state = bec.rxerr >= bec.txerr ? new_state : 0;
		tx_state = bec.rxerr <= bec.txerr ? new_state : 0;
	}

	/* state hasn't changed */
	if (likely(new_state == priv->can.state))
		return;

	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return;

	can_change_state(dev, cf, tx_state, rx_state);

	if (unlikely(new_state == CAN_STATE_BUS_OFF))
		can_bus_off(dev);

	can_rx_offload_queue_tail(&priv->offload, skb);
}

static inline struct flexcan_priv *rx_offload_to_priv(struct can_rx_offload *offload)
{
	return container_of(offload, struct flexcan_priv, offload);
}

static unsigned int flexcan_mailbox_read(struct can_rx_offload *offload,
					 struct sk_buff *skb,
					 u32 *timestamp, unsigned int n)
{
	struct flexcan_priv *priv = rx_offload_to_priv(offload);
	struct flexcan_regs __iomem *regs = priv->regs;
	struct flexcan_mb __iomem *mb;
	struct canfd_frame *cfd = (struct canfd_frame *)skb->data;
	u32 reg_ctrl, reg_id, reg_iflag1;
	int i;

	mb = flexcan_get_mb(priv, n);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		u32 code;

		do {
			reg_ctrl = priv->read(&mb->can_ctrl);
		} while (reg_ctrl & FLEXCAN_MB_CODE_RX_BUSY_BIT);

		/* is this MB empty? */
		code = reg_ctrl & FLEXCAN_MB_CODE_MASK;
		if ((code != FLEXCAN_MB_CODE_RX_FULL) &&
		    (code != FLEXCAN_MB_CODE_RX_OVERRUN))
			return 0;

		if (code == FLEXCAN_MB_CODE_RX_OVERRUN) {
			/* This MB was overrun, we lost data */
			offload->dev->stats.rx_over_errors++;
			offload->dev->stats.rx_errors++;
		}
	} else {
		reg_iflag1 = priv->read(&regs->iflag1);
		if (!(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE))
			return 0;

		reg_ctrl = priv->read(&mb->can_ctrl);
	}

	/* increase timstamp to full 32 bit */
	*timestamp = reg_ctrl << 16;

	reg_id = priv->read(&mb->can_id);
	if (reg_ctrl & FLEXCAN_MB_CNT_IDE)
		cfd->can_id = ((reg_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cfd->can_id = (reg_id >> 18) & CAN_SFF_MASK;

	if (reg_ctrl & FLEXCAN_MB_CNT_RTR)
		cfd->can_id |= CAN_RTR_FLAG;

	if (can_is_canfd_skb(skb))
		cfd->len = can_dlc2len(get_canfd_dlc((reg_ctrl >> 16) & 0xf));
	else
		cfd->len = get_can_dlc((reg_ctrl >> 16) & 0xf);

	for (i = 0; i < cfd->len; i += sizeof(u32)) {
		__be32 data = cpu_to_be32(priv->read(&mb->data[i / sizeof(u32)]));
		*(__be32 *)(cfd->data + i) = data;
	}

	/* mark as read */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		/* Clear IRQ */
		if (n < 32)
			priv->write(BIT(n), &regs->iflag1);
		else
			priv->write(BIT(n - 32), &regs->iflag2);
	} else {
		priv->write(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
		priv->read(&regs->timer);
	}

	return 1;
}


static inline u64 flexcan_read_reg_iflag_rx(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u64 iflag;

	iflag = ((u64)priv->read(&regs->iflag2) << 32) |
		 priv->read(&regs->iflag1);

	iflag &= priv->reg_imask_default;

	iflag &= ~FLEXCAN_IFLAG_MB(priv->tx_mb_idx);

	return iflag;
}

static irqreturn_t flexcan_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_device_stats *stats = &dev->stats;
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	irqreturn_t handled = IRQ_NONE;
	u64 reg_iflag;
	u32 reg_esr;
	enum can_state last_state = priv->can.state;

	/* reception interrupt */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		u64 reg_iflag;
		int ret;

		while ((reg_iflag = flexcan_read_reg_iflag_rx(priv))) {
			handled = IRQ_HANDLED;
			ret = can_rx_offload_irq_offload_timestamp(&priv->offload,
								   reg_iflag);
			if (!ret)
				break;
		}
	} else {
		u32 reg_iflag1;

		reg_iflag1 = priv->read(&regs->iflag1);
		if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
			handled = IRQ_HANDLED;
			can_rx_offload_irq_offload_fifo(&priv->offload);
		}

		/* FIFO overflow interrupt */
		if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
			handled = IRQ_HANDLED;
			priv->write(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW,
				    &regs->iflag1);
			dev->stats.rx_over_errors++;
			dev->stats.rx_errors++;
		}
	}

	reg_iflag = ((u64)priv->read(&regs->iflag2) << 32) |
		     priv->read(&regs->iflag1);

	/* transmission complete interrupt */
	if (reg_iflag & FLEXCAN_IFLAG_MB(priv->tx_mb_idx)) {
		handled = IRQ_HANDLED;
		stats->tx_bytes += can_get_echo_skb(dev, 0);
		stats->tx_packets++;
		can_led_event(dev, CAN_LED_EVENT_TX);

		/* after sending a RTR frame MB is in RX mode */
		priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
			    &priv->tx_mb->can_ctrl);
		reg_iflag = FLEXCAN_IFLAG_MB(priv->tx_mb_idx);
		priv->write(reg_iflag >> 32, &regs->iflag2);
		priv->write((u32)reg_iflag, &regs->iflag1);
		netif_wake_queue(dev);
	}

	reg_esr = priv->read(&regs->esr);

	/* ACK all bus error and state change IRQ sources */
	if (reg_esr & FLEXCAN_ESR_ALL_INT) {
		handled = IRQ_HANDLED;
		priv->write(reg_esr & FLEXCAN_ESR_ALL_INT, &regs->esr);
	}

	/* state change interrupt or broken error state quirk fix is enabled */
	if ((reg_esr & FLEXCAN_ESR_ERR_STATE) ||
	    (priv->devtype_data->quirks & (FLEXCAN_QUIRK_BROKEN_WERR_STATE |
	                                   FLEXCAN_QUIRK_BROKEN_PERR_STATE)))
		flexcan_irq_state(dev, reg_esr);

	/* bus error IRQ - handle if bus error reporting is activated */
	if ((reg_esr & FLEXCAN_ESR_ERR_BUS) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING))
		flexcan_irq_bus_err(dev, reg_esr);

	/* availability of error interrupt among state transitions in case
	 * bus error reporting is de-activated and
	 * FLEXCAN_QUIRK_BROKEN_PERR_STATE is enabled:
	 *  +--------------------------------------------------------------+
	 *  | +----------------------------------------------+ [stopped /  |
	 *  | |                                              |  sleeping] -+
	 *  +-+-> active <-> warning <-> passive -> bus off -+
	 *        ___________^^^^^^^^^^^^_______________________________
	 *        disabled(1)  enabled             disabled
	 *
	 * (1): enabled if FLEXCAN_QUIRK_BROKEN_WERR_STATE is enabled
	 */
	if ((last_state != priv->can.state) &&
	    (priv->devtype_data->quirks & FLEXCAN_QUIRK_BROKEN_PERR_STATE) &&
	    !(priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)) {
		switch (priv->can.state) {
		case CAN_STATE_ERROR_ACTIVE:
			if (priv->devtype_data->quirks &
			    FLEXCAN_QUIRK_BROKEN_WERR_STATE)
				flexcan_error_irq_enable(priv);
			else
				flexcan_error_irq_disable(priv);
			break;

		case CAN_STATE_ERROR_WARNING:
			flexcan_error_irq_enable(priv);
			break;

		case CAN_STATE_ERROR_PASSIVE:
		case CAN_STATE_BUS_OFF:
			flexcan_error_irq_disable(priv);
			break;

		default:
			break;
		}
	}

	return handled;
}

static void flexcan_set_bittiming(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *data_bt = &priv->can.data_bittiming;
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl, reg_mcr, reg_cbt, reg_fdcbt, reg_fdctrl;

	reg_mcr = priv->read(&regs->mcr);
	reg_ctrl = priv->read(&regs->ctrl);

	reg_ctrl &= ~(FLEXCAN_CTRL_LPB |
		      FLEXCAN_CTRL_SMP |
		      FLEXCAN_CTRL_LOM);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg_ctrl |= FLEXCAN_CTRL_LPB;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		reg_ctrl |= FLEXCAN_CTRL_LOM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		reg_ctrl |= FLEXCAN_CTRL_SMP;

	if (reg_mcr & FLEXCAN_MCR_FDEN) {
		reg_cbt = priv->read(&regs->cbt);
		reg_fdcbt = priv->read(&regs->fdcbt);
		reg_fdctrl = priv->read(&regs->fdctrl);

		reg_cbt |= FLEXCAN_CBT_BTF;
		reg_cbt &= ~(FLEXCAN_CBT_EPRESDIV(0x3ff) |
			     FLEXCAN_CBT_ERJW(0x0f) |
			     FLEXCAN_CBT_EPROPSEG(0x3f) |
			     FLEXCAN_CBT_EPSEG1(0x1f) |
			     FLEXCAN_CBT_EPSEG2(0x1f));

		/* CBT[EPSEG1] is 5 bit long and CBT[EPROPSEG] is 6 bit long.
		 * The can_calc_bittiming tries to divide the tseg1 equally
		 * between phase_seg1 and prop_seg, which may not fit in CBT
		 * register. Therefore, if phase_seg1 is more than possible
		 * value, increase prop_seg and decrease phase_seg1
		 */
		if (bt->phase_seg1 > 0x20) {
			bt->prop_seg += (bt->phase_seg1 - 0x20);
			bt->phase_seg1 = 0x20;
		}

		reg_cbt |= FLEXCAN_CBT_EPRESDIV(bt->brp - 1) |
			   FLEXCAN_CBT_ERJW(bt->sjw - 1) |
			   FLEXCAN_CBT_EPROPSEG(bt->prop_seg - 1) |
			   FLEXCAN_CBT_EPSEG1(bt->phase_seg1 - 1) |
			   FLEXCAN_CBT_EPSEG2(bt->phase_seg2 - 1);

		reg_fdcbt &= ~(FLEXCAN_FDCBT_FPRESDIV(0x3ff) |
			       FLEXCAN_FDCBT_FRJW(0x03) |
			       FLEXCAN_CBT_EPROPSEG(0x1f) |
			       FLEXCAN_FDCBT_FPSEG1(0x07) |
			       FLEXCAN_FDCBT_FPSEG2(0x07));

		/* FDCBT[FPSEG1] is 3 bit long and FDCBT[FPROPSEG] is 5 bit long.
		 * The can_calc_bittiming tries to divide the tseg1 equally
		 * between phase_seg1 and prop_seg, which may not fit in FDCBT
		 * register. Therefore, if phase_seg1 is more than possible
		 * value, increase prop_seg and decrease phase_seg1
		 */
		if (data_bt->phase_seg1 > 0x8) {
			data_bt->prop_seg += (data_bt->phase_seg1 - 0x8);
			data_bt->phase_seg1 = 0x8;
		}

		reg_fdcbt |= FLEXCAN_FDCBT_FPRESDIV(data_bt->brp - 1) |
			     FLEXCAN_FDCBT_FRJW(data_bt->sjw - 1) |
			     FLEXCAN_FDCBT_FPROPSEG(data_bt->prop_seg) |
			     FLEXCAN_FDCBT_FPSEG1(data_bt->phase_seg1 - 1) |
			     FLEXCAN_FDCBT_FPSEG2(data_bt->phase_seg2 - 1);

		if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
			u32 tdcoff;
			/* No TDC is needed for data bit rates up to 2.5 MBit/s
			 * refer https://www.can-cia.org/fileadmin/resources/documents/proceedings/2013_hartwich_v2.pdf
			 */
			if (reg_ctrl & FLEXCAN_CTRL_LPB ||
			    data_bt->bitrate <= 2500000)
				reg_fdctrl &= ~FLEXCAN_FDCTRL_TDCEN;
			else
				reg_fdctrl |= FLEXCAN_FDCTRL_TDCEN;

			reg_fdctrl |= (FLEXCAN_FDCTRL_FDRATE |
				       FLEXCAN_FDCTRL_TDCFAIL);

			/* offset should be within the can FD bit timings */
			tdcoff = (data_bt->phase_seg1 + data_bt->prop_seg) *
				  data_bt->brp;

			reg_fdctrl |= FLEXCAN_FDCTRL_TDCOFF(tdcoff);
			reg_fdctrl |= FLEXCAN_FDCTRL_MBDSR0(3) |
				      FLEXCAN_FDCTRL_MBDSR1(3);
		} else {
			reg_fdctrl |= FLEXCAN_FDCTRL_MBDSR0(0) |
				      FLEXCAN_FDCTRL_MBDSR1(0);
			reg_fdctrl &= ~FLEXCAN_FDCTRL_FDRATE;
		}
		netdev_dbg(dev, "writing fdctrl=0x%08x", reg_fdctrl);
		netdev_dbg(dev, "writing cbt=0x%08x\n", reg_cbt);
		netdev_dbg(dev, "writing fdcbt=0x%08x\n", reg_fdcbt);
		priv->write(reg_fdctrl, &regs->fdctrl);
		priv->write(reg_cbt, &regs->cbt);
		priv->write(reg_fdcbt, &regs->fdcbt);
	} else {
		reg_ctrl &= ~(FLEXCAN_CTRL_PRESDIV(0xff) |
			FLEXCAN_CTRL_RJW(0x3) |
			FLEXCAN_CTRL_PSEG1(0x7) |
			FLEXCAN_CTRL_PSEG2(0x7) |
			FLEXCAN_CTRL_PROPSEG(0x7));

		reg_ctrl |= FLEXCAN_CTRL_PRESDIV(bt->brp - 1) |
			FLEXCAN_CTRL_PSEG1(bt->phase_seg1 - 1) |
			FLEXCAN_CTRL_PSEG2(bt->phase_seg2 - 1) |
			FLEXCAN_CTRL_RJW(bt->sjw - 1) |
			FLEXCAN_CTRL_PROPSEG(bt->prop_seg - 1);
	}

	netdev_dbg(dev, "writing ctrl=0x%08x\n", reg_ctrl);
	priv->write(reg_ctrl, &regs->ctrl);

	/* print chip status */
	netdev_dbg(dev, "%s: mcr=0x%08x ctrl=0x%08x\n", __func__,
		   priv->read(&regs->mcr), priv->read(&regs->ctrl));
}

/* flexcan_chip_start
 *
 * this functions is entered with clocks enabled
 *
 */
static int flexcan_chip_start(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr, reg_ctrl, reg_ctrl2, reg_mecr;
	int err, i;
	struct flexcan_mb __iomem *mb;

	/* enable module */
	err = flexcan_chip_enable(priv);
	if (err)
		return err;

	/* soft reset */
	err = flexcan_chip_softreset(priv);
	if (err)
		goto out_chip_disable;

	/* MCR
	 *
	 * enable freeze
	 * enable fifo
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * enable individual RX masking
	 * choose format C
	 * set max mailbox number
	 * enable FD mode
	 */
	reg_mcr = priv->read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_MAXMB(0xff);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV |
		FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_IRMQ | FLEXCAN_MCR_IDAM_C;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		reg_mcr &= ~FLEXCAN_MCR_FEN;
		reg_mcr |= FLEXCAN_MCR_MAXMB(priv->offload.mb_last);
	} else {
		reg_mcr |= FLEXCAN_MCR_FEN |
			FLEXCAN_MCR_MAXMB(priv->tx_mb_idx);
	}

	/* MCR
	 *
	 * NOTE: In loopback mode, the CAN_MCR[SRXDIS] cannot be
	 *       asserted because this will impede the self reception
	 *       of a transmitted message. This is not documented in
	 *       earlier versions of flexcan block guide.
	 *
	 * Self Reception:
	 * - enable Self Reception for loopback mode
	 *   (by clearing "Self Reception Disable" bit)
	 * - disable for normal operation
	 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg_mcr &= ~FLEXCAN_MCR_SRX_DIS;
	else
		reg_mcr |= FLEXCAN_MCR_SRX_DIS;
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_FD)
		reg_mcr |= FLEXCAN_MCR_FDEN;

	netdev_dbg(dev, "%s: writing mcr=0x%08x", __func__, reg_mcr);
	priv->write(reg_mcr, &regs->mcr);

	flexcan_set_bittiming(dev);

	/* CTRL
	 *
	 * disable timer sync feature
	 *
	 * disable auto busoff recovery
	 * transmit lowest buffer first
	 *
	 * enable tx and rx warning interrupt
	 * enable bus off interrupt
	 * (== FLEXCAN_CTRL_ERR_STATE)
	 */
	reg_ctrl = priv->read(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF |
		FLEXCAN_CTRL_ERR_STATE;

	/* enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_BROKEN_WERR_STATE ||
	    priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;
	else
		reg_ctrl &= ~FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	/* leave interrupts disabled for now */
	reg_ctrl &= ~FLEXCAN_CTRL_ERR_ALL;
	netdev_dbg(dev, "%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	priv->write(reg_ctrl, &regs->ctrl);

	if ((priv->devtype_data->quirks & FLEXCAN_QUIRK_ENABLE_EACEN_RRS)) {
		reg_ctrl2 = priv->read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_EACEN | FLEXCAN_CTRL2_RRS;
		priv->write(reg_ctrl2, &regs->ctrl2);
	}

	/* clear and invalidate all mailboxes first */
	for (i = priv->tx_mb_idx; i < priv->mb_count ; i++) {
		mb = flexcan_get_mb(priv, i);
		priv->write(FLEXCAN_MB_CODE_RX_INACTIVE,
			    &mb->can_ctrl);
	}

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		for (i = priv->offload.mb_first; i <= priv->offload.mb_last; i++) {
			mb = flexcan_get_mb(priv, i);
			priv->write(FLEXCAN_MB_CODE_RX_EMPTY,
				    &mb->can_ctrl);
		}
	}

	/* Errata ERR005829: mark first TX mailbox as INACTIVE */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb_reserved->can_ctrl);

	/* mark TX mailbox as INACTIVE */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb->can_ctrl);

	/* acceptance mask/acceptance code (accept everything) */
	priv->write(0x0, &regs->rxgmask);
	priv->write(0x0, &regs->rx14mask);
	priv->write(0x0, &regs->rx15mask);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		priv->write(0x0, &regs->rxfgmask);

	/* clear acceptance filters */
	for (i = 0; i < priv->mb_count; i++)
		priv->write(0, &regs->rximr[i]);

	/* On Vybrid, disable memory error detection interrupts
	 * and freeze mode.
	 * This also works around errata e5295 which generates
	 * false positive memory errors and put the device in
	 * freeze mode.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_MECR) {
		/* Follow the protocol as described in "Detection
		 * and Correction of Memory Errors" to write to
		 * MECR register
		 */
		reg_ctrl2 = priv->read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_ECRWRE;
		priv->write(reg_ctrl2, &regs->ctrl2);

		reg_mecr = priv->read(&regs->mecr);
		reg_mecr &= ~FLEXCAN_MECR_ECRWRDIS;
		priv->write(reg_mecr, &regs->mecr);
		reg_mecr &= ~(FLEXCAN_MECR_NCEFAFRZ | FLEXCAN_MECR_HANCEI_MSK |
			      FLEXCAN_MECR_FANCEI_MSK);
		priv->write(reg_mecr, &regs->mecr);
	}

	err = flexcan_transceiver_enable(priv);
	if (err)
		goto out_chip_disable;

	/* synchronize with the can bus */
	err = flexcan_chip_unfreeze(priv);
	if (err)
		goto out_transceiver_disable;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts atomically */
	disable_irq(dev->irq);
	priv->write(priv->reg_ctrl_default, &regs->ctrl);
	priv->write(priv->reg_imask_default >> 32, &regs->imask2);
	priv->write((u32)priv->reg_imask_default, &regs->imask1);
	enable_irq(dev->irq);

	/* print chip status */
	netdev_dbg(dev, "%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		   priv->read(&regs->mcr), priv->read(&regs->ctrl));

	return 0;

 out_transceiver_disable:
	flexcan_transceiver_disable(priv);
 out_chip_disable:
	flexcan_chip_disable(priv);
	return err;
}

/* flexcan_chip_stop
 *
 * this functions is entered with clocks enabled
 */
static void flexcan_chip_stop(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;

	/* freeze + disable module */
	flexcan_chip_freeze(priv);
	flexcan_chip_disable(priv);

	/* Disable all interrupts */
	priv->write(0, &regs->imask2);
	priv->write(0, &regs->imask1);
	priv->write(priv->reg_ctrl_default & ~FLEXCAN_CTRL_ERR_ALL,
		    &regs->ctrl);

	flexcan_transceiver_disable(priv);
	priv->can.state = CAN_STATE_STOPPED;
}

static int flexcan_open(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	err = open_candev(dev);
	if (err)
		goto out_disable_per;

	err = request_irq(dev->irq, flexcan_irq, IRQF_SHARED, dev->name, dev);
	if (err)
		goto out_close;

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		priv->offload.fd_enable = true;
		priv->mb_size = sizeof(struct flexcan_mb) + CANFD_MAX_DLEN;
	} else {
		priv->offload.fd_enable = false;
		priv->mb_size = sizeof(struct flexcan_mb) + CAN_MAX_DLEN;
	}

	priv->mb_count = (sizeof(priv->regs->mb[0]) / priv->mb_size) +
			 (sizeof(priv->regs->mb[1]) / priv->mb_size);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		priv->tx_mb_idx = FLEXCAN_TX_MB_OFF_TIMESTAMP;
		priv->tx_mb_reserved =
			flexcan_get_mb(priv, FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP);
	} else {
		priv->tx_mb_idx = FLEXCAN_TX_MB_OFF_FIFO;
		priv->tx_mb_reserved =
			flexcan_get_mb(priv, FLEXCAN_TX_MB_RESERVED_OFF_FIFO);
	}
	priv->tx_mb = flexcan_get_mb(priv, priv->tx_mb_idx);

	priv->reg_imask_default = FLEXCAN_IFLAG_MB(priv->tx_mb_idx);

	priv->offload.mailbox_read = flexcan_mailbox_read;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		u64 imask;

		priv->offload.mb_first = FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST;
		priv->offload.mb_last = priv->mb_count - 2;

		imask = GENMASK_ULL(priv->offload.mb_last, priv->offload.mb_first);
		priv->reg_imask_default |= imask;

		err = can_rx_offload_add_timestamp(dev, &priv->offload);
	} else {
		priv->reg_imask_default |= FLEXCAN_IFLAG_RX_FIFO_OVERFLOW |
			FLEXCAN_IFLAG_RX_FIFO_AVAILABLE;
		err = can_rx_offload_add_fifo(dev, &priv->offload, FLEXCAN_NAPI_WEIGHT);
	}
	if (err)
		goto out_free_irq;

	/* start chip and queuing */
	err = flexcan_chip_start(dev);
	if (err)
		goto out_offload_del;

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	can_rx_offload_enable(&priv->offload);
	netif_start_queue(dev);

	return 0;

 out_offload_del:
	can_rx_offload_del(&priv->offload);
 out_free_irq:
	free_irq(dev->irq, dev);
 out_close:
	close_candev(dev);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static int flexcan_close(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	can_rx_offload_disable(&priv->offload);
	flexcan_chip_stop(dev);

	can_rx_offload_del(&priv->offload);
	free_irq(dev->irq, dev);
	clk_disable_unprepare(priv->clk_per);
	clk_disable_unprepare(priv->clk_ipg);

	close_candev(dev);

	can_led_event(dev, CAN_LED_EVENT_STOP);

	return 0;
}

static int flexcan_set_mode(struct net_device *dev, enum can_mode mode)
{
	int err;

	switch (mode) {
	case CAN_MODE_START:
		err = flexcan_chip_start(dev);
		if (err)
			return err;

		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops flexcan_netdev_ops = {
	.ndo_open	= flexcan_open,
	.ndo_stop	= flexcan_close,
	.ndo_start_xmit	= flexcan_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int register_flexcandev(struct net_device *dev)
{
	struct flexcan_priv *priv = netdev_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg, err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		goto out_disable_ipg;

	/* select "bus clock", chip must be disabled */
	err = flexcan_chip_disable(priv);
	if (err)
		goto out_disable_per;
	reg = priv->read(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	priv->write(reg, &regs->ctrl);

	err = flexcan_chip_enable(priv);
	if (err)
		goto out_chip_disable;

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	priv->write(reg, &regs->mcr);

	/* Currently we only support newer versions of this core
	 * featuring a RX hardware FIFO (although this driver doesn't
	 * make use of it on some cores). Older cores, found on some
	 * Coldfire derivates are not tested.
	 */
	reg = priv->read(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		netdev_err(dev, "Could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out_chip_disable;
	}

	err = register_candev(dev);

	/* disable core and turn off clocks */
 out_chip_disable:
	flexcan_chip_disable(priv);
 out_disable_per:
	clk_disable_unprepare(priv->clk_per);
 out_disable_ipg:
	clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static void unregister_flexcandev(struct net_device *dev)
{
	unregister_candev(dev);
}

static int flexcan_setup_stop_mode(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	struct device_node *gpr_np;
	struct flexcan_priv *priv;
	phandle phandle;
	u32 out_val[5];
	int ret;

	if (!np)
		return -EINVAL;

	/* stop mode property format is:
	 * <&gpr req_gpr req_bit ack_gpr ack_bit>.
	 */
	ret = of_property_read_u32_array(np, "fsl,stop-mode", out_val,
					 ARRAY_SIZE(out_val));
	if (ret) {
		dev_dbg(&pdev->dev, "no stop-mode property\n");
		return ret;
	}
	phandle = *out_val;

	gpr_np = of_find_node_by_phandle(phandle);
	if (!gpr_np) {
		dev_dbg(&pdev->dev, "could not find gpr node by phandle\n");
		return PTR_ERR(gpr_np);
	}

	priv = netdev_priv(dev);
	priv->stm.gpr = syscon_node_to_regmap(gpr_np);
	of_node_put(gpr_np);
	if (IS_ERR(priv->stm.gpr)) {
		dev_dbg(&pdev->dev, "could not find gpr regmap\n");
		return PTR_ERR(priv->stm.gpr);
	}

	priv->stm.req_gpr = out_val[1];
	priv->stm.req_bit = out_val[2];
	priv->stm.ack_gpr = out_val[3];
	priv->stm.ack_bit = out_val[4];

	dev_dbg(&pdev->dev,
		"gpr %s req_gpr=0x02%x req_bit=%u ack_gpr=0x02%x ack_bit=%u\n",
		gpr_np->full_name, priv->stm.req_gpr, priv->stm.req_bit,
		priv->stm.ack_gpr, priv->stm.ack_bit);

	device_set_wakeup_capable(&pdev->dev, true);

	return 0;
}

static const struct of_device_id flexcan_of_match[] = {
	{ .compatible = "fsl,imx6q-flexcan", .data = &fsl_imx6q_devtype_data, },
	{ .compatible = "fsl,imx28-flexcan", .data = &fsl_imx28_devtype_data, },
	{ .compatible = "fsl,imx53-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,imx35-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,imx25-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,p1010-flexcan", .data = &fsl_p1010_devtype_data, },
	{ .compatible = "fsl,vf610-flexcan", .data = &fsl_vf610_devtype_data, },
	{ .compatible = "fsl,ls1021ar2-flexcan", .data = &fsl_ls1021a_r2_devtype_data, },
	{ .compatible = "fsl,lx2160ar1-flexcan", .data = &fsl_lx2160a_r1_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, flexcan_of_match);

static const struct platform_device_id flexcan_id_table[] = {
	{ .name = "flexcan", .driver_data = (kernel_ulong_t)&fsl_p1010_devtype_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, flexcan_id_table);

static int flexcan_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	const struct flexcan_devtype_data *devtype_data;
	struct net_device *dev;
	struct flexcan_priv *priv;
	struct regulator *reg_xceiver;
	struct resource *mem;
	struct clk *clk_ipg = NULL, *clk_per = NULL;
	struct flexcan_regs __iomem *regs;
	int err, irq;
	u32 clock_freq = 0;

	reg_xceiver = devm_regulator_get(&pdev->dev, "xceiver");
	if (PTR_ERR(reg_xceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (IS_ERR(reg_xceiver))
		reg_xceiver = NULL;

	if (pdev->dev.of_node)
		of_property_read_u32(pdev->dev.of_node,
				     "clock-frequency", &clock_freq);

	if (!clock_freq) {
		clk_ipg = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(clk_ipg)) {
			dev_err(&pdev->dev, "no ipg clock defined\n");
			return PTR_ERR(clk_ipg);
		}

		clk_per = devm_clk_get(&pdev->dev, "per");
		if (IS_ERR(clk_per)) {
			dev_err(&pdev->dev, "no per clock defined\n");
			return PTR_ERR(clk_per);
		}
		clock_freq = clk_get_rate(clk_per);
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	of_id = of_match_device(flexcan_of_match, &pdev->dev);
	if (of_id) {
		devtype_data = of_id->data;
	} else if (platform_get_device_id(pdev)->driver_data) {
		devtype_data = (struct flexcan_devtype_data *)
			platform_get_device_id(pdev)->driver_data;
	} else {
		return -ENODEV;
	}

	dev = alloc_candev(sizeof(struct flexcan_priv), 1);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->netdev_ops = &flexcan_netdev_ops;
	dev->irq = irq;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);

	if (of_property_read_bool(pdev->dev.of_node, "big-endian")) {
		priv->read = flexcan_read_be;
		priv->write = flexcan_write_be;
	} else {
		if (of_device_is_compatible(pdev->dev.of_node,
					    "fsl,p1010-flexcan")) {
			priv->read = flexcan_read_be;
			priv->write = flexcan_write_be;
		} else {
			priv->read = flexcan_read_le;
			priv->write = flexcan_write_le;
		}
	}

	if ((devtype_data->quirks & FLEXCAN_QUIRK_USE_FD) &&
	    !(devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP)) {
		dev_err(&pdev->dev, "can't use FIFO with FD mode\n");
		err = -ENODEV;
		goto failed_register;
	}

	priv->can.clock.freq = clock_freq;
	priv->can.bittiming_const = &flexcan_bittiming_const;
	priv->can.do_set_mode = flexcan_set_mode;
	priv->can.do_get_berr_counter = flexcan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY	| CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_BERR_REPORTING;
	priv->regs = regs;
	priv->clk_ipg = clk_ipg;
	priv->clk_per = clk_per;
	priv->devtype_data = devtype_data;
	priv->reg_xceiver = reg_xceiver;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_FD) {
		priv->can.bittiming_const = &extended_flexcan_bittiming_const;
		priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD;
		priv->can.data_bittiming_const = &flexcan_data_bittiming_const;
	}

	err = register_flexcandev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed\n");
		goto failed_register;
	}

	devm_can_led_init(dev);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_SETUP_STOP_MODE) {
		err = flexcan_setup_stop_mode(pdev);
		if (err)
			dev_dbg(&pdev->dev, "failed to setup stop-mode\n");
	}

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d)\n",
		 priv->regs, dev->irq);

	return 0;

 failed_register:
	free_candev(dev);
	return err;
}

static int flexcan_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	unregister_flexcandev(dev);
	free_candev(dev);

	return 0;
}

static int __maybe_unused flexcan_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	if (netif_running(dev)) {
		/* if wakeup is enabled, enter stop mode
		 * else enter disabled mode.
		 */
		if (device_may_wakeup(device)) {
			enable_irq_wake(dev->irq);
			flexcan_enter_stop_mode(priv);
		} else {
			err = flexcan_chip_disable(priv);
			if (err)
				return err;
		}
		netif_stop_queue(dev);
		netif_device_detach(dev);
	}
	priv->can.state = CAN_STATE_SLEEPING;

	return 0;
}

static int __maybe_unused flexcan_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);
	int err;

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(dev)) {
		netif_device_attach(dev);
		netif_start_queue(dev);
		if (device_may_wakeup(device)) {
			disable_irq_wake(dev->irq);
		} else {
			err = flexcan_chip_enable(priv);
			if (err)
				return err;
		}
	}
	return 0;
}

static int __maybe_unused flexcan_noirq_suspend(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);

	if (netif_running(dev) && device_may_wakeup(device))
		flexcan_enable_wakeup_irq(priv, true);

	return 0;
}

static int __maybe_unused flexcan_noirq_resume(struct device *device)
{
	struct net_device *dev = dev_get_drvdata(device);
	struct flexcan_priv *priv = netdev_priv(dev);

	if (netif_running(dev) && device_may_wakeup(device)) {
		flexcan_enable_wakeup_irq(priv, false);
		flexcan_exit_stop_mode(priv);
	}

	return 0;
}

static const struct dev_pm_ops flexcan_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(flexcan_suspend, flexcan_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(flexcan_noirq_suspend, flexcan_noirq_resume)
};

static struct platform_driver flexcan_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &flexcan_pm_ops,
		.of_match_table = flexcan_of_match,
	},
	.probe = flexcan_probe,
	.remove = flexcan_remove,
	.id_table = flexcan_id_table,
};

module_platform_driver(flexcan_driver);

MODULE_AUTHOR("Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver for flexcan based chip");
