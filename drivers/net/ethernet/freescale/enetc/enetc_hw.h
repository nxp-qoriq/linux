#include <linux/bitops.h>

/* SI regs, offset: 0h */
#define ENETC_SIMR	0
#define ENETC_SIMR_EN	BIT(31)

#define ENETC_SICAPR0	0x900
#define ENETC_SICAPR1	0x904

#define ENETC_SIMSITRV(n) (0xB00 + (n) * 0x4)
#define ENETC_SIMSIRRV(n) (0xB80 + (n) * 0x4)

#define ENETC_SICCAPR	0x1200

/** SI BDR sub-blocks, n = 0..7 */
enum enetc_bdr_type {TX, RX};
#define ENETC_BDR(type, n, off)	(0x8000 + (type) * 0x100 + (n) * 0x200 + (off))
/*** RX BDR reg offsets */
#define ENETC_RBMR	0
#define ENETC_RBMR_EN	BIT(31)

#define ENETC_RBSR	0x4
#define ENETC_RBBSR	0x8
#define ENETC_RBCIR	0xc
#define ENETC_RBBAR0	0x10
#define ENETC_RBBAR1	0x14
#define ENETC_RBPIR	0x18
#define ENETC_RBIER	0xa0
#define ENETC_RBIER_RXTIE	BIT(0)
#define ENETC_RBIDR	0xa4
#define ENETC_RBICIR0	0xa8
#define ENETC_RBICIR0_ICEN	BIT(31)

/*** TX BDR reg offsets */
#define ENETC_TBMR	0
#define ENETC_TBSR	0x4
#define ENETC_TBBAR0	0x10
#define ENETC_TBBAR1	0x14
#define ENETC_TBCIR	0x18
#define ENETC_TBCISR	0x1c
#define ENETC_TBCISR_IDX_MASK	0xffff
#define ENETC_TBIER	0xa0
#define ENETC_TBIER_TXFIE	BIT(1)
#define ENETC_TBIDR	0xa4

#define ENETC_RTBMR_RSIZE(n) __ilog2_u32((n) >> 6)
#define ENETC_TBMR_EN	BIT(31)

/* Port regs, offset: 1_0000h */
#define ENETC_PMR	0x10000
#define ENETC_PMR_EN	BIT(31)
#define ENETC_PSR	0x10004 /* RO */
#define ENETC_PSIPMR	0x10018
#define ENETC_PCAPR0	0x10900
#define ENETC_PCAPR1	0x10904

#define ENETC_PV0CFGR	0x10920
#define ENETC_PVCFGR_SET_TXBDR(val)	((val) & 0xffff)
#define ENETC_PVCFGR_SET_RXBDR(val)	(((val) & 0xffff) << 8)

#define ENETC_PM0_CMD_CFG	0x18008
#define ENETC_PM0_TX_EN		BIT(31)
#define ENETC_PM0_RX_EN		BIT(30)

#define ENETC_PM0_MAXFRM	0x18014
#define ENETC_SET_MAXFRM(val)	((val) << 16)

/* Global regs, offset: 2_0000h */
#define ENETC_G_EIPBRR0		0x20bf8
#define ENETC_G_EIPBRR1		0x20bfc

/* PCI device info */
struct enetc_hw {
	void __iomem *reg;
};

/* general register accessors */
#define enetc_rd_reg(reg)	ioread32((reg))
#define enetc_wr_reg(reg, val)	iowrite32((val), (reg))
#define enetc_rd(hw, off)	enetc_rd_reg((hw)->reg + (off))
#define enetc_wr(hw, off, val)	enetc_wr_reg((hw)->reg + (off), val)
/* BDR register accessors, see ENETC_BDR() */
#define enetc_bdr_rd(hw, t, n, off) \
				enetc_rd(hw, ENETC_BDR(t, n, off))
#define enetc_bdr_wr(hw, t, n, off, val) \
				enetc_wr(hw, ENETC_BDR(t, n, off), val)
#define enetc_txbdr_rd(hw, n, off) enetc_bdr_rd(hw, TX, n, off)
#define enetc_rxbdr_rd(hw, n, off) enetc_bdr_rd(hw, RX, n, off)
#define enetc_txbdr_wr(hw, n, off, val) \
				enetc_bdr_wr(hw, TX, n, off, val)
#define enetc_rxbdr_wr(hw, n, off, val) \
				enetc_bdr_wr(hw, RX, n, off, val)

/* Buffer Descriptors (BD) */
struct enetc_tx_bd {
	__le64 addr;
	__le16 buf_len;
	__le16 frm_len;
	__le16 err_csum;
	__le16 flags;
};

#define ENETC_TXBD_FLAGS_IE	BIT(13)
#define ENETC_TXBD_FLAGS_F	BIT(15)

union enetc_rx_bd {
	struct {
		__le64 addr;
		u8 reserved[8];
	} w;
	struct {
		__le16 inet_csum;
		__le16 parse_summary;
		__le32 rss_hash;
		__le16 buf_len;
		__le16 vlan_opt;
		union {
			struct {
				__le16 flags;
				__le16 error;
			};
			__le32 lstatus;
		};
	} r;
};

#define ENETC_RXBD_LSTATUS_R	BIT(30)
#define ENETC_RXBD_LSTATUS_F	BIT(31)
#define ENETC_RXBD_ERR_MASK	0xff
#define ENETC_RXBD_LSTATUS(flags)	((flags) << 16)
