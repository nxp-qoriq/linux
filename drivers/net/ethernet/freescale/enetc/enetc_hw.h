#include <linux/bitops.h>

/* ENETC device IDs */
#define ENETC_DEV_ID_PF	0xe100
#define ENETC_DEV_ID_VF	0xef00

/* ENETC register block BAR */
#define ENETC_BAR_REGS	0

/* SI regs, offset: 0h */
#define ENETC_SIMR	0
#define ENETC_SIMR_EN	BIT(31)

#define ENETC_SIPMAR0	0x80
#define ENETC_SIPMAR1	0x84

/* SI statistics */
#define ENETC_SIROCT	0x300
#define ENETC_SIRFRM	0x308
#define ENETC_SIRUCA	0x310
#define ENETC_SIRMCA	0x318
#define ENETC_SITOCT	0x320
#define ENETC_SITFRM	0x328
#define ENETC_SITUCA	0x330
#define ENETC_SITMCA	0x338

/* Control BDR regs */
#define ENETC_SICBDRMR		0x800
#define ENETC_SICBDRSR		0x804	/* RO */
#define ENETC_SICBDRBAR0	0x810
#define ENETC_SICBDRBAR1	0x814
#define ENETC_SICBDRCIR		0x818
#define ENETC_SICBDRCISR	0x81c
#define ENETC_SICBDRLENR	0x820

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
#define ENETC_RBLENR	0x20
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
#define ENETC_TBLENR	0x20
#define ENETC_TBCISR_IDX_MASK	0xffff
#define ENETC_TBIER	0xa0
#define ENETC_TBIER_TXFIE	BIT(1)
#define ENETC_TBIDR	0xa4

//FIXME , BG 0.7: #define ENETC_RTBMR_RSIZE(n)  __ilog2_u32((n) >> 6)
#define ENETC_RTBLENR_LEN(n)	(((n) >> 3) << 7)
#define ENETC_TBMR_EN	BIT(31)

/* Port regs, offset: 1_0000h */
#define ENETC_PORT_BASE	0x10000
#define ENETC_PMR	0x00000
#define ENETC_PMR_EN	GENMASK(18, 16)
#define ENETC_PSR	0x00004 /* RO */
#define ENETC_PSIPMR	0x00018
#define ENETC_PSIPMR_SET_UP(n)	(0x1 << (n)) /* n = SI index */
#define ENETC_PSIPMR_SET_MP(n)	(0x1 << ((n) + 8))
#define ENETC_PSIPMR_SET_VP(n)	(0x1 << ((n) + 16))
#define ENETC_PSIPMAR0(n)	(0x00100 + (n) * 0x20) /* n = SI index */
#define ENETC_PSIPMAR1(n)	(0x00104 + (n) * 0x20)
#define ENETC_PSIIVLANR(n)	(0x00210 + (n) * 4) /* n = SI index */
#define ENETC_PSIIVLAN_EN	BIT(31)
#define ENETC_PSIIVLAN_SET_QOS(val)	((u32)(val) << 12)
#define ENETC_PCAPR0	0x00900
#define ENETC_PCAPR1	0x00904

#define ENETC_PV0CFGR(n)	(0x00920 + (n) * 0x10)  /* n = SI index */
#define ENETC_PVCFGR_SET_TXBDR(val)	((val) & 0xff)
#define ENETC_PVCFGR_SET_RXBDR(val)	(((val) & 0xff) << 16)

#define ENETC_PM0_CMD_CFG	0x08008
#define ENETC_PM0_TX_EN		BIT(31)
#define ENETC_PM0_RX_EN		BIT(30)

#define ENETC_PM0_MAXFRM	0x08014
#define ENETC_SET_MAXFRM(val)	((val) << 16)

/* Global regs, offset: 2_0000h */
#define ENETC_GLOBAL_BASE		0x20000
#define ENETC_G_EIPBRR0		0x00bf8
#define ENETC_G_EIPBRR1		0x00bfc

/* PCI device info */
struct enetc_hw {
	/* SI registers, used by all PCI functions */
	void __iomem *reg;
	/* Port registers, PF only */
	void __iomem *port;
	/* IP global registers, PF only */
	void __iomem *global;
};

/* general register accessors */
#define enetc_rd_reg(reg)	ioread32((reg))
#define enetc_wr_reg(reg, val)	iowrite32((val), (reg))
#define enetc_rd(hw, off)	enetc_rd_reg((hw)->reg + (off))
#define enetc_wr(hw, off, val)	enetc_wr_reg((hw)->reg + (off), val)
#define enetc_rd_reg64(reg)		ioread64((reg))
#define enetc_wr_reg64(reg)		iowrite64((val), (reg))
#define enetc_rd64(hw, off)		enetc_rd_reg64((hw)->reg + (off))
#define enetc_wr64(hw, off, val)	enetc_wr_reg64((hw)->reg + (off), val)
/* port register accessors - PF only */
#define enetc_port_rd(hw, off)		enetc_rd_reg((hw)->port + (off))
#define enetc_port_wr(hw, off, val)	enetc_wr_reg((hw)->port + (off), val)
/* global register accessors - PF only */
#define enetc_global_rd(hw, off)	enetc_rd_reg((hw)->global + (off))
#define enetc_global_wr(hw, off, val)	enetc_wr_reg((hw)->global + (off), val)
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

#define ENETC_MAC_ADDR_FILT_CNT	8 /* # of supported entries per port */
#define ENETC_MAX_NUM_VFS	2
#define ENETC_MAC_FILT_PER_SI	(ENETC_MAC_ADDR_FILT_CNT / \
				(ENETC_MAX_NUM_VFS + 1))

struct enetc_cbd {
	union {
		struct {
			__le32 addr[2];
			__le32 opt[4];
		};
		__le32 data[6];
	};
	__le16 index;
	__le16 length;
	u8 cmd;
	u8 cls;
	u8 _res;
	u8 status_flags;
};

#define ENETC_CBD_FLAGS_SF	BIT(7) /* short format */
#define ENETC_CBD_FLAGS_IE	BIT(6) /* interrupt enable */
#define ENETC_CBD_STATUS_MASK	0xf
