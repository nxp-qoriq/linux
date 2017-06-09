/* Copyright 2015 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of NXP Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NXP Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DPAA2_QDMA_H
#define __DPAA2_QDMA_H

#define LONG_FORMAT 1

#define DPAA2_QDMA_STORE_SIZE 16
#define NUM_CH 8
#define NUM_SG_PER_BLK	16

#define QDMA_DMR_OFFSET	0x0
#define QDMA_DQ_EN (0 << 30)
#define QDMA_DQ_DIS (1 << 30)

#define QDMA_DSR_M_OFFSET 0x10004

struct dpaa2_qdma_sd_d {
	uint32_t rsv:32;
	union {
		struct {
			uint32_t ssd:12; /* souce stride distance */
			uint32_t sss:12; /* souce stride size */
			uint32_t rsv1:8;
		} sdf;
		struct {
			uint32_t dsd:12; /* Destination stride distance */
			uint32_t dss:12; /* Destination stride size */
			uint32_t rsv2:8;
		} ddf;
	} df;
	uint32_t rbpcmd;	/* Route-by-port command */
	uint32_t cmd;
} __attribute__((__packed__));
/* Source descriptor command read transaction type for RBP=0:
 coherent copy of cacheable memory */
#define QDMA_SD_CMD_RDTTYPE_COHERENT (0xb << 28)
/* Destination descriptor command write transaction type for RBP=0:
 coherent copy of cacheable memory */
#define QDMA_DD_CMD_WRTTYPE_COHERENT (0x6 << 28)

#define QDMA_SG_FMT_SDB		0x0 /* single data buffer */
#define QDMA_SG_FMT_FDS		0x1 /* frame data section */
#define QDMA_SG_FMT_SGTE	0x2 /* SGT extension */
#define QDMA_SG_SL_SHORT	0x1 /* short length */
#define QDMA_SG_SL_LONG		0x0 /* short length */
#define QDMA_SG_F		0x1 /* last sg entry */
struct dpaa2_qdma_sg {
	uint32_t addr_lo;		/* address 0:31 */
	uint32_t addr_hi:17;	/* address 32:48 */
	uint32_t rsv:15;
	union {
		uint32_t data_len_sl0; /* SL=0, the long format */
		struct {
			uint32_t len:17; /* SL=1, the short format */
			uint32_t reserve:3;
			uint32_t sf:1;
			uint32_t sr:1;
			uint32_t size:10; /* buff size */
		} data_len_sl1;
	} data_len;	/* AVAIL_LENGTH */
	struct {
		uint32_t bpid:14;
		uint32_t ivp:1;
		uint32_t mbt:1;
		uint32_t offset:12;
		uint32_t fmt:2;
		uint32_t sl:1;
		uint32_t f:1;
	} ctrl;
} __attribute__((__packed__));

#define QMAN_FD_FMT_ENABLE (1 << 12) /* frame list table enable */
#define QMAN_FD_BMT_ENABLE (1 << 15) /* bypass memory translation */
#define QMAN_FD_BMT_DISABLE (0 << 15) /* bypass memory translation */
#define QMAN_FD_SL_DISABLE (0 << 14) /* short lengthe disabled */
#define QMAN_FD_SL_ENABLE (1 << 14) /* short lengthe enabled */

#define QDMA_SB_FRAME (0 << 28) /* single frame */
#define QDMA_SG_FRAME (2 << 28) /* scatter gather frames */
#define QDMA_FINAL_BIT_DISABLE (0 << 31) /* final bit disable */
#define QDMA_FINAL_BIT_ENABLE (1 << 31) /* final bit enable */

#define QDMA_FD_SHORT_FORMAT (1 << 11) /* short format */
#define QDMA_FD_LONG_FORMAT (0 << 11) /* long format */
#define QDMA_SER_DISABLE (0 << 8) /* no notification */
#define QDMA_SER_CTX (1 << 8) /* notification by FQD_CTX[fqid] */
#define QDMA_SER_DEST (2 << 8) /* notification by destination desc */
#define QDMA_SER_BOTH (3 << 8) /* soruce and dest notification */
#define QDMA_FD_SPF_ENALBE (1 << 30) /* source prefetch enable */

#define QMAN_FD_VA_ENABLE (1 << 14)  /* Address used is virtual address */
#define QMAN_FD_VA_DISABLE (0 << 14)/* Address used is a real address */
#define QMAN_FD_CBMT_ENABLE (1 << 15) /* Flow Context: 49bit physical address */
#define QMAN_FD_CBMT_DISABLE (0 << 15) /* Flow Context: 64bit virtual address */
#define QMAN_FD_SC_DISABLE (0 << 27) /* stashing control */

#define QDMA_FL_FMT_SBF 0x0	/* Single buffer frame */
#define QDMA_FL_FMT_SGE 0x2 /* Scatter gather frame */
#define QDMA_FL_BMT_ENABLE 0x1 /* enable bypass memory translation */
#define QDMA_FL_BMT_DISABLE 0x0 /* enable bypass memory translation */
#define QDMA_FL_SL_LONG 0x0 /* long length */
#define QDMA_FL_SL_SHORT 0x1 /* short length */
#define QDMA_FL_F 0x1 /* last frame list bit */
/*Description of Frame list table structure*/
struct dpaa2_frame_list  {
	uint32_t addr_lo;	/* lower 32 bits of address */
	uint32_t addr_hi:17; /* upper 17 bits of address */
	uint32_t resrvd:15;
	union {
		uint32_t data_len_sl0; /* If SL=0, then data length is 32 */
		struct {
			uint32_t data_len:18; /* IF SL=1; length is 18bit */
			uint32_t resrvd:2;
			uint32_t mem:12; /* Valid only when SL=1 */
		} data_len_sl1;
	} data_len;
	/* word 4 */
	uint32_t bpid:14; /* Frame buffer pool ID */
	uint32_t ivp:1; /* Invalid Pool ID. */
	uint32_t bmt:1; /* Bypass Memory Translation */
	uint32_t offset:12; /* Frame offset */
	uint32_t fmt:2; /* Frame Format */
	uint32_t sl:1; /* Short Length */
	uint32_t f:1; /* Final bit */

	uint32_t frc; /* Frame Context */
	/* word 6 */
	uint32_t err:8; /* Frame errors */
	uint32_t resrvd0:8;
	uint32_t asal:4; /* accelerator-specific annotation length */
	uint32_t resrvd1:1;
	uint32_t ptv2:1;
	uint32_t ptv1:1;
	uint32_t pta:1; /* pass-through annotation */
	uint32_t resrvd2:8;

	uint32_t flc_lo; /* lower 32 bits fo flow context */
	uint32_t flc_hi; /* higher 32 bits fo flow context */
} __attribute__((__packed__));

struct dpaa2_qdma_chan {
	struct virt_dma_chan		vchan;
	struct virt_dma_desc		vdesc;
	enum dma_status			status;
	struct dpaa2_qdma_engine	*qdma;

	struct mutex            dpaa2_queue_mutex;
	spinlock_t		queue_lock;
	struct dma_pool		*fd_pool;
	struct dma_pool		*sg_blk_pool;

	struct list_head	comp_used;
	struct list_head	comp_free;

	struct list_head	sgb_free;
};

struct qdma_sg_blk {
	dma_addr_t	blk_bus_addr;
	void		*blk_virt_addr;
	struct list_head	list;
};

struct dpaa2_qdma_comp {
	dma_addr_t		fd_bus_addr;
	dma_addr_t		fl_bus_addr;
	dma_addr_t		desc_bus_addr;
	dma_addr_t		sge_src_bus_addr;
	dma_addr_t		sge_dst_bus_addr;
	void			*fd_virt_addr;
	void			*fl_virt_addr;
	void			*desc_virt_addr;
	void			*sg_src_virt_addr;
	void			*sg_dst_virt_addr;
	struct qdma_sg_blk	*sg_blk;
	uint32_t		sg_blk_num;
	struct list_head	sg_src_head;
	struct list_head	sg_dst_head;
	struct dpaa2_qdma_chan	*qchan;
	struct virt_dma_desc	vdesc;
	struct list_head	list;
};

struct dpaa2_qdma_engine {
	struct dma_device	dma_dev;
	u32			n_chans;
	struct dpaa2_qdma_chan	chans[NUM_CH];

	struct dpaa2_qdma_priv *priv;
};

/*
 * dpaa2_qdma_priv - driver private data
 */
struct dpaa2_qdma_priv {
	int dpqdma_id;

	struct iommu_domain *iommu_domain;
	struct dpdmai_attr dpdmai_attr;
	struct device *dev;
	struct fsl_mc_io *mc_io;
	struct fsl_mc_device *dpdmai_dev;

	struct dpdmai_rx_queue_attr rx_queue_attr[DPDMAI_PRIO_NUM];
	struct dpdmai_tx_queue_attr tx_queue_attr[DPDMAI_PRIO_NUM];

	uint8_t num_pairs;

	struct dpaa2_qdma_engine *dpaa2_qdma;
	struct dpaa2_qdma_priv_per_prio *ppriv;
};

struct dpaa2_qdma_priv_per_prio {
	int req_fqid;
	int rsp_fqid;
	int prio;

	struct dpaa2_io_store *store;
	struct dpaa2_io_notification_ctx nctx;

	struct dpaa2_qdma_priv *priv;
};

/* FD pool size: one FD + 3 Frame list + 2 source/destination descriptor */
#define FD_POOL_SIZE (sizeof(struct dpaa2_fd) + \
		sizeof(struct dpaa2_frame_list) * 3 + \
		sizeof(struct dpaa2_qdma_sd_d) * 2)

/* qdma_sg_blk + 16 SGs */
#define SG_POOL_SIZE (sizeof(struct qdma_sg_blk) +\
		sizeof(struct dpaa2_qdma_sg) * NUM_SG_PER_BLK)
#endif /* __DPAA2_QDMA_H */
