/* Copyright 2014 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __FSL_DPAA2_FD_H
#define __FSL_DPAA2_FD_H

/**
 * DOC: DPAA2 FD - Frame Descriptor APIs for DPAA2
 *
 * Frame Descriptors (FDs) are used to describe frame data in the DPAA2.
 * Frames can be enqueued and dequeued to Frame Queues which are consumed
 * by the various DPAA accelerators (WRIOP, SEC, PME, DCE)
 *
 * There are three types of frames: Single, Scatter Gather and Frame Lists.
 *
 * The set of APIs in this file must be used to create, manipulate and
 * query Frame Descriptor.
 *
 */

/**
 * struct dpaa2_fd - Place-holder for FDs.
 * @words: for easier/faster copying the whole FD structure.
 * @addr_lo: the lower 32 bits of the address in FD.
 * @addr_hi: the upper 32 bits of the address in FD.
 * @len: the length field in FD.
 * @bpid_offset: represent the bpid and offset fields in FD
 * @frc: frame context
 * @ctrl: the 32bit control bits including dd, sc,... va, err.
 * @flc_lo: the lower 32bit of flow context.
 * @flc_hi: the upper 32bits of flow context.
 *
 * This structure represents the basic Frame Descriptor used in the system.
 * We represent it via the simplest form that we need for now. Different
 * overlays may be needed to support different options, etc. (It is impractical
 * to define One True Struct, because the resulting encoding routines (lots of
 * read-modify-writes) would be worst-case performance whether or not
 * circumstances required them.)
 */
struct dpaa2_fd {
	union {
		u32 words[8];
		struct dpaa2_fd_simple {
			u32 addr_lo;
			u32 addr_hi;
			u32 len;
			/* offset in the MS 16 bits, BPID in the LS 16 bits */
			u32 bpid_offset;
			u32 frc; /* frame context */
			/* "err", "va", "cbmt", "asal", [...] */
			u32 ctrl;
			/* flow context */
			u32 flc_lo;
			u32 flc_hi;
		} simple;
	};
};

enum dpaa2_fd_format {
	dpaa2_fd_single = 0,
	dpaa2_fd_list,
	dpaa2_fd_sg
};

/* Accessors for SG entry fields
 *
 * These setters and getters assume little endian format. For converting
 * between LE and cpu endianness, the specific conversion functions must be
 * called before the SGE contents are accessed by the core (on Rx),
 * respectively before the SG table is sent to hardware (on Tx)
 */

/**
 * dpaa2_fd_get_addr() - get the addr field of frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the address in the frame descriptor.
 */
static inline dma_addr_t dpaa2_fd_get_addr(const struct dpaa2_fd *fd)
{
	return (dma_addr_t)((((uint64_t)fd->simple.addr_hi) << 32)
				+ fd->simple.addr_lo);
}

/**
 * dpaa2_fd_set_addr() - Set the addr field of frame descriptor
 * @fd: the given frame descriptor.
 * @addr: the address needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_addr(struct dpaa2_fd *fd, dma_addr_t addr)
{
	fd->simple.addr_hi = upper_32_bits(addr);
	fd->simple.addr_lo = lower_32_bits(addr);
}

/**
 * dpaa2_fd_get_frc() - Get the frame context in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the frame context field in the frame descriptor.
 */
static inline u32 dpaa2_fd_get_frc(const struct dpaa2_fd *fd)
{
	return fd->simple.frc;
}

/**
 * dpaa2_fd_set_frc() - Set the frame context in the frame descriptor
 * @fd: the given frame descriptor.
 * @frc: the frame context needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_frc(struct dpaa2_fd *fd, u32 frc)
{
	fd->simple.frc = frc;
}

/**
 * dpaa2_fd_get_flc() - Get the flow context in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the flow context in the frame descriptor.
 */
static inline dma_addr_t dpaa2_fd_get_flc(const struct dpaa2_fd *fd)
{
	return (dma_addr_t)((((uint64_t)fd->simple.flc_hi) << 32) +
			    fd->simple.flc_lo);
}

/**
 * dpaa2_fd_set_flc() - Set the flow context field of frame descriptor
 * @fd: the given frame descriptor.
 * @flc_addr: the flow context needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_flc(struct dpaa2_fd *fd,  dma_addr_t flc_addr)
{
	fd->simple.flc_hi = upper_32_bits(flc_addr);
	fd->simple.flc_lo = lower_32_bits(flc_addr);
}

/**
 * dpaa2_fd_get_len() - Get the length in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the length field in the frame descriptor.
 */
static inline u32 dpaa2_fd_get_len(const struct dpaa2_fd *fd)
{
	return fd->simple.len;
}

/**
 * dpaa2_fd_set_len() - Set the length field of frame descriptor
 * @fd: the given frame descriptor.
 * @len: the length needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_len(struct dpaa2_fd *fd, u32 len)
{
	fd->simple.len = len;
}

/**
 * dpaa2_fd_get_offset() - Get the offset field in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the offset.
 */
static inline uint16_t dpaa2_fd_get_offset(const struct dpaa2_fd *fd)
{
	return (uint16_t)(fd->simple.bpid_offset >> 16) & 0x0FFF;
}

/**
 * dpaa2_fd_set_offset() - Set the offset field of frame descriptor
 *
 * @fd: the given frame descriptor.
 * @offset: the offset needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_offset(struct dpaa2_fd *fd, uint16_t offset)
{
	fd->simple.bpid_offset &= 0xF000FFFF;
	fd->simple.bpid_offset |= (u32)offset << 16;
}

/**
 * dpaa2_fd_get_format() - Get the format field in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the format.
 */
static inline enum dpaa2_fd_format dpaa2_fd_get_format(
						const struct dpaa2_fd *fd)
{
	return (enum dpaa2_fd_format)((fd->simple.bpid_offset >> 28) & 0x3);
}

/**
 * dpaa2_fd_set_format() - Set the format field of frame descriptor
 *
 * @fd: the given frame descriptor.
 * @format: the format needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_format(struct dpaa2_fd *fd,
				       enum dpaa2_fd_format format)
{
	fd->simple.bpid_offset &= 0xCFFFFFFF;
	fd->simple.bpid_offset |= (u32)format << 28;
}

/**
 * dpaa2_fd_get_bpid() - Get the bpid field in the frame descriptor
 * @fd: the given frame descriptor.
 *
 * Return the bpid.
 */
static inline uint16_t dpaa2_fd_get_bpid(const struct dpaa2_fd *fd)
{
	return (uint16_t)(fd->simple.bpid_offset & 0xFFFF);
}

/**
 * dpaa2_fd_set_bpid() - Set the bpid field of frame descriptor
 *
 * @fd: the given frame descriptor.
 * @bpid: the bpid needs to be set in frame descriptor.
 */
static inline void dpaa2_fd_set_bpid(struct dpaa2_fd *fd, uint16_t bpid)
{
	fd->simple.bpid_offset &= 0xFFFF0000;
	fd->simple.bpid_offset |= (u32)bpid;
}

/**
 * struct dpaa2_sg_entry - the scatter-gathering structure
 * @addr_lo: the lower 32bit of address
 * @addr_hi: the upper 32bit of address
 * @len: the length in this sg entry.
 * @bpid_offset: offset in the MS 16 bits, BPID in the LS 16 bits.
 */
struct dpaa2_sg_entry {
	u32 addr_lo;
	u32 addr_hi;
	u32 len;
	u32 bpid_offset;
};

enum dpaa2_sg_format {
	dpaa2_sg_single = 0,
	dpaa2_sg_frame_data,
	dpaa2_sg_sgt_ext
};

/**
 * dpaa2_sg_get_addr() - Get the address from SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return the address.
 */
static inline dma_addr_t dpaa2_sg_get_addr(const struct dpaa2_sg_entry *sg)
{
	return (dma_addr_t)((((u64)sg->addr_hi) << 32) + sg->addr_lo);
}

/**
 * dpaa2_sg_set_addr() - Set the address in SG entry
 * @sg: the given scatter-gathering object.
 * @addr: the address to be set.
 */
static inline void dpaa2_sg_set_addr(struct dpaa2_sg_entry *sg, dma_addr_t addr)
{
	sg->addr_hi = upper_32_bits(addr);
	sg->addr_lo = lower_32_bits(addr);
}


static inline bool dpaa2_sg_short_len(const struct dpaa2_sg_entry *sg)
{
	return (sg->bpid_offset >> 30) & 0x1;
}

/**
 * dpaa2_sg_get_len() - Get the length in SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return the length.
 */
static inline u32 dpaa2_sg_get_len(const struct dpaa2_sg_entry *sg)
{
	if (dpaa2_sg_short_len(sg))
		return sg->len & 0x1FFFF;
	return sg->len;
}

/**
 * dpaa2_sg_set_len() - Set the length in SG entry
 * @sg: the given scatter-gathering object.
 * @len: the length to be set.
 */
static inline void dpaa2_sg_set_len(struct dpaa2_sg_entry *sg, u32 len)
{
	sg->len = len;
}

/**
 * dpaa2_sg_get_offset() - Get the offset in SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return the offset.
 */
static inline u16 dpaa2_sg_get_offset(const struct dpaa2_sg_entry *sg)
{
	return (u16)(sg->bpid_offset >> 16) & 0x0FFF;
}

/**
 * dpaa2_sg_set_offset() - Set the offset in SG entry
 * @sg: the given scatter-gathering object.
 * @offset: the offset to be set.
 */
static inline void dpaa2_sg_set_offset(struct dpaa2_sg_entry *sg,
				       u16 offset)
{
	sg->bpid_offset &= 0xF000FFFF;
	sg->bpid_offset |= (u32)offset << 16;
}

/**
 * dpaa2_sg_get_format() - Get the SG format in SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return the format.
 */
static inline enum dpaa2_sg_format
	dpaa2_sg_get_format(const struct dpaa2_sg_entry *sg)
{
	return (enum dpaa2_sg_format)((sg->bpid_offset >> 28) & 0x3);
}

/**
 * dpaa2_sg_set_format() - Set the SG format in SG entry
 * @sg: the given scatter-gathering object.
 * @format: the format to be set.
 */
static inline void dpaa2_sg_set_format(struct dpaa2_sg_entry *sg,
				       enum dpaa2_sg_format format)
{
	sg->bpid_offset &= 0xCFFFFFFF;
	sg->bpid_offset |= (u32)format << 28;
}

/**
 * dpaa2_sg_get_bpid() - Get the buffer pool id in SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return the bpid.
 */
static inline u16 dpaa2_sg_get_bpid(const struct dpaa2_sg_entry *sg)
{
	return (u16)(sg->bpid_offset & 0x3FFF);
}

/**
 * dpaa2_sg_set_bpid() - Set the buffer pool id in SG entry
 * @sg: the given scatter-gathering object.
 * @bpid: the bpid to be set.
 */
static inline void dpaa2_sg_set_bpid(struct dpaa2_sg_entry *sg, u16 bpid)
{
	sg->bpid_offset &= 0xFFFFC000;
	sg->bpid_offset |= (u32)bpid;
}

/**
 * dpaa2_sg_is_final() - Check final bit in SG entry
 * @sg: the given scatter-gathering object.
 *
 * Return bool.
 */
static inline bool dpaa2_sg_is_final(const struct dpaa2_sg_entry *sg)
{
	return !!(sg->bpid_offset >> 31);
}

/**
 * dpaa2_sg_set_final() - Set the final bit in SG entry
 * @sg: the given scatter-gathering object.
 * @final: the final boolean to be set.
 */
static inline void dpaa2_sg_set_final(struct dpaa2_sg_entry *sg, bool final)
{
	sg->bpid_offset &= 0x7FFFFFFF;
	sg->bpid_offset |= (u32)final << 31;
}

/* Endianness conversion helper functions
 * The accelerator drivers which construct / read scatter gather entries
 * need to call these in order to account for endianness mismatches between
 * hardware and cpu
 */
#ifdef __BIG_ENDIAN
/**
 * dpaa2_sg_cpu_to_le() - convert scatter gather entry from native cpu
 * format little endian format.
 * @sg: the given scatter gather entry.
 */
static inline void dpaa2_sg_cpu_to_le(struct dpaa2_sg_entry *sg)
{
	uint32_t *p = (uint32_t *)sg;
	int i;

	for (i = 0; i < sizeof(*sg) / sizeof(u32); i++)
		cpu_to_le32s(p++);
}

/**
 * dpaa2_sg_le_to_cpu() - convert scatter gather entry from little endian
 * format to native cpu format.
 * @sg: the given scatter gather entry.
 */
static inline void dpaa2_sg_le_to_cpu(struct dpaa2_sg_entry *sg)
{
	uint32_t *p = (uint32_t *)sg;
	int i;

	for (i = 0; i < sizeof(*sg) / sizeof(u32); i++)
		le32_to_cpus(p++);
}
#else
#define dpaa2_sg_cpu_to_le(sg)
#define dpaa2_sg_le_to_cpu(sg)
#endif /* __BIG_ENDIAN */


/**
 * struct dpaa2_fl_entry - structure for frame list entry.
 * @addr_lo: the lower 32bit of address
 * @addr_hi: the upper 32bit of address
 * @len: the length in this sg entry.
 * @bpid_offset: offset in the MS 16 bits, BPID in the LS 16 bits.
 * @frc: frame context
 * @ctrl: the 32bit control bits including dd, sc,... va, err.
 * @flc_lo: the lower 32bit of flow context.
 * @flc_hi: the upper 32bits of flow context.
 *
 * Frame List Entry (FLE)
 * Identical to dpaa2_fd.simple layout, but some bits are different
 */
struct dpaa2_fl_entry {
	u32 addr_lo;
	u32 addr_hi;
	u32 len;
	u32 bpid_offset;
	u32 frc;
	u32 ctrl;
	u32 flc_lo;
	u32 flc_hi;
};

enum dpaa2_fl_format {
	dpaa2_fl_single = 0,
	dpaa2_fl_res,
	dpaa2_fl_sg
};

/**
 * dpaa2_fl_get_addr() - Get address in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return address for the get function.
 */
static inline dma_addr_t dpaa2_fl_get_addr(const struct dpaa2_fl_entry *fle)
{
	return (dma_addr_t)((((uint64_t)fle->addr_hi) << 32) + fle->addr_lo);
}

/**
 * dpaa2_fl_set_addr() - Set the address in the frame list entry
 * @fle: the given frame list entry.
 * @addr: the address needs to be set.
 *
 */
static inline void dpaa2_fl_set_addr(struct dpaa2_fl_entry *fle,
				     dma_addr_t addr)
{
	fle->addr_hi = upper_32_bits(addr);
	fle->addr_lo = lower_32_bits(addr);
}

/**
 * dpaa2_fl_get_flc() - Get the flow context in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return flow context for the get function.
 */
static inline dma_addr_t dpaa2_fl_get_flc(const struct dpaa2_fl_entry *fle)
{
	return (dma_addr_t)((((uint64_t)fle->flc_hi) << 32) + fle->flc_lo);
}

/**
 * dpaa2_fl_set_flc() - Set the flow context in the frame list entry
 * @fle: the given frame list entry.
 * @flc_addr: the flow context address needs to be set.
 *
 */
static inline void dpaa2_fl_set_flc(struct dpaa2_fl_entry *fle,
				    dma_addr_t flc_addr)
{
	fle->flc_hi = upper_32_bits(flc_addr);
	fle->flc_lo = lower_32_bits(flc_addr);
}

/**
 * dpaa2_fl_get_len() - Get the length in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return length for the get function.
 */
static inline u32 dpaa2_fl_get_len(const struct dpaa2_fl_entry *fle)
{
	return fle->len;
}

/**
 * dpaa2_fl_set_len() - Set the length in the frame list entry
 * @fle: the given frame list entry.
 * @len: the length needs to be set.
 *
 */
static inline void dpaa2_fl_set_len(struct dpaa2_fl_entry *fle, u32 len)
{
	fle->len = len;
}

/**
 * dpaa2_fl_get_offset() - Get/Set the offset in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return offset for the get function.
 */
static inline uint16_t dpaa2_fl_get_offset(const struct dpaa2_fl_entry *fle)
{
	return (uint16_t)(fle->bpid_offset >> 16) & 0x0FFF;
}

/**
 * dpaa2_fl_set_offset() - Set the offset in the frame list entry
 * @fle: the given frame list entry.
 * @offset: the offset needs to be set.
 *
 */
static inline void dpaa2_fl_set_offset(struct dpaa2_fl_entry *fle,
				       uint16_t offset)
{
	fle->bpid_offset &= 0xF000FFFF;
	fle->bpid_offset |= (u32)(offset & 0x0FFF) << 16;
}

/**
 * dpaa2_fl_get_format() - Get the format in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return frame list format for the get function.
 */
static inline enum dpaa2_fl_format dpaa2_fl_get_format(
	const struct dpaa2_fl_entry *fle)
{
	return (enum dpaa2_fl_format)((fle->bpid_offset >> 28) & 0x3);
}

/**
 * dpaa2_fl_set_format() - Set the format in the frame list entry
 * @fle: the given frame list entry.
 * @format: the frame list format needs to be set.
 *
 */
static inline void dpaa2_fl_set_format(struct dpaa2_fl_entry *fle,
				       enum dpaa2_fl_format format)
{
	fle->bpid_offset &= 0xCFFFFFFF;
	fle->bpid_offset |= (u32)(format & 0x3) << 28;
}

/**
 * dpaa2_fl_get_bpid() - Get the buffer pool id in the frame list entry
 * @fle: the given frame list entry.
 *
 * Return bpid for the get function.
 */
static inline uint16_t dpaa2_fl_get_bpid(const struct dpaa2_fl_entry *fle)
{
	return (uint16_t)(fle->bpid_offset & 0x3FFF);
}

/**
 * dpaa2_fl_set_bpid() - Set the buffer pool id in the frame list entry
 * @fle: the given frame list entry.
 * @bpid: the buffer pool id needs to be set.
 *
 */
static inline void dpaa2_fl_set_bpid(struct dpaa2_fl_entry *fle, uint16_t bpid)
{
	fle->bpid_offset &= 0xFFFFC000;
	fle->bpid_offset |= (u32)bpid;
}

/** dpaa2_fl_is_final() - check the final bit is set or not in the frame list.
 * @fle: the given frame list entry.
 *
 * Return final bit settting.
 */
static inline bool dpaa2_fl_is_final(const struct dpaa2_fl_entry *fle)
{
	return !!(fle->bpid_offset >> 31);
}

/**
 * dpaa2_fl_set_final() - Set the final bit in the frame list entry
 * @fle: the given frame list entry.
 * @final: the final bit needs to be set.
 *
 */
static inline void dpaa2_fl_set_final(struct dpaa2_fl_entry *fle, bool final)
{
	fle->bpid_offset &= 0x7FFFFFFF;
	fle->bpid_offset |= (u32)final << 31;
}

/**
 * struct dpaa2_dq - the qman result structure
 * @dont_manipulate_directly: the 16 32bit data to represent the whole
 * possible qman dequeue result.
 *
 * When frames are dequeued, the FDs show up inside "dequeue" result structures
 * (if at all, not all dequeue results contain valid FDs). This structure type
 * is intentionally defined without internal detail, and the only reason it
 * isn't declared opaquely (without size) is to allow the user to provide
 * suitably-sized (and aligned) memory for these entries.
 */
struct dpaa2_dq {
	uint32_t dont_manipulate_directly[16];
};

/* Parsing frame dequeue results */
/* FQ empty */
#define DPAA2_DQ_STAT_FQEMPTY       0x80
/* FQ held active */
#define DPAA2_DQ_STAT_HELDACTIVE    0x40
/* FQ force eligible */
#define DPAA2_DQ_STAT_FORCEELIGIBLE 0x20
/* Valid frame */
#define DPAA2_DQ_STAT_VALIDFRAME    0x10
/* FQ ODP enable */
#define DPAA2_DQ_STAT_ODPVALID      0x04
/* Volatile dequeue */
#define DPAA2_DQ_STAT_VOLATILE      0x02
/* volatile dequeue command is expired */
#define DPAA2_DQ_STAT_EXPIRED       0x01

/**
 * dpaa2_dq_flags() - Get the stat field of dequeue response
 * @dq: the dequeue result.
 */
uint32_t dpaa2_dq_flags(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_is_pull() - Check whether the dq response is from a pull
 * command.
 * @dq: the dequeue result.
 *
 * Return 1 for volatile(pull) dequeue, 0 for static dequeue.
 */
static inline int dpaa2_dq_is_pull(const struct dpaa2_dq *dq)
{
	return (int)(dpaa2_dq_flags(dq) & DPAA2_DQ_STAT_VOLATILE);
}

/**
 * dpaa2_dq_is_pull_complete() - Check whether the pull command is completed.
 * @dq: the dequeue result.
 *
 * Return boolean.
 */
static inline int dpaa2_dq_is_pull_complete(
					const struct dpaa2_dq *dq)
{
	return (int)(dpaa2_dq_flags(dq) & DPAA2_DQ_STAT_EXPIRED);
}

/**
 * dpaa2_dq_seqnum() - Get the seqnum field in dequeue response
 * seqnum is valid only if VALIDFRAME flag is TRUE
 * @dq: the dequeue result.
 *
 * Return seqnum.
 */
uint16_t dpaa2_dq_seqnum(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_odpid() - Get the seqnum field in dequeue response
 * odpid is valid only if ODPVAILD flag is TRUE.
 * @dq: the dequeue result.
 *
 * Return odpid.
 */
uint16_t dpaa2_dq_odpid(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_fqid() - Get the fqid in dequeue response
 * @dq: the dequeue result.
 *
 * Return fqid.
 */
uint32_t dpaa2_dq_fqid(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_byte_count() - Get the byte count in dequeue response
 * @dq: the dequeue result.
 *
 * Return the byte count remaining in the FQ.
 */
uint32_t dpaa2_dq_byte_count(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_frame_count() - Get the frame count in dequeue response
 * @dq: the dequeue result.
 *
 * Return the frame count remaining in the FQ.
 */
uint32_t dpaa2_dq_frame_count(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_fd_ctx() - Get the frame queue context in dequeue response
 * @dq: the dequeue result.
 *
 * Return the frame queue context.
 */
uint64_t dpaa2_dq_fqd_ctx(const struct dpaa2_dq *dq);

/**
 * dpaa2_dq_fd() - Get the frame descriptor in dequeue response
 * @dq: the dequeue result.
 *
 * Return the frame descriptor.
 */
const struct dpaa2_fd *dpaa2_dq_fd(const struct dpaa2_dq *dq);

#endif /* __FSL_DPAA2_FD_H */
