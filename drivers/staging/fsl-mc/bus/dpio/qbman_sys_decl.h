/* Copyright (C) 2014 Freescale Semiconductor, Inc.
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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/memblock.h>
#include <linux/completion.h>
#include <linux/log2.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include "fsl_qbman_base.h"

/* The platform-independent code shouldn't need endianness, except for
 * weird/fast-path cases like qbman_result_has_token(), which needs to
 * perform a passive and endianness-specific test on a read-only data structure
 * very quickly. It's an exception, and this symbol is used for that case. */
#if defined(__BIG_ENDIAN)
#define DQRR_TOK_OFFSET 0
#define QBMAN_RESULT_VERB_OFFSET_IN_MEM 24
#define SCN_STATE_OFFSET_IN_MEM 8
#define SCN_RID_OFFSET_IN_MEM 8
#else
#define DQRR_TOK_OFFSET 24
#define QBMAN_RESULT_VERB_OFFSET_IN_MEM 0
#define SCN_STATE_OFFSET_IN_MEM 16
#define SCN_RID_OFFSET_IN_MEM 0
#endif

/* Similarly-named functions */
#define upper32(a) upper_32_bits(a)
#define lower32(a) lower_32_bits(a)

	/****************/
	/* arch assists */
	/****************/

#define dcbz(p) { asm volatile("dc zva, %0" : : "r" (p) : "memory"); }
#define lwsync() { asm volatile("dmb st" : : : "memory"); }
#define dcbf(p) { asm volatile("dc cvac, %0;" : : "r" (p) : "memory"); }
#define dcivac(p) { asm volatile("dc ivac, %0" : : "r"(p) : "memory"); }
static inline void prefetch_for_load(void *p)
{
	asm volatile("prfm pldl1keep, [%0, #64]" : : "r" (p));
}
static inline void prefetch_for_store(void *p)
{
	asm volatile("prfm pstl1keep, [%0, #64]" : : "r" (p));
}
