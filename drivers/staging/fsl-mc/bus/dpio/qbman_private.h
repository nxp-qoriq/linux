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

/* Perform extra checking */
#define QBMAN_CHECKING

/* To maximise the amount of logic that is common between the Linux driver and
 * other targets (such as the embedded MC firmware), we pivot here between the
 * inclusion of two platform-specific headers.
 *
 * The first, qbman_sys_decl.h, includes any and all required system headers as
 * well as providing any definitions for the purposes of compatibility. The
 * second, qbman_sys.h, is where platform-specific routines go.
 *
 * The point of the split is that the platform-independent code (including this
 * header) may depend on platform-specific declarations, yet other
 * platform-specific routines may depend on platform-independent definitions.
 */

#define QMAN_REV_4000   0x04000000
#define QMAN_REV_4100   0x04010000
#define QMAN_REV_4101   0x04010001

/* When things go wrong, it is a convenient trick to insert a few FOO()
 * statements in the code to trace progress. TODO: remove this once we are
 * hacking the code less actively.
 */
#define FOO() fsl_os_print("FOO: %s:%d\n", __FILE__, __LINE__)

/* Any time there is a register interface which we poll on, this provides a
 * "break after x iterations" scheme for it. It's handy for debugging, eg.
 * where you don't want millions of lines of log output from a polling loop
 * that won't, because such things tend to drown out the earlier log output
 * that might explain what caused the problem. (NB: put ";" after each macro!)
 * TODO: we should probably remove this once we're done sanitising the
 * simulator...
 */
#define DBG_POLL_START(loopvar) (loopvar = 1000)
#define DBG_POLL_CHECK(loopvar) \
	do {if (!((loopvar)--)) WARN_ON(1); } while (0)

/* For CCSR or portal-CINH registers that contain fields at arbitrary offsets
 * and widths, these macro-generated encode/decode/isolate/remove inlines can
 * be used.
 *
 * Eg. to "d"ecode a 14-bit field out of a register (into a "u16" type),
 * where the field is located 3 bits "up" from the least-significant bit of the
 * register (ie. the field location within the 32-bit register corresponds to a
 * mask of 0x0001fff8), you would do;
 *                u16 field = d32_u16(3, 14, reg_value);
 *
 * Or to "e"ncode a 1-bit boolean value (input type is "int", zero is FALSE,
 * non-zero is TRUE, so must convert all non-zero inputs to 1, hence the "!!"
 * operator) into a register at bit location 0x00080000 (19 bits "in" from the
 * LS bit), do;
 *                reg_value |= e32_int(19, 1, !!field);
 *
 * If you wish to read-modify-write a register, such that you leave the 14-bit
 * field as-is but have all other fields set to zero, then "i"solate the 14-bit
 * value using;
 *                reg_value = i32_u16(3, 14, reg_value);
 *
 * Alternatively, you could "r"emove the 1-bit boolean field (setting it to
 * zero) but leaving all other fields as-is;
 *                reg_val = r32_int(19, 1, reg_value);
 *
 */
#define MAKE_MASK32(width) (width == 32 ? 0xffffffff : \
				 (u32)((1 << width) - 1))
#define DECLARE_CODEC32(t) \
static inline u32 e32_##t(u32 lsoffset, u32 width, t val) \
{ \
	WARN_ON(width > (sizeof(t) * 8)); \
	return ((u32)val & MAKE_MASK32(width)) << lsoffset; \
} \
static inline t d32_##t(u32 lsoffset, u32 width, u32 val) \
{ \
	WARN_ON(width > (sizeof(t) * 8)); \
	return (t)((val >> lsoffset) & MAKE_MASK32(width)); \
} \
static inline u32 i32_##t(u32 lsoffset, u32 width, \
				u32 val) \
{ \
	WARN_ON(width > (sizeof(t) * 8)); \
	return e32_##t(lsoffset, width, d32_##t(lsoffset, width, val)); \
} \
static inline u32 r32_##t(u32 lsoffset, u32 width, \
				u32 val) \
{ \
	WARN_ON(width > (sizeof(t) * 8)); \
	return ~(MAKE_MASK32(width) << lsoffset) & val; \
}
DECLARE_CODEC32(u32)
DECLARE_CODEC32(u16)
DECLARE_CODEC32(u8)
DECLARE_CODEC32(int)

	/*********************/
	/* Debugging assists */
	/*********************/

static inline void __hexdump(unsigned long start, unsigned long end,
			     unsigned long p, size_t sz,
			     const unsigned char *c)
{
	while (start < end) {
		unsigned int pos = 0;
		char buf[64];
		int nl = 0;

		pos += sprintf(buf + pos, "%08lx: ", start);
		do {
			if ((start < p) || (start >= (p + sz)))
				pos += sprintf(buf + pos, "..");
			else
				pos += sprintf(buf + pos, "%02x", *(c++));
			if (!(++start & 15)) {
				buf[pos++] = '\n';
				nl = 1;
			} else {
				nl = 0;
				if (!(start & 1))
					buf[pos++] = ' ';
				if (!(start & 3))
					buf[pos++] = ' ';
			}
		} while (start & 15);
		if (!nl)
			buf[pos++] = '\n';
		buf[pos] = '\0';
		pr_info("%s", buf);
	}
}

static inline void hexdump(const void *ptr, size_t sz)
{
	unsigned long p = (unsigned long)ptr;
	unsigned long start = p & ~15ul;
	unsigned long end = (p + sz + 15) & ~15ul;
	const unsigned char *c = ptr;

	__hexdump(start, end, p, sz, c);
}
