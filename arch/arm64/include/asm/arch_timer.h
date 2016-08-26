/*
 * arch/arm64/include/asm/arch_timer.h
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_ARCH_TIMER_H
#define __ASM_ARCH_TIMER_H

#include <asm/barrier.h>

#include <linux/bug.h>
#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/types.h>

#include <clocksource/arm_arch_timer.h>

extern struct static_key_false arch_timer_read_ool_enabled;

#define ARCH_TIMER_REG_READ(reg, func) \
extern u64 func##_ool(void); \
static inline u64 __##func(void) \
{ \
	u64 val; \
	asm volatile("mrs %0, " reg : "=r" (val)); \
	return val; \
} \
static inline u64 _##func(void) \
{ \
	if (IS_ENABLED(CONFIG_FSL_ERRATUM_A008585) && \
	    static_branch_unlikely(&arch_timer_read_ool_enabled)) \
		return func##_ool(); \
	else \
		return __##func(); \
}

ARCH_TIMER_REG_READ("cntp_tval_el0", arch_timer_get_ptval)
ARCH_TIMER_REG_READ("cntv_tval_el0", arch_timer_get_vtval)
ARCH_TIMER_REG_READ("cntvct_el0", arch_counter_get_cntvct)

#undef ARCH_TIMER_REG_READ

/*
 * These register accessors are marked inline so the compiler can
 * nicely work out which register we want, and chuck away the rest of
 * the code.
 */
static __always_inline
void arch_timer_reg_write_cp15(int access, enum arch_timer_reg reg, u32 val)
{
	if (access == ARCH_TIMER_PHYS_ACCESS) {
		switch (reg) {
		case ARCH_TIMER_REG_CTRL:
			asm volatile("msr cntp_ctl_el0,  %0" : : "r" (val));
			break;
		case ARCH_TIMER_REG_TVAL:
			asm volatile("msr cntp_tval_el0, %0" : : "r" (val));
			break;
		}
	} else if (access == ARCH_TIMER_VIRT_ACCESS) {
		switch (reg) {
		case ARCH_TIMER_REG_CTRL:
			asm volatile("msr cntv_ctl_el0,  %0" : : "r" (val));
			break;
		case ARCH_TIMER_REG_TVAL:
			asm volatile("msr cntv_tval_el0, %0" : : "r" (val));
			break;
		}
	}

	isb();
}

static __always_inline void arch_timer_cval_write_cp15(int access, u64 val)
{
	if (access == ARCH_TIMER_PHYS_ACCESS)
		asm volatile("msr cntp_cval_el0, %0" : : "r" (val));
	else if (access == ARCH_TIMER_VIRT_ACCESS)
		asm volatile("msr cntv_cval_el0, %0" : : "r" (val));

	isb();
}

static __always_inline
u32 arch_timer_reg_read_cp15(int access, enum arch_timer_reg reg)
{
	u32 val;

	if (access == ARCH_TIMER_PHYS_ACCESS) {
		switch (reg) {
		case ARCH_TIMER_REG_CTRL:
			asm volatile("mrs %0, cntp_ctl_el0" : "=r" (val));
			break;
		case ARCH_TIMER_REG_TVAL:
			val = _arch_timer_get_ptval();
			break;
		}
	} else if (access == ARCH_TIMER_VIRT_ACCESS) {
		switch (reg) {
		case ARCH_TIMER_REG_CTRL:
			asm volatile("mrs %0, cntv_ctl_el0" : "=r" (val));
			break;
		case ARCH_TIMER_REG_TVAL:
			val = _arch_timer_get_vtval();
			break;
		}
	}

	return val;
}

static inline u32 arch_timer_get_cntfrq(void)
{
	u32 val;
	asm volatile("mrs %0,   cntfrq_el0" : "=r" (val));
	return val;
}

static inline u32 arch_timer_get_cntkctl(void)
{
	u32 cntkctl;
	asm volatile("mrs	%0, cntkctl_el1" : "=r" (cntkctl));
	return cntkctl;
}

static inline void arch_timer_set_cntkctl(u32 cntkctl)
{
	asm volatile("msr	cntkctl_el1, %0" : : "r" (cntkctl));
}

static inline u64 arch_counter_get_cntpct(void)
{
	/*
	 * AArch64 kernel and user space mandate the use of CNTVCT.
	 */
	BUG();
	return 0;
}

static inline u64 arch_counter_get_cntvct(void)
{
	isb();
	return _arch_counter_get_cntvct();
}

static inline int arch_timer_arch_init(void)
{
	return 0;
}

#endif
