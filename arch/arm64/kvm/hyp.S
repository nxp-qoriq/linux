/*
 * Copyright (C) 2012,2013 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
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

#include <linux/linkage.h>

#include <asm/alternative.h>
#include <asm/assembler.h>
#include <asm/cpufeature.h>

/*
 * u64 __kvm_call_hyp(void *hypfn, ...);
 *
 * This is not really a variadic function in the classic C-way and care must
 * be taken when calling this to ensure parameters are passed in registers
 * only, since the stack will change between the caller and the callee.
 *
 * Call the function with the first argument containing a pointer to the
 * function you wish to call in Hyp mode, and subsequent arguments will be
 * passed as x0, x1, and x2 (a maximum of 3 arguments in addition to the
 * function pointer can be passed).  The function being called must be mapped
 * in Hyp mode (see init_hyp_mode in arch/arm/kvm/arm.c).  Return values are
 * passed in x0.
 *
 * A function pointer with a value less than 0xfff has a special meaning,
 * and is used to implement __hyp_get_vectors in the same way as in
 * arch/arm64/kernel/hyp_stub.S.
 */
ENTRY(__kvm_call_hyp)
alternative_if_not ARM64_HAS_VIRT_HOST_EXTN
	str     lr, [sp, #-16]!
	hvc	#0
	ldr     lr, [sp], #16
	ret
alternative_else
	b   __vhe_hyp_call
	nop
	nop
	nop
alternative_endif
ENDPROC(__kvm_call_hyp)
#if 0
.macro invalid_vector	label, target
	.align	2
\label:
	b \target
ENDPROC(\label)
.endm

	/* None of these should ever happen */
	invalid_vector	el2t_sync_invalid, __kvm_hyp_panic
	invalid_vector	el2t_irq_invalid, __kvm_hyp_panic
	invalid_vector	el2t_fiq_invalid, __kvm_hyp_panic
	invalid_vector	el2t_error_invalid, __kvm_hyp_panic
	invalid_vector	el2h_sync_invalid, __kvm_hyp_panic
	invalid_vector	el2h_irq_invalid, __kvm_hyp_panic
	invalid_vector	el2h_fiq_invalid, __kvm_hyp_panic
	invalid_vector	el2h_error_invalid, __kvm_hyp_panic
	invalid_vector	el1_sync_invalid, __kvm_hyp_panic
	invalid_vector	el1_irq_invalid, __kvm_hyp_panic
	invalid_vector	el1_fiq_invalid, __kvm_hyp_panic
	invalid_vector	el1_error_invalid, __kvm_hyp_panic

el1_sync:					// Guest trapped into EL2
	push	x0, x1
	push	x2, x3

	mrs	x1, esr_el2
	lsr	x2, x1, #ESR_ELx_EC_SHIFT

	cmp	x2, #ESR_ELx_EC_HVC64
	b.ne	el1_trap

	mrs	x3, vttbr_el2			// If vttbr is valid, the 64bit guest
	cbnz	x3, el1_trap			// called HVC

	/* Here, we're pretty sure the host called HVC. */
	pop	x2, x3
	pop	x0, x1

	/* Check for __hyp_get_vectors */
	cbnz	x0, 1f
	mrs	x0, vbar_el2
	b	2f

1:	push	lr, xzr

	/*
	 * Compute the function address in EL2, and shuffle the parameters.
	 */
	kern_hyp_va	x0
	mov	lr, x0
	mov	x0, x1
	mov	x1, x2
	mov	x2, x3
	blr	lr

	pop	lr, xzr
2:	eret

el1_trap:
	/*
	 * x1: ESR
	 * x2: ESR_EC
	 */

	/* Guest accessed VFP/SIMD registers, save host, restore Guest */
	cmp	x2, #ESR_ELx_EC_FP_ASIMD
	b.eq	switch_to_guest_fpsimd

	cmp	x2, #ESR_ELx_EC_DABT_LOW
	mov	x0, #ESR_ELx_EC_IABT_LOW
	ccmp	x2, x0, #4, ne
	b.ne	1f		// Not an abort we care about

	/* This is an abort. Check for permission fault */
alternative_if_not ARM64_WORKAROUND_834220
	and	x2, x1, #ESR_ELx_FSC_TYPE
	cmp	x2, #FSC_PERM
	b.ne	1f		// Not a permission fault
alternative_else
	nop			// Use the permission fault path to
	nop			// check for a valid S1 translation,
	nop			// regardless of the ESR value.
alternative_endif

	/*
	 * Check for Stage-1 page table walk, which is guaranteed
	 * to give a valid HPFAR_EL2.
	 */
	tbnz	x1, #7, 1f	// S1PTW is set

	/* Preserve PAR_EL1 */
	mrs	x3, par_el1
	push	x3, xzr

	/*
	 * Permission fault, HPFAR_EL2 is invalid.
	 * Resolve the IPA the hard way using the guest VA.
	 * Stage-1 translation already validated the memory access rights.
	 * As such, we can use the EL1 translation regime, and don't have
	 * to distinguish between EL0 and EL1 access.
	 */
	mrs	x2, far_el2
	at	s1e1r, x2
	isb

	/* Read result */
	mrs	x3, par_el1
	pop	x0, xzr			// Restore PAR_EL1 from the stack
	msr	par_el1, x0
	tbnz	x3, #0, 3f		// Bail out if we failed the translation
	ubfx	x3, x3, #12, #36	// Extract IPA
	lsl	x3, x3, #4		// and present it like HPFAR
	b	2f

1:	mrs	x3, hpfar_el2
	mrs	x2, far_el2

2:	mrs	x0, tpidr_el2
	str	w1, [x0, #VCPU_ESR_EL2]
	str	x2, [x0, #VCPU_FAR_EL2]
	str	x3, [x0, #VCPU_HPFAR_EL2]

	mov	x1, #ARM_EXCEPTION_TRAP
	b	__kvm_vcpu_return

	/*
	 * Translation failed. Just return to the guest and
	 * let it fault again. Another CPU is probably playing
	 * behind our back.
	 */
3:	pop	x2, x3
	pop	x0, x1

	eret

el1_irq:
	push	x0, x1
	push	x2, x3
	mrs	x0, tpidr_el2
	mov	x1, #ARM_EXCEPTION_IRQ
	b	__kvm_vcpu_return

	.ltorg

	.align 11

ENTRY(__kvm_hyp_vector)
	ventry	el2t_sync_invalid		// Synchronous EL2t
	ventry	el2t_irq_invalid		// IRQ EL2t
	ventry	el2t_fiq_invalid		// FIQ EL2t
	ventry	el2t_error_invalid		// Error EL2t

	ventry	el2h_sync_invalid		// Synchronous EL2h
	ventry	el2h_irq_invalid		// IRQ EL2h
	ventry	el2h_fiq_invalid		// FIQ EL2h
	ventry	el2h_error_invalid		// Error EL2h

	ventry	el1_sync			// Synchronous 64-bit EL1
	ventry	el1_irq				// IRQ 64-bit EL1
	ventry	el1_fiq_invalid			// FIQ 64-bit EL1
	ventry	el1_error_invalid		// Error 64-bit EL1

	ventry	el1_sync			// Synchronous 32-bit EL1
	ventry	el1_irq				// IRQ 32-bit EL1
	ventry	el1_fiq_invalid			// FIQ 32-bit EL1
	ventry	el1_error_invalid		// Error 32-bit EL1
ENDPROC(__kvm_hyp_vector)


ENTRY(__kvm_get_mdcr_el2)
	mrs	x0, mdcr_el2
	ret
ENDPROC(__kvm_get_mdcr_el2)

	.popsection
#endif
