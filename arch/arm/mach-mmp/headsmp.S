/*
 * linux/arch/arm/mach-mmp/headsmp.S
 *
 * Copyright (C) 2012 Marvell, Inc.
 *
 * Author: Neil Zhang <zhangwm@marvell.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/memory.h>
#include <asm/cache.h>
#include <asm/assembler.h>
#include <mach/addr-map.h>

	__CPUINIT

/*
 * Marvell specific entry point for secondary CPUs.
 * The secondary kernel init calls v7_flush_dcache_all before it enables
 * the L1; however, the L1 comes out of reset in an undefined state, so
 * the clean + invalidate performed by v7_flush_dcache_all causes a bunch
 * of cache lines with uninitialized data and uninitialized tags to get
 * written out to memory, which does really unpleasant things to the main
 * processor.  We fix this by performing an invalidate, rather than a
 * clean + invalidate for secondary core, before jumping into the kernel.
 *
 * This funciton is cloned from arch/arm/mach-tegra/headsmp.S, and needs
 * to be called for both secondary cores startup and primary core resume
 * procedures.
 */
	.align L1_CACHE_SHIFT


/*
 * PXA specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until we're
 * ready for them to initialise.
 */
ENTRY(mmp_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #15
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]
	cmp	r7, r0
	bne	pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	bl	v7_invalidate_l1
	b	secondary_startup
ENDPROC(mmp_secondary_startup)

	.align	2
1:	.long	.
	.long	pen_release


/*
 * Note: The following code is located into the .data section. This is to
 *       allow sw_reset_flag and cpu_plugin_handler to be accessed with a
 *       relative load while we can't rely on any MMU translation.
 *       Reference from: arch/arm/kernel/sleep.S
 */

	.data
	.align

/*
 * ROM code jumps to this function while waking up from CPU
 * OFF or software reset state. Physical address of the function is
 * stored at CIU_WARM_RESET_VECTOR while system is bring up.
 */
ENTRY(mmp_cpu_reset_entry)
	adr	r1, mmp_entry_vectors
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #15		@ fetch CPUID
1:
	ldr	r2, [r1, r0, lsl #2]    @ get the handler addr for this core
	cmp	r2, #0
	movne	pc, r2			@ jump to the handler
	beq	1b
ENDPROC(mmp_cpu_reset_entry)

	/* Point to the address that save handlers for each core */
	.global mmp_entry_vectors
mmp_entry_vectors:
	.rept   CONFIG_NR_CPUS
        .long   0                               @ preserve stack phys ptr here
	.endr
