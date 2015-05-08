/*
 *  linux/arch/arm/mach-mmp/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <linux/irqchip/arm-gic.h>
#include <asm/mach-types.h>
#include <asm/mcpm.h>
#include <asm/localtimer.h>
#include <asm/smp_scu.h>

#include <mach/irqs.h>
#include <mach/addr-map.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>

#include "common.h"
#include "reset.h"

#ifdef CONFIG_TZ_HYPERVISOR
#include <asm/smp_plat.h>
#include <linux/pxa_tzlc.h>
#endif

#ifdef CONFIG_HAVE_ARM_SCU
void __iomem *scu_get_base_addr(void)
{
	return SCU_VIRT_BASE;
}
#endif

static inline unsigned int get_core_count(void)
{
	u32 ret = 1;

	if (cpu_is_pxa1088() || cpu_is_pxa1L88()) {
		/* Read L2 control register */
		asm volatile("mrc p15, 1, %0, c9, c0, 2" : "=r"(ret));
		/* core count : [25:24] of L2 register + 1 */
		ret = ((ret>>24) & 3) + 1;
	} else {
#ifdef CONFIG_HAVE_ARM_SCU
		ret = scu_get_core_count(scu_get_base_addr());
#endif
	}

	return ret;
}
#ifdef CONFIG_CPU_PXA988
void pxa988_gic_raise_softirq(const struct cpumask *mask, unsigned int irq)
{
	unsigned int val = 0;
	int targ_cpu;
#ifdef CONFIG_TZ_HYPERVISOR
	int cpu;
	unsigned long map = 0;
	tzlc_cmd_desc cmd_desc;
	tzlc_handle tzlc_handle;

	/* Convert our logical CPU mask into a physical one. */
	for_each_cpu(cpu, mask)
		map |= 1 << cpu_logical_map(cpu);

	/*
	 * Ensure that stores to Normal memory are visible to the
	 * other CPUs before issuing the IPI.
	 */
	dsb();

	tzlc_handle = pxa_tzlc_create_handle();

	cmd_desc.op = TZLC_CMD_TRIGER_SGI;
	cmd_desc.args[0] = map << 16 | irq;
	pxa_tzlc_cmd_op(tzlc_handle, &cmd_desc);

	pxa_tzlc_destroy_handle(tzlc_handle);
#else
	gic_raise_softirq(mask, irq);
#endif

	/*
	 * Set the wakeup bits to make sure the core(s) can respond to
	 * the IPI interrupt.
	 * If the target core(s) is alive, this operation is ignored by
	 * the APMU. After the core wakes up, these corresponding bits
	 * are clearly automatically by PMU hardware.
	 */
	preempt_disable();
	for_each_cpu(targ_cpu, mask) {
		BUG_ON(targ_cpu >= CONFIG_NR_CPUS);
		val |= APMU_WAKEUP_CORE(targ_cpu);
	}
	__raw_writel(val, APMU_COREn_WAKEUP_CTL(smp_processor_id()));
	preempt_enable();
}
#endif

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init mmp_smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	if (ncores > nr_cpu_ids) {
		pr_warn("SMP: %u cores greater than maximum (%u), clipping\n",
			ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);

	if (cpu_is_pxa988())
		scu_enable(scu_get_base_addr());

#ifdef CONFIG_CPU_PXA988
	set_smp_cross_call(pxa988_gic_raise_softirq);
#endif
}

void __init mmp_smp_init_ops(void)
{
	mmp_smp_init_cpus();
	mmp_entry_vector_init();
	mcpm_smp_set_ops();
}
