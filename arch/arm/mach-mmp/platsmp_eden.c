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
#include <asm/localtimer.h>
#include <asm/smp_scu.h>

#include <mach/irqs.h>
#include <mach/addr-map.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>

#include "common.h"
#include "reset_eden.h"

/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void __cpuinit write_pen_release(int val)
{
	pen_release = val;
	smp_wmb();
	__cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
	outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

#ifdef CONFIG_HAVE_ARM_SCU
static void __iomem *scu_get_base_addr(void)
{
	return SCU_VIRT_BASE;
}
#endif

static inline unsigned int get_core_count(void)
{
	u32 ret = 1;

	if (cpu_is_pxa1088()) {
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

#ifdef CONFIG_CPU_EDEN
static void __iomem *APMU_CORE_PWRMODE[] = {
	APMU_CORE0_PWRMODE, APMU_CORE1_PWRMODE,
	APMU_CORE2_PWRMODE, APMU_CORE3_PWRMODE,
};

void __cpuinit eden_secondary_init(u32 cpu)
{
	u32 core_pwrmode;

	/*
	 * cleanup APMU_CORE_PWRMODE, so that make sure if APMU's registers
	 * have been set by uboot then here APMU's value is dirty value;
	 * at here cleanup apmu's registers for safe "wfi".
	 */
	core_pwrmode = readl(APMU_CORE_PWRMODE[cpu]);
	core_pwrmode &= ~(PMUA_CPU_PWRMODE_C2 | PMUA_L2SR_PWROFF_VT |
			PMUA_APCORESS_MP2);
	writel(core_pwrmode, APMU_CORE_PWRMODE[cpu]);

	return;
}
#else
#define eden_secondary_init(cpu)	do {} while (0)
#endif

static DEFINE_SPINLOCK(boot_lock);

static void __cpuinit mmp_secondary_init(unsigned int cpu)
{
	if (cpu_is_eden())
		eden_secondary_init(cpu);

	/*
	 * let the primary processor know we're out of the
	 * pen, then head off into the C entry point
	 */
	write_pen_release(-1);

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static int __cpuinit mmp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long timeout;

	/*
	 * Avoid timer calibration on slave cpus. Use the value calibrated
	 * on master cpu. Referenced from tegra3
	 */
	preset_lpj = loops_per_jiffy;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */

	spin_lock(&boot_lock);

	/*
	 * The secondary processor is waiting to be released from
	 * the holding pen - release it, then wait for it to flag
	 * that it has been released by resetting pen_release.
	 *
	 * Note that "pen_release" is the hardware CPU ID, whereas
	 * "cpu" is Linux's internal ID.
	 */
	write_pen_release(cpu);

	/*
	 * FIXME:
	 * reset the cpu, let it branch to the kernel entry.
	 * since pxa988 has only 1 cluseter, parameter 2 is
	 * set to 0 for tempoary usage.
	 */
	mmp_cpu_power_up(cpu, 0);

	timeout = jiffies + (1 * HZ);
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;

		udelay(10);
	}

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
static void __init mmp_smp_init_cpus(void)
{
	unsigned int i, ncores = get_core_count();

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

static void __init mmp_smp_prepare_cpus(unsigned int max_cpus)
{
	int i;

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

#ifdef CONFIG_HAVE_ARM_SCU
	scu_enable(scu_get_base_addr());
#endif

	mmp_entry_vector_init();
}

struct smp_operations mmp_smp_ops __initdata = {
	.smp_init_cpus		= mmp_smp_init_cpus,
	.smp_prepare_cpus	= mmp_smp_prepare_cpus,
	.smp_secondary_init	= mmp_secondary_init,
	.smp_boot_secondary	= mmp_boot_secondary,
};
