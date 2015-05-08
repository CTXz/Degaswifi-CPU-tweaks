/*
 * linux/arch/arm/mach-mmp/mmp_cpuidle.c
 *
 * Author:	Fangsuo Wu <fswu@marvell.com>
 * Copyright:	(C) 2013 marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/init.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/kernel.h>
#include <linux/pm_qos.h>
#include <linux/clk/mmpdcstat.h>
#include <asm/io.h>
#include <asm/mcpm.h>
#include <mach/help_v7.h>
#include <mach/cputype.h>
#include <mach/mmp_cpuidle.h>

#include <trace/events/pxa.h>

#include "reset.h"

#define LPM_NUM			16
#define INVALID_LPM		-1
#define DEFAULT_LPM_FLAG	0xFFFFFFFF

struct platform_idle *mmp_idle;
static int mmp_wake_saved;
static arch_spinlock_t mmp_lpm_lock = __ARCH_SPIN_LOCK_UNLOCKED;
static int mmp_enter_lpm[MAX_NR_CLUSTERS][MAX_CPUS_PER_CLUSTER];
static int mmp_pm_use_count[MAX_NR_CLUSTERS][MAX_CPUS_PER_CLUSTER];
/*
 * find_couple_state - Find the maximum state platform can enter
 *
 * @index: pointer to variable which stores the maximum state
 * @cluster: cluster number
 *
 * Must be called with function holds mmp_lpm_lock
 */
static void find_coupled_state(int *index, int cluster)
{
	int i;
	int platform_lpm = DEFAULT_LPM_FLAG;

	for (i = 0; i < MAX_CPUS_PER_CLUSTER; i++)
		platform_lpm &= mmp_enter_lpm[cluster][i];

	*index = min(find_first_zero_bit((void *)&platform_lpm,
			LPM_NUM), \
			pm_qos_request(PM_QOS_CPUIDLE_BLOCK)) - 1;

}

static bool cluster_is_idle(int cluster)
{
	int i;

	for(i = 0; i < MAX_CPUS_PER_CLUSTER; i++)
		if(mmp_pm_use_count[cluster][i] != 0)
			return false;

	return true;
}

/*
 * mmp_pm_down - Programs CPU to enter the specified state
 *
 * @addr: address points to the state selected by cpu governor
 *
 * Called from the CPUidle framework to program the device to the
 * specified target state selected by the governor.
 */
static void mmp_pm_down(unsigned long addr)
{
	int *idx = (int *)addr;
	int mpidr, cpu, cluster;
	bool skip_wfi = false, last_man = false;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cluster >= MAX_NR_CLUSTERS || cpu >= MAX_CPUS_PER_CLUSTER);

	__mcpm_cpu_going_down(cpu, cluster);

	arch_spin_lock(&mmp_lpm_lock);

	mmp_pm_use_count[cluster][cpu]--;

	if (mmp_pm_use_count[cluster][cpu] == 0) {
		mmp_enter_lpm[cluster][cpu] = (1 << (*idx + 1)) - 1;
		*idx = mmp_idle->cpudown_state;
		if (cluster_is_idle(cluster)) {
			cpu_cluster_pm_enter();
			find_coupled_state(idx, cluster);
			if (*idx >= mmp_idle->wakeup_state && \
				mmp_idle->ops->save_wakeup) {
				mmp_wake_saved = 1;
				mmp_idle->ops->save_wakeup();
			}
			BUG_ON(__mcpm_cluster_state(cluster) != CLUSTER_UP);
			last_man = true;
		}
		if(mmp_idle->ops->set_pmu)
			mmp_idle->ops->set_pmu(cpu, *idx);
	} else if (mmp_pm_use_count[cluster][cpu] == 1) {
		/*
		 * A power_up request went ahead of us.
		 * Even if we do not want to shut this CPU down,
		 * the caller expects a certain state as if the WFI
		 * was aborted.  So let's continue with cache cleaning.
		 */
		skip_wfi = true;
		*idx = INVALID_LPM;
	} else
		BUG();

	if (last_man && (*idx >= mmp_idle->cpudown_state))
		cpu_dcstat_event(cpu_dcstat_clk, cpu, CPU_M2_OR_DEEPER_ENTER,
			*idx);

	trace_pxa_cpu_idle(LPM_ENTRY(*idx), cpu, cluster);
	cpu_dcstat_event(cpu_dcstat_clk, cpu, CPU_IDLE_ENTER, *idx);
	if (last_man && __mcpm_outbound_enter_critical(cpu, cluster)) {
		arch_spin_unlock(&mmp_lpm_lock);
		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);
		__mcpm_cpu_down(cpu, cluster);
		/*
		 * __mcpm_cpu_down and __mcpm_outbound_leave_critical needs
		 * to use spinlock.
		 * But for ca9, spinlock can't use if l2 cache is enabled
		 * while l1 is disabled.
		 * Also for ca7, mcpm_cpu_down() function call after
		 * ca7_power_down() will involve cache coherence issue, which
		 * will cause mcpm flag cannot be updated correctly.
		 */
		if (cpu_is_ca9())
			ca9_power_down();
		else if (cpu_is_ca7()) {
			if (*idx >= mmp_idle->l2_flush_state)
				ca7_power_down_udr();
			else
				ca7_power_down();
		}
	} else {
		arch_spin_unlock(&mmp_lpm_lock);
		__mcpm_cpu_down(cpu, cluster);
		if (cpu_is_ca9())
			ca9_power_down();
		else if (cpu_is_ca7())
			ca7_power_down();
	}
	if (!skip_wfi)
		cpu_do_idle();
}

static int mmp_pm_power_up(unsigned int cpu, unsigned int cluster)
{
	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	if (cluster >= MAX_NR_CLUSTERS || cpu >= MAX_CPUS_PER_CLUSTER)
		return -EINVAL;
	/*
	 * Since this is called with IRQs enabled, and no arch_spin_lock_irq
	 * variant exists, we need to disable IRQs manually here.
	 */
	local_irq_disable();
	arch_spin_lock(&mmp_lpm_lock);
	/*
	 * TODO: Check if we need to do cluster related ops here?
	 * (Seems no need since this function should be called by
	 * other core, which should not enter lpm at this point).
	 */
	mmp_pm_use_count[cluster][cpu]++;

	if (mmp_pm_use_count[cluster][cpu] == 1) {
		mmp_cpu_power_up(cpu, cluster);
	} else if (mmp_pm_use_count[cluster][cpu] != 2) {
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}

	arch_spin_unlock(&mmp_lpm_lock);
	local_irq_enable();

	return 0;
}

static void mmp_pm_power_down(void)
{
	int idx = mmp_idle->hotplug_state;
	mmp_pm_down((unsigned long)&idx);
}

static void mmp_pm_suspend(unsigned long addr)
{
	mmp_pm_down(addr);
}

static void mmp_pm_powered_up(void)
{
	int mpidr, cpu, cluster;
	unsigned long flags;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	pr_debug("%s: cpu %u cluster %u\n", __func__, cpu, cluster);
	BUG_ON(cluster >= MAX_NR_CLUSTERS || cpu >= MAX_CPUS_PER_CLUSTER);

	cpu_dcstat_event(cpu_dcstat_clk, cpu, CPU_IDLE_EXIT, MAX_LPM_INDEX);
	trace_pxa_cpu_idle(LPM_EXIT(0), cpu, cluster);
	local_irq_save(flags);
	arch_spin_lock(&mmp_lpm_lock);

	if (cluster_is_idle(cluster)) {
		if (mmp_wake_saved && mmp_idle->ops->restore_wakeup) {
			mmp_wake_saved = 0;
			mmp_idle->ops->restore_wakeup();
		}
		/* If hardware really shutdown MP subsystem */
		if (!(readl_relaxed(gic_get_dist_base() + GIC_DIST_CTRL) & 0x1))
			cpu_cluster_pm_exit();
	}

	if (!mmp_pm_use_count[cluster][cpu])
		mmp_pm_use_count[cluster][cpu] = 1;

	mmp_enter_lpm[cluster][cpu]= 0;

	if(mmp_idle->ops->clr_pmu)
		mmp_idle->ops->clr_pmu(cpu);

	arch_spin_unlock(&mmp_lpm_lock);
	local_irq_restore(flags);
}

/**
 * mmp_platform_power_register - register platform power ops
 *
 * @idle: platform_idle structure points to platform power ops
 *
 * An error is returned if the registration has been done previously.
 */
int __init mmp_platform_power_register(struct platform_idle *idle)
{
	if (mmp_idle)
		return -EBUSY;
	mmp_idle = idle;

#ifdef CONFIG_CPU_IDLE_MMP
	mcpm_platform_state_register(mmp_idle->states, mmp_idle->state_count);
#endif

	return 0;
}

static const struct mcpm_platform_ops mmp_pm_power_ops = {
	.power_up	= mmp_pm_power_up,
	.power_down	= mmp_pm_power_down,
	.suspend	= mmp_pm_suspend,
	.powered_up	= mmp_pm_powered_up,
};

static void __init mmp_pm_usage_count_init(void)
{
	unsigned int mpidr, cpu, cluster;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	BUG_ON(cpu >= MAX_CPUS_PER_CLUSTER || cluster >= MAX_NR_CLUSTERS);
	memset(mmp_pm_use_count, 0, sizeof(mmp_pm_use_count));
	mmp_pm_use_count[cluster][cpu] = 1;
}


static int __init mmp_pm_init(void)
{
	int ret;

	/*
	 * TODO:Should check if hardware is initialized here.
	 * See vexpress_spc_check_loaded()
	 */
	memset(mmp_enter_lpm, DEFAULT_LPM_FLAG, sizeof(mmp_enter_lpm));
	mmp_pm_usage_count_init();

	ret = mcpm_platform_register(&mmp_pm_power_ops);
	if (!ret)
		pr_warning("Power ops has already been initialized\n");

	if (mmp_idle->ops->power_up_setup) {
		ret = mcpm_sync_init(mmp_idle->ops->power_up_setup);
		if (!ret)
			pr_info("mmp power management initialized\n");
	} else
		pr_warning("mmp power_up_setup function is NULL!\n");

	return ret;
}
early_initcall(mmp_pm_init);
