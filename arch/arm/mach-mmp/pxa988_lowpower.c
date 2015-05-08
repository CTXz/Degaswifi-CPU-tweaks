/*
 * linux/arch/arm/mach-mmp/pxa988_lowpower.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
 *        	Fangsuo Wu <fswu@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/edge_wakeup_mmp.h>
#include <linux/regs-addr.h>
#include <asm/cpuidle.h>
#include <asm/mach/map.h>
#include <asm/mcpm.h>
#include <mach/cputype.h>
#include <mach/mmp_cpuidle.h>
#include <mach/pxa988_lowpower.h>
#include <mach/addr-map.h>

static void __iomem *icu_virt_addr;
/* All these regs are per-cpu owned */
static void __iomem *APMU_CORE_IDLE_CFG[4];
static void __iomem *APMU_MP_IDLE_CFG[4];
static void __iomem *ICU_GBL_INT_MSK[4];
static u32 s_apcr, s_awucrm;

static struct cpuidle_state pxa988_modes[] = {
	[0] = {
		.exit_latency		= 18,
		.target_residency	= 36,
		/*
		 * Use CPUIDLE_FLAG_TIMER_STOP flag to let the cpuidle
		 * framework handle the CLOCK_EVENT_NOTIFY_BROADCAST_
		 * ENTER/EXIT when entering idle states.
		 */
		.flags			= CPUIDLE_FLAG_TIME_VALID,
		.name			= "C1",
		.desc			= "C1: Core internal clock gated",
		.enter			= arm_cpuidle_simple_enter,
	},
	[1] = {
		.exit_latency		= 350,
		.target_residency	= 700,
		.flags			= CPUIDLE_FLAG_TIME_VALID | \
					  CPUIDLE_FLAG_TIMER_STOP,
		.name			= "C2",
		.desc			= "C2: Core power down",
	},
	[2] = {
		.exit_latency		= 500,
		.target_residency	= 1000,
		.flags			= CPUIDLE_FLAG_TIME_VALID | \
					  CPUIDLE_FLAG_TIMER_STOP,
		.name			= "D1p",
		.desc			= "D1p: AP idle state",
	},
	[3] = {
		.exit_latency		= 600,
		.target_residency	= 1200,
		.flags			= CPUIDLE_FLAG_TIME_VALID | \
					  CPUIDLE_FLAG_TIMER_STOP,
		.name			= "D1",
		.desc			= "D1: Chip idle state",
	},

};

/*
 * edge wakeup
 *
 * To enable edge wakeup:
 * 1. Set mfp reg edge detection bits;
 * 2. Enable mfp ICU interrupt, but disable gic interrupt;
 * 3. Enable corrosponding wakeup port;
 */
#define ICU_IRQ_ENABLE	((1 << 6) | (1 << 4) | (1 << 0))
#define EDGE_WAKEUP_ICU		50	//icu interrupt num for edge wakeup

static void edge_wakeup_port_enable(void)
{
	u32 pad_apcr, pad_awucrm;

	pad_awucrm = readl_relaxed(get_mpmu_base_va() + AWUCRM);
	pad_apcr = readl_relaxed(get_mpmu_base_va() + APCR);

	writel_relaxed(pad_awucrm | PMUM_WAKEUP2, get_mpmu_base_va() + AWUCRM);
	writel_relaxed(pad_apcr & ~PMUM_SLPWP2, get_mpmu_base_va() + APCR);
}

static void edge_wakeup_port_disable(void)
{
	u32 pad_apcr, pad_awucrm;

	pad_awucrm = readl_relaxed(get_mpmu_base_va() + AWUCRM);
	pad_apcr = readl_relaxed(get_mpmu_base_va() + APCR);

	writel_relaxed(pad_awucrm & ~PMUM_WAKEUP2, get_mpmu_base_va() + AWUCRM);
	writel_relaxed(pad_apcr | PMUM_SLPWP2, get_mpmu_base_va() + APCR);
}

static void edge_wakeup_icu_enable(void)
{
	writel_relaxed(ICU_IRQ_ENABLE, icu_virt_addr + (EDGE_WAKEUP_ICU << 2));
}

static void edge_wakeup_icu_disable(void)
{
	writel_relaxed(0, icu_virt_addr + (EDGE_WAKEUP_ICU << 2));
}

static int need_restore_pad_wakeup;
static void pxa988_edge_wakeup_enable(void)
{
	edge_wakeup_mfp_enable();
	edge_wakeup_icu_enable();
	edge_wakeup_port_enable();
	need_restore_pad_wakeup = 1;
}

static void pxa988_edge_wakeup_disable(void)
{
	if (!need_restore_pad_wakeup)
		return;
	edge_wakeup_port_disable();
	edge_wakeup_icu_disable();
	edge_wakeup_mfp_disable();
	need_restore_pad_wakeup = 0;
}

/*
 * low power config
 */
static void pxa988_lowpower_config(u32 cpu, u32 power_state, \
		u32 lowpower_enable)
{
	u32 core_idle_cfg, mp_idle_cfg[CONFIG_NR_CPUS] = {0}, apcr;
	int cpu_idx;

	core_idle_cfg = readl_relaxed(APMU_CORE_IDLE_CFG[cpu]);
	apcr = readl_relaxed(get_mpmu_base_va() + APCR);

	if (power_state < POWER_MODE_UDR || !lowpower_enable)
		mp_idle_cfg[cpu] = readl_relaxed(APMU_MP_IDLE_CFG[cpu]);
	else {
		for_each_possible_cpu(cpu_idx)
			mp_idle_cfg[cpu_idx] = readl_relaxed(APMU_MP_IDLE_CFG[cpu_idx]);
	}

	if (lowpower_enable) {
		/* Suppose that there should be no one touch the MPMU_APCR
		 * but only Last man entering the low power modes which are
		 * deeper than M2. Thus, add BUG check here.
		 */
		if (apcr & (PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD |
					PMUM_VCTCXOSD | PMUM_STBYEN | PMUM_SLPEN)) {
			pr_err("APCR is set to 0x%X by the Non-lastman\n", apcr);
			BUG();
		}

		switch (power_state) {
		case POWER_MODE_UDR:
			for_each_possible_cpu(cpu_idx)
				mp_idle_cfg[cpu_idx] |= PMUA_MP_L2_SRAM_POWER_DOWN;
			apcr |= PMUM_VCTCXOSD;
			/* fall through */
		case POWER_MODE_UDR_VCTCXO:
			apcr |= PMUM_STBYEN;
			/* fall through */
		case POWER_MODE_SYS_SLEEP:
			apcr |= PMUM_APBSD;
			apcr |= PMUM_SLPEN;
			apcr |= PMUM_DDRCORSD;
			/* fall through */
		case POWER_MODE_APPS_IDLE:
			apcr |= PMUM_AXISD;
			/*
			 * Normally edge wakeup should only take effect in D1 and
			 * deeper modes. But due to SD host hardware requirement,
			 * it has to be put in D1P mode.
			 */
			pxa988_edge_wakeup_enable();
			/* fall through */
		case POWER_MODE_CORE_POWERDOWN:
			core_idle_cfg |= PMUA_CORE_POWER_DOWN;
			mp_idle_cfg[cpu] |= PMUA_MP_POWER_DOWN;
			if (cpu_is_pxa988()) {
				core_idle_cfg |= PMUA_CORE_L1_SRAM_POWER_DOWN;
				mp_idle_cfg[cpu] |= PMUA_MP_SCU_SRAM_POWER_DOWN;
			}
			mp_idle_cfg[cpu] |= PMUA_MP_IDLE;
			core_idle_cfg |= PMUA_CORE_IDLE;
			/* fall through */
		case POWER_MODE_CORE_INTIDLE:
			break;
		default:
			WARN(1, "Invalid power state!\n");
		}
	} else {
		core_idle_cfg &= ~(PMUA_CORE_IDLE | PMUA_CORE_POWER_DOWN);
		mp_idle_cfg[cpu] &= ~(PMUA_MP_IDLE | PMUA_MP_POWER_DOWN |
				PMUA_MP_L2_SRAM_POWER_DOWN |
				PMUA_MP_MASK_CLK_OFF);
		if (cpu_is_pxa988()) {
			core_idle_cfg &= ~(PMUA_CORE_L1_SRAM_POWER_DOWN);
			mp_idle_cfg[cpu] &= ~(PMUA_MP_SCU_SRAM_POWER_DOWN);
		}
		apcr &= ~(PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD |
			PMUM_VCTCXOSD | PMUM_STBYEN | PMUM_SLPEN);
		pxa988_edge_wakeup_disable();
	}

	writel_relaxed(core_idle_cfg, APMU_CORE_IDLE_CFG[cpu]);
	writel_relaxed(apcr, get_mpmu_base_va() + APCR);
	if (power_state < POWER_MODE_UDR || !lowpower_enable)
		writel_relaxed(mp_idle_cfg[cpu], APMU_MP_IDLE_CFG[cpu]);
	else {
		for_each_possible_cpu(cpu_idx)
			writel_relaxed(mp_idle_cfg[cpu_idx], APMU_MP_IDLE_CFG[cpu_idx]);
	}

	return;
}

#define DISABLE_ALL_WAKEUP_PORTS		\
	(PMUM_SLPWP0 | PMUM_SLPWP1 | PMUM_SLPWP2 | PMUM_SLPWP3 |	\
	 PMUM_SLPWP4 | PMUM_SLPWP5 | PMUM_SLPWP6 | PMUM_SLPWP7)
/* Here we don't enable CP wakeup sources since CP will enable them */
#define ENABLE_AP_WAKEUP_SOURCES	\
	(PMUM_AP_ASYNC_INT | PMUM_AP_FULL_IDLE | PMUM_SQU_SDH1 | PMUM_SDH_23 | \
	 PMUM_KEYPRESS | PMUM_WDT | PMUM_RTC_ALARM | PMUM_AP1_TIMER_1 |	\
	 PMUM_AP1_TIMER_2 | PMUM_AP2_TIMER_1 | PMUM_AP2_TIMER_2 | \
	 PMUM_WAKEUP7 | PMUM_WAKEUP6 | PMUM_WAKEUP5 | PMUM_WAKEUP4 | \
	 PMUM_WAKEUP3 | PMUM_WAKEUP2)
/*
 * Enable AP wakeup sources and ports. To enalbe wakeup
 * ports, it needs both AP side to configure MPMU_APCR
 * and CP side to configure MPMU_CPCR to really enable
 * it. To enable wakeup sources, either AP side to set
 * MPMU_AWUCRM or CP side to set MPMU_CWRCRM can really
 * enable it.
 */
static void pxa988_save_wakeup(void)
{
	s_awucrm = readl_relaxed(get_mpmu_base_va() + AWUCRM);
	s_apcr = readl_relaxed(get_mpmu_base_va() + APCR);
	writel_relaxed(s_awucrm | ENABLE_AP_WAKEUP_SOURCES, \
			get_mpmu_base_va() + AWUCRM);
	writel_relaxed(s_apcr & ~DISABLE_ALL_WAKEUP_PORTS, \
			get_mpmu_base_va() + APCR);
}

static void pxa988_restore_wakeup(void)
{
	writel_relaxed(s_awucrm, get_mpmu_base_va() + AWUCRM);
	writel_relaxed(s_apcr, get_mpmu_base_va() + APCR);
}

static void pxa988_gic_global_mask(u32 cpu, u32 mask)
{
	u32 core_idle_cfg;

	core_idle_cfg = readl_relaxed(APMU_CORE_IDLE_CFG[cpu]);

	if (mask) {
		core_idle_cfg |= PMUA_GIC_IRQ_GLOBAL_MASK;
		core_idle_cfg |= PMUA_GIC_FIQ_GLOBAL_MASK;
	} else {
		core_idle_cfg &= ~(PMUA_GIC_IRQ_GLOBAL_MASK |
					PMUA_GIC_FIQ_GLOBAL_MASK);
	}
	writel_relaxed(core_idle_cfg, APMU_CORE_IDLE_CFG[cpu]);
}

static void pxa988_icu_global_mask(u32 cpu, u32 mask)
{
	u32 icu_msk;

	icu_msk = readl_relaxed(ICU_GBL_INT_MSK[cpu]);

	if (mask) {
		icu_msk |= ICU_MASK_FIQ;
		icu_msk |= ICU_MASK_IRQ;
	} else {
		icu_msk &= ~(ICU_MASK_FIQ | ICU_MASK_IRQ);
	}
	writel_relaxed(icu_msk, ICU_GBL_INT_MSK[cpu]);
}

static void pxa988_set_pmu(u32 cpu, u32 power_mode)
{
	pxa988_lowpower_config(cpu, power_mode, 1);

	/* Mask GIC global interrupt */
	pxa988_gic_global_mask(cpu, 1);
	/* Mask ICU global interrupt */
	pxa988_icu_global_mask(cpu, 1);
}

static void pxa988_clr_pmu(u32 cpu)
{
	/* Unmask GIC interrtup */
	pxa988_gic_global_mask(cpu, 0);
	/* Mask ICU global interrupt */
	pxa988_icu_global_mask(cpu, 1);

	pxa988_lowpower_config(cpu, -1, 0);
}

static struct platform_power_ops pxa988_power_ops = {
	.set_pmu	= pxa988_set_pmu,
	.clr_pmu	= pxa988_clr_pmu,
	.save_wakeup	= pxa988_save_wakeup,
	.restore_wakeup	= pxa988_restore_wakeup,
	.power_up_setup = ca9_power_up_setup,
};

static struct platform_idle pxa988_idle = {
	.cpudown_state	= POWER_MODE_CORE_POWERDOWN,
	.wakeup_state	= POWER_MODE_SYS_SLEEP,
	.hotplug_state	= POWER_MODE_UDR,
	.l2_flush_state	= POWER_MODE_UDR,
	.ops		= &pxa988_power_ops,
	.states		= pxa988_modes,
	.state_count	= ARRAY_SIZE(pxa988_modes),
};

static void __init pxa988_reg_init(void)
{
	icu_virt_addr  = icu_get_base_addr();

	APMU_CORE_IDLE_CFG[0] = get_apmu_base_va() + CORE0_IDLE;
	APMU_CORE_IDLE_CFG[1] = get_apmu_base_va() + CORE1_IDLE;
	APMU_MP_IDLE_CFG[0]   = get_apmu_base_va() + MP_CFG0;
	APMU_MP_IDLE_CFG[1]   = get_apmu_base_va() + MP_CFG1;

	if (cpu_is_pxa988()) {
		ICU_GBL_INT_MSK[0] = icu_virt_addr  + CORE0_CA9_GLB_INT_MASK;
		ICU_GBL_INT_MSK[1] = icu_virt_addr  + CORE1_CA9_GLB_INT_MASK;
		return;
	}

	APMU_CORE_IDLE_CFG[2] = get_apmu_base_va() + CORE2_IDLE;
	APMU_CORE_IDLE_CFG[3] = get_apmu_base_va() + CORE3_IDLE;
	APMU_MP_IDLE_CFG[2]   = get_apmu_base_va() + MP_CFG2;
	APMU_MP_IDLE_CFG[3]   = get_apmu_base_va() + MP_CFG3;

	ICU_GBL_INT_MSK[0] = icu_virt_addr  + CORE0_CA7_GLB_INT_MASK;
	ICU_GBL_INT_MSK[1] = icu_virt_addr  + CORE1_CA7_GLB_INT_MASK;
	ICU_GBL_INT_MSK[2] = icu_virt_addr  + CORE2_CA7_GLB_INT_MASK;
	ICU_GBL_INT_MSK[3] = icu_virt_addr  + CORE3_CA7_GLB_INT_MASK;
}

static int __init pxa988_lowpower_init(void)
{
	u32 apcr;

	pxa988_reg_init();

	if (cpu_is_ca7())
		pxa988_idle.ops->power_up_setup = ca7_power_up_setup;

	BUG_ON(!pxa988_idle.ops->power_up_setup);

	mmp_platform_power_register(&pxa988_idle);
	/* set DSPSD, DTCMSD, BBSD, MSASLPEN */
	apcr = readl_relaxed(get_mpmu_base_va() + APCR);
	apcr |= PMUM_DTCMSD | PMUM_BBSD | PMUM_MSASLPEN;
	apcr &= ~PMUM_STBYEN;

	writel_relaxed(apcr, get_mpmu_base_va() + APCR);
	/*
	 * Set SCU control register standby enable bit.
	 * 0x0 refers to SCU_CTRL reg. See arch/arm/
	 * kernel/smp_scu.c for details.
	 */
	writel_relaxed(readl_relaxed(scu_get_base_addr() + 0x0) | (1 << 5),
			scu_get_base_addr()  + 0x0);

	/* set the power stable timer as 10us */
	__raw_writel(0x28207, get_apmu_base_va() + STBL_TIMER);

#ifdef CONFIG_ARM_ERRATA_802022
	{
		u32 mp_idle_cfg;
		int i;
		for_each_possible_cpu(i) {
			mp_idle_cfg = __raw_readl(APMU_MP_IDLE_CFG[i]);
			mp_idle_cfg |= (PMUA_DIS_MP_SLP);
			__raw_writel(mp_idle_cfg, APMU_MP_IDLE_CFG[i]);
		}
	}
#endif

	return 0;
}
early_initcall(pxa988_lowpower_init);


