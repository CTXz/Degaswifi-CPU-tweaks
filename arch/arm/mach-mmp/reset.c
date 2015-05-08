/*
 * linux/arch/arm/mach-mmp/reset.c
 *
 * Author:	Neil Zhang <zhangwm@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/smp.h>

#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/mach/map.h>
#include <asm/mcpm.h>

#include <mach/addr-map.h>
#include <mach/cputype.h>

#include "reset.h"
#ifdef CONFIG_TZ_HYPERVISOR
#include <linux/pxa_tzlc.h>
#endif


#define PMU_CC2_AP		APMU_REG(0x0100)
#define CIU_WARM_RESET_VECTOR	CIU_REG(0x00d8)
#define SW_SCRATCH		CIU_REG(0x24)

/*
 * This function is called from boot_secondary to bootup the secondary cpu.
 */
void mmp_cpu_power_up(unsigned int cpu, unsigned int cluster)
{
	u32 tmp;

	BUG_ON(cpu == 0);

	tmp = readl(PMU_CC2_AP);
	if (cpu_is_pxa1088() || cpu_is_pxa1L88()) {
		if (tmp & PXA1088_CPU_CORE_RST(cpu)) {
			/* Release secondary core from reset */
			tmp &= ~(PXA1088_CPU_POR_RST(cpu)
				| PXA1088_CPU_CORE_RST(cpu) | PXA1088_CPU_DBG_RST(cpu));
			writel(tmp, PMU_CC2_AP);
		}
	} else {
		if (tmp & CPU_CORE_RST(cpu)) {
			/* Release secondary core from reset */
			tmp &= ~(CPU_CORE_RST(cpu)
				| CPU_DBG_RST(cpu) | CPU_WDOG_RST(cpu));
			writel(tmp, PMU_CC2_AP);
		}
	}
}

void __init mmp_entry_vector_init(void)
{
#ifdef CONFIG_TZ_HYPERVISOR
	int ret = -1;
	tzlc_cmd_desc cmd_desc;
	tzlc_handle tzlc_hdl;
#endif

#ifdef CONFIG_CPU_PXA988
/*
 * with TrustZone enabled, CIU_WARM_RESET_VECTOR is used by TrustZone software,
 * and kernel use CIU_SW_SCRATCH_REG to save the cpu reset entry. Accessing
 * CIU_SW_SCRATCH_REG is performend in secure world.
*/
#ifdef CONFIG_TZ_HYPERVISOR
	tzlc_hdl = pxa_tzlc_create_handle();
	if (tzlc_hdl != (tzlc_handle)1) {
		pr_err("tzlc: failed to create tzlc handler\n");
		return;
	}
	memset(&cmd_desc, 0, sizeof(tzlc_cmd_desc));
	cmd_desc.op = TZLC_CMD_SET_WARM_RESET_ENTRY;
	cmd_desc.args[0] = __pa(mcpm_entry_point);
	ret = pxa_tzlc_cmd_op(tzlc_hdl, &cmd_desc);
	if (ret != 0)
		pr_err("tzlc: failed to execute tzlc cmd\n");
	pxa_tzlc_destroy_handle(tzlc_hdl);
#else
	writel(__pa(mcpm_entry_point), CIU_WARM_RESET_VECTOR);
#endif
#endif
}
