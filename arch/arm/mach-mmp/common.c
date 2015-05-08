/*
 *  linux/arch/arm/mach-mmp/common.c
 *
 *  Code common to PXA168 processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_address.h>

#include <asm/page.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>

#include "common.h"

#ifdef CONFIG_CPU_PXA1986
#define MMP_CHIPID	(APB_VIRT_BASE + 0x30c00c)
#else
#define MMP_CHIPID	(AXI_VIRT_BASE + 0x82c00)
#endif

unsigned int mmp_chip_id;
EXPORT_SYMBOL(mmp_chip_id);

static struct map_desc standard_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(APB_PHYS_BASE),
		.virtual	= (unsigned long)APB_VIRT_BASE,
		.length		= APB_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(AXI_PHYS_BASE),
		.virtual	= (unsigned long)AXI_VIRT_BASE,
		.length		= AXI_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(MMP_CORE_PERIPH_PHYS_BASE),
		.virtual	= (unsigned long)MMP_CORE_PERIPH_VIRT_BASE,
		.length		= MMP_CORE_PERIPH_PHYS_SIZE,
		.type		= MT_DEVICE,
#ifdef DMCU_PHYS_BASE
	}, {
		.pfn		= __phys_to_pfn(DMCU_PHYS_BASE),
		.virtual	= (unsigned long)DMCU_VIRT_BASE,
		.length		= DMCU_PHYS_SIZE,
		.type		= MT_DEVICE,
#endif
	}
};

void __init mmp_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));

	/* this is early, initialize mmp_chip_id here */
	mmp_chip_id = __raw_readl(MMP_CHIPID);
}

void mmp_restart(char mode, const char *cmd)
{
	soft_restart(0);
}
