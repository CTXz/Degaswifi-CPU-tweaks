/*
 * linux/arch/arm/mach-mmp/mmp-debug.c
 *
 * Author:	Neil Zhang <zhangwm@marvell.com>
 * Copyright:	(C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/kexec.h>
#include <linux/features.h>

#include <asm/io.h>
#include <asm/signal.h>
#include <asm/siginfo.h>
#include <asm/cacheflush.h>

#include <mach/addr-map.h>

#define FAB_TIMEOUT_CTRL	0x60
#define FAB_TIMEOUT_STATUS0	0x68
#define FAB_TIMEOUT_STATUS1	0x70

#define STATE_HOLD_CTRL		0x88
#define FABTIMEOUT_HELD_WSTATUS	0x78
#define FABTIMEOUT_HELD_RSTATUS	0x80
#define DVC_HELD_STATUS		0x90
#define FCDONE_HELD_STATUS	0x98
#define PMULPM_HELD_STATUS	0xA0
#define CORELPM_HELD_STATUS	0xA8

struct held_status {
	u32 fabws;
	u32 fabrs;
	u32 dvcs;
	u32 fcdones;
	u32 pmulpms;
	u32 corelpms;
};

void __attribute__((weak)) set_emmd_indicator(void) { }

static void __iomem *squ_base;

static u32 fab_timeout_write_addr, fab_timeout_read_addr;
static u32 finish_save_cpu_ctx;

static struct held_status recorded_helds;

static int mmp_axi_timeout(unsigned long addr, unsigned int fsr,
				struct pt_regs *regs)
{
	struct pt_regs fixed_regs;

	fab_timeout_write_addr = readl_relaxed(squ_base + FAB_TIMEOUT_STATUS0);
	fab_timeout_read_addr = readl_relaxed(squ_base + FAB_TIMEOUT_STATUS1);

	set_emmd_indicator();

	//keep_silent = 1;
	crash_setup_regs(&fixed_regs, regs);
	crash_save_vmcoreinfo();
	machine_crash_shutdown(&fixed_regs);
	finish_save_cpu_ctx = 1;

	flush_cache_all();
	outer_flush_all();

	/* Waiting wdt to reset the Soc */
	while (1);

	return 0;
}

/* dump Fabric/LPM/DFC/DVC held status and enable held feature */
static int __init mmp_dump_heldstatus(void __iomem *squ_base)
{
	recorded_helds.fabws =
		readl_relaxed(squ_base + FABTIMEOUT_HELD_WSTATUS);
	recorded_helds.fabrs =
		readl_relaxed(squ_base + FABTIMEOUT_HELD_RSTATUS);
	recorded_helds.dvcs = readl_relaxed(squ_base + DVC_HELD_STATUS);
	recorded_helds.fcdones = readl_relaxed(squ_base + FCDONE_HELD_STATUS);
	recorded_helds.pmulpms = readl_relaxed(squ_base + PMULPM_HELD_STATUS);
	recorded_helds.corelpms = readl_relaxed(squ_base + CORELPM_HELD_STATUS);

	/* after register dump, then enable the held feature for debug */
	writel_relaxed(0x1, squ_base + STATE_HOLD_CTRL);

	pr_info("*************************************\n");
	pr_info("Fabric/LPM/DFC/DVC held status dump:\n");

	pr_info("Fabric W[%x],R[%x]\n", recorded_helds.fabws,
		recorded_helds.fabrs);
	pr_info("DVC[%x]\n", recorded_helds.dvcs);
	pr_info("FCDONE[%x]\n", recorded_helds.fcdones);
	pr_info("PMULPM[%x]\n", recorded_helds.pmulpms);
	pr_info("CORELPM[%x]\n", recorded_helds.corelpms);

	pr_info("*************************************\n");
	return 0;
}

static int __init mmp_debug_init(void)
{
	u32 tmp;

	if (!has_feat_debug())
		return 0;

	squ_base = ioremap(APB_PHYS_BASE + 0x2A0000, SZ_4K);
	if(!squ_base) {
		pr_err("mmp_debug_init : ioremap failed\n");
		return -ENOMEM;
	}

	/* configure to data abort mode */
	tmp = readl_relaxed(squ_base + FAB_TIMEOUT_CTRL);
	tmp |= (1 << 29) | (1 << 30);
	writel_relaxed(tmp, squ_base + FAB_TIMEOUT_CTRL);

	/* Register debug fault handler. */
	hook_fault_code(0x8, mmp_axi_timeout, SIGBUS,
			0, "AXI Fabric#1 S2 timeout exception");
	hook_fault_code(0xc, mmp_axi_timeout, SIGBUS,
			0, "AXI Fabric#1 S2 timeout exception");
	hook_fault_code(0xe, mmp_axi_timeout, SIGBUS,
			0, "AXI Fabric#1 S2 timeout exception");
	hook_fault_code(0x16, mmp_axi_timeout, SIGBUS,
			BUS_OBJERR, "AXI Fabric#1 S2 timeout exception");

	mmp_dump_heldstatus(squ_base);
	return 0;
}

arch_initcall(mmp_debug_init);
