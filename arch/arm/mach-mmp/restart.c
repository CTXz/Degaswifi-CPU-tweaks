/*
 * linux/arch/arm/mach-mmp/restart.c
 *
 * Author:	Yilu Mao <ylmao@marvell.com>
 * Copyright:	(C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/mm.h>
#include <linux/io.h>
#include <linux/features.h>
#include <mach/cputype.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>


#define RTC_BASE	(0xD4010000)
#define RTC_SIZE	(0x10000)

#define REG_RTC_BR0	(0x14)

#define MPMU_BASE	(0xD4050000)
#define MPMU_SIZE	(0x20000)

#define MPMU_APRR	(0x1020)
#define MPMU_CPRR	(0x0020)
#define MPMU_WDTPCR	(0x0200)

#define MPMU_APRR_WDTR	(1<<4)
#define MPMU_APRR_CPR	(1<<0)
#define MPMU_CPRR_DSPR	(1<<2)
#define MPMU_CPRR_BBR	(1<<3)

extern void pxa_wdt_reset(void);
/* Using watchdog restart */
static void do_wdt_restart(const char *cmd)
{
	u32 reg, backup;
	void __iomem *mpmu_vaddr, *rtc_vaddr;
	int i;

	mpmu_vaddr = ioremap(MPMU_BASE, MPMU_SIZE);
	BUG_ON(!mpmu_vaddr);

	rtc_vaddr = ioremap(RTC_BASE, RTC_SIZE);
	BUG_ON(!rtc_vaddr);

	/* Hold cp to avoid restart watchdog */
	if (cpu_is_pxa910()) {
		/* hold CP first */
		reg = readl(mpmu_vaddr + MPMU_APRR) | MPMU_APRR_CPR;
		writel(reg, mpmu_vaddr + MPMU_APRR);
		udelay(10);

		/* CP restart MSA */
		reg = readl(mpmu_vaddr + MPMU_CPRR) | MPMU_CPRR_DSPR | MPMU_CPRR_BBR;
		writel(reg, mpmu_vaddr + MPMU_CPRR);
		udelay(10);
	} else
		pr_info("do not hold CP in %s!!!\n", __func__);

	/* If reboot by recovery, store info for uboot */
	if (cpu_is_pxa910() || cpu_is_pxa1088() || cpu_is_pxa1L88()) {
		if (cmd && !strcmp(cmd, "recovery")) {
			for (i = 0, backup = 0; i < 4; i++) {
				backup <<= 8;
				backup |= *(cmd + i);
			}
			do {
				writel(backup, rtc_vaddr + REG_RTC_BR0);
			} while (readl(rtc_vaddr + REG_RTC_BR0) != backup);
		}
	}

	/* disable IRQ to avoid interrupt between read and set WDT */
	local_fiq_disable();
	local_irq_disable();

	/* Using Watchdog to reset.
	 * Note that every platform should provide such API,
	 * or this part can't pass compiling
	 */
	pxa_wdt_reset();

	iounmap(mpmu_vaddr);
	iounmap(rtc_vaddr);
}

void mmp_arch_restart(char mode, const char *cmd)
{
	if (!has_feat_arch_restart()) {
		pr_err("%s: unsupported cpu.\n", __func__);
		return;
	}
	
	printk("%s (%c)\n", __func__, mode);
	flush_cache_all();
	outer_flush_all();

	switch (mode) {
	case 's':
		/* Jump into ROM at address 0 */
		cpu_reset(0);
		break;
	case 'w':
	default:
		do_wdt_restart(cmd);
		break;
	}
}
