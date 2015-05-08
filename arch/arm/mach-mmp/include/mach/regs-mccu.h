/*
 * linux/arch/arm/mach-mmp/include/mach/regs-mccu.h
 *
 *   Main Clock Control Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_MCCU_H
#define __ASM_MACH_REGS_MCCU_H

#if defined(CONFIG_CPU_PXA1986)
/*
 * MCCU register offsets for PXA1986
 */
#define MCCU_APBCR		(0x000)
#define MCCU_AXICR		(0x004)
#define MCCU_DDRCR		(0x008)
#define MCCU_TIMSTCR		(0x00C)
#define MCCU_PDBGCR		(0x010)
#define MCCU_AP_CKREQ		(0x014)
#define MCCU_CP_CKREQ		(0x018)
#define MCCU_AUD_CKREQ		(0x01C)
#define MCCU_CKDBG		(0x020)
#define MCCU_TSCR		(0x024)
#define MCCU_I2CCR		(0x028)
#define MCCU_GPNMCR		(0x02C)
#define MCCU_GPNMEN		(0x030)
#define MCCU_TROCR		(0x034)
#define MCCU_CKENCR		(0x038)
#define MCCU_CKCTR		(0x03C)
#define MCCU_AP_CKSTAT		(0x050)
#define MCCU_CP_CKSTAT		(0x054)
#define MCCU_AUD_CKSTAT		(0x058)
#endif

#endif /* __ASM_MACH_REGS_MCCU_H */
