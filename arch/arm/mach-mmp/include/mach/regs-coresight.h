/*
 * linux/arch/arm/mach-mmp/include/mach/regs-coresight.h
 *
 * Author:     Neil Zhang <zhangwm@marvell.com>
 * Copyright:  (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ASM_MACH_CORSIGHT_H
#define __ASM_MACH_CORSIGHT_H

#include <mach/addr-map.h>

#define CORESIGHT_VIRT_BASE	(APB_VIRT_BASE + 0x100000)

#if defined(CONFIG_CPU_PXA988) || defined(CONFIG_CPU_PXA1088)
#define ETB_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x5000)
#define TPIU_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x8000)
#define CSTF_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x9000)
#elif defined(CONFIG_CPU_EDEN)
#define ETB_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x20000)
#define CSS_CTI1_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x21000)
#define TPIU_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x22000)
/* core funnel */
#define CORE_FUN_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x23000)
/* coresight subsystem funnel */
#define CSTF_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x24000)
#define CSS_ITM_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x25000)
#define TS_READ_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x27000)
#define TS_CTRL_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x28000)
#define CSS_CTI2_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x29000)
#define CSS_CTI3_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x2A000)
#define DEBUGCCU_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x2B000)
#define HUBF_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x2C000)
#define HUBF_FUN_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x2D000)
#define PML_VIRT_BASE		(CORESIGHT_VIRT_BASE + 0x2E000)
#endif

#define DBG_CORE0_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x10000)
#define CTI_CORE0_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x18000)
#define CTI_CORE1_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x19000)
#define CTI_CORE2_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1A000)
#define CTI_CORE3_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1B000)
#define PTM_CORE0_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1C000)
#define PTM_CORE1_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1D000)
#define PTM_CORE2_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1E000)
#define PTM_CORE3_VIRT_BASE	(CORESIGHT_VIRT_BASE + 0x1F000)

#define ETB_REG(x)		(ETB_VIRT_BASE + (x))
#define TPIU_REG(x)		(TPIU_VIRT_BASE + (x))
#define CSTF_REG(x)		(CSTF_VIRT_BASE + (x))

#define PTM_REG(x)		((PTM_CORE0_VIRT_BASE \
				+ 0x1000 * raw_smp_processor_id()) \
				+ (x))

#define CTI_REG(x)		((CTI_CORE0_VIRT_BASE \
				+ 0x1000 * raw_smp_processor_id()) \
				+ (x))

#define ETB_LOCK		ETB_REG(0xFB0)
#define TPIU_LOCK		TPIU_REG(0xFB0)
#define CSTF_LOCK		CSTF_REG(0xFB0)
#define PTM_LOCK		PTM_REG(0xFB0)
#define CTI_LOCK		CTI_REG(0xFB0)

#define DBG_REG(cpu, addr)	(DBG_CORE0_VIRT_BASE + cpu * 0x2000 + addr)

#define DBG_ID(cpu)		DBG_REG(cpu, 0x0)
#define DBG_DTRRX(cpu)		DBG_REG(cpu, 0x80)
#define DBG_ITR(cpu)		DBG_REG(cpu, 0x84)	/* Write only */
#define DBG_PCSR(cpu)		DBG_REG(cpu, 0x84)	/* Read only */
#define DBG_DSCR(cpu)		DBG_REG(cpu, 0x88)
#define DBG_DTRTX(cpu)		DBG_REG(cpu, 0x8C)
#define DBG_DRCR(cpu)		DBG_REG(cpu, 0x90)
#define DBG_LAR(cpu)		DBG_REG(cpu, 0xFB0)

#define CTI_EN_MASK		0x0F
#define CTI_CTRL_OFFSET		0x0
#define CTI_INTACK_OFFSET	0x10
#define CTI_EN_IN0_OFFSET	0x20
#define CTI_EN_IN1_OFFSET	0x24
#define CTI_EN_IN2_OFFSET	0x28
#define CTI_EN_IN3_OFFSET	0x2C
#define CTI_EN_IN4_OFFSET	0x30
#define CTI_EN_IN5_OFFSET	0x34
#define CTI_EN_IN6_OFFSET	0x38
#define CTI_EN_IN7_OFFSET	0x3C
#define CTI_EN_OUT0_OFFSET	0xA0
#define CTI_EN_OUT1_OFFSET	0xA4
#define CTI_EN_OUT2_OFFSET	0xA8
#define CTI_EN_OUT3_OFFSET	0xAC
#define CTI_EN_OUT4_OFFSET	0xB0
#define CTI_EN_OUT5_OFFSET	0xB4
#define CTI_EN_OUT6_OFFSET	0xB8
#define CTI_EN_OUT7_OFFSET	0xBC
#define CTI_LOCK_OFFSET		0xFB0

#ifdef CONFIG_CORESIGHT_SUPPORT
void coresight_dump_pcsr(u32 cpu);
void coresight_panic_locked_cpu(int cpu);
#else
#define coresight_dump_pcsr(cpu)     do {} while (0)
#define coresight_panic_locked_cpu(cpu)	do {} while (0)
#endif

#ifdef CONFIG_CORESIGHT_TRACE_SUPPORT
void coresight_ptm_disable(u32 cpu);
#else
#define coresight_ptm_disable(cpu) do {} while (0)
#endif

#endif /* __ASM_MACH_CORSIGHT_H */
