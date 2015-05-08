/*
 * linux/arch/arm/mach-mmp/include/mach/regs-apbc.h
 *
 *   Application Peripheral Bus Clock Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_APBC_H
#define __ASM_MACH_REGS_APBC_H

#include <mach/addr-map.h>

/* Common APB clock register bit definitions */
#define APBC_APBCLK	(1 << 0)  /* APB Bus Clock Enable */
#define APBC_FNCLK	(1 << 1)  /* Functional Clock Enable */
#define APBC_RST	(1 << 2)  /* Reset Generation */

/* Functional Clock Selection Mask */
#define APBC_FNCLKSEL(x)	(((x) & 0xf) << 4)

#define APBC_MMPX_TIMER0	APBC_REG(0x034)
#define APBC_MMPX_TIMER1	APBC_REG(0x044)
#define APBC_MMPX_TIMER2	APBC_REG(0x068)
#define APBC_PXA988_KPC		APBC_REG(0x030)

/* Clock Control Register for Generic Counter */
#define APBC_COUNTER_CLK_SEL    APBC_REG(0x64)
#define FREQ_HW_CTRL            0x1
#define FREQ_SW_SEL             0x2

/* memory maped registers for generic timers */
#define CNTCR			0x00	 /* Counter Control Register */
#define CNTCR_EN		(1 << 0) /* The counter is enabled */
#define CNTCR_HDBG		(1 << 1) /* Halt on debug */

#define CNTSR			0x04	 /* Counter Status Register */
#define CNTCVLW			0x08	 /* Current value of counter[31:0] */
#define CNTCVUP			0x0C	 /* Current value of counter[63:32] */
#define CNTFID0			0x20	 /* Base frequency ID */

#endif /* __ASM_MACH_REGS_APBC_H */
