/*
 * linux/arch/arm/mach-mmp/include/mach/regs-mpmu.h
 *
 *   Main Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_MPMU_H
#define __ASM_MACH_REGS_MPMU_H

#ifdef CONFIG_CPU_PXA1986
/*
 * MPMU_PCR_0 : used by the Cortex-A7 CPU0 Apps Subsystem
 * MPMU_PCR_1 : used by the Secure Processor subsystem
 * MPMU_PCR_2 : used by the Comm Cortex R-5 subsystem core (aka CP1)
 * MPMU_PCR_3 : used by the Comm MSA subsystem (aka CP2)
 * MPMU_PCR_4 : used by the Audio subsystem
 */
#define MPMU_PCR_0		(0x0000)
#define MPMU_PCR_1		(0x0004)
#define MPMU_PCR_2		(0x0008)
#define MPMU_PCR_3		(0x000C)
#define MPMU_PCR_4		(0x0010)
#define MPMU_VCXOCR		(0x0014)
#define MPMU_DFCCR		(0x0018)
#define MPMU_PLL1DLY		(0x001C)
#define MPMU_PLL2DLY		(0x0020)
#define MPMU_PLLDDLY		(0x0024)
#define MPMU_PLLADLY		(0x0028)
#define MPMU_ANAGCR		(0x002C)
#define MPMU_MCKCR		(0x0030)
#define MPMU_EXVCXOCR		(0x0034)
#define MPMU_RSTOCR		(0x0038)
#define MPMU_AP_PLLREQ		(0x003C)
#define MPMU_CP_PLLREQ		(0x0040)
#define MPMU_AUD_PLLREQ		(0x0044)
#define MPMU_SRD_PLLREQ		(0x0048)
#define MPMU_RSTSR		(0x004C)
#define MPMU_PWRDLY		(0x0050)
#define MPMU_CKEN32K		(0x0054)
#define MPMU_RST32K		(0x0058)
#define MPMU_CPCTL		(0x005C)
#define MPMU_AUDSSCR		(0x0060)
#define MPMU_RSRV1		(0x0064)
#define MPMU_PLLSTAT		(0x0068)
#define MPMU_SLPIND_CR		(0x006C)
#define MPMU_PLL1_CF1		(0x0070)
#define MPMU_PLL1_CF2		(0x0074)
#define MPMU_PLL2_CF1		(0x0078)
#define MPMU_PLL2_CF2		(0x007C)
#define MPMU_PLL2_CF3		(0x0080)
#define MPMU_PLLD_CF1		(0x0084)
#define MPMU_PLLD_CF2		(0x0088)
#define MPMU_PLLD_CF3		(0x008C)
#define MPMU_PLLD_CF4		(0x0090)
#define MPMU_PLLA_CF1		(0x0094)
#define MPMU_PLLA_CF2		(0x0098)
#define MPMU_PCSTAT		(0x00A0)
#define MPMU_PLL1FCGO		(0x00A4)
#define MPMU_PLL2FCGO		(0x00A8)
#define MPMU_PLLDFCGO		(0x00AC)
#define MPMU_PLLAFCGO		(0x00B0)
#define MPMU_LPMUDLY0		(0x00B4)
#define MPMU_LPMUDLY1		(0x00B8)
#define MPMU_LPMUDLY2		(0x00BC)
#define MPMU_TROPCR		(0x00C0)
#define MPMU_AUD_PMUDLY		(0x00C4)
#define MPMU_TRO_PMUDLY		(0x00C8)
#define MPMU_CK32KCR		(0x00CC)
#define MPMU_MIPSHL_CR		(0x00D0)
#define MPMU_RSRV2		(0x00D4)
#define MPMU_IOPADCR		(0x00D8)
#define MPMU_PLL1SEL		(0x00DC)
#define MPMU_WKUP_STAT		(0x00E0)
#define MPMU_GENSW1		(0x00E4)
#define MPMU_GENSW2		(0x00E8)
#define MPMU_GENSW3		(0x00EC)
#define MPMU_GENSW4		(0x00F0)
#define MPMU_AUDVDLY		(0x00F4)
#define MPMU_AVS_SNSCR		(0x00F8)
/*
 * MPMU_PCR_AP_1 : used by the Cortex-A7 CPU1 Apps Subsystem
 * MPMU_PCR_AP_2 : used by the Cortex-A15 CPU0 Apps Subsystem
 * MPMU_PCR_AP_3 : used by the Cortex-A15 CPU1 Apps Subsystem
 */
#define MPMU_PCR_AP_1		(0x0100)
#define MPMU_PCR_AP_2		(0x0104)
#define MPMU_PCR_AP_3		(0x0108)
#define MAVS_LVDDL		(0x0120)
#define MAVS_HVDDL		(0x0124)
#define MAVS_DELTA		(0x0128)
#define MAVS_SPDT_0		(0x012C)
#define MAVS_SPDT_1		(0x0130)
#define MAVS_EN_GEN		(0x0134)
#define MAVS_VC			(0x0138)
#define MAVS_GP1		(0x013C)
#define MAVS_GP2		(0x0140)
#define MAVS_DRO_CFG		(0x0144)
#define MAVS_TP			(0x0148)
#define MAVS_DRO_CNT_STATUS1	(0x014C)
#define MAVS_DRO_CNT_STATUS2	(0x0150)
#define MAVS_STATUS1		(0x0154)
#define MAVS_STATUS2		(0x0158)
#define CAVS_LVDDL		(0x0160)
#define CAVS_HVDDL		(0x0164)
#define CAVS_DELTA		(0x0168)
#define CAVS_SPDT_0		(0x016C)
#define CAVS_SPDT_1		(0x0170)
#define CAVS_EN_GEN		(0x0174)
#define CAVS_VC			(0x0178)
#define CAVS_GP1		(0x017C)
#define CAVS_GP2		(0x0180)
#define CAVS_DRO_CFG		(0x0184)
#define CAVS_TP			(0x0188)
#define CAVS_DRO_CNT_STATUS1	(0x018C)
#define CAVS_DRO_CNT_STATUS2	(0x0190)
#define CAVS_STATUS1		(0x0194)
#define CAVS_STATUS2		(0x0198)
#define GAVS_LVDDL		(0x01A0)
#define GAVS_HVDDL		(0x01A4)
#define GAVS_DELTA		(0x01A8)
#define GAVS_SPDT_0		(0x01AC)
#define GAVS_SPDT_1		(0x01B0)
#define GAVS_EN_GEN		(0x01B4)
#define GAVS_VC			(0x01B8)
#define GAVS_GP1		(0x01BC)
#define GAVS_GP2		(0x01C0)
#define GAVS_DRO_CFG		(0x01C4)
#define GAVS_TP			(0x01C8)
#define GAVS_DRO_CNT_STATUS1	(0x01CC)
#define GAVS_DRO_CNT_STATUS2	(0x01D0)
#define GAVS_STATUS1		(0x01D4)
#define GAVS_STATUS2		(0x01D8)
#define MDRO_CLCTR_CTL		(0x01E0)
#define CDRO_CLCTR_CTL		(0x01E4)
#define GDRO_CLCTR_CTL		(0x01E8)
#endif /* CONFIG_CPU_PXA1986 */

#endif /* __ASM_MACH_REGS_MPMU_H */
