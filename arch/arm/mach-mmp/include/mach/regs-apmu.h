/*
 * linux/arch/arm/mach-mmp/include/mach/regs-apmu.h
 *
 *   Application Subsystem Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_APMU_H
#define __ASM_MACH_REGS_APMU_H

#include <mach/addr-map.h>

#if defined(CONFIG_CPU_PXA1986)
/*
 * APMU register offsets for PXA1986
 */
#define APMU_PCR_0			(0x0000)
#define APMU_PCR_1			(0x0004)
#define APMU_PCR_2			(0x0008)
#define APMU_PCR_3			(0x000C)
#define APMU_PCR_4			(0x0010)
#define APMU_PSR			(0x0014)
#define APMU_CORE_RESET_0		(0x0018)
#define APMU_CORE_RESET_1		(0x001C)
#define APMU_CORE_RESET_2		(0x0020)
#define APMU_CORE_RESET_3		(0x0024)
#define APMU_CORE_RESET_4		(0x0028)
#define APMU_CORE_STATUS		(0x002C)
#define APMU_SP_IDLE_CFG		(0x0030)
#define APMU_CORE_IDLE_CFG_0		(0x0034)
#define APMU_CORE_IDLE_CFG_1		(0x0038)
#define APMU_CORE_IDLE_CFG_2		(0x003C)
#define APMU_CORE_IDLE_CFG_3		(0x0040)
#define APMU_RES_FRM_SLP_0		(0x0044)
#define APMU_RES_FRM_SLP_1		(0x0048)
#define APMU_RES_FRM_SLP_2		(0x004C)
#define APMU_RES_FRM_SLP_3		(0x0050)
#define APMU_AP_CLK_REQ			(0x0054)
#define APMU_AP_CLK_ST			(0x0058)
#define APMU_GPU_PWR_CTRL		(0x005C)
#define APMU_IMG_PWR_CTRL		(0x0060)
#define APMU_USB_PWR_CTRL		(0x0064)
#define APMU_VIDEO_PWR_CTRL		(0x0068)
#define APMU_GPU_PWR_STATUS		(0x006C)
#define APMU_IMG_PWR_STATUS		(0x0070)
#define APMU_USB_PWR_STATUS		(0x0074)
#define APMU_VIDEO_PWR_STATUS		(0x0078)
#define APMU_PWR_ISL_TIMER		(0x007C)
#define APMU_PLL3_CTL_0			(0x0080)
#define APMU_PLL3_CTL_1			(0x0084)
#define APMU_PLL3_CTL_2			(0x0088)
#define APMU_PLL3_CTL_3			(0x008C)
#define APMU_PLL3_CTL_4			(0x0090)
#define APMU_CORE_PLL1_CTL_0		(0x0094)
#define APMU_CORE_PLL1_CTL_1		(0x0098)
#define APMU_CORE_PLL1_CTL_2		(0x009C)
#define APMU_CORE_PLL1_CTL_3		(0x00A0)
#define APMU_CORE_PLL1_CTL_4		(0x00A4)
#define APMU_PWR_STBL_TIMER		(0x00A8)
#define APMU_POCR			(0x00AC)
#define APMU_SRAM_PWR_DWN		(0x00B0)
#define APMU_GP_LPM_TIMER		(0x00B4)
#define APMU_GP_CORE_TIMER		(0x00B8)
#define APMU_GP_LPI_TIMER		(0x00BC)
#define APMU_VCC_SEL			(0x00C8)
#define APMU_PLL4_CTL_0			(0x00CC)
#define APMU_PLL4_CTL_1			(0x00D0)
#define APMU_PLL4_CTL_2			(0x00D4)
#define APMU_PLL4_CTL_3			(0x00D8)
#define APMU_PLL4_CTL_4			(0x00DC)
#define APMU_USB3_PMNG			(0x00E0)
#define APMU_USB3_PSTAT			(0x00E4)
#define AP_WAKEUP_ENABLE_SET		(0x00E8)
#define AP_WAKEUP_ENABLE_CLR		(0x00EC)
#define APMU_TEST_REG			(0x00F0)
#define APMU_CORE_PLL2_CTL_0		(0x00F4)
#define APMU_CORE_PLL2_CTL_1		(0x00F8)
#define APMU_CORE_PLL2_CTL_2		(0x00FC)
#define APMU_CORE_PLL2_CTL_3		(0x0100)
#define APMU_CORE_PLL2_CTL_4		(0x0104)
#define APMU_FORCE_CPU_IDLE		(0x0108)
#define APMU_HDMI_PLL_GATE		(0x010C)
#define APMU_SLP_IND_CLR		(0x0110)
#define APMU_CORE_IDLE_ST		(0x0114)
#define APMU_RST_EXIT_1			(0x0118)
#define APMU_RST_EXIT_2			(0x011C)
#define APMU_RST_EXIT_3			(0x0120)
#define APMU_RST_EXIT_4			(0x0124)
#define APMU_INT_MASK_0			(0x0128)
#define APMU_INT_MASK_1			(0x012C)
#define APMU_INT_MASK_2			(0x0130)
#define APMU_INT_MASK_3			(0x0134)
#define APMU_WKP_STKY_SEL		(0x0138)
#define APMU_WKP_ST			(0x013C)

/* APMU_GPU_PWR_CTRL */
#define GPU1_3D_PWR_ON			BIT(0)
#define GPU2_3D_PWR_ON			BIT(4)
#define GPU_SS_PWR_ON			BIT(8)
/* APMU_VIDEO_PWR_CTRL */
#define VIDEO_SS_PWR_ON			BIT(0)
#define ISP_DMA_PWR_ON_OFF		BIT(4)
#define DISPLAY_VDMA_PWR_ON_OFF		BIT(8)

/* APMU_IMG_PWR_STATUS */
#define ISP_DMA_POWER_STATUS		BIT(4)
#define DISPLAY_VDMA_POWER_STATUS	BIT(8)

/* APMU_USB_PWR_CTRL */
#define USB_SS_PWR_ON			BIT(0)

#else /* CONFIG_CPU_PXA1986 */

#define APMU_FNCLK_EN	(1 << 4)
#define APMU_AXICLK_EN	(1 << 3)
#define APMU_FNRST_DIS	(1 << 1)
#define APMU_AXIRST_DIS	(1 << 0)

/* Wake Clear Register */
#define APMU_WAKE_CLR	APMU_REG(0x07c)

#define APMU_PXA168_KP_WAKE_CLR		(1 << 7)
#define APMU_PXA168_CFI_WAKE_CLR	(1 << 6)
#define APMU_PXA168_XD_WAKE_CLR		(1 << 5)
#define APMU_PXA168_MSP_WAKE_CLR	(1 << 4)
#define APMU_PXA168_SD4_WAKE_CLR	(1 << 3)
#define APMU_PXA168_SD3_WAKE_CLR	(1 << 2)
#define APMU_PXA168_SD2_WAKE_CLR	(1 << 1)
#define APMU_PXA168_SD1_WAKE_CLR	(1 << 0)

#ifdef CONFIG_CPU_EDEN
#define PMUA_APCORESS_MP2               (0x3 << 8)
#define PMUA_L2SR_PWROFF_VT             (0x1 << 4)
#define PMUA_CPU_PWRMODE_C2             (0x3 << 0)

#define APMU_CORE0_PWRMODE              APMU_REG(0x0280)
#define APMU_CORE1_PWRMODE              APMU_REG(0x0284)
#define APMU_CORE2_PWRMODE              APMU_REG(0x0288)
#define APMU_CORE3_PWRMODE              APMU_REG(0x028C)
#endif /* CONFIG_CPU_EDEN */

/* Debug register */
#define APMU_DEBUG             APMU_REG(0x0088)
#define APMU_CORE_STATUS       APMU_REG(0x090)

#define APMU_WAKEUP_CORE(n)		(1 << (n & 0x3))
#define APMU_COREn_WAKEUP_CTL(n)	(APMU_REG(0x012c) + 4 * (n & 0x3))
#endif /* !CONFIG_CPU_PXA1986 */

#endif /* __ASM_MACH_REGS_APMU_H */
