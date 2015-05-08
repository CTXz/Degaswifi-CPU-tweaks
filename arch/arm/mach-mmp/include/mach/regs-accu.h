/*
 * linux/arch/arm/mach-mmp/include/mach/regs-accu.h
 *
 *   Application Clock Control Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_ACCU_H
#define __ASM_MACH_REGS_ACCU_H

/*
 * ACCU register offsets for PXA1986
 */
#define ACCU_FABRIC_N1_CLK_CNTRL_REG		(0x004)
#define ACCU_APPS_MDMA_CLK_CNTRL_REG		(0x00C)
#define ACCU_APPS_STM_CLK_CNTRL_REG		(0x010)
#define ACCU_APPS_IC_CLK_CNTRL_REG		(0x014)
#define ACCU_WTM_MAIN_CLK_CNTRL_REG		(0x018)
#define ACCU_WTM_CORE_CLK_CNTRL_REG		(0x01C)
#define ACCU_WTM_CORE_BUS_CLK_CNTRL_REG		(0x020)
#define ACCU_FABRIC_N2_CLK_CNTRL_REG		(0x030)
#define ACCU_VID_DEC_CLK_CNTRL_REG		(0x034)
#define ACCU_VID_ENC_CLK_CNTRL_REG		(0x038)
#define ACCU_FABRIC_N3_CLK_CNTRL_REG		(0x040)
#define ACCU_USB_CLK_CNTRL_REG			(0x044)
#define ACCU_HSIC_CLK_CNTRL_REG			(0x048)
#define ACCU_NAND_CLK_CNTRL_REG			(0x04C)
#define ACCU_FABRIC_N4_CLK_CNTRL_REG		(0x058)
#define ACCU_APPS_INT_SRAM_CLK_CNTRL_REG	(0x05C)
#define ACCU_VDMA_CLK_CNTRL_REG			(0x060)
#define ACCU_CI1_CCIC_CLK_CNTRL_REG		(0x064)
#define ACCU_CI2_CCIC_CLK_CNTRL_REG		(0x068)
#define ACCU_ISP_CLK_CNTRL_REG			(0x06C)
#define ACCU_DISPLAY1_CLK_CNTRL_REG		(0x074)
#define ACCU_DISPLAY2_CLK_CNTRL_REG		(0x078)
#define ACCU_DISPLAY_UNIT_CLK_CNTRL_REG		(0x07C)
#define ACCU_FABRIC_N5_CLK_CNTRL_REG		(0x080)
#define ACCU_GC1_3D_CLK_CNTRL_REG		(0x084)
#define ACCU_GC2_3D_CLK_CNTRL_REG		(0x088)
#define ACCU_GC_2D_CLK_CNTRL_REG		(0x08C)
#define ACCU_FABRIC_N6_CLK_CNTRL_REG		(0x090)
#define ACCU_APPS_LSP_APB_CLK_CNTRL_REG		(0x0A0)
#define ACCU_APPS_OW_CLK_CNTRL_REG		(0x0A4)
#define ACCU_APPS_PWM01_CLK_CNTRL_REG		(0x0A8)
#define ACCU_APPS_PWM23_CLK_CNTRL_REG		(0x0AC)
#define ACCU_APPS_TIMERS1_CLK_CNTRL_REG		(0x0B0)
#define ACCU_APPS_TIMERS2_CLK_CNTRL_REG		(0x0B4)
#define ACCU_APPS_TIMERS3_CLK_CNTRL_REG		(0x0B8)
#define ACCU_APPS_KP_CLK_CNTRL_REG		(0x0C4)
#define ACCU_APPS_RTC_CLK_CNTRL_REG		(0x0C8)
#define ACCU_APPS_SSP1_CLK_CNTRL_REG		(0x0D0)
#define ACCU_APPS_SSP2_CLK_CNTRL_REG		(0x0D4)
#define ACCU_APPS_I2C1_CLK_CNTRL_REG		(0x0DC)
#define ACCU_APPS_I2C2_CLK_CNTRL_REG		(0x0E0)
#define ACCU_APPS_I2C3_CLK_CNTRL_REG		(0x0E4)
#define ACCU_APPS_I2C4_CLK_CNTRL_REG		(0x0E8)
#define ACCU_APPS_I2C5_CLK_CNTRL_REG		(0x0EC)
#define ACCU_APPS_UART1_CLK_CNTRL_REG		(0x0F0)
#define ACCU_APPS_UART2_CLK_CNTRL_REG		(0x0F4)
#define ACCU_APPS_UART3_CLK_CNTRL_REG		(0x0F8)
#define ACCU_APPS_TEMP_S1_CLK_CNTRL_REG		(0x100)
#define ACCU_APPS_TEMP_S2_CLK_CNTRL_REG		(0x104)
#define ACCU_APPS_TEMP_S3_CLK_CNTRL_REG		(0x108)
#define ACCU_APPS_CORE_B_CLK_CNTRL_REG		(0x120)
#define ACCU_APPS_CORE_B_CLK_CNTRL2_REG		(0x124)
#define ACCU_APPS_CORE_L_CLK_CNTRL_REG		(0x12C)
#define ACCU_APPS_CORE_L_CLK_CNTRL2_REG		(0x130)
#define ACCU_APPS_CORE_TOP_CLK_CNTRL_REG	(0x138)
#define ACCU_APPS_CORE_CS_CLK_CNTRL_REG		(0x13C)
#define ACCU_SDH_BUS_CLK_CNTRL_REG		(0x170)
#define ACCU_SDH_CLK_CNTRL_REG			(0x174)
#define ACCU_HSI_CLK_CNTRL_REG			(0x17C)

/* Common ACCU clock register bit definitions */
#define ACCU_ST_BIT	BIT(29)
#define ACCU_GO_BIT	BIT(28)

#define ACCU_AHBCLK	BIT(10)
#define ACCU_APBCLK	BIT(9)  /* Bus Clock Enable */
#define ACCU_FNCLK	BIT(8)  /* Functional Clock Enable */

#define ACCU_AHBRST	BIT(2)
#define ACCU_APBRST	BIT(1)
#define ACCU_RST	BIT(0)  /* Reset Generation */

/* Functional Clock Selection Mask */
#define ACCU_RATIO_MASK		(0xf << 20)
#define ACCU_SOURCE_MASK	(0x7 << 16)

/* Functional Clock Set */
#define SET_ACCU_RATIO(x)	(((x) & 0xf) << 20)
#define SET_ACCU_SOURCE(x)	(((x) & 0x7) << 16)

/* Functional Clock Get */
#define GET_ACCU_RATIO(x)	((x & ACCU_RATIO_MASK) >> 20)
#define GET_ACCU_SOURCE(x)	((x & ACCU_SOURCE_MASK) >> 16)

#define ACCU_GC3D_INTLCLK	(1 << 12)
#define ACCU_GC3D_AHBCLK	(1 << 11)
#define ACCU_GC3D_APBCLK	(1 << 10)  /* Bus Clock Enable */
#define ACCU_GC3D_SHADERCLK	(1 << 9)
#define ACCU_GC3D_FNCLK		(1 << 8)  /* Functional Clock Enable */

#endif /* __ASM_MACH_REGS_ACCU_H */
