/*
 * linux/arch/arm/mach-mmp/include/mach/regs-ciu.h
 *
 *  CPU Interface Unit Registers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_CIU_H
#define __ASM_MACH_REGS_CIU_H

#include <mach/addr-map.h>

#define CIU_VIRT_BASE		(AXI_VIRT_BASE + 0x82c00)
#define CIU_REG(x)		(CIU_VIRT_BASE + (x))

#if defined(CONFIG_CPU_PXA988) || defined(CONFIG_CPU_PXA1088)

#define CIU_CHIP_ID				CIU_REG(0x0000)
#define CIU_SEAGULL_CPU_CONF			CIU_REG(0x0004)
#define CIU_SEAGULL_CPU_SRAM_SPD		CIU_REG(0x000c)
#define CIU_SEAGULL_CPU_L2C_SRAM_SPD		CIU_REG(0x0014)
#define CIU_SYS_BOOT_CNTRL			CIU_REG(0x0020)
#define CIU_SW_BRANCH_ADDR			CIU_REG(0x0024)
#define CIU_PERF_COUNT2				CIU_REG(0x003c)
#define CIU_MC_CONF				CIU_REG(0x0040)
#define CIU_CS_CONF				CIU_REG(0x004c)
#define CIU_CS_DEBUG_CONF			CIU_REG(0x0050)
#define CIU_MCB_CONFIG2_REG			CIU_REG(0x008c)
#define CIU_DDR_PHY_TST_CONFIG_REG		CIU_REG(0x0090)
#define CIU_DDR_PHY_TST_SEED_REG		CIU_REG(0x0094)
#define CIU_DDR_PHY_TST_SIGNATURE_REG		CIU_REG(0x0098)
#define CIU_DDR_PHY_TST_STATUS_REG		CIU_REG(0x009c)
#define CIU_GPU2D_XTC_REG			CIU_REG(0x00a0)
#define CIU_GPU_XTC_REG				CIU_REG(0x00a4)
#define CIU_VPU_XTC_REG				CIU_REG(0x00a8)
#define CIU_CPU_CONF_ADDR_FILTER		CIU_REG(0x00b0)
#define CIU_CPU_CONF_L2C			CIU_REG(0x00b4)
#define CIU_CPU_CONF_SCU			CIU_REG(0x00b8)
#define CIU_CPU_CONF_MISC			CIU_REG(0x00bc)
#define CIU_CPU_CONF_CORESIGHT_ROM_ADDR		CIU_REG(0x00c0)
#define CIU_CPU_CONF_CORESIGHT_SELF_ADDR	CIU_REG(0x00c4)
#define CIU_CPU_CONF_SRAM_0			CIU_REG(0x00c8)
#define CIU_CPU_CONF_SRAM_1			CIU_REG(0x00cc)
#define CIU_CPU_CORE0_CONF			CIU_REG(0x00d0)
#define CIU_CPU_CORE0_STATUS			CIU_REG(0x00d4)
#define CIU_WARM_RESET_VECTOR			CIU_REG(0x00d8)
#define CIU_CPU_CORE1_CONF			CIU_REG(0x00e0)
#define CIU_CPU_CORE1_STATUS			CIU_REG(0x00e4)
#define CIU_SW_SCRATCH_REG			CIU_REG(0x00e8)
#define CIU_CPU_CORE2_CONF			CIU_REG(0x00f0)
#define CIU_CPU_CORE2_STATUS			CIU_REG(0x00f4)
#define CIU_CPU_CORE3_CONF			CIU_REG(0x00f8)
#define CIU_CPU_CORE3_STATUS			CIU_REG(0x00fc)
#endif

#endif /* __ASM_MACH_REGS_CIU_H */
