/*
 * linux/arch/arm/mach-mmp/include/mach/addr-map.h
 *
 *   Common address map definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_ADDR_MAP_H
#define __ASM_MACH_ADDR_MAP_H

/* APB - Application Subsystem Peripheral Bus
 *
 * NOTE: the DMA controller registers are actually on the AXI fabric #1
 * slave port to AHB/APB bridge, due to its close relationship to those
 * peripherals on APB, let's count it into the ABP mapping area.
 */
#define APB_PHYS_BASE		0xd4000000
#define APB_VIRT_BASE		IOMEM(0xfe000000)

#ifdef CONFIG_CPU_PXA1986
#define APB_PHYS_SIZE          0x00a00000
#else
#define APB_PHYS_SIZE		0x00200000
#endif

#define AXI_PHYS_BASE		0xd4200000
#define AXI_VIRT_BASE		IOMEM(APB_VIRT_BASE + APB_PHYS_SIZE)
#define AXI_PHYS_SIZE		0x00200000

#if defined(CONFIG_CPU_EDEN)
#define DMCU_PHYS_BASE		0xd0000000
#define DMCU_VIRT_BASE		IOMEM(0xfe500000)
#define DMCU_PHYS_SIZE		0x00010000
#endif

#if defined(CONFIG_CPU_PXA1986)
#define MMP_CORE_PERIPH_PHYS_BASE      0xd0020000
#define MMP_CORE_PERIPH_PHYS_SIZE      0x0000a000
#else
#define MMP_CORE_PERIPH_PHYS_BASE	0xd1dfe000
#define MMP_CORE_PERIPH_PHYS_SIZE	0x00002000
#endif

#define MMP_CORE_PERIPH_VIRT_BASE      IOMEM(AXI_VIRT_BASE + AXI_PHYS_SIZE)

/* Static Memory Controller - Chip Select 0 and 1 */
#define SMC_CS0_PHYS_BASE	0x80000000
#define SMC_CS0_PHYS_SIZE	0x10000000
#define SMC_CS1_PHYS_BASE	0x90000000
#define SMC_CS1_PHYS_SIZE	0x10000000

/*
 * below definition is used in eden-dt.c, and will be removed
 * after eden-Zx stepping is not supported.
 */
#define AUD_PHYS_BASE           0xc0ffd800

#define AUD_PHYS_BASE2          0xc0140000

/* audio MAP base */
#define AUD_MAP_BASE			0xd1200000
#define AUD_AUX_BASE			0xd1230000

#ifdef CONFIG_CPU_PXA1986
#define APMU_VIRT_BASE		(APB_VIRT_BASE + 0x80000)
#else
#define APMU_VIRT_BASE		(AXI_VIRT_BASE + 0x82800)
#endif
#define APMU_REG(x)		IOMEM(APMU_VIRT_BASE + (x))

#define APMU_PHY_BASE  (AXI_PHYS_BASE + 0x82800)

#define APBC_VIRT_BASE		(APB_VIRT_BASE + 0x015000)
#define APBC_REG(x)		(APBC_VIRT_BASE + (x))

#define MPMU_VIRT_BASE		(APB_VIRT_BASE + 0x50000)
#define MPMU_REG(x)		(MPMU_VIRT_BASE + (x))

#define CIU_VIRT_BASE		(AXI_VIRT_BASE + 0x82c00)
#define CIU_REG(x)		(CIU_VIRT_BASE + (x))

#define SCU_VIRT_BASE		(MMP_CORE_PERIPH_VIRT_BASE)

#define GIC_DIST_VIRT_BASE	(MMP_CORE_PERIPH_VIRT_BASE + 0x1000)
#define GIC_DIST_PHYS_BASE  (MMP_CORE_PERIPH_PHYS_BASE + 0x1000)


#endif /* __ASM_MACH_ADDR_MAP_H */
