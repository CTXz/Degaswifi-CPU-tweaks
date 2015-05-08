/*
 * linux/drivers/iommu/mmp-iommu.h
 *
 * This is lite version of the MMU400 iommu driver for Marvell silicon.
 *
 * Copyright:   (C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define SMMU_SCR0		0x0
#define SMMU_CR0		0x0
#define SMMU_SCR1		0x4
#define SMMU_ACR		0x10
#define SMMU_SACR		0x10
#define SMMU_IDR0		0x20
#define SMMU_IDR1		0x24
#define SMMU_IDR2		0x28
#define SMMU_IDR7		0x3c
#define SMMU_SGFAR		0x40
#define SMMU_GFAR		0x40
#define SMMU_SGFAR_H		0x44
#define SMMU_GFAR_H		0x44
#define SMMU_GFSR		0x48
#define SMMU_SGFSR		0x48
#define SMMU_GFSRRESTORE	0x4c
#define SMMU_SGFSRRESTORE	0x4c
#define SMMU_GFSYNR0		0x50
#define SMMU_SGFSYNR0		0x50
#define SMMU_GFSYNR1		0x54
#define SMMU_SGFSYNR1		0x54
#define SMMU_STLBIALL		0x60
#define SMMU_TLBIVMID		0x64
#define SMMU_TLBIALLNSNH	0x68
#define SMMU_TLBIALLH		0x6c
#define SMMU_STLBGSYNC		0x70
#define SMMU_TLBGSYNC		0x70
#define SMMU_STLBGSTATUS	0x74
#define SMMU_TLBGSTATUS		0x74
#define SMMU_DBGRPTR		0x80
#define SMMU_DBGRDATA		0x84
#define SMMU_NSCR0		0x400
#define SMMU_NSACR		0x410
#define SMMU_NSGFAR		0x440
#define SMMU_NSGFAR_H		0x444
#define SMMU_NSGFSR		0x448
#define SMMU_NSGFSRRESTORE	0x44c
#define SMMU_NSGFSYNR0		0x450
#define SMMU_NSGFSYNR1		0x454
#define SMMU_NSTLBGSYNC		0x470
#define SMMU_NSTLBGSTATUS	0x474
#define SMMU_SMR(n)		(0x800 + (n << 2))
#define SMMU_S2CR(n)		(0xc00 + (n << 2))

#define SMMU_CBFRSYNRA(n)	(0x1400 + (n << 2))
#define SMMU_CBAR(n)		(0x1000 + (n << 2))

#define ITCTRL			0x2000
#define ITOP			0x2004
#define ITIP			0x2008

#define PMEVCNTR(n)		(0x3000 + (n << 2))
#define PMEVTYPE(n)		(0x3400 + (n << 2))
#define PMCGCR			0x3800
#define PMCGSMR			0x3A00
#define PMCNTENSET		0x3C00
#define PMCNTENCLR		0x3C20
#define PMINTENSET		0x3C40
#define PMINTENCLR		0x3C60
#define PMOVSCLR		0x3C80
#define PMOVSSET		0x3CC0
#define PMCFGR			0x3E00
#define PMCR			0x3E04
#define PMCEID(n)		(0x3E20 + (n << 2))
#define PMAUTHSTATUS		0x3FB8
#define PMDEVTYPE		0x3FCC

#define SMMU_SSDR(n)		(0x4000 + (n << 2))

#define PERIPH_ID4		0xFFD0
#define PERIPH_ID5		0xFFD4
#define PERIPH_ID6		0xFFD8
#define PERIPH_ID7		0xFFDC
#define PERIPH_ID0		0xFFE0
#define PERIPH_ID1		0xFFE4
#define PERIPH_ID2		0xFFE8
#define PERIPH_ID3		0xFFEC

#define COMPONENT_ID0		0xFFF0
#define COMPONENT_ID1		0xFFF4
#define COMPONENT_ID2		0xFFF8
#define COMPONENT_ID3		0xFFFC

#define SMMU_CB_SCTLR		0x0
#define SMMU_CB_TTBR0_L		0x20
#define SMMU_CB_TTBR0_H		0x24
#define SMMU_CB_TTBCR		0x30
#define SMMU_CB_FSR		0x58
#define SMMU_CB_FSRRESTORE	0x5C
#define SMMU_CB_FAR		0x60
#define SMMU_CB_FAR_H		0x64
#define SMMU_CB_FSYNR0		0x68
#define SMMU_CB_PMXEVCNTR	0xE00
#define SMMU_CB_PMXEVTYPER	0xE80
#define SMMU_CB_PMCFGR		0xF00
#define SMMU_CB_PMCR		0xF04
#define SMMU_CB_PMCEID0		0xF20
#define SMMU_CB_PMCNTENSET	0xF40
#define SMMU_CB_PMCNTENCLR	0xF44
#define SMMU_CB_PMINTENSET	0xF48
#define SMMU_CB_PMINTENCLR	0xF4C
#define SMMU_CB_PMOVSRCLR	0xF54
#define SMMU_CB_PMOVSRSET	0xF50
#define SMMU_CB_PMAUTHSTATUS	0xFB8

#define sCR0_CLIENTPD		(1 << 0)
#define sCR0_GFRE		(1 << 1)
#define sCR0_GFIE		(1 << 2)
#define sCR0_GCFGFRE		(1 << 4)
#define sCR0_GCFGFIE		(1 << 5)
#define sCR0_USFCFG		(1 << 10)
#define sCR0_VMIDPNE		(1 << 11)
#define sCR0_PTM		(1 << 12)
#define sCR0_FB			(1 << 13)
#define sCR0_BSU_SHIFT		14
#define sCR0_BSU_MASK		0x3

struct mmp_iommu_drvdata {
	struct list_head node_domains;
	void __iomem	*regbase;
	struct device	*dev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
};

struct mmp_device_node {
	struct list_head node;
	struct device *dev;
};

struct mmp_iommu_domain {
	struct mmp_iommu_drvdata *drvdata;
	struct list_head node;
	struct list_head node_devs;
	u64 *pgd;
	spinlock_t lock;
	spinlock_t pgtablelock;
};
