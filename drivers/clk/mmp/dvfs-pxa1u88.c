/*
 *  linux/arch/arm/mach-mmp/dvfs-pxa1u88.c
 *
 *  based on arch/arm/mach-mmp/dvfs-pxa1x88.c
 *  Copyright (C) 2013 Mrvl, Inc. by Zhoujie Wu <zjwu@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/88pm80x.h>
#include <mach/addr-map.h>
#include <mach/dvfs-dvc.h>
#include <mach/cputype.h>

/* components that affect the vmin */
enum dvfs_comp {
	CORE = 0,
	DDR,
	AXI,
	GC3D,
	GC2D,
	GC_SHADER,
	VPU,
	ISP,
	VM_RAIL_MAX,
};

#define VL_MAX	4

#define ACTIVE_RAIL_FLAG	(AFFECT_RAIL_ACTIVE)
#define ACTIVE_M2_RAIL_FLAG	(AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2)
#define ACTIVE_M2_D1P_RAIL_FLAG \
	(AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P)

/* Fuse information related register definition */
#define APMU_GEU		0x068
#define UIMAINFUSE_31_00	0x410
#define UIMAINFUSE_63_32	0x414
#define UIMAINFUSE_95_64	0x418
#define BLOCK0_224_255		0x420
/* For chip unique ID */
#define UID_H_32		0x48C
#define UID_L_32		0x4A8
static u64 uluid;
/* For chip DRO and profile */
#define NUM_PROFILES	11
static unsigned int uiprofile;
static unsigned int uidro;

/* components frequency combination */
/* FIXME: below freq-cmb and svc should be adjusted after SVC is avaliable */
static unsigned long freqs_cmb_1u88a0[VM_RAIL_MAX][VL_MAX] = {
	{ 312000, 624000, 800000, 1183000 },	/* CORE */
	{ 312000, 400000, 400000, 533000 },	/* DDR */
	{ 208000, 208000, 312000, 312000 },	/* AXI */
	{ 0, 416000, 624000, 624000 },		/* GC3D */
	{ 416000, 416000, 624000, 624000 },	/* GC2D */
	{ 0, 416000, 624000, 710000 },		/* GC Shader */
	{ 312000, 416000, 533000, 533000 },	/* VPU */
	{ 312000, 416000, 416000, 416000 },	/* ISP */
};

/* 1u88 A0 SVC table, CP/MSA votes VL2 by default */
static int vm_millivolts_1u88a0_svc[][VL_MAX] = {
	/* FIXME: use typical voltage for 28nm */
	{1100, 1100, 1100, 1125},
#if 0
	{1113, 1225, 1300, 1375},	/* profile 0 */
	{1050, 1088, 1125, 1150},	/* profile 1 */
	{1050, 1088, 1125, 1150},	/* profile 2 */
	{1050, 1088, 1125, 1150},	/* profile 3 */
	{1050, 1088, 1138, 1175},	/* profile 4 */
	{1050, 1088, 1150, 1213},	/* profile 5 */
	{1050, 1100, 1175, 1250},	/* profile 6 */
	{1100, 1138, 1200, 1288},	/* profile 7 */
	{1100, 1150, 1225, 1325},	/* profile 8 */
	{1100, 1175, 1250, 1350},	/* profile 9 */
	{1100, 1200, 1275, 1375},	/* profile 10 */
	{1113, 1225, 1300, 1375},	/* profile 11 */
#endif
};

/*
 * dvfs_rail_component.freqs is inited dynamicly, due to different stepping
 * may have different VL combination
 */
static struct dvfs_rail_component vm_rail_comp_tbl_dvc[VM_RAIL_MAX] = {
	/* Jira Emei-285 is fixed, core only affects active voltage */
	INIT_DVFS("cpu", true, ACTIVE_RAIL_FLAG, NULL),
#if 0
	/* setting in case Jira 285 is not fixed */
	INIT_DVFS("cpu", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
#endif
	INIT_DVFS("ddr", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
	INIT_DVFS("axi", true, ACTIVE_M2_RAIL_FLAG, NULL),
	INIT_DVFS("GCCLK", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
	INIT_DVFS("GC2DCLK", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
	INIT_DVFS("GC_SHADER_CLK", true,
		ACTIVE_M2_D1P_RAIL_FLAG, NULL),
	INIT_DVFS("VPUCLK", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
	INIT_DVFS("isp-clk", true, ACTIVE_M2_D1P_RAIL_FLAG, NULL),
};

static int set_pmic_volt(unsigned int lvl, unsigned int mv)
{
	return pm8xx_dvc_setvolt(PM800_ID_BUCK1, lvl, mv * mV2uV);
}

static int get_pmic_volt(unsigned int lvl)
{
	int uv = 0, ret = 0;

	ret = pm8xx_dvc_getvolt(PM800_ID_BUCK1, lvl, &uv);
	if (ret < 0)
		return ret;
	return DIV_ROUND_UP(uv, mV2uV);
}

static struct dvc_plat_info dvc_pxa1u88_info = {
	.comps = vm_rail_comp_tbl_dvc,
	.num_comps = ARRAY_SIZE(vm_rail_comp_tbl_dvc),
	.num_volts = VL_MAX,
	/* FIXME: CP/MSA VL may need to be adjusted */
	.cp_pmudvc_lvl = VL2,
	.dp_pmudvc_lvl = VL2,
	.dvc_pin_switch = 0,
	.set_vccmain_volt = set_pmic_volt,
	.get_vccmain_volt = get_pmic_volt,
	.pmic_rampup_step = 12500,
	/* by default use debug print lvl */
	.dbglvl = 1,
	.regname = "BUCK1",
};

static int __init setup_pxa1u88_dvfs_platinfo(void)
{
	void __iomem *hwdvc_base;
	enum dvfs_comp idx;
	struct dvc_plat_info *plat_info = &dvc_pxa1u88_info;
	unsigned long (*freqs_cmb)[VL_MAX];

	if (!cpu_is_pxa1U88())
		return 0;

	/* Here may need to identify chip stepping */
	dvc_pxa1u88_info.millivolts = vm_millivolts_1u88a0_svc[0];
	freqs_cmb = freqs_cmb_1u88a0;

	/* register the platform info into dvfs-dvc.c(hwdvc driver) */
	hwdvc_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	if (hwdvc_base == NULL) {
		pr_err("error to ioremap hwdvc base\n");
		return -EINVAL;
	}
	plat_info->dvc_reg_base = hwdvc_base;
	for (idx = CORE; idx < VM_RAIL_MAX; idx++)
		plat_info->comps[idx].freqs = freqs_cmb[idx];
	return dvfs_setup_dvcplatinfo(&dvc_pxa1u88_info);
}
core_initcall_sync(setup_pxa1u88_dvfs_platinfo);

/* FIXME: copied from pxa1L88, may need to be adjusted */
static unsigned int convert_fuses2profile(unsigned int uifuses)
{
	unsigned int uiprofile = 0;
	unsigned int uitemp = 3, uitemp2 = 1;
	unsigned int i;

	for (i = 0; i < NUM_PROFILES; i++) {
		uitemp |= uitemp2 << (i + 1);
		if (uitemp == uifuses)
			uiprofile = i + 1;
	}
	return uiprofile;
}

static int __init __init_read_droinfo(void)
{
	void __iomem *apmu_base, *geu_base;

	unsigned int uimainFuse_63_32 = 0;
	unsigned int uimainFuse_95_64 = 0;
	unsigned int uimainFuse_31_00 = 0;
	unsigned int uiallocrev;
	unsigned int uifab;
	unsigned int uirun;
	unsigned int uiwafer;
	unsigned int uix;
	unsigned int uiy;
	unsigned int uiparity;
	unsigned int uigeustatus;
	unsigned int uifuses;

	if (!cpu_is_pxa1U88())
		return 0;

	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return -EINVAL;
	}

	geu_base = ioremap(AXI_PHYS_BASE + 0x1000, SZ_4K);
	if (geu_base == NULL) {
		pr_err("error to ioremap GEU base\n");
		return -EINVAL;
	}

	/*
	 * Read out DRO value, need enable GEU clock, if already disable,
	 * need enable it firstly
	 */
	uigeustatus = __raw_readl(apmu_base + APMU_GEU);
	if (!(uigeustatus & 0x08)) {
		__raw_writel((uigeustatus | 0x09), apmu_base + APMU_GEU);
		udelay(10);
	}
	uimainFuse_31_00 = __raw_readl(geu_base + UIMAINFUSE_31_00);
	uimainFuse_63_32 = __raw_readl(geu_base + UIMAINFUSE_63_32);
	uimainFuse_95_64 = __raw_readl(geu_base + UIMAINFUSE_95_64);
	uifuses = __raw_readl(geu_base + BLOCK0_224_255);
	uluid = __raw_readl(geu_base + UID_H_32);
	uluid = (uluid << 32) | __raw_readl(geu_base + UID_L_32);
	__raw_writel(uigeustatus, apmu_base + APMU_GEU);

	pr_info("  0x%x   0x%x   0x%x\n",
		uimainFuse_31_00, uimainFuse_63_32,
		uimainFuse_95_64);

	uiallocrev	= uimainFuse_31_00 & 0x7;
	uifab		= (uimainFuse_31_00 >>  3) & 0x1f;
	uirun		= ((uimainFuse_63_32 & 0x3) << 24)
			| ((uimainFuse_31_00 >> 8) & 0xffffff);
	uiwafer		= (uimainFuse_63_32 >>  2) & 0x1f;
	uix		= (uimainFuse_63_32 >>  7) & 0xff;
	uiy		= (uimainFuse_63_32 >> 15) & 0xff;
	uiparity	= (uimainFuse_63_32 >> 23) & 0x1;
	uidro		= (uimainFuse_95_64 >>  4) & 0x3ff;

	/* bit 240 ~ 255 for Profile information */
	uifuses = (uifuses >> 16) & 0x0000FFFF;
	uiprofile = convert_fuses2profile(uifuses);

	pr_info(" ");
	pr_info("	*********************************\n");
	pr_info("	*	ULT: %08X%08X	*\n",
		uimainFuse_63_32, uimainFuse_31_00);
	pr_info("	*********************************\n");
	pr_info("	 ULT decoded below\n");
	pr_info("		alloc_rev[2:0]	= %d\n", uiallocrev);
	pr_info("		fab [ 7: 3]	= %d\n", uifab);
	pr_info("		run [33: 8]	= %d (0x%X)\n", uirun, uirun);
	pr_info("		wafer [38:34]	= %d\n", uiwafer);
	pr_info("		x [46:39]	= %d\n", uix);
	pr_info("		y [54:47]	= %d\n", uiy);
	pr_info("		parity [55:55]	= %d\n", uiparity);
	pr_info("	*********************************\n");
	pr_info("	*********************************\n");
	pr_info("		DRO [77:68]	= %d\n", uidro);
	pr_info("		Profile	= %d\n", uiprofile);
	pr_info("		UID	= %llx\n", uluid);
	pr_info("	*********************************\n");
	pr_info("\n");
	return 0;
}
pure_initcall(__init_read_droinfo);
