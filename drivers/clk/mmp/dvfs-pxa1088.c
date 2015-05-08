/*
 *  linux/arch/arm/mach-mmp/dvfs-pxa1x88.c
 *
 *  based on arch/arm/mach-tegra/tegra2_dvfs.c
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 *  Xiaoguang Chen <chenxg@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mfd/88pm80x.h>
#include <linux/clk/mmp.h>
#include <mach/addr-map.h>
#include <mach/dvfs-dvc.h>
#include <mach/cputype.h>

#ifdef CONFIG_TZ_HYPERVISOR
#include <linux/pxa_tzlc.h>
#endif

enum {
	CORE = 0,
	DDR,
	GC,
	GC2D,
	ISP,
	VPU,
	VM_RAIL_MAX,
};

#define VL_MAX	4
#define mV2uV	1000

#ifndef CONFIG_TZ_HYPERVISOR
#define APMU_GEU	0x068
#define UIMAINFUSE_31_00	0x410
#define UIMAINFUSE_63_32	0x414
#define UIMAINFUSE_95_64	0x418
#define BLOCK0_224_255		0x420
/* For chip UID */
#define UID_H_32	0x48C
#define UID_L_32	0x4A8
#endif

/* components frequency combination */
static unsigned long freqs_cmb_1088[][VL_MAX] = {
	{312000, 312000, 800000, 1482000},	/* CORE */
	{156000, 312000, 400000, 533000},	/* DDR/AXI */
	{0, 416000, 416000, 624000},	/* GC */
	{0, 312000, 416000, 416000},	/* GC2D */
	{312000, 416000, 416000, 416000},	/* ISP */
	{0, 312000, 416000, 533000}	/* VPU */
};

/* get from Wilcox JBP : prevent kernel lock up*/
static int vm_mv_1088_svc_1p2G[][VL_MAX] = {
	{1075, 1075, 1250, 1375},	/* profile 0 */
	{1075, 1075, 1100, 1225},	/* profile 1 */
	{1075, 1075, 1100, 1225},	/* profile 2 */
	{1075, 1075, 1100, 1238},	/* profile 3 */
	{1075, 1075, 1125, 1288},	/* profile 4 */
	{1075, 1075, 1150, 1325},	/* profile 5 */
	{1075, 1075, 1188, 1363},	/* profile 6 */
	{1075, 1075, 1225, 1375},	/* profile 7 */
	{1075, 1075, 1250, 1375},	/* profile 8 */
};

static int vm_mv_1088_svc_1p25G[][VL_MAX] = {
	{1025, 1075, 1250, 1375},	/* profile 0 */
	{1025, 1075, 1088, 1250},	/* profile 1 */
	{1025, 1075, 1088, 1250},	/* profile 2 */
	{1025, 1075, 1088, 1263},	/* profile 3 */
	{1025, 1075, 1125, 1313},	/* profile 4 */
	{1025, 1075, 1150, 1350},	/* profile 5 */
	{1025, 1075, 1188, 1388},	/* profile 6 */
	{1025, 1075, 1225, 1375},	/* profile 7 */
	{1025, 1075, 1250, 1375},	/* profile 8 */
};

static struct dvfs_rail_component vm_rail_comp_tbl_dvc[] = {
	INIT_DVFS("cpu", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[CORE][0]),
#if 0
	INIT_DVFS("cpu", true, (AFFECT_RAIL_ACTIVE),
		  &freqs_cmb_1L88[CORE][0]),
#endif
	INIT_DVFS("ddr", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[DDR][0]),
	INIT_DVFS("GCCLK", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[GC][0]),
	INIT_DVFS("GC2DCLK", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[GC2D][0]),
	INIT_DVFS("isp-clk", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[ISP][0]),
	INIT_DVFS("VPUCLK", true,
		  (AFFECT_RAIL_ACTIVE | AFFECT_RAIL_M2 | AFFECT_RAIL_D1P),
		  &freqs_cmb_1088[VPU][0]),
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

static struct dvc_plat_info dvc_pxa1088_info = {
	.comps = vm_rail_comp_tbl_dvc,
	.num_comps = ARRAY_SIZE(vm_rail_comp_tbl_dvc),
	.num_volts = VL_MAX,
	.cp_pmudvc_lvl = VL3,
	.dp_pmudvc_lvl = VL3,
	.dvc_pin_switch = 1,
	.set_vccmain_volt = set_pmic_volt,
	.get_vccmain_volt = get_pmic_volt,
	.pmic_rampup_step = 12500,
	.dbglvl = 1,
	.regname = "BUCK1",
	.no_extra_timer = true,
};

#define NUM_PROFILES	8
static unsigned int uichipprofile;
static u64 uluid;
static unsigned int pm_dro_status;

static unsigned int convert_fuses_to_profile(unsigned int uifuses)
{
	unsigned int uiprofile = 0;
	unsigned int uitemp = 3, uitemp2 = 3;
	unsigned int i;

	for (i = 0; i < NUM_PROFILES; i++) {
		uitemp |= uitemp2 << (i * 2);
		if (uitemp == uifuses)
			uiprofile = i + 1;
	}

	return uiprofile;
}

static unsigned int get_profile(void)
{
	return uichipprofile;
}

static int __init __init_read_droinfo(void)
{
	void __iomem *apmu_base, *geu_base;

	unsigned int ui_main_fuse_63_32 = 0;
	unsigned int ui_main_fuse_95_64 = 0;
	unsigned int ui_main_fuse_31_00 = 0;
	unsigned int uiallocrev;
	unsigned int uifab;
	unsigned int uirun;
	unsigned int uiwafer;
	unsigned int uix;
	unsigned int uiy;
	unsigned int uiparity;
	unsigned int uidro_avg;
	unsigned int uigeustatus;
	unsigned int uifuses;
#ifdef CONFIG_TZ_HYPERVISOR
	int ret = -1;
	tzlc_cmd_desc cmd_desc;
	tzlc_handle tzlc_hdl;
#endif

	if (!cpu_is_pxa1088())
		return 0;

#ifndef CONFIG_TZ_HYPERVISOR
	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return -1;
	}

	geu_base = ioremap(AXI_PHYS_BASE + 0x1000, SZ_4K);
	if (geu_base == NULL) {
		pr_err("error to ioremap GEU base\n");
		return -1;
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

	ui_main_fuse_31_00 = __raw_readl(geu_base + UIMAINFUSE_31_00);
	ui_main_fuse_63_32 = __raw_readl(geu_base + UIMAINFUSE_63_32);
	ui_main_fuse_95_64 = __raw_readl(geu_base + UIMAINFUSE_95_64);
	uifuses = __raw_readl(geu_base + BLOCK0_224_255);
	uluid = __raw_readl(geu_base + UID_H_32);
	uluid = (uluid << 32) | __raw_readl(geu_base + UID_L_32);

	__raw_writel(uigeustatus, apmu_base + APMU_GEU);
#else
	/*
	 * with TrustZone enabled, fuse related info should be read in
	 * secure world.
	 */
	tzlc_hdl = pxa_tzlc_create_handle();

	memset(&cmd_desc, 0, sizeof(tzlc_cmd_desc));
	cmd_desc.op = TZLC_CMD_READ_MANUFACTURING_BITS;
	ret = pxa_tzlc_cmd_op(tzlc_hdl, &cmd_desc);

	if (ret == 0) {
		uifuses = cmd_desc.args[0];
		ui_main_fuse_95_64 = cmd_desc.args[1];
		ui_main_fuse_63_32 = cmd_desc.args[2];
		ui_main_fuse_31_00 = cmd_desc.args[3];
	}

	memset(&cmd_desc, 0, sizeof(tzlc_cmd_desc));
	cmd_desc.op = TZLC_CMD_READ_OEM_UNIQUE_ID;
	ret = pxa_tzlc_cmd_op(tzlc_hdl, &cmd_desc);

	if (ret == 0) {
		uluid = cmd_desc.args[0];
		uluid = (uluid << 32) | cmd_desc.args[1];
	}

	pxa_tzlc_destroy_handle(tzlc_hdl);
#endif

	pr_info("  0x%x   0x%x   0x%x\n",
		ui_main_fuse_31_00, ui_main_fuse_63_32, ui_main_fuse_95_64);

	uiallocrev = ui_main_fuse_31_00 & 0x7;
	uifab = (ui_main_fuse_31_00 >> 3) & 0x1f;
	uirun = ((ui_main_fuse_63_32 & 0x3) << 24)
	    | ((ui_main_fuse_31_00 >> 8) & 0xffffff);
	uiwafer = (ui_main_fuse_63_32 >> 2) & 0x1f;
	uix = (ui_main_fuse_63_32 >> 7) & 0xff;
	uiy = (ui_main_fuse_63_32 >> 15) & 0xff;
	uiparity = (ui_main_fuse_63_32 >> 23) & 0x1;
	uidro_avg = (ui_main_fuse_95_64 >> 4) & 0x3ff;

	/* bit 240 ~ 255 for Profile information */
	uifuses = (uifuses >> 16) & 0x0000FFFF;
	uichipprofile = convert_fuses_to_profile(uifuses);

	pm_dro_status = uidro_avg;

	pr_info(" ");
	pr_info("	*********************************\n");
	pr_info("	*	ULT: %08X%08X	*\n",
		ui_main_fuse_63_32, ui_main_fuse_31_00);
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
	pr_info("		DRO [77:68]	= %d\n", uidro_avg);
	pr_info("		Profile	= %d\n", uichipprofile);
	pr_info("		UID	= %llx\n", uluid);
	pr_info("	*********************************\n");
	pr_info("\n");

	return uidro_avg;
}

pure_initcall(__init_read_droinfo);

static int __init setup_pxa1088_dvfs_platinfo(void)
{
	unsigned int uiprofile = get_profile();
	void __iomem *hwdvc_base;
	if (!cpu_is_pxa1088())
		return 0;
	hwdvc_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	if (hwdvc_base == NULL) {
		pr_err("error to ioremap hwdvc base\n");
		return -EINVAL;
	}
	if (max_freq > CORE_1p2G)
		dvc_pxa1088_info.millivolts = vm_mv_1088_svc_1p25G[uiprofile];
	else
		dvc_pxa1088_info.millivolts = vm_mv_1088_svc_1p2G[uiprofile];
	dvc_pxa1088_info.dvc_reg_base = hwdvc_base;
	return dvfs_setup_dvcplatinfo(&dvc_pxa1088_info);
}

core_initcall_sync(setup_pxa1088_dvfs_platinfo);
