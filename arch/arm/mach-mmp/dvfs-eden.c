/*
 *  linux/arch/arm/mach-mmp/dvfs-eden.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk/dvfs.h>
#include <linux/mfd/88pm80x.h>
#include <linux/regulator/88pg870.h>

#include <mach/cputype.h>
#include <mach/addr-map.h>

#ifndef CONFIG_TZ_HYPERVISOR
#define WTM_VIRT_BASE		(AXI_VIRT_BASE + 0x90000)
#define WTM_REG(x)		IOMEM(WTM_VIRT_BASE + (x))
#define WTM_FUSE_BLK11		WTM_REG(0x2A34)
#endif

/* bit definition of APMU_AP_DEBUG1 */
#define APMU_AP_DEBUG1		APMU_REG(0x38C)
#define DVC_UPDATE_DISABLE	(1 << 24)
/* end */

#define APMU_DVC_APCORESS		APMU_REG(0x350)
/* bit definition of APMU_DVC_APSS */
#define APMU_DVC_APSS		APMU_REG(0x354)
#define VC_INT			(1 << 31)
#define VC_INT_MSK		(1 << 30)
#define VC_INT_CLR		(1 << 29)
#define VL_D2_SHIFT		20
#define VL_D1_SHIFT		16
#define VL_D0CG_SHIFT		8
#define VL_D0_SHIFT		0
#define VL_RW_MSK		0x7
/*end*/

/* bit definition of MPMU_DVC_STB1 */
#define MPMU_DVC_STB1		MPMU_REG(0x1140)
#define VL0_VL1_SHIFT		0
#define VL1_VL2_SHIFT		16
/* end */

/* bit definition of MPMU_DVC_STB2 */
#define MPMU_DVC_STB2		MPMU_REG(0x1144)
#define VL2_VL3_SHIFT		0
#define VL3_VL4_SHIFT		16
/* end */

#define MPMU_DVC_STB3		MPMU_REG(0x1148)
#define MPMU_DVC_STB4		MPMU_REG(0x114C)

/* bit definition of MPMU_DVC_STB2 */
#define MPMU_DVC_DEBUG		MPMU_REG(0x1150)
#define DVC_AP_MSK		(1 << 0)
#define DVC_CP_MSK		(1 << 1)
/* end */

#define EDEN_DVC_LEVEL_NUM	4
#define PMIC_RAMP_RATE		12500	/* uV/uS */
#define DVC_STB_UNIT		38	/* nS */
#define US_TO_NS		1000

static unsigned int mv_profile;
/***************
 * EDEN_DVC_D0
 ***************/
static int eden_dvc_set_d0_level(struct dvfs_rail *rail, int new_level);
static struct dvfs_rail eden_dvfs_rail_d0 = {
	.reg_id = "dvc_d0",
	.max_millivolts = EDEN_DVC_LEVEL_NUM - 1,
	.min_millivolts = 0,
	.nominal_millivolts = EDEN_DVC_LEVEL_NUM - 1,
	.step = EDEN_DVC_LEVEL_NUM,
	.update_inatomic = 1,
	.set_volt = eden_dvc_set_d0_level,
};

enum {
	CORE = 0,
	GC3D, GC2D, DDR, VPUEN, VPUDE,
	EDEN_DVC_D0_FACTOR_NUM,
};

static char *eden_dvfs_d0_clk_name[] = {
	[CORE]	= "cpu",
	[GC3D]	= "GC3D_CLK1X",
	[GC2D]	= "GC2D_CLK",
	[DDR]	= "ddr",
	[VPUEN]	= "VPU_ENC_CLK",
	[VPUDE]	= "VPU_DEC_CLK",
};

static const int eden_dvfs_d0_threshold[][EDEN_DVC_LEVEL_NUM + 1] = {
	[CORE]	= { 1, 0, 531000000, 797000000, 1057000000}, /* HZ */
	[GC3D]	= { 1, 0, 312000000, 312000000, 528666666}, /* HZ */
	[GC2D]	= { 1, 0, 312000000, 528666666, 528666666}, /* HZ */
	[DDR]	= { 0, 0, 312000, 312000, 528000}, /* kHZ */
	[VPUEN]	= { 1, 0, 312000000, 416000000, 416000000}, /* HZ */
	[VPUDE]	= { 1, 0, 264333333, 416000000, 416000000}, /* HZ */
};

static const int eden_dvfs_d0_threshold_z2[][EDEN_DVC_LEVEL_NUM + 1] = {
	[CORE]	= { 1, 0, 531000000, 797000000, 1386000000}, /* HZ */
	[GC3D]	= { 1, 0, 312000000, 312000000, 528666666}, /* HZ */
	[GC2D]	= { 1, 0, 312000000, 528666666, 528666666}, /* HZ */
	[DDR]	= { 0, 0, 312000, 312000, 528000}, /* kHZ */
	[VPUEN]	= { 1, 0, 312000000, 416000000, 416000000}, /* HZ */
	[VPUDE]	= { 1, 0, 264333333, 416000000, 416000000}, /* HZ */
};

static int eden_dvfs_d0_init(void)
{
	struct dvfs *vd = NULL;
	struct vol_table *vt = NULL;
	struct clk *c;
	unsigned long rate;
	const int (*thr_tbl)[EDEN_DVC_D0_FACTOR_NUM][EDEN_DVC_LEVEL_NUM + 1];
	int i, j;
	int ret = 0;

	if (cpu_is_eden_z1())
		thr_tbl = &eden_dvfs_d0_threshold;
	else
		thr_tbl = &eden_dvfs_d0_threshold_z2;

	for (i = 0; i < EDEN_DVC_D0_FACTOR_NUM; i++) {
		/* factor is not enabled */
		if (!(*thr_tbl)[i][0])
			continue;

		vd = kzalloc(sizeof(struct dvfs), GFP_KERNEL);
		if (!vd) {
			pr_err("DVFS: Failed to request dvfs struct!\n");
			ret = -ENOMEM;
			continue;
		}
		vd->num_freqs = EDEN_DVC_LEVEL_NUM;

		vt = kzalloc(sizeof(struct vol_table) * vd->num_freqs,
			     GFP_KERNEL);
		if (!vt) {
			pr_err("DVFS: Failed to request vol table\n");
			kzfree(vd);
			ret = -ENOMEM;
			continue;
		}

		for (j = 1; j < EDEN_DVC_LEVEL_NUM + 1; j++) {
			vt[j - 1].freq = (*thr_tbl)[i][j];
			vt[j - 1].millivolts = j - 1;
		}
		vd->vol_freq_table = vt;
		vd->clk_name = eden_dvfs_d0_clk_name[i];
		vd->dvfs_rail = &eden_dvfs_rail_d0;

		c = clk_get_sys(NULL, vd->clk_name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("DVFS-D0: no clk found:%s\n", vd->clk_name);
			kzfree(vd->vol_freq_table);
			kzfree(vd);
			ret = -EINVAL;
			continue;
		}
		ret = enable_dvfs_on_clk(c, vd);
		if (ret) {
			pr_err("DVFS: fail to enable dvfs on %s\n", c->name);
			kzfree(vd->vol_freq_table);
			kzfree(vd);
		}
		if (c->enable_count) {
			rate = clk_get_rate(c);
			while (i < vd->num_freqs &&
				rate > vd->vol_freq_table[i].freq)
				i++;
			vd->millivolts = vd->vol_freq_table[i].millivolts;
			vd->cur_rate = rate;
		}
	}
	return ret;
}

static int eden_dvc_set_d0_level(struct dvfs_rail *rail, int new_level)
{
	u32 apss, dbg;
	int i;

	pr_debug("[DVC-D0] set to %d.\n", new_level);

	/* enable and clear irq */
	apss = readl(APMU_DVC_APSS);
	apss &= ~VC_INT_MSK;
	apss |= VC_INT_CLR;
	writel(apss, APMU_DVC_APSS);

	/* disable DVC update */
	dbg = readl(APMU_AP_DEBUG1);
	dbg |= DVC_UPDATE_DISABLE;
	writel(dbg, APMU_AP_DEBUG1);
	/* get synced with clk */
	for (i = 0; i < 5; i++)
		dbg = readl(APMU_AP_DEBUG1);

	/* set D0 level value */
	new_level = new_level & VL_RW_MSK;
	apss &= ~(VL_RW_MSK << VL_D0_SHIFT);
	apss |= new_level << VL_D0_SHIFT;
	writel(apss, APMU_DVC_APSS);

	/* enable DVC update */
	dbg &= ~DVC_UPDATE_DISABLE;
	writel(dbg, APMU_AP_DEBUG1);

	/* wait for irq */
	for (i = 0; i < 100000; i++) {
		if (readl(APMU_DVC_APSS) & VC_INT)
			break;
	}
	if (i == 100000) {
		pr_err("no ack irq from dvc!\n");
		return -EINVAL;
	} else {
		/* clear irq bit */
		apss = readl(APMU_DVC_APSS);
		apss |= VC_INT_CLR;
		writel(apss, APMU_DVC_APSS);
		pr_debug("[DVC-D0] changing finished.\n");
		return 0;
	}
}

/***************
 * EDEN_DVC_D0CG
 ***************/
static int eden_dvc_set_d0cg_level(struct dvfs_rail *rail, int new_level);
static struct dvfs_rail eden_dvfs_rail_d0cg = {
	.reg_id = "dvc_d0cg",
	.max_millivolts = EDEN_DVC_LEVEL_NUM - 1,
	.min_millivolts = 0,
	.nominal_millivolts = EDEN_DVC_LEVEL_NUM - 1,
	.step = EDEN_DVC_LEVEL_NUM,
	.update_inatomic = 1,
	.set_volt = eden_dvc_set_d0cg_level,
};

enum {
	GC3D_D0CG = 0,
	GC2D_D0CG, DDR_D0CG, VPUEN_D0CG, VPUDE_D0CG,
	EDEN_DVC_D0CG_FACTOR_NUM,
};

static char *eden_dvfs_d0cg_clk_name[] = {
	[GC3D_D0CG]	= "GC3D_CLK1X",
	[GC2D_D0CG]	= "GC2D_CLK",
	[DDR_D0CG]	= "ddr",
	[VPUEN_D0CG]	= "VPU_ENC_CLK",
	[VPUDE_D0CG]	= "VPU_DEC_CLK",
};

static const int eden_dvfs_d0cg_threshold[][EDEN_DVC_LEVEL_NUM + 1] = {
	[GC3D_D0CG]	= { 1, 312000000, 312000000, 312000000, 528666666},
	[GC2D_D0CG]	= { 1, 312000000, 312000000, 528666666, 528666666},
	[DDR_D0CG]	= { 0, 312000, 312000, 312000, 528000}, /* kHZ */
	[VPUEN_D0CG] = { 1, 312000000, 312000000, 416000000, 416000000},
	[VPUDE_D0CG] = { 1, 264333333, 264333333, 416000000, 416000000},
};

static int eden_dvfs_d0cg_init(void)
{
	struct dvfs *vd = NULL;
	struct vol_table *vt = NULL;
	struct clk *c;
	unsigned long rate;
	int i, j;
	int ret = 0;

	for (i = 0; i < EDEN_DVC_D0CG_FACTOR_NUM; i++) {
		/* factor is not enabled */
		if (!eden_dvfs_d0cg_threshold[i][0])
			continue;

		vd = kzalloc(sizeof(struct dvfs), GFP_KERNEL);
		if (!vd) {
			pr_err("DVFS: Failed to request dvfs struct!\n");
			ret = -ENOMEM;
			continue;
		}
		vd->num_freqs = EDEN_DVC_LEVEL_NUM;

		vt = kzalloc(sizeof(struct vol_table) * vd->num_freqs,
			     GFP_KERNEL);
		if (!vt) {
			pr_err("DVFS: Failed to request vol table\n");
			kzfree(vd);
			ret = -ENOMEM;
			continue;
		}

		for (j = 1; j < EDEN_DVC_LEVEL_NUM + 1; j++) {
			vt[j - 1].freq = eden_dvfs_d0cg_threshold[i][j];
			vt[j - 1].millivolts = j - 1;
		}
		vd->vol_freq_table = vt;
		vd->clk_name = eden_dvfs_d0cg_clk_name[i];
		vd->dvfs_rail = &eden_dvfs_rail_d0cg;

		c = clk_get_sys(NULL, vd->clk_name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("DVFS-D0CG: no clk found:%s\n", vd->clk_name);
			kzfree(vd->vol_freq_table);
			kzfree(vd);
			ret = -EINVAL;
			continue;
		}
		ret = enable_dvfs_on_clk(c, vd);
		if (ret) {
			pr_err("DVFS: fail to enable dvfs on %s\n", c->name);
			kzfree(vd->vol_freq_table);
			kzfree(vd);
		}
		if (c->enable_count) {
			rate = clk_get_rate(c);
			while (i < vd->num_freqs &&
				rate > vd->vol_freq_table[i].freq)
				i++;
			vd->millivolts = vd->vol_freq_table[i].millivolts;
			vd->cur_rate = rate;
		}
	}
	return ret;
}


static int eden_dvc_set_d0cg_level(struct dvfs_rail *rail, int new_level)
{
	u32 apss;

	pr_debug("[DVC-D0CG] set to %d.\n", new_level);
	apss = readl(APMU_DVC_APSS);
	apss &= ~(VL_RW_MSK << VL_D0CG_SHIFT);
	apss |= new_level << VL_D0CG_SHIFT;
	writel(apss, APMU_DVC_APSS);
	pr_debug("[DVC-D0CG] changing finished.\n");
	return 0;
}

/***************
 * DVFS
 ***************/
static unsigned int eden_dvfs_map_table[][EDEN_DVC_LEVEL_NUM] = {
	/* profile 0 */
	[0] = {1225000, 1225000, 1225000, 1225000},
	/* profile 1 */
	[1] = {887500, 887500, 987500, 1162500},
	/* profile 2 */
	[2] = {987500, 987500, 987500, 1162500},
	/* profile 3 */
	[3] = {987500, 987500, 1025000, 1200000},
	/* profile 4 */
	[4] = {1037500, 1037500, 1037500, 1200000},
	/* profile 5 */
	[5] = {1100000, 1100000, 1112500, 1200000},
	/* profile 6 */
	[6] = {1112500, 1112500, 1112500, 1212500},
	/* profile 7 */
	[7] = {1112500, 1112500, 1112500, 1212500},
	/* profile 8 */
	[8] = {1225000, 1225000, 1225000, 1225000},
};

static unsigned int eden_dvfs_map_table_z2[][EDEN_DVC_LEVEL_NUM] = {
	/* profile 0 */
	[0] = {1225000, 1225000, 1225000, 1225000},
	/* profile 1 */
	[1] = {887500, 887500, 987500, 1162500},
	/* profile 2 */
	[2] = {987500, 987500, 987500, 1162500},
	/* profile 3 */
	[3] = {987500, 987500, 1025000, 1200000},
	/* profile 4 */
	[4] = {1037500, 1037500, 1037500, 1200000},
	/* profile 5 */
	[5] = {1100000, 1100000, 1112500, 1200000},
	/* profile 6 */
	[6] = {1112500, 1112500, 1112500, 1212500},
	/* profile 7 */
	[7] = {1112500, 1112500, 1112500, 1212500},
	/* profile 8 */
	[8] = {1225000, 1225000, 1225000, 1225000},
};

static void pmic_init_setup(void)
{
	unsigned int (*dvc_map)[EDEN_DVC_LEVEL_NUM];
	int i;

	if (cpu_is_eden_z1())
		dvc_map = &eden_dvfs_map_table[mv_profile];
	else
		dvc_map = &eden_dvfs_map_table_z2[mv_profile];

	for (i = 0; i < EDEN_DVC_LEVEL_NUM; i++)
		pg870_dvc_setvolt(i, (*dvc_map)[i]);
}

static void eden_dvfs_setup(void)
{
	u32 val, dbg, stb_val[3];
	int i;
	unsigned int vol_gap, delay;
	unsigned int (*dvc_map)[EDEN_DVC_LEVEL_NUM];

	/* set DVC delay counters */
	if (cpu_is_eden_z1())
		dvc_map = &eden_dvfs_map_table[mv_profile];
	else
		dvc_map = &eden_dvfs_map_table_z2[mv_profile];
	for (i = 0; i < 3; i++) {
		vol_gap = (*dvc_map)[i + 1] - (*dvc_map)[i];
		delay = vol_gap / PMIC_RAMP_RATE;	/* uS */
		stb_val[i] = delay * US_TO_NS / DVC_STB_UNIT;	/* nS */
		/* add 15200nS (400 * 38) more delay here */
		stb_val[i] += 400;
	}
	val = (stb_val[0] << VL0_VL1_SHIFT) | (stb_val[1] << VL1_VL2_SHIFT);
	writel(val, MPMU_DVC_STB1);
	val = (stb_val[2] << VL2_VL3_SHIFT) | (0xFFFF << VL3_VL4_SHIFT);
	writel(val, MPMU_DVC_STB2);
	writel(0xFFFFFFFF, MPMU_DVC_STB3);
	writel(0xFFFFFFFF, MPMU_DVC_STB4);

	/* unmask AP DVC bit */
	val = readl(MPMU_DVC_DEBUG);
	val &= DVC_AP_MSK;
	writel(val, MPMU_DVC_DEBUG);

	/* enable and clear irq */
	val = readl(APMU_DVC_APSS);
	val &= ~VC_INT_MSK;
	val |= VC_INT_CLR;
	writel(val, APMU_DVC_APSS);

	/* disable DVC update */
	dbg = readl(APMU_AP_DEBUG1);
	dbg |= DVC_UPDATE_DISABLE;
	writel(dbg, APMU_AP_DEBUG1);
	/* get synced with clk */
	for (i = 0; i < 5; i++)
		val = readl(APMU_AP_DEBUG1);

	val = readl(APMU_DVC_APSS);
	/* D2 */
	val &= ~(VL_RW_MSK << VL_D2_SHIFT);
	val |= (0 << VL_D2_SHIFT);
	/* D1 */
	val &= ~(VL_RW_MSK << VL_D1_SHIFT);
	val |= (0 << VL_D1_SHIFT);
	/* D0CG */
	val &= ~(VL_RW_MSK << VL_D0CG_SHIFT);
	val |= ((EDEN_DVC_LEVEL_NUM - 1) << VL_D0CG_SHIFT);
	/* D0 */
	val &= ~(VL_RW_MSK << VL_D0_SHIFT);
	val |= ((EDEN_DVC_LEVEL_NUM - 1) << VL_D0_SHIFT);
	writel(val, APMU_DVC_APSS);

	writel(0x0, APMU_DVC_APCORESS);

	/* enable DVC update */
	dbg &= ~DVC_UPDATE_DISABLE;
	writel(dbg, APMU_AP_DEBUG1);

	/* wait for irq */
	for (i = 0; i < 100000; i++) {
		if (readl(APMU_DVC_APSS) & VC_INT)
			break;
	}
	if (i == 100000)
		pr_err("no ack irq from dvc!\n");
	else {
		/* clear irq bit */
		val = readl(APMU_DVC_APSS);
		val |= VC_INT_CLR;
		writel(val, APMU_DVC_APSS);
	}
}

static struct dvfs_rail *eden_dvfs_rails[] = {
	&eden_dvfs_rail_d0,
	&eden_dvfs_rail_d0cg,
};

static void read_mv_profile(void)
{
#ifndef CONFIG_TZ_HYPERVISOR
	static int dro_map_table[8] = {444, 424, 402, 384, 366, 347, 322, 1};
	unsigned int uifuseblk11_159_128;
	unsigned int uisvtdro_avg;
	int i;

	uifuseblk11_159_128 = readl(WTM_FUSE_BLK11);
	uisvtdro_avg = (uifuseblk11_159_128 >> 1) & 0x7ff;
	/* profile mapping */
	for (i = 0; i < 8; i++) {
		if (uisvtdro_avg >= dro_map_table[i]) {
			mv_profile = i + 1;
			break;
		}
	}
#endif
	/* TODO: read profile by TZ API if TZ is enabled */

	pr_info("EDEN SoC profile number: %d\n", mv_profile);
}

int __init eden_init_dvfs(void)
{
	read_mv_profile();
	eden_dvfs_setup();
	pmic_init_setup();

	dvfs_init_rails(eden_dvfs_rails, ARRAY_SIZE(eden_dvfs_rails));
	eden_dvfs_d0_init();
	eden_dvfs_d0cg_init();

	return 0;
}
late_initcall(eden_init_dvfs);
