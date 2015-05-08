/*
 *  linux/arch/arm/mach-mmp/mmpx-dt.c
 *
 *  Copyright (C) 2012 Marvell Technology Group Ltd.
 *  Author: Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/clocksource.h>
#include <linux/clk/mmp.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/usb/mv_usb2_phy.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/regs-addr.h>
#include <linux/features.h>
#include <asm/smp_twd.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/irqs.h>
#include <mach/regs-apbc.h>
#include <mach/regs-ciu.h>
#include <mach/addr-map.h>
#include <mach/regs-coresight.h>
#include <media/soc_camera.h>
#include <media/mrvl-camera.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/memblock.h>
#include <linux/irqchip/arm-gic.h>
#ifdef CONFIG_GPU_RESERVE_MEM
#include <mach/gpu_mem.h>
#endif
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#include "common.h"
#include "reset.h"
#include <linux/regdump_ops.h>

#include <mach/pxa988_lowpower.h>

#define MHZ_TO_KHZ	1000

void buck2_sleepmode_control_for_wifi(int on);
void ldo10_sleepmode_control_for_wifi(int on);

static struct mv_usb_platform_data mmpx_usb_pdata = {
	.mode				= MV_USB_MODE_OTG,
	.extern_attr			= MV_USB_HAS_VBUS_DETECTION,
	.otg_force_a_bus_req		= 1,
	.disable_otg_clock_gating	= 1,
};

#ifdef CONFIG_REGDUMP
static struct regdump_ops pmua_regdump_ops = {
	.dev_name = "pxa1x88-common-pmua",
};

static struct regdump_region pmua_dump_region[] = {
	{"PMUA_CC_CP",			0x000, 4, regdump_cond_true},
	{"PMUA_CC_AP",			0x004, 4, regdump_cond_true},
	{"PMUA_DM_CC_CP",		0x008, 4, regdump_cond_true},
	{"PMUA_DM_CC_AP",		0x00c, 4, regdump_cond_true},
	{"PMUA_FC_TIMER",		0x010, 4, regdump_cond_true},
	{"PMUA_CP_IDLE_CFG",		0x014, 4, regdump_cond_true},
	{"PMUA_AP_IDLE_CFG",		0x018, 4, regdump_cond_true},
	{"PMUA_SQU_CLK_GATE_CTRL",		0x01c, 4, regdump_cond_true},
	{"PMUA_CCIC_CLK_GATE_CTRL",	0x028, 4, regdump_cond_true},
	{"PMUA_FBRC0_CLK_GATE_CTRL",	0x02c, 4, regdump_cond_true},
	{"PMUA_FBRC1_CLK_GATE_CTRL",	0x030, 4, regdump_cond_true},
	{"PMUA_USB_CLK_GATE_CTRL",	0x034, 4, regdump_cond_true},
	{"PMUA_ISP_CLK_RES_CTRL",	0x038, 4, regdump_cond_true},
	{"PMUA_PMU_CLK_GATE_CTRL",	0x040, 4, regdump_cond_true},
	{"PMUA_DSI_CLK_RES_CTRL",	0x044, 4, regdump_cond_true},
	{"PMUA_LCD_DSI_CLK_RES_CTRL",	0x04c, 4, regdump_cond_true},
	{"PMUA_CCIC_CLK_RES_CTRL",	0x050, 4, regdump_cond_true},
	{"PMUA_SDH0_CLK_RES_CTRL",	0x054, 4, regdump_cond_true},
	{"PMUA_SDH1_CLK_RES_CTRL",	0x058, 4, regdump_cond_true},
	{"PMUA_USB_CLK_RES_CTRL",	0x05c, 4, regdump_cond_true},
	{"PMUA_NF_CLK_RES_CTRL",	0x060, 4, regdump_cond_true},
	{"PMUA_DMA_CLK_RES_CTRL",	0x064, 4, regdump_cond_true},
	{"PMUA_AES_CLK_RES_CTRL",	0x068, 4, regdump_cond_true},
	{"PMUA_MCB_CLK_RES_CTRL",	0x06c, 4, regdump_cond_true},
	{"PMUA_CP_IMR",			0x070, 4, regdump_cond_true},
	{"PMUA_CP_IRWC",			0x074, 4, regdump_cond_true},
	{"PMUA_CP_ISR",			0x078, 4, regdump_cond_true},
	{"PMUA_SD_ROT_WAKE_CLR",		0x07c, 4, regdump_cond_true},
	{"PMUA_PWR_STBL_TIMER",		0x084, 4, regdump_cond_true},
	{"PMUA_DEBUG_REG",		0x088, 4, regdump_cond_true},
	{"PMUA_SRAM_PWR_DWN",		0x08c, 4, regdump_cond_true},
	{"PMUA_CORE_STATUS",		0x090, 4, regdump_cond_true},
	{"PMUA_RES_FRM_SLP_CLR",	0x094, 4, regdump_cond_true},
	{"PMUA_AP_IMR",			0x098, 4, regdump_cond_true},
	{"PMUA_AP_IRWC",		0x09c, 4, regdump_cond_true},
	{"PMUA_AP_ISR",			0x0a0, 4, regdump_cond_true},
	{"PMUA_VPU_CLK_RES_CTRL",	0x0a4, 4, regdump_cond_true},
	{"PMUA_DTC_CLK_RES_CTRL",	0x0ac, 4, regdump_cond_true},
	{"PMUA_MC_HW_SLP_TYPE",		0x0b0, 4, regdump_cond_true},
	{"PMUA_MC_SLP_REQ_AP",		0x0b4, 4, regdump_cond_true},
	{"PMUA_MC_SLP_REQ_CP",		0x0b8, 4, regdump_cond_true},
	{"PMUA_MC_SLP_REQ_MSA",		0x0bc, 4, regdump_cond_true},
	{"PMUA_MC_SW_SLP_TYPE",		0x0c0, 4, regdump_cond_true},
	{"PMUA_PLL_SEL_STATUS",		0x0c4, 4, regdump_cond_true},
	{"PMUA_SYNC_MODE_BYPASS",	0x0c8, 4, regdump_cond_true},
	{"PMUA_GPU_3D_CLK_RES_CTRL",	0x0cc, 4, regdump_cond_true},
	{"PMUA_SMC_CLK_RES_CTRL",	0x0d4, 4, regdump_cond_true},
	{"PMUA_PWR_CTRL_REG",	0x0d8, 4, regdump_cond_true},
	{"PMUA_PWR_BLK_TMR_REG",		0x0dc, 4, regdump_cond_true},
	{"PMUA_SDH2_CLK_RES_CTRL",	0x0e0, 4, regdump_cond_true},
	{"PMUA_CA7MP_IDLE_CFG1",		0x0e4, 4, regdump_cond_true},
	{"PMUA_MC_CTRL",	0x0e8, 4, regdump_cond_true},
	{"PMUA_PWR_STATUS_REG",	0x0f0, 4, regdump_cond_true},
	{"PMUA_GPU_2D_CLK_RES_CTRL",	0x0f4, 4, regdump_cond_true},
	{"PMUA_CC2_AP",	0x100, 4, regdump_cond_true},
	{"PMUA_DM_CC2_AP",	0x104, 4, regdump_cond_true},
	{"PMUA_TRACE_CONFIG",	0x108, 4, regdump_cond_true},
	{"PMUA_CA7MP_IDLE_CFG0",		0x120, 4, regdump_cond_true},
	{"PMUA_CA7_CORE0_IDLE_CFG",		0x124, 4, regdump_cond_true},
	{"PMUA_CA7_CORE1_IDLE_CFG",		0x128, 4, regdump_cond_true},
	{"PMUA_CA7_CORE0_WAKEUP",		0x12c, 4, regdump_cond_true},
	{"PMUA_CA7_CORE1_WAKEUP",		0x130, 4, regdump_cond_true},
	{"PMUA_CA7_CORE2_WAKEUP",		0x134, 4, regdump_cond_true},
	{"PMUA_CA7_CORE3_WAKEUP",		0x138, 4, regdump_cond_true},
	{"PMUA_DVC_DEBUG",		0x140, 4, regdump_cond_true},
	{"PMUA_CA7MP_IDLE_CFG2",		0x150, 4, regdump_cond_true},
	{"PMUA_CA7MP_IDLE_CFG3",		0x154, 4, regdump_cond_true},
	{"PMUA_CA7_CORE2_IDLE_CFG",		0x160, 4, regdump_cond_true},
	{"PMUA_CA7_CORE3_IDLE_CFG",		0x164, 4, regdump_cond_true},
	{"PMUA_CA7_PWR_MISC",		0x170, 4, regdump_cond_true},
};

static struct regdump_ops pmua_regdump_ops_1088 = {
	.dev_name = "pxa1088-pmua",
};

static struct regdump_region pmua_dump_region_1088[] = {
	{"PMUA_IRE_CLK_GATE_CTRL",		0x020, 4, regdump_cond_true},
	{"PMUA_HSI_CLK_RES_CTRL",	0x048, 4, regdump_cond_true},
	{"PMUA_FBRC_CLK",		0x080, 4, regdump_cond_true},
	{"PMUA_VPRO_PWRDWN",	0x0a8, 4, regdump_cond_true},
	{"PMUA_GPU_3D_PWRDWN",	0x0d0, 4, regdump_cond_true},
};

static struct regdump_ops pmua_regdump_ops_1L88 = {
	.dev_name = "pxa1L88-pmua",
};

static struct regdump_region pmua_dump_region_1L88[] = {
	{"PMUA_CCIC2_CLK_RES_CTRL",		0x024, 4, regdump_cond_true},
	{"PMUA_LTEDMA_CLK_RES_CTRL",	0x048, 4, regdump_cond_true},
	{"DFC_AP",		0x180, 4, regdump_cond_true},
	{"DFC_CP",		0x184, 4, regdump_cond_true},
	{"DFC_STATUS",		0x188, 4, regdump_cond_true},
	{"DFC_LEVEL0",		0x190, 4, regdump_cond_true},
	{"DFC_LEVEL1",		0x194, 4, regdump_cond_true},
	{"DFC_LEVEL2",		0x198, 4, regdump_cond_true},
	{"DFC_LEVEL3",		0x19c, 4, regdump_cond_true},
	{"DFC_LEVEL4",		0x1a0, 4, regdump_cond_true},
	{"DFC_LEVEL5",		0x1a4, 4, regdump_cond_true},
	{"DFC_LEVEL6",		0x1a8, 4, regdump_cond_true},
	{"DFC_LEVEL7",		0x1ac, 4, regdump_cond_true},
};

static void __init pxa_init_pmua_regdump(void)
{
	pmua_regdump_ops.base = get_apmu_base_va();
	pmua_regdump_ops.phy_base = get_apmu_base_pa();
	pmua_regdump_ops.regions = pmua_dump_region;
	pmua_regdump_ops.reg_nums = ARRAY_SIZE(pmua_dump_region);
	register_regdump_ops(&pmua_regdump_ops);
}

static void __init pxa_init_pmua_regdump_1x88(void)
{
	if (cpu_is_pxa1088()) {
		pmua_regdump_ops_1088.base = get_apmu_base_va();
		pmua_regdump_ops_1088.phy_base = get_apmu_base_pa();
		pmua_regdump_ops_1088.regions = pmua_dump_region_1088;
		pmua_regdump_ops_1088.reg_nums =
			ARRAY_SIZE(pmua_dump_region_1088);
		register_regdump_ops(&pmua_regdump_ops_1088);
	}
	if (cpu_is_pxa1L88()) {
		pmua_regdump_ops_1L88.base = get_apmu_base_va();
		pmua_regdump_ops_1L88.phy_base = get_apmu_base_pa();
		pmua_regdump_ops_1L88.regions = pmua_dump_region_1L88;
		pmua_regdump_ops_1L88.reg_nums =
			ARRAY_SIZE(pmua_dump_region_1L88);
		register_regdump_ops(&pmua_regdump_ops_1L88);
	}
}

static struct regdump_ops gic_regdump_ops = {
	.dev_name = "pxa1x88-gic",
};

static struct regdump_region gic_dump_region[] = {
	{"GIC_GICD_CTLR",			0x000, 4, regdump_cond_true},
	{"GIC_GICD_TYPER",			0x004, 4, regdump_cond_true},
	{"GIC_GICD_IIDR",			0x008, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER0",			0x100, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER1",			0x104, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER2",			0x108, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER3",			0x10c, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER4",			0x110, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER5",			0x114, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER6",			0x118, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER7",			0x11c, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER8",			0x120, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER9",			0x124, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER10",		0x128, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER11",		0x12c, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER12",		0x130, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER13",		0x134, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER14",		0x138, 4, regdump_cond_true},
	{"GIC_GICD_ISENABLER15",		0x13c, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR0",			0x200, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR1",			0x204, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR2",			0x208, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR3",			0x20c, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR4",			0x210, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR5",			0x214, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR6",			0x218, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR7",			0x21c, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR8",			0x220, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR9",			0x224, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR10",			0x228, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR11",			0x22c, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR12",			0x230, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR13",			0x234, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR14",			0x238, 4, regdump_cond_true},
	{"GIC_GICD_ISPENDR15",			0x23c, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER0",			0x300, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER1",			0x304, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER2",			0x308, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER3",			0x30c, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER4",			0x310, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER5",			0x314, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER6",			0x318, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER7",			0x31c, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER8",			0x320, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER9",			0x324, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER10",		0x328, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER11",		0x32c, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER12",		0x330, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER13",		0x334, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER14",		0x338, 4, regdump_cond_true},
	{"GIC_GICD_ISACTIVER15",		0x33c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR0",			0xc00, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR1",			0xc04, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR2",			0xc08, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR3",			0xc0c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR4",			0xc10, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR5",			0xc14, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR6",			0xc18, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR7",			0xc1c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR8",			0xc20, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR9",			0xc24, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR10",			0xc28, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR11",			0xc2c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR12",			0xc30, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR13",			0xc34, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR14",			0xc38, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR15",			0xc3c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR16",			0xc40, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR17",			0xc44, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR18",			0xc48, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR19",			0xc4c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR20",			0xc50, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR21",			0xc54, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR22",			0xc58, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR23",			0xc5c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR24",			0xc60, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR25",			0xc64, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR26",			0xc68, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR27",			0xc6c, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR28",			0xc70, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR29",			0xc74, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR30",			0xc78, 4, regdump_cond_true},
	{"GIC_GICD_ICFGR31",			0xc7c, 4, regdump_cond_true},
};

static void __init pxa_init_gic_regdump(void)
{
	gic_regdump_ops.base = gic_get_dist_base();
	gic_regdump_ops.phy_base = gic_dist_base_phys();
	gic_regdump_ops.regions = gic_dump_region;
	gic_regdump_ops.reg_nums = ARRAY_SIZE(gic_dump_region);
	register_regdump_ops(&gic_regdump_ops);
}

#else
static inline void  __init pxa_init_pmua_regdump(void) {}
static inline void __init pxa_init_gic_regdump(void) {}
#endif

#ifdef CONFIG_SD8XXX_RFKILL
/*
 * PXA1L88 DKB V20 and PXA1L88 TABLET DKB V10
 * use external ldo SGM2035C for WIFI card 3.3v power supply
 * and this ldo controlled by gpio36.
 */
static void wireless_card_set_power(unsigned int on)
{
	static int enabled;
	if (on && !enabled) {
		enabled = 1;
	}
	if (!on && enabled) {
		enabled = 0;
	}
	
	buck2_sleepmode_control_for_wifi(on);
	ldo10_sleepmode_control_for_wifi(on);
	return;
}

struct sd8x_rfkill_platform_data sd8x_rfkill_platdata = {
	.set_power	= wireless_card_set_power,
	};

#endif

#define CCIC1_PWDN_GPIO 80
#define CCIC1_RESET_GPIO_1L88 67
#define CCIC1_RESET_GPIO_1088 81
#define CCIC2_PWDN_GPIO 68
#define CCIC2_RESET_GPIO 69
#define GPIO_TORCH_EN 12 /* For pxa1L88 */
#define GPIO_FLASH_EN 18 /* For pxa1L88 */
/* this is just define for more soc cameras */
static struct regulator *c1_avdd_2v8;
static struct regulator *c1_dovdd_1v8;
static struct regulator *c1_af_2v8;
static struct regulator *c1_dvdd_1v2;
#ifdef CONFIG_SOC_CAMERA_S5K8AA
static int s5k8aa_sensor_power(struct device *dev, int on)
{
	int cam_enable = CCIC1_PWDN_GPIO;
	int cam_reset = CCIC1_RESET_GPIO_1088;
	int ret;

	/* 8aa as front sensor on 1L88 */
	if (cpu_is_pxa1L88()) {
		cam_enable = CCIC2_PWDN_GPIO;
		cam_reset = CCIC2_RESET_GPIO;
	}

	/* Get the regulators and never put it */
	/*
	 * The regulators is for sensor and should be in sensor driver
	 * As SoC camera does not support device tree, workaround here
	 */

	if (!c1_avdd_2v8) {
		c1_avdd_2v8 = regulator_get(dev, "avdd_2v8");
		if (IS_ERR(c1_avdd_2v8)) {
			dev_warn(dev, "Failed to get regulator avdd_2v8\n");
			c1_avdd_2v8 = NULL;
		}
	}

	if (cpu_is_pxa1L88()) {
		if (!c1_dovdd_1v8) {
			c1_dovdd_1v8 = regulator_get(dev, "dovdd_1v8");
			if (IS_ERR(c1_dovdd_1v8)) {
				dev_warn(dev, "Failed to get regulator dovdd_1v8\n");
				c1_dovdd_1v8 = NULL;
			}
		}
	}

	if (!c1_af_2v8) {
		c1_af_2v8 = regulator_get(dev, "af_2v8");
		if (IS_ERR(c1_af_2v8)) {
			dev_warn(dev, "Failed to get regulator af_2v8\n");
			c1_af_2v8 = NULL;
		}
	}

	if (cpu_is_pxa1L88()) {
		if (!c1_dvdd_1v2) {
			c1_dvdd_1v2 = regulator_get(dev, "dvdd_1v2");
			if (IS_ERR(c1_dvdd_1v2)) {
				dev_warn(dev, "Failed to get regulator dvdd_1v2\n");
				c1_dvdd_1v2 = NULL;
			}
		}
	}

	if (gpio_request(cam_enable, "CAM2_POWER")) {
		dev_err(dev, "Request GPIO failed, gpio: %d\n", cam_enable);
		goto cam_enable_failed;
	}
	if (gpio_request(cam_reset, "CAM2_RESET")) {
		dev_err(dev, "Request GPIO failed, gpio: %d\n", cam_reset);
		goto cam_reset_failed;
	}

	if (on) {
		if (cpu_is_pxa1L88()) {
			if (c1_dovdd_1v8) {
				regulator_set_voltage(c1_dovdd_1v8,
						1800000, 1800000);
				ret = regulator_enable(c1_dovdd_1v8);
				if (ret)
					goto cam_regulator_enable_failed;
			}
		}

		if (c1_avdd_2v8) {
			regulator_set_voltage(c1_avdd_2v8, 2800000, 2800000);
			ret = regulator_enable(c1_avdd_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (c1_af_2v8) {
			regulator_set_voltage(c1_af_2v8, 2800000, 2800000);
			ret = regulator_enable(c1_af_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (cpu_is_pxa1L88()) {
			if (c1_dvdd_1v2) {
				regulator_set_voltage(c1_dvdd_1v2,
						1200000, 1200000);
				ret = regulator_enable(c1_dvdd_1v2);
				if (ret)
					goto cam_regulator_enable_failed;
			}
		}

		gpio_direction_output(cam_reset, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_enable, 1);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 1);
		usleep_range(5000, 20000);
	} else {
		gpio_direction_output(cam_enable, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 0);

		if (c1_avdd_2v8)
			regulator_disable(c1_avdd_2v8);
		if (cpu_is_pxa1L88())
			if (c1_dovdd_1v8)
				regulator_disable(c1_dovdd_1v8);
		if (c1_af_2v8)
			regulator_disable(c1_af_2v8);
		if (cpu_is_pxa1L88())
			if (c1_dvdd_1v2)
				regulator_disable(c1_dvdd_1v2);
	}

	gpio_free(cam_enable);
	gpio_free(cam_reset);

	return 0;

cam_reset_failed:
	gpio_free(cam_enable);
cam_enable_failed:
	ret = -EIO;
cam_regulator_enable_failed:
	if (cpu_is_pxa1L88()) {
		if (c1_dvdd_1v2)
			regulator_put(c1_dvdd_1v2);
		c1_dvdd_1v2 = NULL;
	}
	if (c1_af_2v8)
		regulator_put(c1_af_2v8);
	c1_af_2v8 = NULL;
	if (cpu_is_pxa1L88()) {
		if (c1_dovdd_1v8)
			regulator_put(c1_dovdd_1v8);
		c1_dovdd_1v8 = NULL;
	}
	if (c1_avdd_2v8)
		regulator_put(c1_avdd_2v8);
	c1_avdd_2v8 = NULL;

	return ret;
}

static struct sensor_board_data s5k8aa_data = {
	.mount_pos	= SENSOR_USED | SENSOR_POS_FRONT | SENSOR_RES_LOW,
	.bus_type	= V4L2_MBUS_CSI2,
	.bus_flag	= V4L2_MBUS_CSI2_1_LANE,
	.dphy = {0xFF1D00, 0x00024733, 0x04001001},
};

static struct i2c_board_info dkb_i2c_s5k8aay = {
		I2C_BOARD_INFO("s5k8aay", 0x3c),
};

static struct soc_camera_desc s5k8aa_desc = {
	.subdev_desc = {
		.power          = s5k8aa_sensor_power,
		.drv_priv		= &s5k8aa_data,
		.flags		= 0,
	},
	.host_desc = {
		.bus_id = 0,	/* Default as ccic0 */
		.i2c_adapter_id = 0,
		.board_info     = &dkb_i2c_s5k8aay,
		.module_name    = "s5k8aay",
	},
};
#endif

#ifdef CONFIG_SOC_CAMERA_OV5640_ECS
static int soc_sensor_flash_led_set(void *control, bool op)
{
	int flash_on = GPIO_FLASH_EN;
	int torch_on = GPIO_TORCH_EN;
	struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *) control;

	if (gpio_request(flash_on, "flash_power_on")) {
		printk(KERN_ERR  "Request GPIO failed, gpio:%d\n", flash_on);
		return -EIO;
	}
	if (gpio_request(torch_on, "torch_power_on")) {
		printk(KERN_ERR  "Request GPIO failed, gpio:%d\n", torch_on);
		return -EIO;
	}

	switch (ctrl->id) {
		case V4L2_CID_FLASH_STROBE:
			gpio_direction_output(flash_on, 1);
			gpio_direction_output(torch_on, 1);
			break;
		case V4L2_CID_FLASH_TORCH_INTENSITY:
			gpio_direction_output(torch_on, 1);
			gpio_direction_output(flash_on, 0);
			break;
		case V4L2_CID_FLASH_STROBE_STOP:
			gpio_direction_output(flash_on, 0);
			gpio_direction_output(torch_on, 0);
			break;
		default:
			break;
	}

	gpio_free(flash_on);
	gpio_free(torch_on);

	return 0;
}

static int ov5640_sensor_power(struct device *dev, int on)
{
	int cam_enable = CCIC1_PWDN_GPIO;
	int cam_reset = CCIC1_RESET_GPIO_1088;
	int ret = 0;

	if (cpu_is_pxa1L88()) {
		cam_reset = CCIC1_RESET_GPIO_1L88;
	}

	if (!c1_avdd_2v8) {
		c1_avdd_2v8 = regulator_get(dev, "avdd_2v8");
		if (IS_ERR(c1_avdd_2v8)) {
			dev_warn(dev, "Failed to get regulator avdd_2v8\n");
			c1_avdd_2v8 = NULL;
		}
	}

	if (cpu_is_pxa1L88()) {
		if (!c1_dovdd_1v8) {
			c1_dovdd_1v8 = regulator_get(dev, "dovdd_1v8");
			if (IS_ERR(c1_dovdd_1v8)) {
				dev_warn(dev, "Failed to get regulator dovdd_1v8\n");
				c1_dovdd_1v8 = NULL;
			}
		}
	}

	if (!c1_af_2v8) {
		c1_af_2v8 = regulator_get(dev, "af_2v8");
		if (IS_ERR(c1_af_2v8)) {
			dev_warn(dev, "Failed to get regulator af_2v8\n");
			c1_af_2v8 = NULL;
		}
	}

	if (cpu_is_pxa1L88()) {
		if (!c1_dvdd_1v2) {
			c1_dvdd_1v2 = regulator_get(dev, "dvdd_1v2");
			if (IS_ERR(c1_dvdd_1v2)) {
				dev_warn(dev, "Failed to get regulator dvdd_1v2\n");
				c1_dvdd_1v2 = NULL;
			}
		}
	}

	if (gpio_request(cam_enable, "CAM2_POWER")) {
		dev_err(dev, "Request GPIO failed, gpio: %d\n", cam_enable);
		goto cam_enable_failed;
	}
	if (gpio_request(cam_reset, "CAM2_RESET")) {
		dev_err(dev, "Request GPIO failed, gpio: %d\n", cam_reset);
		goto cam_reset_failed;
	}

	if (on) {
		if (cpu_is_pxa1L88()) {
			if (c1_dvdd_1v2) {
				regulator_set_voltage(c1_dvdd_1v2,
						1200000, 1200000);
				ret = regulator_enable(c1_dvdd_1v2);
				if (ret)
					goto cam_regulator_enable_failed;
			}
		}

		if (cpu_is_pxa1L88()) {
			if (c1_dovdd_1v8) {
				regulator_set_voltage(c1_dovdd_1v8,
						1800000, 1800000);
				ret = regulator_enable(c1_dovdd_1v8);
				if (ret)
					goto cam_regulator_enable_failed;
			}
		}

		if (c1_avdd_2v8) {
			regulator_set_voltage(c1_avdd_2v8, 2800000, 2800000);
			ret = regulator_enable(c1_avdd_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (c1_af_2v8) {
			regulator_set_voltage(c1_af_2v8, 2800000, 2800000);
			ret = regulator_enable(c1_af_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		usleep_range(5000, 20000);
		gpio_direction_output(cam_enable, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 1);
		usleep_range(5000, 20000);
	} else {
		gpio_direction_output(cam_reset, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 1);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_enable, 1);

		if (c1_avdd_2v8)
			regulator_disable(c1_avdd_2v8);
		if (cpu_is_pxa1L88())
			if (c1_dovdd_1v8)
				regulator_disable(c1_dovdd_1v8);
		if (c1_af_2v8)
			regulator_disable(c1_af_2v8);
		if (cpu_is_pxa1L88())
			if (c1_dvdd_1v2)
				regulator_disable(c1_dvdd_1v2);
	}

	gpio_free(cam_enable);
	gpio_free(cam_reset);

	return 0;

cam_reset_failed:
	gpio_free(cam_enable);
cam_enable_failed:
	ret = -EIO;
cam_regulator_enable_failed:
	if (cpu_is_pxa1L88()) {
		if (c1_dvdd_1v2)
			regulator_put(c1_dvdd_1v2);
		c1_dvdd_1v2 = NULL;
	}
	if (c1_af_2v8)
		regulator_put(c1_af_2v8);
	c1_af_2v8 = NULL;
	if (cpu_is_pxa1L88()) {
		if (c1_dovdd_1v8)
			regulator_put(c1_dovdd_1v8);
		c1_dovdd_1v8 = NULL;
	}
	if (c1_avdd_2v8)
		regulator_put(c1_avdd_2v8);
	c1_avdd_2v8 = NULL;

	return ret;
}

static struct sensor_board_data ov5640_data = {
	.mount_pos	= SENSOR_USED | SENSOR_POS_BACK | SENSOR_RES_HIGH,
	.bus_type	= V4L2_MBUS_CSI2,
	.bus_flag	= V4L2_MBUS_CSI2_2_LANE, /* ov5640 used 2 lanes */
	.dphy		= {0x0D06, 0x33, 0x0900},
	.v4l2_flash_if = soc_sensor_flash_led_set,
};

static struct i2c_board_info dkb_i2c_ov5640 = {
		I2C_BOARD_INFO("ov5640", 0x3c),
};

static struct soc_camera_desc ov5640_desc = {
	.subdev_desc = {
		.power          = ov5640_sensor_power,
		.drv_priv		= &ov5640_data,
		.flags		= 0,
	},
	.host_desc = {
		.bus_id = 0,	/* Must match with the camera ID */
		.i2c_adapter_id = 0,
		.board_info     = &dkb_i2c_ov5640,
		.module_name    = "ov5640",
	},
};
#endif

#ifdef CONFIG_VPU_DEVFREQ
static struct devfreq_platform_data devfreq_vpu_pdata = {
	.clk_name = "VPUCLK",
};
static void __init pxa988_devfreq_vpu_init(void)
{
	unsigned int vpu_freq_num;
	unsigned int *vpu_freq_table;
	unsigned int i;

	struct clk *clk = clk_get_sys(NULL, "VPUCLK");
	if (IS_ERR(clk)) {
		WARN_ON(1);
		return;
	}
	vpu_freq_num =  __clk_periph_get_opnum(clk);

	vpu_freq_table = kzalloc(sizeof(unsigned int) * vpu_freq_num,
				 GFP_KERNEL);
	if (!vpu_freq_table)
		return;
	for (i = 0; i < vpu_freq_num; i++)
		vpu_freq_table[i] = __clk_periph_get_oprate(clk, i) / MHZ_TO_KHZ;
	devfreq_vpu_pdata.freq_tbl_len = vpu_freq_num;
	devfreq_vpu_pdata.freq_table = vpu_freq_table;
}

#endif

/* PXA988 */
static const struct of_dev_auxdata pxa988_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4017000, "pxa2xx-uart.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4018000, "pxa2xx-uart.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4036000, "pxa2xx-uart.2", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4011000, "pxa2xx-i2c.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4010800, "pxa2xx-i2c.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4037000, "pxa2xx-i2c.2", NULL),
	OF_DEV_AUXDATA("marvell,mmp-gpio", 0xd4019000, "mmp-gpio", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-edge-wakeup", 0xd4019800, "mmp-edge-wakeup", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-rtc", 0xd4010000, "sa1100-rtc", NULL),
	OF_DEV_AUXDATA("marvell,pdma-1.0", 0xd4000000, "mmp-pdma", NULL),
	OF_DEV_AUXDATA("marvell,pxa27x-keypad", 0xd4012000, "pxa27x-keypad", NULL),
	OF_DEV_AUXDATA("marvell,usb2-phy-40lp", 0xd4207000,
			"pxa988-usb-phy", NULL),
	OF_DEV_AUXDATA("marvell,mv-udc", 0xd4208100, "mv-udc", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("marvell,pxa-u2oehci", 0xd4208100, "pxa-u2oehci", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("marvell,mv-otg", 0xd4208100, "mv-otg", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xD128dc00, "mmp-sspa-dai.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xD128dd00, "mmp-sspa-dai.1", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xD128D800, "mmp-adma.0", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xD128D900, "mmp-adma.1", NULL),
#ifdef CONFIG_DDR_DEVFREQ
	OF_DEV_AUXDATA("marvell,devfreq-ddr", 0xc0100000, "devfreq-ddr", NULL),
#endif
#ifdef CONFIG_VPU_DEVFREQ
	OF_DEV_AUXDATA("marvell,devfreq-vpu", 0xf0400000,
			"devfreq-vpu.0", (void *)&devfreq_vpu_pdata),
#endif
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280000, "sdhci-pxav3.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280800, "sdhci-pxav3.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4281000, "sdhci-pxav3.2", NULL),
#ifdef CONFIG_SOC_CAMERA_S5K8AA
	OF_DEV_AUXDATA("soc-camera-pdrv", 0, "soc-camera-pdrv.0", &s5k8aa_desc),
#endif
#ifdef CONFIG_SOC_CAMERA_OV5640_ECS
	OF_DEV_AUXDATA("soc-camera-pdrv", 1, "soc-camera-pdrv.1", &ov5640_desc),
#endif
	OF_DEV_AUXDATA("marvell,mmp-ccic", 0xd420a000, "mmp-camera.0", NULL),
	OF_DEV_AUXDATA("marvell,mmp-ccic", 0xd420a800, "mmp-camera.1", NULL),
	OF_DEV_AUXDATA("marvell,mmp-disp", 0xd420b000, "mmp-disp", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb", 0, "mmp-fb", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 0, "mmp-fb-overlay.0", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 1, "mmp-fb-overlay.1", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 2, "mmp-fb-overlay.2", NULL),
	OF_DEV_AUXDATA("marvell,mmp-nt35565", 0, "mmp-nt35565", NULL),
	OF_DEV_AUXDATA("marvell,mmp-lg4591", 0, "mmp-lg4591", NULL),
	OF_DEV_AUXDATA("marvell,mmp-r63311", 0, "mmp-r63311", NULL),
	OF_DEV_AUXDATA("marvell,mmp-dsi", 0xd420b800, "mmp-dsi", NULL),
	OF_DEV_AUXDATA("marvell,pxa910-squ", 0xd42a0800, "pxa910-squ", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd401b000, "pxa988-ssp.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd42a0c00, "pxa988-ssp.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd4039000, "pxa988-ssp.4", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-ssp-dai", 1, "pxa-ssp-dai.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-ssp-dai", 4, "pxa-ssp-dai.2", NULL),
	OF_DEV_AUXDATA("marvell,pxa-88pm805-snd-card", 0, "sound", NULL),
	OF_DEV_AUXDATA("marvell,pxa28nm-thermal", 0xd4013300,
			"pxa28nm-thermal", NULL),
	OF_DEV_AUXDATA("pxa-ion", 0, "pxa-ion", NULL),
#ifdef CONFIG_PXA_THERMAL
	OF_DEV_AUXDATA("marvell,pxa-thermal", 0xd4013200, "pxa-thermal", NULL),
#endif
#ifdef CONFIG_PXA1088_THERMAL
	OF_DEV_AUXDATA("marvell,pxa1088-thermal", 0xd4013200,
			"pxa1088-thermal", NULL),
#endif
#ifdef CONFIG_SD8XXX_RFKILL
	OF_DEV_AUXDATA("mrvl,sd8x-rfkill", 0, "sd8x-rfkill",
		       &sd8x_rfkill_platdata),
#endif
	OF_DEV_AUXDATA("marvell,mmp-gps", 0, "mmp-gps", NULL),
	{}
};

static void __init pxa988_dt_irq_init(void)
{
	irqchip_init();
	/* only for wake up */
	mmp_of_wakeup_init();
}

#define APMU_SDH0      0x54
static void __init pxa988_sdhc_reset_all(void)
{
	unsigned int reg_tmp;

	/* use bit0 to reset all 3 sdh controls */
	reg_tmp = __raw_readl(get_apmu_base_va() + APMU_SDH0);
	__raw_writel(reg_tmp & (~1), get_apmu_base_va() + APMU_SDH0);
	udelay(10);
	__raw_writel(reg_tmp | (1), get_apmu_base_va() + APMU_SDH0);
}

static void __init pxa988_dt_init_machine(void)
{
	if (of_machine_is_compatible("mrvl,pxa988")) {
		l2x0_of_init(0x30800000, 0xFE7FFFFF);
		l2x0_save_phys_reg_addr(&l2x0_regs_phys);
	}

	pxa988_clk_init();

#ifdef CONFIG_VPU_DEVFREQ
	pxa988_devfreq_vpu_init();
#endif

	pxa988_sdhc_reset_all();

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_add_gpu();
#endif

	if (of_machine_is_compatible("mrvl,pxa988-dkb")
	 || of_machine_is_compatible("mrvl,pxa1088-dkb")
	 || of_machine_is_compatible("mrvl,pxa1L88-dkb-v10")
	 || of_machine_is_compatible("mrvl,pxa1L88-dkb-v20")
	 || of_machine_is_compatible("sec,agera-v10")) {
		pr_info("UDC only is enabled\n");
		mmpx_usb_pdata.mode = MV_USB_MODE_UDC;
		mmpx_usb_pdata.otg_force_a_bus_req = 0;
		mmpx_usb_pdata.disable_otg_clock_gating = 0;
	}

	of_platform_populate(NULL, of_default_bus_match_table,
			     pxa988_auxdata_lookup, &platform_bus);

	/* s5k8aa as front sensot on pxa1L88 DKB, use ccic1 */
	if (cpu_is_pxa1L88()) {
		s5k8aa_desc.host_desc.bus_id = 1;
	}
	if (cpu_is_pxa1088() || cpu_is_pxa1L88()) {
		pxa_init_gic_regdump();
		pxa_init_pmua_regdump();
		pxa_init_pmua_regdump_1x88();
	}
}

static void __init pxa_enable_external_agent(void __iomem *addr)
{
	u32 tmp;

	tmp = readl_relaxed(addr);
	tmp |= 0x100000;
	writel_relaxed(tmp, addr);
}

static int __init pxa_external_agent_init(void)
{
	/* if enable TrustZone, move core config to TZSW. */
#ifndef CONFIG_TZ_HYPERVISOR
	if (has_feat_enable_cti()) {
		/* enable access CTI registers for core */
		pxa_enable_external_agent(CIU_CPU_CORE0_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE1_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE2_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE3_CONF);
	}
#endif

	return 0;
}
core_initcall(pxa_external_agent_init);

static void __init mmp_cti_enable(u32 cpu)
{
	void __iomem *cti_base = CTI_CORE0_VIRT_BASE + 0x1000 * cpu;
	u32 tmp;

	/* Unlock CTI */
	writel_relaxed(0xC5ACCE55, cti_base + CTI_LOCK_OFFSET);

	/*
	 * Enables a cross trigger event to the corresponding channel.
	 */
	tmp = readl_relaxed(cti_base + CTI_EN_IN1_OFFSET);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1 << cpu;
	writel_relaxed(tmp, cti_base + CTI_EN_IN1_OFFSET);

	tmp = readl_relaxed(cti_base + CTI_EN_OUT6_OFFSET);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1 << cpu;
	writel_relaxed(tmp, cti_base + CTI_EN_OUT6_OFFSET);

	/* Enable CTI */
	writel_relaxed(0x1, cti_base + CTI_CTRL_OFFSET);
}

static int __init mmp_cti_init(void)
{
	int cpu;
	if (!has_feat_enable_cti())
		return 1;

	for (cpu = 0; cpu < nr_cpu_ids; cpu++)
		mmp_cti_enable(cpu);
	return 0;
}
arch_initcall(mmp_cti_init);

void mmp_pmu_ack(void)
{
	writel_relaxed(0x40, CTI_REG(CTI_INTACK_OFFSET));
}
EXPORT_SYMBOL(mmp_pmu_ack);


#define MPMU_APRR		(0x1020)
#define MPMU_WDTPCR		(0x0200)
/* wdt and cp use the clock */
void enable_pxawdt_clock(void)
{
	void __iomem *mpmu_base;
	void __iomem *mpmu_wdtpcr;
	void __iomem *mpmu_aprr;
	mpmu_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	mpmu_aprr = mpmu_base + MPMU_APRR;
	mpmu_wdtpcr = mpmu_base + MPMU_WDTPCR;

	/* reset/enable WDT clock */
	writel(0x7, mpmu_wdtpcr);
	readl(mpmu_wdtpcr);
	writel(0x3, mpmu_wdtpcr);
	return;
}

#define GENERIC_COUNTER_VIRT_BASE       (APB_VIRT_BASE + 0x101000)
static __init void enable_arch_timer(void)
{
	uint32_t tmp;

	tmp = readl(APBC_COUNTER_CLK_SEL);

	/* Default is 26M/32768 = 0x319 */
	if ((tmp >> 16) != 0x319) {
		pr_warn("Generic Counter step of Low Freq is not right\n");
		return;
	}
	/* bit0 = 1: Generic Counter Frequency control by hardware VCTCXO_EN
	   VCTCXO_EN = 1, Generic Counter Frequency is 26Mhz;
	   VCTCXO_EN = 0, Generic Counter Frequency is 32KHz */
	writel(tmp | FREQ_HW_CTRL, APBC_COUNTER_CLK_SEL);

	/* NOTE: can not read CNTCR before write, otherwise write will fail
	   Halt on debug;
	   start the counter */
	writel(CNTCR_HDBG | CNTCR_EN, GENERIC_COUNTER_VIRT_BASE + CNTCR);
}

static __init void mmpx_timer_init(void)
{
	enable_pxawdt_clock();

#ifdef CONFIG_ARM_ARCH_TIMER
	enable_arch_timer();
#endif
	/* Select the configurable timer clock rate to be 3.25MHz */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMPX_TIMER0);
	__raw_writel(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3),
			APBC_MMPX_TIMER0);

	/* Select the configurable timer clock rate to be 3.25MHz */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMPX_TIMER1);
	__raw_writel(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3),
			APBC_MMPX_TIMER1);

	clocksource_of_init();
}

/* For HELANLTE CP memeory reservation, 32MB by default */
static u32 cp_area_size = 0x02000000;
static u32 cp_area_addr = 0x06000000;

static int __init early_cpmem(char *p)
{
	char *endp;

	cp_area_size = memparse(p, &endp);
	if (*endp == '@')
		cp_area_addr = memparse(endp + 1, NULL);

	return 0;
}
early_param("cpmem", early_cpmem);

static void pxa_reserve_cp_memblock(void)
{
	/* Reserve memory for CP */
	BUG_ON(memblock_reserve(cp_area_addr, cp_area_size) != 0);
	memblock_free(cp_area_addr, cp_area_size);
	memblock_remove(cp_area_addr, cp_area_size);
	pr_info("Reserved CP memory: 0x%x@0x%x\n", cp_area_size, cp_area_addr);
}

static void pxa_reserve_obmmem(void)
{
	/* Reserve 1MB memory for obm */
	BUG_ON(memblock_reserve(PLAT_PHYS_OFFSET, 0x100000) != 0);
	memblock_free(PLAT_PHYS_OFFSET, 0x100000);
	memblock_remove(PLAT_PHYS_OFFSET, 0x100000);
	pr_info("Reserved OBM memory: 0x%x@0x%lx\n", 0x100000, PLAT_PHYS_OFFSET);
}

static void __init pxa988_reserve(void)
{
	pxa_reserve_obmmem();

	pxa_reserve_cp_memblock();

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_reserve_gpu_memblock();
#endif
}

static const char *pxa1088_dt_board_compat[] __initdata = {
	"mrvl,pxa988-dkb",
	"mrvl,pxa1088-dkb",
	NULL,
};

static const char *pxa1L88_dt_board_compat[] __initdata = {
	"mrvl,pxa1L88-dkb-v10",
	"mrvl,pxa1L88-dkb-v20",
	"mrvl,pxa1L88-tablet-v10",
	NULL,
};

static const char *pxa1U88_dt_board_compat[] __initdata = {
	"mrvl,pxa1U88-dkb",
	NULL,
};

DT_MACHINE_START(PXA1088_DT, "PXA1088")
	.smp_init	= smp_init_ops(mmp_smp_init_ops),
	.map_io		= mmp_map_io,
	.init_irq	= pxa988_dt_irq_init,
	.init_time	= mmpx_timer_init,
	.init_machine	= pxa988_dt_init_machine,
	.dt_compat	= pxa1088_dt_board_compat,
	.reserve	= pxa988_reserve,
	.restart	= mmp_arch_restart,
MACHINE_END

DT_MACHINE_START(PXA1L88_DT, "PXA1L88")
	.smp_init	= smp_init_ops(mmp_smp_init_ops),
	.map_io		= mmp_map_io,
	.init_irq	= pxa988_dt_irq_init,
	.init_time	= mmpx_timer_init,
	.init_machine	= pxa988_dt_init_machine,
	.dt_compat	= pxa1L88_dt_board_compat,
	.reserve	= pxa988_reserve,
	.restart	= mmp_arch_restart,
MACHINE_END

DT_MACHINE_START(PXA1U88_DT, "PXA1U88")
	.smp_init	= smp_init_ops(mmp_smp_init_ops),
	.map_io		= mmp_map_io,
	.init_irq	= pxa988_dt_irq_init,
	.init_time	= mmpx_timer_init,
	.init_machine	= pxa988_dt_init_machine,
	.dt_compat	= pxa1U88_dt_board_compat,
	.reserve	= pxa988_reserve,
	.restart	= mmp_arch_restart,
MACHINE_END
