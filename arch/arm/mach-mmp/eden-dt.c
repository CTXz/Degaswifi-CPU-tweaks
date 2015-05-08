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
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/mm.h>
#include <linux/memblock.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/usb/phy.h>
#include <linux/usb/mv_usb2_phy.h>
#include <linux/delay.h>
#include <linux/pstore_ram.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/mmc/sdhci.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <dt-bindings/mmc/pxa_sdhci.h>
#include <asm/smp_twd.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/irqs.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-timers.h>
#include <media/soc_camera.h>
#include <media/mrvl-camera.h>
#include <linux/regulator/consumer.h>
#include <mach/regs-apmu.h>
#include <video/mmp_disp.h>
#ifdef CONFIG_GPU_RESERVE_MEM
#include <mach/gpu_mem.h>
#endif
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#include "common.h"
#define APMU_AP_DEBUG2			APMU_REG(0x390)
#define APMU_COREAPSS_CFG		APMU_REG(0x39C)
#define APMU_APPS_MEM_CTRL		APMU_REG(0x0194)
#define APMU_MC_MEM_CTRL		APMU_REG(0x0198)
#define APMU_ISLD_SP_CTRL		APMU_REG(0x019c)
#define APMU_ISLD_USB_CTRL		APMU_REG(0x01a0)
#define APMU_ISLD_LCD_CTRL		APMU_REG(0x01ac)
#define APMU_ISLD_VPU_CTRL		APMU_REG(0x01b0)
#define APMU_ISLD_GC_CTRL		APMU_REG(0x01b4)
#define APMU_ISLD_ISP_CTRL		APMU_REG(0x01A4)
#define APMU_ISLD_AU_CTRL		APMU_REG(0x01A8)
#define MPMU_WDTPCR			MPMU_REG(0x0200)

#ifdef CONFIG_ARM_ARCH_TIMER
#define MPMU_CPRR			MPMU_REG(0x0020)
#define MPMU_WDTPCR1			MPMU_REG(0x0204)
#define MPMU_APRR			MPMU_REG(0x1020)
#define REG_RTC_BR0			(APB_VIRT_BASE + 0x010014)
#define GENERIC_COUNTER_VIRT_BASE	(APB_VIRT_BASE + 0x80000)

#define MPMU_APRR_WDTR	(1<<4)
#define MPMU_APRR_CPR	(1<<0)
#define MPMU_CPRR_DSPR	(1<<2)
#define MPMU_CPRR_BBR	(1<<3)

/* Get SoC Access to Generic Timer */
inline void arch_timer_soc_access_enable(void)
{
	__raw_writel(0xbaba, GENERIC_COUNTER_VIRT_BASE + TMR_WFAR);
	__raw_writel(0xeb10, GENERIC_COUNTER_VIRT_BASE + TMR_WSAR);
}

void arch_timer_soc_config(void)
{
	u32 tmp;

	/* Enable WDTR2*/
	tmp  = __raw_readl(MPMU_CPRR);
	tmp = tmp | MPMU_APRR_WDTR;
	__raw_writel(tmp, MPMU_CPRR);

	/* Initialize Counter to zero */
	arch_timer_soc_access_enable();
	__raw_writel(0x0, GENERIC_COUNTER_VIRT_BASE + GEN_TMR_LD1);

	/* Program Generic Timer Clk Frequency */
	arch_timer_soc_access_enable();
	tmp = __raw_readl(GENERIC_COUNTER_VIRT_BASE + GEN_TMR_CFG);
	tmp |= (3 << 4); /* 3.25MHz/32KHz Counter auto switch enabled */
	arch_timer_soc_access_enable();
	__raw_writel(tmp, GENERIC_COUNTER_VIRT_BASE + GEN_TMR_CFG);

	/* Start the Generic Timer Counter */
	arch_timer_soc_access_enable();
	tmp = __raw_readl(GENERIC_COUNTER_VIRT_BASE + GEN_TMR_CFG);
	tmp |= 0x3;
	arch_timer_soc_access_enable();
	__raw_writel(tmp, GENERIC_COUNTER_VIRT_BASE + GEN_TMR_CFG);

	/* init RTC/WTC settings for clocks */
	tmp = __raw_readl(APMU_AP_DEBUG2);
	tmp |= 1<<8;
	__raw_writel(tmp, APMU_AP_DEBUG2);

	tmp = __raw_readl(APMU_COREAPSS_CFG);
	tmp &= ~(0xFFFFFF00);
	if (cpu_is_eden_z1())
		tmp |= (0x66909000);
	else
		tmp |= (0x50505000);
	__raw_writel(tmp, APMU_COREAPSS_CFG);
	tmp = __raw_readl(APMU_APPS_MEM_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_APPS_MEM_CTRL);
	tmp = __raw_readl(APMU_MC_MEM_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_MC_MEM_CTRL);
	tmp = __raw_readl(APMU_ISLD_SP_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_SP_CTRL);
	tmp = __raw_readl(APMU_ISLD_USB_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_USB_CTRL);
#ifdef CONFIG_VIDEO_MMP
	tmp = __raw_readl(APMU_ISLD_ISP_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_ISP_CTRL);
#endif
	tmp = __raw_readl(APMU_ISLD_AU_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_AU_CTRL);
	tmp = __raw_readl(APMU_ISLD_LCD_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_LCD_CTRL);
	tmp = __raw_readl(APMU_ISLD_VPU_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_VPU_CTRL);
	tmp = __raw_readl(APMU_ISLD_GC_CTRL);
	tmp &= ~(0xFFFF0000);
	tmp |= (0xA9AA0000);
	__raw_writel(tmp, APMU_ISLD_GC_CTRL);

	tmp = __raw_readl(APMU_AP_DEBUG2);
	tmp &= ~(1<<8);
	__raw_writel(tmp, APMU_AP_DEBUG2);
}

/* Using watchdog reset */
static void do_wdt_reset(const char *cmd)
{
	u32 reg, backup;
	int i;
	int match = 0, count = 0;
	void __iomem *watchdog_virt_base = GENERIC_COUNTER_VIRT_BASE;

	if (cmd && !strcmp(cmd, "recovery")) {
		for (i = 0, backup = 0; i < 4; i++) {
			backup <<= 8;
			backup |= *(cmd + i);
		}
		do {
			writel(backup, REG_RTC_BR0);
		} while (readl(REG_RTC_BR0) != backup);
	}

	/* reset/enable WDT clock */
	writel(0x1, MPMU_WDTPCR1);
	readl(MPMU_WDTPCR1);
	writel(0x0, MPMU_WDTPCR1);
	readl(MPMU_WDTPCR1);

	writel(0x3, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);

	/* enable WDT reset */
	arch_timer_soc_access_enable();
	writel(0x3, watchdog_virt_base + TMR_WMER);

	/* negate hardware reset to the WDT after system reset */
	reg = readl(MPMU_APRR) | MPMU_APRR_WDTR;
	writel(reg, MPMU_APRR);

	/* clear previous WDT status */
	arch_timer_soc_access_enable();
	writel(0, watchdog_virt_base + TMR_WSR);

	match = readl(watchdog_virt_base + TMR_WMR);
	count = readl(watchdog_virt_base + TMR_WVR);

	/* set match counter */
	arch_timer_soc_access_enable();
	writel((0x20 + count) & 0xFFFF, watchdog_virt_base + TMR_WMR);
}
#endif /* #ifdef CONFIG_ARM_ARCH_TIMER */

static void eden_restart(char mode, const char *cmd)
{
	switch (mode) {
	case 's':
		/* Jump into ROM at address 0 */
		cpu_reset(0);
		break;
	case 'w':
	default:
#ifdef CONFIG_ARM_ARCH_TIMER
		do_wdt_reset(cmd);
#endif
		break;
	}
}

void apb_timer_soc_config(void)
{
	struct device_node *np;
	struct property *pp_zx, *pp;

	__raw_writel(0x13, MPMU_WDTPCR);

	if (!cpu_is_eden_a0()) {
		np = of_find_compatible_node(NULL, NULL, "marvell,mmp-timer");
		if (!np) {
			pr_info("Cannot find the soc timer device node.\n");
			return;
		}
		pp_zx = of_find_property(np,
					"marvell,timer-flag,zx", NULL);
		if (!pp_zx) {
			pr_info("Cannot find timer-flag for soc timer.\n");
			return;
		}
		pp = of_find_property(np,
					"marvell,timer-flag", NULL);
		if (!pp) {
			pr_info("Cannot find timer-flag for soc timer.\n");
			return;
		}
		*(int *)pp->value = *(int *)pp_zx->value;
	}
}

static __init void eden_timer_init(void)
{
#ifdef CONFIG_ARM_ARCH_TIMER
	arch_timer_soc_config();
#endif
	apb_timer_soc_config();

	clocksource_of_init();
}

static struct mv_usb_platform_data eden_usb_pdata = {
	.mode		= MV_USB_MODE_OTG,
	.extern_attr	= MV_USB_HAS_VBUS_DETECTION |
			MV_USB_HAS_IDPIN_DETECTION,
	.otg_force_a_bus_req = 1,
};

#ifdef CONFIG_UIO_HANTRO
#define APMU_VPU_CLK_RES_CTRL		APMU_REG(0x00a4)
#define APMU_ISLD_VPU_CTRL		APMU_REG(0x01b0)
#define APMU_ISLD_VPU_CMEM_DMMYCLK_EN	(1 << 4)

#define VPU_PWRUP_SLOW_RAMP		(1 << 9)
#define VPU_PWRUP_ON			(3 << 9)
#define VPU_PWRUP_MASK			(3 << 9)
#define VPU_ISB					(1 << 8)
#define VPU_REDUN_START			(1 << 2)

#define VPU_DECODER_RST				(1 << 28)
#define VPU_DECODER_CLK_EN			(1 << 27)
#define VPU_DECODER_CLK_DIV_MASK	(7 << 24)
#define VPU_DECODER_CLK_DIV_SHIFT	24
#define VPU_DECODER_CLK_SEL_MASK	(3 << 22)
#define VPU_DECODER_CLK_SEL_SHIFT	22

#define VPU_AXI_RST					(1 << 0)
#define VPU_AXI_CLK_EN				(1 << 3)
#define VPU_AXI_CLK_DIV_MASK		(7 << 19)
#define VPU_AXI_CLK_DIV_SHIFT		19
#define VPU_AXI_CLK_SEL_MASK		(3 << 12)
#define VPU_AXI_CLK_SEL_SHIFT		12

#define VPU_ENCODER_RST				(1 << 1)
#define VPU_ENCODER_CLK_EN			(1 << 4)
#define VPU_ENCODER_CLK_DIV_MASK	(7 << 16)
#define VPU_ENCODER_CLK_DIV_SHIFT	16
#define VPU_ENCODER_CLK_SEL_MASK	(3 << 6)
#define VPU_ENCODER_CLK_SEL_SHIFT	6

void hantro_power_switch(unsigned int enable)
{
	unsigned int reg = 0;
	unsigned int timeout;
	static unsigned int count;

	reg = readl(APMU_VPU_CLK_RES_CTRL);
	if (enable && count++ == 0) {
		if (reg & (VPU_PWRUP_ON | VPU_ISB))
			return; /*Pwr is already on*/

		/* 1. Turn on power switches */
		reg &= ~VPU_PWRUP_MASK;
		reg |= VPU_PWRUP_SLOW_RAMP;
		writel(reg, APMU_VPU_CLK_RES_CTRL);
		udelay(10);

		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_PWRUP_ON;
		writel(reg, APMU_VPU_CLK_RES_CTRL);
		udelay(10);

		/* 2. deassert isolation*/
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_ISB;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 3a. Assert the redundancy repair signal */
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_REDUN_START;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 3b. Poll and wait the REDUN_START bit back to 0*/
		timeout = 1000;
		do {
			if (--timeout == 0) {
				WARN(1, "hantro: REDUN_START timeout!\n");
				return;
			}
			udelay(100);
			reg = readl(APMU_VPU_CLK_RES_CTRL);
		} while (reg & VPU_REDUN_START);

		/* 4. enable dummy clks to the SRAM */
		reg = readl(APMU_ISLD_VPU_CTRL);
		reg |= APMU_ISLD_VPU_CMEM_DMMYCLK_EN;
		writel(reg, APMU_ISLD_VPU_CTRL);
		udelay(10);
		reg = readl(APMU_ISLD_VPU_CTRL);
		reg &= ~APMU_ISLD_VPU_CMEM_DMMYCLK_EN;
		writel(reg, APMU_ISLD_VPU_CTRL);

		/* 5. enable AXI clock */
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_AXI_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 6a. enable encoder clk */
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_ENCODER_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 6b. enable decoder clock */
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_DECODER_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 7. deassert AXI reset*/
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_AXI_RST;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 8a. deassert encoder reset*/
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_ENCODER_RST;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 8b. deassert decoder reset*/
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg |= VPU_DECODER_RST;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		/* 9. gate clocks */
		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg &= ~VPU_DECODER_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg &= ~VPU_ENCODER_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		reg = readl(APMU_VPU_CLK_RES_CTRL);
		reg &= ~VPU_AXI_CLK_EN;
		writel(reg, APMU_VPU_CLK_RES_CTRL);

		pr_debug("%s, power on\n", __func__);
	} else if (0 == enable) {
		if (count == 0)
			return;
		if (--count == 0) {

			if ((reg & (VPU_PWRUP_ON | VPU_ISB)) == 0)
				return; /*Pwr is already off*/

			/* 1. isolation */
			reg &= ~VPU_ISB;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			/* 2. reset*/
			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_AXI_RST;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_ENCODER_RST;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_DECODER_RST;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			/* 3. make sure clock disabled*/
			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_AXI_CLK_EN;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_ENCODER_CLK_EN;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			reg = readl(APMU_VPU_CLK_RES_CTRL);
			reg &= ~VPU_DECODER_CLK_EN;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			/* 4. turn off power */
			reg &= ~VPU_PWRUP_ON;
			writel(reg, APMU_VPU_CLK_RES_CTRL);

			pr_debug("%s, power off\n", __func__);
		}
	}
}
#endif

#ifdef CONFIG_VPU_DEVFREQ
#define VPU_DEVFREQ_DEC	0
#define VPU_DEVFREQ_ENC	1
#define VPU_DEVFREQ_NR	2

static int *vpu_freq_table[VPU_DEVFREQ_NR];
static struct devfreq_platform_data eden_devfreq_vpu_pdata[] = {
/* decoder */
	{
		.clk_name = "VPU_DEC_CLK",
	},
/* encoder */
	{
		.clk_name = "VPU_ENC_CLK",
	},
};

static void __init eden_devfreq_vpu_init(void)
{
	unsigned i, dev, op_num;

	for (dev = 0; dev < VPU_DEVFREQ_NR; dev++) {
		op_num = eden_get_vpu_op_num(dev);
		vpu_freq_table[dev] = kzalloc(sizeof(unsigned int) * op_num,
					GFP_KERNEL);
		if (!vpu_freq_table[dev]) {
			pr_err("%s, allocate memory fail!\n", __func__);
			return;
		}

		for (i = 0; i < op_num; i++)
			vpu_freq_table[dev][i] = eden_get_vpu_op_rate(dev, i);
		eden_devfreq_vpu_pdata[dev].freq_tbl_len = op_num;
		eden_devfreq_vpu_pdata[dev].freq_table = vpu_freq_table[dev];
	}
}
#endif

/* Reserve 1MB memory for OBM */
static void __init eden_reserve_obmmem(void)
{
	BUG_ON(memblock_reserve(PHYS_OFFSET, 0x100000) != 0);
	memblock_free(PHYS_OFFSET, 0x100000);
	memblock_remove(PHYS_OFFSET, 0x100000);
	pr_info("Reserved OBM memory: 0x%x@0x%lx\n",
			0x100000, PHYS_OFFSET);
}

/* CP memeory reservation, 64MB by default */
static u32 cp_area_size = 0x04000000;
static u32 cp_area_addr = 0x04000000;

static int __init early_cpmem(char *p)
{
	char *endp;

	cp_area_size = memparse(p, &endp);
	if (*endp == '@')
		cp_area_addr = memparse(endp + 1, NULL);

	return 0;
}
early_param("cpmem", early_cpmem);

static void __init eden_reserve_cpmem(void)
{
	BUG_ON(memblock_reserve(cp_area_addr, cp_area_size) != 0);
	memblock_free(cp_area_addr, cp_area_size);
	memblock_remove(cp_area_addr, cp_area_size);
	pr_info("Reserved CP memory: 0x%x@0x%x\n", cp_area_size, cp_area_addr);
}

#ifdef CONFIG_PSTORE_RAM
/*
 * reserve 256K memory from DDR address 0x8100000
 * pstore dump: 2 chunks, only dump panic log, no oops
 * console size: 192K
 * dump size: 64K
 */
struct ramoops_platform_data ram_data = {
	.mem_size = 0x40000,
	.mem_address = 0x8100000,
	.record_size = 0x8000,
	.console_size = 0x30000,
	.dump_oops = 0,
};
#endif

static void __init eden_init_ramoops(void)
{
#ifdef CONFIG_PSTORE_RAM
	int ret;
	unsigned long start, size;

	start = ram_data.mem_address;
	size = ram_data.mem_size;

	ret = memblock_reserve(start, size);
	if (ret) {
		pr_err("Failed to reserve persistent memory from %08lx-%08lx\n",
			start, (start + size - 1));
	}

	pr_info("reserve persistent memory from %08lx-%08lx\n",
		    start, (start + size - 1));
#endif
}

static void __init eden_reserve(void)
{
	extern char panel_type[];

	eden_reserve_obmmem();

	if (!strcmp(panel_type , "1080_50"))
		pr_info("1080p panel, no reserved memory for CP\n");
	else
		eden_reserve_cpmem();

#ifdef CONFIG_MMP_DISP
	mmp_reserve_fbmem();
#endif

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_reserve_gpu_memblock();
#endif

	eden_init_ramoops();
}

#ifdef CONFIG_VIDEO_MMP
/* At least one camere sensor should be configured */
#define ISP_POLL_COUNT (10)
static atomic_t isppwr_count;
#define APMU_CCIC_RST	APMU_REG(0x050)
#define APMU_CCIC2_RST	APMU_REG(0x0f4)
#define APMU_CCIC_DBG	APMU_REG(0x088)
#define APMU_ISPPWR	APMU_REG(0x1FC)
#define APMU_ISPCLK	APMU_REG(0x224)
#define APMU_ISLD_CI_CTRL		APMU_REG(0x01E0)
int isppwr_power_control(int on)
{
	int reg;
	unsigned char count = ISP_POLL_COUNT;

	if (on) {
		if (0 == atomic_read(&isppwr_count)) {
			/*set ISP regs to the default value*/
			writel(0, APMU_ISPPWR);

			/*1. turn on the power switch*/
			reg = readl(APMU_ISPPWR);
			reg |= 0x1 << 9;
			writel(reg, APMU_ISPPWR);
			udelay(10);
			reg |= 0x3 << 9;
			writel(reg, APMU_ISPPWR);
			udelay(10);

			/*2. disable isp isolation*/
			reg |= 0x1 << 8;
			writel(reg, APMU_ISPPWR);

			/*3. start memory redundacy repair*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 2;
			writel(reg, APMU_ISPCLK);
			udelay(10);
			while ((readl(APMU_ISPCLK) & (0x1 << 2)) && count--)
				udelay(5);
			if (readl(APMU_ISPCLK) & (0x1 << 2))
				printk(KERN_ERR "dxoisp err in memory redundacy repair\n");

			/*4. enable dummy clocks to the SRAMS*/
			reg = readl(APMU_ISLD_ISP_CTRL);
			reg |= 0x1 << 4;
			writel(reg, APMU_ISLD_ISP_CTRL);
			udelay(250);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_ISLD_ISP_CTRL);

			/* additional, enable CSI DVDD & AVDD */
			reg = readl(APMU_CCIC_DBG);
			reg |= 0x3 << 27;
			writel(reg, APMU_CCIC_DBG);
			reg |= 0x3 << 25;
			writel(reg, APMU_CCIC_DBG);

			/*5. enable clks*/
			/*enable CCIC1 AXI Arbiter clock*/
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 15;
			writel(reg, APMU_CCIC_RST);
			/*enable CCIC AXI clock*/
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 3;
			writel(reg, APMU_CCIC_RST);
			/*enable CCIC2 AXI clock*/
			reg = readl(APMU_CCIC2_RST);
			reg |= 0x1 << 3;
			writel(reg, APMU_CCIC2_RST);

			/* 6. changing clock source if needed */

			/* 7. Enable ISP & CCIC clock */
			/*enable ISP clk*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 4;
			writel(reg, APMU_ISPCLK);
			/* enable CCIC clk */
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 4;
			writel(reg, APMU_CCIC_RST);
			reg |= 0x1 << 5;
			writel(reg, APMU_CCIC_RST);
			/* enable CCIC2 clk */
			reg = readl(APMU_CCIC2_RST);
			reg |= 0x1 << 4;
			writel(reg, APMU_CCIC2_RST);
			reg |= 0x1 << 5;
			writel(reg, APMU_CCIC2_RST);

			/* 8. de-assert ISPDMACI AXI reset */
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 16;
			writel(reg, APMU_CCIC_RST);
			reg |= 0x1 << 0;
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg |= 0x1 << 0;
			writel(reg, APMU_CCIC2_RST);

			/* 9 de-assert ISP & CCIC software reset */
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 1;
			writel(reg, APMU_ISPCLK);
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 1;
			writel(reg, APMU_CCIC_RST);
			reg |= 0x1 << 2;
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg |= 0x1 << 1;
			writel(reg, APMU_CCIC2_RST);
			reg |= 0x1 << 2;
			writel(reg, APMU_CCIC2_RST);
		}

		if (atomic_inc_return(&isppwr_count) < 1) {
			printk(KERN_ERR "isp power on err\n");
			return -EINVAL;
		}
	} else {
		if (1 == atomic_read(&isppwr_count)) {
			/*enable clk for reset*/
			reg = readl(APMU_ISPCLK);
			reg |= 0x1 << 3;
			writel(reg, APMU_ISPCLK);
			reg |= 0x1 << 4;
			writel(reg, APMU_ISPCLK);
			reg = readl(APMU_CCIC_RST);
			reg |= 0x1 << 15;
			writel(reg, APMU_CCIC_RST);

			/*start to power down ISP*/
			/*1. enable isp isolation*/
			reg = readl(APMU_ISPPWR);
			reg &= ~(0x1 << 8);
			writel(reg, APMU_ISPPWR);

			/*2. assert peripheral AXI reset*/
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 16);
			writel(reg, APMU_CCIC_RST);
			reg &= ~(0x1 << 0);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg &= ~(0x1 << 0);
			writel(reg, APMU_CCIC2_RST);

			/* 3. assert peripheral software reset */
			reg = readl(APMU_ISPCLK);
			reg &= ~(0x1 << 1);
			writel(reg, APMU_ISPCLK);
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 1);
			writel(reg, APMU_CCIC_RST);
			reg &= ~(0x1 << 2);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg &= ~(0x1 << 1);
			writel(reg, APMU_CCIC2_RST);
			reg &= ~(0x1 << 2);
			writel(reg, APMU_CCIC2_RST);


			/* 4. Reset AXI clock enable */
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 15);
			writel(reg, APMU_CCIC_RST);
			reg &= ~(0x1 << 3);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg &= ~(0x1 << 3);
			writel(reg, APMU_CCIC2_RST);


			/* 5. Reset Peripheral clock enable */
			reg = readl(APMU_ISPCLK);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_ISPCLK);
			reg = readl(APMU_CCIC_RST);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_CCIC_RST);
			reg &= ~(0x1 << 5);
			writel(reg, APMU_CCIC_RST);
			reg = readl(APMU_CCIC2_RST);
			reg &= ~(0x1 << 4);
			writel(reg, APMU_CCIC2_RST);
			reg &= ~(0x1 << 5);
			writel(reg, APMU_CCIC2_RST);

			/* 6. Turn OFF the power switch */
			reg = readl(APMU_ISPPWR);
			reg &= ~(0x3 << 9);
			writel(reg, APMU_ISPPWR);

			atomic_set(&isppwr_count, 0);
		} else if (atomic_dec_return(&isppwr_count) < 0) {
			printk(KERN_ERR "isp power off err\n");
			return -EINVAL;
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_SOC_CAMERA_S5K8AA
#define CCIC2_PWDN_GPIO 13
#define CCIC2_RESET_GPIO 111
static int s5k8aa_sensor_power(struct device *dev, int on)
{
	static struct regulator *avdd_2v8;
	static struct regulator *dovdd_1v8;
	static struct regulator *af_2v8;
	static struct regulator *dvdd_1v2;
	int cam_enable = CCIC2_PWDN_GPIO;
	int cam_reset = CCIC2_RESET_GPIO;
	int ret;

	/* Get the regulators and never put it */
	/*
	 * The regulators is for sensor and should be in sensor driver
	 * As SoC camera does not support device tree, workaround here
	 */

	if (!avdd_2v8) {
		avdd_2v8 = regulator_get(dev, "avdd_2v8");
		if (IS_ERR(avdd_2v8)) {
			dev_warn(dev, "Failed to get regulator avdd_2v8\n");
			avdd_2v8 = NULL;
		}
	}

	if (!dovdd_1v8) {
		dovdd_1v8 = regulator_get(dev, "dovdd_1v8");
		if (IS_ERR(dovdd_1v8)) {
			dev_warn(dev, "Failed to get regulator dovdd_1v8\n");
			dovdd_1v8 = NULL;
		}
	}

	if (!af_2v8) {
		af_2v8 = regulator_get(dev, "af_2v8");
		if (IS_ERR(af_2v8)) {
			dev_warn(dev, "Failed to get regulator af_2v8\n");
			af_2v8 = NULL;
		}
	}

	if (!dvdd_1v2) {
		dvdd_1v2 = regulator_get(dev, "dvdd_1v2");
		if (IS_ERR(dvdd_1v2)) {
			dev_warn(dev, "Failed to get regulator dvdd_1v2\n");
			dvdd_1v2 = NULL;
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
		if (avdd_2v8) {
			regulator_set_voltage(avdd_2v8, 2800000, 2800000);
			ret = regulator_enable(avdd_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (af_2v8) {
			regulator_set_voltage(af_2v8, 2800000, 2800000);
			ret = regulator_enable(af_2v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (dvdd_1v2) {
			regulator_set_voltage(dvdd_1v2, 1200000, 1200000);
			ret = regulator_enable(dvdd_1v2);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		if (dovdd_1v8) {
			regulator_set_voltage(dovdd_1v8, 1800000, 1800000);
			ret = regulator_enable(dovdd_1v8);
			if (ret)
				goto cam_regulator_enable_failed;
		}

		gpio_direction_output(cam_reset, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_enable, 1);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 1);
		usleep_range(5000, 20000);
		isppwr_power_control(1);
	} else {
		gpio_direction_output(cam_enable, 0);
		usleep_range(5000, 20000);
		gpio_direction_output(cam_reset, 0);

		if (dovdd_1v8)
			regulator_disable(dovdd_1v8);
		if (dvdd_1v2)
			regulator_disable(dvdd_1v2);
		if (avdd_2v8)
			regulator_disable(avdd_2v8);
		if (af_2v8)
			regulator_disable(af_2v8);
		isppwr_power_control(0);
	}

	gpio_free(cam_enable);
	gpio_free(cam_reset);

	return 0;

cam_reset_failed:
	gpio_free(cam_enable);
cam_enable_failed:
	ret = -EIO;
cam_regulator_enable_failed:
	if (dvdd_1v2)
		regulator_put(dvdd_1v2);
	dvdd_1v2 = NULL;
	if (af_2v8)
		regulator_put(af_2v8);
	af_2v8 = NULL;
	if (dovdd_1v8)
		regulator_put(dovdd_1v8);
	dovdd_1v8 = NULL;
	if (avdd_2v8)
		regulator_put(avdd_2v8);
	avdd_2v8 = NULL;

	return ret;
}

static struct sensor_board_data s5k8aa_data = {
	.mount_pos	= SENSOR_USED | SENSOR_POS_FRONT | SENSOR_RES_LOW,
	.bus_type	= V4L2_MBUS_CSI2,
	.bus_flag	= V4L2_MBUS_CSI2_1_LANE,
	.dphy = {0xFF1D00, 0x00024733, 0x04001001},
};

static struct i2c_board_info concord_i2c_camera = {
	I2C_BOARD_INFO("s5k8aay", 0x3c),
};

static struct soc_camera_desc s5k8aa_desc = {
	.subdev_desc = {
		.power			= s5k8aa_sensor_power,
		.drv_priv		= &s5k8aa_data,
		.flags		= 0,
	},
	.host_desc = {
		.bus_id = 1,	/* Must match with the camera ID */
		.i2c_adapter_id = 2,
		.board_info		= &concord_i2c_camera,
		.module_name	= "s5k8aay",
	},
};
#endif

#ifdef CONFIG_SD8XXX_RFKILL
/*
 * This function depends on the platform and is specific to
 * Concord Rev1 currently.
 */
static void concord_wireless_set_power(unsigned int on)
{
	static int enabled;

	/*
	 * wib_1v8 -> v_buck2, wib_3v3 -> v_sys. Both should be always on.
	 */

	if (on && !enabled) {
		enabled = 1;
	}

	if (!on && enabled) {
		enabled = 0;
	}
	return;
}

struct sd8x_rfkill_platform_data sd8x_rfkill_platdata = {
	.set_power	= concord_wireless_set_power,
};
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
static struct sdhci_pxa_platdata eden_sdio_platdata = {
	.flags		= PXA_FLAG_WAKEUP_HOST
		| PXA_FLAG_ENABLE_CLOCK_GATING
		| PXA_FLAG_TX_SEL_BUS_CLK
		| PXA_FLAG_EN_PM_RUNTIME
		| PXA_FLAG_NEW_RX_CFG_REG,
	.quirks		= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.quirks2        = SDHCI_QUIRK2_BUS_CLK_GATE_ENABLED
		| SDHCI_QUIRK2_SDIO_SW_CLK_GATE,
	.pm_caps	= MMC_PM_KEEP_POWER,
};
static struct sdhci_pxa_platdata eden_emmc_platdata = {
	.quirks		= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
        .host_caps      = MMC_CAP_1_8V_DDR,
};
#endif

static const struct of_dev_auxdata eden_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("marvell,pdma-1.0", 0xd4000000, "mmp-pdma.0", NULL),
	OF_DEV_AUXDATA("marvell,pdma-1.0", 0xd4008000, "mmp-pdma.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4030000, "pxa2xx-uart.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4017000, "pxa2xx-uart.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4018000, "pxa2xx-uart.2", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4016000, "pxa2xx-uart.3", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4011000, "pxa2xx-i2c.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4031000, "pxa2xx-i2c.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4032000, "pxa2xx-i2c.2", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4033000, "pxa2xx-i2c.3", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4033800, "pxa2xx-i2c.4", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4034000, "pxa2xx-i2c.5", NULL),
	OF_DEV_AUXDATA("marvell,pxa25x-pwm", 0xd401a000, "mmp-pwm.0", NULL),
	OF_DEV_AUXDATA("marvell,pxa25x-pwm", 0xd401a400, "mmp-pwm.1", NULL),
	OF_DEV_AUXDATA("marvell,pxa25x-pwm", 0xd401a800, "mmp-pwm.2", NULL),
	OF_DEV_AUXDATA("marvell,pxa25x-pwm", 0xd401ac00, "mmp-pwm.3", NULL),
	OF_DEV_AUXDATA("marvell,eden-gpio", 0xd4019000, "eden-gpio", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-rtc", 0xd4010000, "sa1100-rtc", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xc0ffdc00, "mmp-sspa-dai.0", NULL),
	OF_DEV_AUXDATA("marvell,mmp-adma", 0xC0FFD800, "mmp-adma", NULL),
	OF_DEV_AUXDATA("marvell,mmp-88ce170-snd-card", 0, "sound", NULL),
#ifdef CONFIG_MV_USB2_PHY
	OF_DEV_AUXDATA("marvell,usb2-phy-28lp", 0xd4207000,
			"eden-usb-phy", NULL),
#endif
#ifdef CONFIG_USB_MV_UDC
	OF_DEV_AUXDATA("marvell,mv-udc", 0xd4208100, "mv-udc", &eden_usb_pdata),
#endif
#ifdef CONFIG_USB_EHCI_MV_U2O
	OF_DEV_AUXDATA("marvell,pxa-u2oehci", 0xd4208100, "pxa-u2oehci", &eden_usb_pdata),
#endif
#ifdef CONFIG_USB_MV_OTG
	OF_DEV_AUXDATA("marvell,mv-otg", 0xd4208100, "mv-otg", &eden_usb_pdata),
#endif
#ifdef CONFIG_VPU_DEVFREQ
	OF_DEV_AUXDATA("marvell,devfreq-vpu", 0xf0400000,
			"devfreq-vpu.0", (void *)&eden_devfreq_vpu_pdata[0]),
	OF_DEV_AUXDATA("marvell,devfreq-vpu", 0xf0400800,
			"devfreq-vpu.1", (void *)&eden_devfreq_vpu_pdata[1]),
#endif
#ifdef CONFIG_EDEN_THERMAL
	OF_DEV_AUXDATA("mrvl,thermal", 0xd403b000, "thermal", NULL),
#endif
#ifdef CONFIG_SOC_CAMERA_S5K8AA
	OF_DEV_AUXDATA("soc-camera-pdrv", 0, "soc-camera-pdrv.0", &s5k8aa_desc),
#endif
	OF_DEV_AUXDATA("marvell,mmp-ccic", 0xd420a800, "mmp-camera.1", NULL),
	OF_DEV_AUXDATA("pxa-ion", 0, "pxa-ion", NULL),
#ifdef CONFIG_SMC91X
	OF_DEV_AUXDATA("smsc,lan91c111", 0x80000300, "smc91x", NULL),
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280000, "sdhci-pxav3.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280800, "sdhci-pxav3.1", &eden_sdio_platdata),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4217000, "sdhci-pxav3.2", &eden_emmc_platdata),
#endif
#ifdef CONFIG_SD8XXX_RFKILL
	OF_DEV_AUXDATA("mrvl,sd8x-rfkill", 0, "sd8x-rfkill",
		       &sd8x_rfkill_platdata),
#endif
#ifdef CONFIG_PSTORE_RAM
	OF_DEV_AUXDATA("ramoops", 0, "ramoops", &ram_data),
#endif
	{}
};

static void apb_thermal_soc_config(void)
{
	struct device_node *np;
	struct property *pp_zx = NULL;
	struct property *pp = NULL;

	if (!cpu_is_eden_a0()) {
		np = of_find_compatible_node(NULL, NULL, "mrvl,thermal");
		if (!np) {
			pr_err("Cannot find the thermal device node.\n");
			return;
		}
		if (cpu_is_eden_z1()) {
			pp_zx = of_find_property(np,
					"marvell,version-flag,z1", NULL);
			if (!pp_zx) {
				pr_err("Cannot find theraml for z1 platform.\n");
				return;
			}
		} else if (cpu_is_eden_z2()) {
			pp_zx = of_find_property(np,
					"marvell,version-flag,z2", NULL);
			if (!pp_zx) {
				pr_err("Cannot find theraml for z2 platform.\n");
				return;
			}
		}
		pp = of_find_property(np,
				"marvell,version-flag", NULL);
		if (!pp) {
			pr_err("Cannot find thermal version flag\n");
			return;
		}
		*(int *)pp->value = *(int *)pp_zx->value;
	}
}

static void __init eden_dt_init_machine(void)
{
	eden_clk_init(APB_PHYS_BASE, AXI_PHYS_BASE,
		AUD_PHYS_BASE, AUD_PHYS_BASE2);
	eden_core_clk_init(AXI_PHYS_BASE);

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_add_gpu();
#endif

#ifdef CONFIG_VPU_DEVFREQ
	eden_devfreq_vpu_init();
#endif

#ifdef CONFIG_EDEN_THERMAL
	apb_thermal_soc_config();
#endif

	of_platform_populate(NULL, of_default_bus_match_table,
			     eden_auxdata_lookup, &platform_bus);
	/* put gc in reset state */
	gc_reset();

	/* raise lcd/vdma port priority */
	writel(0xff003030, DMCU_VIRT_BASE + (0x84));
}

static const char *eden_dt_board_compat[] __initdata = {
	"mrvl,eden-concord",
	NULL,
};

DT_MACHINE_START(EDEN_DT, "EDEN")
	.smp		= smp_ops(mmp_smp_ops),
	.map_io		= mmp_map_io,
	.init_irq	= irqchip_init,
	.init_time	= eden_timer_init,
	.init_machine	= eden_dt_init_machine,
	.dt_compat	= eden_dt_board_compat,
	.restart	= eden_restart,
	.reserve	= eden_reserve,
MACHINE_END
