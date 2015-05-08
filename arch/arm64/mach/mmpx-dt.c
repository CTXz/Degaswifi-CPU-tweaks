/*
 *  linux/arch/arm64/mach/mmpx-dt.c
 *
 *  Copyright (C) 2012 Marvell Technology Group Ltd.
 *  Author: Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/clocksource.h>
#include <linux/gpio.h>
#include <linux/memblock.h>
#include <linux/platform_data/mv_usb.h>

#include <asm/device_mapping.h>

#include <asm/mach/mach_desc.h>

#ifdef CONFIG_USB_SUPPORT
static struct mv_usb_platform_data eden_usb_pdata = {
	.mode		= MV_USB_MODE_OTG,
	.extern_attr	= MV_USB_HAS_VBUS_DETECTION |
			MV_USB_HAS_IDPIN_DETECTION,
	.otg_force_a_bus_req = 1,
};
#endif

static const struct of_dev_auxdata mmpx_auxdata_lookup[] __initconst = {
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
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xc0ffdc00, "mmp-sspa-dai.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xc0ffdd00, "mmp-sspa-dai.1", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xC0FFD800, "mmp-adma.0", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xC0FFD900, "mmp-adma.1", NULL),
#ifdef CONFIG_RTC_DRV_SA1100
	OF_DEV_AUXDATA("mrvl,mmp-rtc", 0xd4010000, "sa1100-rtc", NULL),
#endif
#ifdef CONFIG_MMC_SDHCI_PXAV3
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280000, "sdhci-pxav3.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280800, "sdhci-pxav3.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4217000, "sdhci-pxav3.2", NULL),
#endif
#ifdef CONFIG_SMC91X
	OF_DEV_AUXDATA("smsc,lan91c111", 0x80000300, "smc91x", NULL),
#endif
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

	{}
};

#define APB_PHYS_BASE		0xd4000000
#define AXI_PHYS_BASE		0xd4200000
#define AUD_PHYS_BASE           0xc3000000
#define AUD_PHYS_BASE2          0xc0140000
extern void __init eden_clk_init(unsigned long, unsigned long, unsigned long, unsigned long);

#ifdef CONFIG_ARM_ARCH_TIMER
#define MPMU_CPRR              0x0020
#define MPMU_WDTPCR1           0x0204
#define MPMU_APRR              0x1020

#define MPMU_APRR_WDTR         (1 << 4)
#define MPMU_APRR_CPR          (1 << 0)
#define MPMU_CPRR_DSPR         (1 << 2)
#define MPMU_CPRR_BBR          (1 << 3)

#define TMR_WFAR               (0x009c)
#define TMR_WSAR               (0x00A0)

#define GEN_TMR_CFG            (0x00B0)
#define GEN_TMR_LD1            (0x00B8)

/* Get SoC Access to Generic Timer */
inline void arch_timer_soc_access_enable(void __iomem *gen_counter_base)
{
	__raw_writel(0xbaba, gen_counter_base + TMR_WFAR);
	__raw_writel(0xeb10, gen_counter_base + TMR_WSAR);
}

void arch_timer_soc_config(void)
{
	void __iomem *gen_counter_base = apb_virt_base + 0x80000;
	void __iomem *mpmu_base = apb_virt_base + 0x50000;

	u32 tmp;

	/* Enable WDTR2*/
	tmp  = __raw_readl(mpmu_base + MPMU_CPRR);
	tmp = tmp | MPMU_APRR_WDTR;
	__raw_writel(tmp, mpmu_base + MPMU_CPRR);

	/* Initialize Counter to zero */
	arch_timer_soc_access_enable(gen_counter_base);
	__raw_writel(0x0, gen_counter_base + GEN_TMR_LD1);

	/* Program Generic Timer Clk Frequency */
	arch_timer_soc_access_enable(gen_counter_base);
	tmp = __raw_readl(gen_counter_base + GEN_TMR_CFG);
	tmp |= (3 << 4); /* 3.25MHz/32KHz Counter auto switch enabled */
	arch_timer_soc_access_enable(gen_counter_base);
	__raw_writel(tmp, gen_counter_base + GEN_TMR_CFG);

	/* Start the Generic Timer Counter */
	arch_timer_soc_access_enable(gen_counter_base);
	tmp = __raw_readl(gen_counter_base + GEN_TMR_CFG);
	tmp |= 0x3;
	arch_timer_soc_access_enable(gen_counter_base);
	__raw_writel(tmp, gen_counter_base + GEN_TMR_CFG);
}
#endif /* #ifdef CONFIG_ARM_ARCH_TIMER */

#define MPMU_WDTPCR	0x0200

static __init void eden_timer_init(void)
{
	void __iomem *mpmu_base = apb_virt_base + 0x50000;

#ifdef CONFIG_ARM_ARCH_TIMER
	arch_timer_soc_config();
#endif

	__raw_writel(0x13, mpmu_base + MPMU_WDTPCR);

	clocksource_of_init();
}

/* CP memeory reservation, 64MB by default */
static u32 cp_area_size = 0x04000000;
static phys_addr_t cp_area_addr = 0x04000000;

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
	pr_info("Reserved CP memory: 0x%x@0x%llx\n", cp_area_size, cp_area_addr);
}

static void __init eden_reserve(void)
{
	eden_reserve_cpmem();
}

static void __init mmpx_dt_init_machine(void)
{
	/* init clk */
	eden_clk_init(APB_PHYS_BASE, AXI_PHYS_BASE,
		AUD_PHYS_BASE, AUD_PHYS_BASE2);

	of_platform_populate(NULL, of_default_bus_match_table,
			     mmpx_auxdata_lookup, NULL);
}

static const char *mmpx_dt_board_compat[] __initdata = {
	"mrvl,eden",
	NULL,
};

struct machine_desc mmp_machine_desc __initdata = {
	.name		= "Marvell Eden",
	.reserve	= eden_reserve,
	.init_time	= eden_timer_init,
	.init_machine	= mmpx_dt_init_machine,
	.dt_compat	= mmpx_dt_board_compat,
};
