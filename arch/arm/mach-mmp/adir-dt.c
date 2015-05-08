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
#include <linux/usb/mv_usb2.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/features.h>
#include <asm/smp_twd.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/irqs.h>
#include <mach/regs-accu.h>
#include <mach/regs-timers.h>
#include <mach/addr-map.h>
#include <mach/regs-coresight.h>
#include <media/soc_camera.h>
#include <media/mrvl-camera.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/card.h>
#include <linux/platform_data/pxa_sdhci.h>
#include <dt-bindings/mmc/pxa_sdhci.h>
#include <linux/memblock.h>
#include <linux/pstore_ram.h>
#include <video/mmp_disp.h>
#ifdef CONFIG_GPU_RESERVE_MEM
#include <mach/gpu_mem.h>
#endif
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#include "common.h"
#include "reset.h"
#include <linux/regdump_ops.h>
#include <linux/regs-addr.h>

#define MHZ_TO_KHZ	1000

static struct mv_usb_platform_data pxa1986_usb_pdata = {
	.mode		= MV_USB_MODE_UDC,
	.extern_attr	= MV_USB_HAS_VBUS_DETECTION
			| MV_USB_HAS_IDPIN_DETECTION,
};

static struct sdhci_pxa_dtr_data sd_dtr_data[] = {
	{
		.timing		= MMC_TIMING_LEGACY, /* < 25MHz */
		.preset_rate	= PXA_SDH_DTR_26M,
		.src_rate	= PXA_SDH_DTR_52M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR12, /* 25MHz */
		.preset_rate	= PXA_SDH_DTR_26M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR25, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_SD_HS, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_DDR50, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR50, /* 100MHz */
		.preset_rate	= PXA_SDH_DTR_104M,
		.src_rate	= PXA_SDH_DTR_208M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR104, /* 208MHz */
		.preset_rate	= PXA_SDH_DTR_208M,
		.src_rate	= PXA_SDH_DTR_416M,
	},
		/*
		 * end of sdhc dtr table
		 * set as the default src rate
		 */
	{
		.timing		= MMC_TIMING_MAX,
		.preset_rate	= PXA_SDH_DTR_PS_NONE,
		.src_rate	= PXA_SDH_DTR_208M,
	},
};

static struct sdhci_pxa_dtr_data sdio_dtr_data[] = {
	{
		.timing		= MMC_TIMING_LEGACY, /* < 25MHz */
		.preset_rate	= PXA_SDH_DTR_26M,
		.src_rate	= PXA_SDH_DTR_52M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR12, /* 25MHz */
		.preset_rate	= PXA_SDH_DTR_26M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR25, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_SD_HS, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_45M,
		.src_rate	= PXA_SDH_DTR_89M,
	},
	{
		.timing		= MMC_TIMING_UHS_DDR50, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR50, /* 100MHz */
		.preset_rate	= PXA_SDH_DTR_104M,
		.src_rate	= PXA_SDH_DTR_208M,
	},
	{
		.timing		= MMC_TIMING_UHS_SDR104, /* 208MHz */
		.preset_rate	= PXA_SDH_DTR_208M,
		.src_rate	= PXA_SDH_DTR_416M,
	},
	{
		.timing		= MMC_TIMING_MAX,
		.preset_rate	= PXA_SDH_DTR_PS_NONE,
		.src_rate	= PXA_SDH_DTR_89M,
	},
};

static struct sdhci_pxa_dtr_data emmc_dtr_data[] = {
	{
		.timing		= MMC_TIMING_LEGACY, /* < 25MHz */
		.preset_rate	= PXA_SDH_DTR_26M,
		.src_rate	= PXA_SDH_DTR_52M,
	},
	{
		.timing		= MMC_TIMING_MMC_HS, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_UHS_DDR50, /* 50MHz */
		.preset_rate	= PXA_SDH_DTR_52M,
		.src_rate	= PXA_SDH_DTR_104M,
	},
	{
		.timing		= MMC_TIMING_MMC_HS200, /* 208MHz */
		.preset_rate	= PXA_SDH_DTR_156M,
		.src_rate	= PXA_SDH_DTR_156M,
	},
	{
		.timing		= MMC_TIMING_MAX,
		.preset_rate	= PXA_SDH_DTR_PS_NONE,
		.src_rate	= PXA_SDH_DTR_208M,
	},
};

/* For emeiDKB, MMC1(SDH1) used for SD/MMC Card slot */
static struct sdhci_pxa_platdata pxa1986_sdh_platdata_mmc1 = {
	.host_caps_disable	= MMC_CAP_UHS_SDR104,
	.dtr_data		= sd_dtr_data,
};

/* For emeiDKB, MMC2(SDH2) used for WIB card */
static struct sdhci_pxa_platdata pxa1986_sdh_platdata_mmc2 = {
	.flags		= PXA_FLAG_WAKEUP_HOST
		| PXA_FLAG_ENABLE_CLOCK_GATING
		| PXA_FLAG_TX_SEL_BUS_CLK
		| PXA_FLAG_EN_PM_RUNTIME
		| PXA_FLAG_DISABLE_PROBE_CDSCAN,
	.quirks2        = SDHCI_QUIRK2_BUS_CLK_GATE_ENABLED
		| SDHCI_QUIRK2_SDIO_SW_CLK_GATE,
	.pm_caps	= MMC_PM_KEEP_POWER,
	.dtr_data	= sdio_dtr_data,
};

/* For emeiDKB, MMC3(SDH3) used for eMMC */
static struct sdhci_pxa_platdata pxa1986_sdh_platdata_mmc3 = {
	.quirks		= SDHCI_QUIRK_BROKEN_ADMA,
	.host_caps	= MMC_CAP_1_8V_DDR,
	.host_caps2	= MMC_CAP2_DISABLE_BLK_ASYNC,
	.dtr_data	= emmc_dtr_data,
};

#ifdef CONFIG_DDR_DEVFREQ
static struct devfreq_platform_data mmpx_devfreq_ddr_pdata = {
	.clk_name = "ddr",
};

static struct devfreq_pm_qos_table ddr_freq_qos_table[] = {
	/* list all possible frequency level here */
	{
		.freq = 156000,
		.qos_value = DDR_CONSTRAINT_LVL0,
	},
	{
		.freq = 312000,
		.qos_value = DDR_CONSTRAINT_LVL1,
	},
	{
		.freq = 400000,
		.qos_value = DDR_CONSTRAINT_LVL2,
	},
	{
		.freq = 533000,
		.qos_value = DDR_CONSTRAINT_LVL3,
	},
	{0, 0},
};

static void __init pxa1986_devfreq_ddr_init(void)
{
	unsigned int ddr_freq_num;
	unsigned int *ddr_freq_table;
	unsigned int i;

	ddr_freq_num = get_ddr_op_num();

	ddr_freq_table = kzalloc(sizeof(unsigned int) * ddr_freq_num,
			GFP_KERNEL);
	if (!ddr_freq_table)
		return;

	for (i = 0; i < ddr_freq_num; i++)
		ddr_freq_table[i] = get_ddr_op_rate(i);

	mmpx_devfreq_ddr_pdata.freq_tbl_len = ddr_freq_num;
	mmpx_devfreq_ddr_pdata.freq_table = ddr_freq_table;

	mmpx_devfreq_ddr_pdata.qos_list = ddr_freq_qos_table;
}
#endif

#ifdef CONFIG_VPU_DEVFREQ
static struct devfreq_platform_data devfreq_vpu_pdata = {
	.clk_name = "VPUCLK",
};
static void __init pxa1986_devfreq_vpu_init(void)
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

/* PXA1986 */
static const struct of_dev_auxdata pxa1986_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("marvell,pdma-1.0", 0xd4000000, "mmp-pdma.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4030000, "pxa2xx-uart.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4017000, "pxa2xx-uart.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd430b000, "pxa910-i2c.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4011000, "pxa910-i2c.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd401b000, "pxa910-i2c.2", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4032000, "pxa910-i2c.3", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4033000, "pxa910-i2c.4", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4034000, "pxa910-i2c.5", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-rtc", 0xd4086000, "sa1100-rtc", NULL),
	OF_DEV_AUXDATA("marvell,pxa27x-keypad", 0xd4085000,
			"pxa27x-keypad", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xc0ffdc00, "mmp-sspa-dai.0", NULL),
	OF_DEV_AUXDATA("marvell,mmp-disp", 0xd420b000, "mmp-disp", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb", 0, "mmp-fb", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 0, "mmp-fb-overlay", NULL),
	OF_DEV_AUXDATA("marvell,mmp-tft-10801920-1e", 0, "mmp-tft-10801920-1e", NULL),
	OF_DEV_AUXDATA("marvell,mmp-dsi", 0xd420b800, "mmp-dsi", NULL),
	OF_DEV_AUXDATA("marvell,mmp-adma", 0xC0FFD800, "mmp-adma", NULL),
#ifdef CONFIG_MV_USB2_PHY
	OF_DEV_AUXDATA("marvell,usb2-phy-28lp", 0xd4220000,
			"pxa1986-usb-phy", NULL),
#endif
#ifdef CONFIG_USB_MV_UDC
	OF_DEV_AUXDATA("marvell,mv-udc", 0xd4208100, "mv-udc", &pxa1986_usb_pdata),
#endif
#ifdef CONFIG_USB_MV_OTG
	OF_DEV_AUXDATA("marvell,mv-otg", 0xd4208100, "mv-otg", &pxa1986_usb_pdata),
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
#ifdef CONFIG_MMC_SDHCI_PXAV3
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280000, "sdhci-pxav3.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280800, "sdhci-pxav3.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4217000, "sdhci-pxav3.2", NULL),
#endif
#ifdef CONFIG_PSTORE_RAM
	OF_DEV_AUXDATA("ramoops", 0, "ramoops", &ram_data),
#endif
	{}
};

static void __init pxa1986_dt_irq_init(void)
{
	irqchip_init();
}

static void __init pxa1986_dt_init_machine(void)
{
#if 0
#ifdef CONFIG_DDR_DEVFREQ
	pxa1986_devfreq_ddr_init();
#endif

#ifdef CONFIG_VPU_DEVFREQ
	pxa1986_devfreq_vpu_init();
#endif


#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_add_gpu();
#endif

#endif
	pxa1986_clk_init();

	of_platform_populate(NULL, of_default_bus_match_table,
			     pxa1986_auxdata_lookup, &platform_bus);
}

#ifdef CONFIG_ARM_ARCH_TIMER
static void enable_ts_fast_cnt(void)
{
	u32 val;

	/* Enable timestamp fast counter: used by generic timer */
	val = __raw_readl(TIMESTAMP_VIRT_BASE + TIMESTAMP_CTRL);
	__raw_writel(val | (1 << 1), TIMESTAMP_VIRT_BASE + TIMESTAMP_CTRL);

	/* Cannot do this: SEC write only, undef exception in NS.
		Will be done in SEC CPU init. Dynamic changes are TBD. */
	/* Update system counter frequency register - otherwise Generic Timer
	 * initializtion fails.
	 * By default(after reset) fast couter is configured to 3.25 MHz.
	 * If fast counter clock will be chnaged, value below should be
	 * modified as well.
	 */
	asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r" (val));
	if (!val) {
		/* Not set by secure init, so assume we are in secure state */
		val = 3250000;
		asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (val));
	}
	pr_info("Timestamp fast counter is enabled\n");
}
#endif

static __init void pxa1986_timer_init(void)
{
	uint32_t clk_rst;

	/* Select the configurable timer clock source to be 6.5MHz */
	writel_relaxed(ACCU_APBCLK | ACCU_RST, get_accu_base_va() + ACCU_APPS_TIMERS1_CLK_CNTRL_REG);
	clk_rst = ACCU_APBCLK | ACCU_FNCLK | SET_ACCU_RATIO(4);
	writel_relaxed(clk_rst, get_accu_base_va() + ACCU_APPS_TIMERS1_CLK_CNTRL_REG);

#ifdef CONFIG_ARM_ARCH_TIMER
	enable_ts_fast_cnt();
#endif
	clocksource_of_init();
}

#define CP_MEM_MAX_SEGMENTS 2
unsigned _cp_area_addr[CP_MEM_MAX_SEGMENTS];
unsigned _cp_area_size[CP_MEM_MAX_SEGMENTS+1]; /* last entry 0 */
/* This must be early_param: reservation is done by MACHINE_DESC.reserve
	(see below), invoked by arch/arm/mm/.c:arm_memblock_init() called in
	arch/arm/kernel/setup.c:setup_arch()
	after parse_early_param() and before paging_init(). */
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0xa7000000;
	int seg;

	size  = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);
	for (seg = 0; seg < CP_MEM_MAX_SEGMENTS; seg++)
		if (!_cp_area_size[seg])
			break;
	BUG_ON(seg == CP_MEM_MAX_SEGMENTS);
	_cp_area_addr[seg] = (unsigned)start;
	_cp_area_size[seg] = (unsigned)size;
	return 0;
}
early_param("cpmem", setup_cpmem);

/* Exported for telephony use */
unsigned cp_area_addr(void)
{
	/* _cp_area_addr[] contain actual CP region addresses for reservation.
	This function returns the address of the first region, which is
	the main one used for AP-CP interface, aligned to 16MB.
	The AP-CP interface code takes care of the offsets inside the region,
	including the non-CP area at the beginning of the 16MB aligned range. */
	return _cp_area_addr[0]&0xFF000000;
}
EXPORT_SYMBOL(cp_area_addr);

static void __init pxa1986_reserve_cpmem(void)
{
	int seg;

	/* Built-in default: the legacy 16MB option */
	if (!_cp_area_size[0]) {
		_cp_area_addr[0] = 0x0;
		_cp_area_size[0] = 0x01000000;
	}
	for (seg = 0; _cp_area_size[seg]; seg++) {
		/* Reserve 32MB memory for CP */
		unsigned addr = _cp_area_addr[seg];
		unsigned size = _cp_area_size[seg];
		BUG_ON(memblock_reserve(addr, size) != 0);
		memblock_free(addr, size);
		memblock_remove(addr, size);
		pr_info("Reserved CP memory: 0x%x@0x%x\n", size, addr);
	}
}

static void __init pxa1986_reserve(void)
{
	pxa1986_reserve_cpmem();
#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_reserve_gpu_memblock();
#endif

#ifdef CONFIG_MMP_DISP
	mmp_reserve_fbmem();
#endif
}

static const char *pxa1986_dt_board_compat[] __initdata = {
	"mrvl,pxa1986sdk",
	NULL,
};

DT_MACHINE_START(PXA1986_DT, "PXA1986")
	.smp_init	= smp_init_ops(mmp_smp_init_ops),
	.map_io		= mmp_map_io,
	.init_irq	= pxa1986_dt_irq_init,
	.init_time	= pxa1986_timer_init,
	.init_machine	= pxa1986_dt_init_machine,
	.dt_compat	= pxa1986_dt_board_compat,
	.reserve	= pxa1986_reserve,
	.restart	= mmp_arch_restart,
MACHINE_END
