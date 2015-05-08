/*
 * pxa910 clock framework source file
 *
 * Copyright (C) 2012 Marvell
 * Chao Xie <xiechao.mail@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <mach/addr-map.h>

#include "clk.h"

#define APBC_RTC	0x28
#define APBC_TWSI0	0x2c
#define APBC_KPC	0x18
#define APBC_UART0	0x0
#define APBC_UART1	0x4
#define APBC_GPIO	0x8
#define APBC_PWM0	0xc
#define APBC_PWM1	0x10
#define APBC_PWM2	0x14
#define APBC_PWM3	0x18
#define APBC_SSP0	0x1c
#define APBC_SSP1	0x20
#define APBC_SSP2	0x4c
#define APBCP_TWSI1	0x28
#define APBCP_UART2	0x1c
#define APMU_SDH0	0x54
#define APMU_SDH1	0x58
#define APMU_SDH2	0xe0
#define APMU_USB	0x5c
#define APMU_DSI1	0x44
#define APMU_DISP0	0x4c
#define APMU_CCIC0	0x50
#define APMU_DFC	0x60
#define MPMU_UART_PLL	0x14

static DEFINE_SPINLOCK(clk_lock);

#define LCD_PN_SCLK	(0xd420b1a8)

static struct clk_factor_masks uart_factor_masks = {
	.factor = 2,
	.num_mask = 0x1fff,
	.den_mask = 0x1fff,
	.num_shift = 16,
	.den_shift = 0,
};

static struct clk_factor_tbl uart_factor_tbl[] = {
	{.num = 8125, .den = 1536},	/*14.745MHZ */
};

static const char *uart_parent[] = {"pll1_3_16", "uart_pll"};
static const char *ssp_parent[] = {"pll1_96", "pll1_48", "pll1_24", "pll1_12"};
static const char *sdh_parent[] = {"pll1_12", "pll1_13"};
static const char *disp_parent[] = {"pll1", "pll1_416m"};
static const char *ccic_parent[] = {"pll1_2", "pll1_12"};
static const char *ccic_phy_parent[] = {"pll1_6", "pll1_12"};
/* ccic_fn_parent:
 * 00: 416MHz
 * 01: 624MHz
 * 10: pll2
 * 11: pll2p
 * the pll2 and pll2p are not implemented here, using pll1_12, pll1_24 instead
 * make sure the last 2 source clks are not used in driver
 */
static const char *ccic_fn_parent[] = {"pll1_13_1_5", "pll1",
					"pll1_12", "pll1_24"};

#define POSR_PLL2_LOCK		(1 << 29)
#define POSR_PLL3_LOCK		(1 << 30)
static DEFINE_SPINLOCK(pll2_lock);
static DEFINE_SPINLOCK(pll3_lock);

struct kvco_range kvco_rng_table[] = {
	{2400, 2500, 7, 7},
	{2150, 2400, 6, 6},
	{1950, 2150, 5, 5},
	{1750, 1950, 4, 4},
	{1550, 1750, 3, 3},
	{1350, 1550, 2, 2},
	{1200, 1350, 1, 1},
};
/* PLL post divider table */
static struct div_map pll_post_div_tbl[] = {
	/* divider, reg vaule */
	{1, 0},
	{2, 2},
	{3, 4},
	{4, 5},
	{6, 7},
	{8, 8},
};

struct pll_offset pll2_offset = {
	.fbd_shift = 10,
	.fbd_width = 9,
	.refd_shift = 19,
	.refd_width = 5,
	.kvco_shift = 17,
	.kvco_width = 4,
	.vrng_shift = 14,
	.vrng_width = 3,
};

struct pll_offset pll3_offset = {
	.fbd_shift = 5,
	.fbd_width = 9,
	.refd_shift = 0,
	.refd_width = 5,
	.kvco_shift = 17,
	.kvco_width = 4,
	.vrng_shift = 14,
	.vrng_width = 3,
};

struct mmp_vco_params pll2_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 2500000000UL,
	.enable_bit = 0x100,
	.ctrl_bit = 0x200,
	.lock_enable_bit = POSR_PLL2_LOCK,
	.lock_delay = 100,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pll2_offset,
};

struct mmp_vco_params pll3_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 2500000000UL,
	.enable_bit = 0x80000,
	.ctrl_bit = 0x40000,
	.lock_enable_bit = POSR_PLL3_LOCK,
	.lock_delay = 100,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pll3_offset,
};

static struct mmp_pll_params pll2_pll_params = {
	.div_shift = 4,
	.div_width = 4,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll2_pllp_params = {
	.div_shift = 8,
	.div_width = 4,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll3_pll_params = {
	.div_shift = 4,
	.div_width = 4,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll3_pllp_params = {
	.div_shift = 8,
	.div_width = 4,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

void __init pxa988_pll_init(void *mpmu_base, void *apbs_base)
{
	unsigned long pll2_vco_default;
	unsigned long pll2_default;
	unsigned long pll2p_default;
	unsigned long pll3_vco_default;
	unsigned long pll3_default;
	unsigned long pll3p_default;
	struct clk *clk;
	struct clk *pll2_vco, *pll2, *pll2p, *pll3_vco, *pll3, *pll3p;

	clk = clk_register_fixed_rate(NULL, "pll1_624", NULL, CLK_IS_ROOT,
				624000000);
	clk_register_clkdev(clk, "pll1_624", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_416m", NULL, CLK_IS_ROOT,
				416000000);
	clk_register_clkdev(clk, "pll1_416m", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_1248", NULL, CLK_IS_ROOT,
				1248000000);
	clk_register_clkdev(clk, "pll1_1248", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_2", "pll1_624",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_2", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_4", "pll1_2",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_4", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_8", "pll1_4",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_8", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_16", "pll1_8",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_16", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_6", "pll1_2",
				CLK_SET_RATE_PARENT, 1, 3);
	clk_register_clkdev(clk, "pll1_6", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_12", "pll1_6",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_12", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_24", "pll1_12",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_24", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_48", "pll1_24",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_48", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_96", "pll1_48",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_96", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_13", "pll1_624",
				CLK_SET_RATE_PARENT, 1, 13);
	clk_register_clkdev(clk, "pll1_13", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_13_1_5", "pll1_624",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_13_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_2_1_5", "pll1_624",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_2_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_3_16", "pll1_624",
				CLK_SET_RATE_PARENT, 3, 16);
	clk_register_clkdev(clk, "pll1_3_16", NULL);

	pll2_vco_params.cr_reg = mpmu_base + 0x0034;
	pll2_vco_params.pll_swcr = apbs_base + 0x0104;
	pll2_vco_params.lock_reg = mpmu_base + 0x0010;
	pll2_vco_params.default_vco_rate = 2132 * MHZ_TO_HZ;
	pll2_pll_params.pll_swcr = apbs_base + 0x0104;
	pll2_pllp_params.pll_swcr = apbs_base + 0x0104;

	pll3_vco_params.cr_reg = mpmu_base + 0x001c;
	pll3_vco_params.pll_swcr = apbs_base + 0x0108;
	pll3_vco_params.lock_reg = mpmu_base + 0x0010;
	pll3_vco_params.default_vco_rate = 1205 * MHZ_TO_HZ;
	pll3_pll_params.pll_swcr = apbs_base + 0x0108;
	pll3_pllp_params.pll_swcr = apbs_base + 0x0108;

	pll2_vco_default = 2132 * MHZ_TO_HZ;
	pll2_default = 1066 * MHZ_TO_HZ;
	pll2p_default = 1066 * MHZ_TO_HZ;
	pll2_vco = mmp_clk_register_vco("pll2_vco", NULL, CLK_IS_ROOT,
		MMP_PLL_USE_LOCK|MMP_PLL_LOCK_SETTING, &pll2_lock,
		&pll2_vco_params, NULL, 0);
	clk_register_clkdev(pll2_vco, "pll2_vco", NULL);
	clk_set_rate(pll2_vco, pll2_vco_default);

	pll2 = mmp_clk_register_pll("pll2", "pll2_vco", CLK_SET_RATE_PARENT,
		0, &pll2_lock, &pll2_pll_params);
	clk_register_clkdev(pll2, "pll2", NULL);

	pll2p = mmp_clk_register_pll("pll2p", "pll2_vco", CLK_SET_RATE_PARENT,
		0, &pll2_lock, &pll2_pllp_params);
	clk_register_clkdev(pll2p, "pll2p", NULL);
	clk_set_rate(pll2, pll2_default);
	clk_set_rate(pll2p, pll2p_default);

	pll3_vco_default = 1205 * MHZ_TO_HZ;
	pll3_default = 1205 * MHZ_TO_HZ;
	pll3p_default = 1205 * MHZ_TO_HZ;
	pll3_vco = mmp_clk_register_vco("pll3_vco", NULL, CLK_IS_ROOT,
		MMP_PLL_USE_LOCK, &pll3_lock,
		&pll3_vco_params, NULL, 0);
	clk_register_clkdev(pll3_vco, "pll3_vco", NULL);
	clk_set_rate(pll3_vco, pll3_vco_default);

	pll3 = mmp_clk_register_pll("pll3", "pll3_vco", CLK_SET_RATE_PARENT,
		0, &pll3_lock, &pll3_pll_params);
	clk_register_clkdev(pll3, "pll3", NULL);

	pll3p = mmp_clk_register_pll("pll3p", "pll3_vco", CLK_SET_RATE_PARENT,
		0, &pll3_lock, &pll3_pllp_params);
	clk_register_clkdev(pll3p, "pll3p", NULL);
	clk_set_rate(pll3, pll3_default);
	clk_set_rate(pll3p, pll3p_default);
}

struct core_reg_offset pxa988_reg_off = {
	/* clk src sel status register */
	.ap_pll_sel_shift = 2,
	.ap_pll_sel_width = 2,
	/* core clk src sel set register */
	.clk_sel_shift = 29,
	.clk_sel_width = 3,
	/* reg offset reserved for device tree */
	.clksrcsel_off = 0x0008,
	.clksrcst_off = 0x00c4,
	.clkdivfctrg_off = 0x0004,
	.clkdivst_off = 0x000c,
	.cpdivfctrg_off = 0x0000,
	.perisrcsel_off = 0x0100,
	.peridiv_off = 0x0104,
	.isr_off = 0x00a0,
	.pll3cr_off = 0x001c,
	.cpu_conf0_off = 0x00c8,
	.cpu_conf1_off = 0x00cc,
};

static struct cpu_rtcwtc cpu_rtcwtc_ax[] = {
	{.max_pclk = 800, .l1_rtc = 0x88888888, .l2_rtc = 0x00008444,},
	{.max_pclk = 1066, .l1_rtc = 0x99999999, .l2_rtc = 0x00009555,},
	{.max_pclk = 1205, .l1_rtc = 0xAAAAAAAA, .l2_rtc = 0x0000A555,},
};

struct core_parents_table parent_table[] = {
	{
		.parent_name = "pll1_624",
		.hw_sel_val = AP_CLK_SRC_PLL1_624,
	},
	{
		.parent_name = "pll1_1248",
		.hw_sel_val = AP_CLK_SRC_PLL1_1248,
	},
	{
		.parent_name = "pll2",
		.hw_sel_val = AP_CLK_SRC_PLL2,
	},
	{
		.parent_name = "pll2p",
		.hw_sel_val = AP_CLK_SRC_PLL2P,
	},
	/* Since hw_sel_val = 0x1 can select pll1_1248 or pll3p,
	   we use bit 1 to indicate src selection of pll3p */
	{
		.parent_name = "pll3p",
		.hw_sel_val = AP_CLK_SRC_PLL3P,
	},
};
static const char *core_parent[] = {"pll1_624", "pll1_1248", "pll2", "pll2p",
					"pll3p",};

/*
 * For 988:
 * L2CLK = PCLK / (L2_CLK_DIV +1)
 * BIU_CLK = L2_CLK / (BIU_CLK_DIV +1)
 * MC_CLK = L2_CLK / (MC_CLK_DIV +1)
 * Emei Ax:
 * PERIPHCLK = PCLK /4 * (PERI_CLK_DIV+1)
 *
 * FIXME:
 * 1. pdclk/paclk can use 1:1 with l2clk when in low speed,
 *    and 1:2 when pclk is in high speed
 * 2.It is better to select a lower frequency for power saving
 *   since it does NOT have very higher frequency requirement.
 *   Current DE suggests to use pclk/8 as PERIPHCLK.
 * 3.For Emei Ax, PERIPHCLK divider is from 4~32.
 */

/* The PP table only list the possible op here */
static struct cpu_opt pxa988_op_array_z3ax[] = {
	{
		.pclk = 312,
		.l2clk = 312,
		.pdclk = 156,
		.baclk = 156,
		.periphclk = 39,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 624,
		.l2clk = 312,
		.pdclk = 312,
		.baclk = 156,
		.periphclk = 78,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 800,
		.l2clk = 400,
		.pdclk = 400,
		.baclk = 200,
		.periphclk = 100,
		.ap_clk_sel = AP_CLK_SRC_PLL2P,
	},
	{
		.pclk = 1066,
		.l2clk = 533,
		.pdclk = 533,
		.baclk = 266,
		.periphclk = 133,
		.ap_clk_sel = AP_CLK_SRC_PLL2P,
	},
	{
		.pclk = 1205,
		.l2clk = 602,
		.pdclk = 602,
		.baclk = 301,
		.periphclk = 150,
		.ap_clk_sel = AP_CLK_SRC_PLL3P,
	},
	{
		.pclk = 1248,
		.l2clk = 624,
		.pdclk = 624,
		.baclk = 312,
		.periphclk = 156,
		.ap_clk_sel = AP_CLK_SRC_PLL1_1248,
	},
};

/* core,ddr,axi clk div and fc trigger register description */
union pmua_cc {
	struct {
		unsigned int core_clk_div:3;
		unsigned int bus_mc_clk_div:3;
		unsigned int biu_clk_div:3;
		unsigned int l2_clk_div:3;
		unsigned int ddr_clk_div:3;
		unsigned int bus_clk_div:3;
		unsigned int async1:1;
		unsigned int async2:1;
		unsigned int async3:1;
		unsigned int async3_1:1;
		unsigned int async4:1;
		unsigned int async5:1;
		unsigned int core_freq_chg_req:1;
		unsigned int ddr_freq_chg_req:1;
		unsigned int bus_freq_chg_req:1;
		unsigned int core_allow_spd_chg:1;
		unsigned int core_dyn_fc:1;
		unsigned int dclk_dyn_fc:1;
		unsigned int aclk_dyn_fc:1;
		unsigned int core_rd_st_clear:1;
	} b;
	unsigned int v;
};

static void __init __init_fc_setting(void *apmu_base)
{
	unsigned int regval;
	union pmua_cc cc_ap, cc_cp;
#define APMU_IMR	(apmu_base + 0x0098)
#define APMU_CP_CCR	(apmu_base + 0x0000)
#define APMU_CCR	(apmu_base + 0x0004)
#define APMU_DEBUG	(apmu_base + 0x0088)
	/*
	 * enable AP FC done interrupt for one step,
	 * while not use three interrupts by three steps
	 */
	__raw_writel((1 << 1), APMU_IMR);

	/* always vote for CP allow AP FC */
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	regval = __raw_readl(APMU_DEBUG);
	/* CA9 doesn't support halt acknowledge, mask it */
	regval |= (1 << 1);
	/*
	 * Always set AP_WFI_FC and CP_WFI_FC, then PMU will
	 * automaticlly send out clk-off ack when core is WFI
	 */
	regval |= (1 << 21) | (1 << 22);
	/*
	 * mask CP clk-off ack and cp halt ack for DDR/AXI FC
	 * this bits should be unmasked after cp is released
	 */
	regval |= (1 << 0) | (1 << 3);
	__raw_writel(regval, APMU_DEBUG);

	/*
	 * Always use async for DDR, AXI interface,
	 * and always vote for AP allow FC
	 */
	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.async5 = 1;
	cc_ap.b.async4 = 1;
	cc_ap.b.async3_1 = 1;
	cc_ap.b.async3 = 1;
	cc_ap.b.async2 = 1;
	cc_ap.b.async1 = 1;
	cc_ap.b.core_allow_spd_chg = 1;
	__raw_writel(cc_ap.v, APMU_CCR);
}

struct core_params pxa988_core_params = {
	.core_offset = &pxa988_reg_off,
	.parent_table = parent_table,
	.parent_table_size = ARRAY_SIZE(parent_table),
	.cpu_opt = pxa988_op_array_z3ax,
	.cpu_opt_size = ARRAY_SIZE(pxa988_op_array_z3ax),
	.cpu_rtcwtc_table = cpu_rtcwtc_ax,
	.cpu_rtcwtc_table_size = ARRAY_SIZE(cpu_rtcwtc_ax),
	.max_cpurate = 1205,
};
static DEFINE_SPINLOCK(fc_seq_lock);

void __init pxa988_acpu_init(void *apmu_base, void *mpmu_base, void *ciu_base)
{
	struct clk *clk;
	__init_fc_setting(apmu_base);
	pxa988_core_params.apmu_reg = apmu_base;
	pxa988_core_params.mpmu_reg = mpmu_base;
	pxa988_core_params.ciu_reg = ciu_base;
	clk = mmp_clk_register_core("cpu", core_parent, ARRAY_SIZE(core_parent),
		CLK_GET_RATE_NOCACHE, MMP_CORE_PLL3_SEL, &fc_seq_lock,
		&pxa988_core_params);
	clk_register_clkdev(clk, "cpu", NULL);
	clk_prepare_enable(clk);
}

void __init pxa910_clk_init(void)
{
	struct clk *clk;
	struct clk *uart_pll;
	void __iomem *mpmu_base;
	void __iomem *apmu_base;
	void __iomem *apbcp_base;
	void __iomem *apbc_base;
	void __iomem *disp_div;
	void __iomem *apbs_base;
	void __iomem *ciu_base;

	mpmu_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	if (mpmu_base == NULL) {
		pr_err("error to ioremap MPMU base\n");
		return;
	}

	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	ciu_base = ioremap(AXI_PHYS_BASE + 0x82c00, SZ_4K);
	if (ciu_base == NULL) {
		pr_err("error to ioremap CIU base\n");
		return;
	}

	apbcp_base = ioremap(APB_PHYS_BASE + 0x3b000, SZ_4K);
	if (apbcp_base == NULL) {
		pr_err("error to ioremap APBC extension base\n");
		return;
	}

	apbc_base = ioremap(APB_PHYS_BASE + 0x15000, SZ_4K);
	if (apbc_base == NULL) {
		pr_err("error to ioremap APBC base\n");
		return;
	}

	apbs_base = ioremap(APB_PHYS_BASE + 0x90000, SZ_4K);
	if (apbs_base == NULL) {
		pr_err("error to ioremap APB_spare base\n");
		return;
	}

	disp_div = ioremap(LCD_PN_SCLK, 4);
	if (disp_div == NULL) {
		pr_err("error to ioremap disp div\n");
		return;
	}

	clk = clk_register_fixed_rate(NULL, "clk32", NULL, CLK_IS_ROOT, 3200);
	clk_register_clkdev(clk, "clk32", NULL);

	clk = clk_register_fixed_rate(NULL, "vctcxo", NULL, CLK_IS_ROOT,
				26000000);
	clk_register_clkdev(clk, "vctcxo", NULL);

	pxa988_pll_init(mpmu_base, apbs_base);

	pxa988_acpu_init(apmu_base, mpmu_base, ciu_base);

	uart_pll =  mmp_clk_register_factor("uart_pll", "pll1_4", 0,
				mpmu_base + MPMU_UART_PLL,
				&uart_factor_masks, uart_factor_tbl,
				ARRAY_SIZE(uart_factor_tbl));
	clk_set_rate(uart_pll, 14745600);
	clk_register_clkdev(uart_pll, "uart_pll", NULL);

	clk = mmp_clk_register_apbc("twsi0", "pll1_13_1_5",
				apbc_base + APBC_TWSI0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.0");

	clk = mmp_clk_register_apbc("twsi1", "pll1_13_1_5",
				apbcp_base + APBCP_TWSI1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.1");

	clk = mmp_clk_register_apbc("gpio", "vctcxo",
				apbc_base + APBC_GPIO, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-gpio");

	clk = mmp_clk_register_apbc("kpc", "clk32",
				apbc_base + APBC_KPC, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa27x-keypad");

	clk = mmp_clk_register_apbc("rtc", "clk32",
				apbc_base + APBC_RTC, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "sa1100-rtc");

	clk = mmp_clk_register_apbc("pwm0", "pll1_48",
				apbc_base + APBC_PWM0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa910-pwm.0");

	clk = mmp_clk_register_apbc("pwm1", "pll1_48",
				apbc_base + APBC_PWM1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa910-pwm.1");

	clk = mmp_clk_register_apbc("pwm2", "pll1_48",
				apbc_base + APBC_PWM2, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa910-pwm.2");

	clk = mmp_clk_register_apbc("pwm3", "pll1_48",
				apbc_base + APBC_PWM3, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa910-pwm.3");

	clk = clk_register_mux(NULL, "uart0_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART0, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, uart_pll);
	clk_register_clkdev(clk, "uart_mux.0", NULL);

	clk = mmp_clk_register_apbc("uart0", "uart0_mux",
				apbc_base + APBC_UART0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.0");

	clk = clk_register_mux(NULL, "uart1_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART1, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, uart_pll);
	clk_register_clkdev(clk, "uart_mux.1", NULL);

	clk = mmp_clk_register_apbc("uart1", "uart1_mux",
				apbc_base + APBC_UART1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.1");

	clk = clk_register_mux(NULL, "uart2_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbcp_base + APBCP_UART2, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, uart_pll);
	clk_register_clkdev(clk, "uart_mux.2", NULL);

	clk = mmp_clk_register_apbc("uart2", "uart2_mux",
				apbcp_base + APBCP_UART2, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.2");

	clk = clk_register_mux(NULL, "ssp0_mux", ssp_parent,
				ARRAY_SIZE(ssp_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_SSP0, 4, 3, 0, &clk_lock);
	clk_register_clkdev(clk, "uart_mux.0", NULL);

	clk = mmp_clk_register_apbc("ssp0", "ssp0_mux",
				apbc_base + APBC_SSP0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-ssp.0");

	clk = clk_register_mux(NULL, "ssp1_mux", ssp_parent,
				ARRAY_SIZE(ssp_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_SSP1, 4, 3, 0, &clk_lock);
	clk_register_clkdev(clk, "ssp_mux.1", NULL);

	clk = mmp_clk_register_apbc("ssp1", "ssp1_mux",
				apbc_base + APBC_SSP1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-ssp.1");

	clk = mmp_clk_register_apmu("dfc", "pll1_4",
				apmu_base + APMU_DFC, 0x19b, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa3xx-nand.0");

	clk = clk_register_mux(NULL, "sdh0_mux", sdh_parent,
				ARRAY_SIZE(sdh_parent), CLK_SET_RATE_PARENT,
				apmu_base + APMU_SDH0, 6, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "sdh0_mux", NULL);

	clk = mmp_clk_register_apmu("sdh0", "sdh0_mux",
				apmu_base + APMU_SDH0, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.0");

	clk = clk_register_mux(NULL, "sdh1_mux", sdh_parent,
				ARRAY_SIZE(sdh_parent), CLK_SET_RATE_PARENT,
				apmu_base + APMU_SDH1, 6, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "sdh1_mux", NULL);

	clk = mmp_clk_register_apmu("sdh1", "sdh1_mux",
				apmu_base + APMU_SDH1, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.1");

	clk = clk_register_mux(NULL, "sdh2_mux", sdh_parent,
			ARRAY_SIZE(sdh_parent), CLK_SET_RATE_PARENT,
			apmu_base + APMU_SDH2, 6, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "sdh2_mux", NULL);

	clk = mmp_clk_register_apmu("sdh2", "sdh2_mux",
			apmu_base + APMU_SDH2, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.2");

	clk = mmp_clk_register_apmu("usb", "usb_pll",
				apmu_base + APMU_USB, 0x9, &clk_lock);
	clk_register_clkdev(clk, NULL, "mv-otg");
	clk_register_clkdev(clk, NULL, "mv-udc");
	clk_register_clkdev(clk, NULL, "mv-ehci");
	clk_register_clkdev(clk, NULL, "pxa988-usb-phy");

	clk = mmp_clk_register_apmu("sph", "usb_pll",
				apmu_base + APMU_USB, 0x12, &clk_lock);
	clk_register_clkdev(clk, "sph_clk", NULL);

	clk = clk_register_mux(NULL, "disp0_mux", disp_parent,
				ARRAY_SIZE(disp_parent), CLK_SET_RATE_PARENT,
				apmu_base + APMU_DISP0, 6, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "disp_mux.0", NULL);

	clk = mmp_clk_register_apmu("disp_pn_sclk", "disp0_mux",
				apmu_base + APMU_DISP0, 0x7f, &clk_lock);
	clk_register_clkdev(clk, "disp_pn_sclk", NULL);

	clk = clk_register_divider(NULL, "mmp_dsi1", "disp_pn_sclk",
				CLK_SET_RATE_PARENT, disp_div,
				8, 4, 0, &clk_lock);
	clk_register_clkdev(clk, "mmp_dsi1", NULL);

	clk = clk_register_divider(NULL, "mmp_pnpath", "disp_pn_sclk",
				CLK_SET_RATE_PARENT, disp_div,
				0, 8, 0, &clk_lock);
	clk_register_clkdev(clk, "mmp_pnpath", NULL);

	clk = mmp_clk_register_apmu("lcdcih", "disp0_mux",
				apmu_base + APMU_DISP0, 0x0, &clk_lock);
	clk_register_clkdev(clk, "LCDCIHCLK", NULL);

	clk = clk_register_mux(NULL, "ccic0_mux", ccic_parent,
				ARRAY_SIZE(ccic_parent), CLK_SET_RATE_PARENT,
				apmu_base + APMU_CCIC0, 6, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "ccic_mux.0", NULL);

	clk = mmp_clk_register_apmu("ccic0", "ccic0_mux",
				apmu_base + APMU_CCIC0, 0x1b, &clk_lock);
	clk_register_clkdev(clk, "fnclk", "mmp-camera.0");

	clk = mmp_clk_register_apmu("ccic0_axi", "ccic0_mux",
				apmu_base + APMU_CCIC0, 0x9, &clk_lock);
	clk_register_clkdev(clk, "ccic_axi", NULL);

	clk = clk_register_mux(NULL, "ccic0_fn_mux", ccic_fn_parent,
				ARRAY_SIZE(ccic_fn_parent),
				CLK_SET_RATE_PARENT, apmu_base + APMU_CCIC0,
				16, 2, 0, &clk_lock);
	clk_register_clkdev(clk, "ccic_fn_mux", NULL);

	clk = clk_register_divider(NULL, "ccic0_fn_div", "ccic0_fn_mux",
				CLK_SET_RATE_PARENT, apmu_base + APMU_CCIC0,
				18, 3, 0, &clk_lock);
	clk_register_clkdev(clk, "ccic_fn_div", NULL);

	clk = mmp_clk_register_apmu("ccic0_fn", "ccic_fn_mux",
				apmu_base + APMU_CCIC0, 0x12, &clk_lock);
	clk_register_clkdev(clk, "ccic_fn", NULL);

	clk = clk_register_mux(NULL, "ccic0_phy_mux", ccic_phy_parent,
				ARRAY_SIZE(ccic_phy_parent),
				CLK_SET_RATE_PARENT, apmu_base + APMU_CCIC0,
				7, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "ccic_phy_mux", NULL);

	clk = mmp_clk_register_apmu("ccic0_phy", "ccic0_phy_mux",
				apmu_base + APMU_CCIC0, 0x24, &clk_lock);
	clk_register_clkdev(clk, "phyclk", "mmp-camera.0");

	clk = clk_register_divider(NULL, "ccic0_sphy_div", "vctcxo",
				CLK_SET_RATE_PARENT, apmu_base + APMU_CCIC0,
				10, 5, 0, &clk_lock);
	clk_register_clkdev(clk, "sphyclk_div", NULL);

	clk = mmp_clk_register_apmu("ccic0_sphy", "ccic0_sphy_div",
				apmu_base + APMU_CCIC0, 0x300, &clk_lock);
	clk_register_clkdev(clk, "sphyclk", "mmp-camera.0");
}
