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
#include <linux/features.h>
#include <plat/debugfs.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>
#include <linux/clk/mmpdcstat.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <video/mmp_disp.h>
#include "clk.h"

#define APBC_RTC	0x28
#define APBC_TWSI0	0x2c
#define APBC_KPC	0x30
#define APBC_TWSI1	0x60
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
#define APBC_TERMAL	0x6c
#define APBCP_TWSI2	0x28
#define APBCP_UART2	0x1c
#define APMU_SDH0	0x54
#define APMU_SDH1	0x58
#define APMU_SDH2	0xe0
#define APMU_USB	0x5c
#define APMU_DSI1	0x44
#define APMU_DISP1	0x4c
#define APMU_CCIC0	0x50
#define APMU_CCIC1	0x24
#define APMU_DFC	0x60
#define APMU_PLL_SEL	0xc4
#define MPMU_PLL2CR	0x34
#define MPMU_PLL3CR	0x1c
#define MPMU_UART_PLL	0x14
#define MPMU_VRCR	0x18
#define MPMU_FCCR	0x8
#define MPMU_ISCCR0	0x40
#define MPMU_ISCCR1	0x44
#define APMU_VPU_CLK_RES_CTRL	0xa4
#define APMU_GC		0xcc
#define APMU_GC2D	0xf4
#define APMU_LCD	0x04c
#define APMU_DSI	0x044
#define APMU_ISP	0x038
#define APMU_TRACE	0x108
#define APMU_GEU	0x68
#define CIU_MC_CONF	0x0040
#define APMU_CORE_STATUS 0x090
#define APMU_AUD_CLK_RES_CTRL	0x80

/* GBS: clock for GSSP */
#define APBC_GBS       0xc
#define APBC_GCER      0x34
#define APBC_DROTS	0x058

/* IPC/RIPC clock */
#define APBC_IPC_CLK_RST	0x24
#define APBCP_AICER			0x38

/* pll1 d1p gating feature */
#define APMU_CLK_GATE_CTRL	0x040
static DEFINE_SPINLOCK(d1p_gate_lock);

/* RTC/WTC register */
#define VPU_XTC		0x00a8
#define GPU2D_XTC	0x00a0
#define GPU_XTC		0x00a4

/* use to save some important clk ptr */
enum helanx_clk {
	cpu = 0, ddr, axi, gc_aclk,
	gc, gc2d, gcshader = 5, vpu_aclk,
	vpu, isp, ispccicaclk,
	PLL2_VCO, PLL2, PLL2P, PLL2_DIV3,
	PLL3_VCO, PLL3, PLL3P, PLL3_DIV3,
	PLL4_VCO, PLL4, PLL4P, PLL4_DIV3,
	clk_max,
};
static struct clk *clks[clk_max];


static DEFINE_SPINLOCK(clk_lock);
static DEFINE_SPINLOCK(disp_lock);
static DEFINE_SPINLOCK(trace_lock);
static DEFINE_SPINLOCK(rtc_lock);

#define LCD_PN_SCLK	(0xd420b1a8)

enum {
	CORE_1p2G = 1183,
	CORE_1p5G = 1482,
};

static unsigned long pll2_vco_default;
static unsigned long pll2_default;
static unsigned long pll2p_default;
static unsigned long pll3_vco_default;
static unsigned long pll3_default;
static unsigned long pll3p_default;
static unsigned long pll4_vco_default;
static unsigned long pll4_default;
static unsigned long pll4p_default;

/* parameter passed from cmdline to identify DDR mode */
static int ddr_mode;
static int __init __init_ddr_mode(char *arg)
{
	int n;
	if (!get_option(&arg, &n))
		return 0;
	if ((n != 0) && (n != 1))
		pr_info("WARNING: unknown DDR type!");
	else
		ddr_mode = n;

	return 1;
}
__setup("ddr_mode=", __init_ddr_mode);

static unsigned long max_freq;
static int __init max_freq_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_freq = n;
	return 1;
}
__setup("max_freq=", max_freq_setup);

/*
 * uboot pass pll3_vco = xxx (unit is Mhz), it is used
 * to distinguish whether core and display can share pll3.
 * if core can use it, core max_freq is setup, else core
 * will not use pll3p, pll3 will only be used by display
*/
static int __init pll3_vco_value_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	pll3_vco_default = n * MHZ_TO_HZ;
	return 1;
}
__setup("pll3_vco=", pll3_vco_value_setup);

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

static struct clk_factor_masks isccr1_factor_masks = {
	.factor = 1,
	.den_mask = 0xfff,
	.num_mask = 0x7fff,
	.den_shift = 15,
	.num_shift = 0,
};

static struct clk_factor_tbl isccr1_factor_tbl[] = {
	{.num = 1625, .den = 256},	/*8kHz*/
	{.num = 3042, .den = 1321},	/*44.1kHZ */
};

static const struct clk_div_table clk_ssp1_ref_table[] = {
	{ .val = 0, .div = 2 },
	{ .val = 1, .div = 4 },
	{ .val = 2, .div = 6 },
	{ .val = 3, .div = 8 },
	{ .val = 0, .div = 0 },
};

static const char *uart_parent[] = {"pll1_3_16", "uart_pll"};
static const char *ssp_parent[] = {"pll1_96", "pll1_48", "pll1_24", "pll1_12"};
static const char *ssp1_parent[] = {"pll1_2", "vctcxo"};
static const char *gssp_parent[] = {"isccr0_i2sclk", "sys clk",
					"ext clk", "vctcxo"};
static const char *disp1_parent[] = {"pll1_624", "pll1_416m"};
static const char *dispaxi_parent[] = {"lcd_ci_isp_axi"};
static const char *dsipll_parent1[] = {"pll3"};
static const char *dsipll_parent2[] = {"pll3", "pll2", "pll1_832m"};

static const char *pnsclk_parent[] = {"disp1", "dsi_pll"};
static const char *pnpath_parent[] = {"pn_sclk"};
static const char *pnsclk_depend[] = {"LCDCIHCLK"};
static const char *dsi_depend[] = {"dsi_phy_slow", "LCDCIHCLK"};
static u32 pnsclk_parent_tbl[] = {1, 7};


/* clk source of ccic phy clk: 104Mhz and 52Mhz */
static const char *ccic_phy_parent[] = {"pll1_6", "pll1_12"};

/*
 * ccic_fn_parent:
 * CCIC function clk select use APMU_CCIC1[16:17] & APMU_CCIC1[21]
 * 0x00: 416MHz
 * 0x01: 624MHz
 * 0x02: PLL2_DIV3
 * 0x22: PLL2P
 * 0x03: PLL4_DIV3
 */
static const char *ccic_fn_parent[] = {"pll1_416_gate", "pll1_624_gate",
					"pll2_div3", "pll2p", "pll4_div3"};

/*
 * ccic4_fn_parent:
 * CCIC function clk select use APMU_CCIC0[16:17] & APMU_CCIC0[23]
 * 0x00: 832MHz
 * 0x01: 624MHz
 * 0x02: PLL2_DIV3
 * 0x82: PLL2P
 * 0x03: PLL4_DIV3
 */
static const char *ccic4_fn_parent[] = {"pll1_832_gate", "pll1_624_gate",
					"pll2_div3", "pll2p", "pll4_div3"};
#define POSR_PLL2_LOCK		(1 << 29)
#define POSR_PLL3_LOCK		(1 << 30)
#define POSR_PLL4_LOCK		(1 << 31)
static DEFINE_SPINLOCK(pll2_lock);
static DEFINE_SPINLOCK(pll3_lock);
static DEFINE_SPINLOCK(pll4_lock);

static struct kvco_range kvco_rng_table[] = {
	{2600, 3000, 0xf, 0},
	{2400, 2600, 0xe, 0},
	{2200, 2400, 0xd, 0},
	{2000, 2200, 0xc, 0},
	{1750, 2000, 0xb, 0},
	{1500, 1750, 0xa, 0},
	{1350, 1500, 0x9, 0},
	{1200, 1350, 0x8, 0},
};
/* PLL post divider table */
static struct div_map pll_post_div_tbl[] = {
	/* divider, reg vaule */
	{1, 0x0},
	{2, 0x1},
	{4, 0x2},
	{8, 0x3},
	{16, 0x4},
	{32, 0x5},
	{64, 0x6},
	{128, 0x7},
};

static struct pll_offset pll2_offset = {
	.fbd_shift = 10,
	.fbd_width = 9,
	.refd_shift = 19,
	.refd_width = 5,
	.kvco_shift = 11,
	.kvco_width = 4,
};

static struct pll_offset pllx_offset = {
	.fbd_shift = 5,
	.fbd_width = 9,
	.refd_shift = 0,
	.refd_width = 5,
	.kvco_shift = 11,
	.kvco_width = 4,
};

static struct mmp_vco_params pll2_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 3000000000UL,
	.enable_bit = 0x100,
	.ctrl_bit = 0x200,
	.lock_enable_bit = POSR_PLL2_LOCK,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pll2_offset,
};

static struct mmp_vco_params pll3_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 3000000000UL,
	.enable_bit = 0x80000,
	.ctrl_bit = 0x80000,
	.lock_enable_bit = POSR_PLL3_LOCK,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pllx_offset,
};

static struct mmp_vco_params pll4_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 3000000000UL,
	.enable_bit = 0x80000,
	.ctrl_bit = 0x80000,
	.lock_enable_bit = POSR_PLL4_LOCK,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pllx_offset,
};

static struct mmp_pll_params pll2_pll_params = {
	.div_shift = 20,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll2_pllp_params = {
	.div_shift = 17,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll3_pll_params = {
	.div_shift = 20,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll3_pllp_params = {
	.div_shift = 17,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll4_pll_params = {
	.div_shift = 20,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

static struct mmp_pll_params pll4_pllp_params = {
	.div_shift = 17,
	.div_width = 3,
	.div_map = pll_post_div_tbl,
	.div_map_size = ARRAY_SIZE(pll_post_div_tbl),
};

#define LCD_PST_CKEN            (1 << 9)
#define LCD_PST_OUTDIS          (1 << 8)
#define LCD_CLK_EN              (1 << 5 | 1 << 4 | 1 << 3)
#define LCD_CLK_RST             (1 << 1 | 1 << 0)
#define DSI_PHYSLOW_PRER        (0x1A << 6)
#define DSI_ESC_SEL             (0x0)
#define DSI_PHYESC_SELDIV       \
	(DSI_PHYSLOW_PRER | DSI_ESC_SEL)
#define DSI_PHYESC_SELDIV_MSK   ((0x1f << 6) | 0x3)
#define DSI_PHY_CLK_EN  ((1 << 2) | (1 << 5))
#define DSI_PHY_CLK_RST ((1 << 3) | (1 << 4))

static struct mmp_clk_disp disp_axi = {
	.reg_rst = APMU_DISP1,
	.reg_rst_shadow = LCD_PST_CKEN |
			  LCD_CLK_EN   |
			  LCD_CLK_RST,
	.reg_rst_mask = LCD_PST_CKEN   |
			LCD_PST_OUTDIS |
			LCD_CLK_EN     |
			LCD_CLK_RST,
	.lock = &disp_lock,
};

static struct mmp_clk_disp dsi_phy_slow = {
	.reg_rst = APMU_DSI1,
	.reg_rst_shadow = DSI_PHYESC_SELDIV |
			  DSI_PHY_CLK_EN    |
			  DSI_PHY_CLK_RST,
	.reg_rst_mask = DSI_PHYESC_SELDIV_MSK |
			DSI_PHY_CLK_EN        |
			DSI_PHY_CLK_RST,
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp1 = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 1,
	.mux.shift = 6,
	.mux.lock = &disp_lock,
	.reg_mux = APMU_DISP1,
	.div_ops = &clk_divider_ops,
	.divider.width = 5,
	.divider.shift = 10,
	.divider.lock = &disp_lock,
	.reg_div = APMU_DISP1,
};

static struct mmp_clk_disp dsi_pll = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 3,
	.mux.shift = 24,
	.mux.lock = &disp_lock,
	.reg_mux = APMU_DISP1,
};

static struct mmp_clk_disp pnsclk = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 3,
	.mux.shift = 30,
	.mux.lock = &disp_lock,
	.mux.table = pnsclk_parent_tbl,
	.dependence = pnsclk_depend,
	.num_dependence = ARRAY_SIZE(pnsclk_depend),
	.reg_mux_shadow = 0x40000000,
};

static struct mmp_clk_disp pnpath = {
	.div_ops = &clk_divider_ops,
	.divider.width = 8,
	.divider.shift = 0,
	.divider.lock = &disp_lock,
	.dependence = pnsclk_depend,
	.num_dependence = ARRAY_SIZE(pnsclk_depend),
	.reg_div_shadow = 4,
};

static struct mmp_clk_disp dsi1 = {
	.div_ops = &clk_divider_ops,
	.divider.width = 4,
	.divider.shift = 8,
	.divider.lock = &disp_lock,
	.dependence = dsi_depend,
	.num_dependence = ARRAY_SIZE(dsi_depend),
	.reg_div_shadow = 0x100,
};

/*
 * max_freq could only be configured to predefined frequency
 * round it to just use platform allowed frequency.
 * For 1x88, it is 1183M or 1482M
*/
static inline void round_max_freq(void)
{
	if (!max_freq)
		max_freq = CORE_1p2G;

	if (max_freq <= CORE_1p2G)
		max_freq = CORE_1p2G;
	else
		max_freq = CORE_1p5G;
}
static inline void setup_pll3(void)
{
	/* LCD pll3 source can't be higher than 1Ghz */
	if (pll3_vco_default > 2000 * MHZ_TO_HZ)
		pll3_default = pll3_vco_default / 3;
	else if (pll3_vco_default > 1000 * MHZ_TO_HZ)
		pll3_default = pll3_vco_default / 2;
	else
		pll3_default = pll3_vco_default;
}

static inline int pll3_best_mul(unsigned long rate)
{
	int i;
	unsigned long vco;
	for (i = 0; i < ARRAY_SIZE(pll_post_div_tbl); i++) {
		vco = rate * pll_post_div_tbl[i].div;
		if ((vco > 1200 * MHZ_TO_HZ) &&
		    (vco < (unsigned long)2500 * MHZ_TO_HZ))
			break;
	}
	if (i == ARRAY_SIZE(pll_post_div_tbl)) {
		pr_err("Multiplier is out of range!\n");
		return -1;
	}
	return pll_post_div_tbl[i].div;
}

/*
 * uboot may pass pll3_vco value to kernel.
 * pll3 may be shared between core and display.
 * if it can be shared, core will use pll3p and it is set to max_freq
 * or pll3 and pll3p will be set to be the same with pll3_vco
*/
static inline void setup_pll3_vco(void)
{
	int bestmul;
	if (!pll3_vco_default) {
		bestmul = pll3_best_mul(max_freq * MHZ_TO_HZ);
		if (bestmul == -1) {
			pr_err("Cannot use PLL3 as max freq!\n");
			BUG_ON(1);
		}
		pll3_vco_default = max_freq * MHZ_TO_HZ * bestmul;
		pll3p_default = max_freq * MHZ_TO_HZ;
	} else {
		if ((pll3_vco_default < 1200 * MHZ_TO_HZ) ||
		   (pll3_vco_default > (unsigned long)2500 * MHZ_TO_HZ)) {
			pr_err("PLL3 VCO value is out of range !\n");
			BUG_ON(1);
		}
		if (!(pll3_vco_default % max_freq))
			pll3p_default = max_freq * MHZ_TO_HZ;
		else /* Core will not use pll3p */
			pll3p_default = pll3_vco_default;
	}
	setup_pll3();
}

#define PLL_INIT_MASK	((PLL_MASK(3) << 26) | (PLL_MASK(1) << 25) |\
	(PLL_MASK(1) << 24) | (PLL_MASK(1) << 23) | (PLL_MASK(2) << 15) |\
	(PLL_MASK(1) << 10) | (PLL_MASK(4) << 6) | (PLL_MASK(3) << 3) |\
	(PLL_MASK(2) << 1) | (PLL_MASK(1) << 0))

#define PLL_INIT	((4 << 26) | (0 << 25) | (0 << 24) | (1 << 23) |\
			(1 << 15) | (0 << 10) | (3 << 6) | (4 << 3) |\
			(1 << 1) | (1 << 0))
static void __init helan2_pll_init(void *mpmu_base, void *apbs_base,
				   void *apmu_base)
{
	u32 pll_init;
	struct clk *clk;
	struct clk *pll2_vco, *pll2, *pll2p, *pll3_vco,
		*pll3, *pll3p, *pll4_vco, *pll4, *pll4p;

	clk = clk_register_fixed_rate(NULL, "pll1_624", NULL, CLK_IS_ROOT,
				624000000);
	clk_register_clkdev(clk, "pll1_624", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_416m", NULL, CLK_IS_ROOT,
				416000000);
	clk_register_clkdev(clk, "pll1_416m", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_832m", NULL, CLK_IS_ROOT,
				832000000);
	clk_register_clkdev(clk, "pll1_832m", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_1248", NULL, CLK_IS_ROOT,
				1248000000);
	clk_register_clkdev(clk, "pll1_1248", NULL);

	/* pll1_416/624/832 clock which has d1p gating feature */
	clk = clk_register_gate(NULL, "pll1_416_gate", "pll1_416m",
			0, apmu_base + APMU_CLK_GATE_CTRL,
			27, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_416_gate", NULL);

	clk = clk_register_gate(NULL, "pll1_624_gate", "pll1_624",
			0, apmu_base + APMU_CLK_GATE_CTRL,
			26, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_624_gate", NULL);

	clk = clk_register_gate(NULL, "pll1_832_gate", "pll1_832m",
			0, apmu_base + APMU_CLK_GATE_CTRL,
			30, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_832_gate", NULL);

	clk = clk_register_gate(NULL, "pll1_1248_gate", "pll1_1248",
			0, apmu_base + APMU_CLK_GATE_CTRL,
			28, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_1248_gate", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_2", "pll1_624",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pll1_2", NULL);

	/* pll1_312 clock which has d1p gating feature */
	clk = clk_register_gate(NULL, "pll1_312_gate", "pll1_2",
			0, apmu_base + APMU_CLK_GATE_CTRL,
			29, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_312_gate", NULL);

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

	clk = clk_register_fixed_factor(NULL, "pll1_13_1_5", "pll1_13",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_13_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_2_1_5", "pll1_2",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_2_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_3_16", "pll1_624",
				CLK_SET_RATE_PARENT, 3, 16);
	clk_register_clkdev(clk, "pll1_3_16", NULL);
	if (ddr_mode) {
		pll2_vco_default = 2132 * MHZ_TO_HZ;
		pll2_default = 1066 * MHZ_TO_HZ;
		pll2p_default = 1066 * MHZ_TO_HZ;
	} else {
		pll2_vco_default = 1600 * MHZ_TO_HZ;
		pll2_default = 800 * MHZ_TO_HZ;
		pll2p_default = 800 * MHZ_TO_HZ;
	}
	pll3_vco_default = 2358lu * MHZ_TO_HZ;
	pll3_default = 1179 * MHZ_TO_HZ;
	pll3p_default = 1179 * MHZ_TO_HZ;
	pll4_vco_default = 2668lu * MHZ_TO_HZ;
	pll4_default = 1334 * MHZ_TO_HZ;
	pll4p_default = 667 * MHZ_TO_HZ;

	/* PLL2_vco PLL2 PLL2P */
	pll2_vco_params.cr_reg = mpmu_base + 0x0034;
	pll2_vco_params.pll_swcr = apbs_base + 0x0104;
	pll2_vco_params.lock_reg = mpmu_base + 0x0010;
	pll2_vco_params.default_vco_rate = pll2_vco_default;
	pll2_pll_params.pll_swcr = apbs_base + 0x0104;
	pll2_pllp_params.pll_swcr = apbs_base + 0x0104;
	pll_init = __raw_readl(pll2_pll_params.pll_swcr);
	pll_init &= ~PLL_INIT_MASK;
	pll_init |= PLL_INIT;
	__raw_writel(pll_init, pll2_pll_params.pll_swcr);

	/* PLL3_vco PLL3 PLL3P */
	pll3_vco_params.cr_reg = mpmu_base + 0x001c;
	pll3_vco_params.pll_swcr = apbs_base + 0x0108;
	pll3_vco_params.lock_reg = mpmu_base + 0x0010;
	pll3_vco_params.default_vco_rate = pll3_vco_default;
	pll3_pll_params.pll_swcr = apbs_base + 0x0108;
	pll3_pllp_params.pll_swcr = apbs_base + 0x0108;
	pll_init = __raw_readl(pll3_pll_params.pll_swcr);
	pll_init &= ~PLL_INIT_MASK;
	pll_init |= PLL_INIT;
	__raw_writel(pll_init, pll3_pll_params.pll_swcr);

	/* PLL4_vco PLL4 PLL4P */
	pll4_vco_params.cr_reg = mpmu_base + 0x0050;
	pll4_vco_params.pll_swcr = apbs_base + 0x0124;
	pll4_vco_params.lock_reg = mpmu_base + 0x0010;
	pll4_vco_params.default_vco_rate = pll4_vco_default;
	pll4_pll_params.pll_swcr = apbs_base + 0x0124;
	pll4_pllp_params.pll_swcr = apbs_base + 0x0124;
	pll_init = __raw_readl(pll4_pll_params.pll_swcr);
	pll_init &= ~PLL_INIT_MASK;
	pll_init |= PLL_INIT;
	__raw_writel(pll_init, pll4_pll_params.pll_swcr);

	pll2_vco = mmp_clk_register_vco("pll2_vco", NULL, CLK_IS_ROOT,
		MMP_PLL_28NM | MMP_PLL_LOCK_SETTING, &pll2_lock,
		&pll2_vco_params, NULL, 0);
	clk_register_clkdev(pll2_vco, "pll2_vco", NULL);
	clk_set_rate(pll2_vco, pll2_vco_default);
	clks[PLL2_VCO] = pll2_vco;

	pll2 = mmp_clk_register_pll("pll2", "pll2_vco", 0,
		MMP_PLL_28NM, &pll2_lock, &pll2_pll_params);
	clk_register_clkdev(pll2, "pll2", NULL);
	clks[PLL2] = pll2;

	pll2p = mmp_clk_register_pll("pll2p", "pll2_vco", 0,
		MMP_PLL_28NM, &pll2_lock, &pll2_pllp_params);
	clk_register_clkdev(pll2p, "pll2p", NULL);
	clks[PLL2P] = pll2p;

	clk = clk_register_fixed_factor(NULL, "pll2_div3", "pll2_vco",
				CLK_SET_RATE_PARENT, 1, 3);
	clk_register_clkdev(clk, "pll2_div3", NULL);
	clks[PLL2_DIV3] = clk;
	clk_set_rate(pll2, pll2_default);
	clk_set_rate(pll2p, pll2p_default);

	pll3_vco = mmp_clk_register_vco("pll3_vco", NULL, CLK_IS_ROOT,
		MMP_PLL_28NM, &pll3_lock, &pll3_vco_params, NULL, 0);
	clk_register_clkdev(pll3_vco, "pll3_vco", NULL);
	clk_set_rate(pll3_vco, pll3_vco_default);
	clks[PLL3_VCO] = pll3_vco;

	pll3 = mmp_clk_register_pll("pll3", "pll3_vco", 0,
		MMP_PLL_28NM, &pll3_lock, &pll3_pll_params);
	clk_register_clkdev(pll3, "pll3", NULL);
	clks[PLL3] = pll3;

	pll3p = mmp_clk_register_pll("pll3p", "pll3_vco", 0,
		MMP_PLL_28NM, &pll3_lock, &pll3_pllp_params);
	clk_register_clkdev(pll3p, "pll3p", NULL);
	clks[PLL3P] = pll3p;

	clk = clk_register_fixed_factor(NULL, "pll3_div3", "pll3_vco",
				CLK_SET_RATE_PARENT, 1, 3);
	clk_register_clkdev(clk, "pll3_div3", NULL);
	clks[PLL3_DIV3] = clk;
	clk_set_rate(pll3, pll3_default);
	clk_set_rate(pll3p, pll3p_default);

	pll4_vco = mmp_clk_register_vco("pll4_vco", NULL, CLK_IS_ROOT,
		MMP_PLL_28NM, &pll4_lock, &pll4_vco_params, NULL, 0);
	clk_register_clkdev(pll4_vco, "pll4_vco", NULL);
	clk_set_rate(pll4_vco, pll4_vco_default);
	clks[PLL4_VCO] = pll4_vco;

	pll4 = mmp_clk_register_pll("pll4", "pll4_vco", 0,
		MMP_PLL_28NM, &pll4_lock, &pll4_pll_params);
	clk_register_clkdev(pll4, "pll4", NULL);
	clks[PLL4] = pll4;

	pll4p = mmp_clk_register_pll("pll4p", "pll4_vco", 0,
		MMP_PLL_28NM, &pll4_lock, &pll4_pllp_params);
	clk_register_clkdev(pll4p, "pll4p", NULL);
	clks[PLL4P] = pll4p;

	clk = clk_register_fixed_factor(NULL, "pll4_div3", "pll4_vco",
				CLK_SET_RATE_PARENT, 1, 3);
	clk_register_clkdev(clk, "pll4_div3", NULL);
	clks[PLL4_DIV3] = clk;
	clk_set_rate(pll4, pll4_default);
	clk_set_rate(pll4p, pll4p_default);
}

static struct clk *lcd_get_parent(char *panel_type, char *item, char *df_parent)
{
#ifdef CONFIG_OF
	struct device_node *np = of_find_node_by_name(NULL, panel_type);
	struct clk *parent = NULL;
	const char *str = NULL;
	if (np && !of_property_read_string(np, item, &str))
		parent = clk_get(NULL, str);

	return !IS_ERR(parent) ? parent : clk_get(NULL, df_parent);
#else
	return clk_get(NULL, df_parent);
#endif
}

static void __init pxa988_disp_clk_init(void __iomem *apmu_base)
{
	struct clk *clk, *parent_clk;
	char *panel_type = mmp_get_paneltype();

	clk = mmp_clk_register_disp("LCDCIHCLK", dispaxi_parent,
			ARRAY_SIZE(dispaxi_parent),
			MMP_DISP_BUS, 0, apmu_base, &disp_axi);
	clk_register_clkdev(clk, "LCDCIHCLK", NULL);

	clk = mmp_clk_register_disp("dsi_phy_slow", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, apmu_base,
			&dsi_phy_slow);
	clk_register_clkdev(clk, "dsi_phy_slow", NULL);

	clk = mmp_clk_register_disp("disp1", disp1_parent,
			ARRAY_SIZE(disp1_parent), 0, CLK_SET_RATE_PARENT,
			apmu_base, &disp1);
	clk_register_clkdev(clk, "disp1", NULL);
	parent_clk = lcd_get_parent(panel_type, "disp1_clksrc", "pll1_416m");
	if (!IS_ERR(parent_clk))
		clk_set_parent(clk, parent_clk);

	if (has_feat_lcd_more_source())
		clk = mmp_clk_register_disp("dsi_pll", dsipll_parent2,
			ARRAY_SIZE(dsipll_parent2), MMP_DISP_MUX_ONLY,
			CLK_SET_RATE_PARENT, apmu_base, &dsi_pll);
	else
		clk = mmp_clk_register_disp("dsi_pll", dsipll_parent1,
			ARRAY_SIZE(dsipll_parent1), MMP_DISP_MUX_ONLY,
			CLK_SET_RATE_PARENT, apmu_base, &dsi_pll);
	clk_register_clkdev(clk, "dsi_pll", NULL);
	parent_clk = lcd_get_parent(panel_type, "dsipll_clksrc", "pll3");
	if (!IS_ERR(parent_clk))
		clk_set_parent(clk, parent_clk);

	pnsclk.mux.reg = ioremap(LCD_PN_SCLK, 4);
	clk = mmp_clk_register_disp("pn_sclk", pnsclk_parent,
			ARRAY_SIZE(pnsclk_parent),
			MMP_DISP_MUX_ONLY, CLK_SET_RATE_PARENT, apmu_base,
			&pnsclk);
	clk_register_clkdev(clk, "pn_sclk", NULL);
	parent_clk = lcd_get_parent(panel_type, "pn_sclk_clksrc", "disp1");
	if (!IS_ERR(parent_clk))
		clk_set_parent(clk, parent_clk);

	pnpath.divider.reg = ioremap(LCD_PN_SCLK, 4);
	clk = mmp_clk_register_disp("mmp_pnpath", pnpath_parent,
			ARRAY_SIZE(pnpath_parent),
			MMP_DISP_DIV_ONLY, 0, apmu_base, &pnpath);
	clk_register_clkdev(clk, "mmp_pnpath", NULL);

	dsi1.divider.reg = ioremap(LCD_PN_SCLK, 4);
	clk = mmp_clk_register_disp("mmp_dsi1", pnpath_parent,
			ARRAY_SIZE(pnpath_parent),
			MMP_DISP_DIV_ONLY, CLK_SET_RATE_PARENT, apmu_base,
			&dsi1);
	clk_register_clkdev(clk, "mmp_dsi1", NULL);
}

/* helan2 uses MPMU_FCAP to set clk src */
static struct core_reg_offset corefc_reg_off = {
	/* core clk src sel set register */
	.fcap_off = 0x0054,
	.clk_sel_shift = 0,
	.clk_sel_width = 3,
	.pll1_pll3_swbit = BIT(8),
};

static struct cpu_rtcwtc cpu_rtcwtc_1x88[] = {
	{.max_pclk = 624, .l1_rtc = 0x02222222, .l2_rtc = 0x00002221,},
	{.max_pclk = 1066, .l1_rtc = 0x02666666, .l2_rtc = 0x00006265,},
	{.max_pclk = 1283, .l1_rtc = 0x2AAAAAA, .l2_rtc = 0x0000A2A9,},
	{.max_pclk = 1482, .l1_rtc = 0x02EEEEEE, .l2_rtc = 0x0000E2ED,},
};

static struct core_parents_table core_parent_table[] = {
	{
		.parent_name = "pll1_624",
		.hw_sel_val = 0x0,
	},
	{
		.parent_name = "pll1_1248",
		.hw_sel_val = 0x1,
	},
	{
		.parent_name = "pll2",
		.hw_sel_val = 0x2,
	},
	{
		.parent_name = "pll1_832m",
		.hw_sel_val = 0x3,
	},
	{
		.parent_name = "pll3p",
		.hw_sel_val = 0x5,
	},
};
static const char *core_parent[] = {"pll1_624", "pll1_1248", "pll2",
					"pll1_832m", "pll3p",};

/*
 * For HELAN2:
 * PCLK = AP_CLK_SRC / (CORE_CLK_DIV + 1)
 * BIU_CLK = PCLK / (BIU_CLK_DIV + 1)
 * MC_CLK = PCLK / (MC_CLK_DIV + 1)
 *
 * AP clock source:
 * 0x0 = PLL1 624 MHz
 * 0x1 = PLL1 1248 MHz  or PLL3_CLKOUT
 * (depending on FCAP[2])
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL1 832 MHZ
 * 0x5 = PLL3_CLKOUTP
 */
static struct cpu_opt helan2_op_array[] = {
	{
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.ap_clk_sel = 0x0,
	},
	{
		.pclk = 624,
		.pdclk = 312,
		.baclk = 156,
		.ap_clk_sel = 0x0,
	},
	{
		.pclk = 832,
		.pdclk = 416,
		.baclk = 208,
		.ap_clk_sel = 0x3,
	},
	{
		.pclk = 1066,
		.pdclk = 533,
		.baclk = 266,
		.ap_clk_sel = 0x2,
	},
	/* pp 1179 is only for test purpose */
	{
		.pclk = 1179,
		.pdclk = 589,
		.baclk = 294,
		.ap_clk_sel = 0x5,
	},
	{
		.pclk = 1248,
		.pdclk = 624,
		.baclk = 312,
		.ap_clk_sel = 0x1,
	},
};

static struct core_params helan2_core_params = {
	.core_offset = &corefc_reg_off,
	.parent_table = core_parent_table,
	.parent_table_size = ARRAY_SIZE(core_parent_table),
	.cpu_opt = helan2_op_array,
	.cpu_opt_size = ARRAY_SIZE(helan2_op_array),
#if 0
	.cpu_rtcwtc_table = cpu_rtcwtc_1x88,
	.cpu_rtcwtc_table_size = ARRAY_SIZE(cpu_rtcwtc_1x88),
#endif
	.max_cpurate = 1248,
	.dcstat_support = true,
};

static int helan2_powermode(u32 cpu)
{
	unsigned status_temp = 0;
	status_temp = ((__raw_readl(helan2_core_params.apmu_base +
			APMU_CORE_STATUS)) &
			((1 << (6 + 3 * cpu)) | (1 << (7 + 3 * cpu))));
	if (!status_temp)
		return MAX_LPM_INDEX;
	if (status_temp & (1 << (6 + 3 * cpu)))
		return LPM_C1;
	else if (status_temp & (1 << (7 + 3 * cpu)))
		return LPM_C2;
	return 0;
}

static DEFINE_SPINLOCK(fc_seq_lock);

/* helan2 uses MPMU_FCDCLK to set clk src */
static struct ddr_reg_offset ddr_reg_off = {
	/* ddr clk src sel set register */
	.fcdclk_off = 0x005c,
	.clk_sel_shift = 0,
	.clk_sel_width = 3,
	.tbl_enable_shift = 10,
	.tbl_index_shift = 3,
	.tbl_index_width = 5,
};

/*
 * DDR clock source:
 * 0x0 = PLL1 624 MHz
 * 0x1 = PLL1 832 MHz
 * 0x4 = PLL2_CLKOUT
 * 0x5 = PLL4_CLKOUT
 * 0x6 = PLL3_CLKOUTP
 */
static struct parents_table ddr_parent_table[] = {
	{
		.parent_name = "pll1_624",
		.hw_sel_val = 0x0,
	},
	{
		.parent_name = "pll1_832m",
		.hw_sel_val = 0x1,
	},
	{
		.parent_name = "pll2",
		.hw_sel_val = 0x4,
	},
	{
		.parent_name = "pll4",
		.hw_sel_val = 0x5,
	},
	{
		.parent_name = "pll3p",
		.hw_sel_val = 0x6,
	},
};

static const char *ddr_parent[] = {"pll1_624", "pll1_832m", "pll2p",
				"pll4", "pll3p",};

static struct ddr_opt lpddr533_oparray[] = {
	{
		.dclk = 156,
		.ddr_tbl_index = 2,
		.ddr_freq_level = 0,
		.ddr_clk_sel = 0x0,
	},
	{
		.dclk = 312,
		.ddr_tbl_index = 4,
		.ddr_freq_level = 1,
		.ddr_clk_sel = 0x0,
	},
	{
		.dclk = 416,
		.ddr_tbl_index = 6,
		.ddr_freq_level = 2,
		.ddr_clk_sel = 0x1,
	},
	{
		.dclk = 533,
		.ddr_tbl_index = 8,
		.ddr_freq_level = 3,
		.ddr_clk_sel = 0x4,
	},
};

static struct ddr_opt lpddr400_oparray[] = {
	{
		.dclk = 156,
		.ddr_tbl_index = 2,
		.ddr_freq_level = 0,
		.ddr_clk_sel = 0x0,
	},
	{
		.dclk = 312,
		.ddr_tbl_index = 4,
		.ddr_freq_level = 1,
		.ddr_clk_sel = 0x0,
	},
	{
		.dclk = 400,
		.ddr_tbl_index = 6,
		.ddr_freq_level = 2,
		.ddr_clk_sel = 0x4,
	},
};

static unsigned long hwdfc_freq_table[] = {
	312000, 400000, 400000, 533000
};

static struct ddr_params ddr_params = {
	.ddr_offset = &ddr_reg_off,
	.parent_table = ddr_parent_table,
	.parent_table_size = ARRAY_SIZE(ddr_parent_table),
	.hwdfc_freq_table = hwdfc_freq_table,
	.hwdfc_table_size = ARRAY_SIZE(hwdfc_freq_table),
	.dcstat_support = true,
};

/* helan2 uses MPMU_FCACLK to set clk src */
static struct axi_reg_offset axi_reg_off = {
	.fcaclk_off = 0x0060,
	.clk_sel0_shift = 0,
	.clk_sel0_width = 2,
};

/*
 * AXI clock source:
 * 0x0 = PLL1 416 MHz
 * 0x1 = PLL1 624 MHz
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL2_CLKOUTP
 */
static struct parents_table axi_parent_table[] = {
	{
		.parent_name = "pll1_416m",
		.hw_sel_val = 0x0,
	},
	{
		.parent_name = "pll1_624",
		.hw_sel_val = 0x1,
	},
	{
		.parent_name = "pll2",
		.hw_sel_val = 0x2,
	},
	{
		.parent_name = "pll2p",
		.hw_sel_val = 0x3,
	},
};

static const char *axi_parent[] = {"pll1_416m", "pll1_624", "pll2",
					"pll2p",};

static struct axi_opt axi_oparray[] = {
	{
		.aclk = 156,
		.axi_clk_sel = 0x1,
	},
	{
		.aclk = 208,
		.axi_clk_sel = 0x0,
	},
	{
		.aclk = 312,
		.axi_clk_sel = 0x1,
	},
};

static struct axi_params axi_params = {
	.axi_offset = &axi_reg_off,
	.parent_table = axi_parent_table,
	.parent_table_size = ARRAY_SIZE(axi_parent_table),
	.dcstat_support = true,
};

static struct ddr_combclk_relation aclk_dclk_relationtbl_1U88[] = {
	{.dclk_rate = 156000000, .combclk_rate = 156000000},
	{.dclk_rate = 312000000, .combclk_rate = 156000000},
	{.dclk_rate = 400000000, .combclk_rate = 208000000},
	{.dclk_rate = 533000000, .combclk_rate = 312000000},
};

static void __init helan2_acpu_init(void __iomem *apmu_base,
				    void __iomem *mpmu_base,
				    void __iomem *ciu_base,
				    void __iomem *dmcu_base)
{
	struct clk *clk;
	struct core_params *core_params = NULL;

	core_params = &helan2_core_params;
	if (ddr_mode) {
		ddr_params.ddr_opt = lpddr533_oparray;
		ddr_params.ddr_opt_size = ARRAY_SIZE(lpddr533_oparray);
	} else {
		ddr_params.ddr_opt = lpddr400_oparray;
		ddr_params.ddr_opt_size = ARRAY_SIZE(lpddr400_oparray);
	}
	axi_params.axi_opt = axi_oparray;
	axi_params.axi_opt_size = ARRAY_SIZE(axi_oparray);

	core_params->apmu_base = apmu_base;
	core_params->mpmu_base = mpmu_base;
	core_params->ciu_base = ciu_base;
	core_params->pxa_powermode = helan2_powermode;
	core_params->core_offset->pll1_pll3_swreg = apmu_base + APMU_PLL_SEL;
	ddr_params.apmu_base = apmu_base;
	ddr_params.mpmu_base = mpmu_base;
	ddr_params.dmcu_base = dmcu_base;
	axi_params.apmu_base = apmu_base;
	axi_params.mpmu_base = mpmu_base;

	clk = mmp_clk_register_core("cpu", core_parent,
		ARRAY_SIZE(core_parent), CLK_GET_RATE_NOCACHE,
		0, &fc_seq_lock, core_params);
	clk_register_clkdev(clk, "cpu", NULL);
	clk_prepare_enable(clk);
	clks[cpu] = clk;

	clk = mmp_clk_register_ddr("ddr", ddr_parent, ARRAY_SIZE(ddr_parent),
				   CLK_GET_RATE_NOCACHE,
				   MMP_DDR_HWDFC_FEAT | MMP_DDR_PLLSEL_3BIT,
				   &fc_seq_lock, &ddr_params);
	clk_register_clkdev(clk, "ddr", NULL);
	clk_prepare_enable(clk);
	clks[ddr] = clk;

	clk = mmp_clk_register_axi("axi", axi_parent, ARRAY_SIZE(axi_parent),
		CLK_GET_RATE_NOCACHE, 0,
		&fc_seq_lock, &axi_params);
	clk_register_clkdev(clk, "axi", NULL);
	clk_prepare_enable(clk);
	clks[axi] = clk;
	register_clk_bind2ddr(clk, axi_params.axi_opt
			      [axi_params.axi_opt_size - 1].aclk * MHZ_TO_HZ,
			      aclk_dclk_relationtbl_1U88,
			      ARRAY_SIZE(aclk_dclk_relationtbl_1U88));
}

/* Protect GC 3D register access APMU_GC&APMU_GC2D */
static DEFINE_SPINLOCK(gc_lock);
static DEFINE_SPINLOCK(gc2d_lock);
/* Protect VPU register access APMU_VPU_CLK_RES_CTRL */
static DEFINE_SPINLOCK(vpu_lock);
/* Protect SDH register access APMU_SDH */
static DEFINE_SPINLOCK(sdh_lock);

/* isp and ci shared aclk and ahb clock
 * Protect isp&ccic aclk register access APMU_ISP
 */
static DEFINE_SPINLOCK(isp_ci_aclk_lock);

/* Protect isp&ccic ahb register access APMU_CCIC0 */
static DEFINE_SPINLOCK(isp_ci_ahb_lock);

/* Protect ccic register access APMU_CCIC1 */
static DEFINE_SPINLOCK(ccic_lock);
/*
 * peripheral clock source:
 * 0x0 = PLL1 416 MHz
 * 0x1 = PLL1 624 MHz
 * 0x2 = PLL2_CLKOUT
 * 0x3 = PLL2_CLKOUTP
 */
enum periph_clk_src {
	CLK_PLL1_416 = 0x0,
	CLK_PLL1_624 = 0x1,
	CLK_PLL2 = 0x2,
	CLK_PLL2P = 0x3,
};

static struct clk_mux_sel periph_mux_sel[] = {
	{.parent_name = "pll1_416m", .value = CLK_PLL1_416},
	{.parent_name = "pll1_624", .value = CLK_PLL1_624},
	{.parent_name = "pll2", .value = CLK_PLL2},
	{.parent_name = "pll2p", .value = CLK_PLL2P},
};

/* For components which has d1p gate feature */
static struct clk_mux_sel periph_mux_sel_gate[] = {
	{.parent_name = "pll1_416_gate", .value = CLK_PLL1_416},
	{.parent_name = "pll1_624_gate", .value = CLK_PLL1_624},
	{.parent_name = "pll2", .value = CLK_PLL2},
	{.parent_name = "pll2p", .value = CLK_PLL2P},
};

#define SDH_FCLK_EN		((1 << 4) | (1 << 1))
#define SDH_ACLK_EN		(1 << 3)
#define REG_WIDTH_1BIT	1
#define REG_WIDTH_2BIT	2
#define REG_WIDTH_3BIT	3
#define REG_WIDTH_4BIT	4
#define REG_WIDTH_5BIT	5
static const char *sdhci_clk_parents[] = {"pll1_416m", "pll1_624",};
static struct clk_mux_sel sdhc_clk_mux[] = {
	{.parent_name = "pll1_416m", .value = CLK_PLL1_416},
	{.parent_name = "pll1_624", .value = CLK_PLL1_624},
};

static struct peri_params sdh_params = {
	.comclk_name = "sdh_shared_axi",
	.inputs = sdhc_clk_mux,
	.inputs_size = ARRAY_SIZE(sdhc_clk_mux),
};

static struct peri_reg_info sdh_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_1BIT),
	.div_shift = 8,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 11,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = SDH_FCLK_EN,
	.disable_val = SDH_FCLK_EN,
};

#define VPU_ACLK_EN	(0x1 << 3)
#define VPU_FCLK_EN	(0x1 << 4)
#define	VPU_AHBCLK_EN	(0x1 << 5)

static struct periph_clk_tbl vpu_aclk_tbl[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 208000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 312000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 416000000, .parent_name = "pll1_416_gate"},
};

static const char *vpu_aclk_parents[] = {"pll1_416_gate", "pll1_624_gate",
					"pll2", "pll2p",};

static struct peri_reg_info vpu_aclk_reg = {
	.src_sel_shift = 11,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 13,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 21,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = VPU_ACLK_EN,
	.disable_val = VPU_ACLK_EN,
};

static struct peri_params vpu_aclk_params = {
	.clktbl = vpu_aclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_aclk_tbl),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct periph_clk_tbl vpu_fclk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 208000000,
		.parent_name = "pll1_416_gate",
		.comclk_rate = 208000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416_gate",
		.comclk_rate = 416000000
	},
};

static const char *vpu_fclk_parents[] = {"pll1_416_gate", "pll1_624_gate",
					"pll2p", "pll2_div3",};

static struct peri_reg_info vpu_fclk_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 8,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 20,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (VPU_FCLK_EN | VPU_AHBCLK_EN),
	.disable_val = (VPU_FCLK_EN | VPU_AHBCLK_EN),
};

/* For components which has d1p gate feature */
static struct clk_mux_sel vpu_fclk_mux_sel[] = {
	{.parent_name = "pll1_416_gate", .value = 0x0},
	{.parent_name = "pll1_624_gate", .value = 0x1},
	{.parent_name = "pll2p", .value = 0x2},
	{.parent_name = "pll2_div3", .value = 0x3},
};

static struct peri_params vpu_fclk_params = {
	.clktbl = vpu_fclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_fclk_tbl),
	.comclk_name = "VPUACLK",
	.inputs = vpu_fclk_mux_sel,
	.inputs_size = ARRAY_SIZE(vpu_fclk_mux_sel),
	.dcstat_support = true,
};

#define GC_ACLK_EN	(0x1 << 3)
#define GC_FCLK_EN	(0x1 << 4)
#define GC_HCLK_EN	(0x1 << 5)
static const char *gc_parents[] = {"pll1_416_gate", "pll1_624_gate",
					"pll2", "pll2p",};

static const char *gc_3d_parents[] = {"pll1_832_gate", "pll1_624_gate",
					"pll2p", "pll2_div3",};

static struct periph_clk_tbl gc_aclk_tbl[] = {
	{.clk_rate = 78000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 104000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 156000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 208000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 312000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 416000000, .parent_name = "pll1_416_gate"},
};

static struct ddr_combclk_relation gcaclk_dclk_relationtbl_1U88[] = {
	{.dclk_rate = 104000000, .combclk_rate = 104000000},
	{.dclk_rate = 156000000, .combclk_rate = 156000000},
	{.dclk_rate = 312000000, .combclk_rate = 312000000},
	{.dclk_rate = 400000000, .combclk_rate = 400000000},
	{.dclk_rate = 416000000, .combclk_rate = 416000000},
	{.dclk_rate = 533000000, .combclk_rate = 416000000},
	{.dclk_rate = 667000000, .combclk_rate = 416000000},
};

static struct peri_reg_info gc_aclk_reg = {
	.src_sel_shift = 20,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 17,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 16,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = GC_ACLK_EN,
	.disable_val = GC_ACLK_EN,
};

static struct peri_params gc_aclk_params = {
	.clktbl = gc_aclk_tbl,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct periph_clk_tbl gc3d_fclk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_832_gate",
	},
	{
		.clk_rate = 624000000,
		.parent_name = "pll1_624_gate",
	},
};

static struct clk_mux_sel gc3d_periph_mux_sel[] = {
	{.parent_name = "pll1_832_gate", .value = 0x0},
	{.parent_name = "pll1_624_gate", .value = 0x1},
	{.parent_name = "pll2p", .value = 0x2},
	{.parent_name = "pll2_div3", .value = 0x3},
};

static struct peri_reg_info gc_fclk_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 12,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 15,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (GC_FCLK_EN | GC_HCLK_EN),
	.disable_val = (GC_FCLK_EN | GC_HCLK_EN),
};

static struct peri_params gc3d_fclk_params = {
	.clktbl = gc3d_fclk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_fclk_tbl),
	.comclk_name = "GC_ACLK",
	.inputs = gc3d_periph_mux_sel,
	.inputs_size = ARRAY_SIZE(gc3d_periph_mux_sel),
	.dcstat_support = true,
};

static struct periph_clk_tbl gc2d_fclk_tbl[] = {
	{
		.clk_rate = 78000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 208000000,
		.parent_name = "pll1_416_gate",
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
	},
};

static struct peri_params gc2d_fclk_params = {
	.clktbl = gc2d_fclk_tbl,
	.clktblsize = ARRAY_SIZE(gc2d_fclk_tbl),
	.comclk_name = "GC_ACLK",
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
	.dcstat_support = true,
};

static struct periph_clk_tbl gcshader_clk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416_gate",
	},
	{
		.clk_rate = 624000000,
		.parent_name = "pll1_624_gate",
	},
};

static struct peri_reg_info gcshader_clk_reg = {
	.src_sel_shift = 26,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 28,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 31,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = BIT(25),
	.disable_val = BIT(25),
};

static struct peri_params gcshader_clk_params = {
	.clktbl = gcshader_clk_tbl,
	.clktblsize = ARRAY_SIZE(gcshader_clk_tbl),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
	.dcstat_support = true,
};

#define ISP_CCIC_ACLK_EN		(1 << 17)
#define ISP_CCIC_ACLK_RST		(1 << 16)
#define ISP_CCIC_AHB_EN			(1 << 21)
#define ISP_CCIC_AHB_RST		(1 << 22)
static const char *isp_ccic_aclk_parents[] = {"pll1_416_gate", "pll1_624_gate",
					"pll2", "pll2p",};

static struct peri_reg_info isp_ccic_axi_reg = {
	.src_sel_shift = 21,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 18,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 23,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (ISP_CCIC_ACLK_RST | ISP_CCIC_ACLK_EN),
	.disable_val = ISP_CCIC_ACLK_EN,
};

static struct peri_params isp_ccic_axi_params = {
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct peri_params isp_ccic_ahb_params;
static struct peri_reg_info isp_ccic_ahb_reg = {
	.enable_val = (ISP_CCIC_AHB_RST | ISP_CCIC_AHB_EN),
	.disable_val = ISP_CCIC_AHB_EN,
};
#define ISP_FCLK_EN		\
	((1 << 1) | (1 << 28))
#define ISP_FCLK_RST		\
	((1 << 0) | (1 << 27))

static const char *isp_fclk_parents[] = {"pll1_416_gate", "pll1_624_gate",
					"pll4_div3", "pll2p",};

static struct clk_mux_sel isp_fclk_mux_sel[] = {
	{.parent_name = "pll1_416_gate", .value = 0x0},
	{.parent_name = "pll1_624_gate", .value = 0x1},
	{.parent_name = "pll4_div3", .value = 0x2},
	{.parent_name = "pll2p", .value = 0x3},
};

static struct peri_reg_info isp_fclk_reg = {
	.src_sel_shift = 2,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 4,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 7,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (ISP_FCLK_EN | ISP_FCLK_RST),
	.disable_val = ISP_FCLK_EN,
};

static struct peri_params isp_fclk_params = {
	.inputs = isp_fclk_mux_sel,
	.inputs_size = ARRAY_SIZE(isp_fclk_mux_sel),
};

/* APMU_CCIC1 */
#define CCIC_VCLK_EN		(1 << 26)
#define CCIC_FN_CLK_EN		(1 << 4)
#define CCIC_FN_CLK_RST		(1 << 1)
#define CCIC_PHY_CLK_EN		(1 << 5)
#define CCIC_PHY_CLK_RST	(1 << 2)

static struct clk_mux_sel ccic_fn_mux_sel[] = {
	{.parent_name = "pll1_416_gate", .value = 0x00 },
	{.parent_name = "pll1_624_gate", .value = 0x01 },
	{.parent_name = "pll2_div3", .value = 0x02 },
	{.parent_name = "pll2p", .value = 0x22 },
	{.parent_name = "pll4_div3", .value = 0x03 },
};

/*
 * CCIC function clk select use APMU_CCIC1[16:17] & APMU_CCIC1[21]
 * source select mask should be set as 0x23 to align with register
 */
static struct peri_reg_info ccic_func_reg = {
	.src_sel_shift = 16,
	.src_sel_mask = 0x23,
	.div_shift = 18,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 15,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_CLK_RST),
	.disable_val = CCIC_FN_CLK_EN,
};

static struct peri_params ccic_func_params = {
	.inputs = ccic_fn_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic_fn_mux_sel),
};

static struct peri_params ccic_phy_params;
static struct peri_reg_info ccic_phy_reg = {
	.enable_val = (CCIC_PHY_CLK_EN | CCIC_PHY_CLK_RST),
	.disable_val = CCIC_PHY_CLK_EN,
};
/* APMU_CCIC0 */
static struct clk_mux_sel ccic4_fn_mux_sel[] = {
	{.parent_name = "pll1_832_gate", .value = 0x00 },
	{.parent_name = "pll1_624_gate", .value = 0x01 },
	{.parent_name = "pll2_div3", .value = 0x02 },
	{.parent_name = "pll2p", .value = 0x82 },
	{.parent_name = "pll4_div3", .value = 0x03 },
};

/*
 * CCIC4x function clk select use APMU_CCIC1[16:17] & APMU_CCIC1[23]
 * source select mask should be set as 0x23 to align with register
 */
static struct peri_reg_info ccic4_func_reg = {
	.src_sel_shift = 16,
	.src_sel_mask = 0x83,
	.div_shift = 18,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 15,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_CLK_RST),
	.disable_val = CCIC_FN_CLK_EN,
};

static struct peri_params ccic4_func_params = {
	.inputs = ccic4_fn_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic4_fn_mux_sel),
};

static struct plat_clk_list peri_clk_list[] = {
	{ "sdhci-pxav3.0", NULL, 208000000},
	{ "sdhci-pxav3.1", NULL, 89000000},
	{ "sdhci-pxav3.2", NULL, 104000000},
	{ NULL, "VPUACLK", 416000000},
	{ NULL, "VPUCLK", 416000000},
	{ NULL, "GC_ACLK", 312000000},
	{ NULL, "GCCLK", 416000000},
	{ NULL, "GC2D_ACLK", 312000000},
	{ NULL, "GC2DCLK", 416000000},
	{ NULL, "GC_SHADER_CLK", 416000000},
	{ NULL, "SC2CCICAXI", 208000000},
	{ NULL, "isp-clk", 312000000},
	{ NULL, "SC2CSICLK", 312000000},
	{ "sc2_mv_camera.0", "CCICPHYCLK", 52000000},
	{ NULL, "SC24XCLK", 312000000},
	{ "sc2_mv_camera.1", "CCICPHYCLK", 52000000},
	{ NULL, "SC2MCLK", 26000000},
};

/* interface use to get peri clock avaliable op num and rate */
unsigned int __clk_periph_get_opnum(struct clk *clk)
{
	struct clk_peri *peri = to_clk_peri(clk->hw);
	struct periph_clk_tbl *cop;
	unsigned int idx = 0;

	BUG_ON(!peri);
	list_for_each_entry(cop, &peri->clktbl_list, node)
		idx++;
	return idx;
}

unsigned long __clk_periph_get_oprate(struct clk *clk,
		unsigned int index)
{
	struct clk_peri *peri = to_clk_peri(clk->hw);
	struct periph_clk_tbl *cop;
	unsigned int idx = 0;

	BUG_ON(!peri || list_empty(&peri->clktbl_list));
	list_for_each_entry(cop, &peri->clktbl_list, node) {
		if ((idx == index) ||
				list_is_last(&cop->node, &peri->clktbl_list))
			break;
		idx++;
	}
	return cop->clk_rate;
}

int __get_gcu_freqs_table(struct clk *clk,
		unsigned long *freqs_table, unsigned int *item_counts,
		unsigned int max_item_counts)
{
	unsigned int index, tbl_size;
	*item_counts = 0;

	if (!freqs_table) {
		pr_err("%s NULL ptr!\n", __func__);
		return -EINVAL;
	}

	tbl_size = __clk_periph_get_opnum(clk);
	if (max_item_counts < tbl_size) {
		pr_err("%s [%s]Too many GC frequencies %u!\n",
				__func__, clk->name,
				max_item_counts);
		return -EINVAL;
	}

	for (index = 0; index < tbl_size; index++)
		freqs_table[index] =
			__clk_periph_get_oprate(clk, index);

	*item_counts = index;
	return 0;
}

/* API used by GC driver to get avaliable GC3d/2d/shader frequencies, unit HZ */
int get_gcu_freqs_table(unsigned long *gcu_freqs_table,
		unsigned int *item_counts, unsigned int max_item_counts)
{
	return __get_gcu_freqs_table(clks[gc], gcu_freqs_table,
			item_counts, max_item_counts);
}
EXPORT_SYMBOL(get_gcu_freqs_table);

int get_gcu2d_freqs_table(unsigned long *gcu_freqs_table,
		unsigned int *item_counts, unsigned int max_item_counts)
{
	return __get_gcu_freqs_table(clks[gc2d], gcu_freqs_table,
			item_counts, max_item_counts);
}
EXPORT_SYMBOL(get_gcu2d_freqs_table);

int get_gc_shader_freqs_table(unsigned long *gcu_freqs_table,
		unsigned int *item_counts, unsigned int max_item_counts)
{
	return __get_gcu_freqs_table(clks[gcshader], gcu_freqs_table,
			item_counts, max_item_counts);
}
EXPORT_SYMBOL(get_gc_shader_freqs_table);

static void __init peri_init_set_rate(struct plat_clk_list *peri_clk_list,
					int size)
{
	int i;
	struct clk *clk;
	for (i = 0; i < size; i++) {
		clk = clk_get_sys(peri_clk_list[i].dev_id,
			peri_clk_list[i].con_id);
		if (IS_ERR(clk)) {
			pr_err(" can't find clk %s\n",
				peri_clk_list[i].con_id);
			continue;
		}
		clk_set_rate(clk, peri_clk_list[i].initrate);
	}
}

static void __init clk_peri_init(void __iomem *apmu_base, void __iomem *ciu_base)
{
	struct clk *clk;
	int tbl_size;

	clk = mmp_clk_register_apmu("sdh_shared_axi", NULL,
				apmu_base + APMU_SDH0, SDH_ACLK_EN, &clk_lock);
	clk_register_clkdev(clk, "sdh_shared_axi", NULL);

	clk = mmp_clk_register_peri("sdh0", sdhci_clk_parents,
		ARRAY_SIZE(sdhci_clk_parents), 0, apmu_base + APMU_SDH0,
		&sdh_lock, &sdh_params, &sdh_reg);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.0");

	clk = mmp_clk_register_peri("sdh1", sdhci_clk_parents,
		ARRAY_SIZE(sdhci_clk_parents), 0, apmu_base + APMU_SDH1,
		&sdh_lock, &sdh_params, &sdh_reg);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.1");

	clk = mmp_clk_register_peri("sdh2", sdhci_clk_parents,
		ARRAY_SIZE(sdhci_clk_parents), 0, apmu_base + APMU_SDH2,
		&sdh_lock, &sdh_params, &sdh_reg);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.2");

	/* VPU clock */
	clk = mmp_clk_register_peri("vpu_aclk", vpu_aclk_parents,
		ARRAY_SIZE(vpu_aclk_parents), 0,
		apmu_base + APMU_VPU_CLK_RES_CTRL,
		&vpu_lock, &vpu_aclk_params, &vpu_aclk_reg);
	clk_register_clkdev(clk, "VPUACLK", NULL);
	clks[vpu_aclk] = clk;

	clk = mmp_clk_register_peri("vpu", vpu_fclk_parents,
		ARRAY_SIZE(vpu_fclk_parents), 0,
		apmu_base + APMU_VPU_CLK_RES_CTRL,
		&vpu_lock, &vpu_fclk_params, &vpu_fclk_reg);
	clk_register_clkdev(clk, "VPUCLK", NULL);
	clks[vpu] = clk;

	/* CG2d&3D shared aclk , share same register with GC2D */
	clk = mmp_clk_register_peri("gc_aclk", gc_parents,
		ARRAY_SIZE(gc_parents), 0, apmu_base + APMU_GC2D,
		&gc2d_lock, &gc_aclk_params, &gc_aclk_reg);
	clk_register_clkdev(clk, "GC_ACLK", NULL);
	clks[gc_aclk] = clk;

	tbl_size = ARRAY_SIZE(gc_aclk_tbl);
	register_clk_bind2ddr(clk, gc_aclk_tbl[tbl_size - 1].clk_rate,
			      gcaclk_dclk_relationtbl_1U88,
			      ARRAY_SIZE(gcaclk_dclk_relationtbl_1U88));

	/* GC 3D function clock */
	clk = mmp_clk_register_peri("gc", gc_3d_parents,
		ARRAY_SIZE(gc_3d_parents), 0, apmu_base + APMU_GC,
		&gc_lock, &gc3d_fclk_params, &gc_fclk_reg);
	clk_register_clkdev(clk, "GCCLK", NULL);
	clks[gc] = clk;

	/* GC 2D function clock */
	clk = mmp_clk_register_peri("gc2d", gc_parents,
		ARRAY_SIZE(gc_parents), 0, apmu_base + APMU_GC2D,
		&gc2d_lock, &gc2d_fclk_params, &gc_fclk_reg);
	clk_register_clkdev(clk, "GC2DCLK", NULL);
	clks[gc2d] = clk;

	/* GC shader clock, share same register with GC3D */
	clk = mmp_clk_register_peri("gc_shader", gc_parents,
		ARRAY_SIZE(gc_parents), 0, apmu_base + APMU_GC,
		&gc_lock, &gcshader_clk_params, &gcshader_clk_reg);
	clk_register_clkdev(clk, "GC_SHADER_CLK", NULL);
	clks[gcshader] = clk;

	/* ISP&CCIC shared axi clock, share same register with APMU_ISP */
	clk = mmp_clk_register_peri("isp_ccic_aclk", isp_ccic_aclk_parents,
		ARRAY_SIZE(isp_ccic_aclk_parents), 0, apmu_base + APMU_ISP,
		&isp_ci_aclk_lock, &isp_ccic_axi_params, &isp_ccic_axi_reg);
	clk_register_clkdev(clk, "SC2CCICAXI", NULL);
	clks[ispccicaclk] = clk;

	/* ISP&CCIC AHB clock, share same register with APMU_CCIC0 */
	clk = mmp_clk_register_peri_gate("isp_ccic_ahb",
				NULL, 0, apmu_base + APMU_CCIC0,
				&isp_ci_ahb_lock, &isp_ccic_ahb_params,
				&isp_ccic_ahb_reg);
	clk_register_clkdev(clk, "ISPCCICAHB", NULL);

	/* ISP function clock */
	clk = mmp_clk_register_peri("isp", isp_fclk_parents,
		ARRAY_SIZE(isp_fclk_parents), 0, apmu_base + APMU_ISP,
		&isp_ci_aclk_lock, &isp_fclk_params, &isp_fclk_reg);
	clk_register_clkdev(clk, "ISP", NULL);
	clks[isp] = clk;

	/* CSI function clock */
	clk = mmp_clk_register_peri("sc2_csi_clk", ccic_fn_parent,
		ARRAY_SIZE(ccic_fn_parent), 0, apmu_base + APMU_CCIC1,
		&ccic_lock, &ccic_func_params, &ccic_func_reg);
	clk_register_clkdev(clk, "SC2CSICLK", NULL);

	/* CCIC VCLK */
	clk = clk_register_divider(NULL, "isim_vclk_div", "pll1_312_gate",
		0, apmu_base + APMU_CCIC1, 22, 4, 0, &ccic_lock);
	clk_register_clkdev(clk, "isim_vclk_div", NULL);

	clk = clk_register_gate(NULL, "isim_vclk_gate", "isim_vclk_div",
				CLK_SET_RATE_PARENT, apmu_base + APMU_CCIC1,
				26, 0, &ccic_lock);
	clk_register_clkdev(clk, "SC2MCLK", NULL);

	/* CCIC PHY CLK */
	clk = clk_register_mux(NULL, "ccic_phy_mux", ccic_phy_parent,
		ARRAY_SIZE(ccic_phy_parent), 0, apmu_base + APMU_CCIC1,
		7, 1, 0, &ccic_lock);
	clk_register_clkdev(clk, "ccic_phy_mux", NULL);

	clk = mmp_clk_register_peri_gate("ccic_phy_gate",
				"ccic_phy_mux", CLK_SET_RATE_PARENT,
				apmu_base + APMU_CCIC1,
				&ccic_lock, &ccic_phy_params,
				&ccic_phy_reg);
	clk_register_clkdev(clk, "CCICPHYCLK", "sc2_mv_camera.1");

	clk = clk_register_mux(NULL, "ccic4_phy_mux", ccic_phy_parent,
		ARRAY_SIZE(ccic_phy_parent), 0, apmu_base + APMU_CCIC0,
		7, 1, 0, &isp_ci_ahb_lock);
	clk_register_clkdev(clk, "ccic4_phy_mux", NULL);

	clk = mmp_clk_register_peri_gate("ccic4_phy_gate",
				"ccic4_phy_mux", CLK_SET_RATE_PARENT,
				apmu_base + APMU_CCIC0,
				&isp_ci_ahb_lock, &ccic_phy_params,
				&ccic_phy_reg);
	clk_register_clkdev(clk, "CCICPHYCLK", "sc2_mv_camera.0");

	/* CCIC function clock */
	clk = mmp_clk_register_peri("sc2_4x_clk", ccic4_fn_parent,
		ARRAY_SIZE(ccic4_fn_parent), 0, apmu_base + APMU_CCIC0,
		&isp_ci_ahb_lock, &ccic4_func_params, &ccic4_func_reg);
	clk_register_clkdev(clk, "SC24XCLK", NULL);

	peri_init_set_rate(peri_clk_list, ARRAY_SIZE(peri_clk_list));
}

static void __init clk_misc_init(void *apmu_base, void *ciu_base)
{
	unsigned int dcg_regval = 0;
	/*
	 * DE suggest:enable SQU MP3 playback sleep mode
	 * APMU_SQU_CLK_GATE_CTRL
	 */
	__raw_writel(__raw_readl(apmu_base + 0x001c) | (1 << 30),
			(apmu_base + 0x001c));

	/*
	 * disable nand controller clock as it is not used
	 */
	__raw_writel(0, apmu_base + APMU_DFC);

	/* enable all MCK and AXI fabric dynamic clk gating */
	dcg_regval = __raw_readl(ciu_base + CIU_MC_CONF);
	/* enable dclk gating */
	dcg_regval &= ~(1 << 19);
	/* enable 1x2 fabric AXI clock dynamic gating */
	dcg_regval |= (0xff << 8) |	/* MCK5 P0~P7*/
		(1 << 16) |		/* CP 2x1 fabric*/
		(1 << 17) | (1 << 18) |	/* AP&CP */
		(1 << 20) | (1 << 21) |	/* SP&CSAP 2x1 fabric */
		(1 << 26) | (1 << 27) | /* Fabric 0/1 */
		(1 << 29) | (1 << 30);	/* CA7 2x1 fabric */
	__raw_writel(dcg_regval, ciu_base + CIU_MC_CONF);
}

#ifdef CONFIG_SOUND
static void helan2_audio_subsystem_poweron(void __iomem *apmu_base, int pwr_on)
{
	u32 val;

	if (pwr_on) {
		/* clock enable */
		val = readl_relaxed(apmu_base + APMU_AUD_CLK_RES_CTRL);
		val |= 0x3;
		writel_relaxed(val, apmu_base + APMU_AUD_CLK_RES_CTRL);
	} else {
		/* clock disable */
		val = readl_relaxed(apmu_base + APMU_AUD_CLK_RES_CTRL);
		val &= ~0x3;
		writel_relaxed(val, apmu_base + APMU_AUD_CLK_RES_CTRL);
	}

}
#endif

void __init pxa988_clk_init(void)
{
	struct clk *clk;
	struct clk *uart_pll;
	struct clk *vct_pll;
	struct clk *i2s_clk;
	struct device_node *np;
	void __iomem *mpmu_base;
	void __iomem *apmu_base;
	void __iomem *apbcp_base;
	void __iomem *apbc_base;
	void __iomem *disp_div;
	void __iomem *apbs_base;
	void __iomem *ciu_base;
	void __iomem *audio_base;
	void __iomem *audio_aux_base;
	void __iomem *dmcu_base;

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

	np = of_find_compatible_node(NULL, NULL, "marvell,helan2-clock");
	if (np)
		dmcu_base = of_iomap(np, 0);
	else
		dmcu_base = NULL;
	if (dmcu_base == NULL) {
		pr_err("error to ioremap MCU base\n");
		return;
	}

	disp_div = ioremap(LCD_PN_SCLK, 4);
	if (disp_div == NULL) {
		pr_err("error to ioremap disp div\n");
		return;
	}

	audio_base = ioremap(AUD_MAP_BASE, SZ_2K);
	if (audio_base == NULL) {
		pr_err("error to ioremap audio base\n");
		return;
	}

	audio_aux_base = ioremap(AUD_AUX_BASE, SZ_2K);
	if (audio_aux_base == NULL) {
		pr_err("error to ioremap audio aux base\n");
		return;
	}

	clk = clk_register_fixed_rate(NULL, "clk32", NULL, CLK_IS_ROOT, 32000);
	clk_register_clkdev(clk, "clk32", NULL);

	vct_pll = clk_register_fixed_rate(NULL, "vctcxo", NULL, CLK_IS_ROOT,
				26000000);
	clk_register_clkdev(vct_pll, "vctcxo", NULL);

	helan2_pll_init(mpmu_base, apbs_base, apmu_base);

	helan2_acpu_init(apmu_base, mpmu_base, ciu_base, dmcu_base);

	clk_misc_init(apmu_base, ciu_base);

	uart_pll =  mmp_clk_register_factor("uart_pll", "pll1_4", 0,
				mpmu_base + MPMU_UART_PLL,
				&uart_factor_masks, uart_factor_tbl,
				ARRAY_SIZE(uart_factor_tbl));
	clk_set_rate(uart_pll, 14745600);
	clk_register_clkdev(uart_pll, "uart_pll", NULL);

	clk = clk_register_gate(NULL, "vcxo_gate_share", "vctcxo", 0,
				mpmu_base + MPMU_VRCR, 8, 0, &clk_lock);
	clk_register_clkdev(clk, "vcxo_gate_share", NULL);

	clk = clk_register_fixed_rate(NULL, "VCXO_OUT", "vcxo_gate_share",
				      0, 26000000);
	clk_register_clkdev(clk, "VCXO_OUT", NULL);

	clk = clk_register_fixed_rate(NULL, "VCXO_OUT2", "vcxo_gate_share",
				      0, 26000000);
	clk_register_clkdev(clk, "VCXO_OUT2", NULL);

	clk = mmp_clk_register_apbc("twsi0", "pll1_13_1_5",
				apbc_base + APBC_TWSI0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.0");

	clk = mmp_clk_register_apbc("twsi1", "pll1_13_1_5",
				apbc_base + APBC_TWSI1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.1");

	clk = mmp_clk_register_apbc("twsi2", "pll1_13_1_5",
				apbcp_base + APBCP_TWSI2, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.2");

	clk = mmp_clk_register_apbc("gpio", "vctcxo",
				apbc_base + APBC_GPIO, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-gpio");

	clk = mmp_clk_register_apbc("kpc", "clk32",
				apbc_base + APBC_KPC, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa27x-keypad");

	clk = clk_register_gate(NULL, "rtc_pe", "clk32", CLK_SET_RATE_PARENT,
				apbc_base + APBC_RTC, 7, 0, &rtc_lock);
	clk_register_clkdev(clk, "rtc_pe", NULL);

	clk = mmp_clk_register_apbc("rtc", "rtc_pe",
				    apbc_base + APBC_RTC, 10, 0, &rtc_lock);
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
	clk_register_clkdev(clk, "ssp0_mux", NULL);

	clk = mmp_clk_register_apbc("ssp0", "ssp0_mux",
				apbc_base + APBC_SSP0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa988-ssp.0");

	clk = clk_register_mux(NULL, "ssp1_mux", ssp1_parent,
			ARRAY_SIZE(ssp1_parent), CLK_SET_RATE_PARENT,
			mpmu_base + MPMU_FCCR, 28, 1, 0, &clk_lock);
	clk_set_parent(clk, vct_pll);
	clk_register_clkdev(clk, "ssp_mux.1", NULL);

	clk = clk_register_fixed_factor(NULL, "pllclk_2", "ssp1_mux",
				CLK_SET_RATE_PARENT, 1, 2);
	clk_register_clkdev(clk, "pllclk_2", NULL);

	clk = clk_register_gate(NULL, "sysclk_en", "pllclk_2", CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR1, 31, 0, &clk_lock);
	clk_register_clkdev(clk, "sysclk_en", NULL);

	clk =  mmp_clk_register_factor("mn_div", "sysclk_en", CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR1,
				&isccr1_factor_masks, isccr1_factor_tbl,
				ARRAY_SIZE(isccr1_factor_tbl));
	clk_set_rate(clk, 5644800);
	clk_register_clkdev(clk, "mn_div", NULL);

	clk = clk_register_gate(NULL, "bitclk_en", "mn_div", CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR1, 29, 0, &clk_lock);
	clk_register_clkdev(clk, "bitclk_en", NULL);

	clk = clk_register_divider_table(NULL, "bitclk_div_468", "bitclk_en", 0,
			mpmu_base + MPMU_ISCCR1, 27, 2, 0, clk_ssp1_ref_table,
			&clk_lock);
	clk_set_rate(clk, 1411200);
	clk_register_clkdev(clk, "bitclk_div_468", NULL);

	clk = clk_register_gate(NULL, "fnclk", "bitclk_div_468", CLK_SET_RATE_PARENT,
				apbc_base + APBC_SSP1, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "fnclk", NULL);

	clk = mmp_clk_register_apbc("pxa988-ssp.1", "fnclk",
				apbc_base + APBC_SSP1, 10, CLK_SET_RATE_PARENT, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa988-ssp.1");
	clk = clk_register_gate(NULL, "isccr0_sysclk_en", "pllclk_2",
				CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0, 31, 0, &clk_lock);
	clk_register_clkdev(clk, "isccr0_sysclk_en", NULL);

	clk =  mmp_clk_register_factor("isccr0_mn_div", "isccr0_sysclk_en",
				CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0,
				&isccr1_factor_masks, isccr1_factor_tbl,
				ARRAY_SIZE(isccr1_factor_tbl));
	clk_set_rate(clk, 2048000);
	clk_register_clkdev(clk, "isccr0_mn_div", NULL);

	clk = clk_register_gate(NULL, "isccr0_i2sclk_en", "isccr0_mn_div",
				CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0, 29, 0, &clk_lock);
	clk_register_clkdev(clk, "isccr0_i2sclk_en", NULL);

	i2s_clk = clk_register_divider_table(NULL, "isccr0_i2sclk",
				"isccr0_i2sclk_en", CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0, 27, 2, 0,
				clk_ssp1_ref_table,
				&clk_lock);
	clk_set_rate(i2s_clk, 256000);
	clk_register_clkdev(i2s_clk, "isccr0_i2sclk", NULL);

	clk = clk_register_mux(NULL, "gssp_func_mux", gssp_parent,
			ARRAY_SIZE(gssp_parent), CLK_SET_RATE_PARENT,
			apbcp_base + APBC_GCER, 8, 2, 0, &clk_lock);
	clk_set_parent(clk, i2s_clk);
	clk_register_clkdev(clk, "gssp_func_mux", NULL);

	clk = clk_register_gate(NULL, "gssp_fnclk", "gssp_func_mux",
				CLK_SET_RATE_PARENT,
				apbcp_base + APBC_GCER, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "gssp_fnclk", NULL);

	clk = mmp_clk_register_apbc("pxa988-ssp.4", "gssp_fnclk",
				apbcp_base + APBC_GCER, 10,
				CLK_SET_RATE_PARENT, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa988-ssp.4");

	clk_peri_init(apmu_base, ciu_base);

	clk = mmp_clk_register_apmu("usb", "usb_pll",
				apmu_base + APMU_USB, 0x9, &clk_lock);
	clk_register_clkdev(clk, NULL, "mv-otg");
	clk_register_clkdev(clk, NULL, "mv-udc");
	clk_register_clkdev(clk, NULL, "mv-ehci");
	clk_register_clkdev(clk, NULL, "pxa988-usb-phy");

	clk = mmp_clk_register_apmu("sph", "usb_pll",
				apmu_base + APMU_USB, 0x12, &clk_lock);
	clk_register_clkdev(clk, "sph_clk", NULL);


	pxa988_disp_clk_init(apmu_base);

#ifdef CONFIG_SOUND
	audio_clk_init(apmu_base, audio_base, audio_aux_base,
					helan2_audio_subsystem_poweron);
#endif
	clk = mmp_clk_register_apmu("dbgclk", "pll_416m",
				    apmu_base + APMU_TRACE,
				    (1 << 3) | (1 << 16), &trace_lock);
	clk_register_clkdev(clk, "DBGCLK", NULL);

	clk = mmp_clk_register_apmu("traceclk", "pll_416m",
				    apmu_base + APMU_TRACE,
				    (1 << 4) | (1 << 16), &trace_lock);
	clk_register_clkdev(clk, "TRACECLK", NULL);
	clk = mmp_clk_register_apmu("aes", "pll1_13", apmu_base + APMU_GEU,
				    (1 << 0) | (1 << 3), &clk_lock);
	clk_register_clkdev(clk, "AESCLK", NULL);

	clk = mmp_clk_register_apbc("ipc", NULL,
				apbc_base + APBC_IPC_CLK_RST, 10,
				0, NULL);
	clk_register_clkdev(clk, "ipc_clk", NULL);
	clk_prepare_enable(clk);

	clk = mmp_clk_register_apbc("ripc", NULL,
				apbcp_base + APBCP_AICER, 10,
				0, NULL);
	clk_register_clkdev(clk, "ripc_clk", NULL);
	clk_prepare_enable(clk);
	/*
	 * since ts clk register is different with regular apb_clock register,
	 * we use clk_register_gate to control the clk_en and clk_rst.
	 * enable: clk_en: 1; clk_rst: 0
	 * disable: clk_en: 0; clk_rst: 0
	 */
	clk = clk_register_gate(NULL, "ts_clk_rst", NULL, 0
				apbcp_base + APBC_TERMAL, 2,
				CLK_GATE_SET_TO_DISABLE, NULL);
	clk_register_clkdev(clk, "ts_clk_rst", NULL);
	clk_prepare_enable(clk);

	clk = clk_register_gate(NULL, "ts_clk", NULL, 0
				apbcp_base + APBC_TERMAL, 1, 0, NULL);
	clk_register_clkdev(clk, "ts_clk", NULL);
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *stat;
CLK_DCSTAT_OPS(clks[ddr], ddr)
CLK_DCSTAT_OPS(clks[axi], axi)
CLK_DCSTAT_OPS(clks[gc], gc);
CLK_DCSTAT_OPS(clks[gc2d], gc2d);
CLK_DCSTAT_OPS(clks[gcshader], gc_shader);
CLK_DCSTAT_OPS(clks[vpu], vpu);

static ssize_t pxa988_clk_stats_read(struct file *filp,
	char __user *buffer, size_t count, loff_t *ppos)
{
	char *buf;
	int len = 0, ret = 0;
	unsigned int reg, size = PAGE_SIZE - 1;
	void __iomem *apbc_base, *apbcp_base, *mpmu_base, *apmu_base;

	apbc_base = ioremap(APB_PHYS_BASE + 0x15000, SZ_4K);
	if (apbc_base == NULL) {
		pr_err("error to ioremap APBC base\n");
		return ret;
	}

	apbcp_base = ioremap(APB_PHYS_BASE + 0x3b000, SZ_4K);
	if (apbcp_base == NULL) {
		pr_err("error to ioremap APBC extension base\n");
		return ret;
	}

	mpmu_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	if (mpmu_base == NULL) {
		pr_err("error to ioremap MPMU base\n");
		return ret;
	}

	apmu_base = ioremap(AXI_PHYS_BASE + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return ret;
	}


	buf = (char *)__get_free_pages(GFP_NOIO, 0);
	if (!buf)
		return -ENOMEM;

	len += snprintf(buf + len, size,
		       "|---------------|-------|\n|%14s\t|%s|\n"
		       "|---------------|-------|\n", "Clock Name", " Status");

	/* PMU */
	reg = __raw_readl(mpmu_base + MPMU_PLL2CR);
	if (((reg & (3 << 8)) >> 8) == 2)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PLL2", "off");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PLL2", "on");

	reg = __raw_readl(mpmu_base + MPMU_PLL3CR);
	if (((reg & (3 << 18)) >> 18) == 0)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PLL3", "off");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PLL3", "on");

	reg = __raw_readl(apmu_base + APMU_SDH0);
	if ((((reg & (1 << 3)) >> 3) == 1) && (((reg & (1 << 0)) >> 0) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH ACLK", "off");

	if ((((reg & (1 << 4)) >> 4) == 1) && (((reg & (1 << 1)) >> 1) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH0 FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH0 FCLK", "off");

	reg = __raw_readl(apmu_base + APMU_SDH1);
	if ((((reg & (1 << 4)) >> 4) == 1) && (((reg & (1 << 1)) >> 1) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH1 FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH1 FCLK", "off");

	reg = __raw_readl(apmu_base + APMU_SDH2);
	if ((((reg & (1 << 4)) >> 4) == 1) && (((reg & (1 << 1)) >> 1) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH2 FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SDH2 FCLK", "off");

	reg = __raw_readl(apmu_base + APMU_VPU_CLK_RES_CTRL);
	if (((reg & (3 << 4)) >> 4) == 3)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "VPU FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "VPU FCLK", "off");
	if (((reg & (1 << 3)) >> 3) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "VPU ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "VPU ACLK", "off");

	reg = __raw_readl(apmu_base + APMU_GC);
	if (((reg & (3 << 4)) >> 4) == 3)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC FCLK", "off");
	if (((reg & (1 << 3)) >> 3) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC ACLK", "off");
	if (((reg & (1 << 25)) >> 25) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC SHADERCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC SHADERCLK", "off");

	reg = __raw_readl(apmu_base + APMU_GC2D);
	if (((reg & (3 << 4)) >> 4) == 3)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC2D FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC2D FCLK", "off");
	if (((reg & (1 << 3)) >> 3) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC2D ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GC2D ACLK", "off");

	reg = __raw_readl(apmu_base + APMU_LCD);
	if ((((reg & (1 << 1)) >> 1) == 1) && (((reg & (1 << 4)) >> 4) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD FCLK", "off");
	if (((reg & (1 << 3)) >> 3) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD ACLK", "off");
	if ((((reg & (1 << 5)) >> 5) == 1) && (((reg & (1 << 2)) >> 2) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD HCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "LCD HCLK", "off");

	reg = __raw_readl(apmu_base + APMU_DSI);
	if (((reg & (0xf << 2)) >> 2) == 0xf)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DSI PHYCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DSI PHYCLK", "off");
	reg = __raw_readl(apmu_base + APMU_ISP);
	if ((reg & 0xf03) == 0xf03)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "ISP_DXO CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "ISP_DXO CLK", "off");

	reg = __raw_readl(apmu_base + APMU_USB);
	if ((reg & 0x9) == 0x9)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "USB ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "USB ACLK", "off");

	reg = __raw_readl(apmu_base + APMU_CCIC0);
	if ((((reg & (1 << 4)) >> 4) == 1) && (((reg & (1 << 1)) >> 1) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 FCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 FCLK", "off");

	if ((((reg & (1 << 3)) >> 3) == 1) && (((reg & (1 << 0)) >> 0) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 ACLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 ACLK", "off");
	if ((((reg & (1 << 5)) >> 5) == 1) && (((reg & (1 << 2)) >> 2) == 1))
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 PHYCLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "CCIC0 PHYCLK", "off");

	reg = __raw_readl(apmu_base + APMU_TRACE);
	if (((reg & (1 << 3)) >> 3) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DBG CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DBG CLK", "off");
	if (((reg & (1 << 4)) >> 4) == 1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TRACE CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TRACE CLK", "off");

	reg = __raw_readl(apmu_base + APMU_GEU);
	if ((reg & 0x9) == 0x8)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "AES CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "AES CLK", "off");

	reg = __raw_readl(apmu_base + APMU_DFC);
	if ((reg & 0x19b) == 0x19b)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DFC CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "DFC CLK", "off");

	/* APB */
	reg = __raw_readl(apbc_base + APBC_TWSI0);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TWSI0 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TWSI0 CLK", "off");

	reg = __raw_readl(apbcp_base + APBCP_TWSI2);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TWSI2 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "TWSI2 CLK", "off");

	reg = __raw_readl(apbc_base + APBC_GPIO);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GPIO CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "GPIO CLK", "off");

	reg = __raw_readl(apbc_base + APBC_KPC);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "KPC CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "KPC CLK", "off");

	reg = __raw_readl(apbc_base + APBC_RTC);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "RTC CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "RTC CLK", "off");

	reg = __raw_readl(apbc_base + APBC_UART0);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "UART0 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "UART0 CLK", "off");

	reg = __raw_readl(apbc_base + APBC_UART1);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "UART1 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "UART1 CLK", "off");

	reg = __raw_readl(apbc_base + APBC_SSP0);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP0 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP0 CLK", "off");

	reg = __raw_readl(apbc_base + APBC_SSP1);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP1 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP1 CLK", "off");

	reg = __raw_readl(apbcp_base + APBC_GCER);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP.4 CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "SSP.4 CLK", "off");

	reg = __raw_readl(apbc_base + APBC_DROTS);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "THERMAL CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "THERMAL CLK", "off");

	reg = __raw_readl(apbc_base + APBC_PWM0);
	if ((reg & 0x1) == 0x1) {
		if ((reg & 0x6) == 0x2)
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM0 CLK", "on");
		else
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM0 CLK", "off");
		reg = __raw_readl(apbc_base + APBC_PWM1);
		if ((reg & 0x6) == 0x2)
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM1 CLK", "on");
		else
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM1 CLK", "off");
	} else {
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PWM0 CLK", "off");
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PWM1 CLK", "off");
	}

	reg = __raw_readl(apbc_base + APBC_PWM2);
	if ((reg & 0x1) == 0x1) {
		if ((reg & 0x6) == 0x2)
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM2 CLK", "on");
		else
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM2 CLK", "off");
		reg = __raw_readl(apbc_base + APBC_PWM3);
		if ((reg & 0x6) == 0x2)
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM3 CLK", "on");
		else
			len += snprintf(buf + len, size,
					"|%14s\t|%5s\t|\n", "PWM3 CLK", "off");
	} else {
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PWM2 CLK", "off");
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "PWM3 CLK", "off");
	}

	reg = __raw_readl(apbc_base + APBC_IPC_CLK_RST);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "IPC CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "IPC CLK", "off");

	reg = __raw_readl(apbcp_base + APBCP_AICER);
	if ((reg & 0x5) == 0x1)
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "RIPC CLK", "on");
	else
		len += snprintf(buf + len, size,
				"|%14s\t|%5s\t|\n", "RIPC CLK", "off");

	len += snprintf(buf + len, size, "|---------------|-------|\n\n");

	ret = simple_read_from_buffer(buffer, count, ppos, buf, len);
	free_pages((unsigned long)buf, 0);
	return ret;
}

static const struct file_operations pxa988_clk_stats_ops = {
	.owner = THIS_MODULE,
	.read = pxa988_clk_stats_read,
};

static int __init __init_dcstat_debugfs_node(void)
{
	struct dentry *gc_dc_stat = NULL, *vpu_dc_stat = NULL;
	struct dentry *gc2d_dc_stat = NULL, *gcshader_dc_stat = NULL;
	struct dentry *cpu_dc_stat = NULL, *ddr_dc_stat = NULL;
	struct dentry *axi_dc_stat = NULL;
	struct dentry *clock_status = NULL;

	stat = debugfs_create_dir("stat", pxa);
	if (!stat)
		return -ENOENT;

	cpu_dc_stat = cpu_dcstat_file_create("cpu_dc_stat", stat);
	if (!cpu_dc_stat)
		goto err_cpu_dc_stat;

	ddr_dc_stat = clk_dcstat_file_create("ddr_dc_stat", stat,
			&ddr_dc_ops);
	if (!ddr_dc_stat)
		goto err_ddr_dc_stat;

	axi_dc_stat = clk_dcstat_file_create("axi_dc_stat", stat,
			&axi_dc_ops);
	if (!axi_dc_stat)
		goto err_axi_dc_stat;

	gc_dc_stat = clk_dcstat_file_create("gc_dc_stat",
		stat, &gc_dc_ops);
	if (!gc_dc_stat)
		goto err_gc_dc_stat;

	gc2d_dc_stat = clk_dcstat_file_create("gc2d_dc_stat",
		stat, &gc2d_dc_ops);
	if (!gc2d_dc_stat)
		goto err_gc2d_dc_stat;

	gcshader_dc_stat = clk_dcstat_file_create("gc_shader_dc_stat",
		stat, &gc_shader_dc_ops);
	if (!gcshader_dc_stat)
		goto err_gcshader_dc_stat;

	vpu_dc_stat = clk_dcstat_file_create("vpu_dc_stat",
		stat, &vpu_dc_ops);
	if (!vpu_dc_stat)
		goto err_vpu_dc_stat;

	clock_status = debugfs_create_file("clock_status", 0444,
					   pxa, NULL, &pxa988_clk_stats_ops);
	if (!clock_status)
		goto err_clk_stats;

	return 0;
err_clk_stats:
	debugfs_remove(vpu_dc_stat);
err_vpu_dc_stat:
	debugfs_remove(gcshader_dc_stat);
err_gcshader_dc_stat:
	debugfs_remove(gc2d_dc_stat);
err_gc2d_dc_stat:
	debugfs_remove(gc_dc_stat);
err_gc_dc_stat:
	debugfs_remove(axi_dc_stat);
err_axi_dc_stat:
	debugfs_remove(ddr_dc_stat);
err_ddr_dc_stat:
	debugfs_remove(cpu_dc_stat);
err_cpu_dc_stat:
	debugfs_remove(stat);
	return -ENOENT;
}
late_initcall(__init_dcstat_debugfs_node);
#endif
