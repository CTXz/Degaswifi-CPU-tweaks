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
#include <linux/clk/mmp.h>
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
#define APMU_ISPDXO	0x038
#define APMU_TRACE	0x108
#define APMU_GEU	0x68
#define CIU_MC_CONF	0x0040
#define APMU_CORE_STATUS 0x090

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
	cpu = 0, ddr, axi,
	gc, gc2d, gcshader = 5, vpu, isp, clk_max,
};
static struct clk *clks[clk_max];


static DEFINE_SPINLOCK(clk_lock);
static DEFINE_SPINLOCK(disp_lock);
static DEFINE_SPINLOCK(trace_lock);
static DEFINE_SPINLOCK(rtc_lock);
static DEFINE_SPINLOCK(ccic0_lock);
static DEFINE_SPINLOCK(ccic1_lock);

#define LCD_PN_SCLK	(0xd420b1a8)

static unsigned long pll2_vco_default;
static unsigned long pll2_default;
static unsigned long pll2p_default;
static unsigned long pll3_vco_default;
static unsigned long pll3_default;
static unsigned long pll3p_default;

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

unsigned long max_freq;
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
static u32 disp_pll3_vco_setup;
static int __init pll3_vco_value_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	pll3_vco_default = n * MHZ_TO_HZ;
	disp_pll3_vco_setup = 1;
	return 1;
}
__setup("pll3_vco=", pll3_vco_value_setup);

u32 get_disp_pll3_vco_setup()
{
	return disp_pll3_vco_setup;
}
/*
 * Sim card num
 * 0: no card
 * 1/2 one card
 * 3: dual cards
 */
int simcard_num;
static int __init simcard_num_setup(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	simcard_num = n;
	return 1;
}
__setup("simcard=", simcard_num_setup);

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
	{.num = 14906, .den = 440},	/*for 0kHz*/
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

static const struct clk_div_table clk_twsi_ref_table[] = {
	{ .val = 0, .div = 19 },
	{ .val = 1, .div = 12 },
	{ .val = 2, .div = 10 },
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
static u32 pnsclk_parent_tbl[] = {1, 3};

static const char *ccic_phy_parent[] = {"pll1_6", "pll1_12"};
/* ccic_fn_parent:
 * 00: 416MHz
 * 01: 624MHz
 * 10: pll2
 * 11: pll2p
 */
static const char *ccic_fn_parent[] = {"pll1_416m", "pll1_624",
					"pll2", "pll2p"};

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

struct pi_cnf_reg pll2_pi_cnf_reg = {
	.intpishift = 4,
	.intpiwidth = 4,
	.diffclkselbit = 9,
	.seclkselbit = 8,
};

struct pi_ctrl_reg pll2_pi_ctrl_reg = {
	.enbit = 0,
	.clkdetenbit = 2,
	.sscclkenbit = 1
};

struct ssc_conf_reg pll2_ssc_conf_reg = {
	.divshift = 16,
	.divwidth = 16,
	.rngshift = 3,
	.rngwidth = 11,
};

struct ssc_modsel_reg pll2_ssc_modsel_reg = {
	.modselbit = 1,
};

struct ssc_ctrl_reg pll2_ssc_ctrl_reg = {
	.rstbit = 2,
	.enbit = 0,
};

struct pi_ssc_reg_des pll2_pi_ssc_reg_des = {
	.pi_cnf_reg = &pll2_pi_cnf_reg,
	.pi_ctrl_reg = &pll2_pi_ctrl_reg,
	.ssc_conf_reg = &pll2_ssc_conf_reg,
	.ssc_modsel_reg = &pll2_ssc_modsel_reg,
	.ssc_ctrl_reg = &pll2_ssc_ctrl_reg,
};

struct ssc_params pll2_ssc_params = {
	.ssc_mode = DOWN_SPREAD,
	.base = 1000,
	.amplitude = 25,
	.desired_mod_freq = 30000,
	.des = &pll2_pi_ssc_reg_des,
};

struct mmp_vco_params pll2_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 2500000000UL,
	.enable_bit = 0x100,
	.ctrl_bit = 0x200,
	.lock_enable_bit = POSR_PLL2_LOCK,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pll2_offset,
	.ssc_params = &pll2_ssc_params,
};

struct pi_cnf_reg pll3_pi_cnf_reg = {
	.intpishift = 4,
	.intpiwidth = 4,
	.diffclkselbit = 9,
	.seclkselbit = 8,
};

struct pi_ctrl_reg pll3_pi_ctrl_reg = {
	.enbit = 0,
	.clkdetenbit = 2,
	.sscclkenbit = 1
};

struct ssc_conf_reg pll3_ssc_conf_reg = {
	.divshift = 16,
	.divwidth = 16,
	.rngshift = 3,
	.rngwidth = 11,
};

struct ssc_modsel_reg pll3_ssc_modsel_reg = {
	.modselbit = 1,
};

struct ssc_ctrl_reg pll3_ssc_ctrl_reg = {
	.rstbit = 2,
	.enbit = 0,
};

struct pi_ssc_reg_des pll3_pi_ssc_reg_des = {
	.pi_cnf_reg = &pll3_pi_cnf_reg,
	.pi_ctrl_reg = &pll3_pi_ctrl_reg,
	.ssc_conf_reg = &pll3_ssc_conf_reg,
	.ssc_modsel_reg = &pll3_ssc_modsel_reg,
	.ssc_ctrl_reg = &pll3_ssc_ctrl_reg,
};
struct ssc_params pll3_ssc_params = {
	.ssc_mode = CENTER_SPREAD,
	.base = 1000,
	.amplitude = 25,
	.desired_mod_freq = 30000,
	.des = &pll3_pi_ssc_reg_des,
};

struct mmp_vco_params pll3_vco_params = {
	.vco_min = 1200000000UL,
	.vco_max = 2500000000UL,
	.enable_bit = 0x80000,
	.ctrl_bit = 0x40000,
	.lock_enable_bit = POSR_PLL3_LOCK,
	.kvco_rng_table = kvco_rng_table,
	.kvco_rng_size = ARRAY_SIZE(kvco_rng_table),
	.pll_offset = &pll3_offset,
	.ssc_params = &pll3_ssc_params,
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

#define LCD_PST_CKEN            (1 << 9)
#define LCD_PST_OUTDIS          (1 << 8)
#define LCD_HCLK_SWT_EN         (1 << 31)
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
			  LCD_HCLK_SWT_EN |
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
	else if (max_freq <= CORE_1p25G)
		max_freq = CORE_1p25G;
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

#define PLL_INIT_MASK	((PLL_MASK(2) << 30) | (PLL_MASK(4) << 26) |\
	(PLL_MASK(2) << 24) | (PLL_MASK(3) << 21) | (PLL_MASK(2) << 12) |\
	(PLL_MASK(1) << 3))

#define PLL_INIT	((1 << 30) | (9 << 26) | (2 << 24) | (4 << 21) |\
			(1 << 12) | (1 << 3))
void __init pxa988_pll_init(void *mpmu_base, void *apbs_base, void *apmu_base)
{
	u32 pll_init, ssc_flags = 0;
	struct clk *clk;
	struct clk *pll2_vco, *pll2, *pll2p, *pll3_vco, *pll3, *pll3p;

	clk = clk_register_fixed_rate(NULL, "pll1_624", NULL, CLK_IS_ROOT,
				624000000);
	clk_register_clkdev(clk, "pll1_624", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_416m", NULL, CLK_IS_ROOT,
				416000000);
	clk_register_clkdev(clk, "pll1_416m", NULL);

	if (has_feat_lcd_more_source()) {
		clk = clk_register_fixed_rate(NULL, "pll1_832m", NULL,
					      CLK_IS_ROOT, 832000000);
		clk_register_clkdev(clk, "pll1_832m", NULL);
	}
	clk = clk_register_fixed_rate(NULL, "pll1_1248", NULL, CLK_IS_ROOT,
				1248000000);
	clk_register_clkdev(clk, "pll1_1248", NULL);

	/* pll1_416/624 clock which has d1p gating feature */
	clk = clk_register_gate(NULL, "pll1_416_gate", "pll1_416m",
			CLK_SET_RATE_PARENT, apmu_base + APMU_CLK_GATE_CTRL,
			27, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_416_gate", NULL);

	clk = clk_register_gate(NULL, "pll1_624_gate", "pll1_624",
			CLK_SET_RATE_PARENT, apmu_base + APMU_CLK_GATE_CTRL,
			26, 0, &d1p_gate_lock);
	clk_register_clkdev(clk, "pll1_624_gate", NULL);

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

	clk = clk_register_fixed_factor(NULL, "pll1_13_1_5", "pll1_13",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_13_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_2_1_5", "pll1_2",
				CLK_SET_RATE_PARENT, 2, 3);
	clk_register_clkdev(clk, "pll1_2_1_5", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_3_16", "pll1_624",
				CLK_SET_RATE_PARENT, 3, 16);
	clk_register_clkdev(clk, "pll1_3_16", NULL);

	pll2_vco_params.cr_reg = mpmu_base + 0x0034;
	pll2_vco_params.pll_swcr = apbs_base + 0x0104;
	pll2_vco_params.lock_reg = mpmu_base + 0x0010;
	pll2_pi_cnf_reg.regoff = apbs_base + 0x118;
	pll2_pi_ctrl_reg.regoff = apbs_base + 0x118;
	pll2_ssc_conf_reg.regoff = apbs_base + 0x11c;
	pll2_ssc_modsel_reg.regoff = apbs_base + 0x11c;
	pll2_ssc_ctrl_reg.regoff = apbs_base + 0x11c;

	if (ddr_mode)
		pll2_vco_params.default_vco_rate = 2132 * MHZ_TO_HZ;
	else
		pll2_vco_params.default_vco_rate = 1600 * MHZ_TO_HZ;
	pll2_pll_params.pll_swcr = apbs_base + 0x0104;
	pll2_pllp_params.pll_swcr = apbs_base + 0x0104;
	pll_init = __raw_readl(pll2_pll_params.pll_swcr);
	pll_init &= ~PLL_INIT_MASK;
	pll_init |= PLL_INIT;
	__raw_writel(pll_init, pll2_pll_params.pll_swcr);

	pll3_vco_params.cr_reg = mpmu_base + 0x001c;
	pll3_vco_params.pll_swcr = apbs_base + 0x0108;
	pll3_vco_params.lock_reg = mpmu_base + 0x0010;
	pll3_pi_cnf_reg.regoff = apbs_base + 0x124;
	pll3_pi_ctrl_reg.regoff = apbs_base + 0x124;
	pll3_ssc_conf_reg.regoff = apbs_base + 0x128;
	pll3_ssc_modsel_reg.regoff = apbs_base + 0x128;
	pll3_ssc_ctrl_reg.regoff = apbs_base + 0x128;

	pll3_vco_params.default_vco_rate = (unsigned long)2366 * MHZ_TO_HZ;
	pll3_pll_params.pll_swcr = apbs_base + 0x0108;
	pll3_pllp_params.pll_swcr = apbs_base + 0x0108;
	pll_init = __raw_readl(pll3_pll_params.pll_swcr);
	pll_init &= ~PLL_INIT_MASK;
	pll_init |= PLL_INIT;
	__raw_writel(pll_init, pll3_pll_params.pll_swcr);

	if (ddr_mode) {
		pll2_vco_default = 2132 * MHZ_TO_HZ;
		pll2_default = 1066 * MHZ_TO_HZ;
		pll2p_default = 1066 * MHZ_TO_HZ;
	} else {
		pll2_vco_default = 1600 * MHZ_TO_HZ;
		pll2_default = 800 * MHZ_TO_HZ;
		pll2p_default = 800 * MHZ_TO_HZ;
	}

	if (cpu_is_pxa1L88_a0c())
		ssc_flags = MMP_PLL_SSC_FEAT | MMP_PLL_SSC_AON;

	pll2_vco = mmp_clk_register_vco("pll2_vco", NULL, CLK_IS_ROOT,
					MMP_PLL_LOCK_SETTING | ssc_flags,
					&pll2_lock, &pll2_vco_params, NULL, 0);
	clk_register_clkdev(pll2_vco, "pll2_vco", NULL);
	clk_set_rate(pll2_vco, pll2_vco_default);

	pll2 = mmp_clk_register_pll("pll2", "pll2_vco", 0,
		0, &pll2_lock, &pll2_pll_params);
	clk_register_clkdev(pll2, "pll2", NULL);

	pll2p = mmp_clk_register_pll("pll2p", "pll2_vco", 0,
		0, &pll2_lock, &pll2_pllp_params);
	clk_register_clkdev(pll2p, "pll2p", NULL);
	clk_set_rate(pll2, pll2_default);
	clk_set_rate(pll2p, pll2p_default);

	setup_pll3_vco();
	pll3_vco_params.default_vco_rate = pll3_vco_default;
	if (cpu_is_pxa1L88_a0c())
		ssc_flags = MMP_PLL_SSC_FEAT;

	pll3_vco = mmp_clk_register_vco("pll3_vco", NULL, CLK_IS_ROOT,
		ssc_flags, &pll3_lock, &pll3_vco_params, NULL, 0);
	clk_register_clkdev(pll3_vco, "pll3_vco", NULL);
	clk_set_rate(pll3_vco, pll3_vco_default);

	pll3 = mmp_clk_register_pll("pll3", "pll3_vco", 0,
		0, &pll3_lock, &pll3_pll_params);
	clk_register_clkdev(pll3, "pll3", NULL);

	pll3p = mmp_clk_register_pll("pll3p", "pll3_vco", 0,
		0, &pll3_lock, &pll3_pllp_params);
	clk_register_clkdev(pll3p, "pll3p", NULL);
	clk_set_rate(pll3, pll3_default);
	clk_set_rate(pll3p, pll3p_default);
}

static struct clk *lcd_get_parent(char *panel_type, char *item, char *df_parent)
{
#ifdef CONFIG_OF
	struct device_node *np = of_find_node_by_name(NULL, panel_type);
	struct clk *parent = NULL;
	const char *str = NULL;
	if (np && !of_property_read_string(np, item, &str))
		parent = clk_get(NULL, str);

	return !IS_ERR_OR_NULL(parent) ? parent : clk_get(NULL, df_parent);
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
			0, apmu_base, &dsi_pll);
	else
		clk = mmp_clk_register_disp("dsi_pll", dsipll_parent1,
			ARRAY_SIZE(dsipll_parent1), MMP_DISP_MUX_ONLY,
			0, apmu_base, &dsi_pll);
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

struct core_reg_offset pxa988_reg_off = {
	/* core clk src sel set register */
	.fcap_off = 0x0008,
	.clk_sel_shift = 29,
	.clk_sel_width = 3,
	.pll1_pll3_swbit = BIT(18),
};

static struct cpu_rtcwtc cpu_rtcwtc_1088[] = {
	{.max_pclk = 312, .l1_rtc = 0x02222222, .l2_rtc = 0x00002221,},
	{.max_pclk = 800, .l1_rtc = 0x02666666, .l2_rtc = 0x00006265,},
	{.max_pclk = 1183, .l1_rtc = 0x2AAAAAA, .l2_rtc = 0x0000A2A9,},
	{.max_pclk = 1482, .l1_rtc = 0x02EEEEEE, .l2_rtc = 0x0000E2ED,},
};

static struct cpu_rtcwtc cpu_rtcwtc_1x88[] = {
	{.max_pclk = 624, .l1_rtc = 0x02222222, .l2_rtc = 0x00002221,},
	{.max_pclk = 1066, .l1_rtc = 0x02666666, .l2_rtc = 0x00006265,},
	{.max_pclk = 1283, .l1_rtc = 0x2AAAAAA, .l2_rtc = 0x0000A2A9,},
	{.max_pclk = 1482, .l1_rtc = 0x02EEEEEE, .l2_rtc = 0x0000E2ED,},
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

/* The PP table only list the possible op here */
static struct cpu_opt pxa1x88_op_array[] = {
	{
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 624,
		.pdclk = 312,
		.baclk = 156,
		.ap_clk_sel = AP_CLK_SRC_PLL1_624,
	},
	{
		.pclk = 800,
		.pdclk = 400,
		.baclk = 200,
		.ap_clk_sel = AP_CLK_SRC_PLL2P,
	},
	{
		.pclk = 1066,
		.pdclk = 533,
		.baclk = 266,
		.ap_clk_sel = AP_CLK_SRC_PLL2P,
	},
	{
		.pclk = 1183,
		.pdclk = 591,
		.baclk = 295,
		.ap_clk_sel = AP_CLK_SRC_PLL3P,
	},
	{
		.pclk = 1248,
		.pdclk = 624,
		.baclk = 312,
		.ap_clk_sel = AP_CLK_SRC_PLL1_1248,
	},
	{
		.pclk = 1482,
		.pdclk = 741,
		.baclk = 370,
		.ap_clk_sel = AP_CLK_SRC_PLL3P,
	},
};

struct core_params pxa1x88_core_params = {
	.core_offset = &pxa988_reg_off,
	.parent_table = parent_table,
	.parent_table_size = ARRAY_SIZE(parent_table),
	.cpu_opt = pxa1x88_op_array,
	.cpu_opt_size = ARRAY_SIZE(pxa1x88_op_array),
	.cpu_rtcwtc_table = cpu_rtcwtc_1x88,
	.cpu_rtcwtc_table_size = ARRAY_SIZE(cpu_rtcwtc_1x88),
	.max_cpurate = 1183,
	.dcstat_support = true,
	.shared_lock = &pll3_lock,
};

int pxa1x88_powermode(u32 cpu)
{
	unsigned status_temp = 0;
	status_temp = ((__raw_readl(pxa1x88_core_params.apmu_base +
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

struct ddr_reg_offset ddr_reg_off = {
	/* core clk src sel set register */
	.fcdclk_off = 0x0008,
	.tbl_ctrl_off = 0x01c0,
	.clk_sel_shift = 23,
	.clk_sel_width = 2,
	.tbl_enable_shift = 6,
	.tbl_index_shift = 3,
	.tbl_index_width = 3,
};

struct parents_table ddr_parent_table[] = {
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

static const char *ddr_parent[] = {
	"pll1_416m", "pll1_624",
	"pll2", "pll2p",
};

static struct ddr_opt lpddr400_oparray[] = {
	{
		.dclk = 156,
		.ddr_tbl_index = 2,
		.ddr_freq_level = 0,
		.ddr_clk_sel = 0x1,
	},
	{
		.dclk = 312,
		.ddr_tbl_index = 3,
		.ddr_freq_level = 1,
		.ddr_clk_sel = 0x1,
	},
	{
		.dclk = 400,
		.ddr_tbl_index = 4,
		.ddr_freq_level = 2,
		.ddr_clk_sel = 0x3,
	},
};

static struct ddr_opt lpddr533_oparray[] = {
	{
		.dclk = 156,
		.ddr_tbl_index = 2,
		.ddr_freq_level = 0,
		.ddr_clk_sel = 0x1,
	},
	{
		.dclk = 312,
		.ddr_tbl_index = 3,
		.ddr_freq_level = 1,
		.ddr_clk_sel = 0x1,
	},
	{
		.dclk = 533,
		.ddr_tbl_index = 4,
		.ddr_freq_level = 2,
		.ddr_clk_sel = 0x3,
	},
};

static unsigned long hwdfc_freq_table[] = {
	312000, 400000, 400000, 533000
};

struct ddr_params ddr_params = {
	.ddr_offset = &ddr_reg_off,
	.parent_table = ddr_parent_table,
	.parent_table_size = ARRAY_SIZE(ddr_parent_table),
	.hwdfc_freq_table = hwdfc_freq_table,
	.hwdfc_table_size = ARRAY_SIZE(hwdfc_freq_table),
	.dcstat_support = true,
};

struct axi_reg_offset axi_reg_off = {
	/* core clk src sel set register */
	.fcaclk_off = 0x0008,
	.clk_sel0_shift = 19,
	.clk_sel0_width = 1,
	.clk_sel1_shift = 25,
	.clk_sel1_width = 1,
};

struct parents_table axi_parent_table[] = {
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

static const char *axi_parent[] = {
	"pll1_416m", "pll1_624",
	"pll2", "pll2p",
};

static struct axi_opt axi200_oparray[] = {
	{
		.aclk = 78,
		.axi_clk_sel = 0x1,
	},
	{
		.aclk = 156,
		.axi_clk_sel = 0x1,
	},
	{
		.aclk = 200,
		.axi_clk_sel = 0x3,
	},
};

static struct axi_opt axi266_oparray[] = {
	{
		.aclk = 78,
		.axi_clk_sel = 0x1,
	},
	{
		.aclk = 156,
		.axi_clk_sel = 0x1,
	},
	{
		.aclk = 266,
		.axi_clk_sel = 0x3,
	},
};

struct axi_params axi_params = {
	.axi_offset = &axi_reg_off,
	.parent_table = axi_parent_table,
	.parent_table_size = ARRAY_SIZE(axi_parent_table),
	.dcstat_support = true,
};

struct ddr_combclk_relation aclk_dclk_relationtbl_1088[] = {
	{.dclk_rate = 156000000, .combclk_rate = 78000000},
	{.dclk_rate = 312000000, .combclk_rate = 156000000},
	{.dclk_rate = 400000000, .combclk_rate = 200000000},
	{.dclk_rate = 533000000, .combclk_rate = 266000000},
};

struct ddr_combclk_relation aclk_dclk_relationtbl_1L88[] = {
	{.dclk_rate = 156000000, .combclk_rate = 156000000},
	{.dclk_rate = 312000000, .combclk_rate = 156000000},
	{.dclk_rate = 400000000, .combclk_rate = 200000000},
	{.dclk_rate = 533000000, .combclk_rate = 266000000},
};

void __init pxa988_acpu_init(void __iomem *apmu_base, void __iomem *mpmu_base,
	void __iomem *ciu_base, void __iomem *dmcu_base)
{
	struct clk *clk;
	struct core_params *core_params = NULL;
	unsigned int ddr_clk_flag = 0, hwdfc_flag = 0;
	int i;

	core_params = &pxa1x88_core_params;
	core_params->max_cpurate = max_freq;

	if (ddr_mode) {
		ddr_params.ddr_opt = lpddr533_oparray;
		ddr_params.ddr_opt_size = ARRAY_SIZE(lpddr533_oparray);
		axi_params.axi_opt = axi266_oparray;
		axi_params.axi_opt_size = ARRAY_SIZE(axi266_oparray);
	} else {
		ddr_params.ddr_opt = lpddr400_oparray;
		ddr_params.ddr_opt_size = ARRAY_SIZE(lpddr400_oparray);
		axi_params.axi_opt = axi200_oparray;
		axi_params.axi_opt_size = ARRAY_SIZE(axi200_oparray);
	}

	if (cpu_is_pxa1L88()) {
		ddr_clk_flag = MMP_DDR_HWDFC_FEAT;
		hwdfc_flag = MMP_DDR_HWDFC_WR;
	} else {
		/* For 1088, ddr reg table index is 1, 3, 5 */
		for (i = 0; i < ARRAY_SIZE(lpddr533_oparray); i++)
			lpddr533_oparray[i].ddr_tbl_index = i * 2 + 1;
		for (i = 0; i < ARRAY_SIZE(lpddr400_oparray); i++)
			lpddr400_oparray[i].ddr_tbl_index = i * 2 + 1;
		core_params->cpu_rtcwtc_table = cpu_rtcwtc_1088;
		core_params->cpu_rtcwtc_table_size =
				ARRAY_SIZE(cpu_rtcwtc_1088);
		/* TD dual sim card */
		if (simcard_num == 3)
			aclk_dclk_relationtbl_1088[0].combclk_rate = 156000000;

		/* For 1088, we enable DLL bypass optimization */
		ddr_clk_flag = MMP_DDR_DLL_BYPASS;
	}

	core_params->apmu_base = apmu_base;
	core_params->mpmu_base = mpmu_base;
	core_params->ciu_base = ciu_base;
	core_params->pxa_powermode = pxa1x88_powermode;
	core_params->core_offset->pll1_pll3_swreg = mpmu_base + MPMU_PLL3CR;
	ddr_params.apmu_base = apmu_base;
	ddr_params.mpmu_base = mpmu_base;
	ddr_params.dmcu_base = dmcu_base;
	axi_params.apmu_base = apmu_base;
	axi_params.mpmu_base = mpmu_base;

	clk = mmp_clk_register_core("cpu", core_parent,
		ARRAY_SIZE(core_parent), CLK_GET_RATE_NOCACHE,
		MMP_CORE_PLL3_SEL | hwdfc_flag,
		&fc_seq_lock, core_params);
	clk_register_clkdev(clk, "cpu", NULL);
	clk_prepare_enable(clk);
	clks[cpu] = clk;

	clk = mmp_clk_register_ddr("ddr", ddr_parent, ARRAY_SIZE(ddr_parent),
				   CLK_GET_RATE_NOCACHE, ddr_clk_flag,
				   &fc_seq_lock, &ddr_params);
	clk_register_clkdev(clk, NULL, "devfreq-ddr");
	clk_register_clkdev(clk, "ddr", NULL);
	clk_prepare_enable(clk);
	clks[ddr] = clk;

	clk = mmp_clk_register_axi("axi", axi_parent, ARRAY_SIZE(axi_parent),
		0, MMP_AXI_SEPERATED_SRC_SEL | hwdfc_flag,
		&fc_seq_lock, &axi_params);
	clk_register_clkdev(clk, "axi", NULL);
	clk_prepare_enable(clk);
	clks[axi] = clk;
	if (cpu_is_pxa1L88())
		register_clk_bind2ddr(clk, axi_params.axi_opt
				[axi_params.axi_opt_size - 1].aclk * MHZ_TO_HZ,
				aclk_dclk_relationtbl_1L88,
				ARRAY_SIZE(aclk_dclk_relationtbl_1L88));
	else
		register_clk_bind2ddr(clk, axi_params.axi_opt
				[axi_params.axi_opt_size - 1].aclk * MHZ_TO_HZ,
				aclk_dclk_relationtbl_1088,
				ARRAY_SIZE(aclk_dclk_relationtbl_1088));
}

/* Protect GC 3D register access APMU_GC&APMU_GC2D */
static DEFINE_SPINLOCK(gc_lock);
static DEFINE_SPINLOCK(gc2d_lock);
/* Protect VPU register access APMU_VPU_CLK_RES_CTRL */
static DEFINE_SPINLOCK(vpu_lock);
/* Protect SDH register access APMU_SDH */
static DEFINE_SPINLOCK(sdh_lock);
/* Protect LCD register access APMU_LCD */
static DEFINE_SPINLOCK(lcd_ci_share_lock);
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

/*
 * CCIC phy clock source:
 * 0x0 = PLL1_6 104 MHz
 * 0x1 = PLL1_12 52 MHz
 */
enum ccic_phy_clk_src {
	CLK_PLL1_104 = 0x0,
	CLK_PLL1_52 = 0x1,
};

static struct clk_mux_sel ccic_phy_mux_sel[] = {
	{.parent_name = "pll1_6", .value = CLK_PLL1_104},
	{.parent_name = "pll1_12", .value = CLK_PLL1_52},
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

static struct periph_clk_tbl vpu_aclk_tbl_common[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_416m"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 416000000, .parent_name = "pll1_416m"},
	{.clk_rate = 533000000, .parent_name = "pll2p"},
};

static struct periph_clk_tbl vpu_aclk_tbl_gate[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 208000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 312000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 416000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 533000000, .parent_name = "pll2p"},
};

static const char *vpu_parents_common[] = {
		"pll1_416m", "pll1_624",
		"pll2", "pll2p",
};

static const char *vpu_parents_gate[] = {
		"pll1_416_gate", "pll1_624_gate",
		"pll2", "pll2p",
};

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

static struct peri_params vpu_aclk_params_common = {
	.clktbl = vpu_aclk_tbl_common,
	.clktblsize = ARRAY_SIZE(vpu_aclk_tbl_common),
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_params vpu_aclk_params_gate = {
	.clktbl = vpu_aclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(vpu_aclk_tbl_gate),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct periph_clk_tbl vpu_fclk_tbl_common[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 208000000,
		.parent_name = "pll1_416m",
		.comclk_rate = 208000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416m",
		.comclk_rate = 416000000
	},
	{
		.clk_rate = 533000000,
		.parent_name = "pll2p",
		.comclk_rate = 533000000
	},
};

static struct periph_clk_tbl vpu_fclk_tbl_gate[] = {
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
	{
		.clk_rate = 533000000,
		.parent_name = "pll2p",
		.comclk_rate = 533000000
	},
};

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

struct xpu_rtcwtc vpu_rtcwtc_1088[] = {
	{.max_rate = 156000000, .rtcwtc = 0x00B04444,},
	{.max_rate = 208000000, .rtcwtc = 0x00B05544,},
	{.max_rate = 533000000, .rtcwtc = 0x00B06655,},
};

struct xpu_rtcwtc vpu_rtcwtc[] = {
	{.max_rate = 208000000, .rtcwtc = 0x00B04444,},
	{.max_rate = 312000000, .rtcwtc = 0x00B05555,},
	{.max_rate = 416000000, .rtcwtc = 0x00B06655,},
	{.max_rate = 624000000, .rtcwtc = 0x00B07777,},
};

static struct peri_params vpu_fclk_params_common = {
	.clktbl = vpu_fclk_tbl_common,
	.clktblsize = ARRAY_SIZE(vpu_fclk_tbl_common),
	.comclk_name = "VPUACLK",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
	.rwtctbl = vpu_rtcwtc,
	.rwtctblsize = ARRAY_SIZE(vpu_rtcwtc),
	.dcstat_support = true,
};

static struct peri_params vpu_fclk_params_gate = {
	.clktbl = vpu_fclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(vpu_fclk_tbl_gate),
	.comclk_name = "VPUACLK",
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
	.rwtctbl = vpu_rtcwtc,
	.rwtctblsize = ARRAY_SIZE(vpu_rtcwtc),
	.dcstat_support = true,
};

#define GC_ACLK_EN	(0x1 << 3)
#define GC_FCLK_EN	(0x1 << 4)
#define GC_HCLK_EN	(0x1 << 5)
static const char *gc_parents_common[] = {
	"pll1_416m", "pll1_624",
	"pll2", "pll2p",
};

static const char *gc_parents_gate[] = {
	"pll1_416_gate", "pll1_624_gate",
	"pll2", "pll2p",
};

static struct periph_clk_tbl gc_aclk_tbl_common[] = {
	{.clk_rate = 78000000, .parent_name = "pll1_624"},
	{.clk_rate = 104000000, .parent_name = "pll1_416m"},
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_416m"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 416000000, .parent_name = "pll1_416m"},
};

static struct periph_clk_tbl gc_aclk_tbl_gate[] = {
	{.clk_rate = 78000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 104000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 156000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 208000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 312000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 416000000, .parent_name = "pll1_416_gate"},
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

static struct peri_params gc_aclk_params_common = {
	.clktbl = gc_aclk_tbl_common,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl_common),
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_params gc_aclk_params_gate = {
	.clktbl = gc_aclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl_gate),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct periph_clk_tbl gc_fclk_tbl_common[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416m",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 624000000,
		.parent_name = "pll1_624",
		.comclk_rate = 416000000,
	},
};

static struct periph_clk_tbl gc_fclk_tbl_gate[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416_gate",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 624000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 416000000,
	},
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

static struct peri_params gc_fclk_params_common = {
	.clktbl = gc_fclk_tbl_common,
	.clktblsize = ARRAY_SIZE(gc_fclk_tbl_common),
	.comclk_name = "GC_ACLK",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
	.dcstat_support = true,
};

static struct peri_params gc_fclk_params_gate = {
	.clktbl = gc_fclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(gc_fclk_tbl_gate),
	.comclk_name = "GC_ACLK",
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
	.dcstat_support = true,
};

static struct periph_clk_tbl gc2d_aclk_tbl_common[] = {
	{.clk_rate = 78000000, .parent_name = "pll1_624"},
	{.clk_rate = 104000000, .parent_name = "pll1_416m"},
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_416m"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 416000000, .parent_name = "pll1_416m"},
};

static struct periph_clk_tbl gc2d_aclk_tbl_gate[] = {
	{.clk_rate = 78000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 104000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 156000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 208000000, .parent_name = "pll1_416_gate"},
	{.clk_rate = 312000000, .parent_name = "pll1_624_gate"},
	{.clk_rate = 416000000, .parent_name = "pll1_416_gate"},
};

static struct peri_params gc2d_aclk_params_common = {
	.clktbl = gc2d_aclk_tbl_common,
	.clktblsize = ARRAY_SIZE(gc2d_aclk_tbl_common),
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_params gc2d_aclk_params_gate = {
	.clktbl = gc2d_aclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(gc2d_aclk_tbl_gate),
	.inputs = periph_mux_sel_gate,
	.inputs_size = ARRAY_SIZE(periph_mux_sel_gate),
};

static struct periph_clk_tbl gc2d_fclk_tbl_common[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416m",
		.comclk_rate = 312000000,
	},
};

static struct periph_clk_tbl gc2d_fclk_tbl_gate[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416_gate",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 624000000,
		.parent_name = "pll1_624_gate",
		.comclk_rate = 416000000,
	},
};

static struct peri_params gc2d_fclk_params_common = {
	.clktbl = gc2d_fclk_tbl_common,
	.clktblsize = ARRAY_SIZE(gc2d_fclk_tbl_common),
	.comclk_name = "GC2D_ACLK",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
	.dcstat_support = true,
};

static struct peri_params gc2d_fclk_params_gate = {
	.clktbl = gc2d_fclk_tbl_gate,
	.clktblsize = ARRAY_SIZE(gc2d_fclk_tbl_gate),
	.comclk_name = "GC2D_ACLK",
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

#define LCD_CI_ISP_ACLK_EN		(1 << 3)
#define LCD_CI_ISP_ACLK_RST		(1 << 16)
static const char *lcd_ci_isp_parents[] = {"pll1_416m", "pll1_624",
					"pll2", "pll2p",};
static struct peri_reg_info lcd_ci_isp_axi_reg = {
	.src_sel_shift = 17,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 19,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 22,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (LCD_CI_ISP_ACLK_RST | LCD_CI_ISP_ACLK_EN),
	.disable_val = LCD_CI_ISP_ACLK_EN,
};

static struct peri_params lcd_ci_isp_axi_params = {
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

#define ISP_DXO_CLK_EN		\
	((1 << 1) | (1 << 9) | (1 << 11))
#define ISP_DXO_CLK_RST		\
	((1 << 0) | (1 << 8) | (1 << 10))

static struct peri_reg_info isp_dxo_reg = {
	.src_sel_shift = 2,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 4,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 7,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (ISP_DXO_CLK_EN | ISP_DXO_CLK_RST),
	.disable_val = ISP_DXO_CLK_EN,
};

static struct peri_params isp_dxo_params = {
	.comclk_name = "LCDCIISPAXI",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

/* CCIC */
#define CCIC_AXI_CLK_EN		(1 << 3)
#define CCIC_AXI_CLK_RST	(1 << 0)
#define CCIC_FN_CLK_REQ		(1 << 15)
#define CCIC_FN_CLK_EN		(1 << 4)
#define CCIC_FN_CLK_RST		(1 << 1)
#define CCIC_PHY_CLK_EN		\
	((1 << 5) | (1 << 8) | (1 << 9))
#define CCIC_PHY_CLK_DIS	\
	((1 << 5) | (1 << 9))
#define CCIC_PHY_CLK_RST	(1 << 2)

static struct peri_reg_info ccic_axi_reg = {
	.enable_val = (CCIC_AXI_CLK_EN | CCIC_AXI_CLK_RST),
	.disable_val = CCIC_AXI_CLK_EN,
};
static struct peri_params ccic_axi_params = {
	.comclk_name = "LCDCIISPAXI",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_reg_info ccic_func_reg = {
	.src_sel_shift = 16,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 18,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 15,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_CLK_RST),
	.disable_val = CCIC_FN_CLK_EN,
};
static struct peri_params ccic_func_params = {
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_reg_info ccic_phy_reg = {
	.src_sel_shift = 7,
	.src_sel_mask = MASK(REG_WIDTH_1BIT),
	.div_shift = 10,
	.div_mask = MASK(REG_WIDTH_5BIT),
	.enable_val = (CCIC_PHY_CLK_EN | CCIC_PHY_CLK_RST),
	.disable_val = CCIC_PHY_CLK_DIS,
};
static struct peri_params ccic_phy_params = {
	.inputs = ccic_phy_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic_phy_mux_sel),
};

/* CCIC2 for pxa1L88 */
static struct peri_reg_info ccic2_axi_reg = {
	.enable_val = (CCIC_AXI_CLK_EN | CCIC_AXI_CLK_RST),
	.disable_val = CCIC_AXI_CLK_EN,
};
static struct peri_params ccic2_axi_params = {
	.comclk_name = "LCDCIISPAXI",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_reg_info ccic2_func_reg = {
	.src_sel_shift = 16,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 18,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 15,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_CLK_RST),
	.disable_val = CCIC_FN_CLK_EN,
};
static struct peri_params ccic2_func_params = {
	.comclk_name = "CCICFUNCLK_0",
	.inputs = periph_mux_sel,
	.inputs_size = ARRAY_SIZE(periph_mux_sel),
};

static struct peri_reg_info ccic2_phy_reg = {
	.src_sel_shift = 7,
	.src_sel_mask = MASK(REG_WIDTH_1BIT),
	.div_shift = 10,
	.div_mask = MASK(REG_WIDTH_5BIT),
	.enable_val = (CCIC_PHY_CLK_EN | CCIC_PHY_CLK_RST),
	.disable_val = CCIC_PHY_CLK_DIS,
};
static struct peri_params ccic2_phy_params = {
	.inputs = ccic_phy_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic_phy_mux_sel),
};

struct plat_clk_list peri_clk_list[] = {
	{ "sdhci-pxav3.0", NULL, 208000000},
	{ "sdhci-pxav3.1", NULL, 208000000},
	{ "sdhci-pxav3.2", NULL, 104000000},
	{ NULL, "VPUACLK", 416000000},
	{ NULL, "VPUCLK", 416000000},
	{ NULL, "GC_ACLK", 312000000},
	{ NULL, "GCCLK", 416000000},
	{ NULL, "GC2D_ACLK", 312000000},
	{ NULL, "GC2DCLK", 416000000},
	{ NULL, "GC_SHADER_CLK", 416000000},
	{ NULL, "LCDCIISPAXI", 208000000},
	{ NULL, "isp-clk", 312000000},
};

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

static void __init pxa988_ccic_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;
	/* ccic */
	clk = mmp_clk_register_peri("ccic_axi_clk", lcd_ci_isp_parents,
		ARRAY_SIZE(lcd_ci_isp_parents), 0, apmu_base + APMU_CCIC0,
		&ccic0_lock, &ccic_axi_params, &ccic_axi_reg);
	clk_register_clkdev(clk, "CCICAXICLK", "mmp-camera.0");

	/* ccic2 depend on this func clk:CCICFUNCLK_0 */
	clk = mmp_clk_register_peri("ccic_func_clk", ccic_fn_parent,
		ARRAY_SIZE(ccic_fn_parent), CLK_SET_RATE_GATE,
		apmu_base + APMU_CCIC0, &ccic0_lock, &ccic_func_params,
		&ccic_func_reg);
	clk_register_clkdev(clk, "CCICFUNCLK_0", NULL);

	clk = mmp_clk_register_peri("ccic_phy_clk", ccic_phy_parent,
		ARRAY_SIZE(ccic_phy_parent), CLK_SET_RATE_GATE,
		apmu_base + APMU_CCIC0, &ccic0_lock, &ccic_phy_params,
		&ccic_phy_reg);
	clk_register_clkdev(clk, "CCICPHYCLK", "mmp-camera.0");

	clk = mmp_clk_register_apmu("ccic_phy_slow_clk", "NULL",
		apmu_base + APMU_CCIC0, (0x1A << 10) | (0x1 << 7),
		 &ccic0_lock);
	clk_register_clkdev(clk, "CCICSPHYCLK", "mmp-camera.0");
	clk_prepare_enable(clk);
	if (cpu_is_pxa1L88()) {
		/* ccic2 */
		clk = mmp_clk_register_peri("ccic2_axi_clk",
			lcd_ci_isp_parents, ARRAY_SIZE(lcd_ci_isp_parents),
			0, apmu_base + APMU_CCIC1, &ccic1_lock,
			&ccic2_axi_params, &ccic2_axi_reg);
		clk_register_clkdev(clk, "CCICAXICLK", "mmp-camera.1");

		clk = mmp_clk_register_peri("ccic2_func_clk", ccic_fn_parent,
			ARRAY_SIZE(ccic_fn_parent), CLK_SET_RATE_GATE,
			apmu_base + APMU_CCIC1, &ccic1_lock,
			&ccic2_func_params, &ccic2_func_reg);
		clk_register_clkdev(clk, "CCICFUNCLK", "mmp-camera.1");

		clk = mmp_clk_register_peri("ccic2_phy_clk", ccic_phy_parent,
			ARRAY_SIZE(ccic_phy_parent), CLK_SET_RATE_GATE,
			apmu_base + APMU_CCIC1, &ccic1_lock, &ccic2_phy_params,
			&ccic2_phy_reg);
		clk_register_clkdev(clk, "CCICPHYCLK", "mmp-camera.1");

		clk = mmp_clk_register_apmu("ccic2_phy_slow_clk", "NULL",
				apmu_base + APMU_CCIC1,
				(0x1A << 10) | (0x1 << 7), &ccic1_lock);
		clk_register_clkdev(clk, "CCICSPHYCLK", "mmp-camera.1");
		clk_prepare_enable(clk);
	}
}

static void __init clk_peri_init(void __iomem *apmu_base, void __iomem *ciu_base)
{
	struct clk *clk;
	const char const **vpu_par, **gc_par;
	int vpu_par_size, gc_par_size;
	struct peri_params *vpua_parm, *vpuf_parm, *gca_parm, *gcf_parm,
			   *gc2da_parm, *gc2df_parm;

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

	if (cpu_is_pxa1L88()) {
		vpu_par = vpu_parents_gate;
		vpu_par_size = ARRAY_SIZE(vpu_parents_gate);
		gc_par = gc_parents_gate;
		gc_par_size = ARRAY_SIZE(gc_parents_gate);
		vpua_parm = &vpu_aclk_params_gate;
		vpuf_parm = &vpu_fclk_params_gate;
		gca_parm = &gc_aclk_params_gate;
		gcf_parm = &gc_fclk_params_gate;
		gc2da_parm = &gc2d_aclk_params_gate;
		gc2df_parm = &gc2d_fclk_params_gate;
	} else {
		vpu_par = vpu_parents_common;
		vpu_par_size = ARRAY_SIZE(vpu_parents_common);
		gc_par = gc_parents_common;
		gc_par_size = ARRAY_SIZE(gc_parents_common);
		vpua_parm = &vpu_aclk_params_common;
		vpuf_parm = &vpu_fclk_params_common;
		vpuf_parm->rwtctbl = vpu_rtcwtc_1088;
		vpuf_parm->rwtctblsize = ARRAY_SIZE(vpu_rtcwtc_1088);
		gca_parm = &gc_aclk_params_common;
		gcf_parm = &gc_fclk_params_common;
		gc2da_parm = &gc2d_aclk_params_common;
		gc2df_parm = &gc2d_fclk_params_common;
	}
	clk = mmp_clk_register_peri("vpu_aclk", vpu_par,
		vpu_par_size, 0, apmu_base + APMU_VPU_CLK_RES_CTRL,
		&vpu_lock, vpua_parm, &vpu_aclk_reg);
	clk_register_clkdev(clk, "VPUACLK", NULL);

	vpu_fclk_reg.reg_rtcwtc = ciu_base + VPU_XTC;
	clk = mmp_clk_register_peri("vpu", vpu_par,
		vpu_par_size, 0, apmu_base + APMU_VPU_CLK_RES_CTRL,
		&vpu_lock, vpuf_parm, &vpu_fclk_reg);
	clk_register_clkdev(clk, "VPUCLK", NULL);
	clks[vpu] = clk;

	/* GC 3D clock */
	clk = mmp_clk_register_peri("gc_aclk", gc_par,
		gc_par_size, 0, apmu_base + APMU_GC,
		&gc_lock, gca_parm, &gc_aclk_reg);
	clk_register_clkdev(clk, "GC_ACLK", NULL);

	clk = mmp_clk_register_peri("gc", gc_par,
		gc_par_size, 0, apmu_base + APMU_GC,
		&gc_lock, gcf_parm, &gc_fclk_reg);
	clk_register_clkdev(clk, "GCCLK", NULL);
	clks[gc] = clk;

	/* GC 2D clock */
	clk = mmp_clk_register_peri("gc2d_aclk", gc_par,
		gc_par_size, 0, apmu_base + APMU_GC2D,
		&gc2d_lock, gc2da_parm, &gc_aclk_reg);
	clk_register_clkdev(clk, "GC2D_ACLK", NULL);

	clk = mmp_clk_register_peri("gc2d", gc_par,
		gc_par_size, 0, apmu_base + APMU_GC2D,
		&gc2d_lock, gc2df_parm, &gc_fclk_reg);
	clk_register_clkdev(clk, "GC2DCLK", NULL);
	clks[gc2d] = clk;
	if (cpu_is_pxa1L88()) {
		/* GC shader clock, share same register with GC3D */
		clk = mmp_clk_register_peri("gc_shader", gc_par,
			gc_par_size, 0, apmu_base + APMU_GC,
			&gc_lock, &gcshader_clk_params, &gcshader_clk_reg);
		clk_register_clkdev(clk, "GC_SHADER_CLK", NULL);
		clks[gcshader] = clk;
	}
	clk = mmp_clk_register_peri("lcd_ci_isp_axi", lcd_ci_isp_parents,
		ARRAY_SIZE(lcd_ci_isp_parents), 0, apmu_base + APMU_LCD,
		&lcd_ci_share_lock, &lcd_ci_isp_axi_params, &lcd_ci_isp_axi_reg);
	clk_register_clkdev(clk, "LCDCIISPAXI", NULL);

	clk = mmp_clk_register_peri("isp_dxo", lcd_ci_isp_parents,
		ARRAY_SIZE(lcd_ci_isp_parents), 0, apmu_base + APMU_ISPDXO,
		NULL, &isp_dxo_params, &isp_dxo_reg);
	clk_register_clkdev(clk, "isp-clk", NULL);

	peri_init_set_rate(peri_clk_list, ARRAY_SIZE(peri_clk_list));
}

void __init clk_misc_init(void *apmu_base, void *ciu_base)
{
	unsigned int dcg_regval = 0;
	/*
	 * DE suggest:enable SQU MP3 playback sleep mode
	 * APMU_SQU_CLK_GATE_CTRL
	 */
	__raw_writel(__raw_readl(apmu_base + 0x001c) | (1 << 30),
			(apmu_base + 0x001c));

	/* init GC related RTC register here */
	__raw_writel(0x00055555, ciu_base + GPU_XTC);
	__raw_writel(0x00055555, ciu_base + GPU2D_XTC);

	/*
	 * disable nand controller clock as it is not used
	 */
	__raw_writel(0, apmu_base + APMU_DFC);
	/*
	 * disable ase clock at init stage and security will
	 * enable it prior to use it
	 */
	__raw_writel(0, apmu_base + APMU_GEU);

	/* enable MC4 and AXI fabric dynamic clk gating */
	dcg_regval = __raw_readl(ciu_base + CIU_MC_CONF);
	/* disable cp fabric clk gating */
	dcg_regval &= ~(1 << 16);
	/* enable dclk gating */
	dcg_regval &= ~(1 << 19);
	/* enable 1x2 fabric AXI clock dynamic gating */
	dcg_regval |= (1 << 29) | (1 << 30);
	dcg_regval |= (0xff << 8) |	/* MCK4 P0~P7*/
		(1 << 17) | (1 << 18) |	/* Fabric 0 */
		(1 << 20) | (1 << 21) |	/* VPU fabric */
		(1 << 26) | (1 << 27);	/* Fabric 0/1 */
	__raw_writel(dcg_regval, ciu_base + CIU_MC_CONF);
}

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

	np = of_find_compatible_node(NULL, NULL, "marvell,pxa988-clock");
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

	clk = clk_register_fixed_rate(NULL, "clk32", NULL, CLK_IS_ROOT, 32000);
	clk_register_clkdev(clk, "clk32", NULL);

	vct_pll = clk_register_fixed_rate(NULL, "vctcxo", NULL, CLK_IS_ROOT,
				26000000);
	clk_register_clkdev(vct_pll, "vctcxo", NULL);

	round_max_freq();

	pxa988_pll_init(mpmu_base, apbs_base, apmu_base);

	pxa988_acpu_init(apmu_base, mpmu_base, ciu_base, dmcu_base);

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

	clk = clk_register_divider_table(NULL, "twsi0_div",
				"pll1_624", CLK_SET_RATE_PARENT,
				apbc_base + APBC_TWSI0, 4, 3, 0,
				clk_twsi_ref_table, NULL);
	clk_register_clkdev(clk, "twsi0_div", NULL);

	clk = mmp_clk_register_apbc("twsi0", "twsi0_div",
				apbc_base + APBC_TWSI0, 10, 0, NULL);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.0");

	clk = clk_register_divider_table(NULL, "twsi1_div",
				"pll1_624", CLK_SET_RATE_PARENT,
				apbc_base + APBC_TWSI1, 4, 3, 0,
				clk_twsi_ref_table, NULL);
	clk_register_clkdev(clk, "twsi1_div", NULL);

	clk = mmp_clk_register_apbc("twsi1", "twsi1_div",
				apbc_base + APBC_TWSI1, 10, 0, NULL);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.1");

	clk = clk_register_divider_table(NULL, "twsi2_div",
				"pll1_624", CLK_SET_RATE_PARENT,
				apbcp_base + APBCP_TWSI2, 3, 2, 0,
				clk_twsi_ref_table, NULL);
	clk_register_clkdev(clk, "twsi2_div", NULL);

	clk = mmp_clk_register_apbc("twsi2", "twsi2_div",
				apbcp_base + APBCP_TWSI2, 10, 0, NULL);
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

	clk = mmp_clk_register_apbc("thermal", NULL,
				apbc_base + APBC_DROTS, 10, 0, &clk_lock);
	clk_register_clkdev(clk, "THERMALCLK", NULL);

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
	clk = clk_register_gate(NULL, "isccr0_i2sclk_base", "pllclk_2",
				CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0, 30, 0, NULL);
	clk_register_clkdev(clk, "isccr0_i2sclk_base", NULL);

	clk = clk_register_gate(NULL, "isccr0_sysclk_en", "isccr0_i2sclk_base",
				CLK_SET_RATE_PARENT,
				mpmu_base + MPMU_ISCCR0, 31, 0, NULL);
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
	clk_register_clkdev(clk, NULL, "pxa-u2oehci");
	clk_register_clkdev(clk, NULL, "pxa988-usb-phy");

	clk = mmp_clk_register_apmu("sph", "usb_pll",
				apmu_base + APMU_USB, 0x12, &clk_lock);
	clk_register_clkdev(clk, "sph_clk", NULL);


	pxa988_disp_clk_init(apmu_base);

	pxa988_ccic_clk_init(apmu_base);

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
	reg = __raw_readl(apmu_base + APMU_ISPDXO);
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
