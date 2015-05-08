/*
 * eden clock framework source file
 *
 * Copyright (C) 2013 Marvell
 * Guoqing <ligq@marvell.com>
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
#include <linux/of.h>

#include "clk.h"

#define APBC_RTC	0x0
#define APBC_TWSI0	0x4
#define APBC_TWSI1	0x8
#define APBC_TWSI2	0xc
#define APBC_TWSI3	0x10
#define APBC_TWSI4	0x7c
#define APBC_TWSI5	0x80
#define APBC_KPC	0x18
#define APBC_UART0	0x2c
#define APBC_UART1	0x30
#define APBC_UART2	0x34
#define APBC_UART3	0x88
#define APBC_GPIO	0x38
#define APBC_PWM0	0x3c
#define APBC_PWM1	0x40
#define APBC_PWM2	0x44
#define APBC_PWM3	0x48
#define APBC_SSP0	0x4c
#define APBC_SSP1	0x50
#define APBC_SSP2	0x54
#define APBC_SSP3	0x58
#define APBC_SSP4	0x5c
#define APBC_SSP5	0x60
#define APBC_THSENS	0x90 /* Thermal Sensor */

#define APMU_SDH0	0x54
#define APMU_SDH1	0x58
#define APMU_SDH2	0xe8
#define APMU_SDH3	0xec
#define APMU_USB	0x5c
#define APMU_DISP0	0x4c
#define APMU_DISP1	0x110
#define APMU_CCIC0	0x50
#define APMU_CCIC1	0xf4
#define APMU_GC		0xcc
#define APMU_GC2	0x27c
#define APMU_VPU	0xa4
#define APMU_COREAPSS	0x2e0
#define APMU_APDBG	0x340
#define APMU_AP_DEBUG1	0x38c
#define APMU_SMC	0xd4

#define MPMU_UART_PLL	0x14

#define POSR_PLL2_LOCK		(1 << 29)
#define POSR_PLL3_LOCK		(1 << 14)
#define POSR_PLL4_LOCK		(1 << 15)
#define POSR_PLL5_LOCK		(1 << 16)
#define POSR_PLL6_LOCK		(1 << 14)
#define POSR_PLL7_LOCK		(1 << 17)

#define APMU_MC_CLK_RES_CTRL	(0x258)
#define MPMU_PLL2CR	(0x0034)
#define MPMU_PLL3CR	(0x0050)
#define MPMU_PLL4CR	(0x1100)
#define MPMU_PLL5CR	(0x1114)
#define MPMU_PLL6CR	(0x1300)
#define MPMU_PLL7CR	(0x1400)
#define MPMU_POSR	(0x0010)
#define MPMU_POSR2	(0x0054)
#define MPMU_POSR3	(0x0074)
#define MPMU_PLL2_CTRL1	(0x0414)
#define MPMU_PLL3_CTRL1	(0x0058)
#define MPMU_PLL4_CTRL1	(0x1104)
#define MPMU_PLL5_CTRL1	(0x1118)
#define MPMU_PLL6_CTRL1	(0x1304)
#define MPMU_PLL7_CTRL1	(0x1404)
#define MPMU_PLL2_CTRL2	(0x0418)
#define MPMU_PLL3_CTRL2	(0x0060)
#define MPMU_PLL4_CTRL2	(0x1108)
#define MPMU_PLL5_CTRL2	(0x111c)
#define MPMU_PLL6_CTRL2	(0x1308)
#define MPMU_PLL7_CTRL2	(0x1408)
#define MPMU_PLL2_CTRL3	(0x041c)
#define MPMU_PLL3_CTRL3	(0x0064)
#define MPMU_PLL4_CTRL3	(0x110c)
#define MPMU_PLL5_CTRL3	(0x1120)
#define MPMU_PLL6_CTRL3	(0x130c)
#define MPMU_PLL7_CTRL3	(0x140c)
#define MPMU_PLL2_CTRL4	(0x0068)
#define MPMU_PLL3_CTRL4	(0x006c)
#define MPMU_PLL4_CTRL4	(0x1110)
#define MPMU_PLL5_CTRL4	(0x1124)
#define MPMU_PLL6_CTRL4	(0x1310)
#define MPMU_PLL7_CTRL4	(0x1410)
#define APMU_AP_PLL_CTRL	(0x0348)
#define GATE_CTRL_SHIFT_PLL2	(22)
#define GATE_CTRL_SHIFT_PLL3	(24)
#define GATE_CTRL_SHIFT_PLL4	(26)
#define GATE_CTRL_SHIFT_PLL5	(28)
#define GATE_CTRL_SHIFT_PLL6	(30)
#define GATE_CTRL_SHIFT_PLL7	(16)

#define APMU_DISP1_CLK_CTRL (0x004c)
#define APMU_DISP2_CLK_CTRL (0x0110)
#define LCD_PN_SCLK	(0xd420b1a8)

/* APMU of EDEN Ax */
#define APMU_DISP_RST_CTRL	(0x180)
#define APMU_DISP_CLK_CTRL	(0x184)

#define APMU_AUDIO_PWR_UP		(3 << 9)
#define APMU_AUDIO_PWR_DOWN		(0 << 9)
#define APMU_AUDIO_ISO_DIS		(1 << 8)
#define APMU_AUDIO_CLK_ENA		(1 << 4)
#define APMU_AUDIO_RST_DIS		(1 << 1)

struct pll_tbl {
	spinlock_t *lock;
	const char *vco_name;
	const char *out_name;
	const char *outp_name;
	unsigned long out_flags;
	unsigned long outp_flags;
	struct eden_clk_pll_vco_table  *vco_tbl;
	struct eden_clk_pll_out_table  *out_tbl;
	struct eden_clk_pll_out_table  *outp_tbl;
	struct eden_clk_pll_vco_params vco_params;
	struct eden_clk_pll_out_params out_params;
	struct eden_clk_pll_out_params outp_params;
};

static DEFINE_SPINLOCK(clk_lock);
static DEFINE_SPINLOCK(pll2_lock);
static DEFINE_SPINLOCK(pll3_lock);
static DEFINE_SPINLOCK(pll4_lock);
static DEFINE_SPINLOCK(pll5_lock);
static DEFINE_SPINLOCK(pll6_lock);
static DEFINE_SPINLOCK(pll7_lock);
static DEFINE_SPINLOCK(disp_lock);
static DEFINE_SPINLOCK(gc3d_lock);
static DEFINE_SPINLOCK(gc2d_lock);
#ifdef CONFIG_UIO_HANTRO
static DEFINE_SPINLOCK(vpu_lock);
#endif

#define REG_WIDTH_1BIT	1
#define REG_WIDTH_2BIT	2
#define REG_WIDTH_3BIT	3
#define REG_WIDTH_4BIT	4
#define REG_WIDTH_5BIT	5

/* use to save some important clk ptr */
enum edenx_clk {
	cpu = 0, ddr, axi, gc3d_aclk, gc2d_aclk,
	gc3d_1x, gc3d_2x, gc2d,
	vpu_aclk, vpu_dec, vpu_enc, isp,
	clk_max,
};
static struct clk *clks[clk_max];

static struct clk_factor_masks uart_factor_masks = {
	.factor = 2,
	.num_mask = 0x1fff,
	.den_mask = 0x1fff,
	.num_shift = 16,
	.den_shift = 0,
};

static struct clk_factor_tbl uart_factor_tbl[] = {
	{.num = 832, .den = 234},	/*58.5MHZ */
	{.num = 1, .den = 1},		/*26MHZ */
};

static const char *uart_parent[] = {"uart_pll", "vctcxo"};
static const char *disp1_parent[] = {"pll1_624", "pll5p", "pll5", "pll1_416"};
static const char *disp1_parent_ax[] = {"pll1_624", "pll1_416"};
static const char *pnsclk_parent[] = {"disp1", "pll3"};
static const char *pnpath_parent[] = {"pn_sclk"};
static const char *pnsclk_depend[] = {"LCDCIHCLK", "vdma_axi"};
static const char *dsi_depend[] = {"dsi_phy_slow", "LCDCIHCLK"};
static const char *root_depend[] = {"disp_rst_ctrl"};
static const char *ccic_fn_parent[] = {"pll1_624", "pll5p", "pll5", "pll1_416"};
static const char *ccic_sphy_parent[] = {"vctcxo"};
static u32 pnsclk_parent_tbl[] = {1, 7};
#ifdef CONFIG_SOUND
static const char *sspa1_parent1[] = {
	"apll1_slow", "apll1_fast",
	"apll2_slow", "apll2_fast",
	"ext_pll_clk1", "ext_pll_clk2"
};
static const char *sspa1_parent2[] = {"sspa1_mn_div", "vctcxo"};
static const char *sspa1_parent3[] = {
	"i2s_ptk_apb_clk", "sspa1_clk_src"
};
static const char *sysclk_parent[] = {"sspa1_div_clk", "sspa2_div_clk"};
#endif
static const char *sdh_parent[] = {"pll1_624", "pll5p", "pll5", "pll1_416"};

static struct eden_clk_pll_vco_table pll2_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 1594666666, 1594666666, 3, 46, 3, 0xa, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_vco_table pll3_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 1768000000, 1768000000, 3, 51, 3, 0xb, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_vco_table pll4_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 2114666666, 2133333333, 3, 61, 3, 0xc, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_vco_table pll5_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 2114666666, 2133333333, 3, 61, 3, 0xc, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_vco_table pll6_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 2114666666, 2133333333, 3, 61, 3, 0xc, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_vco_table pll7_vco_table[] = {
	/* input  out  out_offsted refdiv fbdiv icp kvco ssc_en offset_en*/
	{26000000, 2114666666, 2133333333, 3, 61, 3, 0xc, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0, 0}
};

static struct eden_clk_pll_out_table pll2_out_table[] = {
	/* input_rate output_rate div_sel */
	{1594666666, 797333333, 1},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll2_outp_table[] = {
	/* input_rate output_rate div_sel */
	{1594666666, 531555555, EDEN_PLL_DIV_3},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll3_out_table[] = {
	/* input_rate output_rate div_sel */
	{1768000000, 884000000, 1},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll3_outp_table[] = {
	/* input_rate output_rate div_sel */
	{1768000000, 442000000, 2},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll4_out_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 1057333333, 1},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll4_outp_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 528666666, 2},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll5_out_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 528666666, 2},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll5_outp_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 1057333333, 1},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll6_out_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 528666666, 2},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll6_outp_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 1057333333, 1},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll7_out_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 528666666, 2},
	{0, 0, 0},
};

static struct eden_clk_pll_out_table pll7_outp_table[] = {
	/* input_rate output_rate div_sel */
	{2114666666, 1057333333, 1},
	{0, 0, 0},
};

static struct pll_tbl pllx_tbl[] = {
	{
		.lock = &pll2_lock,
		.vco_name = "pll2_vco",
		.out_name = "pll2",
		.outp_name = "pll2p",
		.vco_tbl = pll2_vco_table,
		.out_tbl = pll2_out_table,
		.outp_tbl = pll2_outp_table,
		.out_flags = 0,
		.outp_flags = EDEN_PLL_USE_DIV_3 | EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL2CR, MPMU_PLL2_CTRL1,
			MPMU_PLL2_CTRL2, MPMU_PLL2_CTRL3, MPMU_PLL2_CTRL4,
			MPMU_POSR, POSR_PLL2_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL2
		},
		.out_params = {
			1500000000, MPMU_PLL2_CTRL1, 3, 26, MPMU_PLL2_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL2_CTRL4, 3, 5, MPMU_PLL2_CTRL1
		}
	},
	{
		.lock = &pll3_lock,
		.vco_name = "pll3_vco",
		.out_name = "pll3",
		.outp_name = "pll3p",
		.vco_tbl = pll3_vco_table,
		.out_tbl = pll3_out_table,
		.outp_tbl = pll3_outp_table,
		.out_flags = 0,
		.outp_flags = EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL3CR, MPMU_PLL3_CTRL1,
			MPMU_PLL3_CTRL2, MPMU_PLL3_CTRL3, MPMU_PLL3_CTRL4,
			MPMU_POSR2, POSR_PLL3_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL3
		},
		.out_params = {
			1500000000, MPMU_PLL3_CTRL1, 3, 26, MPMU_PLL3_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL3_CTRL4, 3, 5, MPMU_PLL3_CTRL1
		}
	},
	{
		.lock = &pll4_lock,
		.vco_name = "pll4_vco",
		.out_name = "pll4",
		.outp_name = "pll4p",
		.vco_tbl = pll4_vco_table,
		.out_tbl = pll4_out_table,
		.outp_tbl = pll4_outp_table,
		.out_flags = EDEN_PLL_USE_SYNC_DDR,
		.outp_flags = EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL4CR, MPMU_PLL4_CTRL1,
			MPMU_PLL4_CTRL2, MPMU_PLL4_CTRL3, MPMU_PLL4_CTRL4,
			MPMU_POSR2, POSR_PLL4_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL4
		},
		.out_params = {
			1500000000, APMU_MC_CLK_RES_CTRL, 3, 12, MPMU_PLL4_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL4_CTRL4, 3, 5, MPMU_PLL4_CTRL1
		}
	},
	{
		.lock = &pll5_lock,
		.vco_name = "pll5_vco",
		.out_name = "pll5",
		.outp_name = "pll5p",
		.vco_tbl = pll5_vco_table,
		.out_tbl = pll5_out_table,
		.outp_tbl = pll5_outp_table,
		.out_flags = 0,
		.outp_flags = EDEN_PLL_USE_DIV_3 | EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL5CR, MPMU_PLL5_CTRL1,
			MPMU_PLL5_CTRL2, MPMU_PLL5_CTRL3, MPMU_PLL5_CTRL4,
			MPMU_POSR2, POSR_PLL5_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL5
		},
		.out_params = {
			1500000000, MPMU_PLL5_CTRL1, 3, 26, MPMU_PLL5_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL5_CTRL4, 3, 5, MPMU_PLL5_CTRL1
		}
	},
	{
		.lock = &pll6_lock,
		.vco_name = "pll6_vco",
		.out_name = "pll6",
		.outp_name = "pll6p",
		.vco_tbl = pll6_vco_table,
		.out_tbl = pll6_out_table,
		.outp_tbl = pll6_outp_table,
		.out_flags = 0,
		.outp_flags = EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL6CR, MPMU_PLL6_CTRL1,
			MPMU_PLL6_CTRL2, MPMU_PLL6_CTRL3, MPMU_PLL6_CTRL4,
			MPMU_POSR3, POSR_PLL6_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL6
		},
		.out_params = {
			1500000000, MPMU_PLL6_CTRL1, 3, 26, MPMU_PLL6_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL6_CTRL4, 3, 5, MPMU_PLL6_CTRL1
		}
	},
	{
		.lock = &pll7_lock,
		.vco_name = "pll7_vco",
		.out_name = "pll7",
		.outp_name = "pll7p",
		.vco_tbl = pll7_vco_table,
		.out_tbl = pll7_out_table,
		.outp_tbl = pll7_outp_table,
		.out_flags = 0,
		.outp_flags = EDEN_PLL_USE_ENABLE_BIT,
		.vco_params = {
			1500000000, 3000000000UL, MPMU_PLL7CR, MPMU_PLL7_CTRL1,
			MPMU_PLL7_CTRL2, MPMU_PLL7_CTRL3, MPMU_PLL7_CTRL4,
			MPMU_POSR2, POSR_PLL7_LOCK, APMU_AP_PLL_CTRL, 2,
			GATE_CTRL_SHIFT_PLL7
		},
		.out_params = {
			1500000000, MPMU_PLL7_CTRL1, 3, 26, MPMU_PLL7_CTRL1
		},
		.outp_params = {
			1500000000, MPMU_PLL7_CTRL4, 3, 5, MPMU_PLL7_CTRL1
		}
	}
};

static struct mmp_clk_disp disp_rst_ctrl = {
	.reg_rst = APMU_DISP_RST_CTRL,
	.lock = &disp_lock,
};

static struct mmp_clk_disp vdma_axi_ax = {
	.reg_rst = APMU_DISP_CLK_CTRL,
	.reg_rst_shadow = 1 << 24,
	.reg_rst_mask = (7 << 24) | (7 << 28),
	.dependence = root_depend,
	.num_dependence = ARRAY_SIZE(root_depend),
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp_axi_ax = {
	.reg_rst = APMU_DISP_CLK_CTRL,
	.reg_rst_shadow = 1 | (1 << 8) | (1 << 16) | (1 << 24),
	.reg_rst_mask = 1 | (3 << 8) | (3 << 16) | (3 << 24),
	.dependence = root_depend,
	.num_dependence = ARRAY_SIZE(root_depend),
	.lock = &disp_lock,
};

static struct mmp_clk_disp dsi_phy_slow_ax = {
	.reg_rst = APMU_DISP_CLK_CTRL,
	.reg_rst_shadow = 1 << 4,
	.reg_rst_mask = (1 << 4) | (3 << 5),
	.dependence = root_depend,
	.num_dependence = ARRAY_SIZE(root_depend),
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp_axi = {
	.reg_rst = APMU_DISP1_CLK_CTRL,
	.reg_rst_shadow = 1 | (1 << 1) | (1 << 3) | (1 << 4),
	.reg_rst_mask = (1 << 1) | (1 << 3) | (1 << 4),
	.lock = &disp_lock,
};

#define DSI_PHYSLOW_PRER_SHIFT  (15)
#define DSI_ESC_CLK_SEL_SHIFT   (22)
static struct mmp_clk_disp vdma_axi = {
	.reg_rst = APMU_DISP2_CLK_CTRL,
	.reg_rst_shadow = 1 | (1 << 3),
	.reg_rst_mask = (1 << 3),
	.lock = &disp_lock,
};

static struct mmp_clk_disp dsi_phy_slow = {
	.reg_rst = APMU_DISP1_CLK_CTRL,
	.reg_rst_shadow = (1 << 2) | (1 << 5) | (1 << 12) |
			 (0x1a << DSI_PHYSLOW_PRER_SHIFT) |
			 (0 << DSI_ESC_CLK_SEL_SHIFT),
	.reg_rst_mask = (1 << 2) | (1 << 5) | (1 << 12)  |
			(0x1f << DSI_PHYSLOW_PRER_SHIFT) |
			(3 << DSI_ESC_CLK_SEL_SHIFT),
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp1 = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 3,
	.mux.shift = 6,
	.mux.lock = &disp_lock,
	.reg_mux = APMU_DISP1_CLK_CTRL,
	.div_ops = &clk_divider_ops,
	.divider.width = 4,
	.divider.shift = 8,
	.divider.lock = &disp_lock,
	.reg_div = APMU_DISP1_CLK_CTRL,
};

static struct mmp_clk_disp disp1_ax = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 7,
	.mux.shift = 12,
	.mux.lock = &disp_lock,
	.reg_mux = APMU_DISP_CLK_CTRL,
	.div_ops = &clk_divider_ops,
	.divider.width = 3,
	.divider.shift = 8,
	.divider.lock = &disp_lock,
	.reg_div = APMU_DISP_CLK_CTRL,
};

static struct mmp_clk_disp pnsclk = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 7,
	.mux.shift = 29,
	.mux.lock = &disp_lock,
	.mux.table = pnsclk_parent_tbl,
	.dependence = pnsclk_depend,
	.num_dependence = ARRAY_SIZE(pnsclk_depend),
	.reg_mux_shadow = 0x20000000,
};

static struct mmp_clk_disp pnpath = {
	.div_ops = &clk_divider_ops,
	.divider.width = 8,
	.divider.shift = 0,
	.divider.lock = &disp_lock,
	.reg_rst_mask = (1 << 28),
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

static struct clk *get_parent(char *pname, struct clk *default_parent)
{
	struct device_node *np = of_find_node_by_name(NULL, pname);
	struct clk *parent = NULL;
	const char *str = NULL;

	if (np && !of_property_read_string(np, "clksrc", &str))
		parent = clk_get(NULL, str);

	return (IS_ERR_OR_NULL(parent)) ? default_parent : parent;
}

static unsigned int get_pll1_freq(char *pll_name)
{
	struct device_node *np = of_find_node_by_name(NULL, pll_name);
	unsigned int freq;

	if (np && !of_property_read_u32(np, "freq", &freq))
		return freq;

	if (strcmp(pll_name, "pll1_416_freq"))
		return 624000000;

	return 416000000;
}

static int soc_is_ax(void)
{
	struct device_node *np = of_find_node_by_name(NULL, "version");
	const char *str = NULL;

	if (np && !of_property_read_string(np, "ver", &str)) {
		if (!strcmp(str, "ax"))
			return 1;
	}

	return 0;
}

static void __init eden_pll_init(void __iomem *mpmu_base,
		void __iomem *apmu_base)
{
	struct clk *clk;
	int i, pll_num = ARRAY_SIZE(pllx_tbl);

	if (!soc_is_ax())
		/* last two PLLs(PLL6,PLL7) not support for EDEN Zx */
		pll_num -= 2;
	for (i = 0; i < pll_num; i++) {
		clk = eden_clk_register_pll_vco(pllx_tbl[i].vco_name, "vctcxo",
			0, pllx_tbl[i].lock, mpmu_base, apmu_base,
			&pllx_tbl[i].vco_params, pllx_tbl[i].vco_tbl);
		clk_register_clkdev(clk, pllx_tbl[i].vco_name, NULL);
		clk_set_rate(clk, pllx_tbl[i].vco_tbl[0].output_rate);

		clk = eden_clk_register_pll_out(pllx_tbl[i].out_name,
			pllx_tbl[i].vco_name, pllx_tbl[i].out_flags,
			pllx_tbl[i].lock, mpmu_base, apmu_base,
			&pllx_tbl[i].out_params, pllx_tbl[i].out_tbl);
		clk_register_clkdev(clk, pllx_tbl[i].out_name, NULL);
		clk_set_rate(clk, pllx_tbl[i].out_tbl[0].output_rate);

		clk = eden_clk_register_pll_out(pllx_tbl[i].outp_name,
			pllx_tbl[i].vco_name, pllx_tbl[i].outp_flags,
			pllx_tbl[i].lock, mpmu_base, apmu_base,
			&pllx_tbl[i].outp_params, pllx_tbl[i].outp_tbl);
		clk_register_clkdev(clk, pllx_tbl[i].outp_name, NULL);
		clk_set_rate(clk, pllx_tbl[i].outp_tbl[0].output_rate);
	}
}

static void __init eden_disp_clk_init(void __iomem *apmu_base)
{
	struct clk *clk, *clk_disp1, *pll1_416;

	if (soc_is_ax()) {
		clk = mmp_clk_register_disp("disp_rst_ctrl", NULL, 0,
			MMP_DISP_RST_CTRL, CLK_IS_ROOT,
			apmu_base, &disp_rst_ctrl);
		clk_register_clkdev(clk, "disp_rst_ctrl", NULL);

		clk = mmp_clk_register_disp("LCDCIHCLK", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT,
			apmu_base, &disp_axi_ax);
		clk_register_clkdev(clk, "LCDCIHCLK", NULL);

		clk = mmp_clk_register_disp("vdma_axi", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, apmu_base, &vdma_axi_ax);
		clk_register_clkdev(clk, "vdma_axi", NULL);

		clk = mmp_clk_register_disp("dsi_phy_slow", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, apmu_base,
			&dsi_phy_slow_ax);
		clk_register_clkdev(clk, "dsi_phy_slow", NULL);

		clk_disp1 = mmp_clk_register_disp("disp1", disp1_parent_ax,
			ARRAY_SIZE(disp1_parent_ax), 0, CLK_SET_RATE_PARENT,
			apmu_base, &disp1_ax);
	} else {
		clk = mmp_clk_register_disp("LCDCIHCLK", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT,
			apmu_base, &disp_axi);
		clk_register_clkdev(clk, "LCDCIHCLK", NULL);

		clk = mmp_clk_register_disp("vdma_axi", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, apmu_base, &vdma_axi);
		clk_register_clkdev(clk, "vdma_axi", NULL);

		clk = mmp_clk_register_disp("dsi_phy_slow", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, apmu_base,
			&dsi_phy_slow);
		clk_register_clkdev(clk, "dsi_phy_slow", NULL);

		clk_disp1 = mmp_clk_register_disp("disp1", disp1_parent,
			ARRAY_SIZE(disp1_parent), 0, CLK_SET_RATE_PARENT,
			apmu_base, &disp1);
	}
	clk_register_clkdev(clk_disp1, "disp1", NULL);
	pll1_416 = clk_get(NULL, "pll1_416");
	clk_set_parent(clk_disp1, get_parent("disp1_clksrc", pll1_416));

	pnsclk.mux.reg = ioremap(LCD_PN_SCLK, 4);
	clk = mmp_clk_register_disp("pn_sclk", pnsclk_parent,
			ARRAY_SIZE(pnsclk_parent),
			MMP_DISP_MUX_ONLY, CLK_SET_RATE_PARENT, apmu_base,
			&pnsclk);
	clk_register_clkdev(clk, "pn_sclk", NULL);
	clk_set_parent(clk, get_parent("pn_sclk_clksrc", clk_disp1));

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

#define CCIC_AXI_CLK_EN		(1 << 3)
#define CCIC_AXI_RST	(1 << 0)
#define CCIC_FN_CLK_EN		(1 << 4)
#define CCIC_FN_RST		(1 << 1)
#define CCIC_PHY_RST	(1 << 2)
#define CCIC_PHY_CLK_EN	(1 << 5)
#define CCIC_SPHY_RST	(1 << 8)
#define CCIC_SPHY_CLK_EN	(1 << 9)
#define CCIC_SPHY_DIV_OFFSET	10
#define CCIC_SPHY_DIV_MASK	(MASK(5))

#define CCIC_AXI_ARB_CLK_EN	(1 << 15)
#define CCIC2_CCCI1_SYNC_EN	(1 << 15)
#define CCIC_AXI_ARB_RST	(1 << 16)

enum periph_ccic_fn_src {
	CCIC_PLL1_624 = 0x0,
	CCIC_PLL5P = 0x1,
	CCIC_PLL5 = 0x2,
	CCIC_PLL1_416 = 0x3,
};
/* ccic1 & ccic2 use the same ccic_fn_mux_sel */
static struct clk_mux_sel ccic_fn_mux_sel[] = {
	{.parent_name = "pll1_624", .value = CCIC_PLL1_624},
	{.parent_name = "pll5p", .value = CCIC_PLL5P},
	{.parent_name = "pll5", .value = CCIC_PLL5},
	{.parent_name = "pll1_416", .value = CCIC_PLL1_416},
};
static struct peri_params ccic1_func_params = {
	.inputs = ccic_fn_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic_fn_mux_sel),
};
static struct peri_params ccic2_func_params = {
	.comclk_name = "CCIC_SYNC_EN",
	.inputs = ccic_fn_mux_sel,
	.inputs_size = ARRAY_SIZE(ccic_fn_mux_sel),
};
static struct peri_reg_info ccic1_func_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 17,
	.div_mask = MASK(REG_WIDTH_4BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_RST),
	.disable_val = CCIC_FN_CLK_EN,
};
static struct peri_reg_info ccic2_func_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 16,
	.div_mask = MASK(REG_WIDTH_4BIT),
	.enable_val = (CCIC_FN_CLK_EN | CCIC_FN_RST),
	.disable_val = CCIC_FN_CLK_EN,
};

/* ccic1 & ccic2 use the same ccic_phy_reg */
static struct peri_reg_info ccic_phy_reg = {
	.src_sel_shift = 9,
	.src_sel_mask = MASK(REG_WIDTH_1BIT),
	.div_shift = 10,
	.div_mask = MASK(REG_WIDTH_5BIT),
	.enable_val = (CCIC_SPHY_CLK_EN | CCIC_SPHY_RST |
				   CCIC_PHY_CLK_EN | CCIC_PHY_RST),
	.disable_val = (CCIC_SPHY_CLK_EN | CCIC_PHY_CLK_EN),
};

static struct clk_mux_sel ccic_phy_mux_sel[] = {
	{.parent_name = "pll1_624", .value = CCIC_PLL1_624},
	{.parent_name = "vctcxo", .value = 1},
};

static struct peri_params ccic_phy_params = {
	.inputs = ccic_phy_mux_sel,
	.inputs_size =  ARRAY_SIZE(ccic_phy_mux_sel),
};

static void __init eden_ccic_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;
	struct device_node *np = of_find_node_by_name(NULL, "ccic_couple");
	const u32 *tmp;
	int len;
	int sync = 0;

	if (np) {
		tmp = of_get_property(np, "ccic_coupled", &len);
		if (tmp)
			sync = be32_to_cpup(tmp);
	}

	/* ccic1 & ccic2 will use the same axi arb clk */
	clk = mmp_clk_register_apmu("ccic_axi_arb_clk", "NULL",
			apmu_base + APMU_CCIC0, 0x18000, &clk_lock);
	clk_register_clkdev(clk, "CCICARBCLK", NULL);

	clk = mmp_clk_register_apmu("ccic1_axi_clk", "CCICARBCLK",
			apmu_base + APMU_CCIC0, 0x9, &clk_lock);
	clk_register_clkdev(clk, "CCICAXICLK", "mmp-camera.0");

	clk = mmp_clk_register_apmu("ccic2_axi_clk", "CCICARBCLK",
			apmu_base + APMU_CCIC1, 0x9, &clk_lock);
	clk_register_clkdev(clk, "CCICAXICLK", "mmp-camera.1");

	/* CCIC1 & CCIC2 sync enable clk */
	clk = mmp_clk_register_apmu("ccic1_ccic2_sync", "NULL",
			apmu_base + APMU_CCIC1, 0x8000, &clk_lock);
	clk_register_clkdev(clk, "CCIC_SYNC_EN", NULL);
	clk_prepare_enable(clk);
	clk_disable_unprepare(clk);

	if (sync) {
		/* ccic2 depends on this func clk:CCICFUNCLK_0 */
		clk = mmp_clk_register_peri("ccic1_func_clk", ccic_fn_parent,
			ARRAY_SIZE(ccic_fn_parent), 0, apmu_base + APMU_CCIC0,
			&clk_lock, &ccic1_func_params, &ccic1_func_reg);
		clk_register_clkdev(clk, "CCICFUNCLK", "mmp-camera.0");

		clk = mmp_clk_register_peri("ccic2_func_clk", ccic_fn_parent,
			ARRAY_SIZE(ccic_fn_parent), 0, apmu_base + APMU_CCIC0,
			&clk_lock, &ccic2_func_params, &ccic1_func_reg);
		clk_register_clkdev(clk, "CCICFUNCLK", "mmp-camera.1");
	} else {
		clk = mmp_clk_register_peri("ccic1_func_clk", ccic_fn_parent,
			ARRAY_SIZE(ccic_fn_parent), 0, apmu_base + APMU_CCIC0,
			&clk_lock, &ccic1_func_params, &ccic1_func_reg);
		clk_register_clkdev(clk, "CCICFUNCLK", "mmp-camera.0");

		clk = mmp_clk_register_peri("ccic2_func_clk", ccic_fn_parent,
			ARRAY_SIZE(ccic_fn_parent), 0, apmu_base + APMU_CCIC1,
			&clk_lock, &ccic1_func_params, &ccic2_func_reg);
		clk_register_clkdev(clk, "CCICFUNCLK", "mmp-camera.1");
	}

	clk = mmp_clk_register_peri("ccic1_phy_clk", ccic_sphy_parent,
		ARRAY_SIZE(ccic_sphy_parent), 0, apmu_base + APMU_CCIC0,
		&clk_lock, &ccic_phy_params, &ccic_phy_reg);
	clk_register_clkdev(clk, "CCICPHYCLK", "mmp-camera.0");

	clk = mmp_clk_register_peri("ccic2_phy_clk", ccic_sphy_parent,
		ARRAY_SIZE(ccic_sphy_parent), 0, apmu_base + APMU_CCIC1,
		&clk_lock, &ccic_phy_params, &ccic_phy_reg);
	clk_register_clkdev(clk, "CCICPHYCLK", "mmp-camera.1");

	return;
}


#ifdef CONFIG_SOUND
static void __init eden_audio_clk_init(void __iomem *apmu_base,
	void __iomem *audio_base, void __iomem *audio_aux_base,
	poweron_cb poweron)
{
	struct clk *apll1_slow, *sspa1_mn_div, *sspa1_clk_src, *sspa1_div_clk;
	struct device_node *np = of_find_node_by_name(NULL, "sound");
	struct clk *clk;

	if (!np)
		return;

	if (soc_is_ax())
		audio_clk_init(apmu_base, audio_base, audio_aux_base, poweron);
	else {
		apll1_slow = mmp_clk_register_audio("apll1_slow", "vctcxo",
			apmu_base, audio_base, audio_aux_base,
			APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
			APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP, &clk_lock);
		clk_register_clkdev(apll1_slow, "mmp-audio", NULL);

		clk_prepare_enable(apll1_slow);
		clk_set_rate(apll1_slow, 22579200);

		clk = clk_register_mux(NULL, "sspa1_mux1", sspa1_parent1,
				ARRAY_SIZE(sspa1_parent1), 0,
				audio_aux_base + 0x84, 3, 3, 0, &clk_lock);
		clk_set_parent(clk, apll1_slow);
		clk_register_clkdev(clk, "sspa1_mux.1", NULL);

		sspa1_mn_div = clk_register_gate(NULL, "sspa1_mn_div",
					"sspa1_mux1", 0,
					audio_aux_base + 0x94, 30, 0,
					&clk_lock);
		clk_register_clkdev(clk, "mn_bypass.1", NULL);
		clk_prepare_enable(sspa1_mn_div);

		sspa1_clk_src = clk_register_mux(NULL, "sspa1_clk_src",
				sspa1_parent2,
				ARRAY_SIZE(sspa1_parent2), 0,
				audio_aux_base + 0x84, 9, 1, 0,
				&clk_lock);
		clk_set_parent(sspa1_clk_src, sspa1_mn_div);
		clk_register_clkdev(clk, "sspa1_mux.2", NULL);

		sspa1_div_clk = clk_register_mux(NULL, "sspa1_div_clk",
				sspa1_parent3,
				ARRAY_SIZE(sspa1_parent3), 0,
				audio_aux_base + 0x84, 11, 1, 0,
				&clk_lock);
		clk_set_parent(sspa1_div_clk, sspa1_clk_src);
		clk_register_clkdev(clk, "sspa1_mux.3", NULL);

		clk = clk_register_mux(NULL, "sysclk_mux", sysclk_parent,
				ARRAY_SIZE(sysclk_parent), 0,
				audio_aux_base + 0x84, 13, 1, 0, &clk_lock);
		clk_set_parent(clk, sspa1_div_clk);
		clk_register_clkdev(clk, "sysclk_mux.0", NULL);

		clk = clk_register_divider(NULL, "sspa1_div",
				"sspa1_div_clk", 0,
				audio_base + 0x34, 9, 6,
				CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
				&clk_lock);
		clk_set_rate(clk, 2822400);
		clk_register_clkdev(clk, "sspa1_div", NULL);

		clk = clk_register_divider(NULL, "sysclk_div",
				"sspa1_div_clk", 0,
				audio_base + 0x34, 1, 6,
				CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
				&clk_lock);
		clk_set_rate(clk, 11289600);
		clk_register_clkdev(clk, "sysclk_div", NULL);

		clk = clk_register_gate(NULL, "mmp-sspa-dai.0", "sspa1_div", 0,
					audio_base + 0x34, 8, 0, &clk_lock);

		clk_register_clkdev(clk, NULL, "mmp-sspa-dai.0");

		clk = clk_register_gate(NULL, "sysclk.0", "sysclk_div", 0,
					audio_base + 0x34, 8, 0, &clk_lock);

		clk_register_clkdev(clk, "mmp-sysclk", NULL);
	}
}
#endif


#define GC2D_CLK_EN			(1u << 15)
#define GC2D_ACLK_EN			(1u << 19)

#define GC3D_CLK_EN			(1u << 3)
#define GC3D_ACLK_EN			(1u << 2)

static struct clk_mux_sel gc3d_mux_pll[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll5", .value = 2},
	{.parent_name = "pll2p", .value = 3},
};

static struct clk_mux_sel gc3d_mux_pll_ax[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll5", .value = 1},
	{.parent_name = "pll2", .value = 2},
	{.parent_name = "pll2p", .value = 6},
};

static struct clk_mux_sel gc2d_mux_pll[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll5", .value = 2},
	{.parent_name = "pll5p", .value = 3},
};

static struct clk_mux_sel gc2d_mux_pll_ax[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll5", .value = 2},
	{.parent_name = "pll5p", .value = 6},
};

static struct clk_mux_sel gc_aclk_mux_pll[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll5p", .value = 2},
	{.parent_name = "pll1_416", .value = 3},
};

static struct clk_mux_sel gc3d_aclk_mux_pll_ax[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll1_416", .value = 2},
	{.parent_name = "pll5p", .value = 6},
};

static struct clk_mux_sel gc2d_aclk_mux_pll_ax[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll2", .value = 1},
	{.parent_name = "pll1_416", .value = 2},
	{.parent_name = "pll5", .value = 6},
};

static const char const *gc_aclk_parents[] = {"pll1_624", "pll2",
						"pll5p", "pll1_416",};
static const char const *gc3d_aclk_parents_ax[] = {"pll1_624", "pll2",
						"pll1_416", "pll5p",};
static const char const *gc2d_aclk_parents_ax[] = {"pll1_624", "pll2",
						"pll1_416", "pll5",};
static const char const *gc3d_clk_parents[] = {"pll1_624", "pll2",
						"pll5", "pll2p",};
static const char const *gc3d_clk_parents_ax[] = {"pll1_624", "pll5",
						"pll2", "pll2p",};
static const char const *gc2d_clk_parents[] = {"pll1_624", "pll2",
						"pll5", "pll5p",};
static const char const *gc2d_clk_parents_ax[] = {"pll1_624", "pll2",
						"pll5", "pll5p",};

static struct periph_clk_tbl gc_aclk_tbl[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_624"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 416000000, .parent_name = "pll1_416"},
};

static struct peri_reg_info gc3d_aclk_reg = {
	.src_sel_shift = 4,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 23,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_GC,
	.enable_val = GC3D_ACLK_EN,
	.disable_val = GC3D_ACLK_EN,
};

static struct peri_reg_info gc3d_aclk_reg_ax = {
	.src_sel_shift = 4,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 0,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_reg_info gc2d_aclk_reg = {
	.src_sel_shift = 0,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 2,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_GC2,
	.enable_val = GC2D_ACLK_EN,
	.disable_val = GC2D_ACLK_EN,
};

static struct peri_reg_info gc2d_aclk_reg_ax = {
	.src_sel_shift = 4,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 0,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params gc_aclk_params = {
	.clktbl = gc_aclk_tbl,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl),
	.inputs = gc_aclk_mux_pll,
	.inputs_size = ARRAY_SIZE(gc_aclk_mux_pll),
};

static struct peri_params gc3d_aclk_params_ax = {
	.clktbl = gc_aclk_tbl,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl),
	.inputs = gc3d_aclk_mux_pll_ax,
	.inputs_size = ARRAY_SIZE(gc3d_aclk_mux_pll_ax),
};

static struct peri_params gc2d_aclk_params_ax = {
	.clktbl = gc_aclk_tbl,
	.clktblsize = ARRAY_SIZE(gc_aclk_tbl),
	.inputs = gc2d_aclk_mux_pll_ax,
	.inputs_size = ARRAY_SIZE(gc2d_aclk_mux_pll_ax),
};


static const char const *gc3d_clk1x_depend[] = {"GC3D_ACLK",};

static struct periph_clk_tbl gc3d_clk1x_tbl[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_624"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 528666666, .parent_name = "pll5"},
	{.clk_rate = 624000000, .parent_name = "pll1_624"},
};

static struct peri_reg_info gc3d_clk1x_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 26,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_GC,
	.enable_val = GC3D_CLK_EN,
	.disable_val = GC3D_CLK_EN,
};

static struct peri_reg_info gc3d_clk1x_reg_ax = {
	.src_sel_shift = 12,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 8,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params gc3d_clk1x_params = {
	.clktbl = gc3d_clk1x_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk1x_tbl),
	.inputs = gc3d_mux_pll,
	.inputs_size = ARRAY_SIZE(gc3d_mux_pll),
	.dcstat_support = true,
};

static struct peri_params gc3d_clk1x_params_ax = {
	.clktbl = gc3d_clk1x_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk1x_tbl),
	.inputs = gc3d_mux_pll_ax,
	.inputs_size = ARRAY_SIZE(gc3d_mux_pll_ax),
};

static const char const *gc3d_clk2x_depend[] = {"GC3D_CLK1X",};

static struct periph_clk_tbl gc3d_clk2x_tbl[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_624"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 528666666, .parent_name = "pll5"},
	{.clk_rate = 624000000, .parent_name = "pll1_624"},
};

static struct peri_reg_info gc3d_clk2x_reg = {
	.src_sel_shift = 12,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 29,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_GC,
	.enable_val = GC3D_CLK_EN,
	.disable_val = GC3D_CLK_EN,
};

static struct peri_reg_info gc3d_clk2x_reg_ax = {
	.src_sel_shift = 20,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 16,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params gc3d_clk2x_params = {
	.clktbl = gc3d_clk2x_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk2x_tbl),
	.inputs = gc3d_mux_pll,
	.inputs_size = ARRAY_SIZE(gc3d_mux_pll),
	.dcstat_support = true,
};

static struct peri_params gc3d_clk2x_params_ax = {
	.clktbl = gc3d_clk2x_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk2x_tbl),
	.inputs = gc3d_mux_pll_ax,
	.inputs_size = ARRAY_SIZE(gc3d_mux_pll_ax),
};

static const char const *gc2d_clk_depend[] = {"GC3D_ACLK", "GC2D_ACLK",};

static struct periph_clk_tbl gc2d_clk_tbl[] = {
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_624"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 528666666, .parent_name = "pll5"},
	{.clk_rate = 624000000, .parent_name = "pll1_624"},
};

static struct peri_reg_info gc2d_clk_reg = {
	.src_sel_shift = 16,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 20,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_GC2,
	.enable_val = GC2D_CLK_EN,
	.disable_val = GC2D_CLK_EN,
};

static struct peri_reg_info gc2d_clk_reg_ax = {
	.src_sel_shift = 12,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 8,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params gc2d_clk_params = {
	.clktbl = gc2d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc2d_clk_tbl),
	.inputs = gc2d_mux_pll,
	.inputs_size = ARRAY_SIZE(gc2d_mux_pll),
	.dcstat_support = true,
};

static struct peri_params gc2d_clk_params_ax = {
	.clktbl = gc2d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc2d_clk_tbl),
	.inputs = gc2d_mux_pll_ax,
	.inputs_size = ARRAY_SIZE(gc2d_mux_pll_ax),
};

struct plat_clk_list gc_clk_list[] = {
	{NULL, "GC3D_ACLK", 208000000},
	{NULL, "GC3D_CLK1X", 156000000},
	{NULL, "GC3D_CLK2X", 156000000},
	{NULL, "GC2D_ACLK", 208000000},
	{NULL, "GC2D_CLK", 156000000},
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

/* interface used by GC driver to get avaliable GC frequencies, unit HZ */
/* Get GC3D core frequency */
int get_gc3d_freqs_table(unsigned long *gcu3d_freqs_table,
	unsigned int *item_counts, unsigned int max_item_counts)
{
	unsigned int index;
	*item_counts = 0;

	if (!gcu3d_freqs_table) {
		pr_err("%s NULL ptr!\n", __func__);
		return -EINVAL;
	}

	if (max_item_counts < ARRAY_SIZE(gc3d_clk1x_tbl)) {
		pr_err("%s Too many GC frequencies %u!\n", __func__,
			max_item_counts);
		return -EINVAL;
	}

	for (index = 0; index < ARRAY_SIZE(gc3d_clk1x_tbl); index++)
		gcu3d_freqs_table[index] = gc3d_clk1x_tbl[index].clk_rate;
	*item_counts = index;
	return 0;
}
EXPORT_SYMBOL(get_gc3d_freqs_table);

/* Get GC3D shader frequency */
int get_gc3d_sh_freqs_table(unsigned long *gcu3d_sh_freqs_table,
	unsigned int *item_counts, unsigned int max_item_counts)
{
	unsigned int index;
	*item_counts = 0;

	if (!gcu3d_sh_freqs_table) {
		pr_err("%s NULL ptr!\n", __func__);
		return -EINVAL;
	}

	if (max_item_counts < ARRAY_SIZE(gc3d_clk2x_tbl)) {
		pr_err("%s Too many GC frequencies %u!\n", __func__,
			max_item_counts);
		return -EINVAL;
	}

	for (index = 0; index < ARRAY_SIZE(gc3d_clk2x_tbl); index++)
		gcu3d_sh_freqs_table[index] = gc3d_clk2x_tbl[index].clk_rate;
	*item_counts = index;
	return 0;
}
EXPORT_SYMBOL(get_gc3d_sh_freqs_table);

/* Get GC2D frequency */
int get_gc2d_freqs_table(unsigned long *gcu2d_freqs_table,
	unsigned int *item_counts, unsigned int max_item_counts)
{
	unsigned int index;
	*item_counts = 0;

	if (!gcu2d_freqs_table) {
		pr_err("%s NULL ptr!\n", __func__);
		return -EINVAL;
	}

	if (max_item_counts < ARRAY_SIZE(gc2d_clk_tbl)) {
		pr_err("%s Too many GC frequencies %u!\n", __func__,
			max_item_counts);
		return -EINVAL;
	}

	for (index = 0; index < ARRAY_SIZE(gc2d_clk_tbl); index++)
		gcu2d_freqs_table[index] = gc2d_clk_tbl[index].clk_rate;
	*item_counts = index;
	return 0;
}
EXPORT_SYMBOL(get_gc2d_freqs_table);

#ifdef CONFIG_UIO_HANTRO
#define VPU_DEC_CLK_EN			(1 << 27)
#define VPU_AXI_CLK_EN			(1 << 3)
#define VPU_ENC_CLK_EN			(1 << 4)

static struct periph_clk_tbl vpu_aclk_tbl[] = {
	{.clk_rate = 104000000, .parent_name = "pll1_624"},
	{.clk_rate = 156000000, .parent_name = "pll1_624"},
	{.clk_rate = 208000000, .parent_name = "pll1_624"},
	{.clk_rate = 264333333, .parent_name = "pll5"},
	{.clk_rate = 312000000, .parent_name = "pll1_624"},
	{.clk_rate = 416000000, .parent_name = "pll1_416"},
};

static const char const *vpu_parents[] = {"pll1_624", "pll5p",
					"pll5", "pll1_416",};

static const char const *vpu_parents_ax[] = {"pll1_624", "pll1_416",
						"pll5", "pll7"};

static const char const *vpu_depend_clk[] = {"VPU_AXI_CLK",};
static struct clk_mux_sel vpu_mux_sel[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll5p", .value = 1},
	{.parent_name = "pll5", .value = 2},
	{.parent_name = "pll1_416", .value = 3},
};

static struct clk_mux_sel vpu_mux_sel_ax[] = {
	{.parent_name = "pll1_624", .value = 0},
	{.parent_name = "pll1_416", .value = 1},
	{.parent_name = "pll5", .value = 2},
	{.parent_name = "pll7", .value = 6},
};

static struct peri_reg_info vpu_aclk_reg = {
	.src_sel_shift = 12,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 19,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_VPU,
	.enable_val = VPU_AXI_CLK_EN,
	.disable_val = VPU_AXI_CLK_EN,
};

static struct peri_reg_info vpu_aclk_reg_ax = {
	.src_sel_shift = 4,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 0,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params vpu_aclk_params = {
	.clktbl = vpu_aclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_aclk_tbl),
	.inputs = vpu_mux_sel,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel),
};

static struct peri_params vpu_aclk_params_ax = {
	.clktbl = vpu_aclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_aclk_tbl),
	.inputs = vpu_mux_sel_ax,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel_ax),
};

static struct periph_clk_tbl vpu_dclk_tbl[] = {
	{
		.clk_rate = 104000000,
		.parent_name = "pll1_624",
		.comclk_rate = 104000000,
	},
	{
		.clk_rate = 132166666,
		.parent_name = "pll5",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 208000000,
		.parent_name = "pll1_624",
		.comclk_rate = 208000000
	},
	{
		.clk_rate = 264333333,
		.parent_name = "pll5",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416",
		.comclk_rate = 416000000,
	},
};

static struct peri_reg_info vpu_dclk_reg = {
	.src_sel_shift = 22,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 24,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_VPU,
	.enable_val = VPU_DEC_CLK_EN,
	.disable_val = VPU_DEC_CLK_EN,
};

static struct peri_reg_info vpu_dclk_reg_ax = {
	.src_sel_shift = 12,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 8,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params vpu_dclk_params = {
	.clktbl = vpu_dclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_dclk_tbl),
	.comclk_name = "VPU_AXI_CLK",
	.inputs = vpu_mux_sel,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel),
	.dcstat_support = true,
};

static struct peri_params vpu_dclk_params_ax = {
	.clktbl = vpu_dclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_dclk_tbl),
	.comclk_name = "VPU_AXI_CLK",
	.inputs = vpu_mux_sel_ax,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel_ax),
};

static struct periph_clk_tbl vpu_eclk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = "pll1_624",
		.comclk_rate = 156000000,
	},
	{
		.clk_rate = 208000000,
		.parent_name = "pll1_624",
		.comclk_rate = 208000000
	},
	{
		.clk_rate = 264333333,
		.parent_name = "pll5",
		.comclk_rate = 264333333,
	},
	{
		.clk_rate = 312000000,
		.parent_name = "pll1_624",
		.comclk_rate = 312000000,
	},
	{
		.clk_rate = 416000000,
		.parent_name = "pll1_416",
		.comclk_rate = 416000000,
	},
};

static struct peri_reg_info vpu_eclk_reg = {
	.src_sel_shift = 6,
	.src_sel_mask = MASK(REG_WIDTH_2BIT),
	.div_shift = 16,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
	.reg_offset = APMU_VPU,
	.enable_val = VPU_ENC_CLK_EN,
	.disable_val = VPU_ENC_CLK_EN,
};

static struct peri_reg_info vpu_eclk_reg_ax = {
	.src_sel_shift = 20,
	.src_sel_mask = MASK(REG_WIDTH_3BIT),
	.div_shift = 16,
	.div_mask = MASK(REG_WIDTH_3BIT),
	.fcreq_shift = 9,
	.fcreq_mask = MASK(REG_WIDTH_1BIT),
	.flags = CLK_DIVIDER_ONE_BASED,
};

static struct peri_params vpu_eclk_params = {
	.clktbl = vpu_eclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_eclk_tbl),
	.comclk_name = "VPU_AXI_CLK",
	.inputs = vpu_mux_sel,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel),
	.dcstat_support = true,
};

static struct peri_params vpu_eclk_params_ax = {
	.clktbl = vpu_eclk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_eclk_tbl),
	.comclk_name = "VPU_AXI_CLK",
	.inputs = vpu_mux_sel_ax,
	.inputs_size = ARRAY_SIZE(vpu_mux_sel_ax),
};

struct plat_clk_list vpu_clk_list[] = {
	{NULL, "VPU_AXI_CLK", 208000000},
	{NULL, "VPU_DEC_CLK", 208000000},
	{NULL, "VPU_ENC_CLK", 312000000},
};

#define APMU_VPU_CLKCTRL	0x1f4
#define APMU_VPU_RSTCTRL	0x1f0
static void __init eden_vpu_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;

	if (soc_is_ax()) {
		clk = eden_clk_register_gc_vpu("VPU_AXI_CLK", vpu_parents_ax,
				ARRAY_SIZE(vpu_parents_ax), 0,
				apmu_base + APMU_VPU_CLKCTRL,
				apmu_base + APMU_VPU_RSTCTRL, &vpu_lock,
				&vpu_aclk_params_ax, &vpu_aclk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "VPU_AXI_CLK", NULL);
		clks[vpu_aclk] = clk;

		clk = eden_clk_register_gc_vpu("VPU_DEC_CLK", vpu_parents_ax,
				ARRAY_SIZE(vpu_parents_ax), 0,
				apmu_base + APMU_VPU_CLKCTRL,
				apmu_base + APMU_VPU_RSTCTRL, &vpu_lock,
				&vpu_dclk_params_ax, &vpu_dclk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "VPU_DEC_CLK", NULL);
		clks[vpu_dec] = clk;

		clk = eden_clk_register_gc_vpu("VPU_ENC_CLK", vpu_parents_ax,
				ARRAY_SIZE(vpu_parents_ax), 0,
				apmu_base + APMU_VPU_CLKCTRL,
				apmu_base + APMU_VPU_RSTCTRL, &vpu_lock,
				&vpu_eclk_params_ax, &vpu_eclk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "VPU_ENC_CLK", NULL);
		clks[vpu_enc] = clk;
	} else {
		clk = eden_clk_register_gc_vpu("VPU_AXI_CLK", vpu_parents,
				ARRAY_SIZE(vpu_parents), 0,
				apmu_base + APMU_VPU, 0, &vpu_lock,
				&vpu_aclk_params, &vpu_aclk_reg, NULL, 0);
		clk_register_clkdev(clk, "VPU_AXI_CLK", NULL);
		clks[vpu_aclk] = clk;

		clk = eden_clk_register_gc_vpu("VPU_DEC_CLK", vpu_parents,
				ARRAY_SIZE(vpu_parents), 0,
				apmu_base + APMU_VPU, 0, &vpu_lock,
				&vpu_dclk_params, &vpu_dclk_reg,
				vpu_depend_clk, ARRAY_SIZE(vpu_depend_clk));
		clk_register_clkdev(clk, "VPU_DEC_CLK", NULL);
		clks[vpu_dec] = clk;

		clk = eden_clk_register_gc_vpu("VPU_ENC_CLK", vpu_parents,
				ARRAY_SIZE(vpu_parents), 0,
				apmu_base + APMU_VPU, 0, &vpu_lock,
				&vpu_eclk_params, &vpu_eclk_reg,
				vpu_depend_clk, ARRAY_SIZE(vpu_depend_clk));
		clk_register_clkdev(clk, "VPU_ENC_CLK", NULL);
		clks[vpu_enc] = clk;
	}

	peri_init_set_rate(vpu_clk_list, ARRAY_SIZE(vpu_clk_list));
}

#define VPU_DEVFREQ_DEC	0
#define VPU_DEVFREQ_ENC	1

unsigned int eden_get_vpu_op_num(unsigned int vpu_type)
{
	switch (vpu_type) {
	case VPU_DEVFREQ_DEC:
		return ARRAY_SIZE(vpu_dclk_tbl);
	case VPU_DEVFREQ_ENC:
		return ARRAY_SIZE(vpu_eclk_tbl);
	default:
		pr_err("%s, unable to find the clock table!\n",
				__func__);
		return 0;
	}
}

unsigned int eden_get_vpu_op_rate(unsigned int vpu_type, unsigned int index)
{
	struct periph_clk_tbl *clk_tbl;
	unsigned int tbl_size = 0;

	switch (vpu_type) {
	case VPU_DEVFREQ_DEC:
		clk_tbl = vpu_dclk_tbl;
		tbl_size = ARRAY_SIZE(vpu_dclk_tbl);
		break;
	case VPU_DEVFREQ_ENC:
		clk_tbl = vpu_eclk_tbl;
		tbl_size = ARRAY_SIZE(vpu_eclk_tbl);
		break;
	default:
		pr_err("%s, unable to find the clock table!\n",
				__func__);
		return 0;
	}

	WARN_ON(index >= tbl_size);

	return clk_tbl[index].clk_rate / MHZ_TO_KHZ;
}
#endif

#define APMU_GC3D_RSTCTRL	0x170
#define APMU_GC3D_CLKCTRL	0x174
#define APMU_GC2D_RSTCTRL	0x178
#define APMU_GC2D_CLKCTRL	0x17c
#define APMU_ACLK11_CLKCTRL	0x120
static void __init eden_gc_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;

	if (soc_is_ax()) {
		clk = eden_clk_register_gc_vpu("GC3D_ACLK",
				gc3d_aclk_parents_ax,
				ARRAY_SIZE(gc3d_aclk_parents_ax), 0,
				apmu_base + APMU_GC3D_CLKCTRL,
				apmu_base + APMU_GC3D_RSTCTRL,
				&gc3d_lock, &gc3d_aclk_params_ax,
				&gc3d_aclk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "GC3D_ACLK", NULL);
		clks[gc3d_aclk] = clk;

		clk = eden_clk_register_gc_vpu("GC3D_CLK1X",
				gc3d_clk_parents_ax,
				ARRAY_SIZE(gc3d_clk_parents_ax), 0,
				apmu_base + APMU_GC3D_CLKCTRL,
				apmu_base + APMU_GC3D_RSTCTRL,
				&gc3d_lock, &gc3d_clk1x_params_ax,
				&gc3d_clk1x_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "GC3D_CLK1X", NULL);
		clks[gc3d_1x] = clk;

		clk = eden_clk_register_gc_vpu("GC3D_CLK2X",
				gc3d_clk_parents_ax,
				ARRAY_SIZE(gc3d_clk_parents_ax), 0,
				apmu_base + APMU_GC3D_CLKCTRL,
				apmu_base + APMU_GC3D_RSTCTRL,
				&gc3d_lock, &gc3d_clk2x_params_ax,
				&gc3d_clk2x_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "GC3D_CLK2X", NULL);
		clks[gc3d_2x] = clk;

		clk = eden_clk_register_gc_vpu("GC2D_ACLK",
				gc2d_aclk_parents_ax,
				ARRAY_SIZE(gc2d_aclk_parents_ax), 0,
				apmu_base + APMU_ACLK11_CLKCTRL,
				apmu_base + APMU_ACLK11_CLKCTRL,
				&gc2d_lock, &gc2d_aclk_params_ax,
				&gc2d_aclk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "GC2D_ACLK", NULL);
		clks[gc2d_aclk] = clk;

		clk = eden_clk_register_gc_vpu("GC2D_CLK", gc2d_clk_parents_ax,
				ARRAY_SIZE(gc2d_clk_parents_ax), 0,
				apmu_base + APMU_GC2D_CLKCTRL,
				apmu_base + APMU_GC2D_RSTCTRL,
				&gc2d_lock, &gc2d_clk_params_ax,
				&gc2d_clk_reg_ax, NULL, 0);
		clk_register_clkdev(clk, "GC2D_CLK", NULL);
		clks[gc2d] = clk;
	} else {
		clk = eden_clk_register_gc_vpu("GC3D_ACLK", gc_aclk_parents,
				ARRAY_SIZE(gc_aclk_parents), 0,
				apmu_base + APMU_GC, 0,
				&gc3d_lock, &gc_aclk_params,
				&gc3d_aclk_reg, NULL, 0);
		clk_register_clkdev(clk, "GC3D_ACLK", NULL);
		clks[gc3d_aclk] = clk;

		clk = eden_clk_register_gc_vpu("GC3D_CLK1X", gc3d_clk_parents,
				ARRAY_SIZE(gc3d_clk_parents), 0,
				apmu_base + APMU_GC, 0,
				&gc3d_lock, &gc3d_clk1x_params,
				&gc3d_clk1x_reg, gc3d_clk1x_depend,
				ARRAY_SIZE(gc3d_clk1x_depend));
		clk_register_clkdev(clk, "GC3D_CLK1X", NULL);
		clks[gc3d_1x] = clk;

		clk = eden_clk_register_gc_vpu("GC3D_CLK2X", gc3d_clk_parents,
				ARRAY_SIZE(gc3d_clk_parents), 0,
				apmu_base + APMU_GC, 0,
				&gc3d_lock, &gc3d_clk2x_params,
				&gc3d_clk2x_reg, gc3d_clk2x_depend,
				ARRAY_SIZE(gc3d_clk2x_depend));
		clk_register_clkdev(clk, "GC3D_CLK2X", NULL);
		clks[gc3d_2x] = clk;

		clk = eden_clk_register_gc_vpu("GC2D_ACLK", gc_aclk_parents,
				ARRAY_SIZE(gc_aclk_parents), 0,
				apmu_base + APMU_GC2, 0,
				&gc2d_lock, &gc_aclk_params,
				&gc2d_aclk_reg, NULL, 0);
		clk_register_clkdev(clk, "GC2D_ACLK", NULL);
		clks[gc2d_aclk] = clk;

		clk = eden_clk_register_gc_vpu("GC2D_CLK", gc2d_clk_parents,
				ARRAY_SIZE(gc2d_clk_parents), 0,
				apmu_base + APMU_GC2, 0,
				&gc2d_lock, &gc2d_clk_params, &gc2d_clk_reg,
				gc2d_clk_depend, ARRAY_SIZE(gc2d_clk_depend));
		clk_register_clkdev(clk, "GC2D_CLK", NULL);
		clks[gc2d] = clk;
	}

	peri_init_set_rate(gc_clk_list, ARRAY_SIZE(gc_clk_list));
}

#ifdef CONFIG_THERMAL
static void __init eden_thermal_clk_init(void __iomem *apbc_base)
{
	struct clk *clk;

	clk = mmp_clk_register_apbc("thermal_g", "NULL",
				apbc_base + APBC_THSENS, 10, 0, &clk_lock);
	clk_register_clkdev(clk, "THERMALCLK_G", NULL);
	clk = mmp_clk_register_apbc("thermal_vpu", "NULL",
				apbc_base + APBC_THSENS + 0x8,
				10, 0, &clk_lock);
	clk_register_clkdev(clk, "THERMALCLK_VPU", NULL);
	clk = mmp_clk_register_apbc("thermal_cpu", "NULL",
				apbc_base + APBC_THSENS + 0xc,
				10, 0, &clk_lock);
	clk_register_clkdev(clk, "THERMALCLK_CPU", NULL);
	clk = mmp_clk_register_apbc("thermal_gc", "NULL",
				apbc_base + APBC_THSENS + 0x10,
				10, 0, &clk_lock);
	clk_register_clkdev(clk, "THERMALCLK_GC", NULL);
}
#endif

#ifdef CONFIG_CORESIGHT_SUPPORT
#define CORESIGHT_ETB_MEMORY_CLK_EN	(1 << 25)
static void __init eden_coresight_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;

	/* ETB memory clock */
	clk = mmp_clk_register_apmu("ETBCLK", "NULL", apmu_base +
		APMU_AP_DEBUG1, CORESIGHT_ETB_MEMORY_CLK_EN, &clk_lock);
	clk_register_clkdev(clk, "ETBCLK", NULL);

	/*
	 * set dividers for debug interfaces,
	 * ATCLK_DIV = 3, ATB interface freq = PCLK / 3
	 * APBCLK_DIV = 4, AP Core APB debug interface freq = ATCLK / 4
	 */
	clk = mmp_clk_register_apmu("APSSCLK", "ETBCLK", apmu_base +
		APMU_COREAPSS, (4 << 20) | (3 << 16), &clk_lock);
	clk_register_clkdev(clk, "APSSCLK", NULL);

	/*
	 * ATCLK, coresight logic including that in AP core subsustem,
	 *  Fixed 624MHz PLL1 input, ATCLK = PLL1/3 = 208MHz,
	 * PCLKENDBG, coresight logic excluding that in AP Core subsystem,
	 *  PCLKENDBG_DIV = ATCLK/3 = 69.33MHz
	 */
	clk = mmp_clk_register_apmu("DBGCLK", "APSSCLK", apmu_base +
		APMU_APDBG, (3 << 8) | (1 << 4) | 3, &clk_lock);
	clk_register_clkdev(clk, "DBGCLK", NULL);

	if (of_find_node_by_name(NULL, "coresight")) {
		/* enable coresight to get ETB trace for system hang,
		 * this feature maybe disabled on customer boards */
		clk_prepare_enable(clk);
	}
}
#endif

#ifdef CONFIG_SMC91X
static void __init eden_smc91x_clk_init(void __iomem *apmu_base)
{
	struct device_node *np = of_find_node_by_name(NULL, "smc91x");
	const char *str = NULL;
	if (np && !of_property_read_string(np, "clksrc", &str))
		if (!strcmp(str, "smc91x")) {
			/* Enable clock to SMC Controller */
			writel(0x5b, apmu_base + APMU_SMC);
			/* Configure SMC Controller */
			/* Set CS0 to A\D type memory */
			writel(0x52880008, apmu_base + 0x1090);
		}
}
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
static void __init eden_sdh_clk_init(void __iomem *apmu_base)
{
	struct clk *clk;

	/* Select pll1_416 as sdh clock source by default */
	clk = clk_register_mux(NULL, "sdh_mux", sdh_parent,
			ARRAY_SIZE(sdh_parent), CLK_SET_RATE_PARENT,
			apmu_base + APMU_SDH0, 8, 2, 0, &clk_lock);
	clk_set_parent(clk, clk_get(NULL, "pll1_416"));
	clk_register_clkdev(clk, "sdh_mux", NULL);

	/* Set sdh base clock as 104Mhz */
	clk = clk_register_divider(NULL, "sdh_div", "sdh_mux", 0,
			apmu_base + APMU_SDH0, 10, 4,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO, &clk_lock);
	clk_set_rate(clk, 104000000);
	clk_register_clkdev(clk, "sdh_div", NULL);

	/* Register clock device for sdh1/2/3 which share the same base clock */
	clk = mmp_clk_register_apmu("sdh1", "sdh_div",
			apmu_base + APMU_SDH0, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.0");
	clk = mmp_clk_register_apmu("sdh2", "sdh_div",
			apmu_base + APMU_SDH1, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.1");
	clk = mmp_clk_register_apmu("sdh3", "sdh_div",
			apmu_base + APMU_SDH2, 0x1b, &clk_lock);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.2");
}
#endif

#ifdef CONFIG_SOUND
#define CLK_RES_CTRL		(0x010c)
#define ISLAND_SRAM_PWR_DWN_CTRL  (0x0240)
#define DSA_CLK_RES_CTRL  (0x0164)
#define ISLD_AU_CTRL  (0x01A8)
/*
 * check with clk guys, no need to add refercence cnt even both
 * audio pll call this power on/off func.
 */
void eden_audio_subsystem_poweron(void __iomem *apmu_base, int pwr_on)
{
	unsigned int reg_audio_rstctrl, reg_dsa_rstctrl;
	unsigned int reg_aud_isld_ctrl, reg_aud_isld_sram_pwdn;
	reg_audio_rstctrl = readl_relaxed(apmu_base + CLK_RES_CTRL);
#ifdef CONFIG_TZ_HYPERVISOR
	void *addr;
#endif
	if (pwr_on) {
		/* Audio island slow ramp up 1 */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(3<<9);
		reg_audio_rstctrl |= (1<<9);
		writel_relaxed(reg_audio_rstctrl,
			apmu_base + CLK_RES_CTRL);
		udelay(10);

		/* Audio island power up */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (3<<9);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/* audio sram slow ramp up */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<0);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/* audio sram power up DTCM */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<0);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/* audio core sram slow ramp up */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<2);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/* audio core sram power up ITCM */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<2);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/* ???need to find what this if for DTCM */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<12);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/* ???need to find what this if for DTCM */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<12);
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);

		/* Audio island isolation disabled */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<8);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/* Audio island memory redundancy start */
		writel_relaxed((readl_relaxed(apmu_base + CLK_RES_CTRL) |
			(1<<2)), apmu_base + CLK_RES_CTRL);
		/* Check for repair to finish */
		while (readl_relaxed(apmu_base + CLK_RES_CTRL) & (1<<2))
			;
		udelay(50);

		/* audio AXI and APB out of reset */
		reg_dsa_rstctrl = readl_relaxed(apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl |= (1<<0)|(1<<2);
		writel_relaxed(reg_dsa_rstctrl, apmu_base +
			DSA_CLK_RES_CTRL);
		udelay(10);

		/* Enable dummy clocks to Audio SRAM */
		reg_aud_isld_ctrl = readl_relaxed(apmu_base +
			ISLD_AU_CTRL);
		reg_aud_isld_ctrl |= (1<<4);
		writel_relaxed(reg_aud_isld_ctrl, apmu_base +
			ISLD_AU_CTRL);

		reg_aud_isld_ctrl = readl_relaxed(apmu_base +
			ISLD_AU_CTRL);
		reg_aud_isld_ctrl &= ~(1<<4);
		writel_relaxed(reg_aud_isld_ctrl, apmu_base +
			ISLD_AU_CTRL);

		/* audio AXI and APB clock enable */
		reg_dsa_rstctrl = readl_relaxed(apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl |= (1<<1)|(1<<3)|(1<<4)|(1<<5);
		writel_relaxed(reg_dsa_rstctrl, apmu_base +
			DSA_CLK_RES_CTRL);

		/* Audio bus peripheral clock enable */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<4);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);

		/* Audio bus peripheral out of reset */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<1);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

#ifdef CONFIG_TZ_HYPERVISOR
		addr = ioremap_nocache(0xc0ffd000, 4);
		writel(0x60, addr);
		iounmap(addr);
#endif
	} else {
		/* Check if audio already power off, if yes then return */
		if ((reg_audio_rstctrl & ((0x3<<9)|(0x1<<8))) == 0x0) {
			/* Audio is powered off, return */
			return;
		}

		/* ISB off */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(1<<8);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/* Audio bus peripheral in reset */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(0x1<<1);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/* Audio bus peripheral clock disable */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(0x1<<4);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);

		/* audio AXI and APB and external clock disable */
		reg_dsa_rstctrl = readl_relaxed(apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl &= ~((1<<1)|(1<<3)|(1<<4)|(1<<5));
		writel_relaxed(reg_dsa_rstctrl, apmu_base +
			DSA_CLK_RES_CTRL);

		/* audio AXI and APB out of reset */
		reg_dsa_rstctrl = readl_relaxed(apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl &= ~((1<<0)|(1<<2));
		writel_relaxed(reg_dsa_rstctrl, apmu_base +
			DSA_CLK_RES_CTRL);
		udelay(10);

		/* disable Audio island power switch */
		reg_audio_rstctrl = readl_relaxed(apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(3<<9);
		writel_relaxed(reg_audio_rstctrl, apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/* disable power to audio SRAM */
		reg_aud_isld_sram_pwdn = readl_relaxed(apmu_base +
				ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn &= ~((0x3<<0)|(3<<2)|(0x3<<12));
		writel_relaxed(reg_aud_isld_sram_pwdn,
			apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
	}
}
#endif

void __init eden_clk_init(unsigned long apb_phy_base,
		unsigned long axi_phy_base, unsigned long aud_phy_base,
		unsigned long aud_phy_base2)
{
	struct clk *clk;
	struct clk *vctcxo;
	struct clk *uart_pll;
	void __iomem *mpmu_base;
	void __iomem *apmu_base;
	void __iomem *apbc_base;
	void __iomem *audio_base;
	void __iomem *audio_aux_base;

	mpmu_base = ioremap(apb_phy_base + 0x50000, SZ_8K);
	if (mpmu_base == NULL) {
		pr_err("error to ioremap MPMU base\n");
		return;
	}

	apmu_base = ioremap(axi_phy_base + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}

	apbc_base = ioremap(apb_phy_base + 0x15000, SZ_4K);
	if (apbc_base == NULL) {
		pr_err("error to ioremap APBC base\n");
		return;
	}

	if (soc_is_ax()) {
		audio_base = ioremap(aud_phy_base, SZ_2K);
		if (audio_base == NULL) {
			pr_err("error to ioremap audio base\n");
			return;
		}

		audio_aux_base = ioremap(aud_phy_base2, SZ_2K);
		if (audio_aux_base == NULL) {
			pr_err("error to ioremap audio aux base\n");
			return;
		}
	} else {
		audio_base = ioremap(aud_phy_base + 0x400, SZ_2K);
		if (audio_base == NULL) {
			pr_err("error to ioremap audio base\n");
			return;
		}

		audio_aux_base = ioremap(aud_phy_base2, SZ_256);
		if (audio_aux_base == NULL) {
			pr_err("error to ioremap audio aux base\n");
			return;
		}
	}

	clk = clk_register_fixed_rate(NULL, "clk32", NULL, CLK_IS_ROOT, 3200);
	clk_register_clkdev(clk, "clk32", NULL);

	vctcxo = clk_register_fixed_rate(NULL, "vctcxo", NULL, CLK_IS_ROOT,
				26000000);
	clk_register_clkdev(vctcxo, "vctcxo", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_416", NULL,
				CLK_IS_ROOT, get_pll1_freq("pll1_416_freq"));
	clk_register_clkdev(clk, "pll1_416", NULL);

	clk = clk_register_fixed_rate(NULL, "pll1_624", NULL,
				CLK_IS_ROOT, get_pll1_freq("pll1_624_freq"));
	clk_register_clkdev(clk, "pll1_624", NULL);

	clk = clk_register_fixed_rate(NULL, "usb_pll", NULL, CLK_IS_ROOT,
				480000000);
	clk_register_clkdev(clk, "usb_pll", NULL);

	eden_pll_init(mpmu_base, apmu_base);

	clk = clk_register_fixed_factor(NULL, "vctcxo_d2", "vctcxo", 0, 1, 2);
	clk_register_clkdev(clk, "vctcxo_d2", NULL);

	clk = clk_register_fixed_factor(NULL, "vctcxo_d4", "vctcxo", 0, 1, 4);
	clk_register_clkdev(clk, "vctcxo_d4", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_d9", "pll1_624", 0, 1, 9);
	clk_register_clkdev(clk, "pll1_d9", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_d12", "pll1_624", 0, 1, 12);
	clk_register_clkdev(clk, "pll1_d12", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_d16", "pll1_624", 0, 1, 16);
	clk_register_clkdev(clk, "pll1_d16", NULL);

	clk = clk_register_fixed_factor(NULL, "pll1_d20", "pll1_624", 0, 1, 20);
	clk_register_clkdev(clk, "pll1_d20", NULL);

	uart_pll = mmp_clk_register_factor("uart_pll", "pll1_416", 0,
				mpmu_base + MPMU_UART_PLL,
				&uart_factor_masks, uart_factor_tbl,
				ARRAY_SIZE(uart_factor_tbl));
	clk_set_rate(uart_pll, 58593749);
	clk_register_clkdev(uart_pll, "uart_pll", NULL);

	clk = mmp_clk_register_apbc("twsi0", "vctcxo",
				apbc_base + APBC_TWSI0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.0");

	clk = mmp_clk_register_apbc("twsi1", "vctcxo",
				apbc_base + APBC_TWSI1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.1");

	clk = mmp_clk_register_apbc("twsi2", "vctcxo",
				apbc_base + APBC_TWSI2, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.2");

	clk = mmp_clk_register_apbc("twsi3", "vctcxo",
				apbc_base + APBC_TWSI3, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.3");

	clk = mmp_clk_register_apbc("twsi4", "vctcxo",
				apbc_base + APBC_TWSI4, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.4");

	clk = mmp_clk_register_apbc("twsi5", "vctcxo",
				apbc_base + APBC_TWSI5, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-i2c.5");

	clk = mmp_clk_register_apbc("gpio", "vctcxo",
				apbc_base + APBC_GPIO, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "eden-gpio");

	clk = mmp_clk_register_apbc("kpc", "clk32",
				apbc_base + APBC_KPC, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa27x-keypad");

	clk = mmp_clk_register_apbc("rtc", "clk32",
				apbc_base + APBC_RTC, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "sa1100-rtc");

	clk = mmp_clk_register_apbc("pwm0", "vctcxo",
				apbc_base + APBC_PWM0, 10, APBC_PWM, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-pwm.0");

	clk = mmp_clk_register_apbc("pwm1", "vctcxo",
				apbc_base + APBC_PWM1, 10, APBC_PWM, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-pwm.1");

	clk = mmp_clk_register_apbc("pwm2", "vctcxo",
				apbc_base + APBC_PWM2, 10, APBC_PWM, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-pwm.2");

	clk = mmp_clk_register_apbc("pwm3", "vctcxo",
				apbc_base + APBC_PWM3, 10, APBC_PWM, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-pwm.3");

	clk = clk_register_mux(NULL, "uart0_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART0, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, get_parent("uart0_clksrc", uart_pll));
	clk_register_clkdev(clk, "uart_mux.0", NULL);

	clk = mmp_clk_register_apbc("uart0", "uart0_mux",
				apbc_base + APBC_UART0, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.0");

	clk = clk_register_mux(NULL, "uart1_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART1, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, vctcxo);
	clk_register_clkdev(clk, "uart_mux.1", NULL);

	clk = mmp_clk_register_apbc("uart1", "uart1_mux",
				apbc_base + APBC_UART1, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.1");

	clk = clk_register_mux(NULL, "uart2_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART2, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, vctcxo);
	clk_register_clkdev(clk, "uart_mux.2", NULL);

	clk = mmp_clk_register_apbc("uart2", "uart2_mux",
				apbc_base + APBC_UART2, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.2");

	clk = clk_register_mux(NULL, "uart3_mux", uart_parent,
				ARRAY_SIZE(uart_parent), CLK_SET_RATE_PARENT,
				apbc_base + APBC_UART3, 4, 3, 0, &clk_lock);
	clk_set_parent(clk, vctcxo);
	clk_register_clkdev(clk, "uart_mux.3", NULL);

	clk = mmp_clk_register_apbc("uart3", "uart3_mux",
				apbc_base + APBC_UART3, 10, 0, &clk_lock);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.3");

	clk = mmp_clk_register_apmu("usb", "usb_pll", apmu_base + APMU_USB,
				0x9, &clk_lock);
	clk_register_clkdev(clk, "usb_clk", NULL);
	clk_register_clkdev(clk, NULL, "mv-otg");
	clk_register_clkdev(clk, NULL, "mv-udc");
	clk_register_clkdev(clk, NULL, "pxa-u2oehci");
	clk_register_clkdev(clk, NULL, "eden-usb-phy");

#ifdef CONFIG_CORESIGHT_SUPPORT
	eden_coresight_clk_init(apmu_base);
#endif

#ifdef CONFIG_THERMAL
	eden_thermal_clk_init(apbc_base);
#endif

#ifdef	CONFIG_UIO_HANTRO
	eden_vpu_clk_init(apmu_base);
#endif
	eden_gc_clk_init(apmu_base);
	eden_disp_clk_init(apmu_base);

	eden_ccic_clk_init(apmu_base);

#ifdef CONFIG_SOUND
	eden_audio_clk_init(apmu_base, audio_base, audio_aux_base,
				eden_audio_subsystem_poweron);
#endif

#ifdef CONFIG_SMC91X
	eden_smc91x_clk_init(apmu_base);
#endif

#ifdef CONFIG_MMC_SDHCI_PXAV3
	eden_sdh_clk_init(apmu_base);
#endif
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *stat;
CLK_DCSTAT_OPS(clks[cpu], cpu)
CLK_DCSTAT_OPS(clks[ddr], ddr)
CLK_DCSTAT_OPS(clks[axi], axi)
CLK_DCSTAT_OPS(clks[gc3d_1x], gc3d_1x);
CLK_DCSTAT_OPS(clks[gc3d_2x], gc3d_2x);
CLK_DCSTAT_OPS(clks[gc2d], gc2d);
CLK_DCSTAT_OPS(clks[vpu_dec], vpu_dec);
CLK_DCSTAT_OPS(clks[vpu_enc], vpu_enc);

static int __init __init_dcstat_debugfs_node(void)
{
	struct dentry *cpu_dc_stat = NULL, *ddr_dc_stat = NULL;
	struct dentry *axi_dc_stat = NULL;
	struct dentry *gc2d_dc_stat = NULL;
	struct dentry *vpu_dec_dc_stat = NULL, *vpu_enc_dc_stat = NULL;
	struct dentry *gc3d_1x_dc_stat = NULL, *gc3d_2x_dc_stat = NULL;

	stat = debugfs_create_dir("stat", NULL);
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

	gc3d_1x_dc_stat = clk_dcstat_file_create("gc3d_1x_dc_stat",
		stat, &gc3d_1x_dc_ops);
	if (!gc3d_1x_dc_stat)
		goto err_gc3d_1x_dc_stat;

	gc3d_2x_dc_stat = clk_dcstat_file_create("gc3d_2x_dc_stat",
		stat, &gc3d_2x_dc_ops);
	if (!gc3d_2x_dc_stat)
		goto err_gc3d_2x_dc_stat;

	gc2d_dc_stat = clk_dcstat_file_create("gc2d_dc_stat",
		stat, &gc2d_dc_ops);
	if (!gc2d_dc_stat)
		goto err_gc2d_dc_stat;

	vpu_dec_dc_stat = clk_dcstat_file_create("vpu_dec_dc_stat",
		stat, &vpu_dec_dc_ops);
	if (!vpu_dec_dc_stat)
		goto err_vpu_dec_dc_stat;

	vpu_enc_dc_stat = clk_dcstat_file_create("vpu_enc_dc_stat",
		stat, &vpu_enc_dc_ops);
	if (!vpu_enc_dc_stat)
		goto err_vpu_enc_dc_stat;

	return 0;

err_vpu_enc_dc_stat:
	debugfs_remove(vpu_dec_dc_stat);
err_vpu_dec_dc_stat:
	debugfs_remove(gc2d_dc_stat);
err_gc2d_dc_stat:
	debugfs_remove(gc3d_2x_dc_stat);
err_gc3d_2x_dc_stat:
	debugfs_remove(gc3d_1x_dc_stat);
err_gc3d_1x_dc_stat:
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
