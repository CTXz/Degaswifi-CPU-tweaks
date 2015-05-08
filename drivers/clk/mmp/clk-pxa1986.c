/*
 * pxa1986 clock framework source file
 *
 * Copyright (C) 2013 Marvell
 * Qing Xu <qingx@marvell.com>
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
#include <linux/of_address.h>

#include <linux/clk-private.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>

#include <mach/regs-accu.h>
#include <mach/regs-mccu.h>
#include <mach/regs-apmu.h>
#include <trace/events/power.h>

enum reg_unit {
	ACCU_REGS,
	MCCU_REGS,
};

static void __iomem *clk_iomap_register(const char *reg_name)
{
	void __iomem *reg_virt_addr;
	struct device_node *node;

	BUG_ON(!reg_name);
	node = of_find_compatible_node(NULL, NULL, reg_name);
	BUG_ON(!node);
	reg_virt_addr = of_iomap(node, 0);
	BUG_ON(!reg_virt_addr);

	return reg_virt_addr;
}

static void __iomem *accu_base_va(void)
{
	static void __iomem *accu_virt_addr;
	if (unlikely(!accu_virt_addr))
		accu_virt_addr = clk_iomap_register("mrvl,mmp-ccu-accu");
	return accu_virt_addr;
}

static void __iomem *apmu_base_va(void)
{
	static void __iomem *apmu_virt_addr;
	if (unlikely(!apmu_virt_addr))
		apmu_virt_addr = clk_iomap_register("mrvl,mmp-pmu-apmu");
	return apmu_virt_addr;
}

static void __iomem *mccu_base_va(void)
{
	static void __iomem *mccu_virt_addr;
	if (unlikely(!mccu_virt_addr))
		mccu_virt_addr = clk_iomap_register("mrvl,mmp-ccu-mccu");
	return mccu_virt_addr;
}

static void __iomem *lcd_base_va(void)
{
	static void __iomem *lcd_virt_addr;
	if (unlikely(!lcd_virt_addr))
		lcd_virt_addr = clk_iomap_register("marvell,mmp-disp");
	return lcd_virt_addr;
}

#define ACCU_UARTI2C_RATIO_BIT		(1 << 20)
#define SET_ACCU_UARTI2C_SEL(x)		(((x) & 0x1) << 20)
#define GET_ACCU_UARTI2C_BITS(x)	((x & ACCU_UARTI2C_RATIO_BIT) >> 20)

#define ACCU_SDH1_CLK_EN	(1 << 8)
#define ACCU_SDH2_CLK_EN	(1 << 9)
#define ACCU_SDH3_CLK_EN	(1 << 10)
#define ACCU_SDH4_CLK_EN	(1 << 11)
#define ACCU_SDH_INT_CLK_EN		(1 << 12)
#define ACCU_SDH_MASTER_BUS_CLK_EN	(1 << 13)
#define ACCU_SDH_SLAVE_BUS_CLK_EN	(1 << 14)
#define ACCU_SDH_CARD_INS_CLK_EN	(1 << 15)

#define ACCU_SDH_BUS_CLK_EN		(1 << 8)
#define ACCU_SDH_BUS_CONFIG_CLK_EN	(1 << 9)

#define MCCU_I2C_FNC_CKEN	(1 << 8)
#define MCCU_I2C_BUS_CKEN	(1 << 9)

/* Display Unit Clock Enable Registers */
#define ACCU_HDMI_CEC_CLK_EN	(1 << 8)
#define ACCU_DIS_UNIT_BUS_CLK_EN	(1 << 9)
#define ACCU_DIS_UNIT_AHB_CLK_EN	(1 << 10)
#define ACCU_LCD_TX_ESC_CLK_EN	(1 << 12)
#define ACCU_LCD_PHY_CAL_CLK_EN	(1 << 13)
#define ACCU_DISP_BCK_CLK_EN	(1 << 14)

#define ACCU_APPS_TEMP_ANALOG_CLK_EN	(1 << 10)
#define ACCU_APPS_TEMP_CTL32_CLK_EN	(1 << 11)

#define ACCU_USB2_APB_EN	(1 << 14)
#define ACCU_USB2_OTG_EN	(1 << 11)

DEFINE_SPINLOCK(sdh_lock);
DEFINE_SPINLOCK(gc2d_lock);
DEFINE_SPINLOCK(gc3d_lock);
DEFINE_SPINLOCK(disp_lock);

#define CLK_SET_BITS(set, clear, addr)	{				\
	unsigned long tmp;						\
	tmp = readl_relaxed(addr);					\
	tmp &= ~(clear);						\
	tmp |= (set);							\
	writel_relaxed(tmp, addr);					\
}									\

struct mmp_clk_disp {
	struct clk_hw		hw;
	struct clk_mux		mux;
	struct clk_divider	divider;

	spinlock_t	*lock;
	void __iomem	*apmu_base;
	u32		reg_mux;
	u32		reg_div;
	u32		reg_div_shadow;
	u32		reg_mux_shadow;

	u32		reg_rst;
	u32		reg_rst_shadow;
	u32		reg_rst_mask;

	const char	**dependence;
	int		num_dependence;
	const struct clk_ops	*mux_ops;
	const struct clk_ops	*div_ops;
};

struct clk_mux_sel {
	char *parent_name;
	u32 value;
	struct clk *parent;
};

struct periph_clk_tbl {
	unsigned long clk_rate;	/* clk rate */
	char *parent_name;	/* clk parent name*/
	struct clk *parent;	/* clk parent */
	unsigned long src_val;	/* clk src field reg val */
	unsigned long div_val;	/* clk div field reg val */

	struct list_head node;
};

struct peri_params {
	/* periph clock tbl, list all possible freq here */
	struct periph_clk_tbl *clktbl;
	unsigned int clktblsize;
	struct clk_mux_sel *inputs;
	unsigned int inputs_size;
	char **dependence;
	struct clk **depend_clk;
	unsigned int dependence_count;
	u32 reg_unit;
	void __iomem *reg_base;
	u32 reg_offset;
	u32 enable_val;
};

struct clk_peri {
	struct clk_hw hw;
	struct peri_params *params;
	/* list used represent all supported freq */
	struct list_head clktbl_list;
	/* lock to protect same register access */
	spinlock_t *lock;
};

#define to_clk_peri(peri_hw) container_of(peri_hw, struct clk_peri, hw)

struct gc_anchor_clk_tbl {
	unsigned int pp;
	unsigned long axi5_rate;
	unsigned long gc3d_fn_rate;
	unsigned long gc3d_sh_rate;
	unsigned long gc2d_rate;
};

#define PP0 0
#define PP1 1
#define PP2 2
#define PP3 3
#define PP4 4
#define PP5 5

#define CLK_26M_NAME "vcxo_26M"
#define USB_PLL_NAME "usb_pll"
#define PLL1416_NAME "pll1_416"
#define PLL1500_NAME "pll1_500"
#define PLL1624_NAME "pll1_624"
#define PLL2800_NAME "pll2_800"
#define PLL3700_NAME "pll3_700"

#define CLKSRC_PLL1_416	0
#define CLKSRC_PLL1_500	1
#define CLKSRC_PLL1_624	2
#define CLKSRC_PLL2_800	3
#define CLKSRC_PLL3	4
#define CLKSRC_HDMI	5

/*pll_parents & clk_mux_pll array member arrange should be the same*/
static const char const *pll_parents[] = {
	PLL1416_NAME,
	PLL1500_NAME,
	PLL1624_NAME,
	PLL2800_NAME,
	PLL3700_NAME,
};

static struct clk_mux_sel clk_mux_pll[] = {
	{
		.parent_name = PLL1416_NAME,
		.value = CLKSRC_PLL1_416,
	},
	{
		.parent_name = PLL1500_NAME,
		.value = CLKSRC_PLL1_500,
	},
	{
		.parent_name = PLL1624_NAME,
		.value = CLKSRC_PLL1_624,
	},
	{
		.parent_name = PLL2800_NAME,
		.value = CLKSRC_PLL2_800,
	},
	{
		.parent_name = PLL3700_NAME,
		.value = CLKSRC_PLL3,
	},
	{NULL, 0, NULL},
};

static void __clk_fill_periph_tbl(struct clk_hw *hw,
	struct periph_clk_tbl *clk_tbl, unsigned int clk_tbl_size)
{
	unsigned int i = 0;
	struct clk_peri *peri = to_clk_peri(hw);

	for (i = 0; i < clk_tbl_size; i++) {
		clk_tbl[i].parent = clk_get_sys(NULL, clk_tbl[i].parent_name);
		list_add_tail(&clk_tbl[i].node, &peri->clktbl_list);
	}
}

static long __clk_set_mux_div(struct clk_hw *hw, struct clk *best_parent,
	unsigned int src, unsigned int div)
{
	unsigned int regval, regval_after;
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned int en_value = peri->params->enable_val;
	u32 reg_offset = peri->params->reg_offset;
	void __iomem *reg_base = peri->params->reg_base;
	BUG_ON(!div);

	/*
	 * For smooth mux clock src switch, must make sure both clocks on
	 * or smooth mux can not finish clock switch.
	 */
	clk_prepare_enable(best_parent);
	if (!__clk_is_enabled(hw->clk) && hw->clk->parent)
		clk_prepare_enable(hw->clk->parent);

	/* below 2 steps should implement in as short time as possiable*/
	/* step1: disable clk*/
	regval = readl_relaxed(reg_base + reg_offset);
	regval &= ~(en_value);

	regval_after = regval&(~(ACCU_RATIO_MASK | ACCU_SOURCE_MASK));
	regval_after |= src|div;
	if (hw->clk->enable_count)
		regval_after |= en_value;
	writel_relaxed(regval, reg_base + reg_offset);

	/* step2: enable clk with new freq*/
	writel_relaxed(regval_after, reg_base + reg_offset);

	/*
	 * disable its parent if clk change done, no matter clk is enabled
	 * or not
	 */
	if (hw->clk->parent)
		clk_disable_unprepare(hw->clk->parent);
	if (!__clk_is_enabled(hw->clk))
		clk_disable_unprepare(best_parent);
	__clk_reparent(hw->clk, best_parent);

	return 0;
}

static void __clk_get_mux_div(struct clk_hw *hw,
		unsigned int *mux, unsigned int *div)
{
	struct clk_peri *peri = to_clk_peri(hw);
	u32 reg_offset = peri->params->reg_offset;
	void __iomem *reg_base = peri->params->reg_base;
	unsigned int regval;

	regval = readl_relaxed(reg_base + reg_offset);
	*mux = GET_ACCU_SOURCE(regval);	/* clk source */
	*div = GET_ACCU_RATIO(regval);	/* clk div */
}

static struct clk *__clk_mux_to_parent(struct clk_hw *hw, unsigned int mux)
{
	unsigned int i;

	i = 0;
	while (clk_mux_pll[i].parent && (clk_mux_pll[i].value != mux))
		i++;

	return clk_mux_pll[i].parent;
}

#if 0
static unsigned int __clk_parent_to_mux(struct clk_hw *clk, struct clk *parent)
{
	unsigned int i;

	i = 0;
	while (clk_mux_pll[i].parent && (clk_mux_pll[i].parent != parent))
		i++;

	return clk_mux_pll[i].value;
}
#endif

static unsigned long __clk_periph_get_rate(struct clk_hw *hw)
{
	struct clk *cur_parent;
	unsigned int mux, div;

	__clk_get_mux_div(hw, &mux, &div);
	if (div == 0) {
		div = 1;
		pr_err("%s %s div value is invalide!\n", __func__, hw->clk->name);
	}
	cur_parent = __clk_mux_to_parent(hw, mux);

	return clk_get_rate(cur_parent) / div;
}

static struct periph_clk_tbl *__clk_periph_get_tbl(
		struct clk_peri *peri, unsigned long rate)
{
	struct periph_clk_tbl *cop;
	unsigned int idx = 0;

	if (unlikely(!peri || !peri->params->clktbl))
		return NULL;

	list_for_each_entry(cop, &peri->clktbl_list, node) {
		if ((cop->clk_rate >= rate) ||
				list_is_last(&cop->node, &peri->clktbl_list))
			break;
		idx++;
	}
	return cop;
}

#if 0
static struct clk *__clk_periph_get_parent(struct clk_hw *hw)
{
	struct clk *cur_parent;
	unsigned int mux, div;

	__clk_get_mux_div(hw, &mux, &div);

	cur_parent = __clk_mux_to_parent(hw, mux);
	BUG_ON(NULL == cur_parent);

	return cur_parent;
}
#endif

static void periph_clk_init(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);

	INIT_LIST_HEAD(&peri->clktbl_list);

	if (peri->params->clktbl)
		__clk_fill_periph_tbl(hw, peri->params->clktbl,
				peri->params->clktblsize);
}

static int periph_clk_prepare(struct clk_hw *hw)
{
	int i;
	struct clk *clk_depend;
	struct clk_peri *peri = to_clk_peri(hw);
	for (i = 0; i < peri->params->dependence_count; i++) {
		if (peri->params->dependence[i] == NULL)
			BUG_ON("periph_clk_prepare: failed to get depend clk name !!\n");
		clk_depend = clk_get_sys(NULL, peri->params->dependence[i]);
		if (clk_depend)
			clk_prepare_enable(clk_depend);
	}
	return 0;
}

static void periph_clk_unprepare(struct clk_hw *hw)
{
	int i;
	struct clk *clk_depend;
	struct clk_peri *peri = to_clk_peri(hw);
	for (i = (peri->params->dependence_count - 1); i >= 0; i--) {
		clk_depend = clk_get_sys(NULL, peri->params->dependence[i]);
		if (clk_depend)
			clk_disable_unprepare(clk_depend);
	}
}

static int periph_clk_enable(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long irqflags = 0;
	u32 reg_offset = peri->params->reg_offset;
	void __iomem *reg_base = peri->params->reg_base;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, irqflags);
	CLK_SET_BITS(peri->params->enable_val, 0, reg_base + reg_offset);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, irqflags);

	trace_clock_enable(hw->clk->name, 1, 0);
	return 0;
}

static void periph_clk_disable(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long irqflags = 0;
	u32 reg_offset = peri->params->reg_offset;
	void __iomem *reg_base = peri->params->reg_base;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, irqflags);
	CLK_SET_BITS(0, peri->params->enable_val, reg_base + reg_offset);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, irqflags);

	trace_clock_disable(hw->clk->name, 0, 0);
}

static long periph_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct clk_peri *peri = to_clk_peri(hw);
	struct periph_clk_tbl *cop;

	cop = __clk_periph_get_tbl(peri, rate);
	if (!cop)
		return rate;
	return cop->clk_rate;
}

static unsigned long periph_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	return __clk_periph_get_rate(hw);
}

static int periph_clk_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long old_rate;
	struct clk *new_fparent;
	struct periph_clk_tbl *periph_pp = NULL;
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long irqflags = 0;

	old_rate = hw->clk->rate;
	if (rate == old_rate)
		return 0;

	periph_pp = __clk_periph_get_tbl(peri, rate);
	if (likely(periph_pp)) {
		/* set peripheral clk rate */
		new_fparent = periph_pp->parent;
		if (peri->lock)
			spin_lock_irqsave(peri->lock, irqflags);

		__clk_set_mux_div(hw, new_fparent, periph_pp->src_val,
				 periph_pp->div_val);

		if (peri->lock)
			spin_unlock_irqrestore(peri->lock, irqflags);
	} else {
		pr_err("%s %s failed to find clk table!\n", __func__, hw->clk->name);
	}

	trace_clock_set_rate(hw->clk->name, rate, 0);
	return 0;
}

static u8 periph_clk_get_parent(struct clk_hw *hw)
{
	int i;
	unsigned int mux;
	struct clk_peri *peri = to_clk_peri(hw);
	u32 reg_offset = peri->params->reg_offset;
	void __iomem *reg_base = peri->params->reg_base;

	u32 regval = readl_relaxed(reg_base + reg_offset);

	mux = GET_ACCU_SOURCE(regval);

	for (i = 0; i < ARRAY_SIZE(clk_mux_pll); i++) {
		if (clk_mux_pll[i].value == mux)
			break;
	}
	return i;
}

struct clk_ops peri_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.get_parent	= periph_clk_get_parent,
};

void __init pxa1986_pll_init(void)
{
	int i;
	struct clk *clk;
	clk = clk_register_fixed_rate(NULL, PLL1416_NAME,
					NULL, CLK_IS_ROOT, 416000000);
	clk_register_clkdev(clk, PLL1416_NAME, NULL);

	clk = clk_register_fixed_rate(NULL, PLL1500_NAME,
					NULL, CLK_IS_ROOT, 500000000);
	clk_register_clkdev(clk, PLL1500_NAME, NULL);

	clk = clk_register_fixed_rate(NULL, PLL1624_NAME,
					NULL, CLK_IS_ROOT, 624000000);
	clk_register_clkdev(clk, PLL1624_NAME, NULL);

	clk = clk_register_fixed_rate(NULL, PLL2800_NAME,
					NULL, CLK_IS_ROOT, 800000000);
	clk_register_clkdev(clk, PLL2800_NAME, NULL);

	clk = clk_register_fixed_rate(NULL, PLL3700_NAME,
					NULL, CLK_IS_ROOT, 700000000);
	clk_register_clkdev(clk, PLL3700_NAME, NULL);

	for (i = 0; i < ARRAY_SIZE(clk_mux_pll); i++) {
		clk_mux_pll[i].parent = clk_get_sys(NULL,
						clk_mux_pll[i].parent_name);
	}

	clk = clk_register_fixed_rate(NULL, CLK_26M_NAME,
					NULL, CLK_IS_ROOT, 26000000);
	clk_register_clkdev(clk, CLK_26M_NAME, NULL);

	clk = clk_register_fixed_rate(NULL, USB_PLL_NAME,
					NULL, CLK_IS_ROOT, 48000000);
	clk_register_clkdev(clk, USB_PLL_NAME, NULL);
}

static struct gc_anchor_clk_tbl axi5_anchor_tbl[] = {
	{
		.pp = PP0,
		.axi5_rate = 156000000,
		.gc3d_fn_rate = 156000000,
		.gc3d_sh_rate = 156000000,
		.gc2d_rate = 104000000,
	},
	{
		.pp = PP1,
		.axi5_rate = 208000000,
		.gc3d_fn_rate = 208000000,
		.gc3d_sh_rate = 208000000,
		.gc2d_rate = 104000000,
	},
	{
		.pp = PP2,
		.axi5_rate = 312000000,
		.gc3d_fn_rate = 312000000,
		.gc3d_sh_rate = 312000000,
		.gc2d_rate = 156000000,
	},
	{
		.pp = PP3,
		.axi5_rate = 416000000,
		.gc3d_fn_rate = 416000000,
		.gc3d_sh_rate = 416000000,
		.gc2d_rate = 208000000,
	},
	{
		.pp = PP4,
		.axi5_rate = 500000000,
		.gc3d_fn_rate = 500000000,
		.gc3d_sh_rate = 500000000,
		.gc2d_rate = 312000000,
	},
	{
		.pp = PP5,
		.axi5_rate = 624000000,
		.gc3d_fn_rate = 624000000,
		.gc3d_sh_rate = 624000000,
		.gc2d_rate = 312000000,
	},
};

static struct periph_clk_tbl gc3d_clk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(4),
	},
	{
		.clk_rate = 208000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(3),
	},
	{
		.clk_rate = 312000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(2),
	},
	{
		.clk_rate = 416000000,
		.parent_name = PLL1416_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_416),
		.div_val = SET_ACCU_RATIO(1),
	},
	{
		.clk_rate = 500000000,
		.parent_name = PLL1500_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_500),
		.div_val = SET_ACCU_RATIO(1),
	},
	{
		.clk_rate = 624000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(1),
	},
	/* for shader clk, it is 580Mhz and 700Mhz, come from PLL3 */
	{
		.clk_rate = 700000000,
		.parent_name = PLL3700_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL3),
		.div_val = SET_ACCU_RATIO(1),
	},
};

static struct periph_clk_tbl gc2d_clk_tbl[] = {
	{
		.clk_rate = 104000000,
		.parent_name = PLL1416_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_416),
		.div_val = SET_ACCU_RATIO(4),
	},
	{
		.clk_rate = 156000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(4),
	},
	{
		.clk_rate = 208000000,
		.parent_name = PLL1416_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_416),
		.div_val = SET_ACCU_RATIO(2),
	},
	{
		.clk_rate = 312000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(2),
	},
};

static unsigned long axi5_clk_recalcrate(
				struct clk_hw *hw, unsigned long parent_rate)
{
	unsigned long i, rate;
	rate = __clk_periph_get_rate(hw);
	for (i = 0; i < ARRAY_SIZE(axi5_anchor_tbl); i++) {
		if (axi5_anchor_tbl[i].axi5_rate == rate)
			break;
	}
	BUG_ON(i == ARRAY_SIZE(axi5_anchor_tbl));
	return axi5_anchor_tbl[i].pp;
}

static int axi5_clk_setrate(struct clk_hw *hw, unsigned long idx,
			unsigned long parent_rate)
{
	unsigned long old_idx;
	unsigned long axi5_rate;
	unsigned int i;
	struct clk *new_parent;

	old_idx = hw->clk->rate;
	if (idx == old_idx)
		return 0;

	for (i = 0; i < ARRAY_SIZE(axi5_anchor_tbl); i++) {
		if (axi5_anchor_tbl[i].pp == idx)
			break;
	}
	BUG_ON(i == ARRAY_SIZE(axi5_anchor_tbl));
	axi5_rate = axi5_anchor_tbl[i].axi5_rate;

	i = 0;
	while (gc3d_clk_tbl[i].clk_rate != axi5_rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc3d_clk_tbl));
	new_parent = clk_get_sys(NULL, gc3d_clk_tbl[i].parent_name);
	__clk_set_mux_div(hw, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);

	pr_info("%s PP %lu->%lu\n", hw->clk->name, old_idx, idx);
	return 0;
}

struct clk_ops axi5_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.set_rate		= axi5_clk_setrate,
	.recalc_rate	= axi5_clk_recalcrate,
	.round_rate		= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

static struct peri_params axi5_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.reg_offset = ACCU_FABRIC_N5_CLK_CNTRL_REG,
	.enable_val = ACCU_AHBCLK | ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params axi6_params = {
	.reg_offset = ACCU_FABRIC_N6_CLK_CNTRL_REG,
	.enable_val = ACCU_FNCLK,
};

#if 0 /*TODO*/
static int gc2d_clk_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long old_rate;
	unsigned int i;
	unsigned int pp_idx;
	struct clk *new_parent;

	old_rate = hw->clk->rate;
	if (rate == old_rate)
		return 0;

	for (i = 0; i < ARRAY_SIZE(axi5_anchor_tbl); i++) {
		if (rate <= axi5_anchor_tbl[i].gc2d_rate)
			break;
	}
	pm_qos_update_request(&axi5_gc2d_min, axi5_anchor_tbl[i].pp);
	pp_idx = pm_qos_request(PM_QOS_AXI5_MIN);

	i = 0;
	while (gc2d_clk_tbl[i].clk_rate != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc2d_clk_tbl));

	new_parent = gc2d_clk_tbl[i].parent;

	if (rate > old_rate) {
		clk_set_rate(&clk_axi5, pp_idx);
		__get_gc_reg_lock(hw->clk, 1, &flags);
		__clk_set_mux_div(hw->clk, new_parent, gc2d_clk_tbl[i].src_val,
					gc2d_clk_tbl[i].div_val);
		__get_gc_reg_lock(hw->clk, 0, &flags);
	} else {
		__get_gc_reg_lock(hw->clk, 1, &flags);
		__clk_set_mux_div(hw->clk, new_parent, gc2d_clk_tbl[i].src_val,
					gc2d_clk_tbl[i].div_val);
		__get_gc_reg_lock(hw->clk, 0, &flags);
		clk_set_rate(&clk_axi5, pp_idx);
	}

	pr_debug("%s rate %lu->%lu\n", hw->clk->name, old_rate, rate);
	return 0;
}

static int gc3d_fn_clk_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long old_rate;
	unsigned int i;
	unsigned int pp_idx;
	struct clk *new_parent;

	if (clk == &clk_gc3d1)
		pm_qos_update_request(&gc1_fn_qos_req_min, rate);
	else
		pm_qos_update_request(&gc2_fn_qos_req_min, rate);
	rate = pm_qos_request(PM_QOS_GC_FN_MIN);

	old_rate = clk->rate;
	if (rate == old_rate)
		return 0;

	for (i = 0; i < ARRAY_SIZE(axi5_anchor_tbl); i++)
		if (rate <= axi5_anchor_tbl[i].gc3d_fn_rate)
			break;
	pm_qos_update_request(&axi5_gc3d_fn_min, axi5_anchor_tbl[i].pp);
	pp_idx = pm_qos_request(PM_QOS_AXI5_MIN);

	i = 0;
	while (gc3d_clk_tbl[i].clk_rate != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc3d_clk_tbl));

	new_parent = gc3d_clk_tbl[i].parent;

	if (rate > old_rate) {
		clk_set_rate(&clk_axi5, pp_idx);
		gc3d_clk_set_mux_div(clk, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);
	} else {
		gc3d_clk_set_mux_div(clk, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);
		clk_set_rate(&clk_axi5, pp_idx);
	}

	pr_debug("%s rate %lu->%lu\n", clk->name, old_rate, rate);
	return 0;
}

static int gc3d_sh_clk_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long old_rate;
	unsigned int i;
	unsigned int pp_idx;
	struct clk *new_parent;

	if (hw->clk == &clk_gc3d1_shader)
		pm_qos_update_request(&gc1_sh_qos_req_min, rate);
	else
		pm_qos_update_request(&gc2_sh_qos_req_min, rate);
	rate = pm_qos_request(PM_QOS_GC_SH_MIN);

	old_rate = hw->clk->rate;
	if (rate == old_rate)
		return 0;

	for (i = 0; i < ARRAY_SIZE(axi5_anchor_tbl); i++)
		if (rate <= axi5_anchor_tbl[i].gc3d_sh_rate)
			break;
	pm_qos_update_request(&axi5_gc3d_sh_min, axi5_anchor_tbl[i].pp);
	pp_idx = pm_qos_request(PM_QOS_AXI5_MIN);

	i = 0;
	while (gc3d_clk_tbl[i].clk_rate != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc3d_clk_tbl));

	new_parent = gc3d_clk_tbl[i].parent;

	if (rate > old_rate) {
		clk_set_rate(&clk_axi5, pp_idx);
		if (gc3d_clk_tbl[i].src_val == SET_ACCU_SOURCE(CLKSRC_PLL3))
			pll3_clk_setrate(&clk_pll3, rate / MHZ_TO_HZ);
		gc3d_clk_set_mux_div(hw->clk, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);
	} else {
		if (gc3d_clk_tbl[i].src_val == SET_ACCU_SOURCE(CLKSRC_PLL3))
			pll3_clk_setrate(&clk_pll3, rate / MHZ_TO_HZ);
		gc3d_clk_set_mux_div(hw->clk, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);
		clk_set_rate(&clk_axi5, pp_idx);
	}

	pr_debug("%s rate %lu->%lu\n", hw->clk->name, old_rate, rate);
	return 0;
}


static void __get_gc_reg_lock(struct clk *clk, unsigned int lock,
	unsigned long *flags)
{
	unsigned int is_gc3d = (clk->clk_rst == ACCU_GC1_3D_CLK_CNTRL_REG) ? 1 : 0;
	unsigned int is_gc2_3d = (clk->clk_rst == ACCU_GC2_3D_CLK_CNTRL_REG) ? 1 : 0;
	unsigned int is_gc2d = (clk->clk_rst == ACCU_GC_2D_CLK_CNTRL_REG) ? 1 : 0;

	if (lock) {
		if (is_gc3d || is_gc2_3d)
			spin_lock_irqsave(&gc3dclk_lock, *flags);
		else if (is_gc2d)
			spin_lock_irqsave(&gc2dclk_lock, *flags);
	} else {
		if (is_gc3d || is_gc2_3d)
			spin_unlock_irqrestore(&gc3dclk_lock, *flags);
		else if (is_gc2d)
			spin_unlock_irqrestore(&gc2dclk_lock, *flags);
	}
}
#endif

struct clk *clk_gc3d1;
struct clk *clk_gc3d2;
struct clk *clk_gc3d1_shader;
struct clk *clk_gc3d2_shader;
struct clk *clk_gc3d1_shader;
struct clk *clk_gc3d1_shader;
struct clk *clk_gc3d_unit;
struct clk *clk_gc3d_unit_shader;

static long gc3d_clk_set_mux_div(struct clk_hw *hw, struct clk *best_parent,
				unsigned int src, unsigned int div)
{
	unsigned int regval1, regval2;
	unsigned int regval1_after, regval2_after;
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *clk = hw->clk;
	void __iomem *reg_base = peri->params->reg_base;

	/*
	 * For smooth clock src switch, must make sure both clocks on,
	 * otherwise can not finish clock switch.
	 */
	clk_prepare_enable(best_parent);
	if (!__clk_is_enabled(hw->clk) && hw->clk->parent)
		clk_prepare_enable(hw->clk->parent);

	/* disable gc1/gc2 clk, and then, reconfig freq + enable */
	regval1 = readl_relaxed(reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);
	regval2 = readl_relaxed(reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	/*
	 * if change func or sharder,
	 * need to at first gated GC1 and GC2 at first
	 */
	regval1 &= ~(peri->params->enable_val);
	regval2 &= ~(peri->params->enable_val);
	regval2_after = regval2;
	regval1_after = regval1;

	if ((clk == clk_gc3d1_shader) ||
		(clk == clk_gc3d2_shader) ||
		(clk == clk_gc3d_unit_shader)) {
		regval2_after = regval2&(~(ACCU_RATIO_MASK|ACCU_SOURCE_MASK));
		regval2_after |= src|div;
	} else if ((clk == clk_gc3d1) ||
			(clk == clk_gc3d2) ||
			(clk == clk_gc3d_unit)) {
		regval1_after = regval1&(~(ACCU_RATIO_MASK|ACCU_SOURCE_MASK));
		regval1_after |= src|div;
	}

	if (clk_gc3d1->enable_count)
		regval1_after |= ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK;
	if (clk_gc3d2->enable_count)
		regval2_after |= ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK;

	if (clk_gc3d1_shader->enable_count)
		regval1_after |= ACCU_GC3D_SHADERCLK;
	if (clk_gc3d2_shader->enable_count)
		regval2_after |= ACCU_GC3D_SHADERCLK;

	if (clk_gc3d_unit->enable_count) {
		regval1_after |= ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK;
		regval2_after |= ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK;
	}
	if (clk_gc3d_unit_shader->enable_count) {
		regval1_after |= ACCU_GC3D_SHADERCLK;
		regval2_after |= ACCU_GC3D_SHADERCLK;
	}

	writel_relaxed(regval1, reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);
	writel_relaxed(regval1_after, reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);

	writel_relaxed(regval2, reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	writel_relaxed(regval2_after, reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);

	/*
	 * disable its old parent if clk change done, no matter clk is enabled
	 * or not
	 */
	if (hw->clk->parent)
		clk_disable_unprepare(hw->clk->parent);
	if (!__clk_is_enabled(hw->clk))
		clk_disable_unprepare(best_parent);

	__clk_reparent(hw->clk, best_parent);

	return 0;
}

static int gc3d_unit_clk_enable(struct clk_hw *hw)
{
	unsigned int regval;
	unsigned long irqflags;
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_base = peri->params->reg_base;

	spin_lock_irqsave(&gc3d_lock, irqflags);
	regval = readl_relaxed(reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);
	regval |= peri->params->enable_val;
	writel_relaxed(regval, reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);

	regval = readl_relaxed(reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	regval |= peri->params->enable_val;
	writel_relaxed(regval, reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	spin_unlock_irqrestore(&gc3d_lock, irqflags);

	return 0;
}

static void gc3d_unit_clk_disable(struct clk_hw *hw)
{
	unsigned int regval;
	unsigned long irqflags;
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_base = peri->params->reg_base;

	spin_lock_irqsave(&gc3d_lock, irqflags);
	regval = readl_relaxed(reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);
	regval &= ~(peri->params->enable_val);
	writel_relaxed(regval, reg_base + ACCU_GC1_3D_CLK_CNTRL_REG);

	regval = readl_relaxed(reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	regval &= ~(peri->params->enable_val);
	writel_relaxed(regval, reg_base + ACCU_GC2_3D_CLK_CNTRL_REG);
	spin_unlock_irqrestore(&gc3d_lock, irqflags);
}

static int gc3d_unit_clk_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned long old_rate;
	unsigned int i;
	struct clk *new_parent;

	old_rate = hw->clk->rate;
	if (rate == old_rate)
		return 0;

	i = 0;
	while (gc3d_clk_tbl[i].clk_rate != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc3d_clk_tbl));

	new_parent = clk_get_sys(NULL, gc3d_clk_tbl[i].parent_name);
	gc3d_clk_set_mux_div(hw, new_parent, gc3d_clk_tbl[i].src_val,
				gc3d_clk_tbl[i].div_val);

	pr_debug("%s rate %lu->%lu\n", hw->clk->name, old_rate, rate);
	return 0;
}

struct clk_ops gc2d_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.recalc_rate	= periph_clk_recalc_rate,
	.round_rate		= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

struct clk_ops gc3d_fn_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.recalc_rate	= periph_clk_recalc_rate,
	.round_rate		= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

struct clk_ops gc3d_sh_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.recalc_rate	= periph_clk_recalc_rate,
	.round_rate		= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

struct clk_ops gc3d_unit_clk_ops = {
	.init			= periph_clk_init,
	.prepare		= periph_clk_prepare,
	.unprepare		= periph_clk_unprepare,
	.enable			= gc3d_unit_clk_enable,
	.disable		= gc3d_unit_clk_disable,
	.set_rate		= gc3d_unit_clk_setrate,
	.recalc_rate	= periph_clk_recalc_rate,
	.round_rate	= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

struct clk_ops gc3d_share_clk_ops = {
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
};

static char *gc2d_depend_clk[] = {
	"ACLK5",
	"ACLK6",
};

static char *gc3d_depend_clk[] = {
	"GC3D_FN_SHARE",
	"ACLK5",
	"ACLK6",
};

static char *gc3d_shader_depend_clk[] = {
	"GC3D_SH_SHARE",
};

/*TODO: need the share function/shader clock node*/

static struct peri_params gc2d_params = {
	.clktbl = gc2d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc2d_clk_tbl),
	.dependence = gc2d_depend_clk,
	.reg_offset = ACCU_GC_2D_CLK_CNTRL_REG,
	.enable_val = ACCU_AHBCLK | ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params gc3d1_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_depend_clk,
	.reg_offset = ACCU_GC1_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK,
};

static struct peri_params gc3d2_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_depend_clk,
	.reg_offset = ACCU_GC2_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK,
};

static struct peri_params gc3d1_sh_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_shader_depend_clk,
	.dependence_count = ARRAY_SIZE(gc3d_shader_depend_clk),
	.reg_offset = ACCU_GC1_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_SHADERCLK,
};

static struct peri_params gc3d2_sh_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_shader_depend_clk,
	.dependence_count = ARRAY_SIZE(gc3d_shader_depend_clk),
	.reg_offset = ACCU_GC2_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_SHADERCLK,
};

static struct peri_params gc3d_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_depend_clk,
	.dependence_count = ARRAY_SIZE(gc3d_depend_clk),
	.reg_offset = ACCU_GC1_3D_CLK_CNTRL_REG,
	.enable_val =  ACCU_GC3D_AHBCLK | ACCU_GC3D_APBCLK | ACCU_GC3D_FNCLK,
};

static struct peri_params gc3d_sh_params = {
	.clktbl = gc3d_clk_tbl,
	.clktblsize = ARRAY_SIZE(gc3d_clk_tbl),
	.dependence = gc3d_shader_depend_clk,
	.dependence_count = ARRAY_SIZE(gc3d_shader_depend_clk),
	.reg_offset = ACCU_GC2_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_SHADERCLK,
};

static struct peri_params gc3d_fn_shared_params = {
	.reg_offset = ACCU_GC1_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_INTLCLK,
};

static struct peri_params gc3d_sh_shared_params = {
	.reg_offset = ACCU_GC2_3D_CLK_CNTRL_REG,
	.enable_val = ACCU_GC3D_INTLCLK,
};

/*uart*/
static struct periph_clk_tbl uart_clk_tbl[] = {
	{
		.clk_rate = 14857000,
		.parent_name = PLL1624_NAME,
		.src_val = 0,
		.div_val = SET_ACCU_UARTI2C_SEL(0),
	},
	{
		.clk_rate = 59428000,
		.parent_name = PLL1624_NAME,
		.src_val = 0,
		.div_val = SET_ACCU_UARTI2C_SEL(1),
	},
};

static unsigned long uart_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned int i;
	struct clk_peri *peri = to_clk_peri(hw);

	i = GET_ACCU_UARTI2C_BITS(readl_relaxed(peri->params->reg_base +
		peri->params->reg_offset));
	return uart_clk_tbl[i].clk_rate;
}

struct clk_ops uart_clk_ops = {
	.init			= periph_clk_init,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.set_rate		= periph_clk_setrate,
	.recalc_rate		= uart_clk_recalc_rate,
	.round_rate		= periph_clk_round_rate,
};

static struct peri_params uart1_params = {
	.clktbl = uart_clk_tbl,
	.clktblsize = ARRAY_SIZE(uart_clk_tbl),
	.reg_offset = ACCU_APPS_UART1_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params uart2_params = {
	.clktbl = uart_clk_tbl,
	.clktblsize = ARRAY_SIZE(uart_clk_tbl),
	.reg_offset = ACCU_APPS_UART2_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params uart3_params = {
	.clktbl = uart_clk_tbl,
	.clktblsize = ARRAY_SIZE(uart_clk_tbl),
	.reg_offset = ACCU_APPS_UART3_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static const char const *uarti2c_parents[] = {
	PLL1624_NAME,
};

/*i2c*/
static struct periph_clk_tbl i2c_clk_tbl[] = {
	{
		.clk_rate = 32840000,
		.parent_name = PLL1624_NAME,
		.src_val = 0,
		.div_val = SET_ACCU_UARTI2C_SEL(0),
	},
	{
		.clk_rate = 69330000,
		.parent_name = PLL1624_NAME,
		.src_val = 0,
		.div_val = SET_ACCU_UARTI2C_SEL(1),
	},
};

static unsigned long i2c_clk_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned int i;
	struct clk_peri *peri = to_clk_peri(hw);

	i = GET_ACCU_UARTI2C_BITS(readl_relaxed(peri->params->reg_base +
		peri->params->reg_offset));
	return i2c_clk_tbl[i].clk_rate;
}

struct clk_ops i2c_clk_ops = {
	.init			= periph_clk_init,
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
	.set_rate		= periph_clk_setrate,
	.recalc_rate		= i2c_clk_recalc_rate,
	.round_rate		= periph_clk_round_rate,
};

static struct peri_params i2c0_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_unit = MCCU_REGS,
	.reg_offset = MCCU_I2CCR,
	.enable_val = MCCU_I2C_FNC_CKEN | MCCU_I2C_BUS_CKEN,
};

static struct peri_params i2c1_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_offset = ACCU_APPS_I2C1_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params i2c2_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_offset = ACCU_APPS_I2C2_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params i2c3_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_offset = ACCU_APPS_I2C3_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params i2c4_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_offset = ACCU_APPS_I2C4_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params i2c5_params = {
	.clktbl = i2c_clk_tbl,
	.clktblsize = ARRAY_SIZE(i2c_clk_tbl),
	.reg_offset = ACCU_APPS_I2C5_CLK_CNTRL_REG,
	.enable_val = ACCU_APBCLK | ACCU_FNCLK,
};

/*sdh*/
static struct clk_ops sdh_clk_ops = {
	.get_parent = periph_clk_get_parent,
	.enable = periph_clk_enable,
	.disable = periph_clk_disable,
};

static char *sdh_depend_clk[] = {
	"SDH-INTERNAL",
};

static struct peri_params sdh_internal_params = {
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH_INT_CLK_EN,
};

static struct peri_params sdh1_params = {
	.dependence = sdh_depend_clk,
	.dependence_count = ARRAY_SIZE(sdh_depend_clk),
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH1_CLK_EN,
};

static struct peri_params sdh2_params = {
	.dependence = sdh_depend_clk,
	.dependence_count = ARRAY_SIZE(sdh_depend_clk),
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH2_CLK_EN,
};

static struct peri_params sdh3_params = {
	.dependence = sdh_depend_clk,
	.dependence_count = ARRAY_SIZE(sdh_depend_clk),
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH3_CLK_EN,
};

static struct peri_params sdh4_params = {
	.dependence = sdh_depend_clk,
	.dependence_count = ARRAY_SIZE(sdh_depend_clk),
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH4_CLK_EN,
};

static struct peri_params sdh_mst_slv_params = {
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH_MASTER_BUS_CLK_EN | ACCU_SDH_SLAVE_BUS_CLK_EN,
};

static struct peri_params sdh_card_params = {
	.reg_offset = ACCU_SDH_CLK_CNTRL_REG,
	.enable_val = ACCU_SDH_CARD_INS_CLK_EN,
};

static const char const *thermal_parents[] = {
	CLK_26M_NAME,
};

struct clk_ops thermal_clk_ops = {
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
};

static struct peri_params thermal1_params = {
	.reg_offset = ACCU_APPS_TEMP_S1_CLK_CNTRL_REG,
	.enable_val = ACCU_FNCLK | ACCU_APBCLK |
		ACCU_APPS_TEMP_ANALOG_CLK_EN |
		ACCU_APPS_TEMP_CTL32_CLK_EN,
};

static struct peri_params thermal2_params = {
	.reg_offset = ACCU_APPS_TEMP_S2_CLK_CNTRL_REG,
	.enable_val = ACCU_FNCLK | ACCU_APBCLK |
		ACCU_APPS_TEMP_ANALOG_CLK_EN |
		ACCU_APPS_TEMP_CTL32_CLK_EN,
};

static struct peri_params thermal3_params = {
	.reg_offset = ACCU_APPS_TEMP_S3_CLK_CNTRL_REG,
	.enable_val = ACCU_FNCLK | ACCU_APBCLK |
		ACCU_APPS_TEMP_ANALOG_CLK_EN |
		ACCU_APPS_TEMP_CTL32_CLK_EN,
};

static const char const *usb_parents[] = {
	USB_PLL_NAME,
};

struct clk_ops usb_clk_ops = {
	.enable			= periph_clk_enable,
	.disable		= periph_clk_disable,
};

static struct peri_params usb2_params = {
	.reg_offset = ACCU_USB_CLK_CNTRL_REG,
	.enable_val = ACCU_USB2_APB_EN | ACCU_USB2_OTG_EN,
};

struct clk_ops vpu_clk_ops = {
	.init		= periph_clk_init,
	.prepare	= periph_clk_prepare,
	.unprepare	= periph_clk_unprepare,
	.enable		= periph_clk_enable,
	.disable	= periph_clk_disable,
	.set_rate	= periph_clk_setrate,
	.recalc_rate	= periph_clk_recalc_rate,
	.round_rate	= periph_clk_round_rate,
	.get_parent	= periph_clk_get_parent,
};

/* FIXME:
 * now fix AXI/VPU_decoder/VPU_encoder to 156MHz.
 * we need provide more freqs in the future.
 */
static struct periph_clk_tbl vpu_clk_tbl[] = {
	{
		.clk_rate = 156000000,
		.parent_name = PLL1624_NAME,
		.src_val = SET_ACCU_SOURCE(CLKSRC_PLL1_624),
		.div_val = SET_ACCU_RATIO(4),
	},
};

static struct peri_params vpu_dec_params = {
	.clktbl = vpu_clk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_clk_tbl),
	.reg_offset = ACCU_VID_DEC_CLK_CNTRL_REG,
	.enable_val = ACCU_AHBCLK | ACCU_APBCLK | ACCU_FNCLK,
};

static struct peri_params vpu_enc_params = {
	.clktbl = vpu_clk_tbl,
	.clktblsize = ARRAY_SIZE(vpu_clk_tbl),
	.reg_offset = ACCU_VID_ENC_CLK_CNTRL_REG,
	.enable_val = ACCU_AHBCLK | ACCU_APBCLK | ACCU_FNCLK,
};

/*display*/
static u32 pnsclk_parent_tbl[] = {1, 7};
static const char const *disp1_parent[] = {
	PLL2800_NAME,
	PLL1624_NAME,
	PLL1500_NAME,
	PLL1416_NAME
};
static const char const *pnsclk_parent[] = {"disp1"};
static const char const *pnpath_parent[] = {"pn_sclk"};
static const char const *pnsclk_depend[] = {"LCDCIHCLK", "vdma_axi"};
static const char const *dsi_depend[] = {"dsi_phy", "LCDCIHCLK"};

#define LCD_PN_SCLK		(0x1a8)
#define MMP_DISP_DIV_ONLY	BIT(0)
#define MMP_DISP_MUX_ONLY	BIT(1)
#define MMP_DISP_BUS		BIT(2)

static struct mmp_clk_disp vdma_axi = {
	.reg_rst = ACCU_VDMA_CLK_CNTRL_REG,
	.reg_rst_shadow = ACCU_FNCLK | ACCU_AHBCLK | ACCU_APBCLK,
	.reg_rst_mask = ACCU_FNCLK | ACCU_AHBCLK | ACCU_APBCLK,
	.lock = &disp_lock,
};


static struct mmp_clk_disp dsi_phy = {
	.reg_rst = ACCU_DISPLAY_UNIT_CLK_CNTRL_REG,
	.reg_rst_shadow = ACCU_DIS_UNIT_BUS_CLK_EN |
					ACCU_DIS_UNIT_AHB_CLK_EN |
					ACCU_LCD_PHY_CAL_CLK_EN |
					ACCU_LCD_TX_ESC_CLK_EN,
	.reg_rst_mask = ACCU_DIS_UNIT_BUS_CLK_EN |
					ACCU_DIS_UNIT_AHB_CLK_EN |
					ACCU_LCD_PHY_CAL_CLK_EN |
					ACCU_LCD_TX_ESC_CLK_EN,
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp_axi = {
	.reg_rst = ACCU_DISPLAY1_CLK_CNTRL_REG,
	.reg_rst_shadow = ACCU_FNCLK,
	.reg_rst_mask = ACCU_FNCLK,
	.lock = &disp_lock,
};

static struct mmp_clk_disp disp1 = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 0x7,
	.mux.shift = 16,
	.mux.lock = &disp_lock,
	.reg_mux = ACCU_DISPLAY1_CLK_CNTRL_REG,
	.div_ops = &clk_divider_ops,
	.divider.width = 4,
	.divider.shift = 20,
	.divider.lock = &disp_lock,
	.reg_div = ACCU_DISPLAY1_CLK_CNTRL_REG,
};

static struct mmp_clk_disp pnsclk = {
	.mux_ops = &clk_mux_ops,
	.mux.mask = 0x7,
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
	.reg_div_shadow = 0x4,
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

static struct clk *of_clk_get_parent(char *pname, struct clk *default_parent)
{
	struct device_node *np = of_find_node_by_name(NULL, pname);
	struct clk *parent = NULL;
	const char *str = NULL;

	if (np && !of_property_read_string(np, "clksrc", &str))
		parent = clk_get(NULL, str);
	pr_debug("default_parent name = %s\n", default_parent->name);
	if (parent)
		pr_debug("parent name = %s\n", parent->name);
	return parent ? parent : default_parent;
}

static void turn_pwr_islands_on(void)
{
	writel_relaxed(DISPLAY_VDMA_PWR_ON_OFF, apmu_base_va() + APMU_IMG_PWR_CTRL);
	while (!(readl_relaxed(apmu_base_va() + APMU_IMG_PWR_STATUS) & DISPLAY_VDMA_PWR_ON_OFF))
		pr_debug("%s:%d - polling APMU_IMG_PWR_STATUS\n",
							__func__, __LINE__);
}

extern struct clk *mmp_clk_register_disp(const char *name,
		const char **parent_name,
		int num_parents, u32 disp_flags, unsigned long flags,
		void __iomem *accu_base, struct mmp_clk_disp *disp);

static void __init pxa1986_disp_clk_init(void)
{
	struct clk *clk, *clk_disp1;
	unsigned long regval = 0;

	turn_pwr_islands_on();

	regval |= DISPLAY_VDMA_PWR_ON_OFF;
	__raw_writel(regval, apmu_base_va() + APMU_IMG_PWR_CTRL);

	clk = mmp_clk_register_disp("LCDCIHCLK", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, accu_base_va(), &disp_axi);
	clk_register_clkdev(clk, "LCDCIHCLK", NULL);

	clk = mmp_clk_register_disp("vdma_axi", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, accu_base_va(), &vdma_axi);
	clk_register_clkdev(clk, "vdma_axi", NULL);

	clk = mmp_clk_register_disp("dsi_phy", NULL, 0,
			MMP_DISP_BUS, CLK_IS_ROOT, accu_base_va(),
			&dsi_phy);
	clk_register_clkdev(clk, "dsi_phy", NULL);

	clk_disp1 = mmp_clk_register_disp("disp1", disp1_parent,
			ARRAY_SIZE(disp1_parent), 0, CLK_SET_RATE_PARENT,
			accu_base_va(), &disp1);
	clk_register_clkdev(clk_disp1, "disp1", NULL);

	clk = clk_get(NULL, "pll1_416");
	clk_set_parent(clk_disp1, of_clk_get_parent("disp1_clksrc", clk));

	pnsclk.mux.reg = lcd_base_va() + LCD_PN_SCLK;
	clk = mmp_clk_register_disp("pn_sclk", pnsclk_parent,
			ARRAY_SIZE(pnsclk_parent),
			MMP_DISP_MUX_ONLY, CLK_SET_RATE_PARENT, accu_base_va(),
			&pnsclk);
	clk_register_clkdev(clk, "pn_sclk", NULL);
	clk_set_parent(clk, of_clk_get_parent("pn_sclk_clksrc", clk_disp1));

	pnpath.divider.reg = lcd_base_va() + LCD_PN_SCLK;
	clk = mmp_clk_register_disp("mmp_pnpath", pnpath_parent,
			ARRAY_SIZE(pnpath_parent),
			MMP_DISP_DIV_ONLY, 0, accu_base_va(), &pnpath);
	clk_register_clkdev(clk, "mmp_pnpath", NULL);

	dsi1.divider.reg = lcd_base_va() + LCD_PN_SCLK;
	clk = mmp_clk_register_disp("mmp_dsi1", pnpath_parent,
			ARRAY_SIZE(pnpath_parent),
			MMP_DISP_DIV_ONLY, CLK_SET_RATE_PARENT, accu_base_va(),
			&dsi1);
	clk_register_clkdev(clk, "mmp_dsi1", NULL);
}

static struct clk *pxa1986_clk_register(
		const char *name, const char const **parent_name, u8 num_parents,
		unsigned long flags, struct clk_ops *ops,
		spinlock_t *lock, struct peri_params *params)
{
	struct clk_peri *peri;
	struct clk *clk;
	struct clk_init_data init;
	peri = kzalloc(sizeof(struct clk_peri), GFP_KERNEL);
	if (!peri)
		return NULL;

	init.name = name;
	init.ops = ops;
	init.flags = flags;
	init.parent_names = parent_name;
	init.num_parents = num_parents;

	if (params->reg_unit == ACCU_REGS)
		params->reg_base = accu_base_va();
	else if (params->reg_unit == MCCU_REGS)
		params->reg_base = mccu_base_va();

	peri->lock = lock;
	peri->params = params;
	peri->hw.init = &init;

	clk = clk_register(NULL, &peri->hw);
	if (IS_ERR(clk))
		kfree(peri);
	return clk;
}

void __init pxa1986_clk_init(void)
{
	struct clk *clk;
	pxa1986_pll_init();

	/*uart*/
	clk = pxa1986_clk_register("uart0", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &uart_clk_ops,
		NULL, &uart1_params);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.0");

	clk = pxa1986_clk_register("uart1", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &uart_clk_ops,
		NULL, &uart2_params);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.1");

	clk = pxa1986_clk_register("uart2", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &uart_clk_ops,
		NULL, &uart3_params);
	clk_register_clkdev(clk, NULL, "pxa2xx-uart.2");

	/*i2c*/
	clk = pxa1986_clk_register("twsi0", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c0_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.0");

	clk = pxa1986_clk_register("twsi1", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c1_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.1");

	clk = pxa1986_clk_register("twsi2", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c2_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.2");

	clk = pxa1986_clk_register("twsi3", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c3_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.3");

	clk = pxa1986_clk_register("twsi4", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c4_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.4");

	clk = pxa1986_clk_register("twsi5", uarti2c_parents,
		ARRAY_SIZE(uarti2c_parents), 0, &i2c_clk_ops,
		NULL, &i2c5_params);
	clk_register_clkdev(clk, NULL, "pxa910-i2c.5");

	/*sdh*/
	clk = pxa1986_clk_register("sdh-internal", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh_internal_params);
	clk_register_clkdev(clk, "SDH-INTERNAL", NULL);

	clk = pxa1986_clk_register("sdh1", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh1_params);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.0");

	clk = pxa1986_clk_register("sdh2", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh2_params);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.1");

	clk = pxa1986_clk_register("sdh3", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh3_params);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.2");

	clk = pxa1986_clk_register("sdh4", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh4_params);
	clk_register_clkdev(clk, NULL, "sdhci-pxav3.3");

	clk = pxa1986_clk_register("sdh-card", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh_card_params);
	clk_register_clkdev(clk, "SDH-CARD", NULL);

	clk = pxa1986_clk_register("sdh-mst-slv", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &sdh_clk_ops,
		&sdh_lock, &sdh_mst_slv_params);
	clk_register_clkdev(clk, "SDH-MASTER-SLAVE", NULL);

#if 0/*TODO*/
	/* vpu/axi2 clock */
	clk = pxa1986_clk_register("vpudec", pll_parents,
		ARRAY_SIZE(vpu_parents), 0, &vpu_decoder_clk_ops,
		&vpu_lock, &vpudec_params);
	clk_register_clkdev(clk, "VPUDEC_CLK", NULL);

	clk = pxa1986_clk_register("vpuenc", pll_parents,
		ARRAY_SIZE(vpu_parents), 0, &vpu_encoder_clk_ops,
		&vpu_lock, &vpuenc_params);
	clk_register_clkdev(clk, "VPUENC_CLK", NULL);
#endif
	/* gc/axi5/axi6 clock */
	clk = pxa1986_clk_register("axi5", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &axi5_clk_ops,
		NULL, &axi5_params);
	clk_register_clkdev(clk, "ACLK5", NULL);

	clk = pxa1986_clk_register("axi6", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &peri_clk_ops,
		NULL, &axi6_params);
	clk_register_clkdev(clk, "ACLK6", NULL);

	/*gc3d share clk*/
	clk = pxa1986_clk_register("gc3d_fn_share", NULL,
		0, 0, &gc3d_share_clk_ops,
		&gc3d_lock, &gc3d_fn_shared_params);
	clk_register_clkdev(clk, "GC3D_FN_SHARE", NULL);

	clk = pxa1986_clk_register("gc3d_sh_share", NULL,
		0, 0, &gc3d_share_clk_ops,
		&gc3d_lock, &gc3d_sh_shared_params);
	clk_register_clkdev(clk, "GC3D_SH_SHARE", NULL);

	/*gc2d clk*/
	clk = pxa1986_clk_register("gc2d", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc2d_clk_ops,
		NULL, &gc2d_params);
	clk_register_clkdev(clk, "GC2D_CLK", NULL);

	/*gc3d1/2/shader1/shader2 clk*/
	clk_gc3d1 = pxa1986_clk_register("gc3d1", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_fn_clk_ops,
		&gc3d_lock, &gc3d1_params);
	clk_register_clkdev(clk_gc3d1, "GC3D1_CLK", NULL);

	clk_gc3d2 = pxa1986_clk_register("gc3d2", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_fn_clk_ops,
		&gc3d_lock, &gc3d2_params);
	clk_register_clkdev(clk_gc3d2, "GC3D2_CLK", NULL);

	clk_gc3d1_shader = pxa1986_clk_register("gc3d1_shader", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_sh_clk_ops,
		&gc3d_lock, &gc3d1_sh_params);
	clk_register_clkdev(clk_gc3d1_shader, "GC3D1_SHADER_CLK", NULL);

	clk_gc3d2_shader = pxa1986_clk_register("gc3d2_shader", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_sh_clk_ops,
		&gc3d_lock, &gc3d2_sh_params);
	clk_register_clkdev(clk_gc3d2_shader, "GC3D2_SHADER_CLK", NULL);

	/*gc3d unit clk*/
	clk_gc3d_unit = pxa1986_clk_register("gc3d", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_unit_clk_ops,
		&gc3d_lock, &gc3d_params);
	clk_register_clkdev(clk_gc3d_unit, "GC3D_CLK", NULL);

	clk_gc3d_unit_shader = pxa1986_clk_register("gc3d_shader", pll_parents,
		ARRAY_SIZE(pll_parents), 0, &gc3d_unit_clk_ops,
		&gc3d_lock, &gc3d_sh_params);
	clk_register_clkdev(clk_gc3d_unit_shader, "GC3D_SHADER_CLK", NULL);

	pxa1986_disp_clk_init();

	clk = clk_register_fixed_rate(NULL, "rtc",
					NULL, CLK_IS_ROOT, 32768);
	clk_register_clkdev(clk, NULL, "sa1100-rtc");

	clk = clk_register_fixed_rate(NULL, "kpc",
					NULL, CLK_IS_ROOT, 32768);
	clk_register_clkdev(clk, NULL, "pxa27x-keypad");

	clk = pxa1986_clk_register("thermal1", thermal_parents,
		ARRAY_SIZE(thermal_parents), CLK_SET_RATE_PARENT,
		&thermal_clk_ops, NULL, &thermal1_params);
	clk_register_clkdev(clk, NULL, "pxa1986-thermal.1");

	clk = pxa1986_clk_register("thermal2", thermal_parents,
		ARRAY_SIZE(thermal_parents), CLK_SET_RATE_PARENT,
		&thermal_clk_ops, NULL, &thermal2_params);
	clk_register_clkdev(clk, NULL, "pxa1986-thermal.2");

	clk = pxa1986_clk_register("thermal3", thermal_parents,
		ARRAY_SIZE(thermal_parents), CLK_SET_RATE_PARENT,
		&thermal_clk_ops, NULL, &thermal3_params);
	clk_register_clkdev(clk, NULL, "pxa1986-thermal.3");

	/* FIXME remove this hard-coded soon:
	 * enable USB power island always.
	 * enable AXI3 and fix the source/ratio.
	 */
	writel_relaxed(0x1, apmu_base_va() + APMU_USB_PWR_CTRL);

	writel_relaxed(0x20420f0f, accu_base_va() + ACCU_FABRIC_N3_CLK_CNTRL_REG);

	clk = pxa1986_clk_register("usb2", usb_parents,
		ARRAY_SIZE(usb_parents), CLK_SET_RATE_PARENT,
		&usb_clk_ops, NULL, &usb2_params);
	clk_register_clkdev(clk, NULL, "mv-udc");
	clk_register_clkdev(clk, NULL, "mv-otg");
	/* we use the same phy settings as eden */
	clk_register_clkdev(clk, NULL, "pxa1986-usb-phy");

	/* FIXME remove this hard-coded soon:
	 * enable video power island always.
	 * enable AXI2 and fix the source/ratio.
	 */
	writel_relaxed(0x1, apmu_base_va() + APMU_VIDEO_PWR_CTRL);

	writel_relaxed(0x00420701, accu_base_va() +
		ACCU_FABRIC_N2_CLK_CNTRL_REG);

	clk = pxa1986_clk_register("vpu_dec", pll_parents,
		ARRAY_SIZE(pll_parents), CLK_SET_RATE_PARENT,
		&vpu_clk_ops, NULL, &vpu_dec_params);
	clk_register_clkdev(clk, "VPU_DEC_CLK", NULL);

	clk = pxa1986_clk_register("vpu_enc", pll_parents,
		ARRAY_SIZE(pll_parents), CLK_SET_RATE_PARENT,
		&vpu_clk_ops, NULL, &vpu_enc_params);
	clk_register_clkdev(clk, "VPU_ENC_CLK", NULL);
}

void hantro_power_switch(unsigned int power_on)
{
	/* FIXME: currently always enable the power of video subsystem */
	writel_relaxed(0x1, apmu_base_va() + APMU_VIDEO_PWR_CTRL);
	while (!(readl_relaxed(apmu_base_va() + APMU_VIDEO_PWR_STATUS) & VIDEO_SS_PWR_ON))
		;
}
EXPORT_SYMBOL(hantro_power_switch);
