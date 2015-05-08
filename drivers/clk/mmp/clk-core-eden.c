/*
 * eden core clock framework source file
 *
 * Copyright (C) 2013 Marvell
 * Yipeng Yao <ypyao@marvell.com>
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
#include <linux/cpufreq.h>
#include "clk.h"

#ifdef CONFIG_ARM64
static inline int cpu_is_eden_z1(void) { return 0; }
static inline int cpu_is_eden_z2(void) { return 0; }
#else
#include <mach/cputype.h>
#endif

#define APMU_COREAPSS_CLKCTRL		(0x2E0)
static void __iomem *apmu_coreapss_clkctrl;
	/* bit definition of APMU_COREAPSS_CLKCTRL */
#define COREFREQ_SRC_SHIFT		(30)
#define COREFREQ_RTCWTC_SHIFT		(27)
#define COREFREQ_AXIM_SHIFT		(8)
	/* end */

#define APMU_COREAPSS_CFG		(0x39C)
#define COREAPSS_CFG_WTCRTC_MSK		(0xFFFFFF00)
#define COREAPSS_CFG_WTCRTC_NZ1_HMIPS	(0x50505000)
#define COREAPSS_CFG_WTCRTC_NZ1_EHMIPS	(0x50506000)

#define KHZ_TO_HZ	(1000)
#define MHZ_TO_KHZ	(1000)

static DEFINE_SPINLOCK(cpu_clk_lock);
static void __iomem *apmu_coreapss_cfg;
static struct clk *pll2_clk, *pll2p_clk;

enum core_clk_sel {
	CORE_PLL1	= 0,
	CORE_PLL2	= 1,
	CORE_PLL2P	= 3,
};

struct eden_core_opt {
	unsigned int freq_khz;
	enum core_clk_sel src;
	unsigned int div;
	unsigned int rtc_wtc;
	unsigned int axim_div;
	unsigned int src_freq;
};
static struct eden_core_opt *cur_core_opt;
static int cur_core_opt_size;

static unsigned int pll2_oparray[] = {
	797000, 1057000, 1386000,
};

static struct eden_core_opt core_oparray[] = {
	/*{156000, CORE_PLL1,	4, 0, 1, 624000},*/
	{312000, CORE_PLL1,	2, 0, 2, 624000},
	/*{398000, CORE_PLL2,	2, 0, 4, 797000},
	{531000, CORE_PLL2P,	1, 0, 4, 531000},*/
	{624000, CORE_PLL1,	1, 1, 4, 624000},
	{797000, CORE_PLL2,	1, 1, 4, 797000},
	{1057000, CORE_PLL2,	1, 1, 4, 1057000},
	/*{1386000, CORE_PLL2,	1, 1, 2, 1386000},*/
};

static struct eden_core_opt core_oparray_z2[] = {
	{156000, CORE_PLL1,	4, 0, 2, 624000},
	{312000, CORE_PLL1,	2, 0, 2, 624000},
	{398000, CORE_PLL2,	2, 0, 2, 797000},
	{531000, CORE_PLL2P,	1, 0, 2, 531000},
	{624000, CORE_PLL1,	1, 0, 2, 624000},
	{797000, CORE_PLL2,	1, 0, 2, 797000},
	{1057000, CORE_PLL2,	1, 1, 2, 1057000},
	{1386000, CORE_PLL2,	1, 1, 2, 1386000},
};

static void core_setrate_idx(int idx)
{
	int src, div, rwtc, axim, axim_cur;
	u32 reg, val;
	struct eden_core_opt *core_op_array;

	core_op_array = cur_core_opt;
	src = core_op_array[idx].src;
	div = core_op_array[idx].div;
	rwtc = core_op_array[idx].rtc_wtc;
	axim = core_op_array[idx].axim_div;

	reg = readl(apmu_coreapss_clkctrl);
	axim_cur = (reg >> COREFREQ_AXIM_SHIFT) & 0x7;

	/* Walkaround: if axim is getting slower, we should
	 * change axim freq firtst, then core freq */
	if (axim > axim_cur) {
		reg &= ~(0x7 << COREFREQ_AXIM_SHIFT);
		reg |= (axim << COREFREQ_AXIM_SHIFT);
		writel(reg, apmu_coreapss_clkctrl);
		/* dummy read */
		reg = readl(apmu_coreapss_clkctrl);
	}

	if (!cpu_is_eden_z1()) {
		val = __raw_readl(apmu_coreapss_cfg);
		val &= ~(COREAPSS_CFG_WTCRTC_MSK);
		if (core_op_array[idx].freq_khz > 1400000)
			val |= (COREAPSS_CFG_WTCRTC_NZ1_EHMIPS);
		else
			val |= (COREAPSS_CFG_WTCRTC_NZ1_HMIPS);
		__raw_writel(val, apmu_coreapss_cfg);
	}

	reg &= ~(0x3 << COREFREQ_SRC_SHIFT);
	reg |= (src << COREFREQ_SRC_SHIFT);
	reg &= ~(0x1 << COREFREQ_RTCWTC_SHIFT);
	reg |= (rwtc << COREFREQ_RTCWTC_SHIFT);
	reg &= ~0x7;
	reg |= div;

	/* Walkaround: if axim is getting faster, we should
	 * change core frequency firtst, then axim freq */
	if (axim < axim_cur) {
		writel(reg, apmu_coreapss_clkctrl);
		/* dummy read */
		reg = readl(apmu_coreapss_clkctrl);
		reg &= ~(0x7 << COREFREQ_AXIM_SHIFT);
		reg |= (axim << COREFREQ_AXIM_SHIFT);
	}

	writel(reg, apmu_coreapss_clkctrl);
	/* dummy read */
	reg = readl(apmu_coreapss_clkctrl);
}

static inline void jump_to_tmp_pll1(int idx)
{
	int i;
	struct eden_core_opt *core_op_array;

	core_op_array = cur_core_opt;

	if (idx <= 0)
		return;
	for (i = idx - 1; i >= 0; i--) {
		if (core_op_array[i].src == CORE_PLL1)
			break;
	}
	core_setrate_idx(i);
	return;
}

static int cpu_clk_setrate(struct clk_hw *hw, unsigned long rate,
				unsigned long prate)
{
	u32 reg;
	int i, src, cur_src;
	int ret = 0;
	unsigned long flags;
	struct eden_core_opt *core_op_array;
	unsigned int core_opt_size = 0;

	if (!pll2_clk || !pll2p_clk) {
		ret = -EINVAL;
		goto cpu_clk_setrate_err;
	}

	core_op_array = cur_core_opt;
	core_opt_size = cur_core_opt_size;

	/* kHz align */
	rate /= KHZ_TO_HZ;
	pr_debug("[Core DFC]-enter: target value: %lukHZ...\n", rate);

	spin_lock_irqsave(&cpu_clk_lock, flags);
	reg = readl(apmu_coreapss_clkctrl);
	cur_src = (reg >> COREFREQ_SRC_SHIFT) & 0x3;

	for (i = 0; i < core_opt_size; i++) {
		if (core_op_array[i].freq_khz == rate)
			break;
	}
	src = core_op_array[i].src;

	switch (cur_src) {
	case CORE_PLL1:
		if (src == CORE_PLL2) {
			clk_set_rate(pll2_clk,
				     core_op_array[i].src_freq * 1000);
			clk_prepare_enable(pll2_clk);
		} else if (src == CORE_PLL2P) {
			clk_set_rate(pll2p_clk,
				     core_op_array[i].src_freq * 1000);
			clk_prepare_enable(pll2p_clk);
		}
		core_setrate_idx(i);
		break;
	case CORE_PLL2:
		if (src == CORE_PLL1) {
			core_setrate_idx(i);
			clk_disable_unprepare(pll2_clk);
		} else if (src == CORE_PLL2P) {
			jump_to_tmp_pll1(i);
			clk_disable_unprepare(pll2_clk);
			clk_set_rate(pll2p_clk,
				     core_op_array[i].src_freq * 1000);
			clk_prepare_enable(pll2p_clk);
			core_setrate_idx(i);
		} else if (src == CORE_PLL2) {
			jump_to_tmp_pll1(i);
			clk_disable_unprepare(pll2_clk);
			clk_set_rate(pll2_clk,
				     core_op_array[i].src_freq * 1000);
			clk_prepare_enable(pll2_clk);
			core_setrate_idx(i);
		}
		break;
	case CORE_PLL2P:
		if (src == CORE_PLL1) {
			core_setrate_idx(i);
			clk_disable_unprepare(pll2p_clk);
		} else if (src == CORE_PLL2) {
			jump_to_tmp_pll1(i);
			clk_disable_unprepare(pll2p_clk);
			clk_set_rate(pll2_clk,
				     core_op_array[i].src_freq * 1000);
			clk_prepare_enable(pll2_clk);
			core_setrate_idx(i);
		}
		break;
	}
	spin_unlock_irqrestore(&cpu_clk_lock, flags);

cpu_clk_setrate_err:
	if (ret)
		pr_debug("[Core DFC]-exit: %lukHZ change FAIL\n", rate);
	else
		pr_debug("[Core DFC]-exit: %lukHZ change SUCCESS\n", rate);

	return ret;
}

static inline unsigned int pll2_round_rate(unsigned int raw_rate)
{
	const int gap = 20000;
	int i;

	for (i = 0; i < ARRAY_SIZE(pll2_oparray); i++) {
		if ((raw_rate > (pll2_oparray[i] - gap))
			&& (raw_rate < (pll2_oparray[i]) + gap))
			return pll2_oparray[i];
	}
	return 0;
}

static unsigned long cpu_clk_recalc(struct clk_hw *hw, unsigned long prate)
{
	int i, src, div;
	unsigned int pll2_rate;
	unsigned int ret = 0;
	unsigned long flags;
	u32 reg;
	struct eden_core_opt *core_op_array;
	unsigned int core_opt_size = 0;

	core_op_array = cur_core_opt;
	core_opt_size = cur_core_opt_size;

	spin_lock_irqsave(&cpu_clk_lock, flags);
	reg = readl(apmu_coreapss_clkctrl);
	src = (reg >> COREFREQ_SRC_SHIFT) & 0x3;
	div = reg & 0x7;

	switch (src) {
	case CORE_PLL1:
	case CORE_PLL2P:
		for (i = 0; i < core_opt_size; i++) {
			if ((src == core_op_array[i].src) &&
				(div == core_op_array[i].div)) {
				ret = core_op_array[i].freq_khz;
				break;
			}
		}
		break;
	case CORE_PLL2:
		if (!pll2_clk)
			goto cpu_getrate_err;
		pll2_rate = clk_get_rate(pll2_clk) / KHZ_TO_HZ;
		pll2_rate = pll2_round_rate(pll2_rate);
		if (!pll2_rate)
			goto cpu_getrate_err;
		ret = pll2_rate / div / MHZ_TO_KHZ;	/* MHz align */
		ret *= KHZ_TO_HZ;			/* KHz align */
		break;
	default:
		goto cpu_getrate_err;
		break;
	}

	spin_unlock_irqrestore(&cpu_clk_lock, flags);
	/* Hz align */
	return ret * KHZ_TO_HZ;

cpu_getrate_err:
	pr_err("Cannot get frequency value for cpu core!\n");
	return 0;
}

static void cpu_clk_init(struct clk_hw *hw)
{
	hw->clk->rate = cpu_clk_recalc(hw, 0);
	pr_info("CPU boot up @%luHZ\n", hw->clk->rate);
}

static long cpu_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	int i, num;
	struct eden_core_opt *core_op_array;
	unsigned int core_opt_size = 0;

	core_op_array = cur_core_opt;
	core_opt_size = cur_core_opt_size;

	/* kHz align */
	rate /= 1000;

	num = core_opt_size;
	if (rate >= core_op_array[num - 1].freq_khz)
		return (core_op_array[num - 1].freq_khz) * 1000;

	for (i = 0; i < num; i++) {
		if (core_op_array[i].freq_khz >= rate)
			break;
	}
	return (core_op_array[i].freq_khz) * KHZ_TO_HZ;
}

#ifdef CONFIG_CPU_FREQ_TABLE
static void __init_cpufreq_table(void)
{
	int i, num;
	struct eden_core_opt *core_op_array;
	unsigned int core_opt_size = 0;
	struct cpufreq_frequency_table *cpufreq_tbl;

	core_op_array = cur_core_opt;
	core_opt_size = cur_core_opt_size;

	num = core_opt_size;
	cpufreq_tbl =
		kmalloc(sizeof(struct cpufreq_frequency_table) *
					(num + 1), GFP_KERNEL);
	if (!cpufreq_tbl)
		return;

	for (i = 0; i < num; i++) {
		cpufreq_tbl[i].index = i;
		cpufreq_tbl[i].frequency = core_op_array[i].freq_khz;
	}

	cpufreq_tbl[i].index = i;
	cpufreq_tbl[i].frequency = CPUFREQ_TABLE_END;

	for_each_possible_cpu(i)
		cpufreq_frequency_table_get_attr(cpufreq_tbl, i);
}
#else
#define __init_cpufreq_table() do {} while (0)
#endif

static struct clk_ops cpu_clk_ops = {
	.init		= cpu_clk_init,
	.set_rate	= cpu_clk_setrate,
	.round_rate	= cpu_clk_round_rate,
	.recalc_rate	= cpu_clk_recalc,
};

void __init eden_core_clk_init(unsigned long axi_phy_base)
{
	struct clk *clk;
	struct clk_hw *hw;
	struct clk_init_data init;
	void __iomem *apmu_base;
	u32 reg;
	int src;

	apmu_base = ioremap(axi_phy_base + 0x82800, SZ_4K);
	if (apmu_base == NULL) {
		pr_err("error to ioremap APMU base\n");
		return;
	}
	apmu_coreapss_clkctrl = apmu_base + APMU_COREAPSS_CLKCTRL;
	apmu_coreapss_cfg = apmu_base + APMU_COREAPSS_CFG;

	hw = kzalloc(sizeof(struct clk_hw), GFP_KERNEL);
	if (!hw) {
		pr_err("error to alloc mem for cpu clk\n");
		return;
	}

	if (cpu_is_eden_z1()) {
		/* ASC code z1 */
		cur_core_opt = core_oparray;
		cur_core_opt_size = ARRAY_SIZE(core_oparray);
	} else if (cpu_is_eden_z2()) {
		/* ASC code z2 */
		cur_core_opt = core_oparray_z2;
		cur_core_opt_size = ARRAY_SIZE(core_oparray_z2);
	} else {
		pr_err("unknown eden soc stepping\n");
		return;
	}

	pll2p_clk = clk_get(NULL, "pll2p");
	pll2_clk = clk_get(NULL, "pll2");
	if (!pll2p_clk || !pll2_clk) {
		pr_err("Cannot find clock source for CPU!\n");
		return;
	}
	reg = readl(apmu_coreapss_clkctrl);
	src = (reg >> COREFREQ_SRC_SHIFT) & 0x3;
	switch (src) {
	case CORE_PLL1:
		pll2p_clk->ops->disable(pll2p_clk->hw);
		pll2_clk->ops->disable(pll2_clk->hw);
		break;
	case CORE_PLL2:
		pll2p_clk->ops->disable(pll2p_clk->hw);
		clk_prepare_enable(pll2_clk);
		break;
	case CORE_PLL2P:
		clk_prepare_enable(pll2p_clk);
		break;
	}

	init.name = "cpu";
	init.ops = &cpu_clk_ops;
	init.flags = CLK_IS_ROOT;
	init.parent_names = NULL;
	init.num_parents = 0;
	hw->init = &init;

	clk = clk_register(NULL, hw);
	if (IS_ERR(clk)) {
		pr_err("error to register cpu clk\n");
		kfree(hw);
		return;
	}
	clk_register_clkdev(clk, "cpu", NULL);
	clk_prepare_enable(clk);

	__init_cpufreq_table();
}
