/*
 * mmp display clock operation source file
 *
 * Copyright (C) 2013 Marvell
 * Guoqing Li <ligq@marvell.com>
 *
 * There are four levels clocks in mmp display module, the top down architecture
 * is as below:
 *   pll3      pll1
 *     |        |
 *     |    disp1(disp2) ----------(mmp_clk_disp_ops)
 *     |________|
 *              |
 *         pn_sclk(tv_sclk) -------(mmp_clk_disp_mux_ops)
 *              |
 *          pn_path(dsi) ----------(mmp_clk_disp_div_ops)
 * There are also some dependence clocks maybe needed varied on different
 * SOC: disp_axi, vdma_axi, dsi_phy_esc, etc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "clk.h"

#define HWMODE_EN	(1 << 0)
#define INT_MASK	(1 << 7)
#define PWRUP		(1 << 1)
#define PWR_STATUS	(1 << 4)
#define INT_STATUS	(1 << 8)
#define INT_CLR		(1 << 6)
/* could be shared with GC/VPU */
void mmp_clk_media_power_up(void __iomem *pwr_ctrl)
{
	u32 val;
	int count = 0;

	val = readl_relaxed(pwr_ctrl) | HWMODE_EN;
	writel_relaxed(val, pwr_ctrl);
	val &= ~INT_MASK;
	writel_relaxed(val, pwr_ctrl);
	val |= PWRUP;
	writel_relaxed(val, pwr_ctrl);

	while (!(readl_relaxed(pwr_ctrl) & PWR_STATUS)) {
		count++;
		if (count > 1000) {
			pr_err("Timeout for polling power on\n");
			return;
		}
	}
	count = 0;
	while (!(readl_relaxed(pwr_ctrl) & INT_STATUS)) {
		count++;
		if (count > 1000) {
			pr_err("Timeout for polling active interrupt\n");
			return;
		}
	}

	val = readl_relaxed(pwr_ctrl) | INT_MASK | INT_CLR;
	writel_relaxed(val, pwr_ctrl);
}

/* could be shared with GC/VPU */
void mmp_clk_media_clk_pwrdown_late(void __iomem *pwr_ctrl)
{
	u32 val;
	int count = 0;

	val = readl_relaxed(pwr_ctrl) | HWMODE_EN;
	writel_relaxed(val, pwr_ctrl);
	val &= ~PWRUP;
	writel_relaxed(val, pwr_ctrl);

	while (readl_relaxed(pwr_ctrl) & PWR_STATUS) {
		count++;
		if (count > 1000) {
			pr_err("Timeout for polling power on\n");
			return;
		}
	}
}

#define DMMY_CLK	(1 << 4)
/* could be shared with GC/VPU */
void mmp_clk_media_clk_pre_enable(void __iomem *isld_ctrl)
{
	u32 val;

	val = readl_relaxed(isld_ctrl) | DMMY_CLK;
	writel_relaxed(val, isld_ctrl);
	/* wait for 500ns */
	udelay(2);

	val &= ~DMMY_CLK;
	writel_relaxed(val, isld_ctrl);
}

#define DISP_PWR_CTRL	(0x204)
#define ISLD_LCD_CTRL	(0x1ac)
static int mmp_clk_disp_rst_enable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	u32 val;

	mmp_clk_media_power_up(disp->apmu_base + DISP_PWR_CTRL);

	mmp_clk_media_clk_pre_enable(disp->apmu_base + ISLD_LCD_CTRL);

	val = readl_relaxed(disp->apmu_base + disp->reg_rst);
	val |= (1 << 1) | (1 << 2);
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);
	/* wait for 500ns */
	udelay(2);

	val |= (1 << 3) | (1 << 4) | (1 << 5);
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);
	/* wait for 500ns */
	udelay(2);

	val |= 1;
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);
	/* wait for 500ns */
	udelay(2);

	return 0;
}

static void mmp_clk_disp_rst_disable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	u32 val;

	val = readl_relaxed(disp->apmu_base + disp->reg_rst);
	val &= ~0x3f;
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);

	mmp_clk_media_clk_pwrdown_late(disp->apmu_base + DISP_PWR_CTRL);
}

static struct clk_ops mmp_clk_disp_rst_ops = {
	.enable = mmp_clk_disp_rst_enable,
	.disable = mmp_clk_disp_rst_disable,
};

static int mmp_clk_disp_bus_enable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	unsigned long flags = 0;
	u32 val;

	if (disp->lock)
		spin_lock_irqsave(disp->lock, flags);
	val = readl_relaxed(disp->apmu_base + disp->reg_rst);
	val &= ~(disp->reg_rst_mask);
	val |= disp->reg_rst_shadow;
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);
	if (disp->lock)
		spin_unlock_irqrestore(disp->lock, flags);

	pr_debug("%s: disp->reg_rst %p is set to %x\n", __func__,
		 disp->apmu_base + disp->reg_rst, val);
	return 0;
}

static void mmp_clk_disp_bus_disable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	unsigned long flags = 0;
	u32 val;

	if (disp->lock)
		spin_lock_irqsave(disp->lock, flags);
	val = readl_relaxed(disp->apmu_base + disp->reg_rst);
	val &= ~(disp->reg_rst_mask);
	writel_relaxed(val, disp->apmu_base + disp->reg_rst);
	if (disp->lock)
		spin_unlock_irqrestore(disp->lock, flags);

	pr_debug("%s: disp->reg_rst %p is set to %x\n", __func__,
		 disp->apmu_base + disp->reg_rst, val);
}

static int mmp_clk_disp_prepare(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	struct clk *clk_depend;
	int i;

	for (i = 0; i < disp->num_dependence; i++) {
		clk_depend = clk_get(NULL, disp->dependence[i]);
		clk_prepare_enable(clk_depend);
	}

	return 0;
}

static void mmp_clk_disp_unprepare(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	struct clk *clk_depend;
	int i;

	for (i = 0; i < disp->num_dependence; i++) {
		clk_depend = clk_get(NULL, disp->dependence[i]);
		clk_disable_unprepare(clk_depend);
	}
}

static struct clk_ops mmp_clk_disp_bus_ops = {
	.prepare = mmp_clk_disp_prepare,
	.unprepare = mmp_clk_disp_unprepare,
	.enable = mmp_clk_disp_bus_enable,
	.disable = mmp_clk_disp_bus_disable,
};

static int __mmp_clk_depend_is_enabled(struct mmp_clk_disp *disp)
{
	struct clk *clk_depend;
	int i;

	for (i = 0; i < disp->num_dependence; i++) {
		clk_depend = clk_get(NULL, disp->dependence[i]);
		if (!__clk_is_enabled(clk_depend))
			return 0;
	}

	return 1;
}

static void mmp_clk_disp_disable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	u32 val;

	/* disable clock */
	val = readl_relaxed(disp->divider.reg);
	val |= (disp->reg_rst_mask);
	writel_relaxed(val, disp->divider.reg);
}

static int mmp_clk_disp_div_enable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	u32 val, mask = ((1 << disp->divider.width) - 1) << disp->divider.shift;

	val = readl_relaxed(disp->divider.reg);
	val &= ~mask;
	val |= disp->reg_div_shadow & mask;
	/* enable clock */
	val &= ~(disp->reg_rst_mask);
	writel_relaxed(val, disp->divider.reg);
	pr_debug("%s: disp->divider reg %p is set to %x\n",
		 __func__, disp->divider.reg, val);
	return 0;
}

static unsigned long mmp_clk_disp_div_recalc_rate(struct clk_hw *hw,
			unsigned long prate)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *div_ops = disp->div_ops;
	struct clk_hw *div_hw = &disp->divider.hw;
	unsigned long ret;
	void __iomem *tmp;

	if (!div_hw->clk)
		div_hw->clk = hw->clk;

	if (__mmp_clk_depend_is_enabled(disp)) {
		ret = div_ops->recalc_rate(div_hw, prate);
		disp->reg_div_shadow = readl_relaxed(disp->divider.reg);
	} else {
		/*
		 * depends clock maybe not enabled,
		 * save the configration in shadow
		 */
		tmp = disp->divider.reg;
		disp->divider.reg = (void __iomem *)&disp->reg_div_shadow;
		ret = div_ops->recalc_rate(div_hw, prate);
		disp->divider.reg = tmp;
	}

	return ret;
}

static int mmp_clk_disp_div_setrate(struct clk_hw *hw,
		unsigned long rate, unsigned long prate)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *div_ops = disp->div_ops;
	struct clk_hw *div_hw = &disp->divider.hw;
	u32 mask = ((1 << disp->divider.width) - 1) << disp->divider.shift;
	unsigned long ret;
	void __iomem *tmp;

	if (!div_hw->clk)
		div_hw->clk = hw->clk;

	if (__mmp_clk_depend_is_enabled(disp)) {
		ret = div_ops->set_rate(div_hw, rate, prate);
		disp->reg_div_shadow = readl_relaxed(disp->divider.reg);
		if (!(disp->reg_div_shadow & mask)) {
			/* div 0 indicates disable */
			disp->reg_div_shadow |= 1 << disp->divider.shift;
			writel_relaxed(disp->reg_div_shadow, disp->divider.reg);
		}
	} else {
		/*
		 * depends clock maybe not enabled,
		 * save the configration in shadow
		 */
		tmp = disp->divider.reg;
		disp->divider.reg = (void __iomem *)&disp->reg_div_shadow;
		ret = div_ops->set_rate(div_hw, rate, prate);
		disp->divider.reg = tmp;
		if (!(disp->reg_div_shadow & mask))
			/* div 0 indicates disable */
			disp->reg_div_shadow |= 1 << disp->divider.shift;
	}

	pr_debug("%s: rate: %lu, prate: %lu\n", __func__, rate, prate);
	return ret;
}

static int mmp_clk_disp_mux_enable(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	u32 val;

	val = readl_relaxed(disp->mux.reg);
	val &= ~(disp->mux.mask << disp->mux.shift);
	val |= disp->reg_mux_shadow & (disp->mux.mask << disp->mux.shift);
	writel_relaxed(val, disp->mux.reg);

	pr_debug("%s: disp->mux reg %p is set to %x\n",
		 __func__, disp->mux.reg, val);
	return 0;
}

static u8 mmp_clk_disp_mux_get_parent(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *mux_ops = disp->mux_ops;
	struct clk_hw *mux_hw = &disp->mux.hw;
	u8 ret;
	void __iomem *tmp;

	if (!mux_hw->clk)
		mux_hw->clk = hw->clk;

	tmp = disp->mux.reg;
	disp->mux.reg = (void __iomem *)&disp->reg_mux_shadow;
	ret = mux_ops->get_parent(mux_hw);
	disp->mux.reg = tmp;

	return ret;
}

static int mmp_clk_disp_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *mux_ops = disp->mux_ops;
	struct clk_hw *mux_hw = &disp->mux.hw;
	unsigned long ret;
	void __iomem *tmp;

	if (!mux_hw->clk)
		mux_hw->clk = hw->clk;

	if (__mmp_clk_depend_is_enabled(disp)) {
		ret = mux_ops->set_parent(mux_hw, index);
		disp->reg_mux_shadow = readl_relaxed(disp->mux.reg);
	} else {
		/*
		 * depends clock maybe not enabled,
		 * save the configration in shadow
		 */
		tmp = disp->mux.reg;
		disp->mux.reg = (void __iomem *)&disp->reg_mux_shadow;
		ret = mux_ops->set_parent(mux_hw, index);
		disp->mux.reg = tmp;
	}

	return ret;
}

static unsigned long mmp_clk_disp_recalc_rate(struct clk_hw *hw,
			unsigned long prate)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *div_ops = disp->div_ops;
	struct clk_hw *div_hw = &disp->divider.hw;

	if (!div_hw->clk)
		div_hw->clk = hw->clk;

	return div_ops->recalc_rate(div_hw, prate);
}

static int mmp_clk_disp_setrate(struct clk_hw *hw,
		unsigned long rate, unsigned long prate)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *div_ops = disp->div_ops;
	struct clk_hw *div_hw = &disp->divider.hw;

	if (!div_hw->clk)
		div_hw->clk = hw->clk;

	pr_debug("%s: rate %lu, prate %lu\n", __func__, rate, prate);
	return div_ops->set_rate(div_hw, rate, prate);
}

static long mmp_clk_disp_round_rate(struct clk_hw *hw,
		unsigned long rate, unsigned long *prate)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	struct clk_hw *div_hw = &disp->divider.hw;
	int maxdiv, div;
	unsigned long parent_rate;

	if (!div_hw->clk)
		div_hw->clk = hw->clk;

	maxdiv = ((1 << disp->divider.width) - 1);
	if ((__clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT))
		*prate = __clk_round_rate(__clk_get_parent(hw->clk), rate);

	parent_rate = *prate + (rate >> 1);
	div = parent_rate / rate;
	div = div == 0 ? 1 : div;
	div = div > maxdiv ? maxdiv : div;

	return *prate / div;
}

static u8 mmp_clk_disp_get_parent(struct clk_hw *hw)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *mux_ops = disp->mux_ops;
	struct clk_hw *mux_hw = &disp->mux.hw;

	if (!mux_hw->clk)
		mux_hw->clk = hw->clk;

	return mux_ops->get_parent(mux_hw);
}

static int mmp_clk_disp_set_parent(struct clk_hw *hw, u8 index)
{
	struct mmp_clk_disp *disp = to_mmp_clk_disp(hw);
	const struct clk_ops *mux_ops = disp->mux_ops;
	struct clk_hw *mux_hw = &disp->mux.hw;

	if (!mux_hw->clk)
		mux_hw->clk = hw->clk;

	return mux_ops->set_parent(mux_hw, index);
}

static struct clk_ops mmp_clk_disp_div_ops = {
	.prepare = mmp_clk_disp_prepare,
	.unprepare = mmp_clk_disp_unprepare,
	.enable = mmp_clk_disp_div_enable,
	.disable = mmp_clk_disp_disable,
	.set_rate = mmp_clk_disp_div_setrate,
	.recalc_rate = mmp_clk_disp_div_recalc_rate,
	.round_rate = mmp_clk_disp_round_rate,
};

static struct clk_ops mmp_clk_disp_mux_ops = {
	.prepare = mmp_clk_disp_prepare,
	.unprepare = mmp_clk_disp_unprepare,
	.enable = mmp_clk_disp_mux_enable,
	.disable = mmp_clk_disp_disable,
	.get_parent = mmp_clk_disp_mux_get_parent,
	.set_parent = mmp_clk_disp_mux_set_parent,
};

static struct clk_ops mmp_clk_disp_ops = {
	.set_rate = mmp_clk_disp_setrate,
	.recalc_rate = mmp_clk_disp_recalc_rate,
	.round_rate = mmp_clk_disp_round_rate,
	.get_parent = mmp_clk_disp_get_parent,
	.set_parent = mmp_clk_disp_set_parent,
};

struct clk *mmp_clk_register_disp(const char *name, const char **parent_name,
		int num_parents, u32 disp_flags, unsigned long flags,
		void __iomem *apmu_base, struct mmp_clk_disp *disp)
{
	struct clk *clk;
	struct clk_init_data init;

	init.name = name;
	switch (disp_flags) {
	case MMP_DISP_MUX_ONLY:
		init.ops = &mmp_clk_disp_mux_ops;
		break;
	case MMP_DISP_DIV_ONLY:
		init.ops = &mmp_clk_disp_div_ops;
		break;
	case MMP_DISP_BUS:
		init.ops = &mmp_clk_disp_bus_ops;
		break;
	case MMP_DISP_RST_CTRL:
		init.ops = &mmp_clk_disp_rst_ops;
		break;
	default:
		init.ops = &mmp_clk_disp_ops;
		break;
	}

	init.flags = flags;
	init.parent_names = parent_name;
	init.num_parents = num_parents;
	disp->hw.init = &init;
	disp->apmu_base = apmu_base;
	disp->mux.reg = disp->mux.reg ? disp->mux.reg :
		apmu_base + disp->reg_mux;
	disp->divider.reg = disp->divider.reg ? disp->divider.reg :
		apmu_base + disp->reg_div;
	disp->divider.flags = CLK_DIVIDER_ALLOW_ZERO | CLK_DIVIDER_ONE_BASED;

	clk = clk_register(NULL, &disp->hw);

	disp->mux.hw.clk = clk;
	disp->divider.hw.clk = clk;

	return clk;
}
