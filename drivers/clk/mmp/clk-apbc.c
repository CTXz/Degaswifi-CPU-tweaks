/*
 * mmp APB clock operation source file
 *
 * Copyright (C) 2012 Marvell
 * Chao Xie <xiechao.mail@gmail.com>
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

/* Common APB clock register bit definitions */
#define APBC_APBCLK	(1 << 0)  /* APB Bus Clock Enable */
#define APBC_FNCLK	(1 << 1)  /* Functional Clock Enable */
#define APBC_RST	(1 << 2)  /* Reset Generation */
#define APBC_POWER	(1 << 7)  /* Reset Generation */

#define to_clk_apbc(hw_var) container_of(hw_var, struct clk_apbc, hw)
struct clk_apbc {
	struct clk_hw		hw;
	void __iomem		*base;
	unsigned int		delay;
	unsigned int		flags;
	spinlock_t		*lock;
};

static void clk_apbc_set_reg(struct clk_apbc *apbc, u32 mask, u32 set)
{
	u32 data;
	unsigned long flags = 0;

	if (apbc->lock)
		spin_lock_irqsave(apbc->lock, flags);

	data = readl_relaxed(apbc->base);
	data &= ~mask;
	data |= set;
	writel_relaxed(data, apbc->base);

	if (apbc->lock)
		spin_unlock_irqrestore(apbc->lock, flags);
}

static int clk_apbc_prepare(struct clk_hw *hw)
{
	struct clk_apbc *apbc = to_clk_apbc(hw);

	/*
	 * It may share same register as MUX clock,
	 * and it will impact FNCLK enable. Spinlock is needed
	 */
	clk_apbc_set_reg(apbc, 0,
			APBC_FNCLK | ((apbc->flags & APBC_POWER_CTRL)
			? APBC_POWER : 0));
	udelay(apbc->delay);

	clk_apbc_set_reg(apbc, 0, APBC_APBCLK);
	udelay(apbc->delay);

	if (!(apbc->flags & APBC_NO_BUS_CTRL))
		clk_apbc_set_reg(apbc, APBC_RST, 0);

	return 0;
}

static void clk_apbc_unprepare(struct clk_hw *hw)
{
	struct clk_apbc *apbc = to_clk_apbc(hw);

	clk_apbc_set_reg(apbc,
			APBC_FNCLK | ((apbc->flags & APBC_POWER_CTRL)
			? APBC_POWER : 0), 0);
	udelay(10);

	clk_apbc_set_reg(apbc, APBC_APBCLK, 0);
}

struct clk_ops clk_apbc_ops = {
	.prepare = clk_apbc_prepare,
	.unprepare = clk_apbc_unprepare,
};

static void clk_pwm_get_share(struct clk *clk,
		struct clk **clk_apb, struct clk **clk_share)
{
	/*
	 * A dependence exists between pwm0 and pwm1. pwm0 can controll its
	 * apb bus clk independently, while pwm1 apb bus clk is controlled
	 * by pwm0's. The same relationship exists between pwm2 and pwm3.
	 */
	if (!strcmp(clk->name, "pwm1")) {
		*clk_share = clk_get_sys("mmp-pwm.0", NULL);
		BUG_ON(IS_ERR(*clk_share));
		*clk_apb = *clk_share;
	} else if (!strcmp(clk->name, "pwm2")) {
		*clk_share = clk_get_sys("mmp-pwm.3", NULL);
		BUG_ON(IS_ERR(*clk_share));
		*clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm3")) {
		*clk_share = clk_get_sys("mmp-pwm.2", NULL);
		BUG_ON(IS_ERR(*clk_share));
		*clk_apb = *clk_share;
	} else {
		*clk_share = clk_get_sys("mmp-pwm.1", NULL);
		BUG_ON(IS_ERR(*clk_share));
		*clk_apb = clk;
	}
}

static int clk_apbc_pwm_enable(struct clk_hw *hw)
{
	struct clk_apbc *apbc = to_clk_apbc(hw);
	struct clk_apbc *apbc_apb;
	struct clk *clk_apb, *clk_share;

	clk_pwm_get_share(hw->clk, &clk_apb, &clk_share);
	apbc_apb = to_clk_apbc(clk_apb->hw);

	clk_apbc_set_reg(apbc, 0, APBC_FNCLK);
	if ((hw->clk->enable_count + clk_share->enable_count) == 0) {
		clk_apbc_set_reg(apbc_apb, 0, APBC_APBCLK);
		clk_apbc_set_reg(apbc, APBC_RST, 0);
		if (strcmp(hw->clk->name, clk_apb->name))
			clk_apbc_set_reg(apbc_apb, APBC_RST, 0);
	}
	udelay(apbc->delay);

	return 0;
}

static void clk_apbc_pwm_disable(struct clk_hw *hw)
{
	struct clk_apbc *apbc = to_clk_apbc(hw);
	struct clk_apbc *apbc_apb;
	struct clk *clk_apb, *clk_share;

	clk_apbc_set_reg(apbc, APBC_FNCLK, 0);
	udelay(apbc->delay);

	clk_pwm_get_share(hw->clk, &clk_apb, &clk_share);
	apbc_apb = to_clk_apbc(clk_apb->hw);

	if ((hw->clk->enable_count + clk_share->enable_count) == 0)
		clk_apbc_set_reg(apbc_apb, APBC_APBCLK, 0);
}

struct clk_ops clk_apbc_pwm_ops = {
	.enable = clk_apbc_pwm_enable,
	.disable = clk_apbc_pwm_disable,
};

struct clk *mmp_clk_register_apbc(const char *name, const char *parent_name,
		void __iomem *base, unsigned int delay,
		unsigned int apbc_flags, spinlock_t *lock)
{
	struct clk_apbc *apbc;
	struct clk *clk;
	struct clk_init_data init;

	apbc = kzalloc(sizeof(*apbc), GFP_KERNEL);
	if (!apbc)
		return NULL;

	init.name = name;
	if (apbc_flags & APBC_PWM)
		init.ops = &clk_apbc_pwm_ops;
	else
		init.ops = &clk_apbc_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	apbc->base = base;
	apbc->delay = delay;
	apbc->flags = apbc_flags;
	apbc->lock = lock;
	apbc->hw.init = &init;

	clk = clk_register(NULL, &apbc->hw);
	if (IS_ERR(clk))
		kfree(apbc);

	return clk;
}
