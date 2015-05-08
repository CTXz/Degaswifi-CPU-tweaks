/*
 * mmp PLL clock operation source file
 *
 * Copyright (C) 2012 Marvell
 * Zhoujie Wu <zjwu@marvell.com>
 * Lu Cao <Lucao@gmail.com>
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
#include <linux/of.h>
#include <trace/events/power.h>
#include <linux/clk/mmpdcstat.h>
#include <linux/of.h>
#include "clk.h"

#define MMP_PERI_GATE_FLAG	BIT(31)
#define CLK_SET_BITS(set, clear)	{		\
	unsigned long tmp;				\
	tmp = __raw_readl(peri->reg_addr);		\
	tmp &= ~(clear);				\
	tmp |= (set);					\
	__raw_writel(tmp, peri->reg_addr);		\
}							\

static void __clk_fill_periph_tbl(struct clk_hw *hw,
	struct periph_clk_tbl *clk_tbl, unsigned int clk_tbl_size)
{
	unsigned int i = 0, idx;
	unsigned long prate = 0;
	const struct clk_mux_sel *sel;
	struct periph_clk_tbl *cop;
	struct clk *clk = hw->clk;
	struct clk_peri *peri = to_clk_peri(hw);
	u32 flags = peri->reg_info->flags;

	pr_info("************** clk_%s_tbl  **************\n", clk->name);

	for (i = 0; i < clk_tbl_size; i++) {
		list_for_each_entry(cop, &peri->clktbl_list, node)
			if (cop->clk_rate == clk_tbl[i].clk_rate)
				break;
		if (cop->clk_rate == clk_tbl[i].clk_rate)
			continue;
		for (idx = 0; idx < peri->params->inputs_size; idx++) {
			sel = &peri->params->inputs[idx];
			if (!strcmp(sel->parent_name, clk_tbl[i].parent_name)) {
				clk_tbl[i].parent = sel->parent;
				prate = clk_get_rate(clk_tbl[i].parent);
				if ((prate % clk_tbl[i].clk_rate) ||\
					!prate)
					break;
				clk_tbl[i].src_val = sel->value;
				if (flags & CLK_DIVIDER_ONE_BASED)
					clk_tbl[i].div_val =
						prate / clk_tbl[i].clk_rate;
				else
					clk_tbl[i].div_val =
						prate / clk_tbl[i].clk_rate - 1;
				list_add_tail(&clk_tbl[i].node,
					&peri->clktbl_list);
				pr_info("clk[%lu] src[%lu:%lu] div[%lu] w/rtc[%x]\n",
					clk_tbl[i].clk_rate, prate, \
					clk_tbl[i].src_val, \
					clk_tbl[i].div_val, \
					clk_tbl[i].rtcwtc);
				break;
			}
		}
	}
}

static void __clk_fill_periphtbl_xtc(struct periph_clk_tbl *tbl,
	unsigned int tbl_len, struct xpu_rtcwtc *xtc, unsigned int xtctbl_len)
{
	int i = 0;
	int j = 0;
	while (i < tbl_len) {
		if (tbl[i].clk_rate <= xtc[j].max_rate) {
			tbl[i].rtcwtc = xtc[j].rtcwtc;
			i++;
		} else if (j < xtctbl_len)
			j++;
		if (j == xtctbl_len)
			tbl[i].rtcwtc = xtc[j - 1].rtcwtc;
	}
}

/*
 * This help function can get the rate close to the required rate,
 * we'd better not use it for clock which need to dynamic change
 * as efficiency consideration.
 */
static unsigned long __clk_sel_mux_div(struct clk_hw *hw, unsigned long rate,
	unsigned int *mux, unsigned int *div, struct clk **best_parent)
{
	const struct clk_mux_sel *sel;
	struct clk *clk, *parent_selh = NULL, *parent_sell = NULL;
	unsigned long i, parent_rate, now, maxdiv;
	unsigned long bestl = 0, besth = ULONG_MAX;
	unsigned int bestdivl = 0, bestmuxl = 0, bestdivh = 0, bestmuxh = 0;
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned int idx;

	maxdiv = peri->reg_info->div_mask + 1;
	clk = peri->params->inputs[0].parent;
	if (rate < (clk_get_rate(clk) / maxdiv))
		rate = clk_get_rate(clk) / maxdiv;
	/*
	 * The maximum divider we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxdiv = min(ULONG_MAX / rate, maxdiv);
	for (idx = 0; idx < peri->params->inputs_size; idx++) {
		sel = &peri->params->inputs[idx];
		parent_rate = clk_get_rate(sel->parent);
		for (i = 1; i <= maxdiv; i++) {
			now = parent_rate / i;
			/* condition to select a best closest rate >= rate */
			if (now >= rate && now < besth) {
				bestdivh = i;
				besth = now;
				parent_selh = sel->parent;
				bestmuxh = sel->value;
			/* condition to select a best closest rate <= rate */
			} else if (now <= rate && now > bestl) {
				bestdivl = i;
				bestl = now;
				parent_sell = sel->parent;
				bestmuxl = sel->value;
			}
		}
	}

	now = ((besth - rate) <= (rate - bestl)) ? besth : bestl;
	if (now == besth) {
		*div = bestdivh;
		*mux = bestmuxh;
		*best_parent = parent_selh;
	} else {
		*div = bestdivl;
		*mux = bestmuxl;
		*best_parent = parent_sell;
	}
	BUG_ON(!(*div));
	BUG_ON(!(*best_parent));

	pr_debug("%s clk:%s mux:%u div:%u, %lu\n", __func__, \
		clk->name, *mux, *div, clk_get_rate(*best_parent));

	return clk_get_rate(*best_parent)/(*div);
}

static void __clk_wait_fc_done(struct clk_hw *hw)
{
	/* fc done should be asserted in several clk cycles */
	unsigned int i = 50;
	unsigned int fcreqmsk;
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_addr = peri->reg_addr;

	udelay(2);
	fcreqmsk = peri->reg_info->fcreq_mask << \
		peri->reg_info->fcreq_shift;
	while ((__raw_readl(reg_addr) & fcreqmsk) && i) {
		udelay(10);
		i--;
	}
	if (i == 0) {
		pr_info("CLK[%s] fc req failed![%p]= 0x%x, fc_req = 0x%x\n",
			hw->clk->name, reg_addr, __raw_readl(reg_addr),
			fcreqmsk);
		BUG_ON(1);
	}
}

static long __clk_set_mux_div(struct clk_hw *hw, struct clk *best_parent,
	unsigned int mux, unsigned int div)
{
	unsigned int muxmask, divmask;
	unsigned int muxval, divval;
	unsigned int regval, fcreqmsk;
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *old_parent = hw->clk->parent;
	void __iomem *reg_addr = peri->reg_addr;
	u32 flags = peri->reg_info->flags;
	unsigned long irqflags = 0;

	BUG_ON(!div);

	if (!(flags & CLK_DIVIDER_ONE_BASED))
		div = div - 1;	/* rate = parent_rate / (div_regval + 1) */

	muxval = mux & peri->reg_info->src_sel_mask;
	divval = div & peri->reg_info->div_mask;
	muxval = muxval << peri->reg_info->src_sel_shift;
	divval = divval << peri->reg_info->div_shift;

	muxmask = peri->reg_info->src_sel_mask << \
			peri->reg_info->src_sel_shift;
	divmask = peri->reg_info->div_mask << \
			peri->reg_info->div_shift;
	/*
	 * Migrate prepare state to new parent
	 * For smooth mux clock src switch, must make sure both clocks on
	 * or smooth mux can not finish clock switch.
	 */
	clk_prepare_enable(best_parent);
	if (hw->clk->prepare_count)
		clk_enable(hw->clk);
	else if (old_parent)
		clk_prepare_enable(old_parent);

	__clk_reparent(hw->clk, best_parent);

	if (peri->lock)
		spin_lock_irqsave(peri->lock, irqflags);
	/*
	 * mux and div are from the same reg, if clk is enabled,
	 * set mux, div and trigger(fcreqmsk) at the same time.
	 * If clock is disabled, we still set mux, div and fc_request here.
	 */
	regval = __raw_readl(reg_addr);
	regval &= ~(muxmask | divmask);
	regval |= (muxval | divval);


	if (peri->reg_info->fcreq_mask) {
		fcreqmsk = peri->reg_info->fcreq_mask << \
				peri->reg_info->fcreq_shift;
		regval |= fcreqmsk;
	}

	__raw_writel(regval, reg_addr);

	if (peri->reg_info->fcreq_mask)
		__clk_wait_fc_done(hw);

	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, irqflags);

	/*
	 * Finish the migration of prepare state and undo the changes done
	 * for preventing a race with clk_enable().
	 */
	if (hw->clk->prepare_count)
		clk_disable(hw->clk);
	else
		clk_disable_unprepare(best_parent);
	if (old_parent)
		clk_disable_unprepare(old_parent);

	pr_debug("\n%s clk:%s [%p] = [%x]\n", __func__, hw->clk->name,
		reg_addr, __raw_readl(reg_addr));

	return 0;
}

static void __clk_get_mux_div(struct clk_hw *hw,
		unsigned int *mux, unsigned int *div)
{
	unsigned int muxval, divval;
	unsigned int regval;
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_addr = peri->reg_addr;
	u32 flags = peri->reg_info->flags;

	regval = __raw_readl(reg_addr);
	muxval = (regval >> peri->reg_info->src_sel_shift) & \
			peri->reg_info->src_sel_mask;
	divval = (regval >> peri->reg_info->div_shift) & \
			peri->reg_info->div_mask;

	pr_debug("\n%s clk:%s [%p]val%x mux[%d] div[%d]\n", __func__,
		hw->clk->name, reg_addr,
		regval, muxval, divval+1);

	*mux = muxval;
	if (flags & CLK_DIVIDER_ONE_BASED)
		*div = divval;
	else
		*div = divval + 1;
}

static struct clk *__clk_mux_to_parent(struct clk_hw *hw, unsigned int mux)
{
	unsigned int idx;
	struct clk_peri *peri = to_clk_peri(hw);

	for (idx = 0; idx < peri->params->inputs_size; idx++) {
		if (peri->params->inputs[idx].value == mux)
			break;
	}
	BUG_ON(idx == peri->params->inputs_size);

	return peri->params->inputs[idx].parent;
}

static unsigned int __maybe_unused __clk_parent_to_mux
	(struct clk_hw *hw, struct clk *parent)
{
	unsigned int idx;
	struct clk_peri *peri = to_clk_peri(hw);

	for (idx = 0; idx < peri->params->inputs_size; idx++) {
		if (!strcmp(peri->params->inputs[idx].parent_name,
				parent->name))
			break;
	}
	BUG_ON(idx == peri->params->inputs_size);
	return peri->params->inputs[idx].value;
}

static unsigned long __clk_periph_get_rate(struct clk_hw *hw)
{
	struct clk *cur_parent;
	unsigned int mux, div = 1;

	__clk_get_mux_div(hw, &mux, &div);
	cur_parent = __clk_mux_to_parent(hw, mux);

	return clk_get_rate(cur_parent) / div;
}

/* for clock which has no table, we select best parent for it */
static int __clk_periph_set_rate(struct clk_hw *hw, unsigned long rate)
{
	unsigned long new_rate;
	unsigned int mux, div;
	struct clk *best_parent;

	new_rate = __clk_sel_mux_div(hw, rate, &mux, &div, &best_parent);
	if (rate != new_rate)
		pr_warning("clk[%s] required rate %lu, set as %lu\n", \
			hw->clk->name, rate, new_rate);
	__clk_set_mux_div(hw, best_parent, mux, div);
	return 0;
}

static struct periph_clk_tbl *__clk_periph_get_opptr
	(struct clk_peri *peri, unsigned long rate, unsigned int *index)
{
	struct periph_clk_tbl *cop;
	unsigned int idx = 0;

	if (unlikely(!peri || !peri->params->clktbl))
		return NULL;

	list_for_each_entry(cop, &peri->clktbl_list, node) {
		if ((cop->clk_rate >= rate) || \
			list_is_last(&cop->node, &peri->clktbl_list))
			break;
		idx++;
	}
	*index = idx;
	return cop;
}

static void periph_clk_init(struct clk_hw *hw)
{
	unsigned int idx = 0;
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *clk;
	struct periph_clk_tbl *cop;
	unsigned long op[MAX_OP_NUM];

	INIT_LIST_HEAD(&peri->clktbl_list);

	if (peri->params->inputs) {
		for (idx = 0; idx < peri->params->inputs_size; idx++) {
			clk = clk_get_sys(NULL,
				peri->params->inputs[idx].parent_name);
			if (!IS_ERR(clk))
				peri->params->inputs[idx].parent = clk;
			else
				pr_err("can't find parent clk %s\n",
					peri->params->inputs[idx].parent_name);
		}
	}

	/* Fill rtc/wtc if platform has */
	if (peri->params->rwtctbl && peri->params->clktbl)
		__clk_fill_periphtbl_xtc(peri->params->clktbl,
			peri->params->clktblsize,
			peri->params->rwtctbl, peri->params->rwtctblsize);
	/* Fill op tbl if platform has */
	if (peri->params->clktbl)
		__clk_fill_periph_tbl(hw, peri->params->clktbl,
				peri->params->clktblsize);

	if (peri->params->comclk_name) {
		peri->params->comclk = clk_get_sys(NULL,
				peri->params->comclk_name);
		if (IS_ERR(peri->params->comclk))
			WARN_ON(1);
	}

	/* support dc_stat? */
	if (peri->params->dcstat_support) {
		idx = 0;
		list_for_each_entry(cop, &peri->clktbl_list, node)
			op[idx++] = cop->clk_rate;
		clk_register_dcstat(hw->clk, op, idx);
	}
}

static int periph_clk_prepare(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);

	if (peri->params->flags & MMP_PERI_QOS_FEAT)
		pm_qos_update_request(&peri->qos_idle,
				peri->params->qos_idle_value);

	if (peri->params->comclk)
		clk_prepare_enable(peri->params->comclk);
	return 0;
}

static int periph_clk_enable(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long irqflags = 0;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, irqflags);
	CLK_SET_BITS(peri->reg_info->enable_val, 0);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, irqflags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_ON, 0);

	trace_clock_enable(hw->clk->name, 1, 0);
	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return 0;
}

static void periph_clk_unprepare(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);

	if (peri->params->comclk)
		clk_disable_unprepare(peri->params->comclk);

	if (peri->params->flags & MMP_PERI_QOS_FEAT)
		pm_qos_update_request(&peri->qos_idle,
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
}

static void periph_clk_disable(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long irqflags = 0;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, irqflags);
	CLK_SET_BITS(0, peri->reg_info->disable_val);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, irqflags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_OFF, 0);

	trace_clock_disable(hw->clk->name, 0, 0);
	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return;
}

static long periph_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct clk_peri *peri = to_clk_peri(hw);
	struct periph_clk_tbl *cop;
	unsigned int index;

	cop = __clk_periph_get_opptr(peri, rate, &index);
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
	unsigned int index = 0;
	struct clk *new_fparent;
	struct periph_clk_tbl *periph_pp = NULL;
	struct clk_peri *peri = to_clk_peri(hw);
	u32 flags = peri->reg_info->flags;

	old_rate = hw->clk->rate;
	if (rate == old_rate)
		return 0;

	periph_pp = __clk_periph_get_opptr(peri, rate, &index);
	if (likely(periph_pp)) {
		/* low -> high rtc/wtc adjust */
		if (peri->reg_info->reg_rtcwtc && \
			periph_pp->rtcwtc && (hw->clk->rate < rate))
			writel_relaxed(periph_pp->rtcwtc,
					peri->reg_info->reg_rtcwtc);

		/* set peripheral clk rate */
		new_fparent = periph_pp->parent;
		if (flags & CLK_DIVIDER_ONE_BASED)
			__clk_set_mux_div(hw, new_fparent, periph_pp->src_val,
					 periph_pp->div_val);
		else
			__clk_set_mux_div(hw, new_fparent, periph_pp->src_val,
					 (periph_pp->div_val + 1));

		/* set combinded clk rate here */
		if (peri->params->comclk && periph_pp->comclk_rate)
			clk_set_rate(peri->params->comclk,
					periph_pp->comclk_rate);

		/* high->low rtc/wtc adjust */
		if (peri->reg_info->reg_rtcwtc && \
			periph_pp->rtcwtc && (hw->clk->rate > rate))
			writel_relaxed(periph_pp->rtcwtc,
					peri->reg_info->reg_rtcwtc);
		/* duty cycle stat here */
		if (peri->params->dcstat_support)
			clk_dcstat_event(hw->clk, CLK_RATE_CHANGE, index);
	} else {
		/* for clock doesn't has table,
		search all possible clk rate for it */
		__clk_periph_set_rate(hw, rate);
	}

	trace_clock_set_rate(hw->clk->name, rate, 0);
	pr_debug("%s %s rate %lu->%lu [%p]=%x\n", __func__, hw->clk->name,
		hw->clk->rate, rate, peri->reg_addr,
		__raw_readl(peri->reg_addr));
	return 0;
}

static u8 peri_get_parent(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_addr = peri->reg_addr;
	int idx = 0;
	u32 val = __raw_readl(reg_addr);
	val = (val >> peri->reg_info->src_sel_shift) &
			peri->reg_info->src_sel_mask;

	for (idx = 0; idx < peri->params->inputs_size; idx++) {
		if (peri->params->inputs[idx].value == val)
			break;
	}

	BUG_ON(idx == peri->params->inputs_size);

	return idx;
}

struct clk_ops peri_clk_ops = {
	.init = periph_clk_init,
	.prepare = periph_clk_prepare,
	.unprepare = periph_clk_unprepare,
	.enable = periph_clk_enable,
	.disable = periph_clk_disable,
	.round_rate = periph_clk_round_rate,
	.set_rate = periph_clk_setrate,
	.recalc_rate = periph_clk_recalc_rate,
	.get_parent = peri_get_parent,
};

static struct clk_ops peri_gate_ops = {
	.enable = periph_clk_enable,
	.disable = periph_clk_disable,
};

static int eden_apmu_is_ax(void)
{
	struct device_node *np = of_find_node_by_name(NULL, "eden_apmu_ver");
	const char *str = NULL;

	if (np && !of_property_read_string(np, "version", &str)
			&& !strcmp(str, "ax"))
		return 1;

	return 0;
}

/* block/unblock memory controller port */
#define APMU_GC_CLK_RES_CTRL	0xcc
#define APMU_GC_CLK_RES_CTRL2	0x27c
#define APMU_VPU_CLK_RES_CTRL	0xa4
#define DMCU_VIRT_BASE		(void __force __iomem *)(0xfe500000)
#define MCU_REG(x) (DMCU_VIRT_BASE + (x))
#define MC_CTRL_0	MCU_REG(0x44)
#define USER_CMD_0	MCU_REG(0x20)
#define MC_STATUS	MCU_REG(0x4)

#define MC_GC_PORT	(1 << 12)
#define MC_VPU_PORT	(1 << 15)

static int __mcu_port_block(unsigned int port_msk)
{
	unsigned int regval;

	regval = __raw_readl(MC_CTRL_0);
	regval |= port_msk;
	regval &= ~((1 << 0) | (1 << 27));
	__raw_writel(regval, MC_CTRL_0);

	/* drain WCB buffer */
	do {
		__raw_writel(0x2, USER_CMD_0);
	} while (!(__raw_readl(MC_STATUS) & 0x8));

	return 0;
}

static void __mcu_port_unblock(unsigned int port_msk)
{
	unsigned int regval;

	regval = __raw_readl(MC_CTRL_0);
	regval &= ~(port_msk | (1 << 0) | (1 << 27));
	__raw_writel(regval, MC_CTRL_0);
}

static DEFINE_SPINLOCK(gc3d_reg_lock);
void get_gc3d_reg_lock(unsigned int lock, unsigned long *flags)
{
	if (lock)
		spin_lock_irqsave(&gc3d_reg_lock, *flags);
	else
		spin_unlock_irqrestore(&gc3d_reg_lock, *flags);
}
EXPORT_SYMBOL(get_gc3d_reg_lock);

static DEFINE_SPINLOCK(gc2d_reg_lock);
void get_gc2d_reg_lock(unsigned int lock, unsigned long *flags)
{
	if (lock)
		spin_lock_irqsave(&gc2d_reg_lock, *flags);
	else
		spin_unlock_irqrestore(&gc2d_reg_lock, *flags);
}
EXPORT_SYMBOL(get_gc2d_reg_lock);

static void __get_gc_reg_lock(struct clk_hw *hw, unsigned int lock,
	unsigned long *flags)
{
	struct clk_peri *peri = to_clk_peri(hw);
	u32 reg_offset = peri->reg_info->reg_offset;

	unsigned int is_gc3d = (reg_offset == APMU_GC_CLK_RES_CTRL) ? 1 : 0;
	unsigned int is_gc2d = (reg_offset == APMU_GC_CLK_RES_CTRL2) ? 1 : 0;

	if (lock) {
		if (is_gc3d)
			spin_lock_irqsave(&gc3d_reg_lock, *flags);
		else if (is_gc2d)
			spin_lock_irqsave(&gc2d_reg_lock, *flags);
	} else {
		if (is_gc3d)
			spin_unlock_irqrestore(&gc3d_reg_lock, *flags);
		else if (is_gc2d)
			spin_unlock_irqrestore(&gc2d_reg_lock, *flags);
	}
}

static int eden_gc_vpu_clk_prepare_zx(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *clk_depend;
	int i;

	/* enable dependency clk */
	for (i = 0; i < peri->num_dependence; i++) {
		clk_depend = clk_get(NULL, peri->dependence[i]);
		clk_prepare_enable(clk_depend);
	}

	return 0;
}

static int eden_gc_vpu_clk_prepare_ax(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);

	if (peri->params->comclk)
		clk_prepare_enable(peri->params->comclk);

	return 0;
}

static int eden_gc_vpu_clk_prepare(struct clk_hw *hw)
{
	if (eden_apmu_is_ax())
		return eden_gc_vpu_clk_prepare_ax(hw);
	else
		return eden_gc_vpu_clk_prepare_zx(hw);
}

static void eden_gc_vpu_clk_unprepare_zx(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *clk_depend;
	int i;

	/* disable dependency clk */
	for (i = 0; i < peri->num_dependence; i++) {
		clk_depend = clk_get(NULL, peri->dependence[i]);
		clk_disable_unprepare(clk_depend);
	}

	return;
}

static void eden_gc_vpu_clk_unprepare_ax(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);

	if (peri->params->comclk)
		clk_disable_unprepare(peri->params->comclk);

	return;
}

static void eden_gc_vpu_clk_unprepare(struct clk_hw *hw)
{
	if (eden_apmu_is_ax())
		eden_gc_vpu_clk_unprepare_ax(hw);
	else
		eden_gc_vpu_clk_unprepare_zx(hw);
}

static int eden_gc_vpu_clk_enable_zx(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long flags = 0;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, flags);

	CLK_SET_BITS(peri->reg_info->enable_val, 0);

	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, flags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_ON, 0);

	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return 0;
}

static int eden_gc_vpu_clk_enable_ax(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_addr = peri->reg_addr;
	unsigned long flags = 0;
	unsigned int regval;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, flags);
	regval = __raw_readl(reg_addr);
	regval |= 1 << peri->reg_info->div_shift;
	__raw_writel(regval, reg_addr);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, flags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_ON, 0);

	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return 0;
}

static int eden_gc_vpu_clk_enable(struct clk_hw *hw)
{
	if (eden_apmu_is_ax())
		return eden_gc_vpu_clk_enable_ax(hw);
	else
		return eden_gc_vpu_clk_enable_zx(hw);
}

static void eden_gc_vpu_clk_disable_zx(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned long flags = 0, vpu_flags = 0;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, flags);

	if (peri->clk_data)
		spin_lock_irqsave(peri->clk_data, vpu_flags);

	CLK_SET_BITS(0, peri->reg_info->disable_val);

	if (peri->clk_data)
		spin_unlock_irqrestore(peri->clk_data, vpu_flags);

	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, flags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_OFF, 0);

	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return;
}

static void eden_gc_vpu_clk_disable_ax(struct clk_hw *hw)
{
	struct clk_peri *peri = to_clk_peri(hw);
	void __iomem *reg_addr = peri->reg_addr;
	unsigned int regval;
	unsigned long flags = 0;
	unsigned int fcreqmsk = peri->reg_info->div_mask <<
		peri->reg_info->div_shift;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, flags);
	regval = __raw_readl(reg_addr);
	regval &= ~fcreqmsk;
	__raw_writel(regval, reg_addr);
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, flags);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_STATE_OFF, 0);

	pr_debug("%s %s %x\n", __func__, hw->clk->name,
			__raw_readl(peri->reg_addr));
	return;
}

static void eden_gc_vpu_clk_disable(struct clk_hw *hw)
{
	if (eden_apmu_is_ax())
		eden_gc_vpu_clk_disable_ax(hw);
	else
		eden_gc_vpu_clk_disable_zx(hw);
}

static void __do_gc_vpu_dfc(struct clk_hw *hw, unsigned int new_val,
		unsigned int en_mask)
{
	struct clk_peri *peri = to_clk_peri(hw);
	unsigned int regval = 0, clk_is_on = 0, fcreqmsk;
	void __iomem *fc_reg = peri->fc_reg;
	unsigned long flags = 0, gc_flags = 0, vpu_flags = 0;
	unsigned int blk_port;
	u32 reg_offset = peri->reg_info->reg_offset;
	unsigned int is_vpu = (reg_offset == APMU_VPU_CLK_RES_CTRL) ? 1 : 0;

	if (peri->lock)
		spin_lock_irqsave(peri->lock, flags);

	if (eden_apmu_is_ax()) {
		/*
		 * A 3-step progaming is recommended for changing clock freq.
		 * 1) set FC_EN field to 0 to block frequency changes.
		 * 2) configure clk source and clk divider.
		 * 3) set FC_EN field to 1 to allow frequency changes
		 */
		fcreqmsk = peri->reg_info->fcreq_mask <<
			peri->reg_info->fcreq_shift;
		regval = __raw_readl(fc_reg);
		regval &= ~fcreqmsk;
		__raw_writel(regval, fc_reg);

		/* change frequency */
		__raw_writel(new_val, peri->reg_addr);

		regval = __raw_readl(fc_reg);
		regval |= fcreqmsk;
		__raw_writel(regval, fc_reg);

	} else {
		blk_port = is_vpu ? MC_VPU_PORT : MC_GC_PORT;
		if (!is_vpu)
			__get_gc_reg_lock(hw, 1, &gc_flags);

		if (peri->clk_data)
			spin_lock_irqsave(peri->clk_data, vpu_flags);

		__mcu_port_block(blk_port);

		clk_is_on = __raw_readl(peri->reg_addr) & en_mask;
		if (clk_is_on) {
			new_val &= ~en_mask;
			/* disable clk before change frequency */
			CLK_SET_BITS(0, en_mask);
			/* change frequency */
			__raw_writel(new_val, peri->reg_addr);
			/* restore enable bit */
			CLK_SET_BITS(en_mask, 0);
		} else
			/* change frequency */
			__raw_writel(new_val, peri->reg_addr);
		__mcu_port_unblock(blk_port);

		if (peri->clk_data)
			spin_unlock_irqrestore(peri->clk_data, vpu_flags);

		if (!is_vpu)
			__get_gc_reg_lock(hw, 0, &gc_flags);

	}
	if (peri->lock)
		spin_unlock_irqrestore(peri->lock, flags);

}

static int gc_vpu_dfc(struct clk_hw *hw, struct clk *best_parent,
		unsigned int mux, unsigned int div, unsigned int en_mask)
{
	unsigned int muxmask, divmask;
	unsigned int muxval, divval;
	unsigned int regval;
	struct clk_peri *peri = to_clk_peri(hw);
	struct clk *clk = hw->clk;
	void __iomem *reg_addr = peri->reg_addr;

	BUG_ON(!div);

	muxval = mux & peri->reg_info->src_sel_mask;
	divval = div & peri->reg_info->div_mask;
	muxval = muxval << peri->reg_info->src_sel_shift;
	divval = divval << peri->reg_info->div_shift;

	muxmask = peri->reg_info->src_sel_mask <<
			peri->reg_info->src_sel_shift;
	divmask = peri->reg_info->div_mask <<
			peri->reg_info->div_shift;

	regval = __raw_readl(reg_addr);
	regval &= ~(muxmask | divmask);
	regval |= (muxval | divval);

	if (clk->parent == best_parent) {
		__do_gc_vpu_dfc(hw, regval, en_mask);

		return 0;
	}

	/* enable new parent clock and
	 * disable current parent clock
	 */
	if (__clk_is_enabled(hw->clk)) {
		/* enable new parent clock */
		clk_prepare_enable(best_parent);

		__do_gc_vpu_dfc(hw, regval, en_mask);

		/* disable current parent clock */
		if (clk->parent)
			clk_disable_unprepare(clk->parent);
	} else
		__do_gc_vpu_dfc(hw, regval, en_mask);

	__clk_reparent(clk, best_parent);

	return 0;
}

static int eden_gc_vpu_clk_setrate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	unsigned long old_rate;
	unsigned int index = 0;
	struct periph_clk_tbl *periph_pp = NULL;
	struct clk_peri *peri = to_clk_peri(hw);

	old_rate = hw->clk->rate;
	pr_debug("%s %s rate %lu->%lu [%p]=%x\n", __func__, hw->clk->name,
		old_rate, rate, peri->reg_addr,
		__raw_readl(peri->reg_addr));

	if (rate == old_rate)
		return 0;

	periph_pp = __clk_periph_get_opptr(peri, rate, &index);


	gc_vpu_dfc(hw, periph_pp->parent,
			periph_pp->src_val, periph_pp->div_val,
			peri->reg_info->enable_val);


	if (peri->params->comclk && periph_pp->comclk_rate)
		clk_set_rate(peri->params->comclk,
				periph_pp->comclk_rate);

	if (peri->params->dcstat_support)
		clk_dcstat_event(hw->clk, CLK_RATE_CHANGE, index);

	/* TODO: fabric update */
	return 0;
}

/* clk ops for gc and vpu of eden */
struct clk_ops eden_gc_vpu_clk_ops = {
	.init = periph_clk_init,
	.prepare = eden_gc_vpu_clk_prepare,
	.unprepare = eden_gc_vpu_clk_unprepare,
	.enable = eden_gc_vpu_clk_enable,
	.disable = eden_gc_vpu_clk_disable,
	.round_rate = periph_clk_round_rate,
	.set_rate = eden_gc_vpu_clk_setrate,
	.recalc_rate = periph_clk_recalc_rate,
	.get_parent = peri_get_parent,
};

#ifdef CONFIG_OF
static const struct of_device_id peri_clk_match[] = {
	{.compatible = "marvell,mmp-peri-clock"},
	{},
};
#endif

struct clk *mmp_clk_register_peri(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, void __iomem *reg_addr,
		spinlock_t *lock, struct peri_params *params,
		struct peri_reg_info *reg_info)
{
	struct clk_peri *peri;
	struct clk *clk;
	struct clk_init_data init;
	struct device_node *np, *npp;
	unsigned int proplen;
	const __be32 *prop;
	peri = kzalloc(sizeof(*peri), GFP_KERNEL);
	if (!peri)
		return NULL;

	init.name = name;
	init.ops = (params->flags & MMP_PERI_GATE_FLAG) ?
			&peri_gate_ops : &peri_clk_ops;
	init.flags = flags | CLK_GET_RATE_NOCACHE;
	init.parent_names = parent_name;
	init.num_parents = num_parents;

	peri->lock = lock;
	peri->params = params;
	peri->reg_info = reg_info;
	peri->hw.init = &init;
	peri->reg_addr = reg_addr;
#ifdef CONFIG_OF
	np = of_find_matching_node(NULL, peri_clk_match);
	if (np) {
		for_each_child_of_node(np, npp) {
			if (!strcmp(npp->name, name)) {
				prop = of_get_property(npp, "lpm-qos",
							&proplen);
				if (prop) {
					params->flags |= MMP_PERI_QOS_FEAT;
					params->qos_idle_value =
							be32_to_cpup(prop);
					peri->qos_idle.name =
						kstrdup(name, GFP_KERNEL);
					pm_qos_add_request(&peri->qos_idle,
						PM_QOS_CPUIDLE_BLOCK,
						PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
					break;
				}
			} else
				continue;
		}
	}
#else
	if (params->flags & MMP_PERI_QOS_FEAT) {
		peri->qos_idle.name = kstrdup(name, GFP_KERNEL);
		pm_qos_add_request(&peri->qos_idle, PM_QOS_CPUIDLE_BLOCK,
			PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
	}
#endif
	clk = clk_register(NULL, &peri->hw);
	if (IS_ERR(clk))
		kfree(peri);
	return clk;
}

struct clk *mmp_clk_register_peri_gate(const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg_addr, spinlock_t *lock,
		struct peri_params *params,
		struct peri_reg_info *reg_info)
{
	const char **parent;
	u8 num_parents;
	parent = (parent_name ? &parent_name : NULL);
	num_parents = (parent_name ? 1 : 0);
	params->flags |= MMP_PERI_GATE_FLAG;
	return mmp_clk_register_peri(name, parent, num_parents, flags,
				reg_addr, lock, params, reg_info);
}

struct clk *eden_clk_register_gc_vpu(const char *name, const char **parent_name,
		u8 num_parents, unsigned long flags, void __iomem *reg_addr,
		void __iomem *fc_reg, spinlock_t *lock,
		struct peri_params *params, struct peri_reg_info *reg_info,
		const char **clk_depend, u8 num_depend)
{
	struct clk_peri *peri;
	struct clk *clk;
	struct clk_init_data init;
	peri = kzalloc(sizeof(*peri), GFP_KERNEL);
	if (!peri)
		return NULL;

	init.name = name;
	init.ops = &eden_gc_vpu_clk_ops;
	init.flags = flags;
	init.parent_names = parent_name;
	init.num_parents = num_parents;

	peri->lock = lock;
	peri->params = params;
	peri->reg_info = reg_info;
	peri->hw.init = &init;
	peri->reg_addr = reg_addr;
	peri->fc_reg = fc_reg;
	peri->dependence = clk_depend;
	peri->num_dependence = num_depend;
	clk = clk_register(NULL, &peri->hw);
	if (IS_ERR(clk))
		kfree(peri);
	return clk;
}
