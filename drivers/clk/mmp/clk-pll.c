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
#include <trace/events/power.h>

#include "clk.h"
#define pll_readl(reg) readl_relaxed(reg)
#define pll_readl_cr(p) pll_readl(p->params->cr_reg)
#define pll_readl_pll_swcr(p) pll_readl(p->params->pll_swcr)

#define pll_writel(val, reg) writel_relaxed(val, reg)
#define pll_writel_cr(val, p) pll_writel(val, p->params->cr_reg)
#define pll_writel_pll_swcr(val, p) pll_writel(val, p->params->pll_swcr)


static struct intpi_range pll40nm_intpi_tbl[] = {
	{2400, 2500, 6},
	{2100, 2400, 5},
	{1800, 2100, 4},
	{1500, 1800, 3},
	{1200, 1500, 2},
};

static struct intpi_range pll28nm_intpi_tbl[] = {
	{2500, 3000, 8},
	{2000, 2500, 6},
	{1500, 2000, 5},
};

static int __pll_is_enabled(struct clk_hw *hw)
{
	struct clk_vco *vco = to_clk_vco(hw);
	u32 val;
	struct mmp_vco_params *params = vco->params;

	if (vco->flags & MMP_PLL_LOCK_SETTING) {
		val = pll_readl_cr(vco);
		/* ctrl = 0(hw enable) or ctrl = 1&&en = 1(sw enable) */
		/* ctrl = 1&&en = 0(sw disable) */
		if ((val & params->ctrl_bit) && (!(val & params->enable_bit)))
			return 0;
		else
			return 1;
	} else {
		val = pll_readl_cr(vco);
		/*
		 * PLL3CR[19:18] = 0x1, 0x2, 0x3 means PLL3 is enabled.
		 * PLL3CR[19:18] = 0x0 means PLL3 is disabled
		 */
		if ((val & params->ctrl_bit) | (val & params->enable_bit))
			return 1;
	}
	return 0;
}

/* frequency unit Mhz, return pll vco freq */
static unsigned int __get_vco_freq(struct clk_hw *hw)
{
	u32 val;
	unsigned int pllx_vco, pllxrefd, pllxfbd;
	struct clk_vco *vco = to_clk_vco(hw);
	struct mmp_vco_params *params = vco->params;

	struct pll_offset *pll_offset = params->pll_offset;
	/* return 0 if pll2 is disabled(ctrl = 1, en = 0) */
	if (!__pll_is_enabled(hw))
		return 0;

	val = pll_readl_cr(vco);
	pllxrefd = (val >> pll_offset->refd_shift) &
			MASK(pll_offset->refd_width);
	pllxfbd = (val >> pll_offset->fbd_shift) &
			MASK(pll_offset->fbd_width);
	if (!(vco->flags & MMP_PLL_28NM))
		BUG_ON(pllxrefd == 1);
	if (pllxrefd == 0)
		pllxrefd = 1;
	if (vco->flags & MMP_PLL_28NM)
		pllx_vco = DIV_ROUND_UP(4 * 26 * pllxfbd, pllxrefd);
	else
		pllx_vco = DIV_ROUND_UP(26 * pllxfbd, pllxrefd);
	hw->clk->rate = pllx_vco * MHZ;
	return pllx_vco * MHZ;
}

static unsigned int __clk_pll_vco2intpi(struct clk_vco *vco)
{
	unsigned int intpi, vco_rate, i, itp_tbl_size;
	struct ssc_params *ssc_params;
	struct intpi_range *itp_tbl;
	vco_rate =  __get_vco_freq(&vco->hw) / MHZ_TO_HZ;
	ssc_params = vco->params->ssc_params;
	intpi = 6;

	if (vco->flags & MMP_PLL_28NM) {
		itp_tbl = pll28nm_intpi_tbl;
		itp_tbl_size = ARRAY_SIZE(pll28nm_intpi_tbl);
	} else {
		itp_tbl = pll40nm_intpi_tbl;
		itp_tbl_size = ARRAY_SIZE(pll40nm_intpi_tbl);
	}

	for (i = 0; i < itp_tbl_size; i++) {
		if ((vco_rate >= itp_tbl[i].vco_min) &&
				(vco_rate <=  itp_tbl[i].vco_max)) {
			intpi = itp_tbl[i].value;
			break;
		}
	}
	if (i == itp_tbl_size)
		BUG_ON("Unsupported vco frequency for INTPI!\n");

	return intpi;
}

/* Real amplitude percentage = amplitude / base */
static void __clk_pll40nm_get_divrng(enum ssc_mode mode, unsigned long rate,
				 unsigned int amplitude, unsigned int base,
				 unsigned long vco, unsigned int *div,
				 unsigned int *rng)
{
	if (amplitude > (25 * base / 1000))
		BUG_ON("Amplitude can't exceed 2.5\%\n");
	switch (mode) {
	case CENTER_SPREAD:
		*div = (vco / rate) >> 2;
		break;
	case DOWN_SPREAD:
		*div = (vco / rate) >> 1;
		break;
	default:
		pr_err("Unsupported SSC MODE!\n");
		return;
	}
	if (*div == 0)
		*div = 1;
	*rng = (1 << 29) / (*div * base / amplitude);
}

static void __clk_pll28nm_get_divrng(enum ssc_mode mode, unsigned long rate,
				 unsigned int amplitude, unsigned int base,
				 unsigned long vco, unsigned int *div,
				 unsigned int *rng)
{
	unsigned int vco_avg;
	if (amplitude > (50 * base / 1000))
		BUG_ON("Amplitude can't exceed 5\%\n");
	switch (mode) {
	case CENTER_SPREAD:
		/* VCO_CLK_AVG = REFCLK * (4*N /M) */
		vco_avg = vco;
		/* SSC_FREQ_DIV = VCO_CLK _AVG /
			(4*4 * Desired_Modulation_Frequency) */
		*div = (vco_avg / rate) >> 4;
		break;
	case DOWN_SPREAD:
		/* VCO_CLK_AVG= REFCLK * (4*N /M)*(1-Desired_SSC_Amplitude/2) */
		vco_avg = vco - vco * amplitude / 2 / base;
		/* SSC_FREQ_DIV = VCO_CLK_AVG /
			(4*2 * Desired_Modulation_Frequency) */
		*div = (vco_avg / rate) >> 3;
		break;
	default:
		pr_err("Unsupported SSC MODE!\n");
		return;
	}
	if (*div == 0)
		*div = 1;
	/* SSC_RNGE = Desired_SSC_Amplitude / (SSC_FREQ_DIV * 2^-28) */
	*rng = (1 << 28) / (*div * base / amplitude);
}

static inline void config_pi_module(struct clk_vco *vco, enum pllip pllip)
{
	u32 pll_pi_conf;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;
	unsigned int intpi = __clk_pll_vco2intpi(vco);
	des = ssc_params->des;

	pll_pi_conf = __raw_readl(des->pi_cnf_reg->regoff);
	/* 1) set intpi */
	pll_pi_conf &= ~(MASK(des->pi_cnf_reg->intpiwidth) <<
			des->pi_cnf_reg->intpishift);
	pll_pi_conf |= intpi << des->pi_cnf_reg->intpishift;

	if (pllip == PLL_28nm) {
		/* 2) set intpr */
		pll_pi_conf &= ~(MASK(des->pi_cnf_reg->intprwidth) <<
				des->pi_cnf_reg->intprshift);
		pll_pi_conf |= ssc_params->intpr << des->pi_cnf_reg->intprshift;
	} else {
		/* 2) clear sel_vco_diff & sel_vco_se */
		pll_pi_conf &= ~(BIT(des->pi_cnf_reg->diffclkselbit));
		pll_pi_conf &= ~(BIT(des->pi_cnf_reg->seclkselbit));
	}
	__raw_writel(pll_pi_conf, des->pi_cnf_reg->regoff);
}

static inline void enable_pi_module(struct clk_vco *vco, enum pllip pllip)
{
	u32 pll_pi_ctrl;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	des = ssc_params->des;

	pll_pi_ctrl = __raw_readl(des->pi_ctrl_reg->regoff);
	/* 1) set ssc_clk_en clk_det_en and pi_en */
	pll_pi_ctrl |= BIT(des->pi_ctrl_reg->sscclkenbit);
	pll_pi_ctrl |= BIT(des->pi_ctrl_reg->clkdetenbit);
	pll_pi_ctrl |= BIT(des->pi_ctrl_reg->enbit);

	if (pllip == PLL_28nm) {
		/* 2) clear pi_reset bit */
		pll_pi_ctrl &= ~(BIT(des->pi_ctrl_reg->rstbit));
	}
	__raw_writel(pll_pi_ctrl, des->pi_ctrl_reg->regoff);
}

static inline void config_ssc_module(struct clk_vco *vco, enum pllip pllip,
			enum ssc_mode mode,
			unsigned int div, unsigned int rng)
{
	u32 pll_ssc_conf, pll_ssc_mode;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	des = ssc_params->des;

	/* ssc conf register */
	pll_ssc_conf = __raw_readl(des->ssc_conf_reg->regoff);
	/* 1) set div */
	pll_ssc_conf &= ~(MASK(des->ssc_conf_reg->divwidth) <<
				des->ssc_conf_reg->divshift);
	pll_ssc_conf |= div << des->ssc_conf_reg->divshift;

	/* 2) set range */
	pll_ssc_conf &= ~(MASK(des->ssc_conf_reg->rngwidth) <<
				des->ssc_conf_reg->rngshift);
	pll_ssc_conf |= rng << des->ssc_conf_reg->rngshift;
	__raw_writel(pll_ssc_conf, des->ssc_conf_reg->regoff);

	/* ssc mode register */
	/* 1) set ssc mode */
	pll_ssc_mode = __raw_readl(des->ssc_modsel_reg->regoff);
	pll_ssc_mode &= ~(BIT(des->ssc_modsel_reg->modselbit));
	pll_ssc_mode |= mode << des->ssc_modsel_reg->modselbit;
	__raw_writel(pll_ssc_mode, des->ssc_modsel_reg->regoff);
}

static inline void enable_ssc_module(struct clk_vco *vco, enum pllip pllip)
{
	u32 pi_loop_mode, pll_ssc_ctrl;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	des = ssc_params->des;

	/* 1) set ssc_en & ssc_reset_ext */
	pll_ssc_ctrl = __raw_readl(des->ssc_ctrl_reg->regoff);
	pll_ssc_ctrl |= BIT(des->ssc_ctrl_reg->enbit);
	pll_ssc_ctrl |= BIT(des->ssc_ctrl_reg->rstbit);
	__raw_writel(pll_ssc_ctrl, des->ssc_ctrl_reg->regoff);
	udelay(1);
	if (pllip == PLL_28nm) {
		/* 2) clear ssc_reset */
		pll_ssc_ctrl = __raw_readl(des->ssc_ctrl_reg->regoff);
		pll_ssc_ctrl &= ~(BIT(des->ssc_ctrl_reg->rstbit));
		__raw_writel(pll_ssc_ctrl, des->ssc_ctrl_reg->regoff);
		udelay(1);
		/* 3) set pi_loop_mode */
		pi_loop_mode = __raw_readl(des->pi_loopmode_reg->regoff);
		pi_loop_mode |= BIT(des->pi_loopmode_reg->modeselbit);
		__raw_writel(pi_loop_mode, des->pi_loopmode_reg->regoff);
	} else {
		/* 2) clear ssc_reset_ext */
		pll_ssc_ctrl &= ~(BIT(des->ssc_ctrl_reg->rstbit));
		__raw_writel(pll_ssc_ctrl, des->ssc_ctrl_reg->regoff);
	}
}

static void __enable_ssc(struct clk_vco *vco, enum pllip pllip,
			   enum ssc_mode mode,
			   unsigned int div, unsigned int rng)
{
	config_pi_module(vco, pllip);
	enable_pi_module(vco, pllip);
	config_ssc_module(vco, pllip, mode, div, rng);
	enable_ssc_module(vco, pllip);
}

static void enable_pll_ssc(struct clk_hw *hw)
{
	struct clk_vco *vco = to_clk_vco(hw);
	unsigned int div = 0, rng = 0;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	if (vco->flags & MMP_PLL_28NM) {
		__clk_pll28nm_get_divrng(ssc_params->ssc_mode,
				ssc_params->desired_mod_freq,
				ssc_params->amplitude,
				ssc_params->base,
				hw->clk->rate, &div, &rng);
		__enable_ssc(vco, PLL_28nm, ssc_params->ssc_mode,
				div, rng);
	} else {
		__clk_pll40nm_get_divrng(ssc_params->ssc_mode,
				ssc_params->desired_mod_freq,
				ssc_params->amplitude,
				ssc_params->base,
				hw->clk->rate, &div, &rng);
		__enable_ssc(vco, PLL_40nm, ssc_params->ssc_mode,
				div, rng);
	}

}
static inline void disable_pi_module(struct clk_vco *vco, enum pllip pllip)
{
	u32 pll_pi_ctrl, pll_pi_conf, pi_loop_mode;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	des = ssc_params->des;

	pll_pi_ctrl = __raw_readl(des->pi_ctrl_reg->regoff);
	/* 1) clear ssc_clk_en clk_det_en and pi_en */
	pll_pi_ctrl &= ~(BIT(des->pi_ctrl_reg->sscclkenbit));
	pll_pi_ctrl &= ~(BIT(des->pi_ctrl_reg->clkdetenbit));
	pll_pi_ctrl &= ~(BIT(des->pi_ctrl_reg->enbit));
	__raw_writel(pll_pi_ctrl, des->pi_ctrl_reg->regoff);

	if (pllip == PLL_40nm) {
		/* 2) set sel_vco_diff & sel_vco_se */
		pll_pi_conf = __raw_readl(des->pi_cnf_reg->regoff);
		pll_pi_conf |= BIT(des->pi_cnf_reg->diffclkselbit);
		pll_pi_conf |= BIT(des->pi_cnf_reg->seclkselbit);
		__raw_writel(pll_pi_conf, des->pi_cnf_reg->regoff);
	} else {
		/* 2) clear pi loop mode */
		pi_loop_mode = __raw_readl(des->pi_loopmode_reg->regoff);
		pi_loop_mode &= ~(BIT(des->pi_loopmode_reg->modeselbit));
		__raw_writel(pi_loop_mode, des->pi_loopmode_reg->regoff);
	}
}

static inline void disable_ssc_module(struct clk_vco *vco, enum pllip pllip)
{
	u32 pll_ssc_ctrl;
	struct pi_ssc_reg_des *des;
	struct mmp_vco_params *params = vco->params;
	struct ssc_params *ssc_params = params->ssc_params;

	des = ssc_params->des;

	/* 1) clear ssc_en */
	pll_ssc_ctrl = __raw_readl(des->ssc_ctrl_reg->regoff);
	pll_ssc_ctrl &= ~(BIT(des->ssc_ctrl_reg->enbit));
	__raw_writel(pll_ssc_ctrl, des->ssc_ctrl_reg->regoff);
}

static void __disable_ssc(struct clk_vco *vco, enum pllip pllip)
{
	disable_pi_module(vco, pllip);
	disable_ssc_module(vco, pllip);
}

static void disable_pll_ssc(struct clk_vco *vco)
{
	if (vco->flags & MMP_PLL_28NM)
		__disable_ssc(vco, PLL_28nm);
	else
		__disable_ssc(vco, PLL_40nm);
}

/* convert post div reg setting to divider val */
static unsigned int __pll_div_hwval2div(struct clk_pll *pll,
		unsigned int hw_val)
{
	struct mmp_pll_params	*params;
	struct div_map		*div_map;
	int i, size;
	params = pll->params;
	size = params->div_map_size;
	div_map = params->div_map;
	for (i = 0; i < size; i++) {
		if (hw_val == div_map[i].hw_val)
			return div_map[i].div;
	}
	BUG_ON(i == size);
	return 0;
}

/* PLL range 1.2G~2.5G, vco_vrng = kvco */
static void __clk_pll_rate2rng(struct clk_vco *vco, unsigned long rate,
		unsigned int *kvco, unsigned int *vco_rng)
{
	struct mmp_vco_params	*params;
	struct kvco_range	kvco_rng;
	int i, size;
	params = vco->params;
	size = params->kvco_rng_size;

	for (i = 0, kvco_rng = params->kvco_rng_table[i];
		i < size; i++, kvco_rng = params->kvco_rng_table[i]) {
		if (rate >= kvco_rng.vco_min && rate <=  kvco_rng.vco_max) {
			*kvco = kvco_rng.kvco;
			*vco_rng = kvco_rng.vrng;
			return;
		}
	}
	WARN_ON(i == size);
	*kvco = params->kvco_rng_table[0].kvco;
	*vco_rng = params->kvco_rng_table[0].vrng;
	return;
}

static unsigned int __clk_pll_calc_div(struct clk_pll *pll, unsigned long rate,
	unsigned long parent_rate, unsigned int *div)
{
	struct mmp_pll_params	*params;
	struct div_map		*div_map;
	int i, size;
	params = pll->params;
	size = params->div_map_size;
	div_map = params->div_map;
	*div = 0;

	rate /= MHZ_TO_HZ;
	parent_rate /= MHZ_TO_HZ;

	/* Choose the min divider, max freq */
	for (i = 1; i < size; i++) {
		if ((rate <= (parent_rate / div_map[i - 1].div)) &&
		    (rate > (parent_rate / div_map[i].div))) {
			*div = div_map[i - 1].div;
			return div_map[i - 1].hw_val;
		}
	}
	/* rate is higher than all acceptable rates, uses the min divider */
	*div = div_map[0].div;
	return div_map[0].hw_val;
}

static void clk_pll_vco_init(struct clk_hw *hw)
{
	struct clk_vco *vco = to_clk_vco(hw);
	unsigned long vco_rate;
	unsigned int vco_rngl, vco_rngh, tmp;
	struct mmp_vco_params *params = vco->params;

	if (!__pll_is_enabled(hw))
		pr_info("%s is not enabled\n", hw->clk->name);
	else {
		vco_rate = __get_vco_freq(hw) / MHZ_TO_HZ;
		/* check whether vco is in the range of 2% our expectation */
		tmp = params->default_vco_rate / MHZ_TO_HZ;
		if (tmp != vco_rate) {
			vco_rngh = tmp + tmp * 2 / 100;
			vco_rngl = tmp - tmp * 2 / 100;
			BUG_ON(!((vco_rngl <= vco_rate) && \
				(vco_rate <= vco_rngh)));
		}
		hw->clk->rate = params->default_vco_rate;

		/* Make sure SSC is enabled if pll is on */
		if (vco->flags & MMP_PLL_SSC_FEAT) {
			enable_pll_ssc(hw);
			params->ssc_enabled = true;
		}

		pr_info("%s is enabled @ %lu\n", hw->clk->name,
			vco_rate * MHZ_TO_HZ);
	}
}

static int clk_pll_vco_enable(struct clk_hw *hw)
{
	unsigned int delaytime = 14;
	u32 val;
	unsigned long flags;
	struct clk_vco *vco;
	struct mmp_vco_params *params;
	vco = to_clk_vco(hw);
	params = vco->params;

	if (__pll_is_enabled(hw))
		return 0;

	spin_lock_irqsave(vco->lock, flags);
	if (vco->flags & MMP_PLL_LOCK_SETTING) {
		val = pll_readl_cr(vco);
		/* we must lock refd/fbd first before enabling PLL2 */
		val |= params->ctrl_bit;
		pll_writel_cr(val, vco);
		val &= ~params->ctrl_bit;
		/* Let HW control PLL2 */
		pll_writel_cr(val, vco);
	} else {
		val = pll_readl_cr(vco);
		val |= params->enable_bit;
		pll_writel_cr(val, vco);
	}
	spin_unlock_irqrestore(vco->lock, flags);

	/* check lock status */
	udelay(30);
	while ((!(__raw_readl(params->lock_reg) & params->lock_enable_bit))
	       && delaytime) {
		udelay(5);
		delaytime--;
	}
	if (unlikely(!delaytime))
		BUG_ON(1);

	trace_clock_enable(hw->clk->name, 1, 0);

	if (vco->flags & MMP_PLL_SSC_FEAT)
		if (((vco->flags & MMP_PLL_SSC_AON) && !params->ssc_enabled)
		    || !(vco->flags & MMP_PLL_SSC_AON)) {
			enable_pll_ssc(hw);
			params->ssc_enabled = true;
		}

	return 0;
}

static void clk_pll_vco_disable(struct clk_hw *hw)
{
	u32 val;
	unsigned long flags;
	struct clk_vco *vco;
	struct mmp_vco_params *params;
	vco = to_clk_vco(hw);
	params = vco->params;

	spin_lock_irqsave(vco->lock, flags);
	if (vco->flags & MMP_PLL_LOCK_SETTING) {
		val = pll_readl_cr(vco);
		val |= params->ctrl_bit;
		val &= ~params->enable_bit;
		pll_writel_cr(val, vco);
	} else {
		val = pll_readl_cr(vco);
		val &= ~params->ctrl_bit;
		val &= ~params->enable_bit;
		pll_writel_cr(val, vco);
	}
	spin_unlock_irqrestore(vco->lock, flags);
	trace_clock_disable(hw->clk->name, 0, 0);

	if ((vco->flags & MMP_PLL_SSC_FEAT) &&
	   !(vco->flags & MMP_PLL_SSC_AON)) {
		disable_pll_ssc(vco);
		params->ssc_enabled = false;
	}
}
/*
 * pll2 rate change requires sequence:
 * clock off -> change rate setting -> clock on
 * This function doesn't really change rate, but cache the config
 */
static int clk_pll_vco_setrate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	unsigned int i, kvco, vcovnrg, refd, fbd;
	unsigned long flags;
	unsigned long new_rate = rate;
	u32 val;
	struct clk_vco *vco;
	struct mmp_vco_params *params;
	vco = to_clk_vco(hw);
	params = vco->params;
	if (__pll_is_enabled(hw)) {
		pr_info("%s pll vco is enabled\n",\
			__func__);
		return 0;
	}

	if (rate > params->vco_max || rate < params->vco_min) {
		pr_err("%lu rate out of range!\n", rate);
		return -EINVAL;
	}
	rate /= MHZ;
	/* setp 1: calculate fbd refd kvco and vcovnrg */
	if (vco->freq_table) {
		for (i = 0; i < vco->freq_table_size; i++) {
			if (rate == vco->freq_table[i].output_rate) {
				kvco = vco->freq_table[i].kvco;
				vcovnrg = vco->freq_table[i].vcovnrg;
				refd = vco->freq_table[i].refd;
				fbd = vco->freq_table[i].fbd;
				break;
			}
		}
		if (i == vco->freq_table_size) {
			WARN_ON(1);
			kvco = vco->freq_table[0].kvco;
			vcovnrg = vco->freq_table[0].vcovnrg;
			refd = vco->freq_table[0].refd;
			fbd = vco->freq_table[0].fbd;
		}
	} else {
		__clk_pll_rate2rng(vco, rate, &kvco, &vcovnrg);
		/* FIXME */
		/* refd need be calculated by certain function
		other than a fixed number*/
		refd = 3;
		if (vco->flags & MMP_PLL_28NM)
			fbd = rate * refd / 104;
		else
			fbd = rate * refd / 26;
	}

	spin_lock_irqsave(vco->lock, flags);
	/* setp 2: set pll kvco and vcovnrg setting */
	val = pll_readl_pll_swcr(vco);
	val &= ~(MASK(params->pll_offset->kvco_width) <<
			params->pll_offset->kvco_shift);
	val |= kvco << params->pll_offset->kvco_shift;
	if (!(vco->flags & MMP_PLL_28NM)) {
		val &= ~(MASK(params->pll_offset->vrng_width) <<
				params->pll_offset->vrng_shift);
		val |= vcovnrg << params->pll_offset->vrng_shift;
	}
	pll_writel_pll_swcr(val, vco);

	/* setp 3: set pll fbd and refd setting */
	val = pll_readl_cr(vco);
	val &= ~(MASK(params->pll_offset->fbd_width) <<
			params->pll_offset->fbd_shift);
	val |= fbd << params->pll_offset->fbd_shift;
	val &= ~(MASK(params->pll_offset->refd_width) <<
			params->pll_offset->refd_shift);
	val |= refd << params->pll_offset->refd_shift;
	pll_writel_cr(val, vco);

	hw->clk->rate = new_rate;
	spin_unlock_irqrestore(vco->lock, flags);
	trace_clock_set_rate(hw->clk->name, new_rate, 0);
	return 0;
}

static unsigned long clk_vco_recalc_rate(struct clk_hw *hw,
			unsigned long parent_rate)
{
	return hw->clk->rate;
}

static long clk_vco_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_vco *vco;
	int fbd, refd = 3;
	unsigned long max_rate = 0;
	int i;
	struct mmp_vco_params *params;
	vco = to_clk_vco(hw);
	params = vco->params;

	if (rate > params->vco_max || rate < params->vco_min) {
		pr_err("%lu rate out of range!\n", rate);
		if (rate > params->vco_max)
			return params->vco_max;
		else
			return params->vco_min;
	}

	if (vco->freq_table) {
		for (i = 0; i < vco->freq_table_size; i++) {
			if (vco->freq_table[i].output_rate <= rate) {
				if (max_rate < vco->freq_table[i].output_rate)
					max_rate =\
						vco->freq_table[i].output_rate;
			}
		}
	} else {
		rate = rate / MHZ;
		if (vco->flags & MMP_PLL_28NM) {
			fbd = rate * refd / 104;
			max_rate = DIV_ROUND_UP(104 * fbd, refd);
			max_rate *= MHZ;
		} else {
			fbd = rate * refd / 26;
			max_rate = DIV_ROUND_UP(26 * fbd, refd);
			max_rate *= MHZ;
		}
	}
	return max_rate;
}

static struct clk_ops clk_vco_ops = {
	.init = clk_pll_vco_init,
	.enable = clk_pll_vco_enable,
	.disable = clk_pll_vco_disable,
	.set_rate = clk_pll_vco_setrate,
	.recalc_rate = clk_vco_recalc_rate,
	.round_rate = clk_vco_round_rate,
	.is_enabled = __pll_is_enabled,
};

static void clk_pll_init(struct clk_hw *hw)
{
	unsigned long parent_rate;
	struct mmp_pll_params *params;
	struct clk_pll *pll;
	int div, div_hw;
	u32 val;
	struct clk *parent = hw->clk->parent;
	if (!__pll_is_enabled(parent->hw)) {
		pr_info("%s is not enabled\n", hw->clk->name);
		return;
	}
	parent_rate = clk_get_rate(parent);
	pll = to_clk_pll(hw);
	params = pll->params;
	val = pll_readl_pll_swcr(pll);
	div_hw = (val >> params->div_shift) & MASK(params->div_width);
	div = __pll_div_hwval2div(pll, div_hw);
	hw->clk->rate = parent_rate / div;
	pr_info("%s is enabled @ %lu\n", hw->clk->name,
		hw->clk->rate);
}

static int clk_pll_setrate(struct clk_hw *hw, unsigned long new_rate,
				unsigned long best_parent_rate)
{
	unsigned int div_val;
	unsigned long flags;
	u32 val;
	struct mmp_pll_params *params;
	struct clk_pll *pll;
	pll = to_clk_pll(hw);
	params = pll->params;
	spin_lock_irqsave(pll->lock, flags);
	div_val = __clk_pll_calc_div(pll, new_rate, best_parent_rate,
			&pll->div);
	val = pll_readl_pll_swcr(pll);
	val &= ~(MASK(params->div_width) << params->div_shift);
	val |= div_val << params->div_shift;
	pll_writel_pll_swcr(val, pll);
	hw->clk->rate = new_rate;
	spin_unlock_irqrestore(pll->lock, flags);
	trace_clock_set_rate(hw->clk->name, new_rate, 0);
	return 0;
}

static unsigned long clk_pll_recalc_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	return hw->clk->rate;
}

static long clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct mmp_pll_params *params;
	struct clk_pll *pll;
	struct clk_vco *vco;
	struct mmp_vco_params *vco_params;
	int i;
	unsigned long delta, new_rate, max_rate, parent_rate;
	bool need_chg_prate = false;
	struct clk *parent = hw->clk->parent;
	pll = to_clk_pll(hw);
	params = pll->params;
	vco = to_clk_vco(parent->hw);
	max_rate = 0;
	vco_params = vco->params;

	if (vco->flags & MMP_PLL_28NM)
		delta = 104 / 3;
	else
		delta = 26 / 3;
	parent_rate = *prate / MHZ;
	rate /= MHZ;

	if (rate <= parent_rate) {
		for (i = 0; i < params->div_map_size; i++) {
			new_rate = parent_rate / params->div_map[i].div;
			if (new_rate <= rate)
				if (max_rate < new_rate)
					max_rate = new_rate;
		}
		if (hw->clk->flags & CLK_SET_RATE_PARENT) {
			if (abs(rate - max_rate) <= delta)
				return max_rate * MHZ;
			else
				need_chg_prate = true;
		} else
			return max_rate * MHZ;
	}
	if ((rate > parent_rate) || need_chg_prate) {
		if (!(hw->clk->flags & CLK_SET_RATE_PARENT)) {
			WARN_ON(1);
			return parent_rate;
		}
		for (i = 0; i < params->div_map_size; i++) {
			max_rate = rate * params->div_map[i].div * MHZ;
			if (max_rate <= vco_params->vco_max && \
				max_rate >= vco_params->vco_min)
				break;
		}
		*prate = rate * MHZ * params->div_map[i].div;
	}
	return rate * MHZ;
}

static struct clk_ops clk_pll_ops = {
	.init = clk_pll_init,
	.set_rate = clk_pll_setrate,
	.recalc_rate = clk_pll_recalc_rate,
	.round_rate = clk_pll_round_rate,
};

struct clk *mmp_clk_register_vco(const char *name, const char *parent_name,
		unsigned long flags, u32 vco_flags, spinlock_t *lock,
		struct mmp_vco_params *params,
		struct mmp_vco_freq_table *freq_table, int table_size)
{
	struct clk_vco *vco;
	struct clk *clk;
	struct clk_init_data init;

	vco = kzalloc(sizeof(*vco), GFP_KERNEL);
	if (!vco)
		return NULL;

	init.name = name;
	init.ops = &clk_vco_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	vco->flags = vco_flags;
	vco->lock = lock;
	vco->hw.init = &init;
	vco->params = params;
	if (freq_table) {
		vco->freq_table = freq_table;
		vco->freq_table_size = table_size;
	}
	clk = clk_register(NULL, &vco->hw);
	if (IS_ERR(clk))
		kfree(vco);

	return clk;
}


struct clk *mmp_clk_register_pll(const char *name, const char *parent_name,
		unsigned long flags, u32 pll_flags, spinlock_t *lock,
		struct mmp_pll_params *params)
{
	struct clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return NULL;

	init.name = name;
	init.ops = &clk_pll_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	pll->flags = pll_flags;
	pll->lock = lock;
	pll->params = params;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
