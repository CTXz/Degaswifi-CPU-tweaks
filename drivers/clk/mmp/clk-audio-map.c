/*
 * mmp map Audio clock operation source file
 *
 * Copyright (C) 2014 Marvell
 * Zhao Ye <zhaoy@marvell.com>
 * Nenghua Cao <nhcao@marvell.com>
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

#define APMU_AUDIO_PWR_UP		(3 << 9)
#define APMU_AUDIO_PWR_DOWN		(0 << 9)
#define APMU_AUDIO_ISO_DIS		(1 << 8)
#define APMU_AUDIO_CLK_SEL_PLL1_DIV_2	(0 << 6)
#define APMU_AUDIO_CLK_SEL_PLL2_DIV_2	(1 << 6)
#define APMU_AUDIO_CLK_SEL_PLL2_DIV_3	(2 << 6)
#define APMU_AUDIO_CLK_SEL_PLL1_DIV_8	(3 << 6)
#define APMU_AUDIO_CLK_ENA		(1 << 4)
#define APMU_AUDIO_RST_DIS		(1 << 1)

#define CLK_RES_CTRL		(0x010c)
#define ISLAND_SRAM_PWR_DWN_CTRL  (0x0240)
#define DSA_CLK_RES_CTRL  (0x0164)
#define ISLD_AU_CTRL  (0x01A8)

#define APLL1_CONFIG_REG1	(0x68)
#define APLL1_CONFIG_REG2	(0x6c)
#define APLL1_CONFIG_REG3	(0x70)
#define APLL1_CONFIG_REG4	(0x74)

#define APLL2_CONFIG_REG1	(0x78)
#define APLL2_CONFIG_REG2	(0x7c)
#define APLL2_CONFIG_REG3	(0x80)
/* DSP Audio PLL 2 Config Register 1 */
#define APLL2_CONF1_PLL_LOCK		(1 << 31)
#define APLL2_CONF1_ICP(x)		((x) << 27)
#define APLL2_CONF1_FBDIV(x)		((x) << 18)
#define APLL2_CONF1_REFDIV(x)	((x) << 9)
#define APLL2_CONF1_CLK_DET_EN	(1 << 8)
#define APLL2_CONF1_INTPI(x)		((x) << 6)
#define APLL2_CONF1_FD_SEL(x)	((x) << 4)
#define APLL2_CONF1_CTUNE(x)		((x) << 2)
#define APLL2_CONF1_RESET		(1 << 1)
#define APLL2_CONF1_PU		(1 << 0)

/* DSP Audio PLL 2 Config Register 2 */
#define APLL2_CONF2_RESET_INTP_EXT		(1 << 27)
#define APLL2_CONF2_POSTDIV_AUDIO(x)		((x) << 20)
#define APLL2_CONF2_FREQ_OFFSET(x)		((x) << 4)
#define APLL2_CONF2_FREQ_OFFSET_EXT		(1 << 3)
#define APLL2_CONF2_FREQ_OFFSET_VALID	(1 << 2)
#define APLL2_CONF2_POSTDIV_AUDIO_EN		(1 << 1)
#define APLL2_CONF2_PI_EN			(1 << 0)

#define to_clk_audio(clk) (container_of(clk, struct clk_audio, clk))

#define AUD_NO_RESET_CTRL (1)

#ifdef CONFIG_SOUND
static DEFINE_SPINLOCK(clk_lock);

struct map_clk_audio_pll_table {
	unsigned long input_rate;
	unsigned long output_rate;
	unsigned long output_rate_offseted;
	unsigned long freq_offset;
	u16 refdiv;
	u16 fbdiv;
	u16 icp;
	u16 postdiv;
};

struct clk_audio {
	struct clk_hw   hw;
	void __iomem    *apmu_base;
	void __iomem    *aud_base;
	void __iomem    *aud_aux_base;
	u32		rst_mask;
	u32		enable_mask;
	int		enabled;
	spinlock_t	*lock;
	struct map_clk_audio_pll_table *freq_table;
	poweron_cb	poweron;
};

struct post_divider_phase_interpolator {
	uint32_t audio_sampling_freq;
	uint32_t over_sampleing_rate;
	uint32_t clkout_audio;
	uint32_t pre_divider;
	uint32_t postdiv_audio;
};

struct popular_reference_clock_freq {
	uint32_t refclk;
	uint32_t refdiv;
	uint32_t update_rate;
	uint32_t fbdiv;
	uint32_t freq_intp_out;
	uint32_t freq_intp_in;
	uint32_t freq_offset_0_14;
	uint32_t freq_offset_15;
	uint32_t freq_offset_0_14_hex;
};

struct popular_reference_clock_freq refclock_map[] = {
/* refclk refdiv update fbdiv  out      in   bit_0_14 bit_15 bit_hex *
 * -----  ----  -----   --    ------  --------   ---   ---   ---    */
{ 11289600, 2, 5644800, 24, 135475200, 135475200,    0, 0,    0x0 },
{ 11289600, 2, 5644800, 26, 146764800, 147456000, 2469, 0, 0x09A5 },
{ 12288000, 2, 6144000, 22, 135168000, 135475200, 1192, 0, 0x04A8 },
{ 12288000, 2, 6144000, 24, 147456000, 147456000,    0, 1,    0x0 },
{ 13000000, 3, 4333333, 31, 134333333, 135475200, 4457, 0, 0x1169 },
{ 13000000, 3, 4333333, 34, 147333333, 147456000,  437, 0, 0x01B5 },
{ 16934400, 3, 5644800, 24, 135475200, 135475200,    0, 0,    0x0 },
{ 16934400, 3, 5644800, 26, 146764800, 147456000, 2469, 0, 0x09A5 },
{ 18432000, 3, 6144000, 22, 135168000, 135475200, 1192, 0, 0x04A8 },
{ 18432000, 3, 6144000, 24, 147456000, 147456000,    0, 0,    0x0 },
{ 22579200, 4, 5644800, 24, 135475200, 135475200,    0, 0,    0x0 },
{ 22579200, 4, 5644800, 26, 146764800, 147456000, 2469, 0, 0x09A5 },
{ 24576000, 4, 6144000, 22, 135168000, 135475200, 1192, 0, 0x04A8 },
{ 24576000, 4, 6144000, 24, 147456000, 147456000,    0, 0,    0x0 },
{ 26000000, 6, 4333333, 31, 134333333, 135475200, 4457, 0, 0x1169 },
{ 26000000, 6, 4333333, 34, 147333333, 147456000,  437, 0, 0x01B5 },
{ 38400000, 6, 6400000, 21, 134400000, 135475200, 4194, 0, 0x1062 },
{ 38400000, 6, 6400000, 23, 147200000, 147456000,  912, 0, 0x0390 },
};

struct post_divider_phase_interpolator pdiv_map_8k[] = {
/* audio  over  clkout  prediv  postdiv *
 * -----  ----  -----   -----   ------- */
{ 32000, 128,  4096000,  147456000,  36 },
{ 32000, 192,  6144000,  147456000,  24 },
{ 32000, 256,  8192000,  147456000,  18 },
{ 32000, 384, 12288000,  147456000,  12 },
{ 32000, 512, 16384000,  147456000,   9 },
{ 48000, 128,  6144000,  147456000,  24 },
{ 48000, 192,  9216000,  147456000,  16 },
{ 48000, 256, 12288000,  147456000,  12 },
{ 48000, 384, 18432000,  147456000,   8 },
{ 48000, 512, 24567000,  147456000,   6 },
{ 96000, 128, 12288000,  147456000,  12 },
{ 96000, 192, 18432000,  147456000,   8 },
{ 96000, 256, 24567000,  147456000,   6 },
{ 96000, 384, 36864000,  147456000,   4 },
{ 96000, 512, 49152000,  147456000,   3 },
};

struct post_divider_phase_interpolator pdiv_map_11k[] = {
/* audio  over  clkout  prediv  postdiv *
 * -----  ----  -----   -----   ------- */
{ 11025, 128,  1411200,  135475200,  96 },
{ 11025, 192,  2116800,  135475200,  64 },
{ 11025, 256,  2822400,  135475200,  48 },
{ 11025, 384,  4233600,  135475200,  32 },
{ 11025, 512,  5644800,  135475200,  24 },
{ 22050, 128,  2822400,  135475200,  48 },
{ 22050, 192,  4233600,  135475200,  32 },
{ 22050, 256,  5644800,  135475200,  24 },
{ 22050, 384,  8467200,  135475200,  16 },
{ 22050, 512, 11289600,  135475200,  12 },
{ 44100, 128,  5644800,  135475200,  24 },
{ 44100, 192,  8467200,  135475200,  16 },
{ 44100, 256, 11289600,  135475200,  12 },
{ 44100, 384, 16934400,  135475200,   8 },
{ 44100, 512, 22579200,  135475200,   6 },
{ 88200, 128, 11289600,  135475200,  12 },
{ 88200, 192, 16934400,  135475200,   8 },
{ 88200, 256, 22579200,  135475200,   6 },
{ 88200, 384, 33868800,  135475200,   4 },
{ 88200, 512, 45158400,  135475200,   3 },
};

struct fvco_clk_with_32kref {
	uint32_t fbdiv_int;
	uint32_t fbdiv_dec;
	uint32_t fvco;
};

/* audio pll1 clock table */
struct fvco_clk_with_32kref fvco_clock_with_32kref_map[] = {
	/* fbdiv_int	fbdiv_dec	fvco*
	 * --------	-----		------*/
	{137,	13,		451584000},
	{165,	6,		541900800},
	{180,	0,		589824000},
};

struct post_divider_phase_interpolator pdiv_for_8k_with_32kref[] = {
	/* audio  over  clkout  prediv  postdiv *
	 * -----  ----  -----   -----   ------- */
	{ 8000,  32,  256000,  589824000,  2304},
	{ 8000,  64,  512000,  589824000,  1152},
	{ 8000, 128, 1024000,  589824000,  576},
	{ 32000, 32,  1024000, 589824000,  576},
	{ 32000, 64,  2048000, 589824000,  288},
	{ 32000, 128, 4096000, 589824000,  144},
	{ 48000, 32,  1536000, 589824000,  384},
	{ 48000, 64,  3072000, 589824000,  192},
	{ 48000, 128, 6144000, 589824000,  96},
};

struct map_clk_apll1_table {
	unsigned long input_rate;
	unsigned long output_rate;
	uint32_t fbdiv_int;
	uint32_t fbdiv_dec;
	uint32_t fvco;
};

static int clk_apll_enable(struct clk_hw *hw)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int val = 0;
	unsigned long flags;

	/* enable audio power domain */
	audio->poweron(audio->apmu_base, 1);

	spin_lock_irqsave(audio->lock, flags);
	val = audio->enable_mask;
	writel_relaxed(val, audio->apmu_base + CLK_RES_CTRL);

	audio->enabled = 1;
	spin_unlock_irqrestore(audio->lock, flags);

	return 0;
}

static void clk_apll_disable(struct clk_hw *hw)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int val = 0;
	unsigned long flags;

	spin_lock_irqsave(audio->lock, flags);
	val = readl_relaxed(audio->apmu_base + CLK_RES_CTRL);
	val &= ~(audio->enable_mask);
	writel_relaxed(val, audio->apmu_base + CLK_RES_CTRL);

	audio->enabled = 0;
	spin_unlock_irqrestore(audio->lock, flags);

	/* if audio pll is closed, we can close the audio island */
	audio->poweron(audio->apmu_base, 0);
}

static long __map_apll1_get_rate_table(struct clk_hw *hw,
		struct map_clk_apll1_table *freq_tbl,
		unsigned long drate, unsigned long prate)
{
	int i;
	struct fvco_clk_with_32kref *ref_p;

	for (i = 0; i < ARRAY_SIZE(fvco_clock_with_32kref_map); i++) {

		/* find audio pll setting */
		ref_p = &fvco_clock_with_32kref_map[i];

		if (drate == ref_p->fvco)
			goto found;
	}

	return -EINVAL;
found:
	freq_tbl->input_rate = prate;
	freq_tbl->fbdiv_int = ref_p->fbdiv_int;
	freq_tbl->fbdiv_dec = ref_p->fbdiv_dec;
	freq_tbl->output_rate = drate;

	return drate;
}

static long clk_apll1_round_rate(struct clk_hw *hw, unsigned long drate,
		unsigned long *prate)
{
	struct map_clk_apll1_table freq_tbl;

	memset(&freq_tbl, 0, sizeof(freq_tbl));
	if (__map_apll1_get_rate_table(hw, &freq_tbl, drate, *prate) < 0)
		return -EINVAL;

	*prate = freq_tbl.input_rate;

	return freq_tbl.output_rate;
}

static unsigned long clk_apll1_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long fbdiv_dec, fbdiv_int;
	unsigned long pll_config1, pll_config2;
	unsigned long freq_intp_out;
	struct clk_audio *audio = to_clk_audio(hw);

	/* before touch the pll register, we need audio island is opened */
	if (audio->enabled) {
		pll_config1 = readl_relaxed(audio->aud_aux_base +
			APLL1_CONFIG_REG1);
		pll_config2 = readl_relaxed(audio->aud_aux_base +
			APLL1_CONFIG_REG2);

		fbdiv_dec = (pll_config1 & (0xf<<23))>>23;
		fbdiv_int = (pll_config1 & (0xff<<15))>>15;
		freq_intp_out = parent_rate * 100 * (fbdiv_int + fbdiv_dec/16);

		/* accurate to KHz */
		if (((freq_intp_out)/1000) != ((hw->clk->rate)/1000))
			return -EINVAL;
	}

	return hw->clk->rate;
}

/* Configures new clock rate */
static int clk_apll1_set_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long prate)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int regConfig1, regConfig2;
	unsigned int CTUNE = 0x1;	/* 1 unit cap loading */
	struct map_clk_apll1_table freq_tbl;
	unsigned long flags;

	memset(&freq_tbl, 0, sizeof(freq_tbl));
	spin_lock_irqsave(audio->lock, flags);

	if (__map_apll1_get_rate_table(hw, &freq_tbl, drate, prate) < 0) {
		spin_unlock_irqrestore(audio->lock, flags);
		return -EINVAL;
	}

	/* Reset PLL Reset=1 */
	regConfig1 = readl_relaxed(audio->aud_aux_base +
			APLL1_CONFIG_REG1);
	regConfig1 |= (1<<1); /* reset */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL1_CONFIG_REG1);

	/* Disable PLL PU=0 */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL1_CONFIG_REG1);
	regConfig1 &= ~(1<<0); /* pll disable */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL1_CONFIG_REG1);

	/* Audio PLL config register 1 */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL1_CONFIG_REG1);
	regConfig1 &= ~(0xF<<27);	/* icp */
	regConfig1 &= ~(0xf<<23);	/* fbdiv_dec */
	regConfig1 &= ~(0xFF<<15);	/* fbdiv_int */
	regConfig1 &= ~(0x3<<2);	/* ctune */
	regConfig1 &= ~(0x3<<4);	/* lock detector accuracy set */

	regConfig1 |= 1 << 31; /* hardcoded to output 540M or 590M */
	regConfig1 |= 0x6 << 27; /* icp */
	regConfig1 |= freq_tbl.fbdiv_dec << 23; /* fbdiv_dec */
	regConfig1 |= freq_tbl.fbdiv_int << 15; /* fbdiv_int */
	regConfig1 |= (CTUNE<<2);	/* ctune */
	regConfig1 |= (1<<4);		/* lock detector accuracy set */

	/* Audio PLL config register 2 */
	regConfig2 = readl_relaxed(audio->aud_aux_base + APLL1_CONFIG_REG2);
	/* Fixme: we put post div into common clock tree */
#if 0
	regConfig2 &= ~(0xfff<<11);	/* postdiv_audio */

	regConfig2 |= freq_tbl.postdiv << 11;
	regConfig2 |= (1<<10);		/* postdiv_audio_en */
#endif
	/* Fixme: maybe don't need to enable fvso */
	regConfig2 |= (1<<23);		/* fvco_en */

	/* Audio PLL config register 2 */
	writel_relaxed(regConfig2, audio->aud_aux_base + APLL1_CONFIG_REG2);

	/* enable PLL PU=1 */
	regConfig1 |= (1<<0);	/* pll disable */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL1_CONFIG_REG1);
	spin_unlock_irqrestore(audio->lock, flags);
	/* need to check if it is needed when enabling silicon */
	udelay(10);

	spin_lock_irqsave(audio->lock, flags);
	/* Release reset */
	regConfig1 &= ~(1<<1);	/* release reset */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL1_CONFIG_REG1);
	spin_unlock_irqrestore(audio->lock, flags);
	/* need to check if it is needed when enabling silicon */
	udelay(10);

	spin_lock_irqsave(audio->lock, flags);
	hw->clk->rate = drate;
	spin_unlock_irqrestore(audio->lock, flags);

	return 0;
}

struct clk_ops clk_apll1_ops = {
	.enable = clk_apll_enable,
	.disable = clk_apll_disable,
	.recalc_rate = clk_apll1_recalc_rate,
	.round_rate = clk_apll1_round_rate,
	.set_rate = clk_apll1_set_rate,
};

struct clk *mmp_clk_register_apll1(const char *name, const char *parent_name,
		void __iomem *apmu_base, void __iomem *aud_base,
		void __iomem *aud_aux_base, u32 enable_mask,
		poweron_cb poweron, spinlock_t *lock)
{
	struct clk_audio *audio;
	struct clk *clk;
	struct clk_init_data init;

	audio = kzalloc(sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return NULL;

	init.name = name;
	init.ops = &clk_apll1_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	audio->apmu_base = apmu_base;
	audio->aud_base = aud_base;
	audio->aud_aux_base = aud_aux_base;
	audio->enable_mask = enable_mask;
	audio->poweron = poweron;
	audio->lock = lock;
	audio->hw.init = &init;

	clk = clk_register(NULL, &audio->hw);

	if (IS_ERR(clk))
		kfree(audio);

	return clk;
}

static long __map_apll2_get_rate_table(struct clk_hw *hw,
		struct map_clk_audio_pll_table *freq_tbl,
			unsigned long drate, unsigned long prate)
{
	int i;
	struct popular_reference_clock_freq *ref_p;

	for (i = 0; i < ARRAY_SIZE(refclock_map); i++) {

		/* find audio pll setting */
		ref_p = &refclock_map[i];
		if (ref_p->refclk != prate)
			continue;

		if (drate == ref_p->freq_intp_in)
			goto found;
	}

	return -EINVAL;
found:
	freq_tbl->input_rate = prate;
	freq_tbl->fbdiv = ref_p->fbdiv;
	freq_tbl->refdiv = ref_p->refdiv;
	freq_tbl->freq_offset = ref_p->freq_offset_0_14_hex;
	freq_tbl->output_rate = drate;

	return drate;
}

static long clk_apll2_round_rate(struct clk_hw *hw, unsigned long drate,
		unsigned long *prate)
{
	struct map_clk_audio_pll_table freq_tbl;

	memset(&freq_tbl, 0, sizeof(freq_tbl));
	if (__map_apll2_get_rate_table(hw, &freq_tbl, drate, *prate) < 0)
		return -EINVAL;

	*prate = freq_tbl.input_rate;

	return freq_tbl.output_rate;
}

static unsigned long clk_apll2_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long fbdiv, refdiv, offset;
	unsigned long pll_config1, pll_config2;
	unsigned long freq_intp_out, freq_intp_in;
	struct clk_audio *audio = to_clk_audio(hw);

	/* before touch the pll register, we need audio island is opened */
	if (audio->enabled) {
		pll_config1 = readl_relaxed(audio->aud_aux_base +
			APLL2_CONFIG_REG1);
		pll_config2 = readl_relaxed(audio->aud_aux_base +
			APLL2_CONFIG_REG2);

		fbdiv = (pll_config1 >> 18) & 0x1ff;
		refdiv = (pll_config1 >> 9) & 0x1FF;
		offset = (pll_config2 >> 4) & 0xFFFF;
		freq_intp_out = (parent_rate / refdiv * fbdiv);
		freq_intp_in = (freq_intp_out >> 19) * offset + freq_intp_out;

		/* accurate to KHz */
		if ((freq_intp_in/1000) != ((hw->clk->rate)/1000))
			return -EINVAL;
	}

	return hw->clk->rate;
}

/* Configures new clock rate */
static int clk_apll2_set_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long prate)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int regConfig1, regConfig2, regConfig3;
	unsigned int clkout_vco_div;
	unsigned int CTUNE = 0x1;	/* 1 unit cap loading */
	unsigned int INTPI = 0x2;
	unsigned int count = 0x200;
	struct map_clk_audio_pll_table freq_tbl;
	unsigned long flags;

	memset(&freq_tbl, 0, sizeof(freq_tbl));
	spin_lock_irqsave(audio->lock, flags);

	if (__map_apll2_get_rate_table(hw, &freq_tbl, drate, prate) < 0) {
		spin_unlock_irqrestore(audio->lock, flags);
		return -EINVAL;
	}

	/* hardcoded to output 22.579M or 24.576M */
	clkout_vco_div = 1; /* hardcoded to output 541.9008M or 589.824M */

	/* Reset PLL Reset=1 */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG1);
	regConfig1 |= (1<<1); /* reset */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL2_CONFIG_REG1);

	/* Disable PLL PU=0 */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG1);
	regConfig1 &= ~(1<<0); /* pll disable */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL2_CONFIG_REG1);

	/* Audio PLL config register 1 */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG1);
	regConfig1 &= ~(0xF<<27);	/* icp */
	regConfig1 &= ~(0x1FF<<18);	/* fbdiv */
	regConfig1 &= ~(0x1FF<<9);	/* refdiv */
	regConfig1 &= ~(0x3<<6);	/* intpi */
	regConfig1 &= ~(0x3<<2);	/* ctune */
	regConfig1 &= ~(0x3<<4);	/* lock detector accuracy set */

	regConfig1 |= APLL2_CONF1_ICP(0x6); /* icp */
	regConfig1 |= APLL2_CONF1_FBDIV(freq_tbl.fbdiv); /* fbdiv */
	regConfig1 |= APLL2_CONF1_REFDIV(freq_tbl.refdiv); /* refdiv */
	regConfig1 |= (INTPI<<6);	/* intpi */
	regConfig1 |= (CTUNE<<2);	/* ctune */
	regConfig1 |= (1<<8);		/* PI clockout enable */
	regConfig1 |= (1<<4);		/* lock detector accuracy set */

	/* Audio PLL config register 2 */
	regConfig2 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG2);

	regConfig2 &= ~(0xFFFF<<4);	/* freq_offset */

	/* Fixme: configure postdiv in common clock tree */
	regConfig2 |= ((freq_tbl.freq_offset)<<4);	/* freq_offset */

	regConfig2 |= (1<<0);		/* phase-interpolator enable */
	/* (RESET_INTP_EXT)external interpolator reset signal::no reset */
	regConfig2 &= ~(1<<27);
	/* (RESET_OFFSET_EXT)external offset logic reset signal::no reset */
	regConfig2 &= ~(1<<3);
	/* (FREQ_OFFSET_VALID)frequency offset valid */
	regConfig2 &= ~(1<<2);

	/* Audio PLL config register 3 */
	regConfig3 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG3);
	regConfig3 &= ~(0x1FF<<15);	/* clk_vco_div_sel */

	regConfig3 |= (1<<24);		/* clk_vco_div_en */
	regConfig3 |= (clkout_vco_div<<15);	/* clk_vco_div_sel */
	/* clk_vco_en :: TODO:: need to find if this should be set */
	regConfig3 |= (1<<14);

	writel_relaxed(regConfig3, audio->aud_aux_base + APLL2_CONFIG_REG3);

	/* Audio PLL config register 2 */
	writel_relaxed(regConfig2, audio->aud_aux_base + APLL2_CONFIG_REG2);

	/* enable PLL PU=1 */
	regConfig1 |= (1<<0);	/* pll disable */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL2_CONFIG_REG1);
	spin_unlock_irqrestore(audio->lock, flags);
	/* need to check if it is needed when enabling silicon */
	udelay(10);

	spin_lock_irqsave(audio->lock, flags);
	/* Release reset */
	regConfig1 &= ~(1<<1);	/* release reset */
	writel_relaxed(regConfig1, audio->aud_aux_base + APLL2_CONFIG_REG1);
	spin_unlock_irqrestore(audio->lock, flags);
	/* need to check if it is needed when enabling silicon */
	udelay(10);

	spin_lock_irqsave(audio->lock, flags);
	/* send pulse for freq_offset_valid */
	regConfig2 |= (1<<2);
	writel_relaxed(regConfig2, audio->aud_aux_base + APLL2_CONFIG_REG2);
	spin_unlock_irqrestore(audio->lock, flags);
	/* need to check if it is needed when enabling silicon */
	udelay(10);

	spin_lock_irqsave(audio->lock, flags);
	/* check if PLL is locked */
	regConfig1 = readl_relaxed(audio->aud_aux_base + APLL2_CONFIG_REG1);
	regConfig1 &= 0x80000000;
	while (!regConfig1) {
		regConfig1 = readl_relaxed(audio->aud_aux_base +
			APLL2_CONFIG_REG1);
		regConfig1 &= 0x80000000;
		spin_unlock_irqrestore(audio->lock, flags);
		/* need to check if it is needed when enabling silicon */
		udelay(10);

		spin_lock_irqsave(audio->lock, flags);
		count--;
		if (count <= 0) {
			pr_err("PLL lock timeout error PLL2_CONFIG_REG1g %x\n",
				regConfig1);
			spin_unlock_irqrestore(audio->lock, flags);
			return -1;
		}
	}

	hw->clk->rate = drate;
	spin_unlock_irqrestore(audio->lock, flags);

	return 0;
}

struct clk_ops clk_apll2_ops = {
	.enable = clk_apll_enable,
	.disable = clk_apll_disable,
	.recalc_rate = clk_apll2_recalc_rate,
	.round_rate = clk_apll2_round_rate,
	.set_rate = clk_apll2_set_rate,
};

struct clk *mmp_clk_register_apll2(const char *name, const char *parent_name,
		void __iomem *apmu_base, void __iomem *aud_base,
		void __iomem *aud_aux_base, u32 enable_mask,
		poweron_cb poweron, spinlock_t *lock)
{
	struct clk_audio *audio;
	struct clk *clk;
	struct clk_init_data init;

	audio = kzalloc(sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return NULL;

	init.name = name;
	init.ops = &clk_apll2_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	audio->apmu_base = apmu_base;
	audio->aud_base = aud_base;
	audio->aud_aux_base = aud_aux_base;
	audio->enable_mask = enable_mask;
	audio->poweron = poweron;
	audio->lock = lock;
	audio->hw.init = &init;

	clk = clk_register(NULL, &audio->hw);

	if (IS_ERR(clk))
		kfree(audio);

	return clk;
}

/* Common audio component reset/enable bit */

#define to_clk_audio_res(hw) container_of(hw, struct clk_audio_res, hw)
struct clk_audio_res {
	struct clk_hw		hw;
	void __iomem		*base;
	unsigned int		en_bit_offset;
	unsigned int		res_bit_offset;
	unsigned int		delay;
	unsigned int		flags;
	spinlock_t		*lock;
};

static int clk_audio_prepare(struct clk_hw *hw)
{
	struct clk_audio_res *audio = to_clk_audio_res(hw);
	unsigned int data;
	unsigned long flags = 0;

	if (audio->lock)
		spin_lock_irqsave(audio->lock, flags);

	data = readl_relaxed(audio->base);
	data |= (1 << audio->en_bit_offset);
	writel_relaxed(data, audio->base);

	if (audio->lock)
		spin_unlock_irqrestore(audio->lock, flags);

	udelay(audio->delay);

	if (!(audio->flags & AUD_NO_RESET_CTRL)) {
		if (audio->lock)
			spin_lock_irqsave(audio->lock, flags);

		data = readl_relaxed(audio->base);
		data |= (1 << audio->res_bit_offset);
		writel_relaxed(data, audio->base);

		if (audio->lock)
			spin_unlock_irqrestore(audio->lock, flags);
	}

	return 0;
}

static void clk_audio_unprepare(struct clk_hw *hw)
{
	struct clk_audio_res *audio = to_clk_audio_res(hw);
	unsigned long data;
	unsigned long flags = 0;

	if (audio->lock)
		spin_lock_irqsave(audio->lock, flags);

	data = readl_relaxed(audio->base);
	data &= ~(1 << audio->en_bit_offset);
	writel_relaxed(data, audio->base);

	if (audio->lock)
		spin_unlock_irqrestore(audio->lock, flags);
}

struct clk_ops clk_audio_res_ops = {
	.prepare = clk_audio_prepare,
	.unprepare = clk_audio_unprepare,
};

struct clk *mmp_clk_register_aud_res(const char *name, const char *parent_name,
		void __iomem *base, unsigned int en_bit_offset,
		unsigned int res_bit_offset, unsigned int delay,
		unsigned int audio_res_flags, spinlock_t *lock)
{
	struct clk_audio_res *audio;
	struct clk *clk;
	struct clk_init_data init;

	audio = kzalloc(sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return NULL;

	init.name = name;
	init.ops = &clk_audio_res_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	audio->base = base;
	audio->en_bit_offset = en_bit_offset;
	audio->res_bit_offset = res_bit_offset;
	audio->delay = delay;
	audio->flags = audio_res_flags;
	audio->lock = lock;
	audio->hw.init = &init;

	clk = clk_register(NULL, &audio->hw);
	if (IS_ERR(clk))
		kfree(audio);

	return clk;
}

const char *adma_parent[] = {
	"apll1_slow", "apll1_fast",
	"vctcxo", "312mhz",
};
const char *sspa1_parent[] = {
	"apll1_slow", "apll2_slow",
	"vctcxo", "312mhz",
};
static struct clk_factor_masks sspa_factor_masks = {
	.factor = 1,
	.den_mask = 0xfff,
	.num_mask = 0x7fff,
	.den_shift = 19,
	.num_shift = 4,
};
static struct clk_factor_tbl sspa1_factor_tbl[] = {
	{.num = 1625, .den = 256},	/* 8kHz */
	{.num = 3042, .den = 1321},	/* 44.1kHZ */
};

const char *sspa2_parent[] = {
	"apll1_slow", "apll2_slow",
	"vctcxo", "312mhz",
};
static struct clk_factor_tbl sspa2_factor_tbl[] = {
	{.num = 1625, .den = 256},	/* 8kHz */
	{.num = 3042, .den = 1321},	/* 44.1kHZ */
};

const char *map_parent[] = {
	"apll1_fast", "apll1_slow",
};

void __init audio_clk_init(void __iomem *apmu_base, void __iomem *audio_base,
		void __iomem *audio_aux_base, poweron_cb poweron)
{
	struct clk *apll1, *apll2, *apll1_slow;
	struct clk *clk;
	/* apll1 */
	apll1 = mmp_clk_register_apll1("mmp-apll1", "clk32768",
			apmu_base, audio_base, audio_aux_base,
			APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
			APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP,
			poweron, &clk_lock);
	clk_register_clkdev(apll1, "mmp-apll1", NULL);
	clk_prepare_enable(apll1);

	clk = clk_register_gate(NULL, "apll1_clkout_en", "mmp-apll1",
				CLK_SET_RATE_PARENT,
				audio_aux_base + 0x6c, 23, 0, &clk_lock);
	clk_register_clkdev(clk, "apll1_clkout_en", NULL);

	clk = clk_register_divider(NULL, "apll1_fast", "apll1_clkout_en", 0,
			audio_aux_base + 0x74, 0, 12,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "apll1_fast", NULL);

	clk = clk_register_divider(NULL, "postdiv_audio", "mmp-apll1", 0,
			audio_aux_base + 0x6c, 11, 12,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "postdiv_audio", NULL);

	apll1_slow = clk_register_gate(NULL, "apll1_slow", "postdiv_audio", 0,
				audio_aux_base + 0x6c, 10, 0, &clk_lock);
	clk_register_clkdev(apll1_slow, "apll1_slow", NULL);

	/* apb and sram */
	clk = clk_register_mux(NULL, "apb_mux", map_parent,
			ARRAY_SIZE(map_parent), 0,
			audio_aux_base + 0x0, 0, 1, 0, &clk_lock);
	clk_set_parent(clk, apll1_slow);
	clk_register_clkdev(clk, "apb_mux", NULL);

	clk = clk_register_divider(NULL, "apb_div", "apb_mux", 0,
			audio_aux_base + 0x0, 3, 4,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "apb_div", NULL);

	clk = mmp_clk_register_aud_res("mmp-apb", "apb_div",
				audio_aux_base + 0x0, 1, 2, 10,
				0, &clk_lock);
	clk_register_clkdev(clk, "mmp-apb", NULL);

	/* map apb */
	clk = clk_register_divider(NULL, "map_apb_div", "mmp-apb", 0,
			audio_aux_base + 0x14, 2, 4,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "map_apb_div", NULL);

	clk = mmp_clk_register_aud_res("mmp-map-apb", "map_apb_div",
				audio_aux_base + 0x14, 0, 1, 10,
				0, &clk_lock);
	clk_register_clkdev(clk, "mmp-map-apb", NULL);

	/* adma */
	clk = clk_register_mux(NULL, "adma_mux", adma_parent,
			ARRAY_SIZE(adma_parent), 0,
			audio_aux_base + 0x4, 2, 2, 0, &clk_lock);
	clk_set_parent(clk, apll1);
	clk_register_clkdev(clk, "adma_mux", NULL);

	clk = clk_register_divider(NULL, "adma_div", "adma_mux", 0,
			audio_aux_base + 0x4, 4, 4,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "adma_div", NULL);

	clk = mmp_clk_register_aud_res("mmp-adma", "adma_div",
				audio_aux_base + 0x4, 0, 1, 10,
				0, &clk_lock);
	clk_register_clkdev(clk, "mmp-adma", NULL);

	/* apll2 */
	apll2 = mmp_clk_register_apll2("apll2_fast", "vctcxo",
			apmu_base, audio_base, audio_aux_base,
			APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
			APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP,
			poweron, &clk_lock);
	clk_register_clkdev(apll2, "apll2-fast", NULL);
	clk_prepare_enable(apll2);

	clk = clk_register_divider(NULL, "apll2_post_div", "apll2_fast", 0,
			audio_aux_base + 0x7c, 20, 7,
			CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_ALLOW_ZERO,
			&clk_lock);
	clk_register_clkdev(clk, "apll2_post_div", NULL);

	clk = clk_register_gate(NULL, "apll2_slow", "apll2_post_div", 0,
				audio_aux_base + 0x7c, 1, 0, &clk_lock);
	clk_register_clkdev(clk, "apll2_slow", NULL);

	/* sspa1 */
	clk = clk_register_mux(NULL, "sspa1_mux", sspa1_parent,
			ARRAY_SIZE(sspa1_parent), 0,
			audio_aux_base + 0xc, 0, 2, 0, &clk_lock);
	clk_set_parent(clk, apll1);
	clk_register_clkdev(clk, "sspa1_mux", NULL);

	clk =  mmp_clk_register_factor("sspa1_mn_div", "sspa1_mux",
				CLK_SET_RATE_PARENT,
				audio_aux_base + 0xc,
				&sspa_factor_masks, sspa1_factor_tbl,
				ARRAY_SIZE(sspa1_factor_tbl));
	clk_register_clkdev(clk, "sspa1_mn_div", NULL);

	clk = mmp_clk_register_aud_res("mmp-sspa-dai.0", "sspa1_mn_div",
				audio_aux_base + 0xc, 3, 2, 10,
				0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-sspa-dai.0");

	/* sspa2 */
	clk = clk_register_mux(NULL, "sspa2_mux", sspa2_parent,
			ARRAY_SIZE(sspa2_parent), 0,
			audio_aux_base + 0x10, 0, 2, 0, &clk_lock);
	clk_set_parent(clk, apll1);
	clk_register_clkdev(clk, "sspa2_mux", NULL);

	clk =  mmp_clk_register_factor("sspa2_mn_div", "sspa2_mux",
				CLK_SET_RATE_PARENT,
				audio_aux_base + 0x10,
				&sspa_factor_masks, sspa2_factor_tbl,
				ARRAY_SIZE(sspa2_factor_tbl));
	clk_register_clkdev(clk, "sspa2_mn_div", NULL);

	clk = mmp_clk_register_aud_res("mmp-sspa-dai.1", "sspa2_mn_div",
				audio_aux_base + 0x10, 3, 2, 10,
				0, &clk_lock);
	clk_register_clkdev(clk, NULL, "mmp-sspa-dai.1");

	/* map clock source select */
	clk = clk_register_mux(NULL, "map_mux", map_parent,
			ARRAY_SIZE(map_parent), 0,
			audio_aux_base + 0x2c, 8, 1, 0, &clk_lock);
	clk_set_parent(clk, apll1);
	clk_register_clkdev(clk, "map_mux", NULL);
}
#endif
