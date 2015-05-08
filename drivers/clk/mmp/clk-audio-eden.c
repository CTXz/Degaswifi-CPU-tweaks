/*
 * mmp Audio clock operation source file
 *
 * Copyright (C) 2013 Marvell
 * Zhao Ye <zhaoy@marvell.com>
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

#define SSPA_AUD_CTRL		(0x34)
#define CLOCK_MUXES_CTRL (0x84)
#define DSA_CORE_CLK_RES_CTRL	(0x28)
#define PLL1_CONFIG_REG1	(0x68)
#define PLL1_CONFIG_REG2	(0x6c)
#define PLL1_CONFIG_REG3	(0x70)
#define I2C_CLK_RES_CTRL	(0x74)

/* DSP Audio PLL 1 Config Register 1 */
#define AUD_PLL1_CONF1_PLL_LOCK		(1 << 31)
#define AUD_PLL1_CONF1_ICP(x)		((x) << 27)
#define AUD_PLL1_CONF1_FBDIV(x)		((x) << 18)
#define AUD_PLL1_CONF1_REFDIV(x)	((x) << 9)
#define AUD_PLL1_CONF1_CLK_DET_EN	(1 << 8)
#define AUD_PLL1_CONF1_INTPI(x)		((x) << 6)
#define AUD_PLL1_CONF1_FD_SEL(x)	((x) << 4)
#define AUD_PLL1_CONF1_CTUNE(x)		((x) << 2)
#define AUD_PLL1_CONF1_RESET		(1 << 1)
#define AUD_PLL1_CONF1_PU		(1 << 0)

/* DSP Audio PLL 1 Config Register 2 */
#define AUD_PLL1_CONF2_RESET_INTP_EXT		(1 << 27)
#define AUD_PLL1_CONF2_POSTDIV_AUDIO(x)		((x) << 20)
#define AUD_PLL1_CONF2_FREQ_OFFSET(x)		((x) << 4)
#define AUD_PLL1_CONF2_FREQ_OFFSET_EXT		(1 << 3)
#define AUD_PLL1_CONF2_FREQ_OFFSET_VALID	(1 << 2)
#define AUD_PLL1_CONF2_POSTDIV_AUDIO_EN		(1 << 1)
#define AUD_PLL1_CONF2_PI_EN			(1 << 0)

#define to_clk_audio(clk) (container_of(clk, struct clk_audio, clk))

struct eden_clk_audio_pll_table {
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
	struct eden_clk_audio_pll_table *freq_table;
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

struct popular_reference_clock_freq refclock_eden[] = {
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

struct post_divider_phase_interpolator pdiv_eden_8k[] = {
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

struct post_divider_phase_interpolator pdiv_eden_11k[] = {
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

static void audio_subsystem_poweron(struct clk_audio *audio, int pwr_on)
{
	unsigned int reg_audio_rstctrl, reg_dsa_rstctrl;
	unsigned int reg_aud_isld_ctrl, reg_aud_isld_sram_pwdn;

	unsigned int reg_audio_i2c_res_ctrl, reg_dspaux_res_ctrl;

	reg_audio_rstctrl = __raw_readl(audio->apmu_base + CLK_RES_CTRL);
#ifdef CONFIG_TZ_HYPERVISOR
	void *addr;
#endif
	if (pwr_on) {
		/*check if Audio already power on, if yes then return*/
		if ((reg_audio_rstctrl &
			((0x3<<9)|(0x1<<8))) == ((0x3<<9)|(0x1<<8))) {
			/* Audio is powered up, return */
			return;
		}

		/*Audio island slow ramp up 1*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(3<<9);
		reg_audio_rstctrl |= (1<<9);
		__raw_writel(reg_audio_rstctrl,
			audio->apmu_base + CLK_RES_CTRL);
		udelay(10);

		/*Audio island power up */
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (3<<9);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*audio sram slow ramp up*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<0);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/*audio sram power up DTCM*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<0);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/*audio core sram slow ramp up*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<2);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/*audio core sram power up ITCM*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<2);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/*???need to find what this if for DTCM*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (1<<12);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
		/*???need to find what this if for DTCM*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
			ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn |= (3<<12);
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);

		/*Audio island isolation disabled*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<8);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*Audio island memory redundancy start*/
		__raw_writel((__raw_readl(audio->apmu_base + CLK_RES_CTRL) |
			(1<<2)), audio->apmu_base + CLK_RES_CTRL);
		/*Check for repair to finish*/
		while (__raw_readl(audio->apmu_base + CLK_RES_CTRL) & (1<<2))
			;
		udelay(50);

		/*audio AXI and APB out of reset*/
		reg_dsa_rstctrl = __raw_readl(audio->apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl |= (1<<0)|(1<<2);
		__raw_writel(reg_dsa_rstctrl, audio->apmu_base +
			DSA_CLK_RES_CTRL);
		udelay(10);

		/*Enable dummy clocks to Audio SRAM*/
		reg_aud_isld_ctrl = __raw_readl(audio->apmu_base +
			ISLD_AU_CTRL);
		reg_aud_isld_ctrl |= (1<<4);
		__raw_writel(reg_aud_isld_ctrl, audio->apmu_base +
			ISLD_AU_CTRL);

		reg_aud_isld_ctrl = __raw_readl(audio->apmu_base +
			ISLD_AU_CTRL);
		reg_aud_isld_ctrl &= ~(1<<4);
		__raw_writel(reg_aud_isld_ctrl, audio->apmu_base +
			ISLD_AU_CTRL);

		/*audio AXI and APB clock enable*/
		reg_dsa_rstctrl = __raw_readl(audio->apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl |= (1<<1)|(1<<3)|(1<<4)|(1<<5);
		__raw_writel(reg_dsa_rstctrl, audio->apmu_base +
			DSA_CLK_RES_CTRL);

		/*Audio bus peripheral clock enable*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<4);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);

		/*Audio bus peripheral out of reset*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl |= (1<<1);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*DSP core\axi reset and clock enable*/
		reg_dspaux_res_ctrl  = __raw_readl(audio->aud_aux_base +
			DSA_CORE_CLK_RES_CTRL);
		reg_dspaux_res_ctrl  |= (1<<3);	/* divider enable */
		__raw_writel(reg_dspaux_res_ctrl,
			audio->aud_aux_base + DSA_CORE_CLK_RES_CTRL);
		reg_dspaux_res_ctrl  |= (1<<0);	/* release core reset */
		__raw_writel(reg_dspaux_res_ctrl,
			audio->aud_aux_base + DSA_CORE_CLK_RES_CTRL);
		reg_dspaux_res_ctrl  |= (1<<1);	/* release axi reset */
		__raw_writel(reg_dspaux_res_ctrl,
			audio->aud_aux_base + DSA_CORE_CLK_RES_CTRL);

		/*Audio I2C bus reset*/
		reg_audio_i2c_res_ctrl =
			__raw_readl(audio->aud_aux_base + I2C_CLK_RES_CTRL);
		reg_audio_i2c_res_ctrl &= ~(1<<2);	/* select VCTXO */
		reg_audio_i2c_res_ctrl |= (1<<1);	/* divide by 1 */
		reg_audio_i2c_res_ctrl |= (1<<0);	/* release reset */
		__raw_writel(reg_audio_i2c_res_ctrl,
			audio->aud_aux_base + I2C_CLK_RES_CTRL);
		reg_audio_i2c_res_ctrl |= (1<<1);	/* enable clock */
		__raw_writel(reg_audio_i2c_res_ctrl,
			audio->aud_aux_base + I2C_CLK_RES_CTRL);

#ifdef CONFIG_TZ_HYPERVISOR
		addr = ioremap_nocache(0xc0ffd000, 4);
		writel(0x60, addr);
		iounmap(addr);
#endif
	} else {
		/*Check if audio already power off, if yes then return*/
		if ((reg_audio_rstctrl & ((0x3<<9)|(0x1<<8))) == 0x0) {
			/* Audio is powered off, return */
			return;
		}

		/*ISB off*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(1<<8);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*Audio bus peripheral in reset*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(0x1<<1);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*Audio bus peripheral clock disable*/
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(0x1<<4);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);

		/*audio AXI and APB and external clock disable*/
		reg_dsa_rstctrl = __raw_readl(audio->apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl &= ~((1<<1)|(1<<3)|(1<<4)|(1<<5));
		__raw_writel(reg_dsa_rstctrl, audio->apmu_base +
			DSA_CLK_RES_CTRL);

		/*audio AXI and APB out of reset*/
		reg_dsa_rstctrl = __raw_readl(audio->apmu_base +
			DSA_CLK_RES_CTRL);
		reg_dsa_rstctrl &= ~((1<<0)|(1<<2));
		__raw_writel(reg_dsa_rstctrl, audio->apmu_base +
			DSA_CLK_RES_CTRL);
		udelay(10);

		/*disable Audio island power switch */
		reg_audio_rstctrl = __raw_readl(audio->apmu_base +
			CLK_RES_CTRL);
		reg_audio_rstctrl &= ~(3<<9);
		__raw_writel(reg_audio_rstctrl, audio->apmu_base +
			CLK_RES_CTRL);
		udelay(10);

		/*disable power to audio SRAM*/
		reg_aud_isld_sram_pwdn = __raw_readl(audio->apmu_base +
				ISLAND_SRAM_PWR_DWN_CTRL);
		reg_aud_isld_sram_pwdn &= ~((0x3<<0)|(3<<2)|(0x3<<12));
		__raw_writel(reg_aud_isld_sram_pwdn,
			audio->apmu_base + ISLAND_SRAM_PWR_DWN_CTRL);
		udelay(10);
	}
}

static int clk_audio_enable(struct clk_hw *hw)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int val = 0;
	unsigned long flags;

	/* before we open audio island, we need close it first */
	audio_subsystem_poweron(audio, 0);
	audio_subsystem_poweron(audio, 1);

	spin_lock_irqsave(audio->lock, flags);
	val = APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
	      APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP;
	__raw_writel(val, audio->apmu_base + CLK_RES_CTRL);

	audio->enabled = 1;
	spin_unlock_irqrestore(audio->lock, flags);

	return 0;
}

static void clk_audio_disable(struct clk_hw *hw)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int val = 0;
	unsigned long flags;

	spin_lock_irqsave(audio->lock, flags);
	val = __raw_readl(audio->apmu_base + CLK_RES_CTRL);
	val &= ~(APMU_AUDIO_RST_DIS | APMU_AUDIO_ISO_DIS |
		 APMU_AUDIO_CLK_ENA | APMU_AUDIO_PWR_UP);
	__raw_writel(val, audio->apmu_base + CLK_RES_CTRL);

	audio->enabled = 0;
	spin_unlock_irqrestore(audio->lock, flags);

	/* if audio pll is closed, we can close the audio island */
	audio_subsystem_poweron(audio, 0);
}

static long __eden_audio_get_rate_table(struct clk_hw *hw,
		struct eden_clk_audio_pll_table *freq_tbl,
			unsigned long drate, unsigned long prate)
{
	int i, j, map_size;
	struct popular_reference_clock_freq *ref_p;
	struct post_divider_phase_interpolator *div_p;

	if ((drate % 8000) == 0) { /* 8KHz family sample range */
		/*output 147.4560M VCO Clock*/
		div_p = &pdiv_eden_8k[0];
		map_size = ARRAY_SIZE(pdiv_eden_8k);
	} else if ((drate % 11025) == 0) { /* 11KHz family sample range */
		/*output 135.475M VCO Clock*/
		div_p = &pdiv_eden_11k[0];
		map_size = ARRAY_SIZE(pdiv_eden_11k);
	} else {
		pr_err("%s : Incorrect sample rate.....\r\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(refclock_eden); i++) {

		/* find audio pll setting */
		ref_p = &refclock_eden[i];
		if (ref_p->refclk != prate)
			continue;

		/* find audio clk setting */
		for (j = 0; j < map_size; j++) {
			if (div_p->clkout_audio == drate)
				if (div_p->pre_divider ==
					ref_p->freq_intp_in)
					goto found;
			div_p++;
		}
	}

	return -EINVAL;
found:
	freq_tbl->input_rate = prate;
	freq_tbl->postdiv = div_p->postdiv_audio;
	freq_tbl->fbdiv = ref_p->fbdiv;
	freq_tbl->refdiv = ref_p->refdiv;
	freq_tbl->freq_offset = ref_p->freq_offset_0_14_hex;
	freq_tbl->output_rate = drate;

	return drate;
}

static long clk_audio_round_rate(struct clk_hw *hw, unsigned long drate,
		unsigned long *prate)
{
	struct eden_clk_audio_pll_table freq_tbl;

	if (!__eden_audio_get_rate_table(hw, &freq_tbl, drate, *prate))
		return -EINVAL;

	*prate = freq_tbl.input_rate;

	return freq_tbl.output_rate;
}

static unsigned long clk_audio_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	unsigned long fbdiv, refdiv, postdiv, offset;
	unsigned long pll_config1, pll_config2;
	unsigned long freq_intp_out, freq_intp_in;
	struct clk_audio *audio = to_clk_audio(hw);

	/* before touch the pll register, we need audio island is opened */
	if (audio->enabled) {
		pll_config1 = __raw_readl(audio->aud_aux_base +
			PLL1_CONFIG_REG1);
		pll_config2 = __raw_readl(audio->aud_aux_base +
			PLL1_CONFIG_REG2);

		fbdiv = (pll_config1 & (0x1FF<<18))>>18;
		refdiv = (pll_config1 & (0x1FF<<9))>>9;
		postdiv = (pll_config2 & (0x7F<<20))>>20;
		offset = (pll_config2 & (0xFFFF<<4))>>4;
		freq_intp_out = (parent_rate / refdiv * fbdiv);
		freq_intp_in = (freq_intp_out/(1<<19)*offset) + freq_intp_out;

		/* accurate to KHz */
		if (((freq_intp_in/postdiv)/1000) != ((hw->clk->rate)/1000))
			return -EINVAL;
	}

	return hw->clk->rate;
}

/* Configures new clock rate*/
static int clk_audio_set_rate(struct clk_hw *hw, unsigned long drate,
				unsigned long prate)
{
	struct clk_audio *audio = to_clk_audio(hw);
	unsigned int regConfig1, regConfig2, regConfig3;
	unsigned int sspa_div, sysclk_div;
	unsigned int postdiv_audio, clkout_vco_div;
	unsigned int CTUNE = 0x1;	/* 1 unit cap loading */
	unsigned int INTPI = 0x2;
	unsigned int count = 0x200;
	unsigned int reg_dspaux_res_ctrl, reg_dspaux_clk_muxes;
	unsigned int reg_sspa_aud_ctrl;
	struct eden_clk_audio_pll_table freq_tbl;
	unsigned long flags;

	spin_lock_irqsave(audio->lock, flags);
	/*switch clocks to vcxo before programming the source*/
	reg_dspaux_res_ctrl  = __raw_readl(audio->aud_aux_base +
		DSA_CORE_CLK_RES_CTRL);
	reg_dspaux_clk_muxes = __raw_readl(audio->aud_aux_base +
		CLOCK_MUXES_CTRL);
	/*for SSPA and ZSP*/
	reg_dspaux_res_ctrl &= ~(3<<4);
	__raw_writel(reg_dspaux_res_ctrl, audio->aud_aux_base +
		DSA_CORE_CLK_RES_CTRL);
	reg_dspaux_clk_muxes |= (1<<9);		/* switch to VCXO */

	__raw_writel(reg_dspaux_clk_muxes, audio->aud_aux_base +
		CLOCK_MUXES_CTRL);

	if (!__eden_audio_get_rate_table(hw, &freq_tbl, drate, prate))
		return -EINVAL;

	/* hardcoded to output 22.579M or 24.576M */
	postdiv_audio  = freq_tbl.postdiv;
	clkout_vco_div = 1; /* hardcoded to output 541.9008M or 589.824M */

	/* Reset PLL Reset=1 */
	regConfig1 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG1);
	regConfig1 |= (1<<1); /* reset */
	__raw_writel(regConfig1, audio->aud_aux_base + PLL1_CONFIG_REG1);

	/* Disable PLL PU=0 */
	regConfig1 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG1);
	regConfig1 &= ~(1<<0); /* pll disable */
	__raw_writel(regConfig1, audio->aud_aux_base + PLL1_CONFIG_REG1);

	/* Audio PLL config register 1 */
	regConfig1 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG1);
	regConfig1 &= ~(0xF<<27);	/* icp */
	regConfig1 &= ~(0x1FF<<18);	/* fbdiv */
	regConfig1 &= ~(0x1FF<<9);	/* refdiv */
	regConfig1 &= ~(0x3<<6);	/* intpi */
	regConfig1 &= ~(0x3<<2);	/* ctune */
	regConfig1 &= ~(0x3<<4);	/* lock detector accuracy set */

	regConfig1 |= AUD_PLL1_CONF1_ICP(0x6); /* icp */
	regConfig1 |= AUD_PLL1_CONF1_FBDIV(freq_tbl.fbdiv); /* fbdiv */
	regConfig1 |= AUD_PLL1_CONF1_REFDIV(freq_tbl.refdiv); /* refdiv */
	regConfig1 |= (INTPI<<6);	/* intpi */
	regConfig1 |= (CTUNE<<2);	/* ctune */
	regConfig1 |= (1<<8);		/* PI clockout enable */
	regConfig1 |= (1<<4);		/* lock detector accuracy set */

	/* Audio PLL config register 2 */
	regConfig2 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG2);
	regConfig2 &= ~(0x7F<<20);	/* postdiv_audio */
	regConfig2 &= ~(0xFFFF<<4);	/* freq_offset */

	regConfig2 |= AUD_PLL1_CONF2_POSTDIV_AUDIO(freq_tbl.postdiv);
	regConfig2 |= ((freq_tbl.freq_offset)<<4);	/* freq_offset */
	regConfig2 |= (1<<1);		/* postdiv_audio_en */
	regConfig2 |= (1<<0);		/* phase-interpolator enable */
	/* (RESET_INTP_EXT)external interpolator reset signal::no reset */
	regConfig2 &= ~(1<<27);
	/* (RESET_OFFSET_EXT)external offset logic reset signal::no reset */
	regConfig2 &= ~(1<<3);
	/* (FREQ_OFFSET_VALID)frequency offset valid */
	regConfig2 &= ~(1<<2);

	/* Audio PLL config register 3 */
	regConfig3 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG3);
	regConfig3 &= ~(0x1FF<<15);	/* clk_vco_div_sel */

	regConfig3 |= (1<<24);		/* clk_vco_div_en */
	regConfig3 |= (clkout_vco_div<<15);	/* clk_vco_div_sel */
	/* clk_vco_en :: TODO:: need to find if this should be set */
	regConfig3 |= (1<<14);

	__raw_writel(regConfig3, audio->aud_aux_base + PLL1_CONFIG_REG3);

	/* Audio PLL config register 2 */
	__raw_writel(regConfig2, audio->aud_aux_base + PLL1_CONFIG_REG2);

	/* enable PLL PU=1 */
	regConfig1 |= (1<<0);	/* pll disable */
	__raw_writel(regConfig1, audio->aud_aux_base + PLL1_CONFIG_REG1);
	udelay(10);
	/* Release reset */
	regConfig1 &= ~(1<<1);	/* release reset */
	__raw_writel(regConfig1, audio->aud_aux_base + PLL1_CONFIG_REG1);
	udelay(10);
	/* send pulse for freq_offset_valid */
	regConfig2 |= (1<<2);
	__raw_writel(regConfig2, audio->aud_aux_base + PLL1_CONFIG_REG2);
	udelay(10);

	/* check if PLL is locked */
	regConfig1 = __raw_readl(audio->aud_aux_base + PLL1_CONFIG_REG1);
	regConfig1 &= 0x80000000;
	while (!regConfig1) {
		regConfig1 = __raw_readl(audio->aud_aux_base +
			PLL1_CONFIG_REG1);
		regConfig1 &= 0x80000000;
		udelay(10);
		count--;
		if (count <= 0) {
			pr_err("PLL lock timeout error PLL1_CONFIG_REG1g %x\n",
				regConfig1);
			return -1;
		}
	}

	sysclk_div = 2;
	sspa_div = 8;
	/* program the SSPA dividers and source */
	reg_dspaux_clk_muxes = __raw_readl(audio->aud_aux_base +
		CLOCK_MUXES_CTRL);
	reg_sspa_aud_ctrl = __raw_readl(audio->aud_base + SSPA_AUD_CTRL);
	reg_sspa_aud_ctrl &= ~(0x7f<<0);
	reg_sspa_aud_ctrl |= (1<<0)|(sysclk_div<<1);
	reg_sspa_aud_ctrl &= ~(0x7f<<8);
	reg_sspa_aud_ctrl |= (1<<8)|(sspa_div<<9);
	reg_dspaux_clk_muxes &= ~(0x7<<3);
	reg_dspaux_clk_muxes &= ~(1<<9);
	reg_dspaux_clk_muxes &= ~(1<<11);
	reg_dspaux_clk_muxes |= (1<<11);
	__raw_writel(reg_dspaux_clk_muxes, audio->aud_aux_base +
		CLOCK_MUXES_CTRL);
	__raw_writel(reg_sspa_aud_ctrl, audio->aud_base + SSPA_AUD_CTRL);

	reg_dspaux_res_ctrl  = __raw_readl(audio->aud_aux_base +
		DSA_CORE_CLK_RES_CTRL);
	reg_dspaux_res_ctrl |= (1<<6); /* slow clock to audio PLL */
	reg_dspaux_res_ctrl &= ~(3<<4);
	reg_dspaux_res_ctrl |= (3<<4);	/* zsp clock to audio PLL */
	__raw_writel(reg_dspaux_res_ctrl, audio->aud_aux_base +
		DSA_CORE_CLK_RES_CTRL);

	hw->clk->rate = drate;
	spin_unlock_irqrestore(audio->lock, flags);

	return 0;
}

struct clk_ops clk_audio_ops = {
	.enable = clk_audio_enable,
	.disable = clk_audio_disable,
	.recalc_rate = clk_audio_recalc_rate,
	.round_rate = clk_audio_round_rate,
	.set_rate = clk_audio_set_rate,
};

struct clk *mmp_clk_register_audio(const char *name, const char *parent_name,
		void __iomem *apmu_base, void __iomem *aud_base,
		void __iomem *aud_aux_base, u32 enable_mask, spinlock_t *lock)
{
	struct clk_audio *audio;
	struct clk *clk;
	struct clk_init_data init;

	audio = kzalloc(sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return NULL;

	init.name = name;
	init.ops = &clk_audio_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	audio->apmu_base = apmu_base;
	audio->aud_base = aud_base;
	audio->aud_aux_base = aud_aux_base;
	audio->enable_mask = enable_mask;
	audio->lock = lock;
	audio->hw.init = &init;

	clk = clk_register(NULL, &audio->hw);

	if (IS_ERR(clk))
		kfree(audio);

	return clk;
}
