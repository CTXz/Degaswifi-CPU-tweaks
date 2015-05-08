/*
 * linux/sound/soc/pxa/mmp-map-be-dai.c
 * Base on mmp-be-dai.c
 *
 * Copyright (C) 2014 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/mfd/mmp-map.h>

/* DEI2S/AUXI2S audio private data */
struct map_be_dai_private {
	struct device *dev;
	struct map_private *map_priv;
	struct	regmap *regmap;
	int dai_fmt;
	int running_cnt;

	/* dei2s slot configuration */
	unsigned int dei2s_if1_tx[2];
	unsigned int dei2s_if1_rx[2];
	int dei2s_if1_tx_num;
	int dei2s_if1_rx_num;
	unsigned int dei2s_if2_tx[2];
	unsigned int dei2s_if2_rx[2];
	int dei2s_if2_tx_num;
	int dei2s_if2_rx_num;

	/* check if the channel is in used */
	unsigned int ch_bit_map;
};

/* Set SYSCLK */
static int mmp_map_set_be_dai_sysclk(struct snd_soc_dai *cpu_dai,
				    int clk_id, unsigned int freq, int dir)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
	int ret = 0;

	if (cpu_dai->active > 1)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	map_priv = map_be_dai_priv->map_priv;
	switch (clk_id) {
	case MCLK_32K_PLL:
		ret = clk_set_rate(map_priv->audio_clk, freq);
		if (ret)
			return ret;
		break;
	case MCLK_312M:
	case MCLK_26M:
		/* not support yet */
		return -EINVAL;
	default:
		return -EINVAL;
	}

	return ret;
}

/* need to consider use clkdiv or add clock source for these interfaces */
static int mmp_map_set_be_dai_clkdiv(struct snd_soc_dai *cpu_dai,
		int div_id, int div)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
	unsigned int addr;

	if (cpu_dai->active > 1)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	map_priv = map_be_dai_priv->map_priv;
	switch (cpu_dai->id) {
	case 1:
		addr =	MAP_I2S4_BCLK_DIV;
		break;
	case 2:
	case 3:
		addr =	MAP_I2S_OUT_BCLK_DIV;
		break;
	default:
		return -EINVAL;
	}
	/* set dai divider <0:27> */
	map_raw_write(map_priv, addr, div);

	return 0;
}

static int mmp_map_set_be_dai_fmt(struct snd_soc_dai *cpu_dai,
				 unsigned int fmt)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;

	if (cpu_dai->active > 1)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	map_priv = map_be_dai_priv->map_priv;
	switch (cpu_dai->id) {
	case 1:
		addr =	MAP_I2S4_CTRL_REG;
		inf = map_raw_read(map_priv, addr);

		break;
	case 2:
	case 3:
		addr =	MAP_DEI2S_CTRL_REG;
		inf = map_raw_read(map_priv, addr);

		break;
	default:
		return -EINVAL;
	}

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		inf &= ~MAP_DEI2S_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		inf |= MAP_DEI2S_MASTER;
		break;
	default:
		return -EINVAL;
	}

	inf &= ~MAP_DEI2S_MODE_MASK;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		inf |= MAP_DEI2S_MODE_I2S_FORMAT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		inf |= MAP_DEI2S_MODE_RIGHT_JUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		inf |= MAP_DEI2S_MODE_LEFT_JUST;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		inf |= MAP_DEI2S_MODE_PCM_FORMAT;
		inf &= ~MAP_DEI2S_PCM_MODE_B;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		inf |= MAP_DEI2S_MODE_PCM_FORMAT;
		inf |= MAP_DEI2S_PCM_MODE_B;
		break;
	default:
		inf &= ~MAP_I2S_MODE_I2S_FORMAT;
		break;
	}

	map_raw_write(map_priv, addr, inf);
	return 0;
}

static int mmp_map_be_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;

	if (dai->active > 1)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(dai);
	map_priv = map_be_dai_priv->map_priv;
	switch (dai->id) {
	case 1:
		addr =	MAP_I2S4_CTRL_REG;
		inf = map_raw_read(map_priv, addr);

		break;
	case 2:
	case 3:
		addr =	MAP_DEI2S_CTRL_REG;
		inf = map_raw_read(map_priv, addr);

		break;
	default:
		return -EINVAL;
	}
	inf &= ~MAP_DEI2S_WLEN_MASK;
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		inf |= MAP_DEI2S_WLEN_16_BIT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		inf |= MAP_DEI2S_WLEN_20_BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		inf |= MAP_DEI2S_WLEN_24_BIT;
		break;
	default:
		return -EINVAL;
	}
	map_raw_write(map_priv, addr, inf);

	/* sample rate */
	switch (dai->id) {
	case 1:
		map_set_port_freq(map_priv, I2S4, params_rate(params));
		break;
	case 2:
	case 3:
		map_set_port_freq(map_priv, I2S_OUT, params_rate(params));
		break;
	}

	return 0;
}

static int mmp_map_be_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;

	if (dai->active)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(dai);
	map_priv = map_be_dai_priv->map_priv;
	/*
	 * Fixme: need to check whether the register exist
	 * to enable/disable clock
	 */
#if 0
	clk_enable(map_priv->audio_clk);
	clk_enable(map_priv->apb_clk);
	clk_set_rate(map_priv->map_apb_clk, 0x0);
#endif
	switch (dai->id) {
	case 1:
		map_reset_port(map_priv, I2S4);
		break;
	case 2:
	case 3:
		map_reset_port(map_priv, I2S_OUT);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void mmp_map_be_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
#if 0
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
#endif
	if (dai->active)
		return;
#if 0
	map_be_dai_priv = snd_soc_dai_get_drvdata(dai);
	map_priv = map_be_dai_priv->map_priv;
	clk_disable(map_priv->apb_clk);
	clk_disable(map_priv->audio_clk);
#endif
	return;
}

static int mmp_map_be_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct map_be_dai_private *map_be_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;
	int ret = 0, i;
	int tx_num, tx_ch;
	int rx_num, rx_ch;

	map_be_dai_priv = snd_soc_dai_get_drvdata(dai);
	map_priv = map_be_dai_priv->map_priv;
	switch (dai->id) {
	case 1:
		addr =	MAP_I2S4_CTRL_REG;
		break;
	case 2:
	case 3:
		addr =	MAP_DEI2S_CTRL_REG;
		break;
	default:
		return -EINVAL;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * Fixme: maybe I2S_GEN_EN/I2S_REC_EN should be set
		 * in hw_param with other bits
		 */
		if ((dai->id == 1) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			inf |= I2S_GEN_EN;
			inf |= I2S_REC_EN;
			map_raw_write(map_priv, addr, inf);
		} else if ((dai->id == 2) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			tx_num = map_be_dai_priv->dei2s_if1_tx_num;
			for (i = 0; i < tx_num; i++) {
				tx_ch = map_be_dai_priv->
						dei2s_if1_tx[i];
				inf |= 1 << (tx_ch - 1);
			}
			rx_num =  map_be_dai_priv->dei2s_if1_rx_num;
			for (i = 0; i < rx_num; i++) {
				rx_ch = map_be_dai_priv->
						dei2s_if1_rx[i];
				inf |= 1 << (rx_ch - 1);
			}
			map_raw_write(map_priv, addr, inf);
		} else if ((dai->id == 3) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			tx_num = map_be_dai_priv->dei2s_if2_tx_num;
			for (i = 0; i < tx_num; i++) {
				tx_ch = map_be_dai_priv->
						dei2s_if2_tx[i];
				inf |= 1 << (tx_ch - 1);
			}
			rx_num = map_be_dai_priv->dei2s_if2_rx_num;
			for (i = 0; i < rx_num; i++) {
				rx_ch = map_be_dai_priv->
						dei2s_if2_rx[i];
				inf |= 1 << (rx_ch - 1);
			}
			map_raw_write(map_priv, addr, inf);
		}
		/* apply the change */
		map_apply_change(map_priv);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * Fixme: maybe I2S_GEN_EN/I2S_REC_EN should be set
		 * in hw_param with other bits
		 */
		if ((dai->id == 1) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			inf &= ~I2S_GEN_EN;
			inf &= ~I2S_REC_EN;
			map_raw_write(map_priv, addr, inf);
		} else if ((dai->id == 2) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			tx_num = map_be_dai_priv->dei2s_if1_tx_num;
			for (i = 0; i < tx_num; i++) {
				tx_ch = map_be_dai_priv->
						dei2s_if1_tx[i];
				inf &= ~(1 << (tx_ch - 1));
				/* Fixme: clear bit_map here */
				map_be_dai_priv->ch_bit_map &=
						~(1 << (tx_ch - 1));
			}
			rx_num = map_be_dai_priv->dei2s_if1_rx_num;
			for (i = 0; i < rx_num; i++) {
				rx_ch = map_be_dai_priv->
						dei2s_if1_rx[i];
				inf &= ~(1 << (rx_ch - 1));
				/* Fixme: clear bit_map here */
				map_be_dai_priv->ch_bit_map &=
						~(1 << (rx_ch - 1));
			}
			map_raw_write(map_priv, addr, inf);
		} else if ((dai->id == 3) && (dai->active == 1)) {
			inf = map_raw_read(map_priv, addr);
			tx_num = map_be_dai_priv->dei2s_if2_tx_num;
			for (i = 0; i < tx_num; i++) {
				tx_ch = map_be_dai_priv->
						dei2s_if2_tx[i];
				inf &= ~(1 << (tx_ch - 1));
				/* Fixme: clear bit_map here */
				map_be_dai_priv->ch_bit_map &=
						~(1 << (tx_ch - 1));
			}
			rx_num = map_be_dai_priv->dei2s_if2_rx_num;
			for (i = 0; i < rx_num; i++) {
				rx_ch = map_be_dai_priv->
						dei2s_if2_rx[i];
				inf &= ~(1 << (rx_ch - 1));
				/* Fixme: clear bit_map here */
				map_be_dai_priv->ch_bit_map &=
						~(1 << (rx_ch - 1));
			}
			map_raw_write(map_priv, addr, inf);
		}
		/* apply the change */
		map_apply_change(map_priv);

		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int mmp_map_dei2s_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct map_be_dai_private *map_be_dai_priv;
	int i;

	if (dai->active > 1)
		return 0;

	map_be_dai_priv = snd_soc_dai_get_drvdata(dai);
	/* Fixme: 1<->channel 0; 2<->channel 1 */
	for (i = 0; i < tx_num; i++) {
		if (map_be_dai_priv->ch_bit_map & (1 << tx_slot[i])) {
			pr_err("the tx channel[%d] is in used", tx_slot[i]);
			return -EINVAL;
		}
		map_be_dai_priv->ch_bit_map |= 1 << tx_slot[i];
	}
	for (i = 0; i < rx_num; i++) {
		if (map_be_dai_priv->ch_bit_map & (1 << rx_slot[i])) {
			pr_err("the rx channel[%d] is in used", rx_slot[i]);
			return -EINVAL;
		}
		map_be_dai_priv->ch_bit_map |= 1 << rx_slot[i];
	}

	switch (dai->id) {
	case 2:
		for (i = 0; i < tx_num; i++)
			/* Fixme: add 1 for avoiding 0 (initialize value) */
			map_be_dai_priv->dei2s_if1_tx[i] = tx_slot[i] + 1;
		for (i = 0; i < rx_num; i++)
			map_be_dai_priv->dei2s_if1_rx[i] = rx_slot[i] + 1;
		map_be_dai_priv->dei2s_if1_tx_num = tx_num;
		map_be_dai_priv->dei2s_if1_rx_num = rx_num;
		break;
	case 3:
		for (i = 0; i < tx_num; i++)
			map_be_dai_priv->dei2s_if2_tx[i] = tx_slot[i] + 1;
		for (i = 0; i < rx_num; i++)
			map_be_dai_priv->dei2s_if2_rx[i] = rx_slot[i] + 1;
		map_be_dai_priv->dei2s_if2_tx_num = tx_num;
		map_be_dai_priv->dei2s_if2_rx_num = rx_num;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define MMP_MAP_RATES SNDRV_PCM_RATE_8000_192000
#define MMP_MAP_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops mmp_map_be_ops = {
	.startup	= mmp_map_be_startup,
	.shutdown	= mmp_map_be_shutdown,
	.trigger	= mmp_map_be_trigger,
	.hw_params	= mmp_map_be_hw_params,
	.set_sysclk	= mmp_map_set_be_dai_sysclk,
	.set_clkdiv     = mmp_map_set_be_dai_clkdiv,
	.set_fmt	= mmp_map_set_be_dai_fmt,
	.set_channel_map = mmp_map_dei2s_set_channel_map,
};

struct snd_soc_dai_driver mmp_map_be_dais[] = {
	/* map be cpu dai */
	{
		.name = "map-be-aux-dai",
		.id = 1,
		.playback = {
			.stream_name  = "BT_VC_DL",
			.channels_min = 1,
			.channels_max = 128,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "BT_VC_UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_map_be_ops,
	},
	{
		.name = "map-be-dei2s-if1",
		.id = 2,
		.playback = {
			.stream_name  = "DEI2S_IF1_DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "DEI2S_IF1_UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_map_be_ops,
	},
	{
		.name = "map-be-dei2s-if2",
		.id = 3,
		.playback = {
			.stream_name  = "DEI2S_IF2_DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "DEI2S_IF2_UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_map_be_ops,
	}
};

static const struct snd_soc_component_driver mmp_map_be_component = {
	.name           = "mmp-map-be",
};

static int mmp_map_be_dai_probe(struct platform_device *pdev)
{
	struct map_private *map_priv = dev_get_drvdata(pdev->dev.parent);
	struct map_be_dai_private *map_be_dai_priv;
	int ret;

	map_be_dai_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct map_be_dai_private), GFP_KERNEL);
	if (map_be_dai_priv == NULL)
		return -ENOMEM;

	map_be_dai_priv->map_priv = map_priv;
	map_be_dai_priv->regmap = map_priv->regmap;

	map_be_dai_priv->dai_fmt = (unsigned int) -1;
	platform_set_drvdata(pdev, map_be_dai_priv);

	ret = snd_soc_register_component(&pdev->dev, &mmp_map_be_component,
				mmp_map_be_dais, ARRAY_SIZE(mmp_map_be_dais));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register MAP be dai\n");
		return ret;
	}

	return ret;
}

static int mmp_map_be_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static struct platform_driver mmp_map_be_dai_driver = {
	.driver = {
		.name = "mmp-map-be",
		.owner = THIS_MODULE,
	},
	.probe = mmp_map_be_dai_probe,
	.remove = mmp_map_be_dai_remove,
};

module_platform_driver(mmp_map_be_dai_driver);

MODULE_AUTHOR("Nenghua Cao<nhcao@marvell.com>");
MODULE_DESCRIPTION("MMP MAP BE DAI Interface");
