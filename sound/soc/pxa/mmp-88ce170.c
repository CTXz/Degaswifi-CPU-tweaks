/*
 * linux/sound/soc/pxa/mmp-88ce170.c
 *
 * Copyright (C) 2013 Marvell International Ltd.
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
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <linux/io.h>
#include <linux/uaccess.h>

#include "../codecs/88ce170-codec.h"
#include "mmp-sspa.h"
#include <linux/delay.h>


#define EDENASOC_SAMPLE_RATES SNDRV_PCM_RATE_44100

static int sspa_master = 1;

static DEVICE_INT_ATTR(ssp_master, 0644, sspa_master);

static int mmp_88ce170_ce170_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->playback.rates = EDENASOC_SAMPLE_RATES;
	cpu_dai->driver->capture.rates = EDENASOC_SAMPLE_RATES;

	return 0;
}

static int mmp_88ce170_ce170_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;

	freq_in = 26000000;
	if (params_rate(params) > 11025) {
		freq_out = params_rate(params) * 512;
		sysclk = 11289600;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out = params_rate(params) * 1024;
		sysclk = 11289600;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

	if (sspa_master) {
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	} else {
		snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	}

	/* SSPA clock ctrl register changes, and can't use previous API */
	snd_soc_dai_set_sysclk(cpu_dai, MMP_SSPA_CLK_AUDIO, freq_out, 0);
	snd_soc_dai_set_pll(cpu_dai, MMP_SYSCLK, 0, freq_out, sysclk);
	snd_soc_dai_set_pll(cpu_dai, MMP_SSPA_CLK, 0, sysclk, sspa_mclk);

	/* set ce170 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 1);

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops mmp_88ce170_machine_ops[] = {
	{
	 .startup = mmp_88ce170_ce170_startup,
	 .hw_params = mmp_88ce170_ce170_hw_params,
	 },
};

#ifdef CONFIG_PM_SLEEP
static int edenasoc_suspend(struct device *dev)
{
	snd_soc_suspend(dev);

	return 0;
}

static int edenasoc_resume(struct device *dev)
{
	snd_soc_resume(dev);

	return 0;
}
static UNIVERSAL_DEV_PM_OPS(edenasoc_pm_ops, edenasoc_suspend,
			 edenasoc_resume, NULL);
#endif

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link mmp_88ce170_asoc_ce170_dai[] = {
	{
	 .name = "88CE170",
	 .stream_name = "88ce170 Audio",
	 .codec_name = "88ce170-codec",
	 .platform_name = "mmp-sspa-dai.0",
	 .cpu_dai_name = "mmp-sspa-dai.0",
	 .codec_dai_name = "88ce170-hifi-dai",
	 .ops = &mmp_88ce170_machine_ops[0],
	 },
};

/* audio machine driver */
static struct snd_soc_card snd_soc_mmp_88ce170 = {
	 .name = "mmp 88ce170 asoc",
	 .dai_link = &mmp_88ce170_asoc_ce170_dai[0],
	 .num_links = 1,
};

static struct of_device_id mmp_88ce170_dt_ids[] = {
	{ .compatible = "marvell,mmp-88ce170-snd-card", },
	{}
};
MODULE_DEVICE_TABLE(of, mmp_88ce170_dt_ids);

static int mmp_88ce170_probe_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *sspa_np;
	int ret = 0;

	if (!np)
		return 1; /* no device tree */

	sspa_np = of_parse_phandle(np, "mrvl,sspa-controllers", 0);
	if (!sspa_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	/* now, we don't use get codec node from dt*/
	mmp_88ce170_asoc_ce170_dai[0].cpu_dai_name = NULL;
	mmp_88ce170_asoc_ce170_dai[0].cpu_of_node = sspa_np;
	mmp_88ce170_asoc_ce170_dai[0].platform_name = NULL;
	mmp_88ce170_asoc_ce170_dai[0].platform_of_node = sspa_np;

	of_node_put(sspa_np);

	return ret;
}

static int mmp_88ce170_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card;
	card = &snd_soc_mmp_88ce170;

	ret = mmp_88ce170_probe_dt(pdev);
	if (ret < 0)
		return ret;

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
				ret);

	/* add sspa_master sysfs entries */
	ret = device_create_file(&pdev->dev, &dev_attr_ssp_master.attr);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s: failed to add sspa_master sysfs files: %d\n",
			__func__, ret);
		goto err;
	}

	return ret;
err:
	snd_soc_unregister_card(card);

	return ret;
}

static int mmp_88ce170_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_ssp_master.attr);
	snd_soc_unregister_card(card);
	return 0;
}

static struct platform_driver mmp_88ce170_audio_driver = {
	.driver		= {
		.name	= "mmp-88ce170-audio-hifi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm		= &edenasoc_pm_ops,
#endif
		.of_match_table = mmp_88ce170_dt_ids,
	},
	.probe		= mmp_88ce170_audio_probe,
	.remove		= mmp_88ce170_audio_remove,
};

module_platform_driver(mmp_88ce170_audio_driver);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC CE170 Eden");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mmp-88ce170-audio-hifi");
