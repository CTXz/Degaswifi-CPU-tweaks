/*
 * linux/sound/soc/pxa/pxa-d2199.c
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <sound/pcm_params.h>
#include <linux/pxa2xx_ssp.h>
#include "pxa-ssp.h"

int ssp_master = 1;
static int gssp_master = 1;

/*
 * SSP audio private data
 */
struct ssp_priv {
	struct ssp_device *ssp;
	unsigned int sysclk;
	int dai_fmt;
#ifdef CONFIG_PM
	uint32_t	cr0;
	uint32_t	cr1;
	uint32_t	to;
	uint32_t	psp;
#endif
};

static ssize_t ssp_master_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssp_master);
}

static ssize_t ssp_master_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &ssp_master);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(ssp_master, 0644, ssp_master_show, ssp_master_set);

static ssize_t gssp_master_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gssp_master);
}

static ssize_t gssp_master_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &gssp_master);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(gssp_master, 0644, gssp_master_show, gssp_master_set);

static int pxa_d2199_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	cpu_dai->driver->playback.rates = SNDRV_PCM_RATE_44100;
	cpu_dai->driver->capture.rates = SNDRV_PCM_RATE_44100;

	return 0;
}

static int pxa_d2199_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	if (ssp_master)
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	else
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);

	return 0;
}

static int pxa_d2199_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0, sscr1;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);

	if (ssp_master)
		sscr1 = (sscr1 & (~0x03000000)) | 0x00b01DC0;
	else
		sscr1 |= 0x03b01DC0;

	__raw_writel(sscr0 | 0x41D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);

	return 0;
}

static int pxa_d2199_lofi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* support narrow band: 8K */
	cpu_dai->driver->playback.rates = SNDRV_PCM_RATE_8000;
	cpu_dai->driver->capture.rates = SNDRV_PCM_RATE_8000;
	codec_dai->driver->playback.channels_max = 1;
	codec_dai->driver->capture.channels_max = 1;

	return 0;
}

static int pxa_d2199_dummy_nb_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* support both 8K*/
	cpu_dai->driver->playback.rates = SNDRV_PCM_RATE_8000;
	cpu_dai->driver->capture.rates = SNDRV_PCM_RATE_8000;
	codec_dai->driver->playback.channels_max = 1;
	codec_dai->driver->capture.channels_max = 1;
	codec_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S32_LE;
	codec_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE;

	return 0;
}

static int pxa_d2199_dummy_wb_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* support both 16K*/
	cpu_dai->driver->playback.rates = SNDRV_PCM_RATE_16000;
	cpu_dai->driver->capture.rates = SNDRV_PCM_RATE_16000;
	codec_dai->driver->playback.channels_max = 1;
	codec_dai->driver->capture.channels_max = 1;
	codec_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S32_LE;
	codec_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE;

	return 0;
}

static int pxa_d2199_lofi_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);

	/*
	 * gssp(ssp-dai.4) is shared by AP and CP. AP would read the gssp
	 * register configured by CP after voice call. it would impact DMA
	 * configuration. AP and CP use different GSSP configuration, and
	 * CP's configuration would set DMA to 16bit width while AP set it
	 * to 32 bit. so we need to re-init GSSP register setting.
	 */
	if (!(sscr0 & SSCR0_SSE)) {
		__raw_writel(0x0, ssp->mmio_base + SSCR0);
		__raw_writel(0x0, ssp->mmio_base + SSCR1);
		__raw_writel(0x0, ssp->mmio_base + SSPSP);
		__raw_writel(0x0, ssp->mmio_base + SSTSA);
	}

#if 0
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#endif

	return 0;
}

static int pxa_d2199_lofi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct ssp_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *ssp = priv->ssp;
	u32 sscr0, sscr1;

	sscr0 = __raw_readl(ssp->mmio_base + SSCR0);
	sscr1 = __raw_readl(ssp->mmio_base + SSCR1);
	if (gssp_master)
		sscr1 = (sscr1 & (~0x03000000)) | 0x60b01C01;
	else
		sscr1 |= 0x63301C01;

	__raw_writel(sscr0 | 0xC0C0001F, ssp->mmio_base + SSCR0);
	__raw_writel(sscr1, ssp->mmio_base + SSCR1);
	__raw_writel(0x1, ssp->mmio_base + SSPSP);
	__raw_writel(0x1, ssp->mmio_base + SSTSA);
	__raw_writel(0x1, ssp->mmio_base + SSRSA);

	return 0;
}

static struct snd_soc_ops pxa_d2199_machine_ops[] = {
{
	.startup = pxa_d2199_hifi_startup,
	.hw_params = pxa_d2199_hw_params,
	.prepare = pxa_d2199_hifi_prepare,
},
{
	.startup = pxa_d2199_lofi_startup,
	.hw_params = pxa_d2199_lofi_hw_params,
	.prepare = pxa_d2199_lofi_prepare,
},
{
	.startup = pxa_d2199_dummy_nb_startup,
	.hw_params = pxa_d2199_lofi_hw_params,
	.prepare = pxa_d2199_lofi_prepare,
},
{
	.startup = pxa_d2199_dummy_wb_startup,
	.hw_params = pxa_d2199_lofi_hw_params,
	.prepare = pxa_d2199_lofi_prepare,
},
};

static struct snd_soc_dai_link pxa_d2199_dai[] = {
{
	.name = "d2199 i2s",
	.stream_name = "mrvl-ssp",
	.codec_name = "d2199-codec.2-0018",
	.platform_name = "pxa-ssp-dai.1",
	.cpu_dai_name = "pxa-ssp-dai.1",
	.codec_dai_name = "d2199-i2s",
	.ops = &pxa_d2199_machine_ops[0],
},
{
	.name = "d2199 pcm",
	.stream_name = "mrvl-gssp",
	.codec_name = "d2199-codec.2-0018",
	.platform_name = "pxa-ssp-dai.2",
	.cpu_dai_name = "pxa-ssp-dai.2",
	.codec_dai_name = "d2199-pcm",
	.ops = &pxa_d2199_machine_ops[1],
},
{
	.name = "d2199 dummy nb",
	.stream_name = "mrvl-gssp-nb",
	.codec_name = "d2199-codec.2-0018",
	.platform_name = "pxa-ssp-dai.2",
	.cpu_dai_name = "pxa-ssp-dai.2",
	.codec_dai_name = "d2199-dummy",
	.ops = &pxa_d2199_machine_ops[2],
},
{
	.name = "d2199 dummy wb",
	.stream_name = "mrvl-gssp-wb",
	.codec_name = "d2199-codec.2-0018",
	.platform_name = "pxa-ssp-dai.2",
	.cpu_dai_name = "pxa-ssp-dai.2",
	.codec_dai_name = "d2199-dummy",
	.ops = &pxa_d2199_machine_ops[3],
},
};

static struct snd_soc_card pxa_d2199_card = {
	.name = "pxa-d2199-hifi",
	.dai_link = pxa_d2199_dai,
	.num_links = ARRAY_SIZE(pxa_d2199_dai),
};

#ifdef CONFIG_PM
static int pxa_d2199_suspend(struct device *dev)
{
	return 0;
}

static int pxa_d2199_resume(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(pxa_d2199_pm_ops, pxa_d2199_suspend,
			 pxa_d2199_resume);

static int pxa_d2199_probe_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssp_np[2];
	int i, ret = 0;

	if (!np)
		return 1; /* no device tree */

	ssp_np[0] = of_parse_phandle(np, "ssp-controllers", 0);
	ssp_np[1] = of_parse_phandle(np, "ssp-controllers", 1);
	if (!ssp_np[0] || !ssp_np[1]) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		/* now, we don't use get codec node from dt*/
		pxa_d2199_dai[i].cpu_dai_name = NULL;
		pxa_d2199_dai[i].cpu_of_node = ssp_np[i];
		pxa_d2199_dai[i].platform_name = NULL;
		pxa_d2199_dai[i].platform_of_node = ssp_np[i];
	}

	of_node_put(ssp_np[0]);
	of_node_put(ssp_np[1]);

	return ret;
}

static struct of_device_id pxa_d2199_dt_ids[] = {
	{ .compatible = "marvell,pxa-d2199-snd-card", },
	{}
};
MODULE_DEVICE_TABLE(of, pxa_d2199_dt_ids);

static int pxa_d2199_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &pxa_d2199_card;
	int ret;

	ret = pxa_d2199_probe_dt(pdev);
	if (ret < 0)
		return ret;

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);

	/* add ssp_master sysfs entries */
	ret = device_create_file(&pdev->dev, &dev_attr_ssp_master);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s: failed to add ssp_master sysfs files: %d\n",
			__func__, ret);
		goto err;
	}

	/* add gssp_master sysfs entries */
	ret = device_create_file(&pdev->dev, &dev_attr_gssp_master);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s: failed to add gssp_master sysfs files: %d\n",
			__func__, ret);
		goto err;
	}
	return ret;
err:
	snd_soc_unregister_card(card);
	return ret;
}

static int  pxa_d2199_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_ssp_master);
	device_remove_file(&pdev->dev, &dev_attr_gssp_master);
	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver pxa_d2199_driver = {
	.driver		= {
		.name	= "pxa-d2199-hifi",
		.owner	= THIS_MODULE,
		.of_match_table = pxa_d2199_dt_ids,
		.pm		= &pxa_d2199_pm_ops,
	},
	.probe		= pxa_d2199_probe,
	.remove		= pxa_d2199_remove,
};

module_platform_driver(pxa_d2199_driver);

/* Module information */
MODULE_AUTHOR("zhouqiao@marvell.com");
MODULE_DESCRIPTION("ALSA SoC TTC DKB");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa-d2199-hifi");
