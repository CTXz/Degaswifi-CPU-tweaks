/*
 * linux/sound/soc/pxa/mmp-map-card.c
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
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include "mmp-tdm.h"
#include <linux/delay.h>

#define MAPASOC_SAMPLE_RATES SNDRV_PCM_RATE_44100
/* SSPA and sysclk pll sources */
#define SSPA_AUDIO_PLL                          0
#define SSPA_I2S_PLL                            1
#define SSPA_VCXO_PLL                           2
#define AUDIO_PLL                               3

static int mapasoc_88pm860_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	cpu_dai->driver->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->driver->playback.rates = MAPASOC_SAMPLE_RATES;
	cpu_dai->driver->capture.rates = MAPASOC_SAMPLE_RATES;

	return 0;
}

static int mapasoc_88pm860_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;

	freq_in = 32768;
	if (params_rate(params) > 11025) {
		freq_out = params_rate(params) * 128;
		sysclk = 11289600;
		sspa_mclk = params_rate(params) * 64;
	} else {
		freq_out = params_rate(params) * 64;
		sysclk = 11289600;
		sspa_mclk = params_rate(params) * 64;
	}
	sspa_div = freq_out;
	do_div(sspa_div, sspa_mclk);

	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	/* SSPA clock ctrl register changes, and can't use previous API */

	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);
	/* sysclk */
	snd_soc_dai_set_pll(cpu_dai, SSPA_AUDIO_PLL, 0, freq_out, sysclk);
	/* ssp clk */
	snd_soc_dai_set_pll(cpu_dai, SSPA_I2S_PLL, 0, freq_out, sspa_mclk);

	/* set 88pm860 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 1);

	return 0;
}

static int mapasoc_fe_hw_params(struct snd_pcm_substream *substream,
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

#if 1
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#else
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif

	/* SSPA clock ctrl register changes, and can't use previous API */
	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);
	snd_soc_dai_set_clkdiv(cpu_dai, 0, 0);

	/* set i2s1/2/3/4 sysclk */
	snd_soc_dai_set_sysclk(codec_dai, AUDIO_PLL, freq_out, 0);
	/* set the interface to 44.1k sample rate */
	snd_soc_dai_set_clkdiv(codec_dai, 0, 0x13721fbd);

	return 0;
}

static int mapasoc_be_dei2s_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;
	/*
	 * Now, hard code here, suppose change it according to channel num
	 * the channel allocation..
	 */
	unsigned int if1_tx_channel[2] = {3, 1};
	unsigned int if1_rx_channel[2] = {7, 5};
	unsigned int if2_tx_channel[2] = {2, 0};
	unsigned int if2_rx_channel[2] = {6, 4};

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

#if 0
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
#else
	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
#endif

	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);
	/* set the interface to 44.1k sample rate */
	snd_soc_dai_set_clkdiv(cpu_dai, 0, 0x13721fbd);
	/*
	 * set slot for both cpu_dai and codec dai,
	 * enable L1/R1 channel
	 */
	if (cpu_dai->id == 2) {
		snd_soc_dai_set_channel_map(cpu_dai,
					2, if1_tx_channel, 2, if1_rx_channel);
		snd_soc_dai_set_channel_map(codec_dai,
					2, if1_rx_channel, 2, if1_tx_channel);
	} else if (cpu_dai->id == 3) {
		snd_soc_dai_set_channel_map(cpu_dai,
					2, if2_tx_channel, 2, if2_rx_channel);
		snd_soc_dai_set_channel_map(codec_dai,
					2, if2_rx_channel, 2, if2_tx_channel);
	}

	/* set codec sysclk */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 1);

	return 0;
}

static int mmp_tdm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_RATE);

	rate->min = rate->max = 48000;
	return 0;
}

static int mapasoc_tdm_spkr_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;
	int channel;
#ifdef USE_STATIC_SLOT_ALLOC
	int tx[2] = {4, 5};
	int tx_num = 2;
#endif
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

	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);

#ifdef USE_STATIC_SLOT_ALLOC
	mmp_tdm_static_slot_alloc(substream, tx, tx_num, NULL, 0);
	snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
	snd_soc_dai_set_channel_map(codec_dai, 0, NULL, tx_num, tx);
#else
	/*allocate slot*/
	channel = params_channels(params);
	mmp_tdm_request_slot(substream, channel);
#endif
	return 0;
}

void mapasoc_tdm_spkr_shutdown(struct snd_pcm_substream *substream)
{
#ifdef USE_STATIC_SLOT_ALLOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int tx[2] = {0, 0};
	int tx_num = 2;
#endif

#ifdef USE_STATIC_SLOT_ALLOC
	snd_soc_dai_set_channel_map(codec_dai, 0, NULL, tx_num, tx);
	snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
	mmp_tdm_static_slot_free(substream);
#else
	mmp_tdm_free_slot(substream);
#endif
}

static int mapasoc_tdm_hs_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;
	int channel;
#ifdef USE_STATIC_SLOT_ALLOC
	int tx[2] = {1, 2};
	int tx_num = 2;
#endif
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

	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);

#ifdef USE_STATIC_SLOT_ALLOC
	mmp_tdm_static_slot_alloc(substream, tx, tx_num, NULL, 0);
	snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
	snd_soc_dai_set_channel_map(codec_dai, 0, NULL, tx_num, tx);
#else
	/* allocate slot */
	channel = params_channels(params);
	mmp_tdm_request_slot(substream, channel);
#endif
	return 0;
}

void mapasoc_tdm_hs_shutdown(struct snd_pcm_substream *substream)
{
#ifdef USE_STATIC_SLOT_ALLOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int tx[2] = {0, 0};
	int tx_num = 2;
#endif

#ifdef USE_STATIC_SLOT_ALLOC
	snd_soc_dai_set_channel_map(codec_dai, 0, NULL, tx_num, tx);
	snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
	mmp_tdm_static_slot_free(substream);
#else
	mmp_tdm_free_slot(substream);
#endif
}

static int mapasoc_tdm_mic_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int freq_in, freq_out, sspa_mclk, sysclk, sspa_div;
	int channel;
#ifdef USE_STATIC_SLOT_ALLOC
	int tx[1] = {3};
	int tx_num = 1;
#endif
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

	snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);

	snd_soc_dai_set_sysclk(cpu_dai, AUDIO_PLL, freq_out, 0);

#ifdef USE_STATIC_SLOT_ALLOC
	mmp_tdm_static_slot_alloc(substream, 0, NULL, tx, tx_num);
	snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, tx_num, tx);
	snd_soc_dai_set_channel_map(codec_dai, tx_num, tx, 0, NULL);
#else
	/* allocate slot */
	channel = params_channels(params);
	mmp_tdm_request_slot(substream, channel);
#endif
	return 0;
}

void mapasoc_tdm_mic_shutdown(struct snd_pcm_substream *substream)
{
#ifdef USE_STATIC_SLOT_ALLOC
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int tx[1] = {0};
	int tx_num = 1;
#endif

#ifdef USE_STATIC_SLOT_ALLOC
	snd_soc_dai_set_channel_map(codec_dai, tx_num, tx, 0, NULL);
	snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, tx_num, tx);
	mmp_tdm_static_slot_free(substream);
#else
	mmp_tdm_free_slot(substream);
#endif
}

/* machine stream operations */
static struct snd_soc_ops mapasoc_machine_ops[] = {
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_88pm860_hw_params,
	},
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_fe_hw_params,
	},
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_be_dei2s_hw_params,
	},
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_tdm_spkr_hw_params,
	 .shutdown = mapasoc_tdm_spkr_shutdown,
	},
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_tdm_mic_hw_params,
	 .shutdown = mapasoc_tdm_mic_shutdown,
	},
	{
	 .startup = mapasoc_88pm860_startup,
	 .hw_params = mapasoc_tdm_hs_hw_params,
	 .shutdown = mapasoc_tdm_hs_shutdown,
	},

};

/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link map_asoc_88pm860_dai[] = {
#ifdef CONFIG_MAP_BYPASS
	/* legacy audio: sspa1 */
	{
		.name = "88CE170",
		.stream_name = "88pm860 Audio",
		.platform_name = "mmp-sspa-dai.0",
		.cpu_dai_name = "mmp-sspa-dai.0",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-hifi-dai",
		.ops = &mapasoc_machine_ops[0],
	},
	/* legacy audio: sspa2 */
	{
		.name = "sspa2 playback",
		.stream_name = "88pm860 sspa2 playback",
		.platform_name = "mmp-sspa-dai.1",
		.cpu_dai_name = "mmp-sspa-dai.1",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-voice-dai",
		.ops = &mapasoc_machine_ops[0],
	},
#else
	/* MAP FE */
	{
		.name = "MAP I2S1 audio",
		.stream_name = "low power audio",
		.platform_name = "mmp-sspa-dai.0",
		.cpu_dai_name = "mmp-sspa-dai.0",
		.codec_name = "mmp-map-codec",
		.codec_dai_name = "map-i2s1-dai",
		.ops = &mapasoc_machine_ops[1],

		.dynamic = 1, /* BE is dynamic */
	},
	{
		.name = "MAP I2S2 audio",
		.stream_name = "Low latency audio",
		.platform_name = "mmp-sspa-dai.1",
		.cpu_dai_name = "mmp-sspa-dai.1",
		.codec_name = "mmp-map-codec",
		.codec_dai_name = "map-i2s2-dai",
		.ops = &mapasoc_machine_ops[1],

		.dynamic = 1, /* BE is dynamic */
	},
	/* MAP BE */
#if 0
	{
		.name = "MAP DEI2S IF1 audio",
		.stream_name = "codec if1 audio",
		.cpu_dai_name = "map-be-dei2s-if1",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-mic1",
		.ops = &mapasoc_machine_ops[2],

		.no_pcm = 1, /* don't create ALSA pcm for this */
	},
	{
		.name = "MAP DEI2S IF2 audio",
		.stream_name = "codec if2 audio",
		.cpu_dai_name = "map-be-dei2s-if2",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-mic2",
		.ops = &mapasoc_machine_ops[2],

		.no_pcm = 1, /* don't create ALSA pcm for this */
	},
#endif
	{
		.name = "MAP AUXI2S audio",
		.stream_name = "BT audio",
		.cpu_dai_name = "map-be-aux-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ops = &mapasoc_machine_ops[1],

		.no_pcm = 1, /* don't create ALSA pcm for this */
	},
	{
		.name = "MAP TDM hs audio",
		.stream_name = "codec hs audio",
		.cpu_dai_name = "tdm-out1",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-out1",
		.ops = &mapasoc_machine_ops[5],

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mmp_tdm_be_hw_params_fixup,
	},
	{
		.name = "MAP TDM speaker audio",
		.stream_name = "codec speaker audio",
		.cpu_dai_name = "tdm-out2",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-out2",
		.ops = &mapasoc_machine_ops[3],

		.no_pcm = 1, /* don't create ALSA pcm for this */
		.be_hw_params_fixup = mmp_tdm_be_hw_params_fixup,
	},
	{
		.name = "MAP TDM mic1 audio",
		.stream_name = "codec mic1 audio",
		.cpu_dai_name = "tdm-codec-mic1",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-mic1",
		.ops = &mapasoc_machine_ops[4],

		.no_pcm = 1, /* don't create ALSA pcm for this */
	},
	{
		.name = "MAP TDM mic2 audio",
		.stream_name = "codec mic2 audio",
		.cpu_dai_name = "tdm-codec-mic2",
		.codec_name = "88pm860-codec",
		.codec_dai_name = "88pm860-tdm-mic2",
		.ops = &mapasoc_machine_ops[4],

		.no_pcm = 1, /* don't create ALSA pcm for this */
	},
#endif
};

static const struct snd_soc_dapm_route map_interface_intercon[] = {

	{"ADC input1", NULL, "TDM_MIC1_CAP"},
	{"ADC input2", NULL, "TDM_MIC2_CAP"},
	{"ADC input3", NULL, "BT_VC_UL"},

	{"TDM_OUT1_PLAYBACK", NULL, "DAC1 output out1"},
	{"TDM_OUT2_PLAYBACK", NULL, "DAC2 output out2"},
	{"BT_VC_DL", NULL, "ADC output ain3"},
};

/* audio machine driver */
static struct snd_soc_card snd_soc_mapasoc = {
	.name = "map asoc",
	.dapm_routes = map_interface_intercon,
	.num_dapm_routes = ARRAY_SIZE(map_interface_intercon),
	.dai_link = &map_asoc_88pm860_dai[0],
	.num_links = ARRAY_SIZE(map_asoc_88pm860_dai),
};

static int pxa_map_probe_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *sspa_np[2];
	int i, ret = 0;

	if (!np)
		return 1; /* no device tree */

	sspa_np[0] = of_parse_phandle(np, "sspa-controllers", 0);
	sspa_np[1] = of_parse_phandle(np, "sspa-controllers", 1);
	if (!sspa_np[0] || !sspa_np[1]) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		/* now, we don't use get codec node from dt */
		map_asoc_88pm860_dai[i].cpu_dai_name = NULL;
		map_asoc_88pm860_dai[i].cpu_of_node = sspa_np[i];
		map_asoc_88pm860_dai[i].platform_name = NULL;
		map_asoc_88pm860_dai[i].platform_of_node = sspa_np[i];
	}

	of_node_put(sspa_np[0]);
	of_node_put(sspa_np[1]);

	return ret;
}

static struct of_device_id mmp_map_dt_ids[] = {
	{ .compatible = "marvell,mmp-map-snd-card", },
	{}
};
MODULE_DEVICE_TABLE(of, mmp_map_dt_ids);

static int map_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card;
	card = &snd_soc_mapasoc;

	ret = pxa_map_probe_dt(pdev);
	if (ret < 0)
		return ret;

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
				ret);

	return ret;
}

static int map_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

static struct platform_driver map_audio_driver = {
	.driver		= {
		.name	= "marvell-map-audio",
		.owner	= THIS_MODULE,
		.of_match_table = mmp_map_dt_ids,
	},
	.probe		= map_audio_probe,
	.remove		= map_audio_remove,
};

module_platform_driver(map_audio_driver);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC MAP-88PM860");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:map-audio");
