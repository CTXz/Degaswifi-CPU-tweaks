/*
 * linux/sound/soc/pxa/mmp-pcm.c
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_data/dma-mmp_tdma.h>
#include <linux/platform_data/mmp_audio.h>

#include <sound/pxa2xx-lib.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <linux/of.h>

#define MMP_PCM_INFO (SNDRV_PCM_INFO_MMAP |	\
		SNDRV_PCM_INFO_MMAP_VALID |	\
		SNDRV_PCM_INFO_INTERLEAVED |	\
		SNDRV_PCM_INFO_PAUSE |		\
		SNDRV_PCM_INFO_RESUME)

#define MMP_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | \
			 SNDRV_PCM_FMTBIT_S24_LE | \
			 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_pcm_hardware mmp_pcm_hardware[] = {
	{
		.info			= MMP_PCM_INFO,
		.formats		= MMP_PCM_FORMATS,
		.period_bytes_min	= 1024,
		.period_bytes_max	= 2048,
		.periods_min		= 2,
		.periods_max		= 32,
		.buffer_bytes_max	= 4096,
		.fifo_size		= 32,
	},
	{
		.info			= MMP_PCM_INFO,
		.formats		= MMP_PCM_FORMATS,
		.period_bytes_min	= 1024,
		.period_bytes_max	= 2048,
		.periods_min		= 2,
		.periods_max		= 32,
		.buffer_bytes_max	= 4096,
		.fifo_size		= 32,
	},
};

#ifdef CONFIG_SND_PXA_SSP_DUMP
static int ssp_playback_enable;
static int ssp_capture_enable;
#endif

static int mmp_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct dma_slave_config slave_config;
	int ret;

	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, &slave_config);
	if (ret)
		return ret;

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret)
		return ret;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static const char * const dmaengine_pcm_dma_channel_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK] = "tx",
	[SNDRV_PCM_STREAM_CAPTURE] = "rx",
};

static int mmp_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct platform_device *pdev = to_platform_device(rtd->platform->dev);
	struct dma_chan *chan;

	snd_soc_set_runtime_hwparams(substream,
				&mmp_pcm_hardware[substream->stream]);
	chan = dma_request_slave_channel(&pdev->dev,
			dmaengine_pcm_dma_channel_names[substream->stream]);
	return snd_dmaengine_pcm_open(substream, chan);
}

static int mmp_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long off = vma->vm_pgoff;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
		__phys_to_pfn(runtime->dma_addr) + off,
		vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static struct snd_pcm_ops mmp_pcm_ops = {
	.open		= mmp_pcm_open,
	.close		= snd_dmaengine_pcm_close_release_chan,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mmp_pcm_hw_params,
	.trigger	= snd_dmaengine_pcm_trigger,
	.pointer	= snd_dmaengine_pcm_pointer,
	.mmap		= mmp_pcm_mmap,
};

static void mmp_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct platform_device *pdev = to_platform_device(rtd->platform->dev);
	struct device_node *np = pdev->dev.of_node;
	struct gen_pool *gpool = NULL;
	struct snd_dma_buffer *buf;
	int stream;

	if (!gpool) {
		gpool = of_get_named_gen_pool(np, "asram", 0);
		if (!gpool)
			return;
	}

	for (stream = 0; stream < 2; stream++) {
		size_t size = mmp_pcm_hardware[stream].buffer_bytes_max;

		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;
		gen_pool_free(gpool, (unsigned long)buf->area, size);
		buf->area = NULL;
	}

	return;
}

static int mmp_pcm_preallocate_dma_buffer(struct snd_pcm_substream *substream,
								int stream)
{
	struct snd_pcm *pcm = substream->pcm;
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct platform_device *pdev = to_platform_device(rtd->platform->dev);
	struct device_node *np = pdev->dev.of_node;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mmp_pcm_hardware[stream].buffer_bytes_max;
	struct gen_pool *gpool;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = substream->pcm->card->dev;
	buf->private_data = NULL;

	/* Get sram pool from device tree or platform data */
	gpool = of_get_named_gen_pool(np, "asram", 0);
	if (!gpool)
		return -ENOMEM;

	buf->area = (unsigned char *)gen_pool_alloc(gpool, size);
	if (!buf->area)
		return -ENOMEM;
	buf->addr = gen_pool_virt_to_phys(gpool, (unsigned long)buf->area);
	buf->bytes = size;
	return 0;
}

static int mmp_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0, stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;

		ret = mmp_pcm_preallocate_dma_buffer(substream,	stream);
		if (ret)
			goto err;
	}

	return 0;

err:
	mmp_pcm_free_dma_buffers(pcm);
	return ret;
}

static struct snd_soc_platform_driver mmp_soc_platform = {
	.ops		= &mmp_pcm_ops,
	.pcm_new	= mmp_pcm_new,
	.pcm_free	= mmp_pcm_free_dma_buffers,
};

#ifdef CONFIG_SND_PXA_SSP_DUMP
static ssize_t ssp_playback_dump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssp_playback_enable);
}

static ssize_t ssp_playback_dump_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &ssp_playback_enable);
	snd_pcm_enable_playback_dump(ssp_playback_enable);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(ssp_playback_dump, 0644,
		ssp_playback_dump_show, ssp_playback_dump_set);

static ssize_t ssp_capture_dump_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssp_capture_enable);
}

static ssize_t ssp_capture_dump_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int ret;

	ret = kstrtoint(buf, 10, &ssp_capture_enable);
	snd_pcm_enable_capture_dump(ssp_capture_enable);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(ssp_capture_dump, 0644,
		ssp_capture_dump_show, ssp_capture_dump_set);
#endif

int mmp_pcm_platform_register(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 playback_period_bytes_max;
	u32 playback_buffer_bytes_max;
	u32 capture_period_bytes_max;
	u32 capture_buffer_bytes_max;
	int ret;

	ret = of_property_read_u32(np, "playback_period_bytes",
			&playback_period_bytes_max);
	if (ret >= 0)
		mmp_pcm_hardware[0].period_bytes_max =
					playback_period_bytes_max;
	ret = of_property_read_u32(np, "playback_buffer_bytes",
			&playback_buffer_bytes_max);
	if (ret >= 0)
		mmp_pcm_hardware[0].buffer_bytes_max =
					playback_buffer_bytes_max;
	ret = of_property_read_u32(np, "capture_period_bytes",
			&capture_period_bytes_max);
	if (ret >= 0)
		mmp_pcm_hardware[1].period_bytes_max =
					capture_period_bytes_max;
	ret = of_property_read_u32(np, "capture_buffer_bytes",
			&capture_buffer_bytes_max);
	if (ret >= 0)
		mmp_pcm_hardware[1].buffer_bytes_max =
					capture_buffer_bytes_max;

#ifdef CONFIG_SND_PXA_SSP_DUMP
	/* add ssp_playback_dump sysfs entries */
	ret = device_create_file(dev, &dev_attr_ssp_playback_dump);
	if (ret < 0)
		dev_err(dev,
			"%s: failed to add ssp_playback_dump sysfs files: %d\n",
			__func__, ret);

	/* add ssp_capture_dump sysfs entries */
	ret = device_create_file(dev, &dev_attr_ssp_capture_dump);
	if (ret < 0)
		dev_err(dev,
			"%s: failed to add ssp_capture_dump sysfs files: %d\n",
			__func__, ret);
#endif

	return snd_soc_register_platform(dev, &mmp_soc_platform);
}
EXPORT_SYMBOL_GPL(mmp_pcm_platform_register);

void mmp_pcm_platform_unregister(struct device *dev)
{
#ifdef CONFIG_SND_PXA_SSP_DUMP
	device_remove_file(dev, &dev_attr_ssp_playback_dump);
	device_remove_file(dev, &dev_attr_ssp_capture_dump);
#endif
	snd_soc_unregister_platform(dev);
}
EXPORT_SYMBOL_GPL(mmp_pcm_platform_unregister);

MODULE_LICENSE("GPL");
