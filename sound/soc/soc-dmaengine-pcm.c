/*
 *  Copyright (C) 2012, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 *  Based on:
 *	imx-pcm-dma-mx2.c, Copyright 2009 Sascha Hauer <s.hauer@pengutronix.de>
 *	mxs-pcm.c, Copyright (C) 2011 Freescale Semiconductor, Inc.
 *	ep93xx-pcm.c, Copyright (C) 2006 Lennert Buytenhek <buytenh@wantstofly.org>
 *		      Copyright (C) 2006 Applied Data Systems
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>
#include <linux/features.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <sound/dmaengine_pcm.h>

#ifdef CONFIG_SND_PXA_SSP_DUMP
static int ssp_playback_enable;
static int ssp_capture_enable;
static struct workqueue_struct *ssp_playback_wq;
static struct workqueue_struct *ssp_capture_wq;

#define DUMP_FILE1	"/data/log/audio/ssp_playback.dump"
#define DUMP_FILE2	"/data/log/audio/ssp_capture.dump"
#endif

struct dmaengine_pcm_runtime_data {
	struct dma_chan *dma_chan;
	dma_cookie_t cookie;

	unsigned int pos;

#ifdef CONFIG_SND_PXA_SSP_DUMP
	u32 playback_dump_addr;
	u32 playback_transfer_addr;
	u32 playback_totsize;
	struct work_struct playback_dump_work;
	struct file *playback_fp;
	struct snd_pcm_runtime *playback_runtime;
	u32 capture_dump_addr;
	u32 capture_transfer_addr;
	u32 capture_totsize;
	struct work_struct capture_dump_work;
	struct file *capture_fp;
	struct snd_pcm_runtime *capture_runtime;
#endif
};

#ifdef CONFIG_SND_PXA_SSP_DUMP
void snd_pcm_enable_playback_dump(int value)
{
	ssp_playback_enable = value;
}

void snd_pcm_enable_capture_dump(int value)
{
	ssp_capture_enable = value;
}
#endif

static inline struct dmaengine_pcm_runtime_data *substream_to_prtd(
	const struct snd_pcm_substream *substream)
{
	return substream->runtime->private_data;
}

struct dma_chan *snd_dmaengine_pcm_get_chan(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);

	return prtd->dma_chan;
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_get_chan);

/**
 * snd_hwparams_to_dma_slave_config - Convert hw_params to dma_slave_config
 * @substream: PCM substream
 * @params: hw_params
 * @slave_config: DMA slave config
 *
 * This function can be used to initialize a dma_slave_config from a substream
 * and hw_params in a dmaengine based PCM driver implementation.
 */
int snd_hwparams_to_dma_slave_config(const struct snd_pcm_substream *substream,
	const struct snd_pcm_hw_params *params,
	struct dma_slave_config *slave_config)
{
	enum dma_slave_buswidth buswidth;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		buswidth = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		if (has_feat_hifi_gssp_record())
			buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES * params_channels(params);
		else
			buswidth = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		buswidth = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config->direction = DMA_MEM_TO_DEV;
		slave_config->dst_addr_width = buswidth;
	} else {
		slave_config->direction = DMA_DEV_TO_MEM;
		slave_config->src_addr_width = buswidth;
	}

	slave_config->device_fc = false;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_hwparams_to_dma_slave_config);

/**
 * snd_dmaengine_pcm_set_config_from_dai_data() - Initializes a dma slave config
 *  using DAI DMA data.
 * @substream: PCM substream
 * @dma_data: DAI DMA data
 * @slave_config: DMA slave configuration
 *
 * Initializes the {dst,src}_addr, {dst,src}_maxburst, {dst,src}_addr_width and
 * slave_id fields of the DMA slave config from the same fields of the DAI DMA
 * data struct. The src and dst fields will be initialized depending on the
 * direction of the substream. If the substream is a playback stream the dst
 * fields will be initialized, if it is a capture stream the src fields will be
 * initialized. The {dst,src}_addr_width field will only be initialized if the
 * addr_width field of the DAI DMA data struct is not equal to
 * DMA_SLAVE_BUSWIDTH_UNDEFINED.
 */
void snd_dmaengine_pcm_set_config_from_dai_data(
	const struct snd_pcm_substream *substream,
	const struct snd_dmaengine_dai_dma_data *dma_data,
	struct dma_slave_config *slave_config)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config->dst_addr = dma_data->addr;
		slave_config->dst_maxburst = dma_data->maxburst;
		if (dma_data->addr_width != DMA_SLAVE_BUSWIDTH_UNDEFINED)
			slave_config->dst_addr_width = dma_data->addr_width;
	} else {
		slave_config->src_addr = dma_data->addr;
		slave_config->src_maxburst = dma_data->maxburst;
		if (dma_data->addr_width != DMA_SLAVE_BUSWIDTH_UNDEFINED)
			slave_config->src_addr_width = dma_data->addr_width;
	}

	slave_config->slave_id = dma_data->slave_id;
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_set_config_from_dai_data);

static void dmaengine_pcm_dma_complete(void *arg)
{
	struct snd_pcm_substream *substream = arg;
	struct dmaengine_pcm_runtime_data *prtd;

	snd_pcm_stream_lock(substream);
	if (!substream || !substream->runtime) {
		snd_pcm_stream_unlock(substream);
		return;
	}

	prtd = substream_to_prtd(substream);
	prtd->pos += snd_pcm_lib_period_bytes(substream);
	if (prtd->pos >= snd_pcm_lib_buffer_bytes(substream))
		prtd->pos = 0;

	snd_pcm_stream_unlock(substream);

	snd_pcm_period_elapsed(substream);

#ifdef CONFIG_SND_PXA_SSP_DUMP
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK
			&& ssp_playback_enable) {
		prtd->playback_transfer_addr =
				substream->runtime->dma_addr + prtd->pos;
		queue_work(ssp_playback_wq, &prtd->playback_dump_work);
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE
			&& ssp_capture_enable) {
		prtd->capture_transfer_addr =
				substream->runtime->dma_addr + prtd->pos;
		queue_work(ssp_capture_wq, &prtd->capture_dump_work);
	}
#endif
}

static int dmaengine_pcm_prepare_and_submit(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct dma_chan *chan = prtd->dma_chan;
	struct dma_async_tx_descriptor *desc;
	enum dma_transfer_direction direction;
	unsigned long flags = DMA_CTRL_ACK;

	direction = snd_pcm_substream_to_dma_direction(substream);

	if (!substream->runtime->no_period_wakeup)
		flags |= DMA_PREP_INTERRUPT;

	prtd->pos = 0;
	desc = dmaengine_prep_dma_cyclic(chan,
		substream->runtime->dma_addr,
		snd_pcm_lib_buffer_bytes(substream),
		snd_pcm_lib_period_bytes(substream), direction, flags);

	if (!desc)
		return -ENOMEM;

	desc->callback = dmaengine_pcm_dma_complete;
	desc->callback_param = substream;
	prtd->cookie = dmaengine_submit(desc);

#ifdef CONFIG_SND_PXA_SSP_DUMP
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK
			&& ssp_playback_enable) {
		prtd->playback_totsize = snd_pcm_lib_buffer_bytes(substream);
		prtd->playback_transfer_addr = substream->runtime->dma_addr;
		prtd->playback_dump_addr = substream->runtime->dma_addr;
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE
			&& ssp_capture_enable) {
		prtd->capture_totsize = snd_pcm_lib_buffer_bytes(substream);
		prtd->capture_transfer_addr = substream->runtime->dma_addr;
		prtd->capture_dump_addr = substream->runtime->dma_addr;
	}
#endif
	return 0;
}

/**
 * snd_dmaengine_pcm_trigger - dmaengine based PCM trigger implementation
 * @substream: PCM substream
 * @cmd: Trigger command
 *
 * Returns 0 on success, a negative error code otherwise.
 *
 * This function can be used as the PCM trigger callback for dmaengine based PCM
 * driver implementations.
 */
int snd_dmaengine_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	int ret;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = dmaengine_pcm_prepare_and_submit(substream);
		if (ret)
			return ret;
		dma_async_issue_pending(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dmaengine_resume(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dmaengine_pause(prtd->dma_chan);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		dmaengine_terminate_all(prtd->dma_chan);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_trigger);

/**
 * snd_dmaengine_pcm_pointer_no_residue - dmaengine based PCM pointer implementation
 * @substream: PCM substream
 *
 * This function is deprecated and should not be used by new drivers, as its
 * results may be unreliable.
 */
snd_pcm_uframes_t snd_dmaengine_pcm_pointer_no_residue(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	return bytes_to_frames(substream->runtime, prtd->pos);
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_pointer_no_residue);

/**
 * snd_dmaengine_pcm_pointer - dmaengine based PCM pointer implementation
 * @substream: PCM substream
 *
 * This function can be used as the PCM pointer callback for dmaengine based PCM
 * driver implementations.
 */
snd_pcm_uframes_t snd_dmaengine_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int buf_size;
	unsigned int pos = 0;

	status = dmaengine_tx_status(prtd->dma_chan, prtd->cookie, &state);
	if (status == DMA_IN_PROGRESS || status == DMA_PAUSED) {
		buf_size = snd_pcm_lib_buffer_bytes(substream);
		if (state.residue > 0 && state.residue <= buf_size)
			pos = buf_size - state.residue;
	}

	return bytes_to_frames(substream->runtime, pos);
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_pointer);

/**
 * snd_dmaengine_pcm_request_channel - Request channel for the dmaengine PCM
 * @filter_fn: Filter function used to request the DMA channel
 * @filter_data: Data passed to the DMA filter function
 *
 * Returns NULL or the requested DMA channel.
 *
 * This function request a DMA channel for usage with dmaengine PCM.
 */
struct dma_chan *snd_dmaengine_pcm_request_channel(dma_filter_fn filter_fn,
	void *filter_data)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_cap_set(DMA_CYCLIC, mask);

	return dma_request_channel(mask, filter_fn, filter_data);
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_request_channel);

#ifdef CONFIG_SND_PXA_SSP_DUMP
void playback_dump_handler(struct work_struct *work)
{
	struct dmaengine_pcm_runtime_data *prtd = container_of(work,
		struct dmaengine_pcm_runtime_data, playback_dump_work);
	struct snd_pcm_runtime *runtime = prtd->playback_runtime;
	mm_segment_t old_fs;
	struct file *f = prtd->playback_fp;
	u32 transfer_addr, size1, size2;
	u32 dump_virt = prtd->playback_dump_addr - (u32)runtime->dma_addr
		+ (u32)runtime->dma_area;
	ssize_t ret;

	if (!f)
		return;

	transfer_addr = prtd->playback_transfer_addr;
	if (transfer_addr > prtd->playback_dump_addr) {
		size1 = transfer_addr - prtd->playback_dump_addr;
		size2 = 0;
	} else {
		size1 = runtime->dma_addr + prtd->playback_totsize
			-  prtd->playback_dump_addr;
		size2 = transfer_addr - runtime->dma_addr;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	ret = f->f_op->write(f, (const char *)dump_virt, size1, &f->f_pos);
	if (ret < 0)
		pr_info("write playback file error %d\n", ret);
	else if (transfer_addr > prtd->playback_dump_addr)
		prtd->playback_dump_addr = prtd->playback_dump_addr + size1;
	else
		prtd->playback_dump_addr = runtime->dma_addr
			+ prtd->playback_totsize - 1;
	if (ret >= 0) {
		if (size2) {
			ret = f->f_op->write(f, runtime->dma_area,
					size2, &f->f_pos);
			if (ret < 0)
				pr_info("write playback file error %d\n", ret);
			else
				prtd->playback_dump_addr =
					runtime->dma_addr + size2;
		} else if (transfer_addr == runtime->dma_addr)
			prtd->playback_dump_addr = runtime->dma_addr;
	}
	set_fs(old_fs);
}

void capture_dump_handler(struct work_struct *work)
{
	struct dmaengine_pcm_runtime_data *prtd = container_of(work,
		struct dmaengine_pcm_runtime_data, capture_dump_work);
	struct snd_pcm_runtime *runtime = prtd->capture_runtime;
	mm_segment_t old_fs;
	struct file *f = prtd->capture_fp;
	u32 transfer_addr, size1, size2;
	u32 dump_virt = prtd->capture_dump_addr - (u32)runtime->dma_addr
		+ (u32)runtime->dma_area;
	ssize_t ret;

	if (!f)
		return;

	transfer_addr = prtd->capture_transfer_addr;
	if (transfer_addr > prtd->capture_dump_addr) {
		size1 = transfer_addr - prtd->capture_dump_addr;
		size2 = 0;
	} else {
		size1 = runtime->dma_addr + prtd->capture_totsize
			-  prtd->capture_dump_addr;
		size2 = transfer_addr - runtime->dma_addr;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ret = f->f_op->write(f, (const char *)dump_virt, size1, &f->f_pos);

	if (ret < 0)
		pr_info("write capture file error %d\n", ret);
	else if (transfer_addr > prtd->capture_dump_addr)
		prtd->capture_dump_addr = prtd->capture_dump_addr + size1;
	else
		prtd->capture_dump_addr = runtime->dma_addr
			+ prtd->capture_totsize - 1;
	if (ret >= 0) {
		if (size2) {
			ret = f->f_op->write(f, runtime->dma_area,
					size2, &f->f_pos);
			if (ret < 0)
				pr_info("write capture file error %d\n", ret);
			else
				prtd->capture_dump_addr =
					runtime->dma_addr + size2;
		} else if (transfer_addr == runtime->dma_addr)
				prtd->capture_dump_addr = runtime->dma_addr;
	}
	set_fs(old_fs);
}
#endif

/**
 * snd_dmaengine_pcm_open - Open a dmaengine based PCM substream
 * @substream: PCM substream
 * @chan: DMA channel to use for data transfers
 *
 * Returns 0 on success, a negative error code otherwise.
 *
 * The function should usually be called from the pcm open callback. Note that
 * this function will use private_data field of the substream's runtime. So it
 * is not availabe to your pcm driver implementation.
 */
int snd_dmaengine_pcm_open(struct snd_pcm_substream *substream,
	struct dma_chan *chan)
{
	struct dmaengine_pcm_runtime_data *prtd;
	int ret;

	if (!chan)
		return -ENXIO;

	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (!prtd)
		return -ENOMEM;

	prtd->dma_chan = chan;

	substream->runtime->private_data = prtd;

#ifdef CONFIG_SND_PXA_SSP_DUMP
	INIT_WORK(&prtd->playback_dump_work, playback_dump_handler);
	INIT_WORK(&prtd->capture_dump_work, capture_dump_handler);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (!ssp_playback_wq)
			ssp_playback_wq =
				create_singlethread_workqueue("ssp_playback");

		pr_debug("[pxa-ssp]: Create playback dump file\n");
		prtd->playback_fp = filp_open(DUMP_FILE1, O_RDWR | O_CREAT
			| O_LARGEFILE, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR(prtd->playback_fp)) {
			pr_info("playback dump file create error %d\n",
				(int)prtd->playback_fp);
			prtd->playback_fp = 0;
		}
		prtd->playback_runtime = substream->runtime;
	} else {
		if (!ssp_capture_wq)
			ssp_capture_wq =
				create_singlethread_workqueue("ssp_capture");

		pr_debug("[pxa-ssp]: Create capture dump file\n");
		prtd->capture_fp = filp_open(DUMP_FILE2, O_RDWR | O_CREAT
			| O_LARGEFILE, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR(prtd->capture_fp)) {
			pr_info("capture dump file create error %d\n",
				(int)prtd->capture_fp);
			prtd->capture_fp = 0;
		}
		prtd->capture_runtime = substream->runtime;
	}
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_open);

/**
 * snd_dmaengine_pcm_open_request_chan - Open a dmaengine based PCM substream and request channel
 * @substream: PCM substream
 * @filter_fn: Filter function used to request the DMA channel
 * @filter_data: Data passed to the DMA filter function
 *
 * Returns 0 on success, a negative error code otherwise.
 *
 * This function will request a DMA channel using the passed filter function and
 * data. The function should usually be called from the pcm open callback. Note
 * that this function will use private_data field of the substream's runtime. So
 * it is not availabe to your pcm driver implementation.
 */
int snd_dmaengine_pcm_open_request_chan(struct snd_pcm_substream *substream,
	dma_filter_fn filter_fn, void *filter_data)
{
	return snd_dmaengine_pcm_open(substream,
		    snd_dmaengine_pcm_request_channel(filter_fn, filter_data));
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_open_request_chan);

/**
 * snd_dmaengine_pcm_close - Close a dmaengine based PCM substream
 * @substream: PCM substream
 */
int snd_dmaengine_pcm_close(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);

#ifdef CONFIG_SND_PXA_SSP_DUMP
	if (ssp_playback_wq)
		flush_workqueue(ssp_playback_wq);
	if (ssp_capture_wq)
		flush_workqueue(ssp_capture_wq);
	if (prtd->playback_fp)
		filp_close(prtd->playback_fp, NULL);
	if (prtd->capture_fp)
		filp_close(prtd->capture_fp, NULL);
#endif
	kfree(prtd);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_close);

/**
 * snd_dmaengine_pcm_release_chan_close - Close a dmaengine based PCM substream and release channel
 * @substream: PCM substream
 *
 * Releases the DMA channel associated with the PCM substream.
 */
int snd_dmaengine_pcm_close_release_chan(struct snd_pcm_substream *substream)
{
	struct dmaengine_pcm_runtime_data *prtd = substream_to_prtd(substream);

	dma_release_channel(prtd->dma_chan);

	return snd_dmaengine_pcm_close(substream);
}
EXPORT_SYMBOL_GPL(snd_dmaengine_pcm_close_release_chan);

MODULE_LICENSE("GPL");
