/*
 * linux/sound/soc/pxa/mmp-tdm.c
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
#include <linux/delay.h>
#include <linux/mfd/mmp-map.h>
#include "mmp-tdm.h"

struct tdm_manage_private {
	/* tdm mode select */
	bool use_4_wires;

	/* for read/write reg */
	struct device *dev;
	struct map_private *map_priv;

	struct mutex mutex;
	/* slot maintaining */
	unsigned int slot_tol;
	/* for 4 wires */
	unsigned int tx_unused_slot_tol;
	unsigned int rx_unused_slot_tol;
	/* for 3 wires */
	unsigned int unused_slot_tol;

	struct list_head substream_list;
	unsigned int substream_num;

	/* tdm clock source */
	unsigned int clk_src;

	/* tdm_cfg_param */
	unsigned int slot_size;
	unsigned int slot_space;
	unsigned int start_slot;
	unsigned int fsyn_rate;
	unsigned int bclk_rate;
	unsigned int fsyn_pulse_width;
};

/* TDM dai private data */
struct slot_info {
	int slot;
	int channel_id;
	enum tdm_out_reg cntrl_reg_id;
};

struct tdm_dai_private {
	struct device *dev;
	struct map_private *map_priv;
	struct regmap *regmap;

	/* point to tdm management*/
	struct tdm_manage_private *tdm_manage_priv;

	/* tdm codec slot configuration */
	struct slot_info tdm_out1_tx[3];
	int tdm_out1_tx_num;
	struct slot_info tdm_out2_tx[3];
	int tdm_out2_tx_num;
	unsigned int tdm_codec_mic1_rx[2];
	int tdm_codec_mic1_rx_num;
	unsigned int tdm_codec_mic2_rx[2];
	int tdm_codec_mic2_rx_num;
};

struct tdm_used_entity {
	struct list_head list;
	struct snd_pcm_substream *substream;
	/* suppose each entity only supports mono/stereo */
	int tx[2];
	int tx_num;
	int rx[2];
	int rx_num;
};

int tdm_reg_update(struct device *dev, unsigned int reg,
			unsigned int mask, unsigned int value)
{
	struct tdm_dai_private *tdm_dai_priv = dev_get_drvdata(dev);
	int ret = 0;

	ret = regmap_update_bits(tdm_dai_priv->regmap, reg, mask, value);
	if (ret < 0)
		dev_err(tdm_dai_priv->dev, "Failed to update reg 0x%x: %d\n",
								reg, ret);
	return ret;
}

/* for 3 wire mode */
struct tdm_used_entity *get_tdm_entity_from_slot(
				struct tdm_manage_private *tdm_man_priv,
				unsigned int slot, int stream)
{
	struct tdm_used_entity *entity;
	int i;

	if (unlikely(slot == 0))
		return NULL;

	if (!tdm_man_priv->use_4_wires) {
		list_for_each_entry(entity,
				&tdm_man_priv->substream_list, list) {
			for (i = 0; i < entity->tx_num; i++) {
				if (entity->tx[i] == slot)
					return entity;
			}
			for (i = 0; i < entity->rx_num; i++) {
				if (entity->rx[i] == slot)
					return entity;
			}
		}
	} else {
		list_for_each_entry(entity,
				&tdm_man_priv->substream_list, list) {
			if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
				for (i = 0; i < entity->tx_num; i++) {
					if (entity->tx[i] == slot)
						return entity;
				}
			} else {
				for (i = 0; i < entity->rx_num; i++) {
					if (entity->rx[i] == slot)
						return entity;
				}
			}
		}
	}
	return NULL;
}

struct tdm_used_entity *get_tdm_entity(struct snd_pcm_substream *substream)
{
	struct tdm_used_entity *entity;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct tdm_manage_private *tdm_man_priv;

	if (unlikely(substream == NULL))
		return NULL;

	tdm_man_priv = tdm_dai_priv->tdm_manage_priv;
	list_for_each_entry(entity, &tdm_man_priv->substream_list, list) {
		if (entity->substream == substream)
			return entity;
	}
	return NULL;
}
EXPORT_SYMBOL(get_tdm_entity);

/* slot number config */
static inline int mmp_tdm_slot_config(struct tdm_manage_private *tdm_man_priv,
						int slot)
{
	struct map_private *map_priv = tdm_man_priv->map_priv;
	int tdm_ctrl1;
	unsigned int bclk_osr;
	unsigned int fsyn_rate, bclk_rate;
	unsigned int addr, mask;

	if (slot > 15) {
		dev_err(tdm_man_priv->dev,
				"Can't alloc slot(%d) due to reach maximum\n",
				slot);
		return -EINVAL;
	}

	if (slot != 0) {
		/* clock setting */
		tdm_man_priv->fsyn_rate = tdm_man_priv->clk_src/48000 - 4;
		fsyn_rate = map_raw_read(map_priv, TDM_CTRL_REG8);
		fsyn_rate &= ~TDM_FSYN_RATE_MASK;
		fsyn_rate |= TDM_FSYN_RATE(tdm_man_priv->fsyn_rate);
		fsyn_rate &= ~TDM_FSYN_PULSE_WIDTH_MASK;
		fsyn_rate |=
			TDM_FSYN_PULSE_WIDTH(tdm_man_priv->fsyn_pulse_width);
		map_raw_write(map_priv, TDM_CTRL_REG8, fsyn_rate);

		bclk_osr = tdm_man_priv->slot_size * slot;
		bclk_osr += tdm_man_priv->slot_space * (slot - 1);
		bclk_osr += tdm_man_priv->start_slot;
		tdm_man_priv->bclk_rate =
			(tdm_man_priv->fsyn_rate + 4)/bclk_osr - 1;

		bclk_rate = map_raw_read(map_priv, TDM_CTRL_REG9);
		bclk_rate &= ~TDM_BCLK_RATE_MASK;
		bclk_rate |= TDM_BCLK_RATE(tdm_man_priv->bclk_rate);
		map_raw_write(map_priv, TDM_CTRL_REG9, bclk_rate);

		tdm_ctrl1 = map_raw_read(map_priv, TDM_CTRL_REG1);
		tdm_ctrl1 &= ~TDM_MAX_SLOT_MASK;
		tdm_ctrl1 |= TDM_MAX_SLOT(slot);
		tdm_ctrl1 |= TDM_EN;
		tdm_ctrl1 |= TDM_MNT_FIFO_EN;
		map_raw_write(map_priv, TDM_CTRL_REG1, tdm_ctrl1);

		/* apply change*/
		addr = TDM_CTRL_REG1;
		mask = TDM_APPLY_CKG_CONF | TDM_APPLY_TDM_CONF;
		tdm_reg_update(tdm_man_priv->dev, addr, mask, mask);
	} else {
		tdm_ctrl1 = TDM_EN | TDM_MNT_FIFO_EN;
		map_raw_write(map_priv, TDM_CTRL_REG1, tdm_ctrl1);
		map_raw_write(map_priv, TDM_CTRL_REG2, 0);
		map_raw_write(map_priv, TDM_CTRL_REG3, 0);
		map_raw_write(map_priv, TDM_CTRL_REG4, 0);
		map_raw_write(map_priv, TDM_CTRL_REG5, 0);
		map_raw_write(map_priv, TDM_CTRL_REG6, 0);
		map_raw_write(map_priv, TDM_CTRL_REG7, 0);
		map_raw_write(map_priv, TDM_CTRL_REG8, 0);
		map_raw_write(map_priv, TDM_CTRL_REG9, 0);

		/* apply change */
		addr = TDM_CTRL_REG1;
		mask = TDM_APPLY_CKG_CONF | TDM_APPLY_TDM_CONF;
		tdm_reg_update(tdm_man_priv->dev, addr, mask, mask);
		map_raw_write(map_priv, TDM_CTRL_REG1, 0);
	}

	return slot;
}

static inline int mmp_tdm_bus_config(struct tdm_manage_private *tdm_man_priv)
{
	int tdm_ctrl1;

	/*
	 * need to configure LR rate before release out int
	 * the clock frequency is fixed at 48k
	 * */
	map_set_port_freq(tdm_man_priv->map_priv, I2S_OUT, 48000);

	/* reset/release out interface */
	map_reset_port(tdm_man_priv->map_priv, I2S_OUT);

	/*
	 * Fixme: configure tdm_double_edge, slot_size
	 * slot_space, start_slot
	 */
	tdm_ctrl1 = map_raw_read(tdm_man_priv->map_priv, TDM_CTRL_REG1);
	tdm_ctrl1 &= ~TDM_DOUBLE_EDGE;

	tdm_ctrl1 &= ~TDM_START_SLOT_MASK;
	tdm_ctrl1 |= TDM_MST; /* master always */
	tdm_ctrl1 |= TDM_START_SLOT(tdm_man_priv->start_slot);
	tdm_ctrl1 &= ~TDM_SLOT_SIZE_MASK;
	switch (tdm_man_priv->slot_size) {
	case 20:
		tdm_ctrl1 |= TDM_SLOT_SIZE(0);
		break;
	case 24:
		tdm_ctrl1 |= TDM_SLOT_SIZE(2);
		break;
	default:
		return -EINVAL;
	}

	tdm_ctrl1 &= ~TDM_SLOT_SPACE_MASK;
	tdm_ctrl1 |= TDM_SLOT_SPACE(tdm_man_priv->slot_space);
	map_raw_write(tdm_man_priv->map_priv, TDM_CTRL_REG1, tdm_ctrl1);

	return 0;
}

/* TDM slot mangement, return the unused slot number */
int mmp_tdm_slot_inc(struct tdm_manage_private *tdm_man_priv,
			int *tx, int tx_num, int *rx, int rx_num)
{
	int slot;
	int last_slot_tol, i;
	int rx_slot_tol, tx_slot_tol;

	last_slot_tol = tdm_man_priv->slot_tol;
	if (last_slot_tol == 0) {
		/*
		 * Fixme: configure tdm_double_edge, slot_size
		 * slot_space, start_slot
		 */
		mmp_tdm_bus_config(tdm_man_priv);
	}

	if (!tdm_man_priv->use_4_wires) {
		slot = tdm_man_priv->slot_tol + tx_num + rx_num;
	} else {
		tx_slot_tol = tdm_man_priv->slot_tol + tx_num;
		tx_slot_tol -= tdm_man_priv->tx_unused_slot_tol;
		rx_slot_tol = tdm_man_priv->slot_tol + rx_num;
		rx_slot_tol -= tdm_man_priv->rx_unused_slot_tol;
		if (tx_slot_tol > rx_slot_tol) {
			slot = tx_slot_tol;
			tdm_man_priv->rx_unused_slot_tol =
						tx_slot_tol - rx_slot_tol;
			tdm_man_priv->tx_unused_slot_tol += tx_num;
		} else if (tx_slot_tol < rx_slot_tol) {
			slot = rx_slot_tol;
			tdm_man_priv->tx_unused_slot_tol =
						rx_slot_tol - tx_slot_tol;
			tdm_man_priv->rx_unused_slot_tol += rx_num;
		} else
			slot = tx_slot_tol;
	}
	if (slot != tdm_man_priv->slot_tol) {
		slot = mmp_tdm_slot_config(tdm_man_priv, slot);
		if (slot < 0)
			return slot;
	} else
		dev_info(tdm_man_priv->dev, "doesn't need increase slot\n");

	if (!tdm_man_priv->use_4_wires) {
		for (i = 0; i < tx_num; i++)
			*(tx + i) = ++tdm_man_priv->slot_tol;
		for (i = 0; i < rx_num; i++)
			*(rx + i) = ++tdm_man_priv->slot_tol;
	} else {
		tdm_man_priv->slot_tol = slot;
		for (i = 0; i < tx_num; i++) {
			*(tx + i) = tdm_man_priv->slot_tol -
					tdm_man_priv->tx_unused_slot_tol;
			*(tx + i) += 1;
			tdm_man_priv->tx_unused_slot_tol--;
		}
		for (i = 0; i < rx_num; i++) {
			*(rx + i) = tdm_man_priv->slot_tol -
					tdm_man_priv->rx_unused_slot_tol;
			*(rx + i) += 1;
			tdm_man_priv->rx_unused_slot_tol--;
		}
	}
	return slot;
}

int mmp_tdm_slot_dec(struct tdm_manage_private *tdm_man_priv)
{
	int slot;

	if (!tdm_man_priv->use_4_wires) {
		slot = tdm_man_priv->slot_tol - tdm_man_priv->unused_slot_tol;
		tdm_man_priv->unused_slot_tol = 0;
	} else {
		if (tdm_man_priv->tx_unused_slot_tol >
				tdm_man_priv->rx_unused_slot_tol) {
			slot = tdm_man_priv->slot_tol -
					tdm_man_priv->rx_unused_slot_tol;
			tdm_man_priv->tx_unused_slot_tol -=
					tdm_man_priv->rx_unused_slot_tol;
			tdm_man_priv->rx_unused_slot_tol = 0;
		} else {
			slot = tdm_man_priv->slot_tol -
					tdm_man_priv->tx_unused_slot_tol;
			tdm_man_priv->rx_unused_slot_tol -=
					tdm_man_priv->tx_unused_slot_tol;
			tdm_man_priv->tx_unused_slot_tol = 0;
		}
	}

	if (slot == tdm_man_priv->slot_tol) {
		dev_info(tdm_man_priv->dev, "doesn't need decrease slot\n");
		return slot;
	}

	mmp_tdm_slot_config(tdm_man_priv, slot);

	tdm_man_priv->slot_tol = slot;

	return 0;
}

/* dst < src */
int mmp_tdm_move_slot(struct snd_pcm_substream *substream, int dst)
{
	struct tdm_used_entity *entity;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int tx_num = 0, tx[3];
	int rx_num = 0, rx[2];
	int src_slot = 0, i;

	entity = get_tdm_entity(substream);
	if (unlikely(entity == NULL))
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < entity->tx_num; i++) {
			if (entity->tx[i] > src_slot)
				src_slot = entity->tx[i];
			tx[i] = entity->tx[i];
		}
		tx[i] = dst;
		tx_num = entity->tx_num + 1;
		snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
		usleep_range(21, 25);
		for (i = 0; i < entity->tx_num; i++) {
			if (entity->tx[i] == src_slot) {
				rx[i] = dst;
				entity->tx[i] = dst;
			} else
				rx[i] = entity->tx[i];
			rx_num++;
		}
		snd_soc_dai_set_channel_map(codec_dai, 0, NULL, rx_num, rx);
		usleep_range(21, 25);
		for (i = 0; i < entity->tx_num; i++) {
			if (tx[i] == src_slot) {
				tx[i] = dst;
				entity->tx[i] = dst;
			} else
				tx[i] = entity->tx[i];
		}
		snd_soc_dai_set_channel_map(cpu_dai, entity->tx_num,
						tx, 0, NULL);
	} else {
		for (i = 0; i < entity->rx_num; i++) {
			tx[i] = entity->rx[i];
			if (entity->rx[i] > src_slot)
				src_slot = entity->rx[i];
		}
		tx[i] = dst;
		tx_num = entity->tx_num + 1;
		snd_soc_dai_set_channel_map(codec_dai, tx_num, tx, 0, NULL);
		usleep_range(21, 25);
		for (i = 0; i < entity->rx_num; i++) {
			if (entity->rx[i] == src_slot) {
				rx[i] = dst;
				entity->rx[i] = dst;
			} else
				rx[i] = entity->rx[i];
			rx_num++;
		}
		snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, rx_num, rx);
		usleep_range(21, 25);
		for (i = 0; i < entity->rx_num; i++) {
			if (tx[i] == src_slot) {
				tx[i] = dst;
				entity->rx[i] = dst;
			} else
				tx[i] = entity->rx[i];
		}
		snd_soc_dai_set_channel_map(codec_dai, entity->tx_num,
						tx, 0, NULL);
	}
	return 0;
}
EXPORT_SYMBOL(mmp_tdm_move_slot);

/* TDM slot allocate for static allocation */
int mmp_tdm_static_slot_alloc(struct snd_pcm_substream *substream,
				int *tx, int tx_num, int *rx, int rx_num)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct tdm_manage_private *tdm_man_priv;
	int slot;
	struct tdm_used_entity *entity;
	int i;
	int rx_slot_tol, tx_slot_tol;

	if (unlikely(substream == NULL))
		return -EINVAL;

	tdm_man_priv = tdm_dai_priv->tdm_manage_priv;
	mutex_lock(&tdm_man_priv->mutex);
	/*
	 * Fixme: 0<->channel 0; 1<->channel 1
	 * suppose we don't care the bit map in rx direction due to
	 * Many clients can receiver stream from the same slot???
	 * talked with xin, suggest using the different slot for different
	 * peripheral device even if the stream is the same for simple handling.
	 */
	for (i = 0; i < tx_num; i++) {
		entity = get_tdm_entity_from_slot(tdm_man_priv, tx[i],
						SNDRV_PCM_STREAM_PLAYBACK);
		if (entity) {
			pr_err("the tx channel[%d] is in used\n", tx[i]);
			return -EINVAL;
		}
	}
	for (i = 0; i < rx_num; i++) {
		entity = get_tdm_entity_from_slot(tdm_man_priv, rx[i],
						SNDRV_PCM_STREAM_CAPTURE);
		if (entity) {
			pr_err("the rx channel[%d] is in used\n", rx[i]);
			return -EINVAL;
		}
	}

	slot = tdm_man_priv->slot_tol;
	if (slot == 0) {
		/*
		 * Fixme: configure tdm_double_edge, slot_size
		 * slot_space, start_slot
		 */
		mmp_tdm_bus_config(tdm_man_priv);
	}
	/* malloc one entity */
	entity = devm_kzalloc(tdm_man_priv->dev,
					sizeof(struct tdm_used_entity),
					GFP_KERNEL);
	if (unlikely(entity == NULL)) {
		mutex_unlock(&tdm_man_priv->mutex);
		return -ENOMEM;
	}
	entity->substream = substream;
	for (i = 0; i < tx_num; i++)
		entity->tx[i] = tx[i];
	entity->tx_num = tx_num;
	for (i = 0; i < rx_num; i++)
		entity->rx[i] = rx[i];
	entity->rx_num = rx_num;
	INIT_LIST_HEAD(&entity->list);
	list_add_tail(&entity->list, &tdm_man_priv->substream_list);
	tdm_man_priv->substream_num++;

	if (!tdm_man_priv->use_4_wires) {
		for (i = 0; i < tx_num; i++) {
			if (tx[i] > slot)
				slot = tx[i];
		}
		for (i = 0; i < rx_num; i++) {
			if (rx[i] > slot)
				slot = rx[i];
		}
		if (slot == tdm_man_priv->slot_tol) {
			dev_info(tdm_man_priv->dev,
					"tdm slot has been allocated\n");
			mutex_unlock(&tdm_man_priv->mutex);
			return slot;
		}
		slot = mmp_tdm_slot_config(tdm_man_priv, slot);
		if (slot < 0) {
			list_del(&entity->list);
			devm_kfree(tdm_man_priv->dev, entity);
			mutex_unlock(&tdm_man_priv->mutex);
			return -EINVAL;
		}
	} else {
		rx_slot_tol = slot - tdm_man_priv->rx_unused_slot_tol;
		tx_slot_tol = slot - tdm_man_priv->tx_unused_slot_tol;

		for (i = 0; i < tx_num; i++) {
			if (tx[i] > slot)
				slot = tx[i];
			if (tx[i] > tx_slot_tol)
				tx_slot_tol = tx[i];
		}

		for (i = 0; i < rx_num; i++) {
			if (rx[i] > slot)
				slot = rx[i];
			if (rx[i] > rx_slot_tol)
				rx_slot_tol = rx[i];
		}
		slot = mmp_tdm_slot_config(tdm_man_priv, slot);
		if (slot < 0) {
			list_del(&entity->list);
			devm_kfree(tdm_man_priv->dev, entity);
			mutex_unlock(&tdm_man_priv->mutex);
			return -EINVAL;
		}

		tdm_man_priv->tx_unused_slot_tol = (slot - tx_slot_tol);
		tdm_man_priv->rx_unused_slot_tol = (slot - rx_slot_tol);

		if (slot == tdm_man_priv->slot_tol) {
			dev_info(tdm_man_priv->dev,
					"tdm slot has been allocated\n");
			mutex_unlock(&tdm_man_priv->mutex);
			return slot;
		}
	}
	tdm_man_priv->slot_tol = slot;
	mutex_unlock(&tdm_man_priv->mutex);
	return slot;
}
EXPORT_SYMBOL(mmp_tdm_static_slot_alloc);

/* TDM slot free for static allocation */
int mmp_tdm_static_slot_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct tdm_manage_private *tdm_man_priv;
	struct tdm_used_entity *entity;
	int max;

	tdm_man_priv = tdm_dai_priv->tdm_manage_priv;
	mutex_lock(&tdm_man_priv->mutex);
	entity = get_tdm_entity(substream);
	if (unlikely(entity == NULL)) {
		mutex_unlock(&tdm_man_priv->mutex);
		return -EINVAL;
	}

	/* free one entity */
	list_del(&entity->list);
	devm_kfree(tdm_man_priv->dev, entity);

	if (!tdm_man_priv->use_4_wires) {
		max = tdm_man_priv->slot_tol;
		for (; max > 0; max--) {
			entity = get_tdm_entity_from_slot(tdm_man_priv, max, 0);
			if (entity == NULL) {
				tdm_man_priv->unused_slot_tol++;
				continue;
			}
			break;
		}
	} else {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			max = tdm_man_priv->slot_tol -
					tdm_man_priv->tx_unused_slot_tol;
			for (; max > 0; max--) {
				entity = get_tdm_entity_from_slot(tdm_man_priv,
						max,
						SNDRV_PCM_STREAM_PLAYBACK);
				if (entity == NULL) {
					tdm_man_priv->tx_unused_slot_tol++;
					continue;
				}
				break;
			}
		} else {
			max = tdm_man_priv->slot_tol -
				tdm_man_priv->rx_unused_slot_tol;
			for (; max > 0; max--) {
				entity = get_tdm_entity_from_slot(tdm_man_priv,
						max,
						SNDRV_PCM_STREAM_CAPTURE);
				if (entity == NULL) {
					tdm_man_priv->rx_unused_slot_tol++;
					continue;
				}
				break;
			}
		}
	}
	mmp_tdm_slot_dec(tdm_man_priv);
	mutex_unlock(&tdm_man_priv->mutex);
	return 0;
}
EXPORT_SYMBOL(mmp_tdm_static_slot_free);

unsigned int mmp_tdm_request_slot(struct snd_pcm_substream *substream,
						int channel)
{
	int tx[2], rx[2];
	int tx_num, rx_num;
	int ret = 0;
	int i;
	struct tdm_used_entity *entity;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct tdm_dai_private *tdm_dai_priv =
				snd_soc_dai_get_drvdata(cpu_dai);
	struct tdm_manage_private *tdm_man_priv;

	if (unlikely(substream == NULL))
		return -EINVAL;

	tdm_man_priv = tdm_dai_priv->tdm_manage_priv;
	entity = get_tdm_entity(substream);
	if (entity != NULL) {
		dev_dbg(tdm_man_priv->dev, "shouldn't happen!!!\n");
		return 0;
	}

	/* malloc one entity */
	entity = devm_kzalloc(tdm_man_priv->dev,
					sizeof(struct tdm_used_entity),
					GFP_KERNEL);
	if (unlikely(entity == NULL))
		return -ENOMEM;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tx_num = channel;
		rx_num = 0;
		mutex_lock(&tdm_man_priv->mutex);
		ret = mmp_tdm_slot_inc(tdm_man_priv, tx, tx_num, rx, rx_num);
		if (ret < 0) {
			mutex_unlock(&tdm_man_priv->mutex);
			return -EINVAL;
		}
		entity->substream = substream;
		for (i = 0; i < tx_num; i++)
			entity->tx[i] = tx[i];

		entity->tx_num = tx_num;
		INIT_LIST_HEAD(&entity->list);
		list_add_tail(&entity->list, &tdm_man_priv->substream_list);
		tdm_man_priv->substream_num++;
		mutex_unlock(&tdm_man_priv->mutex);
		snd_soc_dai_set_channel_map(cpu_dai, tx_num, tx, 0, NULL);
		/*
		 * Fixme: now, the cpu confiuration is slow
		 * Maybe we don't need it on real silicon
		 */
		usleep_range(21, 25);
		snd_soc_dai_set_channel_map(codec_dai, 0, NULL, tx_num, tx);
	} else {
		tx_num = 0;
		rx_num = channel;
		mutex_lock(&tdm_man_priv->mutex);
		ret = mmp_tdm_slot_inc(tdm_man_priv, tx, tx_num, rx, rx_num);
		if (ret < 0) {
			mutex_unlock(&tdm_man_priv->mutex);
			return -EINVAL;
		}
		entity->substream = substream;
		for (i = 0; i < rx_num; i++)
			entity->rx[i] = rx[i];
		entity->rx_num = rx_num;
		INIT_LIST_HEAD(&entity->list);
		list_add_tail(&entity->list, &tdm_man_priv->substream_list);
		tdm_man_priv->substream_num++;
		mutex_unlock(&tdm_man_priv->mutex);
		snd_soc_dai_set_channel_map(codec_dai, rx_num, rx, 0, NULL);
		snd_soc_dai_set_channel_map(cpu_dai, 0, NULL, rx_num, rx);
	}
	return ret;
}
EXPORT_SYMBOL(mmp_tdm_request_slot);

unsigned int mmp_tdm_free_slot(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct tdm_manage_private *tdm_man_priv;
	struct tdm_used_entity *entity;
	struct tdm_used_entity *entity_resch;
	int empty[2] = {0, 0};
	int i, j, max;
	int min, min_idx = 0;

	tdm_man_priv = tdm_dai_priv->tdm_manage_priv;
	mutex_lock(&tdm_man_priv->mutex);
	entity = get_tdm_entity(substream);
	if (unlikely(entity == NULL)) {
		mutex_unlock(&tdm_man_priv->mutex);
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* firstly, stop receiver to get stream */
		snd_soc_dai_set_channel_map(codec_dai, 0, NULL,
						entity->tx_num, empty);
		/*
		 * Fixme: now, the codec confiuration is slow
		 * Maybe we don't need it on real silicon
		 */
		usleep_range(21, 25);
		snd_soc_dai_set_channel_map(cpu_dai, entity->tx_num,
						empty, 0, NULL);
		/* reuse the free slot */
		for (i = 0; i < entity->tx_num; i++) {
			if (!tdm_man_priv->use_4_wires) {
				max = tdm_man_priv->slot_tol - i;
				entity_resch =
					get_tdm_entity_from_slot(tdm_man_priv,
								max, 0);
			} else {
				max = tdm_man_priv->slot_tol -
					tdm_man_priv->tx_unused_slot_tol;
				entity_resch =
					get_tdm_entity_from_slot(tdm_man_priv,
						max,
						SNDRV_PCM_STREAM_PLAYBACK);
			}

			if (unlikely(entity_resch == NULL)) {
				dev_err(tdm_man_priv->dev,
						"shouldn't happen!!!\n");
				mutex_unlock(&tdm_man_priv->mutex);
				return -EINVAL;
			}
			if (entity != entity_resch) {
				/*
				 * need to resch due to
				 * it remove the maximum slot
				 */
				min = 0xff;
				for (j = 0; j < entity->tx_num; j++) {
					if (min > entity->tx[j]) {
						min = entity->tx[j];
						min_idx = j;
					}
				}
				entity->tx[min_idx] = 0xff;
				mmp_tdm_move_slot(entity_resch->substream,
							min);
			}
			if (!tdm_man_priv->use_4_wires)
				tdm_man_priv->unused_slot_tol++;
			else
				tdm_man_priv->tx_unused_slot_tol++;
		}
	} else {
		/* firstly, stop receiver to get stream */
		snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
						entity->tx_num, empty);
		snd_soc_dai_set_channel_map(codec_dai, entity->tx_num,
						empty, 0, NULL);
		/* reuse the free slot */
		for (i = 0; i < entity->rx_num; i++) {
			if (!tdm_man_priv->use_4_wires) {
				max = tdm_man_priv->slot_tol - i;
				entity_resch =
					get_tdm_entity_from_slot(tdm_man_priv,
							max, 0);
			} else {
				max = tdm_man_priv->slot_tol -
					tdm_man_priv->rx_unused_slot_tol;
				entity_resch =
					get_tdm_entity_from_slot(tdm_man_priv,
						max, SNDRV_PCM_STREAM_CAPTURE);
			}
			if (entity != entity_resch) {
				/*
				 * need to resch due to
				 * it remove the maximum slot
				 */
				min = 0xff;
				for (j = 0; j < entity->rx_num; j++) {
					if (min > entity->rx[j]) {
						min = entity->rx[j];
						min_idx = j;
					}
				}
				entity->rx[min_idx] = 0xff;
				mmp_tdm_move_slot(entity_resch->substream,
							min);
			}
			if (!tdm_man_priv->use_4_wires)
				tdm_man_priv->unused_slot_tol++;
			else
				tdm_man_priv->rx_unused_slot_tol++;
		}
	}
	/* free one entity */
	list_del(&entity->list);
	devm_kfree(tdm_man_priv->dev, entity);
	mmp_tdm_slot_dec(tdm_man_priv);
	mutex_unlock(&tdm_man_priv->mutex);
	return 0;
}

/* Set SYSCLK */
static int mmp_set_tdm_dai_sysclk(struct snd_soc_dai *cpu_dai,
				    int clk_id, unsigned int freq, int dir)
{
	struct tdm_dai_private *tdm_dai_priv;
	struct map_private *map_priv;
	int ret = 0;

	tdm_dai_priv = snd_soc_dai_get_drvdata(cpu_dai);
	map_priv = tdm_dai_priv->map_priv;
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

static int mmp_tdm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	unsigned int tdm_ctrl_reg6 = 0, tdm_ctrl_reg7 = 0;
	unsigned int mask = 0, addr, offset;
	struct tdm_used_entity *entity;
	int i;

	entity = get_tdm_entity(substream);
	if (unlikely(entity == NULL))
		return -EINVAL;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		for (i = 0; i < entity->tx_num; i++) {
			offset = (entity->tx[i] - 1) * TDM_CHO_DL_LEN;
			tdm_ctrl_reg6 |= TDM_CHO_DL_16 << offset;
			mask |= TDM_CHO_DL_MASK << offset;
		}
		for (i = 0; i < entity->rx_num; i++) {
			offset = (entity->rx[i] - 1) * TDM_CHI_DL_LEN;
			tdm_ctrl_reg7 |= TDM_CHI_DL_16 << offset;
			mask |= TDM_CHI_DL_MASK << offset;
		}
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		for (i = 0; i < entity->tx_num; i++) {
			offset = (entity->tx[i] - 1) * TDM_CHO_DL_LEN;
			tdm_ctrl_reg6 |= TDM_CHO_DL_20 << offset;
			mask |= TDM_CHO_DL_MASK << offset;
		}
		for (i = 0; i < entity->rx_num; i++) {
			offset = (entity->rx[i] - 1) * TDM_CHI_DL_LEN;
			tdm_ctrl_reg7 |= TDM_CHI_DL_20 << offset;
			mask |= TDM_CHI_DL_MASK << offset;
		}
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		for (i = 0; i < entity->tx_num; i++) {
			offset = (entity->tx[i] - 1) * TDM_CHO_DL_LEN;
			tdm_ctrl_reg6 |= TDM_CHO_DL_24 << offset;
			mask |= TDM_CHO_DL_MASK << offset;
		}
		for (i = 0; i < entity->rx_num; i++) {
			offset = (entity->rx[i] - 1) * TDM_CHI_DL_LEN;
			tdm_ctrl_reg7 |= TDM_CHI_DL_24 << offset;
			mask |= TDM_CHI_DL_MASK << offset;
		}
		break;
	default:
		return -EINVAL;
	}

	if (entity->tx_num != 0)
		tdm_reg_update(dai->dev, TDM_CTRL_REG6, mask, tdm_ctrl_reg6);
	if (entity->rx_num != 0)
		tdm_reg_update(dai->dev, TDM_CTRL_REG7, mask, tdm_ctrl_reg7);

	/* apply tdm config */
	addr = TDM_CTRL_REG1;
	mask = TDM_APPLY_TDM_CONF;
	tdm_reg_update(dai->dev, addr, mask, mask);

	return 0;
}

static int mmp_tdm_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
#if 0
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	struct map_private *map_priv = tdm_dai_priv->map_priv;
#endif

	/* debug clock on real silicon */
#if 0
	clk_enable(map_priv->audio_clk);
	clk_enable(map_priv->apb_clk);
	clk_set_rate(map_priv->map_apb_clk, 0x0);
#endif

	return 0;
}

static void mmp_tdm_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
#if 0
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	struct map_private *map_priv = tdm_dai_priv->map_priv;
#endif

	/* debug clock on real silicon */
#if 0
	clk_disable(map_priv->apb_clk);
	clk_disable(map_priv->audio_clk);
#endif

	return;
}

#if 0
/*
 * now, tdm enable/diable is put into request/free slot phase,
 * maybe we need to enable/disable tdm at the trigger phase
 */
static int mmp_tdm_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	unsigned int inf = 0;
	int ret = 0, i;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
/* tdm_en should be controled in tdm slot allocation driver */
#if 0
		inf = tdm_read(dai->dev, TDM_CTRL_REG1);
		inf |= TDM_EN;
		tdm_write(dai->dev, TDM_CTRL_REG1, inf);
#endif
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * Fixme: should be disable after all the slot doesn't be used
		 * should consider rx
		 */
/* tdm_en should be controled in tdm slot allocation driver */
#if 0
		if (tdm_dai_priv->ch_bit_map == 0) {
			inf = tdm_read(dai->dev, TDM_CTRL_REG1);
			inf &= ~TDM_EN;
			tdm_write(dai->dev, TDM_CTRL_REG1, inf);
		}
#endif
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}
#endif

/* out1 */
static int mmp_tdm_set_out1_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	unsigned int slot_1 = 0, slot_2 = 0;
	unsigned int mask = 0;
	int channel_id;
	int tdm_out1_tx_num;
	struct slot_info *tdm_out1_tx;
	bool found;
	enum tdm_out_reg cntrl_reg_id;
	int i, j;

	tdm_out1_tx_num = tdm_dai_priv->tdm_out1_tx_num;
	tdm_out1_tx = tdm_dai_priv->tdm_out1_tx;
	cntrl_reg_id = NO_OUT_CONTROL_REG;
	if (tx_num > tdm_out1_tx_num) {
		for (i = 0; i < tx_num; i++) {
			if (tdm_dai_priv->tdm_out1_tx_num == 0)
				channel_id = OUT1_CH1;
			else if (tdm_dai_priv->tdm_out1_tx_num == 1)
				channel_id = OUT1_CH2;
			else if (tdm_dai_priv->tdm_out1_tx_num == 2)
				channel_id =
					(tdm_out1_tx[0].slot
					 > tdm_out1_tx[1].slot) ?
					tdm_out1_tx[0].channel_id :
					tdm_out1_tx[1].channel_id;

			found = false;
			for (j = 0; j < tdm_out1_tx_num; j++) {
				if (tx_slot[i] == tdm_out1_tx[j].slot) {
					found = true;
					break;
				}
				if (channel_id == tdm_out1_tx[j].channel_id) {
					cntrl_reg_id =
						tdm_out1_tx[j].cntrl_reg_id;
				}
			}
			if (found)
				continue;

			if (cntrl_reg_id != TDM_CONTRL_REG2) {
				slot_1 |= (tx_slot[i] << ((channel_id-1) * 4));
				mask |= 0xf << ((channel_id-1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].slot =
					tx_slot[i];
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].channel_id =
					channel_id;
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].cntrl_reg_id =
					TDM_CONTRL_REG2;
				tdm_dai_priv->tdm_out1_tx_num++;
			} else {
				slot_2 |= (tx_slot[i] << ((channel_id-1) * 4));
				mask |= 0xf << ((channel_id-1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG3,
						mask, slot_2);
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].slot =
					tx_slot[i];
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].channel_id =
					channel_id;
				tdm_out1_tx[tdm_dai_priv->
					tdm_out1_tx_num].cntrl_reg_id =
					TDM_CONTRL_REG3;
				tdm_dai_priv->tdm_out1_tx_num++;
			}
		}
	} else if (tx_num < tdm_out1_tx_num) {
		for (i = 0; i < tdm_out1_tx_num; i++) {
			found = false;
			for (j = 0; j < tx_num; j++) {
				if (tx_slot[j] == tdm_out1_tx[i].slot) {
					found = true;
					break;
				}
			}
			if (found)
				continue;

			/* reduce the slot */
			if (tdm_out1_tx[i].cntrl_reg_id == TDM_CONTRL_REG2) {
				slot_1 &= ~(0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
			} else {
				slot_2 &= ~(0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG3,
						mask, slot_2);
			}
			/* rearrage the tdm_out1_tx array */
			for (j = i; j < tdm_out1_tx_num; j++) {
				tdm_out1_tx[j].slot =
					tdm_out1_tx[j+1].slot;
				tdm_out1_tx[j].channel_id =
					tdm_out1_tx[j+1].channel_id;
				tdm_out1_tx[j].cntrl_reg_id =
					tdm_out1_tx[j+1].cntrl_reg_id;
			}
			tdm_out1_tx[j - 1].slot = 0;
			tdm_out1_tx[j - 1].channel_id = 0;
			tdm_out1_tx[j - 1].cntrl_reg_id = 0;
			tdm_dai_priv->tdm_out1_tx_num--;
			break;
		}
	} else {
		/* replace for disabling slot purpose */
		for (i = 0; i < tdm_out1_tx_num; i++) {
			if ((tx_slot[i] == 0) &&
				(tdm_out1_tx[i].cntrl_reg_id ==
					TDM_CONTRL_REG2)) {
				slot_1 &= ~(0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
				tdm_out1_tx[i].slot = 0;
				tdm_out1_tx[i].channel_id = 0;
				tdm_out1_tx[i].cntrl_reg_id = 0;
			} else if ((tx_slot[i] == 0) &&
				(tdm_out1_tx[i].cntrl_reg_id ==
					TDM_CONTRL_REG3)) {
				slot_2 &= ~(0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out1_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG3,
						mask, slot_2);
				tdm_out1_tx[i].slot = 0;
				tdm_out1_tx[i].channel_id = 0;
				tdm_out1_tx[i].cntrl_reg_id = 0;
			}
		}
		tdm_dai_priv->tdm_out1_tx_num = 0;
	}

	/* apply change */
	mask = TDM_APPLY_TDM_CONF;
	tdm_reg_update(dai->dev, TDM_CTRL_REG1, mask, mask);

	return 0;
}

/* out2 */
static int mmp_tdm_set_out2_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	unsigned int slot_1 = 0, slot_2 = 0;
	unsigned int mask = 0;
	int channel_id;
	int tdm_out2_tx_num;
	struct slot_info *tdm_out2_tx;
	bool found;
	enum tdm_out_reg cntrl_reg_id;
	int i, j;

	tdm_out2_tx_num = tdm_dai_priv->tdm_out2_tx_num;
	tdm_out2_tx = tdm_dai_priv->tdm_out2_tx;
	cntrl_reg_id = NO_OUT_CONTROL_REG;
	if (tx_num > tdm_out2_tx_num) {
		for (i = 0; i < tx_num; i++) {
			if (tdm_dai_priv->tdm_out2_tx_num == 0)
				channel_id = OUT2_CH1;
			else if (tdm_dai_priv->tdm_out2_tx_num == 1)
				channel_id = OUT2_CH2;
			else if (tdm_dai_priv->tdm_out2_tx_num == 2)
				channel_id =
					(tdm_out2_tx[0].slot
					 > tdm_out2_tx[1].slot) ?
					tdm_out2_tx[0].channel_id :
					tdm_out2_tx[1].channel_id;

			found = false;
			for (j = 0; j < tdm_out2_tx_num; j++) {
				if (tx_slot[i] == tdm_out2_tx[j].slot) {
					found = true;
					break;
				}
				if (channel_id == tdm_out2_tx[j].channel_id) {
					cntrl_reg_id =
						tdm_out2_tx[j].cntrl_reg_id;
				}
			}
			if (found)
				continue;

			if (cntrl_reg_id != TDM_CONTRL_REG2) {
				slot_1 |= (tx_slot[i] << ((channel_id-1) * 4));
				mask |= 0xf << ((channel_id-1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].slot =
					tx_slot[i];
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].channel_id =
					channel_id;
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].cntrl_reg_id =
					TDM_CONTRL_REG2;
				tdm_dai_priv->tdm_out2_tx_num++;
			} else {
				slot_2 |= (tx_slot[i] << ((channel_id-1) * 4));
				mask |= 0xf << ((channel_id-1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG3,
						mask, slot_2);
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].slot =
					tx_slot[i];
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].channel_id =
					channel_id;
				tdm_out2_tx[tdm_dai_priv->
					tdm_out2_tx_num].cntrl_reg_id =
					TDM_CONTRL_REG3;
				tdm_dai_priv->tdm_out2_tx_num++;
			}
		}
	} else if (tx_num < tdm_out2_tx_num) {
		for (i = 0; i < tdm_out2_tx_num; i++) {
			found = false;
			for (j = 0; j < tx_num; j++) {
				if (tx_slot[j] == tdm_out2_tx[i].slot) {
					found = true;
					break;
				}
			}
			if (found)
				continue;
			/* reduce the slot */
			if (tdm_out2_tx[i].cntrl_reg_id == TDM_CONTRL_REG2) {
				slot_1 &= ~(0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
			} else {
				slot_2 &= ~(0xf <<
					((tdm_out2_tx[i].channel_id-1) * 4));
				mask |= 0xf <<
					((tdm_out2_tx[i].channel_id-1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG3,
						mask, slot_2);
			}
			/* re-arrange the tdm_out2_tx array */
			for (j = i; j < tdm_out2_tx_num; j++) {
				tdm_out2_tx[j].slot =
					tdm_out2_tx[j+1].slot;
				tdm_out2_tx[j].channel_id =
					tdm_out2_tx[j+1].channel_id;
				tdm_out2_tx[j].cntrl_reg_id =
					tdm_out2_tx[j+1].cntrl_reg_id;
			}
			tdm_out2_tx[j - 1].slot = 0;
			tdm_out2_tx[j - 1].channel_id = 0;
			tdm_out2_tx[j - 1].cntrl_reg_id = 0;
			tdm_dai_priv->tdm_out2_tx_num--;
			break;
		}
	} else {
		/* replace for disabling slot purpose */
		for (i = 0; i < tdm_out2_tx_num; i++) {
			if ((tx_slot[i] == 0) &&
				(tdm_out2_tx[i].cntrl_reg_id ==
					TDM_CONTRL_REG2)) {
				slot_1 &= ~(0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev, TDM_CTRL_REG2,
						mask, slot_1);
				tdm_out2_tx[i].slot = 0;
				tdm_out2_tx[i].channel_id = 0;
				tdm_out2_tx[i].cntrl_reg_id = 0;
			} else if ((tx_slot[i] == 0) &&
					(tdm_out2_tx[i].cntrl_reg_id ==
						TDM_CONTRL_REG3)) {
				slot_2 &= ~(0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4));
				mask |= 0xf <<
					((tdm_out2_tx[i].channel_id - 1) * 4);
				tdm_reg_update(dai->dev,
						TDM_CTRL_REG3, mask, slot_2);
				tdm_out2_tx[i].slot = 0;
				tdm_out2_tx[i].channel_id = 0;
				tdm_out2_tx[i].cntrl_reg_id = 0;
			}
		}
		tdm_dai_priv->tdm_out2_tx_num = 0;
	}

	/* apply change */
	mask = TDM_APPLY_TDM_CONF;
	tdm_reg_update(dai->dev, TDM_CTRL_REG1, mask, mask);
	return 0;
}

/* mic1 */
static int mmp_tdm_set_mic1_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	unsigned int rx_slot_1 = 0;
	unsigned int mask;

	/* FixME: For MIC1, we assume it use the [4-7] bits */
	mask = 0xff;
	rx_slot_1 |= rx_slot[0];
	rx_slot_1 |= rx_slot[1] << 4;
	tdm_reg_update(dai->dev, TDM_CTRL_REG5, mask, rx_slot_1);

	tdm_dai_priv->tdm_codec_mic1_rx[0] = rx_slot[0];
	tdm_dai_priv->tdm_codec_mic1_rx[1] = rx_slot[1];
	tdm_dai_priv->tdm_codec_mic1_rx_num = 2;

	/* apply change */
	mask = TDM_APPLY_TDM_CONF;
	tdm_reg_update(dai->dev, TDM_CTRL_REG1, mask, mask);
	return 0;
}

/* mic2 */
static int mmp_tdm_set_mic2_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct tdm_dai_private *tdm_dai_priv = snd_soc_dai_get_drvdata(dai);
	unsigned int rx_slot_1 = 0;
	unsigned int mask;

	/* FixME: For MIC2, we assume it use the [8-11] bits */
	mask = 0xff00;
	rx_slot_1 |= rx_slot[0] << 8;
	rx_slot_1 |= rx_slot[1] << 12;
	tdm_reg_update(dai->dev, TDM_CTRL_REG5, mask, rx_slot_1);

	tdm_dai_priv->tdm_codec_mic2_rx[0] = rx_slot[0];
	tdm_dai_priv->tdm_codec_mic2_rx[1] = rx_slot[1];
	tdm_dai_priv->tdm_codec_mic2_rx_num = 2;

	/* apply change */
	mask = TDM_APPLY_TDM_CONF;
	tdm_reg_update(dai->dev, TDM_CTRL_REG1, mask, mask);
	return 0;
}

#define MMP_MAP_RATES SNDRV_PCM_RATE_8000_192000
#define MMP_MAP_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops mmp_tdm_out1_ops = {
	.startup	= mmp_tdm_startup,
	.shutdown	= mmp_tdm_shutdown,
	.hw_params	= mmp_tdm_hw_params,
	.set_sysclk	= mmp_set_tdm_dai_sysclk,
	.set_channel_map = mmp_tdm_set_out1_channel_map,
};

static struct snd_soc_dai_ops mmp_tdm_out2_ops = {
	.startup	= mmp_tdm_startup,
	.shutdown	= mmp_tdm_shutdown,
	.hw_params	= mmp_tdm_hw_params,
	.set_sysclk	= mmp_set_tdm_dai_sysclk,
	.set_channel_map = mmp_tdm_set_out2_channel_map,
};

static struct snd_soc_dai_ops mmp_tdm_mic1_ops = {
	.startup	= mmp_tdm_startup,
	.shutdown	= mmp_tdm_shutdown,
	.hw_params	= mmp_tdm_hw_params,
	.set_sysclk	= mmp_set_tdm_dai_sysclk,
	.set_channel_map = mmp_tdm_set_mic1_channel_map,
};

static struct snd_soc_dai_ops mmp_tdm_mic2_ops = {
	.startup	= mmp_tdm_startup,
	.shutdown	= mmp_tdm_shutdown,
	.hw_params	= mmp_tdm_hw_params,
	.set_sysclk	= mmp_set_tdm_dai_sysclk,
	.set_channel_map = mmp_tdm_set_mic2_channel_map,
};

struct snd_soc_dai_driver mmp_tdm_dais[] = {
	/* tdm dai */
	{
		.name = "tdm-out1",
		.id = 2,
		.playback = {
			.stream_name  = "TDM_OUT1_PLAYBACK",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_tdm_out1_ops,
	},
	{
		.name = "tdm-out2",
		.id = 3,
		.playback = {
			.stream_name  = "TDM_OUT2_PLAYBACK",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_tdm_out2_ops,
	},
	{
		.name = "tdm-codec-mic1",
		.id = 4,
		.capture = {
			.stream_name  = "TDM_MIC1_CAP",
			.channels_min = 1,
			.channels_max = 1,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_tdm_mic1_ops,
	},
	{
		.name = "tdm-codec-mic2",
		.id = 5,
		.capture = {
			.stream_name  = "TDM_MIC2_CAP",
			.channels_min = 1,
			.channels_max = 1,
			.rates = MMP_MAP_RATES,
			.formats = MMP_MAP_FORMATS,
		},
		.ops = &mmp_tdm_mic2_ops,
	},
};

static const struct snd_soc_component_driver mmp_map_be_tdm_component = {
	.name           = "mmp-map-be-tdm",
};

static int mmp_tdm_dai_probe(struct platform_device *pdev)
{
	struct map_private *map_priv = dev_get_drvdata(pdev->dev.parent);
	struct tdm_dai_private *tdm_dai_priv;
	struct tdm_manage_private *tdm_priv;
	struct tdm_platform_data *pdata;
	int ret;

	tdm_dai_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct tdm_dai_private), GFP_KERNEL);
	if (tdm_dai_priv == NULL)
		return -ENOMEM;

	tdm_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct tdm_manage_private), GFP_KERNEL);
	if (tdm_priv == NULL)
		return -ENOMEM;

	tdm_dai_priv->map_priv = map_priv;
	tdm_dai_priv->regmap = map_priv->regmap;
	tdm_dai_priv->tdm_manage_priv = tdm_priv;

	platform_set_drvdata(pdev, tdm_dai_priv);

	tdm_priv->dev = &pdev->dev;
	tdm_priv->map_priv = map_priv;
	INIT_LIST_HEAD(&tdm_priv->substream_list);
	mutex_init(&tdm_priv->mutex);
	/* get tdm parameter from pdata */
	pdata = dev_get_platdata(&pdev->dev);
	tdm_priv->slot_size = pdata->slot_size;
	tdm_priv->slot_space = pdata->slot_space;
	tdm_priv->start_slot = pdata->start_slot;
	tdm_priv->clk_src = 26000000; /* 26M */
	tdm_priv->fsyn_pulse_width = pdata->fsyn_pulse_width;
	tdm_priv->use_4_wires = pdata->use_4_wires;
	ret = snd_soc_register_component(&pdev->dev, &mmp_map_be_tdm_component,
				mmp_tdm_dais, ARRAY_SIZE(mmp_tdm_dais));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register MAP tdm dai\n");
		return ret;
	}

	return ret;
}

static int mmp_tdm_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static struct platform_driver mmp_tdm_dai_driver = {
	.driver = {
		.name = "mmp-map-be-tdm",
		.owner = THIS_MODULE,
	},
	.probe = mmp_tdm_dai_probe,
	.remove = mmp_tdm_dai_remove,
};

module_platform_driver(mmp_tdm_dai_driver);

MODULE_AUTHOR("Nenghua Cao<nhcao@marvell.com>");
MODULE_DESCRIPTION("MMP MAP BE TDM DAI Interface");
