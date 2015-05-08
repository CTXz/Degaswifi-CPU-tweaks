/*
 * 88ce170.c  --  88CE170 ALSA Soc Audio driver
 *
 * Copyright (C) 2013 Marvell International Ltd.
 * Author: Yi Zeng <zengy@marvell.com>
 * Update: Zhao Ye <zhaoy@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/mfd/88ce170.h>

#include "88ce170-codec.h"

#define AUDIO_NAME      "88CE170"
#define DRIVER_VERSION  "1.00"
#define DRIVER_RELEASE_DATE     "Jan.07 2013"

#define USE_DAPM_CTRL_88CE170

struct ce170_private {
	struct snd_soc_codec *codec;

	enum snd_soc_control_type control_type;
	void *control_data;

#ifdef CONFIG_GPIOLIB
	struct work_struct work;
	unsigned gpio;
	unsigned det_hs;
	unsigned hs_det_status;
	unsigned det_mic;
	unsigned mic_det_status;
#endif

#if defined(CONFIG_INPUT) || defined(CONFIG_INPUT_MODULE)
	struct input_dev *input;
#endif

	struct regmap *regmap;
	struct ce170_chip *chip;

};

struct ce170_init_reg {
	char name[30];
	u8	reg_value;
	u8	reg_index;
};

#define CE170_INIT_REG_NUM ARRAY_SIZE(ce170_init_list)

static const u8 ce170_reg[CE170_CACHE_SIZE] = {
	0x00, 0x00, 0x00, 0x00,		/* Reg00 - Reg03 */
	0x00, 0x00, 0x00, 0x00,		/* Reg04 - Reg07 */
	0x40, 0x00, 0x00, 0x00,		/* Reg08 - Reg0b */
	0x00, 0x00, 0x00, 0x3f,		/* Reg0c - Reg0f */
	0x3f, 0x3f, 0x3f, 0x44,		/* Reg10 - Reg13 */
	0x00, 0x00, 0x00, 0x00,		/* Reg14 - Reg17 */
	0xa1, 0x00, 0x08, 0x82,		/* Reg18 - Reg1b */
	0x00, 0x00, 0x06, 0x00,		/* Reg1c - Reg1e */
	0x00,				/* filled */
	0x00, 0x00, 0x00, 0x00,		/* Reg20 - Reg23 */
	0x00, 0x00, 0x00, 0x00,		/* Reg24 - Reg27 */
	0x00, 0x00, 0xa0, 0x00,		/* Reg28 - Reg2b */
	0x00, 0x00, 0x00, 0x00,		/* Reg2c - Reg2f */
	0x00, 0x00, 0x00, 0x00,		/* Reg30 - Reg33 */
	0x00, 0x00, 0x00, 0x00,		/* Reg34 - Reg37 */
	0x20, 0x00, 0x00,		/* Reg38 - Reg3a */
	0x00,				/* filled */
	0x00, 0x00, 0x00, 0x00,		/* filled */
	0x00, 0x00, 0x00, 0x00,		/* Reg40 - Reg43 */
	0x00, 0x00, 0x00, 0x00,		/* Reg44 - Reg47 */
	0x03, 0xff, 0x12,		/* Reg48 - Reg4a */
	/* need to finish */
};

static int caps_charge = 2000;
module_param(caps_charge, int, 0);
MODULE_PARM_DESC(caps_charge, "88CE170 caps charge time (msecs)");

static unsigned int ce170_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	struct ce170_private *ce170_priv = snd_soc_codec_get_drvdata(codec);
	struct ce170_chip *chip = ce170_priv->chip;
	unsigned int value, ret;

	ret = regmap_read(chip->regmap, reg, &value);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read reg 0x%x: %d\n", reg, ret);
		return ret;
	}

	return value;
}

static int ce170_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	struct ce170_private *ce170_priv = snd_soc_codec_get_drvdata(codec);
	struct ce170_chip *chip = ce170_priv->chip;
	int ret;

	ret = regmap_write(chip->regmap, reg, value);
	if (ret < 0)
		dev_err(chip->dev, "Failed to write reg 0x%x: %d\n", reg, ret);

	return ret;

}

int ce170_hw_init(struct snd_soc_codec *codec)
{
	return 0;
}
EXPORT_SYMBOL_GPL(ce170_hw_init);

static int ce170_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	return 0;
}

int ce170_set_sample_rate(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	return 0;
}
EXPORT_SYMBOL_GPL(ce170_set_sample_rate);

static const struct snd_kcontrol_new ce170_snd_controls[] = {
	SOC_SINGLE("CE170_REVISION_CTRL", 0x00, 0, 0xff, 0),
	SOC_SINGLE("CE170_SYSTEM_REF_CLK_1", 0x01, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_PROGRAM_REG", 0x02, 0, 0xff, 0),
	SOC_SINGLE("CE170_SYSTEM_REF_CLK_2", 0x03, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_1", 0x04, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_2", 0x05, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_3", 0x06, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_4", 0x07, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_5", 0x08, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_OVERRIDE_REG_6", 0x09, 0, 0xff, 0),
	SOC_SINGLE("CE170_LRCLK_RATE_REG", 0x0a, 0, 0xff, 0),
	SOC_SINGLE("CE170_I2S1_CTRL_REG_1", 0x0b, 0, 0xff, 0),
	SOC_SINGLE("CE170_I2S_AUD_CTRL_REG_2", 0x0c, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL1_CTRL_REG_2", 0x0d, 0, 0xff, 0),
	SOC_SINGLE("CE170_STATUS_REG_1", 0x0e, 0, 0xff, 0),
	SOC_SINGLE("CE170_STATUS_REG_2", 0x0f, 0, 0xff, 0),
	SOC_SINGLE("CE170_STATUS_REG_3", 0x10, 0, 0xff, 0),
	SOC_SINGLE("CE170_OVERRIDE_STATUS_REG", 0x11, 0, 0xff, 0),
	SOC_SINGLE("CE170_PROM_WRITE_CTRL", 0x12, 0, 0xff, 0),
	SOC_SINGLE("CE170_PROM_DATA_MSB_HIGH", 0x13, 0, 0xff, 0),
	SOC_SINGLE("CE170_PROM_DATA_MSB_LOW", 0x14, 0, 0xff, 0),
	SOC_SINGLE("CE170_PROM_DATA_LSB_HIGH", 0x15, 0, 0xff, 0),
	SOC_SINGLE("CE170_PROM_DATA_LSB_LOW", 0x16, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_CTRL_REG_1", 0x17, 0, 0xff, 0),
	SOC_SINGLE("CE170_INTERRUPT_CTRL_REG_1", 0x18, 0, 0xff, 0),
	SOC_SINGLE("CE170_INTERRUPT_CTRL_REG_2", 0x19, 0, 0xff, 0),
	SOC_SINGLE("CE170_INTERRUPT_STATUS_REG_1", 0x1a, 0, 0xff, 0),
	SOC_SINGLE("CE170_INTERRUPT_STATUS_REG_2", 0x1b, 0, 0xff, 0),
	SOC_SINGLE("CE170_I2S2_CTRL_REG_1", 0x1c, 0, 0xff, 0),
	SOC_SINGLE("CE170_I2S2_CTRL_REG_2", 0x1d, 0, 0xff, 0),
	SOC_SINGLE("CE170_DMIC_CTRL_REG", 0x1e, 0, 0xff, 0),
	SOC_SINGLE("CE170_ASRC_CTRL_REG", 0x1f, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_PROCESSING_REG_1", 0x20, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_PROCESSING_REG_2", 0x21, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_CTRL_REG", 0x22, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_LEFT_VOLUME", 0x23, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_RIGHT_VOLUME", 0x24, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_VOL_UPDATE_TIME", 0x25, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_U_REG_2", 0x26, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_U_REG_1", 0x27, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_LAMBA_REG_2", 0x28, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_LAMBA_REG_1", 0x29, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_BETA_REG_2",  0x2a, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_BETA_REG_1",  0x2b, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_ERRTH_REG_2", 0x2c, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANC_PARAM_ERRTH_REG_1", 0x2d, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND1_GAIN",   0x2e, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND1_CENTER_FREQ_2", 0x2f, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND1_CENTER_FREQ_1", 0x30, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND2_GAIN",   0x31, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND2_CENTER_FREQ_2", 0x32, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND2_CENTER_FREQ_1", 0x33, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND2_BANDWIDTH_2",  0x34, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND2_BANDWIDTH_1",  0x35, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND3_GAIN",   0x36, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND3_CENTER_FREQ_2", 0x37, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND3_CENTER_FREQ_1", 0x38, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND3_BANDWIDTH_2",  0x39, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND3_BANDWIDTH_1",  0x3a, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND4_GAIN",   0x3b, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND4_CENTER_FREQ_2", 0x3c, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND4_CENTER_FREQ_1", 0x3d, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND4_BANDWIDTH_2",  0x3e, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND4_BANDWIDTH_1",  0x3f, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND5_GAIN",   0x40, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND5_CENTER_FREQ_2", 0x41, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND5_CENTER_FREQ_1", 0x42, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND5_BANDWIDTH_2",  0x43, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND5_BANDWIDTH_1",  0x44, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND6_GAIN",   0x45, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND6_CENTER_FREQ_2", 0x46, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND6_CENTER_FREQ_1", 0x47, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND6_BANDWIDTH_2",  0x48, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND6_BANDWIDTH_1",  0x49, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND7_GAIN",   0x4a, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND7_CENTER_FREQ_2", 0x4b, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND7_CENTER_FREQ_1", 0x4c, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND7_BANDWIDTH_2",  0x4d, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND7_BANDWIDTH_1",  0x4e, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND8_GAIN",   0x4f, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND8_CENTER_FREQ_2", 0x50, 0, 0xff, 0),
	SOC_SINGLE("CE170_EQ_BAND8_CENTER_FREQ_1", 0x51, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_THRESHOLD_2",   0x52, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_THRESHOLD_1",   0x53, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_OFFSET_2",   0x54, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_OFFSET_1",   0x55, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_COMPRESSION_RATIO", 0x56, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_ENERGY_ALPHA_REG_2", 0x57, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_ENERGY_ALPHA_REG_1", 0x58, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_ATTACK_ALPHA_REG_2", 0x59, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_ATTACK_ALPHA_REG_1", 0x5a, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_DECAY_ALPHA_REG_2", 0x5b, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DRC_DECAY_ALPHA_REG_1", 0x5c, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_LEFT_TO_LEFT_MIX", 0x5d, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_RIGHT_TO_LEFT_MIX", 0x5e, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_LEFT_TO_RIGHT_MIX", 0x5f, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_RIGHT_TO_RIGHT_MIX", 0x60, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_L_DUMMY_1", 0x61, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_R_DUMMY_1", 0x62, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_C2_DUMMY_1", 0x63, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_DATA_TO_TXRX_MIX_COEF_REG", 0x64, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_DATA_TO_TXRX_MIX_COEF_REG", 0x65, 0, 0xff, 0),
	SOC_SINGLE("CE170_DIN_TO_ASRC_MIX_COEF_REG",   0x66, 0, 0xff, 0),
	SOC_SINGLE("CE170_ASRC_OUTPUT_TO_ASRC_MIX_COEF_REG", 0x67, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_C2_DUMMY_2", 0x68, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_C1_DUMMY_1", 0x69, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1_L_DUMMY_2", 0x6a, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP_R_DUMMY_2", 0x6b, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP_DWA_CTRL_REG", 0x6c, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSM_SCALING_REG", 0x6d, 0, 0xff, 0),
	SOC_SINGLE("CE170_PRESERVED_REG_6e", 0x6e, 0, 0xff, 0),
	SOC_SINGLE("CE170_PRESERVED_REG_6f", 0x6f, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_PROCESSING_REG_1", 0x70, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_PROCESSING_REG_2", 0x71, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_CTRL_REG", 0x72, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_LEFT_VOLUME", 0x73, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_RIGHT_VOLUME", 0x74, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_VOLUME_UPDATE_TIME", 0x75, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_UPPER_THRESHOLD_2", 0x76, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_UPPER_THRESHOLD_1", 0x77, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_LOWER_THRESHOLD_2", 0x78, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_LOWER_THRESHOLD_1", 0x79, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_OFFSET_2", 0x7a, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_OFFSET_1", 0x7b, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_COMPRESSION_RATIO", 0x7c, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_ENERGY_ALPHA_REG_2", 0x7d, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_ENERGY_ALPHA_REG_1", 0x7e, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_ATTACK_ALPHA_REG_2", 0x7f, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_ATTACK_ALPHA_REG_1", 0x80, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_DECAY_ALPHA_REG_2", 0x81, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ALC_DECAY_ALPHA_REG_1", 0x82, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_NOISE_GATE_THRESHOD_2", 0x83, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_NOISE_GATE_THRESHOD_1", 0x84, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_LEFT_TO_LEFT_MIX", 0x85, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_RIGHT_TO_LEFT_MIX", 0x86, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_LEFT_TO_RIGHT_MIX", 0x87, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_RIGHT_TO_RIGHT_MIX", 0x88, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_U_REG_2", 0x89, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_U_REG_1", 0x8a, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_LAMBA_REG_2", 0x8b, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_LAMBA_REG_1", 0x8c, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_BETA_REG_2", 0x8d, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_BETA_REG_1", 0x8e, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_ERR_TH_REG_2", 0x8f, 0, 0xff, 0),
	SOC_SINGLE("CE170_AEC_PARAM_ERR_TH_REG_1", 0x90, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1A_L_DUMMY_1", 0x91, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1A_R_DUMMY_1", 0x92, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1A_C1_DUMMY_1", 0x93, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1A_C2_DUMMY_1", 0x94, 0, 0xff, 0),
	SOC_SINGLE("CE170_DSP1A_C2_DUMMY_2", 0x95, 0, 0xff, 0),
	SOC_SINGLE("CE170_SSL_PARAM_MU", 0x96, 0, 0xff, 0),
	SOC_SINGLE("CE170_SSL_PARAM_PHI", 0x97, 0, 0xff, 0),
	SOC_SINGLE("CE170_AMIC_TO_AMIC_DMIC_MIX", 0x98, 0, 0xff, 0),
	SOC_SINGLE("CE170_DMIC_TO_AMIC_DMIC_MIX", 0x99, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_ALPHA_REG_MSB", 0x9a, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_ALPHA_REG_LSB", 0x9b, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_BETA_REG", 0x9c, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_NETA_REG", 0x9d, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_K_REG", 0x9e, 0, 0xff, 0),
	SOC_SINGLE("CE170_BF_PARAM_COUNTER_LIMIT_REG", 0x9f, 0, 0xff, 0),
	SOC_SINGLE("CE170_DIGITAL_TEST_MUX_DATA_CTRL_REG", 0xa0, 0, 0xff, 0),
	SOC_SINGLE("CE170_DIGITAL_TEST_MUX_CLK_CTRL_ANA_TEST_MUX_REG",
		0xa1, 0, 0xff, 0),
	SOC_SINGLE("CE170_LOOPBACK_MODES", 0xa2, 0, 0xff, 0),
	SOC_SINGLE("CE170_I2S_FIFO_RD_CNT_LMT", 0xa3, 0, 0xff, 0),
	SOC_SINGLE("CE170_INPUT_DELAY_BUF_LEN", 0xa4, 0, 0xff, 0),
	SOC_SINGLE("CE170_INPUT_DELAY_BUF_SEL", 0xa5, 0, 0xff, 0),
	SOC_SINGLE("CE170_RECORD_TST1", 0xa6, 0, 0xff, 0),
	SOC_SINGLE("CE170_RECORD_TST2", 0xa7, 0, 0xff, 0),
	SOC_SINGLE("CE170_TST_SEL1", 0xa8, 0, 0xff, 0),
	SOC_SINGLE("CE170_TST_SEL2", 0xa9, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANA_MISC_CLK_GATE_REG", 0xaa, 0, 0xff, 0),
	SOC_SINGLE("CE170_MBIST_CTRL_REG", 0xab, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL_TEST_MONITOR_REG", 0xac, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL_RSVD_REG", 0xad, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_RSVD_REG", 0xae, 0, 0xff, 0),
	SOC_SINGLE("CE170_PRESERVED_REG_af", 0xaf, 0, 0xff, 0),
	SOC_SINGLE("CE170_CHARGEPUMP_REG", 0xb0, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLKGEN1", 0xb1, 0, 0xff, 0),
	SOC_SINGLE("CE170_MIC1_CTRL", 0xb2, 0, 0xff, 0),
	SOC_SINGLE("CE170_MIC2_CTRL", 0xb3, 0, 0xff, 0),
	SOC_SINGLE("CE170_MIC1_PGA_GAIN", 0xb4, 0, 0xff, 0),
	SOC_SINGLE("CE170_MIC2_PGA_GAIN", 0xb5, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC1_PGA_GAIN", 0xb6, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC2_PGA_GAIN", 0xb7, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANALOG_PATH_SEL", 0xb8, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_RSVD", 0xb9, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_HS_CTRL", 0xba, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_SPKR_CTRL", 0xbb, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_ANA_MISC", 0xbc, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANA_PWR_CLK", 0xbd, 0, 0xff, 0),
	SOC_SINGLE("CE170_AUDIO_PWR_ENABLE", 0xbe, 0, 0xff, 0),
	SOC_SINGLE("CE170_ADC_ANA_ENABLE", 0xbf, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_ANA_ENABLE", 0xc0, 0, 0xff, 0),
	SOC_SINGLE("CE170_AUDIO_CAL_1", 0xc1, 0, 0xff, 0),
	SOC_SINGLE("CE170_OFFSET_CAL_2", 0xc2, 0, 0xff, 0),
	SOC_SINGLE("CE170_OFFSET_CAL_3", 0xc3, 0, 0xff, 0),
	SOC_SINGLE("CE170_OFFSET_CAL_4", 0xc4, 0, 0xff, 0),
	SOC_SINGLE("CE170_OFFSET_CAL_5", 0xc5, 0, 0xff, 0),
	SOC_SINGLE("CE170_DAC_INPUT_SEL", 0xc6, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANA_RSVD", 0xc7, 0, 0xff, 0),
	SOC_SINGLE("CE170_HS_MIC_DET", 0xc8, 0, 0xff, 0),
	SOC_SINGLE("CE170_STATUS_1", 0xc9, 0, 0xff, 0),
	SOC_SINGLE("CE170_ANA_RSVD_OUT", 0xca, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASSG_CP1", 0xcb, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASSG_CTRL", 0xcc, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASSG_CP3", 0xcd, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASSG_CP2", 0xce, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASSD_CTRL", 0xcf, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACL_OFST_LSB", 0xd0, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACL_OFST_MSB", 0xd1, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACR_OFST_LSB", 0xd2, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACRLOFST_MSB", 0xd3, 0, 0xff, 0),
	SOC_SINGLE("CE170_PAD_SLEW_RATE", 0xd4, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACL_OFST_STATUS_LSB", 0xd5, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACL_OFST_STATUS_MSB", 0xd6, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACR_OFST_STATUS_MSB", 0xd7, 0, 0xff, 0),
	SOC_SINGLE("CE170_DACR_OFST_STATUS_LSB", 0xd8, 0, 0xff, 0),
	SOC_SINGLE("CE170_HS_SPK_TEST", 0xd9, 0, 0xff, 0),
	SOC_SINGLE("CE170_EP_CTRL", 0xda, 0, 0xff, 0),
	SOC_SINGLE("CE170_ACF_CTRL", 0xdb, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_1", 0xdc, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_2", 0xdd, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_3", 0xde, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_4", 0xdf, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_5", 0xe0, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_6", 0xe1, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_7", 0xe2, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MIN_8", 0xe3, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_1", 0xe4, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_2", 0xe5, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_3", 0xe6, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_4", 0xe7, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_6", 0xe8, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_7", 0xe9, 0, 0xff, 0),
	SOC_SINGLE("CE170_CLASS_G_LMT_MAX_8", 0xea, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_PROGRAM_REG", 0xeb, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_1", 0xec, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_2", 0xed, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_3", 0xee, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_4", 0xef, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_5", 0xf0, 0, 0xff, 0),
	SOC_SINGLE("CE170_PLL2_OVERRIDE_REG_6", 0xf1, 0, 0xff, 0),
};

static int ce170_set_dai_pll(struct snd_soc_dai *codec_dai,
			      int pll_id, int source,
			      unsigned int freq_in,
			      unsigned int freq_out)
{
	return 0;
}

static int ce170_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int ce170_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int ce170_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int ce170_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

/*
 * 88CE170 can support sample rate 12000 and 24000, if let ASOC support
 * these two rates, you need to change include/sound/pcm.h and
 * sound/core/pcm_native.c
 */
#define CE170_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | \
	SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_176400)

#define CE170_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops ce170_dai_ops = {
	.hw_params	= ce170_hw_params,
	.digital_mute	= ce170_mute,
	.set_fmt	= ce170_set_dai_fmt,
	.set_sysclk	= ce170_set_dai_sysclk,
	.set_pll	= ce170_set_dai_pll,
};

struct snd_soc_dai_driver ce170_dai[2] = {
	/* hifi codec dai */
	{
		.name = "88ce170-hifi-dai",
		.id = 1,
		.playback = {
			.stream_name  = "Hifi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates	      = CE170_RATES,
			.formats      = CE170_FORMATS,
		},
		.capture = {
			.stream_name  = "Hifi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = CE170_RATES,
			.formats      = CE170_FORMATS,
		},
		.ops = &ce170_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "88ce170-voice-dai",
		.id = 2,
		.playback = {
			.stream_name  = "Voice Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = CE170_RATES,
			.formats      = CE170_FORMATS,
		},
		.capture = {
			.stream_name  = "Voice Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = CE170_RATES,
			.formats      = CE170_FORMATS,
		},
		.ops = &ce170_dai_ops,
		.symmetric_rates = 1,
	}
};

static void ce170_work(struct work_struct *work)
{
	struct snd_soc_dapm_context *dapm =
		container_of(work, struct snd_soc_dapm_context,
			delayed_work.work);
	struct snd_soc_codec *codec = dapm->codec;
	ce170_set_bias_level(codec, dapm->bias_level);
}

static int ce170_remove(struct snd_soc_codec *codec)
{
	ce170_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

#ifdef CONFIG_PM
static int ce170_suspend(struct snd_soc_codec *codec)
{

	ce170_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int ce170_resume(struct snd_soc_codec *codec)
{

	ce170_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}
#else
static int ce170_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int ce170_resume(struct snd_soc_codec *codec)
{
	return 0;
}
#endif

static int ce170_probe(struct snd_soc_codec *codec)
{
	struct ce170_private *ce170_priv = snd_soc_codec_get_drvdata(codec);
	struct ce170_chip *chip = ce170_priv->chip;
	int ret = 0;

	INIT_DELAYED_WORK(&codec->dapm.delayed_work, ce170_work);

	codec->control_data = chip;

	ce170_priv->codec = codec;

	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	ce170_set_bias_level(codec, SND_SOC_BIAS_PREPARE);
	codec->dapm.bias_level = SND_SOC_BIAS_STANDBY;
	schedule_delayed_work(&codec->dapm.delayed_work,
				msecs_to_jiffies(caps_charge));

	return ret;
}

struct snd_soc_codec_driver soc_codec_dev_ce170 = {
	.probe   = ce170_probe,
	.remove  = ce170_remove,
	.suspend = ce170_suspend,
	.resume  = ce170_resume,
	.read = ce170_read,
	.write = ce170_write,
	.set_bias_level = ce170_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(ce170_reg),
	.reg_cache_step = 1,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = ce170_reg,
	.controls = ce170_snd_controls,
	.num_controls = ARRAY_SIZE(ce170_snd_controls),
};

static struct ce170_private *ce170_private;

static int ce170_codec_probe(struct platform_device *pdev)
{
	struct ce170_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct ce170_private *ce170_priv;
	int ret = 0;

	ce170_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct ce170_private), GFP_KERNEL);
	if (ce170_priv == NULL)
		return -ENOMEM;

	ce170_private = ce170_priv;
	ce170_priv->chip = chip;
	ce170_priv->regmap = chip->regmap;

	platform_set_drvdata(pdev, ce170_priv);

	ret = snd_soc_register_codec(&pdev->dev,
		&soc_codec_dev_ce170, ce170_dai, ARRAY_SIZE(ce170_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register codec 88ce170\n");
		goto err_out1;
	}

	return ret;

err_out1:
	platform_set_drvdata(pdev, NULL);

	return ret;
}

static int ce170_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ce170_codec_driver = {
	.probe		= ce170_codec_probe,
	.remove		= ce170_codec_remove,
	.driver		= {
		.name	= "88ce170-codec",
		.owner	= THIS_MODULE,
	},
};

static int ce170_init(void)
{
	return platform_driver_register(&ce170_codec_driver);
}

static void ce170_exit(void)
{
	platform_driver_unregister(&ce170_codec_driver);
}

module_init(ce170_init);
module_exit(ce170_exit);

MODULE_DESCRIPTION("ASoc Marvell 88CE170 driver");
MODULE_AUTHOR("Yi Zeng <zengy@marvell.com>");
MODULE_LICENSE("GPL");
