/*
 * mmp-map-fe-dai.c  --  MAP(MARVELL AUDIO PROCESSOR) FE DAI driver
 *
 * Copyright (C) 2014 Marvell International Ltd.
 * Author: Nenghua Cao <nhcao@marvell.com>
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
#include <linux/clk.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/mfd/mmp-map.h>

struct map_fe_dai_private {
	struct snd_soc_codec *codec;
	struct regmap *regmap;
	enum	snd_soc_control_type control_type;
	void	*control_data;
	struct	proc_dir_entry *proc_file;
	/* point to mmp-map */
	struct map_private *map_priv;
};

static const struct snd_kcontrol_new map_snd_controls[] = {
	SND_SOC_BYTES("MAP_REVISION", MAP_REV, 1),
	SND_SOC_BYTES("MAP_LRCLK_RATE_REG", MAP_LRCLK_RATE_REG, 1),
	SND_SOC_BYTES("MAP_I2S1_CTRL_REG", MAP_I2S1_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_I2S2_CTRL_REG", MAP_I2S2_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_I2S3_CTRL_REG", MAP_I2S3_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_I2S4_CTRL_REG", MAP_I2S4_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_DEI2S_CTRL_REG", MAP_DEI2S_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_STATUS_REG_1", MAP_STATUS_REG_1, 1),
	SND_SOC_BYTES("MAP_STATUS_REG_2", MAP_STATUS_REG_2, 1),
	SND_SOC_BYTES("MAP_TOP_CTRL_REG_1", MAP_TOP_CTRL_REG_1, 1),
	SND_SOC_BYTES("MAP_TOP_CTRL_REG_2", MAP_TOP_CTRL_REG_2, 1),
	SND_SOC_BYTES("MAP_DATAPATH_FLOW_CTRL_REG_1",
			MAP_DATAPATH_FLOW_CTRL_REG_1, 1),
	SND_SOC_BYTES("MAP_DATAPATH_FLOW_CTRL_REG_2",
			MAP_DATAPATH_FLOW_CTRL_REG_2, 1),
	SND_SOC_BYTES("MAP_DATAPATH_FLOW_CTRL_REG_3",
			MAP_DATAPATH_FLOW_CTRL_REG_3, 1),
	SND_SOC_BYTES("MAP_ASRC_CTRL_REG", MAP_ASRC_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_1", MAP_TDM_CTRL_REG_1, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_2", MAP_TDM_CTRL_REG_2, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_3", MAP_TDM_CTRL_REG_3, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_4", MAP_TDM_CTRL_REG_4, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_5", MAP_TDM_CTRL_REG_5, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_6", MAP_TDM_CTRL_REG_6, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_7", MAP_TDM_CTRL_REG_7, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_8", MAP_TDM_CTRL_REG_8, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_9", MAP_TDM_CTRL_REG_9, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_10", MAP_TDM_CTRL_REG_10, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_11", MAP_TDM_CTRL_REG_11, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_12", MAP_TDM_CTRL_REG_12, 1),
	SND_SOC_BYTES("MAP_TDM_CTRL_REG_13", MAP_TDM_CTRL_REG_13, 1),
	SND_SOC_BYTES("MAP_INTERRUPT_CTRL_REG", MAP_INTERRUPT_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_I2S1_BCLK_DIV", MAP_I2S1_BCLK_DIV, 1),
	SND_SOC_BYTES("MAP_I2S2_BCLK_DIV", MAP_I2S2_BCLK_DIV, 1),
	SND_SOC_BYTES("MAP_I2S3_BCLK_DIV", MAP_I2S3_BCLK_DIV, 1),
	SND_SOC_BYTES("MAP_I2S4_BCLK_DIV", MAP_I2S4_BCLK_DIV, 1),
	SND_SOC_BYTES("MAP_I2S_OUT_BCLK_DIV", MAP_I2S_OUT_BCLK_DIV, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_PROCESSING_REG",
			MAP_DSP1_DAC_PROCESSING_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_CTRL_REG",
			MAP_DSP1_DAC_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_VOLUME",
			MAP_DSP1_DAC_VOLUME, 1),
	SND_SOC_BYTES("MAP_DSP1_ANC_PARAM_U_REG",
			MAP_DSP1_ANC_PARAM_U_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_ANC_PARAM_LAMBA_REG",
			MAP_DSP1_ANC_PARAM_LAMBA_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_ANC_PARAM_BETA_REG",
			MAP_DSP1_ANC_PARAM_BETA_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_ANC_PARAM_ERRTH_REG",
			MAP_DSP1_ANC_PARAM_ERRTH_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND1_GAIN",
			MAP_DSP1_EQ_BAND1_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND1_CENTER_FREQ",
			MAP_DSP1_EQ_BAND1_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND2_GAIN",
			MAP_DSP1_EQ_BAND2_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND2_CENTER_FREQ",
			MAP_DSP1_EQ_BAND2_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND2_BANDWIDTH",
			MAP_DSP1_EQ_BAND2_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND3_GAIN",
			MAP_DSP1_EQ_BAND3_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND3_CENTER_FREQ",
			MAP_DSP1_EQ_BAND3_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND3_BANDWIDTH",
			MAP_DSP1_EQ_BAND3_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND4_GAIN",
			MAP_DSP1_EQ_BAND4_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND4_CENTER_FREQ",
			MAP_DSP1_EQ_BAND4_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND4_BANDWIDTH",
			MAP_DSP1_EQ_BAND4_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND5_GAIN",
			MAP_DSP1_EQ_BAND5_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND5_CENTER_FREQ",
			MAP_DSP1_EQ_BAND5_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND5_BANDWIDTH",
			MAP_DSP1_EQ_BAND5_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND6_GAIN",
			MAP_DSP1_EQ_BAND6_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND6_CENTER_FREQ",
			MAP_DSP1_EQ_BAND6_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND6_BANDWIDTH",
			MAP_DSP1_EQ_BAND6_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND7_GAIN",
			MAP_DSP1_EQ_BAND7_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND7_CENTER_FREQ",
			MAP_DSP1_EQ_BAND7_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND7_BANDWIDTH",
			MAP_DSP1_EQ_BAND7_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND8_GAIN",
			MAP_DSP1_EQ_BAND8_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP1_EQ_BAND8_CENTER_FREQ",
			MAP_DSP1_EQ_BAND8_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_THRESHOLD",
			MAP_DSP1_DAC_DRC_THRESHOLD, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_OFFSET",
			MAP_DSP1_DAC_DRC_OFFSET, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_COMPRESSION_RATIO",
			MAP_DSP1_DAC_DRC_COMPRESSION_RATIO, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_ENERGY_ALPHA_REG",
			MAP_DSP1_DAC_DRC_ENERGY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_ATTACK_ALPHA_REG",
			MAP_DSP1_DAC_DRC_ATTACK_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_DRC_DECAY_ALPHA_REG",
			MAP_DSP1_DAC_DRC_DECAY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_DAC_OUTPUT_MIX",
			MAP_DSP1_DAC_OUTPUT_MIX, 1),
	SND_SOC_BYTES("MAP_DSP1_TXRX_MIX_COEF_REG",
			MAP_DSP1_TXRX_MIX_COEF_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_INMIX_COEF_REG",
			MAP_DSP1_INMIX_COEF_REG, 1),
	SND_SOC_BYTES("MAP_DSP1_3D_REG1",
			MAP_DSP1_3D_REG1, 1),
	SND_SOC_BYTES("MAP_DSP1_3D_REG2",
			MAP_DSP1_3D_REG2, 1),
	SND_SOC_BYTES("MAP_DSP1_DUMMY_1",
			MAP_DSP1_DUMMY_1, 1),
	SND_SOC_BYTES("MAP_DSP1_DUMMY_2",
			MAP_DSP1_DUMMY_2, 1),
	SND_SOC_BYTES("MAP_DSP1_DUMMY_3",
			MAP_DSP1_DUMMY_3, 1),
	SND_SOC_BYTES("MAP_DSP1_DUMMY_4",
			MAP_DSP1_DUMMY_4, 1),
	SND_SOC_BYTES("MAP_DSP1_DUMMY_5",
			MAP_DSP1_DUMMY_5, 1),
	SND_SOC_BYTES("MAP_DSP1_DSM_SCALING_REG",
			MAP_DSP1_DSM_SCALING_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_PROCESSING_REG",
			MAP_DSP2_DAC_PROCESSING_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_CTRL_REG",
			MAP_DSP2_DAC_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_VOLUME",
			MAP_DSP2_DAC_VOLUME, 1),
	SND_SOC_BYTES("MAP_DSP2_ANC_PARAM_U_REG",
			MAP_DSP2_ANC_PARAM_U_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_ANC_PARAM_LAMBA_REG",
			MAP_DSP2_ANC_PARAM_LAMBA_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_ANC_PARAM_BETA_REG",
			MAP_DSP2_ANC_PARAM_BETA_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_ANC_PARAM_ERRTH_REG",
			MAP_DSP2_ANC_PARAM_ERRTH_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND1_GAIN",
			MAP_DSP2_EQ_BAND1_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND1_CENTER_FREQ",
			MAP_DSP2_EQ_BAND1_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND2_GAIN",
			MAP_DSP2_EQ_BAND2_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND2_CENTER_FREQ",
			MAP_DSP2_EQ_BAND2_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND2_BANDWIDTH",
			MAP_DSP2_EQ_BAND2_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND3_GAIN",
			MAP_DSP2_EQ_BAND3_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND3_CENTER_FREQ",
			MAP_DSP2_EQ_BAND3_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND3_BANDWIDTH",
			MAP_DSP2_EQ_BAND3_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND4_GAIN",
			MAP_DSP2_EQ_BAND4_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND4_CENTER_FREQ",
			MAP_DSP2_EQ_BAND4_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND4_BANDWIDTH",
			MAP_DSP2_EQ_BAND4_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND5_GAIN",
			MAP_DSP2_EQ_BAND5_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND5_CENTER_FREQ",
			MAP_DSP2_EQ_BAND5_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND5_BANDWIDTH",
			MAP_DSP2_EQ_BAND5_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND6_GAIN",
			MAP_DSP2_EQ_BAND6_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND6_CENTER_FREQ",
			MAP_DSP2_EQ_BAND6_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND6_BANDWIDTH",
			MAP_DSP2_EQ_BAND6_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND7_GAIN",
			MAP_DSP2_EQ_BAND7_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND7_CENTER_FREQ",
			MAP_DSP2_EQ_BAND7_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND7_BANDWIDTH",
			MAP_DSP2_EQ_BAND7_BANDWIDTH, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND8_GAIN",
			MAP_DSP2_EQ_BAND8_GAIN, 1),
	SND_SOC_BYTES("MAP_DSP2_EQ_BAND8_CENTER_FREQ",
			MAP_DSP2_EQ_BAND8_CENTER_FREQ, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_THRESHOLD",
			MAP_DSP2_DAC_DRC_THRESHOLD, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_OFFSET",
			MAP_DSP2_DAC_DRC_OFFSET, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_COMPRESSION_RATIO",
			MAP_DSP2_DAC_DRC_COMPRESSION_RATIO, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_ENERGY_ALPHA_REG",
			MAP_DSP2_DAC_DRC_ENERGY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_ATTACK_ALPHA_REG",
			MAP_DSP2_DAC_DRC_ATTACK_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_DRC_DECAY_ALPHA_REG",
			MAP_DSP2_DAC_DRC_DECAY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_DAC_OUTPUT_MIX",
			MAP_DSP2_DAC_OUTPUT_MIX, 1),
	SND_SOC_BYTES("MAP_DSP2_TXRX_MIX_COEF_REG",
			MAP_DSP2_TXRX_MIX_COEF_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_INMIX_COEF_REG",
			MAP_DSP2_INMIX_COEF_REG, 1),
	SND_SOC_BYTES("MAP_DSP2_3D_REG1",
			MAP_DSP2_3D_REG1, 1),
	SND_SOC_BYTES("MAP_DSP2_3D_REG2",
			MAP_DSP2_3D_REG2, 1),
	SND_SOC_BYTES("MAP_DSP2_DUMMY_1",
			MAP_DSP2_DUMMY_1, 1),
	SND_SOC_BYTES("MAP_DSP2_DUMMY_2",
			MAP_DSP2_DUMMY_2, 1),
	SND_SOC_BYTES("MAP_DSP2_DUMMY_3",
			MAP_DSP2_DUMMY_3, 1),
	SND_SOC_BYTES("MAP_DSP2_DUMMY_4",
			MAP_DSP2_DUMMY_4, 1),
	SND_SOC_BYTES("MAP_DSP2_DUMMY_5",
			MAP_DSP2_DUMMY_5, 1),
	SND_SOC_BYTES("MAP_DSP2_DSM_SCALING_REG",
			MAP_DSP2_DSM_SCALING_REG, 1),
	SND_SOC_BYTES("MAP_ADC_PROCESSING_REG",
			MAP_ADC_PROCESSING_REG, 1),
	SND_SOC_BYTES("MAP_ADC_CTRL_REG",
			MAP_ADC_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_ADC_VOLUME",
			MAP_ADC_VOLUME, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_UPPER_THRESHOLD",
			MAP_ADC_ALC_UPPER_THRESHOLD, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_LOWER_THRESHOLD",
			MAP_ADC_ALC_LOWER_THRESHOLD, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_OFFSET",
			MAP_ADC_ALC_OFFSET, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_COMPRESSION_RATIO",
			MAP_ADC_ALC_COMPRESSION_RATIO, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_ENERGY_ALPHA_REG",
			MAP_ADC_ALC_ENERGY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_ATTACK_ALPHA_REG",
			MAP_ADC_ALC_ATTACK_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_ADC_ALC_DECAY_ALPHA_REG",
			MAP_ADC_ALC_DECAY_ALPHA_REG, 1),
	SND_SOC_BYTES("MAP_ADC_NOISE_GATE_THRESHOD",
			MAP_ADC_NOISE_GATE_THRESHOD, 1),
	SND_SOC_BYTES("MAP_ADC_OUTPUT_MIX", MAP_ADC_OUTPUT_MIX, 1),
	SND_SOC_BYTES("MAP_AEC_PARAM_U_REG", MAP_AEC_PARAM_U_REG, 1),
	SND_SOC_BYTES("MAP_AEC_PARAM_LAMBA_REG", MAP_AEC_PARAM_LAMBA_REG, 1),
	SND_SOC_BYTES("MAP_AEC_PARAM_BETA_REG", MAP_AEC_PARAM_BETA_REG, 1),
	SND_SOC_BYTES("MAP_AEC_PARAM_ERR_TH_REG", MAP_AEC_PARAM_ERR_TH_REG, 1),
	SND_SOC_BYTES("MAP_WNR_FILTER_COEF", MAP_WNR_FILTER_COEF, 1),
	SND_SOC_BYTES("MAP_SSL_PARAM_MU", MAP_SSL_PARAM_MU, 1),
	SND_SOC_BYTES("MAP_INPUT_MIX_REG", MAP_INPUT_MIX_REG, 1),
	SND_SOC_BYTES("MAP_BF_PARAM_REG1", MAP_BF_PARAM_REG1, 1),
	SND_SOC_BYTES("MAP_BF_PARAM_REG2", MAP_BF_PARAM_REG2, 1),
	SND_SOC_BYTES("MAP_DSP1A_DUMMY_1", MAP_DSP1A_DUMMY_1, 1),
	SND_SOC_BYTES("MAP_DSP1A_DUMMY_2", MAP_DSP1A_DUMMY_2, 1),
	SND_SOC_BYTES("MAP_DSP1A_DUMMY_3", MAP_DSP1A_DUMMY_3, 1),
	SND_SOC_BYTES("MAP_DIG_TEST_MUX_CTRL_REG",
			MAP_DIG_TEST_MUX_CTRL_REG, 1),
	SND_SOC_BYTES("MAP_LOOPBACK_MODES", MAP_LOOPBACK_MODES, 1),
	SND_SOC_BYTES("MAP_DELAY_BUF_CTRL", MAP_DELAY_BUF_CTRL, 1),
	SND_SOC_BYTES("MAP_DAC_ANA_MISC", MAP_DAC_ANA_MISC, 1),
};

/* dac1 input mux selection */
/* d1in1 */
static const char * const map_dac_input_d1in1_mux_txt[] = {
	"AOUT1", "D1AIN1", "D1AIN2", "D2OUT"
};
static const struct soc_enum dac_input_d1in1_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			6,
			4,
			map_dac_input_d1in1_mux_txt);

static const struct snd_kcontrol_new dac_input_d1in1_mux =
	SOC_DAPM_ENUM("dac input d1in1 mux",
			dac_input_d1in1_mux_enum);
/* d1in2 */
static const char * const map_dac_input_d1in2_mux_txt[] = {
	"AOUT2", "D1AIN1", "D1AIN2", "D2OUT"
};
static const struct soc_enum dac_input_d1in2_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			4,
			4,
			map_dac_input_d1in2_mux_txt);

static const struct snd_kcontrol_new dac_input_d1in2_mux =
	SOC_DAPM_ENUM("dac input d1in2 mux",
			dac_input_d1in2_mux_enum);

/* d1in3 */
static const char * const map_dac_input_d1in3_mux_txt[] = {
	"AOUT3", "D1AIN1", "D1AIN2", "D2OUT"
};
static const struct soc_enum dac_input_d1in3_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			2,
			4,
			map_dac_input_d1in3_mux_txt);

static const struct snd_kcontrol_new dac_input_d1in3_mux =
	SOC_DAPM_ENUM("dac input d1in3 mux",
			dac_input_d1in3_mux_enum);

/* d1in4 */
static const char * const map_dac_input_d1in4_mux_txt[] = {
	"AOUT4", "D1AIN1", "D1AIN2", "D2OUT"
};
static const struct soc_enum dac_input_d1in4_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			0,
			4,
			map_dac_input_d1in4_mux_txt);

static const struct snd_kcontrol_new dac_input_d1in4_mux =
	SOC_DAPM_ENUM("dac input d1in4 mux",
			dac_input_d1in4_mux_enum);

/* dac2 input mux selection */
/* d2in1 */
static const char * const map_dac_input_d2in1_mux_txt[] = {
	"AOUT1", "D1AIN1", "D1AIN2", "D1OUT"
};
static const struct soc_enum dac_input_d2in1_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			14,
			4,
			map_dac_input_d2in1_mux_txt);

static const struct snd_kcontrol_new dac_input_d2in1_mux =
	SOC_DAPM_ENUM("dac input d2in1 mux",
			dac_input_d2in1_mux_enum);
/* d2in2 */
static const char * const map_dac_input_d2in2_mux_txt[] = {
	"AOUT2", "D1AIN1", "D1AIN2", "D1OUT"
};
static const struct soc_enum dac_input_d2in2_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			12,
			4,
			map_dac_input_d2in2_mux_txt);

static const struct snd_kcontrol_new dac_input_d2in2_mux =
	SOC_DAPM_ENUM("dac input d2in2 mux",
			dac_input_d2in2_mux_enum);

/* d2in3 */
static const char * const map_dac_input_d2in3_mux_txt[] = {
	"AOUT3", "D1AIN1", "D1AIN2", "D1OUT"
};
static const struct soc_enum dac_input_d2in3_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			10,
			4,
			map_dac_input_d2in3_mux_txt);

static const struct snd_kcontrol_new dac_input_d2in3_mux =
	SOC_DAPM_ENUM("dac input d2in3 mux",
			dac_input_d2in3_mux_enum);

/* d2in4 */
static const char * const map_dac_input_d2in4_mux_txt[] = {
	"AOUT4", "D1AIN1", "D1AIN2", "D1OUT"
};
static const struct soc_enum dac_input_d2in4_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			8,
			4,
			map_dac_input_d2in4_mux_txt);

static const struct snd_kcontrol_new dac_input_d2in4_mux =
	SOC_DAPM_ENUM("dac input d2in4 mux",
			dac_input_d2in4_mux_enum);

/* ADC output mux selection */
/* AIN1 */
static const char * const map_adc_output_ain_mux_txt[] = {
	"D1AOUT", "AOUT1", "AOUT2", "AOUT3", "AOUT4",
	"MIC1", "MIC2", "D1OUT", "D2OUT"
};
static const struct soc_enum adc_output_ain1_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			12,
			9,
			map_adc_output_ain_mux_txt);

static const struct snd_kcontrol_new adc_output_ain1_mux =
	SOC_DAPM_ENUM("adc output ain1 mux",
			adc_output_ain1_mux_enum);

/* AIN2 */
static const struct soc_enum adc_output_ain2_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			8,
			9,
			map_adc_output_ain_mux_txt);

static const struct snd_kcontrol_new adc_output_ain2_mux =
	SOC_DAPM_ENUM("adc output ain2 mux",
			adc_output_ain2_mux_enum);

/* AIN3 */
static const struct soc_enum adc_output_ain3_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			4,
			9,
			map_adc_output_ain_mux_txt);

static const struct snd_kcontrol_new adc_output_ain3_mux =
	SOC_DAPM_ENUM("adc output ain3 mux",
			adc_output_ain3_mux_enum);

/* AIN4 */
static const struct soc_enum adc_output_ain4_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			0,
			9,
			map_adc_output_ain_mux_txt);

static const struct snd_kcontrol_new adc_output_ain4_mux =
	SOC_DAPM_ENUM("adc output ain4 mux",
			adc_output_ain4_mux_enum);

/* DAC output mux */
/* OUT1 */
static const char * const map_dac_output_out1_mux_txt[] = {
	"D1OUT", "D1IN1", "D1IN2", "D1IN3", "D1IN4", "D2OUT", "DSPMIX"
};
static const struct soc_enum dac_output_out1_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			19,
			7,
			map_dac_output_out1_mux_txt);

static const struct snd_kcontrol_new dac_output_out1_mux =
	SOC_DAPM_ENUM("dac output out1 mux",
			dac_output_out1_mux_enum);

/* OUT2 */
static const char * const map_dac_output_out2_mux_txt[] = {
	"D2OUT", "D2IN1", "D2IN2", "D2IN3", "D2IN4", "D1OUT", "DSPMIX"
};
static const struct soc_enum dac_output_out2_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_1,
			16,
			7,
			map_dac_output_out2_mux_txt);

static const struct snd_kcontrol_new dac_output_out2_mux =
	SOC_DAPM_ENUM("dac output out2 mux",
			dac_output_out2_mux_enum);

/* dsp1 enable */
static const struct snd_kcontrol_new dsp1_en_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP1_DAC_CTRL_REG, 0, 3, 0);

static const struct snd_kcontrol_new dac1_d1in1_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP1_DAC_PROCESSING_REG, 9, 1, 0);

static const struct snd_kcontrol_new dac1_d1in2_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP1_DAC_PROCESSING_REG, 10, 1, 0);

static const struct snd_kcontrol_new dac1_d1in3_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP1_DAC_PROCESSING_REG, 11, 1, 0);

static const struct snd_kcontrol_new dac1_d1in4_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP1_DAC_PROCESSING_REG, 12, 1, 0);

/* DSP2 enable */
static const struct snd_kcontrol_new dsp2_en_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP2_DAC_CTRL_REG, 0, 3, 0);

static const struct snd_kcontrol_new dac2_d1in1_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP2_DAC_PROCESSING_REG, 9, 1, 0);

static const struct snd_kcontrol_new dac2_d1in2_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP2_DAC_PROCESSING_REG, 10, 1, 0);

static const struct snd_kcontrol_new dac2_d1in3_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP2_DAC_PROCESSING_REG, 11, 1, 0);

static const struct snd_kcontrol_new dac2_d1in4_inmix_control =
	SOC_DAPM_SINGLE("Switch", MAP_DSP2_DAC_PROCESSING_REG, 12, 1, 0);

/* virtual mux for inmix due to there is no control register */
static const char * const dac1_in_mux_text[] = {
	"d1in1",
	"in mix",
};

static const struct soc_enum dac1_in_mux_enum =
	SOC_ENUM_SINGLE(0, 0, 2, dac1_in_mux_text);

static const struct snd_kcontrol_new dac1_in_mux =
	SOC_DAPM_ENUM_VIRT("DAC1 in mux", dac1_in_mux_enum);

static const char * const dac2_in_mux_text[] = {
	"d2in1",
	"in mix",
};

static const struct soc_enum dac2_in_mux_enum =
	SOC_ENUM_SINGLE(0, 0, 2, dac2_in_mux_text);

static const struct snd_kcontrol_new dac2_in_mux =
	SOC_DAPM_ENUM_VIRT("DAC2 in mux", dac2_in_mux_enum);

/* DSP1 TXRX mux: 0 for not enable txrx mixer, 1 for enable it */
static const char * const map_dac1_txrx_mux_txt[] = {
	"inmix", "txrx"
};
static const struct soc_enum dac1_txrx_mux_enum =
	SOC_ENUM_SINGLE(MAP_DSP1_DAC_PROCESSING_REG,
			13,
			2,
			map_dac1_txrx_mux_txt);

static const struct snd_kcontrol_new dac1_txrx_mux =
	SOC_DAPM_ENUM("dac1 txrx mux",
			dac1_txrx_mux_enum);

/* DSP1 out mixer */
static const char * const map_dac1_outmix_mux_txt[] = {
	"out_L_R", "out_L+R"
};
static const struct soc_enum dac1_outmix_mux_enum =
	SOC_ENUM_SINGLE(MAP_DSP1_DAC_PROCESSING_REG,
			8,
			2,
			map_dac1_outmix_mux_txt);

static const struct snd_kcontrol_new dac1_outmix_mux =
	SOC_DAPM_ENUM("dac1 outmix mux",
			dac1_outmix_mux_enum);

/* DSP2 TXRX mux: 0 for not enable txrx mixer, 1 for enable it */
static const char * const map_dac2_txrx_mux_txt[] = {
	"inmix", "txrx"
};
static const struct soc_enum dac2_txrx_mux_enum =
	SOC_ENUM_SINGLE(MAP_DSP2_DAC_PROCESSING_REG,
			13,
			2,
			map_dac2_txrx_mux_txt);

static const struct snd_kcontrol_new dac2_txrx_mux =
	SOC_DAPM_ENUM("dac2 txrx mux",
			dac2_txrx_mux_enum);

/* DSP2 out mixer */
static const char * const map_dac2_outmix_mux_txt[] = {
	"out_L_R", "out_L+R"
};
static const struct soc_enum dac2_outmix_mux_enum =
	SOC_ENUM_SINGLE(MAP_DSP2_DAC_PROCESSING_REG,
			8,
			2,
			map_dac2_outmix_mux_txt);

static const struct snd_kcontrol_new dac2_outmix_mux =
	SOC_DAPM_ENUM("dac2 outmix mux",
			dac2_outmix_mux_enum);

/* ADC input mux */
/* D1AIN1 */
static const char * const map_adc_input_d1ain1_mux_txt[] = {
	"MIC1", "AOUT1", "AOUT2", "AOUT3", "AOUT4"
};
static const struct soc_enum adc_input_d1ain1_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			19,
			5,
			map_adc_input_d1ain1_mux_txt);

static const struct snd_kcontrol_new adc_input_d1ain1_mux =
	SOC_DAPM_ENUM("adc input d1ain1 mux",
			adc_input_d1ain1_mux_enum);

/* D1AIN2 */
static const char * const map_adc_input_d1ain2_mux_txt[] = {
	"MIC2", "AOUT1", "AOUT2", "AOUT3", "AOUT4"
};
static const struct soc_enum adc_input_d1ain2_mux_enum =
	SOC_ENUM_SINGLE(MAP_DATAPATH_FLOW_CTRL_REG_2,
			16,
			5,
			map_adc_input_d1ain2_mux_txt);

static const struct snd_kcontrol_new adc_input_d1ain2_mux =
	SOC_DAPM_ENUM("adc input d1ain2 mux",
			adc_input_d1ain2_mux_enum);

/* ADC DSP1A enable */
static const struct snd_kcontrol_new dsp1a_en_control =
	SOC_DAPM_SINGLE("Switch", MAP_ADC_CTRL_REG, 0, 3, 0);

/* mux for ADC inmix */
static const char * const adc_in_mux_text[] = {
	"d1ain1",
	"adc in mix",
};

static const struct soc_enum adc_in_mux_enum =
	SOC_ENUM_SINGLE(MAP_ADC_PROCESSING_REG, 10, 2, adc_in_mux_text);

static const struct snd_kcontrol_new adc_in_mux =
	SOC_DAPM_ENUM("ADC in mux", adc_in_mux_enum);

static const struct snd_soc_dapm_widget map_dapm_widgets[] = {
	/* DAC1 input mux */
	SND_SOC_DAPM_MUX("DAC input d1in1", SND_SOC_NOPM,
			0, 0, &dac_input_d1in1_mux),
	SND_SOC_DAPM_MUX("DAC input d1in2", SND_SOC_NOPM,
			0, 0, &dac_input_d1in2_mux),
	SND_SOC_DAPM_MUX("DAC input d1in3", SND_SOC_NOPM,
			0, 0, &dac_input_d1in3_mux),
	SND_SOC_DAPM_MUX("DAC input d1in4", SND_SOC_NOPM,
			0, 0, &dac_input_d1in4_mux),

	/* DAC2 input mux */
	SND_SOC_DAPM_MUX("DAC input d2in1", SND_SOC_NOPM,
			0, 0, &dac_input_d2in1_mux),
	SND_SOC_DAPM_MUX("DAC input d2in2", SND_SOC_NOPM,
			0, 0, &dac_input_d2in2_mux),
	SND_SOC_DAPM_MUX("DAC input d2in3", SND_SOC_NOPM,
			0, 0, &dac_input_d2in3_mux),
	SND_SOC_DAPM_MUX("DAC input d2in4", SND_SOC_NOPM,
			0, 0, &dac_input_d2in4_mux),

	/* DAC is transparent for dapm */
	SND_SOC_DAPM_DAC("DAC1 out", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC2 out", NULL, SND_SOC_NOPM, 0, 0),

	/* DAC output mux */
	SND_SOC_DAPM_MUX("DAC1 output out1", SND_SOC_NOPM,
			0, 0, &dac_output_out1_mux),
	SND_SOC_DAPM_MUX("DAC2 output out2", SND_SOC_NOPM,
			0, 0, &dac_output_out2_mux),

	/* ADC input mux */
	SND_SOC_DAPM_MUX("ADC input d1ain1", SND_SOC_NOPM,
			0, 0, &adc_input_d1ain1_mux),
	SND_SOC_DAPM_MUX("ADC input d1ain2", SND_SOC_NOPM,
			0, 0, &adc_input_d1ain2_mux),

	SND_SOC_DAPM_SWITCH("dsp1a_enable", SND_SOC_NOPM, 0, 0,
				&dsp1a_en_control),

	SND_SOC_DAPM_MIXER("ADC in mix", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ADC in mux", SND_SOC_NOPM, 0, 0, &adc_in_mux),

	/* ADC is transparent for dapm */
	SND_SOC_DAPM_ADC("ADC out", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC input1", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC input2", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC input3", NULL, SND_SOC_NOPM, 0, 0),

	/* ADC output mux */
	SND_SOC_DAPM_MUX("ADC output ain1", SND_SOC_NOPM,
			0, 0, &adc_output_ain1_mux),
	SND_SOC_DAPM_MUX("ADC output ain2", SND_SOC_NOPM,
			0, 0, &adc_output_ain2_mux),
	SND_SOC_DAPM_MUX("ADC output ain3", SND_SOC_NOPM,
			0, 0, &adc_output_ain3_mux),
	SND_SOC_DAPM_MUX("ADC output ain4", SND_SOC_NOPM,
			0, 0, &adc_output_ain4_mux),

	SND_SOC_DAPM_SWITCH("dsp1_enable", SND_SOC_NOPM, 0, 0,
				&dsp1_en_control),

	SND_SOC_DAPM_SWITCH("d1in1_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac1_d1in1_inmix_control),
	SND_SOC_DAPM_SWITCH("d1in2_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac1_d1in2_inmix_control),
	SND_SOC_DAPM_SWITCH("d1in3_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac1_d1in3_inmix_control),
	SND_SOC_DAPM_SWITCH("d1in4_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac1_d1in4_inmix_control),

	SND_SOC_DAPM_MIXER("DAC1 in mix", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_VIRT_MUX("DAC1 in mux", SND_SOC_NOPM, 0, 0, &dac1_in_mux),

	/* txrx */
	SND_SOC_DAPM_MIXER("DAC1 txrx mix", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("DAC1 txrx mux", SND_SOC_NOPM,
			0, 0, &dac1_txrx_mux),

	SND_SOC_DAPM_MIXER("DAC1 out mix", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("DAC1 outmix mux", SND_SOC_NOPM,
			0, 0, &dac1_outmix_mux),

	SND_SOC_DAPM_MIXER("DAC dspmix",
				SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SWITCH("dsp2_enable", SND_SOC_NOPM, 0, 0,
				&dsp2_en_control),

	SND_SOC_DAPM_SWITCH("d2in1_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac2_d1in1_inmix_control),
	SND_SOC_DAPM_SWITCH("d2in2_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac2_d1in2_inmix_control),
	SND_SOC_DAPM_SWITCH("d2in3_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac2_d1in3_inmix_control),
	SND_SOC_DAPM_SWITCH("d2in4_mix_enable", SND_SOC_NOPM, 0, 0,
				&dac2_d1in4_inmix_control),

	SND_SOC_DAPM_MIXER("DAC2 in mix", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_VIRT_MUX("DAC2 in mux", SND_SOC_NOPM, 0, 0, &dac2_in_mux),

	/* txrx */
	SND_SOC_DAPM_MIXER("DAC2 txrx mix", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("DAC2 txrx mux", SND_SOC_NOPM,
			0, 0, &dac2_txrx_mux),

	SND_SOC_DAPM_MIXER("DAC2 out mix", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MUX("DAC2 outmix mux", SND_SOC_NOPM,
			0, 0, &dac2_outmix_mux),

};

static const struct snd_soc_dapm_route map_intercon[] = {
	/* ADC input */
	{"ADC input d1ain1", "MIC1", "ADC input1"},
	{"ADC input d1ain1", "AOUT1", "MM_DL1"},
	{"ADC input d1ain1", "AOUT4", "MM_DL2"},
	{"ADC input d1ain1", "AOUT2", "VC_DL"},
	{"ADC input d1ain1", "AOUT3", "ADC input3"},

	{"ADC input d1ain2", "MIC2", "ADC input2"},
	{"ADC input d1ain2", "AOUT1", "MM_DL1"},
	{"ADC input d1ain2", "AOUT4", "MM_DL2"},
	{"ADC input d1ain2", "AOUT2", "VC_DL"},
	{"ADC input d1ain2", "AOUT3", "ADC input3"},

	{"ADC in mix", NULL, "ADC input d1ain1"},
	{"ADC in mix", NULL, "ADC input d1ain2"},

	{"ADC in mux", "d1ain1", "ADC input d1ain1"},
	{"ADC in mux", "adc in mix", "ADC in mix"},

	{"dsp1a_enable", "Switch", "ADC in mux"},

	{"ADC out", NULL, "dsp1a_enable"},

	/* AIN1 mux */
	{"ADC output ain1", "D1AOUT", "ADC out"},
	{"ADC output ain1", "AOUT1", "MM_DL1"},
	{"ADC output ain1", "AOUT3", "ADC input3"},
	{"ADC output ain1", "AOUT2", "VC_DL"},
	{"ADC output ain1", "AOUT4", "MM_DL2"},
	{"ADC output ain1", "MIC1", "ADC input1"},
	{"ADC output ain1", "MIC2", "ADC input2"},
	{"ADC output ain1", "D1OUT", "DAC1 out"},
	{"ADC output ain1", "D2OUT", "DAC2 out"},

	/* AIN2 mux */
	{"ADC output ain2", "D1AOUT", "ADC out"},
	{"ADC output ain2", "AOUT1", "MM_DL1"},
	{"ADC output ain2", "AOUT3", "ADC input3"},
	{"ADC output ain2", "AOUT2", "VC_DL"},
	{"ADC output ain2", "AOUT4", "MM_DL2"},
	{"ADC output ain2", "MIC1", "ADC input1"},
	{"ADC output ain2", "MIC2", "ADC input2"},
	{"ADC output ain2", "D1OUT", "DAC1 out"},
	{"ADC output ain2", "D2OUT", "DAC2 out"},

	/* AIN3 mux */
	{"ADC output ain3", "D1AOUT", "ADC out"},
	{"ADC output ain3", "AOUT1", "MM_DL1"},
	{"ADC output ain3", "AOUT3", "ADC input3"},
	{"ADC output ain3", "AOUT2", "VC_DL"},
	{"ADC output ain3", "AOUT4", "MM_DL2"},
	{"ADC output ain3", "MIC1", "ADC input1"},
	{"ADC output ain3", "MIC2", "ADC input2"},
	{"ADC output ain3", "D1OUT", "DAC1 out"},
	{"ADC output ain3", "D2OUT", "DAC2 out"},

	/* AIN4 mux */
	{"ADC output ain4", "D1AOUT", "ADC out"},
	{"ADC output ain4", "AOUT1", "MM_DL1"},
	{"ADC output ain4", "AOUT3", "ADC input3"},
	{"ADC output ain4", "AOUT2", "VC_DL"},
	{"ADC output ain4", "AOUT4", "MM_DL2"},
	{"ADC output ain4", "MIC1", "ADC input1"},
	{"ADC output ain4", "MIC2", "ADC input2"},
	{"ADC output ain4", "D1OUT", "DAC1 out"},
	{"ADC output ain4", "D2OUT", "DAC2 out"},

	{"MM_UL1", NULL, "ADC output ain1"},
	{"MM_UL2", NULL, "ADC output ain4"},
	{"VC_UL", NULL, "ADC output ain2"},

	/* DAC1 input */
	{"DAC input d1in1", "AOUT1", "MM_DL1"},
	{"DAC input d1in1", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d1in1", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d1in1", "D2OUT", "DAC2 out"},

	{"DAC input d1in2", "AOUT2", "VC_DL"},
	{"DAC input d1in2", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d1in2", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d1in2", "D2OUT", "DAC2 out"},

	{"DAC input d1in3", "AOUT3", "FM_DL"},
	{"DAC input d1in3", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d1in3", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d1in3", "D2OUT", "DAC2 out"},

	{"DAC input d1in4", "AOUT4", "MM_DL2"},
	{"DAC input d1in4", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d1in4", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d1in4", "D2OUT", "DAC2 out"},

	{"d1in1_mix_enable", "Switch", "DAC input d1in1"},
	{"d1in2_mix_enable", "Switch", "DAC input d1in2"},
	{"d1in3_mix_enable", "Switch", "DAC input d1in3"},
	{"d1in4_mix_enable", "Switch", "DAC input d1in4"},

	{"DAC1 in mix", NULL, "d1in1_mix_enable"},
	{"DAC1 in mix", NULL, "d1in2_mix_enable"},
	{"DAC1 in mix", NULL, "d1in3_mix_enable"},
	{"DAC1 in mix", NULL, "d1in4_mix_enable"},

	{"DAC1 in mux", "d1in1", "DAC input d1in1"},
	{"DAC1 in mux", "in mix", "DAC1 in mix"},

	/* txrx */
	{"DAC1 txrx mix", NULL, "DAC1 in mux"},
	{"DAC1 txrx mix", NULL, "ADC out"},
	{"DAC1 txrx mux", "inmix", "DAC1 in mux"},
	{"DAC1 txrx mux", "txrx", "DAC1 txrx mix"},

	/* DAC1 is transparent for dapm */
	{"dsp1_enable", "Switch", "DAC1 txrx mux"},
	{"DAC1 out", NULL, "dsp1_enable"},

	/* DAC2 input */
	{"DAC input d2in1", "AOUT1", "MM_DL1"},
	{"DAC input d2in1", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d2in1", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d2in1", "D1OUT", "DAC1 out"},

	{"DAC input d2in2", "AOUT2", "VC_DL"},
	{"DAC input d2in2", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d2in2", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d2in2", "D1OUT", "DAC1 out"},

	{"DAC input d2in3", "AOUT3", "FM_DL"},
	{"DAC input d2in3", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d2in3", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d2in3", "D1OUT", "DAC1 out"},

	{"DAC input d2in4", "AOUT4", "MM_DL2"},
	{"DAC input d2in4", "D1AIN1", "ADC input d1ain1"},
	{"DAC input d2in4", "D1AIN2", "ADC input d1ain2"},
	{"DAC input d2in4", "D1OUT", "DAC1 out"},

	{"d2in1_mix_enable", "Switch", "DAC input d2in1"},
	{"d2in2_mix_enable", "Switch", "DAC input d2in2"},
	{"d2in3_mix_enable", "Switch", "DAC input d2in3"},
	{"d2in4_mix_enable", "Switch", "DAC input d2in4"},

	{"DAC2 in mix", NULL, "d2in1_mix_enable"},
	{"DAC2 in mix", NULL, "d2in2_mix_enable"},
	{"DAC2 in mix", NULL, "d2in3_mix_enable"},
	{"DAC2 in mix", NULL, "d2in4_mix_enable"},

	{"DAC2 in mux", "d2in1", "DAC input d2in1"},
	{"DAC2 in mux", "in mix", "DAC2 in mix"},

	/* txrx */
	{"DAC2 txrx mix", NULL, "DAC2 in mux"},
	{"DAC2 txrx mix", NULL, "ADC out"},
	{"DAC2 txrx mux", "inmix", "DAC2 in mux"},
	{"DAC2 txrx mux", "txrx", "DAC2 txrx mix"},

	/* DAC1 is transparent for dapm */
	{"dsp2_enable", "Switch", "DAC2 txrx mux"},
	{"DAC2 out", NULL, "dsp2_enable"},

	/* out mixer */
	{"DAC1 out mix", NULL, "DAC1 out"},
	{"DAC1 outmix mux", "out_L_R", "DAC1 out"},
	{"DAC1 outmix mux", "out_L+R", "DAC1 out mix"},

	{"DAC2 out mix", NULL, "DAC2 out"},
	{"DAC2 outmix mux", "out_L_R", "DAC2 out"},
	{"DAC2 outmix mux", "out_L+R", "DAC2 out mix"},

	/* dsp mixer */
	{"DAC dspmix", NULL, "DAC1 outmix mux"},
	{"DAC dspmix", NULL, "DAC2 outmix mux"},

	/* DAC1 output part */
	{"DAC1 output out1", "D1OUT", "DAC1 outmix mux"},
	{"DAC1 output out1", "D1IN1", "DAC input d1in1"},
	{"DAC1 output out1", "D1IN2", "DAC input d1in2"},
	{"DAC1 output out1", "D1IN3", "DAC input d1in3"},
	{"DAC1 output out1", "D1IN4", "DAC input d1in4"},
	{"DAC1 output out1", "D2OUT", "DAC2 out"},
	{"DAC1 output out1", "DSPMIX", "DAC dspmix"},

	/* DAC2 output part */
	{"DAC2 output out2", "D2OUT", "DAC2 outmix mux"},
	{"DAC2 output out2", "D2IN1", "DAC input d2in1"},
	{"DAC2 output out2", "D2IN2", "DAC input d2in2"},
	{"DAC2 output out2", "D2IN3", "DAC input d2in3"},
	{"DAC2 output out2", "D2IN4", "DAC input d2in4"},
	{"DAC2 output out2", "D1OUT", "DAC1 out"},
	{"DAC2 output out2", "DSPMIX", "DAC dspmix"},
};

static int map_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, map_dapm_widgets,
				ARRAY_SIZE(map_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, map_intercon, ARRAY_SIZE(map_intercon));

	snd_soc_dapm_new_widgets(dapm);
	pr_info("MAP: add widgets and routes\n");

	return 0;
}

/* set map interface source clock */
static int map_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
	int ret = 0;

	if (codec_dai->active > 1)
		return 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
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

	return 0;
}

/*
 * need to consider use clkdiv or add clock source for these interfaces
 */
static int map_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
	unsigned int addr;

	if (codec_dai->active > 1)
		return 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
	switch (codec_dai->id) {
	case 1:
		addr =	MAP_I2S1_BCLK_DIV;
		break;
	case 2:
		addr =	MAP_I2S2_BCLK_DIV;
		break;
	case 3:
		addr =	MAP_I2S3_BCLK_DIV;
		break;
	case 4:
		addr =	MAP_I2S4_BCLK_DIV;
		break;
	default:
		return -EINVAL;
	}
	/* set dai divider <0:27> */
	map_raw_write(map_priv, addr, div);

	return 0;
}

static int map_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;

	if (dai->active > 1)
		return 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(dai);
	map_priv = map_fe_dai_priv->map_priv;
	switch (dai->id) {
	case 1:
		addr =	MAP_I2S1_CTRL_REG;
		break;
	case 2:
		addr =	MAP_I2S2_CTRL_REG;
		break;
	case 3:
		addr =	MAP_I2S3_CTRL_REG;
		break;
	case 4:
		addr =	MAP_I2S4_CTRL_REG;
		break;
	default:
		return -EINVAL;
	}

	/* bit size */
	inf = map_raw_read(map_priv, addr);
	inf &= ~MAP_WLEN_MASK;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		inf |= MAP_WLEN_16_BIT;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		inf |= MAP_WLEN_20_BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		inf |= MAP_WLEN_24_BIT;
		break;
	default:
		return -EINVAL;
	}
	map_raw_write(map_priv, addr, inf);

	/* sample rate */
	map_set_port_freq(map_priv, dai->id, params_rate(params));

	return 0;
}

static int map_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;

	if (codec_dai->active > 1)
		return 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
	switch (codec_dai->id) {
	case 1:
		addr =	MAP_I2S1_CTRL_REG;
		break;
	case 2:
		addr =	MAP_I2S2_CTRL_REG;
		break;
	case 3:
		addr =	MAP_I2S3_CTRL_REG;
		break;
	case 4:
		addr =	MAP_I2S4_CTRL_REG;
		break;
	default:
		return -EINVAL;
	}

	/* set master/slave audio interface */
	inf = map_raw_read(map_priv, addr);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		inf |= MAP_I2S_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		inf &= ~MAP_I2S_MASTER;
		break;
	default:
		return -EINVAL;
	}

	inf &= ~MAP_I2S_MODE_I2S_FORMAT;
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		inf |= MAP_I2S_MODE_I2S_FORMAT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		inf |= MAP_I2S_MODE_RIGHT_JUST;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		inf |= MAP_I2S_MODE_LEFT_JUST;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		inf |= MAP_I2S_MODE_PCM_FORMAT;
		inf &= ~MAP_PCM_MODE_B;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		inf |= MAP_I2S_MODE_PCM_FORMAT;
		inf |= MAP_PCM_MODE_B;
		break;
	default:
		break;
	}
	map_raw_write(map_priv, addr, inf);

	return 0;
}

static int mmp_map_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *codec_dai)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;

	if (codec_dai->active)
		return 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
	/*
	 * Fixme: debug clk on real silicon
	 */
#if 0
	clk_enable(map_priv->audio_clk);
	clk_enable(map_priv->apb_clk);
	clk_set_rate(map_priv->map_apb_clk, 0x0);
#endif

	switch (codec_dai->id) {
	case 1:
		/* reset i2s1 interface(audio) */
		map_reset_port(map_priv, I2S1);
		break;
	case 2:
		/* reset i2s2 interface(audio) */
		map_reset_port(map_priv, I2S2);
		break;
	case 3:
		/* reset i2s3 interface(audio) */
		map_reset_port(map_priv, I2S3);
		break;
	case 4:
		/* reset i2s4 interface (hifi) */
		map_reset_port(map_priv, I2S4);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void mmp_map_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *codec_dai)
{
#if 0
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
#endif
	if (codec_dai->active)
		return;

	/*
	 * Fixme: debug clk on real silicon
	 */
#if 0
	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
	clk_disable(map_priv->apb_clk);
	clk_disable(map_priv->audio_clk);
#endif

	return;
}

static int mmp_map_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *codec_dai)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	struct map_private *map_priv;
	unsigned int inf = 0, addr;
	int ret = 0;

	map_fe_dai_priv = snd_soc_dai_get_drvdata(codec_dai);
	map_priv = map_fe_dai_priv->map_priv;
	switch (codec_dai->id) {
	case 1:
		addr =	MAP_I2S1_CTRL_REG;
		break;
	case 2:
		addr =	MAP_I2S2_CTRL_REG;
		break;
	case 3:
		addr =	MAP_I2S3_CTRL_REG;
		break;
	case 4:
		addr =	MAP_I2S4_CTRL_REG;
		break;
	default:
		return -EINVAL;
	}

	inf = map_raw_read(map_priv, addr);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * Fixme: maybe I2S_GEN_EN/I2S_REC_EN should be set
		 * in hw_param with other bits
		 */
		if (codec_dai->active == 1) {
			inf |= I2S_GEN_EN;
			inf |= I2S_REC_EN;
			map_raw_write(map_priv, addr, inf);

			/* apply the change */
			map_apply_change(map_priv);
		}
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/*
		 * Fixme: maybe I2S_GEN_EN/I2S_REC_EN should be set
		 * in hw_param with other bits
		 */
		if (codec_dai->active == 1) {
			inf &= ~I2S_GEN_EN;
			inf &= ~I2S_REC_EN;
			map_raw_write(map_priv, addr, inf);

			/* apply the change */
			map_apply_change(map_priv);
		}
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}

static int map_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

/*
 * MAP can support sample rate 12000 and 24000, if let ASOC support
 * these two rates, you need to change include/sound/pcm.h and
 * sound/core/pcm_native.c
 */
#define MAP_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 | \
	SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_176400)

#define MAP_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops map_dai_ops = {
	.startup	= mmp_map_startup,
	.shutdown	= mmp_map_shutdown,
	.trigger	= mmp_map_trigger,
	.hw_params	= map_hw_params,
	.digital_mute	= map_mute,
	.set_fmt	= map_set_dai_fmt,
	.set_sysclk	= map_set_dai_sysclk,
	.set_clkdiv	= map_set_dai_clkdiv,
};

struct snd_soc_dai_driver map_dai[] = {
	/* map codec dai */
	{
		.name = "map-i2s1-dai",
		.id = 1,
		.playback = {
			.stream_name  = "MM_DL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates	      = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "MM_UL1",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.ops = &map_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "map-i2s2-dai",
		.id = 4,
		.playback = {
			.stream_name  = "MM_DL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "MM_UL2",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.ops = &map_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "map-i2s3-dai",
		.id = 2,
		.playback = {
			.stream_name  = "VC_DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.capture = {
			.stream_name  = "VC_UL",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.ops = &map_dai_ops,
		.symmetric_rates = 1,
	},
	/* For FM playback */
	{
		.name = "map-i2s4-dai",
		.id = 3,
		.playback = {
			.stream_name  = "FM_DL",
			.channels_min = 1,
			.channels_max = 2,
			.rates        = MAP_RATES,
			.formats      = MAP_FORMATS,
		},
		.ops = &map_dai_ops,
		.symmetric_rates = 1,
	}
};

static int map_probe(struct snd_soc_codec *codec)
{
	struct map_fe_dai_private *map_fe_dai_priv;
	int ret = 0;

	map_fe_dai_priv = snd_soc_codec_get_drvdata(codec);
	map_fe_dai_priv->codec = codec;
	codec->control_data = map_fe_dai_priv->regmap;
	/*
	 * Fixme: bypass codec cache due to cache only support 8/16 bits
	 * Enabling it will result in BUG() (soc-cache:60)
	 */
	codec->cache_bypass = 1;
	snd_soc_codec_set_cache_io(codec, 32, 32, SND_SOC_REGMAP);

	map_add_widgets(codec);
	return ret;
}

static int map_remove(struct snd_soc_codec *codec)
{
	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_map = {
	.probe   = map_probe,
	.remove  = map_remove,
	.reg_cache_size = MAP_CACHE_SIZE,
	.reg_word_size = sizeof(u32),
	.controls = map_snd_controls,
	.num_controls = ARRAY_SIZE(map_snd_controls),
};

static int mmp_map_fe_dai_probe(struct platform_device *pdev)
{
	struct map_private *map_priv = dev_get_drvdata(pdev->dev.parent);
	struct map_fe_dai_private *map_fe_dai_priv;
	int ret = 0;

	map_fe_dai_priv = devm_kzalloc(&pdev->dev,
		sizeof(struct map_fe_dai_private), GFP_KERNEL);
	if (map_fe_dai_priv == NULL)
		return -ENOMEM;

	map_fe_dai_priv->map_priv = map_priv;
	map_fe_dai_priv->regmap = map_priv->regmap;
	platform_set_drvdata(pdev, map_fe_dai_priv);

	ret = snd_soc_register_codec(&pdev->dev,
		&soc_codec_dev_map, map_dai, ARRAY_SIZE(map_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register MAP codec\n");
		return ret;
	}

	return ret;
}

static int mmp_map_fe_dai_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver mmp_map_fe_dai_driver = {
	.probe		= mmp_map_fe_dai_probe,
	.remove		= mmp_map_fe_dai_remove,
	.driver		= {
		.name	= "mmp-map-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init mmp_map_fe_dai_init(void)
{
	return platform_driver_register(&mmp_map_fe_dai_driver);
}

static void __exit mmp_map_fe_dai_exit(void)
{
	platform_driver_unregister(&mmp_map_fe_dai_driver);
}

module_init(mmp_map_fe_dai_init);
module_exit(mmp_map_fe_dai_exit);

MODULE_DESCRIPTION("ASoc Marvell MAP FE driver");
MODULE_AUTHOR("Nenghua Cao <nhcao@marvell.com>");
MODULE_LICENSE("GPL");
