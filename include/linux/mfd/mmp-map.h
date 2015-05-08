/*
 * Marvell MAP(Marvell Audio processor) Interface
 *
 * Copyright (C) 2014 Marvell International Ltd.
 * Nenghua Cao <nhcao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_MMP_MAP_H
#define __LINUX_MFD_MMP_MAP_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>

/* AUX reg */
#define DSP_MAP_CONF_REG	0x2c

#define MAP_RESET	(1 << 0)
#define AUD_SSPA1_INPUT_SRC	(1 << 1)
#define AUD_SSPA2_INPUT_SRC	(1 << 2)

#define SSPA1_PAD_OUT_SRC_MASK	(3 << 3)
#define SSPA1_DRIVE_SSPA1_PAD	(0 << 3)
#define TDM_DRIVE_SSPA1_PAD	(1 << 3)
#define DEI2S_DRIVE_SSPA1_PAD	(2 << 3)

#define SSPA2_PAD_OUT_SRC_MASK	(3 << 5)
#define SSPA2_DRIVE_SSPA2_PAD	(0 << 5)
#define DEI2S_DRIVE_SSPA2_PAD	(1 << 5)
#define MAP_I2S4_DRIVE_SSPA2_PAD	(2 << 5)

/* MAP reg */
#define MMP_MAP_REV		0x0

/* Clock and PLL Registers */
#define MAP_REV				0x00
/* I2S and Sample Rate Registers */
#define MAP_LRCLK_RATE_REG		0x04
#define MAP_I2S1_CTRL_REG		0x08
#define MAP_I2S2_CTRL_REG		0x0c
#define MAP_I2S3_CTRL_REG		0x10
#define MAP_I2S4_CTRL_REG		0x14
#define MAP_DEI2S_CTRL_REG		0x18
/* Interrupt and Status Registers */
#define MAP_STATUS_REG_1		0x1c
#define MAP_STATUS_REG_2		0x20
#define MAP_INTERRUPT_CTRL_REG		0x70
/* DAC DSP1 Path Registers */
#define MAP_DSP1_DAC_PROCESSING_REG	0x100
#define MAP_DSP1_DAC_CTRL_REG		0x104
#define MAP_DSP1_DAC_VOLUME		0x108
#define MAP_DSP1_ANC_PARAM_U_REG	0x10c
#define MAP_DSP1_ANC_PARAM_LAMBA_REG	0x110
#define MAP_DSP1_ANC_PARAM_BETA_REG	0x114
#define MAP_DSP1_ANC_PARAM_ERRTH_REG	0x118
#define MAP_DSP1_EQ_BAND1_GAIN		0x11c
#define MAP_DSP1_EQ_BAND1_CENTER_FREQ	0x120
#define MAP_DSP1_EQ_BAND2_GAIN		0x124
#define MAP_DSP1_EQ_BAND2_CENTER_FREQ	0x128
#define MAP_DSP1_EQ_BAND2_BANDWIDTH	0x12c
#define MAP_DSP1_EQ_BAND3_GAIN		0x130
#define MAP_DSP1_EQ_BAND3_CENTER_FREQ	0x134
#define MAP_DSP1_EQ_BAND3_BANDWIDTH	0x138
#define MAP_DSP1_EQ_BAND4_GAIN		0x13c
#define MAP_DSP1_EQ_BAND4_CENTER_FREQ	0x140
#define MAP_DSP1_EQ_BAND4_BANDWIDTH	0x144
#define MAP_DSP1_EQ_BAND5_GAIN		0x148
#define MAP_DSP1_EQ_BAND5_CENTER_FREQ	0x14c
#define MAP_DSP1_EQ_BAND5_BANDWIDTH	0x150
#define MAP_DSP1_EQ_BAND6_GAIN		0x154
#define MAP_DSP1_EQ_BAND6_CENTER_FREQ	0x158
#define MAP_DSP1_EQ_BAND6_BANDWIDTH	0x15c
#define MAP_DSP1_EQ_BAND7_GAIN		0x160
#define MAP_DSP1_EQ_BAND7_CENTER_FREQ	0x164
#define MAP_DSP1_EQ_BAND7_BANDWIDTH	0x168
#define MAP_DSP1_EQ_BAND8_GAIN		0x16c
#define MAP_DSP1_EQ_BAND8_CENTER_FREQ	0x170
#define MAP_DSP1_DAC_DRC_THRESHOLD	0x174
#define MAP_DSP1_DAC_DRC_OFFSET		0x178
#define MAP_DSP1_DAC_DRC_COMPRESSION_RATIO	0x17c
#define MAP_DSP1_DAC_DRC_ENERGY_ALPHA_REG	0x180
#define MAP_DSP1_DAC_DRC_ATTACK_ALPHA_REG	0x184
#define MAP_DSP1_DAC_DRC_DECAY_ALPHA_REG	0x188
#define MAP_DSP1_DAC_OUTPUT_MIX		0x18c
#define MAP_DSP1_TXRX_MIX_COEF_REG	0x190
#define MAP_DSP1_INMIX_COEF_REG		0x194
#define MAP_DSP1_3D_REG1			0x198
#define MAP_DSP1_3D_REG2			0x19c
#define MAP_DSP1_DUMMY_1		0x1a0
#define MAP_DSP1_DUMMY_2		0x1a4
#define MAP_DSP1_DUMMY_3		0x1a8
#define MAP_DSP1_DUMMY_4		0x1ac
#define MAP_DSP1_DUMMY_5		0x1b0
#define MAP_DSP1_DSM_SCALING_REG	0x1b4
/* DAC DSP2 Path Registers */
#define MAP_DSP2_DAC_PROCESSING_REG	0x200
#define MAP_DSP2_DAC_CTRL_REG		0x204
#define MAP_DSP2_DAC_VOLUME		0x208
#define MAP_DSP2_ANC_PARAM_U_REG	0x20c
#define MAP_DSP2_ANC_PARAM_LAMBA_REG	0x210
#define MAP_DSP2_ANC_PARAM_BETA_REG	0x214
#define MAP_DSP2_ANC_PARAM_ERRTH_REG	0x218
#define MAP_DSP2_EQ_BAND1_GAIN		0x21c
#define MAP_DSP2_EQ_BAND1_CENTER_FREQ	0x220
#define MAP_DSP2_EQ_BAND2_GAIN		0x224
#define MAP_DSP2_EQ_BAND2_CENTER_FREQ	0x228
#define MAP_DSP2_EQ_BAND2_BANDWIDTH	0x22c
#define MAP_DSP2_EQ_BAND3_GAIN		0x230
#define MAP_DSP2_EQ_BAND3_CENTER_FREQ	0x234
#define MAP_DSP2_EQ_BAND3_BANDWIDTH	0x238
#define MAP_DSP2_EQ_BAND4_GAIN		0x23c
#define MAP_DSP2_EQ_BAND4_CENTER_FREQ	0x240
#define MAP_DSP2_EQ_BAND4_BANDWIDTH	0x244
#define MAP_DSP2_EQ_BAND5_GAIN		0x248
#define MAP_DSP2_EQ_BAND5_CENTER_FREQ	0x24c
#define MAP_DSP2_EQ_BAND5_BANDWIDTH	0x250
#define MAP_DSP2_EQ_BAND6_GAIN		0x254
#define MAP_DSP2_EQ_BAND6_CENTER_FREQ	0x258
#define MAP_DSP2_EQ_BAND6_BANDWIDTH	0x25c
#define MAP_DSP2_EQ_BAND7_GAIN		0x260
#define MAP_DSP2_EQ_BAND7_CENTER_FREQ	0x264
#define MAP_DSP2_EQ_BAND7_BANDWIDTH	0x268
#define MAP_DSP2_EQ_BAND8_GAIN		0x26c
#define MAP_DSP2_EQ_BAND8_CENTER_FREQ	0x270
#define MAP_DSP2_DAC_DRC_THRESHOLD	0x274
#define MAP_DSP2_DAC_DRC_OFFSET		0x278
#define MAP_DSP2_DAC_DRC_COMPRESSION_RATIO	0x27c
#define MAP_DSP2_DAC_DRC_ENERGY_ALPHA_REG	0x280
#define MAP_DSP2_DAC_DRC_ATTACK_ALPHA_REG	0x284
#define MAP_DSP2_DAC_DRC_DECAY_ALPHA_REG	0x288
#define MAP_DSP2_DAC_OUTPUT_MIX		0x28c
#define MAP_DSP2_TXRX_MIX_COEF_REG	0x290
#define MAP_DSP2_INMIX_COEF_REG		0x294
#define MAP_DSP2_3D_REG1		0x298
#define MAP_DSP2_3D_REG2		0x29c
#define MAP_DSP2_DUMMY_1		0x2a0
#define MAP_DSP2_DUMMY_2		0x2a4
#define MAP_DSP2_DUMMY_3		0x2a8
#define MAP_DSP2_DUMMY_4		0x2ac
#define MAP_DSP2_DUMMY_5		0x2b0
#define MAP_DSP2_DSM_SCALING_REG	0x2b4
/* ADC DSP1A Path Registers */
#define MAP_ADC_PROCESSING_REG		0x300
#define MAP_ADC_CTRL_REG		0x304
#define MAP_ADC_VOLUME			0x308
#define MAP_ADC_ALC_UPPER_THRESHOLD	0x30c
#define MAP_ADC_ALC_LOWER_THRESHOLD	0x310
#define MAP_ADC_ALC_OFFSET		0x314
#define MAP_ADC_ALC_COMPRESSION_RATIO	0x318
#define MAP_ADC_ALC_ENERGY_ALPHA_REG	0x31c
#define MAP_ADC_ALC_ATTACK_ALPHA_REG	0x320
#define MAP_ADC_ALC_DECAY_ALPHA_REG	0x324
#define MAP_ADC_NOISE_GATE_THRESHOD	0x328
#define MAP_ADC_OUTPUT_MIX		0x32c
#define	MAP_AEC_PARAM_U_REG		0x330
#define MAP_AEC_PARAM_LAMBA_REG		0x334
#define MAP_AEC_PARAM_BETA_REG		0x338
#define MAP_AEC_PARAM_ERR_TH_REG	0x33c
#define MAP_WNR_FILTER_COEF		0x340
#define MAP_SSL_PARAM_MU		0x344
#define MAP_INPUT_MIX_REG		0x348
#define MAP_BF_PARAM_REG1		0x34c
#define MAP_BF_PARAM_REG2		0x350
#define MAP_DSP1A_DUMMY_1		0x354
#define MAP_DSP1A_DUMMY_2		0x358
#define MAP_DSP1A_DUMMY_3		0x35c
/*Signal Path Routing for test*/
#define MAP_DIG_TEST_MUX_CTRL_REG	0x400
#define MAP_LOOPBACK_MODES		0x404
#define MAP_DELAY_BUF_CTRL		0x408
#define MAP_DAC_ANA_MISC		0x40c
/*TDM interface reg*/
#define MAP_TDM_CTRL_REG_1		0x3c
#define MAP_TDM_CTRL_REG_2		0x40
#define MAP_TDM_CTRL_REG_3		0x44
#define MAP_TDM_CTRL_REG_4		0x48
#define MAP_TDM_CTRL_REG_5		0x4c
#define MAP_TDM_CTRL_REG_6		0x50
#define MAP_TDM_CTRL_REG_7		0x54
#define MAP_TDM_CTRL_REG_8		0x58
#define MAP_TDM_CTRL_REG_9		0x5c
#define MAP_TDM_CTRL_REG_10		0x60
#define MAP_TDM_CTRL_REG_11		0x64
#define MAP_TDM_CTRL_REG_12		0x68
#define MAP_TDM_CTRL_REG_13		0x6c
/*I2S interface bit clock divider*/
#define MAP_I2S1_BCLK_DIV		0x74
#define MAP_I2S2_BCLK_DIV		0x78
#define MAP_I2S3_BCLK_DIV		0x7c
#define MAP_I2S4_BCLK_DIV		0x80
#define MAP_I2S_OUT_BCLK_DIV		0x8c
/* Top Control and Program ROM */
#define MAP_TOP_CTRL_REG_1		0x24
#define MAP_TOP_CTRL_REG_2		0x28
#define MAP_DATAPATH_FLOW_CTRL_REG_1	0x2c
#define MAP_DATAPATH_FLOW_CTRL_REG_2	0x30
#define MAP_DATAPATH_FLOW_CTRL_REG_3	0x34
#define MAP_ASRC_CTRL_REG		0x38

#define MAP_SAMPLE_RATE_8000        0
#define MAP_SAMPLE_RATE_11025       1
#define MAP_SAMPLE_RATE_12000       2
#define MAP_SAMPLE_RATE_16000       3
#define MAP_SAMPLE_RATE_22050       4
#define MAP_SAMPLE_RATE_24000       5
#define MAP_SAMPLE_RATE_32000       6
#define MAP_SAMPLE_RATE_44100       7
#define MAP_SAMPLE_RATE_48000	8
#define MAP_SAMPLE_RATE_88200       9
#define MAP_SAMPLE_RATE_96000       10
#define MAP_SAMPLE_RATE_176400      11
#define MAP_SAMPLE_RATE_192000	12

#define MAP_CACHE_SIZE	0x8e

/*TOP control register 1 setting*/
#define I2S_OUT_RESET	(1 << 19)
#define I2S4_RESET	(1 << 18)
#define I2S3_RESET	(1 << 17)
#define I2S2_RESET	(1 << 16)
#define I2S1_RESET	(1 << 15)
#define ASRC4_RESET	(1 << 14)
#define ASRC3_RESET	(1 << 13)
#define ASRC2_RESET	(1 << 12)
#define ASRC1_RESET	(1 << 11)
#define DSAP1A_RESET	(1 << 10)
#define DSP2_RESET	(1 << 9)
#define DSP1_RESET	(1 << 8)
#define AUDIO_CLK_DIS	(1 << 4)
#define DATAPATH_RSTB	(1 << 1)
#define AUDIO_RSTB	(1 << 0)

/* I2S setting */
#define MAP_I2S_LRCLK_WIDTH_64BCLK	(0x0 << 16)
#define MAP_I2S_LRCLK_WIDTH_56BCLK	(0x1 << 16)
#define MAP_I2S_LRCLK_WIDTH_48BCLK	(0x2 << 16)
#define MAP_I2S_LRCLK_WIDTH_40BCLK	(0x3 << 16)
#define MAP_I2S_LRCLK_WIDTH_32BCLK	(0x4 << 16)
#define MAP_I2S_LRCLK_WIDTH_MASK	(0x7 << 18)

#define MAP_I2S_MODE_I2S_FORMAT	(0x0 << 14)
#define MAP_I2S_MODE_RIGHT_JUST	(0x1 << 14)
#define MAP_I2S_MODE_LEFT_JUST	(0x2 << 14)
#define MAP_I2S_MODE_PCM_FORMAT	(0x3 << 14)
#define MAP_I2S_MODE_MASK		(0x3 << 14)

#define MAP_WLEN_16_BIT	(0x0 << 12)
#define MAP_WLEN_20_BIT	(0x1 << 12)
#define MAP_WLEN_24_BIT	(0x2 << 12)
#define MAP_WLEN_MASK	(0x3 << 12)

#define MAP_LRCLK_POL	(0x1 << 11)

#define MAP_BCLK_POL	(0x1 << 10)

#define MAP_PCM_MODE_A	(0x0 << 9)
#define MAP_PCM_MODE_B	(0x1 << 9)

#define MAP_I2S_MASTER	(0x1 << 8)

#define I2S_GEN_EN	(0x1 << 1)
#define I2S_REC_EN	(0x1 << 0)

/* DEI2S setting 1 */
#define MAP_DEI2S_LRCLK_WIDTH_64BCLK	(0x0 << 18)
#define MAP_DEI2S_LRCLK_WIDTH_56BCLK	(0x1 << 18)
#define MAP_DEI2S_LRCLK_WIDTH_48BCLK	(0x2 << 18)
#define MAP_DEI2S_LRCLK_WIDTH_40BCLK	(0x3 << 18)
#define MAP_DEI2S_LRCLK_WIDTH_32BCLK	(0x4 << 18)
#define MAP_DEI2S_LRCLK_WIDTH_MASK	(0x7 << 18)

#define MAP_DEI2S_MASTER		(0x1 << 17)

#define MAP_DEI2S_HIZ_DOUT		(0x1 << 16)

#define MAP_DEI2S_MODE_I2S_FORMAT	(0x0 << 14)
#define MAP_DEI2S_MODE_RIGHT_JUST	(0x1 << 14)
#define MAP_DEI2S_MODE_LEFT_JUST	(0x2 << 14)
#define MAP_DEI2S_MODE_PCM_FORMAT	(0x3 << 14)
#define MAP_DEI2S_MODE_MASK		(0x3 << 14)

#define MAP_DEI2S_WLEN_16_BIT	(0x0 << 12)
#define MAP_DEI2S_WLEN_20_BIT	(0x1 << 12)
#define MAP_DEI2S_WLEN_24_BIT	(0x2 << 12)
#define MAP_DEI2S_WLEN_MASK	(0x3 << 12)

#define MAP_DEI2S_LRCLK_POL	(0x1 << 11)

#define MAP_DEI2S_BCLK_POL	(0x1 << 10)

#define MAP_DEI2S_PCM_MODE_A	(0x0 << 9)
#define MAP_DEI2S_PCM_MODE_B	(0x1 << 9)

#define MAP_DEI2S_MSB_LSB	(0x1 << 8)

#define DEI2S_REC_L1_EN		(0X1 << 7)
#define DEI2S_REC_L2_EN		(0X1 << 6)
#define DEI2S_REC_R1_EN		(0X1 << 5)
#define DEI2S_REC_R2_EN		(0X1 << 4)
#define DEI2S_GEN_L1_EN		(0X1 << 3)
#define DEI2S_GEN_L2_EN		(0X1 << 2)
#define DEI2S_GEN_R1_EN		(0X1 << 1)
#define DEI2S_GEN_R2_EN		(0X1 << 0)

/*DAC_ANA_MISC*/
#define APPLY_CHANGES	(1 << 2)

/* Source clocks for MAP interface sample rate generator */
enum MMP_MAP_I2S_clksrc_clk {
	MCLK_32K_PLL,
	MCLK_312M,
	MCLK_26M,
};

struct map_private {
	struct	device *dev;
	void	*regs_aux;
	void	*regs_map;

	struct	regmap *regmap;
	/*clock*/
	struct clk *audio_clk;
	struct clk *apb_clk;
	struct clk *map_apb_clk;

	unsigned int id;
};

enum mmp_map_port {
	I2S1 = 1,
	I2S2,
	I2S3,
	I2S4,
	I2S_OUT,
};

/*tdm platform data*/
struct tdm_platform_data {
	bool use_4_wires;
	unsigned int slot_size;
	unsigned int slot_space;
	unsigned int start_slot;
	unsigned int fsyn_pulse_width;
};

/* set port frame clock frequence*/
void map_set_port_freq(struct map_private *map_priv, enum mmp_map_port port,
						unsigned int rate);
void map_reset_port(struct map_private *map_priv, enum mmp_map_port port);
void map_apply_change(struct map_private *map_priv);
int map_raw_write(struct map_private *map_priv, unsigned int reg,
					unsigned int value);
unsigned int map_raw_read(struct map_private *map_priv,
					unsigned int reg);
#endif /* __LINUX_MFD_MMP_MAP_H */
