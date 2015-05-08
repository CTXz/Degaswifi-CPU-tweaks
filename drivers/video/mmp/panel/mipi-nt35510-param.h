/* drivers/video/mmp/panel/mipi-nt35510-param.h
 *
 * Copyright (C) 2013 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MIPI_NT35510_PARAM_H
#define __MIPI_NT35510_PARAM_H

#include <video/mipi_display.h>

#define HS_MODE 0
#define LP_MODE 1

#define NT35510_SLEEP_OUT_DELAY 120
#define NT35510_DISP_ON_DELAY 10
#define NT35510_SLEEP_IN_DELAY 120
#define NT35510_DISP_OFF_DELAY 10

/* GMRCTR1  Gamma Correction for Red(positive) */
#define GAMMA_SET_1_D1		0xD1
/* GMGCTR1  Gamma Correction for Green(positive) */
#define GAMMA_SET_1_D2		0xD2
/* GMBCTR1  Gamma Correction for Blue(positive) */
#define GAMMA_SET_1_D3		0xD3

/* GMRCTR2  Gamma Correction for Red(negative) */
#define GAMMA_SET_2_D4		0xD4
/* GMGCTR2  Gamma Correction for Green(negative) */
#define GAMMA_SET_2_D5		0xD5
/* GMBCTR2  Gamma Correction for Blue(negative) */
#define GAMMA_SET_2_D6		0xD6

#define GAMMA_SET_1(x)		{\
					GAMMA_SET_1_##x,\
					0x00, 0x01, 0x00, 0x43, 0x00, \
					0x6B, 0x00, 0x87, 0x00, 0xA3, \
					0x00, 0xCE, 0x00, 0xF1, 0x01, \
					0x27, 0x01, 0x53, 0x01, 0x98, \
					0x01, 0xCE, 0x02, 0x22, 0x02, \
					0x83, 0x02, 0x78, 0x02, 0x9E, \
					0x02, 0xDD, 0x03, 0x00, 0x03, \
					0x2E, 0x03, 0x54, 0x03, 0x7F, \
					0x03, 0x95, 0x03, 0xB3, 0x03, \
					0xC2, 0x03, 0xE1, 0x03, 0xF1, \
					0x03, 0xFE \
}

#define GAMMA_SET_2(x)		{\
					GAMMA_SET_2_##x, \
					0x00, 0x01, 0x00, 0x43, 0x00, \
					0x6B, 0x00, 0x87, 0x00, 0xA3, \
					0x00, 0xCE, 0x00, 0xF1, 0x01, \
					0x27, 0x01, 0x53, 0x01, 0x98, \
					0x01, 0xCE, 0x02, 0x22, 0x02, \
					0x43, 0x02, 0x50, 0x02, 0x9E, \
					0x02, 0xDD, 0x03, 0x00, 0x03, \
					0x2E, 0x03, 0x54, 0x03, 0x7F, \
					0x03, 0x95, 0x03, 0xB3, 0x03, \
					0xC2, 0x03, 0xE1, 0x03, 0xF1, \
					0x03, 0xFE \
}

static char pkt_size_cmd[] = {0x1};

static char pkt_size_cmd_4[] = {0x4};

static char read_id[] = {0x04};
static char read_id1[] = {0xda};
static char read_id2[] = {0xdb};
static char read_id3[] = {0xdc};

static char exit_sleep[] = {0x11};
static char display_on[] = {0x29};
static char sleep_in[] = {0x10};
static char display_off[] = {0x28};

static struct mmp_dsi_cmd_desc nt35510_video_power_off_cmds[] = {
	{MIPI_DSI_DCS_SHORT_WRITE, LP_MODE, NT35510_DISP_OFF_DELAY,
		sizeof(display_off), display_off},
	{MIPI_DSI_DCS_SHORT_WRITE, LP_MODE, NT35510_SLEEP_IN_DELAY,
		sizeof(sleep_in), sleep_in},
};

static struct mmp_dsi_cmd_desc nt35510_video_power_on_cmds[] = {
	{MIPI_DSI_DCS_SHORT_WRITE, LP_MODE, NT35510_SLEEP_OUT_DELAY,
		sizeof(exit_sleep),	exit_sleep},
	{MIPI_DSI_DCS_SHORT_WRITE, LP_MODE, NT35510_DISP_ON_DELAY,
		sizeof(display_on),	display_on},
};

static struct mmp_dsi_cmd_desc read_id_cmds[] = {
	{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, LP_MODE, 0,
		sizeof(pkt_size_cmd_4), pkt_size_cmd_4},
	{MIPI_DSI_DCS_READ, LP_MODE, 0, sizeof(read_id), read_id},
};

static struct mmp_dsi_cmd_desc read_id1_cmds[] = {
	{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, LP_MODE, 0,
		sizeof(pkt_size_cmd), pkt_size_cmd},
	{MIPI_DSI_DCS_READ, LP_MODE, 0, sizeof(read_id1), read_id1},
};

static struct mmp_dsi_cmd_desc read_id2_cmds[] = {
	{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, LP_MODE, 0,
		sizeof(pkt_size_cmd), pkt_size_cmd},
	{MIPI_DSI_DCS_READ, LP_MODE, 0, sizeof(read_id2), read_id2},
};

static struct mmp_dsi_cmd_desc read_id3_cmds[] = {
	{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, LP_MODE, 0,
		sizeof(pkt_size_cmd), pkt_size_cmd},
	{MIPI_DSI_DCS_READ, LP_MODE, 0, sizeof(read_id3), read_id3},
};

/* POWER SET */
static char nt35510_boe_01[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x01
};

static char nt35510_boe_02[] = {
	0xB0, /* SETAVDD */
	0x09, 0x09, 0x09
};

static char nt35510_boe_03[] = {
	0xB6, /* BT1CTR */
	0x34, 0x34, 0x34
};

static char nt35510_boe_04[] = {
	0xB1, /* SETAVEE */
	0x09, 0x09, 0x09,
};

static char nt35510_boe_05[] = {
	0xB7, /* BT2CTR */
	0x24, 0x24, 0x24,
};

static char nt35510_boe_06[] = {
	0xB3, /* SETVGH */
	0x05, 0x05, 0x05
};

static char nt35510_boe_07[] = {
	0xB9, /* BT4CTR */
	0x25, 0x25, 0x25,	/* VGH boosting times/freq */
};

static char nt35510_boe_08[] = {
	0xBF, /* VGHCTR */
	0x01,			/* VGH output ctrl */
};

static char nt35510_boe_09[] = {
	0xB5, /* SETVGL_REG */
	0x0B, 0x0B, 0x0B,
};

static char nt35510_boe_010[] = {
	0xBA, /* BT5CTR */
	0x24, 0x24, 0x24,
};

static char nt35510_boe_010_1[] = {
	0xc2,
	0x01,
};

static char nt35510_boe_011[] = {
	0xBC, /* SETVPG */
	0x00, 0xA3, 0x00,	/* VGMP/VGSN */
};

static char nt35510_boe_012[] = {
	0xBD, /* SETVGN */
	0x00, 0xA3, 0x00,	/* VGMN/VGSN */
};

/* GAMMA1 CONTROL */
static char nt35510_boe_013[] = GAMMA_SET_1(D1);
static char nt35510_boe_014[] = GAMMA_SET_1(D2);
static char nt35510_boe_015[] = GAMMA_SET_1(D3);

/* GAMMA2 CONTROL */
static char nt35510_boe_016[] = GAMMA_SET_2(D4);
static char nt35510_boe_017[] = GAMMA_SET_2(D5);
static char nt35510_boe_018[] = GAMMA_SET_2(D6);

/* Display Param */
static char nt35510_boe_019[] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x00,	/* Page 0 */
};

static char nt35510_boe_020[] = {
	0xB1, /* DOPCTR */
	0xfC, 0x04,
};

static char nt35510_boe_020_1[] = {
	0x36,
	0x02,
};

static char nt35510_boe_022[] = {
	0xB6, /* SDHDTCTR */
	0x0A,		/* Source data hold time */
};

static char nt35510_boe_023[] = {
	0xB7, /* GSEQCTR */
	0x00, 0x00, /* Gate EQ */
};

static char nt35510_boe_024[] = {
	0xB8, /* SDEQCTR */
	0x01, 0x05, 0x05, 0x05, /* Source EQ */
};

static char nt35510_boe_025[] = {
	0xBA, /* BT56CTR */
	0x01,
};

static char nt35510_boe_026[] = {
	0xBD, /* DPFRCTR1 */
	0x01, 0x84, 0x07, 0x32, 0x00, /* Disp Timing */
};

static char nt35510_boe_027[] = {
	0xBE, /* SETVCMOFF */
	0x01, 0x84, 0x07, 0x31, 0x00,
};

static char nt35510_boe_028[] = {
	0xBF, /* VGHCTR */
	0x01, 0x84, 0x07, 0x31, 0x00,
};

static char nt35510_boe_029[] = {
	0x35,				0x00,
};

static char nt35510_boe_029_1[] = {
	0x2a,
	0x00, 0x00, 0x01, 0xdf
};

static char nt35510_boe_029_2[] = {
	0x2b,
	0x00, 0x00, 0x03, 0x1f
};

static char nt35510_boe_029_3[] = {
	0x3a,
	0x77
};

static char nt35510_boe_030[] = {
	0xCC, /* DPTMCTR12 */
	0x03, 0x00, 0x00,	/* aRD(Gateless) Setting */
};

static struct mmp_dsi_cmd_desc nt35510_video_init_cmds[] = {
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_01),
		nt35510_boe_01},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_02),
		nt35510_boe_02},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_03),
		nt35510_boe_03},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_04),
		nt35510_boe_04},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_05),
		nt35510_boe_05},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_06),
		nt35510_boe_06},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_07),
		nt35510_boe_07},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_08),
		nt35510_boe_08},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_09),
		nt35510_boe_09},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_010),
		nt35510_boe_010},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_010_1),
		nt35510_boe_010_1},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_011),
		nt35510_boe_011},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 120, sizeof(nt35510_boe_012),
		nt35510_boe_012},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_013),
		nt35510_boe_013},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_014),
		nt35510_boe_014},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_015),
		nt35510_boe_015},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_016),
		nt35510_boe_016},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_017),
		nt35510_boe_017},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_018),
		nt35510_boe_018},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_019),
		nt35510_boe_019},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_020),
		nt35510_boe_020},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_020_1),
		nt35510_boe_020_1},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_022),
		nt35510_boe_022},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_023),
		nt35510_boe_023},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_024),
		nt35510_boe_024},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_025),
		nt35510_boe_025},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_026),
		nt35510_boe_026},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_027),
		nt35510_boe_027},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_028),
		nt35510_boe_028},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_029),
		nt35510_boe_029},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_029_1),
		nt35510_boe_029_1},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_029_2),
		nt35510_boe_029_2},
	{MIPI_DSI_DCS_SHORT_WRITE_PARAM, LP_MODE, 0, sizeof(nt35510_boe_029_3),
		nt35510_boe_029_3},
	{MIPI_DSI_DCS_LONG_WRITE, LP_MODE, 0, sizeof(nt35510_boe_030),
		nt35510_boe_030},
};

#define MAX_BRIGHTNESS	(100)
#define MAX_BRIGHTNESS_IN_BLU	32
#define DEFAULT_BRIGHTNESS 50
#define TPS61158_DATA_BITS 8
#define TPS61158_ADDRESS_BITS 8
#define TPS61158_TIME_T_EOS 300
#define TPS61158_TIME_ES_DET_DELAY 150
#define TPS61158_TIME_ES_DET_TIME 500
#define TPS61158_TIME_T_START 10
#define TPS61158_TIME_DATA_DELAY_SHORT 180
#define TPS61158_TIME_DATA_DELAY_LONG 400

struct tps61158_data {
	int bl_value;
	unsigned int bl_level;
	unsigned char bl_send_address; /* Always setted to 0x1A */
	unsigned char bl_send_data;
};

static struct tps61158_data tps61158_table[] = {
	/* Brightness tuning available by adjusting "bl_value" of table */
	/*D0 D1 D2 D3  D4 A0 A1 RFA */
	{   0,  0, 0x1A, 0x00}, /* 0  0  0  0   0  0  0  0 */
	 /* 1  0  0  0   0  0  0  0 Requested setted as MIN */
	{   1,  1, 0x1A, 0x80},
	{  29,  2, 0x1A, 0x40}, /* 0  1  0  0   0  0  0  0 */
	{  35,  3, 0x1A, 0xC0}, /* 1  1  0  0   0  0  0  0 */
	{  40,  4, 0x1A, 0x20}, /* 0  0  1  0   0  0  0  0 */
	{  45,  5, 0x1A, 0xA0}, /* 1  0  1  0   0  0  0  0 */
	{  50,  6, 0x1A, 0x60}, /* 0  1  1  0   0  0  0  0 */
	{  55,  7, 0x1A, 0xE0}, /* 1  1  1  0   0  0  0  0 */
	{  60,  8, 0x1A, 0x10}, /* 0  0  0  1   0  0  0  0 */
	{  65,  9, 0x1A, 0x90}, /* 1  0  0  1   0  0  0  0 */
	{  70, 10, 0x1A, 0x50}, /* 0  1  0  1   0  0  0  0 */
	{  75, 11, 0x1A, 0xD0}, /* 1  1  0  1   0  0  0  0 */
	{  80, 12, 0x1A, 0x30}, /* 0  0  1  1   0  0  0  0 */
	{  85, 13, 0x1A, 0xB0}, /* 1  0  1  1   0  0  0  0 */
	{  90, 14, 0x1A, 0x70}, /* 0  1  1  1   0  0  0  0 */
	{  95, 15, 0x1A, 0xF0}, /* 1  1  1  1   0  0  0  0 */
	{ 100, 16, 0x1A, 0x08}, /* 0  0  0  0   1  0  0  0 */
	{ 110, 17, 0x1A, 0x88}, /* 1  0  0  0   1  0  0  0 */
	/* 0  1  0  0   1  0  0  0 Requested setted as default */
	{ 120, 18, 0x1A, 0x48},
	{ 130, 19, 0x1A, 0xC8}, /* 1  1  0  0   1  0  0  0 */
	{ 140, 20, 0x1A, 0x28}, /* 0  0  1  0   1  0  0  0 */
	{ 155, 21, 0x1A, 0xA8}, /* 1  0  1  0   1  0  0  0 */
	{ 170, 22, 0x1A, 0x68}, /* 0  1  1  0   1  0  0  0 */
	{ 180, 23, 0x1A, 0xE8}, /* 1  1  1  0   1  0  0  0 */
	{ 190, 24, 0x1A, 0x18}, /* 0  0  0  1   1  0  0  0 */
	{ 200, 25, 0x1A, 0x98}, /* 1  0  0  1   1  0  0  0 */
	{ 215, 26, 0x1A, 0x58}, /* 0  1  0  1   1  0  0  0 */
	{ 230, 27, 0x1A, 0xD8}, /* 1  1  0  1   1  0  0  0 */
	/* 0  0  1  1   1  0  0  0 Requested setted as MAX */
	{ 255, 28, 0x1A, 0x38},
	{ 400, 29, 0x1A, 0xB8}, /* 1  0  1  1   1  0  0  0  Disabled*/
	{ 500, 30, 0x1A, 0x78}, /* 0  1  1  1   1  0  0  0  Disabled*/
	{ 600, 31, 0x1A, 0xF8}, /* 1  1  1  1   1  0  0  0  Disabled*/
};

#endif /* __MIPI_NT35510_PARAM_H */
