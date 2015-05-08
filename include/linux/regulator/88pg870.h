/*
 * 88pg870.h - Marvell DC/DC Regulator 88PG870 Driver
 *
 * Copyright (C) 2013 Marvell Technology Ltd.
 * Yipeng Yao <ypyao@marvell.com>
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
 */

#ifndef __88PG870_H__

#define PG870_MIN_UV		600000
#define PG870_MID_UV		1600000
#define PG870_MAX_UV		3950000
#define SEL_STEPS		((PG870_MID_UV - PG870_MIN_UV) / 12500)

/* Status register */
#define PG870_STATUS_REG	0x01

/* System Control Register Setting.
 * ----------------------------------------
 *   Bit |  Description
 * ------|---------------------------------
 *   [0] | VSEL0 pull down resistor enable
 * ------|---------------------------------
 *   [1] | VSEL1 pull down resistor enable
 * ------|---------------------------------
 *   [2] | ENABLE pull down resistor enable
 * ------|---------------------------------
 *   [3] | Always set to 1
 * ------|---------------------------------
 *   [4] | Always set to 1
 * ------|---------------------------------
 *   [5] | PWM mode enable
 * ------|---------------------------------
 *   [6] | Relaxed mode enable
 * ------|---------------------------------
 *   [7] | Always set to 0
 * ------|---------------------------------
 */
/* Sysctrl register */
#define PG870_SYSCTRL_REG	0x02
#define PD_VSEL0	(1 << 0)
#define PD_VSEL1	(1 << 1)
#define PD_ENABLE	(1 << 2)
#define PD_RESV1	(1 << 3)/*should always set to be 1*/
#define PD_RESV2	(1 << 4)/*should always set to be 1*/
#define DVC_PWM	(1 << 5)/*0-diasbled 1-force PWM*/
#define VOUT_CLAMP	(1 << 6)/*0-tight 1-relaxed*/
#define PD_RESV3	(1 << 7)/*should always set to be 0*/

/* Sleep Mode register */
#define PG870_SLEEP_MODE_REG	0x04
/* Sleep Mode Voltage register */
#define PG870_SLEEP_VOL_REG	0x0D
/* Control register */
#define PG870_CONTROL_REG	0x0E
/* IC Type */
#define PG870_CHIP_ID_REG	0x33

#define CTL_RAMP_MASK		0x7
#define SM_PD			(1 << 4)
#define PG870_DEF_SYSCTRL	0x1F
#define PG870_NVOLTAGES		128	/* Numbers of voltages */


/* Ramp rate limiting during DVC change.
 * -----------------------
 *   Bin |Ramp Rate(mV/uS)
 * ------|----------------
 *   000 |     0.25
 * ------|----------------
 *   001 |     0.50
 * ------|----------------
 *   010 |     1.00
 * ------|----------------
 *   011 |     2.00
 * ------|----------------
 *   100 |     3.50
 * ------|----------------
 *   101 |     7.00
 * ------|----------------
 *   110 |    14.00
 * ------|----------------
 *   111 |  Reserved
 * -----------------------
 */
enum {
	PG870_RAMP_RATE_250UV = 0,
	PG870_RAMP_RATE_500UV,
	PG870_RAMP_RATE_1000UV,
	PG870_RAMP_RATE_2000UV,
	PG870_RAMP_RATE_3500UV,
	PG870_RAMP_RATE_7000UV,
	PG870_RAMP_RATE_14000UV,
};

/* Sleep Mode select-PG870_SLEEP_MODE_REG- BK_SLP[1:0] */
enum {
	/* turn off buck converter in sleep mode */
	PG870_SM_TURN_OFF = 0,
	/* use sleep voltage in sleep mode */
	PG870_SM_RUN_SLEEP,
	/* use sleep voltage and enter low power in sleep mode */
	PG870_SM_LPM_SLEEP,
	/* use active voltage in sleep mode */
	PG870_SM_RUN_ACTIVE,
	/* sleep mode disable */
	PG870_SM_DISABLE,
};

struct pg870_platform_data {
	struct regulator_init_data *regulator;
	unsigned int ramp_rate;
	/* Sleep System Control Setting */
	unsigned int sysctrl;
	/* Sleep Mode */
	unsigned int sleep_mode;
	/* Sleep Voltage */
	unsigned int sleep_vol;
	/* Voltage select table idx */
	unsigned int vsel_idx;

	unsigned int *dvc_init_map;
};

extern int pg870_dvc_setvolt(unsigned int lvl, unsigned int uv);
extern int pg870_dvc_getvolt(unsigned int lvl);

#endif /* __88PG870_H__ */

