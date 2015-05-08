/* arch/arm/boot/dts/sec-pxa1x88-pinfunc.h
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation,
 * and may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __DTS_SEC_PXA1X88_PINFUNC_H
#define __DTS_SEC_PXA1X88_PINFUNC_H


#define PRI_TDI		0x0B4
#define PRI_TMS		0x0B8
#define PRI_TCK		0x0BC
#define PRI_TDO		0x0C0

#define SLAVE_RESET_OUTN 0x0C8

#define CLK_REQ		0x0CC
#define VCXO_OUT	0x0D8

#define USIM2_UCLK	0x260
#define USIM2_UIO	0x264
#define USIM2_URSTN	0x268

#define PA_MODE		0x270
#define RF_CONT_4	0x274
#define RXTEN		0x288
#define RXTDATA		0x28C

#define SYSCLKEN	0x294

#define USIM_UCLK	0x320
#define USIM_UIO	0x324
#define USIM_URSTN	0x328
#define SLEEP_IND	0x32C

/*
 * Below two macros are used to differentiate power supply options
 * If power supply to LCD is PMIC based use REGULATOR_SUPPLY
 * If power supply to lcd is gpio based use LDO_SUPPLY option
 */
#define LDO_SUPPLY 0
#define REGULATOR_SUPPLY 1

#endif /* __DTS_SEC_PXA1X88_PINFUNC_H */
