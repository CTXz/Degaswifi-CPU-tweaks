/*
 *
 * Zinitix bt531 touch driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */


#ifndef _LINUX_BT432_TS_H
#define _LINUX_BT432_TS_H

#define TS_DRVIER_VERSION	"1.0.18_1"

#define BT432_TS_DEVICE		"bt432_ts_device"

#define SUPPORTED_TOUCH_KEY_LED	1

#define zinitix_debug_msg(fmt, args...) \
	do { \
		if (m_ts_debug_mode) \
			printk(KERN_INFO "bt432_ts[%-18s:%5d] " fmt, \
					__func__, __LINE__, ## args); \
	} while (0);

#define zinitix_printk(fmt, args...) \
	do { \
		printk(KERN_INFO "bt432_ts[%-18s:%5d] " fmt, \
				__func__, __LINE__, ## args); \
	} while (0);

struct bt432_ts_platform_data {
	u32		gpio_int;
	u32		gpio_ldo_en;
	u16		x_resolution;
	u16		y_resolution;
	u16		page_size;
	u8		orientation;
	u32		tsp_irq;
	int (*tsp_power)(int on);
#if SUPPORTED_TOUCH_KEY_LED
	u32		gpio_keyled;
#endif
	int		tsp_en_gpio;
	u32		tsp_supply_type;
};

extern struct class *sec_class;

#endif /* LINUX_BT531_TS_H */
