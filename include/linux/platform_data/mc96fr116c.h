/*
 * Copyright (C) 2013 Samsung Electronics
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef _MC96FR116C_PLATFORM_DATA_H_
#define _MC96FR116C_PLATFORM_DATA_H_

struct mc96fr116c_platform_data {
	char char_name;
	int irda_irq_gpio;
	u32 irq_gpio_flags;
	int irda_led_en;
	u32 led_en_flags;
	int irda_wake_en;
	u32 wake_en_flags;
	int irda_rst_gpio;
	u32 rst_gpio_flags;
	int irda_scl_gpio;
	u32 irda_scl_flags;
	int irda_sda_gpio;
	u32 irda_sda_flags;
	void	(*ir_wake_en)(struct device *dev, bool onoff);
	void	(*ir_vdd_onoff)(bool onoff);
	void	(*ir_irled_onoff)(bool onoff);
	int	(*ir_reset)(struct device *dev);
	int	(*ir_read_ready)(struct device *dev);
};

//extern struct class *sec_class;

#endif /* _MC96FR116C_PLATFORM_DATA_H_ */
