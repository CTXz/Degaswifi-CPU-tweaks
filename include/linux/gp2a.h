/* include/linux/gp2a.h
 *
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
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

#ifndef __LINUX_GP2A_H
#define __LINUX_GP2A_H

#include <linux/types.h>

#ifdef __KERNEL__
#define GP2A_OPT "gp2a-opt"
struct gp2a_platform_data {

	int vdd_supply_type;
	int vled_supply_type;

	struct regulator *vdd_regulator;
	struct regulator *vled_regulator;

	int vdd_regulator_volt;
	int vled_regulator_volt;

	int vdd_ldo_en;  /* proximity-sensor-output gpio */
	int vled_ldo_en;

	int (*power)(struct device *, bool); /* power to the chip */
};
#endif /* __KERNEL__ */

#endif
