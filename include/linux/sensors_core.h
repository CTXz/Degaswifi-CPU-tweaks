/* include/linux/sensors_core.h
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


#ifndef ___SENSORS_CORE_H_INCLUDED
#define ___SENSORS_CORE_H_INCLUDED

#include <linux/device.h>

struct class *sensors_class;
EXPORT_SYMBOL_GPL(sensors_class);

int sensors_register(struct device **dev, void *drvdata,
	struct device_attribute *attributes[],
	char *name);
void sensors_unregister(struct device *dev,
	struct device_attribute *attributes[]);

#endif
