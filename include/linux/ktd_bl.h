/* inclue/linux/ktd_bl.h
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 * Header file for LCD backlight driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __KTD_BL_H__
#define __KTD_BL_H__

enum {
	BRT_VALUE_OFF = 0,
	BRT_VALUE_DIM,
	BRT_VALUE_MIN,
	BRT_VALUE_DEF,
	BRT_VALUE_MAX,
	MAX_BRT_VALUE_IDX,
};

struct brt_value {
	int brightness;	/* brightness level from user */
	int tune_level;	/* tuning value be sent */
};

struct ktd_bl_info {
	char *name;
	bool enable;
	int current_brightness;
	int prev_tune_level;
	int gpio_bl_ctrl;
	int gpio_bl_pwm_en;
	struct brt_value range[MAX_BRT_VALUE_IDX];
	struct brt_value *brt_table;
	unsigned int sz_table;
};

int ktd_backlight_disable(void);
int ktd_backlight_enable(void);
void backlight_set_brightness(int);
#endif	/* __KTD_BL_H__ */
