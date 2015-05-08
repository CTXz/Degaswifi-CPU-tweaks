/* inclue/linux/platform_data/panel-sc7798a.h
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 * Header file for Samsung Display Panel(LCD) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef _MMP_PANEL_GENERIC_H
#define _MMP_PANEL_GENERIC_H
#include <linux/mutex.h>
#include <video/mmp_disp.h>
#include <linux/platform_data/mmp-panel-mdnie.h>

struct mmp_panel_data {
	struct mmp_mach_panel_info *mi;
	struct mmp_mode *mode;
	u32 panel_id;
	void (*update_backlight)(int intensity);
};

enum {
	/* gpio controlled LDO based supply to LCD */
	EXT_PIN_GPIO = 0,
	/* PMIC Regulator based supply to LCD */
	EXT_PIN_REGULATOR = 1,
};

enum {
	TEMPERATURE_LOW = 0,
	TEMPERATURE_HIGH = 1,
};

struct ext_pin {
	const char *name;
	u32 type;
	union {
		int gpio;
		struct regulator *supply;
	};
};

struct ext_pins {
	struct ext_pin *pins;
	unsigned int nr_pins;
};

struct ext_pin_ctrl {
	const char *name;
	u32 on;
	u32 usec;
};

struct ext_pin_ctrls {
	struct ext_pin_ctrl *ctrls;
	unsigned int nr_ctrls;
};

struct temp_compensation {
	u8 *new_data;
	u8 *old_data;
	u32 data_len;
	u32 trig_type;
	int temperature;
};

struct lcd {
	struct device *dev;
	struct class *lcd_class;
	struct mmp_panel_data  *pdata;
	struct mmp_panel *panel;
	u32 (*set_panel_id)(struct mmp_panel *panel);
	/* Further fields can be added here */
	struct mutex access_ok;
	const char *manufacturer_name;
	const char *panel_model_name;
	unsigned int esd_type;
	int esd_gpio;
	unsigned active:1;
	unsigned esd_en:1;
	unsigned temp_comp_en:1;
	unsigned power:1;
	/* mdnie lite */
	struct mdnie_lite mdnie;
	/* external pins */
	struct ext_pins pins;
	struct ext_pin_ctrls power_on;
	struct ext_pin_ctrls power_off;
	/* pin ctrl */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_enable;
	struct pinctrl_state *pin_disable;
	/* command tables */
	struct mmp_dsi_cmds init_cmds;
	struct mmp_dsi_cmds enable_cmds;
	struct mmp_dsi_cmds disable_cmds;
	struct mmp_dsi_cmds backlight_on_cmds;
	struct mmp_dsi_cmds backlight_set_brightness_cmds;
	/* temperature compensation */
	struct temp_compensation *temp_comp;
	unsigned int nr_temp_comp;
	u8 id_regs[3];
	u8 status_reg;
	u8 status_ok;
};

u32 get_panel_id(void);
int mmp_dsi_panel_verify_reg(struct mmp_panel *panel,
		struct mmp_dsi_cmd_desc cmds[], int count);
#endif /* _MMP_PANEL_GENERIC_H */
