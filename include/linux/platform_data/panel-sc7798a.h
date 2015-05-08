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


#ifndef _PANEL_SC7798A_H
#define _PANEL_SC7798A_H

#include <video/mmp_disp.h>

struct sc7798a_panel_data {
	struct mmp_mach_panel_info *mi;
	struct mmp_mode *mode;
	u32 panel_id;
	void (*update_backlight)(int intensity);
};

u32 get_panel_id(void);
#endif
