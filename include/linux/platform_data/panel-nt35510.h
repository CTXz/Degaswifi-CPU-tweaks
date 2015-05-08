/* inclue/linux/platform_data/panel-nt35510.h
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


#ifndef _PANEL_NT35510_H
#define _PANEL_NT35510_H

#include <video/mmp_disp.h>

struct nt35510_panel_data {
	struct mmp_mach_panel_info *mi;
	struct mmp_mode *mode;
	u32 panel_id;
	void (*update_backlight)(int intensity);
};

u32 get_panel_id(void);
#endif
