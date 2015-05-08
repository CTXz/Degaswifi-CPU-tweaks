/* linux/arch/arm/mach-mmp/include/mach/pxa-dvfs.h
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

#ifndef _MACH_MMP_PXA_DVFS_H_
#define _MACH_MMP_PXA_DVFS_H_

struct dvfs_info {
	int (*set_vccmain_volt)(int,  unsigned int);
	int (*get_vccmain_volt)(int);
	int pmic_rampup_step;
};

void setup_pmic_dvfs(struct dvfs_info *dvfs_info);

#endif
