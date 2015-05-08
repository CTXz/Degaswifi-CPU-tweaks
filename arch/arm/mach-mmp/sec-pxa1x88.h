/* arch/arm/mach-mmp/sec-pxa1x88.h
 * Copyright (C) 2013 Samsung Electronics Co, Ltd.
 *
 * Author: Shankar Bandal <shankar.b@samsung.com>
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

#ifndef __SEC_PXA1X88_H
#define __SEC_PXA1X88_H


/** @category pxa regdump */
void  __init pxa_init_pmua_regdump(void);
void __init pxa_init_gic_regdump(void);
void __init pxa_init_pmua_regdump_1x88(void);

#endif
