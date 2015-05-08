/*
 * arch/arm/mach-mmp/pm-eden.c
 *
 * Author:	Timothy Xia <wlxia@marvell.com>
 * Copyright:	(C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <mach/cputype.h>
#include <mach/addr-map.h>
#include <mach/irqs.h>
#include <mach/pm.h>
#include <linux/pm_qos.h>
#include <mach/regs-icu.h>

static int __init eden_suspend_init(void)
{

	return 0;
}
late_initcall(eden_suspend_init);
