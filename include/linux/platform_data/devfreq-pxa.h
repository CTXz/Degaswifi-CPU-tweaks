/*
 * devfreq: Generic Dynamic Voltage and Frequency Scaling (DVFS) Framework
 *	    for Non-CPU Devices.
 *
 * Copyright (C) 2012 Marvell
 *	Xiaoguang Chen <chenxg@marvell.com>
 *	Qiming Wu <wuqm@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PXA_DEVFREQ_H__
#define __PXA_DEVFREQ_H__

struct devfreq_platform_data {
	const char *clk_name;
	unsigned int freq_tbl_len;
	unsigned int *freq_table;

#ifdef CONFIG_DDR_DEVFREQ
	struct devfreq_pm_qos_table *qos_list;
#endif
};

#ifdef CONFIG_DDR_DEVFREQ
/*
 * DDR PM constraint value, these lvl can be used by device driver.
 * The relationship of Qos lvl and corresponding ddr frequency
 * should be handled in platform. Frequency should be ordered in
 * ascending.
 * This can be used to avoid driver use the same lvl Qos value
 * instead of different frequency on different platforms.
 */
enum ddr_pm_qos_constraint {
	DDR_CONSTRAINT_LVL_RSV = 0,
	DDR_CONSTRAINT_LVL0,
	DDR_CONSTRAINT_LVL1,
	DDR_CONSTRAINT_LVL2,
	DDR_CONSTRAINT_LVL3,
	DDR_CONSTRAINT_LVL4,
	DDR_CONSTRAINT_LVL_MAX,
};
#endif

#endif
