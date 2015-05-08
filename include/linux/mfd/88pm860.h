/*
 * Marvell 88pm860 Interface
 *
 * Copyright (C) 2014 Marvell International Ltd.
 * Zhao Ye <zhaoye@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_PM860_H
#define __LINUX_MFD_PM860_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>

/* 88PM860 Registers */
enum {
	PM860_IRQ_HP1_SHRT,		/* 0 */
	PM860_IRQ_HP2_SHRT,
	PM860_IRQ_MIC_CONFLICT,
	PM860_IRQ_CLIP_FAULT,
	PM860_IRQ_LDO_OFF,
	PM860_IRQ_AUTO_MUTE,
	PM860_IRQ_RAW_PLL_FAULT,
	PM860_IRQ_FINE_PLL_FAULT,	/* 7 */
	PM860_IRQ_SAI_INT,
	PM860_MAX_IRQ,
};

#define PM860_INT_MANAGE		(0x02)
#define PM860_INT_STATUS1		(0x03)
#define PM860_INT_STATUS2		(0x04)

#define PM860_INT_MASK1			(0x05)
#define PM860_HP1_SHRT_MASK		(1<<0)
#define PM860_HP2_SHRT_MASK		(1<<1)
#define PM860_MIC_CONFLICT_MASK		(1<<2)
#define PM860_CLIP_DEFAULT_MASK		(1<<3)
#define PM860_LDO_OFF_MASK		(1<<4)
#define PM860_AUTO_MUTE_MASK		(1<<5)
#define PM860_RAW_PLL_FAULT_MASK	(1<<6)
#define PM860_FINE_PLL_FAULT_MASK	(1<<7)

#define PM860_INT_MASK2			(0x06)
#define PM860_SAI_INT_MASK		(1<<0)

struct pm860_chip {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *companion;
	struct regmap *regmap;
	unsigned char version;
	int id;
	int irq;
	int irq_mode;
	int irq_base;
	struct regmap_irq_chip *regmap_irq_chip;
	struct regmap_irq_chip_data *irq_data;

	unsigned int wu_flag;
	spinlock_t lock;
	struct proc_dir_entry *proc_file;
	int  revision;
};

struct pm860_platform_data {
	struct pm860_headset_pdata *headset;
	unsigned irq_mode;
	unsigned use_gpio; /* if use gpio port for irq, set as 1 */
	unsigned gpio_port; /* gpio port as irq */

	int debounce_time; /* debounce time in msecs */

	/* The I2S-MCLK must start when to initialize 88CE156 codec */
	void (*start_i2s_mclk)(void);
	/* digital mic need PMIC supply power */
	void (*mic_set_power)(int on);
};

#endif /* __LINUX_MFD_PM860_H */
