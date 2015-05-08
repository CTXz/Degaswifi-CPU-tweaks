/*
 * Marvell 88CE170 Interface
 *
 * Copyright (C) 2013 Marvell International Ltd.
 * Zhaoy Ye <zhaoy@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MFD_CE170_H
#define __LINUX_MFD_CE170_H

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/atomic.h>
#include <linux/regulator/machine.h>
#include <linux/proc_fs.h>

/* 88CE170 Registers */
#define	CE170_IRQ_HSR_SC_GND		0
#define	CE170_IRQ_HSL_SC_GND		1
#define	CE170_IRQ_EPP_SC_PG		2
#define	CE170_IRQ_EPP_SC_EPN		3
#define	CE170_IRQ_SPKRPN_SC_PG		4
#define	CE170_IRQ_SPKRP_SC_SPKRN	5
#define	CE170_IRQ_SPKLPN_SC_PG		6
#define	CE170_IRQ_SPKLP_SC_SPKLN	7
#define	CE170_IRQ_HSINSERT		8
#define	CE170_IRQ_EXTMIC		9
#define	CE170_IRQ_EXTMIC_SC_GND		10
#define	CE170_IRQ_TE			11
#define	CE170_IRQ_ONBOARDMIC_DET	12
#define	CE170_IRQ_ONBOARDMIC_SC_GND	13
#define	CE170_IRQ_HSREMOVE		14
#define	CE170_MAX_IRQ			15

#define CE170_INT_CONTROL_REG1		(0x18)
#define CE170_INT_CONTROL_REG2		(0x19)
#define CE170_MIC2_CTRL_REG		(0xb3)

#define CE170_CLR_IRQ_STATUS_REG	(1 << 0)
#define CE170_IRQ_OUT_POLARITY		(1 << 1)
#define CE170_IRQ_OUT_CONFIG		(1 << 2)

#define CE170_INT_STATUS1		(0x1a)
#define CE170_HS_MIC_DET_REG		(0xc8)

#define CE170_INT1_HSR_SC_GND		(1 << 0)
#define CE170_INT1_HSL_SC_GND		(1 << 1)
#define CE170_INT1_EPP_SC_PG		(1 << 2)
#define CE170_INT1_EPP_SC_EPN		(1 << 3)
#define CE170_INT1_SPKRPN_SC_PG		(1 << 4)
#define CE170_INT1_SPKRP_SC_SPKRN	(1 << 5)
#define CE170_INT1_SPKLPN_SC_PG		(1 << 6)
#define CE170_INT1_SPKLP_SC_SPKLN	(1 << 7)

#define CE170_INT_STATUS2		(0x1b)

#define CE170_INT2_HSINSERT		(1 << 0)
#define CE170_INT2_EXTMIC		(1 << 1)
#define CE170_INT2_EXTMIC_SC_GND	(1 << 2)
#define CE170_INT2_TE			(1 << 3)
#define CE170_INT2_ONBOARDMIC_DET	(1 << 4)
#define CE170_INT2_ONBOARDMIC_SC_GND	(1 << 5)
#define CE170_INT2_HSREMOVE		(1 << 6)

struct ce170_chip {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *companion;
	struct regmap *regmap;
	unsigned char version;
	int id;
	int irq;
	int irq_mode;
	int irq_base;
	struct regmap_irq_chip_data *irq_data;

	unsigned long wu_flag;
	spinlock_t lock;
	struct proc_dir_entry *proc_file;
	int  revision;
};

struct ce170_platform_data {
	struct ce170_headset_pdata *headset;
	unsigned irq_mode;
	unsigned use_gpio; /* if use gpio port for irq, set as 1 */
	unsigned gpio_port; /* gpio port as irq */

	/* if need to detect headset insert/remove, set as 1*/
	unsigned use_hs_det;

	/* if need to detect micphone insert/remove, set as 1*/
	unsigned use_mic_det;

	int debounce_time; /* debounce time in msecs */

	/* The I2S-MCLK must start when to initialize 88CE156 codec */
	void (*start_i2s_mclk)(void);
	/* digital mic need PMIC supply power */
	void (*mic_set_power)(int on);
};

#ifdef CONFIG_PM
static inline int ce170_dev_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ce170_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq = platform_get_irq(pdev, 0);

	if (device_may_wakeup(dev))
		set_bit(irq, &chip->wu_flag);

	return 0;
}

static inline int ce170_dev_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ce170_chip *chip = dev_get_drvdata(pdev->dev.parent);
	int irq = platform_get_irq(pdev, 0);

	if (device_may_wakeup(dev))
		clear_bit(irq, &chip->wu_flag);

	return 0;
}
#endif

extern int ce170_irq_init(struct ce170_chip *ce170);
extern void ce170_irq_exit(struct ce170_chip *ce170);
#endif /* __LINUX_MFD_CE170_H */
