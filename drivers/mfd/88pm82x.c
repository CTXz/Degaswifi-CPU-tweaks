/*
 * I2C driver for Marvell 88PM80x
 *
 * Copyright (C) 2012 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 * Qiao Zhou <zhouqiao@marvell.com>
 * Hongyan Song <hysong@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm822.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>

/* 88pm80x chips have same definition for chip id register. */
#define PM80X_CHIP_ID			(0x00)
#define PM82X_CHIP_ID_NUM(x)		(((x) >> 5) & 0x7)
#define PM80X_CHIP_ID_REVISION(x)	((x) & 0x1F)

struct pm82x_chip_mapping {
	unsigned int	id;
	int		type;
};

static struct pm82x_chip_mapping chip_mapping[] = {
	/* 88PM822 chip id number */
	{0x4,	CHIP_PM822},
	/* 88PM805 chip id number */
	{0x0,	CHIP_PM806},
};

/*
 * workaround: some registers needed by pm805 are defined in pm800, so
 * need to use this global variable to maintain the relation between
 * pm800 and pm805. would remove it after HW chip fixes the issue.
 */
static struct pm822_chip *g_pm82x_chip;

const struct regmap_config pm82x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
EXPORT_SYMBOL_GPL(pm82x_regmap_config);


int pm82x_init(struct i2c_client *client)
{
	struct pm822_chip *chip;
	struct regmap *map;
	unsigned int val;
	int i, ret = 0;

	chip =
	    devm_kzalloc(&client->dev, sizeof(struct pm822_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	map = devm_regmap_init_i2c(client, &pm82x_regmap_config);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	chip->client = client;
	chip->regmap = map;

	chip->irq = client->irq;

	chip->dev = &client->dev;
	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	ret = regmap_read(chip->regmap, PM822_CHIP_ID, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(chip_mapping); i++) {
		if (chip_mapping[i].id == PM82X_CHIP_ID_NUM(val)) {
			chip->type = chip_mapping[i].type;
			break;
		}
	}

	if (i == ARRAY_SIZE(chip_mapping)) {
		dev_err(chip->dev,
			"Failed to detect Marvell 88PM822:ChipID[0x%x]\n", val);
		return -EINVAL;
	}

	device_init_wakeup(&client->dev, 1);

	/*
	 * workaround: set g_pm80x_chip to the first probed chip. if the
	 * second chip is probed, just point to the companion to each
	 * other so that pm805 can access those specific register. would
	 * remove it after HW chip fixes the issue.
	 */
	if (!g_pm82x_chip)
		g_pm82x_chip = chip;
	else {
		chip->companion = g_pm82x_chip->client;
		g_pm82x_chip->companion = chip->client;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pm82x_init);

int pm82x_deinit(void)
{
	/*
	 * workaround: clear the dependency between pm800 and pm805.
	 * would remove it after HW chip fixes the issue.
	 */
	if (g_pm82x_chip->companion)
		g_pm82x_chip->companion = NULL;
	else
		g_pm82x_chip = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(pm82x_deinit);
#if 0

#ifdef CONFIG_PM
static int pm822_suspend(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm822_chip *chip = i2c_get_clientdata(client);
	int i, tmp = chip->wu_flag;

	if (chip && tmp &&
	    device_may_wakeup(chip->dev)) {
		enable_irq_wake(chip->irq);

		for (i = 0; i < 32; i++) {
			if (tmp & (1 << i))
				enable_irq_wake(chip->irq_base + i);
		}
	}

	return 0;
}

static int pm822_resume(struct device *dev)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm822_chip *chip = i2c_get_clientdata(client);
	int i, tmp = chip->wu_flag;

	if (chip && tmp &&
	    device_may_wakeup(chip->dev)) {
		disable_irq_wake(chip->irq);

		for (i = 0; i < 32; i++) {
			if (tmp & (1 << i))
				disable_irq_wake(chip->irq_base + i);
		}
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(pm822_pm_ops, pm822_suspend, pm822_resume);

#endif
#endif

MODULE_DESCRIPTION("I2C Driver for Marvell 88PM82x");
MODULE_AUTHOR("Hongyan Song <hysong@marvell.com>");
MODULE_LICENSE("GPL");
