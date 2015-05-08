/*
 * I2C driver for Marvell 88PM860-audio.c
 *
 * Copyright (C) 2014 Marvell International Ltd.
 *
 * Zhao Ye <zhaoy@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm860.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>

#define	PM860_AUDIO_REG_NUM		0xc5
static int reg_pm860 = 0xffff;

struct regmap_config pm860_base_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static struct resource pm860_codec_resources[] = {
	{
	 /* Audio short HP1 */
	 .name = "audio-short1",
	 .start = PM860_IRQ_HP1_SHRT,
	 .end = PM860_IRQ_HP1_SHRT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 /* Audio short HP2 */
	 .name = "audio-short2",
	 .start = PM860_IRQ_HP2_SHRT,
	 .end = PM860_IRQ_HP2_SHRT,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct regmap_irq pm860_irqs[] = {
	/* INT0 */
	[PM860_IRQ_HP1_SHRT] = {
		.mask = PM860_HP1_SHRT_MASK,
	},
	[PM860_IRQ_HP2_SHRT] = {
		.mask = PM860_HP2_SHRT_MASK,
	},
	[PM860_IRQ_MIC_CONFLICT] = {
		.mask = PM860_MIC_CONFLICT_MASK,
	},
	[PM860_IRQ_CLIP_FAULT] = {
		.mask = PM860_CLIP_DEFAULT_MASK,
	},
	[PM860_IRQ_LDO_OFF] = {
		.mask = PM860_LDO_OFF_MASK,
	},
	[PM860_IRQ_AUTO_MUTE] = {
		.mask = PM860_AUTO_MUTE_MASK,
	},
	[PM860_IRQ_RAW_PLL_FAULT] = {
		.mask = PM860_RAW_PLL_FAULT_MASK,
	},
	[PM860_IRQ_FINE_PLL_FAULT] = {
		.mask = PM860_FINE_PLL_FAULT_MASK,
	},
	/* INT1 */
	[PM860_IRQ_SAI_INT] = {
		.reg_offset = 1,
		.mask = PM860_SAI_INT_MASK,
	},
};

static struct regmap_irq_chip pm860_irq_chip = {
	.name = "88pm860",
	.irqs = pm860_irqs,
	.num_irqs = ARRAY_SIZE(pm860_irqs),

	.num_regs = 2,
	.status_base = PM860_INT_STATUS1,
	.mask_base = PM860_INT_MASK1,
	.ack_base = PM860_INT_STATUS1,
	.mask_invert = 1,
};

static struct mfd_cell pm860_devs[] = {
	{
		.name = "88pm860-codec",
		.num_resources = ARRAY_SIZE(pm860_codec_resources),
		.resources = pm860_codec_resources,
	},
};

static ssize_t pm860_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int reg_val = 0;
	int len = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm860_chip *chip = i2c_get_clientdata(client);

	int i;

	if (reg_pm860 == 0xffff) {
		pr_info("pm860: register dump:\n");
		for (i = 0; i < PM860_AUDIO_REG_NUM; i++) {
			regmap_read(chip->regmap, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}
	} else {
		regmap_read(chip->regmap, reg_pm860, &reg_val);
		len = sprintf(buf, "reg_pm860=0x%x, val=0x%x\n",
			      reg_pm860, reg_val);
	}
	return len;
}

static ssize_t pm860_reg_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	u8 reg_val;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct pm860_chip *chip = i2c_get_clientdata(client);
	int i = 0;
	int ret;

	char messages[20];
	memset(messages, '\0', 20);
	strncpy(messages, buf, count);

	if ('+' == messages[0]) {
		/* enable to get all the reg value */
		reg_pm860 = 0xffff;
		pr_info("read all reg enabled!\n");
	} else {
		if (messages[1] != 'x') {
			pr_err("Right format: 0x[addr]\n");
			return -EINVAL;
		}

		if (strlen(messages) > 5) {
			while (messages[i] != ' ')
				i++;
			messages[i] = '\0';
			if (kstrtouint(messages, 16, &reg_pm860) < 0)
				return -EINVAL;
			i++;
			if (kstrtou8(messages + i, 16, &reg_val) < 0)
				return -EINVAL;
			ret = regmap_write(chip->regmap, reg_pm860,
					   reg_val & 0xff);
			if (ret < 0) {
				pr_err("write reg error!\n");
				return -EINVAL;
			}
		} else {
			if (kstrtouint(messages, 16, &reg_pm860) < 0)
				return -EINVAL;
		}
	}

	return count;
}

static DEVICE_ATTR(pm860_reg, 0644, pm860_reg_show, pm860_reg_set);

/*
 * Instantiate the generic non-control parts of the device.
 */
static int pm860_dev_init(struct pm860_chip *pm860, int irq)
{
	int ret;
	unsigned int val;

	dev_set_drvdata(pm860->dev, pm860);

	ret = regmap_read(pm860->regmap, 0x0, &val);
	if (ret < 0) {
		dev_err(pm860->dev, "Failed to read ID register\n");
		goto err;
	}
	pm860->revision = val;


	dev_info(pm860->dev, "pm860 revision 0x%x\n", pm860->revision);

	pm860->regmap_irq_chip = &pm860_irq_chip;

	ret = mfd_add_devices(pm860->dev, -1,
			      pm860_devs, ARRAY_SIZE(pm860_devs),
			      NULL,
			      0,
			      NULL);
	if (ret != 0) {
		dev_err(pm860->dev, "Failed to add children: %d\n", ret);
		goto err;
	}

	return 0;

err:
	return ret;
}

static void pm860_dev_exit(struct pm860_chip *pm860)
{
	mfd_remove_devices(pm860->dev);
}

static const struct of_device_id pm860_of_match[] = {
	{ .compatible = "marvell,pm860", },
	{ }
};
MODULE_DEVICE_TABLE(of, pm860_of_match);

static int pm860_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct pm860_chip *pm860;
	int ret;

	pm860 = devm_kzalloc(&i2c->dev, sizeof(struct pm860_chip), GFP_KERNEL);
	if (pm860 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, pm860);
	pm860->dev = &i2c->dev;
	pm860->irq = i2c->irq;

	pm860->regmap = devm_regmap_init_i2c(i2c, &pm860_base_regmap_config);
	if (IS_ERR(pm860->regmap)) {
		ret = PTR_ERR(pm860->regmap);
		dev_err(pm860->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/* add pm860_reg sysfs entries */
	ret = device_create_file(pm860->dev, &dev_attr_pm860_reg);
	if (ret < 0) {
		dev_err(pm860->dev,
			"%s: failed to add pm860_reg sysfs files: %d\n",
			__func__, ret);
		return ret;
	}

	return pm860_dev_init(pm860, i2c->irq);
}

static int pm860_i2c_remove(struct i2c_client *i2c)
{
	struct pm860_chip *pm860 = i2c_get_clientdata(i2c);

	device_remove_file(pm860->dev, &dev_attr_pm860_reg);
	regmap_del_irq_chip(pm860->irq, pm860->irq_data);
	pm860_dev_exit(pm860);

	return 0;
}

static const struct i2c_device_id pm860_i2c_id[] = {
	{ "88pm860", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pm860_i2c_id);

static int pm860_suspend(struct device *dev)
{
	return 0;
}

static int pm860_resume(struct device *dev)
{
	return 0;
}

static UNIVERSAL_DEV_PM_OPS(pm860_pm_ops, pm860_suspend, pm860_resume,
			    NULL);
static const struct of_device_id pm860_dt_ids[] = {
	{ .compatible = "marvell,88pm860", },
	{},
};
MODULE_DEVICE_TABLE(of, pm860_dt_ids);


static struct i2c_driver pm860_i2c_driver = {
	.driver = {
		.name = "88pm860",
		.owner = THIS_MODULE,
		.pm = &pm860_pm_ops,
		.of_match_table	= of_match_ptr(pm860_dt_ids),
	},
	.probe = pm860_i2c_probe,
	.remove = pm860_i2c_remove,
	.id_table = pm860_i2c_id,
};

static int __init pm860_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&pm860_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pm860 I2C driver: %d\n", ret);

	return ret;
}
module_init(pm860_i2c_init);

static void __exit pm860_i2c_exit(void)
{
	i2c_del_driver(&pm860_i2c_driver);
}
module_exit(pm860_i2c_exit);

MODULE_DESCRIPTION("Core support for the PM860 audio CODEC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhao Ye <zhaoy@marvell.com>");
