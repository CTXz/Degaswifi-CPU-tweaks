/*
 * I2C driver for Marvell 88CE170
 *
 * Copyright (C) 2013 Marvell International Ltd.
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
#include <linux/mfd/88ce170.h>
#include <linux/mfd/core.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/of_device.h>

#define CE170_REG_NUM			(0xf6)

struct regmap_config ce170_base_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = CE170_REG_NUM,
};

static struct resource ce170_codec_resources[] = {
	{
	 /* Headset microphone insertion or removal */
	 .name = "micin",
	 .start = CE170_IRQ_EXTMIC,
	 .end = CE170_IRQ_EXTMIC,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 /* Audio short HP1 */
	 .name = "audio-short1",
	 .start = CE170_IRQ_HSINSERT,
	 .end = CE170_IRQ_HSINSERT,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 /* Audio short HP2 */
	 .name = "audio-short2",
	 .start = CE170_IRQ_HSREMOVE,
	 .end = CE170_IRQ_HSREMOVE,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell ce170_devs[] = {
	{
		.name = "88ce170-codec",
		.num_resources = ARRAY_SIZE(ce170_codec_resources),
		.resources = ce170_codec_resources,
	},
};

/*
 * Instantiate the generic non-control parts of the device.
 */
static int ce170_dev_init(struct ce170_chip *ce170, int irq)
{
	int ret;
	unsigned int val;

	dev_set_drvdata(ce170->dev, ce170);

	ret = regmap_read(ce170->regmap, 0x0, &val);
	if (ret < 0) {
		dev_err(ce170->dev, "Failed to read ID register\n");
		goto err_get;
	}
	ce170->revision = val;

	dev_info(ce170->dev, "ce170 revision 0x%x\n", ce170->revision);

	ret = mfd_add_devices(ce170->dev, -1,
			      ce170_devs, ARRAY_SIZE(ce170_devs),
			      NULL, 0, NULL);
	if (ret != 0) {
		dev_err(ce170->dev, "Failed to add children: %d\n", ret);
		goto err;
	}

	return 0;

err:
	mfd_remove_devices(ce170->dev);
err_get:
	return ret;
}

static void ce170_dev_exit(struct ce170_chip *ce170)
{
	mfd_remove_devices(ce170->dev);
}

static const struct of_device_id ce170_of_match[] = {
	{ .compatible = "mrvl,ce170", },
	{ }
};
MODULE_DEVICE_TABLE(of, ce170_of_match);

static int ce170_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	struct ce170_chip *ce170;
	int ret;

	ce170 = devm_kzalloc(&i2c->dev, sizeof(struct ce170_chip), GFP_KERNEL);
	if (ce170 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ce170);
	ce170->dev = &i2c->dev;
	ce170->irq = i2c->irq;

	ce170->regmap = devm_regmap_init_i2c(i2c, &ce170_base_regmap_config);
	if (IS_ERR(ce170->regmap)) {
		ret = PTR_ERR(ce170->regmap);
		dev_err(ce170->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	return ce170_dev_init(ce170, i2c->irq);
}

static int ce170_i2c_remove(struct i2c_client *i2c)
{
	struct ce170_chip *ce170 = i2c_get_clientdata(i2c);

	ce170_dev_exit(ce170);

	return 0;
}

static const struct i2c_device_id ce170_i2c_id[] = {
	{ "88ce170", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ce170_i2c_id);

static const struct of_device_id ce170_dt_ids[] = {
	{ .compatible = "marvell,88ce170", },
	{},
};

MODULE_DEVICE_TABLE(of, pm860x_dt_ids);

static struct i2c_driver ce170_i2c_driver = {
	.driver = {
		.name = "88ce170",
		.owner = THIS_MODULE,
		.of_match_table	= of_match_ptr(ce170_dt_ids),
	},
	.probe = ce170_i2c_probe,
	.remove = ce170_i2c_remove,
	.id_table = ce170_i2c_id,
};

static int ce170_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&ce170_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register ce170 I2C driver: %d\n", ret);

	return ret;
}
module_init(ce170_i2c_init);

static void ce170_i2c_exit(void)
{
	i2c_del_driver(&ce170_i2c_driver);
}
module_exit(ce170_i2c_exit);

MODULE_DESCRIPTION("Core support for the CE170 audio CODEC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Zhao Ye <zhaoy@marvell.com>");
