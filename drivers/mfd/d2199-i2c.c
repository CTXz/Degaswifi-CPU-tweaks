/*
 * d2199-i2c.c: I2C (Serial Communication) driver for D2199
 *
 * Copyright(c) 2012 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd. D. Chen, D. Patel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/of_regulator.h>
#include <linux/d2199/pmic.h>
#include <linux/d2199/d2199_reg.h>
#include <linux/d2199/hwmon.h>
#include <linux/d2199/rtc.h>
#include <linux/d2199/core.h>
#if defined(CONFIG_D2199_DVC)
#include <mach/pxa-dvfs.h>
#endif


static const struct of_device_id d2199_dt_ids[] = {
		    { .compatible = "marvell,d2199", },
			{},
};
MODULE_DEVICE_TABLE(of, d2199_dt_ids);

static int d2199_i2c_read_device(struct d2199 *d2199, char reg,
					int bytes, void *dest)
{
	int ret;
	struct i2c_msg msgs[2];
	struct i2c_adapter *adap = d2199->pmic_i2c_client->adapter;

	msgs[0].addr = d2199->pmic_i2c_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = d2199->pmic_i2c_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = bytes;
	msgs[1].buf = (char *)dest;

	mutex_lock(&d2199->i2c_mutex);
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));

	mutex_unlock(&d2199->i2c_mutex);

	if (ret < 0)
		return ret;
	else if (ret == ARRAY_SIZE(msgs))
		return 0;
	else
		return -EFAULT;
}

static int d2199_i2c_write_device(struct d2199 *d2199, char reg,
				   int bytes, u8 *src)
{
	int ret;
	struct i2c_msg msgs[1];
	u8 data[12];
	u8 *buf = data;

	struct i2c_adapter *adap = d2199->pmic_i2c_client->adapter;

	if (bytes == 0)
		return -EINVAL;

	BUG_ON(bytes >= ARRAY_SIZE(data));

	msgs[0].addr = d2199->pmic_i2c_client->addr;
	msgs[0].flags = d2199->pmic_i2c_client->flags & I2C_M_TEN;
	msgs[0].len = 1+bytes;
	msgs[0].buf = data;

	*buf++ = reg;
	while (bytes--)
		*buf++ = *src++;

	mutex_lock(&d2199->i2c_mutex);
	ret = i2c_transfer(adap, msgs, ARRAY_SIZE(msgs));
	mutex_unlock(&d2199->i2c_mutex);

	if (ret < 0)
		return ret;
	else if (ret == ARRAY_SIZE(msgs))
		return 0;
	else
		return -EFAULT;
}

static int d2199_regulator_dt_init(struct i2c_client *i2c,
	struct of_regulator_match **regulator_matches)
{
	struct device_node *np = i2c->dev.of_node->child;
	int range;

	*regulator_matches = d2199_regulator_matches;
	range = ARRAY_SIZE(d2199_regulator_matches);
	return of_regulator_match(&i2c->dev, np, *regulator_matches, range);
}

int d2199_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct d2199 *d2199;
	int ret, i;
	int irq_no;
	const struct of_device_id *match;
	struct of_regulator_match *regulator_matches;
	struct d2199_platform_data *d2199_pdata;
	struct d2199_regl_init_data d2199_reg_data[D2199_NUMBER_OF_REGULATORS];
#if defined(CONFIG_D2199_DVC)
	struct dvfs_info d2199_info;
#endif

	dlg_info("%s() Starting I2C\n", __func__);

	match = of_match_device(d2199_dt_ids, &i2c->dev);
	if (!match) {
		dev_err(&i2c->dev, "Unknown device model\n");
		return -EINVAL;
	}

	d2199 = kzalloc(sizeof(struct d2199), GFP_KERNEL);
	if (d2199 == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}
	d2199_pdata = kzalloc(sizeof(struct d2199_platform_data), GFP_KERNEL);
	if (d2199_pdata == NULL) {
		kfree(i2c);
		return -ENOMEM;
	}

	ret = d2199_regulator_dt_init(i2c, &regulator_matches);
	if (ret < 0)
		dev_err(&i2c->dev,
			"regulator initialization failed with error %d\n", ret);

	i2c_set_clientdata(i2c, d2199);
	d2199->pmic_i2c_client = i2c;
	d2199->dev = &i2c->dev;

	mutex_init(&d2199->i2c_mutex);

	for (i = 0; i < ARRAY_SIZE(d2199_regulator_matches); i++) {
		if (!d2199->pdata) {
			d2199_reg_data[i].regl_id = i;
			 (d2199_reg_data + i)->initdata =
					 regulator_matches->init_data;
			 (d2199_reg_data + i)->of_node =
					 regulator_matches->of_node;
		}
		if (!regulator_matches->init_data)
			continue;

		regulator_matches++;
	}

	d2199_pdata->regulator_data = &d2199_reg_data[0];
	d2199->pdata = d2199_pdata;

	d2199->read_dev = d2199_i2c_read_device;
	d2199->write_dev = d2199_i2c_write_device;

	irq_no = irq_of_parse_and_map(i2c->dev.of_node, 0);

	ret = d2199_device_init(d2199, irq_no, d2199->pdata);
	dev_info(d2199->dev, "I2C initialized err=%d\n", ret);
	if (ret < 0)
		goto err;

#if defined(CONFIG_D2199_DVC)
	d2199_info.set_vccmain_volt = d2199_extern_dvc_write;
	d2199_info.get_vccmain_volt = d2199_extern_dvc_read;
	d2199_info.pmic_rampup_step = 6250;
	setup_pmic_dvfs(&d2199_info);
#endif

	dlg_info("%s() Finished I2C setup\n", __func__);
	return ret;

err:
	kfree(d2199);
	return ret;
}

static int d2199_i2c_remove(struct i2c_client *i2c)
{
	struct d2199 *d2199 = i2c_get_clientdata(i2c);

	d2199_device_exit(d2199);
	kfree(d2199);
	return 0;
}

static const struct i2c_device_id d2199_i2c_id[] = {
	{ D2199_I2C, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, d2199_i2c_id);

static struct i2c_driver d2199_i2c_driver = {
	.driver = {
		   .name = D2199_I2C,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(d2199_dt_ids),
	},
	.probe = d2199_i2c_probe,
	.remove = d2199_i2c_remove,
	.id_table = d2199_i2c_id,
};

static int __init d2199_i2c_init(void)
{
	return i2c_add_driver(&d2199_i2c_driver);
}

/* Initialised very early during bootup (in parallel with Subsystem init) */
subsys_initcall(d2199_i2c_init);

static void __exit d2199_i2c_exit(void)
{
	i2c_del_driver(&d2199_i2c_driver);
}
module_exit(d2199_i2c_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd < william.seo@diasemi.com >");
MODULE_DESCRIPTION("I2C MFD driver for Dialog D2199 PMIC plus Audio");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D2199_I2C);
