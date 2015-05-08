/*
 * Base driver for Marvell 88PM822
 *
 * Copyright (C) 2012 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 * Qiao Zhou <zhouqiao@marvell.com>
 * Yipeng Yao <ypyao@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm822.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/of_device.h>

#define PM822_BASE_REG_NUM		0xF0
#define PM822_POWER_REG_NUM		0xD9
#define PM822_GPADC_REG_NUM		0xC8
#define PM822_PROC_FILE			"driver/pm822_reg"

struct i2c_client *pm822_client;

/* Interrupt Number in 88PM822 */
enum {
	PM822_IRQ_ONKEY,	/* EN1b0 *//*0 */
	PM822_IRQ_RSVD1,	/* EN1b1 */
	PM822_IRQ_CHG,		/* EN1b2 */
	PM822_IRQ_BAT,		/* EN1b3 */
	PM822_IRQ_RTC,		/* EN1b4 */
	PM822_IRQ_CLASSD,	/* EN1b5 *//*5 */
	PM822_IRQ_VBAT,		/* EN2b0 */
	PM822_IRQ_VSYS,		/* EN2b1 */
	PM822_IRQ_RSVD2,	/* EN2b2 */
	PM822_IRQ_TINT,		/* EN2b3 */
	PM822_IRQ_LDO_PGOOD,		/* EN2b4*/
	PM822_IRQ_BUCK_PGOOD,		/* EN2b5*/
	PM822_IRQ_GPADC0,	/* EN3b0 *//*10 */
	PM822_IRQ_GPADC1,	/* EN3b1 */
	PM822_IRQ_GPADC2,	/* EN3b2 */
	PM822_IRQ_GPADC3,	/* EN3b3 */
	PM822_IRQ_MIC_DET,	/* EN3b4 */
	PM822_IRQ_HS_DET,	/* EN3b5 */
	PM822_MAX_IRQ,
};

static const struct i2c_device_id pm822_id_table[] = {
	{"88PM822", 0},
	{} /* NULL terminated */
};
MODULE_DEVICE_TABLE(i2c, pm822_id_table);

static const struct of_device_id pm822_dt_ids[] = {
	{ .compatible = "marvell,88pm822", },
	{},
};
MODULE_DEVICE_TABLE(of, pm822_dt_ids);

static struct resource rtc_resources[] = {
	{
	 .name = "88pm822-rtc",
	 .start = PM822_IRQ_RTC,
	 .end = PM822_IRQ_RTC,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell rtc_devs[] = {
	{
	 .name = "88pm822-rtc",
	 .of_compatible = "marvell,88pm822-rtc",
	 .num_resources = ARRAY_SIZE(rtc_resources),
	 .resources = &rtc_resources[0],
	 .id = -1,
	 },
};
static struct resource onkey_resources[] = {
	{
	 .name = "88pm822-onkey",
	 .start = PM822_IRQ_ONKEY,
	 .end = PM822_IRQ_ONKEY,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell onkey_devs[] = {
	{
	 .name = "88pm822-onkey",
	 .of_compatible = "marvell,88pm822-onkey",
	 .num_resources = 1,
	 .resources = &onkey_resources[0],
	 .id = -1,
	 },
};

static struct mfd_cell regulator_devs[] = {
	{
	 .name = "88pm822-regulator",
	 .of_compatible = "marvell,88pm822-regulator",
	 .id = -1,
	},
};
#if 0
static struct resource usb_resources[] = {
	{
	.name = "88pm822-usb",
	.start = PM822_IRQ_CHG,
	.end = PM822_IRQ_CHG,
	.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell usb_devs[] = {
	{
	.name = "88pm822-usb",
	 .of_compatible = "marvell,88pm822-usb",
	.num_resources = 1,
	.resources = &usb_resources[0],
	.id = -1,
	},
};
#endif
static struct mfd_cell vibrator_devs[] = {
	{
	 .name = "88pm822-vibrator",
	 .of_compatible = "marvell,88pm822-vibrator",
	 .id = -1,
	},
};

static struct mfd_cell dvc_devs[] = {
	{
	 .name = "88pm8xx-dvc",
	 .of_compatible = "marvell,88pm822-dvc",
	 .id = -1,
	},
};

static const struct regmap_irq pm822_irqs[] = {
	/* INT0 */
	[PM822_IRQ_ONKEY] = {
		.mask = PM822_IRQ_ONKEY_EN,
	},
	[PM822_IRQ_RSVD1] = {
		.mask = 0,
	},
	[PM822_IRQ_CHG] = {
		.mask = PM822_IRQ_CHG_EN,
	},
	[PM822_IRQ_BAT] = {
		.mask = PM822_IRQ_BAT_EN,
	},
	[PM822_IRQ_RTC] = {
		.mask = PM822_IRQ_RTC_EN,
	},
	[PM822_IRQ_CLASSD] = {
		.mask = PM822_IRQ_CLASSD_EN,
	},
	/* INT1 */
	[PM822_IRQ_VBAT] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_VBAT_EN,
	},
	[PM822_IRQ_VSYS] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_VSYS_EN,
	},
	[PM822_IRQ_LDO_PGOOD] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_LDO_PGOOD_EN,
	},
	[PM822_IRQ_BUCK_PGOOD] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_BUCK_PGOOD_EN,
	},
	[PM822_IRQ_RSVD2] = {
		.reg_offset = 1,
		.mask = 0,
	},
	[PM822_IRQ_TINT] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_TINT_EN,
	},
	/* INT2 */
	[PM822_IRQ_GPADC0] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC0_EN,
	},
	[PM822_IRQ_GPADC1] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC1_EN,
	},
	[PM822_IRQ_GPADC2] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC2_EN,
	},
	[PM822_IRQ_GPADC3] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_GPADC3_EN,
	},
	[PM822_IRQ_MIC_DET] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_MIC_DET_EN,
	},
	[PM822_IRQ_HS_DET] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_HS_DET_EN,
	},
};

static int device_gpadc_init(struct pm822_chip *chip,
				       struct pm822_platform_data *pdata)
{
	/* FIXME add inital seq for gpadc */
	return 0;
}

static int device_onkey_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			      ARRAY_SIZE(onkey_devs), &onkey_resources[0], 0,
			      NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		return ret;
	} else
		dev_info(chip->dev,
			"[%s]:Added mfd onkey_devs\n", __func__);

	return 0;
}

static int device_rtc_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	int ret;

	if (pdata && pdata->rtc) {

		rtc_devs[0].platform_data = pdata->rtc;
		rtc_devs[0].pdata_size =
				pdata->rtc ? sizeof(struct pm822_rtc_pdata) : 0;
		ret = mfd_add_devices(chip->dev, 0, &rtc_devs[0],
				      ARRAY_SIZE(rtc_devs), NULL, 0, NULL);
		if (ret) {
			dev_err(chip->dev, "Failed to add rtc subdev\n");
			return ret;
		} else
			dev_info(chip->dev,
				"[%s]:Added mfd rtc_devs\n", __func__);
	}

	return 0;
}
#if 0
static int device_usb_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	int ret;

	if (pdata && pdata->usb) {
		usb_devs[0].platform_data = pdata->usb;
		usb_devs[0].pdata_size = sizeof(struct pm822_usb_pdata);
		ret = mfd_add_devices(chip->dev, 0, &usb_devs[0],
				      ARRAY_SIZE(usb_devs), NULL, 0, NULL);
		if (ret < 0) {
			dev_err(chip->dev, "Failed to add usb subdev\n");
			return ret;
		} else
			dev_info(chip->dev,
				 "[%s]:Added mfd usb_devs\n", __func__);
	}

	return 0;
}
#endif
static int device_vibrator_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	int ret;

	vibrator_devs[0].platform_data = pdata->vibrator;
	vibrator_devs[0].pdata_size =
					sizeof(struct pm822_vibrator_pdata);
	ret = mfd_add_devices(chip->dev, 0, &vibrator_devs[0],
			ARRAY_SIZE(vibrator_devs), NULL, 0, NULL);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add vibrator subdev\n");
		return ret;
	} else
		dev_info(chip->dev,
			"[%s]:Added mfd vibrator_devs\n", __func__);

	return 0;

}

static int device_regulator_init(struct pm822_chip *chip,
				struct pm822_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &regulator_devs[0],
			      ARRAY_SIZE(regulator_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add regulator subdev\n");
		return ret;
	}

	return 0;
}

static int device_dvc_init(struct pm822_chip *chip,
			   struct pm822_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &dvc_devs[0],
			      ARRAY_SIZE(dvc_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add dvc subdev\n");
		return ret;
	}
	return 0;
}

static int device_pm822_irq_init(struct pm822_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned long flags = IRQF_ONESHOT;
	int data, mask, ret = -EINVAL;

	if (!map || !chip->irq) {
		dev_err(chip->dev, "incorrect parameters\n");
		return -EINVAL;
	}

	/* irq pin: low active
	 * irq clear on write
	 * irq status bit: not set when it is masked
	 */
	mask = PM822_INV_INT | PM822_INT_CLEAR_MODE | PM822_INT_MASK_MODE;
	data = PM822_INT_CLEAR_MODE;
	ret = regmap_update_bits(map, PM822_WAKEUP2, mask, data);
	if (ret < 0)
		goto out;

	ret = regmap_add_irq_chip(chip->regmap, chip->irq, flags, -1,
				chip->regmap_irq_chip, &chip->irq_data);
out:
	return ret;
}

static void device_pm822_irq_exit(struct pm822_chip *chip)
{
	regmap_del_irq_chip(chip->irq, chip->irq_data);
}

static struct regmap_irq_chip pm822_irq_chip = {
	.name = "88pm822",
	.irqs = pm822_irqs,
	.num_irqs = ARRAY_SIZE(pm822_irqs),

	.num_regs = 3,
	.status_base = PM822_INT_STATUS1,
	.mask_base = PM822_INT_EN1,
	.ack_base = PM822_INT_STATUS1,
	.mask_invert = 1,
};

static int pm822_pages_init(struct pm822_chip *chip)
{
	struct pm82x_subchip *subchip = chip->subchip;
	struct i2c_client *client = chip->client;
	int ret = 0;

	if (!subchip || !subchip->power_page_addr || !subchip->gpadc_page_addr)
		return -ENODEV;

	subchip->power_page = i2c_new_dummy(client->adapter,
					    subchip->power_page_addr);
	if (subchip->power_page == NULL) {
		ret = -ENODEV;
		goto out;
	}

	subchip->regmap_power = devm_regmap_init_i2c(subchip->power_page,
							&pm82x_regmap_config);
	if (IS_ERR(subchip->regmap_power)) {
		ret = PTR_ERR(subchip->regmap_power);
		dev_err(chip->dev,
			"Failed to allocate regmap_power: %d\n", ret);
		goto out;
	}

	i2c_set_clientdata(subchip->power_page, chip);

	/* PM822 block GPADC */
	subchip->gpadc_page = i2c_new_dummy(client->adapter,
					    subchip->gpadc_page_addr);
	if (subchip->gpadc_page == NULL) {
		ret = -ENODEV;
		goto out;
	}

	subchip->regmap_gpadc = devm_regmap_init_i2c(subchip->gpadc_page,
						     &pm82x_regmap_config);
	if (IS_ERR(subchip->regmap_gpadc)) {
		ret = PTR_ERR(subchip->regmap_gpadc);
		dev_err(chip->dev,
			"Failed to allocate regmap_gpadc: %d\n", ret);
		goto out;
	}
	i2c_set_clientdata(subchip->gpadc_page, chip);

out:
	return ret;
}

static void pm822_pages_exit(struct pm822_chip *chip)
{
	struct pm82x_subchip *subchip = chip->subchip;

	if (subchip && subchip->power_page)
		i2c_unregister_device(subchip->power_page);

	if (subchip && subchip->gpadc_page)
		i2c_unregister_device(subchip->gpadc_page);
}

static int device_pm822_init(struct pm822_chip *chip,
				     struct pm822_platform_data *pdata)
{
	int ret;
	unsigned int val;

	/*
	 * alarm wake up bit will be clear in device_irq_init(),
	 * read before that
	 */
	ret = regmap_read(chip->regmap, PM822_RTC_CTRL, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read RTC register: %d\n", ret);
		goto out;
	}
	if (val & PM822_ALARM_WAKEUP) {
		if (pdata && pdata->rtc)
			pdata->rtc->rtc_wakeup = 1;
	}

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init gpadc\n", __func__);
		goto out;
	}

	chip->regmap_irq_chip = &pm822_irq_chip;

	ret = device_pm822_irq_init(chip);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init pm822 irq\n", __func__);
		goto out;
	}

	ret = device_regulator_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add regulators subdev\n");
		goto out_dev;
	}

	ret = device_onkey_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	} else
		dev_info(chip->dev, "onkey init success!\n");

	ret = device_rtc_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add rtc subdev\n");
		goto out;
	}
#if 0
	ret = device_usb_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add usb subdev\n");
		goto out;
	}
#endif
	ret = device_vibrator_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add vibrator subdev\n");
		goto out;
	}

	ret = device_dvc_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add dvc subdev\n");

		goto out;
	}

	return 0;

out_dev:
	mfd_remove_devices(chip->dev);
	device_pm822_irq_exit(chip);
out:
	return ret;
}

static int pm822_dt_init(struct device_node *np,
			 struct device *dev,
			 struct pm822_platform_data *pdata)
{
	pdata->irq_mode =
		!of_property_read_bool(np, "marvell,88pm822-irq-write-clear");
	pdata->batt_det =
		of_property_read_bool(np, "marvell,88pm822-battery-detection");

	return 0;
}

static int pm822_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct pm822_chip *chip;
	struct pm822_platform_data *pdata = client->dev.platform_data;
	struct device_node *node = client->dev.of_node;
	struct pm82x_subchip *subchip;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = pm822_dt_init(node, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		return -EINVAL;
	}

	/*
	 * RTC in pmic can run even the core is powered off, and user can set
	 * alarm in RTC. When the alarm is time out, the PMIC will power up
	 * the core, and the whole system will boot up. When PMIC driver is
	 * probed, it will read out some register to find out whether this
	 * boot is caused by RTC timeout or not, and it need pass this
	 * information to RTC driver.
	 * So we need rtc platform data to be existed to pass this information.
	 */
	if (!pdata->rtc) {
		pdata->rtc = devm_kzalloc(&client->dev,
					  sizeof(*(pdata->rtc)), GFP_KERNEL);
		if (!pdata->rtc)
			return -ENOMEM;
	}

	ret = pm82x_init(client);
	if (ret) {
		dev_err(&client->dev, "pm822_init fail\n");
		goto out_init;
	}

	chip = i2c_get_clientdata(client);

	/* init subchip for PM800 */
	subchip =
	    devm_kzalloc(&client->dev, sizeof(struct pm82x_subchip),
			 GFP_KERNEL);
	if (!subchip) {
		ret = -ENOMEM;
		goto err_subchip_alloc;
	}

	/* pm822 has 2 addtional pages to support power and gpadc. */
	subchip->power_page_addr = client->addr + 1;
	subchip->gpadc_page_addr = client->addr + 2;
	chip->subchip = subchip;

	ret = pm822_pages_init(chip);
	if (ret) {
		dev_err(&client->dev, "pm822_pages_init failed!\n");
		goto err_page_init;
	}

	ret = device_pm822_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to initialize 88pm822 devices\n");
		goto err_device_init;
	}

	if (pdata && pdata->plat_config)
		pdata->plat_config(chip, pdata);

	return 0;

err_device_init:
	pm822_pages_exit(chip);
err_page_init:
err_subchip_alloc:
	pm82x_deinit();
out_init:
	return ret;
}

static int pm822_remove(struct i2c_client *client)
{
	struct pm822_chip *chip = i2c_get_clientdata(client);

	mfd_remove_devices(chip->dev);
	device_pm822_irq_exit(chip);

	pm822_pages_exit(chip);
	pm82x_deinit();

	return 0;
}

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
static struct i2c_driver pm822_driver = {
	.driver = {
		.name = "88PM822",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pm822_dt_ids),
#ifdef CONFIG_PM
		.pm = &pm822_pm_ops,
#endif
		},
	.probe = pm822_probe,
	.remove =  pm822_remove,
	.id_table = pm822_id_table,
};

static int __init pm822_i2c_init(void)
{
	return i2c_add_driver(&pm822_driver);
}
subsys_initcall(pm822_i2c_init);

static void __exit pm822_i2c_exit(void)
{
	i2c_del_driver(&pm822_driver);
}
module_exit(pm822_i2c_exit);

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM822");
MODULE_AUTHOR("Yipeng Yao <ypyao@marvell.com>");
MODULE_LICENSE("GPL");
