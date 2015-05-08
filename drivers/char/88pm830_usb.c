/*
 * 88pm830 VBus driver for Marvell USB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mfd/88pm830.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/platform_data/mv_usb.h>

struct pm830_usb_info {
	struct pm830_chip	*chip;
	int			vbus_irq;
	int			id_irq;
	int			vbus_gpio;
	int			id_gpadc;
};

static struct pm830_usb_info *usb_info;

static int pm830_get_vbus(unsigned int *level)
{
	int ret;
	unsigned int val;

	ret = regmap_read(usb_info->chip->regmap, PM830_STATUS, &val);
	if (ret)
		return ret;

	if (val & PM830_VBUS_STATUS)
		*level = VBUS_HIGH;
	else
		*level = VBUS_LOW;
	return 0;
}

static int pm830_set_vbus(unsigned int vbus)
{
	int ret;
	unsigned int data = 0;

	if (vbus == VBUS_HIGH) {
		/* 1. [0xf0].0 = 1'b1 */
		regmap_update_bits(usb_info->chip->regmap,
				   0xf0, (1 << 0), (1 << 0));
		/* 2. [0x3c].7 = 1'b1 */
		ret = regmap_update_bits(usb_info->chip->regmap,
					 PM830_PRE_REGULATOR, PM830_USB_OTG_EN,
					 PM830_USB_OTG_EN);
		/* 3. [0xdb].[3,2,0] = 3'b1 */
		regmap_update_bits(usb_info->chip->regmap,
				   0xdb, 0xd, 0xd);
		/* 4. wait for 2ms */
		usleep_range(2000, 3000);
		/* 5. [0xdb].[3,2,0] = 3'b0 */
		regmap_update_bits(usb_info->chip->regmap,
				   0xdb, 0xd, 0x0);
		/* 6. [0xf0].0 = 1'b0 */
		regmap_update_bits(usb_info->chip->regmap,
				   0xf0, (1 << 0), (0 << 0));
	}else
		ret = regmap_update_bits(usb_info->chip->regmap,
					 PM830_PRE_REGULATOR,
					 PM830_USB_OTG_EN, 0);

	if (ret)
		return ret;

	usleep_range(10000, 20000);

	ret = pm830_get_vbus(&data);
	if (ret)
		return ret;

	if (data != vbus)
		pr_info("vbus set failed %x\n", vbus);
	else
		pr_info("vbus set done %x\n", vbus);

	return 0;
}

static int pm830_read_id_val(unsigned int *level)
{
	int ret, data;
	unsigned int val;
	unsigned int meas1, meas2, upp_th, low_th;

	switch (usb_info->id_gpadc) {
	case PM830_GPADC0:
		meas1 = PM830_GPADC0_MEAS1;
		meas2 = PM830_GPADC0_MEAS2;
		low_th = PM830_GPADC0_LOW_TH;
		upp_th = PM830_GPADC0_UPP_TH;
		break;
	case PM830_GPADC1:
		meas1 = PM830_GPADC1_MEAS1;
		meas2 = PM830_GPADC1_MEAS2;
		low_th = PM830_GPADC1_LOW_TH;
		upp_th = PM830_GPADC1_UPP_TH;
		break;
	default:
		return -ENODEV;
	}

	ret = regmap_read(usb_info->chip->regmap, meas1, &val);
	data = val << 4;
	if (ret)
		return ret;

	ret = regmap_read(usb_info->chip->regmap, meas2, &val);
	data |= val & 0x0f;
	if (ret)
		return ret;
	if (data > 0x10) {
		regmap_write(usb_info->chip->regmap, low_th, 0x10);
		regmap_write(usb_info->chip->regmap, upp_th, 0xff);
		*level = 1;
	} else {
		regmap_write(usb_info->chip->regmap, low_th, 0);
		regmap_write(usb_info->chip->regmap, upp_th, 0x10);
		*level = 0;
	}

	return 0;
};

int pm830_init_id(void)
{
	int ret;
	unsigned int en, low_th, upp_th;

	switch (usb_info->id_gpadc) {
	case PM830_GPADC0:
		low_th = PM830_GPADC0_LOW_TH;
		upp_th = PM830_GPADC0_UPP_TH;
		en = PM830_GPADC0_MEAS_EN;
		break;
	case PM830_GPADC1:
		low_th = PM830_GPADC1_LOW_TH;
		upp_th = PM830_GPADC1_UPP_TH;
		en = PM830_GPADC1_MEAS_EN;
		break;
	default:
		return -ENODEV;
	}

	/* set the threshold for GPADC to prepare for interrupt */
	regmap_write(usb_info->chip->regmap, low_th, 0x10);
	regmap_write(usb_info->chip->regmap, upp_th, 0xff);

	ret = regmap_update_bits(usb_info->chip->regmap,
				 PM830_GPADC_MEAS_EN, en, en);
	if (ret)
		return ret;
	/* the global enable has been done in host driver */

	return 0;
}

static irqreturn_t pm830_vbus_handler(int irq, void *data)
{

	struct pm830_usb_info *info = data;
	/*
	 * 88pm830 has no ability to distinguish
	 * AC/USB charger, so notify usb framework to do it
	 */
	pxa_usb_notify(PXA_USB_DEV_OTG, EVENT_VBUS, 0);
	dev_dbg(info->chip->dev, "88pm830 vbus interrupt is served..\n");
	return IRQ_HANDLED;
}

static irqreturn_t pm830_id_handler(int irq, void *data)
{
	struct pm830_usb_info *info = data;

	 /* notify to wake up the usb subsystem if ID pin is pulled down */
	pxa_usb_notify(PXA_USB_DEV_OTG, EVENT_ID, 0);
	dev_dbg(info->chip->dev, "88pm830 id interrupt is served..\n");
	return IRQ_HANDLED;
}

static int pm830_usb_dt_init(struct device_node *np,
			     struct device *dev,
			     struct pm830_usb_pdata *pdata)
{
	return of_property_read_u32(np, "gpadc-number", &pdata->id_gpadc);
}

static int pm830_usb_probe(struct platform_device *pdev)
{
	struct pm830_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm830_usb_pdata *pdata;
	struct pm830_usb_info *usb;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	usb = devm_kzalloc(&pdev->dev,
			   sizeof(struct pm830_usb_info), GFP_KERNEL);
	if (!usb)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&pdev->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = pm830_usb_dt_init(node, &pdev->dev, pdata);
		if (ret)
			goto out;
	} else if (!pdata) {
		return -EINVAL;
	}

	usb->chip = chip;
	usb->vbus_gpio = pdata->vbus_gpio;
	usb->id_gpadc = pdata->id_gpadc;

	usb->vbus_irq = platform_get_irq(pdev, 0);
	if (usb->vbus_irq < 0) {
		dev_err(&pdev->dev, "failed to get vbus irq\n");
		ret = -ENXIO;
		goto out;
	}

	usb->id_irq = platform_get_irq(pdev, usb->id_gpadc + 1);
	if (usb->id_irq < 0) {
		dev_err(&pdev->dev, "failed to get idpin irq\n");
		ret = -ENXIO;
		goto out;
	}

	ret = devm_request_threaded_irq(&pdev->dev, usb->vbus_irq, NULL,
					pm830_vbus_handler,
					IRQF_ONESHOT | IRQF_NO_SUSPEND,
					"usb detect", usb);
	if (ret) {
		dev_info(&pdev->dev,
			"cannot request irq for VBUS, return\n");
		goto out;
	}

	ret = devm_request_threaded_irq(&pdev->dev, usb->id_irq, NULL,
					pm830_id_handler,
					IRQF_ONESHOT | IRQF_NO_SUSPEND,
					"id detect", usb);
	if (ret) {
		dev_info(&pdev->dev,
			"cannot request irq for idpin, return\n");
		goto out;
	}

	/* global variable used by get/set_vbus */
	usb_info = usb;

	platform_set_drvdata(pdev, usb);
	device_init_wakeup(&pdev->dev, 1);

	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, set_vbus,
				pm830_set_vbus);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, get_vbus,
				pm830_get_vbus);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, get_idpin,
				pm830_read_id_val);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, init,
				pm830_init_id);

	/*
	 * GPADC is enabled in otg driver via calling pm830_init_id()
	 * without pm830_init_id(), the GPADC is disabled
	 * pm830_init_id();
	 */

	return 0;

out:
	return ret;
}

static int pm830_usb_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM
static SIMPLE_DEV_PM_OPS(pm830_usb_pm_ops, pm830_dev_suspend,
			 pm830_dev_resume);
#endif

static const struct of_device_id pm830_vbus_dt_match[] = {
	{ .compatible = "marvell,88pm830-vbus", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm830_fg_dt_match);

static struct platform_driver pm830_usb_driver = {
	.driver		= {
		.name	= "88pm830-vbus",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(pm830_vbus_dt_match),
	},
	.probe		= pm830_usb_probe,
	.remove		= pm830_usb_remove,
};

static int pm830_usb_init(void)
{
	return platform_driver_register(&pm830_usb_driver);
}
module_init(pm830_usb_init);

static void pm830_usb_exit(void)
{
	platform_driver_unregister(&pm830_usb_driver);
}
module_exit(pm830_usb_exit);

MODULE_DESCRIPTION("VBUS driver for Marvell Semiconductor 88PM830");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm830-usb");
