/*
 * Marvell 88PM80x ONKEY driver
 *
 * Copyright (C) 2012 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 * Qiao Zhou <zhouqiao@marvell.com>
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
#include <linux/input.h>
#include <linux/mfd/88pm80x.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif

#ifdef CONFIG_SEC_DEBUG
#include <linux/sec-debug.h>
#endif

#define PM800_LONG_ONKEY_EN1		(1 << 0)
#define PM800_LONG_ONKEY_EN2		(1 << 1)
#define PM800_LONG_ONKEY_MASK		(0x03)
#define PM800_LONG_KEY_DELAY		(8)	/* 1 .. 16 seconds */
#define PM800_LONKEY_PRESS_TIME		((PM800_LONG_KEY_DELAY-1) << 4)
#define PM800_LONKEY_PRESS_TIME_MASK	(0xF0)
#define PM800_LONKEY_RESET		(1 << 3)
#define PM800_SW_PDOWN			(1 << 5)

struct pm80x_onkey_info {
	struct input_dev *idev;
	struct pm80x_chip *pm80x;
	struct regmap *map;
	int irq;
#ifdef CONFIG_MACH_PXA_SAMSUNG
	struct device *sec_power_key;
	bool state;
#endif
};

/* 88PM80x gives us an interrupt when ONKEY is held */
static irqreturn_t pm80x_onkey_handler(int irq, void *data)
{
	struct pm80x_onkey_info *info = data;
	int ret = 0;
	unsigned int val;

	/* reset the LONGKEY reset time */
	regmap_update_bits(info->map, PM800_WAKEUP1,
			   PM800_LONKEY_RESET, PM800_LONKEY_RESET);
	ret = regmap_read(info->map, PM800_STATUS_1, &val);
	if (ret < 0) {
		dev_err(info->idev->dev.parent, "failed to read status: %d\n", ret);
		return IRQ_NONE;
	}
	val &= PM800_ONKEY_STS1;

#ifdef CONFIG_MACH_PXA_SAMSUNG
	info->state = val;
#endif
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
	pr_info("[KEY] %d gpio_key %s\n", KEY_POWER, val ? "Press" : "Release");
#else
	pr_info("[KEY] gpio_key %s\n", val ? "Press" : "Release");
#endif

	input_report_key(info->idev, KEY_POWER, val);
	input_sync(info->idev);

	return IRQ_HANDLED;
}

static SIMPLE_DEV_PM_OPS(pm80x_onkey_pm_ops, pm80x_dev_suspend,
			 pm80x_dev_resume);

#ifdef CONFIG_MACH_PXA_SAMSUNG
static ssize_t show_key_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pm80x_onkey_info *info = dev_get_drvdata(dev);
	bool state = info->state;

	return sprintf(buf, state ? "PRESS" : "RELEASE");
}
static DEVICE_ATTR(sec_power_key_pressed, S_IRUGO, show_key_status, NULL);

static struct attribute *power_key_attributes[] = {
	&dev_attr_sec_power_key_pressed.attr,
	NULL,
};

static struct attribute_group power_key_attr_group = {
	.attrs = power_key_attributes,
};
#endif

static int pm80x_onkey_probe(struct platform_device *pdev)
{

	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm80x_onkey_info *info;
	int err;

	info = kzalloc(sizeof(struct pm80x_onkey_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pm80x = chip;

	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		err = -EINVAL;
		goto out;
	}

	info->map = info->pm80x->regmap;
	if (!info->map) {
		dev_err(&pdev->dev, "no regmap!\n");
		err = -EINVAL;
		goto out;
	}

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(&pdev->dev, "Failed to allocate input dev\n");
		err = -ENOMEM;
		goto out;
	}

	info->idev->name = "88pm80x_on";
	info->idev->phys = "88pm80x_on/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	__set_bit(KEY_POWER, info->idev->keybit);

	err = pm80x_request_irq(info->pm80x, info->irq, pm80x_onkey_handler,
					    IRQF_ONESHOT | IRQF_NO_SUSPEND,
					    "onkey", info);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, err);
		goto out_reg;
	}

	err = input_register_device(info->idev);
	if (err) {
		dev_err(&pdev->dev, "Can't register input device: %d\n", err);
		goto out_irq;
	}

	platform_set_drvdata(pdev, info);

	if (!sec_debug_level.en.kernel_fault) {
		/* Debug Level LOW, Forces a supply power down */
		/* Enable long onkey1  detection */
		regmap_update_bits(info->map, PM800_RTC_MISC4, PM800_LONG_ONKEY_MASK,
				   PM800_LONG_ONKEY_EN1);
	} else {
		/* Enable long onkey2  detection */
		regmap_update_bits(info->map, PM800_RTC_MISC4, PM800_LONG_ONKEY_MASK,
				   PM800_LONG_ONKEY_EN2);
	}

	/* Set 8-second interval */
	regmap_update_bits(info->map, PM800_RTC_MISC3,
			   PM800_LONKEY_PRESS_TIME_MASK,
			   PM800_LONKEY_PRESS_TIME);

#ifdef CONFIG_MACH_PXA_SAMSUNG
	info->sec_power_key = device_create(sec_class, NULL, (dev_t)NULL,
									info, "sec_power_key");
	if (IS_ERR(info->sec_power_key)) {
		err = PTR_ERR(info->sec_power_key);
		dev_err(&pdev->dev,
					"Failed to create device for sec power key(%d)\n", err);
	}

	err = sysfs_create_group(&info->sec_power_key->kobj,
								&power_key_attr_group);
	if (err)
		dev_err(&pdev->dev,
					"Failed to create sysfs for sec power key(%d)\n", err);
#endif

	device_init_wakeup(&pdev->dev, 1);
	return 0;

out_irq:
	pm80x_free_irq(info->pm80x, info->irq, info);
out_reg:
	input_free_device(info->idev);
out:
	kfree(info);
	return err;
}

static int pm80x_onkey_remove(struct platform_device *pdev)
{
	struct pm80x_onkey_info *info = platform_get_drvdata(pdev);

#ifdef CONFIG_MACH_PXA_SAMSUNG
	sysfs_remove_group(&info->sec_power_key->kobj, &power_key_attr_group);
	device_destroy(sec_class, (dev_t)NULL);
#endif

	device_init_wakeup(&pdev->dev, 0);
	pm80x_free_irq(info->pm80x, info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}

static struct platform_driver pm80x_onkey_driver = {
	.driver = {
		   .name = "88pm80x-onkey",
		   .owner = THIS_MODULE,
		   .pm = &pm80x_onkey_pm_ops,
		   },
	.probe = pm80x_onkey_probe,
	.remove = pm80x_onkey_remove,
};

module_platform_driver(pm80x_onkey_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell 88PM80x ONKEY driver");
MODULE_AUTHOR("Qiao Zhou <zhouqiao@marvell.com>");
MODULE_ALIAS("platform:88pm80x-onkey");
