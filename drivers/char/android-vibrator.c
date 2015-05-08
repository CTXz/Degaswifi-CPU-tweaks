/*
 * android vibrator driver
 *
 * Copyright (C) 2013 Samsung. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <plat/mfp.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#include "../staging/android/timed_output.h"

static struct wake_lock vib_wake_lock;

struct vibrator_data {
	int vib_ldo_en ;
	struct timed_output_dev vibrator_timed_dev;
	struct timer_list vibrate_timer;
	struct work_struct vibrator_off_work;
};

static void vibrator_onoff(struct vibrator_data *info, int onoff)
{
	int ret;
	ret = gpio_direction_output(info->vib_ldo_en, onoff);
	if (ret) {
			pr_err("error setting direction (%d) for vib_ldo_en\n",onoff);
			return ret;
	}
	return ret;

}
static void vibrator_off_worker(struct work_struct *work)
{
	struct vibrator_data *info;

	info = container_of(work, struct vibrator_data, vibrator_off_work);
	vibrator_onoff(info,0);
	if(wake_lock_active(&vib_wake_lock))
		wake_unlock(&vib_wake_lock);
}

static void on_vibrate_timer_expired(unsigned long x)
{
	struct vibrator_data *info;

	info = (struct vibrator_data *)x;
	schedule_work(&info->vibrator_off_work);
}

static void vibrator_enable_set_timeout(struct timed_output_dev *sdev,
								int timeout)
{
	struct vibrator_data *info;

	info = container_of(sdev, struct vibrator_data, vibrator_timed_dev);
	pr_info("Vibrator: Set duration: %dms\n", timeout);

	if (timeout == 0)
	{
		vibrator_onoff(info,0);
		if(wake_lock_active(&vib_wake_lock))
			wake_unlock(&vib_wake_lock);
	}
	else {
		cancel_work_sync(&info->vibrator_off_work);
		vibrator_onoff(info,1);
		if(!wake_lock_active(&vib_wake_lock))
			wake_lock(&vib_wake_lock);
		mod_timer(&info->vibrate_timer, jiffies +
					msecs_to_jiffies(timeout));
	}
	return;
}

static int vibrator_get_remaining_time(struct timed_output_dev *sdev)
{
	struct vibrator_data *info;

	info = container_of(sdev, struct vibrator_data, vibrator_timed_dev);
	int retTime = jiffies_to_msecs(jiffies - info->vibrate_timer.expires);
	pr_info("Vibrator: Current duration: %dms\n", retTime);

	return retTime;
}
#ifdef CONFIG_OF
static const struct of_device_id android_vibrator_dt_ids[] = {
	{ .compatible = "marvell,android-vibrator", },
	{},
};

static int get_vibrator_platdata(struct device_node *np,
					struct vibrator_data *info)
{
/*	
	const __be32 *min_timeout, *duty_cycle;

	min_timeout = of_get_property(np, "min_timeout", NULL);

	if (min_timeout)
		info->min_timeout = be32_to_cpu(*min_timeout);
	else
		pr_err(" vibrator min time out is NULL\n");


	duty_cycle = of_get_property(np, "duty_cycle", NULL);

	if (duty_cycle)
		info->duty_cycle = be32_to_cpu(*duty_cycle);
	else
		pr_err("vibrator duty cycle out is NULL\n");

	return 0;
*/
	info->vib_ldo_en = of_get_named_gpio(np, "android_vibrator,vib_ldo_en", 0);
			if (unlikely(info->vib_ldo_en < 0)) {
					pr_err( "error reading property vled_ldo_en from device node\n");
			}
			
	return 0;


}
#endif

static int vibrator_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;

	struct vibrator_data *info = kzalloc(sizeof(struct vibrator_data),
								GFP_KERNEL);
	
	if (!info)
		return -ENOMEM;
	if (np != NULL)
		get_vibrator_platdata(np, info);

	ret = gpio_request(info->vib_ldo_en, "android_vibrator");
	if (unlikely(ret < 0)) {
		pr_err("vibrator_probe error requesting gpio\n");
	}
	
/*
	if (pdev->dev.platform_data)
		info->pdata = pdev->dev.platform_data;
	else {
		pr_err("Invalid vibrator platform data\n");
		kfree(info);
		return -EINVAL;
	}
*/
	/* Setup timed_output obj */
	info->vibrator_timed_dev.name = "vibrator";
	info->vibrator_timed_dev.enable = vibrator_enable_set_timeout;
	info->vibrator_timed_dev.get_time = vibrator_get_remaining_time;
	/* Vibrator dev register in /sys/class/timed_output/ */
	ret = timed_output_dev_register(&info->vibrator_timed_dev);
	if (ret < 0) {
		pr_err("Vibrator: timed_output dev registration failure\n");
		timed_output_dev_unregister(&info->vibrator_timed_dev);
	}

	init_timer(&info->vibrate_timer);
	info->vibrate_timer.function = on_vibrate_timer_expired;
	info->vibrate_timer.data = (unsigned long)info;
	INIT_WORK(&info->vibrator_off_work, vibrator_off_worker);

	platform_set_drvdata(pdev, info);
	wake_lock_init(&vib_wake_lock, WAKE_LOCK_SUSPEND, "vib_present");
	return 0;
}

static int  vibrator_remove(struct platform_device *pdev)
{
	struct vibrator_data *info;

	info = platform_get_drvdata(pdev);
	timed_output_dev_unregister(&info->vibrator_timed_dev);
	wake_lock_destroy(&vib_wake_lock);
	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe = vibrator_probe,
	.remove = vibrator_remove,
	.driver = {
		.name = "android-vibrator",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(android_vibrator_dt_ids),
#endif
	},
};

static int __init vibrator_init(void)
{
	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vibrator_init);
module_exit(vibrator_exit);

MODULE_DESCRIPTION("Android Vibrator driver");
MODULE_LICENSE("GPL");
