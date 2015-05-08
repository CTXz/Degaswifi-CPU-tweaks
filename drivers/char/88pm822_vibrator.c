/* Copyright (C) 2010 Marvell */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm822.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "../staging/android/timed_output.h"

struct pm822_vibrator_info {
	struct device *vib_dev;
	struct regmap *map;
	struct regulator *vibr_power;
	struct timed_output_dev vibrator_timed_dev;
	struct timer_list vibrate_timer;
	struct work_struct vibrator_off_work;
	struct mutex vib_mutex;
	int enable;
	int min_timeout;
	int duty_cycle;  /*sync with patch:" char: vibrator: add
				support for setting duty cycle" in 988*/
};

#define VIBRA_OFF_VALUE	0
#define VIBRA_ON_VALUE	1

static void vibrator_set_power(int on, struct pm822_vibrator_info *info)
{
	int ret = 0;

	if (on)
		ret = regulator_enable(info->vibr_power);
	else
		ret = regulator_disable(info->vibr_power);

	if (ret) {
		dev_err(info->vib_dev,
			"failed to enable/disable regulator: %d\n", ret);
		return;
	}

}


static int get_vibrator_platdata(struct device_node *np,
					struct pm822_vibrator_info *info)
{
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
}

int pm822_control_vibrator(struct pm822_vibrator_info *info,
				unsigned char value)
{

	mutex_lock(&info->vib_mutex);
	if (info->enable == value) {
		mutex_unlock(&info->vib_mutex);
		return 0;
	}

	if (value == VIBRA_OFF_VALUE) {
		regmap_write(info->map, PM822_PWM4, 0x0);
		if (info->vibr_power)
			vibrator_set_power(0, info);
	} else if (value == VIBRA_ON_VALUE) {
		if (info->vibr_power)
			vibrator_set_power(1, info);
		regmap_write(info->map, PM822_PWM1,
			info->duty_cycle ? info->duty_cycle : 0x3f);
		regmap_write(info->map, PM822_PWM4, 0x1);
	}
	info->enable = value;
	mutex_unlock(&info->vib_mutex);

	return 0;
}

static void vibrator_off_worker(struct work_struct *work)
{
	struct pm822_vibrator_info *info;

	info = container_of(work, struct pm822_vibrator_info,
				vibrator_off_work);
	pm822_control_vibrator(info, VIBRA_OFF_VALUE);
}

static void on_vibrate_timer_expired(unsigned long x)
{
	struct pm822_vibrator_info *info;
	info = (struct pm822_vibrator_info *)x;
	schedule_work(&info->vibrator_off_work);
}

static void vibrator_enable_set_timeout(struct timed_output_dev *sdev,
					int timeout)
{
	struct pm822_vibrator_info *info;
	info = container_of(sdev, struct pm822_vibrator_info,
				vibrator_timed_dev);
	pr_debug("Vibrator: Set duration: %dms\n", timeout);

	if (timeout <= 0) {
		pm822_control_vibrator(info, VIBRA_OFF_VALUE);
		del_timer(&info->vibrate_timer);
	} else {
		if (info->min_timeout)
			timeout = (timeout < info->min_timeout) ?
				   info->min_timeout : timeout;

		pm822_control_vibrator(info, VIBRA_ON_VALUE);
		mod_timer(&info->vibrate_timer,
			  jiffies + msecs_to_jiffies(timeout));
	}

	return;
}

static int vibrator_get_remaining_time(struct timed_output_dev *sdev)
{
	struct pm822_vibrator_info *info;
	int rettime;
	info = container_of(sdev, struct pm822_vibrator_info,
			vibrator_timed_dev);
	rettime = jiffies_to_msecs(jiffies - info->vibrate_timer.expires);
	pr_debug("Vibrator: Current duration: %dms\n", rettime);
	return rettime;
}

static int vibrator_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pm822_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np = pdev->dev.of_node;
	struct pm822_vibrator_info *info;

	info = kzalloc(sizeof(struct pm822_vibrator_info),
			GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->map = chip->regmap;
	info->vibr_power = regulator_get(&pdev->dev, "marvell,power");
	if (IS_ERR(info->vibr_power))
		info->vibr_power = NULL;

	/*get vibrator platdata from dts file*/
	if ((np != NULL) && (info != NULL))
		get_vibrator_platdata(np, info);

	info->vib_dev = &pdev->dev;

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

	INIT_WORK(&info->vibrator_off_work, vibrator_off_worker);
	mutex_init(&info->vib_mutex);
	info->enable = 0;

	init_timer(&info->vibrate_timer);
	info->vibrate_timer.function = on_vibrate_timer_expired;
	info->vibrate_timer.data = (unsigned long)info;

	platform_set_drvdata(pdev, info);

	return 0;
}

static int vibrator_remove(struct platform_device *pdev)
{
	struct pm822_vibrator_info *info;
	info = platform_get_drvdata(pdev);
	timed_output_dev_unregister(&info->vibrator_timed_dev);
	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe = vibrator_probe,
	.remove = vibrator_remove,
	.driver = {
		   .name = "88pm822-vibrator",
		   .owner = THIS_MODULE,
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
