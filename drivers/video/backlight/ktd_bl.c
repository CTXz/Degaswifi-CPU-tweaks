/*
 * linux/drivers/video/backlight/ktd_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ktd_bl.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#define __BACKLIGHT_DEBUG__  0

#define BACKLIGHT_DEV_NAME	"panel"
#define MAX_BRIGHTNESS_IN_BLU	(32)

#if defined(CONFIG_BACKLIGHT_KTD253)
#define CTRL_T_EN_US	(55)	/* CTRL Start-up Time */
#elif defined(CONFIG_BACKLIGHT_KTD3102)
#define KTD_USE_PWM_EN
#define CTRL_T_EN_US	(500)	/* CTRL Start-up Time */
#else
#define CTRL_T_EN_US	(55)	/* CTRL Start-up Time */
#endif
#define CTRL_T_HIGH_US	(1)	/* CTRL High Pulse Width */
#define CTRL_T_LOW_US	(1)	/* CTRL Low Pulse Width */
#define CTRL_T_OFF_MS	(3)	/* CTRL Shutdown Pulse Width */
#define PWM_T_OFF_MS	(100)	/* PWM Shutdown Pulse Width */
#define VOUT_T_OFF_MS	(100)	/* Vout Off Stable Time */

static DEFINE_MUTEX(bl_ctrl_mutex);
static DEFINE_SPINLOCK(bl_ctrl_lock);

static inline int get_tune_level(struct brt_value *range,
		int brightness)
{
	int tune_level = 0;

	if (unlikely(!range)) {
		pr_err("%s: brightness range not exist!\n", __func__);
		return -EINVAL;
	}

	if (brightness > range[BRT_VALUE_MAX].brightness ||
			brightness < 0) {
		pr_err("%s: out of range (%d)\n", __func__, brightness);
		return -EINVAL;
	}

	if (brightness <= range[BRT_VALUE_OFF].brightness) {
		tune_level = range[BRT_VALUE_OFF].tune_level;
	} else if (brightness <= range[BRT_VALUE_DIM].brightness) {
		tune_level = range[BRT_VALUE_DIM].tune_level;
	} else if (brightness <= range[BRT_VALUE_MIN].brightness) {
		tune_level = range[BRT_VALUE_MIN].tune_level;
	} else if (brightness <= range[BRT_VALUE_DEF].brightness) {
			tune_level = range[BRT_VALUE_DEF].tune_level;
			tune_level -= (range[BRT_VALUE_DEF].brightness -
					brightness) *
				(range[BRT_VALUE_DEF].tune_level -
				 range[BRT_VALUE_MIN].tune_level) /
				(range[BRT_VALUE_DEF].brightness -
				 range[BRT_VALUE_MIN].brightness);
	} else if (brightness <= range[BRT_VALUE_MAX].brightness) {
			tune_level = range[BRT_VALUE_MAX].tune_level;
			tune_level -= (range[BRT_VALUE_MAX].brightness -
					brightness) *
				(range[BRT_VALUE_MAX].tune_level -
				 range[BRT_VALUE_DEF].tune_level) /
				(range[BRT_VALUE_MAX].brightness -
				 range[BRT_VALUE_DEF].brightness);
	}

	return tune_level;
}

static int ktd_backlight_set_brightness(struct ktd_bl_info *bl_info, int level)
{
	int pulse;
	int tune_level;
	unsigned long flags;

	unsigned int nr_brt_level = bl_info->sz_table;
	int gpio_bl_ctrl = bl_info->gpio_bl_ctrl;
	int gpio_bl_pwm_en = bl_info->gpio_bl_pwm_en;

	mutex_lock(&bl_ctrl_mutex);
	if (unlikely(level < 0 || level > 255)) {
		pr_err("%s, warning - out of range (level : %d)\n",
				__func__, level);
		goto out;
	}

	tune_level = get_tune_level(bl_info->range, level);
	if (unlikely(tune_level < 0)) {
		pr_err("%s, failed to find tune_level. (level : %d)\n",
				__func__, level);
		goto out;
	}

	if (bl_info->current_brightness == level) {
		pr_info("set_brightnesss : same level(%d) ignored\n", level);
		goto out;
	}

	pr_info("set_brightness : level(%d) tune (%d)\n", level, tune_level);
	bl_info->current_brightness = level;

	if (!tune_level) {
		if (gpio_bl_pwm_en >= 0) {
			gpio_set_value(gpio_bl_pwm_en, 0);
			/* wait until vout gets stable */
			msleep(PWM_T_OFF_MS);
		}
		gpio_set_value(gpio_bl_ctrl, 0);
		msleep(VOUT_T_OFF_MS);
		bl_info->prev_tune_level = 0;
		goto out;
	}

	if (unlikely(bl_info->prev_tune_level < 0)) {
		gpio_set_value(gpio_bl_ctrl, 0);
		msleep(CTRL_T_OFF_MS);
		bl_info->prev_tune_level = 0;
		pr_info("LCD Baklight init in boot time on kernel\n");
	}

	if (!(bl_info->prev_tune_level)) {
		gpio_set_value(gpio_bl_ctrl, 1);
		udelay(CTRL_T_EN_US);
		bl_info->prev_tune_level = 1;
	}

	pulse = (tune_level - bl_info->prev_tune_level + MAX_BRIGHTNESS_IN_BLU)
		% MAX_BRIGHTNESS_IN_BLU;

	for (; pulse > 0; pulse--) {
		spin_lock_irqsave(&bl_ctrl_lock, flags);
		gpio_set_value(gpio_bl_ctrl, 0);
		udelay(CTRL_T_LOW_US);
		gpio_set_value(gpio_bl_ctrl, 1);
		spin_unlock_irqrestore(&bl_ctrl_lock, flags);
		udelay(CTRL_T_HIGH_US);
	}

	usleep_range(900, 1000);
	if ((gpio_bl_pwm_en >= 0) &&
			(!gpio_get_value(gpio_bl_pwm_en))) {
		gpio_set_value(gpio_bl_pwm_en, 1);
		if (!gpio_get_value(gpio_bl_pwm_en))
			pr_err("%s, gpio_bl_pwm_en is LOW\n", __func__);
		usleep_range(10, 20);
	}

	bl_info->prev_tune_level = tune_level;

out:
	mutex_unlock(&bl_ctrl_mutex);
	return 0;
}

static int ktd_backlight_update_status(struct backlight_device *bl)
{
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl_get_data(bl);
	int brightness = bl->props.brightness;

	if (unlikely(!bl_info)) {
		pr_err("%s, no platform data\n", __func__);
		return 0;
	}

	if (bl->props.power != FB_BLANK_UNBLANK ||
		bl->props.fb_blank != FB_BLANK_UNBLANK ||
		!bl_info->enable)
		brightness = 0;

	ktd_backlight_set_brightness(bl_info, brightness);

	return 0;
}

static int ktd_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops ktd_backlight_ops = {
	.update_status	= ktd_backlight_update_status,
	.get_brightness	= ktd_backlight_get_brightness,
};

static int ktd_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct backlight_properties props;
	struct ktd_bl_info *bl_info;
	unsigned int max, def, min;
	int ret;

	pr_debug("called %s\n", __func__);

	bl_info = devm_kzalloc(&pdev->dev,
			sizeof(*bl_info), GFP_KERNEL);
	if (unlikely(!bl_info))
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		struct device_node *np = pdev->dev.of_node;
		int arr[MAX_BRT_VALUE_IDX * 2], i;

		bl_info->gpio_bl_ctrl =
			of_get_named_gpio(np, "ktd,ctrl-gpio", 0);
		if (unlikely(bl_info->gpio_bl_ctrl < 0)) {
			pr_err("%s: of_get_named_gpio ktd,ctrl-gpio failed: %d\n",
					__func__, bl_info->gpio_bl_ctrl);
			ret = -EINVAL;
			goto err_no_platform_data;
		}

		bl_info->gpio_bl_pwm_en =
			of_get_named_gpio(np, "ktd,pwm-en-gpio", 0);
		if (unlikely(bl_info->gpio_bl_pwm_en < 0)) {
			pr_err("%s: of_get_named_gpio ktd,pwm-en-gpio failed: %d\n",
					__func__, bl_info->gpio_bl_pwm_en);
		}

		ret = of_property_read_u32_array(np,
				"ktd,backlight-brt-range",
				arr, MAX_BRT_VALUE_IDX * 2);
		if (unlikely(ret < 0)) {
			pr_err("%s: invalid brightness table : %d\n",
					__func__, ret);
			goto err_no_platform_data;
		}

		for (i = 0; i < MAX_BRT_VALUE_IDX; i++) {
			bl_info->range[i].brightness = arr[i * 2];
			bl_info->range[i].tune_level = arr[i * 2 + 1];
		}
		min = bl_info->range[BRT_VALUE_MIN].brightness;
		def = bl_info->range[BRT_VALUE_DEF].brightness;
		max = bl_info->range[BRT_VALUE_MAX].brightness;
	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
			goto err_no_platform_data;
		}
		bl_info = (struct ktd_bl_info *)pdev->dev.platform_data;
	}

	pr_debug("%s: ctrl-gpio : %d\n", __func__,
				bl_info->gpio_bl_ctrl);
	pr_debug("%s: pwm-en-gpio : %d\n", __func__,
				bl_info->gpio_bl_pwm_en);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = max;

	bd = backlight_device_register(BACKLIGHT_DEV_NAME, &pdev->dev, bl_info,
			&ktd_backlight_ops, &props);
	if (IS_ERR(bd)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bd);
		goto err_no_platform_data;
	}

	bd->props.brightness = (u8)def;
	bl_info->current_brightness = def;
	bl_info->prev_tune_level =
		get_tune_level(bl_info->range, def);
	if (bl_info->gpio_bl_ctrl >= 0) {
		ret = gpio_request(bl_info->gpio_bl_ctrl, "BL_CTRL");
		if (unlikely(ret < 0)) {
			pr_err("request gpio(%d) failed\n",
					bl_info->gpio_bl_ctrl);
			goto err_bl_gpio_request;
		}
		gpio_direction_output(bl_info->gpio_bl_ctrl, 1);
	}

	if (bl_info->gpio_bl_pwm_en >= 0) {
		ret = gpio_request(bl_info->gpio_bl_pwm_en, "BL_PWM_EN");
		if (unlikely(ret < 0)) {
			pr_err("request gpio(%d) failed\n",
					bl_info->gpio_bl_pwm_en);
			goto err_bl_pwm_gpio_request;
		}
		gpio_direction_output(bl_info->gpio_bl_pwm_en, 1);
	}
	bl_info->enable = true;
	pm_runtime_enable(&pdev->dev);
	platform_set_drvdata(pdev, bd);
	pm_runtime_get_sync(&pdev->dev);

	return 0;

err_bl_pwm_gpio_request:
	if (bl_info->gpio_bl_ctrl >= 0)
		gpio_free(bl_info->gpio_bl_ctrl);
err_bl_gpio_request:
	backlight_device_unregister(bd);
err_pm_runtime:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_brightness_table:
err_no_platform_data:
	devm_kfree(&pdev->dev, bl_info);

	return ret;
}

static int ktd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return 0;
	}

	if (bl_info->gpio_bl_ctrl >= 0)
		gpio_free(bl_info->gpio_bl_ctrl);
	if (bl_info->gpio_bl_pwm_en >= 0)
		gpio_free(bl_info->gpio_bl_pwm_en);

	ktd_backlight_set_brightness(bl_info, 0);
	backlight_device_unregister(bd);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

void ktd_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return;
	}

	ktd_backlight_set_brightness(bl_info, 0);
	pm_runtime_disable(&pdev->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id ktd_backlight_dt_match[] __initconst = {
	{ .compatible = "kinetic,backlight-ktd253" },
	{ .compatible = "kinetic,backlight-ktd3102" },
	{},
};
#endif

#if defined(CONFIG_PM_RUNTIME) || defined(CONFIG_PM_SLEEP)
static int ktd_backlight_runtime_suspend(struct device *dev)
{
	struct backlight_device *bd = dev_get_drvdata(dev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return -EINVAL;
	}

	bl_info->enable = false;
	backlight_update_status(bd);

	dev_info(dev, "ktd_backlight suspended\n");
	return 0;
}

static int ktd_backlight_runtime_resume(struct device *dev)
{
	struct backlight_device *bd = dev_get_drvdata(dev);
	struct ktd_bl_info *bl_info =
		(struct ktd_bl_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return -EINVAL;
	}

	bl_info->enable = true;
	backlight_update_status(bd);

	dev_info(dev, "ktd_backlight resumed.\n");
	return 0;
}
#endif

const struct dev_pm_ops ktd_backlight_pm_ops = {
	SET_RUNTIME_PM_OPS(ktd_backlight_runtime_suspend,
		ktd_backlight_runtime_resume, NULL)
};

static struct platform_driver ktd_backlight_driver = {
	.driver		= {
		.name	= BACKLIGHT_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &ktd_backlight_pm_ops,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(ktd_backlight_dt_match),
#endif
	},
	.probe		= ktd_backlight_probe,
	.remove		= ktd_backlight_remove,
	.shutdown       = ktd_backlight_shutdown,
};

static int __init ktd_backlight_init(void)
{
	return platform_driver_register(&ktd_backlight_driver);
}
module_init(ktd_backlight_init);

static void __exit ktd_backlight_exit(void)
{
	platform_driver_unregister(&ktd_backlight_driver);
}
module_exit(ktd_backlight_exit);

MODULE_DESCRIPTION("KTD based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ktd-backlight");
