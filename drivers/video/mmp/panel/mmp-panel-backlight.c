#include <linux/module.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/platform_data/mmp-panel-backlight.h>

int mmp_panel_attach_backlight(struct mmp_panel *panel,
		const struct mmp_panel_backlight_ops *ops)
{
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(panel->bd);

	if (!bl_info) 
	{               
		pr_err("%s, no platform data\n", __func__);              
		return -EINVAL;      
	}

	bl_info->panel = panel;
	bl_info->ops = ops;
	backlight_update_status(panel->bd);

	return 0;
}
EXPORT_SYMBOL(mmp_panel_attach_backlight);

void mmp_panel_detach_backlight(struct mmp_panel *panel)
{
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(panel->bd);

	if (!bl_info) 
	{               
		pr_err("%s, no platform data\n", __func__);              
		return;      
	}

	mutex_lock(&bl_info->ops_lock);
	bl_info->ops = NULL;
	mutex_unlock(&bl_info->ops_lock);
	return;
}
EXPORT_SYMBOL(mmp_panel_detach_backlight);

int mmp_panel_get_tune_level(struct mmp_panel_backlight_info *bl_info,
		int brightness)
{
	int tune_level = 0;
	struct brt_value *range = bl_info->range;

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

static int mmp_panel_backlight_set_brightness(
		struct mmp_panel_backlight_info *bl_info, int brightness)
{
	int tune_level;
	tune_level = mmp_panel_get_tune_level(bl_info, brightness);
	if (unlikely(tune_level < 0)) {
		pr_err("%s, failed to find tune_level. (%d)\n",
				__func__, brightness);
		return -EINVAL;
	}
	pr_info("%s: brightness(%d), tune_level(%d)\n",
			__func__, brightness, tune_level);
	mutex_lock(&bl_info->ops_lock);
	if (bl_info->panel && bl_info->ops &&
			bl_info->ops->set_brightness)
		bl_info->ops->set_brightness(bl_info->panel, tune_level);
	mutex_unlock(&bl_info->ops_lock);

	return tune_level;
}

static int mmp_panel_backlight_update_status(struct backlight_device *bd)
{
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (unlikely(!bl_info)) {
		pr_err("%s, no platform data\n", __func__);
		return 0;
	}

	if (bd->props.power != FB_BLANK_UNBLANK ||
		bd->props.fb_blank != FB_BLANK_UNBLANK ||
		!bl_info->enable)
		brightness = 0;

	mmp_panel_backlight_set_brightness(bl_info, brightness);

	return 0;
}

static int mmp_panel_backlight_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static const struct backlight_ops mmp_panel_backlight_ops = {
	.update_status	= mmp_panel_backlight_update_status,
	.get_brightness	= mmp_panel_backlight_get_brightness,
};

static int mmp_panel_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bd;
	struct backlight_properties props;
	struct mmp_panel_backlight_info *bl_info;
	int ret;

	pr_info("called %s\n", __func__);

	bl_info = devm_kzalloc(&pdev->dev,
			sizeof(*bl_info), GFP_KERNEL);
	if (unlikely(!bl_info))
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		struct device_node *np = pdev->dev.of_node;
		int arr[MAX_BRT_VALUE_IDX * 2], i;

		ret = of_property_read_string(np,
				"marvell,panel-backlight-name",
				&bl_info->name);
		ret = of_property_read_u32_array(np,
				"marvell,panel-backlight-brt-range",
				arr, MAX_BRT_VALUE_IDX * 2);
		for (i = 0; i < MAX_BRT_VALUE_IDX; i++) {
			bl_info->range[i].brightness = arr[i * 2];
			bl_info->range[i].tune_level = arr[i * 2 + 1];
		}

		pr_info("backlight device : %s\n", bl_info->name);
		pr_info("[BRT_VALUE_OFF] brightness(%d), tune_level(%d)\n",
				bl_info->range[BRT_VALUE_OFF].brightness,
				bl_info->range[BRT_VALUE_OFF].tune_level);
		pr_info("[BRT_VALUE_DIM] brightness(%d), tune_level(%d)\n",
				bl_info->range[BRT_VALUE_DIM].brightness,
				bl_info->range[BRT_VALUE_DIM].tune_level);
		pr_info("[BRT_VALUE_MIN] brightness(%d), tune_level(%d)\n",
				bl_info->range[BRT_VALUE_MIN].brightness,
				bl_info->range[BRT_VALUE_MIN].tune_level);
		pr_info("[BRT_VALUE_DEF] brightness(%d), tune_level(%d)\n",
				bl_info->range[BRT_VALUE_DEF].brightness,
				bl_info->range[BRT_VALUE_DEF].tune_level);
		pr_info("[BRT_VALUE_MAX] brightness(%d), tune_level(%d)\n",
				bl_info->range[BRT_VALUE_MAX].brightness,
				bl_info->range[BRT_VALUE_MAX].tune_level);
	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
		}
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = bl_info->range[BRT_VALUE_MAX].brightness;
	props.brightness = (u8)bl_info->range[BRT_VALUE_DEF].brightness;
	bl_info->current_brightness =
		(u8)bl_info->range[BRT_VALUE_DEF].brightness;

	bd = backlight_device_register(bl_info->name, &pdev->dev, bl_info,
			&mmp_panel_backlight_ops, &props);
	if (IS_ERR(bd)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bd);
	}

	mutex_init(&bl_info->ops_lock);
	bl_info->enable = true;
	pm_runtime_enable(&pdev->dev);
	platform_set_drvdata(pdev, bd);
	pm_runtime_get_sync(&pdev->dev);

	return 0;

}

static int mmp_panel_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return 0;
	}

	mmp_panel_backlight_set_brightness(bl_info, 0);
	pm_runtime_disable(&pdev->dev);
	backlight_device_unregister(bd);

	return 0;
}

void mmp_panel_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return;
	}
	mmp_panel_backlight_set_brightness(bl_info, 0);
	pm_runtime_disable(&pdev->dev);
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_panel_backlight_dt_match[] __initconst = {
	{ .compatible = "marvell,mmp-panel-backlight" },
	{},
};
#endif

#if defined(CONFIG_PM_RUNTIME) || defined(CONFIG_PM_SLEEP)
static int mmp_panel_backlight_runtime_suspend(struct device *dev)
{
	struct backlight_device *bd = dev_get_drvdata(dev);
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return -EINVAL;
	}

	bl_info->enable = false;
	backlight_update_status(bd);

	dev_info(dev, "mmp_panel_backlight suspended\n");
	return 0;
}

static int mmp_panel_backlight_runtime_resume(struct device *dev)
{
	struct backlight_device *bd = dev_get_drvdata(dev);
	struct mmp_panel_backlight_info *bl_info =
		(struct mmp_panel_backlight_info *)bl_get_data(bd);

	if (!bl_info) {
		pr_err("%s, no platform data\n", __func__);
		return -EINVAL;
	}

	bl_info->enable = true;
	backlight_update_status(bd);

	dev_info(dev, "mmp_panel_backlight resumed.\n");
	return 0;
}
#endif

const struct dev_pm_ops mmp_panel_backlight_pm_ops = {
	SET_RUNTIME_PM_OPS(mmp_panel_backlight_runtime_suspend,
		mmp_panel_backlight_runtime_resume, NULL)
};

static struct platform_driver mmp_panel_backlight_driver = {
	.driver		= {
		.name	= "mmp-panel-backlight",
		.owner	= THIS_MODULE,
		.pm	= &mmp_panel_backlight_pm_ops,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(mmp_panel_backlight_dt_match),
#endif
	},
	.probe		= mmp_panel_backlight_probe,
	.remove		= mmp_panel_backlight_remove,
	.shutdown       = mmp_panel_backlight_shutdown,
};

static int __init mmp_panel_backlight_init(void)
{
	return platform_driver_register(&mmp_panel_backlight_driver);
}
module_init(mmp_panel_backlight_init);

static void __exit mmp_panel_backlight_exit(void)
{
	platform_driver_unregister(&mmp_panel_backlight_driver);
}
module_exit(mmp_panel_backlight_exit);

MODULE_DESCRIPTION("MMP PANEL GENERIC BACKLIGHT Driver");
MODULE_LICENSE("GPL");
