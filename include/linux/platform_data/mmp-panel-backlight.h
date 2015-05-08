/* inclue/linux/platform_data/mmp-panel-generic.h
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 * http://www.samsung.com/
 * Header file for Samsung Display Backlight(LCD) driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MMP_PANEL_BACKLIGHT_H
#define _MMP_PANEL_BACKLIGHT_H
#include <video/mmp_disp.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/of.h>

enum {
	BL_CTRL_PWM,
	BL_CTRL_EASY_SCALE,
	BL_CTRL_STEP_CTRL,
	MIPI_CTRL,
};

enum {
	BRT_VALUE_OFF = 0,
	BRT_VALUE_DIM,
	BRT_VALUE_MIN,
	BRT_VALUE_DEF,
	BRT_VALUE_MAX,
	MAX_BRT_VALUE_IDX,
};

struct brt_value {
	int brightness;	/* brightness level from user */
	int tune_level;	/* tuning value be sent */
};

struct brt_range {
	struct brt_value off;
	struct brt_value dim;
	struct brt_value min;
	struct brt_value def;
	struct brt_value max;
};

struct mmp_panel_backlight_ops {
	int (*set_brightness)(struct mmp_panel *, int level);
	int (*get_brightness)(struct mmp_panel *);
};

struct mmp_panel_backlight_info {
	const char *name;
	bool enable;
	struct mmp_panel *panel;
	struct mutex ops_lock;
	const struct mmp_panel_backlight_ops *ops;
	struct brt_value range[MAX_BRT_VALUE_IDX];
	int current_brightness;
	int prev_tune_level;
};

extern int mmp_panel_attach_backlight(struct mmp_panel *,
		const struct mmp_panel_backlight_ops *);
extern void mmp_panel_detach_backlight(struct mmp_panel *);

#ifdef CONFIG_OF
static inline struct device_node *
mmp_panel_find_dt_backlight(struct platform_device *pdev, const char *pname)
{
	struct device_node *backlight_node;
	struct device *dev = &pdev->dev;

	backlight_node = of_parse_phandle(pdev->dev.of_node, pname, 0);
	if (!backlight_node) {
		dev_err(dev, "%s: backlight_node not found\n", __func__);
		return NULL;
	}

	return backlight_node;
}
#else
static inline struct device_node *
mmp_panel_find_dt_backlight(struct platform_device *pdev, const char *pname)
{
	return NULL;
}
#endif

static inline int
mmp_panel_backlight_enable(struct backlight_device *bd)
{
#ifdef CONFIG_PM_RUNTIME
	if (bd && bd->dev.parent)
		return pm_runtime_get_sync(bd->dev.parent);
#endif
	return 1;
}

static inline int
mmp_panel_backlight_disable(struct backlight_device *bd)
{
#ifdef CONFIG_PM_RUNTIME
	if (bd && bd->dev.parent)
		return pm_runtime_put_sync(bd->dev.parent);
#endif
	return 1;
}

static inline int
mmp_panel_backlight_is_on(struct backlight_device *bd)
{
#ifdef CONFIG_PM_RUNTIME
	if (bd && bd->dev.parent)
		return atomic_read(&bd->dev.parent->power.usage_count);
#endif
	return 0;
}

static inline int
mmp_panel_backlight_onoff(struct backlight_device *bd, int on)
{
	int ret;

	if (on)
		ret = mmp_panel_backlight_enable(bd);
	else
		ret = mmp_panel_backlight_disable(bd);

	if (ret)
		pr_err("%s: error enter %s\n",
				__func__, on ? "resume" : "suspend");

	return ret;
}

#endif	/* _MMP_PANEL_BACKLIGHT_H */

