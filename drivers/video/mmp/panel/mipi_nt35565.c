/*
 * drivers/video/mmp/panel/mipi_nt35565.c
 * active panel using DSI interface to do init
 *
 * Copyright (C) 2013 Marvell Technology Group Ltd.
 * Authors:  Yu Xu <yuxu@marvell.com>
 *          Guoqing Li <ligq@marvell.com>
 *          Lisa Du <cldu@marvell.com>
 *          Zhou Zhu <zzhu3@marvell.com>
 *          Jing Xiang <jxiang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>

#define NT35565_SLEEP_OUT_DELAY 200
#define NT35565_DISP_ON_DELAY	0

struct nt35565_plat_data {
	struct mmp_panel *panel;
	void (*plat_onoff)(int status);
	void (*plat_set_backlight)(struct mmp_panel *panel, int level);
};

static char exit_sleep[] = {0x11};
static char display_on[] = {0x29};

static struct mmp_dsi_cmd_desc nt35565_display_on_cmds[] = {
	{MIPI_DSI_DCS_SHORT_WRITE, 0, NT35565_SLEEP_OUT_DELAY,
		sizeof(exit_sleep), exit_sleep},
	{MIPI_DSI_DCS_SHORT_WRITE, 0, NT35565_DISP_ON_DELAY,
		sizeof(display_on), display_on},
};

static char enter_sleep[] = {0x10};
static char display_off[] = {0x28};

static struct mmp_dsi_cmd_desc nt35565_display_off_cmds[] = {
	{MIPI_DSI_DCS_SHORT_WRITE, 0, NT35565_SLEEP_OUT_DELAY,
		sizeof(enter_sleep), enter_sleep},
	{MIPI_DSI_DCS_SHORT_WRITE, 0, NT35565_DISP_ON_DELAY,
		sizeof(display_off), display_off},
};

static void nt35565_panel_onoff(struct mmp_path *path, int on)
{
	if (on) {
		mmp_dphy_ulps_set_on(path->phy, 0);
		mmp_phy_dsi_tx_cmd_array(path->phy, nt35565_display_on_cmds,
			ARRAY_SIZE(nt35565_display_on_cmds));
	} else {
		mmp_phy_dsi_tx_cmd_array(path->phy, nt35565_display_off_cmds,
			ARRAY_SIZE(nt35565_display_off_cmds));
		mmp_dphy_ulps_set_on(path->phy, 1);
	}
}

#ifdef CONFIG_OF
static void nt35565_panel_power(struct mmp_panel *panel, int skip_on, int on)
{
	static struct regulator *lcd_iovdd;
	static struct regulator *lcd_avdd;
	int lcd_rst_n, err;

	lcd_rst_n = of_get_named_gpio(panel->dev->of_node, "rst_gpio", 0);
	if (lcd_rst_n < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
		       lcd_rst_n);
		return;
	}

	err = gpio_request(lcd_rst_n, "lcd reset gpio");
	if (err) {
		pr_err("%s: gpio_request failed: %d\n", __func__, err);
		return;
	}

	if (!lcd_iovdd) {
		lcd_iovdd = regulator_get(panel->dev, "iovdd");
		if (IS_ERR_OR_NULL(lcd_iovdd)) {
			pr_err("%s regulator get error!\n", __func__);
			goto regu_lcd_iovdd;
		}
	}

	if (!lcd_avdd) {
		lcd_avdd = regulator_get(panel->dev, "avdd");
		if (IS_ERR(lcd_avdd)) {
			pr_err("%s regulator get error!\n", __func__);
			goto regu_lcd_avdd;
		}
	}

	if (on) {
		regulator_set_voltage(lcd_avdd, 3100000, 3100000);
		err = regulator_enable(lcd_avdd);
		if (err < 0) {
			pr_err("%s: lcd_avdd(3.1v) enalbe failed!\n",
				__func__);
			goto out;
		}
		usleep_range(5000, 5500);

		regulator_set_voltage(lcd_iovdd, 1800000, 1800000);
		err = regulator_enable(lcd_iovdd);
		if (err < 0) {
			pr_err("%s: lcd_iovdd(1.8v) enalbe failed!\n",
				__func__);
			goto out;
		}
		usleep_range(15000, 15500);
		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(20, 40);
			gpio_direction_output(lcd_rst_n, 0);
			usleep_range(50, 100);
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(15000, 15500);
		}
	} else {
		/* disable LCD_AVDD 3.1v */
		regulator_disable(lcd_avdd);

		/* disable LCD_IOVDD 1.8v */
		regulator_disable(lcd_iovdd);

		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);
	}

out:
	gpio_free(lcd_rst_n);
	if (err < 0) {
		lcd_avdd = NULL;
		lcd_iovdd = NULL;
	 }
	pr_debug("%s on %d\n", __func__, on);
	return;

regu_lcd_avdd:
	lcd_avdd = NULL;
	regulator_put(lcd_iovdd);

regu_lcd_iovdd:
	lcd_iovdd = NULL;
	gpio_free(lcd_rst_n);
}

static DEFINE_SPINLOCK(bl_lock);
static void nt35565_panel_set_bl(struct mmp_panel *panel, int intensity)
{
	int gpio_bl, bl_level, p_num;
	unsigned long flags;
	/*
	 * FIXME
	 * the initial value of bl_level_last is the
	 * uboot backlight level, it should be aligned.
	 */
	static int bl_level_last = 17;

	gpio_bl = of_get_named_gpio(panel->dev->of_node, "bl_gpio", 0);
	if (gpio_bl < 0) {
		pr_err("%s: of_get_named_gpio failed\n", __func__);
		return;
	}

	if (gpio_request(gpio_bl, "lcd backlight")) {
		pr_err("gpio %d request failed\n", gpio_bl);
		return;
	}

	/*
	 * Brightness is controlled by a series of pulses
	 * generated by gpio. It has 32 leves and level 1
	 * is the brightest. Pull low for 3ms makes
	 * backlight shutdown
	 */
	bl_level = (100 - intensity) * 32 / 100 + 1;

	if (bl_level == bl_level_last)
		goto set_bl_return;

	if (bl_level == 33) {
		/* shutdown backlight */
		gpio_direction_output(gpio_bl, 0);
		goto set_bl_return;
	}

	if (bl_level > bl_level_last)
		p_num = bl_level - bl_level_last;
	else
		p_num = bl_level + 32 - bl_level_last;

	while (p_num--) {
		spin_lock_irqsave(&bl_lock, flags);
		gpio_direction_output(gpio_bl, 0);
		udelay(1);
		gpio_direction_output(gpio_bl, 1);
		spin_unlock_irqrestore(&bl_lock, flags);
		udelay(1);
	}

set_bl_return:
	if (bl_level == 33)
		bl_level_last = 0;
	else
		bl_level_last = bl_level;
	gpio_free(gpio_bl);
	pr_debug("%s, intensity:%d\n", __func__, intensity);
}
#else
static void nt35565_panel_power(struct mmp_panel *panel, int on) {}
#endif

static void nt35565_onoff(struct mmp_panel *panel, int status)
{
	struct nt35565_plat_data *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	BUG_ON(!path);
	if (status) {
		/* power on */
		if (plat->plat_onoff)
			plat->plat_onoff(1);
		else
			nt35565_panel_power(panel, 0, 1);
		nt35565_panel_onoff(path, 1);
	} else {
		nt35565_panel_onoff(path, 0);
		/* power off */
		if (plat->plat_onoff)
			plat->plat_onoff(0);
		else
			nt35565_panel_power(panel, 0, 0);
	}
}

static void nt35565_reduced_onoff(struct mmp_panel *panel, int status)
{
	if (status) {
		/* power on */
		nt35565_panel_power(panel, 1, 1);
	} else {
		/* power off */
		nt35565_panel_power(panel, 1, 0);
	}
}

static int nt35565_ls_bl_update_status(struct backlight_device *bl)
{
	struct nt35565_plat_data *data = dev_get_drvdata(&bl->dev);
	struct mmp_panel *panel = data->panel;
	int level;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	/* If there is backlight function of board, use it */
	if (data && data->plat_set_backlight) {
		data->plat_set_backlight(panel, level);
		return 0;
	}

	if (panel && panel->set_brightness)
		panel->set_brightness(panel, level);

	return 0;
}

static int nt35565_ls_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops nt35565_ls_bl_ops = {
	.get_brightness = nt35565_ls_bl_get_brightness,
	.update_status  = nt35565_ls_bl_update_status,
};


static struct mmp_mode mmp_modes_nt35565[] = {
	[0] = {
		.pixclock_freq = 34421160,
		.refresh = 57,
#if defined(CONFIG_MMP_VIRTUAL_RESOLUTION)
		.xres = CONFIG_MMP_VIRTUAL_RESOLUTION_X,
		.yres = CONFIG_MMP_VIRTUAL_RESOLUTION_Y,
#else
		.xres = 540,
		.yres = 960,
#endif
		.real_xres = 540,
		.real_yres = 960,
		.hsync_len = 2,
		.left_margin = 10,
		.right_margin = 68,
		.vsync_len = 2,
		.upper_margin = 6,
		.lower_margin = 6,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_RGB888PACK,
		.hsync_invert = FB_SYNC_HOR_HIGH_ACT,
		.vsync_invert = FB_SYNC_VERT_HIGH_ACT,
		.height = 95,
		.width = 53,
	},
};

static int nt35565_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_modes_nt35565;
	return 1;
}

static struct mmp_panel panel_nt35565 = {
	.name = "nt35565",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = nt35565_get_modelist,
	.set_onoff = nt35565_onoff,
	.reduced_onoff = nt35565_reduced_onoff,
};

static int nt35565_probe(struct platform_device *pdev)
{
	struct mmp_mach_panel_info *mi;
	struct nt35565_plat_data *plat_data;
	struct device_node *np = pdev->dev.of_node;
	const char *path_name;
	struct backlight_properties props;
	struct backlight_device *bl;
	int ret;

	plat_data = kzalloc(sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	panel_nt35565.plat_data = plat_data;

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np, "marvell,path-name", &path_name);
		if (ret < 0)
			return ret;
		panel_nt35565.plat_path_name = path_name;

		if (of_get_named_gpio(np, "bl_gpio", 0) < 0)
			pr_debug("%s: get bl_gpio failed\n", __func__);
		else
			plat_data->plat_set_backlight = nt35565_panel_set_bl;
	} else {
		/* get configs from platform data */
		mi = pdev->dev.platform_data;
		if (mi == NULL) {
			dev_err(&pdev->dev, "no platform data defined\n");
			return -EINVAL;
		}
		plat_data->plat_onoff = mi->plat_set_onoff;
		panel_nt35565.plat_path_name = mi->plat_path_name;
		plat_data->plat_set_backlight = mi->plat_set_backlight;
	}

	panel_nt35565.plat_data = plat_data;
	panel_nt35565.dev = &pdev->dev;
	plat_data->panel = &panel_nt35565;
	mmp_register_panel(&panel_nt35565);

	/*
	 * if no panel or plat associate backlight control,
	 * don't register backlight device here.
	 */
	if (!panel_nt35565.set_brightness && !plat_data->plat_set_backlight)
		return 0;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 100;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register("lcd-bl", &pdev->dev, plat_data,
			&nt35565_ls_bl_ops, &props);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		dev_err(&pdev->dev, "failed to register lcd-backlight\n");
		return ret;
	}

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = 100;
	ret = nt35565_ls_bl_update_status(bl);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to set lcd brightness\n");

	return 0;
}

static int nt35565_remove(struct platform_device *dev)
{
	mmp_unregister_panel(&panel_nt35565);
	kfree(panel_nt35565.plat_data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_nt35565_dt_match[] = {
	{ .compatible = "marvell,mmp-nt35565" },
	{},
};
#endif

static struct platform_driver nt35565_driver = {
	.driver		= {
		.name	= "mmp-nt35565",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mmp_nt35565_dt_match),
	},
	.probe		= nt35565_probe,
	.remove		= nt35565_remove,
};

static int nt35565_init(void)
{
	return platform_driver_register(&nt35565_driver);
}
static void nt35565_exit(void)
{
	platform_driver_unregister(&nt35565_driver);
}
module_init(nt35565_init);
module_exit(nt35565_exit);

MODULE_AUTHOR("Yu Xu <yuxu@marvell.com>");
MODULE_DESCRIPTION("Panel driver for MIPI panel NT35565");
MODULE_LICENSE("GPL");
