/* drivers/video/mmp/panel/mipi-sc7798a.c
 *
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>
#include "mipi-sc7798a-param.h"
#include <linux/gpio.h>
#include <linux/platform_data/panel-sc7798a.h>

/* temporarily add */
#define ID_SC7798A_CPT  (0x5544F0)
#define ID_SC7798A_BOE  (0x55B8F0)

struct sc7798a {
	struct device *dev;
	struct class *lcd_class;
	struct sc7798a_panel_data  *pdata;
	u32 (*set_panel_id)(struct mmp_panel *panel);
	/* Further fields can be added here */
	int lcd_rst_n;
	int lcd_en_gpio1;
	int lcd_en_gpio2;
	u32 lcd_iovdd_type;
	u32 lcd_avdd_type;
	bool ldo_supply;
	bool regulator_supply;
};

static u32 boot_panel_id;

static int __init get_boot_panel_id(char *str)
{
	char *endp;

	boot_panel_id = memparse(str, &endp);
	pr_info("%s: boot_panel_id = 0x%8x\n", __func__, boot_panel_id);

	return 1;
}
early_param("lcd_id", get_boot_panel_id);

static u32 sc7798a_set_panel_id(struct mmp_panel *panel)
{
	u32 read_id = 0;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	struct mmp_dsi_buf *dbuf;

	dbuf = kmalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (dbuf) {
		mmp_phy_dsi_rx_cmd_array(path->phy, dbuf, read_id_cmds,
				ARRAY_SIZE(read_id_cmds));
		mmp_phy_dsi_rx_cmd_array(path->phy, dbuf, read_id1_cmds,
				ARRAY_SIZE(read_id1_cmds));
		read_id |= dbuf->data[0] << 16;
		mmp_phy_dsi_rx_cmd_array(path->phy, dbuf, read_id2_cmds,
				ARRAY_SIZE(read_id2_cmds));
		read_id |= dbuf->data[0] << 8;
		mmp_phy_dsi_rx_cmd_array(path->phy, dbuf, read_id3_cmds,
				ARRAY_SIZE(read_id3_cmds));
		read_id |= dbuf->data[0];
		kfree(dbuf);
		pr_info("Panel id is 0x%x\n", read_id);
	} else {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
	}

	return read_id;
}

static int sc7798a_panel_enable(struct mmp_path *path)
{
	struct sc7798a *plat = path->panel->plat_data;
	u32 panel_id;

	if (IS_ENABLED(CONFIG_OF))
		panel_id = path->panel->id;
	else if (plat->pdata)
		panel_id = plat->pdata->panel_id;

	switch (panel_id) {
	case ID_SC7798A_CPT:
		mmp_phy_dsi_tx_cmd_array(path->phy, sc7798a_cpt_init_cmds,
				ARRAY_SIZE(sc7798a_cpt_init_cmds));
		break;
	case ID_SC7798A_BOE:
	default:
		mmp_phy_dsi_tx_cmd_array(path->phy, sc7798a_boe_init_cmds,
				ARRAY_SIZE(sc7798a_boe_init_cmds));
		break;
	}
	mmp_phy_dsi_tx_cmd_array(path->phy, sc7798a_video_power_on_cmds,
			ARRAY_SIZE(sc7798a_video_power_on_cmds));

	return 0;
}

static int sc7798a_panel_disable(struct mmp_path *path)
{
	mmp_phy_dsi_tx_cmd_array(path->phy, sc7798a_video_power_off_cmds,
			ARRAY_SIZE(sc7798a_video_power_off_cmds));

	return 0;
}

#ifdef CONFIG_OF
static void sc7798a_panel_power(struct mmp_panel *panel,
		int skip_on, int on)
{
	static struct regulator *lcd_iovdd;
	static struct regulator *lcd_avdd;
	int err;
	struct sc7798a *plat = panel->plat_data;
	int lcd_rst_n = plat->lcd_rst_n;

	if (plat->regulator_supply) {
		if (!lcd_iovdd) {
			lcd_iovdd = regulator_get(panel->dev, "iovdd");
			if (IS_ERR_OR_NULL(lcd_iovdd)) {
				pr_err("%s: regulatori(iovdd- 1.8v) get error!\n",
						__func__);
				goto err_lcd_iovdd_type;
			}
		}

		/* Regulator V_LCD_3.0V */
		if (!lcd_avdd) {
			lcd_avdd = regulator_get(panel->dev, "avdd");
			if (IS_ERR_OR_NULL(lcd_avdd)) {
				pr_err("%s regulator(avdd - 3.0v) get error!\n",
						__func__);
				goto err_lcd_avdd_type;
			}
		}

	}

	if (on) {
		if (plat->regulator_supply) {
			err = regulator_enable(lcd_avdd);
			if (unlikely(err < 0)) {
				pr_err("%s: lcd_avdd regulator enable failed %d\n",
						__func__, err);
				goto err_lcd_avdd_enable_failed;
			}
			usleep_range(5000, 10000);

			err = regulator_enable(lcd_iovdd);
			if (unlikely(err < 0)) {
				pr_err("%s: lcd_iovdd regulator enable failed %d\n",
						__func__, err);
				goto err_lcd_iovdd_enable_failed;
			}
			msleep(20);
		}

		if (plat->ldo_supply) {
			if (plat->lcd_en_gpio1)
				gpio_direction_output(plat->lcd_en_gpio1, 1);

			if ((plat->lcd_en_gpio2) &&
				(plat->lcd_en_gpio1 != plat->lcd_en_gpio2))
				gpio_direction_output(plat->lcd_en_gpio2, 1);
		}

		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 1);
			usleep_range(20, 30);
			gpio_direction_output(lcd_rst_n, 0);
			usleep_range(2000, 3000);
			gpio_direction_output(lcd_rst_n, 1);
			msleep(21);
		}

	} else {
		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);

		if (plat->regulator_supply) {
			/* disable V_LCD_1.8V */
			regulator_disable(lcd_iovdd);
			usleep_range(10000, 11000);

			/* disable V_LCD_3.0V */
			regulator_disable(lcd_avdd);
		}

		if (plat->ldo_supply) {
			if ((plat->lcd_en_gpio2) &&
				(plat->lcd_en_gpio1 != plat->lcd_en_gpio2))
				gpio_direction_output(plat->lcd_en_gpio2, 0);

			gpio_direction_output(plat->lcd_en_gpio1, 0);
		}
	}

	return;

err_lcd_iovdd_enable_failed:
	if (plat->regulator_supply)
		regulator_disable(lcd_avdd);

err_lcd_avdd_enable_failed:
	if (plat->regulator_supply)
		regulator_put(lcd_avdd);

err_lcd_avdd_type:
	if (plat->regulator_supply) {
		lcd_avdd = NULL;
		regulator_put(lcd_iovdd);
	}

err_lcd_iovdd_type:
	if (plat->regulator_supply)
		lcd_iovdd = NULL;

	return;
}
#else
static void sc7798a_panel_power(struct mmp_panel *panel, int on) {}
#endif

#ifdef CONFIG_OF
static void of_sc7798a_set_panel_id(struct mmp_panel *panel)
{
	struct sc7798a *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	path->panel->id = (boot_panel_id) ? boot_panel_id :
					plat->set_panel_id(panel);
	if (!(path->panel->id))
		pr_err("%s: panel id configuration is missing\n",
			__func__);
}
#else
static void of_sc7798a_set_panel_id(struct mmp_panel *panel) {}
#endif

static void sc7798a_onoff(struct mmp_panel *panel, int status)
{
	struct sc7798a *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	pr_info("called %s with status %d\n", __func__, status);

	if (status) {
		/* power on */
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(1);
		else
			sc7798a_panel_power(panel, 0, 1);

		if ((path && path->panel && !path->panel->id) ||
				(plat->pdata && !plat->pdata->panel_id &&
				plat->set_panel_id)) {
			if (plat->pdata) {
				plat->pdata->panel_id = (boot_panel_id) ?
				boot_panel_id : plat->set_panel_id(panel);
			} else
				of_sc7798a_set_panel_id(panel);
		}

		sc7798a_panel_enable(path);

	} else {
		/* power off */
		sc7798a_panel_disable(path);
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(0);
		else
			sc7798a_panel_power(panel, 0, 0);
	}
}

static void sc7798a_reduced_onoff(struct mmp_panel *panel, int status)
{
	struct sc7798a *plat = panel->plat_data;
	if (status) {
		/* power on */
		sc7798a_panel_power(panel, 1, 1);
		if (plat->pdata) {
			if (boot_panel_id)
				plat->pdata->panel_id = boot_panel_id ?
					boot_panel_id :
					plat->set_panel_id(panel);
		} else
			of_sc7798a_set_panel_id(panel);
	} else {
		/* power off */
		sc7798a_panel_power(panel, 1, 0);
	}
}

static struct mmp_mode mmp_modes_sc7798a[] = {
	[0] = {
		.pixclock_freq = 34104000,
		.refresh = 60,
#if defined(CONFIG_MMP_VIRTUAL_RESOLUTION)
		.xres = CONFIG_MMP_VIRTUAL_RESOLUTION_X,
		.yres = CONFIG_MMP_VIRTUAL_RESOLUTION_Y,
#else
		.xres = 480,
		.yres = 800,
#endif
		.real_xres = 480,
		.real_yres = 800,
		.hsync_len = 60,
		.left_margin = 70,
		.right_margin = 90,
		.vsync_len = 4,
		.upper_margin = 12,
		.lower_margin = 8,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_RGB888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
	},
};

static int sc7798a_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_modes_sc7798a;
	return 1;
}

static struct mmp_panel panel_sc7798a = {
	.name = "sc7798a",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = sc7798a_get_modelist,
	.set_onoff = sc7798a_onoff,
	.reduced_onoff = sc7798a_reduced_onoff,
};

static ssize_t sc7798a_panelName_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "SC7798A");
}

static DEVICE_ATTR(sc7798a_panelName, S_IRUGO | S_IXOTH,
		sc7798a_panelName_show, NULL);

static ssize_t lcd_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct mmp_path *path = mmp_get_path(panel_sc7798a.plat_path_name);
	u32 panel_id;

#ifdef CONFIG_OF
	panel_id = path->panel->id;
#else
	struct sc7798a *lcd = dev_get_drvdata(dev);
	u32 panel_id = lcd->pdata->panel_id;
#endif

	return sprintf(buf, "NT_%x%x%x",
			(panel_id >> 16) & 0xFF,
			(panel_id >> 8) & 0xFF,
			panel_id  & 0xFF);
}

static DEVICE_ATTR(lcd_type, S_IRUGO | S_IXOTH,
		lcd_type_show, NULL);

static int sc7798a_probe(struct platform_device *pdev)
{

	struct sc7798a *lcd;
	int ret;

	const char *path_name;
	struct device_node *np = pdev->dev.of_node;

	pr_debug("called %s\n", __func__);

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (unlikely(!lcd))
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np,
				"marvell,path-name", &path_name);
		if (unlikely(ret < 0)) {
			pr_err("%s: Path name not found\n", __func__);
			goto err_no_platform_data;
		}

		panel_sc7798a.plat_path_name = path_name;

	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
			goto err_no_platform_data;
		}

		lcd->pdata = pdev->dev.platform_data;
		panel_sc7798a.plat_path_name = lcd->pdata->mi->plat_path_name;
	}

	panel_sc7798a.plat_data = lcd;
	lcd->set_panel_id = &sc7798a_set_panel_id;
	panel_sc7798a.dev = &pdev->dev;

	lcd->lcd_rst_n = of_get_named_gpio(np, "rst-gpio", 0);
	if (unlikely(lcd->lcd_rst_n < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
				lcd->lcd_rst_n);
		ret = -EINVAL;
		goto err_no_platform_data;
	}

	ret = of_property_read_u32(np, "iovdd-supply-type",
			&lcd->lcd_iovdd_type);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to read property iovdd-supply-type\n",
				__func__);
		goto err_no_platform_data;
	}

	ret = of_property_read_u32(np, "avdd-supply-type", &lcd->lcd_avdd_type);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to read property avdd-supply-type\n",
				__func__);
		goto err_no_platform_data;
	}

	ret = gpio_request(lcd->lcd_rst_n, "lcd reset gpio");
	if (unlikely(ret < 0)) {
		pr_err("%s: gpio_request %d failed: %d\n",
				__func__, lcd->lcd_rst_n, ret);
		goto err_no_platform_data;
	}

	/*
	 * regulator_supply: PMIC Regulator based supply to LCD.
	 * ldo_supply: gpio controlled LDO based supply to LCD.
	 */
	lcd->ldo_supply = false;
	lcd->regulator_supply = false;
	if (!lcd->lcd_iovdd_type && !lcd->lcd_avdd_type)
		lcd->ldo_supply = true;
	else
		lcd->regulator_supply = true;

	if (lcd->ldo_supply) {
		lcd->lcd_en_gpio1 = of_get_named_gpio(np, "ldo_en_gpio1", 0);
		if (unlikely(lcd->lcd_en_gpio1 < 0)) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
					lcd->lcd_en_gpio1);
			ret = -EINVAL;
			goto err_ldo1_gpio_failed;
		}
		ret = gpio_request(lcd->lcd_en_gpio1, "lcd_ldo_en1");
		if (unlikely(ret < 0)) {
			pr_err("%s: gpio_request %d failed: %d\n",
				__func__, lcd->lcd_en_gpio1, ret);
			goto err_ldo1_gpio_failed;
		}

		lcd->lcd_en_gpio2 = of_get_named_gpio(np, "ldo_en_gpio2", 0);
		if (unlikely(lcd->lcd_en_gpio2 < 0)) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
					lcd->lcd_en_gpio2);
			ret = -EINVAL;
			goto err_ldo2_gpio_failed;
		}

		if (lcd->lcd_en_gpio1 != lcd->lcd_en_gpio2) {
			ret = gpio_request(lcd->lcd_en_gpio2, "lcd_ldo_en2");
			if (unlikely(ret < 0)) {
				pr_err("%s: gpio_request %d failed: %d\n",
					__func__, lcd->lcd_en_gpio2, ret);
				goto err_ldo2_gpio_failed;
			}
		}
	}

	lcd->lcd_class = class_create(THIS_MODULE, "lcd");
	if (IS_ERR(lcd->lcd_class)) {
		ret = PTR_ERR(lcd->lcd_class);
		pr_err("Failed to create lcd_class!");
		goto err_class_create;
	}

	lcd->dev = device_create(lcd->lcd_class, NULL, 0, "%s", "panel");
	if (IS_ERR(lcd->dev)) {
		ret = PTR_ERR(lcd->dev);
		pr_err("Failed to create device(panel)!\n");
		goto err_device_create;
	}

	dev_set_drvdata(lcd->dev, lcd);

	ret = device_create_file(lcd->dev, &dev_attr_sc7798a_panelName);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_sc7798a_panelName.attr.name);
		goto err_lcd_device;
	}

	ret = device_create_file(lcd->dev, &dev_attr_lcd_type);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_lcd_type.attr.name);
		goto err_lcd_name;
	}

	mmp_register_panel(&panel_sc7798a);
	return 0;

err_lcd_name:
	device_remove_file(lcd->dev, &dev_attr_sc7798a_panelName);
err_lcd_device:
	device_destroy(lcd->lcd_class, 0);
err_device_create:
	class_destroy(lcd->lcd_class);
err_class_create:
	if (!lcd->lcd_avdd_type) {
		if ((lcd->lcd_en_gpio2) &&
			(lcd->lcd_en_gpio1 != lcd->lcd_en_gpio2))
			gpio_free(lcd->lcd_en_gpio2);
	}
err_ldo2_gpio_failed:
if (!lcd->lcd_iovdd_type)
	gpio_free(lcd->lcd_en_gpio1);
err_ldo1_gpio_failed:
	gpio_free(lcd->lcd_rst_n);
err_no_platform_data:
	kfree(lcd);

	return ret;

}

static int sc7798a_remove(struct platform_device *dev)
{
	struct sc7798a *lcd = panel_sc7798a.plat_data;

	if (lcd->ldo_supply) {
		if ((lcd->lcd_en_gpio2) &&
			(lcd->lcd_en_gpio1 != lcd->lcd_en_gpio2))
			gpio_free(lcd->lcd_en_gpio2);

		gpio_free(lcd->lcd_en_gpio1);
	}

	gpio_free(lcd->lcd_rst_n);

	device_remove_file(lcd->dev, &dev_attr_sc7798a_panelName);
	device_remove_file(lcd->dev, &dev_attr_lcd_type);
	device_destroy(lcd->lcd_class, 0);
	class_destroy(lcd->lcd_class);

	mmp_unregister_panel(&panel_sc7798a);
	kfree(lcd);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_sc7798a_dt_match[] __initconst = {
	{ .compatible = "marvell,mmp-sc7798a" },
	{},
};
#endif
static struct platform_driver sc7798a_driver = {
	.driver     = {
		.name   = "mmp-sc7798a",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mmp_sc7798a_dt_match),
#endif
	},
	.probe      = sc7798a_probe,
	.remove     = sc7798a_remove,
};

static int sc7798a_module_init(void)
{
	return platform_driver_register(&sc7798a_driver);
}
static void sc7798a_module_exit(void)
{
	platform_driver_unregister(&sc7798a_driver);
}
module_init(sc7798a_module_init);
module_exit(sc7798a_module_exit);

MODULE_DESCRIPTION("Panel driver for MIPI panel SC7798A");
MODULE_LICENSE("GPL");
