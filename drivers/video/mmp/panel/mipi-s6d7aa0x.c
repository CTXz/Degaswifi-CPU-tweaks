/* drivers/video/mmp/panel/mipi-s6d7aa0x.c
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
#include <asm/uaccess.h>
#include <video/mmp_disp.h>
#include <video/mipi_display.h>
#include "mipi-s6d7aa0x-param.h"
#include <linux/gpio.h>
#include <linux/platform_data/panel-s6d7aa0x.h>

struct s6d7aa0x {

	struct device *dev;
	struct class *lcd_class;
	struct class *lcd_mDNIe_class;
	struct s6d7aa0x_panel_data  *pdata;
	struct backlight_device *bd;
	u32 (*set_panel_id)(struct mmp_panel *panel);
	/* Further fields can be added here */
	int lcd_rst_n;
	int lcd_bl_n;
	int last_bl;
	int lcd_1v8_vcc;
	int lcd_3v3_vdd;
	u32 lcd_vcc_type;
	u32 lcd_vdd_type;
	bool ldo_supply;
	bool regulator_supply;
};

static u32 boot_panel_id;

#ifdef CONFIG_LDI_SUPPORT_MDNIE
static int isReadyTo_mDNIe = 1;
static struct mdnie_config mDNIe_cfg;
static void s6d7aa0x_set_mdnie(struct mmp_path *path, int scenario);
#ifdef ENABLE_MDNIE_TUNING
#define TUNING_FILE_PATH "/sdcard/mdnie/"
#define MAX_TUNING_FILE_NAME 100
#define NR_MDNIE_DATA 119
#define NR_MDNIE_DATA_E7	8
#define NR_MDNIE_DATA_E8	17
#define NR_MDNIE_DATA_E9	25
#define NR_MDNIE_DATA_EA	25
#define NR_MDNIE_DATA_EB	25
#define NR_MDNIE_DATA_EC	19
static int tuning_enable;
static char tuning_filename[MAX_TUNING_FILE_NAME];

static unsigned char mDNIe_data[NR_MDNIE_DATA];
static unsigned char mDNIe_data_E7[NR_MDNIE_DATA_E7];
static unsigned char mDNIe_data_E8[NR_MDNIE_DATA_E8];
static unsigned char mDNIe_data_E9[NR_MDNIE_DATA_E9];
static unsigned char mDNIe_data_EA[NR_MDNIE_DATA_EA];
static unsigned char mDNIe_data_EB[NR_MDNIE_DATA_EB];
static unsigned char mDNIe_data_EC[NR_MDNIE_DATA_EC];

static struct mmp_dsi_cmd_desc s6d7aa0x_video_display_mDNIe_cmds[] = {
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_E7), mDNIe_data_E7},
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_E8), mDNIe_data_E8},
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_E9), mDNIe_data_E9},
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_EA), mDNIe_data_EA},
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_EB), mDNIe_data_EB},
	{MIPI_DSI_DCS_LONG_WRITE, 0, 0, sizeof(mDNIe_data_EC), mDNIe_data_EC},
};

static int degas_parse_array(char *set_mdnie_data);
static void s6d7aa0x_set_mdnie_tuning(struct mmp_path *path);
static void set_mDNIe_Mode(struct mdnie_config *mDNIeCfg);
#endif /* ENABLE_MDNIE_TUNING */
#endif /* CONFIG_LDI_SUPPORT_MDNIE */


static int __init get_boot_panel_id(char *str)
{
	char *endp;

	boot_panel_id = memparse(str, &endp);
	pr_info("%s: boot_panel_id = 0x%8x\n", __func__, boot_panel_id);

	return 1;
}
early_param("lcd_id", get_boot_panel_id);

static u32 s6d7aa0x_set_panel_id(struct mmp_panel *panel)
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
	} else
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);

	return read_id;
}


static int s6d7aa0x_panel_init(struct mmp_path *path)
{
	pr_info("%s, init s6d7aa0x\n", __func__);
	mmp_phy_dsi_tx_cmd_array(path->phy,
			s6d7aa0x_power_on_init,
			ARRAY_SIZE(s6d7aa0x_power_on_init));

#ifdef CONFIG_LDI_SUPPORT_MDNIE
	isReadyTo_mDNIe = 1;
	set_mDNIe_Mode(&mDNIe_cfg);
#endif /* CONFIG_LDI_SUPPORT_MDNIE */

	return 0;
}


static int s6d7aa0x_panel_enable(struct mmp_path *path)
{
	pr_info("%s, enable s6d7aa0x\n", __func__);
	mmp_phy_dsi_tx_cmd_array(path->phy,
			s6d7aa0x_backlight_brightness_cmds,
			ARRAY_SIZE(s6d7aa0x_backlight_brightness_cmds));

	return 0;
}

static int s6d7aa0x_panel_disable(struct mmp_path *path)
{
	pr_info("%s\n", __func__);
	mmp_phy_dsi_tx_cmd_array(path->phy, s6d7aa0x_video_display_off_cmds,
				ARRAY_SIZE(s6d7aa0x_video_display_off_cmds));
#ifdef CONFIG_LDI_SUPPORT_MDNIE
	isReadyTo_mDNIe = 0;
#endif

	return 0;
}


#ifdef CONFIG_OF
static void s6d7aa0x_panel_power(struct mmp_panel *panel, int skip_on, int on)
{
	static struct regulator *lcd_iovdd;
	static struct regulator *lcd_avdd;
	struct s6d7aa0x *plat = panel->plat_data;
	int lcd_rst_n = plat->lcd_rst_n, err;

	if (plat->regulator_supply) {
		if (!lcd_iovdd) {
			lcd_iovdd = regulator_get(panel->dev, "iovdd");
			if (IS_ERR_OR_NULL(lcd_iovdd)) {
				pr_err("%s: regulatori(iovdd- 1.8v) get error!\n",
						__func__);
				goto err_lcd_vcc_type;
			}
		}
		/* Regulator V_LCD_3.0V */
		if (!lcd_avdd) {
			lcd_avdd = regulator_get(panel->dev, "avdd");
			if (IS_ERR_OR_NULL(lcd_avdd)) {
				pr_err("%s regulator(avdd - 3.0v) get error!\n",
						__func__);
				goto err_lcd_vdd_type;
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
			mdelay(5);

			err = regulator_enable(lcd_iovdd);
			if (unlikely(err < 0)) {
				pr_err("%s: lcd_iovdd regulator enable failed %d\n",
						__func__, err);
				goto err_lcd_iovdd_enable_failed;
			}
			mdelay(15);
		}

		if (plat->ldo_supply) {
			if (plat->lcd_1v8_vcc)
				gpio_direction_output(plat->lcd_1v8_vcc, 1);
			if ((plat->lcd_3v3_vdd) &&
				(plat->lcd_1v8_vcc != plat->lcd_3v3_vdd))
				gpio_direction_output(plat->lcd_3v3_vdd, 1);
		}
		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 1);
			udelay(20);
			gpio_direction_output(lcd_rst_n, 0);
			udelay(2000);
			gpio_direction_output(lcd_rst_n, 1);
			mdelay(5);
		}

	} else {
		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);

		if (plat->regulator_supply) {
			/* disable V_LCD_1.8V */
			regulator_disable(lcd_iovdd);
			mdelay(10);

			/* disable V_LCD_3.0V */
			regulator_disable(lcd_avdd);
		}

		 if (plat->ldo_supply) {
			gpio_direction_output(plat->lcd_1v8_vcc, 0);

			if (plat->lcd_1v8_vcc != plat->lcd_3v3_vdd)
				gpio_direction_output(plat->lcd_3v3_vdd, 0);
		}
	}
	return;

err_lcd_iovdd_enable_failed:
	if (plat->regulator_supply)
		regulator_disable(lcd_avdd);

err_lcd_avdd_enable_failed:
	if (plat->regulator_supply)
		regulator_put(lcd_avdd);

err_lcd_vdd_type:
	if (plat->regulator_supply) {
		lcd_avdd = NULL;
		regulator_put(lcd_iovdd);
	}

err_lcd_vcc_type:
	if (plat->regulator_supply)
		lcd_iovdd = NULL;

	return;
}
#else
static void s6d7aa0x_panel_power(struct mmp_panel *panel, int on) {}
#endif

#ifdef CONFIG_OF
static void of_s6d7aa0x_set_panel_id(struct mmp_panel *panel)
{
	struct s6d7aa0x *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	path->panel->id = (boot_panel_id) ? boot_panel_id :
					plat->set_panel_id(panel);
	if (!(path->panel->id))
		pr_err("%s: panel id configuration is missing\n",
			__func__);
}
#else
static void of_s6d7aa0x_set_panel_id(struct mmp_panel *panel) {}
#endif

static void s6d7aa0x_onoff(struct mmp_panel *panel, int status)
{
	struct s6d7aa0x *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	pr_info("called %s with status %d\n", __func__, status);

	if (status) {
		/* power on */
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(1);
		else
			s6d7aa0x_panel_power(panel, 0, 1);

		if (((path) && (path->panel) && !(path->panel->id)) ||
			((plat->pdata) && !(plat->pdata->panel_id) &&
					(plat->set_panel_id))) {
			if (plat->pdata) {
				plat->pdata->panel_id = (boot_panel_id) ?
					boot_panel_id : plat->set_panel_id(panel);
			}
		 else
			of_s6d7aa0x_set_panel_id(panel);
		}

		s6d7aa0x_panel_init(path);
		s6d7aa0x_panel_enable(path);

	} else {
		/* power off */
		s6d7aa0x_panel_disable(path);
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(0);
		else
			s6d7aa0x_panel_power(panel, 0, 0);
	}
}

static void s6d7aa0x_reduced_onoff(struct mmp_panel *panel, int status)
{
	struct s6d7aa0x *plat = panel->plat_data;
	if (status) {
		/* power on */
		s6d7aa0x_panel_power(panel, 1, 1);
		if (plat->pdata) {
			if (boot_panel_id)
				plat->pdata->panel_id = boot_panel_id ?
					boot_panel_id :
					plat->set_panel_id(panel);
		} else
			of_s6d7aa0x_set_panel_id(panel);
	} else {
		/* power off */
		s6d7aa0x_panel_power(panel, 1, 0);
	}
}

static struct mmp_mode mmp_modes_s6d7aa0x[] = {
	[0] = {
		.pixclock_freq = 68428800,
		.refresh = 60,
#if defined(CONFIG_MMP_VIRTUAL_RESOLUTION)
		.xres = CONFIG_MMP_VIRTUAL_RESOLUTION_X,
		.yres = CONFIG_MMP_VIRTUAL_RESOLUTION_Y,
#else
		.xres = 800,
		.yres = 1280,
#endif
		.real_xres = 800,
		.real_yres = 1280,
		.hsync_len = 16,		/*HSW*/
		.left_margin = 48,		/*HBP*/
		.right_margin = 16,		/*HFP*/
		.vsync_len = 2,			/*VSW*/
		.upper_margin = 6,		/*VBP*/
		.lower_margin = 8,		/*VFP*/
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_RGB888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
	},
};

static int s6d7aa0x_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_modes_s6d7aa0x;
	return 1;
}

static struct mmp_panel panel_s6d7aa0x = {
	.name = "s6d7aa0x",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = s6d7aa0x_get_modelist,
	.set_onoff = s6d7aa0x_onoff,
	.reduced_onoff = s6d7aa0x_reduced_onoff,
};

static int s6d7aa0x_get_brightness(struct backlight_device *bd)
{
	struct s6d7aa0x *lcd = bl_get_data(bd);

	return lcd->last_bl;
}


#define PLATFORM_MAX_BRIGHTNESS	255
#define PLATFORM_MID_BRIGHTNESS	143
#define PLATFORM_LOW_BRIGHTNESS	10

static int adjust_brightness_range(int low, int mid, int max, int brightness)
{
	int ret = 0;

	if (brightness > PLATFORM_MAX_BRIGHTNESS)
		ret = PLATFORM_MAX_BRIGHTNESS;
	if (brightness >= PLATFORM_MID_BRIGHTNESS) {
		ret = (brightness - PLATFORM_MID_BRIGHTNESS) * (max - mid) /
			(PLATFORM_MAX_BRIGHTNESS - PLATFORM_MID_BRIGHTNESS) + mid;
	} else if (brightness >= PLATFORM_LOW_BRIGHTNESS) {
		ret = (brightness - PLATFORM_LOW_BRIGHTNESS) * (mid - low) /
			(PLATFORM_MID_BRIGHTNESS - PLATFORM_LOW_BRIGHTNESS) + low;
	} else
		ret = 0;

	return ret;
}

#define READ_BRIGHTNESS_52H	0
static int s6d7aa0x_set_brightness(struct backlight_device *bd)
{
	struct mmp_path *path = mmp_get_path(panel_s6d7aa0x.plat_path_name);
	struct s6d7aa0x *lcd = bl_get_data(bd);
	int brightness, bl_level = bd->props.brightness;

#if READ_BRIGHTNESS_52H
	struct mmp_dsi_buf *dbuf;
	dbuf = kmalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (!dbuf) {
		printk("%s: can't alloc dsi rx buffer\n", __func__);
	}
#endif

	if (bl_level == lcd->last_bl)
		return 0;

	brightness = adjust_brightness_range(2, 87, 174, bl_level);

	pr_info(KERN_INFO "%s, brightness = %d platform = %d\n",
			__func__, brightness, bl_level);

	s6d7aa0x_backlight_brightness_cmds[0].data[1] = (u8)brightness;

	mmp_phy_dsi_tx_cmd_array(path->phy, s6d7aa0x_backlight_brightness_cmds,
			ARRAY_SIZE(s6d7aa0x_backlight_brightness_cmds));

#if READ_BRIGHTNESS_52H
	printk(KERN_INFO "%s, before Read 52h : %d\n", __func__, dbuf->data[0]);
	mmp_phy_dsi_rx_cmd_array(path->phy, dbuf, read_52h_cmds,
			ARRAY_SIZE(read_52h_cmds));
	printk(KERN_INFO "%s, Read 52h : %d\n", __func__, dbuf->data[0]);
	kfree(dbuf);
#endif

	lcd->last_bl = bl_level;
	return 0;
}

static const struct backlight_ops s6d7aa0x_backlight_ops = {

	.get_brightness = s6d7aa0x_get_brightness,
	.update_status = s6d7aa0x_set_brightness,
};

static ssize_t lcd_panel_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "S6D7AA0X");
}

static DEVICE_ATTR(lcd_panel_name, S_IRUGO | S_IRUSR | S_IRGRP | S_IXOTH,
			lcd_panel_name_show, NULL);

static ssize_t lcd_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct mmp_path *path = mmp_get_path(panel_s6d7aa0x.plat_path_name);
	u32 panel_id;
	int ret = 0;

#ifdef CONFIG_OF
	panel_id = path->panel->id;
#else
	struct s6d7aa0x *lcd = dev_get_drvdata(dev);
	panel_id = lcd->pdata->panel_id;
#endif

	switch (panel_id) {
		case 0x5eb810 :
			ret = sprintf(buf, "BOE_BP070WX1-300");
			break;
		default :
			ret = sprintf(buf, "NULL");
			break;
	}
	return ret;
}

static DEVICE_ATTR(lcd_type, S_IRUGO | S_IRUSR | S_IRGRP | S_IXOTH,
			lcd_type_show, NULL);

/* mDNIe Functions */
#ifdef CONFIG_LDI_SUPPORT_MDNIE

static void s6d7aa0x_set_mdnie(struct mmp_path *path, int scenario)
{
	int file_num = 0;

	if (scenario > SCENARIO_MAX) {
		printk(KERN_INFO "%s, exceeded max scenario(val:%d)\n",
				__func__, scenario);
		return;
	}
	mmp_phy_dsi_tx_cmd_array(path->phy,
			s6d7aa0x_video_display_enable_access_register,
			ARRAY_SIZE(s6d7aa0x_video_display_enable_access_register));

	file_num = degas_parse_array(s6d7aa0x_video_display_mDNIe_scenario_cmds[scenario].data);

	if (file_num != NR_MDNIE_DATA)
		printk(KERN_INFO "[ERROR] : degas_parse_array() failed! : %d\n",
				file_num);

	mmp_phy_dsi_tx_cmd_array(path->phy, s6d7aa0x_video_display_mDNIe_cmds,
			ARRAY_SIZE(s6d7aa0x_video_display_mDNIe_cmds));
	printk(KERN_INFO "%s, done! \n", __func__);
}

#ifdef ENABLE_MDNIE_TUNING
static void s6d7aa0x_set_mdnie_tuning(struct mmp_path *path)
{
	printk(KERN_INFO "%s, set mDNIe tuning data\n", __func__);
	mmp_phy_dsi_tx_cmd_array(path->phy,
			s6d7aa0x_video_display_enable_access_register,
			ARRAY_SIZE(s6d7aa0x_video_display_enable_access_register));
	mmp_phy_dsi_tx_cmd_array(path->phy, s6d7aa0x_video_display_mDNIe_cmds,
			ARRAY_SIZE(s6d7aa0x_video_display_mDNIe_cmds));
	printk(KERN_INFO "%s, done! \n", __func__);

}
#endif /* ENABLE_MDNIE_TUNING */
#endif /* CONFIG_LDI_SUPPORT_MDNIE */

#ifdef CONFIG_LDI_SUPPORT_MDNIE
#ifdef ENABLE_MDNIE_TUNING
static int parse_text(char *src, int len)
{
	int i;
	int data = 0, value = 0, count = 0, comment = 0;
	char *cur_position;

	cur_position = src;
	for (i = 0; i < len; i++, cur_position++) {
		char a = *cur_position;
		switch (a) {
		case '\r':
		case '\n':
			comment = 0;
			data = 0;
			break;
		case '/':
			comment++;
			data = 0;
			break;
		case '0'...'9':
			if (comment > 1)
				break;
			if (data == 0 && a == '0')
				data = 1;
			else if (data == 2) {
				data = 3;
				value = (a - '0') * 16;
			} else if (data == 3) {
				value += (a - '0');
				mDNIe_data[count] = value;
				printk(KERN_INFO "Tuning value[%d] = 0x%02X\n",
				       count, value);
				count++;
				data = 0;
			};
			break;
		case 'a'...'f':
		case 'A'...'F':
			if (comment > 1)
				break;
			if (data == 2) {
				data = 3;
				if (a < 'a')
					value = (a - 'A' + 10) * 16;
				else
					value = (a - 'a' + 10) * 16;
			} else if (data == 3) {
				if (a < 'a')
					value += (a - 'A' + 10);
				else
					value += (a - 'a' + 10);
				mDNIe_data[count] = value;
				printk(KERN_INFO "Tuning value[%d]=0x%02X\n",
				       count, value);
				count++;
				data = 0;
			};
			break;
		case 'x':
		case 'X':
			if (data == 1)
				data = 2;
			break;
		default:
			if (comment == 1)
				comment = 0;
			data = 0;
			break;
		}
	}

	return count;
}

static int degas_parse_array(char *set_mdnie_data)
{
	int i, k = 0;
	for (i = 0 ; i < NR_MDNIE_DATA_E7 ; i++)
		mDNIe_data_E7[i] = set_mdnie_data[k++];

	for (i = 0 ; i < NR_MDNIE_DATA_E8 ; i++)
		mDNIe_data_E8[i] = set_mdnie_data[k++];

	for (i = 0 ; i < NR_MDNIE_DATA_E9 ; i++)
		mDNIe_data_E9[i] = set_mdnie_data[k++];

	for (i = 0 ; i < NR_MDNIE_DATA_EA ; i++)
		mDNIe_data_EA[i] = set_mdnie_data[k++];

	for (i = 0 ; i < NR_MDNIE_DATA_EB ; i++)
		mDNIe_data_EB[i] = set_mdnie_data[k++];

	for (i = 0 ; i < NR_MDNIE_DATA_EC ; i++)
		mDNIe_data_EC[i] = set_mdnie_data[k++];

	printk(KERN_INFO "%s, is done! ,param count : %d", __func__, k);

	return k;
}

static int load_tuning_data(char *filename)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret, num, file_num;
	mm_segment_t fs;

	printk(KERN_INFO "[INFO]:%s called loading file name : [%s]\n",
	       __func__, filename);

	fs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		printk(KERN_ERR "[ERROR]:File open failed %d\n",
		       (int)IS_ERR(filp));
		return -1;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	printk(KERN_INFO "[INFO]: Loading File Size : %ld(bytes)", l);

	dp = kmalloc(l + 10, GFP_KERNEL);
	if (dp == NULL) {
		printk(KERN_ERR
		       "Can't not alloc memory for tuning file load\n");
		filp_close(filp, current->files);
		return -1;
	}
	pos = 0;
	memset(dp, 0, l);
	printk(KERN_INFO "[INFO] : before vfs_read()\n");
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	printk(KERN_INFO "[INFO] : after vfs_read()\n");

	if (ret != l) {
		printk(KERN_INFO "[ERROR] : vfs_read() filed ret : %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -1;
	}

	filp_close(filp, current->files);

	set_fs(fs);
	num = parse_text(dp, l);

	file_num = degas_parse_array(mDNIe_data);
	if (file_num != NR_MDNIE_DATA)
		printk(KERN_INFO "[ERROR] : degas_parse_array() failed! : %d\n",
				file_num);

	if (!num) {
		printk(KERN_INFO "[ERROR]:Nothing to parse\n");
		kfree(dp);
		return -1;
	}

	printk(KERN_INFO "[INFO] : Loading Tuning Value's Count : %d", num);

	kfree(dp);
	return num;
}

static ssize_t mDNIeTuning_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "Tunned File Name : %s\n", tuning_filename);

	return ret;
}

static ssize_t mDNIeTuning_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct mmp_path *path = mmp_get_path(panel_s6d7aa0x.plat_path_name);
	char *pt;
	char a = *buf;

	if (a == '1') {
		tuning_enable = 1;
		printk(KERN_INFO "%s:Tuning_enable\n", __func__);
	} else if (a == '0') {
		tuning_enable = 0;
		printk(KERN_INFO "%s:Tuning_disable\n", __func__);
	} else {
		memset(tuning_filename, 0, sizeof(tuning_filename));
		sprintf(tuning_filename, "%s%s", TUNING_FILE_PATH, buf);

		pt = tuning_filename;
		while (*pt) {
			if (*pt == '\r' || *pt == '\n') {
				*pt = 0;
				break;
			}
			pt++;
		}
		printk(KERN_INFO "%s:%s\n", __func__, tuning_filename);
		if (load_tuning_data(tuning_filename) <= 0) {
			printk(KERN_ERR "[ERROR]:load_tunig_data() failed\n");
			return size;
		}

		if (tuning_enable && mDNIe_data[0]) {
			printk(KERN_INFO
			       "========================mDNIe!!!!!!!\n");
			s6d7aa0x_set_mdnie_tuning(path);
		}
	}
	return size;
}
#endif /* ENABLE_MDNIE_TUNING */

static void set_mDNIe_Mode(struct mdnie_config *mDNIeCfg)
{
	struct mmp_path *path = mmp_get_path(panel_s6d7aa0x.plat_path_name);
	int value;

	if (!isReadyTo_mDNIe) {
		printk(KERN_INFO "%s, is not ready to mDNIE\n", __func__);
		return;
	}
#ifdef ENABLE_MDNIE_TUNING
	if (tuning_enable && mDNIe_data[0]) {
		s6d7aa0x_set_mdnie_tuning(path);
		return;
	}
#endif

	printk(KERN_INFO "%s:[mDNIe] negative=%d\n", __func__,
	       mDNIe_cfg.negative);
	msleep(100);
	if (mDNIe_cfg.negative) {
		printk(KERN_INFO "%s : apply negative color\n", __func__);
		s6d7aa0x_set_mdnie(path, SCENARIO_MAX);
		return;
	}

	switch (mDNIeCfg->scenario) {
		case UI_MODE:
			value = mDNIeCfg->scenario;
			break;

		case VIDEO_MODE:
			if (mDNIeCfg->outdoor == OUTDOOR_ON)
				value = SCENARIO_MAX + 1;
			else
				value = mDNIeCfg->scenario;
			break;

		case VIDEO_WARM_MODE:
			if (mDNIeCfg->outdoor == OUTDOOR_ON)
				value = SCENARIO_MAX + 2;
			else
				value = mDNIeCfg->scenario;

			break;

		case VIDEO_COLD_MODE:
			if (mDNIeCfg->outdoor == OUTDOOR_ON)
				value = SCENARIO_MAX + 3;
			else
				value = mDNIeCfg->scenario;

			break;

		case CAMERA_MODE:
			if (mDNIeCfg->outdoor == OUTDOOR_ON)
				value = SCENARIO_MAX + 4;
			else
				value = mDNIeCfg->scenario;

			break;

		case GALLERY_MODE:
			value = mDNIeCfg->scenario;
			break;

		case VT_MODE:
			value = mDNIeCfg->scenario;
			break;

		case BROWSER_MODE:
			value = mDNIeCfg->scenario;
			break;

		case EBOOK_MODE:
			value = mDNIeCfg->scenario;
			break;

		case EMAIL_MODE:
			value = mDNIeCfg->scenario;
			break;

		default:
			value = UI_MODE;
			break;
	};

	if (mDNIe_cfg.negative && value == UI_MODE)
		return;
	printk(KERN_INFO "%s:[mDNIe] value=%d ++ \n", __func__, value);
	s6d7aa0x_set_mdnie(path, value);
	return;
}

static ssize_t mDNIeScenario_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "mDNIeScenario_show : %d\n", mDNIe_cfg.scenario);
	return ret;
}

static ssize_t mDNIeScenario_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	unsigned int value;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	printk(KERN_INFO "%s:(1)value=%d\n", __func__, value);

	switch (value) {
	case UI_MODE:
		value = UI_MODE;
		break;
	case VIDEO_MODE:
		value = VIDEO_MODE;
		break;
	case CAMERA_MODE:
		value = CAMERA_MODE;
		break;
	case GALLERY_MODE:
		value = GALLERY_MODE;
		break;
	case VT_MODE:
		value = VT_MODE;
		break;
	case BROWSER_MODE:
		value = BROWSER_MODE;
		break;
	case EBOOK_MODE:
		value = EBOOK_MODE;
		break;
	case EMAIL_MODE:
		value = EMAIL_MODE;
		break;
	default:
		value = UI_MODE;
		break;
	};

	mDNIe_cfg.scenario = value;
	set_mDNIe_Mode(&mDNIe_cfg);
	return size;
}

static ssize_t mDNIeOutdoor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "mDNIeOutdoor_show : %d\n", mDNIe_cfg.outdoor);

	return ret;
}

static ssize_t mDNIeOutdoor_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	unsigned int value;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	printk(KERN_INFO "%s:value=%d\n", __func__, value);
	mDNIe_cfg.outdoor = value;
	set_mDNIe_Mode(&mDNIe_cfg);
	return size;
}

static ssize_t mDNIeNegative_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "mDNIeNegative_show : %d\n", mDNIe_cfg.negative);
	return ret;
}

static ssize_t mDNIeNegative_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	unsigned int value;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	printk(KERN_INFO "%s:value=%d\n", __func__, value);

	if (value == 1)
		mDNIe_cfg.negative = 1;
	else
		mDNIe_cfg.negative = 0;

	set_mDNIe_Mode(&mDNIe_cfg);
	return size;
}
#endif /* CONFIG_LDI_SUPPORT_MDNIE */
#ifdef CONFIG_LDI_SUPPORT_MDNIE
#ifdef ENABLE_MDNIE_TUNING
static DEVICE_ATTR(tuning, 0664, mDNIeTuning_show, mDNIeTuning_store);
#endif /* ENABLE_MDNIE_TUNING */
static DEVICE_ATTR(scenario, 0664, mDNIeScenario_show, mDNIeScenario_store);
static DEVICE_ATTR(outdoor, 0664, mDNIeOutdoor_show, mDNIeOutdoor_store);
static DEVICE_ATTR(negative, 0664, mDNIeNegative_show, mDNIeNegative_store);
#endif /* CONFIG_LDI_SUPPORT_MDNIE */

static int s6d7aa0x_probe(struct platform_device *pdev)
{

	struct s6d7aa0x *lcd;
	int ret;
	const char *path_name;
	struct device_node *np = pdev->dev.of_node;

	struct backlight_properties props = {
		.brightness = DEFAULT_BRIGHTNESS,
		.max_brightness = 255,
		.type = BACKLIGHT_RAW,
	};

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (unlikely(!lcd))
		return  -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np,
				"marvell,path-name", &path_name);
		if (unlikely(ret < 0)) {
			pr_err("%s: Path name not found\n", __func__);
			goto err_no_platform_data;
		}

		panel_s6d7aa0x.plat_path_name = path_name;

	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
			goto err_no_platform_data;
		}

		lcd->pdata = pdev->dev.platform_data;
		panel_s6d7aa0x.plat_path_name = lcd->pdata->mi->plat_path_name;
	}

	panel_s6d7aa0x.plat_data = lcd;
	lcd->set_panel_id = &s6d7aa0x_set_panel_id;
	panel_s6d7aa0x.dev = &pdev->dev;

	lcd->lcd_rst_n = of_get_named_gpio(np, "rst-gpio", 0);
	if (unlikely(lcd->lcd_rst_n < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
		       lcd->lcd_rst_n);
		ret = lcd->lcd_rst_n;
		goto err_no_platform_data;
	}
	ret = of_property_read_u32(np, "vcc-supply-type", &lcd->lcd_vcc_type);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to read property iovdd-supply-type\n",
				__func__);
		goto err_no_platform_data;
	}

	ret = of_property_read_u32(np, "vdd-supply-type", &lcd->lcd_vdd_type);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to read property avdd-supply-type\n",
				__func__);
		goto err_no_platform_data;
	}

	/*
	 * regulator_supply: PMIC Regulator based supply to LCD.
	 * ldo_supply: gpio controlled LDO based supply to LCD.
	 */
	lcd->ldo_supply = false;
	lcd->regulator_supply = false;
	if (!lcd->lcd_vcc_type && !lcd->lcd_vdd_type)
		lcd->ldo_supply = true;
	else
		lcd->regulator_supply = true;

	ret = gpio_request(lcd->lcd_rst_n, "lcd reset gpio");
	if (unlikely(ret < 0)) {
		pr_err("%s: gpio_request failed: %d\n", __func__, ret);
		goto err_no_platform_data;
	}

	if (lcd->ldo_supply) {
		lcd->lcd_1v8_vcc = of_get_named_gpio(np, "vcc-gpio", 0);
		if (lcd->lcd_1v8_vcc < 0) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
					lcd->lcd_1v8_vcc);
			ret = lcd->lcd_1v8_vcc;
			goto err_ldo1_gpio_failed;
		}
		ret = gpio_request(lcd->lcd_1v8_vcc, "vcc_1v8_lcd");
		if (unlikely(ret < 0)) {
			pr_err("%s: gpio_request failed: %d\n", __func__, ret);
			goto err_ldo1_gpio_failed;
		}

		lcd->lcd_3v3_vdd = of_get_named_gpio(np, "vdd-gpio", 0);
		if (lcd->lcd_3v3_vdd < 0) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
					lcd->lcd_3v3_vdd);
			ret = lcd->lcd_3v3_vdd;
			goto err_ldo2_gpio_failed;
		}
		if (lcd->lcd_1v8_vcc != lcd->lcd_3v3_vdd) {
			ret = gpio_request(lcd->lcd_3v3_vdd, "vdd_3v3_lcd");
			if (unlikely(ret < 0)) {
				pr_err("%s: gpio_request failed: %d\n",
					__func__, ret);
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

	/*later backlight device name needs to be changed as panel */
	lcd->bd = backlight_device_register("panel", &pdev->dev, lcd,
			&s6d7aa0x_backlight_ops, &props);

	if (IS_ERR(lcd->bd)) {
		ret = PTR_ERR(lcd->bd);
		goto err_backlight_device_register;
	}

	lcd->bd->props.max_brightness = PLATFORM_MAX_BRIGHTNESS;
	lcd->bd->props.brightness = PLATFORM_MID_BRIGHTNESS;
	lcd->bd->props.type = BACKLIGHT_RAW;

	ret = device_create_file(lcd->dev, &dev_attr_lcd_panel_name);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_lcd_panel_name.attr.name);
		 goto err_lcd_device;
	}

	ret = device_create_file(lcd->dev, &dev_attr_lcd_type);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_lcd_type.attr.name);
		goto err_lcd_name;
	}

#ifdef ENABLE_MDNIE_TUNING
	if (device_create_file(lcd->dev, &dev_attr_tuning) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_tuning.attr.name);

#endif /* ENABLE_MDNIE_TUNING */

#ifdef CONFIG_LDI_SUPPORT_MDNIE
	lcd->lcd_mDNIe_class = class_create(THIS_MODULE, "mdnie");

	if (IS_ERR(lcd->lcd_mDNIe_class)) {
		printk(KERN_INFO "Failed to create mdnie!\n");
		ret = PTR_ERR(lcd->lcd_mDNIe_class);
		goto err_lcd_name;
	}

	lcd->dev = device_create(lcd->lcd_mDNIe_class, NULL, 0, "%s", "mdnie");

	if (device_create_file(lcd->dev, &dev_attr_scenario) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_scenario.attr.name);
	if (device_create_file(lcd->dev, &dev_attr_outdoor) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_outdoor.attr.name);
	if (device_create_file(lcd->dev, &dev_attr_negative) < 0)
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_negative.attr.name);
#endif /* CONFIG_LDI_SUPPORT_MDNIE */

	mmp_register_panel(&panel_s6d7aa0x);
	return 0;

err_lcd_name:
	 device_remove_file(lcd->dev, &dev_attr_lcd_panel_name);
err_lcd_device:
	 backlight_device_unregister(lcd->bd);
err_backlight_device_register:
	device_destroy(lcd->lcd_class, 0);
err_device_create:
	class_destroy(lcd->lcd_class);
err_class_create:
	if (!lcd->lcd_vdd_type) {
		if ((lcd->lcd_3v3_vdd) &&
		(lcd->lcd_1v8_vcc != lcd->lcd_3v3_vdd))
			gpio_free(lcd->lcd_3v3_vdd);
	}
err_ldo2_gpio_failed:
	if (!lcd->lcd_vcc_type)
		gpio_free(lcd->lcd_1v8_vcc);
err_ldo1_gpio_failed:
	gpio_free(lcd->lcd_rst_n);
err_no_platform_data:
	kfree(lcd);

	return ret;

}

static int s6d7aa0x_remove(struct platform_device *dev)
{
	struct s6d7aa0x *lcd = panel_s6d7aa0x.plat_data;
	if (lcd->ldo_supply) {
		if ((lcd->lcd_3v3_vdd) &&
			(lcd->lcd_1v8_vcc != lcd->lcd_3v3_vdd))
			gpio_free(lcd->lcd_3v3_vdd);

		gpio_free(lcd->lcd_1v8_vcc);
	}


	gpio_free(lcd->lcd_rst_n);

	device_remove_file(lcd->dev, &dev_attr_lcd_panel_name);
	device_remove_file(lcd->dev, &dev_attr_lcd_type);
	backlight_device_unregister(lcd->bd);
	device_destroy(lcd->lcd_class, 0);
	class_destroy(lcd->lcd_class);

	mmp_unregister_panel(&panel_s6d7aa0x);
	kfree(lcd);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_s6d7aa0x_dt_match[] __initconst = {
	{ .compatible = "marvell,mmp-s6d7aa0x" },
	{},
};
#endif

static struct platform_driver s6d7aa0x_driver = {
	    .driver     = {
			.name   = "mmp-s6d7aa0x",
			.owner  = THIS_MODULE,
#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(mmp_s6d7aa0x_dt_match),
#endif
		},
		.probe      = s6d7aa0x_probe,
		.remove     = s6d7aa0x_remove,
};

static int s6d7aa0x_module_init(void)
{
	return platform_driver_register(&s6d7aa0x_driver);
}
static void s6d7aa0x_module_exit(void)
{
	    platform_driver_unregister(&s6d7aa0x_driver);
}
module_init(s6d7aa0x_module_init);
module_exit(s6d7aa0x_module_exit);


MODULE_DESCRIPTION("Panel driver for MIPI panel S6D7AA0X");
MODULE_LICENSE("GPL");
