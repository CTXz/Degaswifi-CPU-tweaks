 /* drivers/video/mmp/panel/mipi-ea8868b.c
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
#include <video/mmp_disp.h>
#include <video/mipi_display.h>
#include "mipi-ea8868b-param.h"
#include <linux/gpio.h>
#include <linux/platform_data/panel-ea8868b.h>
#include "smart_mtp_ea8868.h"

/* PMIC Regulator based supply to LCD */
#define REGULATOR_SUPPLY	1
/* gpio controlled LDO based supply to LCD */
#define LDO_SUPPLY			0
#define EA8868B_DEBUG
#define EA8868B_ACL_USE
#define MAX_BRIGHTNESS		(255)
#define DEFAULT_BRIGHTNESS	(180)

#ifdef EA8868B_SMART_DIMMING
#define EA8868B_MTP_GAMMA_1	(0xD3)
#define EA8868B_MTP_GAMMA_2	(0xD4)
#define EA8868B_MTP_GAMMA_3	(0xE0)
#define EA8868B_MTP_SIZE	(21)
#endif

#define EA8868B_ID2	(0x4A)

struct ea8868b {
	struct device *dev;
	struct class *lcd_class;
	struct ea8868b_panel_data  *pdata;
	struct backlight_device *bd;
	u32 (*set_panel_id)(struct mmp_panel *panel);
	/* Further fields can be added here */
#ifdef EA8868B_SMART_DIMMING
	struct SMART_DIM smart_ea8868;
#endif
#ifdef EA8868B_ACL_USE
		bool acl_on;
#endif

	int last_bl;
	int lcd_rst_n;
	int lcd_en_gpio1;
	int lcd_en_gpio2;
	u32 lcd_iovdd_type;
	u32 lcd_avdd_type;
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

static size_t ea8868b_read_reg(struct mmp_panel *panel,
		u8 addr, u8 len, char *dst)
{
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	struct mmp_dsi_buf *dbuf;

	static char set_read_addr[] = {0xFD, 0x00};
	static char get_mcs_data[] = {0xFE};
	static char pkt_size_cmd[] = {0x00};

	static struct mmp_dsi_cmd_desc smart_dim_mtp_read_cmds[] = {
		{MIPI_DSI_DCS_SHORT_WRITE_PARAM, HS_MODE, 10,
			sizeof(set_read_addr), set_read_addr},
		{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, HS_MODE, 0,
			sizeof(pkt_size_cmd), pkt_size_cmd},
		{MIPI_DSI_DCS_READ, HS_MODE, 0, sizeof(get_mcs_data),
			get_mcs_data},
	};

	if (unlikely(!dst)) {
		pr_err("%s: dst addr is null\n", __func__);
		return 0;
	}

	dbuf = kmalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (unlikely(!dbuf)) {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
		return 0;
	}

	pkt_size_cmd[0] = len;
	set_read_addr[1] = addr;

	mmp_phy_dsi_rx_cmd_array(path->phy, dbuf,
		smart_dim_mtp_read_cmds,
		ARRAY_SIZE(smart_dim_mtp_read_cmds));

	memcpy(dst, dbuf->data, len);

#ifdef EA8868B_DEBUG
{
	int show_buffer_pos = 0;
	char show_buffer[256];
	int i;
	pr_info("%s, addr : 0x%x, len : %d\n", __func__, addr, len);
	/* for debug */
	show_buffer_pos +=
		snprintf(show_buffer, 256,
				"read_reg :\n");
	for (i = 1; i < len + 1; i++) {
		show_buffer_pos +=
			snprintf(show_buffer + show_buffer_pos, 256,
					"%02x ", dbuf->data[i - 1]);
		if (!(i % 7)) {
			show_buffer_pos +=
				snprintf(show_buffer + show_buffer_pos,
						256, "\n");
		}
	}
	pr_info("%s\n", show_buffer);
}
#endif
	kfree(dbuf);

	return len;
}

#ifdef EA8868B_SMART_DIMMING
size_t ea8868b_read_mtp(struct mmp_panel *panel, char *mtp_data)
{
	char first_mtp[EA8868B_MTP_SIZE];
	int mtp_size = EA8868B_MTP_SIZE;

	ea8868b_read_reg(panel, EA8868B_MTP_GAMMA_1, mtp_size, &first_mtp[0]);

	memcpy(mtp_data, first_mtp, mtp_size);

	return 0;
}
#endif

/********************* ELVSS *******************/
#define LCD_ELVSS_DELTA_300CD (0x00)
#define LCD_ELVSS_DELTA_200CD (0x06)
#define LCD_ELVSS_DELTA_160CD (0x0A)
#define LCD_ELVSS_DELTA_100CD (0x0F)

#define LCD_ELVSS_RESULT_LIMIT (0x1F)

static int GET_ELVSS_ID[] = {
	LCD_ELVSS_DELTA_100CD,/* 0 = 30_dimming,*/
	LCD_ELVSS_DELTA_100CD,/* 1 = 40*/
	LCD_ELVSS_DELTA_100CD,/* 2 = 50*/
	LCD_ELVSS_DELTA_100CD,/* 3 = 60,*/
	LCD_ELVSS_DELTA_100CD,/* 4 = 70,*/
	LCD_ELVSS_DELTA_100CD,/* 5 = 80,*/
	LCD_ELVSS_DELTA_100CD,/* 6 = 90,*/
	LCD_ELVSS_DELTA_100CD,/* 7 = 100,*/
	LCD_ELVSS_DELTA_160CD,/* 8 = 110,*/
	LCD_ELVSS_DELTA_160CD,/* 9 = 120,*/
	LCD_ELVSS_DELTA_160CD,/* 10 = 130,*/
	LCD_ELVSS_DELTA_160CD,/* 11 = 140,*/
	LCD_ELVSS_DELTA_160CD,/* 12 = 150,*/
	LCD_ELVSS_DELTA_160CD,/* 13 = 160,*/
	LCD_ELVSS_DELTA_200CD,/* 14 = 170,*/
	LCD_ELVSS_DELTA_200CD,/* 15 = 180,*/
	LCD_ELVSS_DELTA_200CD,/* 16 = 190,*/
	LCD_ELVSS_DELTA_200CD,/* 17 = 200,*/
	LCD_ELVSS_DELTA_300CD,/* 18 = 210,*/
	LCD_ELVSS_DELTA_300CD,/* 19 = 220,*/
	LCD_ELVSS_DELTA_300CD,/* 20 = 230,*/
	LCD_ELVSS_DELTA_300CD,/* 21 = 240,*/
	LCD_ELVSS_DELTA_300CD,/* 22 = 250,*/
	LCD_ELVSS_DELTA_300CD/* 23 = 300,*/
};

#define LCD_ELVSS_DEFAULT_100CD (0x15)
#define LCD_ELVSS_DEFAULT_160CD (0x0F)
#define LCD_ELVSS_DEFAULT_200CD (0x0B)
#define LCD_ELVSS_DEFAULT_300CD (0x05)
static int GET_DEFAULT_ELVSS_ID[] = {
	LCD_ELVSS_DEFAULT_100CD,/* 0 = 30_dimming,*/
	LCD_ELVSS_DEFAULT_100CD,/* 1 = 40*/
	LCD_ELVSS_DEFAULT_100CD,/* 2 = 50*/
	LCD_ELVSS_DEFAULT_100CD,/* 3 = 60,*/
	LCD_ELVSS_DEFAULT_100CD,/* 4 = 70,*/
	LCD_ELVSS_DEFAULT_100CD,/* 5 = 80,*/
	LCD_ELVSS_DEFAULT_100CD,/* 6 = 90,*/
	LCD_ELVSS_DEFAULT_100CD,/* 7 = 100,*/
	LCD_ELVSS_DEFAULT_160CD,/* 8 = 110,*/
	LCD_ELVSS_DEFAULT_160CD,/* 9 = 120,*/
	LCD_ELVSS_DEFAULT_160CD,/* 10 = 130,*/
	LCD_ELVSS_DEFAULT_160CD,/* 11 = 140,*/
	LCD_ELVSS_DEFAULT_160CD,/* 12 = 150,*/
	LCD_ELVSS_DEFAULT_160CD,/* 13 = 160,*/
	LCD_ELVSS_DEFAULT_200CD,/* 14 = 170,*/
	LCD_ELVSS_DEFAULT_200CD,/* 15 = 180,*/
	LCD_ELVSS_DEFAULT_200CD,/* 16 = 190,*/
	LCD_ELVSS_DEFAULT_200CD,/* 17 = 200,*/
	LCD_ELVSS_DEFAULT_300CD,/* 18 = 210,*/
	LCD_ELVSS_DEFAULT_300CD,/* 19 = 220,*/
	LCD_ELVSS_DEFAULT_300CD,/* 20 = 230,*/
	LCD_ELVSS_DEFAULT_300CD,/* 21 = 240,*/
	LCD_ELVSS_DEFAULT_300CD,/* 22 = 250,*/
	LCD_ELVSS_DEFAULT_300CD/* 23 = 300,*/
};

static int get_candela_index(int bl_level)
{
	int backlightlevel;

	/* brightness setting from platform is from 0 to 255
	 * But in this driver, brightness is only supported
	 * from 0 to 24 */

	switch (bl_level) {
	case 0 ... 39:
		backlightlevel = GAMMA_30CD; /* 0*/
		break;
	case 40 ... 49:
		backlightlevel = GAMMA_40CD; /* 1 */
		break;
	case 50 ... 59:
		backlightlevel = GAMMA_50CD; /* 2 */
		break;
	case 60 ... 69:
		backlightlevel = GAMMA_60CD; /* 3 */
		break;
	case 70 ... 79:
		backlightlevel = GAMMA_70CD; /* 4 */
		break;
	case 80 ... 89:
		backlightlevel = GAMMA_80CD; /* 5 */
		break;
	case 90 ... 99:
		backlightlevel = GAMMA_90CD; /* 6 */
		break;
	case 100 ... 109:
		backlightlevel = GAMMA_100CD; /* 7 */
		break;
	case 110 ... 119:
		backlightlevel = GAMMA_110CD; /* 8 */
		break;
	case 120 ... 129:
		backlightlevel = GAMMA_120CD; /* 9 */
		break;
	case 130 ... 139:
		backlightlevel = GAMMA_130CD; /* 10 */
		break;
	case 140 ... 149:
		backlightlevel = GAMMA_140CD; /* 11 */
		break;
	case 150 ... 159:
		backlightlevel = GAMMA_150CD; /* 12 */
		break;
	case 160 ... 169:
		backlightlevel = GAMMA_160CD; /* 13 */
		break;
	case 170 ... 179:
		backlightlevel = GAMMA_170CD; /* 14 */
		break;
	case 180 ... 189:
		backlightlevel = GAMMA_180CD; /* 15 */
		break;
	case 190 ... 199:
		backlightlevel = GAMMA_190CD; /* 16 */
		break;
	case 200 ... 209:
		backlightlevel = GAMMA_200CD; /* 17 */
		break;
	case 210 ... 219:
		backlightlevel = GAMMA_210CD; /* 18 */
		break;
	case 220 ... 229:
		backlightlevel = GAMMA_220CD; /* 10 */
		break;
	case 230 ... 239:
		backlightlevel = GAMMA_230CD; /* 20 */
		break;
	case 240 ... 249:
		backlightlevel = GAMMA_240CD; /* 21 */
		break;
	case 250 ... 254:
		backlightlevel = GAMMA_250CD; /* 22 */
		break;
	case 255:
		backlightlevel = GAMMA_300CD; /* 23 */
		break;
	default:
		backlightlevel = GAMMA_40CD; /* 1 */
		break;
	}
	return backlightlevel;
}

static int ea8868b_set_gamma_level(struct mmp_panel *panel, int bl_level)
{
	struct ea8868b *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	int cd_index;
	int gamma_lux;

	cd_index = get_candela_index(bl_level);
	gamma_lux = lux_tbl_acl[cd_index];

#ifdef EA8868B_SMART_DIMMING
	plat->smart_ea8868.brightness_level = gamma_lux;
	generate_gamma(&plat->smart_ea8868, &gamma_set[1], GAMMA_SET_MAX);
#else
	memcpy(&gamma_set[1], gamma_set_default[cd_index], GAMMA_SET_MAX);
#endif
#ifdef EA8868B_DEBUG
	{
		int i;
		for (i = 1; i < GAMMA_SET_MAX + 1; i++)
			pr_info("GAMMA[%d] : 0x%x\n", i, gamma_set[i]);
	}
#endif
	pr_info("bl_level : %d, gamma_lux : %d\n", bl_level, gamma_lux);

	mmp_phy_dsi_tx_cmd_array(path->phy,
			ea8868b_gamma_update_cmds,
			ARRAY_SIZE(ea8868b_gamma_update_cmds));

	return 0;
}

static int ea8868b_set_elvss_level(struct mmp_panel *panel, int bl_level)
{
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	unsigned char calc_elvss;
	int id3, id2;
	int cd_index = get_candela_index(bl_level);

#ifdef CONFIG_OF
	id3 = path->panel->id && 0xFF;
	id2 = (path->panel->id >> 8) & 0xFF;
#else
	struct ea8868b *plat = panel->plat_data;

	id3 = plat->pdata->panel_id & 0xFF;
	id2 = (plat->pdata->panel_id >> 8) & 0xFF;
#endif

	if (id2 == EA8868B_ID2)
		calc_elvss = id3 + GET_ELVSS_ID[cd_index];
	else
		calc_elvss = GET_DEFAULT_ELVSS_ID[cd_index];
	pr_debug("%s: ID2=%x, ID3=%x, calc_elvss = %x\n",
			__func__, id2, id3, calc_elvss);
	if (calc_elvss > LCD_ELVSS_RESULT_LIMIT)
		calc_elvss = LCD_ELVSS_RESULT_LIMIT;
	elvss_cond_set[1] = calc_elvss;
	elvss_cond_set[2] = calc_elvss;
	elvss_cond_set[3] = calc_elvss;
	elvss_cond_set[4] = calc_elvss;
	mmp_phy_dsi_tx_cmd_array(path->phy,
			ea8868b_elvss_update_cmds,
			ARRAY_SIZE(ea8868b_elvss_update_cmds));
	return 0;
}

#ifdef EA8868B_ACL_USE
static int ea8868b_set_acl_level(struct mmp_panel *panel, int bl_level)
{
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	int cd_index = get_candela_index(bl_level);
	if (cd_index < GAMMA_50CD) {
		mmp_phy_dsi_tx_cmd_array(path->phy,
				ea8868b_acl_off_cmds,
				ARRAY_SIZE(ea8868b_acl_off_cmds));
		return 0;
	}
	if (cd_index < GAMMA_300CD) {
		ea8868b_acl_on_cmds[1].length = sizeof(acl_cond_set_40);
		ea8868b_acl_on_cmds[1].data = acl_cond_set_40;
	} else {
		ea8868b_acl_on_cmds[1].length = sizeof(acl_cond_set_50);
		ea8868b_acl_on_cmds[1].data = acl_cond_set_50;
	}
	mmp_phy_dsi_tx_cmd_array(path->phy, ea8868b_acl_on_cmds,
			ARRAY_SIZE(ea8868b_acl_on_cmds));
	return 0;
}
#endif

static u32 ea8868b_set_panel_id(struct mmp_panel *panel)
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
		pr_info("panel id :0x%x\n", read_id);
	} else {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
	}

	return read_id;
}

static int ea8868b_panel_enable(struct mmp_path *path)
{
	struct ea8868b *plat = path->panel->plat_data;
#ifdef EA8868B_SMART_DIMMING
	static int mtp_read_done;
#endif

	mmp_phy_dsi_tx_cmd_array(path->phy, ea8868b_video_init_cmds,
			ARRAY_SIZE(ea8868b_video_init_cmds));

#ifdef EA8868B_SMART_DIMMING
	if (!mtp_read_done && path && path->panel &&
			!path->panel->skip_poweron) {
		ea8868b_read_mtp(path->panel,
				(char *)&(plat->smart_ea8868.MTP));
		smart_dimming_init(&(plat->smart_ea8868));
		mtp_read_done = 1;
	}
#endif

	mmp_phy_dsi_tx_cmd_array(path->phy, ea8868b_video_power_on_cmds,
			ARRAY_SIZE(ea8868b_video_power_on_cmds));

	return 0;
}

static int ea8868b_panel_disable(struct mmp_path *path)
{
	mmp_phy_dsi_tx_cmd_array(path->phy,
			ea8868b_video_display_off_cmds,
			ARRAY_SIZE(ea8868b_video_display_off_cmds));

	mmp_phy_dsi_tx_cmd_array(path->phy,
			ea8868b_video_sleep_in_cmds,
			ARRAY_SIZE(ea8868b_video_sleep_in_cmds));

	return 0;
}
#ifdef CONFIG_OF
static void ea8868b_panel_power(struct mmp_panel *panel, int skip_on, int on)
{
	static struct regulator *lcd_iovdd;
	static struct regulator *lcd_avdd;
	int err;
	struct ea8868b *plat = panel->plat_data;
	int lcd_rst_n = plat->lcd_rst_n;

	if (plat->lcd_iovdd_type == REGULATOR_SUPPLY) {
		if (!lcd_iovdd) {
			lcd_iovdd = regulator_get(panel->dev, "iovdd");
			if (IS_ERR_OR_NULL(lcd_iovdd)) {
				pr_err("%s: regulatori(iovdd- 1.8v) get error!\n",
						__func__);
				goto err_lcd_iovdd_type;
			}
		}
	}

	if ((plat->lcd_avdd_type == REGULATOR_SUPPLY)) {
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
		if (plat->lcd_avdd_type == REGULATOR_SUPPLY) {
			regulator_set_voltage(lcd_avdd, 3000000, 3000000);
			err = regulator_enable(lcd_avdd);
			if (unlikely(err < 0)) {
				pr_err("%s: lcd_avdd regulator enable failed %d\n",
						__func__, err);
				goto err_lcd_avdd_enable_failed;
			}
			mdelay(5);
		}

		if (plat->lcd_iovdd_type == REGULATOR_SUPPLY) {
			regulator_set_voltage(lcd_iovdd, 1800000, 1800000);
			err = regulator_enable(lcd_iovdd);
			if (unlikely(err < 0)) {
				pr_err("%s: lcd_iovdd regulator enable failed %d\n",
						__func__, err);
				goto err_lcd_iovdd_enable_failed;
			}
			mdelay(15);
		}

		if (plat->lcd_iovdd_type == LDO_SUPPLY) {
			if (plat->lcd_en_gpio1)
				gpio_direction_output(plat->lcd_en_gpio1, 1);
		}

		if (plat->lcd_avdd_type == LDO_SUPPLY) {
			if ((plat->lcd_en_gpio2) &&
				(plat->lcd_en_gpio1 != plat->lcd_en_gpio2))
				gpio_direction_output(plat->lcd_en_gpio2, 1);
			}

		if (!skip_on) {
			gpio_direction_output(lcd_rst_n, 1);
			udelay(20);
			gpio_direction_output(lcd_rst_n, 0);
			udelay(2000);
			gpio_direction_output(lcd_rst_n, 1);
			mdelay(150);
		}

	} else {
		/* set panel reset */
		gpio_direction_output(lcd_rst_n, 0);

		if (plat->lcd_iovdd_type == REGULATOR_SUPPLY) {
			/* disable V_LCD_1.8V */
			regulator_disable(lcd_iovdd);
			mdelay(10);
		}

		if (plat->lcd_avdd_type == REGULATOR_SUPPLY)
			/* disable V_LCD_3.0V */
			regulator_disable(lcd_avdd);

		 if (plat->lcd_avdd_type == LDO_SUPPLY) {
			if ((plat->lcd_en_gpio2) &&
				(plat->lcd_en_gpio1 != plat->lcd_en_gpio2))
				gpio_direction_output(plat->lcd_en_gpio2, 0);
		 }

		 if (plat->lcd_iovdd_type == LDO_SUPPLY)
			gpio_direction_output(plat->lcd_en_gpio1, 0);
	}
	return;

err_lcd_iovdd_enable_failed:
	if (plat->lcd_avdd_type == REGULATOR_SUPPLY)
		regulator_disable(lcd_avdd);
err_lcd_avdd_enable_failed:
	if (plat->lcd_avdd_type == REGULATOR_SUPPLY)
		regulator_put(lcd_avdd);
err_lcd_avdd_type:
	if (plat->lcd_avdd_type == REGULATOR_SUPPLY)
		lcd_avdd = NULL;
	if (plat->lcd_iovdd_type == REGULATOR_SUPPLY)
		regulator_put(lcd_iovdd);

err_lcd_iovdd_type:
	if (plat->lcd_iovdd_type == REGULATOR_SUPPLY)
		lcd_iovdd = NULL;

	return;
}
#else
static void ea8868b_panel_power(struct mmp_panel *panel, int on) {}
#endif

#ifdef CONFIG_OF
static void of_ea8868b_set_panel_id(struct mmp_panel *panel)
{
	struct ea8868b *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	path->panel->id = boot_panel_id ?
		boot_panel_id : plat->set_panel_id(panel);
	if (!path->panel->id)
		pr_err("%s: panel id configuration is missing\n", __func__);
}
#else
static void of_ea8868b_set_panel_id(struct mmp_panel *panel) {}
#endif
static void ea8868b_onoff(struct mmp_panel *panel, int status)
{
	struct ea8868b *plat = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	pr_info("called %s with status %d\n", __func__, status);
	if (status) {
		/* power on */
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(1);
		else
			ea8868b_panel_power(panel, 0, 1);

		if ((path && path->panel && !path->panel->id) ||
				(plat->pdata && !plat->pdata->panel_id &&
				 plat->set_panel_id)) {
			if (plat->pdata)
				plat->pdata->panel_id = boot_panel_id ?
					boot_panel_id :
					plat->set_panel_id(panel);
		} else
			of_ea8868b_set_panel_id(panel);

		ea8868b_panel_enable(path);
	} else {
		/* power off */
		ea8868b_panel_disable(path);
		if ((plat->pdata) && (plat->pdata->mi->plat_set_onoff))
			plat->pdata->mi->plat_set_onoff(0);
		else
			ea8868b_panel_power(panel, 0, 0);
	}
}


static void ea8868b_reduced_onoff(struct mmp_panel *panel, int status)
{
	struct ea8868b *plat = panel->plat_data;
#ifdef EA8868B_SMART_DIMMING
	static int mtp_read_done;
#endif
	if (status) {
		/* power on */
		ea8868b_panel_power(panel, 1, 1);

		/*
		 * dsi rx is not proper for first smart dim mtp read.
		 * So it temporarly rely on set_panel_id whcih uses dsi rx
		 * read command and bringing dsi rx state to normal.
		 * Now, smart dim mtp could read using set_read_addr
		 */
		if (plat->pdata) {
			if (boot_panel_id)
				plat->pdata->panel_id = boot_panel_id ?
					boot_panel_id :
				plat->set_panel_id(panel);
		} else
			of_ea8868b_set_panel_id(panel);
#ifdef EA8868B_SMART_DIMMING
	if (!mtp_read_done) {
		ea8868b_read_mtp(panel,
				(char *)&(plat->smart_ea8868.MTP));
		smart_dimming_init(&(plat->smart_ea8868));
		mtp_read_done = 1;
	}
#endif
	} else {
		/* power off */
		ea8868b_panel_power(panel, 1, 0);
	}
	panel->skip_poweron = 1;
}

static struct mmp_mode mmp_modes_ea8868b[] = {
	[0] = {
		.pixclock_freq = 26764320,
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
		.hsync_len = 4,
		.left_margin = 32,
		.right_margin = 32,
		.vsync_len = 2,
		.upper_margin = 8,
		.lower_margin = 4,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_RGB888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
	},
};

static int ea8868b_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_modes_ea8868b;
	if (get_disp_pll3_vco_setup()) {
		(*modelist)->pixclock_freq = 28988400;
		(*modelist)->hsync_len = 4;
		(*modelist)->left_margin = 48;
		(*modelist)->right_margin = 48;
		(*modelist)->vsync_len = 2;
		(*modelist)->upper_margin = 3;
		(*modelist)->lower_margin = 28;
	}
	return 1;
}

static struct mmp_panel panel_ea8868b = {
	.name = "ea8868b",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = ea8868b_get_modelist,
	.set_onoff = ea8868b_onoff,
	.reduced_onoff = ea8868b_reduced_onoff,
};

static int ea8868b_get_brightness(struct backlight_device *bd)
{
	struct ea8868b *lcd = bl_get_data(bd);

	return lcd->last_bl;
}


static int ea8868b_set_brightness(struct backlight_device *bd)
{
	struct ea8868b *lcd = bl_get_data(bd);
	int bl_level = bd->props.brightness;

	if (bl_level == lcd->last_bl)
		return 0;

	ea8868b_set_gamma_level(&panel_ea8868b, bl_level);
	ea8868b_set_elvss_level(&panel_ea8868b, bl_level);
#ifdef EA8868B_ACL_USE
	ea8868b_set_acl_level(&panel_ea8868b, lcd->acl_on ? bl_level : 0);
#endif
	lcd->last_bl = bl_level;
	return 0;
}

static const struct backlight_ops ea8868b_backlight_ops = {

	.get_brightness = ea8868b_get_brightness,
	.update_status = ea8868b_set_brightness,
};

static ssize_t lcd_panelname_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ea8868b");
}

static DEVICE_ATTR(lcd_panelname, S_IRUGO | S_IXOTH,
		lcd_panelname_show, NULL);

static ssize_t lcd_type_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct mmp_path *path = mmp_get_path(panel_ea8868b.plat_path_name);
	u32 panel_id;

#ifdef CONFIG_OF
	panel_id = path->panel->id;
#else
	struct ea8868b *lcd = dev_get_drvdata(dev);
	panel_id = lcd->pdata->panel_id;
#endif
	return sprintf(buf, "EA_%x%x%x",
			(panel_id >> 16) & 0xFF,
			(panel_id >> 8) & 0xFF,
			panel_id  & 0xFF);
}

static DEVICE_ATTR(lcd_type, S_IRUGO | S_IXOTH,
		lcd_type_show, NULL);

static int ea8868b_probe(struct platform_device *pdev)
{

	struct ea8868b *lcd;
	int ret;
	const char *path_name;
	struct device_node *np = pdev->dev.of_node;
	struct backlight_properties props = {
		.brightness = DEFAULT_BRIGHTNESS,
		.max_brightness = MAX_BRIGHTNESS,
		.type = BACKLIGHT_RAW,
	};

	pr_debug("called %s\n", __func__);
	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (unlikely(!lcd)) {
		ret =  -ENOMEM;
		return ret;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(np,
				"marvell,path-name", &path_name);
		if (unlikely(ret < 0)) {
			pr_err("%s: Path name not found\n", __func__);
			goto err_no_platform_data;
		}

		panel_ea8868b.plat_path_name = path_name;

	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
			goto err_no_platform_data;
		}

		lcd->pdata = pdev->dev.platform_data;
		panel_ea8868b.plat_path_name = lcd->pdata->mi->plat_path_name;
	}

	panel_ea8868b.plat_data = lcd;
	lcd->set_panel_id = &ea8868b_set_panel_id;
	panel_ea8868b.dev = &pdev->dev;

	lcd->lcd_rst_n = of_get_named_gpio(np, "rst-gpio", 0);
	if (unlikely(lcd->lcd_rst_n < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
			lcd->lcd_rst_n);
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

	if (!lcd->lcd_iovdd_type) {
		lcd->lcd_en_gpio1 = of_get_named_gpio(np, "ldo_en_gpio1", 0);
		if (unlikely(lcd->lcd_en_gpio1 < 0)) {
			pr_err("%s: of_get_named_gpio failed: %d\n",
				__func__, lcd->lcd_en_gpio1);
			goto err_ldo1_gpio_failed;
		}

		ret = gpio_request(lcd->lcd_en_gpio1, "lcd_ldo_en1");
		if (unlikely(ret < 0)) {
			pr_err("%s: gpio_request %d failed: %d\n",
				__func__, lcd->lcd_en_gpio1, ret);
			goto err_ldo1_gpio_failed;
		}
	}

	if (!lcd->lcd_avdd_type) {
		lcd->lcd_en_gpio2 = of_get_named_gpio(np, "ldo_en_gpio2", 0);
		if (unlikely(lcd->lcd_en_gpio2 < 0)) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
					lcd->lcd_en_gpio2);
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

	/*later backlight device name needs to be changed as panel */
	lcd->bd = backlight_device_register("lcd-bl", &pdev->dev, lcd,
			&ea8868b_backlight_ops, &props);

	if (IS_ERR(lcd->bd)) {
		ret = PTR_ERR(lcd->bd);
		goto err_backlight_device_register;
	}

	ret = device_create_file(lcd->dev, &dev_attr_lcd_panelname);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_lcd_panelname.attr.name);
		goto err_lcd_device;
	}
	ret = device_create_file(lcd->dev, &dev_attr_lcd_type);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_lcd_type.attr.name);
		goto err_lcd_name;
	}

	mmp_register_panel(&panel_ea8868b);
	return 0;

err_lcd_name:
	device_remove_file(lcd->dev, &dev_attr_lcd_panelname);
err_lcd_device:
	backlight_device_unregister(lcd->bd);
err_backlight_device_register:
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

static int ea8868b_remove(struct platform_device *dev)
{
	struct ea8868b *lcd = panel_ea8868b.plat_data;

	if (lcd->lcd_avdd_type == LDO_SUPPLY) {
		if ((lcd->lcd_en_gpio2) &&
			(lcd->lcd_en_gpio1 != lcd->lcd_en_gpio2))
			gpio_free(lcd->lcd_en_gpio2);
	}
	if (lcd->lcd_iovdd_type == LDO_SUPPLY)
		gpio_free(lcd->lcd_en_gpio1);

	gpio_free(lcd->lcd_rst_n);

	device_remove_file(lcd->dev, &dev_attr_lcd_panelname);
	device_remove_file(lcd->dev, &dev_attr_lcd_type);
	backlight_device_unregister(lcd->bd);
	device_destroy(lcd->lcd_class, 0);
	class_destroy(lcd->lcd_class);

	mmp_unregister_panel(&panel_ea8868b);
	kfree(lcd);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_ea8868b_dt_match[] = {
	{ .compatible = "marvell,mmp-ea8868b" },
	{},
};
#endif
static struct platform_driver ea8868b_driver = {
	.driver     = {
		.name   = "mmp-ea8868b",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(mmp_ea8868b_dt_match),
#endif
	},
	.probe      = ea8868b_probe,
	.remove     = ea8868b_remove,
};

static int ea8868b_module_init(void)
{
	return platform_driver_register(&ea8868b_driver);
}
static void ea8868b_module_exit(void)
{
	platform_driver_unregister(&ea8868b_driver);
}
module_init(ea8868b_module_init);
module_exit(ea8868b_module_exit);

MODULE_DESCRIPTION("Panel driver for MIPI panel ea8868b");
MODULE_LICENSE("GPL");
