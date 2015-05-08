/* drivers/video/mmp/panel/mmp-dsi-panel.c
 *
 * Copyright (C) 2013 Samsung Electronics Co, Ltd.
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
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <video/mmp_disp.h>
#include <video/mmp_esd.h>
#include <video/mipi_display.h>
#include <linux/gpio.h>
#include <linux/platform_data/mmp-panel-generic.h>
#include <linux/platform_data/mmp-panel-mdnie.h>
#include <linux/platform_data/mmp-panel-backlight.h>
#include <linux/power_supply.h>

static u32 boot_panel_id;
static struct mmp_panel mmp_dsi_panel;
const char *tx_modes[] = {
	"dsi-hs-mode",
	"dsi-lp-mode",
};

static int __init get_boot_panel_id(char *str)
{
	char *endp;

	boot_panel_id = memparse(str, &endp);
	pr_info("%s: boot_panel_id = 0x%8x\n", __func__, boot_panel_id);

	return 1;
}
early_param("lcd_id", get_boot_panel_id);

static struct mmp_dsi_cmd_desc *
find_cmd_desc(struct mmp_dsi_cmds *cmds, u8 reg)
{
	int i;

	for (i = 0; i < cmds->nr_cmds; i++)
		if (cmds->cmds[i].data[0] == reg) {
			pr_info("%s found desc\n", __func__);
			return &cmds->cmds[i];
		}

	pr_warn("%s: can't find 0x%02x\n", __func__, reg);
	return NULL;
}

static int mmp_dsi_panel_get_temperature(int *temp)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = power_supply_get_by_name("battery");
	if (psy && psy->get_property) {
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
		if (ret)
			return ret;
		*temp = val.intval / 10;
		pr_debug("%s: temperature (%d)\n", __func__, *temp);
	}
	return 0;
}

static int mmp_dsi_panel_temp_compensation(struct mmp_panel *panel)
{
	struct lcd *lcd = panel->plat_data;
	struct temp_compensation *temp_comp;
	unsigned int nr_temp_comp = lcd->nr_temp_comp;
	int temperature = 0, ret, i;
	struct mmp_dsi_cmd_desc *desc;

	if (!lcd->init_cmds.nr_cmds) {
		pr_warn("%s: init_cmds empty\n", __func__);
		return 0;
	}

	ret = mmp_dsi_panel_get_temperature(&temperature);
	if (ret) {
		pr_warn("%s: can't get temperature (%d)\n",
				__func__, ret);
		return 0;
	}

	for (i = 0; i < nr_temp_comp; i++) {
		temp_comp = &lcd->temp_comp[i];
		if (!temp_comp || !temp_comp->old_data ||
				!temp_comp->new_data) {
			pr_warn("%s: wrong input data\n", __func__);
			continue;
		}

		desc = find_cmd_desc(&lcd->init_cmds,
				temp_comp->old_data[0]);
		if (!desc) {
			pr_warn("%s: can't find (0x%02X) cmd desc\n",
					__func__, temp_comp->old_data[0]);
			continue;
		}

		if ((temp_comp->trig_type == TEMPERATURE_LOW &&
					temperature <= temp_comp->temperature)
				|| (temp_comp->trig_type == TEMPERATURE_HIGH &&
				 temperature >= temp_comp->temperature)) {
			if (!memcmp(desc->data, temp_comp->new_data,
						desc->length))
				continue;
			memcpy(desc->data, temp_comp->new_data, desc->length);
			pr_info("%s: update 0x%02Xh, 0x%02X\n",
					__func__, desc->data[0], desc->data[1]);
			pr_info("%s: temperature (%d) is %s (%d)\n",
					__func__, temperature,
					temp_comp->trig_type ==
					TEMPERATURE_LOW ?
					"lower than" : "higher than",
					temp_comp->temperature);
		} else {
			if (!memcmp(desc->data, temp_comp->old_data,
						desc->length)) {
				continue;
			}
			memcpy(desc->data, temp_comp->old_data, desc->length);
			pr_info("%s: restore 0x%02Xh, 0x%02X\n",
					__func__, desc->data[0], desc->data[1]);
			pr_info("%s: temperature (%d) is %s (%d)\n",
					__func__, temperature,
					temp_comp->trig_type ==
					TEMPERATURE_LOW ?
					"lower than" : "higher than",
					temp_comp->temperature);
		}
	}

	return 0;
}

static size_t mmp_dsi_panel_read_reg(struct mmp_panel *panel,
		struct mmp_dsi_buf *out_dbuf, u8 addr, u8 len)
{
	struct mmp_path *path =
		mmp_get_path(panel->plat_path_name);
	static char read_len[] = {0x00};
	static char read_addr[] = {0x00};
	static struct mmp_dsi_cmd_desc read_cmds[] = {
		{MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, DSI_HS_MODE, 0,
			sizeof(read_len), read_len},
		{MIPI_DSI_DCS_READ, DSI_HS_MODE, 0,
			sizeof(read_addr), read_addr},
	};

	read_len[0] = len;
	read_addr[0] = addr;

	mmp_phy_dsi_rx_cmd_array(path->phy, out_dbuf, read_cmds,
			ARRAY_SIZE(read_cmds));

	pr_debug("addr:0x%2x, len:%d, data[0]:%x\n",
			addr, len, out_dbuf->data[0]);
	return len;
}

static u32 set_panel_id(struct mmp_panel *panel)
{
	u32 read_id = 0;
	struct mmp_dsi_buf *dbuf;
	struct lcd *lcd = panel->plat_data;

	mutex_lock(&lcd->access_ok);
	dbuf = kzalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (dbuf) {
		mmp_dsi_panel_read_reg(panel, dbuf, 0x04, 4);
		mmp_dsi_panel_read_reg(panel, dbuf, lcd->id_regs[0], 1);
		read_id |= dbuf->data[0] << 16;
		mmp_dsi_panel_read_reg(panel, dbuf, lcd->id_regs[1], 1);
		read_id |= dbuf->data[0] << 8;
		mmp_dsi_panel_read_reg(panel, dbuf, lcd->id_regs[2], 1);
		read_id |= dbuf->data[0];
		kfree(dbuf);
		pr_info("Panel id is 0x%x\n", read_id);
	} else {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
	}
	mutex_unlock(&lcd->access_ok);

	return read_id;
}

static int mmp_dsi_panel_read_status(struct mmp_panel *panel)
{
	unsigned int status = 0;
	struct mmp_dsi_buf *dbuf;
	struct lcd *lcd = panel->plat_data;

	mutex_lock(&lcd->access_ok);
	if (!lcd->active) {
		mutex_unlock(&lcd->access_ok);
		return 0;
	}

	dbuf = kzalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (dbuf) {
		mmp_dsi_panel_read_reg(panel, dbuf, 0x04, 4);
		mmp_dsi_panel_read_reg(panel, dbuf, lcd->status_reg, 1);
		status = dbuf->data[0];
		kfree(dbuf);
		pr_info("%s: status 0x%x\n", __func__, status);
	} else {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
	}

	if (status != lcd->status_ok)
		pr_info("%s: err detected(0x%x, 0x%x)\n",
				__func__, status, lcd->status_ok);
	mutex_unlock(&lcd->access_ok);

	return (status == lcd->status_ok);
}

static int mmp_dsi_panel_read_comapre_reg(struct mmp_panel *panel,
		u8 *table, u8 len)
{
	struct mmp_dsi_buf *dbuf;
	unsigned char addr = table[0];
	unsigned char *data = table + 1;
	int i, ret = 0;

	dbuf = kzalloc(sizeof(struct mmp_dsi_buf), GFP_KERNEL);
	if (dbuf) {
		mmp_dsi_panel_read_reg(panel, dbuf, addr, len);
		for (i = 0; i < len - 1; i++)
			if (dbuf->data[i] != data[i]) {
				pr_warn("[%Xh] - (0x%x, 0x%x) not match\n",
						addr, data[i], dbuf->data[i]);
				ret = -EINVAL;
			} else {
				pr_debug("[%Xh] - (0x%x, 0x%x) match\n",
						addr, data[i], dbuf->data[i]);
			}
		kfree(dbuf);
	} else {
		pr_err("%s: can't alloc dsi rx buffer\n", __func__);
	}

	return ret;
}


int mmp_dsi_panel_verify_reg(struct mmp_panel *panel,
		struct mmp_dsi_cmd_desc cmds[], int count)
{
	u32 loop;
	int ret = 0;

	for (loop = 0; loop < count; loop++) {
		ret = mmp_dsi_panel_read_comapre_reg(panel,
				cmds[loop].data, cmds[loop].length);
		if (ret < 0)
			pr_err("%s: not match table\n", __func__);

	}
	return ret;
}
EXPORT_SYMBOL(mmp_dsi_panel_verify_reg);

static int mmp_dsi_panel_enable(struct mmp_path *path)
{
	struct lcd *lcd = path->panel->plat_data;
	struct device *dev = path->panel->dev;

	mutex_lock(&lcd->access_ok);
	if (IS_ENABLED(CONFIG_OF) && lcd->init_cmds.nr_cmds) {
		if (lcd->temp_comp_en)
			mmp_dsi_panel_temp_compensation(path->panel);
		dev_info(dev, "send init_cmds\n");
		mmp_phy_dsi_tx_cmd_array(path->phy,
				lcd->init_cmds.cmds,
				lcd->init_cmds.nr_cmds);
	}

	if (IS_ENABLED(CONFIG_OF) && lcd->enable_cmds.nr_cmds) {
		dev_info(dev, "send enable_cmds\n");
		mmp_phy_dsi_tx_cmd_array(path->phy, lcd->enable_cmds.cmds,
				lcd->enable_cmds.nr_cmds);
	}
	mutex_unlock(&lcd->access_ok);

	return 0;
}

static int mmp_dsi_panel_disable(struct mmp_path *path)
{
	struct lcd *lcd = path->panel->plat_data;
	struct device *dev = path->panel->dev;

	mutex_lock(&lcd->access_ok);
	if (IS_ENABLED(CONFIG_OF) && lcd->disable_cmds.nr_cmds) {
		dev_info(dev, "send disable_cmds\n");
		mmp_phy_dsi_tx_cmd_array(path->phy,
				lcd->disable_cmds.cmds,
				lcd->disable_cmds.nr_cmds);
	}
	mutex_unlock(&lcd->access_ok);

	return 0;
}

#ifdef CONFIG_MMP_PANEL_BACKLIGHT
static int mmp_dsi_panel_set_brightness(struct mmp_panel *panel, int intensity)
{
	struct lcd *lcd = panel->plat_data;
	struct device *dev = panel->dev;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	dev_info(dev, "set brightness (%d)\n", intensity);

	mutex_lock(&lcd->access_ok);
	if (!lcd->active) {
		mutex_unlock(&lcd->access_ok);
		return 0;
	}

	if (IS_ENABLED(CONFIG_OF) && lcd->backlight_on_cmds.nr_cmds) {
		lcd->backlight_set_brightness_cmds.cmds[0].data[1] = intensity;
		mmp_phy_dsi_tx_cmd_array(path->phy,
				lcd->backlight_set_brightness_cmds.cmds,
				lcd->backlight_set_brightness_cmds.nr_cmds);
		mmp_phy_dsi_tx_cmd_array(path->phy,
				lcd->backlight_on_cmds.cmds,
				lcd->backlight_on_cmds.nr_cmds);
	}
	mutex_unlock(&lcd->access_ok);

	return intensity;
}

static const struct mmp_panel_backlight_ops backlight_ops = {
	.set_brightness = mmp_dsi_panel_set_brightness,
	.get_brightness = NULL,
};
#endif

static int mmp_dsi_panel_set_mdnie(struct mdnie_config *config)
{
	struct mmp_path *path =
		mmp_get_path(mmp_dsi_panel.plat_path_name);
	struct lcd *lcd = mmp_dsi_panel.plat_data;
	struct mdnie_lite *mdnie = &lcd->mdnie;
	struct mmp_dsi_cmds *cmds;

	mutex_lock(&lcd->access_ok);
	if (!lcd->active) {
		mutex_unlock(&lcd->access_ok);
		return 0;
	}

	if (config->tuning) {
		pr_info("%s: set tuning data\n", __func__);
		cmds = &mdnie->tuning_mode_cmds;
	} else if ((config->accessibility == NEGATIVE) || (config->negative)) {
		pr_info("%s: set negative\n", __func__);
		cmds = &mdnie->negative_mode_cmds;
	} else if (config->accessibility == COLOR_BLIND) {
		pr_info("%s: set color adjustment\n", __func__);
		cmds = &mdnie->color_adjustment_cmds;
	} else {
		pr_info("%s: set scenario(%d)\n", __func__, config->scenario);
		switch (config->scenario) {
		case MDNIE_CAMERA_MODE:
			cmds = &mdnie->camera_mode_cmds;
			break;
		case MDNIE_GALLERY_MODE:
			cmds = &mdnie->gallery_mode_cmds;
			break;
		case MDNIE_VIDEO_WARM_MODE:
		case MDNIE_VIDEO_COLD_MODE:
		case MDNIE_VIDEO_MODE:
			cmds = &mdnie->video_mode_cmds;
			break;
		case MDNIE_VT_MODE:
			cmds = &mdnie->vt_mode_cmds;
			break;
		case MDNIE_BROWSER_MODE:
			cmds = &mdnie->browser_mode_cmds;
			break;
		case MDNIE_EBOOK_MODE:
			cmds = &mdnie->ebook_mode_cmds;
			break;
		case MDNIE_EMAIL_MODE:
			cmds = &mdnie->email_mode_cmds;
			break;
		case MDNIE_UI_MODE:
		default:
			cmds = &mdnie->ui_mode_cmds;
			break;
		}
		if (!cmds->cmds) {
			pr_warn("%s: scenario(%d) not exist\n", __func__,
					config->scenario);
			cmds = &mdnie->ui_mode_cmds;
		}
	}

	mdnie->config = *config;
	mmp_phy_dsi_tx_cmd_array(path->phy, cmds->cmds, cmds->nr_cmds);
	mutex_unlock(&lcd->access_ok);

	return 1;
}

static const struct mdnie_ops mmp_panel_mdnie_ops = {
	.set_mdnie = mmp_dsi_panel_set_mdnie,
	.get_mdnie = NULL,
};

#ifdef CONFIG_OF
static struct ext_pin *
mmp_dsi_panel_find_ext_pin(struct ext_pins *pins, const char *name)
{
	int i;
	for (i = 0; i < pins->nr_pins; i++) {
		if (!strcmp(pins->pins[i].name, name))
			return &pins->pins[i];
	}

	return NULL;
}

static int mmp_dsi_panel_ext_pin_ctrl(struct mmp_panel *panel,
		struct ext_pin *pin, int on)
{
	int rc;

	if (on) {
		if (pin->type == EXT_PIN_REGULATOR) {
			if (!pin->supply) {
				dev_err(panel->dev, "invalid regulator(%s)\n",
						pin->name);
				return -EINVAL;
			}

			if (regulator_is_enabled(pin->supply))
				pr_info("regulator(%s) already enabled\n",
						pin->name);
			rc = regulator_enable(pin->supply);
			if (unlikely(rc)) {
				dev_err(panel->dev, "regulator(%s) enable failed\n",
						pin->name);
				return -EINVAL;
			}
		} else {
			if (!gpio_is_valid(pin->gpio)) {
				dev_err(panel->dev, "invalid gpio(%s:%d)\n",
						pin->name, pin->gpio);
				return -EINVAL;
			}
			gpio_direction_output(pin->gpio, 1);
		}
	} else {
		if (pin->type == EXT_PIN_REGULATOR) {
			if (!pin->supply) {
				dev_err(panel->dev, "invalid regulator(%s)\n",
						pin->name);
				return -EINVAL;
			}

			if (!regulator_is_enabled(pin->supply))
				pr_info("regulator(%s) already disabled\n",
						pin->name);
			rc = regulator_disable(pin->supply);
			if (unlikely(rc)) {
				dev_err(panel->dev, "regulator(%s) disable failed\n",
						pin->name);
				return -EINVAL;
			}
		} else {
			if (!gpio_is_valid(pin->gpio)) {
				dev_err(panel->dev, "invalid gpio(%s:%d)\n",
						pin->name, pin->gpio);
				return -EINVAL;
			}
			gpio_direction_output(pin->gpio, 0);
		}
	}

	return 0;
}

static int mmp_dsi_panel_set_pin_state(struct mmp_panel *panel, int on)
{
	struct lcd *lcd = panel->plat_data;
	struct pinctrl_state *pinctrl_state;
	int ret = 0;

	if (!lcd->pinctrl)
		return 0;

	pinctrl_state = on ? lcd->pin_enable : lcd->pin_disable;
	if (!pinctrl_state)
		return 0;

	ret = pinctrl_select_state(lcd->pinctrl, pinctrl_state);
	if (unlikely(ret)) {
		pr_err("%s: can't set pinctrl %s state\n",
				__func__, on ? "enable" : "disable");
	}
	return ret;
}

static void mmp_dsi_panel_power(struct mmp_panel *panel, int on)
{
	struct lcd *lcd = panel->plat_data;
	static struct ext_pin *pin;
	struct ext_pin_ctrl *pin_ctrl;
	size_t nr_pin_ctrl;
	int i;

	if (lcd->power == !!on) {
		pr_warn("%s: power already %s state\n",
				__func__, on ? "on" : "off");
		return;
	}

	if (on) {
		pin_ctrl = lcd->power_on.ctrls;
		nr_pin_ctrl = lcd->power_on.nr_ctrls;
	} else {
		pin_ctrl = lcd->power_off.ctrls;
		nr_pin_ctrl = lcd->power_off.nr_ctrls;
	}

	for (i = 0; i < nr_pin_ctrl; i++) {
		pin = mmp_dsi_panel_find_ext_pin(&lcd->pins,
				pin_ctrl[i].name);
		if (unlikely(!pin)) {
			dev_err(panel->dev, "external pin(%s) not found\n",
					pin_ctrl[i].name);
			return;
		}
		pr_info("%s %s %dusec\n",
				pin_ctrl[i].name,
				pin_ctrl[i].on ? "on" : "off",
				pin_ctrl[i].usec);
		mmp_dsi_panel_ext_pin_ctrl(panel, pin, pin_ctrl[i].on);
		if (pin_ctrl[i].usec < 20 * 1000)
			usleep_range(pin_ctrl[i].usec, pin_ctrl[i].usec + 10);
		else
			msleep(pin_ctrl[i].usec / 1000);
	}
	lcd->power = on;
	return;
}
#else
static void mmp_dsi_panel_power(struct mmp_panel *panel, int on) {}
#endif

#ifdef CONFIG_OF
static void of_set_panel_id(struct mmp_panel *panel)
{
	struct lcd *lcd = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);

	path->panel->id = (boot_panel_id) ? boot_panel_id :
		lcd->set_panel_id(panel);
	if (!(path->panel->id))
		pr_err("%s: panel id configuration is missing\n",
				__func__);
}
#else
static void of_set_panel_id(struct mmp_panel *panel) {}
#endif

static void mmp_dsi_panel_set_panel_id(struct mmp_panel *panel)
{
	struct lcd *lcd = panel->plat_data;

	if (lcd->pdata) {
		if (!lcd->pdata->panel_id) {
			if (boot_panel_id)
				lcd->pdata->panel_id = boot_panel_id;
			else if (lcd->set_panel_id)
				lcd->pdata->panel_id =
					lcd->set_panel_id(panel);
		}
	} else {
		if (!panel->id)
			of_set_panel_id(panel);
	}
}

static inline unsigned int mmp_dsi_panel_get_panel_id(struct mmp_panel *panel)
{
	struct lcd *lcd = panel->plat_data;

	return lcd->pdata ? lcd->pdata->panel_id : panel->id;
}

static bool mmp_dsi_panel_is_connected(struct mmp_panel *panel)
{
	struct lcd *lcd = panel->plat_data;

	/* Check Panel ID */
	if (mmp_dsi_panel_get_panel_id(panel))
		return true;

	/* Check VGH PIN */
	/*
	if (lcd->esd_en)
		return esd_check_vgh_on(&panel->esd);
	*/

	return false;
}

static void mmp_dsi_panel_onoff(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel->plat_data;
	struct mmp_path *path = mmp_get_path(panel->plat_path_name);
	pr_info("called %s with status %d\n", __func__, status);
	BUG_ON(!path);
	if (status) {
		/* power on */
		mmp_dsi_panel_set_pin_state(panel, 1);
		mmp_dsi_panel_power(panel, 1);
		mmp_dsi_panel_set_panel_id(panel);
		if (!mmp_dsi_panel_is_connected(panel)) {
			if (mmp_panel_backlight_is_on(panel->bd))
				mmp_panel_backlight_onoff(panel->bd, 0);
			pr_warn("%s: no panel\n", __func__);
			goto panel_off;
		}
		mmp_dsi_panel_enable(path);
		mutex_lock(&lcd->access_ok);
		lcd->active = true;
		mutex_unlock(&lcd->access_ok);
	} else {
		/* power off */
panel_off:
		mutex_lock(&lcd->access_ok);
		lcd->active = false;
		mutex_unlock(&lcd->access_ok);
		mmp_dsi_panel_disable(path);
		mmp_dsi_panel_power(panel, 0);
		mmp_dsi_panel_set_pin_state(panel, 0);
	}
}

static void mmp_dsi_panel_reduced_onoff(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel->plat_data;

	pr_info("called %s with status %d\n", __func__, status);
	if (status) {
		/* power on */
		mmp_dsi_panel_set_pin_state(panel, 1);
		mmp_dsi_panel_power(panel, 1);
		mmp_dsi_panel_set_panel_id(panel);
		mutex_lock(&lcd->access_ok);
		lcd->active = true;
		mutex_unlock(&lcd->access_ok);
		if (!mmp_dsi_panel_is_connected(panel)) {
			if (mmp_panel_backlight_is_on(panel->bd))
				mmp_panel_backlight_onoff(panel->bd, 0);
			pr_warn("%s: no panel\n", __func__);
			goto panel_reduced_off;
		}
	} else {
		/* power off */
panel_reduced_off:
		mutex_lock(&lcd->access_ok);
		lcd->active = false;
		mutex_unlock(&lcd->access_ok);
		mmp_dsi_panel_power(panel, 0);
		mmp_dsi_panel_set_pin_state(panel, 0);
	}
}

static void mmp_dsi_panel_start(struct mmp_panel *panel, int status)
{
	struct lcd *lcd = panel->plat_data;

	if (!lcd->active) {
		pr_warn("%s: no panel\n", __func__);
		return;
	}

	if (status) {
		update_mdnie_mode(&lcd->mdnie);
		/* after video on, wait more than 2 frames */
		msleep(33);
	}

	if ((!!status) != mmp_panel_backlight_is_on(panel->bd))
		mmp_panel_backlight_onoff(panel->bd, status);
}

static void mmp_dsi_panel_esd_onoff(struct mmp_panel *panel, int status)
{
	struct lcd *plat = panel->plat_data;

	if (!plat->esd_en)
		return;

	if (status)
		esd_start(&panel->esd);
	else
		esd_stop(&panel->esd);
}

static int mmp_dsi_panel_get_status(struct mmp_panel *panel)
{
	return ESD_STATUS_OK;
}

static void mmp_dsi_panel_esd_recover(struct mmp_panel *panel)
{
	struct mmp_path *path =
		mmp_get_path(panel->plat_path_name);

	esd_panel_recover(path);
	pr_info("%s done\n", __func__);
}

static struct mmp_mode mmp_dsi_panel_modes[] = {
	[0] = {
		.refresh = 60,
		.invert_pixclock = 0,
		.pix_fmt_out = PIXFMT_RGB888PACK,
		.hsync_invert = 0,
		.vsync_invert = 0,
	},
};

static int mmp_dsi_panel_get_modelist(struct mmp_panel *panel,
		struct mmp_mode **modelist)
{
	*modelist = mmp_dsi_panel_modes;
	return 1;
}

static struct mmp_panel mmp_dsi_panel = {
	.name = "dummy panel",
	.panel_type = PANELTYPE_DSI_VIDEO,
	.get_modelist = mmp_dsi_panel_get_modelist,
	.set_onoff = mmp_dsi_panel_onoff,
	.reduced_onoff = mmp_dsi_panel_reduced_onoff,
	.panel_start = mmp_dsi_panel_start,
	.panel_esd_recover = mmp_dsi_panel_esd_recover,
	.get_status = mmp_dsi_panel_get_status,
	.esd_set_onoff = mmp_dsi_panel_esd_onoff,
};

static ssize_t mmp_dsi_panel_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", mmp_dsi_panel.name);
}

static DEVICE_ATTR(mmp_dsi_panel_name, S_IRUGO | S_IXOTH,
		mmp_dsi_panel_name_show, NULL);

static ssize_t lcd_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	const char *manufacturer_name = "INH";
	u32 panel_id;
#ifdef CONFIG_OF
	struct mmp_path *path =
		mmp_get_path(mmp_dsi_panel.plat_path_name);
	struct lcd *lcd = mmp_dsi_panel.plat_data;
	panel_id = path->panel->id;
#else
	struct lcd *lcd = dev_get_drvdata(dev);
	panel_id = lcd->pdata->panel_id;
#endif
	if (lcd->manufacturer_name)
		manufacturer_name = lcd->manufacturer_name;
	if (lcd->panel_model_name) {
		return sprintf(buf, "%s_%s",
				manufacturer_name,
				lcd->panel_model_name);
	}

	return sprintf(buf, "%s_%x%x%x",
			manufacturer_name,
			(panel_id >> 16) & 0xFF,
			(panel_id >> 8) & 0xFF,
			panel_id  & 0xFF);
}

static DEVICE_ATTR(lcd_type, S_IRUGO | S_IXOTH,
		lcd_type_show, NULL);

#ifdef CONFIG_OF
static inline int of_property_read_u32_with_suffix(const struct device_node *np,
		const char *propname, const char *suffix, u32 *out_value)
{
	char *name;
	size_t len;
	int ret;

	len = strlen(propname) + strlen(suffix) + 1;
	name = kzalloc(sizeof(char) * len, GFP_KERNEL);
	if (unlikely(!name))
		return -ENOMEM;

	snprintf(name, len, "%s%s", propname, suffix);
	ret = of_property_read_u32(np, name, out_value);
	if (unlikely(ret < 0))
		pr_err("%s: failed to read property(%s)\n",
				__func__, name);

	kfree(name);
	return ret;
}

static inline int of_property_read_string_with_suffix(struct device_node *np,
		const char *propname, const char *suffix,
		const char **out_string)
{
	char *name;
	size_t len;
	int ret;

	len = strlen(propname) + strlen(suffix) + 1;
	name = kzalloc(sizeof(char) * len, GFP_KERNEL);
	if (unlikely(!name))
		return -ENOMEM;

	snprintf(name, len, "%s%s", propname, suffix);
	ret = of_property_read_string(np, name, out_string);
	if (unlikely(ret))
		pr_err("%s: failed to read property(%s)\n",
				__func__, name);

	kfree(name);
	return ret;
}

static struct device_node *mmp_dsi_find_panel_of_node(
		struct platform_device *pdev, char *panel_cfg)
{
	struct device_node *panel_node = NULL;
	struct device_node *mmp_dsi_panel;
	struct device *dev = &pdev->dev;
	char *panel_name;
	u32 panel_id;

	/*
	 * priority of panel node being found
	 * 1. corresponding name of panel nodes with panel_cfg.
	 * 2. corresponding panel id of mmp_dsi_panel
	 * 3. first one of child nodes of mmp_dsi_panel.
	 */
	mmp_dsi_panel = of_parse_phandle(pdev->dev.of_node,
			"marvell,dsi-panel", 0);
	if (!mmp_dsi_panel) {
		dev_err(dev, "%s: mmp_dsi_panel not found\n", __func__);
		return NULL;
	}

	if (panel_cfg && strlen(panel_cfg)) {
		panel_name = panel_cfg + 2;
		panel_node = of_find_node_by_name(mmp_dsi_panel, panel_name);
		if (panel_node)
			dev_info(dev, "found (%s) by name\n", panel_node->name);
	} else {
		struct device_node *np;
		int ret;

		/* Find a node corresponding panel id */
		for_each_child_of_node(mmp_dsi_panel, np) {
			ret = of_property_read_u32(np,
					"marvell,dsi-panel-id", &panel_id);
			if (boot_panel_id && panel_id) {
				if (boot_panel_id == panel_id) {
					panel_node = np;
					dev_info(dev,
						"found (%s) by id(0x%X)\n",
						panel_node->name, panel_id);
					break;
				}
			}
		}

		if (!panel_node) {
			panel_node = of_get_next_child(mmp_dsi_panel, NULL);
			if (panel_node)
				dev_info(dev, "found (%s) by node order\n",
						panel_node->name);
		}
	}
	of_node_put(mmp_dsi_panel);
	if (!panel_node) {
		dev_err(dev, "%s: panel_node not found\n", __func__);
		return NULL;
	}
	return panel_node;
}

static int mmp_dsi_parse_dt_dcs_cmds(struct device_node *np,
		const char *propname, struct mmp_dsi_cmds *dsi_cmds)
{
	struct mmp_dsi_cmd_hdr *dchdr;
	struct mmp_dsi_cmd_desc *cmds;
	const char *val, *txmode;
	char *data;
	int sz = 0, i = 0, nr_cmds = 0;
	int nr, ret;
	u8 mode = DSI_HS_MODE;

	if (!dsi_cmds) {
		pr_err("%s: invalid dsi_cmds address\n", __func__);
		return -EINVAL;
	}

	if (!of_find_property(np, propname, NULL)) {
		pr_debug("%s: %s is not exist\n", __func__, propname);
		return -EINVAL;
	}

	ret = of_property_read_string_with_suffix(np,
			propname, "-txmode", &txmode);
	if (unlikely(ret))
		return -EINVAL;

	if (!strncmp(txmode, tx_modes[DSI_LP_MODE],
				strlen(tx_modes[DSI_LP_MODE])))
		mode = DSI_LP_MODE;
	else
		mode = DSI_HS_MODE;

	val = of_get_property(np, propname, &sz);
	if (unlikely(!val)) {
		pr_err("%s: failed, key=%s\n", __func__, propname);
		return -ENOMEM;
	}

	data = kzalloc(sizeof(char) * sz , GFP_KERNEL);
	if (unlikely(!data))
		return -ENOMEM;

	memcpy(data, val, sz);

	/* scan dcs commands */
	while (sz > i + sizeof(*dchdr)) {
		dchdr = (struct mmp_dsi_cmd_hdr *)(data + i);
		dchdr->dlen = ntohs(dchdr->dlen);
		dchdr->wait = ntohs(dchdr->wait);
		if (i + dchdr->dlen > sz) {
			pr_err("%s: parse error dtsi cmd=0x%02x, len=0x%02x, nr_cmds=%d",
				__func__, dchdr->dtype, dchdr->dlen, nr_cmds);
			dchdr = NULL;
			kfree(data);
			return -ENOMEM;
		}
		i += sizeof(*dchdr) + dchdr->dlen;
		nr_cmds++;
	}

	if (unlikely(sz != i)) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, data[0], sz);
		kfree(data);
		return -ENOMEM;
	}

	dsi_cmds->cmds = kzalloc(nr_cmds * sizeof(struct mmp_dsi_cmd_desc),
						GFP_KERNEL);
	dsi_cmds->nr_cmds = nr_cmds;
	if (unlikely(!dsi_cmds->cmds)) {
		kfree(data);
		return -ENOMEM;
	}

	cmds = dsi_cmds->cmds;
	for (i = 0, nr = 0; nr < nr_cmds; nr++) {
		dchdr = (struct mmp_dsi_cmd_hdr *)(data + i);
		cmds[nr].data_type = dchdr->dtype;
		cmds[nr].lp = mode;
		cmds[nr].delay = dchdr->wait;
		cmds[nr].length = dchdr->dlen;
		cmds[nr].data = &data[i + sizeof(*dchdr)];
		i += sizeof(*dchdr) + dchdr->dlen;
		pr_info("type:%x, %s, %d ms, %d bytes, %02Xh\n",
				cmds[nr].data_type, tx_modes[cmds[nr].lp],
				cmds[nr].delay, cmds[nr].length,
				(cmds[nr].data)[0]);
	}
	pr_info("parse %s done!\n", propname);
	return 0;
}

static int mmp_dsi_panel_parse_pinctrl(
		struct platform_device *pdev, struct lcd *lcd)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state;

	if (unlikely(!pdev))
		return -ENODEV;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl))
		return 0;

	lcd->pinctrl = pinctrl;
	pinctrl_state = pinctrl_lookup_state(lcd->pinctrl, "enable");
	if (!IS_ERR(pinctrl_state)) {
		dev_info(&pdev->dev, "%s: found pinctrl enable\n", __func__);
		lcd->pin_enable = pinctrl_state;
	}

	pinctrl_state = pinctrl_lookup_state(lcd->pinctrl, "disable");
	if (!IS_ERR(pinctrl_state)) {
		dev_info(&pdev->dev, "%s: found pinctrl disable\n", __func__);
		lcd->pin_disable = pinctrl_state;
	}

	if ((!lcd->pin_enable && lcd->pin_disable) ||
			(lcd->pin_enable && !lcd->pin_disable)) {
		dev_warn(&pdev->dev, "%s: warning - pinctrl %s not exist\n",
				__func__,
				!lcd->pin_enable ? "enable" : "disable");
	}

	return 0;
}

static int mmp_dsi_panel_parse_dt_esd(
		struct device_node *np, struct lcd *lcd)
{
	int ret;

	if (!np)
		return -EINVAL;

	if (!of_property_read_bool(np, "marvell,panel-esd-en"))
		return 0;

	lcd->esd_gpio = of_get_named_gpio(np,
			"marvell,panel-esd-gpio", 0);
	if (unlikely(lcd->esd_gpio < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n",
				__func__, lcd->esd_gpio);
		return -EINVAL;
	}
	ret = of_property_read_u32(np, "marvell,panel-esd-type",
			&lcd->esd_type);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to read property(%s)\n",
				__func__, "marvell,panel-esd-type");
		return -EINVAL;
	}
	lcd->esd_en = true;
	return 0;
}

static int mmp_dsi_panel_parse_dt_temp_compensation(
		struct device_node *parent, struct lcd *lcd)
{
	struct temp_compensation *temp_comp;
	struct device_node *temp_comp_node = NULL;
	int nr_nodes = 0, i = 0, ret = 0;
	u32 data_len, temperature;

	if (lcd->temp_comp) {
		pr_warn("%s: temperature compensation already exist\n",
				__func__);
		return 0;
	}
	nr_nodes = of_get_child_count(parent);
	temp_comp = kmalloc(sizeof(struct temp_compensation) * nr_nodes,
			GFP_KERNEL);
	if (unlikely(!temp_comp)) {
		ret = -ENOMEM;
		goto err_temp_compensation;
	};

	for_each_child_of_node(parent, temp_comp_node) {
		pr_info("found (%s)\n", temp_comp_node->name);
		ret = of_property_read_u32(temp_comp_node, "trig-type",
				&temp_comp[i].trig_type);
		if (unlikely(ret < 0)) {
			pr_err("%s: failed to read property(%s)\n",
					__func__, "trig-type");
			ret = -EINVAL;
			goto err_temp_compensation;
		}
		ret = of_property_read_u32(temp_comp_node, "temperature",
				&temperature);
		if (unlikely(ret < 0)) {
			pr_err("%s: failed to read property(%s)\n",
					__func__, "temperature");
			ret = -EINVAL;
			goto err_temp_compensation;
		}
		if (temperature) {
			temp_comp[i].temperature = temperature - 273;
			pr_info("%s: set temperature threshold (%d)\n",
					__func__, temp_comp[i].temperature);
		} else {
			pr_warn("%s: temperature might be not initialized\n",
					__func__);
		}
		if (of_find_property(temp_comp_node, "old-data", &data_len)) {
			temp_comp[i].old_data = kzalloc(sizeof(char) *
					data_len, GFP_KERNEL);
			of_property_read_u8_array(temp_comp_node, "old-data",
					temp_comp[i].old_data, data_len);
			temp_comp[i].data_len = data_len;
		} else {
			pr_err("%s: failed to read property(%s)\n",
					__func__, "old-data");
			ret = -EINVAL;
			goto err_temp_compensation;
		}
		if (of_find_property(temp_comp_node, "new-data", &data_len)) {
			temp_comp[i].new_data = kzalloc(sizeof(char) *
					temp_comp[i].data_len, GFP_KERNEL);
			of_property_read_u8_array(temp_comp_node, "new-data",
					temp_comp[i].new_data, data_len);
			temp_comp[i].data_len = data_len;
		} else {
			pr_err("%s: failed to read property(%s)\n",
					__func__, "new-data");
			ret = -EINVAL;
			goto err_temp_compensation;
		}
		i++;
	}

	lcd->temp_comp = temp_comp;
	lcd->nr_temp_comp = nr_nodes;
	lcd->temp_comp_en = true;
	return 0;

err_temp_compensation:
	if (temp_comp) {
		for (i = 0; i < nr_nodes; i++) {
			kfree(temp_comp[i].new_data);
			kfree(temp_comp[i].old_data);
		}
		kfree(temp_comp);
	}
	return ret;
}

static int mmp_dsi_panel_parse_dt_panel(
		struct device_node *np, struct lcd *lcd)
{
	const char *panel_type, *addr;
	static struct mmp_mode *panel_mode =
		&mmp_dsi_panel_modes[0];
	struct device_node *temp_comp_node;
	u32 tmp, panel_id;
	size_t sz;
	int ret;

	if (!np)
		return -EINVAL;

	ret = of_property_read_string(np, "marvell,dsi-panel-manu",
			&lcd->manufacturer_name);
	ret = of_property_read_string(np, "marvell,dsi-panel-model",
			&lcd->panel_model_name);
	ret = of_property_read_string(np, "marvell,dsi-panel-name",
			&mmp_dsi_panel.name);
	ret = of_property_read_u32(np, "marvell,dsi-panel-id",
			&panel_id);
	ret = of_property_read_string(np, "marvell,dsi-panel-type",
			&panel_type);
	ret = of_property_read_u32(np, "marvell,dsi-panel-xres",
			&panel_mode->xres);
	ret = of_property_read_u32(np, "marvell,dsi-panel-yres",
			&panel_mode->yres);
	ret = of_property_read_u32(np, "marvell,dsi-panel-xres",
			&panel_mode->real_xres);
	ret = of_property_read_u32(np, "marvell,dsi-panel-yres",
			&panel_mode->real_yres);
	ret = of_property_read_u32(np, "marvell,dsi-panel-width",
			&panel_mode->width);
	ret = of_property_read_u32(np, "marvell,dsi-panel-height",
			&panel_mode->height);
	ret = of_property_read_u32(np, "marvell,dsi-h-front-porch",
			&panel_mode->right_margin);
	ret = of_property_read_u32(np, "marvell,dsi-h-back-porch",
			&panel_mode->left_margin);
	ret = of_property_read_u32(np, "marvell,dsi-h-sync-width",
			&panel_mode->hsync_len);
	ret = of_property_read_u32(np, "marvell,dsi-v-front-porch",
			&panel_mode->lower_margin);
	ret = of_property_read_u32(np, "marvell,dsi-v-back-porch",
			&panel_mode->upper_margin);
	ret = of_property_read_u32(np, "marvell,dsi-v-sync-width",
			&panel_mode->vsync_len);

	panel_mode->pixclock_freq = panel_mode->refresh *
		(panel_mode->xres + panel_mode->right_margin +
		 panel_mode->left_margin + panel_mode->hsync_len) *
		(panel_mode->yres + panel_mode->upper_margin +
		 panel_mode->lower_margin + panel_mode->vsync_len);

	ret = of_property_read_u32(np, "marvell,dsi-hsync-invert", &tmp);
	ret = of_property_read_u32(np, "marvell,dsi-vsync-invert", &tmp);
	ret = of_property_read_u32(np, "marvell,dsi-lanes", &tmp);
	ret = of_property_read_u32(np, "marvell,dsi-burst-mode", &tmp);
	if (of_property_read_bool(np, "marvell,dsi-master-mode"))
		pr_debug("%s, master mode\n", __func__);
	else
		pr_debug("%s, slave mode\n", __func__);
	if (of_property_read_bool(np, "marvell,dsi-hbp-en"))
		pr_debug("%s, hbp_en\n", __func__);
	if (of_property_read_bool(np, "marvell,dsi-hfp-en"))
		pr_debug("%s, hfp_en\n", __func__);

	mmp_dsi_parse_dt_dcs_cmds(np, "marvell,dsi-panel-init-cmds",
			&lcd->init_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np, "marvell,dsi-panel-enable-cmds",
			&lcd->enable_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np, "marvell,dsi-panel-disable-cmds",
			&lcd->disable_cmds);
#ifdef CONFIG_MMP_PANEL_BACKLIGHT
	mmp_dsi_parse_dt_dcs_cmds(np, "marvell,dsi-panel-backlight-on-cmds",
			&lcd->backlight_on_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-backlight-set-brightness-cmds",
			&lcd->backlight_set_brightness_cmds);
#endif

	addr = of_get_property(np, "marvell,dsi-panel-read-id-regs", &sz);
	if (unlikely(!addr)) {
		pr_err("%s: failed to parse read-id-regs\n", __func__);
		return -EINVAL;
	}
	memcpy(lcd->id_regs, addr, sz);
	of_property_read_u8(np, "marvell,dsi-panel-read-status-regs",
			&lcd->status_reg);
	of_property_read_u8(np, "marvell,dsi-panel-read-status-ok",
			&lcd->status_ok);

	temp_comp_node = of_find_node_by_name(np,
			"marvell,dsi-panel-temp-compensation");
	if (temp_comp_node)
		mmp_dsi_panel_parse_dt_temp_compensation(temp_comp_node, lcd);
	of_node_put(temp_comp_node);

	return 0;
}

static int mmp_dsi_panel_parse_dt_mdnie(
		struct device_node *np, struct mdnie_lite *mdnie)
{
	if (!np || !mdnie)
		return -EINVAL;

	if (!of_property_read_bool(np,
				"marvell,dsi-panel-mdnie-support")) {
		mdnie->support = false;
		return 0;
	}
	if (of_property_read_bool(np,
				"marvell,dsi-panel-mdnie-not-support")) {
		mdnie->support = false;
		return 0;
	}

	of_property_read_u8(np,
			"marvell,dsi-panel-mdnie-cmd-reg",
			&mdnie->cmd_reg);
	of_property_read_u32(np,
			"marvell,dsi-panel-mdnie-cmd-len",
			&mdnie->cmd_len);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-ui-mode-cmds",
			&mdnie->ui_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-video-mode-cmds",
			&mdnie->video_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-camera-mode-cmds",
			&mdnie->camera_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-gallery-mode-cmds",
			&mdnie->gallery_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-negative-mode-cmds",
			&mdnie->negative_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-vt-mode-cmds",
			&mdnie->vt_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-browser-mode-cmds",
			&mdnie->browser_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-ebook-mode-cmds",
			&mdnie->ebook_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-email-mode-cmds",
			&mdnie->email_mode_cmds);
	mmp_dsi_parse_dt_dcs_cmds(np,
			"marvell,dsi-panel-mdnie-color-adjustment-mode-cmds",
			&mdnie->color_adjustment_cmds);
	mdnie->support = true;
	return 0;
}

static int mmp_dsi_panel_parse_dt_pin(struct device_node *np,
		const char *propname, struct ext_pin *pin)
{
	size_t len;
	int ret;

	ret = of_property_read_string_with_suffix(np,
			propname, "-name", &pin->name);
	if (unlikely(ret))
		return -EINVAL;

	ret = of_property_read_u32_with_suffix(np,
			propname, "-type", &pin->type);
	if (unlikely(ret))
		return -EINVAL;

	if (pin->type == EXT_PIN_REGULATOR) {
		struct device_node *node;
		struct regulator *supply;
		char *supply_name;

		len = strlen(propname) + strlen("-supply") + 1;
		supply_name = kmalloc(sizeof(char) * len, GFP_KERNEL);
		if (unlikely(!supply_name))
			return -ENOMEM;
		snprintf(supply_name, len, "%s-supply", propname);

		node = of_parse_phandle(np, supply_name, 0);
		if (!node) {
			pr_err("%s: failed to parse %s\n",
					__func__, supply_name);
			kfree(supply_name);
			return -EINVAL;
		}
		supply = regulator_get(NULL, node->name);
		if (IS_ERR_OR_NULL(supply)) {
			pr_err("%s regulator(%s) get error!\n",
					__func__, node->name);
			of_node_put(node);
			kfree(supply_name);
			regulator_put(supply);
			return -EINVAL;
		}
		pin->supply = supply;
		if (regulator_is_enabled(pin->supply)) {
			ret = regulator_enable(pin->supply);
			if (unlikely(ret)) {
				pr_err("regulator(%s) enable failed\n",
					pin->name);
			}
		}
		pr_info("%s, get regulator(%s)\n", pin->name, node->name);
		of_node_put(node);
		kfree(supply_name);
	} else {
		char *gpio_name;
		int gpio;

		len = strlen(propname) + strlen("-gpio") + 1;
		gpio_name = kmalloc(sizeof(char) * len, GFP_KERNEL);
		if (unlikely(!gpio_name))
			return -ENOMEM;
		snprintf(gpio_name, len, "%s-gpio", propname);

		gpio = of_get_named_gpio(np, gpio_name, 0);
		if (unlikely(gpio < 0)) {
			pr_err("%s: of_get_named_gpio failed: %d\n",
					__func__, gpio);
			kfree(gpio_name);
			return -EINVAL;
		}

		ret = gpio_request(gpio, pin->name);
		if (unlikely(ret)) {
			pr_err("%s: gpio_request failed: %d\n",
					__func__, ret);
			kfree(gpio_name);
		}
		pin->gpio = gpio;
		pr_info("%s, get gpio(%d)\n", pin->name, pin->gpio);
		kfree(gpio_name);
	}

	return 0;
}

static int mmp_dsi_panel_parse_dt_pin_ctrl(struct device_node *np,
		const char *propname,
		struct ext_pin_ctrls *out_ctrl)
{
	static struct ext_pin_ctrl_dt {
		phandle phandle;
		u32 on;
		u32 usec;
	} *ctrl_dt;
	struct ext_pin_ctrl *tctrl;
	struct device_node *node;
	u32 *arr;
	size_t size, nr_arr, nr_ctrl_dt;
	int i, ret;

	of_get_property(np, propname, &size);
	arr = kmalloc(sizeof(u32) * size, GFP_KERNEL);
	if (!arr)
		return -ENOMEM;

	nr_arr = size / sizeof(u32);
	of_property_read_u32_array(np, propname, arr, nr_arr);
	ctrl_dt = (struct ext_pin_ctrl_dt *)arr;
	nr_ctrl_dt = size / sizeof(struct ext_pin_ctrl_dt);

	tctrl = kzalloc(sizeof(struct ext_pin_ctrl) * nr_ctrl_dt , GFP_KERNEL);
	if (!tctrl)
		return -ENOMEM;

	for (i = 0; i < nr_ctrl_dt; i++) {
		node = of_find_node_by_phandle(ctrl_dt[i].phandle);
		if (unlikely(!node)) {
			pr_err("%s: failed to find node(%s)\n",
					__func__, propname);
			goto err_find_node;
		}
		ret = of_property_read_string(node,
				"panel-ext-pin-name", &tctrl[i].name);
		if (unlikely(ret)) {
			pr_err("%s: failed to read property(%s)\n",
					__func__, propname);
			goto err_find_node;
		}
		tctrl[i].on = ctrl_dt[i].on;
		tctrl[i].usec = ctrl_dt[i].usec;
		pr_info("[%d] %s, %s, %dusec\n", i, tctrl[i].name,
				tctrl[i].on ? "on" : "off", tctrl[i].usec);
		of_node_put(node);
	}
	out_ctrl->ctrls = tctrl;
	out_ctrl->nr_ctrls = nr_ctrl_dt;
	kfree(arr);
	return 0;

err_find_node:
	of_node_put(node);
	kfree(tctrl);
	kfree(arr);
	return -EINVAL;
}

static int mmp_dsi_panel_parse_external_pins(
		struct device_node *pin_node, struct lcd *lcd)
{
	struct device_node *np = NULL;
	struct ext_pin *pins;
	int nr_nodes = 0, i = 0;
	int ret;
	char *name;
	u32 len;

	if (!pin_node)
		return -EINVAL;

	len = strlen(pin_node->name) + strlen("-on") + 1;
	name = kmalloc(sizeof(char) * len, GFP_KERNEL);
	if (unlikely(!name))
		return -ENOMEM;
	snprintf(name, len, "%s-on", pin_node->name);
	mmp_dsi_panel_parse_dt_pin_ctrl(pin_node, name, &lcd->power_on);
	kfree(name);

	len = strlen(pin_node->name) + strlen("-off") + 1;
	name = kmalloc(sizeof(char) * len, GFP_KERNEL);
	if (unlikely(!name))
		return -ENOMEM;
	snprintf(name, len, "%s-off", pin_node->name);
	mmp_dsi_panel_parse_dt_pin_ctrl(pin_node, name, &lcd->power_off);
	kfree(name);

	nr_nodes = of_get_child_count(pin_node);
	pins = kmalloc(sizeof(struct ext_pin) * nr_nodes, GFP_KERNEL);
	if (unlikely(!pins))
		return -ENOMEM;

	for_each_child_of_node(pin_node, np) {
		ret = mmp_dsi_panel_parse_dt_pin(np,
				pin_node->name, &pins[i++]);
		if (unlikely(ret < 0)) {
			pr_err("%s: failed to parse %s node\n",
					__func__, pin_node->name);
			kfree(pins);
			return -EINVAL;
		}
	}
	lcd->pins.pins = pins;
	lcd->pins.nr_pins = nr_nodes;

	return 0;
}
#endif

static int mmp_dsi_panel_probe(struct platform_device *pdev)
{
	struct device_node *panel_node, *pin_node, *backlight_node;
	struct lcd *lcd;
	int ret;

	pr_debug("called %s\n", __func__);

	lcd = kzalloc(sizeof(*lcd), GFP_KERNEL);
	if (unlikely(!lcd))
		return -ENOMEM;

	if (IS_ENABLED(CONFIG_OF)) {
		ret = of_property_read_string(pdev->dev.of_node,
				"marvell,path-name",
				&mmp_dsi_panel.plat_path_name);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "fail to parse path-name\n");
			goto err_no_platform_data;
		}

		ret = mmp_dsi_panel_parse_pinctrl(pdev, lcd);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "fail to parse pinctrl\n");
			goto err_no_platform_data;
		}

		/* parse dt of dsi-esd */
		ret = mmp_dsi_panel_parse_dt_esd(pdev->dev.of_node, lcd);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "fail to parse dsi-esd\n");
			goto err_no_platform_data;
		}

		/* parse dt of external pin node */
		pin_node = of_find_node_by_name(pdev->dev.of_node,
				"panel-ext-pin");
		if (unlikely(!pin_node)) {
			dev_err(&pdev->dev, "not found pin_node!\n");
			goto err_no_platform_data;
		}
		ret = mmp_dsi_panel_parse_external_pins(pin_node, lcd);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "failed to parse pin_node\n");
			goto err_no_platform_data;
		}
		of_node_put(pin_node);

		/* parse dt of backlight */
		backlight_node = mmp_panel_find_dt_backlight(pdev,
				"marvell,panel-backlight");
		if (backlight_node) {
			mmp_dsi_panel.bd =
				of_find_backlight_by_node(backlight_node);
			of_node_put(backlight_node);
		}

		/* parse dt of panel node */
		panel_node = mmp_dsi_find_panel_of_node(pdev, NULL);
		if (unlikely(!panel_node)) {
			dev_err(&pdev->dev, "not found panel_node!\n");
			goto err_no_platform_data;
		}
		ret = mmp_dsi_panel_parse_dt_panel(panel_node, lcd);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "failed to parse panel_node\n");
			of_node_put(panel_node);
			goto err_no_platform_data;
		}
		ret = mmp_dsi_panel_parse_dt_mdnie(panel_node, &lcd->mdnie);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "failed to parse mdnie\n");
			of_node_put(panel_node);
			goto err_no_platform_data;
		}
		of_node_put(panel_node);
	} else {
		if (unlikely(pdev->dev.platform_data == NULL)) {
			dev_err(&pdev->dev, "no platform data!\n");
			ret = -EINVAL;
			goto err_no_platform_data;
		}
		lcd->pdata = pdev->dev.platform_data;
		mmp_dsi_panel.plat_path_name = lcd->pdata->mi->plat_path_name;
	}
	mmp_dsi_panel.plat_data = lcd;
	lcd->set_panel_id = &set_panel_id;
	lcd->power = get_skip_power_on();
	mmp_dsi_panel.dev = &pdev->dev;

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
	mutex_init(&lcd->access_ok);

	ret = device_create_file(lcd->dev, &dev_attr_mmp_dsi_panel_name);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_mmp_dsi_panel_name.attr.name);
		goto err_lcd_device;
	}
	ret = device_create_file(lcd->dev, &dev_attr_lcd_type);
	if (unlikely(ret < 0)) {
		pr_err("Failed to create device file(%s)!\n",
				dev_attr_lcd_type.attr.name);
		goto err_lcd_name;
	}

	lcd->panel = &mmp_dsi_panel;
	if (lcd->mdnie.support)
		mmp_panel_attach_mdnie(&lcd->mdnie, &mmp_panel_mdnie_ops);
#ifdef CONFIG_MMP_PANEL_BACKLIGHT
	if (lcd->panel->bd)
		mmp_panel_attach_backlight(lcd->panel, &backlight_ops);
#endif
	mmp_register_panel(&mmp_dsi_panel);
	if (lcd->esd_en) {
		lcd->panel->esd_type = lcd->esd_type;
		lcd->panel->esd_gpio = lcd->esd_gpio;
		esd_init(&mmp_dsi_panel);
	}
	return 0;

err_lcd_name:
	device_remove_file(lcd->dev, &dev_attr_mmp_dsi_panel_name);
err_lcd_device:
	device_destroy(lcd->lcd_class, 0);
err_device_create:
	class_destroy(lcd->lcd_class);
err_class_create:
err_no_platform_data:
	kfree(lcd);

	return ret;
}

static int mmp_dsi_panel_remove(struct platform_device *dev)
{
	struct lcd *lcd = mmp_dsi_panel.plat_data;
	int i;

	if (lcd->mdnie.support)
		mmp_panel_detach_mdnie(&lcd->mdnie);
	device_remove_file(lcd->dev, &dev_attr_mmp_dsi_panel_name);
	device_remove_file(lcd->dev, &dev_attr_lcd_type);
	device_destroy(lcd->lcd_class, 0);
	class_destroy(lcd->lcd_class);
	mmp_unregister_panel(&mmp_dsi_panel);

	kfree(lcd->pins.pins);
	kfree(lcd->power_on.ctrls);
	kfree(lcd->power_off.ctrls);
	kfree(lcd->init_cmds.cmds->data);
	kfree(lcd->init_cmds.cmds);
	kfree(lcd->enable_cmds.cmds->data);
	kfree(lcd->enable_cmds.cmds);
	kfree(lcd->disable_cmds.cmds->data);
	kfree(lcd->disable_cmds.cmds);
#ifdef CONFIG_MMP_PANEL_BACKLIGHT
	if (lcd->panel->bd) {
		mmp_panel_detach_backlight(lcd->panel);
		kfree(lcd->backlight_on_cmds.cmds->data);
		kfree(lcd->backlight_on_cmds.cmds);
		kfree(lcd->backlight_set_brightness_cmds.cmds->data);
		kfree(lcd->backlight_set_brightness_cmds.cmds);
	}
#endif
	if (lcd->temp_comp) {
		for (i = 0; i < lcd->nr_temp_comp; i++) {
			kfree(lcd->temp_comp[i].new_data);
			kfree(lcd->temp_comp[i].old_data);
		}
		kfree(lcd->temp_comp);
	}
	kfree(lcd);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_dsi_panel_dt_match[] = {
	{ .compatible = "marvell,mmp-dsi-panel" },
	{},
};
#endif
static struct platform_driver mmp_dsi_panel_driver = {
	.driver     = {
		.name   = "mmp-dsi-panel",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mmp_dsi_panel_dt_match),
#endif
	},
	.probe      = mmp_dsi_panel_probe,
	.remove     = mmp_dsi_panel_remove,
};

static int mmp_dsi_panel_module_init(void)
{
	return platform_driver_register(&mmp_dsi_panel_driver);
}
static void mmp_dsi_panel_module_exit(void)
{
	platform_driver_unregister(&mmp_dsi_panel_driver);
}
module_init(mmp_dsi_panel_module_init);
module_exit(mmp_dsi_panel_module_exit);

MODULE_DESCRIPTION("Panel driver for MMP DSI PANEL");
MODULE_LICENSE("GPL");
