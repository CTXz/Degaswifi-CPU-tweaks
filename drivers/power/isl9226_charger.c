/*
 * ISL9226 system voltage regulator and battery charger driver.
 *
 * Copyright (c) 2011 Marvell Technology Ltd.
 * Wenzeng Chen <wzch@marvell.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/power/isl9226.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/of_device.h>

#define MAX_CAPACITY	100
#define RECHARGE_CAPACITY	80
#define UPDATA_INTERVAL		(30*HZ)

struct isl9226_device_info {
	struct device *dev;
	struct i2c_client *client;
	struct power_supply ac;
	struct power_supply usb;
	struct notifier_block chg_notif;
	struct delayed_work chg_update_work;
	int interval;
	int ischarging;
	int is_eoc;
	int ac_chg_online;
	int usb_chg_online;
	u8 chg_cur_in;
	u8 chg_cur_out;
	u8 prechg_cur;
	u8 prechg_vol;
	u8 eoc_cur;
	u8 usb_chg_cur;
	u8 default_chg_cur;
	u8 ac_chg_cur;
};
static const u32 input_current_table[] = {
	95, 475, 855, 950, 1425, 2900, 2900, 2900,
};

static const u32 eoc_current_table[] = {
	50, 75, 100, 240
};

static const u32 prechg_current_table[] = {
	120, 150, 180,
};

static const u32 prechg_voltage_table[] = {
	3000, 2900, 2800, 2700,
};

static char *isl9226_supply_to[] = {
	"battery",
	"bq27425-0",
};

static enum power_supply_property isl9226_ac_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property isl9226_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

static int isl9226_get_table_data(const u32 *table, u32 size, u32 data)
{
	int i, ret = -1;
	for (i = 0; i < size; i++) {
		if (data == table[i]) {
			ret = i;
			break;
		}
	}
	if (ret < 0) {
		ret = 0;
		pr_err("invalid input data %d\n", data);
	}
	return ret;
}

static int isl9226_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;
	if (!client)
		return -EINVAL;
	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0) {
		dev_err(&client->dev, "isl9226: failed to write reg-0x%02x\n",
			reg);
		return ret;
	}
	return 0;
}

static int isl9226_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret = 0;
	if (!client || !data)
		return -EINVAL;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "isl9226: failed to read reg-0x%02x\n",
			reg);
		return ret;
	}
	*data = ret;
	return 0;
}

static int isl9226_set_charge(struct isl9226_device_info *info, bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= CHGEN;
	else
		value &= ~CHGEN;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static int isl9226_set_autostop(struct isl9226_device_info *info, bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= AUTOSTP;
	else
		value &= ~AUTOSTP;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static int isl9226_set_autorestart(struct isl9226_device_info *info,
				   bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= AUTORES;
	else
		value &= ~AUTORES;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static int isl9226_set_eocsppm(struct isl9226_device_info *info, bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= EOCSPPM;
	else
		value &= ~EOCSPPM;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static int isl9226_set_jeita(struct isl9226_device_info *info, bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= JEITAEN;
	else
		value &= ~JEITAEN;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static inline int isl9226_set_enidpm(struct isl9226_device_info *info,
				bool enable)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return ret;
	if (enable)
		value |= ENIDPM;
	else
		value &= ~ENIDPM;
	return isl9226_write_reg(info->client, OPMOD, value);
}

static int isl9226_set_eoc_cur(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, CCTRL1, &value);
	if (ret)
		return ret;
	value = (value & 0x3F) | ((data & 0x03) << 6);
	return isl9226_write_reg(info->client, CCTRL1, value);
}

static int isl9226_set_prechg_cur(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, CCTRL1, &value);
	if (ret)
		return ret;
	value = (value & 0xCF) | ((data & 0x03) << 4);
	return isl9226_write_reg(info->client, CCTRL1, value);
}

static int isl9226_set_current_limit(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, CCTRL1, &value);
	if (ret)
		return ret;
	value = (value & 0xF8) | (data & 0x07);
	return isl9226_write_reg(info->client, CCTRL1, value);
}

static int isl9226_set_charge_cur(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, CCTRL2, &value);
	if (ret)
		return ret;
	value = (value & 0xF8) | (data & 0x07);
	return isl9226_write_reg(info->client, CCTRL2, value);
}

static int isl9226_set_prechg_vol(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, VCTRL1, &value);
	if (ret)
		return ret;
	value = (value & 0x3F) | ((data & 0x03) << 6);
	return isl9226_write_reg(info->client, VCTRL1, value);
}

static int isl9226_set_vmax(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, VCTRL1, &value);
	if (ret)
		return ret;
	value = (value & 0xE0) | (data & 0x1F);
	return isl9226_write_reg(info->client, VCTRL1, value);
}

static inline int isl9226_set_vsppm(struct isl9226_device_info *info, u8 data)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, VCTRL2, &value);
	if (ret)
		return ret;
	value = (value & 0x3F) | ((data & 0x03) << 6);
	return isl9226_write_reg(info->client, VCTRL2, value);
}

static inline int isl9226_is_charge_enabled(struct isl9226_device_info *info)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, OPMOD, &value);
	if (ret)
		return 0;
	else
		return value & CHGEN;
}

static inline int isl9226_get_sta1(struct isl9226_device_info *info)
{
	int ret;
	u8 value;
	ret = isl9226_read_reg(info->client, STAT1, &value);
	if (ret)
		return 0;
	else
		return value;
}

static int isl9226_has_supplicant(struct isl9226_device_info *info)
{
#ifndef CONFIG_BATTERY_88PM80X
	struct power_supply *psy;
	union power_supply_propval val;
	int i, ret = 0;

	for (i = 0; i < info->ac.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->ac.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret == 0 && val.intval == 1)
			return 1;
	}
	for (i = 0; i < info->usb.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->usb.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (ret == 0 && val.intval == 1)
			return 1;
	}
	return 0;
#else
	int ret;
	u8 value;

	/* enable charger */
	isl9226_set_charge(info, 1);

	ret = isl9226_read_reg(info->client, STAT1, &value);
	if (ret)
		return ret;
	if (value & CHGING) {
		pr_info("%s: battery is present!\n", __func__);
		return 1;
	} else {
		pr_info("%s: battery is not present!\n", __func__);
		return 0;
	}
#endif
}

/*to save power isl9226 will power off after USB/AC
*cable pulg out, so set all the register
*/
static int isl9226_start_charging(struct isl9226_device_info *info)
{
	if (!isl9226_has_supplicant(info)) {
		dev_err(info->dev, "haven't supplicant, but start charging\n");
		return 0;
	}

	if (!info->usb_chg_online && !info->ac_chg_online) {
		dev_err(info->dev, "haven't charger, but start charging\n");
		return 0;
	}
	/*disable auto stop */
	isl9226_set_autostop(info, 0);

	/*enable auto restart */
	isl9226_set_autorestart(info, 1);

	/*disable eocsppm, use charger as power supply when eoc */
	isl9226_set_eocsppm(info, 0);

	/*disable jeita, we don't use that */
	isl9226_set_jeita(info, 0);

	/*
	 * set EOC detection current
	 * the current can be controlled by Riset2
	 */
	isl9226_set_eoc_cur(info, info->eoc_cur);

	/*set precharge current */
	isl9226_set_prechg_cur(info, info->prechg_cur);

	/*set out put current limit */
	isl9226_set_current_limit(info, info->chg_cur_out);

	/*set input current limit */
	isl9226_set_charge_cur(info, info->chg_cur_in);

	/*set precharge voltage */
	isl9226_set_prechg_vol(info, info->prechg_vol);

	/*set vmax to 4.2V */
	isl9226_set_vmax(info, VMAX4_2V);

	/*enable charger */
	isl9226_set_charge(info, 1);

	info->ischarging = 1;
	return 1;
}

static void isl9226_stop_charging(struct isl9226_device_info *info)
{
	if (info->usb_chg_online || info->ac_chg_online)
		isl9226_set_charge(info, 0);
	info->ischarging = 0;
}

static void isl9226_update_work_func(struct work_struct *work)
{
	u8 eoc;
	struct isl9226_device_info *info =
		container_of(work, struct isl9226_device_info,
			     chg_update_work.work);

	isl9226_read_reg(info->client, STAT1, &eoc);
	dev_dbg(info->dev, "eoc data = 0x%x\n", eoc);

	if ((((eoc >> 1) & 0x1) == 1) ||
	    (((eoc >> 2) & 0x3) == 3)) {
		dev_dbg(info->dev, "battery full...\n");
		info->ischarging = 0;
		info->is_eoc = 1;
	} else {
		dev_dbg(info->dev, "battery not full...\n");
		info->ischarging = 1;
		info->is_eoc = 0;
	}

	schedule_delayed_work(&info->chg_update_work, info->interval);
}

static int isl9226_chg_notifier_callback(struct notifier_block *nb,
					 unsigned long type, void *chg_event)
{
	struct isl9226_device_info *info =
	    container_of(nb, struct isl9226_device_info, chg_notif);
	switch (type) {
	case NULL_CHARGER:
		info->ac_chg_online = 0;
		info->usb_chg_online = 0;
		break;
	case DEFAULT_CHARGER:
		info->chg_cur_in = info->default_chg_cur;
		info->chg_cur_out = 0;
		info->usb_chg_online = 1;
		break;
	case VBUS_CHARGER:
		info->chg_cur_in = info->usb_chg_cur;
		/*set output current higher than input */
		if (info->usb_chg_cur == 475)
			info->chg_cur_out = 1;
		else
			info->chg_cur_out = 0;

		info->usb_chg_online = 1;
		break;
	case AC_CHARGER_STANDARD:
	case AC_CHARGER_OTHER:
		info->chg_cur_in = info->ac_chg_cur;
		/*set output current higher than input */
		if (info->ac_chg_cur <= 1000)
			info->chg_cur_out = 4;
		else
			info->chg_cur_out = 6;
		info->ac_chg_online = 1;
		break;
	default:
		break;
	}
	if (info->usb_chg_online || info->ac_chg_online) {
		if (isl9226_start_charging(info))
			schedule_delayed_work(&info->chg_update_work,
					      info->interval);
	} else {
		/*cable plug out, isl9226 power down, i2c read write error */
		isl9226_stop_charging(info);
		cancel_delayed_work_sync(&info->chg_update_work);
	}
	power_supply_changed(&info->ac);
	power_supply_changed(&info->usb);
	return 0;
}

static int isl9226_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct isl9226_device_info *info =
	    container_of(psy, struct isl9226_device_info, ac);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->ac_chg_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (info->ischarging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (info->is_eoc)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int isl9226_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct isl9226_device_info *info =
	    container_of(psy, struct isl9226_device_info, usb);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->usb_chg_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (info->ischarging)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (info->is_eoc)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t show_charging_attrs(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct isl9226_device_info *info = dev_get_drvdata(dev);
	char charging[] = "Charging";
	char discharging[] = "Discharging";
	char *status;

	if (info->ischarging)
		status = charging;
	else
		status = discharging;

	return sprintf(buf, "%s\n", status);
}

static ssize_t set_charging_attrs(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct isl9226_device_info *info = dev_get_drvdata(dev);
	char cmd[12];

	sscanf(buf, "%11s", cmd);

	if (!strcmp(cmd, "charging")) {
		cancel_delayed_work(&info->chg_update_work);
		isl9226_start_charging(info);
		pr_info(KERN_INFO "Start charging\n");
	}

	if (!strcmp(cmd, "discharging")) {
		cancel_delayed_work(&info->chg_update_work);
		isl9226_stop_charging(info);
		pr_info(KERN_INFO "Stop charging\n");
	}

	if (!strcmp(cmd, "exit")) {
		schedule_delayed_work(&info->chg_update_work, 0);
		pr_info(KERN_INFO "Exit charging test\n");
	}

	return count;
}


static DEVICE_ATTR(set_charging, S_IRUSR|S_IWUSR, show_charging_attrs,
		set_charging_attrs);

static struct attribute *battery_attributes[] = {
	&dev_attr_set_charging.attr,
	NULL,
};

static struct attribute_group battery_attr_group = {
	.attrs = battery_attributes,
};

static int isl9226_dt_init(struct device_node *np,
		struct device *dev,
		struct isl9226_charger_pdata *pdata)
{
	int ret;
	ret = of_property_read_u32(np, "default_input_current",
		&pdata->default_input_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "usb_input_current",
		&pdata->usb_input_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ac_input_current",
		&pdata->ac_input_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "eoc_current", &pdata->eoc_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "prechg_current",
		&pdata->prechg_current);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "prechg_voltage",
		&pdata->prechg_voltage);
	if (ret)
		return ret;

	return 0;
}

static int isl9226_charger_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct isl9226_device_info *info;
	struct isl9226_charger_pdata *pdata;
	struct device_node *node = client->dev.of_node;
	int ret = 0;
	pdata = client->dev.platform_data;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = isl9226_dt_init(node, &client->dev, pdata);
		if (ret)
			goto out;
	} else if (!pdata) {
		return -EINVAL;
	}

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}
	info->client = client;
	info->dev = &client->dev;
	i2c_set_clientdata(client, info);
	info->interval = UPDATA_INTERVAL;
	info->eoc_cur =
	    isl9226_get_table_data(eoc_current_table,
				ARRAY_SIZE(eoc_current_table),
				pdata->eoc_current);
	info->prechg_cur =
	    isl9226_get_table_data(prechg_current_table,
				ARRAY_SIZE(prechg_current_table),
				pdata->prechg_current);
	info->prechg_vol =
	    isl9226_get_table_data(prechg_voltage_table,
				ARRAY_SIZE(prechg_voltage_table),
				pdata->prechg_voltage);
	info->usb_chg_cur =
	    isl9226_get_table_data(input_current_table,
				ARRAY_SIZE(input_current_table),
				pdata->usb_input_current);
	info->ac_chg_cur =
	    isl9226_get_table_data(input_current_table,
				ARRAY_SIZE(input_current_table),
				pdata->ac_input_current);
	info->default_chg_cur =
	    isl9226_get_table_data(input_current_table,
				ARRAY_SIZE(input_current_table),
				pdata->default_input_current);
	info->ac.name = "ac";
	info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac.supplied_to = isl9226_supply_to;
	info->ac.num_supplicants = ARRAY_SIZE(isl9226_supply_to);
	info->ac.get_property = isl9226_ac_get_property;
	info->ac.properties = isl9226_ac_props;
	info->ac.num_properties = ARRAY_SIZE(isl9226_ac_props);

	ret = power_supply_register(info->dev, &info->ac);
	if (ret) {
		dev_err(&client->dev,
			"AC power supply resisteration failed!\n");
		goto out;
	}
	info->usb.name = "usb";
	info->usb.type = POWER_SUPPLY_TYPE_USB;
	info->usb.supplied_to = isl9226_supply_to;
	info->usb.num_supplicants = ARRAY_SIZE(isl9226_supply_to);
	info->usb.get_property = isl9226_usb_get_property;
	info->usb.properties = isl9226_usb_props;
	info->usb.num_properties = ARRAY_SIZE(isl9226_usb_props);
	ret = power_supply_register(info->dev, &info->usb);
	if (ret) {
		dev_err(&client->dev, "USB power supply resisteration failed!\n");
		goto out;
	}
	INIT_DELAYED_WORK(&info->chg_update_work, isl9226_update_work_func);

	info->chg_notif.notifier_call = isl9226_chg_notifier_callback;
#ifdef CONFIG_USB_MV_UDC
	ret = mv_udc_register_client(&info->chg_notif);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register  client: %d\n", ret);
		goto out;
	}
#endif

	ret = sysfs_create_group(&info->dev->kobj, &battery_attr_group);
	dev_set_drvdata(info->dev, info);

	dev_info(&client->dev, "isl9226 probe finished\n");
	return 0;
out:
	return ret;
}

static int isl9226_charger_remove(struct i2c_client *client)
{
	struct isl9226_device_info *info = i2c_get_clientdata(client);
	isl9226_stop_charging(info);
#ifdef CONFIG_USB_MV_UDC
	mv_udc_unregister_client(&info->chg_notif);
#endif
	cancel_delayed_work_sync(&info->chg_update_work);
	power_supply_unregister(&info->usb);
	power_supply_unregister(&info->ac);
	return 0;
}

#ifdef CONFIG_PM
static int isl9226_charger_suspend(struct device *dev)
{
	return 0;
}

static int isl9226_charger_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops isl9226_pm_ops = {
	.suspend = isl9226_charger_suspend,
	.resume = isl9226_charger_resume,
};

#endif
static void isl9226_charger_shutdown(struct i2c_client *client)
{
	struct isl9226_device_info *info = i2c_get_clientdata(client);

	/* stop Charger */
	isl9226_stop_charging(info);
}

static const struct i2c_device_id isl9226_id[] = {
	{"isl9226", -1},
	{}
};

static const struct of_device_id isl9226_dt_match[] = {
	{ .compatible = "marvell,isl9226", },
	{ },
};

MODULE_DEVICE_TABLE(of, isl9226_dt_match);

static struct i2c_driver isl9226_charger_driver = {
	.driver = {.name = "isl9226",
#ifdef CONFIG_PM
		   .pm = &isl9226_pm_ops,
#endif
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(isl9226_dt_match),
		   },
	.probe = isl9226_charger_probe,
	.remove = isl9226_charger_remove,
	.shutdown = isl9226_charger_shutdown,
	.id_table = isl9226_id,
};

static int __init isl9226_charger_init(void)
{
	int ret;
	ret = i2c_add_driver(&isl9226_charger_driver);
	if (ret)
		pr_err("Unable to register isl9226 charger driver");
	return ret;
}

module_init(isl9226_charger_init);
static void __exit isl9226_charger_exit(void)
{
	i2c_del_driver(&isl9226_charger_driver);
}

module_exit(isl9226_charger_exit);

MODULE_AUTHOR("Wenzeng Chen <wzch@marvell.com>");
MODULE_DESCRIPTION("isl9226 battery charger driver");
MODULE_LICENSE("GPL");
