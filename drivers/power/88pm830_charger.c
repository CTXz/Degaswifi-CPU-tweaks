/*
 * 88PM830 Battery Charger driver
 *
 * Copyright (c) 2013 Marvell Technology Ltd.
 * Yi Zhang<yizhang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include <linux/err.h>
#include <linux/mfd/88pm830.h>
#include <linux/of_device.h>
#include <linux/platform_data/mv_usb.h>

#define UPDATA_INTERVAL		(30*HZ)

/* preregulator and charger register */
#define PM830_CHG_CTRL1			(0x3c)
#define	VBUS_BOOSTER_EN			(1 << 7)
#define BATT_TEMP_MONITOR_EN		(1 << 4)
#define CHG_START			(1 << 0)

#define PM830_BSUP_CTRL			(0x3d)
#define SMTH_EN				(1 << 7)
#define SMTH_SET_MASK			(0x7 << 4)
#define BYPASS_SMTH_EN			(0x1 << 3)
#define OC_OUT_EN			(0x1 << 2)
#define OC_OUT_SET			(0x3 << 0)

#define PM830_BAT_CTRL			(0x3e)
#define BAT_REM_UV_EN			(0x1 << 7)
#define BAT_SHRT_SET_OFFSET		(4)
#define BAT_SHRT_EN			(0x1 << 3)
#define	OV_BAT_SET_OFFSET		(1)
#define OV_VBAT_EN			(0x1 << 0)
#define SUPPL_PRE_DIS_MASK	(0x1 << 6)
#define SUPPL_PRE_DIS_SET(x)	((x) << 6)

#define PM830_CHG_CTRL2			(0x41)
#define VBAT_PRE_TERM_MASK		(0x3 << 6)
#define VBAT_PRE_TERM_OFFSET		(6)
#define ICHG_PRE_SET_MASK		(0x1f << 0)
#define ICHG_PRE_SET_OFFSET		(0)

#define PM830_CHG_CTRL3			(0x42)
#define ICHG_FAST_TERM_MASK		(0x7 << 5)
#define ICHG_FAST_TERM_OFFSET		(5)
#define ICHG_FAST_SET_MASK		(0xf << 0)
#define ICHG_FAST_SET_OFFSET		(0)
/* for B0 revision and above */
#define ICHG_FAST_TERM_10MA		(0x0)
#define ICHG_FAST_TERM_20MA		(0x1)
#define ICHG_FAST_TERM_40MA		(0x2)
#define ICHG_FAST_TERM_60MA		(0x3)
#define ICHG_FAST_TERM_100MA	(0x4)
#define ICHG_FAST_TERM_150MA	(0x5)
#define ICHG_FAST_TERM_200MA	(0x6)
#define ICHG_FAST_TERM_300MA	(0x7)

#define PM830_CHG_CTRL4			(0x43)
#define FAST_TERM_NUM(x)		((x) << 6)
#define VBAT_FAST_SET_MASK_A1	(0x1f << 0)
#define VBAT_FAST_SET_MASK		(0x3f << 0)
#define VBAT_FAST_SET_OFFSET		(0)


#define PM830_CHG_CTRL5			(0x44)
#define FASTCHG_TIMEOUT_MASK		(0x7 << 4)
#define FASTCHG_TIMEOUT_SET(x)		((x - 1) << 4)
#define PRECHG_TIMEOUT_MASK		(0x7 << 0)
#define PRECHG_TIMEOUT_SET(x)		(((x - 16) >> 3))

#define PM830_CHG_CTRL6			(0x45)
#define VSYS_PREREG_SET(x)		(((x - 3400) / 50) - 1)

#define PM830_CHG_CTRL7			(0x46)
#define PM830_CHG_ILIM_10		(0x4)
#define PM830_CHG_ILIM_OFFSET		(4)
#define PM830_CHG_ILIM_MASK		(0x7)
#define PM830_CHG_CTRL8			(0x47)
#define PM830_CHG_CTRL9			(0x48)
#define PM830_CHG_CTRL10		(0x49)
#define PM830_CHG_CTRL11		(0x4a)
#define PM830_CHG_CTRL12		(0x4b)
#define PM830_CHG_CTRL13		(0x4c)

#define PM830_CHG_STATUS1_A1		(0x4d)
/* B0 change: charger status move to 0x57, 0x58 */
#define PM830_CHG_STATUS1		(0x57)
#define PM830_CHG_STATUS4		(0x58)
#define PM830_FAULT_VBAT_SHORT	(0x01 << 0)
#define PM830_FAULT_OV_VBAT		(0x01 << 1)
#define PM830_FAULT_BATTEMP_NOK	(0x01 << 2)
#define PM830_FAULT_VPWR_SHORT	(0x01 << 3)
#define PM830_FAULT_CHG_REMOVAL	(0x01 << 4)
#define PM830_FAULT_BAT_REMOVAL	(0x01 << 5)
#define PM830_FAULT_CHG_TIMEOUT	(0x01 << 6)
#define PM830_FAULT_OV_TEMP_INT	(0x01 << 7)
#define PM830_FAULT_CHGWDG_EXPIRED (0x01 << 8)
#define PM830_FAULT_PRE_SUPPL_STOP (0x01 << 9)

#define PM830_CHG_CTRL14		(0x4e)
#define PM830_CHG_CTRL15		(0x4f)
#define PM830_WDG_MASK			(0x1 << 4)
#define PM830_WDG_SET(x)		((x) << 4)

#define PM830_CHG_STATUS2		(0x50)
#define PM830_SMTH			(1 << 3)
#define PM830_CHG_STATUS3		(0x51)

#define PM830_CHG_CTRL16		(0x52)
#define PM830_CHG_CTRL17		(0x53)
#define PM830_CHG_CTRL18		(0x54)
#define PM830_CHG_CTRL19		(0x55)
#define PM830_CHG_CTRL20		(0x56)

struct pm830_charger_info {
	struct pm830_chg_pdata *pdata;
	struct device *dev;
	struct power_supply ac;
	struct power_supply usb;
	struct notifier_block chg_notif;
	int ac_chg_online;
	int usb_chg_online;
	struct delayed_work chg_update_work;

	unsigned int prechg_cur;	/* precharge current limit */
	unsigned int prechg_vol;	/* precharge voltage limit */
	unsigned int prechg_timeout;	/* precharge time limit: min */

	unsigned int fastchg_eoc;	/* fastcharge end current */
	unsigned int fastchg_cur;	/* fastcharge current */
	unsigned int fastchg_vol;	/* fastcharge voltage */
	unsigned int fastchg_timeout;	/* fastcharge time limit: hr */
	unsigned int over_vol;

	unsigned int limit_cur;

	unsigned int thermal_en;
	unsigned int *thermal_thr;

	unsigned int temp_cfg;
	unsigned int temp_thr;

	unsigned int mppt_wght;
	unsigned int mppt_per;
	unsigned int mppt_max_cur;

	int irq_nums;
	int irq[7];

	struct pm830_chip *chip;
	struct regmap *regmap;
	int pm830_status;
};

/* status updated when != 0 */
static int updated;

static enum power_supply_property pm830_props[] = {
	POWER_SUPPLY_PROP_STATUS, /* Charger status output */
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};

static inline int get_prechg_cur(struct pm830_charger_info *info)
{
	static int ret;
	ret = info->prechg_cur / 10 - 1;
	dev_dbg(info->dev, "%s: precharge current = 0x%x\n",
		__func__, ret);
	return (ret < 0) ? 0 : ret;
}

static inline int get_prechg_vol(struct pm830_charger_info *info)
{
	static int ret;
	ret = (info->prechg_vol - 3300) / 100;
	ret = (ret < 0) ? 0 : ((ret > 0x3) ? 0x3 : ret);
	dev_dbg(info->dev, "%s: precharge voltage = 0x%x\n",
		__func__, ret);
	return ret;
}

static inline int get_fastchg_eoc(struct pm830_charger_info *info)
{
	static int ret;
	/* B0 change */
	unsigned int eoc = info->fastchg_eoc;
	if (info->chip->version > PM830_A1_VERSION) {
		if (eoc >= 300)
			ret = ICHG_FAST_TERM_300MA;
		else if (eoc >= 200)
			ret = ICHG_FAST_TERM_200MA;
		else if (eoc >= 150)
			ret = ICHG_FAST_TERM_150MA;
		else if (eoc >= 100)
			ret = ICHG_FAST_TERM_100MA;
		else if (eoc >= 60)
			ret = ICHG_FAST_TERM_60MA;
		else if (eoc >= 40)
			ret = ICHG_FAST_TERM_40MA;
		else if (eoc >= 20)
			ret = ICHG_FAST_TERM_20MA;
		else
			ret = ICHG_FAST_TERM_10MA;
	} else
		ret = (eoc - 10) / 10;

	dev_dbg(info->dev, "%s: fastcharge eoc = 0x%x\n",
		__func__, ret);
	return (ret < 0) ? 0 : ret;
}

static inline int get_fastchg_cur(struct pm830_charger_info *info)
{
	static int ret;
	ret = (info->fastchg_cur - 500) / 100;
	dev_dbg(info->dev, "%s: fastcharge current = 0x%x\n",
		__func__, ret);
	return (ret < 0) ? 0 : ret;
}

static inline int get_fastchg_vol(struct pm830_charger_info *info)
{
	static int ret;
	if (info->chip->version > PM830_A1_VERSION) {
		/* B0 change */
		ret = (info->fastchg_vol - 3600) / 25;
		if (ret > 0x24)
			ret = 0x24;
	} else
		ret = (info->fastchg_vol - 3600) / 50;

	dev_dbg(info->dev, "%s: fastcharge voltage = 0x%x\n",
		__func__, ret);
	return (ret < 0) ? 0 : ret;
}

static inline int get_limit_cur(struct pm830_charger_info *info)
{
	static unsigned int ret;
	if (info->limit_cur > 2400)
		info->limit_cur = 2400;
	else if (info->limit_cur < 0)
		info->limit_cur = 0;
	if (info->limit_cur > 1600) {
		info->limit_cur *= 2;
		info->limit_cur /= 3;
		ret = 1 << 7;
	}

	ret |= ((info->limit_cur / 100) - 1) ;
	dev_dbg(info->dev, "%s: current limitation = 0x%x\n",
		__func__, ret);
	return ret;
}

static inline int get_temp_thr(struct pm830_charger_info *info)
{
	static int ret;
	ret = (info->temp_thr * 1000 - 46488) / 832;
	dev_dbg(info->dev, "%s: temperature threshold = 0x%x\n",
		__func__, ret);
	return (ret < 0) ? 0 : ret;
}

static char *pm830_supplied_to[] = {
	"battery",
};

static int pm830_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	unsigned int value;
	struct pm830_charger_info *info =
		dev_get_drvdata(psy->dev->parent);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		regmap_read(info->regmap, PM830_STATUS, &value);
		if ((value & 0x1) == PM830_CHG_DET) {
			if (!strncmp(psy->name, "ac", 2))
				val->intval = info->ac_chg_online;
			else
				val->intval = info->usb_chg_online;

		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (info->chip->version > PM830_A1_VERSION) {
			/* B0 change:
			 * read 0x51[7:5] to get device status
			 *		0x0: DC/DC converter power-up
			 *		0x1: Automatic Precharge
			 *		0x2: VSYS Pre-regulated
			 *		0x3: Linear Precharge
			 *		0x4: Fastcharge
			 *		0x5: Suppliment mode
			 *		0x6: VSYS battery Powered
			 *		0x7: Boost Mode(OTG, Flash, Torch)
			 */
			regmap_read(info->regmap, PM830_CHG_STATUS3, &value);
			value >>= 5;
			if ((value == 0x6) || (value == 0x5))
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			/* 1. Write 0xf0 with the value 0x1
			 * 2. read 0xd1
			 *	0x3: auto precharge
			 *	0x4: preregulation
			 *	0x6: precharge
			 *	0x7: fastcharge
			 *	0x9: supplementmode
			 *	0xa: system is battery powered
			 */
			regmap_write(info->regmap, 0xf0, 0x1);
			regmap_read(info->regmap, 0xd1, &value);
			if ((value == 0xa) || (value == 0x9))
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		/*
		 * interrupt or workqueue happens,
		 * let's check what's the status.
		 */
		if (updated)
			val->intval = info->pm830_status;
		updated = 0;

		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int pm830_powersupply_init(struct pm830_charger_info *info,
				  struct pm830_chg_pdata *pdata)
{
	int ret = 0;

	if (pdata->supplied_to) {
		info->ac.supplied_to = pdata->supplied_to;
		info->ac.num_supplicants = pdata->num_supplicants;
		info->usb.supplied_to = pdata->supplied_to;
		info->usb.num_supplicants = pdata->num_supplicants;
	} else {
		info->ac.supplied_to = pm830_supplied_to;
		info->ac.num_supplicants = ARRAY_SIZE(pm830_supplied_to);
		info->usb.supplied_to = pm830_supplied_to;
		info->usb.num_supplicants = ARRAY_SIZE(pm830_supplied_to);
	}
	/* register ac charger props */
	info->ac.name = "ac";
	info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties = pm830_props;
	info->ac.num_properties = ARRAY_SIZE(pm830_props);
	info->ac.get_property = pm830_get_property;

	ret = power_supply_register(info->dev, &info->ac);
	if (ret)
		goto err_reg_ac;
	/* register usb charger props */
	info->usb.name = "usb";
	info->usb.type = POWER_SUPPLY_TYPE_USB;
	info->usb.properties = pm830_props;
	info->usb.num_properties = ARRAY_SIZE(pm830_props);
	info->usb.get_property = pm830_get_property;

	ret = power_supply_register(info->dev, &info->usb);
	if (ret)
		goto err_reg_usb;

	return ret;

err_reg_usb:
	power_supply_unregister(&info->ac);
err_reg_ac:
	return ret;
}

/*
 * mppt feature:
 * try to get maximum possible power from wall adapter
 */
static void pm830_mppt_enable(struct pm830_charger_info *info, int enable)
{
	if (enable) {
		regmap_update_bits(info->regmap, PM830_CHG_CTRL19,
				   0xf, info->mppt_max_cur);
		regmap_update_bits(info->regmap, PM830_CHG_CTRL19,
				   0x7 << 4, (info->mppt_per) << 4);
		regmap_update_bits(info->regmap, PM830_CHG_CTRL18,
				   0xf, info->mppt_wght);
		regmap_update_bits(info->regmap, PM830_CHG_CTRL19,
				   1 << 7, 1 << 7);
	} else {
		regmap_update_bits(info->regmap, PM830_CHG_CTRL19,
				   1 << 7, 0);
	}
}

static int pm830_start_charging(struct pm830_charger_info *info)
{
	dev_info(info->dev, "start charging...!\n");

	/* Clear CHG_START bit */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
			   CHG_START, 0);
	/* Precharge config */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL2,
			   VBAT_PRE_TERM_MASK,
			   get_prechg_vol(info) << VBAT_PRE_TERM_OFFSET);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL2,
			   ICHG_PRE_SET_MASK,
			   get_prechg_cur(info) << ICHG_PRE_SET_OFFSET);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL5,
			   PRECHG_TIMEOUT_MASK,
			   PRECHG_TIMEOUT_SET(info->prechg_timeout));
	/* Fastcharge config */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL3,
			   ICHG_FAST_TERM_MASK,
			   get_fastchg_eoc(info) << ICHG_FAST_TERM_OFFSET);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL3,
			   ICHG_FAST_SET_MASK,
			   get_fastchg_cur(info) << ICHG_FAST_SET_OFFSET);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL4,
			   (info->chip->version > PM830_A1_VERSION) ?
			   VBAT_FAST_SET_MASK : VBAT_FAST_SET_MASK_A1,
			   get_fastchg_vol(info) << VBAT_FAST_SET_OFFSET);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL5,
			   FASTCHG_TIMEOUT_MASK,
			   FASTCHG_TIMEOUT_SET(info->fastchg_timeout));

	/* set chg current limit */
	regmap_write(info->regmap, PM830_CHG_CTRL7,
		     get_limit_cur(info));
	/* current limit fine setting on USB: 0x4->10% */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL7,
			   (PM830_CHG_ILIM_MASK << PM830_CHG_ILIM_OFFSET),
			   (PM830_CHG_ILIM_10 << PM830_CHG_ILIM_OFFSET));

	/* Start charging... */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
			   CHG_START, CHG_START);

	return 0;
}

static void pm830_chg_init(struct pm830_charger_info *info)
{
	int val;
	/* Clear CHG_START bit */
	regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
			   CHG_START, 0);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
			   (1 << 1), (1 << 1));
	/* Enable bat detection */
	regmap_update_bits(info->regmap, PM830_GPADC_MEAS_EN,
			   PM830_VBAT_MEAS_EN, PM830_VBAT_MEAS_EN);
	/* enable GPADC with non-stop mode */
	regmap_update_bits(info->regmap, PM830_GPADC_CONFIG1, 0x3, 0x3);

	/* Enable detection of EOC */
	regmap_update_bits(info->regmap, PM830_CC_IBAT, 0x1, 0x1);

	if (info->over_vol < info->fastchg_vol)
		info->over_vol = info->fastchg_vol;
	/* set over-vol threshold*/
	if (info->over_vol >= 5000)
		val = 0x3;
	else if (info->over_vol > 4800)
		val = 0x2;
	else if (info->over_vol > 4600)
		val = 0x1;
	else
		val = 0x0;

	regmap_update_bits(info->regmap, PM830_BAT_CTRL,
			   (0x3 << OV_BAT_SET_OFFSET),
			   (val << OV_BAT_SET_OFFSET));

	/* Enable over-vol/short-detection detectors */
	regmap_update_bits(info->regmap, PM830_BAT_CTRL,
			   OV_VBAT_EN, OV_VBAT_EN);
	regmap_update_bits(info->regmap, PM830_BAT_CTRL,
			   BAT_SHRT_EN, BAT_SHRT_EN);

	/* thermal loop */
	if (info->thermal_en) {
		/* battery temperature measurement enable */
		regmap_update_bits(info->regmap, PM830_GPADC_MEAS_EN,
				   PM830_GPADC0_MEAS_EN, PM830_GPADC0_MEAS_EN);
		/* set temperature threshold */
		regmap_write(info->regmap, PM830_CHG_CTRL10,
			     info->thermal_thr[0]);
		regmap_write(info->regmap, PM830_CHG_CTRL11,
			     info->thermal_thr[1]);
		regmap_write(info->regmap, PM830_CHG_CTRL12,
			     info->thermal_thr[2]);
		regmap_write(info->regmap, PM830_CHG_CTRL13,
			     info->thermal_thr[3]);
		/* enable thermal loop */
		regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
				   BATT_TEMP_MONITOR_EN, BATT_TEMP_MONITOR_EN);
	}

	/* Internal temperature protection */
	regmap_write(info->regmap, PM830_CHG_CTRL8, get_temp_thr(info));
	regmap_write(info->regmap, PM830_CHG_CTRL9, info->temp_cfg);

	if (info->chip->version > PM830_A1_VERSION) {
		/* disable watch dog now */
		regmap_update_bits(info->regmap, PM830_CHG_CTRL15,
				PM830_WDG_MASK, PM830_WDG_SET(1));

		/* Disable precharge when the device enter suppliment mode */
		regmap_update_bits(info->regmap, PM830_BAT_CTRL,
				SUPPL_PRE_DIS_MASK, SUPPL_PRE_DIS_SET(0));
	}
}

static int pm830_stop_charging(struct pm830_charger_info *info)
{
	dev_info(info->dev, "stop charging...!\n");
	pm830_mppt_enable(info, 0);
	regmap_update_bits(info->regmap, PM830_CHG_CTRL1,
			   CHG_START, 0);
	return 0;
}

static int pm830_chg_notifier_callback(struct notifier_block *nb,
				       unsigned long type, void *chg_event)
{
	struct pm830_charger_info *info =
		container_of(nb, struct pm830_charger_info, chg_notif);
	switch (type) {
	case NULL_CHARGER:
		info->ac_chg_online = 0;
		info->usb_chg_online = 0;
		break;
	case VBUS_CHARGER:
	case DEFAULT_CHARGER:
		info->ac_chg_online = 0;
		info->usb_chg_online = 1;
		info->fastchg_cur = 500;
		info->limit_cur = 500;
		break;
	case AC_CHARGER_STANDARD:
	case AC_CHARGER_OTHER:
		info->ac_chg_online = 1;
		info->usb_chg_online = 0;
		/* the max value according to spec */
		info->fastchg_cur = 2000;
		info->limit_cur = 2400;
		pm830_mppt_enable(info, 1);
		break;
	default:
		info->ac_chg_online = 0;
		info->usb_chg_online = 1;
		info->fastchg_cur = 500;
		info->limit_cur = 500;
		pm830_mppt_enable(info, 1);
		break;
	}
	if (info->usb_chg_online || info->ac_chg_online) {
		pm830_start_charging(info);
		schedule_delayed_work(&info->chg_update_work,
				      UPDATA_INTERVAL);
	} else {
		pm830_stop_charging(info);
		cancel_delayed_work_sync(&info->chg_update_work);
	}
	dev_dbg(info->dev, "usb inserted: ac = %d, usb = %d\n",
		info->ac_chg_online, info->usb_chg_online);
	power_supply_changed(&info->ac);
	power_supply_changed(&info->usb);
	return 0;
}

static irqreturn_t pm830_bat_handler(int irq, void *data)
{
	struct pm830_charger_info *info = data;
	int value;

	dev_info(info->dev, "battery in/out interrupt is served\n");

	regmap_read(info->regmap, PM830_STATUS, &value);
	switch (value & 0x3) {
	case (PM830_BAT_DET | PM830_CHG_DET):
		dev_info(info->dev, "charger/battery both online\n");
		/*
		 * battery is plugged in, with usb is present,
		 * let's try to charge.
		 */
		pm830_start_charging(info);
		info->pm830_status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case PM830_CHG_DET:
		dev_info(info->dev, "only charger online\n");
		/* battery is plugged out, stop charging */
		pm830_stop_charging(info);
		info->pm830_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case PM830_BAT_DET:
		dev_info(info->dev, "only battery online\n");
		/*
		 * battery is plugged in, while usb is not present
		 * this case doesn't exist, do nothing.
		 */
		break;
	default:
		/*
		 * battery is plugged out, before that,
		 * the usb is not present, do nothing.
		 */
		break;
	}
	updated++;

	return IRQ_HANDLED;
}

static irqreturn_t pm830_temp_handler(int irq, void *data)
{
	static unsigned int hi, lo, val;
	static int itemp_fine;
	struct pm830_charger_info *info = data;

	/* report internal temperature */
	regmap_read(info->regmap, PM830_ITEMP_HI, &hi);
	regmap_read(info->regmap, PM830_ITEMP_LO, &lo);
	val = (hi << 4) | lo;

	/* result = value * 0.104 * 2048 / (2^23) */
	val = (((val & 0xfff) * 213) >> 23) - 273;
	dev_err(info->dev, "internal temperature: %d\n", val);

	if (!itemp_fine) {
		itemp_fine = 1;
		pm830_stop_charging(info);
		info->pm830_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		itemp_fine = 0;
		pm830_start_charging(info);
		info->pm830_status = POWER_SUPPLY_STATUS_CHARGING;
	}
	updated++;

	return IRQ_HANDLED;
}

static irqreturn_t pm830_done_handler(int irq, void *data)
{
	static int value;
	int ret, volt;
	unsigned char buf[2];
	struct pm830_charger_info *info = data;
	dev_dbg(info->dev, "charge done interrupt is served:\n");

	regmap_read(info->regmap, PM830_STATUS, &value);
	switch (value & 0x3) {
	case (PM830_BAT_DET | PM830_CHG_DET):
		dev_dbg(info->dev, "may real done\n");
		ret = regmap_bulk_read(info->regmap,
				       PM830_IBAT_EOC1, buf, 2);
		if (ret < 0) {
			/* rely on the charger when fails */
			info->pm830_status = POWER_SUPPLY_STATUS_FULL;
			return IRQ_HANDLED;
		}
		volt = ((buf[1] & 0x3f) << 8) | (buf[0] & 0xff);
		volt <<= 12;
		volt *= 17;
		if (volt <= (info->fastchg_eoc) * 100000)
			info->pm830_status = POWER_SUPPLY_STATUS_FULL;
		else {
			dev_info(info->dev,
				 "88pm830 reports different value: %d--%d\n",
				 volt, (info->fastchg_eoc * 100000));
			info->pm830_status = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case PM830_CHG_DET:
		dev_dbg(info->dev, "battery plugged out case\n");
		info->pm830_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case PM830_BAT_DET:
		dev_dbg(info->dev, "usb plugged out case\n");
		info->pm830_status = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		/* internal temperatue case: do nothing */
		dev_dbg(info->dev, "internal temperature case\n");
		break;
	}
	updated++;

	return IRQ_HANDLED;
}

static irqreturn_t pm830_exception_handler(int irq, void *data)
{
	static int status = 0, value;
	struct pm830_charger_info *info = data;

	if (info->chip->version < PM830_B0_VERSION) {
		/* report interrupt reason according to register 0x4d */
		regmap_read(info->regmap, PM830_CHG_STATUS1_A1, &status);
	} else {
		/*
		* report interrupt reason according to register 0x57,
		* 0x58(B0 change)
		*/
		regmap_read(info->regmap, PM830_CHG_STATUS1, &value);
		status = value & 0x0f;
		regmap_read(info->regmap, PM830_CHG_STATUS4, &value);
		status |= (value & 0x03) << 8;
	}
	/* PM830_IRQ_CHG_FAULT */
	if (status & PM830_FAULT_VBAT_SHORT)
		dev_err(info->dev, "battery short detect!\n");
	if (status & PM830_FAULT_OV_VBAT)
		dev_err(info->dev, "battery over voltage!\n");
	if (status & PM830_FAULT_BATTEMP_NOK)
		/* TODO: polling temperature to reenable charge */
		dev_err(info->dev, "battery over temperature!\n");
	/* PM830_IRQ_CHG_TOUT */
	if (status & PM830_FAULT_CHG_TIMEOUT)
		dev_err(info->dev, "charge timeout!\n");
	if (status & PM830_FAULT_PRE_SUPPL_STOP)
		dev_err(info->dev, "charger supplement stop!\n");
	if (status & PM830_FAULT_CHGWDG_EXPIRED)
		dev_err(info->dev, "charger watchdog timeout!\n");
	/* write clear, only clear fault flag bits */
	if (info->chip->version < PM830_B0_VERSION) {
		regmap_write(info->regmap, PM830_CHG_STATUS1_A1, 0x0f);
	} else {
		regmap_write(info->regmap, PM830_CHG_STATUS1, 0x0f);
		regmap_write(info->regmap, PM830_CHG_STATUS4, 0x03);
	}

	pm830_stop_charging(info);
	info->pm830_status = POWER_SUPPLY_STATUS_DISCHARGING;
	updated++;

	return IRQ_HANDLED;
}

static void pm830_chg_work_func(struct work_struct *work)
{
	struct pm830_charger_info *info =
		container_of(work, struct pm830_charger_info,
			     chg_update_work.work);
	int value;

	/* handle long time in supplement mode */
	if (info->chip->version > PM830_A1_VERSION) {
		/* B0 change: read 0x51[7:5] to get device status */
		regmap_read(info->regmap, PM830_CHG_STATUS3, &value);
		value >>= 5;
		if (value == 0x6) {
			info->pm830_status = POWER_SUPPLY_STATUS_DISCHARGING;
			updated++;
		}
	} else {
		/* read reserved register 0xd1 to get device status */
		regmap_write(info->regmap, 0xf0, 0x1);
		regmap_read(info->regmap, 0xd1, &value);
		if (value == 0xa) {
			info->pm830_status = POWER_SUPPLY_STATUS_DISCHARGING;
			updated++;
		}
	}
	schedule_delayed_work(&info->chg_update_work,
			      UPDATA_INTERVAL);
}

static struct pm830_irq_desc {
	const char *name;
	irqreturn_t (*handler)(int irq, void *data);
} pm830_irq_descs[] = {
	{"battery detect", pm830_bat_handler},
	{"charger internal temp", pm830_temp_handler},
	{"charge done", pm830_done_handler},
	{"charge timeout", pm830_exception_handler},
	{"charge fault", pm830_exception_handler},
};

static int pm830_chg_dt_init(struct device_node *np,
			 struct device *dev,
			 struct pm830_chg_pdata *pdata)
{
	int ret;
	ret = of_property_read_u32(np, "prechg-current", &pdata->prechg_cur);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "prechg-voltage", &pdata->prechg_vol);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "prechg-timeout",
				   &pdata->prechg_timeout);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "fastchg-eoc", &pdata->fastchg_eoc);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "fastchg-voltage", &pdata->fastchg_vol);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "fastchg-timeout",
				   &pdata->fastchg_timeout);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "over-voltage", &pdata->over_vol);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "no-thermal-support", &pdata->thermal_en);
	if (ret)
		return ret;

	ret = of_property_read_u32_array(np, "thermal-threshold",
					 pdata->thermal_thr, 4);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "temp-configure", &pdata->temp_cfg);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "temp-threshold", &pdata->temp_thr);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "mppt-weight", &pdata->mppt_wght);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "mppt-period", &pdata->mppt_per);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "mppt-start-current",
				   &pdata->mppt_max_cur);
	if (ret)
		return ret;

	return 0;
}

static int pm830_charger_probe(struct platform_device *pdev)
{
	struct pm830_charger_info *info;
	struct pm830_chg_pdata *pdata = pdev->dev.platform_data;
	struct pm830_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct device_node *node = pdev->dev.of_node;
	int ret = 0;
	int i;
	int j;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&pdev->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = pm830_chg_dt_init(node, &pdev->dev, pdata);
		if (ret)
			goto out;
	} else if (!pdata) {
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct pm830_charger_info),
			GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "Cannot allocate memory.\n");
		return -ENOMEM;
	}

	info->pdata = pdata;
	info->dev = &pdev->dev;

	info->prechg_cur = pdata->prechg_cur;
	info->prechg_vol = pdata->prechg_vol;
	info->prechg_timeout = pdata->prechg_timeout;

	info->fastchg_eoc = pdata->fastchg_eoc;
	info->fastchg_vol = pdata->fastchg_vol;
	info->fastchg_timeout = pdata->fastchg_timeout;

	info->over_vol = pdata->over_vol;

	info->thermal_en = pdata->thermal_en;
	info->thermal_thr = pdata->thermal_thr;

	info->temp_cfg = pdata->temp_cfg;
	info->temp_thr = pdata->temp_thr;

	info->mppt_wght = pdata->mppt_wght;
	info->mppt_per = pdata->mppt_per;
	info->mppt_max_cur = pdata->mppt_max_cur;

	info->chip = chip;
	info->regmap = chip->regmap;

	platform_set_drvdata(pdev, info);

	for (i = 0, j = 0; i < pdev->num_resources; i++) {
		info->irq[j] = platform_get_irq(pdev, i);
		if (info->irq[j] < 0)
			continue;
		j++;
	}
	info->irq_nums = j;

	/* register powersupply */
	ret = pm830_powersupply_init(info, pdata);
	if (ret) {
		dev_err(info->dev, "register powersupply fail!\n");
		goto out;
	}

	INIT_DELAYED_WORK(&info->chg_update_work, pm830_chg_work_func);
	/* register charger event notifier */
	info->chg_notif.notifier_call = pm830_chg_notifier_callback;

#ifdef CONFIG_USB_MV_UDC
	ret = mv_udc_register_client(&info->chg_notif);
	if (ret < 0) {
		dev_err(info->dev, "failed to register client!\n");
		goto err_psy;
	}
#endif

	pm830_chg_init(info);
	/* interrupt should be request in the last stage */
	for (i = 0; i < info->irq_nums; i++) {
		ret = devm_request_threaded_irq(info->dev, info->irq[i], NULL,
					   pm830_irq_descs[i].handler,
					   IRQF_ONESHOT | IRQF_NO_SUSPEND,
					   pm830_irq_descs[i].name, info);
		if (ret < 0) {
			dev_err(info->dev, "failed to request IRQ: #%d: %d\n",
				info->irq[i], ret);
			goto out_irq;
		}
	}

	dev_info(info->dev, "%s is successful!\n", __func__);

	return 0;
out_irq:
	while (--i >= 0)
		devm_free_irq(info->dev, info->irq[i], info);
#ifdef CONFIG_USB_MV_UDC
err_psy:
#endif
	power_supply_unregister(&info->ac);
	power_supply_unregister(&info->usb);
out:
	return ret;
}

static int pm830_charger_remove(struct platform_device *pdev)
{
	struct pm830_charger_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (info) {
		pm830_stop_charging(info);
#ifdef CONFIG_USB_MV_UDC
		mv_udc_unregister_client(&info->chg_notif);
#endif
		power_supply_unregister(&info->ac);
		power_supply_unregister(&info->usb);
	}

	return 0;
}

static const struct of_device_id pm830_chg_dt_match[] = {
	{ .compatible = "marvell,88pm830-chg", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm830_chg_dt_match);

static struct platform_driver pm830_charger_driver = {
	.probe = pm830_charger_probe,
	.remove = pm830_charger_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "88pm830-chg",
		.of_match_table = of_match_ptr(pm830_chg_dt_match),
	},
};

module_platform_driver(pm830_charger_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("88pm830 Charger Driver");
MODULE_AUTHOR("Yi Zhang<yizhang@marvell.com>");
