/*
 * Battery driver for Marvell 88PM830 charger/fuel-gauge chip
 *
 * Copyright (c) 2013 Marvell International Ltd.
 * Author:	Yi Zhang <yizhang@marvell.com>
 *		Jett Zhou <jtzhou@marvell.com>
 *		Shay Pathov <shayp@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/mfd/88pm830.h>
#include <linux/delay.h>
#include <linux/math64.h>
#include <linux/of_device.h>
#include "88pm830_fg.h"

#define MONITOR_INTERVAL		(HZ * 60)
#define LOW_BAT_INTERVAL		(HZ * 10)

/* coulomb counter regieter */
#define PM830_CC_AVG(x)			(x << 4)
#define PM830_CC_CLR_ON_READ		(1 << 2)
#define PM830_DEC_2ND_ORD_EN		(1 << 1)
#define PM830_CC_EN			(1 << 0)

#define PM830_CC_CTRL2			(0x25)
#define PM830_OFFCOMP_EN		(1 << 1)
#define PM830_CC_READ_REQ		(1 << 0)

/* cc 0x26 ~ 0x2A */
#define PM830_CCNT_VAL1			(0x26)
/* ibat 0x2B ~ 0x2D*/
#define PM830_IBAT_CC1			(0x2B)

#define PM830_CC_FLAG			(0x2E)
#define PM830_IBAT_OFFVAL_REG		(0x2F)
#define PM830_CC_MISC1			(0x30)
#define PM830_IBAT_SLP_SEL_A1	(1 << 3)
#define PM830_CC_MISC2			(0x32)
#define PM830_IBAT_SLP_TH(x)		(x << 4)
#define PM830_VBAT_SMPL_NUM(x)		(x << 1)
#define PM830_IBAT_EOC_EN		(1 << 0)

#define PM830_SLP_TIMER			(0x35)

#define PM830_VBAT_AVG_SD1		(0x37)
#define PM830_VBAT_SLP1			(0x39)
#define PM830_VBAT_LOW_TH		(0x76)
#define PM830_VBAT_AVG1			(0xBA)

/* bit definitions of GPADC Bias Current 1 Register */
#define GP0_BAIS_SET			(5)

/* GPADC0 Bias Current value in uA unit */
#define GP0_BAIS_SET_UA		((GP0_BAIS_SET) * 5 + 1)

struct pm830_battery_params {
	int status;
	int present;
	int volt;	/* µV */
	int ibat;	/* mA */
	int cap;	/* percents: 0~100% */
	int health;
	int tech;
	int temp;
};

struct pm830_battery_info {
	struct pm830_chip	*chip;
	struct device	*dev;
	struct pm830_battery_params	bat_params;

	struct power_supply	battery;
	struct delayed_work	monitor_work;
	struct delayed_work	charged_work;
	struct workqueue_struct *bat_wqueue;

	unsigned	present:1;
	int		r_int;
	unsigned int	bat_ntc;

	int		use_ocv;
	int		range_low_th;
	int		range_high_th;
	int		slp_con;	/* slp cnt for relaxed battery */
};

static char *pm830_supply_to[] = {
	"ac",
	"usb",
};

struct ccnt {
	int debug;

	/* mC, 1mAH = 1mA * 3600 = 3600 mC */
	int soc;
	int max_cc;
	int last_cc;
};
static struct ccnt ccnt_data;

struct buffed {
	int soc;
	int temp;
	int use_ocv;
};
static struct buffed extern_data;

static int pm800_battery_update_buffed(struct buffed *value)
{
	int data;
	/* save values in RTC registers on PMIC */
	data = (value->soc & 0x7f) | (value->use_ocv << 7);
	write_pmic(1, 0, data);

	/* Bits 0,1 are in use for different purpose,
	 * so give up on 2 LSB of temperatue */
	data = ((value->temp / 10) + 50) & 0xfc;
	write_pmic(1, 1, data);

	return 0;
}

static int pm800_battery_get_buffed(struct buffed *value)
{
	int data1, data2;

	/* read values from RTC registers on PMIC */
	data1 = read_pmic(1, 0);
	value->soc = data1 & 0x7f;
	value->use_ocv = (data1 & 0x80) >> 7;

	data2 = read_pmic(1, 1);
	value->temp = ((data2 & 0xfc) - 50) * 10;
	if (value->temp < 0)
		value->temp = 0;

	/* If both registers are 0, then stored values are invalid */
	if (data1 == 0 && data2 == 0)
		return -1;

	return 0;
}

/*
 * register 1 bit[7:0] -- bit[11:4] of measured value of voltage
 * register 0 bit[3:0] -- bit[3:0] of measured value of voltage
 */
static int pm830_get_batt_vol(struct pm830_battery_info *info, int active)
{
	int ret, data;
	unsigned char buf[2];

	if (active) {
		ret = regmap_bulk_read(info->chip->regmap,
				       PM830_VBAT_AVG1, buf, 2);
		if (ret < 0)
			return ret;
		data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	} else {
		ret = regmap_bulk_read(info->chip->regmap,
				       PM830_VBAT_SLP1, buf, 2);
		if (ret < 0)
			return ret;
		data = (buf[0] & 0xff) | ((buf[1] & 0x0f) << 8);
	}

	/* measure(mv) = value * 4 * 1.4 *1000/(2^12) */
	data = ((data & 0xfff) * 7 * 100) >> 9;
	dev_dbg(info->dev, "%s--> %s: vbat: %dmV\n",
		__func__, active ? "active" : "sleep", data);
	return data;
}

/* get soc from ocv: lookup table */
static int pm830_get_batt_soc(int ocv)
{
	int i, count;

	if (ocv < ocv_table[0]) {
		ccnt_data.soc = 0;
		return 0;
	}

	count = 100;
	for (i = count - 1; i >= 0; i--) {
		if (ocv >= ocv_table[i]) {
			ccnt_data.soc = i + 1;
			break;
		}
	}
	ccnt_data.last_cc = ccnt_data.max_cc / 1000 * (ccnt_data.soc * 10 + 5);
	return 0;
}

static int pm830_get_ibat_cc(struct pm830_battery_info *info)
{
	int ret, data;
	unsigned char buf[3];

	ret = regmap_bulk_read(info->chip->regmap,
			       PM830_IBAT_CC1, buf, 3);
	if (ret < 0)
		return ret;
	data = ((buf[2] & 0x3) << 16)
		| ((buf[1] & 0xff) << 8)
		| (buf[0] & 0xff);
	/* ibat is negative if discharging, poistive is charging */
	if (data & (1 << 17))
		data = (0xff << 24) | (0x3f << 18) | data;
	if (ccnt_data.debug)
		dev_dbg(info->dev, "%s--> *data = 0x%x, %d\n",
			__func__, data, data);
	data = (data * 458) / 10;
	if (ccnt_data.debug)
		dev_dbg(info->dev, "%s--> ibat_cc = %duA, %dmA\n",
			__func__, data, data / 1000);
	return data;
}

/* read gpadc0 to get temperature */
static int get_batt_temp(struct pm830_battery_info *info)
{
	int ret, data;
	int temp;
	unsigned char buf[2];

	ret = regmap_bulk_read(info->chip->regmap,
			       PM830_GPADC0_MEAS1, buf, 2);
	if (ret < 0)
		return ret;
	data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	/* measure(mv) = value * 1.4 *1000/(2^12) */
	data = ((data & 0xfff) * 7 * 25) >> 9;
	/* meausered Vtbat(mV) / Ibias_current */
	data = (data * 1000) / GP0_BAIS_SET_UA;

	if (data > TBAT_NEG_25D)
		temp = -30;	/* over cold , suppose -30 roughly */
	else if (data > TBAT_NEG_10D)
		temp = -15;	/* -15 degree, code */
	else if (data > TBAT_0D)
		temp = -5;	/* -5 degree */
	else if (data > TBAT_10D)
		temp = 5;	/* in range of (0, 10) */
	else if (data > TBAT_20D)
		temp = 15;	/* in range of (10, 20) */
	else if (data > TBAT_30D)
		temp = 25;	/* in range of (20, 30) */
	else if (data > TBAT_40D)
		temp = 35;	/* in range of (30, 40) */
	else
		temp = 45;	/* over heat ,suppose 45 roughly */

	dev_dbg(info->dev, "temp_C:%dC, NTC Resistor: %dohm\n", temp, data);

	return temp;
}

static int is_charger_online(struct pm830_battery_info *info,
			     int *who_is_chg)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int i, ret = 0;

	for (i = 0; i < info->battery.num_supplicants; i++) {
		psy = power_supply_get_by_name(info->battery.supplied_to[i]);
		if (!psy || !psy->get_property)
			continue;
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (ret == 0) {
			if (val.intval) {
				*who_is_chg = i;
				return val.intval;
			}
		}
	}
	*who_is_chg = 0;
	return 0;
}

static inline int pm830_calc_soc(struct pm830_battery_info *info,
				 struct ccnt *ccnt_val)
{
	ccnt_val->soc = ccnt_val->last_cc * 100 / ccnt_val->max_cc;
	return 0;
}

/*
 * if ocv is not reliable, we'll try to caculate a new ocv
 * only when
 * 1) battery is relaxed enough
 * and
 * 2) battery is in good range
 * then OCV will be updated;
 * OR, the caculated OCV is NOT reliable
 */
static void update_ocv_flag(struct pm830_battery_info *info,
			    struct ccnt *ccnt_val)
{
	unsigned char buf[2];
	int ret, prev_soc, vol, data = 0;
	if (info->use_ocv)
		return;

	/* save old SOC in case to recover */
	prev_soc = ccnt_val->soc;

	/* check if battery is relaxed enough */
	ret = regmap_bulk_read(info->chip->regmap,
			       PM830_SLP_TIMER, buf, 2);
	data = (buf[0] & 0x0f) | ((buf[1] & 0xff) << 4);
	dev_dbg(info->dev, "88pm830 slp_cnt = %d seconds\n", data);
	if (data >= info->slp_con) {
		dev_info(info->dev, "Long sleep detected! %d seconds\n",
			 data);
		/* read last sleep voltage and calc new SOC */
		vol = pm830_get_batt_vol(info, 0);
		pm830_get_batt_soc(vol);
		/* check if the new OSC is in good range or not */
		if ((ccnt_val->soc < info->range_low_th)
		    || (ccnt_val->soc > info->range_high_th)) {
			info->use_ocv = 1;
			dev_info(info->dev, "SOC is in good range! new SOC = %d\n",
				 ccnt_val->soc);
		} else {
			/* SOC is NOT in good range, so restore previous SOC */
			dev_info(info->dev, "NOT in good range (%d), don't update\n",
				 ccnt_val->soc);
			ccnt_val->soc = prev_soc;
			ccnt_val->last_cc = (ccnt_val->max_cc / 1000)
				* (ccnt_val->soc * 10 + 5);
		}
	}
}

static int pm830_fg_calc_ccnt(struct pm830_battery_info *info,
			      struct ccnt *ccnt_val)
{
	int ret, data, factor;
	unsigned char buf[5];
	s64 ccnt_uc = 0, ccnt_mc = 0;
	unsigned char buf1[2];

	ret = regmap_bulk_read(info->chip->regmap,
			       PM830_CC_FLAG, buf1, 2);
	if (ccnt_val->debug)
		dev_dbg(info->dev, "%s--> [0x2E: 0x%x], [0x2F: 0x%x]\n",
			__func__, buf1[0], buf1[1]);

	regmap_read(info->chip->regmap,
		    PM830_CC_CTRL2, &data);
	if ((data & PM830_CC_READ_REQ) == 0) {
		regmap_update_bits(info->chip->regmap,
				   PM830_CC_CTRL2,
				   PM830_CC_READ_REQ, PM830_CC_READ_REQ);
	}

	ret = regmap_bulk_read(info->chip->regmap,
			       PM830_CCNT_VAL1, buf, 5);
	if (ret < 0)
		return ret;
	ccnt_uc = (s64) (((s64)(buf[4]) << 32)
			 | (buf[3] << 24) | (buf[2] << 16)
			 | (buf[1] << 8) | buf[0]);
	if (ccnt_val->debug)
		dev_dbg(info->dev, "%s--> data: 0x%llx, %lld\n",
			__func__, ccnt_uc, ccnt_uc);

	/* Factor is nC */
	factor = 715;
	ccnt_uc = ccnt_uc * factor;
	ccnt_uc = div_s64(ccnt_uc, 1000);
	ccnt_mc = div_s64(ccnt_uc, 1000);
	if (ccnt_val->debug)
		dev_dbg(info->dev, "%s--> ccnt_uc: %lld uC, ccnt_mc: %lld mC\n",
			__func__, ccnt_uc, ccnt_mc);
	ccnt_val->last_cc += ccnt_mc;

	return 0;
}

/* update battery status */
static void pm830_bat_update_status(struct pm830_battery_info *info)
{
	int ret, who, ibat, data = 0;
	struct power_supply *psy;
	union power_supply_propval val;

	/* hardcode type[Lion] and health[Good] */
	info->bat_params.health = POWER_SUPPLY_HEALTH_GOOD;
	info->bat_params.tech = POWER_SUPPLY_TECHNOLOGY_LION;

	if (info->bat_ntc) {
		data = get_batt_temp(info);
		info->bat_params.temp = data * 10;
	} else {
		info->bat_params.temp = 260;
	}

	info->bat_params.volt = pm830_get_batt_vol(info, 1);

	ibat = pm830_get_ibat_cc(info);
	info->bat_params.ibat = ibat / 1000; /* Change to mA */

	/* charging status */
	if (info->bat_params.present == 0) {
		info->bat_params.status = POWER_SUPPLY_STATUS_UNKNOWN;
		return;
	}

	/* report charger online timely */
	if (!is_charger_online(info, &who))
		info->bat_params.status =
			POWER_SUPPLY_STATUS_DISCHARGING;
	else {
		psy = power_supply_get_by_name(info->battery.supplied_to[who]);
		if (!psy || !psy->get_property) {
			pr_info("%s: get power supply failed.\n", __func__);
			return;
		}
		ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
		if (ret == 0) {
			switch (val.intval) {
			case POWER_SUPPLY_STATUS_CHARGING:
			case POWER_SUPPLY_STATUS_FULL:
			case POWER_SUPPLY_STATUS_DISCHARGING:
				info->present = 1;
				info->bat_params.status = val.intval;
				break;
			case POWER_SUPPLY_STATUS_NOT_CHARGING:
				info->present = 0;
				break;
			default:
				pr_info("%s: charger status: %d\n",
					__func__, val.intval);
				info->present = 0;
				break;
			}
		}
	}

	/* update SOC using Columb counter */
	pm830_fg_calc_ccnt(info, &ccnt_data);
	pm830_calc_soc(info, &ccnt_data);

	update_ocv_flag(info, &ccnt_data);

	/* whether the battery is full depends on charger */
	if (info->bat_params.status == POWER_SUPPLY_STATUS_FULL)
		info->bat_params.cap = 100;
	else if (ccnt_data.soc >= 100)
		info->bat_params.cap = 99;
	else
		info->bat_params.cap = ccnt_data.soc;
}

static void pm830_battery_work(struct work_struct *work)
{
	struct pm830_battery_info *info =
		container_of(work, struct pm830_battery_info,
			     monitor_work.work);

	pm830_bat_update_status(info);

	dev_dbg(info->dev, "%s--> soc=%d, temp=%d, use_ocv=%d\n", __func__,
		ccnt_data.soc, info->bat_params.temp, info->use_ocv);

	/* save the recent value in non-volatile memory */
	extern_data.soc = ccnt_data.soc;
	extern_data.temp = info->bat_params.temp;
	extern_data.use_ocv = info->use_ocv;
	pm800_battery_update_buffed(&extern_data);

	power_supply_changed(&info->battery);
	if (info->bat_params.cap <= LOW_BAT_CAP)
		queue_delayed_work(info->bat_wqueue, &info->monitor_work,
				   LOW_BAT_INTERVAL);
	else
		queue_delayed_work(info->bat_wqueue, &info->monitor_work,
				   MONITOR_INTERVAL);
}

static void pm830_charged_work(struct work_struct *work)
{
	struct pm830_battery_info *info =
		container_of(work, struct pm830_battery_info,
			     charged_work.work);

	pm830_bat_update_status(info);
	power_supply_changed(&info->battery);
	/* NO need to be scheduled again */
	return;
}

static void pm830_fg_setup(struct pm830_battery_info *info)
{
	int data, mask;

	/*
	 * 1) Enable system clock of fg
	 * 2) Sigma-delta power up
	 * done in mfd driver
	 */

	/* Ibat offset compensation */
	data = mask = PM830_OFFCOMP_EN;
	regmap_update_bits(info->chip->regmap,
			   PM830_CC_CTRL2, mask, data);

	/*
	 * enable ibat sel and eoc measurement
	 * IBAT_EOC_EN is used by charger to decide EOC
	 */
	if (info->chip->version > PM830_A1_VERSION) {
		/*
		* B0 change: use Ibat_EOC to detect Low-Power
		* state (sleep)
		*/
		data = mask = PM830_VBAT_SMPL_NUM(3)
			| PM830_IBAT_EOC_EN
			| PM830_IBAT_SLP_TH(3);
		regmap_update_bits(info->chip->regmap,
				   PM830_CC_MISC2, mask, data);
	} else {
		data = mask = PM830_IBAT_SLP_SEL_A1
			| PM830_VBAT_SMPL_NUM(3)
			| PM830_IBAT_EOC_EN
			| PM830_IBAT_SLP_TH(3);
		regmap_update_bits(info->chip->regmap,
				   PM830_CC_MISC2, mask, data);
	}

	/* enable the second order filter */
	data = mask = PM830_DEC_2ND_ORD_EN
		| PM830_CC_CLR_ON_READ
		| PM830_CC_AVG(0x6);
	regmap_update_bits(info->chip->regmap,
			   PM830_CC_CTRL1, mask, data);

	/* enable coulomb counter */
	regmap_update_bits(info->chip->regmap,
			   PM830_CC_CTRL1, PM830_CC_EN, PM830_CC_EN);
}

static void pm830_init_fg(struct pm830_battery_info *info)
{
	int data, mask, ret;
	int vbat, vbat_slp, slp_cnt;
	unsigned char buf[2];

	/* enable VBAT, which is used to measure voltage */
	data = mask = PM830_VBAT_MEAS_EN;
	regmap_update_bits(info->chip->regmap,
			   PM830_GPADC_MEAS_EN, mask, data);

	/* gpadc0 is used to measure NTC */
	if (info->bat_ntc) {
		data = mask = PM830_GPADC0_MEAS_EN;
		regmap_update_bits(info->chip->regmap,
				   PM830_GPADC_MEAS_EN, mask, data);

		data = mask = GP0_BAIS_SET;
		regmap_update_bits(info->chip->regmap,
				   PM830_GPADC_BIAS0, mask, data);

		data = mask = PM830_GPADC_GP_BIAS_EN(1)
			| PM830_GPADC_GP_BIAS_OUT(1);
		regmap_update_bits(info->chip->regmap,
				   PM830_GPADC_BIAS_ENA, mask, data);

		data = mask = PM830_BD_GP_SEL(0) | PM830_BD_EN;
		regmap_update_bits(info->chip->regmap,
				   PM830_GPADC_CONFIG2, mask, data);

		/* battery detection */
		regmap_read(info->chip->regmap,
			    PM830_STATUS, &data);

		if (PM830_BAT_DET & data)
			info->present = 1;
		else
			info->present = 0;
	}
	/* set VBAT low threshold as 3.5V */
	regmap_write(info->chip->regmap,
		     PM830_VBAT_LOW_TH, 0xa0);

	/* check whether battery present */
	info->bat_params.present = info->present;
	/* fg set up */
	pm830_fg_setup(info);

	/* read vbat sleep value from PMIC */
	buf[0] = read_pmic(3, 0);
	buf[1] = read_pmic(3, 1);
	data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	/* measure(mv) = value * 4 * 1.4 *1000/(2^12) */
	vbat_slp = ((data & 0xfff) * 7 * 100) >> 9;

	/* read sleep conter from PMIC */
	slp_cnt = read_pmic(1, -5) << 3;

	dev_info(info->dev, "PMIC: vbat_slp = %dmV, slp_cnt = %ds\n",
		 vbat_slp, slp_cnt);

	/* read Vbat from 88pm830 */
	vbat = pm830_get_batt_vol(info, 1);
	dev_info(info->dev, "88pm830: vbat = %dmV\n", vbat);

	/*
	 * 1) 88pm830 vbat_slp is 0, so use PMIC vbat_slp
	 * 2) if PMIC vbat_slp == 0, use 88pm830 vbat
	 * 3) "reset button pressed" case is not handled
	 */
	if (vbat_slp != 0)
		vbat = vbat_slp;

	/*
	 * calc SOC via present voltage:
	 * 1) stored values invalid: get soc via ocv;
	 * 2) battery changed: get soc via ocv;
	 * 3) battery no changed:
	 *	a) relaxed: good range->get soc via ocv
	 *		    bad  range->use stored value
	 *	b) no-relaxed: use stored value
	 */
	pm830_get_batt_soc(vbat);
	dev_info(info->dev, "%s: SOC caculated from VBAT is %d%%\n",
		 __func__, ccnt_data.soc);

	/* recover previous SOC from non-volatile memory */
	ret = pm800_battery_get_buffed(&extern_data);
	dev_info(info->dev,
		 "%s: stored: soc=%d, temp=%d, use_ocv=%d\n", __func__,
		 extern_data.soc,
		 extern_data.temp,
		 extern_data.use_ocv);

	if ((ret < 0) ||
	    (extern_data.soc > ccnt_data.soc + 15) ||
	    (ccnt_data.soc > extern_data.soc + 15)) {
		if (ret < 0) /* stored values are invalid */
			dev_info(info->dev, "battery may be changed!\n");
		else /* battery is changed */
			dev_info(info->dev, "battery is changed!\n");
		info->use_ocv = 0;
	} else {
		/* same battery */
		dev_info(info->dev, "%s: same battery\n", __func__);
		if (ccnt_data.soc >= extern_data.soc) {
			/* mainly for the relaxed + good range case */
			dev_dbg(info->dev, "clamp soc for same battery!\n");
			ccnt_data.soc = extern_data.soc;
		}
		if (slp_cnt >= info->slp_con) {
			/* battery was relaxed */
			dev_info(info->dev, "battery relxed for %ds",
				 info->slp_con);
			if ((ccnt_data.soc < info->range_low_th)
			    || (ccnt_data.soc > info->range_high_th)) {
				dev_info(info->dev, "in good range, caculate...\n");
				info->use_ocv = 1;
			} else {/* in the flaten range */
				dev_info(info->dev,
					 "NOT in good range, use stored value...\n");
				ccnt_data.soc = extern_data.soc;
				ccnt_data.last_cc = (ccnt_data.max_cc / 1000)
					* (ccnt_data.soc * 10 + 5);
				info->use_ocv = extern_data.use_ocv;
			}
		} else {
			/*
			 * for battery is not relaxed,
			 * new measured value is not reliable,
			 * use the stored value instead
			 */
			dev_info(info->dev, "NOT relaxed enough, use stored SOC\n");
			ccnt_data.soc = extern_data.soc;
			ccnt_data.last_cc = (ccnt_data.max_cc / 1000)
				* (ccnt_data.soc * 10 + 5);
			info->use_ocv = extern_data.use_ocv;
		}
	}

	dev_info(info->dev, "%s: 88pm830 battery present: %d soc: %d%%\n",
		 __func__, info->bat_params.present, ccnt_data.soc);
}

static void pm830_external_power_changed(struct power_supply *psy)
{
	struct pm830_battery_info *info;

	info = container_of(psy, struct pm830_battery_info, battery);
	queue_delayed_work(info->bat_wqueue,
			   &info->charged_work, 0);
	return;
}

static enum power_supply_property pm830_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
};

static int pm830_batt_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct pm830_battery_info *info = dev_get_drvdata(psy->dev->parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = info->bat_params.status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = info->bat_params.present;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* report fake capacity without battery */
		if (!info->bat_params.present)
			info->bat_params.cap = 80;
		val->intval = info->bat_params.cap;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = info->bat_params.tech;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = info->bat_params.volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = info->bat_params.ibat;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (!info->bat_params.present)
			info->bat_params.temp = 240;
		val->intval = info->bat_params.temp;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = info->bat_params.health;
		break;
	default:
		return -ENODEV;
	}
	return 0;
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int s = 0;
	s += sprintf(buf, "debug  %d\n", ccnt_data.debug);
	return s;
}

static ssize_t debug_store(
			   struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t size)
{
	sscanf(buf, "%d", &ccnt_data.debug);
	dev_info(dev, "%s: debug: %d\n", __func__, ccnt_data.debug);
	return size;
}
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, debug_show, debug_store);

static int pm830_fg_dt_init(struct device_node *np,
			 struct device *dev,
			 struct pm830_bat_pdata *pdata)
{
	int ret;

	ret = of_property_read_u32(np, "bat-ntc-support", &pdata->bat_ntc);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "bat-capacity", &pdata->capacity);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "external-resistor", &pdata->r_int);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "sleep-period", &pdata->slp_con);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "low-threshold", &pdata->range_low_th);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "high-threshold", &pdata->range_high_th);
	if (ret)
		return ret;

	return 0;
}

static int pm830_battery_probe(struct platform_device *pdev)
{
	struct pm830_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm830_battery_info *info;
	struct pm830_bat_pdata *pdata;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	info = devm_kzalloc(&pdev->dev,
			    sizeof(struct pm830_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	pdata = pdev->dev.platform_data;
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&pdev->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = pm830_fg_dt_init(node, &pdev->dev, pdata);
		if (ret)
			return -EINVAL;
	} else if (!pdata) {
		return -EINVAL;
	}

	info->bat_ntc = pdata->bat_ntc;
	if (pdata->capacity)
		ccnt_data.max_cc = pdata->capacity * 3600;
	else
		ccnt_data.max_cc = 1500 * 3600;
	info->r_int = pdata->r_int;
	info->slp_con = pdata->slp_con;
	info->range_low_th = pdata->range_low_th;
	info->range_high_th = pdata->range_high_th;

	info->chip = chip;
	info->dev = &pdev->dev;
	info->bat_params.status = POWER_SUPPLY_STATUS_UNKNOWN;

	platform_set_drvdata(pdev, info);

	pm830_init_fg(info);
	pm830_bat_update_status(info);

	info->battery.name = "battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = pm830_batt_props;
	info->battery.num_properties = ARRAY_SIZE(pm830_batt_props);
	info->battery.get_property = pm830_batt_get_prop;
	info->battery.external_power_changed = pm830_external_power_changed;
	info->battery.supplied_to = pm830_supply_to;
	info->battery.num_supplicants = ARRAY_SIZE(pm830_supply_to);

	power_supply_register(&pdev->dev, &info->battery);

	info->battery.dev->parent = &pdev->dev;
	info->bat_wqueue = create_singlethread_workqueue("bat-88pm830");
	if (!info->bat_wqueue) {
		dev_info(chip->dev,
			 "[%s]Failed to create bat_wqueue\n", __func__);
		ret = -ESRCH;
		goto out;
	}

	INIT_DEFERRABLE_WORK(&info->monitor_work, pm830_battery_work);
	INIT_DELAYED_WORK(&info->charged_work, pm830_charged_work);
	queue_delayed_work(info->bat_wqueue, &info->monitor_work, HZ);

	ret = device_create_file(&pdev->dev, &dev_attr_debug);
	if (ret < 0) {
		dev_err(&pdev->dev, "device attr create fail: %d\n", ret);
		goto out;
	}

	device_init_wakeup(&pdev->dev, 1);
	return 0;

out:
	power_supply_unregister(&info->battery);
	return ret;
}

static int pm830_battery_remove(struct platform_device *pdev)
{
	struct pm830_battery_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work(&info->monitor_work);
	cancel_delayed_work(&info->charged_work);
	flush_workqueue(info->bat_wqueue);
	power_supply_unregister(&info->battery);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void pm830_battery_shutdown(struct platform_device *pdev)
{
	struct pm830_battery_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work(&info->monitor_work);
	flush_workqueue(info->bat_wqueue);
}

#ifdef CONFIG_PM
static int pm830_battery_suspend(struct device *dev)
{
	struct pm830_battery_info *info = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&info->monitor_work);
	return pm830_dev_suspend(dev);
}

static int pm830_battery_resume(struct device *dev)
{
	struct pm830_battery_info *info = dev_get_drvdata(dev);

	/*
	 * When exiting sleep, we might want to have new OCV.
	 * Since reading the sleep counter resets it,
	 * We would like to avoid reading it on short wakeups.
	 * Therefore delay this work with 300 msec,
	 * so in case of short wake up -
	 * it will be canceled before entering sleep again.
	 */
	queue_delayed_work(info->bat_wqueue, &info->monitor_work,
			   300 * HZ / 1000);
	return pm830_dev_resume(dev);
}

static const struct dev_pm_ops pm830_battery_pm_ops = {
	.suspend	= pm830_battery_suspend,
	.resume		= pm830_battery_resume,
};
#endif

static const struct of_device_id pm830_fg_dt_match[] = {
	{ .compatible = "marvell,88pm830-bat", },
	{ },
};
MODULE_DEVICE_TABLE(of, pm830_fg_dt_match);

static struct platform_driver pm830_battery_driver = {
	.driver		= {
		.name	= "88pm830-bat",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(pm830_fg_dt_match),
#ifdef CONFIG_PM
		.pm	= &pm830_battery_pm_ops,
#endif
	},
	.probe		= pm830_battery_probe,
	.remove		= pm830_battery_remove,
	.shutdown	= pm830_battery_shutdown,
};

static int __init pm830_battery_init(void)
{
	return platform_driver_register(&pm830_battery_driver);
}
module_init(pm830_battery_init);

static void __exit pm830_battery_exit(void)
{
	platform_driver_unregister(&pm830_battery_driver);
}
module_exit(pm830_battery_exit);

MODULE_DESCRIPTION("Marvell 88PM830 battery driver");
MODULE_LICENSE("GPL");
