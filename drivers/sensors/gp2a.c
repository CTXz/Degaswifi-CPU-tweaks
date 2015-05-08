/* drivers/sensors/gp2a.c
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/gp2a.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#endif

#include <linux/sensors_head.h>
#include <linux/regulator/machine.h>


/* Note about power vs enable/disable:
 *  The chip has two functions, proximity and ambient light sensing.
 *  There is no separate power enablement to the two functions (unlike
 *  the Capella CM3602/3623).
 *  This module implements two drivers: /dev/proximity and /dev/light.
 *  When either driver is enabled (via sysfs attributes), we give power
 *  to the chip.  When both are disabled, we remove power from the chip.
 *  In suspend, we remove power if light is disabled but not if proximity is
 *  enabled (proximity is allowed to wakeup from suspend).
 *
 *  There are no ioctls for either driver interfaces.  Output is via
 *  input device framework and control via sysfs attributes.
 */


#define gp2a_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)

#define GP2AP002X_PROXIMITY_OFFSET

/* ADDSEL is LOW */
#define REGS_PROX		0x0 /* Read  Only */
#define REGS_GAIN		0x1 /* Write Only */
#define REGS_HYS		0x2 /* Write Only */
#define REGS_CYCLE		0x3 /* Write Only */
#define REGS_OPMOD		0x4 /* Write Only */
#define REGS_CON		0x6 /* Write Only */

/* B1 mode */
#define PROX_NONDETECT	0x40
#define PROX_DETECT		0x20

#ifdef GP2AP002X_PROXIMITY_OFFSET
#define PROX_NONDETECT_MODE1	0x43
#define PROX_DETECT_MODE1		0x28
#define PROX_NONDETECT_MODE2	0x48
#define PROX_DETECT_MODE2		0x42
#define OFFSET_FILE_PATH	"/data/prox_cal"
#endif

/* PMIC Regulator based supply */
#define REGULATOR_SUPPLY_TYPE        1
/* gpio controlled LDO based supply */
#define LDO_SUPPLY_TYPE              0

static int nondetect;
static int detect;
/* sensor type */
#define PROXIMITY	1
#define CHIP_DEV_NAME	"GP2AP002"
#define CHIP_DEV_VENDOR	"SHARP"
struct workqueue_struct *prox_wq;

struct gp2a_data;
enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

#ifdef CONFIG_OF
static const struct of_device_id gp2a_dt_ids[] = {
	{ .compatible = "sharp,gp2a"},
	{},
};

MODULE_DEVICE_TABLE(of, gp2a_dt_ids);
#endif

/* driver data */
struct gp2a_data {
	struct input_dev *proximity_input_dev;
	struct device *proximity_dev;
	struct gp2a_platform_data *pdata;
	struct i2c_client *i2c_client;
	int irq;
	bool on;
	u8 power_state;
	struct mutex power_lock;
	struct wakeup_source prx_wakeup_source;
	struct workqueue_struct *wq;
	struct work_struct work_prox;
	char val_state;
#ifdef GP2AP002X_PROXIMITY_OFFSET
	char cal_mode;
#endif
};

int gp2a_i2c_read(struct gp2a_data *gp2a, u8 reg, u8 *val)
{
	int err = 0;
	unsigned char data[2] = {reg, 0};
	int retry = 10;
	struct i2c_msg msg[2] = {};
	struct i2c_client *client = gp2a->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].flags = 1;
	msg[1].len = 2;
	msg[1].buf = data;

	while (retry--) {
		data[0] = reg;

		err = i2c_transfer(client->adapter, msg, 2);

		if (err >= 0) {
			*val = data[1];
			return 0;
		}
	}
	return err;
}
int gp2a_i2c_write(struct gp2a_data *gp2a, u8 reg, u8 *val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 10;
	struct i2c_client *client = gp2a->i2c_client;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		data[0] = reg;
		data[1] = *val;

		msg->addr = client->addr;
		msg->flags = 0; /* write */
		msg->len = 2;
		msg->buf = data;

		err = i2c_transfer(client->adapter, msg, 1);

		if (err >= 0)
			return 0;
	}
	return err;
}

static ssize_t adc_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->val_state);
}

static ssize_t state_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->val_state);
}

static ssize_t name_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_NAME);
}

static ssize_t vendor_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_DEV_VENDOR);
}


#ifdef GP2AP002X_PROXIMITY_OFFSET
int gp2a_cal_mode_read_file(char *mode)
{
	int err = 0;
	mm_segment_t old_fs;
	struct file *cal_mode_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH,
		O_RDONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		err = PTR_ERR(cal_mode_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open cal_mode file\n", __func__);
		set_fs(old_fs);
		return err;
	}
	err = cal_mode_filp->f_op->read(cal_mode_filp,
		(char *)&mode,
		sizeof(u8), &cal_mode_filp->f_pos);

	if (err != sizeof(u8)) {
		pr_err("%s: Can't read the cal_mode from file\n",
			__func__);
		filp_close(cal_mode_filp, current->files);
		set_fs(old_fs);
		return -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int gp2a_cal_mode_save_file(char mode)
{
	struct file *cal_mode_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_mode_filp = filp_open(OFFSET_FILE_PATH,
		O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_mode_filp)) {
		pr_err("%s: Can't open cal_mode file\n",
			__func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_mode_filp);
		pr_err("%s: err = %d\n",
			__func__, err);
		return err;
	}

	err = cal_mode_filp->f_op->write(cal_mode_filp,
		(char *)&mode, sizeof(u8), &cal_mode_filp->f_pos);
	if (err != sizeof(u8)) {
		pr_err("%s: Can't read the cal_mode from file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_mode_filp, current->files);
	set_fs(old_fs);

	return err;
}

static ssize_t prox_cal_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", gp2a->cal_mode);
}

static ssize_t prox_cal_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	u8 value;
	int err;

	if (sysfs_streq(buf, "1")) {
		gp2a->cal_mode = 1;
		nondetect = PROX_NONDETECT_MODE1;
		detect = PROX_DETECT_MODE1;
	} else if (sysfs_streq(buf, "2")) {
		gp2a->cal_mode = 2;
		nondetect = PROX_NONDETECT_MODE2;
		detect = PROX_DETECT_MODE2;
	} else if (sysfs_streq(buf, "0")) {
		gp2a->cal_mode = 0;
		nondetect = PROX_NONDETECT;
		detect = PROX_DETECT;
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	value = 0x08;
	gp2a_i2c_write(gp2a, REGS_GAIN, &value);
	value = nondetect;
	gp2a_i2c_write(gp2a, REGS_HYS, &value);
	value = 0x04;
	gp2a_i2c_write(gp2a, REGS_CYCLE, &value);
	value = 0x03;
	gp2a_i2c_write(gp2a, REGS_OPMOD, &value);
	value = 0x00;
	gp2a_i2c_write(gp2a, REGS_CON, &value);

	err = gp2a_cal_mode_save_file(gp2a->cal_mode);

	if (err < 0) {
		pr_err("%s: prox_cal_write() failed\n", __func__);
		return err;
	}

	return size;
}
#endif

static DEVICE_ATTR(adc, 0440, adc_read, NULL);
static DEVICE_ATTR(state, 0440, state_read, NULL);
static DEVICE_ATTR(name, 0440, name_read, NULL);
static DEVICE_ATTR(vendor, 0440, vendor_read, NULL);
#ifdef GP2AP002X_PROXIMITY_OFFSET
static DEVICE_ATTR(prox_cal, 0664, prox_cal_read, prox_cal_write);
#endif

static struct device_attribute *proxi_attrs[] = {
	&dev_attr_adc,
	&dev_attr_state,
	&dev_attr_name,
	&dev_attr_vendor,
#ifdef GP2AP002X_PROXIMITY_OFFSET
	&dev_attr_prox_cal,
#endif
	NULL,
};

static ssize_t proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		       (gp2a->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static ssize_t proximity_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	bool new_value;
	u8 value;
	int ret = 0;

	if (sysfs_streq(buf, "1")) {
		new_value = true;
	} else if (sysfs_streq(buf, "0")) {
		new_value = false;
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&gp2a->power_lock);
	gp2a_dbgmsg("new_value = %d, old state = %d\n",
		    new_value,
		    (gp2a->power_state & PROXIMITY_ENABLED) ? 1 : 0);
#ifdef ALPS_DEBUG
	pr_info("[TMP] new_value = %d, old state = %d\n",
			new_value,
			(gp2a->power_state & PROXIMITY_ENABLED) ? 1 : 0);
#endif
	if (new_value && !(gp2a->power_state & PROXIMITY_ENABLED)) {
#ifdef GP2AP002X_PROXIMITY_OFFSET
		int err;
#endif
		pr_info("[TMP] %s, %d\n", __func__, __LINE__);
#ifdef GP2AP002X_PROXIMITY_OFFSET
		err = gp2a_cal_mode_read_file(&gp2a->cal_mode);
		if (err < 0 && err != -ENOENT)
			pr_err("%s: cal_mode file read fail\n", __func__);

		pr_info("%s: mode = %02x\n", __func__, gp2a->cal_mode);
		if (gp2a->cal_mode == 2) {
			nondetect = PROX_NONDETECT_MODE2;
			detect = PROX_DETECT_MODE2;
		} else if (gp2a->cal_mode == 1) {
			nondetect = PROX_NONDETECT_MODE1;
			detect = PROX_DETECT_MODE1;
		} else {
			nondetect = PROX_NONDETECT;
			detect = PROX_DETECT;
		}
#endif
		gp2a->val_state = 1;
		input_report_abs(gp2a->proximity_input_dev,
			ABS_DISTANCE,
			gp2a->val_state);
		input_sync(gp2a->proximity_input_dev);

		gp2a->power_state |= PROXIMITY_ENABLED;
		gp2a->pdata->power(dev, true);
		usleep_range(20000, 20000);

		value = 0x18;
		ret = gp2a_i2c_write(gp2a, REGS_CON, &value);
		if (ret < 0)
			goto i2c_error;

		value = 0x08;
		ret = gp2a_i2c_write(gp2a, REGS_GAIN, &value);
		if (ret < 0)
			goto i2c_error;

		value = nondetect;
		ret = gp2a_i2c_write(gp2a, REGS_HYS, &value);
		if (ret < 0)
			goto i2c_error;

		value = 0x04;
		ret = gp2a_i2c_write(gp2a, REGS_CYCLE, &value);
		if (ret < 0)
			goto i2c_error;

		enable_irq_wake(gp2a->irq);

		value = 0x03;
		ret = gp2a_i2c_write(gp2a, REGS_OPMOD, &value);
		if (ret < 0)
			goto i2c_error;

		enable_irq(gp2a->irq);

		value = 0x00;
		ret = gp2a_i2c_write(gp2a, REGS_CON, &value);
		if (ret < 0)
			goto i2c_error;

	} else if (!new_value && (gp2a->power_state & PROXIMITY_ENABLED)) {
		pr_info("[TMP] %s, %d\n", __func__, __LINE__);
		disable_irq(gp2a->irq);

		value = 0x02;
		ret = gp2a_i2c_write(gp2a, REGS_OPMOD, &value);
		if (ret < 0)
			goto i2c_error;

		gp2a->power_state &= ~PROXIMITY_ENABLED;
		gp2a->pdata->power(dev, false);
	}
	mutex_unlock(&gp2a->power_lock);

	return size;

i2c_error:
	pr_err("%s: read/write to gp2a failed, i2c error (%d)\n", __func__, ret);
	return ret;
}

static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	       proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */

static void gp2a_prox_work_func(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of(work,
		struct gp2a_data, work_prox);
	u8 vo, value;
	int ret = 0;

	pr_info("In %s", __func__);

	if (gp2a->irq != 0) {
		disable_irq_wake(gp2a->irq);
		disable_irq(gp2a->irq);
	} else {
		return;
	}

	ret = gp2a_i2c_read(gp2a, REGS_PROX, &vo);
	if (ret < 0)
		goto i2c_error;

	vo = 0x01 & vo;
	if (vo == gp2a->val_state) {
		if (!vo) {
			vo = 0x01;
			value = nondetect;
		} else {
			vo = 0x00;
			value = detect;
		}
#ifdef ALPS_DEBUG
		pr_info("%s: %d\n", __func__, gp2a->val_state);
#endif
		ret = gp2a_i2c_write(gp2a, REGS_HYS, &value);
		if (ret < 0)
			goto i2c_error;
		gp2a->val_state = vo;
	}


	input_report_abs(gp2a->proximity_input_dev,
		ABS_DISTANCE,
		gp2a->val_state);
	input_sync(gp2a->proximity_input_dev);
	usleep_range(20000, 20000);

	value = 0x18;
	ret = gp2a_i2c_write(gp2a, REGS_CON, &value);
	if (ret < 0)
		goto i2c_error;

	if (gp2a->irq != 0) {
		enable_irq(gp2a->irq);
		enable_irq_wake(gp2a->irq);
	}
	value = 0x00;
	ret = gp2a_i2c_write(gp2a, REGS_CON, &value);
	if (ret < 0)
		goto i2c_error;

	return;

i2c_error:
	pr_err("%s: Failed to read gp2a registers, i2c error (%d)\n", __func__, ret);
}

/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t gp2a_irq_handler(int irq, void *data)
{
	struct gp2a_data *gp2a = data;
	if (gp2a->irq != -1) {
		schedule_work((struct work_struct *)&gp2a->work_prox);
		__pm_wakeup_event(&gp2a->prx_wakeup_source, 3*HZ);
	}
	return IRQ_HANDLED;
}

#define GPIO_PS_VOUT mfp_to_gpio(GPIO092_GPIO_92)

static int gp2a_setup_irq(struct gp2a_data *gp2a)
{
	int rc;
	int irq;
	u8 value;

	gp2a_dbgmsg("start\n");

	value = 0x18;
	rc = gp2a_i2c_write(gp2a, REGS_CON, &value);
	if (rc < 0)
		goto error_i2c_write;

	irq = gp2a->irq;
	rc = request_irq(irq,
			 gp2a_irq_handler,
			 IRQF_TRIGGER_FALLING,
			 "proximity_int",
			 gp2a);

	if (rc < 0) {
		pr_err("%s: request_irq(%d) failed with err %d\n",
			__func__, irq, rc);
		return rc;
	} else {
		pr_info("%s: request_irq(%d) success\n",
			__func__, irq);
	}
	/* start with interrupts disabled */
	disable_irq(irq);

	gp2a->val_state = 1;
	gp2a->power_state &= PROXIMITY_ENABLED;
	gp2a_dbgmsg("success\n");

	value = 0x08;
	rc = gp2a_i2c_write(gp2a, REGS_GAIN, &value);
	if (rc < 0)
		goto error_i2c_write;

	value = nondetect;
	rc = gp2a_i2c_write(gp2a, REGS_HYS, &value);
	if (rc < 0)
		goto error_i2c_write;

	value = 0x04;
	rc = gp2a_i2c_write(gp2a, REGS_CYCLE, &value);
	if (rc < 0)
		goto error_i2c_write;

	value = 0x18;
	rc = gp2a_i2c_write(gp2a, REGS_CON, &value);
	if (rc < 0)
		goto error_i2c_write;

	value = 0x02;
	rc = gp2a_i2c_write(gp2a, REGS_OPMOD, &value);
	if (rc < 0)
		goto error_i2c_write;

	pr_info("%s: Setting up irq %d for gp2a SUCCESS !!\n", __func__, irq);
	return 0;

error_i2c_write:
	pr_info("%s: error setting up irq, i2c failed (%d)\n", __func__, rc);
	return rc;
}

int gp2a_device_power(struct device *dev, bool on)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	struct gp2a_platform_data *pdata = gp2a->pdata;
	static bool bFirst = 1;
	static bool bStatus;
	int min_uv, max_uv;
	int ret = 0;

	if (bFirst) {
		/* VDD Power On */
		if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {
			pdata->vdd_regulator = regulator_get(dev, "proxi_vdd");
			if (IS_ERR(pdata->vdd_regulator)) {
				dev_err(dev, "%s: vdd_regulator get error!\n", __func__);
				pdata->vdd_regulator = NULL;
				return -1;
			}

			max_uv = min_uv = pdata->vdd_regulator_volt;
			ret = regulator_set_voltage(pdata->vdd_regulator, min_uv, max_uv);
			if (ret) {
				dev_err(dev, "%s: error setting vdd regulator voltage to %d %d, returns %d\n",
							__func__, min_uv, max_uv, ret);
			}
		}

		/* LDO Power On */
		if (pdata->vled_supply_type == REGULATOR_SUPPLY_TYPE) {
			pdata->vled_regulator = regulator_get(dev, "proxi_vled");
			if (IS_ERR(pdata->vled_regulator)) {
				dev_err(dev, "%s: vled_regulator get error!\n", __func__);
				pdata->vled_regulator = NULL;
				return -1;
			}

			max_uv = min_uv = pdata->vled_regulator_volt;
			ret = regulator_set_voltage(pdata->vled_regulator, min_uv, max_uv);
			if (ret) {
				dev_err(dev, "%s: error setting vled regulator voltage to %d %d, returns %d\n",
							__func__, min_uv, max_uv, ret);
			}
		}

		if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {
			ret = gpio_direction_output(pdata->vdd_ldo_en, 0);
                        if (ret) {
                                dev_err(dev, "error setting direction (output) for vdd_ldo_en\n");
                                return ret;
                        }
		}

		if (pdata->vled_supply_type == LDO_SUPPLY_TYPE) {
			ret = gpio_direction_output(pdata->vled_ldo_en, 0);
                        if (ret) {
                                dev_err(dev, "error setting direction (output) for vled_ldo_en\n");
                                return ret;
                        }
		}

		bFirst = 0;
	}

	if (on == bStatus)
		return 0;

	if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = (on) ? regulator_enable(pdata->vdd_regulator) : regulator_disable(pdata->vdd_regulator);
		if (ret != 0)
			goto error;
		usleep_range(2000, 2000);
	}

	if (pdata->vled_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = (on) ? regulator_enable(pdata->vled_regulator) : regulator_disable(pdata->vled_regulator);
		if (ret != 0)
			goto error;
		usleep_range(2000, 2000);
	}

	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {
		(on) ? gpio_direction_output(pdata->vdd_ldo_en, 1) : gpio_direction_output(pdata->vdd_ldo_en, 0);
	}

	if (pdata->vled_supply_type == LDO_SUPPLY_TYPE) {
		(on) ? gpio_direction_output(pdata->vled_ldo_en, 1) : gpio_direction_output(pdata->vled_ldo_en, 0);
	}
	bStatus = (on) ? 1 : 0;

	return 0;

error:
	if (on) {
		pr_err("%s error in enabling regulator! ret = %d\n",
					__func__, ret);
	}
	else {
		pr_err("%s error in disabling regulator! ret = %d\n",
					__func__, ret);
	}
	return ret;
}

static int gp2a_probe_dt(struct device_node *np,
			struct device *dev,
			struct gp2a_platform_data *pdata)
{
	const struct of_device_id *match;
	int ret = 0;

	if (!np) {
		pr_info("GP2A: In %s, device node is NULL\n", __func__);
		return -EINVAL;
	}

	match = of_match_device(gp2a_dt_ids, dev);
	if (!match) {
		pr_info("GP2A: compatible mismatch\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "gp2a,vdd_supply_type", &pdata->vdd_supply_type);
        if (unlikely(ret)) {
                dev_err(dev, "error reading property vdd_supply_type from device node\n");
                goto error;
        }
	if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = of_property_read_u32(np, "gp2a,vdd_regulator_volt", &pdata->vdd_regulator_volt);
                if (unlikely(ret)) {
                        dev_err(dev, "error reading property vdd_regulator_volt from device node\n");
                        goto error;
                }
	}
	else if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {

		pdata->vdd_ldo_en = of_get_named_gpio(np, "gp2a,vdd_ldo_en", 0);
                if (unlikely(pdata->vdd_ldo_en < 0)) {
                        dev_err(dev, "error reading property vdd_ldo_en from device node\n");
                        ret = pdata->vdd_ldo_en;
                        goto error;
                }
	}

	ret = of_property_read_u32(np, "gp2a,vled_supply_type", &pdata->vled_supply_type);
        if (unlikely(ret)) {
                dev_err(dev, "error reading property vled_supply_type from device node\n");
                goto error;
        }
	if (pdata->vled_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = of_property_read_u32(np, "gp2a,vled_regulator_volt", &pdata->vled_regulator_volt);
                if (unlikely(ret)) {
                        dev_err(dev, "error reading property vled_regulator_volt from device node\n");
                        goto error;
                }
	}
	else if (pdata->vled_supply_type == LDO_SUPPLY_TYPE) {

		pdata->vled_ldo_en = of_get_named_gpio(np, "gp2a,vled_ldo_en", 0);
                if (unlikely(pdata->vled_ldo_en < 0)) {
                        dev_err(dev, "error reading property vled_ldo_en from device node\n");
                        ret = pdata->vled_ldo_en;
                        goto error;
                }
	}

	pdata->power = &gp2a_device_power;

	return 0;

error:
	return ret;
}


static int gp2a_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct gp2a_data *gp2a;
	struct gp2a_platform_data *pdata = client->dev.platform_data;

	struct device_node *np = client->dev.of_node;

	pr_info("[TMP] %s, %d\n", __func__, __LINE__);
	nondetect = PROX_NONDETECT;
	detect = PROX_DETECT;
	pr_info("%s: %02x %02x\n", __func__, nondetect, detect);

	if (IS_ENABLED(CONFIG_OF)) {

		if (unlikely(!pdata)) {
			pdata = devm_kzalloc(&client->dev,
					sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = gp2a_probe_dt(np, &client->dev, pdata);
		if (unlikely(ret)) {
			dev_err(&client->dev, "%s: error getting platform data from device node!!\n", __func__);
			goto kfree_pdata;
		}

	} else if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		return ret;
	}

	if (unlikely(!pdata->power)) {
		pr_err("%s: incomplete pdata!\n", __func__);
		goto kfree_pdata;
	}

	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {

		ret = gpio_request(pdata->vdd_ldo_en, "proxi_vdd_ldo_en");
		if (unlikely(ret)) {
			dev_err(&client->dev, "%s: error requesting vdd ldo gpio %d (%d)",
					__func__, pdata->vdd_ldo_en, ret);
			goto kfree_pdata;
		}
	}

	if (pdata->vled_supply_type == LDO_SUPPLY_TYPE) {

		ret = gpio_request(pdata->vled_ldo_en, "proxi_vled_ldo_en");
		if (unlikely(ret)) {
			dev_err(&client->dev, "%s: error requesting vled ldo gpio %d (%d)",
					__func__, pdata->vled_ldo_en, ret);
			goto error_gpio_request;
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		ret = -EIO;
		goto error_functionality;
	}

	gp2a = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (unlikely(!gp2a)) {
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		ret = -ENOMEM;
		goto error_functionality;
	}

	gp2a->pdata = pdata;

	gp2a->irq = irq_of_parse_and_map(client->dev.of_node, 0);
	gp2a->i2c_client = client;
	i2c_set_clientdata(client, gp2a);

	/* power on gp2a */
	ret = pdata->power(&client->dev, true);
	if (unlikely(ret)) {
		dev_err(&client->dev, "%s: error powering on gp2a chip (%d)!!\n", __func__, ret);
		goto err_power_on;
	}
	usleep_range(10000, 10000);

	/* wakeup source init */
	wakeup_source_init(&gp2a->prx_wakeup_source,
		       "prx_wakeup_source");
	mutex_init(&gp2a->power_lock);

	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (unlikely(!input_dev)) {
		pr_err("%s: could not allocate input device\n", __func__);
		goto err_input_allocate;
	}

	gp2a->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, gp2a);
	input_dev->name = "proximity_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(input_dev);
	if (unlikely(ret < 0)) {
		pr_err("%s: could not register input device\n",
			__func__);
		input_free_device(input_dev);
		goto err_input_allocate;
	}

	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &proximity_attribute_group);
	if (unlikely(ret)) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group;
	}

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	INIT_WORK(&gp2a->work_prox, gp2a_prox_work_func);
	ret = gp2a_setup_irq(gp2a);
	if (ret) {
		pr_err("%s: could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	ret = sensors_register(&gp2a->proximity_dev, gp2a,
		proxi_attrs, "proximity_sensor");
	if (unlikely(ret < 0)) {
		pr_info("%s: could not sensors_register\n", __func__);
		goto err_sensors_register;
	}

	/* set initial proximity value as 1 */
	input_report_abs(gp2a->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(gp2a->proximity_input_dev);

	pr_info("[TMP] %s, %d\n", __func__, __LINE__);

	pdata->power(&client->dev, false);

	pr_info("done\n");
	return 0;

	/* error, unwind it all */
err_sensors_register:
	free_irq(gp2a->irq, gp2a);
	gpio_free(gp2a->i2c_client->irq);
err_setup_irq:
	sysfs_remove_group(&gp2a->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
err_sysfs_create_group:
	input_unregister_device(gp2a->proximity_input_dev);
err_input_allocate:
	gp2a->pdata->power(&client->dev, false);
err_power_on:
	mutex_destroy(&gp2a->power_lock);
	wakeup_source_trash(&gp2a->prx_wakeup_source);
	kfree(gp2a);
error_functionality:
	if (pdata->vled_supply_type == LDO_SUPPLY_TYPE)
		gpio_free(pdata->vled_ldo_en);
error_gpio_request:
	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE)
		gpio_free(pdata->vdd_ldo_en);
kfree_pdata:
	devm_kfree(&client->dev, pdata);

	return ret;
}

static int gp2a_suspend(struct device *dev)
{
	return 0;
}

static int gp2a_resume(struct device *dev)
{
	return 0;
}

static void gp2a_i2c_shutdown(struct i2c_client *client)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	if (gp2a != NULL) {
		if (gp2a->power_state & PROXIMITY_ENABLED) {
			disable_irq_wake(gp2a->irq);
			disable_irq(gp2a->irq);
			usleep_range(20000, 20000);
		}
		sysfs_remove_group(&gp2a->proximity_input_dev->dev.kobj,
				   &proximity_attribute_group);
		input_unregister_device(gp2a->proximity_input_dev);

		free_irq(gp2a->irq, gp2a);
		gpio_free(gp2a->i2c_client->irq);

		gp2a->pdata->power(&client->dev, false);

		mutex_destroy(&gp2a->power_lock);

		wakeup_source_trash(&gp2a->prx_wakeup_source);

		kfree(gp2a);
	}
}

static const struct i2c_device_id gp2a_device_id[] = {
	{"gp2a", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gp2a_device_id);

static const struct dev_pm_ops gp2a_pm_ops = {
	.suspend = gp2a_suspend,
	.resume = gp2a_resume
};

static struct i2c_driver gp2a_i2c_driver = {
	.driver = {
		.name = "gp2a",
		.owner = THIS_MODULE,
		.pm = &gp2a_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(gp2a_dt_ids),
#endif
	},
	.probe		= gp2a_i2c_probe,
	.shutdown	= gp2a_i2c_shutdown,
	.id_table	= gp2a_device_id,
};


static int __init gp2a_init(void)
{
	pr_info("[TMP] %s, %d\n", __func__, __LINE__);
	return i2c_add_driver(&gp2a_i2c_driver);
}

static void __exit gp2a_exit(void)
{
	i2c_del_driver(&gp2a_i2c_driver);
}

module_init(gp2a_init);
module_exit(gp2a_exit);

MODULE_AUTHOR("mjchen@sta.samsung.com");
MODULE_DESCRIPTION("Optical Sensor driver for gp2ap002a00f");
MODULE_LICENSE("GPL");
