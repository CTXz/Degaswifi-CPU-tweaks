/* drivers/sensors/bma254_driver.c
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

#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/bma254.h>
#include <linux/sensors_head.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#endif
#define DEFENCE_REACTIVE_ALERT
#if defined(DEFENCE_REACTIVE_ALERT)
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/ktime.h>
#endif
#include <linux/regulator/machine.h>


#undef BMA254_LOGGING

#if defined(DEFENCE_REACTIVE_ALERT)
static unsigned long long t_before;
static unsigned long nanosec_before;
#endif

static int bma254_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		goto error;
	*data = dummy & 0x000000ff;

	return 0;
error:
	dev_err(&client->dev, "%s: error reading data (%d)\n", __func__, dummy);
	return dummy;
}

static int bma254_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		goto  error;

	return 0;
error:
	dev_err(&client->dev, "%s: error writing data (%d)\n", __func__, dummy);
	return dummy;

}

static int bma254_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		goto error;

	return 0;
error:
	dev_err(&client->dev, "%s: error reading block data (%d)\n",
							__func__, dummy);
	return dummy;
}

static int bma254_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data1;

	struct bma254_data *bma254 = i2c_get_clientdata(client);

#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	pr_info("%s: %d reactive_enable = %d, reactive_state =%d\n", __func__,
		mode, atomic_read(&bma254->reactive_enable),
		atomic_read(&bma254->reactive_state));
#endif

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		if (unlikely(mode < 3)) {

			comres = bma254_smbus_read_byte(client,
					BMA254_EN_LOW_POWER__REG, &data1);
			if (comres < 0)
				return -1;

			switch (mode) {
			case BMA254_MODE_NORMAL:
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
				if (atomic_read(&bma254->reactive_enable)) {
					pr_info("%s: %d Break! cause \
						reactive_enable is 1\n",
						__func__, mode);
					break;
				}
#endif
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_LOW_POWER, 0);
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_SUSPEND, 0);
				break;

			case BMA254_MODE_LOWPOWER:
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_LOW_POWER, 1);
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_SUSPEND, 0);
				break;

			case BMA254_MODE_SUSPEND:
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
				if (atomic_read(&bma254->reactive_enable))
					break;
#endif
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_LOW_POWER, 0);
				data1  = BMA254_SET_BITSLICE(data1,
						BMA254_EN_SUSPEND, 1);
				break;

			default:
				break;
			}

			comres += bma254_smbus_write_byte(client,
					BMA254_EN_LOW_POWER__REG, &data1);
		} else {
			comres = -1;
		}
	}

	return comres;
}

static int bma254_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		comres = bma254_smbus_read_byte(client,
				BMA254_EN_LOW_POWER__REG, mode);
		*mode  = (*mode) >> 6;
	}

	return comres;
}

static int bma254_set_range(struct i2c_client *client, unsigned char range)
{
	int comres = 0;
	unsigned char data1;
	int i;

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		for (i = 0; i < ARRAY_SIZE(bma254_valid_range); i++) {
			if (bma254_valid_range[i] == range)
				break;
		}
		if (ARRAY_SIZE(bma254_valid_range) > i) {
			comres = bma254_smbus_read_byte(client,
					BMA254_RANGE_SEL_REG, &data1);

			data1  = BMA254_SET_BITSLICE(data1,
					BMA254_RANGE_SEL, range);

			comres += bma254_smbus_write_byte(client,
					BMA254_RANGE_SEL_REG, &data1);
		} else {
			comres = -EINVAL;
		}
	}

	return comres;
}

static int bma254_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data;

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		comres = bma254_smbus_read_byte(client, BMA254_RANGE_SEL__REG,
				&data);
		if (comres < 0)
			return -EINVAL;
		data = BMA254_GET_BITSLICE(data, BMA254_RANGE_SEL);
		*range = data;
	}

	return comres;
}


static int bma254_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data;
	int i = 0;

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		for (i = 0; i < ARRAY_SIZE(bma254_valid_bw); i++) {
			if (bma254_valid_bw[i] == BW)
				break;
		}
		if (ARRAY_SIZE(bma254_valid_bw) > i) {
			comres = bma254_smbus_read_byte(client,
					BMA254_BANDWIDTH__REG, &data);
			data = BMA254_SET_BITSLICE(data, BMA254_BANDWIDTH, BW);
			comres += bma254_smbus_write_byte(client,
					BMA254_BANDWIDTH__REG, &data);
		} else {
			comres = -EINVAL;
		}
	}

	return comres;
}

static int bma254_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data;

	if (unlikely(client == NULL)) {
		comres = -1;
	} else {
		comres = bma254_smbus_read_byte(client, BMA254_BANDWIDTH__REG,
				&data);
		data = BMA254_GET_BITSLICE(data, BMA254_BANDWIDTH);
		if (data < BMA254_BW_7_81HZ)
			*BW = BMA254_BW_7_81HZ;
		else if (data > BMA254_BW_1000HZ)
			*BW = BMA254_BW_1000HZ;
		else
			*BW = data;
	}

	return comres;
}

static void bma254_xyz_position_adjust(struct bma254acc *acc,
		int position)
{
	const int position_map[][3][3] = {
	{{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1} }, /* 0 top/upper-left */
	{{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1} }, /* 1 top/upper-right */
	{{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1} }, /* 2 top/lower-right */
	{{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1} }, /* 3 top/lower-left */
	{{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1} }, /* 4 bottom/upper-left */
	{{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1} }, /* 5 bottom/upper-right */
	{{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1} }, /* 6 bottom/lower-right */
	{{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1} }, /* 7 bottom/lower-left*/
	};

	struct bma254acc xyz_adjusted = {0,};
	s16 raw[3] = {0,};
	int j;

	raw[0] = acc->x;
	raw[1] = acc->y;
	raw[2] = acc->z;

	for (j = 0; j < 3; j++) {
		xyz_adjusted.x += (position_map[position][0][j] * raw[j]);
		xyz_adjusted.y += (position_map[position][1][j] * raw[j]);
		xyz_adjusted.z += (position_map[position][2][j] * raw[j]);
	}

	acc->x = xyz_adjusted.x;
	acc->y = xyz_adjusted.y;
	acc->z = xyz_adjusted.z;
}

static int bma254_read_accel_xyz(struct bma254_data *bma254,
		struct bma254acc *acc)
{
	int comres;
	unsigned char data[6];

	if (unlikely(bma254->bma254_client == NULL)) {
		comres = -1;
		pr_err("%s, client err = %d\n", __func__, comres);
	}
	else {
		comres = bma254_smbus_read_byte_block(bma254->bma254_client,
				BMA254_ACC_X_LSB__REG, data, 6);

		acc->x = BMA254_GET_BITSLICE(data[0], BMA254_ACC_X_LSB)
			|(BMA254_GET_BITSLICE(data[1], BMA254_ACC_X_MSB)
					<<BMA254_ACC_X_LSB__LEN);
		acc->x = acc->x << (sizeof(short)*8-(BMA254_ACC_X_LSB__LEN
					+ BMA254_ACC_X_MSB__LEN));
		acc->x = acc->x >> (sizeof(short)*8-(BMA254_ACC_X_LSB__LEN
					+ BMA254_ACC_X_MSB__LEN));

		acc->y = BMA254_GET_BITSLICE(data[2], BMA254_ACC_Y_LSB)
			| (BMA254_GET_BITSLICE(data[3], BMA254_ACC_Y_MSB)
					<<BMA254_ACC_Y_LSB__LEN);
		acc->y = acc->y << (sizeof(short)*8-(BMA254_ACC_Y_LSB__LEN
					+ BMA254_ACC_Y_MSB__LEN));
		acc->y = acc->y >> (sizeof(short)*8-(BMA254_ACC_Y_LSB__LEN
					+ BMA254_ACC_Y_MSB__LEN));

		acc->z = BMA254_GET_BITSLICE(data[4], BMA254_ACC_Z_LSB)
			| (BMA254_GET_BITSLICE(data[5], BMA254_ACC_Z_MSB)
					<<BMA254_ACC_Z_LSB__LEN);
		acc->z = acc->z << (sizeof(short)*8-(BMA254_ACC_Z_LSB__LEN
					+ BMA254_ACC_Z_MSB__LEN));
		acc->z = acc->z >> (sizeof(short)*8-(BMA254_ACC_Z_LSB__LEN
					+ BMA254_ACC_Z_MSB__LEN));
	}

#ifdef	BMA254_LOGGING
	pr_info("preraw x = %d, y = %d, z = %d, adjust = %d ",
		acc->x, acc->y, acc->z, bma254->axis_adjust);
#endif

	if (bma254->axis_adjust)
		bma254_xyz_position_adjust(acc, bma254->position);

#ifdef	BMA254_LOGGING
	pr_err("raw  x = %d, y = %d, z = %d, adjust = %d\n",
		acc->x, acc->y, acc->z, bma254->axis_adjust);
#endif

	return comres;
}

static void bma254_work_func(struct work_struct *work)
{
	struct bma254_data *bma254 = container_of((struct delayed_work *)work,
			struct bma254_data, work);
	static struct bma254acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma254->delay));

	bma254_read_accel_xyz(bma254, &acc);

	input_report_abs(bma254->input, ABS_X, acc.x);
	input_report_abs(bma254->input, ABS_Y, acc.y);
	input_report_abs(bma254->input, ABS_Z, acc.z);
	input_sync(bma254->input);

	mutex_lock(&bma254->value_mutex);
	bma254->value = acc;
	mutex_unlock(&bma254->value_mutex);

	schedule_delayed_work(&bma254->work, delay);
}

static ssize_t bma254_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (unlikely(bma254_get_range(bma254->bma254_client, &data) < 0))
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma254_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_range(bma254->bma254_client,
			(unsigned char)data) < 0))
		return -EINVAL;

	return count;
}

static ssize_t bma254_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (unlikely(bma254_get_bandwidth(bma254->bma254_client, &data) < 0))
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma254_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_bandwidth(bma254->bma254_client,
			(unsigned char) data) < 0))
		return -EINVAL;

	return count;
}

static ssize_t bma254_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (unlikely(bma254_get_mode(bma254->bma254_client, &data) < 0))
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma254_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_mode(bma254->bma254_client,
			(unsigned char) data) < 0))
		return -EINVAL;

	return count;
}


static ssize_t bma254_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma254_data *bma254 = input_get_drvdata(input);
	int err;

	mutex_lock(&bma254->value_mutex);
	err = sprintf(buf, "%d, %d, %d\n", bma254->value.x, bma254->value.y,
			bma254->value.z);
	mutex_unlock(&bma254->value_mutex);

	return err;
}

static ssize_t bma254_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma254->delay));

}

static ssize_t bma254_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(data < BMA254_MIN_DELAY))
		data = BMA254_MIN_DELAY;

	if (unlikely(data > BMA254_DEFAULT_DELAY))
		data = BMA254_DEFAULT_DELAY;

	atomic_set(&bma254->delay, (unsigned int) data);

	return count;
}


static ssize_t bma254_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma254->enable));

}

static void bma254_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&bma254->enable);

	mutex_lock(&bma254->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			bma254_set_mode(bma254->bma254_client,
					BMA254_MODE_NORMAL);
			schedule_delayed_work(&bma254->work,
					msecs_to_jiffies(
						atomic_read(&bma254->delay)));
			atomic_set(&bma254->enable, 1);
		}
	} else {
		if (pre_enable == 1) {
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
			if (!atomic_read(&bma254->reactive_enable)) {
				bma254_set_mode(bma254->bma254_client,
						BMA254_MODE_SUSPEND);
			}
#else
			bma254_set_mode(bma254->bma254_client,
						BMA254_MODE_SUSPEND);
#endif
			cancel_delayed_work_sync(&bma254->work);
			atomic_set(&bma254->enable, 0);
		}
	}
	mutex_unlock(&bma254->enable_mutex);

}

static ssize_t bma254_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if ((data == 0) || (data == 1))
		bma254_set_enable(dev, data);

	return count;
}

static ssize_t bma254_update_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	mutex_lock(&bma254->value_mutex);
	bma254_read_accel_xyz(bma254, &bma254->value);
	mutex_unlock(&bma254->value_mutex);

	return count;
}

static int bma254_set_selftest_st(struct i2c_client *client,
		unsigned char selftest)
{
	int comres = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client,
			BMA254_EN_SELF_TEST__REG, &data);
	if (comres < 0)
		return comres;

	data = BMA254_SET_BITSLICE(data,
			BMA254_EN_SELF_TEST, selftest);
	comres = bma254_smbus_write_byte(client,
			BMA254_EN_SELF_TEST__REG, &data);

	return comres;
}

static int bma254_set_selftest_stn(struct i2c_client *client,
					unsigned char stn)
{
	int comres = 0;
	unsigned char data;

	bma254_smbus_read_byte(client,
			BMA254_NEG_SELF_TEST__REG, &data);
	data = BMA254_SET_BITSLICE(data,
			BMA254_NEG_SELF_TEST, stn);
	comres = bma254_smbus_write_byte(client,
			BMA254_NEG_SELF_TEST__REG, &data);

	return comres;
}

static int bma254_read_accel_x(struct i2c_client *client, short *a_x)
{
	int comres;
	unsigned char data[2];

	comres = bma254_smbus_read_byte_block(client,
			BMA254_ACC_X_LSB__REG, data, 2);

	*a_x = BMA254_GET_BITSLICE(data[0], BMA254_ACC_X_LSB)
		| (BMA254_GET_BITSLICE(data[1], BMA254_ACC_X_MSB)
				<< BMA254_ACC_X_LSB__LEN);

	*a_x = *a_x << (sizeof(short)*8
			- (BMA254_ACC_X_LSB__LEN+BMA254_ACC_X_MSB__LEN));

	*a_x = *a_x >> (sizeof(short)*8
			- (BMA254_ACC_X_LSB__LEN+BMA254_ACC_X_MSB__LEN));

	return comres;
}
static int bma254_read_accel_y(struct i2c_client *client, short *a_y)
{
	int comres;
	unsigned char data[2];

	comres = bma254_smbus_read_byte_block(client,
			BMA254_ACC_Y_LSB__REG, data, 2);

	*a_y = BMA254_GET_BITSLICE(data[0], BMA254_ACC_Y_LSB)
		| (BMA254_GET_BITSLICE(data[1], BMA254_ACC_Y_MSB)
				<< BMA254_ACC_Y_LSB__LEN);

	*a_y = *a_y << (sizeof(short)*8
			- (BMA254_ACC_Y_LSB__LEN+BMA254_ACC_Y_MSB__LEN));

	*a_y = *a_y >> (sizeof(short)*8
			- (BMA254_ACC_Y_LSB__LEN+BMA254_ACC_Y_MSB__LEN));

	return comres;
}

static int bma254_read_accel_z(struct i2c_client *client, short *a_z)
{
	int comres;
	unsigned char data[2];

	comres = bma254_smbus_read_byte_block(client,
			BMA254_ACC_Z_LSB__REG, data, 2);

	*a_z = BMA254_GET_BITSLICE(data[0], BMA254_ACC_Z_LSB)
		| BMA254_GET_BITSLICE(data[1], BMA254_ACC_Z_MSB)
		<< BMA254_ACC_Z_LSB__LEN;

	*a_z = *a_z << (sizeof(short)*8
			- (BMA254_ACC_Z_LSB__LEN+BMA254_ACC_Z_MSB__LEN));

	*a_z = *a_z >> (sizeof(short)*8
			- (BMA254_ACC_Z_LSB__LEN+BMA254_ACC_Z_MSB__LEN));

	return comres;
}

static ssize_t bma254_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma254->selftest_result));
}

static ssize_t bma254_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	unsigned char clear_value = 0;
	int error;
	short value1 = 0;
	short value2 = 0;
	short diff = 0;
	unsigned long result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(data != 1))
		return -EINVAL;
	/* set to 2 G range */
	if (unlikely(bma254_set_range(bma254->bma254_client, 0) < 0))
		return -EINVAL;

	bma254_smbus_write_byte(bma254->bma254_client, 0x32, &clear_value);

	/* 1 for x-axis*/
	bma254_set_selftest_st(bma254->bma254_client, 1);

	/* positive direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 0);
	usleep_range(10000, 20000);
	bma254_read_accel_x(bma254->bma254_client, &value1);

	/* negative direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 1);
	usleep_range(10000, 20000);
	bma254_read_accel_x(bma254->bma254_client, &value2);
	diff = value1-value2;

	pr_info("diff x is %d, value1 is %d, value2 is %d\n",
			diff, value1, value2);

	if (abs(diff) < 204)
		result |= 1;

	/* 2 for y-axis*/
	bma254_set_selftest_st(bma254->bma254_client, 2);

	/* positive direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 0);
	usleep_range(10000, 20000);
	bma254_read_accel_y(bma254->bma254_client, &value1);

	/* negative direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 1);
	usleep_range(10000, 20000);
	bma254_read_accel_y(bma254->bma254_client, &value2);
	diff = value1-value2;
	pr_info("diff y is %d, value1 is %d, value2 is %d\n",
			diff, value1, value2);
	if (abs(diff) < 204)
		result |= 2;

	/* 3 for z-axis*/
	bma254_set_selftest_st(bma254->bma254_client, 3);

	/* positive direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 0);
	usleep_range(10000, 20000);
	bma254_read_accel_z(bma254->bma254_client, &value1);

	/* negative direction*/
	bma254_set_selftest_stn(bma254->bma254_client, 1);
	usleep_range(10000, 20000);
	bma254_read_accel_z(bma254->bma254_client, &value2);
	diff = value1 - value2;

	pr_info("diff z is %d, value1 is %d, value2 is %d\n",
			diff, value1, value2);
	if (abs(diff) < 102)
		result |= 4;

	atomic_set(&bma254->selftest_result, (unsigned int) result);

	pr_info("self test finished\n");

	return count;
}

static int bma254_set_offset_target_x(struct i2c_client *client,
		unsigned char offsettarget)
{
	int comres = 0;
	unsigned char data;

	bma254_smbus_read_byte(client,
			BMA254_COMP_TARGET_OFFSET_X__REG, &data);
	data = BMA254_SET_BITSLICE(data,
			BMA254_COMP_TARGET_OFFSET_X, offsettarget);
	comres = bma254_smbus_write_byte(client,
			BMA254_COMP_TARGET_OFFSET_X__REG, &data);

	return comres;
}

static int bma254_get_offset_target_x(struct i2c_client *client,
		unsigned char *offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client,
			BMA254_OFFSET_PARAMS_REG, &data);
	data = BMA254_GET_BITSLICE(data, BMA254_COMP_TARGET_OFFSET_X);
	*offsettarget = data;

	return comres;
}

static int bma254_set_offset_target_y(struct i2c_client *client,
		unsigned char offsettarget)
{
	int comres = 0;
	unsigned char data;

	bma254_smbus_read_byte(client,
			BMA254_COMP_TARGET_OFFSET_Y__REG, &data);
	data = BMA254_SET_BITSLICE(data,
			BMA254_COMP_TARGET_OFFSET_Y, offsettarget);
	comres = bma254_smbus_write_byte(client,
			BMA254_COMP_TARGET_OFFSET_Y__REG, &data);

	return comres;
}

static int bma254_get_offset_target_y(struct i2c_client *client,
		unsigned char *offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client,
			BMA254_OFFSET_PARAMS_REG, &data);
	data = BMA254_GET_BITSLICE(data, BMA254_COMP_TARGET_OFFSET_Y);
	*offsettarget = data;

	return comres;
}

static int bma254_set_offset_target_z(struct i2c_client *client,
		unsigned char offsettarget)
{
	int comres = 0;
	unsigned char data;

	bma254_smbus_read_byte(client,
			BMA254_COMP_TARGET_OFFSET_Z__REG, &data);
	data = BMA254_SET_BITSLICE(data,
			BMA254_COMP_TARGET_OFFSET_Z, offsettarget);
	comres = bma254_smbus_write_byte(client,
			BMA254_COMP_TARGET_OFFSET_Z__REG, &data);

	return comres;
}

static int bma254_get_offset_target_z(struct i2c_client *client,
		unsigned char *offsettarget)
{
	int comres = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client,
			BMA254_OFFSET_PARAMS_REG, &data);
	data = BMA254_GET_BITSLICE(data, BMA254_COMP_TARGET_OFFSET_Z);
	*offsettarget = data;

	return comres;
}

static int bma254_get_cal_ready(struct i2c_client *client,
		unsigned char *calrdy)
{
	int comres = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client,
			BMA254_OFFSET_CTRL_REG, &data);
	data = BMA254_GET_BITSLICE(data, BMA254_FAST_COMP_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma254_set_cal_trigger(struct i2c_client *client,
		unsigned char caltrigger)
{
	int comres = 0;
	unsigned char data;

	bma254_smbus_read_byte(client,
			BMA254_EN_FAST_COMP__REG, &data);
	data = BMA254_SET_BITSLICE(data,
			BMA254_EN_FAST_COMP, caltrigger);
	comres = bma254_smbus_write_byte(client,
			BMA254_EN_FAST_COMP__REG, &data);

	return comres;
}


static ssize_t bma254_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	int cal_data[3];
	int err;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;
	int result = 1;

	pr_info("bma254_calibration_show");

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp)) {
		pr_err("[ACC] %s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp, (char *) cal_data,
				3 * sizeof(int), &cal_filp->f_pos);
	if (err != 3 * sizeof(int)) {
		pr_err("[ACC] %s: Can't read the cal data from file\n",
							__func__);
		filp_close(cal_filp, current->files);
		set_fs(old_fs);
		return -EIO;
	}

	if ((cal_data[0] == 0) && (cal_data[1] == 0) && (cal_data[2] == 0))
		result = 0;

	pr_info("bma254_calibration_show %d  %d %d %d\n", result,
				cal_data[0], cal_data[1], cal_data[2]);

	return sprintf(buf, "%d %d %d %d\n", result,
				cal_data[0], cal_data[1], cal_data[2]);
}

static ssize_t bma254_calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct	i2c_client *client = to_i2c_client(dev);
	struct	bma254_data *bma254 = i2c_get_clientdata(client);
	int		cal_data[3] = {0,};
	unsigned long	data;
	signed char	tmp;
	unsigned char	timeout = 0;
        int		err;
	mm_segment_t	old_fs;
	struct file	*cal_filp = NULL;

	err = kstrtoul(buf, 10, &data);
	if (unlikely(err))
		return err;

	pr_info("bma254_calibration_store");
	if (data) {
#if defined(BMA254_MINUS_SIGNED_Z) || defined(CONFIG_MACH_DELOS_CTC)

		/* Check the current z value and
		set offset_target_z based on it */
		short		acc_value_z = 0;
		unsigned char	offset_target_z = 0;
		bma254_read_accel_z(bma254->bma254_client, bma254->sensor_type,
								&acc_value_z);
		pr_info("acc_value_z = [%d], while accel calibration\n",
								acc_value_z);
		if (acc_value_z > 0)
			offset_target_z = 1;
		else
			offset_target_z = 2;
#endif
		/* x axis fast calibration */
		if (unlikely(bma254_set_offset_target_x(bma254->bma254_client,
					(unsigned char)0) < 0))
			return -EINVAL;

		if (bma254_set_cal_trigger(bma254->bma254_client, 1) < 0)
			return -EINVAL;

		do {
			mdelay(2);
			bma254_get_cal_ready(bma254->bma254_client, &tmp);
			timeout++;

			if (timeout == 50) {
				pr_info("get fast calibration ready error\n");
				return -EINVAL;
			};
		} while (tmp == 0);

		/* y axis fast calibration */
		if (unlikely(bma254_set_offset_target_y(bma254->bma254_client,
					(unsigned char)0) < 0))
			return -EINVAL;

		if (bma254_set_cal_trigger(bma254->bma254_client, 2) < 0)
			return -EINVAL;

		do {
			mdelay(2);
			bma254_get_cal_ready(bma254->bma254_client, &tmp);
			timeout++;

			if (timeout == 50) {
				pr_info("get fast calibration ready error\n");
				return -EINVAL;
			};
		} while (tmp == 0);

		/* z axis fast calibration */
#if defined (BMA254_MINUS_SIGNED_Z) || defined(CONFIG_MACH_DELOS_CTC)
		/* Check the current z value and
		set offset_target_z based on it */
		if (unlikely(bma254_set_offset_target_z(bma254->bma254_client,
					offset_target_z) < 0))
			return -EINVAL;
#else
		if (unlikely(bma254_set_offset_target_z(bma254->bma254_client,
					(unsigned char)1) < 0))
			return -EINVAL;
#endif

		if (bma254_set_cal_trigger(bma254->bma254_client, 3) < 0)
			return -EINVAL;

		do {
			mdelay(2);
			bma254_get_cal_ready(bma254->bma254_client, &tmp);
			timeout++;

			if (timeout == 50) {
				pr_info("get fast calibration ready error\n");
				return -EINVAL;
			};
		} while (tmp == 0);

		/* calibration */
		bma254_get_offset_target_x(bma254->bma254_client,
					(unsigned char*)&cal_data[0]);
		bma254_get_offset_target_y(bma254->bma254_client,
					(unsigned char*)&cal_data[1]);
		bma254_get_offset_target_z(bma254->bma254_client,
					(unsigned char*)&cal_data[2]);

		pr_info("[ACC] (%d,%d,%d)", cal_data[0],
					cal_data[1], cal_data[2]);

		old_fs = get_fs();
		set_fs(KERNEL_DS);
		cal_filp = filp_open(CALIBRATION_FILE_PATH,
				O_CREAT | O_TRUNC | O_WRONLY, 0666);
		if (IS_ERR(cal_filp)) {
			pr_err("[ACC] %s: Can't open calibration file\n",
						__func__);
			set_fs(old_fs);
			err = PTR_ERR(cal_filp);
			return err;
		}

		err = cal_filp->f_op->write(cal_filp, (char *)cal_data,
				3 * sizeof(int), &cal_filp->f_pos);
		if (err != 3 * sizeof(int)) {
			pr_err("[ACC] %s: Can't write the cal data to file\n",
						__func__);
			err = -EIO;
		}
		filp_close(cal_filp, current->files);
		set_fs(old_fs);
	}
	else {
		/* erase cal data */
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
		if (IS_ERR(cal_filp)) {
			pr_err("[ACC] %s: Can't open calibration file\n",
				__func__);
			set_fs(old_fs);
			err = PTR_ERR(cal_filp);
			return err;
		}

		cal_data[0] = 0;
		cal_data[1] = 0;
		cal_data[2] = 0;

		err = cal_filp->f_op->write(cal_filp, (char *)cal_data,
			3 * sizeof(int), &cal_filp->f_pos);
		if (err != 3 * sizeof(int)) {
			pr_err("[ACC] %s: Can't write the cal data to file\n",
				__func__);
			err = -EIO;
		}

		filp_close(cal_filp, current->files);
		set_fs(old_fs);

		bma254_set_offset_target_x(bma254->bma254_client,
						(unsigned char)cal_data[0]);
	        bma254_set_offset_target_y(bma254->bma254_client,
						(unsigned char)cal_data[1]);
		bma254_set_offset_target_z(bma254->bma254_client,
						(unsigned char)cal_data[2]);
	}

	pr_info("[ACC] cal_data (%d,%d,%d)===== end", cal_data[0],
					cal_data[1], cal_data[2]);

	return count;
}


static ssize_t bma254_fast_calibration_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (unlikely(bma254_get_offset_target_x(bma254->bma254_client,
						&data) < 0))
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma254_fast_calibration_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_offset_target_x(bma254->bma254_client,
				(unsigned char)data) < 0))
		return -EINVAL;

	if (unlikely(bma254_set_cal_trigger(bma254->bma254_client, 1) < 0))
		return -EINVAL;

	do {
		usleep_range(2000, 3000);
		bma254_get_cal_ready(bma254->bma254_client, &tmp);

		pr_info("wait 2ms and got cal ready flag is %d\n", tmp);
		timeout++;
		if (timeout == 50) {
			pr_err("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	pr_info("x axis fast calibration finished\n");
	return count;
}

static ssize_t bma254_fast_calibration_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (bma254_get_offset_target_y(bma254->bma254_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma254_fast_calibration_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_offset_target_y(bma254->bma254_client,
				(unsigned char)data) < 0))
		return -EINVAL;

	if (bma254_set_cal_trigger(bma254->bma254_client, 2) < 0)
		return -EINVAL;

	do {
		usleep_range(2000, 3000);
		bma254_get_cal_ready(bma254->bma254_client, &tmp);

		pr_info("wait 2ms and got cal ready flag is %d\n", tmp);
		timeout++;
		if (timeout == 50) {
			pr_err("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	pr_info("y axis fast calibration finished\n");
	return count;
}

static ssize_t bma254_fast_calibration_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{


	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	if (unlikely(bma254_get_offset_target_z(bma254->bma254_client,
						&data) < 0))
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma254_fast_calibration_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	error = kstrtoul(buf, 10, &data);
	if (unlikely(error))
		return error;

	if (unlikely(bma254_set_offset_target_z(bma254->bma254_client,
				(unsigned char)data) < 0))
		return -EINVAL;

	if (unlikely(bma254_set_cal_trigger(bma254->bma254_client, 3) < 0))
		return -EINVAL;

	do {
		usleep_range(2000, 3000);
		bma254_get_cal_ready(bma254->bma254_client, &tmp);

		pr_info("wait 2ms and got cal ready flag is %d\n", tmp);
		timeout++;
		if (timeout == 50) {
			pr_err("get fast calibration ready error\n");
			return -EINVAL;
		}
	} while (tmp == 0);

	pr_info("z axis fast calibration finished\n");
	return count;
}

static int bma254_soft_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = BMA254_EN_SOFT_RESET_VALUE;

	comres = bma254_smbus_write_byte(client, BMA254_EN_SOFT_RESET__REG,
					&data);

	return comres;
}

#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
static int bma254_get_motion_interrupt(struct i2c_client *client)
{
	int comres;
	int result = 0;
	unsigned char data;

	comres = bma254_smbus_read_byte(client, BMA254_SLOPE_INT_S__REG,
			&data);

	pr_info("%s: ACC_INT_STATUS %x\n", __func__, data);
	if (data & 0x04)
		result = 1;
	else
		pr_info("%s: Skip!! INTRPT report cause STATUS is not 0x04\n",
								__func__);

        return result;
}

static void bma254_set_motion_interrupt(struct i2c_client *client,
					bool enable, bool factorytest)
{
	struct bma254_data *bma254 = i2c_get_clientdata(client);
	int comres;
        unsigned char data;

	if (enable) {
		bma254_soft_reset(bma254->bma254_client);
		mdelay(20);

		bma254_set_mode(bma254->bma254_client, BMA254_MODE_LOWPOWER);
		mdelay(20);

		bma254_set_bandwidth(client, BMA254_BW_SET);
		bma254_set_range(client, BMA254_RANGE_SET);
		mdelay(5);

		data = 0x04;
		comres = bma254_smbus_write_byte(client,
					BMA254_EN_INT1_PAD_SLOPE__REG, &data);
		mdelay(3);

		/*0x02: Count3(Ignore 2 interrupts), 0x03:Count4*/
		data =  0x02;
		comres = bma254_smbus_write_byte(client, BMA254_SLOPE_DUR__REG,
		                                        &data);
		mdelay(3);
		if (factorytest) {
			data = 0x00;
			comres = bma254_smbus_write_byte(client,
					BMA254_SLOPE_THRES__REG, &data);
		}
		else {
		#if defined(CONFIG_MACH_CS02)
			data = 0x0c;
		#else
			data = 0x0a;
		#endif
		pr_info("bma254_set_motion_interrupt:%x\n",data);
			comres = bma254_smbus_write_byte(client,
					BMA254_SLOPE_THRES__REG, &data);
		}
		mdelay(3);

		data = 0x07;
		comres = bma254_smbus_write_byte(client,
					BMA254_INT_ENABLE1_REG, &data);
	}
	else {
		data = 0x00;
		comres = bma254_smbus_write_byte(client,
					BMA254_EN_INT1_PAD_SLOPE__REG, &data);
		mdelay(2);
		data = 0x03;
		comres = bma254_smbus_write_byte(client,
					BMA254_SLOPE_DUR__REG, &data);
		mdelay(2);
		data = 0x00;
		comres = bma254_smbus_write_byte(client,
					BMA254_INT_ENABLE1_REG, &data);
		mdelay(2);
		data = 0xff;
		comres = bma254_smbus_write_byte(client,
					BMA254_SLOPE_THRES__REG, &data);

		mdelay(2);
		if (atomic_read(&bma254->factory_test)) {
			atomic_set(&bma254->factory_test, 0);
			bma254_set_mode(bma254->bma254_client,
						BMA254_MODE_NORMAL);
		}else {
			bma254_set_mode(bma254->bma254_client,
						BMA254_MODE_SUSPEND);
		}
		mdelay(5);
	}
}
#endif

#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
static ssize_t bma254_reactive_enable_show(struct device *dev,
					struct device_attribute
						*attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);

	pr_info("%s: state =%d onoff=%d\n", __func__,
		atomic_read(&bma254->reactive_state),
		atomic_read(&bma254->reactive_enable));

	return sprintf(buf, "%d\n",
		atomic_read(&bma254->reactive_state));
}

static ssize_t bma254_reactive_enable_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct bma254_data	*bma254 = i2c_get_clientdata(client);
	bool			onoff = false;
	bool			factory_test = false;
	unsigned long		value = 0;
	int			err = count;
#if defined(DEFENCE_REACTIVE_ALERT)
	int			this_cpu;
	unsigned long long	t;
	unsigned long		nanosec_rem;
#endif

	if (kstrtoul(buf, 10, &value)) {
		err = -EINVAL;
		return err;
	}

	pr_info("%s: value=%lu\n", __func__, value);
	switch (value) {
	case 0:
		break;
	case 1:
		onoff = true;
		break;
	case 2:
		onoff = true;
		factory_test = true;
		atomic_set(&bma254->factory_test, 1);
		break;
	default:
		err = -EINVAL;
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return count;
	}

	if (bma254->IRQ) {
		if (!value) {
			disable_irq_wake(bma254->IRQ);
			disable_irq(bma254->IRQ);
		}
	}

	pr_info("%s: =============2 After IRQ\n", __func__);
	mutex_lock(&bma254->data_mutex);
	atomic_set(&bma254->reactive_enable, onoff);
	atomic_set(&bma254->reactive_state, false);

	mdelay(1);
#if defined(DEFENCE_REACTIVE_ALERT)
	this_cpu = raw_smp_processor_id();
	t = cpu_clock(this_cpu);
	nanosec_rem = do_div(t, 1000000000);
	t_before = t;
	nanosec_before = nanosec_rem;
#endif
	if (bma254->IRQ)
		bma254_set_motion_interrupt(bma254->bma254_client, onoff,
							factory_test);
	mdelay(1);

	pr_info("%s: =============3 After Set Motion\n", __func__);
	mutex_unlock(&bma254->data_mutex);

	pr_info("%s: onoff = %d, state =%d \n", __func__,
		atomic_read(&bma254->reactive_enable),
		atomic_read(&bma254->reactive_state));

	if (bma254->IRQ) {
		if (value) {
			enable_irq(bma254->IRQ);
			enable_irq_wake(bma254->IRQ);
		}
	}

	return count;
}
#endif

static ssize_t bma254_accel_name_show(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", SENSOR_NAME);
}

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_range_show, bma254_range_store);
static DEVICE_ATTR(bandwidth, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_bandwidth_show, bma254_bandwidth_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_mode_show, bma254_mode_store);
static DEVICE_ATTR(value, S_IRUGO,
		bma254_value_show, NULL);
static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_delay_show, bma254_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_enable_show, bma254_enable_store);
static DEVICE_ATTR(update,S_IWUSR|S_IWGRP,
		NULL, bma254_update_store);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_selftest_show, bma254_selftest_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_fast_calibration_x_show,
		bma254_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_fast_calibration_y_show,
		bma254_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_fast_calibration_z_show,
		bma254_fast_calibration_z_store);
static DEVICE_ATTR(calibration, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_calibration_show,
		bma254_calibration_store);
static DEVICE_ATTR(name, S_IRUGO, bma254_accel_name_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, bma254_value_show, NULL);
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
static DEVICE_ATTR(reactive_alert, S_IRUGO|S_IWUSR|S_IWGRP,
		bma254_reactive_enable_show, bma254_reactive_enable_store);
#endif

static struct device_attribute *bma254_attributes_sensors[] = {
	&dev_attr_range,
	&dev_attr_bandwidth,
	&dev_attr_mode,
	&dev_attr_value,
	&dev_attr_poll_delay,
	&dev_attr_enable,
	&dev_attr_update,
	&dev_attr_selftest,
	&dev_attr_fast_calibration_x,
	&dev_attr_fast_calibration_y,
	&dev_attr_fast_calibration_z,
	&dev_attr_calibration,
	&dev_attr_raw_data,
	&dev_attr_name,
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	&dev_attr_reactive_alert,
#endif
	NULL
};

static struct attribute *bma254_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_update.attr,
	&dev_attr_selftest.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	&dev_attr_reactive_alert.attr,
#endif
	NULL
};

static struct attribute_group bma254_attribute_group = {
	.attrs = bma254_attributes
};


#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
static void bma254_work_func_alert(struct work_struct *work)
{
	struct bma254_data *bma254 = container_of(work,
			struct bma254_data, alert_work);
	int result;
#if defined(DEFENCE_REACTIVE_ALERT)
	int this_cpu;
	unsigned long long t;
	unsigned long nanosec_rem;
#endif

	result = bma254_get_motion_interrupt(bma254->bma254_client);
	if (result || bma254->factory_mode) {
		/* handle motion recognition */
		mdelay(10);

#if defined(DEFENCE_REACTIVE_ALERT)
		this_cpu = raw_smp_processor_id();
		t = cpu_clock(this_cpu);
		nanosec_rem = do_div(t, 1000000000);
		if ((t-t_before)<2) {
			unsigned long long calc;
			calc = (unsigned long long)((t-t_before) * 1000 +
						(nanosec_rem/1000000)) -
				(unsigned long long)(nanosec_before/1000000);
			if (calc < 750) {
				pr_info("%s: But (t-t_before=calc=%llu) is \
					too short!!!!! \n", __func__, calc);
				return;
			}
		}
#endif
		atomic_set(&bma254->reactive_state, true);
		bma254->factory_mode = false;
		pr_info("%s: motion interrupt happened\n", __func__);
		__pm_wakeup_event(&bma254->reactive_wakeup_source, 2000);
	}

	pr_info("%s: =============4\n", __func__);
}

irqreturn_t bma254_acc_irq_thread(int irq, void *dev)
{
	struct bma254_data *data = dev;
	schedule_work(&data->alert_work);
	return IRQ_HANDLED;
}
#endif

static int bma254_input_init(struct bma254_data *bma254)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;
	dev->name = "accelerometer_sensor";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
	input_set_drvdata(dev, bma254);

	err = input_register_device(dev);
	if (err < 0) {
		pr_err("bma254_input_init input_register_device=%d\n", err);
		input_free_device(dev);
		return err;
	}

	bma254->input = dev;

	return 0;
}

static void bma254_input_delete(struct bma254_data *bma254)
{
	struct input_dev *dev = bma254->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
static int bma254_setup_irq(struct bma254_data *bma254)
{
	int rc = -EIO;
	int irq;

	irq = bma254->bma254_client->irq;
	if (irq > 0) {
		rc = request_threaded_irq(irq,	NULL, bma254_acc_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT ,
			"accelerometer", bma254);

		pr_info("%s: irq = %d\n", __func__, irq);

		if (rc < 0) {
			pr_err("%s request_threaded_irq fail err=%d\n",
				__func__, rc);
			return rc;
		}
		/* start with interrupts disabled */
		disable_irq(irq);
	}

	bma254->IRQ = irq;
	return rc;
}
#endif

int bma254_device_power(struct device *dev, bool on)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma254_data *bma254 = i2c_get_clientdata(client);
	struct bma254_platform_data *pdata = bma254->pdata;
	static bool bFirst = 1;
	static bool bStatus;
	int min_uv, max_uv;
	int ret = 0;

	if (bFirst) {
		/* VDD Power On */
		if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {
			pdata->vdd_regulator = regulator_get(dev,
							"bma254_vdd");
			if (IS_ERR(pdata->vdd_regulator)) {
				dev_err(dev, "%s: vdd_regulator get error!\n",
								__func__);
				pdata->vdd_regulator = NULL;
				return -1;
			}

			max_uv = min_uv = pdata->vdd_regulator_volt;
			ret = regulator_set_voltage(pdata->vdd_regulator,
							min_uv, max_uv);
			if (ret) {
				dev_err(dev, "%s: error setting vdd regulator \
					voltage to %d %d, returns %d\n",
					__func__, min_uv, max_uv, ret);
			}
		}

		if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {
			ret = gpio_direction_output(pdata->vdd_ldo_en, 0);
                        if (ret) {
                                dev_err(dev, "error setting direction \
						(output) for vdd_ldo_en\n");
                                return ret;
			}
		}

		bFirst = 0;
	}

	if (on == bStatus)
		return 0;

	if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = (on) ? regulator_enable(pdata->vdd_regulator) :
				regulator_disable(pdata->vdd_regulator);
		if (ret != 0)
			goto error;
		usleep_range(2000, 2000);
	}

	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {
		(on) ? gpio_direction_output(pdata->vdd_ldo_en, 1) :
				gpio_direction_output(pdata->vdd_ldo_en, 0);
	}

	bStatus = (on) ? 1 : 0;

	return 0;

error:
	pr_err("%s: error %s regulator!, ret = %d\n", __func__,
			(on) ? "enabling" : "disabling", ret);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id bma254_dt_ids[] __initconst = {
	{ .compatible = "bosch,bma254", },
	{},
};
MODULE_DEVICE_TABLE(of, bma254_dt_ids);
#endif

static int bma254_probe_dt(struct device_node *np, struct device *dev,
				struct bma254_platform_data *pdata) {

	const struct of_device_id *match;
	int ret;

	if (!np) {
		dev_err(dev, "%s: device node is NULL!!\n", __func__);
		return -ENODEV;
	}

	match = of_match_device(bma254_dt_ids, dev);
	if (!match) {
		dev_err(dev, "%s: compatible mismatch!!\n", __func__);
		return -ENODEV;
	}

	pdata->gpio_int = of_get_named_gpio(np, "bma254,gpio_int", 0);
	if (!pdata->gpio_int) {
		dev_err(dev, "%s: error reading property gpio interrupt \
						from DT node\n", __func__);
		ret = pdata->gpio_int;
	}

	ret = of_property_read_u32(np, "bma254,accel_position",
						&pdata->accel_position);
	if (ret) {
		dev_err(dev, "%s: error reading property accel_position \
						from DT node\n", __func__);
		return ret;
	}

	pdata->axis_adjust = of_property_read_bool(np, "bma254,axis_adjust");
	pdata->power = bma254_device_power;

	pdata->vdd_supply_type = -1;
	ret = of_property_read_u32(np, "bma254,vdd_supply_type",
						&pdata->vdd_supply_type);
	if (unlikely(ret)) {
		dev_warn(dev, "No supply_type found in device node\n");
		return 0;
	}

	if (pdata->vdd_supply_type == REGULATOR_SUPPLY_TYPE) {

		ret = of_property_read_u32(np, "bma254,vdd_regulator_volt",
						&pdata->vdd_regulator_volt);
		if (unlikely(ret)) {
			dev_err(dev, "error reading property regulator_volt \
							from device node\n");
			goto error;
		}
	}
	else if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {

		pdata->vdd_ldo_en = of_get_named_gpio(np,
						"bma254,vdd_ldo_en", 0);
                if (unlikely(pdata->vdd_ldo_en < 0)) {
                        dev_err(dev, "error reading property vdd_ldo_en \
							from device node\n");
                        ret = pdata->vdd_ldo_en;
                        goto error;
                }
	}

	return 0;

error:
	return ret;
}

static int bma254_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma254_data *data;
	struct bma254_platform_data *pdata = client->dev.platform_data;

	struct device_node *np = client->dev.of_node;

	pr_info("%s, is called, acceleometer\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality error\n");
		goto exit;
	}

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = kzalloc(sizeof(struct bma254_platform_data),
								GFP_KERNEL);
			if (unlikely(!pdata)) {
				pr_err("error allocation memory\n");
				err = -ENOMEM;
				goto exit;
			}
		}
		err = bma254_probe_dt(np, &client->dev, pdata);
		if (unlikely(err))
			goto kfree_pdata;
	}

	if (unlikely(!pdata->power)) {
		pr_err("%s: incomplete pdata!\n", __func__);
		goto kfree_pdata;
	}

	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE) {

		err = gpio_request(pdata->vdd_ldo_en, "bma254_vdd_ldo_en");
		if (unlikely(err)) {
			dev_err(&client->dev, "%s: error requesting \
				vdd ldo gpio %d (%d)", __func__,
				pdata->vdd_ldo_en, err);
			goto kfree_pdata;
		}
		gpio_direction_output(pdata->vdd_ldo_en, 0);
	}

	err = gpio_request(pdata->gpio_int, "bma254_int");
	if (unlikely(err)) {
		dev_err(&client->dev, "%s: error requesting gpio \
				int %d (%d)", __func__, pdata->gpio_int, err);
		goto kfree_pdata;
	}
	gpio_direction_input(pdata->gpio_int);

	data = kzalloc(sizeof(struct bma254_data), GFP_KERNEL);
	if (unlikely(!data)) {
		err = -ENOMEM;
		goto gpio_free;
	}

	i2c_set_clientdata(client, data);
	data->bma254_client = client;
	data->pdata = pdata;

	err = pdata->power(&client->dev, true);
	if (unlikely(err)) {
		dev_err(&client->dev, "%s: error powering on bma245 \
				chip, (%d)\n", __func__, err);
		goto kfree_exit;
	}
	msleep(20);

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	mutex_init(&data->data_mutex);
#endif

	/* read chip id */
	tempvalue = 0;
	tempvalue = i2c_smbus_read_word_data(client, BMA254_CHIP_ID_REG);

	if ((tempvalue&0x00FF) == BMA254_CHIP_ID) {
		pr_info("Bosch Sensortec Device detected!\n \
				BMA254 registered I2C driver!\n");
	} else {
		pr_err("Bosch Sensortec Device not found, \
				i2c error %d\n", tempvalue);
		err = -1;
		goto error_i2c;
	}

	bma254_set_bandwidth(client, BMA254_BW_SET);
	bma254_set_range(client, BMA254_RANGE_SET);

	/* accelerometer position set */
	if (!pdata) {
		/*Set by default position 1, it doesn't adjust raw value*/
		data->position = 1;
		data->axis_adjust = false;
		pr_err("using defualt position = %d\n", data->position);
	} else {
		data->position = pdata->accel_position;
		data->axis_adjust = pdata->axis_adjust;
		pr_info("successful, position = %d\n", data->position);
	}

#ifdef	BMA254_LOGGING
	pr_info("%s, ## data->position = %d\n", __func__, data->position);
#endif
	INIT_DELAYED_WORK(&data->work, bma254_work_func);
	atomic_set(&data->delay, BMA254_DEFAULT_DELAY);
	atomic_set(&data->enable, 0);

#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT

	INIT_WORK(&data->alert_work, bma254_work_func_alert);
	wakeup_source_init(&data->reactive_wakeup_source, 
						"reactive_wakeup_source");
	err = bma254_setup_irq(data);
	if (err) {
		pr_err("%s: could not setup irq\n", __func__);
		goto exit_bma254_acc_alert_int;
	}
	atomic_set(&data->factory_test, 0);
#endif

	err = bma254_input_init(data);
	if (err < 0)
		goto error_input_init;

	err = sysfs_create_group(&data->input->dev.kobj,
			&bma254_attribute_group);
	if (err < 0)
		goto error_sysfs;


	err = sensors_register(&data->bma254_device, data,
					bma254_attributes_sensors,
					"accelerometer_sensor");
	if (err < 0) {
		pr_info("%s: could not sensors_register\n", __func__);
		goto error_register;
	}

	pr_info("%s success\n", __func__);

	return 0;

error_register:
	sysfs_remove_group(&data->input->dev.kobj, &bma254_attribute_group);
error_sysfs:
	bma254_input_delete(data);
error_input_init:
	free_irq(data->IRQ, data);
exit_bma254_acc_alert_int:
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	wakeup_source_trash(&data->reactive_wakeup_source);
#endif
error_i2c:
	mutex_destroy(&data->value_mutex);
	mutex_destroy(&data->mode_mutex);
	mutex_destroy(&data->enable_mutex);
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	mutex_destroy(&data->data_mutex);
#endif
	data->pdata->power(&client->dev, false);
kfree_exit:
	kfree(data);
gpio_free:
	if (pdata->vdd_supply_type == LDO_SUPPLY_TYPE)
		gpio_free(pdata->vdd_ldo_en);
	gpio_free(pdata->gpio_int);
kfree_pdata:
#ifdef CONFIG_OF
	kfree(pdata);
#endif
exit:
	return err;
}

static int bma254_remove(struct i2c_client *client)
{
	struct bma254_data *data = i2c_get_clientdata(client);

	bma254_set_enable(&client->dev, 0);
	sysfs_remove_group(&data->input->dev.kobj, &bma254_attribute_group);

	device_remove_file(data->dev, &dev_attr_name);
	device_remove_file(data->dev, &dev_attr_raw_data);

	bma254_input_delete(data);

	free_irq(data->IRQ, data);
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	wakeup_source_trash(&data->reactive_wakeup_source);
#endif
	mutex_destroy(&data->value_mutex);
	mutex_destroy(&data->mode_mutex);
	mutex_destroy(&data->enable_mutex);
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	mutex_destroy(&data->data_mutex);
#endif
	if (data->pdata->vdd_supply_type == LDO_SUPPLY_TYPE)
		gpio_free(data->pdata->vdd_ldo_en);
	gpio_free(data->pdata->gpio_int);
	data->pdata->power(&client->dev, false);
#if CONFIG_OF
	kfree(data->pdata);
#endif
	kfree(data);
	return 0;
}

static const struct i2c_device_id bma254_id[] __initconst = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma254_id);

static struct i2c_driver bma254_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(bma254_dt_ids),
#endif
	},
	.id_table		= bma254_id,
	.probe			= bma254_probe,
	.remove			= bma254_remove,

};

static int __init BMA254_init(void)
{
	pr_info(KERN_ERR "=====BMA254_init =====\n");

	return i2c_add_driver(&bma254_driver);
}

static void __exit BMA254_exit(void)
{
	i2c_del_driver(&bma254_driver);
}

MODULE_AUTHOR("Albert Zhang <xu.zhang@bosch-sensortec.com>");
MODULE_DESCRIPTION("BMA254 driver");
MODULE_LICENSE("GPL");

module_init(BMA254_init);
module_exit(BMA254_exit);
