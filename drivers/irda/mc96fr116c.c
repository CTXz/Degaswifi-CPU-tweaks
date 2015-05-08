/*
 * driver/irda IRDA driver
 *
 * Copyright (C) 2012, 2013 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_data/mc96fr116c.h>
#include <linux/regulator/consumer.h>
#include "mc96fr116c_fw.h"
#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include <mach/gpio.h>
#include <linux/sec-common.h>
#include <plat/mfp.h>

#define MAX_SIZE		2048
#define MC96FR116C_READ_LENGTH	8
#define DUMMY			0xffff

#define MC96FR116C_RESET_BOOTLOADER_MODE	0
#define MC96FR116C_RESET_USER_IR_MODE		1

#define MC96FR116C_UNKNOWN_MODE		0
#define MC96FR116C_BOOTLOADER_MODE	1
#define MC96FR116C_USER_IR_MODE		2

/* Delays for both mode of them */
#define MC96FR116C_START_FIRMWARE_DELAY_MS		75
#define MC96FR116C_I2C_RECEIVE_DELAY_MS			25

/* Delays for bootloader mode */
#define MC96FR116C_WATCHDOG_RESET_DELAY_MS		50
#define MC96FR116C_CHECKSUM_CALCULATION_DELAY_MS	30
#define MC96FR116C_ERASE_PROGRAM_CODE_DELAY_MS		40
#define MC96FR116C_RETRY_UPDATING_FW_DELAY_MS		100

/* Delays for user IR mode */
#define MC96FR116C_WAKEUP_STABLE_DELAY_MS		21
#define MC96FR116C_IDLE_MODE_WAIT_DELAY_MS		50
#define MC96FR116C_WAKE_ACTIVE_LOW_DELAY_US		100
#define MC96FR116C_RETRY_I2C_WRITE_DELAY_MS		10
#define MC96FR116C_ENTER_IDLE_STATE_DELAY_MS		400

#define IRDA_RDY_PULL_HIGH	(MFP_CFG(SM_nCS1, AF1) | MFP_PULL_HIGH)
#define IRDA_RDY_PULL_LOW		(MFP_CFG(SM_nCS1, AF1) | MFP_PULL_LOW)

struct mc96fr116c_data {
	struct i2c_client		*client;
	struct mc96fr116c_platform_data	*pdata;

	struct mutex			signal_mutex;
	u8 signal[MAX_SIZE];
	int signal_length;
	int carrier_freq;
	int ir_sum;

	int dev_id;
	int mode;
	bool last_send_ok;
};

static const u8 INITIAL_BOOT_MODE_DATA[MC96FR116C_READ_LENGTH] = {
	0x00, 0x01, 0x3F, 0xFF, 0x10, 0x00, 0x01, 0x4F,
};
static const u8 CHECKSUM_CALCULATION_COMMAND[8] = {
	0x3A, 0x02, 0x10, 0x00, 0xF0, 0x20, 0xFF, 0xDF,
};
static const u8 SW_RESET_COMMAND[8] = {
	0x3A, 0x02, 0x10, 0x00, 0xF3, 0x10, 0x00, 0xEB,
};


static void irda_wake_en(struct device *dev, bool onoff)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	
	gpio_set_value(data->pdata->irda_wake_en, onoff);
}

static int irda_hwreset(struct device *dev)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);
	
	gpio_set_value(data->pdata->irda_rst_gpio, 0);
	msleep(20);
	gpio_direction_output(data->pdata->irda_wake_en, 0); /* Bootloader mode */
	gpio_set_value(data->pdata->irda_rst_gpio, 1);
	msleep(53);
	gpio_direction_output(data->pdata->irda_wake_en, 1); /* Not for user IR mode */

	return 0;
}

static int irda_read_ready(struct device *dev)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);
	return gpio_get_value(data->pdata->irda_irq_gpio);
}

static bool mc96fr116c_wakeup_stop_mode(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int retry = 10;

	if (data->mode == MC96FR116C_BOOTLOADER_MODE) {
		dev_warn(&client->dev, "%s: No need wake up in not Bootloader mode\n",
				__func__);
		return true;
	} else if (data->mode == MC96FR116C_UNKNOWN_MODE) {
		dev_warn(&client->dev, "%s: Try to enter idle state\n",
				__func__);
		data->pdata->ir_wake_en(&client->dev, 1);
		msleep(MC96FR116C_ENTER_IDLE_STATE_DELAY_MS);
	}

	if (data->pdata->ir_read_ready(&client->dev))
		return true;

	data->pdata->ir_wake_en(&client->dev, 0);
	msleep(MC96FR116C_WAKEUP_STABLE_DELAY_MS);
	do {
		udelay(MC96FR116C_WAKE_ACTIVE_LOW_DELAY_US);
		if (data->pdata->ir_read_ready(&client->dev)) {
			data->pdata->ir_wake_en(&client->dev, 1);
			return true;
		}
		dev_warn(&client->dev, "%s: retry %d for idle state\n",
			 __func__, retry);
	} while (retry--);

	dev_err(&client->dev, "%s: Can not wake up\n", __func__);

	return false;
}

static bool mc96fr116c_make_idle_state(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int retry = 10;

	data->pdata->ir_wake_en(&client->dev, 1);
	do {
		udelay(MC96FR116C_WAKE_ACTIVE_LOW_DELAY_US);

		if (data->pdata->ir_read_ready(&client->dev))
			return true;

		dev_warn(&client->dev, "%s: retry %d for idle state\n",
			 __func__, retry);

		msleep(MC96FR116C_IDLE_MODE_WAIT_DELAY_MS);
	} while (retry--);

	dev_err(&client->dev, "%s: Can not make idle state\n", __func__);

	return false;
}

static int mc96fr116c_send_command(struct mc96fr116c_data *data,
		const u8 *command, int len)
{
	struct i2c_client *client = data->client;
	int ret;

	if (data->mode == MC96FR116C_USER_IR_MODE)
		mc96fr116c_wakeup_stop_mode(data);

	ret = i2c_master_send(client, command, len);

	if (ret == len) {
		ret = 0;
	} else  {
		dev_err(&client->dev, "%s: i2c communication error %d\n",
				__func__, ret);
		print_hex_dump_bytes("command: ", DUMP_PREFIX_NONE,
				command, len);
		ret = -EIO;
	}

	return ret;
}

static bool mc96fr116c_is_valid_code_checksum(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int ret;
	u8 i2c_buf[MC96FR116C_READ_LENGTH];

	if (data->mode != MC96FR116C_BOOTLOADER_MODE) {
		dev_warn(&client->dev, "%s: No bootloader mode\n", __func__);
		return false;
	}

	ret = i2c_master_recv(client, i2c_buf, sizeof(i2c_buf));
	msleep(MC96FR116C_I2C_RECEIVE_DELAY_MS);
	if (ret < 0 || ret != sizeof(i2c_buf)) {
		dev_err(&client->dev, "%s: i2c communication error %d\n",
				__func__, ret);

		goto out;
	}

	if (!memcmp(i2c_buf, VALID_CODE_CHECKSUM, sizeof(VALID_CODE_CHECKSUM)))
		return true;

	print_hex_dump_bytes("Checksum error: ", DUMP_PREFIX_NONE,
			i2c_buf, sizeof(i2c_buf));
out:
	return false;
}

static bool mc96fr116c_ensure_user_ir_mode(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int ret;
	u8 i2c_buf[MC96FR116C_READ_LENGTH];

	ret = i2c_master_recv(client, i2c_buf, sizeof(i2c_buf));
	msleep(MC96FR116C_I2C_RECEIVE_DELAY_MS);
	if (ret < 0 || ret != sizeof(i2c_buf)) {
		dev_err(&client->dev, "%s: i2c communication error %d\n",
				__func__, ret);

		goto out;
	}

	if (!memcmp(i2c_buf, INITIAL_USER_IR_MODE_DATA,
				sizeof(INITIAL_USER_IR_MODE_DATA))) {
		data->dev_id = i2c_buf[4] << BITS_PER_BYTE | i2c_buf[5];
		data->mode = MC96FR116C_USER_IR_MODE;
		mc96fr116c_make_idle_state(data);
		if (!ret)
			dev_warn(&client->dev, "%s: Can not make idle state.\n",
					__func__);
		return true;
	}

	print_hex_dump_bytes("NOT User IR Mode: ", DUMP_PREFIX_NONE,
			i2c_buf, sizeof(i2c_buf));
out:
	data->mode = MC96FR116C_UNKNOWN_MODE;
	return false;
}

static int mc96fr116c_reset_sw(struct mc96fr116c_data *data, int entry_mode)
{
	int ret;
	struct i2c_client *client = data->client;

	if (data->mode == MC96FR116C_USER_IR_MODE) {
		if (!data->pdata->ir_reset(&client->dev)) {
			dev_err(&data->client->dev, "%s: Not support sw reset in user IR mode.\n",
					__func__);
			return -EINVAL;
		}

		if (entry_mode == MC96FR116C_RESET_USER_IR_MODE)
			data->pdata->ir_wake_en(&client->dev, 1);	/* USER IR MODE */
		else if (entry_mode == MC96FR116C_RESET_BOOTLOADER_MODE)
			data->pdata->ir_wake_en(&client->dev, 0);	/* BOOTLOADER MODE */

		data->pdata->ir_reset(&client->dev);
		msleep(MC96FR116C_START_FIRMWARE_DELAY_MS);
		data->pdata->ir_wake_en(&client->dev, 1);	/* Not for USER IR MODE */

		if (entry_mode == MC96FR116C_RESET_USER_IR_MODE) {
			ret = mc96fr116c_ensure_user_ir_mode(data);

			if (!ret) {
				dev_err(&data->client->dev, "%s: HW Reset complete. But It was not entered to User IR mode.\n",
						__func__);
				return -ENODEV;
			}
		}

		return 0;
	}

	ret = mc96fr116c_send_command(data, SW_RESET_COMMAND,
			sizeof(SW_RESET_COMMAND));
	if (ret < 0)
		return ret;

	if (entry_mode == MC96FR116C_RESET_USER_IR_MODE) {
		data->pdata->ir_wake_en(&client->dev, 1);	/* USER IR MODE */
		msleep(MC96FR116C_WATCHDOG_RESET_DELAY_MS);
		msleep(MC96FR116C_START_FIRMWARE_DELAY_MS);

		ret = mc96fr116c_ensure_user_ir_mode(data);
		if (!ret) {
			dev_err(&data->client->dev, "%s: Reset complete. But It was not entered to User IR mode.\n",
					__func__);
			return -EINVAL;
		}
	}
	return 0;
}

static int mc96fr116c_check_need_fw_update(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	u8 i2c_buf[MC96FR116C_READ_LENGTH];
	int ret;
	int retry_count;
	bool boot_condition_ok = false;

	//mc96fr116c_wakeup_stop_mode(data);
	for (retry_count = 0; retry_count < 5; retry_count++) {
		ret = i2c_master_recv(client, i2c_buf, sizeof(i2c_buf));
		msleep(MC96FR116C_I2C_RECEIVE_DELAY_MS);
		if (ret < 0 || ret != sizeof(i2c_buf)) {
			dev_err(&client->dev, "%s: i2c communication error %d\n",
					__func__, ret);
			continue;
		}

		if (!memcmp(INITIAL_BOOT_MODE_DATA, i2c_buf,
					MC96FR116C_READ_LENGTH)) {
			dev_info(&client->dev, "%s: Bootloader mode\n",
					__func__);
			dev_dbg(&client->dev, "%s: Start address is 3FFF\n",
					__func__);
			boot_condition_ok = true;
			data->mode = MC96FR116C_BOOTLOADER_MODE;
			break;
		} else if (!memcmp(INITIAL_USER_IR_MODE_DATA, i2c_buf,
					MC96FR116C_READ_LENGTH)) {
			dev_info(&client->dev, "%s: User IR mode, F/W version is V%d.%d",
					__func__, i2c_buf[2], i2c_buf[3]);

			data->mode = MC96FR116C_USER_IR_MODE;
			ret = mc96fr116c_reset_sw(data,
					MC96FR116C_RESET_BOOTLOADER_MODE);
			if (!ret)
				return ret;

			if (ret == -EINVAL && !data->pdata->ir_reset(&client->dev))
				return 0;

			retry_count--;
			continue;
		}
		dev_warn(&client->dev, "%s: Retry i2c read %d\n", __func__,
				retry_count);
	}

	if (!boot_condition_ok) {
		if (ret == sizeof(i2c_buf)) {
			dev_err(&client->dev, "%s: The start address is NOT 3FFFh\n",
					__func__);
			print_hex_dump_bytes("Received data: ",
					DUMP_PREFIX_NONE,
					i2c_buf, sizeof(i2c_buf));
			ret = -EINVAL;
		}
		goto error;
	}

	for (retry_count = 0; retry_count < 5; retry_count++) {
		ret = mc96fr116c_send_command(data,
				CHECKSUM_CALCULATION_COMMAND,
				sizeof(CHECKSUM_CALCULATION_COMMAND));

		msleep(MC96FR116C_CHECKSUM_CALCULATION_DELAY_MS);
		if (!ret)
			break;

		dev_warn(&client->dev, "%s: Retry %d i2c write, ret: %d\n",
				__func__, retry_count, ret);
	}

	if (ret < 0)
		goto error;

	msleep(MC96FR116C_CHECKSUM_CALCULATION_DELAY_MS);
	ret = mc96fr116c_is_valid_code_checksum(data);
	if (!ret) {
		ret = -EINVAL;
		goto error;
	}

	return 0;
error:
	return ret;
}

static int __mc96fr116c_update_fw(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int i;
	int ret;

	for (i = 0; i < FRAME_COUNT - 1; i++) {
		ret = i2c_master_send(client, &IRDA_binary[i * ONE_PACKET_SIZE],
				ONE_PACKET_SIZE);
		if (ret < 0 || ret != ONE_PACKET_SIZE) {
			dev_err(&client->dev, "%s: Error sending packet %d.\n",
					__func__, i);
			return -EIO;
		}

		msleep(MC96FR116C_ERASE_PROGRAM_CODE_DELAY_MS);
	}

	ret = i2c_master_send(client, &IRDA_binary[i * ONE_PACKET_SIZE], 6);
	if (ret != 6) {
		dev_err(&client->dev, "%s: Error sending packet %d.\n",
				__func__, i);
		return -EIO;
	}

	msleep(MC96FR116C_ERASE_PROGRAM_CODE_DELAY_MS);
	msleep(MC96FR116C_CHECKSUM_CALCULATION_DELAY_MS);

	ret = mc96fr116c_is_valid_code_checksum(data);
	if (!ret) {
		dev_err(&client->dev, "%s: Updating complete, but checksum error.\n",
				__func__);
		return -EIO;
	}

	return 0;
}

static int mc96fr116c_update_fw(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int retry_count;
	int ret;

	dev_info(&client->dev, "%s: Firmware update start!\n", __func__);

	for (retry_count = 0; retry_count < 5; retry_count++) {
		ret = __mc96fr116c_update_fw(data);
		if (!ret)
			return 0;

		dev_warn(&client->dev, "%s: Retry %d for updating fw.\n",
				__func__, retry_count);
		msleep(MC96FR116C_RETRY_UPDATING_FW_DELAY_MS);
	}

	dev_err(&client->dev, "%s: Failed to update F/W\n", __func__);

	return ret;
}

static void mc96fr116c_push_checksum(struct mc96fr116c_data *data)
{
	int i;
	int signal_length = data->signal_length;
	u16 checksum = 0;

	data->signal[0] = (signal_length >> BITS_PER_BYTE) & 0xFF;
	data->signal[1] = signal_length & 0xFF;

	for (i = 0; i < signal_length; i++)
		checksum += data->signal[i];

	dev_dbg(&data->client->dev, "%s: checksum: %04X\n", __func__, checksum);

	data->signal[signal_length]	= checksum >> BITS_PER_BYTE;
	data->signal[signal_length + 1]	= checksum & 0xFF;

	data->signal_length += 2;
}

static int mc96fr116c_read_device_id(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	u8 i2c_buf[8];
	int ret;

	if (data->mode != MC96FR116C_USER_IR_MODE) {
		dev_err(&client->dev, "%s: Reading the device id was proceeded in not User IR mode.\n",
				__func__);

		return -EINVAL;
	}

	mc96fr116c_wakeup_stop_mode(data);
	ret = i2c_master_recv(client, i2c_buf, MC96FR116C_READ_LENGTH);
	msleep(MC96FR116C_I2C_RECEIVE_DELAY_MS);
	if (ret != MC96FR116C_READ_LENGTH) {
		dev_err(&client->dev, "%s: i2c communication error %d\n",
				__func__, ret);
		return ret;
	}

	if (memcmp(i2c_buf, INITIAL_USER_IR_MODE_DATA,
				sizeof(INITIAL_USER_IR_MODE_DATA))) {
		dev_warn(&client->dev, "%s: Need Firmware update.\n", __func__);
		print_hex_dump_bytes("i2c_buf: ", DUMP_PREFIX_NONE, i2c_buf,
				sizeof(i2c_buf));
	}

	data->dev_id = i2c_buf[4] << BITS_PER_BYTE | i2c_buf[5];
	dev_info(&client->dev, "Device ID: 0x%04X\n", data->dev_id);

	ret = mc96fr116c_make_idle_state(data);
	if (!ret)
		dev_warn(&client->dev, "%s: Can not make idle state.\n",
				__func__);

	return 0;
}

static int mc96fr116c_send_signal(struct mc96fr116c_data *data)
{
	struct i2c_client *client = data->client;
	int buf_size = data->signal_length + 2;
	int retry;
	int ret;
	bool send_checksum_ok = false;
	int sleep_timing;
	int end_data;
	int emission_time;

	dev_dbg(&client->dev, "%s: total buf_size: %d\n", __func__, buf_size);

	gpio_set_value(data->pdata->irda_led_en, 1);

	mc96fr116c_push_checksum(data);

	for (retry = 2; retry; retry--) {
		mc96fr116c_wakeup_stop_mode(data);
		ret = i2c_master_send(client, data->signal, buf_size);

		msleep(MC96FR116C_RETRY_I2C_WRITE_DELAY_MS);

		if (ret == buf_size)
			break;

		if (ret < 0)
			dev_err(&client->dev, "%s: i2c send failed. Retry %d\n",
					__func__, retry);
	}

	if (ret < 0) {
		data->last_send_ok = false;

		return ret;
	}

	for (retry = 10; retry; msleep(1), retry--) {
		if (!data->pdata->ir_read_ready(&client->dev)) {
			send_checksum_ok = true;
			break;
		}
		dev_warn(&client->dev, "%s: Checking checksum. Retry %d\n",
				__func__, retry);
	}

	end_data = data->signal[data->signal_length - 2] << 8
		| data->signal[data->signal_length - 1];
	emission_time = (1000 * (data->ir_sum - end_data)
			/ (data->carrier_freq))
		+ 10;
	sleep_timing = emission_time - 130;
	if (sleep_timing > 0)
		msleep(sleep_timing);
	emission_time = (1000 * (data->ir_sum) / (data->carrier_freq)) + 50;
	if (emission_time > 0)
		msleep(emission_time);

	for (retry = 10; retry; msleep(100), retry--) {
		if (data->pdata->ir_read_ready(&client->dev)) {
			dev_dbg(&client->dev, "%s: ir_read_ready Okay\n",
					__func__);
			break;
		}
		dev_warn(&client->dev, "%s: Wating for ir_read_ready. Retry %d\n",
				__func__, retry);
	}

	if (data->pdata->ir_read_ready(&client->dev)) {
		if (send_checksum_ok) {
			data->last_send_ok = true;
			ret = 0;
		} else {
			dev_err(&client->dev,
					"%s: Sending IR Checksum failed\n",
					__func__);
			ret = -EIO;
		}
	} else {
		dev_err(&client->dev, "%s: Sending IR NG!\n", __func__);
		data->last_send_ok = false;

		ret = -EIO;
	}

	gpio_set_value(data->pdata->irda_led_en, 0);
	data->signal_length = 0;
	data->carrier_freq = 0;
	data->ir_sum = 0;

	return ret;
}

static ssize_t mc96fr1196c_ir_send_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);
	unsigned int value;
	char *string, *pstring;
	int signal_length = 2; /* By default, 2 bytes for data length */
	int ir_sum = 0;
	int number_freq = 0;
	int ret;

	if (data->mode != MC96FR116C_USER_IR_MODE) {
		dev_err(&data->client->dev, "%s: Not user IR mode.\n",
				__func__);
		return -EINVAL;
	}

	if (!mutex_trylock(&data->signal_mutex))
		return -EBUSY;

	string = (char*)kmalloc(size + 1, GFP_KERNEL);
	strlcpy(string, buf, size);
	pstring = strim(string);

	while (pstring) {
		char *curr_string;

		curr_string = strsep(&pstring, " ,");

		if (*curr_string == '\0')
			continue;

		ret = kstrtouint(curr_string, 0, &value);

		if (!value)
			break;

		if (ret < 0) {
			dev_err(dev, "Error: Invalid argument. Could not convert.\n");
			goto out;
		}
		if (signal_length == 2) {
			data->carrier_freq = value;
			data->signal[2] = (value >> 16) & 0xFF;
			data->signal[3] = (value >> 8) & 0xFF;
			data->signal[4] = value & 0xFF;
			signal_length += 3;
		} else {
			ir_sum += value;
			data->signal[signal_length] = (value >> 8) & 0xFF;
			data->signal[signal_length + 1] = value & 0xFF;
			signal_length += 2;
			number_freq++;
		}
	}
	data->signal_length = signal_length;

	ret = mc96fr116c_send_signal(data);
 out:
	kfree(string);
	mutex_unlock(&data->signal_mutex);

	if (ret < 0)
		return ret;

	return size;
}

static DEVICE_ATTR(ir_send, 0220, NULL, mc96fr1196c_ir_send_store);

static ssize_t mc96fr116c_ir_send_result_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", !!data->last_send_ok);
}
static DEVICE_ATTR(ir_send_result, 0444, mc96fr116c_ir_send_result_show, NULL);

static ssize_t device_id_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct mc96fr116c_data *data = dev_get_drvdata(dev);

	mc96fr116c_read_device_id(data);

	return scnprintf(buf, 5, "%04X\n", data->dev_id);
}
static DEVICE_ATTR(device_id, 0444, device_id_show, NULL);

static int irda_mc96_parse_dt(struct device *dev, struct mc96fr116c_platform_data *pdata)
{

        struct device_node *np = dev->of_node;

        pdata->irda_irq_gpio = of_get_named_gpio_flags(np, "mc96fr116c,irda_irq_gpio",
                               0, &pdata->irq_gpio_flags);
        pdata->irda_led_en = of_get_named_gpio_flags(np, "mc96fr116c,irda_led_en",
                               0, &pdata->led_en_flags);
        pdata->irda_wake_en = of_get_named_gpio_flags(np, "mc96fr116c,irda_wake",
				0, &pdata->wake_en_flags);
	pdata->irda_rst_gpio = of_get_named_gpio_flags(np, "mc96fr116c,irda_rst",
                               0, &pdata->rst_gpio_flags);
	pdata->irda_scl_gpio = of_get_named_gpio_flags(np, "mc96fr116c,scl-gpio",
                               0, &pdata->irda_scl_flags);
	pdata->irda_sda_gpio = of_get_named_gpio_flags(np, "mc96fr116c,sda-gpio",
                               0, &pdata->irda_sda_flags);
        pr_info("%s: irq-gpio:%u led_en:%u wake_up:%u rst:%u irda-scl:%u irda-sda:%u \n", __func__,
			 pdata->irda_irq_gpio,pdata->irda_led_en,pdata->irda_wake_en,pdata->irda_rst_gpio,pdata->irda_scl_gpio,pdata->irda_sda_gpio);
	
        return 0;
}


static int mc96fr116c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mc96fr116c_data *data;
	struct device *mc96fr116c_dev;
	struct mc96fr116c_platform_data *pdata;
	int ret;

	printk(KERN_INFO "%s start!\n", __func__);
	
	pdata = devm_kzalloc(&client->dev,
				sizeof(struct mc96fr116c_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
	}
	ret = irda_mc96_parse_dt(&client->dev, pdata);
	if (ret < 0)
	{
		dev_err(&client->dev,"Parse failed \n");
		return ret;
	}

	pdata->ir_reset = irda_hwreset;
	pdata->ir_wake_en = irda_wake_en;
	pdata->ir_read_ready = irda_read_ready;

	if(gpio_request(pdata->irda_wake_en, "IRDA Wakeup EN Pin"))
		pr_err("%s IRDA Wakeup EN GPIO Request failed\n",__func__);
	gpio_direction_output(pdata->irda_wake_en, 0);
	
	if(gpio_request(pdata->irda_led_en, "irda_led_en"))
		pr_err("%s IRDA LED EN	GPIO Request failed\n",__func__);
	gpio_direction_output(pdata->irda_led_en, 0);

	if(gpio_request(pdata->irda_rst_gpio, "irda_rst_gpio"))
		pr_err("%s IRDA RST  GPIO Request failed\n",__func__);
	gpio_direction_output(pdata->irda_rst_gpio, 1);
	
	if(gpio_request(pdata->irda_irq_gpio, "ira_irq"))
		pr_err("%s IRDA IRQ GPIO Request failed\n", __func__);
	gpio_direction_input(pdata->irda_irq_gpio);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	data = kzalloc(sizeof(struct mc96fr116c_data), GFP_KERNEL);
	if (NULL == data) {
		dev_err(&client->dev, "Failed to data allocate %s\n", __func__);
		ret = -ENOMEM;
		goto err_free_mem;
	}

	data->client = client;
	data->pdata = pdata;
	#if 0
	data->pdata = *((struct mc96fr116c_platform_data *)
			client->dev.platform_data);	
	#endif

#if 0
	if (!data->pdata->ir_wake_en || !data->pdata->ir_read_ready) {
		dev_err(&client->dev, "%s: The platform data for mc96fr116c has not enough functions\n",
				__func__);
		ret = -ENODEV;
		goto err_free_mem;
	}
#endif

	mutex_init(&data->signal_mutex);
	data->signal_length = 0;
	data->mode = MC96FR116C_UNKNOWN_MODE;

	i2c_set_clientdata(client, data);

	gpio_set_value(data->pdata->irda_rst_gpio, 0);
	msleep(20);
	gpio_direction_output(data->pdata->irda_wake_en, 0); /* Bootloader mode */
	gpio_set_value(data->pdata->irda_rst_gpio, 1);
	msleep(53);

	ret = mc96fr116c_check_need_fw_update(data);
	if (ret) {
		pr_info("%s: Need to update the firmware!", __func__);
		ret = mc96fr116c_update_fw(data);
	}
	if (ret < 0)
		goto err_remove_clientdata;

	if (data->mode == MC96FR116C_BOOTLOADER_MODE) {
		ret = mc96fr116c_reset_sw(data, MC96FR116C_RESET_USER_IR_MODE);

		if (ret < 0) {
			ret = -ENODEV;
			goto err_remove_clientdata;
		}
	}

	mc96fr116c_read_device_id(data);

	if (data->dev_id !=
			(INITIAL_USER_IR_MODE_DATA[4] << BITS_PER_BYTE |
			INITIAL_USER_IR_MODE_DATA[5])) {
		dev_err(&client->dev, "%s: data->dev_id is %04X. It is not matched with %04X.\n",
				__func__, data->dev_id,
				(INITIAL_USER_IR_MODE_DATA[4] << BITS_PER_BYTE |
				INITIAL_USER_IR_MODE_DATA[5]));
		ret = -ENODEV;
		goto err_remove_clientdata;
	}

	pr_info("MC96FR116C IR Blaster, Abov semicondoctor Co., Ltd.\n");

	mc96fr116c_dev = device_create(sec_class, NULL, 0, data, "sec_ir");

	if (IS_ERR(mc96fr116c_dev))
		dev_err(&client->dev, "Failed to create mc96fr116c_dev device\n");

	if (device_create_file(mc96fr116c_dev, &dev_attr_ir_send) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
				dev_attr_ir_send.attr.name);

	if (device_create_file(mc96fr116c_dev, &dev_attr_ir_send_result) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
				dev_attr_ir_send.attr.name);

	if (device_create_file(mc96fr116c_dev, &dev_attr_device_id) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
				dev_attr_device_id.attr.name);

	return 0;

err_remove_clientdata:
	i2c_set_clientdata(client, NULL);
	mutex_destroy(&data->signal_mutex);
err_free_mem:
	kfree(data);
	return ret;
}

#if defined(CONFIG_PM)
static int mc96fr116c_suspend(struct device *dev)
{
	mfp_cfg_t irda_rdy_pull_low = IRDA_RDY_PULL_LOW;
	
	pr_info("mc96fr116c_suspend\n");
	mfp_config(&irda_rdy_pull_low, 1);

	return 0;
}

static int mc96fr116c_resume(struct device *dev)
{
	mfp_cfg_t irda_rdy_pull_high = IRDA_RDY_PULL_HIGH;
	
	pr_info("mc96fr116c_resume\n");
	mfp_config(&irda_rdy_pull_high, 1);
	
	return 0;
}

static const struct dev_pm_ops mc96fr116c_pm_ops = {
	.suspend	= mc96fr116c_suspend,
	.resume		= mc96fr116c_resume,
};
#endif

static int mc96fr116c_remove(struct i2c_client *client)
{
	struct mc96fr116c_data *data = i2c_get_clientdata(client);

	mutex_destroy(&data->signal_mutex);
	i2c_set_clientdata(client, NULL);
	kfree(data);
	return 0;
}

static const struct i2c_device_id mc96fr116c_id[] = {
	{ "mc96fr116c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mc96fr116c_id);

static struct of_device_id mc96_i2c_match_table[] = {
	{ .compatible = "mc96fr116c,i2c",},
	{},
};
MODULE_DEVICE_TABLE(of, mc96_i2c_match_table);


static struct i2c_driver mc96_i2c_driver = {
	.driver = {
		.name = "mc96fr116c",
#if defined(CONFIG_PM)
		.pm	= &mc96fr116c_pm_ops,
		.of_match_table = mc96_i2c_match_table,
#endif
	},
	.probe = mc96fr116c_probe,
	.remove = mc96fr116c_remove,

	.id_table = mc96fr116c_id,
};

static int __init mc96fr116c_init(void)
{
	return i2c_add_driver(&mc96_i2c_driver);
}
module_init(mc96fr116c_init);

static void __exit mc96fr116c_exit(void)
{
	i2c_del_driver(&mc96_i2c_driver);
}
module_exit(mc96fr116c_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MC96FR116C IR remote controller for Samsung mobile");
