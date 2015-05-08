/*
 *  Copyright (C) 2014 Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/mms134s.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/regulator/consumer.h>
//#include <linux/platform_data/mv_usb.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif
#include <linux/power_supply.h>

extern void (*tsp_charger_status_cb)(int);

static struct mms_info *m_info;

#ifdef CONFIG_OF
static struct regulator *mms_vdd_regulator;

static int mms_setup_power(struct mms_info *info, bool onoff)
{
	struct i2c_client *client = info->client;
	int min_uv, max_uv;
	int ret = 0;

	if (onoff) {
		if (!mms_vdd_regulator) {
			mms_vdd_regulator = regulator_get(&client->dev, "v_tsp_3v0");

			if (IS_ERR(mms_vdd_regulator)) {
				ret = PTR_ERR(mms_vdd_regulator);
				dev_err(&client->dev,
							"Failed to get mms_vdd_regulator(%d)\n", ret);
				return ret;
			}

			min_uv = max_uv = info->pdata->vdd_regulator_volt;

			ret = regulator_set_voltage(mms_vdd_regulator, min_uv, max_uv);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to set mms_vdd_regulator to \
					%d, %d uV(%d)\n", min_uv, max_uv, ret);
				regulator_put(mms_vdd_regulator);
				return ret;
			}

			dev_info(&client->dev, "Set mms_vdd_regulator to %d, %d uV\n",
				min_uv, max_uv);
		}
	}
	else {
		if (mms_vdd_regulator) {
			regulator_put(mms_vdd_regulator);
			dev_info(&client->dev, "Put mms_vdd_regulator\n");
		}
	}

	return ret;
}

static int mms_onoff_power(struct mms_info *info, bool onoff)
{
	struct i2c_client *client = info->client;
	int ret = 0;

	if (!mms_vdd_regulator) {
		dev_err(&client->dev, "%s : no regulator\n", __func__);
		ret = -EPERM;
		goto out;
	}

	if (onoff)
		ret = regulator_enable(mms_vdd_regulator);
	else
		ret = regulator_disable(mms_vdd_regulator);

	if (ret)
		dev_err(&client->dev, "%s : Failed to %s regulator\n", __func__,
					(onoff) ? "enable" : "disable");
	else
		dev_info(&client->dev, "%s : %s\n", __func__, (onoff) ? "on" : "off");

out:
	return ret;
}
#endif

static void mms_charger_status_cb(int status)
{
	int mode = (status == POWER_SUPPLY_TYPE_BATTERY);

	if (mode)
		i2c_smbus_write_byte_data(m_info->client, MMS_OPERATION_MODE, 0x06);
	else
		i2c_smbus_write_byte_data(m_info->client, MMS_OPERATION_MODE, 0x05);

	dev_info(&m_info->client->dev, "%s mode", mode ? "Battery" : "TA");
}

/* mms_enable - wake-up func (VDD on)  */
static void mms_enable(struct mms_info *info)
{
	if (info->enabled) {
		dev_err(&info->client->dev, "%s : Already enable\n", __func__);
		return;
	}

	/* use vdd control instead */
	mms_onoff_power(info, true);
	msleep(50);

	info->enabled = true;
	enable_irq(info->irq);
}

/* mms_disable - sleep func (VDD off) */
static void mms_disable(struct mms_info *info)
{
	if (!info->enabled) {
		dev_err(&info->client->dev, "%s : Already disable\n", __func__);
		return;
	}

	i2c_smbus_write_byte_data(info->client, MMS_POWER_CONTROL, 1);

	disable_irq(info->irq);

	msleep(50);
	mms_onoff_power(info, false);
	msleep(50);

	info->enabled = false;
}

/* mms_reboot - IC reset */
static void mms_reboot(struct mms_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_smbus_write_byte_data(info->client, MMS_POWER_CONTROL, 1);
	i2c_lock_adapter(adapter);

	msleep(50);
	mms_onoff_power(info, false);
	msleep(150);

	mms_onoff_power(info, true);
	msleep(50);

	i2c_unlock_adapter(adapter);
}

/*
 * mms_clear_input_data - all finger point release
 */
static void mms_clear_input_data(struct mms_info *info)
{
	int i;

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		input_mt_slot(info->input_dev, i);

		if (!info->finger_state[i])
			continue;

		info->finger_state[i] = false;
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
		dev_info(&info->client->dev, "[R][%d] : forced clear\n", i);
	}

	input_sync(info->input_dev);

	return;
}

/* mms_report_input_data - The position of a touch send to platfrom  */
static void mms_report_input_data(struct mms_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	static int esd_cnt = 0;
	int x, y, id, i;
	int major;
	int pressure;
	int key_code;
	int key_state;
	u8 *tmp;

	if (buf[0] == MMS_TEST_MODE) {
		dev_info(&client->dev, "Universal cmd is finished(%d)\n", buf[1]);
		goto out;
	} else if (buf[0] == MMS_NOTIFY_EVENT) {
		dev_info(&client->dev, "TSP mode changed (%d)\n", buf[1]);
		goto out;
	} else if (buf[0] == MMS_ERROR_EVENT) {
		dev_info(&client->dev, "Error detected, restarting TSP\n");
		mms_clear_input_data(info);
		mms_reboot(info);
		esd_cnt++;

		if (esd_cnt >= ESD_DETECT_COUNT) {
			i2c_smbus_write_byte_data(info->client, MMS_MODE_CONTROL, 0x04);
			esd_cnt =0;
		}

		goto out;
	}

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;
		esd_cnt = 0;

		if (tmp[0] & MMS_TOUCH_KEY_EVENT) {
			switch (tmp[0] & 0xf) {
			case 1:
				key_code = KEY_RECENT;
				break;
			case 2:
				key_code = KEY_BACK;
				break;
			default:
				dev_err(&client->dev, "Unknown key type\n");
				continue;
			}

			key_state = (tmp[0] & 0x80) ? 1 : 0;
			input_report_key(info->input_dev, key_code, key_state);
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
			dev_info(&client->dev, "key[%3d] is %s\n", key_code,
				key_state ? "pressed" : "releaseed");
#else
			dev_info(&client->dev,
				"key is %s\n", key_state ? "pressed" : "releaseed");
#endif

		} else {
			id = (tmp[0] & 0xf) - 1;

			x = tmp[2] | ((tmp[1] & 0xf) << 8);
			y = tmp[3] | (((tmp[1] >> 4 ) & 0xf) << 8);
			major = tmp[4];
			pressure = tmp[5];

			input_mt_slot(info->input_dev, id);

			if (!(tmp[0] & 0x80)) {
				if (info->finger_state[id]) {
					info->finger_state[id] = false;
					input_mt_report_slot_state(info->input_dev,
						MT_TOOL_FINGER, false);

#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
					dev_info(&client->dev,
								"[R][%d]: X[%d], Y[%d]\n", id, x, y);
#else
					dev_info(&client->dev, "[R][%d]\n", id);
#endif
				}
				continue;
			}

			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, major);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);

			if (!info->finger_state[id]) {
				info->finger_state[id] = true;
#ifndef CONFIG_SAMSUNG_PRODUCT_SHIP
				dev_info(&client->dev,
					"[P][%d]: X[%d], Y[%d], W[%d], Z[%d]\n",
					id, x, y, major, pressure);
#else
				dev_info(&client->dev, "[P][%d]\n", id);
#endif
			}
		}
	}

	input_sync(info->input_dev);

out:
	return;
}

/* mms_interrupt - interrupt thread */
static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_info *info = (struct mms_info *)dev_id;
	struct i2c_client *client = info->client;
	u8 reg = MMS_EVENT_PKT;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
		},
	};
	int ret, sz;
	int buf_len = (MAX_FINGER_NUM + info->key_num) * FINGER_EVENT_SZ;
	static u8 *buf = NULL;

	if (!buf) {
		buf = kzalloc(buf_len, GFP_KERNEL);
		if (!buf) {
			dev_err(&client->dev,
				"%s : Failed to allocate memory\n", __func__);
			buf = NULL;
			goto out;
		}
	}

	msg[1].buf = buf;

	sz = i2c_smbus_read_byte_data(client, MMS_EVENT_PKT_SZ);
	if (sz < 0 || sz > buf_len) {
		dev_err(&client->dev,
			"%s : Failed to read event packet size(%d)\n", __func__, sz);
		{
			char *buff = kzalloc(sz, GFP_KERNEL);
			msg[1].buf = buff;
			msg[1].len = sz;

			ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
			print_hex_dump(KERN_INFO, "event packet : ", DUMP_PREFIX_NONE,
				16, 1, buff, sz, false);

			kfree(buff);
		}
		goto out;
	}

	msg[1].len = sz;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg))
		dev_err(&client->dev,
			"Failed to read %d bytes of touch data (%d)\n", sz, ret);
	else
		mms_report_input_data(info, sz, buf);

out:
	return IRQ_HANDLED;
}

static int mms_config(struct mms_info *info)
{
	struct i2c_client *client = info->client;
	int ret = 0;

#if MMS_HAS_TOUCH_KEY
	ret = i2c_smbus_read_byte_data(client, MMS_KEY_NUM);
	if (ret <= 0) {
		dev_err(&client->dev, "Failed to read keynode num(%d)\n", ret);
		goto out;
	}

	info->key_num = ret;

	ret = 0;
#endif

	info->finger_state = kzalloc(sizeof(bool) * MAX_FINGER_NUM, GFP_KERNEL);
	if (!info->finger_state) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	info->enabled = true;
	m_info = info;

	tsp_charger_status_cb = mms_charger_status_cb;

out:
	return ret;
}

/*
 * mms_isc_read_status - Check erase state function
 */
static int mms_isc_read_status(struct mms_info *info, u32 val)
{
	struct i2c_client *client = info->client;
	u8 cmd = ISC_CMD_READ_STATUS;
	u32 result = 0;
	int cnt = 100;
	int ret = 0;

	do {
		i2c_smbus_read_i2c_block_data(client, cmd, 4, (u8 *)&result);

		if (result == val)
			break;

		msleep(1);
	} while (--cnt);

	if (!cnt) {
		dev_err(&client->dev,
			"Failed to read status. cnt : %d, val : 0x%x != 0x%x\n",
			cnt, result, val);
	}

	return ret;
}

static int mms_isc_transfer_cmd(struct mms_info *info, int cmd)
{
	struct i2c_client *client = info->client;
	struct isc_packet pkt = { ISC_ADDR, cmd };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(struct isc_packet),
		.buf = (u8 *)&pkt,
	};

	return i2c_transfer(client->adapter, &msg, 1) != 1;
}

/*
 * mms_isc_erase_page - ic page erase(1 kbyte)
 */
static int mms_isc_erase_page(struct mms_info *info, int page)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_PAGE_ERASE | page) ||
		mms_isc_read_status(
			info, ISC_PAGE_ERASE_DONE | ISC_PAGE_ERASE_ENTER | page
		);
}

/*
 * mms_isc_enter - isc enter command
 */
static int mms_isc_enter(struct mms_info *info)
{
	return i2c_smbus_write_byte_data(info->client, MMS_CMD_ENTER_ISC, true);
}

static int mms_isc_exit(struct mms_info *info)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_EXIT);
}

/*
 * mms_get_fw_version - f/w version read
 */
static int mms_get_fw_version(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_BL_VERSION;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = MAX_SECTION_NUM,
		},
	};

	return i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))
				!= ARRAY_SIZE(msg);
}

/*
 * mms_flash_section - f/w section(boot,core,config) download func
 */
static int mms_flash_section(struct mms_info *info, struct mms_fw_img *img,
			const u8 *data)
{
	struct i2c_client *client = info->client;
	struct isc_packet *isc_packet;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = ISC_XFER_LEN,
		},
	};
	int ptr = img->offset;
	int ret;
	int page, i;
	u32 tmp;

	isc_packet = kzalloc(sizeof(struct isc_packet) + ISC_XFER_LEN, GFP_KERNEL);
	isc_packet->cmd = ISC_ADDR;

	msg[0].buf = (u8 *)isc_packet;
	msg[1].buf = kzalloc(ISC_XFER_LEN, GFP_KERNEL);

	for (page = img->start_page; page <= img->end_page; page++) {
		mms_isc_erase_page(info, page);

		for (i = 0; i < ISC_BLOCK_NUM; i++, ptr += ISC_XFER_LEN) {
			/* flash firmware */
			tmp = page * 256 + i * (ISC_XFER_LEN / 4);
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet) + ISC_XFER_LEN;

			memcpy(isc_packet->data, data + ptr, ISC_XFER_LEN);

			if (i2c_transfer(client->adapter, msg, 1) != 1)
				goto i2c_err;

			/* verify firmware */
			tmp |= ISC_CMD_READ;
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet);

			if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg))
					!= ARRAY_SIZE(msg))
				goto i2c_err;

			if (memcmp(isc_packet->data, msg[1].buf, ISC_XFER_LEN)) {
#if FLASH_VERBOSE_DEBUG
				print_hex_dump(KERN_ERR, "mms134s fw wr : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						isc_packet->data, ISC_XFER_LEN, false);

				print_hex_dump(KERN_ERR, "mms134s fw rd : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						msg[1].buf, ISC_XFER_LEN, false);
#endif
				dev_err(&client->dev, "Failed to flash verify\n");
				ret = -1;
				goto out;
			}
		}
	}

	dev_info(&client->dev, "Section [%d] update succeeded\n", img->type);

	ret = 0;
	goto out;

i2c_err:
	dev_err(&client->dev, "%s : i2c failed\n", __func__);
	ret = -1;

out:
	kfree(isc_packet);
	kfree(msg[1].buf);

	return ret;
}

/*
 * mms_flash_fw -flash_section of parent function
 * this function is IC version binary version compare and Download
 */
static int mms_flash_fw(const struct firmware *fw, struct mms_info *info,
						bool force)
{
	int ret = 0;
	int i;
	struct mms_bin_hdr *fw_hdr = (struct mms_bin_hdr *)fw->data;
	struct mms_fw_img **img;
	struct i2c_client *client = info->client;
	u8 ver[MAX_SECTION_NUM];
	u8 target[MAX_SECTION_NUM];
	int offset = sizeof(struct mms_bin_hdr);
	int retires = 3;
	bool update_flag = false;
	bool isc_flag = true;

	img = kzalloc(
			sizeof(struct mms_fw_img *) * fw_hdr->section_num, GFP_KERNEL);

	while (retires--) {
		if (!mms_get_fw_version(client, ver))
			break;
		else
			mms_reboot(info);
	}

	if (retires < 0) {
		dev_err(&client->dev, "Failed to obtain ver. info\n");
		isc_flag = false;
		memset(ver, 0xff, sizeof(ver));
	} else {
		print_hex_dump(KERN_INFO, "mms134s fw ver : ", DUMP_PREFIX_NONE, 16, 1,
				ver, MAX_SECTION_NUM, false);
	}

	for (i = 0; i < fw_hdr->section_num; i++,
				offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		target[i] = img[i]->version;

		if (force || (ver[img[i]->type] < target[i])) {
			if(isc_flag){
				mms_isc_enter(info);
				isc_flag = false;
			}

			update_flag = true;
			dev_info(&client->dev,
				"Section [%d] is need to be updated. ver : 0x%x, bin : 0x%x\n",
				img[i]->type, ver[img[i]->type], target[i]);

			if (ret = mms_flash_section(
						info, img[i], fw->data + fw_hdr->binary_offset)) {
				mms_reboot(info);
				goto out;
			}
		} else {
			dev_info(&client->dev, "Section [%d] is up to date\n", i);
		}
	}

	if (update_flag) {
		mms_isc_exit(info);
		msleep(5);
		mms_reboot(info);
	}

	if (mms_get_fw_version(client, ver)) {
		dev_err(&client->dev, "Failed to obtain version after flash\n");
		ret = -1;
		goto out;
	} else {
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (ver[img[i]->type] < target[i]) {
				dev_info(&client->dev,
					"Version mismatch after flash."
					" [%d] IC Ver 0x%x, FW Ver 0x%x\n",
					i, ver[img[i]->type], target[i]);

				ret = -1;
				goto out;
			}
		}
	}

out:
	kfree(img);

	return ret;
}

/*
 * mms_fw_update_controller - firmware check & update
 */
static int mms_fw_update_controller(struct mms_info *info)
{
	struct i2c_client *client = info->client;
	const struct firmware *fw;
	int retires = 3;
	int ret = 0;

	ret = request_firmware(&fw, info->pdata->inkernel_fw_name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "Failed to requesting firmware\n");
		goto out;
	}

	do {
		ret = mms_flash_fw(fw, info, false);
	} while (ret && --retires);

	if (!retires) {
		dev_err(&client->dev, "Failed to flash fw after retires\n");
		goto out;
	}

	info->fw = fw;

out:
	return ret;
}

#ifdef CONFIG_OF
static struct of_device_id mms_dt_ids[] = {
	{ .compatible = "melfas,mms134s", },
	{}
};
MODULE_DEVICE_TABLE(of, mms_dt_ids);

static int mms_probe_dt(struct device_node *np, struct device *dev,
			struct mms_platform_data *pdata)
{
	struct of_device_id *match;
	int ret = 0;

	if (!np) {
		dev_err(dev, "Device node not found\n");
		return -EINVAL;
	}

	match = of_match_device(mms_dt_ids, dev);
	if (!match) {
		dev_err(dev, "Compatible mismatch\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "melfas,max_x", &pdata->max_x);
	if (ret) {
		dev_err(dev, "Failed to get property max_x from node(%d)\n", ret);
		goto out;
	}

	ret = of_property_read_u32(np, "melfas,max_y", &pdata->max_y);
	if (ret) {
		dev_err(dev, "Failed to get property max_y from node(%d)\n", ret);
		goto out;
	}

	ret = of_property_read_u32(np, "melfas,irqflags", &pdata->irqflags);
	if (ret) {
		dev_err(dev, "Failed to get property irqflags from node(%d)\n", ret);
		goto out;
	}

	pdata->gpio_int = of_get_named_gpio(np, "melfas,gpio_int", 0);
	if (pdata->gpio_int < 0) {
		dev_err(dev, "Failed to get property gpio_int(%d)\n", pdata->gpio_int);
		ret = pdata->gpio_int;
		goto out;
	}

	ret = of_property_read_u32(np,
			"melfas,vdd_regulator_volt", &pdata->vdd_regulator_volt);
	if (ret) {
		dev_err(dev,
			"Failed to get property vdd_regulator_volt from node(%d)\n", ret);
		goto out;
	}

	ret = of_property_read_string(np, "melfas,inkernel_fw_name",
				&pdata->inkernel_fw_name);
	if (ret) {
		dev_err(dev,
			"Failed to get property inkernel_fw_name from node(%d)\n", ret);
		goto out;
	}

out:
	return ret;
}
#endif

#ifdef CONFIG_MACH_PXA_SAMSUNG
#include "mms134s_sec.c"
#endif

static int mms_probe(struct i2c_client *client, struct i2c_device_id *id)
{
	struct mms_platform_data *pdata = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device_node *np = client->dev.of_node;
	struct mms_info *info;
	struct input_dev *input_dev;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}

#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(struct mms_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ret = mms_probe_dt(np, &client->dev, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to get platform data from node\n");
		goto err_probe_dt;
	}
#else
	if (!pdata) {
		dev_err(&client->dev, "Platform data not found\n");
		return -EINVAL;
	}
#endif

	info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}

	info->client = client;
	info->pdata = pdata;

	ret = mms_setup_power(info, true);
	if (ret < 0)
		goto err_power;

	ret = mms_onoff_power(info, true);
	if (ret)
		goto err_power;

	msleep(50);

	ret = mms_fw_update_controller(info);
	if (ret) {
		dev_err(&client->dev, "Failed to firmware update (%d)\n", ret);
		goto err_power;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_power;
	}

	info->input_dev = input_dev;

	snprintf(info->phys, sizeof(info->phys), "%s/input0",
				dev_name(&client->dev));

	input_dev->name = "sec_touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if MMS_HAS_TOUCH_KEY
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_RECENT, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
#endif

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
							info->pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
							info->pdata->max_y, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "Failed to register input dev(%d)\n", ret);
		goto err_input_register;
	}

	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);

	info->irq = gpio_to_irq(info->pdata->gpio_int);
	ret = request_threaded_irq(info->irq, NULL, mms_interrupt,
				info->pdata->irqflags, client->dev.driver->name, info);
	if (ret) {
		dev_err(&client->dev, "Failed to register irq(%d)\n", ret);
		goto err_request_irq;
	}

	ret = mms_config(info);
	if (ret) {
		dev_err(&client->dev, "Failed mms134 configuration\n");
		goto err_mms_config;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&client->dev);
#endif

#ifdef CONFIG_MACH_PXA_SAMSUNG
	ret = init_sec(info);
	if (ret)
		dev_err(&client->dev, "Failed to initialize SEC_FACTORY\n");
#endif

	dev_info(&client->dev, "mms134s initialized\n");

	return 0;

err_mms_config:
	free_irq(info->irq, info);
err_request_irq:
	input_unregister_device(input_dev);
err_input_register:
	input_free_device(input_dev);
err_power:
	kfree(info);
err_info_alloc:
#ifdef CONFIG_OF
err_probe_dt:
	kfree(pdata);
#endif
	dev_info(&client->dev, "Failed to initialization\n");

	return ret;
}

static int mms_remove(struct i2c_client *client)
{
	struct mms_info *info = i2c_get_clientdata(client);

	free_irq(info->irq, info);
	kfree(info->finger_state);
	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
#ifdef CONFIG_MACH_PXA_SAMSUNG
	destroy_sec(info);
#endif
#ifdef CONIFG_OF
	kfree(info->pdata);
#endif
	kfree(info);

	return 0;
}

#ifdef CONFIG_PM
static int mms_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		mms_clear_input_data(info);
		mms_disable(info);
	}

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

static int mms_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users)
		mms_enable(info);

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}
#endif

static struct i2c_device_id mms_id[] = {
	{MMS_DEV_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

#ifdef CONFIG_PM
static struct dev_pm_ops mms_pm_ops = {
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(mms_suspend, mms_resume, NULL)
#else
	SET_SYSTEM_SLEEP_PM_OPS(mms_suspend, mms_resume);
#endif
};
#endif

static struct i2c_driver mms_driver = {
	.id_table	= mms_id,
	.probe		= mms_probe,
	.remove		= mms_remove,
	.driver		= {
			.owner	= THIS_MODULE,
			.name	= MMS_DEV_NAME,
#ifdef CONFIG_OF
			.of_match_table	= of_match_ptr(mms_dt_ids),
#endif
#ifdef CONFIG_PM
			.pm	= &mms_pm_ops,
#endif
	},
};

module_i2c_driver(mms_driver);

MODULE_DESCRIPTION("Touchscreen driver for MELFAS MMS134S");
MODULE_LICENSE("GPL");