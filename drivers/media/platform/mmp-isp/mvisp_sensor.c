/* Marvell ISP Sensor Driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * Based on mt9v011 -Micron 1/4-Inch VGA Digital Image Sensor
 *
 * Copyright (c) 2009 Mauro Carvalho Chehab (mchehab@redhat.com)
 * This code is placed under the terms of the GNU General Public License v2
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>

#include <asm/div64.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <media/mvisp_sensor.h>

MODULE_DESCRIPTION("Marvell ISP sensor driver");
MODULE_AUTHOR("Henry Zhao <xzhao10@marvell.com>");
MODULE_LICENSE("GPL");

#define MAX_DETECT_NUM			3
#define MAX_SENSOR_PADS_NUM		1

#define SENSOR_PAD_SOURCE		0

struct sensor_power {
	struct regulator *af_2v8;
	struct regulator *avdd_2v8;
	struct regulator *dovdd_1v8;
	struct regulator *dvdd_1v2;
	int pwdn;
	int reset;
	int pwdn_validvalue;
	int reset_validvalue;
};
struct sensor_id {
	u32 pid_addr[2];
	u32 pid_val[2];
};
struct sensor_core {
	struct v4l2_ctrl_handler	hdl;
	struct v4l2_subdev		sd;
	struct device_node		*np;
	struct sensor_power		power;
	struct sensor_id		id;
	struct mutex			sensor_mutex;
	struct media_pad		pads[MAX_SENSOR_PADS_NUM];
	u32				addr_flag;
	u8				openflag;
};
enum sensor_register_access_e {
	SENSOR_REG_INVALID = 0,
	SENSOR_REG_READ,
	SENSOR_REG_WRITE,
};
struct reg_tab_wb {
		u16 reg;
		u8 val;
} __packed;

struct reg_tab_bb {
		u8 reg;
		u8 val;
} __packed;
/* supported controls */
static struct v4l2_queryctrl sensor_qctrl[] = {
	{
		.id = V4L2_CID_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Gain",
		.minimum = 0,
		.maximum = (1 << 10) - 1,
		.step = 1,
		.default_value = 0x0020,
		.flags = 0,
	}, {
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Red Balance",
		.minimum = -1 << 9,
		.maximum = (1 << 9) - 1,
		.step = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Blue Balance",
		.minimum = -1 << 9,
		.maximum = (1 << 9) - 1,
		.step = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id      = V4L2_CID_HFLIP,
		.type    = V4L2_CTRL_TYPE_BOOLEAN,
		.name    = "Mirror",
		.minimum = 0,
		.maximum = 1,
		.step    = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id      = V4L2_CID_VFLIP,
		.type    = V4L2_CTRL_TYPE_BOOLEAN,
		.name    = "Vflip",
		.minimum = 0,
		.maximum = 1,
		.step    = 1,
		.default_value = 0,
		.flags = 0,
	}, {
	}
};
static inline struct sensor_core *to_sensor_core(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_core, sd);
}
static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct sensor_core, hdl)->sd;
}

static int sensor_read_bb(struct v4l2_subdev *sd, u8 reg, unsigned char *value)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= (u8 *)&reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		goto out;

out:
	return (ret < 0) ? ret : 0;
}

static int sensor_read_wb(struct v4l2_subdev *sd, u16 reg, unsigned char *value)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= (u8 *)&reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= value,
		},
	};

	reg = swab16(reg);
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		goto out;

out:
	return (ret < 0) ? ret : 0;
}

static int sensor_write_bb(struct v4l2_subdev *sd, u8 reg, unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	struct reg_tab_bb buf;
	int ret = 0;

	buf.reg = reg;
	buf.val = value;

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 2;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int sensor_write_wb(struct v4l2_subdev *sd, u16 reg, unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	struct reg_tab_wb buf;
	int ret = 0;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = value;

	msg.addr	= client->addr;
	msg.flags	= 0;
	msg.len		= 3;
	msg.buf		= (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static int sensor_write(struct v4l2_subdev *sd,
				u16 reg, unsigned char value)
{
	struct sensor_core *core = to_sensor_core(sd);
	int ret = 0;

	if (core->addr_flag == REG_TYPE_16) {
		ret = sensor_write_wb(sd, (u16)reg, value);
		if (ret < 0)
			return ret;
	} else if (core->addr_flag == REG_TYPE_8) {
		ret = sensor_write_bb(sd, (u8)reg, value);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int sensor_read(struct v4l2_subdev *sd, unsigned long reg,
		unsigned char *value)
{
	struct sensor_core *core = to_sensor_core(sd);
	int ret = 0;

	if (core->addr_flag == REG_TYPE_16) {
		ret = sensor_read_wb(sd, (u16)reg, value);
		if (ret < 0)
			return ret;
	} else if (core->addr_flag == REG_TYPE_8) {
		ret = sensor_read_bb(sd, (u8)reg, value);
		if (ret < 0)
			return ret;
	}

	return ret;
}
static long sensor_register_access(struct v4l2_subdev *sd,
			struct v4l2_sensor_register_access *param,
			enum sensor_register_access_e access_type)
{
	int ret = -EINVAL;

	switch (access_type) {
	case SENSOR_REG_READ:
		ret = sensor_read(sd, param->reg,
			(unsigned char *)&param->value);
		break;
	case SENSOR_REG_WRITE:
		ret = sensor_write(sd, param->reg,
			(unsigned char) param->value);
		break;
	default:
		break;
	}

	return ret;
}
static int sensor_detect(struct sensor_id *pid,
				struct v4l2_subdev *sd)
{
	int j = 0, ret = 0;
	unsigned char v = 0;
	for (j = 0; j < 2; j++) {
		ret = sensor_read(sd, pid->pid_addr[j], &v);
		if ((ret < 0) || (v != pid->pid_val[j]))
			goto error;
	}
	return 0;
error:
	return -1;
}
static int sensor_set_csi_dphy(struct v4l2_subdev *sd,
		struct v4l2_sensor_csi_dphy *u_dphy)
{
	if (!u_dphy || !sd)
		return -EINVAL;

	mvisp_set_fclk_dphy(sd, u_dphy);

	return 0;
}
static long sensor_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int ret = -EINVAL;
	struct sensor_core *core = to_sensor_core(sd);

	mutex_lock(&core->sensor_mutex);
	switch (cmd) {
	case VIDIOC_PRIVATE_SENSER_REGISTER_GET:
		ret = sensor_register_access(sd,
			(struct v4l2_sensor_register_access *)arg,
			SENSOR_REG_READ);
		break;
	case VIDIOC_PRIVATE_SENSER_REGISTER_SET:
		ret = sensor_register_access(sd,
			(struct v4l2_sensor_register_access *)arg,
			SENSOR_REG_WRITE);
		break;
	case VIDIOC_PRIVATE_SENSER_SET_CSI_DPHY:
		ret = sensor_set_csi_dphy(sd,
			(struct v4l2_sensor_csi_dphy *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	mutex_unlock(&core->sensor_mutex);
	return ret;
}
static int sensor_power(struct sensor_power *power, int on)
{
	int ret = 0;
	if (on) {
		if (gpio_request(power->pwdn, "cam power gpio")) {
			pr_err("gpio %d request failed\n", power->pwdn);
			return -1;
		}
		if (gpio_request(power->reset, "cam reset gpio")) {
			pr_err("gpio %d request failed\n", power->reset);
			return -1;
		}
		if (power->avdd_2v8) {
			regulator_set_voltage(power->avdd_2v8,
						2800000, 2800000);
			ret = regulator_enable(power->avdd_2v8);
			if (ret < 0)
				return ret;
		}
		gpio_direction_output(power->pwdn, !power->pwdn_validvalue);
		if (power->dovdd_1v8) {
			regulator_set_voltage(power->dovdd_1v8,
							1800000, 1800000);
			ret = regulator_enable(power->dovdd_1v8);
			if (ret < 0)
				goto dovdd_err;
		}
		if (power->dvdd_1v2) {
			regulator_set_voltage(power->dvdd_1v2,
							1200000, 1200000);
			ret = regulator_enable(power->dvdd_1v2);
			if (ret < 0)
				goto dvdd_err;
		}
		if (power->af_2v8) {
			regulator_set_voltage(power->af_2v8,
							2800000, 2800000);
			ret = regulator_enable(power->af_2v8);
			if (ret < 0)
				goto af_err;
		}
		gpio_direction_output(power->reset, power->reset_validvalue);
		usleep_range(100, 120);
		mvisp_set_sensor_mclk(1);
		gpio_direction_output(power->reset, !power->reset_validvalue);
	} else {
		gpio_direction_output(power->reset, power->reset_validvalue);
		if (power->dvdd_1v2)
			regulator_disable(power->dvdd_1v2);
		if (power->avdd_2v8)
			regulator_disable(power->avdd_2v8);
		gpio_direction_output(power->pwdn, power->pwdn_validvalue);
		if (power->dovdd_1v8)
			regulator_disable(power->dovdd_1v8);
		if (power->af_2v8)
			regulator_disable(power->af_2v8);
		mvisp_set_sensor_mclk(0);
		gpio_free(power->pwdn);
		gpio_free(power->reset);
	}
	return 0;
af_err:
	if (power->dvdd_1v2)
		regulator_disable(power->dvdd_1v2);
dvdd_err:
	if (power->dovdd_1v8)
		regulator_disable(power->dovdd_1v8);
dovdd_err:
	if (power->af_2v8)
		regulator_disable(power->af_2v8);
	return ret;
}
static int sensor_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i, ret = -EINVAL;
	struct sensor_core *core = to_sensor_core(sd);

	mutex_lock(&core->sensor_mutex);

	for (i = 0; i < ARRAY_SIZE(sensor_qctrl); i++)
		if (qc->id && qc->id == sensor_qctrl[i].id) {
			*qc = sensor_qctrl[i];
			ret = 0;
			break;
		}

	mutex_unlock(&core->sensor_mutex);
	return ret;
}
static int sensor_control_torch(struct v4l2_subdev *sd,
		int on)
{
	int torch_on = 0;
	if (torch_on >= 0) {
		if (gpio_request(torch_on, "SENSOR_SET_POWER_ON")) {
			pr_err("Request GPIO failed,gpio:%d\n",
					torch_on);
			return -EIO;
		}

		gpio_direction_output(torch_on, on);

		gpio_free(torch_on);
	} else
		pr_err("GPIO is invalid\n");

	return 0;
}

static int sensor_control_flash(struct v4l2_subdev *sd,
		int on)
{
	int flash_on = 0;
	if (flash_on >= 0) {
		if (gpio_request(flash_on, "SENSOR_SET_POWER_ON")) {
			pr_err("Request GPIO failed,gpio:%d\n",
					flash_on);
			return -EIO;
		}

		gpio_direction_output(flash_on, on);

		gpio_free(flash_on);
	} else
		pr_err("GPIO is invalid\n");

	return 0;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct sensor_core *core = to_sensor_core(sd);
	u8 i, n;
	int ret = 0;
	static int cur_mode = V4L2_FLASH_LED_MODE_NONE;
	static int cur_active_mode = V4L2_FLASH_LED_MODE_NONE;

	mutex_lock(&core->sensor_mutex);

	n = ARRAY_SIZE(sensor_qctrl);

	for (i = 0; i < n; i++) {
		if (ctrl->id != sensor_qctrl[i].id)
			continue;
		if (ctrl->val < sensor_qctrl[i].minimum ||
		    ctrl->val > sensor_qctrl[i].maximum) {
			ret = -ERANGE;
			goto error;
		}
	}

	switch (ctrl->id) {
	case V4L2_CID_FLASH_LED_MODE:
			cur_mode = ctrl->val;
			break;
	case V4L2_CID_FLASH_STROBE_STOP:
		switch (cur_active_mode) {
		case V4L2_FLASH_LED_MODE_FLASH:
			ret = sensor_control_flash(sd, 0);
			break;
		case V4L2_FLASH_LED_MODE_TORCH:
			ret = sensor_control_torch(sd, 0);
			break;
		default:
			ret = 0;
			pr_err("Warning: You haven't call flash open!\n");
			break;
		}
		cur_active_mode = V4L2_FLASH_LED_MODE_NONE;
		break;
	case V4L2_CID_FLASH_STROBE:
		if (cur_active_mode != V4L2_FLASH_LED_MODE_NONE) {
			ret = -EIO;
			goto error;
		}
		switch (cur_mode) {
		case V4L2_FLASH_LED_MODE_FLASH:
			ret = sensor_control_flash(sd, 1);
			break;
		case V4L2_FLASH_LED_MODE_TORCH:
			ret = sensor_control_torch(sd, 1);
			break;
		default:
			ret = 0;
			pr_err("Warning: Flash ID mismatch!\n");
			break;
		}
		cur_active_mode = cur_mode;
		break;
	default:
		break;
	}

error:
	mutex_unlock(&core->sensor_mutex);

	return ret;
}
static int sensor_subdev_close(struct v4l2_subdev *sd,
		struct v4l2_subdev_fh *fh)
{
	struct sensor_core *core = to_sensor_core(sd);

	mutex_lock(&core->sensor_mutex);
	sensor_power(&core->power, 0);
	core->openflag = 0;
	mutex_unlock(&core->sensor_mutex);
	return 0;
}

static int sensor_subdev_open(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct sensor_core *core = to_sensor_core(sd);
	mutex_lock(&core->sensor_mutex);
	if (core->openflag == 1) {
		mutex_unlock(&core->sensor_mutex);
		return -EBUSY;
	}
	core->openflag = 1;
	sensor_power(&core->power, 1);

	mutex_unlock(&core->sensor_mutex);

	return 0;
}
static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.s_ctrl = sensor_s_ctrl,
};
static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.ioctl = sensor_subdev_ioctl,
	.queryctrl = sensor_queryctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.querymenu = v4l2_subdev_querymenu,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
};
static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct sensor_core *core = to_sensor_core(sd);

	mutex_lock(&core->sensor_mutex);
	dev_warn(sd->v4l2_dev->dev, "sensor_s_stream %d\n", enable);
	mutex_unlock(&core->sensor_mutex);

	return ret;
}

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core  = &sensor_core_ops,
	.video = &sensor_video_ops,
};
/* subdev internal operations */
static const struct v4l2_subdev_internal_ops sensor_v4l2_internal_ops = {
	.open = sensor_subdev_open,
	.close = sensor_subdev_close,
};
/* media operations */
static int sensor_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct sensor_core *core;
	struct v4l2_subdev *sd;
	struct media_pad *pads;
	struct media_entity *me;
	struct device_node *power_np, *pid_np;
	char *chip_name;
	int ret = 0;
	int flag = 0;
	core = devm_kzalloc(&client->dev,
			sizeof(struct sensor_core),
			GFP_KERNEL);
	if (!core)
		return -ENOMEM;
	core->np = (struct device_node *)client->dev.of_node;
	power_np = of_get_next_available_child(core->np, NULL);
	core->power.pwdn = of_get_named_gpio(power_np, "pwdn-gpios", 0);
	if (core->power.pwdn < 0) {
		pr_err("%x: of_get_named_gpio failed\n", __LINE__);
		return -1;
	}
	ret = of_property_read_u32(power_np, "pwdn-validvalue",
				&core->power.pwdn_validvalue);
	if (ret < 0) {
		pr_err("%x: of_get_gpio_value failed\n", __LINE__);
		return -1;
	}
	core->power.reset = of_get_named_gpio(power_np, "reset-gpios", 0);
	if (core->power.reset < 0) {
		pr_err("%x: of_get_named_gpio failed\n", __LINE__);
		return -1;
	}
	ret = of_property_read_u32(power_np, "reset-validvalue",
				&core->power.reset_validvalue);
	if (ret < 0) {
		pr_err("%x: of_get_gpio_value failed\n", __LINE__);
		return ret;
	}
	pid_np =  of_get_next_available_child(core->np, power_np);
	ret = of_property_read_u32_array(pid_np, "pid_addr",
				(u32 *)&core->id.pid_addr, 2);
	if (ret < 0) {
		pr_err("%d: of_get_id_value failed:%d\n", __LINE__, ret);
		return ret;
	}
	ret = of_property_read_u32_array(pid_np, "pid_value",
				(u32 *)&core->id.pid_val, 2);
	if (ret < 0) {
		pr_err("%d: of_get_id_value failed:%d\n", __LINE__, ret);
		return ret;
	}
	core->power.af_2v8 = devm_regulator_get(&client->dev, "af_2v8");
	if (IS_ERR(core->power.af_2v8)) {
		dev_warn(&client->dev, "Failed to get regulator af_2v8\n");
		core->power.af_2v8 = NULL;
	}
	core->power.avdd_2v8 = devm_regulator_get(&client->dev, "avdd_2v8");
	if (IS_ERR(core->power.avdd_2v8)) {
		dev_warn(&client->dev, "Failed to get regulator avdd_2v8\n");
		core->power.avdd_2v8 = NULL;
	}
	core->power.dovdd_1v8 = devm_regulator_get(&client->dev, "dovdd_1v8");
	if (IS_ERR(core->power.dovdd_1v8)) {
		dev_warn(&client->dev, "Failed to get regulator dovdd_1v8\n");
		core->power.dovdd_1v8 = NULL;
	}
	core->power.dvdd_1v2 = devm_regulator_get(&client->dev, "dvdd_1v2");
	if (IS_ERR(core->power.dvdd_1v2)) {
		dev_warn(&client->dev, "Failed to get regulator dvdd_1v2\n");
		core->power.dvdd_1v2 = NULL;
	}
	ret = of_property_read_u32(core->np, "type", (u32 *)&core->addr_flag);
	if (ret < 0)
		core->addr_flag = 1;
	mutex_init(&core->sensor_mutex);
	sd = &core->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);
	sd->internal_ops = &sensor_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pads = core->pads;
	ret = sensor_power(&core->power, 1);
	if (ret < 0)
		goto power_err;
	flag = sensor_detect(&core->id, sd);
	sensor_power(&core->power, 0);
	if (flag < 0) {
		ret = -1;
		goto error;
	}
	v4l2_ctrl_handler_init(&core->hdl, 3);

	v4l2_ctrl_new_std_menu(&core->hdl, &sensor_ctrl_ops,
			V4L2_CID_FLASH_LED_MODE, 2, ~7,
			V4L2_FLASH_LED_MODE_NONE);
	v4l2_ctrl_new_std(&core->hdl, &sensor_ctrl_ops,
			V4L2_CID_FLASH_STROBE, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&core->hdl, &sensor_ctrl_ops,
			V4L2_CID_FLASH_STROBE_STOP, 0, 1, 1, 0);
	sd->ctrl_handler = &core->hdl;

	ret = core->hdl.error;
	if (ret)
		goto sensor_flash_error;
	me = &sd->entity;
	of_property_read_string(core->np, "sensor-name",
					(char const **)&chip_name);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		chip_name, i2c_adapter_id(client->adapter), client->addr);
	pr_info("cam: %s: sensor detected!\n", chip_name);
	pads[SENSOR_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	me->type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(me, MAX_SENSOR_PADS_NUM, pads, 0);
	if (ret < 0)
		goto sensor_media_init_error;
	return 0;
sensor_media_init_error:
	media_entity_cleanup(&sd->entity);
sensor_flash_error:
	v4l2_ctrl_handler_free(&core->hdl);
error:
	if (core->power.avdd_2v8)
		devm_regulator_put(core->power.avdd_2v8);
	if (core->power.dvdd_1v2)
		devm_regulator_put(core->power.dvdd_1v2);
	if (core->power.af_2v8)
		devm_regulator_put(core->power.af_2v8);
	if (core->power.dovdd_1v8)
		devm_regulator_put(core->power.dovdd_1v8);
power_err:
	v4l2_device_unregister_subdev(sd);

	devm_kfree(&client->dev, core);
	core = NULL;
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sensor_core *core = to_sensor_core(sd);
	if (core == NULL)
		return 0;
	if (core->power.af_2v8)
		regulator_put(core->power.af_2v8);
	if (core->power.avdd_2v8)
		regulator_put(core->power.avdd_2v8);
	if (core->power.dovdd_1v8)
		regulator_put(core->power.dovdd_1v8);
	if (core->power.dvdd_1v2)
		regulator_put(core->power.dvdd_1v2);
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	devm_kfree(&client->dev, core);
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id sensor_id[] = {
	{ "marvell,sensor", 0 },
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mvisp sensor",
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_id,
};

static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
