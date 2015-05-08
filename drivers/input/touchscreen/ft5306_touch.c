/*
 * * Copyright (C) 2009, Notioni Corporation chenjian@notioni.com).
 * *
 * * Author: jeremy.chen
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/suspend.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define MAX_FINGER		(5)  /* ft5306 can only support max 5 point */

#define FT5306_LEN 31

#define POINT_PUTDOWN		(0)
#define POINT_PUTUP			(1)
#define POINT_CONTACT		(2)
#define POINT_INVALID		(3)

struct touch_finger {
	int pi;			/* point index */
	int ps;			/* point status */
	u16 px;			/* point x */
	u16 py;			/* point y */
};

struct ft5306_touch {
	struct input_dev *idev;
	struct input_dev *virtual_key;
	struct i2c_client *i2c;
	struct work_struct work;
	struct touch_finger finger[MAX_FINGER];
	struct mutex lock;
	struct regulator *power_ldo;
	int reset_gpio;
	int reset_gpio_invert;
	int abs_x_max;
	int abs_y_max;
	int abs_flags;
	int power_on;
	int irq;
	struct pm_qos_request	cpufreq_qos_req_min;
};

#ifdef CONFIG_PM_RUNTIME
static u8 ft5306_mode_cmd_sleep[2] = { 0xA5, 0x03 };
#endif
static u8 ft5306_cmd[2] = { 0x0, 0x0 };

int ft5306_touch_read_reg(struct ft5306_touch *touch, u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_read_byte_data(touch->i2c, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}

int ft5306_touch_write_reg(struct ft5306_touch *touch, u8 reg, u8 val)
{
	int ret;
	int status;

	if (touch->i2c == NULL)
		return -1;
	ret = i2c_smbus_write_byte_data(touch->i2c, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}

static int ft5306_touch_read(struct ft5306_touch *touch, char *buf, int count)
{
	int ret;

	ret = i2c_master_recv(touch->i2c, (char *) buf, count);

	return ret;
}

static int ft5306_touch_write(struct ft5306_touch *touch, char *buf, int count)
{
	int ret;

	ret = i2c_master_send(touch->i2c, buf, count);

	return ret;
}

static u8 buflog[FT5306_LEN * 5], *pbuf;
static inline int ft5306_touch_read_data(struct ft5306_touch *touch)
{
	int ps, pi, i, b, ret;
	u8 buf[FT5306_LEN];
	u16 px, py;

	memset(touch->finger, 0xFF, MAX_FINGER * sizeof(struct touch_finger));

	ret = ft5306_touch_read(touch, buf, FT5306_LEN);
	if (ret < 0)
		goto out;

	pbuf = buflog;
	for (i = 0; i < FT5306_LEN; ++i)
		pbuf += sprintf(pbuf, "%02x ", buf[i]);

	dev_dbg(&touch->i2c->dev, "RAW DATA: %s\n", buflog);

	for (i = 0; i < MAX_FINGER; ++i) {
		b = 3 + i * 6;
		px = ((u16) (buf[b + 0] & 0x0F) << 8) | (u16) buf[b + 1];
		py = ((u16) (buf[b + 2] & 0x0F) << 8) | (u16) buf[b + 3];
		ps = ((u16) (buf[b + 0] & 0xC0) >> 6);
		pi = ((u16) (buf[b + 2] & 0xF0) >> 4);

		touch->finger[i].px = px;
		touch->finger[i].py = py;
		touch->finger[i].ps = ps;
		touch->finger[i].pi = pi;
	}
out:
	return ret;
}

static void ft5306_touch_work(struct work_struct *work)
{
	struct ft5306_touch *touch =
			container_of(work, struct ft5306_touch, work);
	struct i2c_client *client = touch->i2c;
	struct input_dev *input_dev = touch->idev;
	int status, i, ret;
	int pi, ps;
	u16 px, py, tmp;

	if (!touch->power_on)
		return;

	dev_dbg(&client->dev, "I'm in ft5306 work\n");

	/*pm_qos_update_request_timeout(&touch->cpufreq_qos_req_min,
		LONG_MAX, 200000);*/

	ret = ft5306_touch_read_data(touch);

	for (i = 0; i < MAX_FINGER; ++i) {

		dev_dbg(&client->dev,
			"REPP: i=%d pi=%d ps=0x%02x px=%d py=%d\n",
			i, touch->finger[i].pi, touch->finger[i].ps,
			touch->finger[i].px, touch->finger[i].py);

		ps = touch->finger[i].ps;
		if (POINT_INVALID == ps)
			continue;

		pi = touch->finger[i].pi;
		status = (POINT_PUTUP != ps);

		input_mt_slot(input_dev, pi);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, status);

		if (status) {
			px = touch->finger[i].px;
			py = touch->finger[i].py;
			if (touch->abs_flags == 1) {
				tmp = px;
				px = py;
				py = tmp;
			} else if (touch->abs_flags == 2) {
				tmp = px;
				px = py;
				py = touch->abs_y_max - tmp;
			} else if (touch->abs_flags == 3) {
				tmp = px;
				px = touch->abs_x_max - py;
				py = tmp;
			}
			dev_dbg(&client->dev, "x: %d y:%d\n", px, py);
			input_report_abs(input_dev, ABS_MT_POSITION_X, px);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, py);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 16);
		}
	}

	input_sync(input_dev);
}

static irqreturn_t ft5306_touch_irq_handler(int irq, void *dev_id)
{
	struct ft5306_touch *touch = dev_id;

	dev_dbg(&touch->i2c->dev, "ft5306_touch_irq_handler.\n");

	schedule_work(&touch->work);

	return IRQ_HANDLED;
}

static int index;
static ssize_t ft5306_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	u8 reg_val;
	struct ft5306_touch *touch = dev_get_drvdata(dev);

	if ((index < 0) || (index > FT5306_LEN))
		return 0;

	ft5306_touch_read_reg(touch, index, (u8 *) &reg_val);
	dev_info(dev, "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t ft5306_reg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t len)
{
	int ret;
	char vol[256] = { 0 };
	long unsigned int reg = 0, val = 0;
	int i;
	struct ft5306_touch *touch = dev_get_drvdata(dev);

	if (len > 256)
		len = 256;

	if ('w' == buff[0]) {
		memcpy(vol, buff + 2, 4);
		ret = (int) kstrtoul(vol, 16, &reg);
		memcpy(vol, buff + 7, 4);
		ret = (int) kstrtoul(vol, 16, &val);
		ft5306_cmd[0] = reg;
		ft5306_cmd[1] = val;
		ret = ft5306_touch_write(touch, ft5306_cmd, 2);
		dev_info(dev, "write! reg:0x%lx, val:0x%lx\n", reg, val);

	} else if ('r' == buff[0]) {
		memcpy(vol, buff + 2, 4);
		ret = (int) kstrtoul(vol, 16, &reg);
		ret = ft5306_touch_read_reg(touch, reg, (u8 *) &val);
		dev_info(dev, "Read! reg:0x%lx, val:0x%lx\n", reg, val);

	} else if ('d' == buff[0]) {
		for (i = 0x00; i <= 0x3E; i++) {
			reg = i;
			ft5306_touch_read_reg(touch, reg, (u8 *) &val);
			usleep_range(2000, 2500);
			dev_info(dev, "Display! reg:0x%lx, val:0x%lx\n",
				 reg, val);
		}
	}
	return len;
}

static DEVICE_ATTR(reg_show, 0444, ft5306_reg_show, NULL);
static DEVICE_ATTR(reg_store, 0664, NULL, ft5306_reg_store);

static struct attribute *ft5306_attributes[] = {
	&dev_attr_reg_show.attr,
	&dev_attr_reg_store.attr,
	NULL
};

static const struct attribute_group ft5306_attr_group = {
	.attrs = ft5306_attributes,
};

#ifdef CONFIG_PM_RUNTIME
static void ft5306_touch_wakeup_reset(struct ft5306_touch *touch)
{
	struct input_dev *input_dev = touch->idev;
	int i = 0, ret = 0;

	ret = ft5306_touch_read_data(touch);

	for (i = 0; i < MAX_FINGER; ++i) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(input_dev);
	return;
}

static int ft5306_runtime_suspend(struct device *dev)
{
	int ret, i = 0;
	struct ft5306_touch *touch = dev_get_drvdata(dev);

sleep_retry:
	ret = ft5306_touch_write(touch, ft5306_mode_cmd_sleep, 2);
	if (ret < 0) {
		if (i < 10) {
			usleep_range(5000, 5500);
			i++;
			dev_dbg(&touch->i2c->dev,
			"ft5306_touch can't enter sleep, retry %d\n", i);
			goto sleep_retry;
		}
		dev_info(&touch->i2c->dev,
			"ft5306_touch can't enter sleep\n");
		return 0;
	} else
		dev_dbg(&touch->i2c->dev,
			"ft5306_touch enter sleep mode.\n");

	if (touch->power_ldo && touch->power_on) {
		regulator_disable(touch->power_ldo);
		mutex_lock(&touch->lock);
		touch->power_on = 0;
		mutex_unlock(&touch->lock);
	}
	return 0;
}

static int ft5306_runtime_resume(struct device *dev)
{
	struct ft5306_touch *touch = dev_get_drvdata(dev);
	int ret;

	if (touch->power_ldo && !touch->power_on) {
		regulator_set_voltage(touch->power_ldo, 3100000, 3100000);
		ret = regulator_enable(touch->power_ldo);
		if (ret) {
			dev_err(dev, "failed to enable power_ldo\n");
			return ret;
		}
	}

	usleep_range(10000, 10500);
	if (gpio_is_valid(touch->reset_gpio)) {
		int value = touch->reset_gpio_invert ? 0 : 1;

		gpio_direction_output(touch->reset_gpio, value);
		usleep_range(1000, 1500);
		gpio_direction_output(touch->reset_gpio, !value);
		usleep_range(5000, 5500);
		gpio_direction_output(touch->reset_gpio, value);
		/*
		 * According to spec, the min time of starting to
		 * report point after powering on or resetting is
		 * 300ms. Set it to 200ms since touch works ok
		 * after optimizing.
		 */
		msleep(200);
	}

	ft5306_touch_wakeup_reset(touch);
	mutex_lock(&touch->lock);
	if (!touch->power_on)
		touch->power_on = 1;
	mutex_unlock(&touch->lock);
	return 0;
}
#endif				/* CONFIG_PM_RUNTIME */

static int ft5306_touch_parse_dt(struct device_node *np,
				 struct device *dev,
				 struct ft5306_touch *touch)
{
	int ret;
	enum of_gpio_flags flags;

	ret = of_property_read_u32(np, "ft5306,abs-x-max", &touch->abs_x_max);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ft5306,abs-y-max", &touch->abs_y_max);
	if (ret)
		return ret;
	ret = of_property_read_u32(np, "ft5306,abs-flags", &touch->abs_flags);
	if (ret)
		return ret;

	touch->power_ldo = devm_regulator_get(dev, "power-ldo");
	if (IS_ERR(touch->power_ldo))
		touch->power_ldo = NULL;

	touch->reset_gpio = of_get_named_gpio_flags(np, "reset-gpio", 0, &flags);
	if (gpio_is_valid(touch->reset_gpio)) {
		ret = devm_gpio_request_one(dev, touch->reset_gpio, GPIOF_DIR_OUT,
					    "ft5306-reset-gpio");
		if (ret)
			return ret;
		if (!(flags & OF_GPIO_ACTIVE_LOW))
			touch->reset_gpio_invert = 1;
	}

	return 0;
}

#define VIRT_KEYS(x...)  __stringify(x)
static ssize_t virtual_keys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_MENU)
		":67:1010:135:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_HOMEPAGE)
		":202:1010:135:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_SEARCH)
		":338:1010:135:100" ":"
		VIRT_KEYS(EV_KEY) ":" VIRT_KEYS(KEY_BACK)
		":472:1010:135:100\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.ft5306-ts",
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *props_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group props_attr_group = {
	.attrs = props_attrs,
};

static int ft5306_touch_set_virtual_key(struct ft5306_touch *touch)
{
	struct kobject *props_kobj;
	int ret = 0;

	props_kobj = kobject_create_and_add("board_properties", NULL);
	if (props_kobj)
		ret = sysfs_create_group(props_kobj, &props_attr_group);
	if (!props_kobj || ret) {
		dev_err(&touch->i2c->dev,
			"failed to create board_properties\n");
		return ret;
	}

	return 0;
}

static int ft5306_touch_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	struct ft5306_touch *touch;
	int ret;
	u8 reg_val;

	dev_dbg(&client->dev, "ft5306_touch.c----ft5306_touch_probe.\n");

	touch = devm_kzalloc(&client->dev, sizeof(struct ft5306_touch),
			     GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;

	touch->i2c = client;
	touch->irq = client->irq;
	mutex_init(&touch->lock);

	ret = ft5306_touch_parse_dt(client->dev.of_node, &client->dev, touch);
	if (ret) {
		dev_err(&client->dev, "failed to parse dt, error %d\n", ret);
		return ret;
	}

	/* register input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out;
	}

	touch->idev = input_dev;
	touch->idev->name = "ft5306-ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_mt_init_slots(input_dev, MAX_FINGER, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     touch->abs_x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     touch->abs_y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 16, 0, 0);

	ret = input_register_device(touch->idev);
	if (ret) {
		dev_dbg(&client->dev,
			"%s: unabled to register input device, ret = %d\n",
			__func__, ret);
		goto out_rg;
	}

	i2c_set_clientdata(client, touch);

	pm_runtime_enable(&client->dev);
	pm_runtime_get_sync(&client->dev);
	ret = ft5306_touch_read_reg(touch, 0x00, (u8 *) &reg_val);
	pm_runtime_put_sync_suspend(&client->dev);
	if (ret < 0) {
		dev_dbg(&client->dev, "ft5306 detect fail!\n");
		touch->i2c = NULL;
		goto out_rg;
	} else {
		dev_dbg(&client->dev, "ft5306 detect ok.\n");
	}

	ret = devm_request_irq(&client->dev, touch->irq,
			       ft5306_touch_irq_handler,
			       IRQF_DISABLED | IRQF_TRIGGER_FALLING,
			       "ft5306 touch", touch);
	if (ret < 0) {
		dev_info(&client->dev,
			"Request IRQ for Bigstream touch failed, return:%d\n",
			ret);
		goto out_rg;
	}
	INIT_WORK(&touch->work, ft5306_touch_work);

	touch->cpufreq_qos_req_min.name = "ft5306-ts";
	pm_qos_add_request(&touch->cpufreq_qos_req_min,
			PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);

	ret = sysfs_create_group(&client->dev.kobj, &ft5306_attr_group);
	if (ret)
		goto out_rg;

	ret = ft5306_touch_set_virtual_key(touch);
	if (ret)
		goto out_sysfs;

	pm_runtime_forbid(&client->dev);
	return 0;

out_sysfs:
	sysfs_remove_group(&client->dev.kobj, &ft5306_attr_group);
out_rg:
	pm_runtime_disable(&client->dev);
	input_free_device(touch->idev);
out:
	return ret;

}

static int ft5306_touch_remove(struct i2c_client *client)
{
	struct ft5306_touch *touch = i2c_get_clientdata(client);

	pm_runtime_disable(&client->dev);
	free_irq(touch->irq, touch);
	sysfs_remove_group(&client->dev.kobj, &ft5306_attr_group);
	input_unregister_device(touch->idev);
	return 0;
}

static const struct dev_pm_ops ft5306_ts_pmops = {
	SET_RUNTIME_PM_OPS(ft5306_runtime_suspend,
			   ft5306_runtime_resume, NULL)
};

static const struct of_device_id ft5306_dt_ids[] = {
	{ .compatible = "notioni,ft5306", },
	{},
};
MODULE_DEVICE_TABLE(of, ft5306_dt_ids);

static const struct i2c_device_id ft5306_touch_id[] = {
	{"ft5306_touch", 0},
	{}
};

static struct i2c_driver ft5306_touch_driver = {
	.driver = {
		.name = "ft5306_touch",
		.owner = THIS_MODULE,
		.pm = &ft5306_ts_pmops,
		.of_match_table	= of_match_ptr(ft5306_dt_ids),
	},
	.id_table = ft5306_touch_id,
	.probe = ft5306_touch_probe,
	.remove = ft5306_touch_remove,
};

module_i2c_driver(ft5306_touch_driver);

MODULE_DESCRIPTION("ft5306 touch Driver");
MODULE_LICENSE("GPL");
