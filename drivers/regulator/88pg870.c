/*
 * Marvell 88PG870 Buck Converter Regulator Driver.
 *
 * Copyright (c) 2013 Marvell Technology Ltd.
 * Yipeng Yao <ypyao@marvell.com>
 *
 * This package is free software; you can reinfostribute it and/or moinfofy
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/regmap.h>
#include <linux/regulator/88pg870.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of_device.h>

#define PG870_DVC_MAX_LVL		3
static unsigned int vsel_reg_map[] = {0x05, 0x06, 0x07, 0x08};
static unsigned int vsel_attr_level;

struct pg870_device_info {
	struct regmap *regmap;
	struct device *dev;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct regulator_init_data *regulator;
	/* IC Type and Rev */
	int chip_id;
	/* Voltage slew rate limiting */
	unsigned int ramp_rate;
	/* Voltage select table idx */
	unsigned int vsel_idx;
};


struct pg870_chip {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_client *companion;
	struct regmap *regmap;
	struct regmap_irq_chip *regmap_irq_chip;
	struct regmap_irq_chip_data *irq_data;
	unsigned char version;
	int type;
	int irq;
	int irq_base;
	unsigned long wu_flag;
	struct proc_dir_entry *proc_file;
};

static struct pg870_device_info *pg870_di;
/* If the system has several 88PG870 we need a infofferent id and name for each
 * of them...
 */
static DEFINE_IDR(regulator_id);
static DEFINE_MUTEX(regulator_mutex);

static inline int get_chip_id(struct pg870_device_info *info)
{
	unsigned int val;
	int ret;

	ret = regmap_read(info->regmap, PG870_CHIP_ID_REG, &val);
	if (ret < 0)
		return ret;
	return val;
}

static inline int check_range(struct pg870_device_info *info, int min_uv,
				int max_uv)
{
	if (min_uv < PG870_MIN_UV || max_uv > PG870_MAX_UV
		|| min_uv > PG870_MAX_UV) {
		return -EINVAL;
	}
	return 0;
}

/*
 * VOUT = 0.60V + NSELx * 12.5mV, from 0.6V to 1.6V.
 * VOUT = 1.60V + NSELx * 50mV, from 1.6V to 3.95V.
 */
static inline int choose_voltage(struct pg870_device_info *info,
				int min_uv, int max_uv)
{
	int data = 0;

	if (min_uv < PG870_MID_UV)
		data = (min_uv - PG870_MIN_UV) / 12500;
	else
		data = SEL_STEPS + (min_uv - PG870_MID_UV) / 50000;
	return data;
}

static int pg870_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	int vol = 0;

	if (selector < SEL_STEPS)
		vol = selector * 12500 + PG870_MIN_UV;
	else
		vol = (selector - SEL_STEPS) * 50000 + PG870_MID_UV;
	return vol;
}

static int pg870_set_voltage(struct regulator_dev *rdev,
				int min_uv, int max_uv, unsigned *selector)
{
	struct pg870_device_info *info = rdev_get_drvdata(rdev);
	unsigned int data;
	int ret = 0;

	if (check_range(info, min_uv, max_uv)) {
		dev_err(info->dev,
			"invalid voltage range (%d, %d) uv\n", min_uv, max_uv);
		return -EINVAL;
	}
	ret = choose_voltage(info, min_uv, max_uv);
	if (ret < 0)
		return ret;
	data = ret;
	*selector = data;

	return regmap_update_bits(info->regmap, vsel_reg_map[info->vsel_idx],
					0x7F, data);
}

static int pg870_get_voltage(struct regulator_dev *rdev)
{
	struct pg870_device_info *info = rdev_get_drvdata(rdev);
	unsigned int data;
	int ret;

	ret = regmap_read(info->regmap, vsel_reg_map[info->vsel_idx], &data);
	if (ret < 0)
		return ret;
	data = data & 0x7F;

	return pg870_list_voltage(rdev, data);
}

static struct regulator_ops pg870_regulator_ops = {
	.set_voltage = pg870_set_voltage,
	.get_voltage = pg870_get_voltage,
	.list_voltage = pg870_list_voltage,
};

static int pg870_set_ramp_rate(struct pg870_device_info *info,
				struct pg870_platform_data *pdata)
{
	unsigned int reg, data, mask;

	if (pdata->ramp_rate & 0x7)
		info->ramp_rate = pdata->ramp_rate;
	else
		info->ramp_rate = PG870_RAMP_RATE_14000UV;
	reg = PG870_CONTROL_REG;
	data = info->ramp_rate;
	mask = CTL_RAMP_MASK;
	return regmap_update_bits(info->regmap, reg, mask, data);
}

static int pg870_device_setup(struct pg870_device_info *info,
				struct pg870_platform_data *pdata)
{
	unsigned int data, sm, sv, i;
	unsigned int *map;
	int ret = 0;
	/* Set ramp rate */
	ret = pg870_set_ramp_rate(info, pdata);
	if (ret < 0) {
		dev_err(info->dev, "Fialed to set ramp rate!\n");
		return ret;
	}

	/* Set SYSCTRL */
	if (pdata->sysctrl)
		data = pdata->sysctrl;
	else
		data = PG870_DEF_SYSCTRL;
	ret = regmap_write(info->regmap, PG870_SYSCTRL_REG, data);
	if (ret < 0) {
		dev_err(info->dev, "Failed to set SYSCTRL!\n");
		return ret;
	}

	/* Set sleep mode */
	if (pdata->sleep_mode)
		sm = pdata->sleep_mode;
	else
		sm = PG870_SM_DISABLE;
	switch (sm) {
	case PG870_SM_TURN_OFF:
	case PG870_SM_RUN_SLEEP:
	case PG870_SM_LPM_SLEEP:
	case PG870_SM_RUN_ACTIVE:
		data = 0x40 | SM_PD | sm;
		break;
	case PG870_SM_DISABLE:
		data = 0x40;
		break;
	}
	ret = regmap_write(info->regmap, PG870_SLEEP_MODE_REG, data);
	if (ret < 0) {
		dev_err(info->dev, "Failed to set Sleep Mode register!\n");
		return ret;
	}

	if (pdata->sleep_vol)
		sv = pdata->sleep_vol;
	else
		sv = PG870_MIN_UV;
	data = choose_voltage(info, sv, sv);
	ret = regmap_write(info->regmap, PG870_SLEEP_VOL_REG, data);
	if (ret < 0) {
		dev_err(info->dev, "Failed to set Sleep Voltage register!\n");
		return ret;
	}

	/* init dvc table */
	map = pdata->dvc_init_map;
	if (map) {
		for (i = 0; i < 4; i++) {
			data = choose_voltage(info, map[i], map[i]);
			if (data < 0)
				continue;
			regmap_update_bits(info->regmap,
					     vsel_reg_map[i], 0x7F, data);
		}
	}
	/* Voltage select table idx */
	if (pdata->vsel_idx)
		info->vsel_idx = pdata->vsel_idx;
	else
		info->vsel_idx = 0;

	return ret;
}

int pg870_dvc_setvolt(unsigned int lvl, unsigned int uv)
{
	if (lvl > PG870_DVC_MAX_LVL)
		return -EINVAL;
	if (check_range(pg870_di, uv, uv)) {
		dev_err(pg870_di->dev,
			"invalid voltage setting: %d uv\n", uv);
		return -EINVAL;
	}
	uv = choose_voltage(pg870_di, uv, uv);
	if (uv < 0)
		return -EINVAL;
	return regmap_update_bits(pg870_di->regmap,
				vsel_reg_map[lvl], 0x7F, uv);
}
EXPORT_SYMBOL(pg870_dvc_setvolt);

int pg870_dvc_getvolt(unsigned int lvl)
{
	int ret, data;

	if (lvl > PG870_DVC_MAX_LVL)
		return -EINVAL;
	ret = regmap_read(pg870_di->regmap, vsel_reg_map[lvl], &data);
	if (ret < 0)
		return ret;
	data = data & 0x7F;

	return pg870_list_voltage(pg870_di->rdev, data);
}
EXPORT_SYMBOL(pg870_dvc_getvolt);

static struct regmap_config pg870_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};


int pg870_device_init(struct i2c_client *client)
{
	struct pg870_chip *chip;
	struct regmap *map;
	unsigned int val;
	int ret = 0;

	chip =
	    devm_kzalloc(&client->dev, sizeof(struct pg870_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	map = devm_regmap_init_i2c(client, &pg870_regmap_config);
	if (IS_ERR(map)) {
		ret = PTR_ERR(map);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	chip->client = client;
	chip->regmap = map;
	chip->dev = &client->dev;
	dev_set_drvdata(chip->dev, chip);
	i2c_set_clientdata(chip->client, chip);

	ret = regmap_read(chip->regmap, PG870_CHIP_ID_REG, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read CHIP ID: %d\n", ret);
		return ret;
	}

	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int pg870_dt_init(struct device_node *np,
			 struct device *dev,
			 struct pg870_platform_data *pdata)
{
	int ret;

	ret = of_property_read_u32(np, "88pg870-ramp-rate", &pdata->ramp_rate);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "88pg870-sysctrl", &pdata->sysctrl);
	if (ret)
		return ret;
	/* according to spec:make sure the reserved bits were rightly setted,
	PD_RESV1 and PD_RESV2 always be 1, PD_RESV3 always be 0 */
	pdata->sysctrl |= PD_RESV1 | PD_RESV2;
	pdata->sysctrl &= ~PD_RESV3;

	ret = of_property_read_u32(np, "88pg870-sleep-mode",
						&pdata->sleep_mode);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "88pg870-vsel-idx", &pdata->vsel_idx);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "88pg870-sleep-vol", &pdata->sleep_vol);
	if (ret)
		return ret;

	return 0;
}

static int voltage_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	int ret;

	ret = pg870_dvc_getvolt(vsel_attr_level);
	if (ret < 0) {
		dev_err(dev, "Can't get voltage!\n");
		return 0;
	}
	return sprintf(buf, "%d\n", ret);
}

static int voltage_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long vol;
	int ret;

	if (kstrtoul(buf, 10, &vol))
		dev_err(dev, "Invalid voltage!\n");
	ret = pg870_dvc_setvolt(vsel_attr_level, vol);
	if (ret < 0)
		dev_err(dev, "Can't set voltage!\n");
	return count;
}

static DEVICE_ATTR(voltage, S_IRUGO | S_IWUSR, voltage_show, voltage_store);

static int regs_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pg870_device_info *di = i2c_get_clientdata(client);
	unsigned int val;
	int cnt = 0, ret, i;

	/* STATUS */
	ret = regmap_read(di->regmap, PG870_STATUS_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "STATUS:  0x%02x\n", val);
	/* SYSCTRL */
	ret = regmap_read(di->regmap, PG870_SYSCTRL_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "SYSCTRL: 0x%02x\n", val);
	/* SLEEP MODE */
	ret = regmap_read(di->regmap, PG870_SLEEP_MODE_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "SLP_MOD: 0x%02x\n", val);
	/* VOL SETTING */
	for (i = 0; i < 4; i++) {
		ret = regmap_read(di->regmap, vsel_reg_map[i], &val);
		if (ret < 0)
			return ret;
		cnt += sprintf(buf + cnt, "VOL%d:    0x%02x\n", i, val);
	}
	/* SLEEP VOL SETTING */
	ret = regmap_read(di->regmap, PG870_SLEEP_VOL_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "SLP_VOL: 0x%02x\n", val);
	/* BUCK CONTROL */
	ret = regmap_read(di->regmap, PG870_CONTROL_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "CTRL:    0x%02x\n", val);
	/* ID */
	ret = regmap_read(di->regmap, PG870_CHIP_ID_REG, &val);
	if (ret < 0)
		return ret;
	cnt += sprintf(buf + cnt, "CHIP_ID: 0x%x\n", val);

	return cnt;
}

static DEVICE_ATTR(regs, S_IRUGO | S_IWUSR, regs_show, NULL);

static int level_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", vsel_attr_level);
}

static int level_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long lvl;

	if (kstrtoul(buf, 10, &lvl))
		dev_err(dev, "Invalid input!\n");
	if (lvl > PG870_DVC_MAX_LVL)
		dev_err(dev, "Invalid level number!\n");
	vsel_attr_level = lvl;
	return count;
}

static DEVICE_ATTR(level, S_IRUGO | S_IWUSR, level_show, level_store);

static struct attribute *pg870_attributes[] = {
	&dev_attr_voltage.attr,
	&dev_attr_regs.attr,
	&dev_attr_level.attr,
	NULL,
};

static struct attribute_group pg870_attr_grp = {
	.attrs = pg870_attributes,
};

static int pg870_regulator_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct pg870_device_info *di;
	struct pg870_platform_data *pdata = client->dev.platform_data;
	struct regulator_desc *rdesc;
	struct regulator_config config = { };
	struct device_node *np = client->dev.of_node;
	char *name;
	int num, ret = 0;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = pg870_dt_init(np, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		return -EINVAL;
	}

	di = devm_kzalloc(&client->dev, sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "Failed to allocate device info data\n");
		return -ENOMEM;
	}
	di->regmap = regmap_init_i2c(client, &pg870_regmap_config);
	if (IS_ERR(di->regmap)) {
		ret = PTR_ERR(di->regmap);
		dev_err(&client->dev, "Failed to allocate regmap\n");
		goto err_regmap;
	}

	di->dev = &client->dev;
	di->regulator = pdata->regulator;
	i2c_set_clientdata(client, di);

	mutex_lock(&regulator_mutex);
	num = idr_alloc(&regulator_id, di, 0, 0, GFP_KERNEL);
	mutex_unlock(&regulator_mutex);
	if (num < 0)
		return num;

	/* Generate a name with new id */
	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	/* Get chip ID */
	di->chip_id = get_chip_id(di);
	if (di->chip_id < 0) {
		dev_err(&client->dev, "Failed to get chip ID!\n");
		ret = -ENODEV;
		goto err_get_chip_id;
	}
	dev_info(&client->dev, "88PG870 Rev[0X%X] Detected!\n",
				di->chip_id);
	/* Device init */
	ret = pg870_device_setup(di, pdata);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to setup device!\n");
		goto err_device_setup;
	}

	rdesc = &di->desc;
	rdesc->name = name;
	rdesc->id = num;
	rdesc->ops = &pg870_regulator_ops;
	rdesc->type = REGULATOR_VOLTAGE;
	rdesc->n_voltages = PG870_NVOLTAGES;
	rdesc->owner = THIS_MODULE;

	config.dev = di->dev;
	config.init_data = of_get_regulator_init_data(&client->dev, np);
	config.driver_data = di;
	config.regmap = di->regmap;

	di->rdev = regulator_register(&di->desc, &config);
	if (IS_ERR(di->rdev)) {
		dev_err(&client->dev, "Failed to register regulator!\n");
		goto err_rdev_register;
	}

	/* Create sysfs interface */
	ret = sysfs_create_group(&client->dev.kobj, &pg870_attr_grp);
	if (ret) {
		dev_err(&client->dev, "Failed to create sysfs group!\n");
		goto err_create_sysfs;
	}
	pg870_di = di;
	return 0;

err_create_sysfs:
	regulator_unregister(di->rdev);
err_rdev_register:
err_device_setup:
err_get_chip_id:
	kfree(name);
	mutex_lock(&regulator_mutex);
	idr_remove(&regulator_id, num);
	mutex_unlock(&regulator_mutex);
err_regmap:
	devm_kfree(di->dev, di);
	return ret;

}

static int pg870_regulator_remove(struct i2c_client *client)
{
	struct pg870_device_info *info = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &pg870_attr_grp);
	regulator_unregister(info->rdev);

	kfree(info->desc.name);
	mutex_lock(&regulator_mutex);
	idr_remove(&regulator_id, info->desc.id);
	mutex_unlock(&regulator_mutex);

	regmap_exit(info->regmap);
	devm_kfree(info->dev, info);
	return 0;
}

static const struct i2c_device_id pg870_id_table[] = {
	{"88PG870", 0},
	{} /* NULL terminated */
};
MODULE_DEVICE_TABLE(i2c, pg870_id_table);

static const struct of_device_id pg870_dt_ids[] = {
	{ .compatible = "marvell,88pg870", },
	{},
};
MODULE_DEVICE_TABLE(of, pg870_dt_ids);


static struct i2c_driver pg870_regulator_driver = {
	.driver = {
		.name = "88PG870",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pg870_dt_ids),
	},
	.probe = pg870_regulator_probe,
	.remove = pg870_regulator_remove,
	.id_table = pg870_id_table,
};

static int __init pg870_init(void)
{
	return i2c_add_driver(&pg870_regulator_driver);
}
module_init(pg870_init);

static void __exit pg870_exit(void)
{
	i2c_del_driver(&pg870_regulator_driver);
}
module_exit(pg870_exit);

MODULE_DESCRIPTION("88PG870 regulator driver");
MODULE_LICENSE("GPL");

