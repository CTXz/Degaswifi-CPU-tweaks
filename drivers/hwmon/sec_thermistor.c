/*
 * sec_thermistor.c - SEC Thermistor
 *
 *  Copyright (C) 2013 Samsung Electronics
 *  Minsung Kim <ms925.kim@samsung.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_data/sec_thermistor.h>
#include <linux/mfd/88pm80x.h>

#define ADC_SAMPLING_CNT	7

struct sec_therm_info {
	struct device *dev;
	struct sec_therm_platform_data *pdata;
	char name[PLATFORM_NAME_SIZE];
};

#ifdef CONFIG_OF
static const struct of_device_id sec_therm_match[] = {
	{ .compatible = "samsung,sec-thermistor", },
	{ },
};
MODULE_DEVICE_TABLE(of, sec_therm_match);

static struct sec_therm_platform_data *
sec_therm_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sec_therm_platform_data *pdata;
	u32 len1, len2;
	int i;
	u32 adc, temp;

	if (!np)
		return NULL;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	if (!of_get_property(np, "adc_array", &len1))
		return ERR_PTR(-ENOENT);
	if (!of_get_property(np, "temp_array", &len2))
		return ERR_PTR(-ENOENT);

	if (len1 != len2) {
		dev_err(&pdev->dev, "%s: invalid array length(%u,%u)\n",
				__func__, len1, len2);
		return ERR_PTR(-EINVAL);
	}

	if (of_property_read_u32(np, "adc-channel", &pdata->adc_channel) < 0) {
		return ERR_PTR(-EINVAL);
	}
	pdata->adc_arr_size = len1 / sizeof(u32);
	pdata->adc_table = devm_kzalloc(&pdev->dev,
			sizeof(*pdata->adc_table) * pdata->adc_arr_size,
			GFP_KERNEL);
	if (!pdata->adc_table)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < pdata->adc_arr_size; i++) {
		if (of_property_read_u32_index(np, "adc_array", i, &adc))
			return ERR_PTR(-EINVAL);
		if (of_property_read_u32_index(np, "temp_array", i, &temp))
			return ERR_PTR(-EINVAL);

		pdata->adc_table[i].adc = (int)adc;
		pdata->adc_table[i].temperature = (int)temp;
	}

	return pdata;
}
#else
static struct sec_therm_platform_data *
sec_therm_parse_dt(struct platform_device *pdev) { return NULL; }
#endif

static int sec_therm_get_adc_data(struct sec_therm_info *info)
{
	int adc_data;
	int adc_max = 0, adc_min = 0, adc_total = 0;
	int i;

	for (i = 0; i < ADC_SAMPLING_CNT; i++) {
		adc_data = extern_get_gpadc_bias_volt(info->pdata->adc_channel, 31);
		if (adc_data < 0) {
			pr_info("%s : err(%d) returned, skip read\n",
				__func__, adc_data);
			return adc_data;
		}

		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_SAMPLING_CNT - 2);
}

static int convert_adc_to_temper(struct sec_therm_info *info, unsigned int adc)
{
	int low = 0;
	int high = 0;
	int mid = 0;

	if (!info->pdata->adc_table || !info->pdata->adc_arr_size) {
		/* using fake temp */
		return 300;
	}

	high = info->pdata->adc_arr_size - 1;

	if (adc >= info->pdata->adc_table[high].adc)
		return info->pdata->adc_table[high].temperature;

	while (low <= high) {
		mid = (low + high) / 2;
		if (info->pdata->adc_table[mid].adc > adc)
			high = mid - 1;
		else if (info->pdata->adc_table[mid + 1].adc <= adc)
			low = mid + 1;
		else
			break;
	}
	return info->pdata->adc_table[mid].temperature;
}

static ssize_t sec_therm_show_temperature(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct sec_therm_info *info = dev_get_drvdata(dev);
	int adc, temp;

	adc = sec_therm_get_adc_data(info);

	if (adc < 0)
		return adc;
	else
		temp = convert_adc_to_temper(info, adc);

	return sprintf(buf, "%d\n", temp);
}

static ssize_t sec_therm_show_temp_adc(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct sec_therm_info *info = dev_get_drvdata(dev);
	int adc;

	adc = sec_therm_get_adc_data(info);

	return sprintf(buf, "%d\n", adc);
}

static ssize_t sec_therm_show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_therm_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", info->name);
}

static DEVICE_ATTR(temperature, S_IRUGO, sec_therm_show_temperature,
		NULL);
static DEVICE_ATTR(temp_adc, S_IRUGO, sec_therm_show_temp_adc, NULL);
static DEVICE_ATTR(name, S_IRUGO, sec_therm_show_name, NULL);

static struct attribute *sec_therm_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_temp_adc.attr,
	&dev_attr_name.attr,
	NULL
};

static const struct attribute_group sec_therm_attr_group = {
	.attrs = sec_therm_attributes,
};

static int sec_therm_probe(struct platform_device *pdev)
{
	struct sec_therm_platform_data *pdata;
	struct sec_therm_info *info;
	int ret;

	pdata = sec_therm_parse_dt(pdev);
	if (IS_ERR(pdata)) {
		return PTR_ERR(pdata);
	} else if (pdata == NULL) {
		pdata = pdev->dev.platform_data;
	}

	if (!pdata) {
		return -ENODEV;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		return -ENOMEM;
	}

	pdev->name = "sec-thermistor";
	info->dev = &pdev->dev;
	info->pdata = pdata;
	platform_set_drvdata(pdev, info);

	ret = sysfs_create_group(&info->dev->kobj, &sec_therm_attr_group);
	if (ret) {
		goto err_create_sysfs;
	}

	return 0;

err_create_sysfs:
	return ret;
}

static int sec_therm_remove(struct platform_device *pdev)
{
	struct sec_therm_info *info = platform_get_drvdata(pdev);

	if (!info)
		return 0;

	sysfs_remove_group(&info->dev->kobj, &sec_therm_attr_group);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int sec_thermistor_shutdown(struct platform_device *pdev)
{
	struct sec_therm_info *info = platform_get_drvdata(pdev);

	if (!info)
		return 0;

	sysfs_remove_group(&info->dev->kobj, &sec_therm_attr_group);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sec_thermistor_driver = {
	.driver = {
		.name = "sec-thermistor",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sec_therm_match),
	},
	.probe = sec_therm_probe,
	.remove = sec_therm_remove,
	.shutdown = sec_thermistor_shutdown,
};

static int __init sec_therm_init(void)
{
    int ret;

    ret = platform_driver_register(&sec_thermistor_driver);

    return ret;
}

static void __exit sec_therm_exit(void)
{
    platform_driver_unregister(&sec_thermistor_driver);
}

module_init(sec_therm_init);
module_exit(sec_therm_exit);

MODULE_DESCRIPTION("SEC Thermistor Driver");
MODULE_AUTHOR("Minsung Kim <ms925.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sec-thermistor");
