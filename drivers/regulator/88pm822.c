/*
 * Regulators driver for Marvell 88PM822
 *
 * Copyright (C) 2013 Marvell International Ltd.
 * Yipeng Yao <ypyao@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/88pm822.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>

/* LDO1 with DVC[0..3] */
#define PM822_LDO1_VOUT		(0x08) /* VOUT1 */
#define PM822_LDO1_VOUT_2	(0x09)
#define PM822_LDO1_VOUT_3	(0x0A)
#define PM822_LDO2_VOUT		(0x0B)
#define PM822_LDO3_VOUT		(0x0C)
#define PM822_LDO4_VOUT		(0x0D)
#define PM822_LDO5_VOUT		(0x0E)
#define PM822_LDO6_VOUT		(0x0F)
#define PM822_LDO7_VOUT		(0x10)
#define PM822_LDO8_VOUT		(0x11)
#define PM822_LDO9_VOUT		(0x12)
#define PM822_LDO10_VOUT	(0x13)
#define PM822_LDO11_VOUT	(0x14)
#define PM822_LDO12_VOUT	(0x15)
#define PM822_LDO13_VOUT	(0x16)
#define PM822_LDO14_VOUT	(0x17)
#define PM822_VOUTSW_VOUT	(0xFF)	/* fake register */

/* BUCK1 with DVC[0..3] */
#define PM822_BUCK1_1		(0x3D)
#define PM822_BUCK1_2		(0x3E)
#define PM822_BUCK1_3		(0x3F)
#define PM822_BUCK4_1		(0x43)
#define PM822_BUCK4_2		(0x44)
#define PM822_BUCK4_3		(0x45)
#define PM822_BUCK5		(0x46)

#define PM822_BUCK_ENA		(0x50)
#define PM822_LDO_ENA1_1	(0x51)
#define PM822_LDO_ENA1_2	(0x52)
#define PM822_LDO_ENA1_3	(0x53)

#define PM822_LDO_ENA2_1	(0x56)
#define PM822_LDO_ENA2_2	(0x57)

#define PM822_BUCK1_MISC1	(0x78)
#define PM822_BUCK3_MISC1	(0x7E)
#define PM822_BUCK4_MISC1	(0x81)
#define PM822_BUCK5_MISC1	(0x84)


struct pm822_regulator_volt_range {
	int	min_uv;
	int	max_uv;
	int	step_uv;
	/* the register value for min_uv */
	int	min_val;
};

struct pm822_regulator_info {
	struct regulator_desc desc;
	int max_ua;

	struct pm822_regulator_volt_range *volt;
	unsigned int ranges;
};

struct pm822_regulators {
	struct regulator_dev *regulators[PM822_ID_RG_MAX];
	struct pm822_chip *chip;
	struct regmap *map;
};


/*
 * vreg - the buck regs string.
 * ereg - the string for the enable register.
 * ebit - the bit number in the enable register.
 * amax - the current
 * Buck has 2 kinds of voltage steps. It is easy to find voltage by ranges,
 * not the constant voltage table.
 */
#define PM822_BUCK(vreg, ereg, ebit, amax, volt_ranges)			\
{									\
	.desc	= {							\
		.name	= #vreg,					\
		.ops	= &pm822_volt_range_ops,			\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= PM822_ID_##vreg,				\
		.owner	= THIS_MODULE,					\
		.vsel_reg	= PM822_##vreg,				\
		.vsel_mask	= 0x7f,					\
		.enable_reg	= PM822_##ereg,				\
		.enable_mask	= 1 << (ebit),				\
	},								\
	.max_ua		= (amax),					\
	.volt		= (volt_ranges),				\
	.ranges		= ARRAY_SIZE(volt_ranges),			\
}

/*
 * vreg - the LDO regs string
 * ereg -  the string for the enable register.
 * ebit - the bit number in the enable register.
 * amax - the current
 * volt_table - the LDO voltage table
 * For all the LDOes, there are too many ranges. Using volt_table will be
 * simpler and faster.
 */
#define PM822_LDO(vreg, ereg, ebit, amax, ldo_volt_table)		\
{									\
	.desc	= {							\
		.name	= #vreg,					\
		.ops	= &pm822_volt_table_ops,			\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= PM822_ID_##vreg,				\
		.owner	= THIS_MODULE,					\
		.n_voltages = ARRAY_SIZE(ldo_volt_table),		\
		.vsel_reg	= PM822_##vreg##_VOUT,			\
		.vsel_mask	= 0x0f,					\
		.enable_reg	= PM822_##ereg,				\
		.enable_mask	= 1 << (ebit),				\
		.volt_table	= ldo_volt_table,			\
	},								\
	.max_ua		= (amax),					\
}

/* Ranges are sorted in ascending order. */
static struct pm822_regulator_volt_range buck1_volt_range[] = {
	{600000,	1587500,	12500,	0x0},
	{1600000,	1800000,	50000,	0x50},
};

/* BUCK 2~4 have same ranges. */
static struct pm822_regulator_volt_range buck2_4_volt_range[] = {
	{600000,	1587500,	12500,	0x0},
	{1600000,	3300000,	50000,	0x50},
};

static struct pm822_regulator_volt_range buck5_volt_range[] = {
	{600000,	1587500,	12500,	0x0},
	{1600000,	3950000,	50000,	0x50},
};

static const unsigned int ldo1_2_volt_table[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

/* LDO 3~11 have same voltage table. */
static const unsigned int ldo3_11_volt_table[] = {
	1200000, 1250000, 1700000, 1800000, 1850000, 1900000, 2500000, 2600000,
	2700000, 2750000, 2800000, 2850000, 2900000, 3000000, 3100000, 3300000,
};

static const unsigned int ldo12_volt_table[] = {
	600000,  650000,  700000,  750000,  800000,  850000,  900000,  950000,
	1000000, 1050000, 1100000, 1150000, 1200000, 1300000, 1400000, 1500000,
};

static const unsigned int ldo13_volt_table[] = {
	1700000, 1800000, 1900000, 2500000, 2800000, 2900000, 3100000, 3300000,
};

static const unsigned int ldo14_volt_table[] = {
	1700000, 1800000, 1900000, 2000000, 2100000, 2500000, 2700000, 2800000,
};

static const unsigned int voutsw_table[] = {
};

static int pm822_get_current_limit(struct regulator_dev *rdev)
{
	struct pm822_regulator_info *info = rdev_get_drvdata(rdev);

	return info->max_ua;
}

static int pm822_set_voltage(struct regulator_dev *rdev,
			     int min_uv, int max_uv, unsigned *selector)
{
	struct pm822_regulator_info *info = rdev_get_drvdata(rdev);
	struct pm822_regulator_volt_range *range;
	int i, best_index = -1;

	if (info->volt == NULL)
		return -EINVAL;

	if (info->desc.id == PM822_ID_VOUTSW)
		return 0;

	/*
	 * Ranges are sorted in ascending order. So if we found a best_uv
	 * in this range, we can break out.
	 */
	for (i = 0; i < info->ranges; i++) {
		range = &info->volt[i];

		if (min_uv <= range->max_uv && max_uv >= range->min_uv) {
			if (min_uv <= range->min_uv)
				best_index = 0;
			else
				best_index = (min_uv - range->min_uv +
					range->step_uv - 1) / range->step_uv;
			break;
		}
	}

	if (best_index == -1)
		return -EINVAL;

	*selector = best_index + range->min_val;

	return regulator_set_voltage_sel_regmap(rdev, *selector);
}

static int pm822_get_voltage(struct regulator_dev *rdev)
{
	struct pm822_regulator_info *info = rdev_get_drvdata(rdev);
	struct pm822_regulator_volt_range *range;
	int i, val, max_val, volt = -EINVAL;

	if (info->volt == NULL)
		return -EINVAL;

	val = regulator_get_voltage_sel_regmap(rdev);
	if (val < 0)
		return val;

	if (info->desc.id == PM822_ID_VOUTSW)
		return 0;

	/* get the voltage via the register value */
	for (i = 0; i < info->ranges; i++) {
		range = &info->volt[i];
		max_val = (range->max_uv - range->min_uv) / range->step_uv
				+ range->min_val;

		if (val >= range->min_val && val <= max_val) {
			volt = (val - range->min_val) * range->step_uv
				+ range->min_uv;
			break;
		}
	}

	return volt;
}

static struct regulator_ops pm822_volt_range_ops = {
	.set_voltage = pm822_set_voltage,
	.get_voltage = pm822_get_voltage,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_current_limit = pm822_get_current_limit,
};

static struct regulator_ops pm822_volt_table_ops = {
	.list_voltage = regulator_list_voltage_table,
	.map_voltage = regulator_map_voltage_iterate,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.get_current_limit = pm822_get_current_limit,
};

static struct pm822_regulator_info pm822_regulator_info[] = {
	PM822_BUCK(BUCK1, BUCK_ENA, 0, 3500000, buck1_volt_range),
	PM822_BUCK(BUCK2, BUCK_ENA, 1, 750000, buck2_4_volt_range),
	PM822_BUCK(BUCK3, BUCK_ENA, 2, 1500000, buck2_4_volt_range),
	PM822_BUCK(BUCK4, BUCK_ENA, 3, 750000, buck2_4_volt_range),
	PM822_BUCK(BUCK5, BUCK_ENA, 4, 1500000, buck5_volt_range),

	PM822_LDO(LDO1, LDO_ENA1_1, 0, 100000, ldo1_2_volt_table),
	PM822_LDO(LDO2, LDO_ENA1_1, 1, 100000, ldo1_2_volt_table),
	PM822_LDO(LDO3, LDO_ENA1_1, 2, 400000, ldo3_11_volt_table),
	PM822_LDO(LDO4, LDO_ENA1_1, 3, 400000, ldo3_11_volt_table),
	PM822_LDO(LDO5, LDO_ENA1_1, 4, 200000, ldo3_11_volt_table),
	PM822_LDO(LDO6, LDO_ENA1_1, 5, 200000, ldo3_11_volt_table),
	PM822_LDO(LDO7, LDO_ENA1_1, 6, 100000, ldo3_11_volt_table),
	PM822_LDO(LDO8, LDO_ENA1_1, 7, 100000, ldo3_11_volt_table),
	PM822_LDO(LDO9, LDO_ENA1_2, 0, 200000, ldo3_11_volt_table),
	PM822_LDO(LDO10, LDO_ENA1_2, 1, 400000, ldo3_11_volt_table),
	PM822_LDO(LDO11, LDO_ENA1_2, 2, 200000, ldo3_11_volt_table),
	PM822_LDO(LDO12, LDO_ENA1_2, 3, 400000, ldo12_volt_table),
	PM822_LDO(LDO13, LDO_ENA1_2, 4, 100000, ldo13_volt_table),
	PM822_LDO(LDO14, LDO_ENA1_2, 5, 8000, ldo14_volt_table),
	PM822_LDO(VOUTSW, MISC_EN1, 4, 0, voutsw_table),
};

#define PM822_REGULATOR_OF_MATCH(id)					\
	{								\
		.name = "88PM822-" #id,					\
		.driver_data = &pm822_regulator_info[PM822_ID_##id],	\
	}

static struct of_regulator_match pm822_regulator_matches[] = {
	PM822_REGULATOR_OF_MATCH(BUCK1),
	PM822_REGULATOR_OF_MATCH(BUCK2),
	PM822_REGULATOR_OF_MATCH(BUCK3),
	PM822_REGULATOR_OF_MATCH(BUCK4),
	PM822_REGULATOR_OF_MATCH(BUCK5),
	PM822_REGULATOR_OF_MATCH(LDO1),
	PM822_REGULATOR_OF_MATCH(LDO2),
	PM822_REGULATOR_OF_MATCH(LDO3),
	PM822_REGULATOR_OF_MATCH(LDO4),
	PM822_REGULATOR_OF_MATCH(LDO5),
	PM822_REGULATOR_OF_MATCH(LDO6),
	PM822_REGULATOR_OF_MATCH(LDO7),
	PM822_REGULATOR_OF_MATCH(LDO8),
	PM822_REGULATOR_OF_MATCH(LDO9),
	PM822_REGULATOR_OF_MATCH(LDO10),
	PM822_REGULATOR_OF_MATCH(LDO11),
	PM822_REGULATOR_OF_MATCH(LDO12),
	PM822_REGULATOR_OF_MATCH(LDO13),
	PM822_REGULATOR_OF_MATCH(LDO14),
	PM822_REGULATOR_OF_MATCH(VOUTSW),
};

static int pm822_regulator_dt_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = of_regulator_match(&pdev->dev, np,
				 pm822_regulator_matches,
				 ARRAY_SIZE(pm822_regulator_matches));
	if (ret < 0)
		return ret;

	return 0;
}

static int pm822_regulator_probe(struct platform_device *pdev)
{
	struct pm822_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm822_platform_data *pdata = (pdev->dev.parent)->platform_data;
	struct pm822_regulators *pm822_data;
	struct pm822_regulator_info *info;
	struct regulator_config config = { };
	struct regulator_init_data *init_data;
	int i, ret;

	if (!pdata || pdata->num_regulators == 0) {
		if (IS_ENABLED(CONFIG_OF)) {
			ret = pm822_regulator_dt_init(pdev);
			if (ret)
				return ret;
		} else {
			return -ENODEV;
		}
	} else if (pdata->num_regulators) {
		/* Check whether num_regulator is valid. */
		unsigned int count = 0;
		for (i = 0; pdata->regulators[i]; i++)
			count++;
		if (count != pdata->num_regulators)
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	pm822_data = devm_kzalloc(&pdev->dev, sizeof(*pm822_data),
					GFP_KERNEL);
	if (!pm822_data) {
		dev_err(&pdev->dev, "Failed to allocate pm822_regualtors");
		return -ENOMEM;
	}

	pm822_data->map = chip->subchip->regmap_power;
	pm822_data->chip = chip;

	platform_set_drvdata(pdev, pm822_data);

	for (i = 0; i < PM822_ID_RG_MAX; i++) {
		if (!pdata || pdata->num_regulators == 0)
			init_data = pm822_regulator_matches[i].init_data;
		else
			init_data = pdata->regulators[i];
		if (!init_data)
			continue;
		info = pm822_regulator_matches[i].driver_data;
		config.dev = &pdev->dev;
		config.init_data = init_data;
		config.driver_data = info;
		config.regmap = pm822_data->map;
		config.of_node = pm822_regulator_matches[i].of_node;

		pm822_data->regulators[i] =
				regulator_register(&info->desc, &config);
		if (IS_ERR(pm822_data->regulators[i])) {
			ret = PTR_ERR(pm822_data->regulators[i]);
			dev_err(&pdev->dev, "Failed to register %s\n",
				info->desc.name);

			while (--i >= 0 && pm822_data->regulators[i])
				regulator_unregister(pm822_data->regulators[i]);

			return ret;
		}
	}

	return 0;
}

static int pm822_regulator_remove(struct platform_device *pdev)
{
	struct pm822_regulators *pm822_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; pm822_data->regulators[i] && i < PM822_ID_RG_MAX; i++)
		regulator_unregister(pm822_data->regulators[i]);

	return 0;
}

static struct platform_driver pm822_regulator_driver = {
	.driver		= {
		.name	= "88pm822-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= pm822_regulator_probe,
	.remove		= pm822_regulator_remove,
};

static int __init pm822_regulator_init(void)
{
	return platform_driver_register(&pm822_regulator_driver);
}
subsys_initcall(pm822_regulator_init);

static void __exit pm822_regulator_exit(void)
{
	platform_driver_unregister(&pm822_regulator_driver);
}
module_exit(pm822_regulator_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Regulator Driver for Marvell 88PM822 PMIC");
MODULE_ALIAS("platform:88pm822-regulator");
