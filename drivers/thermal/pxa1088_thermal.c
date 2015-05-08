/*
 * linux/driver/thermal/pxa1088_thermal.c
 *
 * Author:      Liang Chen <chl@marvell.com>
 * Copyright:   (C) 2013 Marvell International Ltd.
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
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_data/pxa_thermal.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/of.h>
#include <mach/addr-map.h>
#include <asm/cputype.h>

/* debug: Use for sysfs set temp */
/* #define DEBUG_TEMPERATURE */
#define TRIP_POINTS_NUM	4
#define TRIP_POINTS_ACTIVE_NUM (TRIP_POINTS_NUM - 1)

#define THERMAL_VIRT_BASE (APB_VIRT_BASE + 0x13200)
#define THERMAL_REG(x) (THERMAL_VIRT_BASE + (x))
#define THERMAL_TS_CTRL (0x20)
#define THERMAL_TS_READ (0x24)
#define THERMAL_TS_CLR (0x28)
#define THERMAL_TS_THD (0x2c)
#define THERMAL_TS_DUR (0x30)
#define THERMAL_TS_CNT (0x34)

/* TS_CTRL 0x20 */
#define TS_HW_AUTO_ENABLE (1 << 29)
#define TS_OVER_RANGE_RST_ENABLE (1 << 28)
#define TS_WARNING_MASK (1 << 25)
#define TS_ON_INT_MASK (1 << 18)
#define TS_CTRL_TSEN_TEMP_ON (1 << 3)
#define TS_CTRL_RST_N_TSEN (1 << 2)
#define TS_CTRL_TSEN_LOW_RANGE (1 << 1)
#define TS_CTRL_TSEN_CHOP_EN (1 << 0)
/* TS_READ 0x24 */
#define TS_RST_FLAG (1 << 12)
#define TS_WRAINGING_INT (1 << 9)
#define TS_READ_TS_ON (1 << 4)
#define TS_READ_OUT_DATA (0xf << 0)
/* TS_CLR 0x28 */
#define TS_ON_INT_CLR (1 << 4)
#define TS_OVER_RANGE_INT_CLR (1 << 3)
#define TS_HIGH_INT_CLR (1 << 2)
#define TS_WAINING_INT_CLR (1 << 1)
#define TS_RST_FLAG_CLR (1 << 0)

/* In-kernel thermal framework related macros & definations */
#define SENSOR_NAME_LEN	16
#define MAX_TRIP_COUNT	8
#define MAX_COOLING_DEVICE 4

#define MCELSIUS	1000

/* CPU Zone information */
#define PANIC_ZONE      3
#define WARN_ZONE2       2
#define WARN_ZONE1       1
#define WARN_ZONE0       0

#define PXA_ZONE_COUNT	4

struct pxa_tmu_data {
	struct pxa_tmu_platform_data *pdata;
	struct resource *mem;
	void __iomem *base;
	int irq;
	enum soc_type soc;
	struct work_struct irq_work;
	struct mutex lock;
	struct clk *clk;
	u8 temp_error1, temp_error2;
	int mode;
	int state;
	int old_state;
};

struct	thermal_trip_point_conf {
	int trip_val[MAX_TRIP_COUNT];
	int trip_count;
	u8 trigger_falling;
};

struct	thermal_cooling_conf {
	struct freq_clip_table freq_data[MAX_TRIP_COUNT];
	int freq_clip_count;
	int bind_trip;
};

struct thermal_sensor_conf {
	char name[SENSOR_NAME_LEN];
	int (*read_temperature)(void *data);
	struct thermal_trip_point_conf trip_data;
	struct thermal_cooling_conf cooling_data;
	void *private_data;
};

struct pxa_thermal_zone {
	enum thermal_device_mode mode;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	struct platform_device *pxa_dev;
	struct thermal_sensor_conf *sensor_conf;
	bool bind;
};

static int cpu_thermal_trips_temp[TRIP_POINTS_NUM] = {
	90,			/* bind to active type */
	100,			/* bind to active type */
	107,			/* bind to active type */
	112,			/* bind to critical type */
};

static struct pxa_thermal_zone *th_zone;
static void pxa_unregister_thermal(void);
static int pxa_register_thermal(struct thermal_sensor_conf *sensor_conf);
static void pxa_tmu_control(bool on);
struct pxa_tmu_data *pxa1088_thermal_data;


static int pxa_tmu_initialize(struct platform_device *pdev)
{
	struct pxa_tmu_data *data = platform_get_drvdata(pdev);
	int ret = 0;
	unsigned long ts_ctrl;
	mutex_lock(&data->lock);

	ts_ctrl = readl(data->base + THERMAL_TS_CTRL);
	ts_ctrl |= TS_CTRL_TSEN_CHOP_EN;
	/* we only care greater than 80 */
	ts_ctrl &= ~TS_CTRL_TSEN_LOW_RANGE;
	writel(ts_ctrl, data->base + THERMAL_TS_CTRL);

	mutex_unlock(&data->lock);

	pxa_tmu_control(true);

	return ret;
}

static void thermal_module_reset(int interval_temp)
{
	int i;
	pxa_tmu_control(false);
	/*
	 * delay 2s to reset module, during this time,
	 * no lpm allowed
	 */
	for (i = 0; i < 100000; i++)
		usleep_range(20, 30);

	pxa_tmu_control(true);

	pr_info("thermal module reset, interval temp %d\n", interval_temp);
}

/* This function decode 4bit long number of gray code into original binary */
static int gray_decode(unsigned int gray)
{
	int num, i, tmp;

	if (gray >= 16)
		return 0;

	num = gray & 0x8;
	tmp = num >> 3;
	for (i = 2; i >= 0; i--) {
		tmp = ((gray & (1 << i)) >> i) ^ tmp;
		num |= tmp << i;
	}
	return num;
}

/*
 * This function may be called from interrupt based temperature sensor
 * when threshold is changed.
 */
static void pxa_report_trigger(struct pxa_tmu_data *data, int temp)
{
	unsigned int i;
	int max, min;
	char *temp_info[3] = { "TYPE=thsens_cpu", "TEMP=100000", NULL };
	if (!th_zone || !th_zone->therm_dev || !th_zone->sensor_conf)
		return;

	/* Find the level for which trip happened */
	for (i = 0; i < th_zone->sensor_conf->trip_data.trip_count; i++) {
		if (temp <
			th_zone->sensor_conf->trip_data.trip_val[i] * MCELSIUS)
			break;
	}

	data->state = i;

	if (data->state == data->old_state)
		goto out;

	max = max(data->state, data->old_state);
	min = min(data->state, data->old_state);

	for (i = min; i < max; i++) {
		sprintf(temp_info[1], "TEMP=%d", temp);
		kobject_uevent_env(&th_zone->therm_dev->device.kobj,
					KOBJ_CHANGE, temp_info);
	}

	data->old_state = data->state;

out:
	return;
}

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(u8 temp_code)
{
	int temp = 0;

	/* temp_code should range between 75 and 175 */
	if (temp_code == 0) {
		temp = -ENODATA;
		goto out;
	}

	temp = (gray_decode(temp_code)) * 5 / 2 + 80;
out:
	if (temp <= 0)
		temp = 80;
	return temp;
}

#define RETRY_TIMES (10)
static int pxa_get_temp(struct thermal_zone_device *thermal,
			unsigned long *temp)
{
	int i = 0;
	unsigned long ts_read;
	int interval_temp = 0;
	int gray_code = 0;
	int ret = 0;
	struct pxa_tmu_data *data = pxa1088_thermal_data;

	if (!th_zone->therm_dev) {
		*temp = 80000;
		return 0;
	}

	ts_read = readl(data->base + THERMAL_TS_READ);
	if (likely(ts_read & TS_READ_TS_ON)) {
		gray_code = ts_read & TS_READ_OUT_DATA;
		*temp = (code_to_temp(gray_code)) * MCELSIUS;
	} else {
		for (i = 0; i < RETRY_TIMES; i++) {
			ts_read = readl(data->base + THERMAL_TS_READ);
			if (ts_read & TS_READ_TS_ON) {
				gray_code = ts_read & TS_READ_OUT_DATA;
				break;
			}
			msleep(20);
		}
		if (RETRY_TIMES == i) {
			*temp = 0;
			ret = -1;
		} else
			*temp = code_to_temp(gray_code) * MCELSIUS;
	}
	/* restart measure */
	pxa_tmu_control(false);
	pxa_tmu_control(true);

	if (!th_zone->therm_dev->last_temperature) {
		/* Let's recognize it's unnormal first time >= 110C */
		if (*temp >= 110000) {
			thermal_module_reset(*temp - 80000);
			*temp = 80000;
		}
	} else {
		interval_temp = (*temp >=
			th_zone->therm_dev->last_temperature) ?
			(*temp - th_zone->therm_dev->last_temperature) :
			(th_zone->therm_dev->last_temperature - *temp);
		/*
		 * it's impossible interval temp > 20C,
		 * currently, it's for EMEI-251:
		 * after thermal come out of "reset" and "clock off"
		 * there is posibility second read get false high
		 * temperature
		 */
		if (interval_temp > 20000) {
			thermal_module_reset(interval_temp);
			*temp = th_zone->therm_dev->last_temperature;
		}
	}
	pxa_report_trigger(th_zone->sensor_conf->private_data, *temp);
	return 0;
}


/* Get mode callback functions for thermal zone */
static int pxa_get_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode *mode)
{
	if (th_zone)
		*mode = th_zone->mode;
	return 0;
}

/* Set mode callback functions for thermal zone */
static int pxa_set_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode mode)
{
	if (!th_zone->therm_dev) {
		pr_notice("thermal zone not registered\n");
		return 0;
	}

	th_zone->mode = mode;
	thermal_zone_device_update(th_zone->therm_dev);

	return 0;
}

/* Get trip type callback functions for thermal zone */
static int pxa_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	switch (trip) {
	case WARN_ZONE0:
	case WARN_ZONE1:
	case WARN_ZONE2:
		*type = THERMAL_TRIP_ACTIVE;
		break;
	case PANIC_ZONE:
		*type = THERMAL_TRIP_CRITICAL;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* Get trip temperature callback functions for thermal zone */
static int pxa_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				unsigned long *temp)
{
	if ((trip < 0) || (trip > TRIP_POINTS_NUM))
		return -EINVAL;

	*temp = th_zone->sensor_conf->trip_data.trip_val[trip];
	/* convert the temperature into millicelsius */
	*temp = *temp * MCELSIUS;

	return 0;
}

/* Get critical temperature callback functions for thermal zone */
static int pxa_get_crit_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	int ret;
	/* Panic zone */
	ret = pxa_get_trip_temp(thermal, PANIC_ZONE, temp);
	return ret;
}

/* Operation callback functions for thermal zone */
static struct thermal_zone_device_ops const pxa_dev_ops = {
	.get_temp = pxa_get_temp,
	.get_mode = pxa_get_mode,
	.set_mode = pxa_set_mode,
	.get_trip_type = pxa_get_trip_type,
	.get_trip_temp = pxa_get_trip_temp,
	.get_crit_temp = pxa_get_crit_temp,
};

static struct thermal_sensor_conf pxa_sensor_conf = {
	.name			= "pxa1088-thermal",
	.read_temperature	= NULL,
};

/* Un-Register with the in-kernel thermal management */
static void pxa_unregister_thermal(void)
{
	int i;

	if (!th_zone)
		return;

	if (th_zone->therm_dev)
		thermal_zone_device_unregister(th_zone->therm_dev);

	for (i = 0; i < th_zone->cool_dev_size; i++) {
		if (th_zone->cool_dev[i])
			cpufreq_cooling_unregister(th_zone->cool_dev[i]);
	}

	kfree(th_zone);
	pr_info("Pxa: Kernel Thermal management unregistered\n");
}


/* Register with the in-kernel thermal management */
static int pxa_register_thermal(struct thermal_sensor_conf *sensor_conf)
{
	int ret;
	struct cpumask mask_val;

	if (!sensor_conf) {
		pr_err("Temperature sensor not initialised\n");
		return -EINVAL;
	}

	th_zone = kzalloc(sizeof(struct pxa_thermal_zone), GFP_KERNEL);
	if (!th_zone)
		return -ENOMEM;

	th_zone->sensor_conf = sensor_conf;
	cpumask_set_cpu(0, &mask_val);
	th_zone->cool_dev[0] = cpufreq_cooling_register(&mask_val);
	if (IS_ERR(th_zone->cool_dev[0])) {
		pr_err("Failed to register cpufreq cooling device\n");
		ret = -EINVAL;
		goto err_unregister;
	}
	th_zone->cool_dev_size++;

	th_zone->therm_dev = thermal_zone_device_register("thsens_cpu",
			PXA_ZONE_COUNT, 0, NULL, &pxa_dev_ops, NULL, 0, 2000);

	if (IS_ERR(th_zone->therm_dev)) {
		pr_err("Failed to register thermal zone device\n");
		ret = PTR_ERR(th_zone->therm_dev);
		goto err_unregister;
	}
	th_zone->mode = THERMAL_DEVICE_ENABLED;

	pr_info("Pxa: Kernel Thermal management registered\n");

	return 0;

err_unregister:
	pxa_unregister_thermal();
	return ret;
}

static void pxa_tmu_control(bool on)
{
	unsigned long ts_ctrl;
	struct pxa_tmu_data *data = pxa1088_thermal_data;

	/* start measure */
	ts_ctrl = readl(data->base + THERMAL_TS_CTRL);
	if (on) {
		/* start measure */
		ts_ctrl |= (TS_CTRL_RST_N_TSEN | TS_CTRL_TSEN_TEMP_ON);
	} else {
		ts_ctrl &= ~TS_CTRL_RST_N_TSEN;
		ts_ctrl &= ~TS_CTRL_TSEN_TEMP_ON;
	}
	writel(ts_ctrl, data->base + THERMAL_TS_CTRL);

}


#ifdef CONFIG_OF
static const struct of_device_id pxa_tmu_match[] = {
	{
		.compatible = "marvell,pxa1088-thermal", .data = NULL},
	{},
};
MODULE_DEVICE_TABLE(of, pxa_tmu_match);
#endif


static int pxa_tmu_probe(struct platform_device *pdev)
{
	struct pxa_tmu_data *data;
	struct pxa_tmu_platform_data *pdata;
	int ret, i;
	pdata = devm_kzalloc(&pdev->dev, sizeof(struct pxa_tmu_platform_data),
					GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "Failed to allocate pdata structure\n");
		return -ENOMEM;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(struct pxa_tmu_data),
				GFP_KERNEL);

	data->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, data->mem);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	data->clk = clk_get(NULL, "THERMALCLK");
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		return  PTR_ERR(data->clk);
	}

	data->pdata = pdata;
	pxa1088_thermal_data = data;
	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	clk_prepare_enable(data->clk);

	ret = pxa_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err_clk;
	}

	pxa_tmu_control(true);

	/* Register the sensor with thermal management interface */
	(&pxa_sensor_conf)->private_data = data;
	pxa_sensor_conf.trip_data.trip_count = 4;

	for (i = 0; i < pxa_sensor_conf.trip_data.trip_count; i++)
		pxa_sensor_conf.trip_data.trip_val[i] =
			cpu_thermal_trips_temp[i];

	ret = pxa_register_thermal(&pxa_sensor_conf);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register thermal interface\n");
		goto err_clk;
	}

	return 0;

err_clk:
	platform_set_drvdata(pdev, NULL);
	clk_put(data->clk);
	return ret;
}

static int pxa_tmu_remove(struct platform_device *pdev)
{
	struct pxa_tmu_data *data = platform_get_drvdata(pdev);

	pxa_tmu_control(false);

	pxa_unregister_thermal();

	clk_put(data->clk);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


#ifdef CONFIG_PM_SLEEP
static int pxa_tmu_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pxa_tmu_data *data = platform_get_drvdata(pdev);

	pxa_tmu_control(false);
	clk_disable_unprepare(data->clk);
	return 0;
}

static int pxa_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pxa_tmu_data *data = platform_get_drvdata(pdev);

	clk_prepare_enable(data->clk);
	pxa_tmu_control(true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(pxa_tmu_pm,
			 pxa_tmu_suspend, pxa_tmu_resume);
#define PXA_TMU_PM	(&pxa_tmu_pm)
#else
#define PXA_TMU_PM	NULL
#endif

static struct platform_driver pxa_tmu_driver = {
	.driver = {
		.name   = "pxa1088-thermal",
		.pm     = PXA_TMU_PM,
		.of_match_table = of_match_ptr(pxa_tmu_match),
	},
	.probe = pxa_tmu_probe,
	.remove	= pxa_tmu_remove,
};

module_platform_driver(pxa_tmu_driver);

MODULE_DESCRIPTION("PXA TMU Driver");
MODULE_AUTHOR("Liang Chen <chl@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pxa-tmu");
