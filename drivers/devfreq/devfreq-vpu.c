/*
 * devfreq-vpu: Generic Dynamic Voltage and Frequency Scaling (DVFS) Framework
 *		  for vpu Device.
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *	Xiaoguang Chen <chenxg@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/devfreq.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/clk-private.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define VPU_FREQ_MAX	8
#define KHZ_TO_HZ	1000

struct vpu_devfreq_data {
	struct devfreq *devfreq;
	struct clk *vclk;

	/* VPU frequency table used for platform */
	unsigned int vpu_freq_tbl[VPU_FREQ_MAX]; /* unit Khz */
	unsigned int vpu_freq_tbl_len;
};

static int vpu_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	struct vpu_devfreq_data *data = platform_get_drvdata(pdev);
	int ret = 0;

	ret = clk_set_rate(data->vclk, *freq * KHZ_TO_HZ);

	if (!ret)
		*freq = clk_get_rate(data->vclk) / KHZ_TO_HZ;

	return ret;
}

static struct devfreq_dev_profile vpu_devfreq_profile = {
	.target = vpu_target,
};

static int vpu_devfreq_probe(struct platform_device *pdev)
{
	struct vpu_devfreq_data *data = NULL;
	struct devfreq_platform_data *pdata;
	int err = 0;
	int i = 0;
	struct device *dev = &pdev->dev;
	pdata = (struct devfreq_platform_data *)dev->platform_data;
	if (!pdata) {
		dev_err(dev, "No platform data!\n");
		goto out;
	}

	data = kzalloc(sizeof(struct vpu_devfreq_data), GFP_KERNEL);

	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory for vpu devfreq!\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, data);
	data->vclk = clk_get(NULL, pdata->clk_name);
	if (IS_ERR(data->vclk)) {
		err = PTR_ERR(data->vclk);
		goto out;
	}

	/* save vpu frequency tbl */
	if (pdata->freq_table) {
		data->vpu_freq_tbl_len = pdata->freq_tbl_len;
		for (i = 0; i < data->vpu_freq_tbl_len; i++)
			data->vpu_freq_tbl[i] = pdata->freq_table[i];
	}

	vpu_devfreq_profile.initial_freq =
			clk_get_rate(data->vclk) / KHZ_TO_HZ;

	/* set the frequency table of devfreq profile */
	if (data->vpu_freq_tbl_len) {
		vpu_devfreq_profile.freq_table = data->vpu_freq_tbl;
		vpu_devfreq_profile.max_state = data->vpu_freq_tbl_len;
		for (i = 0; i < data->vpu_freq_tbl_len; i++)
			opp_add(dev, data->vpu_freq_tbl[i], 1000);
	}

	data->devfreq = devfreq_add_device(dev, &vpu_devfreq_profile,
					   "userspace", NULL);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		goto out;
	}

	/* init default devfreq min_freq and max_freq */
	if (pdata->freq_table) {
		data->devfreq->min_freq = data->vpu_freq_tbl[0];
		data->devfreq->max_freq =
			data->vpu_freq_tbl[data->vpu_freq_tbl_len - 1];
	}

	return 0;
out:
	kfree(data);
	return err;
}

static int vpu_devfreq_remove(struct platform_device *pdev)
{
	struct vpu_devfreq_data *data = platform_get_drvdata(pdev);
	devfreq_remove_device(data->devfreq);
	kfree(data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mmp_devfreq_vpu_dt_match[] = {
	{.compatible = "marvell,devfreq-vpu" },
	{},
};
#endif
static struct platform_driver vpu_devfreq_driver = {
	.probe = vpu_devfreq_probe,
	.remove = vpu_devfreq_remove,
	.driver = {
		   .name = "devfreq-vpu",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mmp_devfreq_vpu_dt_match),
		   },
};

static int __init vpu_devfreq_init(void)
{
	return platform_driver_register(&vpu_devfreq_driver);
}

static void __init vpu_devfreq_exit(void)
{
	platform_driver_unregister(&vpu_devfreq_driver);
}

module_init(vpu_devfreq_init);
module_exit(vpu_devfreq_exit);

MODULE_DESCRIPTION("vpu devfreq device driver");
MODULE_LICENSE("GPL");
