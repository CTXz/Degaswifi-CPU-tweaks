/*
 * MFP Init configuration driver for MMP related platform.
 *
 * This driver is aims to do the initialization on the MFP configuration
 * which is not handled by the component driver.
 * The driver user is required to add your "mfp init deivce" in your used
 * DTS file, whose complatiable is "mrvl,mmp-mfp", in which you can implement
 * pinctrl setting.
 *
 * Copyright:   (C) 2013 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static int mfp_probe(struct platform_device *dev)
{
	pr_info("MFP Configuration is inited in MFP init driver\n");
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id mfp_dt_ids[] = {
	{ .compatible = "marvell,mmp-mfp-leftover", },
	{}
};
MODULE_DEVICE_TABLE(of, mfp_dt_ids);
#endif

static struct platform_driver mfp_driver = {
	.probe		= mfp_probe,
	.driver		= {
		.name	= "mmp-mfp-leftover",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mfp_dt_ids),
#endif
	},
};

int __init mfp_init(void)
{
	return platform_driver_register(&mfp_driver);
}

subsys_initcall(mfp_init);

MODULE_AUTHOR("Fan Wu<fwu@marvell.com>");
MODULE_DESCRIPTION("MFP Initialization Driver for MMP");
MODULE_LICENSE("GPL v2");
