/*
 * fake frame-buffer driver, as a platform device
 *
 * Copyright (c) 2013, Zhou Zhu
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

static struct fb_fix_screeninfo fakefb_fix = {
	.id		= "fake",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo fakefb_var = {
	.height		= -1,
	.width		= -1,
	.activate	= FB_ACTIVATE_NOW,
	.vmode		= FB_VMODE_NONINTERLACED,
};

static int fakefb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			      u_int transp, struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= 16)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;
		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static struct fb_ops fakefb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= fakefb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

struct fakefb_format {
	const char *name;
	u32 bits_per_pixel;
	struct fb_bitfield red;
	struct fb_bitfield green;
	struct fb_bitfield blue;
	struct fb_bitfield transp;
};

static struct fakefb_format fakefb_formats[] = {
	{ "r5g6b5", 16, {11, 5}, {5, 6}, {0, 5}, {0, 0} },
};

struct fakefb_params {
	u32 width;
	u32 height;
	u32 stride;
	struct fakefb_format *format;
};

static int fakefb_parse_dt(struct platform_device *pdev,
			   struct fakefb_params *params)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	const char *format;
	int i;

	ret = of_property_read_u32(np, "width", &params->width);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "height", &params->height);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "stride", &params->stride);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse stride property\n");
		return ret;
	}

	ret = of_property_read_string(np, "format", &format);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse format property\n");
		return ret;
	}
	params->format = NULL;
	for (i = 0; i < ARRAY_SIZE(fakefb_formats); i++) {
		if (strcmp(format, fakefb_formats[i].name))
			continue;
		params->format = &fakefb_formats[i];
		break;
	}
	if (!params->format) {
		dev_err(&pdev->dev, "Invalid format value\n");
		return -EINVAL;
	}

	return 0;
}

static int fakefb_probe(struct platform_device *pdev)
{
	int ret;
	struct fakefb_params params;
	struct fb_info *info;

	if (fb_get_options("fakefb", NULL))
		return -ENODEV;

	ret = fakefb_parse_dt(pdev, &params);
	if (ret)
		return ret;

	info = framebuffer_alloc(sizeof(u32) * 16, &pdev->dev);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);

	info->fix = fakefb_fix;
	info->fix.line_length = params.stride;
	info->var = fakefb_var;
	info->var.xres = params.width;
	info->var.yres = params.height;
	info->var.xres_virtual = params.width;
	info->var.yres_virtual = params.height;
	info->var.bits_per_pixel = params.format->bits_per_pixel;
	info->var.red = params.format->red;
	info->var.green = params.format->green;
	info->var.blue = params.format->blue;
	info->var.transp = params.format->transp;

	info->fix.smem_len = info->fix.line_length * info->var.yres_virtual;
	info->screen_base = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(info->fix.smem_len),
				(u64 *)(&info->fix.smem_start), GFP_KERNEL);
	if (!info->screen_base) {
		dev_err(&pdev->dev, "can't alloc framebuffer\n");
		framebuffer_release(info);
		return -ENOMEM;
	}
	memset(info->screen_base, 0, info->fix.smem_len);
	dev_info(&pdev->dev, "fb %dk allocated\n", info->fix.smem_len >> 10);

	info->fbops = &fakefb_ops;
	info->flags = FBINFO_DEFAULT;
	info->pseudo_palette = (void *)(info + 1);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register fakefb: %d\n", ret);
		dma_free_coherent(&pdev->dev, PAGE_ALIGN(info->fix.smem_len),
			info->screen_base, info->fix.smem_start);
		framebuffer_release(info);
		return ret;
	}

	dev_info(&pdev->dev, "fb%d: fakefb registered!\n", info->node);

	return 0;
}

static int fakefb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	unregister_framebuffer(info);
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(info->fix.smem_len),
		info->screen_base, info->fix.smem_start);
	framebuffer_release(info);

	return 0;
}

static const struct of_device_id fakefb_of_match[] = {
	{ .compatible = "fake-framebuffer", },
	{ },
};
MODULE_DEVICE_TABLE(of, fakefb_of_match);

static struct platform_driver fakefb_driver = {
	.driver = {
		.name = "fake-framebuffer",
		.owner = THIS_MODULE,
		.of_match_table = fakefb_of_match,
	},
	.probe = fakefb_probe,
	.remove = fakefb_remove,
};
module_platform_driver(fakefb_driver);

MODULE_AUTHOR("Zhou Zhu <zzhu3@marvell.com>");
MODULE_DESCRIPTION("Fake framebuffer driver");
MODULE_LICENSE("GPL v2");
