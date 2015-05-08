/*
 * V4L2 Driver for Marvell Mobile SoC PXA910/PXA688/PXA2128 CCIC
 * (CMOS Camera Interface Controller)
 *
 * This driver is based on soc_camera and videobuf2 framework,
 * but part of the low level register function is base on cafe-driver.c
 *
 * Copyright 2006-2011 One Laptop Per Child Association, Inc.
 * Copyright 2006-2011 Jonathan Corbet <corbet@lwn.net>
 *
 * Copyright (C) 2011-2012, Marvell International Ltd.
 *	Libin Yang <lbyang@marvell.com>
 *	Kassey Lee <ygli@marvell.com>
 *	Angela Wan <jwan@marvell.com>
 *	Albert Wang <twang13@marvell.com>
 *	Lei Wen <leiwen@marvell.com>
 *	Fangsuo Wu <fswu@marvell.com>
 *	Sarah Zhang <xiazh@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/videodev2.h>
#include <linux/pm_qos.h>
#include <mach/cputype.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-chip-ident.h>

#include <mach/regs-apmu.h>
#include <linux/platform_data/camera-mmp.h>

#include "mmp_camera.h"
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define MMP_CAM_DRV_NAME "mmp-camera"
#define CCIC_REGS "ccic-regs"
#define APMU_REGS "apmu-regs"

static const struct soc_mbus_pixelfmt ccic_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.name = "YUV422PACKED",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.name = "YVYU422PACKED",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc	= V4L2_PIX_FMT_VYUY,
		.name = "VYUY422PACKED",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc	= V4L2_PIX_FMT_YVYU,
		.name = "YVYU422PACKED",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.name = "YUV422PLANAR",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.name = "YUV420PLANAR",
		.bits_per_sample = 12,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_YVU420,
		.name = "YVU420PLANAR",
		.bits_per_sample = 12,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.name = "YCbCrSP",
		.bits_per_sample = 12,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.name = "YCrCbSP",
		.bits_per_sample = 12,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.name = "JPEG",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_VARIABLE,
		.order = SOC_MBUS_ORDER_LE,
	},
};

#ifdef CONFIG_CCICS_CLK_COUPLED
/*
 * CCIC2's clk depends on CCIC1
 * The following code is
 * the quirk for the HW limitation
 */
static DEFINE_SPINLOCK(ccic_spin_lock);
static int ccic1_cnt;
static inline void ccic1_clk_enable(struct mmp_camera_dev *pcdev,
					int clkctrl_val, int ctrl1_val)
{
	spin_lock(&ccic_spin_lock);

	if (pcdev->pdev->id == 0) {
		/* for ccic1, just increase the count */
		ccic1_cnt++;
		spin_unlock(&ccic_spin_lock);
		return;
	}

	if (ccic1_cnt) {
		/* ccic1 is working */
		ccic1_cnt++;
	} else {
		ccic1_cnt++;
		writel(clkctrl_val, CCIC1_REG(REG_CLKCTRL));
		writel(ctrl1_val, CCIC1_REG(REG_CTRL1));
	}
	spin_unlock(&ccic_spin_lock);
	return;
}

static inline void ccic1_clk_disable(void)
{
	spin_lock(&ccic_spin_lock);
	ccic1_cnt--;
	if (ccic1_cnt < 0)
		ccic1_cnt = 0;	/* cnt error? */

	if (ccic1_cnt == 0) {
		writel(0x0, CCIC1_REG(REG_CLKCTRL));
		writel(C1_RESERVED, CCIC1_REG(REG_CTRL1));
	}
	spin_unlock(&ccic_spin_lock);
	return;
}
#endif

static void __attribute__((unused)) dump_register(struct mmp_camera_dev *pcdev)
{
	unsigned int ret;

	/*
	 * CCIC IRQ REG
	 */
	ret = ccic_reg_read(pcdev, REG_IRQSTAT);
	pr_info("CCIC: REG_IRQSTAT is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_IRQSTATRAW);
	pr_info("CCIC: REG_IRQSTATRAW is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_IRQMASK);
	pr_info("CCIC: REG_IRQMASK is 0x%08x\n\n", ret);

	/*
	 * CCIC IMG REG
	 */
	ret = ccic_reg_read(pcdev, REG_IMGPITCH);
	pr_info("CCIC: REG_IMGPITCH is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_IMGSIZE);
	pr_info("CCIC: REG_IMGSIZE is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_IMGOFFSET);
	pr_info("CCIC: REG_IMGOFFSET is 0x%08x\n\n", ret);

	/*
	 * CCIC CTRL REG
	 */
	ret = ccic_reg_read(pcdev, REG_CTRL0);
	pr_info("CCIC: REG_CTRL0 is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_CTRL1);
	pr_info("CCIC: REG_CTRL1 is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_CLKCTRL);
	pr_info("CCIC: REG_CLKCTRL is 0x%08x\n\n", ret);

	/*
	 * CCIC CSI2 REG
	 */
	ret = ccic_reg_read(pcdev, REG_CSI2_DPHY3);
	pr_info("CCIC: REG_CSI2_DPHY3 is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_CSI2_DPHY5);
	pr_info("CCIC: REG_CSI2_DPHY5 is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_CSI2_DPHY6);
	pr_info("CCIC: REG_CSI2_DPHY6 is 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_CSI2_CTRL0);
	pr_info("CCIC: REG_CSI2_CTRL0 is 0x%08x\n\n", ret);

	/*
	 * CCIC YUV REG
	 */
	ret = ccic_reg_read(pcdev, REG_Y0BAR);
	pr_info("CCIC: REG_Y0BAR 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_U0BAR);
	pr_info("CCIC: REG_U0BAR 0x%08x\n", ret);
	ret = ccic_reg_read(pcdev, REG_V0BAR);
	pr_info("CCIC: REG_V0BAR 0x%08x\n\n", ret);

	/*
	 * CCIC APMU REG
	 */
	if (pcdev->pdev->id == 0) {
		/*
		 * CCIC1 APMU REG
		 */
		ret = readl(pcdev->apmu_base + REG_APMU_CCIC_GATE);
		pr_info("CCIC: APMU_CCIC_GATE 0x%08x\n", ret);
		ret = readl(pcdev->apmu_base + REG_APMU_CCIC_RST);
		pr_info("CCIC: APMU_CCIC_RST 0x%08x\n", ret);
	} else if (pcdev->pdev->id == 1) {
		/*
		 * CCIC2 APMU REG
		 */
		ret = readl(pcdev->apmu_base + REG_APMU_CCIC2_GATE);
		pr_info("CCIC: APMU_CCIC2_GATE 0x%08x\n", ret);
		ret = readl(pcdev->apmu_base + REG_APMU_CCIC2_RST);
		pr_info("CCIC: APMU_CCIC2_RST 0x%08x\n", ret);
	}
	ret = readl(pcdev->apmu_base + REG_APMU_CCIC_DBG);
	pr_info("CCIC: APMU_DEBUG 0x%08x\n\n", ret);
}

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
static void ccic_ctlr_reset(struct mmp_camera_dev *pcdev);
static void ccic_stop(struct mmp_camera_dev *pcdev);
static void ccic_start(struct mmp_camera_dev *pcdev);

static void csi_dphy_write(void *hw_ctx, const struct csi_dphy_reg *regs)
{
	struct mmp_camera_dev *pcdev = hw_ctx;
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;


	if (sdata->bus_type == V4L2_MBUS_CSI2) {
		u32 regval = 0;

		/* Disable CCIC */
		ccic_stop(pcdev);

		/* reset MIPI */
		ccic_ctlr_reset(pcdev);

		regval = regs->hs_settle & 0xFF;
		regval = regs->hs_termen | (regval << 8);
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, regval);

		regval = regs->cl_settle & 0xFF;
		regval = regs->cl_termen | (regval << 8);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, regval);

		regval = (1 << regs->lane) - 1;
		regval = regval | (regval << 4);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, regval);

		regval = (regs->lane - 1) & 0x03;
		regval = (regval << 1) | 0x41;
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, regval);

		/* Enable CCIC */
		ccic_start(pcdev);
	} else {
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);
		sdata->mipi_enabled = 0;
	}
};

static void csi_dphy_read(void *hw_ctx, struct csi_dphy_reg *regs)
{
	struct mmp_camera_dev *pcdev = hw_ctx;
	u32 phy3, phy5, phy6;

	phy3 = ccic_reg_read(pcdev, REG_CSI2_DPHY3);
	phy5 = ccic_reg_read(pcdev, REG_CSI2_DPHY5);
	phy6 = ccic_reg_read(pcdev, REG_CSI2_DPHY6);

	regs->cl_termen	= phy6 & 0xFF;
	regs->cl_settle	= (phy6>>8) & 0xFF;
	regs->cl_miss	= 0;
	regs->hs_termen = phy3 & 0xFF;
	regs->hs_settle	= (phy3>>8) & 0xFF;
	regs->hs_rx_to	= 0xFFFF;
	regs->lane	= 0;
	phy5		&= 0xF;
	while (phy5) {
		phy5 = phy5 & (phy5-1);
		regs->lane++;
	}
	return;
};
#endif

static int ccic_config_phy(struct mmp_camera_dev *pcdev, int enable)
{
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	u32 val;
	struct device *dev = &pcdev->pdev->dev;
	int ret = 0;

	val = readl(pcdev->apmu_base + REG_APMU_CCIC_DBG);
	if (sdata->bus_type == V4L2_MBUS_CSI2 && enable) {
		dev_dbg(dev, "camera: DPHY3=0x%x, DPHY5=0x%x, DPHY6=0x%x\n",
			sdata->dphy[0], sdata->dphy[1], sdata->dphy[2]);
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, sdata->dphy[0]);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, sdata->dphy[2]);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, sdata->dphy[1]);
		__raw_writel(0x06000000 | val,
					 pcdev->apmu_base + REG_APMU_CCIC_DBG);
		if (sdata->mipi_enabled == 0) {
			/*
			 * 0x41 actives 1 lane
			 * 0x43 actives 2 lanes
			 * 0x47 actives 4 lanes
			 * There is no 3 lanes case
			 */
			if (sdata->bus_flag == V4L2_MBUS_CSI2_1_LANE)
				ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x41);
			else if (sdata->bus_flag == V4L2_MBUS_CSI2_2_LANE)
				ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x43);
			else if (sdata->bus_flag == V4L2_MBUS_CSI2_4_LANE)
				ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x47);
			else {
				dev_err(dev,
				"camera: board config wrong lane number!");
				return -EINVAL;
			}
			sdata->mipi_enabled = 1;
		}
	} else {
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);
		__raw_writel((~0x06000000) & val,
					 pcdev->apmu_base + REG_APMU_CCIC_DBG);
		sdata->mipi_enabled = 0;
	}

	return ret;
}

static void ccic_enable_pmu_clks(struct mmp_camera_dev *pcdev)
{
#if defined(CONFIG_CPU_PXA988)
	clk_prepare_enable(pcdev->axi_clk);
	clk_set_rate(pcdev->fn_clk, 312000000);
	clk_prepare_enable(pcdev->fn_clk);
	clk_prepare_enable(pcdev->phy_clk);
#elif defined(CONFIG_CPU_EDEN)
	clk_prepare_enable(pcdev->axi_clk);
	clk_set_rate(pcdev->fn_clk, 312000000);
	clk_prepare_enable(pcdev->fn_clk);
	clk_set_rate(pcdev->phy_clk, 1000000);
	clk_prepare_enable(pcdev->phy_clk);
#endif
}

static void ccic_disable_pmu_clks(struct mmp_camera_dev *pcdev)
{
	clk_disable_unprepare(pcdev->axi_clk);
	clk_disable_unprepare(pcdev->fn_clk);
	clk_disable_unprepare(pcdev->phy_clk);
}

void ccic_enable_clk(struct mmp_camera_dev *pcdev)
{
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	int ctrl1 = 0;
	int mipi,ret = 0;

	/*here config mclk pin mfpr to AF1 for mclk function,which is enabled just after camera power on*/
	ret = pinctrl_select_state(pcdev->pinctrl, pcdev->pin_mclk);
               if (ret) {
                       pr_err( "could not set mclk pin\n");
                       return -1;
               }


	if (sdata->bus_type == V4L2_MBUS_CSI2)
		mipi = MIPI_ENABLE;
	else
		mipi = MIPI_DISABLE;

	ccic_enable_pmu_clks(pcdev);

	ccic_reg_write(pcdev, REG_CLKCTRL,
			(pcdev->mclk_src << 29) | pcdev->mclk_div);

	switch (pcdev->dma_burst) {
	case 128:
		ctrl1 = C1_DMAB128;
		break;
	case 256:
		ctrl1 = C1_DMAB256;
		break;
	default:
		ctrl1 = C1_DMAB64;
		break;
	}
	ccic_reg_write(pcdev, REG_CTRL1, ctrl1 | C1_RESERVED | C1_DMAPOSTED);
#ifdef CONFIG_CCICS_CLK_COUPLED
	ccic1_clk_enable(pcdev, (pcdev->mclk_src << 29) | pcdev->mclk_div,
				ctrl1 | C1_RESERVED | C1_DMAPOSTED);
#endif
	if (sdata->bus_type != V4L2_MBUS_CSI2)
		ccic_reg_write(pcdev, REG_CTRL3, 0x4);
}

void ccic_disable_clk(struct mmp_camera_dev *pcdev)
{
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	int mipi, ret = 0;
       struct device_node *np = pcdev->pdev->dev.of_node;


#ifdef CONFIG_CCICS_CLK_COUPLED
	if (pcdev->pdev->id != 0) {
		ccic_reg_write(pcdev, REG_CLKCTRL, 0x0);
		ccic_reg_write(pcdev, REG_CTRL1, C1_RESERVED);
	}
	ccic1_clk_disable();
#else
	ccic_reg_write(pcdev, REG_CLKCTRL, 0x0);
	/*
	 * Bit[5:1] reserved and should not be changed
	 */
	ccic_reg_write(pcdev, REG_CTRL1, C1_RESERVED);
#endif

	if (sdata->bus_type == V4L2_MBUS_CSI2)
		mipi = MIPI_ENABLE;
	else
		mipi = MIPI_DISABLE;

	ccic_disable_pmu_clks(pcdev);
	
	/*here config mclk pin mfpr to AF0 for gpio function,which should be default/standby status*/
       ret = pinctrl_select_state(pcdev->pinctrl, pcdev->pin_gpio);
               if (ret) {
                       pr_err("could not set mclk pin to gpio\n");
                       return -1;
               }

	/*here control mclk to gpio output 0*/
       int mclk_pin = of_get_named_gpio(pcdev->pdev->dev.of_node, "mclk-gpio", 0);
	if (unlikely(mclk_pin < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n",
				__func__, mclk_pin);
		return -EINVAL;
	}
                       gpio_request(mclk_pin, NULL);
                       gpio_direction_output(mclk_pin,0);
                       gpio_free(mclk_pin);
}

static u32 ccic_yuvendfmt(u32 fourcc, enum v4l2_mbus_pixelcode code)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return C0_YUVE_YVYU;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return C0_YUVE_YUYV;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return C0_YUVE_VYUY;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return C0_YUVE_UYVY;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YUYV:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return C0_YUVE_YUYV;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return C0_YUVE_VYUY;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_VYUY:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return C0_YUVE_YVYU;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return C0_YUVE_YUYV;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_UYVY:
		switch (code) {
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return C0_YUVE_YVYU;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return C0_YUVE_YUYV;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YVYU:
		switch (code) {
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return C0_YUVE_YUYV;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return C0_YUVE_VYUY;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_NV12:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return C0_YUVE_YUYV;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return C0_YUVE_VYUY;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_NV21:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return C0_YUVE_VYUY;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return C0_YUVE_YUYV;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YUV422P:
		switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return C0_YUVE_YVYU;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_JPEG:
		switch (code) {
		case V4L2_MBUS_FMT_JPEG_1X8:
			return C0_YUVE_YUYV;
		default:
			return 0;
		}
	default:
		/* Other fmts to be added later */
		return 0;
	}
	return 0;
}

static int ccic_config_image(struct mmp_camera_dev *pcdev)
{
	struct v4l2_pix_format_mplane *fmt = &pcdev->mp;
	struct device *dev = &pcdev->pdev->dev;
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	u32 yuvendfmt;

	u32 widthy = 0, widthuv = 0, imgsz_h, imgsz_w;
	int ret = 0;

	yuvendfmt = ccic_yuvendfmt(fmt->pixelformat, icd->current_fmt->code);
	imgsz_h = (fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK;
	imgsz_w = (fmt->width * 2) & IMGSZ_H_MASK;

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		widthy = fmt->width * 2;
		widthuv = fmt->width * 2;
		break;
	case V4L2_PIX_FMT_RGB565:
		widthy = fmt->width * 2;
		widthuv = 0;
		break;
	case V4L2_PIX_FMT_JPEG:
		widthy = fmt->plane_fmt[0].bytesperline;
		widthuv = fmt->plane_fmt[0].bytesperline;
		break;
	case V4L2_PIX_FMT_YUV422P:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	default:
		break;
	}

	ccic_reg_write(pcdev, REG_IMGPITCH, widthuv << 16 | widthy);
	ccic_reg_write(pcdev, REG_IMGSIZE, imgsz_h | imgsz_w);
	ccic_reg_write(pcdev, REG_IMGOFFSET, 0x0);

	/*
	 * Tell the controller about the image format we are using.
	 */
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PLANAR | yuvendfmt, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_420PL | yuvendfmt, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PACKED | yuvendfmt, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_JPEG:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PACKED | yuvendfmt |
				C0_VEDGE_CTRL | C0_EOF_VSYNC, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_RGB444:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_RGB | C0_RGBF_444 | C0_RGB4_XRGB, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_RGB565:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_RGB | C0_RGBF_565 | C0_RGB5_BGGR, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PLANAR | yuvendfmt | C0_YUV420SP,
			C0_DF_MASK);
		break;
	default:
		dev_err(dev, "camera: unknown format: %c\n", fmt->pixelformat);
		break;
	}

	/*
	 * Make sure it knows we want to use hsync/vsync.
	 */
	//ccic_reg_write_mask(pcdev, REG_CTRL0, C0_SIF_HVSYNC, C0_SIFM_MASK);
	ccic_reg_write_mask(pcdev, REG_CTRL0, C0_VPOL_LOW | C0_EOF_VSYNC , C0_SYNC_MASK);
	/*
	 * This field controls the generation of EOF(DVP only)
	 */
	if (sdata->bus_type != V4L2_MBUS_CSI2)
		ccic_reg_set_bit(pcdev, REG_CTRL0,
				C0_EOF_VSYNC | C0_VEDGE_CTRL);

	if (sdata->bus_type == V4L2_MBUS_CSI2)
		ccic_reg_set_bit(pcdev, REG_CTRL2, ISIM_FIX);

	return ret;
}

static void ccic_frameirq_enable(struct mmp_camera_dev *pcdev)
{
	ccic_reg_write(pcdev, REG_IRQSTAT,
			IRQ_OVERFLOW | FRAMEIRQS_SOF | FRAMEIRQS_EOF);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, FRAMEIRQS_EOF);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, FRAMEIRQS_SOF);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, IRQ_OVERFLOW);
}

static void ccic_frameirq_disable(struct mmp_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, FRAMEIRQS_EOF);
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, FRAMEIRQS_SOF);
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, IRQ_OVERFLOW);
}

static void ccic_start(struct mmp_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

static void ccic_stop(struct mmp_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

static void ccic_stop_dma(struct mmp_camera_dev *pcdev)
{
	ccic_stop(pcdev);
	ccic_frameirq_disable(pcdev);
}

static void ccic_power_up(struct mmp_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void ccic_power_down(struct mmp_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

/*
 * Fetch buffer from list, if single mode, we reserve the last buffer
 * until new buffer is got, or fetch directly
 */
static void mmp_set_contig_buffer(struct mmp_camera_dev *pcdev,
				unsigned int frame)
{
	struct mmp_buffer *buf;
	struct v4l2_pix_format_mplane *fmt = &pcdev->mp;
	unsigned long flags = 0;
	struct device *dev = &pcdev->pdev->dev;

	spin_lock_irqsave(&pcdev->list_lock, flags);
	if (list_empty(&pcdev->buffers)) {
		/*
		 * If there are no available buffers
		 * go into single buffer mode
		 */
		dev_dbg(dev, "camera drop a frame!\n");
#if MAX_DMA_BUFS == 2
		/*
		 * CCIC use Two Buffers mode
		 * will use another remaining frame buffer
		 * frame 0 -> buf 1
		 * frame 1 -> buf 0
		 */
		buf = pcdev->vb_bufs[frame ^ 0x1];
		set_bit(CF_SINGLE_BUF, &pcdev->flags);
		pcdev->frame_state.singles++;
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
		PEG(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_DROP_FRAME, 1);
#endif
#elif MAX_DMA_BUFS == 3
		/*
		 * CCIC use Three Buffers mode
		 * will use the 2rd remaining frame buffer
		 * frame 0 -> buf 2
		 * frame 1 -> buf 0
		 * frame 2 -> buf 1
		 */
		buf = pcdev->vb_bufs[(frame + 0x2) % 0x3];
		if (pcdev->frame_state.tribufs == 0)
			pcdev->frame_state.tribufs++;
		else {
			set_bit(CF_SINGLE_BUF, &pcdev->flags);
			pcdev->frame_state.singles++;
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
		PEG(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_DROP_FRAME, 1);
#endif
			if (pcdev->frame_state.tribufs < 2)
				pcdev->frame_state.tribufs++;
		}
#endif
	} else {
		/*
		 * OK, we have a buffer we can use.
		 */
		buf = list_first_entry(&pcdev->buffers, struct mmp_buffer,
					queue);
		list_del_init(&buf->queue);
		clear_bit(CF_SINGLE_BUF, &pcdev->flags);
#if MAX_DMA_BUFS == 3
		if (pcdev->frame_state.tribufs != 0)
			pcdev->frame_state.tribufs--;
#endif
	}

	pcdev->vb_bufs[frame] = buf;
	ccic_reg_write(pcdev, REG_Y0BAR + (frame << 2), buf->yuv_p.y);
	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P
			|| fmt->pixelformat == V4L2_PIX_FMT_YUV420
			|| fmt->pixelformat == V4L2_PIX_FMT_YVU420) {
		ccic_reg_write(pcdev, REG_U0BAR + (frame << 2), buf->yuv_p.u);
		ccic_reg_write(pcdev, REG_V0BAR + (frame << 2), buf->yuv_p.v);
	} else if (fmt->pixelformat == V4L2_PIX_FMT_NV12 ||
			   fmt->pixelformat == V4L2_PIX_FMT_NV21) {
		ccic_reg_write(pcdev, REG_U0BAR + (frame << 2), buf->yuv_p.u);
	}
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

static void mmp_dma_setup(struct mmp_camera_dev *pcdev)
{
	unsigned int frame;

	pcdev->nbufs = MAX_DMA_BUFS;
	for (frame = 0; frame < pcdev->nbufs; frame++)
		mmp_set_contig_buffer(pcdev, frame);

#if MAX_DMA_BUFS == 2
	/*
	 * CCIC use Two Buffers mode
	 */
	ccic_reg_set_bit(pcdev, REG_CTRL1, C1_TWOBUFS);
#endif
}

static void ccic_ctlr_reset(struct mmp_camera_dev *pcdev)
{
	unsigned long val;

	/*
	 * Used CCIC2
	 * Recommend to do full reset when stream off by DE.
	 */
	if (pcdev->pdev->id) {
		val = readl(pcdev->apmu_base + REG_APMU_CCIC2_RST);
		writel(val & ~0x103, pcdev->apmu_base + REG_APMU_CCIC2_RST);
		usleep_range(100, 1000);
		writel(val | 0x103, pcdev->apmu_base + REG_APMU_CCIC2_RST);
	}

	val = readl(pcdev->apmu_base + REG_APMU_CCIC_RST);
	writel(val & ~0x103, pcdev->apmu_base + REG_APMU_CCIC_RST);
	usleep_range(100, 1000);
	writel(val | 0x103, pcdev->apmu_base + REG_APMU_CCIC_RST);
}

/*
 * Get everything ready, and start grabbing frames.
 */
static int mmp_read_setup(struct mmp_camera_dev *pcdev)
{
	int ret = 0;

	ret = ccic_config_phy(pcdev, 1);
	if (ret < 0)
		return ret;

	ccic_frameirq_enable(pcdev);
	mmp_dma_setup(pcdev);
	ccic_start(pcdev);
	pcdev->state = S_STREAMING;

	return ret;
}

static int mmp_videobuf_setup(struct vb2_queue *vq,
			const struct v4l2_format *fmt,
			u32 *count, u32 *num_planes,
			unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);
	int i;

	int minbufs = 2;
	if (*count < minbufs)
		*count = minbufs;

	if (bytes_per_line < 0)
		return bytes_per_line;

	if (pcdev->mp.num_planes == 0) {
		dev_err(&pcdev->pdev->dev, "num_planes is 0 when reqbuf\n");
		return -EINVAL;
	}

	*num_planes = pcdev->mp.num_planes;
	for (i = 0; i < pcdev->mp.num_planes; i++) {
		sizes[i] = pcdev->mp.plane_fmt[i].sizeimage;
		alloc_ctxs[i] = pcdev->vb_alloc_ctx;
	}

	return 0;
}

static int mmp_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct mmp_buffer *buf = container_of(vb, struct mmp_buffer, vb_buf);
	unsigned long size;
	unsigned long flags = 0;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);

	if (bytes_per_line < 0)
		return bytes_per_line;

	dev_dbg(&pcdev->pdev->dev, "%s; (vb = 0x%p), 0x%p, %lu\n", __func__,
		vb, vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));
	spin_lock_irqsave(&pcdev->list_lock, flags);
	/*
	 * Added list head initialization on alloc
	 */
	WARN(!list_empty(&buf->queue), "Buffer %p on queue!\n", vb);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
	BUG_ON(NULL == icd->current_fmt);
	size = vb2_plane_size(vb, 0);
	vb2_set_plane_payload(vb, 0, size);

	return 0;
}

static void mmp_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct mmp_buffer *buf = container_of(vb, struct mmp_buffer, vb_buf);
	unsigned long flags = 0;
	int start;

	mutex_lock(&pcdev->s_mutex);
	spin_lock_irqsave(&pcdev->list_lock, flags);
	/*
	 * Wait until two buffers already queued to the list
	 * then start DMA
	 */
	start = (pcdev->state == S_BUFWAIT) && !list_empty(&pcdev->buffers);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);

	switch (pcdev->mp.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
		buf->yuv_p.y = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->yuv_p.y += buf->vb_buf.v4l2_planes[0].data_offset;
		buf->yuv_p.u = vb2_dma_contig_plane_dma_addr(vb, 1);
		buf->yuv_p.u += buf->vb_buf.v4l2_planes[1].data_offset;
		buf->yuv_p.v = vb2_dma_contig_plane_dma_addr(vb, 2);
		buf->yuv_p.v += buf->vb_buf.v4l2_planes[2].data_offset;
		break;
	case V4L2_PIX_FMT_YVU420:
		buf->yuv_p.y = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->yuv_p.y += buf->vb_buf.v4l2_planes[0].data_offset;
		buf->yuv_p.v = vb2_dma_contig_plane_dma_addr(vb, 1);
		buf->yuv_p.v += buf->vb_buf.v4l2_planes[1].data_offset;
		buf->yuv_p.u = vb2_dma_contig_plane_dma_addr(vb, 2);
		buf->yuv_p.u += buf->vb_buf.v4l2_planes[2].data_offset;
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		buf->yuv_p.y = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->yuv_p.y += buf->vb_buf.v4l2_planes[0].data_offset;
		buf->yuv_p.u = vb2_dma_contig_plane_dma_addr(vb, 1);
		buf->yuv_p.u += buf->vb_buf.v4l2_planes[1].data_offset;
		buf->yuv_p.v = buf->yuv_p.u;
		break;
	default:
		buf->yuv_p.y = vb2_dma_contig_plane_dma_addr(vb, 0);
		buf->yuv_p.y += buf->vb_buf.v4l2_planes[0].data_offset;
		break;
	}

	spin_lock_irqsave(&pcdev->list_lock, flags);
	list_add_tail(&buf->queue, &pcdev->buffers);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);

	if (start)
		mmp_read_setup(pcdev);
	mutex_unlock(&pcdev->s_mutex);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_QBUF, 1);
#endif
}

static void mmp_videobuf_cleanup(struct vb2_buffer *vb)
{
	struct mmp_buffer *buf = container_of(vb, struct mmp_buffer, vb_buf);
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	unsigned long flags = 0;

	spin_lock_irqsave(&pcdev->list_lock, flags);
	/*
	 * queue list must be initialized before del
	 */
	if (buf->list_init_flag)
		list_del_init(&buf->queue);
	buf->list_init_flag = 0;
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

/*
 * only the list that queued could be initialized
 */
static int mmp_videobuf_init(struct vb2_buffer *vb)
{
	struct mmp_buffer *buf = container_of(vb, struct mmp_buffer, vb_buf);
	INIT_LIST_HEAD(&buf->queue);
	buf->list_init_flag = 1;

	return 0;
}

static int mmp_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct soc_camera_device *icd = container_of(vq,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	unsigned long flags = 0;
	unsigned int frame;
	int ret = 0;

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_STREAM, 1);
	CLEAR(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_QBUF);
	CLEAR(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DQBUF);
	SET(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DUMP,
		GET(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DUMP));
	CLEAR(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_SOF);
	CLEAR(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_EOF);
	CLEAR(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_OVERFLOW);
	CLEAR(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_DROP_FRAME);
#endif
	mutex_lock(&pcdev->s_mutex);
	if (count < 2) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (pcdev->state != S_IDLE) {
		ret = -EINVAL;
		goto out_unlock;
	}

	/*
	 * Videobuf2 sneakily hoards all the buffers and won't
	 * give them to us until *after* streaming starts.  But
	 * we can't actually start streaming until we have a
	 * destination.  So go into a wait state and hope they
	 * give us buffers soon.
	 */
	spin_lock_irqsave(&pcdev->list_lock, flags);
	if (list_empty(&pcdev->buffers)) {
		pcdev->state = S_BUFWAIT;
		spin_unlock_irqrestore(&pcdev->list_lock, flags);
		ret = 0;
		goto out_unlock;
	}
	spin_unlock_irqrestore(&pcdev->list_lock, flags);

	/*
	 * Ensure clear the obsolete frame flags
	 * before every really start streaming
	 */
	for (frame = 0; frame < pcdev->nbufs; frame++)
		clear_bit(CF_FRAME_SOF0 + frame, &pcdev->flags);

#if MAX_DMA_BUFS == 3
	pcdev->frame_state.tribufs = 0;
#endif

	ret = mmp_read_setup(pcdev);
out_unlock:
	mutex_unlock(&pcdev->s_mutex);

	return ret;
}

static int mmp_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = container_of(vq,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	unsigned long flags = 0;
	int ret = 0;

	mutex_lock(&pcdev->s_mutex);
	if (pcdev->state == S_BUFWAIT) {
		/* They never gave us buffers */
		pcdev->state = S_IDLE;
		goto out_unlock;
	}

	if (pcdev->state != S_STREAMING) {
		ret = -EINVAL;
		goto out_unlock;
	}

	ccic_stop_dma(pcdev);
	pcdev->state = S_IDLE;
	ccic_ctlr_reset(pcdev);

	spin_lock_irqsave(&pcdev->list_lock, flags);
	INIT_LIST_HEAD(&pcdev->buffers);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_STREAM, -1);
#endif
out_unlock:
	mutex_unlock(&pcdev->s_mutex);

	return ret;
}

static struct vb2_ops mmp_videobuf_ops = {
	.queue_setup		= mmp_videobuf_setup,
	.buf_prepare		= mmp_videobuf_prepare,
	.buf_queue		= mmp_videobuf_queue,
	.buf_cleanup		= mmp_videobuf_cleanup,
	.buf_init		= mmp_videobuf_init,
	.start_streaming	= mmp_start_streaming,
	.stop_streaming		= mmp_stop_streaming,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};

static int mmp_camera_init_videobuf(struct vb2_queue *q,
			struct soc_camera_device *icd)
{
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = icd;
	q->ops = &mmp_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mmp_buffer);
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(q);
}

/*
 * Hand a completed buffer back to user space.
 */
static void mmp_buffer_done(struct mmp_camera_dev *pcdev, unsigned int frame,
				struct vb2_buffer *vbuf)
{
	int i;
	for (i = 0; i < pcdev->mp.num_planes; i++)
		vb2_set_plane_payload(vbuf, i,
				pcdev->mp.plane_fmt[i].sizeimage);
	vb2_buffer_done(vbuf, VB2_BUF_STATE_DONE);
}

/*
 * Interrupt handler stuff
 */
static inline void mmp_frame_complete(struct mmp_camera_dev *pcdev,
				unsigned int frame)
{
	struct mmp_buffer *buf;
	unsigned long flags = 0;

	pcdev->frame_state.frames++;
	/*
	 * "This should never happen"
	 */
	if (pcdev->state != S_STREAMING)
		return;

	spin_lock_irqsave(&pcdev->list_lock, flags);
	buf = pcdev->vb_bufs[frame];
	if (!test_bit(CF_SINGLE_BUF, &pcdev->flags)) {
		pcdev->frame_state.delivered++;
		mmp_buffer_done(pcdev, frame, &buf->vb_buf);
	}
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
	mmp_set_contig_buffer(pcdev, frame);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
		PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DQBUF, 1);
		if (GET(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DUMP) > 0) {
			vb_dump_nonblock(&buf->vb_buf, "/data/dump.yuv");
			PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_DUMP, -1);
		}
#endif
}

static irqreturn_t mmp_camera_frameirq(int irq, void *data)
{
	struct mmp_camera_dev *pcdev = data;
	struct vb2_buffer *vbuf;
	u32 irqs, frame;
	struct device *dev = &pcdev->pdev->dev;

	irqs = ccic_reg_read(pcdev, REG_IRQSTAT);

	if (!(irqs & ALLIRQS))
		return IRQ_NONE;

	if (irqs & IRQ_OVERFLOW) {
		set_bit(CF_FRMAE_OVERFLOW, &pcdev->flags);
		dev_err(dev, "Overflow irq occurs!\n");
	}

	ccic_reg_write(pcdev, REG_IRQSTAT, irqs);

	/*
	 * Use the first loop handle the EOFx irq is more safety
	 * in potential EOFx and SOFx irqs co-exist case
	 * we may receive the EOFx of the last time and SOFx of this time
	 * during switch formats or resolutions
	 * if we can ensure this case never occur,
	 * then we can merge these 2 loops into 1 loop
	 */
	for (frame = 0; frame < pcdev->nbufs; frame++)
		if (irqs & (IRQ_EOF0 << frame) &&
			test_bit(CF_FRAME_SOF0 + frame, &pcdev->flags)) {
			if (!test_bit(CF_FRMAE_OVERFLOW, &pcdev->flags))
				mmp_frame_complete(pcdev, frame);
			else
				clear_bit(CF_FRMAE_OVERFLOW, &pcdev->flags);
			ccic_reg_read(pcdev, REG_CCIC_FRAME_BYTE_CNT);
			clear_bit(CF_FRAME_SOF0 + frame, &pcdev->flags);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
			PEG(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_EOF, 1);
#endif
		}

	for (frame = 0; frame < pcdev->nbufs; frame++)
		if (irqs & (IRQ_SOF0 << frame)) {
			set_bit(CF_FRAME_SOF0 + frame, &pcdev->flags);
			vbuf = &(pcdev->vb_bufs[frame]->vb_buf);
			do_gettimeofday(&vbuf->v4l2_buf.timestamp);
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
		PEG(pcdev->mcd_root.mcd, MCD_DMA, MCD_DMA_SOF, 1);
#endif
		}

	return IRQ_HANDLED;
}

static irqreturn_t mmp_camera_frameirq(int irq, void *data);

static int mmp_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct device *dev = &pcdev->pdev->dev;
	int ret = 0;

	if (pcdev->icd)
		return -EBUSY;

	pcdev->frame_state.frames = 0;
	pcdev->frame_state.singles = 0;
	pcdev->frame_state.delivered = 0;

	pcdev->qos_idle.name = pcdev->pdev->name;
	pm_qos_update_request(&pcdev->qos_idle, pcdev->qos_val);

#ifdef CONFIG_DDR_DEVFREQ

		pcdev->ddrfreq_qos = 2;
		pcdev->ddrfreq_qos_req_min.name = "camera";
		if(&pcdev->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&pcdev->ddrfreq_qos_req_min,
				pcdev->ddrfreq_qos);
#endif

	pcdev->icd = icd;
	icd->vb2_vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	pcdev->state = S_IDLE;
	soc_camera_power_on(&pcdev->pdev->dev, ssdd);
//	ccic_enable_clk(pcdev);
	ccic_power_up(pcdev);
	ccic_stop(pcdev);

	/*
	 * Need sleep 1ms-2ms to wait for CCIC stable
	 * This is a workround for OV5640 MIPI
	 * TODO: Fix me in the future
	 */
	usleep_range(1000, 2000);

	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(pcdev, REG_IRQMASK, 0);
	ret = v4l2_subdev_call(sd, core, init, 0);
	/*
	 * When v4l2_subdev_call return -ENOIOCTLCMD,
	 * means No ioctl command
	 */
	if ((ret < 0) && (ret != -ENOIOCTLCMD) && (ret != -ENODEV)) {
		dev_info(icd->parent,
			"camera: Failed to initialize subdev: %d\n", ret);
		goto err1;
	}

	ret = devm_request_irq(&pcdev->pdev->dev, pcdev->irq,
				mmp_camera_frameirq,
				IRQF_SHARED, MMP_CAM_DRV_NAME, pcdev);
	if (ret) {
		dev_err(&pcdev->pdev->dev, "camera: IRQ request failed\n");
		goto err2;
	}

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_ACT, 1);
	pcdev->mcd_sensor = default_mcd_sensor;
	strcpy(pcdev->mcd_sensor.entity.name, icd->link->module_name);
	pcdev->mcd_sensor.entity.priv = sd;
	ret = mcd_entity_init(&pcdev->mcd_sensor.entity, &pcdev->mcd_root.mcd);
	if (ret < 0)
		goto err2;
	else
		pcdev->mcd_root.pitem[MCD_SENSOR] = &pcdev->mcd_ccic.entity;
	pr_info("cam: mount node debugfs/%s/%s\n",
		pcdev->mcd_root.mcd.name, pcdev->mcd_sensor.entity.name);

	pcdev->mcd_ccic = default_mcd_dma;
	strcpy(pcdev->mcd_ccic.entity.name, "ccic");
	ret = mcd_entity_init(&pcdev->mcd_ccic.entity, &pcdev->mcd_root.mcd);
	if (ret < 0)
		goto err2;
	else
		pcdev->mcd_root.pitem[MCD_DMA] = &pcdev->mcd_ccic.entity;
	pr_info("cam: mount node debugfs/%s/%s\n",
		pcdev->mcd_root.mcd.name, pcdev->mcd_ccic.entity.name);

	/* CSI attached, now add debug interface for it*/
	pcdev->mcd_dphy = default_mcd_dphy;
	strcpy(pcdev->mcd_dphy.entity.name, "dphy");
	pcdev->mcd_dphy_hw.hw_ctx = pcdev;
	pcdev->mcd_dphy_hw.reg_write = &csi_dphy_write;
	pcdev->mcd_dphy_hw.reg_read = &csi_dphy_read;
	pcdev->mcd_dphy.entity.priv = &pcdev->mcd_dphy_hw;

	ret = mcd_entity_init(&pcdev->mcd_dphy.entity, &pcdev->mcd_root.mcd);
	if (ret < 0)
		goto err2;
	pcdev->mcd_root.pitem[MCD_DPHY] = &pcdev->mcd_dphy.entity;
	pr_info("cam: mount node debugfs/%s/%s\n",
			pcdev->mcd_root.mcd.name, pcdev->mcd_dphy.entity.name);

#endif
	return 0;
err2:
	devm_free_irq(dev, pcdev->irq, pcdev);
err1:
	ccic_config_phy(pcdev, 0);
	ccic_power_down(pcdev);
//	ccic_disable_clk(pcdev);
	soc_camera_power_off(dev, ssdd);
	pcdev->mp.num_planes = 0;
	pcdev->icd = NULL;		
	return ret;
}

static void mmp_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->pdev->dev;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	int i;

	BUG_ON(icd != pcdev->icd);

	dev_err(dev, "Release %d frames, %d singles, %d delivered\n",
		pcdev->frame_state.frames, pcdev->frame_state.singles,
		pcdev->frame_state.delivered);
	ccic_config_phy(pcdev, 0);
	ccic_power_down(pcdev);
//	ccic_disable_clk(pcdev);
	soc_camera_power_off(dev, ssdd);
	pcdev->mp.num_planes = 0;
	devm_free_irq(dev, pcdev->irq, pcdev);
	pcdev->icd = NULL;

	for (i = 0; i  < pcdev->mbus_fmt_num; i++)
		pcdev->mbus_fmt_code[i] = 0;
	pcdev->mbus_fmt_num = 0;

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	pr_info("cam: dismount node debugfs/%s/%s\n",
		pcdev->mcd_root.mcd.name, pcdev->mcd_dphy.entity.name);
	mcd_entity_remove(&pcdev->mcd_dphy.entity);
	pr_info("cam: dismount node debugfs/%s/%s\n",
		pcdev->mcd_root.mcd.name, pcdev->mcd_ccic.entity.name);
	mcd_entity_remove(&pcdev->mcd_ccic.entity);
	pr_info("cam: dismount node debugfs/%s/%s\n",
		pcdev->mcd_root.mcd.name, pcdev->mcd_sensor.entity.name);
	mcd_entity_remove(&pcdev->mcd_sensor.entity);
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_ACT, -1);
#endif
	pm_qos_update_request(&pcdev->qos_idle,
					PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
#ifdef CONFIG_DDR_DEVFREQ

		if(pcdev->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&pcdev->ddrfreq_qos_req_min,
				PM_QOS_DEFAULT_VALUE);
#endif

}

static int mmp_camera_set_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	struct mmp_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->pdev->dev;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_config cfg;
	int ret = 0;

	ret = v4l2_subdev_call(sd, video, g_mbus_config, &cfg);
	if ((ret < 0) && (ret != -ENOIOCTLCMD) && (ret != -ENODEV)) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	if (cfg.type == V4L2_MBUS_CSI2)
		sdata->bus_type = V4L2_MBUS_CSI2;
	else
		sdata->bus_type = V4L2_MBUS_PARALLEL;

	ret = v4l2_subdev_call(sd, video, s_mbus_config, &cfg);
	if ((ret < 0) && (ret != -ENOIOCTLCMD) && (ret != -ENODEV)) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	return 0;
}

static int mmp_get_num_planes(const struct soc_mbus_pixelfmt *host_fmt)
{
	switch (host_fmt->fourcc) {
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		return 3;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return 2;
	default:
		return 1;
	}
	return 1;	/* should never come here */
}

static void mmp_setup_mp_pixfmt(struct device *dev,
				struct v4l2_pix_format_mplane *pix_mp,
				const struct soc_mbus_pixelfmt *host_fmt)
{
	int i;
	int bpl, lpp;

	switch (host_fmt->fourcc) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		bpl = pix_mp->width * 2;
		lpp = pix_mp->height;
		if (pix_mp->plane_fmt[0].bytesperline < bpl)
			pix_mp->plane_fmt[0].bytesperline = bpl;
		pix_mp->plane_fmt[0].sizeimage = bpl * lpp;
		break;
	case V4L2_PIX_FMT_YUV422P:
		bpl = pix_mp->width;
		lpp = pix_mp->height;
		if (pix_mp->plane_fmt[0].bytesperline < bpl)
			pix_mp->plane_fmt[0].bytesperline = bpl;
		pix_mp->plane_fmt[0].sizeimage = bpl * lpp;
		bpl = pix_mp->width / 2;
		lpp = pix_mp->height;
		for (i = 1; i < 3; i++) {
			if (pix_mp->plane_fmt[i].bytesperline < bpl)
				pix_mp->plane_fmt[i].bytesperline = bpl;
			pix_mp->plane_fmt[i].sizeimage = bpl * lpp;
		}
		break;
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YVU420:
		bpl = pix_mp->width;
		lpp = pix_mp->height;
		if (pix_mp->plane_fmt[0].bytesperline < bpl)
			pix_mp->plane_fmt[0].bytesperline = bpl;
		pix_mp->plane_fmt[0].sizeimage = bpl * lpp;
		bpl = pix_mp->width / 2;
		lpp = pix_mp->height / 2;
		for (i = 1; i < 3; i++) {
			if (pix_mp->plane_fmt[i].bytesperline < bpl)
				pix_mp->plane_fmt[i].bytesperline = bpl;
			pix_mp->plane_fmt[i].sizeimage = bpl * lpp;
		}
		break;
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		bpl = pix_mp->width;
		lpp = pix_mp->height;
		if (pix_mp->plane_fmt[0].bytesperline < bpl)
			pix_mp->plane_fmt[0].bytesperline = bpl;
		pix_mp->plane_fmt[0].sizeimage = bpl * lpp;
		bpl = pix_mp->width / 2;
		lpp = pix_mp->height;
		if (pix_mp->plane_fmt[1].bytesperline < bpl)
			pix_mp->plane_fmt[1].bytesperline = bpl;
		pix_mp->plane_fmt[1].sizeimage = bpl * lpp;
		break;
	default:
		dev_warn(dev, "camera: use userspace assigned sizeimage\n");
		/* Use the assigned value from userspace.
		 * Manually add new fmts if needed.
		 */
		break;
	}
}

static int mmp_camera_set_fmt(struct soc_camera_device *icd,
			struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_subdev_desc *ssdd = &sdesc->subdev_desc;
	struct sensor_board_data *sdata = ssdd->drv_priv;
	struct mmp_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->pdev->dev;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_mbus_framefmt mf;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	int ret = 0;

	dev_dbg(dev, "camera: set_fmt: %c, width = %u, height = %u\n",
			pix_mp->pixelformat, pix_mp->width, pix_mp->height);
	xlate = soc_camera_xlate_by_fourcc(icd, pix_mp->pixelformat);
	if (!xlate) {
		dev_err(dev, "camera: format: %c not found\n",
				pix_mp->pixelformat);
		return -EINVAL;
	}

	mf.width = pix_mp->width;
	mf.height = pix_mp->height;
	mf.field = V4L2_FIELD_NONE;
	mf.colorspace = pix_mp->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0) {
		dev_err(dev, "camera: set_fmt failed %d\n", __LINE__);
		return ret;
	}

	if (mf.code != xlate->code) {
		dev_err(dev, "camera: wrong code %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	dev_dbg(dev, "camera: DPHY sets: dphy3=0x%x, dphy5=0x%x, dphy6=0x%x\n",
			sdata->dphy[0], sdata->dphy[1], sdata->dphy[2]);

	pix_mp->width = mf.width;
	pix_mp->height = mf.height;
	pix_mp->field = mf.field;
	pix_mp->colorspace = mf.colorspace;
	pix_mp->num_planes = mmp_get_num_planes(xlate->host_fmt);
	mmp_setup_mp_pixfmt(dev, pix_mp, xlate->host_fmt);
	icd->current_fmt = xlate;

	memcpy(&(pcdev->mp), pix_mp, sizeof(struct v4l2_pix_format_mplane));
	ret = ccic_config_image(pcdev);

	return ret;
}

static int mmp_camera_try_fmt(struct soc_camera_device *icd,
			struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->pdev->dev;
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix_mp->pixelformat;
	int ret = 0;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_err(dev, "camera: format: %c not found\n",
				pix_mp->pixelformat);
		return -EINVAL;
	}

	/*
	 * limit to sensor capabilities
	 */
	mf.width = pix_mp->width;
	mf.height = pix_mp->height;
	mf.field = V4L2_FIELD_NONE;
	mf.colorspace = pix_mp->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix_mp->width = mf.width;
	pix_mp->height = mf.height;
	pix_mp->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix_mp->field = V4L2_FIELD_NONE;
		break;
	default:
		dev_err(dev, "camera: Field type %d unsupported.\n", mf.field);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static unsigned int mmp_camera_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int mmp_camera_querycap(struct soc_camera_host *ici,
			struct v4l2_capability *cap)
{
	struct mmp_camera_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_desc *sdesc = to_soc_camera_desc(icd);
	struct soc_camera_host_desc *shd = &sdesc->host_desc;

	cap->version = KERNEL_VERSION(0, 0, 5);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING;

	strcpy(cap->card, shd->module_name);
	strcpy(cap->driver, shd->module_name);

	return 0;
}

static int mmp_camera_set_parm(struct soc_camera_device *icd,
			struct v4l2_streamparm *para)
{
	return 0;
}

/*
 * CCIC supports convertion between some formats
 * This function ranks the match between fourcc and mbus pixelcode
 * The highest score is 4, the lowest is 0
 * 4 means matching best while 0 means not match.
 */
static int mmp_mbus_fmt_score(u32 fourcc, enum v4l2_mbus_pixelcode code)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_YUV422P:
		switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 4;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_UYVY:
		switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 4;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 3;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_NV12:
		switch (code) {
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 4;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 3;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YUYV:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 4;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 3;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_VYUY:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 0;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 3;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 4;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_NV21:
		switch (code) {
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 3;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 0;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 4;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YVU420:
		switch (code) {
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 4;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 3;
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 2;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 1;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_YVYU:
		switch (code) {
		case V4L2_MBUS_FMT_YVYU8_2X8:
			return 4;
		case V4L2_MBUS_FMT_VYUY8_2X8:
			return 0;
		case V4L2_MBUS_FMT_YUYV8_2X8:
			return 3;
		case V4L2_MBUS_FMT_UYVY8_2X8:
			return 0;
		default:
			return 0;
		}
	case V4L2_PIX_FMT_JPEG:
		switch (code) {
		case V4L2_MBUS_FMT_JPEG_1X8:
			return 4;
		default:
			return 0;
		}
	default:
		/* Other fmts to be added later */
		return 0;
	}
	return 0;
}

static int mmp_fmt_match_best(struct mmp_camera_dev *pcdev,
			const struct soc_mbus_pixelfmt *ccic_format,
			enum v4l2_mbus_pixelcode code)
{
	int i, score = 0, hscore = 0, tmp;

	for (i = 0; i < pcdev->mbus_fmt_num; i++) {
		tmp = mmp_mbus_fmt_score(ccic_format->fourcc,
				pcdev->mbus_fmt_code[i]);
		if (hscore < tmp)
			hscore = tmp;
	}
	score = mmp_mbus_fmt_score(ccic_format->fourcc, code);
	if (score >= hscore && score > 0)
		return 1;
	else
		return 0;
}

static int mmp_camera_get_formats(struct soc_camera_device *icd, u32 idx,
			struct soc_camera_format_xlate  *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct mmp_camera_dev *pcdev = ici->priv;
	enum v4l2_mbus_pixelcode code;
	int formats = 0, ret = 0, i;

	if (pcdev->mbus_fmt_num == 0) {
		i = 0;
		while (!v4l2_subdev_call(sd, video, enum_mbus_fmt, i, &code))
			pcdev->mbus_fmt_code[i++] = code;
		pcdev->mbus_fmt_num = i;
	}

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		 return 0;   /*  No more formats */

	for (i = 0; i < ARRAY_SIZE(ccic_formats); i++) {
		ret = mmp_fmt_match_best(pcdev, &ccic_formats[i], code);
		if (ret == 0)
			continue;
		formats++;
		if (xlate) {
			xlate->host_fmt = &ccic_formats[i];
			xlate->code = code;
			xlate++;
		}
	}

	return formats;
}

static struct soc_camera_host_ops mmp_soc_camera_host_ops = {
	.owner		= THIS_MODULE,
	.add		= mmp_camera_add_device,
	.remove		= mmp_camera_remove_device,
	.set_fmt	= mmp_camera_set_fmt,
	.try_fmt	= mmp_camera_try_fmt,
	.set_parm	= mmp_camera_set_parm,
	.init_videobuf2	= mmp_camera_init_videobuf,
	.poll		= mmp_camera_poll,
	.querycap	= mmp_camera_querycap,
	.set_bus_param	= mmp_camera_set_bus_param,
	.get_formats	= mmp_camera_get_formats,
};

static int init_clk(struct mmp_camera_dev *pcdev)
{
#if defined(CONFIG_CPU_PXA988)
	pcdev->axi_clk = devm_clk_get(&pcdev->pdev->dev, "CCICAXICLK");
	if (IS_ERR(pcdev->axi_clk))
		return PTR_ERR(pcdev->axi_clk);

	if (pcdev->pdev->id == 0) {
		pcdev->fn_clk = devm_clk_get(&pcdev->pdev->dev,
				"CCICFUNCLK_0");
		if (IS_ERR(pcdev->fn_clk))
			return PTR_ERR(pcdev->fn_clk);
	} else {
		pcdev->fn_clk = devm_clk_get(&pcdev->pdev->dev, "CCICFUNCLK");
		if (IS_ERR(pcdev->fn_clk))
			return PTR_ERR(pcdev->fn_clk);
	}

	pcdev->phy_clk = devm_clk_get(&pcdev->pdev->dev, "CCICPHYCLK");
	if (IS_ERR(pcdev->phy_clk))
		return PTR_ERR(pcdev->phy_clk);
#elif defined(CONFIG_CPU_EDEN)
	pcdev->axi_clk = devm_clk_get(&pcdev->pdev->dev, "CCICAXICLK");
	if (IS_ERR(pcdev->axi_clk))
		return PTR_ERR(pcdev->axi_clk);

	pcdev->fn_clk = devm_clk_get(&pcdev->pdev->dev, "CCICFUNCLK");
	if (IS_ERR(pcdev->fn_clk))
		return PTR_ERR(pcdev->fn_clk);

	pcdev->phy_clk = devm_clk_get(&pcdev->pdev->dev, "CCICPHYCLK");
	if (IS_ERR(pcdev->phy_clk))
		return PTR_ERR(pcdev->phy_clk);
#endif

	return 0;
}

static int mmp_camera_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mmp_camera_dev *pcdev;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err;
	int len,ret = 0;
	const u32 *tmp;
	u32 lpm;

	err = of_alias_get_id(np, "mmp-camera");
	if (err < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", err);
		return err;
	}
	pdev->id = err;

	dev_info(&pdev->dev, "camera: probing CCIC%d\n", pdev->id + 1);

	pcdev = devm_kzalloc(&pdev->dev, sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "camera: Could not allocate pcdev\n");
		return -ENOMEM;
	}

	pcdev->pdev = pdev;

	INIT_LIST_HEAD(&pcdev->buffers);

	spin_lock_init(&pcdev->list_lock);
	mutex_init(&pcdev->s_mutex);

	pcdev->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pcdev->pinctrl)) {
		ret = PTR_ERR(pcdev->pinctrl);
		goto err_pinctrl;
	}

	pcdev->pin_mclk = pinctrl_lookup_state(pcdev->pinctrl, "enable");
	if (IS_ERR(pcdev->pin_mclk)) {
		pr_err( "could not get mclk pinstate\n");
		ret = IS_ERR(pcdev->pin_mclk);
		goto err_pinctrl;
	}

	pcdev->pin_gpio = pinctrl_lookup_state(pcdev->pinctrl, "default");
	if (IS_ERR(pcdev->pin_gpio)) {
		pr_err( "could not get default(gpio) pinstate\n");
		ret = IS_ERR(pcdev->pin_gpio);
		goto err_pinctrl;
	}
	/*
	 * Request the regions and ioremap
	 */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, CCIC_REGS);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev,
			"camera: Failed to request and remap CCIC io memory\n");
		return PTR_ERR(base);
	}

	pcdev->res = res;
	pcdev->base = base;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, APMU_REGS);
	base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		dev_err(&pdev->dev,
			"camera: Failed to remap APMU io memory\n");
		return PTR_ERR(base);
	}
	pcdev->apmu_res = res;
	pcdev->apmu_base = base;

	tmp = of_get_property(np, "dma-burst", &len);
	if (!tmp)
		pcdev->dma_burst = 64;	/* set the default value */
	else
		pcdev->dma_burst = be32_to_cpup(tmp);

	tmp = of_get_property(np, "mclk-src", &len);
	if (!tmp)
		return -EINVAL;
	else
		pcdev->mclk_src = be32_to_cpup(tmp);
	tmp = of_get_property(np, "mclk-div", &len);
	if (!tmp)
		return -EINVAL;
	else
		pcdev->mclk_div = be32_to_cpup(tmp);

	if (!of_property_read_u32(np, "lpm-qos", &lpm))
		pcdev->qos_val = lpm;

	pm_qos_add_request(&pcdev->qos_idle, PM_QOS_CPUIDLE_BLOCK,
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

#ifdef CONFIG_DDR_DEVFREQ

		//pcdev->ddrfreq_qos = 2;
		//pcdev->ddrfreq_qos_req_min.name = "camera";
		pm_qos_add_request(&pcdev->ddrfreq_qos_req_min,
					PM_QOS_DDR_DEVFREQ_MIN,
					PM_QOS_DEFAULT_VALUE);
		pr_debug("panel %s has ddrfreq min request: %u\n", pcdev->ddrfreq_qos_req_min.name, pcdev->ddrfreq_qos);
              
#endif

	err = init_clk(pcdev);
	if (err) {
		dev_err(&pdev->dev, "Can't get the clks\n");
		return err;
	}

#ifdef CONFIG_CPU_EDEN
	err = of_property_read_string(np, "fn_parent",
				&pcdev->fn_parent);
	if (!pcdev->fn_parent)
		return -EINVAL;
#endif

#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	pcdev->mcd_dphy = default_mcd_dphy;
	sprintf(pcdev->mcd_root.mcd.name, "cam%d", pdev->id);
	pcdev->mcd_root.mcd.nr_entity = MCD_ENTITY_END;
	err = mcd_init(&pcdev->mcd_root.mcd);
	if (err < 0)
		return err;
	pr_info("cam: Marvell Camera Debug interface created in debugfs/%s\n",
			pcdev->mcd_root.mcd.name);

	pcdev->mcd_vdev = default_mcd_vdev;
	strcpy(pcdev->mcd_vdev.entity.name, "vdev");
	err = mcd_entity_init(&pcdev->mcd_vdev.entity, &pcdev->mcd_root.mcd);
	if (err < 0)
		return err;
	else
		pcdev->mcd_root.pitem[MCD_VDEV] = &pcdev->mcd_vdev.entity;
	pr_info("cam: mount node debugfs/%s/%s\n",
			pcdev->mcd_root.mcd.name, pcdev->mcd_vdev.entity.name);
#endif


	/*
	 * Request irq
	 */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "camera: Failed to get irq resource\n");
		return -ENXIO;
	}

	pcdev->irq = irq;

	pcdev->soc_host.drv_name = MMP_CAM_DRV_NAME;
	pcdev->soc_host.ops = &mmp_soc_camera_host_ops;
	pcdev->soc_host.priv = pcdev;
	pcdev->soc_host.v4l2_dev.dev = &pdev->dev;
	pcdev->soc_host.nr = pdev->id;
	pcdev->vb_alloc_ctx = (struct vb2_alloc_ctx *)
				vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(pcdev->vb_alloc_ctx)) {
		err = PTR_ERR(pcdev->vb_alloc_ctx);
		goto exit_clk;
	}

	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto exit_free_ctx;
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_REG, 1);
#endif

	return 0;

exit_free_ctx:
	vb2_dma_contig_cleanup_ctx(pcdev->vb_alloc_ctx);
exit_clk:
err_pinctrl:
	return err;
}

static int mmp_camera_remove(struct platform_device *pdev)
{

	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct mmp_camera_dev *pcdev = container_of(soc_host,
			struct mmp_camera_dev, soc_host);

	pm_qos_remove_request(&pcdev->qos_idle);

#ifdef CONFIG_DDR_DEVFREQ

	if(pcdev->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
		pm_qos_remove_request(&pcdev->ddrfreq_qos_req_min);
#endif

	soc_camera_host_unregister(soc_host);
	vb2_dma_contig_cleanup_ctx(pcdev->vb_alloc_ctx);
	pcdev->vb_alloc_ctx = NULL;
	dev_info(&pdev->dev, "camera: MMP Camera driver unloaded\n");
#ifdef CONFIG_VIDEO_MRVL_CAM_DEBUG
	PEG(pcdev->mcd_root.mcd, MCD_VDEV, MCD_VDEV_REG, -1);
#endif

	return 0;
}

static int mmp_camera_suspend(struct device *dev)
{
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct v4l2_subdev *sd = NULL;
	struct soc_camera_desc *sdesc;
	struct soc_camera_subdev_desc *ssdd;
	int ret = 0;

	if (icd == NULL || icd->use_count == 0)
		return 0;

	sdesc = to_soc_camera_desc(icd);
	ssdd = &sdesc->subdev_desc;

	dev_err(dev, "camera: someone is stil using ccic\n");

	mutex_lock(&pcdev->s_mutex);
	if (pcdev->state == S_STREAMING)
		ccic_stop_dma(pcdev);
	mutex_unlock(&pcdev->s_mutex);

	/*
	 * FIXME:
	 * Set sensor to hardware standby mode.The
	 * implementation is only valid for one
	 * sensor.For multi sensors. Need to power
	 * saving every active sensor.
	 */
	sd = soc_camera_to_subdev(icd);
	ret = v4l2_subdev_call(sd, core, s_power, 0);
	if (ret < 0) {
		dev_err(dev, "camera: enter sensor hardware standby failed\n");
		return ret;
	}

//	ccic_disable_clk(pcdev);
	ccic_power_down(pcdev);
	soc_camera_power_off(dev, ssdd);
	pm_qos_update_request(&pcdev->qos_idle,
				PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
#ifdef CONFIG_DDR_DEVFREQ
 
		if(pcdev->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&pcdev->ddrfreq_qos_req_min,
				PM_QOS_DEFAULT_VALUE);
#endif

	return ret;
}

static int mmp_camera_resume(struct device *dev)
{
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct mmp_camera_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct v4l2_subdev *sd = NULL;
	struct soc_camera_desc *sdesc;
	struct soc_camera_subdev_desc *ssdd;
	int ret = 0;

	if (icd == NULL || icd->use_count == 0)
		return 0;

	pm_qos_update_request(&pcdev->qos_idle, pcdev->qos_val);
#ifdef CONFIG_DDR_DEVFREQ

		if(pcdev->ddrfreq_qos != PM_QOS_DEFAULT_VALUE)
			pm_qos_update_request(&pcdev->ddrfreq_qos_req_min,
				pcdev->ddrfreq_qos);
#endif

	sdesc = to_soc_camera_desc(icd);
	ssdd = &sdesc->subdev_desc;

	soc_camera_power_on(dev, ssdd);
	ccic_power_up(pcdev);
//	ccic_enable_clk(pcdev);

	sd = soc_camera_to_subdev(icd);
	ret = v4l2_subdev_call(sd, core, s_power, 1);
	if (ret < 0) {
		dev_err(dev, "camera: exit sensor hardware standby failed\n");
		return ret;
	}

	mutex_lock(&pcdev->s_mutex);
	if (pcdev->state == S_STREAMING) {
		ccic_frameirq_enable(pcdev);
		ccic_start(pcdev);
	}
	mutex_unlock(&pcdev->s_mutex);

	return ret;
}

static const struct dev_pm_ops mmp_camera_pm = {
	.suspend = mmp_camera_suspend,
	.resume	= mmp_camera_resume,
};

static const struct of_device_id mv_ccic_dt_match[] = {
	{ .compatible = "marvell,mmp-ccic", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, mv_usbphy_dt_match);

static struct platform_driver mmp_camera_driver = {
	.driver = {
		.name = MMP_CAM_DRV_NAME,
		.pm = &mmp_camera_pm,
		.of_match_table = of_match_ptr(mv_ccic_dt_match),
	},
	.probe = mmp_camera_probe,
	.remove = mmp_camera_remove,
};

module_platform_driver(mmp_camera_driver);

MODULE_DESCRIPTION("Marvell MMP CMOS Camera Interface Controller driver");
MODULE_AUTHOR("Libin Yang <lbyang@marvell.com>");
MODULE_AUTHOR("Kassey Lee <ygli@marvell.com>");
MODULE_AUTHOR("Angela Wan <jwan@marvell.com>");
MODULE_AUTHOR("Albert Wang <twang13@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");
