/* arch/arm/mach-mmp/sec-pxa1x88-dt.c
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
 *
 * Based on linux/arch/arm/mach-mmp/mmpx-dt.c
 * Author: Shankar Bandal <shankar.b@samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation,
 * and may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/clocksource.h>
#include <linux/clk/mmp.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/usb/mv_usb2_phy.h>
#include <linux/platform_data/mv_usb.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/regs-addr.h>
#include <linux/features.h>
#include <asm/smp_twd.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/irqs.h>
#include <mach/regs-apbc.h>
#include <mach/regs-ciu.h>
#include <mach/addr-map.h>
#include <mach/regs-coresight.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/memblock.h>
#include <linux/irqchip/arm-gic.h>
#include <video/mmp_disp.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_SEC_DEBUG
#include <linux/sec-debug.h>
#endif
#ifdef CONFIG_GPU_RESERVE_MEM
#include <mach/gpu_mem.h>
#endif
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif
#ifdef CONFIG_MFD_88PM822
#include <linux/mfd/88pm822.h>
#endif

#include "common.h"
#include "reset.h"
#include <linux/regdump_ops.h>
#include "sec-pxa1x88.h"
#include <media/soc_camera.h>
#include <media/mrvl-camera.h>

#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif

#define MHZ_TO_KHZ	1000

static struct mv_usb_platform_data mmpx_usb_pdata = {
	.mode		= MV_USB_MODE_OTG,
	.extern_attr	= MV_USB_HAS_VBUS_DETECTION,
	.otg_force_a_bus_req            = 1,
	.disable_otg_clock_gating       = 1,
};

#ifdef CONFIG_SD8XXX_RFKILL
#if defined(CONFIG_MACH_DEGAS) || defined(CONFIG_MACH_AGERA)
void buck2_sleepmode_control_for_wifi(int on);
void ldo10_sleepmode_control_for_wifi(int on);
#endif

static void wireless_card_set_power(unsigned int on)
{
	pr_err(KERN_ERR "wireless_card_set_power, on : %d\n", on);
#if defined(CONFIG_MACH_DEGAS) || defined(CONFIG_MACH_AGERA)
	buck2_sleepmode_control_for_wifi(on);
	ldo10_sleepmode_control_for_wifi(on);
#endif
	return;
}

struct sd8x_rfkill_platform_data sd8x_rfkill_platdata = {
	.set_power	= wireless_card_set_power,
	};

#endif

#ifdef CONFIG_VPU_DEVFREQ
static struct devfreq_platform_data devfreq_vpu_pdata = {
	.clk_name = "VPUCLK",
};
static void __init pxa988_devfreq_vpu_init(void)
{
	unsigned int vpu_freq_num;
	unsigned int *vpu_freq_table;
	unsigned int i;

	struct clk *clk = clk_get_sys(NULL, "VPUCLK");
	if (IS_ERR(clk)) {
		WARN_ON(1);
		return;
	}
	vpu_freq_num =  __clk_periph_get_opnum(clk);

	vpu_freq_table = kzalloc(sizeof(unsigned int) * vpu_freq_num,
				 GFP_KERNEL);
	if (!vpu_freq_table)
		return;
	for (i = 0; i < vpu_freq_num; i++)
		vpu_freq_table[i] = __clk_periph_get_oprate(clk, i) / MHZ_TO_KHZ;
	devfreq_vpu_pdata.freq_tbl_len = vpu_freq_num;
	devfreq_vpu_pdata.freq_table = vpu_freq_table;
}

#endif

static struct regulator *vcamera_ldo4;		  /* SENSOR AVDD : 2.8V */
static struct regulator *vcamera_ldo9;		   /* SENSOR IO : 1.8V */
static struct regulator *vcamera_ldo12;   /* 3M CORE : 1.2V */
static struct platform_device *pdev;
struct device_node *np;
extern void ccic_enable_clk(struct mmp_camera_dev *pcdev);
extern void ccic_disable_clk(struct mmp_camera_dev *pcdev);
#ifdef CONFIG_ION_PXA
extern void __init mmp_reserve_ion(void);
#endif

#ifdef CONFIG_PSTORE_RAM
extern void __init mmp_reserve_ramoops(void);
#endif

#ifdef CONFIG_SOC_CAMERA_S5K4ECGX
int s5k4ecgx_sensor_power(struct device *dev, int flag)
{
	struct soc_camera_host *ici = to_soc_camera_host(dev);
       struct mmp_camera_dev *pcdev = ici->priv;
	static struct regulator *cam_core;   /* 5M CORE : 1.2V */
	static int gpio_status_s5k4ecgx;
	static int cam_io;
	static int cam_avdd;
	static int cam_af;
	static int cam_stby;
	static int cam_rst;
	int ret = 0;
	pdev =  container_of(dev , struct platform_device , dev);
	if (!pdev)
		return -EINVAL;
	np = pdev->dev.of_node;
	if (!cam_core) {
		cam_core = regulator_get(dev, "cam_core");
		if (IS_ERR(cam_core)) {
			cam_core = NULL;
			pr_err(KERN_ERR "Regulator cam_core get failed!\n");
			return -EIO;
		}
	}
	if (!gpio_status_s5k4ecgx) {
		cam_avdd = of_get_named_gpio(np, "cam_avdd", 0);
	    if (unlikely(cam_avdd < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_avdd);
		   ret = cam_avdd;
		   return ret;
	    }
		ret = gpio_request(cam_avdd, "cam_avdd");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		cam_io = of_get_named_gpio(np, "cam_io", 0);
	    if (unlikely(cam_io < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_io);
	       ret = cam_io;
	       return ret;
	    }
		ret = gpio_request(cam_io, "cam_io");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		cam_af = of_get_named_gpio(np, "cam_af", 0);
	    if (unlikely(cam_io < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_af);
	       ret = cam_af;
	       return ret;
	    }
		ret = gpio_request(cam_af, "cam_af");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		cam_stby = of_get_named_gpio(np, "cam_stby", 0);
	    if (unlikely(cam_stby < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_stby);
	       ret = cam_stby;
	       return ret;
	    }
		ret = gpio_request(cam_stby, "cam_stby");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		cam_rst = of_get_named_gpio(np, "cam_rst", 0);
	    if (unlikely(cam_io < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_rst);
	       ret = cam_rst;
	       return ret;
	    }
		ret = gpio_request(cam_rst, "cam_rst");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		gpio_status_s5k4ecgx = 1;
	}
	if (flag) {
		pr_info("---camera power ON : S5K4ECGX SGLTE ----------\n");
		/* 5M Core : 1.2V ON */
		regulator_set_voltage(cam_core, 1200000, 1200000);
		regulator_enable(cam_core);
		msleep(1);
		/* Sensor AVDD : 2.8V ON */
		gpio_direction_output(cam_avdd, 1);
		msleep(1);
		/* Sensor IO : 1.8V ON */
		gpio_direction_output(cam_io, 1);
		/* AF : 2.8V ON */
		gpio_direction_output(cam_af, 1);
		msleep(1);
		/* MCLK On */
		ccic_enable_clk(pcdev);
		/* 5M STBY Enable */
		gpio_direction_output(cam_stby, 1);
		/* 5M Reset Enable*/
		gpio_direction_output(cam_rst, 0);
		msleep(2);
		gpio_direction_output(cam_rst, 1);
	} else {
		pr_info("---camera power OFF S5K4ECGX SGLTE----------\n");
		/* 5M Reset Disable*/
		gpio_direction_output(cam_rst, 0);
		msleep(1);
		/* MCLK Off */
		ccic_disable_clk(pcdev);
		/* 5M STBY Disable */
		gpio_direction_output(cam_stby, 0);
		/* Sensor IO : 1.8V OFF */
		gpio_direction_output(cam_io, 0);
		/* Sensor AVDD : 2.8V OFF */
		gpio_direction_output(cam_avdd, 0);
		/*  5M Core : 1.2V OFF  */
		regulator_disable(cam_core);
		/* AF : 2.8V OFF */
		gpio_direction_output(cam_af, 0);
	}
	return 0;
}
static struct sensor_board_data s5k4ecgx_data = {
	.mount_pos		= SENSOR_USED | SENSOR_POS_BACK | SENSOR_RES_HIGH,
	.bus_type		= V4L2_MBUS_CSI2,
	.bus_flag		= V4L2_MBUS_CSI2_2_LANE, /* s5k4ecgx sensor uses 2 lanes */
	.dphy = {0x0a06, 0x33, 0x0900},
	.mipi_enabled		= 0,
	.dphy3_algo		= 0,
};
static struct i2c_board_info i2c_board_info_s5k4ecgx = {
	I2C_BOARD_INFO("s5k4ecgx", (0xac >> 1))
};
struct soc_camera_desc main_desc = {
	.subdev_desc = {
		.power = s5k4ecgx_sensor_power,
		.drv_priv		= &s5k4ecgx_data,
		.flags			= 0,
	},
	.host_desc = {
		.bus_id = 0,	/* Must match with the camera ID */
		.i2c_adapter_id = 0,
		.board_info = &i2c_board_info_s5k4ecgx,
		.module_name = "s5k4ecgx",
	},
};
#endif
#ifdef CONFIG_SOC_CAMERA_SR352
static int sr352_sensor_power(struct device *dev, int on)
{
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct mmp_camera_dev *pcdev = ici->priv;
	static int gpio_status_sr352;
	static int cam_stby;
	static int cam_rst;
	int ret = 0;
	pdev =  container_of(dev , struct platform_device , dev);
	if (!pdev)
		return -EINVAL;
	np = pdev->dev.of_node;
	if (!vcamera_ldo4) {
		vcamera_ldo4 = regulator_get(dev, "cam_avdd");
		if (IS_ERR(vcamera_ldo4)) {
			vcamera_ldo4 = NULL;
			pr_err(KERN_ERR "Enable vcamera_ldo4 failed!\n");
			return -EIO;
		}
	}
	if (!vcamera_ldo9) {
		vcamera_ldo9 = regulator_get(dev, "cam_io");
		if (IS_ERR(vcamera_ldo9)) {
			vcamera_ldo9 = NULL;
			pr_err(KERN_ERR "Enable vcamera_ldo9 failed!\n");
			return -EIO;
		}
	}
	if (!vcamera_ldo12) {
		vcamera_ldo12 = regulator_get(dev, "cam_core");
		if (IS_ERR(vcamera_ldo12)) {
			vcamera_ldo12 = NULL;
			pr_err(KERN_ERR "Enable vcamera_ldo12 failed!\n");
			return -EIO;
		}
	}
	if (!gpio_status_sr352) {
		cam_stby = of_get_named_gpio(np, "cam_stby", 0);
	    if (unlikely(cam_stby < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_stby);
		   ret = cam_stby;
	    }
		ret = gpio_request(cam_stby, "cam_stby");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		cam_rst = of_get_named_gpio(np, "cam_rst", 0);
	    if (unlikely(cam_rst < 0)) {
	       pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
														cam_stby);
	       ret = cam_rst;
	    }
		ret = gpio_request(cam_rst, "cam_rst");
	    if (unlikely(ret < 0)) {
	       pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	    }
		gpio_status_sr352 = 1;
	}
    if (on) {
	pr_info("---sr352 camera power ON ----------\n");
	/* Sensor IO & VT Core : 1.8V ON */
	regulator_set_voltage(vcamera_ldo9, 1800000, 1800000);
	regulator_enable(vcamera_ldo9);
	msleep(1);
	/* Sensor AVDD : 2.8V ON */
	regulator_set_voltage(vcamera_ldo4, 2800000, 2800000);
	regulator_enable(vcamera_ldo4);
	msleep(1);
	/* 3M Core : 1.2V ON */
	regulator_set_voltage(vcamera_ldo12, 1300000, 1300000);
	regulator_enable(vcamera_ldo12);
	msleep(5);
	/* MCLK On */
	ccic_enable_clk(pcdev);
	/* 3M STBY Enable */
	gpio_direction_output(cam_stby, 1);
	msleep(5);
	/* 3M Reset Enable*/
	gpio_direction_output(cam_rst, 0);
	msleep(10);
	gpio_direction_output(cam_rst, 1);
	msleep(5);
    } else {
	pr_info("---sr352 camera power OFF ----------\n");
	/* 3M Reset Disable*/
	gpio_direction_output(cam_rst, 0);
	msleep(1);
	/* MCLK Off */
	ccic_disable_clk(pcdev);
	msleep(1);
	/* 3M STBY Disable */
	gpio_direction_output(cam_stby, 0);
	msleep(1);
	/*  3M Core : 1.2V OFF  */
	regulator_disable(vcamera_ldo12);
	msleep(1);
	/* Sensor AVDD : 2.8V OFF */
	regulator_disable(vcamera_ldo4);
	msleep(1);
	/* Sensor IO & VT Core: 1.8V OFF */
	regulator_disable(vcamera_ldo9);
    }
    return ret;
}
static int soc_sensor_flash_led_set(void *control, bool op)
{
	int flash_on;
	int torch_on;
	int ret = 0;
	struct v4l2_ctrl *ctrl = (struct v4l2_ctrl *) control;
	flash_on = of_get_named_gpio(np, "cam_flash", 0);
	if (unlikely(flash_on < 0)) {
	   pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
												   flash_on);
	   ret = flash_on;
	}
	ret = gpio_request(flash_on, "cam_flash");
	if (unlikely(ret < 0)) {
	   pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	}
	torch_on = of_get_named_gpio(np, "cam_torch", 0);
	if (unlikely(torch_on < 0)) {
	   pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
												   torch_on);
	   ret = torch_on;
	}
	ret = gpio_request(torch_on, "cam_torch");
	if (unlikely(ret < 0)) {
	   pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	}
	switch (ctrl->id) {
	case V4L2_CID_FLASH_STROBE:
		gpio_direction_output(flash_on, 1);
		gpio_direction_output(torch_on, 1);
		break;
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		gpio_direction_output(torch_on, 1);
		gpio_direction_output(flash_on, 0);
		break;
	case V4L2_CID_FLASH_STROBE_STOP:
		gpio_direction_output(flash_on, 0);
		gpio_direction_output(torch_on, 0);
		break;
	default:
		break;
	}
	gpio_free(flash_on);
	gpio_free(torch_on);
	return ret;
}
static struct sensor_board_data sr352_data = {
	.mount_pos	= SENSOR_USED | SENSOR_POS_BACK | SENSOR_RES_HIGH,
	.bus_type	= V4L2_MBUS_CSI2,
	.bus_flag	= V4L2_MBUS_CSI2_1_LANE,
	.dphy = {0x0e07, 0x11, 0x0400},
	.v4l2_flash_if = 0,//soc_sensor_flash_led_set,
};
static struct i2c_board_info dkb_i2c_sr352 = {
		I2C_BOARD_INFO("sr352", (0x40 >> 1)),
};
static struct soc_camera_desc main_desc = {
	.subdev_desc = {
		.power          = sr352_sensor_power,
		.drv_priv		= &sr352_data,
		.flags		= 0,
	},
	.host_desc = {
		.bus_id = 0,	/* Must match with the camera ID */
		.i2c_adapter_id = 0,
		.board_info     = &dkb_i2c_sr352,
		.module_name    = "sr352",
	},
};
#endif
#ifdef CONFIG_SOC_CAMERA_SR130PC10
static int sr130pc10_sensor_power(struct device *dev, int on)
{
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct mmp_camera_dev *pcdev = ici->priv;
	static int gpio_status_sr130pc10;
	static int vt_cam_stby;
	static int vt_cam_rst;
	int ret = 0;
	if (!vcamera_ldo4) {
	   vcamera_ldo4 = regulator_get(dev, "cam_avdd");
	   if (IS_ERR(vcamera_ldo4)) {
	      vcamera_ldo4 = NULL;
		  pr_err(KERN_ERR "Enable vcamera_ldo4 failed!\n");
		  return -EIO;
	   }
	}
	if (!vcamera_ldo9) {
	   vcamera_ldo9 = regulator_get(dev, "cam_io");
	   if (IS_ERR(vcamera_ldo9)) {
		  vcamera_ldo9 = NULL;
		  pr_err(KERN_ERR "Enable vcamera_ldo9 failed!\n");
		  return -EIO;
	   }
	}
	if (!vcamera_ldo12) {
		vcamera_ldo12 = regulator_get(dev, "cam_core");
		if (IS_ERR(vcamera_ldo12)) {
			vcamera_ldo12 = NULL;
			pr_err(KERN_ERR "Enable vcamera_ldo12 failed!\n");
			return -EIO;
		}
	}
	if (!gpio_status_sr130pc10) {
	    vt_cam_stby = of_get_named_gpio(np, "vt_cam_stby", 0);
	if (unlikely(vt_cam_stby < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
												vt_cam_stby);
		ret = vt_cam_stby;
	}
	ret = gpio_request(vt_cam_stby, "vt_cam_stby");
	if (unlikely(ret < 0))
		pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	vt_cam_rst = of_get_named_gpio(np, "vt_cam_rst", 0);
	if (unlikely(vt_cam_rst < 0)) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
													vt_cam_rst);
		ret = vt_cam_rst;
	}
	ret = gpio_request(vt_cam_rst, "vt_cam_rst");
	if (unlikely(ret < 0))
	pr_err("%s: gpio_request failed: %d\n", __func__, ret);
	gpio_status_sr130pc10 = 1;
	}
	if (on) {
	/* Sensor IO & VT Core : 1.8V ON */
	regulator_set_voltage(vcamera_ldo9, 1800000, 1800000);
	regulator_enable(vcamera_ldo9);
	msleep(1);
	/* Sensor AVDD : 2.8V ON */
	regulator_set_voltage(vcamera_ldo4, 2800000, 2800000);
	regulator_enable(vcamera_ldo4);
	msleep(1);
	/* 3M Core : 1.2V ON */
	regulator_set_voltage(vcamera_ldo12, 1300000, 1300000);
	regulator_enable(vcamera_ldo12);
	msleep(5);
	/* MCLK On */
	ccic_enable_clk(pcdev);
	/* VT STBY Enable */
	gpio_direction_output(vt_cam_stby, 1);
	msleep(10);
	/* VT Rest Enable */
	gpio_direction_output(vt_cam_rst, 0);
	msleep(25);
	gpio_direction_output(vt_cam_rst, 1);
	msleep(5);
	pr_info("[%s:%d]---sr130pc10_power power ON end----------\n", __func__, __LINE__);

	} else {
	pr_info("---sr130pc10_power power OFF ----------\n");
	/* VT Rest Disable */
	gpio_direction_output(vt_cam_rst, 0);
	msleep(1);
	/* MCLK Off */
	ccic_disable_clk(pcdev);
	msleep(1);
	/* VT STBY Disable */
	gpio_direction_output(vt_cam_stby, 0);
	msleep(1);
	/*  3M Core : 1.2V OFF  */
	regulator_disable(vcamera_ldo12);
	msleep(1);
	/* Sensor AVDD : 2.8V OFF */
	regulator_disable(vcamera_ldo4);
	msleep(1);
	/* Sensor IO : 1.8V OFF */
	regulator_disable(vcamera_ldo9);
	}
	return ret;
}
static struct sensor_board_data sr130pc10_data = {
	.mount_pos	= SENSOR_USED | SENSOR_POS_FRONT | SENSOR_RES_LOW,
	.bus_type	= V4L2_MBUS_PARALLEL,
	.bus_flag	= 0, /* sr130pc10 used PARALLEL */
};
static struct i2c_board_info dkb_i2c_sr130pc10 = {
		I2C_BOARD_INFO("sr130pc10", (0x40 >> 1)),
};
static struct soc_camera_desc secondary_desc = {
	.subdev_desc = {
		.power          = sr130pc10_sensor_power,
		.drv_priv		= &sr130pc10_data,
		.flags		= 0,
	},
	.host_desc = {
		.bus_id = 0,	/* Must match with the camera ID */
		.i2c_adapter_id = 6,
		.board_info     = &dkb_i2c_sr130pc10,
		.module_name    = "sr130pc10",
	},
};
#endif
/* PXA988 */
static const struct of_dev_auxdata pxa988_auxdata_lookup[] __initconst  = {
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4036000, "pxa2xx-uart.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4017000, "pxa2xx-uart.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-uart", 0xd4018000, "pxa2xx-uart.2", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4011000, "pxa2xx-i2c.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4010800, "pxa2xx-i2c.1", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-twsi", 0xd4037000, "pxa2xx-i2c.2", NULL),
	OF_DEV_AUXDATA("marvell,mmp-gpio", 0xd4019000, "mmp-gpio", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-edge-wakeup", 0xd4019800, "mmp-edge-wakeup", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-rtc", 0xd4010000, "sa1100-rtc", NULL),
	OF_DEV_AUXDATA("marvell,pdma-1.0", 0xd4000000, "mmp-pdma", NULL),
	OF_DEV_AUXDATA("marvell,pxa27x-keypad", 0xd4012000, "pxa27x-keypad", NULL),
	OF_DEV_AUXDATA("marvell,usb2-phy-40lp", 0xd4207000,
			"pxa988-usb-phy", NULL),
	OF_DEV_AUXDATA("marvell,mv-udc", 0xd4208100, "mv-udc", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("marvell,pxa-u2oehci", 0xd4208100, "pxa-u2oehci", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("marvell,mv-otg", 0xd4208100, "mv-otg", &mmpx_usb_pdata),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xD128dc00, "mmp-sspa-dai.0", NULL),
	OF_DEV_AUXDATA("mrvl,mmp-sspa-dai", 0xD128dd00, "mmp-sspa-dai.1", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xD128D800, "mmp-adma.0", NULL),
	OF_DEV_AUXDATA("marvell,adma-1.0", 0xD128D900, "mmp-adma.1", NULL),
#ifdef CONFIG_DDR_DEVFREQ
	OF_DEV_AUXDATA("marvell,devfreq-ddr", 0xc0100000, "devfreq-ddr", NULL),
#endif

#ifdef CONFIG_VPU_DEVFREQ
	OF_DEV_AUXDATA("marvell,devfreq-vpu", 0xf0400000,
			"devfreq-vpu.0", (void *)&devfreq_vpu_pdata),
#endif
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280000, "sdhci-pxav3.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4280800, "sdhci-pxav3.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxav3-mmc", 0xd4281000, "sdhci-pxav3.2", NULL),
	OF_DEV_AUXDATA("samsung,sec-battery", 0, "sec-battery", NULL),


	OF_DEV_AUXDATA("marvell,mmp-disp", 0xd420b000, "mmp-disp", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb", 0, "mmp-fb", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 0, "mmp-fb-overlay.0", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 1, "mmp-fb-overlay.1", NULL),
	OF_DEV_AUXDATA("marvell,mmp-fb-overlay", 2, "mmp-fb-overlay.2", NULL),

	OF_DEV_AUXDATA("marvell,mmp-dsi", 0xd420b800, "mmp-dsi", NULL),
	OF_DEV_AUXDATA("marvell,pxa910-squ", 0xd42a0800, "pxa910-squ", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd401b000, "pxa988-ssp.0", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd42a0c00, "pxa988-ssp.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa910-ssp", 0xd4039000, "pxa988-ssp.4", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-ssp-dai", 1, "pxa-ssp-dai.1", NULL),
	OF_DEV_AUXDATA("mrvl,pxa-ssp-dai", 4, "pxa-ssp-dai.2", NULL),
	OF_DEV_AUXDATA("marvell,pxa-88pm805-snd-card", 0, "sound", NULL),
	OF_DEV_AUXDATA("marvell,pxa28nm-thermal", 0xd4013300,
			"pxa28nm-thermal", NULL),
	OF_DEV_AUXDATA("pxa-ion", 0, "pxa-ion", NULL),
#ifdef CONFIG_PXA_THERMAL
	OF_DEV_AUXDATA("marvell,pxa-thermal", 0xd4013200, "pxa-thermal", NULL),
#endif
#ifdef CONFIG_PXA1088_THERMAL
	OF_DEV_AUXDATA("marvell,pxa1088-thermal", 0xd4013200,
			"pxa1088-thermal", NULL),
#endif
#ifdef CONFIG_SEC_LOG
	OF_DEV_AUXDATA("sec-log", 0, "sec-log", NULL),
#endif
	OF_DEV_AUXDATA("gpio-keys", 0, "gpio-keys", NULL),
#ifdef CONFIG_SD8XXX_RFKILL
	OF_DEV_AUXDATA("mrvl,sd8x-rfkill", 0, "sd8x-rfkill",
			&sd8x_rfkill_platdata),
#endif
#if defined(CONFIG_SOC_CAMERA_S5K4ECGX) || defined(CONFIG_SOC_CAMERA_SR352)
	OF_DEV_AUXDATA("soc-camera-pdrv", 0, "soc-camera-pdrv.0", &main_desc),
#endif
#if defined(CONFIG_SOC_CAMERA_SR130PC10)
	OF_DEV_AUXDATA("soc-camera-pdrv", 1, "soc-camera-pdrv.1",
				&secondary_desc),
#endif
	OF_DEV_AUXDATA("marvell,mmp-ccic", 0xd420a000, "mmp-camera.0", NULL),
#ifdef CONFIG_MMP_GPS
	OF_DEV_AUXDATA("marvell,mmp-gps", 0, "mmp-gps", NULL),
#endif
#ifdef CONFIG_SENSORS_SEC_THERMISTOR
	OF_DEV_AUXDATA("samsung,sec-thermistor", 0, "sec-thermistor", NULL),
#endif
	{}
};

static void __init pxa988_dt_irq_init(void)
{
	irqchip_init();
	/* only for wake up */
	mmp_of_wakeup_init();
}

#define APMU_SDH0      0x54
static void __init pxa988_sdhc_reset_all(void)
{
	unsigned int reg_tmp;

	/* use bit0 to reset all 3 sdh controls */
	reg_tmp = __raw_readl(get_apmu_base_va() + APMU_SDH0);
	__raw_writel(reg_tmp & (~1), get_apmu_base_va() + APMU_SDH0);
	udelay(10);
	__raw_writel(reg_tmp | (1), get_apmu_base_va() + APMU_SDH0);
}

static void __init pxa988_dt_init_machine(void)
{
	if (of_machine_is_compatible("mrvl,pxa988")) {
		l2x0_of_init(0x30800000, 0xFE7FFFFF);
		l2x0_save_phys_reg_addr(&l2x0_regs_phys);
	}
#ifdef CONFIG_MACH_PXA_SAMSUNG
	sec_common_init();
#endif

	pxa988_clk_init();

#ifdef CONFIG_VPU_DEVFREQ
	pxa988_devfreq_vpu_init();
#endif

	pxa988_sdhc_reset_all();

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_add_gpu();
#endif

	if (of_machine_is_compatible("mrvl,pxa988-dkb")
	|| of_machine_is_compatible("mrvl,pxa1088-dkb")
	|| of_machine_is_compatible("mrvl,pxa1L88-dkb-v10")
	|| of_machine_is_compatible("mrvl,pxa1L88-dkb-v20")
	|| of_machine_is_compatible("mrvl,pxa1L88-sec")
	|| of_machine_is_compatible("mrvl,pxa1088-sec")) {
		pr_info("UDC only is enabled\n");
		mmpx_usb_pdata.mode = MV_USB_MODE_UDC;
		mmpx_usb_pdata.otg_force_a_bus_req = 0;
		mmpx_usb_pdata.disable_otg_clock_gating = 0;
	}

	of_platform_populate(NULL, of_default_bus_match_table,
			     pxa988_auxdata_lookup, &platform_bus);

	if (cpu_is_pxa1088() || cpu_is_pxa1L88()) {
		pxa_init_gic_regdump();
		pxa_init_pmua_regdump();
		pxa_init_pmua_regdump_1x88();
	}

}

static void __init pxa_enable_external_agent(void __iomem *addr)
{
	u32 tmp;

	tmp = readl_relaxed(addr);
	tmp |= 0x100000;
	writel_relaxed(tmp, addr);
}

static int __init pxa_external_agent_init(void)
{
	/* if enable TrustZone, move core config to TZSW. */
#ifndef CONFIG_TZ_HYPERVISOR
	if (has_feat_enable_cti()) {
		/* enable access CTI registers for core */
		pxa_enable_external_agent(CIU_CPU_CORE0_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE1_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE2_CONF);
		pxa_enable_external_agent(CIU_CPU_CORE3_CONF);
	}
#endif

	return 0;
}
core_initcall(pxa_external_agent_init);

static void __init mmp_cti_enable(u32 cpu)
{
	void __iomem *cti_base = CTI_CORE0_VIRT_BASE + 0x1000 * cpu;
	u32 tmp;

	/* Unlock CTI */
	writel_relaxed(0xC5ACCE55, cti_base + CTI_LOCK_OFFSET);

	/*
	 * Enables a cross trigger event to the corresponding channel.
	 */
	tmp = readl_relaxed(cti_base + CTI_EN_IN1_OFFSET);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1 << cpu;
	writel_relaxed(tmp, cti_base + CTI_EN_IN1_OFFSET);

	tmp = readl_relaxed(cti_base + CTI_EN_OUT6_OFFSET);
	tmp &= ~CTI_EN_MASK;
	tmp |= 0x1 << cpu;
	writel_relaxed(tmp, cti_base + CTI_EN_OUT6_OFFSET);

	/* Enable CTI */
	writel_relaxed(0x1, cti_base + CTI_CTRL_OFFSET);
}

static int __init mmp_cti_init(void)
{
	int cpu;
	if (!has_feat_enable_cti())
		return 1;

	for (cpu = 0; cpu < nr_cpu_ids; cpu++)
		mmp_cti_enable(cpu);
	return 0;
}
arch_initcall(mmp_cti_init);

void mmp_pmu_ack(void)
{
	writel_relaxed(0x40, CTI_REG(CTI_INTACK_OFFSET));
}
EXPORT_SYMBOL(mmp_pmu_ack);

#define MPMU_APRR		(0x1020)
#define MPMU_WDTPCR		(0x0200)
/* wdt and cp use the clock */
void enable_pxawdt_clock(void)
{
	void __iomem *mpmu_base;
	void __iomem *mpmu_wdtpcr;
	void __iomem *mpmu_aprr;
	mpmu_base = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	mpmu_aprr = mpmu_base + MPMU_APRR;
	mpmu_wdtpcr = mpmu_base + MPMU_WDTPCR;

	/* reset/enable WDT clock */
	writel(0x7, mpmu_wdtpcr);
	readl(mpmu_wdtpcr);
	writel(0x3, mpmu_wdtpcr);
	return;
}

#define GENERIC_COUNTER_VIRT_BASE       (APB_VIRT_BASE + 0x101000)
static __init void enable_arch_timer(void)
{
	uint32_t tmp;

	tmp = readl(APBC_COUNTER_CLK_SEL);

	/* Default is 26M/32768 = 0x319 */
	if ((tmp >> 16) != 0x319) {
		pr_warn("Generic Counter step of Low Freq is not right\n");
		return;
	}
	/* bit0 = 1: Generic Counter Frequency control by hardware VCTCXO_EN
	   VCTCXO_EN = 1, Generic Counter Frequency is 26Mhz;
	   VCTCXO_EN = 0, Generic Counter Frequency is 32KHz */
	writel(tmp | FREQ_HW_CTRL, APBC_COUNTER_CLK_SEL);

	/* NOTE: can not read CNTCR before write, otherwise write will fail
	   Halt on debug;
	   start the counter */
	writel(CNTCR_HDBG | CNTCR_EN, GENERIC_COUNTER_VIRT_BASE + CNTCR);
}

static __init void mmpx_timer_init(void)
{
	enable_pxawdt_clock();

#ifdef CONFIG_ARM_ARCH_TIMER
	enable_arch_timer();
#endif
	/* Select the configurable timer clock rate to be 3.25MHz */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMPX_TIMER0);
	__raw_writel(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3),
			APBC_MMPX_TIMER0);

	/* Select the configurable timer clock rate to be 3.25MHz */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMPX_TIMER1);
	__raw_writel(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3),
			APBC_MMPX_TIMER1);

	clocksource_of_init();
}

#ifdef CONFIG_OF
static u32 __initdata cp_base;
static u32 __initdata cp_size;

static int __init mmp_cp_fdt_find_info(unsigned long node, const char *uname,
		int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "marvell,cp-heap"))
		return 0;

	prop = of_get_flat_dt_prop(node, "cp-base", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find cp-base property\n", __func__);
		return 0;
	}
	cp_base = be32_to_cpu(prop[0]);

	prop = of_get_flat_dt_prop(node, "cp-size", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find cp-size property\n", __func__);
		return 0;
	}
	cp_size = be32_to_cpu(prop[0]);

	return 1;
}

static void __init mmp_reserve_cp(void)
{
	if (of_scan_flat_dt(mmp_cp_fdt_find_info, NULL)) {
		BUG_ON(memblock_reserve(cp_base, cp_size) != 0);
		memblock_free(cp_base, cp_size);
		memblock_remove(cp_base, cp_size);
		pr_info("Reserved CP memory : %dMB at %#.8x\n",
			(unsigned)cp_size/0x100000,
			(unsigned)cp_base);
	} else
		pr_info("Reserved CP memory dt prop fails : %dMB at %#.8x\n",
			(unsigned)cp_size/0x100000,
			(unsigned)cp_base);
}
#endif

/* For HELANLTE CP memeory reservation, 32MB by default */
static u32 cp_area_size = 0x02000000;
static u32 cp_area_addr = 0x06000000;

static int __init early_cpmem(char *p)
{
	char *endp;

	cp_area_size = memparse(p, &endp);
	if (*endp == '@')
		cp_area_addr = memparse(endp + 1, NULL);

	return 0;
}
early_param("cpmem", early_cpmem);

static void pxa_reserve_cp_memblock(void)
{
#ifdef CONFIG_OF
	mmp_reserve_cp();
#else
	/* Reserve memory for CP */
	BUG_ON(memblock_reserve(cp_area_addr, cp_area_size) != 0);
	memblock_free(cp_area_addr, cp_area_size);
	memblock_remove(cp_area_addr, cp_area_size);
	pr_info("Reserved CP memory : %dMB at %#.8x\n",
		(unsigned)cp_area_size/0x100000,
		(unsigned)cp_area_addr);
#endif
}

#ifdef CONFIG_OF
static u32 __initdata obm_base;
static u32 __initdata obm_size;

static int __init mmp_obm_fdt_find_info(unsigned long node, const char *uname,
		int depth, void *data)
{
	__be32 *prop;
	unsigned long len;

	if (!of_flat_dt_is_compatible(node, "marvell,obm-heap"))
		return 0;
#if defined(CONFIG_CRASH_DUMP)
	prop = of_get_flat_dt_prop(node, "obm-base-with-crash-dump", &len);
#elif defined(CONFIG_TZ_HYPERVISOR)
	prop = of_get_flat_dt_prop(node, "obm-base-with-tz", &len);
#else
	prop = of_get_flat_dt_prop(node, "obm-base-default", &len);
#endif
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find any obm-base property\n", __func__);
		return 0;
	}
	obm_base = be32_to_cpu(prop[0]);

	prop = of_get_flat_dt_prop(node, "obm-size", &len);
	if (!prop || (len != sizeof(unsigned long))) {
		pr_err("%s: Can't find obm-size property\n", __func__);
		return 0;
	}
	obm_size = be32_to_cpu(prop[0]);

	return 1;
}

static void __init mmp_reserve_obm(void)
{
	if (of_scan_flat_dt(mmp_obm_fdt_find_info, NULL)) {
		BUG_ON(memblock_reserve(obm_base, obm_size) != 0);
		memblock_free(obm_base, obm_size);
		memblock_remove(obm_base, obm_size);
		pr_info("Reserved OBM memory : %dMB at %#.8x\n",
			(unsigned)obm_size/0x100000,
			(unsigned)obm_base);
	} else
		pr_info("Reserved OBM memory dt prop fails : %dMB at %#.8x\n",
			(unsigned)obm_size/0x100000,
			(unsigned)obm_base);
}
#endif

static void pxa_reserve_obmmem(void)
{
#ifdef CONFIG_OF
	mmp_reserve_obm();
#else
	/* Reserve 1MB memory for obm */
	u32 obm_size = 0x100000;

	BUG_ON(memblock_reserve(PLAT_PHYS_OFFSET, obm_size) != 0);
	memblock_free(PLAT_PHYS_OFFSET, obm_size);
	memblock_remove(PLAT_PHYS_OFFSET, obm_size);
	pr_info("Reserved OBM memory : %dMB at %#.8x\n",
		(unsigned)obm_size/0x100000,
		(unsigned)PLAT_PHYS_OFFSET);
#endif
}

static void __init pxa988_reserve(void)
{
	pxa_reserve_obmmem();

	pxa_reserve_cp_memblock();

#ifdef CONFIG_PSTORE_RAM
	mmp_reserve_ramoops();
#endif

#ifdef CONFIG_GPU_RESERVE_MEM
	pxa_reserve_gpu_memblock();
#endif

#ifdef CONFIG_ION_PXA
	mmp_reserve_ion();
#endif

#ifdef CONFIG_MMP_DISP
	mmp_reserve_fbmem();
#endif

}

static const char *pxa1x88_dt_board_compat[] __initdata = {
	"mrvl,pxa988-dkb",
	"mrvl,pxa1088-dkb",
	"mrvl,pxa1L88-dkb",
	"mrvl,pxa1L88-sec",
	"mrvl,pxa1088-sec",
	NULL,
};

DT_MACHINE_START(PXA1L88_DT, "PXA1L88")
	.smp_init	= smp_init_ops(mmp_smp_init_ops),
	.map_io		= mmp_map_io,
	.init_irq	= pxa988_dt_irq_init,
	.init_time	= mmpx_timer_init,
	.init_machine	= pxa988_dt_init_machine,
	.dt_compat	= pxa1x88_dt_board_compat,
	.reserve	= pxa988_reserve,
	.restart	= mmp_arch_restart,
MACHINE_END
