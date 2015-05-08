/*
 * Base driver for Marvell 88PM800
 *
 * Copyright (C) 2013 Marvell International Ltd.
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 * Joseph(Yossi) Hanin <yhanin@marvell.com>
 * Qiao Zhou <zhouqiao@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/switch.h>
#include <linux/mfd/core.h>
#include <linux/mfd/88pm80x.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/reboot.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sec-common.h>
#include <linux/battery/sec_charging_common.h>

#if defined(CONFIG_MFD_88PM80x_DVC)
#include <mach/pxa-dvfs.h>

#define mV2uV	1000
#endif

#define PM800_CHIP_ID		(0x00)
#define PM800_REG_NUM		(0xf9)

#define PM80X_BASE_REG_NUM		0xf0
#define PM80X_POWER_REG_NUM		0x9b
#define PM80X_GPADC_REG_NUM		0xb6
#define	PM800_PROC_FILE		"driver/pm800_reg"

static int reg_pm800 = 0xffff;
static int pg_index;

/* Interrupt Registers */
#define PM800_INT_STATUS1		(0x05)
#define PM800_ONKEY_INT_STS1		(1 << 0)
#define PM800_EXTON_INT_STS1		(1 << 1)
#define PM800_CHG_INT_STS1			(1 << 2)
#define PM800_BAT_INT_STS1			(1 << 3)
#define PM800_RTC_INT_STS1			(1 << 4)
#define PM800_CLASSD_OC_INT_STS1	(1 << 5)

#define PM800_INT_STATUS2		(0x06)
#define PM800_VBAT_INT_STS2		(1 << 0)
#define PM800_VSYS_INT_STS2		(1 << 1)
#define PM800_VCHG_INT_STS2		(1 << 2)
#define PM800_TINT_INT_STS2		(1 << 3)
#define PM800_GPADC0_INT_STS2	(1 << 4)
#define PM800_TBAT_INT_STS2		(1 << 5)
#define PM800_GPADC2_INT_STS2	(1 << 6)
#define PM800_GPADC3_INT_STS2	(1 << 7)

#define PM800_INT_STATUS3		(0x07)

#define PM800_INT_STATUS4		(0x08)
#define PM800_GPIO0_INT_STS4		(1 << 0)
#define PM800_GPIO1_INT_STS4		(1 << 1)
#define PM800_GPIO2_INT_STS4		(1 << 2)
#define PM800_GPIO3_INT_STS4		(1 << 3)
#define PM800_GPIO4_INT_STS4		(1 << 4)

#define PM800_INT_ENA_1		(0x09)
#define PM800_ONKEY_INT_ENA1		(1 << 0)
#define PM800_EXTON_INT_ENA1		(1 << 1)
#define PM800_CHG_INT_ENA1			(1 << 2)
#define PM800_BAT_INT_ENA1			(1 << 3)
#define PM800_RTC_INT_ENA1			(1 << 4)
#define PM800_CLASSD_OC_INT_ENA1	(1 << 5)

#define PM800_INT_ENA_2		(0x0A)
#define PM800_VBAT_INT_ENA2		(1 << 0)
#define PM800_VSYS_INT_ENA2		(1 << 1)
#define PM800_VCHG_INT_ENA2		(1 << 2)
#define PM800_TINT_INT_ENA2		(1 << 3)
#define PM822_IRQ_LDO_PGOOD_EN		(1 << 4)
#define PM822_IRQ_BUCK_PGOOD_EN		(1 << 5)

#define PM800_INT_ENA_3		(0x0B)
#define PM800_GPADC0_INT_ENA3		(1 << 0)
#define PM800_GPADC1_INT_ENA3		(1 << 1)
#define PM800_GPADC2_INT_ENA3		(1 << 2)
#define PM800_GPADC3_INT_ENA3		(1 << 3)
#define PM800_GPADC4_INT_ENA3		(1 << 4)
#define PM822_IRQ_HS_DET_EN		(1 << 5)

#define PM800_INT_ENA_4		(0x0C)
#define PM800_GPIO0_INT_ENA4		(1 << 0)
#define PM800_GPIO1_INT_ENA4		(1 << 1)
#define PM800_GPIO2_INT_ENA4		(1 << 2)
#define PM800_GPIO3_INT_ENA4		(1 << 3)
#define PM800_GPIO4_INT_ENA4		(1 << 4)

#define PM800_POWER_REG_NUM		(0x9a)
#define PM800_GPADC_REG_NUM		(0xb5)
#define PM822_POWER_REG_NUM		(0x98)
#define PM822_GPADC_REG_NUM		(0xc7)

#define PM80X_BASE_REG_NUM		0xf0
#define PM80X_POWER_REG_NUM		0x9b
#define PM80X_GPADC_REG_NUM		0xb6

/* number of INT_ENA & INT_STATUS regs */
#define PM800_INT_REG_NUM			(4)

const struct regmap_config pm800_power_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PM800_POWER_REG_NUM,
};

const struct regmap_config pm800_gpadc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PM800_GPADC_REG_NUM,
};

const struct regmap_config pm822_power_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PM822_POWER_REG_NUM,
};

const struct regmap_config pm822_gpadc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PM822_GPADC_REG_NUM,
};

const struct regmap_config pm80x_test_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

/* Interrupt Number in 88PM800 */
enum {
	PM800_IRQ_ONKEY = 0,	/*EN1b0 *//*0 */
	PM800_IRQ_EXTON,	/*EN1b1 */
	PM800_IRQ_CHG,		/*EN1b2 */
	PM800_IRQ_BAT,		/*EN1b3 */
	PM800_IRQ_RTC,		/*EN1b4 */
	PM800_IRQ_CLASSD,	/*EN1b5 *//*5 */
	PM800_IRQ_VBAT,		/*EN2b0 */
	PM800_IRQ_VSYS,		/*EN2b1 */
	PM800_IRQ_VCHG,		/*EN2b2 */
	PM800_IRQ_TINT,		/*EN2b3 */
	PM822_IRQ_LDO_PGOOD,	/*EN2b4 *//*10 */
	PM822_IRQ_BUCK_PGOOD,	/*EN2b5 */
	PM800_IRQ_GPADC0,	/*EN3b0 */
	PM800_IRQ_GPADC1,	/*EN3b1 */
	PM800_IRQ_GPADC2,	/*EN3b2 */
	PM800_IRQ_GPADC3,	/*EN3b3 *//*15 */
	PM800_IRQ_GPADC4 = 16,	/*EN3b4 */
	PM822_IRQ_MIC_DET = 16,	/*EN3b4 */
	PM822_IRQ_HS_DET = 17,	/*EN3b4 */
	PM800_IRQ_GPIO0,	/*EN4b0 */
	PM800_IRQ_GPIO1,	/*EN4b1 */
	PM800_IRQ_GPIO2,	/*EN4b2 *//*20 */
	PM800_IRQ_GPIO3,	/*EN4b3 */
	PM800_IRQ_GPIO4,	/*EN4b4 */
	PM800_MAX_IRQ,
};

/* PM800: generation identification number */
#define PM800_CHIP_GEN_ID_NUM	0x3

enum pm8xx_parent {
	PM822 = 0x822,
	PM800 = 0x800,
};

static const struct i2c_device_id pm80x_id_table[] = {
	{"88pm822", PM822},
	{"88pm800", PM800},
	{} /* NULL terminated */
};
MODULE_DEVICE_TABLE(i2c, pm80x_id_table);

static const struct of_device_id pm80x_dt_ids[] = {
	{ .compatible = "marvell,88pm800", },
	{ .compatible = "marvell,88pm822", },
	{},
};
MODULE_DEVICE_TABLE(of, pm80x_dt_ids);

static struct resource rtc_resources[] = {
	{
	 .name = "88pm80x-rtc",
	 .start = PM800_IRQ_RTC,
	 .end = PM800_IRQ_RTC,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell rtc_devs[] = {
	{
	 .name = "88pm80x-rtc",
	 .of_compatible = "marvell,88pm80x-rtc",
	 .num_resources = ARRAY_SIZE(rtc_resources),
	 .resources = &rtc_resources[0],
	 .id = -1,
	 },
};

static struct resource onkey_resources[] = {
	{
	 .name = "88pm80x-onkey",
	 .start = PM800_IRQ_ONKEY,
	 .end = PM800_IRQ_ONKEY,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct mfd_cell onkey_devs[] = {
	{
	 .name = "88pm80x-onkey",
	 .of_compatible = "marvell,88pm80x-onkey",
	 .num_resources = 1,
	 .resources = &onkey_resources[0],
	 .id = -1,
	 },
};

static struct resource bat_resources[] = {
	{
	.name = "sec-fuelgauge",
	.start = PM800_IRQ_VBAT,
	.end = PM800_IRQ_VBAT,
	.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell bat_devs[] = {
	{
	.name = "sec-fuelgauge",
	.of_compatible = "marvell,88pm80x-battery",
	.num_resources = 1,
	.resources = &bat_resources[0],
	.id = -1,
	},
};

static struct resource usb_resources[] = {
	{
	.name = "88pm80x-usb",
	.start = PM800_IRQ_CHG,
	.end = PM800_IRQ_CHG,
	.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell usb_devs[] = {
	{
	.name = "88pm80x-usb",
	 .of_compatible = "marvell,88pm80x-usb",
	.num_resources = 1,
	.resources = &usb_resources[0],
	.id = -1,
	},
};

static struct mfd_cell regulator_devs[] = {
	{
	 .name = "88pm80x-regulator",
	 .of_compatible = "marvell,88pm80x-regulator",
	 .id = -1,
	},
};

static struct mfd_cell dvc_devs[] = {
	{
	 .name = "88pm8xx-dvc",
	 .of_compatible = "marvell,88pm8xx-dvc",
	 .id = -1,
	},
};

static struct mfd_cell vibrator_devs[] = {
	{
	 .name = "88pm80x-vibrator",
	 .of_compatible = "marvell,88pm80x-vibrator",
	 .id = -1,
	},
};

static const struct regmap_irq pm800_irqs[] = {
	/* INT0 */
	[PM800_IRQ_ONKEY] = {
		.mask = PM800_ONKEY_INT_ENA1,
	},
	[PM800_IRQ_EXTON] = {
		.mask = PM800_EXTON_INT_ENA1,
	},
	[PM800_IRQ_CHG] = {
		.mask = PM800_CHG_INT_ENA1,
	},
	[PM800_IRQ_BAT] = {
		.mask = PM800_BAT_INT_ENA1,
	},
	[PM800_IRQ_RTC] = {
		.mask = PM800_RTC_INT_ENA1,
	},
	[PM800_IRQ_CLASSD] = {
		.mask = PM800_CLASSD_OC_INT_ENA1,
	},
	/* INT1 */
	[PM800_IRQ_VBAT] = {
		.reg_offset = 1,
		.mask = PM800_VBAT_INT_ENA2,
	},
	[PM800_IRQ_VSYS] = {
		.reg_offset = 1,
		.mask = PM800_VSYS_INT_ENA2,
	},
	[PM800_IRQ_VCHG] = {
		.reg_offset = 1,
		.mask = PM800_VCHG_INT_ENA2,
	},
	[PM800_IRQ_TINT] = {
		.reg_offset = 1,
		.mask = PM800_TINT_INT_ENA2,
	},
	[PM822_IRQ_LDO_PGOOD] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_LDO_PGOOD_EN,
	},
	[PM822_IRQ_BUCK_PGOOD] = {
		.reg_offset = 1,
		.mask = PM822_IRQ_BUCK_PGOOD_EN,
	},
	/* INT2 */
	[PM800_IRQ_GPADC0] = {
		.reg_offset = 2,
		.mask = PM800_GPADC0_INT_ENA3,
	},
	[PM800_IRQ_GPADC1] = {
		.reg_offset = 2,
		.mask = PM800_GPADC1_INT_ENA3,
	},
	[PM800_IRQ_GPADC2] = {
		.reg_offset = 2,
		.mask = PM800_GPADC2_INT_ENA3,
	},
	[PM800_IRQ_GPADC3] = {
		.reg_offset = 2,
		.mask = PM800_GPADC3_INT_ENA3,
	},
	[PM800_IRQ_GPADC4] = {
		.reg_offset = 2,
		.mask = PM800_GPADC4_INT_ENA3,
	},
	[PM822_IRQ_HS_DET] = {
		.reg_offset = 2,
		.mask = PM822_IRQ_HS_DET_EN,
	},
	/* INT3 */
	[PM800_IRQ_GPIO0] = {
		.reg_offset = 3,
		.mask = PM800_GPIO0_INT_ENA4,
	},
	[PM800_IRQ_GPIO1] = {
		.reg_offset = 3,
		.mask = PM800_GPIO1_INT_ENA4,
	},
	[PM800_IRQ_GPIO2] = {
		.reg_offset = 3,
		.mask = PM800_GPIO2_INT_ENA4,
	},
	[PM800_IRQ_GPIO3] = {
		.reg_offset = 3,
		.mask = PM800_GPIO3_INT_ENA4,
	},
	[PM800_IRQ_GPIO4] = {
		.reg_offset = 3,
		.mask = PM800_GPIO4_INT_ENA4,
	},
};

#if defined(CONFIG_SEC_DEBUG)
static u8 power_on_reason;
unsigned char pm80x_get_power_on_reason(void)
{
	return power_on_reason;
}
#endif

static struct resource headset_resources_800[] = {
	{
		.name = "gpio-03",
		.start = PM800_IRQ_GPIO3,
		.end = PM800_IRQ_GPIO3,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "gpadc4",
		.start = PM800_IRQ_GPADC4,
		.end = PM800_IRQ_GPADC4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource headset_resources_822[] = {
	{
		.name = "88pm822-headset",
		.start = PM822_IRQ_HS_DET,
		.end = PM822_IRQ_HS_DET,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "gpadc4",
		.start = PM800_IRQ_GPADC4,
		.end = PM800_IRQ_GPADC4,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell headset_devs_800[] = {
	{
	 .name = "88pm800-headset",
	 .of_compatible = "marvell,88pm80x-headset",
	 .num_resources = ARRAY_SIZE(headset_resources_800),
	 .resources = &headset_resources_800[0],
	 .id = -1,
	 },
};

static int device_gpadc_init(struct pm80x_chip *chip,
				       struct pm80x_platform_data *pdata)
{
	struct pm80x_subchip *subchip = chip->subchip;
	struct regmap *map = subchip->regmap_gpadc;
	int ret = 0;

	if (!map) {
		dev_warn(chip->dev,
			 "Warning: gpadc regmap is not available!\n");
		return -EINVAL;
	}
	/*
	 * initialize GPADC without activating it turn on GPADC
	 * measurments
	 */
	ret = regmap_update_bits(map,
				 PM800_GPADC_MISC_CONFIG2,
				 PM800_GPADC_MISC_GPFSM_EN,
				 PM800_GPADC_MISC_GPFSM_EN);
	if (ret < 0)
		goto out;
	/*
	 * This function configures the ADC as requires for
	 * CP implementation.CP does not "own" the ADC configuration
	 * registers and relies on AP.
	 * Reason: enable automatic ADC measurements needed
	 * for CP to get VBAT and RF temperature readings.
	 */
	ret = regmap_update_bits(map, PM800_GPADC_MEAS_EN1,
				 PM800_MEAS_EN1_VBAT, PM800_MEAS_EN1_VBAT);
	if (ret < 0)
		goto out;

	/* enable all of the GPADC */
	ret = regmap_update_bits(map, PM800_GPADC_MEAS_EN2, 0x5c, 0x5c);
	if (ret < 0)
		goto out;

	/*
	 * the defult of PM800 is GPADC operates at 100Ks/s rate
	 * and Number of GPADC slots with active current bias prior
	 * to GPADC sampling = 1 slot for all GPADCs set for
	 * Temprature mesurmants
	 *
	 * enable all of the bias_en/bias_out_en
	 */
	ret = regmap_update_bits(map, PM800_GP_BIAS_ENA1, 0xff, 0xff);
	if (ret < 0)
		goto out;

	dev_info(chip->dev, "pm800 device_gpadc_init: Done\n");
	return 0;

out:
	dev_info(chip->dev, "pm800 device_gpadc_init: Failed!\n");
	return ret;
}

static int device_onkey_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &onkey_devs[0],
			      ARRAY_SIZE(onkey_devs), &onkey_resources[0],
			      regmap_irq_chip_get_base(chip->irq_data), NULL);

	if (ret) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		return ret;
	}

	return 0;
}


static int device_usb_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	int ret;

	usb_devs[0].platform_data = pdata->usb;
	usb_devs[0].pdata_size = sizeof(struct pm80x_usb_pdata);
	ret = mfd_add_devices(chip->dev, 0, &usb_devs[0],
			      ARRAY_SIZE(usb_devs), NULL,
			      regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add usb subdev\n");
		return ret;
	}

	return 0;
}

static int device_vibrator_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	int ret;

	vibrator_devs[0].platform_data = pdata->vibrator;
	vibrator_devs[0].pdata_size = sizeof(struct pm80x_vibrator_pdata);

	ret = mfd_add_devices(chip->dev, 0, &vibrator_devs[0],
		ARRAY_SIZE(vibrator_devs), NULL,
		regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add vibrator subdev\n");
		return ret;
	}

	return 0;
}

static int device_rtc_init(struct pm80x_chip *chip,
				struct pm80x_platform_data *pdata)
{
	int ret;

	rtc_devs[0].platform_data = pdata->rtc;
	rtc_devs[0].pdata_size =
			pdata->rtc ? sizeof(struct pm80x_rtc_pdata) : 0;
	ret = mfd_add_devices(chip->dev, 0, &rtc_devs[0],
			      ARRAY_SIZE(rtc_devs), NULL,
			      regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add rtc subdev\n");
		return ret;
	}

	return 0;
}

static int device_battery_init(struct pm80x_chip *chip,
					   struct pm80x_platform_data *pdata)
{
	int ret;



	if (pdata) {
		bat_devs[0].platform_data = pdata->fuelgauge_data;
		bat_devs[0].pdata_size =
				sizeof(struct sec_battery_platform_data);

		ret = mfd_add_devices(chip->dev, 0, &bat_devs[0],
			      ARRAY_SIZE(bat_devs), NULL,
			      regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add battery subdev\n");
		return ret;
		} else {
			dev_info(chip->dev,
				"[%s]:Added mfd bat_devs\n", __func__);
		}
	}

	return 0;
}

static int device_regulator_init(struct pm80x_chip *chip,
					   struct pm80x_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &regulator_devs[0],
			      ARRAY_SIZE(regulator_devs), NULL,
			      regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add regulator subdev\n");
		return ret;
	}

	return 0;
}

static int device_headset_init(struct pm80x_chip *chip,
					   struct pm80x_platform_data *pdata)
{
	int ret;

	switch (chip->type) {
	case CHIP_PM800:
		headset_devs_800[0].resources =
			&headset_resources_800[0];
		headset_devs_800[0].num_resources =
			ARRAY_SIZE(headset_resources_800);
		break;
	case CHIP_PM822:
		headset_devs_800[0].resources =
			&headset_resources_822[0];
		headset_devs_800[0].num_resources =
			ARRAY_SIZE(headset_resources_822);
		break;
	default:
		return -EINVAL;
	}

	headset_devs_800[0].platform_data = pdata->headset;
	ret = mfd_add_devices(chip->dev, 0, &headset_devs_800[0],
			      ARRAY_SIZE(headset_devs_800), NULL,
			      regmap_irq_chip_get_base(chip->irq_data),
			      NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add headset subdev\n");
		return ret;
	}

	return 0;
}

static int device_dvc_init(struct pm80x_chip *chip,
					   struct pm80x_platform_data *pdata)
{
	int ret;

	ret = mfd_add_devices(chip->dev, 0, &dvc_devs[0],
			      ARRAY_SIZE(dvc_devs), NULL,
			      regmap_irq_chip_get_base(chip->irq_data), NULL);
	if (ret) {
		dev_err(chip->dev, "Failed to add dvc subdev\n");
		return ret;
	}

	return 0;
}

static int device_irq_init_800(struct pm80x_chip *chip)
{
	struct regmap *map = chip->regmap;
	unsigned long flags = IRQF_ONESHOT;
	int data, mask, ret = -EINVAL;

	if (!map || !chip->irq) {
		dev_err(chip->dev, "incorrect parameters\n");
		return -EINVAL;
	}

	/*
	 * irq_mode defines the way of clearing interrupt. it's read-clear by
	 * default.
	 */
	mask =
	    PM800_WAKEUP2_INV_INT | PM800_WAKEUP2_INT_CLEAR |
	    PM800_WAKEUP2_INT_MASK;

	data = (chip->irq_mode) ?
		PM800_WAKEUP2_INT_WRITE_CLEAR : PM800_WAKEUP2_INT_READ_CLEAR;
	ret = regmap_update_bits(map, PM800_WAKEUP2, mask, data);

	if (ret < 0)
		goto out;

	ret =
	    regmap_add_irq_chip(chip->regmap, chip->irq, flags, -1,
				chip->regmap_irq_chip, &chip->irq_data);

out:
	return ret;
}

static void device_irq_exit_800(struct pm80x_chip *chip)
{
	regmap_del_irq_chip(chip->irq, chip->irq_data);
}

static struct regmap_irq_chip pm800_irq_chip = {
	.name = "88pm800",
	.irqs = pm800_irqs,
	.num_irqs = ARRAY_SIZE(pm800_irqs),

	.num_regs = 4,
	.status_base = PM800_INT_STATUS1,
	.mask_base = PM800_INT_ENA_1,
	.ack_base = PM800_INT_STATUS1,
	.init_ack_masked = true,
	.mask_invert = 1,
};

static int pm800_pages_init(struct pm80x_chip *chip)
{
	struct pm80x_subchip *subchip;
	struct i2c_client *client = chip->client;

	int ret = 0;

	subchip = chip->subchip;
	if (!subchip || !subchip->power_page_addr || !subchip->gpadc_page_addr)
		return -ENODEV;

	/* PM800 block power page */
	subchip->power_page = i2c_new_dummy(client->adapter,
					    subchip->power_page_addr);
	if (subchip->power_page == NULL) {
		ret = -ENODEV;
		goto out;
	}

	if (chip->type == CHIP_PM800)
		subchip->regmap_power = devm_regmap_init_i2c(
				subchip->power_page,
				&pm800_power_regmap_config);
	else if (chip->type == CHIP_PM822)
		subchip->regmap_power = devm_regmap_init_i2c(
				subchip->power_page,
				&pm822_power_regmap_config);

	if (IS_ERR(subchip->regmap_power)) {
		ret = PTR_ERR(subchip->regmap_power);
		dev_err(chip->dev,
			"Failed to allocate regmap_power: %d\n", ret);
		goto out;
	}

	i2c_set_clientdata(subchip->power_page, chip);

	/* PM800 block GPADC */
	subchip->gpadc_page = i2c_new_dummy(client->adapter,
					    subchip->gpadc_page_addr);
	if (subchip->gpadc_page == NULL) {
		ret = -ENODEV;
		goto out;
	}

	if (chip->type == CHIP_PM800)
		subchip->regmap_gpadc = devm_regmap_init_i2c(
				subchip->gpadc_page,
				&pm800_gpadc_regmap_config);
	else if (chip->type == CHIP_PM822)
		subchip->regmap_gpadc = devm_regmap_init_i2c(
				subchip->gpadc_page,
				&pm822_gpadc_regmap_config);

	if (IS_ERR(subchip->regmap_gpadc)) {
		ret = PTR_ERR(subchip->regmap_gpadc);
		dev_err(chip->dev,
			"Failed to allocate regmap_gpadc: %d\n", ret);
		goto out;
	}
	i2c_set_clientdata(subchip->gpadc_page, chip);

	/* PM800 block TEST: i2c addr 0x37 */
	if (subchip->test_page_addr) {
		subchip->test_page = i2c_new_dummy(client->adapter,
				subchip->test_page_addr);
		if (subchip->test_page == NULL) {
			ret = -ENODEV;
			goto out;
		}
		subchip->regmap_test = devm_regmap_init_i2c(
					subchip->test_page,
					&pm80x_test_regmap_config);

		if (IS_ERR(subchip->regmap_test)) {
			ret = PTR_ERR(subchip->regmap_test);
			dev_err(chip->dev,
				"Failed to allocate regmap_test: %d\n", ret);
			goto out;
		}
	}
	i2c_set_clientdata(subchip->test_page, chip);
out:
	return ret;
}

static void pm800_pages_exit(struct pm80x_chip *chip)
{
	struct pm80x_subchip *subchip;

	subchip = chip->subchip;

	if (subchip && subchip->power_page)
		i2c_unregister_device(subchip->power_page);

	if (subchip && subchip->gpadc_page)
		i2c_unregister_device(subchip->gpadc_page);

	if (subchip && subchip->test_page) {
		regmap_exit(subchip->regmap_test);
		i2c_unregister_device(subchip->test_page);
	}
}

static int device_800_init(struct pm80x_chip *chip,
				     struct pm80x_platform_data *pdata)
{
	int ret;
	unsigned int val,data;

	if (!pdata) {
		dev_warn(chip->dev, "pdata is null!!!\n");
		return -EINVAL;
	}

#if defined(CONFIG_SEC_DEBUG)
	/* read power on reason from PMIC general use register */
	ret = regmap_read(chip->regmap, PMIC_GENERAL_USE_REGISTER, &data);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read PMIC_GENERAL_USE_REGISTER : %d\n"
				, ret);
		goto out;
	}
	
	pr_info("%s read register PMIC_GENERAL_USE_REGISTER [%d]\n", __func__,
			data);
	val = data & (PMIC_GENERAL_USE_REBOOT_DN_MASK);
	/* read power on reason from PMIC general use register */	
	if (val != PMIC_GENERAL_USE_BOOT_BY_FULL_RESET)		
	{
		data &= ~(PMIC_GENERAL_USE_REBOOT_DN_MASK);
		data |= PMIC_GENERAL_USE_BOOT_BY_HW_RESET;
		regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,data);
	}
	power_on_reason = (u8)val;
#endif
	/*
	 * alarm wake up bit will be clear in device_irq_init(),
	 * read before that
	 */
	ret = regmap_read(chip->regmap, PM800_RTC_CONTROL, &val);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to read RTC register: %d\n", ret);
		goto out;
	}
	if (val & PM800_ALARM_WAKEUP) {
		if (pdata && pdata->rtc)
			pdata->rtc->rtc_wakeup = 1;
	}

	ret = device_gpadc_init(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init gpadc\n", __func__);
		goto out;
	}

	chip->regmap_irq_chip = &pm800_irq_chip;
	chip->irq_mode = pdata->irq_mode;

	ret = device_irq_init_800(chip);
	if (ret < 0) {
		dev_err(chip->dev, "[%s]Failed to init pm800 irq\n", __func__);
		goto out;
	}

	ret = device_onkey_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add onkey subdev\n");
		goto out_dev;
	}

	ret = device_rtc_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add rtc subdev\n");
		goto out;
	}
	ret = device_battery_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add battery subdev\n");
		goto out;
	}

	ret = device_regulator_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add regulators subdev\n");
		goto out;
	}

	ret = device_headset_init(chip, pdata);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to add headset subdev\n");
		goto out_dev;
	}

	ret = device_dvc_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add dvc subdev\n");

		goto out;
	}

	ret = device_usb_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add usb subdev\n");
		goto out;
	}

	ret = device_vibrator_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to add vibrator subdev\n");

		goto out;
	}

	return 0;
out_dev:
	mfd_remove_devices(chip->dev);
	device_irq_exit_800(chip);
out:
	return ret;
}

static int pm800_dt_init(struct device_node *np,
			 struct device *dev,
			 struct pm80x_platform_data *pdata)
{
	pdata->irq_mode =
		!of_property_read_bool(np, "marvell,88pm800-irq-write-clear");

	return 0;
}
static int pm800_init_config(struct pm80x_chip *chip)
{
	int data;
	if (!chip || !chip->regmap || !chip->subchip
	    || !chip->subchip->regmap_power) {
		pr_err("%s:chip is not availiable!\n", __func__);
		return -EINVAL;
	}

	/*base page:reg 0xd0.7 = 1 32kHZ generated from XO */
	regmap_read(chip->regmap, PM800_RTC_CONTROL, &data);
	data |= (1 << 7);
	regmap_write(chip->regmap, PM800_RTC_CONTROL, data);

	/* Set internal digital sleep voltage as 1.2V */
	regmap_read(chip->regmap, PM800_LOW_POWER1, &data);
	data &= ~(0xf << 4);
	regmap_write(chip->regmap, PM800_LOW_POWER1, data);
	/* Enable 32Khz-out-3 low jitter XO_LJ = 1 in pm800
	 * Enable 32Khz-out-2 low jitter XO_LJ = 1 in pm822
	 * they are the same bit
	 */
	regmap_read(chip->regmap, PM800_LOW_POWER2, &data);
	data |= (1 << 5);
	regmap_write(chip->regmap, PM800_LOW_POWER2, data);

	switch (chip->type) {
	case CHIP_PM800:
		/* Enable 32Khz-out-from XO 1, 2, 3 all enabled */
		regmap_write(chip->regmap, PM800_RTC_MISC2, 0x2a);
		break;

	case CHIP_PM822:
		/* select 15pF internal capacitance on XTAL1 and XTAL2*/
		regmap_read(chip->regmap, PM800_RTC_MISC6, &data);
		data = (data & 0xf) |(0x5 << 4);
		regmap_write(chip->regmap, PM800_RTC_MISC6, data);

		/* Enable 32Khz-out-from XO 1, 2 all enabled */
		regmap_write(chip->regmap, PM800_RTC_MISC2, 0xa);

		/* gps use the LDO13 set the current 170mA  */
		regmap_read(chip->subchip->regmap_power,
					PM822_LDO13_CTRL, &data);
		data &= ~(0x3);
		data |= (0x2);
		regmap_write(chip->subchip->regmap_power,
					PM822_LDO13_CTRL, data);

		/*Disable Soft Start of LDO 1/4/13 */
	        regmap_read(chip->subchip->regmap_power,
					PM822_LDO_MISC1, &data);
		data |= (1 << 3);
		regmap_write(chip->subchip->regmap_power,
					PM822_LDO_MISC1, data);
		regmap_read(chip->subchip->regmap_power,
					PM822_LDO_MISC2, &data);
		data |= (1 << 7);
		regmap_write(chip->subchip->regmap_power,
					PM822_LDO_MISC2, data);
		regmap_read(chip->subchip->regmap_power,
					PM822_LDO_MISC8, &data);
		data |= (1 << 3);
		regmap_write(chip->subchip->regmap_power,
					PM822_LDO_MISC8, data);
		break;

	case CHIP_PM86X:
		/* enable buck1 dual phase mode*/
		regmap_read(chip->subchip->regmap_power, PM860_BUCK1_MISC,
				&data);
		data |= BUCK1_DUAL_PHASE_SEL;
		regmap_write(chip->subchip->regmap_power, PM860_BUCK1_MISC,
				data);
		break;

	default:
		dev_err(chip->dev, "Unknown device type: %d\n", chip->type);
	}

	/*
	 * Block wakeup attempts when VSYS rises above
	 * VSYS_UNDER_RISE_TH1, or power off may fail.
	 * it is set to prevent contimuous attempt to power up
	 * incase the VSYS is above the VSYS_LOW_TH threshold.
	 * Enable Fault_WU to make watchdog as reset.
	 */
	regmap_read(chip->regmap, PM800_RTC_MISC5, &data);
	data |= 0x5;
	regmap_write(chip->regmap, PM800_RTC_MISC5, data);

	/* Enabele LDO and BUCK clock gating in lpm */
	regmap_read(chip->regmap, PM800_LOW_POWER_CONFIG3, &data);
	data |= (1 << 7);
	regmap_write(chip->regmap, PM800_LOW_POWER_CONFIG3, data);
	/*
	 * Disable reference group sleep mode
	 * - to reduce power fluctuation in suspend
	 */
	regmap_read(chip->regmap, PM800_LOW_POWER_CONFIG4, &data);
	data &= ~(1 << 7);
	regmap_write(chip->regmap, PM800_LOW_POWER_CONFIG4, data);

	/* Enable voltage change in pmic, POWER_HOLD = 1 */
	regmap_read(chip->regmap, PM800_WAKEUP1, &data);
	data |= (1 << 7);
	regmap_write(chip->regmap, PM800_WAKEUP1, data);

	/* Set sleep mode as 0.8V */
	regmap_write(chip->subchip->regmap_power, PM800_BUCK1_SLEEP, 0x10);
	/* Set buck1 audio mode as 0.8V */
	regmap_write(chip->subchip->regmap_power, PM800_AUDIO_MODE_CONFIG1, 0x10);

	/* Enable buck sleep mode */
	regmap_write(chip->subchip->regmap_power, PM800_BUCK_SLP1, 0xaa);
	regmap_write(chip->subchip->regmap_power, PM800_BUCK_SLP2, 0x2);

	/* Dump power-down log */
	regmap_read(chip->regmap, PM800_POWER_DOWN_LOG1, &data);
	dev_dbg(chip->dev, "PowerDW Log1 0x%x: 0x%x\n",
		PM800_POWER_DOWN_LOG1, data);
	regmap_read(chip->regmap, PM800_POWER_DOWN_LOG2, &data);
	dev_dbg(chip->dev, "PowerDW Log2 0x%x: 0x%x\n",
		PM800_POWER_DOWN_LOG2, data);

	/*set buck2 and buck4 driver selection to be full.
	* this bit is now reserved and default value is 0, if want full
	* drive possibility it should be set to 1.
	* In A1 version it will be set to 1 by default.
	*/
	regmap_read(chip->subchip->regmap_power, 0x7c, &data);
	data |= (1 << 2);
	regmap_write(chip->subchip->regmap_power, 0x7c, data);

	regmap_read(chip->subchip->regmap_power, 0x82, &data);
	data |= (1 << 2);
	regmap_write(chip->subchip->regmap_power, 0x82, data);

	/* 0126 pmic workaround */
	regmap_write(chip->regmap, 0x21, 0x30);
	regmap_write(chip->regmap, 0x23, 0x80);
	regmap_write(chip->regmap, 0x50, 0x0c);
	regmap_write(chip->regmap, 0x55, 0x0c);

	regmap_write(chip->subchip->regmap_power, 0x5c, 0xaa);
	regmap_write(chip->subchip->regmap_power, 0x5d, 0xaa);
	regmap_write(chip->subchip->regmap_power, 0x5e, 0xaa);
	regmap_write(chip->subchip->regmap_power, 0x5f, 0x0a);

	regmap_write(chip->subchip->regmap_gpadc, PM800_GPADC_MISC_CONFIG2, 0x33);
	regmap_write(chip->subchip->regmap_gpadc, PM800_GPADC_MEASURE_OFF2, 0x0f);
	regmap_write(chip->subchip->regmap_power, PM800_BUCK_SHARED_CTRL, 0x06);

	return 0;
}

static struct pm80x_chip *chip_g;
#define PM800_SW_PDOWN			(1 << 5)
void sw_poweroff(void)
{
	int ret;
	u8 reg, data, buf[2];
	struct i2c_client *client = chip_g->client;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
	};

	pr_info("turning off power....\n");

	/* set i2c to pio mode */
	i2c_set_pio_mode(client->adapter, 1);

	reg = PMIC_GENERAL_USE_REGISTER;
	data = PMIC_GENERAL_USE_BOOT_BY_NONE |
				PMIC_GENERAL_USE_SHUTDOWN_BY_POWEROFF;
	buf[0] = reg;

	/* send register */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = data;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s send reg fails...\n", __func__);
		BUG();
	}

	/* send register */
	reg = PM800_WAKEUP1;
	buf[0] = reg;
	ret = __i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("%s send reg fails...\n", __func__);
		BUG();
	}

	/* send data */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf[0] = reg;
	msgs[0].buf[1] = data | PM800_SW_PDOWN;
	ret = __i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0) {
		pr_err("%s send data fails...\n", __func__);
		BUG();
	}
	/* we will not see this log if power off is sucessful */
	pr_err("turning off power failes!\n");
}

static int reboot_notifier_func(struct notifier_block *this,
		unsigned long code, void *p)
{
	int data;
	struct pm80x_chip *chip;
	unsigned char pmic_download_register = 0;

	char *cmd = p;
	pr_info("reboot notifier...\n");

	chip = container_of(this, struct pm80x_chip, reboot_notifier);
	regmap_read(chip->regmap, PMIC_GENERAL_USE_REGISTER, &data);
	data &= ~(PMIC_GENERAL_USE_REBOOT_DN_MASK);

	if (cmd) {
		if (!strcmp(cmd, "recovery")) {
			pr_info("Device will enter recovery mode on next booting\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_FULL_RESET;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "recovery_done")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "arm11_fota")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_FOTA;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "alarm")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_RTC_ALARM;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x4f4c")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_LOW;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x494d")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_MID;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "debug0x4948")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_HIGH;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "GlobalActions restart")) {
			data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strcmp(cmd, "download")) {
			pmic_download_register = PMIC_GENERAL_DOWNLOAD_MODE_FUS
							+ DOWNLOAD_FUS_SUD_BASE;
			data |= pmic_download_register;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else if (!strncmp(cmd, "sud", 3)) {
			/* Value : 21 ~ 29 */
			pmic_download_register = cmd[3] - '0' +
							DOWNLOAD_FUS_SUD_BASE;
			data |= pmic_download_register;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
									data);
		} else {
			if (get_recoverymode() == 1) {
				pr_info("reset noti recovery mode\n");
				data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
				regmap_write(chip->regmap,
					PMIC_GENERAL_USE_REGISTER, data);
			} else {
				pr_info("reset noti intended\n");
				data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
				regmap_write(chip->regmap,
					PMIC_GENERAL_USE_REGISTER, data);
			}
		}
	} else {
		if (get_recoverymode() == 1) {
			pr_info("reset noti recovery p = null\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
								data);
		} else {
			pr_info("reset noti intended p = null\n");
			data |= PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET;
			regmap_write(chip->regmap, PMIC_GENERAL_USE_REGISTER,
								data);
		}
	}
	if (code != SYS_POWER_OFF) {
		pr_info("enable PMIC watchdog\n");
		regmap_read(chip->regmap, PM800_WAKEUP1, &data);
		pr_info("PM800_WAKEUP1 Reg(0x%02x) is 0x%02x\n",
				PM800_WAKEUP1, data);
		data |= PM800_WAKEUP1_WD_MODE;
		regmap_write(chip->regmap, PM800_WAKEUP1, data);

		regmap_read(chip->regmap, PM800_WAKEUP2, &data);
		pr_info("PM800_WAKEUP2 Reg(0x%02x) is 0x%02x\n",
				PM800_WAKEUP2, data);
		data &= ~(PM800_WD_TIMER_ACT_MASK);
		data |= PM800_WD_TIMER_ACT_8S;
		regmap_write(chip->regmap,
				PM800_WAKEUP2, data);
		pr_info("0x%02x is written to PM800_WAKEUP2 Reg(0x%02x)\n",
				data, PM800_WAKEUP2);

		regmap_write(chip->regmap, PM800_WATCHDOG_REG,
				PM800_WD_EN);
		regmap_read(chip->regmap, PM800_WATCHDOG_REG, &data);
		pr_info("WATCHDOG Reg(0x%02x) is 0x%02x\n",
				PM800_WATCHDOG_REG, data);
	}

	return 0;
}

#if defined(CONFIG_MFD_88PM80x_DVC)
static int set_pmic_volt(int lvl, unsigned int mv)
{
	return pm8xx_dvc_setvolt(PM800_ID_BUCK1, lvl, mv * mV2uV);
}

static int get_pmic_volt(int lvl)
{
	int uv = 0, ret = 0;

	ret = pm8xx_dvc_getvolt(PM800_ID_BUCK1, lvl, &uv);
	if (ret < 0)
		return ret;
	return DIV_ROUND_UP(uv, mV2uV);
}
#endif

static ssize_t pm800_dump_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	unsigned int reg_val = 0;
	int len = 0;
	struct pm80x_chip *chip = file->private_data;
	int i;

	if (reg_pm800 == 0xffff) {
		pr_info("pm800: base page:\n");
		for (i = 0; i < PM80X_BASE_REG_NUM; i++) {
			regmap_read(chip->regmap, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}
		pr_info("pm80x: power page:\n");
		for (i = 0; i < PM80X_POWER_REG_NUM; i++) {
			regmap_read(chip->subchip->regmap_power, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}
		pr_info("pm80x: gpadc page:\n");
		for (i = 0; i < PM80X_GPADC_REG_NUM; i++) {
			regmap_read(chip->subchip->regmap_gpadc, i, &reg_val);
			pr_info("[0x%02x]=0x%02x\n", i, reg_val);
		}

		len = 0;
	} else {

		switch (pg_index) {
		case 0:
			regmap_read(chip->regmap, reg_pm800, &reg_val);
			break;
		case 1:
			regmap_read(chip->subchip->regmap_power, reg_pm800,
					&reg_val);
			break;
		case 2:
			regmap_read(chip->subchip->regmap_gpadc, reg_pm800,
					&reg_val);
			break;
		case 7:
			regmap_read(chip->subchip->regmap_test, reg_pm800,
					&reg_val);
			break;
		default:
			pr_err("pg_index error!\n");
			return 0;
		}

		len = sprintf(buf, "reg_pm800=0x%x, pg_index=0x%x, val=0x%x\n",
			      reg_pm800, pg_index, reg_val);
	}
	return len;
}

static ssize_t pm800_dump_write(struct file *file,
				const char __user *buff,
				size_t len, loff_t *ppos)
{
	u8 reg_val;
	struct pm80x_chip *chip = file->private_data;

	char messages[20], index[20];
	memset(messages, '\0', 20);
	memset(index, '\0', 20);

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		if ((strlen(messages) != 10) &&
		    (strlen(messages) != 9)) {
			pr_err("Right format: -0x[page_addr] 0x[reg_addr]\n");
			return len;
		}
		/* set the register index */
		memcpy(index, messages + 1, 3);

		if (kstrtoint(index, 16, &pg_index) < 0)
			return -EINVAL;

		pr_info("pg_index = 0x%x\n", pg_index);

		memcpy(index, messages + 5, 4);
		if (kstrtoint(index, 16, &reg_pm800) < 0)
			return -EINVAL;
		pr_info("reg_pm800 = 0x%x\n", reg_pm800);
	} else if ('+' == messages[0]) {
		/* enable to get all the reg value */
		reg_pm800 = 0xffff;
		pr_info("read all reg enabled!\n");
	} else {
		if ((reg_pm800 == 0xffff) ||
		    ('0' != messages[0])) {
			pr_err("Right format: -0x[page_addr] 0x[reg_addr]\n");
			return len;
		}
		/* set the register value */
		if (kstrtou8(messages, 16, &reg_val) < 0)
			return -EINVAL;

		switch (pg_index) {
		case 0:
			regmap_write(chip->regmap, reg_pm800, reg_val & 0xff);
			break;
		case 1:
			regmap_write(chip->subchip->regmap_power,
				     reg_pm800, reg_val & 0xff);
			break;
		case 2:
			regmap_write(chip->subchip->regmap_gpadc,
				     reg_pm800, reg_val & 0xff);
			break;
		case 7:
			regmap_write(chip->subchip->regmap_test,
				     reg_pm800, reg_val & 0xff);
			break;
		default:
			pr_err("pg_index error!\n");
			break;

		}
	}

	return len;
}

static const struct file_operations pm800_dump_ops = {
	.owner		= THIS_MODULE,
	.open		= simple_open,
	.read		= pm800_dump_read,
	.write		= pm800_dump_write,
};

static inline int pm800_dump_debugfs_init(struct pm80x_chip *chip)
{
	struct dentry *pm800_dump_reg;

	pm800_dump_reg = debugfs_create_file("pm800_reg", S_IRUGO | S_IFREG,
			    NULL, (void *)chip, &pm800_dump_ops);

	if (pm800_dump_reg == NULL) {
		pr_err("create pm800 debugfs error!\n");
		return -ENOENT;
	} else if (pm800_dump_reg == ERR_PTR(-ENODEV)) {
		pr_err("pm800_dump_reg error!\n");
		return -ENOENT;
	}
	return 0;
}

static void pm800_dump_debugfs_remove(struct pm80x_chip *chip)
{
		debugfs_remove_recursive(chip->debugfs);
}

static int pm800_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret = 0;
	struct pm80x_chip *chip;
	struct pm80x_platform_data *pdata = client->dev.platform_data;
	struct device_node *node = client->dev.of_node;
	struct pm80x_subchip *subchip;
#if defined(CONFIG_MFD_88PM80x_DVC)
	struct dvfs_info pm822_info;
#endif
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					     sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
			client->dev.platform_data = pdata;
		}
		ret = pm800_dt_init(node, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		return -EINVAL;
	}

	/*
	 * RTC in pmic can run even the core is powered off, and user can set
	 * alarm in RTC. When the alarm is time out, the PMIC will power up
	 * the core, and the whole system will boot up. When PMIC driver is
	 * probed, it will read out some register to find out whether this
	 * boot is caused by RTC timeout or not, and it need pass this
	 * information to RTC driver.
	 * So we need rtc platform data to be existed to pass this information.
	 */
	if (!pdata->rtc) {
		pdata->rtc = devm_kzalloc(&client->dev,
					  sizeof(*(pdata->rtc)), GFP_KERNEL);
		if (!pdata->rtc)
			return -ENOMEM;
	}

	/* will pass the data in sec-fuelgauge driver itself */
	/*registring here with NUL data*/
	pdata->fuelgauge_data = NULL;
	ret = pm80x_init(client);
	if (ret) {
		dev_err(&client->dev, "pm800_init fail\n");
		goto out_init;
	}

	chip = i2c_get_clientdata(client);

	/* init subchip for PM800 */
	subchip =
	    devm_kzalloc(&client->dev, sizeof(struct pm80x_subchip),
			 GFP_KERNEL);
	if (!subchip) {
		ret = -ENOMEM;
		goto err_subchip_alloc;
	}

	/* pm800 has 2 addtional pages to support power and gpadc. */
	subchip->power_page_addr = client->addr + 1;
	subchip->gpadc_page_addr = client->addr + 2;
	subchip->test_page_addr =  client->addr + 7;
	chip->subchip = subchip;

	ret = pm800_pages_init(chip);
	if (ret) {
		dev_err(&client->dev, "pm800_pages_init failed!\n");
		goto err_page_init;
	}

	ret = device_800_init(chip, pdata);
	if (ret) {
		dev_err(chip->dev, "Failed to initialize 88pm800 devices\n");
		goto err_device_init;
	}

#if defined(CONFIG_MFD_88PM80x_DVC)
	pm822_info.set_vccmain_volt = set_pmic_volt;
	pm822_info.get_vccmain_volt = get_pmic_volt;
	pm822_info.pmic_rampup_step = 12500;
	setup_pmic_dvfs(&pm822_info);
#endif

	if (pdata && pdata->plat_config)
		pdata->plat_config(chip, pdata);
	/*common config registers for pmic*/
	pm800_init_config(chip);

	chip->reboot_notifier.notifier_call = reboot_notifier_func;

	chip_g = chip;
	pm_power_off = sw_poweroff;
	register_reboot_notifier(&(chip->reboot_notifier));

	ret = pm800_dump_debugfs_init(chip);
	if (!ret)
		dev_info(chip->dev, "88pm800 debugfs created!\n");

	return 0;

err_device_init:
	pm800_pages_exit(chip);
err_page_init:
err_subchip_alloc:
	pm80x_deinit();
out_init:
	return ret;
}

static int pm800_remove(struct i2c_client *client)
{
	struct pm80x_chip *chip = i2c_get_clientdata(client);

	mfd_remove_devices(chip->dev);
	device_irq_exit_800(chip);

	pm800_pages_exit(chip);
	pm80x_deinit();

	pm800_dump_debugfs_remove(chip);

	return 0;
}

void buck2_sleepmode_control_for_wifi(int on)
{
	int data;

	if (on) {
		/*
		 * Disable VBUCK2's sleep mode, allow big current output
		 * even system enter to suspend
		 * As SD8787 use it as 1.8V supply, it would still work
		 * druing suspend and need much current
		 *
		 */
		regmap_read(chip_g->subchip->regmap_power,
				PM800_BUCK_SLP1, &data);
		data = (data & ~PM800_BUCK2_SLP1_MASK) |
			(PM800_BUCK_SLP_PWR_ACT2 << PM800_BUCK2_SLP1_SHIFT);
		regmap_write(chip_g->subchip->regmap_power, PM800_BUCK_SLP1,
				data);
	} else {
		/*
		 * Enable VBUCK2's sleep mode again (Only 5mA ability)
		 * If SD8787 is power off, VBUCK2 sleep mode has not side
		 * impact to Sd8787, but has benifit to whole power consumption
		 */
		regmap_read(chip_g->subchip->regmap_power,
				PM800_BUCK_SLP1, &data);
		data = (data & ~PM800_BUCK2_SLP1_MASK) |
			(PM800_BUCK_SLP_PWR_LOW << PM800_BUCK2_SLP1_SHIFT);
		regmap_write(chip_g->subchip->regmap_power, PM800_BUCK_SLP1,
				data);
	}

}
EXPORT_SYMBOL(buck2_sleepmode_control_for_wifi);

void ldo10_sleepmode_control_for_wifi(int on)
{
	int data;

	if (on) {
		regmap_read(chip_g->subchip->regmap_power,
				0x5e, &data);
		data = (data & ~PM800_LDO10_SLP3_MASK) |
			(0x3 << PM800_LDO10_SLP3_SHIFT);
		regmap_write(chip_g->subchip->regmap_power, 0x5e, data);
	} else {
		regmap_read(chip_g->subchip->regmap_power, 0x5e, &data);
		data = (data & ~PM800_LDO10_SLP3_MASK) |
			(0x2 << PM800_LDO10_SLP3_SHIFT);
		regmap_write(chip_g->subchip->regmap_power, 0x5e, data);
	}
}
EXPORT_SYMBOL(ldo10_sleepmode_control_for_wifi);

/* return gpadc voltage */
int get_gpadc_volt(struct pm80x_chip *chip, int gpadc_id)
{
	int ret, volt;
	unsigned char buf[2];
	int gp_meas;

	switch (gpadc_id) {
	case PM800_GPADC0:
		gp_meas = PM800_GPADC0_MEAS1;
		break;
	case PM800_GPADC1:
		gp_meas = PM800_GPADC1_MEAS1;
		break;
	case PM800_GPADC2:
		gp_meas = PM800_GPADC2_MEAS1;
		break;
	case PM800_GPADC3:
		gp_meas = PM800_GPADC3_MEAS1;
		break;
	default:
		dev_err(chip->dev, "get GPADC failed!\n");
		return -EINVAL;

	}
	ret = regmap_bulk_read(chip->subchip->regmap_gpadc,
			       gp_meas, buf, 2);
	if (ret < 0) {
		dev_err(chip->dev, "Attention: failed to get volt!\n");
		return -EINVAL;
	}

	volt = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	dev_dbg(chip->dev, "%s: volt value = 0x%x\n", __func__, volt);
	/* volt = value * 1.4 * 1000 / (2^12) */
	volt = ((volt & 0xfff) * 7 * 100) >> 11;
	dev_dbg(chip->dev, "%s: voltage = %dmV\n", __func__, volt);

	return volt;
}

/* return voltage via bias current from GPADC */
int get_gpadc_bias_volt(struct pm80x_chip *chip, int gpadc_id, int bias)
{
	int volt, data, gp_bias;

	switch (gpadc_id) {
	case PM800_GPADC0:
		gp_bias = PM800_GPADC_BIAS1;
		break;
	case PM800_GPADC1:
		gp_bias = PM800_GPADC_BIAS2;
		break;
	case PM800_GPADC2:
		gp_bias = PM800_GPADC_BIAS3;
		break;
	case PM800_GPADC3:
		gp_bias = PM800_GPADC_BIAS4;
		break;
	default:
		dev_err(chip->dev, "get GPADC failed!\n");
		return -EINVAL;

	}

	/* get the register value */
	if (bias > 76)
		bias = 76;
	if (bias < 1)
		bias = 1;
	bias = (bias - 1) / 5;

	regmap_read(chip->subchip->regmap_gpadc, gp_bias, &data);
	data &= 0xf0;
	data |= bias;
	regmap_write(chip->subchip->regmap_gpadc, gp_bias, data);

	volt = get_gpadc_volt(chip, gpadc_id);

	return volt;
}

/*
 * used by non-pmic driver
 * TODO: remove later
 */
int extern_get_gpadc_bias_volt(int gpadc_id, int bias)
{
	return get_gpadc_bias_volt(chip_g, gpadc_id, bias);
}
EXPORT_SYMBOL(extern_get_gpadc_bias_volt);

static struct i2c_driver pm800_driver = {
	.driver = {
		.name = "88PM800",
		.owner = THIS_MODULE,
		.pm = &pm80x_pm_ops,
		.of_match_table	= of_match_ptr(pm80x_dt_ids),
		},
	.probe = pm800_probe,
	.remove = pm800_remove,
	.id_table = pm80x_id_table,
};

static int __init pm800_i2c_init(void)
{
	return i2c_add_driver(&pm800_driver);
}
subsys_initcall(pm800_i2c_init);

static void __exit pm800_i2c_exit(void)
{
	i2c_del_driver(&pm800_driver);
}
module_exit(pm800_i2c_exit);

MODULE_DESCRIPTION("PMIC Driver for Marvell 88PM800");
MODULE_AUTHOR("Qiao Zhou <zhouqiao@marvell.com>");
MODULE_LICENSE("GPL");
