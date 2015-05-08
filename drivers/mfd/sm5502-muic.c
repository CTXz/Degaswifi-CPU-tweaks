/*
 * Copyright (c) 2013 Samsung Electronics Co, Ltd.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define DEBUG

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/gpio-pxa.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/pm_wakeup.h>
#include <plat/pm.h>
#include <mach/irqs.h>
#include <mach/sm5502-muic.h>

#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/battery/sec_charging_common.h>
#include <linux/sec-common.h>

static struct sm5502_usbsw {
	struct i2c_client *client;
	struct sm5502_platform_data *pdata;
	struct work_struct work;
	struct pm_qos_request qos_idle;
	struct switch_dev dock_dev;
	struct mutex mutex;
	u8 id;
	u8 dev1;
	u8 dev2;
	u8 dev3;
	u8 adc;
	u8 vbusin;
	u8 mansw;
};

static struct ic_vendor {
	u8 id;
	char *part_num;
};

static struct ic_vendor muic_list[] = {
	{0x01, "SM5502"},
};

static BLOCKING_NOTIFIER_HEAD(usb_switch_notifier);

static struct sm5502_usbsw *chip;
static struct wakeup_source jig_suspend_wake;

static int jig_wakelock_acq;
static int probing;
static int reset_count;
static int first_acce;

extern int jack_is_detected;

/**
 * usb_register_notify - register a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 */
int usb_switch_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&usb_switch_notifier, nb);
}

EXPORT_SYMBOL_GPL(usb_switch_register_notify);

/**
 * usb_unregister_notify - unregister a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 * usb_register_notify() must have been previously called for this function
 * to work properly.
 */
int usb_switch_unregister_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&usb_switch_notifier, nb);
}

EXPORT_SYMBOL_GPL(usb_switch_unregister_notify);

static ssize_t adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sm5502_usbsw *usbsw = chip;
	u8 adc_value[] = "1C";
	u8 adc_fail = 0;

	if (usbsw->dev2 & DEV_JIG_ALL) {
		pr_info("adc_show JIG_UART_OFF\n");
		return sprintf(buf, "%s\n", adc_value);
	} else {
		pr_info("adc_show no detect\n");
		return sprintf(buf, "%d\n", adc_fail);
	}
}

static DEVICE_ATTR(adc, S_IRUGO | S_IXOTH /*0665 */ ,
		   adc_show, NULL);

static int write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;
	u8 buf[2];
	struct i2c_msg msg[1];

	buf[0] = reg;
	buf[1] = data;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	dev_info(&client->dev, "I2C Write REG[0x%2x] DATA[0x%2x]\n", buf[0],
		 buf[1]);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "I2C Write Failed (ret=%d) \n", ret);
		return -EIO;
	}

	return ret;
}

static int read_reg(struct i2c_client *client, u8 reg, u8 * data)
{
	int ret = 0;
	u8 buf[1];
	struct i2c_msg msg[2];

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "I2C Read Failed REG[0x%2x] (ret=%d)\n",
			reg, ret);
		return -EIO;
	}
	*data = buf[0];

	dev_info(&client->dev, "I2C Read REG[0x%2x] DATA[0x%2x]\n", reg,
		 buf[0]);
	return 0;
}

int extert_muic_read_reg(u8 reg, u8 * data)
{
	int ret = 0;
	u8 buf[1];
	struct i2c_msg msg[2];
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "I2C Read Failed REG[0x%2x] (ret=%d)\n",
			reg, ret);
		return -EIO;
	}

	*data = buf[0];

	return 0;
}

EXPORT_SYMBOL(extert_muic_read_reg);

void manual_usbpath_ctrl(int on)
{
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 ctrl_reg, mansw1;

	read_reg(client, REG_MANSW1, &mansw1);
	read_reg(client, REG_CTRL, &ctrl_reg);

	mansw1 = on ? CON_TO_USB : AUTO_SWITCH;
	on ? (ctrl_reg &= ~MANUAL_SWITCH) : (ctrl_reg |= MANUAL_SWITCH);

	write_reg(client, REG_MANSW1, mansw1);
	write_reg(client, REG_CTRL, ctrl_reg);
}

void sm5502_dock_audiopath_ctrl(int on)
{
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 ctrl_reg, mansw1;

	read_reg(client, REG_MANSW1, &mansw1);
	read_reg(client, REG_CTRL, &ctrl_reg);

	mansw1 = on ? CON_TO_AUDIO : AUTO_SWITCH;
	on ? (ctrl_reg &= ~MANUAL_SWITCH) : (ctrl_reg |= MANUAL_SWITCH);

	/* Manual --> Auto case, Do not write REG_MANSW1 */
	if (on)
		write_reg(client, REG_MANSW1, mansw1);

	write_reg(client, REG_CTRL, ctrl_reg);
}

EXPORT_SYMBOL_GPL(sm5502_dock_audiopath_ctrl);

int sm5502_ic_reset(void)
{
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	u8 sintm1, sintm2, stime1, smansw1, sctrl, srsvid3;

	mutex_lock(&usbsw->mutex);
	disable_irq(client->irq);

	read_reg(client, REG_INT1_MASK, &sintm1);
	read_reg(client, REG_INT2_MASK, &sintm2);
	read_reg(client, REG_TIMING1, &stime1);
	read_reg(client, REG_MANSW1, &smansw1);
	read_reg(client, REG_CTRL, &sctrl);
	read_reg(client, REG_RSV_ID3, &srsvid3);

	write_reg(client, REG_RESET, IC_RESET);
	msleep(20);

	write_reg(client, REG_INT1_MASK, sintm1);
	write_reg(client, REG_INT2_MASK, sintm2);
	write_reg(client, REG_TIMING1, stime1);
	write_reg(client, REG_MANSW1, smansw1);
	write_reg(client, REG_RSV_ID3, srsvid3);
	write_reg(client, REG_CTRL, sctrl);

	dev_info(&client->dev, "SM5502 was reset!\n");

	enable_irq(client->irq);
	mutex_unlock(&usbsw->mutex);

	return 0;
}

EXPORT_SYMBOL_GPL(sm5502_ic_reset);

void sm5502_chgpump_ctrl(int enable)
{
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 ctrl_reg, chgpump;

	mutex_lock(&usbsw->mutex);
	read_reg(client, REG_CTRL, &ctrl_reg);
	read_reg(client, REG_RSV_ID3, &chgpump);

	/* If enabled is true
	 * Control Register(0x02) : RAW DATA(BIT3) = 0
	 * Reserved_ID Register(0x3A) : CHGPUMP_nEN(BIT0) = 0
	 * If enable is false
	 * Control Register(0x02) : RAW DATA(BIT3) = 1
	 * Reserved_ID Register(0x3A) : CHGPUMP_nEN(BIT0) = 1
	 */

	enable ? (ctrl_reg &= ~RAW_DATA) : (ctrl_reg |= RAW_DATA);
	enable ? (chgpump &= ~CHGPUMP_DIS) : (chgpump |= CHGPUMP_DIS);

	if (enable)
		pr_info("SM5502 CHG Pump Enable!\n");
	else
		pr_info("SM5502 CHG Pump Disable!\n");

	write_reg(client, REG_CTRL, ctrl_reg);
	write_reg(client, REG_RSV_ID3, chgpump);
	mutex_unlock(&usbsw->mutex);
}

EXPORT_SYMBOL_GPL(sm5502_chgpump_ctrl);

static ssize_t sm5502_set_syssleep(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sm5502_usbsw *usbsw = chip;

	if (!strncmp(buf, "1", 1)) {
		pm_qos_update_request(&usbsw->qos_idle,
				      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);
		__pm_relax(&jig_suspend_wake);
	}
	return count;
}

static ssize_t usb_state_show_attrs(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t usb_sel_show_attrs(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "PDA");
}

static ssize_t uart_sel_show_attrs(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t uart_sel_store_attrs(struct device *dev,
				    struct device_attribute *attr, char *buf,
				    size_t count)
{
	return count;
}

static DEVICE_ATTR(syssleep, (S_IWUSR | S_IWGRP), NULL, sm5502_set_syssleep);
static DEVICE_ATTR(usb_state, S_IRUGO, usb_state_show_attrs, NULL);
static DEVICE_ATTR(usb_sel, S_IRUGO, usb_sel_show_attrs, NULL);
static DEVICE_ATTR(uart_sel, S_IRUGO | S_IWUSR | S_IWGRP, uart_sel_show_attrs,
		   uart_sel_store_attrs);

static irqreturn_t microusb_irq_handler(int irq, void *data)
{
	struct sm5502_usbsw *usbsw = data;

	dev_info(&usbsw->client->dev, "%s\n", __func__);
	schedule_work(&usbsw->work);

	return IRQ_HANDLED;
}

static void additional_vbus_int_enable(struct sm5502_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	u8 int2m;

	read_reg(client, REG_INT2_MASK, &int2m);
	int2m &= ~(VBUSOUT_ON_M | VBUSOUT_OFF_M);
	write_reg(client, REG_INT2_MASK, int2m);

	dev_info(&client->dev, "Additiona VBUS Intr Enabled*****\n");
}

static void additional_vbus_int_disable(struct sm5502_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	u8 int2m;

	read_reg(client, REG_INT2_MASK, &int2m);
	int2m |= (VBUSOUT_ON_M | VBUSOUT_OFF_M);
	write_reg(client, REG_INT2_MASK, int2m);

	dev_info(&client->dev, "Additiona VBUS Intr Disabled*****\n");
}

static int sm5502_reg_init(struct sm5502_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	int i, ret;

	read_reg(client, REG_DEVID, &usbsw->id);
	for (i = 0; i < ARRAY_SIZE(muic_list); i++) {
		if (usbsw->id == muic_list[i].id)
			dev_info(&client->dev, "PartNum : %s\n",
				 muic_list[i].part_num);
	}

	/* INT MASK1, 2 */
	ret = write_reg(client, REG_INT1_MASK, INTMASK1_INIT);
	if (ret < 0)
		return ret;

	ret = write_reg(client, REG_INT2_MASK, INTMASK2_INIT);
	if (ret < 0)
		return ret;

	/*Set Timing1 to 200ms */
	ret = write_reg(client, REG_TIMING1, ADC_DET_T200);
	if (ret < 0)
		return ret;

	/* CONTROL REG */
	ret = write_reg(client, REG_CTRL, CTRL_INIT);
	if (ret < 0)
		return ret;

	return 0;
}

 /* microUSB switch IC : SM5502 - Silicon Mitus */
static void detect_dev_sm5502(struct sm5502_usbsw *usbsw, u8 intr1, u8 intr2,
			      void *data)
{
	struct sm5502_platform_data *pdata = usbsw->pdata;
	struct i2c_client *client = usbsw->client;
	u8 val1, val2, val3, adc, vbusin, intr1_tmp, val;
	int dev_classifi = 0;

	read_reg(client, REG_DEV_T1, &val1);
	read_reg(client, REG_DEV_T2, &val2);
	read_reg(client, REG_DEV_T3, &val3);
	read_reg(client, REG_ADC, &adc);
	read_reg(client, REG_RSV_ID1, &vbusin);

	/* IC Bug Case W/A */
	if (intr1 & OVP_EVENT_M) {
		read_reg(client, REG_CTRL, &val);
		if (val == 0x1F) {
			sm5502_reg_init(usbsw);
			return;
		}
	}
	/* Detach -> Attach quickly */
	if (intr1 == (ATTACHED | DETACHED)) {
		dev_info(&client->dev, "Bug Case 1\n");
		intr1 &= ~(DETACHED);
	}
	/* Attach -> Detach quickly */
	else if (intr1 & ATTACHED && probing != 1) {
		read_reg(client, REG_INT1, &intr1_tmp);
		if (intr1_tmp & DETACHED) {
			dev_info(&client->dev, "Bug Case 2\n");
			intr1 &= ~(ATTACHED);
		}
		intr1 |= intr1_tmp;
	}

	/* Attached */
	if (intr1 & ATTACHED
	    || (intr2 & (VBUSOUT_ON | VBUSOUT_OFF) && !(intr1 & DETACHED))
	    || intr2 & REV_ACCE) {
		if (val1 & DEV_USB && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE1_USB_MUIC;
			dev_info(&client->dev, "USB ATTACHED*****\n");
		}
		if (val1 & DEV_CHARGER && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE1_TA_MUIC;
			dev_info(&client->dev, "TA(DCP/CDP) ATTACHED*****\n");
		}
		if (val1 & DEV_USB_OTG) {
			dev_classifi = CABLE_TYPE1_OTG_MUIC;
			dev_info(&client->dev, "OTG ATTACHED*****\n");
		}
		if (val1 & DEV_CARKIT_CHG && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE1_CARKIT_T1OR2_MUIC;
			manual_usbpath_ctrl(1);
			dev_info(&client->dev,
				 "CARKIT or L USB Cable ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_UART_OFF) {
			if (vbusin & VBUSIN_VALID) {
				dev_classifi = CABLE_TYPE2_JIG_UART_OFF_VB_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTOFF_VB ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_TYPE2_JIG_UART_OFF_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTOFF ATTACHED*****\n");
			}
			additional_vbus_int_enable(usbsw);
		}
		if (val2 & DEV_JIG_UART_ON) {
			if (vbusin & VBUSIN_VALID) {
				dev_classifi = CABLE_TYPE2_JIG_UART_ON_VB_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTON_VB ATTACHED*****\n");

			} else {
				dev_classifi = CABLE_TYPE2_JIG_UART_ON_MUIC;
				dev_info(&client->dev,
					 "JIG_UARTON ATTACHED*****\n");
			}
			additional_vbus_int_enable(usbsw);
		}
		if (val2 & DEV_JIG_USB_OFF && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE2_JIG_USB_OFF_MUIC;
			dev_info(&client->dev, "JIG_USB_OFF ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_USB_ON && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE2_JIG_USB_ON_MUIC;
			dev_info(&client->dev, "JIG_USB_ON ATTACHED*****\n");
		}
		if (val2 & DEV_JIG_WAKEUP) {
			if (!jig_wakelock_acq) {
				__pm_stay_awake(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						pdata->qos_val);

				jig_wakelock_acq = 1;
				dev_info(&client->dev,
					 "AP WakeLock for FactoryTest *****\n");
			}
		}
		/* Desktop Dock Case */
		if (val2 & DEV_AV) {
			/* Check device3 register for Dock+VBUS */
			if (val3 & DEV_AV_VBUS && vbusin & VBUSIN_VALID) {
				dev_classifi = CABLE_TYPE3_DESKDOCK_VB_MUIC;
				dev_info(&client->dev,
					 "DESKDOCK+VBUS ATTACHED*****\n");
			} else {
				dev_classifi = CABLE_TYPE2_DESKDOCK_MUIC;
				dev_info(&client->dev,
					 "DESKDOCK ATTACHED*****\n");
			}
			additional_vbus_int_enable(usbsw);
			/* Dock */
			switch_set_state(&usbsw->dock_dev, 1);
			if (jack_is_detected)
				sm5502_dock_audiopath_ctrl(0);
			else
				sm5502_dock_audiopath_ctrl(1);
		}
		if (val3 & DEV_U200_CHG && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE3_U200CHG_MUIC;
			dev_info(&client->dev, "TA(U200 CHG) ATTACHED*****\n");
		}
		if (val3 & DEV_DCD_OUT_SDP && vbusin & VBUSIN_VALID) {
			dev_classifi = CABLE_TYPE3_NONSTD_SDP_MUIC;
			dev_info(&client->dev,
				 "TA(NON-STANDARD SDP) ATTACHED*****\n");
		}
		/* W/A */
		if (val1 == 0 && val2 == 0 && val3 == 0
		    && reset_count < MAX_RESET_TRIAL && probing != 1) {

			u8 sintm1, sintm2, sctrl, stime1, smansw1;

			read_reg(client, REG_INT1_MASK, &sintm1);
			read_reg(client, REG_INT2_MASK, &sintm2);
			read_reg(client, REG_TIMING1, &stime1);
			read_reg(client, REG_MANSW1, &smansw1);
			read_reg(client, REG_CTRL, &sctrl);

			write_reg(client, REG_RESET, IC_RESET);
			msleep(20);
			dev_info(&client->dev,
				 "SM5502 was reset, reset_count : %d\n",
				 reset_count);

			write_reg(client, REG_INT1_MASK, sintm1);
			write_reg(client, REG_INT2_MASK, sintm2);
			write_reg(client, REG_TIMING1, stime1);
			write_reg(client, REG_MANSW1, smansw1);
			write_reg(client, REG_CTRL, sctrl);

			reset_count++;

			return;
		}
		/* for Charger driver */
		if (pdata->charger_cb)
			pdata->charger_cb(dev_classifi);
		if (probing == 1)
			*(int *)data = dev_classifi;
		blocking_notifier_call_chain(&usb_switch_notifier, dev_classifi,
					     NULL);
	}

	/* Detached */
	if (intr1 & DETACHED) {
		if (usbsw->dev1 & DEV_USB && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev, "USB DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CHARGER && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev, "TA(DCP/CDP) DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_USB_OTG) {
			dev_info(&client->dev, "OTG DETACHED*****\n");
		}
		if (usbsw->dev1 & DEV_CARKIT_CHG
		    && usbsw->vbusin & VBUSIN_VALID) {
			manual_usbpath_ctrl(0);
			dev_info(&client->dev,
				 "CARKIT or L USB Cable DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_UART_OFF) {
			if (usbsw->vbusin & VBUSIN_VALID) {
				dev_info(&client->dev,
					 "JIG_UARTOFF+VBUS DETACHED*****\n");
			} else {
				dev_info(&client->dev,
					 "JIG_UARTOFF DETACHED*****\n");
			}
			additional_vbus_int_disable(usbsw);
		}
		if (usbsw->dev2 & DEV_JIG_UART_ON) {
			if (usbsw->vbusin & VBUSIN_VALID) {
				dev_info(&client->dev,
					 "JIG_UARTON_VB DETACHED*****\n");
			} else {
				dev_info(&client->dev,
					 "JIG_UARTON DETACHED*****\n");
			}
			additional_vbus_int_disable(usbsw);
		}
		if (usbsw->dev2 & DEV_JIG_USB_OFF
		    && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev, "JIG_USB_OFF DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_USB_ON
		    && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev, "JIG_USB_ON DETACHED*****\n");
		}
		if (usbsw->dev2 & DEV_JIG_WAKEUP) {
			if (jig_wakelock_acq) {
				__pm_relax(&jig_suspend_wake);
				pm_qos_update_request(&usbsw->qos_idle,
						      PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

				jig_wakelock_acq = 0;
				dev_info(&client->dev,
					 "AP WakeLock Release *****\n");
			}
		}
		if (usbsw->dev2 & DEV_AV) {
			/* Check device3 register for Dock+VBUS */
			if (usbsw->dev3 & DEV_AV_VBUS
			    && usbsw->vbusin & VBUSIN_VALID) {
				dev_info(&client->dev,
					 "DESKDOCK+VBUS DETTACHED*****\n");
			} else {
				dev_info(&client->dev,
					 "DESKDOCK DETACHED*****\n");
			}
			additional_vbus_int_disable(usbsw);
			/* Dock */
			switch_set_state(&usbsw->dock_dev, 0);
			sm5502_dock_audiopath_ctrl(0);
		}
		if (usbsw->dev3 & DEV_U200_CHG && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev, "TA(U200_CHG) DETTACHED*****\n");
		}
		if (usbsw->dev3 & DEV_DCD_OUT_SDP
		    && usbsw->vbusin & VBUSIN_VALID) {
			dev_info(&client->dev,
				 "TA(NON-STANDARD SDP) DETACHED*****\n");
		}
		/* for Charger driver */
		if (pdata->charger_cb)
			pdata->charger_cb(CABLE_TYPE_NONE_MUIC);
		blocking_notifier_call_chain(&usb_switch_notifier,
					     CABLE_TYPE_NONE_MUIC, NULL);
		reset_count = 0;
	}

	usbsw->dev1 = val1;
	usbsw->dev2 = val2;
	usbsw->dev3 = val3;
	usbsw->adc = adc;
	usbsw->vbusin = vbusin;

	return;
}

void muic_attached_accessory_inquire(void)
{
	struct sm5502_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	dev_info(&client->dev, "%s\n", __func__);
	blocking_notifier_call_chain(&usb_switch_notifier, first_acce, NULL);
}

EXPORT_SYMBOL_GPL(muic_attached_accessory_inquire);

static void sm5502_work_cb(struct work_struct *work)
{
	u8 intr1, intr2;

	struct sm5502_usbsw *usbsw =
	    container_of(work, struct sm5502_usbsw, work);
	struct i2c_client *client = usbsw->client;

	msleep(50);

	mutex_lock(&usbsw->mutex);

	disable_irq(client->irq);

	/* Read and Clear Interrupt1/2 */
	read_reg(client, REG_INT1, &intr1);
	read_reg(client, REG_INT2, &intr2);

	detect_dev_sm5502(usbsw, intr1, intr2, NULL);

	enable_irq(client->irq);

	mutex_unlock(&usbsw->mutex);
}

static int sm5502_int_init(struct device_node *np, struct sm5502_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;
	int ret;
	u8 intr1, intr2, val;
	int irq;

	INIT_WORK(&usbsw->work, sm5502_work_cb);

	irq = of_get_named_gpio(np, "connector-gpio", 0);
	if (irq < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__, irq);
		return irq;
	}

	client->irq = gpio_to_irq(irq);
	ret = request_irq(client->irq, microusb_irq_handler, IRQF_NO_SUSPEND | IRQF_TRIGGER_FALLING, "sm5502 micro USB", usbsw);
	if (ret) {
		dev_err(&client->dev, "Unable to get IRQ %d\n", client->irq);
		return ret;
	}

	/* Read and Clear INTERRUPT1,2 REGS */
	mutex_lock(&usbsw->mutex);
	read_reg(client, REG_INT1, &intr1);
	read_reg(client, REG_INT2, &intr2);
	mutex_unlock(&usbsw->mutex);

	if (intr1 & OVP_EVENT_M) {
		mutex_lock(&usbsw->mutex);
		read_reg(client, REG_CTRL, &val);
		mutex_unlock(&usbsw->mutex);
		if (val == 0x1F) {
			mutex_lock(&usbsw->mutex);
			sm5502_reg_init(usbsw);
			mutex_unlock(&usbsw->mutex);
			return;
		}
	}

	if ((usbsw->dev1 != 0 || usbsw->dev2 != 0 || usbsw->dev3 != 0)
	    && (intr1 == 0 && intr2 == 0)) {
		dev_err(&client->dev,
			"Accs inserted but no data on int regs\n");
	}

	return 0;

}

static const struct of_device_id sm5502_dt_ids[] = {
	{.compatible = "samsung,sm5502",},
	{}
};

MODULE_DEVICE_TABLE(of, sec_charger_dt_ids);

static int sm5502_probe_dt(struct device_node *np,
			   struct device *dev,
			   struct sm5502_platform_data *pdata)
{
	int tsp_intr_n, keyled_n;
	int ret = 0;
	int connector_gpio = 0;
	const struct of_device_id *match;
	u32 lpm;

	if (!np)
		return -EINVAL;

	match = of_match_device(sm5502_dt_ids, dev);
	if (!match)
		return -EINVAL;

	connector_gpio = of_get_named_gpio(np, "connector-gpio", 0);
	if (connector_gpio < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
		       connector_gpio);
		return connector_gpio;
	}
	ret = gpio_request(connector_gpio, "connector-gpio");
	if (ret) {
		pr_err("%s:%d gpio_request failed: connector_gpio %d\n",
		       __func__, __LINE__, connector_gpio);
		return ret;
	}

	gpio_direction_input(connector_gpio);

	if (!of_property_read_u32(np, "lpm-qos", &lpm))
		pdata->qos_val = lpm;
	else {
		pr_err("SM5502: failed to get 'lpm-qos' from dt\n");
		return -EINVAL;
	}

	return 0;
}

static struct sm5502_platform_data sm5502_info = {
	.charger_cb = sec_charger_cb,
};

static int sm5502_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sm5502_usbsw *usbsw;
	struct device *switch_dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct sm5502_platform_data *pdata = client->dev.platform_data;
	struct device_node *np = client->dev.of_node;
	int ret = 0;

	dev_info(&client->dev, "probe start\n");
	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata)
			pdata = &sm5502_info;

		ret = sm5502_probe_dt(np, &client->dev, pdata);
		if (ret)
			return ret;
	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}

	probing = 1;

	/* For AT Command FactoryTest */
	wakeup_source_init(&jig_suspend_wake, "JIG_UART Connect suspend wake");

	usbsw = kzalloc(sizeof(struct sm5502_usbsw), GFP_KERNEL);
	if (!usbsw) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	chip = usbsw;
	usbsw->client = client;
	usbsw->pdata = pdata;

	i2c_set_clientdata(client, usbsw);

	mutex_init(&usbsw->mutex);

	/* DeskTop Dock  */
	usbsw->dock_dev.name = "dock";
	ret = switch_dev_register(&usbsw->dock_dev);
	if (ret < 0)
		dev_err(&client->dev, "dock_dev_register error !!\n");

	if (!sec_class) {
		sec_class = class_create(THIS_MODULE, "sec");
	}
	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	if (device_create_file(switch_dev, &dev_attr_adc) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_adc.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_state.attr.name);
	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_usb_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_uart_sel) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_uart_sel.attr.name);
	if (device_create_file(switch_dev, &dev_attr_syssleep) < 0)
		dev_err(&client->dev, "Failed to create device file(%s)!\n",
			dev_attr_syssleep.attr.name);
	dev_set_drvdata(switch_dev, usbsw);

	usbsw->qos_idle.name = "Jig driver";
	pm_qos_add_request(&usbsw->qos_idle, PM_QOS_CPUIDLE_BLOCK,
			   PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE);

	ret = sm5502_reg_init(usbsw);
	if (ret)
		goto sm5502_probe_fail;

	/* device detection */
	dev_info(&client->dev, "First Detection\n");
	detect_dev_sm5502(usbsw, ATTACHED, REV_ACCE, &first_acce);

	ret = sm5502_int_init(np, usbsw);
	if (ret)
		goto sm5502_probe_fail;

	probing = 0;
	dev_info(&client->dev, "PROBE Done.\n");

	return 0;

sm5502_probe_fail:
	device_destroy(sec_class, 0);
	i2c_set_clientdata(client, NULL);
	kfree(usbsw);
	return ret;
}

static int sm5502_remove(struct i2c_client *client)
{
	struct sm5502_usbsw *usbsw = i2c_get_clientdata(client);
	if (client->irq)
		free_irq(client->irq, NULL);
	i2c_set_clientdata(client, NULL);

	pm_qos_remove_request(&usbsw->qos_idle);

	kfree(usbsw);
	return 0;
}

static int sm5502_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int sm5502_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sm5502_id[] = {
	{"sm5502", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sm5502_id);

static struct i2c_driver sm5502_i2c_driver = {
	.driver = {
		   .name = "sm5502",
		   .of_match_table = sm5502_dt_ids,
		   },
	.probe = sm5502_probe,
	.remove = sm5502_remove,
	.suspend = sm5502_suspend,
	.resume = sm5502_resume,
	.id_table = sm5502_id,
};

static int __init sm5502_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&sm5502_i2c_driver);
}

module_init(sm5502_init);

static void __exit sm5502_exit(void)
{
	i2c_del_driver(&sm5502_i2c_driver);
}

module_exit(sm5502_exit);

MODULE_LICENSE("GPL");
