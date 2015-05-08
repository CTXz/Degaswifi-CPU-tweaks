/******************************************************************************
*(C) Copyright 2011 Marvell International Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as published
    by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/edge_wakeup_mmp.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <stddef.h>
#include <linux/platform_device.h>

enum {
	GE_DEBUG_DEBUG = 1U << 0,
	GE_DEBUG_INFO = 1U << 1,
	GE_DEBUG_ERROR = 1U << 2,
};

#define usim_event_debug(mask, x...) \
	do { \
		if (usim_debug_mask & mask) \
			dev_info(uedevice->dev, "usim event:\n" x); \
	} while (0)

static uint32_t usim_debug_mask = GE_DEBUG_INFO | GE_DEBUG_ERROR;
module_param_named(debug_mask, usim_debug_mask, uint, S_IWUSR | S_IRUGO);

static uint32_t usim_wakeup_timeout_in_ms = 3000;
module_param_named(wakeup_timeout_in_ms, usim_wakeup_timeout_in_ms, uint,
		   S_IWUSR | S_IRUGO);

static uint32_t usim_delay_time_in_jiffies = HZ;
module_param_named(delay_time_in_jiffies, usim_delay_time_in_jiffies, uint,
		   S_IWUSR | S_IRUGO);

static struct class *usim_event_class;

struct usim_event_device {
	char name[16];

	struct device *gpio_dev;
	struct device *dev;

	int enable;
	int state;

	int gpio;
	int irq;

	struct delayed_work work;
	struct workqueue_struct *wq;

	spinlock_t lock;
};

static struct usim_event_device *uedevice;

static void report_usim_event(struct usim_event_device *uedev, int state)
{
	char name_buf[50];
	char *env[3];

	snprintf(name_buf, sizeof(name_buf), "USIM_NAME=%s", uedev->name);

	env[0] = name_buf;
	env[1] = state ? "USIM_EVENT=plugin" : "USIM_EVENT=plugout";
	env[2] = NULL;

	kobject_uevent_env(&uedev->dev->kobj, KOBJ_CHANGE, env);
	usim_event_debug(GE_DEBUG_INFO, "%s: usim uevent [%s %s] is sent\n",
			 __func__, env[0], env[1]);
}

static void usim_event_work(struct work_struct *work)
{
	struct usim_event_device *uedev =
	    container_of(to_delayed_work(work), struct usim_event_device, work);
	int state = !!gpio_get_value(uedev->gpio);

	if (state != uedev->state) {
		uedev->state = state;
		report_usim_event(uedev, state);
	}
}

static void usim_event_wakeup(int gpio, void *data)
{
	struct usim_event_device *uedev = (struct usim_event_device *)data;
	pm_wakeup_event(uedev->dev, usim_wakeup_timeout_in_ms);
}

irqreturn_t usim_event_handler(int irq, void *dev_id)
{
	struct usim_event_device *uedev = (struct usim_event_device *)dev_id;
	unsigned long flags = 0;
	spin_lock_irqsave(&uedev->lock, flags);
	queue_delayed_work(uedev->wq, &uedev->work, usim_delay_time_in_jiffies);
	spin_unlock_irqrestore(&uedev->lock, flags);

	usim_event_debug(GE_DEBUG_INFO,
		"%s: gpio event irq received. irq[%d]\n", __func__, irq);
	return IRQ_HANDLED;
}

int usim_enable_interrupt(struct usim_event_device *uedev, int enable)
{
	int ret = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&uedev->lock, flags);
	if(enable != uedev->enable) {
		usim_event_debug(GE_DEBUG_INFO,
			 "%s: %s interrupt\n", __func__,
			 enable ? "enable" : "disable");
		if(enable) {
			ret = request_mfp_edge_wakeup(uedev->gpio,
					      usim_event_wakeup,
					      NULL, uedev->gpio_dev);
			if (ret) {
				usim_event_debug(GE_DEBUG_ERROR, "failed to request edge wakeup.\n");
			}
			enable_irq(uedev->irq);
		} else {
			disable_irq(uedev->irq);
			ret = remove_mfp_edge_wakeup(uedev->gpio);
			if (ret) {
				usim_event_debug(GE_DEBUG_ERROR, "%s: gpio edge del failed!\n",
					__func__);
			}
		}
		uedev->enable = enable;
	}

	spin_unlock_irqrestore(&uedev->lock, flags);

	return ret;
}

static ssize_t usim_send_event(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct usim_event_device *uedev =
	    (struct usim_event_device *)dev_get_drvdata(dev);

	unsigned long state;
	if (kstrtoul(buf, 10, &state)) {
		usim_event_debug(GE_DEBUG_ERROR, "%s: kstrtoul failed!\n",
					__func__);
		return -1;
	}
	report_usim_event(uedev, (int)state);
	return count;
}

static ssize_t usim_show_state(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct usim_event_device *uedev =
	    (struct usim_event_device *)dev_get_drvdata(dev);

	int len;
	len = sprintf(buf, "%d\n", uedev->state);
	return len;
}

static ssize_t usim_show_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct usim_event_device *uedev =
	    (struct usim_event_device *)dev_get_drvdata(dev);

	int len;
	len = sprintf(buf, "%d\n", uedev->enable);
	return len;
}

static ssize_t usim_enable(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct usim_event_device *uedev =
	    (struct usim_event_device *)dev_get_drvdata(dev);
	unsigned long enable;

	if (kstrtoul(buf, 10, &enable)) {
		usim_event_debug(GE_DEBUG_ERROR, "%s: kstrtoul failed!\n",
					__func__);
		return -1;
	}

	usim_enable_interrupt(uedev, (int)enable);
	return count;
}

static DEVICE_ATTR(send_event, 0222, NULL, usim_send_event);
static DEVICE_ATTR(state, 0444, usim_show_state, NULL);
static DEVICE_ATTR(enable, 0666, usim_show_enable, usim_enable);
static struct device_attribute *usim_event_attr[] = {
	&dev_attr_send_event,
	&dev_attr_state,
	&dev_attr_enable,
	NULL
};

static int usim_event_create_sys_device(struct device *dev)
{
	int ret = 0;
	struct device_attribute **attr = usim_event_attr;

	for (; *attr; ++attr) {
		ret = device_create_file(dev, *attr);
		if (ret)
			break;
	}

	if (ret) {
		for (--attr; attr >= usim_event_attr; --attr)
			device_remove_file(dev, *attr);
	}
	return 0;
}

static int usim_event_remove_sys_device(struct device *dev)
{
	struct device_attribute **attr = usim_event_attr;

	for (; *attr; ++attr)
		device_remove_file(dev, *attr);

	return 0;
}

static int usim_event_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = -1;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_sleep;	

	usim_event_class = class_create(THIS_MODULE, "usim_event");
	if (IS_ERR(usim_event_class))
		return PTR_ERR(usim_event_class);

	uedevice = kzalloc(sizeof(struct usim_event_device), GFP_KERNEL);

	if(uedevice == NULL)
	{
		usim_event_debug(GE_DEBUG_ERROR,
				 "%s:kmalloc error\n", __func__);
		goto out;
	}
	snprintf(uedevice->name, sizeof(uedevice->name) - 1, "usim%d", 0);
	spin_lock_init(&uedevice->lock);

	uedevice->gpio_dev = dev;
	uedevice->dev = device_create(usim_event_class, NULL,
				   MKDEV(0, 0), NULL, uedevice->name);
	if (IS_ERR(uedevice->dev)) {
		ret = PTR_ERR(uedevice->dev);
		goto out;
	}

	dev_set_drvdata(uedevice->dev, uedevice);

        of_property_read_u32(dev->of_node, "edge-wakeup-gpio", &uedevice->gpio);
	pr_info("###### haisheng uedevice->gpio = %d\n", uedevice->gpio);
	if (uedevice->gpio >= 0) {
		ret = request_mfp_edge_wakeup(uedevice->gpio,
					      usim_event_wakeup,
					      NULL, dev);
		if (ret) {
			usim_event_debug(GE_DEBUG_ERROR,
				"failed to request edge wakeup.\n");
			goto edge_wakeup;
		}
	}

	uedevice->state = !!gpio_get_value(uedevice->gpio);

	ret = usim_event_create_sys_device(uedevice->dev);
	if (ret < 0) {
		usim_event_debug(GE_DEBUG_ERROR,
				 "%s: create sys device failed!\n", __func__);
		goto destroy_device;
	}

	ret = gpio_request(uedevice->gpio, uedevice->name);

	gpio_direction_input(uedevice->gpio);

	uedevice->irq = gpio_to_irq(uedevice->gpio);

	ret =
	    request_irq(uedevice->irq, usim_event_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING |
			IRQF_NO_SUSPEND, uedevice->name,
			uedevice);
	if (ret < 0) {
		usim_event_debug(GE_DEBUG_ERROR, "%s: request irq failed!\n",
				 __func__);
		goto free_gpio;
	}

	INIT_DELAYED_WORK(&uedevice->work, usim_event_work);
	uedevice->wq = create_workqueue(uedevice->name);
	if (uedevice->wq == NULL) {
		usim_event_debug(GE_DEBUG_ERROR,
				 "%s:Can't create work queue!\n", __func__);
		ret = -ENOMEM;
		goto free_irq;
	}

	pr_info("%s: haisheng init OK\n", __FUNCTION__);
	return 0;

free_irq:
	free_irq(uedevice->irq, uedevice);
free_gpio:
	gpio_free(uedevice->gpio);
destroy_device:
	device_destroy(usim_event_class, MKDEV(0, 0));
edge_wakeup:
	if (uedevice->gpio >= 0)
		remove_mfp_edge_wakeup(uedevice->gpio);
out:
	return ret; 	
}

static int usim_event_remove(struct platform_device *pdev)
{
	if (uedevice->wq != NULL)
		destroy_workqueue(uedevice->wq);
	free_irq(uedevice->irq, uedevice);
	if (uedevice->gpio >= 0)
		remove_mfp_edge_wakeup(uedevice->gpio);
	gpio_free(uedevice->gpio);
	usim_event_remove_sys_device(uedevice->dev);
	device_destroy(usim_event_class, MKDEV(0, 0));
	if(uedevice != NULL)
	{
		kfree(uedevice);
		uedevice = NULL;
	}
	return 0;
}

static struct of_device_id usim_event_of_match[] = {
	{ .compatible = "usim_event", },
	{ },
};
MODULE_DEVICE_TABLE(of, usim_event_of_match);

static struct platform_driver  usim_event_device_driver = {
	.probe 		= usim_event_probe,
	.remove		= usim_event_remove,
	.driver		= {
		.name 	= "usim_event",
		.owner  = THIS_MODULE,
		.of_match_table	= of_match_ptr(usim_event_of_match),
	}
};

static int __init usim_event_init(void)
{
	return platform_driver_register(&usim_event_device_driver);
}

static void __exit usim_event_exit(void)
{
	platform_driver_unregister(&usim_event_device_driver);
}

late_initcall(usim_event_init);
module_exit(usim_event_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell USIM event notify");
