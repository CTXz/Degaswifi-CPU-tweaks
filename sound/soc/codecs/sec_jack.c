/*
 *	drivers/switch/sec_jack.c
 *
 *	headset & hook detect driver for pm822 (SAMSUNG COMSTOM KSND)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/88pm80x.h>
#include <linux/mfd/88pm8xxx-headset.h>
#include <linux/switch.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mfd/sec_jack.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/timer.h> //KSND
#include <linux/time.h>  //KSND
#if defined(SEC_USE_ANLOGDOCK_DEVICE)
#include <mach/sm5502-muic.h>       /* Use Docking device for external audio */
#endif

#define PM800_GPADC_AVG1(x)			(0xA0 + 2*(x))
#define PM800_GPADC_LOW_TH(x)		(0x20 + x)
#define PM800_GPADC_UPP_TH(x)		(0x30 + x)

#define PM822_HEADSET_DET_MASK		(1 << 7)

#define PM800_HS_DET_INVERT			1

#define PM800_INT_ENA_3				(0x0B)
#define PM800_INT_ENA_4				(0x0C)
#define PM800_GPIO_INT_ENA4(x)		(1 << (x))
#define PM800_GPADC_INT_ENA3(x)		(1 << (x))
#define PM800_GPADC4_DIR			(1 << 6)

#define PM822_GPADC_MEAS_EN2		(0x02)

#define PM800_MIC_CNTRL			(0x39)
#define PM800_MICDET_EN			(1 << 0)

#define PM822_MIC_DET_MEAS_EN	(1 << 6)

#define MIC_DET_DBS				(3 << 1)
#define MIC_DET_DBS_32MS		(3 << 1)
#define MIC_DET_PRD				(3 << 3)
#define MIC_DET_PRD_256MS		(1 << 3)
#define MIC_DET_PRD_512MS		(2 << 3)
#define MIC_DET_PRD_CTN			(3 << 3)

#define MAX_ZONE_LIMIT 			10

/* Headset detection info */
struct pm800_headset_info {
	struct pm80x_chip *chip;
	struct device *dev;
	struct regmap *map;
	struct regmap *map_gpadc;

	int irq_headset;
	int irq_hook;

	struct work_struct work_headset, work_release, work_init;
	//struct work_struct work_hook;
	//struct headset_switch_data *psw_data_headset;
	//struct sec_jack_platform_data *pdata;
	struct regulator *mic_bias;

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	void (*dock_audiopath_ctrl)(int on);
	void (*chgpump_ctrl)(int enable);
	int (*usb_switch_register_notify)(struct notifier_block *nb);
	int (*usb_switch_unregister_notify)(struct notifier_block *nb);
#endif

	/* Headset Data */
	int headset_flag;
	int press_release_th;
	struct sec_jack_zone			*jack_zones;
	struct sec_jack_buttons_zone	*buttons_zones;
	int	num_jack_zones;
	int	num_buttons_zones;
#ifdef SAMSUNG_JACK_SW_WATERPROOF
	int ear_reselector_zone;
#endif

	struct snd_soc_jack *hs_jack, *hk_jack;

	u32 hook_count;
	int hook_vol_status;

	int hs_status, hk_status, state;
	int headset_gpio_num;
	int mic_gpadc_num;

	struct mutex hs_mutex;

	struct device *hsdetect_dev;

	struct timer_list headset_timer;
	struct timer_list hook_timer;

	struct timespec ts; 		/* Get Current time for KSND */
	struct timespec ts_after;	/* Get Current time After Event */
};


static struct pm800_headset_info *info;
#ifdef SAMSUNG_JACK_SW_WATERPROOF
static bool recheck_jack;
#endif

static char *sec_jack_status[] = {
	"NONE",
	"HEADSET_4POLE",
	"HEADSET_3POLE",
};

static char *sec_button_status[] = {
	"Hook",
	"Vol_Up",
	"Vol_Down",
};

static void pm800_jack_buttons_work(struct pm800_headset_info *info);

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
/* Use Docking device for external audio */
static int dock_state;
int jack_is_detected = 0;
EXPORT_SYMBOL(jack_is_detected);

static int dock_notify(struct notifier_block *self, unsigned long action, void *dev)
{
	dock_state = action;
	return NOTIFY_OK;
}

static struct notifier_block dock_nb = {
  .notifier_call = dock_notify,
};
#endif /* SEC_USE_ANLOGDOCK_DEVICE */

/*
 * separate hs & hk, since snd_jack_report reports both key & switch. just to
 * avoid confusion in upper layer. when reporting hs, only headset/headphone/
 * mic status are reported. when reporting hk, hook/vol up/vol down status are
 * reported.
 */
int pm800_headset_detect(struct snd_soc_jack *jack)
{
	if (!info)
		return -EINVAL;

	/* Store the configuration */
	info->hs_jack = jack;

	/* detect headset status during boot up */
	queue_work(system_wq, &info->work_init);

	return 0;
}
EXPORT_SYMBOL_GPL(pm800_headset_detect);

int pm800_hook_detect(struct snd_soc_jack *jack)
{
	if (!info)
		return -EINVAL;

	/* Store the configuration */
	info->hk_jack = jack;

	return 0;
}
EXPORT_SYMBOL_GPL(pm800_hook_detect);

static int gpadc_measure_voltage( struct pm800_headset_info *info )
{
	unsigned char buf[2];
	int sum = 0, ret = -1;

	ret = regmap_raw_read(info->map_gpadc,PM800_GPADC_AVG1(info->mic_gpadc_num), buf, 2);

	if (ret < 0){
	  pr_debug("%s: Fail to get the voltage!! \n", __func__);
	  return 0;
	}

	/* GPADC4_dir = 1, measure(mv) = value *1.4 *1000/(2^12) */
	sum = ((buf[0] & 0xFF) << 4) | (buf[1] & 0x0F);
	sum = ((sum & 0xFFF) * 1400) >> 12;
	//pr_debug("the voltage is %d mv\n", sum);
	return sum;
}

static void gpadc_set_threshold(struct pm800_headset_info *info, int min,int max)
{
	unsigned int data;

	if (min <= 0){
		data = 0;
	} else {
		data = ((min << 12) / 1400) >> 4;
	}
	regmap_write(info->map_gpadc, PM800_GPADC_LOW_TH(info->mic_gpadc_num), data);

	if (max <= 0){
		data = 0xFF;
	} else {
		data = ((max << 12) / 1400) >> 4;
	}
	regmap_write(info->map_gpadc, PM800_GPADC_UPP_TH(info->mic_gpadc_num), data);
}

static void mic_set_power(int on, struct pm800_headset_info *info)
{
	static int mic_power_flag = 0;

	int ret = 0;

	if (on && (!mic_power_flag)){
		ret = regulator_enable(info->mic_bias);
		mic_power_flag = 1;
	}

	if (mic_power_flag && (!on)) {
		ret = regulator_disable(info->mic_bias);
		mic_power_flag = 0;
	}

	if (ret) {
		dev_err(info->dev, "failed to enable/disable regulator: %d\n", ret);
		return;
	}
}

static int get_mic_gpadc_num(struct resource *resource)
{
	if (strcmp(resource->name, "gpadc4") == 0)
		return 4;
	else if (strcmp(resource->name, "gpadc3") == 0)
		return 3;
	else if (strcmp(resource->name, "gpadc2") == 0)
		return 2;
	else if (strcmp(resource->name, "gpadc1") == 0)
		return 1;
	else
		return 0;
}

static int get_headset_gpio_num(struct resource *resource)
{
	if (strcmp(resource->name, "gpio-04") == 0)
		return 4;
	else if (strcmp(resource->name, "gpio-03") == 0)
		return 3;
	else if (strcmp(resource->name, "gpio-02") == 0)
		return 2;
	else if (strcmp(resource->name, "gpio-01") == 0)
		return 1;
	else
		return 0;
}

static void pm800_hook_int(struct pm800_headset_info *info, int enable)
{
	mutex_lock(&info->hs_mutex);
	if (enable && info->hook_count == 0) {
		enable_irq(info->irq_hook);
		info->hook_count = 1;
	} else if (!enable && info->hook_count == 1) {
		disable_irq(info->irq_hook);
		info->hook_count = 0;
	}
	mutex_unlock(&info->hs_mutex);
}

static void pm800_button_handle(struct pm800_headset_info *info, int voltage)
{
	struct sec_jack_buttons_zone *btn_zones = info->buttons_zones;
	int report = 0;
	int i, button;
	int mask = SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2;

	if (voltage < info->press_release_th) {
		/* press event */
		if( info->hook_vol_status > HOOK_VOL_ALL_RELEASED ){
			/* No Need to sent the event */
			return;
		}

		for(i = 0; i < info->num_buttons_zones; i++){
			if (voltage >= btn_zones[i].adc_low && voltage <= btn_zones[i].adc_high){
				info->hk_status = btn_zones[i].code;

				if( info->hk_status == SND_JACK_BTN_0 ){
					report = SND_JACK_BTN_0;
					info->hook_vol_status = HOOK_PRESSED;
					button = 0;
					/* for telephony */
					kobject_uevent(&info->hsdetect_dev->kobj, KOBJ_ONLINE);
				} else if( info->hk_status == SND_JACK_BTN_1 ){
					report = SND_JACK_BTN_1;
					info->hook_vol_status = VOL_UP_PRESSED;
					button = 1;
				} else {
					report = SND_JACK_BTN_2;
					info->hook_vol_status = VOL_DOWN_PRESSED;
					button = 2;
				}

				snd_soc_jack_report(info->hk_jack, report, mask);
				info->hk_status = report;
				gpadc_set_threshold(info, 0, info->press_release_th );

				pr_info("%s: [%s] is pressed![%d mV]\n", __func__, sec_button_status[button], voltage);
				return;
			}
		}
		pr_warn("%s: key is skipped. [%d mv]\n", __func__, voltage);
	}
	else {
		/* release event */
		if(info->hook_vol_status <= HOOK_VOL_ALL_RELEASED){
			/* No Need to sent the event */
			return;
		}

		if( info->hk_status == SND_JACK_BTN_0 ){
			info->hk_jack->jack->type = SND_JACK_BTN_0;
			button = 0;
			/* for telephony */
			kobject_uevent(&info->hsdetect_dev->kobj, KOBJ_ONLINE);
			info->hook_vol_status = HOOK_RELEASED;

		} else if( info->hk_status== SND_JACK_BTN_1 ){
			info->hk_jack->jack->type = SND_JACK_BTN_1;
			info->hook_vol_status = VOL_UP_RELEASED;
			button = 1;
		} else {
			info->hk_jack->jack->type = SND_JACK_BTN_2;
			info->hook_vol_status = VOL_DOWN_RELEASED;
			button = 2;
		}

		snd_soc_jack_report(info->hk_jack, report, mask);
		info->hk_jack->jack->type = mask;
		info->hk_status = report;
		gpadc_set_threshold(info, info->press_release_th, 0);

		pr_info("%s: [%s] is released![%d mV]\n", __func__, sec_button_status[button], voltage);
	}
}

static void pm800_jack_set_type(struct pm800_headset_info *info, int jack_type )
{
	int old_state = SEC_JACK_NO_DEVICE;
	int report = 0;

	/* Backup the old switch state */
	old_state = info->state;

	switch( jack_type ){
		case SEC_HEADSET_4POLE:
			report = SND_JACK_HEADSET;
			info->state = SEC_HEADSET_4POLE;
			/* for telephony */
			kobject_uevent(&info->hsdetect_dev->kobj, KOBJ_ADD);
			gpadc_set_threshold(info, info->press_release_th, 0);
			/* enable hook irq if headset is 4 pole */
			pm800_hook_int(info, 1);
			break;

		case SEC_HEADSET_3POLE:
			report = SND_JACK_HEADPHONE;
			info->state = SEC_HEADSET_3POLE;
			/* we already disable mic power when it is headphone */
			if(info->mic_bias){
				mic_set_power(0, info);
			}
			/* disable MIC detection and measurement */
			regmap_update_bits(info->map, PM800_MIC_CNTRL, PM800_MICDET_EN, 0);
			regmap_update_bits(info->map_gpadc, PM822_GPADC_MEAS_EN2, PM822_MIC_DET_MEAS_EN, 0);
			break;

		case SEC_JACK_NO_DEVICE:
			info->state = SEC_JACK_NO_DEVICE;
			info->hook_vol_status = HOOK_VOL_ALL_RELEASED;
			/* for telephony */
			kobject_uevent(&info->hsdetect_dev->kobj, KOBJ_REMOVE);

			/* we already disable mic power when it is headphone */
			if ( info->mic_bias){
				mic_set_power(0, info);
			}

			/* disable MIC detection and measurement */
			regmap_update_bits(info->map, PM800_MIC_CNTRL, PM800_MICDET_EN, 0);
			regmap_update_bits(info->map_gpadc, PM822_GPADC_MEAS_EN2,PM822_MIC_DET_MEAS_EN, 0);

			/* disable the hook irq */
			regmap_update_bits(info->map, PM800_INT_ENA_3,  PM800_GPADC_INT_ENA3(info->mic_gpadc_num), 0);
			break;

		case SEC_UNKNOWN_DEVICE:
			/* No excute */
			pr_info("%s: jack:SEC_UNKNOWN_DEVICE!\n",__func__);
			return;
			
		default:
			pr_debug("Invalid sec jack type!\n");
			return;

	}

	info->hs_status = report;
	info->hk_status = report;

	switch_set_state(&switch_jack_detection, jack_type);
	snd_soc_jack_report(info->hs_jack, report, SND_JACK_HEADSET);
	pr_info("%s: %s from %s\n",__func__ , sec_jack_status[info->state], sec_jack_status[old_state] );

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	/* Use Docking device for external audio */
	jack_is_detected = info->state;
#endif

	/* Estimate Headset detection time */  //KSND
	if( jack_type == SEC_HEADSET_4POLE || jack_type == SEC_HEADSET_3POLE ) {
		info->ts_after = current_kernel_time();
		info->ts.tv_nsec = info->ts_after.tv_nsec - info->ts.tv_nsec;
		if( info->ts.tv_nsec < 0) {
			--info->ts.tv_sec;
			info->ts.tv_nsec += 1000000000L;
		}
		pr_info("%s: detect time : %d ms\n", __func__, (int)info->ts.tv_nsec/1000000 );
	}
}

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
static void pm800_audio_dock_ctrl(struct pm800_headset_info *info, int enable)
{
	if( info->chgpump_ctrl == NULL || info->dock_audiopath_ctrl == NULL ){
		pr_debug("Invalid audio dock interface pointer!\n");
		return;
	}

	/* Use Docking device for external audio */
	if ( enable ){
		/* Output through the Dock device */
		info->chgpump_ctrl(0);
		if( dock_state == CABLE_TYPE2_DESKDOCK_MUIC 
		 || dock_state == CABLE_TYPE3_DESKDOCK_VB_MUIC){
			info->dock_audiopath_ctrl(1);
		}
	} else {
		/* Stop the path of Dock device */
		info->chgpump_ctrl(1);
		if( dock_state == CABLE_TYPE2_DESKDOCK_MUIC
		 || dock_state == CABLE_TYPE3_DESKDOCK_VB_MUIC){
			info->dock_audiopath_ctrl(0);
		}
	}
}
#endif

static void pm800_jack_timer_handler(unsigned long data)
{
	struct pm800_headset_info *info = (struct pm800_headset_info *)data;

	if (info != NULL) {
		queue_work(system_wq, &info->work_headset);
	}	
}

static void pm800_button_timer_handler(unsigned long data)
{
	struct pm800_headset_info *info = (struct pm800_headset_info *)data;

	if (info != NULL) {
		queue_work(system_wq, &info->work_release);
	}
}

static irqreturn_t pm800_jack_detect_irq_thread(int irq, void *data)
{
	struct pm800_headset_info *info = data;
	pr_info("%s:\n", __func__);

	if (info->hs_jack != NULL) {
		/*
	 	* in any case headset interrupt comes,
	 	* hook interrupt is not welcomed. so just
	 	* disable it.
	 	*/
		pm800_hook_int(info, 0);
		//pm800_headset_work(info);
		queue_work(system_wq, &info->work_headset);

		/* Get Current time of detecting headset */ //KSND
		info->ts = current_kernel_time();
	}
	return IRQ_HANDLED;
}

static irqreturn_t pm800_jack_buttons_irq_thread(int irq, void *data)
{
	struct pm800_headset_info *info = data;
	pr_info("%s:\n", __func__);

	mod_timer(&info->hook_timer, jiffies + msecs_to_jiffies(130));

	/* hook interrupt */
	if (info->hk_jack != NULL) {
		/* disable hook interrupt anyway */
		regmap_update_bits(info->map, PM800_INT_ENA_3, PM800_GPADC_INT_ENA3(info->mic_gpadc_num), 0);
		pm800_jack_buttons_work(info);
		//queue_work(system_wq, &info->work_hook);
	}
	return IRQ_HANDLED;
}

static void pm800_init_work(struct work_struct *work)
{
	struct pm800_headset_info *info = container_of(work, struct pm800_headset_info, work_init);
	/* headset status is not stable when boot up. wait 200ms */
	msleep(200);
	pm800_jack_detect_irq_thread(info->irq_headset, info);
}

static void pm800_jack_detect_work(struct work_struct *work)
{
	struct pm800_headset_info *info = container_of(work, struct pm800_headset_info, work_headset);

	int ret, i;
	unsigned int value, voltage;
	int count[MAX_ZONE_LIMIT] = {0};

	if (info == NULL){
		return;
	}

	ret = regmap_read(info->map, PM800_MIC_CNTRL, &value);
	if (ret < 0) {
		dev_err(info->dev, "Failed to read PM822_MIC_CTRL: 0x%x\n", ret);
		return;
	}
	value &= PM822_HEADSET_DET_MASK;

	if (info->headset_flag == PM800_HS_DET_INVERT)
		value = !value;

	if( value ){
		/* MIC_BIAS on.  Enable mic power when it is headphone */
		if (info->mic_bias){
			mic_set_power(1, info);
		}
		/* enable MIC detection also enable measurement */
		regmap_update_bits(info->map, PM800_MIC_CNTRL, PM800_MICDET_EN, PM800_MICDET_EN);
		regmap_update_bits(info->map_gpadc, PM822_GPADC_MEAS_EN2, PM822_MIC_DET_MEAS_EN, PM822_MIC_DET_MEAS_EN);
		msleep(200);

		while( value ){
			voltage = gpadc_measure_voltage(info);
			pr_info("%s: adc[%d mv]\n", __func__, voltage);

			for ( i=0; i < info->num_jack_zones; i++ ){
				if( voltage <= info->jack_zones[i].adc_high ){
					if( ++count[i] > info->jack_zones[i].check_count ){
#ifdef SAMSUNG_JACK_SW_WATERPROOF
						if ((recheck_jack == true) && (info->ear_reselector_zone < voltage)) {
							pr_info("%s: wrong connect! reselect_zone: %d,[%d mv]\n",__func__, reselector_zone, voltage);
							pm800_jack_set_type(info, SEC_JACK_NO_DEVICE );
							recheck_jack = false;
							return;
						}
#endif
						// Set Earjack type (3 pole or 4 pole)
						pm800_jack_set_type(info, info->jack_zones[i].jack_type);
#if defined(SEC_USE_ANLOGDOCK_DEVICE)
						/* Use Docking device for external audio */
						pm800_audio_dock_ctrl(info,0);	//output only headset
#endif
						return;
					}
					if( info->jack_zones[i].delay_ms > 0){
						msleep(info->jack_zones[i].delay_ms);
					}
					break;
				}
			}
		}
		/* jack detection is failed */
		pm800_jack_set_type(info, SEC_JACK_NO_DEVICE );
		pr_info("%s : detection is failed\n", __func__);
	} 
	else {
		/* we already disable mic power when it is headphone */
		del_timer(&info->hook_timer);
		pm800_button_handle(info, info->press_release_th);
		pm800_jack_set_type(info, SEC_JACK_NO_DEVICE );
#ifdef SAMSUNG_JACK_SW_WATERPROOF
		recheck_jack = false;
#endif
#if defined(SEC_USE_ANLOGDOCK_DEVICE)
		/* Use Docking device for external audio */
		pm800_audio_dock_ctrl(info,1); //Output only Dock device
#endif
		pr_info("%s : KSND HEADSET released !!\n", __func__);
	}
}

static void pm800_jack_release_work(struct work_struct *work)
{
	struct pm800_headset_info *info = container_of(work, struct pm800_headset_info, work_release);
	unsigned int voltage;

	if (info->hook_vol_status >= HOOK_PRESSED){
		/* disable hook interrupt anyway */
		//regmap_update_bits(info->map, PM800_INT_ENA_3,
		//						PM800_GPADC_INT_ENA3(info->mic_gpadc_num), 0);
		//pm800_jack_buttons_work(info);
		////queue_work(system_wq, &info->work_hook);
		voltage = gpadc_measure_voltage(info);
		if (voltage >= info->press_release_th) { //KSND
			pm800_button_handle(info, voltage);
			pr_info("%s : force button released!!\n", __func__);
		}
	}
}

//static void pm800_jack_buttons_work(struct work_struct *work)
static void pm800_jack_buttons_work(struct pm800_headset_info *info)
{
	unsigned int value, voltage;
	int ret;

	if( info == NULL ) {
		pr_debug("Invalid hook info!\n");
		return;
	}

	voltage = gpadc_measure_voltage(info);
	msleep(80);

	ret = regmap_read(info->map, PM800_MIC_CNTRL, &value);
	if (ret < 0) {
		dev_err(info->dev, "Failed to read PM822_MIC_CTRL: 0x%x\n", ret);
		return;
	}
	value &= PM822_HEADSET_DET_MASK;

	if (info->headset_flag == PM800_HS_DET_INVERT)
		value = !value;

	if (!value) {
		/* in case of headset unpresent, do nothing */
		trace_printk("hook: false exit\n");
		goto FAKE_HOOK;
	}

	pm800_button_handle(info, voltage);
	regmap_update_bits(info->map, PM800_INT_ENA_3,	
						PM800_GPADC_INT_ENA3(info->mic_gpadc_num),
						PM800_GPADC_INT_ENA3(info->mic_gpadc_num));
	return;

FAKE_HOOK:
	dev_err(info->dev, "fake hook interupt\n");

}


static int pm800_headset_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pm800_headset_info *info = platform_get_drvdata(pdev);

	/* enable low power mode headset detection */
	regmap_update_bits(info->map, PM800_HEADSET_CNTRL, PM800_HSDET_SLP, PM800_HSDET_SLP);

	/* it's not proper to use "pm80x_dev_suspend" here */
	if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(info->irq_headset);
		if (info->hs_status == SND_JACK_HEADSET)
			enable_irq_wake(info->irq_hook);
	}
	return 0;
}

static int pm800_headset_resume(struct platform_device *pdev)
{
	struct pm800_headset_info *info = platform_get_drvdata(pdev);

	/* disable low power mode headset detection */
	regmap_update_bits(info->map, PM800_HEADSET_CNTRL, PM800_HSDET_SLP, 0);

	if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(info->irq_headset);
		if (info->hs_status == SND_JACK_HEADSET)
			disable_irq_wake(info->irq_hook);
	}
	return 0;
}

#ifdef SEC_SYSFS_FOR_FACTORY_TEST //KSND
/*************************************
  * add sysfs for factory test. KSND 131029
  * 
 *************************************/
static ssize_t key_state_onoff_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", (info->hook_vol_status == HOOK_PRESSED)? "1":"0");
}

static ssize_t earjack_state_onoff_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", info->state);
}

static ssize_t select_jack_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);
	return 0;
}

static ssize_t select_jack_store(struct device *dev, 	struct device_attribute *attr, const char *buf, size_t size)
{
//	struct pm800_headset_info *info = dev_get_drvdata(dev);
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_info("%s: User  selection : 0X%x", __func__, value);

	if( value == SEC_HEADSET_4POLE || value == SEC_HEADSET_3POLE ) {
		if (info->mic_bias) {
			mic_set_power(1, info);
		}
		/* enable MIC detection also enable measurement */
		regmap_update_bits(info->map, PM800_MIC_CNTRL, PM800_MICDET_EN,PM800_MICDET_EN);
		regmap_update_bits(info->map_gpadc, PM822_GPADC_MEAS_EN2,PM822_MIC_DET_MEAS_EN, PM822_MIC_DET_MEAS_EN);
		msleep(200);

		pm800_jack_set_type(info, value);
	} else {
		pm800_jack_set_type(info, SEC_JACK_NO_DEVICE);
	}

	return size;
}

static DEVICE_ATTR(key_state, S_IRUGO , key_state_onoff_show, NULL);
static DEVICE_ATTR(state, S_IRUGO , earjack_state_onoff_show, NULL);
static DEVICE_ATTR(select_jack, 0664, select_jack_show, 	select_jack_store);
#endif /* SEC_SYSFS_FOR_FACTORY_TEST */

#ifdef SAMSUNG_JACK_SW_WATERPROOF
static ssize_t reselect_jack_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);
	return 0;
}

static ssize_t reselect_jack_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct pm800_headset_info *info = dev_get_drvdata(dev);
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);

	if (value == 1) {
		recheck_jack = true;
		pm800_jack_detect_work(&info->work_headset);
	}
	return size;
}
static DEVICE_ATTR(reselect_jack,0664,reselect_jack_show,reselect_jack_store);
#endif /* SAMSUNG_JACK_SW_WATERPROOF */

static void pm800_dt_init( struct device_node *np, struct device *dev )
{
	if (of_property_read_u32( np, "marvell,headset-flag", &info->headset_flag))
		dev_dbg(dev, "Do not get headset flag\n");
	if (of_property_read_u32( np, "marvell,press-release-th", &info->press_release_th))
		dev_dbg(dev, "Do not get headset flag\n");

	if (of_property_read_u32( np, "sec0,adc_high", &sec_jack_zones[0].adc_high))
		dev_dbg(dev, "Do not get sec0 flag\n");
	if (of_property_read_u32( np, "sec0,delay_ms", &sec_jack_zones[0].delay_ms))
		dev_dbg(dev, "Do not get sec0 flag\n");
	if (of_property_read_u32( np, "sec0,check_count", &sec_jack_zones[0].check_count))
		dev_dbg(dev, "Do not get sec0 flag\n");
	if (of_property_read_u32( np, "sec0,jack_type", &sec_jack_zones[0].jack_type))
		dev_dbg(dev, "Do not get sec0 flag\n");

	if (of_property_read_u32( np, "sec1,adc_high", &sec_jack_zones[1].adc_high))
		dev_dbg(dev, "Do not get sec1 flag\n");
	if (of_property_read_u32( np, "sec1,delay_ms", &sec_jack_zones[1].delay_ms))
		dev_dbg(dev, "Do not get sec1 flag\n");
	if (of_property_read_u32( np, "sec1,check_count", &sec_jack_zones[1].check_count))
		dev_dbg(dev, "Do not get sec1 flag\n");
	if (of_property_read_u32( np, "sec1,jack_type", &sec_jack_zones[1].jack_type))
		dev_dbg(dev, "Do not get sec1 flag\n");

	if (of_property_read_u32( np, "sec2,adc_high", &sec_jack_zones[2].adc_high))
		dev_dbg(dev, "Do not get sec2 flag\n");
	if (of_property_read_u32( np, "sec2,delay_ms", &sec_jack_zones[2].delay_ms))
		dev_dbg(dev, "Do not get sec2 flag\n");
	if (of_property_read_u32( np, "sec2,check_count", &sec_jack_zones[2].check_count))
		dev_dbg(dev, "Do not get sec2 flag\n");
	if (of_property_read_u32( np, "sec2,jack_type", &sec_jack_zones[2].jack_type))
		dev_dbg(dev, "Do not get sec2 flag\n");

	if (of_property_read_u32( np, "sec3,adc_high", &sec_jack_zones[3].adc_high))
		dev_dbg(dev, "Do not get sec3 flag\n");
	if (of_property_read_u32( np, "sec3,delay_ms", &sec_jack_zones[3].delay_ms))
		dev_dbg(dev, "Do not get sec3 flag\n");
	if (of_property_read_u32( np, "sec3,check_count", &sec_jack_zones[3].check_count))
		dev_dbg(dev, "Do not get sec3 flag\n");
	if (of_property_read_u32( np, "sec3,jack_type", &sec_jack_zones[3].jack_type))
		dev_dbg(dev, "Do not get sec3 flag\n");

	if (of_property_read_u32( np, "sec4,adc_high", &sec_jack_zones[4].adc_high))
		dev_dbg(dev, "Do not get sec4 flag\n");
	if (of_property_read_u32( np, "sec4,delay_ms", &sec_jack_zones[4].delay_ms))
		dev_dbg(dev, "Do not get sec4 flag\n");
	if (of_property_read_u32( np, "sec4,check_count", &sec_jack_zones[4].check_count))
		dev_dbg(dev, "Do not get sec4 flag\n");
	if (of_property_read_u32( np, "sec4,jack_type", &sec_jack_zones[4].jack_type))
		dev_dbg(dev, "Do not get sec4 flag\n");

	if (of_property_read_u32( np, "sec_sendend0,code", &sec_jack_buttons_zones[0].code))
		dev_dbg(dev, "Do not get sec_sendend0 flag\n");
	if (of_property_read_u32( np, "sec_sendend0,adc_low", &sec_jack_buttons_zones[0].adc_low))
		dev_dbg(dev, "Do not get sec_sendend0 flag\n");
	if (of_property_read_u32( np, "sec_sendend0,adc_high", &sec_jack_buttons_zones[0].adc_high))
		dev_dbg(dev, "Do not get sec_sendend0 flag\n");

	if (of_property_read_u32( np, "sec_sendend1,code", &sec_jack_buttons_zones[1].code))
		dev_dbg(dev, "Do not get sec_sendend1 flag\n");
	if (of_property_read_u32( np, "sec_sendend1,adc_low", &sec_jack_buttons_zones[1].adc_low))
		dev_dbg(dev, "Do not get sec_sendend1 flag\n");
	if (of_property_read_u32( np, "sec_sendend1,adc_high", &sec_jack_buttons_zones[1].adc_high))
		dev_dbg(dev, "Do not get sec_sendend1 flag\n");

	if (of_property_read_u32( np, "sec_sendend2,code", &sec_jack_buttons_zones[2].code))
		dev_dbg(dev, "Do not get sec_sendend2 flag\n");
	if (of_property_read_u32( np, "sec_sendend2,adc_low", &sec_jack_buttons_zones[2].adc_low))
		dev_dbg(dev, "Do not get sec_sendend2 flag\n");
	if (of_property_read_u32( np, "sec_sendend2,adc_high", &sec_jack_buttons_zones[2].adc_high))
		dev_dbg(dev, "Do not get sec_sendend2 flag\n");

#ifdef SAMSUNG_JACK_SW_WATERPROOF
	//Need to add from dts db
#endif

	info->jack_zones = sec_jack_zones;
	info->num_jack_zones = ARRAY_SIZE(sec_jack_zones);
	info->buttons_zones = sec_jack_buttons_zones;
	info->num_buttons_zones =  ARRAY_SIZE(sec_jack_buttons_zones);
}

static int pm800_headset_probe(struct platform_device *pdev)
{
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct resource *headset_resource, *mic_resource;
	//struct headset_switch_data *switch_data_headset = NULL;
#ifdef SEC_SYSFS_FOR_FACTORY_TEST //KSND
	struct class *audio;
	struct device *earjack;
#endif /* SEC_SYSFS_FOR_FACTORY_TEST */
	int irq_headset, irq_hook, ret = 0;
	//struct device_node *node = pdev->dev.of_node;

	headset_resource = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!headset_resource) {
		dev_err(&pdev->dev, "No resource for headset!\n");
		return -EINVAL;
	}

	mic_resource = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	if (!mic_resource) {
		dev_err(&pdev->dev, "No resource for mic!\n");
		return -EINVAL;
	}

	irq_headset = platform_get_irq(pdev, 0);
	if (irq_headset < 0) {
		dev_err(&pdev->dev, "No IRQ resource for headset!\n");
		return -EINVAL;
	}
	irq_hook = platform_get_irq(pdev, 1);
	if (irq_hook < 0) {
		dev_err(&pdev->dev, "No IRQ resource for hook/mic!\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct pm800_headset_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->chip = chip;
    info->mic_bias = regulator_get(&pdev->dev, "marvell,micbias");
	if (IS_ERR(info->mic_bias)) {
		info->mic_bias = NULL;
		dev_err(&pdev->dev, "Get regulator error\n");
		return -EINVAL;
	}

	info->headset_gpio_num = get_headset_gpio_num(headset_resource);
	info->mic_gpadc_num = get_mic_gpadc_num(mic_resource);

	if (IS_ENABLED(CONFIG_OF)) {
		pm800_dt_init( pdev->dev.of_node, &pdev->dev );
	} else {
		dev_dbg(&pdev->dev, "Fail to read from DTS data!\n");
	}

	info->map = chip->regmap;
	info->map_gpadc = chip->subchip->regmap_gpadc;
	info->dev = &pdev->dev;
	info->irq_headset = irq_headset;
	info->irq_hook = irq_hook;
#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	info->chgpump_ctrl = sm5502_chgpump_ctrl;
	info->dock_audiopath_ctrl = sm5502_dock_audiopath_ctrl;
	info->usb_switch_register_notify = usb_switch_register_notify;
	info->usb_switch_unregister_notify = usb_switch_unregister_notify;
#endif
	info->hk_status = 0;
	info->hook_vol_status = HOOK_VOL_ALL_RELEASED;

	//switch_data_headset->sdev = switch_jack_detection;
	//info->psw_data_headset = switch_data_headset;

	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0){
		goto err_out;
	}

	mutex_init(&info->hs_mutex);

	INIT_WORK(&info->work_headset, pm800_jack_detect_work);
	//INIT_WORK(&info->work_hook, pm800_jack_buttons_work);
	INIT_WORK(&info->work_release, pm800_jack_release_work);
	INIT_WORK(&info->work_init, pm800_init_work);

	/* Initialize timer for detecting loop */
	setup_timer(&info->headset_timer, pm800_jack_timer_handler,(unsigned long)info);

	/* init timer for hook release */
	setup_timer(&info->hook_timer, pm800_button_timer_handler,(unsigned long)info);


	ret = devm_request_threaded_irq(&pdev->dev, info->irq_headset, NULL, pm800_jack_detect_irq_thread,
									IRQF_ONESHOT | IRQF_NO_SUSPEND, "headset", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n", info->irq_headset, ret);
		goto err_out;
	}

	ret = devm_request_threaded_irq(&pdev->dev, info->irq_hook, NULL, pm800_jack_buttons_irq_thread,
									IRQF_ONESHOT | IRQF_NO_SUSPEND, "hook", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n", info->irq_hook, ret);
		goto err_out;
	}

	/*
	 * disable hook irq to ensure this irq will be enabled
	 * after plugging in headset
	 */
	info->hook_count = 1;
	pm800_hook_int(info, 0);

	platform_set_drvdata(pdev, info);
	info->hsdetect_dev = &pdev->dev;

	/* Hook:32 ms debounce time */
	regmap_update_bits(info->map, PM800_MIC_CNTRL, MIC_DET_DBS, MIC_DET_DBS_32MS);
	/* Hook:continue duty cycle */
	regmap_update_bits(info->map, PM800_MIC_CNTRL, MIC_DET_PRD, MIC_DET_PRD_256MS);
	/* set GPADC_DIR to 1, set to 0 cause pop noise in recording */
	regmap_update_bits(info->map_gpadc, PM800_GPADC_MISC_CONFIG1,
			   PM800_GPADC4_DIR, PM800_GPADC4_DIR);

	disable_irq(info->irq_headset);
	regmap_update_bits(info->map, PM800_HEADSET_CNTRL, PM800_HEADSET_DET_EN, PM800_HEADSET_DET_EN);
	enable_irq(info->irq_headset);

	device_init_wakeup(&pdev->dev, 1);

#ifdef SEC_SYSFS_FOR_FACTORY_TEST //KSND
	/* sys fs for factory test */
	audio = class_create(THIS_MODULE, "audio");
	if (IS_ERR(audio))
		pr_err("Failed to create class(audio)!\n");
		
	earjack = device_create(audio, NULL, 0, NULL, "earjack");
	if (IS_ERR(earjack))
		pr_err("Failed to create device(earjack)!\n");

	ret = device_create_file(earjack, &dev_attr_key_state);
	if (ret)
		pr_err("Failed to create device file in sysfs entries!\n");
			
	ret = device_create_file(earjack, &dev_attr_state);
	if (ret)
		pr_err("Failed to create device file in sysfs entries!\n");
		
	ret = device_create_file(earjack, &dev_attr_select_jack);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n", dev_attr_select_jack.attr.name);
			
	dev_set_drvdata(earjack, info);	//KSND	
#endif /* SEC_SYSFS_FOR_FACTORY_TEST */

#ifdef SAMSUNG_JACK_SW_WATERPROOF
	ret = device_create_file(earjack, &dev_attr_reselect_jack);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n", dev_attr_reselect_jack.attr.name);
#endif

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	/* Use Docking device for external audio */
	if( info->usb_switch_register_notify != NULL ){
		info->usb_switch_register_notify(&dock_nb);
	}
#endif

	return 0;

err_out:
	return ret;

}

static int pm800_headset_remove(struct platform_device *pdev)
{
	struct pm800_headset_info *info = platform_get_drvdata(pdev);

	/* disable headset detection on GPIO */
	regmap_update_bits(info->map, PM800_HEADSET_CNTRL, PM800_HEADSET_DET_EN, 0);

	/* disable GPIO interrupt */
	regmap_update_bits(info->map, PM800_INT_ENA_4, PM800_GPIO_INT_ENA4(info->headset_gpio_num), 0);

	cancel_work_sync(&info->work_headset);
	//cancel_work_sync(&info->work_hook);
	cancel_work_sync(&info->work_release);

	free_irq(info->irq_hook, info);
	free_irq(info->irq_headset, info);

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	/* Use Docking device for external audio */
	if( info->usb_switch_register_notify != NULL ){
		info->usb_switch_unregister_notify(&dock_nb);
	}
#endif

	devm_kfree(&pdev->dev, info);

	return 0;
}

static struct platform_driver pm800_headset_driver = {
	.probe = pm800_headset_probe,
	.remove = pm800_headset_remove,
	.suspend = pm800_headset_suspend,
	.resume = pm800_headset_resume,
	.driver = {
		   .name = "88pm800-headset",
		   .owner = THIS_MODULE,
		   },
};

module_platform_driver(pm800_headset_driver);

MODULE_DESCRIPTION("Samsung Custom 88PM822 Headset driver");
MODULE_AUTHOR("Youngki Park <ykp74@samsung.com>");
MODULE_LICENSE("GPL");

