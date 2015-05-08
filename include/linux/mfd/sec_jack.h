/*
 * Copyright (C) 2014 Samsung Electronics, Inc.
 *
 * filename : sec_jack.h
 * headset & hook detect driver for pm822 (SAMSUNG COMSTOM KSND)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_SEC_HEADSET_H

#define SEC_SYSFS_FOR_FACTORY_TEST

#if defined(CONFIG_MACH_DEGAS)
/* Enable the Analog Deck Dock device */
#define SEC_USE_ANLOGDOCK_DEVICE
#endif

enum {
	SEC_JACK_NO_DEVICE		= 0x0,
	SEC_HEADSET_4POLE		= 0x01 << 0,
	SEC_HEADSET_3POLE		= 0x01 << 1,
	SEC_UNKNOWN_DEVICE		= 0x01 << 2,
};

struct sec_jack_zone {
	unsigned int adc_high;
	unsigned int delay_ms;
	unsigned int check_count;
	unsigned int jack_type;
};

struct sec_jack_buttons_zone {
	unsigned int code;
	unsigned int adc_low;
	unsigned int adc_high;
};

struct sec_jack_platform_data {
	int headset_flag;
	void (*mic_set_power)(int on);

#if defined(SEC_USE_ANLOGDOCK_DEVICE)
	void (*dock_audiopath_ctrl)(int on);
	void (*chgpump_ctrl)(int enable);
	int (*usb_switch_register_notify)(struct notifier_block *nb);
	int (*usb_switch_unregister_notify)(struct notifier_block *nb);
#endif /* SEC_USE_ANLOGDOCK_DEVICE */

	struct	sec_jack_zone	*zones;
	struct	sec_jack_buttons_zone	*buttons_zones;

	int	num_zones;
	int	num_buttons_zones;
	int	press_release_th;
	
#ifdef SAMSUNG_JACK_SW_WATERPROOF
	int ear_reselector_zone;
#endif
#ifdef CONFIG_SAMSUNG_JACK_COM_DET
	unsigned long com_det_gpio; 
#endif /* SAMSUNG_JACK_SW_WATERPROOF */

};

/* Samsung headset driver data KSND */
static struct switch_dev switch_jack_detection = {
	.name = "h2w",
};

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, default to 3pole if it stays
		 * in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 0,
		.delay_ms = 20,
		.check_count = 2,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 350, unstable zone, default to 3pole if it stays
		 * in this range for a 100ms (20ms delays, 5 samples)
		 */
		.adc_high = 350,
		.delay_ms = 20,
		.check_count = 5,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 350 < adc <= 500, unstable zone, default to 4pole if it
		 * stays in this range for 200ms (20ms delays, 10 samples)
		 */
		.adc_high = 500,
		.delay_ms = 20,
		.check_count = 10,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 500 < adc <= 1399, default to 4 pole if it stays */
		/* in this range for 40ms (20ms delays, 2 samples)
		 */
		.adc_high = 1399,
		.delay_ms = 20,
		.check_count = 2,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 1400, unstable zone, default to 3pole if it stays
		 * in this range for a second (10ms delays, 100 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 100,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* to support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=60, stable zone */
		.code		= SND_JACK_BTN_0,
		.adc_low	= 0,
		.adc_high	= 160,
	},
	{
		/* 61 <= adc <= 110, stable zone */
		.code		= SND_JACK_BTN_1,
		.adc_low	= 161,
		.adc_high	= 300,
	},
	{
		/* 111 <= adc <= 240, stable zone */
		.code		= SND_JACK_BTN_2,
		.adc_low	= 301,
		.adc_high	= 700,
	},
};

#endif //__ASM_ARCH_SEC_HEADSET_H

