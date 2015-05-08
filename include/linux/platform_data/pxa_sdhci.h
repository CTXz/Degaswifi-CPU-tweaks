/*
 * include/linux/platform_data/pxa_sdhci.h
 *
 * Copyright 2010 Marvell
 *	Zhangfei Gao <zhangfei.gao@marvell.com>
 *
 * PXA Platform - SDHCI platform data definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PXA_SDHCI_H_
#define _PXA_SDHCI_H_

/*
 * sdhci_pxa_dtr_data: sdhc data transfer rate table
 * @timing: the specification used timing
 * @preset_rate: the clock could set by the SOC
 * @src_rate: send to the clock subsystem
 * related to APMU on SOC
 * @tx_delay: use DDLL to delay clock waveform
 */
struct sdhci_pxa_dtr_data {
	unsigned char timing;
	unsigned long preset_rate;
	unsigned long src_rate;
	unsigned int tx_delay;
	unsigned int rx_delay;
};

#include <linux/pm_qos.h>

/*
 * struct pxa_sdhci_platdata() - Platform device data for PXA SDHCI
 * @flags: flags for platform requirement
 * @clk_delay_cycles:
 *	mmp2: each step is roughly 100ps, 5bits width
 *	pxa910: each step is 1ns, 4bits width
 * @clk_delay_sel: select clk_delay, used on pxa910
 *	0: choose feedback clk
 *	1: choose feedback clk + delay value
 *	2: choose internal clk
 * @clk_delay_enable: enable clk_delay or not, used on pxa910
 * @ext_cd_gpio: gpio pin used for external CD line
 * @ext_cd_gpio_invert: invert values for external CD gpio line
 * @max_speed: the maximum speed supported
 * @host_caps: Standard MMC host capabilities bit field.
 * @quirks: quirks of platfrom
 * @quirks2: quirks2 of platfrom
 * @pm_caps: pm_caps of platfrom
 * @signal_vol_change: signaling voltage change
 */
struct sdhci_pxa_platdata {
	unsigned int	flags;
	unsigned int	clk_delay_cycles;
	unsigned int	clk_delay_sel;
	bool		clk_delay_enable;
	unsigned int	max_speed;
	u32		host_caps;
	u32		host_caps2;
	unsigned int    host_caps_disable;
	unsigned int    host_caps2_disable;
	unsigned int	quirks;
	unsigned int	quirks2;
	unsigned int	pm_caps;
	struct sdhci_pxa_dtr_data *dtr_data;
#ifdef CONFIG_SD8XXX_RFKILL
	/* for sd8xxx-rfkill device */
	struct mmc_host **pmmc;
#endif
	struct  pm_qos_request  qos_idle;
	u32	lpm_qos;
	void (*clear_wakeup_event)(void);
	void (*signal_vol_change)(unsigned int set);
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_slow;
	struct pinctrl_state *pin_fast;
};

struct sdhci_pxa {
	u8	clk_enable;
	u8	power_mode;
	struct device_attribute tx_delay;
	unsigned int tx_dly_val;
	struct device_attribute rx_delay;
	unsigned int rx_dly_val;
};
#endif /* _PXA_SDHCI_H_ */
