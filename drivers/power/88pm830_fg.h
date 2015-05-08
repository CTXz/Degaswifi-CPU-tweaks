#ifndef __PM830_FG_H
#define __PM830_FG_H

#ifdef CONFIG_MFD_88PM800
#include <linux/mfd/88pm80x.h>
#endif

#ifdef CONFIG_MFD_88PM822
#include <linux/mfd/88pm822.h>
#endif

#define STORE_BAT_VALUES_IN_RTC
/*
 * We define 3650mV is the low bat voltage;
 * define 3450mV is the power off voltage.
 */
#define LOW_BAT_THRESHOLD		(3650)
#define LOW_BAT_CAP			(15)
#define POWER_OFF_THRESHOLD		(3450)
/*
 * battery temperature based on NTC resistor, defined
 * corresponding resistor value  -- Ohm / C degeree.
 */
#define TBAT_NEG_25D		127773	/* -25 */
#define TBAT_NEG_10D		54564	/* -10 */
#define TBAT_0D			32330	/* 0 */
#define TBAT_10D		19785	/* 10 */
#define TBAT_20D		12468	/* 20 */
#define TBAT_30D		8072	/* 30 */
#define TBAT_40D		5356	/* 40 */

static int ocv_table[] = {
	3450, 3510, 3540, 3558, 3580, 3609, 3622, 3637, 3649, 3657,
	3661, 3665, 3670, 3675, 3683, 3692, 3698, 3703, 3709, 3716,
	3720, 3726, 3728, 3732, 3734, 3740, 3744, 3749, 3750, 3754,
	3760, 3760, 3763, 3764, 3768, 3769, 3771, 3772, 3774, 3776,
	3777, 3780, 3782, 3784, 3785, 3789, 3792, 3796, 3798, 3801,
	3804, 3808, 3810, 3815, 3818, 3824, 3829, 3833, 3840, 3843,
	3849, 3854, 3862, 3868, 3874, 3880, 3887, 3892, 3898, 3904,
	3910, 3916, 3922, 3929, 3934, 3941, 3946, 3954, 3962, 3968,
	3976, 3982, 3987, 3995, 4006, 4013, 4019, 4028, 4035, 4044,
	4053, 4060, 4070, 4081, 4088, 4102, 4122, 4136, 4154, 4170
};

#ifdef STORE_BAT_VALUES_IN_RTC
static int read_pmic(int page_id, int reg_id)
{
	if ((reg_id != 0) && (reg_id != 1) && (reg_id != -5))
		return -EINVAL;
	if ((page_id != 1) && (page_id != 3))
		return -EINVAL;

	/* TODO: add read function here */
	return 0;
}

static int write_pmic(int page_id, int reg_id, int data)
{
	if ((reg_id != 0) && (reg_id != 1))
		return -EINVAL;
	if (page_id != 1)
		return -EINVAL;
	/* TODO: add write function here */
	return 0;
}

#else
static int read_pmic(int page_id, int reg_id)
{
	return 0;
}

static int write_pmic(int page_id, int reg_id, int data)
{
	return 0;
}
#endif

#endif
