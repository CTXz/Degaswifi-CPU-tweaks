/* include/linux/bma254.h
 *
 * Copyright (C) 2014 Samsung Electronics Co, Ltd.
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

#ifndef __BMA254_H__
#define __BMA254_H__

#include <linux/pm_wakeup.h>

#define SENSOR_NAME			"bma254"
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)
#define SLOPE_THRESHOLD_VALUE		32
#define SLOPE_DURATION_VALUE		1
#define INTERRUPT_LATCH_MODE		13
#define INTERRUPT_ENABLE		1
#define INTERRUPT_DISABLE		0
#define MAP_SLOPE_INTERRUPT		2
#define SLOPE_X_INDEX			5
#define SLOPE_Y_INDEX			6
#define SLOPE_Z_INDEX			7

#define BMA254_MIN_DELAY		1
#define BMA254_DEFAULT_DELAY		200

#define BMA254_CHIP_ID			0xFA
#define BMA254_RANGE_SET		0
#define BMA254_BW_SET			10


/*
 *
 *      register definitions
 *
 */

#define BMA254_CHIP_ID_REG                      0x00
#define BMA254_VERSION_REG                      0x01
#define BMA254_X_AXIS_LSB_REG                   0x02
#define BMA254_X_AXIS_MSB_REG                   0x03
#define BMA254_Y_AXIS_LSB_REG                   0x04
#define BMA254_Y_AXIS_MSB_REG                   0x05
#define BMA254_Z_AXIS_LSB_REG                   0x06
#define BMA254_Z_AXIS_MSB_REG                   0x07
#define BMA254_TEMP_RD_REG                      0x08
#define BMA254_STATUS1_REG                      0x09
#define BMA254_STATUS2_REG                      0x0A
#define BMA254_STATUS_TAP_SLOPE_REG             0x0B
#define BMA254_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA254_RANGE_SEL_REG                    0x0F
#define BMA254_BW_SEL_REG                       0x10
#define BMA254_MODE_CTRL_REG                    0x11
#define BMA254_LOW_NOISE_CTRL_REG               0x12
#define BMA254_DATA_CTRL_REG                    0x13
#define BMA254_RESET_REG                        0x14
#define BMA254_INT_ENABLE1_REG                  0x16
#define BMA254_INT_ENABLE2_REG                  0x17
#define BMA254_INT1_PAD_SEL_REG                 0x19
#define BMA254_INT_DATA_SEL_REG                 0x1A
#define BMA254_INT2_PAD_SEL_REG                 0x1B
#define BMA254_INT_SRC_REG                      0x1E
#define BMA254_INT_SET_REG                      0x20
#define BMA254_INT_CTRL_REG                     0x21
#define BMA254_LOW_DURN_REG                     0x22
#define BMA254_LOW_THRES_REG                    0x23
#define BMA254_LOW_HIGH_HYST_REG                0x24
#define BMA254_HIGH_DURN_REG                    0x25
#define BMA254_HIGH_THRES_REG                   0x26
#define BMA254_SLOPE_DURN_REG                   0x27
#define BMA254_SLOPE_THRES_REG                  0x28
#define BMA254_TAP_PARAM_REG                    0x2A
#define BMA254_TAP_THRES_REG                    0x2B
#define BMA254_ORIENT_PARAM_REG                 0x2C
#define BMA254_THETA_BLOCK_REG                  0x2D
#define BMA254_THETA_FLAT_REG                   0x2E
#define BMA254_FLAT_HOLD_TIME_REG               0x2F
#define BMA254_STATUS_LOW_POWER_REG             0x31
#define BMA254_SELF_TEST_REG                    0x32
#define BMA254_EEPROM_CTRL_REG                  0x33
#define BMA254_SERIAL_CTRL_REG                  0x34
#define BMA254_CTRL_UNLOCK_REG                  0x35
#define BMA254_OFFSET_CTRL_REG                  0x36
#define BMA254_OFFSET_PARAMS_REG                0x37
#define BMA254_OFFSET_FILT_X_REG                0x38
#define BMA254_OFFSET_FILT_Y_REG                0x39
#define BMA254_OFFSET_FILT_Z_REG                0x3A
#define BMA254_OFFSET_UNFILT_X_REG              0x3B
#define BMA254_OFFSET_UNFILT_Y_REG              0x3C
#define BMA254_OFFSET_UNFILT_Z_REG              0x3D
#define BMA254_SPARE_0_REG                      0x3E
#define BMA254_SPARE_1_REG                      0x3F




#define BMA254_ACC_X_LSB__POS           4
#define BMA254_ACC_X_LSB__LEN           4
#define BMA254_ACC_X_LSB__MSK           0xF0
#define BMA254_ACC_X_LSB__REG           BMA254_X_AXIS_LSB_REG

#define BMA254_ACC_X_MSB__POS           0
#define BMA254_ACC_X_MSB__LEN           8
#define BMA254_ACC_X_MSB__MSK           0xFF
#define BMA254_ACC_X_MSB__REG           BMA254_X_AXIS_MSB_REG

#define BMA254_ACC_Y_LSB__POS           4
#define BMA254_ACC_Y_LSB__LEN           4
#define BMA254_ACC_Y_LSB__MSK           0xF0
#define BMA254_ACC_Y_LSB__REG           BMA254_Y_AXIS_LSB_REG

#define BMA254_ACC_Y_MSB__POS           0
#define BMA254_ACC_Y_MSB__LEN           8
#define BMA254_ACC_Y_MSB__MSK           0xFF
#define BMA254_ACC_Y_MSB__REG           BMA254_Y_AXIS_MSB_REG

#define BMA254_ACC_Z_LSB__POS           4
#define BMA254_ACC_Z_LSB__LEN           4
#define BMA254_ACC_Z_LSB__MSK           0xF0
#define BMA254_ACC_Z_LSB__REG           BMA254_Z_AXIS_LSB_REG

#define BMA254_ACC_Z_MSB__POS           0
#define BMA254_ACC_Z_MSB__LEN           8
#define BMA254_ACC_Z_MSB__MSK           0xFF
#define BMA254_ACC_Z_MSB__REG           BMA254_Z_AXIS_MSB_REG

#define BMA254_RANGE_SEL__POS             0
#define BMA254_RANGE_SEL__LEN             4
#define BMA254_RANGE_SEL__MSK             0x0F
#define BMA254_RANGE_SEL__REG             BMA254_RANGE_SEL_REG

#define BMA254_BANDWIDTH__POS             0
#define BMA254_BANDWIDTH__LEN             5
#define BMA254_BANDWIDTH__MSK             0x1F
#define BMA254_BANDWIDTH__REG             BMA254_BW_SEL_REG

#define BMA254_EN_LOW_POWER__POS          6
#define BMA254_EN_LOW_POWER__LEN          1
#define BMA254_EN_LOW_POWER__MSK          0x40
#define BMA254_EN_LOW_POWER__REG          BMA254_MODE_CTRL_REG

#define BMA254_EN_SUSPEND__POS            7
#define BMA254_EN_SUSPEND__LEN            1
#define BMA254_EN_SUSPEND__MSK            0x80
#define BMA254_EN_SUSPEND__REG            BMA254_MODE_CTRL_REG

#define BMA254_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA254_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* range and bandwidth */

#define BMA254_RANGE_2G                 3
#define BMA254_RANGE_4G                 5
#define BMA254_RANGE_8G                 8
#define BMA254_RANGE_16G                12


#define BMA254_BW_7_81HZ        0x08
#define BMA254_BW_15_63HZ       0x09
#define BMA254_BW_31_25HZ       0x0A
#define BMA254_BW_62_50HZ       0x0B
#define BMA254_BW_125HZ         0x0C
#define BMA254_BW_250HZ         0x0D
#define BMA254_BW_500HZ         0x0E
#define BMA254_BW_1000HZ        0x0F

/* mode settings */

#define BMA254_MODE_NORMAL      0
#define BMA254_MODE_LOWPOWER    1
#define BMA254_MODE_SUSPEND     2


#define BMA254_EN_SELF_TEST__POS                0
#define BMA254_EN_SELF_TEST__LEN                2
#define BMA254_EN_SELF_TEST__MSK                0x03
#define BMA254_EN_SELF_TEST__REG                BMA254_SELF_TEST_REG



#define BMA254_NEG_SELF_TEST__POS               2
#define BMA254_NEG_SELF_TEST__LEN               1
#define BMA254_NEG_SELF_TEST__MSK               0x04
#define BMA254_NEG_SELF_TEST__REG               BMA254_SELF_TEST_REG

#define BMA254_EN_FAST_COMP__POS                5
#define BMA254_EN_FAST_COMP__LEN                2
#define BMA254_EN_FAST_COMP__MSK                0x60
#define BMA254_EN_FAST_COMP__REG                BMA254_OFFSET_CTRL_REG

#define BMA254_FAST_COMP_RDY_S__POS             4
#define BMA254_FAST_COMP_RDY_S__LEN             1
#define BMA254_FAST_COMP_RDY_S__MSK             0x10
#define BMA254_FAST_COMP_RDY_S__REG             BMA254_OFFSET_CTRL_REG

#define BMA254_COMP_TARGET_OFFSET_X__POS        1
#define BMA254_COMP_TARGET_OFFSET_X__LEN        2
#define BMA254_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA254_COMP_TARGET_OFFSET_X__REG        BMA254_OFFSET_PARAMS_REG

#define BMA254_COMP_TARGET_OFFSET_Y__POS        3
#define BMA254_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA254_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA254_COMP_TARGET_OFFSET_Y__REG        BMA254_OFFSET_PARAMS_REG

#define BMA254_COMP_TARGET_OFFSET_Z__POS        5
#define BMA254_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA254_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA254_COMP_TARGET_OFFSET_Z__REG        BMA254_OFFSET_PARAMS_REG


#define BMA254_SLOPE_INT_S__POS          2
#define BMA254_SLOPE_INT_S__LEN          1
#define BMA254_SLOPE_INT_S__MSK          0x04
#define BMA254_SLOPE_INT_S__REG          BMA254_STATUS1_REG

#define BMA254_SLOPE_THRES__POS                  0
#define BMA254_SLOPE_THRES__LEN                  8
#define BMA254_SLOPE_THRES__MSK                  0xFF
#define BMA254_SLOPE_THRES__REG                  BMA254_SLOPE_THRES_REG

#define BMA254_EN_INT1_PAD_SLOPE__POS       2
#define BMA254_EN_INT1_PAD_SLOPE__LEN       1
#define BMA254_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA254_EN_INT1_PAD_SLOPE__REG       BMA254_INT1_PAD_SEL_REG

#define BMA254_SLOPE_DUR__POS                    0
#define BMA254_SLOPE_DUR__LEN                    2
#define BMA254_SLOPE_DUR__MSK                    0x03
#define BMA254_SLOPE_DUR__REG                    BMA254_SLOPE_DURN_REG

#define BMA254_EN_SOFT_RESET__POS         0
#define BMA254_EN_SOFT_RESET__LEN         8
#define BMA254_EN_SOFT_RESET__MSK         0xFF
#define BMA254_EN_SOFT_RESET__REG         BMA254_RESET_REG

#define BMA254_EN_SOFT_RESET_VALUE        0xB6

#define CALIBRATION_FILE_PATH	"/efs/calibration_data"

/* PMIC Regulator based supply */
#define REGULATOR_SUPPLY_TYPE        1
/* gpio controlled LDO based supply */
#define LDO_SUPPLY_TYPE              0

struct bma254_platform_data {
	int accel_position;
	 /* Change axis or not for user-level
	 * If it is true, driver reports adjusted axis-raw-data
	 * to user-space based on accel_get_position() value,
	 * or if it is false, driver reports original axis-raw-data */
	bool axis_adjust;

	int vdd_supply_type;
	struct regulator *vdd_regulator;
	int vdd_regulator_volt;
	int vdd_ldo_en;

	int gpio_int;

	int (*power)(struct device *, bool);
};

static const u8 bma254_valid_range[] = {
	BMA254_RANGE_2G,
	BMA254_RANGE_4G,
	BMA254_RANGE_8G,
	BMA254_RANGE_16G,
};

static const u8 bma254_valid_bw[] = {
	BMA254_BW_7_81HZ,
	BMA254_BW_15_63HZ,
	BMA254_BW_31_25HZ,
	BMA254_BW_62_50HZ,
	BMA254_BW_125HZ,
	BMA254_BW_250HZ,
	BMA254_BW_500HZ,
	BMA254_BW_1000HZ,
};

struct bma254acc {
	s16	x,
		y,
		z;
};

struct bma254_data {

	struct i2c_client *bma254_client;
	struct input_dev *input;
	struct device *bma254_device;

	atomic_t delay;
	atomic_t enable;
	unsigned char mode;

	struct bma254acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;

	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_INPUT_BMA254_ACC_ALERT_INT
	struct mutex data_mutex;
	struct work_struct alert_work;
	struct wakeup_source reactive_wakeup_source;
	atomic_t reactive_state;
	atomic_t reactive_enable;
	atomic_t factory_test;
	bool factory_mode;
	int IRQ;
#endif
	struct bma254_platform_data *pdata;
	atomic_t selftest_result;
	struct device *dev;
	int position;
	bool axis_adjust;
};

#endif
