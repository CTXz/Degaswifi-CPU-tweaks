/*
 *
 * Zinitix bt432 touchscreen driver
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#define TSP_VERBOSE_DEBUG
#ifdef CONFIG_MACH_PXA_SAMSUNG
#define SEC_FACTORY_TEST
#endif
#define SUPPORTED_TOUCH_KEY
#define TOUCH_BOOSTER 0

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/input/bt432_ts.h>
#include <linux/input/mt.h>
#include <linux/regulator/machine.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#ifdef CONFIG_MACH_PXA_SAMSUNG
#include <linux/sec-common.h>
#endif
#include <asm/io.h>

#if defined(CONFIG_PM_RUNTIME)
#include <linux/pm_runtime.h>
#endif
#if TOUCH_BOOSTER
#include <linux/pm_qos.h>
#endif

#include "zinitix_touch_bt432_firmware.h"

#define ZINITIX_DEBUG			0
#define ZINITIX_I2C_CHECKSUM		1
#define TOUCH_POINT_MODE		0
#define MAX_SUPPORTED_FINGER_NUM	5 /* max 10 */

#ifdef SUPPORTED_TOUCH_KEY
#define MAX_SUPPORTED_BUTTON_NUM	2 /* max 8 */
#define SUPPORTED_BUTTON_NUM		2
#endif

/* Upgrade Method*/
#define TOUCH_ONESHOT_UPGRADE		0

/* resolution offset */
#define ABS_PT_OFFSET			(-1)

#define TOUCH_FORCE_UPGRADE		1
#define USE_CHECKSUM			0
#define CHECK_HWID			0

#define CHIP_OFF_DELAY			50 /*ms*/
#define CHIP_ON_DELAY			15 /*ms*/
#define FIRMWARE_ON_DELAY		40 /*ms*/

#define DELAY_FOR_SIGNAL_DELAY		30 /*us*/
#define DELAY_FOR_TRANSCATION		50
#define DELAY_FOR_POST_TRANSCATION	10

/* PMIC Regulator based supply to TSP */
#define TSP_REGULATOR_SUPPLY		1
/* gpio controlled LDO based supply to TSP */
#define TSP_LDO_SUPPLY			0

enum power_control {
	POWER_OFF,
	POWER_ON,
	POWER_ON_SEQUENCE,
};

/* Key Enum */
enum key_event {
	ICON_BUTTON_UNCHANGE,
	ICON_BUTTON_DOWN,
	ICON_BUTTON_UP,
};

/* ESD Protection */
/*second : if 0, no use. If you have to use, 3 is recommended*/
#define ESD_TIMER_INTERVAL		0
#define SCAN_RATE_HZ			100
#define CHECK_ESD_TIMER			3

 /*Test Mode (Monitoring Raw Data) */
#define SEC_DND_N_COUNT			20
#define SEC_DND_FREQUENCY		33
#define SEC_PDND_N_COUNT		16
#define SEC_PDND_U_COUNT		12
#define SEC_PDND_FREQUENCY		46

/* 19x10*/  /*448  28x16*/ /*576  32x18 */
#define MAX_RAW_DATA_SZ			190
#define MAX_TRAW_DATA_SZ	\
	(MAX_RAW_DATA_SZ + 4*MAX_SUPPORTED_FINGER_NUM + 2)

/* preriod raw data interval */
#define RAWDATA_DELAY_FOR_HOST		100

struct raw_ioctl {
	int sz;
	u8 *buf;
};

struct reg_ioctl {
	int addr;
	int *val;
};

#define TOUCH_SEC_MODE			48
#define TOUCH_REF_MODE			10
#define TOUCH_NORMAL_MODE		5
#define TOUCH_DELTA_MODE		3
#define TOUCH_DND_MODE			6
#define TOUCH_PDND_MODE			11

#define INIT_RETRY_CNT			1
#define I2C_SUCCESS			0
#define I2C_FAIL			1

/* Register Map*/
#define BT432_SWRESET_CMD				0x0000
#define BT432_WAKEUP_CMD				0x0001

#define BT432_IDLE_CMD					0x0004
#define BT432_SLEEP_CMD					0x0005

#define BT432_CLEAR_INT_STATUS_CMD			0x0003
#define BT432_CALIBRATE_CMD				0x0006
#define BT432_SAVE_STATUS_CMD				0x0007
#define BT432_SAVE_CALIBRATION_CMD			0x0008
#define BT432_RECALL_FACTORY_CMD			0x000f

#define BT432_THRESHOLD					0x0020

#define BT432_DEBUG_REG					0x0115 /* 0~7 */

#define BT432_TOUCH_MODE				0x0010
#define BT432_CHIP_REVISION				0x0011
#define BT432_FIRMWARE_VERSION				0x0012

#define BT432_MINOR_FW_VERSION				0x0121

#define BT432_VENDOR_ID					0x001C
#define BT432_HW_ID					0x0014

#define BT432_DATA_VERSION_REG				0x0013
#define BT432_SUPPORTED_FINGER_NUM			0x0015
#define BT432_EEPROM_INFO				0x0018
#define BT432_INITIAL_TOUCH_MODE			0x0019

#define BT432_TOTAL_NUMBER_OF_X				0x0060
#define BT432_TOTAL_NUMBER_OF_Y				0x0061

#define BT432_DELAY_RAW_FOR_HOST			0x007f

#define BT432_BUTTON_SUPPORTED_NUM			0x00B0
#define BT432_BUTTON_SENSITIVITY			0x00B2
#define BT432_DUMMY_BUTTON_SENSITIVITY			0X00C8

#define BT432_X_RESOLUTION				0x00C0
#define BT432_Y_RESOLUTION				0x00C1

#define BT432_POINT_STATUS_REG				0x0080
#define BT432_ICON_STATUS_REG				0x00AA

#define BT432_AFE_FREQUENCY				0x0100
#define BT432_DND_N_COUNT				0x0122
#define BT432_DND_U_COUNT				0x0135

#define BT432_RAWDATA_REG				0x0200

#define BT432_EEPROM_INFO_REG				0x0018

#define BT432_INT_ENABLE_FLAG				0x00f0
#define BT432_PERIODICAL_INTERRUPT_INTERVAL		0x00f1

#define BT432_BTN_WIDTH					0x016d

#define BT432_CHECKSUM_RESULT				0x012c

#define BT432_INIT_FLASH				0x01d0
#define BT432_WRITE_FLASH				0x01d1
#define BT432_READ_FLASH				0x01d2

#define ZINITIX_INTERNAL_FLAG_02			0x011e

/* Interrupt & status register flag bit
-------------------------------------------------
*/
#define BIT_PT_CNT_CHANGE	0
#define BIT_DOWN		1
#define BIT_MOVE		2
#define BIT_UP			3
#define BIT_PALM		4
#define BIT_PALM_REJECT		5
#define RESERVED_0		6
#define RESERVED_1		7
#define BIT_WEIGHT_CHANGE	8
#define BIT_PT_NO_CHANGE	9
#define BIT_REJECT		10
#define BIT_PT_EXIST		11
#define RESERVED_2		12
#define BIT_MUST_ZERO		13
#define BIT_DEBUG		14
#define BIT_ICON_EVENT		15

/* button */
#define BIT_O_ICON0_DOWN	0
#define BIT_O_ICON1_DOWN	1
#define BIT_O_ICON2_DOWN	2
#define BIT_O_ICON3_DOWN	3
#define BIT_O_ICON4_DOWN	4
#define BIT_O_ICON5_DOWN	5
#define BIT_O_ICON6_DOWN	6
#define BIT_O_ICON7_DOWN	7

#define BIT_O_ICON0_UP		8
#define BIT_O_ICON1_UP		9
#define BIT_O_ICON2_UP		10
#define BIT_O_ICON3_UP		11
#define BIT_O_ICON4_UP		12
#define BIT_O_ICON5_UP		13
#define BIT_O_ICON6_UP		14
#define BIT_O_ICON7_UP		15


#define SUB_BIT_EXIST		0
#define SUB_BIT_DOWN		1
#define SUB_BIT_MOVE		2
#define SUB_BIT_UP		3
#define SUB_BIT_UPDATE		4
#define SUB_BIT_WAIT		5


#define zinitix_bit_set(val, n)		((val) &= ~(1<<(n)), (val) |= (1<<(n)))
#define zinitix_bit_clr(val, n)		((val) &= ~(1<<(n)))
#define zinitix_bit_test(val, n)	((val) & (1<<(n)))
#define zinitix_swap_v(a, b, t)		((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)		(((((s) & 0xff) << 8) | (((s) >> 8) \
								& 0xff)))

#ifdef SEC_FACTORY_TEST
/* Touch Screen */
#define TSP_CMD_STR_LEN			32
#define TSP_CMD_RESULT_STR_LEN		512
#define TSP_CMD_PARAM_NUM		8
#define TSP_CMD_Y_NUM			10
#define TSP_CMD_X_NUM			20
#define TSP_CMD_NODE_NUM		(TSP_CMD_Y_NUM * TSP_CMD_X_NUM)

struct tsp_factory_info {
	struct list_head cmd_list_head;
	char cmd[TSP_CMD_STR_LEN];
	char cmd_param[TSP_CMD_PARAM_NUM];
	char cmd_result[TSP_CMD_RESULT_STR_LEN];
	char cmd_buff[TSP_CMD_RESULT_STR_LEN];
	struct mutex cmd_lock;
	bool cmd_is_running;
	u8 cmd_state;
};

struct tsp_raw_data {
	u16 ref_data[TSP_CMD_NODE_NUM];
	u16 pref_data[TSP_CMD_NODE_NUM];
	s16 delta_data[TSP_CMD_NODE_NUM];
};

enum {
	WAITING = 0,
	RUNNING,
	OK,
	FAIL,
	NOT_APPLICABLE,
};

struct tsp_cmd {
	struct list_head list;
	const char *cmd_name;
	void (*cmd_func)(void *device_data);
};

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void not_support_cmd(void *device_data);

/* Vendor dependant command */
static void run_reference_read(void *device_data);
static void get_reference(void *device_data);
static void run_preference_read(void *device_data);
static void get_preference(void *device_data);
static void run_delta_read(void *device_data);
static void get_delta(void *device_data);

#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

static struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", module_off_slave),},
	{TSP_CMD("module_on_slave", module_on_slave),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},

	/* vendor dependant command */
	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("run_dnd_read", run_preference_read),},
	{TSP_CMD("get_dnd", get_preference),},
	{TSP_CMD("run_delta_read", run_delta_read),},
	{TSP_CMD("get_delta", get_delta),},
};

#endif

#define TSP_NORMAL_EVENT_MSG 1
static int m_ts_debug_mode = ZINITIX_DEBUG;

struct coord {
	u16	x;
	u16	y;
	u8	width;
	u8	sub_status;
#if (TOUCH_POINT_MODE == 2)
	u8	minor_width;
	u8	angle;
#endif
};

struct point_info {
	u16	status;
#if (TOUCH_POINT_MODE == 1)
	u16	event_flag;
#else
	u8	finger_cnt;
	u8	time_stamp;
#endif
	struct coord coord[MAX_SUPPORTED_FINGER_NUM];
};

#define TOUCH_V_FLIP	0x01
#define TOUCH_H_FLIP	0x02
#define TOUCH_XY_SWAP	0x04

struct capa_info {
	u16	chip_code;
	u16	vendor_id;
	u16	ic_revision;
	u16	fw_version;
	u16	fw_minor_version;
	u16	reg_data_version;
	u16	threshold;
	u16	key_threshold;
	u16	dummy_threshold;
	u32	ic_fw_size;
	u32	MaxX;
	u32	MaxY;
	u32	MinX;
	u32	MinY;
	u8	gesture_support;
	u16	multi_fingers;
	u16	button_num;
	u16	ic_int_mask;
	u16	x_node_num;
	u16	y_node_num;
	u16	total_node_num;
	u16	hw_id;
	u16	afe_frequency;
	u16	N_cnt;
	u16	U_cnt;
	u16	i2s_checksum;
};

enum work_state {
	NOTHING = 0,
	NORMAL,
	ESD_TIMER,
	SUSPEND,
	RESUME,
	UPGRADE,
	REMOVE,
	SET_MODE,
	HW_CALIBRAION,
	RAW_DATA,
};

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct bt432_ts_info {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	struct bt432_ts_platform_data *pdata;
	char			phys[32];
	struct capa_info	cap_info;
	struct point_info	touch_info;
	struct point_info	reported_touch_info;
	u16			icon_event_reg;
	u16			prev_icon_event;
	int			irq;
	u8			button[MAX_SUPPORTED_BUTTON_NUM];
	u8			work_state;
	struct semaphore	work_lock;
#if TOUCH_BOOSTER
	struct pm_qos_request	cpufreq_qos_req_min;
	u8			finger_cnt;
	bool			double_booster;
#endif

#if ESD_TIMER_INTERVAL
	struct workqueue_struct	*esd_tmr_workqueue;
	struct work_struct	tmr_work;
	struct timer_list	esd_timeout_tmr;
	struct timer_list	*p_esd_timeout_tmr;
	spinlock_t lock;
#endif
	struct semaphore	raw_data_lock;
	u16			touch_mode;
	s16			cur_data[MAX_TRAW_DATA_SZ];
	u8			update;

#ifdef SEC_FACTORY_TEST
	struct tsp_factory_info	*factory_info;
	struct tsp_raw_data	*raw_data;
#endif
};

u8 *m_firmware_data;

/* key button mapping*/
u32 BUTTON_MAPPING_KEY[MAX_SUPPORTED_BUTTON_NUM] = {
	KEY_MENU,
	KEY_BACK
};

/* define i2c sub functions*/
static inline s32 read_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
retry:
	/* select register*/
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		udelay(1000);

		if (++count < 8)
			goto retry;

		return ret;
	}
	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_data(struct i2c_client *client,
	u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;
	u8 pkt[10]; /* max packet */
	pkt[0] = (reg) & 0xff; /* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

retry:
	ret = i2c_master_send(client , pkt , length + 2);
	if (ret < 0) {
		udelay(1000);

		if (++count < 8)
			goto retry;

		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 write_reg(struct i2c_client *client, u16 reg, u16 value)
{
	if (write_data(client, reg, (u8 *)&value, 2) < 0)
		return I2C_FAIL;

	return I2C_SUCCESS;
}

static inline s32 write_cmd(struct i2c_client *client, u16 reg)
{
	s32 ret;
	int count = 0;

retry:
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		udelay(1000);

		if (++count < 8)
			goto retry;

		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);
	return I2C_SUCCESS;
}

static inline s32 read_raw_data(struct i2c_client *client,
		u16 reg, u8 *values, u16 length)
{
	s32 ret;
	int count = 0;

retry:
	/* select register */
	ret = i2c_master_send(client , (u8 *)&reg , 2);
	if (ret < 0) {
		udelay(1000);

		if (++count < 8)
			goto retry;

		return ret;
	}

	/* for setup tx transaction. */
	udelay(200);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static inline s32 read_firmware_data(struct i2c_client *client,
	u16 addr, u8 *values, u16 length)
{
	s32 ret;
	/* select register*/

	ret = i2c_master_send(client , (u8 *)&addr , 2);
	if (ret < 0)
		return ret;

	/* for setup tx transaction. */
	udelay(1000);

	ret = i2c_master_recv(client , values , length);
	if (ret < 0)
		return ret;

	udelay(DELAY_FOR_POST_TRANSCATION);
	return length;
}

static bool bt432_power_control(struct bt432_ts_info *info, u8 ctl);

static bool init_touch(struct bt432_ts_info *info, bool force);
static bool mini_init_touch(struct bt432_ts_info *info);
static void clear_report_data(struct bt432_ts_info *info);
#if ESD_TIMER_INTERVAL
static void esd_timer_start(u16 sec, struct bt432_ts_info *info);
static void esd_timer_stop(struct bt432_ts_info *info);
static void esd_timer_init(struct bt432_ts_info *info);
static void esd_timeout_handler(unsigned long data);
#endif

static long ts_misc_fops_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg);
static int ts_misc_fops_open(struct inode *inode, struct file *filp);
static int ts_misc_fops_close(struct inode *inode, struct file *filp);

static const struct file_operations ts_misc_fops = {
	.owner = THIS_MODULE,
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_close,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zinitix_touch_misc",
	.fops = &ts_misc_fops,
};

#define TOUCH_IOCTL_BASE	0xbc
#define TOUCH_IOCTL_GET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 0, int)
#define TOUCH_IOCTL_SET_DEBUGMSG_STATE		_IOW(TOUCH_IOCTL_BASE, 1, int)
#define TOUCH_IOCTL_GET_CHIP_REVISION		_IOW(TOUCH_IOCTL_BASE, 2, int)
#define TOUCH_IOCTL_GET_FW_VERSION		_IOW(TOUCH_IOCTL_BASE, 3, int)
#define TOUCH_IOCTL_GET_REG_DATA_VERSION	_IOW(TOUCH_IOCTL_BASE, 4, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_SIZE		_IOW(TOUCH_IOCTL_BASE, 5, int)
#define TOUCH_IOCTL_VARIFY_UPGRADE_DATA		_IOW(TOUCH_IOCTL_BASE, 6, int)
#define TOUCH_IOCTL_START_UPGRADE		_IOW(TOUCH_IOCTL_BASE, 7, int)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 8, int)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 9, int)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM		_IOW(TOUCH_IOCTL_BASE, 10, int)
#define TOUCH_IOCTL_SET_RAW_DATA_MODE		_IOW(TOUCH_IOCTL_BASE, 11, int)
#define TOUCH_IOCTL_GET_RAW_DATA		_IOW(TOUCH_IOCTL_BASE, 12, int)
#define TOUCH_IOCTL_GET_X_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 13, int)
#define TOUCH_IOCTL_GET_Y_RESOLUTION		_IOW(TOUCH_IOCTL_BASE, 14, int)
#define TOUCH_IOCTL_HW_CALIBRAION		_IOW(TOUCH_IOCTL_BASE, 15, int)
#define TOUCH_IOCTL_GET_REG			_IOW(TOUCH_IOCTL_BASE, 16, int)
#define TOUCH_IOCTL_SET_REG			_IOW(TOUCH_IOCTL_BASE, 17, int)
#define TOUCH_IOCTL_SEND_SAVE_STATUS		_IOW(TOUCH_IOCTL_BASE, 18, int)
#define TOUCH_IOCTL_DONOT_TOUCH_EVENT		_IOW(TOUCH_IOCTL_BASE, 19, int)

struct bt432_ts_info *misc_info;

static bool get_raw_data(struct bt432_ts_info *info, u8 *buff, int skip_cnt)
{
	struct i2c_client *client = info->client;
	struct bt432_ts_platform_data *pdata = info->pdata;
	u32 total_node = info->cap_info.total_node_num;
	u32 sz;
	int i;

	disable_irq(info->irq);

	down(&info->work_lock);
	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
				__func__, info->work_state);

		goto out;
	}

	info->work_state = RAW_DATA;

	for (i = 0; i < skip_cnt; i++) {
		while (gpio_get_value(pdata->gpio_int))
			usleep_range(1000, 1000);
		write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
		usleep_range(1000, 1000);
	}

	sz = total_node * 2;

	while (gpio_get_value(pdata->gpio_int))
		usleep_range(1000, 1000);

	if (read_raw_data(client, BT432_RAWDATA_REG, (char *)buff, sz) < 0) {
		dev_err(&client->dev, "%s: Failed to read raw data\n",
			__func__);
		info->work_state = NOTHING;

		goto out;
	}

	write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return true;

out:
	enable_irq(info->irq);
	up(&info->work_lock);

	return false;
}

static bool ts_get_raw_data(struct bt432_ts_info *info)
{
	struct i2c_client *client = info->client;
	u32 total_node = info->cap_info.total_node_num;
	u32 sz;

	if (down_trylock(&info->raw_data_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n",
			__func__);
		info->touch_info.status = 0;

		return true;
	}

	sz = total_node * 2 + sizeof(struct point_info);

	if (read_raw_data(info->client, BT432_RAWDATA_REG,
			(char *)info->cur_data, sz) < 0) {
		dev_err(&client->dev, "%s: Failed to read raw data\n",
			__func__);
		up(&info->raw_data_lock);

		return false;
	}

	info->update = 1;
	memcpy((u8 *)(&info->touch_info), (u8 *)&info->cur_data[total_node],
			sizeof(struct point_info));
	up(&info->raw_data_lock);

	return true;
}

#if	ZINITIX_I2C_CHECKSUM
#define ZINITIX_I2C_CHECKSUM_WCNT 0x016a
#define ZINITIX_I2C_CHECKSUM_RESULT 0x016c
static bool i2c_checksum(struct bt432_ts_info *info, s16 *pChecksum,
			u16 wlength)
{
	s16 checksum_result;
	s16 checksum_cur;
	int i;

	checksum_cur = 0;
	for (i = 0; i < wlength; i++)
		checksum_cur += (s16)pChecksum[i];
	if (read_data(info->client,
		ZINITIX_I2C_CHECKSUM_RESULT,
		(u8 *)(&checksum_result), 2) < 0) {
		pr_err("error read i2c checksum rsult.\n");
		return false;
	}
	if (checksum_cur != checksum_result) {
		pr_err("checksum error : %d, %d\n", checksum_cur,
			checksum_result);
		return false;
	}
	return true;
}

#endif
static bool ts_read_coord(struct bt432_ts_info *info)
{
	struct i2c_client *client = info->client;
#if (TOUCH_POINT_MODE == 1)
	int i;
#endif
	/* for  Debugging Tool */
	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (ts_get_raw_data(info) == false)
			return false;
		dev_err(&client->dev, "status = 0x%04X\n",
			info->touch_info.status);

		goto out;
	}

#if (TOUCH_POINT_MODE == 1)
	memset(&info->touch_info,
			0x0, sizeof(struct point_info));

#if	ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (write_reg(info->client,
			ZINITIX_I2C_CHECKSUM_WCNT, 2) != I2C_SUCCESS)
			return false;

#endif
	if (read_data(info->client, BT432_POINT_STATUS_REG,
			(u8 *)(&info->touch_info), 4) < 0) {
		dev_err(&client->dev, "Failed to read point info\n");

		return false;
	}

#if	ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (i2c_checksum(info, (s16 *)(&info->touch_info), 2) == false)
			return false;
#endif
	if (info->touch_info.event_flag == 0)
		goto out;

#if	ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (write_reg(info->client,
			ZINITIX_I2C_CHECKSUM_WCNT,
			sizeof(struct point_info)/2) != I2C_SUCCESS)
			return false;
#endif
	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		if (zinitix_bit_test(info->touch_info.event_flag, i)) {
			udelay(20);

			if (read_data(info->client,
				BT432_POINT_STATUS_REG + 2 + (i * 4),
				(u8 *)(&info->touch_info.coord[i]),
				sizeof(struct coord)) < 0) {
				dev_err(&client->dev, "Failed to read point" \
					"info\n");
				return false;
			}
#if	ZINITIX_I2C_CHECKSUM
			if (info->cap_info.i2s_checksum)
				if (i2c_checksum(info,
					(s16 *)(&info->touch_info.coord[i]),
					sizeof(struct point_info)/2) == false)
					return false;
#endif
		}
}
#else
#if	ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (write_reg(info->client,
				ZINITIX_I2C_CHECKSUM_WCNT,
				(sizeof(struct point_info)/2)) != I2C_SUCCESS) {
			dev_err(&client->dev, "error write checksum wcnt.\n");
			return false;
	}
#endif
	if (read_data(info->client,
			BT432_POINT_STATUS_REG,
			(u8 *)(&info->touch_info),
			sizeof(struct point_info)) < 0) {
		dev_err(&client->dev, "Failed to read point info\n");

		return false;
	}
#if	ZINITIX_I2C_CHECKSUM
	if (info->cap_info.i2s_checksum)
		if (i2c_checksum(info, (s16 *)(&info->touch_info),
				sizeof(struct point_info)/2) == false)
			return false;
#endif
#endif
out:
	/* error */
	if (zinitix_bit_test(info->touch_info.status, BIT_MUST_ZERO)) {
		dev_err(&client->dev, "Invalid must zero bit(%04x)\n",
			info->touch_info.status);

		return false;
	}

	write_cmd(info->client, BT432_CLEAR_INT_STATUS_CMD);

	return true;
}

#if ESD_TIMER_INTERVAL
static void esd_timeout_handler(unsigned long data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)data;

	info->p_esd_timeout_tmr = NULL;
	queue_work(info->esd_tmr_workqueue, &info->tmr_work);
}

static void esd_timer_start(u16 sec, struct bt432_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr != NULL)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif
	info->p_esd_timeout_tmr = NULL;
	init_timer(&(info->esd_timeout_tmr));
	info->esd_timeout_tmr.data = (unsigned long)(info);
	info->esd_timeout_tmr.function = esd_timeout_handler;
	info->esd_timeout_tmr.expires = jiffies + (HZ * sec);
	info->p_esd_timeout_tmr = &info->esd_timeout_tmr;
	add_timer(&info->esd_timeout_tmr);
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_stop(struct bt432_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	if (info->p_esd_timeout_tmr)
#ifdef CONFIG_SMP
		del_singleshot_timer_sync(info->p_esd_timeout_tmr);
#else
		del_timer(info->p_esd_timeout_tmr);
#endif

	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void esd_timer_init(struct bt432_ts_info *info)
{
	unsigned long flags;
	spin_lock_irqsave(&info->lock, flags);
	init_timer(&(info->esd_timeout_tmr));
	info->esd_timeout_tmr.data = (unsigned long)(info);
	info->esd_timeout_tmr.function = esd_timeout_handler;
	info->p_esd_timeout_tmr = NULL;
	spin_unlock_irqrestore(&info->lock, flags);
}

static void ts_tmr_work(struct work_struct *work)
{
	struct bt432_ts_info *info =
			container_of(work, struct bt432_ts_info, tmr_work);
	struct i2c_client *client = info->client;

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n",
			__func__);
		esd_timer_start(CHECK_ESD_TIMER, info);

		return;
	}

	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied (%d)\n",
			__func__, info->work_state);
		up(&info->work_lock);

		return;
	}
	info->work_state = ESD_TIMER;

	disable_irq(info->irq);
	bt432_power_control(info, POWER_OFF);
	bt432_power_control(info, POWER_ON_SEQUENCE);

	clear_report_data(info);
	if (mini_init_touch(info) == false)
		goto fail_time_out_init;

	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);
#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "tmr queue work\n");
#endif

	return;

fail_time_out_init:
	dev_err(&client->dev, "Failed to restart\n");
	esd_timer_start(CHECK_ESD_TIMER, info);
	info->work_state = NOTHING;
	enable_irq(info->irq);
	up(&info->work_lock);

	return;
}
#endif

static bool bt432_power_sequence(struct bt432_ts_info *info)
{
	struct i2c_client *client = info->client;
	int retry = 0;

retry_power_sequence:
	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS)
		goto fail_power_sequence;

	udelay(10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS)
		goto fail_power_sequence;

	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS)
		goto fail_power_sequence;

	udelay(2000);

	if (write_reg(client, 0xc001, 0x0001) != I2C_SUCCESS)
		goto fail_power_sequence;

	/* wait for checksum cal */
	usleep_range(FIRMWARE_ON_DELAY*1000, FIRMWARE_ON_DELAY*1000);
	return true;

fail_power_sequence:
	if (retry++ < 3) {
		usleep_range(CHIP_ON_DELAY*1000, CHIP_ON_DELAY*1000);
		dev_info(&client->dev, "retry = %d\n", retry);

		goto retry_power_sequence;
	}

	dev_err(&client->dev, "Failed to send power sequence\n");

	return false;
}

static int bt432_hw_power(struct bt432_ts_info *info, u8 on)
{
	int ret = 0;
	static u8 is_power_on;
	static struct regulator *touch_regulator;
	struct bt432_ts_platform_data *pdata = info->pdata;

	if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY) {
		if (unlikely(!touch_regulator)) {
			touch_regulator = regulator_get(&info->client->dev, "tsp_vdd");
			if (IS_ERR(touch_regulator)) {
				pr_err("[TSP]: %s: get touch_regulator error\n",
					__func__);
				goto err_regulator_get;
			}
		}
	}
	if (on == 0) {
		if (is_power_on) {
			is_power_on = 0;
			if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY) {
				ret = regulator_disable(touch_regulator);
				if (unlikely(ret)) {
					is_power_on = 1;
					pr_err("[TSP]: %s: touch_regulator disable " \
						"failed  (%d)\n", __func__, ret);
					goto err_regulator;
				}
			}
			if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
				if (pdata->tsp_en_gpio)
					gpio_direction_output(pdata->tsp_en_gpio, 0);
			}
		}
	} else {
		if (!is_power_on) {
			is_power_on = 1;
			if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY) {
				regulator_set_voltage(touch_regulator, 3300000, 3300000);
				ret = regulator_enable(touch_regulator);
				if (unlikely(ret)) {
					is_power_on = 0;
					pr_err("[TSP]: %s: touch_regulator enable "\
						"failed (%d)\n", __func__, ret);
					goto err_regulator;
				}
			}
			if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
				if (pdata->tsp_en_gpio)
					gpio_direction_output(pdata->tsp_en_gpio, 1);
			}
		}
	}
	pr_info("[TSP]: %s, expected power[%d], actural power[%d]\n",
		__func__, on, is_power_on);
	return 0;

err_regulator:
	if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY)
		regulator_put(touch_regulator);
err_regulator_get:
	if (pdata->tsp_supply_type == TSP_REGULATOR_SUPPLY)
		touch_regulator = NULL;
	return -EIO;
}

static bool bt432_power_control(struct bt432_ts_info *info, u8 ctl)
{
	int ret = 0;
	pr_info("[TSP] %s, %d\n", __func__, ctl);

	ret = bt432_hw_power(info, ctl);
	if (unlikely(ret))
		return false;

	if (ctl == POWER_ON_SEQUENCE) {
		usleep_range(CHIP_ON_DELAY*1000, CHIP_ON_DELAY*1000);
		return bt432_power_sequence(info);
	}
	return true;
}

#if TOUCH_ONESHOT_UPGRADE
static bool ts_check_need_upgrade(struct bt432_ts_info *info,
		u16 version, u16 minor_version, u16 reg_version, u16 hw_id)
{
	struct i2c_client *client = info->client;
	u16	new_version;
	u16	new_minor_version;
	u16	new_reg_version;
#if CHECK_HWID
	u16	new_hw_id;
	if (info->cap_info.chip_code == 0xf400)
		new_hw_id = (u16) (m_firmware_data[0x6b12] |
				(m_firmware_data[0x6b13]<<8));
	else
		new_hw_id =  (u16)(m_firmware_data[0x57d2] |
				(m_firmware_data[0x57d3]<<8));

	dev_info(&client->dev, "HW_ID = 0x%x, new HW_ID = 0x%x\n",
				hw_id, new_hw_id);
	if (hw_id != new_hw_id)
		return false;
#endif

	new_version = (u16) (m_firmware_data[52] |
				(m_firmware_data[53]<<8));
	new_minor_version = (u16) (m_firmware_data[56] |
				(m_firmware_data[57]<<8));
	new_reg_version = (u16) (m_firmware_data[60] |
				(m_firmware_data[61]<<8));

#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "version = 0x%x, new version = 0x%x\n",
		version, new_version);
	dev_info(&client->dev, "minor version = 0x%x, new minor version =" \
		" 0x%x\n", minor_version, new_minor_version);
	dev_info(&client->dev, "reg data version = 0x%x, new reg data" \
		" version = 0x%x\n", reg_version, new_reg_version);
#endif
	if (version < new_version ||
		minor_version < new_minor_version ||
		reg_version < new_reg_version)
		return true;
	else
		return false;
}
#endif

#define TC_SECTOR_SZ		8

static bool ts_upgrade_firmware(struct bt432_ts_info *info,
	const u8 *firmware_data, u32 size)
{
	struct bt432_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	u16 flash_addr;
	u8 *verify_data;
	int retry_cnt = 0;
	int i;

	verify_data = kzalloc(size, GFP_KERNEL);
	if (unlikely(!verify_data)) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n",
			__func__);

		return false;
	}

retry_upgrade:
	bt432_power_control(info, POWER_OFF);
	bt432_power_control(info, POWER_ON);
	mdelay(100);

	if (write_reg(client, 0xc000, 0x0001) != I2C_SUCCESS)
		goto fail_upgrade;

	udelay(10);

	if (write_cmd(client, 0xc004) != I2C_SUCCESS)
		goto fail_upgrade;

	udelay(10);

	if (write_reg(client, 0xc002, 0x0001) != I2C_SUCCESS)
		goto fail_upgrade;

	mdelay(5);

#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "init flash\n");
#endif

	if (write_reg(client, 0xc003, 0x0001) != I2C_SUCCESS)
		goto fail_upgrade;


	if (write_reg(client, 0xc104, 0x0001) != I2C_SUCCESS)
		goto fail_upgrade;


	if (write_cmd(client, BT432_INIT_FLASH) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to init flash\n");

		goto fail_upgrade;
	}

	dev_info(&client->dev, "Write firmware data\n");
	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < pdata->page_size / TC_SECTOR_SZ; i++) {
			if (write_data(client, BT432_WRITE_FLASH,
				(u8 *)&firmware_data[flash_addr],
				TC_SECTOR_SZ) < 0) {
				dev_err(&client->dev,
					"Failed to write firmare\n");

				goto fail_upgrade;
			}
			flash_addr += TC_SECTOR_SZ;
			udelay(100);
		}

		mdelay(30); /*for fuzing delay*/
	}

	if (write_reg(client, 0xc003, 0x0000) != I2C_SUCCESS)
		goto fail_upgrade;


	if (write_reg(client, 0xc104, 0x0000) != I2C_SUCCESS)
		goto fail_upgrade;


#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "init flash\n");
#endif

	if (write_cmd(client, BT432_INIT_FLASH) != I2C_SUCCESS) {
		dev_err(&client->dev, "Failed to init flash\n");

		goto fail_upgrade;
	}

	dev_info(&client->dev, "Read firmware data\n");

	for (flash_addr = 0; flash_addr < size; ) {
		for (i = 0; i < pdata->page_size / TC_SECTOR_SZ; i++) {
			if (read_firmware_data(client, BT432_READ_FLASH,
					(u8 *)&verify_data[flash_addr],
					TC_SECTOR_SZ) < 0) {
					dev_err(&client->dev,
					"Failed to read firmare\n");
				goto fail_upgrade;
			}

			flash_addr += TC_SECTOR_SZ;
		}
	}
	/* verify */
	dev_info(&client->dev, "verify firmware data\n");
	if (memcmp((u8 *)&firmware_data[0],
		(u8 *)&verify_data[0], size) == 0) {
		dev_info(&client->dev, "upgrade finished\n");
		kfree(verify_data);
		bt432_power_control(info, POWER_OFF);
		bt432_power_control(info, POWER_ON_SEQUENCE);

		return true;
	}

fail_upgrade:
	bt432_power_control(info, POWER_OFF);

	if (retry_cnt++ < INIT_RETRY_CNT) {
		dev_err(&client->dev, "Failed to upgrade. retry (%d)\n",
			retry_cnt);
		goto retry_upgrade;
	}

	if (verify_data != NULL)
		kfree(verify_data);

	dev_info(&client->dev, "Failed to upgrade\n");

	return false;
}

static bool ts_hw_calibration(struct bt432_ts_info *info)
{
	struct i2c_client *client = info->client;
	u16	chip_eeprom_info;
	int time_out = 0;

	if (write_reg(client, BT432_TOUCH_MODE, 0x07) != I2C_SUCCESS)
		return false;

	mdelay(10);
	write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
	mdelay(10);
	write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
	mdelay(50);
	write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
	mdelay(10);

	if (write_cmd(client, BT432_CALIBRATE_CMD) != I2C_SUCCESS)
		return false;

	if (write_cmd(client, BT432_CLEAR_INT_STATUS_CMD) != I2C_SUCCESS)
		return false;

	mdelay(10);
	write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);

	/* wait for h/w calibration*/
	do {
		mdelay(500);
		write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);

		if (read_data(client, BT432_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0)
			return false;

#if defined(TSP_VERBOSE_DEBUG)
		dev_info(&client->dev, "touch eeprom info = 0x%04X\n",
			chip_eeprom_info);
#endif
		if (!zinitix_bit_test(chip_eeprom_info, 0))
			break;

		if (time_out++ == 4) {
			write_cmd(client, BT432_CALIBRATE_CMD);
			mdelay(10);
			write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
			dev_err(&client->dev, "h/w calibration retry" \
				"timeout.\n");
		}

		if (time_out > 10) {
			dev_err(&client->dev, "h/w calibration timeout.\n");
			break;
		}
	} while (1);

	if (write_reg(client, BT432_TOUCH_MODE, TOUCH_POINT_MODE)
						!= I2C_SUCCESS)
		return false;

	if (info->cap_info.ic_int_mask != 0) {
		if (write_reg(client, BT432_INT_ENABLE_FLAG,
			info->cap_info.ic_int_mask) != I2C_SUCCESS)
			return false;
	}

	write_reg(client, 0xc003, 0x0001);
	write_reg(client, 0xc104, 0x0001);
	udelay(100);
	if (write_cmd(client, BT432_SAVE_CALIBRATION_CMD) != I2C_SUCCESS)
		return false;

	mdelay(1000);
	write_reg(client, 0xc003, 0x0000);
	write_reg(client, 0xc104, 0x0000);

	return true;
}

static bool init_touch(struct bt432_ts_info *info, bool force)
{
	struct bt432_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	u16 reg_val;
	int i;
	u16 chip_eeprom_info;
#if USE_CHECKSUM
	u16 chip_check_sum;
	u8 checksum_err;
#endif
	int retry_cnt = 0;

retry_init:
	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT432_EEPROM_INFO_REG,
			(u8 *)&chip_eeprom_info, 2) < 0) {
			dev_err(&client->dev, "Failed to read eeprom info" \
				"(%d)\n", i);
			mdelay(10);
			continue;
		} else
			break;
	}

	if (i == INIT_RETRY_CNT)
		goto fail_init;

#if USE_CHECKSUM
	checksum_err = 0;

	for (i = 0; i < INIT_RETRY_CNT; i++) {
		if (read_data(client, BT432_CHECKSUM_RESULT,
			(u8 *)&chip_check_sum, 2) < 0) {
			mdelay(10);
			continue;
		}

#if defined(TSP_VERBOSE_DEBUG)
		dev_info(&client->dev, "0x%04X\n", chip_check_sum);
#endif
		if (chip_check_sum != 0x55aa)
			checksum_err = 1;
		break;
	}

	if (i == INIT_RETRY_CNT || checksum_err) {
		dev_err(&client->dev, "Failed to check firmware data\n");
		if (checksum_err == 1 && retry_cnt < INIT_RETRY_CNT)
			retry_cnt = INIT_RETRY_CNT;

		goto fail_init;
	}
#endif

	if (write_cmd(client, BT432_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	cap->button_num = SUPPORTED_BUTTON_NUM;

	reg_val = 0;
	zinitix_bit_set(reg_val, BIT_PT_CNT_CHANGE);
	zinitix_bit_set(reg_val, BIT_DOWN);
	zinitix_bit_set(reg_val, BIT_MOVE);
	zinitix_bit_set(reg_val, BIT_UP);
#if (TOUCH_POINT_MODE == 2)
	zinitix_bit_set(reg_val, BIT_PALM);
	zinitix_bit_set(reg_val, BIT_PALM_REJECT);
#endif

	if (cap->button_num > 0)
		zinitix_bit_set(reg_val, BIT_ICON_EVENT);

	cap->ic_int_mask = reg_val;

	if (write_reg(client, BT432_INT_ENABLE_FLAG, 0x0) != I2C_SUCCESS)
		goto fail_init;

	if (write_cmd(client, BT432_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_init;

	/* get chip information */
	if (read_data(client, BT432_VENDOR_ID, (u8 *)&cap->vendor_id, 2) < 0)
		goto fail_init;


	if (read_data(client, BT432_CHIP_REVISION,
					(u8 *)&cap->ic_revision, 2) < 0)
		goto fail_init;

	cap->ic_fw_size = 24 * 1024;
	if (read_data(client, BT432_HW_ID, (u8 *)&cap->hw_id, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_THRESHOLD, (u8 *)&cap->threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_BUTTON_SENSITIVITY,
					(u8 *)&cap->key_threshold, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_TOTAL_NUMBER_OF_X,
					(u8 *)&cap->x_node_num, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_TOTAL_NUMBER_OF_Y,
					(u8 *)&cap->y_node_num, 2) < 0)
		goto fail_init;

	cap->total_node_num = cap->x_node_num * cap->y_node_num;

	if (read_data(client, BT432_AFE_FREQUENCY,
					(u8 *)&cap->afe_frequency, 2) < 0)
		goto fail_init;
	if (read_data(client, BT432_DND_U_COUNT,
					(u8 *)&cap->U_cnt, 2) < 0)
		goto fail_init;
	if (read_data(client, BT432_DND_N_COUNT,
					(u8 *)&cap->N_cnt, 2) < 0)
		goto fail_init;

	/* get chip firmware version */
	if (read_data(client, BT432_FIRMWARE_VERSION,
					(u8 *)&cap->fw_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_MINOR_FW_VERSION,
					(u8 *)&cap->fw_minor_version, 2) < 0)
		goto fail_init;

	if (read_data(client, BT432_DATA_VERSION_REG,
					(u8 *)&cap->reg_data_version, 2) < 0)
		goto fail_init;

#if TOUCH_ONESHOT_UPGRADE
	if (!force) {
		if (ts_check_need_upgrade(info, cap->fw_version,
				cap->fw_minor_version, cap->reg_data_version,
				cap->hw_id) == true) {

			if (ts_upgrade_firmware(info, m_firmware_data,
				cap->ic_fw_size) == false)
				goto fail_init;

			if (ts_hw_calibration(info) == false)
				goto fail_init;

			/* disable chip interrupt */
			if (write_reg(client, BT432_INT_ENABLE_FLAG, 0)
							!= I2C_SUCCESS)
				goto fail_init;

			/* get chip firmware version */
			if (read_data(client, BT432_FIRMWARE_VERSION,
					(u8 *)&cap->fw_version, 2) < 0)
				goto fail_init;

			if (read_data(client, BT432_MINOR_FW_VERSION,
					(u8 *)&cap->fw_minor_version, 2) < 0)
				goto fail_init;

			if (read_data(client, BT432_DATA_VERSION_REG,
					(u8 *)&cap->reg_data_version, 2) < 0)
				goto fail_init;
		}
	}
#endif

	if (read_data(client, BT432_EEPROM_INFO_REG,
				(u8 *)&chip_eeprom_info, 2) < 0)
		goto fail_init;

	if (zinitix_bit_test(chip_eeprom_info, 0)) { /* hw calibration bit*/
		if (ts_hw_calibration(info) == false)
			goto fail_init;

		/* disable chip interrupt */
		if (write_reg(client, BT432_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			goto fail_init;
	}

	/* initialize */
	if (write_reg(client, BT432_X_RESOLUTION,
			(u16)pdata->x_resolution) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT432_Y_RESOLUTION,
			(u16)pdata->y_resolution) != I2C_SUCCESS)
		goto fail_init;

	cap->MinX = (u32)0;
	cap->MinY = (u32)0;
	cap->MaxX = (u32)pdata->x_resolution;
	cap->MaxY = (u32)pdata->y_resolution;

	if (write_reg(client, BT432_BUTTON_SUPPORTED_NUM,
			(u16)cap->button_num) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT432_SUPPORTED_FINGER_NUM,
			(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_init;

	cap->multi_fingers = MAX_SUPPORTED_FINGER_NUM;
	cap->gesture_support = 0;

	if (write_reg(client, BT432_INITIAL_TOUCH_MODE,
			TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT432_TOUCH_MODE,
			info->touch_mode) != I2C_SUCCESS)
		goto fail_init;

#if	ZINITIX_I2C_CHECKSUM
	if (read_data(client, ZINITIX_INTERNAL_FLAG_02,
			(u8 *)&reg_val, 2) < 0)
		goto fail_init;

	cap->i2s_checksum = !(!zinitix_bit_test(reg_val, 15));
	pr_debug("use i2s checksum = %d\r\n", cap->i2s_checksum);
#endif

	/* soft calibration */
	if (write_cmd(client, BT432_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_init;

	if (write_reg(client, BT432_INT_ENABLE_FLAG,
			cap->ic_int_mask) != I2C_SUCCESS)
		goto fail_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) { /* Test Mode */
		if (write_reg(client, BT432_DELAY_RAW_FOR_HOST,
				RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS) {
			dev_err(&client->dev, "%s: Failed to set DELAY_RAW_FOR_HOST\n",
				__func__);

			goto fail_init;
		}
	}
#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT432_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_init;
#endif

	return true;

fail_init:
	if (++retry_cnt <= INIT_RETRY_CNT) {
		bt432_power_control(info, POWER_OFF);
		bt432_power_control(info, POWER_ON_SEQUENCE);

		goto retry_init;

	} else if (retry_cnt == INIT_RETRY_CNT+1) {
		cap->ic_fw_size = 24*1024;

#if TOUCH_FORCE_UPGRADE
		if (ts_upgrade_firmware(info, m_firmware_data,
					cap->ic_fw_size) == false) {
			dev_err(&client->dev, "Failed to upgrade\n");

			return false;
		}
		mdelay(100);

		/* hw calibration and make checksum*/
		if (ts_hw_calibration(info) == false)
			return false;
#endif
	}

	dev_err(&client->dev, "Failed to initiallize\n");

	return false;
}

static bool mini_init_touch(struct bt432_ts_info *info)
{
	struct bt432_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
#if USE_CHECKSUM
	u16 chip_check_sum;

	if (read_data(client, BT432_CHECKSUM_RESULT,
					(u8 *)&chip_check_sum, 2) < 0)
		goto fail_mini_init;

	if (chip_check_sum != 0x55aa) {
		dev_err(&client->dev, "%s: Failed to check firmware data\n",
				__func__);

		goto fail_mini_init;
	}
#endif

	if (write_cmd(client, BT432_SWRESET_CMD) != I2C_SUCCESS)
		goto fail_mini_init;


	/* initialize */
	if (write_reg(client, BT432_X_RESOLUTION,
			(u16)(pdata->x_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_Y_RESOLUTION,
			(u16)(pdata->y_resolution)) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_BUTTON_SUPPORTED_NUM,
			(u16)info->cap_info.button_num) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_SUPPORTED_FINGER_NUM,
			(u16)MAX_SUPPORTED_FINGER_NUM) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_INITIAL_TOUCH_MODE,
			TOUCH_POINT_MODE) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_TOUCH_MODE, info->touch_mode)
			!= I2C_SUCCESS)
		goto fail_mini_init;

	/* soft calibration */
	if (write_cmd(client, BT432_CALIBRATE_CMD) != I2C_SUCCESS)
		goto fail_mini_init;

	if (write_reg(client, BT432_INT_ENABLE_FLAG,
			info->cap_info.ic_int_mask) != I2C_SUCCESS)
		goto fail_mini_init;

	/* read garbage data */
	for (i = 0; i < 10; i++) {
		write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	if (info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(client, BT432_DELAY_RAW_FOR_HOST,
				RAWDATA_DELAY_FOR_HOST) != I2C_SUCCESS)
			goto fail_mini_init;
	}

#if ESD_TIMER_INTERVAL
	if (write_reg(client, BT432_PERIODICAL_INTERRUPT_INTERVAL,
			SCAN_RATE_HZ * ESD_TIMER_INTERVAL) != I2C_SUCCESS)
		goto fail_mini_init;

	esd_timer_start(CHECK_ESD_TIMER, info);
#endif

	return true;

fail_mini_init:
	bt432_power_control(info, POWER_OFF);
	bt432_power_control(info, POWER_ON_SEQUENCE);

	if (init_touch(info, false) == false) {
		dev_err(&client->dev, "Failed to initialize\n");

		return false;
	}

#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, info);
#endif
	return true;
}

static void clear_report_data(struct bt432_ts_info *info)
{
	struct i2c_client *client = info->client;
	int i;
	u8 reported = 0;
	u8 sub_status;

	for (i = 0; i < info->cap_info.button_num; i++) {
		if (info->button[i] == ICON_BUTTON_DOWN) {
			info->button[i] = ICON_BUTTON_UP;
			input_report_key(info->input_dev,
					BUTTON_MAPPING_KEY[i], 0);
			reported = true;
			dev_info(&client->dev, "Button up = %d\n", i);
		}
	}

	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->reported_touch_info.coord[i].sub_status;
		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
							MT_TOOL_FINGER, 0);
			reported = true;
			dev_info(&client->dev, "Finger [%02d] up\n", i);
		}
		info->reported_touch_info.coord[i].sub_status = 0;
	}

	if (reported)
		input_sync(info->input_dev);

#if TOUCH_BOOSTER
	info->finger_cnt = 0;
	info->double_booster = false;
	pm_qos_update_request(&info->cpufreq_qos_req_min,
				PM_QOS_DEFAULT_VALUE);
#endif
}

#define	PALM_REPORT_WIDTH	200
#define	PALM_REJECT_WIDTH	255

static irqreturn_t bt432_touch_irq_handler(int irq, void *data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)data;
	struct bt432_ts_platform_data *pdata = info->pdata;
	struct i2c_client *client = info->client;
	int i;
	u8 sub_status;
	u8 prev_sub_status;
	u16 prev_icon_data;
	u32 x, y, maxX, maxY;
	u32 w;
	u32 tmp;
	u8 palm = 0;

	if (gpio_get_value(info->pdata->gpio_int)) {
		dev_err(&client->dev, "Invalid interrupt\n");
		return IRQ_HANDLED;
	}

	if (down_trylock(&info->work_lock)) {
		dev_err(&client->dev, "%s: Failed to occupy work lock\n",
			__func__);
		write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);

		return IRQ_HANDLED;
	}

	if (info->work_state != NOTHING) {
		dev_err(&client->dev, "%s: Other process occupied\n",
			__func__);
		udelay(DELAY_FOR_SIGNAL_DELAY);

		if (!gpio_get_value(info->pdata->gpio_int)) {
			write_cmd(client, BT432_CLEAR_INT_STATUS_CMD);
			udelay(DELAY_FOR_SIGNAL_DELAY);
		}
		goto out;
	}

	info->work_state = NORMAL;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif
#if	ZINITIX_I2C_CHECKSUM
	i = 0;
	if (ts_read_coord(info) == false || info->touch_info.status == 0xffff
		|| info->touch_info.status == 0x1) {
		/* more retry */
		for (i = 1; i < 50; i++) {	/* about 10ms */
			if (!(ts_read_coord(info) == false ||
				info->touch_info.status == 0xffff
				|| info->touch_info.status == 0x1))
				break;
		}
	}
	if (i == 50) {
		dev_err(&client->dev, "Failed to read info coord\n");
		bt432_power_control(info, POWER_OFF);
		bt432_power_control(info, POWER_ON_SEQUENCE);

		clear_report_data(info);
		mini_init_touch(info);

		goto out;
	}
#else
	if (ts_read_coord(info) == false || info->touch_info.status == 0xffff
		|| info->touch_info.status == 0x1) { /* maybe desirable reset */
		dev_err(&client->dev, "Failed to read info coord\n");
		bt432_power_control(info, POWER_OFF);
		bt432_power_control(info, POWER_ON_SEQUENCE);

		clear_report_data(info);
		mini_init_touch(info);

		goto out;
	}
#endif
	/* invalid : maybe periodical repeated int. */
	if (info->touch_info.status == 0x0)
		goto out;

	if (zinitix_bit_test(info->touch_info.status, BIT_ICON_EVENT)) {
		if (read_data(info->client, BT432_ICON_STATUS_REG,
			(u8 *)(&info->icon_event_reg), 2) < 0) {
			dev_err(&client->dev, "Failed to read button info\n");

			goto out;
		}

		prev_icon_data = info->icon_event_reg ^ info->prev_icon_event;

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg &
						prev_icon_data,
						(BIT_O_ICON0_DOWN + i))) {
				info->button[i] = ICON_BUTTON_DOWN;
				input_report_key(info->input_dev,
						BUTTON_MAPPING_KEY[i], 1);
				dev_info(&client->dev, "Button down = %d\n", i);
			}
		}

		for (i = 0; i < info->cap_info.button_num; i++) {
			if (zinitix_bit_test(info->icon_event_reg &
						prev_icon_data,
						(BIT_O_ICON0_UP + i))) {
				info->button[i] = ICON_BUTTON_UP;
				input_report_key(info->input_dev,
						BUTTON_MAPPING_KEY[i], 0);
				dev_info(&client->dev, "Button up = %d\n", i);
			}
		}

		info->prev_icon_event = info->icon_event_reg;
	}
	for (i = 0; i < info->cap_info.multi_fingers; i++) {
		sub_status = info->touch_info.coord[i].sub_status;
		prev_sub_status = info->reported_touch_info.coord[i].sub_status;

		if (zinitix_bit_test(sub_status, SUB_BIT_EXIST)) {
			x = info->touch_info.coord[i].x;
			y = info->touch_info.coord[i].y;
			w = info->touch_info.coord[i].width;

			 /* transformation from touch to screen orientation */
			if (pdata->orientation & TOUCH_V_FLIP)
				y = info->cap_info.MaxY
					+ info->cap_info.MinY - y;

			if (pdata->orientation & TOUCH_H_FLIP)
				x = info->cap_info.MaxX
					+ info->cap_info.MinX - x;

			maxX = info->cap_info.MaxX;
			maxY = info->cap_info.MaxY;

			if (pdata->orientation & TOUCH_XY_SWAP) {
				zinitix_swap_v(x, y, tmp);
				zinitix_swap_v(maxX, maxY, tmp);
			}

			if (x > maxX || y > maxY) {
				dev_err(&client->dev,
					"Invalid coord %d : x=%d, y=%d\n",
					 i, x, y);
				continue;
			}

			info->touch_info.coord[i].x = x;
			info->touch_info.coord[i].y = y;

			if (w == 0)
				w = 1;

			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
							MT_TOOL_FINGER, 1);

#if (TOUCH_POINT_MODE == 2)
			if (palm == 0)
				if (w >= PALM_REPORT_WIDTH)
					w = PALM_REPORT_WIDTH - 10;
			else if (palm == 1)	/* palm report*/
				w = PALM_REPORT_WIDTH;
			else if (palm == 2)	/* palm reject */
				w = PALM_REJECT_WIDTH;
#endif

			input_report_abs(info->input_dev,
					ABS_MT_TOUCH_MAJOR, (u32)w);
			input_report_abs(info->input_dev,
					ABS_MT_PRESSURE, (u32)w);
			input_report_abs(info->input_dev, ABS_MT_WIDTH_MAJOR,
					(u32)((palm == 1) ? w-40 : w));
#if (TOUCH_POINT_MODE == 2)
			input_report_abs(info->input_dev,
				ABS_MT_TOUCH_MINOR,
				(u32)info->touch_info.coord[i].minor_width);
			input_report_abs(info->input_dev, ABS_MT_ANGLE,
					(palm > 1) ? 70 :
					info->touch_info.coord[i].angle - 90);
			dev_info(&client->dev, "finger [%02d] angle = %03d\n",
					i, info->touch_info.coord[i].angle);
			input_report_abs(info->input_dev, ABS_MT_PALM,
					(palm > 2) ? 1 : 0);
#endif
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);

			if (zinitix_bit_test(sub_status, SUB_BIT_DOWN)) {
				dev_info(&client->dev,
					"Finger [%02d] down :x = %d, y = %d," \
					" w = %d\n", i, x, y, w);
#if TOUCH_BOOSTER
				info->finger_cnt++;

				if (info->finger_cnt == 1)
					pm_qos_update_request(
						&info->cpufreq_qos_req_min,
						1066);
				else if (info->double_booster == false) {
					pm_qos_update_request(
						&info->cpufreq_qos_req_min,
						1205);
					info->double_booster = true;
				}
#endif
			}
		} else if (zinitix_bit_test(sub_status, SUB_BIT_UP) ||
			zinitix_bit_test(prev_sub_status, SUB_BIT_EXIST)) {
			memset(&info->touch_info.coord[i], 0x0,
						sizeof(struct coord));
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
							MT_TOOL_FINGER, 0);
			dev_info(&client->dev, "Finger [%02d] up\n", i);
#if TOUCH_BOOSTER
			info->finger_cnt--;

			if (info->finger_cnt == 1) {
				info->double_booster = false;
				pm_qos_update_request(
					&info->cpufreq_qos_req_min, 1066);
			}

			if (!info->finger_cnt)
				pm_qos_update_request(
					&info->cpufreq_qos_req_min,
					PM_QOS_DEFAULT_VALUE);
#endif
		} else
			memset(&info->touch_info.coord[i], 0x0,
				sizeof(struct coord));
	}
	memcpy((char *)&info->reported_touch_info, (char *)&info->touch_info,
			sizeof(struct point_info));
	input_sync(info->input_dev);

out:
	if (info->work_state == NORMAL) {
#if ESD_TIMER_INTERVAL
		esd_timer_start(CHECK_ESD_TIMER, info);
#endif
		info->work_state = NOTHING;
	}

	up(&info->work_lock);

	return IRQ_HANDLED;
}

#if defined(CONFIG_PM)
static int bt432_ts_resume(struct device *dev)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	dev_info(&client->dev, "%s\n", __func__);
	down(&info->work_lock);
	if (info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);

		return 0;
	}

	bt432_power_control(info, POWER_ON_SEQUENCE);

	if (mini_init_touch(info) == false)
		dev_err(&client->dev, "Failed to resume\n");
	if (!gpio_get_value(info->pdata->gpio_int)) {
		write_cmd(info->client, BT432_CLEAR_INT_STATUS_CMD);
		udelay(50);
		write_cmd(info->client, BT432_CLEAR_INT_STATUS_CMD);
		udelay(50);
		write_cmd(info->client, BT432_CLEAR_INT_STATUS_CMD);
	}
	info->work_state = NOTHING;

#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "resume\n");
#endif
	up(&info->work_lock);
	enable_irq(info->irq);
	return 0;
}

static int bt432_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bt432_ts_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s\n", __func__);
	disable_irq(info->irq);
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
#endif

	down(&info->work_lock);
	if (info->work_state != NOTHING
		&& info->work_state != SUSPEND) {
		dev_err(&client->dev, "%s: Invalid work proceedure (%d)\n",
				__func__, info->work_state);
		up(&info->work_lock);
		enable_irq(info->irq);
		return 0;
	}

	clear_report_data(info);

#if ESD_TIMER_INTERVAL
	esd_timer_stop(info);
#endif
	bt432_power_control(info, POWER_OFF);
	info->work_state = SUSPEND;

#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&client->dev, "suspend\n");
#endif
	up(&info->work_lock);

	return 0;
}
#endif

static bool ts_set_touchmode(u16 value)
{
	int i;

	disable_irq(misc_info->irq);

	down(&misc_info->work_lock);
	if (misc_info->work_state != NOTHING) {
		dev_err(&misc_info->client->dev, "%s: Other process occupied" \
			" (%d)\n", __func__, misc_info->work_state);
		enable_irq(misc_info->irq);
		up(&misc_info->work_lock);

		return -1;
	}

	misc_info->work_state = SET_MODE;

	if (value == TOUCH_DND_MODE) {
		if (write_reg(misc_info->client, BT432_DND_N_COUNT,
					SEC_DND_N_COUNT) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set DND_N_COUNT\n");

		if (write_reg(misc_info->client, BT432_AFE_FREQUENCY,
					SEC_DND_FREQUENCY) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set AFE_FREQUENCY\n");

	} else if (value == TOUCH_PDND_MODE) {
		if (write_reg(misc_info->client, BT432_DND_N_COUNT,
					SEC_PDND_N_COUNT) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set PDND_N_COUNT\n");

		if (write_reg(misc_info->client, BT432_DND_U_COUNT,
					SEC_PDND_U_COUNT) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set PDND_N_COUNT\n");

		if (write_reg(misc_info->client, BT432_AFE_FREQUENCY,
					SEC_DND_FREQUENCY) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set AFE_FREQUENCY\n");

	} else if ((misc_info->touch_mode == TOUCH_DND_MODE) ||
				(misc_info->touch_mode == TOUCH_PDND_MODE)) {
		if (write_reg(misc_info->client, BT432_AFE_FREQUENCY,
				misc_info->cap_info.afe_frequency)
				!= I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set AFE_FREQUENCY\n");
		if (write_reg(misc_info->client, BT432_DND_U_COUNT,
				misc_info->cap_info.U_cnt) != I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set U_COUNT\n");
		if (write_reg(misc_info->client, BT432_DND_N_COUNT,
				misc_info->cap_info.N_cnt)
				!= I2C_SUCCESS)
			dev_err(&misc_info->client->dev,
				"Failed to set N_COUNT\n");
	}

	if (value == TOUCH_SEC_MODE)
		misc_info->touch_mode = TOUCH_POINT_MODE;
	else
		misc_info->touch_mode = value;

#if defined(TSP_VERBOSE_DEBUG)
	dev_info(&misc_info->client->dev,
		"tsp_set_testmode, touchkey_testmode = %d\n",
		misc_info->touch_mode);
#endif

	if (misc_info->touch_mode != TOUCH_POINT_MODE) {
		if (write_reg(misc_info->client, BT432_DELAY_RAW_FOR_HOST,
						RAWDATA_DELAY_FOR_HOST)
						!= I2C_SUCCESS)
			dev_err(&misc_info->client->dev, "%s: Failed to set" \
				" DELAY_RAW_FOR_HOST\n", __func__);
	}

	if (write_reg(misc_info->client, BT432_TOUCH_MODE,
			misc_info->touch_mode) != I2C_SUCCESS)
		dev_err(&misc_info->client->dev, "Failed to set TOUCH_MODE\n");

	/* clear garbage data */
	for (i = 0; i < 10; i++) {
		mdelay(20);
		write_cmd(misc_info->client, BT432_CLEAR_INT_STATUS_CMD);
	}

	misc_info->work_state = NOTHING;
	enable_irq(misc_info->irq);
	up(&misc_info->work_lock);

	return 1;
}

static int ts_upgrade_sequence(const u8 *firmware_data)
{
	disable_irq(misc_info->irq);
	down(&misc_info->work_lock);
	misc_info->work_state = UPGRADE;

#if ESD_TIMER_INTERVAL
	esd_timer_stop(misc_info);
#endif
	clear_report_data(misc_info);

	if (ts_upgrade_firmware(misc_info, firmware_data,
				misc_info->cap_info.ic_fw_size) == false)
		goto out;


	if (init_touch(misc_info, true) == false)
		goto out;


#if ESD_TIMER_INTERVAL
	esd_timer_start(CHECK_ESD_TIMER, misc_info);
#endif

	enable_irq(misc_info->irq);
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return 0;

out:
	enable_irq(misc_info->irq);
	misc_info->work_state = NOTHING;
	up(&misc_info->work_lock);
	return -1;
}

#ifdef SEC_FACTORY_TEST
static inline void set_cmd_result(struct bt432_ts_info *info,
						char *buff, int len)
{
	strncat(info->factory_info->cmd_result, buff, len);
}

static inline void set_default_result(struct bt432_ts_info *info)
{
	char delim = ':';
	memset(info->factory_info->cmd_result, 0x00,
		ARRAY_SIZE(info->factory_info->cmd_result));
	memcpy(info->factory_info->cmd_result, info->factory_info->cmd,
		strlen(info->factory_info->cmd));
	strncat(info->factory_info->cmd_result, &delim, 1);
}

#define MAX_FW_PATH 255
#define TSP_FW_FILENAME "zinitix_fw.bin"

static void fw_update(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	int ret = 0;
	const u8 *buff = 0;
	mm_segment_t old_fs = {0};
	struct file *fp = NULL;
	long fsize = 0, nread = 0;
	char fw_path[MAX_FW_PATH+1];

	set_default_result(info);

	switch (info->factory_info->cmd_param[0]) {
	case BUILT_IN:
		ret = ts_upgrade_sequence((u8 *)m_firmware_data);
		if (unlikely(ret < 0)) {
			info->factory_info->cmd_state = FAIL;
			return;
		}
		break;

	case UMS:
		old_fs = get_fs();
		set_fs(get_ds());

		snprintf(fw_path, MAX_FW_PATH, "/sdcard/%s", TSP_FW_FILENAME);
		fp = filp_open(fw_path, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			dev_err(&client->dev, "Failed to open %s (%d)\n",
				fw_path, (s32)fp);
			info->factory_info->cmd_state = FAIL;

			goto err_open;
		}

		fsize = fp->f_path.dentry->d_inode->i_size;

		if (fsize != info->cap_info.ic_fw_size) {
			dev_err(&client->dev, "Invalid fw size!!\n");
			info->factory_info->cmd_state = FAIL;

			goto err_open;
		}

		buff = kzalloc((size_t)fsize, GFP_KERNEL);
		if (unlikely(!buff)) {
			dev_err(&client->dev, "%s: Failed to allocate " \
						"memory\n", __func__);
			info->factory_info->cmd_state = FAIL;

			goto err_alloc;
		}

		nread = vfs_read(fp, (char __user *)buff, fsize, &fp->f_pos);
		if (nread != fsize) {
			info->factory_info->cmd_state = FAIL;

			goto err_fw_size;
		}

		filp_close(fp, current->files);
		set_fs(old_fs);

		ret = ts_upgrade_sequence((u8 *)buff);
		if (unlikely(ret < 0)) {
			kfree(buff);
			info->factory_info->cmd_state = FAIL;

			return;
		}
		break;

	default:
		dev_err(&client->dev, "Invalid fw file type!!\n");

		goto not_support;
	}

	info->factory_info->cmd_state = OK;
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff) , "%s", "OK");
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

if (fp != NULL) {
err_fw_size:
	kfree(buff);
err_alloc:
	filp_close(fp, NULL);
err_open:
	set_fs(old_fs);
}
	return;

not_support:
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff) , "%s", "NG");
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_fw_ver_bin(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id, vendor_id;
	u32 version, length;

	set_default_result(info);

	/* To Do */
	/* modify m_firmware_data */
	fw_version = (u16)(m_firmware_data[52] | (m_firmware_data[53] << 8));
	fw_minor_version = (u16)(m_firmware_data[56] |
				(m_firmware_data[57] << 8));
	reg_version = (u16)(m_firmware_data[60] | (m_firmware_data[61] << 8));
	if (info->cap_info.chip_code == 0xf400) {
		hw_id = (u16) (m_firmware_data[0x6b12] |
				(m_firmware_data[0x6b13]<<8));
		vendor_id = ntohs(*(u16 *)&m_firmware_data[0x6b22]);
	} else {
		hw_id = (u16)(m_firmware_data[0x57d2] |
				(m_firmware_data[0x57d3]<<8));
		vendor_id = ntohs(*(u16 *)&m_firmware_data[0x57e2]);
	}
	version = (u32)((u32)(hw_id & 0xff) << 16) |
				((fw_version & 0xf) << 12)
				| ((fw_minor_version & 0xf) << 8) |
				(reg_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length,
				"%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
				strnlen(finfo->cmd_buff,
				sizeof(finfo->cmd_buff)));

	return;
}

static void get_fw_ver_ic(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	u16 fw_version, fw_minor_version, reg_version, hw_id, vendor_id;
	u32 version, length;

	set_default_result(info);

	fw_version = info->cap_info.fw_version;
	fw_minor_version = info->cap_info.fw_minor_version;
	reg_version = info->cap_info.reg_data_version;
	hw_id = info->cap_info.hw_id;
	vendor_id = ntohs(info->cap_info.vendor_id);
	version = (u32)((u32)(hw_id & 0xff) << 16) | ((fw_version & 0xf) << 12)
				| ((fw_minor_version & 0xf) << 8) |
				(reg_version & 0xff);

	length = sizeof(vendor_id);
	snprintf(finfo->cmd_buff, length + 1, "%s", (u8 *)&vendor_id);
	snprintf(finfo->cmd_buff + length, sizeof(finfo->cmd_buff) - length,
				"%06X", version);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_threshold(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%d", info->cap_info.threshold);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void module_off_master(void *device_data)
{
	return;
}

static void module_on_master(void *device_data)
{
	return;
}

static void module_off_slave(void *device_data)
{
	return;
}

static void module_on_slave(void *device_data)
{
	return;
}

#define BT432_VENDOR_NAME "ZINITIX"

static void get_chip_vendor(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%s", BT432_VENDOR_NAME);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

#define BT432_CHIP_NAME "BT432"

static void get_chip_name(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
						BT432_CHIP_NAME);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_x_num(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
		"%u", info->cap_info.x_node_num);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void get_y_num(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
				"%u", info->cap_info.y_node_num);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void not_support_cmd(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	set_default_result(info);

	sprintf(finfo->cmd_buff, "%s", "NA");
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	info->factory_info->cmd_state = NOT_APPLICABLE;

	dev_info(&client->dev, "%s: \"%s(%d)\"\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_reference_read(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	u16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DND_MODE);
	get_raw_data(info, (u8 *)raw_data->ref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++)	{
			if (raw_data->ref_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->ref_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->ref_data[i * info->cap_info.y_node_num + j];

			if (raw_data->ref_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->ref_data[i * info->cap_info.y_node_num + j];
		}
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
		strlen(finfo->cmd_buff));

	return;
}

static void get_reference(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
			"%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
				strnlen(finfo->cmd_buff,
				sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->ref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_preference_read(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	u16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_PDND_MODE);
	get_raw_data(info, (u8 *)raw_data->pref_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);

	min = 0xFFFF;
	max = 0x0000;

	for (i = 0; i < info->cap_info.x_node_num; i++) {
		for (j = 0; j < info->cap_info.y_node_num; j++)	{
			if (raw_data->pref_data[i * info->cap_info.y_node_num + j] < min &&
				raw_data->pref_data[i * info->cap_info.y_node_num + j] != 0)
				min = raw_data->pref_data[i * info->cap_info.y_node_num + j];

			if (raw_data->pref_data[i * info->cap_info.y_node_num + j] > max)
				max = raw_data->pref_data[i * info->cap_info.y_node_num + j];
		}
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n",
		min, max);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
		strlen(finfo->cmd_buff));

	return;
}

static void get_preference(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node >= info->cap_info.x_node_num ||
		y_node < 0 || y_node >= info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
			"%s", "abnormal");
		set_cmd_result(info, finfo->cmd_buff,
				strnlen(finfo->cmd_buff,
				sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.y_node_num + y_node;

	val = raw_data->pref_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static void run_delta_read(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	s16 min, max;
	s32 i, j;

	set_default_result(info);

	ts_set_touchmode(TOUCH_DELTA_MODE);
	get_raw_data(info, (u8 *)(u8 *)raw_data->delta_data, 10);
	ts_set_touchmode(TOUCH_POINT_MODE);
	finfo->cmd_state = OK;

	min = (s16)0x7FFF;
	max = (s16)0x8000;

	for (i = 0; i < 30; i++) {
		for (j = 0; j < 18; j++) {
			if (raw_data->delta_data[j+i] < min)
				min = raw_data->delta_data[j+i];

			if (raw_data->delta_data[j+i] > max)
				max = raw_data->delta_data[j+i];
		}
	}

	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%d,%d\n", min, max);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	finfo->cmd_state = OK;
	dev_info(&client->dev, "%s: \"%s\"(%d)\n", __func__, finfo->cmd_buff,
		strlen(finfo->cmd_buff));

	return;
}

static void get_delta(void *device_data)
{
	struct bt432_ts_info *info = (struct bt432_ts_info *)device_data;
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	struct tsp_raw_data *raw_data = info->raw_data;
	unsigned int val;
	int x_node, y_node;
	int node_num;

	set_default_result(info);

	x_node = finfo->cmd_param[0];
	y_node = finfo->cmd_param[1];

	if (x_node < 0 || x_node > info->cap_info.x_node_num ||
		y_node < 0 || y_node > info->cap_info.y_node_num) {
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%s",
			"abnormal");
		set_cmd_result(info, finfo->cmd_buff,
				strnlen(finfo->cmd_buff,
				sizeof(finfo->cmd_buff)));
		info->factory_info->cmd_state = FAIL;

		return;
	}

	node_num = x_node * info->cap_info.x_node_num + y_node;

	val = raw_data->delta_data[node_num];
	snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "%u", val);
	set_cmd_result(info, finfo->cmd_buff,
			strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));
	info->factory_info->cmd_state = OK;

	dev_info(&client->dev, "%s: %s(%d)\n", __func__, finfo->cmd_buff,
		strnlen(finfo->cmd_buff, sizeof(finfo->cmd_buff)));

	return;
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;
	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;

	if (finfo->cmd_is_running == true) {
		dev_err(&client->dev, "%s: other cmd is running\n", __func__);
		goto err_out;
	}

	/* check lock  */
	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = true;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = RUNNING;

	for (i = 0; i < ARRAY_SIZE(finfo->cmd_param); i++)
		finfo->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;

	memset(finfo->cmd, 0x00, ARRAY_SIZE(finfo->cmd));
	memcpy(finfo->cmd, buf, len);

	cur = strchr(buf, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
		if (!strcmp(buff, tsp_cmd_ptr->cmd_name)) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &finfo->cmd_list_head, list) {
			if (!strcmp("not_support_cmd", tsp_cmd_ptr->cmd_name))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strlen(buff)) = '\0';
				finfo->cmd_param[param_cnt] =
					(int)simple_strtol(buff, NULL, 10);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);

	tsp_cmd_ptr->cmd_func(info);

err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: status:%d\n", finfo->cmd_state);

	if (finfo->cmd_state == WAITING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "WAITING");

	else if (finfo->cmd_state == RUNNING)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "RUNNING");

	else if (finfo->cmd_state == OK)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "OK");

	else if (finfo->cmd_state == FAIL)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff), "FAIL");

	else if (finfo->cmd_state == NOT_APPLICABLE)
		snprintf(finfo->cmd_buff, sizeof(finfo->cmd_buff),
			"NOT_APPLICABLE");

	return snprintf(buf, sizeof(finfo->cmd_buff),
			"%s\n", finfo->cmd_buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct tsp_factory_info *finfo = info->factory_info;

	dev_info(&client->dev, "tsp cmd: result: %s\n", finfo->cmd_result);

	mutex_lock(&finfo->cmd_lock);
	finfo->cmd_is_running = false;
	mutex_unlock(&finfo->cmd_lock);

	finfo->cmd_state = WAITING;

	return snprintf(buf, sizeof(finfo->cmd_result),
					"%s\n", finfo->cmd_result);
}

static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_cmd.attr,
	&dev_attr_cmd_status.attr,
	&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes,
};

#ifdef SUPPORTED_TOUCH_KEY
static ssize_t show_touchkey_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	struct capa_info *cap = &(info->cap_info);
	dev_info(&client->dev, "%s: key threshold = %d\n", __func__,
		cap->key_threshold);

	return snprintf(buf, sizeof(buf), "%d", cap->key_threshold);
}

static ssize_t enable_dummy_key(struct device *dev,
				struct device_attribute *attr,
				char *buf, size_t count)
{
	static char enable = '0';
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	if (!strcmp(buf, ""))
		count = sprintf(buf, "%c", enable);
	else {
		if ((buf[0] - '0' <= 1) && count == 2)
			enable = *buf;
		else {
			dev_err(&client->dev, "%s: Invalid parameter\n",
				__func__);
			goto err_out;
		}
	}

	dev_info(&client->dev, "%s: Extra button event %c\n",
		__func__, enable);

	return count;

err_out:
	return sprintf(buf, "NG");
}

static ssize_t show_touchkey_sensitivity(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct bt432_ts_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;
	u16 val;
	int ret;
	int i;

	if (!strcmp(attr->attr.name, "touchkey_menu"))
		i = 0;
	else if (!strcmp(attr->attr.name, "touchkey_back"))
		i = 1;
	else {
		dev_err(&client->dev, "%s: Invalid attribut\n", __func__);
		goto err_out;
	}

	ret = read_data(client, BT432_BTN_WIDTH + i, (u8 *)&val, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Failed to read %d's key " \
			"sensitivity\n", __func__, i);
		goto err_out;
	}

	dev_info(&client->dev, "%s: %d's key sensitivity = %d\n",
		__func__, i, val);

	return snprintf(buf, 6, "%d", val);

err_out:
	return sprintf(buf, "NG");
}

static ssize_t show_back_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t show_menu_key_raw_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}
#if SUPPORTED_TOUCH_KEY_LED
static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	unsigned char data;
	if (sscanf(buf, "%c\n", &data) == 1) {
		if (data == 0x31)
			gpio_direction_output(misc_info->pdata->gpio_keyled, 1);
		else if (data == 0x30)
			gpio_direction_output(misc_info->pdata->gpio_keyled, 0);
	}
	return size;
}
#endif
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, show_touchkey_threshold, NULL);
static DEVICE_ATTR(extra_button_event, S_IWUSR | S_IWGRP | S_IRUGO,
					enable_dummy_key, enable_dummy_key);
static DEVICE_ATTR(touchkey_menu, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, show_touchkey_sensitivity, NULL);
static DEVICE_ATTR(touchkey_raw_back, S_IRUGO, show_back_key_raw_data, NULL);
static DEVICE_ATTR(touchkey_raw_menu, S_IRUGO, show_menu_key_raw_data, NULL);
#if SUPPORTED_TOUCH_KEY_LED
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL,  touch_led_control);
#endif

static struct attribute *touchkey_attributes[] = {
	&dev_attr_touchkey_threshold.attr,
	&dev_attr_touchkey_back.attr,
	&dev_attr_touchkey_menu.attr,
	&dev_attr_extra_button_event.attr,
	&dev_attr_touchkey_raw_menu.attr,
	&dev_attr_touchkey_raw_back.attr,
#if SUPPORTED_TOUCH_KEY_LED
	&dev_attr_brightness.attr,
#endif
	NULL,
};
static struct attribute_group touchkey_attr_group = {
	.attrs = touchkey_attributes,
};
#endif

static int init_sec_factory(struct bt432_ts_info *info)
{
	struct device *factory_ts_dev;
#ifdef SUPPORTED_TOUCH_KEY
	struct device *factory_tk_dev;
#endif
	struct tsp_factory_info *factory_info;
	struct tsp_raw_data *raw_data;
	int ret;
	int i;

	factory_info = kzalloc(sizeof(struct tsp_factory_info), GFP_KERNEL);
	if (unlikely(!factory_info)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
			__func__);
		ret = -ENOMEM;

		goto err_alloc1;
	}
	raw_data = kzalloc(sizeof(struct tsp_raw_data), GFP_KERNEL);
	if (unlikely(!raw_data)) {
		dev_err(&info->client->dev, "%s: Failed to allocate memory\n",
			__func__);
		ret = -ENOMEM;

		goto err_alloc2;
	}

	INIT_LIST_HEAD(&factory_info->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &factory_info->cmd_list_head);

	factory_ts_dev = device_create(sec_class, NULL, 0, info, "tsp");
	if (unlikely(!factory_ts_dev)) {
		dev_err(&info->client->dev, "Failed to create factory" \
			" dev\n");
		ret = -ENODEV;

		goto err_create_device;
	}

#ifdef SUPPORTED_TOUCH_KEY
	factory_tk_dev = device_create(sec_class, NULL, 0, info,
					"sec_touchkey");
	if (IS_ERR(factory_tk_dev)) {
		dev_err(&info->client->dev, "Failed to create factory" \
			" dev\n");
		ret = -ENODEV;

		goto err_create_device;
	}
#endif

	ret = sysfs_create_group(&factory_ts_dev->kobj,
				&touchscreen_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev, "Failed to create" \
					" touchscreen sysfs group\n");

		goto err_create_sysfs;
	}

#ifdef SUPPORTED_TOUCH_KEY
	ret = sysfs_create_group(&factory_tk_dev->kobj, &touchkey_attr_group);
	if (unlikely(ret)) {
		dev_err(&info->client->dev,
			"Failed to create touchkey sysfs group\n");
		goto err_create_sysfs;
	}
#endif

	mutex_init(&factory_info->cmd_lock);
	factory_info->cmd_is_running = false;

	info->factory_info = factory_info;
	info->raw_data = raw_data;

	return ret;

err_create_sysfs:
err_create_device:
	kfree(raw_data);
err_alloc2:
	kfree(factory_info);
err_alloc1:

	return ret;
}
#endif

static int ts_misc_fops_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int ts_misc_fops_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct raw_ioctl raw_ioctl;
	u8 *u8Data;
	int ret = 0;
	size_t sz = 0;
	u16 version;
	u16 mode;

	struct reg_ioctl reg_ioctl;
	u16 val;
	int nval = 0;

	if (misc_info == NULL)
		return -1;

	switch (cmd) {

	case TOUCH_IOCTL_GET_DEBUGMSG_STATE:
		ret = m_ts_debug_mode;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_SET_DEBUGMSG_STATE:
		if (copy_from_user(&nval, argp, 4)) {
			pr_err("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}
		if (nval)
			pr_debug("[zinitix_touch] on debug mode (%d)\n",
				nval);
		else
			pr_debug("[zinitix_touch] off debug mode (%d)\n",
				nval);
		m_ts_debug_mode = nval;
		break;

	case TOUCH_IOCTL_GET_CHIP_REVISION:
		ret = misc_info->cap_info.ic_revision;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_FW_VERSION:
		ret = misc_info->cap_info.fw_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_REG_DATA_VERSION:
		ret = misc_info->cap_info.reg_data_version;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_SIZE:
		if (copy_from_user(&sz, argp, sizeof(size_t)))
			return -1;

		printk(KERN_INFO "[zinitix_touch]: firmware size = %d\r\n", sz);
		if (misc_info->cap_info.ic_fw_size != sz) {
			pr_err("[zinitix_touch]: firmware size error\r\n");
			return -1;
		}
		break;

	case TOUCH_IOCTL_VARIFY_UPGRADE_DATA:
		if (copy_from_user(m_firmware_data,
			argp, misc_info->cap_info.ic_fw_size))
			return -1;

		version = (u16) (m_firmware_data[52] |
				(m_firmware_data[53]<<8));

		pr_debug("[zinitix_touch]: firmware version = %x\r\n", version);

		if (copy_to_user(argp, &version, sizeof(version)))
			return -1;
		break;

	case TOUCH_IOCTL_START_UPGRADE:
		return ts_upgrade_sequence((u8 *)m_firmware_data);

	case TOUCH_IOCTL_GET_X_RESOLUTION:
		ret = misc_info->pdata->x_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_RESOLUTION:
		ret = misc_info->pdata->y_resolution;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_X_NODE_NUM:
		ret = misc_info->cap_info.x_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_Y_NODE_NUM:
		ret = misc_info->cap_info.y_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_GET_TOTAL_NODE_NUM:
		ret = misc_info->cap_info.total_node_num;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -1;
		break;

	case TOUCH_IOCTL_HW_CALIBRAION:
		ret = -1;
		disable_irq(misc_info->irq);
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_err("[zinitix_touch]: other process occupied.. (%d)\r\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = HW_CALIBRAION;
		mdelay(100);

		/* h/w calibration */
		if (ts_hw_calibration(misc_info) == true)
			ret = 0;

		mode = misc_info->touch_mode;
		if (write_reg(misc_info->client,
			BT432_TOUCH_MODE, mode) != I2C_SUCCESS) {
			pr_err("[zinitix_touch]: failed to set touch mode %d.\n",
				mode);
			goto fail_hw_cal;
		}

		if (write_cmd(misc_info->client,
			BT432_SWRESET_CMD) != I2C_SUCCESS)
			goto fail_hw_cal;

		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;
fail_hw_cal:
		enable_irq(misc_info->irq);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return -1;

	case TOUCH_IOCTL_SET_RAW_DATA_MODE:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		if (copy_from_user(&nval, argp, 4)) {
			pr_err("[zinitix_touch] error : copy_from_user\r\n");
			misc_info->work_state = NOTHING;
			return -1;
		}
		ts_set_touchmode((u16)nval);

		return 0;

	case TOUCH_IOCTL_GET_REG:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_err("[zinitix_touch]:other process occupied.. (%d)\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;

		if (copy_from_user(&reg_ioctl,
			argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_err("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (read_data(misc_info->client,
			reg_ioctl.addr, (u8 *)&val, 2) < 0)
			ret = -1;

		nval = (int)val;

		if (copy_to_user(reg_ioctl.val, (u8 *)&nval, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_err("[zinitix_touch] error : copy_to_user\n");
			return -1;
		}

		zinitix_debug_msg("read : reg addr = 0x%x, val = 0x%x\n",
			reg_ioctl.addr, nval);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SET_REG:

		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_err("[zinitix_touch]: other process occupied.. (%d)\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (copy_from_user(&reg_ioctl,
				argp, sizeof(struct reg_ioctl))) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_err("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (copy_from_user(&val, reg_ioctl.val, 4)) {
			misc_info->work_state = NOTHING;
			up(&misc_info->work_lock);
			pr_err("[zinitix_touch] error : copy_from_user\n");
			return -1;
		}

		if (write_reg(misc_info->client,
			reg_ioctl.addr, val) != I2C_SUCCESS)
			ret = -1;

		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x%x\r\n",
			reg_ioctl.addr, val);
		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_DONOT_TOUCH_EVENT:

		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_err("[zinitix_touch]: other process occupied.. (%d)\r\n",
				misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}

		misc_info->work_state = SET_MODE;
		if (write_reg(misc_info->client,
			BT432_INT_ENABLE_FLAG, 0) != I2C_SUCCESS)
			ret = -1;
		zinitix_debug_msg("write : reg addr = 0x%x, val = 0x0\r\n",
			BT432_INT_ENABLE_FLAG);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_SEND_SAVE_STATUS:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}
		down(&misc_info->work_lock);
		if (misc_info->work_state != NOTHING) {
			pr_err("[zinitix_touch]: other process occupied.." \
				"(%d)\r\n", misc_info->work_state);
			up(&misc_info->work_lock);
			return -1;
		}
		misc_info->work_state = SET_MODE;
		ret = 0;
		write_reg(misc_info->client, 0xc003, 0x0001);
		write_reg(misc_info->client, 0xc104, 0x0001);
		if (write_cmd(misc_info->client,
			BT432_SAVE_STATUS_CMD) != I2C_SUCCESS)
			ret =  -1;

		mdelay(1000);	/* for fusing eeprom */
		write_reg(misc_info->client, 0xc003, 0x0000);
		write_reg(misc_info->client, 0xc104, 0x0000);

		misc_info->work_state = NOTHING;
		up(&misc_info->work_lock);
		return ret;

	case TOUCH_IOCTL_GET_RAW_DATA:
		if (misc_info == NULL) {
			zinitix_debug_msg("misc device NULL?\n");
			return -1;
		}

		if (misc_info->touch_mode == TOUCH_POINT_MODE)
			return -1;

		down(&misc_info->raw_data_lock);
		if (misc_info->update == 0) {
			up(&misc_info->raw_data_lock);
			return -2;
		}

		if (copy_from_user(&raw_ioctl,
			argp, sizeof(raw_ioctl))) {
			up(&misc_info->raw_data_lock);
			pr_err("[zinitix_touch] error : copy_from_user\r\n");
			return -1;
		}

		misc_info->update = 0;

		u8Data = (u8 *)&misc_info->cur_data[0];

		if (copy_to_user(raw_ioctl.buf, (u8 *)u8Data,
			raw_ioctl.sz)) {
			up(&misc_info->raw_data_lock);
			return -1;
		}

		up(&misc_info->raw_data_lock);
		return 0;

	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tsp_dt_ids[] = {
	{ .compatible = "zinitix,bt432_tsp", },
	{},
};
MODULE_DEVICE_TABLE(of, tsp_dt_ids);
#endif

static int bt432_ts_probe_dt(struct device_node *np,
			 struct device *dev,
			 struct bt432_ts_platform_data *pdata)
{
	int tsp_intr_n = -1, keyled_n = -1;
	int ret;
	const struct of_device_id *match;

	if (!np)
		return -EINVAL;
	match = of_match_device(tsp_dt_ids, dev);
	if (!match)
		return -EINVAL;

	ret = of_property_read_u16(np, "bt432,x_resolution",
						&pdata->x_resolution);
	if (ret < 0)
		return ret;
	ret = of_property_read_u16(np, "bt432,y_resolution",
						&pdata->y_resolution);
	if (ret < 0)
		return ret;
	ret = of_property_read_u8(np, "bt432,orientation",
						&pdata->orientation);
	if (ret < 0)
		return ret;
	ret = of_property_read_u16(np, "bt432,page_size", &pdata->page_size);
	if (ret < 0)
		return ret;

	keyled_n = of_get_named_gpio(np, "keyled_gpio", 0);
	if (keyled_n < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
			keyled_n);
		return -EINVAL;
	}
	tsp_intr_n = of_get_named_gpio(np, "tsp_gpio", 0);
	if (tsp_intr_n < 0) {
		pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
			tsp_intr_n);
		return -EINVAL;
	}
	pdata->gpio_keyled = keyled_n;
	pdata->gpio_int = tsp_intr_n;

	ret = of_property_read_u32(np, "tsp_vdd_supply_type",
				&pdata->tsp_supply_type);
	if (ret < 0) {
		pr_err("%s: failed to read property tsp_vdd_supply_type\n",
			__func__);
		return ret;
	}

	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		pdata->tsp_en_gpio = of_get_named_gpio(np, "tsp_en_gpio", 0);
		if (pdata->tsp_en_gpio < 0) {
			pr_err("%s: of_get_named_gpio failed: %d\n", __func__,
				pdata->tsp_en_gpio);
			return -EINVAL;
		}
	}

	return 0;
}

static int bt432_ts_probe(struct i2c_client *client,
				const struct i2c_device_id *i2c_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct bt432_ts_platform_data *pdata = client->dev.platform_data;
	struct bt432_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	int i;

	struct device_node *np = client->dev.of_node;

	if (IS_ENABLED(CONFIG_OF)) {
		if (!pdata) {
			pdata = devm_kzalloc(&client->dev,
					sizeof(*pdata), GFP_KERNEL);
			if (!pdata)
				return -ENOMEM;
		}
		ret = bt432_ts_probe_dt(np, &client->dev, pdata);
		if (ret)
			goto err_no_platform_data;

	} else if (!pdata) {
		dev_err(&client->dev, "%s: no platform data defined\n",
			__func__);
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		ret = -EIO;
		goto err_no_platform_data;
	}

	ret = gpio_request(pdata->gpio_keyled, "key_led_gpio");
	if (ret < 0) {
		pr_err("%s: gpio_request failed: %d\n", __func__, ret);
			ret = -EINVAL;
			goto err_no_platform_data;
	}

	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		ret = gpio_request(pdata->tsp_en_gpio, "tsp_en_gpio");
		if (ret < 0) {
			pr_err("%s: gpio_request failed: %d\n", __func__, ret);
			goto err_gpio_request;
		}
	}

	info = kzalloc(sizeof(struct bt432_ts_info), GFP_KERNEL);
	if (unlikely(!info)) {
		dev_err(&client->dev, "%s: Failed to allocate memory\n",
			__func__);
		ret = -ENOMEM;
		goto err_mem_alloc;
	}

	i2c_set_clientdata(client, info);
	info->client = client;
	info->pdata = pdata;

	input_dev = input_allocate_device();
	if (unlikely(!input_dev)) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	info->input_dev = input_dev;

	/* power on */
	if (bt432_power_control(info, POWER_ON_SEQUENCE) == false) {
		ret = -EPERM;
		goto err_power_sequence;
	}

	if (read_data(info->client, 0xcc00,
		(u8 *)&info->cap_info.chip_code, 2) < 0) {
		pr_err("fail to read chip code\n");
		goto err_power_sequence;
	}
	pr_debug("chip code = 0x%x\n", info->cap_info.chip_code);
	m_firmware_data = m_firmware_data_432;
	info->cap_info.ic_fw_size = 24 * 1024;


/* To Do */
/* FW version read from tsp */

	memset(&info->reported_touch_info,
		0x0, sizeof(struct point_info));

	/* init touch mode */
	info->touch_mode = TOUCH_POINT_MODE;
	misc_info = info;

	if (init_touch(info, false) == false) {
		ret = -EPERM;
		goto err_input_register_device;
	}

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		info->button[i] = ICON_BUTTON_UNCHANGE;

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchscreen";
	input_dev->id.bustype = BUS_I2C;
/*	input_dev->id.vendor = 0x0001; */
	input_dev->phys = info->phys;
/*	input_dev->id.product = 0x0002; */
/*	input_dev->id.version = 0x0100; */
	input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, info->input_dev->evbit);
	set_bit(EV_KEY, info->input_dev->evbit);
	set_bit(EV_ABS, info->input_dev->evbit);
	set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);
	__set_bit(EV_LED, info->input_dev->evbit);
	__set_bit(LED_MISC, info->input_dev->ledbit);

	for (i = 0; i < MAX_SUPPORTED_BUTTON_NUM; i++)
		set_bit(BUTTON_MAPPING_KEY[i], info->input_dev->keybit);

	if (pdata->orientation & TOUCH_XY_SWAP) {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	} else {
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
			info->cap_info.MinX,
			info->cap_info.MaxX + ABS_PT_OFFSET,
			0, 0);
		input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
			info->cap_info.MinY,
			info->cap_info.MaxY + ABS_PT_OFFSET,
			0, 0);
	}

	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_WIDTH_MAJOR,
		0, 255, 0, 0);

#if (TOUCH_POINT_MODE == 2)
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MINOR,
		0, 255, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_ORIENTATION,
		-128, 127, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_ANGLE,
		-90, 90, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PALM,
		0, 1, 0, 0);
#endif

	set_bit(MT_TOOL_FINGER, info->input_dev->keybit);
	input_mt_init_slots(info->input_dev, info->cap_info.multi_fingers, 0);

	input_set_drvdata(info->input_dev, info);
	ret = input_register_device(info->input_dev);
	if (ret) {
		dev_err(&client->dev, "Unable to register %s input device\n",
					info->input_dev->name);

		goto err_input_register_device;
	}

	/* configure irq */

	info->work_state = NOTHING;
	sema_init(&info->work_lock, 1);

#if ESD_TIMER_INTERVAL
	spin_lock_init(&info->lock);
	INIT_WORK(&info->tmr_work, ts_tmr_work);
	info->esd_tmr_workqueue =
		create_singlethread_workqueue("esd_tmr_workqueue");

	if (!info->esd_tmr_workqueue) {
		dev_err(&client->dev, "Failed to create esd tmr work queue\n");
		ret = -EPERM;

		goto err_input_register_device;
	}

	esd_timer_init(info);
	esd_timer_start(CHECK_ESD_TIMER, info);
#endif

	info->irq = irq_of_parse_and_map(client->dev.of_node, 0);
	if (!info->irq) {
		dev_err(&client->dev, "Failed to retrieve IRQ from device tree.\n");
		ret = -ENODEV;
		goto err_irq_of_parse;
	}
	pdata->tsp_irq = info->irq;
	ret = request_threaded_irq(info->irq, NULL, bt432_touch_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT ,
				info->input_dev->name, info);
	if (ret) {
		dev_err(&client->dev, "Unable to register irq\n");

		goto err_request_irq;
	}
#if TOUCH_BOOSTER
	pm_qos_add_request(&info->cpufreq_qos_req_min, PM_QOS_CPUFREQ_MIN,
						PM_QOS_DEFAULT_VALUE);
#endif

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_enable(&client->dev);
#endif

	sema_init(&info->raw_data_lock, 1);

	ret = misc_register(&touch_misc_device);
	if (unlikely(ret)) {
		dev_err(&client->dev, "Failed to register touch misc device\n");

		goto err_misc_register;
	}

#ifdef SEC_FACTORY_TEST
	ret = init_sec_factory(info);
	if (unlikely(ret)) {
		dev_err(&client->dev, "Failed to init sec factory device\n");

		goto err_kthread_create_failed;
	}
#endif
	return 0;

#ifdef SEC_FACTORY_TEST
err_kthread_create_failed:
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
err_misc_register:
	free_irq(info->irq, info);
err_irq_of_parse:
err_request_irq:
	input_unregister_device(info->input_dev);
err_input_register_device:
	input_free_device(info->input_dev);
err_power_sequence:
err_alloc:
	kfree(info);
err_mem_alloc:
	gpio_free(pdata->tsp_en_gpio);
err_gpio_request:
	gpio_free(pdata->gpio_keyled);
err_no_platform_data:
	if (IS_ENABLED(CONFIG_OF))
		devm_kfree(&client->dev, (void *)pdata);
	dev_info(&client->dev, "Failed to probe\n");
	return ret;
}

static int bt432_ts_remove(struct i2c_client *client)
{
	struct bt432_ts_info *info = i2c_get_clientdata(client);
	struct bt432_ts_platform_data *pdata = info->pdata;

	disable_irq(info->irq);
	down(&info->work_lock);

	info->work_state = REMOVE;

#ifdef SEC_FACTORY_TEST
	kfree(info->factory_info);
	kfree(info->raw_data);
#endif
#if ESD_TIMER_INTERVAL
	flush_work(&info->tmr_work);
	esd_timer_stop(info);
	destroy_workqueue(info->esd_tmr_workqueue);
#endif

	if (info->irq)
		free_irq(info->irq, info);

	misc_deregister(&touch_misc_device);

	if (gpio_is_valid(pdata->gpio_int) != 0)
		gpio_free(pdata->gpio_int);
#if SUPPORTED_TOUCH_KEY_LED
	if (gpio_is_valid(pdata->gpio_keyled) != 0)
		gpio_free(pdata->gpio_keyled);
#endif
	if (pdata->tsp_supply_type == TSP_LDO_SUPPLY) {
		if (gpio_is_valid(pdata->tsp_en_gpio) != 0)
			gpio_free(pdata->tsp_en_gpio);
	}

	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	up(&info->work_lock);
	kfree(info);

	return 0;
}

static struct i2c_device_id bt432_idtable[] = {
	{BT432_TS_DEVICE, 0},
	{ }
};

#if defined(CONFIG_PM)
static const struct dev_pm_ops bt432_ts_pm_ops = {
#if defined(CONFIG_PM_RUNTIME)
	SET_RUNTIME_PM_OPS(bt432_ts_suspend, bt432_ts_resume, NULL)
#else
	.suspend        = bt432_ts_suspend,
	.resume         = bt432_ts_resume,
#endif
};
#endif

static struct i2c_driver bt432_ts_driver = {
	.probe	= bt432_ts_probe,
	.remove	= bt432_ts_remove,
	.id_table	= bt432_idtable,
	.driver		= {
				.owner	= THIS_MODULE,
				.name	= BT432_TS_DEVICE,
#if defined(CONFIG_PM)
				.pm	= &bt432_ts_pm_ops,
#endif
	.of_match_table = of_match_ptr(tsp_dt_ids),
	},
};

#if defined(CONFIG_SPA) || defined(CONFIG_SPA_LPM_MODE)
extern int spa_lpm_charging_mode_get();
#else
extern unsigned int lpcharge;
#endif

/*
 * TODO: Uncomment after adding battery driver.
 *	Because LP charging mode state declared in board-kaverilte-battery.c
 */
static int __init bt432_ts_init(void)
{
	pr_debug("[TSP]: %s\n", __func__);
/*
#if defined(CONFIG_SPA) || defined(CONFIG_SPA_LPM_MODE)
	if (!spa_lpm_charging_mode_get())
#else
	if (!lpcharge)
#endif
*/
		return i2c_add_driver(&bt432_ts_driver);
/*	else
		return 0;
*/
}

static void __exit bt432_ts_exit(void)
{
	i2c_del_driver(&bt432_ts_driver);
}

module_init(bt432_ts_init);
module_exit(bt432_ts_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("<mika.kim@samsung.com>");
MODULE_LICENSE("GPL");
