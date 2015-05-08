/*
 * d2199-core.c  --  Device access for Dialog D2199
 *
 * Copyright(c) 2013 Dialog Semiconductor Ltd.
 *
 * Author: Dialog Semiconductor Ltd. D. Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/bug.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/i2c.h>

#include <linux/smp.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>

#include <linux/d2199/d2199_version.h>
#include <linux/d2199/pmic.h>
#include <linux/d2199/d2199_reg.h>
#include <linux/d2199/rtc.h>
#include <linux/d2199/hwmon.h>
#include <linux/d2199/core.h>

#ifdef CONFIG_SEC_DEBUG
#include <linux/sec-debug.h>
#endif
#include <linux/reboot.h>

/*#define D2199_REG_DEBUG */
/*#define D2199_SUPPORT_I2C_HIGH_SPEED */
/*define D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE */


#ifdef D2199_REG_DEBUG
#define D2199_MAX_HISTORY			100
#define D2199_HISTORY_READ_OP		0
#define D2199D_HISTORY_WRITE_OP		1

#define d2199_write_reg_cache(reg, data)   (gD2199RegCache[reg] = data);


struct d2199_reg_history {
	u8 mode;
	u8 regnum;
	u8 value;
	long long time;
};

static u8 gD2199RegCache[D2199_MAX_REGISTER_CNT];
static u8 gD2199CurHistory;
static struct d2199_reg_history gD2199RegHistory[D2199_MAX_HISTORY];
#endif /* D2199_REG_DEBUG */

/*
 *   Static global variable
 */
static struct d2199 *d2199_dev_info;

struct d2199 *d2199_regl_info;
EXPORT_SYMBOL(d2199_regl_info);

/*
 * D2199 Device IO
 */

#ifdef D2199_REG_DEBUG
void d2199_write_reg_history(u8 opmode, u8 reg, u8 data)
{

	if (gD2199CurHistory == D2199_MAX_HISTORY)
		gD2199CurHistory = 0;

	gD2199RegHistory[gD2199CurHistory].mode = opmode;
	gD2199RegHistory[gD2199CurHistory].regnum = reg;
	gD2199RegHistory[gD2199CurHistory].value = data;
	gD2199CurHistory++;
}
#endif /* D2199_REG_DEBUG */
static int d2199_read(struct d2199 *d2199, u8 reg, int num_regs, u8 *dest)
{
	int bytes = num_regs;

	if (d2199->read_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2199_MAX_REGISTER_CNT) {
		dev_err(d2199->dev, "Invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}

	/* Actually read it out */
	return d2199->read_dev(d2199, reg, bytes, (char *)dest);
}

static int d2199_write(struct d2199 *d2199, u8 reg, int num_regs, u8 *src)
{
	int bytes = num_regs;

	if (d2199->write_dev == NULL)
		return -ENODEV;

	if ((reg + num_regs - 1) > D2199_MAX_REGISTER_CNT) {
		dev_err(d2199->dev, "Invalid reg %x\n", reg + num_regs - 1);
		return -EINVAL;
	}
	/* Actually write it out */
	return d2199->write_dev(d2199, reg, bytes, (char *)src);
}

int d2199_clear_bits(struct d2199 * const d2199, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&d2199->d2199_io_mutex);
	err = d2199_read(d2199, reg, 1, &data);
	if (err != 0) {
		dev_err(d2199->dev, "read from reg R%d failed\n", reg);
		goto out;
	}
#ifdef D2199_REG_DEBUG
	else
		d2199_write_reg_history(D2199_HISTORY_READ_OP, reg, data);
#endif
	data &= ~mask;
	err = d2199_write(d2199, reg, 1, &data);
	if (err != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2199_REG_DEBUG
	else  {
		d2199_write_reg_history(D2199D_HISTORY_WRITE_OP, reg, data);
		d2199_write_reg_cache(reg, data);
	}
#endif
out:
	mutex_unlock(&d2199->d2199_io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2199_clear_bits);

int d2199_set_bits(struct d2199 * const d2199, u8 const reg, u8 const mask)
{
	u8 data;
	int err;

	mutex_lock(&d2199->d2199_io_mutex);
	err = d2199_read(d2199, reg, 1, &data);
	if (err != 0) {
		dev_err(d2199->dev, "read from reg R%d failed\n", reg);
		goto out;
	}
#ifdef D2199_REG_DEBUG
	else
		d2199_write_reg_history(D2199_HISTORY_READ_OP, reg, data);
#endif
	data |= mask;
	err = d2199_write(d2199, reg, 1, &data);
	if (err != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2199_REG_DEBUG
	else  {
		d2199_write_reg_history(D2199D_HISTORY_WRITE_OP, reg, data);
		d2199_write_reg_cache(reg, data);
	}
#endif
out:
	mutex_unlock(&d2199->d2199_io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2199_set_bits);

int d2199_reg_read(struct d2199 * const d2199, u8 const reg, u8 *dest)
{
	u8 data;
	int err;

	mutex_lock(&d2199->d2199_io_mutex);
	err = d2199_read(d2199, reg, 1, &data);
	if (err != 0)
		dlg_err("read from reg R%d failed\n", reg);
#ifdef D2199_REG_DEBUG
	else
		d2199_write_reg_history(D2199_HISTORY_READ_OP, reg, data);
#endif
	*dest = data;
	mutex_unlock(&d2199->d2199_io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2199_reg_read);

int d2199_reg_write(struct d2199 * const d2199, u8 const reg, u8 const val)
{
	int ret;
	u8 data = val;

	mutex_lock(&d2199->d2199_io_mutex);
	ret = d2199_write(d2199, reg, 1, &data);
	if (ret != 0)
		dlg_err("write to reg R%d failed\n", reg);
#ifdef D2199_REG_DEBUG
	else  {
		d2199_write_reg_history(D2199D_HISTORY_WRITE_OP, reg, data);
		d2199_write_reg_cache(reg, data);
	}
#endif
	mutex_unlock(&d2199->d2199_io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d2199_reg_write);

/*
 * d2199_reg_write_lock -
 * @ d2199 :
 * @ reg :
 * @ val :
 *
 */
int d2199_reg_write_lock(struct d2199 * const d2199, u8 const reg, u8 const val)
{
	int ret;
	u8 data = val;

	mutex_lock(&d2199->d2199_io_mutex);
	ret = d2199_write(d2199, reg, 1, &data);
	if (ret != 0)
		dlg_err("write to reg R%d failed\n", reg);
	return ret;
}
EXPORT_SYMBOL_GPL(d2199_reg_write_lock);
int d2199_block_read(struct d2199 * const d2199, u8 const start_reg,
			u8 const regs, u8 * const dest)
{
	int err = 0;
#ifndef D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE
	int i;
#endif

	mutex_lock(&d2199->d2199_io_mutex);
#ifndef D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE
	for (i = 0; i < regs; i++) {
		err = d2199_read(d2199, start_reg+i, 1, (dest+i));
		if (err != 0)
			dlg_err("block read starting from R%d failed\n",
							start_reg);
#ifdef D2199_REG_DEBUG
		else
			d2199_write_reg_history(D2199D_HISTORY_WRITE_OP,
							start_reg+i, *(dest+i));
#endif
	}
#else
	err = d2199_read(d2199, start_reg, regs, dest);
	if (err != 0)
		dlg_err("block read starting from R%d failed\n", start_reg);
#ifdef D2199_REG_DEBUG
	else {
		for (i = 0; i < regs; i++)
			d2199_write_reg_history(D2199D_HISTORY_WRITE_OP,
							start_reg+i, *(dest+i));
	}
#endif
#endif	/* D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE */
	mutex_unlock(&d2199->d2199_io_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(d2199_block_read);

int d2199_block_write(struct d2199 * const d2199, u8 const start_reg,
			u8 const regs, u8 * const src)
{
	int ret = 0;
#ifndef D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE
	int i;
#endif

	mutex_lock(&d2199->d2199_io_mutex);
#ifndef D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE

	for (i = 0; i < regs; i++) {
		ret = d2199_write(d2199, start_reg+i, 1, (src+i));
		if (ret != 0)
			dlg_err("block write starting at R%d failed\n",
							start_reg);
#ifdef D2199_REG_DEBUG
		else
				d2199_write_reg_cache(start_reg+i, *(src+i));
#endif
	}
#else
	ret = d2199_write(d2199, start_reg, regs, src);
	if (ret != 0)
		dlg_err("block write starting at R%d failed\n", start_reg);
#ifdef D2199_REG_DEBUG
	else {
		for (i = 0; i < regs; i++)
			d2199_write_reg_cache(start_reg+i, *(src+i));
	}
#endif
#endif	/* D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE */
	mutex_unlock(&d2199->d2199_io_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(d2199_block_write);

int d2199_extern_reg_read(u8 const reg, u8 *dest)
{
	int ret = -1;

	if (d2199_regl_info == NULL)
		return -1;

	ret = d2199_reg_read(d2199_regl_info, reg, dest);

	return ret;
}
EXPORT_SYMBOL(d2199_extern_reg_read);

int d2199_extern_reg_write(u8 const reg, u8 const val)
{
	int ret = -1;

	if (d2199_regl_info == NULL)
		return -1;

	ret = d2199_reg_write(d2199_regl_info, reg, val);

	return ret;
}
EXPORT_SYMBOL(d2199_extern_reg_write);

/*
 * d2199_extern_block_read - Read block of D2199 registers
 *
 * @start_reg: Register address of D2199 to start reading
 * @dest: Buffer pointer where register contents to be stored
 *        Note: the dest buffer size has to atleast the 'num_regs'.
 * @num_regs: Number of consecutive registers to be read
 */
int d2199_extern_block_read(u8 const start_reg, u8 *dest, u8 num_regs)
{
	int err;
	struct d2199 *const d2199 = d2199_regl_info;

	if (d2199 == NULL)
		return -EINVAL;

	err = d2199_block_read(d2199, start_reg, num_regs, dest);
	if (err != 0)
		pr_err("Error! D2199 register read from 0x%02X\n", start_reg);

	return err;
}
EXPORT_SYMBOL(d2199_extern_block_read);

/*
 * Register a client device.  This is non-fatal since there is no need to
 * fail the entire device init due to a single platform device failing.
 */
static void d2199_client_dev_register(struct d2199 *d2199,
					   const char *name,
					   struct platform_device **pdev)
{
	int ret;
	*pdev = platform_device_alloc(name, -1);
	if (*pdev == NULL) {
		dev_err(d2199->dev, "Failed to allocate %s\n", name);
		return;
	}

	(*pdev)->dev.parent = d2199->dev;
	platform_set_drvdata(*pdev, d2199);
	ret = platform_device_add(*pdev);
	if (ret != 0) {
		dev_err(d2199->dev, "Failed to register %s: %d\n", name, ret);
		platform_device_put(*pdev);
		*pdev = NULL;
	}
}

/*
 * d2199_aad_dev_register
 */
static void d2199_codec_dev_register(struct d2199 *d2199,
					   const char *name)
{
	struct i2c_board_info info;

	strncpy(info.type, name, sizeof(info.type) - 1);
	/* D2199 actual address */
	info.addr = D2199_CODEC_I2C_ADDR;
	info.platform_data = d2199;
	info.flags = 0;
	info.irq = 0;
	info.archdata = NULL;
	info.of_node = NULL;

	/* Add the Codec I2C device.  This calls the probe() function. */

	d2199->codec_i2c_client = i2c_new_device
			(d2199->pmic_i2c_client->adapter, &info);

	if (!d2199->codec_i2c_client)
		dev_err(d2199->dev, "Unable to add I2C device for 0x%x\n",
								info.addr);
}


static void d2199_codec_dev_unregister(struct i2c_client *codec_i2c_client)
{
	 i2c_unregister_device(codec_i2c_client);
}

/*
 * d2199_system_event_handler
 */
static irqreturn_t d2199_system_event_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

/*
 * d2199_system_event_init
 */
static void d2199_system_event_init(struct d2199 *d2199)
{
	d2199_register_irq(d2199, D2199_IRQ_EVDD_MON,
		d2199_system_event_handler, 0, "VDD MON", d2199);
}

/*****************************************/
/*	Debug using proc entry		   */
/*****************************************/
#ifdef CONFIG_PROC_FS

static int d2199_ioctl_open(struct inode *inode, struct file *file)
{
	dlg_info("%s\n", __func__);
	file->private_data = PDE_DATA(inode);
	return 0;
}

int d2199_ioctl_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*
 * d2199_ioctl
 */

static long d2199_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
		struct d2199 *d2199 = (struct d2199 *) file->private_data;
		pmu_reg reg;
		long ret = 0;
		u8 reg_val;

		if (!d2199)
			return -ENOTTY;

		switch (cmd) {
		case D2199_IOCTL_READ_REG:
			if (copy_from_user(&reg, (pmu_reg *)arg,
						sizeof(pmu_reg)) != 0)
				return -EFAULT;

			ret = d2199_read(d2199, reg.reg, 1, &reg_val);
			reg.val = (unsigned short)reg_val;
			if (copy_to_user((pmu_reg *)arg, &reg,
						sizeof(pmu_reg)) != 0)
				return -EFAULT;
			break;

		case D2199_IOCTL_WRITE_REG:
			if (copy_from_user(&reg, (pmu_reg *)arg,
						sizeof(pmu_reg)) != 0)
				return -EFAULT;
			reg_val = (u8)reg.val;
			d2199_write(d2199, reg.reg, 1, &reg_val);
			break;

		default:
			dlg_err("%s: unsupported cmd\n", __func__);
			ret = -ENOTTY;
		}

		return ret;
}

#define MAX_USER_INPUT_LEN	  100
#define MAX_REGS_READ_WRITE	 10

enum pmu_debug_ops {
	PMUDBG_READ_REG = 0UL,
	PMUDBG_WRITE_REG,
};

struct pmu_debug {
	int read_write;
	int len;
	int addr;
	u8 val[MAX_REGS_READ_WRITE];
};

/*
 * d2199_dbg_usage
 */
static void d2199_dbg_usage(void)
{
	pr_info("Usage:\n");
	pr_info("Read a register: echo 0x0800 > /proc/pmu0\n");
	pr_info("Read multiple regs: echo 0x0800 -c 10 > /proc/pmu0\n");
	pr_info("Write multiple regs: echo 0x0800 0xFF 0xFF > /proc/pmu0\n");
	pr_info("Write single reg: echo 0x0800 0xFF > /proc/pmu0\n");
	pr_info("Max number of regs in single write is :%d\n",
		MAX_REGS_READ_WRITE);
	pr_info("Register address is encoded as follows:\n");
	pr_info("0xSSRR, SS: i2c slave addr, RR: register addr\n");
}


/*
 * d2199_dbg_parse_args
 */
static int d2199_dbg_parse_args(char *cmd, struct pmu_debug *dbg)
{
	char *tok;		 /* used to separate tokens	*/
	const char ct[] = " \t";   /* space or tab delimits the tokens	*/
	bool count_flag = false;   /* whether -c option is present or not */
	int tok_count = 0;		 /* total number of tokens parsed */
	int i = 0;

	dbg->len		= 0;

	/* parse the input string */
	while ((tok = strsep(&cmd, ct)) != NULL) {
		dlg_info("token: %s\n", tok);

		/* first token is always address */
		if (tok_count == 0) {
			sscanf(tok, "%x", &dbg->addr);
		} else if (strnicmp(tok, "-c", 2) == 0) {
			/* the next token will be number of regs to read */
			tok = strsep(&cmd, ct);
			if (tok == NULL)
				return -EINVAL;

			tok_count++;
			sscanf(tok, "%d", &dbg->len);
			count_flag = true;
			break;
		} else {
			int val;
			/* this is a value to be written to the pmu register */
			sscanf(tok, "%x", &val);
			if (i < MAX_REGS_READ_WRITE) {
				dbg->val[i] = val;
				i++;
			}
		}

		tok_count++;
	}

	/* decide whether it is a read or write operation based on the
	 * value of tok_count and count_flag.
	 * tok_count = 0: no inputs, invalid case.
	 * tok_count = 1: only reg address is given, so do a read.
	 * tok_count > 1, count_flag = false: reg address and atleast one
	 *	 value is present, so do a write operation.
	 * tok_count > 1, count_flag = true: to a multiple reg read operation.
	 */
	switch (tok_count) {
	case 0:
		return -EINVAL;
	case 1:
		dbg->read_write = PMUDBG_READ_REG;
		dbg->len = 1;
		break;
	default:
		if (count_flag == true) {
			dbg->read_write = PMUDBG_READ_REG;
		} else {
			dbg->read_write = PMUDBG_WRITE_REG;
			dbg->len = i;
		}
	}
	return 0;
}

/*
 * d2199_ioctl_write
 */

static ssize_t d2199_ioctl_write(struct file *file, const char __user *buffer,
	size_t len, loff_t *offset)
{
	struct d2199 *d2199 = file->private_data;
	struct pmu_debug dbg;
	char cmd[MAX_USER_INPUT_LEN];
	int ret, i;

	dlg_info("%s\n", __func__);

	if (!d2199) {
		dlg_err("%s: driver not initialized\n", __func__);
		return -EINVAL;
	}

	if (len > MAX_USER_INPUT_LEN)
		len = MAX_USER_INPUT_LEN;

	if (copy_from_user(cmd, buffer, len)) {
		dlg_err("%s: copy_from_user failed\n", __func__);
		return -EFAULT;
	}

	/* chop of '\n' introduced by echo at the end of the input */
	if (cmd[len - 1] == '\n')
		cmd[len - 1] = '\0';

	if (d2199_dbg_parse_args(cmd, &dbg) < 0) {
		d2199_dbg_usage();
		return -EINVAL;
	}

	dlg_info("operation: %s\n", (dbg.read_write == PMUDBG_READ_REG) ?
		"read" : "write");
	dlg_info("address  : 0x%x\n", dbg.addr);
	dlg_info("length   : %d\n", dbg.len);

	if (dbg.read_write == PMUDBG_READ_REG) {
		ret = d2199_read(d2199, dbg.addr, dbg.len, dbg.val);
		if (ret < 0) {
			dlg_err("%s: pmu reg read failed\n", __func__);
			return -EFAULT;
		}

		for (i = 0; i < dbg.len; i++, dbg.addr++)
			dlg_info("[%x] = 0x%02x\n", dbg.addr,
				dbg.val[i]);
	} else {
		ret = d2199_write(d2199, dbg.addr, dbg.len, dbg.val);
		if (ret < 0) {
			dlg_err("%s: pmu reg write failed\n", __func__);
			return -EFAULT;
		}
	}

	*offset += len;
	return len;
}


static const struct file_operations d2199_pmu_ops = {
	.open = d2199_ioctl_open,
	.unlocked_ioctl = d2199_ioctl,
	.write = d2199_ioctl_write,
	.release = d2199_ioctl_release,
	.owner = THIS_MODULE,
};

/*
 * d2199_debug_proc_init
 */
void d2199_debug_proc_init(struct d2199 *d2199)
{
	struct proc_dir_entry *entry;

	disable_irq(d2199->chip_irq);
	entry = proc_create_data("pmu0", S_IRWXUGO, NULL, &d2199_pmu_ops,
								d2199);
	enable_irq(d2199->chip_irq);

	dlg_crit("\nD2199-core.c: proc_create_data() = %p; status=\"%s\"\n",
					entry, (entry ? "PASS" : "FAILED"));
}

/*
 * d2199_debug_proc_exit
 */
void d2199_debug_proc_exit(void)
{
	remove_proc_entry("pmu0", NULL);
}
#endif /* CONFIG_PROC_FS */

#if defined(CONFIG_MFD_D2199_BRINGUP_RECHECK)
/*
 * NOTES: DVC is used to change voltage, currently use max
 * voltage lvl 4, could add if has requirement
 */
enum {
	VL0 = 0,
	VL1,
	VL2,
	VL3,
	VL_MAX,
};

extern unsigned int pxa988_get_vl_num(void);
extern unsigned int pxa988_get_vl(unsigned int vl);

int dlg_buck_volts[VL_MAX] = {
	0x58, 0x67, 0x68
};

int d2199_dvfs_get_voltage(int reg_addr)
{
	int val = 0;


	if (reg_addr < VL_MAX) {
		pr_info("[WS][DVFS][%s] reg_addr[%d], dlg_buck_volts[0x%x]\n",
			__func__, reg_addr, dlg_buck_volts[reg_addr]);
		val = dlg_buck_volts[reg_addr];
		return val;
	} else
		return -1;
}
EXPORT_SYMBOL(d2199_dvfs_get_voltage);

#endif
#define CHG_STATUS_HIGH 1
#define CHG_STATUS_LOW 0

static u8 power_on_reason;
unsigned char d2199_get_power_on_reason(void)
{
	return power_on_reason;
}

#ifdef CONFIG_SEC_PM
extern struct class *sec_class;

#define ONKEY_LONG_PRESS_MIN_TIME 3000
#define ONKEY_LONG_PRESS_STEP_TIME 500
#define ONKEY_LONG_PRESS_TIME_BIT (BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define ONKEY_SD_BIT BIT(4)

static int d2199_set_onkey_long_press_time(int time)
{
	struct d2199 *d2199 = d2199_dev_info;
	u8 val;
	u8 press_time_val = (time - ONKEY_LONG_PRESS_MIN_TIME) /
		ONKEY_LONG_PRESS_STEP_TIME;
	int ret;

	dlg_info("%s time : %d(msec)\n", __func__, time);

	d2199_reg_read(d2199, D2199_ONKEY_CONT1_REG, &val);
	val &= ~(ONKEY_LONG_PRESS_TIME_BIT);
	val |= press_time_val;
	ret = d2199_reg_write(d2199, D2199_ONKEY_CONT1_REG, val);

	return ret;
}

static ssize_t switch_show_onkey_long_press(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct d2199 *d2199 = d2199_dev_info;
	u8 val;

	d2199_reg_read(d2199, D2199_CONTROL_D_REG, &val);
	dlg_info("%s, val = 0x%x\n", __func__, val);
	val &= ONKEY_SD_BIT;

	return sprintf(buf, "%d", val ? 1 : 0);
}

static ssize_t switch_store_onkey_long_press(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct d2199 *d2199 = d2199_dev_info;
	u8 val;
	int ret;

	d2199_reg_read(d2199, D2199_CONTROL_D_REG, &val);

	if (!strncmp(buf, "0", 1))
		val &= ~(ONKEY_SD_BIT);
	else if (!strncmp(buf, "1", 1))
		val |= ONKEY_SD_BIT;
	else {
		dlg_warn("%s: Wrong command:%s\n", __func__, buf);
		return -EINVAL;
	}

	ret = d2199_reg_write(d2199, D2199_CONTROL_D_REG, val);

	if (ret != 0)
		return ret;

	return count;
}

static DEVICE_ATTR(onkey_long_press, 0664, switch_show_onkey_long_press,
			switch_store_onkey_long_press);
#endif /* CONFIG_SEC_PM */

#ifdef CONFIG_MACH_PXA_SAMSUNG

int d2199_reboot_notifier_func(struct notifier_block *this,
					unsigned long code, void *p)
{
	char *cmd = p;
	unsigned char data;
	unsigned char pmic_download_register;
	unsigned char pmic_register;
	pr_emerg("reboot notifier: %s\n", cmd);
	pm_power_off = d2199_system_poweroff;
	if (cmd && !strcmp(cmd, "recovery")) {
		d2199_extern_reg_read(D2199_GP_ID_1_REG, &data);
		d2199_extern_reg_write(D2199_GP_ID_1_REG, data | 0x1);
	} else {
		d2199_extern_reg_read(D2199_GP_ID_1_REG, &data);
		d2199_extern_reg_write(D2199_GP_ID_1_REG, data & 0xfe);
	}
	if (cmd) {
		if (!strcmp(cmd, "recovery"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_FULL_RESET);
		else if (!strcmp(cmd, "recovery_done"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_RECOVERY_DONE);
		else if (!strcmp(cmd, "arm11_fota"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_FOTA);
		else if (!strcmp(cmd, "alarm"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_RTC_ALARM);
		else if (!strcmp(cmd, "debug0x4f4c"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_LOW);
		else if (!strcmp(cmd, "debug0x494d"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_MID);
		else if (!strcmp(cmd, "debug0x4948"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_DEBUGLEVEL_HIGH);
		else if (!strcmp(cmd, "GlobalActions restart"))
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET);
		else if (!strcmp(cmd, "download")) {
			pmic_download_register = PMIC_GENERAL_DOWNLOAD_MODE_FUS;
			pmic_register = (pmic_download_register<<4)&0xF0;
			d2199_extern_reg_write(D2199_GP_ID_1_REG,
							pmic_register);
			pr_info("pmic_download_register FUS : %d\n",
							pmic_download_register);
		} else if (!strncmp(cmd, "sud", 3)) {
			pmic_download_register = ((char *)cmd)[3] - '0';
			pmic_register = (pmic_download_register << 4) & 0xF0;
			d2199_extern_reg_write(D2199_GP_ID_1_REG,
							pmic_register);
			pr_info("pmic_download_register SUDDLMOD : %d\n",
							pmic_download_register);
		} else
			d2199_extern_reg_write(D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_INTENDED_RESET);
	}
	if (code != SYS_POWER_OFF) {
		d2199_extern_reg_read(D2199_GP_ID_1_REG, &data);
		/* this bit is for charger server */
		d2199_extern_reg_write(D2199_GP_ID_1_REG, data | 0x2);
	}

	return 0;
}
#endif

int d2199_device_init(struct d2199 *d2199, int irq,
			   struct d2199_platform_data *pdata)
{
	u8 res_msb, res_lsb, val;
	u16 read_adc;
	int ret = 0;
#ifdef D2199_REG_DEBUG
	int i;
	u8 data;
#endif

	if (d2199 != NULL)
		d2199_regl_info = d2199;
	else
		goto err;

	dlg_info("D2199 Driver : built at %s on %s\n", __TIME__, __DATE__);

	/*DLG eric. d2199->pmic.max_dcdc = 25; */
	d2199->pdata = pdata;

	mutex_init(&d2199->d2199_io_mutex);

	d2199_reg_write(d2199, D2199_GPADC_MCTL_REG, 0x55);
	usleep_range(1000, 1000);

	d2199_reg_write(d2199, D2199_ADC_CONT_REG, (D2199_ADC_AUTO_EN_MASK |
	  D2199_ADC_MODE_MASK | D2199_AUTO_VBAT_EN_MASK));
	udelay(100);
	d2199_reg_read(d2199, D2199_VDD_RES_VBAT_RES_REG, &res_msb);
	d2199_reg_read(d2199, D2199_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2199->vbat_init_adc[0] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n",
		__LINE__, __func__, d2199->vbat_init_adc[0]);
	d2199_reg_write(d2199, D2199_ADC_CONT_REG, 0x0);

#ifdef D2199_SUPPORT_I2C_HIGH_SPEED
	d2199_set_bits(d2199, D2199_CONTROL_B_REG, D2199_I2C_SPEED_MASK);

/* Page write for I2C we donot support repeated write
* and I2C speed set to 1.7MHz
*/
	d2199_clear_bits(d2199, D2199_CONTROL_B_REG, D2199_WRITE_MODE_MASK);
#else
#ifdef D2199_SUPPORT_I2C_USE_PAGE_WRITE_MODE
/* Page write for I2C we donot support repeated write and I2C
* speed set to 400KHz */
	d2199_clear_bits(d2199, D2199_CONTROL_B_REG, D2199_WRITE_MODE_MASK
			| D2199_I2C_SPEED_MASK);
#else
	d2199_clear_bits(d2199, D2199_CONTROL_B_REG, D2199_I2C_SPEED_MASK);
#endif
#endif

	d2199_reg_write(d2199, D2199_PD_DIS_REG, 0x0);
	d2199_reg_write(d2199, D2199_PULLDOWN_D_REG, 0x0C);
	d2199_reg_write(d2199, D2199_BBAT_CONT_REG, 0x1F);
	d2199_set_bits(d2199,  D2199_SUPPLY_REG, D2199_BBCHG_EN_MASK);

	d2199_reg_write(d2199, D2199_GPADC_MCTL_REG, 0x55);

	d2199_set_bits(d2199,  D2199_OUT2_32K_ONKEY_CONT_REG,
					D2199_OUT2_32K_EN_MASK);

	/* test */
	d2199_reg_write(d2199, D2199_AUDIO_REG_DFLT_0_REG, 0x0);
	d2199_reg_write(d2199, D2199_AUDIO_REG_DFLT_7_REG, 0x21);

	d2199_dev_info = d2199;

	ret = d2199_irq_init(d2199, irq, pdata);
	if (ret < 0)
		goto err;

	if (pdata && pdata->init) {
		ret = pdata->init(d2199);
		if (ret != 0) {
			dev_err(d2199->dev, "Platform init() failed: %d\n",
									  ret);
			goto err_irq;
		}
	}

	d2199_reg_write(d2199, D2199_ADC_CONT_REG, (D2199_ADC_AUTO_EN_MASK |
		   D2199_ADC_MODE_MASK | D2199_AUTO_VBAT_EN_MASK));
	udelay(100);
	d2199_reg_read(d2199, D2199_VDD_RES_VBAT_RES_REG, &res_msb);
	d2199_reg_read(d2199, D2199_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2199->vbat_init_adc[1] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n",
				__LINE__, __func__, d2199->vbat_init_adc[1]);
	d2199_reg_write(d2199, D2199_ADC_CONT_REG, 0x0);
	/* Regulator Specific Init */
	ret = d2199_platform_regulator_init(d2199);
	if (ret != 0) {
		dev_err(d2199->dev, "Platform Regulator init() failed: %d\n",
									ret);
		goto err_irq;
	}

	d2199_reg_write(d2199, D2199_ADC_CONT_REG, (D2199_ADC_AUTO_EN_MASK |
			D2199_ADC_MODE_MASK | D2199_AUTO_VBAT_EN_MASK));
	udelay(100);
	d2199_reg_read(d2199, D2199_VDD_RES_VBAT_RES_REG, &res_msb);
	d2199_reg_read(d2199, D2199_ADC_RES_AUTO1_REG, &res_lsb);
	read_adc = (((res_msb & 0xFF) << 4) | (res_lsb & 0x0F));
	d2199->vbat_init_adc[2] = read_adc;
	pr_info(">>>>>>>>>>>> [L%04d] %s. READ VBAT ADC is %d\n",
				__LINE__, __func__, d2199->vbat_init_adc[2]);
	d2199_reg_write(d2199, D2199_ADC_CONT_REG, 0x0);
	d2199_client_dev_register(d2199, "d2199-onkey", &(d2199->onkey.pdev));
	d2199_codec_dev_register(d2199, "d2199-codec");
	d2199_system_event_init(d2199);
#ifdef CONFIG_PROC_FS
	d2199_debug_proc_init(d2199);
#endif

#ifdef D2199_REG_DEBUG
	for (i = 0; i < D2199_MAX_REGISTER_CNT; i++) {
		d2199_reg_read(d2199, i, &data);
		d2199_write_reg_cache(i, data);
	}
#endif

	d2199_clk32k_enable(1); /* for Aruba, wilcox */

	/* set MCTRL_EN enabled */
	d2199_set_mctl_enable();

#ifdef D2199_REG_DEBUG
	data = 0;
	d2199_reg_read(d2199, D2199_STATUS_A_REG, &data);
#endif

#if defined(CONFIG_MFD_D2199_BRINGUP_RECHECK)
{
	extern void d2199_usb_configure(void);

	d2199_usb_configure();
}
#endif
	/* read power on reason from PMIC general use register */
	d2199_reg_read(d2199, D2199_GP_ID_0_REG, &val);
	pr_info("%s read register D2199_GP_ID_0_REG [0x%x]\n", __func__, val);

	/* read power on reason from PMIC general use register */
	if (val != PMIC_GENERAL_USE_BOOT_BY_FULL_RESET) {
		pr_info("%s writeregister D2199_GP_ID_0_REG [0x%x]\n", __func__,
					PMIC_GENERAL_USE_BOOT_BY_HW_RESET);
		d2199_reg_write(d2199, D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_HW_RESET);
	}
	power_on_reason	= (u8)val;

	d2199_reg_read(d2199, D2199_GP_ID_0_REG, &val);
	pr_info("%s read register D2199_GP_ID_0_REG [0x%x]\n", __func__, val);

#if defined(CONFIG_SEC_PM)
	if (pdata->onkey_long_press_time)
		d2199_set_onkey_long_press_time(pdata->onkey_long_press_time);

	d2199->sec_power = device_create(sec_class, NULL, 0, NULL, "sec_power");
	if (IS_ERR(d2199->sec_power)) {
		ret = PTR_ERR(d2199->sec_power);
		dev_err(d2199->dev, "Failed to create sec_power:%d\n", ret);
		return ret;
	}
	dev_set_drvdata(d2199->sec_power, d2199);

	ret = device_create_file(d2199->sec_power, &dev_attr_onkey_long_press);
	if (ret < 0) {
		dev_err(d2199->dev, "Failed to create onkey_long_press:%d\n", ret);
		return ret;
	}
#endif /* CONFIG_SEC_PM */

#ifdef CONFIG_MACH_PXA_SAMSUNG
	d2199->reboot_notifier.notifier_call = d2199_reboot_notifier_func;
	register_reboot_notifier(&(d2199->reboot_notifier));
#endif

	return 0;

err_irq:
	d2199_irq_exit(d2199);
	d2199_dev_info = NULL;
	pm_power_off = NULL;
err:
	dlg_crit("D2199-core.c: device init failed !\n");
	return ret;
}
EXPORT_SYMBOL_GPL(d2199_device_init);


/*
 * d2199_device_exit
 */
void d2199_device_exit(struct d2199 *d2199)
{
#ifdef CONFIG_PROC_FS
	d2199_debug_proc_exit();
#endif
	d2199_dev_info = NULL;

	platform_device_unregister(d2199->batt.pdev);
	platform_device_unregister(d2199->rtc.pdev);
	platform_device_unregister(d2199->onkey.pdev);
	d2199_codec_dev_unregister(d2199->codec_i2c_client);

	d2199_free_irq(d2199, D2199_IRQ_EVDD_MON);
	d2199_irq_exit(d2199);
}
EXPORT_SYMBOL_GPL(d2199_device_exit);

/*
 * d2199_system_poweroff
 */
void d2199_system_poweroff(void)
{
	u8 dst;
	struct d2199 *d2199 = d2199_dev_info;

	dlg_info("%s\n", __func__);

	preempt_enable();

	if (d2199 == NULL || d2199->read_dev == NULL) {
		dlg_err("%s. Platform or Device data is NULL\n", __func__);
		return;
	}

	/* Disable OTP reloading and MCTL */
	d2199_clear_bits(d2199, D2199_CONTROL_B_REG, D2199_OTPREAD_EN_MASK);
	d2199_clear_bits(d2199, D2199_POWER_CONT_REG, D2199_MCTRL_EN_MASK);

	/* Disable nOnkey interrupt */
	d2199_reg_write(d2199, D2199_IRQ_MASK_B_REG, 0x0); /*onkey mask clear */



	d2199_reg_read(d2199, D2199_GP_ID_0_REG, &dst);
	d2199_reg_read(d2199, D2199_GP_ID_0_REG, &dst);

	/* save power off reason */
	d2199_reg_write(d2199, D2199_GP_ID_0_REG,
				PMIC_GENERAL_USE_BOOT_BY_NONE);

	d2199_reg_write(d2199, D2199_POWER_CONT_REG, 0x0A);
	d2199_reg_write(d2199, D2199_PD_DIS_REG, 0xCF);

	d2199_reg_write(d2199, D2199_SUPPLY_REG, 0x10);
	d2199_reg_write(d2199, D2199_BUCK2PH_BUCK1_REG, 0xD8);

	d2199_reg_read(d2199, D2199_CONTROL_B_REG, &dst);
	dst |= D2199_DEEP_SLEEP_MASK;
	d2199_reg_write_lock(d2199, D2199_CONTROL_B_REG, dst);
	while (1)
		;
}
EXPORT_SYMBOL(d2199_system_poweroff);


MODULE_AUTHOR("Dialog Semiconductor Ltd < william.seo@diasemi.com >");
MODULE_DESCRIPTION("D2199 PMIC Core");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" D2199_I2C);
#ifdef CONFIG_MFD_D2199_BRINGUP_RECHECK
#include <linux/platform_data/mv_usb.h>

static int d2199_set_vbus(unsigned int vbus)
{
	pr_info("[WS][%s]\n", __func__);

	return 0;
}

static int d2199_read_id_val(unsigned int *level)
{

	pr_info("[WS][%s]\n", __func__);

	*level = 0;

	return 0;
}

int d2199_init_id(void)
{
	pr_info("[WS][%s]\n", __func__);
	return 0;
}


void d2199_usb_configure(void)
{

	pr_info("[WS][%s]\n", __func__);

	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, vbus, set_vbus,
				d2199_set_vbus);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, get_idpin,
				d2199_read_id_val);
	pxa_usb_set_extern_call(PXA_USB_DEV_OTG, idpin, init,
				d2199_init_id);

}
#endif
