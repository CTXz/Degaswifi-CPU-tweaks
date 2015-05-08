/*
 *  linux/arch/arm/mach-mmp/dc_profiling.c
 *
 *  Author:	Liang Chen <chl@marvell.com>
 *		Xiangzhan Meng <mengxzh@marvell.com>
 *  Copyright:	(C) 2013 Marvell International Ltd.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2012 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/clk-private.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/io.h>

#include <linux/clk/mmpdcstat.h>
#include "clk.h"

static DEFINE_PER_CPU(struct clk_dc_stat_info, cpu_dc_stat);
static DEFINE_SPINLOCK(clk_dc_lock);

static struct idle_dcstat_info idle_dcstat_info;
static spinlock_t allidle_lock;
static spinlock_t c1c2_enter_lock;
static spinlock_t c1c2_exit_lock;

static LIST_HEAD(core_dcstat_list);
static LIST_HEAD(clk_dcstat_list);

static powermode pxa_powermode;

static void clk_dutycycle_stats(struct clk *clk,
				enum clk_stat_msg msg,
				struct clk_dc_stat_info *dc_stat_info,
				unsigned int tgtstate)
{
	struct timespec cur_ts, prev_ts;
	long time_ms;
	struct op_dcstat_info *cur, *tgt;

	/* do nothing if no stat operation is issued */
	if (!dc_stat_info->stat_start)
		return;

	cur = &dc_stat_info->ops_dcstat[dc_stat_info->curopindex];
	getnstimeofday(&cur_ts);
	prev_ts = cur->prev_ts;
	time_ms = ts2ms(cur_ts, prev_ts);
	switch (msg) {
	case CLK_STAT_START:
		/* duty cycle stat start */
		cur->prev_ts = cur_ts;
		break;
	case CLK_STAT_STOP:
		/* duty cycle stat stop */
		if (clk->enable_count)
			cur->busy_time += time_ms;
		else
			cur->idle_time += time_ms;
		break;
	case CLK_STATE_ON:
		/* clk switch from off->on */
		cur->prev_ts = cur_ts;
		cur->idle_time += time_ms;
		break;
	case CLK_STATE_OFF:
		/* clk switch from off->on */
		cur->prev_ts = cur_ts;
		cur->busy_time += time_ms;
		break;
	case CLK_RATE_CHANGE:
		/* rate change from old->new */
		cur->prev_ts = cur_ts;
		if (clk->enable_count)
			cur->busy_time += time_ms;
		else
			cur->idle_time += time_ms;
		BUG_ON(tgtstate >= dc_stat_info->ops_stat_size);
		tgt = &dc_stat_info->ops_dcstat[tgtstate];
		tgt->prev_ts = cur_ts;
		break;
	default:
		break;
	}
}

int clk_register_dcstat(struct clk *clk,
			    unsigned long *opt, unsigned int opt_size)
{
	struct clk_dcstat *cdcs;
	struct clk_dc_stat_info *clk_dcstat;
	unsigned int i;
	unsigned long rate;

	/* search the list of the registation for this clk */
	list_for_each_entry(cdcs, &clk_dcstat_list, node)
	    if (cdcs->clk == clk)
		break;

	/* if clk wasn't in the list, allocate new dcstat info */
	if (cdcs->clk != clk) {
		cdcs = kzalloc(sizeof(struct clk_dcstat), GFP_KERNEL);
		if (!cdcs)
			goto out;

		rate = clk_get_rate(clk);
		cdcs->clk = clk;
		/* allocate and fill dc stat information */
		clk_dcstat = &cdcs->clk_dcstat;
		clk_dcstat->ops_dcstat = kzalloc(opt_size *
						     sizeof(struct
							    op_dcstat_info),
						     GFP_KERNEL);
		if (!clk_dcstat->ops_dcstat) {
			pr_err("%s clk %s memory allocate failed!\n",
			       __func__, clk->name);
			goto out1;
		}
		for (i = 0; i < opt_size; i++) {
			clk_dcstat->ops_dcstat[i].ppindex = i;
			clk_dcstat->ops_dcstat[i].pprate = opt[i];
			if (rate == opt[i])
				clk_dcstat->curopindex = i;
		}
		clk_dcstat->ops_stat_size = opt_size;
		clk_dcstat->stat_start = false;

		list_add(&cdcs->node, &clk_dcstat_list);
	}

	return 0;
out1:
	kfree(cdcs);
out:
	return -ENOMEM;
}
EXPORT_SYMBOL(clk_register_dcstat);

void clk_dcstat_event(struct clk *clk,
			  enum clk_stat_msg msg, unsigned int tgtstate)
{
	struct clk_dcstat *cdcs;
	struct clk_dc_stat_info *dcstat_info;

	list_for_each_entry(cdcs, &clk_dcstat_list, node)
	    if (cdcs->clk == clk) {
		dcstat_info = &cdcs->clk_dcstat;
		clk_dutycycle_stats(clk, msg, dcstat_info, tgtstate);
		/*
		 * always update curopindex, no matter stat
		 * is started or not
		 */
		if (msg == CLK_RATE_CHANGE)
			dcstat_info->curopindex = tgtstate;
		break;
	}
}
EXPORT_SYMBOL(clk_dcstat_event);

int show_dc_stat_info(struct clk *clk, char *buf, ssize_t size)
{
	int len = 0;
	unsigned int i, dc_int, dc_fraction;
	long total_time = 0, run_total = 0, idle_total = 0;
	struct clk_dcstat *cdcs;
	struct clk_dc_stat_info *dc_stat_info = NULL;

	list_for_each_entry(cdcs, &clk_dcstat_list, node)
	    if (cdcs->clk == clk) {
		dc_stat_info = &cdcs->clk_dcstat;
		break;
	}

	if (!dc_stat_info) {
		pr_err("clk %s NULL dc stat info\n", clk->name);
		return -EINVAL;
	}

	if (dc_stat_info->stat_start) {
		len += snprintf(buf + len, size - len,
				"Please stop the %s duty cycle stats at first\n",
				clk->name);
		return len;
	}

	for (i = 0; i < dc_stat_info->ops_stat_size; i++) {
		run_total += dc_stat_info->ops_dcstat[i].busy_time;
		idle_total += dc_stat_info->ops_dcstat[i].idle_time;
	}
	total_time = run_total + idle_total;
	if (!total_time) {
		len += snprintf(buf + len, size - len, "No stat information! ");
		len += snprintf(buf + len, size - len, "Help information :\n");
		len += snprintf(buf + len, size - len,
				"1. echo 1 to start duty cycle stat:\n");
		len += snprintf(buf + len, size - len,
				"2. echo 0 to stop duty cycle stat:\n");
		len += snprintf(buf + len, size - len,
				"3. cat to check duty cycle info from start to stop:\n\n");
		return len;
	}

	len += snprintf(buf + len, size - len, "\n");
	dc_int = calculate_dc(run_total, total_time, &dc_fraction);
	len += snprintf(buf + len, size - len,
			"| CLK %s | %10s %lums| %10s %lums| %10s %2u.%2u%%|\n",
			clk->name, "idle time", idle_total,
			"total time", total_time,
			"duty cycle", dc_int, dc_fraction);
	len += snprintf(buf + len, size - len,
			"| %3s | %12s | %15s | %15s | %15s |\n", "OP#",
			"rate(HZ)", "run time(ms)", "idle time(ms)",
			"rt ratio");
	for (i = 0; i < dc_stat_info->ops_stat_size; i++) {
		dc_int = calculate_dc(dc_stat_info->ops_dcstat[i].busy_time,
				     total_time, &dc_fraction);
		len += snprintf(buf + len, size - len,
				"| %3u | %12lu | %15ld | %15ld | %12u.%2u%%|\n",
				dc_stat_info->ops_dcstat[i].ppindex,
				dc_stat_info->ops_dcstat[i].pprate,
				dc_stat_info->ops_dcstat[i].busy_time,
				dc_stat_info->ops_dcstat[i].idle_time,
				dc_int, dc_fraction);
	}
	return len;
}
EXPORT_SYMBOL(show_dc_stat_info);

int start_stop_dc_stat(struct clk *clk, unsigned int start)
{
	unsigned int i;
	struct clk_dcstat *cdcs;
	struct clk_dc_stat_info *dc_stat_info = NULL;

	list_for_each_entry(cdcs, &clk_dcstat_list, node)
	    if (cdcs->clk == clk) {
		dc_stat_info = &cdcs->clk_dcstat;
		break;
	}

	if (!dc_stat_info) {
		pr_err("clk %s NULL dc stat info\n", clk->name);
		return -EINVAL;
	}

	start = !!start;
	if (start == dc_stat_info->stat_start) {
		pr_err("[WARNING]%s stat is already %s\n",
		       clk->name,
		       dc_stat_info->stat_start ? "started" : "stopped");
		return -EINVAL;
	}

	if (start) {
		/* clear old stat information */
		for (i = 0; i < dc_stat_info->ops_stat_size; i++) {
			dc_stat_info->ops_dcstat[i].idle_time = 0;
			dc_stat_info->ops_dcstat[i].busy_time = 0;
		}
		dc_stat_info->stat_start = true;
		clk_dutycycle_stats(clk, CLK_STAT_START, dc_stat_info, 0);
	} else {
		clk_dutycycle_stats(clk, CLK_STAT_STOP, dc_stat_info, 0);
		dc_stat_info->stat_start = false;
	}
	return 0;
}
EXPORT_SYMBOL(start_stop_dc_stat);

static int cpu_id;
int register_cpu_dcstat(struct clk *clk, unsigned int cpunum,
	unsigned int *op_table, unsigned int opt_size, powermode func)
{
	struct clk_dc_stat_info *cpu_dcstat;
	struct core_dcstat *op;
	unsigned int i, j, cpu, rate;
	static int cpu_dc_init;
	BUG_ON(!func);
	pxa_powermode = func;

	if (!cpu_dc_init) {
		spin_lock_init(&allidle_lock);
		spin_lock_init(&c1c2_enter_lock);
		spin_lock_init(&c1c2_exit_lock);
		cpu_dc_init++;
	}

	list_for_each_entry(op, &core_dcstat_list, node) {
		if (op->clk == clk)
			return 0;
	}

	op = kzalloc(sizeof(struct core_dcstat), GFP_KERNEL);
	if (!op) {
		pr_err("[WARNING]CPU stat info malloc failed\n");
		return -ENOMEM;
	}
	op->clk = clk;

	list_add(&op->node, &core_dcstat_list);
	op->cpu_id = kzalloc(sizeof(int) * cpunum, GFP_KERNEL);
	if (!op->cpu_id) {
		pr_err("[WARNING]CPU stat cpuid info malloc failed\n");
		goto err1;
	}

	for (i = 0; i < cpunum; i++)
		op->cpu_id[i] = cpu_id++;

	op->cpu_num = i;
	rate = clk_get_rate(clk);
	rate /= MHZ;
	/* get cur core rate */
	for (j = 0; j < op->cpu_num; j++) {
		cpu = op->cpu_id[j];
		cpu_dcstat = &per_cpu(cpu_dc_stat, cpu);
		cpu_dcstat->ops_dcstat = kzalloc(opt_size *
						 sizeof(struct
							op_dcstat_info),
						 GFP_KERNEL);
		if (!cpu_dcstat->ops_dcstat) {
			pr_err("%s: memory allocate failed!\n", __func__);
			goto err2;
		}
		for (i = 0; i < opt_size; i++) {
			cpu_dcstat->ops_dcstat[i].ppindex = i;
			/*pprate should be MHZ*/
			cpu_dcstat->ops_dcstat[i].pprate = op_table[i];
			if (cpu_dcstat->ops_dcstat[i].pprate == rate)
				cpu_dcstat->curopindex = i;
		}
		cpu_dcstat->ops_stat_size = i;
		cpu_dcstat->stat_start = false;
	}

	return 0;
err2:
	kfree(op->cpu_id);
err1:
	kfree(op);
	return -ENOMEM;
}
EXPORT_SYMBOL(register_cpu_dcstat);

void cpu_dcstat_event(struct clk *clk, unsigned int cpuid,
			  enum clk_stat_msg msg, unsigned int tgtop)
{
	struct clk_dc_stat_info *dc_stat_info = NULL;
	cputime64_t cur_wall, cur_idle;
	cputime64_t prev_wall, prev_idle;
	u32 idle_time_ms, total_time_ms;
	struct op_dcstat_info *cur, *tgt;
	unsigned int cpu_i;
	bool mark_keytime;
	ktime_t ktime_temp, ktime_temp1;
	u32 i, temp_time = 0;
	int j;
	struct core_dcstat *op;
	list_for_each_entry(op, &core_dcstat_list, node) {
		if (op->clk == clk)
			break;
	}

	dc_stat_info = &per_cpu(cpu_dc_stat, cpuid);
	cur = &dc_stat_info->ops_dcstat[dc_stat_info->curopindex];
	if (msg == CLK_RATE_CHANGE) {
		/* BUG_ON(tgtop >= dc_stat_info->ops_stat_size); */
		dc_stat_info->curopindex = tgtop;
	}
	/* do nothing if no stat operation is issued */
	if (!dc_stat_info->stat_start)
		return;

	cur_idle = get_cpu_idle_time(cpuid, &cur_wall);
	prev_wall = cur->prev_cpu_wall;
	prev_idle = cur->prev_cpu_idle;
	idle_time_ms = (u32) (cur_idle - prev_idle);
	total_time_ms = (u32) (cur_wall - prev_wall);
	if (idle_time_ms > total_time_ms)
		idle_time_ms = total_time_ms;

	switch (msg) {
	case CLK_STAT_START:
		cur->prev_cpu_wall = cur_wall;
		cur->prev_cpu_idle = cur_idle;
		if (0 == cpuid) {
			memset(&idle_dcstat_info, 0,
			       sizeof(idle_dcstat_info));
			ktime_temp = ktime_get();
			idle_dcstat_info.all_idle_start = ktime_temp;
			idle_dcstat_info.all_idle_end = ktime_temp;
			idle_dcstat_info.all_active_start = ktime_temp;
			idle_dcstat_info.all_active_end = ktime_temp;
			idle_dcstat_info.cal_duration =
			    ktime_to_us(ktime_temp);
			ktime_temp = ktime_set(0, 0);
			idle_dcstat_info.M2_idle_start = ktime_temp;
			idle_dcstat_info.D1P_idle_start = ktime_temp;
			idle_dcstat_info.D1_idle_start = ktime_temp;
			idle_dcstat_info.D2_idle_start = ktime_temp;

			for (j = 0; j < op->cpu_num; j++) {
				cpu_i = op->cpu_id[j];
				dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
				dc_stat_info->power_mode = pxa_powermode(cpu_i);
				dc_stat_info->breakdown_start = ktime_temp;
				for (i = 0; i < MAX_BREAKDOWN_TIME; i++) {
					dc_stat_info->breakdown_time_total[i]
					    = 0;
					dc_stat_info->breakdown_time_count[i]
					    = 0;
				}
				dc_stat_info->C1_idle_start = ktime_temp;
				dc_stat_info->C2_idle_start = ktime_temp;
				for (i = 0; i < MAX_LPM_INDEX_DC; i++) {
					dc_stat_info->C1_op_total[i] = 0;
					dc_stat_info->C1_count[i] = 0;
					dc_stat_info->C2_op_total[i] = 0;
					dc_stat_info->C2_count[i] = 0;
				}
			}
			for (i = 0; i < MAX_LPM_INDEX_DC; i++) {
				idle_dcstat_info.all_idle_op_total[i] = 0;
				idle_dcstat_info.all_idle_count[i] = 0;
			}
		}
		break;
	case CLK_STAT_STOP:
		if (idle_time_ms > total_time_ms)
			cur->busy_time += 0;
		else
			cur->busy_time += (total_time_ms - idle_time_ms);
		cur->idle_time += idle_time_ms;
		if (0 == cpuid) {
			idle_dcstat_info.cal_duration =
			    ktime_to_us(ktime_get())
			    - idle_dcstat_info.cal_duration;
		}
		break;
	case CLK_RATE_CHANGE:
		/* rate change from old->new */
		cur->prev_cpu_idle = cur_idle;
		cur->prev_cpu_wall = cur_wall;
		if (idle_time_ms > total_time_ms)
			cur->busy_time += 0;
		else
			cur->busy_time += (total_time_ms - idle_time_ms);
		cur->idle_time += idle_time_ms;
		tgt = &dc_stat_info->ops_dcstat[tgtop];
		tgt->prev_cpu_idle = cur_idle;
		tgt->prev_cpu_wall = cur_wall;
		ktime_temp = ktime_get();

		for (j = 0; j < op->cpu_num; j++) {
			cpu_i = op->cpu_id[j];
			if (cpuid == cpu_i)
				continue;
			dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
			spin_lock(&c1c2_exit_lock);
			if ((dc_stat_info->idle_flag == LPM_C1) &&
			    ((s64) 0 !=
			     ktime_to_us(dc_stat_info->C1_idle_start))) {
				dc_stat_info->C1_idle_end = ktime_temp;
				dc_stat_info->C1_op_total
				    [dc_stat_info->C1_op_index] +=
				    ktime_to_us(ktime_sub
						(dc_stat_info->C1_idle_end,
						 dc_stat_info->C1_idle_start));
				dc_stat_info->C1_count[dc_stat_info->
						       C1_op_index]++;
				dc_stat_info->C1_idle_start = ktime_temp;
				dc_stat_info->C1_op_index = tgtop;
			} else if ((dc_stat_info->idle_flag == LPM_C2) &&
				   ((s64) 0 !=
				    ktime_to_us(dc_stat_info->C2_idle_start))) {
				dc_stat_info->C2_idle_end = ktime_temp;
				dc_stat_info->C2_op_total
				    [dc_stat_info->C2_op_index] +=
				    ktime_to_us(ktime_sub
						(dc_stat_info->C2_idle_end,
						 dc_stat_info->C2_idle_start));
				dc_stat_info->C2_count[dc_stat_info->
						       C2_op_index]++;
				dc_stat_info->C2_idle_start = ktime_temp;
				dc_stat_info->C2_op_index = tgtop;
			}
			spin_unlock(&c1c2_exit_lock);
		}
		break;
	case CPU_IDLE_ENTER:
		ktime_temp = ktime_get();
		spin_lock(&c1c2_enter_lock);
		if (LPM_C1 == tgtop) {
			dc_stat_info->C1_op_index = dc_stat_info->curopindex;
			dc_stat_info->C1_idle_start = ktime_temp;
			dc_stat_info->idle_flag = LPM_C1;
		} else if (tgtop >= LPM_C2 && tgtop <= LPM_D2_UDR) {
			dc_stat_info->C2_op_index = dc_stat_info->curopindex;
			dc_stat_info->C2_idle_start = ktime_temp;
			dc_stat_info->idle_flag = LPM_C2;
		}
		if ((tgtop >= LPM_C1) && (tgtop <= LPM_D2_UDR))
			dc_stat_info->breakdown_start = ktime_temp;
		spin_unlock(&c1c2_enter_lock);
		dc_stat_info->power_mode = tgtop;
		/*      this mark_keytime is flag indicate enter all idle mode,
		 *      if two core both enter the idle,and power mode isn't
		 *  eaqual to MAX_LPM_INDEX, mean the other core don't
		 *  exit idle.
		 */
		mark_keytime = true;
		for (j = 0; j < op->cpu_num; j++) {
			cpu_i = op->cpu_id[j];
			if (cpuid == cpu_i)
				continue;
			dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
			if (MAX_LPM_INDEX == dc_stat_info->power_mode) {
				mark_keytime = false;
				break;
			}
		}

		if (mark_keytime) {
			idle_dcstat_info.all_idle_start = ktime_temp;
			idle_dcstat_info.all_idle_op_index =
			    dc_stat_info->curopindex;
		}

		/*      this mark_keytime is flag indicate enter all active
		 *      mode,if two core both exit the idle,and power mode
		 *      is both eaqual to MAX_LPM_INDEX, mean the other
		 *      core both exit idle.
		 */

		mark_keytime = true;
		for (j = 0; j < op->cpu_num; j++) {
			cpu_i = op->cpu_id[j];
			if (cpuid == cpu_i)
				continue;
			dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
			if (MAX_LPM_INDEX != dc_stat_info->power_mode) {
				mark_keytime = false;
				break;
			}
		}
		if (mark_keytime) {
			idle_dcstat_info.all_active_end = ktime_temp;
			idle_dcstat_info.total_all_active += ktime_to_us
			    (ktime_sub(idle_dcstat_info.all_active_end,
				       idle_dcstat_info.all_active_start));
			idle_dcstat_info.total_all_active_count++;
		}
		break;
	case CPU_IDLE_EXIT:
		ktime_temp = ktime_get();
		spin_lock(&c1c2_exit_lock);
		if ((dc_stat_info->idle_flag == LPM_C1) &&
		    ((s64) 0 != ktime_to_us(dc_stat_info->C1_idle_start))) {
			dc_stat_info->C1_idle_end = ktime_temp;
			dc_stat_info->C1_op_total[dc_stat_info->C1_op_index] +=
			    ktime_to_us(ktime_sub(dc_stat_info->C1_idle_end,
						  dc_stat_info->C1_idle_start));
			dc_stat_info->C1_count[dc_stat_info->C1_op_index]++;
			dc_stat_info->C1_idle_start = ktime_set(0, 0);
		} else if ((dc_stat_info->idle_flag == LPM_C2) &&
			   ((s64) 0 !=
			    ktime_to_us(dc_stat_info->C2_idle_start))) {
			dc_stat_info->C2_idle_end = ktime_temp;
			dc_stat_info->C2_op_total[dc_stat_info->C2_op_index] +=
			    ktime_to_us(ktime_sub(dc_stat_info->C2_idle_end,
						  dc_stat_info->C2_idle_start));
			dc_stat_info->C2_count[dc_stat_info->C2_op_index]++;
			dc_stat_info->C2_idle_start = ktime_set(0, 0);
		}
		spin_unlock(&c1c2_exit_lock);
		dc_stat_info->idle_flag = MAX_LPM_INDEX;
		if ((s64) 0 != ktime_to_us(dc_stat_info->breakdown_start)) {
			dc_stat_info->breakdown_end = ktime_temp;
			temp_time = ktime_to_us(ktime_sub
						(dc_stat_info->breakdown_end,
						 dc_stat_info->
						 breakdown_start));
			if (temp_time) {
				if (temp_time >= 100 * 1000) {
					dc_stat_info->breakdown_time_count
					    [MAX_BREAKDOWN_TIME - 1]++;
					dc_stat_info->breakdown_time_total
					    [MAX_BREAKDOWN_TIME - 1] +=
					    temp_time;
				} else {
					i = (temp_time / (10 * 1000));
					dc_stat_info->breakdown_time_count[i]++;
					dc_stat_info->breakdown_time_total[i]
					    += temp_time;
				}
			}
		}

		dc_stat_info->power_mode = tgtop;
		mark_keytime = true;

		for (j = 0; j < op->cpu_num; j++) {
			cpu_i = op->cpu_id[j];
			if (cpuid == cpu_i)
				continue;
			dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
			if (MAX_LPM_INDEX == dc_stat_info->power_mode) {
				mark_keytime = false;
				break;
			}
		}
		spin_lock(&allidle_lock);
		if (mark_keytime) {
			idle_dcstat_info.all_idle_end = ktime_temp;
			idle_dcstat_info.total_all_idle += ktime_to_us
			    (ktime_sub(idle_dcstat_info.all_idle_end,
				       idle_dcstat_info.all_idle_start));
			idle_dcstat_info.total_all_idle_count++;

			if ((s64) 0 != ktime_to_us
			    (idle_dcstat_info.all_idle_start)) {
				idle_dcstat_info.all_idle_op_total
				    [idle_dcstat_info.all_idle_op_index] +=
				    ktime_to_us(ktime_sub
						(idle_dcstat_info.
						 all_idle_end,
						 idle_dcstat_info.
						 all_idle_start));
				idle_dcstat_info.
				    all_idle_count[idle_dcstat_info.
						   all_idle_op_index]++;
			}

			if ((s64) 0 != ktime_to_us
			    (idle_dcstat_info.M2_idle_start)) {
				idle_dcstat_info.M2_idle_total +=
				    ktime_to_us(ktime_sub
						(idle_dcstat_info.
						 all_idle_end,
						 idle_dcstat_info.
						 M2_idle_start));
				idle_dcstat_info.M2_count++;
			} else if ((s64) 0 != ktime_to_us
				   (idle_dcstat_info.D1P_idle_start)) {
				idle_dcstat_info.D1P_idle_total +=
				    ktime_to_us(ktime_sub
						(idle_dcstat_info.
						 all_idle_end,
						 idle_dcstat_info.
						 D1P_idle_start));
				idle_dcstat_info.D1p_count++;
			} else if ((s64) 0 != ktime_to_us
				   (idle_dcstat_info.D1_idle_start)) {
				idle_dcstat_info.D1_idle_total +=
				    ktime_to_us(ktime_sub
						(idle_dcstat_info.
						 all_idle_end,
						 idle_dcstat_info.
						 D1_idle_start));
				idle_dcstat_info.D1_count++;
			} else if ((s64) 0 != ktime_to_us
				   (idle_dcstat_info.D2_idle_start)) {
				idle_dcstat_info.D2_idle_total +=
				    ktime_to_us(ktime_sub
						(idle_dcstat_info.
						 all_idle_end,
						 idle_dcstat_info.
						 D2_idle_start));
				idle_dcstat_info.D2_count++;
			}
			ktime_temp1 = ktime_set(0, 0);
			idle_dcstat_info.M2_idle_start = ktime_temp1;
			idle_dcstat_info.D1P_idle_start = ktime_temp1;
			idle_dcstat_info.D1_idle_start = ktime_temp1;
			idle_dcstat_info.D2_idle_start = ktime_temp1;
		}
		spin_unlock(&allidle_lock);
		mark_keytime = true;
		for (j = 0; j < op->cpu_num; j++) {
			cpu_i = op->cpu_id[j];
			if (cpuid == cpu_i)
				continue;
			dc_stat_info = &per_cpu(cpu_dc_stat, cpu_i);
			if (MAX_LPM_INDEX != dc_stat_info->power_mode) {
				mark_keytime = false;
				break;
			}
		}
		if (mark_keytime)
			idle_dcstat_info.all_active_start = ktime_temp;
		break;
	case CPU_M2_OR_DEEPER_ENTER:
		ktime_temp = ktime_get();
		if (LPM_C2 == tgtop)
			idle_dcstat_info.M2_idle_start = ktime_temp;
		else if (LPM_D1P == tgtop)
			idle_dcstat_info.D1P_idle_start = ktime_temp;
		else if (LPM_D1 == tgtop)
			idle_dcstat_info.D1_idle_start = ktime_temp;
		else if (LPM_D2 == tgtop)
			idle_dcstat_info.D2_idle_start = ktime_temp;
		break;
	default:
		break;
	}
}

static ssize_t cpu_dc_read(struct file *filp,
			       char __user *buffer, size_t count,
			       loff_t *ppos)
{
	char *buf;
	int len = 0;
	ssize_t ret, size = 2 * PAGE_SIZE - 1;
	unsigned int cpu, i, dc_int = 0, dc_fra = 0;
	struct clk_dc_stat_info *percpu_stat = NULL;
	u64 total_time, run_total, idle_total, busy_time;
	u64 av_mips;
	u32 av_mips_l, av_mips_h;
	u64 temp_total_time = 0, temp_total_count = 0;
	char *lpm_time_string[12] = { "<10 ms", "<20 ms", "<30 ms",
		"<40 ms", "<50 ms", "<60 ms", "<70 ms", "<80 ms",
		"<90 ms", "<100 ms", ">100 ms"
	};

	buf = (char *)__get_free_pages(GFP_NOIO, get_order(size));
	if (!buf)
		return -ENOMEM;

	percpu_stat = &per_cpu(cpu_dc_stat, 0);
	if (percpu_stat->stat_start) {
		len += snprintf(buf + len, size - len,
				"Please stop the cpu duty cycle stats at first\n");
		goto out;
	}

	for_each_possible_cpu(cpu) {
		percpu_stat = &per_cpu(cpu_dc_stat, cpu);
		av_mips = run_total = idle_total = 0;
		for (i = 0; i < percpu_stat->ops_stat_size; i++) {
			idle_total += percpu_stat->ops_dcstat[i].idle_time;
			run_total += percpu_stat->ops_dcstat[i].busy_time;
			av_mips += (u64) (percpu_stat->ops_dcstat[i].pprate *
					  percpu_stat->ops_dcstat[i].busy_time);
		}
		total_time = idle_total + run_total;
		if (!total_time) {
			len += snprintf(buf + len, size - len,
					"No stat information! ");
			len += snprintf(buf + len, size - len,
					"Help information :\n");
			len += snprintf(buf + len, size - len,
					"1. echo 1 to start duty cycle stat:\n");
			len += snprintf(buf + len, size - len,
					"2. echo 0 to stop duty cycle stat:\n");
			len += snprintf(buf + len, size - len,
					"3. cat to check duty cycle info from start to stop:\n\n");
			goto out;
		}
		av_mips_l = 0;
		av_mips_h = div_u64_rem(av_mips, total_time, &av_mips_l);
		av_mips_l = div_u64(av_mips_l * 100, total_time);
		dc_int = calculate_dc(run_total, total_time, &dc_fra);
		len += snprintf(buf + len, size - len,
				"\n| CPU %u | %10s %lldms| %10s %lldms|"
				"%10s %2u.%2u%%|%10s %u.%02uMHz |\n", cpu,
				"idle time", idle_total, "total time",
				total_time, "duty cycle", dc_int, dc_fra,
				"average mips", av_mips_h, av_mips_l);
		len +=
		    snprintf(buf + len, size - len,
			     "| %3s | %5s | %8s | %8s | %8s | %8s |"
			     " %8s | %8s | %8s | %8s | %8s |\n", "OP#", "rate",
			     "run time", "idle time", "rt ratio", "All idle",
			     "Aidle count", "C1 ratio", "C1 count", "C2 ratio",
			     "C2 count");
		for (i = 0; i < percpu_stat->ops_stat_size; i++) {
			if (total_time) {
				busy_time =
				    percpu_stat->ops_dcstat[i].busy_time;
				dc_int = calculate_dc(busy_time, total_time,
							  &dc_fra);
			}
			len += snprintf(buf + len, size - len,
					"| %3u | %5lu | %8lu | %9lu | %4u.%2u%% |"
					" %7lld%% | %11lld | %7lld%% | %8lld |"
					" %7lld%% | %8lld |\n",
					percpu_stat->ops_dcstat[i].ppindex,
					percpu_stat->ops_dcstat[i].pprate,
					percpu_stat->ops_dcstat[i].busy_time,
					percpu_stat->ops_dcstat[i].idle_time,
					dc_int, dc_fra,
					div64_u64
					(idle_dcstat_info.
					 all_idle_op_total[i] * (u64) (100),
					 idle_dcstat_info.cal_duration),
					idle_dcstat_info.all_idle_count[i],
					div64_u64(percpu_stat->C1_op_total[i] *
						  (u64) (100),
						  idle_dcstat_info.
						  cal_duration),
					percpu_stat->C1_count[i],
					div64_u64(percpu_stat->C2_op_total[i] *
						  (u64) (100),
						  idle_dcstat_info.
						  cal_duration),
					percpu_stat->C2_count[i]
			    );
		}
	}

	len += snprintf(buf + len, size - len,
		     "| %10s | %15s | %15s | %15s |\n",
		     "state", "ratio", "time(ms)", "count");
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% |"
		     " %15lld | %15lld | === > All core active\n", "All active",
		     div64_u64(idle_dcstat_info.total_all_active *
			       (u64) (100), idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.total_all_active,
			       (u64) 1000),
		     idle_dcstat_info.total_all_active_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% |"
		     " %15lld | %15lld | === > All core idle\n", "All idle",
		     div64_u64(idle_dcstat_info.total_all_idle *
			       (u64) (100), idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.total_all_idle, (u64) 1000),
		     idle_dcstat_info.total_all_idle_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% | %15lld |" " %15lld |\n", "M2",
		     div64_u64(idle_dcstat_info.M2_idle_total * (u64) (100),
			       idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.M2_idle_total, (u64) 1000),
		     idle_dcstat_info.M2_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% | %15lld |" " %15lld |\n", "D1P",
		     div64_u64(idle_dcstat_info.D1P_idle_total *
			       (u64) (100), idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.D1P_idle_total, (u64) 1000),
		     idle_dcstat_info.D1p_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% | %15lld |" " %15lld |\n", "D1",
		     div64_u64(idle_dcstat_info.D1_idle_total * (u64) (100),
			       idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.D1_idle_total, (u64) 1000),
		     idle_dcstat_info.D1_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% | %15lld |" " %15lld |\n", "D2",
		     div64_u64(idle_dcstat_info.D2_idle_total * (u64) (100),
			       idle_dcstat_info.cal_duration),
		     div64_u64(idle_dcstat_info.D2_idle_total, (u64) 1000),
		     idle_dcstat_info.D2_count);
	len +=
	    snprintf(buf + len, size - len,
		     "| %10s | %14lld%% | %15lld |" " === > Total test time\n",
		     "All time", (u64) 100,
		     div64_u64(idle_dcstat_info.cal_duration, (u64) 1000));

	for_each_possible_cpu(cpu) {
		percpu_stat = &per_cpu(cpu_dc_stat, cpu);
		len += snprintf(buf + len, size - len,
				"|  cpu%d idle | %15s |"
				" %15s |\n", cpu, "all time(ms)", "count");
		temp_total_time = temp_total_count = 0;
		for (i = 0; i < MAX_BREAKDOWN_TIME; i++) {
			if (0 != percpu_stat->breakdown_time_total[i] ||
			    0 != percpu_stat->breakdown_time_count[i]) {
				len += snprintf(buf + len, size - len,
						"| %10s | %15lld | %15lld |\n",
						lpm_time_string[i],
						div64_u64(percpu_stat->
							  breakdown_time_total
							  [i], (u64) 1000),
						percpu_stat->
						breakdown_time_count[i]);
				temp_total_time +=
				    div64_u64(percpu_stat->
					      breakdown_time_total[i],
					      (u64) 1000);
				temp_total_count +=
				    percpu_stat->breakdown_time_count[i];
			}
		}
		len += snprintf(buf + len, size - len,
				"| %10s | %15lld | %15lld |"
				" === > Total 10~100ms time\n", "All time",
				temp_total_time, temp_total_count);
	}
out:
	if (len == size)
		pr_warn("%s The dump buf is not large enough!\n", __func__);

	ret = simple_read_from_buffer(buffer, count, ppos, buf, len);
	free_pages((unsigned long)buf, get_order(size));
	return ret;
}

static ssize_t cpu_dc_write(struct file *filp,
				const char __user *buffer, size_t count,
				loff_t *ppos)
{
	unsigned int start, cpu, i;
	char buf[10] = { 0 };
	struct clk_dc_stat_info *percpu_stat = NULL;
	struct clk *cpu_clk;
	int j;
	struct core_dcstat *op;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%d", &start);

	start = !!start;
	percpu_stat = &per_cpu(cpu_dc_stat, 0);
	if (start == percpu_stat->stat_start) {
		pr_err("[WARNING]CPU stat is already %s\n",
		       percpu_stat->stat_start ? "started" : "stopped");
		return -EINVAL;
	}

	list_for_each_entry(op, &core_dcstat_list, node) {
		cpu_clk = op->clk;
		/*
		 * hold the same lock of clk_enable, disable, set_rate ops
		 * here to avoid the status change when start/stop and lead
		 * to incorrect stat info
		 */
		spin_lock(&clk_dc_lock);
		if (start) {
			/* clear old stat information */
			for (j = 0; j < op->cpu_num; j++) {
				cpu = op->cpu_id[j];
				percpu_stat = &per_cpu(cpu_dc_stat, cpu);
				for (i = 0; i < percpu_stat->ops_stat_size; i++) {
					percpu_stat->ops_dcstat[i].idle_time
						= 0;
					percpu_stat->ops_dcstat[i].busy_time
						= 0;
				}
				percpu_stat->stat_start = true;
				cpu_dcstat_event(cpu_clk, cpu,
					CLK_STAT_START, 0);
			}
		} else {
			for (j = 0; j < op->cpu_num; j++) {
				cpu = op->cpu_id[j];
				percpu_stat = &per_cpu(cpu_dc_stat, cpu);
				cpu_dcstat_event(cpu_clk, cpu,
					CLK_STAT_STOP, 0);
				percpu_stat->stat_start = false;
			}
		}
		spin_unlock(&clk_dc_lock);
	}
	return count;
}

static const struct file_operations cpu_dc_ops = {
	.owner = THIS_MODULE,
	.read = cpu_dc_read,
	.write = cpu_dc_write,
};

struct dentry *cpu_dcstat_file_create(const char *file_name,
				struct dentry *parent)
{
	struct dentry *cpu_dc_stat;
	cpu_dc_stat = debugfs_create_file(file_name, 0664, parent,
					NULL, &cpu_dc_ops);
	if (!cpu_dc_stat) {
		pr_err("debugfs entry created failed in %s\n", __func__);
		return 0;
	}

	return cpu_dc_stat;
}
EXPORT_SYMBOL(cpu_dcstat_file_create);

struct dentry *clk_dcstat_file_create(const char *file_name,
	struct dentry *parent,
	const struct file_operations *dc_ops)
{
	struct dentry *dc_stat;
	dc_stat = debugfs_create_file(file_name, 0664, parent,
					NULL, dc_ops);
	if (!dc_stat) {
		pr_err("debugfs entry created failed in %s\n", __func__);
		return 0;
	}

	return dc_stat;
}
EXPORT_SYMBOL(clk_dcstat_file_create);
