/*
 * linux/drivers/devfreq/mck4_memorybus.c
 *
 *  Copyright (C) 2012 Marvell International Ltd.
 *  All rights reserved.
 *
 *  2012-03-16	Yifan Zhang <zhangyf@marvell.com>
 *		Zhoujie Wu<zjwu@marvell.com>
 *		Qiming Wu<wuqm@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/platform_data/devfreq-pxa.h>

#include <trace/events/pxa.h>

#include "mck_memorybus.h"

#define DDR_DEVFREQ_UPTHRESHOLD 65
#define DDR_DEVFREQ_DOWNDIFFERENTIAL 5

#define KHZ_TO_HZ   1000

static struct ddr_devfreq_data *cur_data;
#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
/* notifier to change the devfreq govoner's upthreshold */
static int upthreshold_freq_notifer_call(struct notifier_block *nb,
	       unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct devfreq *devfreq = cur_data->devfreq;
	struct devfreq_throughput_data *gov_data;
	int i;

	if (val != CPUFREQ_POSTCHANGE)
		return NOTIFY_OK;

	mutex_lock(&devfreq->lock);
	gov_data = devfreq->data;
	if (freq->new >= cur_data->high_upthrd_swp) {
		gov_data->upthreshold = cur_data->high_upthrd;
		for (i = 0; i < gov_data->table_len; i++) {
			gov_data->throughput_table[i].up =
				gov_data->upthreshold * gov_data->freq_table[i] / 100;
			gov_data->throughput_table[i].down =
				(gov_data->upthreshold - gov_data->downdifferential) * gov_data->freq_table[i] / 100;
		}
	} else {
		gov_data->upthreshold = DDR_DEVFREQ_UPTHRESHOLD;
		for (i = 0; i < gov_data->table_len; i++) {
			gov_data->throughput_table[i].up =
				gov_data->upthreshold * gov_data->freq_table[i] / 100;
			gov_data->throughput_table[i].down =
				(gov_data->upthreshold - gov_data->downdifferential) * gov_data->freq_table[i] / 100;
		}
	}
	mutex_unlock(&devfreq->lock);

	trace_pxa_ddr_upthreshold(gov_data->upthreshold);

	return NOTIFY_OK;
}

static struct notifier_block upthreshold_freq_notifier = {
	.notifier_call = upthreshold_freq_notifer_call
};
#endif

#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
/* default using 65% as upthreshold and 5% as downdifferential */
static struct devfreq_throughput_data devfreq_throughput_data = {
	.upthreshold = DDR_DEVFREQ_UPTHRESHOLD,
	.downdifferential = DDR_DEVFREQ_DOWNDIFFERENTIAL,
};
#endif /* CONFIG_DDR_DEVFREQ_GOV_THROUGHPUT */

static void write_static_register(unsigned int val, unsigned int expected_val,
				  void *reg, unsigned int ver)
{
	int max_try = 1000;
	unsigned int temp;

	if (ver == MCK4) {
		while (max_try-- > 0) {
			writel(val, reg);
			temp = readl(reg);
			if (expected_val == temp)
				break;
		}
		if (!max_try)
			pr_err("Can't write register %p with value %X\n", reg, val);
	} else if (ver == MCK5)
		writel(val, reg);
	else
		/* this should never happen */
		BUG_ON(1);
}

static void config_ddr_performance_counter(struct ddr_devfreq_data *data)
{
	unsigned int i;
	void __iomem *mc_base = data->dmc.hw_base;
	struct mck_pmu_regs_offset *regs = &data->dmc.mck_regs;
	unsigned int ver = data->dmc.version;

	/*
	 * Step1: Write to Performance Counter Configuration Register to
	 * disable interrupts and counters.
	 */
	write_static_register(0x0, 0x0, mc_base + regs->cfg, ver);

	/*
	 * Step2: Write to Performance Counter Control Register to select
	 * the desired settings.
	 * bit18:16 0x0 = Divide clock by 1
	 * bit4     0x1 = Continue counting on any counter overflow
	 * bit0     0x0 = Enabled counters begin counting
	 */
	write_static_register(0x10, 0x10, mc_base + regs->ctrl, ver);

	/*
	 * Step3: Write to Performance Counter Register to set the starting
	 * value.
	 */
	for (i = 0; i < data->dmc.pmucnt_in_use; i++) {
		write_static_register(0x0, 0x0,
			mc_base + regs->cnt_base + i * 4, ver);
	}

	/* clear overflow flag */
	write_static_register(0xf, 0x0, mc_base + regs->cnt_stat, ver);

	/* reset old data */
	memset(data->dmc.ddr_perf_cnt_old, 0,
		sizeof(unsigned int) * data->dmc.pmucnt_in_use);
}

static void stop_ddr_performance_counter(struct ddr_devfreq_data *data)
{
	void __iomem *mc_base = data->dmc.hw_base;
	struct mck_pmu_regs_offset *regs = &data->dmc.mck_regs;
	unsigned int ver = data->dmc.version;

	/*
	 * Write to Performance Counter Configuration Register to
	 * disable interrupts and counters.
	 */
	write_static_register(0x0, 0x0, mc_base + regs->cfg, ver);
}

static void start_ddr_performance_counter(struct ddr_devfreq_data *data)
{
	void __iomem *mc_base = data->dmc.hw_base;
	struct mck_pmu_regs_offset *regs = &data->dmc.mck_regs;
	unsigned int ver = data->dmc.version;
	unsigned int val;

	/*
	 * Write to Performance Counter Configuration Register to
	 * enable counters and choose the events for counters.
	 */
	switch (ver) {
	case MCK4:
		/*
		 * cnt0, event=0x00, clock cycles
		 * cnt1, event=0x01, DPC idle cycles
		 * cnt2, event=0x14, Read + Write command count
		 * cnt3, event=0x04, no bus utilization when not idle
		 */
		val = ((0x80 | 0x00) <<  0) | ((0x80 | 0x01) <<  8) |
		      ((0x80 | 0x14) << 16) | ((0x80 | 0x04) << 24);
		break;
	case MCK5:
		/*
		 * cnt0, event=0x00, clock cycles
		 * cnt1, event=0x18, busy cycles
		 * cnt2, event=0x1A, Read + Write command count
		 * cnt3, event=0x28, no bus utilization when not idle
		 */
		val = ((0x80 | 0x00) <<  0) | ((0x80 | 0x18) <<  8) |
		      ((0x80 | 0x1A) << 16) | ((0x80 | 0x28) << 24);
		break;
	default:
		/* this should never happen */
		BUG_ON(1);
	}

	write_static_register(val, val, mc_base + regs->cfg, ver);
}

static int ddr_rate2_index(struct ddr_devfreq_data *data)
{
	unsigned int rate;
	int i;

	rate = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
	for (i = 0; i < data->ddr_freq_tbl_len; i++)
		if (data->ddr_freq_tbl[i] == rate)
			return i;
	dev_err(&data->devfreq->dev, "unknow ddr rate %d\n", rate);
	return -1;
}

static unsigned int ddr_index2_rate(struct ddr_devfreq_data *data, int index)
{
	if ((index >= 0) && (index < data->ddr_freq_tbl_len))
		return data->ddr_freq_tbl[index];
	else {
		dev_err(&data->devfreq->dev,
			"unknow ddr index %d\n", index);
		return 0;
	}
}

static void update_ddr_performance_data(struct ddr_devfreq_data *data)
{
	struct perf_counters *ddr_ticks = data->dmc.ddr_ticks;
	void *mc_base = (void *)data->dmc.hw_base;
	struct mck_pmu_regs_offset *regs = &data->dmc.mck_regs;
	unsigned int cnt, i, overflow_flag;
	int ddr_idx;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);

	/* stop counters, to keep data synchronized */
	stop_ddr_performance_counter(data);

	overflow_flag = readl(mc_base + regs->cnt_stat) & 0xf;

	ddr_idx = ddr_rate2_index(data);

	if ((ddr_idx >= 0) && (ddr_idx < data->ddr_freq_tbl_len)) {
		for (i = 0; i < data->dmc.pmucnt_in_use; i++) {
			cnt = readl(mc_base + regs->cnt_base + i * 4);

			if (overflow_flag & (1 << i)) {
				dev_dbg(&data->devfreq->dev,
					"DDR perf counter overflow!\n");
				ddr_ticks[ddr_idx].reg[i] +=
					0x100000000LLU + cnt -
					data->dmc.ddr_perf_cnt_old[i];
			} else
				ddr_ticks[ddr_idx].reg[i] += cnt -
					data->dmc.ddr_perf_cnt_old[i];

			data->dmc.ddr_perf_cnt_old[i] = cnt;
		}
	} else
		dev_err(&data->devfreq->dev, "%s: invalid ddr_idx %u\n",
			__func__, ddr_idx);

	spin_unlock_irqrestore(&data->lock, flags);
}

static int __init ddr_perf_counter_init(struct ddr_devfreq_data *data)
{
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&data->lock, flags);
	for (i = 0; i < data->ddr_freq_tbl_len; i++)
		memset(data->dmc.ddr_ticks[i].reg, 0,
			sizeof(u64) * data->dmc.pmucnt_in_use);
	config_ddr_performance_counter(data);
	start_ddr_performance_counter(data);
	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static inline void reset_ddr_counters(struct ddr_devfreq_data *data)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	config_ddr_performance_counter(data);
	start_ddr_performance_counter(data);
	spin_unlock_irqrestore(&data->lock, flags);
}

/* calculate ddr workload according to busy and total time, unit percent */
static inline unsigned int cal_workload(unsigned long busy_time,
	unsigned long total_time)
{
	u64 tmp0, tmp1;

	if (!total_time || !busy_time)
		return 0;
	tmp0 = busy_time * 100;
	tmp1 = div_u64(tmp0, total_time);
	return (unsigned int)tmp1;
}

/*
 * get the mck4 total_ticks, data_ticks, speed.
 */
static void get_ddr_cycles(struct ddr_devfreq_data *data,
	unsigned long *total_ticks, unsigned long *data_ticks, int *speed)
{
	void __iomem *mc_base;
	unsigned long flags;
	unsigned int diff_ms;
	unsigned long long time_stamp_cur;
	static unsigned long long time_stamp_old;
	struct mck_pmu_regs_offset *regs = &data->dmc.mck_regs;

	mc_base = data->dmc.hw_base;

	spin_lock_irqsave(&data->lock, flags);
	/* stop counters, to keep data synchronized */
	stop_ddr_performance_counter(data);
	*total_ticks = readl(mc_base + regs->cnt_base);
	*data_ticks = readl(mc_base + regs->cnt_base + 4 * 2)
		* data->bst_len / 2;
	start_ddr_performance_counter(data);
	spin_unlock_irqrestore(&data->lock, flags);

	time_stamp_cur = sched_clock();
	diff_ms = (unsigned int)div_u64(time_stamp_cur - time_stamp_old,
			1000000);
	time_stamp_old = time_stamp_cur;

	if (diff_ms != 0)
		*speed = *data_ticks / diff_ms;
	else
		*speed = -1;
}

static int ddr_get_dev_status(struct device *dev,
			       struct devfreq_dev_status *stat)
{
	struct ddr_devfreq_data *data = dev_get_drvdata(dev);
	struct devfreq *df = data->devfreq;
	unsigned int workload;
	unsigned long polling_jiffies;
	unsigned long now = jiffies;

	stat->current_frequency = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
	/*
	 * ignore the profiling if it is not from devfreq_monitor
	 * or there is no profiling
	 */
	polling_jiffies = msecs_to_jiffies(df->profile->polling_ms);
	if (!polling_jiffies ||
		(polling_jiffies && data->last_polled_at &&
		time_before(now, (data->last_polled_at + polling_jiffies)))) {
		dev_dbg(dev,
			"No profiling or interval is not expired %lu, %lu, %lu\n",
			polling_jiffies, now, data->last_polled_at);
		return -EINVAL;
	}

	get_ddr_cycles(data, &stat->total_time,
			&stat->busy_time, &stat->throughput);
	if (data->is_ddr_stats_working)
		update_ddr_performance_data(data);
	reset_ddr_counters(data);
	data->last_polled_at = now;

	/* Ajust the workload calculation here to align with devfreq governor */
	if (stat->busy_time >= (1 << 24) || stat->total_time >= (1 << 24)) {
		stat->busy_time >>= 7;
		stat->total_time >>= 7;
	}

	workload = cal_workload(stat->busy_time, stat->total_time);

	dev_dbg(dev, "workload is %d precent\n", workload);
	dev_dbg(dev, "busy time is 0x%x, %u\n", (unsigned int)stat->busy_time,
		 (unsigned int)stat->busy_time);
	dev_dbg(dev, "total time is 0x%x, %u\n\n",
		(unsigned int)stat->total_time,
		(unsigned int)stat->total_time);
	dev_dbg(dev, "throughput is 0x%x, throughput * 8 (speed) is %u\n\n",
		(unsigned int)stat->throughput, 8 * stat->throughput);

	trace_pxa_ddr_workload(workload, stat->current_frequency,
				stat->throughput);
	return 0;
}

static int ddr_set_rate(struct ddr_devfreq_data *data, unsigned long tgt_rate)
{
	unsigned long cur_freq, tgt_freq;

	cur_freq = clk_get_rate(data->ddr_clk);
	tgt_freq = tgt_rate * KHZ_TO_HZ;

	if (cur_freq == tgt_freq)
		return 0;

	dev_dbg(&data->devfreq->dev, "%s: curfreq %lu, tgtfreq %lu\n",
		__func__, cur_freq, tgt_freq);

	/* update performance data before ddr clock change */
	if (data->is_ddr_stats_working)
		update_ddr_performance_data(data);

	/* clk_set_rate will find a frequency larger or equal tgt_freq */
	clk_set_rate(data->ddr_clk, tgt_freq);

	/* re-init ddr performance counters after ddr clock change */
	if (data->is_ddr_stats_working)
		reset_ddr_counters(data);

	return 0;
}

static void find_best_freq(struct ddr_devfreq_data *data, unsigned long *freq,
			   u32 flags)
{
	int i;
	unsigned long temp = *freq;

	u32 *freq_table = data->ddr_freq_tbl;
	u32 len = data->ddr_freq_tbl_len;

	if (*freq < freq_table[0]) {
		*freq = freq_table[0];
		return;
	}
	if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND) {
		for (i = 1; i < len; i++)
			if (freq_table[i - 1] <= temp
			    && freq_table[i] > temp) {
				*freq = freq_table[i - 1];
				break;
			}
	} else {
		for (i = 0; freq_table[i]; i++)
			if (freq_table[i] >= temp) {
				*freq = freq_table[i];
				break;
			}
	}

	if (i == len)
		*freq = freq_table[i - 1];
}

static int ddr_target(struct device *dev, unsigned long *freq,
		unsigned int flags)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *df;
	unsigned int *ddr_freq_table, ddr_freq_len;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	/* in normal case ddr fc will NOT be disabled */
	if (unlikely(atomic_read(&data->is_disabled))) {
		df = data->devfreq;
		/*
		 * this function is called with df->locked, it is safe to
		 * read the polling_ms here
		 */
		if (df->profile->polling_ms)
			dev_err(dev, "[WARN] ddr ll fc is disabled from "
				"debug interface, suggest to disable "
				"the profiling at first!\n");
		*freq = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
		return 0;
	}

	ddr_freq_table = &data->ddr_freq_tbl[0];
	ddr_freq_len = data->ddr_freq_tbl_len;
	dev_dbg(dev, "%s: %u\n", __func__, (unsigned int)*freq);

	find_best_freq(data, freq, flags);
	ddr_set_rate(data, *freq);

	*freq = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;
	return 0;
}

static int configure_mck_pmu_regs(struct ddr_devfreq_data *data)
{
	unsigned int ver = data->dmc.version;

	switch (ver) {
	case MCK4:
		data->dmc.mck_regs.cfg = MCK4_PERF_CONFIG;
		data->dmc.mck_regs.cnt_stat = MCK4_PERF_STATUS;
		data->dmc.mck_regs.ctrl = MCK4_PERF_CONTRL;
		data->dmc.mck_regs.cnt_base = MCK4_PERF_CNT_BASE;
		data->dmc.mck_regs.intr_stat = MCK4_INTR_STATUS;
		data->dmc.mck_regs.intr_en = MCK4_INTR_EN;
		return 0;
	case MCK5:
		data->dmc.mck_regs.cfg = MCK5_PERF_CONFIG;
		data->dmc.mck_regs.cnt_stat = MCK5_PERF_STATUS;
		data->dmc.mck_regs.ctrl = MCK5_PERF_CONTRL;
		data->dmc.mck_regs.cnt_base = MCK5_PERF_CNT_BASE;
		data->dmc.mck_regs.intr_stat = MCK5_INTR_STATUS;
		data->dmc.mck_regs.intr_en = MCK5_INTR_EN;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ddr_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	*freq = clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;

	return 0;
}

static struct devfreq_dev_profile ddr_devfreq_profile = {
	/* Profiler is not enabled by default */
	.polling_ms = 0,
	.target = ddr_target,
	.get_dev_status = ddr_get_dev_status,
	.get_cur_freq = ddr_get_cur_freq,
};

/* interface to change the switch point of high aggresive upthreshold */
static ssize_t high_swp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *devfreq;
	unsigned int swp;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	devfreq = data->devfreq;

	if (0x1 != sscanf(buf, "%u", &swp)) {
		dev_err(dev, "<ERR> wrong parameter\n");
		return -E2BIG;
	}

	mutex_lock(&devfreq->lock);
	data->high_upthrd_swp = swp;
	mutex_unlock(&devfreq->lock);

	return size;
}

static ssize_t high_swp_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "%u\n", data->high_upthrd_swp);
}

/* interface to change the aggresive upthreshold value */
static ssize_t high_upthrd_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct devfreq *devfreq;
	unsigned int high_upthrd;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	devfreq = data->devfreq;

	if (0x1 != sscanf(buf, "%u", &high_upthrd)) {
		dev_err(dev, "<ERR> wrong parameter\n");
		return -E2BIG;
	}

	mutex_lock(&devfreq->lock);
	data->high_upthrd = high_upthrd;
	mutex_unlock(&devfreq->lock);

	return size;
}

static ssize_t high_upthrd_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "%u\n", data->high_upthrd);
}

/* debug interface used to totally disable ddr fc */
static ssize_t disable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int is_disabled;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	if (0x1 != sscanf(buf, "%d", &is_disabled)) {
		dev_err(dev, "<ERR> wrong parameter\n");
		return -E2BIG;
	}

	is_disabled = !!is_disabled;
	if (is_disabled == atomic_read(&data->is_disabled)) {
		dev_info(dev, "[WARNING] ddr fc is already %s\n",
			atomic_read(&data->is_disabled) ?
			"disabled" : "enabled");
		return size;
	}

	if (is_disabled)
		atomic_inc(&data->is_disabled);
	else
		atomic_dec(&data->is_disabled);

	dev_info(dev, "[WARNING]ddr fc is %s from debug interface!\n",
		atomic_read(&data->is_disabled) ? "disabled" : "enabled");
	return size;
}

static ssize_t disable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "ddr fc is_disabled = %d\n",
		 atomic_read(&data->is_disabled));
}

/*
 * Debug interface used to change ddr rate.
 * It will ignore all devfreq and Qos requests.
 * Use interface disable_ddr_fc prior to it.
 */
static ssize_t ddr_freq_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int freq;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	if (!atomic_read(&data->is_disabled)) {
		dev_err(dev, "<ERR> It will change ddr rate,"
			"disable ddr fc at first\n");
		return -EPERM;
	}

	if (0x1 != sscanf(buf, "%d", &freq)) {
		dev_err(dev, "<ERR> wrong parameter, "
			"echo freq > ddr_freq to set ddr rate(unit Khz)\n");
		return -E2BIG;
	}
	clk_set_rate(data->ddr_clk, freq * KHZ_TO_HZ);

	dev_dbg(dev, "ddr freq read back: %lu\n",
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);

	return size;
}

static ssize_t ddr_freq_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	return sprintf(buf, "current ddr freq is: %lu\n",
		 clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);
}

/* used to collect ddr cnt during 20ms */
static ssize_t dp_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	struct perf_counters *ddr_ticks;
	int len = 0;
	unsigned long flags;
	unsigned int ver;

	int i, j, k;
	unsigned int glob_ratio;
	unsigned int idle_ratio, busy_ratio, data_ratio, util_ratio;
	unsigned int tmp_total, tmp_rw_cmd;
	unsigned int tmp_dpc_idle, tmp_no_util, tmp_busy;
	unsigned int tmp_data_cycle;
	u64 glob_ticks;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	ddr_ticks = data->dmc.ddr_ticks;
	ver = data->dmc.version;
	idle_ratio = busy_ratio = data_ratio = util_ratio = 0;

	/* ddr ticks show */
	len += sprintf(buf + len, "\nddr_ticks operating point list:\n");

	len += sprintf(buf + len, "idx|dmcfs|  total_ticks   |"
			" DPC_idle_ticks |   R+W_cmd_cnt  |no_util_not_idle\n");

	len += sprintf(buf + len, "----------------------------------"
			"-------------------------------------------\n");

	spin_lock_irqsave(&data->lock, flags);

	for (i = 0; i < data->ddr_freq_tbl_len; i++) {
		len += sprintf(buf + len,
			"%3d|%5u|%16llu|%16llu|%16llu|%16llu\n",
			i, ddr_index2_rate(data, i)/1000,
			ddr_ticks[i].reg[0],
			ddr_ticks[i].reg[1],
			ddr_ticks[i].reg[2],
			ddr_ticks[i].reg[3]);
	}
	spin_unlock_irqrestore(&data->lock, flags);

	len += sprintf(buf + len, "\n");

	/* ddr duty cycle show */
	glob_ticks = 0;

	len += sprintf(buf + len,
			"\nddr_duty_cycle operating point list:\n");

	len += sprintf(buf + len, "idx|dmcfs|glob_ratio|idle_ratio"
			"|busy_ratio|data_ratio|util_ratio\n");

	len += sprintf(buf + len, "-----------------------------"
			"------------------------------------\n");

	spin_lock_irqsave(&data->lock, flags);

	for (i = 0; i < data->ddr_freq_tbl_len; i++)
		glob_ticks += ddr_ticks[i].reg[0];

	k = 0;
	while ((glob_ticks >> k) > 0x7FFF)
		k++;

	for (i = 0; i < data->ddr_freq_tbl_len; i++) {

		if ((unsigned int)(glob_ticks>>k) != 0)
			glob_ratio =
				(unsigned int)(ddr_ticks[i].reg[0] >> k)
				* 100000 / (unsigned int)(glob_ticks >> k) + 5;
		else
			glob_ratio = 0;

		j = 0;
		while ((ddr_ticks[i].reg[0] >> j) > 0x7FFF)
			j++;

		tmp_total = ddr_ticks[i].reg[0] >> j;
		tmp_rw_cmd = ddr_ticks[i].reg[2] >> j;
		tmp_no_util = ddr_ticks[i].reg[3] >> j;

		if (ver == MCK4)
			tmp_dpc_idle = ddr_ticks[i].reg[1] >> j;
		else if (ver == MCK5)
			tmp_busy = ddr_ticks[i].reg[1] >> j;
		else
			/* this should never happen */
			BUG_ON(1);

		if (tmp_total != 0) {
			tmp_data_cycle = tmp_rw_cmd * data->bst_len / 2;

			data_ratio = tmp_data_cycle * 100000 / tmp_total + 5;

			if (ver == MCK4) {
				busy_ratio = (tmp_data_cycle + tmp_no_util)
					* 100000 / tmp_total + 5;

				idle_ratio = (tmp_total - tmp_data_cycle
					- tmp_no_util) * 100000 / tmp_total + 5;

				util_ratio = tmp_data_cycle * 100000
					/ (tmp_data_cycle + tmp_no_util) + 5;
			} else if (ver == MCK5) {
				busy_ratio = tmp_busy * 100000 / tmp_total + 5;

				idle_ratio = (tmp_total - tmp_busy)
					* 100000 / tmp_total + 5;

				util_ratio = tmp_data_cycle * 100000
					/ tmp_busy + 5;
			}
		} else {
			idle_ratio = 0;
			busy_ratio = 0;
			data_ratio = 0;
			util_ratio = 0;
		}

		len += sprintf(buf + len, "%3d|%5u|%6u.%02u%%|%6u.%02u%%"
			"|%6u.%02u%%|%6u.%02u%%|%6u.%02u%%\n",
			       i, ddr_index2_rate(data, i)/1000,
				glob_ratio/1000, (glob_ratio%1000)/10,
				idle_ratio/1000, (idle_ratio%1000)/10,
				busy_ratio/1000, (busy_ratio%1000)/10,
				data_ratio/1000, (data_ratio%1000)/10,
				util_ratio/1000, (util_ratio%1000)/10);
	}
	spin_unlock_irqrestore(&data->lock, flags);

	len += sprintf(buf + len, "\n");

	return len;
}

/* used to collect ddr cnt during a time */
static ssize_t dp_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	unsigned int cap_flag, i;
	unsigned long flags;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	sscanf(buf, "%u", &cap_flag);

	if (cap_flag == 1) {
		spin_lock_irqsave(&data->lock, flags);
		for (i = 0; i < data->ddr_freq_tbl_len; i++) {
			memset(data->dmc.ddr_ticks[i].reg, 0,
				sizeof(u64) * data->dmc.pmucnt_in_use);
		}
		spin_unlock_irqrestore(&data->lock, flags);
		data->is_ddr_stats_working = 1;
	} else if (cap_flag == 0) {
		data->is_ddr_stats_working = 0;
	} else
		dev_err(&data->devfreq->dev,
			"echo 1 > ddr_profiling to reset and start\n"
			"echo 0 > ddr_profiling to stop\n"
			"cat ddr_profiling to show ddr ticks and duty cycle\n");

	return size;
}

static struct pm_qos_request ddrfreq_qos_req_max;
static ssize_t ddr_max_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int max_level, min_level = PM_QOS_DEFAULT_VALUE;
	struct list_head *list_min;
	struct plist_node *node;
	struct pm_qos_request *req;

	static int inited;
	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);


	if (0x1 != sscanf(buf, "%d", &max_level)) {
		dev_err(dev, "<ERR> wrong parameter,");
		dev_err(dev, "echo level > ddr_max to set max ddr rate\n");
		return -E2BIG;
	}
	if (!inited) {
		ddrfreq_qos_req_max.name = "userspace";
		pm_qos_add_request(&ddrfreq_qos_req_max,
				PM_QOS_DDR_DEVFREQ_MAX,
				PM_QOS_DEFAULT_VALUE);
		inited = 1;
	}

	list_min = &pm_qos_array[PM_QOS_DDR_DEVFREQ_MIN]
			->constraints->list.node_list;
	list_for_each_entry(node, list_min, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (req->name && !strcmp(req->name, "cp")) {
			min_level = node->prio;
			break;
		}
	}
	if ((max_level == PM_QOS_DEFAULT_VALUE) || (max_level >= min_level))
		pm_qos_update_request(&ddrfreq_qos_req_max, max_level);

	return size;
}
static ssize_t ddr_max_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);
	if (ddr_devfreq_profile.max_qos_type)
		return sprintf(buf, "%d\n",
		      pm_qos_request(PM_QOS_DDR_DEVFREQ_MAX));
	else
		return sprintf(buf, "ddr_max Unsupport!!\n");

}

static struct pm_qos_request ddrfreq_qos_req_min;
static ssize_t ddr_min_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;
	int min_level = PM_QOS_DEFAULT_VALUE;

	static int inited;
	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);


	if (0x1 != sscanf(buf, "%d", &min_level)) {
		dev_err(dev, "<ERR> wrong parameter, echo level > ddr_min to set min ddr rate\n");
		return -E2BIG;
	}
	if (unlikely(!inited)) {
		ddrfreq_qos_req_min.name = "userspace";
		pm_qos_add_request(&ddrfreq_qos_req_min,
				PM_QOS_DDR_DEVFREQ_MIN,
				PM_QOS_DEFAULT_VALUE);
		inited = 1;
	}

	pm_qos_update_request(&ddrfreq_qos_req_min, min_level);

	return size;
}
static ssize_t ddr_min_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	if (ddr_devfreq_profile.min_qos_type)
		return sprintf(buf, "%d\n",
		      pm_qos_request(PM_QOS_DDR_DEVFREQ_MIN));
	else
		return sprintf(buf, "ddr_min Unsupport!!\n");
}

static DEVICE_ATTR(high_upthrd_swp, S_IRUGO | S_IWUSR,
	high_swp_show, high_swp_store);
static DEVICE_ATTR(high_upthrd, S_IRUGO | S_IWUSR,
	high_upthrd_show, high_upthrd_store);
static DEVICE_ATTR(disable_ddr_fc, S_IRUGO | S_IWUSR,
	disable_show, disable_store);
static DEVICE_ATTR(ddr_freq, S_IRUGO | S_IWUSR, ddr_freq_show, ddr_freq_store);
static DEVICE_ATTR(ddr_profiling, S_IRUGO | S_IWUSR, dp_show, dp_store);
static DEVICE_ATTR(ddr_max, S_IRUGO | S_IWUSR, ddr_max_show, ddr_max_store);
static DEVICE_ATTR(ddr_min, S_IRUGO | S_IWUSR, ddr_min_show, ddr_min_store);

static int ddr_devfreq_probe(struct platform_device *pdev)
{
	int i = 0, res;
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct ddr_devfreq_data *data = NULL;
	struct devfreq_frequency_table *tbl;
	unsigned int reg_info[2];
	unsigned int freq_qos = 0;
	struct devfreq_pm_qos_table *qos_list;
	unsigned int tmp, ver, pmucnt_in_use;

	data = devm_kzalloc(dev, sizeof(struct ddr_devfreq_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory for devfreq data.\n");
		return -ENOMEM;
	}

	data->ddr_clk = clk_get(dev, NULL);
	if (IS_ERR(data->ddr_clk)) {
		dev_err(dev, "Cannot get clk ptr.\n");
		return PTR_ERR(data->ddr_clk);
	}

	if (IS_ENABLED(CONFIG_OF)) {
		if (of_property_read_u32_array(pdev->dev.of_node,
					       "reg", reg_info, 2)) {
			dev_err(dev, "Failed to get register info\n");
			return -ENODATA;
		}
	} else {
		reg_info[0] = DEFAULT_MCK_BASE_ADDR;
		reg_info[1] = DEFAULT_MCK_REG_SIZE;
	}

	data->dmc.hw_base = ioremap(reg_info[0], reg_info[1]);

	/* read MCK controller version */
	data->dmc.version = MCK_UNKNOWN;

	tmp = readl(data->dmc.hw_base);

	ver = (tmp & MCK5_VER_MASK) >> MCK5_VER_SHIFT;
	if (ver == MCK5) {
		data->dmc.version = ver;
		data->dmc.pmucnt_in_use = DEFAULT_PERCNT_IN_USE;
	} else {
		ver = (tmp & MCK4_VER_MASK) >> MCK4_VER_SHIFT;
		if (ver == MCK4) {
			data->dmc.version = ver;
			data->dmc.pmucnt_in_use = DEFAULT_PERCNT_IN_USE;
		}
	}

	if (data->dmc.version == MCK_UNKNOWN) {
		dev_err(dev, "Unsupported mck version!\n");
		return -EINVAL;
	}
	dev_info(dev, "mck%d controller is detected!\n", ver);

	configure_mck_pmu_regs(data);

	/* get ddr burst length */
	if (data->dmc.version == MCK4) {
		data->bst_len = 1 << ((readl(data->dmc.hw_base
			+ MCK4_SDRAM_CTRL4) & MCK4_SDRAM_CTRL4_BL_MASK)
			>> MCK4_SDRAM_CTRL4_BL_SHIFT);
	} else if (data->dmc.version == MCK5) {
		data->bst_len = 1 << ((readl(data->dmc.hw_base
			+ MCK5_CH0_SDRAM_CFG1) & MCK5_CH0_SDRAM_CFG1_BL_MASK)
			>> MCK5_CH0_SDRAM_CFG1_BL_SHIFT);
	}
	dev_info(dev, "ddr burst length = %d\n", data->bst_len);

	/* save ddr frequency tbl */
	i = 0;
	tbl = devfreq_frequency_get_table(DEVFREQ_DDR);
	if (tbl) {
		while (tbl->frequency != DEVFREQ_TABLE_END) {
			data->ddr_freq_tbl[i] = tbl->frequency;
			tbl++;
			i++;
		}
		data->ddr_freq_tbl_len = i;
	}

	ddr_devfreq_profile.initial_freq =
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ;

	/* set the frequency table of devfreq profile */
	if (data->ddr_freq_tbl_len) {
		ddr_devfreq_profile.freq_table = data->ddr_freq_tbl;
		ddr_devfreq_profile.max_state = data->ddr_freq_tbl_len;
		for (i = 0; i < data->ddr_freq_tbl_len; i++)
			opp_add(dev, data->ddr_freq_tbl[i], 1000);
	}

	/* allocate memory for performnace counter related arrays */
	pmucnt_in_use = data->dmc.pmucnt_in_use;
	data->dmc.ddr_perf_cnt_old
		= devm_kzalloc(dev, sizeof(unsigned int) * pmucnt_in_use,
			GFP_KERNEL);
	if (data->dmc.ddr_perf_cnt_old == NULL) {
		dev_err(dev, "Cannot allocate memory for perf_cnt.\n");
		return -ENOMEM;
	}

	for (i = 0; i < data->ddr_freq_tbl_len; i++) {
		data->dmc.ddr_ticks[i].reg = devm_kzalloc(dev,
			sizeof(u64) * pmucnt_in_use, GFP_KERNEL);
		if (data->dmc.ddr_ticks[i].reg == NULL) {
			dev_err(dev, "Cannot allocate memory for perf_cnt.\n");
			return -ENOMEM;
		}
	}

	/*
	 * Initilize the devfreq QoS if freq-qos flag is enabled.
	 * By default, the flag is disabled.
	 */
	freq_qos = 0;

	if (IS_ENABLED(CONFIG_OF)) {
		if (of_property_read_bool(pdev->dev.of_node, "marvell,qos"))
			freq_qos = 1;
	}

	if (freq_qos) {
		qos_list = devm_kzalloc(dev,
			sizeof(struct devfreq_pm_qos_table)
				* data->ddr_freq_tbl_len, GFP_KERNEL);
		if (qos_list == NULL) {
			dev_err(dev, "Cannot allocate memory for qos_list.\n");
			return -ENOMEM;
		}

		for (i = 0; i < data->ddr_freq_tbl_len; i++) {
			qos_list[i].freq = data->ddr_freq_tbl[i];
			qos_list[i].qos_value = DDR_CONSTRAINT_LVL0 + i;
			dev_dbg(dev, "ddr_devfreq: qos: %ld, %d\n",
				qos_list[i].freq, qos_list[i].qos_value);
		}
		/* add the tail of qos_list */
		qos_list[i].freq = 0;
		qos_list[i].qos_value = 0;

		ddr_devfreq_profile.qos_list = qos_list;
		ddr_devfreq_profile.min_qos_type = PM_QOS_DDR_DEVFREQ_MIN;
		ddr_devfreq_profile.max_qos_type = PM_QOS_DDR_DEVFREQ_MAX;
	}

#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
	devfreq_throughput_data.freq_table = data->ddr_freq_tbl;
	devfreq_throughput_data.table_len = data->ddr_freq_tbl_len;

	devfreq_throughput_data.throughput_table =
		kzalloc(devfreq_throughput_data.table_len
			* sizeof(struct throughput_threshold), GFP_KERNEL);
	if (NULL == devfreq_throughput_data.throughput_table) {
		dev_err(dev,
			"Cannot allocate memory for throughput table\n");
		return -ENOMEM;
	}

	for (i = 0; i < devfreq_throughput_data.table_len; i++) {
		devfreq_throughput_data.throughput_table[i].up =
		   devfreq_throughput_data.upthreshold
		   * devfreq_throughput_data.freq_table[i] / 100;
		devfreq_throughput_data.throughput_table[i].down =
		   (devfreq_throughput_data.upthreshold
		   - devfreq_throughput_data.downdifferential)
		   * devfreq_throughput_data.freq_table[i] / 100;
	}
#endif /* CONFIG_DEVFREQ_GOV_THROUGHPUT */

	spin_lock_init(&data->lock);

	data->devfreq = devfreq_add_device(&pdev->dev, &ddr_devfreq_profile,
				"throughput", &devfreq_throughput_data);
	if (IS_ERR(data->devfreq)) {
		dev_err(dev, "devfreq add error !\n");
		ret =  (unsigned long)data->devfreq;
		goto err_devfreq_add;
	}

	data->high_upthrd_swp = 800000;
	data->high_upthrd = 30;

	/* init default devfreq min_freq and max_freq */
	data->devfreq->min_freq = data->devfreq->qos_min_freq =
		data->ddr_freq_tbl[0];
	data->devfreq->max_freq = data->devfreq->qos_max_freq =
		data->ddr_freq_tbl[data->ddr_freq_tbl_len - 1];
	data->last_polled_at = jiffies;

	res = device_create_file(&pdev->dev, &dev_attr_disable_ddr_fc);
	if (res) {
		dev_err(dev,
			"device attr disable_ddr_fc create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create0;
	}

	res = device_create_file(&pdev->dev, &dev_attr_ddr_freq);
	if (res) {
		dev_err(dev, "device attr ddr_freq create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create1;
	}

	res = device_create_file(&pdev->dev, &dev_attr_ddr_profiling);
	if (res) {
		dev_err(dev,
			"device attr ddr_profiling create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create2;
	}
	res = device_create_file(&pdev->dev, &dev_attr_ddr_max);
	if (res) {
		dev_err(dev, "device attr ddr_max create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create3;
	}

	res = device_create_file(&pdev->dev, &dev_attr_ddr_min);
	if (res) {
		dev_err(dev,
			"device attr ddr_min create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create4;
	}

#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
	res = device_create_file(&pdev->dev, &dev_attr_high_upthrd_swp);
	if (res) {
		dev_err(dev,
			"device attr high_upthrd_swp create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create5;
	}

	res = device_create_file(&pdev->dev, &dev_attr_high_upthrd);
	if (res) {
		dev_err(dev,
			"device attr high_upthrd create fail: %d\n", res);
		ret = -ENOENT;
		goto err_file_create5;
	}

	/*
	 * register the notifier to cpufreq driver,
	 * it is triggered when core freq-chg is done
	 */
	cpufreq_register_notifier(&upthreshold_freq_notifier,
					CPUFREQ_TRANSITION_NOTIFIER);
#endif
	cur_data = data;
	platform_set_drvdata(pdev, data);
	ddr_perf_counter_init(data);

	return 0;
err_file_create5:
	device_remove_file(&pdev->dev, &dev_attr_high_upthrd_swp);
	device_remove_file(&pdev->dev, &dev_attr_high_upthrd);
err_file_create4:
	device_remove_file(&pdev->dev, &dev_attr_ddr_max);
err_file_create3:
	device_remove_file(&pdev->dev, &dev_attr_ddr_profiling);
err_file_create2:
	device_remove_file(&pdev->dev, &dev_attr_ddr_freq);
err_file_create1:
	device_remove_file(&pdev->dev, &dev_attr_disable_ddr_fc);
err_file_create0:
	devfreq_remove_device(data->devfreq);
err_devfreq_add:

#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
	kfree(devfreq_throughput_data.throughput_table);
#endif /* CONFIG_DEVFREQ_GOV_THROUGHPUT */
	return ret;
}

static int ddr_devfreq_remove(struct platform_device *pdev)
{
	struct ddr_devfreq_data *data = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_disable_ddr_fc);
	device_remove_file(&pdev->dev, &dev_attr_ddr_freq);
	device_remove_file(&pdev->dev, &dev_attr_ddr_profiling);
	device_remove_file(&pdev->dev, &dev_attr_high_upthrd_swp);
	device_remove_file(&pdev->dev, &dev_attr_high_upthrd);
	devfreq_remove_device(data->devfreq);

#ifdef CONFIG_DEVFREQ_GOV_THROUGHPUT
	kfree(devfreq_throughput_data.throughput_table);
#endif /* CONFIG_DEVFREQ_GOV_THROUGHPUT */

	return 0;
}

static const struct of_device_id devfreq_ddr_dt_match[] = {
	{.compatible = "marvell,devfreq-ddr" },
	{},
};
MODULE_DEVICE_TABLE(of, devfreq_ddr_dt_match);

#ifdef CONFIG_PM
static unsigned long saved_ddrclk;
static int mck4_suspend(struct device *dev)
{
	struct list_head *list_min;
	struct plist_node *node;
	struct pm_qos_request *req;
	unsigned int qos_min, i = 0;
	unsigned long new_ddrclk, cp_request = 0;
	static struct devfreq_pm_qos_table *ddr_freq_qos_table;
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	ddr_freq_qos_table = data->devfreq->profile->qos_list;
	new_ddrclk = data->ddr_freq_tbl[0];

	mutex_lock(&data->devfreq->lock);

	/* scaling to the min frequency before entering suspend */
	saved_ddrclk = clk_get_rate(data->ddr_clk);
	qos_min = (unsigned int)pm_qos_request(PM_QOS_DDR_DEVFREQ_MIN);
	if (qos_min > DDR_CONSTRAINT_LVL_RSV) {
		list_min = &pm_qos_array[PM_QOS_DDR_DEVFREQ_MIN]
			   ->constraints->list.node_list;
		list_for_each_entry(node, list_min, node_list) {
			req = container_of(node, struct pm_qos_request, node);
			if (req->name && !strcmp(req->name, "cp") &&
			    (node->prio > DDR_CONSTRAINT_LVL0)) {
				dev_info(dev, "%s request min qos\n",
					req->name);
				cp_request = 1;
				break;
			}
		}
	}

	/* if CP request QOS min, set rate as CP request */
	if (cp_request) {
		do {
			if (node->prio == ddr_freq_qos_table[i].qos_value) {
				new_ddrclk = ddr_freq_qos_table[i].freq;
				break;
			}
			i++;
		} while (ddr_freq_qos_table[i].freq != 0);

		if (ddr_freq_qos_table[i].freq == 0)
			dev_err(dev, "DDR qos value is wrong!\n");
	}

	clk_set_rate(data->ddr_clk, new_ddrclk * KHZ_TO_HZ);
	dev_info(dev, "Change ddr freq to lowest value. (cur: %luKhz)\n",
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);
	mutex_unlock(&data->devfreq->lock);
	return 0;
}

static int mck4_resume(struct device *dev)
{
	struct platform_device *pdev;
	struct ddr_devfreq_data *data;

	pdev = container_of(dev, struct platform_device, dev);
	data = platform_get_drvdata(pdev);

	mutex_lock(&data->devfreq->lock);
	/* scaling to saved frequency after exiting suspend */
	clk_set_rate(data->ddr_clk, saved_ddrclk);
	dev_info(dev, "Change ddr freq to saved value. (cur: %luKhz)\n",
		clk_get_rate(data->ddr_clk) / KHZ_TO_HZ);
	mutex_unlock(&data->devfreq->lock);
	return 0;
}

static const struct dev_pm_ops mck4_pm_ops = {
	.suspend	= mck4_suspend,
	.resume		= mck4_resume,
};
#endif

static struct platform_driver ddr_devfreq_driver = {
	.probe = ddr_devfreq_probe,
	.remove = ddr_devfreq_remove,
	.driver = {
		.name = "devfreq-ddr",
		.of_match_table = of_match_ptr(devfreq_ddr_dt_match),
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &mck4_pm_ops,
#endif
	},
};

static int __init ddr_devfreq_init(void)
{
	return platform_driver_register(&ddr_devfreq_driver);
}
fs_initcall(ddr_devfreq_init);

static void __exit ddr_devfreq_exit(void)
{
	platform_driver_unregister(&ddr_devfreq_driver);
}
module_exit(ddr_devfreq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mck4 memorybus devfreq driver");
