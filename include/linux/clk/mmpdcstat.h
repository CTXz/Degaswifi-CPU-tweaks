#ifndef __MMPDCSTAT_H
#define __MMPDCSTAT_H

#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk-private.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <asm/cputime.h>
#include <asm/div64.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>

enum lowpower_mode {
	LPM_C1,
	LPM_C2,
	LPM_D1P,
	LPM_D1,
	LPM_D2,
	LPM_D2_UDR,
	MAX_LPM_INDEX = 15,
};

#define MAX_BREAKDOWN_TIME 11
/* use the largest possible number is 10 */
#define MAX_LPM_INDEX_DC  10

struct op_dcstat_info {
	unsigned int ppindex;
	unsigned long pprate;
	struct timespec prev_ts;
	long idle_time;		/* ms */
	long busy_time;		/* ms */

	/* used for core stat */
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
};

struct clk_dc_stat_info {
	bool stat_start;
	struct op_dcstat_info *ops_dcstat;
	u32 power_mode;
	unsigned int ops_stat_size;
	unsigned int curopindex;
	unsigned int idle_flag;
	ktime_t C1_idle_start;
	ktime_t C1_idle_end;
	s64 C1_op_total[MAX_LPM_INDEX_DC];
	int C1_op_index;
	u64 C1_count[MAX_LPM_INDEX_DC];
	ktime_t C2_idle_start;
	ktime_t C2_idle_end;
	s64 C2_op_total[MAX_LPM_INDEX_DC];
	int C2_op_index;
	u64 C2_count[MAX_LPM_INDEX_DC];
	ktime_t breakdown_start;
	ktime_t breakdown_end;
	s64 breakdown_time_total[MAX_BREAKDOWN_TIME+1];
	s64 breakdown_time_count[MAX_BREAKDOWN_TIME+1];
};

struct clk_dcstat {
	struct clk *clk;
	struct clk_dc_stat_info clk_dcstat;
	struct list_head node;
};

struct core_dcstat {
	struct clk *clk;
	int cpu_num;
	int *cpu_id;
	struct list_head node;
};


enum clk_stat_msg {
	CLK_STAT_START = 0,
	CLK_STAT_STOP,
	CLK_STATE_ON,
	CLK_STATE_OFF,
	CLK_RATE_CHANGE,
	CLK_DYNVOL_CHANGE,
	CPU_IDLE_ENTER,
	CPU_IDLE_EXIT,
	CPU_M2_OR_DEEPER_ENTER,
};

struct idle_dcstat_info {
	ktime_t all_idle_start;
	ktime_t all_idle_end;
	s64 total_all_idle;
	s64 total_all_idle_count;
	ktime_t all_active_start;
	ktime_t all_active_end;
	s64 total_all_active;
	s64 total_all_active_count;
	ktime_t M2_idle_start;
	s64 M2_idle_total;
	s64 M2_count;
	ktime_t D1P_idle_start;
	s64 D1P_idle_total;
	s64 D1p_count;
	ktime_t D1_idle_start;
	s64 D1_idle_total;
	s64 D1_count;
	ktime_t D2_idle_start;
	s64 D2_idle_total;
	s64 D2_count;
	s64 cal_duration;
	s64 all_idle_op_total[MAX_LPM_INDEX_DC];
	int all_idle_op_index;
	u64 all_idle_count[MAX_LPM_INDEX_DC];
};

#define CLK_DCSTAT_OPS(clk, name)					\
static ssize_t name##_dc_read(struct file *filp,			\
	char __user *buffer, size_t count, loff_t *ppos)		\
{									\
	char *p;							\
	int len = 0;							\
	size_t ret, size = PAGE_SIZE - 1;				\
	p = (char *)__get_free_pages(GFP_NOIO, 0);			\
	if (!p)								\
		return -ENOMEM;						\
	len = show_dc_stat_info(clk, p, size);			\
	if (len < 0) {						\
		free_pages((unsigned long)p, 0);\
		return -EINVAL;					\
	}							\
	if (len == size)						\
		pr_warn("%s The dump buf is not large enough!\n",	\
			 __func__);					\
	ret = simple_read_from_buffer(buffer, count, ppos, p, len);	\
	free_pages((unsigned long)p, 0);				\
	return ret;							\
}									\
static ssize_t name##_dc_write(struct file *filp,			\
		const char __user *buffer, size_t count, loff_t *ppos)	\
{									\
	unsigned int start;						\
	char buf[10] = { 0 };						\
	size_t ret = 0;							\
									\
	if (copy_from_user(buf, buffer, count))				\
		return -EFAULT;						\
	sscanf(buf, "%d", &start);					\
	ret = start_stop_dc_stat(clk, start);			\
	if (ret < 0)							\
		return ret;						\
	return count;							\
}									\
static const struct file_operations name##_dc_ops = {			\
	.owner = THIS_MODULE,						\
	.read = name##_dc_read,						\
	.write = name##_dc_write,					\
};									\

/* function used for clk duty cycle stat */
static inline long ts2ms(struct timespec cur, struct timespec prev)
{
	return (cur.tv_sec - prev.tv_sec) * MSEC_PER_SEC +
		(cur.tv_nsec - prev.tv_nsec) / NSEC_PER_MSEC;
}

static inline u32 calculate_dc(u32 busy, u32 total, u32 *fraction)
{
	u32 result, remainder;
	u64 busy64, remainder64;

	busy64 = (u64)busy;
	result = div_u64_rem(busy64 * 100, total, &remainder);
	remainder64 = (u64)remainder;
	*fraction = div_u64(remainder64 * 100, total);

	return result;
}

int show_dc_stat_info(struct clk *clk, char *buf, ssize_t size);
int start_stop_dc_stat(struct clk *clk,	unsigned int start);
int clk_register_dcstat(struct clk *clk,
	unsigned long *opt, unsigned int opt_size);
typedef int (*powermode)(u32);
#ifdef CONFIG_DEBUG_FS
extern void cpu_dcstat_event(struct clk *clk, unsigned int cpuid,
		enum clk_stat_msg msg, unsigned int tgtop);
extern void clk_dcstat_event(struct clk *clk,
	enum clk_stat_msg msg, unsigned int tgtstate);
extern int register_cpu_dcstat(struct clk *clk, unsigned int cpunum,
	unsigned int *op_table, unsigned int opt_size, powermode func);
extern struct dentry *cpu_dcstat_file_create(const char *file_name,
		struct dentry *parent);
extern struct dentry *clk_dcstat_file_create(const char *file_name,
	struct dentry *parent, const struct file_operations *fops);

extern struct clk *cpu_dcstat_clk;
#else
static inline void cpu_dcstat_event(struct clk *clk, unsigned int cpuid,
			  enum clk_stat_msg msg, unsigned int tgtop)
{

}
static inline void clk_dcstat_event(struct clk *clk,
	enum clk_stat_msg msg, unsigned int tgtstate)
{

}
static int register_cpu_dcstat(struct clk *clk, unsigned int cpunum,
	unsigned int *op_table, unsigned int opt_size, powermode func);
{

}
static struct dentry *cpu_dcstat_file_create(const char *file_name,
		struct dentry *parent);
{

}
static struct dentry *clk_dcstat_file_create(const char *file_name,
	struct dentry *parent, const struct file_operations *fops);
{

}
#endif

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	idle_time = jiffies_to_usecs(idle_time);

	/* change time from us to ms */
	if (wall)
		do_div(*wall, 1000);
	do_div(idle_time, 1000);

	return idle_time;
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu,
					    cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	/* change time from us to ms */
	do_div(*wall, 1000);
	do_div(idle_time, 1000);

	return idle_time;
}

#endif /* __MMPDCSTAT_H */
