/*
 *  Do flush operation when panic to save those stuff still in cache to mem
 *
 *  Copyright (C) 2013 Marvell International Ltd.
 *  Lei Wen <leiwen@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/reboot.h>
#include <linux/kexec.h>
#include <linux/kdebug.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <asm/setup.h>
#include <linux/kmsg_dump.h>

#include <mach/regs-coresight.h>
#include <mach/cputype.h>
#ifdef CONFIG_REGDUMP
#include <linux/regdump_ops.h>
#endif
#ifdef CONFIG_SEC_DEBUG
#include <linux/sec-debug.h>
#endif

static void *indicator;
static DEFINE_RAW_SPINLOCK(panic_lock);
extern void arm_machine_flush_console(void);
extern void (*arm_pm_restart)(char str, const char *cmd);
#define PANIC_TIMER_STEP 100

static void dump_one_task_info(struct task_struct *tsk)
{
	char stat_array[3] = { 'R', 'S', 'D'};
	char stat_ch;

	stat_ch = tsk->state <= TASK_UNINTERRUPTIBLE ?
		stat_array[tsk->state] : '?';
	pr_info("%8d %8d %8d %16lld %c(%d) %4d    %p %s\n",
			tsk->pid, (int)(tsk->utime), (int)(tsk->stime),
			tsk->se.exec_start, stat_ch, (int)(tsk->state),
			task_cpu(tsk), tsk, tsk->comm);

	show_stack(tsk, NULL);
}

void dump_task_info(void)
{
	struct task_struct *g, *p;

	pr_info("\n");
	pr_info("current proc: %d %s\n", current->pid, current->comm);
	pr_info("-----------------------------------------------------------------------------------\n");
	pr_info("      pid     uTime    sTime    exec(ns)    stat   cpu   task_struct\n");
	pr_info("-----------------------------------------------------------------------------------\n");

	do_each_thread(g, p) {
		if (p->state == TASK_RUNNING ||
			p->state == TASK_UNINTERRUPTIBLE)
			dump_one_task_info(p);
	} while_each_thread(g, p);

	pr_info("-----------------------------------------------------------------------------------\n");
}

void set_emmd_indicator(void)
{
	memset(indicator, 0, PAGE_SIZE);
	*(unsigned long *)indicator = 0x454d4d44;
}

void panic_flush(struct pt_regs *regs)
{
	int i;

	raw_spin_lock(&panic_lock);

	for (i = 0; i < nr_cpu_ids; i++)
		coresight_dump_pcsr(i);
#ifdef CONFIG_REGDUMP
	if (cpu_is_pxa1088() || cpu_is_pxa1L88())
		dump_reg_to_console();
#endif

	printk(KERN_EMERG "EMMD: ready to perform memory dump\n");

	set_emmd_indicator();

	printk(KERN_EMERG "EMMD: done\n");

	raw_spin_unlock(&panic_lock);

}

static int panic_flush_notifier(struct notifier_block *nb,
				   unsigned long l, void *buf)
{
	panic_flush(NULL);
	return NOTIFY_DONE;
}

#if (defined CONFIG_PM)
static ssize_t panic_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t len)
{
	char _buf[80];

	if (strncmp(buf, "PID", 3) == 0) {
		snprintf(_buf, 80, "\nUser Space Panic:%s", buf);
		panic(_buf);
		goto OUT;
	}

	printk(KERN_WARNING "Not valid value!!!\n");
OUT:
	return len;
}

static struct kobj_attribute panic_attr = {
	.attr	= {
		.name = __stringify(panic),
		.mode = 0644,
	},
	.store	= panic_store,
};

static struct attribute *g[] = {
	&panic_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#endif /* CONFIG_PM */

static struct notifier_block pxa_panic_noti = {
	.notifier_call = panic_flush_notifier,
};

static int __init pxa_panic_notifier(void)
{
	struct page *page;

	atomic_notifier_chain_register(&panic_notifier_list, &pxa_panic_noti);

#if (defined CONFIG_PM)
	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;
#endif
	page = pfn_to_page(crashk_res.end >> PAGE_SHIFT);
	indicator = page_address(page);

	panic_on_oops = 1;
	return 0;
}

core_initcall_sync(pxa_panic_notifier);
