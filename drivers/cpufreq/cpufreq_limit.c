/*
 * drivers/cpufreq/cpufreq_limit.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *	Minsung Kim <ms925.kim@samsung.com>
 *
 * Modified by: Praveen BP <bp.praveen@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq_limit.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/err.h>

static DEFINE_MUTEX(cpufreq_limit_lock);
static LIST_HEAD(cpufreq_limit_requests);

/**
 * cpufreq_limit_get - limit min or max freq, return cpufreq_limit pointer
 * @name	Name given for request
 * @min		limit minimum frequency (0: none)
 * @max		limit maximum frequency (0: none)
 */
struct cpufreq_limit *cpufreq_limit_get(const char *name,
			unsigned long min, unsigned long max)
{
	struct cpufreq_limit *lt;
	int i;

	if (max && max < min)
		return ERR_PTR(-EINVAL);

	lt = kzalloc(sizeof(struct cpufreq_limit), GFP_KERNEL);
	if (!lt)
		return ERR_PTR(-ENOMEM);

	lt->min = min;
	lt->max = max;
	strncpy(lt->name, name, (sizeof(lt->name) - 1));

	pr_debug("%s: %s,%lu,%lu\n", __func__, lt->name, lt->min, lt->max);

	mutex_lock(&cpufreq_limit_lock);
	list_add_tail(&lt->node, &cpufreq_limit_requests);
	mutex_unlock(&cpufreq_limit_lock);

	for_each_online_cpu(i)
		cpufreq_update_policy(i);

	return lt;
}

/**
 * cpufreq_limit_put - release the request for min or max frequency and free
 *			the cpufreq_limit pointer.
 * @limit	a cpufreq_limit pointer that has been requested
 */
int cpufreq_limit_put(struct cpufreq_limit *limit)
{
	int i;

	if (limit == NULL || IS_ERR(limit))
		return -EINVAL;

	pr_debug("%s: %s,%lu,%lu\n", __func__,
				limit->name, limit->min, limit->max);

	mutex_lock(&cpufreq_limit_lock);
	list_del(&limit->node);
	mutex_unlock(&cpufreq_limit_lock);

	for_each_online_cpu(i)
		cpufreq_update_policy(i);

	kfree(limit);
	return 0;
}

static int cpufreq_limit_notifier_policy(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct cpufreq_policy *policy = data;
	struct cpufreq_limit *limit;
	unsigned long min = 0, max = ULONG_MAX;

	if (val != CPUFREQ_ADJUST)
		goto done;

	mutex_lock(&cpufreq_limit_lock);
	list_for_each_entry(limit, &cpufreq_limit_requests, node) {
		if (limit->min > min)
			min = limit->min;
		if (limit->max && limit->max < max)
			max = limit->max;
	}
	mutex_unlock(&cpufreq_limit_lock);

	if (!min && max == ULONG_MAX)
		goto done;

	if (!min)
		min = policy->cpuinfo.min_freq;
	if (max == ULONG_MAX)
		max = policy->cpuinfo.max_freq;

	pr_debug("%s: limiting cpu%d cpufreq to %lu,%lu", __func__,
			policy->cpu, min, max);

	cpufreq_verify_within_limits(policy, min, max);
done:
	return 0;

}

static struct notifier_block notifier_policy_block = {
	.notifier_call = cpufreq_limit_notifier_policy
};

static ssize_t show_cpufreq_limit_requests(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct cpufreq_limit *limit;
	ssize_t len = 0;

	mutex_lock(&cpufreq_limit_lock);
	list_for_each_entry(limit, &cpufreq_limit_requests, node) {
		len += sprintf(buf + len, "%s\t%lu\t%lu\n", limit->name,
				limit->min, limit->max);
	}
	mutex_unlock(&cpufreq_limit_lock);
	return len;
}

static struct global_attr cpufreq_limit_requests_attr =
	__ATTR(cpufreq_limit_requests, 0444, show_cpufreq_limit_requests, NULL);

static struct attribute *limit_attributes[] = {
	&cpufreq_limit_requests_attr.attr,
	NULL,
};

static struct attribute_group limit_attr_group = {
	.attrs = limit_attributes,
	.name = "cpufreq_limit",
};

static int __init cpufreq_limit_init(void)
{
	int ret = cpufreq_register_notifier(&notifier_policy_block,
						CPUFREQ_POLICY_NOTIFIER);
	if (ret)
		return ret;

	return sysfs_create_group(cpufreq_global_kobject, &limit_attr_group);
}

static void __exit cpufreq_limit_exit(void)
{
	cpufreq_unregister_notifier(&notifier_policy_block,
					CPUFREQ_POLICY_NOTIFIER);
	sysfs_remove_group(cpufreq_global_kobject, &limit_attr_group);
}

MODULE_AUTHOR("Minsung Kim <ms925.kim@samsung.com>");
MODULE_DESCRIPTION("'cpufreq_limit' - A driver to limit cpu frequency");
MODULE_LICENSE("GPL");

module_init(cpufreq_limit_init);
module_exit(cpufreq_limit_exit);
