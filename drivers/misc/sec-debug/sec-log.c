/*
 * drivers/misc/sec-debug/sec-log.c
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

#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

#include <linux/sec-debug.h>
#include <asm/mach/map.h>
#include <asm/tlbflush.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#ifdef CONFIG_SEC_LOG_LAST_KMSG
#include <linux/proc_fs.h>
#endif


/*
 * Example usage: sec_log=256K@0x45000000
 * In above case, log_buf size is 256KB and its base address is
 * 0x45000000 physically. Actually, *(int *)(base - 8) is log_magic and
 * *(int *)(base - 4) is log_ptr. So we reserve (size + 8) bytes from
 * (base - 8).
 */
#define LOG_MAGIC 0x4d474f4c	/* "LOGM" */

/* These variables are also protected by logbuf_lock */
static unsigned *sec_log_ptr;
static char *sec_log_buf;
static unsigned sec_log_size;

#ifdef CONFIG_SEC_LOG_LAST_KMSG
static char *last_kmsg_buffer;
static unsigned last_kmsg_size;
static void __init sec_log_save_old(void);
#else
static inline void sec_log_save_old(void)
{
}
#endif

static unsigned size;
static unsigned long base;

extern void register_log_text_hook(void (*f)(char *text, size_t size),
	char *buf, unsigned *position, size_t bufsize);

static inline void emit_sec_log(char *text, size_t size)
{
	if (sec_log_buf && sec_log_ptr) {
		/* Check overflow */
		size_t pos = *sec_log_ptr & (sec_log_size - 1);
		if (likely(size + pos <= sec_log_size))
			memcpy(&sec_log_buf[pos], text, size);
		else {
			size_t first = sec_log_size - pos;
			size_t second = size - first;
			memcpy(&sec_log_buf[pos], text, first);
			memcpy(&sec_log_buf[0], text + first, second);
		}
		(*sec_log_ptr) += size;
	}
}

static int __init sec_log_setup(char *str)
{
	unsigned size = memparse(str, &str);
	unsigned long base = 0;
	unsigned *sec_log_mag;

	/* If we encounter any problem parsing str ... */
	if (!size || size != roundup_pow_of_two(size) || *str != '@'
	    || kstrtoul(str + 1, 0, &base))
		goto out;

	if (reserve_bootmem(base, size + 8, BOOTMEM_EXCLUSIVE)) {
		pr_err("%s: failed reserving size %lu + 8 " \
		       "at base 0x%lx\n",
		       __func__, (unsigned long)(size >> 20), base);
		goto out;
	} else {
		pr_err("%s: Success reserving size %ldMB + 8 " \
		       "at base 0x%lx\n",
		       __func__, (unsigned long)(size >> 20), base);
	}
	
	sec_log_mag = phys_to_virt(base) + size + 4;
	sec_log_ptr = phys_to_virt(base) + size;
	sec_log_buf = phys_to_virt(base);
	sec_log_size = size;

	pr_info("%s: *sec_log_mag:%x *sec_log_ptr:%x " \
		"sec_log_buf:%p sec_log_size:%d\n",
		__func__, *sec_log_mag, *sec_log_ptr, sec_log_buf,
		sec_log_size);
	
	if (*sec_log_mag != LOG_MAGIC) {
		pr_info("%s: no old log found\n", __func__);
		*sec_log_ptr = 0;
		*sec_log_mag = LOG_MAGIC;
	} else {
		sec_log_save_old();
	}

	register_log_text_hook(emit_sec_log, sec_log_buf, sec_log_ptr,
		sec_log_size);

	sec_getlog_supply_kloginfo(phys_to_virt(base));

out:
	return 0;
}
__setup("sec_log=", sec_log_setup);


#ifdef CONFIG_SEC_LOG_LAST_KMSG
static void __init sec_log_save_old(void)
{
	/* provide previous log as last_kmsg */
	last_kmsg_size =
	    min((unsigned)(1 << CONFIG_LOG_BUF_SHIFT), *sec_log_ptr);
	last_kmsg_buffer = (char *)alloc_bootmem(last_kmsg_size);

	if (last_kmsg_size && last_kmsg_buffer) {
		unsigned i;
		for (i = 0; i < last_kmsg_size; i++)
			last_kmsg_buffer[i] =
			    sec_log_buf[(*sec_log_ptr - last_kmsg_size +
					 i) & (sec_log_size - 1)];

		pr_info("%s: saved old log at %d@%p\n",
			__func__, last_kmsg_size, last_kmsg_buffer);
	} else
		pr_err("%s: failed saving old log %d@%p\n",
		       __func__, last_kmsg_size, last_kmsg_buffer);
}

static ssize_t sec_log_read_old(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= last_kmsg_size)
		return 0;

	count = min(len, (size_t) (last_kmsg_size - pos));
	if (copy_to_user(buf, last_kmsg_buffer + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations last_kmsg_file_ops = {
	.owner = THIS_MODULE,
	.read = sec_log_read_old,
};

static int __init sec_log_late_init(void)
{
	struct proc_dir_entry *entry;

	if (last_kmsg_buffer == NULL)
		return 0;

	entry = proc_create("last_kmsg", S_IFREG | S_IRUGO,
			NULL, &last_kmsg_file_ops);
	if (!entry) {
		pr_err("%s: failed to create proc entry\n", __func__);
		return 0;
	}

	//entry->size = last_kmsg_size;
	proc_set_size(entry, last_kmsg_size);
	return 0;
}

late_initcall(sec_log_late_init);
#endif
