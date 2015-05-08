/*
 * PXA9xx CP related
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <mach/hardware.h>
#include <linux/atomic.h>
#include <linux/io.h>

#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <asm/pgtable.h>

#include <mach/cputype.h>
#include <mach/regs-apmu.h>
#include <asm/gpio.h>

#include "pxa_cp_load.h"
#include "pxa_cp_load_ioctl.h"
#include "msocket.h"
#include "shm.h"

struct cp_buffer {
	char *addr;
	int len;
};

uint32_t arbel_bin_phys_addr;
EXPORT_SYMBOL(arbel_bin_phys_addr);
static void *arbel_bin_virt_addr;
static void *reliable_bin_virt_addr;

void __iomem *mpmu_base_addr;

void checkloadinfo(void)
{
	char *buff;
	int i;

	/* Check CP Arbel image in DDR */
	printk(KERN_INFO "Check loading Arbel image: ");
	buff = (char *)arbel_bin_virt_addr;
	for (i = 0; i < 32; i++)
		printk(KERN_INFO "%02x", *buff++);
	printk(KERN_INFO "\n\n\n");

	/* Check ReliableData image in DDR */
	printk(KERN_INFO "Check loading ReliableData image: ");
	buff = (char *)reliable_bin_virt_addr;
	for (i = 0; i < 32; i++)
		printk(KERN_INFO "%02x", *buff++);
	printk(KERN_INFO "\n");
}

static ssize_t cp_store(struct device *sys_dev, struct device_attribute *attr,
			const char *buf, size_t len)
{
	int cp_enable;

	if (kstrtoint(buf, 10, &cp_enable) < 0)
		return 0;
	printk(KERN_INFO "buf={%s}, cp_enable=%d\n", buf, cp_enable);

	if (cp_enable) {
		checkloadinfo();
		cp_releasecp();
	} else {
		cp_holdcp();
	}

	return len;
}

static ssize_t cp_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int len;
	int cp_enable;

	cp_enable = cp_get_status();
	len = sprintf(buf, "%d\n", cp_enable);

	return len;
}

static ssize_t cputype_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int len;
	const char *cputype;

	if (cpu_is_pxa988())
		cputype = "pxa988";
	else if (cpu_is_pxa986())
		cputype = "pxa986";
	else if (cpu_is_pxa1088())
		cputype = "pxa1088";
	else if (cpu_is_pxa1L88())
		cputype = "pxa1L88";
	else
		cputype = "unknown";

	len = sprintf(buf, "%s\n", cputype);

	return len;
}

#define RF_COND_N 33           //0: RF cable is disconnected 1: RF cable is connected
static ssize_t rfCondGpio_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int len;
	int state;

	state = !!gpio_get_value(RF_COND_N);
	printk(KERN_INFO "gpio %d status is %d\n", RF_COND_N, state);
	len = sprintf(buf, "%d\n", state);

	return len;
}


static DEVICE_ATTR(cp, 0644, cp_show, cp_store);
static DEVICE_ATTR(cputype, 0444, cputype_show, NULL);
static DEVICE_ATTR(rfCondGpio, 0444, rfCondGpio_show, NULL);
static struct attribute *cp_attr[] = {
	&dev_attr_cp.attr,
	&dev_attr_cputype.attr,
	&dev_attr_rfCondGpio.attr,
};

static int cp_add(struct device *dev, struct subsys_interface *sif)
{
	int i, n;
	int ret;

	n = ARRAY_SIZE(cp_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(dev->kobj), cp_attr[i]);
		if (ret)
			return -EIO;
	}
	return 0;
}

static int cp_rm(struct device *dev, struct subsys_interface *sif)
{
	int i, n;

	n = ARRAY_SIZE(cp_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(dev->kobj), cp_attr[i]);
	return 0;
}

static struct subsys_interface cp_interface = {
	.name           = "cp",
	.subsys         = &cpu_subsys,
	.add_dev        = cp_add,
	.remove_dev     = cp_rm,
};

static void cp_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_INFO "cp vma open 0x%lx -> 0x%lx (0x%lx)\n",
	       vma->vm_start, vma->vm_pgoff << PAGE_SHIFT,
	       (long unsigned int)vma->vm_page_prot);
}

static void cp_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_INFO "cp vma close 0x%lx -> 0x%lx\n",
	       vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

/* These are mostly for debug: do nothing useful otherwise */
static struct vm_operations_struct vm_ops = {
	.open  = cp_vma_open,
	.close = cp_vma_close
};

/*
 * vma->vm_end, vma->vm_start: specify the user space process address
 *                             range assigned when mmap has been called;
 * vma->vm_pgoff: the physical address supplied by user to mmap in the
 *                last argument (off)
 * However, mmap restricts the offset, so we pass this shifted 12 bits right
 */
int cp_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long pa = vma->vm_pgoff;

	/* we do not want to have this area swapped out, lock it */
	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start, pa, /* physical page index */
			       size, vma->vm_page_prot)) {
		printk(KERN_ERR "remap page range failed\n");
		return -ENXIO;
	}
	vma->vm_ops = &vm_ops;
	cp_vma_open(vma);
	return 0;
}

/*
 * actions after memory address set
 */
static void post_mem_set(struct cpload_cp_addr *addr)
{
	if (addr->first_boot)
		shm_ch_init(addr);
	else {
		cp_shm_exit();
		cp_shm_init(addr);
	}
}

static long cp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;

	if (_IOC_TYPE(cmd) != CPLOAD_IOC_MAGIC) {
		printk(KERN_ERR "%s: seh magic number is wrong!\n", __func__);
		return -ENOTTY;
	}

	ret = 0;

	switch (cmd) {
	case CPLOAD_IOCTL_SET_CP_ADDR:
		{
			struct cpload_cp_addr addr;

			if (copy_from_user
			    (&addr, (struct cpload_cp_addr *)arg, sizeof(addr)))
				return -EFAULT;

			if (arbel_bin_virt_addr)
				iounmap(arbel_bin_virt_addr);
			arbel_bin_virt_addr = NULL;

			if (reliable_bin_virt_addr)
				iounmap(reliable_bin_virt_addr);
			reliable_bin_virt_addr = NULL;

			printk(KERN_INFO "%s: arbel physical " \
			       "address 0x%08x, size %u\n",
			       __func__, addr.arbel_pa, addr.arbel_sz);
			printk(KERN_INFO "%s: reliable bin physical " \
			     "address 0x%08x, size %u\n",
			     __func__, addr.reliable_pa, addr.reliable_sz);

			arbel_bin_phys_addr = addr.arbel_pa;
			arbel_bin_virt_addr =
			    ioremap_nocache(addr.arbel_pa, addr.arbel_sz);
			reliable_bin_virt_addr =
			    ioremap_nocache(addr.reliable_pa, addr.reliable_sz);

			post_mem_set(&addr);
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations cp_fops = {
	.owner		= THIS_MODULE,
	.mmap		= cp_mmap,
	.unlocked_ioctl = cp_ioctl,
};

static struct miscdevice cp_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "cpmem",
	.fops		= &cp_fops,
};

void (*watchdog_count_stop_fp)(void);
EXPORT_SYMBOL(watchdog_count_stop_fp);

static int __init cp_init(void)
{
	int ret;

	mpmu_base_addr = ioremap(APB_PHYS_BASE + 0x50000, SZ_4K);
	if (mpmu_base_addr == NULL) {
		printk(KERN_ERR "error to ioremap MPMU base address\n");
		return -ENOENT;
	}

	ret = misc_register(&cp_miscdev);
	if (ret) {
		printk(KERN_WARNING "%s: failed to register arbel_bin\n",
		       __func__);
		goto err1;
	}
	ret = subsys_interface_register(&cp_interface);
	if (ret) {
		printk(KERN_WARNING "%s: failed to register cp_sysdev\n",
		       __func__);
		goto err2;
	}
	return 0;
err2:
	misc_deregister(&cp_miscdev);
err1:
	iounmap(mpmu_base_addr);

	return -EIO;
}

static void cp_exit(void)
{
	if (arbel_bin_virt_addr)
		iounmap(arbel_bin_virt_addr);
	if (reliable_bin_virt_addr)
		iounmap(reliable_bin_virt_addr);

	misc_deregister(&cp_miscdev);

	if (mpmu_base_addr)
		iounmap(mpmu_base_addr);

	return subsys_interface_unregister(&cp_interface);
}

module_init(cp_init);
module_exit(cp_exit);


MODULE_DESCRIPTION("PXA9XX CP Related Operation");
MODULE_LICENSE("GPL");
