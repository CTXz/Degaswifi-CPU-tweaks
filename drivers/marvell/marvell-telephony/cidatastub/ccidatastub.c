/*

 *(C) Copyright 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/aio.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include <asm/system.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <mach/dma.h>
#include <mach/irqs.h>

#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include "common_datastub.h"
#include "shm_share.h"
#include "data_channel_kernel.h"
#include "psd_data_channel.h"
#ifdef CONFIG_SSIPC_SUPPORT
#include "data_path.h"
#endif

static const char *const ccidatastub_name = "ccidatastub";
int dataSvgHandle;
DATAHANDLELIST *hDataList = NULL, *hCsdataList = NULL;
int dataChannelInited;
int csdChannelInited;
int imsChannelInited;
spinlock_t data_handle_list_lock;

#if     1
#define DPRINT(fmt, args ...)     printk(fmt, ## args)
#define DBGMSG(fmt, args ...)     printk(KERN_DEBUG "CIN: " fmt, ## args)
#define ENTER()         printk(KERN_DEBUG "CIN: ENTER %s\n", __func__)
#define LEAVE()         printk(KERN_DEBUG "CIN: LEAVE %s\n", __func__)
#define FUNC_EXIT()     printk(KERN_DEBUG "CIN: EXIT %s\n", __func__)
#else
#define DPRINT(fmt, args ...)     printk(fmt, ##args)
#define DBGMSG(fmt, args ...)     printk(KERN_DEBUG fmt, ##args)
#define ENTER()         do {} while (0)
#define LEAVE()         do {} while (0)
#define FUNC_EXIT()     do {} while (0)
#endif

static void remove_handle_by_cid(DATAHANDLELIST **plist, unsigned char cid)
{
	DATAHANDLELIST *pCurrNode, *pPrevNode;

	pPrevNode = NULL;	/* pPrevNode set to NULL */
	pCurrNode = *plist;	/* pCurrNode set to header */

	ENTER();
	/* search list to find which cid equals */
	while (pCurrNode && pCurrNode->handle.m_cid != cid) {
		pPrevNode = pCurrNode;
		pCurrNode = pCurrNode->next;
	}

	if (pCurrNode) {	/* if found it */
		if (!pPrevNode) {	/* first node */
			*plist = pCurrNode->next;
		} else {	/* in the middle */
			pPrevNode->next = pCurrNode->next;
		}

		/* in any case, free node memory */
		kfree(pCurrNode);
	}
	/* else nothing to do */
	LEAVE();
}

static void add_to_handle_list(DATAHANDLELIST **plist,
			       DATAHANDLELIST *newdrvnode)
{
	/* we add the new node before header */
	newdrvnode->next = *plist;
	*plist = newdrvnode;
}

DATAHANDLELIST *search_handlelist_by_cid(DATAHANDLELIST *pHeader,
					 unsigned char cid)
{
	DATAHANDLELIST *pCurrNode = pHeader;

	while (pCurrNode && pCurrNode->handle.m_cid != cid)
		pCurrNode = pCurrNode->next;

	return pCurrNode;
}

static void remove_handle_list(DATAHANDLELIST **plist)
{
	DATAHANDLELIST *next, *pCurrNode = *plist;

	ENTER();
	/* loop and free all nodes */
	while (pCurrNode) {
		next = pCurrNode->next;
		kfree(pCurrNode);
		pCurrNode = next;
	}

	/* set header pointer to NULL */
	*plist = NULL;
	LEAVE();
}

static int ccidatastub_open(struct inode *inode, struct file *filp)
{
	ENTER();
	return 0;
	LEAVE();
}

static long ccidatastub_ioctl(struct file *filp,
			      unsigned int cmd, unsigned long arg)
{
	DATAHANDLELIST *pNode, **plist;
	struct datahandle_obj dataHandle;
	GCFDATA gcfdata;
	struct sk_buff *gcfbuf;
	unsigned char cid;

	ENTER();
	if (_IOC_TYPE(cmd) != CCIDATASTUB_IOC_MAGIC) {
		DBGMSG("ccidatastub_ioctl: cci magic number is wrong!\n");
		return -ENOTTY;
	}

	DBGMSG("ccidatastub_ioctl,cmd=0x%x\n", cmd);
	switch (cmd) {
	case CCIDATASTUB_START:
#ifndef CONFIG_SSIPC_SUPPORT
		InitPsdChannel();
		InitCsdChannel();
		InitImsChannel();
#endif
		break;

	case CCIDATASTUB_DATAHANDLE:
		if (copy_from_user(&dataHandle, (struct datahandle_obj *)arg,
				   sizeof(dataHandle)))
			return -EFAULT;
		DPRINT("CCIDATASTUB_DATAHANDLE: cid =%d, type =%d\n",
		       dataHandle.m_cid, dataHandle.m_connType);

		spin_lock_irq(&data_handle_list_lock);

		if (dataHandle.m_connType == CI_DAT_CONNTYPE_PS)
			plist = &hDataList;
		else	/* dataHandle.m_connType == CI_DAT_CONNTYPE_CS */
			plist = &hCsdataList;

		if (!search_handlelist_by_cid(*plist, dataHandle.m_cid)) {
			pNode = kmalloc(sizeof(*pNode), GFP_ATOMIC);
			if (!pNode) {
				spin_unlock_irq(&data_handle_list_lock);
				return -ENOMEM;
			}
			pNode->handle = dataHandle;
			pNode->next = NULL;
			add_to_handle_list(plist, pNode);
		} else {
			DPRINT("CCIDATASTUB_DATAHANDLE: cid already exist\n");
		}

		spin_unlock_irq(&data_handle_list_lock);

		break;

	case CCIDATASTUB_DATASVGHANDLE:
		dataSvgHandle = arg;
		DBGMSG("ccidatastub_ioctl,dataSvgHandle=0x%x\n", dataSvgHandle);

		break;

	case CCIDATASTUB_LINKSTATUS:
		/* gDlLinkStatusFlag = arg; */
		/* DBGMSG("ccidatastub_ioctl,DL link status=%d\n",
		 * gDlLinkStatusFlag); */

		break;

	case CCIDATASTUB_CHNOK:
		cid = (unsigned char)arg;
		DPRINT("CCIDATASTUB_CHNOK: cid =%d\n", cid);

		spin_lock_irq(&data_handle_list_lock);
		remove_handle_by_cid(&hDataList, cid);
		spin_unlock_irq(&data_handle_list_lock);

		break;

	case CCIDATASTUB_CHOK:
		cid = (unsigned char)arg;
		DPRINT("CCIDATASTUB_CHOK: cid =%d\n", cid);

		spin_lock_irq(&data_handle_list_lock);
		pNode = search_handlelist_by_cid(hDataList, cid);
		if (pNode)
			pNode->handle.chanState = DATA_CHAN_READY_STATE;
		spin_unlock_irq(&data_handle_list_lock);

		break;
	case CCIDATASTUB_CS_CHNOK:
		cid = (unsigned char)arg;
		DPRINT("CCIDATASTUB_CS_CHNOK: cid =%d\n", cid);

		spin_lock_irq(&data_handle_list_lock);
		remove_handle_by_cid(&hCsdataList, cid);
		spin_unlock_irq(&data_handle_list_lock);

		break;

	case CCIDATASTUB_CS_CHOK:
		cid = (unsigned char)arg;
		DPRINT("CCIDATASTUB_CS_CHOK: cid =%d\n", cid);

		spin_lock_irq(&data_handle_list_lock);
		pNode = search_handlelist_by_cid(hCsdataList, cid);
		if (pNode)
			pNode->handle.chanState = DATA_CHAN_READY_STATE;
		spin_unlock_irq(&data_handle_list_lock);

		break;

	case CCIDATASTUB_REMOVE_ALLCH:
		DPRINT("CCIDATASTUB_REMOVE_ALLCH\n");

		spin_lock_irq(&data_handle_list_lock);
		remove_handle_list(&hDataList);
		remove_handle_list(&hCsdataList);
		spin_unlock_irq(&data_handle_list_lock);

		break;

	case CCIDATASTUB_GCFDATA:	/* For local CGSEND and TGSINK */
	case CCIDATASTUB_GCFDATA_REMOTE: /* For remote CGSEND and TGSINK */
		if (copy_from_user(&gcfdata, (GCFDATA *) arg, sizeof(GCFDATA)))
			return -EFAULT;
		gcfbuf = alloc_skb(gcfdata.len, GFP_KERNEL);
		if (!gcfbuf)
			return -ENOMEM;
		if (copy_from_user
		    (skb_put(gcfbuf, gcfdata.len), gcfdata.databuf,
		     gcfdata.len)) {
			kfree_skb(gcfbuf);
			return -EFAULT;
		}
		sendPSDData(gcfdata.cid, gcfbuf);
		break;
	}
	LEAVE();
	return 0;
}

static const struct file_operations ccidatastub_fops = {
	.open = ccidatastub_open,
	.unlocked_ioctl = ccidatastub_ioctl,
	.owner = THIS_MODULE
};

static struct miscdevice ccidatastub_miscdev = {
	MISC_DYNAMIC_MINOR,
	"ccidatastub",
	&ccidatastub_fops,
};

#ifdef CONFIG_SSIPC_SUPPORT
static void ccidatastub_ready_cb(void)
{
	DPRINT("%s executed\n", __func__);
	InitPsdChannel();
	InitCsdChannel();
	InitImsChannel();
}

static void ccidatastub_delete_cb(void)
{
	DPRINT("%s executed\n", __func__);
	DeInitPsdChannel();
	DeInitCsdChannel();
	DeInitImsChannel();
}
#endif

static int ccidatastub_probe(struct platform_device *dev)
{
	int ret;

	ENTER();

	ret = misc_register(&ccidatastub_miscdev);
	if (ret)
		printk(KERN_ERR "register misc device error\n");

#ifdef CONFIG_SSIPC_SUPPORT
	dp_ready_cb_regist((void *)ccidatastub_ready_cb,
			(void *)ccidatastub_delete_cb);
#endif

	LEAVE();

	return ret;
}

static int ccidatastub_remove(struct platform_device *dev)
{
	ENTER();
#ifndef CONFIG_SSIPC_SUPPORT
	DeInitPsdChannel();
	DeInitCsdChannel();
	DeInitImsChannel();
#endif

	misc_deregister(&ccidatastub_miscdev);
	LEAVE();
	return 0;
}

static void ccidatastub_dev_release(struct device *dev)
{
	return;
}

static struct platform_device ccidatastub_device = {
	.name = "ccidatastub",
	.id = 0,
	.dev = {
		.release = ccidatastub_dev_release,
	},
};

static struct platform_driver ccidatastub_driver = {
	.probe = ccidatastub_probe,
	.remove = ccidatastub_remove,
	.driver = {
		.name = "ccidatastub",
	}
};

static int __init ccidatastub_init(void)
{
	int ret;

	ret = platform_device_register(&ccidatastub_device);
	if (!ret) {
		ret = platform_driver_register(&ccidatastub_driver);
		if (ret)
			platform_device_unregister(&ccidatastub_device);
	} else {
		DPRINT("Cannot register CCIDATASTUB platform device\n");
	}

	spin_lock_init(&data_handle_list_lock);

	return ret;
}

static void __exit ccidatastub_exit(void)
{
	platform_driver_unregister(&ccidatastub_driver);
	platform_device_unregister(&ccidatastub_device);
}

module_init(ccidatastub_init);
module_exit(ccidatastub_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell CI data stub.");
