/*
    Marvell SSIPC misc driver for Linux
    Copyright (C) 2010 Marvell International Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as
    published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef CONFIG_SSIPC_SUPPORT
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#include "shm.h"
#include "shm_share.h"
#include "msocket.h"
#include "seh_linux.h"
#include "modem_utils.h"

#define DPRINT(fmt, args ...)   printk(KERN_INFO "SSIPC MISC: " fmt, ## args)
#define DBGMSG(fmt, args ...)   printk(KERN_DEBUG "SSIPC MISC: " fmt, ## args)
#define ERRMSG(fmt, args ...)   printk(KERN_ERR "SSIPC MISC: " fmt, ## args)

#define SHMEM_RECV_LIMIT 2048
#define SHMEM_PAYLOAD_LIMIT (SHMEM_RECV_LIMIT - sizeof(struct sk_buff) \
		- sizeof(struct shm_skhdr))

#define SSIPC_START_PROC_ID	0x06
#define SSIPC_DATA_PROC_ID	0x04

#define MAX_IOD_RXQ_LEN 2000

struct io_device {
	char *name;
	char *app;
	int   port;
	struct miscdevice  miscdev;
	wait_queue_head_t wq;
	struct sk_buff_head sk_rx_q;
	int sock_fd;
	struct task_struct *misc_init_task_ref;
	struct task_struct *misc_rcv_task_ref;
	int channel_inited;

	/* Reference count */
	atomic_t opened;

	/*channel status*/
	int channel_status;
};

#define to_io_device(misc) container_of(misc, struct io_device, miscdev)

static struct io_device umts_io_devices[] = {
	[0] = {
	       .name = "umts_boot0",
	       .app = "s-ril",
	       .port = 0},
	[1] = {
	       .name = "umts_ipc0",
	       .app = "s-ril",
	       .port = CISTUB_PORT},
	[2] = {
	       .name = "umts_attest0",
	       .app = "serial_client",
	       .port = RAW_AT_PORT},
	[3] = {
	       .name = "umts_atdun0",
	       .app = "atcmdsrv",
	       .port = RAW_AT_DUN_PORT},
	[4] = {
	       .name = "umts_atprod0",
	       .app = "atcmdsrv",
	       .port = RAW_AT_PROD_PORT},
	[5] = {
	       .name = "umts_atsimal0",
	       .app = "simal",
	       .port = RAW_AT_SIMAL_PORT},
	[6] = {
	       .name = "umts_rfs0",
	       .app = "s-ril",
	       .port = NVMSRV_PORT},
};

/* polling modem status */
#define IOCTL_MODEM_STATUS		_IO('o', 0x27)
/* trigger modem force reset */
#define IOCTL_MODEM_RESET		_IO('o', 0x21)
/* trigger modem crash, final action rely on EE_CFG in NVM */
#define IOCTL_MODEM_FORCE_CRASH_EXIT		_IO('o', 0x34)

/* modem state report for SSIPC use
 * currently only report OFFLINE/ONLINE/CRASH_RESET
 */

enum modem_state {
	STATE_OFFLINE,
	STATE_CRASH_RESET,
	STATE_CRASH_EXIT,
	STATE_BOOTING,
	STATE_ONLINE,
	STATE_NV_REBUILDING,
	STATE_LOADER_DONE,
	STATE_SIM_ATTACH,
	STATE_SIM_DETACH,
};

static const char const *modem_state_str[] = {
	[STATE_OFFLINE]		= "OFFLINE",
	[STATE_CRASH_RESET]	= "CRASH_RESET",
	[STATE_CRASH_EXIT]	= "CRASH_EXIT",
	[STATE_BOOTING]		= "BOOTING",
	[STATE_ONLINE]		= "ONLINE",
	[STATE_NV_REBUILDING]	= "NV_REBUILDING",
	[STATE_LOADER_DONE]	= "LOADER_DONE",
	[STATE_SIM_ATTACH]	= "SIM_ATTACH",
	[STATE_SIM_DETACH]	= "SIM_DETACH",
};

static const inline char *get_modem_state_str(int state)
{
	return modem_state_str[state];
}

static int if_msocket_connect(void)
{
	return msocket_is_synced;
}

static int get_modem_state(struct io_device *iod)
{
	if (!iod->port)
		return if_msocket_connect() ? STATE_ONLINE : STATE_OFFLINE;

	else
		return iod->channel_status;
}

static inline int queue_skb_to_iod(struct sk_buff *skb, struct io_device *iod)
{
	struct sk_buff_head *rxq = &iod->sk_rx_q;
	struct sk_buff *victim;

	skb_queue_tail(rxq, skb);

	if (rxq->qlen > MAX_IOD_RXQ_LEN) {
		ERRMSG("%s: %s application may be dead (rxq->qlen %d > %d)\n",
			iod->name, iod->app ? iod->app : "corresponding",
			rxq->qlen, MAX_IOD_RXQ_LEN);
		victim = skb_dequeue(rxq);
		if (victim)
			dev_kfree_skb_any(victim);
		return -ENOSPC;
	} else {
		DBGMSG("%s: rxq->qlen = %d\n", iod->name, rxq->qlen);
		return 0;
	}
}

static int rx_raw_misc(struct sk_buff *skb, struct io_device *iod)
{
	/* Remove the msocket header */
	skb_pull(skb, SHM_HEADER_SIZE);

	queue_skb_to_iod(skb, iod);

	wake_up_interruptible(&iod->wq);

	return 0;
}


static int misc_open(struct inode *inode, struct file *filp)
{
	struct io_device *iod = to_io_device(filp->private_data);
	int ref_cnt;

	filp->private_data = (void *)iod;

	ref_cnt = atomic_inc_return(&iod->opened);

	DPRINT("%s (opened %d)\n", iod->name, ref_cnt);

	return 0;
}

static int misc_release(struct inode *inode, struct file *filp)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	int ref_cnt;

	skb_queue_purge(&iod->sk_rx_q);
	ref_cnt = atomic_dec_return(&iod->opened);

	DPRINT("%s (opened %d)\n", iod->name, ref_cnt);

	return 0;
}

static unsigned int misc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	int p_state = get_modem_state(iod);

	poll_wait(filp, &iod->wq, wait);

	if (!skb_queue_empty(&iod->sk_rx_q) && p_state != STATE_OFFLINE)
		return POLLIN | POLLRDNORM;

	if (p_state == STATE_CRASH_RESET
	    || p_state == STATE_CRASH_EXIT
	    || p_state == STATE_NV_REBUILDING) {
		return POLLHUP;
	} else {
		return 0;
	}
}

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	int p_state;

	switch (cmd) {
	case IOCTL_MODEM_STATUS:
		p_state = get_modem_state(iod);
		DBGMSG("%s: IOCTL_MODEM_STATUS (state %s)\n",
				iod->name, get_modem_state_str(p_state));
		return p_state;
	case IOCTL_MODEM_FORCE_CRASH_EXIT:
		DPRINT("%s: IOCTL_MODEM_FORCE_CRASH_EXIT triggered\n",
				iod->name);
		trigger_modem_crash(0);
		break;
	case IOCTL_MODEM_RESET:
		DPRINT("%s: IOCTL_MODEM_RESET triggered\n",
				iod->name);
		trigger_modem_crash(1);
		break;

	default:
		ERRMSG("%s: ERR! undefined cmd 0x%X\n", iod->name, cmd);
		return -EINVAL;
	}

	return 0;
}

int ssipc_init_task(void *data)
{
	struct io_device *iod = (struct io_device *)data;
	ShmApiMsg shm_msg_hdr;

	while (!kthread_should_stop()) {
		if (!if_msocket_connect()) {
			msleep_interruptible(1000);
			continue;
		}
		break;
	}

	if (!if_msocket_connect()) {
		ERRMSG("%s: channel of %s fail & close\n",
				__func__, iod->name);
		return -1;
	}

	DPRINT("%s:io->port:%d opened\n", __func__, iod->port);
	iod->sock_fd = msocket(iod->port);
	if (iod->sock_fd < 0) {
		ERRMSG("%s: sock fd of %s opened fail\n",
				__func__, iod->name);
		return -1;
	}

	shm_msg_hdr.svcId = iod->port;
	shm_msg_hdr.msglen = 0;
	shm_msg_hdr.procId = SSIPC_START_PROC_ID;

	while (!kthread_should_stop()) {
		if (!iod->channel_inited) {
			msend(iod->sock_fd, (u8 *)&shm_msg_hdr,
					SHM_HEADER_SIZE, MSOCKET_ATOMIC);
			msleep_interruptible(1000);
			DBGMSG("%s: port:%d, send_handshake: %d!\n",
					iod->name,
					iod->port,
					iod->channel_inited);
		} else {
			break;
		}
	}

	iod->misc_init_task_ref = NULL;
	DPRINT("%s: port:%d, init: %d!\n",
			iod->name, iod->port, iod->channel_inited);
	return 0;
}

static int event_handler(struct sk_buff *skb, struct io_device *iod)
{
	ShmApiMsg *shm_msg_hdr;
	u8 *rxmsg = skb->data;
	int handled = 1;

	pr_rx_skb_with_format(skb);

	shm_msg_hdr = (ShmApiMsg *) rxmsg;
	if (shm_msg_hdr->svcId != iod->port) {
		ERRMSG("%s, svcId(%d) is incorrect, expect %d",
			__func__, shm_msg_hdr->svcId, shm_msg_hdr->procId);
		return handled;
	}

	DBGMSG("%s,srvId=%d, procId=%d, len=%d\n",
		__func__,
		shm_msg_hdr->svcId,
		shm_msg_hdr->procId,
		shm_msg_hdr->msglen);

	switch (shm_msg_hdr->procId) {
	case SSIPC_START_PROC_ID:
		iod->channel_inited = 1;
		iod->channel_status = STATE_ONLINE;
		break;
	case SSIPC_DATA_PROC_ID:
		if (atomic_read(&iod->opened) <= 0)
			break;
		if (iod->channel_inited) {
			rx_raw_misc(skb, iod);
			handled = 0;
		}
		break;
	case MsocketLinkdownProcId:
		DPRINT("%s: %s: received  MsocketLinkdownProcId!\n",
				__func__, iod->name);
		iod->channel_inited = 0;
		if (read_ee_config_b_cp_reset() == 1)
			iod->channel_status = STATE_CRASH_RESET;
		else
			iod->channel_status = STATE_CRASH_EXIT;
		wake_up_interruptible(&iod->wq);
		break;
	case MsocketLinkupProcId:
		DPRINT("%s: %s: received  MsocketLinkupProcId!\n",
				__func__, iod->name);
		iod->channel_status = STATE_OFFLINE;
		skb_queue_purge(&iod->sk_rx_q);
		if (iod->misc_init_task_ref)
			kthread_stop(iod->misc_init_task_ref);
		iod->misc_init_task_ref =
		    kthread_run(ssipc_init_task, iod, "SSIPC init task");
		break;
	}
	return handled;
}

int ssipc_recv_task(void *data)
{
	struct io_device *iod = (struct io_device *)data;
	struct sk_buff *skb = NULL;
	int handled = 0;
	allow_signal(SIGSTOP);

	while (!kthread_should_stop()) {
		if (!if_msocket_connect() || iod->sock_fd == -1) {
			msleep_interruptible(1000);
			continue;
		}
		break;
	}

	if (!if_msocket_connect() || iod->sock_fd == -1) {
		ERRMSG("%s: channel of %s recv fail & close\n",
				__func__, iod->name);
		return -1;
	}

	while (!kthread_should_stop()) {
		if (iod->sock_fd == -1) {
			DBGMSG("%s: sock fd of %s is closed, quit thread\n",
			       __func__, iod->name);
			break;
		}
		skb = mrecvskb(iod->sock_fd, SHMEM_RECV_LIMIT, 0);
		if (!skb) {
			msleep_interruptible(1000);
			continue;
		}
		handled = event_handler(skb, iod);
		if (handled) {
			kfree_skb(skb);
			skb = NULL;
		}
	}
	return 0;
}

void io_channel_init(struct io_device *iod)
{
	iod->channel_status = STATE_OFFLINE;
	iod->sock_fd = -1;

	if (!iod->port)
		return;

	if (iod->misc_rcv_task_ref == NULL)
		iod->misc_rcv_task_ref =
		    kthread_run(ssipc_recv_task, iod, "SSIPC recv task");
	if (iod->misc_init_task_ref == NULL)
		iod->misc_init_task_ref =
		    kthread_run(ssipc_init_task, iod, "SSIPC init task");
}

void io_channel_deinit(struct io_device *iod)
{
	mclose(iod->sock_fd);
	iod->sock_fd = -1;
	if (iod->misc_rcv_task_ref) {
		send_sig(SIGSTOP, iod->misc_rcv_task_ref, 1);
		iod->misc_rcv_task_ref = NULL;
	}
	if (iod->misc_init_task_ref)
		kthread_stop(iod->misc_init_task_ref);
	while (iod->misc_init_task_ref)
		msleep_interruptible(20);
}

static ssize_t misc_write(struct file *filp, const char __user *data,
			size_t count, loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;

	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	ShmApiMsg *pshm;


	if (get_modem_state(iod) != STATE_ONLINE) {
		ERRMSG("%s: channel is not ready\n", __func__);
		return -EAGAIN;
	}

	if (count > SHMEM_PAYLOAD_LIMIT) {
		ERRMSG("%s: DATA size bigger than buffer size\n", __func__);
		return -ENOMEM;
	}

	skb = alloc_skb(count + sizeof(*hdr)+sizeof(*pshm), GFP_KERNEL);
	if (!skb) {
		ERRMSG("Data_channel: %s: out of memory.\n", __func__);
		return -ENOMEM;
	}

	skb_reserve(skb, sizeof(*hdr) + sizeof(*pshm));
	if (copy_from_user(skb_put(skb, count), data, count)) {
		kfree_skb(skb);
		ERRMSG("%s: %s: copy_from_user failed.\n",
		       __func__, iod->name);
		return -EFAULT;
	}

	pshm = (ShmApiMsg *)skb_push(skb, sizeof(*pshm));
	pshm->msglen = count;
	pshm->procId = SSIPC_DATA_PROC_ID;
	pshm->svcId = iod->port;
	pr_tx_skb_with_format(skb);
	if (!msendskb(iod->port, skb, count + SHM_HEADER_SIZE, MSOCKET_KERNEL))
		DBGMSG(":%s:msg send Success\n",__func__);
	else
		DBGMSG(":%s:msg send Fail\n",__func__);

	return count;
}

static ssize_t misc_read(struct file *filp, char *buf, size_t count,
			loff_t *fpos)
{
	struct io_device *iod = (struct io_device *)filp->private_data;
	struct sk_buff_head *rxq = &iod->sk_rx_q;
	struct sk_buff *skb;
	int copied = 0;


	if (skb_queue_empty(rxq) && filp->f_flags & O_NONBLOCK) {
		DPRINT("%s: ERR! no data in rxq\n", iod->name);
		return -EAGAIN;
	}

	while (skb_queue_empty(rxq)) {
		if (wait_event_interruptible(iod->wq,
			(!skb_queue_empty(rxq) ||
			 (get_modem_state(iod) != STATE_ONLINE)))) {
			return -ERESTARTSYS;
		}
		if (get_modem_state(iod) != STATE_ONLINE) {
			ERRMSG("%s: channel is not ready\n", __func__);
			return 0;
		}
	}

	skb = skb_dequeue(rxq);

	copied = skb->len > count ? count : skb->len;

	if (copy_to_user(buf, skb->data, copied)) {
		ERRMSG("%s: ERR! copy_to_user fail\n", iod->name);
		dev_kfree_skb_any(skb);
		return -EFAULT;
	}

	DBGMSG("%s: data:%d copied:%d qlen:%d\n",
		iod->name, skb->len, copied, rxq->qlen);

	if (skb->len > count) {
		skb_pull(skb, count);
		skb_queue_head(rxq, skb);
	} else {
		dev_kfree_skb_any(skb);
	}

	return copied;
}

static const struct file_operations misc_io_fops = {
	.owner = THIS_MODULE,
	.open = misc_open,
	.release = misc_release,
	.poll = misc_poll,
	.unlocked_ioctl = misc_ioctl,
	.write = misc_write,
	.read = misc_read,
};

int init_io_device(struct io_device *iod)
{
	int ret = 0;

	atomic_set(&iod->opened, 0);

	/* Register misc */
	init_waitqueue_head(&iod->wq);
	skb_queue_head_init(&iod->sk_rx_q);

	iod->miscdev.minor = MISC_DYNAMIC_MINOR;
	iod->miscdev.name = iod->name;
	iod->miscdev.fops = &misc_io_fops;

	ret = misc_register(&iod->miscdev);
	if (ret)
		ERRMSG("%s: ERR! misc_register failed\n", iod->name);

	DPRINT("%s is created\n", iod->name);

	io_channel_init(iod);

	return ret;
}

void deinit_io_device(struct io_device *iod)
{
	/* release misc */
	misc_deregister(&iod->miscdev);

	io_channel_deinit(iod);

	DPRINT("%s is released\n", iod->name);
}

/* module initialization */
static int __init ssipc_misc_init(void)
{
	int i, ret = 0;
	int num_iodevs = ARRAY_SIZE(umts_io_devices);

	/* init io deivces and connect to modemctl device */
	for (i = 0; i < num_iodevs; i++) {
		struct io_device *iod = &umts_io_devices[i];
		ret = init_io_device(iod);
		if (ret) {
			ERRMSG("%s: init %s io device fail\n", __func__,
					iod->name);
			goto err_free;
		}
	}
	return ret;

err_free:
	ERRMSG("%s: err happened\n", __func__);
	return ret;
}

/* module exit */
static void __exit ssipc_misc_exit(void)
{
	int i;
	int num_iodevs = ARRAY_SIZE(umts_io_devices);

	/* deinit io deivces and connect to modemctl device */
	for (i = 0; i < num_iodevs; i++)
		deinit_io_device(&umts_io_devices[i]);
}

module_init(ssipc_misc_init);
module_exit(ssipc_misc_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell SSIPC misc Driver");
#endif
