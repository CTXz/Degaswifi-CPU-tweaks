/*
    msocket.c Created on: Aug 2, 2010, Jinhua Huang <jhhuang@marvell.com>

    Marvell PXA9XX ACIPC-MSOCKET driver for Linux
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

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <mach/regs-apmu.h>
#include "acipcd.h"
#include "shm.h"
#include "portqueue.h"
#include "msocket.h"
#include "shm_share.h"
#include "data_path.h"
#include "direct_rb.h"

#define CMSOCKDEV_NR_DEVS PORTQ_NUM_MAX

static int shm_ch_inited;

int cmsockdev_major;
int cmsockdev_minor;
int cmsockdev_nr_devs = CMSOCKDEV_NR_DEVS;
struct cmsockdev_dev {
	struct cdev cdev;	/* Char device structure */
};

static int dump_flag;
struct cmsockdev_dev *cmsockdev_devices;
static struct class *cmsockdev_class;

typedef enum
{
	PowerBackOff_ON_CS,
	PowerBackOff_ON_PS,
	PowerBackOff_ON_RB_TEST,
	PowerBackOff_ON_CS_PS,
	PowerBackOff_ON_CS_RB_TEST,
	PowerBackOff_ON_PS_RB_TEST,
	PowerBackOff_ON_CS_PS_RB_TEST,
	PowerBackOff_OFF
} PowerBackOffStatus;

static int gPowerBackOffStatus = PowerBackOff_OFF;
static int gCpPowerMode = 0;


/* forward msocket sync related static function prototype */
static void msocket_sync_worker(struct work_struct *work);
static void msocket_connect(void);
static void msocket_disconnect(void);

static DECLARE_WORK(sync_work, msocket_sync_worker);
static volatile bool msocket_is_sync_canceled;
static spinlock_t msocket_sync_lock;
static struct wakeup_source cp_power_resume_wakeup;

bool msocket_is_synced;
EXPORT_SYMBOL(msocket_is_synced);
bool msocket_recv_up_ioc;
DECLARE_COMPLETION(msocket_peer_sync);

static int seq_flag = SEQ_PORTQUEUE;

static void dump(const unsigned char *data, unsigned int len)
{
	int i;
	char *buf = NULL;
	char *tmp_buf = NULL;
	buf = kmalloc(2 * len + 1, GFP_ATOMIC);
	if (buf == NULL) {
		printk(KERN_ERR "msocket: %s: malloc error\n", __func__);
		return;
	}
	tmp_buf = buf;
	for (i = 0; i < len; i++)
		tmp_buf += sprintf(tmp_buf, "%02x", data[i]);
	printk(KERN_INFO "%s\n", buf);
	kfree(buf);
	return;
}

void data_dump(const unsigned char *data, unsigned int len, int port,
	       int direction)
{
	if (direction == DATA_TX) {
		if ((dump_flag & DUMP_TX) && (dump_flag & DUMP_PORT(port))) {
			printk(KERN_INFO
			     " Msocket TX: DUMP BEGIN port :%d, Length:%d --\n",
			     port, len);
			if (!(dump_flag & DUMP_TX_SP))
				dump(data, len);
		}
	} else if (direction == DATA_RX) {
		if ((dump_flag & DUMP_RX) && (dump_flag & DUMP_PORT(port))) {
			printk(KERN_INFO
			     " Msocket RX: DUMP BEGIN port :%d, Length:%d --\n",
			     port, len);
			if (!(dump_flag & DUMP_RX_SP))
				dump(data, len);
		}
	}
	return;
}

/* open a msocket in kernel */
int msocket(int port)
{
	struct portq *portq;

	portq = portq_open(port);

	if (IS_ERR(portq)) {
		printk(KERN_ERR "MSOCK: can't open queue port %d\n", port);
		return -1;
	}

	return port;
}
EXPORT_SYMBOL(msocket);

/* close a msocket */
int mclose(int sock)
{
	struct portq *portq;

	portq = (struct portq *)portq_array[sock];

	if (!portq) {
		printk(KERN_ERR "MSOCK: closed socket %d failed\n", sock);
		return -1;
	}

	portq_close(portq);

	return 0;
}
EXPORT_SYMBOL(mclose);

/* send packet to msocket */
int msend(int sock, const void *buf, int len, int flags)
{
	struct portq *portq;
	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	bool block = flags == MSOCKET_KERNEL;

	portq = (struct portq *)portq_array[sock];
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: sock %d not opened!\n",
		       __func__, sock);
		return -1;
	}

	/* check the len first */
	if (len > (portq_rbctl->tx_skbuf_size - sizeof(*hdr))) {
		printk(KERN_ERR "MSOCK: %s: port %d, len is %d!!\n",
		       __func__, portq->port, len);
		portq->stat_tx_drop++;
		return -1;
	}

	/* alloc space */
	if (block)
		skb = alloc_skb(len + sizeof(*hdr), GFP_KERNEL);
	else
		skb = alloc_skb(len + sizeof(*hdr), GFP_ATOMIC);

	if (!skb) {
		printk(KERN_ERR "MSOCK: %s: out of memory.\n", __func__);
		return -ENOMEM;
	}
	skb_reserve(skb, sizeof(*hdr));	/* reserve header space */

	memcpy(skb_put(skb, len), buf, len);

	/* push header back */
	hdr = (struct shm_skhdr *)skb_push(skb, sizeof(*hdr));

	hdr->address = 0;
	hdr->port = portq->port;
	hdr->checksum = 0;
	hdr->length = len;

	if (!msocket_is_synced || portq_xmit(portq, skb, block) < 0) {
		kfree_skb(skb);
		printk(KERN_ERR "MSOCK: %s: port %d xmit error.\n",
		       __func__, portq->port);
		return -1;
	}

	return len;
}
EXPORT_SYMBOL(msend);

/* send sk_buf packet to msocket */
int msendskb(int sock, struct sk_buff *skb, int len, int flags)
{
	struct portq *portq;
	struct shm_skhdr *hdr;
	int length;
	bool block = flags == MSOCKET_KERNEL;
	if (NULL == skb) {
		printk(KERN_ERR "MSOCK:%s:skb buff is NULL!\n", __func__);
		return -1;
	}
	portq = (struct portq *)portq_array[sock];
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: sock %d not opened!\n",
		       __func__, sock);
		kfree_skb(skb);
		return -1;
	}

	length = skb->len;
	if (length > (portq_rbctl->tx_skbuf_size - sizeof(*hdr))) {
		printk(KERN_ERR
		       "MSOCK: %s: port %d, len is %d larger than tx_skbuf_size\n",
		       __func__, portq->port, len);
		kfree_skb(skb);
		portq->stat_tx_drop++;
		return -1;
	}

	hdr = (struct shm_skhdr *)skb_push(skb, sizeof(*hdr));
	hdr->address = 0;
	hdr->port = portq->port;
	hdr->checksum = 0;
	hdr->length = len;

	if (!msocket_is_synced || portq_xmit(portq, skb, block) < 0) {
		kfree_skb(skb);
		printk(KERN_ERR "MSOCK: %s: port %d xmit error.\n",
		       __func__, portq->port);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(msendskb);

/* receive packet from msocket */
int mrecv(int sock, void *buf, int len, int flags)
{
	struct portq *portq;
	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	int packet_len;
	bool block = flags == MSOCKET_KERNEL;

	portq = (struct portq *)portq_array[sock];
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: sock %d not opened!\n",
		       __func__, sock);
		return -1;
	}

	skb = portq_recv(portq, block);
	if (IS_ERR(skb)) {
		printk(KERN_DEBUG "MSOCK: %s: portq_recv returns %d\n",
		       __func__, (int)skb);
		return -1;
	}

	if (!skb)
		return 0;

	hdr = (struct shm_skhdr *)skb->data;
	packet_len = hdr->length;
	if (packet_len > len) {
		printk(KERN_ERR "MSOCK: %s: error: no enough space.\n",
		       __func__);
		kfree_skb(skb);
		return -1;	/* error */
	}

	memcpy(buf, skb_pull(skb, sizeof(*hdr)), hdr->length);

	kfree_skb(skb);

	return packet_len;
}
EXPORT_SYMBOL(mrecv);

struct sk_buff *mrecvskb(int sock, int len, int flags)
{
	struct portq *portq;
	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	bool block = flags == MSOCKET_KERNEL;

	portq = (struct portq *)portq_array[sock];
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: sock %d not opened!\n",
		       __func__, sock);
		return NULL;
	}

	skb = portq_recv(portq, block);
	if (IS_ERR(skb)) {
		printk(KERN_DEBUG "MSOCK: %s: portq_recv returns %d\n",
		       __func__, (int)skb);
		return NULL;
	}

	if (!skb)
		return NULL;

	hdr = (struct shm_skhdr *)skb->data;
	if (hdr->length > len) {
		printk(KERN_ERR "MSOCK: %s: error: no enough space.\n",
		       __func__);
		kfree_skb(skb);
		return NULL;	/* error */
	}
	skb_pull(skb, sizeof(*hdr));
	return skb;
}
EXPORT_SYMBOL(mrecvskb);

static void msocket_sync_worker(struct work_struct *work)
{
	/* acquire lock first */
	spin_lock(&msocket_sync_lock);

	while (!msocket_is_sync_canceled) {
		/* send peer sync notify */
		shm_notify_peer_sync(portq_rbctl);

		/* unlock before wait completion */
		spin_unlock(&msocket_sync_lock);

		if (wait_for_completion_timeout(&msocket_peer_sync, HZ)) {
			/* we get CP sync response here */
			printk(KERN_INFO "msocket connection sync with CP O.K.!\n");
			/* acquire lock again */
			spin_lock(&msocket_sync_lock);

			if (!msocket_is_sync_canceled) {
				/* if no one cancel me */
				msocket_is_synced = true;
				/* only when we have received linkup ioctl
				 * can we report the linkup message */
				if (msocket_recv_up_ioc) {
					portq_broadcase_msg
					    (MsocketLinkupProcId);
					data_path_link_up();
					direct_rb_broadcast_msg
					    (MsocketLinkupProcId);
					msocket_recv_up_ioc = false;
				}
			}
			break;
		}
		/* acquire lock again */
		spin_lock(&msocket_sync_lock);
	}

	/* unlock before return */
	spin_unlock(&msocket_sync_lock);
}

/* start msocket sync */
static void msocket_connect(void)
{
	spin_lock(&msocket_sync_lock);
	msocket_is_sync_canceled = false;
	spin_unlock(&msocket_sync_lock);

	portq_schedule_sync(&sync_work);
}

/* stop msocket sync */
static void msocket_disconnect(void)
{
	spin_lock(&msocket_sync_lock);

	/* flag used to cancel any new packet activity */
	msocket_is_synced = false;

	/* flag used to cancel potential peer sync worker */
	msocket_is_sync_canceled = true;

	spin_unlock(&msocket_sync_lock);
	/* ensure that any scheduled work has run to completion */
	portq_flush_workqueue();

	/*
	 * and now no work is active or will be schedule, so we can
	 * cleanup any packet queued and initialize some key data
	 * structure to the beginning state
	 */
	shm_rb_data_init(portq_rbctl);
	portq_flush_init();

}

/*
 * msocket device driver <-------------------------------------->
 */

#define PROC_FILE_NAME		"driver/msocket"

/*
 * This function is called at the beginning of a sequence.
 */
static void *msocket_seq_start(struct seq_file *s, loff_t *pos)
{
	int i = *pos;

	spin_lock_irq(&portq_list_lock);

	if (!i)
		return SEQ_START_TOKEN;

	while (i < PORTQ_NUM_MAX && !portq_array[i])
		i++;

	/* return a non null value to begin the sequence */
	return i >= PORTQ_NUM_MAX ? NULL : (*pos = i, portq_array[i]);
}

/*
 * This function is called after the beginning of a sequence.
 * It's called untill the return is NULL (this ends the sequence).
 */
static void *msocket_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	int i = *pos + 1;
	if (seq_flag == SEQ_DIRECT_RB) {
		seq_flag = SEQ_PORTQUEUE;
		return NULL;
	}

	while ((i < PORTQ_NUM_MAX && !portq_array[i]) || (i == DIAG_PORT))
		i++;
	if (i >= PORTQ_NUM_MAX) {
		seq_flag = SEQ_DIRECT_RB;
		*pos = PORTQ_NUM_MAX;
		return portq_array[DIAG_PORT];
	}

	*pos = i;
	/* return a non null value to step the sequence */
	return portq_array[i];
}

/*
 * This function is called at the end of a sequence
 */
static void msocket_seq_stop(struct seq_file *s, void *v)
{
	spin_unlock_irq(&portq_list_lock);
}

/*
 * This function is called for each "step" of a sequence
 */
static int msocket_seq_show(struct seq_file *s, void *v)
{

	if (v == SEQ_START_TOKEN) {
		seq_printf(s, "shm_is_ap_xmit_stopped: %d\n",
			   portq_rbctl->is_ap_xmit_stopped);
		seq_printf(s, "shm_is_cp_xmit_stopped: %d\n",
			   portq_rbctl->is_cp_xmit_stopped);
		seq_printf(s, "acipc_ap_stopped_num:   %ld\n",
			   portq_rbctl->ap_stopped_num);
		seq_printf(s, "acipc_ap_resumed_num:   %ld\n",
			   portq_rbctl->ap_resumed_num);
		seq_printf(s, "acipc_cp_stopped_num:   %ld\n",
			   portq_rbctl->cp_stopped_num);
		seq_printf(s, "acipc_cp_resumed_num:   %ld\n",
			   portq_rbctl->cp_resumed_num);
		seq_printf(s, "tx_socket_total:        %d\n",
			   portq_rbctl->tx_skbuf_num);
		seq_printf(s, "tx_socket_free:         %d\n",
			   shm_free_tx_skbuf_safe(portq_rbctl));
		seq_printf(s, "rx_socket_total:        %d\n",
			   portq_rbctl->rx_skbuf_num);
		seq_printf(s, "rx_socket_free:         %d\n",
			   shm_free_rx_skbuf_safe(portq_rbctl));
		seq_printf(s, "NVM_wake_lock_num:       %d\n",
			   NVM_wake_lock_num);
		seq_printf(s, "NVM_wake_unlock_num:     %d\n",
			   NVM_wake_unlock_num);
		seq_printf(s, "rx_workq_sch_num:     %d\n", rx_workq_sch_num);
		seq_puts(s, "\nport  ");
		seq_puts(s, "tx_current  tx_request  tx_sent  tx_drop" \
			 "  tx_queue_max" \
			 "  rx_current  rx_indicate   rx_got  rx_queue_max" \
			 "  ap_throttle_cp  ap_unthrottle_cp" \
			 "  cp_throttle_ap  cp_unthrottle_ap\n");
	} else if (seq_flag == SEQ_DIRECT_RB) {
		struct direct_rbctl *dir_ctl;
		const struct direct_rbctl *dir_ctl_end =
		    direct_rbctl + direct_rb_type_total_cnt;
		seq_printf(s, "\n");
		seq_printf(s, "direct_rb:\n");
		for (dir_ctl = direct_rbctl; dir_ctl != dir_ctl_end;
		     ++dir_ctl) {
			seq_printf(s,
				   "tx_sent: %ld, tx_drop: %ld,"
				   " rx_fail: %ld, rx_got: %ld,"
				   " interrupt: %ld, broadcast_msg: %ld\n",
				   dir_ctl->stat_tx_sent, dir_ctl->stat_tx_drop,
				   dir_ctl->stat_rx_fail, dir_ctl->stat_rx_got,
				   dir_ctl->stat_interrupt,
				   dir_ctl->stat_broadcast_msg);
		}
	} else {
		struct portq *portq = (struct portq *)v;
		spin_lock(&portq->lock);
		seq_printf(s, "%4d", portq->port);
		seq_printf(s, "%12d", skb_queue_len(&portq->tx_q));
		seq_printf(s, "%12ld", portq->stat_tx_request);
		seq_printf(s, "%9ld", portq->stat_tx_sent);
		seq_printf(s, "%9ld", portq->stat_tx_drop);
		seq_printf(s, "%14ld", portq->stat_tx_queue_max);
		seq_printf(s, "%12d", skb_queue_len(&portq->rx_q));
		seq_printf(s, "%13ld", portq->stat_rx_indicate);
		seq_printf(s, "%9ld", portq->stat_rx_got);
		seq_printf(s, "%14ld", portq->stat_rx_queue_max);
		seq_printf(s, "%16ld", portq->stat_fc_ap_throttle_cp);
		seq_printf(s, "%18ld", portq->stat_fc_ap_unthrottle_cp);
		seq_printf(s, "%16ld", portq->stat_fc_cp_throttle_ap);
		seq_printf(s, "%18ld\n", portq->stat_fc_cp_unthrottle_ap);
		spin_unlock(&portq->lock);
	}

	return 0;
}

static void msocket_dump_port(void)
{
	struct portq *portq;
	int i;

	printk(KERN_ERR "shm_is_ap_xmit_stopped: %d\n",
	       portq_rbctl->is_ap_xmit_stopped);
	printk(KERN_ERR "shm_is_cp_xmit_stopped: %d\n",
	       portq_rbctl->is_cp_xmit_stopped);
	printk(KERN_ERR "acipc_ap_stopped_num:   %ld\n",
	       portq_rbctl->ap_stopped_num);
	printk(KERN_ERR "acipc_ap_resumed_num:   %ld\n",
	       portq_rbctl->ap_resumed_num);
	printk(KERN_ERR "acipc_cp_stopped_num:   %ld\n",
	       portq_rbctl->cp_stopped_num);
	printk(KERN_ERR "acipc_cp_resumed_num:   %ld\n",
	       portq_rbctl->cp_resumed_num);
	printk(KERN_ERR "tx_socket_total:        %d\n",
	       portq_rbctl->tx_skbuf_num);
	printk(KERN_ERR "tx_socket_free:         %d\n",
	       shm_free_tx_skbuf(portq_rbctl));
	printk(KERN_ERR "rx_socket_total:        %d\n",
	       portq_rbctl->rx_skbuf_num);
	printk(KERN_ERR "rx_socket_free:         %d\n",
	       shm_free_rx_skbuf(portq_rbctl));

	printk(KERN_ERR "NVM_wake_lock_num:      %d\n", NVM_wake_lock_num);
	printk(KERN_ERR "NVM_wake_unlock_num:    %d\n", NVM_wake_unlock_num);
	printk(KERN_ERR "\nport  tx_current  tx_request  tx_sent  tx_drop" \
	       "  tx_queue_max" \
	       "  rx_current  rx_indicate   rx_got  rx_queue_max" \
	       "  ap_throttle_cp  ap_unthrottle_cp" \
	       "  cp_throttle_ap  cp_unthrottle_ap\n");
	for (i = 1; i < 7; i++) {
		portq = portq_array[i];
		if (!portq)
			continue;
		printk(KERN_ERR
		       "%4d %12d %12ld %9ld %9ld %14ld %12d %13ld "
		       "%9ld %14ld %16ld %18ld %16ld %18ld\n",
		       portq->port, skb_queue_len(&portq->tx_q),
		       portq->stat_tx_request, portq->stat_tx_sent,
		       portq->stat_tx_drop, portq->stat_tx_queue_max,
		       skb_queue_len(&portq->rx_q), portq->stat_rx_indicate,
		       portq->stat_rx_got, portq->stat_rx_queue_max,
		       portq->stat_fc_ap_throttle_cp,
		       portq->stat_fc_ap_unthrottle_cp,
		       portq->stat_fc_cp_throttle_ap,
		       portq->stat_fc_cp_unthrottle_ap);
	}
}

/**
 * This structure gather "function" to manage the sequence
 *
 */
static const struct seq_operations msocket_seq_ops = {
	.start = msocket_seq_start,
	.next = msocket_seq_next,
	.stop = msocket_seq_stop,
	.show = msocket_seq_show
};

/**
 * This function is called when the /proc file is open.
 *
 */
static int msocket_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &msocket_seq_ops);
};

/**
 * This structure gather "function" that manage the /proc file
 *
 */
static const struct file_operations msocket_proc_fops = {
	.owner = THIS_MODULE,
	.open = msocket_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

/* open msocket */
static int msocket_open(struct inode *inode, struct file *filp)
{
	/*
	 * explicit set private_data to NULL, we'll use this pointer to
	 * associate file and portq
	 */
	filp->private_data = NULL;

	return 0;
}

/* open msocket */
static int msockdev_open(struct inode *inode, struct file *filp)
{
	struct portq *portq;
	int port;
	struct cmsockdev_dev *dev;	/* device information */

	dev = container_of(inode->i_cdev, struct cmsockdev_dev, cdev);
	/* Extract Minor Number */
	port = MINOR(dev->cdev.dev);

	if (port == DIAG_PORT) {
		if (direct_rb_open(direct_rb_type_diag, port)) {
			printk(KERN_INFO
			       "diag is opened by process id:%d (\"%s\")\n",
			       current->tgid, current->comm);
		} else {
			printk(KERN_INFO
			       "diag open fail by process id:%d (\"%s\")\n",
			       current->tgid, current->comm);
			return -1;
		}
	}
	portq = portq_open(port);
	if (IS_ERR(portq)) {
		printk(KERN_INFO "MSOCK: binding port %d error, %ld\n",
		       port, PTR_ERR(portq));
		return PTR_ERR(portq);
	} else {
		filp->private_data = portq;
		printk(KERN_INFO "MSOCK: binding port %d, success.\n", port);
		printk(KERN_INFO
		       "MSOCK: port %d is opened by process id:%d (\"%s\")\n",
		       port, current->tgid, current->comm);
		return 0;
	}
	return 0;
}

/* close msocket */
static int msocket_close(struct inode *inode, struct file *filp)
{
	struct portq *portq = filp->private_data;

	int port;
	struct cmsockdev_dev *dev;	/* device information */

	dev = container_of(inode->i_cdev, struct cmsockdev_dev, cdev);
	/* Extract Minor Number */
	port = MINOR(dev->cdev.dev);
	if (port == DIAG_PORT) {
		direct_rb_close(direct_rb_type_diag);
		printk(KERN_INFO
		       "%s diag rb is closed by process id:%d (\"%s\")\n",
		       __func__, current->tgid, current->comm);
	}
	if (portq) {		/* file already bind to portq */
		printk(KERN_INFO
		       "MSOCK: port %d is closed by process id:%d (\"%s\")\n",
		       portq->port, current->tgid, current->comm);
		portq_close(portq);
	}

	return 0;
}

/* read from msocket */
static ssize_t
msocket_read(struct file *filp, char __user *buf, size_t len, loff_t *f_pos)
{
	struct portq *portq;
	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	int rc = -EFAULT;

	portq = (struct portq *)filp->private_data;
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: port not bind.\n", __func__);
		return rc;
	}

	if (portq->port == DIAG_PORT)
		return direct_rb_recv(direct_rb_type_diag, buf, len);
	skb = portq_recv(portq, true);

	if (IS_ERR(skb)) {
		printk(KERN_DEBUG "MSOCK: %s: portq_recv returns %d\n",
		       __func__, (int)skb);
		return PTR_ERR(skb);
	}

	hdr = (struct shm_skhdr *)skb->data;
	if (hdr->length > len) {
		printk(KERN_ERR "MSOCK: %s: error: no enough space.\n",
		       __func__);
		goto err_exit;
	}

	if (copy_to_user(buf, skb_pull(skb, sizeof(*hdr)), hdr->length))
		printk(KERN_ERR "MSOCK: %s: copy_to_user failed.\n", __func__);
	else
		rc = hdr->length;

err_exit:
	kfree_skb(skb);
	return rc;
}

static unsigned int msocket_poll(struct file *filp, poll_table *wait)
{
	struct portq *portq;

	portq = (struct portq *)filp->private_data;

	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: port not bind.\n", __func__);
		return 0;
	}

	return portq_poll(portq, filp, wait);
}

/* write to msocket */
static ssize_t
msocket_write(struct file *filp, const char __user *buf, size_t len,
	      loff_t *f_pos)
{
	struct portq *portq;
	struct sk_buff *skb;
	struct shm_skhdr *hdr;
	int rc = -EFAULT;

	portq = (struct portq *)filp->private_data;
	if (!portq) {
		printk(KERN_ERR "MSOCK: %s: port not bind.\n", __func__);
		return rc;
	}

	if (portq->port == DIAG_PORT)
		return direct_rb_xmit(direct_rb_type_diag, buf, len);

	if (len > (portq_rbctl->tx_skbuf_size - sizeof(*hdr))) {
		printk(KERN_ERR "MSOCK: %s: port %d, len is %d!!\n",
		       __func__, portq->port, len);
		return rc;
	}

	skb = alloc_skb(len + sizeof(*hdr), GFP_KERNEL);
	if (!skb) {
		printk(KERN_ERR "MSOCK: %s: out of memory.\n", __func__);
		return -ENOMEM;
	}
	skb_reserve(skb, sizeof(*hdr));	/* reserve header space */

	if (copy_from_user(skb_put(skb, len), buf, len)) {
		kfree_skb(skb);
		printk(KERN_ERR "MSOCK: %s: copy_from_user failed.\n",
		       __func__);
		return rc;
	}

	skb_push(skb, sizeof(*hdr));
	hdr = (struct shm_skhdr *)skb->data;
	hdr->address = 0;
	hdr->port = portq->port;
	hdr->checksum = 0;
	hdr->length = len;

	if (!msocket_is_synced || portq_xmit(portq, skb, true) < 0) {
		kfree_skb(skb);
		printk(KERN_ERR "MSOCK: %s: portq xmit error.\n", __func__);
		return -1;
	}

	return len;
}

/*  the ioctl() implementation */
static long msocket_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long arg)
{
	struct portq *portq;
	int port, status;

	/*
	 * extract the type and number bitfields, and don't decode
	 * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	 */
	if (_IOC_TYPE(cmd) != MSOCKET_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > MSOCKET_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case MSOCKET_IOC_BIND:
		port = arg;

		if (port == DIAG_PORT) {
			if (direct_rb_open(direct_rb_type_diag, port)) {
				printk(KERN_INFO
				       "diag path is opened by process id:%d (\"%s\")\n",
				       current->tgid, current->comm);
			} else {
				printk(KERN_INFO
				       "diag path open fail by process id:%d (\"%s\")\n",
				       current->tgid, current->comm);
				return -1;
			}
		}
		portq = portq_open(port);
		if (IS_ERR(portq)) {
			printk(KERN_INFO "MSOCK: binding port %d error, %d\n",
			       port, (int)portq);
			return (int)portq;
		} else {
			filp->private_data = portq;
			printk(KERN_INFO "MSOCK: binding port %d, success.\n",
			       port);
			printk(KERN_INFO
			       "MSOCK: port %d is opened by process id:%d (\"%s\")\n",
			       port, current->tgid, current->comm);
			return 0;
		}

	case MSOCKET_IOC_UP:
		printk(KERN_INFO "MSOCK: MSOCKET_UP is received!\n");
		/*
		 * in the case AP initiative reset CP, AP will first
		 * make msocket linkdown then hold, so CP still can
		 * send packet to share memory in this interval
		 * cleanup share memory one more time in msocket linkup
		 */
		shm_rb_data_init(portq_rbctl);
		spin_lock(&msocket_sync_lock);
		msocket_recv_up_ioc = true;
		spin_unlock(&msocket_sync_lock);
		/* ensure completion cleared before start */
		INIT_COMPLETION(msocket_peer_sync);
		msocket_connect();
		if (wait_for_completion_timeout(&msocket_peer_sync, 5 * HZ) ==
		    0) {
			printk(KERN_INFO "MSOCK: sync with CP FAIL\n");
			return -1;
		}
		return 0;

	case MSOCKET_IOC_DOWN:
		printk(KERN_INFO "MSOCK: MSOCKET_DOWN is received!\n");
		msocket_dump_port();
		msocket_dump_direct_rb();
		msocket_disconnect();
		/* ok! the world's silent then notify the upper layer */
		portq_broadcase_msg(MsocketLinkdownProcId);
		data_path_link_down();
		direct_rb_broadcast_msg(MsocketLinkdownProcId);
		return 0;

	case MSOCKET_IOC_PMIC_QUERY:
		printk(KERN_INFO "MSOCK: MSOCKET_PMIC_QUERY is received!\n");
		status = shm_is_cp_pmic_master(portq_rbctl) ? 1 : 0;
		if (copy_to_user((void *)arg, &status, sizeof(int)))
			return -1;
		else
			return 0;

	case MSOCKET_IOC_CONNECT:
		printk(KERN_INFO "MSOCK: MSOCKET_IOC_CONNECT is received!\n");
		msocket_connect();
		return 0;

	case MSOCKET_IOC_RESET_CP_REQUEST:
		printk(KERN_INFO "MSOCK: MSOCKET_IOC_RESET_CP_REQUEST is received!\n");
		acipc_reset_cp_request();
		return 0;

	case MSOCKET_IOC_POWERBACKOFF_STATUS_NOTIFY:
		gPowerBackOffStatus = arg;
		printk("MSOCK: PowerBackOff status is %d\n", gPowerBackOffStatus);
		return 0;

	case MSOCKET_IOC_CP_POWER_MODE_NOTIFY:
		gCpPowerMode = arg;
		printk("MSOCK: cp power mode is %d\n", gCpPowerMode);
		if(!gCpPowerMode)
			__pm_relax(&cp_power_resume_wakeup);
		return 0;

	default:
		return -ENOTTY;
	}
}

/* driver methods */
static const struct file_operations msocket_fops = {
	.owner = THIS_MODULE,
	.open = msocket_open,
	.release = msocket_close,
	.read = msocket_read,
	.write = msocket_write,
	.unlocked_ioctl = msocket_ioctl
};

/* driver methods */
static const struct file_operations msockdev_fops = {
	.owner = THIS_MODULE,
	.open = msockdev_open,
	.release = msocket_close,
	.read = msocket_read,
	.write = msocket_write,
	.poll = msocket_poll,
	.unlocked_ioctl = msocket_ioctl
};

/* misc structure */
static struct miscdevice msocket_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msocket",
	.fops = &msocket_fops
};

static int msocketDump_open(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)dump_flag;
	return 0;
}

static int msocketDump_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t msocketDump_read(struct file *filp, char __user *buf,
				size_t len, loff_t *f_pos)
{
	char temp[256];
	int flag = dump_flag;

	sprintf(temp, "0x%08x", flag);
	if (copy_to_user(buf, (void *)&temp, strlen(temp) + 1)) {
		printk(KERN_ERR "MSOCKDUMP: %s: copy_to_user failed.\n",
		       __func__);
		return -EFAULT;
	}
	printk(KERN_INFO "msocketDump:get flag :%s\n", temp);
	/* return strlen(temp)+1; */
	return 0;
}

static ssize_t msocketDump_write(struct file *filp, const char __user *buf,
				 size_t len, loff_t *f_pos)
{
	char temp[256];
	int flag = 0;
	if (copy_from_user((void *)&temp, buf, len)) {
		printk(KERN_ERR "MSOCKDUMP: %s: copy_from_user failed.\n",
		       __func__);
		return -EFAULT;
	}
	if (kstrtoint(temp, 10, &flag) < 0) {
		printk(KERN_ERR "MSOCKDUMP: %s: kstrtoint error.\n",
			__func__);
		return -EFAULT;
	}
	printk(KERN_INFO "msocketDump:set flag :%08x\n", flag);
	dump_flag = flag;
	return len;
}

static const struct file_operations msocketDump_fops = {
	.owner = THIS_MODULE,
	.open = msocketDump_open,
	.release = msocketDump_close,
	.read = msocketDump_read,
	.write = msocketDump_write
};

static struct miscdevice msocketDump_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msocket_dump",
	.fops = &msocketDump_fops
};

static int cmsockdev_setup_cdev(struct cmsockdev_dev *dev, int index)
{
	int err = 0;
	int devno = MKDEV(cmsockdev_major, cmsockdev_minor + index);

	cdev_init(&dev->cdev, &msockdev_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding cmsockdev%d", err, index);
	return err;
}

/*@cmsockdev_added: the number of successfully added cmsockt devices.
*/
void cmsockdev_cleanup_module(int cmsockdev_added)
{
	int i;
	dev_t devno = MKDEV(cmsockdev_major, cmsockdev_minor);

	/* Get rid of our char dev entries */
	if (cmsockdev_devices) {
		for (i = 0; i < cmsockdev_added; i++) {
			cdev_del(&cmsockdev_devices[i].cdev);
			device_destroy(cmsockdev_class,
				       MKDEV(cmsockdev_major,
					     cmsockdev_minor + i));
		}
		kfree(cmsockdev_devices);
	}

	class_destroy(cmsockdev_class);

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, cmsockdev_nr_devs);

}

int cmsockdev_init_module(void)
{
	int result, i = 0;
	dev_t dev = 0;
	char name[256];

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (cmsockdev_major) {
		dev = MKDEV(cmsockdev_major, cmsockdev_minor);
		result =
		    register_chrdev_region(dev, cmsockdev_nr_devs, "cmsockdev");
	} else {
		result =
		    alloc_chrdev_region(&dev, cmsockdev_minor,
					cmsockdev_nr_devs, "cmsockdev");
		cmsockdev_major = MAJOR(dev);
	}

	if (result < 0) {
		printk(KERN_WARNING "cmsockdev: can't get major %d\n",
		       cmsockdev_major);
		return result;
	}

	/*
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	cmsockdev_devices =
	    kzalloc(cmsockdev_nr_devs * sizeof(struct cmsockdev_dev),
		    GFP_KERNEL);
	if (!cmsockdev_devices) {
		result = -ENOMEM;
		goto fail;
	}

	/* Initialize each device. */
	cmsockdev_class = class_create(THIS_MODULE, "cmsockdev");
	for (i = 0; i < cmsockdev_nr_devs; i++) {
		sprintf(name, "%s%d", "cmsockdev", cmsockdev_minor + i);
		device_create(cmsockdev_class, NULL,
			      MKDEV(cmsockdev_major, cmsockdev_minor + i), NULL,
			      name);
		result = cmsockdev_setup_cdev(&cmsockdev_devices[i], i);
		if (result < 0)
			goto fail;

	}

	/* At this point call the init function for any friend device */

	return 0;		/* succeed */

fail:
	cmsockdev_cleanup_module(i);

	return result;
}

static int px_suspend_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		printk("MSOCK: gCpPowerMode:%d,gPowerBackOffStatus:%d\n", gCpPowerMode, gPowerBackOffStatus);
		if (gCpPowerMode == 1 && (gPowerBackOffStatus != PowerBackOff_ON_CS && gPowerBackOffStatus != PowerBackOff_ON_CS_PS
			 && gPowerBackOffStatus != PowerBackOff_ON_RB_TEST && gPowerBackOffStatus != PowerBackOff_ON_PS_RB_TEST
			 && gPowerBackOffStatus != PowerBackOff_ON_CS_RB_TEST && gPowerBackOffStatus != PowerBackOff_ON_CS_PS_RB_TEST)) {
			__pm_wakeup_event(&cp_power_resume_wakeup, 1000 * 5);
			portq_send_msg(CISTUB_PORT, MsocketSuspendNotify);
		}
		break;
	case PM_POST_SUSPEND:
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block suspend_notifier = {
	.notifier_call = px_suspend_notifier_event,
};

static int reboot_notifier_func(struct notifier_block *this,
	unsigned long code, void *cmd)
{
	printk(KERN_INFO "reboot notifier, notify CP\n");
	printk(KERN_INFO "%s: APMU_DEBUG byte3 %02x\n", __func__,
	       __raw_readb(APMU_DEBUG + 3));
	acipc_reset_cp_request();
	return 0;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_func,
};

int shm_ch_init(const struct cpload_cp_addr *addr)
{
	int rc;

	if (shm_ch_inited)
		return 0;

	if (addr->aponly)
		return 0;

	/* share memory area init */
	rc = cp_shm_init(addr);
	if (rc < 0)
		return rc;

	/* port queue init */
	rc = portq_init();
	if (rc < 0)
		goto portq_err;

	/* data channel init */
	rc = data_path_init();
	if (rc < 0)
		goto dp_err;

	/* direct rb init */
	rc = direct_rb_init();
	if (rc < 0)
		goto direct_rb_err;
	/* acipc init */
	rc = acipc_init();
	if (rc < 0)
		goto acipc_err;

	/* register misc device */
	rc = misc_register(&msocket_dev);
	if (rc < 0)
		goto misc_err;

	rc = misc_register(&msocketDump_dev);
	if (rc < 0)
		goto msocketDump_err;

	rc = cmsockdev_init_module();
	if (rc < 0)
		goto cmsock_err;

	wakeup_source_init(&cp_power_resume_wakeup, "cp_power_resume_wakeup");
	if ((rc = register_pm_notifier(&suspend_notifier))) {
		goto suspendReg_err;
	}

	/* start msocket peer sync */
	msocket_connect();

	shm_ch_inited = 1;

#ifdef CONFIG_SSIPC_SUPPORT
	data_path_ready();
#endif

	return 0;

suspendReg_err:
	cmsockdev_cleanup_module(cmsockdev_nr_devs);
cmsock_err:
	misc_deregister(&msocketDump_dev);
msocketDump_err:
	misc_deregister(&msocket_dev);
misc_err:
	acipc_exit();
acipc_err:
	direct_rb_exit();
direct_rb_err:
	data_path_exit();
dp_err:
	portq_exit();
portq_err:
	cp_shm_exit();

	return rc;
}
EXPORT_SYMBOL(shm_ch_init);

void shm_ch_deinit(void)
{
	if (!shm_ch_inited)
		return;
	shm_ch_inited = 0;

#ifdef CONFIG_SSIPC_SUPPORT
	data_path_delete();
#endif

	/* reverse order of initialization */
	unregister_pm_notifier(&suspend_notifier);
	msocket_disconnect();
	cmsockdev_cleanup_module(cmsockdev_nr_devs);
	misc_deregister(&msocketDump_dev);
	misc_deregister(&msocket_dev);
	acipc_exit();
	direct_rb_exit();
	data_path_exit();
	portq_exit();
	cp_shm_exit();
}
EXPORT_SYMBOL(shm_ch_deinit);

#define APMU_DEBUG_BUS_PROTECTION (1 << 4)
/* module initialization */
static int __init msocket_init(void)
{
	u8 apmu_debug_byte3;

	/* init lock */
	spin_lock_init(&msocket_sync_lock);
	shm_rwlock_init();

	/* create proc file */
	proc_create(PROC_FILE_NAME, 0666, NULL, &msocket_proc_fops);

	shm_ch_inited = 0;

	/*enable bus protection*/
	apmu_debug_byte3 = __raw_readb(APMU_DEBUG + 3);
	apmu_debug_byte3 |= APMU_DEBUG_BUS_PROTECTION;
	__raw_writeb(apmu_debug_byte3, APMU_DEBUG + 3);
	register_reboot_notifier(&reboot_notifier);

	return 0;
}

/* module exit */
static void __exit msocket_exit(void)
{
	unregister_reboot_notifier(&reboot_notifier);
	remove_proc_entry(PROC_FILE_NAME, NULL);
	if (shm_ch_inited)
		shm_ch_deinit();
	shm_ch_inited = 0;
}

module_init(msocket_init);
module_exit(msocket_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell MSocket Driver");
