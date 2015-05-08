/*
    acipcd.c Created on: Aug 3, 2010, Jinhua Huang <jhhuang@marvell.com>

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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/rwlock.h>
#include <mach/cputype.h>
#ifdef CONFIG_DDR_DEVFREQ
#include <linux/devfreq.h>
#include <linux/platform_data/devfreq-pxa.h>
#include <linux/pm_qos.h>
#endif

#include "acipcd.h"
#include "shm.h"
#include "portqueue.h"
#include "msocket.h"
#include "pxa_cp_load.h"

struct wake_lock acipc_wakeup;	/* used to ensure Workqueue scheduled. */
/* forward static function prototype, these all are interrupt call-backs */
static u32 acipc_cb_rb_stop(u32 status);
static u32 acipc_cb_rb_resume(u32 status);
static u32 acipc_cb_port_fc(u32 status);
static u32 acipc_cb_psd_rb_stop(u32 status);
static u32 acipc_cb_psd_rb_resume(u32 status);
static u32 acipc_cb_psd_cb(u32 status);
static u32 acipc_cb(u32 status);
#ifdef CONFIG_DDR_DEVFREQ
static u32 acipc_cb_modem_ddrfreq_update(u32 status);
#endif
static u32 acipc_cb_diag_cb(u32 status);

static u32 acipc_cb_event_notify(u32 status);
#define RESET_CP_REQUEST 0x544F4F42
#define RESET_CP_REQUEST_DONE 0x454E4F44
DECLARE_COMPLETION(reset_cp_confirm);

#ifdef CONFIG_DDR_DEVFREQ
static struct pm_qos_request modem_ddr_cons = {
	.name = "cp",
};
struct workqueue_struct *acipc_wq;
struct work_struct acipc_modem_ddr_freq_update;

#define MODEM_DDRFREQ_HI	400
#define MODEM_DDRFREQ_MID	312
#define MODEM_DDRFREQ_LOW	156
static void acipc_modem_ddr_freq_update_handler(struct work_struct *work)
{
	static int cur_ddrfreq;

	printk(KERN_INFO "acipc_cb_modem_ddrfreq_update: %d\n",
	       (unsigned int)shm_rbctl[shm_rb_main].skctl_va->modem_ddrfreq);
	if ((unsigned int)shm_rbctl[shm_rb_main].skctl_va->modem_ddrfreq ==
	    MODEM_DDRFREQ_HI) {
		if (cur_ddrfreq == MODEM_DDRFREQ_HI)
			return;
		printk(KERN_INFO "DDRFreq set to High\n");
		pm_qos_update_request(&modem_ddr_cons, DDR_CONSTRAINT_LVL2);
		cur_ddrfreq = MODEM_DDRFREQ_HI;
	}

	if ((unsigned int)shm_rbctl[shm_rb_main].skctl_va->modem_ddrfreq ==
	    MODEM_DDRFREQ_MID) {
		if (cur_ddrfreq == MODEM_DDRFREQ_MID)
			return;
		printk(KERN_INFO "DDRFreq set to Mid\n");
		pm_qos_update_request(&modem_ddr_cons, DDR_CONSTRAINT_LVL1);
		cur_ddrfreq = MODEM_DDRFREQ_MID;
	}

	if ((unsigned int)shm_rbctl[shm_rb_main].skctl_va->modem_ddrfreq ==
	    MODEM_DDRFREQ_LOW) {
		if (cur_ddrfreq == MODEM_DDRFREQ_LOW)
			return;
		printk(KERN_INFO "DDRFreq set to Low\n");
		pm_qos_update_request(&modem_ddr_cons, PM_QOS_DEFAULT_VALUE);
		cur_ddrfreq = MODEM_DDRFREQ_LOW;
	}
	return;
}
#endif

/* acipc_init is used to register interrupt call-back function */
int acipc_init(void)
{
	wake_lock_init(&acipc_wakeup, WAKE_LOCK_SUSPEND, "acipc_wakeup");

	/* we do not check any return value */
	ACIPCEventBind(ACIPC_MUDP_KEY, acipc_cb, ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_RINGBUF_TX_STOP, acipc_cb_rb_stop,
		       ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_RINGBUF_TX_RESUME, acipc_cb_rb_resume,
		       ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_PORT_FLOWCONTROL, acipc_cb_port_fc,
		       ACIPC_CB_NORMAL, NULL);

	ACIPCEventBind(ACIPC_RINGBUF_PSD_TX_STOP, acipc_cb_psd_rb_stop,
		       ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_RINGBUF_PSD_TX_RESUME, acipc_cb_psd_rb_resume,
		       ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_SHM_PSD_PACKET_NOTIFY, acipc_cb_psd_cb,
		       ACIPC_CB_NORMAL, NULL);
	ACIPCEventBind(ACIPC_SHM_DIAG_PACKET_NOTIFY, acipc_cb_diag_cb,
		       ACIPC_CB_NORMAL, NULL);

	ACIPCEventBind(ACIPC_MODEM_DDR_UPDATE_REQ, acipc_cb_event_notify,
		       ACIPC_CB_NORMAL, NULL);

#ifdef CONFIG_DDR_DEVFREQ
	pm_qos_add_request(&modem_ddr_cons, PM_QOS_DDR_DEVFREQ_MIN,
		PM_QOS_DEFAULT_VALUE);
	INIT_WORK(&acipc_modem_ddr_freq_update,
		acipc_modem_ddr_freq_update_handler);
	acipc_wq = alloc_workqueue("ACIPC_WQ", WQ_HIGHPRI, 0);
#endif

	return 0;
}

/* acipc_exit used to unregister interrupt call-back function */
void acipc_exit(void)
{
	ACIPCEventUnBind(ACIPC_SHM_PSD_PACKET_NOTIFY);
	ACIPCEventUnBind(ACIPC_RINGBUF_PSD_TX_RESUME);
	ACIPCEventUnBind(ACIPC_RINGBUF_PSD_TX_STOP);
	ACIPCEventUnBind(ACIPC_PORT_FLOWCONTROL);
	ACIPCEventUnBind(ACIPC_RINGBUF_TX_RESUME);
	ACIPCEventUnBind(ACIPC_RINGBUF_TX_STOP);
	ACIPCEventUnBind(ACIPC_MUDP_KEY);

	wake_lock_destroy(&acipc_wakeup);

#ifdef CONFIG_DDR_DEVFREQ
	destroy_workqueue(acipc_wq);
	pm_qos_remove_request(&modem_ddr_cons);
#endif
}

/* cp xmit stopped notify interrupt */
static u32 acipc_cb_rb_stop(u32 status)
{
	return shm_rb_stop_cb(&shm_rbctl[shm_rb_main]);
}

/* cp wakeup ap xmit interrupt */
static u32 acipc_cb_rb_resume(u32 status)
{
	return shm_rb_resume_cb(&shm_rbctl[shm_rb_main]);
}

/* cp notify ap port flow control */
static u32 acipc_cb_port_fc(u32 status)
{
	return shm_port_fc_cb(&shm_rbctl[shm_rb_main]);
}

/* cp psd xmit stopped notify interrupt */
static u32 acipc_cb_psd_rb_stop(u32 status)
{
	return shm_rb_stop_cb(&shm_rbctl[shm_rb_psd]);
}

/* cp psd wakeup ap xmit interrupt */
static u32 acipc_cb_psd_rb_resume(u32 status)
{
	return shm_rb_resume_cb(&shm_rbctl[shm_rb_psd]);
}

/* psd new packet arrival interrupt */
static u32 acipc_cb_psd_cb(u32 status)
{
	return shm_packet_send_cb(&shm_rbctl[shm_rb_psd]);
}

/* diag new packet arrival interrupt */
static u32 acipc_cb_diag_cb(u32 status)
{
	return shm_packet_send_cb(&shm_rbctl[shm_rb_diag]);
}

/* new packet arrival interrupt */
static u32 acipc_cb(u32 status)
{
	u32 data;
	u16 event;

	ACIPCDataRead(&data);
	event = (data & 0xFF00) >> 8;

	switch (event) {
	case PACKET_SENT:
		shm_packet_send_cb(&shm_rbctl[shm_rb_main]);
		break;

	case PEER_SYNC:
		shm_peer_sync_cb(&shm_rbctl[shm_rb_main]);
		break;

	default:
		break;
	}

	return 0;
}

#ifdef CONFIG_DDR_DEVFREQ
static u32 acipc_cb_modem_ddrfreq_update(u32 status)
{
	queue_work(acipc_wq, &acipc_modem_ddr_freq_update);

	return 0;
}
#endif

static u32 acipc_cb_reset_cp_confirm(u32 status)
{
	if (shm_rbctl[shm_rb_main].skctl_va->reset_request
		== RESET_CP_REQUEST_DONE)
		complete(&reset_cp_confirm);

	shm_rbctl[shm_rb_main].skctl_va->reset_request = 0;
	return 0;
}

static u32 acipc_cb_event_notify(u32 status)
{
	acipc_cb_reset_cp_confirm(status);
#ifdef CONFIG_DDR_DEVFREQ
	acipc_cb_modem_ddrfreq_update(status);
#endif
	return 0;
}

void acipc_reset_cp_request(void)
{
	struct shm_rbctl *rbctl;

	rbctl = &shm_rbctl[shm_rb_main];
	read_lock(&rbctl->va_rwlock);
	if (!rbctl->skctl_va) {
		read_unlock(&rbctl->va_rwlock);
		return;
	}
	shm_rbctl[shm_rb_main].skctl_va->reset_request = RESET_CP_REQUEST;
	INIT_COMPLETION(reset_cp_confirm);
	acipc_notify_reset_cp_request();
	read_unlock(&rbctl->va_rwlock);
	if (wait_for_completion_timeout(&reset_cp_confirm, 2 * HZ))
		printk(KERN_INFO "reset cp request success!\n");
	else
		printk(KERN_ERR "reset cp request fail!\n");

	read_lock(&rbctl->va_rwlock);
	if (!rbctl->skctl_va) {
		read_unlock(&rbctl->va_rwlock);
		return;
	}
	shm_rbctl[shm_rb_main].skctl_va->reset_request = 0;
	read_unlock(&rbctl->va_rwlock);
	return;
}

