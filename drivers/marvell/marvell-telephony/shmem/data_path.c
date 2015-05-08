/*
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

#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/version.h>
#include <linux/export.h>
#include <linux/netdevice.h>	/* dev_kfree_skb_any */
#include <linux/module.h>
#include "shm_share.h"
#include "acipcd.h"
#include "shm.h"
#include "msocket.h"
#include "data_path.h"
#include "tel_trace.h"


static struct wake_lock dp_rx_wakeup;
static struct wake_lock dp_tx_wakelock;

static struct data_path data_path[dp_type_total_cnt];

#ifdef CONFIG_SSIPC_SUPPORT
static void (*dp_ready_cb)(void);
static void (*dp_delete_cb)(void);
#endif

static enum shm_rb_type dp_rb[dp_type_total_cnt] = {
	shm_rb_psd
};

enum data_path_state {
	dp_state_idle,
	dp_state_opening,
	dp_state_opened,
	dp_state_closing,
};

/*
 * as we do tx/rx in interrupt context, we should avoid lock up the box
 */
/*
 * max tx shots
 */
static int max_tx_shots = 32;
static int set_max_tx_shots(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd max tx shots to %d\n",
		__func__, max_tx_shots);

	return 0;
}

static struct kernel_param_ops max_tx_shots_param_ops = {
	.set = set_max_tx_shots,
	.get = param_get_int,
};

module_param_cb(psd_max_tx_shots, &max_tx_shots_param_ops,
	&max_tx_shots, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_max_tx_shots, "psd max tx shots");

/*
 * max rx shots
 */
static int max_rx_shots = 32;
static int set_max_rx_shots(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd max rx shots to %d\n",
		__func__, max_rx_shots);

	return 0;
}

static struct kernel_param_ops max_rx_shots_param_ops = {
	.set = set_max_rx_shots,
	.get = param_get_int,
};

module_param_cb(psd_max_rx_shots, &max_rx_shots_param_ops,
	&max_rx_shots, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_max_rx_shots, "psd max rx shots");

/*
 * max tx queue length
 */
static int max_tx_q_len = 2048;
static int set_max_tx_q_len(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);
	struct data_path *dp;
	const struct data_path *dp_end = data_path + dp_type_total_cnt;

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd max tx queue length to %d\n",
		__func__, max_tx_q_len);

	for (dp = data_path; dp != dp_end; ++dp)
		dp->tx_q_max_len = max_tx_q_len;

	return 0;
}

static struct kernel_param_ops max_tx_q_len_param_ops = {
	.set = set_max_tx_q_len,
	.get = param_get_int,
};

module_param_cb(psd_max_tx_q_len, &max_tx_q_len_param_ops,
	&max_tx_q_len, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_max_tx_q_len, "psd max tx queue length");

/*
 * tx schedule delay
 */
static int tx_sched_delay_in_ms = 2;
static int set_tx_sched_delay_in_ms(const char *val,
	const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd tx schedule delay to %d\n",
		__func__, tx_sched_delay_in_ms);

	return 0;
}

static struct kernel_param_ops tx_sched_delay_in_ms_param_ops = {
	.set = set_tx_sched_delay_in_ms,
	.get = param_get_int,
};

module_param_cb(psd_tx_sched_delay_in_ms, &tx_sched_delay_in_ms_param_ops,
	&tx_sched_delay_in_ms, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_tx_sched_delay_in_ms, "psd tx schedule delay");

/*
 * tx q schedule length
 */
static int tx_q_min_sched_len;
static int set_tx_q_min_sched_len(const char *val,
	const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd tx q schedule length %d\n",
		__func__, tx_q_min_sched_len);

	return 0;
}

static struct kernel_param_ops tx_q_min_sched_len_param_ops = {
	.set = set_tx_q_min_sched_len,
	.get = param_get_int,
};

module_param_cb(psd_tx_q_min_sched_len, &tx_q_min_sched_len_param_ops,
	&tx_q_min_sched_len, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_tx_q_min_sched_len, "psd tx q schedule length");


static inline void tx_q_enqueue(struct data_path *dp, struct sk_buff *skb,
				enum data_path_priority prio)
{
	unsigned long flags;

	spin_lock_irqsave(&dp->tx_q_lock, flags);
	skb_queue_tail(&dp->tx_q[prio], skb);
	spin_unlock_irqrestore(&dp->tx_q_lock, flags);
}

static inline void tx_q_queue_head(struct data_path *dp, struct sk_buff *skb,
				enum data_path_priority prio)
{
	unsigned long flags;

	spin_lock_irqsave(&dp->tx_q_lock, flags);
	skb_queue_head(&dp->tx_q[prio], skb);
	spin_unlock_irqrestore(&dp->tx_q_lock, flags);
}

static inline struct sk_buff *tx_q_dequeue(struct data_path *dp, int *prio)
{
	struct sk_buff *skb = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dp->tx_q_lock, flags);
	for (i = 0; i < dp_priority_cnt; i++) {
		if (skb_queue_len(&dp->tx_q[i])) {
			skb = skb_dequeue(&dp->tx_q[i]);
			if (prio)
				*prio = i;
			break;
		}
	}
	spin_unlock_irqrestore(&dp->tx_q_lock, flags);

	return skb;
}

static inline struct sk_buff *tx_q_peek(struct data_path *dp, int *prio)
{
	struct sk_buff *skb = NULL;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dp->tx_q_lock, flags);
	for (i = 0; i < dp_priority_cnt; i++) {
		if (skb_queue_len(&dp->tx_q[i])) {
			skb = skb_peek(&dp->tx_q[i]);
			if (prio)
				*prio = i;
			break;
		}
	}
	spin_unlock_irqrestore(&dp->tx_q_lock, flags);

	return skb;
}

static inline void tx_q_init(struct data_path *dp)
{
	int i;
	for (i = 0; i < dp_priority_cnt; i++)
		skb_queue_head_init(&dp->tx_q[i]);
}

static inline void tx_q_clean(struct data_path *dp)
{
	int i;
	for (i = 0; i < dp_priority_cnt; i++)
		skb_queue_purge(&dp->tx_q[i]);
}

static int psd_tx_wm;
static int set_psd_tx_wm(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_int(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: set psd tx wm %d\n",
		__func__, tx_q_min_sched_len);

	data_path[dp_type_psd].tx_wm[dp_priority_default] = psd_tx_wm;
	data_path[dp_type_psd].tx_wm[dp_priority_low] = psd_tx_wm;

	return 0;
}

static int get_psd_tx_wm(char *buf, const struct kernel_param *kp)
{
	psd_tx_wm =
		data_path[dp_type_psd].tx_wm[dp_priority_default];

	return param_get_int(buf, kp);
}

static struct kernel_param_ops psd_tx_wm_param_ops = {
	.set = set_psd_tx_wm,
	.get = get_psd_tx_wm,
};

module_param_cb(psd_tx_wm, &psd_tx_wm_param_ops,
	&psd_tx_wm, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_tx_wm, "ps tx wm");

static bool psd_enable_piggyback = true;
static int set_psd_enable_piggyback(const char *val,
	const struct kernel_param *kp)
{
	int rv;

	rv = param_set_bool(val, kp);

	if (rv)
		return rv;

	data_path[dp_type_psd].enable_piggyback = psd_enable_piggyback;

	return 0;
}

static struct kernel_param_ops psd_enable_piggyback_param_ops = {
	.set = set_psd_enable_piggyback,
	.get = param_get_bool,
};

module_param_cb(psd_enable_piggyback, &psd_enable_piggyback_param_ops,
	&psd_enable_piggyback, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_enable_piggyback, "enable ps piggyback flag");

static inline bool has_enough_free_tx_slot(const struct data_path *dp,
	int free_slots, int prio)
{
	return free_slots > dp->tx_wm[prio];
}

static inline int tx_q_avail_length(struct data_path *dp, int free_slots)
{
	int len = 0;
	int i;

	for (i = 0; i < dp_priority_cnt; i++)
		if (has_enough_free_tx_slot(dp, free_slots, i))
			len += skb_queue_len(&dp->tx_q[i]);

	return len;
}

static void data_path_tx_func(unsigned long arg)
{
	struct data_path *dp = (struct data_path *)arg;
	struct shm_rbctl *rbctl = dp->rbctl;
	struct shm_skctl *skctl = rbctl->skctl_va;
	struct shm_psd_skhdr *skhdr;
	struct sk_buff *packet;
	int slot = 0;
	int pending_slot;
	int free_slots;
	int prio;
	int remain_bytes;
	int used_bytes;
	int start_slot = skctl->ap_wptr;
	int consumed_slot = 0;
	int consumed_packets = 0;

	pending_slot = -1;
	remain_bytes = rbctl->tx_skbuf_size - sizeof(struct shm_psd_skhdr);
	used_bytes = 0;

	wake_lock(&dp_tx_wakelock);

	DP_ENTER();

	while (consumed_slot < max_tx_shots) {
		if (!msocket_is_synced)
			break;

		if (shm_is_xmit_full(rbctl)) {
			/*
			 * notify cp only if we still have packets in queue
			 * otherwise, simply break
			 */
			if (tx_q_length(dp))
				shm_notify_ap_tx_stopped(rbctl);
			break;
		}

		free_slots = shm_free_tx_skbuf(rbctl);
		/* the only left slot is our pending slot */
		if (free_slots == 1 && pending_slot != -1) {
			/*
			 * check if we still have enough space in this
			 * pending slot
			 */
			packet = tx_q_peek(dp, NULL);
			if (!packet)
				break;

			/* packet is too large, notify cp and break */
			if (padded_size(packet->len) > remain_bytes) {
				shm_notify_ap_tx_stopped(rbctl);
				break;
			}
		}

		packet = tx_q_dequeue(dp, &prio);

		if (!packet) {
			DP_PRINT("%s: no packet available\n", __func__);
			break;
		}

		if (packet->len + sizeof *skhdr > rbctl->tx_skbuf_size) {
			DP_ERROR("%s: packet too large %d\n", __func__,
				 packet->len);
			dev_kfree_skb_any(packet);
			continue;
		}

		/* push to ring buffer */

		/* we have one slot pending */
		if (pending_slot != -1) {
			/*
			 * the packet is too large for the pending slot
			 * send out the pending slot firstly
			 */
			if (padded_size(packet->len) > remain_bytes) {
				skctl->ap_wptr = pending_slot;
				pending_slot = -1;
				consumed_slot++;
			} else
				slot = pending_slot;
		}

		/*
		 * each priority has one hard limit to guarantee higher priority
		 * packet is not affected by lower priority packet
		 * if we reach this limit, we can only send higher priority
		 * packets
		 * but in the other hand, if this packet can be filled into our
		 * pending slot, allow it anyway
		 */
		if (!has_enough_free_tx_slot(dp, free_slots, prio) &&
			((pending_slot == -1) || !dp->enable_piggyback)) {
			/* push back the packets and schedule delayed tx */
			tx_q_queue_head(dp, packet, prio);
			__data_path_schedule_tx(dp, true);
			break;
		}

		/* get a new slot from ring buffer */
		if (pending_slot == -1) {
			slot = shm_get_next_tx_slot(dp->rbctl, skctl->ap_wptr);

			remain_bytes =
				rbctl->tx_skbuf_size
				- sizeof(struct shm_psd_skhdr);
			used_bytes = 0;

			pending_slot = slot;
		}

		consumed_packets++;

		skhdr = (struct shm_psd_skhdr *)
			SHM_PACKET_PTR(rbctl->tx_va,
				slot,
				rbctl->tx_skbuf_size);

		/* we are sure our remains is enough for current packet */
		skhdr->length = used_bytes + padded_size(packet->len);
		memcpy((unsigned char *)(skhdr + 1) + used_bytes,
			packet->data, packet->len);

		used_bytes += padded_size(packet->len);
		remain_bytes -= padded_size(packet->len);

		DP_PRINT("%s: xmit to shm with length %d\n",
			__func__, packet->len);

		trace_psd_xmit(packet, slot);

		dev_kfree_skb_any(packet);
	}

	/* send out the pending slot */
	if (pending_slot != -1) {
		skctl->ap_wptr = pending_slot;
		pending_slot = -1;
		consumed_slot++;
	}

	if (consumed_slot > 0) {
		trace_psd_xmit_irq(start_slot, consumed_slot);
		shm_notify_packet_sent(rbctl);
	}

	if (consumed_slot >= max_tx_shots)
		data_path_schedule_tx(dp);

	/*
	 * ring buffer is stopped, just notify upper layer
	 * do not need to check is_tx_stopped here, as we need to handle
	 * following situation:
	 * a new on-demand PDP is activated after tx_stop is called
	 */
	if (rbctl->is_ap_xmit_stopped) {
		if (!dp->is_tx_stopped)
			DP_ERROR("%s tx stop\n", __func__);

		dp->is_tx_stopped = true;

		/* notify upper layer tx stopped */
		if (dp->cbs->tx_stop)
			dp->cbs->tx_stop();

		/* reschedule tx to polling the ring buffer */
		if (tx_q_length(dp))
			__data_path_schedule_tx(dp, true);
	}

	/*
	 * ring buffer is resumed and the remain packets
	 * in queue is also sent out
	 */
	if (!rbctl->is_ap_xmit_stopped && dp->is_tx_stopped
		&& tx_q_length(dp) == 0) {
		DP_ERROR("%s tx resume\n", __func__);

		/* notify upper layer tx resumed */
		if (dp->cbs->tx_resume)
			dp->cbs->tx_resume();

		dp->is_tx_stopped = false;
	}

	wake_unlock(&dp_tx_wakelock);
	DP_LEAVE();
}

static void data_path_rx_func(unsigned long arg)
{
	struct data_path *dp = (struct data_path *)arg;
	struct shm_rbctl *rbctl = dp->rbctl;
	struct shm_skctl *skctl = rbctl->skctl_va;
	struct shm_psd_skhdr *skhdr;
	int slot;
	int count;
	enum data_path_result result;
	int i;

	DP_ENTER();

	for (i = 0; i < max_rx_shots; i++) {
		if (!msocket_is_synced) {
			/* if not sync, just return */
			break;
		}

		/* process share memory socket buffer flow control */
		if (rbctl->is_cp_xmit_stopped
		    && shm_has_enough_free_rx_skbuf(rbctl)) {
			shm_notify_cp_tx_resume(rbctl);
		}

		if (shm_is_recv_empty(rbctl))
			break;

		slot = shm_get_next_rx_slot(rbctl, skctl->ap_rptr);

		skhdr =
		    (struct shm_psd_skhdr *)SHM_PACKET_PTR(rbctl->rx_va, slot,
							   rbctl->
							   rx_skbuf_size);

		count = skhdr->length + sizeof(*skhdr);

		if (count > rbctl->rx_skbuf_size) {
			DP_ERROR(KERN_EMERG
				 "%s: slot = %d, count = %d\n", __func__, slot,
				 count);
			goto error_length;
		}

		DP_PRINT("%s: recv from shm with length %d\n", __func__,
			 skhdr->length);

		trace_psd_recv(slot);

		if (dp->cbs && dp->cbs->data_rx)
			result =
			    dp->cbs->data_rx((unsigned char *)(skhdr + 1),
					     skhdr->length);
		else
			result = dp_success;

		DP_PRINT("%s: result of data_rx: %d\n", __func__, result);

		/*
		 * upper layer decide to keep the packet as pending
		 * and we need to return now
		 */
		if (result == dp_rx_keep_pending) {
			DP_ERROR("%s: packet is pending\n", __func__);
			break;
		}
error_length:
		skctl->ap_rptr = slot;
	}

	if (i == max_rx_shots)
		data_path_schedule_rx(dp);
	else
		wake_unlock(&acipc_wakeup);

	DP_LEAVE();
}

static void tx_sched_timeout(unsigned long data)
{
	struct data_path *dp = (struct data_path *)data;

	if (dp && atomic_read(&dp->state) == dp_state_opened)
		tasklet_schedule(&dp->tx_tl);
}

/*
 * force_delay: delay the schedule forcibly, for the high watermark case
 */
void __data_path_schedule_tx(struct data_path *dp, bool force_delay)
{
	DP_ENTER();

	if (dp && atomic_read(&dp->state) == dp_state_opened) {
		int free_slots = shm_free_tx_skbuf(dp->rbctl);
		int len = tx_q_avail_length(dp, free_slots);

		/*
		 * ok, we have enough packet in queue, fire the work immediately
		 */
		if (!force_delay && len > tx_q_min_sched_len) {
			tasklet_schedule(&dp->tx_tl);
			del_timer(&dp->tx_sched_timer);
		} else {
			if (!timer_pending(&dp->tx_sched_timer)) {
				unsigned long expires = jiffies +
					msecs_to_jiffies(tx_sched_delay_in_ms);
				mod_timer(&dp->tx_sched_timer, expires);
			}
		}
	}

	DP_LEAVE();
}

void data_path_schedule_tx(struct data_path *dp)
{
	__data_path_schedule_tx(dp, false);
}
EXPORT_SYMBOL(data_path_schedule_tx);

void data_path_schedule_rx(struct data_path *dp)
{
	DP_ENTER();

	if (dp && atomic_read(&dp->state) == dp_state_opened)
		tasklet_schedule(&dp->rx_tl);

	DP_LEAVE();
}
EXPORT_SYMBOL(data_path_schedule_rx);

enum data_path_result data_path_xmit(struct data_path *dp,
				     struct sk_buff *skb,
				     enum data_path_priority prio)
{
	enum data_path_result ret = dp_not_open;
	DP_ENTER();

	if (dp && atomic_read(&dp->state) == dp_state_opened) {
		tx_q_enqueue(dp, skb, prio);
		data_path_schedule_tx(dp);
		ret = dp_success;
	}

	DP_LEAVE();
	return ret;
}
EXPORT_SYMBOL(data_path_xmit);

void data_path_broadcast_msg(int proc)
{
	struct data_path *dp;
	const struct data_path *dp_end = data_path + dp_type_total_cnt;

	DP_ENTER();

	for (dp = data_path; dp != dp_end; ++dp) {
		if (atomic_read(&dp->state) == dp_state_opened) {
			if (proc == MsocketLinkdownProcId) {
				spin_lock_irq(&dp->tx_q_lock);
				tx_q_clean(dp);
				spin_unlock_irq(&dp->tx_q_lock);

				if (dp->cbs && dp->cbs->link_down)
					dp->cbs->link_down();
			} else if (proc == MsocketLinkupProcId) {
				/*
				 * Now both AP and CP will not send packet
				 * to ring buffer or receive packet from ring
				 * buffer, so cleanup any packetin ring buffer
				 * and initialize some key data structure to
				 * the beginning state otherwise user space
				 * process and CP may occur error
				 */
				shm_rb_data_init(dp->rbctl);
				if (dp->cbs && dp->cbs->link_up)
					dp->cbs->link_up();
			}
		}
	}

	DP_LEAVE();
}

#ifdef CONFIG_SSIPC_SUPPORT
void dp_ready_cb_regist(void *ready_cb, void *delete_cb)
{
	DP_ENTER();
	dp_ready_cb = ready_cb;
	dp_delete_cb = delete_cb;
	DP_LEAVE();
}
EXPORT_SYMBOL(dp_ready_cb_regist);
#endif

struct data_path *data_path_open(enum data_path_type dp_type,
				 struct data_path_callback *cbs)
{
	DP_ENTER();

	if (dp_type >= dp_type_total_cnt || dp_type < 0) {
		DP_ERROR("%s: incorrect type %d\n", __func__, dp_type);
		return NULL;
	}

	if (!cbs) {
		DP_ERROR("%s: cbs is NULL\n", __func__);
		return NULL;
	}

	if (atomic_cmpxchg(&data_path[dp_type].state, dp_state_idle,
			   dp_state_opening) != dp_state_idle) {
		DP_ERROR("%s: path is already opened(state %d)\n",
			 __func__, atomic_read(&data_path[dp_type].state));
		return NULL;
	}

	data_path[dp_type].tx_q_max_len = max_tx_q_len;
	data_path[dp_type].is_tx_stopped = false;
	spin_lock_init(&data_path[dp_type].tx_q_lock);
	tx_q_init(&data_path[dp_type]);
	data_path[dp_type].tx_wm[dp_priority_high] = 0;
	data_path[dp_type].tx_wm[dp_priority_default]
		= data_path[dp_type].rbctl->tx_skbuf_num / 10;
	data_path[dp_type].tx_wm[dp_priority_low]
		= data_path[dp_type].rbctl->tx_skbuf_num / 10;

	data_path[dp_type].enable_piggyback = psd_enable_piggyback;

	data_path[dp_type].cbs = cbs;
	tasklet_init(&data_path[dp_type].tx_tl, data_path_tx_func,
		     (unsigned long)&data_path[dp_type]);
	tasklet_init(&data_path[dp_type].rx_tl, data_path_rx_func,
		     (unsigned long)&data_path[dp_type]);

	init_timer(&data_path[dp_type].tx_sched_timer);
	data_path[dp_type].tx_sched_timer.function = tx_sched_timeout;
	data_path[dp_type].tx_sched_timer.data =
		(unsigned long)&data_path[dp_type];

	atomic_set(&data_path[dp_type].state, dp_state_opened);

	DP_LEAVE();

	return &data_path[dp_type];
}
EXPORT_SYMBOL(data_path_open);

void data_path_close(struct data_path *dp)
{
	DP_ENTER();

	if (!dp) {
		DP_ERROR("%s: empty data channel\n", __func__);
		return;
	}

	if (atomic_cmpxchg(&dp->state, dp_state_opened,
			   dp_state_closing) != dp_state_opened) {
		DP_ERROR("%s: path is already opened(state %d)\n",
			 __func__, atomic_read(&dp->state));
		return;
	}

	dp->tx_q_max_len = 0;
	dp->is_tx_stopped = false;
	tx_q_clean(dp);

	del_timer_sync(&dp->tx_sched_timer);

	tasklet_kill(&dp->tx_tl);
	tasklet_kill(&dp->rx_tl);
	dp->cbs = NULL;

	atomic_set(&dp->state, dp_state_idle);

	DP_LEAVE();
}
EXPORT_SYMBOL(data_path_close);

void dp_rb_stop_cb(struct shm_rbctl *rbctl)
{
	struct data_path *dp;

	DP_ENTER();

	if (!rbctl)
		return;

	dp = rbctl->priv;

	wake_lock_timeout(&acipc_wakeup, HZ * 5);
	printk(KERN_WARNING "MSOCK: dp_rb_stop_cb!!!\n");

	if (dp && (atomic_read(&dp->state) == dp_state_opened)) {
		if (dp->cbs && dp->cbs->rx_stop)
			dp->cbs->rx_stop();
		data_path_schedule_rx(dp);
	}

	DP_LEAVE();
}

void dp_rb_resume_cb(struct shm_rbctl *rbctl)
{
	struct data_path *dp;

	DP_ENTER();

	if (!rbctl)
		return;

	dp = rbctl->priv;

	wake_lock_timeout(&acipc_wakeup, HZ * 2);
	printk(KERN_WARNING "MSOCK: dp_rb_resume_cb!!!\n");

	if (dp && (atomic_read(&dp->state) == dp_state_opened)) {
		/* do not need to check queue length,
		 * as we need to resume upper layer in tx_func */
		data_path_schedule_tx(dp);
	}

	DP_LEAVE();
}

void dp_packet_send_cb(struct shm_rbctl *rbctl)
{
	struct data_path *dp;
	struct shm_skctl *skctl;

	DP_ENTER();

	if (!rbctl)
		return;

	skctl = rbctl->skctl_va;
	dp = rbctl->priv;

	wake_lock_timeout(&acipc_wakeup, HZ * 5);

	trace_psd_recv_irq(skctl->cp_wptr);

	data_path_schedule_rx(dp);

	DP_LEAVE();
}

struct shm_callback dp_shm_cb = {
	.packet_send_cb  = dp_packet_send_cb,
	.rb_stop_cb      = dp_rb_stop_cb,
	.rb_resume_cb    = dp_rb_resume_cb,
};

#ifdef CONFIG_SSIPC_SUPPORT
void data_path_ready(void)
{
	if (dp_ready_cb)
		dp_ready_cb();
}

void data_path_delete(void)
{
	if (dp_delete_cb)
		dp_delete_cb();
}
#endif

int data_path_init(void)
{
	struct data_path *dp;
	struct data_path *dp2;
	const struct data_path *dp_end = data_path + dp_type_total_cnt;

	wake_lock_init(&dp_tx_wakelock, WAKE_LOCK_SUSPEND, "dp_tx_wakeups");
	wake_lock_init(&dp_rx_wakeup, WAKE_LOCK_SUSPEND, "dp_rx_wakeups");

	for (dp = data_path; dp != dp_end; ++dp) {
		dp->dp_type = dp - data_path;
		dp->rbctl = shm_open(dp_rb[dp - data_path], &dp_shm_cb, dp);
		if (!dp->rbctl) {
			DP_ERROR("%s: cannot open shm\n", __func__);
			goto exit;
		}
		atomic_set(&dp->state, dp_state_idle);
	}

	return 0;

exit:
	for (dp2 = data_path; dp2 != dp; ++dp2)
		shm_close(dp2->rbctl);

	wake_lock_destroy(&dp_tx_wakelock);
	wake_lock_destroy(&dp_rx_wakeup);

	return -1;
}

void data_path_exit(void)
{
	struct data_path *dp;
	const struct data_path *dp_end = data_path + dp_type_total_cnt;

	for (dp = data_path; dp != dp_end; ++dp)
		shm_close(dp->rbctl);

	wake_lock_destroy(&dp_tx_wakelock);
	wake_lock_destroy(&dp_rx_wakeup);
}

#define CREATE_TRACE_POINTS
#include "tel_trace.h"

