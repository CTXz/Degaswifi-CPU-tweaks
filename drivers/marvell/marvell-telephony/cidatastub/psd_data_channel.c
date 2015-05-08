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
#include <linux/netdevice.h>	/* dev_kfree_skb_any */
#include <linux/ip.h>
#include "common_datastub.h"
#include "data_channel_kernel.h"
#include "data_path.h"
#include "psd_data_channel.h"
#include "tel_trace.h"

#define IPV4_ACK_LENGTH_LIMIT 96
#define IPV6_ACK_LENGTH_LIMIT 128

struct pduhdr {
	__be16 length;
	__u8 reserved;
	__u8 offset;
	int cid;
} __packed;

struct data_path *psd_dp;

static bool ack_opt = true;
static int set_ack_opt(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_bool(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: %s psd ack optimization\n",
		__func__, ack_opt ? "enable" : "disable");

	return 0;
}

static struct kernel_param_ops ack_opt_param_ops = {
	.set = set_ack_opt,
	.get = param_get_bool,
};

module_param_cb(psd_ack_opt, &ack_opt_param_ops, &ack_opt, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_ack_opt, "enable/disable psd ack opt");

static bool data_drop = true;
static int set_data_drop(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_bool(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: %s psd data drop\n",
		__func__, data_drop ? "enable" : "disable");

	return 0;
}

static struct kernel_param_ops data_drop_param_ops = {
	.set = set_data_drop,
	.get = param_get_bool,
};

module_param_cb(psd_data_drop, &data_drop_param_ops,
	&data_drop, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_data_drop, "enable/disable psd data drop");


static bool ndev_fc;
static int set_ndev_fc(const char *val, const struct kernel_param *kp)
{
	int rv = param_set_bool(val, kp);

	if (rv)
		return rv;

	printk(KERN_INFO "%s: %s psd netdevice flow control\n",
		__func__, ndev_fc ? "enable" : "disable");

	return 0;
}

static struct kernel_param_ops ndev_fc_param_ops = {
	.set = set_ndev_fc,
	.get = param_get_bool,
};

module_param_cb(psd_ndev_fc, &ndev_fc_param_ops, &ndev_fc, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(psd_ndev_fc, "enable/disable psd netdevice flow control");

#define MMS_CID    1

static inline enum data_path_priority packet_priority(int cid,
	struct sk_buff *skb)
{
	const struct iphdr *iph;
	enum data_path_priority prio = dp_priority_default;

	if (!ack_opt)
		return dp_priority_high;

	if (cid == MMS_CID)
		prio = dp_priority_high;
	else {
		bool is_ack = false;

		iph = ip_hdr(skb);
		if (iph) {
			if (iph->version == 4) {
				if (skb->len <= IPV4_ACK_LENGTH_LIMIT)
					is_ack = true;
			} else if (iph->version == 6) {
				if (skb->len <= IPV6_ACK_LENGTH_LIMIT)
					is_ack = true;
			}

			if (is_ack)
					prio = dp_priority_high;
		}
	}

	return prio;
}

int sendPSDData(int cid, struct sk_buff *skb)
{
	struct pduhdr *hdr;
	struct sk_buff *skb2;
	unsigned len;
	unsigned tailpad;
	enum data_path_priority prio;

	DP_ENTER();

	/* data path is not open, drop the packet and return */
	if (!psd_dp) {
		DP_PRINT("%s: data path is not open!\n", __func__);
		dev_kfree_skb_any(skb);
		return PSD_DATA_SEND_DROP;
	}
	prio = packet_priority(cid, skb);

	/*
	 * tx_q is full or link is down
	 * always allow ack
	 */
	if (prio != dp_priority_high &&
		(data_path_is_tx_q_full(psd_dp) ||
		 !data_path_is_link_up())) {
		DP_PRINT("%s: tx_q is full or link is down\n", __func__);
		/* tx q is full, should schedule tx immediately */
		if (data_path_is_tx_q_full(psd_dp))
			data_path_schedule_tx(psd_dp);

		if (data_drop) {
			DP_PRINT("%s: drop the packet\n", __func__);
			dev_kfree_skb_any(skb);
			return PSD_DATA_SEND_DROP;
		} else {
			DP_PRINT("%s: return net busy to upper layer\n",
				__func__);
			return PSD_DATA_SEND_BUSY;
		}
	}

	len = skb->len;
	tailpad = padding_size(sizeof *hdr + len);

	if (likely(!skb_cloned(skb))) {
		int headroom = skb_headroom(skb);
		int tailroom = skb_tailroom(skb);

		/* enough room as-is? */
		if (likely(sizeof *hdr + tailpad <= headroom + tailroom)) {
			/* do not need to be readjusted */
			if (sizeof *hdr <= headroom && tailpad <= tailroom)
				goto fill;

			skb->data = memmove(skb->head + sizeof *hdr,
					    skb->data, len);
			skb_set_tail_pointer(skb, len);
			goto fill;
		}
	}

	/* create a new skb, with the correct size (and tailpad) */
	skb2 = skb_copy_expand(skb, sizeof *hdr, tailpad + 1, GFP_ATOMIC);
	if (skb2)
		trace_psd_xmit_skb_realloc(skb, skb2);
	dev_kfree_skb_any(skb);
	if (unlikely(!skb2))
		return PSD_DATA_SEND_BUSY;
	skb = skb2;

	/* fill out the pdu header */
fill:
	hdr = (void *)__skb_push(skb, sizeof *hdr);
	memset(hdr, 0, sizeof *hdr);
	hdr->length = cpu_to_be16(len);
	hdr->cid = cid;
	memset(skb_put(skb, tailpad), 0, tailpad);

	data_path_xmit(psd_dp, skb, prio);
	return PSD_DATA_SEND_OK;
}
EXPORT_SYMBOL(sendPSDData);

enum data_path_result psd_data_rx(unsigned char *data, unsigned int length)
{
	unsigned char *p = data;
	unsigned int remains = length;
	int ret = 0;
	DP_ENTER();

	while (remains > 0) {
		struct pduhdr	*hdr = (void *)p;
		u32				iplen, offset_len;
		u32				tailpad;

		iplen = be16_to_cpu(hdr->length);
		offset_len = hdr->offset;
		tailpad = padding_size(sizeof *hdr + iplen + offset_len);

		DP_PRINT("%s: remains, %d, iplen %ld, " \
			"offset %ld, cid %d, tailpad %d\n",
			__func__, remains, iplen, offset_len,
			hdr->cid, tailpad);

		if (unlikely(remains < (iplen + offset_len
					+ sizeof *hdr + tailpad))) {
			DP_ERROR("%s: packet length error\n", __func__);
			return dp_rx_packet_error;
		}

		/* offset domain data */
		p += sizeof *hdr;
		remains -= sizeof *hdr;

		/* ip payload */
		p += offset_len;
		remains -= offset_len;

		/*
		 * Since we need to distinguish the received packets for
		 * PPP or directly IP, so if PPP call back function registered,
		 * first forward the packet to PPP call back function check, if
		 * this packet is for PPP, the call back function will return 1,
		 * otherwise return 0
		 */
		if (dataRxCbFunc[PSD_MODEM])
			ret = dataRxCbFunc[PSD_MODEM] (p, iplen, hdr->cid);
		if (ret == 0) {
			if (dataRxCbFunc[PDP_DIRECTIP])
				dataRxCbFunc[PDP_DIRECTIP] (p, iplen, hdr->cid);
			else
				return dp_rx_packet_dropped;
		}

		p += iplen + tailpad;

		remains -= iplen + tailpad;
	}
	return dp_success;
}

void psd_tx_stop(void)
{
	int i;
	DP_ENTER();

	if (!ndev_fc)
		return;

	for (i = 0; i < SRV_MAX; ++i) {
		if (psdFCCbFunc[i])
			psdFCCbFunc[i](true);
	}

	DP_LEAVE();
	return;
}

void psd_tx_resume(void)
{
	int i;
	DP_ENTER();

	if (!ndev_fc)
		return;

	for (i = 0; i < SRV_MAX; ++i) {
		if (psdFCCbFunc[i])
			psdFCCbFunc[i](false);
	}

	DP_LEAVE();
	return;
}

void psd_rx_stop(void)
{
	DP_ENTER();

	DP_LEAVE();
	return;
}

void psd_link_down(void)
{
	DP_ENTER();

	DP_LEAVE();
	return;
}

void psd_link_up(void)
{
	DP_ENTER();
	DP_LEAVE();
	return;
}

struct data_path_callback psd_cbs = {
	.data_rx = psd_data_rx,
	.tx_stop = psd_tx_stop,
	.tx_resume = psd_tx_resume,
	.rx_stop = psd_rx_stop,
	.link_down = psd_link_down,
	.link_up = psd_link_up,
};

int InitPsdChannel(void)
{
	psd_dp = data_path_open(dp_type_psd, &psd_cbs);
	if (!psd_dp)
		return -1;

	return 0;
}

void DeInitPsdChannel(void)
{
	data_path_close(psd_dp);
}

#define CREATE_TRACE_POINTS
#include "tel_trace.h"

