/*
 * Copyright (C) 2011 Samsung Electronics.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <net/ip.h>
#include "shm_share.h"
#include "modem_utils.h"

#define SSIPC_START_PROC_ID     0x06
#define SSIPC_DATA_PROC_ID      0x04


/* dump2hex
 * dump data to hex as fast as possible.
 * the length of @buf must be greater than "@len * 3"
 * it need 3 bytes per one data byte to print.
 */
static inline int dump2hex(char *buf, const char *data, size_t len)
{
	static const char *hex = "0123456789abcdef";
	char *dest = buf;
	int i;

	for (i = 0; i < len; i++) {
		*dest++ = hex[(data[i] >> 4) & 0xf];
		*dest++ = hex[data[i] & 0xf];
		*dest++ = ' ';
	}
	if (likely(len > 0))
		dest--; /* last space will be overwrited with null */

	*dest = '\0';

	return dest - buf;
}

int pr_ipc(const char *str, const char *data, size_t len)
{
	struct timeval tv;
	struct tm date;
	unsigned char hexstr[128];

	do_gettimeofday(&tv);
	time_to_tm((tv.tv_sec - sys_tz.tz_minuteswest * 60), 0, &date);

	dump2hex(hexstr, data, (len > 40 ? 40 : len));

	return pr_info("mif: %s: [%02d-%02d %02d:%02d:%02d.%03ld] %s\n",
			str, date.tm_mon + 1, date.tm_mday,
			date.tm_hour, date.tm_min, date.tm_sec,
			(tv.tv_usec > 0 ? tv.tv_usec / 1000 : 0), hexstr);
}

/* print buffer as hex string */
int pr_buffer(const char *tag, const char *data, size_t data_len,
							size_t max_len)
{
	size_t len = min(data_len, max_len);
	unsigned char hexstr[len ? len * 3 : 1]; /* 1 <= sizeof <= max_len*3 */
	dump2hex(hexstr, data, len);

	/* don't change this printk to mif_debug for print this as level7 */
	return printk(KERN_INFO "mif: %s(%u): %s%s\n", tag, data_len, hexstr,
			len == data_len ? "" : " ...");
}
EXPORT_SYMBOL(pr_buffer);

/**
 * ipv4str_to_be32 - ipv4 string to be32 (big endian 32bits integer)
 * @return: return zero when errors occurred
 */
__be32 ipv4str_to_be32(const char *ipv4str, size_t count)
{
	unsigned char ip[4];
	char ipstr[16]; /* == strlen("xxx.xxx.xxx.xxx") + 1 */
	char *next = ipstr;
	char *p;
	int i;

	strncpy(ipstr, ipv4str, ARRAY_SIZE(ipstr));

	for (i = 0; i < 4; i++) {
		p = strsep(&next, ".");
		if (kstrtou8(p, 10, &ip[i]) < 0)
			return 0; /* == 0.0.0.0 */
	}

	return *((__be32 *)ip);
}

void pr_rx_skb_with_format(struct sk_buff *skb)
{
        ShmApiMsg *hdr;

        if (unlikely(!skb))
                return;

        hdr = (ShmApiMsg *)skb->data;
        switch (hdr->procId) {
        case SSIPC_START_PROC_ID:
                pr_skb("CH-INIT-RX", skb);
                break;
        case SSIPC_DATA_PROC_ID:
                pr_skb("IPC-RX", skb);
                break;
        case MsocketLinkdownProcId:
                pr_skb("LinkDwn-RX", skb);
                break;
        case MsocketLinkupProcId:
                pr_skb("LinkUp-RX", skb);
                break;
         default:
                pr_skb("Err-RX", skb);
                break;
        }
}
EXPORT_SYMBOL(pr_rx_skb_with_format);

void pr_tx_skb_with_format(struct sk_buff *skb)
{
        ShmApiMsg *hdr;

        if (unlikely(!skb))
                return;

        hdr = (ShmApiMsg *)skb->data;
        switch (hdr->procId) {
        case SSIPC_START_PROC_ID:
                pr_skb("CH-INIT-TX", skb);
                break;
        case SSIPC_DATA_PROC_ID:
                pr_skb("IPC-TX", skb);
                break;
        case MsocketLinkdownProcId:
                pr_skb("LinkDwn-TX", skb);
                break;
        case MsocketLinkupProcId:
                pr_skb("LinkUp-TX", skb);
                break;
         default:
                pr_skb("Err-TX", skb);
                break;
        }
}
EXPORT_SYMBOL(pr_tx_skb_with_format);
