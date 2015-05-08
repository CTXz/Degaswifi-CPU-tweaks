/*

 *(C) Copyright 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All Rights Reserved
 */

#ifndef _PSD_DATA_CHANNEL_H_
#define _PSD_DATA_CHANNEL_H_

#define PSD_DATA_SEND_OK 0
#define PSD_DATA_SEND_BUSY -1
#define PSD_DATA_SEND_DROP -2

extern int sendPSDData(int cid, struct sk_buff *skb);

#endif
