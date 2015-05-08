/*

 *(C) Copyright 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * All Rights Reserved
 */

#ifndef _SEH_LINUX_H_
#define _SEH_LINUX_H_

#include "eeh_ioctl.h"

struct seh_dev {
	struct clk *ripc_clk;
	EehMsgStruct msg;
	struct device *dev;
	struct semaphore read_sem;	/* mutual exclusion semaphore */
	wait_queue_head_t readq;	/* read queue */
};

#ifdef CONFIG_SSIPC_SUPPORT
extern int read_ee_config_b_cp_reset(void);
extern void trigger_modem_crash(int force_reset);
#endif
#endif
