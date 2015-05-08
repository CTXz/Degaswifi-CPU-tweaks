/*
 * mmp_timer: Soc timer driver for mmp architecture.
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MMP_TIMER_H__
#define __LINUX_MMP_TIMER_H__
/* timer flag bit definition */
/* bit[0]: MMP_TIMER_FLAG_SHADOW
 *         Indicate if the timer has shadow registers. If it has,
 *         counter could be read out directly.
 * bit[1]: MMP_TIMER_FLAG_CRSR
 *         Indicate if timer has CRSR register. If it has,
 *         counter could be restarted by directly writing CRSR.
 * bit[2]: MMP_TIMER_FLAG_PLVR
 *         Indicate if tiemr needs preload value when it restarts.
 */
#define MMP_TIMER_FLAG_SHADOW	(1 << 0)
#define MMP_TIMER_FLAG_CRSR	(1 << 1)
#define MMP_TIMER_FLAG_PLVR	(1 << 2)

#define MMP_TIMER_COUNTER_NOTUSED	0
#define MMP_TIMER_COUNTER_CLKSRC	(1 << 0)
#define MMP_TIMER_COUNTER_CLKEVT_GLB	(1 << 1)
#define MMP_TIMER_COUNTER_CLKEVT_LOC	(1 << 2)
#define MMP_TIMER_COUNTER_DELAY		(1 << 3)
#define MMP_TIMER_COUNTER_DYNIRQ	(1 << 4)
#define MMP_TIMER_USAGE_MSK		(MMP_TIMER_COUNTER_CLKSRC | \
					 MMP_TIMER_COUNTER_CLKEVT_GLB | \
					 MMP_TIMER_COUNTER_CLKEVT_LOC | \
					 MMP_TIMER_COUNTER_DELAY | \
					 MMP_TIMER_COUNTER_DYNIRQ)

#define MMP_TIMER_COUNTER_FREERUNNING	0
#define MMP_TIMER_COUNTER_ONESHOT	1

int __init mmp_timer_init(int id, void __iomem *base,
				unsigned int flag, int ld);
int __init mmp_counter_init(int tid, int cid, int mode, int usg, int rating,
				int irq, unsigned int cnt_freq, int cpu);

#endif /* __LINUX_MMP_TIMER_H__ */
