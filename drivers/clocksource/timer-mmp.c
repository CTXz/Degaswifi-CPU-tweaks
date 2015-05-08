/*
 * driver/clocksource/timer-mmp.c
 *
 *   Support for clocksource and clockevents
 *
 * Copyright (C) 2008 Marvell International Ltd.
 * All rights reserved.
 *
 *   2008-04-11: Jason Chagas <Jason.chagas@marvell.com>
 *   2008-10-08: Bin Yang <bin.yang@marvell.com>
 *
 * The timers module actually includes three timers, each timer with up to
 * three match comparators. Timer #0 is used here in free-running mode as
 * the clock source, and match comparator #1 used as clock event device.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/mmp_timer.h>
#include <linux/clockchips.h>

#ifdef CONFIG_ARM
#include <asm/sched_clock.h>
#endif

#define TMR_CCR		(0x0000)
#define TMR_TN_MM(n, m)	(0x0004 + ((n) << 3) + (((n) + (m)) << 2))
#define TMR_CR(n)	(0x0028 + ((n) << 2))
#define TMR_SR(n)	(0x0034 + ((n) << 2))
#define TMR_IER(n)	(0x0040 + ((n) << 2))
#define TMR_PLVR(n)	(0x004c + ((n) << 2))
#define TMR_PLCR(n)	(0x0058 + ((n) << 2))
#define TMR_WMER	(0x0064)
#define TMR_WMR		(0x0068)
#define TMR_WVR		(0x006c)
#define TMR_WSR		(0x0070)
#define TMR_ICR(n)	(0x0074 + ((n) << 2))
#define TMR_WICR	(0x0080)
#define TMR_CER		(0x0084)
#define TMR_CMR		(0x0088)
#define TMR_ILR(n)	(0x008c + ((n) << 2))
#define TMR_WCR		(0x0098)
#define TMR_WFAR	(0x009c)
#define TMR_WSAR	(0x00A0)
#define TMR_CVWR(n)	(0x00A4 + ((n) << 2))
#define TMR_CRSR        (0x00C8)

#define TMR_CCR_CS_0(x)	(((x) & 0x3) << 0)
#define TMR_CCR_CS_1(x)	(((x) & 0x3) << 2)
#define TMR_CCR_CS_2(x)	(((x) & 0x3) << 5)

#define MAX_EVT_NUM		5
#define CLOCK_TICK_RATE_6500KHZ	6500000
#define CLOCK_TICK_RATE_32KHZ	32768
#define CLOCK_TICK_RATE_1KHZ	1000

#define MAX_DELTA		(0xfffffffe)
#define MIN_DELTA		(5)

#define MMP_MAX_COUNTER		3
#define MMP_MAX_TIMER		4

#define TMR_CER_COUNTER(cid)	(1 << cid)
#define MMP_ALL_COUNTERS	((1 << MMP_MAX_COUNTER) - 1)

struct mmp_timer;
struct mmp_timer_counter {
	unsigned int id;
	unsigned int usage;
	unsigned int cnt_freq;
	int mode;
	int cpu;
	struct mmp_timer *timer;
};

struct mmp_timer {
	unsigned int id;
	void __iomem *base;
	struct mmp_timer_counter counters[MMP_MAX_COUNTER];
	unsigned int flag;
	int loop_delay;
	spinlock_t tm_lock;
};

struct mmp_timer_clkevt {
	struct mmp_timer_counter *counter;
	struct clock_event_device ced;
	struct irqaction irqa;
	struct notifier_block nb;
};

struct mmp_timer_clksrc {
	struct mmp_timer_counter *counter;
	struct clocksource cs;
};

#ifdef CONFIG_ARM
struct mmp_timer_dclk {
	struct mmp_timer_counter *counter;
	struct delay_timer *dt;
};
static struct mmp_timer_dclk *dclk;
#endif

static struct mmp_timer *mmp_timers[MMP_MAX_TIMER];
static struct mmp_timer_clksrc *clksrc;

static inline void timer_stop_cnter(struct mmp_timer_counter *cnt);
static inline void timer_restart_cnter(struct mmp_timer_counter *cnt);
static void timer_oneshot_disable(struct mmp_timer_counter *cnt);

static inline uint32_t timer_read(struct mmp_timer_counter *cnt)
{
	struct mmp_timer *tm = cnt->timer;
	int has_shadow = tm->flag & MMP_TIMER_FLAG_SHADOW;
	int delay = 3;
	u32 val1, val2;

	if (has_shadow)
		return __raw_readl(tm->base + TMR_CR(cnt->id));
	else {
		if (cnt->cnt_freq <= CLOCK_TICK_RATE_32KHZ) {
			/* slow clock */
			do {
				val1 = __raw_readl(tm->base + TMR_CR(cnt->id));
				val2 = __raw_readl(tm->base + TMR_CR(cnt->id));
			} while (val2 != val1);
		} else {
			/* fast clock */
			__raw_writel(1, tm->base + TMR_CVWR(cnt->id));
			while (delay--)
				val1 = __raw_readl(tm->base +
						TMR_CVWR(cnt->id));
		}
		return val1;
	}
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;
	struct mmp_timer_clkevt *evt;
	struct mmp_timer_counter *counter;
	unsigned int cnt;
	void __iomem *base;
	unsigned long flags;

	evt = container_of(c, struct mmp_timer_clkevt, ced);
	counter = evt->counter;
	cnt = counter->id;
	base = counter->timer->base;

	spin_lock_irqsave(&(counter->timer->tm_lock), flags);
	if (counter->mode == MMP_TIMER_COUNTER_FREERUNNING)
		__raw_writel(0x01, base + TMR_ICR(cnt));
	else if (counter->mode == MMP_TIMER_COUNTER_ONESHOT)
		timer_oneshot_disable(counter);
	if (c->broadcast && (counter->usage & MMP_TIMER_COUNTER_CLKEVT_GLB))
		c->broadcast(cpu_online_mask);
	spin_unlock_irqrestore(&(counter->timer->tm_lock), flags);

	c->event_handler(c);

	return IRQ_HANDLED;
}

static int timer_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	struct mmp_timer_counter *counter;
	struct mmp_timer_clkevt *evt;
	unsigned long flags;
	unsigned int val, cnt, ccr, msk, cer;
	void __iomem *base;
	int delay;
	int has_crsr;

	evt = container_of(dev, struct mmp_timer_clkevt, ced);
	counter = evt->counter;
	cnt = counter->id;
	base = counter->timer->base;
	has_crsr = counter->timer->flag & MMP_TIMER_FLAG_CRSR;
	delay = counter->timer->loop_delay;

	spin_lock_irqsave(&(counter->timer->tm_lock), flags);
	switch (counter->mode) {
	case MMP_TIMER_COUNTER_FREERUNNING:
		timer_stop_cnter(counter);

		/*
		 * Clear and enable timer match 0 interrupt.
		 */
		__raw_writel(0x01, base + TMR_ICR(cnt));
		__raw_writel(0x01, base + TMR_IER(cnt));

		/*
		 * Setup new clockevent timer value.
		 */
		val = timer_read(counter);
		if (counter->timer->flag & MMP_TIMER_FLAG_PLVR)
			__raw_writel(val, base + TMR_PLVR(cnt));
		val += delta - 1;
		__raw_writel(val, base + TMR_TN_MM(cnt, 0));

		timer_restart_cnter(counter);
		break;
	case MMP_TIMER_COUNTER_ONESHOT:
		if (has_crsr) {
			__raw_writel(0x01, base + TMR_ICR(cnt));
			__raw_writel(0x01, base + TMR_IER(cnt));
			__raw_writel(delta - 1, base + TMR_TN_MM(cnt, 0));
			__raw_writel((1 << cnt), base + TMR_CRSR);
		} else {
			/* disable */
			cer = __raw_readl(base + TMR_CER);
			__raw_writel(cer & ~(1 << cnt), base + TMR_CER);

			/* switch to fast clock */
			ccr = __raw_readl(base + TMR_CCR);
			if (counter->cnt_freq <= CLOCK_TICK_RATE_32KHZ) {
				msk = 0;
				if (cnt == 2)
					msk = TMR_CCR_CS_2(3);
				else if (cnt == 1)
					msk = TMR_CCR_CS_1(3);
				else if (cnt == 0)
					msk = TMR_CCR_CS_0(3);
				__raw_writel(ccr & (~msk), base + TMR_CCR);
			}

			/* Clear pending interrupt status */
			while (delay--)
				__raw_writel(0x1, base + TMR_ICR(cnt));

			/* enable the matching interrupt */
			__raw_writel(0x1, base + TMR_IER(cnt));

			/* Setup new counter value */
			__raw_writel(delta - 1, base + TMR_TN_MM(cnt, 0));

			/* switch back to 32K */
			if (counter->cnt_freq <= CLOCK_TICK_RATE_32KHZ)
				__raw_writel(ccr, base + TMR_CCR);

			/* Enable timer */
			cer = __raw_readl(base + TMR_CER);
			__raw_writel(cer | (1 << cnt), base + TMR_CER);
		}
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&(counter->timer->tm_lock), flags);

	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	unsigned long flags;
	unsigned int val, cnt;
	struct mmp_timer_counter *counter;
	struct mmp_timer_clkevt *evt;
	void __iomem *base;

	evt = container_of(dev, struct mmp_timer_clkevt, ced);
	counter = evt->counter;
	cnt = counter->id;
	base = counter->timer->base;

	spin_lock_irqsave(&(counter->timer->tm_lock), flags);
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		if (counter->mode == MMP_TIMER_COUNTER_FREERUNNING) {
			timer_stop_cnter(counter);
			if (counter->timer->flag & MMP_TIMER_FLAG_PLVR) {
				val = timer_read(counter);
				__raw_writel(val, base + TMR_PLVR(cnt));
			}
			/* disable the matching interrupt */
			__raw_writel(0x00, base + TMR_IER(cnt));
			timer_restart_cnter(counter);
		} else if (counter->mode == MMP_TIMER_COUNTER_ONESHOT) {
			timer_oneshot_disable(counter);
		}
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
	spin_unlock_irqrestore(&(counter->timer->tm_lock), flags);
}

static cycle_t clksrc_read(struct clocksource *cs)
{
	return timer_read(clksrc->counter);
}

static void timer_oneshot_disable(struct mmp_timer_counter *cnt)
{
	struct mmp_timer *tm = cnt->timer;
	int delay = tm->loop_delay;
	u32 val, ccr, msk;

	/* disable counter */
	val = __raw_readl(tm->base + TMR_CER);
	val &= ~(1 << cnt->id);
	__raw_writel(val, tm->base + TMR_CER);

	ccr = __raw_readl(tm->base + TMR_CCR);
	/* switch to fast clock */
	if (cnt->cnt_freq <= CLOCK_TICK_RATE_32KHZ) {
		msk = 0;
		if (cnt->id == 2)
			msk = TMR_CCR_CS_2(3);
		else if (cnt->id == 1)
			msk = TMR_CCR_CS_1(3);
		else if (cnt->id == 0)
			msk = TMR_CCR_CS_0(3);
		__raw_writel(ccr & (~msk), tm->base + TMR_CCR);
	}

	/* disable matching interrupt */
	__raw_writel(0x00, tm->base + TMR_IER(cnt->id));
	/* Clear pending interrupt status */
	while (delay--)
		__raw_writel(0x1, tm->base + TMR_ICR(cnt->id));

	/* switch back to slow clock */
	if (cnt->cnt_freq <= CLOCK_TICK_RATE_32KHZ)
		__raw_writel(ccr, tm->base + TMR_CCR);
}

/*
 * The helper function to stop the timer, which will not reset the counter
 * value and just only stop the counter to increase.
 *
 * Note: if has the CRSR register then it's not necessary to stop the
 * timer counter. so in this function will skip for this situation.
 */
static inline void timer_stop_cnter(struct mmp_timer_counter *cnt)
{
	struct mmp_timer *tm = cnt->timer;
	int has_crsr = tm->flag & MMP_TIMER_FLAG_CRSR;
	unsigned int val;

	if (!has_crsr) {
		val = __raw_readl(tm->base + TMR_CER);
		val &= ~(1 << cnt->id);
		__raw_writel(val, tm->base + TMR_CER);
	}

	return;
}

static inline void timer_restart_cnter(struct mmp_timer_counter *cnt)
{
	struct mmp_timer *tm = cnt->timer;
	int has_crsr = tm->flag & MMP_TIMER_FLAG_CRSR;
	unsigned int val;

	if (!has_crsr) {
		val = __raw_readl(tm->base + TMR_CER);
		val |= (1 << cnt->id);
		__raw_writel(val, tm->base + TMR_CER);
	} else {
		__raw_writel((1 << cnt->id), tm->base + TMR_CRSR);
	}
}

#ifdef CONFIG_ARM
static u32 notrace mmp_read_sched_clock(void)
{
	return timer_read(clksrc->counter);
}

static unsigned long d_read_current_timer(void)
{
	return timer_read(dclk->counter);
}

static struct delay_timer d_timer = {
	.read_current_timer	= d_read_current_timer,
};
#endif

static int __cpuinit mmp_timer_cpu_notify(struct notifier_block *self,
					   unsigned long action, void *hcpu)
{
	struct mmp_timer_clkevt *evt;
	struct mmp_timer_counter *counter;

	evt = container_of(self, struct mmp_timer_clkevt, nb);
	counter = evt->counter;

	if (counter->cpu != (unsigned long)hcpu)
		return NOTIFY_OK;

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		clockevents_config_and_register(&evt->ced,
						evt->counter->cnt_freq,
						MIN_DELTA, MAX_DELTA);
		break;
	case CPU_ONLINE:
		irq_set_affinity(evt->ced.irq, evt->ced.cpumask);
		enable_irq(evt->ced.irq);
		break;
	case CPU_DYING:
		clockevents_set_mode(&evt->ced,
					CLOCK_EVT_MODE_SHUTDOWN);
		disable_irq(evt->ced.irq);
		break;
	}

	return NOTIFY_OK;
}

int __init mmp_timer_init(int id, void __iomem *base,
				unsigned int flag, int ld)
{
	u32 tmp, delay = ld;

	if (mmp_timers[id])
		return -EINVAL;

	mmp_timers[id] = kzalloc(sizeof(struct mmp_timer), GFP_KERNEL);
	if (!mmp_timers[id])
		return -ENOMEM;

	mmp_timers[id]->id = id;
	mmp_timers[id]->base = base;
	mmp_timers[id]->flag = flag;
	mmp_timers[id]->loop_delay = ld;
	spin_lock_init(&(mmp_timers[id]->tm_lock));

	/* disalbe all counters */
	tmp = __raw_readl(base + TMR_CER) & ~MMP_ALL_COUNTERS;
	__raw_writel(tmp, base + TMR_CER);

	while (delay--)
		__raw_writel(tmp, base + TMR_CER);

	return 0;
}
EXPORT_SYMBOL(mmp_timer_init);

int __init mmp_counter_init(int tid, int cid, int mode, int usg, int rating,
				int irq, unsigned int cnt_freq, int cpu)
{
	struct mmp_timer *mt = mmp_timers[tid];
	struct mmp_timer_clkevt *clkevt;
	u32 tmp, delay;

	if (!mt)
		return -EINVAL;

	if (cid < 0 || cid >= MMP_MAX_COUNTER)
		return -EINVAL;
	if (cpu < 0 || cpu >= num_possible_cpus())
		return -EINVAL;
	if (!(usg & MMP_TIMER_USAGE_MSK))
		return -EINVAL;
	if (mode != MMP_TIMER_COUNTER_FREERUNNING &&
		mode != MMP_TIMER_COUNTER_ONESHOT)
		return -EINVAL;

	delay = mt->loop_delay;
	mt->counters[cid].id = cid;
	mt->counters[cid].usage = usg;
	mt->counters[cid].cnt_freq = cnt_freq;
	mt->counters[cid].mode = mode;
	mt->counters[cid].timer = mt;
	mt->counters[cid].cpu = cpu;

	if (usg & MMP_TIMER_COUNTER_CLKSRC) {
		if (mode == MMP_TIMER_COUNTER_ONESHOT) {
			pr_err("Clksrc cannot use oneshot mode!\n");
			return -EINVAL;
		}
		if (clksrc) {
			pr_err("One clksrc has already been registered!\n");
			return -EINVAL;
		}
		clksrc = kzalloc(sizeof(struct mmp_timer_clksrc), GFP_KERNEL);
		if (!clksrc)
			return -ENOMEM;
		clksrc->counter = &mt->counters[cid];
		clksrc->cs.name = "clocksource-mmp";
		clksrc->cs.rating = rating;
		clksrc->cs.read = clksrc_read;
		clksrc->cs.mask = CLOCKSOURCE_MASK(32);
		clksrc->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;

#ifdef CONFIG_ARM
		setup_sched_clock(mmp_read_sched_clock, 32, cnt_freq);
#endif
		clocksource_register_hz(&(clksrc->cs), cnt_freq);
	}

#ifdef CONFIG_ARM
	if (usg & MMP_TIMER_COUNTER_DELAY) {
		if (mode == MMP_TIMER_COUNTER_ONESHOT) {
			pr_err("Clksrc cannot use oneshot mode!\n");
			return -EINVAL;
		}
		if (dclk) {
			pr_err("Delay clock has already been registered!\n");
			return -EINVAL;
		}
		dclk = kzalloc(sizeof(struct mmp_timer_dclk), GFP_KERNEL);
		if (!dclk)
			return -ENOMEM;
		dclk->counter = &mt->counters[cid];
		dclk->dt = &d_timer;
		d_timer.freq = cnt_freq;
		register_current_timer_delay(&d_timer);
	}
#endif

	if (usg & MMP_TIMER_COUNTER_CLKEVT_GLB ||
		usg & MMP_TIMER_COUNTER_CLKEVT_LOC) {
		clkevt = kzalloc(sizeof(struct mmp_timer_clkevt), GFP_KERNEL);
		if (!clkevt)
			return -ENOMEM;
		clkevt->counter = &mt->counters[cid];
		clkevt->ced.name = "clockevent-mmp";
		clkevt->ced.features = CLOCK_EVT_FEAT_ONESHOT;
		clkevt->ced.rating = rating;
		if (usg & MMP_TIMER_COUNTER_DYNIRQ)
			clkevt->ced.features |= CLOCK_EVT_FEAT_DYNIRQ;

		clkevt->ced.set_next_event = timer_set_next_event;
		clkevt->ced.set_mode = timer_set_mode;
#ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
		clkevt->ced.broadcast = tick_broadcast;
#endif

		clkevt->irqa.name = "timer",
		clkevt->irqa.flags = IRQF_DISABLED | IRQF_TIMER |
					IRQF_IRQPOLL;
		clkevt->irqa.handler = timer_interrupt;
		clkevt->irqa.dev_id = &(clkevt->ced);
		clkevt->ced.cpumask = cpumask_of(cpu);
		clkevt->ced.irq = irq;

		if (usg & MMP_TIMER_COUNTER_CLKEVT_GLB) {
			clkevt->irqa.flags |= IRQF_SHARED;
			setup_irq(irq, &(clkevt->irqa));
			clockevents_config_and_register(&clkevt->ced,
					cnt_freq, MIN_DELTA, MAX_DELTA);
		}

		if (usg & MMP_TIMER_COUNTER_CLKEVT_LOC) {
			clkevt->irqa.flags |= IRQF_PERCPU;
			clkevt->nb.notifier_call = mmp_timer_cpu_notify;
			register_cpu_notifier(&clkevt->nb);

			/* set irq for every cpu */
			setup_irq(clkevt->ced.irq, &(clkevt->irqa));

			if (cpu == 0) {
				/* register boot CPU's clockevent now */
				clockevents_config_and_register(&clkevt->ced,
						clkevt->counter->cnt_freq,
						MIN_DELTA, MAX_DELTA);
				irq_set_affinity(clkevt->ced.irq,
						cpumask_of(0));
			} else {
				/* disable none boot CPU's irq at first */
				disable_irq(clkevt->ced.irq);
			}
		}
	}

	tmp = __raw_readl(mt->base + TMR_CCR);
	switch (cnt_freq) {
	case CLOCK_TICK_RATE_32KHZ:
		if (cid == 2)
			tmp |= TMR_CCR_CS_2(2);
		else if (cid == 1)
			tmp |= TMR_CCR_CS_1(1);
		else if (cid == 0)
			tmp |= TMR_CCR_CS_0(1);
		break;
	case CLOCK_TICK_RATE_1KHZ:
		if (cid == 2)
			tmp |= TMR_CCR_CS_2(1);
		else if (cid == 1)
			tmp |= TMR_CCR_CS_1(2);
		break;
	}
	__raw_writel(tmp, mt->base + TMR_CCR);

	/* set timer to free-running mode */
	tmp = __raw_readl(mt->base + TMR_CMR) | TMR_CER_COUNTER(cid);
	__raw_writel(tmp, mt->base + TMR_CMR);

	/* free-running */
	__raw_writel(0x0, mt->base + TMR_PLCR(cid));
	/* clear status */
	__raw_writel(0x7, mt->base + TMR_ICR(cid));

	/* enable counter */
	tmp = __raw_readl(mt->base + TMR_CER) | TMR_CER_COUNTER(cid);
	__raw_writel(tmp, mt->base + TMR_CER);

	while (delay--)
		__raw_writel(tmp, mt->base + TMR_CER);

	return 0;
}
EXPORT_SYMBOL(mmp_counter_init);

#ifdef CONFIG_OF
static void __init mmp_of_init_timer(struct device_node *np)
{
	int timer_id, usg, irq, counter_id, cpu, cnt_rating, mode;
	unsigned int flag, cnt_freq, fc_freq, apb_freq, ld;
	void __iomem *base;
	struct device_node *child_np;
	int ret = 0;

	/* timer initialization */
	of_property_read_u32(np, "marvell,timer-id", &timer_id);
	if (timer_id < 0 || timer_id >= MMP_MAX_TIMER) {
		ret = -EINVAL;
		goto out;
	}
	base = of_iomap(np, 0);
	if (!base) {
		ret = -ENOMEM;
		goto out;
	}
	of_property_read_u32(np, "marvell,timer-flag", &flag);

	/* timer's fast clock and apb frequency */
	of_property_read_u32(np, "marvell,timer-fc_freq",  &fc_freq);
	of_property_read_u32(np, "marvell,timer-apb_freq", &apb_freq);

	/*
	 * Need use loop for more safe register's accessing,
	 * so at here dynamically calculate the loop time.
	 */
	if (!fc_freq || !apb_freq) {
		pr_err("mmp timer's fast clock or apb freq are incorrect!\n");
		ret = -EINVAL;
		goto out;
	}

	/*
	 * The calculation formula for the loop cycle is:
	 *
	 * (1) need wait for 2 timer's clock cycle:
	 *        1             2
	 *     ------- x 2 = -------
	 *     fc_freq       fc_freq
	 *
	 * (2) convert to apb clock cycle:
	 *        2          1        apb_freq * 2
	 *     ------- / -------- = ----------------
	 *     fc_freq   apb_freq       fc_freq
	 *
	 * (3) every apb register's accessing will take 8 apb clock cycle,
	 *     also consider add extral one more time for safe way;
	 *     so finally need loop times for the apb register accessing:
	 *
	 *       (apb_freq * 2)
	 *     ------------------ / 8 + 1
	 *          fc_freq
	 */
	ld = ((apb_freq * 2) / fc_freq / 8) + 1;
	pr_debug("mmp timer loop delay count is %d\n", ld);

	ret = mmp_timer_init(timer_id, base, flag, ld);
	if (ret)
		goto out;

	/* counter initialization */
	for_each_child_of_node(np, child_np) {
		irq = irq_of_parse_and_map(child_np, 0);
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-id", &counter_id)) {
			ret = -EINVAL;
			goto out;
		}
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-cpu", &cpu)) {
			ret = -EINVAL;
			goto out;
		}
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-cnt_freq", &cnt_freq)) {
			ret = -EINVAL;
			goto out;
		}
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-mode", &mode)) {
			ret = -EINVAL;
			goto out;
		}
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-usage", &usg)) {
			ret = -EINVAL;
			goto out;
		}
		if (of_property_read_u32(child_np,
			"marvell,timer-counter-cnt_rating", &cnt_rating)) {
			ret = -EINVAL;
			goto out;
		}
		ret = mmp_counter_init(timer_id, counter_id, mode,
					usg, cnt_rating, irq, cnt_freq, cpu);
		if (ret)
			goto out;
	}

	return;
out:
	pr_err("Failed to get timer from device tree with error:%d\n", ret);
}

CLOCKSOURCE_OF_DECLARE(mmp_timer, "marvell,mmp-timer", mmp_of_init_timer);
#endif
