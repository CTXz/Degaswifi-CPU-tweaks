/* include/linux/sec-debug.h
 *
 * Copyright (C) 2013 Samsung Electronics Co, Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef SEC_DEBUG_H
#define SEC_DEBUG_H

#include <linux/sched.h>

#ifdef CONFIG_SEC_DEBUG

union sec_debug_level_t {
	struct {
		u16 kernel_fault;
		u16 user_fault;
	} en;
	u32 uint_val;
};

struct sec_log_buffer {
	u32     sig;
	u32     start;
	u32     size;
	u8      data[0];
};

struct sec_debug_mmu_reg_t {
	int SCTLR;
	int TTBR0;
	int TTBR1;
	int TTBCR;
	int DACR;
	int DFSR;
	int DFAR;
	int IFSR;
	int IFAR;
	int DAFSR;
	int IAFSR;
	int PMRRR;
	int NMRRR;
	int FCSEPID;
	int CONTEXT;
	int URWTPID;
	int UROTPID;
	int POTPIDR;
};

/* ARM CORE regs mapping structure */
struct sec_debug_core_t {
	/* COMMON */
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
	unsigned int r12;

	/* SVC */
	unsigned int r13_svc;
	unsigned int r14_svc;
	unsigned int spsr_svc;

	/* PC & CPSR */
	unsigned int pc;
	unsigned int cpsr;

	/* USR/SYS */
	unsigned int r13_usr;
	unsigned int r14_usr;

	/* FIQ */
	unsigned int r8_fiq;
	unsigned int r9_fiq;
	unsigned int r10_fiq;
	unsigned int r11_fiq;
	unsigned int r12_fiq;
	unsigned int r13_fiq;
	unsigned int r14_fiq;
	unsigned int spsr_fiq;

	/* IRQ */
	unsigned int r13_irq;
	unsigned int r14_irq;
	unsigned int spsr_irq;

	/* MON */
	unsigned int r13_mon;
	unsigned int r14_mon;
	unsigned int spsr_mon;

	/* ABT */
	unsigned int r13_abt;
	unsigned int r14_abt;
	unsigned int spsr_abt;

	/* UNDEF */
	unsigned int r13_und;
	unsigned int r14_und;
	unsigned int spsr_und;

};

/* core reg dump function*/
static inline void sec_debug_save_core_reg(struct sec_debug_core_t *core_reg)
{
	/* we will be in SVC mode when we enter this function. Collect
	SVC registers along with cmn registers. */
#ifdef CONFIG_ARM
	asm(
		"str r0, [%0,#0]\n\t"	/* R0 is pushed first to core_reg */
		"mov r0, %0\n\t"	/* R0 will be alias for core_reg */
		"str r1, [r0,#4]\n\t"	/* R1 */
		"str r2, [r0,#8]\n\t"	/* R2 */
		"str r3, [r0,#12]\n\t"	/* R3 */
		"str r4, [r0,#16]\n\t"	/* R4 */
		"str r5, [r0,#20]\n\t"	/* R5 */
		"str r6, [r0,#24]\n\t"	/* R6 */
		"str r7, [r0,#28]\n\t"	/* R7 */
		"str r8, [r0,#32]\n\t"	/* R8 */
		"str r9, [r0,#36]\n\t"	/* R9 */
		"str r10, [r0,#40]\n\t"	/* R10 */
		"str r11, [r0,#44]\n\t"	/* R11 */
		"str r12, [r0,#48]\n\t"	/* R12 */
		/* SVC */
		"str r13, [r0,#52]\n\t"	/* R13_SVC */
		"str r14, [r0,#56]\n\t"	/* R14_SVC */
		"mrs r1, spsr\n\t"		/* SPSR_SVC */
		"str r1, [r0,#60]\n\t"
		/* PC and CPSR */
		"sub r1, r15, #0x4\n\t"	/* PC */
		"str r1, [r0,#64]\n\t"
		"mrs r1, cpsr\n\t"		/* CPSR */
		"str r1, [r0,#68]\n\t"
		/* SYS/USR */
		"mrs r1, cpsr\n\t"		/* switch to SYS mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x1f\n\t"
		"msr cpsr,r1\n\t"
		"str r13, [r0,#72]\n\t"	/* R13_USR */
		"str r14, [r0,#76]\n\t"	/* R14_USR */
		/* FIQ */
		"mrs r1, cpsr\n\t"		/* switch to FIQ mode */
		"and r1,r1,#0xFFFFFFE0\n\t"
		"orr r1,r1,#0x11\n\t"
		"msr cpsr,r1\n\t"
		"str r8, [r0,#80]\n\t"	/* R8_FIQ */
		"str r9, [r0,#84]\n\t"	/* R9_FIQ */
		"str r10, [r0,#88]\n\t"	/* R10_FIQ */
		"str r11, [r0,#92]\n\t"	/* R11_FIQ */
		"str r12, [r0,#96]\n\t"	/* R12_FIQ */
		"str r13, [r0,#100]\n\t"	/* R13_FIQ */
		"str r14, [r0,#104]\n\t"	/* R14_FIQ */
		"mrs r1, spsr\n\t"		/* SPSR_FIQ */
		"str r1, [r0,#108]\n\t"
		/* IRQ */
		"mrs r1, cpsr\n\t"		/* switch to IRQ mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x12\n\t"
		"msr cpsr,r1\n\t"
		"str r13, [r0,#112]\n\t"	/* R13_IRQ */
		"str r14, [r0,#116]\n\t"	/* R14_IRQ */
		"mrs r1, spsr\n\t"		/* SPSR_IRQ */
		"str r1, [r0,#120]\n\t"
		/* MON */
		"mrs r1, cpsr\n\t"		/* switch to monitor mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x16\n\t"
		"msr cpsr,r1\n\t"
		"str r13, [r0,#124]\n\t"	/* R13_MON */
		"str r14, [r0,#128]\n\t"	/* R14_MON */
		"mrs r1, spsr\n\t"		/* SPSR_MON */
		"str r1, [r0,#132]\n\t"
		/* ABT */
		"mrs r1, cpsr\n\t"		/* switch to Abort mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x17\n\t"
		"msr cpsr,r1\n\t"
		"str r13, [r0,#136]\n\t"	/* R13_ABT */
		"str r14, [r0,#140]\n\t"	/* R14_ABT */
		"mrs r1, spsr\n\t"		/* SPSR_ABT */
		"str r1, [r0,#144]\n\t"
		/* UND */
		"mrs r1, cpsr\n\t"		/* switch to undef mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x1B\n\t"
		"msr cpsr,r1\n\t"
		"str r13, [r0,#148]\n\t"	/* R13_UND */
		"str r14, [r0,#152]\n\t"	/* R14_UND */
		"mrs r1, spsr\n\t"		/* SPSR_UND */
		"str r1, [r0,#156]\n\t"
		/* restore to SVC mode */
		"mrs r1, cpsr\n\t"		/* switch to SVC mode */
		"and r1, r1, #0xFFFFFFE0\n\t"
		"orr r1, r1, #0x13\n\t"
		"msr cpsr,r1\n\t" :		/* output */
		: "r"(core_reg)		/* input */
		: "%r0", "%r1"		/* clobbered registers */
	);
#endif
	return;
}

static inline void sec_debug_save_mmu_reg(struct sec_debug_mmu_reg_t *mmu_reg)
{
#ifdef CONFIG_ARM
	asm(
		"mrc    p15, 0, r1, c1, c0, 0\n\t"	/* SCTLR */
		"str r1, [%0]\n\t"
		"mrc    p15, 0, r1, c2, c0, 0\n\t"	/* TTBR0 */
		"str r1, [%0,#4]\n\t"
		"mrc    p15, 0, r1, c2, c0,1\n\t"	/* TTBR1 */
		"str r1, [%0,#8]\n\t"
		"mrc    p15, 0, r1, c2, c0,2\n\t"	/* TTBCR */
		"str r1, [%0,#12]\n\t"
		"mrc    p15, 0, r1, c3, c0,0\n\t"	/* DACR */
		"str r1, [%0,#16]\n\t"
		"mrc    p15, 0, r1, c5, c0,0\n\t"	/* DFSR */
		"str r1, [%0,#20]\n\t"
		"mrc    p15, 0, r1, c6, c0,0\n\t"	/* DFAR */
		"str r1, [%0,#24]\n\t"
		"mrc    p15, 0, r1, c5, c0,1\n\t"	/* IFSR */
		"str r1, [%0,#28]\n\t"
		"mrc    p15, 0, r1, c6, c0,2\n\t"	/* IFAR */
		"str r1, [%0,#32]\n\t"
		/* Don't populate DAFSR and RAFSR */
		"mrc    p15, 0, r1, c10, c2,0\n\t"	/* PMRRR */
		"str r1, [%0,#44]\n\t"
		"mrc    p15, 0, r1, c10, c2,1\n\t"	/* NMRRR */
		"str r1, [%0,#48]\n\t"
		"mrc    p15, 0, r1, c13, c0,0\n\t"	/* FCSEPID */
		"str r1, [%0,#52]\n\t"
		"mrc    p15, 0, r1, c13, c0,1\n\t"	/* CONTEXT */
		"str r1, [%0,#56]\n\t"
		"mrc    p15, 0, r1, c13, c0,2\n\t"	/* URWTPID */
		"str r1, [%0,#60]\n\t"
		"mrc    p15, 0, r1, c13, c0,3\n\t"	/* UROTPID */
		"str r1, [%0,#64]\n\t"
		"mrc    p15, 0, r1, c13, c0,4\n\t"	/* POTPIDR */
		"str r1, [%0,#68]\n\t" :		/* output */
		: "r"(mmu_reg)			/* input */
		: "%r1", "memory"			/* clobbered register */
	);
#endif
}

extern int sec_crash_key_panic;
extern union sec_debug_level_t sec_debug_level;

extern int sec_debug_init(void);
extern void sec_debug_check_crash_key(unsigned int code, int value);
extern int sec_debug_panic_dump(char *buf);
extern void sec_getlog_supply_loggerinfo(unsigned char *buffer,
		const char *name);
extern void sec_getlog_supply_fbinfo(void *p_fb, u32 xres, u32 yres, u32 bpp,
		u32 frames);
extern void sec_gaf_supply_rqinfo(unsigned short curr_offset,
		unsigned short rq_offset);
#endif

#ifdef CONFIG_SEC_LOG
extern void sec_getlog_supply_kloginfo(void *klog_buf);
extern void register_log_char_hook();
#endif

struct worker;
struct work_struct;

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
extern void __sec_debug_task_log(int cpu, struct task_struct *task);
extern void __sec_debug_irq_log(unsigned int irq, void *fn, int en);
extern void __sec_debug_work_log(struct worker *worker,
			struct work_struct *work, work_func_t f, int en);
#ifdef CONFIG_SEC_DEBUG_TIMER_LOG
extern void __sec_debug_timer_log(unsigned int type, void *fn);
#endif

static inline void sec_debug_task_log(int cpu, struct task_struct *task)
{
	if (unlikely(sec_debug_level.en.kernel_fault))
		__sec_debug_task_log(cpu, task);
}

static inline void sec_debug_irq_log(unsigned int irq, void *fn, int en)
{
	if (unlikely(sec_debug_level.en.kernel_fault))
		__sec_debug_irq_log(irq, fn, en);
}

static inline void sec_debug_work_log(struct worker *worker,
			struct work_struct *work, work_func_t f, int en)
{
	if (unlikely(sec_debug_level.en.kernel_fault))
		__sec_debug_work_log(worker, work, f, en);
}

#ifdef CONFIG_SEC_DEBUG_TIMER_LOG
static inline void sec_debug_timer_log(unsigned int type, void *fn)
{
	if (unlikely(sec_debug_level.en.kernel_fault))
		__sec_debug_timer_log(type, fn);
}
#else
static inline void sec_debug_timer_log(unsigned int type, void *fn)
{
}
#endif

#ifdef CONFIG_SEC_DEBUG_SOFTIRQ_LOG
static inline void sec_debug_softirq_log(unsigned int irq, void *fn, int en)
{
	if (unlikely(sec_debug_level.en.kernel_fault))
		__sec_debug_irq_log(irq, fn, en);
}
#else
static inline void sec_debug_softirq_log(unsigned int irq, void *fn, int en)
{
}
#endif
#endif
#endif				/* SEC_DEBUG_H */
