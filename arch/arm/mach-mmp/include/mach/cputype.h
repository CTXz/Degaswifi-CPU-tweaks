#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/cputype.h>

/*
 *  CPU   Stepping   CPU_ID      CHIP_ID
 *
 * PXA168    S0    0x56158400   0x0000C910
 * PXA168    A0    0x56158400   0x00A0A168
 * PXA910    Y1    0x56158400   0x00F2C920
 * PXA910    A0    0x56158400   0x00F2C910
 * PXA910    A1    0x56158400   0x00A0C910
 * PXA920    Y0    0x56158400   0x00F2C920
 * PXA920    A0    0x56158400   0x00A0C920
 * PXA920    A1    0x56158400   0x00A1C920
 * MMP2	     Z0	   0x560f5811   0x00F00410
 * MMP2      Z1    0x560f5811   0x00E00410
 * MMP2      A0    0x560f5811   0x00A0A610
 */

extern unsigned int mmp_chip_id;

#ifdef CONFIG_CPU_PXA168
static inline int cpu_is_pxa168(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x84) &&
		((mmp_chip_id & 0xfff) == 0x168);
}
#else
#define cpu_is_pxa168()	(0)
#endif

/* cpu_is_pxa910() is shared on both pxa910 and pxa920 */
#ifdef CONFIG_CPU_PXA910
static inline int cpu_is_pxa910(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x84) &&
		(((mmp_chip_id & 0xfff) == 0x910) ||
		 ((mmp_chip_id & 0xfff) == 0x920));
}
#else
#define cpu_is_pxa910()	(0)
#endif

#ifdef CONFIG_CPU_MMP2
static inline int cpu_is_mmp2(void)
{
	return (((read_cpuid_id() >> 8) & 0xff) == 0x58);
}
#else
#define cpu_is_mmp2()	(0)
#endif

#ifdef CONFIG_CPU_PXA988
static inline int cpu_is_pxa988(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc09) &&
		(((mmp_chip_id & 0xffff) == 0xc988) ||
		((mmp_chip_id & 0xffff) == 0xc928));
}
static inline int cpu_is_pxa986(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc09) &&
		(((mmp_chip_id & 0xffff) == 0xc986) ||
		((mmp_chip_id & 0xffff) == 0xc926));
}
static inline int cpu_is_pxa1088(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffff) == 0x1088));
}
static inline int cpu_is_pxa1L88(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffff) == 0x1188));
}
static inline int cpu_is_pxa1L88_a0(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffffff) == 0xa01188));
}

static inline int cpu_is_pxa1L88_a0c(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffffff) == 0xb01188));
}

static inline int cpu_is_pxa1U88(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffff) == 0x1098));
}

static inline int cpu_is_pxa1U88_z1(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffffff) == 0xa01098));
}

#else
#define cpu_is_pxa988()	(0)
#define cpu_is_pxa986()	(0)
#define cpu_is_pxa1088()	(0)
#define cpu_is_pxa1L88()	(0)
#define cpu_is_pxa1L88_a0()	(0)
#define cpu_is_pxa1L88_a0c()	(0)
#define cpu_is_pxa1U88()	(0)
#define cpu_is_pxa1U88_z1()	(0)
#endif



#ifdef CONFIG_CPU_EDEN
static inline int cpu_is_eden(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		(((mmp_chip_id & 0xffff) == 0xc192) ||
		 ((mmp_chip_id & 0xffff) == 0xc198));
}

static inline int cpu_is_eden_a0(void)
{
	return (((read_cpuid_id() >> 4) & 0xfff) == 0xc07) &&
		((mmp_chip_id & 0xffffff) == 0xa0c198);
}

static inline int cpu_is_eden_z1(void)
{
	        return cpu_is_eden() && ((read_cpuid_id() & 0xf) == 0x3);
}

static inline int cpu_is_eden_z2(void)
{
	        return cpu_is_eden() && ((read_cpuid_id() & 0xf) == 0x5);
}
#else
#define cpu_is_eden()		(0)
#define cpu_is_eden_a0()        (0)
#define cpu_is_eden_z1()        (0)
#define cpu_is_eden_z2()        (0)
#endif

#ifdef CONFIG_CPU_PXA1986
static inline int cpu_is_pxa1986(void)
{
	return (((((read_cpuid_id() >> 4) & 0xfff) == 0xc0f) ||
		(((read_cpuid_id() >> 4) & 0xfff) == 0xc07)) &&
		((mmp_chip_id & 0xe000) == 0x8000));
}

static inline int cpu_is_pxa1986_z1(void)
{
	/*
	 * found some chips with id 0x8000 instead of 0x8001
	 * z2 chip is same as z1 with new comm features. No need for
	 * special function at the moment.
	 */
	return (cpu_is_pxa1986() && (((mmp_chip_id & 0xffff) == 0x8001) ||
				((mmp_chip_id & 0xffff) == 0x8000) ||
				((mmp_chip_id & 0xffff) == 0x8002)));
}

static inline int cpu_is_pxa1986_a0(void)
{
	return (cpu_is_pxa1986() && ((mmp_chip_id & 0xffff) == 0x8020));
}
#else
#define cpu_is_pxa1986()	(0)
#define cpu_is_pxa1986_z1()	(0)
#define cpu_is_pxa1986_a0()	(0)
#endif

static inline bool cpu_is_ca9(void)
{
	if ((read_cpuid_id() & 0xfff0) == 0xc090)
		return true;

	return false;
}

static inline bool cpu_is_ca7(void)
{
	if ((read_cpuid_id() & 0xfff0) == 0xc070)
		return true;

	return false;
}

static inline bool cpu_is_ca15(void)
{
	if ((read_cpuid_id() & 0xfff0) == 0xc0f0)
		return true;

	return false;
}
#endif /* __ASM_MACH_CPUTYPE_H */
