/*
 * PXA988 CP related
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <mach/regs-ciu.h>
#include <mach/regs-apmu.h>
#include <linux/module.h>
#include "pxa_cp_load.h"

#define MPMU_APRR		(0x1020)
#define MPMU_APRR_CPR	(1 << 0)

#define MPMU_CPRR		(0x0020)
#define MPMU_CPRR_DSPR	(1 << 2)
#define MPMU_CPRR_BBR	(1 << 3)
#define MPMU_CPRR_DSRAMINT	(1 << 5)

#define APMU_DEBUG_CP_HALT			(1 << 0)
#define APMU_DEBUG_CP_CLK_OFF_ACK	(1 << 3)

/**
 * CP release procedure
 * 1. acquire fc mutex
 * 2. unmask bits CP_CLK_OFF_ACK, CP_HALT of register APMU_DEBUG to
 *    enable ACK from CP
 * 3. clear bit CPR of register APRR to release CP
 * 4. wait until read back zero value after clear bit CPR
 * 5. release fc mutex
 */
void cp_releasecp(void)
{
	int value;
	int timeout = 1000000;
	u8 apmu_debug;

	acquire_fc_mutex();
	printk(KERN_INFO "%s: acquire fc mutex\n", __func__);

	/*
	 * unmask CP_CLK_OFF_ACK and CP_HALT
	 */
	printk(KERN_INFO "%s: unmask cp halt and cp clk off ack bit\n",
	       __func__);
	apmu_debug = __raw_readb(APMU_DEBUG);
	printk(KERN_INFO "%s: APMU_DEBUG before %02x\n", __func__, apmu_debug);
	apmu_debug &= ~(APMU_DEBUG_CP_HALT | APMU_DEBUG_CP_CLK_OFF_ACK);
	__raw_writeb(apmu_debug, APMU_DEBUG);
	printk(KERN_INFO "%s: APMU_DEBUG after %02x\n", __func__,
	       __raw_readb(APMU_DEBUG));

	writel(arbel_bin_phys_addr, CIU_SW_BRANCH_ADDR);

	printk(KERN_INFO "release cp...\n");
	value = __raw_readl(mpmu_base_addr + MPMU_APRR);
	printk(KERN_INFO "the PMUM_APRR value is 0x%08x\n",
	       (unsigned int)(mpmu_base_addr + MPMU_APRR));
	printk(KERN_INFO "the value is 0x%08x\n", value);
	/* set CPR(bit[0]) bit of APRR register low */
	value &= ~MPMU_APRR_CPR;
	__raw_writel(value, mpmu_base_addr + MPMU_APRR);
	/*
	 * avoid endless loop
	 * TODO: accurate the timeout time
	 */
	while (((__raw_readl(mpmu_base_addr + MPMU_APRR)
		 & MPMU_APRR_CPR) != 0x0) && timeout)
		timeout--;
	if (!timeout) {
		printk(KERN_INFO "fail!\n");
		/*
		 * release CP failed, invoke panic() here???
		 * mask CP_CLK_OFF_ACK and CP_HALT again to avoid hang in FC
		 */
		printk(KERN_INFO
		       "%s: mask cp halt and cp clk off ack bit again\n",
		       __func__);
		apmu_debug = __raw_readb(APMU_DEBUG);
		printk(KERN_INFO "%s: APMU_DEBUG before %02x\n",
		       __func__, apmu_debug);
		apmu_debug |= (APMU_DEBUG_CP_HALT | APMU_DEBUG_CP_CLK_OFF_ACK);
		__raw_writeb(apmu_debug, APMU_DEBUG);
		printk(KERN_INFO "%s: APMU_DEBUG after %02x\n", __func__,
		       __raw_readb(APMU_DEBUG));
	} else
		printk(KERN_INFO "done!\n");

	release_fc_mutex();
	printk(KERN_INFO "%s: release fc mutex\n", __func__);
}
EXPORT_SYMBOL(cp_releasecp);

/**
 * CP hold procedure
 * 1. acquire fc mutex
 * 2. stop watchdog
 * 3. set bit CPR of register APRR to hold CP in reset
 * 4. set bits DSPR, BBR and DSRAMINT of register CPRR to hold MSA in reset
 * 5. wait several us
 * 6. mask bits CP_CLK_OFF_ACK, CP_HALT of register APMU_DEBUG to
 *    disable ACK from CP
 * 7. release fc mutex
 */
void cp_holdcp(void)
{
	int value;
	u8 apmu_debug;

	acquire_fc_mutex();
	printk(KERN_INFO "%s: acquire fc mutex\n", __func__);

	if (watchdog_count_stop_fp)
		watchdog_count_stop_fp();

	printk(KERN_INFO "hold cp...");
	/* set CPR(bit[0]) bit of APRR register high */
	value = __raw_readl(mpmu_base_addr + MPMU_APRR) | MPMU_APRR_CPR;
	__raw_writel(value, mpmu_base_addr + MPMU_APRR);
	printk(KERN_INFO "done!\n");
	printk(KERN_INFO "hold msa...");
	value = __raw_readl(mpmu_base_addr + MPMU_CPRR);
	/* set DSPR(bit[2]), BBR(bit[3]) and DSRAMINT(bit[5])
	 * bits of CPRR register high */
	value |= MPMU_CPRR_DSPR | MPMU_CPRR_BBR | MPMU_CPRR_DSRAMINT;
	__raw_writel(value, mpmu_base_addr + MPMU_CPRR);
	printk(KERN_INFO "done!\n");
	/*
	 * recommended software wait several us after hold CP/MSA
	 * in reset status and then start load the new image
	 * TODO: accurate the wait time
	 */
	udelay(100);

	/*
	 * mask CP_CLK_OFF_ACK and CP_HALT
	 */
	printk(KERN_INFO "%s: mask cp halt and cp clk off ack bit\n",
	       __func__);
	apmu_debug = __raw_readb(APMU_DEBUG);
	printk(KERN_INFO "%s: APMU_DEBUG before %02x\n", __func__, apmu_debug);
	apmu_debug |= (APMU_DEBUG_CP_HALT | APMU_DEBUG_CP_CLK_OFF_ACK);
	__raw_writeb(apmu_debug, APMU_DEBUG);
	printk(KERN_INFO "%s: APMU_DEBUG after %02x\n", __func__,
	       __raw_readb(APMU_DEBUG));

	release_fc_mutex();
	printk(KERN_INFO "%s: release fc mutex\n", __func__);
}
EXPORT_SYMBOL(cp_holdcp);

bool cp_get_status(void)
{
	return !(readl(mpmu_base_addr + MPMU_APRR) & MPMU_APRR_CPR);
}
EXPORT_SYMBOL(cp_get_status);
